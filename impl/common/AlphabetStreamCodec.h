// SPDX-License-Identifier: Apache-2.0
//
// @file AlphabetStreamCodec.h
// @brief 「小字母表逐符号流」的 RC 编码（**多算法共用的那一份**）。
//
// 由来（LoC 重构计划 P11，2026-09-24）：valence 族 codec 的侧流（isPoly /
// fusedFaceSym / color / mode / edgeSym）全都是**同一个形状** ——
// 「每个符号 1 个小 alphabet 值，走一个 qsmodel 的算术编码，前面挂 4 字节
// 符号个数」。各算法各写了一遍，只有**实现形态**不同
// （`unique_ptr<char[]>`+`get()` vs `vector<char>`+`data()`、
// `memcpy` 定长写入 vs `insert`），输出字节一致。
//
// 本文件是**规范形**（取 MMSE 版：`memcpy` 定长写入最直白）。
// 抽取遵守计划 §0 的防凑数规则 3。**当前 3 个真实消费者**，都是各自独立的
// 实现副本（非同一次抽取的残留）：
//   - `impl/MMSE/compress/MMSEDualRingSymbolCollectOperator.cpp`（alphabet=2，无注入缝）
//   - `impl/VQMMC/compress/VQMMCCodebookSymbolCollectOperator.cpp`（alphabet=2，无注入缝）
//   - `impl/PCLTTM/compress/PCLTTMTagSymbolCollectOperator.cpp`（alphabet=4，
//     `fusedFaceSym` 流；**有**注入缝，故传流名）—— P12 加入，见计划 §73
//
// **判据**：切换后 `bench_columnar` + `bench_binary` 的 MMSE / VQMMC 两行
// `compressed_size_bytes` 必须与冻结基线**逐字节相同**（P11 §67 实测）。
//
// 与 `RDTagCanonicalCodec.h` 的分工：那边是 `const char* streamName` 门控的
// **注入缝**版本（RD/PCATM/PCLTTM 用），这边是**无注入缝**的朴素版本。
// 刻意不合并：注入缝是行为差异，不是实现形态差异。
#ifndef PMSF_COMMON_ALPHABET_STREAM_CODEC_H
#define PMSF_COMMON_ALPHABET_STREAM_CODEC_H

#include "compress/RangeCoderStreamEncoder.h"  // pmsf::encodeUint8Queue
#include "rangecoder/qsmodel.h"                // initqsmodel / qsgetsym
#include "rangecoder/rangecod.h"               // start_decoding / decode_culshift
#include "entropy/StreamEntropyCoder.h"        // pmsf::injectedStreamCoder（注入缝）
#include "Global.h"                            // BUFFER_SIZE

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <deque>
#include <memory>
#include <vector>

namespace pmsf {

/// @brief 小字母表逐符号流的 RC 编码：`[i32 numSymbols][RC bytes]`。
///
/// @param syms       每个符号一个值，取值范围 `[0, alphabet)`
/// @param alphabet   字母表大小（2 = 二值流，4 = 四值流，…）
/// @param streamName 非空则**先查按名注入的流编码器**（Phase S2 缝）；
///                   空 / nullptr ⇒ 直接走公共核。
/// @return 与各算法原实现**逐字节相同**的载荷
///
/// ⚠️ **只有真正拥有注入缝的算法才传名字**（与 `RDTagCanonicalCodec.h:94` 同一纪律）：
/// `injectedStreamCoder` 是**按名字全局查表**，给没有缝的算法传名字会把该算法
/// **静默变成可注入的**——默认路径字节虽不变，语义面却变了。当前三个消费者中：
///   - `MMSE` / `VQMMC` → 传 nullptr（从来没有缝）
///   - `PCLTTM`         → 传 "fusedFaceSym"（本来就有缝）
///
/// 缓冲区大小只影响「够不够编码」，**不影响输出字节**（`encodeUint8Queue`
/// 返回的是实际编码长度），故各消费者原用的不同公式在此统一为一式。
inline std::vector<uint8_t> encodeAlphabetStream(const std::deque<uint8_t>& syms,
                                                 int alphabet,
                                                 const char* streamName = nullptr) {
    if (streamName && *streamName) {
        if (auto injected = injectedStreamCoder(streamName)) {
            std::vector<std::uint32_t> flat(syms.begin(), syms.end());
            std::vector<std::uint8_t> blob;
            injected->reset();
            injected->encode(flat, blob);
            std::vector<uint8_t> out(4 + blob.size());
            const int32_t n = static_cast<int32_t>(syms.size());
            std::memcpy(out.data(), &n, 4);
            if (!blob.empty())
                std::memcpy(out.data() + 4, blob.data(), blob.size());
            return out;
        }
    }
    const size_t bufSize = std::max<size_t>(syms.size() / 4 + 1024, 4096);
    std::unique_ptr<char[]> buf(new char[bufSize]);
    const size_t rcSize = encodeUint8Queue(syms, alphabet, buf.get(), bufSize);

    std::vector<uint8_t> out(4 + rcSize);
    const int32_t numSymbols = static_cast<int32_t>(syms.size());
    std::memcpy(out.data(), &numSymbols, 4);
    std::memcpy(out.data() + 4, buf.get(), rcSize);
    return out;
}

/// @brief `encodeAlphabetStream` 的**解码镜像**：`[i32 numSymbols][RC bytes]` → 符号。
///
/// ⚠️ **本函数尚未接线、也尚未做 bit-exact 对拍**（2026-09-25 加入，P16 T3.2）。
/// 它按 `AdaptiveSymbolEncoder` 的参数逐项镜像：
///   alphabet / scaleBits=10 / initCount=1<<9 / `initqsmodel(..., compress=0)`
/// —— 最后一项是关键：编码端传 `compress=1`、解码端必须传 `0`，
/// 二者在 qsmodel.c 里走不同的初始化分支。
///
/// 接线前必须先对拍：用 `encodeAlphabetStream(syms, A, nullptr)` 产出载荷，
/// 本函数解回的序列应与 `syms` **逐符号相同**。
/// 参照实现：`impl/MMSE/decompress/MMSEDualRingSymbolReadOperator.cpp:71`
/// （同一套 decode_culshift/qsgetsym/decode_update/qsupdate 调用次序，每日回环验证）。
///
/// @param payload      指向 `[i32 numSymbols][RC bytes]` 的首字节
/// @param payloadBytes 载荷总字节数（含 4 字节头）
/// @param alphabet    字母表大小，**必须与编码时一致**
/// @return 解码出的符号；输入非法时返回空 vector
inline std::vector<uint8_t> decodeAlphabetStream(const uint8_t* payload,
                                                 std::size_t payloadBytes,
                                                 int alphabet) {
    if (payload == nullptr || payloadBytes < 4 || alphabet <= 0) return {};
    std::int32_t numSymbols = 0;
    std::memcpy(&numSymbols, payload, 4);
    if (numSymbols <= 0) return {};

    std::vector<char> rcbuf(reinterpret_cast<const char*>(payload) + 4,
                            reinterpret_cast<const char*>(payload) + payloadBytes);
    std::size_t offset = 0;
    rangecoder rc;
    rc.p_data = rcbuf.data();
    rc.p_dataOffset = &offset;
    if (start_decoding(&rc) != 0) return {};

    qsmodel model;
    initqsmodel(&model, alphabet, 10, 1 << 9, nullptr, 0);  // 解码端：compress=0

    std::vector<uint8_t> out(static_cast<std::size_t>(numSymbols));
    for (std::size_t i = 0; i < out.size(); ++i) {
        const int lv = decode_culshift(&rc, 10);
        const int sym = qsgetsym(&model, lv);
        int syfreq = 0, ltfreq = 0;
        qsgetfreq(&model, sym, &syfreq, &ltfreq);
        decode_update(&rc, syfreq, ltfreq, 1 << 10);
        qsupdate(&model, sym);
        out[i] = static_cast<uint8_t>(sym);
    }
    deleteqsmodel(&model);
    return out;
}

}  // namespace pmsf

#endif  // PMSF_COMMON_ALPHABET_STREAM_CODEC_H
