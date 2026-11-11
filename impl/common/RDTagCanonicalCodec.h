// SPDX-License-Identifier: Apache-2.0
//
// RDTagCanonicalCodec.h — shared RD_TAG sv=1 encoders for the canonical path.
//
// These helpers mirror the exact entropy coding used by
// RDTagSymbolCollectOperator so that a canonical payload can be reassembled
// into a byte-identical RD_TAG ConnectivityBlobContainer v3 blob.

#ifndef PMSF_COMPRESS_RD_TAG_CANONICAL_CODEC_H
#define PMSF_COMPRESS_RD_TAG_CANONICAL_CODEC_H

#include "Point.h"
#include "RD/compress/RDSymbolCollectOperator.h"
#include "entropy/StreamEntropyCoder.h"
#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <span>
#include <stdexcept>
#include <vector>

namespace pmsf {
namespace rd_tag {

inline std::vector<std::uint8_t> encodeValence5WithContext(
    const std::deque<std::uint8_t>& valCodes,
    const std::deque<std::uint8_t>& otherVals) {
    const std::size_t bufSize =
        std::max<std::size_t>(valCodes.size() * 2 + 1024, 4096);
    std::unique_ptr<char[]> buf(new char[bufSize]);
    std::size_t off = 0;
    rangecoder rc;
    rc.p_data = buf.get();
    rc.p_dataOffset = &off;
    start_encoding(&rc, 0, 0);

    constexpr int alphabet_size = 8;
    constexpr int lg_totf = 10;
    constexpr int rescale = 1 << 9;
    constexpr int kNumCtx = 4;

    qsmodel models[kNumCtx];
    for (int i = 0; i < kNumCtx; ++i) {
        initqsmodel(&models[i], alphabet_size, lg_totf, rescale, nullptr, 1);
    }
    qsmodel escapeModel;
    initqsmodel(&escapeModel, 256, 10, 1 << 9, nullptr, 1);

    auto deriveCtx = [](int prev) {
        if (prev < 0) return 0;
        if (prev == (int)RDValenceCode::VAL_3 ||
            prev == (int)RDValenceCode::VAL_4)
            return 1;
        if (prev == (int)RDValenceCode::VAL_5 ||
            prev == (int)RDValenceCode::VAL_6)
            return 2;
        return 3;
    };

    int prev = -1;
    std::size_t escCursor = 0;
    for (std::uint8_t sym : valCodes) {
        int ctx = deriveCtx(prev);
        int syfreq = 0, ltfreq = 0;
        qsgetfreq(&models[ctx], (int)sym, &syfreq, &ltfreq);
        encode_shift(&rc, syfreq, ltfreq, lg_totf);
        qsupdate(&models[ctx], (int)sym);

        if (sym == (std::uint8_t)RDValenceCode::VAL_OTHER) {
            std::uint8_t esc =
                (escCursor < otherVals.size()) ? otherVals[escCursor++] : 0;
            int esyfreq = 0, eltfreq = 0;
            qsgetfreq(&escapeModel, esc, &esyfreq, &eltfreq);
            encode_shift(&rc, esyfreq, eltfreq, 10);
            qsupdate(&escapeModel, esc);
        }
        prev = (int)sym;
    }
    std::size_t rcSize = done_encoding(&rc);
    for (int i = 0; i < kNumCtx; ++i) deleteqsmodel(&models[i]);
    deleteqsmodel(&escapeModel);

    std::vector<std::uint8_t> out(8 + rcSize);
    std::int32_t numPoly = static_cast<std::int32_t>(valCodes.size());
    std::int32_t numOther = static_cast<std::int32_t>(otherVals.size());
    std::memcpy(out.data(), &numPoly, 4);
    std::memcpy(out.data() + 4, &numOther, 4);
    std::memcpy(out.data() + 8, buf.get(), rcSize);
    return out;
}

/// @param streamName 非空则**先查按名注入的流编码器**（Phase S2 缝）；
///        空 / nullptr ⇒ 直接走公共核。
///
/// ⚠️ 注入是**按名字全局查表**（`pmsf::injectedStreamCoder`）。故调用方必须
/// **显式传自己那条流的名字**：只有真正拥有注入缝的算法才传名字，其余一律传空
/// —— 否则会把"从来没有注入缝"的算法**静默变成可注入的**（默认路径字节虽不变，
/// 语义面却变了）。见计划 §20.2。
inline std::vector<std::uint8_t> encodeBitstream2RC(
    const std::deque<std::uint8_t>& bits,
    const char* streamName = nullptr) {
    if (streamName && *streamName) {
        if (auto injected = pmsf::injectedStreamCoder(streamName)) {
            std::vector<std::uint32_t> syms(bits.begin(), bits.end());
            std::vector<std::uint8_t> blob;
            injected->reset();
            injected->encode(syms, blob);
            std::vector<std::uint8_t> out(4 + blob.size());
            std::int32_t numBits = static_cast<std::int32_t>(bits.size());
            std::memcpy(out.data(), &numBits, 4);
            if (!blob.empty())
                std::memcpy(out.data() + 4, blob.data(), blob.size());
            return out;
        }
    }
    const std::size_t bufSize =
        std::max<std::size_t>(bits.size() / 4 + 1024, 4096);
    std::unique_ptr<char[]> buf(new char[bufSize]);
    std::size_t off = 0;
    rangecoder rc;
    rc.p_data = buf.get();
    rc.p_dataOffset = &off;
    start_encoding(&rc, 0, 0);

    qsmodel m;
    initqsmodel(&m, 2, 10, 1 << 9, nullptr, 1);
    for (std::uint8_t bit : bits) {
        int sym = bit ? 1 : 0;
        int syfreq = 0, ltfreq = 0;
        qsgetfreq(&m, sym, &syfreq, &ltfreq);
        encode_shift(&rc, syfreq, ltfreq, 10);
        qsupdate(&m, sym);
    }
    std::size_t rcSize = done_encoding(&rc);
    deleteqsmodel(&m);

    std::vector<std::uint8_t> out(4 + rcSize);
    std::int32_t numBits = static_cast<std::int32_t>(bits.size());
    std::memcpy(out.data(), &numBits, 4);
    std::memcpy(out.data() + 4, buf.get(), rcSize);
    return out;
}

/// @param streamName 非空则**先查按名注入的流编码器**（Phase S2 缝）；
///        空 / nullptr ⇒ 直接走公共核。语义同 `encodeBitstream2RC`（见其 ⚠️）。
///
/// 缓冲统一取 `max(BUFFER_SIZE/3, size*3+1024)`：原三份局部实现里两份就是这个公式，
/// 第三份（PCATM）用较小的 `BUFFER_SIZE/3`。放大 scratch **不改输出字节**
/// （`out` 只按实际 `rcSize` 分配），只是消除"小缓冲可能溢出"的隐患。
inline std::vector<std::uint8_t> encodeResidualBitplane(
    const std::deque<MCGAL::PointInt>& residuals, int BP_BITS,
    const char* streamName = nullptr) {
    if (streamName && *streamName) {
        if (auto injected = pmsf::injectedStreamCoder(streamName)) {
            // 符号展平约定（与 rd 批 §13.2 相同）：按 residual 顺序、分量序 x,y,z，
            // 每分量 zigzag 有符号→无符号，共 3N 个 uint32（= numCoords）。
            std::vector<std::uint32_t> syms;
            syms.reserve(residuals.size() * 3);
            auto zigzag = [](std::int32_t v) -> std::uint32_t {
                return (static_cast<std::uint32_t>(v) << 1) ^
                       static_cast<std::uint32_t>(v >> 31);
            };
            for (const auto& r : residuals) {
                syms.push_back(zigzag(static_cast<std::int32_t>(r[0])));
                syms.push_back(zigzag(static_cast<std::int32_t>(r[1])));
                syms.push_back(zigzag(static_cast<std::int32_t>(r[2])));
            }
            std::vector<std::uint8_t> blob;
            injected->reset();
            injected->encode(syms, blob);
            std::vector<std::uint8_t> out(4 + blob.size());
            std::int32_t numCoords = static_cast<std::int32_t>(residuals.size() * 3);
            std::memcpy(out.data(), &numCoords, 4);
            if (!blob.empty())
                std::memcpy(out.data() + 4, blob.data(), blob.size());
            return out;
        }
    }
    const std::size_t bufSize =
        std::max<std::size_t>(BUFFER_SIZE / 3, residuals.size() * 3 + 1024);
    std::unique_ptr<char[]> buf(new char[bufSize]);
    std::size_t off = 0;
    rangecoder rc;
    rc.p_data = buf.get();
    rc.p_dataOffset = &off;
    start_encoding(&rc, 0, 0);

    std::vector<qsmodel> bpFirst1(BP_BITS), bpAfter1(BP_BITS);
    for (int b = 0; b < BP_BITS; ++b) {
        initqsmodel(&bpFirst1[b], 2, 10, 1 << 9, nullptr, 1);
        initqsmodel(&bpAfter1[b], 2, 10, 1 << 9, nullptr, 1);
    }

    auto encResidualBP = [&](std::int32_t r) {
        std::uint32_t z = (static_cast<std::uint32_t>(r) << 1) ^
                          static_cast<std::uint32_t>(r >> 31);
        bool seenOne = false;
        for (int b = BP_BITS - 1; b >= 0; --b) {
            int bit = static_cast<int>((z >> b) & 1u);
            qsmodel* mp = seenOne ? &bpAfter1[b] : &bpFirst1[b];
            int syfreq = 0, ltfreq = 0;
            qsgetfreq(mp, bit, &syfreq, &ltfreq);
            encode_shift(&rc, syfreq, ltfreq, 10);
            qsupdate(mp, bit);
            if (bit) seenOne = true;
        }
    };

    for (const auto& res : residuals) {
        encResidualBP(res.x());
        encResidualBP(res.y());
        encResidualBP(res.z());
    }
    std::size_t rcSize = done_encoding(&rc);
    for (int b = 0; b < BP_BITS; ++b) {
        deleteqsmodel(&bpFirst1[b]);
        deleteqsmodel(&bpAfter1[b]);
    }

    std::vector<std::uint8_t> out(4 + rcSize);
    std::int32_t numCoords = static_cast<std::int32_t>(residuals.size() * 3);
    std::memcpy(out.data(), &numCoords, 4);
    std::memcpy(out.data() + 4, buf.get(), rcSize);
    return out;
}

// ===========================================================================
// RD_TAG sv=6 edgeSym 的**上下文**编解码器（编码/解码成对，必须逐字互逆）
// ===========================================================================
//
// 背景（PHASE61 物品2 一手实测，bunny/30 轮/sv=6）：
//   edgeSym 是 3 bit/face 的朴素二元流，1{,}221{,}612 bit 中仅 123{,}540 个 1
//   （10.1%）。`encodeBitstream2RC` 用**单个**自适应模型逐 bit 编码 ⇒ 0.447
//   bit/symbol。但实测该流的 3 bit **窗口内强相关**：
//     - polyFace 窗口（33{,}906 个）只有 0 或 1 个 1：8{,}650 / 25{,}256，**从无 2 个**；
//     - 非 polyFace 窗口（373{,}298 个）从无 3 个 1（111 计数 = 0）。
//   即 8 元联合分布退化：poly 侧只有 4 个有效符号、非 poly 侧 7 个。用
//   (isPoly, 窗口内已出现的 1 的个数) 做上下文，联合熵下界从 1.418 bit/窗口
//   （独立假设）降到 poly 1.978 / 非 poly 1.070 bit/窗口 ⇒ 理想码长约
//   58.1 KB，相对当前的 68.3 KB 省 ~10 KB。
//
// 语义不变式：**符号序列不动**——仍是 3 bit/face，解码端拿到的队列逐元素相同；
// 只换承载它的熵编码。故 BasicReconstruct / RDGateOnly 消费端零改动。
//
// ⚠️ Phase S2 缝保持：旧实现里 edgeSym **也**走 `encodeBitstream2RC(q, "isPoly")`
// —— 即按 "isPoly" 这个名字查注入 coder（`static_huffman_v1` 等方案会命中），
// 解码端 `decodeBitstream2RC` 同样硬查 "isPoly"。为**逐字节保留**该（既有的、
// 名字错配的）语义，上下文编码器只在注入**未**生效时接管；注入生效时两侧
// 一律回落旧路径。谓词必须与解码端同源 —— 两处都调本函数。
inline bool edgeSymInjectionActive() {
    return static_cast<bool>(pmsf::injectedStreamCoder("isPoly"));
}

// 上下文 = (isPoly ? 3 : 0) + min(onesSoFar, 2) ∈ [0,6)，6 个 2 元自适应模型。
// 刻意**不**跳过"结构性必为 0"的 bit：模型自适应后这些 bit 的代价趋近 0
// （qsmodel rescale 下限），而"跳过"会把不变量绑死在"poly 窗口 ≤1 个 1"这条
// 实测经验上 —— 换网格/换轮数若出现反例就会静默错位。
inline std::vector<std::uint8_t> encodeEdgeSymWithContext(
    const std::deque<std::uint8_t>& isPolyBits,
    const std::deque<std::uint8_t>& edgeBits) {
    constexpr int lg_totf = 10;
    constexpr int rescale = 1 << 9;
    constexpr int kNumCtx = 6;

    const std::size_t bufSize =
        std::max<std::size_t>(edgeBits.size() / 4 + 1024, 4096);
    std::unique_ptr<char[]> buf(new char[bufSize]);
    std::size_t off = 0;
    rangecoder rc;
    rc.p_data = buf.get();
    rc.p_dataOffset = &off;
    start_encoding(&rc, 0, 0);

    qsmodel models[kNumCtx];
    for (int i = 0; i < kNumCtx; ++i) {
        initqsmodel(&models[i], 2, lg_totf, rescale, nullptr, 1);
    }

    // ⚠️ 以**扁平 bit 下标**驱动，而不是嵌套 f/k：解码端只知道 numBits，
    //   用同一个 idx → (f = idx/3, k = idx%3) 映射就能逐 bit 复现上下文，
    //   即使两个队列的长度不成 3 倍关系也不会错位（不变量不依赖尺寸假设）。
    int onesSoFar = 0;
    for (std::size_t idx = 0; idx < edgeBits.size(); ++idx) {
        const int bit = edgeBits[idx] ? 1 : 0;
        if (idx % 3 == 0) onesSoFar = 0;
        const std::size_t f = idx / 3;
        const bool isPoly = (f < isPolyBits.size()) && (isPolyBits[f] != 0);
        const int ctx = (isPoly ? 3 : 0) + std::min(onesSoFar, 2);
        int syfreq = 0, ltfreq = 0;
        qsgetfreq(&models[ctx], bit, &syfreq, &ltfreq);
        encode_shift(&rc, syfreq, ltfreq, lg_totf);
        qsupdate(&models[ctx], bit);
        if (bit) ++onesSoFar;
    }
    std::size_t rcSize = done_encoding(&rc);
    for (int i = 0; i < kNumCtx; ++i) deleteqsmodel(&models[i]);

    std::vector<std::uint8_t> out(4 + rcSize);
    std::int32_t numBits = static_cast<std::int32_t>(edgeBits.size());
    std::memcpy(out.data(), &numBits, 4);
    std::memcpy(out.data() + 4, buf.get(), rcSize);
    return out;
}

/// `encodeEdgeSymWithContext` 的严格逆：按同一 (isPoly, onesSoFar) 上下文
/// 解出 `numBits`（= 流头）个 bit，追加到 `outQueue`。
///
/// @param isPolyAll      解码端累计的 isPoly 队列（`std::deque`，非连续）
/// @param isPolyTailStart 本轮 isPoly 在 `isPolyAll` 中的起始下标
///        （即 `isPolyAll.size() - 本轮面数`）
inline void decodeEdgeSymWithContext(const std::span<const std::uint8_t>& sl,
                                     const std::deque<std::uint8_t>& isPolyAll,
                                     std::size_t isPolyTailStart,
                                     std::deque<std::uint8_t>& outQueue) {
    constexpr int lg_totf = 10;
    constexpr int rescale = 1 << 9;
    constexpr int kNumCtx = 6;

    if (sl.size() < 4) {
        throw std::runtime_error("[RDTag decoder] edgeSym header too small");
    }
    std::int32_t numBits = 0;
    std::memcpy(&numBits, sl.data(), 4);
    if (numBits <= 0) return;

    std::vector<char> rcStaging(sl.begin() + 4, sl.end());
    std::size_t rcOffset = 0;
    rangecoder rc;
    rc.p_data = rcStaging.data();
    rc.p_dataOffset = &rcOffset;
    if (start_decoding(&rc) != 0) {
        throw std::runtime_error("[RDTag decoder] edgeSym start_decoding failed");
    }

    qsmodel models[kNumCtx];
    for (int i = 0; i < kNumCtx; ++i) {
        initqsmodel(&models[i], 2, lg_totf, rescale, nullptr, 0);
    }

    // 与编码端逐 bit 同构：同一 idx → (f = idx/3, k = idx%3) 映射。
    // 长度由流头 numBits 决定（不假设 = 3×面数），故两队列长度不匹配也不会错位。
    int onesSoFar = 0;
    const std::size_t nIsPoly = isPolyAll.size();
    for (std::int32_t idx = 0; idx < numBits; ++idx) {
        if (idx % 3 == 0) onesSoFar = 0;
        const std::size_t f = static_cast<std::size_t>(idx / 3);
        const bool isPoly =
            (isPolyTailStart + f < nIsPoly) && (isPolyAll[isPolyTailStart + f] != 0);
        const int ctx = (isPoly ? 3 : 0) + std::min(onesSoFar, 2);
        qsmodel* mp = &models[ctx];
        int syfreq = 0, ltfreq = 0;
        int ltval = decode_culshift(&rc, lg_totf);
        int sym = qsgetsym(mp, ltval);
        qsgetfreq(mp, sym, &syfreq, &ltfreq);
        decode_update(&rc, syfreq, ltfreq, 1 << lg_totf);
        qsupdate(mp, sym);
        outQueue.push_back(static_cast<std::uint8_t>(sym ? 1 : 0));
        if (sym) ++onesSoFar;
    }
    done_decoding(&rc);
    for (int i = 0; i < kNumCtx; ++i) deleteqsmodel(&models[i]);
}

}  // namespace rd_tag
}  // namespace pmsf

#endif  // PMSF_COMPRESS_RD_TAG_CANONICAL_CODEC_H
