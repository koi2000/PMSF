// SPDX-License-Identifier: Apache-2.0
//
// @file ValenceStreamCodec.h
// @brief 5 元 valence 流的 RC 编码（**多算法共用的那一份**）。
//
// 由来（LoC 重构计划 P3，2026-09-23）：`encodeValence5` 原先在 **5 个算法**里
// 各写了一遍（MMSE / RD / PCLTTM / PCATM / VQMMC），四份与 MMSE 版的**代码行**
// 差 20–53 行，差异都在**实现形态**（`unique_ptr<char[]>`+`get()` vs
// `vector<char>`+`data()`、`memcpy` 定长写入 vs `insert`），输出字节应当一致。
//
// 本文件是**规范形**（取 MMSE 版：`memcpy` 定长写入最直白）。
// 抽取遵守计划 §0 的防凑数规则 3：该函数**已被 ≥2 算法引用**（实际 5 个）。
//
// **判据（计划 §4 P3）**：切换后必须
//   `scripts/tests/p3_probe_hashes.sh {MMSE,RD,PCLTTM,PCATM,VQMMC}` 全部逐字节相等。
#ifndef PMSF_COMMON_VALENCE_STREAM_CODEC_H
#define PMSF_COMMON_VALENCE_STREAM_CODEC_H

#include "RD/compress/RDSymbolCollectOperator.h"  // RDValenceCode（暂不改类名，见计划 §3）
#include "compress/RangeCoderStreamEncoder.h"     // pmsf::encodeContextAdaptiveUint8Queue

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <vector>

namespace pmsf {

/// @brief 5 元 valence 流的 RC 编码：`[i32 numPoly][i32 numOther][RC bytes]`。
///
/// @param valCodes 每个 polyFace 的 valence 码（5 元；`VAL_OTHER` 走 escape）
/// @param otherVals `VAL_OTHER` 对应的 inline uint8 真实 valence
/// @return 与各算法原实现**逐字节相同**的载荷
inline std::vector<uint8_t> encodeValence5(const std::deque<uint8_t>& valCodes,
                                           const std::deque<uint8_t>& otherVals) {
    constexpr int kBufScratch = BUFFER_SIZE / 12;
    const size_t bufSize = std::max<size_t>(kBufScratch, valCodes.size() * 2 + 1024);
    std::unique_ptr<char[]> buf(new char[bufSize]);
    auto deriveCtx = [](int prev) {
        if (prev < 0)
            return 0;
        if (prev == (int)RDValenceCode::VAL_3 || prev == (int)RDValenceCode::VAL_4)
            return 1;
        if (prev == (int)RDValenceCode::VAL_5 || prev == (int)RDValenceCode::VAL_6)
            return 2;
        return 3;
    };

    size_t rcSize = pmsf::encodeContextAdaptiveUint8Queue(valCodes,
                                                          8,    // 5 元 padding 到 8
                                                          4,    // 4 contexts
                                                          deriveCtx,
                                                          &otherVals,
                                                          256,
                                                          (int)RDValenceCode::VAL_OTHER,
                                                          buf.get(),
                                                          bufSize);

    std::vector<uint8_t> out(8 + rcSize);
    int32_t numPoly = (int32_t)valCodes.size();
    int32_t numOther = (int32_t)otherVals.size();
    std::memcpy(out.data(), &numPoly, 4);
    std::memcpy(out.data() + 4, &numOther, 4);
    std::memcpy(out.data() + 8, buf.get(), rcSize);
    return out;
}

}  // namespace pmsf

#endif  // PMSF_COMMON_VALENCE_STREAM_CODEC_H
