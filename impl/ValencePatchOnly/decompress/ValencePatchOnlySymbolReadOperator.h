/*
 * @file        ValencePatchOnlySymbolReadOperator.h
 * @brief       Phase 1 · Step 1.2 — 方案 V (Valence-Patch-Only) decoder.
 * @author      koi
 * @date        2026-05-10
 *
 * 见 docs/CONNECTIVITY_BITSTREAM_IMPLEMENTATION_PLAN.md §1.2。
 *
 * 与 collector `ValencePatchOnlySymbolCollectOperator` 完全字节对称：
 *   - 输入：Columnar _symbols.parquet 的 rc_blob（BlobLayout=9, sv=1）。
 *   - 解 ConnectivityBlobContainer v3 拿 3 个子流：
 *       * stream 0 (valence6) → 4-context RC + V_OTHER 256-元 escape
 *           → 拆回 (rcDecodedIsPoly_, rcDecodedValCodes_, rcDecodedOtherVals_)。
 *       * stream 1 (edgeSym)  → 2-元 RC → rcDecodedEdgeSyms_。
 *       * stream 2 (residual) → bit-plane RC → rcDecodedResiduals_。
 *   - 5 个 protected 解码队列灌好后，**直接 reuse** 父类
 *     `RDGateOnlySymbolReadOperator::collectFromDecoded` —— 这是
 *     与 collector BFS 100% 对称的关键不变量 R2。
 *
 * sv 兼容：
 *   - 当前只支持 sv=1（Phase 1 首发）；其他 sv 抛 runtime_error。
 *   - 旧 RDGateOnly v3（BlobLayout=8）的 byte stream **不**进这里；
 *     ColumnarDeserializeOperator 用 layout 区分。
 *
 * Quantum 不变量：
 *   - encoder 要求 enablePrediction && enableQuantization → decoder 同样校验
 *     metadata 标志。
 */

#ifndef PMSF_DECOMPRESS_VALENCE_PATCH_ONLY_SYMBOL_READ_OPERATOR_H
#define PMSF_DECOMPRESS_VALENCE_PATCH_ONLY_SYMBOL_READ_OPERATOR_H

#include "common/patch/RDGateOnlySymbolReadOperator.h"

class ValencePatchOnlySymbolReadOperator : public RDGateOnlySymbolReadOperator {
  public:
    ValencePatchOnlySymbolReadOperator() = default;

    /// @brief 解析 BlobLayout=9 sv=1 的 ConnectivityBlobContainer v3 三段流。
    ///        填充父类 protected 的 5 个 rcDecodedXxx_ 队列后，由父类
    ///        `collectFromDecoded` 复用 BFS / mesh reconstruct 路径。
    void importFromRangeCodedBuffer(char* buffer,
                                    size_t& dataOffset,
                                    unsigned nbQuantBits) override;

    // collectFromDecoded 不重写：直接用父类 RDGateOnlySymbolReadOperator 的版本。
};

#endif  // PMSF_DECOMPRESS_VALENCE_PATCH_ONLY_SYMBOL_READ_OPERATOR_H
