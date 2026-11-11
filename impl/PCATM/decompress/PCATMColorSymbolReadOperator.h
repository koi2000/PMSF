/*
 * @file        PCATMColorSymbolReadOperator.h
 * @brief       PCATM 论文 2-color decoder（CP-3.3 β+1 落地版）。
 * @author      koi
 * @date        2026-05-10
 *
 * 与 collector `PCATMColorSymbolCollectOperator` 字节级对称：
 *   - 输入：Columnar _symbols.parquet 的 rc_blob (BlobLayout=14, sv=3)。
 *   - 解 ConnectivityBlobContainer v3 拿 5 个子流：
 *       * stream 0 (valence)  → 4-context RC + V_OTHER 256-元 escape
 *           → 拆回 (rcDecodedValCodes_, rcDecodedOtherVals_)
 *       * stream 1 (isPoly)   → 2-元 RC → rcDecodedIsPoly_
 *       * stream 2 (edgeSym)  → 2-元 RC → rcDecodedEdgeSyms_
 *       * stream 3 (color)    → 2-元 RC → rcDecodedColors_ (PCATMColor 私有)
 *       * stream 4 (residual) → bit-plane RC → rcDecodedResiduals_
 *   - 5 个父类 protected 队列灌好后，**直接 reuse** 父类
 *     `RDGateOnlySymbolReadOperator::collectFromDecoded` —— 这是与 collector
 *     BFS 100% 对称的关键不变量；color 流目前不参与对角线推断（保留作为
 *     PCATM 论文协议忠实留位 + 未来 β+2 优化路径输入）。
 *
 * sv 兼容：
 *   - 当前只支持 sv=3（CP-3.3 β+1 落地版）；sv=1/sv=2 已被 deprecated, 抛 runtime_error。
 *
 * Quantum 不变量：
 *   - encoder 要求 enablePrediction && enableQuantization → decoder 同样校验。
 */

#ifndef PMSF_DECOMPRESS_PCATM_COLOR_SYMBOL_READ_OPERATOR_H
#define PMSF_DECOMPRESS_PCATM_COLOR_SYMBOL_READ_OPERATOR_H

#include "common/patch/RDGateOnlySymbolReadOperator.h"
#include <cstdint>
#include <deque>

class PCATMColorSymbolReadOperator : public RDGateOnlySymbolReadOperator {
  public:
    PCATMColorSymbolReadOperator() = default;

    /// @brief 解析 BlobLayout=14 sv=3 的 ConnectivityBlobContainer v3 5 段流。
    ///        填充父类 4 个 protected rcDecodedXxx_ 队列 + 自己的 rcDecodedColors_，
    ///        由父类 `collectFromDecoded` 复用 BFS / mesh reconstruct 路径。
    void importFromRangeCodedBuffer(char* buffer,
                                    size_t& dataOffset,
                                    unsigned nbQuantBits) override;

    // collectFromDecoded 不重写：直接用父类 RDGateOnlySymbolReadOperator 的版本
    // （v0-fan 对角线反演 + isPoly conquest）。

  protected:
    /// @brief PCATM color stream（per BFS face 1 bit, RED=0/BLUE=1）。
    /// 当前 CP-3.3 β+1 阶段不参与对角线推断，仅保留作 protocol 字段。
    std::deque<uint8_t> rcDecodedColors_;
};

#endif  // PMSF_DECOMPRESS_PCATM_COLOR_SYMBOL_READ_OPERATOR_H
