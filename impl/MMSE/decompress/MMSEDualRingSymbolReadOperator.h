// SPDX-License-Identifier: Apache-2.0
//
// MMSEDualRingSymbolReadOperator.h — MMSE dual-ring protocol decoder
// (PR-L4b, BlobLayout=22).
//
// Accepts BlobLayout=22.
//   sv=1: legacy 3-stream RDTag-compatible layout.
//   sv=3: adaptive RDTag-compatible residual （**门禁路径**，P8/D1 sv 日落保留项）.
//
// P11（2026-09-24）：sv=2（mode 流）/ sv=4（fixed dual-ring）/ sv=6,7,8（global
// streams）的解码实现已删除 —— 它们**已证不可达**（§66.3 插桩实测，含阳性对照）。
// 编码端自 P8 起就不再发射这些 sv，解码端的谓词恒 false。

#ifndef PMSF_DECOMPRESS_MMSE_DUAL_RING_SYMBOL_READ_OPERATOR_H
#define PMSF_DECOMPRESS_MMSE_DUAL_RING_SYMBOL_READ_OPERATOR_H

#include "RD/decompress/RDTagSymbolReadOperator.h"

#include <cstdint>
#include <deque>
#include <vector>

class MMSEDualRingSymbolReadOperator : public RDTagSymbolReadOperator {
  public:
    MMSEDualRingSymbolReadOperator() = default;

    void importFromRangeCodedBuffer(char* buffer,
                                    size_t& dataOffset,
                                    unsigned nbQuantBits) override;

    void collectFromDecoded(MCGAL::Halfedge* seed,
                            std::vector<MCGAL::Vertex*>& vertices,
                            std::vector<MCGAL::Halfedge*>& halfedge,
                            std::vector<MCGAL::Facet*>& facets) override;

  private:
    /// 监控行专用（`PMSF_BPV_DIAG` 打印 `totalModes=`）。sv=2 的 mode 流已随
    /// sv 日落删除，本队列恒空 —— 但**不删**：硬约束 2 禁止删监控行。
    std::deque<std::uint8_t> rcDecodedModes_;

    // ------------------------------------------------------------------
    // P1 (2026-10-22, WS-B4): MMSE paper predictor stack replay.
    //
    // The encoder writes a self-describing per-cluster 2-bit mode stream inside
    // the residual stream (flag 0x20000000 on the numCoords word).  Import
    // parses it once for all rounds; collectFromDecoded installs the current
    // round's mode list into pmsf::mmse_pred::replayState() so that the
    // predictor hook (src/predict/RDPredictor.h) returns the matching
    // correction for every polyFace in BFS order.
    // ------------------------------------------------------------------
    std::vector<std::vector<std::uint8_t>> rcDecodedPredModeQueues_;
    std::vector<std::int32_t> rcDecodedPredClusterSizes_;
    bool rcPredModeStreamPresent_ = false;
};

#endif  // PMSF_DECOMPRESS_MMSE_DUAL_RING_SYMBOL_READ_OPERATOR_H
