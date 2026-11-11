// SPDX-License-Identifier: Apache-2.0
//
// RDTagSymbolReadOperator.h — RD Tag (Phase 4 PR-B, BlobLayout=18, sv=1)
// decoder 端。
//
// 协议（sv=1, 3 段流 ConnectivityBlobContainer v3, magic='CONN'）：
//   stream 0 (valence):    5-元 4-context RC + V_OTHER inline escape
//   stream 1 (isPolyBit):  2-元 RC，per BFS face 1 bit
//   stream 2 (residual):   bit-plane RC（含 Phase 3 Step 1 二阶 curvature predictor）
//   stream 3 (sv=5 only):  QEM adjustment bitstream + old survivor point residual
//
// 与 PCLTTMTagSymbolReadOperator (BlobLayout=16, sv=3) 的差异（最小化）：
//   - 数 stream 从 3 → 3（一样），blob_layout 18 vs 16，sv 1 vs 3。
//   - stream 1 alphabet：2-元 isPolyBit (本 RD Tag) vs 4-元 fusedFaceSym (PCLTTM)。
//   - decoder 将 isPolyBit 直接 push 到 `rcDecodedIsPoly_`；**不填**
//     `rcDecodedVertexTags_`（保持空）。后续 collectFromDecoded 完全复用
//     父类 PCLTTMTag 的 BFS + v0-fan 对角线反演路径——它在 vertexTags 队列
//     为空时 tagSym 默认 0（Untagged），只影响 trace 输出，不影响几何重建。
//
// 关键依赖（与 PCLTTM Tag sv=3 共享）：MCGAL Phase 2 新增的
//   `Facet::reset_proxy_first(vs)` + `ContextPool::allocateFaceFromPoolProxyFirst`，
//   让 ColumnarDeserialize 重建 base_mesh 时把 proxyHalfedge 设为 vs[0]→vs[1]，
//   对齐 encoder Columnar 序列化时 faceVids[0] = encoder.proxyHalfedge.vertex 的语义。
//
// 详见 docs/paper_alignment/PHASE4_PLAN.md §3.3 (PR-B)。

#ifndef PMSF_DECOMPRESS_RD_TAG_SYMBOL_READ_OPERATOR_H
#define PMSF_DECOMPRESS_RD_TAG_SYMBOL_READ_OPERATOR_H

#include "PCLTTM/decompress/PCLTTMTagSymbolReadOperator.h"

#include <cstdint>
#include <deque>
#include <vector>

namespace MCGAL {
class Facet;
class Halfedge;
class PointInt;
class Vertex;
}  // namespace MCGAL

class RDTagSymbolReadOperator : public PCLTTMTagSymbolReadOperator {
  public:
    RDTagSymbolReadOperator() = default;

    /// @brief 解析 BlobLayout=18 sv=1 的 ConnectivityBlobContainer v3 3 段流。
    ///        填充 rcDecodedValCodes_ / rcDecodedOtherVals_ / rcDecodedIsPoly_
    ///        / rcDecodedResiduals_。**不填** rcDecodedVertexTags_（保持空）。
    void importFromRangeCodedBuffer(char* buffer,
                                    size_t& dataOffset,
                                    unsigned nbQuantBits) override;

    // 先复用父类 PCLTTMTag 的 v0-fan 对角线反演 + removed-vertex residual
    // 还原；若 sv=5 携带 QEM adjustment 流，再按同一 seed BFS 恢复 survivor
    // 的 old position，供 BasicReconstructOperator 在拆点前 setPoint(vertexPoint)。
    void collectFromDecoded(MCGAL::Halfedge* seed,
                            std::vector<MCGAL::Vertex*>& vertices,
                            std::vector<MCGAL::Halfedge*>& halfedge,
                            std::vector<MCGAL::Facet*>& facets) override;

  protected:
    std::deque<uint8_t> rcDecodedVertexAdjustSymbols_;
    std::deque<MCGAL::PointInt> rcDecodedVertexAdjustResiduals_;
    std::vector<int> vertexAdjustBitsPerRound_;
    bool useEdgeSymTopology_ = false;
};

#endif  // PMSF_DECOMPRESS_RD_TAG_SYMBOL_READ_OPERATOR_H
