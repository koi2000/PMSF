// SPDX-License-Identifier: Apache-2.0
//
// RDCanonicalReconstructOperator.h — task-43 route A 的**解码端重建器**。
//
// 每轮取走一层符号记录，在**当前网格**（progressive decode 的 base→fine 状态）
// 上按与编码端**同一份** gate FIFO / 符号序重放：
//   valence ⇒ merge fan 区域回 polygon → 插中心点 → 反 Frenet → fan 三角化
//   null    ⇒ 只做 flag/sign 传播与前沿扩张
// 解码实体是 RDCanonicalTraversal.h 的 decodeLayer（与 encodeLayer 同源）。
#ifndef PMSF_IMPL_RD_RECONSTRUCT_RD_CANONICAL_RECONSTRUCT_OPERATOR_H
#define PMSF_IMPL_RD_RECONSTRUCT_RD_CANONICAL_RECONSTRUCT_OPERATOR_H

#include "RD/decompress/RDCanonicalSymbolReadOperator.h"
#include "RD/RDCanonicalTraversal.h"

#include "reconstruct/ReconstructOpertator.h"

#include <memory>
#include <vector>

namespace rd_canon {

class RDCanonicalReconstructOperator final : public ReconstructOperator {
  public:
    explicit RDCanonicalReconstructOperator(
        std::shared_ptr<RDCanonicalSymbolReadOperator> reader)
        : reader_(std::move(reader)) {}

    void reconstruct(MCGAL::Halfedge* halfedge) override;
    void reconstruct(std::vector<MCGAL::Vertex*>& vertices,
                     std::vector<MCGAL::Halfedge*>& halfedge,
                     std::vector<MCGAL::Facet*>& facets) override;

    bool failed() const {
        return failed_;
    }
    int totalInserted() const {
        return totalInserted_;
    }
    int roundsProcessed() const {
        return roundIdx_;
    }

  private:
    QuantGrid gridFromMesh() const;
    MCGAL::Halfedge* locateSeed(const RDCanonicalSymbolReadOperator::LayerRecord& layer);

    std::shared_ptr<RDCanonicalSymbolReadOperator> reader_;
    State st_;
    int roundIdx_ = 0;
    int totalInserted_ = 0;
    bool failed_ = false;
};

}  // namespace rd_canon

#endif  // PMSF_IMPL_RD_RECONSTRUCT_RD_CANONICAL_RECONSTRUCT_OPERATOR_H
