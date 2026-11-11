// SPDX-License-Identifier: Apache-2.0
//
// RDCQuestReconstructOperator.h — task-19：`rd_cq` 解码端重建器。
//
// 解码实体仍是逐层验证过的 `pmsf::decodeLayer`（见父类
// `ConquestReconstructOperator`）；本类只加一件事：**每层开始时清空 H3 的
// "同层已插入中心"集合**（H3 coarse-graph 对称契约的一部分 —— 解码端预测时
// 必须把本层已插回的中心从图中排除，剩下的才是该层 coarse 图）。
#ifndef PMSF_RD_RECONSTRUCT_RD_CQUEST_RECONSTRUCT_OPERATOR_H
#define PMSF_RD_RECONSTRUCT_RD_CQUEST_RECONSTRUCT_OPERATOR_H

#include "Conquest/decompress/ConquestSymbolReadOperator.h"
#include "Conquest/reconstruct/ConquestReconstructOperator.h"
#include "RD/hooks/RDCQuestHooks.h"

#include <memory>
#include <vector>

namespace pmsf {

class RDCQuestReconstructOperator final : public ConquestReconstructOperator {
  public:
    RDCQuestReconstructOperator(std::shared_ptr<ConquestSymbolReadOperator> reader,
                                std::shared_ptr<rd_cq::RDCQuestGeomState> st)
        : ConquestReconstructOperator(std::move(reader)), st_(std::move(st)) {}

    void reconstruct(MCGAL::Halfedge* halfedge) override {
        if (st_) st_->beginDecoderLayer();
        ConquestReconstructOperator::reconstruct(halfedge);
    }
    void reconstruct(std::vector<MCGAL::Vertex*>& vertices,
                     std::vector<MCGAL::Halfedge*>& halfedge,
                     std::vector<MCGAL::Facet*>& facets) override {
        if (st_) st_->beginDecoderLayer();
        ConquestReconstructOperator::reconstruct(vertices, halfedge, facets);
    }

  private:
    std::shared_ptr<rd_cq::RDCQuestGeomState> st_;
};

}  // namespace pmsf

#endif  // PMSF_RD_RECONSTRUCT_RD_CQUEST_RECONSTRUCT_OPERATOR_H
