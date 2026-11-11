// SPDX-License-Identifier: Apache-2.0
//
// PCLTTMTagEliminateOperator.h — PCLTTM Tag (path-B, BlobLayout=16, sv=1)
// encoder-side eliminate operator.
//
// 设计契约（与 PCLTTMTagReconstructOperator 对称）：
//   1. 默认几何继承 VertexRemovalEliminateOperator —— v0-fan，保护基线。
//      `PMSF_PCLTTM_USE_TAG_TABLE_TRIANGULATION=1` 时使用共享 tag-table
//      descriptor 切 polygon，作为向论文 Figure 9 几何收敛的显式路径。
//   2. override eliminate(entry)：
//        - 调父类 remove_point(h) 以 erase center vertex 并产生 polyFace；
//        - **在 triangulate(h) 之前** 读 polyFace->getVertices() 取 N 个 ring
//          vertex（这是 encoder 唯一能"看见 polygon vertex 顺序"的窗口）；
//        - 用 PCLTTMTagTable::evolve(N, tagLeft, tagRight, V0First) 演化 tag；
//        - 把 (vertex.poolId, newTag) 序列追加到内部 trace（PR-2 测试用）；
//        - 默认调父类 triangulate(h) + postprocess(h) 完成几何 fan
//          triangulation；显式 tag-table 模式下按 PCLTTMTagTable descriptor
//          写入对角线。
//
// 对称性源头：
//   - encoder 端 polyVerts[0] = h->face()->proxyHalfedge()->vertex()；
//   - decoder 端 polyVerts[0] 同样由 face->proxyHalfedge() 决定，因双方
//     共享同一个 ConnectivityBlobContainer 容器顺序，poolId 一一对应。
//
// 详见 docs/paper_alignment/PCLTTM_OFFICIAL_REPRO_PROMPT.md PR-2 §R5。

#pragma once

#include "Facet.h"
#include "elimination/VertexRemovalEliminateOperator.h"
#include "Vertex.h"
#include "common/PCLTTMTagTable.h"
#include "PCLTTM/PCLTTMFrenetPatch.h"

#include <cstdint>
#include <vector>

class PCLTTMTagEliminateOperator : public VertexRemovalEliminateOperator {
  public:
    explicit PCLTTMTagEliminateOperator(std::shared_ptr<pmsf::pclttm_frenet::PendingPatches> pending = {})
        : frenetPatches_(std::move(pending)) {}
    ~PCLTTMTagEliminateOperator() = default;

    bool eliminate(HalfedgeErrorEntry& entry) override;

    /// @brief 取走本轮 tag 演化 trace（test 用）。take 语义：取走并清空。
    std::vector<pmsf::pclttm::TagEvolutionRecord> takeRoundTagEvolutions() {
        std::vector<pmsf::pclttm::TagEvolutionRecord> out;
        out.swap(roundTagEvolutions_);
        return out;
    }

    void resetRoundAccumulators() override {
        VertexRemovalEliminateOperator::resetRoundAccumulators();
        roundTagEvolutions_.clear();
        if (frenetPatches_) frenetPatches_->clear();
    }

  protected:
    MCGAL::Vector3i computeRemovedVertexResidualInt(MCGAL::Halfedge* entry,
        MCGAL::Halfedge* patch, const MCGAL::Point& point, const MCGAL::PointInt& target) override;

  private:
    std::shared_ptr<pmsf::pclttm_frenet::PendingPatches> frenetPatches_;
    bool triangulateWithTagTable(MCGAL::Facet* polyFace,
                                 const std::vector<MCGAL::Vertex*>& polyVerts);

    std::vector<pmsf::pclttm::TagEvolutionRecord> roundTagEvolutions_;
};
