// SPDX-License-Identifier: Apache-2.0
//
// PCLTTMTagReconstructOperator.h — PCLTTM Tag (path-B, BlobLayout=16, sv=1)
// decoder-side reconstruct operator.
//
// 设计契约（与 PCLTTMTagEliminateOperator 对称）：
//   1. 几何完全继承 BasicReconstructOperator —— v0-fan 反演，调
//      create_center_vertex_without_init，零变化。
//   2. override reconstruct(vertices, halfedges, facets)：
//        - 第一阶段：vertices->setPoint(vertexPoint)，halfedges -> merge_face；
//          这部分与父类完全一致（几何 join）。
//        - 第二阶段：对 facets 列表的每个 polyFace f：
//            * **在 insert_point(f) 之前** 读 f->getVertices() 取 N 个 ring vertex
//              （此时 f 是 N-gon polyFace，尚未 split-fan）；
//            * 用同一份 PCLTTMTagTable::evolve(N, tagLeft, tagRight, V0First) 演化；
//            * 调父类 insert_point(f) 完成 v0-fan 几何反演；
//            * 把 (vertex.poolId, newTag) 序列追加到内部 trace（PR-2 测试用）。
//
// 对称性证明：
//   encoder 端 polyVerts 取自 h->face()->getVertices()（在 remove_point 后、
//   triangulate 前），decoder 端 polyVerts 取自 f->getVertices()（在 merge_face
//   后、insert_point 前）。两端的 polyFace 拓扑结构由 ConnectivityBlobContainer
//   保证逐 byte 同构，因此 polyVerts 顺序一致 → tag 演化结果逐 vertex 一致。
//
// 详见 docs/paper_alignment/PCLTTM_OFFICIAL_REPRO_PROMPT.md PR-2 §R5。

#pragma once

#include "reconstruct/BasicReconstructOperator.h"
#include "PCLTTM/elimination/PCLTTMTagEliminateOperator.h"   // for TagEvolutionRecord

#include <vector>

class PCLTTMTagSymbolReadOperator;

class PCLTTMTagReconstructOperator : public BasicReconstructOperator {
  public:
    explicit PCLTTMTagReconstructOperator(std::shared_ptr<PCLTTMTagSymbolReadOperator> reader = {})
        : reader_(std::move(reader)) {}
    ~PCLTTMTagReconstructOperator() = default;

    void reconstruct(std::vector<MCGAL::Vertex*>& vertices,
                     std::vector<MCGAL::Halfedge*>& halfedges,
                     std::vector<MCGAL::Facet*>& facets) override;

    // 与 encoder 同名 API，trace 比对用。
    std::vector<pmsf::pclttm::TagEvolutionRecord> takeRoundTagEvolutions() {
        std::vector<pmsf::pclttm::TagEvolutionRecord> out;
        out.swap(roundTagEvolutions_);
        return out;
    }

  private:
    std::shared_ptr<PCLTTMTagSymbolReadOperator> reader_;
    std::vector<pmsf::pclttm::TagEvolutionRecord> roundTagEvolutions_;
};
