// SPDX-License-Identifier: Apache-2.0
//
// RDTagEliminateOperator.cpp — see header for design contract.
//
// 与 PCLTTMTagEliminateOperator.cpp 的差异：
//   - 删去 step 4 (tag evolution via applyTagEvolution)；其余 step 1-3, 5 保留。
//   - 不依赖 PCLTTMTagTable.h / Vertex::TagFlag。

#include "RDTagEliminateOperator.h"

#include "RD/RDPGS.h"

#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "common/Define.h"

bool RDTagEliminateOperator::eliminate(HalfedgeErrorEntry& entry) {
    MCGAL::Halfedge*& h = entry.edge;
    if (h == nullptr) return false;

    // 1) adjustment 逻辑（同父类）
    if (header_.isEnableAdjustment()) {
        h->vertex()->setAdjusted();
        h->vertex()->setVertexPoint(h->vertex()->point());
        h->vertex()->setPoint(entry.p);
    }

    // 2) remove_point —— polyFace 现在是 N-gon polygon
    // task-19 P1：PGS 模式下先记录被消顶点的量化位置 key（供 P2 selection
    // 顺序投影），再捕获后续 tag 演化所需的上下文。
    std::uint64_t pgsRemovedKey = 0;
    if (rd_impl::pgsEnabled() && h->end_vertex() != nullptr) {
        pgsRemovedKey =
            rd_impl::pgsPosKey(mesh_->floatPosToInt(h->end_vertex()->point()));
    }
    CHECK_TRUE(VertexRemovalEliminateOperator::remove_point(h));

    // 3) Phase 2 Step 3：在 polygon 状态下 normalize polyFace.proxyHalfedge
    //    到 vertex **量化整数坐标**字典序最小的 outgoing halfedge —— encoder/
    //    decoder 共用规则，bit-exact 对齐。
    MCGAL::Facet* polyFace = h->face();
    if (polyFace) {
        polyFace->normalize_proxy_to_min_position_vertex(
            mesh_->bboxMin, mesh_->f_quantStep);
        // 标记协议（PR-A 加的字段；当前不读，但 PR-D 可能用作 BPV diag）。
        polyFace->setProtocolHint(18);  // RD_TAG
    }

    // 4) [默认无 tag evolution] —— RD 协议不依赖 PCLTTM ⊕/⊖ tag 机制。
    //    task-19 P1：PMSF_RD_PGS=1 时启用真 tag 演化 + conquered 标记 +
    //    消除序号（论文 §3.2 prioritized gate selection 的输入）。默认 env 关闭
    //    ⇒ 该块不执行，产物与历史逐字节一致。
    if (rd_impl::pgsEnabled() && polyFace != nullptr) {
        const std::vector<MCGAL::Vertex*> polyVerts = polyFace->getVertices();
        rd_impl::pgsPostRemove(polyVerts, polyFace, pgsRemovedKey);
    }

    // 5) triangulate 用 normalize 后的 proxyHalfedge 作起点
    MCGAL::Halfedge* triH = polyFace ? polyFace->proxyHalfedge() : h;
    CHECK_TRUE(VertexRemovalEliminateOperator::triangulate(triH));
    CHECK_TRUE(VertexRemovalEliminateOperator::postprocess(triH));

    entry.edge = (polyFace ? polyFace->proxyHalfedge() : h);
    return true;
}
