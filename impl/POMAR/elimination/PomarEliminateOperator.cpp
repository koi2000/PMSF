/*
 * @Author: koi
 * @Date: 2025-08-27 22:37:03
 * @Description:
 */
#include "PomarEliminateOperator.h"
#include "common/BoundedRingTraversal.h"
#include "common/DebugTool.h"
#include "Facet.h"
#include "Global.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "MeshUtils.h"
#include "Point.h"
#include "Vertex.h"
#include "common/Define.h"
#include <cassert>
#include <cstddef>
#include <vector>

void PomarEliminateOperator::init(std::shared_ptr<MCGAL::Mesh> mesh, PMSFHeader header,bool compress_boundary) {
    mesh_ = mesh;
    compress_boundary_ = compress_boundary;
    header_ = header;
}

bool PomarEliminateOperator::eliminate(HalfedgeErrorEntry& entry) {
    performCollapse(entry);
    return true;
}

bool PomarEliminateOperator::remove_point(MCGAL::Halfedge* h) {
    return true;
}

bool PomarEliminateOperator::triangulate(MCGAL::Halfedge* h) {
    return true;
}

bool PomarEliminateOperator::postprocess(MCGAL::Halfedge* h) {
    return true;
}

MCGAL::Halfedge* PomarEliminateOperator::encode_boundary(MCGAL::Halfedge* h) {
    return nullptr;
};

void PomarEliminateOperator::performCollapse(HalfedgeErrorEntry& entry) {
    MCGAL::Halfedge* heh = entry.edge;
    MCGAL::Halfedge* hehOpp = heh->opposite();
    MCGAL::Vertex* v2 = entry.customData;
    MCGAL::Vertex* vh_to = heh->end_vertex();
    MCGAL::Vertex* vh_from = heh->vertex();

    // Find which of the remaining edge will be the best predictor.
    MCGAL::PerformedOperation op;
    assert(!v2->isRemoved());

    std::vector<MCGAL::Vertex*> vrNeighborsBefore;
    vrNeighborsBefore.reserve(vh_from ? vh_from->halfedges().size() : 0);
    for (MCGAL::Halfedge* hit : pmsf::boundedRing(vh_from)) {
        vrNeighborsBefore.push_back(hit->end_vertex());
    }

    op.vhPivotL = heh->next()->end_vertex();
    op.vhPivotR = hehOpp->next()->end_vertex();

    op.v1 = vh_to;
    op.v2 = v2;
    op.vr = vh_from;

    MCGAL::contextPool.pushOperation(op.v1->poolId(), op);

    mesh_->new_collapse(heh);
    assert(!vh_to->isRemoved());

    // ── PHASE64：`opposite()` 空指针保护（修 dragon 上的段错误） ──────────────
    // 实测（附 83/84）：dragon（开边界网格）上 POMAR 在 `voh_it->opposite()`
    // 处 SIGSEGV。MCGAL 用 `opposite_ == nullptr` 表示**边界半边**，而
    // `pmsf::boundedRing` 的步进函数本身就显式处理 null（BoundedRingIterator
    // 里 `opp ? opp->next() : nullptr`）⇒ 环里出现空 opposite 是**正常情形**，
    // 不是损坏状态。此处只是**漏防**。
    //
    // 语义：`b_collapseOk` 是**逐半边**的缓存标志，标记一条边时两个方向各标一次；
    // 边界边只有一个方向存在 ⇒ 只标存在的那一侧，另一侧无物可标。
    //
    // ⚠️ 字节中性：bunny/horse 上该路径从未触发 ⇒ 对已通过的门禁格零影响。
    for (MCGAL::Vertex* v : vrNeighborsBefore) {
        for (MCGAL::Halfedge* voh_it : pmsf::boundedRing(v)) {
            voh_it->b_collapseOk = false;
            if (MCGAL::Halfedge* const opp = voh_it->opposite()) {
                opp->b_collapseOk = false;
            }
        }
    }

    for (MCGAL::Vertex* it : vrNeighborsBefore) {
        for (MCGAL::Halfedge* vih_it : pmsf::boundedRing(it)) {
            MCGAL::Halfedge* const opp = vih_it->opposite();
            if (opp != nullptr && opp->b_collapseOk) {
                opp->b_collapseOk = mesh_->new_is_collapse_ok(opp);
            }
        }
    }
}
