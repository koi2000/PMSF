/*
 * @Author: koi
 * @Date: 2025-08-28 21:51:18
 * @Description:
 */

#include "BasicIsRemovableOperator.h"
#include "Facet.h"
#include "Halfedge.h"
#include "MeshUtils.h"
#include "common/FastRingWalk.h"
#include "common/FastRingWalk.h"
#include <cstdlib>
#include <cstring>

namespace {
bool envOn(const char* name) {
    const char* v = std::getenv(name);
    return v && *v && std::strcmp(v, "0") != 0;
}
}

bool BasicIsRemovableOperator::isRemovable(HalfedgeErrorEntry& entry) {
    MCGAL::Halfedge* halfedge = entry.edge;
    // 见 UnifiedManifoldIsRemovableOperator 同名处。
    const bool allowAdded = policy_.allowAddedHalfedges;
    if (halfedge->vertex()->isCollapsed() || halfedge->end_vertex()->isCollapsed() || halfedge->vertex()->isRemoved() ||
        halfedge->end_vertex()->isRemoved()) {
        return false;
    }
    if (!halfedge->end_vertex()->isRemovable()) {
        return false;
    }
    // if (!halfedge->b_collapseOk) {
    //     return true;
    // }
    if (halfedge->face()->isRemoved()) {
        return false;
    }
    // if (halfedge->end_vertex()->vertex_degree() <= 3 || halfedge->end_vertex()->vertex_degree() >= 8 || halfedge->isRemoved()) {
    //     return false;
    // }
    // perf（2026-11-09 QEM 实测：本算子占 14.75% cycles）：两个 1-ring 扫描只做
    // 「任一违例则拒绝」的 OR 判定，与环顺序无关，因此先用 O(1) 环走；
    // 异常环（Broken）回退原 MCGAL 顶点迭代器路径，行为不变。
    auto ringPredicate = [&](MCGAL::Halfedge* h) -> bool {
        if (h->opposite() == nullptr) return false;
        if (h->end_vertex()->isCollapsed() || (!allowAdded && h->isAdded())) return false;
        return true;
    };
    using pmsf::RingWalkResult;
    for (MCGAL::Vertex* ringVertex : {halfedge->end_vertex(), halfedge->vertex()}) {
        const RingWalkResult r = pmsf::walkOutgoingRing(ringVertex, ringPredicate);
        if (r == RingWalkResult::EarlyStop) return false;
        if (r == RingWalkResult::Broken) {
            for (MCGAL::Halfedge* h : *ringVertex) {
                if (!ringPredicate(h)) return false;
            }
        }
    }
    return true;
}
