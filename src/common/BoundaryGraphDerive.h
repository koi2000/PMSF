#pragma once

#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "MeshUtils.h"
#include "Vertex.h"
#include "common/BoundedRingTraversal.h"
#include "common/Graph.h"

#include <algorithm>
#include <limits>

namespace pmsf {

/// Derive the legacy (start, direction, end) boundary triple from a start-only
/// boundary graph node.  The boundary segment is identified by the start vertex
/// plus the ordered cluster pair (groupA -> groupB); direction and end are
/// recovered by walking the boundary halfedges, so the legacy table need not be
/// stored for scheme StartOnly.
inline bool deriveBoundaryTriple(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                 int groupA,
                                 int groupB,
                                 int startVertexPoolId,
                                 Node& out) {
    if (!mesh || startVertexPoolId < 0) return false;
    MCGAL::Vertex* start = nullptr;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (v && !v->isRemoved() && v->poolId() == startVertexPoolId) {
            start = v;
            break;
        }
    }
    if (!start) return false;

    auto isPairHalfedge = [&](MCGAL::Halfedge* h) -> bool {
        if (!h || h->isRemoved() || !h->opposite() || !h->face() ||
            !h->opposite()->face() || h->face()->isRemoved() ||
            h->opposite()->face()->isRemoved()) {
            return false;
        }
        const int fa = h->face()->groupId();
        const int fb = h->opposite()->face()->groupId();
        return (fa == groupA && fb == groupB) || (fa == groupB && fb == groupA);
    };

    MCGAL::Halfedge* startHalfedge = nullptr;
    for (MCGAL::Halfedge* h : pmsf::boundedRing(start)) {
        if (!h || h->isRemoved() || !h->opposite()) continue;
        if (!h->face() || !h->opposite()->face()) continue;
        if (h->face()->groupId() != groupA ||
            h->opposite()->face()->groupId() != groupB) {
            continue;
        }
        if (!startHalfedge || h->poolId() < startHalfedge->poolId()) {
            startHalfedge = h;
        }
    }
    if (!startHalfedge) return false;

    out.st = startVertexPoolId;
    out.ed = startHalfedge->end_vertex() ? startHalfedge->end_vertex()->poolId() : -1;
    out.stop = out.ed;
    MCGAL::Halfedge* h = startHalfedge;
    int guard = 0;
    const int maxSteps = static_cast<int>(static_cast<size_t>(mesh->size_of_facets()) * 3) + 8;
    while (h && guard++ < maxSteps) {
        MCGAL::Halfedge* nxt = MCGAL::next_boundary(groupA, groupB, h);
        if (!nxt || nxt == startHalfedge) break;
        MCGAL::Vertex* endVertex = nxt->end_vertex();
        if (endVertex) out.stop = endVertex->poolId();
        h = nxt;
    }
    return true;
}

}  // namespace pmsf
