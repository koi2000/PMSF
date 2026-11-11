/*
 * @file BFSTraversal.cpp
 * @brief Deterministic BFS driver implementation.
 *
 * The three pass helpers (facet / edge / vertex) reproduce exactly the loops
 * that live inside BasicSymbolCollectOperator::collect today, so that the
 * Phase 9 migration of existing collectors to use them stays byte-identical
 * against the frozen baseline. Any change to gate-queue ordering, version
 * handling, or neighbour enqueue order in this file must be paired with a
 * baseline re-freeze.
 */
#include "BFSTraversal.h"

#include "Global.h"  // MCGAL::contextPool

namespace pmsf {

int runBFS(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Halfedge* seed, BFSVisitor& visitor) {
    GateQueue q;
    q.push(seed->poolId());
    const int version = MCGAL::BFSVersionManager::current_version++;
    while (!q.empty()) {
        const int hid = q.front();
        q.pop();
        MCGAL::Halfedge* h =
            MCGAL::contextPool.getHalfedgeByIndexInSubPool(mesh->meshId(), hid);
        if (h->isVisited(version) || h->isRemoved()) {
            continue;
        }
        visitor.onHalfedge(h, version);
        h->setVisited(version);
        if (!h->face()->isVisited(version)) {
            MCGAL::Facet* f = h->face();
            f->setVisited(version);
            visitor.onFacetFirstVisit(f, version);
        }
        if (!h->vertex()->isVisited(version)) {
            h->vertex()->setVisited(version);
            visitor.onVertexFirstVisit(h->vertex(), version);
        }
        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp != nullptr && !hOpp->isVisited(version)) {
                q.push(hOpp->poolId());
            }
            if (hIt != nullptr && !hIt->isVisited(version)) {
                q.push(hIt->poolId());
            }
            hIt = hIt->next();
        } while (hIt != h);
    }
    return version;
}

// ---------------------------------------------------------------------------
// Canonical three-pass helpers. These are intentionally NOT expressed in
// terms of runBFS() above, because the legacy loops in
// BasicSymbolCollectOperator::collect interleave the "mark visited" and
// "emit symbol" steps slightly differently per pass; preserving that
// interleaving keeps the bitstream bit-identical with the frozen baseline.
// Once Phase 13 retires every legacy byte-parity baseline, the three helpers
// can be collapsed into a single BFSVisitor call. Until then, keep them.
// ---------------------------------------------------------------------------

void runFacetPass(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Halfedge* seed,
                  std::function<void(MCGAL::Facet*)> onFacet) {
    GateQueue q;
    q.push(seed->poolId());
    const int version = MCGAL::BFSVersionManager::current_version++;
    while (!q.empty()) {
        const int hid = q.front();
        q.pop();
        MCGAL::Halfedge* h =
            MCGAL::contextPool.getHalfedgeByIndexInSubPool(mesh->meshId(), hid);
        if (h->isVisited(version) || h->isRemoved()) {
            continue;
        }
        h->setVisited(version);
        if (!h->face()->isVisited(version)) {
            MCGAL::Facet* f = h->face();
            f->setVisited(version);
            onFacet(f);
        }
        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp != nullptr && !hOpp->isVisited(version)) {
                q.push(hOpp->poolId());
            }
            if (hIt != nullptr && !hIt->isVisited(version)) {
                q.push(hIt->poolId());
            }
            hIt = hIt->next();
        } while (hIt != h);
    }
}

void runEdgePass(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Halfedge* seed,
                 std::function<void(MCGAL::Halfedge*)> onHalfedge) {
    GateQueue q;
    q.push(seed->poolId());
    const int version = MCGAL::BFSVersionManager::current_version++;
    while (!q.empty()) {
        const int hid = q.front();
        q.pop();
        MCGAL::Halfedge* h =
            MCGAL::contextPool.getHalfedgeByIndexInSubPool(mesh->meshId(), hid);
        if (h->isVisited(version) || h->isRemoved()) {
            continue;
        }
        h->setVisited(version);
        onHalfedge(h);
        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp != nullptr && !hOpp->isVisited(version)) {
                q.push(hOpp->poolId());
            }
            if (hIt != nullptr && !hIt->isVisited(version)) {
                q.push(hIt->poolId());
            }
            hIt = hIt->next();
        } while (hIt != h);
    }
}

void runVertexPass(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Halfedge* seed,
                   std::function<void(MCGAL::Vertex*)> onVertex) {
    GateQueue q;
    q.push(seed->poolId());
    const int version = MCGAL::BFSVersionManager::current_version++;
    while (!q.empty()) {
        const int hid = q.front();
        q.pop();
        MCGAL::Halfedge* h =
            MCGAL::contextPool.getHalfedgeByIndexInSubPool(mesh->meshId(), hid);
        if (h->isVisited(version) || h->isRemoved()) {
            continue;
        }
        h->setVisited(version);
        if (!h->vertex()->isVisited(version)) {
            h->vertex()->setVisited(version);
            onVertex(h->vertex());
        }
        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp != nullptr && !hOpp->isVisited(version)) {
                q.push(hOpp->poolId());
            }
            if (hIt != nullptr && !hIt->isVisited(version)) {
                q.push(hIt->poolId());
            }
            hIt = hIt->next();
        } while (hIt != h);
    }
}

}  // namespace pmsf
