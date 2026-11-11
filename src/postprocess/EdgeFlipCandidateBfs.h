#pragma once

#include "Facet.h"
#include "ContextPool.h"
#include "Global.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Vertex.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace pmsf::edgeflip {

inline std::array<long long, 3> bfsVertexKey(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                             MCGAL::Vertex* v) {
    if (!v) return {0, 0, 0};
    if (mesh && mesh->f_quantStep > 0.0f) {
        MCGAL::PointInt q = mesh->floatPosToInt(v->point());
        return {static_cast<long long>(q[0]), static_cast<long long>(q[1]),
                static_cast<long long>(q[2])};
    }
    constexpr double scale = 100000000.0;
    return {static_cast<long long>(std::llround(static_cast<double>(v->x()) * scale)),
            static_cast<long long>(std::llround(static_cast<double>(v->y()) * scale)),
            static_cast<long long>(std::llround(static_cast<double>(v->z()) * scale))};
}

inline bool bfsTrianglePairFlippable(MCGAL::Halfedge* h) {
    if (!h || h->isRemoved()) return false;
    MCGAL::Halfedge* ho = h->opposite();
    if (!ho || ho->isRemoved()) return false;
    MCGAL::Facet* f0 = h->face();
    MCGAL::Facet* f1 = ho->face();
    if (!f0 || !f1 || f0->isRemoved() || f1->isRemoved()) return false;
    if (f0->facet_degree() != 3 || f1->facet_degree() != 3) return false;
    MCGAL::Vertex* a = h->vertex();
    MCGAL::Vertex* b = h->end_vertex();
    MCGAL::Vertex* c = h->next() ? h->next()->next() ? h->next()->end_vertex() : nullptr : nullptr;
    MCGAL::Vertex* d = ho->next() ? ho->next()->next() ? ho->next()->end_vertex() : nullptr : nullptr;
    if (!a || !b || !c || !d || a->isRemoved() || b->isRemoved() || c->isRemoved() ||
        d->isRemoved()) {
        return false;
    }
    return a != b && a != c && a != d && b != c && b != d && c != d;
}

inline std::uint64_t bfsEdgeKey(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                MCGAL::Halfedge* h) {
    if (!h || !h->opposite()) return 0;
    auto a = bfsVertexKey(mesh, h->vertex());
    auto b = bfsVertexKey(mesh, h->end_vertex());
    if (b < a) std::swap(a, b);
    std::uint64_t hash = 1469598103934665603ULL;
    for (long long v : a) {
        hash ^= static_cast<std::uint64_t>(v);
        hash *= 1099511628211ULL;
    }
    for (long long v : b) {
        hash ^= static_cast<std::uint64_t>(v);
        hash *= 1099511628211ULL;
    }
    return hash;
}

inline std::vector<MCGAL::Halfedge*> collectMeshBoundaryEdgesCanonical(const std::shared_ptr<MCGAL::Mesh>& mesh);
inline std::vector<MCGAL::Vertex*> collectMeshBoundaryVerticesCanonical(
    const std::shared_ptr<MCGAL::Mesh>& mesh) {
    std::vector<MCGAL::Vertex*> out;
    if (!mesh) return out;
    std::unordered_set<MCGAL::Vertex*> seen;
    const auto edges = collectMeshBoundaryEdgesCanonical(mesh);
    for (MCGAL::Halfedge* h : edges) {
        if (!h || h->isRemoved()) continue;
        for (MCGAL::Vertex* v : {h->vertex(), h->end_vertex()}) {
            if (v && !v->isRemoved() && seen.insert(v).second) out.push_back(v);
        }
    }
    return out;
}

inline std::vector<MCGAL::Halfedge*> collectMeshBoundaryEdgesCanonical(
    const std::shared_ptr<MCGAL::Mesh>& mesh) {
    std::vector<MCGAL::Halfedge*> out;
    if (!mesh) return out;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        if (!h0) continue;
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && !h->isRemoved() && h->opposite() == nullptr) out.push_back(h);
            h = h->next();
        } while (h && h != h0 && ++guard < 4096);
    }
    return out;
}

/// Boundary edges sorted by quantized endpoint coordinates.  Unlike the face-order
/// canonical list this order is stable across encode/decode face allocations, so a
/// split target can be addressed by its rank in this list.
inline std::vector<MCGAL::Halfedge*> collectMeshBoundaryEdgesSorted(
    const std::shared_ptr<MCGAL::Mesh>& mesh) {
    auto out = collectMeshBoundaryEdgesCanonical(mesh);
    auto key = [&](MCGAL::Halfedge* h) {
        auto a = bfsVertexKey(mesh, h ? h->vertex() : nullptr);
        auto b = bfsVertexKey(mesh, h ? h->end_vertex() : nullptr);
        if (b < a) std::swap(a, b);
        return std::array<long long, 6>{a[0], a[1], a[2], b[0], b[1], b[2]};
    };
    std::stable_sort(out.begin(), out.end(), [&](MCGAL::Halfedge* lhs, MCGAL::Halfedge* rhs) {
        return key(lhs) < key(rhs);
    });
    return out;
}

/// Deterministic face-BFS candidate order shared by encoder and decoder.
/// Every flippable triangle-pair edge is emitted exactly once, in BFS order.
inline std::vector<MCGAL::Halfedge*> collectEdgeFlipCandidatesBfs(
    const std::shared_ptr<MCGAL::Mesh>& mesh) {
    std::vector<MCGAL::Halfedge*> out;
    if (!mesh) return out;
    std::unordered_set<std::uint64_t> seenEdges;
    std::unordered_set<MCGAL::Facet*> visitedFaces;
    std::deque<MCGAL::Facet*> queue;
    auto enqueueFace = [&](MCGAL::Facet* f) {
        if (!f || f->isRemoved() || visitedFaces.count(f)) return;
        visitedFaces.insert(f);
        queue.push_back(f);
    };
    for (MCGAL::Facet* seedFace : mesh->faces()) {
        if (!seedFace || seedFace->isRemoved()) continue;
        bool hasFlippable = false;
        for (MCGAL::Halfedge* h : *seedFace) {
            if (bfsTrianglePairFlippable(h)) { hasFlippable = true; break; }
        }
        if (!hasFlippable) continue;
        enqueueFace(seedFace);
        while (!queue.empty()) {
            MCGAL::Facet* f = queue.front();
            queue.pop_front();
            if (!f || f->isRemoved()) continue;
            for (MCGAL::Halfedge* h : *f) {
                if (!h || h->isRemoved() || !h->opposite()) continue;
                MCGAL::Facet* other = h->opposite()->face();
                if (other && !other->isRemoved()) enqueueFace(other);
                if (!bfsTrianglePairFlippable(h)) continue;
                const std::uint64_t edgeKey = bfsEdgeKey(mesh, h);
                if (edgeKey == 0 || !seenEdges.insert(edgeKey).second) continue;
                                out.push_back(h);
            }
        }
    }
    for (MCGAL::Halfedge*& e : out) {
        if (!e || !e->opposite() || !e->vertex() || !e->end_vertex()) continue;
        auto a = bfsVertexKey(mesh, e->vertex());
        auto b = bfsVertexKey(mesh, e->end_vertex());
        if (a < b) e = e->opposite();
    }
    std::sort(out.begin(), out.end(), [&](MCGAL::Halfedge* lhs, MCGAL::Halfedge* rhs) {
        auto lk = std::array<long long, 6>{0,0,0,0,0,0};
        auto rk = std::array<long long, 6>{0,0,0,0,0,0};
        if (lhs) {
            auto a = bfsVertexKey(mesh, lhs->vertex());
            auto b = bfsVertexKey(mesh, lhs->end_vertex());
            if (b < a) std::swap(a, b);
            lk = {a[0], a[1], a[2], b[0], b[1], b[2]};
        }
        if (rhs) {
            auto a = bfsVertexKey(mesh, rhs->vertex());
            auto b = bfsVertexKey(mesh, rhs->end_vertex());
            if (b < a) std::swap(a, b);
            rk = {a[0], a[1], a[2], b[0], b[1], b[2]};
        }
        return lk < rk;
    });
    return out;
}

inline std::vector<MCGAL::Halfedge*> collectAllEdgesCanonical(
    const std::shared_ptr<MCGAL::Mesh>& mesh);

/// Paper E_r^f collection order: deterministic facet BFS starting from the seed
/// halfedge, visiting each undirected edge exactly once in traversal order.
/// This matches the CIKM paper's "deterministic BFS from a predefined seed
/// half-edge; symbols stored sequentially in traversal order".
inline std::vector<MCGAL::Halfedge*> collectAllEdgesSeedBfs(
    const std::shared_ptr<MCGAL::Mesh>& mesh, MCGAL::Halfedge* seed) {
    std::vector<MCGAL::Halfedge*> out;
    if (!mesh || !seed) return out;
    // Replicate OptimizedSymbolCollectOperator's non-segmented gate BFS:
    // FIFO halfedge queue, first-pop per undirected edge (codec_version=3 rule).
    std::deque<int> gateQueue;
    std::unordered_set<int> visitedHalfedges;
    auto halfedgeById = [&](int hid) -> MCGAL::Halfedge* {
        return MCGAL::contextPool.getHalfedgeByIndexInSubPool(mesh->meshId(), hid);
    };
    auto pushGate = [&](MCGAL::Halfedge* h) {
        if (!h || h->isRemoved()) return;
        if (visitedHalfedges.count(h->poolId())) return;
        gateQueue.push_back(h->poolId());
    };
    pushGate(seed);
    while (!gateQueue.empty()) {
        const int hid = gateQueue.front();
        gateQueue.pop_front();
        MCGAL::Halfedge* h = halfedgeById(hid);
        if (!h || h->isRemoved() || visitedHalfedges.count(hid)) continue;
        visitedHalfedges.insert(hid);
        MCGAL::Halfedge* hOpp = h->opposite();
        const bool emit = (hOpp == nullptr) || !visitedHalfedges.count(hOpp->poolId());
        if (emit) out.push_back(h);
        // Neighbor push order exactly as the collector.
        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOppIt = hIt->opposite();
            if (hOppIt != nullptr && !visitedHalfedges.count(hOppIt->poolId())) {
                pushGate(hOppIt);
            }
            if (hIt != nullptr && !visitedHalfedges.count(hIt->poolId())) {
                pushGate(hIt);
            }
            hIt = hIt->next();
        } while (hIt != h);
    }
    return out;
}


/// Canonical list of ALL undirected edges (one halfedge per edge), sorted by
/// endpoint coordinate keys. Shared by encoder/decoder for the all-edge 1-bit
/// flip stream; independent of face iteration order.
inline std::vector<MCGAL::Halfedge*> collectAllEdgesCanonical(
    const std::shared_ptr<MCGAL::Mesh>& mesh) {
    std::vector<MCGAL::Halfedge*> out;
    if (!mesh) return out;
    std::unordered_set<std::uint64_t> seen;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        for (MCGAL::Halfedge* h : *f) {
            if (!h || h->isRemoved() || !h->opposite() || !h->vertex() || !h->end_vertex()) continue;
            const std::uint64_t key = bfsEdgeKey(mesh, h);
            if (key == 0 || !seen.insert(key).second) continue;
            out.push_back(h);
        }
    }
    std::sort(out.begin(), out.end(), [&](MCGAL::Halfedge* lhs, MCGAL::Halfedge* rhs) {
        auto lk = std::array<long long, 6>{0,0,0,0,0,0};
        auto rk = std::array<long long, 6>{0,0,0,0,0,0};
        if (lhs) {
            auto a = bfsVertexKey(mesh, lhs->vertex());
            auto b = bfsVertexKey(mesh, lhs->end_vertex());
            if (b < a) std::swap(a, b);
            lk = {a[0], a[1], a[2], b[0], b[1], b[2]};
        }
        if (rhs) {
            auto a = bfsVertexKey(mesh, rhs->vertex());
            auto b = bfsVertexKey(mesh, rhs->end_vertex());
            if (b < a) std::swap(a, b);
            rk = {a[0], a[1], a[2], b[0], b[1], b[2]};
        }
        return lk < rk;
    });
    return out;
}


/// Same gate-BFS traversal as collectAllEdgesSeedBfs, but emits only boundary
/// edges: mesh boundary or edges between different group ids.  One bit per
/// undirected boundary edge (first-pop rule), matching E_r^t collection.
inline std::vector<MCGAL::Halfedge*> collectBoundaryEdgesSeedBfs(
    const std::shared_ptr<MCGAL::Mesh>& mesh, MCGAL::Halfedge* seed) {
    std::vector<MCGAL::Halfedge*> out;
    if (!mesh || !seed) return out;
    auto isBoundaryEdge = [](MCGAL::Halfedge* h) -> bool {
        if (!h || h->isRemoved()) return false;
        MCGAL::Halfedge* ho = h->opposite();
        if (!ho || ho->isRemoved()) return true;
        if (!h->face() || !ho->face() || h->face()->isRemoved() || ho->face()->isRemoved()) return true;
        return h->isBoundary() || h->face()->groupId() != ho->face()->groupId();
    };
    std::deque<int> gateQueue;
    std::unordered_set<int> visitedHalfedges;
    std::unordered_set<MCGAL::Facet*> visitedFaces;
    std::unordered_set<std::uint64_t> seenEdges;
    auto halfedgeById = [&](int hid) -> MCGAL::Halfedge* {
        return MCGAL::contextPool.getHalfedgeByIndexInSubPool(mesh->meshId(), hid);
    };
    auto pushGate = [&](MCGAL::Halfedge* h) {
        if (!h || h->isRemoved()) return;
        if (visitedHalfedges.count(h->poolId())) return;
        gateQueue.push_back(h->poolId());
    };
    auto drain = [&]() {
        while (!gateQueue.empty()) {
            const int hid = gateQueue.front();
            gateQueue.pop_front();
            MCGAL::Halfedge* h = halfedgeById(hid);
            if (!h || h->isRemoved() || visitedHalfedges.count(hid)) continue;
            visitedHalfedges.insert(hid);
            if (h->face() && !h->face()->isRemoved()) visitedFaces.insert(h->face());
            MCGAL::Halfedge* hOpp = h->opposite();
            const bool emit = (hOpp == nullptr) || !visitedHalfedges.count(hOpp->poolId());
            if (emit && isBoundaryEdge(h)) {
                const std::uint64_t key = bfsEdgeKey(mesh, h);
                if (key != 0 && seenEdges.insert(key).second) out.push_back(h);
            }
            MCGAL::Halfedge* hIt = h;
            do {
                MCGAL::Halfedge* hOppIt = hIt->opposite();
                if (hOppIt != nullptr && !visitedHalfedges.count(hOppIt->poolId())) pushGate(hOppIt);
                if (hIt != nullptr && !visitedHalfedges.count(hIt->poolId())) pushGate(hIt);
                hIt = hIt->next();
            } while (hIt != h);
        }
    };
    pushGate(seed);
    drain();
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || visitedFaces.count(f)) continue;
        pushGate(f->proxyHalfedge());
        drain();
    }
    return out;
}

}  // namespace pmsf::edgeflip
