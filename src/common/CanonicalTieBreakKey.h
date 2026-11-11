#pragma once
// Topology-based tie-break keys for the segmented canonical BFS order.
// Quantized coordinates can collide (fixed low-bit quantization).  The encoder
// and decoder own isomorphic meshes but different pool ids, so poolId must not
// be used to break geometric ties.  These keys are derived only from local
// topology + quantized coordinates and are identical on both sides.
#include "Mesh.h"
#include "Halfedge.h"
#include "Vertex.h"
#include "Facet.h"
#include "common/BoundedRingTraversal.h"
#include <algorithm>
#include <array>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <unordered_map>
#include <vector>

namespace pmsf {

FILE* bfsDumpFile(const char* envValue);

inline std::array<long long, 3> quantizedVertexPoint(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                                     MCGAL::Vertex* v) {
    if (!v) return {LLONG_MIN, LLONG_MIN, LLONG_MIN};
    if (mesh && mesh->enableQuantization) {
        MCGAL::PointInt q = mesh->floatPosToInt(v->point());
        return {static_cast<long long>(q.x()), static_cast<long long>(q.y()),
                static_cast<long long>(q.z())};
    }
    return {static_cast<long long>(std::llround(v->x() * 1000000.0)),
            static_cast<long long>(std::llround(v->y() * 1000000.0)),
            static_cast<long long>(std::llround(v->z() * 1000000.0))};
}

inline std::vector<long long> topologyTieKey(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                             MCGAL::Vertex* v) {
    std::vector<long long> key;
    if (!v) { key.push_back(-1); return key; }
    std::vector<std::array<long long, 4>> neighbors;
    for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
        if (!h || h->isRemoved() || !h->end_vertex() || h->end_vertex()->isRemoved()) continue;
        auto q = quantizedVertexPoint(mesh, h->end_vertex());
        int deg = h->end_vertex()->is_degree_dirty() ? h->end_vertex()->vertex_degree()
                                                     : h->end_vertex()->vertex_degree_cached();
        neighbors.push_back({q[0], q[1], q[2], static_cast<long long>(deg)});
    }
    std::sort(neighbors.begin(), neighbors.end());
    key.push_back(static_cast<long long>(neighbors.size()));
    for (const auto& n : neighbors) {
        key.push_back(n[0]); key.push_back(n[1]); key.push_back(n[2]); key.push_back(n[3]);
    }
    return key;
}

inline std::unordered_map<const MCGAL::Vertex*, std::uint64_t>& stableTopoRankCache() {
    static thread_local std::unordered_map<const MCGAL::Vertex*, std::uint64_t> cache;
    return cache;
}

inline void resetStableTopoRankCache() { stableTopoRankCache().clear(); }

inline std::uint64_t stableTopoRank(const std::shared_ptr<MCGAL::Mesh>&, MCGAL::Vertex* v) {
    if (!v) return 0;
    auto& cache = stableTopoRankCache();
    auto it = cache.find(v);
    if (it != cache.end()) return it->second;
    std::vector<int> neighborDegs;
    int ringSize = 0;
    for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
        if (!h || h->isRemoved() || !h->end_vertex() || h->end_vertex()->isRemoved()) continue;
        MCGAL::Vertex* n = h->end_vertex();
        int nd = 0;
        for (MCGAL::Halfedge* hh : pmsf::boundedRing(n))
            if (hh && !hh->isRemoved()) ++nd;
        neighborDegs.push_back(nd);
        ++ringSize;
    }
    std::sort(neighborDegs.begin(), neighborDegs.end());
    std::uint64_t h = 1469598103934665603ull;
    auto mix = [&](std::uint64_t x) {
        h ^= x;
        h *= 1099511628211ull;
    };
    mix(static_cast<std::uint64_t>(ringSize));
    for (int d : neighborDegs) mix(static_cast<std::uint64_t>(d + 1));
    cache[v] = h;
    return h;
}

inline std::vector<long long> topologyOnlyTieKey(const std::shared_ptr<MCGAL::Mesh>&,
                                                MCGAL::Vertex* v) {
    std::vector<long long> key;
    if (!v) { key.push_back(-1); return key; }
    std::vector<int> degs;
    std::vector<long long> second;
    for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
        if (!h || h->isRemoved() || !h->end_vertex() || h->end_vertex()->isRemoved()) continue;
        MCGAL::Vertex* n = h->end_vertex();
        const int d = n->is_degree_dirty() ? n->vertex_degree() : n->vertex_degree_cached();
        degs.push_back(d);
        int d2 = 0;
        for (MCGAL::Halfedge* hh : pmsf::boundedRing(n))
            if (hh && !hh->isRemoved()) ++d2;
        second.push_back(static_cast<long long>(d2));
    }
    std::sort(degs.begin(), degs.end());
    std::sort(second.begin(), second.end());
    key.push_back(static_cast<long long>(degs.size()));
    for (int d : degs) key.push_back(d);
    key.push_back(-1);
    for (long long d : second) key.push_back(d);
    return key;
}

inline std::vector<long long> faceTieKey(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                         MCGAL::Facet* f) {
    std::vector<long long> key;
    if (!f || f->isRemoved() || !f->proxyHalfedge()) { key.push_back(-1); return key; }
    std::vector<std::array<long long, 3>> pts;
    MCGAL::Halfedge* h0 = f->proxyHalfedge();
    MCGAL::Halfedge* h = h0;
    int guard = 0;
    do {
        if (!h || !h->vertex() || h->vertex()->isRemoved()) { pts.clear(); break; }
        pts.push_back(quantizedVertexPoint(mesh, h->vertex()));
        h = h->next();
    } while (h && h != h0 && ++guard < 256);
    std::sort(pts.begin(), pts.end());
    key.push_back(static_cast<long long>(pts.size()));
    for (const auto& p : pts) { key.push_back(p[0]); key.push_back(p[1]); key.push_back(p[2]); }
    return key;
}

}  // namespace pmsf
