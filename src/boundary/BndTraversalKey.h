#pragma once

// V3 (mode71) canonical BFS traversal key: fully determined by quantized geometry
// and live topology, with NO pool ids.  Used when PMSF_BND_V3 is enabled so the
// encoder collect order and the decoder replay order agree even after boundary
// inverse creates vertices/halfedges at different pool slots.

#include "Mesh.h"
#include "Halfedge.h"
#include "Vertex.h"
#include "common/BoundedRingTraversal.h"

#include <algorithm>
#include <array>
#include <vector>
#include <unordered_map>

namespace pmsf {
namespace bndid {


inline thread_local std::unordered_map<const MCGAL::Vertex*, std::vector<long long>> g_vertexSigCache;
inline thread_local std::unordered_map<const MCGAL::Facet*, std::vector<long long>> g_faceSigCache;
inline void resetTraversalKeyCaches() {
    g_vertexSigCache.clear();
    g_faceSigCache.clear();
}

struct TraversalEntry {
    std::vector<long long> key;
    int hid = -1;
};

struct TraversalCmp {
    bool operator()(const TraversalEntry& a, const TraversalEntry& b) const {
        if (a.key != b.key) return a.key > b.key;  // min-heap on key
        return a.hid > b.hid;                      // only for full-key ties
    }
};

inline void appendVertexSignature(MCGAL::Mesh* mesh, MCGAL::Vertex* v,
                                  std::vector<long long>& out) {
    if (!mesh || !v) {
        out.push_back(-1);
        return;
    }
    const MCGAL::PointInt q = mesh->floatPosToInt(v->point());
    out.push_back(q.x());
    out.push_back(q.y());
    out.push_back(q.z());
    std::vector<std::array<long long, 4>> neighbors;
    for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
        if (!h || h->isRemoved() || !h->end_vertex() || h->end_vertex()->isRemoved()) continue;
        MCGAL::Vertex* n = h->end_vertex();
        const MCGAL::PointInt nq = mesh->floatPosToInt(n->point());
        int deg = 0;
        for (MCGAL::Halfedge* hh : pmsf::boundedRing(n))
            if (hh && !hh->isRemoved()) ++deg;
        neighbors.push_back({nq.x(), nq.y(), nq.z(), static_cast<long long>(deg)});
    }
    std::sort(neighbors.begin(), neighbors.end());
    out.push_back(static_cast<long long>(neighbors.size()));
    for (const auto& n : neighbors) {
        out.push_back(n[0]);
        out.push_back(n[1]);
        out.push_back(n[2]);
        out.push_back(n[3]);
    }
}

inline void appendFaceSignature(MCGAL::Mesh* mesh, MCGAL::Facet* f,
                                std::vector<long long>& out) {
    if (!mesh || !f || f->isRemoved()) {
        out.push_back(-1);
        return;
    }
    std::vector<std::array<long long, 3>> vs;
    for (MCGAL::Halfedge* h : *f) {
        if (!h || !h->vertex()) continue;
        const MCGAL::PointInt q = mesh->floatPosToInt(h->vertex()->point());
        vs.push_back({q.x(), q.y(), q.z()});
    }
    std::sort(vs.begin(), vs.end());
    out.push_back(static_cast<long long>(vs.size()));
    for (const auto& p : vs) {
        out.push_back(p[0]);
        out.push_back(p[1]);
        out.push_back(p[2]);
    }
}

inline TraversalEntry makeTraversalEntry(MCGAL::Mesh* mesh, MCGAL::Halfedge* h) {
    TraversalEntry e;
    e.hid = h ? h->poolId() : -1;
    if (!mesh || !h || !h->vertex() || !h->end_vertex()) {
        e.key.push_back(-1);
        return e;
    }
    auto appendVertex = [&](MCGAL::Vertex* v) {
        if (!v) { e.key.push_back(-1); return; }
        auto it = g_vertexSigCache.find(v);
        if (it == g_vertexSigCache.end()) {
            std::vector<long long> sig;
            appendVertexSignature(mesh, v, sig);
            it = g_vertexSigCache.emplace(v, std::move(sig)).first;
        }
        e.key.insert(e.key.end(), it->second.begin(), it->second.end());
    };
    auto appendFace = [&](MCGAL::Facet* f) {
        if (!f) { e.key.push_back(-1); return; }
        auto it = g_faceSigCache.find(f);
        if (it == g_faceSigCache.end()) {
            std::vector<long long> sig;
            appendFaceSignature(mesh, f, sig);
            it = g_faceSigCache.emplace(f, std::move(sig)).first;
        }
        e.key.insert(e.key.end(), it->second.begin(), it->second.end());
    };
    appendVertex(h->vertex());
    appendVertex(h->end_vertex());
    appendFace(h->face());
    appendFace(h->opposite() ? h->opposite()->face() : nullptr);
    return e;
}

}  // namespace bndid
}  // namespace pmsf