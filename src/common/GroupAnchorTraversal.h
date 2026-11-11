#pragma once
// One permanent anchor per group + deterministic boundary traversal.
#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Vertex.h"
#include "common/BoundedRingTraversal.h"
#include <algorithm>
#include <array>
#include <set>
#include <unordered_set>
#include <vector>

namespace pmsf {

// Deterministic: one mixed-gid boundary vertex per group (mesh->vertices()
// order); fall back to the first face proxy vertex for groups without a
// surviving mixed vertex.
inline std::vector<MCGAL::Vertex*> collectOneAnchorPerGroup(MCGAL::Mesh* mesh) {
    if (!mesh) return {};
    int maxGid = -1;
    for (MCGAL::Facet* f : mesh->faces())
        if (f && !f->isRemoved()) maxGid = std::max(maxGid, f->groupId());
    std::vector<MCGAL::Vertex*> anchors(static_cast<std::size_t>(maxGid + 1), nullptr);
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        std::set<int> gids;
        for (MCGAL::Halfedge* h : pmsf::boundedRing(v))
            if (h && h->face() && !h->face()->isRemoved())
                gids.insert(h->face()->groupId());
        if (gids.size() < 2) continue;
        for (int g : gids)
            if (g >= 0 && g <= maxGid && !anchors[static_cast<std::size_t>(g)])
                anchors[static_cast<std::size_t>(g)] = v;
    }
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        const int g = f->groupId();
        if (g < 0 || g > maxGid || anchors[static_cast<std::size_t>(g)]) continue;
        MCGAL::Halfedge* h = f->proxyHalfedge();
        if (h->vertex() && !h->vertex()->isRemoved())
            anchors[static_cast<std::size_t>(g)] = h->vertex();
        else if (h->end_vertex() && !h->end_vertex()->isRemoved())
            anchors[static_cast<std::size_t>(g)] = h->end_vertex();
    }
    return anchors;
}

// Oriented group-boundary halfedges starting at v with (left gid, right gid).
inline std::vector<MCGAL::Halfedge*> orientedGroupBoundaryAt(MCGAL::Mesh* mesh,
                                                           MCGAL::Vertex* v,
                                                           int gidLeft,
                                                           int gidRight) {
    std::vector<MCGAL::Halfedge*> out;
    if (!mesh || !v) return out;
    for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
        if (!h || h->isRemoved() || !h->vertex() || h->vertex() != v) continue;
        MCGAL::Halfedge* o = h->opposite();
        if (!o || !h->face() || !o->face()) continue;
        if (h->face()->isRemoved() || o->face()->isRemoved()) continue;
        if (h->face()->groupId() == gidLeft && o->face()->groupId() == gidRight)
            out.push_back(h);
    }
    return out;
}

// Walk a group-boundary chain from anchor in both directions.
// Returns unique undirected edges (canonical poolId pair).
inline std::vector<std::pair<int, int>> walkBoundaryFromAnchor(MCGAL::Mesh* mesh,
                                                              MCGAL::Vertex* anchor) {
    std::set<std::pair<int, int>> edges;
    if (!mesh || !anchor || anchor->isRemoved()) return {};
    auto addEdge = [&](MCGAL::Halfedge* h) {
        if (!h || !h->vertex() || !h->end_vertex()) return;
        int a = h->vertex()->poolId();
        int b = h->end_vertex()->poolId();
        if (a > b) std::swap(a, b);
        edges.insert({a, b});
    };
    auto walk = [&](MCGAL::Halfedge* start) {
        MCGAL::Halfedge* h = start;
        for (int guard = 0; guard < 100000 && h; ++guard) {
            addEdge(h);
            MCGAL::Vertex* nextv = h->end_vertex();
            if (!nextv || nextv == anchor) break;
            const int left = h->face() ? h->face()->groupId() : -1;
            const int right = (h->opposite() && h->opposite()->face())
                                  ? h->opposite()->face()->groupId() : -1;
            MCGAL::Halfedge* next = nullptr;
            int incident = 0;
            for (MCGAL::Halfedge* cand : orientedGroupBoundaryAt(mesh, nextv, left, right)) {
                ++incident;
                if (cand == h || (h->opposite() && cand == h->opposite())) continue;
                next = cand;
            }
            if (incident != 1 || !next) break;
            h = next;
        }
    };
    // Deterministic start order: by end-vertex quantized position.
    std::vector<MCGAL::Halfedge*> starts;
    for (MCGAL::Halfedge* h : pmsf::boundedRing(anchor)) {
        if (!h || h->isRemoved() || !h->vertex() || h->vertex() != anchor) continue;
        MCGAL::Halfedge* o = h->opposite();
        if (!o || !h->face() || !o->face() || h->face()->isRemoved() || o->face()->isRemoved()) continue;
        if (h->face()->groupId() == o->face()->groupId()) continue;
        starts.push_back(h);
    }
    std::sort(starts.begin(), starts.end(), [&](MCGAL::Halfedge* a, MCGAL::Halfedge* b) {
        auto qa = mesh->floatPosToInt(a->end_vertex()->point());
        auto qb = mesh->floatPosToInt(b->end_vertex()->point());
        return std::make_tuple(qa.x(), qa.y(), qa.z()) <
               std::make_tuple(qb.x(), qb.y(), qb.z());
    });
    for (MCGAL::Halfedge* s : starts) walk(s);
    std::vector<std::pair<int, int>> out(edges.begin(), edges.end());
    return out;
}


// Traverse all group-boundary edges reachable from the anchor using only
// topology + face group ids.  BFS over boundary-edge graph restricted to
// one side == gid; edges sharing a vertex are adjacent.
inline std::vector<std::pair<int, int>> walkAllGroupBoundaryEdges(
    MCGAL::Mesh* mesh, MCGAL::Vertex* anchor, int gid) {
    std::vector<std::pair<int, int>> out;
    if (!mesh || !anchor || anchor->isRemoved()) return out;
    struct EdgeRef { MCGAL::Halfedge* h; int a; int b; };
    std::vector<EdgeRef> edges;
    std::set<std::pair<int, int>> seen;
    std::unordered_map<int, std::vector<int>> byVertex;
    auto addEdge = [&](MCGAL::Halfedge* h) {
        if (!h || !h->vertex() || !h->end_vertex()) return;
        MCGAL::Vertex* va = h->face() ? h->vertex() : h->end_vertex();
        MCGAL::Vertex* vb = h->face() ? h->end_vertex() : h->vertex();
        int a = va->poolId(), b = vb->poolId();
        if (a > b) std::swap(a, b);
        if (!seen.insert({a, b}).second) return;
        const int idx = static_cast<int>(edges.size());
        edges.push_back({h, a, b});
        byVertex[a].push_back(idx);
        byVertex[b].push_back(idx);
    };
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge() || f->groupId() != gid) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && !h->isRemoved() && h->opposite()) {
                MCGAL::Facet* of = h->opposite()->face();
                if (!of || of->isRemoved() || of->groupId() != gid) addEdge(h);
            } else if (h && !h->isRemoved() && !h->opposite()) {
                addEdge(h);
            }
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
    }
    const int anchorPid = anchor->poolId();
    std::deque<int> dq;
    std::vector<char> visited(edges.size(), 0);
    auto pushVertexEdges = [&](int pid) {
        auto it = byVertex.find(pid);
        if (it == byVertex.end()) return;
        for (int idx : it->second)
            if (!visited[static_cast<std::size_t>(idx)]) {
                visited[static_cast<std::size_t>(idx)] = 1;
                dq.push_back(idx);
            }
    };
    pushVertexEdges(anchorPid);
    while (!dq.empty()) {
        const int idx = dq.front(); dq.pop_front();
        const auto& e = edges[static_cast<std::size_t>(idx)];
        pushVertexEdges(e.a);
        pushVertexEdges(e.b);
    }
    for (std::size_t i = 0; i < edges.size(); ++i) {
        if (visited[i]) out.push_back({edges[i].a, edges[i].b});
    }
    return out;
}

}  // namespace pmsf
