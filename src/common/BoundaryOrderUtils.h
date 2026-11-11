#pragma once

#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Vertex.h"
#include "common/BoundedRingTraversal.h"

#include <algorithm>
#include <array>
#include <map>
#include <set>
#include <vector>

namespace pmsf {

/// Deterministic boundary-edge order derived from geometry only.
inline std::vector<MCGAL::Halfedge*> collectBoundaryEdgesInOrder(
    const std::shared_ptr<MCGAL::Mesh>& mesh) {
    std::vector<MCGAL::Halfedge*> out;
    if (!mesh) return out;
    using Key = std::array<int, 3>;
    auto keyOf = [&](MCGAL::Vertex* v) {
        const MCGAL::PointInt q = mesh->floatPosToInt(v->point());
        return Key{q.x(), q.y(), q.z()};
    };
    struct Edge { MCGAL::Halfedge* h = nullptr; Key a{0,0,0}, b{0,0,0}; };
    std::map<int, std::vector<Edge>> byGroup;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && !h->isRemoved() && h->opposite() == nullptr &&
                h->face() && !h->face()->isRemoved() && h->vertex() && h->end_vertex()) {
                byGroup[h->face()->groupId()].push_back({h, keyOf(h->vertex()), keyOf(h->end_vertex())});
            }
            h = h->next();
        } while (h && h != h0 && ++guard < 64);
    }
    struct Loop { std::vector<Key> verts; std::vector<MCGAL::Halfedge*> edges; };
    std::vector<Loop> loops;
    for (auto& kv : byGroup) {
        std::map<Key, std::vector<Key>> adj;
        std::map<std::pair<Key, Key>, MCGAL::Halfedge*> edgeHe;
        for (const Edge& e : kv.second) {
            adj[e.a].push_back(e.b);
            adj[e.b].push_back(e.a);
            edgeHe.emplace(std::make_pair(e.a, e.b), e.h);
            edgeHe.emplace(std::make_pair(e.b, e.a), e.h);
        }
        std::set<Key> remainingVerts;
        for (auto& a : adj) {
            remainingVerts.insert(a.first);
            std::sort(a.second.begin(), a.second.end());
            a.second.erase(std::unique(a.second.begin(), a.second.end()), a.second.end());
        }
        while (!remainingVerts.empty()) {
            Key start = *remainingVerts.begin();
            std::vector<Key> verts;
            std::vector<MCGAL::Halfedge*> edges;
            if (adj[start].size() != 2) {
                std::vector<std::pair<Key, Key>> rem;
                for (const Edge& e : kv.second)
                    if (remainingVerts.count(e.a) || remainingVerts.count(e.b)) rem.push_back({e.a, e.b});
                std::sort(rem.begin(), rem.end());
                for (auto& p : rem) { auto it = edgeHe.find(p); if (it != edgeHe.end()) out.push_back(it->second); }
                break;
            }
            Key cur = start;
            Key nxt = adj[start][0];
            for (const Key& k : adj[start]) if (k < nxt) nxt = k;
            verts.push_back(cur);
            bool ok = true;
            for (int guard = 0; guard < 100000; ++guard) {
                edges.push_back(edgeHe[std::make_pair(cur, nxt)]);
                Key after = nxt;
                if (after == start) { cur = after; break; }
                verts.push_back(after);
                std::vector<Key> cand;
                for (const Key& k : adj[after]) if (!(k == cur)) cand.push_back(k);
                if (cand.size() != 1) { ok = false; break; }
                cur = after;
                nxt = cand[0];
            }
            if (!ok || cur != start) break;
            for (const Key& v : verts) remainingVerts.erase(v);
            if (verts.size() >= 2 && verts.back() < verts[1]) {
                std::reverse(verts.begin(), verts.end());
                std::reverse(edges.begin(), edges.end());
            }
            loops.push_back({verts, edges});
        }
    }
    std::sort(loops.begin(), loops.end(), [](const Loop& a, const Loop& b) { return a.verts < b.verts; });
    for (auto& lp : loops) out.insert(out.end(), lp.edges.begin(), lp.edges.end());
    return out;
}

inline unsigned long long hashBoundaryOrder(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                            const std::vector<MCGAL::Halfedge*>& order) {
    unsigned long long h = 1469598103934665603ULL;
    auto mix = [&](int v) { h ^= static_cast<unsigned long long>(static_cast<unsigned int>(v)); h *= 1099511628211ULL; };
    for (MCGAL::Halfedge* e : order) {
        if (!e || !e->vertex() || !e->end_vertex()) { mix(-1); continue; }
        const MCGAL::PointInt a = mesh->floatPosToInt(e->vertex()->point());
        const MCGAL::PointInt b = mesh->floatPosToInt(e->end_vertex()->point());
        mix(a.x()); mix(a.y()); mix(a.z()); mix(b.x()); mix(b.y()); mix(b.z());
    }
    return h;
}

}  // namespace pmsf
