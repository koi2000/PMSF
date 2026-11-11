#pragma once

#include "Halfedge.h"
#include "Mesh.h"
#include "Vertex.h"
#include "common/BoundedRingTraversal.h"

#include <algorithm>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <map>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

namespace pmsf {

/// True if the undirected edge of h is a boundary between two different groups.
inline bool groupBoundaryEdge(MCGAL::Halfedge* h, int& gA, int& gB) {
    if (!h || h->isRemoved() || !h->face() || h->face()->isRemoved()) return false;
    MCGAL::Halfedge* o = h->opposite();
    if (o && !o->isRemoved() && o->face() && !o->face()->isRemoved()) {
        gA = h->face()->groupId();
        gB = o->face()->groupId();
        return gA != gB;
    }
    // mesh outer boundary: one side is outside
    gA = h->face()->groupId();
    gB = -1;
    return true;
}

inline bool isGroupBoundaryHalfedge(MCGAL::Halfedge* h, int gA, int gB) {
    if (!h || h->isRemoved() || !h->face() || h->face()->isRemoved()) return false;
    MCGAL::Halfedge* o = h->opposite();
    if (!o || o->isRemoved() || !o->face() || o->face()->isRemoved()) return false;
    const int a = h->face()->groupId();
    const int b = o->face()->groupId();
    return (a == gA && b == gB) || (a == gB && b == gA);
}

/// Find one halfedge on the (gA,gB) interface incident to anchor, oriented
/// with face(gA) on the halfedge side.  Deterministic by halfedge poolId.
inline MCGAL::Halfedge* findAnchorHalfedge(MCGAL::Mesh* mesh,
                                           MCGAL::Vertex* anchor,
                                           int gA,
                                           int gB) {
    if (!mesh || !anchor || anchor->isRemoved()) return nullptr;
    MCGAL::Halfedge* best = nullptr;
    for (MCGAL::Halfedge* h : anchor->halfedges()) {
        if (!h || h->isRemoved() || !h->face() || h->face()->isRemoved()) continue;
        if (h->face()->groupId() != gA) continue;
        if (!isGroupBoundaryHalfedge(h, gA, gB)) continue;
        if (!best || h->poolId() < best->poolId()) best = h;
    }
    return best;
}

/// Next boundary halfedge along the same (gA,gB) interface, starting from the
/// end vertex of cur and keeping the gA-side face on the halfedge side.
/// This walks only the boundary, never the whole mesh.
/// Next boundary halfedge along the A-side face fan of cur.
/// Walk from cur->next() through internal A faces; the first edge whose
/// opposite face is not A is the paired next boundary edge.
inline MCGAL::Halfedge* nextGroupBoundaryHalfedge(MCGAL::Halfedge* cur, int gA, int gB) {
    if (!cur || cur->isRemoved() || !cur->face() || cur->face()->isRemoved()) return nullptr;
    MCGAL::Vertex* e = cur->end_vertex();
    if (!e || e->isRemoved()) return nullptr;
    int incident = 0;
    for (MCGAL::Halfedge* h : e->halfedges()) {
        if (h && !h->isRemoved() && h->vertex() == e && isGroupBoundaryHalfedge(h, gA, gB)) ++incident;
    }
    if (incident != 2) return nullptr;  // junction endpoint: segment stops here
    MCGAL::Halfedge* out = cur->next();
    for (int step = 0; step < 256; ++step) {
        if (!out || out->isRemoved() || !out->face() || out->face()->isRemoved()) break;
        if (out->face()->groupId() != gA) break;
        MCGAL::Halfedge* oo = out->opposite();
        const int og = (oo && !oo->isRemoved() && oo->face() && !oo->face()->isRemoved())
                           ? oo->face()->groupId()
                           : -1;
        if (og != gA) {
            if (og == gB || (gB < 0 && og < 0)) return out;
            break;
        }
        if (!oo) break;
        out = oo->next();
        if (out == cur) return nullptr;
    }
    // Fallback: at a degree-2 boundary vertex there is exactly one other
    // interface edge; use it when the face-fan walk was interrupted by a third
    // group at the same vertex.
    auto sameEdge = [](MCGAL::Halfedge* a, MCGAL::Halfedge* b) {
        return a && b && ((a->vertex() == b->vertex() && a->end_vertex() == b->end_vertex()) ||
                          (a->vertex() == b->end_vertex() && a->end_vertex() == b->vertex()));
    };
    MCGAL::Halfedge* other = nullptr;
    for (MCGAL::Halfedge* h : e->halfedges()) {
        if (!h || h->isRemoved() || h->vertex() != e || !isGroupBoundaryHalfedge(h, gA, gB)) continue;
        if (sameEdge(h, cur)) continue;
        other = h;
        break;
    }
    if (!other) return nullptr;
    return other->face()->groupId() == gA ? other : other->opposite();
}


struct GroupBoundarySegment {
    int gidA = -1;
    int gidB = -1;
    std::vector<MCGAL::Halfedge*> edges;
    MCGAL::Vertex* anchor = nullptr;
};

/// Encoder-side full boundary scan, but produces only per-segment anchors.
/// Each segment is a maximal chain between junctions (incident pair count != 2).
inline std::vector<GroupBoundarySegment> collectGroupBoundarySegments(MCGAL::Mesh* mesh) {
    std::vector<GroupBoundarySegment> out;
    if (!mesh) return out;
    std::set<std::pair<int,int>> seenEdge;
    std::vector<MCGAL::Halfedge*> edges;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && !h->isRemoved() && h->opposite() && h->opposite()->face() &&
                !h->opposite()->face()->isRemoved()) {
                const int a = h->face() ? h->face()->groupId() : -1;
                const int b = h->opposite()->face()->groupId();
                if (a != b) {
                    int x = h->vertex() ? h->vertex()->poolId() : -1;
                    int y = h->end_vertex() ? h->end_vertex()->poolId() : -2;
                    if (x > y) std::swap(x, y);
                    if (seenEdge.insert({x, y}).second) edges.push_back(h);
                }
            }
            h = h->next();
        } while (h && h != h0 && ++guard < 64);
    }
    std::map<std::pair<int,int>, std::vector<MCGAL::Halfedge*>> byPair;
    for (MCGAL::Halfedge* h : edges) {
        int a = h->face()->groupId();
        int b = h->opposite()->face()->groupId();
        byPair[{std::min(a, b), std::max(a, b)}].push_back(h);
    }
    for (auto& [pair, pg] : byPair) {
        std::unordered_map<int, std::vector<MCGAL::Halfedge*>> incident;
        for (MCGAL::Halfedge* h : pg) {
            if (h->vertex()) incident[h->vertex()->poolId()].push_back(h);
            if (h->end_vertex()) incident[h->end_vertex()->poolId()].push_back(h);
        }
        std::set<MCGAL::Halfedge*> visited;
        for (MCGAL::Halfedge* seedEdge : pg) {
            if (visited.count(seedEdge)) continue;
            GroupBoundarySegment seg;
            seg.gidA = pair.first;
            seg.gidB = pair.second;
            std::deque<MCGAL::Halfedge*> q;
            q.push_back(seedEdge);
            visited.insert(seedEdge);
            while (!q.empty()) {
                MCGAL::Halfedge* cur = q.front(); q.pop_front();
                seg.edges.push_back(cur);
                for (int vp : {cur->vertex() ? cur->vertex()->poolId() : -1,
                               cur->end_vertex() ? cur->end_vertex()->poolId() : -1}) {
                    auto it = incident.find(vp);
                    if (it == incident.end() || it->second.size() != 2) continue;
                    for (MCGAL::Halfedge* nx : it->second) {
                        if (nx != cur && visited.insert(nx).second) q.push_back(nx);
                    }
                }
            }
            for (MCGAL::Halfedge* h : seg.edges) {
                for (MCGAL::Vertex* cand : {h->vertex(), h->end_vertex()}) {
                    if (!cand) continue;
                    auto it = incident.find(cand->poolId());
                    if (it != incident.end() && it->second.size() == 2) { seg.anchor = cand; break; }
                }
                if (seg.anchor) break;
            }
            if (!seg.anchor && !seg.edges.empty()) seg.anchor = seg.edges.front()->vertex();
            out.push_back(std::move(seg));
        }
    }
    return out;
}

/// Enumerate one open boundary segment (or closed loop) starting at start.
/// Walks both directions from start so a segment that ends at a triple junction
/// is fully covered by one anchor.
inline std::vector<MCGAL::Halfedge*> enumerateGroupBoundaryChain(MCGAL::Mesh* mesh,
                                                                MCGAL::Halfedge* start,
                                                                int gA,
                                                                int gB,
                                                                int maxSteps = 100000) {
    std::vector<MCGAL::Halfedge*> forward;
    if (!mesh || !start || start->isRemoved() || !start->face()) return forward;
    std::set<int> seen;
    MCGAL::Halfedge* cur = start;
    bool closed = false;
    for (int i = 0; i < maxSteps; ++i) {
        if (!cur || cur->isRemoved()) break;
        if (!seen.insert(cur->poolId()).second) break;
        forward.push_back(cur);
        MCGAL::Halfedge* nxt = nextGroupBoundaryHalfedge(cur, gA, gB);
        if (!nxt) break;
        if (nxt == start) { closed = true; break; }
        cur = nxt;
    }
    if (closed) return forward;
    std::vector<MCGAL::Halfedge*> backward;
    if (start->opposite() && !start->opposite()->isRemoved()) {
        MCGAL::Halfedge* bcur = start->opposite();
        std::set<int> bseen;
        for (int i = 0; i < maxSteps; ++i) {
            if (!bcur || bcur->isRemoved()) break;
            if (!bseen.insert(bcur->poolId()).second) break;
            backward.push_back(bcur);
            MCGAL::Halfedge* nxt = nextGroupBoundaryHalfedge(bcur, gB, gA);
            if (!nxt) break;
            bcur = nxt;
        }
    }
    std::vector<MCGAL::Halfedge*> out;
    out.reserve(backward.size() + forward.size());
    for (auto it = backward.rbegin(); it != backward.rend(); ++it) out.push_back(*it);
    for (MCGAL::Halfedge* h : forward) if (h != start) out.push_back(h);
    return out;
}

/// Enumerate one group-boundary loop starting at start.  Returns the ordered
/// halfedges of the loop.  Stops when the loop closes or when the walk would
/// revisit a halfedge (non-simple junction).
inline std::vector<MCGAL::Halfedge*> enumerateGroupBoundaryLoop(MCGAL::Mesh* mesh,
                                                               MCGAL::Halfedge* start,
                                                               int gA,
                                                               int gB,
                                                               int maxSteps = 100000) {
    std::vector<MCGAL::Halfedge*> out;
    if (!mesh || !start || start->isRemoved() || start->face()->groupId() != gA) return out;
    std::set<int> seen;
    MCGAL::Halfedge* cur = start;
    for (int i = 0; i < maxSteps; ++i) {
        if (!cur || cur->isRemoved()) break;
        if (!seen.insert(cur->poolId()).second) break;
        out.push_back(cur);
        MCGAL::Halfedge* nxt = nextGroupBoundaryHalfedge(cur, gA, gB);
        if (!nxt) break;
        if (nxt == start) break;
        cur = nxt;
    }
    return out;
}

/// Enumerate ALL group-boundary loops by walking from anchors only.
/// Interface key = ordered (min(gidA,gidB), max(...)).
/// Returns loops, each as ordered halfedges with face gA (first smaller group).


/// One anchor per group (as requested: each group maintains one vertex).
/// If a group has multiple disconnected boundary components, only the first is
/// represented; callers that need all components should use the segment walker.
inline std::vector<std::pair<int, MCGAL::Vertex*>> collectGroupBoundaryAnchors(MCGAL::Mesh* mesh) {
    std::vector<std::pair<int, MCGAL::Vertex*>> out;
    if (!mesh) return out;
    auto segs = collectGroupBoundarySegments(mesh);
    std::map<int, std::vector<MCGAL::Halfedge*>> byGroup;
    for (auto& seg : segs) {
        for (MCGAL::Halfedge* h : seg.edges) {
            if (!h) continue;
            byGroup[h->face() ? h->face()->groupId() : -1].push_back(h);
            if (h->opposite() && h->opposite()->face()) byGroup[h->opposite()->face()->groupId()].push_back(h);
        }
    }
    for (auto& [gid, edges] : byGroup) {
        if (gid < 0 || edges.empty()) continue;
        MCGAL::Vertex* anchor = nullptr;
        for (MCGAL::Halfedge* h : edges) {
            for (MCGAL::Vertex* cand : {h->vertex(), h->end_vertex()}) {
                if (!cand) continue;
                int incident = 0;
                for (MCGAL::Halfedge* hh : cand->halfedges()) {
                    if (hh && !hh->isRemoved() && hh->vertex() == cand &&
                        isGroupBoundaryHalfedge(hh, gid, -1)) ++incident;
                }
                if (incident == 2) { anchor = cand; break; }
            }
            if (anchor) break;
        }
        if (!anchor) anchor = edges.front()->vertex();
        out.push_back({gid, anchor});
    }
    return out;
}
}  // namespace pmsf
