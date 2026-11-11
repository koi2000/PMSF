#pragma once

/// Single-copy group-boundary vertex collapse prototype.
///
/// Motivation (new architecture, D181): the old seam-cut path duplicates
/// boundary vertices between groups and later merges by position, which is
/// ambiguous under quantization collisions.  In the intended single-copy
/// layout, the mesh is compressed as ONE global mesh: a group-boundary vertex
/// exists once and is shared by both groups.  Removing it must therefore
/// retriangulate BOTH adjacent group fans and be exactly invertible.
///
/// This header implements that operator and an in-memory round-trip test.
/// No serialization yet: patches hold live vertex pointers (prototype only).

#include "common/BoundedRingTraversal.h"
#include "common/BoundaryPatchCodec.h"
#include "Mesh.h"
#include "Vertex.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace pmsf {

struct SingleBoundaryFace {
    int gid = -1;
    MCGAL::Vertex* v[3] = {nullptr, nullptr, nullptr};
    MCGAL::Facet* facet = nullptr;  // pre-collapse facet (for order restoration)
    int rank = -1;                  // live-facet rank in mesh->faces() before the collapse
};

struct SingleBoundaryPatch {
    MCGAL::Vertex* removed = nullptr;
    MCGAL::Point pos;
    int gid = -1;
    std::vector<SingleBoundaryFace> origFaces;
    std::vector<SingleBoundaryFace> addedFaces;
    // Explicit rings / groups for the two sides (needed by the corner case
    // kA==1 or kB==1, where one side contributes no new faces and the rings
    // cannot be derived from addedFaces).
    std::vector<MCGAL::Vertex*> ringA;
    std::vector<MCGAL::Vertex*> ringB;
    int removedVertexRank = -1;  // live-vertex rank before the collapse (order restore)
    int sideGidA = -1;
    int sideGidB = -1;
    bool valid = false;
};

namespace sbdetail {

inline std::vector<MCGAL::Halfedge*> fanOutHalfedges(MCGAL::Mesh* mesh, MCGAL::Vertex* v) {
    std::vector<MCGAL::Halfedge*> out;
    if (!mesh || !v) return out;
    // Ring-based fan collection: O(ring) instead of scanning every face of the mesh
    // (the old full-face scan dominated K-free stages on 100k+ meshes).
    for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
        if (!h || h->isRemoved() || !h->face() || h->face()->isRemoved()) continue;
        if (h->vertex() != v) continue;
        out.push_back(h);
    }
    return out;
}

inline std::array<MCGAL::Vertex*, 3> faceVertices(MCGAL::Facet* f) {
    std::array<MCGAL::Vertex*, 3> vs{nullptr, nullptr, nullptr};
    if (!f || f->isRemoved() || !f->proxyHalfedge()) return vs;
    MCGAL::Halfedge* h0 = f->proxyHalfedge();
    MCGAL::Halfedge* h = h0;
    int k = 0, guard = 0;
    do {
        if (h && h->vertex() && k < 3) vs[k++] = h->vertex();
        h = h ? h->next() : nullptr;
    } while (h && h != h0 && ++guard < 64);
    if (k != 3) return {nullptr, nullptr, nullptr};
    return vs;
}

/// Walk one side of v's fan starting at h (h->vertex()==v, and the edge
/// (v, h->end_vertex()) is a group-boundary edge on this side).  Returns the
/// polygon ring of this side (starting at h->end_vertex(), ending at the other
/// boundary neighbour) and the number of walked faces.
inline bool walkBoundaryArc(MCGAL::Mesh* mesh,
                            MCGAL::Halfedge* hStart,
                            int group,
                            std::vector<MCGAL::Vertex*>& ring) {
    ring.clear();
    if (!mesh || !hStart || hStart->isRemoved() || !hStart->face()) return false;
    if (!hStart->vertex() || !hStart->end_vertex()) return false;
    if (hStart->face()->groupId() != group) return false;
    ring.push_back(hStart->end_vertex());
    MCGAL::Halfedge* h = hStart;
    int guard = 0;
    while (h && ++guard < 100000) {
        MCGAL::Halfedge* hIn = h->next() ? h->next()->next() : nullptr;  // (q -> v)
        if (!hIn || hIn->end_vertex() != h->vertex()) return false;
        MCGAL::Vertex* q = hIn->vertex();
        if (!q) return false;
        ring.push_back(q);
        MCGAL::Halfedge* opp = hIn->opposite();  // (v -> q)
        if (!opp || opp->isRemoved() || !opp->face() || opp->face()->isRemoved() ||
            opp->face()->groupId() != group) {
            return true;  // reached the other group-boundary edge
        }
        h = opp;
    }
    return false;
}

}  // namespace sbdetail

/// Try to remove one regular group-boundary vertex v (exactly two boundary
/// edges, one group fan on each side).  On success the local patch is stored
/// in the patch and the mesh stays a closed manifold.
struct FaceTripleKey {
    MCGAL::Vertex* v[3] = {nullptr, nullptr, nullptr};
    bool operator==(const FaceTripleKey& o) const {
        return v[0] == o.v[0] && v[1] == o.v[1] && v[2] == o.v[2];
    }
};
struct FaceTripleKeyHash {
    std::size_t operator()(const FaceTripleKey& k) const {
        std::size_t h = 1469598103934665603ull;
        for (int i = 0; i < 3; ++i) {
            h ^= reinterpret_cast<std::uintptr_t>(k.v[i]);
            h *= 1099511628211ull;
        }
        return h;
    }
};
inline FaceTripleKey makeFaceTripleKey(MCGAL::Vertex* a, MCGAL::Vertex* b, MCGAL::Vertex* c) {
    FaceTripleKey k;
    k.v[0] = a;
    k.v[1] = b;
    k.v[2] = c;
    std::sort(std::begin(k.v), std::end(k.v));
    return k;
}
using FaceTripleMap =
    std::unordered_map<FaceTripleKey, MCGAL::Facet*, FaceTripleKeyHash>;

inline FaceTripleMap buildFaceTripleMap(MCGAL::Mesh* mesh) {
    FaceTripleMap m;
    if (!mesh) return m;
    m.reserve(mesh->faces().size() * 2 + 1);
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        std::vector<MCGAL::Vertex*> vs;
        for (MCGAL::Halfedge* h = f->proxyHalfedge(); h; h = h->next()) {
            if (h->vertex()) vs.push_back(h->vertex());
            if (h->next() == f->proxyHalfedge()) break;
        }
        if (vs.size() != 3) continue;
        m[makeFaceTripleKey(vs[0], vs[1], vs[2])] = f;
    }
    return m;
}

inline bool collapseSingleGroupBoundaryVertex(
    const std::shared_ptr<MCGAL::Mesh>& mesh, MCGAL::Vertex* v, SingleBoundaryPatch& patch,
    FaceTripleMap* faceTriples = nullptr) {
    using namespace sbdetail;
    patch = SingleBoundaryPatch{};
    if (!mesh || !v || v->isRemoved()) return false;
    const auto hvs = fanOutHalfedges(mesh.get(), v);
    const bool allowCorner = std::getenv("PMSF_SC_ALLOW_CORNER_COLLAPSE") != nullptr;
    if (hvs.size() < 3) return false;
    if (hvs.size() < 4 && !allowCorner) return false;  // corner: one side is a single triangle

    // Collect group-boundary edges incident to v (undirected, by neighbour).
    std::set<MCGAL::Vertex*> boundaryNeighbours;
    for (MCGAL::Halfedge* h : hvs) {
        if (!h || h->isRemoved() || !h->face()) return false;
        // outgoing edge (v -> p)
        MCGAL::Vertex* p = h->end_vertex();
        MCGAL::Halfedge* opp = h->opposite();
        auto isBoundaryEdge = [&](MCGAL::Halfedge* other) {
            if (!other || other->isRemoved()) return true;  // real boundary
            if (!other->face() || other->face()->isRemoved()) return true;
            return other->face()->groupId() != h->face()->groupId();
        };
        if (isBoundaryEdge(opp) && p) boundaryNeighbours.insert(p);
        // incoming edge (q -> v) == same edge seen from the next face
        MCGAL::Halfedge* hIn = h->next() ? h->next()->next() : nullptr;
        if (hIn && hIn->vertex()) {
            MCGAL::Halfedge* opp2 = hIn->opposite();
            if (!opp2 || opp2->isRemoved() || !opp2->face() || opp2->face()->isRemoved() ||
                opp2->face()->groupId() != h->face()->groupId()) {
                boundaryNeighbours.insert(hIn->vertex());
            }
        }
    }
    if (boundaryNeighbours.size() != 2) return false;
    MCGAL::Vertex* a = *boundaryNeighbours.begin();
    MCGAL::Vertex* b = *boundaryNeighbours.rbegin();
    if (!a || !b || a == b) return false;

    // Find the group-A start halfedge: (v -> a) whose opposite is a different group.
    MCGAL::Halfedge* startA = nullptr;
    for (MCGAL::Halfedge* h : hvs) {
        if (!h || !h->end_vertex()) continue;
        if (h->end_vertex() != a) continue;
        MCGAL::Halfedge* opp = h->opposite();
        if (!opp || opp->isRemoved() || !opp->face() || opp->face()->isRemoved() ||
            opp->face()->groupId() != h->face()->groupId()) {
            startA = h;
            break;
        }
    }
    if (!startA) {
        // Orientation may be v <- a on this side; use (v -> ?) from the other edge.
        for (MCGAL::Halfedge* h : hvs) {
            if (!h || !h->vertex()) continue;
            if (h->vertex() != a) continue;
            MCGAL::Halfedge* opp = h->opposite();
            if (!opp || opp->isRemoved() || !opp->face() || opp->face()->isRemoved() ||
                opp->face()->groupId() != h->face()->groupId()) {
                // boundary edge (a -> v); its opposite (v -> a) starts the arc
                startA = opp;
                break;
            }
        }
    }
    if (!startA || !startA->face()) return false;
    const int gA = startA->face()->groupId();

    std::vector<MCGAL::Vertex*> ringA, ringB;
    if (!walkBoundaryArc(mesh.get(), startA, gA, ringA)) return false;
    if (ringA.size() < 2 || (ringA.size() == 2 && !allowCorner)) return false;
    if (ringA.back() != b) {
        // may be walking the wrong side; try the other boundary edge as startA
        return false;
    }

    // group-B arc starts at (v -> b) on the B side.
    MCGAL::Halfedge* hInLast = nullptr;
    {
        MCGAL::Halfedge* cur = startA;
        while (cur) {
            MCGAL::Halfedge* hIn = cur->next() ? cur->next()->next() : nullptr;
            MCGAL::Halfedge* opp = hIn ? hIn->opposite() : nullptr;
            if (!opp || opp->isRemoved() || !opp->face() || opp->face()->isRemoved() ||
                opp->face()->groupId() != gA) {
                hInLast = hIn;
                break;
            }
            cur = opp;
        }
    }
    if (!hInLast) return false;
    MCGAL::Halfedge* startB = hInLast->opposite();  // (v -> b) on the B side
    if (!startB || startB->isRemoved() || !startB->face()) return false;
    const int gB = startB->face()->groupId();
    if (gB == gA) return false;
    if (!walkBoundaryArc(mesh.get(), startB, gB, ringB)) return false;
    if (ringB.size() < 2 || (ringB.size() == 2 && !allowCorner) || ringB.back() != a)
        return false;

    // record original fan faces
    patch.removed = v;
    patch.pos = v->point();
    patch.gid = v->groupId();
    patch.origFaces.clear();
    for (MCGAL::Halfedge* h : hvs) {
        MCGAL::Facet* f = h ? h->face() : nullptr;
        const auto vs = faceVertices(f);
        if (!vs[0]) return false;
        SingleBoundaryFace rec;
        rec.gid = f->groupId();
        rec.facet = f;
        for (int k = 0; k < 3; ++k) rec.v[k] = vs[k];
        patch.origFaces.push_back(rec);
    }
    // NOTE: rank metadata is normally disabled (O(V+F) per candidate).  When the
    // V3 ordered-inverse mode is enabled we capture it (selected rounds only), so the
    // decoder can restore the encoder's containers order and keep interior replay exact.
    patch.removedVertexRank = -1;
    static const bool wantOrderRanks = [] {
        const char* e = std::getenv("PMSF_BND_V3_ORDER_ROUNDS");
        return e != nullptr && e[0] != '\0';
    }();
    if (wantOrderRanks) {
        int vr = 0;
        bool found = false;
        for (MCGAL::Vertex* mv : mesh->vertices()) {
            if (!mv || mv->isRemoved()) continue;
            if (mv == patch.removed) { found = true; break; }
            ++vr;
        }
        patch.removedVertexRank = found ? vr : -1;
        if (patch.removedVertexRank >= 0) {
            for (auto& rec : patch.origFaces) {
                int pos = 0;
                bool ffound = false;
                for (MCGAL::Facet* mf : mesh->faces()) {
                    if (!mf || mf->isRemoved()) continue;
                    if (mf == rec.facet) { ffound = true; break; }
                    ++pos;
                }
                rec.rank = ffound ? pos : -1;
                if (!ffound) { patch.removedVertexRank = -1; break; }
            }
        }
    }

    // plan the new fan triangles first (needed for duplicate-face pre-check
    // before touching the mesh)
    struct PlannedTri { MCGAL::Vertex* a; MCGAL::Vertex* b; MCGAL::Vertex* c; int gid; };
    std::vector<PlannedTri> planned;
    auto planFan = [&](const std::vector<MCGAL::Vertex*>& ring, int gid) {
        for (std::size_t i = 1; i + 1 < ring.size(); ++i)
            planned.push_back({ring[0], ring[i], ring[i + 1], gid});
    };
    planFan(ringA, gA);
    planFan(ringB, gB);
    if (planned.empty()) return false;  // both sides would vanish => a hole
    patch.ringA = ringA;
    patch.ringB = ringB;
    patch.sideGidA = gA;
    patch.sideGidB = gB;

    // collect old fan faces and reject if any planned triangle already exists
    // outside the old fan (createFaceFromTripleManual would return the existing
    // face and leave a hole instead of creating a new one).
    std::vector<MCGAL::Facet*> oldFaces;
    std::unordered_set<MCGAL::Facet*> oldSet;
    for (MCGAL::Halfedge* h : hvs) {
        if (h && h->face() && !h->face()->isRemoved() && oldSet.insert(h->face()).second)
            oldFaces.push_back(h->face());
    }
    auto sameTriple = [](const std::array<MCGAL::Vertex*, 3>& vs,
                         MCGAL::Vertex* a, MCGAL::Vertex* b, MCGAL::Vertex* c) {
        if (!vs[0] || !vs[1] || !vs[2]) return false;
        for (int i = 0; i < 3; ++i) {
            MCGAL::Vertex* x = vs[i];
            if (x != a && x != b && x != c) return false;
        }
        for (MCGAL::Vertex* x : {a, b, c}) {
            bool found = false;
            for (int i = 0; i < 3; ++i)
                if (vs[i] == x) { found = true; break; }
            if (!found) return false;
        }
        return true;
    };
    if (faceTriples) {
        // O(1) duplicate-triangle test against the maintained face-triple map
        // (built once per stage, updated on successful patches).  A match on an
        // old-fan face is fine because that face is removed by this collapse.
        for (const auto& p : planned) {
            auto it = faceTriples->find(makeFaceTripleKey(p.a, p.b, p.c));
            if (it != faceTriples->end() && it->second && !oldSet.count(it->second)) return false;
        }
    } else {
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved() || oldSet.count(f)) continue;
            const auto vs = faceVertices(f);
            for (const auto& p : planned) {
                if (sameTriple(vs, p.a, p.b, p.c)) return false;
            }
        }
    }

    for (MCGAL::Facet* f : oldFaces) removeFaceForPatch(mesh, f);
    v->halfedges().clear();

    patch.addedFaces.clear();
    std::vector<MCGAL::Facet*> created;
    bool createOk = true;
    for (const auto& p : planned) {
        MCGAL::Facet* nf = createFaceFromTripleManual(mesh, p.a, p.b, p.c, p.gid);
        if (!nf) { createOk = false; break; }
        created.push_back(nf);
        SingleBoundaryFace rec;
        rec.gid = p.gid;
        rec.v[0] = p.a;
        rec.v[1] = p.b;
        rec.v[2] = p.c;
        rec.facet = nf;
        patch.addedFaces.push_back(rec);
    }
    if (!createOk) {
        // roll back: remove created fan, re-add the original faces
        for (MCGAL::Facet* f : created) removeFaceForPatch(mesh, f);
        for (MCGAL::Facet* f : oldFaces) {
            removeFaceForPatch(mesh, f);  // no-op if already removed
        }
        for (const auto& rec : patch.origFaces)
            createFaceFromTripleManual(mesh, rec.v[0], rec.v[1], rec.v[2], rec.gid);
        patch = SingleBoundaryPatch{};
        return false;
    }
    v->setRemoved();
    patch.valid = true;
    if (faceTriples) {
        for (MCGAL::Facet* f : oldFaces) {
            if (!f) continue;
            std::vector<MCGAL::Vertex*> vs;
            for (MCGAL::Halfedge* h = f->proxyHalfedge(); h; h = h->next()) {
                if (h->vertex()) vs.push_back(h->vertex());
                if (h->next() == f->proxyHalfedge()) break;
            }
            if (vs.size() == 3) faceTriples->erase(makeFaceTripleKey(vs[0], vs[1], vs[2]));
        }
        for (MCGAL::Facet* f : created) {
            if (!f) continue;
            std::vector<MCGAL::Vertex*> vs;
            for (MCGAL::Halfedge* h = f->proxyHalfedge(); h; h = h->next()) {
                if (h->vertex()) vs.push_back(h->vertex());
                if (h->next() == f->proxyHalfedge()) break;
            }
            if (vs.size() == 3) (*faceTriples)[makeFaceTripleKey(vs[0], vs[1], vs[2])] = f;
        }
    }
    return true;
}

/// Undo one collapse patch (inverse order of application).
inline bool restoreSingleBoundaryPatch(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                      const SingleBoundaryPatch& patch) {
    if (!mesh || !patch.valid || !patch.removed) return false;
    auto sameFace = [](MCGAL::Facet* f, const SingleBoundaryFace& rec) {
        if (!f || f->isRemoved() || !f->proxyHalfedge() || f->groupId() != rec.gid) return false;
        const auto vs = sbdetail::faceVertices(f);
        if (!vs[0]) return false;
        for (int i = 0; i < 3; ++i) {
            bool found = false;
            for (int k = 0; k < 3; ++k)
                if (vs[k] == rec.v[i]) { found = true; break; }
            if (!found) return false;
        }
        return true;
    };
    std::vector<MCGAL::Facet*> toRemove;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        for (const auto& rec : patch.addedFaces) {
            if (sameFace(f, rec)) { toRemove.push_back(f); break; }
        }
    }
    for (MCGAL::Facet* f : toRemove) removeFaceForPatch(mesh, f);
    patch.removed->setNotRemoved();
    patch.removed->halfedges().clear();
    patch.removed->setPoint(patch.pos);
    patch.removed->setGroupId(patch.gid);
    for (const auto& rec : patch.origFaces) {
        if (!createFaceFromTripleManual(mesh, rec.v[0], rec.v[1], rec.v[2], rec.gid)) return false;
    }
    pmsf::rebuildVertexHalfedgeRings(mesh);
    return true;
}

inline std::string singleBoundarySignature(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!mesh) return "";
    std::vector<std::array<int, 4>> faces;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        const auto vs = sbdetail::faceVertices(f);
        if (!vs[0]) continue;
        std::array<int, 3> pid{vs[0]->poolId(), vs[1]->poolId(), vs[2]->poolId()};
        std::sort(pid.begin(), pid.end());
        faces.push_back({pid[0], pid[1], pid[2], f->groupId()});
    }
    std::sort(faces.begin(), faces.end());
    std::vector<int> verts;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (v && !v->isRemoved()) verts.push_back(v->poolId());
    }
    std::sort(verts.begin(), verts.end());
    std::string sig = "V" + std::to_string(verts.size()) + ":";
    for (int p : verts) sig += std::to_string(p) + ",";
    sig += "|F" + std::to_string(faces.size()) + ":";
    for (const auto& f : faces) {
        sig += std::to_string(f[0]) + "-" + std::to_string(f[1]) + "-" +
               std::to_string(f[2]) + "-" + std::to_string(f[3]) + ";";
    }
    return sig;
}

inline void countSingleBoundaryManifold(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                        int& boundaryEdges, int& nonManifoldEdges,
                                        int& dupFaces, int& nonTriFaces) {
    boundaryEdges = nonManifoldEdges = dupFaces = nonTriFaces = 0;
    if (!mesh) return;
    std::map<std::pair<int, int>, int> edgeCount;
    std::set<std::array<int, 4>> faceKeys;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int n = 0, guard = 0;
        std::array<int, 3> pid{};
        do {
            if (h && h->vertex()) {
                if (n < 3) pid[n] = h->vertex()->poolId();
                ++n;
            }
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
        if (n != 3) { ++nonTriFaces; continue; }
        std::array<int, 3> sortedPid = pid;
        std::sort(sortedPid.begin(), sortedPid.end());
        if (!faceKeys.insert({sortedPid[0], sortedPid[1], sortedPid[2], f->groupId()}).second)
            ++dupFaces;
        h = h0;
        guard = 0;
        do {
            if (h && h->vertex() && h->next() && h->next()->vertex()) {
                int u = h->vertex()->poolId();
                int w = h->next()->vertex()->poolId();
                if (u > w) std::swap(u, w);
                ++edgeCount[{u, w}];
            }
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
    }
    for (const auto& kv : edgeCount) {
        if (kv.second == 1) ++boundaryEdges;
        else if (kv.second > 2) ++nonManifoldEdges;
    }
}

/// Cheap local validation: every halfedge of the newly created fan faces must be
/// opposite-paired (i.e. the local patch is closed and consistently oriented).
inline bool localSingleBoundaryPatchClosed(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                           const SingleBoundaryPatch& patch) {
    if (!mesh || !patch.valid) return false;
    auto sameFace = [](MCGAL::Facet* f, const SingleBoundaryFace& rec) {
        if (!f || f->isRemoved() || !f->proxyHalfedge() || f->groupId() != rec.gid) return false;
        const auto vs = sbdetail::faceVertices(f);
        if (!vs[0]) return false;
        for (int i = 0; i < 3; ++i) {
            bool found = false;
            for (int k = 0; k < 3; ++k)
                if (vs[k] == rec.v[i]) { found = true; break; }
            if (!found) return false;
        }
        return true;
    };
    for (const auto& rec : patch.addedFaces) {
        MCGAL::Facet* found = rec.facet;
        if (!found || found->isRemoved() || !sameFace(found, rec)) {
            found = nullptr;
            for (MCGAL::Facet* f : mesh->faces()) {
                if (!f || f->isRemoved()) continue;
                if (sameFace(f, rec)) { found = f; break; }
            }
        }
        if (!found) return false;
        MCGAL::Halfedge* h0 = found->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (!h || h->isRemoved() || !h->opposite() || h->opposite()->isRemoved())
                return false;
            h = h->next();
        } while (h && h != h0 && ++guard < 64);
    }
    return true;
}

/// In-memory prototype: apply up to maxOps group-boundary collapses, validate
/// the mesh stays closed, then undo in reverse and compare exact signatures.
inline int runSingleBoundaryRoundTripTest(std::shared_ptr<MCGAL::Mesh> mesh, int maxOps) {
    if (!mesh) return 1;
    const std::string sig0 = singleBoundarySignature(mesh);
    auto liveV = [&]() {
        int n = 0;
        for (MCGAL::Vertex* v : mesh->vertices()) if (v && !v->isRemoved()) ++n;
        return n;
    };
    const int V0 = liveV();
    const int F0 = mesh->size_of_facets();
    int baseBoundary = 0, baseNonManifold = 0, baseDup = 0, baseNonTri = 0;
    countSingleBoundaryManifold(mesh, baseBoundary, baseNonManifold, baseDup, baseNonTri);
    std::vector<SingleBoundaryPatch> patches;
    std::set<int> rejected;
    int applied = 0;
    for (int iter = 0; iter < maxOps; ++iter) {
        std::set<MCGAL::Vertex*> cands;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
            MCGAL::Halfedge* h0 = f->proxyHalfedge();
            MCGAL::Halfedge* h = h0;
            int guard = 0;
            do {
                MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
                if (h && h->vertex() && h->next() && h->next()->vertex() && opp &&
                    !opp->isRemoved() && opp->face() && !opp->face()->isRemoved() &&
                    opp->face()->groupId() != f->groupId()) {
                    if (!h->vertex()->isRemoved()) cands.insert(h->vertex());
                    if (!h->next()->vertex()->isRemoved()) cands.insert(h->next()->vertex());
                }
                h = h ? h->next() : nullptr;
            } while (h && h != h0 && ++guard < 64);
        }
        if (cands.empty()) break;
        std::vector<MCGAL::Vertex*> ordered(cands.begin(), cands.end());
        std::sort(ordered.begin(), ordered.end(), [&](MCGAL::Vertex* a, MCGAL::Vertex* b) {
            const auto qa = mesh->floatPosToInt(a->point());
            const auto qb = mesh->floatPosToInt(b->point());
            return std::array<int, 3>{qa.x(), qa.y(), qa.z()} <
                   std::array<int, 3>{qb.x(), qb.y(), qb.z()};
        });
        bool ok = false;
        for (MCGAL::Vertex* v : ordered) {
            if (!v || rejected.count(v->poolId())) continue;
            SingleBoundaryPatch patch;
            if (!collapseSingleGroupBoundaryVertex(mesh, v, patch)) {
                rejected.insert(v->poolId());
                continue;
            }
            // Only accept collapses whose local patch is closed/oriented;
            // invalid local surgeries are rolled back immediately.
            if (!localSingleBoundaryPatchClosed(mesh, patch)) {
                restoreSingleBoundaryPatch(mesh, patch);
                rejected.insert(v->poolId());
                continue;
            }
            patches.push_back(patch);
            ++applied;
            ok = true;
            break;
        }
        if (!ok) break;
    }
    int boundaryEdges = 0, nonManifold = 0, dupFaces = 0, nonTri = 0;
    countSingleBoundaryManifold(mesh, boundaryEdges, nonManifold, dupFaces, nonTri);
    const int V1 = liveV();
    const int F1 = mesh->size_of_facets();
    bool undoOk = true;
    for (auto it = patches.rbegin(); it != patches.rend(); ++it) {
        if (!restoreSingleBoundaryPatch(mesh, *it)) { undoOk = false; break; }
    }
    int boundaryEdges2 = 0, nonManifold2 = 0, dupFaces2 = 0, nonTri2 = 0;
    countSingleBoundaryManifold(mesh, boundaryEdges2, nonManifold2, dupFaces2, nonTri2);
    const std::string sig1 = singleBoundarySignature(mesh);
    const bool exact = undoOk && sig0 == sig1;
    std::fprintf(stderr,
                 "[sc-boundary-test] ops=%d V %d->%d->%d F %d->%d->%d "
                 "closedBoundary=%d nonManifold=%d dup=%d nonTri=%d "
                 "afterUndo boundary=%d nonManifold=%d dup=%d nonTri=%d exact=%d\n",
                 applied, V0, V1, liveV(), F0, F1, mesh->size_of_facets(),
                 boundaryEdges, nonManifold, dupFaces, nonTri,
                 boundaryEdges2, nonManifold2, dupFaces2, nonTri2, exact ? 1 : 0);
    return exact ? 0 : 1;
}

}  // namespace pmsf
