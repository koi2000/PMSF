#pragma once

/// Serialization/interim inverse for the single-copy group-boundary collapse
/// (D183/D185).  Prototype stores explicit quantized positions for the local
/// patch faces (no seam duplication, positions are still stored once globally
/// in the mesh; this patch stream is an interim correctness vehicle before the
/// compact anchor/traversal encoding of D184).
///
/// Integer format per operation:
///   removedQx, removedQy, removedQz, removedGid,
///   oldN,   oldN × [gid, q0x,q0y,q0z, q1x,q1y,q1z, q2x,q2y,q2z],
///   newN,   newN × [gid, q0..., q1..., q2...]
///
/// All coordinates are mesh->floatPosToInt() values.  The decoder builds a
/// unique position -> live-vertex map in the current mesh state; duplicate
/// quantized positions are rejected (the final architecture will use explicit
/// ids instead, see D184).

#include "SingleBoundaryCollapse.h"

#include <array>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace pmsf {

struct PosKey3 {
    int x = 0, y = 0, z = 0;
    bool operator<(const PosKey3& o) const {
        return std::tie(x, y, z) < std::tie(o.x, o.y, o.z);
    }
};

inline PosKey3 sbPosKey(MCGAL::Mesh* mesh, MCGAL::Vertex* v) {
    const auto q = mesh->floatPosToInt(v->point());
    return {q.x(), q.y(), q.z()};
}

inline void sbAppendPos(std::vector<int>& out, const PosKey3& q) {
    out.push_back(q.x);
    out.push_back(q.y);
    out.push_back(q.z);
}

inline void sbAppendFace(std::vector<int>& out, MCGAL::Mesh* mesh,
                         int gid, const SingleBoundaryFace& rec) {
    out.push_back(gid);
    for (int k = 0; k < 3; ++k) {
        if (!rec.v[k]) return;  // invalid face -> caller checks size
        sbAppendPos(out, sbPosKey(mesh, rec.v[k]));
    }
}

inline std::vector<int> captureSingleBoundaryPatchPositional(MCGAL::Mesh* mesh,
                                                             const SingleBoundaryPatch& patch) {
    std::vector<int> out;
    if (!mesh || !patch.valid || !patch.removed) return out;
    sbAppendPos(out, sbPosKey(mesh, patch.removed));
    out.push_back(patch.gid);
    out.push_back(static_cast<int>(patch.origFaces.size()));
    for (const auto& rec : patch.origFaces) sbAppendFace(out, mesh, rec.gid, rec);
    out.push_back(static_cast<int>(patch.addedFaces.size()));
    for (const auto& rec : patch.addedFaces) sbAppendFace(out, mesh, rec.gid, rec);
    return out;
}

namespace sbpos {

inline MCGAL::Vertex* findVertexByPos(MCGAL::Mesh* mesh,
                                      const std::map<PosKey3, MCGAL::Vertex*>& lookup,
                                      const PosKey3& q) {
    auto it = lookup.find(q);
    return it == lookup.end() ? nullptr : it->second;
}

inline MCGAL::Facet* findFaceByTriple(MCGAL::Mesh* mesh, MCGAL::Vertex* a,
                                      MCGAL::Vertex* b, MCGAL::Vertex* c, int gid) {
    if (!a || !b || !c) return nullptr;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge() || f->groupId() != gid) continue;
        const auto vs = sbdetail::faceVertices(f);
        if (!vs[0]) continue;
        auto has = [&](MCGAL::Vertex* x) {
            return vs[0] == x || vs[1] == x || vs[2] == x;
        };
        if (has(a) && has(b) && has(c)) return f;
    }
    return nullptr;
}

}  // namespace sbpos

/// Undo one positional patch on the current (collapsed) mesh state.
using SBCanonicalPosMap = std::unordered_map<MCGAL::Vertex*, PosKey3>;

inline bool applySingleBoundaryPatchInversePositional(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                                     const std::vector<int>& data,
                                                     std::size_t& cursor,
                                                     SBCanonicalPosMap* canonicalPos = nullptr) {
    if (!mesh || cursor + 4 > data.size()) return false;
    auto fail = [&](const char* why) -> bool {
        if (std::getenv("PMSF_SC_DEBUG"))
            std::fprintf(stderr, "[sc-inverse-fail] %s cursor=%zu\n", why, cursor);
        return false;
    };
    PosKey3 removedQ{data[cursor], data[cursor + 1], data[cursor + 2]};
    cursor += 3;
    const int removedGid = data[cursor++];
    if (cursor > data.size()) return false;

    std::map<PosKey3, MCGAL::Vertex*> lookup;
    bool dupPos = false;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        PosKey3 q = sbPosKey(mesh.get(), v);
        if (canonicalPos) {
            auto it = canonicalPos->find(v);
            if (it != canonicalPos->end()) q = it->second;
        }
        if (!lookup.emplace(q, v).second) dupPos = true;
    }
    if (dupPos) return fail("duplicate quantized position");

    auto readFace = [&](int gid, MCGAL::Vertex*& a, MCGAL::Vertex*& b, MCGAL::Vertex*& c) -> bool {
        if (cursor + 9 > data.size()) return false;
        PosKey3 q0{data[cursor], data[cursor + 1], data[cursor + 2]};
        PosKey3 q1{data[cursor + 3], data[cursor + 4], data[cursor + 5]};
        PosKey3 q2{data[cursor + 6], data[cursor + 7], data[cursor + 8]};
        cursor += 9;
        a = sbpos::findVertexByPos(mesh.get(), lookup, q0);
        b = sbpos::findVertexByPos(mesh.get(), lookup, q1);
        c = sbpos::findVertexByPos(mesh.get(), lookup, q2);
        (void)gid;
        return a && b && c;
    };

    if (cursor + 1 > data.size()) return false;
    const int oldN = data[cursor++];
    // remove the fan faces created by the collapse (addedFaces follow oldFaces)
    if (oldN < 0 || cursor + static_cast<std::size_t>(oldN) * 10 > data.size()) return false;
    std::size_t skip = cursor;
    cursor += static_cast<std::size_t>(oldN) * 10;
    if (cursor + 1 > data.size()) return false;
    const int newN = data[cursor++];
    if (newN < 0 || cursor + static_cast<std::size_t>(newN) * 10 > data.size()) return false;
    std::vector<std::array<MCGAL::Vertex*, 3>> added;
    std::vector<int> addedGid;
    for (int i = 0; i < newN; ++i) {
        const int gid = data[cursor++];
        MCGAL::Vertex *a = nullptr, *b = nullptr, *c = nullptr;
        if (!readFace(gid, a, b, c)) return false;
        added.push_back({a, b, c});
        addedGid.push_back(gid);
    }
    for (std::size_t i = 0; i < added.size(); ++i) {
        MCGAL::Facet* f = sbpos::findFaceByTriple(mesh.get(), added[i][0], added[i][1],
                                                  added[i][2], addedGid[i]);
        if (!f) return fail("added fan face not found");
        removeFaceForPatch(mesh, f);
    }

    // re-create the removed vertex at its stored quantized position
    MCGAL::Vertex* v = MCGAL::contextPool.allocateVertexFromPool(
        mesh->meshId(), mesh->intPosToFloat(MCGAL::PointInt(removedQ.x, removedQ.y, removedQ.z)));
    if (!v) return fail("vertex alloc failed");
    v->halfedges().clear();
    v->setGroupId(removedGid);
    mesh->vertices().push_back(v);
    lookup[removedQ] = v;
    if (canonicalPos) (*canonicalPos)[v] = removedQ;

    // restore original fan faces (stored in oriented order)
    std::size_t oldCursor = skip;
    for (int i = 0; i < oldN; ++i) {
        const int gid = data[oldCursor];
        cursor = oldCursor + 1;
        MCGAL::Vertex *a = nullptr, *b = nullptr, *c = nullptr;
        if (!readFace(gid, a, b, c)) return fail("old fan vertex missing");
        oldCursor += 10;
        if (!createFaceFromTripleManual(mesh, a, b, c, gid)) return fail("old fan face create failed");
    }
    // fix cursor to the end of the patch
    cursor = skip + static_cast<std::size_t>(oldN) * 10 + 1 +
             static_cast<std::size_t>(newN) * 10;
    pmsf::rebuildVertexHalfedgeRings(mesh);
    return true;
}

/// Exactness signature for the positional prototype: vertex/face identities are
/// checked via quantized positions (the interim patch format uses positions as
/// ids), so a recreated vertex with a new poolId still compares equal.
inline std::string singleBoundaryPositionalSignature(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!mesh) return "";
    std::vector<PosKey3> verts;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (v && !v->isRemoved()) verts.push_back(sbPosKey(mesh.get(), v));
    }
    std::sort(verts.begin(), verts.end());
    std::vector<std::array<int, 10>> faces;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        const auto vs = sbdetail::faceVertices(f);
        if (!vs[0]) continue;
        PosKey3 p[3] = {sbPosKey(mesh.get(), vs[0]), sbPosKey(mesh.get(), vs[1]),
                        sbPosKey(mesh.get(), vs[2])};
        std::sort(p, p + 3);
        faces.push_back({p[0].x, p[0].y, p[0].z, p[1].x, p[1].y, p[1].z,
                         p[2].x, p[2].y, p[2].z, f->groupId()});
    }
    std::sort(faces.begin(), faces.end());
    std::string sig = "PV" + std::to_string(verts.size()) + ":";
    for (const auto& v : verts)
        sig += std::to_string(v.x) + "," + std::to_string(v.y) + "," + std::to_string(v.z) + ";";
    sig += "|PF" + std::to_string(faces.size()) + ":";
    for (const auto& f : faces) {
        for (int k = 0; k < 10; ++k) sig += std::to_string(f[k]) + ",";
        sig += ";";
    }
    return sig;
}

/// Encoder stage: apply up to maxOps single-copy group-boundary collapses at the
/// end of `roundIndex`, appending positional patch ints (each prefixed with its
/// own length) to `roundData`.  `protectedVerts` (anchors/seeds) are skipped.
inline int runSingleCopyBoundaryEncoderStage(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                             int roundIndex, int maxOps,
                                             const std::set<MCGAL::Vertex*>& protectedVerts,
                                             std::vector<int>& roundData) {
    if (!mesh || maxOps <= 0) return 0;
    int applied = 0;
    std::set<int> rejected;
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
            if (!v || rejected.count(v->poolId()) || protectedVerts.count(v)) continue;
            SingleBoundaryPatch patch;
            if (!collapseSingleGroupBoundaryVertex(mesh, v, patch)) {
                rejected.insert(v->poolId());
                continue;
            }
            if (!localSingleBoundaryPatchClosed(mesh, patch)) {
                restoreSingleBoundaryPatch(mesh, patch);
                rejected.insert(v->poolId());
                continue;
            }
            std::vector<int> data = captureSingleBoundaryPatchPositional(mesh.get(), patch);
            if (data.empty()) {
                restoreSingleBoundaryPatch(mesh, patch);
                return applied;
            }
            roundData.push_back(static_cast<int>(data.size()));
            roundData.insert(roundData.end(), data.begin(), data.end());
            ++applied;
            ok = true;
            break;
        }
        if (!ok) break;
    }
    return applied;
}

/// In-memory prototype of the *posit​ional patch* pipeline: apply up to maxOps
/// collapses, store each patch in the interim positional int format, then undo
/// in reverse using applySingleBoundaryPatchInversePositional and compare exact
/// signatures (V/F set by poolId).  This validates the future decoder path.
inline int runSingleBoundaryPositionalRoundTripTest(std::shared_ptr<MCGAL::Mesh> mesh, int maxOps) {
    if (!mesh) return 1;
    const std::string sig0 = singleBoundaryPositionalSignature(mesh);
    auto liveV = [&]() {
        int n = 0;
        for (MCGAL::Vertex* v : mesh->vertices()) if (v && !v->isRemoved()) ++n;
        return n;
    };
    const int V0 = liveV();
    const int F0 = mesh->size_of_facets();
    int baseBoundary = 0, baseNonManifold = 0, baseDup = 0, baseNonTri = 0;
    countSingleBoundaryManifold(mesh, baseBoundary, baseNonManifold, baseDup, baseNonTri);
    std::vector<std::vector<int>> patches;
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
            if (!localSingleBoundaryPatchClosed(mesh, patch)) {
                restoreSingleBoundaryPatch(mesh, patch);
                rejected.insert(v->poolId());
                continue;
            }
            std::vector<int> data = captureSingleBoundaryPatchPositional(mesh.get(), patch);
            if (data.empty()) {
                restoreSingleBoundaryPatch(mesh, patch);
                return 1;
            }
            patches.push_back(std::move(data));
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
        std::size_t cursor = 0;
        if (!applySingleBoundaryPatchInversePositional(mesh, *it, cursor) || cursor != it->size()) {
            undoOk = false;
            break;
        }
    }
    int boundaryEdges2 = 0, nonManifold2 = 0, dupFaces2 = 0, nonTri2 = 0;
    countSingleBoundaryManifold(mesh, boundaryEdges2, nonManifold2, dupFaces2, nonTri2);
    const std::string sig1 = singleBoundaryPositionalSignature(mesh);
    const bool exact = undoOk && sig0 == sig1;
    std::fprintf(stderr,
                 "[sc-boundary-pos-test] ops=%d V %d->%d->%d F %d->%d->%d "
                 "closedBoundary=%d nonManifold=%d dup=%d nonTri=%d "
                 "afterUndo boundary=%d nonManifold=%d dup=%d nonTri=%d exact=%d\n",
                 applied, V0, V1, liveV(), F0, F1, mesh->size_of_facets(),
                 boundaryEdges, nonManifold, dupFaces, nonTri,
                 boundaryEdges2, nonManifold2, dupFaces2, nonTri2, exact ? 1 : 0);
    return exact ? 0 : 1;
}

}  // namespace pmsf
