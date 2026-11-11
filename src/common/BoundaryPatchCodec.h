#pragma once

#include "MeshSplitUtils.h"
#include "postprocess/EdgeFlipCandidateBfs.h"
#include <array>
#include "Mesh.h"
#include "Vertex.h"
#include "Halfedge.h"
#include "Facet.h"

namespace pmsf {

struct BoundaryCollapsePatch {
    MCGAL::Point removed;
    MCGAL::Point a, b, c;
    MCGAL::Point f0ProxyA, f0ProxyB;
    MCGAL::Point f1ProxyA, f1ProxyB;
    int g0 = -1;
    int g1 = -1;
    bool valid = false;
};

inline std::array<int, 3> patchInt(const std::shared_ptr<MCGAL::Mesh>& mesh, const MCGAL::Point& p) {
    const MCGAL::PointInt q = mesh->floatPosToInt(p);
    return {q.x(), q.y(), q.z()};
}

inline MCGAL::Point patchPoint(const std::shared_ptr<MCGAL::Mesh>& mesh, const int* p) {
    return mesh->intPosToFloat(MCGAL::PointInt(p[0], p[1], p[2]));
}

inline bool captureBoundaryCollapse3(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                     MCGAL::Vertex* v,
                                     BoundaryCollapsePatch& patch) {
    if (!mesh || !v || v->isRemoved()) return false;
    MCGAL::Halfedge* hOut = nullptr;
    for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
        if (h && !h->isRemoved() && h->opposite() == nullptr && h->face()) { hOut = h; break; }
    }
    if (!hOut) return false;
    MCGAL::Halfedge* hbc = hOut->next();
    MCGAL::Halfedge* hcv = hbc ? hbc->next() : nullptr;
    MCGAL::Halfedge* hvc = hcv ? hcv->opposite() : nullptr;
    MCGAL::Halfedge* hca = hvc ? hvc->next() : nullptr;
    MCGAL::Halfedge* hIn = hca ? hca->next() : nullptr;
    if (!hbc || !hcv || !hvc || !hca || !hIn) return false;
    if (hcv->end_vertex() != v || hvc->vertex() != v) return false;
    MCGAL::Vertex* a = hIn->vertex();
    MCGAL::Vertex* b = hOut->end_vertex();
    MCGAL::Vertex* c = hcv->vertex();
    MCGAL::Facet* f0 = hIn->face();
    MCGAL::Facet* f1 = hOut->face();
    if (!a || !b || !c || !f0 || !f1 || f0 == f1) return false;

    patch.removed = v->point();
    patch.a = a->point(); patch.b = b->point(); patch.c = c->point();
    patch.g0 = f0->groupId(); patch.g1 = f1->groupId();
    MCGAL::Halfedge* pf0 = f0->proxyHalfedge();
    MCGAL::Halfedge* pf1 = f1->proxyHalfedge();
    if (!pf0 || !pf1) return false;
    patch.f0ProxyA = pf0->vertex()->point();
    patch.f0ProxyB = pf0->end_vertex()->point();
    patch.f1ProxyA = pf1->vertex()->point();
    patch.f1ProxyB = pf1->end_vertex()->point();
    patch.valid = true;
    return true;
}

inline bool applyBoundaryCollapsePatch(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                       const BoundaryCollapsePatch& patch) {
    if (!mesh || !patch.valid) return false;
    const auto edges = pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(mesh);
    const auto key = [&](const MCGAL::Point& p) { return patchInt(mesh, p); };
    const auto aKey = key(patch.a), bKey = key(patch.b);
    MCGAL::Halfedge* target = nullptr;
    for (MCGAL::Halfedge* h : edges) {
        if (!h || h->isRemoved() || h->opposite() != nullptr || !h->face()) continue;
        const auto ha = key(h->vertex()->point());
        const auto hb = key(h->end_vertex()->point());
        if ((ha == aKey && hb == bKey) || (ha == bKey && hb == aKey)) { target = h; break; }
    }
    if (!target) return false;
    MCGAL::Vertex* nv = nullptr;
    MCGAL::Halfedge* splitEdge = pmsf::splitBoundaryEdgeInsertVertex(mesh, target, patch.removed, &nv);
    if (!splitEdge || !nv) return false;
    nv->setVertexPoint(patch.removed);
    // f0 is the face reused by splitBoundaryEdgeInsertVertex, f1 is the newly allocated face.
    MCGAL::Facet* f0 = splitEdge->face();
    MCGAL::Facet* f1 = splitEdge->opposite() ? splitEdge->opposite()->face() : nullptr;
    // splitBoundaryEdgeInsertVertex creates the second face and returns av; find it by v ring.
    MCGAL::Facet* fNew = nullptr;
    for (MCGAL::Halfedge* h : nv->halfedges()) {
        if (h && h->face() && h->face() != f0) { fNew = h->face(); break; }
    }
    if (!f0 || !fNew) return false;
    f0->setGroupId(patch.g0);
    fNew->setGroupId(patch.g1);
    auto resetProxy = [&](MCGAL::Facet* f, const MCGAL::Point& pa, const MCGAL::Point& pb) {
        if (!f) return;
        const auto ka = patchInt(mesh, pa), kb = patchInt(mesh, pb);
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        if (!h0) return;
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && h->vertex() && h->end_vertex() &&
                patchInt(mesh, h->vertex()->point()) == ka &&
                patchInt(mesh, h->end_vertex()->point()) == kb) {
                f->reset(h);
                return;
            }
            h = h->next();
        } while (h && h != h0 && ++guard < 64);
    };
    resetProxy(f0, patch.f0ProxyA, patch.f0ProxyB);
    resetProxy(fNew, patch.f1ProxyA, patch.f1ProxyB);
    return true;
}


struct BoundaryFanPatch {
    MCGAL::Point removed;
    std::vector<std::array<MCGAL::Point, 3>> origFaces;
    std::vector<std::array<MCGAL::Point, 3>> postFaces;
    std::vector<int> origGroups;
    std::vector<std::pair<MCGAL::Point, MCGAL::Point>> boundaryEdges;
    bool valid = false;
};

inline bool captureBoundaryFanPatch(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                    MCGAL::Vertex* v,
                                    BoundaryFanPatch& patch) {
    if (!mesh || !v || v->isRemoved()) return false;
    patch.removed = v->point();
    patch.origFaces.clear();
    patch.origGroups.clear();
    patch.boundaryEdges.clear();
    std::unordered_set<int> seenFaces;
    for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
        if (!h || h->isRemoved()) continue;
        MCGAL::Facet* f = h->face();
        if (!f || f->isRemoved()) continue;
        if (!seenFaces.insert(f->poolId()).second) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        if (!h0) continue;
        std::array<MCGAL::Point, 3> tri;
        MCGAL::Halfedge* it = h0;
        int k = 0, guard = 0;
        do {
            if (k < 3) tri[k++] = it->vertex()->point();
            it = it->next();
        } while (it && it != h0 && ++guard < 64);
        if (k != 3) continue;
        patch.origFaces.push_back(tri);
        patch.origGroups.push_back(f->groupId());
    }
    // boundary edges incident to v (no opposite), both directions, via incident face rings
    std::set<std::array<int, 6>> seenBoundary;
    auto addBoundary = [&](MCGAL::Halfedge* h) {
        if (!h || h->isRemoved() || h->opposite() != nullptr || !h->face()) return;
        if (!(h->vertex() == v || h->end_vertex() == v)) return;
        const auto a = patchInt(mesh, h->vertex()->point());
        const auto b = patchInt(mesh, h->end_vertex()->point());
        auto lo = std::min(a, b), hi = std::max(a, b);
        std::array<int, 6> k{lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]};
        if (!seenBoundary.insert(k).second) return;
        patch.boundaryEdges.push_back({h->vertex()->point(), h->end_vertex()->point()});
    };
    for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
        if (!h || h->isRemoved()) continue;
        MCGAL::Facet* f = h->face();
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* it = h0;
        int guard = 0;
        do {
            addBoundary(it);
            it = it->next();
        } while (it && it != h0 && ++guard < 64);
    }
    patch.valid = !patch.origFaces.empty();
    return patch.valid;
}

inline void removeFaceForPatch(const std::shared_ptr<MCGAL::Mesh>& mesh, MCGAL::Facet* f) {
    if (!mesh || !f || f->isRemoved()) return;
    MCGAL::Halfedge* h0 = f->proxyHalfedge();
    if (h0) {
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            MCGAL::Halfedge* next = h->next();
            if (h->opposite()) h->opposite()->setOpposite(nullptr);
            if (h->vertex()) {
                auto& ring = h->vertex()->halfedges();
                ring.erase(std::remove(ring.begin(), ring.end(), h), ring.end());
            }
            h->setRemoved();
            h = next;
        } while (h && h != h0 && ++guard < 64);
    }
    f->setRemoved();
    mesh->eraseFacetByPointer(f);
}

inline bool applyBoundaryFanPatch(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                  const BoundaryFanPatch& patch) {
    if (!mesh || !patch.valid) return false;
    auto pointKey = [&](const MCGAL::Point& p) { return patchInt(mesh, p); };
    auto sameTri = [&](MCGAL::Facet* f, const std::array<MCGAL::Point, 3>& tri) {
        std::array<std::array<int,3>,3> got{}, want{};
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0; int k = 0, guard = 0;
        do {
            if (k < 3) got[k++] = patchInt(mesh, h->vertex()->point());
            h = h->next();
        } while (h && h != h0 && ++guard < 64);
        for (int i = 0; i < 3; ++i) want[i] = pointKey(tri[i]);
        std::sort(got.begin(), got.end());
        std::sort(want.begin(), want.end());
        return k == 3 && got == want;
    };
    // remove post-collapse local faces
    std::vector<MCGAL::Facet*> toRemove;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        for (const auto& tri : patch.postFaces) {
            if (sameTri(f, tri)) { toRemove.push_back(f); break; }
        }
    }
    for (MCGAL::Facet* f : toRemove) removeFaceForPatch(mesh, f);
    // create removed vertex
    const int gid0 = patch.origGroups.empty() ? 0 : patch.origGroups.front();
    MCGAL::Vertex* v = MCGAL::contextPool.allocateVertexFromPool(mesh->meshId(), patch.removed);
    if (!v) return false;
    v->halfedges().clear();
    v->setGroupId(gid0);
    mesh->vertices().push_back(v);
    // create original fan faces, choosing vertices in the face's own group so
    // duplicated seam copies are not mixed across groups
    std::map<int, std::map<std::array<int,3>, MCGAL::Vertex*>> lookups;
    for (MCGAL::Vertex* vt : mesh->vertices()) {
        if (vt && !vt->isRemoved()) lookups[vt->groupId()][patchInt(mesh, vt->point())] = vt;
    }
    lookups[gid0][patchInt(mesh, patch.removed)] = v;
    int gi = 0;
    for (const auto& tri : patch.origFaces) {
        const int gid = (gi < static_cast<int>(patch.origGroups.size())) ? patch.origGroups[gi] : gid0;
        auto& lookup = lookups[gid];
        std::vector<MCGAL::Vertex*> vs;
        for (const auto& p : tri) {
            auto it = lookup.find(pointKey(p));
            if (it == lookup.end()) return false;
            vs.push_back(it->second);
        }
        MCGAL::Facet* nf = mesh->add_face(vs);
        if (!nf) return false;
        nf->setGroupId(gid);
        ++gi;
    }
    auto& boundaryLookup = lookups[gid0];
    for (const auto& be : patch.boundaryEdges) {
        auto a = boundaryLookup.find(pointKey(be.first));
        auto b = boundaryLookup.find(pointKey(be.second));
        if (a == boundaryLookup.end() || b == boundaryLookup.end()) continue;
        MCGAL::Halfedge* h = a->second->find_halfedge(b->second);
        if (h) { h->setBoundary(); h->setOpposite(nullptr); }
    }
    return true;
}


inline void captureBoundaryPostFaces(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                     BoundaryFanPatch& patch) {
    patch.postFaces.clear();
    if (!mesh) return;
    std::set<std::array<int,3>> ring;
    for (const auto& tri : patch.origFaces)
        for (const auto& p : tri) ring.insert(patchInt(mesh, p));
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        if (!h0) continue;
        std::array<MCGAL::Point,3> tri;
        std::array<std::array<int,3>,3> keys;
        int k = 0, guard = 0;
        MCGAL::Halfedge* h = h0;
        do {
            if (k < 3) { tri[k] = h->vertex()->point(); keys[k] = patchInt(mesh, tri[k]); }
            ++k;
            h = h->next();
        } while (h && h != h0 && ++guard < 64);
        if (k != 3) continue;
        bool allIn = true;
        for (int i = 0; i < 3; ++i) if (!ring.count(keys[i])) { allIn = false; break; }
        if (allIn) patch.postFaces.push_back(tri);
    }
}


/// General boundary vertex removal: remove all incident faces and fill the star
/// with a deterministic fan triangulation from one boundary neighbour.  The fan
/// patch records original/post faces for the exact inverse.
inline bool removeBoundaryVertexFan(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                    MCGAL::Vertex* v,
                                    BoundaryFanPatch& patch,
                                    MCGAL::Vertex** outApex = nullptr) {
    if (!mesh || !v || v->isRemoved()) return false;
    auto keyOf = [&](const MCGAL::Point& p) { return patchInt(mesh, p); };
    auto vKey = keyOf(v->point());
    const int fanGid = patch.origGroups.empty() ? -1 : patch.origGroups.front();
    std::map<std::array<int,3>, MCGAL::Vertex*> byKey;
    std::map<std::array<int,3>, MCGAL::Vertex*> byKeyAny;
    for (MCGAL::Vertex* vt : mesh->vertices()) {
        if (!vt || vt->isRemoved()) continue;
        byKeyAny[keyOf(vt->point())] = vt;
        if (fanGid < 0 || vt->groupId() == fanGid) byKey[keyOf(vt->point())] = vt;
    }
    for (auto& kv : byKeyAny) byKey.emplace(kv.first, kv.second);
    std::map<std::array<int,3>, std::set<std::array<int,3>>> adj;
    std::vector<std::array<int,3>> ringKeys;
    int facesRemoved = 0;
    for (const auto& tri : patch.origFaces) {
        std::array<int,3> other[2]; int n = 0;
        for (const auto& p : tri) {
            const auto k = keyOf(p);
            if (k != vKey) other[n++] = k;
        }
        if (n != 2) return false;
        adj[other[0]].insert(other[1]);
        adj[other[1]].insert(other[0]);
    }
    // boundary neighbours: boundary edges incident to v (either direction)
    std::array<int,3> aKey{0,0,0}, bKey{0,0,0};
    bool haveA = false, haveB = false;
    for (MCGAL::Halfedge* h : pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(mesh)) {
        if (!h || h->isRemoved()) continue;
        MCGAL::Vertex* other = nullptr;
        if (h->vertex() == v) other = h->end_vertex();
        else if (h->end_vertex() == v) other = h->vertex();
        if (!other) continue;
        const auto k = keyOf(other->point());
        if (!haveA) { aKey = k; haveA = true; }
        else if (!haveB && k != aKey) { bKey = k; haveB = true; }
    }
    if (!haveA || !haveB) return false;
    // shortest path a -> b through link adjacency (should be the whole ring chain)
    std::map<std::array<int,3>, std::array<int,3>> parent;
    std::deque<std::array<int,3>> q;
    q.push_back(aKey);
    std::set<std::array<int,3>> seen{aKey};
    bool found = false;
    while (!q.empty()) {
        auto cur = q.front(); q.pop_front();
        if (cur == bKey) { found = true; break; }
        for (const auto& nx : adj[cur]) {
            if (seen.insert(nx).second) { parent[nx] = cur; q.push_back(nx); }
        }
    }
    if (!found) return false;
    std::vector<std::array<int,3>> path;
    for (auto cur = bKey;; cur = parent[cur]) {
        path.push_back(cur);
        if (cur == aKey) break;
        if (!parent.count(cur)) return false;
    }
    std::reverse(path.begin(), path.end());
    if (path.size() < 3) return false;
    for (const auto& k : path) {
        if (!byKey.count(k)) return false;
    }
    // collect and remove all incident faces
    std::unordered_set<MCGAL::Facet*> faces;
    for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
        if (h && h->face() && !h->face()->isRemoved()) faces.insert(h->face());
    }
    for (MCGAL::Facet* f : faces) removeFaceForPatch(mesh, f);
    v->halfedges().clear();
    v->setRemoved();
    // fill with fan from path[0]
    MCGAL::Vertex* a = byKey[path[0]];
    if (outApex) *outApex = a;
    const int gid = patch.origGroups.empty() ? 0 : patch.origGroups.front();
    patch.postFaces.clear();
    for (size_t i = 1; i + 1 < path.size(); ++i) {
        std::vector<MCGAL::Vertex*> vs{a, byKey[path[i]], byKey[path[i + 1]]};
        MCGAL::Facet* nf = MCGAL::contextPool.allocateFaceFromPool(mesh->meshId(), vs);
        if (!nf) return false;
        nf->setGroupId(gid);
        mesh->add_face(nf);
        patch.postFaces.push_back({vs[0]->point(), vs[1]->point(), vs[2]->point()});
    }
    // boundary flags: only the new closing edge should be boundary
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        if (!h0) continue;
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h->opposite() == nullptr) h->setBoundary();
            else { h->setNotBoundary(); h->opposite()->setNotBoundary(); }
            h = h->next();
        } while (h && h != h0 && ++guard < 64);
    }
    return !patch.postFaces.empty();
}



inline void rebuildVertexHalfedgeRings(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!mesh) return;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (v) v->halfedges().clear();
    }
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && h->vertex()) h->vertex()->halfedges().push_back(h);
            h = h->next();
        } while (h && h != h0 && ++guard < 64);
    }
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        auto& hs = v->halfedges();
        if (hs.empty()) continue;
        MCGAL::Halfedge* proxy = v->proxyHalfedge();
        if (!proxy || proxy->isRemoved()) v->setProxyHalfedge(hs.front());
    }
}

inline void relinkMeshByEndpoints(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!mesh) return;
    auto keyOf = [](MCGAL::Halfedge* h) {
        auto a = reinterpret_cast<std::uintptr_t>(h->vertex());
        auto b = reinterpret_cast<std::uintptr_t>(h->end_vertex());
        if (a > b) std::swap(a, b);
        return std::pair<std::uintptr_t, std::uintptr_t>{a, b};
    };
    std::map<std::pair<std::uintptr_t, std::uintptr_t>, std::vector<MCGAL::Halfedge*>> buckets;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && !h->isRemoved()) {
                h->setOpposite(nullptr);
                buckets[keyOf(h)].push_back(h);
            }
            h = h->next();
        } while (h && h != h0 && ++guard < 64);
    }
    for (auto& kv : buckets) {
        auto& v = kv.second;
        for (size_t i = 0; i + 1 < v.size(); i += 2) {
            v[i]->setOpposite(v[i + 1]);
            v[i + 1]->setOpposite(v[i]);
        }
    }
}


/// Remove later copies of identical vertex triples (same three vertices).
/// Elimination patch triangulation can leave two live copies of one triangle,
/// which makes the serialized base mesh non-manifold (an edge used by 4 faces).
/// Encoder and decoder must apply this at the same round boundary so their
/// states stay in sync.
inline int dedupIdenticalFaces(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!mesh) return 0;
    std::map<std::array<MCGAL::Vertex*, 3>, MCGAL::Facet*> seen;
    std::vector<MCGAL::Facet*> victims;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Vertex* vs[3];
        int n = 0;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && h->vertex() && n < 3) vs[n++] = h->vertex();
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
        if (n != 3) continue;
        std::array<MCGAL::Vertex*, 3> key{vs[0], vs[1], vs[2]};
        std::sort(key.begin(), key.end());
        auto it = seen.find(key);
        if (it == seen.end()) {
            seen[key] = f;
        } else {
            victims.push_back(f);
        }
    }
    for (MCGAL::Facet* f : victims) removeFaceForPatch(mesh, f);
    return static_cast<int>(victims.size());
}

inline MCGAL::Facet* createFaceFromTripleManual(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                                MCGAL::Vertex* v0,
                                                MCGAL::Vertex* v1,
                                                MCGAL::Vertex* v2,
                                                int gid);

inline MCGAL::Vertex* restoreBoundaryCollapseViaFaces(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                                     MCGAL::Halfedge* boundaryEdge,
                                                     const MCGAL::Point& pos,
                                                     int gid) {
    if (!mesh || !boundaryEdge || boundaryEdge->isRemoved()) return nullptr;
    if (boundaryEdge->opposite() != nullptr || !boundaryEdge->face()) return nullptr;
    MCGAL::Halfedge* h = boundaryEdge;
    MCGAL::Vertex* a = h->vertex();
    MCGAL::Vertex* b = h->end_vertex();
    MCGAL::Halfedge* hbc = h->next();
    MCGAL::Halfedge* hca = hbc ? hbc->next() : nullptr;
    if (!a || !b || !hbc || !hca) return nullptr;
    MCGAL::Vertex* c = hbc->end_vertex();
    MCGAL::Facet* f = h->face();
    if (!c || !f || c == a || c == b) return nullptr;
    const int g = gid >= 0 ? gid : f->groupId();
    removeFaceForPatch(mesh, f);
    MCGAL::Vertex* v = MCGAL::contextPool.allocateVertexFromPool(mesh->meshId(), pos);
    if (!v) return nullptr;
    v->halfedges().clear();
    v->setGroupId(g);
    mesh->vertices().push_back(v);
    if (!createFaceFromTripleManual(mesh, a, v, c, g)) return nullptr;
    if (!createFaceFromTripleManual(mesh, v, b, c, g)) return nullptr;
    rebuildVertexHalfedgeRings(mesh);
    return v;
}

/// Create one triangle face without relying on Facet::reset(vertex list).
/// The mesh may contain removed faces whose half-edge opposite pointers are
/// stale; therefore the standard allocation path can crash while walking a
/// vertex ring.  This helper links the three new half-edges directly against
/// live faces and then rebuilds the vertex rings.
inline MCGAL::Facet* createFaceFromTripleManual(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                                MCGAL::Vertex* v0,
                                                MCGAL::Vertex* v1,
                                                MCGAL::Vertex* v2,
                                                int gid) {
    if (!mesh || !v0 || !v1 || !v2 || v0 == v1 || v1 == v2 || v2 == v0) return nullptr;
    auto alloc = [&](MCGAL::Vertex* a, MCGAL::Vertex* b) {
        return MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(mesh->meshId(), a, b);
    };
    // Skip if an identical live triangle already exists (can happen when the two
    // group-side fan fills meet along a degenerate ring path).  This full-face scan
    // is O(F) per created face; K-free callers already maintain a face-triple map
    // and validate via writer verification, so they can opt out.
    if (std::getenv("PMSF_SC_FAST_CREATE_FACE") == nullptr) {
    for (MCGAL::Facet* ff : mesh->faces()) {
        if (!ff || ff->isRemoved() || !ff->proxyHalfedge()) continue;
        MCGAL::Vertex* fv[3];
        int n = 0;
        MCGAL::Halfedge* g0 = ff->proxyHalfedge();
        MCGAL::Halfedge* g = g0;
        int guard = 0;
        do {
            if (g && g->vertex() && n < 3) fv[n++] = g->vertex();
            g = g ? g->next() : nullptr;
        } while (g && g != g0 && ++guard < 64);
        if (n != 3) continue;
        auto sameTriple = [&](MCGAL::Vertex* a, MCGAL::Vertex* b, MCGAL::Vertex* c) {
            return (a == v0 || a == v1 || a == v2) && (b == v0 || b == v1 || b == v2) &&
                   (c == v0 || c == v1 || c == v2) &&
                   !(a == b || b == c || a == c) &&
                   (a == v0 || b == v0 || c == v0) && (a == v1 || b == v1 || c == v1) &&
                   (a == v2 || b == v2 || c == v2);
        };
        if (sameTriple(fv[0], fv[1], fv[2])) return ff;
    }
    }
    MCGAL::Halfedge* h0 = alloc(v0, v1);
    MCGAL::Halfedge* h1 = alloc(v1, v2);
    MCGAL::Halfedge* h2 = alloc(v2, v0);
    if (!h0 || !h1 || !h2) return nullptr;
    h0->setNext(h1);
    h1->setNext(h2);
    h2->setNext(h0);
    MCGAL::Facet* f = MCGAL::contextPool.allocateFaceFromPool(mesh->meshId());
    if (!f) return nullptr;
    for (MCGAL::Halfedge* h : {h0, h1, h2}) h->setFace(f);
    f->reset(h0);
    f->setGroupId(gid);
    mesh->faces().push_back(f);
    auto linkOpposite = [&](MCGAL::Halfedge* nh) {
        MCGAL::Vertex* a = nh->vertex();
        MCGAL::Vertex* b = nh->end_vertex();
        MCGAL::Halfedge* opp = nullptr;
        auto freeOpposite = [](MCGAL::Halfedge* go) {
            return !go || go->isRemoved() || !go->face() ||
                   (go->face() && go->face()->isRemoved());
        };
        // Fast path: the opposite halfedge must be outgoing from b, so a vertex-ring
        // scan is enough.  This turns the fan retriangulation cost from O(F) per new
        // edge into O(deg) and keeps large-base stages practical.
        if (b) {
            for (MCGAL::Halfedge* hh : b->halfedges()) {
                if (!hh || hh->isRemoved() || hh->vertex() != b || hh->end_vertex() != a)
                    continue;
                if (!hh->face() || hh->face()->isRemoved()) continue;
                if (!freeOpposite(hh->opposite())) continue;
                opp = hh;
                break;
            }
        }
        for (MCGAL::Facet* ff : mesh->faces()) {
            if (opp) break;
            if (!ff || ff->isRemoved() || !ff->proxyHalfedge()) continue;
            MCGAL::Halfedge* g0 = ff->proxyHalfedge();
            MCGAL::Halfedge* g = g0;
            int guard = 0;
            do {
                if (g && !g->isRemoved() && g->vertex() == b && g->end_vertex() == a) {
                    MCGAL::Halfedge* go = g->opposite();
                    const bool oppFree = !go || go->isRemoved() || !go->face() ||
                                         (go->face() && go->face()->isRemoved());
                    if (oppFree) { opp = g; break; }
                }
                g = g ? g->next() : nullptr;
            } while (g && g != g0 && ++guard < 64);
            if (opp) break;
        }
        if (opp) {
            nh->setOpposite(opp);
            opp->setOpposite(nh);
        } else {
            nh->setOpposite(nullptr);
            nh->setBoundary();
        }
    };
    linkOpposite(h0);
    linkOpposite(h1);
    linkOpposite(h2);
    rebuildVertexHalfedgeRings(mesh);
    return f;
}


/// Shared-mesh (no seam-cut) boundary vertex removal across a group interface.
/// Removes all faces around v and fills each of the two group-side polygons with
/// a fan, so no splittable/removedVertexPos metadata is created.  The ring data
/// returned is enough for the paired custom inverse in mode23.
inline bool removeSharedGroupBoundaryVertexManual(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                                  MCGAL::Vertex* v,
                                                  int& outD,
                                                  MCGAL::Point& outRemoved,
                                                  std::vector<MCGAL::Halfedge*>& ringHalfedges,
                                                  std::vector<MCGAL::Vertex*>& ringVerts,
                                                  std::vector<int>& ringGroups) {
    if (!mesh || !v || v->isRemoved()) return false;
    rebuildVertexHalfedgeRings(mesh);
    ringHalfedges.clear();
    ringVerts.clear();
    ringGroups.clear();
    for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
        if (!h || h->isRemoved() || !h->face() || h->face()->isRemoved() ||
            h->vertex() != v || !h->end_vertex() || h->end_vertex()->isRemoved()) {
            return false;
        }
        ringHalfedges.push_back(h);
        ringVerts.push_back(h->end_vertex());
        ringGroups.push_back(h->face()->groupId());
    }
    const int d = static_cast<int>(ringHalfedges.size());
    outD = d;
    if (d < 4 || d > 12) return false;
    std::set<MCGAL::Vertex*> ringSet(ringVerts.begin(), ringVerts.end());
    if (static_cast<int>(ringSet.size()) != d) return false;
    std::vector<int> transitions;
    for (int i = 0; i < d; ++i) {
        if (ringGroups[static_cast<size_t>((i + d - 1) % d)] != ringGroups[static_cast<size_t>(i)]) {
            transitions.push_back(i);
        }
    }
    if (transitions.size() < 2) return false;
    outRemoved = v->point();
    std::set<MCGAL::Facet*> fanFaces;
    for (MCGAL::Halfedge* h : ringHalfedges) {
        if (h && h->face() && !h->face()->isRemoved()) fanFaces.insert(h->face());
    }
    for (MCGAL::Facet* f : fanFaces) removeFaceForPatch(mesh, f);
    v->halfedges().clear();
    v->setRemoved();
    rebuildVertexHalfedgeRings(mesh);
    auto buildSide = [&](int start, int end, int gid) -> bool {
        std::vector<MCGAL::Vertex*> path;
        int cur = start;
        while (true) {
            path.push_back(ringVerts[static_cast<size_t>(cur)]);
            if (cur == end) break;
            cur = (cur + 1) % d;
        }
        if (path.size() < 3) return true;
        for (size_t j = 1; j + 1 < path.size(); ++j) {
            if (!createFaceFromTripleManual(mesh, path[0], path[j], path[j + 1], gid))
                return false;
        }
        return true;
    };
    const int runCount = static_cast<int>(transitions.size());
    for (int ri = 0; ri < runCount; ++ri) {
        const int start = transitions[static_cast<size_t>(ri)];
        const int end = transitions[static_cast<size_t>((ri + 1) % runCount)];
        const int gid = std::getenv("PMSF_SHARED_BOUNDARY_PATCH_SINGLE_GROUP")
                            ? ringGroups[0]
                            : ringGroups[static_cast<size_t>(start)];
        if (!buildSide(start, end, gid)) return false;
    }
    return true;
}


/// Inverse of removeSharedGroupBoundaryVertexManual: remove the ring-subset fill
/// faces, insert v at vPos, and rebuild the fan with the recorded ring groups.
inline bool restoreSharedGroupBoundaryVertexManual(
    const std::shared_ptr<MCGAL::Mesh>& mesh,
    int d,
    const MCGAL::PointInt& vPos,
    const std::vector<MCGAL::PointInt>& ringQ,
    const std::vector<int>& ringG) {
    if (!mesh || d < 4 || static_cast<int>(ringQ.size()) != d || static_cast<int>(ringG.size()) != d) return false;
    std::vector<MCGAL::Vertex*> ringV(static_cast<size_t>(d), nullptr);
    int ringTol = 4;
    if (const char* rt = std::getenv("PMSF_RESTORE_RING_TOL"))
        ringTol = std::max(0, std::atoi(rt));
    for (int i = 0; i < d; ++i) {
        MCGAL::Vertex* best = nullptr;
        int bestDist = ringTol + 1;
        for (MCGAL::Vertex* v : mesh->vertices()) {
            if (!v || v->isRemoved()) continue;
            auto q = mesh->floatPosToInt(v->point());
            const int dist = std::abs(q.x() - ringQ[static_cast<size_t>(i)].x()) +
                             std::abs(q.y() - ringQ[static_cast<size_t>(i)].y()) +
                             std::abs(q.z() - ringQ[static_cast<size_t>(i)].z());
            if (dist < bestDist) { bestDist = dist; best = v; }
            if (dist == 0) break;
        }
        ringV[static_cast<size_t>(i)] = best;
        if (std::getenv("PMSF_BOUNDARY_DEBUG") && bestDist > 0) {
            std::fprintf(stderr, "[restore-ring] i=%d dist=%d want=%d,%d,%d d=%d\n", i, bestDist,
                         ringQ[static_cast<size_t>(i)].x(), ringQ[static_cast<size_t>(i)].y(),
                         ringQ[static_cast<size_t>(i)].z(), d);
        }
    }
    for (auto* v : ringV) if (!v) {
        if (std::getenv("PMSF_BOUNDARY_DEBUG")) {
            int missing = 0;
            for (int i = 0; i < d; ++i)
                if (!ringV[static_cast<std::size_t>(i)]) {
                    ++missing;
                    std::fprintf(stderr, "[restore-split] missing ring %d q=%d,%d,%d d=%d\n",
                                 i, ringQ[static_cast<std::size_t>(i)].x(),
                                 ringQ[static_cast<std::size_t>(i)].y(),
                                 ringQ[static_cast<std::size_t>(i)].z(), d);
                }
            std::fprintf(stderr, "[restore-split] missing=%d\n", missing);
        }
        return false;
    }
    std::set<MCGAL::Vertex*> ringSet(ringV.begin(), ringV.end());
    std::vector<MCGAL::Facet*> fillFaces;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Vertex* fv[3]; int n = 0;
        MCGAL::Halfedge* h0 = f->proxyHalfedge(); MCGAL::Halfedge* hh = h0; int guard = 0;
        do { if (hh && hh->vertex() && n < 3) fv[n++] = hh->vertex(); hh = hh->next(); } while (hh && hh != h0 && ++guard < 64);
        if (n != 3) continue;
        if (ringSet.count(fv[0]) && ringSet.count(fv[1]) && ringSet.count(fv[2])) fillFaces.push_back(f);
    }
    for (MCGAL::Facet* f : fillFaces) {
        if (f && !f->isRemoved()) removeFaceForPatch(mesh, f);
    }
    MCGAL::Vertex* nv = MCGAL::contextPool.allocateVertexFromPool(mesh->meshId(), mesh->intPosToFloat(vPos));
    if (!nv) return false;
    nv->halfedges().clear();
    nv->setGroupId(ringG[0]);
    mesh->vertices().push_back(nv);
    for (int i = 0; i < d; ++i) {
        if (!createFaceFromTripleManual(mesh, nv, ringV[static_cast<size_t>(i)],
                                        ringV[static_cast<size_t>((i + 1) % d)],
                                        ringG[static_cast<size_t>(i)]))
            return false;
    }
    return true;
}

/// Manual safe complex-fan removal for boundary vertex v (any k>=2).
/// Uses a full face scan (not boundedRing) so incomplete vertex rings cannot
/// corrupt the surgery.  Fills the star with a fan from boundary neighbour a.
inline bool removeBoundaryVertexFanManual(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                          MCGAL::Vertex* v,
                                          int& outK,
                                          MCGAL::Point& outRemoved,
                                          MCGAL::Point& outA,
                                          MCGAL::Point& outB,
                                          MCGAL::Halfedge** outBoundaryEdge,
                                          MCGAL::Point* outApexPos = nullptr,
                                          std::vector<int>* outFaceIdx = nullptr,
                                          std::vector<int>* outFacePid = nullptr,
                                          int* outVIdx = nullptr,
                                          int* outVPid = nullptr) {
    if (!mesh || !v || v->isRemoved()) return false;
    rebuildVertexHalfedgeRings(mesh);
    auto keyOf = [&](MCGAL::Vertex* vt) {
        const MCGAL::PointInt q = mesh->floatPosToInt(vt->point());
        return std::array<int, 3>{q.x(), q.y(), q.z()};
    };
    std::vector<MCGAL::Facet*> fan;
    std::vector<std::array<MCGAL::Vertex*, 2>> linkEdges;
    int gid = -1;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Vertex* tri[3];
        int n = 0;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && h->vertex() && n < 3) tri[n++] = h->vertex();
            h = h->next();
        } while (h && h != h0 && ++guard < 64);
        if (n != 3) continue;
        bool hasV = (tri[0] == v || tri[1] == v || tri[2] == v);
        if (!hasV) continue;
        fan.push_back(f);
        if (gid < 0) gid = f->groupId();
        MCGAL::Vertex* o[2]; int m = 0;
        for (int i = 0; i < 3; ++i) if (tri[i] != v) o[m++] = tri[i];
        if (m == 2) linkEdges.push_back({o[0], o[1]});
    }
    outK = static_cast<int>(fan.size());
    if (outK < 2) return false;
    outRemoved = v->point();
    // boundary neighbours a,b: boundary edges incident to v
    std::vector<MCGAL::Vertex*> bnd;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && !h->isRemoved() && h->opposite() == nullptr &&
                (h->vertex() == v || h->end_vertex() == v)) {
                MCGAL::Vertex* other = (h->vertex() == v) ? h->end_vertex() : h->vertex();
                if (other && other != v) bnd.push_back(other);
            }
            h = h->next();
        } while (h && h != h0 && ++guard < 64);
    }
    std::sort(bnd.begin(), bnd.end());
    bnd.erase(std::unique(bnd.begin(), bnd.end()), bnd.end());
    if (bnd.size() != 2) return false;
    MCGAL::Vertex* a = bnd[0];
    MCGAL::Vertex* b = bnd[1];
    outA = a->point();
    outB = b->point();
    // path a -> b through link edges
    std::map<MCGAL::Vertex*, std::vector<MCGAL::Vertex*>> adj;
    std::set<std::pair<MCGAL::Vertex*, MCGAL::Vertex*>> seenEdge;
    for (auto& e : linkEdges) {
        auto k1 = std::make_pair(e[0], e[1]);
        auto k2 = std::make_pair(e[1], e[0]);
        if (seenEdge.count(k1) || seenEdge.count(k2)) return false;
        seenEdge.insert(k1); seenEdge.insert(k2);
        adj[e[0]].push_back(e[1]); adj[e[1]].push_back(e[0]);
    }
    if (static_cast<int>(adj.size()) != outK + 1) return false;
    {
        int deg1 = 0;
        for (auto& kv : adj) {
            const int d = static_cast<int>(kv.second.size());
            if (d == 1) ++deg1;
            else if (d != 2) return false;
        }
        if (deg1 != 2) return false;
    }
    std::map<MCGAL::Vertex*, MCGAL::Vertex*> parent;
    std::deque<MCGAL::Vertex*> q;
    q.push_back(a);
    std::set<MCGAL::Vertex*> seen{a};
    while (!q.empty()) {
        MCGAL::Vertex* cur = q.front(); q.pop_front();
        for (MCGAL::Vertex* nx : adj[cur]) {
            if (seen.insert(nx).second) { parent[nx] = cur; q.push_back(nx); }
        }
    }
    if (!seen.count(b)) return false;
    std::vector<MCGAL::Vertex*> path;
    for (MCGAL::Vertex* cur = b;; cur = parent[cur]) {
        path.push_back(cur);
        if (cur == a) break;
        if (!parent.count(cur)) return false;
    }
    std::reverse(path.begin(), path.end());
    if (static_cast<int>(path.size()) != outK + 1) return false;
    if (outVIdx) {
        *outVIdx = -1;
        for (size_t vi = 0; vi < mesh->vertices().size(); ++vi)
            if (mesh->vertices()[vi] == v) { *outVIdx = static_cast<int>(vi); break; }
    }
    if (outVPid) *outVPid = v->poolId();
    if (outFaceIdx || outFacePid) {
        if (outFaceIdx) outFaceIdx->clear();
        if (outFacePid) outFacePid->clear();
        for (int i = 0; i < outK; ++i) {
            MCGAL::Vertex* x = path[static_cast<size_t>(i)];
            MCGAL::Vertex* y = path[static_cast<size_t>(i + 1)];
            MCGAL::Facet* match = nullptr;
            for (MCGAL::Facet* f : fan) {
                bool hv = false, hx = false, hy = false;
                MCGAL::Halfedge* h0 = f->proxyHalfedge();
                MCGAL::Halfedge* h = h0; int g = 0;
                do { if (h && h->vertex() == v) hv = true; if (h && h->vertex() == x) hx = true; if (h && h->vertex() == y) hy = true; h = h->next(); } while (h && h != h0 && ++g < 64);
                if (hv && hx && hy) { match = f; break; }
            }
            int idx = -1;
            if (match) for (size_t fi = 0; fi < mesh->faces().size(); ++fi) if (mesh->faces()[fi] == match) { idx = static_cast<int>(fi); break; }
            if (outFaceIdx) outFaceIdx->push_back(idx);
            if (outFacePid) outFacePid->push_back(match ? match->poolId() : -1);
        }
    }
    for (MCGAL::Facet* f : fan) removeFaceForPatch(mesh, f);
    v->halfedges().clear();
    v->setRemoved();
    std::vector<MCGAL::Facet*> newFill;
    for (size_t i = 1; i + 1 < path.size(); ++i) {
        std::vector<MCGAL::Vertex*> vs{a, path[i], path[i + 1]};
        MCGAL::Facet* nf = MCGAL::contextPool.allocateFaceFromPool(mesh->meshId(), vs);
        if (!nf) return false;
        nf->setGroupId(gid);
        mesh->add_face(nf);
        newFill.push_back(nf);
    }
    relinkMeshByEndpoints(mesh);
    // allocateFaceFromPool can leave fill edges unlinked when external halfedges
    // were dangling during surgery; relink explicitly by endpoint pair.
    for (MCGAL::Facet* nf : newFill) {
        MCGAL::Halfedge* h0 = nf->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && !h->opposite()) {
                for (MCGAL::Facet* of : mesh->faces()) {
                    if (!of || of->isRemoved() || !of->proxyHalfedge()) continue;
                    MCGAL::Halfedge* g0 = of->proxyHalfedge();
                    MCGAL::Halfedge* g = g0;
                    int guard2 = 0;
                    do {
                        if (g && g != h && g->vertex() == h->end_vertex() &&
                            g->end_vertex() == h->vertex() && !g->opposite()) {
                            h->setOpposite(g);
                            g->setOpposite(h);
                            break;
                        }
                        g = g->next();
                    } while (g && g != g0 && ++guard2 < 64);
                }
            }
            h = h->next();
        } while (h && h != h0 && ++guard < 64);
    }
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h) {
                if (h->opposite() == nullptr) h->setBoundary();
                else { h->setNotBoundary(); h->opposite()->setNotBoundary(); }
            }
            h = h->next();
        } while (h && h != h0 && ++guard < 64);
    }
    MCGAL::Halfedge* ab = nullptr;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge() || f->groupId() != gid) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && !h->isRemoved() && h->opposite() == nullptr &&
                ((h->vertex() == a && h->end_vertex() == b) ||
                 (h->vertex() == b && h->end_vertex() == a))) { ab = h; break; }
            h = h->next();
        } while (h && h != h0 && ++guard < 64);
        if (ab) break;
    }
    if (!ab) return false;
    if (outBoundaryEdge) *outBoundaryEdge = ab;
    if (outApexPos) *outApexPos = a->point();
    return true;
}

/// Cheap complex-fan removal: forward uses the full-patch helper only to
/// perform topology surgery; only k / removed position are needed for the
/// inverse (the ring is recovered from the post-removal triangle strip).
inline bool removeBoundaryVertexFanCheap(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                         MCGAL::Vertex* v,
                                         int& outK,
                                         MCGAL::Point& outRemoved,
                                         MCGAL::Point& outA,
                                         MCGAL::Point& outB,
                                         MCGAL::Halfedge** outBoundaryEdge,
                                         MCGAL::Point* outApexPos = nullptr) {
    return removeBoundaryVertexFanManual(mesh, v, outK, outRemoved, outA, outB, outBoundaryEdge, outApexPos);
}

/// Inverse of removeBoundaryVertexFanCheap.  The post-removal fan strip is
/// recovered by rotating around the apex; k tells us how many steps to take.
inline bool restoreBoundaryVertexFanCheap(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                          MCGAL::Halfedge* boundaryEdge,
                                          const MCGAL::Point& vPos,
                                          int k,
                                          int gid,
                                          const MCGAL::Point* apexPos = nullptr,
                                          std::vector<MCGAL::Facet*>* outFaces = nullptr,
                                          MCGAL::Vertex** outVertex = nullptr) {
    if (!mesh || !boundaryEdge || k < 2 || boundaryEdge->opposite() != nullptr) return false;
    auto third = [](MCGAL::Facet* f, MCGAL::Vertex* x, MCGAL::Vertex* y) -> MCGAL::Vertex* {
        if (!f || !f->proxyHalfedge()) return nullptr;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && h->vertex() && h->vertex() != x && h->vertex() != y) return h->vertex();
            h = h->next();
        } while (h && h != h0 && ++guard < 64);
        return nullptr;
    };
    auto findEdgeInFace = [](MCGAL::Facet* f, MCGAL::Vertex* x, MCGAL::Vertex* y) -> MCGAL::Halfedge* {
        if (!f || !f->proxyHalfedge()) return nullptr;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && ((h->vertex() == x && h->end_vertex() == y) ||
                      (h->vertex() == y && h->end_vertex() == x))) return h;
            h = h->next();
        } while (h && h != h0 && ++guard < 64);
        return nullptr;
    };
    std::vector<MCGAL::Facet*> strip;
    std::vector<MCGAL::Vertex*> ring;
    auto discover = [&](MCGAL::Vertex* apex, MCGAL::Vertex* other) -> bool {
        strip.clear();
        ring.clear();
        if (!apex || !other) return false;
        MCGAL::Facet* f0 = boundaryEdge->face();
        MCGAL::Vertex* u = third(f0, apex, other);
        if (!u) return false;
        strip.push_back(f0);
        std::vector<MCGAL::Vertex*> rev{u};
        MCGAL::Facet* curFace = f0;
        for (int step = 1; step < k - 1; ++step) {
            MCGAL::Halfedge* e = findEdgeInFace(curFace, apex, u);
            if (!e || !e->opposite() || !e->opposite()->face()) return false;
            MCGAL::Facet* nf = e->opposite()->face();
            MCGAL::Vertex* w = third(nf, apex, u);
            if (!w) return false;
            strip.push_back(nf);
            rev.push_back(w);
            u = w;
            curFace = nf;
        }
        ring.push_back(apex);
        for (auto it = rev.rbegin(); it != rev.rend(); ++it) ring.push_back(*it);
        ring.push_back(other);
        return static_cast<int>(ring.size()) == k + 1;
    };
    bool preferVertex = true;
    if (apexPos) {
        auto qa = mesh->floatPosToInt(*apexPos);
        auto qv = mesh->floatPosToInt(boundaryEdge->vertex()->point());
        preferVertex = (qa == qv);
    }
    bool firstOk = preferVertex ? discover(boundaryEdge->vertex(), boundaryEdge->end_vertex())
                                : discover(boundaryEdge->end_vertex(), boundaryEdge->vertex());
    if (!firstOk && !discover(boundaryEdge->end_vertex(), boundaryEdge->vertex()) &&
        !discover(boundaryEdge->vertex(), boundaryEdge->end_vertex())) {
        return false;
    }
    for (MCGAL::Facet* f : strip) removeFaceForPatch(mesh, f);
    MCGAL::Vertex* v = MCGAL::contextPool.allocateVertexFromPool(mesh->meshId(), vPos);
    if (!v) return false;
    v->halfedges().clear();
    v->setGroupId(gid);
    mesh->vertices().push_back(v);
    if (outVertex) *outVertex = v;
    if (outFaces) outFaces->clear();
    for (int i = 0; i < k; ++i) {
        std::vector<MCGAL::Vertex*> vs{v, ring[static_cast<size_t>(i)], ring[static_cast<size_t>(i + 1)]};
        MCGAL::Facet* nf = MCGAL::contextPool.allocateFaceFromPool(mesh->meshId(), vs);
        if (!nf) return false;
        nf->setGroupId(gid);
        mesh->add_face(nf);
        if (outFaces) outFaces->push_back(nf);
    }
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h) {
                if (h->opposite() == nullptr) h->setBoundary();
                else { h->setNotBoundary(); h->opposite()->setNotBoundary(); }
            }
            h = h->next();
        } while (h && h != h0 && ++guard < 64);
    }
    return true;
}

}  // namespace pmsf
