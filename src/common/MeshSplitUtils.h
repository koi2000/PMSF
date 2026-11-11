#pragma once

#include "ContextPool.h"
#include "Facet.h"
#include "Global.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Vertex.h"
#include "common/BoundedRingTraversal.h"

#include <algorithm>
#include <array>
#include <set>
#include <unordered_set>
#include <utility>
#include <vector>

namespace pmsf {

/// Split an interior edge (a,b) shared by triangles (a,b,c) and (b,a,d) by
/// inserting a new vertex v on the edge.  Result: four triangles
/// (a,v,c), (v,b,c), (b,v,d), (v,a,d).  This is the inverse primitive needed by
/// boundary vertex-removal decoding.
inline MCGAL::Halfedge* splitInteriorEdgeInsertVertex(
    const std::shared_ptr<MCGAL::Mesh>& mesh,
    MCGAL::Halfedge* h,
    const MCGAL::Point& pos,
    MCGAL::Vertex** outNewVertex = nullptr) {
    if (!mesh || !h || h->isRemoved() || !h->opposite() || h->opposite()->isRemoved()) return nullptr;
    MCGAL::Halfedge* ho = h->opposite();
    MCGAL::Halfedge* hbc = h->next();
    MCGAL::Halfedge* hca = hbc ? hbc->next() : nullptr;
    MCGAL::Halfedge* had = ho->next();
    MCGAL::Halfedge* hdb = had ? had->next() : nullptr;
    if (!hbc || !hca || !had || !hdb) return nullptr;
    MCGAL::Vertex* a = h->vertex();
    MCGAL::Vertex* b = h->end_vertex();
    MCGAL::Vertex* c = hbc->end_vertex();
    MCGAL::Vertex* d = had->end_vertex();
    if (!a || !b || !c || !d) return nullptr;
    MCGAL::Facet* f0 = h->face();
    MCGAL::Facet* f1 = ho->face();
    if (!f0 || !f1) return nullptr;
    const int g0 = f0->groupId();
    const int g1 = f1->groupId();

    MCGAL::Vertex* v = MCGAL::contextPool.allocateVertexFromPool(mesh->meshId(), pos);
    if (!v) return nullptr;
    v->halfedges().clear();
    mesh->vertices().push_back(v);

    auto alloc = [&](MCGAL::Vertex* v1, MCGAL::Vertex* v2) {
        MCGAL::Halfedge* e = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(mesh->meshId(), v1, v2);
        e->setFace(nullptr);
        return e;
    };
    MCGAL::Halfedge* av = alloc(a, v);
    MCGAL::Halfedge* va = alloc(v, a);
    MCGAL::Halfedge* vc = alloc(v, c);
    MCGAL::Halfedge* cv = alloc(c, v);
    MCGAL::Halfedge* vb = alloc(v, b);
    MCGAL::Halfedge* bv = alloc(b, v);
    MCGAL::Halfedge* vd = alloc(v, d);
    MCGAL::Halfedge* dv = alloc(d, v);
    av->setOpposite(va); va->setOpposite(av);
    vc->setOpposite(cv); cv->setOpposite(vc);
    vb->setOpposite(bv); bv->setOpposite(vb);
    vd->setOpposite(dv); dv->setOpposite(vd);

    // T1 (a,v,c) on old f0
    av->setNext(vc); vc->setNext(hca); hca->setNext(av);
    av->setFace(f0); vc->setFace(f0); hca->setFace(f0);
    f0->reset(av);
    f0->setGroupId(g0);

    // T2 (v,b,c) on new f2
    MCGAL::Facet* f2 = MCGAL::contextPool.allocateFaceFromPool(mesh->meshId());
    vb->setNext(hbc); hbc->setNext(cv); cv->setNext(vb);
    vb->setFace(f2); hbc->setFace(f2); cv->setFace(f2);
    f2->reset(vb);
    f2->setGroupId(g0);
    mesh->faces().push_back(f2);

    // T3 (b,v,d) on old f1
    bv->setNext(vd); vd->setNext(hdb); hdb->setNext(bv);
    bv->setFace(f1); vd->setFace(f1); hdb->setFace(f1);
    f1->reset(bv);
    f1->setGroupId(g1);

    // T4 (v,a,d) on new f3
    MCGAL::Facet* f3 = MCGAL::contextPool.allocateFaceFromPool(mesh->meshId());
    va->setNext(had); had->setNext(dv); dv->setNext(va);
    va->setFace(f3); had->setFace(f3); dv->setFace(f3);
    f3->reset(va);
    f3->setGroupId(g1);
    mesh->faces().push_back(f3);

    auto eraseRing = [](MCGAL::Vertex* vt, MCGAL::Halfedge* e) {
        if (!vt || !e) return;
        auto& hs = vt->halfedges();
        hs.erase(std::remove(hs.begin(), hs.end(), e), hs.end());
    };
    eraseRing(a, h);
    eraseRing(b, ho);
    a->halfedges().push_back(av);
    b->halfedges().push_back(bv);
    c->halfedges().push_back(cv);
    d->halfedges().push_back(dv);
    v->halfedges().push_back(va);
    v->halfedges().push_back(vc);
    v->halfedges().push_back(vb);
    v->halfedges().push_back(vd);
    v->setProxyHalfedge(va);
    a->setProxyHalfedge(av);
    b->setProxyHalfedge(bv);
    c->setProxyHalfedge(cv);
    d->setProxyHalfedge(dv);

    h->setRemoved();
    ho->setRemoved();
    if (outNewVertex) *outNewVertex = v;
    return av;
}


/// Split a mesh-boundary edge (h->opposite()==nullptr) by inserting a new
/// vertex on it.  Triangle (a,b,c) becomes (a,v,c),(v,b,c); the two new
/// boundary halfedges a->v and v->b carry the boundary flag.
inline MCGAL::Halfedge* splitBoundaryEdgeInsertVertex(
    const std::shared_ptr<MCGAL::Mesh>& mesh,
    MCGAL::Halfedge* h,
    const MCGAL::Point& pos,
    MCGAL::Vertex** outNewVertex = nullptr) {
    if (!mesh || !h || h->isRemoved() || h->opposite() != nullptr) return nullptr;
    MCGAL::Halfedge* hbc = h->next();
    MCGAL::Halfedge* hca = hbc ? hbc->next() : nullptr;
    if (!hbc || !hca) return nullptr;
    MCGAL::Vertex* a = h->vertex();
    MCGAL::Vertex* b = h->end_vertex();
    MCGAL::Vertex* c = hbc->end_vertex();
    MCGAL::Facet* f0 = h->face();
    if (!a || !b || !c || !f0) return nullptr;
    const int g0 = f0->groupId();

    MCGAL::Vertex* v = MCGAL::contextPool.allocateVertexFromPool(mesh->meshId(), pos);
    if (!v) return nullptr;
    v->halfedges().clear();
    mesh->vertices().push_back(v);

    auto alloc = [&](MCGAL::Vertex* v1, MCGAL::Vertex* v2) {
        MCGAL::Halfedge* e = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(mesh->meshId(), v1, v2);
        e->setFace(nullptr);
        return e;
    };
    MCGAL::Halfedge* av = alloc(a, v);
    MCGAL::Halfedge* vc = alloc(v, c);
    MCGAL::Halfedge* cv = alloc(c, v);
    MCGAL::Halfedge* vb = alloc(v, b);
    vc->setOpposite(cv);
    cv->setOpposite(vc);
    av->setOpposite(nullptr);
    vb->setOpposite(nullptr);
    av->setBoundary();
    vb->setBoundary();

    // T1 (a,v,c) on f0
    av->setNext(vc); vc->setNext(hca); hca->setNext(av);
    av->setFace(f0); vc->setFace(f0); hca->setFace(f0);
    f0->reset(av);
    f0->setGroupId(g0);

    // T2 (v,b,c) on new face
    MCGAL::Facet* f2 = MCGAL::contextPool.allocateFaceFromPool(mesh->meshId());
    vb->setNext(hbc); hbc->setNext(cv); cv->setNext(vb);
    vb->setFace(f2); hbc->setFace(f2); cv->setFace(f2);
    f2->reset(vb);
    f2->setGroupId(g0);
    mesh->faces().push_back(f2);

    auto eraseRing = [](MCGAL::Vertex* vt, MCGAL::Halfedge* e) {
        if (!vt || !e) return;
        auto& hs = vt->halfedges();
        hs.erase(std::remove(hs.begin(), hs.end(), e), hs.end());
    };
    eraseRing(a, h);
    a->halfedges().push_back(av);
    c->halfedges().push_back(cv);
    v->halfedges().push_back(vc);
    v->halfedges().push_back(vb);
    v->setProxyHalfedge(vc);
    a->setProxyHalfedge(av);
    c->setProxyHalfedge(cv);

    h->setRemoved();
    if (outNewVertex) *outNewVertex = v;
    return av;
}


/// Collapse a degree-3 boundary vertex v (mesh-boundary case) back into the
/// boundary edge a-b.  Inverse of splitBoundaryEdgeInsertVertex on the same patch:
/// faces (a,v,c),(v,b,c) become (a,c,b); v is removed.


/// Robust degree-3 boundary vertex collapse for the seam-cut submeshes.
/// Replaces the two incident boundary triangles by one triangle (a,b,c), creates a
/// fresh boundary edge a-b, and avoids patching the old halfedge cycles in place.
inline bool collapseBoundaryVertex3Robust(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                          MCGAL::Vertex* v,
                                          MCGAL::Halfedge** outNewEdge = nullptr) {
    if (!mesh || !v || v->isRemoved()) return false;
    std::vector<MCGAL::Facet*> faces;
    // Scan the mesh rather than trusting v->halfedges(): after several boundary
    // operations the vertex ring vector may not contain every incident face, and
    // removing only a subset leaves dangling faces that reference the removed v.
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        bool has = false;
        do {
            if (h && h->vertex() == v) { has = true; break; }
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
        if (has) faces.push_back(f);
    }
    if (faces.size() > 2) {
        if (std::getenv("PMSF_SEAM_DEBUG"))
            std::cerr << "[robust] reject v=" << v->poolId() << " incidentFaces="
                      << faces.size() << std::endl;
        return false;
    }
    if (std::getenv("PMSF_SEAM_DEBUG")) {
        std::cerr << "[robust] v=" << v << " deg=" << v->vertex_degree()
                  << " faces=" << faces.size() << std::endl;
    }
    if (faces.size() != 2) return false;
    MCGAL::Facet* f0 = faces[0];
    MCGAL::Facet* f1 = faces[1];

    auto faceVerts = [](MCGAL::Facet* f) {
        std::vector<MCGAL::Vertex*> out;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        if (!h0) return out;
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h->vertex()) out.push_back(h->vertex());
            h = h->next();
        } while (h && h != h0 && ++guard < 64);
        return out;
    };
    const auto v0 = faceVerts(f0);
    const auto v1 = faceVerts(f1);
    if (v0.size() != 3 || v1.size() != 3) return false;

    MCGAL::Vertex* a = nullptr;
    MCGAL::Vertex* b = nullptr;
    MCGAL::Vertex* c = nullptr;
    for (MCGAL::Vertex* x : v0) {
        if (x == v) continue;
        if (std::find(v1.begin(), v1.end(), x) != v1.end()) { c = x; break; }
    }
    for (MCGAL::Vertex* x : v0) {
        if (x == v || x == c) continue;
        a = x;
        break;
    }
    for (MCGAL::Vertex* x : v1) {
        if (x == v || x == c || x == a) continue;
        b = x;
        break;
    }
    if (std::getenv("PMSF_SEAM_DEBUG") && faces.size() == 2) {
        std::cerr << "[robust2] v=" << v->poolId() << " f0=" << f0 << " f1=" << f1
                  << " v0=[";
        for (auto* x : v0) std::cerr << (x?x->poolId():-1) << ",";
        std::cerr << "] v1=[";
        for (auto* x : v1) std::cerr << (x?x->poolId():-1) << ",";
        std::cerr << "] a=" << (a?1:0) << " b=" << (b?1:0) << " c=" << (c?1:0) << std::endl;
    }
    if (!a || !b || !c) return false;
    if (a == b || a == c || b == c) return false;
    auto edgeIsBoundary = [&](MCGAL::Vertex* x, MCGAL::Vertex* y) {
        if (!x || !y) return false;
        MCGAL::Halfedge* h = x->find_halfedge(y);
        return h && h->opposite() == nullptr;
    };
    if (!edgeIsBoundary(a, v) || !edgeIsBoundary(v, b)) return false;
    if (a->find_halfedge(b) != nullptr) return false;

    // Remove both incident faces and their halfedges from vertex rings.
    auto removeFace = [&](MCGAL::Facet* f) {
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        if (!h0) return;
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            MCGAL::Halfedge* next = h->next();
            if (h->opposite()) {
                h->opposite()->setOpposite(nullptr);
                h->opposite()->setBoundary();
            }
            if (h->vertex()) {
                auto& ring = h->vertex()->halfedges();
                ring.erase(std::remove(ring.begin(), ring.end(), h), ring.end());
            }
            h->setRemoved();
            h = next;
        } while (h && h != h0 && ++guard < 64);
        f->setRemoved();
        mesh->eraseFacetByPointer(f);
    };
    removeFace(f0);
    removeFace(f1);

    const int gid = f0->groupId();
    std::vector<MCGAL::Vertex*> nvs{v, a, b, c};  // placeholders, set below
    nvs = {a, b, c};
    MCGAL::Facet* nf = MCGAL::contextPool.allocateFaceFromPool(mesh->meshId(), nvs);
    if (!nf) return false;
    nf->setGroupId(gid);
    mesh->add_face(nf);

    v->halfedges().clear();
    v->setRemoved();

    for (MCGAL::Vertex* vt : {a, b, c}) {
        MCGAL::Halfedge* proxy = nullptr;
        for (MCGAL::Halfedge* h : vt->halfedges()) {
            if (h && h->face() == nf) { proxy = h; break; }
        }
        if (proxy) vt->setProxyHalfedge(proxy);
    }
    // Re-establish boundary flags for the new face and its neighbours.
    MCGAL::Halfedge* nh0 = nf->proxyHalfedge();
    MCGAL::Halfedge* nh = nh0;
    int guard = 0;
    do {
        if (nh->opposite() == nullptr) {
            nh->setBoundary();
        } else {
            nh->setNotBoundary();
            nh->opposite()->setNotBoundary();
        }
        nh = nh->next();
    } while (nh && nh != nh0 && ++guard < 64);

    if (outNewEdge) {
        MCGAL::Halfedge* ab = a->find_halfedge(b);
        if (!ab) {
            for (MCGAL::Halfedge* h : *nf) {
                if (h->vertex() == a && h->end_vertex() == b) { ab = h; break; }
            }
        }
        *outNewEdge = ab;
    }
    return true;
}


/// Deterministic face color for a partition id (RGB 0..255).
inline std::array<int, 3> seamGroupColor(int gid) {
    static const std::array<std::array<int, 3>, 32> pal = {{
        {{66, 133, 244}}, {{219, 68, 55}}, {{244, 180, 0}}, {{15, 157, 88}},
        {{171, 71, 188}}, {{0, 172, 193}}, {{255, 112, 67}}, {{124, 179, 66}},
        {{92, 107, 192}}, {{240, 98, 146}}, {{38, 166, 154}}, {{255, 167, 38}},
        {{141, 110, 99}}, {{120, 144, 156}}, {{236, 64, 122}}, {{156, 204, 101}},
        {{63, 81, 181}}, {{0, 150, 136}}, {{205, 220, 57}}, {{229, 115, 115}},
        {{100, 181, 246}}, {{139, 195, 74}}, {{255, 183, 77}}, {{77, 182, 172}},
        {{149, 117, 205}}, {{174, 213, 129}}, {{255, 138, 101}}, {{176, 190, 197}},
        {{121, 85, 72}}, {{197, 202, 51}}, {{233, 30, 99}}, {{69, 90, 100}},
    }};
    return pal[static_cast<std::size_t>(std::max(0, gid)) % pal.size()];
}

/// Dump a mesh snapshot as a "colored OFF": face lines carry extra r g b so a
/// partition-colored render can be produced without a group table side file.
inline bool dumpSeamRoundColored(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                 const std::string& path) {
    if (!mesh) return false;
    FILE* fp = std::fopen(path.c_str(), "w");
    if (!fp) return false;
    std::unordered_map<MCGAL::Vertex*, int> vidx;
    std::vector<MCGAL::Vertex*> verts;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        vidx[v] = static_cast<int>(verts.size());
        verts.push_back(v);
    }
    std::vector<MCGAL::Facet*> faces;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        faces.push_back(f);
    }
    std::fprintf(fp, "OFF\n%zu %zu 0\n", verts.size(), faces.size());
    for (MCGAL::Vertex* v : verts)
        std::fprintf(fp, "%.6f %.6f %.6f\n", v->point().x(), v->point().y(), v->point().z());
    for (MCGAL::Facet* f : faces) {
        std::fprintf(fp, "3");
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && h->vertex() && vidx.count(h->vertex()))
                std::fprintf(fp, " %d", vidx[h->vertex()]);
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
        const auto c = seamGroupColor(f->groupId());
        std::fprintf(fp, " %d %d %d\n", c[0], c[1], c[2]);
    }
    std::fclose(fp);
    return true;
}

/// Deterministic mesh element ordering used by the ordered boundary codec.
/// Both encoder and decoder call this on equivalent meshes so subsequent
/// rounds see the same vertex/face container order without serializing it.
inline void canonicalizeMeshOrder(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!mesh) return;
    mesh->garbage_collection();
    auto qOf = [&](MCGAL::Vertex* v) {
        return mesh->floatPosToInt(v->point());
    };
    // Vertex key: coords + gid + sorted one-ring neighbour descriptors.
    // The neighbour part breaks coordinate ties (seam duplicates share coords).
    auto vertexKey = [&](MCGAL::Vertex* v) {
        std::vector<int> key;
        const MCGAL::PointInt q = qOf(v);
        key.insert(key.end(), {q.x(), q.y(), q.z(), v->groupId()});
        std::vector<std::array<int, 4>> nbrs;
        for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
            if (!h || !h->end_vertex() || h->end_vertex()->isRemoved()) continue;
            const MCGAL::PointInt nq = qOf(h->end_vertex());
            nbrs.push_back({nq.x(), nq.y(), nq.z(), h->end_vertex()->groupId()});
        }
        std::sort(nbrs.begin(), nbrs.end());
        for (const auto& n : nbrs) key.insert(key.end(), {n[0], n[1], n[2], n[3]});
        return key;
    };
    std::vector<std::pair<std::vector<int>, MCGAL::Vertex*>> vkeyed;
    vkeyed.reserve(mesh->vertices().size());
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        vkeyed.emplace_back(vertexKey(v), v);
    }
    std::stable_sort(vkeyed.begin(), vkeyed.end(),
                     [](const auto& a, const auto& b) { return a.first < b.first; });
    {
        std::vector<MCGAL::Vertex*> ordered;
        ordered.reserve(vkeyed.size());
        for (auto& kv : vkeyed) ordered.push_back(kv.second);
        mesh->vertices() = std::move(ordered);
    }
    struct FaceKey {
        std::vector<int> key;
        bool operator<(const FaceKey& o) const { return key < o.key; }
    };
    std::vector<std::pair<FaceKey, MCGAL::Facet*>> keyed;
    keyed.reserve(mesh->faces().size());
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        FaceKey fk;
        fk.key.push_back(f->groupId());
        std::vector<std::array<int, 3>> pts;
        std::vector<std::array<int, 6>> aux;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && h->vertex()) {
                const MCGAL::PointInt q = qOf(h->vertex());
                pts.push_back({q.x(), q.y(), q.z()});
            }
            if (h && h->end_vertex() && h->opposite() && h->opposite()->face() &&
                !h->opposite()->face()->isRemoved()) {
                MCGAL::Facet* nf = h->opposite()->face();
                MCGAL::Halfedge* nh0 = nf->proxyHalfedge();
                MCGAL::Halfedge* nh = nh0;
                int ng = 0;
                while (nh && nh != h->opposite() && ++ng < 64) nh = nh->next();
                MCGAL::Halfedge* other = nh ? nh->next() : nullptr;
                if (other && other->vertex()) {
                    const MCGAL::PointInt oq = qOf(other->vertex());
                    aux.push_back({oq.x(), oq.y(), oq.z(), nf->groupId(), 0, 0});
                }
            }
            h = h->next();
        } while (h && h != h0 && ++guard < 64);
        std::sort(pts.begin(), pts.end());
        std::sort(aux.begin(), aux.end());
        for (const auto& p : pts) fk.key.insert(fk.key.end(), {p[0], p[1], p[2]});
        for (const auto& a : aux) fk.key.insert(fk.key.end(), {a[0], a[1], a[2], a[3]});
        keyed.emplace_back(std::move(fk), f);
    }
    std::stable_sort(keyed.begin(), keyed.end(),
                     [](const auto& a, const auto& b) { return a.first < b.first; });
    std::vector<MCGAL::Facet*> ordered;
    ordered.reserve(keyed.size());
    for (auto& kv : keyed) ordered.push_back(kv.second);
    mesh->faces() = std::move(ordered);
}

/// Debug/experiment: reconstruct per-vertex group ids from incident face group
/// ids. Seam base serialization stores only coordinates, so the decoder starts
/// with all vertex group ids = 0; later rounds that mix boundary/facet symbols
/// rely on those ids in some paths.
inline int inferVertexGroupsFromFaces(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!mesh) return 0;
    int changed = 0;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        int gid = -1;
        for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
            if (!h || !h->face() || h->face()->isRemoved()) continue;
            gid = h->face()->groupId();
            break;
        }
        if (gid >= 0 && v->groupId() != gid) {
            v->setGroupId(gid);
            ++changed;
        }
    }
    return changed;
}

struct VertexPosKeyHash {
    std::size_t operator()(const std::array<int, 3>& k) const noexcept {
        std::size_t h = 1469598103934665603ull;
        for (int v : k) {
            h ^= static_cast<std::size_t>(static_cast<std::uint32_t>(v));
            h *= 1099511628211ull;
        }
        return h;
    }
};

/// Map from quantized position -> distinct live vertex group ids at that position.
/// Seam cutting duplicates boundary vertices at identical positions with
/// different group ids; a vertex is a "seam boundary vertex" iff such a partner
/// exists. True open-mesh boundary vertices have no partner.
inline std::unordered_map<std::array<int, 3>, std::vector<int>, VertexPosKeyHash>
buildVertexPosGroups(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    std::unordered_map<std::array<int, 3>, std::vector<int>, VertexPosKeyHash> out;
    if (!mesh) return out;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        const MCGAL::PointInt q = mesh->floatPosToInt(v->point());
        auto& gids = out[{q.x(), q.y(), q.z()}];
        bool seen = false;
        for (int g : gids) if (g == v->groupId()) { seen = true; break; }
        if (!seen) gids.push_back(v->groupId());
    }
    return out;
}

inline bool isSeamBoundaryVertex(
    const std::unordered_map<std::array<int, 3>, std::vector<int>, VertexPosKeyHash>& map,
    const std::shared_ptr<MCGAL::Mesh>& mesh, MCGAL::Vertex* v) {
    if (!mesh || !v) return false;
    const MCGAL::PointInt q = mesh->floatPosToInt(v->point());
    auto it = map.find({q.x(), q.y(), q.z()});
    if (it == map.end()) return false;
    for (int g : it->second)
        if (g != v->groupId()) return true;
    return false;
}

/// Recompute halfedge opposite pairing for the whole live mesh.
/// Boundary-op created faces can leave an edge with two unpaired halfedges
/// (a topological hole) in the encoder while the base-mesh rebuild on the
/// decoder side pairs them; that divergence changes later reconstruction.
/// Pairing is reciprocal: for each (u->v) halfedge find an unpaired (v->u)
/// halfedge and link them; leftover halfedges become boundary.
inline int repairHalfedgePairing(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!mesh) return 0;
    std::vector<MCGAL::Halfedge*> all;
    std::unordered_set<MCGAL::Halfedge*> seen;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && seen.insert(h).second) all.push_back(h);
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
    }
    for (MCGAL::Halfedge* h : all) {
        if (h->opposite()) {
            h->opposite()->setOpposite(nullptr);
            h->setOpposite(nullptr);
        }
    }
    auto key = [](MCGAL::Vertex* a, MCGAL::Vertex* b) {
        return std::array<long long, 6>{
            a ? static_cast<long long>(a->poolId()) : -1,
            b ? static_cast<long long>(b->poolId()) : -2,
            a ? static_cast<long long>(a->groupId()) : -1,
            b ? static_cast<long long>(b->groupId()) : -2, 0, 0};
    };
    std::map<std::array<long long, 6>, std::vector<MCGAL::Halfedge*>> pending;
    for (MCGAL::Halfedge* h : all) {
        if (!h->vertex() || !h->end_vertex()) continue;
        pending[key(h->vertex(), h->end_vertex())].push_back(h);
    }
    int paired = 0;
    // Pair reciprocal directions, preferring equal poolId ordering.
    for (auto& kv : pending) {
        auto& list = kv.second;
        for (MCGAL::Halfedge* h : list) {
            if (!h || h->opposite()) continue;
            if (!h->vertex() || !h->end_vertex()) continue;
            const auto rkey = key(h->end_vertex(), h->vertex());
            auto it = pending.find(rkey);
            if (it == pending.end()) continue;
            MCGAL::Halfedge* best = nullptr;
            for (MCGAL::Halfedge* cand : it->second) {
                if (cand && !cand->opposite() && cand != h) { best = cand; break; }
            }
            if (!best) continue;
            h->setOpposite(best);
            best->setOpposite(h);
            ++paired;
        }
    }
    for (MCGAL::Halfedge* h : all) {
        if (h->opposite()) {
            h->setNotBoundary();
            h->opposite()->setNotBoundary();
        } else {
            h->setBoundary();
        }
    }
    return paired;
}

/// Debug-only: dump a round snapshot with integer coordinates, vertex group ids
/// and face indices so encoder/decoder states can be compared exactly (seam
/// duplicate coordinates are disambiguated by group id).
inline bool dumpSeamRoundGrouped(const std::shared_ptr<MCGAL::Mesh>& mesh, const std::string& path) {
    if (!mesh) return false;
    FILE* fp = std::fopen(path.c_str(), "w");
    if (!fp) return false;
    std::unordered_map<MCGAL::Vertex*, int> vidx;
    std::vector<MCGAL::Vertex*> verts;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        vidx[v] = static_cast<int>(verts.size());
        verts.push_back(v);
    }
    std::vector<MCGAL::Facet*> faces;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        faces.push_back(f);
    }
    std::size_t heTotal = 0, heUnpaired = 0, heDangling = 0;
    for (MCGAL::Facet* f : faces) {
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h) {
                ++heTotal;
                if (!h->opposite()) ++heUnpaired;
                if (h->vertex() && h->vertex()->isRemoved()) ++heDangling;
            }
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
    }
    std::fprintf(fp, "V %zu F %zu H %zu unpaired %zu dangling %zu\n",
                 verts.size(), faces.size(), heTotal, heUnpaired, heDangling);
    for (MCGAL::Vertex* v : verts) {
        const MCGAL::PointInt q = mesh->floatPosToInt(v->point());
        std::fprintf(fp, "v %d %d %d %d\n", q.x(), q.y(), q.z(), v->groupId());
    }
    for (MCGAL::Facet* f : faces) {
        std::fprintf(fp, "f %d", f->groupId());
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && h->vertex() && vidx.count(h->vertex()))
                std::fprintf(fp, " %d", vidx[h->vertex()]);
            h = h->next();
        } while (h && h != h0 && ++guard < 64);
        std::fputc('\n', fp);
    }
    std::fclose(fp);
    return true;
}

}  // namespace pmsf
