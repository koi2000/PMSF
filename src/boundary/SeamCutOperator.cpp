#include "boundary/SeamCutOperator.h"

#include "Global.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <map>
#include <cstdio>
#include <cstdlib>
#include <unordered_set>

namespace pmsf {
namespace {

using CopyKey = long long;

inline CopyKey copyKey(int poolId, int groupId) {
    return (static_cast<long long>(poolId) << 32) |
           static_cast<unsigned int>(groupId);
}

void copyMeshMetadata(const std::shared_ptr<MCGAL::Mesh>& dst,
                      const std::shared_ptr<MCGAL::Mesh>& src) {
    if (!dst || !src) return;
    dst->bboxMin = src->bboxMin;
    dst->bboxMax = src->bboxMax;
    dst->f_quantStep = src->f_quantStep;
    dst->i_nbQuantBits = src->i_nbQuantBits;
    dst->enableQuantization = src->enableQuantization;
}

}  // namespace

SeamCutResult SeamCutOperator::run(const std::shared_ptr<MCGAL::Mesh>& input,
                                   const std::vector<MCGAL::Halfedge*>& seeds) {
    SeamCutResult result;
    if (!input) return result;

    result.mesh = std::make_shared<MCGAL::Mesh>();
    result.mesh->setMeshId(input->meshId());
    copyMeshMetadata(result.mesh, input);

    std::unordered_map<CopyKey, MCGAL::Vertex*> copies;
    copies.reserve(input->vertices().size() * 2 + 16);
    int maxGroup = -1;

    auto copyFor = [&](MCGAL::Vertex* v, int groupId) -> MCGAL::Vertex* {
        if (!v || groupId < 0) return nullptr;
        const CopyKey key = copyKey(v->poolId(), groupId);
        auto it = copies.find(key);
        if (it != copies.end()) return it->second;
        MCGAL::Vertex* copy =
            MCGAL::contextPool.allocateVertexFromPool(result.mesh->meshId(), v->point());
        if (!copy) return nullptr;
        copy->halfedges().clear();
        copy->setGroupId(groupId);
        result.mesh->add_vertex(copy);
        copies.emplace(key, copy);
        result.originOfPoolId[copy->poolId()] = v->poolId();
        maxGroup = std::max(maxGroup, groupId);
        return copy;
    };

    for (MCGAL::Facet* f : input->faces()) {
        if (!f || f->isRemoved()) continue;
        const int gid = f->groupId();
        if (gid < 0) {
            std::fprintf(stderr, "[seam-cut] face with invalid groupId, aborting\n");
            return {};
        }
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        if (!h0) continue;
        std::vector<MCGAL::Vertex*> vs;
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            MCGAL::Vertex* copy = copyFor(h->vertex(), gid);
            if (!copy) return {};
            vs.push_back(copy);
            h = h->next();
        } while (h && h != h0 && ++guard < 4096);
        if (vs.size() < 3) continue;
        MCGAL::Facet* nf = MCGAL::contextPool.allocateFaceFromPool(result.mesh->meshId(), vs);
        if (!nf) return {};
        nf->setGroupId(gid);
        result.mesh->add_face(nf);
    }

    // Real mesh boundaries in the cut mesh: no opposite halfedge.
    for (MCGAL::Facet* f : result.mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        if (!h0) continue;
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h->opposite() == nullptr) h->setBoundary();
            h = h->next();
        } while (h && h != h0 && ++guard < 4096);
    }

    // Remap seeds: original seed endpoints to their copies in the seed's group.
    for (MCGAL::Halfedge* s : seeds) {
        if (!s || !s->face()) continue;
        const int gid = s->face()->groupId();
        MCGAL::Vertex* a = nullptr;
        MCGAL::Vertex* b = nullptr;
        auto ia = copies.find(copyKey(s->vertex()->poolId(), gid));
        auto ib = copies.find(copyKey(s->end_vertex()->poolId(), gid));
        if (ia != copies.end()) a = ia->second;
        if (ib != copies.end()) b = ib->second;
        if (!a || !b) continue;
        MCGAL::Halfedge* h = a->find_halfedge(b);
        if (h) result.seeds.push_back(h);
    }

    result.groupCount = maxGroup + 1;
    return result;
}

std::shared_ptr<MCGAL::Mesh> SeamMergeOperator::run(
    const std::shared_ptr<MCGAL::Mesh>& seamMesh,
    const std::unordered_map<int, int>& originOfPoolId) {
    if (!seamMesh) return nullptr;
    if (std::getenv("PMSF_SEAM_DEBUG")) { std::cerr << "[merge-pool] V=" << MCGAL::contextPool.getVindex(seamMesh->meshId()) << " H=" << MCGAL::contextPool.getHindex(seamMesh->meshId()) << " F=" << MCGAL::contextPool.getFindex(seamMesh->meshId()) << std::endl; }
    auto out = std::make_shared<MCGAL::Mesh>();
    out->setMeshId(seamMesh->meshId());
    copyMeshMetadata(out, seamMesh);

    std::unordered_map<int, MCGAL::Vertex*> canonical;  // origin id -> merged vertex
    std::unordered_map<int, MCGAL::Vertex*> byPool;     // seam poolId -> merged vertex
    canonical.reserve(originOfPoolId.size() * 2 + 16);
    byPool.reserve(seamMesh->vertices().size() + 16);

    for (MCGAL::Vertex* v : seamMesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        auto it = originOfPoolId.find(v->poolId());
        if (it != originOfPoolId.end()) {
            const int origin = it->second;
            auto c = canonical.find(origin);
            if (c != canonical.end()) {
                byPool[v->poolId()] = c->second;
                continue;
            }
            MCGAL::Vertex* nv =
                MCGAL::contextPool.allocateVertexFromPool(out->meshId(), v->point());
            if (!nv) return nullptr;
            nv->halfedges().clear();
            nv->setGroupId(v->groupId());
            out->add_vertex(nv);
            canonical.emplace(origin, nv);
            byPool[v->poolId()] = nv;
        } else {
            // Reconstructed vertex: keep it as a unique vertex.
            MCGAL::Vertex* nv =
                MCGAL::contextPool.allocateVertexFromPool(out->meshId(), v->point());
            if (!nv) return nullptr;
            nv->halfedges().clear();
            nv->setGroupId(v->groupId());
            out->add_vertex(nv);
            byPool[v->poolId()] = nv;
        }
    }

    for (MCGAL::Facet* f : seamMesh->faces()) {
        if (!f || f->isRemoved()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        if (!h0) continue;
        std::vector<MCGAL::Vertex*> vs;
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            auto it = byPool.find(h->vertex()->poolId());
            if (it == byPool.end()) return nullptr;
            vs.push_back(it->second);
            h = h->next();
        } while (h && h != h0 && ++guard < 4096);
        if (vs.size() < 3) continue;
        MCGAL::Facet* nf = MCGAL::contextPool.allocateFaceFromPool(out->meshId(), vs);
        if (!nf) return nullptr;
        nf->setGroupId(f->groupId());
        out->add_face(nf);
    }

    for (MCGAL::Facet* f : out->faces()) {
        if (!f || f->isRemoved()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        if (!h0) continue;
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h->opposite() == nullptr) h->setBoundary();
            h = h->next();
        } while (h && h != h0 && ++guard < 4096);
    }
    return out;
}

std::shared_ptr<MCGAL::Mesh> SeamMergeOperator::mergeByPosition(
    const std::shared_ptr<MCGAL::Mesh>& seamMesh, bool exactFloat) {
    if (!seamMesh) return nullptr;
    auto out = std::make_shared<MCGAL::Mesh>();
    out->setMeshId(seamMesh->meshId());
    copyMeshMetadata(out, seamMesh);

    using Key = std::array<unsigned int, 3>;
    auto pointKey = [&](MCGAL::Vertex* v) -> Key {
        if (exactFloat) {
            MCGAL::Point p = v->point();
            Key k{};
            for (int i = 0; i < 3; ++i) {
                float f = p[i];
                std::memcpy(&k[i], &f, sizeof(float));
            }
            return k;
        }
        const MCGAL::PointInt q = seamMesh->floatPosToInt(v->point());
        return Key{static_cast<unsigned int>(q.x()), static_cast<unsigned int>(q.y()),
                   static_cast<unsigned int>(q.z())};
    };
    auto isBoundaryVertex = [](MCGAL::Vertex* v) {
        if (!v) return false;
        for (MCGAL::Halfedge* h : v->halfedges()) {
            if (h && (h->isBoundary() || h->opposite() == nullptr)) return true;
        }
        return false;
    };
    std::unordered_set<long long> faceVertexPairs;
    faceVertexPairs.reserve(seamMesh->faces().size() * 3 + 16);
    for (MCGAL::Facet* f : seamMesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        std::array<int, 4> pids{};
        int n = 0;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && h->vertex() && n < 4) pids[n++] = h->vertex()->poolId();
            h = h->next();
        } while (h && h != h0 && ++guard < 64);
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                long long a = pids[i], b = pids[j];
                if (a > b) std::swap(a, b);
                faceVertexPairs.insert((a << 32) ^ b);
            }
        }
    }
    auto areAdjacent = [&](MCGAL::Vertex* a, MCGAL::Vertex* b) {
        if (!a || !b) return false;
        long long x = a->poolId(), y = b->poolId();
        if (x > y) std::swap(x, y);
        return faceVertexPairs.count((x << 32) ^ y) != 0;
    };

    int mergeCount = 0, mergeSameGroup = 0, guardAdjacent = 0;
    std::map<Key, MCGAL::Vertex*> boundaryCanonical;
    std::unordered_map<int, MCGAL::Vertex*> byPool;
    byPool.reserve(seamMesh->vertices().size() + 16);

    for (MCGAL::Vertex* v : seamMesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        MCGAL::Vertex* target = nullptr;
        if (isBoundaryVertex(v)) {
            const auto key = pointKey(v);
            auto it = boundaryCanonical.find(key);
            if (it != boundaryCanonical.end()) {
                if (areAdjacent(v, it->second)) { ++guardAdjacent; }
                else {
                    ++mergeCount; if (v->groupId() == it->second->groupId()) ++mergeSameGroup;
                    target = it->second;
                    byPool[v->poolId()] = target;
                    continue;
                }
            }
        }
        MCGAL::Vertex* nv =
            MCGAL::contextPool.allocateVertexFromPool(out->meshId(), v->point());
        if (!nv) return nullptr;
        nv->halfedges().clear();
        nv->setGroupId(v->groupId());
        out->add_vertex(nv);
        target = nv;
        if (isBoundaryVertex(v)) boundaryCanonical.emplace(pointKey(v), target);
        byPool[v->poolId()] = target;
    }

    for (MCGAL::Facet* f : seamMesh->faces()) {
        if (!f || f->isRemoved()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        if (!h0) continue;
        std::vector<MCGAL::Vertex*> vs;
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            auto it = byPool.find(h->vertex()->poolId());
            if (it == byPool.end()) return nullptr;
            vs.push_back(it->second);
            h = h->next();
        } while (h && h != h0 && ++guard < 4096);
        if (vs.size() < 3) continue;
        MCGAL::Facet* nf = MCGAL::contextPool.allocateFaceFromPool(out->meshId(), vs);
        if (!nf) return nullptr;
        nf->setGroupId(f->groupId());
        out->add_face(nf);
    }
    return out;
}

}  // namespace pmsf
