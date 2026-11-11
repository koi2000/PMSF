#include "EdgeFlipPostProcessOperator.h"
#include "EdgeFlipCandidateBfs.h"
#include "common/BoundedRingTraversal.h"

#include "Facet.h"
#include "Halfedge.h"
#include "MeshUtils.h"
#include "Vertex.h"
#include <algorithm>
#include <array>
#include <cstdio>
#include <cmath>
#include <climits>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <unordered_set>

namespace {

bool isTrianglePairFlippable(MCGAL::Halfedge* h);

double edgeLengthSquared(MCGAL::Vertex* a, MCGAL::Vertex* b) {
    if (!a || !b) return 0.0;
    const double dx = static_cast<double>(a->x()) - static_cast<double>(b->x());
    const double dy = static_cast<double>(a->y()) - static_cast<double>(b->y());
    const double dz = static_cast<double>(a->z()) - static_cast<double>(b->z());
    return dx * dx + dy * dy + dz * dz;
}

int stableVertexId(MCGAL::Vertex* v) {
    if (!v) return -1;
    return v->poolId();
}

std::array<int, 3> symbolPointKey(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                  MCGAL::Vertex* v) {
    if (!v) return {INT_MIN, INT_MIN, INT_MIN};
    if (mesh && mesh->enableQuantization) {
        MCGAL::PointInt q = mesh->floatPosToInt(v->point());
        return {q[0], q[1], q[2]};
    }
    constexpr double kScale = 1000000.0;
    return {static_cast<int>(std::llround(static_cast<double>(v->x()) * kScale)),
            static_cast<int>(std::llround(static_cast<double>(v->y()) * kScale)),
            static_cast<int>(std::llround(static_cast<double>(v->z()) * kScale))};
}

std::pair<int, int> normalizedEdgeId(int a, int b) {
    if (a > b) std::swap(a, b);
    return std::make_pair(a, b);
}

std::uint64_t undirectedEdgeKey(MCGAL::Halfedge* h) {
    if (!h || !h->opposite()) return 0;
    const int a = h->poolId();
    const int b = h->opposite()->poolId();
    const int lo = std::min(a, b);
    const int hi = std::max(a, b);
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(lo)) << 32) ^
           static_cast<std::uint32_t>(hi);
}

MCGAL::Halfedge* findHalfedgeByPoolIds(const std::shared_ptr<MCGAL::Mesh>& mesh, int a, int b) {
    if (!mesh || a < 0 || b < 0) return nullptr;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        for (MCGAL::Halfedge* h : *f) {
            if (!h || h->isRemoved()) continue;
            MCGAL::Vertex* v0 = h->vertex();
            MCGAL::Vertex* v1 = h->end_vertex();
            if (!v0 || !v1 || v0->isRemoved() || v1->isRemoved()) continue;
            const int i0 = v0->poolId();
            const int i1 = v1->poolId();
            if ((i0 == a && i1 == b) || (i0 == b && i1 == a)) return h;
        }
    }
    return nullptr;
}

std::array<long long, 3> vertexKey(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                   MCGAL::Vertex* v) {
    if (!v) return {0, 0, 0};
    if (mesh && mesh->enableQuantization) {
        MCGAL::PointInt q = mesh->floatPosToInt(v->point());
        return {static_cast<long long>(q[0]),
                static_cast<long long>(q[1]),
                static_cast<long long>(q[2])};
    }
    constexpr double kScale = 100000000.0;
    return {static_cast<long long>(std::llround(static_cast<double>(v->x()) * kScale)),
            static_cast<long long>(std::llround(static_cast<double>(v->y()) * kScale)),
            static_cast<long long>(std::llround(static_cast<double>(v->z()) * kScale))};
}

bool vertexKeyLess(const std::array<long long, 3>& lhs,
                   const std::array<long long, 3>& rhs) {
    if (lhs[0] != rhs[0]) return lhs[0] < rhs[0];
    if (lhs[1] != rhs[1]) return lhs[1] < rhs[1];
    return lhs[2] < rhs[2];
}

std::array<long long, 12> replayCandidateKey(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                             MCGAL::Halfedge* h) {
    if (!h || !h->opposite() || !h->next() || !h->opposite()->next()) return {};
    auto e0 = vertexKey(mesh, h->vertex());
    auto e1 = vertexKey(mesh, h->end_vertex());
    auto o0 = vertexKey(mesh, h->next()->end_vertex());
    auto o1 = vertexKey(mesh, h->opposite()->next()->end_vertex());
    if (vertexKeyLess(e1, e0)) std::swap(e0, e1);
    if (vertexKeyLess(o1, o0)) std::swap(o0, o1);
    return {e0[0], e0[1], e0[2],
            e1[0], e1[1], e1[2],
            o0[0], o0[1], o0[2],
            o1[0], o1[1], o1[2]};
}

std::array<long long, 6> canonicalHalfedgeKey(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                              MCGAL::Halfedge* h) {
    auto a = vertexKey(mesh, h ? h->vertex() : nullptr);
    auto b = vertexKey(mesh, h ? h->end_vertex() : nullptr);
    return {a[0], a[1], a[2], b[0], b[1], b[2]};
}

void canonicalizeTraversalProxies(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!mesh) return;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Halfedge* best = nullptr;
        for (MCGAL::Halfedge* h : *f) {
            if (!h || h->isRemoved() || !h->vertex() || h->vertex()->isRemoved()) continue;
            if (!best || canonicalHalfedgeKey(mesh, best) < canonicalHalfedgeKey(mesh, h)) {
                best = h;
            }
        }
        if (best) f->setProxyHalfedge(best);
    }
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        MCGAL::Halfedge* best = nullptr;
        for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
            if (!h || h->isRemoved() || h->vertex() != v ||
                !h->end_vertex() || h->end_vertex()->isRemoved()) {
                continue;
            }
            if (!best || canonicalHalfedgeKey(mesh, best) < canonicalHalfedgeKey(mesh, h)) {
                best = h;
            }
        }
        if (best) v->setProxyHalfedge(best);
    }
}

std::vector<MCGAL::Halfedge*> collectReplayCandidates(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                                      int seedVid0,
                                                      int seedVid1) {
    struct ReplayCandidate {
        MCGAL::Halfedge* h = nullptr;
        std::array<long long, 12> key{};
    };
    std::vector<MCGAL::Halfedge*> out;
    if (!mesh) return out;
    std::vector<ReplayCandidate> candidates;
    candidates.reserve(mesh->size_of_facets() * 2 + 8);
    std::unordered_set<std::uint64_t> seen;
    seen.reserve(mesh->size_of_facets() * 4 + 8);
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        for (MCGAL::Halfedge* h : *f) {
            if (!isTrianglePairFlippable(h)) continue;
            const std::uint64_t edgeKey = undirectedEdgeKey(h);
            if (edgeKey == 0 || !seen.insert(edgeKey).second) continue;
            ReplayCandidate item;
            item.h = h;
            item.key = replayCandidateKey(mesh, h);
            candidates.push_back(item);
        }
    }
    std::stable_sort(candidates.begin(), candidates.end(),
                     [](const ReplayCandidate& lhs, const ReplayCandidate& rhs) {
                         return lhs.key < rhs.key;
                     });
    out.reserve(candidates.size());
    for (const ReplayCandidate& item : candidates) {
        out.push_back(item.h);
    }
    return out;
}

bool isTrianglePairFlippable(MCGAL::Halfedge* h) {
    if (!h || h->isRemoved()) return false;
    MCGAL::Halfedge* ho = h->opposite();
    if (!ho || ho->isRemoved()) return false;
    MCGAL::Facet* f0 = h->face();
    MCGAL::Facet* f1 = ho->face();
    if (!f0 || !f1 || f0->isRemoved() || f1->isRemoved()) return false;
    if (f0->facet_degree() != 3 || f1->facet_degree() != 3) return false;
    if (!h->next() || !h->next()->next() || !ho->next() || !ho->next()->next()) return false;
    MCGAL::Vertex* a = h->vertex();
    MCGAL::Vertex* b = h->end_vertex();
    MCGAL::Vertex* c = h->next()->end_vertex();
    MCGAL::Vertex* d = ho->next()->end_vertex();
    if (!a || !b || !c || !d) return false;
    if (a == b || a == c || a == d || b == c || b == d || c == d) return false;
    return true;
}

bool touchesProtectedSeed(MCGAL::Halfedge* h, int s0, int s1) {
    if (!h || (s0 < 0 && s1 < 0)) return false;
    MCGAL::Halfedge* ho = h->opposite();
    MCGAL::Vertex* vs[4] = {
        h->vertex(),
        h->end_vertex(),
        h->next() ? h->next()->end_vertex() : nullptr,
        (ho && ho->next()) ? ho->next()->end_vertex() : nullptr};
    for (MCGAL::Vertex* v : vs) {
        const int id = stableVertexId(v);
        if (id == s0 || id == s1) return true;
    }
    return false;
}

bool touchesForbiddenVertex(MCGAL::Halfedge* h) {
    static const bool enabled = []() {
        const char* v = std::getenv("PMSF_EDGEFLIP_PROTECT_PATCH");
        return v && *v && std::string(v) != "0";
    }();
    if (!enabled) return false;
    if (!h || !h->opposite()) return true;
    MCGAL::Halfedge* ho = h->opposite();
    MCGAL::Vertex* vs[4] = {
        h->vertex(),
        h->end_vertex(),
        h->next() ? h->next()->end_vertex() : nullptr,
        (ho && ho->next()) ? ho->next()->end_vertex() : nullptr};
    for (MCGAL::Vertex* v : vs) {
        if (!v || v->isRemoved()) return true;
        if (v->forbidden || v->isAnchor()) return true;
    }
    return false;
}

bool touchesRoundSymbolState(MCGAL::Halfedge* h) {
    if (!h || !h->opposite()) return true;
    MCGAL::Halfedge* hs[6] = {
        h,
        h->next(),
        h->next() ? h->next()->next() : nullptr,
        h->opposite(),
        h->opposite()->next(),
        (h->opposite() && h->opposite()->next()) ? h->opposite()->next()->next() : nullptr};
    for (MCGAL::Halfedge* e : hs) {
        if (!e || e->isRemoved()) return true;
        if (e->isAdded() || (e->opposite() && e->opposite()->isAdded())) return true;
        MCGAL::Facet* f = e->face();
        if (f && !f->isRemoved() && f->isSplittable()) return true;
        MCGAL::Facet* of = (e->opposite()) ? e->opposite()->face() : nullptr;
        if (of && !of->isRemoved() && of->isSplittable()) return true;
    }
    return false;
}

int localRoleOf(MCGAL::Halfedge* target,
                MCGAL::Halfedge* h,
                MCGAL::Halfedge* h_bc,
                MCGAL::Halfedge* h_ca,
                MCGAL::Halfedge* ho,
                MCGAL::Halfedge* h_ad,
                MCGAL::Halfedge* h_db) {
    if (!target) return -1;
    if (target == h) return 0;
    if (target == h_bc) return 1;
    if (target == h_ca) return 2;
    if (target == ho) return 3;
    if (target == h_ad) return 4;
    if (target == h_db) return 5;
    return -1;
}

}  // namespace

void EdgeFlipPostProcessOperator::process(std::shared_ptr<MCGAL::Mesh> mesh) {
    flipSymbols_.clear();
    flipCoordSymbols_.clear();
    flipMarkedEdges_.clear();
    roundFlipCounts_.clear();
    roundCandidateBitCounts_.clear();
    roundCandidateBits_.clear();
    protectedVertexIds_.clear();
    if (!mesh) return;

    // Optional scope lock: do not flip anywhere near a boundary.  A "boundary"
    // here is either a mesh boundary edge (no opposite face) or a group seam
    // (adjacent live faces with different groupIds).  Flips at/adjacent to those
    // vertices are what make the segmented decoder's replay state diverge from
    // the encoder (split ordering / groupId tie-break), so excluding them keeps
    // the flip transform inside a region where the reconstruction is unambiguous.
    //
    // PMSF_EDGEFLIP_LOCK_BOUNDARY: unset/0 = off, 1 = lock the boundary vertices
    // themselves, 2 = lock the boundary vertices + their one-ring neighbours.
    int boundaryLockScope = 0;
    if (const char* lockEnv = std::getenv("PMSF_EDGEFLIP_LOCK_BOUNDARY")) {
        boundaryLockScope = std::atoi(lockEnv);
        if (boundaryLockScope != 0) boundaryLockScope = std::max(1, boundaryLockScope);
    }
    std::unordered_set<int> boundaryLockVertices;
    if (boundaryLockScope > 0) {
        auto addLockedVertex = [&](MCGAL::Vertex* v) {
            if (!v || v->isRemoved()) return;
            const int id = stableVertexId(v);
            if (id >= 0) boundaryLockVertices.insert(id);
        };
        std::vector<MCGAL::Vertex*> directBoundaryVertices;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved()) continue;
            for (MCGAL::Halfedge* h : *f) {
                if (!h || h->isRemoved()) continue;
                MCGAL::Halfedge* ho = h->opposite();
                bool boundary = (!ho || ho->isRemoved() || !ho->face());
                if (!boundary && ho->face()->groupId() != f->groupId()) boundary = true;
                if (!boundary) continue;
                for (MCGAL::Vertex* v : {h->vertex(), h->end_vertex()}) {
                    if (!v || v->isRemoved()) continue;
                    directBoundaryVertices.push_back(v);
                }
            }
        }
        for (MCGAL::Vertex* v : directBoundaryVertices) addLockedVertex(v);
        if (boundaryLockScope >= 2) {
            for (MCGAL::Vertex* v : directBoundaryVertices) {
                for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
                    if (!h || h->isRemoved()) continue;
                    addLockedVertex(h->end_vertex());
                }
            }
        }
        if (std::getenv("PMSF_EDGEFLIP_DEBUG")) {
            std::fprintf(stderr, "[ef-lock] scope=%d lockedVerts=%zu\n", boundaryLockScope,
                         boundaryLockVertices.size());
        }
    }
    auto touchesBoundaryLock = [&](MCGAL::Halfedge* h) {
        if (boundaryLockVertices.empty() || !h) return false;
        MCGAL::Halfedge* ho = h->opposite();
        MCGAL::Vertex* vs[4] = {h->vertex(), h->end_vertex(),
                                h->next() ? h->next()->end_vertex() : nullptr,
                                (ho && ho->next()) ? ho->next()->end_vertex() : nullptr};
        for (MCGAL::Vertex* v : vs) {
            const int id = stableVertexId(v);
            if (id >= 0 && boundaryLockVertices.count(id)) return true;
        }
        return false;
    };

    struct Candidate {
        MCGAL::Halfedge* h = nullptr;
        double oldLen2 = 0.0;
        double newLen2 = 0.0;
    };

    std::set<std::array<int, 3>> lockedTriangles;
    auto triangleKey = [](MCGAL::Facet* f) -> std::array<int, 3> {
        std::array<int, 3> k{INT_MAX, INT_MAX, INT_MAX};
        if (!f || f->isRemoved()) return k;
        int n = 0;
        for (MCGAL::Halfedge* h : *f) {
            if (!h || h->isRemoved() || !h->vertex()) continue;
            k[static_cast<std::size_t>(n++)] = h->vertex()->poolId();
            if (n >= 3) break;
        }
        std::sort(k.begin(), k.end());
        return k;
    };

    std::unordered_set<int> seedNeighborhood;
    if (seedVid0_ >= 0 || seedVid1_ >= 0) {
        for (MCGAL::Vertex* v : mesh->vertices()) {
            if (!v || v->isRemoved()) continue;
            const int id = stableVertexId(v);
            if (id != seedVid0_ && id != seedVid1_) continue;
            seedNeighborhood.insert(id);
            std::vector<MCGAL::Vertex*> firstRing;
            for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
                if (!h || h->isRemoved() || !h->end_vertex()) continue;
                const int nid = stableVertexId(h->end_vertex());
                if (nid >= 0 && seedNeighborhood.insert(nid).second) {
                    firstRing.push_back(h->end_vertex());
                }
            }
            for (MCGAL::Vertex* nv : firstRing) {
                for (MCGAL::Halfedge* h2 : pmsf::boundedRing(nv)) {
                    if (!h2 || h2->isRemoved() || !h2->end_vertex()) continue;
                    const int nid2 = stableVertexId(h2->end_vertex());
                    if (nid2 >= 0) seedNeighborhood.insert(nid2);
                }
            }
        }
    }
    auto touchesSeedNeighborhood = [&](MCGAL::Halfedge* h) {
        if (seedNeighborhood.empty() || !h) return false;
        MCGAL::Halfedge* ho = h->opposite();
        MCGAL::Vertex* vs[4] = {h->vertex(), h->end_vertex(),
                                h->next() ? h->next()->end_vertex() : nullptr,
                                (ho && ho->next()) ? ho->next()->end_vertex() : nullptr};
        for (MCGAL::Vertex* v : vs) {
            const int id = stableVertexId(v);
            if (id >= 0 && seedNeighborhood.count(id)) return true;
        }
        return false;
    };

    const int rounds = cfg_.rounds > 0 ? cfg_.rounds : 1;
    for (int round = 0; round < rounds; ++round) {
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved()) continue;
            for (MCGAL::Halfedge* h : *f) {
                if (!h || h->isRemoved()) continue;
                h->clearFlipTag();
            }
        }

        if (cfg_.exportSnapshots && !cfg_.snapshotDir.empty()) {
            std::filesystem::create_directories(cfg_.snapshotDir);
            char pathBefore[512];
            std::snprintf(pathBefore, sizeof(pathBefore), "%s/flip_round_%03d_before.off",
                          cfg_.snapshotDir.c_str(), round);
            mesh->dumpto_oldtype(pathBefore);
        }

        std::vector<Candidate> candidates;
        candidates.reserve(mesh->size_of_facets() * 2);
        std::unordered_set<std::uint64_t> seen;
        seen.reserve(mesh->size_of_facets() * 4 + 1);

        double sumLen = 0.0;
        int edgeCount = 0;

        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved()) continue;
            for (MCGAL::Halfedge* h : *f) {
                if (!isTrianglePairFlippable(h)) continue;
                const std::uint64_t key = undirectedEdgeKey(h);
                if (!seen.insert(key).second) continue;
                if (touchesProtectedSeed(h, seedVid0_, seedVid1_) || touchesSeedNeighborhood(h)) continue;
                if (touchesForbiddenVertex(h)) continue;
                if (touchesBoundaryLock(h)) continue;
                if (touchesRoundSymbolState(h)) continue;
                if (!is_diamond_convex(h)) continue;

                MCGAL::Vertex* a = h->vertex();
                MCGAL::Vertex* b = h->end_vertex();
                MCGAL::Vertex* c = h->next()->end_vertex();
                MCGAL::Halfedge* ho = h->opposite();
                MCGAL::Vertex* d = ho->next()->end_vertex();
                if (c->find_halfedge(d) || d->find_halfedge(c)) continue;

                Candidate cnd;
                cnd.h = h;
                cnd.oldLen2 = edgeLengthSquared(a, b);
                cnd.newLen2 = edgeLengthSquared(c, d);
                if (cnd.oldLen2 <= 0.0) continue;
                candidates.push_back(cnd);
                sumLen += std::sqrt(cnd.oldLen2);
                ++edgeCount;
            }
        }

        if (edgeCount == 0) break;
        const double avgLen = sumLen / static_cast<double>(edgeCount);
        const double lengthThreshold = avgLen * cfg_.longEdgeRatio;
        const double lengthThreshold2 = lengthThreshold * lengthThreshold;

        std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
            return lhs.oldLen2 > rhs.oldLen2;
        });

        std::unordered_set<int> lockedVertices;
        lockedVertices.reserve(candidates.size() * 2 + 4);
        std::unordered_set<std::uint64_t> blockedEdges;
        blockedEdges.reserve(candidates.size() * 8 + 16);
        std::vector<CoordSymbol> roundCoordSymbols;
        roundCoordSymbols.reserve(candidates.size());
        const std::vector<MCGAL::Halfedge*> preFlipEdges =
            pmsf::edgeflip::collectEdgeFlipCandidatesBfs(mesh);
        auto edgeCoordKey = [&](MCGAL::Halfedge* e) {
            std::array<long long, 6> k{0,0,0,0,0,0};
            if (!e || !e->vertex() || !e->end_vertex()) return k;
            auto a = symbolPointKey(mesh, e->vertex());
            auto b = symbolPointKey(mesh, e->end_vertex());
            if (b < a) std::swap(a, b);
            k = {a[0],a[1],a[2],b[0],b[1],b[2]};
            return k;
        };
        std::map<std::array<long long,6>, int> preFlipIndex;
        for (int ei = 0; ei < static_cast<int>(preFlipEdges.size()); ++ei) {
            preFlipIndex.emplace(edgeCoordKey(preFlipEdges[static_cast<size_t>(ei)]), ei);
        }
        std::vector<int> flippedIndices;
        std::stable_sort(candidates.begin(), candidates.end(),
                         [&](const Candidate& lhs, const Candidate& rhs) {
                             auto lit = preFlipIndex.find(edgeCoordKey(lhs.h));
                             auto rit = preFlipIndex.find(edgeCoordKey(rhs.h));
                             const int li = lit == preFlipIndex.end() ? INT_MAX : lit->second;
                             const int ri = rit == preFlipIndex.end() ? INT_MAX : rit->second;
                             return li < ri;
                         });

        int flipsThisRound = 0;
        const int roundCap = cfg_.maxFlipsPerRoundPass > 0 ? cfg_.maxFlipsPerRoundPass : -1;
        for (const Candidate& cnd : candidates) {
            MCGAL::Halfedge* h = cnd.h;
            if (!isTrianglePairFlippable(h)) continue;
            if (!is_diamond_convex(h)) continue;
            if (touchesProtectedSeed(h, seedVid0_, seedVid1_) || touchesSeedNeighborhood(h)) continue;
            if (touchesForbiddenVertex(h)) continue;
            if (touchesBoundaryLock(h)) continue;
            if (touchesRoundSymbolState(h)) continue;

            MCGAL::Halfedge* ho = h->opposite();
            MCGAL::Vertex* a = h->vertex();
            MCGAL::Vertex* b = h->end_vertex();
            MCGAL::Vertex* c = h->next()->end_vertex();
            MCGAL::Vertex* d = ho->next()->end_vertex();
            if (!a || !b || !c || !d) continue;
            if (lockedTriangles.count(triangleKey(h->face())) ||
                lockedTriangles.count(triangleKey(ho->face()))) {
                continue;
            }
            MCGAL::Halfedge* h_n = h->next();
            MCGAL::Halfedge* h_nn = h_n ? h_n->next() : nullptr;
            MCGAL::Halfedge* ho_n = ho->next();
            MCGAL::Halfedge* ho_nn = ho_n ? ho_n->next() : nullptr;
            const std::uint64_t k_center = undirectedEdgeKey(h);
            const std::uint64_t k1 = undirectedEdgeKey(h_n);
            const std::uint64_t k2 = undirectedEdgeKey(h_nn);
            const std::uint64_t k3 = undirectedEdgeKey(ho_n);
            const std::uint64_t k4 = undirectedEdgeKey(ho_nn);
            if ((k_center && blockedEdges.count(k_center)) ||
                (k1 && blockedEdges.count(k1)) ||
                (k2 && blockedEdges.count(k2)) ||
                (k3 && blockedEdges.count(k3)) ||
                (k4 && blockedEdges.count(k4))) {
                continue;
            }

            const int ids[4] = {
                stableVertexId(a), stableVertexId(b), stableVertexId(c), stableVertexId(d)};
            std::vector<int> ringIds;
            ringIds.reserve(64);
            auto appendRingIds = [&](MCGAL::Vertex* v) {
                if (!v || v->isRemoved()) return;
                const int id = stableVertexId(v);
                if (id >= 0) ringIds.push_back(id);
                for (MCGAL::Halfedge* vh : pmsf::boundedRing(v)) {
                    if (!vh || vh->isRemoved()) continue;
                    MCGAL::Vertex* nb = vh->end_vertex();
                    const int nbId = stableVertexId(nb);
                    if (nbId >= 0) ringIds.push_back(nbId);
                }
            };
            for (MCGAL::Vertex* v : {a, b, c, d}) {
                appendRingIds(v);
            }
            bool ringLocked = false;
            for (int id : ringIds) {
                if (lockedVertices.count(id)) {
                    ringLocked = true;
                    break;
                }
            }
            if (ringLocked && !std::getenv("PMSF_EDGEFLIP_EDGE_LOCK_ONLY")) {
                continue;
            }

            const double oldLen2 = edgeLengthSquared(a, b);
            const double newLen2 = edgeLengthSquared(c, d);
            if (oldLen2 <= lengthThreshold2) continue;
            if (newLen2 >= oldLen2 * cfg_.minImprovementRatio) continue;
            if (c->find_halfedge(d) || d->find_halfedge(c)) continue;

            const int idA = stableVertexId(a);
            const int idB = stableVertexId(b);
            const int idC = stableVertexId(c);
            const int idD = stableVertexId(d);
            const int f0ProxyRole = localRoleOf(h->face() ? h->face()->proxyHalfedge() : nullptr,
                                                h, h_n, h_nn, ho, ho_n, ho_nn);
            const int f1ProxyRole = localRoleOf(ho->face() ? ho->face()->proxyHalfedge() : nullptr,
                                                h, h_n, h_nn, ho, ho_n, ho_nn);
            const int vaProxyRole = localRoleOf(a->proxyHalfedge(), h, h_n, h_nn, ho, ho_n, ho_nn);
            const int vbProxyRole = localRoleOf(b->proxyHalfedge(), h, h_n, h_nn, ho, ho_n, ho_nn);
            const int vcProxyRole = localRoleOf(c->proxyHalfedge(), h, h_n, h_nn, ho, ho_n, ho_nn);
            const int vdProxyRole = localRoleOf(d->proxyHalfedge(), h, h_n, h_nn, ho, ho_n, ho_nn);

            if (const auto it = preFlipIndex.find(edgeCoordKey(h)); it != preFlipIndex.end()) {
                flippedIndices.push_back(it->second);
            }
            mesh->edge_flip(h);
            if (h->face()) lockedTriangles.insert(triangleKey(h->face()));
            if (h->opposite() && h->opposite()->face()) {
                lockedTriangles.insert(triangleKey(h->opposite()->face()));
            }
            h->setFlipTag();
            if (h->opposite()) h->opposite()->setFlipTag();
            if (k_center) blockedEdges.insert(k_center);
            if (k1) blockedEdges.insert(k1);
            if (k2) blockedEdges.insert(k2);
            if (k3) blockedEdges.insert(k3);
            if (k4) blockedEdges.insert(k4);

            auto mapId = [&](MCGAL::Vertex* v) -> int {
                if (v == a) return idA;
                if (v == b) return idB;
                if (v == c) return idC;
                if (v == d) return idD;
                return stableVertexId(v);
            };

            MCGAL::Vertex* nv0 = h->vertex();
            MCGAL::Vertex* nv1 = h->end_vertex();
            const int sid0 = mapId(nv0);
            const int sid1 = mapId(nv1);
            if (sid0 >= 0 && sid1 >= 0) {
                flipSymbols_.emplace_back(sid0, sid1);
                flipMarkedEdges_.insert(normalizedEdgeId(sid0, sid1));
            }
            if (h->opposite() && h->next() && h->opposite()->next()) {
                CoordSymbol sym;
                sym.a = symbolPointKey(mesh, h->vertex());
                sym.b = symbolPointKey(mesh, h->end_vertex());
                sym.oa = symbolPointKey(mesh, h->next()->end_vertex());
                sym.ob = symbolPointKey(mesh, h->opposite()->next()->end_vertex());
                sym.f0Proxy = f0ProxyRole;
                sym.f1Proxy = f1ProxyRole;
                sym.vaProxy = vaProxyRole;
                sym.vbProxy = vbProxyRole;
                sym.vcProxy = vcProxyRole;
                sym.vdProxy = vdProxyRole;
                roundCoordSymbols.push_back(sym);
            }
            for (int id : ringIds) {
                if (id >= 0) lockedVertices.insert(id);
            }
            auto protectVertex = [&](MCGAL::Vertex* v) {
                if (!v || v->isRemoved()) return;
                v->setAnchor();
                v->forbidden = true;
                const int vid = stableVertexId(v);
                if (vid >= 0) protectedVertexIds_.push_back(vid);
            };
            auto protectOneRing = [&](MCGAL::Vertex* v) {
                protectVertex(v);
                if (!v || v->isRemoved()) return;
                for (MCGAL::Halfedge* hv : pmsf::boundedRing(v)) {
                    if (!hv || hv->isRemoved()) continue;
                    protectVertex(hv->end_vertex());
                }
            };
            const char* protectPatchEnv = std::getenv("PMSF_EDGEFLIP_PROTECT_PATCH");
            if (protectPatchEnv && *protectPatchEnv && std::string(protectPatchEnv) != "0") {
                for (MCGAL::Vertex* v : {a, b, c, d}) {
                    protectVertex(v);
                }
            }
            ++flipsThisRound;
            if (roundCap > 0 && flipsThisRound >= roundCap) {
                break;
            }
        }


        roundFlipCounts_.push_back(flipsThisRound);
        if (std::getenv("PMSF_EDGEFLIP_DEBUG")) {
            std::cerr << "[ef-round] enc round=" << round << " flips=" << flipsThisRound
                      << " F=" << mesh->size_of_facets() << " V=" << mesh->size_of_vertices()
                      << std::endl;
        }
        // The 1-bit all-edge stream must be addressable by the decoder without any
        // shared BFS root history: canonical (endpoint-q sorted) order is stable
        // across encode/decode face allocations.  The seed-BFS order remains the
        // default for legacy payloads (PMSF_EDGEFLIP_BITS_CANONICAL unset).
        const bool canonicalBitOrder =
            std::getenv("PMSF_EDGEFLIP_BITS_CANONICAL") != nullptr;
        const std::vector<MCGAL::Halfedge*> postFlipEdges =
            canonicalBitOrder ? pmsf::edgeflip::collectAllEdgesCanonical(mesh)
            : (seedHalfedge_ ? pmsf::edgeflip::collectAllEdgesSeedBfs(mesh, seedHalfedge_)
                             : pmsf::edgeflip::collectAllEdgesCanonical(mesh));
        roundCandidateBitCounts_.push_back(static_cast<int>(postFlipEdges.size()));
        std::vector<unsigned char> roundBits(static_cast<size_t>(postFlipEdges.size()), 0);
        for (size_t bi = 0; bi < postFlipEdges.size(); ++bi) {
            MCGAL::Halfedge* e = postFlipEdges[bi];
            const bool tag = e && (e->flipTag() || (e->opposite() && e->opposite()->flipTag()));
            roundBits[bi] = tag ? 1 : 0;
        }
        roundCandidateBits_.insert(roundCandidateBits_.end(), roundBits.begin(), roundBits.end());
        if (!std::getenv("PMSF_EDGEFLIP_BITS_ONLY")) {
            flipCoordSymbols_.insert(flipCoordSymbols_.end(),
                                     roundCoordSymbols.begin(), roundCoordSymbols.end());
        }

        if (cfg_.exportSnapshots && !cfg_.snapshotDir.empty()) {
            char pathAfter[512];
            std::snprintf(pathAfter, sizeof(pathAfter), "%s/flip_round_%03d_after.off",
                          cfg_.snapshotDir.c_str(), round);
            mesh->dumpto_oldtype(pathAfter);
        }

        if (flipsThisRound == 0) break;
    }
}

void EdgeFlipPostProcessOperator::processWithGraph(std::shared_ptr<MCGAL::Mesh> mesh,
                                                   std::shared_ptr<Graph> graph) {
    (void)graph;
    process(std::move(mesh));
}
