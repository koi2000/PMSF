/*
 * @Author: Claude (PCATM alignment)
 * @Date: 2026-04-29
 * @Description: Independent Set Enforcement for PCATM vertex removal
 *
 * This selector enforces the Maximal Independent Set (MIS) constraint as specified
 * in Cohen-Or, Levin, Remez 1999 "Progressive Compression of Arbitrary Triangular Meshes".
 *
 * Key constraint: No two removed vertices in a single round can share an edge
 * (i.e., must be non-adjacent). This ensures all polygon decompositions are
 * independent during BFS collection and enables proper triangle 2-coloring
 * for context-aware entropy encoding.
 *
 * Algorithm:
 * 1. Maintain a set of "removed" vertices in current round
 * 2. Before accepting a candidate halfedge for removal, check that neither
 *    endpoint is already in the removed set
 * 3. Upon removal, add the halfedge's end_vertex to the removed set
 * 4. On round boundary (reset()), clear the removed set
 */
#ifndef INDEPENDENT_SET_SELECTOR_H
#define INDEPENDENT_SET_SELECTOR_H

#include <set>
#include "common/BoundedRingTraversal.h"
#include <unordered_set>
#include <memory>
#include <cstdlib>
#include <cstring>
#include <vector>
#include "Facet.h"
#include "Global.h"
#include "Halfedge.h"
#include "Vertex.h"
#include "Mesh.h"
#include "../common/HalfedgeErrorEntry.h"

/**
 * @brief Independent Set selector wrapper for PriorityErrorSelector
 *
 * This class tracks which vertices have been removed in the current round
 * and validates adjacency constraints before allowing removal.
 *
 * Usage:
 *   IndependentSetSelector mis_selector;
 *   mis_selector.init(mesh);
 *   
 *   while (selector->select(entry)) {
 *       if (mis_selector.canRemove(entry)) {
 *           eliminate(entry);
 *           mis_selector.markRemoved(entry.edge->end_vertex());
 *       }
 *   }
 *   mis_selector.reset();  // For next round
 */
class IndependentSetSelector {
  public:
    IndependentSetSelector() = default;
    ~IndependentSetSelector() = default;

    /**
     * @brief Initialize selector with mesh reference
     * @param mesh The mesh being processed
     */
    void init(std::shared_ptr<MCGAL::Mesh> mesh) {
        mesh_ = mesh;
        // 预 reserve 避免 canRemove 热路径触发 rehash。bunny 第一轮会删到 ~5500 顶点，
        // 大多数轮次 < 1/6·|V|，这里 reserve 顶点数的一半作为上界更安全。
        removed_vertices_.clear();
        if (mesh) {
            removed_vertices_.reserve(mesh->vertices().size() / 2 + 16);
        }
    }

    /**
     * @brief Clear removed vertex set for new round
     */
    void clear() {
        removed_vertices_.clear();
        forbidden_vertices_.clear();
    }

    /**
     * @brief Check if a halfedge can be removed given current MIS state
     * @param entry The halfedge error entry to validate
     * @return true if neither endpoint is already removed in this round
     *
     * This checks:
     * 1. entry.edge is not nullptr
     * 2. entry.edge->end_vertex() is not already removed in this round
     * 3. none of entry.edge->end_vertex()'s live one-ring neighbors has been
     *    removed in this round
     *
     * This enforces the paper's independent-set constraint: no two removed
     * vertices in the same round share an edge.
     */
    bool canRemove(const HalfedgeErrorEntry& entry) const {
        if (entry.edge == nullptr) {
            return false;
        }

        MCGAL::Halfedge* h = entry.edge;
        MCGAL::Vertex* v2 = h->end_vertex();

        if (v2 == nullptr) {
            return false;
        }

        // In PCATM, we remove end_vertex (v2).
        if (removed_vertices_.count(v2->poolId()) > 0) {
            return false;
        }
        if (forbidden_vertices_.count(v2->poolId()) > 0) {
            return false;
        }

        for (MCGAL::Halfedge* hh : pmsf::boundedRing(v2)) {
            if (!hh || hh->isRemoved()) continue;
            MCGAL::Vertex* nb = hh->end_vertex();
            if (!nb) continue;
            if (removed_vertices_.count(nb->poolId()) > 0) {
                return false;
            }
        }

        // perf（2026-11-09）：两个 MIS 严格化开关原先每个候选判定一次 getenv。
        static const bool enableDistance2 = [] {
            const char* e = std::getenv("PMSF_PCATM_STRICT_DISTANCE2_MIS");
            return e && *e && std::strcmp(e, "0") != 0;
        }();
        if (enableDistance2) {
            for (MCGAL::Halfedge* hh : pmsf::boundedRing(v2)) {
                if (!hh || hh->isRemoved()) continue;
                MCGAL::Vertex* nb = hh->end_vertex();
                if (!nb || nb->isRemoved()) continue;
                for (MCGAL::Halfedge* nh : pmsf::boundedRing(nb)) {
                    if (!nh || nh->isRemoved()) continue;
                    MCGAL::Vertex* nb2 = nh->end_vertex();
                    if (!nb2 || nb2 == v2) continue;
                    if (removed_vertices_.count(nb2->poolId()) > 0) {
                        return false;
                    }
                }
            }
        }

        static const bool enableOppositeEdge = [] {
            const char* e = std::getenv("PMSF_PCATM_STRICT_OPPOSITE_EDGE_MIS");
            return e && *e && std::strcmp(e, "0") != 0;
        }();
        if (enableOppositeEdge && hasRemovedOppositeAcrossIncidentEdge(v2)) {
            return false;
        }

        return true;
    }

    /**
     * @brief Mark a vertex as removed in the current round
     * @param v The vertex that was just removed
     */
    void markRemoved(MCGAL::Vertex* v) {
        if (v != nullptr) {
            removed_vertices_.insert(v->poolId());
        }
    }

    /**
     * @brief Mark a removed center and preserve its current one-ring as
     * forbidden centers for later same-round refill passes.
     */
    void markRemovedAndNeighbors(MCGAL::Vertex* v) {
        if (v == nullptr) return;
        removed_vertices_.insert(v->poolId());
        for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
            if (!h || h->isRemoved()) continue;
            MCGAL::Vertex* nb = h->end_vertex();
            if (nb) forbidden_vertices_.insert(nb->poolId());
        }
    }

    /**
     * @brief Get count of vertices removed so far in this round
     */
    int removedCount() const {
        return static_cast<int>(removed_vertices_.size());
    }

    /**
     * @brief Check if a vertex has been removed in this round
     */
    bool isRemoved(MCGAL::Vertex* v) const {
        if (v == nullptr) return false;
        return removed_vertices_.count(v->poolId()) > 0;
    }

    /**
     * @brief Check if a vertex has been removed in this round (by poolId)
     */
    bool isRemoved(int poolId) const {
        return removed_vertices_.count(poolId) > 0;
    }

  private:
    static bool faceHasVertex(MCGAL::Facet* f, MCGAL::Vertex* v) {
        if (!f || !v || f->isRemoved()) return false;
        for (MCGAL::Vertex* fv : f->getVertices()) {
            if (fv == v) return true;
        }
        return false;
    }

    static MCGAL::Halfedge* edgeInFace(MCGAL::Facet* f,
                                       MCGAL::Vertex* a,
                                       MCGAL::Vertex* b) {
        if (!f || !a || !b || !f->proxyHalfedge()) return nullptr;
        MCGAL::Halfedge* start = f->proxyHalfedge();
        MCGAL::Halfedge* h = start;
        int guard = 0;
        do {
            if (!h) break;
            if ((h->vertex() == a && h->end_vertex() == b) ||
                (h->vertex() == b && h->end_vertex() == a)) {
                return h;
            }
            h = h->next();
            if (++guard > 128) break;
        } while (h && h != start);
        return nullptr;
    }

    MCGAL::Vertex* oppositeVertexAcrossEdge(MCGAL::Facet* f,
                                            MCGAL::Vertex* a,
                                            MCGAL::Vertex* b,
                                            MCGAL::Vertex* center) const {
        MCGAL::Halfedge* edge = edgeInFace(f, a, b);
        MCGAL::Facet* nf = (edge && edge->opposite()) ? edge->opposite()->face() : nullptr;
        if (!nf || nf->isRemoved() || nf == f || faceHasVertex(nf, center)) {
            return nullptr;
        }

        std::vector<MCGAL::Vertex*> others;
        for (MCGAL::Vertex* v : nf->getVertices()) {
            if (!v || v == a || v == b) continue;
            others.push_back(v);
        }
        return others.size() == 1 ? others.front() : nullptr;
    }

    bool hasRemovedOppositeAcrossIncidentEdge(MCGAL::Vertex* center) const {
        if (!center) return false;

        std::unordered_set<int> visitedFaces;
        auto inspectFace = [&](MCGAL::Facet* f) -> bool {
            if (!f || f->isRemoved() || !visitedFaces.insert(f->poolId()).second) {
                return false;
            }

            std::vector<MCGAL::Vertex*> verts = f->getVertices();
            if (verts.size() != 3 || !faceHasVertex(f, center)) return false;

            MCGAL::Vertex* a = nullptr;
            MCGAL::Vertex* b = nullptr;
            for (MCGAL::Vertex* v : verts) {
                if (v == center) continue;
                if (!a) a = v;
                else b = v;
            }
            if (!a || !b) return false;

            MCGAL::Vertex* opposite = oppositeVertexAcrossEdge(f, a, b, center);
            return opposite && removed_vertices_.count(opposite->poolId()) > 0;
        };

        for (MCGAL::Halfedge* hh : pmsf::boundedRing(center)) {
            if (!hh || hh->isRemoved()) continue;
            if (inspectFace(hh->face())) {
                return true;
            }
            if (hh->opposite() && inspectFace(hh->opposite()->face())) {
                return true;
            }
        }
        return false;
    }

    std::shared_ptr<MCGAL::Mesh> mesh_;
    std::unordered_set<int> removed_vertices_;  // poolIds of removed vertices in current round
    std::unordered_set<int> forbidden_vertices_;
};

#endif  // INDEPENDENT_SET_SELECTOR_H
