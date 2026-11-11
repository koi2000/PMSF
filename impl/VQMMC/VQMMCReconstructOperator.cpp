#include "VQMMCReconstructOperator.h"

#include "Facet.h"
#include "Halfedge.h"
#include "MeshUtils.h"
#include "Point.h"
#include "Vertex.h"

#include <cstdlib>
#include <iostream>
#include <unordered_set>

namespace {

bool envOn(const char* name) {
    const char* v = std::getenv(name);
    return v && *v && v[0] != '0';
}

bool isRoundDiagonal(MCGAL::Halfedge* h,
                     const std::unordered_set<MCGAL::Halfedge*>& roundDiagonals) {
    if (!h || h->isRemoved()) return false;
    MCGAL::Halfedge* opp = h->opposite();
    if (!opp || opp->isRemoved()) return false;
    MCGAL::Facet* f0 = h->face();
    MCGAL::Facet* f1 = opp->face();
    if (!f0 || !f1 || f0->isRemoved() || f1->isRemoved()) return false;
    return roundDiagonals.find(h) != roundDiagonals.end() ||
           roundDiagonals.find(opp) != roundDiagonals.end();
}

bool mergeNextDiagonalAround(MCGAL::Facet* root,
                             const std::unordered_set<MCGAL::Halfedge*>& roundDiagonals) {
    if (!root || root->isRemoved() || !root->proxyHalfedge()) return false;

    MCGAL::Halfedge* start = root->proxyHalfedge();
    MCGAL::Halfedge* h = start;
    int guard = 0;
    do {
        if (h->face() == root && isRoundDiagonal(h, roundDiagonals)) {
            MCGAL::Halfedge* hprev = MCGAL::find_prev(h);
            MCGAL::Halfedge* gprev = MCGAL::find_prev(h->opposite());
            if (!hprev || !gprev || hprev->isRemoved() || gprev->isRemoved() ||
                hprev == h || hprev == h->opposite() ||
                gprev == h || gprev == h->opposite() ||
                !h->face() || !h->opposite()->face() ||
                h->face()->facet_degree() <= 2 ||
                h->opposite()->face()->facet_degree() <= 2) {
                h = h->next();
                if (++guard > 4096) break;
                continue;
            }
            MCGAL::join_face(h);
            return true;
        }
        h = h->next();
        if (++guard > 4096) break;
    } while (h && h != start);

    return false;
}

}  // namespace

void VQMMCReconstructOperator::reconstruct(std::vector<MCGAL::Vertex*>& vertices,
                                           std::vector<MCGAL::Halfedge*>& halfedge,
                                           std::vector<MCGAL::Facet*>& facets) {
    for (MCGAL::Vertex* v : vertices) {
        if (v) v->setPoint(v->vertexPoint());
    }

    std::unordered_set<MCGAL::Halfedge*> roundDiagonals;
    roundDiagonals.reserve(halfedge.size() * 2 + 16);
    for (MCGAL::Halfedge* h : halfedge) {
        if (!h) continue;
        roundDiagonals.insert(h);
        if (h->opposite()) roundDiagonals.insert(h->opposite());
    }

    const bool diag = envOn("VQMMC_RECON_DIAG");
    std::size_t inserted = 0;
    std::size_t merged = 0;

    for (MCGAL::Facet* root : facets) {
        if (!root || root->isRemoved()) continue;

        int guard = 0;
        while (!root->isRemoved() && mergeNextDiagonalAround(root, roundDiagonals)) {
            ++merged;
            if (++guard > 4096) {
                if (diag) {
                    std::cerr << "[VQMMC-recon] merge guard hit at facet "
                              << root->poolId() << std::endl;
                }
                break;
            }
        }

        if (!root->isRemoved()) {
            insert_point(root);
            ++inserted;
        }
    }

    if (diag) {
        std::cerr << "[VQMMC-recon] facets=" << facets.size()
                  << " inserted=" << inserted
                  << " round_diagonals=" << roundDiagonals.size()
                  << " merged=" << merged << std::endl;
    }
}
