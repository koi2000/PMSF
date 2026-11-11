// SPDX-License-Identifier: Apache-2.0
//
// PCATMReconstructOperator.cpp

#include "PCATMReconstructOperator.h"

#include "Facet.h"
#include "Halfedge.h"
#include "MeshUtils.h"
#include "Vertex.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

constexpr uint8_t kPCATMPaperProtocolHint = 20;

bool hasPCATMPaperFacet(const std::vector<MCGAL::Facet*>& facets) {
    for (MCGAL::Facet* f : facets) {
        if (f && f->getProtocolHint() == kPCATMPaperProtocolHint) return true;
    }
    return false;
}

MCGAL::Facet* mergeOnePCATMPatchDiagonal(
    MCGAL::Facet* root,
    const std::vector<MCGAL::Halfedge*>& candidateDiagonals,
    MCGAL::Facet** mergedNeighbor) {
    if (!root || root->isRemoved()) return nullptr;
    if (mergedNeighbor) *mergedNeighbor = nullptr;

    for (MCGAL::Halfedge* h : candidateDiagonals) {
        if (!h || h->isRemoved() || !h->opposite() || h->opposite()->isRemoved()) {
            continue;
        }

        MCGAL::Facet* fa = h->face();
        MCGAL::Facet* fb = h->opposite()->face();
        if (!fa || !fb || fa->isRemoved() || fb->isRemoved()) continue;
        if (fa != root && fb != root) continue;
        if (mergedNeighbor) *mergedNeighbor = (fa == root) ? fb : fa;

        const int polyValence = root->getPolyValence();
        const auto removedPos = root->getRemovedVertexPos();
        const auto removedPosInt = root->getRemovedVertexPosInt();
        const uint8_t protocolHint = root->getProtocolHint();

        MCGAL::Halfedge* keptHalfedge = MCGAL::join_face(fa == root ? h : h->opposite());
        MCGAL::Facet* kept = keptHalfedge ? keptHalfedge->face() : nullptr;
        if (!kept || kept->isRemoved()) return nullptr;

        kept->setPolyValence(polyValence);
        kept->setRemovedVertexPos(removedPos);
        kept->setRemovedVertexPosInt(removedPosInt);
        kept->setProtocolHint(protocolHint);
        if (!kept->isSplittable()) kept->setSplittable();
        return kept;
    }

    return nullptr;
}

}  // namespace

void PCATMReconstructOperator::reconstruct(
    std::vector<MCGAL::Vertex*>& vertices,
    std::vector<MCGAL::Halfedge*>& halfedges,
    std::vector<MCGAL::Facet*>& facets) {

    if (!hasPCATMPaperFacet(facets)) {
        BasicReconstructOperator::reconstruct(vertices, halfedges, facets);
        return;
    }

    for (MCGAL::Vertex* v : vertices) {
        if (v) v->setPoint(v->vertexPoint());
    }

    std::unordered_map<MCGAL::Facet*, std::vector<MCGAL::Halfedge*>> diagonalsByFace;
    diagonalsByFace.reserve(halfedges.size() * 2 + 16);
    for (MCGAL::Halfedge* h : halfedges) {
        if (!h) continue;
        if (MCGAL::Facet* f = h->face(); f && !f->isRemoved()) {
            diagonalsByFace[f].push_back(h);
        }
        MCGAL::Halfedge* opp = h->opposite();
        if (opp) {
            if (MCGAL::Facet* f = opp->face(); f && !f->isRemoved()) {
                diagonalsByFace[f].push_back(opp);
            }
        }
    }

    for (MCGAL::Facet* f : facets) {
        if (!f || f->getProtocolHint() != kPCATMPaperProtocolHint) continue;

        MCGAL::Facet* root = f;
        const int targetDegree = root->getPolyValence();
        while (root && !root->isRemoved() && root->facet_degree() < targetDegree) {
            auto it = diagonalsByFace.find(root);
            if (it == diagonalsByFace.end()) break;
            MCGAL::Facet* oldRoot = root;
            MCGAL::Facet* mergedNeighbor = nullptr;
            MCGAL::Facet* next = mergeOnePCATMPatchDiagonal(root, it->second, &mergedNeighbor);
            if (!next) break;
            auto appendCandidates = [&](MCGAL::Facet* from) {
                if (!from || from == next) return;
                auto srcIt = diagonalsByFace.find(from);
                if (srcIt == diagonalsByFace.end() || srcIt->second.empty()) return;
                auto& dst = diagonalsByFace[next];
                dst.insert(dst.end(), srcIt->second.begin(), srcIt->second.end());
            };
            appendCandidates(oldRoot);
            appendCandidates(mergedNeighbor);
            root = next;
        }

        insert_point(root);
    }
}
