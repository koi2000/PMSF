/*
 * @Author: koi
 * @Date: 2025-09-02 15:45:26
 * @Description:
 */
#include "SegVertexReconstructOperator.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "MeshUtils.h"

namespace {

MCGAL::Halfedge* safePrevInFace(MCGAL::Halfedge* h) {
    if (!h || !h->face() || !h->face()->proxyHalfedge()) return nullptr;
    MCGAL::Halfedge* start = h->face()->proxyHalfedge();
    MCGAL::Halfedge* it = start;
    int guard = 0;
    while (it && it->next() && it->next() != h) {
        it = it->next();
        if (it == start || ++guard > 4096) return nullptr;
    }
    return (it && it->next() == h) ? it : nullptr;
}

}  // namespace

void SegVertexReconstructOperator::reconstruct(MCGAL::Halfedge* halfedge) {}

void SegVertexReconstructOperator::reconstruct(std::vector<MCGAL::Vertex*>& vertices,
                                               std::vector<MCGAL::Halfedge*>& halfedge,
                                               std::vector<MCGAL::Facet*>& facets) {
    int count = 0;
    for (MCGAL::Halfedge* hit : halfedge) {
        merge_face(hit);
        count++;
    }
    for (MCGAL::Halfedge* hit : halfedge) {
        if (hit->opposite() == nullptr) {
            if (!hit->next()) continue;
            if (!hit->face()->isSplittable()) {
                hit->face()->setRemoved();
            }
            MCGAL::Halfedge* hstart = hit->next();
            MCGAL::Halfedge* hend = safePrevInFace(hit);
            if (!hstart || !hend || !hstart->face()) continue;
            hstart->face()->setProxyHalfedge(hstart);
            hend->setNext(nullptr);
            hit->setRemoved();
            // MCGAL::Halfedge* st = hit;
            // MCGAL::Halfedge* ed = hit;
            // do {
            //     if (st->opposite() != nullptr) {
            //         st->opposite()->setOpposite(nullptr);
            //     }
            //     st->setRemoved();
            //     st = st->next();
            // } while (st != ed);
        }
    }
    count = 0;
    for (MCGAL::Facet* f : facets) {
        insert_point(f);
        count++;
    }
}

void SegVertexReconstructOperator::merge_face(MCGAL::Halfedge* halfedge) {
    if (halfedge->isRemoved() || halfedge->opposite() == nullptr) {
        return;
    }
    MCGAL::join_face(halfedge);
}

/**
    看不懂在做什么，明天继续debug
 */
void SegVertexReconstructOperator::insert_point(MCGAL::Facet* facet) {
    if (!mesh_ || !facet || facet->isRemoved() ||
        !facet->proxyHalfedge() || !facet->proxyHalfedge()->next()) {
        return;
    }
    mesh_->create_center_vertex_plus(facet->proxyHalfedge(), facet->getRemovedVertexPos());
}
