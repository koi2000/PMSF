#include "SegmentedDefaultReconstructOperator.h"

#include "Facet.h"
#include "ContextPool.h"
#include "Global.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Vertex.h"
#include "predict/RDPredictor.h"

#include <cstdint>

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

void SegmentedDefaultReconstructOperator::reconstruct(MCGAL::Halfedge* halfedge) {
    BasicReconstructOperator::reconstruct(halfedge);
}

void SegmentedDefaultReconstructOperator::reconstruct(
    std::vector<MCGAL::Vertex*>& vertices,
    std::vector<MCGAL::Halfedge*>& halfedge,
    std::vector<MCGAL::Facet*>& facets) {
    for (MCGAL::Vertex* v : vertices) {
        if (v) v->setPoint(v->vertexPoint());
    }

    for (MCGAL::Halfedge* h : halfedge) {
        if (!h || h->isRemoved() || h->opposite() == nullptr) continue;
        BasicReconstructOperator::merge_face(h);
    }

    std::unordered_set<MCGAL::Facet*> openFacets;
    openFacets.reserve(halfedge.size() / 4 + 8);
    for (MCGAL::Halfedge* h : halfedge) {
        replayBoundaryHalfedge(h, openFacets);
    }

    for (MCGAL::Facet* f : facets) {
        insertSegmentedPoint(f, openFacets.find(f) != openFacets.end());
    }
}

bool SegmentedDefaultReconstructOperator::replayBoundaryHalfedge(
    MCGAL::Halfedge* h,
    std::unordered_set<MCGAL::Facet*>& openFacets) {
    if (!h || h->isRemoved() || h->opposite() != nullptr || !h->face()) {
        return false;
    }
    MCGAL::Facet* f = h->face();
    if (f->isRemoved()) return false;

    MCGAL::Halfedge* hstart = h->next();
    MCGAL::Halfedge* hend = safePrevInFace(h);
    if (!hstart || !hstart->next() || !hend ||
        hstart == h || hend == h || hend->isRemoved()) {
        return false;
    }

    if (!f->isSplittable()) {
        f->setRemoved();
    }
    hend->setNext(nullptr);
    h->setRemoved();
    if (!f->isRemoved()) {
        f->reset_without_init(hstart);
        openFacets.insert(f);
    }
    return true;
}

MCGAL::Point SegmentedDefaultReconstructOperator::restorePoint(MCGAL::Facet* facet) const {
    if (!enablePrediction_ || !mesh_ || !facet) {
        return facet ? facet->getRemovedVertexPos() : MCGAL::Point(0.0f, 0.0f, 0.0f);
    }

    int64_t sx = 0;
    int64_t sy = 0;
    int64_t sz = 0;
    int n = 0;
    MCGAL::Halfedge* start = facet->proxyHalfedge();
    MCGAL::Halfedge* h = start;
    MCGAL::Vertex* lastEnd = nullptr;
    for (int guard = 0; h && guard < 100000; ++guard) {
        if (h->vertex()) {
            MCGAL::PointInt pi = mesh_->floatPosToInt(h->vertex()->point());
            sx += pi.x();
            sy += pi.y();
            sz += pi.z();
            ++n;
        }
        lastEnd = h->end_vertex();
        h = h->next();
        if (h == start) {
            lastEnd = nullptr;
            break;
        }
    }
    if (lastEnd) {
        MCGAL::PointInt pi = mesh_->floatPosToInt(lastEnd->point());
        sx += pi.x();
        sy += pi.y();
        sz += pi.z();
        ++n;
    }
    if (n == 0) {
        return MCGAL::Point(0.0f, 0.0f, 0.0f);
    }

    MCGAL::PointInt residual = facet->getRemovedVertexPosInt();
    MCGAL::PointInt q(static_cast<int>(sx / n) + residual.x(),
                      static_cast<int>(sy / n) + residual.y(),
                      static_cast<int>(sz / n) + residual.z());
    return MCGAL::rdIntToFloatPoint(q, mesh_->bboxMin, mesh_->f_quantStep);
}

void SegmentedDefaultReconstructOperator::insertSegmentedPoint(MCGAL::Facet* facet,
                                                               bool openBoundary) {
    if (!facet || facet->isRemoved()) return;
    if (!openBoundary) {
        BasicReconstructOperator::insert_point(facet);
        return;
    }
    if (!mesh_ || !facet->proxyHalfedge()) return;
    if (!facet->proxyHalfedge()->next()) return;
    mesh_->create_center_vertex_plus(facet->proxyHalfedge(), restorePoint(facet));
}
