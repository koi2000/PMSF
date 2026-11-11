#pragma once

#include "Halfedge.h"
#include "Mesh.h"
#include "reconstruct/ReconstructOpertator.h"

class PPMCReconstructOperator : public ReconstructOperator {
  public:
    PPMCReconstructOperator() = default;

    ~PPMCReconstructOperator() = default;

    void reconstruct(MCGAL::Halfedge* halfedge) override;

    void reconstruct(std::vector<MCGAL::Vertex*>& vertices, std::vector<MCGAL::Halfedge*>& halfedge, std::vector<MCGAL::Facet*>& facets) override;

    void merge_face(MCGAL::Halfedge* halfedge);

    void insert_point(MCGAL::Facet* facet);
};