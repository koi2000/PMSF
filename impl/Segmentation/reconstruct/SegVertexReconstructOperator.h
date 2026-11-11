/*
 * @Author: koi
 * @Date: 2025-09-02 15:46:27
 * @Description:
 */
#pragma once

#include "Halfedge.h"
#include "Mesh.h"
#include "reconstruct/ReconstructOpertator.h"

class SegVertexReconstructOperator : public ReconstructOperator {
  public:
    SegVertexReconstructOperator() = default;

    ~SegVertexReconstructOperator() = default;

    void reconstruct(MCGAL::Halfedge* halfedge) override;

    void reconstruct(std::vector<MCGAL::Vertex*>& vertices, std::vector<MCGAL::Halfedge*>& halfedge, std::vector<MCGAL::Facet*>& facets) override;

    void merge_face(MCGAL::Halfedge* halfedge);

    void insert_point(MCGAL::Facet* facet);
};
