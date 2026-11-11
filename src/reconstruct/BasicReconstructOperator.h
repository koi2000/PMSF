/*
 * @Author: koi
 * @Date: 2025-09-02 15:46:27
 * @Description:
 */
#ifndef BASIC_Reconstruct_OPERATOR_H
#define BASIC_Reconstruct_OPERATOR_H

#include "Halfedge.h"
#include "Mesh.h"
#include "ReconstructOpertator.h"

class BasicReconstructOperator : public ReconstructOperator {
  public:
    BasicReconstructOperator() = default;

    ~BasicReconstructOperator() = default;

    void reconstruct(MCGAL::Halfedge* halfedge) override;

    void reconstruct(std::vector<MCGAL::Vertex*>& vertices, std::vector<MCGAL::Halfedge*>& halfedge, std::vector<MCGAL::Facet*>& facets) override;

    void merge_face(MCGAL::Halfedge* halfedge);

    void insert_point(MCGAL::Facet* facet);
};

#endif
