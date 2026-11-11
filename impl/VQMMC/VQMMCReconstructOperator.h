#pragma once

#include "reconstruct/BasicReconstructOperator.h"

class VQMMCReconstructOperator : public BasicReconstructOperator {
  public:
    VQMMCReconstructOperator() = default;
    ~VQMMCReconstructOperator() = default;

    void reconstruct(std::vector<MCGAL::Vertex*>& vertices,
                     std::vector<MCGAL::Halfedge*>& halfedge,
                     std::vector<MCGAL::Facet*>& facets) override;
};
