// SPDX-License-Identifier: Apache-2.0
//
// PCATMReconstructOperator.h
//
// Decoder-side reconstruction for the PCATM paper path. The symbol reader
// identifies color-coded patch components and passes the patch root faces plus
// their internal diagonals here; this operator merges each patch back to the
// polygonal hole, then inserts the removed vertex with barycenter prediction.

#pragma once

#include "reconstruct/BasicReconstructOperator.h"

class PCATMReconstructOperator : public BasicReconstructOperator {
  public:
    PCATMReconstructOperator() = default;
    ~PCATMReconstructOperator() = default;

    void reconstruct(std::vector<MCGAL::Vertex*>& vertices,
                     std::vector<MCGAL::Halfedge*>& halfedges,
                     std::vector<MCGAL::Facet*>& facets) override;
};
