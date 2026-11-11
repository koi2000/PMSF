#ifndef SEGMENTED_DEFAULT_RECONSTRUCT_OPERATOR_H
#define SEGMENTED_DEFAULT_RECONSTRUCT_OPERATOR_H

#include "reconstruct/BasicReconstructOperator.h"
#include <unordered_set>

class SegmentedDefaultReconstructOperator : public BasicReconstructOperator {
  public:
    void reconstruct(MCGAL::Halfedge* halfedge) override;

    void reconstruct(std::vector<MCGAL::Vertex*>& vertices,
                     std::vector<MCGAL::Halfedge*>& halfedge,
                     std::vector<MCGAL::Facet*>& facets) override;

  private:
    bool replayBoundaryHalfedge(MCGAL::Halfedge* halfedge,
                                std::unordered_set<MCGAL::Facet*>& openFacets);
    void insertSegmentedPoint(MCGAL::Facet* facet, bool openBoundary);
    MCGAL::Point restorePoint(MCGAL::Facet* facet) const;
};

#endif
