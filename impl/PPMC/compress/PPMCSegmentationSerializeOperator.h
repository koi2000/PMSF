#pragma once

#include "compress/SegmentationSerializeOperator.h"

class PPMCSegmentationSerializeOperator : public SegmentationSerializeOperator {
  public:
    using SegmentationSerializeOperator::SegmentationSerializeOperator;

    void serializeBaseMeshWithSeed(std::shared_ptr<MCGAL::Mesh> mesh,
                                   std::vector<MCGAL::Halfedge*> seeds);
};
