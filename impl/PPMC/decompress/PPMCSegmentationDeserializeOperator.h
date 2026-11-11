#pragma once

#include "decompress/SegmentationDeserializeOperator.h"

class PPMCSegmentationDeserializeOperator : public SegmentationDeserializeOperator {
  public:
    using SegmentationDeserializeOperator::SegmentationDeserializeOperator;

    std::shared_ptr<MCGAL::Mesh> deserializeBaseMesh() override;
};
