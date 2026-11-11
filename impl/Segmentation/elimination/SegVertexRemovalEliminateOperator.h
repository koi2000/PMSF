/*
 * @Author: koi
 * @Date: 2025-08-26 22:18:50
 * @Description:
 */

#pragma once

#include "elimination/EliminateOperator.h"
#include "Halfedge.h"
#include "common/HalfedgeErrorEntry.h"

class SegVertexRemovalEliminateOperator : public EliminateOperator {
  public:
    SegVertexRemovalEliminateOperator() = default;
    ~SegVertexRemovalEliminateOperator() = default;

    void init(std::shared_ptr<MCGAL::Mesh> mesh, PMSFHeader header, bool compress_boundary = false) override;

    bool eliminate(HalfedgeErrorEntry& entry) override;
    bool remove_point(MCGAL::Halfedge* h) override;
    bool triangulate(MCGAL::Halfedge* h) override;
    bool triangulate_plus(MCGAL::Halfedge* h);
    bool postprocess(MCGAL::Halfedge* h) override;
    MCGAL::Halfedge* encode_boundary(MCGAL::Halfedge* h) override;
    int groupId = -1;
};