/*
 * @Author: koi
 * @Date: 2025-09-25 15:45:34
 * @Description: 
 */

#pragma once

#include "EliminateOperator.h"
#include "Halfedge.h"

class BoundaryVertexRemovalEliminateOperator : public EliminateOperator {
  public:
  BoundaryVertexRemovalEliminateOperator() = default;
    ~BoundaryVertexRemovalEliminateOperator() = default;

    void init(std::shared_ptr<MCGAL::Mesh> mesh, PMSFHeader header,bool compress_boundary = false) override;

    bool eliminate(HalfedgeErrorEntry& entry) override;
    bool remove_point(MCGAL::Halfedge* h) override;
    bool triangulate(MCGAL::Halfedge* h) override;
    bool postprocess(MCGAL::Halfedge* h) override;
    MCGAL::Halfedge* encode_boundary(MCGAL::Halfedge* h) override;
    int groupId = -1;
};