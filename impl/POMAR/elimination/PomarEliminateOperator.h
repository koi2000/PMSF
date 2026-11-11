/*
 * @Author: koi
 * @Date: 2025-08-26 22:18:50
 * @Description:
 */

#pragma once

#include "elimination/EliminateOperator.h"
#include "Halfedge.h"
#include "common/HalfedgeErrorEntry.h"

class PomarEliminateOperator : public EliminateOperator {
  public:
    PomarEliminateOperator() = default;
    ~PomarEliminateOperator() = default;

    void init(std::shared_ptr<MCGAL::Mesh> mesh,PMSFHeader header, bool compress_boundary = false) override;

    bool eliminate(HalfedgeErrorEntry& entry) override;
    bool remove_point(MCGAL::Halfedge* h) override;
    bool triangulate(MCGAL::Halfedge* h) override;
    bool postprocess(MCGAL::Halfedge* h) override;
    MCGAL::Halfedge* encode_boundary(MCGAL::Halfedge* h) override;
    void performCollapse(HalfedgeErrorEntry& entry);
    bool usesEdgeCollapseMode() const override { return true; }
    int groupId = -1;
};