/*****************************************************************************
 * @Author: koi
 * @Date: 2025-08-26 22:18:50
 * @Description: PCATM (Progressive Compression of Arbitrary Triangular Meshes)
 *               消除操作实现。基于 VertexRemovalEliminateOperator，仅 override：
 *                 - 整数残差计算（barycentric 而非 rdBarycenterQuantized）
 *                 - 三角化策略（五度/六度 Z-shape + RED/BLUE 2-coloring）
 *               参考论文: Cohen-Or, Levin, Remez - "Progressive Compression of
 *               Arbitrary Triangular Meshes" (1999)
 *****************************************************************************/
#pragma once

#include "elimination/VertexRemovalEliminateOperator.h"
#include "Halfedge.h"
#include <vector>

class PCATMEliminateOperator : public VertexRemovalEliminateOperator {
  public:
    PCATMEliminateOperator() = default;
    ~PCATMEliminateOperator() = default;

    bool triangulate(MCGAL::Halfedge* h) override;

  protected:
    int computeRemovedValence(MCGAL::Halfedge* h,
                              MCGAL::Vertex* removedVertex) override;

    MCGAL::Vector3i computeRemovedVertexResidualInt(
        MCGAL::Halfedge* h,
        MCGAL::Halfedge* hface,
        const MCGAL::Point& vrPoint,
        const MCGAL::PointInt& vrPos) override;

  private:
    bool triangulate_degree_5(MCGAL::Facet* polyFace,
                              const std::vector<MCGAL::Vertex*>& polyVerts,
                              const std::vector<MCGAL::Halfedge*>& polyHalfedges,
                              int groupId);
    bool triangulate_degree_6(MCGAL::Facet* polyFace,
                              const std::vector<MCGAL::Vertex*>& polyVerts,
                              const std::vector<MCGAL::Halfedge*>& polyHalfedges,
                              int groupId);
    bool triangulate_general(MCGAL::Facet* polyFace,
                             const std::vector<MCGAL::Vertex*>& polyVerts,
                             const std::vector<MCGAL::Halfedge*>& polyHalfedges,
                             int groupId);
};
