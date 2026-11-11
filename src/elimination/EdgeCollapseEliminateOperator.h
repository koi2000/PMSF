/*
 * @Author: koi
 * @Date: 2025-08-26 22:18:42
 * @Description: Edge-collapse 操作单元（v2 合并到 v1，移除两侧塌缩面）。
 *
 * ⚠️ 与 VertexRemovalEliminateOperator 的区别：
 *   - VertexRemoval: 删一邻域中心点 + retriangulate（影响 deg 个面）
 *   - EdgeCollapse:  半边塌缩，只影响 2 个面
 * QEM 论文 (Garland-Heckbert 1997) 的标准操作就是 edge-contraction。
 *
 * ⚠️ 当前限制：解码侧 (BasicReconstructOperator) 是为 vertex-removal 的逆操作
 * (insert_center_vertex) 设计的，使用此 EliminateOperator 时需要配套实现 vertex-split
 * 的解码路径（待后续工作）。目前可用于"只编码侧基准比较"或独立简化工具。
 */
#pragma once

#include "EliminateOperator.h"
#include "Mesh.h"
#include "common/HalfedgeErrorEntry.h"
#include <memory>

class EdgeCollapseEliminateOperator : public EliminateOperator {
  public:
    EdgeCollapseEliminateOperator() = default;
    ~EdgeCollapseEliminateOperator() = default;

    bool eliminate(HalfedgeErrorEntry& entry) override;
    bool remove_point(MCGAL::Halfedge* h) override;
    bool triangulate(MCGAL::Halfedge* h) override;
    bool postprocess(MCGAL::Halfedge* h) override;
    MCGAL::Halfedge* encode_boundary(MCGAL::Halfedge* h) override;
};