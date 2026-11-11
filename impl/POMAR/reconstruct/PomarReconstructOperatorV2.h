/*
 * @Author: pomar-refactor
 * @Description: Pomar 存储模块重构版 —— 重建操作符
 *
 * 在框架中 ReconstructOperator 负责"基于 symbol 还原 mesh"。
 * Pomar 原实现把 vertexSplit 的调用塞在 Deserializer 里，这里正式
 * 提取到 ReconstructOperator，使得职责界限与其他压缩方案一致：
 *   - DeserializeOperator 负责产生 PerformedOperation；
 *   - ReconstructOperatorV2 负责应用这些操作改动 mesh。
 */
#pragma once

#include "Halfedge.h"
#include "Mesh.h"
#include "PerformedOperation.h"
#include "reconstruct/ReconstructOpertator.h"

#include <queue>

class PomarReconstructOperatorV2 : public ReconstructOperator {
  public:
    PomarReconstructOperatorV2() = default;
    ~PomarReconstructOperatorV2() = default;

    /// 框架兼容的空接口
    void reconstruct(MCGAL::Halfedge* /*halfedge*/) override {}
    void reconstruct(std::vector<MCGAL::Vertex*>& /*vertices*/,
                     std::vector<MCGAL::Halfedge*>& /*halfedge*/,
                     std::vector<MCGAL::Facet*>& /*facets*/) override {}

    /// Pomar 专用：批量执行一个 LOD 中的所有 vertex split
    void applyOperations(std::queue<MCGAL::PerformedOperation>& opToPerform);

    /// 执行单个 vertex split
    void applyOne(MCGAL::PerformedOperation& op);
};
