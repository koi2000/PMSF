/*
 * @file PreQuantizePreprocessor.h
 * @brief Phase 5 重构：把 PPMC / 其他算法的"量化顶点坐标"抽象为一等公民 PreProcessOperator。
 *
 * 历史上，`mesh->quantizeVertexPositions()` 的调用分散在：
 *   - src/core/PPMCBindings.cpp 的 ppmcPreQuantize（CompressPipeline preInitPreparation hook）
 *   - impl/ 下若干论文的 main.cpp 里按需要手动调用
 *
 * Phase 5 引入 `pmsf::PreQuantizePreprocessor`，让 impl/ 层可以通过
 * `CompressPipeline::Builder::withPreprocessor(...)` 以统一的 Operator 形式插入这一步；
 * 这样"新论文复现时要不要做顶点量化"就是一行 Builder 配置，而非再在 PMSF 里加名为
 * xxxPreQuantize 的特殊入口。
 *
 * 与旧的 PreProcessOperator 兼容 —— 本类直接继承 `PreProcessOperator`。
 */
#pragma once

#include "PreProcessOperator.h"

namespace pmsf {

/// @brief 对 mesh 的顶点坐标进行整数量化（调用 MCGAL::Mesh::quantizeVertexPositions）。
///
/// 对于 PPMC：整个压缩阶段之前必须先调用此步，否则 SelectOperator 读到的顶点坐标
/// 是浮点值；解压阶段不需要反量化（量化是不可逆的，但符号流的残差是基于量化后坐标
/// 生成的，解压阶段直接读回即可）。
class PreQuantizePreprocessor : public PreProcessOperator {
  public:
    PreQuantizePreprocessor() = default;
    ~PreQuantizePreprocessor() = default;

    void process(std::shared_ptr<MCGAL::Mesh> mesh) override;
};

}  // namespace pmsf
