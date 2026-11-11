/*
 * @file PPMCBindings.h
 * @brief PPMC 预测开关在 SymbolCollectOperator / SymbolReadOperator 之间的
 *        双向下传。Phase 1 重构从 src/PMSF.cpp 里提取 —— 它是唯一一处真正的
 *        算法特异性胶水（QEM / MMSE / APTM 等其他算法没有类似 downcast）。
 */
#ifndef PMSF_CORE_PPMC_BINDINGS_H
#define PMSF_CORE_PPMC_BINDINGS_H

#include <memory>

class SymbolCollectOperator;
class SymbolReadOperator;
class PMSFHeader;
class CompressOptions;

namespace MCGAL {
class Mesh;
}

namespace pmsf {

/// @brief 压缩端：若 collector 是 PPMCSymbolCollectOperator，把预测开关同步下去。
///        在 collector->init 之后、主循环之前调用（通过 CompressPipelineConfig::beforeCompressLoop）。
void bindPPMCPredictionOnCompress(
    std::shared_ptr<SymbolCollectOperator> collector,
    const CompressOptions& options);

/// @brief 解压端：若 symbolReader 是 PPMCSymbolReadOperator，把 header 里读出来的
///        预测开关同步过去。在 deserializeBaseMesh 之后、首轮 deserializeOneRound
///        之前调用（通过 runDecompress 的 afterBaseMeshHook）。
void bindPPMCPredictionOnDecompress(
    std::shared_ptr<SymbolReadOperator> symbolReader,
    const PMSFHeader& header,
    std::shared_ptr<MCGAL::Mesh> baseMesh);

/// @brief PPMC 的 quantizeVertexPositions 预处理：在 SelectOperator::init 之前调用。
///        用作 CompressPipelineConfig::preInitPreparation。
void ppmcPreQuantize(std::shared_ptr<MCGAL::Mesh> mesh,
                     PMSFHeader& header,
                     const CompressOptions& options);

}  // namespace pmsf

#endif
