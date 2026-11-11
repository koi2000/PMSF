/*
 * @file MMSEPipeline.h
 * @brief Phase-13+ Task-3: MMSE SelectOptions/CompressOptions factory.
 *
 * MMSE 的论文特化都是 IsRemovable 链 + pre/post processor 的拼装；这些
 * 对所有 MMSE 运行都一样，没有逐次可变的参数。把它们搬进本头文件让 main
 * 只剩 "parse args → call factory → run"。
 *
 * 输出逐字节与 Phase 8 baseline 一致。
 */
#ifndef PMSF_IMPL_MMSE_MMSE_PIPELINE_H
#define PMSF_IMPL_MMSE_MMSE_PIPELINE_H

#include "DegreeSixIsRemovableOperator.h"
#include "DegreeThreeIsRemovableOperator.h"
#include "MMSEPostProcessor.h"
#include "MMSEPreprocessor.h"
#include "elimination/VertexRemovalEliminateOperator.h"
#include "operator/BasicIsRemovableOperator.h"
#include "operator/UnifiedManifoldIsRemovableOperator.h"
#include "options/CompressOptions.h"
#include "options/SelectOptions.h"
#include "selection/ErrorSource.h"
#include "selection/PriorityErrorSelector.h"

#include <memory>
#include <string>

namespace mmse_impl {

inline std::shared_ptr<SelectOptions> buildSelectOptions() {
    return SelectOptions::Builder()
        .withErrorSource(ErrorSource::Halfedge)
        .withIsRemovableOperator(std::make_shared<BasicIsRemovableOperator>())
        .withIsRemovableOperator(std::make_shared<UnifiedManifoldIsRemovableOperator>())
        .withIsRemovableOperator(std::make_shared<DegreeThreeIsRemovableOperator>())
        .withIsRemovableOperator(std::make_shared<DegreeSixIsRemovableOperator>())
        .build();
}

inline CompressOptions buildCompressOptions(
    const std::string& input, const std::string& output, int rounds, double alpha) {
    auto sopt = buildSelectOptions();
    return CompressOptions::Builder()
        .setSelect(std::make_shared<PriorityErrorSelector>(*sopt))
        .setEliminate(std::make_shared<VertexRemovalEliminateOperator>())
        .setRound(rounds)
        .setPreProcessOperator(std::make_shared<MMSEPreprocessor>())
        .setPostProcessOperator(std::make_shared<MMSEPostProcessor>(alpha))
        .setEnableCompress(true)
        .setEnablePrediction(true)
        .setEnableQuantization(true)
        .setPath(input).setOutputPath(output).build();
}

}  // namespace mmse_impl

#endif
