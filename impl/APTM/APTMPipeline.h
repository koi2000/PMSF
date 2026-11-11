/*
 * @file APTMPipeline.h
 * @brief Phase-13+ Task-3: APTM SelectOptions/CompressOptions factory.
 *
 * APTM's Builder chain is uniform across runs — 4 IsRemovable operators +
 * AnchorPreprocessor + fixed compression flags. Extract so the main only
 * handles CLI + run.
 */
#ifndef PMSF_IMPL_APTM_APTM_PIPELINE_H
#define PMSF_IMPL_APTM_APTM_PIPELINE_H

#include "AnchorPreprocessor.h"
#include "DegreeSixIsRemovableOperator.h"
#include "DegreeThreeIsRemovableOperator.h"
#include "elimination/VertexRemovalEliminateOperator.h"
#include "operator/BasicIsRemovableOperator.h"
#include "operator/UnifiedManifoldIsRemovableOperator.h"
#include "options/CompressOptions.h"
#include "options/SelectOptions.h"
#include "selection/ErrorSource.h"
#include "selection/PriorityErrorSelector.h"

#include <memory>
#include <string>

namespace aptm_impl {

inline std::shared_ptr<SelectOptions> buildSelectOptions() {
    return SelectOptions::Builder()
        .withErrorSource(ErrorSource::Halfedge)
        .withIsRemovableOperator(std::make_shared<BasicIsRemovableOperator>())
        .withIsRemovableOperator(std::make_shared<UnifiedManifoldIsRemovableOperator>())
        .withIsRemovableOperator(std::make_shared<DegreeThreeIsRemovableOperator>())
        .withIsRemovableOperator(std::make_shared<DegreeSixIsRemovableOperator>())
        .build();
}

inline CompressOptions buildCompressOptions(const std::string& input, const std::string& output, int rounds) {
    auto sopt = buildSelectOptions();
    return CompressOptions::Builder()
        .setSelect(std::make_shared<PriorityErrorSelector>(*sopt))
        .setEliminate(std::make_shared<VertexRemovalEliminateOperator>())
        .setRound(rounds)
        .setPreProcessOperator(std::make_shared<AnchorPreprocessor>())
        .setEnableCompress(true).setEnablePrediction(true).setEnableQuantization(true)
        .setPath(input).setOutputPath(output).build();
}

}  // namespace aptm_impl

#endif
