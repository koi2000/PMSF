// SPDX-License-Identifier: Apache-2.0
//
// MMSECQuestReconstructOperator.h — B2 reconstructor for the `mmse_cq` codec.
//
// Same orchestration as the verified conquest reconstructor (one layer per
// framework round; seed recovered by coordinate; the single decode entity is
// `pmsf::decodeLayer`), plus one extra step before each layer: install that
// layer's MMSE mode stream into the geometry hook so `decodeLayer`'s H3
// predictor returns the paper §2.2 prediction selected by the encoder.

#ifndef PMSF_IMPL_MMSE_RECONSTRUCT_MMSE_CONQUEST_RECONSTRUCT_OPERATOR_H
#define PMSF_IMPL_MMSE_RECONSTRUCT_MMSE_CONQUEST_RECONSTRUCT_OPERATOR_H

#include "conquest/ConquestDecodeLayer.h"
#include "conquest/FrontierConquestEngine.h"
#include "predict/MMSEConquestHooks.h"
#include "reconstruct/ReconstructOpertator.h"

#include <memory>
#include <vector>

namespace pmsf {

class MMSECQuestSymbolReadOperator;  // impl/MMSE/decompress/

class MMSECQuestReconstructOperator : public ReconstructOperator {
  public:
    MMSECQuestReconstructOperator(std::shared_ptr<MMSECQuestSymbolReadOperator> reader,
                                  std::shared_ptr<mmse_pred::MMSECQuestGeomState> state);

    void reconstruct(MCGAL::Halfedge* halfedge) override;
    void reconstruct(std::vector<MCGAL::Vertex*>& vertices,
                     std::vector<MCGAL::Halfedge*>& halfedge,
                     std::vector<MCGAL::Facet*>& facets) override;

    const ConquestDecodeResult& lastResult() const {
        return last_;
    }
    bool failed() const {
        return failed_;
    }

  private:
    std::shared_ptr<MMSECQuestSymbolReadOperator> reader_;
    std::shared_ptr<mmse_pred::MMSECQuestGeomState> state_;
    EngineHooks hooks_;
    ConquestState st_;
    ConquestDecodeResult last_;
    int totalInserted_ = 0;
    bool failed_ = false;
    bool loggedOnce_ = false;
    bool decoderMeshBound_ = false;
};

}  // namespace pmsf

#endif  // PMSF_IMPL_MMSE_RECONSTRUCT_MMSE_CONQUEST_RECONSTRUCT_OPERATOR_H
