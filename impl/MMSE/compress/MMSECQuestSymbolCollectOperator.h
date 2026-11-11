// SPDX-License-Identifier: Apache-2.0
//
// MMSECQuestSymbolCollectOperator.h — B2 encoder for the `mmse_cq` codec:
// valence-conquest backbone (paper [2] traversal / connectivity) + MMSE paper
// §2.2 predictor stack + §2.3 per-cluster mode decision.
//
// Blob layout ("MMT1"):
//   [u32 trailerLen][trailer][CQST v2 bytes]      (CQST = conquest blob, layers
//                                                  written in reverse order)
//   trailer = 'M''M''T''1' | u32 version=1 | u32 numLayers
//             per layer: u32 gateCount | u32 clusterSize | u32 modeBytes | modes
// The CQST delta field of each layer carries the **MMSE-predicted residual**
// (target cell - chosen-mode prediction) instead of the plain mean residual;
// the mode ids needed to replay the prediction live in the trailer.
//
// Post-layer pass (why the predictor is not computed inside the hook):
// see src/predict/MMSEConquestHooks.h — the paper predictor is evaluated on the
// layer's coarse graph, which only exists after `decimateLayer` returns.

#ifndef PMSF_IMPL_MMSE_COMPRESS_MMSE_CONQUEST_SYMBOL_COLLECT_OPERATOR_H
#define PMSF_IMPL_MMSE_COMPRESS_MMSE_CONQUEST_SYMBOL_COLLECT_OPERATOR_H

#include "compress/SymbolCollectOperator.h"
#include "conquest/ConquestBlobFormat.h"
#include "conquest/FrontierConquestEngine.h"
#include "predict/MMSEConquestHooks.h"

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

namespace pmsf {

class MMSECQuestSymbolCollectOperator : public SymbolCollectOperator {
  public:
    MMSECQuestSymbolCollectOperator() = default;
    void setPaperEntropy(bool enabled, signed_bitplane::Context context = signed_bitplane::Context::Magnitude) {
        paperEntropy_ = enabled; bitplaneContext_ = context;
    }

    void setRecursiveTopology(bool enabled, int threshold = 32, std::uint64_t seed = 1) {
        recursiveTopology_ = enabled; topologyThreshold_ = threshold; topologySeed_ = seed;
    }

    void setPaperRelations(bool enabled) { paperRelations_ = enabled; }
    void setFineRingPrediction(bool enabled) { fineRingPrediction_ = enabled; }

    void init(std::shared_ptr<MCGAL::Mesh> mesh,
              std::vector<MCGAL::Halfedge*> seeds,
              PMSFHeader header) override;

    void collect(MCGAL::Halfedge* seed) override;
    void collect(std::vector<MCGAL::Halfedge*> seeds) override;
    bool supportsRangeCoderExport() const override {
        return true;
    }
    int exportToBuffer(char* buffer, bool enableQuantization = false,
                       bool enableCompress = false) override;
    int exportToBufferWithRangeCoder(char* buffer,
                                     bool enableQuantization = false) override;

    const pmsf::PMSFSymbolTable* pmsfSymbolTable() const override;

    void setQuantBits(unsigned bits) {
        quantBits_ = bits;
    }
    void setQuantGrid(const pmsf::FrontierConquestEngine::QuantGrid& g) {
        grid_ = g;
        gridForced_ = true;
    }
    const pmsf::FrontierConquestEngine::QuantGrid& quantGrid() const {
        return grid_;
    }
    unsigned quantBits() const {
        return quantBits_;
    }
    const conquest_blob::Stream& stream() const {
        return stream_;
    }
    /// @brief Number of modes decided so far (all layers), diagnostics.
    std::size_t totalModes() const;

  private:
    void computeGridFromMesh();
    void snapMeshToCells();

    conquest_blob::Stream stream_;
    std::shared_ptr<MCGAL::Mesh> mesh_;
    std::shared_ptr<mmse_pred::MMSECQuestGeomState> geom_;
    pmsf::FrontierConquestEngine::QuantGrid grid_;
    bool gridForced_ = false;
    unsigned quantBits_ = 12;
    bool paperEntropy_ = false;
    bool paperRelations_ = false;
    bool fineRingPrediction_ = false;
    bool recursiveTopology_ = false;
    int topologyThreshold_ = 32;
    std::uint64_t topologySeed_ = 1;
    signed_bitplane::Context bitplaneContext_ = signed_bitplane::Context::Magnitude;
    std::vector<std::vector<std::uint8_t>> layerModes_;
    mutable pmsf::PMSFSymbolTable paperTable_;
};

}  // namespace pmsf

#endif  // PMSF_IMPL_MMSE_COMPRESS_MMSE_CONQUEST_SYMBOL_COLLECT_OPERATOR_H
