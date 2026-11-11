// SPDX-License-Identifier: Apache-2.0
//
// MMSECQuestSymbolReadOperator.h — B2 decoder reader for the `mmse_cq` codec.
//
// Splits the blob `[u32 trailerLen][MMT1 trailer][CQST v2]`, parses the
// per-layer 2-bit mode stream into memory (needed by the reconstructor before
// each `decodeLayer` call) and hands the CQST part to the verified conquest
// reader for the connectivity symbol streams + geometry residual deltas.

#ifndef PMSF_IMPL_MMSE_DECOMPRESS_MMSE_CONQUEST_SYMBOL_READ_OPERATOR_H
#define PMSF_IMPL_MMSE_DECOMPRESS_MMSE_CONQUEST_SYMBOL_READ_OPERATOR_H

#include "Conquest/decompress/ConquestSymbolReadOperator.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace pmsf {

class MMSECQuestSymbolReadOperator : public ConquestSymbolReadOperator {
  public:
    MMSECQuestSymbolReadOperator() = default;

    void importFromRangeCodedBuffer(char* buffer,
                                    size_t& dataOffset,
                                    unsigned nbQuantBits) override;

    void setPMSFSymbolContext(const PMSFSymbolTable* symbols,
                              const PMSFSymbolPayloadTable* payloads) override;
    void collectFromDecoded(MCGAL::Halfedge* seed,
                            std::vector<MCGAL::Vertex*>& vertices,
                            std::vector<MCGAL::Halfedge*>& halfedges,
                            std::vector<MCGAL::Facet*>& facets) override;

    /// @brief Per-layer replay data (decoder layer order == blob layer order).
    const std::vector<std::uint8_t>* modesForLayer(std::size_t idx) const {
        return (idx < layerModes_.size() && !layerModes_[idx].modes.empty())
                   ? &layerModes_[idx].modes
                   : nullptr;
    }
    int clusterSizeForLayer(std::size_t idx) const {
        return (idx < layerModes_.size()) ? layerModes_[idx].clusterSize : 32;
    }
    std::size_t trailerLayerCount() const {
        return layerModes_.size();
    }
    bool usesRecursiveTopology() const { return recursiveTopology_; }
    bool usesFineRingPrediction() const { return fineRingPrediction_; }
    int topologyThreshold() const { return topologyThreshold_; }
    std::uint64_t topologySeed() const { return topologySeed_; }


  private:
    struct LayerModes {
        std::vector<std::uint8_t> modes;
        int clusterSize = 32;
    };
    std::vector<LayerModes> layerModes_;
    bool recursiveTopology_ = false;
    bool fineRingPrediction_ = false;
    bool awaitingTypedMetadata_ = false;
    int topologyThreshold_ = 32;
    std::uint64_t topologySeed_ = 1;
};

}  // namespace pmsf

#endif  // PMSF_IMPL_MMSE_DECOMPRESS_MMSE_CONQUEST_SYMBOL_READ_OPERATOR_H
