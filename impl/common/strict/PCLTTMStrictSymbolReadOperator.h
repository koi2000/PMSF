// SPDX-License-Identifier: Apache-2.0
//
// PCLTTMStrictSymbolReadOperator.h

#ifndef PMSF_DECOMPRESS_PCLTTM_STRICT_SYMBOL_READ_OPERATOR_H
#define PMSF_DECOMPRESS_PCLTTM_STRICT_SYMBOL_READ_OPERATOR_H

#include "decompress/SymbolReadOperator.h"
#include "common/patch/RDPatchPipeline.h"

#include <array>
#include <memory>
#include <vector>

namespace pmsf {

class PCLTTMStrictSymbolReadOperator : public SymbolReadOperator {
  public:
    PCLTTMStrictSymbolReadOperator() = default;

    void importFromRangeCodedBuffer(char* buffer,
                                    size_t& dataOffset,
                                    unsigned nbQuantBits) override;

    void collectFromDecoded(MCGAL::Halfedge* seed,
                            std::vector<MCGAL::Vertex*>& vertices,
                            std::vector<MCGAL::Halfedge*>& halfedge,
                            std::vector<MCGAL::Facet*>& facets) override;

    void collect(MCGAL::Halfedge* seed, char* buffer, size_t& dataOffset,
                 std::vector<MCGAL::Vertex*>& vertices,
                 std::vector<MCGAL::Halfedge*>& halfedge,
                 std::vector<MCGAL::Facet*>& facets) override;
    void collect(std::vector<MCGAL::Halfedge*> seeds, char* buffer, size_t& dataOffset,
                 std::vector<MCGAL::Vertex*>& vertices,
                 std::vector<MCGAL::Halfedge*>& halfedge,
                 std::vector<MCGAL::Facet*>& facets) override;

    void setMesh(std::shared_ptr<MCGAL::Mesh> mesh) { mesh_ = std::move(mesh); }

  private:
    std::vector<RDPatchSymbol> decodedSymbols_;
    std::vector<int> decodedResidualsX_;
    std::vector<int> decodedResidualsY_;
    std::vector<int> decodedResidualsZ_;
    std::vector<int> decodedFaceRanks_;
    std::vector<uint8_t> decodedInputOffsets_;
    std::vector<std::vector<std::array<int, 3>>> decodedRingQs_;
    int decodedRingStartIndex_ = 0;
    std::vector<std::vector<int>> decodedReconRingIds_;
    std::vector<int> decodedReconOriginalIds_;
    std::vector<std::array<int, 6>> decodedSymbolInputQs_;
    std::vector<std::array<int, 6>> decodedVInputQs_;
    std::vector<std::array<int, 6>> layerSeedQs_;
    bool packedNullInferenceAllowed_ = false;
    std::shared_ptr<MCGAL::Mesh> mesh_;
};

}  // namespace pmsf

#endif  // PMSF_DECOMPRESS_PCLTTM_STRICT_SYMBOL_READ_OPERATOR_H
