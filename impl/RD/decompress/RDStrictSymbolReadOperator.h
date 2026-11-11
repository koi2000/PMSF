// SPDX-License-Identifier: Apache-2.0
//
// Decoder-side reader for RD_STRICT(29).  This intentionally mirrors only the
// storage contract landed by RDStrictSymbolCollectOperator: connectivity symbols
// are decoded with the same range-coder model, side-info is parsed, and geometry
// remains a validated placeholder until the paper RD inverse transform lands.

#ifndef PMSF_DECOMPRESS_RD_STRICT_SYMBOL_READ_OPERATOR_H
#define PMSF_DECOMPRESS_RD_STRICT_SYMBOL_READ_OPERATOR_H

#include "decompress/SymbolReadOperator.h"
#include "common/patch/RDPatchDecoderConquest.h"
#include "common/patch/RDPatchPipeline.h"

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace pmsf {

struct RDStrictSideInfo {
    std::int32_t numVerticesIn = 0;
    std::int32_t numVerticesOut = 0;
    std::int32_t numDecimatedTotal = 0;
    std::int32_t numNullPatches = 0;
    std::int32_t numDecCycles = 0;
    std::int32_t firstGateV0Q[3] = {0, 0, 0};
    std::int32_t firstGateV1Q[3] = {0, 0, 0};
    std::vector<std::int32_t> layerSymbolCounts;
    std::vector<std::int32_t> layerVSymbolCounts;
    std::vector<std::uint8_t> layerBitplaneCounts;
    std::vector<std::uint8_t> layerLeakyFactors;
    std::vector<std::uint8_t> layerKltFlags;

    // v2 side channel: residual-facet alignment / strict replay.
    std::vector<int> absQX, absQY, absQZ;
    std::vector<std::array<int, 9>> faceQ;
    std::vector<unsigned char> faceSide;
    std::vector<unsigned char> ringSizes;
    std::vector<int> ringQX, ringQY, ringQZ;
    std::vector<int> faceRanks;
    std::vector<unsigned char> faceRingSizes;
    std::vector<int> faceRingQX, faceRingQY, faceRingQZ;
    std::vector<std::array<int, 6>> symbolInputQs;
    std::vector<std::array<int, 6>> layerSeedQs;
    std::vector<unsigned char> inputOffsets;
};

struct RDStrictBitplaneRef {
    std::uint16_t round = 0;
    std::uint16_t layer = 0;
    std::uint8_t component = 0;
    std::uint8_t bitplane = 0;  // 0 = MSB
};

struct RDStrictPartitionInfo {
    std::uint16_t layer = 0;
    std::uint16_t partition = 0;
    std::int32_t size = 0;
    std::uint8_t useKltFlags = 0;
    std::vector<std::uint8_t> kltBasis;
};

class RDStrictSymbolReadOperator final : public SymbolReadOperator {
  public:
    RDStrictSymbolReadOperator() = default;

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

    const std::vector<RDPatchSymbol>& decodedSymbols() const {
        return decodedSymbols_;
    }
    const RDStrictSideInfo& sideInfo() const { return sideInfo_; }
    const std::vector<RDStrictBitplaneRef>& bitplaneRefs() const {
        return bitplaneRefs_;
    }
    const std::vector<RDStrictPartitionInfo>& partitionInfos() const {
        return partitionInfos_;
    }
    std::int32_t decodedResidualCount() const { return decodedResidualCount_; }
    const std::vector<int>& decodedResidualsX() const { return decodedResidualsX_; }
    const std::vector<int>& decodedResidualsY() const { return decodedResidualsY_; }
    const std::vector<int>& decodedResidualsZ() const { return decodedResidualsZ_; }
    int decodedBitplaneRefs() const { return decodedBitplaneRefs_; }
    const RDPatchDecoderResult& lastDecoderResult() const {
        return lastDecoderResult_;
    }

    void setMesh(std::shared_ptr<MCGAL::Mesh> mesh) { mesh_ = std::move(mesh); }
    std::shared_ptr<MCGAL::Mesh> mesh() const { return mesh_; }

  private:
    struct RoundState {
        std::vector<RDPatchSymbol> symbols;
        RDStrictSideInfo sideInfo{};
        std::int32_t residualCount = 0;
        std::vector<int> residualsX;
        std::vector<int> residualsY;
        std::vector<int> residualsZ;
        std::vector<RDStrictBitplaneRef> bitplaneRefs;
        std::vector<RDStrictPartitionInfo> partitions;
        int decodedBitplaneRefs = 0;
    };

    std::vector<RDPatchSymbol> decodedSymbols_;
    RDStrictSideInfo sideInfo_{};
    std::int32_t decodedResidualCount_ = 0;
    std::vector<int> decodedResidualsX_;
    std::vector<int> decodedResidualsY_;
    std::vector<int> decodedResidualsZ_;
    std::vector<RDStrictBitplaneRef> bitplaneRefs_;
    std::vector<RDStrictPartitionInfo> partitionInfos_;
    int decodedBitplaneRefs_ = 0;
    std::vector<RoundState> rounds_;
    RDPatchDecoderResult lastDecoderResult_{};
    std::shared_ptr<MCGAL::Mesh> mesh_;
};

}  // namespace pmsf

#endif  // PMSF_DECOMPRESS_RD_STRICT_SYMBOL_READ_OPERATOR_H
