// SPDX-License-Identifier: Apache-2.0
//
// PCATMStrictPMSFSymbolReadOperator.h
//
// Decoder-side parser for PCATM §4 strict storage (BlobLayout=25).
// This reader validates and decodes the paper color/residual streams:
//   stream 0: per-level BFS triangle color vector
//   stream 1: barycenter displacement residuals
//
// sv=4+ supports color-only patch recovery with no valence/isPoly topology side
// streams. sv=1/2 intentionally fail-fast at recovery, and sv=3 remains the
// transitional mixed 2/4-color format with temporary topology side streams.
// The remaining paper gap is exact Z-shape semantics and BPV target closure,
// not basic color-stream parsing.

#ifndef PMSF_DECOMPRESS_PCATM_STRICT_PAPER_SYMBOL_READ_OPERATOR_H
#define PMSF_DECOMPRESS_PCATM_STRICT_PAPER_SYMBOL_READ_OPERATOR_H

#include "PCATM/decompress/PCATMZColorSymbolReadOperator.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

class PCATMStrictPMSFSymbolReadOperator : public PCATMZColorSymbolReadOperator {
  public:
    PCATMStrictPMSFSymbolReadOperator() = default;

    void importFromRangeCodedBuffer(char* buffer,
                                    size_t& dataOffset,
                                    unsigned nbQuantBits) override;

    void collectFromDecoded(MCGAL::Halfedge* seed,
                            std::vector<MCGAL::Vertex*>& vertices,
                            std::vector<MCGAL::Halfedge*>& halfedge,
                            std::vector<MCGAL::Facet*>& facets) override;

    int totalColorBits() const { return totalColorBits_; }
    int totalResiduals() const { return totalResiduals_; }
    int decodedRounds() const { return decodedRounds_; }
    void setMesh(std::shared_ptr<MCGAL::Mesh> mesh) { mesh_ = mesh; }

  private:
    int totalColorBits_ = 0;
    int totalResiduals_ = 0;
    int decodedRounds_ = 0;
    bool orientedZRecovery_ = false;
    std::vector<int> colorModePerRound_;
    std::vector<std::vector<uint8_t>> colorSymbolsPerRound_;
    std::vector<int> residualsPerRound_;
    // WS-B task-18: per-round mode-2 side info (2-bit candidate zero-pair index
    // per patch), parsed from the tail of the per-round color stream when
    // PMSF_PCATM_STRICT_2COLOR_SIDE=1.
    std::vector<std::vector<uint8_t>> mode2SidePerRound_;
    std::shared_ptr<MCGAL::Mesh> mesh_;
};

#endif  // PMSF_DECOMPRESS_PCATM_STRICT_PAPER_SYMBOL_READ_OPERATOR_H
