// SPDX-License-Identifier: Apache-2.0
//
// VQMMCCodebookSymbolReadOperator.h — VQMMC codebook protocol decoder
// (PR-L5, BlobLayout=23).
//
// Inherits RDTagSymbolReadOperator. Accepts BlobLayout=23 in both supported
// physical forms:
//   - 3-stream RDTag-equivalent layout (valence + isPolyBit + residual).
//   - 4-stream VQ layout (topology + codebook indices + codebook).
//
// If the sidecar claims blob-level codebook/index streams but the physical blob
// is still 3-stream, the reader fails fast instead of silently decoding a
// mismatched artifact.

#ifndef PMSF_DECOMPRESS_VQMMC_CODEBOOK_SYMBOL_READ_OPERATOR_H
#define PMSF_DECOMPRESS_VQMMC_CODEBOOK_SYMBOL_READ_OPERATOR_H

#include "RD/decompress/RDTagSymbolReadOperator.h"
#include "VQMMC/VectorQuantizedConnectivityCodec.h"

class VQMMCCodebookSymbolReadOperator : public RDTagSymbolReadOperator {
  public:
    VQMMCCodebookSymbolReadOperator() = default;

    void importFromRangeCodedBuffer(char* buffer,
                                    size_t& dataOffset,
                                    unsigned nbQuantBits) override;

    void collectFromDecoded(MCGAL::Halfedge* seed,
                            std::vector<MCGAL::Vertex*>& vertices,
                            std::vector<MCGAL::Halfedge*>& halfedge,
                            std::vector<MCGAL::Facet*>& facets) override;

  private:
    bool useVectorQuantizedBlob_ = false;
    int currentVectorQuantizedRound_ = 0;
    pmsf::VectorQuantizedDecodedBlob decodedVectorQuantizedBlob_{};
};

#endif  // PMSF_DECOMPRESS_VQMMC_CODEBOOK_SYMBOL_READ_OPERATOR_H
