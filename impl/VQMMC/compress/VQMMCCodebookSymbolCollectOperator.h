// SPDX-License-Identifier: Apache-2.0
//
// VQMMCCodebookSymbolCollectOperator.h — VQMMC codebook protocol
// (PR-L5, BlobLayout=23).
//
// Target protocol: 4 段流 ConnectivityBlobContainer v3, magic='CONN':
//   stream 0 (valence):   5-元 RC + 4-context + V_OTHER inline escape
//   stream 1 (isPolyBit): 2-元 RC, per BFS face 1 bit
//   stream 2 (codebook):  [int32 K][K × 3 × int32 quantized centroids]
//   stream 3 (indices):   per polyFace 3 uint16_t VQ codebook indices
//
// Inherits RDTagSymbolCollectOperator for valence/isPolyBit BFS.
// Adds unified paper payload rows for VQ codebook + index streams from
// VQMMCPostProcessor, plus compact sidecar metadata that records the current
// physical blob capabilities.
//
// NOTE: Full core writer integration is still deferred. The current physical
// blob emitted by this collector is the same 3-stream layout as RDTag(18).
// Non-empty codebook/index queues are exported through the unified paper
// payload table under the same names and binary format used by impl/VQMMC:
//   payload "vqmmc_codebook" and payload "vqmmc_indices".
// The sidecar records payload presence, format, byte order, and stream counts
// so downstream analysis does not mistake the 3-stream blob for a complete
// paper VQMMC stream. The matching core reader can already parse future
// BlobLayout=23 / 4-stream codebook+index blobs; the default writer is kept
// byte-compatible until BPV/time regression is measured.

#ifndef PMSF_COMPRESS_VQMMC_CODEBOOK_SYMBOL_COLLECT_OPERATOR_H
#define PMSF_COMPRESS_VQMMC_CODEBOOK_SYMBOL_COLLECT_OPERATOR_H

#include "RD/compress/RDTagSymbolCollectOperator.h"
#include <deque>
#include <utility>
#include <vector>
#include "Point.h"

class VQMMCCodebookSymbolCollectOperator : public RDTagSymbolCollectOperator {
  public:
    VQMMCCodebookSymbolCollectOperator() = default;

    int exportToBufferWithRangeCoder(char* buffer,
                                     bool enableQuantization = false) override;

    int exportToBuffer(char* buffer, bool enableQuantization = false,
                       bool enableCompress = false) override;

    // VQ-specific accessors (populated by VQMMCPostProcessor integration)
    const std::vector<std::deque<uint16_t>>& vqIndexQueues() const { return vqIndexQueues_; }
    const std::vector<std::vector<MCGAL::Point>>& codebookQueues() const { return codebookQueues_; }
    void setVQStreams(std::vector<std::deque<uint16_t>> indices,
                      std::vector<std::vector<MCGAL::Point>> codebooks) {
        vqIndexQueues_ = std::move(indices);
        codebookQueues_ = std::move(codebooks);
        paperTableCache_.clear();
        payloadTableCache_.clear();
        payloadTableCacheValid_ = false;
    }
    const pmsf::PMSFSymbolTable* pmsfSymbolTable() const override;
    const pmsf::PMSFSymbolPayloadTable* paperSymbolPayloadTable() const override;

  protected:
    std::vector<std::deque<uint16_t>> vqIndexQueues_;
    std::vector<std::vector<MCGAL::Point>> codebookQueues_;
    mutable pmsf::PMSFSymbolTable paperTableCache_{};
    mutable pmsf::PMSFSymbolPayloadTable payloadTableCache_{};
    mutable bool payloadTableCacheValid_ = false;
};

#endif  // PMSF_COMPRESS_VQMMC_CODEBOOK_SYMBOL_COLLECT_OPERATOR_H
