// SPDX-License-Identifier: Apache-2.0
//
// MMSEDualRingSymbolCollectOperator.h — MMSE dual-ring prediction protocol
// (PR-L4b, BlobLayout=22).
//
// Protocol sv=2: 4 段流 ConnectivityBlobContainer v3, magic='CONN':
//   stream 0 (valence):   5-元 RC + 4-context + V_OTHER inline escape
//   stream 1 (isPolyBit): 2-元 RC, per BFS face 1 bit
//   stream 2 (mode):      4-元 RC, per polyFace 1 mode
//                         0=alpha0, 1=alpha0.5, 2=alpha1, 3=MMSE
//   stream 3 (residual):  bit-plane RC, per polyFace 3 coord, predicted by
//                         the selected mode in quantized domain.
//
// sv=1 legacy files are decoder-compatible.
// sv=3 is RDTag-compatible topology with adaptive residual coding.
// sv=4 keeps the same 3 streams but stores fixed dual-ring predicted residuals.

#ifndef PMSF_COMPRESS_MMSE_DUAL_RING_SYMBOL_COLLECT_OPERATOR_H
#define PMSF_COMPRESS_MMSE_DUAL_RING_SYMBOL_COLLECT_OPERATOR_H

#include "RD/compress/RDTagSymbolCollectOperator.h"

#include <cstddef>
#include <cstdint>
#include <deque>

class MMSEDualRingSymbolCollectOperator : public RDTagSymbolCollectOperator {
  public:
    explicit MMSEDualRingSymbolCollectOperator(double alpha = 0.5)
        : alpha_(alpha) {}

    void collect(MCGAL::Halfedge* seed, const std::vector<MCGAL::Halfedge*>& gates) override;
    void collect(MCGAL::Halfedge* seed) override;
    void collect(std::vector<MCGAL::Halfedge*> seeds) override;

    int exportToBufferWithRangeCoder(char* buffer,
                                     bool enableQuantization = false) override;

    int exportToBuffer(char* buffer, bool enableQuantization = false,
                       bool enableCompress = false) override;
    const pmsf::PMSFSymbolTable* pmsfSymbolTable() const override;
    const pmsf::PMSFSymbolPayloadTable* paperSymbolPayloadTable() const override;

  private:
    double alpha_;
    std::vector<std::deque<std::uint8_t>> gateModeQueues_;
    bool fixedDualRingPrediction_ = false;
    bool clusterModeDecision_ = false;
    mutable pmsf::PMSFSymbolTable paperTableCache_{};
    mutable pmsf::PMSFSymbolPayloadTable paperPayloadTableCache_{};
    mutable std::size_t paperPayloadTableCacheRoundCount_ = static_cast<std::size_t>(-1);
    mutable bool paperPayloadTableCacheStrict_ = false;

    // ------------------------------------------------------------------
    // P1 (2026-10-22, WS-B4): paper predictor stack state.
    //
    // The MMSE paper (§2.2) predicts geometry with per-cluster mode decision
    // over 4 modes: dual-ring alpha=0/0.5/1 and MMSE LS.  The chosen modes are
    // emitted as a 2-bit-per-cluster stream *inside* the residual stream
    // (self-describing flag bit 0x20000000 on the numCoords header word), so
    // no blob-layout change is needed and the baseline artifact form is
    // untouched when the feature is off.
    //
    // Indexed by ENCODER round (export reverses rounds, same as the other
    // per-round queues).
    // ------------------------------------------------------------------
    std::vector<std::vector<std::uint8_t>> gatePredModeQueues_;
    std::vector<std::int32_t> gatePredClusterSizes_;
    int predFixedMode_ = -2;  // -2 = off, -1 = per-cluster decision, 0..3 fixed

    /// Post-pass over the just-collected round: recompute every polyFace
    /// residual with the MMSE predictor stack and record the mode stream.
    void applyMMSEPredictorStack();
};

#endif  // PMSF_COMPRESS_MMSE_DUAL_RING_SYMBOL_COLLECT_OPERATOR_H
