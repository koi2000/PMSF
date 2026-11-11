// SPDX-License-Identifier: Apache-2.0
//
// PCLTTMStrictSymbolCollectOperator.h
//
// Encoder-side storage for Alliez & Desbrun 2001 PCLTTM-style patch conquest.
// This is intentionally separate from CODEC_016(16): strict storage follows
// the MEPP2 valence-compression split:
//   stream 0: order-0 arithmetic-coded connectivity alphabet {N,V3,V4,V5,V6}
//   stream 1: per-layer barycenter residual geometry, adaptive arithmetic-coded

#ifndef PMSF_COMPRESS_PCLTTM_STRICT_SYMBOL_COLLECT_OPERATOR_H
#define PMSF_COMPRESS_PCLTTM_STRICT_SYMBOL_COLLECT_OPERATOR_H

#include "compress/SymbolCollectOperator.h"
#include "common/PaperLayoutAliases.h"
#include "common/patch/RDPatchPipeline.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂

namespace pmsf {

constexpr int kPCLTTMStrictBlobLayout = kPaperBlobLayoutPCLTTMStrict;
constexpr int kPCLTTMStrictSchemeVersion =
    kPaperBlobSchemeVersionPCLTTMStrict;

class PCLTTMStrictSymbolCollectOperator : public SymbolCollectOperator {
  public:
    PCLTTMStrictSymbolCollectOperator() = default;

    void collect(MCGAL::Halfedge* h) override;

    int exportToBufferWithRangeCoder(char* buffer,
                                      bool enableQuantization = false) override;
    int exportToBuffer(char* buffer,
                       bool enableQuantization = false,
                       bool enableCompress = false) override;

    const RDPatchPipelineResult& lastResult() const { return lastResult_; }

    const PMSFSymbolTable* pmsfSymbolTable() const override;

    int lastConnectivityBytes() const { return lastConnectivityBytes_; }
    int lastGeometryBytes() const { return lastGeometryBytes_; }
    int lastContainerOverheadBytes() const { return lastContainerOverheadBytes_; }

  protected:
    virtual const char* pmsfChannelPrefix() const { return "pclttm_strict"; }

    int exportStrictToBufferWithLayout(char* buffer,
                                       int blobLayout,
                                       int schemeVersion,
                                       const char* diagLabel);

  private:
    RDPatchPipelineResult lastResult_{};
    mutable PMSFSymbolTable paperTableCache_{};
    int lastConnectivityBytes_ = 0;
    int lastGeometryBytes_ = 0;
    int lastContainerOverheadBytes_ = 0;
};

}  // namespace pmsf

#endif  // PMSF_COMPRESS_PCLTTM_STRICT_SYMBOL_COLLECT_OPERATOR_H
