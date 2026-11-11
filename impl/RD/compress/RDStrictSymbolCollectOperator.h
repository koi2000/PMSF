// SPDX-License-Identifier: Apache-2.0
//
// RDStrictSymbolCollectOperator.h
//
// Encoder-side collector for the paper-aligned RD storage path.  The first
// step intentionally reuses the existing RDPatch conquest driver, but emits a
// separate RD_STRICT(29) blob so future changes can converge toward the paper
// layout without perturbing RD_PATCH(24) experiments.

#ifndef PMSF_COMPRESS_RD_STRICT_SYMBOL_COLLECT_OPERATOR_H
#define PMSF_COMPRESS_RD_STRICT_SYMBOL_COLLECT_OPERATOR_H

#include "compress/SymbolCollectOperator.h"
#include "common/patch/RDPatchPipeline.h"

#include <utility>
#include <vector>

namespace pmsf {

class RDStrictSymbolCollectOperator final : public SymbolCollectOperator {
  public:
    RDStrictSymbolCollectOperator() = default;

    void collect(MCGAL::Halfedge* h) override;

    int exportToBuffer(char* buffer,
                       bool enableQuantization = false,
                       bool enableCompress = false) override;

    int exportToBufferWithRangeCoder(char* buffer,
                                     bool enableQuantization = false) override;

    void setPipelineConfig(RDPatchPipelineConfig cfg) { cfg_ = std::move(cfg); }

    const RDPatchPipelineResult& lastResult() const { return lastResult_; }

    const PMSFSymbolTable* pmsfSymbolTable() const override;

    void setPrebuiltResult(RDPatchPipelineResult result) {
        lastResult_ = std::move(result);
        splitDecodeRounds_.clear();
        prebuiltBlob_.clear();
        paperTableCache_.clear();
    }

    int lastConnectivityBytes() const { return lastConnectivityBytes_; }
    int lastGeometryBytes() const { return lastGeometryBytes_; }
    int lastSideInfoBytes() const { return lastSideInfoBytes_; }
    int lastContainerOverheadBytes() const { return lastContainerOverheadBytes_; }

    void setPrebuiltBlob(std::vector<uint8_t> blob,
                         int connectivityBytes,
                         int geometryBytes,
                         int sideInfoBytes,
                         int overheadBytes) {
        prebuiltBlob_ = std::move(blob);
        lastConnectivityBytes_ = connectivityBytes;
        lastGeometryBytes_ = geometryBytes;
        lastSideInfoBytes_ = sideInfoBytes;
        lastContainerOverheadBytes_ = overheadBytes;
        paperTableCache_.clear();
    }

  private:
    RDPatchPipelineConfig cfg_{};
    RDPatchPipelineResult lastResult_{};
    mutable PMSFSymbolTable paperTableCache_{};
    std::vector<RDPatchPipelineResult> splitDecodeRounds_{};
    std::vector<uint8_t> prebuiltBlob_{};
    int lastConnectivityBytes_ = 0;
    int lastGeometryBytes_ = 0;
    int lastSideInfoBytes_ = 0;
    int lastContainerOverheadBytes_ = 0;
};

}  // namespace pmsf

#endif  // PMSF_COMPRESS_RD_STRICT_SYMBOL_COLLECT_OPERATOR_H
