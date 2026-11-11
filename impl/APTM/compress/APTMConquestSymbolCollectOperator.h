// SPDX-License-Identifier: Apache-2.0
//
// APTMConquestSymbolCollectOperator.h
//
// APTM paper-aligned storage path. It reuses the existing strict
// Alliez-Desbrun/PCLTTM patch-conquest implementation, but writes a distinct
// BlobLayout so APTM runs can be tracked independently.

#ifndef PMSF_COMPRESS_APTM_CONQUEST_SYMBOL_COLLECT_OPERATOR_H
#define PMSF_COMPRESS_APTM_CONQUEST_SYMBOL_COLLECT_OPERATOR_H

#include "common/strict/PCLTTMStrictSymbolCollectOperator.h"
#include "common/PaperLayoutAliases.h"
#include "core/PMSFBoundarySideChannel.h"
#include "APTM/boundary/APTMPMSFBoundary.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂
#include "storage/PMSFSymbolStorageAdapter.h"

#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <stdexcept>

namespace pmsf {

constexpr int kAPTMConquestBlobLayout = kPaperBlobLayoutAPTMConquest;
constexpr int kAPTMConquestSchemeVersion = kPCLTTMStrictSchemeVersion;

class APTMConquestSymbolCollectOperator final
    : public PCLTTMStrictSymbolCollectOperator {
  public:
    void collect(MCGAL::Halfedge* h) override {
        applyStorageDefaults();
        PCLTTMStrictSymbolCollectOperator::collect(h);
    }

    int exportToBufferWithRangeCoder(char* buffer,
                                     bool enableQuantization = false) override {
        (void)enableQuantization;
        requireAPTMPaperExactIfRequested();
        applyStorageDefaults();
        return exportStrictToBufferWithLayout(buffer,
                                              kAPTMConquestBlobLayout,
                                              kAPTMConquestSchemeVersion,
                                              "APTMConquest");
    }

    const PMSFSymbolTable* pmsfSymbolTable() const override {
        const PMSFSymbolTable* base =
            PCLTTMStrictSymbolCollectOperator::pmsfSymbolTable();
        if (!base || base->empty()) {
            return base;
        }
        pmsf::copyPMSFSymbolTableWithRowAugmentation(
            base, paperTableCache_, [](PMSFSymbolRecord& row) {
            pmsf::appendAPTMConquestBoundary(row.delta);
        });
        return &paperTableCache_;
    }

  protected:
    const char* pmsfChannelPrefix() const override { return "aptm_conquest"; }

  private:
    mutable PMSFSymbolTable paperTableCache_{};

    static void requireAPTMPaperExactIfRequested() {
        if (!pmsf::boundaryFlagOn(aptm_boundary::kPaperExactSpec)) {
            return;
        }
        throw std::runtime_error(
            "[APTMConquest] paper-exact APTM requested, but this layout is "
            "still a PCLTTM strict bridge; anchor-aware reconstruction and "
            "APTM polar geometry are not implemented yet");
    }

    static void setDefaultEnv(const char* key, const char* value) {
        if (std::getenv(key) == nullptr) {
            setenv(key, value, /*overwrite=*/0);
        }
    }

    static void applyStorageDefaults() {
        // MEPP2-style storage split:
        //   stream 0: adaptive range-coded {N,V3,V4,V5,V6}
        //   stream 1: adaptive range-coded geometry residuals
        //   stream 3: compact patch-locator ranks for deterministic replay
        //
        // The last stream is an implementation bridge for MCGAL's current
        // halfedge/proxy ordering; it is separate from the valence stream.
        setDefaultEnv("PMSF_PCLTTM_GEOM_MODE", "cartesian");
        setDefaultEnv("PMSF_PCLTTM_REVERSE_V_ONLY", "1");
        setDefaultEnv("PMSF_PCLTTM_REVERSE_RANKS", "1");
        setDefaultEnv("PMSF_PCLTTM_PACK_AS_IS", "1");
        setDefaultEnv("PMSF_PCLTTM_WRITE_RANKS", "1");
        setDefaultEnv("PMSF_PCLTTM_RANK_CODEC", "aptm_locator_v1");
        setDefaultEnv("PMSF_PCLTTM_WRITE_RINGS", "0");
        setDefaultEnv("PMSF_PCLTTM_WRITE_RECON_RING_IDS", "0");
        setDefaultEnv("PMSF_PCLTTM_WRITE_RECON_ORIGINAL_IDS", "0");
        setDefaultEnv("PMSF_PCLTTM_USE_RANK_REPLAY", "1");
        setDefaultEnv("PMSF_PCLTTM_USE_RING_REPLAY", "0");
        setDefaultEnv("PMSF_PCLTTM_USE_RD_REPLAY", "0");
        setDefaultEnv("PMSF_RDSTRICT_BACKWARD_DEC_V3", "1");
    }
};

}  // namespace pmsf

#endif  // PMSF_COMPRESS_APTM_CONQUEST_SYMBOL_COLLECT_OPERATOR_H
