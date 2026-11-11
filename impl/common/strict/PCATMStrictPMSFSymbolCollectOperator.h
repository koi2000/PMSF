// SPDX-License-Identifier: Apache-2.0
//
// PCATMStrictPMSFSymbolCollectOperator.h
//
// Encoder-side storage for Cohen-Or / Levin / Remez PCATM §4:
// one BFS triangle color bit-vector per level, LZ-compressed, plus one
// barycenter displacement vector per removed vertex.

#ifndef PMSF_COMPRESS_PCATM_STRICT_PAPER_SYMBOL_COLLECT_OPERATOR_H
#define PMSF_COMPRESS_PCATM_STRICT_PAPER_SYMBOL_COLLECT_OPERATOR_H

#include "PCATM/compress/PCATMPMSFSymbolCollectOperator.h"
#include "common/PaperLayoutAliases.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂

#include <deque>
#include <vector>

constexpr int kPCATMStrictPaperBlobLayout =
    pmsf::kPaperBlobLayoutPCATMStrictPaper;
constexpr int kPCATMStrictPaperSchemeVersion =
    pmsf::kPaperBlobSchemeVersionPCATMStrictPaper;
constexpr int kPCATMStrictPaperGlobalHuffmanSchemeVersion =
    pmsf::kPaperBlobSchemeVersionCodec002;
constexpr int kPCATMStrictPaperMixedColorSchemeVersion =
    pmsf::kPaperBlobSchemeVersionCodec003;
constexpr int kPCATMStrictPaperColorOnlySchemeVersion =
    pmsf::kPaperBlobSchemeVersionCodec004;
constexpr int kPCATMStrictPaperColorOnlyVarintResidualSchemeVersion =
    pmsf::kPaperBlobSchemeVersionCodec005;
constexpr int kPCATMStrictPaperColorOnlyDenseRangeResidualSchemeVersion =
    pmsf::kPaperBlobSchemeVersionCodec006;
constexpr int kPCATMStrictPaperColorOnlyAxisVarintResidualSchemeVersion =
    pmsf::kPaperBlobSchemeVersionCodec007;
constexpr int
    kPCATMStrictPaperColorOnlyGlobalColorVarintResidualSchemeVersion =
        pmsf::kPaperBlobSchemeVersionCodec008;
constexpr int kPCATMStrictPaperColorOnlySplitAxisVarintResidualSchemeVersion =
    pmsf::kPaperBlobSchemeVersionCodec009;
constexpr int kPCATMStrictPaperColorOnlyVarintByteRangeResidualSchemeVersion =
    pmsf::kPaperBlobSchemeVersionCodec010;
constexpr int kPCATMStrictPaperColorOnlyMeppAxisRangeResidualSchemeVersion =
    pmsf::kPaperBlobSchemeVersionCodec011;

class PCATMStrictPMSFSymbolCollectOperator : public PCATMPMSFSymbolCollectOperator {
  public:
    PCATMStrictPMSFSymbolCollectOperator() = default;

    void collect(MCGAL::Halfedge* seed, const std::vector<MCGAL::Halfedge*>& gates) override;
    void collect(MCGAL::Halfedge* seed) override;
    void collect(std::vector<MCGAL::Halfedge*> seeds) override;

    int exportToBufferWithRangeCoder(char* buffer,
                                      bool enableQuantization = false) override;
    int exportToBuffer(char* buffer,
                       bool enableQuantization = false,
                       bool enableCompress = false) override;

    int lastConnectivityBytes() const { return lastConnectivityBytes_; }
    int lastGeometryBytes() const { return lastGeometryBytes_; }
    int lastContainerOverheadBytes() const { return lastContainerOverheadBytes_; }
    int lastRawColorBits() const { return lastRawColorBits_; }
    int lastRemovedVertices() const { return lastRemovedVertices_; }
    const pmsf::PMSFSymbolTable* pmsfSymbolTable() const override;

  protected:
    const char* pmsfChannelPrefix() const override { return "pcatm_strict"; }

  private:
    std::vector<std::vector<uint8_t>> strictColorSymbolsPerRound_;
    std::vector<std::vector<uint8_t>> strictMode2SymbolsPerRound_;
    std::vector<std::deque<MCGAL::PointInt>> strictResidualQueues_;
    std::vector<std::deque<MCGAL::PointInt>> strictMode2ResidualQueues_;
    std::vector<uint8_t> strictMode2RecoverablePerRound_;
    // WS-B task-18: per-round mode-2 side info (2-bit candidate zero-pair index
    // per patch, in decoder recovery order). Only used when
    // PMSF_PCATM_STRICT_2COLOR_SIDE=1; never affects default artifacts.
    std::vector<std::vector<uint8_t>> strictMode2SidePerRound_;
    // Diagnostic only (PMSF_PCATM_MODE2_DIAG): per-round mode4 colors kept
    // ONLY on blue-less (deg3/4) components, zero elsewhere. Measures the
    // irreducible deg3/4 connectivity cost that any mode2 side-channel must
    // re-transmit. Never serialized.
    std::vector<std::vector<uint8_t>> strictBlueLessColorPerRound_;
    // mode-3 (PMSF_PCATM_MODE2 lever) per-round state.
    std::vector<std::vector<uint8_t>> strictMode3SymbolsPerRound_;
    std::vector<std::deque<MCGAL::PointInt>> strictMode3ResidualQueues_;
    std::vector<uint8_t> strictMode3RecoverablePerRound_;
    mutable pmsf::PMSFSymbolTable paperTableCache_{};

    int lastConnectivityBytes_ = 0;
    int lastGeometryBytes_ = 0;
    int lastContainerOverheadBytes_ = 0;
    int lastRawColorBits_ = 0;
    int lastRemovedVertices_ = 0;
    int lastDegree3_ = 0;
    int lastDegree4_ = 0;
    int lastDegree5_ = 0;
    int lastDegree6_ = 0;
    int lastDegreeOther_ = 0;
    int lastSchemeVersion_ = 0;
    int lastTopologySideBytes_ = 0;
    int lastMode2Rounds_ = 0;
    int lastMode4Rounds_ = 0;
};

#endif  // PMSF_COMPRESS_PCATM_STRICT_PAPER_SYMBOL_COLLECT_OPERATOR_H
