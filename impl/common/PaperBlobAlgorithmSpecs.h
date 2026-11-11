/*
 * @file PaperBlobAlgorithmSpecs.h
 * @brief 各论文算法**专有**的连接性 blob 规格工厂 / 谓词。
 *
 * ## 为什么住在这里
 *
 * 这 39 个函数原先长在 `src/storage/PaperBlobLayoutRegistry.h` 里，
 * 名字直接带算法名（`vqmmcLegacyCodebookConnectivityBlobSpec` 等）。
 * 但 `src/` 是框架层，不该认识任何具体算法（见 lint_src_no_algo_names.sh）。
 *
 * 关键勘察：`PaperBlobLayoutRegistry.h` 的 8 个 `src/` 消费者**没有一个调用
 * 这些工厂** —— 它们 include 那个头只是为了拿 `PaperConnectivityBlobSpec`
 * 等结构体类型。所以这一组可以整体外迁，`src/` 侧不需要反向依赖。
 *
 * ## 与 PaperBlobLayoutRegistry.h 的关系
 *
 * 本头 **include 原头**：这些工厂依赖那边的结构体与中性 helper。
 * 依赖方向是 impl/ -> src/，与框架分层一致。
 * 谁需要这些工厂，就 include 本头。
 */
#pragma once

#include "storage/PaperBlobLayoutRegistry.h"

namespace pmsf {

// Layout 25 sv12: sv8 entropy layout, with oriented-Z / minimum-angle
// mode2 recovery. Never reinterpret legacy sv1..11 with this matcher.
inline constexpr std::int32_t kPCATMStrictPaperOrientedZVersion = 12;
// task-59 B1-alt: identical colour stream (legacy 4-colour rounds), but the
// displacement stream is recoded with the adaptive bit-plane coder from
// PCATMStrictAdaptiveResidual.h (paper §4 records displacements entropy-coded;
// same values, better code). Never reinterpret older scheme versions with it.
inline constexpr std::int32_t kPCATMStrictPaperAdaptiveResidualVersion = 13;

inline PaperConnectivityBlobSpec vqmmcLegacyCodebookConnectivityBlobSpec(
    std::int32_t roundCount) {
    return makePaperConnectivityBlobSpec(
        kPaperBlobLayoutCodec023,
        kPaperBlobSchemeVersionCodec001,
        roundCount,
        kPaperBlobStreamCountCodec003);
}
inline PaperConnectivityBlobSpec pcatmPureConnectivityBlobSpec(
    std::int32_t roundCount,
    std::int32_t schemeVersion = kPaperBlobSchemeVersionCodec001) {
    return makePaperConnectivityBlobSpec(kPaperBlobLayoutCodec021,
                                         schemeVersion,
                                         roundCount,
                                         kPaperBlobStreamCountCodec003);
}
inline constexpr std::int32_t kPCLTTMTagFrenetVersion = kPaperBlobSchemeVersionCodec004;
inline PaperConnectivityBlobSpec pclttmTagConnectivityBlobSpec(
    std::int32_t roundCount, std::int32_t schemeVersion=kPaperBlobSchemeVersionCodec003) {
    return makePaperConnectivityBlobSpec(kPaperBlobLayoutCodec016,
                                         schemeVersion,
                                         roundCount,
                                         kPaperBlobStreamCountCodec003);
}
inline PaperConnectivityBlobSpec pcatmColorConnectivityBlobSpec(
    std::int32_t roundCount) {
    return makePaperConnectivityBlobSpec(kPaperBlobLayoutCodec014,
                                         kPaperBlobSchemeVersionCodec003,
                                         roundCount,
                                         kPaperBlobStreamCountCodec005);
}
inline PaperConnectivityBlobSpec pcatmStrictPaperConnectivityBlobSpec(
    std::int32_t roundCount,
    std::int32_t schemeVersion,
    std::int32_t streamCount = kPaperBlobStreamCountCodec002) {
    return makePaperConnectivityBlobSpec(kPaperBlobLayoutCodec025,
                                         schemeVersion,
                                         roundCount,
                                         streamCount);
}
inline PaperConnectivityBlobSpec pclttmStrictConnectivityBlobSpec(
    std::int32_t blobLayout,
    std::int32_t schemeVersion,
    std::int32_t streamCount) {
    return makePaperConnectivityBlobSpec(blobLayout,
                                         schemeVersion,
                                         /*roundCount=*/1,
                                         streamCount);
}
inline PaperConnectivityBlobFamilySpec pclttmStrictConnectivityBlobFamilySpec() {
    PaperConnectivityBlobFamilySpec spec;
    spec.primaryBlobLayout = kPaperBlobLayoutCodec028;
    spec.secondaryBlobLayout = kPaperBlobLayoutCodec026;
    spec.schemeVersion = kPaperBlobSchemeVersionCodec003;
    spec.roundCount = 1;
    spec.minStreamCount = kPaperBlobStreamCountCodec002;
    spec.maxStreamCount = kPaperBlobStreamCountCodec005;
    return spec;
}
inline PaperConnectivityBlobSpec mmseDualRingConnectivityBlobSpec(
    std::int32_t roundCount,
    std::int32_t schemeVersion,
    std::int32_t streamCount) {
    return makePaperConnectivityBlobSpec(kPaperBlobLayoutCodec022,
                                         schemeVersion,
                                         roundCount,
                                         streamCount);
}
inline bool paperBlobLayoutIsPomarCompatDirectResidual(std::int32_t layout) {
    return layout == kPaperBlobLayoutCodec011;
}
inline bool paperBlobLayoutIsPCATMStrictPaper(std::int32_t layout) {
    return layout == kPaperBlobLayoutCodec025;
}
inline bool paperBlobLayoutIsSegmentedPPMC(std::int32_t layout) {
    return layout == kPaperBlobLayoutCodec031;
}
inline bool paperBlobLayoutIsPPMCRC(std::int32_t layout) {
    return layout == kPaperBlobLayoutCodec003;
}
inline bool paperBlobSchemeVersionIsMMSEDualRing(std::int32_t schemeVersion) {
    switch (schemeVersion) {
        case kPaperBlobSchemeVersionCodec001:
        case kPaperBlobSchemeVersionCodec002:
        case kPaperBlobSchemeVersionCodec003:
        case kPaperBlobSchemeVersionCodec004:
        case kPaperBlobSchemeVersionCodec005:
        case kPaperBlobSchemeVersionCodec006:
        case kPaperBlobSchemeVersionCodec007:
        case kPaperBlobSchemeVersionCodec008:
            return true;
        default:
            return false;
    }
}
inline std::int32_t paperBlobStreamCountMMSEDualRingForScheme(
    std::int32_t schemeVersion) {
    return schemeVersion == kPaperBlobSchemeVersionCodec002
               ? kPaperBlobStreamCountCodec004
               : kPaperBlobStreamCountCodec003;
}
inline bool paperBlobSchemeVersionMMSEDualRingUsesGlobalStreams(
    std::int32_t schemeVersion) {
    return schemeVersion == kPaperBlobSchemeVersionCodec006 ||
           schemeVersion == kPaperBlobSchemeVersionCodec007 ||
           schemeVersion == kPaperBlobSchemeVersionCodec008;
}
inline PaperConnectivityBlobSpec mmseDualRingGlobalConnectivityBlobSpec(
    std::int32_t schemeVersion) {
    return mmseDualRingConnectivityBlobSpec(
        kPaperBlobRoundCountCodec001,
        schemeVersion,
        kPaperBlobStreamCountCodec003);
}
inline bool paperBlobSchemeVersionIsPCATMPure(std::int32_t schemeVersion) {
    switch (schemeVersion) {
        case kPaperBlobSchemeVersionCodec001:
        case kPaperBlobSchemeVersionCodec002:
            return true;
        default:
            return false;
    }
}
inline bool paperBlobSchemeVersionPCATMPureUsesMultiContextIsPoly(
    std::int32_t schemeVersion) {
    return schemeVersion == kPaperBlobSchemeVersionCodec002;
}
inline bool paperBlobSchemeVersionIsPCATMStrictPaper(
    std::int32_t schemeVersion) {
    switch (schemeVersion) {
        case kPaperBlobSchemeVersionCodec001:
        case kPaperBlobSchemeVersionCodec002:
        case kPaperBlobSchemeVersionCodec003:
        case kPaperBlobSchemeVersionCodec004:
        case kPaperBlobSchemeVersionCodec005:
        case kPaperBlobSchemeVersionCodec006:
        case kPaperBlobSchemeVersionCodec007:
        case kPaperBlobSchemeVersionCodec008:
        case kPaperBlobSchemeVersionCodec009:
        case kPaperBlobSchemeVersionCodec010:
        case kPaperBlobSchemeVersionCodec011:
        case kPCATMStrictPaperOrientedZVersion:
        case kPCATMStrictPaperAdaptiveResidualVersion:
            return true;
        default:
            return false;
    }
}
inline bool paperBlobSchemeVersionPCATMStrictPaperHasTopologySideStreams(
    std::int32_t schemeVersion) {
    return schemeVersion ==
           kPaperBlobSchemeVersionCodec003;
}
inline bool paperBlobSchemeVersionPCATMStrictPaperIsColorOnly(
    std::int32_t schemeVersion) {
    switch (schemeVersion) {
        case kPaperBlobSchemeVersionCodec004:
        case kPaperBlobSchemeVersionCodec005:
        case kPaperBlobSchemeVersionCodec006:
        case kPaperBlobSchemeVersionCodec007:
        case kPaperBlobSchemeVersionCodec008:
        case kPaperBlobSchemeVersionCodec009:
        case kPaperBlobSchemeVersionCodec010:
        case kPaperBlobSchemeVersionCodec011:
        case kPCATMStrictPaperOrientedZVersion:
        case kPCATMStrictPaperAdaptiveResidualVersion:
            return true;
        default:
            return false;
    }
}
inline std::int32_t paperBlobStreamCountPCATMStrictPaperForScheme(
    std::int32_t schemeVersion) {
    return paperBlobSchemeVersionPCATMStrictPaperHasTopologySideStreams(
               schemeVersion)
        ? kPaperBlobStreamCountCodec004
        : kPaperBlobStreamCountCodec002;
}
inline bool paperBlobSchemeVersionPCATMStrictPaperUsesGlobalHuffmanResidual(
    std::int32_t schemeVersion) {
    return schemeVersion ==
               kPaperBlobSchemeVersionCodec002 ||
           schemeVersion ==
               kPaperBlobSchemeVersionCodec003 ||
           schemeVersion ==
               kPaperBlobSchemeVersionCodec004;
}
inline bool paperBlobSchemeVersionPCATMStrictPaperUsesGlobalVarintResidual(
    std::int32_t schemeVersion) {
    return schemeVersion ==
               kPaperBlobSchemeVersionCodec005 ||
           schemeVersion ==
               kPaperBlobSchemeVersionCodec008 ||
           schemeVersion == kPCATMStrictPaperOrientedZVersion;
}
inline bool paperBlobSchemeVersionPCATMStrictPaperUsesDenseRangeResidual(
    std::int32_t schemeVersion) {
    return schemeVersion ==
           kPaperBlobSchemeVersionCodec006;
}
inline bool paperBlobSchemeVersionPCATMStrictPaperUsesAxisVarintResidual(
    std::int32_t schemeVersion) {
    return schemeVersion ==
           kPaperBlobSchemeVersionCodec007;
}
inline bool paperBlobSchemeVersionPCATMStrictPaperUsesSplitAxisVarintResidual(
    std::int32_t schemeVersion) {
    return schemeVersion ==
           kPaperBlobSchemeVersionCodec009;
}
inline bool paperBlobSchemeVersionPCATMStrictPaperUsesByteRangeResidual(
    std::int32_t schemeVersion) {
    return schemeVersion ==
           kPaperBlobSchemeVersionCodec010;
}
inline bool paperBlobSchemeVersionPCATMStrictPaperUsesMeppAxisRangeResidual(
    std::int32_t schemeVersion) {
    return schemeVersion ==
           kPaperBlobSchemeVersionCodec011;
}
inline bool paperBlobSchemeVersionPCATMStrictPaperUsesAdaptiveResidual(
    std::int32_t schemeVersion) {
    return schemeVersion == kPCATMStrictPaperAdaptiveResidualVersion;
}
inline bool paperBlobSchemeVersionPCATMStrictPaperUsesGlobalColor(
    std::int32_t schemeVersion) {
    switch (schemeVersion) {
        case kPaperBlobSchemeVersionCodec008:
        case kPaperBlobSchemeVersionCodec009:
        case kPaperBlobSchemeVersionCodec010:
        case kPaperBlobSchemeVersionCodec011:
        case kPCATMStrictPaperOrientedZVersion:
        case kPCATMStrictPaperAdaptiveResidualVersion:
            return true;
        default:
            return false;
    }
}
inline bool paperBlobStreamCountPCLTTMStrictHasOffsets(
    std::int32_t streamCount) {
    return streamCount >= kPaperBlobStreamCountCodec003;
}
inline bool paperBlobStreamCountPCLTTMStrictHasReplay(
    std::int32_t streamCount) {
    return streamCount >= kPaperBlobStreamCountCodec004;
}
inline bool paperBlobStreamCountPCLTTMStrictHasExtra(
    std::int32_t streamCount) {
    return streamCount >= kPaperBlobStreamCountCodec005;
}
inline bool paperBlobStreamCountVQMMCUsesVectorQuantizedBlob(
    std::int32_t streamCount) {
    return streamCount == kPaperBlobStreamCountVectorQuantizedBlob;
}
inline bool paperBlobStreamCountVQMMCUsesLegacyMain(
    std::int32_t streamCount) {
    return streamCount == kPaperBlobStreamCountCodec003;
}
inline bool paperBlobStreamCountIsVQMMCCodebookWrapper(
    std::int32_t streamCount) {
    return paperBlobStreamCountVQMMCUsesLegacyMain(streamCount) ||
           paperBlobStreamCountVQMMCUsesVectorQuantizedBlob(streamCount);
}

}  // namespace pmsf
