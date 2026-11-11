/*
 * @file PaperLayoutAliases.h
 * @brief 各落盘布局的**可读别名**（只在 impl/ 下使用）。
 *
 * 背景：`src/storage/PaperBlobLayoutRegistry.h` 里的布局号是**线格式**，必须保持数值
 * 稳定；但「哪个算法用哪个号」属于算法知识，不该出现在 `src/`（反硬编码纪律：
 * `src/` 核心禁止算法名，算法知识只能表达为数据或插件）。
 *
 * 所以拆成两层：
 *   - `src/` 只认中性 id（kPaperBlobLayoutCodecNNN，NNN = 数值），用于读写与透传；
 *   - `impl/` 通过本文件给这些 id 起可读名字（哪个算法在用、什么形态）。
 *
 * 数值一个字都没动 —— 产物字节不变，回归门禁可证。
 */
#ifndef PMSF_IMPL_PAPER_LAYOUT_ALIASES_H
#define PMSF_IMPL_PAPER_LAYOUT_ALIASES_H

#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂

namespace pmsf {

inline constexpr std::int32_t kPaperBlobLayoutPPMCRC = kPaperBlobLayoutCodec003;
inline constexpr std::int32_t kPaperBlobLayoutDefaultPPMCLite = kPaperBlobLayoutCodec006;
inline constexpr std::int32_t kPaperBlobLayoutPomarCompatFrenet = kPaperBlobLayoutCodec010;
inline constexpr std::int32_t kPaperBlobLayoutPomarCompatDirectResidual = kPaperBlobLayoutCodec011;
inline constexpr std::int32_t kPaperBlobLayoutPomar4Stream = kPaperBlobLayoutCodec012;
inline constexpr std::int32_t kPaperBlobLayoutPCATMColor = kPaperBlobLayoutCodec014;
inline constexpr std::int32_t kPaperBlobLayoutPCLTTMTag = kPaperBlobLayoutCodec016;
inline constexpr std::int32_t kPaperBlobLayoutPCATMZColor = kPaperBlobLayoutCodec017;
inline constexpr std::int32_t kPaperBlobLayoutPCATMPaper = kPaperBlobLayoutCodec020;
inline constexpr std::int32_t kPaperBlobLayoutPCATMPure = kPaperBlobLayoutCodec021;
inline constexpr std::int32_t kPaperBlobLayoutMMSEDualRing = kPaperBlobLayoutCodec022;
inline constexpr std::int32_t kPaperBlobLayoutVQMMCCodebook = kPaperBlobLayoutCodec023;
inline constexpr std::int32_t kPaperBlobLayoutPCATMStrictPaper = kPaperBlobLayoutCodec025;
inline constexpr std::int32_t kPaperBlobLayoutAPTMConquest = kPaperBlobLayoutCodec026;
inline constexpr std::int32_t kPaperBlobLayoutMMSEStrict = kPaperBlobLayoutCodec027;
inline constexpr std::int32_t kPaperBlobLayoutPCLTTMStrict = kPaperBlobLayoutCodec028;
inline constexpr std::int32_t kPaperBlobLayoutVQMMCStrict = kPaperBlobLayoutCodec030;
inline constexpr std::int32_t kPaperBlobLayoutPPMCSegmentedRC = kPaperBlobLayoutCodec031;

inline constexpr std::int32_t kPaperBlobRoundCountMMSEDualRingGlobalStreams = kPaperBlobRoundCountCodec001;
inline constexpr std::int32_t kPaperBlobSchemeVersionMMSEDualRingV1 = kPaperBlobSchemeVersionCodec001;
inline constexpr std::int32_t kPaperBlobSchemeVersionMMSEStrict = kPaperBlobSchemeVersionCodec001;
inline constexpr std::int32_t kPaperBlobSchemeVersionPCATMPaper = kPaperBlobSchemeVersionCodec001;
inline constexpr std::int32_t kPaperBlobSchemeVersionPCATMPure = kPaperBlobSchemeVersionCodec001;
inline constexpr std::int32_t kPaperBlobSchemeVersionPCATMStrictPaper = kPaperBlobSchemeVersionCodec001;
inline constexpr std::int32_t kPaperBlobSchemeVersionPCATMZColor = kPaperBlobSchemeVersionCodec001;
inline constexpr std::int32_t kPaperBlobSchemeVersionVQMMCCodebook = kPaperBlobSchemeVersionCodec001;
inline constexpr std::int32_t kPaperBlobSchemeVersionMMSEDualRingV2 = kPaperBlobSchemeVersionCodec002;
inline constexpr std::int32_t kPaperBlobSchemeVersionPCATMPureV2 = kPaperBlobSchemeVersionCodec002;
inline constexpr std::int32_t kPaperBlobSchemeVersionMMSEDualRingV3 = kPaperBlobSchemeVersionCodec003;
inline constexpr std::int32_t kPaperBlobSchemeVersionPCATMColor = kPaperBlobSchemeVersionCodec003;
inline constexpr std::int32_t kPaperBlobSchemeVersionPCLTTMStrict = kPaperBlobSchemeVersionCodec003;
inline constexpr std::int32_t kPaperBlobSchemeVersionPCLTTMTag = kPaperBlobSchemeVersionCodec003;
inline constexpr std::int32_t kPaperBlobSchemeVersionMMSEDualRingV4 = kPaperBlobSchemeVersionCodec004;
inline constexpr std::int32_t kPaperBlobSchemeVersionMMSEDualRingV5 = kPaperBlobSchemeVersionCodec005;
inline constexpr std::int32_t kPaperBlobSchemeVersionMMSEDualRingV6 = kPaperBlobSchemeVersionCodec006;
inline constexpr std::int32_t kPaperBlobSchemeVersionMMSEDualRingV7 = kPaperBlobSchemeVersionCodec007;
inline constexpr std::int32_t kPaperBlobSchemeVersionMMSEDualRingV8 = kPaperBlobSchemeVersionCodec008;
inline constexpr std::int32_t kPaperBlobStreamMMSEDualRingValence = kPaperBlobStreamCodec000;
inline constexpr std::int32_t kPaperBlobStreamPCATMColorValence = kPaperBlobStreamCodec000;
inline constexpr std::int32_t kPaperBlobStreamPCATMPureValence = kPaperBlobStreamCodec000;
inline constexpr std::int32_t kPaperBlobStreamPCATMStrictPaperColor = kPaperBlobStreamCodec000;
inline constexpr std::int32_t kPaperBlobStreamPCLTTMStrictConnectivity = kPaperBlobStreamCodec000;
inline constexpr std::int32_t kPaperBlobStreamPCLTTMTagValence = kPaperBlobStreamCodec000;
inline constexpr std::int32_t kPaperBlobStreamVQMMCLegacyValence = kPaperBlobStreamCodec000;
inline constexpr std::int32_t kPaperBlobStreamMMSEDualRingIsPoly = kPaperBlobStreamCodec001;
inline constexpr std::int32_t kPaperBlobStreamPCATMColorIsPoly = kPaperBlobStreamCodec001;
inline constexpr std::int32_t kPaperBlobStreamPCATMPureIsPoly = kPaperBlobStreamCodec001;
inline constexpr std::int32_t kPaperBlobStreamPCATMStrictPaperGeometry = kPaperBlobStreamCodec001;
inline constexpr std::int32_t kPaperBlobStreamPCLTTMStrictGeometry = kPaperBlobStreamCodec001;
inline constexpr std::int32_t kPaperBlobStreamPCLTTMTagFusedFaceSym = kPaperBlobStreamCodec001;
inline constexpr std::int32_t kPaperBlobStreamVQMMCLegacyIsPoly = kPaperBlobStreamCodec001;
inline constexpr std::int32_t kPaperBlobStreamMMSEDualRingMode = kPaperBlobStreamCodec002;
inline constexpr std::int32_t kPaperBlobStreamMMSEDualRingResidual = kPaperBlobStreamCodec002;
inline constexpr std::int32_t kPaperBlobStreamPCATMColorEdgeSym = kPaperBlobStreamCodec002;
inline constexpr std::int32_t kPaperBlobStreamPCATMPureResidual = kPaperBlobStreamCodec002;
inline constexpr std::int32_t kPaperBlobStreamPCATMStrictPaperTopologyValence = kPaperBlobStreamCodec002;
inline constexpr std::int32_t kPaperBlobStreamPCLTTMStrictOffsets = kPaperBlobStreamCodec002;
inline constexpr std::int32_t kPaperBlobStreamPCLTTMTagResidual = kPaperBlobStreamCodec002;
inline constexpr std::int32_t kPaperBlobStreamVQMMCLegacyResidual = kPaperBlobStreamCodec002;
inline constexpr std::int32_t kPaperBlobStreamMMSEDualRingResidualWithMode = kPaperBlobStreamCodec003;
inline constexpr std::int32_t kPaperBlobStreamPCATMColorColor = kPaperBlobStreamCodec003;
inline constexpr std::int32_t kPaperBlobStreamPCATMStrictPaperTopologyIsPoly = kPaperBlobStreamCodec003;
inline constexpr std::int32_t kPaperBlobStreamPCLTTMStrictReplay = kPaperBlobStreamCodec003;
inline constexpr std::int32_t kPaperBlobStreamPCATMColorResidual = kPaperBlobStreamCodec004;
inline constexpr std::int32_t kPaperBlobStreamPCLTTMStrictExtra = kPaperBlobStreamCodec004;
inline constexpr std::int32_t kPaperBlobStreamCountPCATMStrictPaper = kPaperBlobStreamCountCodec002;
inline constexpr std::int32_t kPaperBlobStreamCountPCLTTMStrictBase = kPaperBlobStreamCountCodec002;
inline constexpr std::int32_t kPaperBlobStreamCountMMSEDualRing = kPaperBlobStreamCountCodec003;
inline constexpr std::int32_t kPaperBlobStreamCountPCATMPure = kPaperBlobStreamCountCodec003;
inline constexpr std::int32_t kPaperBlobStreamCountPCLTTMStrictWithOffsets = kPaperBlobStreamCountCodec003;
inline constexpr std::int32_t kPaperBlobStreamCountPCLTTMTag = kPaperBlobStreamCountCodec003;
inline constexpr std::int32_t kPaperBlobStreamCountVQMMCLegacyMain = kPaperBlobStreamCountCodec003;
inline constexpr std::int32_t kPaperBlobStreamCountMMSEDualRingWithMode = kPaperBlobStreamCountCodec004;
inline constexpr std::int32_t kPaperBlobStreamCountPCATMPaper = kPaperBlobStreamCountCodec004;
inline constexpr std::int32_t kPaperBlobStreamCountPCATMStrictPaperWithTopologySide = kPaperBlobStreamCountCodec004;
inline constexpr std::int32_t kPaperBlobStreamCountPCATMZColor = kPaperBlobStreamCountCodec004;
inline constexpr std::int32_t kPaperBlobStreamCountPCLTTMStrictWithReplay = kPaperBlobStreamCountCodec004;
inline constexpr std::int32_t kPaperBlobStreamCountVQMMCTarget = kPaperBlobStreamCountCodec004;
inline constexpr std::int32_t kPaperBlobStreamCountPCATMColor = kPaperBlobStreamCountCodec005;
inline constexpr std::int32_t kPaperBlobStreamCountPCLTTMStrictWithExtra = kPaperBlobStreamCountCodec005;
}  // namespace pmsf

#endif  // PMSF_IMPL_PAPER_LAYOUT_ALIASES_H
