#ifndef PMSF_STORAGE_PAPER_BLOB_LAYOUT_REGISTRY_H
#define PMSF_STORAGE_PAPER_BLOB_LAYOUT_REGISTRY_H

#include <cstdint>
#include <stdexcept>
#include <string>

namespace pmsf {

inline constexpr std::int32_t kPaperBlobLayoutUnknown = 0;
inline constexpr std::int32_t kPaperBlobLayoutRangeCodedPred = 1;
inline constexpr std::int32_t kPaperBlobLayoutBasic = 2;
inline constexpr std::int32_t kPaperBlobLayoutCodec003 = 3;
inline constexpr std::int32_t kPaperBlobLayoutOptimizedPlain = 4;
inline constexpr std::int32_t kPaperBlobLayoutValenceRC = 5;
inline constexpr std::int32_t kPaperBlobLayoutCodec006 = 6;
inline constexpr std::int32_t kPaperBlobLayoutRDGate = 7;
inline constexpr std::int32_t kPaperBlobLayoutRDGateOnly = 8;
inline constexpr std::int32_t kPaperBlobLayoutValencePatchOnly = 9;
inline constexpr std::int32_t kPaperBlobLayoutCodec010 = 10;
inline constexpr std::int32_t kPaperBlobLayoutCPMEdgeSplit = 11;
inline constexpr std::int32_t kPaperBlobLayoutCodec011 =
    kPaperBlobLayoutCPMEdgeSplit;
inline constexpr std::int32_t kPaperBlobLayoutCodec012 = 12;
inline constexpr std::int32_t kPaperBlobLayoutRDBitplane = 13;
inline constexpr std::int32_t kPaperBlobLayoutCodec014 = 14;
// P13.4 (B)②：valence-conquest 原生 codec 的连通性流。
// 产出：`ConquestSymbolCollectOperator::exportToBufferWithRangeCoder`；
// 消费：`ConquestSymbolReadOperator::importFromRangeCodedBuffer`
//       + `ConquestReconstructOperator::reconstruct`（调 `pmsf::decodeLayer`）。
// 值 15 此前空闲（0–14、16–32 均已占用），按"旧值不可变"纪律**只能**取空闲值。
inline constexpr std::int32_t kPaperBlobLayoutConquestRC = 15;

inline constexpr std::int32_t kPaperBlobLayoutCodec016 = 16;
inline constexpr std::int32_t kPaperBlobLayoutCodec017 = 17;
inline constexpr std::int32_t kPaperBlobLayoutRDTag = 18;
inline constexpr std::int32_t kPaperBlobLayoutReserved19 = 19;
inline constexpr std::int32_t kPaperBlobLayoutCodec020 = 20;
inline constexpr std::int32_t kPaperBlobLayoutCodec021 = 21;
inline constexpr std::int32_t kPaperBlobLayoutCodec022 = 22;
inline constexpr std::int32_t kPaperBlobLayoutCodec023 = 23;
/// M5-3: QEM progressive four-stream payload carried verbatim in PaperStream v2
/// (PSM2 multi-stream bundle). Container-level provenance only; the QEM decoder
/// reads the bundle directly, so no `paperImportRangeCodedBlobForLayout` case is
/// registered for this value.
inline constexpr std::int32_t kPaperBlobLayoutQEMProgressive = 24;
inline constexpr std::int32_t kPaperBlobLayoutCodec025 = 25;
inline constexpr std::int32_t kPaperBlobLayoutCodec026 = 26;
inline constexpr std::int32_t kPaperBlobLayoutCodec027 = 27;
inline constexpr std::int32_t kPaperBlobLayoutCodec028 = 28;
inline constexpr std::int32_t kPaperBlobLayoutRDStrict = 29;
inline constexpr std::int32_t kPaperBlobLayoutCodec030 = 30;
inline constexpr std::int32_t kPaperBlobLayoutCodec031 = 31;
inline constexpr std::int32_t kPaperBlobLayoutSegmentedRangeCodedPred = 32;

/// Canonical facet/edge/vertex/point + side-channel layout used by the
/// unified codec=4 storage path. This is not an algorithm-native layout;
/// it signals the deserializer to use the canonical reader.
inline constexpr std::int32_t kPaperBlobLayoutCanonical = 100;

inline constexpr std::int32_t kPaperBlobSchemeVersionValencePatchOnly = 1;
inline constexpr std::int32_t kPaperBlobSchemeVersionCodec001 = 1;
inline constexpr std::int32_t kPaperBlobSchemeVersionCodec003 = 3;
inline constexpr std::int32_t kPaperBlobSchemeVersionRDTagV1 = 1;
inline constexpr std::int32_t kPaperBlobSchemeVersionRDTagV2 = 2;
inline constexpr std::int32_t kPaperBlobSchemeVersionRDTagV3 = 3;
inline constexpr std::int32_t kPaperBlobSchemeVersionRDTagV4 = 4;
inline constexpr std::int32_t kPaperBlobSchemeVersionRDTagV5 = 5;
inline constexpr std::int32_t kPaperBlobSchemeVersionRDTagV6 = 6;
inline constexpr std::int32_t kPaperBlobSchemeVersionRDTagV7 = 7;
inline constexpr std::int32_t kPaperBlobSchemeVersionRDTagV8 = 8;
inline constexpr std::int32_t kPaperBlobSchemeVersionRDTagV9 = 9;
inline constexpr std::int32_t kPaperBlobSchemeVersionRDTagV10 = 10;
inline constexpr std::int32_t kPaperBlobSchemeVersionRDTagV11 = 11;
inline constexpr std::int32_t kPaperBlobSchemeVersionRDTagV12 = 12;
inline constexpr std::int32_t kPaperBlobSchemeVersionRDTagV13 = 13;
inline constexpr std::int32_t kPaperBlobSchemeVersionRDTagV14 = 14;
inline constexpr std::int32_t kPaperBlobSchemeVersionRDTagV15 = 15;
inline constexpr std::int32_t kPaperBlobSchemeVersionRDTagV16 = 16;
inline constexpr std::int32_t kPaperBlobSchemeVersionRDTagV17 = 17;
inline constexpr std::int32_t kPaperBlobSchemeVersionRDTagV18 = 18;
// PHASE62 物品2：sv=6 的线格式**修正版**。
//
// 背景：`a2027aa` 把 sv=6 内部的 edgeSym 流从「单模型 3-bit range coder」换成了
// 「6 上下文 (isPoly, min(onesSoFar,2)) range coder」，**却没有 bump sv**。
// 两条流的前 4 字节都还是 `numBits`，长度前缀相同 ⇒ 旧 blob 用新解码器
// **不抛错、静默解出垃圾**（附 29 在案风险）。
//
// 本版 = **V6 逐谓词等价**（同 5 段流、同 adjustment、同 residual 路径），
// 唯一差别是 edgeSym 用上下文编码器。于是：
//   - 老 V6 blob → 仍走老解码器 ⇒ **不再被误读**；
//   - 新 V6Ctx blob → 走上下文解码器。
// 版本字段是定宽 u16，故**产物字节数不变**，只有那 2 字节版本号不同。
inline constexpr std::int32_t kPaperBlobSchemeVersionRDTagV6Ctx = 19;

/// 该 sv 属于「V6 家族」：流布局与 V6 完全相同，只在 edgeSym 熵编码器上分叉。
inline bool paperBlobSchemeVersionRDTagIsV6Family(std::int32_t schemeVersion) {
    return schemeVersion == kPaperBlobSchemeVersionRDTagV6 ||
           schemeVersion == kPaperBlobSchemeVersionRDTagV6Ctx;
}

/// edgeSym 流是否用 (isPoly, onesSoFar) 6 上下文编码器（**仅** V6Ctx）。
inline bool paperBlobSchemeVersionRDTagUsesContextEdgeSym(
    std::int32_t schemeVersion) {
    return schemeVersion == kPaperBlobSchemeVersionRDTagV6Ctx;
}
inline constexpr std::int32_t kPaperBlobSchemeVersionRDStrict = 1;
inline constexpr std::int32_t kPaperBlobSchemeVersionCodec002 = 2;
inline constexpr std::int32_t kPaperBlobSchemeVersionVectorQuantizedV1 = 1;
inline constexpr std::int32_t kPaperBlobSchemeVersionVectorQuantizedV2 = 2;
inline constexpr std::int32_t kPaperBlobSchemeVersionVectorQuantizedV3 = 3;
inline constexpr std::int32_t kPaperBlobSchemeVersionVectorQuantizedV5 = 5;
inline constexpr std::int32_t kPaperBlobSchemeVersionVectorQuantizedV6 = 6;
inline constexpr std::int32_t kPaperBlobSchemeVersionCodec004 = 4;
inline constexpr std::int32_t kPaperBlobSchemeVersionCodec005 = 5;
inline constexpr std::int32_t kPaperBlobSchemeVersionCodec006 = 6;
inline constexpr std::int32_t kPaperBlobSchemeVersionCodec007 = 7;
inline constexpr std::int32_t kPaperBlobSchemeVersionCodec008 = 8;
inline constexpr std::int32_t
    kPaperBlobSchemeVersionCodec009 = 9;
inline constexpr std::int32_t
    kPaperBlobSchemeVersionCodec010 = 10;
inline constexpr std::int32_t
    kPaperBlobSchemeVersionCodec011 = 11;

inline constexpr std::int32_t kPaperBlobStreamCountValencePatchOnly = 3;
inline constexpr std::int32_t kPaperBlobStreamCountRDStrict = 3;
inline constexpr std::int32_t kPaperBlobStreamCountCodec005 = 5;
inline constexpr std::int32_t kPaperBlobStreamCountCodec003 = 3;
inline constexpr std::int32_t kPaperBlobStreamCountPatchColor = 4;
inline constexpr std::int32_t kPaperBlobStreamCountCodec004 =
    kPaperBlobStreamCountPatchColor;

inline constexpr std::int32_t kPaperBlobStreamCountCodec002 = 2;

inline constexpr std::int32_t kPaperBlobRoundCountCodec001 = 1;
inline constexpr std::int32_t kPaperBlobStreamCountVectorQuantizedBlob = 4;

inline constexpr std::int32_t kPaperBlobStreamCountRDTagBase = 3;
inline constexpr std::int32_t kPaperBlobStreamCountRDTagWithFrenet = 4;
inline constexpr std::int32_t kPaperBlobStreamCountRDTagWithAdjustment = 5;

inline constexpr std::int32_t kPaperBlobStreamValencePatchOnlyValence6 = 0;
inline constexpr std::int32_t kPaperBlobStreamValencePatchOnlyEdgeSym = 1;
inline constexpr std::int32_t kPaperBlobStreamValencePatchOnlyResidual = 2;

inline constexpr std::int32_t kPaperBlobStreamCodec000 = 0;
inline constexpr std::int32_t kPaperBlobStreamCodec001 = 1;
inline constexpr std::int32_t kPaperBlobStreamCodec002 = 2;
inline constexpr std::int32_t kPaperBlobStreamCodec003 = 3;
inline constexpr std::int32_t kPaperBlobStreamCodec004 = 4;

inline constexpr std::int32_t kPaperBlobStreamPatchColorValence = 0;
inline constexpr std::int32_t kPaperBlobStreamPatchColorIsPoly = 1;
inline constexpr std::int32_t kPaperBlobStreamPatchColorColor = 2;
inline constexpr std::int32_t kPaperBlobStreamPatchColorResidual = 3;

inline constexpr std::int32_t kPaperBlobStreamRDTagGlobalMeta = 0;
inline constexpr std::int32_t kPaperBlobStreamRDTagGlobalCombinedValenceIsPoly = 1;
inline constexpr std::int32_t kPaperBlobStreamRDTagGlobalValence = 1;
inline constexpr std::int32_t kPaperBlobStreamRDTagGlobalIsPoly = 2;
inline constexpr std::int32_t kPaperBlobStreamRDTagGlobalResidual = 2;
inline constexpr std::int32_t kPaperBlobStreamRDTagGlobalResidualWithIsPoly = 3;
inline constexpr std::int32_t kPaperBlobStreamRDTagValence = 0;
inline constexpr std::int32_t kPaperBlobStreamRDTagIsPoly = 1;
inline constexpr std::int32_t kPaperBlobStreamRDTagEdgeSym = 2;
inline constexpr std::int32_t kPaperBlobStreamRDTagResidual = 2;
inline constexpr std::int32_t kPaperBlobStreamRDTagResidualWithEdge = 3;
inline constexpr std::int32_t kPaperBlobStreamRDTagVertexAdjustment = 4;
inline constexpr std::int32_t kPaperBlobStreamRDTagVertexAdjustmentWithoutEdge = 3;
inline constexpr std::int32_t kPaperBlobStreamRDTagFrenetCorrection = 3;

inline constexpr std::int32_t kPaperBlobStreamVectorQuantizedTopology0 = 0;
inline constexpr std::int32_t kPaperBlobStreamVectorQuantizedTopology1 = 1;
inline constexpr std::int32_t kPaperBlobStreamVectorQuantizedIndices = 2;
inline constexpr std::int32_t kPaperBlobStreamVectorQuantizedCodebook = 3;

struct PaperConnectivityBlobSpec {
    std::int32_t blobLayout = kPaperBlobLayoutUnknown;
    std::int32_t schemeVersion = 0;
    std::int32_t roundCount = 0;
    std::int32_t streamCount = 0;
};

inline PaperConnectivityBlobSpec makePaperConnectivityBlobSpec(
    std::int32_t blobLayout,
    std::int32_t schemeVersion,
    std::int32_t roundCount,
    std::int32_t streamCount) {
    PaperConnectivityBlobSpec spec;
    spec.blobLayout = blobLayout;
    spec.schemeVersion = schemeVersion;
    spec.roundCount = roundCount;
    spec.streamCount = streamCount;
    return spec;
}

template <typename Header>
inline bool paperConnectivityBlobHeaderMatchesSpec(
    const Header& header,
    const PaperConnectivityBlobSpec& spec) {
    return header.blob_layout == static_cast<std::uint32_t>(spec.blobLayout) &&
           header.scheme_version == static_cast<std::uint32_t>(spec.schemeVersion) &&
           header.num_streams == static_cast<std::uint32_t>(spec.streamCount);
}

template <typename Header>
inline void requirePaperConnectivityBlobHeaderMatchesSpec(
    const Header& header,
    const PaperConnectivityBlobSpec& spec,
    const char* context) {
    const std::string prefix = std::string("[") + context + " decoder] ";
    if (header.blob_layout != static_cast<std::uint32_t>(spec.blobLayout)) {
        throw std::runtime_error(prefix + "expected blob_layout=" +
                                 std::to_string(spec.blobLayout) + " got " +
                                 std::to_string(header.blob_layout));
    }
    if (header.scheme_version != static_cast<std::uint32_t>(spec.schemeVersion)) {
        throw std::runtime_error(prefix + "expected scheme_version=" +
                                 std::to_string(spec.schemeVersion) + " got " +
                                 std::to_string(header.scheme_version));
    }
    if (header.num_streams != static_cast<std::uint32_t>(spec.streamCount)) {
        throw std::runtime_error(prefix + "expected num_streams=" +
                                 std::to_string(spec.streamCount) + " got " +
                                 std::to_string(header.num_streams));
    }
}

struct PaperConnectivityBlobFamilySpec {
    std::int32_t primaryBlobLayout = 0;
    std::int32_t secondaryBlobLayout = -1;
    std::int32_t schemeVersion = 0;
    std::int32_t roundCount = -1;
    std::int32_t minStreamCount = 0;
    std::int32_t maxStreamCount = 0;
};

inline bool paperConnectivityBlobFamilyLayoutMatches(
    std::uint32_t blobLayout,
    const PaperConnectivityBlobFamilySpec& spec) {
    return blobLayout == static_cast<std::uint32_t>(spec.primaryBlobLayout) ||
           (spec.secondaryBlobLayout >= 0 &&
            blobLayout == static_cast<std::uint32_t>(spec.secondaryBlobLayout));
}

template <typename Header>
inline bool paperConnectivityBlobHeaderMatchesFamilySpec(
    const Header& header,
    const PaperConnectivityBlobFamilySpec& spec) {
    const std::int32_t streamCount =
        static_cast<std::int32_t>(header.num_streams);
    return paperConnectivityBlobFamilyLayoutMatches(header.blob_layout, spec) &&
           header.scheme_version == static_cast<std::uint32_t>(spec.schemeVersion) &&
           (spec.roundCount < 0 ||
            header.num_rounds == static_cast<std::uint32_t>(spec.roundCount)) &&
           streamCount >= spec.minStreamCount &&
           streamCount <= spec.maxStreamCount;
}

template <typename Header>
inline void requirePaperConnectivityBlobHeaderMatchesFamilySpec(
    const Header& header,
    const PaperConnectivityBlobFamilySpec& spec,
    const char* context) {
    const std::string prefix = std::string("[") + context + " decoder] ";
    if (!paperConnectivityBlobFamilyLayoutMatches(header.blob_layout, spec)) {
        std::string expected = std::to_string(spec.primaryBlobLayout);
        if (spec.secondaryBlobLayout >= 0) {
            expected += " or " + std::to_string(spec.secondaryBlobLayout);
        }
        throw std::runtime_error(prefix + "expected blob_layout=" +
                                 expected + " got " +
                                 std::to_string(header.blob_layout));
    }
    if (header.scheme_version != static_cast<std::uint32_t>(spec.schemeVersion)) {
        throw std::runtime_error(prefix + "expected scheme_version=" +
                                 std::to_string(spec.schemeVersion) + " got " +
                                 std::to_string(header.scheme_version));
    }
    if (spec.roundCount >= 0 &&
        header.num_rounds != static_cast<std::uint32_t>(spec.roundCount)) {
        throw std::runtime_error(prefix + "expected num_rounds=" +
                                 std::to_string(spec.roundCount) + " got " +
                                 std::to_string(header.num_rounds));
    }
    const std::int32_t streamCount =
        static_cast<std::int32_t>(header.num_streams);
    if (streamCount < spec.minStreamCount ||
        streamCount > spec.maxStreamCount) {
        throw std::runtime_error(prefix + "expected num_streams=" +
                                 std::to_string(spec.minStreamCount) + ".." +
                                 std::to_string(spec.maxStreamCount) + " got " +
                                 std::to_string(header.num_streams));
    }
}

inline PaperConnectivityBlobSpec valencePatchOnlyConnectivityBlobSpec(
    std::int32_t roundCount) {
    return makePaperConnectivityBlobSpec(
        kPaperBlobLayoutValencePatchOnly,
        kPaperBlobSchemeVersionValencePatchOnly,
        roundCount,
        kPaperBlobStreamCountValencePatchOnly);
}





inline PaperConnectivityBlobSpec patchColorConnectivityBlobSpec(
    std::int32_t blobLayout,
    std::int32_t schemeVersion,
    std::int32_t roundCount) {
    return makePaperConnectivityBlobSpec(blobLayout,
                                         schemeVersion,
                                         roundCount,
                                         kPaperBlobStreamCountPatchColor);
}

inline PaperConnectivityBlobSpec vectorQuantizedConnectivityBlobSpec(
    std::int32_t blobLayout,
    std::int32_t schemeVersion,
    std::int32_t roundCount) {
    return makePaperConnectivityBlobSpec(
        blobLayout,
        schemeVersion,
        roundCount,
        kPaperBlobStreamCountVectorQuantizedBlob);
}




inline PaperConnectivityBlobSpec rdStrictConnectivityBlobSpec(
    std::int32_t roundCount) {
    return makePaperConnectivityBlobSpec(kPaperBlobLayoutRDStrict,
                                         kPaperBlobSchemeVersionRDStrict,
                                         roundCount,
                                         kPaperBlobStreamCountRDStrict);
}

inline PaperConnectivityBlobSpec rdTagConnectivityBlobSpec(
    std::int32_t roundCount,
    std::int32_t schemeVersion,
    std::int32_t streamCount) {
    return makePaperConnectivityBlobSpec(kPaperBlobLayoutRDTag,
                                         schemeVersion,
                                         roundCount,
                                         streamCount);
}

inline std::int32_t paperBlobLayoutLegacyFallback(bool enablePrediction) {
    return enablePrediction ? kPaperBlobLayoutRangeCodedPred
                            : kPaperBlobLayoutBasic;
}

inline bool paperBlobLayoutIsUnknown(std::int32_t layout) {
    return layout == kPaperBlobLayoutUnknown;
}

inline bool paperBlobLayoutIsRDStrict(std::int32_t layout) {
    return layout == kPaperBlobLayoutRDStrict;
}





inline bool paperBlobLayoutIsSegmentedRangeCodedPred(std::int32_t layout) {
    return layout == kPaperBlobLayoutSegmentedRangeCodedPred;
}


inline bool paperBlobLayoutUsesSeedSnapshotBase(std::int32_t layout) {
    return layout == kPaperBlobLayoutCodec003 ||
           layout == kPaperBlobLayoutCodec026;
}

inline bool paperBlobLayoutUsesDecodedQueues(std::int32_t layout) {
    switch (layout) {
        case kPaperBlobLayoutRangeCodedPred:
        case kPaperBlobLayoutValenceRC:
        case kPaperBlobLayoutCodec006:
        case kPaperBlobLayoutRDGate:
        case kPaperBlobLayoutRDGateOnly:
        case kPaperBlobLayoutValencePatchOnly:
        case kPaperBlobLayoutConquestRC:
        case kPaperBlobLayoutCodec014:
        case kPaperBlobLayoutCodec016:
        case kPaperBlobLayoutCodec017:
        case kPaperBlobLayoutRDTag:
        case kPaperBlobLayoutReserved19:
        case kPaperBlobLayoutCodec020:
        case kPaperBlobLayoutCodec021:
        case kPaperBlobLayoutCodec022:
        case kPaperBlobLayoutCodec023:
        case kPaperBlobLayoutCodec025:
        case kPaperBlobLayoutCodec026:
        case kPaperBlobLayoutCodec027:
        case kPaperBlobLayoutCodec028:
        case kPaperBlobLayoutRDStrict:
        case kPaperBlobLayoutCanonical:
            return true;
        default:
            return false;
    }
}





inline bool paperBlobSchemeVersionIsRDTag(std::int32_t schemeVersion) {
    switch (schemeVersion) {
        case kPaperBlobSchemeVersionRDTagV1:
        case kPaperBlobSchemeVersionRDTagV2:
        case kPaperBlobSchemeVersionRDTagV3:
        case kPaperBlobSchemeVersionRDTagV4:
        case kPaperBlobSchemeVersionRDTagV5:
        case kPaperBlobSchemeVersionRDTagV6:
        case kPaperBlobSchemeVersionRDTagV7:
        case kPaperBlobSchemeVersionRDTagV8:
        case kPaperBlobSchemeVersionRDTagV9:
        case kPaperBlobSchemeVersionRDTagV10:
        case kPaperBlobSchemeVersionRDTagV11:
        case kPaperBlobSchemeVersionRDTagV12:
        case kPaperBlobSchemeVersionRDTagV13:
        case kPaperBlobSchemeVersionRDTagV14:
        case kPaperBlobSchemeVersionRDTagV15:
        case kPaperBlobSchemeVersionRDTagV16:
        case kPaperBlobSchemeVersionRDTagV17:
        case kPaperBlobSchemeVersionRDTagV18:
        case kPaperBlobSchemeVersionRDTagV6Ctx:
            return true;
        default:
            return false;
    }
}

inline bool paperBlobSchemeVersionRDTagHasAdjustmentStream(
    std::int32_t schemeVersion) {
    return schemeVersion == kPaperBlobSchemeVersionRDTagV5 ||
           paperBlobSchemeVersionRDTagIsV6Family(schemeVersion);
}

inline bool paperBlobSchemeVersionRDTagUsesEdgeSymTopology(
    std::int32_t schemeVersion) {
    return paperBlobSchemeVersionRDTagIsV6Family(schemeVersion);
}

inline bool paperBlobSchemeVersionRDTagUsesAuxiliaryFourthStream(
    std::int32_t schemeVersion) {
    switch (schemeVersion) {
        case kPaperBlobSchemeVersionRDTagV8:
        case kPaperBlobSchemeVersionRDTagV9:
        case kPaperBlobSchemeVersionRDTagV10:
        case kPaperBlobSchemeVersionRDTagV11:
        case kPaperBlobSchemeVersionRDTagV12:
        case kPaperBlobSchemeVersionRDTagV13:
        case kPaperBlobSchemeVersionRDTagV14:
        case kPaperBlobSchemeVersionRDTagV15:
        case kPaperBlobSchemeVersionRDTagV17:
        case kPaperBlobSchemeVersionRDTagV18:
            return true;
        default:
            return false;
    }
}

inline std::int32_t paperBlobStreamCountRDTagForScheme(
    std::int32_t schemeVersion) {
    if (paperBlobSchemeVersionRDTagUsesEdgeSymTopology(schemeVersion)) {
        return kPaperBlobStreamCountRDTagWithAdjustment;
    }
    if (paperBlobSchemeVersionRDTagHasAdjustmentStream(schemeVersion) ||
        paperBlobSchemeVersionRDTagUsesAuxiliaryFourthStream(schemeVersion)) {
        return kPaperBlobStreamCountRDTagWithFrenet;
    }
    return kPaperBlobStreamCountRDTagBase;
}

inline bool paperBlobSchemeVersionRDTagUsesFrenetResidual(
    std::int32_t schemeVersion) {
    switch (schemeVersion) {
        case kPaperBlobSchemeVersionRDTagV7:
        case kPaperBlobSchemeVersionRDTagV8:
        case kPaperBlobSchemeVersionRDTagV10:
        case kPaperBlobSchemeVersionRDTagV12:
        case kPaperBlobSchemeVersionRDTagV14:
            return true;
        default:
            return false;
    }
}

inline bool paperBlobSchemeVersionRDTagUsesFrenetExactResidual(
    std::int32_t schemeVersion) {
    return schemeVersion == kPaperBlobSchemeVersionRDTagV8;
}

inline bool paperBlobSchemeVersionRDTagUsesGlobalStreamMetadata(
    std::int32_t schemeVersion) {
    switch (schemeVersion) {
        case kPaperBlobSchemeVersionRDTagV9:
        case kPaperBlobSchemeVersionRDTagV10:
        case kPaperBlobSchemeVersionRDTagV11:
        case kPaperBlobSchemeVersionRDTagV12:
        case kPaperBlobSchemeVersionRDTagV13:
        case kPaperBlobSchemeVersionRDTagV14:
        case kPaperBlobSchemeVersionRDTagV15:
        case kPaperBlobSchemeVersionRDTagV16:
        case kPaperBlobSchemeVersionRDTagV17:
        case kPaperBlobSchemeVersionRDTagV18:
            return true;
        default:
            return false;
    }
}

inline bool paperBlobSchemeVersionRDTagUsesPackedGlobalFaceCounts(
    std::int32_t schemeVersion) {
    switch (schemeVersion) {
        case kPaperBlobSchemeVersionRDTagV13:
        case kPaperBlobSchemeVersionRDTagV14:
        case kPaperBlobSchemeVersionRDTagV15:
        case kPaperBlobSchemeVersionRDTagV16:
        case kPaperBlobSchemeVersionRDTagV17:
        case kPaperBlobSchemeVersionRDTagV18:
            return true;
        default:
            return false;
    }
}

inline bool paperBlobSchemeVersionRDTagUsesCombinedValenceIsPolyStream(
    std::int32_t schemeVersion) {
    return schemeVersion == kPaperBlobSchemeVersionRDTagV16;
}

inline bool paperBlobSchemeVersionRDTagUsesRoundStaticIsPoly(
    std::int32_t schemeVersion) {
    return schemeVersion == kPaperBlobSchemeVersionRDTagV17;
}

inline bool paperBlobSchemeVersionRDTagUsesStaticGlobalIsPoly(
    std::int32_t schemeVersion) {
    return schemeVersion == kPaperBlobSchemeVersionRDTagV11 ||
           schemeVersion == kPaperBlobSchemeVersionRDTagV12;
}

inline bool paperBlobSchemeVersionRDTagUsesHistoryContextGlobalIsPoly(
    std::int32_t schemeVersion) {
    return schemeVersion == kPaperBlobSchemeVersionRDTagV15 ||
           schemeVersion == kPaperBlobSchemeVersionRDTagV18;
}

inline bool paperBlobSchemeVersionRDTagUsesPackedGlobalValence(
    std::int32_t schemeVersion) {
    switch (schemeVersion) {
        case kPaperBlobSchemeVersionRDTagV13:
        case kPaperBlobSchemeVersionRDTagV14:
        case kPaperBlobSchemeVersionRDTagV15:
        case kPaperBlobSchemeVersionRDTagV17:
        case kPaperBlobSchemeVersionRDTagV18:
            return true;
        default:
            return false;
    }
}

inline bool paperBlobSchemeVersionRDTagUsesRoundBucketGlobalResidual(
    std::int32_t schemeVersion) {
    return schemeVersion == kPaperBlobSchemeVersionRDTagV18;
}

inline std::int32_t paperBlobStreamRDTagGlobalResidualForScheme(
    std::int32_t schemeVersion) {
    return paperBlobSchemeVersionRDTagUsesCombinedValenceIsPolyStream(
               schemeVersion)
        ? kPaperBlobStreamRDTagGlobalResidual
        : kPaperBlobStreamRDTagGlobalResidualWithIsPoly;
}

inline bool paperBlobSchemeVersionRDTagUsesLocalSingleContextIsPoly(
    std::int32_t schemeVersion) {
    switch (schemeVersion) {
        case kPaperBlobSchemeVersionRDTagV1:
        case kPaperBlobSchemeVersionRDTagV3:
        case kPaperBlobSchemeVersionRDTagV4:
        case kPaperBlobSchemeVersionRDTagV5:
        case kPaperBlobSchemeVersionRDTagV6:
        case kPaperBlobSchemeVersionRDTagV6Ctx:
        case kPaperBlobSchemeVersionRDTagV7:
        case kPaperBlobSchemeVersionRDTagV8:
            return true;
        default:
            return false;
    }
}

inline std::int32_t paperBlobStreamRDTagLocalResidualForScheme(
    std::int32_t schemeVersion) {
    return paperBlobSchemeVersionRDTagUsesEdgeSymTopology(schemeVersion)
        ? kPaperBlobStreamRDTagResidualWithEdge
        : kPaperBlobStreamRDTagResidual;
}

inline bool paperBlobSchemeVersionRDTagUsesAdaptiveLocalResidual(
    std::int32_t schemeVersion) {
    switch (schemeVersion) {
        case kPaperBlobSchemeVersionRDTagV3:
        case kPaperBlobSchemeVersionRDTagV4:
        case kPaperBlobSchemeVersionRDTagV5:
        case kPaperBlobSchemeVersionRDTagV6:
        case kPaperBlobSchemeVersionRDTagV6Ctx:
        case kPaperBlobSchemeVersionRDTagV7:
        case kPaperBlobSchemeVersionRDTagV8:
            return true;
        default:
            return false;
    }
}

inline bool paperBlobSchemeVersionRDTagRequiresZeroAdaptiveResidualDropByte(
    std::int32_t schemeVersion) {
    return schemeVersion == kPaperBlobSchemeVersionRDTagV3;
}

inline bool paperBlobSchemeVersionRDTagUsesFrenetCorrectionStream(
    std::int32_t schemeVersion) {
    return schemeVersion == kPaperBlobSchemeVersionRDTagV8;
}

inline std::int32_t paperBlobStreamRDTagAdjustmentForScheme(
    std::int32_t schemeVersion) {
    return paperBlobSchemeVersionRDTagUsesEdgeSymTopology(schemeVersion)
        ? kPaperBlobStreamRDTagVertexAdjustment
        : kPaperBlobStreamRDTagVertexAdjustmentWithoutEdge;
}



inline bool paperBlobSchemeVersionIsVectorQuantized(
    std::int32_t schemeVersion) {
    switch (schemeVersion) {
        case kPaperBlobSchemeVersionVectorQuantizedV1:
        case kPaperBlobSchemeVersionVectorQuantizedV2:
        case kPaperBlobSchemeVersionVectorQuantizedV3:
        case kPaperBlobSchemeVersionVectorQuantizedV5:
        case kPaperBlobSchemeVersionVectorQuantizedV6:
            return true;
        default:
            return false;
    }
}

inline bool paperBlobSchemeVersionVectorQuantizedUsesFacetEdgeStreams(
    std::int32_t schemeVersion) {
    return schemeVersion == kPaperBlobSchemeVersionVectorQuantizedV1 ||
           schemeVersion == kPaperBlobSchemeVersionVectorQuantizedV5 ||
           schemeVersion == kPaperBlobSchemeVersionVectorQuantizedV6;
}

inline bool paperBlobSchemeVersionVectorQuantizedUsesFaceCodeTopology(
    std::int32_t schemeVersion) {
    return schemeVersion == kPaperBlobSchemeVersionVectorQuantizedV2 ||
           schemeVersion == kPaperBlobSchemeVersionVectorQuantizedV3;
}

inline bool paperBlobSchemeVersionVectorQuantizedUsesHybridEdgeStream(
    std::int32_t schemeVersion) {
    return schemeVersion == kPaperBlobSchemeVersionVectorQuantizedV5;
}

inline bool paperBlobSchemeVersionVectorQuantizedUsesSplitTopologyStream(
    std::int32_t schemeVersion) {
    return schemeVersion == kPaperBlobSchemeVersionVectorQuantizedV6;
}

inline bool paperBlobSchemeVersionVectorQuantizedUsesContextLocalEdges(
    std::int32_t schemeVersion) {
    return schemeVersion == kPaperBlobSchemeVersionVectorQuantizedV3;
}





















}  // namespace pmsf

#endif  // PMSF_STORAGE_PAPER_BLOB_LAYOUT_REGISTRY_H
