#ifndef PMSF_STORAGE_POMAR_PAPER_PAYLOAD_ADAPTER_H
#define PMSF_STORAGE_POMAR_PAPER_PAYLOAD_ADAPTER_H

#include "common/PMSFLegacyOperationSymbolAdapter.h"
#include "common/PMSFStorageScopeNames.h"
#include "common/PaperLayoutAliases.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂
#include "storage/PMSFPayloadDescriptorRegistry.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace pmsf {

inline constexpr const char* kPomarBaseMeshPayloadName = "pomar_base_mesh";
inline constexpr const char* kPomarCluster0PayloadName = "pomar_cluster0";
inline constexpr const char* kPomarMetaPayloadName = "pomar_meta";
inline constexpr const char* kPomarRestPayloadName = "pomar_rest";
inline constexpr const char* kPomarV2PayloadCodecName = "pomar.v2";
inline constexpr const char* kPomarV3PayloadCodecName = "pomar.v3";
inline constexpr const char* kPomarCompatPayloadWrapperFormat =
    "POMAR_COLUMNAR_INNER_PAYLOAD";
inline constexpr std::int32_t kPomarCompatPayloadWrapperCodecVersion = 2;
inline constexpr std::int32_t kPomarCompatFrenetBlobLayout =
    kPaperBlobLayoutPomarCompatFrenet;
inline constexpr std::int32_t kPomarCompatDirectResidualBlobLayout =
    kPaperBlobLayoutPomarCompatDirectResidual;
inline constexpr const char* kPomarNumRoundsColumn = "num_rounds";
inline constexpr const char* kPomarEnableCompressColumn = "enable_compress";
inline constexpr const char* kPomarEnableQuantizationColumn = "enable_quantization";
inline constexpr const char* kPomarEnableSegmentationColumn = "enable_segmentation";
inline constexpr const char* kPomarEnableAdjustmentColumn = "enable_adjustment";
inline constexpr const char* kPomarEnablePredictionColumn = "enable_prediction";
inline constexpr const char* kPomarNumClustersColumn = "num_clusters";
inline constexpr const char* kPomarNbGlobalLODColumn = "nb_global_lod";
inline constexpr const char* kPomarNbClusteredLODColumn = "nb_clustered_lod";
inline constexpr const char* kPomarBaseMeshSizeColumn = "base_mesh_size";
inline constexpr const char* kPomarCluster0SizeColumn = "cluster0_size";
inline constexpr const char* kPomarClusterMetaSizeColumn = "cluster_meta_size";
inline constexpr const char* kPomarClustersRestSizeColumn = "clusters_rest_size";
inline constexpr const char* kPomarBaseMeshBlobColumn = "base_mesh_blob";
inline constexpr const char* kPomarCluster0RCBlobColumn = "cluster0_rc_blob";
inline constexpr const char* kPomarClusterMetaBlobColumn = "cluster_meta_blob";
inline constexpr const char* kPomarClustersBlobColumn = "clusters_blob";
inline constexpr const char* kPomarCompatRCBlobColumn = "rc_blob";
inline constexpr const char* kPomarRandomAccessColumn = "random_access";
inline constexpr const char* kPomarClusterIdColumn = "cluster_id";
inline constexpr const char* kPomarResidualModeSidePrefix = "pomar";
inline constexpr const char* kPomarDirectResidualSideName = "direct_residual";
inline constexpr const char* kPomarFrenetResidualSideName = "frenet_residual";
inline constexpr const char* kPomarTopoCandidateSideName = "topo_candidate";

inline bool pomarSkipCompatColumnarTables() {
    return pmsfStorageEnvEnabled("PMSF_POMAR_SKIP_COMPAT_COLUMNAR_TABLES");
}

inline std::int32_t pomarCompatBlobLayout(bool directResidual) {
    return directResidual ? kPomarCompatDirectResidualBlobLayout
                          : kPomarCompatFrenetBlobLayout;
}

inline bool pomarCompatBlobLayoutUsesDirectResidual(std::int32_t blobLayout) {
    return paperBlobLayoutIsPomarCompatDirectResidual(blobLayout);
}

template <typename Fn>
inline void forEachPomarV2ResidualModeField(bool directResidual,
                                           bool topoCandidate,
                                           Fn&& fn) {
    fn(kPomarDirectResidualSideName,
       static_cast<std::int32_t>(directResidual ? 1 : 0));
    fn(kPomarFrenetResidualSideName,
       static_cast<std::int32_t>(directResidual ? 0 : 1));
    fn(kPomarTopoCandidateSideName,
       static_cast<std::int32_t>(topoCandidate ? 1 : 0));
}

inline void appendPomarLegacyStorageSideChannel(PaperRoundSymbols& delta) {
    appendPMSFStorageContextMapping(delta, kPMSFStorageScopePomarLegacy, 1, 0);
    appendPaperI32SideChannel(
        delta, kPMSFStorageScopePomarLegacy, "prefer_v2_collector", 1);
    appendPMSFBoundaryAbsentExactPair(
        delta,
        kPMSFStorageScopePomarLegacy,
        "paper",
        legacyOperationPaperExactRequired(kPMSFStorageScopePomarLegacy) ? 1 : 0);
}

inline void appendPomarLegacyStorageSideMap(PaperRoundSymbols& delta) {
    appendPomarLegacyStorageSideChannel(delta);
}

template <typename SideMap>
inline void appendPomarV2ResidualModeSideMap(SideMap& side,
                                             bool directResidual,
                                             bool topoCandidate) {
    forEachPomarV2ResidualModeField(
        directResidual,
        topoCandidate,
        [&](const char* suffix, std::int32_t value) {
            appendPaperI32SideMap(
                side, kPomarResidualModeSidePrefix, suffix, value);
        });
}

inline void appendPomarV2ResidualMode(PaperRoundSymbols& delta,
                                      bool directResidual,
                                      bool topoCandidate) {
    forEachPomarV2ResidualModeField(
        directResidual,
        topoCandidate,
        [&](const char* suffix, std::int32_t value) {
            appendPaperI32SideChannel(
                delta, kPomarResidualModeSidePrefix, suffix, value);
        });
}

inline std::string pomarCluster0TablePath(const std::string& stem) {
    return paperStorageArtifactPath(stem, kPaperCompatCluster0ParquetSuffix);
}

inline std::string pomarClustersTablePath(const std::string& stem) {
    return paperStorageArtifactPath(stem, kPaperCompatClustersParquetSuffix);
}

inline std::string pomarCompatMainTablePath(const std::string& stem) {
    return stem + ".parquet";
}

struct PomarPayloadSpan {
    const char* payloadName = "";
    const char* data = nullptr;
    std::size_t size = 0;
};

struct PomarPayloadStringParts {
    std::string baseMesh;
    std::string cluster0;
    std::string meta;
    std::string rest;
};

template <typename ReadPayload>
inline PomarPayloadStringParts readPomarPayloadStringParts(ReadPayload&& readPayload) {
    return {readPayload(kPomarBaseMeshPayloadName),
            readPayload(kPomarCluster0PayloadName),
            readPayload(kPomarMetaPayloadName),
            readPayload(kPomarRestPayloadName)};
}

inline bool pomarPayloadStringPartsHaveRequired(
    const PomarPayloadStringParts& parts) {
    return !parts.baseMesh.empty() &&
           !parts.cluster0.empty() &&
           !parts.meta.empty();
}

inline std::size_t pomarPayloadStringPartsSize(
    const PomarPayloadStringParts& parts) {
    return parts.baseMesh.size() + parts.cluster0.size() +
           parts.meta.size() + parts.rest.size();
}

inline std::vector<PomarPayloadSpan> pomarPayloadSpans(
    const char* buffer,
    std::size_t baseEnd,
    std::size_t cluster0End,
    std::size_t metaEnd,
    std::size_t bufferSize) {
    return {
        {kPomarBaseMeshPayloadName, buffer, baseEnd},
        {kPomarCluster0PayloadName, buffer + baseEnd, cluster0End - baseEnd},
        {kPomarMetaPayloadName, buffer + cluster0End, metaEnd - cluster0End},
        {kPomarRestPayloadName, buffer + metaEnd, bufferSize - metaEnd},
    };
}

inline PMSFPayloadDescriptor pomarPayloadDescriptor(const char* payloadName,
                                                     const char* codecName);

inline PMSFSymbolPayloadSpec pomarPayloadSpec(const char* payloadName,
                                               const char* codecName) {
    return paperPayloadSpecFromDescriptor(
        pomarPayloadDescriptor(payloadName, codecName));
}

inline PMSFPayloadDescriptor pomarPayloadDescriptor(const char* payloadName,
                                                     const char* codecName) {
    return makePMSFPayloadDescriptorWithInferredRole(
        payloadName == nullptr ? "" : payloadName,
        codecName == nullptr ? "" : codecName);
}

inline PMSFSymbolPayloadTable buildPomarPayloadTable(
    const char* codecName,
    const char* buffer,
    std::size_t baseEnd,
    std::size_t cluster0End,
    std::size_t metaEnd,
    std::size_t bufferSize) {
    PMSFSymbolPayloadTable rows;
    for (const auto& span : pomarPayloadSpans(
             buffer, baseEnd, cluster0End, metaEnd, bufferSize)) {
        appendPMSFPayloadRecordFromDescriptor(
            rows,
            pomarPayloadDescriptor(span.payloadName, codecName),
            span.data,
            span.size);
    }
    return rows;
}

}  // namespace pmsf

#endif  // PMSF_STORAGE_POMAR_PAPER_PAYLOAD_ADAPTER_H
