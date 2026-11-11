/*
 * @file PMSFSymbolStorageAdapter.h
 * @brief Storage-facing adapter for the paper symbol contract.
 *
 * The paper defines each round as:
 *   Delta_r = (F_r, E_r^t, E_r^f, V_r, C_r^f, C_r^v)
 *
 * Existing PMSF byte streams predate that notation and use RoundSymbols with
 * one edge channel. This adapter is the narrow compatibility layer between
 * the paper-level contract and the current Parquet V3 writer. The paper's
 * common flag streams remain representable as 0/1 values, while the physical
 * F/E/V columns are int32 alphabets so codec-specific vertex/patch symbols do
 * not force new algorithm-specific schemas or get truncated to bool.
 */
#ifndef PMSF_STORAGE_PAPER_SYMBOL_STORAGE_ADAPTER_H
#define PMSF_STORAGE_PAPER_SYMBOL_STORAGE_ADAPTER_H

#include "core/PMSFSideChannel.h"
#include "core/Symbol.h"
#include "storage/PMSFStorageContextSideChannel.h"

#include <algorithm>
#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace pmsf {

inline constexpr const char* kPMSFSymbolMappingMetadataKey =
    "pmsf_paper_symbol_mapping";
inline constexpr const char* kPMSFSymbolTableMetadataKey =
    "pmsf_paper_symbol_table";
inline constexpr const char* kPMSFSideChannelsMetadataKey =
    "pmsf_side_channels";
inline constexpr const char* kPMSFVersionMetadataKey =
    "pmsf_version";
inline constexpr const char* kPMSFFormatVersionMetadataKey =
    "pmsf_format_version";
inline constexpr const char* kPMSFPayloadTableMetadataKey =
    "pmsf_paper_payload_table";
inline constexpr const char* kPMSFPayloadBlobNameColumn =
    "blob_name";
inline constexpr const char* kPMSFPayloadBlobKindColumn =
    "blob_kind";
inline constexpr const char* kPMSFPayloadBlobBytesColumn =
    "blob_bytes";
inline constexpr const char* kPayloadWrapperFormatColumn =
    "format";
inline constexpr const char* kPayloadWrapperCodecVersionColumn =
    "codec_version";
inline constexpr const char* kPayloadWrapperBlobLayoutColumn =
    "blob_layout";
inline constexpr const char* kPayloadWrapperPayloadBytesColumn =
    "payload_bytes";
inline constexpr const char* kPayloadWrapperPayloadColumn =
    "payload";
inline constexpr std::int32_t kPMSFPayloadWrapperNoBlobLayout = 0;
inline constexpr const char* kParquetGroupIdColumn =
    "group_id";
inline constexpr const char* kParquetRoundIdColumn =
    "round_id";
inline constexpr const char* kParquetRCBlobColumn =
    "rc_blob";
inline constexpr const char* kParquetBaseModeColumn =
    "base_mode";
inline constexpr const char* kParquetBaseBlobColumn =
    "base_blob";
inline constexpr const char* kParquetMetadataBlobColumn =
    "metadata_blob";
inline constexpr const char* kParquetSymbolsBlobColumn =
    "symbols_blob";
inline constexpr const char* kParquetBoundaryOwnerBlobColumn =
    "boundary_owner_blob";
inline constexpr const char* kParquetNumRoundsColumn =
    "num_rounds";
inline constexpr const char* kParquetGroupNumberColumn =
    "group_number";
inline constexpr const char* kParquetEnableCompressColumn =
    "enable_compress";
inline constexpr const char* kParquetEnableQuantizationColumn =
    "enable_quantization";
inline constexpr const char* kParquetEnableSegmentationColumn =
    "enable_segmentation";
inline constexpr const char* kParquetEnableAdjustmentColumn =
    "enable_adjustment";
inline constexpr const char* kParquetQuantBitsColumn =
    "quant_bits";
inline constexpr const char* kParquetBBoxMinXColumn =
    "bbox_min_x";
inline constexpr const char* kParquetBBoxMinYColumn =
    "bbox_min_y";
inline constexpr const char* kParquetBBoxMinZColumn =
    "bbox_min_z";
inline constexpr const char* kParquetQuantStepColumn =
    "quant_step";
inline constexpr const char* kParquetTotalVerticesColumn =
    "total_vertices";
inline constexpr const char* kParquetTotalFacesColumn =
    "total_faces";
inline constexpr const char* kParquetCodecVersionColumn =
    "codec_version";
inline constexpr const char* kParquetEnablePredictionColumn =
    "enable_prediction";
inline constexpr const char* kParquetBlobLayoutColumn =
    "blob_layout";
// When blob_layout == kPaperBlobLayoutCanonical, this column stores the
// original algorithm-native layout that produced the canonical symbol payload.
// Zero/null means "canonical is Class-I / no special reassembly needed".
inline constexpr const char* kParquetCanonicalSourceLayoutColumn =
    "canonical_source_layout";
inline constexpr const char* kParquetEnableTriangulationColumn =
    "enable_triangulation";
inline constexpr const char* kParquetEnableConnPredFacesColumn =
    "enable_conn_pred_faces";
inline constexpr const char* kParquetEnableConnPredEdgesColumn =
    "enable_conn_pred_edges";
inline constexpr const char* kParquetEnableTriBalanceColumn =
    "enable_tri_balance";
inline constexpr const char* kParquetNumFacetSymbolsColumn =
    "num_facet_symbols";
inline constexpr const char* kParquetNumEdgeSymbolsColumn =
    "num_edge_symbols";
inline constexpr const char* kParquetNumVertexSymbolsColumn =
    "num_vertex_symbols";
inline constexpr const char* kParquetNumInsertedPointsColumn =
    "num_inserted_points";
inline constexpr const char* kParquetNumVertexPointsColumn =
    "num_vertex_points";
inline constexpr const char* kParquetFacetSymbolsColumn =
    "facet_symbols";
inline constexpr const char* kParquetEdgeSymbolsColumn =
    "edge_symbols";
inline constexpr const char* kParquetVertexSymbolsColumn =
    "vertex_symbols";
inline constexpr const char* kParquetPointXColumn =
    "point_x";
inline constexpr const char* kParquetPointYColumn =
    "point_y";
inline constexpr const char* kParquetPointZColumn =
    "point_z";
inline constexpr const char* kParquetVertexPointXColumn =
    "vertex_point_x";
inline constexpr const char* kParquetVertexPointYColumn =
    "vertex_point_y";
inline constexpr const char* kParquetVertexPointZColumn =
    "vertex_point_z";
inline constexpr const char* kParquetPointQXColumn =
    "point_qx";
inline constexpr const char* kParquetPointQYColumn =
    "point_qy";
inline constexpr const char* kParquetPointQZColumn =
    "point_qz";
inline constexpr const char* kParquetVertexPointQXColumn =
    "vertex_point_qx";
inline constexpr const char* kParquetVertexPointQYColumn =
    "vertex_point_qy";
inline constexpr const char* kParquetVertexPointQZColumn =
    "vertex_point_qz";
inline constexpr const char* kParquetNumVerticesColumn =
    "num_vertices";
inline constexpr const char* kParquetNumFacesColumn =
    "num_faces";
inline constexpr const char* kParquetSeedV0CoordIdColumn =
    "seed_v0_coord_id";
inline constexpr const char* kParquetSeedV1CoordIdColumn =
    "seed_v1_coord_id";
inline constexpr const char* kParquetVertexCoordIdsColumn =
    "vertex_coord_ids";
inline constexpr const char* kParquetFaceVerticesAreCoordIdsColumn =
    "face_vertices_are_coord_ids";
inline constexpr const char* kParquetSharedVertexQXColumn =
    "shared_vertex_qx";
inline constexpr const char* kParquetSharedVertexQYColumn =
    "shared_vertex_qy";
inline constexpr const char* kParquetSharedVertexQZColumn =
    "shared_vertex_qz";
inline constexpr const char* kParquetSharedVertexXColumn =
    "shared_vertex_x";
inline constexpr const char* kParquetSharedVertexYColumn =
    "shared_vertex_y";
inline constexpr const char* kParquetSharedVertexZColumn =
    "shared_vertex_z";
inline constexpr const char* kParquetFaceVerticesColumn =
    "face_vertices";
inline constexpr const char* kParquetFaceDegreesColumn =
    "face_degrees";
inline constexpr const char* kParquetSeedV0QXColumn =
    "seed_v0_qx";
inline constexpr const char* kParquetSeedV0QYColumn =
    "seed_v0_qy";
inline constexpr const char* kParquetSeedV0QZColumn =
    "seed_v0_qz";
inline constexpr const char* kParquetSeedV1QXColumn =
    "seed_v1_qx";
inline constexpr const char* kParquetSeedV1QYColumn =
    "seed_v1_qy";
inline constexpr const char* kParquetSeedV1QZColumn =
    "seed_v1_qz";
inline constexpr const char* kParquetVertexQXColumn =
    "vertex_qx";
inline constexpr const char* kParquetVertexQYColumn =
    "vertex_qy";
inline constexpr const char* kParquetVertexQZColumn =
    "vertex_qz";
inline constexpr const char* kParquetSeedV0XColumn =
    "seed_v0_x";
inline constexpr const char* kParquetSeedV0YColumn =
    "seed_v0_y";
inline constexpr const char* kParquetSeedV0ZColumn =
    "seed_v0_z";
inline constexpr const char* kParquetSeedV1XColumn =
    "seed_v1_x";
inline constexpr const char* kParquetSeedV1YColumn =
    "seed_v1_y";
inline constexpr const char* kParquetSeedV1ZColumn =
    "seed_v1_z";
inline constexpr const char* kParquetVertexXColumn =
    "vertex_x";
inline constexpr const char* kParquetVertexYColumn =
    "vertex_y";
inline constexpr const char* kParquetVertexZColumn =
    "vertex_z";
inline constexpr const char* kParquetGroupAColumn =
    "group_a";
inline constexpr const char* kParquetGroupBColumn =
    "group_b";
inline constexpr const char* kParquetBoundaryStColumn =
    "boundary_st";
inline constexpr const char* kParquetBoundaryEdColumn =
    "boundary_ed";
inline constexpr const char* kParquetBoundaryStopColumn =
    "boundary_stop";
// Column names for codec=4 columnar segment tables (Connectivity / Geometry / LOD).
inline constexpr const char* kParquetBitDataColumn = "bit_data";
inline constexpr const char* kParquetResidualDataColumn = "residual_data";
inline constexpr const char* kParquetStreamIdColumn = "stream_id";
inline constexpr const char* kParquetEncodingModeColumn = "encoding_mode";
inline constexpr const char* kParquetBitStartColumn = "bit_start";
inline constexpr const char* kParquetBitLenColumn = "bit_len";
inline constexpr const char* kParquetResidualStartColumn = "residual_start";
inline constexpr const char* kParquetResidualLenColumn = "residual_len";
inline constexpr const char* kParquetBlockIdColumn =
    "block_id";
inline constexpr const char* kParquetFirstCoordIdColumn =
    "first_coord_id";
inline constexpr const char* kParquetCountColumn =
    "count";
inline constexpr const char* kParquetCoordBlobColumn =
    "coord_blob";
inline constexpr const char* kParquetNumBaseVerticesColumn =
    "num_base_vertices";
inline constexpr const char* kParquetNumBaseFacesColumn =
    "num_base_faces";
inline constexpr const char* kParquetVertexInlinePositionsColumn =
    "vertex_inline_positions";
inline constexpr const char* kParquetVertexSharedPositionsColumn =
    "vertex_shared_positions";
inline constexpr const char* kParquetFaceV0Column =
    "face_v0";
inline constexpr const char* kParquetFaceV1Column =
    "face_v1";
inline constexpr const char* kParquetFaceV2Column =
    "face_v2";

inline bool pmsfStorageEnvEnabled(const char* name) {
    const char* value = std::getenv(name);
    return value != nullptr && value[0] != '\0' && value[0] != '0';
}

struct PaperNamedBlob {
    std::string name;
    std::string kind;
    std::vector<std::uint8_t> bytes;
};

struct PMSFPayloadWrapperDescriptor {
    std::string format;
    std::int32_t codecVersion = 0;
    std::int32_t blobLayout = 0;
    std::int32_t numRounds = 0;
    bool enableCompress = false;
    bool enableQuantization = false;
    bool enableSegmentation = false;
    bool enableAdjustment = false;
    std::int64_t payloadBytes = 0;
};

inline std::int64_t paperPayloadWrapperPayloadBytes(std::size_t payloadSize) {
    return static_cast<std::int64_t>(payloadSize);
}

inline PMSFPayloadWrapperDescriptor makePMSFPayloadWrapperDescriptor(
    std::string format,
    std::int32_t codecVersion,
    std::int32_t blobLayout,
    std::int32_t numRounds,
    bool enableCompress,
    bool enableQuantization,
    bool enableSegmentation,
    bool enableAdjustment,
    std::size_t payloadSize) {
    PMSFPayloadWrapperDescriptor descriptor;
    descriptor.format = std::move(format);
    descriptor.codecVersion = codecVersion;
    descriptor.blobLayout = blobLayout;
    descriptor.numRounds = numRounds;
    descriptor.enableCompress = enableCompress;
    descriptor.enableQuantization = enableQuantization;
    descriptor.enableSegmentation = enableSegmentation;
    descriptor.enableAdjustment = enableAdjustment;
    descriptor.payloadBytes = paperPayloadWrapperPayloadBytes(payloadSize);
    return descriptor;
}

inline constexpr const char* kPaperEdgeFlipSideChannel = "paper_edge_flip";
inline constexpr const char* kPaperObjectSideChannel = "paper_object_id";
inline constexpr const char* kPaperPartitionSideChannel = "paper_partition_id";
inline constexpr const char* kPaperLodSideChannel = "paper_lod_id";
inline constexpr const char* kPaperFacetSymbolI32SideChannel =
    paperSymbolRoleSideChannelName(PMSFSymbolRole::Facet);
inline constexpr const char* kPaperTriangulationEdgeI32SideChannel =
    paperSymbolRoleSideChannelName(PMSFSymbolRole::TriangulationEdge);
inline constexpr const char* kPaperFlipEdgeI32SideChannel =
    paperSymbolRoleSideChannelName(PMSFSymbolRole::FlipEdge);
inline constexpr const char* kPaperVertexSymbolI32SideChannel =
    paperSymbolRoleSideChannelName(PMSFSymbolRole::Vertex);
inline constexpr const char* kPMSFPayloadBlobPrefix = "paper_payload_obj";
inline constexpr const char* kPaperRelationModelVersionSideChannel =
    "paper_relation_model_version";
inline constexpr const char* kPaperDeltaTupleAritySideChannel =
    "paper_delta_tuple_arity";
inline constexpr const char* kLegacySingleFileBinaryPayloadName =
    "pmsf_legacy_binary_buffer";
inline constexpr const char* kLegacySingleFileBinaryPayloadCodecName =
    "legacy_singlefile_buffer";
inline constexpr const char* kColumnarCompactBasePayloadName =
    "pmsf_columnar_compact_base";
inline constexpr const char* kColumnarCompactBasePayloadCodecName =
    "columnar_compact_base";
inline constexpr std::int32_t kPaperRelationModelVersion = 3;
inline constexpr std::int32_t kPaperDeltaTupleArity =
    kPMSFSymbolDeltaTupleArity;

inline bool unifiedStorageEnabled() {
    return pmsfStorageEnvEnabled("PMSF_UNIFIED_STORAGE");
}

/// 互操作导出开关（主计划 §4 Phase 4 第 2 条）。
///
/// 唯一规范形态是 `_compact.bin`；以 Parquet 表达的表是**降级形式**，
/// 只在需要与外部工具互操作时额外导出，默认关闭 —— 否则同一份载荷会被
/// 写两遍，产物体积翻倍（那是容器重包装，不是 codec 改进）。
inline bool interopExportEnabled() {
    return pmsfStorageEnvEnabled("PMSF_INTEROP_EXPORT");
}

inline std::int32_t paperSideCellAsInt32(const SideCell& cell,
                                         std::int32_t fallback = 0) {
    if (std::holds_alternative<bool>(cell)) {
        return std::get<bool>(cell) ? 1 : 0;
    }
    if (std::holds_alternative<std::int8_t>(cell)) {
        return std::get<std::int8_t>(cell);
    }
    if (std::holds_alternative<std::int16_t>(cell)) {
        return std::get<std::int16_t>(cell);
    }
    if (std::holds_alternative<std::int32_t>(cell)) {
        return std::get<std::int32_t>(cell);
    }
    if (std::holds_alternative<std::uint16_t>(cell)) {
        return static_cast<std::int32_t>(std::get<std::uint16_t>(cell));
    }
    return fallback;
}

inline std::string paperPayloadBlobName(const PMSFSymbolPayloadRecord& row) {
    std::ostringstream name;
    name << kPMSFPayloadBlobPrefix
         << row.objectId
         << "_p" << row.partitionId
         << "_lod" << row.lodId
         << "_" << row.payloadName;
    return name.str();
}

inline PaperNamedBlob makePaperNamedBlob(const std::string& name,
                                         const std::string& kind,
                                         const void* data,
                                         std::size_t size) {
    PaperNamedBlob blob;
    blob.name = name;
    blob.kind = kind;
    if (data != nullptr && size > 0) {
        const auto* bytes = reinterpret_cast<const std::uint8_t*>(data);
        blob.bytes.assign(bytes, bytes + size);
    }
    return blob;
}

inline void setPaperNamedBlobBytes(PaperNamedBlob& blob,
                                   const void* data,
                                   std::size_t size) {
    blob.bytes.clear();
    if (data != nullptr && size > 0) {
        const auto* bytes = reinterpret_cast<const std::uint8_t*>(data);
        blob.bytes.assign(bytes, bytes + size);
    }
}

inline PaperNamedBlob makePMSFPayloadNamedBlob(PMSFSymbolPayloadRecord row) {
    PaperNamedBlob blob;
    blob.name = paperPayloadBlobName(row);
    blob.kind = row.codecName;
    blob.bytes = std::move(row.payload);
    return blob;
}

inline PaperNamedBlob makePaperNamedBlob(PMSFSymbolPayloadRecord row) {
    return makePMSFPayloadNamedBlob(std::move(row));
}

inline void appendPaperNamedBlobRecord(std::vector<PaperNamedBlob>& blobs,
                                       PaperNamedBlob blob) {
    blobs.push_back(std::move(blob));
}

inline void appendPaperNamedBlobRecord(std::vector<PaperNamedBlob>& blobs,
                                       PMSFSymbolPayloadRecord row) {
    appendPaperNamedBlobRecord(blobs, makePaperNamedBlob(std::move(row)));
}

struct PMSFSymbolPayloadSpec {
    std::int32_t objectId = 0;
    std::int32_t partitionId = 0;
    std::int32_t lodId = 0;
    std::string payloadName;
    std::string codecName;
    std::string payloadRoleName;
};

inline PMSFSymbolPayloadSpec makePMSFSymbolPayloadSpec(
    std::int32_t objectId,
    std::int32_t partitionId,
    std::int32_t lodId,
    const std::string& payloadName,
    const std::string& codecName) {
    PMSFSymbolPayloadSpec spec;
    spec.objectId = objectId;
    spec.partitionId = partitionId;
    spec.lodId = lodId;
    spec.payloadName = payloadName;
    spec.codecName = codecName;
    return spec;
}

inline std::string paperPayloadBlobName(const PMSFSymbolPayloadSpec& spec) {
    PMSFSymbolPayloadRecord row;
    row.objectId = spec.objectId;
    row.partitionId = spec.partitionId;
    row.lodId = spec.lodId;
    row.payloadName = spec.payloadName;
    return paperPayloadBlobName(row);
}

inline PMSFSymbolPayloadRecord makePMSFSymbolPayloadRecord(
    const PMSFSymbolPayloadSpec& spec) {
    return makePMSFSymbolPayloadRecord(spec.objectId,
                                        spec.partitionId,
                                        spec.lodId,
                                        spec.payloadName,
                                        spec.codecName,
                                        spec.payloadRoleName);
}

inline PMSFSymbolPayloadRecord makePMSFPayloadRecordWithBytes(
    std::int32_t objectId,
    std::int32_t partitionId,
    std::int32_t lodId,
    const std::string& payloadName,
    const std::string& codecName,
    const void* data,
    std::size_t size) {
    PMSFSymbolPayloadRecord row = makePMSFSymbolPayloadRecord(
        objectId, partitionId, lodId, payloadName, codecName);
    setPMSFSymbolPayloadBytes(row, data, size);
    return row;
}

inline PMSFSymbolPayloadRecord makePMSFPayloadRecordWithBytes(
    std::int32_t objectId,
    std::int32_t partitionId,
    std::int32_t lodId,
    const std::string& payloadName,
    const std::string& codecName,
    std::vector<std::uint8_t> payload) {
    PMSFSymbolPayloadRecord row = makePMSFSymbolPayloadRecord(
        objectId, partitionId, lodId, payloadName, codecName);
    setPMSFSymbolPayloadBytes(row, std::move(payload));
    return row;
}

inline PMSFSymbolPayloadRecord makePMSFPayloadRecord(
    const PMSFSymbolPayloadSpec& spec,
    const void* data,
    std::size_t size) {
    PMSFSymbolPayloadRecord row = makePMSFSymbolPayloadRecord(spec);
    setPMSFSymbolPayloadBytes(row, data, size);
    return row;
}

inline PMSFSymbolPayloadRecord makePMSFPayloadRecord(
    const PMSFSymbolPayloadSpec& spec,
    std::vector<std::uint8_t> payload) {
    PMSFSymbolPayloadRecord row = makePMSFSymbolPayloadRecord(spec);
    setPMSFSymbolPayloadBytes(row, std::move(payload));
    return row;
}

inline PaperNamedBlob makePaperNamedBlob(const PMSFSymbolPayloadSpec& spec,
                                         const void* data,
                                         std::size_t size) {
    return makePaperNamedBlob(makePMSFPayloadRecord(spec, data, size));
}

inline void appendPMSFPayloadRecord(PMSFSymbolPayloadTable& rows,
                                     PMSFSymbolPayloadRecord row) {
    rows.push_back(std::move(row));
}

inline void appendPMSFPayloadRecord(PMSFSymbolPayloadTable& rows,
                                     const PMSFSymbolPayloadSpec& spec,
                                     const void* data,
                                     std::size_t size) {
    appendPMSFPayloadRecord(rows, makePMSFPayloadRecord(spec, data, size));
}

inline void appendPMSFPayloadRecord(PMSFSymbolPayloadTable& rows,
                                     const PMSFSymbolPayloadSpec& spec,
                                     std::vector<std::uint8_t> payload) {
    appendPMSFPayloadRecord(rows,
                             makePMSFPayloadRecord(spec, std::move(payload)));
}

inline void appendPMSFPayloadRecord(PMSFSymbolPayloadTable& rows,
                                     std::int32_t objectId,
                                     std::int32_t partitionId,
                                     std::int32_t lodId,
                                     const std::string& payloadName,
                                     const std::string& codecName,
                                     const void* data,
                                     std::size_t size) {
    appendPMSFPayloadRecord(
        rows,
        makePMSFSymbolPayloadSpec(
            objectId, partitionId, lodId, payloadName, codecName),
        data,
        size);
}

inline void appendPMSFPayloadRecord(PMSFSymbolPayloadTable& rows,
                                     std::int32_t objectId,
                                     std::int32_t partitionId,
                                     std::int32_t lodId,
                                     const std::string& payloadName,
                                     const std::string& codecName,
                                     std::vector<std::uint8_t> payload) {
    appendPMSFPayloadRecord(
        rows,
        makePMSFSymbolPayloadSpec(
            objectId, partitionId, lodId, payloadName, codecName),
        std::move(payload));
}

inline bool parsePMSFPayloadBlobName(const std::string& name,
                                      PMSFSymbolPayloadRecord& row) {
    const std::string prefix(kPMSFPayloadBlobPrefix);
    if (name.rfind(prefix, 0) != 0) return false;
    const std::size_t pPos = name.find("_p", prefix.size());
    const std::size_t lodPos = name.find("_lod", pPos == std::string::npos ? 0 : pPos + 2);
    const std::size_t payloadPos = name.find("_", lodPos == std::string::npos ? 0 : lodPos + 4);
    if (pPos == std::string::npos || lodPos == std::string::npos ||
        payloadPos == std::string::npos) {
        return false;
    }
    try {
        setPMSFSymbolPayloadRecordKey(
            row,
            std::stoi(name.substr(prefix.size(), pPos - prefix.size())),
            std::stoi(name.substr(pPos + 2, lodPos - (pPos + 2))),
            std::stoi(name.substr(lodPos + 4, payloadPos - (lodPos + 4))));
        setPMSFSymbolPayloadName(row, name.substr(payloadPos + 1));
    } catch (...) {
        return false;
    }
    return true;
}

inline bool isPMSFPayloadBlobName(const std::string& name) {
    PMSFSymbolPayloadRecord ignored;
    return parsePMSFPayloadBlobName(name, ignored);
}

inline const PaperNamedBlob* findPaperNamedBlob(
    const std::vector<PaperNamedBlob>& blobs,
    const std::string& name) {
    for (const auto& blob : blobs) {
        if (blob.name == name) {
            return &blob;
        }
    }
    return nullptr;
}

inline const PaperNamedBlob* findPMSFPayloadOrRawNamedBlob(
    const std::vector<PaperNamedBlob>& blobs,
    std::int32_t objectId,
    std::int32_t partitionId,
    std::int32_t lodId,
    const std::string& payloadName) {
    const auto spec = makePMSFSymbolPayloadSpec(
        objectId, partitionId, lodId, payloadName, "");
    if (const auto* blob = findPaperNamedBlob(blobs, paperPayloadBlobName(spec))) {
        return blob;
    }
    return findPaperNamedBlob(blobs, payloadName);
}

inline const PaperNamedBlob* findPMSFPayloadOrRawNamedBlob(
    const std::vector<PaperNamedBlob>& blobs,
    const PMSFSymbolPayloadSpec& spec) {
    return findPMSFPayloadOrRawNamedBlob(
        blobs, spec.objectId, spec.partitionId, spec.lodId, spec.payloadName);
}

inline const std::vector<std::uint8_t>* findPaperNamedBlobBytes(
    const std::unordered_map<std::string, std::vector<std::uint8_t>>& blobs,
    const std::string& name) {
    const auto it = blobs.find(name);
    return it == blobs.end() ? nullptr : &it->second;
}

inline const std::vector<std::uint8_t>* findPMSFPayloadOrRawNamedBlobBytes(
    const std::unordered_map<std::string, std::vector<std::uint8_t>>& blobs,
    std::int32_t objectId,
    std::int32_t partitionId,
    std::int32_t lodId,
    const std::string& payloadName) {
    const auto spec = makePMSFSymbolPayloadSpec(
        objectId, partitionId, lodId, payloadName, "");
    if (const auto* bytes = findPaperNamedBlobBytes(blobs, paperPayloadBlobName(spec))) {
        return bytes;
    }
    return findPaperNamedBlobBytes(blobs, payloadName);
}

inline const std::vector<std::uint8_t>* findPMSFPayloadOrRawNamedBlobBytes(
    const std::unordered_map<std::string, std::vector<std::uint8_t>>& blobs,
    const PMSFSymbolPayloadSpec& spec) {
    return findPMSFPayloadOrRawNamedBlobBytes(
        blobs, spec.objectId, spec.partitionId, spec.lodId, spec.payloadName);
}

inline bool paperStorageEndsWith(const std::string& value,
                                 const std::string& suffix) {
    return value.size() >= suffix.size() &&
           value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

inline bool pmsfSymbolTableHasSymbolMapping(const PMSFSymbolTable& table) {
    const std::string symbolMappingSuffix =
        std::string("_") + kPMSFStorageContextSymbolMappingSuffix;
    for (const auto& row : table) {
        if (row.delta.totalCoreCells() > 0) {
            return true;
        }
        bool found = false;
        forEachPMSFSideChannel(row.delta, [&](const std::string& name, const auto& cells) {
            if (found || !paperStorageEndsWith(name, symbolMappingSuffix)) {
                return;
            }
            for (const auto& cell : cells) {
                if (paperSideCellAsInt32(cell) != 0) {
                    found = true;
                    return;
                }
            }
        });
        if (found) return true;
    }
    return false;
}

inline std::string paperStorageStripSuffix(const std::string& value,
                                           const std::string& suffix) {
    if (!paperStorageEndsWith(value, suffix)) return value;
    return value.substr(0, value.size() - suffix.size());
}

inline std::string paperStorageStripExtension(const std::string& value) {
    const std::string::size_type slash = value.find_last_of("/\\");
    const std::string::size_type dot = value.find_last_of('.');
    if (dot == std::string::npos || (slash != std::string::npos && dot < slash)) {
        return value;
    }
    return value.substr(0, dot);
}

inline void paperStorageAppendUnique(std::vector<std::string>& values,
                                     const std::string& value) {
    for (const auto& existing : values) {
        if (existing == value) return;
    }
    values.push_back(value);
}

inline constexpr const char kPaperV3MetadataParquetSuffix[] = "_v3_metadata.parquet";
inline constexpr const char kPaperV3BaseMeshParquetSuffix[] = "_v3_base_mesh.parquet";
inline constexpr const char kPaperV3SymbolsParquetSuffix[] = "_v3_symbols.parquet";
inline constexpr const char kPaperV3SideChannelParquetSuffix[] = "_v3_sidechannel.parquet";
inline constexpr const char kPaperV3PayloadBlobParquetSuffix[] = "_v3_blob.parquet";
inline constexpr const char kPaperV3CompactBinSuffix[] = "_v3_compact.bin";
inline constexpr const char kPaperMetadataParquetSuffix[] = "_metadata.parquet";
inline constexpr const char kPaperBaseMeshParquetSuffix[] = "_base_mesh.parquet";
inline constexpr const char kPMSFSymbolsParquetSuffix[] = "_symbols.parquet";
inline constexpr const char kPMSFSideChannelParquetSuffix[] = "_sidechannel.parquet";
inline constexpr const char kPMSFPayloadBlobParquetSuffix[] = "_blob.parquet";
inline constexpr const char kPMSFBoundaryParquetSuffix[] = "_boundary.parquet";
inline constexpr const char kPaperBitParquetSuffix[] = "_bit.parquet";
inline constexpr const char kPaperResidualParquetSuffix[] = "_residual.parquet";
inline constexpr const char kPaperLodParquetSuffix[] = "_lod.parquet";
inline constexpr const char kPaperSideParquetSuffix[] = "_side.parquet";
inline constexpr const char kPaperCompactBinSuffix[] = "_compact.bin";
inline constexpr const char kPMSFPayloadParquetSuffix[] = "_payload.parquet";
inline constexpr const char kPaperPartitionedBaseParquetSuffix[] = "_partitioned_base.parquet";
inline constexpr const char kPaperVerticesParquetSuffix[] = "_vertices.parquet";
inline constexpr const char kPaperPartitionsParquetSuffix[] = "_partitions.parquet";
inline constexpr const char kPaperCompatCluster0ParquetSuffix[] = "_cluster0.parquet";
inline constexpr const char kPaperCompatClustersParquetSuffix[] = "_clusters.parquet";
inline constexpr const char kPaperCompatVQParquetSuffix[] = "_vq.parquet";
inline constexpr const char kPaperCompatCodebookParquetSuffix[] = "_codebook.parquet";
inline constexpr const char kPaperV3StemSuffix[] = "_v3";
inline constexpr const char kPaperV2BlobTempPayloadSuffix[] = ".__v2blob.tmp.bin";

inline std::string paperStorageArtifactPath(const std::string& base,
                                            const char* suffix) {
    return base + suffix;
}

inline std::string paperMetadataTablePath(const std::string& base) {
    return paperStorageArtifactPath(base, kPaperMetadataParquetSuffix);
}

inline std::string paperBaseMeshTablePath(const std::string& base) {
    return paperStorageArtifactPath(base, kPaperBaseMeshParquetSuffix);
}

inline std::string paperSymbolsTablePath(const std::string& base) {
    return paperStorageArtifactPath(base, kPMSFSymbolsParquetSuffix);
}

inline std::string paperSideChannelTablePath(const std::string& base) {
    return paperStorageArtifactPath(base, kPMSFSideChannelParquetSuffix);
}

inline std::string paperPayloadBlobTablePath(const std::string& base) {
    return paperStorageArtifactPath(base, kPMSFPayloadBlobParquetSuffix);
}

inline std::string paperBoundaryTablePath(const std::string& base) {
    return paperStorageArtifactPath(base, kPMSFBoundaryParquetSuffix);
}

inline std::string paperBitTablePath(const std::string& base) {
    return paperStorageArtifactPath(base, kPaperBitParquetSuffix);
}

inline std::string paperResidualTablePath(const std::string& base) {
    return paperStorageArtifactPath(base, kPaperResidualParquetSuffix);
}

inline std::string paperLodTablePath(const std::string& base) {
    return paperStorageArtifactPath(base, kPaperLodParquetSuffix);
}

inline std::string paperSideTablePath(const std::string& base) {
    return paperStorageArtifactPath(base, kPaperSideParquetSuffix);
}

inline std::string paperCompactBinPath(const std::string& base) {
    return paperStorageArtifactPath(base, kPaperCompactBinSuffix);
}

inline void removePMSFStorageArtifactPath(const std::string& path) {
    std::error_code ec;
    std::filesystem::remove(path, ec);
}

inline void removePaperMetadataArtifactForBase(const std::string& base) {
    removePMSFStorageArtifactPath(paperMetadataTablePath(base));
}

inline void removePaperBaseMeshArtifactForBase(const std::string& base) {
    removePMSFStorageArtifactPath(paperBaseMeshTablePath(base));
}

inline void removePMSFSymbolsArtifactForBase(const std::string& base) {
    removePMSFStorageArtifactPath(paperSymbolsTablePath(base));
}

inline void removePaperCompactBinArtifactForBase(const std::string& base) {
    removePMSFStorageArtifactPath(paperCompactBinPath(base));
}

inline void removePaperCompactBaseParquetArtifactsForBase(const std::string& base) {
    removePaperMetadataArtifactForBase(base);
    removePaperBaseMeshArtifactForBase(base);
}

inline void removePaperBaseMeshAndCompactArtifactsForBase(const std::string& base) {
    removePaperBaseMeshArtifactForBase(base);
    removePaperCompactBinArtifactForBase(base);
}

inline void removePaperCompactBaseArtifactsForBase(const std::string& base) {
    removePaperCompactBaseParquetArtifactsForBase(base);
    removePaperCompactBinArtifactForBase(base);
}

inline std::string paperPayloadWrapperTablePath(const std::string& base) {
    return paperStorageArtifactPath(base, kPMSFPayloadParquetSuffix);
}

inline std::string paperColumnarBlobTempPayloadPath(const std::string& base) {
    return paperStorageArtifactPath(base, kPaperV2BlobTempPayloadSuffix);
}

inline std::string paperColumnarBlobTempPayloadPath(
    const std::string& path,
    const std::string& formatName) {
    return path + ".__pmsf_" + formatName + "_blob.tmp.bin";
}

inline std::string paperPartitionedBaseTablePath(const std::string& base) {
    return paperStorageArtifactPath(base, kPaperPartitionedBaseParquetSuffix);
}

inline std::string paperVerticesTablePath(const std::string& base) {
    return paperStorageArtifactPath(base, kPaperVerticesParquetSuffix);
}

inline std::string paperPartitionsTablePath(const std::string& base) {
    return paperStorageArtifactPath(base, kPaperPartitionsParquetSuffix);
}

// 删除某个基名下的**全部**已知产物。
//
// 用途只有一个：两遍式分区压缩里**勘察遍**（pass 1）的废件清理。勘察遍与
// 分区遍共用产物基名时，勘察遍留下的 `_bit` / `_lod` 表不会被分区遍覆盖
// —— 分区遍走 residual-only 布局，按设计**不写** lod 表
// （见 ColumnarSerializeOperator 里 "Skip it and let the decoder reconstruct
// from the single residual row"）—— 于是解码端 `reassembleBlobFromSegmentsV4`
// 会拿勘察遍的 lod 行去切分区遍的 residual，拼出一个 pass1/pass2 混合 blob
// （实测 24 + 50,778 + 58,074 = 161,063 B，前四个 int 是垃圾），分区前缀永远读不到。
//
// 只删本文件列出的已知产物，**不做目录扫描**：宁可漏删也不误删用户文件。
inline void removePaperArtifactsForBase(const std::string& base) {
    for (const std::string& path : {
             paperMetadataTablePath(base),
             paperBaseMeshTablePath(base),
             paperSymbolsTablePath(base),
             paperBitTablePath(base),
             paperResidualTablePath(base),
             paperLodTablePath(base),
             paperSideTablePath(base),
             paperBoundaryTablePath(base),
             paperPayloadBlobTablePath(base),
             paperCompactBinPath(base),
             paperPartitionedBaseTablePath(base),
         }) {
        removePMSFStorageArtifactPath(path);
    }
}

inline std::string paperStorageStripArtifactStemRole(const std::string& stem) {
    for (const char* suffix : {
             "_metadata",
             "_base_mesh",
             "_symbols",
             "_boundary",
             "_sidechannel",
             "_blob",
             "_payload",
             "_compact",
             "_partitioned_base",
             "_vertices",
             "_partitions",
         }) {
        if (paperStorageEndsWith(stem, suffix)) {
            return paperStorageStripSuffix(stem, suffix);
        }
    }
    return stem;
}

inline std::string paperV3StemFromStem(const std::string& stem) {
    if (paperStorageEndsWith(stem, kPaperV3StemSuffix)) {
        return stem;
    }
    return stem + kPaperV3StemSuffix;
}

inline std::string unifiedStemFromArtifactPath(const std::string& path) {
    return paperV3StemFromStem(
        paperStorageStripArtifactStemRole(paperStorageStripExtension(path)));
}

inline std::string unifiedCompatOutputPath(const std::string& outputPath) {
    return paperV3StemFromStem(paperStorageStripExtension(outputPath)) + ".parquet";
}

inline std::vector<std::string> paperStorageBaseCandidates(const std::string& path) {
    std::vector<std::string> bases;
    paperStorageAppendUnique(bases, path);
    paperStorageAppendUnique(bases, paperStorageStripExtension(path));
    for (const char* suffix : {
             kPaperV3MetadataParquetSuffix,
             kPaperV3BaseMeshParquetSuffix,
             kPaperV3SymbolsParquetSuffix,
             kPaperV3SideChannelParquetSuffix,
             kPaperV3PayloadBlobParquetSuffix,
             kPaperV3CompactBinSuffix,
             kPaperMetadataParquetSuffix,
             kPaperBaseMeshParquetSuffix,
             kPMSFSymbolsParquetSuffix,
             kPMSFSideChannelParquetSuffix,
             kPMSFPayloadBlobParquetSuffix,
             kPMSFBoundaryParquetSuffix,
             kPMSFPayloadParquetSuffix,
             kPaperCompactBinSuffix,
             kPaperPartitionedBaseParquetSuffix,
             kPaperVerticesParquetSuffix,
             kPaperPartitionsParquetSuffix,
             kPaperCompatCluster0ParquetSuffix,
             kPaperCompatClustersParquetSuffix,
             kPaperCompatVQParquetSuffix,
             kPaperCompatCodebookParquetSuffix,
         }) {
        paperStorageAppendUnique(bases, paperStorageStripSuffix(path, suffix));
    }
    return bases;
}

inline std::vector<std::string> paperSymbolSidecarCandidates(const std::string& path) {
    std::vector<std::string> candidates;
    for (const auto& base : paperStorageBaseCandidates(path)) {
        paperStorageAppendUnique(candidates, paperSideChannelTablePath(base));
        paperStorageAppendUnique(
            candidates,
            paperStorageArtifactPath(base, kPaperV3SideChannelParquetSuffix));
    }
    return candidates;
}

inline std::vector<std::string> paperPayloadBlobCandidates(const std::string& path) {
    std::vector<std::string> candidates;
    for (const auto& base : paperStorageBaseCandidates(path)) {
        paperStorageAppendUnique(candidates, paperPayloadBlobTablePath(base));
        paperStorageAppendUnique(
            candidates,
            paperStorageArtifactPath(base, kPaperV3PayloadBlobParquetSuffix));
    }
    return candidates;
}

inline bool paperPayloadMatches(const PMSFSymbolPayloadRecord& row,
                                std::int32_t objectId,
                                std::int32_t partitionId,
                                std::int32_t lodId,
                                const std::string& payloadName) {
    return row.objectId == objectId &&
           row.partitionId == partitionId &&
           row.lodId == lodId &&
           row.payloadName == payloadName;
}

inline const PMSFSymbolPayloadRecord* findPMSFPayload(
    const PMSFSymbolPayloadTable& table,
    std::int32_t objectId,
    std::int32_t partitionId,
    std::int32_t lodId,
    const std::string& payloadName) {
    for (const auto& row : table) {
        if (paperPayloadMatches(row, objectId, partitionId, lodId, payloadName)) {
            return &row;
        }
    }
    return nullptr;
}

inline bool paperPayloadMatches(const PMSFSymbolPayloadRecord& row,
                                const PMSFSymbolPayloadSpec& spec) {
    return paperPayloadMatches(
        row, spec.objectId, spec.partitionId, spec.lodId, spec.payloadName);
}

inline const PMSFSymbolPayloadRecord* findPMSFPayload(
    const PMSFSymbolPayloadTable& table,
    const PMSFSymbolPayloadSpec& spec) {
    for (const auto& row : table) {
        if (paperPayloadMatches(row, spec)) {
            return &row;
        }
    }
    return nullptr;
}

inline void setPaperInt32ScalarSideChannel(RoundSymbols& rs,
                                           const char* name,
                                           std::int32_t value) {
    erasePMSFSideChannel(rs, name);
    appendPaperI32SideChannel(rs, name, value);
}

inline void reservePMSFSideChannel(RoundSymbols& rs,
                                    const char* name,
                                    std::size_t additional) {
    reservePMSFSideChannelCells(rs, name, additional);
}

inline void appendPaperInt32SideChannel(RoundSymbols& rs,
                                        const char* name,
                                        std::int32_t value) {
    appendPaperI32SideChannel(rs, name, value);
}

inline void appendPaperBoolSideChannel(RoundSymbols& rs,
                                       const char* name,
                                       bool value) {
    appendPaperSideCellSideChannel(rs, name, value);
}

inline void reservePaperFlipEdgeSideChannels(RoundSymbols& rs,
                                             std::size_t additional) {
    reservePMSFSideChannel(rs, kPaperEdgeFlipSideChannel, additional);
    reservePMSFSideChannel(rs, kPaperFlipEdgeI32SideChannel, additional);
}

inline void appendPaperFlipEdgeSideChannels(RoundSymbols& rs,
                                            std::int32_t value) {
    appendPaperBoolSideChannel(rs, kPaperEdgeFlipSideChannel, value != 0);
    appendPaperInt32SideChannel(rs, kPaperFlipEdgeI32SideChannel, value);
}

inline void setPaperRelationProjectionSideChannels(RoundSymbols& rs,
                                                   const PMSFSymbolRecord& row) {
    setPaperInt32ScalarSideChannel(rs, kPaperObjectSideChannel, row.objectId);
    setPaperInt32ScalarSideChannel(rs, kPaperPartitionSideChannel, row.partitionId);
    setPaperInt32ScalarSideChannel(rs, kPaperLodSideChannel, row.lodId);
    setPaperInt32ScalarSideChannel(
        rs, kPaperRelationModelVersionSideChannel, kPaperRelationModelVersion);
    setPaperInt32ScalarSideChannel(
        rs, kPaperDeltaTupleAritySideChannel, kPaperDeltaTupleArity);
}

inline void erasePaperRelationProjectionSideChannels(PaperRoundSymbols& delta) {
    erasePMSFSideChannel(delta, kPaperObjectSideChannel);
    erasePMSFSideChannel(delta, kPaperPartitionSideChannel);
    erasePMSFSideChannel(delta, kPaperLodSideChannel);
    erasePMSFSideChannel(delta, kPaperFacetSymbolI32SideChannel);
    erasePMSFSideChannel(delta, kPaperTriangulationEdgeI32SideChannel);
    erasePMSFSideChannel(delta, kPaperFlipEdgeI32SideChannel);
    erasePMSFSideChannel(delta, kPaperVertexSymbolI32SideChannel);
}

inline void appendProjectedStorageRound(SymbolStream& out, RoundSymbols round) {
    out.push_back(std::move(round));
}

inline void appendPMSFSymbolRecordPointer(
    std::vector<const PMSFSymbolRecord*>& rows,
    const PMSFSymbolRecord& row) {
    rows.push_back(&row);
}

inline SymbolStream storageStreamFromPMSFSymbolStream(const PMSFSymbolStream& paper) {
    SymbolStream out;
    out.reserve(paper.size());
    for (const auto& pr : paper) {
        RoundSymbols rs;
        auto appendCodes = [](const auto& src, auto append) {
            for (const auto& cell : src) {
                append(static_cast<std::int32_t>(cell));
            }
        };
        appendCodes(pr.facetSymbols, [&](std::int32_t value) {
            appendRoundFacetSymbol(rs, value);
        });
        appendCodes(pr.triangulationEdgeSymbols, [&](std::int32_t value) {
            appendRoundTriangulationEdgeSymbol(rs, value);
        });
        appendCodes(pr.vertexSymbols, [&](std::int32_t value) {
            appendRoundVertexSymbol(rs, value);
        });
        for (const auto& point : pr.removedVertexCoords) {
            appendRoundRemovedVertexCoord(rs, point);
        }
        for (const auto& point : pr.shiftedVertexCoords) {
            appendRoundShiftedVertexCoord(rs, point);
        }
        replacePMSFSideChannelFromRound(rs, pr);
        if (!pr.flipEdgeSymbols.empty()) {
            reservePaperFlipEdgeSideChannels(rs, pr.flipEdgeSymbols.size());
            for (auto bit : pr.flipEdgeSymbols) {
                const auto value = static_cast<std::int32_t>(bit);
                appendPaperFlipEdgeSideChannels(rs, value);
            }
        }
        appendProjectedStorageRound(out, std::move(rs));
    }
    return out;
}

inline SymbolStream storageStreamFromUnifiedRounds(const SymbolStream& rounds) {
    return storageStreamFromPMSFSymbolStream(toPMSFSymbolStream(rounds));
}

inline SymbolStream storageStreamFromPMSFSymbolTable(const PMSFSymbolTable& table,
                                                      std::int32_t partitionId = -1,
                                                      std::int32_t objectId = -1) {
    std::vector<const PMSFSymbolRecord*> rows;
    for (const auto& row : table) {
        if ((objectId < 0 || row.objectId == objectId) &&
            (partitionId < 0 || row.partitionId == partitionId)) {
            appendPMSFSymbolRecordPointer(rows, row);
        }
    }
    std::sort(rows.begin(), rows.end(),
              [](const PMSFSymbolRecord* a, const PMSFSymbolRecord* b) {
                  if (a->objectId != b->objectId) {
                      return a->objectId < b->objectId;
                  }
                  if (a->partitionId != b->partitionId) {
                      return a->partitionId < b->partitionId;
                  }
                  return a->lodId < b->lodId;
              });
    SymbolStream out;
    out.reserve(rows.size());
    for (const PMSFSymbolRecord* row : rows) {
        PMSFSymbolStream one{row->delta};
        SymbolStream projected = storageStreamFromPMSFSymbolStream(one);
        if (projected.empty()) continue;
        setPaperRelationProjectionSideChannels(projected[0], *row);
        appendProjectedStorageRound(out, std::move(projected[0]));
    }
    return out;
}

inline std::int32_t firstInt32SideValue(const RoundSymbols& rs,
                                        const std::string& name,
                                        std::int32_t fallback) {
    std::int32_t out = fallback;
    visitPMSFSideChannelCellAt(rs, name, 0, [&](const SideCell& v) {
        if (std::holds_alternative<std::int32_t>(v)) out = std::get<std::int32_t>(v);
        else if (std::holds_alternative<std::int16_t>(v)) out = std::get<std::int16_t>(v);
        else if (std::holds_alternative<std::int8_t>(v)) out = std::get<std::int8_t>(v);
    });
    return out;
}

inline std::deque<SymbolCode>
paperCodeDequeFromSideOrLegacy(const RoundSymbols& rs,
                               const std::string& sideName,
                               const std::deque<SymbolCode>& legacy) {
    std::deque<SymbolCode> out;
    const std::size_t count = paperSideChannelCellCount(rs, sideName);
    if (count > 0) {
        out.resize(count);
        std::size_t i = 0;
        forEachPMSFSideChannelCell(rs, sideName, [&](const SideCell& v) {
            if (std::holds_alternative<std::int32_t>(v)) {
                out[i] = std::get<std::int32_t>(v);
            } else if (std::holds_alternative<std::int16_t>(v)) {
                out[i] = std::get<std::int16_t>(v);
            } else if (std::holds_alternative<std::int8_t>(v)) {
                out[i] = std::get<std::int8_t>(v);
            } else if (std::holds_alternative<std::uint16_t>(v)) {
                out[i] = static_cast<SymbolCode>(
                    std::get<std::uint16_t>(v));
            } else if (std::holds_alternative<bool>(v)) {
                out[i] = std::get<bool>(v) ? 1 : 0;
            } else {
                out[i] = 0;
            }
            ++i;
        });
        return out;
    }
    return toPMSFSymbolCodeDeque(legacy);
}

inline PMSFSymbolTable pmsfSymbolTableFromStorageStream(const SymbolStream& stream) {
    PMSFSymbolTable table;
    table.reserve(stream.size());
    for (std::size_t r = 0; r < stream.size(); ++r) {
        const auto& rs = stream[r];
        if (paperSideChannelsEmpty(rs) && rs.totalCoreCells() == 0) continue;
        PMSFSymbolRecord& row = appendPMSFSymbolRecord(
            table,
            firstInt32SideValue(rs, kPaperObjectSideChannel, 0),
            firstInt32SideValue(rs, kPaperPartitionSideChannel, 0),
            firstInt32SideValue(
                rs, kPaperLodSideChannel, static_cast<std::int32_t>(r)));
        row.delta = toPaperRoundSymbols(rs);
        setPaperScalarSymbolsForRole(
            row.delta,
            PMSFSymbolRole::Facet,
            paperCodeDequeFromSideOrLegacy(
                rs, kPaperFacetSymbolI32SideChannel, rs.facetSymbols));
        setPaperScalarSymbolsForRole(
            row.delta,
            PMSFSymbolRole::TriangulationEdge,
            paperCodeDequeFromSideOrLegacy(
                rs, kPaperTriangulationEdgeI32SideChannel, rs.edgeSymbols));
        setPaperScalarSymbolsForRole(
            row.delta,
            PMSFSymbolRole::Vertex,
            paperCodeDequeFromSideOrLegacy(
                rs, kPaperVertexSymbolI32SideChannel, rs.vertexSymbols));
        if (paperSideChannelHasCells(rs, kPaperFlipEdgeI32SideChannel)) {
            setPaperScalarSymbolsForRole(
                row.delta,
                PMSFSymbolRole::FlipEdge,
                std::deque<SymbolCode>{});
            forEachPMSFSideChannelCell(rs, kPaperFlipEdgeI32SideChannel, [&](const SideCell& v) {
                if (std::holds_alternative<std::int32_t>(v)) {
                    appendPaperScalarSymbolForRole(
                        row.delta,
                        PMSFSymbolRole::FlipEdge,
                        std::get<std::int32_t>(v));
                } else if (std::holds_alternative<bool>(v)) {
                    appendPaperScalarSymbolForRole(
                        row.delta,
                        PMSFSymbolRole::FlipEdge,
                        std::get<bool>(v) ? 1 : 0);
                }
            });
        }
        erasePaperRelationProjectionSideChannels(row.delta);
    }
    std::sort(table.begin(), table.end(),
              [](const PMSFSymbolRecord& a, const PMSFSymbolRecord& b) {
                  if (a.objectId != b.objectId) return a.objectId < b.objectId;
                  if (a.partitionId != b.partitionId) return a.partitionId < b.partitionId;
                  return a.lodId < b.lodId;
              });
    return table;
}

inline bool sideSpecFromCell(const std::string& name,
                             const SideCell& cell,
                             SideChannelSpec& out) {
    out.name = name;
    out.scope = SideChannelSpec::Scope::PerRoundList;
    if (std::holds_alternative<bool>(cell)) {
        out.cellType = SideChannelSpec::Type::Bool;
        out.shape = SideChannelSpec::Shape::Scalar;
        return true;
    }
    if (std::holds_alternative<std::int8_t>(cell)) {
        out.cellType = SideChannelSpec::Type::Int8;
        out.shape = SideChannelSpec::Shape::Scalar;
        return true;
    }
    if (std::holds_alternative<std::int16_t>(cell)) {
        out.cellType = SideChannelSpec::Type::Int16;
        out.shape = SideChannelSpec::Shape::Scalar;
        return true;
    }
    if (std::holds_alternative<std::int32_t>(cell)) {
        out.cellType = SideChannelSpec::Type::Int32;
        out.shape = SideChannelSpec::Shape::Scalar;
        return true;
    }
    if (std::holds_alternative<std::uint16_t>(cell)) {
        out.cellType = SideChannelSpec::Type::UInt16;
        out.shape = SideChannelSpec::Shape::Scalar;
        return true;
    }
    if (std::holds_alternative<float>(cell)) {
        out.cellType = SideChannelSpec::Type::Float;
        out.shape = SideChannelSpec::Shape::Scalar;
        return true;
    }
    if (std::holds_alternative<MCGAL::Point>(cell)) {
        out.cellType = SideChannelSpec::Type::Vec3f;
        out.shape = SideChannelSpec::Shape::Vec3;
        return true;
    }
    if (std::holds_alternative<std::vector<std::uint8_t>>(cell)) {
        out.cellType = SideChannelSpec::Type::Bytes;
        out.shape = SideChannelSpec::Shape::Scalar;
        return true;
    }
    return false;
}

inline bool hasSideSpec(const std::vector<SideChannelSpec>& specs,
                        const std::string& name) {
    for (const auto& spec : specs) {
        if (spec.name == name) return true;
    }
    return false;
}

inline std::vector<SideChannelSpec> inferSideChannelSpecsForStorage(
    const SymbolStream& stream) {
    std::vector<SideChannelSpec> specs;
    for (const auto& round : stream) {
        forEachPMSFSideChannel(round, [&](const std::string& name, const auto& cells) {
            if (cells.empty() || hasSideSpec(specs, name)) return;
            SideChannelSpec spec;
            if (sideSpecFromCell(name, cells.front(), spec)) {
                appendPMSFSideChannelSpec(specs, std::move(spec));
            }
        });
    }
    std::sort(specs.begin(), specs.end(),
              [](const SideChannelSpec& a, const SideChannelSpec& b) {
                  return a.name < b.name;
              });
    return specs;
}

}  // namespace pmsf

#endif  // PMSF_STORAGE_PAPER_SYMBOL_STORAGE_ADAPTER_H
