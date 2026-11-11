#ifndef PMSF_STORAGE_VECTOR_QUANTIZED_PAPER_PAYLOAD_ADAPTER_H
#define PMSF_STORAGE_VECTOR_QUANTIZED_PAPER_PAYLOAD_ADAPTER_H

#include "Point.h"
#include "VQMMC/common/VQMMCQuantization.h"
#include "common/PaperLayoutAliases.h"
#include "core/PMSFBoundarySideChannel.h"
#include "core/PMSFSideChannel.h"
#include "VQMMC/boundary/VQMMCPMSFBoundary.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂
#include "storage/PMSFPayloadDescriptorRegistry.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <stdexcept>
#include <utility>
#include <variant>
#include <vector>

namespace pmsf {

inline constexpr int kVQMMCPayloadFormatVersion = 1;
inline constexpr const char* kVQMMCPayloadCodebookName = "vqmmc_codebook";
inline constexpr const char* kVQMMCPayloadIndicesName = "vqmmc_indices";
inline constexpr const char* kVQMMCPayloadFacetSymbolsName = "vqmmc_facet_symbols";
inline constexpr const char* kVQMMCPayloadEdgeSymbolsName = "vqmmc_edge_symbols";
inline constexpr const char* kVQMMCPayloadMetadataName = "vqmmc_metadata";
inline constexpr const char* kVQMMCPayloadBaseMeshName = "vqmmc_base_mesh";
inline constexpr const char* kVQMMCPayloadCodecName = "vqmmc_progressive_vq";
inline constexpr const char* kVQMMCPayloadBitstreamCodecName = "vqmmc_bitstream";
inline constexpr const char* kVQMMCCompatPayloadCodecName = "vqmmc_columnar_compat";
inline constexpr const char* kVQMMCCompatBaseTablesSkippedSideName =
    "vqmmc_compat_base_tables_skipped";
inline constexpr const char* kVQMMCMetadataPayloadPresentSideName =
    "vqmmc_metadata_payload_present";
inline constexpr const char* kVQMMCBaseMeshPayloadPresentSideName =
    "vqmmc_base_mesh_payload_present";
inline constexpr const char* kVQMMCLegacyVQBlockPresentSideName =
    "vqmmc_legacy_vq_block_present";
inline constexpr const char* kVQMMCPayloadCodebookPresentSideName =
    "vqmmc_payload_codebook_present";
inline constexpr const char* kVQMMCPayloadIndexPresentSideName =
    "vqmmc_payload_index_present";
inline constexpr const char* kVQMMCPayloadTableFormatVersionSideName =
    "vqmmc_payload_table_format_version";
inline constexpr const char* kVQMMCPayloadLittleEndianSideName =
    "vqmmc_payload_little_endian";
inline constexpr const char* kVQMMCPayloadImplCompatibleSideName =
    "vqmmc_payload_impl_vqmmc_compatible";
inline constexpr const char* kVQMMCCompatSymbolsTableSkippedSideName =
    "vqmmc_compat_symbols_table_skipped";
inline constexpr const char* kVQMMCCompatVQTableSkippedSideName =
    "vqmmc_compat_vq_table_skipped";
inline constexpr const char* kVQMMCPayloadFormatVersionSideName =
    "vqmmc_payload_format_version";
inline constexpr const char* kVQMMCPayloadCodebookQuantBitsSideName =
    "vqmmc_payload_codebook_quant_bits";
inline constexpr const char* kVQMMCPayloadIndexBitsSideName =
    "vqmmc_payload_index_bits";
inline constexpr const char* kVQMMCSidecarCodebookSizeSideName =
    "vqmmc_sidecar_codebook_size";
inline constexpr const char* kVQMMCSidecarIndexCountSideName =
    "vqmmc_sidecar_index_count";
inline constexpr const char* kVQMMCBlobLayoutSideName = "vqmmc_blob_layout";
inline constexpr const char* kVQMMCMainBlobStreamCountSideName =
    "vqmmc_main_blob_stream_count";
inline constexpr const char* kVQMMCTargetBlobStreamCountSideName =
    "vqmmc_target_blob_stream_count";
inline constexpr const char* kVQMMCBlobCodebookStreamPresentSideName =
    "vqmmc_blob_codebook_stream_present";
inline constexpr const char* kVQMMCBlobIndexStreamPresentSideName =
    "vqmmc_blob_index_stream_present";

inline int vqmmcPaperIndexBitsForCodebookSize(int codebookSize);

template <typename SideMap>
inline void appendVQMMCBlobTransportSideMap(
    SideMap& side,
    std::int32_t blobLayout,
    std::int32_t mainBlobStreamCount,
    std::int32_t targetBlobStreamCount,
    std::int32_t codebookStreamPresent,
    std::int32_t indexStreamPresent)
{
    appendPaperI32SideMap(side, kVQMMCBlobLayoutSideName, blobLayout);
    appendPaperI32SideMap(
        side, kVQMMCMainBlobStreamCountSideName, mainBlobStreamCount);
    appendPaperI32SideMap(
        side, kVQMMCTargetBlobStreamCountSideName, targetBlobStreamCount);
    appendPaperI32SideMap(
        side, kVQMMCBlobCodebookStreamPresentSideName, codebookStreamPresent);
    appendPaperI32SideMap(
        side, kVQMMCBlobIndexStreamPresentSideName, indexStreamPresent);
}

template <typename SideMap>
inline void appendVQMMCBlobTransportSideMap(
    SideMap& side,
    const PaperConnectivityBlobSpec& mainSpec,
    std::int32_t targetBlobStreamCount,
    std::int32_t codebookStreamPresent,
    std::int32_t indexStreamPresent)
{
    appendVQMMCBlobTransportSideMap(side,
                                    mainSpec.blobLayout,
                                    mainSpec.streamCount,
                                    targetBlobStreamCount,
                                    codebookStreamPresent,
                                    indexStreamPresent);
}

template <typename PaperRoundSymbols>
inline void appendVQMMCBlobTransportSideChannel(
    PaperRoundSymbols& delta,
    const PaperConnectivityBlobSpec& mainSpec,
    std::int32_t targetBlobStreamCount,
    std::int32_t codebookStreamPresent,
    std::int32_t indexStreamPresent)
{
    appendPaperI32SideChannel(
        delta, kVQMMCBlobLayoutSideName, mainSpec.blobLayout);
    appendPaperI32SideChannel(
        delta, kVQMMCMainBlobStreamCountSideName, mainSpec.streamCount);
    appendPaperI32SideChannel(
        delta, kVQMMCTargetBlobStreamCountSideName, targetBlobStreamCount);
    appendPaperI32SideChannel(
        delta, kVQMMCBlobCodebookStreamPresentSideName, codebookStreamPresent);
    appendPaperI32SideChannel(
        delta, kVQMMCBlobIndexStreamPresentSideName, indexStreamPresent);
}

inline int vqmmcSideCellAsInt(const SideCell& cell) {
    if (const auto* v = std::get_if<bool>(&cell)) return *v ? 1 : 0;
    if (const auto* v = std::get_if<std::int8_t>(&cell)) return *v;
    if (const auto* v = std::get_if<std::int16_t>(&cell)) return *v;
    if (const auto* v = std::get_if<std::int32_t>(&cell)) return *v;
    if (const auto* v = std::get_if<std::uint16_t>(&cell)) return *v;
    return 0;
}

inline bool vqmmcPMSFSymbolTableAnySideFlag(const PMSFSymbolTable* table,
                                             const char* key) {
    if (!table) return false;
    for (const auto& row : *table) {
        bool found = false;
        forEachPMSFSideChannelCell(row.delta, key, [&](const auto& cell) {
            if (vqmmcSideCellAsInt(cell) != 0) {
                found = true;
            }
        });
        if (found) return true;
    }
    return false;
}

inline bool vqmmcPMSFSymbolTableMarksBlobPayloadStreamsPresent(
    const PMSFSymbolTable* table) {
    return vqmmcPMSFSymbolTableAnySideFlag(
               table, kVQMMCBlobCodebookStreamPresentSideName) ||
           vqmmcPMSFSymbolTableAnySideFlag(
               table, kVQMMCBlobIndexStreamPresentSideName);
}

template <typename SideMap>
inline void appendVQMMCPayloadContextSideMap(
    SideMap& side,
    int codebookSize,
    int indexCount,
    std::int32_t compatSymbolsTableSkipped,
    std::int32_t compatVQTableSkipped,
    std::int32_t compatBaseTablesSkipped)
{
    appendPaperI32SideMap(side, kVQMMCLegacyVQBlockPresentSideName, 1);
    appendPaperI32SideMap(
        side,
        kVQMMCPayloadCodebookPresentSideName,
        static_cast<std::int32_t>(codebookSize > 0 ? 1 : 0));
    appendPaperI32SideMap(
        side,
        kVQMMCPayloadIndexPresentSideName,
        static_cast<std::int32_t>(indexCount > 0 ? 1 : 0));
    appendPaperI32SideMap(
        side, kVQMMCPayloadTableFormatVersionSideName,
        static_cast<std::int32_t>(kVQMMCPayloadFormatVersion));
    appendPaperI32SideMap(
        side,
        kVQMMCPayloadCodebookQuantBitsSideName,
        static_cast<std::int32_t>(
            codebookSize > 0 ? vqmmcConfiguredCodebookQuantBits() : 0));
    appendPaperI32SideMap(
        side,
        kVQMMCPayloadIndexBitsSideName,
        static_cast<std::int32_t>(
            indexCount > 0 ? vqmmcPaperIndexBitsForCodebookSize(codebookSize) : 0));
    appendPaperI32SideMap(
        side, kVQMMCSidecarCodebookSizeSideName,
        static_cast<std::int32_t>(codebookSize));
    appendPaperI32SideMap(
        side, kVQMMCSidecarIndexCountSideName,
        static_cast<std::int32_t>(indexCount));
    appendPaperI32SideMap(side, kVQMMCPayloadLittleEndianSideName, 1);
    appendPaperI32SideMap(side, kVQMMCPayloadImplCompatibleSideName, 1);
    appendPaperI32SideMap(
        side, kVQMMCCompatSymbolsTableSkippedSideName, compatSymbolsTableSkipped);
    appendPaperI32SideMap(
        side, kVQMMCCompatVQTableSkippedSideName, compatVQTableSkipped);
    appendPaperI32SideMap(
        side, kVQMMCCompatBaseTablesSkippedSideName, compatBaseTablesSkipped);
}


template <typename PaperRoundSymbols>
inline void appendVQMMCCodebookPayloadStorage(PaperRoundSymbols& delta,
                                              int codebookSize,
                                              int indexCount)
{
    const auto mainSpec = vqmmcLegacyCodebookConnectivityBlobSpec(
        /*roundCount=*/1);
    appendVQMMCBlobTransportSideChannel(delta,
                                        mainSpec,
                                        kPaperBlobStreamCountVQMMCTarget,
                                        0,
                                        0);
    appendPaperI32SideChannel(delta, kVQMMCLegacyVQBlockPresentSideName, 1);
    appendPaperI32SideChannel(
        delta,
        kVQMMCPayloadCodebookPresentSideName,
        static_cast<std::int32_t>(codebookSize > 0 ? 1 : 0));
    appendPaperI32SideChannel(
        delta,
        kVQMMCPayloadIndexPresentSideName,
        static_cast<std::int32_t>(indexCount > 0 ? 1 : 0));
    appendPaperI32SideChannel(
        delta,
        kVQMMCPayloadTableFormatVersionSideName,
        static_cast<std::int32_t>(kVQMMCPayloadFormatVersion));
    appendPaperI32SideChannel(
        delta,
        kVQMMCPayloadCodebookQuantBitsSideName,
        static_cast<std::int32_t>(
            codebookSize > 0 ? vqmmcConfiguredCodebookQuantBits() : 0));
    appendPaperI32SideChannel(
        delta,
        kVQMMCPayloadIndexBitsSideName,
        static_cast<std::int32_t>(
            indexCount > 0 ? vqmmcPaperIndexBitsForCodebookSize(codebookSize) : 0));
    appendPaperI32SideChannel(
        delta,
        kVQMMCSidecarCodebookSizeSideName,
        static_cast<std::int32_t>(codebookSize));
    appendPaperI32SideChannel(
        delta,
        kVQMMCSidecarIndexCountSideName,
        static_cast<std::int32_t>(indexCount));
    appendPaperI32SideChannel(delta, kVQMMCPayloadLittleEndianSideName, 1);
    appendPaperI32SideChannel(delta, kVQMMCPayloadImplCompatibleSideName, 1);
    appendPaperI32SideChannel(delta, kVQMMCCompatSymbolsTableSkippedSideName, 0);
    appendPaperI32SideChannel(delta, kVQMMCCompatVQTableSkippedSideName, 0);
    appendPaperI32SideChannel(delta, kVQMMCCompatBaseTablesSkippedSideName, 0);
    appendPMSFBoundaryAbsentExactPair(
        delta, "vqmmc", "paper", boundaryFlagI32(pmsf::vqmmc_boundary::kPaperExactSpec));
}

inline int vqmmcPaperIndexBitsForCodebookSize(int codebookSize) {
    if (codebookSize <= 1) return 1;
    std::uint32_t value = static_cast<std::uint32_t>(codebookSize - 1);
    int bits = 0;
    while (value) {
        value >>= 1;
        ++bits;
    }
    return std::max(1, bits);
}

inline void appendVQMMCI32LE(std::vector<std::uint8_t>& out, std::int32_t value) {
    const auto u = static_cast<std::uint32_t>(value);
    out.push_back(static_cast<std::uint8_t>(u & 0xffu));
    out.push_back(static_cast<std::uint8_t>((u >> 8) & 0xffu));
    out.push_back(static_cast<std::uint8_t>((u >> 16) & 0xffu));
    out.push_back(static_cast<std::uint8_t>((u >> 24) & 0xffu));
}

inline void appendVQMMCU16LE(std::vector<std::uint8_t>& out, std::uint16_t value) {
    out.push_back(static_cast<std::uint8_t>(value & 0xffu));
    out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xffu));
}

inline void appendVQMMCF32LE(std::vector<std::uint8_t>& out, float value) {
    std::uint32_t u = 0;
    std::memcpy(&u, &value, sizeof(value));
    out.push_back(static_cast<std::uint8_t>(u & 0xffu));
    out.push_back(static_cast<std::uint8_t>((u >> 8) & 0xffu));
    out.push_back(static_cast<std::uint8_t>((u >> 16) & 0xffu));
    out.push_back(static_cast<std::uint8_t>((u >> 24) & 0xffu));
}

inline PMSFPayloadDescriptor vqmmcPayloadDescriptor(std::int32_t lodId,
                                                     const char* payloadName,
                                                     const char* codecName) {
    return makePMSFPayloadDescriptorWithInferredRole(
        0,
        0,
        lodId,
        payloadName == nullptr ? "" : payloadName,
        codecName == nullptr ? "" : codecName);
}

inline PMSFSymbolPayloadSpec vqmmcPayloadSpec(std::int32_t lodId,
                                               const char* payloadName,
                                               const char* codecName) {
    return paperPayloadSpecFromDescriptor(
        vqmmcPayloadDescriptor(lodId, payloadName, codecName));
}

inline void appendVQMMCPMSFPayload(
    PMSFSymbolPayloadTable& rows,
    std::int32_t lodId,
    const char* payloadName,
    const char* codecName,
    std::vector<std::uint8_t> payload) {
    appendPMSFPayloadRecordFromDescriptor(
        rows,
        vqmmcPayloadDescriptor(lodId, payloadName, codecName),
        std::move(payload));
}

inline void appendVQMMCPMSFPayload(
    PMSFSymbolPayloadTable& rows,
    std::int32_t lodId,
    const char* payloadName,
    std::vector<std::uint8_t> payload) {
    appendVQMMCPMSFPayload(rows,
                            lodId,
                            payloadName,
                            kVQMMCPayloadCodecName,
                            std::move(payload));
}

inline std::vector<std::uint8_t> encodeVQMMCPaperCodebookPayload(
    const std::vector<MCGAL::Point>& codebook) {
    std::vector<std::uint8_t> payload;
    if (codebook.empty()) {
        return payload;
    }

    const int quantBits = vqmmcConfiguredCodebookQuantBits();
    const std::uint32_t quantMax = (1u << quantBits) - 1u;
    payload.reserve(32 + codebook.size() * 6);

    float cMin[3] = {0.0f, 0.0f, 0.0f};
    float cStep[3] = {1.0f, 1.0f, 1.0f};
    float lo[3] = {1e30f, 1e30f, 1e30f};
    float hi[3] = {-1e30f, -1e30f, -1e30f};
    for (const auto& p : codebook) {
        const float v[3] = {static_cast<float>(p.x()),
                            static_cast<float>(p.y()),
                            static_cast<float>(p.z())};
        for (int axis = 0; axis < 3; ++axis) {
            lo[axis] = std::min(lo[axis], v[axis]);
            hi[axis] = std::max(hi[axis], v[axis]);
        }
    }
    for (int axis = 0; axis < 3; ++axis) {
        cMin[axis] = lo[axis];
        float range = hi[axis] - lo[axis];
        if (range <= 0.0f) range = 1e-9f;
        cStep[axis] = range / static_cast<float>(quantMax);
    }

    const int codebookSize = static_cast<int>(codebook.size());
    appendVQMMCI32LE(
        payload,
        static_cast<std::int32_t>(vqmmcSignedCodebookSize(
            codebookSize, vqmmcConfiguredAbsoluteGeometry())));
    appendVQMMCI32LE(payload,
                     static_cast<std::int32_t>(quantBits));
    for (float value : cMin) appendVQMMCF32LE(payload, value);
    for (float value : cStep) appendVQMMCF32LE(payload, value);
    for (const auto& p : codebook) {
        const float v[3] = {static_cast<float>(p.x()),
                            static_cast<float>(p.y()),
                            static_cast<float>(p.z())};
        for (int axis = 0; axis < 3; ++axis) {
            int q = static_cast<int>(
                std::round((v[axis] - cMin[axis]) / cStep[axis]));
            q = std::clamp(q, 0, static_cast<int>(quantMax));
            appendVQMMCU16LE(payload, static_cast<std::uint16_t>(q));
        }
    }
    return payload;
}

inline std::vector<std::uint8_t> encodeVQMMCPaperIndexPayload(
    const std::deque<std::uint16_t>& indices,
    int codebookSize) {
    std::vector<std::uint8_t> payload;
    if (indices.empty()) {
        return payload;
    }
    payload.reserve(8 + indices.size() * 2);
    appendVQMMCI32LE(
        payload,
        static_cast<std::int32_t>(vqmmcPaperIndexBitsForCodebookSize(codebookSize)));
    appendVQMMCI32LE(payload, static_cast<std::int32_t>(indices.size()));
    for (std::uint16_t value : indices) {
        appendVQMMCU16LE(payload, value);
    }
    return payload;
}

inline void appendVQMMCPMSFPayloadRows(
    PMSFSymbolPayloadTable& rows,
    const std::vector<std::deque<std::uint16_t>>& indexQueues,
    const std::vector<std::vector<MCGAL::Point>>& codebookQueues) {
    const std::size_t rounds = std::max(indexQueues.size(), codebookQueues.size());
    // Payload lod ids follow the collector's forward order, including legacy
    // files and side-channel rows. The Binary compatibility reader reverses
    // them when reconstructing the coarse-to-fine stream.
    for (std::size_t r = 0; r < rounds; ++r) {
        if (r < codebookQueues.size() && !codebookQueues[r].empty()) {
            auto payload = encodeVQMMCPaperCodebookPayload(codebookQueues[r]);
            if (!payload.empty()) {
                appendVQMMCPMSFPayload(
                    rows,
                    static_cast<std::int32_t>(r),
                    kVQMMCPayloadCodebookName,
                    std::move(payload));
            }
        }
        if (r < indexQueues.size() && !indexQueues[r].empty()) {
            const int cbSize = r < codebookQueues.size()
                                   ? static_cast<int>(codebookQueues[r].size())
                                   : 0;
            auto payload = encodeVQMMCPaperIndexPayload(indexQueues[r], cbSize);
            if (!payload.empty()) {
                appendVQMMCPMSFPayload(
                    rows,
                    static_cast<std::int32_t>(r),
                    kVQMMCPayloadIndicesName,
                    std::move(payload));
            }
        }
    }
}

}  // namespace pmsf

#endif  // PMSF_STORAGE_VECTOR_QUANTIZED_PAPER_PAYLOAD_ADAPTER_H
