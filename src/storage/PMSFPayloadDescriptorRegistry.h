#ifndef PMSF_STORAGE_PAPER_PAYLOAD_DESCRIPTOR_REGISTRY_H
#define PMSF_STORAGE_PAPER_PAYLOAD_DESCRIPTOR_REGISTRY_H

#include "storage/PMSFSymbolStorageAdapter.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace pmsf {

enum class PMSFPayloadRole {
    AlgorithmPayload,
    BaseMesh,
    Cluster0,
    ClusterRest,
    ColumnarContainer,
    CompactBase,
    ConnectivityBlob,
    EdgeSymbols,
    FacetSymbols,
    Indices,
    LegacyBinary,
    Metadata,
    OperationStream,
    SymbolStreamBlob,
};

inline const char* paperPayloadRoleName(PMSFPayloadRole role) {
    switch (role) {
        case PMSFPayloadRole::AlgorithmPayload:
            return "algorithm_payload";
        case PMSFPayloadRole::BaseMesh:
            return "base_mesh";
        case PMSFPayloadRole::Cluster0:
            return "cluster0";
        case PMSFPayloadRole::ClusterRest:
            return "cluster_rest";
        case PMSFPayloadRole::ColumnarContainer:
            return "columnar_container";
        case PMSFPayloadRole::CompactBase:
            return "compact_base";
        case PMSFPayloadRole::ConnectivityBlob:
            return "connectivity_blob";
        case PMSFPayloadRole::EdgeSymbols:
            return "edge_symbols";
        case PMSFPayloadRole::FacetSymbols:
            return "facet_symbols";
        case PMSFPayloadRole::Indices:
            return "indices";
        case PMSFPayloadRole::LegacyBinary:
            return "legacy_binary";
        case PMSFPayloadRole::Metadata:
            return "metadata";
        case PMSFPayloadRole::OperationStream:
            return "operation_stream";
        case PMSFPayloadRole::SymbolStreamBlob:
            return "symbol_stream_blob";
    }
    return "algorithm_payload";
}

inline PMSFPayloadRole inferPMSFPayloadRole(const std::string& payloadName,
                                             const std::string& codecName = {}) {
    if (payloadName == kLegacySingleFileBinaryPayloadName) {
        return PMSFPayloadRole::LegacyBinary;
    }
    if (payloadName == kColumnarCompactBasePayloadName) {
        return PMSFPayloadRole::CompactBase;
    }
    if (payloadName == "pmsf_symbols_rc_blob") {
        return PMSFPayloadRole::SymbolStreamBlob;
    }
    if (payloadName == "ppmc_operation_rc" ||
        payloadName == "pomar_legacy_payload" ||
        payloadName == "seg_edge_collapse_payload") {
        return PMSFPayloadRole::OperationStream;
    }
    if (payloadName == "pomar_base_mesh" ||
        payloadName == "vqmmc_base_mesh") {
        return PMSFPayloadRole::BaseMesh;
    }
    if (payloadName == "pomar_cluster0") {
        return PMSFPayloadRole::Cluster0;
    }
    if (payloadName == "pomar_meta" ||
        payloadName == "vqmmc_metadata") {
        return PMSFPayloadRole::Metadata;
    }
    if (payloadName == "pomar_rest") {
        return PMSFPayloadRole::ClusterRest;
    }
    if (payloadName == "vqmmc_codebook") {
        return PMSFPayloadRole::ConnectivityBlob;
    }
    if (payloadName == "vqmmc_indices") {
        return PMSFPayloadRole::Indices;
    }
    if (payloadName == "vqmmc_facet_symbols") {
        return PMSFPayloadRole::FacetSymbols;
    }
    if (payloadName == "vqmmc_edge_symbols") {
        return PMSFPayloadRole::EdgeSymbols;
    }
    if (codecName == "arrow.columnar" || codecName == "orc.columnar") {
        return PMSFPayloadRole::ColumnarContainer;
    }
    return PMSFPayloadRole::AlgorithmPayload;
}

inline std::string inferPMSFPayloadRoleName(
    const std::string& payloadName,
    const std::string& codecName = {}) {
    return paperPayloadRoleName(inferPMSFPayloadRole(payloadName, codecName));
}

struct PMSFPayloadDescriptor {
    PMSFPayloadRole role = PMSFPayloadRole::AlgorithmPayload;
    std::int32_t objectId = 0;
    std::int32_t partitionId = 0;
    std::int32_t lodId = 0;
    std::string payloadName;
    std::string codecName;
};

inline PMSFPayloadDescriptor makePMSFPayloadDescriptor(
    PMSFPayloadRole role,
    std::int32_t objectId,
    std::int32_t partitionId,
    std::int32_t lodId,
    const std::string& payloadName,
    const std::string& codecName) {
    PMSFPayloadDescriptor desc;
    desc.role = role;
    desc.objectId = objectId;
    desc.partitionId = partitionId;
    desc.lodId = lodId;
    desc.payloadName = payloadName;
    desc.codecName = codecName;
    return desc;
}

inline void setPMSFPayloadRecordRoleFromNameCodec(
    PMSFSymbolPayloadRecord& row) {
    if (!row.payloadRole.empty()) {
        return;
    }
    setPMSFSymbolPayloadRoleName(
        row, inferPMSFPayloadRoleName(row.payloadName, row.codecName));
}

inline PMSFPayloadDescriptor makePMSFPayloadDescriptor(
    PMSFPayloadRole role,
    const std::string& payloadName,
    const std::string& codecName) {
    return makePMSFPayloadDescriptor(
        role, 0, 0, 0, payloadName, codecName);
}

inline PMSFPayloadDescriptor makePMSFPayloadDescriptorWithInferredRole(
    std::int32_t objectId,
    std::int32_t partitionId,
    std::int32_t lodId,
    const std::string& payloadName,
    const std::string& codecName) {
    return makePMSFPayloadDescriptor(inferPMSFPayloadRole(payloadName, codecName),
                                      objectId,
                                      partitionId,
                                      lodId,
                                      payloadName,
                                      codecName);
}

inline PMSFPayloadDescriptor makePMSFPayloadDescriptorWithInferredRole(
    const std::string& payloadName,
    const std::string& codecName) {
    return makePMSFPayloadDescriptorWithInferredRole(
        0, 0, 0, payloadName, codecName);
}

inline PMSFSymbolPayloadSpec paperPayloadSpecFromDescriptor(
    const PMSFPayloadDescriptor& desc) {
    PMSFSymbolPayloadSpec spec = makePMSFSymbolPayloadSpec(desc.objectId,
                                                             desc.partitionId,
                                                             desc.lodId,
                                                             desc.payloadName,
                                                             desc.codecName);
    spec.payloadRoleName = paperPayloadRoleName(desc.role);
    return spec;
}

inline PMSFSymbolPayloadRecord paperPayloadRecordFromDescriptor(
    const PMSFPayloadDescriptor& desc,
    const void* data,
    std::size_t size) {
    PMSFSymbolPayloadRecord row = makePMSFPayloadRecord(
        paperPayloadSpecFromDescriptor(desc), data, size);
    setPMSFPayloadRecordRoleFromNameCodec(row);
    return row;
}

inline PMSFSymbolPayloadRecord paperPayloadRecordFromDescriptor(
    const PMSFPayloadDescriptor& desc,
    std::vector<std::uint8_t> payload) {
    PMSFSymbolPayloadRecord row = makePMSFPayloadRecord(
        paperPayloadSpecFromDescriptor(desc), std::move(payload));
    setPMSFPayloadRecordRoleFromNameCodec(row);
    return row;
}

inline PaperNamedBlob paperNamedBlobFromDescriptor(
    const PMSFPayloadDescriptor& desc,
    const void* data,
    std::size_t size) {
    return makePaperNamedBlob(paperPayloadSpecFromDescriptor(desc), data, size);
}

inline void appendPMSFPayloadRecordFromDescriptor(
    PMSFSymbolPayloadTable& rows,
    const PMSFPayloadDescriptor& desc,
    const void* data,
    std::size_t size) {
    appendPMSFPayloadRecord(rows, paperPayloadRecordFromDescriptor(desc, data, size));
}

inline void appendPMSFPayloadRecordFromDescriptor(
    PMSFSymbolPayloadTable& rows,
    const PMSFPayloadDescriptor& desc,
    std::vector<std::uint8_t> payload) {
    appendPMSFPayloadRecord(
        rows, paperPayloadRecordFromDescriptor(desc, std::move(payload)));
}

inline bool paperPayloadRecordMatchesRole(
    const PMSFSymbolPayloadRecord& row,
    PMSFPayloadRole role) {
    const std::string roleName =
        row.payloadRole.empty()
            ? inferPMSFPayloadRoleName(row.payloadName, row.codecName)
            : row.payloadRole;
    return roleName == paperPayloadRoleName(role);
}

inline const PMSFSymbolPayloadRecord* findPMSFPayloadByRole(
    const PMSFSymbolPayloadTable& rows,
    PMSFPayloadRole role,
    std::int32_t objectId = 0,
    std::int32_t partitionId = 0,
    std::int32_t lodId = 0) {
    for (const auto& row : rows) {
        if (row.objectId != objectId ||
            row.partitionId != partitionId ||
            row.lodId != lodId) {
            continue;
        }
        if (paperPayloadRecordMatchesRole(row, role)) {
            return &row;
        }
    }
    return nullptr;
}

inline const PMSFSymbolPayloadRecord* findPMSFPayloadByNameOrSafeRole(
    const PMSFSymbolPayloadTable& rows,
    std::int32_t objectId,
    std::int32_t partitionId,
    std::int32_t lodId,
    const std::string& payloadName) {
    if (const auto* row = findPMSFPayload(
            rows, objectId, partitionId, lodId, payloadName)) {
        return row;
    }
    const auto role = inferPMSFPayloadRole(payloadName);
    if (role == PMSFPayloadRole::AlgorithmPayload) {
        return nullptr;
    }
    return findPMSFPayloadByRole(
        rows, role, objectId, partitionId, lodId);
}

}  // namespace pmsf

#endif  // PMSF_STORAGE_PAPER_PAYLOAD_DESCRIPTOR_REGISTRY_H
