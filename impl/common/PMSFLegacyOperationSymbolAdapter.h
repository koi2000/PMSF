#ifndef PMSF_STORAGE_LEGACY_OPERATION_PAPER_SYMBOL_ADAPTER_H
#define PMSF_STORAGE_LEGACY_OPERATION_PAPER_SYMBOL_ADAPTER_H

#include "Point.h"
#include "common/PMSFStorageScopeNames.h"
#include "core/PMSFBoundarySideChannel.h"
#include "core/PMSFSideChannel.h"
#include "storage/PMSFPayloadDescriptorRegistry.h"
#include "storage/PMSFStorageContextSideChannel.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pmsf {

inline constexpr const char* kPomarLegacyPayloadName = "pomar_legacy_payload";
inline constexpr const char* kPomarLegacyPayloadCodecName = "pomar.legacy";
inline constexpr const char* kSegEdgeCollapseLegacyPayloadName =
    "seg_edge_collapse_payload";
inline constexpr const char* kSegEdgeCollapseLegacyPayloadCodecName =
    "seg_edge_collapse.legacy";

inline PMSFPayloadDescriptor pomarLegacyPayloadDescriptor() {
    return makePMSFPayloadDescriptor(PMSFPayloadRole::OperationStream,
                                      kPomarLegacyPayloadName,
                                      kPomarLegacyPayloadCodecName);
}

inline PMSFSymbolPayloadSpec pomarLegacyPayloadSpec() {
    return paperPayloadSpecFromDescriptor(pomarLegacyPayloadDescriptor());
}

inline PMSFPayloadDescriptor segEdgeCollapseLegacyPayloadDescriptor() {
    return makePMSFPayloadDescriptor(
        PMSFPayloadRole::OperationStream,
        kSegEdgeCollapseLegacyPayloadName,
        kSegEdgeCollapseLegacyPayloadCodecName);
}

inline PMSFSymbolPayloadSpec segEdgeCollapseLegacyPayloadSpec() {
    return paperPayloadSpecFromDescriptor(
        segEdgeCollapseLegacyPayloadDescriptor());
}

inline bool legacyOperationEnvOn(const char* name) {
    return pmsfStorageEnvEnabled(name);
}

inline bool legacyOperationPaperExactRequired(const std::string& prefix) {
    if (legacyOperationEnvOn("PMSF_LEGACY_OPERATION_REQUIRE_PAPER_EXACT")) {
        return true;
    }
    if (prefix == kPMSFStorageScopePomarLegacy &&
        legacyOperationEnvOn("PMSF_POMAR_REQUIRE_PAPER_EXACT")) {
        return true;
    }
    if (prefix == "seg_edge_collapse" &&
        legacyOperationEnvOn("PMSF_SEG_EDGE_COLLAPSE_REQUIRE_PAPER_EXACT")) {
        return true;
    }
    return false;
}

inline void requireLegacyOperationPaperExactIfRequested(
    const std::string& prefix, const char* scope) {
    if (!legacyOperationPaperExactRequired(prefix)) {
        return;
    }
    throw std::runtime_error(
        std::string(scope) +
        " requested paper-exact legacy operation replay, but this path only "
        "maps split/skip/locator/residual queues into the paper relation and "
        "does not prove full paper-exact reconstruction semantics.");
}

inline PMSFSymbolPayloadTable buildLegacyOperationPayloadTable(
    const std::string& payloadName,
    const std::string& codecName,
    const void* data,
    std::size_t size,
    std::int32_t objectId = 0,
    std::int32_t partitionId = 0,
    std::int32_t lodId = 0) {
    PMSFSymbolPayloadTable payloadRows;
    appendPMSFPayloadRecordFromDescriptor(
        payloadRows,
        makePMSFPayloadDescriptor(PMSFPayloadRole::OperationStream,
                                   objectId,
                                   partitionId,
                                   lodId,
                                   payloadName,
                                   codecName),
        data,
        size);
    return payloadRows;
}

inline PMSFSymbolPayloadTable buildLegacyOperationPayloadTable(
    const PMSFSymbolPayloadSpec& spec,
    const void* data,
    std::size_t size) {
    PMSFSymbolPayloadTable payloadRows;
    appendPMSFPayloadRecord(payloadRows, spec, data, size);
    return payloadRows;
}

inline PMSFSymbolTable buildLegacyOperationPMSFSymbolTable(
    const std::string& prefix,
    int nbLODs,
    int nbClusters,
    int nbGlobalLOD,
    int nbClusteredLOD,
    const std::vector<std::deque<unsigned>>& nbSplitSymbols,
    const std::vector<std::deque<unsigned>>& nbSkipsHehSymbols,
    const std::vector<std::deque<unsigned>>& vertexSplitSymbols,
    const std::vector<std::deque<int>>& geometrySymbols,
    const std::vector<std::deque<int>>& geometryNormalSymbols,
    const std::vector<std::pair<std::string, std::int32_t>>& emptyExtras = {},
    std::string exactPrefix = {},
    bool directResidual = false,
    bool includeResidualComponents = false) {
    PMSFSymbolTable table;
    const std::size_t clusters = nbSplitSymbols.size();
    table.reserve(clusters == 0 ? 1 : clusters);
    const std::string& paperExactPrefix =
        exactPrefix.empty() ? prefix : exactPrefix;

    for (std::size_t cluster = 0; cluster < clusters; ++cluster) {
        PMSFSymbolRecord& row =
            appendPMSFSymbolRecord(table, 0, static_cast<std::int32_t>(cluster), 0);
        appendPMSFStorageContextMapping(row.delta, prefix, 0, 1);
        appendPMSFBoundaryAbsentExactPair(
            row.delta,
            paperExactPrefix,
            "paper",
            legacyOperationPaperExactRequired(paperExactPrefix) ? 1 : 0);
        appendPaperI32SideChannel(row.delta, prefix, "nb_lods", static_cast<std::int32_t>(nbLODs));
        appendPaperI32SideChannel(row.delta, prefix, "cluster_count", static_cast<std::int32_t>(nbClusters));
        appendPaperI32SideChannel(row.delta, prefix, "global_lods", static_cast<std::int32_t>(nbGlobalLOD));
        appendPaperI32SideChannel(row.delta, prefix, "clustered_lods", static_cast<std::int32_t>(nbClusteredLOD));

        if (cluster < nbSplitSymbols.size()) {
            for (unsigned v : nbSplitSymbols[cluster]) {
                appendPaperScalarSymbolForRole(
                    row.delta,
                    PMSFSymbolRole::Facet,
                    static_cast<std::int32_t>(v));
                appendPaperI32SideChannel(row.delta, prefix, "nb_split", static_cast<std::int32_t>(v));
            }
        }
        if (cluster < nbSkipsHehSymbols.size()) {
            for (unsigned v : nbSkipsHehSymbols[cluster]) {
                appendPaperScalarSymbolForRole(
                    row.delta,
                    PMSFSymbolRole::TriangulationEdge,
                    static_cast<std::int32_t>(v));
                appendPaperI32SideChannel(row.delta, prefix, "nb_skips_heh", static_cast<std::int32_t>(v));
            }
        }
        if (cluster < vertexSplitSymbols.size()) {
            for (std::size_t i = 0; i < vertexSplitSymbols[cluster].size(); ++i) {
                const unsigned v = vertexSplitSymbols[cluster][i];
                appendPaperScalarSymbolForRole(
                    row.delta,
                    PMSFSymbolRole::Vertex,
                    static_cast<std::int32_t>(v));
                appendPaperI32SideChannel(row.delta, prefix, "vertex_split", static_cast<std::int32_t>(v));
                appendPaperI32SideChannel(row.delta,
                    prefix,
                    (i & 1u) == 0u
                        ? "vertex_split_left_offset"
                        : "vertex_split_right_offset",
                    static_cast<std::int32_t>(v));
            }
        }
        if (cluster < geometrySymbols.size()) {
            for (std::size_t i = 0; i < geometrySymbols[cluster].size(); ++i) {
                const int v = geometrySymbols[cluster][i];
                appendPaperI32SideChannel(row.delta, prefix, "geometry", static_cast<std::int32_t>(v));
                if (includeResidualComponents) {
                    appendPaperI32SideChannel(row.delta,
                        prefix,
                        (i & 1u) == 0u
                            ? "residual_component0"
                            : "residual_component1",
                        static_cast<std::int32_t>(v));
                }
                appendPaperI32SideChannel(row.delta,
                    prefix,
                    directResidual
                        ? ((i & 1u) == 0u ? "residual_x" : "residual_y")
                        : ((i & 1u) == 0u
                               ? "residual_tangent_u"
                               : "residual_tangent_v"),
                    static_cast<std::int32_t>(v));
            }
        }
        if (cluster < geometryNormalSymbols.size()) {
            for (int v : geometryNormalSymbols[cluster]) {
                appendPaperI32SideChannel(row.delta, prefix, "geometry_normal", static_cast<std::int32_t>(v));
                if (includeResidualComponents) {
                    appendPaperI32SideChannel(row.delta,
                        prefix,
                        "residual_component2",
                        static_cast<std::int32_t>(v));
                }
                appendPaperI32SideChannel(row.delta,
                    prefix,
                    directResidual ? "residual_z" : "residual_normal",
                    static_cast<std::int32_t>(v));
            }
        }
        if (cluster < geometrySymbols.size() &&
            cluster < geometryNormalSymbols.size()) {
            const std::size_t residualCount = std::min(
                geometrySymbols[cluster].size() / 2,
                geometryNormalSymbols[cluster].size());
            for (std::size_t i = 0; i < residualCount; ++i) {
                appendPaperCoordinateForRole(
                    row.delta,
                    PMSFSymbolRole::RemovedVertexCoord,
                    static_cast<double>(geometrySymbols[cluster][2 * i]),
                    static_cast<double>(geometrySymbols[cluster][2 * i + 1]),
                    static_cast<double>(geometryNormalSymbols[cluster][i]));
            }
        }
    }

    if (table.empty()) {
        PMSFSymbolRecord& row = appendPMSFSymbolRecord(table, 0, 0, 0);
        appendPMSFStorageContextMapping(row.delta, prefix, 1, 0);
        appendPMSFBoundaryAbsentExactPair(
            row.delta,
            paperExactPrefix,
            "paper",
            legacyOperationPaperExactRequired(paperExactPrefix) ? 1 : 0);
        for (const auto& extra : emptyExtras) {
            appendPaperI32SideChannel(row.delta, extra.first, extra.second);
        }
    }

    return table;
}

}  // namespace pmsf

#endif  // PMSF_STORAGE_LEGACY_OPERATION_PAPER_SYMBOL_ADAPTER_H
