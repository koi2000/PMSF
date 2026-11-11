/*
 * @file PaperBridgeV2.h
 * @brief Minimal opt-in PaperStream v2 bridge helpers (M4-3).
 *
 * ADDITIVE: the v1 PaperSerializeOperator/PaperDeserializeOperator behavior is
 * unchanged when the switch is off. The switch is a single environment variable:
 *
 *   PMSF_PAPERSTREAM_V2=1   -> PaperSerializeOperator writes a v2 artifact with
 *                              PaperStreamWriterV2; PaperDeserializeOperator
 *                              dispatches v2 artifacts to PaperStreamReaderV2.
 *   unset / "0"             -> original v1 bridge path (byte-identical output).
 *
 * M4-3 only wires the storage reader/writer into the operator bridge. The
 * complete native v2 round driver (base mesh + per-round symbol consumption) is
 * M4-4: the deserializer v2 path deliberately fails closed on
 * deserializeBaseMesh/deserializeOneRound with an explicit M4-4 message.
 */
#ifndef PMSF_STORAGE_PAPER_BRIDGE_V2_H
#define PMSF_STORAGE_PAPER_BRIDGE_V2_H

#include "storage/PaperStreamReaderV2.h"
#include "storage/PaperStreamTable.h"
#include "storage/PaperTypedLodDirectory.h"
#include "storage/PaperStreamWriterV2.h"

#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>

namespace pmsf {

/// @brief Opt-in bridge switch (`PMSF_PAPERSTREAM_V2=1`); default OFF.
inline bool paperStreamV2Enabled() {
    const char* value = std::getenv("PMSF_PAPERSTREAM_V2");
    return value != nullptr && *value != '\0' && std::strcmp(value, "0") != 0;
}

/// @brief True when the artifact's object-row dictionary says version 2.
/// @throws std::runtime_error for unreadable/corrupt artifacts (fail-closed).
inline bool isV2PaperArtifact(const std::string& path) {
    PaperStreamReaderV2 probe(path);
    return !probe.usedV1Fallback();
}

/// @brief Read a v2 artifact through PaperStreamReaderV2.
/// @param maxLod < 0 = read all; otherwise readPrefix(partitionId, maxLod).
inline PaperStreamTable readPaperStreamV2Table(const std::string& path, int maxLod,
                                               int partitionId = 0) {
    PaperStreamReaderV2 reader(path);
    if (reader.usedV1Fallback()) {
        throw std::runtime_error(
            "readPaperStreamV2Table: artifact is PaperStream v1, not v2: " + path);
    }
    if (maxLod >= 0) return reader.readPrefix(partitionId, maxLod);
    return reader.readAll();
}

/// @brief Finalize a bridge-built typed table as a PaperStream v2 table:
///        formatVersion=2, typed compact payload, decode-order lod_id, and full
///        v2 contract validation.
/// @throws std::runtime_error when the table violates the v2 contract.
inline void finalizePaperStreamV2Table(PaperStreamTable& table,
                                       std::int32_t blobLayout) {
    table.object.formatVersion = kPaperStreamFormatVersionV2;
    table.object.schemaMode = PaperSchemaMode::Compact;
    table.object.payloadMode = PaperPayloadMode::Typed;
    table.object.nativeFrame = PaperNativeFrame::Whole;
    table.object.blobLayout = blobLayout;
    table.object.schemeVersion = 0;
    table.object.raPrefixUnit = "partition";
    if (table.object.baseMeshCodec.empty() && !table.partitions.empty()) {
        table.object.baseMeshCodec = table.partitions.front().baseMeshCodec;
    }
    const std::int32_t rounds =
        table.object.numRounds > 0 ? table.object.numRounds
                                   : static_cast<std::int32_t>(table.lods.size());
    table.object.numRounds = rounds;
    // Bridge collectors expose encoder order (fine->coarse); the paper contract
    // stores decoder order.
    PaperStreamWriterV2::assignDecodeOrderLodIds(table.lods, rounds);
    assignPaperTypedLodDirectory(table);
    std::string error;
    if (!paperValidateStreamTable(table, &error)) {
        throw std::runtime_error("finalizePaperStreamV2Table: invalid v2 table: " + error);
    }
}

}  // namespace pmsf

#endif  // PMSF_STORAGE_PAPER_BRIDGE_V2_H
