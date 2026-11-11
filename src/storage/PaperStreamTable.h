/*
 * @file PaperStreamTable.h
 * @brief PaperStream v1 storage-facing rows (M1 prototype).
 *
 * This header deliberately does NOT depend on MCGAL, Symbol.h or the codec
 * types.  It is the physical-storage contract: a writer/reader can be built
 * and tested without any algorithm.  The adapters that fill this table from
 * PMSF collectors live in the compress/decompress layer (M2+).
 *
 * Logical model (paper §3.5-§3.6):
 *   Object O          -> PaperObjectRow
 *   Partition M0^p    -> PaperPartitionRow (base mesh BLOB + seed + bbox + quant)
 *   LOD Delta_r^p     -> PaperLodRow (core streams + declared extension streams)
 *   Boundary          -> PaperBoundaryRow
 *
 * "Extension streams" are first-class named streams, not side channels: every
 * entry in PaperLodRow::extension is backed by a column declared in the
 * object's StreamDictionary.
 */
#ifndef PMSF_STORAGE_PAPER_STREAM_TABLE_H
#define PMSF_STORAGE_PAPER_STREAM_TABLE_H

#include <algorithm>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace pmsf {

enum class PaperRecordType : std::int8_t {
    Object = 0,
    Partition = 1,
    Lod = 2,
    Boundary = 3,
};

enum class PaperCellType : std::int8_t {
    Bytes = 0,
    Int32 = 1,
    Float = 2,
};

enum class PaperSchemaMode : std::int8_t {
    Typed = 0,
    Compact = 1,
};

enum class PaperStreamEncoding : std::int8_t {
    Plain = 0,
    BitPacked = 1,
    Rle = 2,
    Delta = 3,
    ByteStreamSplit = 4,
    Compact = 5,
};

inline const char* paperCellTypeName(PaperCellType t) {
    switch (t) {
        case PaperCellType::Bytes: return "bytes";
        case PaperCellType::Int32: return "int32";
        case PaperCellType::Float: return "float";
    }
    return "bytes";
}

/// @brief PaperStream v2 format version (M4-1 additive).
/// v1 stays the default: PaperObjectRow::formatVersion keeps its v1 default of 1
/// so existing writer/reader output is byte-identical. Only v2 consumers opt in
/// explicitly (dictionary v2 / codec v2 / tests).
inline constexpr std::int32_t kPaperStreamFormatVersionV2 = 2;

/// @brief v2 record types. v1 only uses Object/Partition/Lod/Boundary (=3).
/// Values 3/4 are reserved by v2 for standalone Connectivity/Geometry rows and
/// MUST NOT be emitted by the v1 compact writer (it keeps emitting 0/1/2/3).
enum class PaperRecordTypeV2 : std::int8_t {
    Object = 0,
    Partition = 1,
    Lod = 2,
    Connectivity = 3,
    Geometry = 4,
    Boundary = 5,
};

enum class PaperCellShape : std::int8_t {
    Scalar = 0,
    Vec3 = 1,
};

/// @brief Semantic relation of a declared stream (paper §3.6).
enum class PaperStreamRelation : std::int8_t {
    Connectivity = 0,
    Geometry = 1,
    Side = 2,
    Object = 3,
    Partition = 4,
};

/// @brief Where the bytes of a declared stream come from.
enum class PaperStreamSource : std::int8_t {
    Typed = 0,
    Native = 1,
    Payload = 2,
    BaseMesh = 3,
};

/// @brief How a row carries the algorithm payload.
enum class PaperPayloadMode : std::int8_t {
    Typed = 0,
    Native = 1,
    Mixed = 2,
};

/// @brief Physical framing of the native payload (design §3.4).
enum class PaperNativeFrame : std::int8_t {
    ConnRound = 0,
    ClassIStream = 1,
    Whole = 2,
    NamedPayload = 3,
};

enum class PaperPayloadScope : std::int8_t {
    Lod = 0,
    Partition = 1,
    Object = 2,
};

inline const char* paperCellShapeName(PaperCellShape s) {
    switch (s) {
        case PaperCellShape::Scalar: return "scalar";
        case PaperCellShape::Vec3: return "vec3";
    }
    return "scalar";
}

inline const char* paperStreamRelationName(PaperStreamRelation r) {
    switch (r) {
        case PaperStreamRelation::Connectivity: return "connectivity";
        case PaperStreamRelation::Geometry: return "geometry";
        case PaperStreamRelation::Side: return "side";
        case PaperStreamRelation::Object: return "object";
        case PaperStreamRelation::Partition: return "partition";
    }
    return "side";
}

inline const char* paperStreamSourceName(PaperStreamSource s) {
    switch (s) {
        case PaperStreamSource::Typed: return "typed";
        case PaperStreamSource::Native: return "native";
        case PaperStreamSource::Payload: return "payload";
        case PaperStreamSource::BaseMesh: return "base_mesh";
    }
    return "typed";
}

inline const char* paperPayloadModeName(PaperPayloadMode m) {
    switch (m) {
        case PaperPayloadMode::Typed: return "typed";
        case PaperPayloadMode::Native: return "native";
        case PaperPayloadMode::Mixed: return "mixed";
    }
    return "typed";
}

inline const char* paperNativeFrameName(PaperNativeFrame f) {
    switch (f) {
        case PaperNativeFrame::ConnRound: return "conn-round";
        case PaperNativeFrame::ClassIStream: return "class-i-stream";
        case PaperNativeFrame::Whole: return "whole";
        case PaperNativeFrame::NamedPayload: return "named-payload";
    }
    return "whole";
}

inline const char* paperPayloadScopeName(PaperPayloadScope s) {
    switch (s) {
        case PaperPayloadScope::Lod: return "lod";
        case PaperPayloadScope::Partition: return "partition";
        case PaperPayloadScope::Object: return "object";
    }
    return "partition";
}

namespace paper_v2_names {

inline bool paperCellShapeFromName(const std::string& name, PaperCellShape& out) {
    if (name == "scalar") { out = PaperCellShape::Scalar; return true; }
    if (name == "vec3") { out = PaperCellShape::Vec3; return true; }
    return false;
}

inline bool paperStreamRelationFromName(const std::string& name, PaperStreamRelation& out) {
    if (name == "connectivity") { out = PaperStreamRelation::Connectivity; return true; }
    if (name == "geometry") { out = PaperStreamRelation::Geometry; return true; }
    if (name == "side") { out = PaperStreamRelation::Side; return true; }
    if (name == "object") { out = PaperStreamRelation::Object; return true; }
    if (name == "partition") { out = PaperStreamRelation::Partition; return true; }
    return false;
}

inline bool paperStreamSourceFromName(const std::string& name, PaperStreamSource& out) {
    if (name == "typed") { out = PaperStreamSource::Typed; return true; }
    if (name == "native") { out = PaperStreamSource::Native; return true; }
    if (name == "payload") { out = PaperStreamSource::Payload; return true; }
    if (name == "base_mesh") { out = PaperStreamSource::BaseMesh; return true; }
    return false;
}

inline bool paperPayloadModeFromName(const std::string& name, PaperPayloadMode& out) {
    if (name == "typed") { out = PaperPayloadMode::Typed; return true; }
    if (name == "native") { out = PaperPayloadMode::Native; return true; }
    if (name == "mixed") { out = PaperPayloadMode::Mixed; return true; }
    return false;
}

inline bool paperNativeFrameFromName(const std::string& name, PaperNativeFrame& out) {
    if (name == "conn-round") { out = PaperNativeFrame::ConnRound; return true; }
    if (name == "class-i-stream") { out = PaperNativeFrame::ClassIStream; return true; }
    if (name == "whole") { out = PaperNativeFrame::Whole; return true; }
    if (name == "named-payload") { out = PaperNativeFrame::NamedPayload; return true; }
    return false;
}

inline bool paperPayloadScopeFromName(const std::string& name, PaperPayloadScope& out) {
    if (name == "lod") { out = PaperPayloadScope::Lod; return true; }
    if (name == "partition") { out = PaperPayloadScope::Partition; return true; }
    if (name == "object") { out = PaperPayloadScope::Object; return true; }
    return false;
}

}  // namespace paper_v2_names

inline const char* paperStreamEncodingName(PaperStreamEncoding e) {
    switch (e) {
        case PaperStreamEncoding::Plain: return "plain";
        case PaperStreamEncoding::BitPacked: return "bitpacked";
        case PaperStreamEncoding::Rle: return "rle";
        case PaperStreamEncoding::Delta: return "delta";
        case PaperStreamEncoding::ByteStreamSplit: return "bss";
        case PaperStreamEncoding::Compact: return "compact";
    }
    return "plain";
}

struct PaperStreamSpec {
    std::string name;
    PaperCellType cellType = PaperCellType::Bytes;
    PaperStreamEncoding encoding = PaperStreamEncoding::Plain;
    std::string codecName;  // "rangecoder", "zlib", "compact", ...
    std::int32_t alphabetBits = 0;
    bool perLod = true;

    // ---- v2 additions (appended so {name, cell, encoding, codec, alphabet,
    //      perLod} aggregate initializers stay valid) ----
    PaperCellShape shape = PaperCellShape::Scalar;
    PaperStreamRelation relation = PaperStreamRelation::Side;
    PaperStreamSource source = PaperStreamSource::Typed;
    bool optional = false;
};

/// @brief One byte range of the native payload occupied by a LOD/stream slice.
struct PaperPayloadSlice {
    std::int32_t lodId = -1;
    std::int32_t streamId = -1;  ///< -1 = all streams of that round concatenated
    std::uint64_t offset = 0;    ///< relative to the native payload blob start
    std::uint64_t length = 0;
};

/// @brief Parsed partition -> row-group directory (non-contiguous partition ids).
struct PaperPartitionDirectoryEntry {
    std::int32_t partitionId = -1;
    std::int32_t rgIndex = -1;
    std::int32_t rowIndex = 0;
    std::int32_t numLods = 0;
    std::int64_t payloadBytes = 0;
};

/// @brief Declaration of one named payload (codebook, cluster meta, ...).
struct PaperNamedPayloadSpec {
    std::string name;
    std::string role;
    std::string codec;
    PaperPayloadScope scope = PaperPayloadScope::Partition;
    std::uint64_t byteLength = 0;
};

struct PaperObjectRow {
    std::int32_t objectId = 0;
    std::int32_t numRounds = 0;
    std::int32_t totalVertices = 0;
    std::int32_t totalFaces = 0;
    std::int32_t numPartitions = 0;
    std::int32_t partitionsPerRowGroup = 1;
    bool enableQuantization = false;
    std::int32_t quantBits = 0;
    float bboxMin[3] = {0.f, 0.f, 0.f};
    float quantStep = 0.f;
    std::string algorithm;
    PaperSchemaMode schemaMode = PaperSchemaMode::Compact;
    std::int32_t formatVersion = 1;
    std::int32_t codecVersion = 0;
    std::vector<PaperStreamSpec> streams;

    // ---- v2 additions (ignored by the v1 bridge writer/reader) ----
    PaperPayloadMode payloadMode = PaperPayloadMode::Typed;
    PaperNativeFrame nativeFrame = PaperNativeFrame::Whole;
    std::vector<std::string> nativeStreamNames;  ///< CONN sub-stream logical names
    std::int32_t blobLayout = 0;
    std::int32_t schemeVersion = 0;
    bool pageIndex = false;
    std::int32_t lodsPerRowGroup = 0;  ///< 0 = all LODs of a partition in one RG
    std::vector<PaperPartitionDirectoryEntry> partitionRgDirectory;
    std::vector<PaperNamedPayloadSpec> namedPayloads;
    std::string baseMeshCodec = "compact";  ///< V2CP payload / partition codec id
    std::string boundaryCodec = "raw-int32";
    std::string raPrefixUnit = "lod";  ///< "lod" | "lod_block" | "partition"
};

struct PaperPartitionRow {
    std::int32_t objectId = 0;
    std::int32_t partitionId = 0;
    std::int32_t seedV0 = -1;
    std::int32_t seedV1 = -1;
    std::int32_t numVertices = 0;
    std::int32_t numFaces = 0;
    std::int32_t numLods = 0;
    bool enableQuantization = false;
    std::int32_t quantBits = 0;
    float bboxMin[3] = {0.f, 0.f, 0.f};
    float quantStep = 0.f;
    std::vector<std::uint8_t> baseMesh;
    std::string baseMeshCodec;

    // ---- v2 additions (ignored by the v1 bridge writer/reader) ----
    std::vector<std::uint8_t> nativePayload;  ///< whole/named-payload partition blob
    std::vector<std::uint8_t> connHeader;     ///< CONN header copy (conn-round)
    std::vector<PaperPayloadSlice> payloadDirectory;  ///< conn-round per-LOD ranges
};

struct PaperLodRow {
    std::int32_t objectId = 0;
    std::int32_t partitionId = 0;
    std::int32_t lodId = 0;

    // Core streams.  F/Et/Ef/V are byte streams in M1 (bit-packed or a native
    // codec); Cf/Cv are split into quantized int32 and float variants so the
    // same schema can carry Q12 and F32 without a type switch.
    std::vector<std::uint8_t> F;
    std::vector<std::uint8_t> Et;
    std::vector<std::uint8_t> Ef;
    std::vector<std::uint8_t> V;
    std::vector<std::int32_t> CfQ;  // flattened xyz
    std::vector<std::int32_t> CvQ;  // flattened xyz
    std::vector<float> CfF;         // flattened xyz
    std::vector<float> CvF;         // flattened xyz

    // Declared extension streams.  M1 stores them as BINARY; later revisions
    // may switch a stream to a typed column without changing the dictionary
    // contract.
    std::map<std::string, std::vector<std::uint8_t>> extension;

    std::int32_t countF = 0;
    std::int32_t countEt = 0;
    std::int32_t countEf = 0;
    std::int32_t countV = 0;
    std::int32_t countCf = 0;
    std::int32_t countCv = 0;

    // ---- v2 additions (ignored by the v1 bridge writer/reader) ----
    std::vector<std::uint8_t> nativePayload;  ///< conn-round/class-i LOD slice bytes
    std::uint64_t payloadByteOffset = 0;      ///< absolute offset in partition payload
    std::uint64_t payloadByteLength = 0;      ///< == nativePayload.size() when set
};

struct PaperBoundaryRow {
    std::int32_t objectId = 0;
    std::int32_t partitionA = 0;
    std::int32_t partitionB = 0;
    std::vector<std::int32_t> sharedVertices;
    std::vector<std::int32_t> sharedEdges;
    std::vector<std::int32_t> stopVertices;
    std::string codec = "raw-int32";  ///< v2 addition; v1 writers leave it default
};

struct PaperStreamTable {
    PaperObjectRow object;
    std::vector<PaperPartitionRow> partitions;
    std::vector<PaperLodRow> lods;
    std::vector<PaperBoundaryRow> boundaries;
    /// @brief M2b pilot bridge only: raw legacy artifact files (name -> bytes).
    /// The final M2c decoder reads base/header/symbols natively; this map is
    /// never part of the paper-storage contract and is removed after the pilot.
    std::map<std::string, std::vector<std::uint8_t>> legacyFiles;
};

// ---------------------------------------------------------------------------
// v2 helpers (M4-1 additive; no v1 behavior changes)
// ---------------------------------------------------------------------------

/// @brief Map an encoder/collector round (fine->coarse) to the v2 paper
///        `lod_id` (decoder order, coarse->fine).
inline std::int32_t paperLodIdFromEncoderRound(std::int32_t encoderRound,
                                               std::int32_t numRounds) {
    return numRounds - 1 - encoderRound;
}

/// @brief Inverse of paperLodIdFromEncoderRound.
inline std::int32_t paperEncoderRoundFromLodId(std::int32_t lodId,
                                               std::int32_t numRounds) {
    return numRounds - 1 - lodId;
}

/// @brief Validate the v2 invariants of a PaperStreamTable.
///
/// Only v2 consumers call this: v1 tables/tests/writer/reader never do, so the
/// v1 pipeline keeps its exact behavior. `expectedFormatVersion` defaults to 2;
/// pass v1's 1 if validating a v1 table on purpose.
inline bool paperValidateStreamTable(
    const PaperStreamTable& table, std::string* error = nullptr,
    std::int32_t expectedFormatVersion = kPaperStreamFormatVersionV2) {
    auto fail = [&](const std::string& message) {
        if (error) *error = message;
        return false;
    };

    if (table.object.formatVersion != expectedFormatVersion) {
        return fail("unsupported PaperStream formatVersion=" +
                    std::to_string(table.object.formatVersion) + " (expected " +
                    std::to_string(expectedFormatVersion) + ")");
    }
    if (table.partitions.empty()) {
        return fail("at least one partition is required");
    }
    if (table.object.numPartitions != 0 &&
        table.object.numPartitions != static_cast<std::int32_t>(table.partitions.size())) {
        return fail("object.numPartitions=" +
                    std::to_string(table.object.numPartitions) + " but partitions=" +
                    std::to_string(table.partitions.size()));
    }

    std::set<std::string> declaredStreams;
    for (const auto& spec : table.object.streams) {
        if (spec.name.empty()) return fail("declared stream with empty name");
        if (!declaredStreams.insert(spec.name).second) {
            return fail("duplicate declared stream: " + spec.name);
        }
    }
    if (table.object.raPrefixUnit != "lod" && table.object.raPrefixUnit != "lod_block" &&
        table.object.raPrefixUnit != "partition") {
        return fail("invalid ra.prefix_unit: " + table.object.raPrefixUnit);
    }

    std::set<std::int32_t> partitionIds;
    std::map<std::int32_t, const PaperPartitionRow*> partitionById;
    for (const auto& partition : table.partitions) {
        if (partition.partitionId < 0) {
            return fail("negative partition_id=" + std::to_string(partition.partitionId));
        }
        if (!partitionIds.insert(partition.partitionId).second) {
            return fail("duplicate partition_id=" + std::to_string(partition.partitionId));
        }
        partitionById[partition.partitionId] = &partition;
    }

    std::map<std::int32_t, std::vector<const PaperLodRow*>> lodsByPartition;
    for (const auto& lod : table.lods) {
        if (partitionById.find(lod.partitionId) == partitionById.end()) {
            return fail("LOD references unknown partition_id=" +
                        std::to_string(lod.partitionId));
        }
        lodsByPartition[lod.partitionId].push_back(&lod);
    }
    for (auto& [partitionId, rows] : lodsByPartition) {
        std::stable_sort(rows.begin(), rows.end(),
                         [](const PaperLodRow* a, const PaperLodRow* b) {
                             return a->lodId < b->lodId;
                         });
        for (std::size_t i = 0; i < rows.size(); ++i) {
            if (rows[i]->lodId != static_cast<std::int32_t>(i)) {
                return fail("partition " + std::to_string(partitionId) +
                            " has non-contiguous lod_id=" +
                            std::to_string(rows[i]->lodId) + " at index " +
                            std::to_string(i) +
                            " (lod_id must be 0..numLods-1 in decode order)");
            }
        }
        if (partitionById.at(partitionId)->numLods !=
            static_cast<std::int32_t>(rows.size())) {
            return fail("partition " + std::to_string(partitionId) + " numLods=" +
                        std::to_string(partitionById.at(partitionId)->numLods) +
                        " but rows=" + std::to_string(rows.size()));
        }
    }

    const PaperPayloadMode mode = table.object.payloadMode;
    const PaperNativeFrame frame = table.object.nativeFrame;
    for (const auto& partition : table.partitions) {
        const auto it = lodsByPartition.find(partition.partitionId);
        const std::vector<const PaperLodRow*> emptyRows;
        const auto& rows = (it == lodsByPartition.end()) ? emptyRows : it->second;

        const bool hasPartitionNative = !partition.nativePayload.empty();
        std::size_t nativeLodBytes = 0;
        for (const auto* lod : rows) nativeLodBytes += lod->nativePayload.size();
        const bool hasAnyNative = hasPartitionNative || nativeLodBytes > 0;

        if (mode == PaperPayloadMode::Typed) {
            if (hasAnyNative) {
                return fail("partition " + std::to_string(partition.partitionId) +
                            " is Typed but carries native payload bytes");
            }
            if (!partition.payloadDirectory.empty()) {
                return fail("partition " + std::to_string(partition.partitionId) +
                            " is Typed but has a payload directory");
            }
        } else if (mode == PaperPayloadMode::Native) {
            if (!hasAnyNative && !rows.empty()) {
                return fail("partition " + std::to_string(partition.partitionId) +
                            " is Native but has no native payload");
            }
            for (const auto* lod : rows) {
                if (!lod->F.empty() || !lod->Et.empty() || !lod->Ef.empty() ||
                    !lod->V.empty() || !lod->CfQ.empty() || !lod->CvQ.empty() ||
                    !lod->CfF.empty() || !lod->CvF.empty() || !lod->extension.empty()) {
                    return fail("partition " + std::to_string(partition.partitionId) +
                                " lod " + std::to_string(lod->lodId) +
                                " is Native but carries typed core/extension streams");
                }
            }
        } else {  // Mixed
            if (!hasAnyNative) {
                return fail("partition " + std::to_string(partition.partitionId) +
                            " is Mixed but has no native payload");
            }
            bool hasTyped = false;
            for (const auto* lod : rows) {
                if (!lod->F.empty() || !lod->Et.empty() || !lod->Ef.empty() ||
                    !lod->V.empty() || !lod->CfQ.empty() || !lod->CvQ.empty() ||
                    !lod->CfF.empty() || !lod->CvF.empty() || !lod->extension.empty()) {
                    hasTyped = true;
                    break;
                }
            }
            if (!hasTyped) {
                return fail("partition " + std::to_string(partition.partitionId) +
                            " is Mixed but has no typed streams");
            }
        }

        for (const auto* lod : rows) {
            for (const auto& entry : lod->extension) {
                if (declaredStreams.find(entry.first) == declaredStreams.end()) {
                    return fail("partition " + std::to_string(partition.partitionId) +
                                " lod " + std::to_string(lod->lodId) +
                                " extension stream is not declared: " + entry.first);
                }
            }
        }

        if (mode != PaperPayloadMode::Typed && frame == PaperNativeFrame::ConnRound &&
            !partition.payloadDirectory.empty()) {
            if (partition.payloadDirectory.size() != rows.size()) {
                return fail("partition " + std::to_string(partition.partitionId) +
                            " payloadDirectory entries=" +
                            std::to_string(partition.payloadDirectory.size()) +
                            " but lods=" + std::to_string(rows.size()));
            }
            std::vector<PaperPayloadSlice> directory = partition.payloadDirectory;
            std::stable_sort(directory.begin(), directory.end(),
                             [](const PaperPayloadSlice& a, const PaperPayloadSlice& b) {
                                 if (a.lodId != b.lodId) return a.lodId < b.lodId;
                                 return a.streamId < b.streamId;
                             });
            std::uint64_t cursor = 0;
            for (std::size_t i = 0; i < directory.size(); ++i) {
                if (directory[i].lodId != static_cast<std::int32_t>(i)) {
                    return fail("partition " + std::to_string(partition.partitionId) +
                                " payloadDirectory non-contiguous lodId=" +
                                std::to_string(directory[i].lodId));
                }
                if (i > 0 && directory[i].offset < cursor) {
                    return fail("partition " + std::to_string(partition.partitionId) +
                                " payloadDirectory offsets overlap at lod " +
                                std::to_string(directory[i].lodId));
                }
                cursor = directory[i].offset + directory[i].length;
                const PaperLodRow* lod = rows[i];
                if (lod->payloadByteLength != 0 &&
                    lod->payloadByteLength != directory[i].length) {
                    return fail("partition " + std::to_string(partition.partitionId) +
                                " lod " + std::to_string(lod->lodId) +
                                " native payload length mismatch");
                }
                if (lod->payloadByteLength != 0 &&
                    lod->payloadByteOffset != directory[i].offset) {
                    return fail("partition " + std::to_string(partition.partitionId) +
                                " lod " + std::to_string(lod->lodId) +
                                " native payload offset mismatch");
                }
            }
        }
    }

    for (const auto& boundary : table.boundaries) {
        if (partitionById.find(boundary.partitionA) == partitionById.end() ||
            partitionById.find(boundary.partitionB) == partitionById.end()) {
            return fail("boundary references unknown partition (" +
                        std::to_string(boundary.partitionA) + ", " +
                        std::to_string(boundary.partitionB) + ")");
        }
        if (boundary.partitionA == boundary.partitionB) {
            return fail("boundary with partitionA == partitionB == " +
                        std::to_string(boundary.partitionA));
        }
    }

    if (!table.object.partitionRgDirectory.empty()) {
        if (table.object.partitionRgDirectory.size() != table.partitions.size()) {
            return fail("partitionRgDirectory entries=" +
                        std::to_string(table.object.partitionRgDirectory.size()) +
                        " but partitions=" + std::to_string(table.partitions.size()));
        }
        std::set<std::int32_t> seen;
        std::int32_t previousRg = -1;
        for (const auto& entry : table.object.partitionRgDirectory) {
            if (partitionById.find(entry.partitionId) == partitionById.end()) {
                return fail("partitionRgDirectory references unknown partition_id=" +
                            std::to_string(entry.partitionId));
            }
            if (!seen.insert(entry.partitionId).second) {
                return fail("duplicate partitionRgDirectory entry for partition " +
                            std::to_string(entry.partitionId));
            }
            if (entry.rgIndex < 0 || entry.rgIndex < previousRg) {
                return fail("partitionRgDirectory rgIndex is not monotone at partition " +
                            std::to_string(entry.partitionId));
            }
            previousRg = entry.rgIndex;
            if (entry.numLods != partitionById.at(entry.partitionId)->numLods) {
                return fail("partitionRgDirectory numLods mismatch for partition " +
                            std::to_string(entry.partitionId));
            }
        }
        if (seen.size() != table.partitions.size()) {
            return fail("partitionRgDirectory does not cover every partition");
        }
    }

    if (error) error->clear();
    return true;
}

}  // namespace pmsf

#endif  // PMSF_STORAGE_PAPER_STREAM_TABLE_H
