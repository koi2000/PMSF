/*
 * @file PaperStreamDictionaryV2.h
 * @brief PaperStream v2 stream dictionary (M4-1, additive).
 *
 * ADDITIVE ONLY: storage/PaperStreamDictionary.h (v1) is untouched because the
 * M2b bridge writer/reader still use it. This header adds:
 *   serializePaperStreamDictionaryV2(object, error)
 *   parsePaperStreamDictionaryV2(text, object, error)
 *
 * v2 fields (PHASE2_DESIGN §3.5):
 *   format/version=2/schema/algorithm/object scalars/codec_version/
 *   blob_layout/scheme_version/payload_mode/native_payload{frame,stream_names}/
 *   base_mesh_codec/quant/lod_order=decode/layout{partition directory}/
 *   streams[relation,cell,shape,encoding,codec,alphabet_bits,per_lod,source,
 *           optional]/payloads[]/boundary{codec}/ra{prefix_unit}.
 *
 * Determinism: object keys come from nlohmann::json (ordered map); all arrays
 * are sorted by name/id before emission, so the same table gives byte-identical
 * JSON across runs.
 *
 * Fail-closed: unknown format/version/schema/payload_mode/native frame/
 * relation/source/cell/shape/encoding/ra prefix unit, duplicate or empty names,
 * missing required arrays and type mismatches all return false with an error.
 * v1 dictionaries are rejected with an explicit "recompress" hint.
 */
#ifndef PMSF_STORAGE_PAPER_STREAM_DICTIONARY_V2_H
#define PMSF_STORAGE_PAPER_STREAM_DICTIONARY_V2_H

#include "storage/PaperStreamTable.h"

#include "json.hpp"

#include <algorithm>
#include <cstdint>
#include <set>
#include <string>
#include <vector>

namespace pmsf {
namespace paper_dictionary_v2_detail {

inline const std::string& paperStreamDictionaryV2Format() {
    static const std::string kFormat = "pmsf-paperstream";
    return kFormat;
}

inline std::vector<PaperStreamSpec> sortedStreams(
    const std::vector<PaperStreamSpec>& streams) {
    std::vector<PaperStreamSpec> out = streams;
    std::stable_sort(out.begin(), out.end(),
                     [](const PaperStreamSpec& a, const PaperStreamSpec& b) {
                         return a.name < b.name;
                     });
    return out;
}

inline std::vector<PaperNamedPayloadSpec> sortedPayloads(
    const std::vector<PaperNamedPayloadSpec>& payloads) {
    std::vector<PaperNamedPayloadSpec> out = payloads;
    std::stable_sort(out.begin(), out.end(),
                     [](const PaperNamedPayloadSpec& a, const PaperNamedPayloadSpec& b) {
                         return a.name < b.name;
                     });
    return out;
}

inline std::vector<PaperPartitionDirectoryEntry> sortedDirectory(
    const std::vector<PaperPartitionDirectoryEntry>& entries) {
    std::vector<PaperPartitionDirectoryEntry> out = entries;
    std::stable_sort(out.begin(), out.end(),
                     [](const PaperPartitionDirectoryEntry& a,
                        const PaperPartitionDirectoryEntry& b) {
                         return a.partitionId < b.partitionId;
                     });
    return out;
}

inline bool cellTypeFromNameStrict(const std::string& name, PaperCellType& out) {
    if (name == "bytes") { out = PaperCellType::Bytes; return true; }
    if (name == "int32") { out = PaperCellType::Int32; return true; }
    if (name == "float") { out = PaperCellType::Float; return true; }
    return false;
}

inline bool encodingFromNameStrict(const std::string& name, PaperStreamEncoding& out) {
    if (name == "plain") { out = PaperStreamEncoding::Plain; return true; }
    if (name == "bitpacked") { out = PaperStreamEncoding::BitPacked; return true; }
    if (name == "rle") { out = PaperStreamEncoding::Rle; return true; }
    if (name == "delta") { out = PaperStreamEncoding::Delta; return true; }
    if (name == "bss") { out = PaperStreamEncoding::ByteStreamSplit; return true; }
    if (name == "compact") { out = PaperStreamEncoding::Compact; return true; }
    return false;
}

}  // namespace paper_dictionary_v2_detail

/**
 * @brief Serialize the v2 stream dictionary (deterministic JSON text).
 * @return empty string on invalid descriptor values, with `error` filled.
 */
inline std::string serializePaperStreamDictionaryV2(const PaperObjectRow& object,
                                                    std::string* error = nullptr) {
    using namespace paper_dictionary_v2_detail;
    auto fail = [&](const std::string& message) {
        if (error) *error = message;
        return std::string();
    };

    std::set<std::string> seenStreams;
    for (const auto& spec : object.streams) {
        if (spec.name.empty()) return fail("empty stream name");
        if (!seenStreams.insert(spec.name).second) {
            return fail("duplicate stream name: " + spec.name);
        }
    }
    std::set<std::string> seenPayloads;
    for (const auto& payload : object.namedPayloads) {
        if (payload.name.empty()) return fail("empty named payload name");
        if (!seenPayloads.insert(payload.name).second) {
            return fail("duplicate named payload: " + payload.name);
        }
    }
    if (object.raPrefixUnit != "lod" && object.raPrefixUnit != "lod_block" &&
        object.raPrefixUnit != "partition") {
        return fail("invalid ra.prefix_unit: " + object.raPrefixUnit);
    }
    if (object.partitionsPerRowGroup < 1) return fail("partitionsPerRowGroup must be >= 1");
    if (object.lodsPerRowGroup < 0) return fail("lodsPerRowGroup must be >= 0");

    nlohmann::json j;
    j["format"] = paperStreamDictionaryV2Format();
    j["version"] = kPaperStreamFormatVersionV2;
    j["schema"] = object.schemaMode == PaperSchemaMode::Compact ? "compact" : "typed";
    j["algorithm"] = object.algorithm;
    j["object_id"] = object.objectId;
    j["num_rounds"] = object.numRounds;
    j["total_vertices"] = object.totalVertices;
    j["total_faces"] = object.totalFaces;
    j["num_partitions"] = object.numPartitions;
    j["codec_version"] = object.codecVersion;
    j["blob_layout"] = object.blobLayout;
    j["scheme_version"] = object.schemeVersion;
    j["payload_mode"] = paperPayloadModeName(object.payloadMode);
    {
        nlohmann::json native;
        native["frame"] = paperNativeFrameName(object.nativeFrame);
        native["stream_names"] = nlohmann::json::array();
        for (const auto& name : object.nativeStreamNames)
            native["stream_names"].push_back(name);
        j["native_payload"] = std::move(native);
    }
    j["base_mesh_codec"] =
        object.baseMeshCodec.empty() ? std::string("compact") : object.baseMeshCodec;
    {
        nlohmann::json quant;
        quant["enabled"] = object.enableQuantization;
        quant["bits"] = object.quantBits;
        quant["bbox_min"] = {object.bboxMin[0], object.bboxMin[1], object.bboxMin[2]};
        quant["step"] = object.quantStep;
        j["quant"] = std::move(quant);
    }
    j["lod_order"] = "decode";
    {
        nlohmann::json layout;
        layout["partitions_per_row_group"] = object.partitionsPerRowGroup;
        layout["lods_per_row_group"] = object.lodsPerRowGroup;
        layout["page_index"] = object.pageIndex;
        layout["partitions"] = nlohmann::json::array();
        for (const auto& entry : sortedDirectory(object.partitionRgDirectory)) {
            nlohmann::json e;
            e["partition_id"] = entry.partitionId;
            e["rg_index"] = entry.rgIndex;
            e["row_index"] = entry.rowIndex;
            e["num_lods"] = entry.numLods;
            e["payload_bytes"] = entry.payloadBytes;
            layout["partitions"].push_back(std::move(e));
        }
        j["layout"] = std::move(layout);
    }
    {
        nlohmann::json streams = nlohmann::json::array();
        for (const auto& spec : sortedStreams(object.streams)) {
            nlohmann::json s;
            s["name"] = spec.name;
            s["relation"] = paperStreamRelationName(spec.relation);
            s["cell"] = paperCellTypeName(spec.cellType);
            s["shape"] = paperCellShapeName(spec.shape);
            s["encoding"] = paperStreamEncodingName(spec.encoding);
            s["codec"] = spec.codecName;
            s["alphabet_bits"] = spec.alphabetBits;
            s["per_lod"] = spec.perLod;
            s["source"] = paperStreamSourceName(spec.source);
            s["optional"] = spec.optional;
            streams.push_back(std::move(s));
        }
        j["streams"] = std::move(streams);
    }
    {
        nlohmann::json payloads = nlohmann::json::array();
        for (const auto& payload : sortedPayloads(object.namedPayloads)) {
            nlohmann::json p;
            p["name"] = payload.name;
            p["role"] = payload.role;
            p["codec"] = payload.codec;
            p["scope"] = paperPayloadScopeName(payload.scope);
            p["byte_len"] = payload.byteLength;
            payloads.push_back(std::move(p));
        }
        j["payloads"] = std::move(payloads);
    }
    {
        nlohmann::json boundary;
        boundary["codec"] = object.boundaryCodec;
        j["boundary"] = std::move(boundary);
    }
    {
        nlohmann::json ra;
        ra["prefix_unit"] = object.raPrefixUnit;
        ra["bytes_read_includes_footer"] = true;
        j["ra"] = std::move(ra);
    }

    try {
        return j.dump();
    } catch (const std::exception& e) {
        return fail(std::string("dictionary v2 serialization failed: ") + e.what());
    }
}

/**
 * @brief Parse a v2 stream dictionary into `object` (fail closed).
 *
 * v1 dictionaries are rejected with an explicit recompress hint; v1 readers are
 * expected to keep using parsePaperStreamDictionary() from the v1 header.
 */
inline bool parsePaperStreamDictionaryV2(const std::string& text, PaperObjectRow& object,
                                         std::string* error = nullptr) {
    using namespace paper_dictionary_v2_detail;
    auto fail = [&](const std::string& message) {
        if (error) *error = message;
        return false;
    };
    try {
        const auto j = nlohmann::json::parse(text);
        if (!j.is_object()) return fail("dictionary v2 is not an object");
        if (j.value("format", std::string()) != paperStreamDictionaryV2Format()) {
            return fail("unsupported dictionary format: " +
                        j.value("format", std::string()));
        }
        const std::int32_t version = j.value("version", 0);
        if (version != kPaperStreamFormatVersionV2) {
            if (version == 1) {
                return fail(
                    "unsupported PaperStream version: 1 (v1 bridge artifacts are "
                    "not readable by the v2 dictionary; recompress with paper v2)");
            }
            return fail("unsupported PaperStream version: " + std::to_string(version));
        }
        object = PaperObjectRow{};
        object.formatVersion = version;

        const std::string schema = j.value("schema", std::string());
        if (schema != "compact" && schema != "typed") {
            return fail("unsupported schema: " + schema);
        }
        object.schemaMode = schema == "compact" ? PaperSchemaMode::Compact
                                                : PaperSchemaMode::Typed;
        object.algorithm = j.value("algorithm", std::string());
        if (object.algorithm.empty()) return fail("dictionary v2 has empty algorithm");
        object.objectId = j.value("object_id", 0);
        object.numRounds = j.value("num_rounds", 0);
        object.totalVertices = j.value("total_vertices", 0);
        object.totalFaces = j.value("total_faces", 0);
        object.numPartitions = j.value("num_partitions", 0);
        if (object.numRounds < 0 || object.totalVertices < 0 || object.totalFaces < 0 ||
            object.numPartitions < 0) {
            return fail("dictionary v2 has negative object counters");
        }
        object.codecVersion = j.value("codec_version", 0);
        object.blobLayout = j.value("blob_layout", 0);
        object.schemeVersion = j.value("scheme_version", 0);
        {
            const std::string modeName = j.value("payload_mode", std::string());
            if (!paper_v2_names::paperPayloadModeFromName(modeName, object.payloadMode)) {
                return fail("unknown payload_mode: " + modeName);
            }
        }
        {
            if (!j.contains("native_payload") || !j["native_payload"].is_object()) {
                return fail("dictionary v2 has no native_payload object");
            }
            const auto& native = j["native_payload"];
            const std::string frameName = native.value("frame", std::string());
            if (!paper_v2_names::paperNativeFrameFromName(frameName, object.nativeFrame)) {
                return fail("unknown native_payload.frame: " + frameName);
            }
            if (!native.contains("stream_names") || !native["stream_names"].is_array()) {
                return fail("native_payload has no stream_names array");
            }
            object.nativeStreamNames.clear();
            std::set<std::string> seenNative;
            for (const auto& item : native["stream_names"]) {
                if (!item.is_string()) return fail("native_payload stream name is not a string");
                const std::string name = item.get<std::string>();
                if (name.empty()) return fail("native_payload has empty stream name");
                if (!seenNative.insert(name).second) {
                    return fail("duplicate native_payload stream name: " + name);
                }
                object.nativeStreamNames.push_back(name);
            }
        }
        object.baseMeshCodec = j.value("base_mesh_codec", std::string("compact"));
        if (!j.contains("quant") || !j["quant"].is_object()) {
            return fail("dictionary v2 has no quant object");
        }
        {
            const auto& quant = j["quant"];
            object.enableQuantization = quant.value("enabled", false);
            object.quantBits = quant.value("bits", 0);
            object.quantStep = quant.value("step", 0.f);
            if (!quant.contains("bbox_min") || !quant["bbox_min"].is_array() ||
                quant["bbox_min"].size() != 3) {
                return fail("quant.bbox_min must be an array of 3 numbers");
            }
            for (int i = 0; i < 3; ++i) {
                object.bboxMin[i] = quant["bbox_min"][i].get<float>();
            }
        }
        if (j.value("lod_order", std::string()) != "decode") {
            return fail("unsupported lod_order (must be \"decode\")");
        }
        if (!j.contains("layout") || !j["layout"].is_object()) {
            return fail("dictionary v2 has no layout object");
        }
        {
            const auto& layout = j["layout"];
            object.partitionsPerRowGroup = layout.value("partitions_per_row_group", 1);
            object.lodsPerRowGroup = layout.value("lods_per_row_group", 0);
            if (object.partitionsPerRowGroup < 1 || object.lodsPerRowGroup < 0) {
                return fail("layout row-group sizes are invalid");
            }
            object.pageIndex = layout.value("page_index", false);
            object.partitionRgDirectory.clear();
            if (layout.contains("partitions")) {
                if (!layout["partitions"].is_array()) {
                    return fail("layout.partitions is not an array");
                }
                std::set<std::int32_t> seen;
                for (const auto& item : layout["partitions"]) {
                    if (!item.is_object()) {
                        return fail("layout.partitions entry is not an object");
                    }
                    PaperPartitionDirectoryEntry entry;
                    entry.partitionId = item.value("partition_id", -1);
                    entry.rgIndex = item.value("rg_index", -1);
                    entry.rowIndex = item.value("row_index", 0);
                    entry.numLods = item.value("num_lods", 0);
                    entry.payloadBytes =
                        item.value("payload_bytes", static_cast<std::int64_t>(0));
                    if (entry.partitionId < 0 || entry.rgIndex < 0 || entry.numLods < 0) {
                        return fail("layout.partitions entry has invalid ids/counts");
                    }
                    if (!seen.insert(entry.partitionId).second) {
                        return fail("duplicate layout.partitions partition_id=" +
                                    std::to_string(entry.partitionId));
                    }
                    object.partitionRgDirectory.push_back(entry);
                }
            }
        }
        if (!j.contains("streams") || !j["streams"].is_array()) {
            return fail("dictionary v2 has no streams array");
        }
        object.streams.clear();
        std::set<std::string> seenStreams;
        for (const auto& s : j["streams"]) {
            if (!s.is_object()) return fail("stream spec is not an object");
            PaperStreamSpec spec;
            spec.name = s.value("name", std::string());
            if (spec.name.empty()) return fail("stream spec has empty name");
            if (!seenStreams.insert(spec.name).second) {
                return fail("duplicate stream name: " + spec.name);
            }
            const std::string relation = s.value("relation", std::string());
            if (!paper_v2_names::paperStreamRelationFromName(relation, spec.relation)) {
                return fail("stream " + spec.name + ": unknown relation: " + relation);
            }
            const std::string cell = s.value("cell", std::string());
            if (!cellTypeFromNameStrict(cell, spec.cellType)) {
                return fail("stream " + spec.name + ": unknown cell: " + cell);
            }
            const std::string shape = s.value("shape", std::string("scalar"));
            if (!paper_v2_names::paperCellShapeFromName(shape, spec.shape)) {
                return fail("stream " + spec.name + ": unknown shape: " + shape);
            }
            const std::string encoding = s.value("encoding", std::string());
            if (!encodingFromNameStrict(encoding, spec.encoding)) {
                return fail("stream " + spec.name + ": unknown encoding: " + encoding);
            }
            spec.codecName = s.value("codec", std::string());
            spec.alphabetBits = s.value("alphabet_bits", 0);
            if (spec.alphabetBits < 0) {
                return fail("stream " + spec.name + ": negative alphabet_bits");
            }
            spec.perLod = s.value("per_lod", true);
            const std::string source = s.value("source", std::string());
            if (!paper_v2_names::paperStreamSourceFromName(source, spec.source)) {
                return fail("stream " + spec.name + ": unknown source: " + source);
            }
            spec.optional = s.value("optional", false);
            object.streams.push_back(std::move(spec));
        }
        object.namedPayloads.clear();
        if (j.contains("payloads")) {
            if (!j["payloads"].is_array()) return fail("payloads is not an array");
            std::set<std::string> seen;
            for (const auto& item : j["payloads"]) {
                if (!item.is_object()) return fail("payloads entry is not an object");
                PaperNamedPayloadSpec payload;
                payload.name = item.value("name", std::string());
                if (payload.name.empty()) return fail("named payload has empty name");
                if (!seen.insert(payload.name).second) {
                    return fail("duplicate named payload: " + payload.name);
                }
                payload.role = item.value("role", std::string());
                payload.codec = item.value("codec", std::string());
                const std::string scope = item.value("scope", std::string());
                if (!paper_v2_names::paperPayloadScopeFromName(scope, payload.scope)) {
                    return fail("named payload " + payload.name + ": unknown scope: " + scope);
                }
                payload.byteLength = item.value("byte_len", static_cast<std::uint64_t>(0));
                object.namedPayloads.push_back(std::move(payload));
            }
        }
        if (j.contains("boundary")) {
            if (!j["boundary"].is_object()) return fail("boundary is not an object");
            object.boundaryCodec = j["boundary"].value("codec", std::string("raw-int32"));
        }
        if (j.contains("ra")) {
            if (!j["ra"].is_object()) return fail("ra is not an object");
            object.raPrefixUnit = j["ra"].value("prefix_unit", std::string("lod"));
            if (object.raPrefixUnit != "lod" && object.raPrefixUnit != "lod_block" &&
                object.raPrefixUnit != "partition") {
                return fail("ra.prefix_unit must be lod|lod_block|partition");
            }
        }
        if (error) error->clear();
        return true;
    } catch (const std::exception& e) {
        return fail(std::string("dictionary v2 parse failed: ") + e.what());
    }
}

}  // namespace pmsf

#endif  // PMSF_STORAGE_PAPER_STREAM_DICTIONARY_V2_H
