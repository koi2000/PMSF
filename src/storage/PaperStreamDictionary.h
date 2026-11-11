/*
 * @file PaperStreamDictionary.h
 * @brief JSON serialization for PaperStream v1 stream dictionaries.
 *
 * The dictionary is the only algorithm-specific descriptor in the physical
 * artifact.  It is stored in the object row (M1) and read before any stream
 * column is interpreted.  Unknown names/encodings must fail closed.
 */
#ifndef PMSF_STORAGE_PAPER_STREAM_DICTIONARY_H
#define PMSF_STORAGE_PAPER_STREAM_DICTIONARY_H

#include "storage/PaperStreamTable.h"

#include "json.hpp"

#include <algorithm>
#include <set>
#include <string>

namespace pmsf {

inline PaperCellType paperCellTypeFromName(const std::string& name) {
    if (name == "int32")
        return PaperCellType::Int32;
    if (name == "float")
        return PaperCellType::Float;
    return PaperCellType::Bytes;
}

inline PaperStreamEncoding paperStreamEncodingFromName(const std::string& name) {
    if (name == "bitpacked")
        return PaperStreamEncoding::BitPacked;
    if (name == "rle")
        return PaperStreamEncoding::Rle;
    if (name == "delta")
        return PaperStreamEncoding::Delta;
    if (name == "bss")
        return PaperStreamEncoding::ByteStreamSplit;
    if (name == "compact")
        return PaperStreamEncoding::Compact;
    return PaperStreamEncoding::Plain;
}

inline std::string serializePaperStreamDictionary(const PaperObjectRow& object, std::string* error = nullptr) {
    nlohmann::json j;
    j["format"] = "pmsf-paperstream";
    j["version"] = object.formatVersion;
    j["algorithm"] = object.algorithm;
    j["schema"] = object.schemaMode == PaperSchemaMode::Compact ? "compact" : "typed";
    j["codec_version"] = object.codecVersion;
    j["num_rounds"] = object.numRounds;
    j["num_partitions"] = object.numPartitions;
    j["partitions_per_row_group"] = object.partitionsPerRowGroup;
    j["total_vertices"] = object.totalVertices;
    j["total_faces"] = object.totalFaces;
    j["quantization"] = object.enableQuantization;
    j["quant_bits"] = object.quantBits;
    j["bbox_min"] = {object.bboxMin[0], object.bboxMin[1], object.bboxMin[2]};
    j["quant_step"] = object.quantStep;

    nlohmann::json streams = nlohmann::json::array();
    std::set<std::string> seen;
    for (const auto& spec : object.streams) {
        if (spec.name.empty()) {
            if (error)
                *error = "empty stream name";
            return {};
        }
        if (!seen.insert(spec.name).second) {
            if (error)
                *error = "duplicate stream name: " + spec.name;
            return {};
        }
        nlohmann::json s;
        s["name"] = spec.name;
        s["cell"] = paperCellTypeName(spec.cellType);
        s["encoding"] = paperStreamEncodingName(spec.encoding);
        s["codec"] = spec.codecName;
        s["alphabet_bits"] = spec.alphabetBits;
        s["per_lod"] = spec.perLod;
        streams.push_back(std::move(s));
    }
    j["streams"] = std::move(streams);

    try {
        return j.dump();
    } catch (const std::exception& e) {
        if (error)
            *error = e.what();
        return {};
    }
}

inline bool parsePaperStreamDictionary(const std::string& text, PaperObjectRow& object, std::string* error = nullptr) {
    try {
        const auto j = nlohmann::json::parse(text);
        if (!j.is_object()) {
            if (error)
                *error = "dictionary is not an object";
            return false;
        }
        const std::string format = j.value("format", std::string());
        if (format != "pmsf-paperstream") {
            if (error)
                *error = "unsupported dictionary format: " + format;
            return false;
        }
        const int version = j.value("version", 0);
        if (version != 1) {
            if (error)
                *error = "unsupported PaperStream version: " + std::to_string(version);
            return false;
        }
        object.formatVersion = version;
        object.algorithm = j.value("algorithm", std::string());
        object.schemaMode = j.value("schema", std::string("typed")) == "compact" ? PaperSchemaMode::Compact : PaperSchemaMode::Typed;
        object.codecVersion = j.value("codec_version", 0);
        object.numRounds = j.value("num_rounds", 0);
        object.numPartitions = j.value("num_partitions", 0);
        object.partitionsPerRowGroup = std::max(1, j.value("partitions_per_row_group", 1));
        object.totalVertices = j.value("total_vertices", 0);
        object.totalFaces = j.value("total_faces", 0);
        object.enableQuantization = j.value("quantization", false);
        object.quantBits = j.value("quant_bits", 0);
        object.quantStep = j.value("quant_step", 0.f);
        if (j.contains("bbox_min") && j["bbox_min"].is_array() && j["bbox_min"].size() == 3) {
            for (int i = 0; i < 3; ++i)
                object.bboxMin[i] = j["bbox_min"][i].get<float>();
        }
        object.streams.clear();
        std::set<std::string> seen;
        if (!j.contains("streams") || !j["streams"].is_array()) {
            if (error)
                *error = "dictionary has no streams array";
            return false;
        }
        for (const auto& s : j["streams"]) {
            if (!s.is_object()) {
                if (error)
                    *error = "stream spec is not an object";
                return false;
            }
            PaperStreamSpec spec;
            spec.name = s.value("name", std::string());
            if (spec.name.empty()) {
                if (error)
                    *error = "stream spec has empty name";
                return false;
            }
            if (!seen.insert(spec.name).second) {
                if (error)
                    *error = "duplicate stream name: " + spec.name;
                return false;
            }
            spec.cellType = paperCellTypeFromName(s.value("cell", std::string("bytes")));
            spec.encoding = paperStreamEncodingFromName(s.value("encoding", std::string("plain")));
            spec.codecName = s.value("codec", std::string());
            spec.alphabetBits = s.value("alphabet_bits", 0);
            spec.perLod = s.value("per_lod", true);
            object.streams.push_back(std::move(spec));
        }
        return true;
    } catch (const std::exception& e) {
        if (error)
            *error = e.what();
        return false;
    }
}

}  // namespace pmsf

#endif  // PMSF_STORAGE_PAPER_STREAM_DICTIONARY_H
