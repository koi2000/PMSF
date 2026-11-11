/*
 * @file PaperPrefixBpvParquet.cpp
 * @brief Measure paper-prefix artifact bytes / bpv for LOD/R-D curves.
 *
 * v1 bridge artifacts: truncate LOD rows and re-write through PaperStreamWriter.
 * v2 artifacts: read the prefix through PaperStreamReaderV2 and re-write through
 * PaperStreamWriterV2. A v2 artifact whose nativeFrame is Whole/NamedPayload has no per-round
 * payload slices, so truncating LOD rows does not reduce stored bytes; for those the tool exits 3
 * (no physical prefix rate) instead of printing a misleading flat curve.
 */
#include "storage/PaperBridgeV2.h"
#include "storage/PaperTypedLodDirectory.h"
#include "storage/PaperStreamCompactCodec.h"
#include "storage/PaperStreamReader.h"
#include "storage/PaperStreamTable.h"
#include "storage/PaperStreamWriter.h"
#include "storage/PaperStreamWriterV2.h"

#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/reader.h>
#include <parquet/file_reader.h>
#include "json.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::vector<int> parseKs(const std::string& text, int maxK) {
    std::vector<int> ks;
    std::string token;
    std::stringstream ss(text);
    while (std::getline(ss, token, ',')) {
        if (token.empty()) continue;
        const int k = std::atoi(token.c_str());
        if (k >= 0 && k <= maxK) ks.push_back(k);
    }
    if (ks.empty()) {
        const int step = std::max(1, maxK / 4);
        for (int k = 0; k <= maxK; k += step) ks.push_back(k);
        if (ks.back() != maxK) ks.push_back(maxK);
    }
    std::sort(ks.begin(), ks.end());
    ks.erase(std::unique(ks.begin(), ks.end()), ks.end());
    return ks;
}

}  // namespace

int main(int argc, char* argv[]) {
    std::string input;
    std::string output;
    std::string ksText;
    int vertices = 0;
    int onlyK = -1;
    bool keepLegacy = false;
    bool schemaDump = false;
    bool physicalDump = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&](std::string& dst) {
            if (i + 1 >= argc) { std::cerr << "missing value for " << arg << "\n"; std::exit(2); }
            dst = argv[++i];
        };
        if (arg == "--input") next(input);
        else if (arg == "--output") next(output);
        else if (arg == "--ks") next(ksText);
        else if (arg == "--vertices") { std::string v; next(v); vertices = std::atoi(v.c_str()); }
        else if (arg == "--only-k") { std::string v; next(v); onlyK = std::atoi(v.c_str()); }
        else if (arg == "--keep-legacy") keepLegacy = true;
        else if (arg == "--schema") schemaDump = true;
        else if (arg == "--physical-schema") { physicalDump = true; schemaDump = true; }
        else { std::cerr << "unknown arg: " << arg << "\n"; return 2; }
    }
    if (input.empty() || (!schemaDump && output.empty()) || (!schemaDump && vertices <= 0)) {
        std::cerr << "Usage: PaperPrefixBpv --input <_paper.parquet> --output <csv> "
                     "--vertices N [--ks a,b,c] [--only-k K] [--keep-legacy]\n"
                     "       PaperPrefixBpv --input <artifact> --schema\n";
        return 2;
    }

    const bool isV2 = pmsf::isV2PaperArtifact(input);
    pmsf::PaperStreamTable full;
    if (isV2) {
        full = pmsf::readPaperStreamV2Table(input, -1);
    } else {
        pmsf::PaperStreamReader reader(input);
        full = reader.readAll();
    }
    if (full.partitions.empty()) {
        std::cerr << "no partition in " << input << "\n";
        return 1;
    }
    if (physicalDump) {
        const auto file = arrow::io::ReadableFile::Open(input).ValueOrDie();
        auto arrowReader = parquet::arrow::OpenFile(file, arrow::default_memory_pool()).ValueOrDie();
        std::shared_ptr<arrow::Schema> schema;
        const auto status = arrowReader->GetSchema(&schema);
        if (!status.ok()) throw std::runtime_error(status.ToString());
        auto parquetReader = parquet::ParquetFileReader::OpenFile(input, false);
        const auto metadata = parquetReader->metadata();
        nlohmann::json result;
        result["format_version"] = full.object.formatVersion;
        result["schema_mode"] = full.object.schemaMode == pmsf::PaperSchemaMode::Typed ? "typed" : "compact";
        result["algorithm"] = full.object.algorithm;
        result["original_vertices"] = full.object.totalVertices;
        result["quant_bits"] = full.object.quantBits;
        result["row_groups"] = metadata->num_row_groups();
        result["fields"] = nlohmann::json::object();
        for (const auto& field : schema->fields()) result["fields"][field->name()] = field->type()->ToString();
        result["column_bytes"] = nlohmann::json::object();
        result["encodings"] = nlohmann::json::object();
        for (int rg = 0; rg < metadata->num_row_groups(); ++rg) {
            const auto group = metadata->RowGroup(rg);
            for (int c = 0; c < group->num_columns(); ++c) {
                const auto column = group->ColumnChunk(c);
                const auto name = column->path_in_schema()->ToDotString();
                auto& bytes = result["column_bytes"][name];
                if (bytes.is_null()) bytes = 0;
                bytes = bytes.get<std::int64_t>() + column->total_compressed_size();
                auto& encoding = result["encodings"][name];
                if (encoding.is_null()) encoding = nlohmann::json::array();
                for (const auto kind : column->encodings()) {
                    const int value = static_cast<int>(kind);
                    if (std::find(encoding.begin(), encoding.end(), value) == encoding.end()) encoding.push_back(value);
                }
            }
        }
        result["payload_mode"] = pmsf::paperPayloadModeName(full.object.payloadMode);
        result["ra_prefix_unit"] = full.object.raPrefixUnit;
        result["native_payload_bytes"] = 0;
        for (const auto& partition : full.partitions)
            result["native_payload_bytes"] = result["native_payload_bytes"].get<std::uint64_t>()+partition.nativePayload.size();
        result["lod_directory"] = nlohmann::json::array();
        for (const auto& lod : full.lods) {
            result["native_payload_bytes"] = result["native_payload_bytes"].get<std::uint64_t>()+lod.nativePayload.size();
            nlohmann::json entry;
            entry["lod_id"]=lod.lodId;entry["partition_id"]=lod.partitionId;
            entry["logical_offset"]=lod.payloadByteOffset;entry["logical_bytes"]=lod.payloadByteLength;
            if (full.object.payloadMode==pmsf::PaperPayloadMode::Typed) {
                entry["streams"]=nlohmann::json::object();
                for (const auto& span:pmsf::paperTypedStreamSpans(lod))
                    entry["streams"][span.name]={{"offset",span.offset},{"bytes",span.bytes}};
            }
            result["lod_directory"].push_back(std::move(entry));
        }
        result["logical_core_values"] = nlohmann::json::object();
        for (const auto& lod : full.lods) {
            for (const auto& value : std::vector<std::pair<std::string, std::size_t>>{
                {"F",lod.F.size()},{"Et",lod.Et.size()},{"Ef",lod.Ef.size()},{"V",lod.V.size()},
                {"Cf",lod.CfF.size()},{"Cv",lod.CvF.size()},{"CfQ",lod.CfQ.size()},{"CvQ",lod.CvQ.size()}}) {
                auto& count = result["logical_core_values"][value.first];
                if (count.is_null()) count = 0;
                count = count.get<std::uint64_t>() + value.second;
            }
        }
        std::cout << result.dump(2) << "\n";
        return 0;
    }
    if (schemaDump) {
        std::cout << "OBJECT," << full.object.algorithm << ","
                  << pmsf::paperPayloadModeName(full.object.payloadMode) << ","
                  << pmsf::paperNativeFrameName(full.object.nativeFrame) << ","
                  << full.object.numRounds << "," << full.partitions.size() << "\n";
        for (const auto& spec : full.object.streams) {
            std::cout << "STREAM," << spec.name << ","
                      << pmsf::paperCellTypeName(spec.cellType) << ","
                      << pmsf::paperStreamEncodingName(spec.encoding) << ","
                      << spec.codecName << "," << spec.alphabetBits << ","
                      << (spec.perLod ? 1 : 0) << ","
                      << pmsf::paperCellShapeName(spec.shape) << ","
                      << pmsf::paperStreamRelationName(spec.relation) << ","
                      << pmsf::paperStreamSourceName(spec.source) << ","
                      << (spec.optional ? 1 : 0) << "\n";
        }
        for (const auto& lod : full.lods) {
            std::cout << "LOD," << lod.lodId << "," << lod.countF << ","
                      << lod.countEt << "," << lod.countEf << "," << lod.countV
                      << "," << lod.countCf << "," << lod.countCv << ","
                      << lod.extension.size() << "\n";
        }
        return 0;
    }
    if (isV2 && full.object.payloadMode == pmsf::PaperPayloadMode::Native &&
        (full.object.nativeFrame == pmsf::PaperNativeFrame::Whole ||
         full.object.nativeFrame == pmsf::PaperNativeFrame::NamedPayload)) {
        std::cerr << "prefix-bpv: v2 artifact uses a whole-frame native payload "
                  << "(frame=" << pmsf::paperNativeFrameName(full.object.nativeFrame)
                  << "); truncating LOD rows does not reduce the stored payload bytes, so there "
                     "is no physical prefix rate for this artifact. Encode with a per-round native "
                     "frame (conn-round / class-i-stream) to measure prefix bpv.\n";
        return 3;
    }
    int maxLod = 0;
    for (const auto& lod : full.lods) maxLod = std::max(maxLod, lod.lodId);
    const std::vector<int> allKs = onlyK >= 0 ? std::vector<int>{onlyK} : parseKs(ksText, maxLod);

    std::ofstream csv(output);
    if (!csv) { std::cerr << "cannot write " << output << "\n"; return 1; }
    csv << "max_lod,lods,bytes,bpv,legacy_bundle,conn_bytes,geo_bytes,base_bytes,legacy_bytes\n";
    const auto tmp = std::filesystem::temp_directory_path() / "pmsf_paper_prefix_bpv.parquet";
    for (int k : allKs) {
        pmsf::PaperStreamTable prefix;
        if (isV2) {
            // v2 readPrefix is exclusive on lod_id; +1 makes it inclusive like the v1 filter.
            prefix = pmsf::readPaperStreamV2Table(input, k + 1);
            prefix.partitions[0].numLods = static_cast<std::int32_t>(prefix.lods.size());
            prefix.object.numRounds = static_cast<std::int32_t>(prefix.lods.size());
            // The writer rebuilds this directory from the truncated rows; the stale copy would
            // fail the numLods-mismatch contract check before it gets that far.
            prefix.object.partitionRgDirectory.clear();
        } else {
            prefix = full;
            prefix.lods.erase(
                std::remove_if(prefix.lods.begin(), prefix.lods.end(),
                               [&](const pmsf::PaperLodRow& r) { return r.lodId > k; }),
                prefix.lods.end());
            prefix.partitions[0].numLods = static_cast<std::int32_t>(prefix.lods.size());
            prefix.object.numRounds = static_cast<std::int32_t>(prefix.lods.size());
        }
        std::size_t connBytes = 0;
        std::size_t geoBytes = 0;
        for (const auto& lod : prefix.lods) {
            connBytes += pmsf::packPaperConnectivity(lod).size();
            geoBytes += pmsf::packPaperGeometry(lod).size();
        }
        std::size_t legacyBytes = 0;
        for (const auto& [name, bytes] : prefix.legacyFiles) legacyBytes += bytes.size();
        const std::size_t baseBytes = prefix.partitions[0].baseMesh.size();
        if (isV2) {
            pmsf::PaperStreamWriterV2 writerV2;
            writerV2.write(prefix, tmp.string());
        } else {
            if (!keepLegacy) prefix.legacyFiles.clear();
            pmsf::PaperStreamWriter writer;
            writer.write(prefix, tmp.string());
        }
        const auto bytes = std::filesystem::file_size(tmp);
        const double bpv = 8.0 * static_cast<double>(bytes) / static_cast<double>(vertices);
        csv << k << ',' << prefix.lods.size() << ',' << bytes << ','
            << std::setprecision(6) << bpv << ',' << (keepLegacy ? 1 : 0) << ','
            << connBytes << ',' << geoBytes << ',' << baseBytes << ',' << legacyBytes << "\n";
        std::cout << "k=" << k << " lods=" << prefix.lods.size()
                  << " bytes=" << bytes << " bpv=" << std::fixed << std::setprecision(4)
                  << bpv << "\n";
    }
    std::filesystem::remove(tmp);
    std::cout << "wrote " << output << "\n";
    return 0;
}
