/*
 * @file PaperStreamOpaquePayloadV2.cpp
 * @brief Opaque-payload PaperStream v2 artifact writer/reader (M5-5).
 */
#include "storage/PaperStreamOpaquePayloadV2.h"

#include "storage/PaperStreamBlobContainerV2.h"
#include "storage/PaperStreamReaderV2.h"
#include "storage/PaperStreamTable.h"
#include "storage/PaperStreamWriterV2.h"

#include <exception>
#include <string>
#include <vector>

namespace pmsf {
namespace paper_opaque {

bool writePaperStreamV2OpaquePayloadArtifact(
    const std::string& path, const std::vector<std::uint8_t>& payload,
    const OpaquePaperArtifactSpec& spec,
    paper_multistream::PayloadCodec codec, std::string* error) {
    auto fail = [&](const std::string& message) {
        if (error) *error = message;
        return false;
    };
    if (payload.empty()) return fail("opaque artifact: empty payload");
    if (spec.algorithm.empty()) return fail("opaque artifact: empty algorithm");
    std::vector<std::uint8_t> coded;
    if (!paper_multistream::encodePayload(payload, codec, coded, error)) {
        return false;
    }

    PaperStreamTable table;
    table.object.formatVersion = kPaperStreamFormatVersionV2;
    table.object.schemaMode = PaperSchemaMode::Compact;
    table.object.algorithm = spec.algorithm;
    table.object.objectId = 0;
    table.object.numRounds = spec.rounds;
    table.object.totalVertices = spec.totalVertices;
    table.object.totalFaces = spec.totalFaces;
    table.object.numPartitions = 1;
    table.object.payloadMode = PaperPayloadMode::Native;
    table.object.nativeFrame = PaperNativeFrame::NamedPayload;
    table.object.blobLayout = spec.blobLayout;
    table.object.codecVersion = spec.codecVersion;
    table.object.schemeVersion = spec.schemeVersion;
    table.object.raPrefixUnit = "partition";
    table.object.boundaryCodec = spec.boundaryCodec;

    PaperPartitionRow partition;
    partition.partitionId = 0;
    partition.numVertices = spec.baseVertices;
    partition.numFaces = spec.baseFaces;
    // Opaque payload is partition-level; declare the LOD count for the v2 table
    // contract (the blob container stores only numLods + payload, so the empty
    // LOD rows below add no artifact bytes).
    partition.numLods = spec.rounds;
    partition.nativePayload = std::move(coded);
    table.partitions.push_back(std::move(partition));
    for (std::int32_t lod = 0; lod < spec.rounds; ++lod) {
        PaperLodRow row;
        row.objectId = table.object.objectId;
        row.partitionId = 0;
        row.lodId = lod;
        table.lods.push_back(std::move(row));
    }

    PaperWriterV2Options options;
    options.container = PaperV2Container::Blob;
    options.blobPayloadCodec = static_cast<std::uint8_t>(codec);
    options.blobBaseCodec = paper_blob_v2::kBaseCodecNone;
    try {
        PaperStreamWriterV2 writer;
        writer.write(table, path, options);
    } catch (const std::exception& e) {
        return fail(std::string("opaque artifact write failed: ") + e.what());
    }
    if (error) error->clear();
    return true;
}

bool readPaperStreamV2OpaquePayloadArtifact(
    const std::string& path, std::vector<std::uint8_t>* payload,
    paper_multistream::PayloadCodec* codec, std::string* error) {
    auto fail = [&](const std::string& message) {
        if (error) *error = message;
        return false;
    };
    if (!payload) return fail("opaque artifact reader: null output");
    try {
        PaperStreamReaderV2 reader(path);
        if (reader.usedV1Fallback()) {
            return fail("opaque artifact reader: artifact is PaperStream v1");
        }
        if (!reader.isBlobContainer()) {
            return fail("opaque artifact reader: not a v2 blob-container artifact");
        }
        if (reader.blobBaseCodec() != paper_blob_v2::kBaseCodecNone) {
            return fail("opaque artifact reader: expected baseCodec=None");
        }
        const PaperStreamTable table = reader.readAll();
        if (table.partitions.empty()) return fail("opaque artifact reader: no partition");
        const auto artifactCodec =
            static_cast<paper_multistream::PayloadCodec>(reader.blobPayloadCodec());
        if (!paper_multistream::decodePayload(
                table.partitions.front().nativePayload, artifactCodec, *payload,
                error)) {
            return false;
        }
        if (payload->empty()) return fail("opaque artifact reader: empty payload");
        if (codec) *codec = artifactCodec;
    } catch (const std::exception& e) {
        return fail(std::string("opaque artifact read failed: ") + e.what());
    }
    if (error) error->clear();
    return true;
}

}  // namespace paper_opaque
}  // namespace pmsf
