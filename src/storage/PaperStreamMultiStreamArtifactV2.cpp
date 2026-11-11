/*
 * @file PaperStreamMultiStreamArtifactV2.cpp
 * @brief PSM2 multi-stream <-> PaperStream v2 blob artifact glue (M5-3).
 */
#include "storage/PaperStreamMultiStreamArtifactV2.h"

#include "storage/PaperBlobLayoutRegistry.h"
#include "storage/PaperStreamBlobContainerV2.h"
#include "storage/PaperStreamReaderV2.h"
#include "storage/PaperStreamTable.h"
#include "storage/PaperStreamWriterV2.h"

#include <cstdint>
#include <string>
#include <vector>

namespace pmsf {
namespace paper_multistream {

bool writePaperStreamV2MultiStreamArtifact(
    const std::string& path, const std::vector<SeamStream>& streams,
    const QemBundleMeta& meta, const QemPaperOptions& options,
    std::string* error) {
    if (!validateQemBundle(streams, meta, error)) return false;
    const std::vector<std::uint8_t> bundle = serializeBundle(streams, meta, error);
    if (bundle.empty()) return false;
    std::vector<std::uint8_t> payload;
    if (!encodePayload(bundle, options.payloadCodec, payload, error)) return false;

    PaperStreamTable table;
    table.object.formatVersion = kPaperStreamFormatVersionV2;
    table.object.schemaMode = PaperSchemaMode::Compact;
    table.object.algorithm = "qem";
    table.object.objectId = 0;
    table.object.numRounds = static_cast<std::int32_t>(meta.roundCount);
    table.object.totalVertices = static_cast<std::int32_t>(meta.originalVertices);
    table.object.totalFaces = static_cast<std::int32_t>(meta.originalFaces);
    table.object.numPartitions = 1;
    table.object.payloadMode = PaperPayloadMode::Native;
    table.object.nativeFrame = PaperNativeFrame::NamedPayload;
    table.object.blobLayout = kPaperBlobLayoutQEMProgressive;
    table.object.schemeVersion = 1;
    table.object.raPrefixUnit = "partition";
    table.object.baseMeshCodec = "qem-progressive";
    table.object.boundaryCodec = "";

    PaperPartitionRow partition;
    partition.partitionId = 0;
    partition.numVertices = static_cast<std::int32_t>(meta.baseVertices);
    partition.numFaces = static_cast<std::int32_t>(meta.baseFaces);
    // Progressive payload is partition-level, but the v2 table contract requires
    // numLods rows to match the declared LOD count; the blob container serializes
    // only the partition row (numLods + payload), so these empty LOD rows are a
    // table-level declaration and cost zero artifact bytes.
    partition.numLods = static_cast<std::int32_t>(meta.roundCount);
    partition.baseMeshCodec = "qem-progressive";
    partition.nativePayload = std::move(payload);
    table.partitions.push_back(std::move(partition));
    for (std::int32_t lod = 0; lod < static_cast<std::int32_t>(meta.roundCount);
         ++lod) {
        PaperLodRow row;
        row.objectId = table.object.objectId;
        row.partitionId = 0;
        row.lodId = lod;
        table.lods.push_back(std::move(row));
    }

    PaperWriterV2Options writerOptions;
    writerOptions.container = PaperV2Container::Blob;
    writerOptions.blobPayloadCodec = static_cast<std::uint8_t>(options.payloadCodec);
    writerOptions.blobBaseCodec = paper_blob_v2::kBaseCodecNone;
    try {
        PaperStreamWriterV2 writer;
        writer.write(table, path, writerOptions);
    } catch (const std::exception& e) {
        if (error) *error = std::string("PSM2 artifact write failed: ") + e.what();
        return false;
    }
    if (error) error->clear();
    return true;
}

bool readPaperStreamV2MultiStreamArtifact(
    const std::string& path, std::vector<SeamStream>* streams,
    QemBundleMeta* meta, std::string* error) {
    if (!streams || !meta) {
        if (error) *error = "PSM2 reader: null output";
        return false;
    }
    try {
        PaperStreamReaderV2 reader(path);
        if (reader.usedV1Fallback()) {
            if (error) *error = "PSM2 reader: artifact is PaperStream v1";
            return false;
        }
        if (!reader.isBlobContainer()) {
            if (error) {
                *error = "PSM2 reader: artifact is not a v2 blob-container artifact";
            }
            return false;
        }
        const PaperStreamTable table = reader.readAll();
        if (table.partitions.empty()) {
            if (error) *error = "PSM2 reader: no partition in artifact";
            return false;
        }
        const PayloadCodec codec = static_cast<PayloadCodec>(reader.blobPayloadCodec());
        std::vector<std::uint8_t> bundle;
        if (!decodePayload(table.partitions.front().nativePayload, codec, bundle,
                           error)) {
            return false;
        }
        std::vector<SeamStream> parsed;
        QemBundleMeta parsedMeta;
        if (!parseBundle(bundle, parsed, parsedMeta, error)) return false;
        if (!validateQemBundle(parsed, parsedMeta, error)) return false;
        *streams = std::move(parsed);
        *meta = parsedMeta;
    } catch (const std::exception& e) {
        if (error) *error = std::string("PSM2 artifact read failed: ") + e.what();
        return false;
    }
    if (error) error->clear();
    return true;
}

}  // namespace paper_multistream
}  // namespace pmsf
