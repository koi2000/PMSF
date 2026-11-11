/*
 * @file PMSFSymbolPayloadBlobWriter.h
 * @brief Write unified paper payload blob tables.
 */
#ifndef PMSF_STORAGE_PAPER_SYMBOL_PAYLOAD_BLOB_WRITER_H
#define PMSF_STORAGE_PAPER_SYMBOL_PAYLOAD_BLOB_WRITER_H

#include "storage/PMSFPayloadDescriptorRegistry.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <arrow/api.h>
#include <arrow/builder.h>
#include <arrow/io/api.h>
#include <parquet/arrow/writer.h>
#include <parquet/properties.h>

namespace pmsf {

inline void appendPaperNamedBlob(std::vector<PaperNamedBlob>& blobs,
                                 const std::string& name,
                                 const std::string& kind,
                                 const void* data,
                                 std::size_t size) {
    for (auto& existing : blobs) {
        if (existing.name != name) continue;
        existing.kind = kind;
        setPaperNamedBlobBytes(existing, data, size);
        return;
    }

    PaperNamedBlob blob = makePaperNamedBlob(name, kind, data, size);
    appendPaperNamedBlobRecord(blobs, std::move(blob));
}

inline void appendPaperNamedBlob(std::vector<PaperNamedBlob>& blobs,
                                 PMSFSymbolPayloadRecord row) {
    setPMSFPayloadRecordRoleFromNameCodec(row);
    PaperNamedBlob blob = makePaperNamedBlob(std::move(row));
    appendPaperNamedBlob(blobs,
                         blob.name,
                         blob.kind,
                         blob.bytes.data(),
                         blob.bytes.size());
}


inline void appendPMSFPayloadBlob(std::vector<PaperNamedBlob>& blobs,
                                   const std::string& payloadName,
                                   const std::string& codecName,
                                   const void* data,
                                   std::size_t size,
                                   std::int32_t objectId = 0,
                                   std::int32_t partitionId = 0,
                                   std::int32_t lodId = 0) {
    appendPaperNamedBlob(
        blobs,
        paperPayloadRecordFromDescriptor(
            makePMSFPayloadDescriptorWithInferredRole(objectId,
                                                       partitionId,
                                                       lodId,
                                                       payloadName,
                                                       codecName),
            data,
            size));
}

inline void appendPMSFPayloadRows(std::vector<PaperNamedBlob>& blobs,
                                   const PMSFSymbolPayloadTable& rows) {
    blobs.reserve(blobs.size() + rows.size());
    for (const auto& row : rows) {
        appendPaperNamedBlob(blobs, row);
    }
}

inline std::vector<PaperNamedBlob> paperNamedBlobsFromPayloadRows(
    const PMSFSymbolPayloadTable& rows) {
    std::vector<PaperNamedBlob> blobs;
    appendPMSFPayloadRows(blobs, rows);
    return blobs;
}

inline void appendNormalizedPMSFPayloadBlob(std::vector<PaperNamedBlob>& blobs,
                                             const PaperNamedBlob& blob) {
    if (isPMSFPayloadBlobName(blob.name)) {
        appendPaperNamedBlob(blobs,
                             blob.name,
                             blob.kind,
                             blob.bytes.data(),
                             blob.bytes.size());
        return;
    }
    appendPMSFPayloadBlob(blobs,
                           blob.name,
                           blob.kind,
                           blob.bytes.data(),
                           blob.bytes.size());
}

inline std::vector<PaperNamedBlob> normalizePMSFPayloadBlobs(
    const std::vector<PaperNamedBlob>& blobs) {
    std::vector<PaperNamedBlob> normalized;
    normalized.reserve(blobs.size());
    for (const auto& blob : blobs) {
        appendNormalizedPMSFPayloadBlob(normalized, blob);
    }
    return normalized;
}

inline bool writePaperNamedBlobTable(const std::vector<PaperNamedBlob>& blobs,
                                     const std::string& path) {
    if (blobs.empty()) return true;

    bool hasPMSFPayload = false;
    for (const auto& blob : blobs) {
        if (isPMSFPayloadBlobName(blob.name)) {
            hasPMSFPayload = true;
            break;
        }
    }

    auto kv = arrow::KeyValueMetadata::Make(
        {pmsf::kPMSFFormatVersionMetadataKey, pmsf::kPMSFPayloadTableMetadataKey},
        {"3", hasPMSFPayload ? "1" : "0"});
    auto schema = std::make_shared<arrow::Schema>(
        std::vector<std::shared_ptr<arrow::Field>>{
            arrow::field(pmsf::kPMSFPayloadBlobNameColumn, arrow::utf8()),
            arrow::field(pmsf::kPMSFPayloadBlobKindColumn, arrow::utf8()),
            arrow::field(pmsf::kPMSFPayloadBlobBytesColumn, arrow::binary()),
        },
        kv);

    arrow::StringBuilder nameB;
    arrow::StringBuilder kindB;
    arrow::BinaryBuilder bytesB;
    for (const auto& blob : blobs) {
        auto status = nameB.Append(blob.name);
        if (!status.ok()) return false;
        status = kindB.Append(blob.kind);
        if (!status.ok()) return false;
        const char* data = blob.bytes.empty()
                               ? ""
                               : reinterpret_cast<const char*>(blob.bytes.data());
        status = bytesB.Append(data,
                               static_cast<std::int32_t>(blob.bytes.size()));
        if (!status.ok()) return false;
    }

    std::shared_ptr<arrow::Array> nameA;
    std::shared_ptr<arrow::Array> kindA;
    std::shared_ptr<arrow::Array> bytesA;
    if (!nameB.Finish(&nameA).ok()) return false;
    if (!kindB.Finish(&kindA).ok()) return false;
    if (!bytesB.Finish(&bytesA).ok()) return false;

    auto table = arrow::Table::Make(schema, {nameA, kindA, bytesA},
                                    static_cast<int64_t>(blobs.size()));
    auto outFile = arrow::io::FileOutputStream::Open(path);
    if (!outFile.ok()) return false;
    auto out = outFile.ValueOrDie();

    auto propsBuilder = parquet::WriterProperties::Builder();
    propsBuilder.compression(parquet::Compression::ZSTD);
    propsBuilder.compression_level(3);
    propsBuilder.disable_dictionary();
    auto props = propsBuilder.build();
    auto arrowProps = parquet::ArrowWriterProperties::Builder().store_schema()->build();
    auto status = parquet::arrow::WriteTable(*table,
                                             arrow::default_memory_pool(),
                                             out,
                                             64 * 1024,
                                             props,
                                             arrowProps);
    if (!status.ok()) return false;
    return out->Close().ok();
}

inline bool writePMSFPayloadBlobTable(const PMSFSymbolPayloadTable& rows,
                                       const std::string& path) {
    std::vector<PaperNamedBlob> blobs;
    appendPMSFPayloadRows(blobs, rows);
    return writePaperNamedBlobTable(blobs, path);
}

}  // namespace pmsf

#endif  // PMSF_STORAGE_PAPER_SYMBOL_PAYLOAD_BLOB_WRITER_H
