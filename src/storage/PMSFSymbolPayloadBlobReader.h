/*
 * @file PMSFSymbolPayloadBlobReader.h
 * @brief Read unified paper payload blob tables.
 */
#ifndef PMSF_STORAGE_PAPER_SYMBOL_PAYLOAD_BLOB_READER_H
#define PMSF_STORAGE_PAPER_SYMBOL_PAYLOAD_BLOB_READER_H

#include "storage/PMSFPayloadDescriptorRegistry.h"

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/reader.h>

namespace pmsf {

struct PaperNamedBlobTableReadResult {
    std::string path;
    std::vector<PaperNamedBlob> blobs;
};

struct PMSFPayloadBlobTableReadResult {
    std::string path;
    PMSFSymbolPayloadTable payloads;
};

inline bool paperPayloadBlobExists(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    return f.good();
}

inline std::vector<PaperNamedBlob> readPaperNamedBlobTable(const std::string& path) {
    std::vector<PaperNamedBlob> out;
    if (!paperPayloadBlobExists(path)) return out;

    auto file = arrow::io::ReadableFile::Open(path);
    if (!file.ok()) return out;

    auto readerResult =
        parquet::arrow::OpenFile(file.ValueOrDie(), arrow::default_memory_pool());
    if (!readerResult.ok()) return out;
    auto reader = std::move(readerResult).ValueOrDie();

    std::shared_ptr<arrow::Table> table;
    auto status = reader->ReadTable(&table);
    if (!status.ok() || !table) return out;

    auto combined = table->CombineChunks(arrow::default_memory_pool());
    if (!combined.ok()) return out;
    auto ct = combined.ValueOrDie();

    auto nameCol = ct->GetColumnByName(pmsf::kPMSFPayloadBlobNameColumn);
    auto kindCol = ct->GetColumnByName(pmsf::kPMSFPayloadBlobKindColumn);
    auto bytesCol = ct->GetColumnByName(pmsf::kPMSFPayloadBlobBytesColumn);
    if (!nameCol || !bytesCol || nameCol->num_chunks() != 1 ||
        bytesCol->num_chunks() != 1) {
        return out;
    }

    auto nameArr = std::static_pointer_cast<arrow::StringArray>(nameCol->chunk(0));
    std::shared_ptr<arrow::StringArray> kindArr;
    if (kindCol && kindCol->num_chunks() == 1) {
        kindArr = std::static_pointer_cast<arrow::StringArray>(kindCol->chunk(0));
    }
    auto bytesArr = std::static_pointer_cast<arrow::BinaryArray>(bytesCol->chunk(0));

    for (int64_t row = 0; row < nameArr->length(); ++row) {
        std::string kind;
        if (kindArr && row < kindArr->length() && !kindArr->IsNull(row)) {
            kind = kindArr->GetString(row);
        }
        PaperNamedBlob blob = makePaperNamedBlob(
            nameArr->GetString(row), kind, nullptr, 0);
        if (row < bytesArr->length() && !bytesArr->IsNull(row)) {
            int32_t len = 0;
            const uint8_t* bytes = bytesArr->GetValue(row, &len);
            setPaperNamedBlobBytes(blob, bytes, static_cast<std::size_t>(len));
        }
        appendPaperNamedBlobRecord(out, std::move(blob));
    }

    return out;
}

inline PMSFSymbolPayloadTable paperPayloadTableFromNamedBlobs(
    const std::vector<PaperNamedBlob>& blobs) {
    PMSFSymbolPayloadTable out;
    for (const auto& blob : blobs) {
        PMSFSymbolPayloadRecord record;
        if (!parsePMSFPayloadBlobName(blob.name, record)) {
            continue;
        }
        setPMSFSymbolPayloadCodecName(record, blob.kind);
        setPMSFPayloadRecordRoleFromNameCodec(record);
        setPMSFSymbolPayloadBytes(record, blob.bytes);
        appendPMSFPayloadRecord(out, std::move(record));
    }

    std::sort(out.begin(), out.end(),
              [](const PMSFSymbolPayloadRecord& a,
                 const PMSFSymbolPayloadRecord& b) {
                  if (a.objectId != b.objectId) return a.objectId < b.objectId;
                  if (a.partitionId != b.partitionId) return a.partitionId < b.partitionId;
                  if (a.lodId != b.lodId) return a.lodId < b.lodId;
                  return a.payloadName < b.payloadName;
              });
    return out;
}

inline PMSFSymbolPayloadTable readPMSFPayloadBlobTable(const std::string& path) {
    return paperPayloadTableFromNamedBlobs(readPaperNamedBlobTable(path));
}

inline PaperNamedBlobTableReadResult readFirstPaperNamedBlobTableCandidate(
    const std::string& path) {
    for (const auto& candidate : paperPayloadBlobCandidates(path)) {
        if (!paperPayloadBlobExists(candidate)) {
            continue;
        }
        auto blobs = readPaperNamedBlobTable(candidate);
        if (blobs.empty()) {
            continue;
        }
        return {candidate, std::move(blobs)};
    }
    return {};
}

inline PMSFPayloadBlobTableReadResult readFirstPMSFPayloadBlobTableCandidate(
    const std::string& path) {
    auto named = readFirstPaperNamedBlobTableCandidate(path);
    if (named.blobs.empty()) {
        return {};
    }
    PMSFPayloadBlobTableReadResult out;
    out.path = std::move(named.path);
    out.payloads = paperPayloadTableFromNamedBlobs(named.blobs);
    return out;
}

inline std::string paperNamedBlobBytesAsString(const PaperNamedBlob& blob) {
    return std::string(blob.bytes.begin(), blob.bytes.end());
}

inline std::string findPMSFPayloadOrRawNamedBlobString(
    const std::vector<PaperNamedBlob>& blobs,
    std::int32_t objectId,
    std::int32_t partitionId,
    std::int32_t lodId,
    const std::string& payloadName) {
    const auto* blob = findPMSFPayloadOrRawNamedBlob(
        blobs, objectId, partitionId, lodId, payloadName);
    return blob ? paperNamedBlobBytesAsString(*blob) : std::string();
}

inline bool paperNamedBlobMatchesPayloadRole(
    const PaperNamedBlob& blob,
    PMSFPayloadRole role,
    std::int32_t objectId,
    std::int32_t partitionId,
    std::int32_t lodId) {
    PMSFSymbolPayloadRecord row;
    if (parsePMSFPayloadBlobName(blob.name, row)) {
        if (row.objectId != objectId ||
            row.partitionId != partitionId ||
            row.lodId != lodId) {
            return false;
        }
        setPMSFSymbolPayloadCodecName(row, blob.kind);
        setPMSFPayloadRecordRoleFromNameCodec(row);
        return paperPayloadRecordMatchesRole(row, role);
    }
    if (objectId != 0 || partitionId != 0 || lodId != 0) {
        return false;
    }
    return inferPMSFPayloadRoleName(blob.name, blob.kind) ==
           paperPayloadRoleName(role);
}

inline const PaperNamedBlob* findPMSFPayloadOrRawNamedBlobByRole(
    const std::vector<PaperNamedBlob>& blobs,
    PMSFPayloadRole role,
    std::int32_t objectId = 0,
    std::int32_t partitionId = 0,
    std::int32_t lodId = 0) {
    for (const auto& blob : blobs) {
        if (paperNamedBlobMatchesPayloadRole(
                blob, role, objectId, partitionId, lodId)) {
            return &blob;
        }
    }
    return nullptr;
}

inline std::string findPMSFPayloadOrRawNamedBlobStringByRole(
    const std::vector<PaperNamedBlob>& blobs,
    PMSFPayloadRole role,
    std::int32_t objectId = 0,
    std::int32_t partitionId = 0,
    std::int32_t lodId = 0) {
    const auto* blob = findPMSFPayloadOrRawNamedBlobByRole(
        blobs, role, objectId, partitionId, lodId);
    return blob ? paperNamedBlobBytesAsString(*blob) : std::string();
}

inline const std::vector<std::uint8_t>* findPMSFPayloadOrRawBlobMapBytesByRole(
    const std::unordered_map<std::string, std::vector<std::uint8_t>>& blobs,
    const std::unordered_map<std::string, std::string>& blobKinds,
    PMSFPayloadRole role,
    std::int32_t objectId = 0,
    std::int32_t partitionId = 0,
    std::int32_t lodId = 0) {
    for (const auto& kv : blobs) {
        PMSFSymbolPayloadRecord row;
        auto kind = blobKinds.find(kv.first);
        const std::string codecName =
            kind == blobKinds.end() ? std::string() : kind->second;
        if (parsePMSFPayloadBlobName(kv.first, row)) {
            if (row.objectId != objectId ||
                row.partitionId != partitionId ||
                row.lodId != lodId) {
                continue;
            }
            setPMSFSymbolPayloadCodecName(row, codecName);
            setPMSFPayloadRecordRoleFromNameCodec(row);
            if (paperPayloadRecordMatchesRole(row, role)) {
                return &kv.second;
            }
            continue;
        }
        if (objectId != 0 || partitionId != 0 || lodId != 0) {
            continue;
        }
        if (inferPMSFPayloadRoleName(kv.first, codecName) ==
            paperPayloadRoleName(role)) {
            return &kv.second;
        }
    }
    return nullptr;
}

inline const std::vector<std::uint8_t>*
findPMSFPayloadOrRawBlobMapBytesByNameOrSafeRole(
    const std::unordered_map<std::string, std::vector<std::uint8_t>>& blobs,
    const std::unordered_map<std::string, std::string>& blobKinds,
    std::int32_t objectId,
    std::int32_t partitionId,
    std::int32_t lodId,
    const std::string& payloadName) {
    if (const auto* bytes = findPMSFPayloadOrRawNamedBlobBytes(
            blobs, objectId, partitionId, lodId, payloadName)) {
        return bytes;
    }
    const auto role = inferPMSFPayloadRole(payloadName);
    if (role == PMSFPayloadRole::AlgorithmPayload) {
        return nullptr;
    }
    return findPMSFPayloadOrRawBlobMapBytesByRole(
        blobs, blobKinds, role, objectId, partitionId, lodId);
}

inline std::string findPMSFPayloadOrRawNamedBlobStringInCandidates(
    const std::string& path,
    std::int32_t objectId,
    std::int32_t partitionId,
    std::int32_t lodId,
    const std::string& payloadName,
    std::string* sourcePath = nullptr) {
    const auto role = inferPMSFPayloadRole(payloadName);
    for (const auto& candidate : paperPayloadBlobCandidates(path)) {
        if (!paperPayloadBlobExists(candidate)) {
            continue;
        }
        const auto blobs = readPaperNamedBlobTable(candidate);
        auto value = findPMSFPayloadOrRawNamedBlobString(
            blobs, objectId, partitionId, lodId, payloadName);
        if (value.empty() && role != PMSFPayloadRole::AlgorithmPayload) {
            value = findPMSFPayloadOrRawNamedBlobStringByRole(
                blobs, role, objectId, partitionId, lodId);
        }
        if (value.empty()) {
            continue;
        }
        if (sourcePath != nullptr) {
            *sourcePath = candidate;
        }
        return value;
    }
    return {};
}

inline std::string findPMSFPayloadOrRawNamedBlobStringByRoleInCandidates(
    const std::string& path,
    PMSFPayloadRole role,
    std::int32_t objectId = 0,
    std::int32_t partitionId = 0,
    std::int32_t lodId = 0,
    std::string* sourcePath = nullptr) {
    for (const auto& candidate : paperPayloadBlobCandidates(path)) {
        if (!paperPayloadBlobExists(candidate)) {
            continue;
        }
        auto value = findPMSFPayloadOrRawNamedBlobStringByRole(
            readPaperNamedBlobTable(candidate),
            role,
            objectId,
            partitionId,
            lodId);
        if (value.empty()) {
            continue;
        }
        if (sourcePath) {
            *sourcePath = candidate;
        }
        return value;
    }
    return {};
}

}  // namespace pmsf

#endif  // PMSF_STORAGE_PAPER_SYMBOL_PAYLOAD_BLOB_READER_H
