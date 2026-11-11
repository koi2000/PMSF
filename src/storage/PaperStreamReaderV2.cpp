/*
 * @file PaperStreamReaderV2.cpp
 * @brief PaperStream v2 reader implementation (M4-2; see header).
 */
#include "storage/PaperStreamReaderV2.h"

#include "storage/PaperStreamBlobContainerV2.h"
#include "storage/PaperStreamCompactCodec.h"  // v1 unpack helpers (wire-compatible)
#include "storage/PaperStreamCodecV2.h"
#include "storage/PaperStreamDictionaryV2.h"
#include "storage/PaperStreamReader.h"  // v1 fallback (untouched)

#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/reader.h>
#include <parquet/file_reader.h>
#include <parquet/metadata.h>

#include "json.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace pmsf {
namespace {

constexpr const char* kRecordType = "record_type";
constexpr const char* kObjectId = "object_id";
constexpr const char* kPartitionId = "partition_id";
constexpr const char* kLodId = "lod_id";
constexpr const char* kHeader = "header";
constexpr const char* kBaseMesh = "base_mesh";
constexpr const char* kConnectivity = "connectivity";
constexpr const char* kGeometry = "geometry";
constexpr const char* kExtensions = "extensions";
constexpr const char* kNativePayload = "native_payload";
constexpr const char* kBoundary = "boundary";

std::shared_ptr<arrow::Array> getColumn(const std::shared_ptr<arrow::Table>& table,
                                        const std::string& name) {
    if (!table) return nullptr;
    const int index = table->schema()->GetFieldIndex(name);
    if (index < 0) return nullptr;
    auto column = table->column(index);
    if (column->num_chunks() == 0) return nullptr;
    return column->chunk(0);
}

std::int8_t getInt8(const std::shared_ptr<arrow::Array>& array, std::int64_t index) {
    if (!array || array->IsNull(index)) return 0;
    return static_cast<arrow::Int8Array*>(array.get())->Value(index);
}

std::int32_t getInt32(const std::shared_ptr<arrow::Array>& array, std::int64_t index) {
    if (!array || array->IsNull(index)) return 0;
    return static_cast<arrow::Int32Array*>(array.get())->Value(index);
}

std::vector<std::uint8_t> getBinary(const std::shared_ptr<arrow::Array>& array,
                                    std::int64_t index) {
    std::vector<std::uint8_t> out;
    if (!array || array->IsNull(index)) return out;
    const auto* binary = static_cast<arrow::BinaryArray*>(array.get());
    const auto view = binary->GetView(index);
    out.assign(reinterpret_cast<const std::uint8_t*>(view.data()),
               reinterpret_cast<const std::uint8_t*>(view.data()) + view.size());
    return out;
}

void readTypedLod(const std::shared_ptr<arrow::Table>& table, std::int64_t index, PaperLodRow& lod) {
    auto required = [&](const char* name, arrow::Type::type type) {
        auto column = getColumn(table, name);
        if (!column || column->type_id() != type || column->IsNull(index)) {
            throw std::runtime_error(std::string("PaperStreamReaderV2: invalid/missing typed column ") + name);
        }
        return column;
    };
    auto binary = [&](const char* name) { return getBinary(required(name, arrow::Type::BINARY), index); };
    lod.F = binary("F"); lod.Et = binary("Et"); lod.Ef = binary("Ef"); lod.V = binary("V");
    auto integers = [&](const char* name) {
        auto list = std::static_pointer_cast<arrow::ListArray>(required(name, arrow::Type::LIST));
        if (list->values()->type_id() != arrow::Type::INT32) {
            throw std::runtime_error(std::string("PaperStreamReaderV2: expected int32 list ") + name);
        }
        auto values = std::static_pointer_cast<arrow::Int32Array>(list->values());
        std::vector<std::int32_t> out;
        for (std::int64_t i = list->value_offset(index); i < list->value_offset(index) + list->value_length(index); ++i) {
            if (values->IsNull(i)) throw std::runtime_error("PaperStreamReaderV2: null geometry component");
            out.push_back(values->Value(i));
        }
        return out;
    };
    auto floats = [&](const char* name) {
        auto list = std::static_pointer_cast<arrow::ListArray>(required(name, arrow::Type::LIST));
        if (list->values()->type_id() != arrow::Type::FLOAT) {
            throw std::runtime_error(std::string("PaperStreamReaderV2: expected float32 list ") + name);
        }
        auto values = std::static_pointer_cast<arrow::FloatArray>(list->values());
        std::vector<float> out;
        for (std::int64_t i = list->value_offset(index); i < list->value_offset(index) + list->value_length(index); ++i) {
            if (values->IsNull(i)) throw std::runtime_error("PaperStreamReaderV2: null geometry component");
            out.push_back(values->Value(i));
        }
        return out;
    };
    lod.CfF = floats("Cf"); lod.CvF = floats("Cv");
    lod.CfQ = integers("CfQ"); lod.CvQ = integers("CvQ");
    auto count = [&](const char* name) {
        const auto value = getInt32(required(name, arrow::Type::INT32), index);
        if (value < 0) throw std::runtime_error("PaperStreamReaderV2: negative typed count");
        return value;
    };
    lod.countF = count("count_F"); lod.countEt = count("count_Et");
    lod.countEf = count("count_Ef"); lod.countV = count("count_V");
    lod.countCf = count("count_Cf"); lod.countCv = count("count_Cv");
    lod.payloadByteOffset = std::static_pointer_cast<arrow::UInt64Array>(required("payload_offset", arrow::Type::UINT64))->Value(index);
    lod.payloadByteLength = std::static_pointer_cast<arrow::UInt64Array>(required("payload_length", arrow::Type::UINT64))->Value(index);
    auto checkGeometry = [](const auto& values, int count) {
        if (!values.empty() && values.size() != static_cast<std::size_t>(count) * 3) {
            throw std::runtime_error("PaperStreamReaderV2: typed geometry count/xyz length mismatch");
        }
    };
    checkGeometry(lod.CfQ, lod.countCf); checkGeometry(lod.CfF, lod.countCf);
    checkGeometry(lod.CvQ, lod.countCv); checkGeometry(lod.CvF, lod.countCv);
    if ((lod.countCf && lod.CfQ.empty() && lod.CfF.empty()) ||
        (lod.countCv && lod.CvQ.empty() && lod.CvF.empty())) {
        throw std::runtime_error("PaperStreamReaderV2: typed geometry count has no values");
    }
}

struct ColumnsV2 {
    std::shared_ptr<arrow::Array> type, objectId, partitionId, lodId;
    std::shared_ptr<arrow::Array> header, baseMesh, connectivity;
    std::shared_ptr<arrow::Array> geometry, extensions, nativePayload, boundary;
};

ColumnsV2 readColumns(const std::shared_ptr<arrow::Table>& table) {
    ColumnsV2 c;
    c.type = getColumn(table, kRecordType);
    c.objectId = getColumn(table, kObjectId);
    c.partitionId = getColumn(table, kPartitionId);
    c.lodId = getColumn(table, kLodId);
    c.header = getColumn(table, kHeader);
    c.baseMesh = getColumn(table, kBaseMesh);
    c.connectivity = getColumn(table, kConnectivity);
    c.geometry = getColumn(table, kGeometry);
    c.extensions = getColumn(table, kExtensions);
    c.nativePayload = getColumn(table, kNativePayload);
    c.boundary = getColumn(table, kBoundary);
    return c;
}

}  // namespace

struct PaperStreamReaderV2::Impl {
    std::string path;
    std::shared_ptr<arrow::io::RandomAccessFile> file;
    std::unique_ptr<parquet::arrow::FileReader> reader;
    parquet::ParquetFileReader* parquet = nullptr; // owned by the Arrow reader
    std::unique_ptr<PaperStreamReader> v1Fallback;
    PaperObjectRow object;
    /// M5-1 single-blob container profile (one `paper_blob` BINARY column).
    std::unique_ptr<PaperStreamTable> blobTable;
    std::uint8_t blobPayloadCodec = 0;
    std::uint8_t blobBaseCodec = 0;
    bool v2 = false;
    std::uint64_t footerBytes = 0;
};

namespace {

/// @brief metadata length + trailing magic = on-disk footer bytes of a Parquet file.
std::uint64_t readParquetFooterBytes(const std::string& path) {
    const auto size = std::filesystem::file_size(path);
    if (size < 12) return 0;
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) return 0;
    ifs.seekg(static_cast<std::streamoff>(size - 8), std::ios::beg);
    char tail[8] = {0};
    ifs.read(tail, 8);
    if (std::memcmp(tail + 4, "PAR1", 4) != 0) return 0;
    std::uint32_t metadataLen = 0;
    std::memcpy(&metadataLen, tail, 4);  // little-endian host (x86_64)
    return static_cast<std::uint64_t>(metadataLen) + 8;
}

}  // namespace

PaperStreamReaderV2::PaperStreamReaderV2(const std::string& path)
    : impl_(std::make_unique<Impl>()) {
    impl_->path = path;
    auto fileResult = arrow::io::ReadableFile::Open(path);
    if (!fileResult.ok()) {
        throw std::runtime_error("PaperStreamReaderV2: cannot open input: " + path + ": " +
                                 fileResult.status().ToString());
    }
    impl_->file = fileResult.ValueOrDie();
    auto openResult = parquet::arrow::OpenFile(impl_->file, arrow::default_memory_pool());
    if (!openResult.ok()) {
        throw std::runtime_error("PaperStreamReaderV2: cannot open Parquet reader: " +
                                 openResult.status().ToString());
    }
    impl_->reader = std::move(openResult).ValueOrDie();
    impl_->parquet = impl_->reader->parquet_reader();
    if (!impl_->parquet) {
        throw std::runtime_error("PaperStreamReaderV2: cannot open Parquet metadata: " + path);
    }
    impl_->footerBytes = readParquetFooterBytes(path);
    footerBytes_ = impl_->footerBytes;

    if (impl_->reader->num_row_groups() <= 0) {
        throw std::runtime_error("PaperStreamReaderV2: no row groups in " + path);
    }
    std::shared_ptr<arrow::Table> objectTable;
    auto objectStatus = impl_->reader->ReadRowGroup(0, &objectTable);
    if (!objectStatus.ok()) {
        throw std::runtime_error("PaperStreamReaderV2: object RG read failed: " +
                                 objectStatus.ToString());
    }

    // M5-1 zero-overhead container profile: single `paper_blob` BINARY column.
    if (objectTable && objectTable->schema()->GetFieldIndex("paper_blob") >= 0) {
        const auto blobColumn = getColumn(objectTable, "paper_blob");
        if (!blobColumn || objectTable->num_rows() < 1 || blobColumn->IsNull(0)) {
            throw std::runtime_error("PaperStreamReaderV2: empty paper_blob container");
        }
        const auto blobBytes = getBinary(blobColumn, 0);
        auto blobTable = std::make_unique<PaperStreamTable>();
        std::string blobError;
        if (!parsePaperStreamV2BlobContainer(blobBytes, *blobTable, &blobError,
                                              &impl_->blobPayloadCodec,
                                              &impl_->blobBaseCodec)) {
            throw std::runtime_error("PaperStreamReaderV2: bad paper_blob container: " +
                                     blobError);
        }
        impl_->object = blobTable->object;
        impl_->blobTable = std::move(blobTable);
        impl_->v2 = true;
        return;
    }

    const auto headerColumn = getColumn(objectTable, kHeader);
    std::string headerText;
    if (headerColumn && objectTable->num_rows() > 0 && !headerColumn->IsNull(0)) {
        const auto bytes = getBinary(headerColumn, 0);
        headerText.assign(bytes.begin(), bytes.end());
    }
    if (headerText.empty()) {
        throw std::runtime_error("PaperStreamReaderV2: object row has no dictionary header");
    }

    // v1/v2 dispatch: the dictionary version is the only authoritative switch.
    std::int32_t version = 0;
    try {
        version = nlohmann::json::parse(headerText).value("version", 0);
    } catch (const std::exception& e) {
        throw std::runtime_error("PaperStreamReaderV2: bad dictionary JSON: " +
                                 std::string(e.what()));
    }
    if (version != kPaperStreamFormatVersionV2) {
        impl_->v1Fallback = std::make_unique<PaperStreamReader>(path);
        return;
    }

    std::string error;
    if (!parsePaperStreamDictionaryV2(headerText, impl_->object, &error)) {
        throw std::runtime_error("PaperStreamReaderV2: bad v2 dictionary: " + error);
    }
    if (impl_->object.formatVersion != kPaperStreamFormatVersionV2) {
        throw std::runtime_error("PaperStreamReaderV2: dictionary version mismatch");
    }
    impl_->v2 = true;
}

PaperStreamReaderV2::~PaperStreamReaderV2() = default;

int PaperStreamReaderV2::numRowGroups() const {
    if (impl_->v1Fallback) return impl_->v1Fallback->numRowGroups();
    return impl_->reader ? impl_->reader->num_row_groups() : 0;
}

bool PaperStreamReaderV2::usedV1Fallback() const {
    return impl_ && impl_->v1Fallback != nullptr;
}

const PaperObjectRow* PaperStreamReaderV2::v2Object() const {
    return impl_ && impl_->v2 ? &impl_->object : nullptr;
}

bool PaperStreamReaderV2::isBlobContainer() const {
    return impl_ && impl_->blobTable != nullptr;
}

std::uint8_t PaperStreamReaderV2::blobPayloadCodec() const {
    return impl_ ? impl_->blobPayloadCodec : 0;
}

std::uint8_t PaperStreamReaderV2::blobBaseCodec() const {
    return impl_ ? impl_->blobBaseCodec : 0;
}

std::string PaperStreamReaderV2::granularity() const {
    if (impl_->v1Fallback) return "v1";
    if (impl_->blobTable) return "partition";  // whole-frame blob: partition-level RA
    if (impl_->object.payloadMode == PaperPayloadMode::Native &&
        (impl_->object.nativeFrame == PaperNativeFrame::Whole ||
         impl_->object.nativeFrame == PaperNativeFrame::NamedPayload)) return "partition";
    if (impl_->object.lodsPerRowGroup > 0) return "lod_block";
    return "partition";
}

namespace {

void consumeV2Rows(const std::shared_ptr<arrow::Table>& table, PaperStreamTable& out,
                   const PaperObjectRow& object, int maxLod,
                   const std::set<std::int32_t>* partitionFilter, bool readBoundaries) {
    const auto columns = readColumns(table);
    const bool wholeFrame = object.nativeFrame == PaperNativeFrame::Whole ||
                            object.nativeFrame == PaperNativeFrame::NamedPayload;
    const std::int64_t rows = table->num_rows();
    for (std::int64_t i = 0; i < rows; ++i) {
        const auto type = static_cast<PaperRecordType>(getInt8(columns.type, i));
        if (type == PaperRecordType::Object) {
            out.object = object;
            continue;
        }
        if (type == PaperRecordType::Partition) {
            const std::int32_t partitionId = getInt32(columns.partitionId, i);
            if (partitionFilter && partitionFilter->find(partitionId) == partitionFilter->end()) {
                continue;
            }
            PaperPartitionRow partition;
            partition.objectId = getInt32(columns.objectId, i);
            partition.partitionId = partitionId;
            const auto header = getBinary(columns.header, i);
            if (header.empty()) {
                throw std::runtime_error("PaperStreamReaderV2: partition row without header");
            }
            std::string error;
            if (!unpackPaperPartitionHeaderV2(header, partition, &error)) {
                throw std::runtime_error("PaperStreamReaderV2: bad partition header: " + error);
            }
            partition.baseMesh = getBinary(columns.baseMesh, i);
            if (wholeFrame) {
                const auto native = getBinary(columns.nativePayload, i);
                if (!native.empty()) {
                    PaperPayloadScope scope = PaperPayloadScope::Object;
                    PaperNativeFrame frame = PaperNativeFrame::Whole;
                    std::uint64_t offset = 0;
                    std::uint64_t length = 0;
                    std::string nativeError;
                    if (!unpackPaperNativePayloadV2(native, partition.nativePayload, scope,
                                                    frame, offset, length, &nativeError)) {
                        throw std::runtime_error("PaperStreamReaderV2: bad partition native payload: " +
                                                 nativeError);
                    }
                    if (scope != PaperPayloadScope::Partition || frame != object.nativeFrame) {
                        throw std::runtime_error(
                            "PaperStreamReaderV2: partition native payload frame/scope mismatch");
                    }
                }
            }
            out.partitions.push_back(std::move(partition));
            continue;
        }
        if (type == PaperRecordType::Lod) {
            const std::int32_t partitionId = getInt32(columns.partitionId, i);
            if (partitionFilter && partitionFilter->find(partitionId) == partitionFilter->end()) {
                continue;
            }
            const std::int32_t lodId = getInt32(columns.lodId, i);
            if (maxLod >= 0 && lodId >= maxLod) continue;
            PaperLodRow lod;
            lod.objectId = getInt32(columns.objectId, i);
            lod.partitionId = partitionId;
            lod.lodId = lodId;
            if (object.schemaMode == PaperSchemaMode::Typed) {
                readTypedLod(table, i, lod);
            } else {
            const auto connectivity = getBinary(columns.connectivity, i);
            if (!connectivity.empty()) {
                std::string error;
                if (!unpackPaperConnectivity(connectivity, lod, &error)) {
                    throw std::runtime_error("PaperStreamReaderV2: bad connectivity: " + error);
                }
            }
            const auto geometry = getBinary(columns.geometry, i);
            if (!geometry.empty()) {
                std::string error;
                if (!unpackPaperGeometry(geometry, lod, &error)) {
                    throw std::runtime_error("PaperStreamReaderV2: bad geometry: " + error);
                }
            }
            }
            const auto extensions = getBinary(columns.extensions, i);
            if (!extensions.empty()) {
                std::string error;
                if (!unpackPaperExtensions(extensions, lod, &error)) {
                    throw std::runtime_error("PaperStreamReaderV2: bad extensions: " + error);
                }
            }
            const auto native = getBinary(columns.nativePayload, i);
            if (!native.empty()) {
                PaperPayloadScope scope = PaperPayloadScope::Object;
                PaperNativeFrame frame = PaperNativeFrame::Whole;
                std::uint64_t offset = 0;
                std::uint64_t length = 0;
                std::string error;
                if (!unpackPaperNativePayloadV2(native, lod.nativePayload, scope, frame, offset,
                                                length, &error)) {
                    throw std::runtime_error("PaperStreamReaderV2: bad lod native payload: " +
                                             error);
                }
                if (scope != PaperPayloadScope::Lod || frame != object.nativeFrame) {
                    throw std::runtime_error(
                        "PaperStreamReaderV2: lod native payload frame/scope mismatch");
                }
                if (object.schemaMode == PaperSchemaMode::Typed &&
                    (lod.payloadByteOffset != offset || lod.payloadByteLength != length)) {
                    throw std::runtime_error("PaperStreamReaderV2: physical/native payload directory mismatch");
                }
                lod.payloadByteOffset = offset;
                lod.payloadByteLength = length;
            }
            out.lods.push_back(std::move(lod));
            continue;
        }
        if (type == PaperRecordType::Boundary) {
            if (!readBoundaries) continue;
            PaperBoundaryRow boundary;
            boundary.objectId = getInt32(columns.objectId, i);
            const auto bytes = getBinary(columns.boundary, i);
            if (bytes.empty()) continue;
            std::string error;
            if (!unpackPaperBoundary(bytes, boundary, &error)) {
                throw std::runtime_error("PaperStreamReaderV2: bad boundary: " + error);
            }
            boundary.codec = object.boundaryCodec;
            out.boundaries.push_back(std::move(boundary));
            continue;
        }
        throw std::runtime_error(
            "PaperStreamReaderV2: record_type " +
            std::to_string(static_cast<int>(type)) +
            " is not part of the M4-2 compact schema (typed relation rows are M4-6)");
    }
}

}  // namespace

PaperStreamTable PaperStreamReaderV2::readAll() {
    bytesRead_ = 0;
    rowGroupsRead_ = 0;
    pagesRead_ = 0;
    footerBytes_ = impl_->footerBytes;
    if (impl_->v1Fallback) {
        PaperStreamTable out = impl_->v1Fallback->readAll();
        bytesRead_ = impl_->v1Fallback->bytesRead();
        rowGroupsRead_ = impl_->v1Fallback->rowGroupsRead();
        footerBytes_ = 0;
        return out;
    }
    if (impl_->blobTable) {
        try {
            bytesRead_ = std::filesystem::file_size(impl_->path);
        } catch (...) {
            bytesRead_ = 0;
        }
        rowGroupsRead_ = 1;
        footerBytes_ = impl_->footerBytes;
        return *impl_->blobTable;
    }
    PaperStreamTable out;
    const int groups = impl_->reader->num_row_groups();
    for (int i = 0; i < groups; ++i) {
        std::shared_ptr<arrow::Table> table;
        auto st = impl_->reader->ReadRowGroup(i, &table);
        if (!st.ok()) {
            throw std::runtime_error("PaperStreamReaderV2: ReadRowGroup failed: " + st.ToString());
        }
        consumeV2Rows(table, out, impl_->object, -1, nullptr, true);
        if (impl_->parquet && impl_->parquet->metadata()) {
            bytesRead_ += static_cast<std::uint64_t>(
                impl_->parquet->metadata()->RowGroup(i)->total_compressed_size());
        }
        ++rowGroupsRead_;
    }
    try {
        bytesRead_ = std::filesystem::file_size(impl_->path);
    } catch (...) {
        // keep the row-group sum
    }
    return out;
}

PaperStreamTable PaperStreamReaderV2::readPrefix(int partitionId, int maxLod) {
    return readPrefix(std::vector<int>{partitionId}, maxLod);
}

PaperStreamTable PaperStreamReaderV2::readPrefix(const std::vector<int>& partitionIds,
                                                 int maxLod) {
    bytesRead_ = 0;
    rowGroupsRead_ = 0;
    pagesRead_ = 0;
    footerBytes_ = impl_->footerBytes;
    if (impl_->v1Fallback) {
        if (partitionIds.size() != 1) {
            throw std::runtime_error(
                "PaperStreamReaderV2: v1 fallback supports only one partition in readPrefix");
        }
        PaperStreamTable out = impl_->v1Fallback->readPrefix(partitionIds[0], maxLod);
        bytesRead_ = impl_->v1Fallback->bytesRead();
        rowGroupsRead_ = impl_->v1Fallback->rowGroupsRead();
        footerBytes_ = 0;
        return out;
    }
    if (impl_->blobTable) {
        PaperStreamTable out = *impl_->blobTable;
        std::set<std::int32_t> wanted(partitionIds.begin(), partitionIds.end());
        out.partitions.clear();
        for (const auto& partition : impl_->blobTable->partitions) {
            if (wanted.count(partition.partitionId) != 0) {
                out.partitions.push_back(partition);
            }
        }
        if (out.partitions.empty()) {
            throw std::runtime_error(
                "PaperStreamReaderV2: no partition matches the prefix request");
        }
        std::vector<PaperLodRow> rows;
        for (const auto& lod : impl_->blobTable->lods) {
            if (wanted.count(lod.partitionId) == 0) continue;
            if (maxLod >= 0 && lod.lodId >= maxLod) continue;
            rows.push_back(lod);
        }
        out.lods = std::move(rows);
        std::uint64_t compressed = 0;
        if (impl_->parquet && impl_->parquet->metadata() &&
            impl_->parquet->metadata()->num_row_groups() > 0) {
            compressed = static_cast<std::uint64_t>(
                impl_->parquet->metadata()->RowGroup(0)->total_compressed_size());
        }
        bytesRead_ = compressed + impl_->footerBytes;
        rowGroupsRead_ = 1;
        footerBytes_ = impl_->footerBytes;
        return out;
    }
    return readPrefixV2(partitionIds, maxLod);
}

PaperStreamTable PaperStreamReaderV2::readPrefixV2(const std::vector<int>& partitionIds,
                                                   int maxLod) {
    PaperStreamTable out;
    out.object = impl_->object;
    if (partitionIds.empty()) {
        bytesRead_ = footerBytes_;
        return out;
    }
    std::set<std::int32_t> wanted(partitionIds.begin(), partitionIds.end());
    std::map<std::int32_t, const PaperPartitionDirectoryEntry*> directory;
    for (const auto& entry : impl_->object.partitionRgDirectory) {
        directory[entry.partitionId] = &entry;
    }

    std::set<int> selectedRgs;
    selectedRgs.insert(0);  // object row group
    for (std::int32_t partitionId : wanted) {
        const auto it = directory.find(partitionId);
        if (it == directory.end()) {
            throw std::runtime_error(
                "PaperStreamReaderV2: no row-group directory entry for partition " +
                std::to_string(partitionId));
        }
        const auto* entry = it->second;
        const std::int32_t blockSize = impl_->object.lodsPerRowGroup;
        std::int32_t blocks = 1;
        if (blockSize > 0) {
            blocks = entry->numLods == 0
                         ? 1
                         : (entry->numLods + blockSize - 1) / blockSize;
        }
        if (maxLod >= 0 && blockSize > 0) {
            // The first block also contains the partition base row, so a
            // base-only query must still read it. Later blocks can be skipped
            // entirely instead of reading every LOD and filtering in memory.
            const std::int32_t wantedBlocks = maxLod == 0 ? 1 : 1 + (maxLod - 1) / blockSize;
            blocks = std::min(blocks, wantedBlocks);
        }
        for (std::int32_t k = 0; k < blocks; ++k) {
            selectedRgs.insert(entry->rgIndex + k);
        }
    }

    const int groups = impl_->reader->num_row_groups();
    for (int rg : selectedRgs) {
        if (rg < 0 || rg >= groups) {
            throw std::runtime_error("PaperStreamReaderV2: row-group directory out of bounds");
        }
        std::shared_ptr<arrow::Table> table;
        auto st = impl_->reader->ReadRowGroup(rg, &table);
        if (!st.ok()) {
            throw std::runtime_error("PaperStreamReaderV2: ReadRowGroup failed: " + st.ToString());
        }
        consumeV2Rows(table, out, impl_->object, maxLod, &wanted, false);
        if (impl_->parquet && impl_->parquet->metadata()) {
            bytesRead_ += static_cast<std::uint64_t>(
                impl_->parquet->metadata()->RowGroup(rg)->total_compressed_size());
        }
        ++rowGroupsRead_;
    }
    bytesRead_ += footerBytes_;
    // Deterministic ordering of the returned rows.
    std::stable_sort(out.partitions.begin(), out.partitions.end(),
                     [](const PaperPartitionRow& a, const PaperPartitionRow& b) {
                         return a.partitionId < b.partitionId;
                     });
    std::stable_sort(out.lods.begin(), out.lods.end(),
                     [](const PaperLodRow& a, const PaperLodRow& b) {
                         if (a.partitionId != b.partitionId) return a.partitionId < b.partitionId;
                         return a.lodId < b.lodId;
                     });
    return out;
}

}  // namespace pmsf
