/*
 * @file PaperStreamWriterV2.cpp
 * @brief PaperStream v2 writer implementation (M4-2, opt-in; see header).
 */
#include "storage/PaperStreamWriterV2.h"

#include "storage/PaperStreamBlobContainerV2.h"
#include "storage/PaperStreamCodecV2.h"
#include "storage/PaperStreamCompactCodec.h"  // v1 PCON/PGEO/PEXT/PBDY bundles
#include "storage/PaperStreamDictionaryV2.h"

#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/writer.h>
#include <parquet/properties.h>

#include <algorithm>
#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
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

struct RowRefV2 {
    PaperRecordType type = PaperRecordType::Object;
    const PaperObjectRow* object = nullptr;
    const PaperPartitionRow* partition = nullptr;
    const PaperLodRow* lod = nullptr;
    const PaperBoundaryRow* boundary = nullptr;
};

template <typename Builder>
void appendInt(Builder& builder, bool valid, std::int64_t value) {
    auto st = valid ? builder.Append(static_cast<typename Builder::value_type>(value))
                    : builder.AppendNull();
    if (!st.ok()) throw std::runtime_error("PaperStreamV2: int builder append failed");
}

void appendBinary(arrow::BinaryBuilder& builder, bool valid,
                  const std::vector<std::uint8_t>& bytes) {
    auto st = valid
                  ? builder.Append(reinterpret_cast<const char*>(bytes.data()),
                                   static_cast<std::int32_t>(bytes.size()))
                  : builder.AppendNull();
    if (!st.ok()) throw std::runtime_error("PaperStreamV2: binary builder append failed");
}

std::shared_ptr<arrow::Schema> buildSchemaV2(bool typed) {
    std::vector<std::shared_ptr<arrow::Field>> fields;
    fields.push_back(arrow::field(kRecordType, arrow::int8(), false));
    fields.push_back(arrow::field(kObjectId, arrow::int32(), false));
    fields.push_back(arrow::field(kPartitionId, arrow::int32(), false));
    fields.push_back(arrow::field(kLodId, arrow::int32(), false));
    fields.push_back(arrow::field(kHeader, arrow::binary(), true));
    fields.push_back(arrow::field(kBaseMesh, arrow::binary(), true));
    if (typed) {
        // One encoded connectivity stream per role; geometry is a typed list
        // per LOD. No object/partition/LOD id is repeated per symbol.
        for (const char* role : {"F", "Et", "Ef", "V"}) {
            fields.push_back(arrow::field(role, arrow::binary(), true));
        }
        for (const char* role : {"Cf", "Cv"}) {
            fields.push_back(arrow::field(role, arrow::list(arrow::field("element", arrow::float32())), true));
        }
        for (const char* role : {"CfQ", "CvQ"}) {
            fields.push_back(arrow::field(role, arrow::list(arrow::field("element", arrow::int32())), true));
        }
        for (const char* name : {"count_F", "count_Et", "count_Ef", "count_V", "count_Cf", "count_Cv"}) {
            fields.push_back(arrow::field(name, arrow::int32(), true));
        }
        fields.push_back(arrow::field("payload_offset", arrow::uint64(), true));
        fields.push_back(arrow::field("payload_length", arrow::uint64(), true));
    } else {
        fields.push_back(arrow::field(kConnectivity, arrow::binary(), true));
        fields.push_back(arrow::field(kGeometry, arrow::binary(), true));
    }
    fields.push_back(arrow::field(kExtensions, arrow::binary(), true));
    fields.push_back(arrow::field(kNativePayload, arrow::binary(), true));
    fields.push_back(arrow::field(kBoundary, arrow::binary(), true));
    return arrow::schema(fields);
}

std::vector<std::uint8_t> nativePayloadColumnValue(const RowRefV2& row,
                                                   PaperNativeFrame frame) {
    if (row.lod) {
        if (frame == PaperNativeFrame::ConnRound ||
            frame == PaperNativeFrame::ClassIStream) {
            return packPaperNativePayloadV2(row.lod->nativePayload,
                                            PaperPayloadScope::Lod, frame,
                                            row.lod->payloadByteOffset,
                                            row.lod->payloadByteLength);
        }
        return {};
    }
    if (row.partition) {
        if (frame == PaperNativeFrame::Whole ||
            frame == PaperNativeFrame::NamedPayload) {
            return packPaperNativePayloadV2(
                row.partition->nativePayload, PaperPayloadScope::Partition, frame, 0,
                row.partition->nativePayload.size());
        }
    }
    return {};
}

std::shared_ptr<arrow::Table> buildTableV2(const std::shared_ptr<arrow::Schema>& schema,
                                           const std::vector<RowRefV2>& rows,
                                           const std::string& dictionary,
                                           PaperNativeFrame frame, bool typed) {
    const std::size_t n = rows.size();
    std::vector<std::shared_ptr<arrow::Array>> arrays;
    arrays.reserve(11);

    {
        arrow::Int8Builder b;
        for (const auto& row : rows) appendInt(b, true, static_cast<std::int8_t>(row.type));
        std::shared_ptr<arrow::Array> a;
        if (!b.Finish(&a).ok()) throw std::runtime_error("PaperStreamV2: record_type finish");
        arrays.push_back(a);
    }
    {
        arrow::Int32Builder b;
        for (const auto& row : rows) {
            const std::int32_t v = row.object   ? row.object->objectId
                                   : row.partition ? row.partition->objectId
                                   : row.lod       ? row.lod->objectId
                                                  : row.boundary->objectId;
            appendInt(b, true, v);
        }
        std::shared_ptr<arrow::Array> a;
        if (!b.Finish(&a).ok()) throw std::runtime_error("PaperStreamV2: object_id finish");
        arrays.push_back(a);
    }
    {
        arrow::Int32Builder b;
        for (const auto& row : rows) {
            const std::int32_t v = row.partition ? row.partition->partitionId
                                   : row.lod       ? row.lod->partitionId
                                                  : -1;
            appendInt(b, true, v);
        }
        std::shared_ptr<arrow::Array> a;
        if (!b.Finish(&a).ok()) throw std::runtime_error("PaperStreamV2: partition_id finish");
        arrays.push_back(a);
    }
    {
        arrow::Int32Builder b;
        for (const auto& row : rows) appendInt(b, true, row.lod ? row.lod->lodId : -1);
        std::shared_ptr<arrow::Array> a;
        if (!b.Finish(&a).ok()) throw std::runtime_error("PaperStreamV2: lod_id finish");
        arrays.push_back(a);
    }

    auto addBinary = [&](const auto& packFn) {
        arrow::BinaryBuilder b;
        for (const auto& row : rows) {
            std::vector<std::uint8_t> bytes;
            const bool has = packFn(row, bytes);
            appendBinary(b, has, bytes);
        }
        std::shared_ptr<arrow::Array> a;
        if (!b.Finish(&a).ok()) throw std::runtime_error("PaperStreamV2: binary finish");
        arrays.push_back(a);
    };

    addBinary([&](const RowRefV2& row, std::vector<std::uint8_t>& out) {
        if (row.object) {
            out.assign(dictionary.begin(), dictionary.end());
            return true;
        }
        if (row.partition) {
            out = packPaperPartitionHeaderV2(*row.partition);
            return true;
        }
        return false;
    });
    addBinary([&](const RowRefV2& row, std::vector<std::uint8_t>& out) {
        if (!row.partition) return false;
        out = row.partition->baseMesh;
        return true;
    });
    if (typed) {
        for (auto member : {&PaperLodRow::F, &PaperLodRow::Et, &PaperLodRow::Ef, &PaperLodRow::V}) {
            addBinary([&](const RowRefV2& row, std::vector<std::uint8_t>& out) {
                if (!row.lod) return false;
                out = row.lod->*member;
                return true;
            });
        }
        auto addList = [&](auto member, auto valueBuilder) {
            arrow::ListBuilder lists(arrow::default_memory_pool(), valueBuilder,
                arrow::list(arrow::field("element", valueBuilder->type())));
            for (const auto& row : rows) {
                auto status = row.lod ? lists.Append() : lists.AppendNull();
                if (!status.ok()) throw std::runtime_error("PaperStreamV2: list append failed");
                if (!row.lod) continue;
                const auto& values = row.lod->*member;
                status = valueBuilder->AppendValues(values.data(), static_cast<std::int64_t>(values.size()));
                if (!status.ok()) throw std::runtime_error("PaperStreamV2: list values append failed");
            }
            std::shared_ptr<arrow::Array> array;
            if (!lists.Finish(&array).ok()) throw std::runtime_error("PaperStreamV2: list finish failed");
            arrays.push_back(std::move(array));
        };
        addList(&PaperLodRow::CfF, std::make_shared<arrow::FloatBuilder>());
        addList(&PaperLodRow::CvF, std::make_shared<arrow::FloatBuilder>());
        addList(&PaperLodRow::CfQ, std::make_shared<arrow::Int32Builder>());
        addList(&PaperLodRow::CvQ, std::make_shared<arrow::Int32Builder>());
        for (auto member : {&PaperLodRow::countF, &PaperLodRow::countEt, &PaperLodRow::countEf,
                            &PaperLodRow::countV, &PaperLodRow::countCf, &PaperLodRow::countCv}) {
            arrow::Int32Builder builder;
            for (const auto& row : rows) appendInt(builder, row.lod != nullptr, row.lod ? row.lod->*member : 0);
            std::shared_ptr<arrow::Array> array;
            if (!builder.Finish(&array).ok()) throw std::runtime_error("PaperStreamV2: counter finish failed");
            arrays.push_back(std::move(array));
        }
        for (auto member : {&PaperLodRow::payloadByteOffset, &PaperLodRow::payloadByteLength}) {
            arrow::UInt64Builder builder;
            for (const auto& row : rows) appendInt(builder, row.lod != nullptr, row.lod ? row.lod->*member : 0);
            std::shared_ptr<arrow::Array> array;
            if (!builder.Finish(&array).ok()) throw std::runtime_error("PaperStreamV2: directory finish failed");
            arrays.push_back(std::move(array));
        }
    } else {
    addBinary([&](const RowRefV2& row, std::vector<std::uint8_t>& out) {
        if (!row.lod) return false;
        const bool typed = row.lod->countF != 0 || !row.lod->F.empty() ||
                           !row.lod->Et.empty() || !row.lod->Ef.empty() ||
                           !row.lod->V.empty();
        if (!typed) return false;
        out = packPaperConnectivity(*row.lod);
        return true;
    });
    addBinary([&](const RowRefV2& row, std::vector<std::uint8_t>& out) {
        if (!row.lod) return false;
        const bool typed = !row.lod->CfQ.empty() || !row.lod->CvQ.empty() ||
                           !row.lod->CfF.empty() || !row.lod->CvF.empty() ||
                           row.lod->countCf != 0 || row.lod->countCv != 0;
        if (!typed) return false;
        out = packPaperGeometry(*row.lod);
        return true;
    });
    }
    addBinary([&](const RowRefV2& row, std::vector<std::uint8_t>& out) {
        if (!row.lod || row.lod->extension.empty()) return false;
        out = packPaperExtensions(*row.lod);
        return true;
    });
    addBinary([&](const RowRefV2& row, std::vector<std::uint8_t>& out) {
        std::vector<std::uint8_t> value = nativePayloadColumnValue(row, frame);
        if (value.empty()) return false;
        out = std::move(value);
        return true;
    });
    addBinary([&](const RowRefV2& row, std::vector<std::uint8_t>& out) {
        if (!row.boundary) return false;
        out = packPaperBoundary(*row.boundary);
        return true;
    });

    if (arrays.size() != static_cast<std::size_t>(schema->num_fields())) {
        throw std::runtime_error("PaperStreamV2: column count mismatch");
    }
    return arrow::Table::Make(schema, arrays, static_cast<std::int64_t>(n));
}

std::shared_ptr<parquet::WriterProperties> buildWriterProperties(
    const PaperWriterV2Options& options, bool typed) {
    auto builder = parquet::WriterProperties::Builder();
    builder.compression(parquet::Compression::BROTLI);
    // NOTE: no global compression_level: Parquet rejects a level for the
    // UNCOMPRESSED native-payload column; BROTLI then uses the codec default.
    builder.data_pagesize(static_cast<int64_t>(options.dataPageSize));
    builder.write_batch_size(1024);
    builder.disable_statistics();
    // Native payload is already entropy-coded; never re-compress it (that only
    // adds framing). Typed bundles/header/base mesh keep BROTLI.
    builder.compression(kNativePayload, parquet::Compression::UNCOMPRESSED);
    for (const char* name : {kObjectId, kPartitionId, kLodId}) {
        builder.encoding(name, parquet::Encoding::DELTA_BINARY_PACKED);
    }
    for (const char* name : {kHeader, kBaseMesh, kConnectivity, kGeometry,
                             kExtensions, kNativePayload, kBoundary}) {
        builder.disable_dictionary(name);
        builder.encoding(name, parquet::Encoding::PLAIN);
    }
    if (typed) {
        builder.enable_statistics(kLodId);
        for (const char* path : {"Cf.list.element", "Cv.list.element"}) {
            builder.disable_dictionary(path);
            builder.encoding(path, parquet::Encoding::BYTE_STREAM_SPLIT);
            builder.compression(path, parquet::Compression::ZSTD);
        }
        for (const char* path : {"CfQ.list.element", "CvQ.list.element"}) {
            builder.disable_dictionary(path);
            builder.encoding(path, parquet::Encoding::DELTA_BINARY_PACKED);
            builder.compression(path, parquet::Compression::ZSTD);
        }
    }
    return builder.build();
}

void writeTable(parquet::arrow::FileWriter& writer,
                const std::shared_ptr<arrow::Table>& table) {
    if (!table || table->num_rows() == 0) return;
    auto st = writer.WriteTable(*table, table->num_rows());
    if (!st.ok()) throw std::runtime_error("PaperStreamV2: WriteTable failed: " + st.ToString());
}

}  // namespace

void PaperStreamWriterV2::assignDecodeOrderLodIds(std::vector<PaperLodRow>& rows,
                                                  std::int32_t numRounds) {
    for (std::size_t i = 0; i < rows.size(); ++i) {
        // rows are encoder order (row i = encoder round i); paper lod_id is the
        // decoder order.
        rows[i].lodId = paperLodIdFromEncoderRound(static_cast<std::int32_t>(i), numRounds);
    }
    std::stable_sort(rows.begin(), rows.end(),
                     [](const PaperLodRow& a, const PaperLodRow& b) {
                         return a.lodId < b.lodId;
                     });
}

void PaperStreamWriterV2::write(const PaperStreamTable& table, const std::string& path,
                                const PaperWriterV2Options& options) const {
    if (table.object.formatVersion != kPaperStreamFormatVersionV2) {
        throw std::runtime_error(
            "PaperStreamWriterV2: requires object.formatVersion=" +
            std::to_string(kPaperStreamFormatVersionV2) + " (got " +
            std::to_string(table.object.formatVersion) +
            "); the v1 bridge writer must be used for v1 tables");
    }
    if (options.partitionsPerRowGroup < 1) {
        throw std::runtime_error("PaperStreamWriterV2: partitionsPerRowGroup must be >= 1");
    }
    if (options.lodsPerRowGroup < 0) {
        throw std::runtime_error("PaperStreamWriterV2: lodsPerRowGroup must be >= 0");
    }

    // Copy the table header so the computed row-group directory can be recorded
    // in the dictionary without touching the caller's object.
    PaperStreamTable normalized = table;
    normalized.object.formatVersion = kPaperStreamFormatVersionV2;
    const bool typed = normalized.object.schemaMode == PaperSchemaMode::Typed;
    normalized.object.partitionsPerRowGroup = options.partitionsPerRowGroup;
    normalized.object.lodsPerRowGroup = options.lodsPerRowGroup;
    normalized.object.pageIndex = false;
    if (normalized.object.baseMeshCodec.empty() && !normalized.partitions.empty()) {
        normalized.object.baseMeshCodec = normalized.partitions.front().baseMeshCodec;
    }

    // Deterministic partition/LOD order regardless of caller vector order.
    std::stable_sort(normalized.partitions.begin(), normalized.partitions.end(),
                     [](const PaperPartitionRow& a, const PaperPartitionRow& b) {
                         return a.partitionId < b.partitionId;
                     });
    std::stable_sort(normalized.lods.begin(), normalized.lods.end(),
                     [](const PaperLodRow& a, const PaperLodRow& b) {
                         if (a.partitionId != b.partitionId) return a.partitionId < b.partitionId;
                         return a.lodId < b.lodId;
                     });

    std::string error;
    if (!paperValidateStreamTable(normalized, &error)) {
        throw std::runtime_error("PaperStreamWriterV2: invalid v2 table: " + error);
    }

    if (typed) {
        if (options.container != PaperV2Container::Columns) {
            throw std::runtime_error("PaperStreamWriterV2: typed relations require the Columns container");
        }
        for (const char* role : {"F", "Et", "Ef", "V", "Cf", "Cv", "CfQ", "CvQ"}) {
            if (std::none_of(normalized.object.streams.begin(), normalized.object.streams.end(),
                    [&](const PaperStreamSpec& spec) { return spec.name == role; })) {
                throw std::runtime_error(std::string("PaperStreamWriterV2: typed relations missing declared role ") + role);
            }
        }
        for (const auto& lod : normalized.lods) {
            if (lod.countF < 0 || lod.countEt < 0 || lod.countEf < 0 || lod.countV < 0 || lod.countCf < 0 || lod.countCv < 0) {
                throw std::runtime_error("PaperStreamWriterV2: negative typed stream counter");
            }
            auto checkGeometry = [](const auto& values, int count) {
                if (!values.empty() && values.size() != static_cast<std::size_t>(count) * 3) {
                    throw std::runtime_error("PaperStreamWriterV2: typed geometry count/xyz length mismatch");
                }
            };
            checkGeometry(lod.CfQ, lod.countCf); checkGeometry(lod.CfF, lod.countCf);
            checkGeometry(lod.CvQ, lod.countCv); checkGeometry(lod.CvF, lod.countCv);
            if ((lod.countCf && lod.CfQ.empty() && lod.CfF.empty()) ||
                (lod.countCv && lod.CvQ.empty() && lod.CvF.empty())) {
                throw std::runtime_error("PaperStreamWriterV2: typed geometry counter has no values");
            }
        }
    }

    // Frame-specific fail-closed checks (validator is frame-agnostic for payloads).
    const PaperNativeFrame frame = normalized.object.nativeFrame;
    if (normalized.object.payloadMode != PaperPayloadMode::Typed) {
        if (frame == PaperNativeFrame::Whole ||
            frame == PaperNativeFrame::NamedPayload) {
            for (const auto& partition : normalized.partitions) {
                if (partition.numLods > 0 && partition.nativePayload.empty()) {
                    throw std::runtime_error(
                        "PaperStreamWriterV2: frame=" +
                        std::string(paperNativeFrameName(frame)) +
                        " requires a partition-level native payload (partition " +
                        std::to_string(partition.partitionId) + ")");
                }
            }
        } else {
            for (const auto& lod : normalized.lods) {
                if (lod.payloadByteLength != 0 &&
                    lod.payloadByteLength != lod.nativePayload.size()) {
                    throw std::runtime_error(
                        "PaperStreamWriterV2: lod payloadByteLength mismatch (partition " +
                        std::to_string(lod.partitionId) + " lod " +
                        std::to_string(lod.lodId) + ")");
                }
            }
        }
    }

    // Plan row groups first: the object-row dictionary must carry the final
    // partition -> row-group directory.
    struct PlannedRg {
        std::vector<RowRefV2> rows;
    };
    std::vector<PlannedRg> plan;
    std::map<std::int32_t, std::vector<const PaperLodRow*>> lodsByPartition;
    for (const auto& lod : normalized.lods) lodsByPartition[lod.partitionId].push_back(&lod);

    normalized.object.partitionRgDirectory.clear();
    for (std::size_t start = 0; start < normalized.partitions.size();
         start += static_cast<std::size_t>(options.partitionsPerRowGroup)) {
        const std::size_t end = std::min(
            normalized.partitions.size(),
            start + static_cast<std::size_t>(options.partitionsPerRowGroup));
        for (std::size_t i = start; i < end; ++i) {
            const auto& partition = normalized.partitions[i];
            const auto it = lodsByPartition.find(partition.partitionId);
            const std::vector<const PaperLodRow*> emptyLods;
            const auto& lods = it == lodsByPartition.end() ? emptyLods : it->second;
            PaperPartitionDirectoryEntry entry;
            entry.partitionId = partition.partitionId;
            entry.rgIndex = static_cast<std::int32_t>(1 + plan.size());
            entry.rowIndex = 0;
            entry.numLods = partition.numLods;
            std::int64_t payloadBytes = 0;
            for (const auto* lod : lods) payloadBytes += static_cast<std::int64_t>(lod->nativePayload.size());
            payloadBytes += static_cast<std::int64_t>(partition.nativePayload.size());
            entry.payloadBytes = payloadBytes;
            normalized.object.partitionRgDirectory.push_back(entry);
            if (options.lodsPerRowGroup <= 0) {
                PlannedRg rg;
                RowRefV2 p;
                p.type = PaperRecordType::Partition;
                p.partition = &partition;
                rg.rows.push_back(p);
                for (const auto* lod : lods) {
                    RowRefV2 r;
                    r.type = PaperRecordType::Lod;
                    r.lod = lod;
                    rg.rows.push_back(r);
                }
                plan.push_back(std::move(rg));
            } else {
                std::size_t cursor = 0;
                bool first = true;
                while (cursor < lods.size() || first) {
                    PlannedRg rg;
                    if (first) {
                        RowRefV2 p;
                        p.type = PaperRecordType::Partition;
                        p.partition = &partition;
                        rg.rows.push_back(p);
                        first = false;
                    }
                    const std::size_t blockEnd =
                        std::min(lods.size(), cursor + static_cast<std::size_t>(
                                                            options.lodsPerRowGroup));
                    for (std::size_t k = cursor; k < blockEnd; ++k) {
                        RowRefV2 r;
                        r.type = PaperRecordType::Lod;
                        r.lod = lods[k];
                        rg.rows.push_back(r);
                    }
                    cursor = blockEnd;
                    plan.push_back(std::move(rg));
                    if (lods.empty()) break;
                }
            }
        }
    }

    // M5-1 zero-overhead container profile: whole-frame native payloads fit in
    // ONE binary column / ONE row, avoiding the columnar schema's dictionary +
    // column-chunk + footer overhead.
    if (options.container == PaperV2Container::Blob) {
        std::string blobError;
        PaperBlobContainerOptions blobOptions;
        blobOptions.payloadCodec = options.blobPayloadCodec;
        blobOptions.baseCodec = options.blobBaseCodec;
        const std::vector<std::uint8_t> blob =
            serializePaperStreamV2BlobContainer(normalized, &blobError, blobOptions);
        if (blob.empty()) {
            throw std::runtime_error("PaperStreamWriterV2: blob container failed: " +
                                     blobError);
        }
        auto blobSchema = arrow::schema({arrow::field("paper_blob", arrow::binary(), false)});
        arrow::BinaryBuilder blobBuilder;
        if (!blobBuilder.Append(reinterpret_cast<const char*>(blob.data()),
                                static_cast<std::int32_t>(blob.size())).ok()) {
            throw std::runtime_error("PaperStreamWriterV2: blob builder append failed");
        }
        std::shared_ptr<arrow::Array> blobArray;
        if (!blobBuilder.Finish(&blobArray).ok()) {
            throw std::runtime_error("PaperStreamWriterV2: blob builder finish failed");
        }
        const auto blobTable = arrow::Table::Make(blobSchema, {blobArray}, 1);
        auto blobPropsBuilder = parquet::WriterProperties::Builder();
        // The blob already contains entropy-coded / zlib-enveloped payloads;
        // recompression is a no-op and only adds framing. Keep it raw.
        blobPropsBuilder.compression(parquet::Compression::UNCOMPRESSED);
        blobPropsBuilder.disable_statistics();
        blobPropsBuilder.disable_dictionary("paper_blob");
        blobPropsBuilder.encoding("paper_blob", parquet::Encoding::PLAIN);
        auto blobProps = blobPropsBuilder.build();
        // Do not store Arrow schema metadata: the container is self-describing
        // and the footer shrinks by ~150-250 B (M5-1/M5-2 zero-overhead rows).
        auto blobArrowProps = parquet::ArrowWriterProperties::Builder().build();
        auto blobSinkResult = arrow::io::FileOutputStream::Open(path);
        if (!blobSinkResult.ok()) {
            throw std::runtime_error("PaperStreamWriterV2: cannot open output: " + path);
        }
        auto blobSink = blobSinkResult.ValueOrDie();
        auto blobWriterResult = parquet::arrow::FileWriter::Open(
            *blobSchema, arrow::default_memory_pool(), blobSink, blobProps,
            blobArrowProps);
        if (!blobWriterResult.ok()) {
            throw std::runtime_error("PaperStreamWriterV2: cannot create blob writer: " +
                                     blobWriterResult.status().ToString());
        }
        auto blobWriter = std::move(blobWriterResult).ValueOrDie();
        auto blobWriteStatus = blobWriter->WriteTable(*blobTable, 1);
        if (!blobWriteStatus.ok()) {
            throw std::runtime_error("PaperStreamWriterV2: blob WriteTable failed: " +
                                     blobWriteStatus.ToString());
        }
        auto blobClose = blobWriter->Close();
        if (!blobClose.ok()) {
            throw std::runtime_error("PaperStreamWriterV2: blob writer close failed: " +
                                     blobClose.ToString());
        }
        auto blobSinkClose = blobSink->Close();
        if (!blobSinkClose.ok()) {
            throw std::runtime_error("PaperStreamWriterV2: blob sink close failed: " +
                                     blobSinkClose.ToString());
        }
        return;
    }

    std::string dictError;
    const std::string dictionary =
        serializePaperStreamDictionaryV2(normalized.object, &dictError);
    if (dictionary.empty()) {
        throw std::runtime_error("PaperStreamWriterV2: dictionary serialization failed: " +
                                 dictError);
    }

    const auto schema = buildSchemaV2(typed);
    auto sinkResult = arrow::io::FileOutputStream::Open(path);
    if (!sinkResult.ok()) {
        throw std::runtime_error("PaperStreamWriterV2: cannot open output: " + path + ": " +
                                 sinkResult.status().ToString());
    }
    auto sink = sinkResult.ValueOrDie();
    auto arrowProps = parquet::ArrowWriterProperties::Builder().store_schema()->build();
    auto props = buildWriterProperties(options, typed);
    auto writerResult = parquet::arrow::FileWriter::Open(
        *schema, arrow::default_memory_pool(), sink, props, arrowProps);
    if (!writerResult.ok()) {
        throw std::runtime_error("PaperStreamWriterV2: cannot create writer: " +
                                 writerResult.status().ToString());
    }
    auto writer = std::move(writerResult).ValueOrDie();

    // RG0: object row.
    {
        std::vector<RowRefV2> rows(1);
        rows[0].type = PaperRecordType::Object;
        rows[0].object = &normalized.object;
        writeTable(*writer, buildTableV2(schema, rows, dictionary, frame, typed));
    }
    for (const auto& rg : plan) {
        writeTable(*writer, buildTableV2(schema, rg.rows, dictionary, frame, typed));
    }
    if (!normalized.boundaries.empty()) {
        std::vector<RowRefV2> rows;
        rows.reserve(normalized.boundaries.size());
        for (const auto& boundary : normalized.boundaries) {
            RowRefV2 r;
            r.type = PaperRecordType::Boundary;
            r.boundary = &boundary;
            rows.push_back(r);
        }
        writeTable(*writer, buildTableV2(schema, rows, dictionary, frame, typed));
    }

    auto closeStatus = writer->Close();
    if (!closeStatus.ok()) {
        throw std::runtime_error("PaperStreamWriterV2: writer close failed: " +
                                 closeStatus.ToString());
    }
    auto sinkClose = sink->Close();
    if (!sinkClose.ok()) {
        throw std::runtime_error("PaperStreamWriterV2: sink close failed: " +
                                 sinkClose.ToString());
    }
}

}  // namespace pmsf
