/*
 * @file PaperStreamReader.cpp
 * @brief PaperStream v1 Parquet reader (M1).
 */
#include "storage/PaperStreamReader.h"
#include "storage/PaperStreamCompactCodec.h"
#include "storage/PaperStreamDictionary.h"

#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/reader.h>
#include <parquet/file_reader.h>
#include <parquet/metadata.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pmsf {
namespace {

    constexpr const char* kRecordType = "record_type";
    constexpr const char* kObjectId = "object_id";
    constexpr const char* kPartitionId = "partition_id";
    constexpr const char* kLodId = "lod_id";
    constexpr const char* kNumRounds = "num_rounds";
    constexpr const char* kTotalVertices = "total_vertices";
    constexpr const char* kTotalFaces = "total_faces";
    constexpr const char* kNumPartitions = "num_partitions";
    constexpr const char* kEnableQuant = "enable_quantization";
    constexpr const char* kQuantBits = "quant_bits";
    constexpr const char* kBboxMinX = "bbox_min_x";
    constexpr const char* kBboxMinY = "bbox_min_y";
    constexpr const char* kBboxMinZ = "bbox_min_z";
    constexpr const char* kQuantStep = "quant_step";
    constexpr const char* kSeedV0 = "seed_v0";
    constexpr const char* kSeedV1 = "seed_v1";
    constexpr const char* kNumVertices = "num_vertices";
    constexpr const char* kNumFaces = "num_faces";
    constexpr const char* kNumLods = "num_lods";
    constexpr const char* kBaseMesh = "base_mesh";
    constexpr const char* kBaseMeshCodec = "base_mesh_codec";
    constexpr const char* kStreamDictionary = "stream_dictionary";
    constexpr const char* kCountF = "count_F";
    constexpr const char* kCountEt = "count_Et";
    constexpr const char* kCountEf = "count_Ef";
    constexpr const char* kCountV = "count_V";
    constexpr const char* kCountCf = "count_Cf";
    constexpr const char* kCountCv = "count_Cv";
    constexpr const char* kF = "F";
    constexpr const char* kEt = "Et";
    constexpr const char* kEf = "Ef";
    constexpr const char* kV = "V";
    constexpr const char* kCfQx = "Cf_qx";
    constexpr const char* kCfQy = "Cf_qy";
    constexpr const char* kCfQz = "Cf_qz";
    constexpr const char* kCvQx = "Cv_qx";
    constexpr const char* kCvQy = "Cv_qy";
    constexpr const char* kCvQz = "Cv_qz";
    constexpr const char* kCfFx = "Cf_fx";
    constexpr const char* kCfFy = "Cf_fy";
    constexpr const char* kCfFz = "Cf_fz";
    constexpr const char* kCvFx = "Cv_fx";
    constexpr const char* kCvFy = "Cv_fy";
    constexpr const char* kCvFz = "Cv_fz";
    constexpr const char* kSharedVertices = "shared_vertices";
    constexpr const char* kSharedEdges = "shared_edges";
    constexpr const char* kStopVertices = "stop_vertices";

    bool isCoreStreamName(const std::string& name) {
        return name == kF || name == kEt || name == kEf || name == kV || name == "Cf" || name == "Cv";
    }

    std::string extensionColumnName(const std::string& name) {
        return std::string("ext_") + name;
    }

    std::shared_ptr<arrow::Array> getColumn(const std::shared_ptr<arrow::Table>& table, const std::string& name) {
        if (!table)
            return nullptr;
        const int index = table->schema()->GetFieldIndex(name);
        if (index < 0)
            return nullptr;
        auto column = table->column(index);
        if (column->num_chunks() == 0)
            return nullptr;
        return column->chunk(0);
    }

    std::int32_t getInt32(const std::shared_ptr<arrow::Array>& array, int64_t index) {
        if (!array || array->IsNull(index))
            return 0;
        return static_cast<arrow::Int32Array*>(array.get())->Value(index);
    }

    std::int8_t getInt8(const std::shared_ptr<arrow::Array>& array, int64_t index) {
        if (!array || array->IsNull(index))
            return 0;
        return static_cast<arrow::Int8Array*>(array.get())->Value(index);
    }

    bool getBool(const std::shared_ptr<arrow::Array>& array, int64_t index) {
        if (!array || array->IsNull(index))
            return false;
        return static_cast<arrow::BooleanArray*>(array.get())->Value(index);
    }

    float getFloat(const std::shared_ptr<arrow::Array>& array, int64_t index) {
        if (!array || array->IsNull(index))
            return 0.f;
        return static_cast<arrow::FloatArray*>(array.get())->Value(index);
    }

    std::vector<std::uint8_t> getBinary(const std::shared_ptr<arrow::Array>& array, int64_t index) {
        if (!array || array->IsNull(index))
            return {};
        auto* binary = static_cast<arrow::BinaryArray*>(array.get());
        const auto view = binary->GetView(index);
        return std::vector<std::uint8_t>(reinterpret_cast<const std::uint8_t*>(view.data()),
                                         reinterpret_cast<const std::uint8_t*>(view.data()) + view.size());
    }

    std::string getText(const std::shared_ptr<arrow::Array>& array, int64_t index) {
        if (!array || array->IsNull(index))
            return {};
        auto* binary = static_cast<arrow::BinaryArray*>(array.get());
        const auto view = binary->GetView(index);
        return std::string(view.data(), view.size());
    }

    std::vector<std::int32_t> getInt32List(const std::shared_ptr<arrow::Array>& array, int64_t index) {
        if (!array || array->IsNull(index))
            return {};
        auto* list = static_cast<arrow::ListArray*>(array.get());
        auto values = std::static_pointer_cast<arrow::Int32Array>(list->values());
        const int64_t start = list->value_offset(index);
        const int64_t length = list->value_length(index);
        std::vector<std::int32_t> out(static_cast<std::size_t>(length));
        for (int64_t i = 0; i < length; ++i) {
            out[static_cast<std::size_t>(i)] = values->Value(start + i);
        }
        return out;
    }

    std::vector<float> getFloatList(const std::shared_ptr<arrow::Array>& array, int64_t index) {
        if (!array || array->IsNull(index))
            return {};
        auto* list = static_cast<arrow::ListArray*>(array.get());
        auto values = std::static_pointer_cast<arrow::FloatArray>(list->values());
        const int64_t start = list->value_offset(index);
        const int64_t length = list->value_length(index);
        std::vector<float> out(static_cast<std::size_t>(length));
        for (int64_t i = 0; i < length; ++i) {
            out[static_cast<std::size_t>(i)] = values->Value(start + i);
        }
        return out;
    }

    std::vector<std::int32_t>
    interleave3(const std::vector<std::int32_t>& x, const std::vector<std::int32_t>& y, const std::vector<std::int32_t>& z) {
        const std::size_t n = std::max({x.size(), y.size(), z.size()});
        if (x.size() != y.size() || y.size() != z.size()) {
            throw std::runtime_error("PaperStream: CfQ/CvQ axis lengths differ");
        }
        std::vector<std::int32_t> out;
        out.reserve(n * 3);
        for (std::size_t i = 0; i < n; ++i) {
            out.push_back(x[i]);
            out.push_back(y[i]);
            out.push_back(z[i]);
        }
        return out;
    }

    std::vector<float> interleave3(const std::vector<float>& x, const std::vector<float>& y, const std::vector<float>& z) {
        const std::size_t n = std::max({x.size(), y.size(), z.size()});
        if (x.size() != y.size() || y.size() != z.size()) {
            throw std::runtime_error("PaperStream: CfF/CvF axis lengths differ");
        }
        std::vector<float> out;
        out.reserve(n * 3);
        for (std::size_t i = 0; i < n; ++i) {
            out.push_back(x[i]);
            out.push_back(y[i]);
            out.push_back(z[i]);
        }
        return out;
    }

    struct Columns {
        std::shared_ptr<arrow::Array> recordType;
        std::shared_ptr<arrow::Array> objectId;
        std::shared_ptr<arrow::Array> partitionId;
        std::shared_ptr<arrow::Array> lodId;
        std::shared_ptr<arrow::Array> numRounds;
        std::shared_ptr<arrow::Array> totalVertices;
        std::shared_ptr<arrow::Array> totalFaces;
        std::shared_ptr<arrow::Array> numPartitions;
        std::shared_ptr<arrow::Array> enableQuant;
        std::shared_ptr<arrow::Array> quantBits;
        std::shared_ptr<arrow::Array> bboxMinX;
        std::shared_ptr<arrow::Array> bboxMinY;
        std::shared_ptr<arrow::Array> bboxMinZ;
        std::shared_ptr<arrow::Array> quantStep;
        std::shared_ptr<arrow::Array> seedV0;
        std::shared_ptr<arrow::Array> seedV1;
        std::shared_ptr<arrow::Array> numVertices;
        std::shared_ptr<arrow::Array> numFaces;
        std::shared_ptr<arrow::Array> numLods;
        std::shared_ptr<arrow::Array> baseMesh;
        std::shared_ptr<arrow::Array> baseMeshCodec;
        std::shared_ptr<arrow::Array> dictionary;
        std::shared_ptr<arrow::Array> countF;
        std::shared_ptr<arrow::Array> countEt;
        std::shared_ptr<arrow::Array> countEf;
        std::shared_ptr<arrow::Array> countV;
        std::shared_ptr<arrow::Array> countCf;
        std::shared_ptr<arrow::Array> countCv;
        std::shared_ptr<arrow::Array> F;
        std::shared_ptr<arrow::Array> Et;
        std::shared_ptr<arrow::Array> Ef;
        std::shared_ptr<arrow::Array> V;
        std::shared_ptr<arrow::Array> CfQx;
        std::shared_ptr<arrow::Array> CfQy;
        std::shared_ptr<arrow::Array> CfQz;
        std::shared_ptr<arrow::Array> CvQx;
        std::shared_ptr<arrow::Array> CvQy;
        std::shared_ptr<arrow::Array> CvQz;
        std::shared_ptr<arrow::Array> CfFx;
        std::shared_ptr<arrow::Array> CfFy;
        std::shared_ptr<arrow::Array> CfFz;
        std::shared_ptr<arrow::Array> CvFx;
        std::shared_ptr<arrow::Array> CvFy;
        std::shared_ptr<arrow::Array> CvFz;
        std::shared_ptr<arrow::Array> boundaryA;
        std::shared_ptr<arrow::Array> boundaryB;
        std::shared_ptr<arrow::Array> sharedVertices;
        std::shared_ptr<arrow::Array> sharedEdges;
        std::shared_ptr<arrow::Array> stopVertices;
    };

    Columns readColumns(const std::shared_ptr<arrow::Table>& table) {
        Columns c;
        c.recordType = getColumn(table, kRecordType);
        c.objectId = getColumn(table, kObjectId);
        c.partitionId = getColumn(table, kPartitionId);
        c.lodId = getColumn(table, kLodId);
        c.numRounds = getColumn(table, kNumRounds);
        c.totalVertices = getColumn(table, kTotalVertices);
        c.totalFaces = getColumn(table, kTotalFaces);
        c.numPartitions = getColumn(table, kNumPartitions);
        c.enableQuant = getColumn(table, kEnableQuant);
        c.quantBits = getColumn(table, kQuantBits);
        c.bboxMinX = getColumn(table, kBboxMinX);
        c.bboxMinY = getColumn(table, kBboxMinY);
        c.bboxMinZ = getColumn(table, kBboxMinZ);
        c.quantStep = getColumn(table, kQuantStep);
        c.seedV0 = getColumn(table, kSeedV0);
        c.seedV1 = getColumn(table, kSeedV1);
        c.numVertices = getColumn(table, kNumVertices);
        c.numFaces = getColumn(table, kNumFaces);
        c.numLods = getColumn(table, kNumLods);
        c.baseMesh = getColumn(table, kBaseMesh);
        c.baseMeshCodec = getColumn(table, kBaseMeshCodec);
        c.dictionary = getColumn(table, kStreamDictionary);
        c.countF = getColumn(table, kCountF);
        c.countEt = getColumn(table, kCountEt);
        c.countEf = getColumn(table, kCountEf);
        c.countV = getColumn(table, kCountV);
        c.countCf = getColumn(table, kCountCf);
        c.countCv = getColumn(table, kCountCv);
        c.F = getColumn(table, kF);
        c.Et = getColumn(table, kEt);
        c.Ef = getColumn(table, kEf);
        c.V = getColumn(table, kV);
        c.CfQx = getColumn(table, kCfQx);
        c.CfQy = getColumn(table, kCfQy);
        c.CfQz = getColumn(table, kCfQz);
        c.CvQx = getColumn(table, kCvQx);
        c.CvQy = getColumn(table, kCvQy);
        c.CvQz = getColumn(table, kCvQz);
        c.CfFx = getColumn(table, kCfFx);
        c.CfFy = getColumn(table, kCfFy);
        c.CfFz = getColumn(table, kCfFz);
        c.CvFx = getColumn(table, kCvFx);
        c.CvFy = getColumn(table, kCvFy);
        c.CvFz = getColumn(table, kCvFz);
        c.boundaryA = getColumn(table, "boundary_a");
        c.boundaryB = getColumn(table, "boundary_b");
        c.sharedVertices = getColumn(table, kSharedVertices);
        c.sharedEdges = getColumn(table, kSharedEdges);
        c.stopVertices = getColumn(table, kStopVertices);
        return c;
    }

}  // namespace

struct PaperStreamReader::Impl {
    std::string path;
    std::shared_ptr<arrow::io::RandomAccessFile> file;
    std::unique_ptr<parquet::arrow::FileReader> reader;
    std::unique_ptr<parquet::ParquetFileReader> parquet;
    std::shared_ptr<arrow::Schema> schema;
    bool compact = false;
};

PaperStreamReader::PaperStreamReader(const std::string& path) : impl_(std::make_unique<Impl>()) {
    impl_->path = path;
    auto fileResult = arrow::io::ReadableFile::Open(path);
    if (!fileResult.ok()) {
        throw std::runtime_error("PaperStream: cannot open input: " + path + ": " + fileResult.status().ToString());
    }
    impl_->file = fileResult.ValueOrDie();
    auto openResult = parquet::arrow::OpenFile(impl_->file, arrow::default_memory_pool());
    if (!openResult.ok()) {
        throw std::runtime_error("PaperStream: cannot open Parquet reader: " + openResult.status().ToString());
    }
    impl_->reader = std::move(openResult).ValueOrDie();
    auto schemaStatus = impl_->reader->GetSchema(&impl_->schema);
    if (!schemaStatus.ok()) {
        throw std::runtime_error("PaperStream: cannot read schema: " + schemaStatus.ToString());
    }
    impl_->compact = impl_->schema->GetFieldIndex("connectivity") >= 0;
    impl_->parquet = parquet::ParquetFileReader::OpenFile(path, false);
    if (!impl_->parquet) {
        throw std::runtime_error("PaperStream: cannot open Parquet metadata: " + path);
    }
}

PaperStreamReader::~PaperStreamReader() = default;

int PaperStreamReader::numRowGroups() const {
    return impl_->reader ? impl_->reader->num_row_groups() : 0;
}

namespace {

    void consumeTable(const std::shared_ptr<arrow::Table>& table, PaperStreamTable& out, int maxLod, int onlyPartition, bool hasOnlyPartition) {
        const auto c = readColumns(table);
        const int64_t rows = table->num_rows();
        for (int64_t i = 0; i < rows; ++i) {
            const auto type = static_cast<PaperRecordType>(getInt8(c.recordType, i));
            if (type == PaperRecordType::Object) {
                out.object.objectId = getInt32(c.objectId, i);
                out.object.numRounds = getInt32(c.numRounds, i);
                out.object.totalVertices = getInt32(c.totalVertices, i);
                out.object.totalFaces = getInt32(c.totalFaces, i);
                out.object.numPartitions = getInt32(c.numPartitions, i);
                out.object.enableQuantization = getBool(c.enableQuant, i);
                out.object.quantBits = getInt32(c.quantBits, i);
                out.object.bboxMin[0] = getFloat(c.bboxMinX, i);
                out.object.bboxMin[1] = getFloat(c.bboxMinY, i);
                out.object.bboxMin[2] = getFloat(c.bboxMinZ, i);
                out.object.quantStep = getFloat(c.quantStep, i);
                const std::string dictionary = getText(c.dictionary, i);
                std::string error;
                if (!parsePaperStreamDictionary(dictionary, out.object, &error)) {
                    throw std::runtime_error("PaperStream: bad dictionary: " + error);
                }
                continue;
            }
            if (type == PaperRecordType::Partition) {
                const int32_t partitionId = getInt32(c.partitionId, i);
                if (hasOnlyPartition && partitionId != onlyPartition)
                    continue;
                PaperPartitionRow p;
                p.objectId = getInt32(c.objectId, i);
                p.partitionId = partitionId;
                p.seedV0 = getInt32(c.seedV0, i);
                p.seedV1 = getInt32(c.seedV1, i);
                p.numVertices = getInt32(c.numVertices, i);
                p.numFaces = getInt32(c.numFaces, i);
                p.numLods = getInt32(c.numLods, i);
                p.enableQuantization = getBool(c.enableQuant, i);
                p.quantBits = getInt32(c.quantBits, i);
                p.bboxMin[0] = getFloat(c.bboxMinX, i);
                p.bboxMin[1] = getFloat(c.bboxMinY, i);
                p.bboxMin[2] = getFloat(c.bboxMinZ, i);
                p.quantStep = getFloat(c.quantStep, i);
                p.baseMesh = getBinary(c.baseMesh, i);
                p.baseMeshCodec = getText(c.baseMeshCodec, i);
                out.partitions.push_back(std::move(p));
                continue;
            }
            if (type == PaperRecordType::Lod) {
                const int32_t partitionId = getInt32(c.partitionId, i);
                if (hasOnlyPartition && partitionId != onlyPartition)
                    continue;
                const int32_t lodId = getInt32(c.lodId, i);
                if (maxLod >= 0 && lodId >= maxLod)
                    continue;
                PaperLodRow lod;
                lod.objectId = getInt32(c.objectId, i);
                lod.partitionId = partitionId;
                lod.lodId = lodId;
                lod.countF = getInt32(c.countF, i);
                lod.countEt = getInt32(c.countEt, i);
                lod.countEf = getInt32(c.countEf, i);
                lod.countV = getInt32(c.countV, i);
                lod.countCf = getInt32(c.countCf, i);
                lod.countCv = getInt32(c.countCv, i);
                lod.F = getBinary(c.F, i);
                lod.Et = getBinary(c.Et, i);
                lod.Ef = getBinary(c.Ef, i);
                lod.V = getBinary(c.V, i);
                lod.CfQ = interleave3(getInt32List(c.CfQx, i), getInt32List(c.CfQy, i), getInt32List(c.CfQz, i));
                lod.CvQ = interleave3(getInt32List(c.CvQx, i), getInt32List(c.CvQy, i), getInt32List(c.CvQz, i));
                lod.CfF = interleave3(getFloatList(c.CfFx, i), getFloatList(c.CfFy, i), getFloatList(c.CfFz, i));
                lod.CvF = interleave3(getFloatList(c.CvFx, i), getFloatList(c.CvFy, i), getFloatList(c.CvFz, i));
                for (const auto& spec : out.object.streams) {
                    if (isCoreStreamName(spec.name))
                        continue;
                    auto column = getColumn(table, extensionColumnName(spec.name));
                    if (column && !column->IsNull(i)) {
                        lod.extension[spec.name] = getBinary(column, i);
                    }
                }
                out.lods.push_back(std::move(lod));
                continue;
            }
            if (type == PaperRecordType::Boundary) {
                PaperBoundaryRow b;
                b.objectId = getInt32(c.objectId, i);
                b.partitionA = getInt32(c.boundaryA, i);
                b.partitionB = getInt32(c.boundaryB, i);
                b.sharedVertices = getInt32List(c.sharedVertices, i);
                b.sharedEdges = getInt32List(c.sharedEdges, i);
                b.stopVertices = getInt32List(c.stopVertices, i);
                out.boundaries.push_back(std::move(b));
                continue;
            }
            throw std::runtime_error("PaperStream: unknown record_type " + std::to_string(static_cast<int>(type)));
        }
    }

    void
    consumeCompactTable(const std::shared_ptr<arrow::Table>& table, PaperStreamTable& out, int maxLod, int onlyPartition, bool hasOnlyPartition) {
        const auto typeCol = getColumn(table, kRecordType);
        const auto objectIdCol = getColumn(table, kObjectId);
        const auto partitionIdCol = getColumn(table, kPartitionId);
        const auto lodIdCol = getColumn(table, kLodId);
        const auto headerCol = getColumn(table, "header");
        const auto baseMeshCol = getColumn(table, "base_mesh");
        const auto legacyBundleCol = getColumn(table, "legacy_bundle");
        const auto connectivityCol = getColumn(table, "connectivity");
        const auto geometryCol = getColumn(table, "geometry");
        const auto extensionsCol = getColumn(table, "extensions");
        const auto boundaryCol = getColumn(table, "boundary");
        const int64_t rows = table->num_rows();
        for (int64_t i = 0; i < rows; ++i) {
            const auto type = static_cast<PaperRecordType>(getInt8(typeCol, i));
            if (type == PaperRecordType::Object) {
                const std::string dictionary = getText(headerCol, i);
                std::string error;
                if (!parsePaperStreamDictionary(dictionary, out.object, &error)) {
                    throw std::runtime_error("PaperStream: bad compact dictionary: " + error);
                }
                out.object.schemaMode = PaperSchemaMode::Compact;
                if (legacyBundleCol && !legacyBundleCol->IsNull(i)) {
                    std::string bundleError;
                    if (!unpackPaperLegacyBundle(getBinary(legacyBundleCol, i),
                                                 out.legacyFiles, &bundleError)) {
                        throw std::runtime_error("PaperStream: bad legacy bundle: " + bundleError);
                    }
                }
                continue;
            }
            if (type == PaperRecordType::Partition) {
                const int32_t partitionId = getInt32(partitionIdCol, i);
                if (hasOnlyPartition && partitionId != onlyPartition)
                    continue;
                PaperPartitionRow p;
                p.objectId = getInt32(objectIdCol, i);
                p.partitionId = partitionId;
                const auto header = getBinary(headerCol, i);
                std::string error;
                if (!unpackPaperPartitionHeader(header, p, &error)) {
                    throw std::runtime_error("PaperStream: bad compact partition header: " + error);
                }
                p.baseMesh = getBinary(baseMeshCol, i);
                out.partitions.push_back(std::move(p));
                continue;
            }
            if (type == PaperRecordType::Lod) {
                const int32_t partitionId = getInt32(partitionIdCol, i);
                if (hasOnlyPartition && partitionId != onlyPartition)
                    continue;
                const int32_t lodId = getInt32(lodIdCol, i);
                if (maxLod >= 0 && lodId >= maxLod)
                    continue;
                PaperLodRow lod;
                lod.objectId = getInt32(objectIdCol, i);
                lod.partitionId = partitionId;
                lod.lodId = lodId;
                std::string error;
                if (!unpackPaperConnectivity(getBinary(connectivityCol, i), lod, &error)) {
                    throw std::runtime_error("PaperStream: bad compact connectivity: " + error);
                }
                if (!unpackPaperGeometry(getBinary(geometryCol, i), lod, &error)) {
                    throw std::runtime_error("PaperStream: bad compact geometry: " + error);
                }
                if (extensionsCol && !extensionsCol->IsNull(i)) {
                    if (!unpackPaperExtensions(getBinary(extensionsCol, i), lod, &error)) {
                        throw std::runtime_error("PaperStream: bad compact extensions: " + error);
                    }
                }
                out.lods.push_back(std::move(lod));
                continue;
            }
            if (type == PaperRecordType::Boundary) {
                PaperBoundaryRow b;
                b.objectId = getInt32(objectIdCol, i);
                std::string error;
                if (!unpackPaperBoundary(getBinary(boundaryCol, i), b, &error)) {
                    throw std::runtime_error("PaperStream: bad compact boundary: " + error);
                }
                out.boundaries.push_back(std::move(b));
                continue;
            }
            throw std::runtime_error("PaperStream: unknown record_type " + std::to_string(static_cast<int>(type)));
        }
    }

}  // namespace

PaperStreamTable PaperStreamReader::readAll() {
    PaperStreamTable out;
    bytesRead_ = 0;
    rowGroupsRead_ = 0;
    const int groups = impl_->reader->num_row_groups();
    for (int i = 0; i < groups; ++i) {
        std::shared_ptr<arrow::Table> table;
        auto st = impl_->reader->ReadRowGroup(i, &table);
        if (!st.ok())
            throw std::runtime_error("PaperStream: ReadRowGroup failed: " + st.ToString());
        if (impl_->compact) {
            consumeCompactTable(table, out, -1, 0, false);
        } else {
            consumeTable(table, out, -1, 0, false);
        }
        if (impl_->parquet && impl_->parquet->metadata()) {
            bytesRead_ += static_cast<std::uint64_t>(impl_->parquet->metadata()->RowGroup(i)->total_compressed_size());
        }
        ++rowGroupsRead_;
    }
    try {
        bytesRead_ = std::filesystem::file_size(impl_->path);
    } catch (...) {
        // Keep the row-group sum if the file disappeared mid-read.
    }
    return out;
}

PaperStreamTable PaperStreamReader::readPrefix(int partitionId, int maxLod) {
    PaperStreamTable out;
    bytesRead_ = 0;
    rowGroupsRead_ = 0;
    const int groups = impl_->reader->num_row_groups();
    if (groups <= 0)
        return out;

    std::shared_ptr<arrow::Table> objectTable;
    auto objectStatus = impl_->reader->ReadRowGroup(0, &objectTable);
    if (!objectStatus.ok())
        throw std::runtime_error("PaperStream: object RG read failed: " + objectStatus.ToString());
    if (impl_->compact) {
        consumeCompactTable(objectTable, out, -1, 0, false);
    } else {
        consumeTable(objectTable, out, -1, 0, false);
    }
    if (impl_->parquet && impl_->parquet->metadata()) {
        bytesRead_ += static_cast<std::uint64_t>(impl_->parquet->metadata()->RowGroup(0)->total_compressed_size());
    }
    ++rowGroupsRead_;

    // M1 layout contract: partition groups start at row group 1.  A group
    // holds `partitionsPerRowGroup` partitions in vector order.
    const int stride = std::max(1, out.object.partitionsPerRowGroup);
    const int target = 1 + partitionId / stride;
    if (target >= 0 && target < groups) {
        std::shared_ptr<arrow::Table> table;
        auto st = impl_->reader->ReadRowGroup(target, &table);
        if (!st.ok())
            throw std::runtime_error("PaperStream: partition RG read failed: " + st.ToString());
        if (impl_->compact) {
            consumeCompactTable(table, out, maxLod, partitionId, true);
        } else {
            consumeTable(table, out, maxLod, partitionId, true);
        }
        if (impl_->parquet && impl_->parquet->metadata()) {
            bytesRead_ += static_cast<std::uint64_t>(impl_->parquet->metadata()->RowGroup(target)->total_compressed_size());
        }
        ++rowGroupsRead_;
    }
    return out;
}

}  // namespace pmsf
