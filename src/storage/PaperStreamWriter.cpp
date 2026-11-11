/*
 * @file PaperStreamWriter.cpp
 * @brief PaperStream v1 Parquet writer (M1).
 */
#include "storage/PaperStreamWriter.h"
#include "storage/PaperStreamCompactCodec.h"
#include "storage/PaperStreamDictionary.h"

#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/writer.h>
#include <parquet/properties.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <map>
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

    std::vector<PaperStreamSpec> extensionSpecsOf(const PaperObjectRow& object) {
        std::vector<PaperStreamSpec> out;
        for (const auto& spec : object.streams) {
            if (!isCoreStreamName(spec.name))
                out.push_back(spec);
        }
        return out;
    }

    struct RowRef {
        PaperRecordType type = PaperRecordType::Object;
        const PaperObjectRow* object = nullptr;
        const PaperPartitionRow* partition = nullptr;
        const PaperLodRow* lod = nullptr;
        const PaperBoundaryRow* boundary = nullptr;
    };

    std::shared_ptr<arrow::Schema> buildSchema(const PaperObjectRow& object) {
        std::vector<std::shared_ptr<arrow::Field>> fields;
        fields.push_back(arrow::field(kRecordType, arrow::int8(), false));
        fields.push_back(arrow::field(kObjectId, arrow::int32(), false));
        fields.push_back(arrow::field(kPartitionId, arrow::int32(), false));
        fields.push_back(arrow::field(kLodId, arrow::int32(), false));

        fields.push_back(arrow::field(kNumRounds, arrow::int32(), true));
        fields.push_back(arrow::field(kTotalVertices, arrow::int32(), true));
        fields.push_back(arrow::field(kTotalFaces, arrow::int32(), true));
        fields.push_back(arrow::field(kNumPartitions, arrow::int32(), true));
        fields.push_back(arrow::field(kEnableQuant, arrow::boolean(), true));
        fields.push_back(arrow::field(kQuantBits, arrow::int32(), true));
        fields.push_back(arrow::field(kBboxMinX, arrow::float32(), true));
        fields.push_back(arrow::field(kBboxMinY, arrow::float32(), true));
        fields.push_back(arrow::field(kBboxMinZ, arrow::float32(), true));
        fields.push_back(arrow::field(kQuantStep, arrow::float32(), true));

        fields.push_back(arrow::field(kSeedV0, arrow::int32(), true));
        fields.push_back(arrow::field(kSeedV1, arrow::int32(), true));
        fields.push_back(arrow::field(kNumVertices, arrow::int32(), true));
        fields.push_back(arrow::field(kNumFaces, arrow::int32(), true));
        fields.push_back(arrow::field(kNumLods, arrow::int32(), true));
        fields.push_back(arrow::field(kBaseMesh, arrow::binary(), true));
        fields.push_back(arrow::field(kBaseMeshCodec, arrow::binary(), true));
        fields.push_back(arrow::field(kStreamDictionary, arrow::binary(), true));

        fields.push_back(arrow::field(kCountF, arrow::int32(), true));
        fields.push_back(arrow::field(kCountEt, arrow::int32(), true));
        fields.push_back(arrow::field(kCountEf, arrow::int32(), true));
        fields.push_back(arrow::field(kCountV, arrow::int32(), true));
        fields.push_back(arrow::field(kCountCf, arrow::int32(), true));
        fields.push_back(arrow::field(kCountCv, arrow::int32(), true));

        fields.push_back(arrow::field(kF, arrow::binary(), true));
        fields.push_back(arrow::field(kEt, arrow::binary(), true));
        fields.push_back(arrow::field(kEf, arrow::binary(), true));
        fields.push_back(arrow::field(kV, arrow::binary(), true));
        fields.push_back(arrow::field(kCfQx, arrow::list(arrow::int32()), true));
        fields.push_back(arrow::field(kCfQy, arrow::list(arrow::int32()), true));
        fields.push_back(arrow::field(kCfQz, arrow::list(arrow::int32()), true));
        fields.push_back(arrow::field(kCvQx, arrow::list(arrow::int32()), true));
        fields.push_back(arrow::field(kCvQy, arrow::list(arrow::int32()), true));
        fields.push_back(arrow::field(kCvQz, arrow::list(arrow::int32()), true));
        fields.push_back(arrow::field(kCfFx, arrow::list(arrow::float32()), true));
        fields.push_back(arrow::field(kCfFy, arrow::list(arrow::float32()), true));
        fields.push_back(arrow::field(kCfFz, arrow::list(arrow::float32()), true));
        fields.push_back(arrow::field(kCvFx, arrow::list(arrow::float32()), true));
        fields.push_back(arrow::field(kCvFy, arrow::list(arrow::float32()), true));
        fields.push_back(arrow::field(kCvFz, arrow::list(arrow::float32()), true));
        fields.push_back(arrow::field("boundary_a", arrow::int32(), true));
        fields.push_back(arrow::field("boundary_b", arrow::int32(), true));
        fields.push_back(arrow::field(kSharedVertices, arrow::list(arrow::int32()), true));
        fields.push_back(arrow::field(kSharedEdges, arrow::list(arrow::int32()), true));
        fields.push_back(arrow::field(kStopVertices, arrow::list(arrow::int32()), true));

        for (const auto& spec : object.streams) {
            if (isCoreStreamName(spec.name))
                continue;
            if (spec.cellType != PaperCellType::Bytes) {
                throw std::runtime_error("PaperStream M1: extension stream '" + spec.name +
                                         "' must declare cell=bytes (typed extension streams arrive in M2)");
            }
            fields.push_back(arrow::field(extensionColumnName(spec.name), arrow::binary(), true));
        }
        return arrow::schema(fields);
    }

    template <typename Builder, typename T> void appendInt(Builder& builder, bool has, T value) {
        if (has) {
            auto st = builder.Append(static_cast<T>(value));
            if (!st.ok())
                throw std::runtime_error("PaperStream: int append failed: " + st.ToString());
        } else {
            auto st = builder.AppendNull();
            if (!st.ok())
                throw std::runtime_error("PaperStream: int null append failed: " + st.ToString());
        }
    }

    template <typename Builder> void appendFloat(Builder& builder, bool has, float value) {
        if (has) {
            auto st = builder.Append(value);
            if (!st.ok())
                throw std::runtime_error("PaperStream: float append failed: " + st.ToString());
        } else {
            auto st = builder.AppendNull();
            if (!st.ok())
                throw std::runtime_error("PaperStream: float null append failed: " + st.ToString());
        }
    }

    void appendBinary(arrow::BinaryBuilder& builder, bool has, const std::vector<std::uint8_t>& bytes) {
        if (has) {
            auto st = builder.Append(bytes.data(), static_cast<int32_t>(bytes.size()));
            if (!st.ok())
                throw std::runtime_error("PaperStream: binary append failed: " + st.ToString());
        } else {
            auto st = builder.AppendNull();
            if (!st.ok())
                throw std::runtime_error("PaperStream: binary null append failed: " + st.ToString());
        }
    }

    void appendText(arrow::BinaryBuilder& builder, bool has, const std::string& text) {
        if (has) {
            auto st = builder.Append(reinterpret_cast<const std::uint8_t*>(text.data()), static_cast<int32_t>(text.size()));
            if (!st.ok())
                throw std::runtime_error("PaperStream: text append failed: " + st.ToString());
        } else {
            auto st = builder.AppendNull();
            if (!st.ok())
                throw std::runtime_error("PaperStream: text null append failed: " + st.ToString());
        }
    }

    void appendInt32List(arrow::ListBuilder& list, arrow::Int32Builder& values, bool has, const std::vector<std::int32_t>& data) {
        if (!has) {
            auto st = list.AppendNull();
            if (!st.ok())
                throw std::runtime_error("PaperStream: list null append failed: " + st.ToString());
            return;
        }
        auto st = list.Append();
        if (!st.ok())
            throw std::runtime_error("PaperStream: list append failed: " + st.ToString());
        for (std::int32_t v : data) {
            auto s = values.Append(v);
            if (!s.ok())
                throw std::runtime_error("PaperStream: list value append failed: " + s.ToString());
        }
    }

    void appendFloatList(arrow::ListBuilder& list, arrow::FloatBuilder& values, bool has, const std::vector<float>& data) {
        if (!has) {
            auto st = list.AppendNull();
            if (!st.ok())
                throw std::runtime_error("PaperStream: list null append failed: " + st.ToString());
            return;
        }
        auto st = list.Append();
        if (!st.ok())
            throw std::runtime_error("PaperStream: list append failed: " + st.ToString());
        for (float v : data) {
            auto s = values.Append(v);
            if (!s.ok())
                throw std::runtime_error("PaperStream: list value append failed: " + s.ToString());
        }
    }

    std::vector<std::int32_t> axis(const std::vector<std::int32_t>& xyz, int which) {
        std::vector<std::int32_t> out;
        if (xyz.size() % 3 != 0) {
            throw std::runtime_error("PaperStream: CfQ/CvQ size is not a multiple of 3");
        }
        out.reserve(xyz.size() / 3);
        for (std::size_t i = static_cast<std::size_t>(which); i < xyz.size(); i += 3) {
            out.push_back(xyz[i]);
        }
        return out;
    }

    std::vector<float> axis(const std::vector<float>& xyz, int which) {
        std::vector<float> out;
        if (xyz.size() % 3 != 0) {
            throw std::runtime_error("PaperStream: CfF/CvF size is not a multiple of 3");
        }
        out.reserve(xyz.size() / 3);
        for (std::size_t i = static_cast<std::size_t>(which); i < xyz.size(); i += 3) {
            out.push_back(xyz[i]);
        }
        return out;
    }

    std::shared_ptr<arrow::Table> buildTable(const std::shared_ptr<arrow::Schema>& schema,
                                             const std::vector<RowRef>& rows,
                                             const std::string& dictionary,
                                             const std::vector<PaperStreamSpec>& extensionSpecs) {
        const std::size_t n = rows.size();
        std::vector<std::shared_ptr<arrow::Array>> arrays;
        arrays.reserve(schema->num_fields());

        {
            arrow::Int8Builder b;
            for (const auto& r : rows)
                appendInt(b, true, static_cast<std::int8_t>(r.type));
            std::shared_ptr<arrow::Array> a;
            auto st = b.Finish(&a);
            if (!st.ok())
                throw std::runtime_error("PaperStream: record_type finish failed");
            arrays.push_back(a);
        }
        {
            arrow::Int32Builder b;
            for (const auto& r : rows) {
                std::int32_t v = 0;
                if (r.object)
                    v = r.object->objectId;
                else if (r.partition)
                    v = r.partition->objectId;
                else if (r.lod)
                    v = r.lod->objectId;
                else if (r.boundary)
                    v = r.boundary->objectId;
                appendInt(b, true, v);
            }
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: object_id finish failed");
            arrays.push_back(a);
        }
        {
            arrow::Int32Builder b;
            for (const auto& r : rows) {
                std::int32_t v = -1;
                if (r.partition)
                    v = r.partition->partitionId;
                else if (r.lod)
                    v = r.lod->partitionId;
                appendInt(b, true, v);
            }
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: partition_id finish failed");
            arrays.push_back(a);
        }
        {
            arrow::Int32Builder b;
            for (const auto& r : rows)
                appendInt(b, true, r.lod ? r.lod->lodId : -1);
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: lod_id finish failed");
            arrays.push_back(a);
        }
        {
            arrow::Int32Builder b;
            for (const auto& r : rows)
                appendInt(b, r.object != nullptr, r.object ? r.object->numRounds : 0);
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: num_rounds finish failed");
            arrays.push_back(a);
        }
        {
            arrow::Int32Builder b;
            for (const auto& r : rows)
                appendInt(b, r.object != nullptr, r.object ? r.object->totalVertices : 0);
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: total_vertices finish failed");
            arrays.push_back(a);
        }
        {
            arrow::Int32Builder b;
            for (const auto& r : rows)
                appendInt(b, r.object != nullptr, r.object ? r.object->totalFaces : 0);
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: total_faces finish failed");
            arrays.push_back(a);
        }
        {
            arrow::Int32Builder b;
            for (const auto& r : rows)
                appendInt(b, r.object != nullptr, r.object ? r.object->numPartitions : 0);
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: num_partitions finish failed");
            arrays.push_back(a);
        }
        {
            arrow::BooleanBuilder b;
            for (const auto& r : rows) {
                if (r.object) {
                    b.Append(r.object->enableQuantization);
                } else if (r.partition) {
                    b.Append(r.partition->enableQuantization);
                } else {
                    b.AppendNull();
                }
            }
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: enable_quant finish failed");
            arrays.push_back(a);
        }
        {
            arrow::Int32Builder b;
            for (const auto& r : rows) {
                if (r.object)
                    appendInt(b, true, r.object->quantBits);
                else if (r.partition)
                    appendInt(b, true, r.partition->quantBits);
                else
                    appendInt(b, false, 0);
            }
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: quant_bits finish failed");
            arrays.push_back(a);
        }
        for (int axisIndex = 0; axisIndex < 3; ++axisIndex) {
            arrow::FloatBuilder b;
            for (const auto& r : rows) {
                const float* bbox = r.object ? r.object->bboxMin : (r.partition ? r.partition->bboxMin : nullptr);
                appendFloat(b, bbox != nullptr, bbox ? bbox[axisIndex] : 0.f);
            }
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: bbox finish failed");
            arrays.push_back(a);
        }
        {
            arrow::FloatBuilder b;
            for (const auto& r : rows) {
                if (r.object)
                    appendFloat(b, true, r.object->quantStep);
                else if (r.partition)
                    appendFloat(b, true, r.partition->quantStep);
                else
                    appendFloat(b, false, 0.f);
            }
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: quant_step finish failed");
            arrays.push_back(a);
        }
        {
            arrow::Int32Builder b;
            for (const auto& r : rows)
                appendInt(b, r.partition != nullptr, r.partition ? r.partition->seedV0 : -1);
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: seed_v0 finish failed");
            arrays.push_back(a);
        }
        {
            arrow::Int32Builder b;
            for (const auto& r : rows)
                appendInt(b, r.partition != nullptr, r.partition ? r.partition->seedV1 : -1);
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: seed_v1 finish failed");
            arrays.push_back(a);
        }
        {
            arrow::Int32Builder b;
            for (const auto& r : rows)
                appendInt(b, r.partition != nullptr, r.partition ? r.partition->numVertices : 0);
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: num_vertices finish failed");
            arrays.push_back(a);
        }
        {
            arrow::Int32Builder b;
            for (const auto& r : rows)
                appendInt(b, r.partition != nullptr, r.partition ? r.partition->numFaces : 0);
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: num_faces finish failed");
            arrays.push_back(a);
        }
        {
            arrow::Int32Builder b;
            for (const auto& r : rows)
                appendInt(b, r.partition != nullptr, r.partition ? r.partition->numLods : 0);
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: num_lods finish failed");
            arrays.push_back(a);
        }
        {
            arrow::BinaryBuilder b;
            for (const auto& r : rows)
                appendBinary(b, r.partition != nullptr, r.partition ? r.partition->baseMesh : std::vector<std::uint8_t>{});
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: base_mesh finish failed");
            arrays.push_back(a);
        }
        {
            arrow::BinaryBuilder b;
            for (const auto& r : rows)
                appendText(b, r.partition != nullptr, r.partition ? r.partition->baseMeshCodec : std::string());
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: base_mesh_codec finish failed");
            arrays.push_back(a);
        }
        {
            arrow::BinaryBuilder b;
            for (const auto& r : rows)
                appendText(b, r.object != nullptr, r.object ? dictionary : std::string());
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: stream_dictionary finish failed");
            arrays.push_back(a);
        }
        const char* countNames[6] = {kCountF, kCountEt, kCountEf, kCountV, kCountCf, kCountCv};
        const std::int32_t PaperLodRow::*countFields[6] = {&PaperLodRow::countF, &PaperLodRow::countEt, &PaperLodRow::countEf,
                                                           &PaperLodRow::countV, &PaperLodRow::countCf, &PaperLodRow::countCv};
        (void)countNames;
        for (int i = 0; i < 6; ++i) {
            arrow::Int32Builder b;
            for (const auto& r : rows) {
                appendInt(b, r.lod != nullptr, r.lod ? r.lod->*(countFields[i]) : 0);
            }
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: count column finish failed");
            arrays.push_back(a);
        }
        {
            arrow::BinaryBuilder b;
            for (const auto& r : rows)
                appendBinary(b, r.lod != nullptr, r.lod ? r.lod->F : std::vector<std::uint8_t>{});
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: F finish failed");
            arrays.push_back(a);
        }
        {
            arrow::BinaryBuilder b;
            for (const auto& r : rows)
                appendBinary(b, r.lod != nullptr, r.lod ? r.lod->Et : std::vector<std::uint8_t>{});
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: Et finish failed");
            arrays.push_back(a);
        }
        {
            arrow::BinaryBuilder b;
            for (const auto& r : rows)
                appendBinary(b, r.lod != nullptr, r.lod ? r.lod->Ef : std::vector<std::uint8_t>{});
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: Ef finish failed");
            arrays.push_back(a);
        }
        {
            arrow::BinaryBuilder b;
            for (const auto& r : rows)
                appendBinary(b, r.lod != nullptr, r.lod ? r.lod->V : std::vector<std::uint8_t>{});
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: V finish failed");
            arrays.push_back(a);
        }
        auto addInt32List = [&](const std::vector<std::vector<std::int32_t>>& perRow) {
            auto vb = std::make_shared<arrow::Int32Builder>();
            arrow::ListBuilder lb(arrow::default_memory_pool(), vb);
            for (const auto& r : rows) {
                const bool has = r.lod != nullptr;
                appendInt32List(lb, *vb, has, has ? perRow[static_cast<std::size_t>(&r - rows.data())] : std::vector<std::int32_t>{});
            }
            std::shared_ptr<arrow::Array> a;
            if (!lb.Finish(&a).ok())
                throw std::runtime_error("PaperStream: int list finish failed");
            arrays.push_back(a);
        };
        {
            std::vector<std::vector<std::int32_t>> cols[3];
            for (const auto& r : rows) {
                for (int i = 0; i < 3; ++i)
                    cols[i].push_back(r.lod ? axis(r.lod->CfQ, i) : std::vector<std::int32_t>{});
            }
            for (int i = 0; i < 3; ++i)
                addInt32List(cols[i]);
        }
        {
            std::vector<std::vector<std::int32_t>> cols[3];
            for (const auto& r : rows) {
                for (int i = 0; i < 3; ++i)
                    cols[i].push_back(r.lod ? axis(r.lod->CvQ, i) : std::vector<std::int32_t>{});
            }
            for (int i = 0; i < 3; ++i)
                addInt32List(cols[i]);
        }
        auto addFloatList = [&](const std::vector<std::vector<float>>& perRow) {
            auto vb = std::make_shared<arrow::FloatBuilder>();
            arrow::ListBuilder lb(arrow::default_memory_pool(), vb);
            for (const auto& r : rows) {
                const bool has = r.lod != nullptr;
                appendFloatList(lb, *vb, has, has ? perRow[static_cast<std::size_t>(&r - rows.data())] : std::vector<float>{});
            }
            std::shared_ptr<arrow::Array> a;
            if (!lb.Finish(&a).ok())
                throw std::runtime_error("PaperStream: float list finish failed");
            arrays.push_back(a);
        };
        {
            std::vector<std::vector<float>> cols[3];
            for (const auto& r : rows) {
                for (int i = 0; i < 3; ++i)
                    cols[i].push_back(r.lod ? axis(r.lod->CfF, i) : std::vector<float>{});
            }
            for (int i = 0; i < 3; ++i)
                addFloatList(cols[i]);
        }
        {
            std::vector<std::vector<float>> cols[3];
            for (const auto& r : rows) {
                for (int i = 0; i < 3; ++i)
                    cols[i].push_back(r.lod ? axis(r.lod->CvF, i) : std::vector<float>{});
            }
            for (int i = 0; i < 3; ++i)
                addFloatList(cols[i]);
        }
        {
            arrow::Int32Builder b;
            for (const auto& r : rows)
                appendInt(b, r.boundary != nullptr, r.boundary ? r.boundary->partitionA : -1);
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: boundary_a finish failed");
            arrays.push_back(a);
        }
        {
            arrow::Int32Builder b;
            for (const auto& r : rows)
                appendInt(b, r.boundary != nullptr, r.boundary ? r.boundary->partitionB : -1);
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: boundary_b finish failed");
            arrays.push_back(a);
        }
        auto addBoundaryList = [&](const std::vector<std::vector<std::int32_t>>& perRow) {
            auto vb = std::make_shared<arrow::Int32Builder>();
            arrow::ListBuilder lb(arrow::default_memory_pool(), vb);
            for (std::size_t rowIndex = 0; rowIndex < rows.size(); ++rowIndex) {
                const bool has = rows[rowIndex].boundary != nullptr;
                appendInt32List(lb, *vb, has, has ? perRow[rowIndex] : std::vector<std::int32_t>{});
            }
            std::shared_ptr<arrow::Array> a;
            if (!lb.Finish(&a).ok())
                throw std::runtime_error("PaperStream: boundary list finish failed");
            arrays.push_back(a);
        };
        {
            std::vector<std::vector<std::int32_t>> cols[3];
            for (const auto& r : rows) {
                cols[0].push_back(r.boundary ? r.boundary->sharedVertices : std::vector<std::int32_t>{});
                cols[1].push_back(r.boundary ? r.boundary->sharedEdges : std::vector<std::int32_t>{});
                cols[2].push_back(r.boundary ? r.boundary->stopVertices : std::vector<std::int32_t>{});
            }
            for (int i = 0; i < 3; ++i)
                addBoundaryList(cols[i]);
        }
        for (const auto& spec : extensionSpecs) {
            arrow::BinaryBuilder b;
            for (const auto& r : rows) {
                const std::vector<std::uint8_t>* bytes = nullptr;
                if (r.lod) {
                    auto it = r.lod->extension.find(spec.name);
                    if (it != r.lod->extension.end())
                        bytes = &it->second;
                }
                appendBinary(b, bytes != nullptr, bytes ? *bytes : std::vector<std::uint8_t>{});
            }
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: extension stream finish failed: " + spec.name);
            arrays.push_back(a);
        }
        if (arrays.size() != static_cast<std::size_t>(schema->num_fields())) {
            throw std::runtime_error("PaperStream: column count mismatch (schema " + std::to_string(schema->num_fields()) + ", arrays " +
                                     std::to_string(arrays.size()) + ")");
        }
        return arrow::Table::Make(schema, arrays, static_cast<int64_t>(n));
    }

    std::shared_ptr<parquet::WriterProperties> buildWriterProperties() {
        auto builder = parquet::WriterProperties::Builder();
        builder.compression(parquet::Compression::ZSTD);
        builder.compression_level(3);
        builder.data_pagesize(64 * 1024);
        builder.write_batch_size(1024);
        builder.enable_statistics();
        for (const char* name : {kObjectId, kPartitionId, kLodId, kNumRounds, kTotalVertices, kTotalFaces, kNumPartitions, kQuantBits, kSeedV0,
                                 kSeedV1, kNumVertices, kNumFaces, kNumLods, kCountF, kCountEt, kCountEf, kCountV, kCountCf, kCountCv}) {
            builder.encoding(name, parquet::Encoding::DELTA_BINARY_PACKED);
        }
        for (const char* name : {kBboxMinX, kBboxMinY, kBboxMinZ, kQuantStep}) {
            builder.encoding(name, parquet::Encoding::BYTE_STREAM_SPLIT);
        }
        for (const char* name : {kF, kEt, kEf, kV, kBaseMesh, kBaseMeshCodec, kStreamDictionary}) {
            builder.disable_dictionary(name);
            builder.encoding(name, parquet::Encoding::PLAIN);
        }
        return builder.build();
    }

    std::shared_ptr<arrow::Schema> buildCompactSchema() {
        std::vector<std::shared_ptr<arrow::Field>> fields;
        fields.push_back(arrow::field(kRecordType, arrow::int8(), false));
        fields.push_back(arrow::field(kObjectId, arrow::int32(), false));
        fields.push_back(arrow::field(kPartitionId, arrow::int32(), false));
        fields.push_back(arrow::field(kLodId, arrow::int32(), false));
        fields.push_back(arrow::field("header", arrow::binary(), true));
        fields.push_back(arrow::field("base_mesh", arrow::binary(), true));
        fields.push_back(arrow::field("legacy_bundle", arrow::binary(), true));
        fields.push_back(arrow::field("connectivity", arrow::binary(), true));
        fields.push_back(arrow::field("geometry", arrow::binary(), true));
        fields.push_back(arrow::field("extensions", arrow::binary(), true));
        fields.push_back(arrow::field("boundary", arrow::binary(), true));
        return arrow::schema(fields);
    }

    std::shared_ptr<arrow::Table>
    buildCompactTable(const std::shared_ptr<arrow::Schema>& schema, const std::vector<RowRef>& rows, const std::string& dictionary,
                      const std::vector<std::uint8_t>& legacyBundle) {
        const std::size_t n = rows.size();
        std::vector<std::shared_ptr<arrow::Array>> arrays;
        arrays.reserve(10);

        {
            arrow::Int8Builder b;
            for (const auto& r : rows)
                appendInt(b, true, static_cast<std::int8_t>(r.type));
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: compact record_type finish failed");
            arrays.push_back(a);
        }
        {
            arrow::Int32Builder b;
            for (const auto& r : rows) {
                std::int32_t v = 0;
                if (r.object)
                    v = r.object->objectId;
                else if (r.partition)
                    v = r.partition->objectId;
                else if (r.lod)
                    v = r.lod->objectId;
                else if (r.boundary)
                    v = r.boundary->objectId;
                appendInt(b, true, v);
            }
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: compact object_id finish failed");
            arrays.push_back(a);
        }
        {
            arrow::Int32Builder b;
            for (const auto& r : rows) {
                std::int32_t v = -1;
                if (r.partition)
                    v = r.partition->partitionId;
                else if (r.lod)
                    v = r.lod->partitionId;
                appendInt(b, true, v);
            }
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: compact partition_id finish failed");
            arrays.push_back(a);
        }
        {
            arrow::Int32Builder b;
            for (const auto& r : rows)
                appendInt(b, true, r.lod ? r.lod->lodId : -1);
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: compact lod_id finish failed");
            arrays.push_back(a);
        }

        auto addBinary = [&](const auto& packFn) {
            arrow::BinaryBuilder b;
            for (const auto& r : rows) {
                std::vector<std::uint8_t> bytes;
                const bool has = packFn(r, bytes);
                appendBinary(b, has, bytes);
            }
            std::shared_ptr<arrow::Array> a;
            if (!b.Finish(&a).ok())
                throw std::runtime_error("PaperStream: compact binary finish failed");
            arrays.push_back(a);
        };

        addBinary([&](const RowRef& r, std::vector<std::uint8_t>& out) {
            if (r.object) {
                out.assign(dictionary.begin(), dictionary.end());
                return true;
            }
            if (r.partition) {
                out = packPaperPartitionHeader(*r.partition);
                return true;
            }
            return false;
        });
        addBinary([&](const RowRef& r, std::vector<std::uint8_t>& out) {
            if (!r.partition)
                return false;
            out = r.partition->baseMesh;
            return true;
        });
        addBinary([&](const RowRef& r, std::vector<std::uint8_t>& out) {
            if (!r.object) return false;
            out = legacyBundle;
            return true;
        });
        addBinary([&](const RowRef& r, std::vector<std::uint8_t>& out) {
            if (!r.lod)
                return false;
            out = packPaperConnectivity(*r.lod);
            return true;
        });
        addBinary([&](const RowRef& r, std::vector<std::uint8_t>& out) {
            if (!r.lod)
                return false;
            out = packPaperGeometry(*r.lod);
            return true;
        });
        addBinary([&](const RowRef& r, std::vector<std::uint8_t>& out) {
            if (!r.lod)
                return false;
            out = packPaperExtensions(*r.lod);
            return true;
        });
        addBinary([&](const RowRef& r, std::vector<std::uint8_t>& out) {
            if (!r.boundary)
                return false;
            out = packPaperBoundary(*r.boundary);
            return true;
        });

        if (arrays.size() != static_cast<std::size_t>(schema->num_fields())) {
            throw std::runtime_error("PaperStream: compact column count mismatch (schema " + std::to_string(schema->num_fields()) + ", arrays " +
                                     std::to_string(arrays.size()) + ")");
        }
        return arrow::Table::Make(schema, arrays, static_cast<int64_t>(n));
    }

    PaperSchemaMode effectiveSchemaMode(const PaperObjectRow& object) {
        if (const char* env = std::getenv("PMSF_PAPER_SCHEMA")) {
            if (std::string(env) == "typed")
                return PaperSchemaMode::Typed;
            if (std::string(env) == "compact")
                return PaperSchemaMode::Compact;
        }
        return object.schemaMode;
    }

    std::shared_ptr<parquet::WriterProperties> buildCompactWriterProperties() {
        auto builder = parquet::WriterProperties::Builder();
        builder.compression(parquet::Compression::BROTLI);
        builder.compression_level(11);
        builder.data_pagesize(64 * 1024);
        builder.write_batch_size(1024);
        builder.disable_statistics();
        builder.encoding(kObjectId, parquet::Encoding::DELTA_BINARY_PACKED);
        builder.encoding(kPartitionId, parquet::Encoding::DELTA_BINARY_PACKED);
        builder.encoding(kLodId, parquet::Encoding::DELTA_BINARY_PACKED);
        for (const char* name : {"header", "base_mesh", "connectivity", "geometry", "extensions", "boundary"}) {
            builder.disable_dictionary(name);
            builder.encoding(name, parquet::Encoding::PLAIN);
        }
        return builder.build();
    }

    void writeTable(parquet::arrow::FileWriter& writer, const std::shared_ptr<arrow::Table>& table) {
        if (!table || table->num_rows() == 0)
            return;
        auto st = writer.WriteTable(*table, table->num_rows());
        if (!st.ok())
            throw std::runtime_error("PaperStream: WriteTable failed: " + st.ToString());
    }

}  // namespace

void PaperStreamWriter::write(const PaperStreamTable& table, const std::string& path) const {
    if (table.partitions.empty()) {
        throw std::runtime_error("PaperStream: at least one partition is required");
    }
    const PaperSchemaMode mode = effectiveSchemaMode(table.object);
    auto schema = mode == PaperSchemaMode::Compact ? buildCompactSchema() : buildSchema(table.object);
    // M2c PPMC: the native reader does not need the legacy bundle.  Keep the
    // switch PPMC-only so other algorithms still fail-fast without it.
    std::map<std::string, std::vector<std::uint8_t>> legacyFilesToWrite = table.legacyFiles;
    // M2c: PPMC's native paper reader does not need the legacy bundle, so the
    // PPMC paper artifact is bridge-free by default; the env switch also lets
    // another algorithm be measured bridge-free while its native reader lands.
    const char* dropLegacyEnv = std::getenv("PMSF_PAPER_DROP_LEGACY");
    const bool dropLegacy = table.object.algorithm == "ppmc" ||
                            (dropLegacyEnv && *dropLegacyEnv &&
                             std::strcmp(dropLegacyEnv, "0") != 0);
    if (dropLegacy) {
        legacyFilesToWrite.clear();
    }
    const std::vector<std::uint8_t> legacyBundle = packPaperLegacyBundle(legacyFilesToWrite);
    std::string dictError;
    const std::string dictionary = serializePaperStreamDictionary(table.object, &dictError);
    if (dictionary.empty()) {
        throw std::runtime_error("PaperStream: dictionary serialization failed: " + dictError);
    }

    auto sinkResult = arrow::io::FileOutputStream::Open(path);
    if (!sinkResult.ok()) {
        throw std::runtime_error("PaperStream: cannot open output: " + path + ": " + sinkResult.status().ToString());
    }
    auto sink = sinkResult.ValueOrDie();

    auto arrowProps = parquet::ArrowWriterProperties::Builder().store_schema()->build();
    auto props = mode == PaperSchemaMode::Compact ? buildCompactWriterProperties() : buildWriterProperties();
    auto writerResult = parquet::arrow::FileWriter::Open(*schema, arrow::default_memory_pool(), sink, props, arrowProps);
    if (!writerResult.ok()) {
        throw std::runtime_error("PaperStream: cannot create Parquet writer: " + writerResult.status().ToString());
    }
    auto writer = std::move(writerResult).ValueOrDie();

    auto makeTable = [&](const std::vector<RowRef>& rows) {
        return mode == PaperSchemaMode::Compact
                   ? buildCompactTable(schema, rows, dictionary, legacyBundle)
                   : buildTable(schema, rows, dictionary, extensionSpecsOf(table.object));
    };

    // RG0: object.
    {
        std::vector<RowRef> rows;
        RowRef r;
        r.type = PaperRecordType::Object;
        r.object = &table.object;
        rows.push_back(r);
        writeTable(*writer, makeTable(rows));
    }

    // RG1..: partition groups.  Partitions are laid out in vector order; by
    // default each row group holds one partition (stride=1).  A larger
    // stride trades ROI granularity for less footer metadata.
    std::map<std::int32_t, std::vector<const PaperLodRow*>> lodsByPartition;
    for (const auto& lod : table.lods) {
        lodsByPartition[lod.partitionId].push_back(&lod);
    }
    for (auto& [partitionId, lods] : lodsByPartition) {
        (void)partitionId;
        std::stable_sort(lods.begin(), lods.end(), [](const PaperLodRow* a, const PaperLodRow* b) { return a->lodId < b->lodId; });
    }
    const int stride = std::max(1, table.object.partitionsPerRowGroup);
    for (std::size_t start = 0; start < table.partitions.size(); start += static_cast<std::size_t>(stride)) {
        std::vector<RowRef> rows;
        const std::size_t end = std::min(table.partitions.size(), start + static_cast<std::size_t>(stride));
        for (std::size_t i = start; i < end; ++i) {
            const auto& partition = table.partitions[i];
            RowRef p;
            p.type = PaperRecordType::Partition;
            p.partition = &partition;
            rows.push_back(p);
            auto it = lodsByPartition.find(partition.partitionId);
            if (it != lodsByPartition.end()) {
                for (const PaperLodRow* lod : it->second) {
                    if (lod->objectId != partition.objectId)
                        continue;
                    RowRef r;
                    r.type = PaperRecordType::Lod;
                    r.lod = lod;
                    rows.push_back(r);
                }
            }
        }
        writeTable(*writer, makeTable(rows));
    }

    // Optional boundary row group.
    if (!table.boundaries.empty()) {
        std::vector<RowRef> rows;
        for (const auto& boundary : table.boundaries) {
            RowRef r;
            r.type = PaperRecordType::Boundary;
            r.boundary = &boundary;
            rows.push_back(r);
        }
        writeTable(*writer, makeTable(rows));
    }

    auto closeStatus = writer->Close();
    if (!closeStatus.ok()) {
        throw std::runtime_error("PaperStream: writer close failed: " + closeStatus.ToString());
    }
    auto sinkClose = sink->Close();
    if (!sinkClose.ok()) {
        throw std::runtime_error("PaperStream: sink close failed: " + sinkClose.ToString());
    }
}

}  // namespace pmsf
