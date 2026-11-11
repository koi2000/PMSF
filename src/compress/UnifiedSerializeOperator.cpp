/*
 * @file UnifiedSerializeOperator.cpp
 * @brief Minimal end-to-end round-trip implementation.
 *
 * Format (single file):
 *   One Arrow table, one row per (round_index, symbol_index_in_round).
 *   Columns:
 *     round INT32
 *     pos_in_round INT32  (for reconstruction ordering)
 *     channel INT8        (which core channel this row belongs to; see CoreChannel)
 *     bool_val BOOL       (legacy compatibility for F/E/V rows)
 *     symbol_i32 INT32    (lossless F/E/V alphabet value)
 *     px FLOAT, py FLOAT, pz FLOAT (set for P / VP rows)
 *     + one column (or three for Vec3) per registered side channel
 *
 * Side channels are keyed by name and persisted as file-level KV metadata
 * describing their type/shape; the deserializer reconstructs the SpecList
 * at open time.
 *
 * Zero dependency on V2, on any algorithm's operator, or on the
 * compression pipeline — the whole class can be exercised from a unit
 * test with a hand-built SymbolStream.
 */
#include "UnifiedSerializeOperator.h"
#include "core/PMSFSideChannel.h"
#include "storage/PMSFSymbolStorageAdapter.h"
#include "storage/PMSFUnifiedArtifactWriter.h"

#include <arrow/api.h>
#include <arrow/io/api.h>
#include <arrow/util/key_value_metadata.h>
#include <parquet/arrow/reader.h>
#include <parquet/arrow/writer.h>
#include <parquet/properties.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>

namespace pmsf {

namespace {

// ----- helpers -----
inline void checkStatus(const arrow::Status& st, const char* where) {
    if (!st.ok()) {
        throw std::runtime_error(std::string("Unified: ") + where + ": " + st.ToString());
    }
}

std::string serializeSpecs(const std::vector<SideChannelSpec>& specs) {
    std::ostringstream os;
    for (const auto& s : specs) {
        os << s.name << ":" << parquetTypeName(s.cellType) << ":"
           << (s.shape == SideChannelSpec::Shape::Vec3 ? "Vec3" : "Scalar") << ";";
    }
    return os.str();
}

std::vector<SideChannelSpec> parseSpecs(const std::string& s) {
    std::vector<SideChannelSpec> out;
    std::size_t i = 0;
    while (i < s.size()) {
        auto sep1 = s.find(':', i);
        auto sep2 = s.find(':', sep1 + 1);
        auto sep3 = s.find(';', sep2 + 1);
        if (sep1 == std::string::npos || sep2 == std::string::npos || sep3 == std::string::npos)
            break;
        SideChannelSpec spec;
        spec.name = s.substr(i, sep1 - i);
        const std::string type_str = s.substr(sep1 + 1, sep2 - sep1 - 1);
        const std::string shape_str = s.substr(sep2 + 1, sep3 - sep2 - 1);
        if      (type_str == "bool")          spec.cellType = SideChannelSpec::Type::Bool;
        else if (type_str == "int8")          spec.cellType = SideChannelSpec::Type::Int8;
        else if (type_str == "int16")         spec.cellType = SideChannelSpec::Type::Int16;
        else if (type_str == "int32")         spec.cellType = SideChannelSpec::Type::Int32;
        else if (type_str == "uint16")        spec.cellType = SideChannelSpec::Type::UInt16;
        else if (type_str == "float32")       spec.cellType = SideChannelSpec::Type::Float;
        else if (type_str == "float32_vec3")  spec.cellType = SideChannelSpec::Type::Vec3f;
        else if (type_str == "binary")        spec.cellType = SideChannelSpec::Type::Bytes;
        spec.shape = (shape_str == "Vec3") ? SideChannelSpec::Shape::Vec3
                                           : SideChannelSpec::Shape::Scalar;
        appendPMSFSideChannelSpec(out, std::move(spec));
        i = sep3 + 1;
    }
    return out;
}

std::shared_ptr<arrow::DataType> arrowTypeFor(SideChannelSpec::Type t) {
    switch (t) {
        case SideChannelSpec::Type::Bool:   return arrow::boolean();
        case SideChannelSpec::Type::Int8:   return arrow::int8();
        case SideChannelSpec::Type::Int16:  return arrow::int16();
        case SideChannelSpec::Type::Int32:  return arrow::int32();
        case SideChannelSpec::Type::UInt16: return arrow::uint16();
        case SideChannelSpec::Type::Float:  return arrow::float32();
        case SideChannelSpec::Type::Vec3f:  return arrow::float32();  // 3 columns
        case SideChannelSpec::Type::Bytes:  return arrow::binary();
    }
    return arrow::null();
}

// Append a SideCell value to the appropriate Arrow builder(s). For Vec3 the
// caller passes three builders, one per axis.
void appendSideCell(const SideCell& v, SideChannelSpec::Type t,
                    std::shared_ptr<arrow::ArrayBuilder> b,
                    std::shared_ptr<arrow::ArrayBuilder> bY = nullptr,
                    std::shared_ptr<arrow::ArrayBuilder> bZ = nullptr) {
    switch (t) {
        case SideChannelSpec::Type::Bool:
            checkStatus(static_cast<arrow::BooleanBuilder*>(b.get())->Append(std::get<bool>(v)), "append_bool");
            return;
        case SideChannelSpec::Type::Int8:
            checkStatus(static_cast<arrow::Int8Builder*>(b.get())->Append(std::get<std::int8_t>(v)), "append_i8");
            return;
        case SideChannelSpec::Type::Int16:
            checkStatus(static_cast<arrow::Int16Builder*>(b.get())->Append(std::get<std::int16_t>(v)), "append_i16");
            return;
        case SideChannelSpec::Type::Int32:
            checkStatus(static_cast<arrow::Int32Builder*>(b.get())->Append(std::get<std::int32_t>(v)), "append_i32");
            return;
        case SideChannelSpec::Type::UInt16:
            checkStatus(static_cast<arrow::UInt16Builder*>(b.get())->Append(std::get<std::uint16_t>(v)), "append_u16");
            return;
        case SideChannelSpec::Type::Float:
            checkStatus(static_cast<arrow::FloatBuilder*>(b.get())->Append(std::get<float>(v)), "append_f32");
            return;
        case SideChannelSpec::Type::Vec3f: {
            const auto& p = std::get<MCGAL::Point>(v);
            checkStatus(static_cast<arrow::FloatBuilder*>(b.get())->Append(p.x()), "append_vec3_x");
            checkStatus(static_cast<arrow::FloatBuilder*>(bY.get())->Append(p.y()), "append_vec3_y");
            checkStatus(static_cast<arrow::FloatBuilder*>(bZ.get())->Append(p.z()), "append_vec3_z");
            return;
        }
        case SideChannelSpec::Type::Bytes: {
            const auto& bytes = std::get<std::vector<std::uint8_t>>(v);
            checkStatus(static_cast<arrow::BinaryBuilder*>(b.get())->Append(
                reinterpret_cast<const char*>(bytes.data()),
                static_cast<std::int32_t>(bytes.size())), "append_bytes");
            return;
        }
    }
}

void appendNullSideCell(SideChannelSpec::Type t,
                        std::shared_ptr<arrow::ArrayBuilder> b,
                        std::shared_ptr<arrow::ArrayBuilder> bY = nullptr,
                        std::shared_ptr<arrow::ArrayBuilder> bZ = nullptr) {
    checkStatus(b->AppendNull(), "append_null");
    if (bY) checkStatus(bY->AppendNull(), "append_null_y");
    if (bZ) checkStatus(bZ->AppendNull(), "append_null_z");
}

}  // namespace

// ======================================================================
//                                  SERIALIZE
// ======================================================================

void UnifiedSerializeOperator::serialize(const SymbolStream& stream,
                                           const std::string& path) {
    if (sideChannels_.empty()) {
        sideChannels_ = inferSideChannelSpecsForStorage(stream);
    }

    auto pool = arrow::default_memory_pool();

    arrow::Int32Builder round_b(pool);
    arrow::Int32Builder pos_b(pool);
    arrow::Int8Builder channel_b(pool);
    arrow::BooleanBuilder bool_b(pool);
    arrow::Int32Builder symbol_b(pool);
    arrow::FloatBuilder px_b(pool);
    arrow::FloatBuilder py_b(pool);
    arrow::FloatBuilder pz_b(pool);

    // Per side channel: one builder (Scalar) or three (Vec3).
    std::vector<std::shared_ptr<arrow::ArrayBuilder>> scBuilders;
    std::vector<std::shared_ptr<arrow::ArrayBuilder>> scBuildersY;
    std::vector<std::shared_ptr<arrow::ArrayBuilder>> scBuildersZ;
    for (const auto& s : sideChannels_) {
        auto type = arrowTypeFor(s.cellType);
        std::shared_ptr<arrow::ArrayBuilder> b;
        if (s.cellType == SideChannelSpec::Type::Vec3f) {
            b = std::make_shared<arrow::FloatBuilder>(pool);
            scBuildersY.push_back(std::make_shared<arrow::FloatBuilder>(pool));
            scBuildersZ.push_back(std::make_shared<arrow::FloatBuilder>(pool));
        } else if (s.cellType == SideChannelSpec::Type::Bool) {
            b = std::make_shared<arrow::BooleanBuilder>(pool);
            scBuildersY.push_back(nullptr); scBuildersZ.push_back(nullptr);
        } else if (s.cellType == SideChannelSpec::Type::Int8) {
            b = std::make_shared<arrow::Int8Builder>(pool);
            scBuildersY.push_back(nullptr); scBuildersZ.push_back(nullptr);
        } else if (s.cellType == SideChannelSpec::Type::Int16) {
            b = std::make_shared<arrow::Int16Builder>(pool);
            scBuildersY.push_back(nullptr); scBuildersZ.push_back(nullptr);
        } else if (s.cellType == SideChannelSpec::Type::Int32) {
            b = std::make_shared<arrow::Int32Builder>(pool);
            scBuildersY.push_back(nullptr); scBuildersZ.push_back(nullptr);
        } else if (s.cellType == SideChannelSpec::Type::UInt16) {
            b = std::make_shared<arrow::UInt16Builder>(pool);
            scBuildersY.push_back(nullptr); scBuildersZ.push_back(nullptr);
        } else if (s.cellType == SideChannelSpec::Type::Float) {
            b = std::make_shared<arrow::FloatBuilder>(pool);
            scBuildersY.push_back(nullptr); scBuildersZ.push_back(nullptr);
        } else {  // Bytes
            b = std::make_shared<arrow::BinaryBuilder>(pool);
            scBuildersY.push_back(nullptr); scBuildersZ.push_back(nullptr);
        }
        scBuilders.push_back(b);
    }

    // Helper lambdas to append a "row" of core + side channels.
    auto appendRow = [&](int round, int pos, CoreChannel ch,
                         bool has_bool, std::int32_t symbol_v,
                         bool has_point, const MCGAL::Point* pt,
                         int side_idx_in_round) {
        checkStatus(round_b.Append(round), pmsf::kUnifiedRoundColumn);
        checkStatus(pos_b.Append(pos), pmsf::kUnifiedPositionColumn);
        checkStatus(channel_b.Append(static_cast<int8_t>(ch)), pmsf::kUnifiedChannelColumn);
        if (has_bool) checkStatus(bool_b.Append(symbol_v != 0), "bool");
        else          checkStatus(bool_b.AppendNull(), "bool_null");
        if (has_bool) {
            checkStatus(symbol_b.Append(symbol_v),
                        pmsf::kUnifiedSymbolI32Column);
        } else {
            checkStatus(symbol_b.AppendNull(), "symbol_i32_null");
        }
        if (has_point && pt) {
            checkStatus(px_b.Append(pt->x()), pmsf::kUnifiedPointXColumn);
            checkStatus(py_b.Append(pt->y()), pmsf::kUnifiedPointYColumn);
            checkStatus(pz_b.Append(pt->z()), pmsf::kUnifiedPointZColumn);
        } else {
            checkStatus(px_b.AppendNull(), "px_null");
            checkStatus(py_b.AppendNull(), "py_null");
            checkStatus(pz_b.AppendNull(), "pz_null");
        }
        // Side channels: look up the per-round vector by channel name; if
        // the algorithm has emitted fewer cells than the core channel
        // has rows, pad with nulls.
        for (std::size_t i = 0; i < sideChannels_.size(); ++i) {
            const auto& spec = sideChannels_[i];
            const bool appended =
                round < static_cast<int>(stream.size()) &&
                side_idx_in_round >= 0 &&
                pmsf::visitPMSFSideChannelCellAt(
                    stream[static_cast<std::size_t>(round)],
                    spec.name,
                    static_cast<std::size_t>(side_idx_in_round),
                    [&](const auto& cell) {
                        appendSideCell(cell,
                                       spec.cellType,
                                       scBuilders[i],
                                       scBuildersY[i],
                                       scBuildersZ[i]);
                    });
            if (!appended) {
                appendNullSideCell(spec.cellType, scBuilders[i], scBuildersY[i], scBuildersZ[i]);
            }
        }
    };

    for (std::size_t r = 0; r < stream.size(); ++r) {
        const RoundSymbols& rs = stream[r];
        int pos = 0;
        int side_pos = 0;  // side-channel cells advance in lock-step with emitted symbols
        for (std::int32_t c : rs.facetSymbols) {
            appendRow(static_cast<int>(r), pos++, CoreChannel::FacetSymbol, true, c, false, nullptr, side_pos++);
        }
        for (std::int32_t c : rs.edgeSymbols) {
            appendRow(static_cast<int>(r), pos++, CoreChannel::EdgeSymbol, true, c, false, nullptr, side_pos++);
        }
        for (std::int32_t c : rs.vertexSymbols) {
            appendRow(static_cast<int>(r), pos++, CoreChannel::VertexSymbol, true, c, false, nullptr, side_pos++);
        }
        for (const auto& p : rs.points) {
            appendRow(static_cast<int>(r), pos++, CoreChannel::Point, false, 0, true, &p, side_pos++);
        }
        for (const auto& p : rs.vertexPoints) {
            appendRow(static_cast<int>(r), pos++, CoreChannel::VertexPoint, false, 0, true, &p, side_pos++);
        }
        std::size_t max_side_cells = 0;
        for (const auto& spec : sideChannels_) {
            max_side_cells =
                std::max(max_side_cells, pmsf::paperSideChannelCellCount(rs, spec.name));
        }
        while (side_pos < static_cast<int>(max_side_cells)) {
            appendRow(static_cast<int>(r), pos++, static_cast<CoreChannel>(-1),
                      false, 0, false, nullptr, side_pos++);
        }
    }

    // Finalize arrays.
    std::shared_ptr<arrow::Array> round_a, pos_a, channel_a, bool_a, symbol_a, px_a, py_a, pz_a;
    checkStatus(round_b.Finish(&round_a), "finish round");
    checkStatus(pos_b.Finish(&pos_a), "finish pos");
    checkStatus(channel_b.Finish(&channel_a), "finish channel");
    checkStatus(bool_b.Finish(&bool_a), "finish bool");
    checkStatus(symbol_b.Finish(&symbol_a), "finish symbol_i32");
    checkStatus(px_b.Finish(&px_a), "finish px");
    checkStatus(py_b.Finish(&py_a), "finish py");
    checkStatus(pz_b.Finish(&pz_a), "finish pz");

    std::vector<std::shared_ptr<arrow::Array>> scArrays;
    std::vector<std::shared_ptr<arrow::Array>> scArraysY;
    std::vector<std::shared_ptr<arrow::Array>> scArraysZ;
    for (std::size_t i = 0; i < sideChannels_.size(); ++i) {
        std::shared_ptr<arrow::Array> a;
        checkStatus(scBuilders[i]->Finish(&a), "finish sc");
        scArrays.push_back(a);
        if (sideChannels_[i].cellType == SideChannelSpec::Type::Vec3f) {
            std::shared_ptr<arrow::Array> ay, az;
            checkStatus(scBuildersY[i]->Finish(&ay), "finish sc_y");
            checkStatus(scBuildersZ[i]->Finish(&az), "finish sc_z");
            scArraysY.push_back(ay);
            scArraysZ.push_back(az);
        } else {
            scArraysY.push_back(nullptr);
            scArraysZ.push_back(nullptr);
        }
    }

    // Build schema.
    std::vector<std::shared_ptr<arrow::Field>> fields = {
        arrow::field(pmsf::kUnifiedRoundColumn, arrow::int32()),
        arrow::field(pmsf::kUnifiedPositionColumn, arrow::int32()),
        arrow::field(pmsf::kUnifiedChannelColumn, arrow::int8()),
        arrow::field(pmsf::kUnifiedBoolValueColumn, arrow::boolean()),
        arrow::field(pmsf::kUnifiedSymbolI32Column, arrow::int32()),
        arrow::field(pmsf::kUnifiedPointXColumn, arrow::float32()),
        arrow::field(pmsf::kUnifiedPointYColumn, arrow::float32()),
        arrow::field(pmsf::kUnifiedPointZColumn, arrow::float32()),
    };
    std::vector<std::shared_ptr<arrow::Array>> cols = {round_a, pos_a, channel_a, bool_a, symbol_a, px_a, py_a, pz_a};
    for (std::size_t i = 0; i < sideChannels_.size(); ++i) {
        const auto& s = sideChannels_[i];
        const auto names = sideChannelColumnNames(s);
        if (s.cellType == SideChannelSpec::Type::Vec3f) {
            fields.push_back(arrow::field(names[0], arrow::float32()));
            fields.push_back(arrow::field(names[1], arrow::float32()));
            fields.push_back(arrow::field(names[2], arrow::float32()));
            cols.push_back(scArrays[i]);
            cols.push_back(scArraysY[i]);
            cols.push_back(scArraysZ[i]);
        } else {
            fields.push_back(arrow::field(names[0], arrowTypeFor(s.cellType)));
            cols.push_back(scArrays[i]);
        }
    }

    auto kv = arrow::KeyValueMetadata::Make(
        {pmsf::kPMSFVersionMetadataKey, pmsf::kPMSFSideChannelsMetadataKey, pmsf::kPMSFSymbolTableMetadataKey,
         pmsf::kPMSFSymbolMappingMetadataKey},
        {"3", serializeSpecs(sideChannels_), pmsfSymbolTableMode_ ? "1" : "0",
         pmsfSymbolMappingMode_ ? "1" : "0"});
    auto schema = std::make_shared<arrow::Schema>(fields, kv);
    auto table = arrow::Table::Make(schema, cols);

    // Write parquet.
    auto outFile = arrow::io::FileOutputStream::Open(path);
    checkStatus(outFile.status(), "open output");
    auto props = parquet::WriterProperties::Builder()
                     .compression(parquet::Compression::ZSTD)
                     ->compression_level(3)
                     ->build();
    auto arrowProps = parquet::ArrowWriterProperties::Builder()
                          .store_schema()
                          ->build();
    checkStatus(parquet::arrow::WriteTable(*table, pool, *outFile, /*chunk_size=*/64 * 1024,
                                           props, arrowProps),
                "WriteTable");
}

void UnifiedSerializeOperator::serialize(const PMSFSymbolTable& table,
                                           const std::string& path,
                                           bool emitSidecar) {
    pmsfSymbolTableMode_ = true;
    pmsfSymbolMappingMode_ = pmsfSymbolTableHasSymbolMapping(table);
    try {
        serialize(storageStreamFromPMSFSymbolTable(table), path);
        // 写侧车走 emitSidecar=false（见头文件 D5 说明）：否则 writePMSFSymbolSidecar
        // 会再次进入本函数，path 每层追加一次 _sidechannel.parquet，无界递归。
        if (emitSidecar) {
            writeUnifiedPMSFArtifactsForTable(path, table, false);
        }
    } catch (...) {
        pmsfSymbolTableMode_ = false;
        pmsfSymbolMappingMode_ = false;
        throw;
    }
    pmsfSymbolTableMode_ = false;
    pmsfSymbolMappingMode_ = false;
}

// ======================================================================
//                                DESERIALIZE
// ======================================================================

SymbolStream UnifiedDeserializeOperator::deserialize(const std::string& path) {
    auto pool = arrow::default_memory_pool();
    auto maybe_input = arrow::io::ReadableFile::Open(path, pool);
    checkStatus(maybe_input.status(), "open input");
    std::shared_ptr<arrow::io::ReadableFile> input = *maybe_input;

    std::unique_ptr<parquet::arrow::FileReader> reader;
    checkStatus(parquet::arrow::OpenFile(input, pool).Value(&reader), "OpenFile");

    std::shared_ptr<arrow::Table> table;
    checkStatus(reader->ReadTable(&table), "ReadTable");

    // Recover side-channel specs from schema metadata.
    sideChannels_.clear();
    pmsfSymbolTableMetadataPresent_ = false;
    pmsfSymbolTableMode_ = false;
    paperSymbolMappingMetadataPresent_ = false;
    pmsfSymbolMappingMode_ = false;
    if (table->schema()->metadata()) {
        std::string specs;
        if (table->schema()->metadata()->Get(pmsf::kPMSFSideChannelsMetadataKey).ok()) {
            specs = *table->schema()->metadata()->Get(pmsf::kPMSFSideChannelsMetadataKey);
        }
        sideChannels_ = parseSpecs(specs);
        auto paper = table->schema()->metadata()->Get(pmsf::kPMSFSymbolTableMetadataKey);
        if (paper.ok()) {
            pmsfSymbolTableMetadataPresent_ = true;
            pmsfSymbolTableMode_ = (*paper == "1");
        }
        auto mapping = table->schema()->metadata()->Get(pmsf::kPMSFSymbolMappingMetadataKey);
        if (mapping.ok()) {
            paperSymbolMappingMetadataPresent_ = true;
            pmsfSymbolMappingMode_ = (*mapping == "1");
        }
    }

    auto combined = table->CombineChunks(pool);
    checkStatus(combined.status(), "CombineChunks");
    std::shared_ptr<arrow::Table> ct = *combined;

    auto getCol = [&](const std::string& name) {
        auto ch = ct->GetColumnByName(name);
        if (!ch) throw std::runtime_error("Unified: missing column " + name);
        if (ch->num_chunks() != 1)
            throw std::runtime_error("Unified: column " + name + " not combined");
        return ch->chunk(0);
    };
    auto getOptionalCol = [&](const std::string& name) -> std::shared_ptr<arrow::Array> {
        auto ch = ct->GetColumnByName(name);
        if (!ch) return nullptr;
        if (ch->num_chunks() != 1)
            throw std::runtime_error("Unified: column " + name + " not combined");
        return ch->chunk(0);
    };
    auto round_a = std::static_pointer_cast<arrow::Int32Array>(getCol(pmsf::kUnifiedRoundColumn));
    auto pos_a = std::static_pointer_cast<arrow::Int32Array>(getCol(pmsf::kUnifiedPositionColumn));
    auto channel_a = std::static_pointer_cast<arrow::Int8Array>(getCol(pmsf::kUnifiedChannelColumn));
    auto bool_a = std::static_pointer_cast<arrow::BooleanArray>(getCol(pmsf::kUnifiedBoolValueColumn));
    auto symbol_base = getOptionalCol(pmsf::kUnifiedSymbolI32Column);
    std::shared_ptr<arrow::Int32Array> symbol_a =
        symbol_base ? std::static_pointer_cast<arrow::Int32Array>(symbol_base) : nullptr;
    auto px_a = std::static_pointer_cast<arrow::FloatArray>(getCol(pmsf::kUnifiedPointXColumn));
    auto py_a = std::static_pointer_cast<arrow::FloatArray>(getCol(pmsf::kUnifiedPointYColumn));
    auto pz_a = std::static_pointer_cast<arrow::FloatArray>(getCol(pmsf::kUnifiedPointZColumn));

    // Side-channel columns.
    std::vector<std::shared_ptr<arrow::Array>> scArrays, scY, scZ;
    for (const auto& s : sideChannels_) {
        const auto names = sideChannelColumnNames(s);
        scArrays.push_back(getCol(names[0]));
        if (s.cellType == SideChannelSpec::Type::Vec3f) {
            scY.push_back(getCol(names[1]));
            scZ.push_back(getCol(names[2]));
        } else {
            scY.push_back(nullptr);
            scZ.push_back(nullptr);
        }
    }

    // Determine number of rounds.
    int max_round = -1;
    for (int64_t i = 0; i < round_a->length(); ++i)
        max_round = std::max(max_round, round_a->Value(i));
    SymbolStream out(static_cast<std::size_t>(max_round + 1));

    auto sideCellFor = [&](std::size_t ci, int64_t row) -> SideCell {
        const auto& s = sideChannels_[ci];
        switch (s.cellType) {
            case SideChannelSpec::Type::Bool:
                return static_cast<arrow::BooleanArray*>(scArrays[ci].get())->Value(row);
            case SideChannelSpec::Type::Int8:
                return static_cast<arrow::Int8Array*>(scArrays[ci].get())->Value(row);
            case SideChannelSpec::Type::Int16:
                return static_cast<arrow::Int16Array*>(scArrays[ci].get())->Value(row);
            case SideChannelSpec::Type::Int32:
                return static_cast<arrow::Int32Array*>(scArrays[ci].get())->Value(row);
            case SideChannelSpec::Type::UInt16:
                return static_cast<arrow::UInt16Array*>(scArrays[ci].get())->Value(row);
            case SideChannelSpec::Type::Float:
                return static_cast<arrow::FloatArray*>(scArrays[ci].get())->Value(row);
            case SideChannelSpec::Type::Vec3f: {
                const float x = static_cast<arrow::FloatArray*>(scArrays[ci].get())->Value(row);
                const float y = static_cast<arrow::FloatArray*>(scY[ci].get())->Value(row);
                const float z = static_cast<arrow::FloatArray*>(scZ[ci].get())->Value(row);
                return MCGAL::Point(x, y, z);
            }
            case SideChannelSpec::Type::Bytes: {
                auto ba = static_cast<arrow::BinaryArray*>(scArrays[ci].get());
                int32_t len;
                const uint8_t* p = ba->GetValue(row, &len);
                return std::vector<std::uint8_t>(p, p + len);
            }
        }
        return static_cast<int8_t>(0);
    };

    for (int64_t i = 0; i < round_a->length(); ++i) {
        const int r = round_a->Value(i);
        const auto ch = static_cast<CoreChannel>(channel_a->Value(i));
        const std::int32_t symbol =
            (symbol_a && !symbol_a->IsNull(i))
                ? symbol_a->Value(i)
                : ((!bool_a->IsNull(i) && bool_a->Value(i)) ? 1 : 0);
        switch (ch) {
            case CoreChannel::FacetSymbol:
                appendRoundFacetSymbol(out[r], symbol);
                break;
            case CoreChannel::EdgeSymbol:
                appendRoundTriangulationEdgeSymbol(out[r], symbol);
                break;
            case CoreChannel::VertexSymbol:
                appendRoundVertexSymbol(out[r], symbol);
                break;
            case CoreChannel::Point:
                appendRoundRemovedVertexCoord(
                    out[r], px_a->Value(i), py_a->Value(i), pz_a->Value(i));
                break;
            case CoreChannel::VertexPoint:
                appendRoundShiftedVertexCoord(
                    out[r], px_a->Value(i), py_a->Value(i), pz_a->Value(i));
                break;
            default: break;
        }
        // Side channel cells — one cell per (row, side channel) unless the
        // column is null (meaning "no side value at this position").
        for (std::size_t ci = 0; ci < sideChannels_.size(); ++ci) {
            if (scArrays[ci]->IsNull(i)) continue;
            appendPaperSideCellSideChannel(
                out[r], sideChannels_[ci].name, sideCellFor(ci, i));
        }
    }

    return out;
}

}  // namespace pmsf
