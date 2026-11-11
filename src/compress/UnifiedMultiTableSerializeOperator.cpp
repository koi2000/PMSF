/*
 * @file UnifiedMultiTableSerializeOperator.cpp
 * @brief V3 多表 writer 实现。所有 "core" 工作（4 表）委托给 V2 基类；
 *        仅 side-channel 与 blob 这两张可选表由本文件独立构造。
 */
#include "UnifiedMultiTableSerializeOperator.h"
#include "UnifiedSchema.h"
#include "PMSFCollectorSymbolProjection.h"
#include "core/DefaultCodecPMSFSideChannel.h"
#include "core/PMSFSideChannel.h"
#include "storage/PMSFStorageContextSideChannel.h"

#include "storage/PMSFSymbolPayloadBlobWriter.h"
#include "storage/PMSFSymbolStorageAdapter.h"
#include "PMSFPayloadAdapterRegistry.h"
#include "storage/PMSFUnifiedArtifactWriter.h"

#include <arrow/api.h>
#include <arrow/io/api.h>
#include <arrow/util/key_value_metadata.h>
#include <parquet/arrow/writer.h>
#include <parquet/properties.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <string>
#include <utility>
#include <variant>
#include <vector>

// ============================================================================
// 路径变换
// ============================================================================
std::string UnifiedMultiTableSerializeOperator::toUnifiedPath(const std::string& path) {
    return pmsf::unifiedCompatOutputPath(path);
}

// ============================================================================
// 从 collector 推断 SymbolStream（供 side-channel 写回使用）
// ============================================================================
pmsf::SymbolStream UnifiedMultiTableSerializeOperator::buildStreamFromCollector() const {
    if (collector_) {
        if (pmsf::unifiedStorageEnabled()) {
            if (const auto* table = collector_->pmsfSymbolTable()) {
                if (pmsf::pmsfSymbolTableHasSymbolMapping(*table)) {
                    return pmsf::storageStreamFromPMSFSymbolTable(*table);
                }
            }
        }
        if (const auto* rounds = collector_->rounds()) {
            return pmsf::storageStreamFromUnifiedRounds(*rounds);
        }
    }

    return pmsf::storageStreamFromPMSFSymbolStream(
        pmsf::pmsfRoleSymbolStreamFromCollector(collector_.get()));
}

namespace {

}  // namespace

// ============================================================================
// 主 serialize —— 委托 V2 写 4 表，再写 V3 独有表
// ============================================================================
void UnifiedMultiTableSerializeOperator::serialize(std::string path) {
    // 1. 计算 V3 basename：把 path 的 .parquet 去掉后加 "_v3"，再让 V2 写 4 张
    //    `<base>_v3_{metadata,base_mesh,symbols,boundary}.parquet`。
    const std::string v3Path = toUnifiedPath(path);

    // 2. 委托 V2 基类写核心 4 张表（字节兼容 V2）。
    ColumnarSerializeOperator::serialize(v3Path);

    // 3. 计算 basename（与 V2::serialize 内部去扩展名逻辑一致）。
    std::string baseName = pmsf::unifiedStemFromArtifactPath(path);
    const bool skipCompatBaseTables =
        pmsf::unifiedStorageEnabled() &&
        (pmsf::pmsfStorageEnvEnabled("PMSF_PARQUET_SKIP_COMPAT_BASE_TABLES") ||
         pmsf::pmsfStorageEnvEnabled("PMSF_COLUMNAR_SKIP_COMPAT_BASE_TABLES") ||
         pmsf::pmsfStorageEnvEnabled("PMSF_UNIFIED_SKIP_COMPAT_BASE_TABLES"));
    bool unifiedBasePayloadPresent = false;
    if (skipCompatBaseTables) {
        std::vector<std::uint8_t> compactBase;
        if (buildV2CompactBinPayload(compactBase)) {
            pmsf::PMSFSymbolPayloadTable payloadRows;
            pmsf::appendParquetCompactBasePayload(
                payloadRows, std::move(compactBase));
            setPMSFSymbolPayloadTable(payloadRows);
            unifiedBasePayloadPresent = true;
        } else {
            std::cerr << "[UnifiedSerialize] unified base payload build failed; "
                      << "keep compat metadata/base artifacts" << std::endl;
        }
    }

    // 4. 写 V3 独有两张可选表。
    if (!streamInjected_) {
        pmsf::PMSFSymbolTable serializerTable;
        if (pmsf::unifiedStorageEnabled() &&
            buildPMSFSymbolTableWithSerializerSideChannels(serializerTable)) {
            pmsfSymbolTableMode_ = true;
            pmsfSymbolMappingMode_ =
                pmsf::pmsfSymbolTableHasSymbolMapping(serializerTable);
            stream_ = pmsf::storageStreamFromPMSFSymbolTable(serializerTable);
        } else {
            pmsfSymbolMappingMode_ =
                pmsf::unifiedStorageEnabled() &&
                pmsf::collectorHasPMSFSymbolMapping(collector_);
            pmsfSymbolTableMode_ = pmsfSymbolMappingMode_;
            stream_ = buildStreamFromCollector();
        }
    }
    bool hasCollectorPayloadRows = false;
    if (pmsf::unifiedStorageEnabled() && collector_) {
        if (const auto* payloads = collector_->paperSymbolPayloadTable()) {
            hasCollectorPayloadRows = !payloads->empty();
            setPMSFSymbolPayloadTable(*payloads);
        }
    }
    const bool hasInjectedPayloadRows =
        pmsf::unifiedStorageEnabled() &&
        std::any_of(blobs_.begin(), blobs_.end(),
                    [](const pmsf::NamedBlob& blob) {
                        return pmsf::isPMSFPayloadBlobName(blob.name);
                    });
    const bool skipCompatSymbolsTable =
        pmsf::unifiedStorageEnabled() &&
        (pmsf::pmsfStorageEnvEnabled("PMSF_PARQUET_SKIP_COMPAT_SYMBOLS_TABLE") ||
         pmsf::pmsfStorageEnvEnabled("PMSF_COLUMNAR_SKIP_COMPAT_SYMBOLS_TABLE") ||
         pmsf::pmsfStorageEnvEnabled("PMSF_UNIFIED_SKIP_COMPAT_SYMBOLS_TABLE"));
    bool mirroredSymbolsBlob = false;
    if (pmsf::unifiedStorageEnabled() && !lastSymbolsBlob_.empty() &&
        (skipCompatSymbolsTable || (!hasCollectorPayloadRows && !hasInjectedPayloadRows))) {
        pmsf::PMSFSymbolPayloadTable payloadRows;
        pmsf::appendParquetSymbolsRCPayload(
            payloadRows, lastSymbolsBlob_, getBlobLayout());
        setPMSFSymbolPayloadTable(payloadRows);
        mirroredSymbolsBlob = true;
    }
    if (pmsf::unifiedStorageEnabled() && !blobs_.empty() && !pmsfSymbolTableMode_) {
        if (stream_.empty()) {
            stream_.resize(1);
        }
        auto& round = stream_.front();
        pmsf::appendUnifiedBlobStorage(
            round, static_cast<std::int32_t>(blobs_.size()));
        pmsfSymbolTableMode_ = true;
        pmsfSymbolMappingMode_ = false;
    }
    if (pmsf::unifiedStorageEnabled() && !blobs_.empty()) {
        if (stream_.empty()) {
            stream_.resize(1);
        }
        auto& round = stream_.front();
        pmsf::appendParquetStorageAudit(
            round,
            "unified",
            skipCompatSymbolsTable ? 1 : 0,
            skipCompatBaseTables && unifiedBasePayloadPresent ? 1 : 0,
            unifiedBasePayloadPresent ? 1 : 0,
            mirroredSymbolsBlob ? 1 : 0,
            hasCollectorPayloadRows ? 1 : 0,
            pmsfSymbolMappingMode_ ? 0 : 1,
            pmsfSymbolMappingMode_ ? 1 : 0,
            hasInjectedPayloadRows ? 1 : 0);
        pmsfSymbolTableMode_ = true;
    }
    const bool allowSideChannelOutput =
        pmsf::unifiedStorageEnabled() || streamInjected_ || !sideChannels_.empty();
    if (allowSideChannelOutput) {
        auto inferred = pmsf::inferSideChannelSpecsForStorage(stream_);
        if (sideChannels_.empty()) {
            sideChannels_ = std::move(inferred);
        } else {
            for (auto& spec : inferred) {
                const bool exists = std::any_of(
                    sideChannels_.begin(),
                    sideChannels_.end(),
                    [&](const pmsf::SideChannelSpec& current) {
                        return current.name == spec.name;
	                    });
                if (!exists) {
                    pmsf::appendPMSFSideChannelSpec(
                        sideChannels_, std::move(spec));
                }
            }
        }
    }
    if (allowSideChannelOutput) {
        writeSideChannelTable(pmsf::paperSideChannelTablePath(baseName));
    }
    writeBlobTableForBase(baseName);

    // 5. 阶段 D：紧凑模式 —— 产出 `<base>_v3_compact.bin` 替代
    //    `_v3_metadata.parquet` + `_v3_base_mesh.parquet`。通过环境变量
    //    `PMSF_V3_COMPACT=1` 或 setCompactMode(true) 启用。
    bool wantCompact = compactMode_;
    if (!wantCompact) {
        const char* env = std::getenv("PMSF_V3_COMPACT");
        if (env && env[0] == '1') wantCompact = true;
    }
    if (wantCompact && !unifiedBasePayloadPresent) {
        const std::string compactPath = pmsf::paperCompactBinPath(baseName);
        if (writeCompactBin(compactPath)) {
            // 删除原 parquet 两张小表
            const std::string metaPath = pmsf::paperMetadataTablePath(baseName);
            const std::string baseMeshPath = pmsf::paperBaseMeshTablePath(baseName);
            pmsf::removePaperCompactBaseParquetArtifactsForBase(baseName);
            std::cout << "[UnifiedSerialize] 阶段 D 紧凑模式：删除 "
                      << metaPath << " + " << baseMeshPath << std::endl;
        }
    } else if (unifiedBasePayloadPresent) {
        pmsf::removePaperCompactBaseArtifactsForBase(baseName);
        std::cout << "[UnifiedSerialize] skip compat metadata/base artifacts; "
                  << "using unified "
                  << pmsf::parquetCompactBasePayloadSpec().payloadName
                  << std::endl;
    }

    std::cout << "[UnifiedSerialize] V3 多表写入完成: base=" << baseName
              << " sideChannels=" << sideChannels_.size()
              << " blobs=" << blobs_.size()
              << " compact=" << (wantCompact ? "ON" : "off") << std::endl;
}

// ============================================================================
// 第 5 张表：side-channel
// ============================================================================
namespace {

std::shared_ptr<arrow::DataType> arrowTypeFor(pmsf::SideChannelSpec::Type t) {
    switch (t) {
        case pmsf::SideChannelSpec::Type::Bool:   return arrow::boolean();
        case pmsf::SideChannelSpec::Type::Int8:   return arrow::int8();
        case pmsf::SideChannelSpec::Type::Int16:  return arrow::int16();
        case pmsf::SideChannelSpec::Type::Int32:  return arrow::int32();
        case pmsf::SideChannelSpec::Type::UInt16: return arrow::uint16();
        case pmsf::SideChannelSpec::Type::Float:  return arrow::float32();
        case pmsf::SideChannelSpec::Type::Vec3f:  return arrow::float32();  // 拆 3 列
        case pmsf::SideChannelSpec::Type::Bytes:  return arrow::binary();
    }
    return arrow::null();
}

const char* specTypeName(pmsf::SideChannelSpec::Type t) {
    switch (t) {
        case pmsf::SideChannelSpec::Type::Bool:   return "bool";
        case pmsf::SideChannelSpec::Type::Int8:   return "int8";
        case pmsf::SideChannelSpec::Type::Int16:  return "int16";
        case pmsf::SideChannelSpec::Type::Int32:  return "int32";
        case pmsf::SideChannelSpec::Type::UInt16: return "uint16";
        case pmsf::SideChannelSpec::Type::Float:  return "float32";
        case pmsf::SideChannelSpec::Type::Vec3f:  return "float32_vec3";
        case pmsf::SideChannelSpec::Type::Bytes:  return "binary";
    }
    return "unknown";
}

std::string serializeSpecs(const std::vector<pmsf::SideChannelSpec>& specs) {
    std::ostringstream os;
    for (const auto& s : specs) {
        os << s.name << ":" << specTypeName(s.cellType) << ":"
           << (s.shape == pmsf::SideChannelSpec::Shape::Vec3 ? "Vec3" : "Scalar") << ";";
    }
    return os.str();
}

inline void checkStatus(const arrow::Status& st, const char* where) {
    if (!st.ok()) throw std::runtime_error(std::string("UnifiedMT: ") + where + ": " + st.ToString());
}

}  // namespace

void UnifiedMultiTableSerializeOperator::writeSideChannelTable(const std::string& path) {
    if (sideChannels_.empty()) return;
    // 扫描 stream_：如所有 round 的 side 都为空则不写。
    bool anyData = false;
    for (const auto& rs : stream_) {
        for (const auto& spec : sideChannels_) {
            if (pmsf::paperSideChannelHasCells(rs, spec.name)) { anyData = true; break; }
        }
        if (anyData) break;
    }
    if (!anyData) return;

    auto pool = arrow::default_memory_pool();

    // 固定前缀列：round INT32 / pos INT32 / channel_name STRING。
    arrow::Int32Builder roundB(pool);
    arrow::Int32Builder posB(pool);
    arrow::StringBuilder nameB(pool);

    // 每个 side channel 一（或三）个数据列；对应行若不是该 channel，则写 null。
    std::vector<std::shared_ptr<arrow::ArrayBuilder>> scB, scBy, scBz;
    for (const auto& s : sideChannels_) {
        std::shared_ptr<arrow::ArrayBuilder> b;
        switch (s.cellType) {
            case pmsf::SideChannelSpec::Type::Bool:   b = std::make_shared<arrow::BooleanBuilder>(pool); break;
            case pmsf::SideChannelSpec::Type::Int8:   b = std::make_shared<arrow::Int8Builder>(pool); break;
            case pmsf::SideChannelSpec::Type::Int16:  b = std::make_shared<arrow::Int16Builder>(pool); break;
            case pmsf::SideChannelSpec::Type::Int32:  b = std::make_shared<arrow::Int32Builder>(pool); break;
            case pmsf::SideChannelSpec::Type::UInt16: b = std::make_shared<arrow::UInt16Builder>(pool); break;
            case pmsf::SideChannelSpec::Type::Float:
            case pmsf::SideChannelSpec::Type::Vec3f:  b = std::make_shared<arrow::FloatBuilder>(pool); break;
            case pmsf::SideChannelSpec::Type::Bytes:  b = std::make_shared<arrow::BinaryBuilder>(pool); break;
        }
        scB.push_back(b);
        if (s.cellType == pmsf::SideChannelSpec::Type::Vec3f) {
            scBy.push_back(std::make_shared<arrow::FloatBuilder>(pool));
            scBz.push_back(std::make_shared<arrow::FloatBuilder>(pool));
        } else {
            scBy.push_back(nullptr);
            scBz.push_back(nullptr);
        }
    }

    auto appendNullAllExcept = [&](std::size_t skipIdx) {
        for (std::size_t j = 0; j < sideChannels_.size(); ++j) {
            if (j == skipIdx) continue;
            checkStatus(scB[j]->AppendNull(), "null main");
            if (scBy[j]) checkStatus(scBy[j]->AppendNull(), "null y");
            if (scBz[j]) checkStatus(scBz[j]->AppendNull(), "null z");
        }
    };

    auto appendSideCell = [&](std::size_t ci, const pmsf::SideCell& v) {
        auto& s = sideChannels_[ci];
        switch (s.cellType) {
            case pmsf::SideChannelSpec::Type::Bool:
                checkStatus(static_cast<arrow::BooleanBuilder*>(scB[ci].get())->Append(std::get<bool>(v)), "b"); break;
            case pmsf::SideChannelSpec::Type::Int8:
                checkStatus(static_cast<arrow::Int8Builder*>(scB[ci].get())->Append(std::get<std::int8_t>(v)), "i8"); break;
            case pmsf::SideChannelSpec::Type::Int16:
                checkStatus(static_cast<arrow::Int16Builder*>(scB[ci].get())->Append(std::get<std::int16_t>(v)), "i16"); break;
            case pmsf::SideChannelSpec::Type::Int32:
                checkStatus(static_cast<arrow::Int32Builder*>(scB[ci].get())->Append(std::get<std::int32_t>(v)), "i32"); break;
            case pmsf::SideChannelSpec::Type::UInt16:
                checkStatus(static_cast<arrow::UInt16Builder*>(scB[ci].get())->Append(std::get<std::uint16_t>(v)), "u16"); break;
            case pmsf::SideChannelSpec::Type::Float:
                checkStatus(static_cast<arrow::FloatBuilder*>(scB[ci].get())->Append(std::get<float>(v)), "f32"); break;
            case pmsf::SideChannelSpec::Type::Vec3f: {
                const auto& p = std::get<MCGAL::Point>(v);
                checkStatus(static_cast<arrow::FloatBuilder*>(scB[ci].get())->Append(p.x()), "vx");
                checkStatus(static_cast<arrow::FloatBuilder*>(scBy[ci].get())->Append(p.y()), "vy");
                checkStatus(static_cast<arrow::FloatBuilder*>(scBz[ci].get())->Append(p.z()), "vz");
                break;
            }
            case pmsf::SideChannelSpec::Type::Bytes: {
                const auto& bytes = std::get<std::vector<std::uint8_t>>(v);
                checkStatus(static_cast<arrow::BinaryBuilder*>(scB[ci].get())->Append(
                    reinterpret_cast<const char*>(bytes.data()),
                    static_cast<std::int32_t>(bytes.size())), "bytes");
                break;
            }
        }
    };

    // 主循环：每轮每 channel 逐 cell 写一行。
    for (std::size_t r = 0; r < stream_.size(); ++r) {
        const auto& rs = stream_[r];
        for (std::size_t ci = 0; ci < sideChannels_.size(); ++ci) {
            std::size_t p = 0;
            pmsf::forEachPMSFSideChannelCell(rs, sideChannels_[ci].name, [&](const auto& cell) {
                checkStatus(roundB.Append(static_cast<int32_t>(r)), pmsf::kUnifiedRoundColumn);
                checkStatus(posB.Append(static_cast<int32_t>(p)), pmsf::kUnifiedPositionColumn);
                checkStatus(nameB.Append(sideChannels_[ci].name), "name");
                appendSideCell(ci, cell);
                appendNullAllExcept(ci);
                ++p;
            });
        }
    }

    // Finalize
    std::shared_ptr<arrow::Array> roundA, posA, nameA;
    checkStatus(roundB.Finish(&roundA), "finish round");
    checkStatus(posB.Finish(&posA), "finish pos");
    checkStatus(nameB.Finish(&nameA), "finish name");
    std::vector<std::shared_ptr<arrow::Array>> scA, scAy, scAz;
    for (std::size_t ci = 0; ci < sideChannels_.size(); ++ci) {
        std::shared_ptr<arrow::Array> a;
        checkStatus(scB[ci]->Finish(&a), "finish sc");
        scA.push_back(a);
        if (sideChannels_[ci].cellType == pmsf::SideChannelSpec::Type::Vec3f) {
            std::shared_ptr<arrow::Array> ay, az;
            checkStatus(scBy[ci]->Finish(&ay), "finish sc_y");
            checkStatus(scBz[ci]->Finish(&az), "finish sc_z");
            scAy.push_back(ay);
            scAz.push_back(az);
        } else {
            scAy.push_back(nullptr);
            scAz.push_back(nullptr);
        }
    }

    // Schema
    std::vector<std::shared_ptr<arrow::Field>> fields = {
        arrow::field(pmsf::kUnifiedRoundColumn, arrow::int32()),
        arrow::field(pmsf::kUnifiedPositionColumn, arrow::int32()),
        arrow::field(pmsf::kUnifiedChannelNameColumn, arrow::utf8()),
    };
    std::vector<std::shared_ptr<arrow::Array>> cols = {roundA, posA, nameA};
    for (std::size_t ci = 0; ci < sideChannels_.size(); ++ci) {
        const auto& s = sideChannels_[ci];
        const auto names = pmsf::sideChannelColumnNames(s);
        if (s.cellType == pmsf::SideChannelSpec::Type::Vec3f) {
            fields.push_back(arrow::field(names[0], arrow::float32()));
            fields.push_back(arrow::field(names[1], arrow::float32()));
            fields.push_back(arrow::field(names[2], arrow::float32()));
            cols.push_back(scA[ci]);
            cols.push_back(scAy[ci]);
            cols.push_back(scAz[ci]);
        } else {
            fields.push_back(arrow::field(names[0], arrowTypeFor(s.cellType)));
            cols.push_back(scA[ci]);
        }
    }

    auto kv = arrow::KeyValueMetadata::Make(
        {pmsf::kPMSFFormatVersionMetadataKey, pmsf::kPMSFSideChannelsMetadataKey, pmsf::kPMSFSymbolTableMetadataKey,
         pmsf::kPMSFSymbolMappingMetadataKey},
        {"3", serializeSpecs(sideChannels_), pmsfSymbolTableMode_ ? "1" : "0",
         pmsfSymbolMappingMode_ ? "1" : "0"});
    auto schema = std::make_shared<arrow::Schema>(fields, kv);
    auto table = arrow::Table::Make(schema, cols);

    // 写 parquet
    auto outFile = arrow::io::FileOutputStream::Open(path);
    checkStatus(outFile.status(), "open side-channel out");
    auto props = parquet::WriterProperties::Builder()
                     .compression(parquet::Compression::ZSTD)
                     ->compression_level(3)
                     ->build();
    auto arrowProps = parquet::ArrowWriterProperties::Builder().store_schema()->build();
    checkStatus(parquet::arrow::WriteTable(*table, arrow::default_memory_pool(),
                                           *outFile,
                                           /*chunk_size=*/64 * 1024,
                                           props, arrowProps),
                "side-channel WriteTable");
    std::cout << "[UnifiedSerialize] side-channel table: " << path
              << " rows=" << table->num_rows() << std::endl;
}

// ============================================================================
// 第 6 张表：blob
// ============================================================================
void UnifiedMultiTableSerializeOperator::writeBlobTableForBase(const std::string& baseName) {
    if (blobs_.empty()) return;
    std::vector<pmsf::PaperNamedBlob> paperBlobs;
    paperBlobs.reserve(blobs_.size());
    for (auto& nb : blobs_) {
        pmsf::PaperNamedBlob pb;
        pb.name = std::move(nb.name);
        pb.kind = std::move(nb.kind);
        pb.bytes = std::move(nb.bytes);
        paperBlobs.push_back(std::move(pb));
    }
    std::vector<pmsf::PaperNamedBlob> namedBlobs =
        pmsf::normalizePMSFPayloadBlobs(paperBlobs);
    const std::string path = pmsf::paperPayloadBlobOutputPathForBase(baseName);
    if (!pmsf::writePaperNamedBlobsForBase(baseName, namedBlobs)) {
        throw std::runtime_error("Unified blob WriteTable failed");
    }
    std::cout << "[UnifiedSerialize] blob table: " << path
              << " rows=" << namedBlobs.size() << std::endl;
}

// ============================================================================
// 阶段 D：紧凑 binary —— `<base>_v3_compact.bin`
//
// V3 compact 现在复用 V2 compact container（V2CP）承载论文对象元数据 O
// 和 base mesh M_0^p，避免 V2/V3 为同一逻辑对象维护两套 base layout。
// 旧 V3CP reader fallback 仍保留在 deserialize 端以兼容历史产物。
// ============================================================================
bool UnifiedMultiTableSerializeOperator::writeCompactBin(const std::string& path) {
    std::vector<std::uint8_t> compactBin;
    if (!buildV2CompactBinPayload(compactBin)) {
        std::cerr << "[UnifiedSerialize] compact payload build failed: "
                  << path << std::endl;
        return false;
    }

    std::ofstream ofs(path, std::ios::binary);
    if (!ofs) {
        std::cerr << "[UnifiedSerialize] compact 打开失败: " << path << std::endl;
        return false;
    }
    ofs.write(reinterpret_cast<const char*>(compactBin.data()),
              static_cast<std::streamsize>(compactBin.size()));
    ofs.close();

    std::cout << "[UnifiedSerialize] compact bin: " << path
              << " bytes=" << compactBin.size() << " B"
              << " container=V2CP"
              << " groups=" << groupBaseMeshData_.size()
              << std::endl;
    return true;
}
