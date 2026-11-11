/*
 * @file UnifiedMultiTableDeserializeOperator.cpp
 */
#include "UnifiedMultiTableDeserializeOperator.h"
#include "compress/UnifiedSchema.h"
#include "core/PMSFSideChannel.h"
#include "storage/PMSFCompactBinEnvelope.h"
#include "storage/PMSFSymbolPayloadBlobReader.h"
#include "storage/PMSFSymbolStorageAdapter.h"
#include "storage/PMSFParquetPayloadAdapter.h"

#include <arrow/api.h>
#include <arrow/io/api.h>
#include <arrow/util/key_value_metadata.h>
#include <parquet/arrow/reader.h>
#include <zlib.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include "scheme/StorageSchemeRegistry.h"
#include <string>
#include <variant>
#include <vector>

// ----------------------------------------------------------------------------
// path 变换：`<base>.parquet` → `<base>_v3.parquet`（再喂给 V2::init）
// ----------------------------------------------------------------------------
std::string UnifiedMultiTableDeserializeOperator::toV3BaseName(const std::string& path) {
    return pmsf::unifiedStemFromArtifactPath(path);
}

// ----------------------------------------------------------------------------
// init
// ----------------------------------------------------------------------------
void UnifiedMultiTableDeserializeOperator::init(std::string path) {
    clearPMSFSymbolContext();
    side_ = pmsf::RecoveredSideChannels{};
    blobs_.clear();
    blobKinds_.clear();
    paperPayloadTable_.clear();
    hasPMSFPayloadTable_ = false;

    const std::string baseName = toV3BaseName(path);
    readBlobTableForBase(baseName);

    // 阶段 D：若 `<base>_v3_compact.bin` 存在，则从它读 metadata + base_mesh，
    // 跳过 `_v3_metadata.parquet` + `_v3_base_mesh.parquet`。否则 fallback 到
    // V2 两张 parquet 表（兼容旧产物 + compact 关闭时产物）。
    const std::string compactPath = pmsf::paperCompactBinPath(baseName);
    const bool hasCompact = std::filesystem::exists(compactPath);
    bool compactOk = false;
    if (hasCompact) {
        compactOk = readCompactBin(compactPath);
        if (!compactOk) {
            // compact 文件存在但解析失败 —— 回退到 parquet（需要 parquet 还在；
            // 正常情况下启用 compact 时它们会被删除，此时会解析到空表并使用
            // 默认值，导致 round=0 等 — 但这是 compact 文件损坏的容错路径）。
            std::cerr << "[UnifiedDeserialize] compact bin parse failed, "
                         "falling back to parquet metadata/base_mesh"
                      << std::endl;
        }
    }
    if (!compactOk) {
        const auto compactSpec = pmsf::parquetCompactBasePayloadSpec();
        const std::vector<std::uint8_t>* compactPayload =
            findPMSFPayload(compactSpec);
        if (!compactPayload) {
            compactPayload = findPMSFPayloadByRole(
                pmsf::PMSFPayloadRole::CompactBase,
                compactSpec.objectId,
                compactSpec.partitionId,
                compactSpec.lodId);
        }
        if (compactPayload) {
            compactOk = readV2CompactBinPayload(
                *compactPayload, compactSpec.payloadName);
            if (!compactOk) {
                std::cerr << "[UnifiedDeserialize] unified compact base payload "
                          << "parse failed, falling back to parquet metadata/base_mesh"
                          << std::endl;
            }
        }
    }
    if (!compactOk) {
        readMetadataTable(pmsf::paperMetadataTablePath(baseName));
        readBaseMeshTable(pmsf::paperBaseMeshTablePath(baseName));
    }

    // symbols / boundary 与 V2 路径相同
    readSymbolsTable(pmsf::paperSymbolsTablePath(baseName));
    readBoundaryTable(pmsf::paperBoundaryTablePath(baseName));

    // V3 独有两张可选表
    readSideChannelTable(pmsf::paperSideChannelTablePath(baseName));

    if (UnifiedMultiTableDeserializeOperator::hasPMSFSymbolTable()) {
        DeserializeOperator::setPMSFSymbolTable(side_.paperTable);
        if (side_.hasSymbolMapping) {
            importPMSFSymbolTableAsSymbolData(side_.paperTable);
        }
    }
    if (UnifiedMultiTableDeserializeOperator::hasPMSFSymbolPayloadTable()) {
        DeserializeOperator::setPMSFSymbolPayloadTable(paperSymbolPayloadTable());
    }
}

// ----------------------------------------------------------------------------
// 阶段 D：compact bin 读取
// ----------------------------------------------------------------------------
namespace {

inline std::uint32_t rdU32(const std::uint8_t*& p) {
    std::uint32_t v = (std::uint32_t)p[0] | ((std::uint32_t)p[1] << 8)
                    | ((std::uint32_t)p[2] << 16) | ((std::uint32_t)p[3] << 24);
    p += 4;
    return v;
}
inline std::int32_t rdI32(const std::uint8_t*& p) {
    return static_cast<std::int32_t>(rdU32(p));
}
inline std::uint16_t rdU16(const std::uint8_t*& p) {
    std::uint16_t v = (std::uint16_t)p[0] | ((std::uint16_t)p[1] << 8);
    p += 2;
    return v;
}
inline float rdF32(const std::uint8_t*& p) {
    std::uint32_t u = rdU32(p);
    float f; std::memcpy(&f, &u, 4);
    return f;
}
inline std::uint8_t rdU8(const std::uint8_t*& p) {
    return *p++;
}

}  // namespace

bool UnifiedMultiTableDeserializeOperator::readCompactBin(const std::string& path) {
    pmsf::PMSFCompactBinEnvelope envelope;
    if (!pmsf::readPMSFCompactBinEnvelopeFile(
            path, "[UnifiedDeserialize]", envelope)) {
        return false;
    }
    if (readV2CompactPayloadBytes(
            envelope.payload,
            envelope.rawLen,
            envelope.compLen,
            path,
            true)) {
        return true;
    }

    const auto rawLen = envelope.rawLen;
    const auto compLen = envelope.compLen;
    const auto& payload = envelope.payload;

    const std::uint8_t* p    = payload.data();
    const std::uint8_t* pEnd = payload.data() + rawLen;

    // legacy magic "V3CP"
    if (rawLen < 8 || p[0] != 'V' || p[1] != '3' || p[2] != 'C' || p[3] != 'P') {
        std::cerr << "[UnifiedDeserialize] compact bin magic mismatch" << std::endl;
        return false;
    }
    p += 4;
    const auto compactHeader =
        pmsf::readPaperCompactPayloadHeader(p, /*hasSecondFlagByte=*/false);
    const bool hasQuant = compactHeader.hasQuant;
    const bool hasSeg = compactHeader.hasSeg;
    codecVersion_ = compactHeader.codecVersion;
    blobLayout_ = compactHeader.blobLayout;
    origTotalVertices_ = compactHeader.origTotalVertices;
    (void)compactHeader.origTotalFaces;

    // Phase S0（R13）：同 impl 侧，按产物自描述再实例化存储方案；失败即报错返回。
    {
        std::string why;
        if (!::pmsf::activateStorageSchemeFromArtifact(
                compactHeader.storageSchemeName, compactHeader.storageSchemeVersion, &why)) {
            std::cerr << "[Unified] 存储方案无法激活：" << why << std::endl;
            return false;
        }
    }

    header_.setRound(compactHeader.round);
    header_.setGroupNumber(compactHeader.groupNumber);
    header_.setEnableCompress((compactHeader.flagBits & 0x01) != 0);
    header_.setEnableQuantization((compactHeader.flagBits & 0x02) != 0);
    header_.setEnableSegmentation((compactHeader.flagBits & 0x04) != 0);
    header_.setEnableAdjustment((compactHeader.flagBits & 0x08) != 0);
    enablePrediction_ = (compactHeader.flagBits & 0x10) != 0;
    header_.setEnablePrediction(enablePrediction_);

    if (hasQuant) {
        quantBits_ = compactHeader.quantBits;
        bboxMinX_ = compactHeader.bboxMinX;
        bboxMinY_ = compactHeader.bboxMinY;
        bboxMinZ_ = compactHeader.bboxMinZ;
        quantStep_ = compactHeader.quantStep;
    } else {
        quantBits_ = 0; bboxMinX_ = bboxMinY_ = bboxMinZ_ = quantStep_ = 0;
    }

    // --- base mesh groups ---
    const int32_t nGroups = rdI32(p);
    baseMeshData_.clear();
    baseMeshData_.reserve(nGroups);
    for (int g = 0; g < nGroups; ++g) {
        GroupBaseMeshData data{};
        data.groupId     = rdI32(p);
        data.numVertices = rdI32(p);  // V2 约定：非 seed 顶点数（不含 2 个 seed）
        data.numFaces    = rdI32(p);
        if (hasQuant) {
            data.seedV0QX = rdI32(p); data.seedV0QY = rdI32(p); data.seedV0QZ = rdI32(p);
            data.seedV1QX = rdI32(p); data.seedV1QY = rdI32(p); data.seedV1QZ = rdI32(p);
            // seed 浮点：由量化反算（为了下游一致性）
            data.seedV0X = bboxMinX_ + (data.seedV0QX + 0.5f) * quantStep_;
            data.seedV0Y = bboxMinY_ + (data.seedV0QY + 0.5f) * quantStep_;
            data.seedV0Z = bboxMinZ_ + (data.seedV0QZ + 0.5f) * quantStep_;
            data.seedV1X = bboxMinX_ + (data.seedV1QX + 0.5f) * quantStep_;
            data.seedV1Y = bboxMinY_ + (data.seedV1QY + 0.5f) * quantStep_;
            data.seedV1Z = bboxMinZ_ + (data.seedV1QZ + 0.5f) * quantStep_;
            // vertex quant 数组（非 seed 顶点，count = numVertices）
            const int vlen = data.numVertices;
            if (vlen > 0) {
                data.vertexQX.resize(vlen);
                data.vertexQY.resize(vlen);
                data.vertexQZ.resize(vlen);
                for (int i = 0; i < vlen; ++i) {
                    data.vertexQX[i] = rdI32(p);
                    data.vertexQY[i] = rdI32(p);
                    data.vertexQZ[i] = rdI32(p);
                }
                // 反量化得到浮点坐标（与 V2 readBaseMeshTable 一致）
                data.vertexX.resize(vlen);
                data.vertexY.resize(vlen);
                data.vertexZ.resize(vlen);
                for (int i = 0; i < vlen; ++i) {
                    data.vertexX[i] = bboxMinX_ + (data.vertexQX[i] + 0.5f) * quantStep_;
                    data.vertexY[i] = bboxMinY_ + (data.vertexQY[i] + 0.5f) * quantStep_;
                    data.vertexZ[i] = bboxMinZ_ + (data.vertexQZ[i] + 0.5f) * quantStep_;
                }
            }
        } else {
            data.seedV0X = rdF32(p); data.seedV0Y = rdF32(p); data.seedV0Z = rdF32(p);
            data.seedV1X = rdF32(p); data.seedV1Y = rdF32(p); data.seedV1Z = rdF32(p);
            const int vlen = data.numVertices;
            if (vlen > 0) {
                data.vertexX.resize(vlen);
                data.vertexY.resize(vlen);
                data.vertexZ.resize(vlen);
                for (int i = 0; i < vlen; ++i) {
                    data.vertexX[i] = rdF32(p);
                    data.vertexY[i] = rdF32(p);
                    data.vertexZ[i] = rdF32(p);
                }
            }
        }
        const int32_t nDegrees = rdI32(p);
        data.faceDegrees.resize(nDegrees);
        for (int i = 0; i < nDegrees; ++i) data.faceDegrees[i] = rdI32(p);
        const int32_t nFaceVerts = rdI32(p);
        data.faceVertices.resize(nFaceVerts);
        for (int i = 0; i < nFaceVerts; ++i) data.faceVertices[i] = rdI32(p);

        baseMeshData_.push_back(std::move(data));
    }

    if (p > pEnd) {
        std::cerr << "[UnifiedDeserialize] compact bin 读越界" << std::endl;
        return false;
    }
    std::cout << "[UnifiedDeserialize] compact bin: rawLen=" << rawLen
              << " compLen=" << compLen
              << " round=" << compactHeader.round
              << " groups=" << nGroups
              << std::endl;
    return true;
}

// ----------------------------------------------------------------------------
// helpers
// ----------------------------------------------------------------------------
namespace {

inline bool fileExists(const std::string& p) {
    std::error_code ec;
    return std::filesystem::exists(p, ec);
}

std::vector<pmsf::SideChannelSpec> parseSpecs(const std::string& s) {
    std::vector<pmsf::SideChannelSpec> out;
    std::size_t i = 0;
    while (i < s.size()) {
        auto sep1 = s.find(':', i);
        auto sep2 = s.find(':', sep1 + 1);
        auto sep3 = s.find(';', sep2 + 1);
        if (sep1 == std::string::npos || sep2 == std::string::npos || sep3 == std::string::npos)
            break;
        pmsf::SideChannelSpec spec;
        spec.name = s.substr(i, sep1 - i);
        const std::string type_str = s.substr(sep1 + 1, sep2 - sep1 - 1);
        const std::string shape_str = s.substr(sep2 + 1, sep3 - sep2 - 1);
        if      (type_str == "bool")          spec.cellType = pmsf::SideChannelSpec::Type::Bool;
        else if (type_str == "int8")          spec.cellType = pmsf::SideChannelSpec::Type::Int8;
        else if (type_str == "int16")         spec.cellType = pmsf::SideChannelSpec::Type::Int16;
        else if (type_str == "int32")         spec.cellType = pmsf::SideChannelSpec::Type::Int32;
        else if (type_str == "uint16")        spec.cellType = pmsf::SideChannelSpec::Type::UInt16;
        else if (type_str == "float32")       spec.cellType = pmsf::SideChannelSpec::Type::Float;
        else if (type_str == "float32_vec3")  spec.cellType = pmsf::SideChannelSpec::Type::Vec3f;
        else if (type_str == "binary")        spec.cellType = pmsf::SideChannelSpec::Type::Bytes;
        spec.shape = (shape_str == "Vec3") ? pmsf::SideChannelSpec::Shape::Vec3
                                           : pmsf::SideChannelSpec::Shape::Scalar;
        out.push_back(std::move(spec));
        i = sep3 + 1;
    }
    return out;
}

}  // namespace

void UnifiedMultiTableDeserializeOperator::readSideChannelTable(const std::string& path) {
    if (!fileExists(path)) return;
    auto pool = arrow::default_memory_pool();
    auto maybe_input = arrow::io::ReadableFile::Open(path, pool);
    if (!maybe_input.ok()) return;
    std::shared_ptr<arrow::io::ReadableFile> input = *maybe_input;

    std::unique_ptr<parquet::arrow::FileReader> reader;
    auto st = parquet::arrow::OpenFile(input, pool).Value(&reader);
    if (!st.ok()) return;

    std::shared_ptr<arrow::Table> table;
    st = reader->ReadTable(&table);
    if (!st.ok() || !table) return;

    bool hasExplicitMappingMetadata = false;
    // 从 schema KV 元数据取出 spec 描述字符串。
    if (table->schema()->metadata()) {
        auto r = table->schema()->metadata()->Get(pmsf::kPMSFSideChannelsMetadataKey);
        if (r.ok()) side_.specs = parseSpecs(*r);
        auto paper = table->schema()->metadata()->Get(pmsf::kPMSFSymbolTableMetadataKey);
        side_.isPMSFSymbolTable = paper.ok() && *paper == "1";
        auto mapping = table->schema()->metadata()->Get(pmsf::kPMSFSymbolMappingMetadataKey);
        hasExplicitMappingMetadata = mapping.ok();
        side_.hasSymbolMapping = mapping.ok() && *mapping == "1";
    }
    if (side_.specs.empty()) return;

    auto combined = table->CombineChunks(pool);
    if (!combined.ok()) return;
    auto ct = *combined;
    auto getCol = [&](const std::string& name) -> std::shared_ptr<arrow::Array> {
        auto c = ct->GetColumnByName(name);
        if (!c || c->num_chunks() != 1) return nullptr;
        return c->chunk(0);
    };

    auto roundA = std::static_pointer_cast<arrow::Int32Array>(getCol(pmsf::kUnifiedRoundColumn));
    auto posA   = std::static_pointer_cast<arrow::Int32Array>(getCol(pmsf::kUnifiedPositionColumn));
    auto nameA  = std::static_pointer_cast<arrow::StringArray>(getCol(pmsf::kUnifiedChannelNameColumn));
    if (!roundA || !posA) return;

    // 每个 spec 一组列
    struct ColSet {
        std::shared_ptr<arrow::Array> a, ay, az;
        pmsf::SideChannelSpec::Type t;
    };
    std::unordered_map<std::string, ColSet> colsByName;
    for (const auto& s : side_.specs) {
        ColSet cs;
        cs.t = s.cellType;
        const auto names = pmsf::sideChannelColumnNames(s);
        if (s.cellType == pmsf::SideChannelSpec::Type::Vec3f) {
            cs.a  = getCol(names[0]);
            cs.ay = getCol(names[1]);
            cs.az = getCol(names[2]);
        } else {
            cs.a = getCol(names[0]);
        }
        colsByName.emplace(s.name, cs);
    }

    int32_t maxRound = -1;
    for (int64_t i = 0; i < roundA->length(); ++i)
        if (roundA->Value(i) > maxRound) maxRound = roundA->Value(i);

    if (side_.stream.size() < static_cast<std::size_t>(maxRound + 1)) {
        side_.stream.resize(maxRound + 1);
    }

    auto getSideCell = [&](const ColSet& cs, int64_t row) -> pmsf::SideCell {
        switch (cs.t) {
            case pmsf::SideChannelSpec::Type::Bool:
                return static_cast<arrow::BooleanArray*>(cs.a.get())->Value(row);
            case pmsf::SideChannelSpec::Type::Int8:
                return static_cast<arrow::Int8Array*>(cs.a.get())->Value(row);
            case pmsf::SideChannelSpec::Type::Int16:
                return static_cast<arrow::Int16Array*>(cs.a.get())->Value(row);
            case pmsf::SideChannelSpec::Type::Int32:
                return static_cast<arrow::Int32Array*>(cs.a.get())->Value(row);
            case pmsf::SideChannelSpec::Type::UInt16:
                return static_cast<arrow::UInt16Array*>(cs.a.get())->Value(row);
            case pmsf::SideChannelSpec::Type::Float:
                return static_cast<arrow::FloatArray*>(cs.a.get())->Value(row);
            case pmsf::SideChannelSpec::Type::Vec3f: {
                const float x = static_cast<arrow::FloatArray*>(cs.a.get())->Value(row);
                const float y = static_cast<arrow::FloatArray*>(cs.ay.get())->Value(row);
                const float z = static_cast<arrow::FloatArray*>(cs.az.get())->Value(row);
                return MCGAL::Point(x, y, z);
            }
            case pmsf::SideChannelSpec::Type::Bytes: {
                auto ba = static_cast<arrow::BinaryArray*>(cs.a.get());
                int32_t len;
                const uint8_t* pp = ba->GetValue(row, &len);
                return std::vector<std::uint8_t>(pp, pp + len);
            }
        }
        return static_cast<int8_t>(0);
    };

    if (nameA) {
        for (int64_t i = 0; i < roundA->length(); ++i) {
            const int r = roundA->Value(i);
            const std::string name = nameA->GetString(i);
            auto it = colsByName.find(name);
            if (it == colsByName.end() || !it->second.a) continue;
            if (it->second.t == pmsf::SideChannelSpec::Type::Vec3f &&
                (!it->second.ay || !it->second.az)) {
                continue;
            }
            if (it->second.a->IsNull(i)) continue;
            pmsf::appendPaperSideCellSideMap(
                side_.stream[r].side, name, getSideCell(it->second, i));
        }
    } else {
        // Compatibility with UnifiedSerializeOperator single-file tables:
        // one row can carry values for several side channels and has no
        // `channel_name` discriminator.
        for (int64_t i = 0; i < roundA->length(); ++i) {
            const int r = roundA->Value(i);
        for (const auto& spec : side_.specs) {
            auto it = colsByName.find(spec.name);
            if (it == colsByName.end() || !it->second.a) continue;
            if (it->second.t == pmsf::SideChannelSpec::Type::Vec3f &&
                (!it->second.ay || !it->second.az)) {
                continue;
            }
            if (it->second.a->IsNull(i)) continue;
            pmsf::appendPaperSideCellSideMap(
                side_.stream[r].side, spec.name, getSideCell(it->second, i));
        }
        }
    }

    side_.paperTable = pmsf::pmsfSymbolTableFromStorageStream(side_.stream);
    if (side_.isPMSFSymbolTable && !hasExplicitMappingMetadata) {
        side_.hasSymbolMapping =
            pmsf::pmsfSymbolTableHasSymbolMapping(side_.paperTable);
    }

    std::cout << "[UnifiedDeserialize] side-channel: rows=" << roundA->length()
              << " specs=" << side_.specs.size()
              << " paper_records=" << side_.paperTable.size() << std::endl;
}

void UnifiedMultiTableDeserializeOperator::readBlobTableForBase(
    const std::string& baseName) {
    const auto readResult = pmsf::readFirstPaperNamedBlobTableCandidate(baseName);
    const auto& namedBlobs = readResult.blobs;
    if (namedBlobs.empty()) return;
    paperPayloadTable_ = pmsf::paperPayloadTableFromNamedBlobs(namedBlobs);
    if (!paperPayloadTable_.empty()) {
        hasPMSFPayloadTable_ = true;
    }
    for (const auto& blob : namedBlobs) {
        blobs_[blob.name] = blob.bytes;
        blobKinds_[blob.name] = blob.kind;
        pmsf::PMSFSymbolPayloadRecord row;
        if (pmsf::parsePMSFPayloadBlobName(blob.name, row)) {
            hasPMSFPayloadTable_ = true;
            if (row.objectId == 0 && row.partitionId == 0 && row.lodId == 0) {
                blobs_[row.payloadName] = blob.bytes;
                blobKinds_[row.payloadName] = blob.kind;
            }
        }
    }
    std::cout << "[UnifiedDeserialize] blob table: entries=" << blobs_.size() << std::endl;
}

pmsf::PMSFSymbolPayloadTable
UnifiedMultiTableDeserializeOperator::paperSymbolPayloadTable() const {
    if (!paperPayloadTable_.empty()) {
        return paperPayloadTable_;
    }
    std::vector<pmsf::PaperNamedBlob> namedBlobs;
    namedBlobs.reserve(blobs_.size());
    for (const auto& kv : blobs_) {
        pmsf::PaperNamedBlob blob;
        blob.name = kv.first;
        auto kind = blobKinds_.find(kv.first);
        blob.kind = kind == blobKinds_.end() ? "unknown" : kind->second;
        blob.bytes = kv.second;
        pmsf::appendPaperNamedBlobRecord(namedBlobs, std::move(blob));
    }
    return pmsf::paperPayloadTableFromNamedBlobs(namedBlobs);
}

const std::vector<std::uint8_t>*
UnifiedMultiTableDeserializeOperator::findPMSFPayload(
    std::int32_t objectId,
    std::int32_t partitionId,
    std::int32_t lodId,
    const std::string& payloadName) const {
    return pmsf::findPMSFPayloadOrRawBlobMapBytesByNameOrSafeRole(
        blobs_, blobKinds_, objectId, partitionId, lodId, payloadName);
}

const std::vector<std::uint8_t>*
UnifiedMultiTableDeserializeOperator::findPMSFPayload(
    const pmsf::PMSFSymbolPayloadSpec& spec) const {
    return findPMSFPayload(
        spec.objectId, spec.partitionId, spec.lodId, spec.payloadName);
}

const std::vector<std::uint8_t>*
UnifiedMultiTableDeserializeOperator::findPMSFPayloadByRole(
    pmsf::PMSFPayloadRole role,
    std::int32_t objectId,
    std::int32_t partitionId,
    std::int32_t lodId) const {
    if (hasPMSFPayloadTable_) {
        if (const auto* row = pmsf::findPMSFPayloadByRole(
                paperPayloadTable_, role, objectId, partitionId, lodId)) {
            if (!row->payload.empty()) {
                return &row->payload;
            }
        }
    }
    return pmsf::findPMSFPayloadOrRawBlobMapBytesByRole(
        blobs_, blobKinds_, role, objectId, partitionId, lodId);
}
