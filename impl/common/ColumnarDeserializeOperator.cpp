/*
 * @Author: koi
 * @Description: Parquet V2 多表方案反序列化操作符实现
 */
#include "decompress/ColumnarDeserializeOperator.h"
#include "common/BoundedRingTraversal.h"
#include "common/PaperLayoutAliases.h"
#include "BufferUtils.h"
#include "Conquest/decompress/ConquestSymbolReadOperator.h"   // deserializeCharPointer
#include "Configuration.h"
#include "Global.h"
#include "Halfedge.h"
#include "Vertex.h"
#include "Facet.h"
#include "RD/decompress/RDTagCanonicalReassembler.h"
#include "PCLTTM/decompress/PCLTTMTagCanonicalReassembler.h"
#include "PCATM/decompress/PCATMColorCanonicalReassembler.h"
#include "decompress/PatchColorCanonicalReassembler.h"
#include "PCATM/decompress/PCATMPureCanonicalReassembler.h"
#include "common/strict/PCATMStrictPMSFSymbolReadOperator.h"
#include "PMSFReaderLayoutRegistry.h"
// PHASE60 物品 3：分区（layout-32）产物的后端无关解码镜像（与二进制共用）。
#include "MeshUtils.h"
#include "PCLTTM/decompress/PCLTTMTagSymbolReadOperator.h"
#include "PPMC/decompress/PPMCSymbolReadOperator.h"
#include "SegmentedRangeCodedDecode.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂
#include "common/strict/PCLTTMStrictSymbolReadOperator.h"
#include "compress/EdgeBreakerCodec.h"
#include "compress/CanonicalArrowTableReader.h"
#include "compress/OptimizedSymbolCollectOperator.h"
#include "core/CanonicalSymbolCollector.h"
#include "decompress/OptimizedSymbolReadOperator.h"
#include "scheme/StorageSchemeRegistry.h"
#include "storage/PMSFCompactBinEnvelope.h"
#include "storage/PMSFParquetPayloadAdapter.h"
#include "storage/PMSFSymbolPayloadBlobReader.h"
#include "storage/PMSFSymbolStorageAdapter.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <zlib.h>
#ifdef PMSF_HAS_ZSTD
#include <zstd.h>
#endif

#include <arrow/api.h>
#include <arrow/compute/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/reader.h>

namespace {
bool parquetV2VerboseLogs() {
    static const bool enabled = [] {
        const char* env = std::getenv("PMSF_V2_VERBOSE");
        return env && env[0] != '\0' && std::strcmp(env, "0") != 0 &&
               std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
    }();
    return enabled;
}

bool parquetV2ProfileLogs() {
    static const bool enabled = [] {
        const char* env = std::getenv("PMSF_PROFILE");
        return env && env[0] != '\0' && std::strcmp(env, "0") != 0 &&
               std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
    }();
    return enabled;
}

bool segmentedReverseGroupReconstruct() {
    static const bool enabled = [] {
        const char* env = std::getenv("PMSF_SEGMENTED_RECONSTRUCT_REVERSE_GROUPS");
        return env && env[0] != '\0' && std::strcmp(env, "0") != 0 &&
               std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
    }();
    return enabled;
}

bool segmentedBatchGroupReconstruct() {
    static const bool enabled = [] {
        const char* env = std::getenv("PMSF_SEGMENTED_RECONSTRUCT_BATCH_GROUPS");
        return env && env[0] != '\0' && std::strcmp(env, "0") != 0 &&
               std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
    }();
    return enabled;
}

bool segmentedDefaultSharedTopology() {
    static const bool enabled = [] {
        const char* env = std::getenv("PMSF_SEGMENTED_DEFAULT_SHARED_TOPO");
        return env && env[0] != '\0' && std::strcmp(env, "0") != 0 &&
               std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
    }();
    return enabled;
}

int clampPoolEstimate(long long value, int fallback) {
    if (value <= 0) return fallback;
    if (value > static_cast<long long>(std::numeric_limits<int>::max())) {
        return std::numeric_limits<int>::max();
    }
    return static_cast<int>(value);
}

std::uint32_t readU32LEAt(const std::vector<char>& data, std::size_t off) {
    return static_cast<std::uint32_t>(static_cast<unsigned char>(data[off])) |
           (static_cast<std::uint32_t>(static_cast<unsigned char>(data[off + 1])) << 8) |
           (static_cast<std::uint32_t>(static_cast<unsigned char>(data[off + 2])) << 16) |
           (static_cast<std::uint32_t>(static_cast<unsigned char>(data[off + 3])) << 24);
}

bool unwrapSymbolBlobIfNeeded(std::vector<char>& blob) {
    if (blob.size() < 13 ||
        blob[0] != 'S' || blob[1] != 'B' || blob[2] != 'C' || blob[3] != '1') {
        return true;
    }
    const unsigned char codec = static_cast<unsigned char>(blob[4]);
    const std::uint32_t rawLen = readU32LEAt(blob, 5);
    const std::uint32_t compLen = readU32LEAt(blob, 9);
    if ((codec != 1 && codec != 2) || rawLen == 0 || compLen == 0 ||
        13ull + static_cast<unsigned long long>(compLen) != blob.size()) {
        std::cerr << "[Columnar] invalid symbol blob wrapper" << std::endl;
        return false;
    }
    std::vector<char> raw(static_cast<std::size_t>(rawLen));
    if (codec == 1) {
        uLongf outLen = static_cast<uLongf>(rawLen);
        const int zret = uncompress(reinterpret_cast<Bytef*>(raw.data()), &outLen,
                                    reinterpret_cast<const Bytef*>(blob.data() + 13),
                                    static_cast<uLong>(compLen));
        if (zret != Z_OK || outLen != rawLen) {
            std::cerr << "[Columnar] symbol blob zlib unwrap failed rc=" << zret
                      << " outLen=" << outLen << "/" << rawLen << std::endl;
            return false;
        }
    } else {
#ifdef PMSF_HAS_ZSTD
        const std::size_t outLen =
            ZSTD_decompress(raw.data(), raw.size(), blob.data() + 13, compLen);
        if (ZSTD_isError(outLen) || outLen != rawLen) {
            std::cerr << "[Columnar] symbol blob zstd unwrap failed" << std::endl;
            return false;
        }
#else
        std::cerr << "[Columnar] symbol blob zstd wrapper unsupported by this build"
                  << std::endl;
        return false;
#endif
    }
    if (parquetV2VerboseLogs()) {
        std::cout << "[Columnar] symbols blob wrapper: SBC1:"
                  << (codec == 1 ? "zlib" : "zstd") << " "
                  << blob.size() << " -> " << raw.size() << " bytes" << std::endl;
    }
    blob.swap(raw);
    return true;
}
}  // namespace

// ============================================================================
// Arrow 辅助方法
// ============================================================================

std::vector<float> ColumnarDeserializeOperator::extractFloatList(
    std::shared_ptr<arrow::Array> array, int64_t row) {
    std::vector<float> result;
    if (!array || array->IsNull(row)) return result;
    auto listArray = std::static_pointer_cast<arrow::ListArray>(array);
    auto values = std::static_pointer_cast<arrow::FloatArray>(listArray->values());
    int32_t start = listArray->value_offset(row);
    int32_t end = listArray->value_offset(row + 1);
    result.reserve(end - start);
    for (int32_t i = start; i < end; i++) result.push_back(values->Value(i));
    return result;
}

std::vector<int32_t> ColumnarDeserializeOperator::extractInt32List(
    std::shared_ptr<arrow::Array> array, int64_t row) {
    std::vector<int32_t> result;
    if (!array || array->IsNull(row)) return result;
    auto listArray = std::static_pointer_cast<arrow::ListArray>(array);
    auto values = std::static_pointer_cast<arrow::Int32Array>(listArray->values());
    int32_t start = listArray->value_offset(row);
    int32_t end = listArray->value_offset(row + 1);
    result.reserve(end - start);
    for (int32_t i = start; i < end; i++) result.push_back(values->Value(i));
    return result;
}

std::vector<bool> ColumnarDeserializeOperator::extractBoolList(
    std::shared_ptr<arrow::Array> array, int64_t row) {
    std::vector<bool> result;
    if (!array || array->IsNull(row)) return result;
    auto listArray = std::static_pointer_cast<arrow::ListArray>(array);
    auto values = std::static_pointer_cast<arrow::BooleanArray>(listArray->values());
    int32_t start = listArray->value_offset(row);
    int32_t end = listArray->value_offset(row + 1);
    result.reserve(end - start);
    for (int32_t i = start; i < end; i++) result.push_back(values->Value(i));
    return result;
}

std::vector<int16_t> ColumnarDeserializeOperator::extractInt16List(
    std::shared_ptr<arrow::Array> array, int64_t row) {
    std::vector<int16_t> result;
    if (!array || array->IsNull(row)) return result;
    auto listArray = std::static_pointer_cast<arrow::ListArray>(array);
    auto values = std::static_pointer_cast<arrow::Int16Array>(listArray->values());
    int32_t start = listArray->value_offset(row);
    int32_t end = listArray->value_offset(row + 1);
    result.reserve(end - start);
    for (int32_t i = start; i < end; i++) result.push_back(values->Value(i));
    return result;
}

std::shared_ptr<arrow::Table> ColumnarDeserializeOperator::readParquetFile(const std::string& path) {
    auto result = arrow::io::ReadableFile::Open(path);
    if (!result.ok()) {
        std::cerr << "[Columnar] 无法打开文件: " << path << std::endl;
        return nullptr;
    }
    std::unique_ptr<parquet::arrow::FileReader> reader;
    auto st = parquet::arrow::OpenFile(result.ValueOrDie(), arrow::default_memory_pool()).Value(&reader);
    if (!st.ok()) {
        std::cerr << "[Columnar] 无法解析文件: " << path << std::endl;
        return nullptr;
    }
    std::shared_ptr<arrow::Table> table;
    st = reader->ReadTable(&table);
    if (!st.ok()) {
        std::cerr << "[Columnar] 读取表失败: " << path << std::endl;
        return nullptr;
    }

    auto combined = table->CombineChunks(arrow::default_memory_pool());
    if (combined.ok()) {
        table = combined.ValueOrDie();
    } else {
        std::cerr << "[Columnar] 警告: CombineChunks 失败，继续使用原始 chunked table: "
                  << combined.status().ToString() << std::endl;
    }

    return table;
}

// ============================================================================
// 初始化：从多个 Parquet 文件读取数据
// ============================================================================

void ColumnarDeserializeOperator::init(std::string path) {
    // M2c native experiment: PaperDeserializeOperator sets the symbol table
    // before init(); do not wipe it when PMSF_PAPER_NATIVE=1.
    const bool nativePaperSymbols = []() {
        const char* env = std::getenv("PMSF_PAPER_NATIVE");
        return env && *env && std::strcmp(env, "0") != 0;
    }();
    if (!nativePaperSymbols) {
        clearPMSFSymbolContext();
    }

    currentRound_ = 0;
    origTotalVertices_ = 0;
    origTotalFaces_ = 0;
    baseMeshData_.clear();
    symbolData_.clear();
    boundaryGraph_.reset();

    // 去掉扩展名和已知物理表 role 后缀，恢复逻辑基础名。
    std::string baseName = pmsf::paperStorageStripArtifactStemRole(
        pmsf::paperStorageStripExtension(path));

    if (parquetV2VerboseLogs()) {
        std::cout << "[ColumnarDeserialize] 基础名: " << baseName << std::endl;
    }

    loadPMSFSymbolSidecarIfPresent(path);

    // Tier 2 (2026-05-09)：紧凑模式 — 若 `<base>_compact.bin` 存在则用它替代
    // metadata/base_mesh 两张 parquet 表（解压 zlib 后按固定布局解析）。
    // V3 子类（UnifiedMultiTableDeserializeOperator）覆写此 init() 使用自己的
    // `_v3_compact.bin`，本路径不会触达 V3 数据。
    const std::string compactPath = pmsf::paperCompactBinPath(baseName);
    bool compactOk = false;
    if (std::filesystem::exists(compactPath)) {
        compactOk = readV2CompactBin(compactPath);
        if (!compactOk) {
            // 方案激活失败是**致命的**，不能靠 fallback 掩盖：那会拿着错的方案
            // 继续解（设计文档 §3.3「不静默错解」）。实测过 v99 产物被 fallback
            // 吞掉、进程仍退出 0 —— 所以这里必须先拦。
            if (!::pmsf::storageSchemeFatalError().empty()) {
                // init() 返回 void，无法用返回值表达"致命"；而静默 fallback 会
                // 拿着错的方案继续解。抛出去是最诚实的做法：入口会捕获并打印，
                // 进程以非零码退出。
                throw std::runtime_error("存储方案激活失败，拒绝回落到 parquet：" +
                                         ::pmsf::storageSchemeFatalError());
            }
            std::cerr << "[ColumnarDeserialize] compact bin 解析失败，"
                      << "fallback 到 parquet metadata/base_mesh"
                      << std::endl;
        }
    }
    if (!compactOk) {
        const auto compactSpec = pmsf::parquetCompactBasePayloadSpec();
        const auto* compactPayload = findPMSFSymbolPayloadBytes(
            compactSpec.objectId,
            compactSpec.partitionId,
            compactSpec.lodId,
            compactSpec.payloadName);
        if (!compactPayload) {
            compactPayload = findPMSFSymbolPayloadBytesByRole(
                pmsf::PMSFPayloadRole::CompactBase,
                compactSpec.objectId,
                compactSpec.partitionId,
                compactSpec.lodId);
        }
        if (compactPayload) {
            compactOk = readV2CompactBinPayload(
                *compactPayload, compactSpec.payloadName);
            if (!compactOk) {
                std::cerr << "[ColumnarDeserialize] unified compact base payload "
                          << "解析失败，fallback 到 parquet metadata/base_mesh"
                          << std::endl;
            }
        }
    }
    if (!compactOk) {
        // 1. 读取 metadata
        const std::string metadataPath = pmsf::paperMetadataTablePath(baseName);
        if (std::filesystem::exists(metadataPath)) {
            readMetadataTable(metadataPath);
        } else if (!readEmbeddedMetadataFromPartitionedBase(baseName)) {
            readMetadataTable(metadataPath);
        }

        // 2. 读取 base mesh。新分区紧凑格式优先使用
        // `<base>_vertices.parquet` + `<base>_partitions.parquet`；
        // 不存在时回退旧 `<base>_base_mesh.parquet`。
        const bool partitionedBaseOk = readPartitionedBaseMeshTables(baseName);
        if (!partitionedBaseOk) {
            readBaseMeshTable(pmsf::paperBaseMeshTablePath(baseName));
        }
    }

    // 3. 读取 symbols（M2c native 模式由 paper adapter 直接提供，跳过 legacy 表）
    if (!nativePaperSymbols || !hasPMSFSymbolTable()) {
        readSymbolsTable(pmsf::paperSymbolsTablePath(baseName));
    }

    // 4. 读取 boundary（可选）
    readBoundaryTable(pmsf::paperBoundaryTablePath(baseName));

    if (const auto* paper = DeserializeOperator::pmsfSymbolTable()) {
        if (pmsf::pmsfSymbolTableHasSymbolMapping(*paper)) {
            importPMSFSymbolTableAsSymbolData(*paper);
        }
    }
}

void ColumnarDeserializeOperator::bindPPMCReaderMesh() {
    if (auto* ppmcReader = dynamic_cast<PPMCSymbolReadOperator*>(symbolReadOperator_.get())) {
        ppmcReader->setMesh(mesh_);
    }
}

// ============================================================================
// 读取 metadata 表
// ============================================================================


void ColumnarDeserializeOperator::readMetadataTable(const std::string& path) {
    auto table = readParquetFile(path);
    if (!table || table->num_rows() == 0) {
        std::cerr << "[Columnar] metadata 表为空或不存在" << std::endl;
        return;
    }

    auto getInt32 = [&](const std::string& name) -> int32_t {
        auto col = table->GetColumnByName(name);
        if (!col || col->null_count() == col->length()) return 0;
        auto chunk = std::static_pointer_cast<arrow::Int32Array>(col->chunk(0));
        return chunk->IsNull(0) ? 0 : chunk->Value(0);
    };
    auto getBool = [&](const std::string& name) -> bool {
        auto col = table->GetColumnByName(name);
        if (!col || col->null_count() == col->length()) return false;
        auto chunk = std::static_pointer_cast<arrow::BooleanArray>(col->chunk(0));
        return chunk->IsNull(0) ? false : chunk->Value(0);
    };
    auto getFloat = [&](const std::string& name) -> float {
        auto col = table->GetColumnByName(name);
        if (!col || col->null_count() == col->length()) return 0.0f;
        auto chunk = std::static_pointer_cast<arrow::FloatArray>(col->chunk(0));
        return chunk->IsNull(0) ? 0.0f : chunk->Value(0);
    };

    header_.setRound(getInt32(pmsf::kParquetNumRoundsColumn));
    header_.setGroupNumber(getInt32(pmsf::kParquetGroupNumberColumn));
    header_.setEnableCompress(getBool(pmsf::kParquetEnableCompressColumn));
    header_.setEnableQuantization(getBool(pmsf::kParquetEnableQuantizationColumn));
    header_.setEnableSegmentation(getBool(pmsf::kParquetEnableSegmentationColumn));
    header_.setEnableAdjustment(getBool(pmsf::kParquetEnableAdjustmentColumn));

    // P18.2 PHASE59 物品 3：分区产物守卫**已删**（附 18 落点三 1 步）。
    //
    // 该守卫原先在此按产物自带的 `enableSegmentation` 位拒绝（编码端的分段 blob
    // 布局 `[groupCount][seedA][seedB][groupStart][baseFacetGroupRLE][payload]`
    // 与默认布局不同，下游按默认布局读会越界）。镜像已在
    // `deserializeBaseMesh` 的 `paperBlobLayoutIsSegmentedRangeCodedPred` 分派里
    // 实现，故本守卫连同它的 config 侧孪生一起退役。
    //
    // 对照记录：删守卫前后各跑一次「不传 `segments` 去解分区产物」——
    //   删前 rc=1（本守卫的 throw）；
    //   删后（镜像未齐时）rc=1 `cannot create std::vector larger than max_size()`
    //   —— 同样是在分段 blob 上按默认布局越界，只是以 bad_alloc 而非 SIGSEGV 表现。

    // V2 新增：codec_version 与 enable_prediction（列不存在 → 视为 1 / false，向后兼容）
    {
        auto col = table->GetColumnByName(pmsf::kParquetCodecVersionColumn);
        codecVersion_ = 1;
        if (col && col->num_chunks() > 0) {
            auto chunk = std::static_pointer_cast<arrow::Int32Array>(col->chunk(0));
            if (!chunk->IsNull(0) && chunk->Value(0) > 0) {
                codecVersion_ = chunk->Value(0);
            }
        }
    }
    {
        auto col = table->GetColumnByName(pmsf::kParquetEnablePredictionColumn);
        enablePrediction_ = false;
        if (col && col->num_chunks() > 0) {
            auto chunk = std::static_pointer_cast<arrow::BooleanArray>(col->chunk(0));
            if (!chunk->IsNull(0)) enablePrediction_ = chunk->Value(0);
        }
    }
    {
        auto col = table->GetColumnByName(pmsf::kParquetBlobLayoutColumn);
        blobLayout_ = 0;
        if (col && col->num_chunks() > 0) {
            auto chunk = std::static_pointer_cast<arrow::Int32Array>(col->chunk(0));
            if (!chunk->IsNull(0)) blobLayout_ = chunk->Value(0);
        }
    }
    {
        auto col = table->GetColumnByName(pmsf::kParquetCanonicalSourceLayoutColumn);
        canonicalSourceLayout_ = 0;
        if (col && col->num_chunks() > 0) {
            auto chunk = std::static_pointer_cast<arrow::Int32Array>(col->chunk(0));
            if (!chunk->IsNull(0)) canonicalSourceLayout_ = chunk->Value(0);
        }
    }
    // 2026-05-04 PPMC LetterA 对齐：读取 total_vertices（压缩前 mesh 顶点数），
    //   用于解压端 PPMC "small-mesh triangle-balance prediction disable" 决策。
    {
        auto col = table->GetColumnByName(pmsf::kParquetTotalVerticesColumn);
        origTotalVertices_ = 0;
        if (col && col->num_chunks() > 0) {
            auto chunk = std::static_pointer_cast<arrow::Int32Array>(col->chunk(0));
            if (!chunk->IsNull(0)) origTotalVertices_ = chunk->Value(0);
        }
    }
    {
        auto col = table->GetColumnByName(pmsf::kParquetTotalFacesColumn);
        origTotalFaces_ = 0;
        if (col && col->num_chunks() > 0) {
            auto chunk = std::static_pointer_cast<arrow::Int32Array>(col->chunk(0));
            if (!chunk->IsNull(0)) origTotalFaces_ = chunk->Value(0);
        }
    }
    // 同步到 header_，供 downstream（OptimizedBasicDeserializeOperator 等等价逻辑）使用
    header_.setEnablePrediction(enablePrediction_);

    // enable_triangulation 默认 true（老文件 → header.enableTriangulation=true
    //   → OptimizedSymbolReadOperator 走历史 first-pop 路径，byte-identical）。
    {
        auto col = table->GetColumnByName(pmsf::kParquetEnableTriangulationColumn);
        bool enableTriangulation = true;
        if (col && col->num_chunks() > 0) {
            auto chunk = std::static_pointer_cast<arrow::BooleanArray>(col->chunk(0));
            if (!chunk->IsNull(0)) enableTriangulation = chunk->Value(0);
        }
        header_.setEnableTriangulation(enableTriangulation);
    }

    // 3 个 PPMC 风格预测开关默认 false（老文件 → 默认行为，byte-identical）。
    auto readBoolColumn = [&](const char* colName, bool fallback) -> bool {
        auto col = table->GetColumnByName(colName);
        if (!col || col->num_chunks() == 0) return fallback;
        auto chunk = std::static_pointer_cast<arrow::BooleanArray>(col->chunk(0));
        if (chunk->IsNull(0)) return fallback;
        return chunk->Value(0);
    };
    header_.setEnableConnectivityPredictionFaces(
        readBoolColumn(pmsf::kParquetEnableConnPredFacesColumn, false));
    header_.setEnableConnectivityPredictionEdges(
        readBoolColumn(pmsf::kParquetEnableConnPredEdgesColumn, false));
    header_.setEnableTriangleMeshBalance(
        readBoolColumn(pmsf::kParquetEnableTriBalanceColumn, false));

    quantBits_ = getInt32(pmsf::kParquetQuantBitsColumn);
    bboxMinX_ = getFloat(pmsf::kParquetBBoxMinXColumn);
    bboxMinY_ = getFloat(pmsf::kParquetBBoxMinYColumn);
    bboxMinZ_ = getFloat(pmsf::kParquetBBoxMinZColumn);
    quantStep_ = getFloat(pmsf::kParquetQuantStepColumn);

    if (parquetV2VerboseLogs()) {
        std::cout << "[Columnar] metadata: rounds=" << header_.getRound()
                  << " groups=" << header_.getGroupNumber()
                  << " quant=" << header_.isEnableQuantization()
                  << " seg=" << header_.isEnableSegmentation() << std::endl;
    }
}

bool ColumnarDeserializeOperator::readEmbeddedMetadataFromPartitionedBase(
    const std::string& baseName) {
    const std::string path = pmsf::paperPartitionedBaseTablePath(baseName);
    if (!std::filesystem::exists(path)) return false;
    auto table = readParquetFile(path);
    if (!table || table->num_rows() == 0) return false;
    auto col = table->GetColumnByName(pmsf::kParquetMetadataBlobColumn);
    if (!col || col->num_chunks() == 0) return false;
    auto arr = std::static_pointer_cast<arrow::BinaryArray>(col->chunk(0));
    if (arr->length() == 0 || arr->IsNull(0)) return false;
    int32_t len = 0;
    const std::uint8_t* data = arr->GetValue(0, &len);
    if (!data || len < 4 || data[0] != 'P' || data[1] != 'M' ||
        data[2] != 'D' || data[3] != '1') {
        return false;
    }
    const std::uint8_t* p = data + 4;
    const std::uint8_t* end = data + len;
    auto readI32 = [&]() -> std::int32_t {
        if (p + 4 > end) throw std::runtime_error("metadata blob truncated");
        std::uint32_t u = static_cast<std::uint32_t>(p[0]) |
                          (static_cast<std::uint32_t>(p[1]) << 8) |
                          (static_cast<std::uint32_t>(p[2]) << 16) |
                          (static_cast<std::uint32_t>(p[3]) << 24);
        p += 4;
        return static_cast<std::int32_t>(u);
    };
    auto readU8 = [&]() -> std::uint8_t {
        if (p >= end) throw std::runtime_error("metadata blob truncated");
        return *p++;
    };
    auto readF32 = [&]() -> float {
        const std::uint32_t u = static_cast<std::uint32_t>(readI32());
        float f = 0.0f;
        std::memcpy(&f, &u, sizeof(float));
        return f;
    };
    try {
        header_.setRound(readI32());
        header_.setGroupNumber(readI32());
        const std::uint8_t flags = readU8();
        const bool triBalance = readU8() != 0;
        codecVersion_ = readI32();
        blobLayout_ = readI32();
        // Newer canonical-wrapped files append canonical_source_layout right after
        // blob_layout. Old files omit it. Remaining old payload after blob_layout is
        // totalVertices + totalFaces + quantBits + 3*float + quantStep = 7*4 bytes.
        // New payload has an extra int32, so >= 8*4 bytes.
        const std::size_t remainingAfterLayout = static_cast<std::size_t>(end - p);
        if (remainingAfterLayout >= 8 * sizeof(std::int32_t)) {
            canonicalSourceLayout_ = readI32();
        } else {
            canonicalSourceLayout_ = 0;
        }
        origTotalVertices_ = readI32();
        origTotalFaces_ = readI32();
        quantBits_ = readI32();
        bboxMinX_ = readF32();
        bboxMinY_ = readF32();
        bboxMinZ_ = readF32();
        quantStep_ = readF32();

        header_.setEnableCompress((flags & (1u << 0)) != 0);
        header_.setEnableQuantization((flags & (1u << 1)) != 0);
        header_.setEnableSegmentation((flags & (1u << 2)) != 0);
        header_.setEnableAdjustment((flags & (1u << 3)) != 0);
        enablePrediction_ = (flags & (1u << 4)) != 0;
        header_.setEnablePrediction(enablePrediction_);
        header_.setEnableTriangulation((flags & (1u << 5)) != 0);
        header_.setEnableConnectivityPredictionFaces((flags & (1u << 6)) != 0);
        header_.setEnableConnectivityPredictionEdges((flags & (1u << 7)) != 0);
        header_.setEnableTriangleMeshBalance(triBalance);
    } catch (const std::exception& e) {
        std::cerr << "[Columnar] embedded metadata decode failed: "
                  << e.what() << std::endl;
        return false;
    }
    if (parquetV2VerboseLogs()) {
        std::cout << "[Columnar] embedded metadata: rounds=" << header_.getRound()
                  << " groups=" << header_.getGroupNumber()
                  << " layout=" << blobLayout_ << std::endl;
    }
    return true;
}

// ============================================================================
// 读取 base_mesh 表
// ============================================================================

void ColumnarDeserializeOperator::readBaseMeshTable(const std::string& path) {
    auto table = readParquetFile(path);
    if (!table && codecVersion_ >= 2) {
        std::string sourcePath;
        auto rcBlobString = pmsf::findPMSFPayloadOrRawNamedBlobStringInCandidates(
            path, 0, 0, 0, "pmsf_symbols_rc_blob", &sourcePath);
        if (rcBlobString.empty()) {
            rcBlobString = pmsf::findPMSFPayloadOrRawNamedBlobStringByRoleInCandidates(
                path,
                pmsf::PMSFPayloadRole::SymbolStreamBlob,
                0,
                0,
                0,
                &sourcePath);
        }
        rcBlob_.assign(rcBlobString.begin(), rcBlobString.end());
        if (!rcBlob_.empty()) {
            if (!unwrapSymbolBlobIfNeeded(rcBlob_)) {
                rcBlob_.clear();
                return;
            }
            rcBlobOffset_ = 0;
            rcBlobInitialized_ = false;
            if (parquetV2VerboseLogs()) {
                std::cout << "[Columnar] unified symbols rc_blob fallback: "
                          << rcBlob_.size() << " bytes from " << sourcePath << std::endl;
            }
            return;
        }
    }
    if (!table) return;

    baseMeshData_.clear();
    sharedBaseMeshCoordinates_ = false;
    sharedVertexX_.clear();
    sharedVertexY_.clear();
    sharedVertexZ_.clear();
    sharedVertexQX_.clear();
    sharedVertexQY_.clear();
    sharedVertexQZ_.clear();
    auto groupIdCol = table->GetColumnByName(pmsf::kParquetGroupIdColumn);
    if (!groupIdCol || groupIdCol->num_chunks() == 0) {
        std::cerr << "[Columnar] base_mesh 缺少 group_id 列" << std::endl;
        return;
    }
    auto seedV0CoordCol = table->GetColumnByName(pmsf::kParquetSeedV0CoordIdColumn);
    auto seedV1CoordCol = table->GetColumnByName(pmsf::kParquetSeedV1CoordIdColumn);
    auto vertexCoordIdsCol = table->GetColumnByName(pmsf::kParquetVertexCoordIdsColumn);
    auto sharedQXDetectCol = table->GetColumnByName(pmsf::kParquetSharedVertexQXColumn);
    auto sharedXDetectCol = table->GetColumnByName(pmsf::kParquetSharedVertexXColumn);
    sharedBaseMeshCoordinates_ =
        seedV0CoordCol && seedV1CoordCol && (sharedQXDetectCol || sharedXDetectCol);

    if (sharedBaseMeshCoordinates_) {
        auto sharedQXCol = table->GetColumnByName(pmsf::kParquetSharedVertexQXColumn);
        auto sharedQYCol = table->GetColumnByName(pmsf::kParquetSharedVertexQYColumn);
        auto sharedQZCol = table->GetColumnByName(pmsf::kParquetSharedVertexQZColumn);
        auto sharedXCol = table->GetColumnByName(pmsf::kParquetSharedVertexXColumn);
        auto sharedYCol = table->GetColumnByName(pmsf::kParquetSharedVertexYColumn);
        auto sharedZCol = table->GetColumnByName(pmsf::kParquetSharedVertexZColumn);
        for (int64_t row = 0; row < table->num_rows(); ++row) {
            if (sharedQXCol && sharedQYCol && sharedQZCol) {
                auto i16x = extractInt16List(sharedQXCol->chunk(0), row);
                if (!i16x.empty()) {
                    auto i16y = extractInt16List(sharedQYCol->chunk(0), row);
                    auto i16z = extractInt16List(sharedQZCol->chunk(0), row);
                    sharedVertexQX_.assign(i16x.begin(), i16x.end());
                    sharedVertexQY_.assign(i16y.begin(), i16y.end());
                    sharedVertexQZ_.assign(i16z.begin(), i16z.end());
                    break;
                }
            } else if (sharedXCol && sharedYCol && sharedZCol) {
                auto xs = extractFloatList(sharedXCol->chunk(0), row);
                if (!xs.empty()) {
                    sharedVertexX_ = std::move(xs);
                    sharedVertexY_ = extractFloatList(sharedYCol->chunk(0), row);
                    sharedVertexZ_ = extractFloatList(sharedZCol->chunk(0), row);
                    break;
                }
            }
        }
    }

    for (int64_t row = 0; row < table->num_rows(); row++) {
        auto gidChunk = std::static_pointer_cast<arrow::Int32Array>(groupIdCol->chunk(0));
        int gid = gidChunk->Value(row);

        if (targetGroupId_ >= 0 && gid != targetGroupId_) continue;

        GroupBaseMeshData data;
        data.groupId = gid;

        auto getFloat = [&](const std::string& name) -> float {
            auto col = table->GetColumnByName(name);
            if (!col) return 0.0f;
            auto chunk = std::static_pointer_cast<arrow::FloatArray>(col->chunk(0));
            return chunk->IsNull(row) ? 0.0f : chunk->Value(row);
        };
        auto getInt32 = [&](const std::string& name) -> int32_t {
            auto col = table->GetColumnByName(name);
            if (!col) return 0;
            // 兼容 int16 和 int32 两种类型
            if (col->chunk(0)->type_id() == arrow::Type::INT16) {
                auto chunk = std::static_pointer_cast<arrow::Int16Array>(col->chunk(0));
                return chunk->IsNull(row) ? 0 : (int32_t)chunk->Value(row);
            }
            auto chunk = std::static_pointer_cast<arrow::Int32Array>(col->chunk(0));
            return chunk->IsNull(row) ? 0 : chunk->Value(row);
        };
        auto getBool = [&](const std::string& name) -> bool {
            auto col = table->GetColumnByName(name);
            if (!col) return false;
            auto chunk = std::static_pointer_cast<arrow::BooleanArray>(col->chunk(0));
            return !chunk->IsNull(row) && chunk->Value(row);
        };

        data.seedV0X = getFloat(pmsf::kParquetSeedV0XColumn); data.seedV0Y = getFloat(pmsf::kParquetSeedV0YColumn); data.seedV0Z = getFloat(pmsf::kParquetSeedV0ZColumn);
        data.seedV1X = getFloat(pmsf::kParquetSeedV1XColumn); data.seedV1Y = getFloat(pmsf::kParquetSeedV1YColumn); data.seedV1Z = getFloat(pmsf::kParquetSeedV1ZColumn);
        data.seedV0QX = getInt32(pmsf::kParquetSeedV0QXColumn); data.seedV0QY = getInt32(pmsf::kParquetSeedV0QYColumn); data.seedV0QZ = getInt32(pmsf::kParquetSeedV0QZColumn);
        data.seedV1QX = getInt32(pmsf::kParquetSeedV1QXColumn); data.seedV1QY = getInt32(pmsf::kParquetSeedV1QYColumn); data.seedV1QZ = getInt32(pmsf::kParquetSeedV1QZColumn);
        data.seedV0CoordId = getInt32(pmsf::kParquetSeedV0CoordIdColumn);
        data.seedV1CoordId = getInt32(pmsf::kParquetSeedV1CoordIdColumn);
        data.numVertices = getInt32(pmsf::kParquetNumVerticesColumn);
        data.numFaces = getInt32(pmsf::kParquetNumFacesColumn);
        data.faceVerticesAreCoordIds = getBool(pmsf::kParquetFaceVerticesAreCoordIdsColumn);

        if (vertexCoordIdsCol) {
            data.vertexCoordIds = extractInt32List(vertexCoordIdsCol->chunk(0), row);
        }
        auto inlinePosCol = table->GetColumnByName(pmsf::kParquetVertexInlinePositionsColumn);
        if (inlinePosCol) data.vertexSharedPositions.clear();
        std::vector<int32_t> vertexInlinePositions;
        if (inlinePosCol) vertexInlinePositions = extractInt32List(inlinePosCol->chunk(0), row);
        auto sharedPosCol = table->GetColumnByName(pmsf::kParquetVertexSharedPositionsColumn);
        if (sharedPosCol) data.vertexSharedPositions = extractInt32List(sharedPosCol->chunk(0), row);

        // 顶点坐标：兼容浮点和量化两种模式
        auto vxCol = table->GetColumnByName(pmsf::kParquetVertexXColumn);
        auto vyCol = table->GetColumnByName(pmsf::kParquetVertexYColumn);
        auto vzCol = table->GetColumnByName(pmsf::kParquetVertexZColumn);
        if (vxCol) data.vertexX = extractFloatList(vxCol->chunk(0), row);
        if (vyCol) data.vertexY = extractFloatList(vyCol->chunk(0), row);
        if (vzCol) data.vertexZ = extractFloatList(vzCol->chunk(0), row);

        // 量化顶点坐标：兼容 int16 和 int32
        auto vqxCol = table->GetColumnByName(pmsf::kParquetVertexQXColumn);
        auto vqyCol = table->GetColumnByName(pmsf::kParquetVertexQYColumn);
        auto vqzCol = table->GetColumnByName(pmsf::kParquetVertexQZColumn);
        if (vqxCol) {
            // 检测是 int16 还是 int32 列表
            auto listArr = std::static_pointer_cast<arrow::ListArray>(vqxCol->chunk(0));
            if (listArr->value_type()->id() == arrow::Type::INT16) {
                auto i16x = extractInt16List(vqxCol->chunk(0), row);
                auto i16y = extractInt16List(vqyCol->chunk(0), row);
                auto i16z = extractInt16List(vqzCol->chunk(0), row);
                data.vertexQX.assign(i16x.begin(), i16x.end());
                data.vertexQY.assign(i16y.begin(), i16y.end());
                data.vertexQZ.assign(i16z.begin(), i16z.end());
            } else {
                data.vertexQX = extractInt32List(vqxCol->chunk(0), row);
                if (vqyCol) data.vertexQY = extractInt32List(vqyCol->chunk(0), row);
                if (vqzCol) data.vertexQZ = extractInt32List(vqzCol->chunk(0), row);
            }
        }

        if (sharedBaseMeshCoordinates_ && data.numVertices > 0) {
            if (header_.isEnableQuantization()) {
                std::vector<int32_t> fullX(data.numVertices, 0);
                std::vector<int32_t> fullY(data.numVertices, 0);
                std::vector<int32_t> fullZ(data.numVertices, 0);
                if (!vertexInlinePositions.empty()) {
                    for (size_t i = 0; i < vertexInlinePositions.size() && i < data.vertexQX.size(); ++i) {
                        int32_t pos = vertexInlinePositions[i];
                        if (pos < 0 || pos >= data.numVertices) continue;
                        fullX[pos] = data.vertexQX[i];
                        fullY[pos] = i < data.vertexQY.size() ? data.vertexQY[i] : 0;
                        fullZ[pos] = i < data.vertexQZ.size() ? data.vertexQZ[i] : 0;
                    }
                } else if (data.vertexQX.size() == static_cast<size_t>(data.numVertices)) {
                    fullX = data.vertexQX;
                    fullY = data.vertexQY;
                    fullZ = data.vertexQZ;
                }
                for (size_t i = 0; i < data.vertexSharedPositions.size() && i < data.vertexCoordIds.size(); ++i) {
                    int32_t pos = data.vertexSharedPositions[i];
                    int32_t cid = data.vertexCoordIds[i];
                    if (pos < 0 || pos >= data.numVertices || cid < 0 ||
                        static_cast<size_t>(cid) >= sharedVertexQX_.size()) {
                        continue;
                    }
                    fullX[pos] = sharedVertexQX_[cid];
                    fullY[pos] = sharedVertexQY_[cid];
                    fullZ[pos] = sharedVertexQZ_[cid];
                }
                data.vertexQX = std::move(fullX);
                data.vertexQY = std::move(fullY);
                data.vertexQZ = std::move(fullZ);
            } else {
                std::vector<float> fullX(data.numVertices, 0.0f);
                std::vector<float> fullY(data.numVertices, 0.0f);
                std::vector<float> fullZ(data.numVertices, 0.0f);
                if (!vertexInlinePositions.empty()) {
                    for (size_t i = 0; i < vertexInlinePositions.size() && i < data.vertexX.size(); ++i) {
                        int32_t pos = vertexInlinePositions[i];
                        if (pos < 0 || pos >= data.numVertices) continue;
                        fullX[pos] = data.vertexX[i];
                        fullY[pos] = i < data.vertexY.size() ? data.vertexY[i] : 0.0f;
                        fullZ[pos] = i < data.vertexZ.size() ? data.vertexZ[i] : 0.0f;
                    }
                } else if (data.vertexX.size() == static_cast<size_t>(data.numVertices)) {
                    fullX = data.vertexX;
                    fullY = data.vertexY;
                    fullZ = data.vertexZ;
                }
                for (size_t i = 0; i < data.vertexSharedPositions.size() && i < data.vertexCoordIds.size(); ++i) {
                    int32_t pos = data.vertexSharedPositions[i];
                    int32_t cid = data.vertexCoordIds[i];
                    if (pos < 0 || pos >= data.numVertices || cid < 0 ||
                        static_cast<size_t>(cid) >= sharedVertexX_.size()) {
                        continue;
                    }
                    fullX[pos] = sharedVertexX_[cid];
                    fullY[pos] = sharedVertexY_[cid];
                    fullZ[pos] = sharedVertexZ_[cid];
                }
                data.vertexX = std::move(fullX);
                data.vertexY = std::move(fullY);
                data.vertexZ = std::move(fullZ);
            }
        }

        // 量化模式下如果没有浮点坐标，从量化坐标反量化
        if (data.vertexX.empty() && !data.vertexQX.empty()) {
            data.vertexX.resize(data.vertexQX.size());
            data.vertexY.resize(data.vertexQY.size());
            data.vertexZ.resize(data.vertexQZ.size());
            for (size_t i = 0; i < data.vertexQX.size(); i++) {
                data.vertexX[i] = bboxMinX_ + (data.vertexQX[i] + 0.5f) * quantStep_;
                data.vertexY[i] = bboxMinY_ + (data.vertexQY[i] + 0.5f) * quantStep_;
                data.vertexZ[i] = bboxMinZ_ + (data.vertexQZ[i] + 0.5f) * quantStep_;
            }
        }

        auto fvCol = table->GetColumnByName(pmsf::kParquetFaceVerticesColumn);
        auto fdCol = table->GetColumnByName(pmsf::kParquetFaceDegreesColumn);
        if (fvCol) data.faceVertices = extractInt32List(fvCol->chunk(0), row);
        if (fdCol) data.faceDegrees = extractInt32List(fdCol->chunk(0), row);

        // 向后兼容：如果旧格式有 face_v0/v1/v2 但没有 face_vertices/face_degrees
        if (data.faceVertices.empty()) {
            auto f0Col = table->GetColumnByName(pmsf::kParquetFaceV0Column);
            auto f1Col = table->GetColumnByName(pmsf::kParquetFaceV1Column);
            auto f2Col = table->GetColumnByName(pmsf::kParquetFaceV2Column);
            if (f0Col && f1Col && f2Col) {
                auto v0 = extractInt32List(f0Col->chunk(0), row);
                auto v1 = extractInt32List(f1Col->chunk(0), row);
                auto v2 = extractInt32List(f2Col->chunk(0), row);
                for (size_t i = 0; i < v0.size(); i++) {
                    data.faceVertices.push_back(v0[i]);
                    data.faceVertices.push_back(v1[i]);
                    data.faceVertices.push_back(v2[i]);
                    data.faceDegrees.push_back(3);
                }
            }
        }

        baseMeshData_.push_back(std::move(data));
    }

    if (parquetV2VerboseLogs()) {
        std::cout << "[Columnar] base_mesh: " << baseMeshData_.size() << " 个分区" << std::endl;
    }
}

// ============================================================================
// 读取 symbols 表
// ============================================================================

void ColumnarDeserializeOperator::readSymbolsTable(const std::string& path) {
    // ========== codec=4 列式段表路径（优先）==========
    // codec=4 把符号数据拆成 <base>_bit / _residual / _lod 三张表，不再写 legacy
    // <base>.parquet。因此本路径不依赖 symbols 表是否存在。
    if (codecVersion_ == 4) {
        std::string baseName = path;
        const std::string suffix = pmsf::kPMSFSymbolsParquetSuffix;
        if (baseName.size() >= suffix.size() &&
            baseName.substr(baseName.size() - suffix.size()) == suffix) {
            baseName.resize(baseName.size() - suffix.size());
        }
        if (readColumnarSymbolsTablesV4(baseName)) {
            return;
        }
        std::cerr << "[Columnar] codec=4 读取失败，回退到 codec_version=3 路径"
                  << std::endl;
        codecVersion_ = 3;
    }

    std::shared_ptr<arrow::Table> table;
    if (std::filesystem::exists(path)) {
        table = readParquetFile(path);
    }
    if (!table && codecVersion_ >= 2) {
        std::string baseName = path;
        const std::string suffix = pmsf::kPMSFSymbolsParquetSuffix;
        if (baseName.size() >= suffix.size() &&
            baseName.substr(baseName.size() - suffix.size()) == suffix) {
            baseName.resize(baseName.size() - suffix.size());
        }
        auto baseTable = readParquetFile(pmsf::paperPartitionedBaseTablePath(baseName));
        if (baseTable && baseTable->num_rows() > 0) {
            auto embeddedCol = baseTable->GetColumnByName(pmsf::kParquetSymbolsBlobColumn);
            if (embeddedCol && embeddedCol->num_chunks() > 0) {
                auto arr = std::static_pointer_cast<arrow::BinaryArray>(embeddedCol->chunk(0));
                if (arr->length() > 0 && !arr->IsNull(0)) {
                    int32_t len = 0;
                    const std::uint8_t* ptr = arr->GetValue(0, &len);
                    rcBlob_.assign(reinterpret_cast<const char*>(ptr),
                                   reinterpret_cast<const char*>(ptr) + len);
                    if (!unwrapSymbolBlobIfNeeded(rcBlob_)) {
                        rcBlob_.clear();
                        return;
                    }
                    rcBlobOffset_ = 0;
                    rcBlobInitialized_ = false;
                    if (parquetV2VerboseLogs()) {
                        std::cout << "[Columnar] embedded symbols rc_blob: "
                                  << rcBlob_.size() << " bytes" << std::endl;
                    }
                    return;
                }
            }
        }
    }
    if (!table) return;

    symbolData_.clear();

    // ========== V2 BPV 优化路径：读 rc_blob BINARY 单列 ==========
    if (codecVersion_ >= 2) {
        auto blobCol = table->GetColumnByName(pmsf::kParquetRCBlobColumn);
        if (!blobCol || blobCol->num_chunks() == 0) {
            std::cerr << "[Columnar] codec_version=" << codecVersion_
                      << " 但 symbols 表缺少 rc_blob 列；回退旧路径" << std::endl;
            codecVersion_ = 1;
        } else {
            auto chunk = std::static_pointer_cast<arrow::BinaryArray>(blobCol->chunk(0));
            if (chunk->length() == 0) {
                std::cerr << "[Columnar] rc_blob 列为空" << std::endl;
                return;
            }
            // 合并所有行的 blob（目前 serialize 端单行，兼容多行也能运行）
            size_t total = 0;
            for (int64_t i = 0; i < chunk->length(); i++) {
                if (!chunk->IsNull(i)) total += chunk->value_length(i);
            }
            rcBlob_.assign(total, 0);
            size_t off = 0;
            for (int64_t i = 0; i < chunk->length(); i++) {
                if (chunk->IsNull(i)) continue;
                int32_t len = chunk->value_length(i);
                const char* p = reinterpret_cast<const char*>(chunk->GetValue(i, &len));
                std::memcpy(rcBlob_.data() + off, p, len);
                off += len;
            }
            if (!unwrapSymbolBlobIfNeeded(rcBlob_)) {
                rcBlob_.clear();
                return;
            }
            rcBlobOffset_ = 0;
            rcBlobInitialized_ = false;
            if (parquetV2VerboseLogs()) {
                std::cout << "[Columnar] symbols rc_blob: " << rcBlob_.size()
                          << " bytes (codec_version=" << codecVersion_ << ")" << std::endl;
            }
            return;
        }
    }

    // ========== 旧 bool-list 多列路径 ==========
    auto groupIdCol = table->GetColumnByName(pmsf::kParquetGroupIdColumn);
    auto roundIdCol = table->GetColumnByName(pmsf::kParquetRoundIdColumn);
    if (!groupIdCol || !roundIdCol || groupIdCol->num_chunks() == 0 || roundIdCol->num_chunks() == 0) {
        std::cerr << "[Columnar] symbols 缺少 group_id/round_id 列" << std::endl;
        return;
    }

    for (int64_t row = 0; row < table->num_rows(); row++) {
        auto gidChunk = std::static_pointer_cast<arrow::Int32Array>(groupIdCol->chunk(0));
        auto ridChunk = std::static_pointer_cast<arrow::Int32Array>(roundIdCol->chunk(0));
        int gid = gidChunk->Value(row);
        int rid = ridChunk->Value(row);

        if (targetGroupId_ >= 0 && gid != targetGroupId_) continue;
        if (maxRounds_ >= 0 && rid >= maxRounds_) continue;

        GroupSymbolData data;
        data.groupId = gid;
        data.roundId = rid;

        auto fsCol = table->GetColumnByName(pmsf::kParquetFacetSymbolsColumn);
        auto esCol = table->GetColumnByName(pmsf::kParquetEdgeSymbolsColumn);
        auto vsCol = table->GetColumnByName(pmsf::kParquetVertexSymbolsColumn);
        if (fsCol) data.facetSymbols = extractBoolList(fsCol->chunk(0), row);
        if (esCol) data.edgeSymbols = extractBoolList(esCol->chunk(0), row);
        if (vsCol) data.vertexSymbols = extractBoolList(vsCol->chunk(0), row);

        auto pxCol = table->GetColumnByName(pmsf::kParquetPointXColumn);
        auto pyCol = table->GetColumnByName(pmsf::kParquetPointYColumn);
        auto pzCol = table->GetColumnByName(pmsf::kParquetPointZColumn);
        if (pxCol) data.pointX = extractFloatList(pxCol->chunk(0), row);
        if (pyCol) data.pointY = extractFloatList(pyCol->chunk(0), row);
        if (pzCol) data.pointZ = extractFloatList(pzCol->chunk(0), row);

        auto vpxCol = table->GetColumnByName(pmsf::kParquetVertexPointXColumn);
        auto vpyCol = table->GetColumnByName(pmsf::kParquetVertexPointYColumn);
        auto vpzCol = table->GetColumnByName(pmsf::kParquetVertexPointZColumn);
        if (vpxCol) data.vertexPointX = extractFloatList(vpxCol->chunk(0), row);
        if (vpyCol) data.vertexPointY = extractFloatList(vpyCol->chunk(0), row);
        if (vpzCol) data.vertexPointZ = extractFloatList(vpzCol->chunk(0), row);

        auto pqxCol = table->GetColumnByName(pmsf::kParquetPointQXColumn);
        auto pqyCol = table->GetColumnByName(pmsf::kParquetPointQYColumn);
        auto pqzCol = table->GetColumnByName(pmsf::kParquetPointQZColumn);
        if (pqxCol) {
            // 兼容 int16 和 int32 列表
            auto listArr = std::static_pointer_cast<arrow::ListArray>(pqxCol->chunk(0));
            if (!listArr->IsNull(row) && listArr->value_type()->id() == arrow::Type::INT16) {
                auto i16x = extractInt16List(pqxCol->chunk(0), row);
                auto i16y = extractInt16List(pqyCol->chunk(0), row);
                auto i16z = extractInt16List(pqzCol->chunk(0), row);
                data.pointQX.assign(i16x.begin(), i16x.end());
                data.pointQY.assign(i16y.begin(), i16y.end());
                data.pointQZ.assign(i16z.begin(), i16z.end());
            } else {
                data.pointQX = extractInt32List(pqxCol->chunk(0), row);
                if (pqyCol) data.pointQY = extractInt32List(pqyCol->chunk(0), row);
                if (pqzCol) data.pointQZ = extractInt32List(pqzCol->chunk(0), row);
            }
        }

        auto vpqxCol = table->GetColumnByName(pmsf::kParquetVertexPointQXColumn);
        auto vpqyCol = table->GetColumnByName(pmsf::kParquetVertexPointQYColumn);
        auto vpqzCol = table->GetColumnByName(pmsf::kParquetVertexPointQZColumn);
        if (vpqxCol) {
            auto listArr = std::static_pointer_cast<arrow::ListArray>(vpqxCol->chunk(0));
            if (!listArr->IsNull(row) && listArr->value_type()->id() == arrow::Type::INT16) {
                auto i16x = extractInt16List(vpqxCol->chunk(0), row);
                auto i16y = extractInt16List(vpqyCol->chunk(0), row);
                auto i16z = extractInt16List(vpqzCol->chunk(0), row);
                data.vertexPointQX.assign(i16x.begin(), i16x.end());
                data.vertexPointQY.assign(i16y.begin(), i16y.end());
                data.vertexPointQZ.assign(i16z.begin(), i16z.end());
            } else {
                data.vertexPointQX = extractInt32List(vpqxCol->chunk(0), row);
                if (vpqyCol) data.vertexPointQY = extractInt32List(vpqyCol->chunk(0), row);
                if (vpqzCol) data.vertexPointQZ = extractInt32List(vpqzCol->chunk(0), row);
            }
        }

        symbolData_.push_back(std::move(data));
    }

    if (parquetV2VerboseLogs()) {
        std::cout << "[Columnar] symbols: " << symbolData_.size() << " 条记录" << std::endl;
    }
}

// ============================================================================
// codec=4 列式段表读取与重组
// ============================================================================

bool ColumnarDeserializeOperator::readColumnarSymbolsTablesV4(
    const std::string& baseName) {
    auto bitTable = readParquetFile(pmsf::paperBitTablePath(baseName));
    const std::string residualParquetPath = pmsf::paperResidualTablePath(baseName);
    auto residualTable = readParquetFile(residualParquetPath);
    auto lodTable = readParquetFile(pmsf::paperLodTablePath(baseName));
    auto sideTable = readParquetFile(pmsf::paperSideTablePath(baseName));
    if (!residualTable) {
        // PPMC lean residual channel: a raw _residual.bin sidecar instead of a
        // one-row Parquet file (the payload is already range-coded, so the
        // Parquet footer/statistics were pure overhead).  Only PPMC emits it.
        static const std::string kResidualSuffix = "_residual.parquet";
        if (residualParquetPath.size() > kResidualSuffix.size() &&
            residualParquetPath.compare(residualParquetPath.size() - kResidualSuffix.size(),
                                        kResidualSuffix.size(), kResidualSuffix) == 0) {
            const std::string binPath =
                residualParquetPath.substr(0, residualParquetPath.size() - kResidualSuffix.size()) +
                "_residual.bin";
            std::ifstream rawIn(binPath, std::ios::binary);
            if (rawIn) {
                std::vector<std::uint8_t> rawBytes((std::istreambuf_iterator<char>(rawIn)),
                                                   std::istreambuf_iterator<char>());
                if (!rawBytes.empty()) {
                    std::shared_ptr<arrow::Array> dataArr;
                    arrow::BinaryBuilder dataBuilder;
                    std::shared_ptr<arrow::Array> modeArr;
                    arrow::Int8Builder modeBuilder;
                    if (dataBuilder.Append(rawBytes.data(), rawBytes.size()).ok() &&
                        dataBuilder.Finish(&dataArr).ok() && modeBuilder.Append(0).ok() &&
                        modeBuilder.Finish(&modeArr).ok()) {
                        auto rawSchema = arrow::schema(
                            {arrow::field(pmsf::kParquetResidualDataColumn, arrow::binary()),
                             arrow::field(pmsf::kParquetEncodingModeColumn, arrow::int8())});
                        residualTable = arrow::Table::Make(rawSchema, {dataArr, modeArr}, 1);
                        std::cout << "[Columnar] raw residual blob fallback: " << binPath
                                  << " bytes=" << rawBytes.size() << std::endl;
                    }
                }
            }
        }
    }
    if (!residualTable) {
        std::cerr << "[Columnar] codec=4 缺少 residual 表" << std::endl;
        return false;
    }

    // (roi, lod) 谓词（设计文档 §2.5）的两个维度在哪里落实：
    //
    //   region —— 行级谓词，就在本文件里：`targetGroupId_` 一路透传到
    //             group 循环与每张表的行遍历（见下方各处 `gid != targetGroupId_`）。
    //
    //   lod    —— **不是**行级谓词。`_bit` / `_residual` / `_lod` 三张表都是
    //             整段 blob 的重组输入，读者（reassembleBlobFromSegmentsV4 /
    //             CanonicalArrowTableReader）一次性把全部 segment 拼回一个 blob，
    //             不是前缀感知的。把行裁成前缀只会拼出一个缺头少尾的 blob：
    //             实测把 `_lod` 裁到 0 行会丢掉 blob 的 header/offsets，
    //             下游按垃圾长度分配 vector 直接抛 bad_alloc。
    //
    //             lod 维度由**解码主循环**落实：`runDecompress` 按
    //             `queryMaxLod()` 只跑前 k 轮（见 CompressPipeline.cpp）。它对
    //             任意算法、任意后端成立，且可证等价于「全量解码第 k 轮的中间
    //             状态」—— 契约测试 B3 逐字节断言了这一点。
    //
    // 让 lod 也变成行级谓词，需要 Phase 3 的「lod_index 每 (partition, lod) 一行
    // + 前缀感知 reader」；在那之前，宣称行级 lod 过滤就是虚报。

    // Canonical facet/edge/vertex/point + side-channel path.
    if (blobLayout_ == pmsf::kPaperBlobLayoutCanonical) {
        if (!bitTable) {
            std::cerr << "[Columnar] canonical path 缺少 bit 表" << std::endl;
            return false;
        }
        try {
            // The on-disk blob_layout is the canonical wrapper.  If the file
            // carries a non-zero canonical_source_layout, decode/reassemble using
            // that original algorithm-native layout's spec.
            const std::int32_t effectiveLayout =
                (canonicalSourceLayout_ != 0)
                    ? static_cast<std::int32_t>(canonicalSourceLayout_)
                    : static_cast<std::int32_t>(blobLayout_);
            const auto& specs = pmsf::SymbolStreamRegistry::instance().specsFor(
                effectiveLayout);
            auto payload = pmsf::CanonicalArrowTableReader::readPayload(
                *bitTable, *residualTable, lodTable.get(), sideTable.get(), specs);
            pmsf::CanonicalSymbolDecoder decoder(payload, specs);
            if (header_.isEnableQuantization()) {
                decoder.setQuantization(
                    quantBits_,
                    MCGAL::Point(bboxMinX_, bboxMinY_, bboxMinZ_),
                    quantStep_);
            }
            auto rounds = decoder.decode();

            // Migrated non-Class-I layouts: reassemble the original algorithm
            // blob from the decoded canonical symbol stream, then let the normal
            // paperImportRangeCodedBlobForLayout path consume it.
            if (effectiveLayout == pmsf::kPaperBlobLayoutRDTag) {
                auto rdBlob = reassembleRDTagV1BlobFromCanonical(
                    rounds,
                    header_.isEnableQuantization() ? quantBits_ : 0);
                rcBlob_.assign(
                    reinterpret_cast<const char*>(rdBlob.data()),
                    reinterpret_cast<const char*>(rdBlob.data() + rdBlob.size()));
                rcBlobOffset_ = 0;
                rcBlobInitialized_ = false;
                blobLayout_ = effectiveLayout;
                if (parquetV2VerboseLogs()) {
                    std::cout << "[Columnar] canonical RD_TAG reassembled: "
                              << rcBlob_.size() << " bytes" << std::endl;
                }
                return true;
            }
            if (effectiveLayout == pmsf::kPaperBlobLayoutPCLTTMTag) {
                auto pclBlob = pmsf::reassemblePCLTTMTagV3BlobFromCanonical(
                    rounds,
                    header_.isEnableQuantization() ? quantBits_ : 0);
                rcBlob_.assign(
                    reinterpret_cast<const char*>(pclBlob.data()),
                    reinterpret_cast<const char*>(pclBlob.data() + pclBlob.size()));
                rcBlobOffset_ = 0;
                rcBlobInitialized_ = false;
                blobLayout_ = effectiveLayout;
                if (parquetV2VerboseLogs()) {
                    std::cout << "[Columnar] canonical CODEC_016 reassembled: "
                              << rcBlob_.size() << " bytes" << std::endl;
                }
                return true;
            }
            if (effectiveLayout == pmsf::kPaperBlobLayoutPCATMColor) {
                auto pcBlob = pmsf::reassemblePCATMColorV3BlobFromCanonical(
                    rounds,
                    header_.isEnableQuantization() ? quantBits_ : 0);
                rcBlob_.assign(
                    reinterpret_cast<const char*>(pcBlob.data()),
                    reinterpret_cast<const char*>(pcBlob.data() + pcBlob.size()));
                rcBlobOffset_ = 0;
                rcBlobInitialized_ = false;
                blobLayout_ = effectiveLayout;
                if (parquetV2VerboseLogs()) {
                    std::cout << "[Columnar] canonical CODEC_014 reassembled: "
                              << rcBlob_.size() << " bytes" << std::endl;
                }
                return true;
            }
            if (effectiveLayout == pmsf::kPaperBlobLayoutPCATMZColor ||
                effectiveLayout == pmsf::kPaperBlobLayoutPCATMPaper) {
                const std::int32_t schemeVersion =
                    (effectiveLayout == pmsf::kPaperBlobLayoutPCATMZColor)
                        ? pmsf::kPaperBlobSchemeVersionPCATMZColor
                        : pmsf::kPaperBlobSchemeVersionPCATMPaper;
                auto pcBlob = pmsf::reassemblePatchColorBlobFromCanonical(
                    rounds, effectiveLayout, schemeVersion,
                    header_.isEnableQuantization() ? quantBits_ : 0);
                rcBlob_.assign(
                    reinterpret_cast<const char*>(pcBlob.data()),
                    reinterpret_cast<const char*>(pcBlob.data() + pcBlob.size()));
                rcBlobOffset_ = 0;
                rcBlobInitialized_ = false;
                blobLayout_ = effectiveLayout;
                if (parquetV2VerboseLogs()) {
                    std::cout << "[Columnar] canonical patch-color reassembled: "
                              << rcBlob_.size() << " bytes" << std::endl;
                }
                return true;
            }
            if (effectiveLayout == pmsf::kPaperBlobLayoutPCATMPure) {
                // CODEC_021 sv=2 uses a multi-context isPoly model; keep the
                // legacy scheme version so the existing reader decodes correctly.
                const std::int32_t schemeVersion = pmsf::kPaperBlobSchemeVersionPCATMPure;
                auto pcBlob = pmsf::reassemblePCATMPureBlobFromCanonical(
                    rounds, schemeVersion,
                    header_.isEnableQuantization() ? quantBits_ : 0);
                rcBlob_.assign(
                    reinterpret_cast<const char*>(pcBlob.data()),
                    reinterpret_cast<const char*>(pcBlob.data() + pcBlob.size()));
                rcBlobOffset_ = 0;
                rcBlobInitialized_ = false;
                blobLayout_ = effectiveLayout;
                if (parquetV2VerboseLogs()) {
                    std::cout << "[Columnar] canonical CODEC_021 reassembled: "
                              << rcBlob_.size() << " bytes" << std::endl;
                }
                return true;
            }

            // Class-I canonical path: import facet/edge/vertex/point directly.
            std::vector<bool> facetSymbols;
            std::vector<bool> edgeSymbols;
            std::vector<bool> vertexSymbols;
            std::vector<MCGAL::Point> points;
            std::vector<MCGAL::Point> vertexPoints;
            std::vector<MCGAL::PointInt> residuals;
            std::vector<MCGAL::PointInt> vertexResiduals;
            facetSymbols.reserve(rounds.size() * 16);
            edgeSymbols.reserve(rounds.size() * 16);
            vertexSymbols.reserve(rounds.size() * 4);
            points.reserve(rounds.size() * 8);
            vertexPoints.reserve(rounds.size() * 2);
            residuals.reserve(rounds.size() * 8);
            vertexResiduals.reserve(rounds.size() * 2);

            const bool useResidual = header_.isEnablePrediction();
            for (const auto& round : rounds) {
                for (pmsf::SymbolCode s : round.facetSymbols) {
                    facetSymbols.push_back(s != 0);
                }
                for (pmsf::SymbolCode s : round.triangulationEdgeSymbols) {
                    edgeSymbols.push_back(s != 0);
                }
                for (pmsf::SymbolCode s : round.vertexSymbols) {
                    vertexSymbols.push_back(s != 0);
                }
                if (useResidual) {
                    for (const auto& r : round.removedVertexResiduals) {
                        residuals.push_back(r);
                    }
                    for (const auto& r : round.shiftedVertexResiduals) {
                        vertexResiduals.push_back(r);
                    }
                } else {
                    for (const auto& p : round.removedVertexCoords) {
                        points.push_back(p);
                    }
                    for (const auto& p : round.shiftedVertexCoords) {
                        vertexPoints.push_back(p);
                    }
                }
            }

            symbolReadOperator_->importFromParquetExtended(
                facetSymbols, edgeSymbols, vertexSymbols,
                useResidual ? std::vector<MCGAL::Point>() : points,
                useResidual ? std::vector<MCGAL::Point>() : vertexPoints);
            if (useResidual) {
                symbolReadOperator_->importResidualsFromParquet(residuals);
            }
            rcBlobInitialized_ = true;
            if (parquetV2VerboseLogs()) {
                std::cout << "[Columnar] canonical path decoded: rounds="
                          << rounds.size()
                          << " facets=" << facetSymbols.size()
                          << " edges=" << edgeSymbols.size()
                          << " points=" << points.size()
                          << " vertexPoints=" << vertexPoints.size()
                          << " residuals=" << residuals.size()
                          << std::endl;
            }
            return true;
        } catch (const std::exception& e) {
            std::cerr << "[Columnar] canonical path decode failed: " << e.what()
                      << std::endl;
            return false;
        }
    }

    // Empty bit table is allowed for Class-II/S layouts that store the whole
    // blob as a single residual segment. Build a 0-row placeholder on demand.
    std::shared_ptr<arrow::Table> emptyBitTable;
    if (!bitTable) {
        auto schema = arrow::schema({
            arrow::field(pmsf::kParquetGroupIdColumn,        arrow::int32(),  false),
            arrow::field(pmsf::kParquetRoundIdColumn,        arrow::int32(),  false),
            arrow::field(pmsf::kParquetStreamIdColumn,       arrow::int32(),  false),
            arrow::field(pmsf::kParquetEncodingModeColumn,   arrow::int8(),   false),
            arrow::field(pmsf::kParquetBitDataColumn,        arrow::binary(), false),
        });
        arrow::Int32Builder gb, rb, sb;
        arrow::Int8Builder mb;
        arrow::BinaryBuilder db;
        std::shared_ptr<arrow::Array> ga, ra, sa, ma, da;
        (void)gb.Finish(&ga); (void)rb.Finish(&ra); (void)sb.Finish(&sa);
        (void)mb.Finish(&ma); (void)db.Finish(&da);
        emptyBitTable = arrow::Table::Make(schema, {ga, ra, sa, ma, da}, 0);
        bitTable = emptyBitTable;
    }

    return reassembleBlobFromSegmentsV4(*bitTable, *residualTable, lodTable.get());
}

bool ColumnarDeserializeOperator::reassembleBlobFromSegmentsV4(
    const arrow::Table& bitTable,
    const arrow::Table& residualTable,
    const arrow::Table* lodTable) {
    // LOD 表：取第一行，读 header_blob、bit_start/len、residual_start/len
    const std::uint8_t* headerPtr = nullptr;
    int32_t headerLen = 0;
    std::shared_ptr<arrow::Int32Array> bitStartValues;
    std::shared_ptr<arrow::Int32Array> bitLenValues;
    int streamCount = 0;
    int residualStart = 0;
    int residualTotal = 0;
    if (lodTable && lodTable->num_rows() > 0) {
        auto headerCol = lodTable->GetColumnByName("header_blob");
        auto bitStartCol = lodTable->GetColumnByName(pmsf::kParquetBitStartColumn);
        auto bitLenCol = lodTable->GetColumnByName(pmsf::kParquetBitLenColumn);
        auto resStartCol = lodTable->GetColumnByName(pmsf::kParquetResidualStartColumn);
        auto resLenCol = lodTable->GetColumnByName(pmsf::kParquetResidualLenColumn);
        if (!headerCol || !bitStartCol || !bitLenCol || !resStartCol || !resLenCol) {
            std::cerr << "[Columnar] codec=4 lod 表缺少必要列" << std::endl;
            return false;
        }

        auto headerArr = std::static_pointer_cast<arrow::BinaryArray>(headerCol->chunk(0));
        headerPtr = headerArr->GetValue(0, &headerLen);

        auto bitStartArr = std::static_pointer_cast<arrow::ListArray>(bitStartCol->chunk(0));
        auto bitLenArr = std::static_pointer_cast<arrow::ListArray>(bitLenCol->chunk(0));
        if (bitStartArr->IsNull(0) || bitLenArr->IsNull(0)) {
            std::cerr << "[Columnar] codec=4 lod 表 bit 偏移为空" << std::endl;
            return false;
        }
        bitStartValues = std::static_pointer_cast<arrow::Int32Array>(bitStartArr->value_slice(0));
        bitLenValues = std::static_pointer_cast<arrow::Int32Array>(bitLenArr->value_slice(0));
        streamCount = static_cast<int>(bitStartValues->length());

        auto resStartArr = std::static_pointer_cast<arrow::Int32Array>(resStartCol->chunk(0));
        auto resLenArr = std::static_pointer_cast<arrow::Int32Array>(resLenCol->chunk(0));
        residualStart = resStartArr->Value(0);
        residualTotal = resLenArr->Value(0);
    }

    // bit 表：按 stream_id 排序，取每段 data + encoding_mode
    std::vector<std::vector<std::uint8_t>> bitBytes(streamCount);
    std::vector<int8_t> bitModes(streamCount, 0);
    auto sidCol = bitTable.GetColumnByName(pmsf::kParquetStreamIdColumn);
    auto dataCol = bitTable.GetColumnByName(pmsf::kParquetBitDataColumn);
    auto modeCol = bitTable.GetColumnByName(pmsf::kParquetEncodingModeColumn);
    if (!sidCol || !dataCol) {
        std::cerr << "[Columnar] codec=4 bit 表缺少 stream_id/data 列" << std::endl;
        return false;
    }
    auto sidArr = std::static_pointer_cast<arrow::Int32Array>(sidCol->chunk(0));
    auto dataArr = std::static_pointer_cast<arrow::BinaryArray>(dataCol->chunk(0));
    auto modeArr = modeCol ? std::static_pointer_cast<arrow::Int8Array>(modeCol->chunk(0)) : nullptr;
    for (int64_t r = 0; r < bitTable.num_rows(); ++r) {
        int sid = sidArr->Value(r);
        if (sid < 0 || sid >= streamCount) continue;
        int32_t len = 0;
        const std::uint8_t* ptr = dataArr->GetValue(r, &len);
        bitBytes[sid].assign(ptr, ptr + len);
        if (modeArr && r < modeArr->length()) {
            bitModes[sid] = modeArr->Value(r);
        }
    }

    // residual 表：取第一行 data + encoding_mode
    if (residualTable.num_rows() == 0) {
        std::cerr << "[Columnar] codec=4 residual 表为空" << std::endl;
        return false;
    }
    auto resDataCol = residualTable.GetColumnByName(pmsf::kParquetResidualDataColumn);
    auto resModeCol = residualTable.GetColumnByName(pmsf::kParquetEncodingModeColumn);
    if (!resDataCol) {
        std::cerr << "[Columnar] codec=4 residual 表缺少 data 列" << std::endl;
        return false;
    }
    auto resDataArr = std::static_pointer_cast<arrow::BinaryArray>(resDataCol->chunk(0));
    int32_t resLen = 0;
    const std::uint8_t* resPtr = resDataArr->GetValue(0, &resLen);
    int8_t residualMode = 0;
    if (resModeCol) {
        auto resModeArr = std::static_pointer_cast<arrow::Int8Array>(resModeCol->chunk(0));
        if (resModeArr && resModeArr->length() > 0) residualMode = resModeArr->Value(0);
    }

    // A3 fallback: for mode=1 native columns, deserialize and re-encode to entropy bytes.
    // We need to decode all mode=1 streams first, then re-encode the full Class I blob.
    bool hasMode1 = residualMode == 1;
    for (int8_t m : bitModes) if (m == 1) hasMode1 = true;

    if (hasMode1) {
        ReencodeInput input;
        // bit streams
        for (int s = 0; s < streamCount; ++s) {
            if (bitModes[s] == 1) {
                auto syms = deserializeNativeBoolColumn(bitBytes[s]);
                if (s == 0) input.facetSymbols = std::move(syms);
                else if (s == 1) input.edgeSymbols = std::move(syms);
                else { input.hasVertex = true; input.vertexSymbols = std::move(syms); }
            }
        }
        // residual
        if (residualMode == 1) {
            std::vector<std::uint8_t> resBytes(resPtr, resPtr + resLen);
            // The encoder stores residuals as PointInt only in the residual bit-plane
            // path (prediction + quantization). Otherwise it stores raw float points.
            const bool residualBitPlane = header_.isEnablePrediction() && header_.isEnableQuantization();
            if (residualBitPlane) {
                input.useResidual = true;
                input.residuals = deserializeNativeQuantColumn(resBytes);
            } else {
                input.useResidual = false;
                input.points = deserializeNativeFloatColumn(resBytes);
            }
        }
        // For mode=0 streams we still need their entropy bytes to reassemble.
        // Re-encode produces a full Class I blob, so we can replace rcBlob_ directly.
        if (parquetV2VerboseLogs()) {
            std::cout << "[Columnar] codec=4 mode=1 re-encode input: facet=" << input.facetSymbols.size()
                      << " edge=" << input.edgeSymbols.size()
                      << " residuals=" << input.residuals.size()
                      << " points=" << input.points.size()
                      << " useResidual=" << input.useResidual << std::endl;
        }
        const int maxBlobSize = 64 * 1024 * 1024;
        std::unique_ptr<char[]> blobBuf(new char[maxBlobSize]);
        int blobLen = reencodeClassIFromRawStreamsForDecode(input, blobBuf.get(), maxBlobSize);
        if (blobLen <= 0) {
            std::cerr << "[Columnar] codec=4 mode=1 re-encode failed" << std::endl;
            return false;
        }
        rcBlob_.assign(blobBuf.get(), blobBuf.get() + blobLen);
        rcBlobOffset_ = 0;
        rcBlobInitialized_ = false;
        if (parquetV2VerboseLogs()) {
            std::cout << "[Columnar] codec=4 mode=1 re-encoded rc_blob: "
                      << rcBlob_.size() << " bytes" << std::endl;
        }
        return true;
    }

    // 计算总大小并分配：header + bit 流 + residual 流
    int bitTotal = 0;
    for (int s = 0; s < streamCount; ++s) {
        bitTotal += bitLenValues->Value(s);
    }
    if (residualTotal == 0) {
        residualTotal = static_cast<int>(resLen);
    }
    (void)residualStart;

    std::vector<char> out;
    out.reserve(headerLen + bitTotal + residualTotal);
    out.insert(out.end(), reinterpret_cast<const char*>(headerPtr),
               reinterpret_cast<const char*>(headerPtr + headerLen));

    // 按 lod 记录的 bit_start/bit_len 顺序写入 connectivity payload。
    // 编码端 bit.data 是按 stream_id 分行的；这里按相同顺序拼接。
    for (int s = 0; s < streamCount; ++s) {
        const auto& bytes = bitBytes[s];
        const int expected = bitLenValues->Value(s);
        if (static_cast<int>(bytes.size()) != expected) {
            std::cerr << "[Columnar] codec=4 bit stream " << s
                      << " 大小不匹配: got=" << bytes.size()
                      << " expected=" << expected << std::endl;
        }
        out.insert(out.end(), bytes.begin(), bytes.end());
    }
    out.insert(out.end(), reinterpret_cast<const char*>(resPtr),
               reinterpret_cast<const char*>(resPtr + resLen));

    rcBlob_.assign(out.begin(), out.end());
    rcBlobOffset_ = 0;
    rcBlobInitialized_ = false;
    if (parquetV2VerboseLogs()) {
        std::cout << "[Columnar] codec=4 reassembled rc_blob: " << rcBlob_.size()
                  << " bytes (header=" << headerLen
                  << " bit=" << bitTotal
                  << " residual=" << residualTotal << ")" << std::endl;
    }
    return true;
}

std::shared_ptr<arrow::Table> readParquetFromMemory(const std::vector<std::uint8_t>& bytes) {
    if (bytes.empty()) return nullptr;
    auto buffer = std::make_shared<arrow::Buffer>(bytes.data(), bytes.size());
    auto input = std::make_shared<arrow::io::BufferReader>(buffer);
    std::unique_ptr<parquet::arrow::FileReader> reader;
    auto st = parquet::arrow::OpenFile(input, arrow::default_memory_pool()).Value(&reader);
    if (!st.ok()) {
        std::cerr << "[Columnar] failed to open memory parquet: " << st.ToString() << std::endl;
        return nullptr;
    }
    std::shared_ptr<arrow::Table> table;
    st = reader->ReadTable(&table);
    if (!st.ok()) {
        std::cerr << "[Columnar] failed to read memory parquet: " << st.ToString() << std::endl;
        return nullptr;
    }
    return table;
}

std::vector<char> ColumnarDeserializeOperator::deserializeNativeBoolColumn(
    const std::vector<std::uint8_t>& bytes) {
    auto table = readParquetFromMemory(bytes);
    if (!table || table->num_columns() == 0 || table->num_rows() == 0) return {};
    auto listCol = table->column(0);
    auto listArr = std::static_pointer_cast<arrow::ListArray>(listCol->chunk(0));
    if (!listArr || listArr->IsNull(0)) return {};
    auto values = std::static_pointer_cast<arrow::BooleanArray>(listArr->value_slice(0));
    std::vector<char> out;
    out.reserve(values->length());
    for (int64_t i = 0; i < values->length(); ++i) {
        out.push_back(values->IsNull(i) ? 0 : static_cast<char>(values->Value(i)));
    }
    return out;
}

std::vector<MCGAL::Point> ColumnarDeserializeOperator::deserializeNativeFloatColumn(
    const std::vector<std::uint8_t>& bytes) {
    auto table = readParquetFromMemory(bytes);
    if (!table || table->num_columns() == 0 || table->num_rows() == 0) return {};
    auto listCol = table->column(0);
    auto listArr = std::static_pointer_cast<arrow::ListArray>(listCol->chunk(0));
    if (!listArr || listArr->IsNull(0)) return {};
    auto values = std::static_pointer_cast<arrow::FloatArray>(listArr->value_slice(0));
    std::vector<MCGAL::Point> out;
    out.reserve(values->length() / 3);
    for (int64_t i = 0; i + 2 < values->length(); i += 3) {
        out.emplace_back(values->Value(i), values->Value(i + 1), values->Value(i + 2));
    }
    return out;
}

std::vector<MCGAL::PointInt> ColumnarDeserializeOperator::deserializeNativeQuantColumn(
    const std::vector<std::uint8_t>& bytes) {
    auto table = readParquetFromMemory(bytes);
    if (!table || table->num_columns() == 0 || table->num_rows() == 0) return {};
    auto listCol = table->column(0);
    auto listArr = std::static_pointer_cast<arrow::ListArray>(listCol->chunk(0));
    if (!listArr || listArr->IsNull(0)) return {};
    auto values = std::static_pointer_cast<arrow::Int32Array>(listArr->value_slice(0));
    std::vector<MCGAL::PointInt> out;
    out.reserve(values->length() / 3);
    for (int64_t i = 0; i + 2 < values->length(); i += 3) {
        out.emplace_back(values->Value(i), values->Value(i + 1), values->Value(i + 2));
    }
    return out;
}

int ColumnarDeserializeOperator::reencodeClassIFromRawStreamsForDecode(
    const ReencodeInput& input,
    char* outBuffer,
    int maxSize) {
    if (!outBuffer || maxSize <= 0) {
        std::cerr << "[Columnar] reencode: invalid args" << std::endl;
        return -1;
    }

    // Create a minimal mesh with the quantization parameters read from metadata.
    auto tmpMesh = std::make_shared<MCGAL::Mesh>();
    tmpMesh->i_nbQuantBits = static_cast<unsigned>(quantBits_);
    tmpMesh->bboxMin = MCGAL::Point(bboxMinX_, bboxMinY_, bboxMinZ_);
    tmpMesh->f_quantStep = quantStep_;

    OptimizedSymbolCollectOperator tmpCollector;
    tmpCollector.init(tmpMesh, {}, header_);
    // enablePrediction_ mirror: header has it; if not set on mesh, use header.
    tmpCollector.setEnablePrediction(header_.isEnablePrediction());

    std::vector<std::deque<char>> facetQs(1);
    facetQs[0].assign(input.facetSymbols.begin(), input.facetSymbols.end());
    tmpCollector.setFacetSymbolQueuesForReencode(std::move(facetQs));

    std::vector<std::deque<char>> edgeQs(1);
    edgeQs[0].assign(input.edgeSymbols.begin(), input.edgeSymbols.end());
    tmpCollector.setEdgeSymbolQueuesForReencode(std::move(edgeQs));

    if (input.hasVertex) {
        std::vector<std::deque<char>> vertexQs(1);
        vertexQs[0].assign(input.vertexSymbols.begin(), input.vertexSymbols.end());
        tmpCollector.setVertexSymbolQueuesForReencode(std::move(vertexQs));
    }

    const bool useResidualBitPlane = header_.isEnablePrediction() && header_.isEnableQuantization();
    if (useResidualBitPlane) {
        std::vector<std::deque<MCGAL::PointInt>> ipointQs(1);
        ipointQs[0].assign(input.residuals.begin(), input.residuals.end());
        tmpCollector.setIpointQueuesForReencode(std::move(ipointQs));
        // exportToBufferWithRangeCoder computes totalPointCoords from pointQueues
        // and vertexPointQueues, but uses ipointQueues for actual residual values.
        // We must provide matching-size dummy pointQueues so the header count is
        // correct and the outer loop runs.
        std::vector<std::deque<MCGAL::Point>> pointQs(1);
        pointQs[0].resize(input.residuals.size());
        std::vector<std::deque<MCGAL::Point>> vertexPointQs(1);
        tmpCollector.setPointQueuesForReencode(std::move(pointQs));
        tmpCollector.setVertexPointQueuesForReencode(std::move(vertexPointQs));
    } else {
        std::vector<std::deque<MCGAL::Point>> pointQs(1);
        pointQs[0].assign(input.points.begin(), input.points.end());
        std::vector<std::deque<MCGAL::Point>> vertexPointQs(1);
        tmpCollector.setPointQueuesForReencode(std::move(pointQs));
        tmpCollector.setVertexPointQueuesForReencode(std::move(vertexPointQs));
    }

    int len = tmpCollector.exportToBufferWithRangeCoder(outBuffer, header_.isEnableQuantization());
    if (parquetV2VerboseLogs() || len <= 0) {
        std::cerr << "[Columnar] reencode: export returned " << len
                  << " facet=" << input.facetSymbols.size()
                  << " edge=" << input.edgeSymbols.size()
                  << " residuals=" << input.residuals.size()
                  << " points=" << input.points.size()
                  << " useResidual=" << input.useResidual
                  << " hasVertex=" << input.hasVertex
                  << " pred=" << header_.isEnablePrediction()
                  << " quant=" << header_.isEnableQuantization()
                  << std::endl;
    }
    return len;
}

bool ColumnarDeserializeOperator::importPMSFSymbolTableAsSymbolData(const pmsf::PMSFSymbolTable& table,
                                                                      bool replaceExisting) {
    if (table.empty()) {
        return false;
    }
    if (!replaceExisting && (!symbolData_.empty() || !rcBlob_.empty())) {
        return false;
    }

    std::vector<const pmsf::PMSFSymbolRecord*> rows;
    rows.reserve(table.size());
    for (const auto& row : table) {
        const int gid = row.partitionId < 0 ? 0 : row.partitionId;
        const int rid = row.lodId < 0 ? 0 : row.lodId;
        if (targetGroupId_ >= 0 && gid != targetGroupId_) continue;
        if (maxRounds_ >= 0 && rid >= maxRounds_) continue;
        if (row.delta.totalCoreCells() == 0) continue;
        rows.push_back(&row);
    }
    if (rows.empty()) {
        return false;
    }
    std::sort(rows.begin(), rows.end(), [](const auto* a, const auto* b) {
        if (a->objectId != b->objectId) return a->objectId < b->objectId;
        if (a->partitionId != b->partitionId) return a->partitionId < b->partitionId;
        return a->lodId < b->lodId;
    });

    const auto isBoolCompatible = [](const auto& values) {
        for (std::int32_t value : values) {
            if (value != 0 && value != 1) {
                return false;
            }
        }
        return true;
    };
    for (const auto* row : rows) {
        if (!isBoolCompatible(row->delta.facetSymbols) ||
            !isBoolCompatible(row->delta.triangulationEdgeSymbols) ||
            !isBoolCompatible(row->delta.vertexSymbols)) {
            std::cerr << "[Columnar] paper symbol table contains non-bool "
                      << "F/E/V codes; skip lossy legacy bool-list import"
                      << std::endl;
            return false;
        }
    }

    symbolData_.clear();
    rcBlob_.clear();
    rcBlobOffset_ = 0;
    rcBlobInitialized_ = false;
    codecVersion_ = 1;

    for (const auto* row : rows) {
        GroupSymbolData data;
        data.groupId = row->partitionId < 0 ? 0 : row->partitionId;
        data.roundId = row->lodId < 0 ? 0 : row->lodId;

        data.facetSymbols.reserve(row->delta.facetSymbols.size());
        for (std::int32_t value : row->delta.facetSymbols) {
            data.facetSymbols.push_back(value != 0);
        }

        data.edgeSymbols.reserve(row->delta.triangulationEdgeSymbols.size());
        for (std::int32_t value : row->delta.triangulationEdgeSymbols) {
            data.edgeSymbols.push_back(value != 0);
        }

        data.vertexSymbols.reserve(row->delta.vertexSymbols.size());
        for (std::int32_t value : row->delta.vertexSymbols) {
            data.vertexSymbols.push_back(value != 0);
        }

        data.pointX.reserve(row->delta.removedVertexCoords.size());
        data.pointY.reserve(row->delta.removedVertexCoords.size());
        data.pointZ.reserve(row->delta.removedVertexCoords.size());
        for (const auto& p : row->delta.removedVertexCoords) {
            data.pointX.push_back(static_cast<float>(p.x()));
            data.pointY.push_back(static_cast<float>(p.y()));
            data.pointZ.push_back(static_cast<float>(p.z()));
        }
        if (data.pointX.empty() && !row->delta.removedVertexResiduals.empty()) {
            data.pointQX.reserve(row->delta.removedVertexResiduals.size());
            data.pointQY.reserve(row->delta.removedVertexResiduals.size());
            data.pointQZ.reserve(row->delta.removedVertexResiduals.size());
            for (const auto& p : row->delta.removedVertexResiduals) {
                data.pointQX.push_back(p.x());
                data.pointQY.push_back(p.y());
                data.pointQZ.push_back(p.z());
            }
        }

        data.vertexPointX.reserve(row->delta.shiftedVertexCoords.size());
        data.vertexPointY.reserve(row->delta.shiftedVertexCoords.size());
        data.vertexPointZ.reserve(row->delta.shiftedVertexCoords.size());
        for (const auto& p : row->delta.shiftedVertexCoords) {
            data.vertexPointX.push_back(static_cast<float>(p.x()));
            data.vertexPointY.push_back(static_cast<float>(p.y()));
            data.vertexPointZ.push_back(static_cast<float>(p.z()));
        }
        if (data.vertexPointX.empty() && !row->delta.shiftedVertexResiduals.empty()) {
            data.vertexPointQX.reserve(row->delta.shiftedVertexResiduals.size());
            data.vertexPointQY.reserve(row->delta.shiftedVertexResiduals.size());
            data.vertexPointQZ.reserve(row->delta.shiftedVertexResiduals.size());
            for (const auto& p : row->delta.shiftedVertexResiduals) {
                data.vertexPointQX.push_back(p.x());
                data.vertexPointQY.push_back(p.y());
                data.vertexPointQZ.push_back(p.z());
            }
        }

        symbolData_.push_back(std::move(data));
    }

    if (parquetV2VerboseLogs()) {
        std::cout << "[Columnar] imported paper symbol table: "
                  << symbolData_.size() << " records" << std::endl;
    }
    return !symbolData_.empty();
}

// ============================================================================
// 读取 boundary 表
// ============================================================================

void ColumnarDeserializeOperator::readBoundaryTable(const std::string& path) {
    if (!std::filesystem::exists(path)) return;

    auto table = readParquetFile(path);
    if (!table || table->num_rows() == 0) return;

    boundaryGraph_ = std::make_shared<Graph>();

    auto gaCol = table->GetColumnByName(pmsf::kParquetGroupAColumn);
    auto gbCol = table->GetColumnByName(pmsf::kParquetGroupBColumn);
    auto stCol = table->GetColumnByName(pmsf::kParquetBoundaryStColumn);
    auto edCol = table->GetColumnByName(pmsf::kParquetBoundaryEdColumn);
    auto stopCol = table->GetColumnByName(pmsf::kParquetBoundaryStopColumn);
    auto schemeCol = table->GetColumnByName("boundary_scheme");
    if (!gaCol || !gbCol || !stCol) {
        std::cerr << "[Columnar] boundary 表列不完整，跳过解析" << std::endl;
        return;
    }
    BoundaryGraphScheme scheme = BoundaryGraphScheme::ThreePoint;
    if (schemeCol && schemeCol->length() > 0) {
        auto arr = std::static_pointer_cast<arrow::Int32Array>(schemeCol->chunk(0));
        scheme = static_cast<BoundaryGraphScheme>(arr->Value(0));
    }
    boundaryGraph_->setBoundaryScheme(scheme);

    for (int64_t row = 0; row < table->num_rows(); row++) {
        auto gaChunk = std::static_pointer_cast<arrow::Int32Array>(gaCol->chunk(0));
        auto gbChunk = std::static_pointer_cast<arrow::Int32Array>(gbCol->chunk(0));
        int groupA = gaChunk->Value(row);
        int groupB = gbChunk->Value(row);

        auto stVals = extractInt32List(stCol->chunk(0), row);
        auto edVals = edCol ? extractInt32List(edCol->chunk(0), row) : std::vector<int32_t>{};
        auto stopVals = stopCol ? extractInt32List(stopCol->chunk(0), row) : std::vector<int32_t>{};

        if (scheme == BoundaryGraphScheme::StartOnly) {
            for (int32_t st : stVals) {
                boundaryGraph_->addHEdge(groupA, groupB, Node(st, -1, -1));
            }
        } else {
            for (size_t i = 0; i < stVals.size() && i < edVals.size() && i < stopVals.size(); i++) {
                boundaryGraph_->addHEdge(groupA, groupB, Node(stVals[i], edVals[i], stopVals[i]));
            }
        }
    }

    if (std::getenv("PMSF_BOUNDARY_DEBUG") || parquetV2VerboseLogs()) {
        size_t nodes = 0;
        for (auto& [ga, mp] : boundaryGraph_->graph())
            for (auto& [gb, arr] : mp) nodes += arr.size();
        std::cout << "[boundary-read] scheme="
                  << static_cast<int>(boundaryGraph_->boundaryScheme())
                  << " rows=" << table->num_rows() << " nodes=" << nodes << std::endl;
        std::cout << "[Columnar] boundary: 解析完成" << std::endl;
    }
}

// ============================================================================
// 反序列化基础网格
// ============================================================================

std::shared_ptr<MCGAL::Mesh> ColumnarDeserializeOperator::deserializeBaseMesh() {
    if (baseMeshData_.empty()) {
        std::cerr << "[Columnar] 无基础网格数据" << std::endl;
        return nullptr;
    }

    int totalVertices = 0;
    int totalFaces = 0;
    for (auto& data : baseMeshData_) {
        totalVertices += 2 + data.numVertices;
        totalFaces += data.numFaces;
    }

    const int envVertexPool = MCGAL::ContextPool::poolSizeFromEnv("PMSF_POOL_V", 0);
    const int envHalfedgePool = MCGAL::ContextPool::poolSizeFromEnv("PMSF_POOL_H", 0);
    const int envFacePool = MCGAL::ContextPool::poolSizeFromEnv("PMSF_POOL_F", 0);
    int estimatedVertexPool = MCGAL::ContextPool::poolSizeFromEnv("PMSF_POOL_V", 150000);
    int estimatedHalfedgePool = MCGAL::ContextPool::poolSizeFromEnv("PMSF_POOL_H", 2500000);
    int estimatedFacePool = MCGAL::ContextPool::poolSizeFromEnv("PMSF_POOL_F", 150000);
    const bool segmentedDefaultPartitionedBase =
        header_.isEnableSegmentation() &&
        pmsf::paperBlobLayoutIsSegmentedRangeCodedPred(blobLayout_);
    const bool hasOriginalMeshSize =
        !segmentedDefaultPartitionedBase &&
        origTotalVertices_ >= 1024 &&
        origTotalFaces_ >= 1024;
    if (hasOriginalMeshSize) {
        const int finalVertexEstimate = std::max(origTotalVertices_, totalVertices);
        const int finalFaceEstimate = clampPoolEstimate(
            std::max<long long>(std::max(origTotalFaces_, totalFaces),
                                finalVertexEstimate > 0
                                    ? static_cast<long long>(finalVertexEstimate) * 2ll
                                    : 0ll),
            std::max(origTotalFaces_, totalFaces));
        estimatedVertexPool = clampPoolEstimate(
            std::max<long long>(
                static_cast<long long>(envVertexPool),
                std::max<long long>(static_cast<long long>(totalVertices) * 5ll,
                                    static_cast<long long>(finalVertexEstimate) * 2ll +
                                        static_cast<long long>(totalVertices) + 1024ll)),
            estimatedVertexPool);
        estimatedHalfedgePool = clampPoolEstimate(
            std::max<long long>(
                static_cast<long long>(envHalfedgePool),
                std::max<long long>(static_cast<long long>(totalFaces) * 15ll,
                                    static_cast<long long>(finalFaceEstimate) * 9ll +
                                        static_cast<long long>(totalFaces) * 3ll + 1024ll)),
            estimatedHalfedgePool);
        estimatedFacePool = clampPoolEstimate(
            std::max<long long>(
                static_cast<long long>(envFacePool),
                std::max<long long>(static_cast<long long>(totalFaces) * 5ll,
                                    static_cast<long long>(finalFaceEstimate) * 3ll +
                                        static_cast<long long>(totalFaces) + 1024ll)),
            estimatedFacePool);
    }

    if (!MCGAL::contextPool.hasPool(DEFAULT_MESH_ID)) {
        if (parquetV2VerboseLogs()) {
            std::cerr << "[Columnar] context pool estimate: original_vertices="
                      << origTotalVertices_
                      << " original_faces=" << origTotalFaces_
                      << " base_vertices=" << totalVertices
                      << " base_faces=" << totalFaces
                      << " use_original_size=" << (hasOriginalMeshSize ? 1 : 0)
                      << " pool_v=" << estimatedVertexPool
                      << " pool_h=" << estimatedHalfedgePool
                      << " pool_f=" << estimatedFacePool
                      << "\n";
        }
        MCGAL::contextPool.initPoolSize(DEFAULT_MESH_ID + 1);
        const int poolId = MCGAL::contextPool.registerPool(
            estimatedVertexPool, estimatedHalfedgePool, estimatedFacePool);
        if (poolId != DEFAULT_MESH_ID) {
            throw std::runtime_error("Columnar default context pool id mismatch");
        }
    }
    mesh_ = std::make_shared<MCGAL::Mesh>();
    mesh_->setMeshId(DEFAULT_MESH_ID);
    mesh_->enableQuantization = header_.isEnableQuantization();
    mesh_->vertices().reserve(static_cast<std::size_t>(totalVertices));
    mesh_->faces().reserve(static_cast<std::size_t>(totalFaces));

    if (header_.isEnableQuantization()) {
        mesh_->i_nbQuantBits = quantBits_;
        mesh_->bboxMin = MCGAL::Point(bboxMinX_, bboxMinY_, bboxMinZ_);
        mesh_->f_quantStep = quantStep_;
    }

    bool isSegmented = sharedBaseMeshCoordinates_ ||
                       (header_.isEnableSegmentation() && baseMeshData_.size() > 1);
    std::vector<MCGAL::Vertex*> faceVerticesScratch;
    auto resetFaceVertices = [&](int degree) -> std::vector<MCGAL::Vertex*>& {
        faceVerticesScratch.clear();
        faceVerticesScratch.reserve(static_cast<std::size_t>(degree));
        return faceVerticesScratch;
    };
    auto addBaseFace = [&](const std::vector<MCGAL::Vertex*>& vts, bool openFace, int groupId) {
        if (vts.size() < (openFace ? 2u : 3u)) return;
        MCGAL::Facet* face = MCGAL::contextPool.allocateFaceFromPool(DEFAULT_MESH_ID);
        face->setGroupId(groupId);
        MCGAL::Halfedge* head = nullptr;
        MCGAL::Halfedge* prev = nullptr;
        const std::size_t edgeCount = openFace ? vts.size() - 1 : vts.size();
        for (std::size_t i = 0; i < edgeCount; ++i) {
            MCGAL::Vertex* from = vts[i];
            MCGAL::Vertex* to = vts[(i + 1) % vts.size()];
            MCGAL::Halfedge* h = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(
                DEFAULT_MESH_ID, from, to);
            if (from && from->proxyHalfedge() == nullptr) {
                from->setProxyHalfedge(h);
            }
            if (from) {
                from->halfedges().push_back(h);
            }
            if (to) {
                for (MCGAL::Halfedge* candidate : to->halfedges()) {
                    if (!candidate || candidate == h || candidate->isRemoved()) continue;
                    if (candidate->end_vertex() == from && candidate->opposite() == nullptr) {
                        candidate->setOpposite(h);
                        h->setOpposite(candidate);
                        break;
                    }
                }
            }
            h->setFace(face);
            if (!head) head = h;
            if (prev) prev->setNext(h);
            prev = h;
        }
        if (prev) prev->setNext(openFace ? nullptr : head);
        face->setProxyHalfedge(head);
        face->reset_without_init(head);
        mesh_->add_face(face);
    };

    // ===================== 关键正确性：量化坐标反量化 =====================
    // 非 Parquet 路径 MCGAL::readBaseMeshWithQuantization 始终是这样做的：
    //   pos = bboxMin + (q + 0.5) * quantStep
    // 得到"量化 snap"后的浮点坐标。如果我们直接用 data.seedV0X/vertexX 原始浮点坐标去建 Mesh，
    // 编码端 collect 时的 mesh_->floatPosToInt(point) 与解码端重新 floatPosToInt 会相差 1 个量化单位，
    // 直接导致 PPMC / QEM 等依赖 "量化 BFS + 整数残差" 的 range-coder 编解码完全错位。
    // 因此在 quantization 开启时 **一律优先采用量化坐标反量化得到 snap float**。
    auto sharedCoordPoint = [&](int32_t coordId) -> MCGAL::Point {
        if (coordId < 0) return MCGAL::Point(0, 0, 0);
        const size_t i = static_cast<size_t>(coordId);
        if (header_.isEnableQuantization() && !sharedVertexQX_.empty()) {
            return MCGAL::Point(bboxMinX_ + (sharedVertexQX_[i] + 0.5f) * quantStep_,
                                bboxMinY_ + (sharedVertexQY_[i] + 0.5f) * quantStep_,
                                bboxMinZ_ + (sharedVertexQZ_[i] + 0.5f) * quantStep_);
        }
        if (!sharedVertexX_.empty()) {
            return MCGAL::Point(sharedVertexX_[i], sharedVertexY_[i], sharedVertexZ_[i]);
        }
        return MCGAL::Point(0, 0, 0);
    };
    auto seedPointSnap = [&](const GroupBaseMeshData& d, bool v0) -> MCGAL::Point {
        if (sharedBaseMeshCoordinates_) {
            const int32_t coordId = v0 ? d.seedV0CoordId : d.seedV1CoordId;
            if (coordId >= 0) return sharedCoordPoint(coordId);
        }
        if (header_.isEnableQuantization()) {
            int32_t qx = v0 ? d.seedV0QX : d.seedV1QX;
            int32_t qy = v0 ? d.seedV0QY : d.seedV1QY;
            int32_t qz = v0 ? d.seedV0QZ : d.seedV1QZ;
            return MCGAL::Point(bboxMinX_ + (qx + 0.5f) * quantStep_,
                                bboxMinY_ + (qy + 0.5f) * quantStep_,
                                bboxMinZ_ + (qz + 0.5f) * quantStep_);
        }
        return MCGAL::Point(v0 ? d.seedV0X : d.seedV1X,
                            v0 ? d.seedV0Y : d.seedV1Y,
                            v0 ? d.seedV0Z : d.seedV1Z);
    };
    auto vertexPointSnap = [&](const GroupBaseMeshData& d, size_t i) -> MCGAL::Point {
        if (header_.isEnableQuantization() && !d.vertexQX.empty()) {
            return MCGAL::Point(bboxMinX_ + (d.vertexQX[i] + 0.5f) * quantStep_,
                                bboxMinY_ + (d.vertexQY[i] + 0.5f) * quantStep_,
                                bboxMinZ_ + (d.vertexQZ[i] + 0.5f) * quantStep_);
        }
        return MCGAL::Point(d.vertexX[i], d.vertexY[i], d.vertexZ[i]);
    };

    segmentedSeedVertices_.clear();
    if (isSegmented) {
        if (sharedBaseMeshCoordinates_) {
            seeds_.clear();
            std::vector<std::pair<MCGAL::Vertex*, MCGAL::Vertex*>> seedVertices;
            seedVertices.reserve(baseMeshData_.size());
            std::unordered_map<int32_t, int32_t> sharedCoordToTopo;
            const bool separateSegmentTopology =
                pmsf::paperBlobLayoutIsSegmentedRangeCodedPred(blobLayout_) &&
                !segmentedDefaultSharedTopology();

            for (auto& data : baseMeshData_) {
                if (data.faceVerticesAreCoordIds) {
                    std::unordered_map<int32_t, int32_t> localCoordToTopo;
                    auto& coordToTopo =
                        separateSegmentTopology ? localCoordToTopo : sharedCoordToTopo;
                    auto addCoordVertex = [&](int32_t coordId) {
                        auto it = coordToTopo.find(coordId);
                        if (it != coordToTopo.end()) return it->second;
                        int32_t topoId = static_cast<int32_t>(mesh_->size_of_vertices());
                        MCGAL::Point p = sharedCoordPoint(coordId);
                        MCGAL::Vertex* v = MCGAL::contextPool.allocateVertexFromPool(DEFAULT_MESH_ID, p);
                        v->setGroupId(data.groupId);
                        mesh_->add_vertex(v);
                        coordToTopo.emplace(coordId, topoId);
                        return topoId;
                    };

                    int32_t seed0Topo = addCoordVertex(data.seedV0CoordId);
                    int32_t seed1Topo = addCoordVertex(data.seedV1CoordId);
                    seedVertices.emplace_back(mesh_->get_vertex(seed0Topo),
                                              mesh_->get_vertex(seed1Topo));

                    size_t vidx = 0;
                    for (size_t i = 0; i < data.faceDegrees.size(); i++) {
                        int storedDegree = data.faceDegrees[i];
                        bool openFace = storedDegree < 0;
                        int degree = std::abs(storedDegree);
                        std::vector<MCGAL::Vertex*>& vts = resetFaceVertices(degree);
                        for (int j = 0; j < degree; j++) {
                            int32_t coordId = data.faceVertices[vidx++];
                            vts.push_back(mesh_->get_vertex(addCoordVertex(coordId)));
                        }
                        addBaseFace(vts, openFace, data.groupId);
                    }
                    continue;
                }

                std::vector<int32_t> localToTopo;
                auto addLocalVertex = [&](int32_t coordId) {
                    int32_t topoId = static_cast<int32_t>(mesh_->size_of_vertices());
                    MCGAL::Point p = sharedCoordPoint(coordId);
                    MCGAL::Vertex* v = MCGAL::contextPool.allocateVertexFromPool(DEFAULT_MESH_ID, p);
                    v->setGroupId(data.groupId);
                    mesh_->add_vertex(v);
                    localToTopo.push_back(topoId);
                    return topoId;
                };

                int32_t seed0Topo = addLocalVertex(data.seedV0CoordId);
                int32_t seed1Topo = addLocalVertex(data.seedV1CoordId);
                seedVertices.emplace_back(mesh_->get_vertex(seed0Topo),
                                          mesh_->get_vertex(seed1Topo));

                for (int32_t coordId : data.vertexCoordIds) {
                    (void)addLocalVertex(coordId);
                }

                for (int32_t& localId : data.faceVertices) {
                    if (localId >= 0 && static_cast<size_t>(localId) < localToTopo.size()) {
                        localId = localToTopo[localId];
                    }
                }

                size_t vidx = 0;
                for (size_t i = 0; i < data.faceDegrees.size(); i++) {
                    int storedDegree = data.faceDegrees[i];
                    bool openFace = storedDegree < 0;
                    int degree = std::abs(storedDegree);
                    std::vector<MCGAL::Vertex*>& vts = resetFaceVertices(degree);
                    for (int j = 0; j < degree; j++) {
                        vts.push_back(mesh_->get_vertex(data.faceVertices[vidx++]));
                    }
                    addBaseFace(vts, openFace, data.groupId);
                }
            }

            segmentedSeedVertices_ = seedVertices;
            for (auto [v0, v1] : seedVertices) {
                for (MCGAL::Halfedge* hit : pmsf::boundedRing(v0)) {
                    if (hit->end_vertex() == v1) {
                        seeds_.push_back(hit);
                        break;
                    }
                }
            }
            if (!seeds_.empty()) {
                seed_ = seeds_[0];
                seedV0_ = seedVertices[0].first;
                seedV1_ = seedVertices[0].second;
            }

        } else {
        // 先添加所有 seed 顶点
        for (auto& data : baseMeshData_) {
            MCGAL::Point p0 = seedPointSnap(data, true);
            MCGAL::Point p1 = seedPointSnap(data, false);
            MCGAL::Vertex* v0 = MCGAL::contextPool.allocateVertexFromPool(DEFAULT_MESH_ID, p0);
            MCGAL::Vertex* v1 = MCGAL::contextPool.allocateVertexFromPool(DEFAULT_MESH_ID, p1);
            v0->setGroupId(data.groupId);
            v1->setGroupId(data.groupId);
            mesh_->add_vertex(v0);
            mesh_->add_vertex(v1);
        }

        for (auto& data : baseMeshData_) {
            size_t nv = sharedBaseMeshCoordinates_
                        ? static_cast<size_t>(data.numVertices)
                        : (header_.isEnableQuantization() && !data.vertexQX.empty()
                               ? data.vertexQX.size()
                               : data.vertexX.size());
            for (size_t i = 0; i < nv; i++) {
                MCGAL::Point p = vertexPointSnap(data, i);
                MCGAL::Vertex* vt = MCGAL::contextPool.allocateVertexFromPool(DEFAULT_MESH_ID, p);
                vt->setGroupId(data.groupId);
                mesh_->add_vertex(vt);
            }
        }

        for (auto& data : baseMeshData_) {
            size_t vidx = 0;
            for (size_t i = 0; i < data.faceDegrees.size(); i++) {
                int storedDegree = data.faceDegrees[i];
                bool openFace = storedDegree < 0;
                int degree = std::abs(storedDegree);
                std::vector<MCGAL::Vertex*>& vts = resetFaceVertices(degree);
                for (int j = 0; j < degree; j++) {
                    vts.push_back(mesh_->get_vertex(data.faceVertices[vidx++]));
                }
                // 9 算法（含 PCLTTM Tag Phase 2）统一走默认 reset(vs)。
                // PCLTTM Tag 在 collect/collectFromDecoded 阶段会调
                // normalize_proxy_to_min_position_vertex 把 proxyHalfedge
                // 重置到 vertex 量化坐标字典序最小的入口。
                addBaseFace(vts, openFace, data.groupId);
            }
        }

        seeds_.clear();
        segmentedSeedVertices_.clear();
        segmentedSeedVertices_.reserve(baseMeshData_.size());
        for (size_t gid = 0; gid < baseMeshData_.size(); gid++) {
            MCGAL::Vertex* v0 = mesh_->get_vertex(gid * 2);
            MCGAL::Vertex* v1 = mesh_->get_vertex(gid * 2 + 1);
            segmentedSeedVertices_.emplace_back(v0, v1);
            for (MCGAL::Halfedge* hit : pmsf::boundedRing(v0)) {
                if (hit->end_vertex() == v1) {
                    seeds_.push_back(hit);
                    break;
                }
            }
        }
        if (!seeds_.empty()) {
            seed_ = seeds_[0];
            // P3 对齐：记录首个 seed 两端点，供 collectFromDecoded 每轮重新查找。
            //   （分段模式下每个 group 各自持有 seed；此处仅记录首个以覆盖非分段降级。）
            if (!baseMeshData_.empty()) {
                seedV0_ = mesh_->get_vertex(0);
                seedV1_ = mesh_->get_vertex(1);
            }
        }
        }

    } else {
        auto& data = baseMeshData_[0];

        if (pmsf::paperBlobLayoutUsesSeedSnapshotBase(blobLayout_)) {
            std::deque<MCGAL::Point> pointDeque;
            std::deque<uint32_t*> faceDeque;
            pointDeque.push_back(seedPointSnap(data, true));
            pointDeque.push_back(seedPointSnap(data, false));

            size_t nv = sharedBaseMeshCoordinates_
                        ? static_cast<size_t>(data.numVertices)
                        : (header_.isEnableQuantization() && !data.vertexQX.empty()
                               ? data.vertexQX.size()
                               : data.vertexX.size());
            for (size_t i = 0; i < nv; i++) {
                pointDeque.push_back(vertexPointSnap(data, i));
            }

            size_t vidx = 0;
            for (size_t i = 0; i < data.faceDegrees.size(); i++) {
                int degree = std::abs(data.faceDegrees[i]);
                uint32_t* f = new uint32_t[static_cast<size_t>(degree) + 1u];
                f[0] = static_cast<uint32_t>(degree);
                for (int j = 0; j < degree; j++) {
                    f[j + 1] = static_cast<uint32_t>(data.faceVertices[vidx++]);
                }
                faceDeque.push_back(f);
            }

            mesh_ = MCGAL::buildFromBufferNonTri(DEFAULT_MESH_ID, &pointDeque, &faceDeque);
            for (uint32_t* f : faceDeque) {
                delete[] f;
            }
            mesh_->enableQuantization = header_.isEnableQuantization();
            if (header_.isEnableQuantization()) {
                mesh_->i_nbQuantBits = quantBits_;
                mesh_->bboxMin = MCGAL::Point(bboxMinX_, bboxMinY_, bboxMinZ_);
                mesh_->f_quantStep = quantStep_;
            }
            for (MCGAL::Vertex* v : mesh_->vertices()) {
                if (v) v->setGroupId(data.groupId);
            }
            for (MCGAL::Facet* f : mesh_->faces()) {
                if (f) f->setGroupId(data.groupId);
            }
        } else {
            MCGAL::Point p0 = seedPointSnap(data, true);
            MCGAL::Point p1 = seedPointSnap(data, false);
            MCGAL::Vertex* sv0 = MCGAL::contextPool.allocateVertexFromPool(DEFAULT_MESH_ID, p0);
            MCGAL::Vertex* sv1 = MCGAL::contextPool.allocateVertexFromPool(DEFAULT_MESH_ID, p1);
            mesh_->add_vertex(sv0);
            mesh_->add_vertex(sv1);

            size_t nv = sharedBaseMeshCoordinates_
                        ? static_cast<size_t>(data.numVertices)
                        : (header_.isEnableQuantization() && !data.vertexQX.empty()
                               ? data.vertexQX.size()
                               : data.vertexX.size());
            for (size_t i = 0; i < nv; i++) {
                MCGAL::Point p = vertexPointSnap(data, i);
                MCGAL::Vertex* vt = MCGAL::contextPool.allocateVertexFromPool(DEFAULT_MESH_ID, p);
                mesh_->add_vertex(vt);
            }

            size_t vidx = 0;
            for (size_t i = 0; i < data.faceDegrees.size(); i++) {
                int storedDegree = data.faceDegrees[i];
                bool openFace = storedDegree < 0;
                int degree = std::abs(storedDegree);
                std::vector<MCGAL::Vertex*>& vts = resetFaceVertices(degree);
                for (int j = 0; j < degree; j++) {
                    vts.push_back(mesh_->get_vertex(data.faceVertices[vidx++]));
                }
                addBaseFace(vts, openFace, data.groupId);
            }
        }

        MCGAL::Vertex* v0 = mesh_->vertices()[0];
        MCGAL::Vertex* v1 = mesh_->vertices()[1];
        for (MCGAL::Halfedge* hit : pmsf::boundedRing(v0)) {
            if (hit->end_vertex() == v1) {
                seed_ = hit;
                seeds_ = {seed_};
                break;
            }
        }
        // P3 对齐：记录 seed 两端点，供每轮重新查找。
        seedV0_ = v0;
        seedV1_ = v1;
    }

    reconstructOperator_->init(mesh_);

    if (parquetV2VerboseLogs()) {
        std::cout << "[Columnar] 基础网格重建: 顶点=" << mesh_->size_of_vertices()
                  << " 面片=" << mesh_->size_of_facets()
                  << " 分区=" << baseMeshData_.size() << std::endl;
    }

    // ========== V2 BPV 优化路径：初始化 symbolReadOperator 的 buffer/offsets ==========
    // rc_blob 的字节内容 **逐字节等价** 于非 Parquet 版 collector::exportToBuffer*() 的 output。
    // 解码时按 blob_layout 字段调度到对称的非 Parquet 解码入口：
    //   * RANGE_CODED_PRED → OptimizedSymbolReadOperator::importFromRangeCodedBuffer
    //   * BASIC            → BasicSymbolReadOperator::collect 按轮 (Basic layout)
    //   * CODEC_003          → PPMCSymbolReadOperator::collect 按轮 (RC stream)
    //   * OPTIMIZED_PLAIN  → OptimizedSymbolReadOperator::collect 按轮 (4 段位流)
    if (codecVersion_ >= 2 && symbolReadOperator_) {
        reconstructOperator_->setEnablePrediction(header_.isEnablePrediction());
        // 让 SymbolReadOperator 拿到完整 header（它里面会判断 isEnableAdjustment / isEnablePrediction 等）
        symbolReadOperator_->header_ = header_;
        if (auto* ppmcReader = dynamic_cast<PPMCSymbolReadOperator*>(symbolReadOperator_.get())) {
            ppmcReader->setMesh(mesh_);
            ppmcReader->setEnablePrediction(header_.isEnablePrediction());
            ppmcReader->setConnectivityPredictionFaces(header_.isEnableConnectivityPredictionFaces());
            ppmcReader->setConnectivityPredictionEdges(header_.isEnableConnectivityPredictionEdges());
            ppmcReader->setUseTriangleMeshConnectivityPrediction(header_.isEnableTriangleMeshBalance());
        }
        if (auto* optimizedReader =
                dynamic_cast<OptimizedSymbolReadOperator*>(symbolReadOperator_.get())) {
            optimizedReader->setMesh(mesh_);
        }
        // codec_version=3 触发 per-edge 连通性流解码路径；v2 仍使用 legacy per-halfedge。
        symbolReadOperator_->setCodecVersion(codecVersion_);
        if (header_.isEnableQuantization()) {
            symbolReadOperator_->enableQuantization(
                MCGAL::Point(bboxMinX_, bboxMinY_, bboxMinZ_), quantStep_);
        }

        rcBlobOffset_ = 0;
        if (paperV2Typed_) {
            // The typed symbol context was installed by initPaperV2Typed().
            // There is no rc_blob to import, even if the original layout uses it.
            symbolReadOperator_->init({0, 0, 0},header_);
            return mesh_;
        }

        // BlobLayout 编码：1=RANGE_CODED_PRED, 2=BASIC, 3=CODEC_003, 4=OPTIMIZED_PLAIN
        // 兼容旧文件（layout=0）：根据 enablePrediction_ 推测为 RANGE_CODED_PRED 或 BASIC
        int layout = blobLayout_;
        if (pmsf::paperBlobLayoutIsUnknown(layout)) {
            layout = pmsf::paperBlobLayoutLegacyFallback(enablePrediction_);
        }

        if (layout == pmsf::kPaperBlobLayoutCanonical) {
            // Canonical symbols were already loaded into the reader's decoded
            // queues by readColumnarSymbolsTablesV4(). Just finish setup and
            // tell deserializeOneRound() to use collectFromDecoded().
            // When the payload stores RD-prediction residuals, keep prediction
            // enabled so the reconstruct operator adds them to the barycenter;
            // otherwise disable prediction because the payload stores absolute
            // quantized coordinates.
            reconstructOperator_->setEnablePrediction(header_.isEnablePrediction());
            symbolReadOperator_->init({0, 0, 0}, header_);
            rcBlobInitialized_ = true;
            return mesh_;
        }

        if (layout == pmsf::kPaperBlobLayoutConquestRC) {
            // P13.4 (B)②：valence-conquest 原生 codec（blob_layout = 15）。
            // 整条流**一次性**装填进 reader（`paperBlobLayoutUsesDecodedQueues(15)`
            // 为真 ⇒ 每轮走 `collectFromDecoded`）；真正的解码由
            // `ConquestReconstructOperator` 在 `reconstruct` 里调
            // `pmsf::decodeLayer` 完成 —— 解码实体只有那一处。
            //
            // 量化网格**不从这里取**：它随流走（`ConquestBlobFormat.h`）。
            // 蓝图 §54 的教训：量化器的定义经有损中间表示会伪装成"被测对象错了"。
            symbolReadOperator_->init({0, 0, 0}, header_);
            if (!rcBlob_.empty()) {
                size_t localOff = 0;
                // ⚠️ **不解引用 `mesh_`**：在 `deserializeBaseMesh()` 里走到本分支时
                // `mesh_` 可能仍为 null（实测入口处 mesh_=(nil)）。而量化位数本 codec
                // 根本不用 —— 量化网格随流走（见 ConquestBlobFormat.h），
                // 传 0 是无害且正确的。这一行原先是 `mesh_->i_nbQuantBits`，
                // 在 null 上解引用会段错。
                if (auto* cq = dynamic_cast<pmsf::ConquestSymbolReadOperator*>(symbolReadOperator_.get()))
                    cq->setBufferSize(rcBlob_.size());
                symbolReadOperator_->importFromRangeCodedBuffer(rcBlob_.data(), localOff, 0);
                // 诊断（env 门控，默认零输出）：把**流里自带的**量化网格按 %.17g 打出，
                // 供外部对拍工具用**同一把尺子**比网格。见 ConquestBlobFormat.h。
                if (auto* cq = dynamic_cast<pmsf::ConquestSymbolReadOperator*>(symbolReadOperator_.get())) {
                }
            }
            rcBlobInitialized_ = true;
            return mesh_;
        }

        if (pmsf::paperBlobLayoutIsSegmentedPPMC(layout)) {
            size_t off = 0;
            auto readI32 = [&](int& out) -> bool {
                if (off + sizeof(int) > rcBlob_.size()) return false;
                std::memcpy(&out, rcBlob_.data() + off, sizeof(int));
                off += sizeof(int);
                return true;
            };

            if (!readI32(segmentedGroupNumber_) || !readI32(segmentedRoundNumber_) ||
                segmentedGroupNumber_ < 0 || segmentedRoundNumber_ < 0) {
                std::cerr << "[Columnar] segmented PPMC blob header invalid" << std::endl;
                return mesh_;
            }

            segmentedGroupStart_.assign(segmentedGroupNumber_, 0);
            for (int group = 0; group < segmentedGroupNumber_; ++group) {
                int v = 0;
                if (!readI32(v)) {
                    std::cerr << "[Columnar] segmented PPMC groupStart table truncated" << std::endl;
                    return mesh_;
                }
                segmentedGroupStart_[group] = static_cast<size_t>(std::max(0, v));
            }

            segmentedGroupRoundEnd_.assign(
                segmentedGroupNumber_,
                std::vector<size_t>(segmentedRoundNumber_, 0));
            for (int group = 0; group < segmentedGroupNumber_; ++group) {
                for (int round = 0; round < segmentedRoundNumber_; ++round) {
                    int v = 0;
                    if (!readI32(v)) {
                        std::cerr << "[Columnar] segmented PPMC roundEnd table truncated" << std::endl;
                        return mesh_;
                    }
                    segmentedGroupRoundEnd_[group][round] = static_cast<size_t>(std::max(0, v));
                }
            }

            segmentedPayloadStart_ = off;
            for (int group = 0; group < segmentedGroupNumber_; ++group) {
                segmentedGroupStart_[group] += segmentedPayloadStart_;
                for (int round = 0; round < segmentedRoundNumber_; ++round) {
                    segmentedGroupRoundEnd_[group][round] += segmentedGroupStart_[group];
                }
            }
            segmentedGroupCurrentRound_.assign(segmentedGroupNumber_, 0);
            symbolReadOperator_->init({0}, header_);
            return mesh_;
        }

        if (pmsf::paperBlobLayoutIsSegmentedRangeCodedPred(layout)) {
            // P18.2 PHASE59 物品 3 / PHASE60 物品 3：**分区产物的解码镜像**。
            //
            // 机制（前缀解析 + 基网 facet 通道 + 每组 seed 重算 + 每组 reader）
            // 已抽到 `pmsf::SegmentedRangeCodedDecode`，与二进制后端
            // `OptimizedBasicDeserializeOperator` 共用同一份实现 —— 编码端两条
            // 后端写的是**同一段** layout-32 blob，镜像也只该有一份。
            //
            // seed 通道是**基网 vid**（不是 poolId，见附 20），故本路径消费它。
            pmsf::SegmentedRangeCodedDecode::Context segCtx;
            segCtx.header = header_;
            segCtx.mesh = mesh_;
            segCtx.reconstructor = reconstructOperator_.get();
            segCtx.codecVersion = codecVersion_;
            segCtx.bboxMin = MCGAL::Point(bboxMinX_, bboxMinY_, bboxMinZ_);
            segCtx.quantStep = quantStep_;
            if (!segmentedDecode_.importBlob(rcBlob_.data(), rcBlob_.size(), segCtx)) {
                return mesh_;
            }
            // 每分区 seed 回写到本类的成员：分段 PPMC 轮驱动与外部消费者按
            // 绝对 group id 索引 `seeds_` / `segmentedSeedVertices_`，与镜像
            // 内部的状态在 import 时刻**逐项相同**（镜像即原代码）。
            seeds_ = segmentedDecode_.seeds();
            segmentedSeedVertices_ = segmentedDecode_.seedVertices();
            symbolReadOperator_->init({0, 0, 0}, header_);
            return mesh_;
        }

        if (parquetV2VerboseLogs()) {
            std::cout << "[Columnar] importing rc_blob for layout=" << layout
                      << " size=" << rcBlob_.size() << std::endl;
        }
        const auto paperReaderImport = pmsf::paperImportRangeCodedBlobForLayout(
            layout,
            symbolReadOperator_.get(),
            rcBlob_.data(),
            rcBlobOffset_,
            header_.isEnableQuantization() ? mesh_->i_nbQuantBits : 0,
            header_,
            mesh_);
        if (parquetV2VerboseLogs()) {
            std::cout << "[Columnar] import layout=" << layout
                      << " handled=" << paperReaderImport.handled << std::endl;
        }
        if (paperReaderImport.handled) {
            rcBlobInitialized_ = true;
            return mesh_;
        }

        const auto simpleReaderImport = pmsf::paperImportSimpleUnpackForLayout(
            layout,
            symbolReadOperator_.get(),
            rcBlobOffset_,
            header_);
        if (simpleReaderImport.handled) {
            rcBlobInitialized_ = true;
            return mesh_;
        }

        const auto bufferTransformReaderImport =
            pmsf::paperImportBufferTransformForLayout(
                layout,
                symbolReadOperator_.get(),
                rcBlob_,
                rcBlobOffset_,
                header_);
        if (bufferTransformReaderImport.handled) {
            rcBlobInitialized_ = bufferTransformReaderImport.initialized;
            return mesh_;
        }

        if (pmsf::paperReaderLayoutClass(layout) ==
            pmsf::PMSFReaderLayoutClass::ReservedUnimplemented) {
            throw std::runtime_error(
                "[ColumnarDeserialize] BlobLayout " + std::to_string(blobLayout_) +
                " (scheme C/P/H) reserved but decoder not implemented yet "
                "(Phase 3/4/5 pending).");
        }

        switch (layout) {
            default:
                std::cerr << "[Columnar] 未知 blob_layout=" << blobLayout_ << std::endl;
                return mesh_;
        }

        rcBlobInitialized_ = true;
    }

    return mesh_;
}

// ============================================================================
// 反序列化一轮符号数据
// ============================================================================

void ColumnarDeserializeOperator::deserializeOneRound() {
    vertices_.clear();
    halfedges_.clear();
    facets_.clear();

    if (header_.isEnableQuantization()) {
        symbolReadOperator_->enableQuantization(
            MCGAL::Point(bboxMinX_, bboxMinY_, bboxMinZ_), quantStep_);
    }

    // ⚠️ P3.c 对齐（2026-05-01）：每轮 seed 刷新（原版 PPMC pushHehInit 等价）。
    //   与 PPMCDeserializeOperator::deserializeOneRound 中的 refresh 语义一致：
    //   从 seedV0_->halfedges() 倒序查找 end_vertex==seedV1_ 且 !isRemoved() 的那条。
    //   启用 connectivity prediction 后，原 seed halfedge 会被 merge_face 删除，
    //   若不 refresh，后续轮次从 stale seed 启动，表现为所有轮次"边合并=0 面片分裂=0"。
    // 以下四个的规则单源已下沉到 `pmsf::findLiveHalfedgeInMesh` 一族
    // （PHASE60 物品 3：分区镜像需要同一套 seed 搜索，见
    // impl/common/SegmentedRangeCodedDecode.h）—— 本处只做转发，语义逐字不变。
    auto findLiveHalfedge = [&](MCGAL::Vertex* from, MCGAL::Vertex* to, int groupId = -1) -> MCGAL::Halfedge* {
        return pmsf::findLiveHalfedgeInMesh(mesh_.get(), from, to, groupId);
    };

    auto findAnyLiveHalfedge = [&](int groupId = -1) -> MCGAL::Halfedge* { return pmsf::findAnyLiveHalfedgeInMesh(mesh_.get(), groupId); };

    auto findCanonicalGroupHalfedge = [&](int groupId) -> MCGAL::Halfedge* { return pmsf::findCanonicalGroupHalfedgeInMesh(mesh_.get(), groupId); };

    auto refreshSeed = [&](MCGAL::Vertex* v0, MCGAL::Vertex* v1, MCGAL::Halfedge*& outSeed, int groupId = -1) {
        const char* canonicalSeedEnv = std::getenv("PMSF_SEGMENTED_DEFAULT_CANONICAL_SEED");
        const bool canonicalSeed = canonicalSeedEnv && canonicalSeedEnv[0] != '\0' && canonicalSeedEnv[0] != '0';
        pmsf::refreshLiveSeedHalfedge(mesh_.get(), v0, v1, outSeed, groupId,
                                      canonicalSeed && pmsf::paperBlobLayoutIsSegmentedRangeCodedPred(blobLayout_));
    };

    // ========== V2 BPV 优化路径：直接复用非 Parquet 版本的逐轮消费逻辑 ==========
    if (codecVersion_ >= 2) {
        using Clock = std::chrono::steady_clock;
        const bool profile = parquetV2ProfileLogs();
        auto tpRound = Clock::now();
        auto tpCollect = tpRound;
        auto tpReconstruct = tpRound;
        double collectMs = 0.0;
        double reconstructMs = 0.0;
        // 将 header 传递给 symbolReadOperator（用于判断 isEnableAdjustment 等）
        symbolReadOperator_->header_ = header_;

        int layout = blobLayout_;
        if (pmsf::paperBlobLayoutIsUnknown(layout)) {
            layout = pmsf::paperBlobLayoutLegacyFallback(enablePrediction_);
        }

        if (pmsf::paperBlobLayoutIsSegmentedPPMC(layout)) {
            if (segmentedGroupNumber_ <= 0 || segmentedRoundNumber_ <= 0 ||
                segmentedGroupStart_.empty()) {
                std::cerr << "[Columnar] segmented PPMC offsets are not initialized" << std::endl;
                currentRound_++;
                return;
            }

            const int targetRound = currentRound_ + 1;
            int beginGroup = 0;
            int endGroup = std::min<int>(segmentedGroupNumber_, seeds_.size());
            if (targetGroupId_ >= 0) {
                beginGroup = std::min(targetGroupId_, endGroup);
                endGroup = std::min(beginGroup + 1, endGroup);
            } else if (maxGroups_ >= 0) {
                endGroup = std::min(endGroup, maxGroups_);
            }

            struct PendingSegmentedPPMC {
                int group = -1;
                int cur = -1;
                std::vector<MCGAL::Vertex*> vertices;
                std::vector<MCGAL::Halfedge*> halfedges;
                std::vector<MCGAL::Facet*> facets;
            };
            std::vector<PendingSegmentedPPMC> pending;
            if (mesh_) {
                mesh_->resetState();
            }
            auto restorePendingState = [&]() {
                for (auto& item : pending) {
                    for (MCGAL::Facet* f : item.facets) {
                        if (!f || f->isRemoved()) continue;
                        if (f->isUnsplittable()) {
                            f->resetSplittableFlag();
                        }
                        if (!f->isSplittable()) {
                            f->setSplittable();
                        }
                    }
                    for (MCGAL::Halfedge* h : item.halfedges) {
                        if (!h || h->isRemoved()) continue;
                        if (!h->isAdded()) {
                            h->setAdded();
                        }
                    }
                }
            };
            for (int group = beginGroup; group < endGroup; ++group) {
                if (group < 0 || group >= static_cast<int>(seeds_.size()) ||
                    seeds_[group] == nullptr) {
                    continue;
                }
                if (targetRound > segmentedRoundNumber_) continue;
                while (segmentedGroupCurrentRound_[group] < targetRound) {
                    const int cur = segmentedGroupCurrentRound_[group];
                    const size_t startPos = (cur == 0)
                                                ? segmentedGroupStart_[group]
                                                : segmentedGroupRoundEnd_[group][cur - 1];
                    const size_t endPos = segmentedGroupRoundEnd_[group][cur];
                    if (endPos <= startPos || startPos >= rcBlob_.size()) {
                        segmentedGroupCurrentRound_[group] = cur + 1;
                        continue;
                    }
                    if (group < static_cast<int>(segmentedSeedVertices_.size())) {
                        refreshSeed(segmentedSeedVertices_[group].first,
                                    segmentedSeedVertices_[group].second,
                                    seeds_[group],
                                    group);
                    }
                    if (seeds_[group] == nullptr) {
                        segmentedGroupCurrentRound_[group] = cur + 1;
                        continue;
                    }

	                    PendingSegmentedPPMC item;
	                    item.group = group;
	                    item.cur = cur;
	                    symbolReadOperator_->groupId = group;
	                    if (auto* ppmcReader =
	                            dynamic_cast<PPMCSymbolReadOperator*>(symbolReadOperator_.get())) {
	                        ppmcReader->setDebugOperationId(cur);
	                    }
	                    size_t localOff = startPos;
	                    try {
                        symbolReadOperator_->collect(
                            seeds_[group], rcBlob_.data(), localOff,
                            item.vertices, item.halfedges, item.facets);
                    } catch (const std::exception& e) {
                        throw std::runtime_error(
                            "[Columnar] segmented PPMC collect failed group=" +
                            std::to_string(group) + " cur=" + std::to_string(cur) +
                            ": " + e.what());
                    }
                    if (localOff > rcBlob_.size()) {
                        throw std::runtime_error(
                            "[Columnar] PPMC rc stream over-read (segmented): offset=" +
                            std::to_string(localOff) + " size=" +
                            std::to_string(rcBlob_.size()) +
                            " group=" + std::to_string(group) +
                            " cur=" + std::to_string(cur));
                    }
                    if (parquetV2VerboseLogs()) {
                        std::cerr << "[Columnar] segmented PPMC group=" << group
                                  << " cur=" << cur
                                  << " vertices=" << item.vertices.size()
                                  << " halfedges=" << item.halfedges.size()
                                  << " facets=" << item.facets.size()
                                  << "\n";
                    }
                    pending.push_back(std::move(item));
                    segmentedGroupCurrentRound_[group] = cur + 1;
                    if (mesh_) {
                        mesh_->resetState();
                    }
                }
            }
            restorePendingState();

            auto reconstructPending = [&](PendingSegmentedPPMC& item) {
                reconstructOperator_->reconstruct(item.vertices, item.halfedges, item.facets);
                if (parquetV2VerboseLogs()) {
                    std::cerr << "[Columnar] segmented PPMC group=" << item.group
                              << " cur=" << item.cur
                              << " reconstructed\n";
                }
            };
            if (segmentedBatchGroupReconstruct()) {
                PendingSegmentedPPMC combined;
                combined.group = -1;
                combined.cur = currentRound_;
                for (auto& item : pending) {
                    combined.vertices.insert(combined.vertices.end(),
                                             item.vertices.begin(), item.vertices.end());
                    combined.halfedges.insert(combined.halfedges.end(),
                                              item.halfedges.begin(), item.halfedges.end());
                    combined.facets.insert(combined.facets.end(),
                                           item.facets.begin(), item.facets.end());
                }
                reconstructPending(combined);
            } else if (segmentedReverseGroupReconstruct()) {
                for (auto it = pending.rbegin(); it != pending.rend(); ++it) {
                    reconstructPending(*it);
                }
            } else {
                for (auto& item : pending) {
                    reconstructPending(item);
                }
            }

            if (parquetV2VerboseLogs()) {
                std::cout << "[Columnar] segmented PPMC round " << currentRound_
                          << " decoded groups=" << (endGroup - beginGroup) << std::endl;
            }
            currentRound_++;
            return;
        }

        if (pmsf::paperBlobLayoutIsSegmentedRangeCodedPred(layout)) {
            // PHASE60 物品 3：整段轮驱动（含每分区 seed 刷新、只在本组面内 BFS、
            // 瞬态标记的保存/恢复与 reconstruct 顺序）搬到共享镜像里。
            // 分区选择谓词**每轮现读**（`setOnlyGroupId` / `setTargetGroupId` /
            // `setMaxGroups` 可能在基网之后才设），故逐轮下发。
            segmentedDecode_.setGroupSelection(onlyGroupId_, targetGroupId_, maxGroups_);
            segmentedDecode_.decodeOneRound();
            currentRound_++;
            return;
        }

        // ⚠️ P3.e 对齐（2026-05-14）：PPMC 每轮解码前重置全网格 face/halfedge 状态。
        //   原版 `beginUndecimationConquest()` 调用全网格 hit->resetState() / fit->resetState()。
        //   跳过该重置会导致上一轮标记的 Splittable/Unsplittable/Processed 标志残留，
        //   isConquered()/isProcessed() 提前返回 true → BFS 被阻断 → 后续轮次无重建产出。
        if (pmsf::paperBlobLayoutIsPPMCRC(layout)) {
            mesh_->resetState();
        }

        // P3.c seed refresh：v2 路径 non-segmented 单 seed。
        //
        // OpenMesh 的顺序是 resetState() 后再 pushHehInit()。layout=3 的 fallback
        // seed 搜索会看 face 状态，必须避免上一轮 Splittable/Unsplittable 残留影响
        // BFS 起点。
        refreshSeed(seedV0_, seedV1_, seed_);

        tpCollect = Clock::now();
        if (pmsf::paperBlobLayoutUsesDecodedQueues(layout)) {
            // RANGE_CODED_PRED / VALENCE_RC / CODEC_006 / RD_GATE / RD_GATE_ONLY
            //   / VALENCE_PATCH_ONLY / RD_GATE_CONQUEST / CODEC_014 / CODEC_016
            //   / CODEC_017 / RD_TAG / CODEC_020 / CODEC_021：
            // 内部队列已通过 importFromRangeCodedBuffer 装填（可能仅部分装填 +
            // 延迟解码），按轮消费即可。
            symbolReadOperator_->collectFromDecoded(seed_, vertices_, halfedges_, facets_);
         } else if (paperV2Typed_) {
            // M4-4 typed v2: consume one PMSFSymbolTable row per round through the
            // algorithm's typed symbol driver (currently PPMC).
            bindPPMCReaderMesh();
            symbolReadOperator_->groupId = -1;
            symbolReadOperator_->collectFromDecoded(seed_, vertices_, halfedges_, facets_);
        } else {
            // BASIC / CODEC_003 / OPTIMIZED_PLAIN：每轮通过 collect(buffer, dataOffset, ...) 推进
            if (pmsf::paperBlobLayoutIsPPMCRC(layout)) {
                symbolReadOperator_->groupId = -1;
            }
            symbolReadOperator_->collect(
                seed_, rcBlob_.data(), rcBlobOffset_,
                vertices_, halfedges_, facets_);
            if (rcBlobOffset_ > rcBlob_.size()) {
                throw std::runtime_error(
                    "[Columnar] PPMC rc stream over-read: offset=" +
                    std::to_string(rcBlobOffset_) + " size=" +
                    std::to_string(rcBlob_.size()) +
                    " round=" + std::to_string(currentRound_));
            }
        }
        if (profile) {
            collectMs = std::chrono::duration<double, std::milli>(
                            Clock::now() - tpCollect).count();
        }

        tpReconstruct = Clock::now();
        reconstructOperator_->reconstruct(vertices_, halfedges_, facets_);
        if (profile) {
            auto afterReconstruct = Clock::now();
            reconstructMs = std::chrono::duration<double, std::milli>(
                                afterReconstruct - tpReconstruct).count();
            const double roundMs = std::chrono::duration<double, std::milli>(
                                      afterReconstruct - tpRound).count();
            std::fprintf(stderr,
                         "[DECOMP_PROFILE] round=%d layout=%d collect=%.3fms "
                         "reconstruct=%.3fms total=%.3fms vertices=%zu halfedges=%zu facets=%zu\n",
                         currentRound_, layout, collectMs, reconstructMs, roundMs,
                         vertices_.size(), halfedges_.size(), facets_.size());
        }

        if (parquetV2VerboseLogs()) {
            std::cout << "[Columnar] 第 " << currentRound_ << " 轮重建完成 (v2):"
                      << " 顶点调整=" << vertices_.size()
                      << " 边合并=" << halfedges_.size()
                      << " 面片分裂=" << facets_.size() << std::endl;
        }
        currentRound_++;
        return;
    }

    // ========== 旧 bool-list 路径（向后兼容） ==========
    // 将 header 传递给 symbolReadOperator（用于判断 isEnableAdjustment 等）
    symbolReadOperator_->header_ = header_;

    bool isSegmented = header_.isEnableSegmentation() && seeds_.size() > 1;

    if (isSegmented) {
        // ========== 分区模式：每个 group 独立处理当前轮次 ==========
        for (size_t gid = 0; gid < seeds_.size(); gid++) {
            // 查找当前 group 当前轮次的符号数据
            GroupSymbolData* symData = nullptr;
            for (auto& sd : symbolData_) {
                if (sd.groupId == (int)gid && sd.roundId == currentRound_) {
                    symData = &sd;
                    break;
                }
            }
            if (!symData) {
                std::cerr << "[Columnar] 缺少 group=" << gid
                          << " round=" << currentRound_ << " 的符号数据" << std::endl;
                continue;
            }

            // 从 Parquet 数据重建点坐标（面片点流 + 顶点附加点流）
            std::vector<MCGAL::Point> points;
            std::vector<MCGAL::Point> vertexPoints;
            if (!symData->pointX.empty()) {
                for (size_t i = 0; i < symData->pointX.size(); i++) {
                    points.emplace_back(symData->pointX[i], symData->pointY[i], symData->pointZ[i]);
                }
            } else if (!symData->pointQX.empty()) {
                for (size_t i = 0; i < symData->pointQX.size(); i++) {
                    float x = bboxMinX_ + (symData->pointQX[i] + 0.5f) * quantStep_;
                    float y = bboxMinY_ + (symData->pointQY[i] + 0.5f) * quantStep_;
                    float z = bboxMinZ_ + (symData->pointQZ[i] + 0.5f) * quantStep_;
                    points.emplace_back(x, y, z);
                }
            }
            if (!symData->vertexPointX.empty()) {
                for (size_t i = 0; i < symData->vertexPointX.size(); i++) {
                    vertexPoints.emplace_back(symData->vertexPointX[i], symData->vertexPointY[i], symData->vertexPointZ[i]);
                }
            } else if (!symData->vertexPointQX.empty()) {
                for (size_t i = 0; i < symData->vertexPointQX.size(); i++) {
                    float x = bboxMinX_ + (symData->vertexPointQX[i] + 0.5f) * quantStep_;
                    float y = bboxMinY_ + (symData->vertexPointQY[i] + 0.5f) * quantStep_;
                    float z = bboxMinZ_ + (symData->vertexPointQZ[i] + 0.5f) * quantStep_;
                    vertexPoints.emplace_back(x, y, z);
                }
            }

            // 设置 groupId 用于分区 BFS 过滤
            symbolReadOperator_->groupId = gid;

            // 导入符号数据到 SymbolReadOperator 内部队列；M2c native 模式
            // 由算法 reader 自己从 paper table 导入，跳过 legacy 绝对坐标导入。
            if (!(std::getenv("PMSF_PAPER_NATIVE") != nullptr &&
                  dynamic_cast<PPMCSymbolReadOperator*>(symbolReadOperator_.get()) != nullptr)) {
                symbolReadOperator_->importFromParquetExtended(
                    symData->facetSymbols,
                    symData->edgeSymbols,
                    symData->vertexSymbols,
                    points,
                    vertexPoints);
            }

            // 从内部队列按 BFS 顺序消费符号并重建
            std::vector<MCGAL::Vertex*> gVertices;
            std::vector<MCGAL::Halfedge*> gHalfedges;
            std::vector<MCGAL::Facet*> gFacets;
            symbolReadOperator_->collectFromDecoded(seeds_[gid], gVertices, gHalfedges, gFacets);

            // 合并到全局结果
            vertices_.insert(vertices_.end(), gVertices.begin(), gVertices.end());
            halfedges_.insert(halfedges_.end(), gHalfedges.begin(), gHalfedges.end());
            facets_.insert(facets_.end(), gFacets.begin(), gFacets.end());
        }

        // 统一重建
        reconstructOperator_->reconstruct(vertices_, halfedges_, facets_);

    } else {
        // ========== 非分区模式：单 seed ==========
        // 查找当前轮次的符号数据（group_id=0）
        GroupSymbolData* symData = nullptr;
        for (auto& sd : symbolData_) {
            if (sd.groupId == 0 && sd.roundId == currentRound_) {
                symData = &sd;
                break;
            }
        }
        if (!symData) {
            std::cerr << "[Columnar] 缺少 round=" << currentRound_ << " 的符号数据" << std::endl;
            currentRound_++;
            return;
        }

        // 从 Parquet 数据重建点坐标（面片点流 + 顶点附加点流）
        std::vector<MCGAL::Point> points;
        std::vector<MCGAL::Point> vertexPoints;
        if (!symData->pointX.empty()) {
            for (size_t i = 0; i < symData->pointX.size(); i++) {
                points.emplace_back(symData->pointX[i], symData->pointY[i], symData->pointZ[i]);
            }
        } else if (!symData->pointQX.empty()) {
            for (size_t i = 0; i < symData->pointQX.size(); i++) {
                float x = bboxMinX_ + (symData->pointQX[i] + 0.5f) * quantStep_;
                float y = bboxMinY_ + (symData->pointQY[i] + 0.5f) * quantStep_;
                float z = bboxMinZ_ + (symData->pointQZ[i] + 0.5f) * quantStep_;
                points.emplace_back(x, y, z);
            }
        }
        if (!symData->vertexPointX.empty()) {
            for (size_t i = 0; i < symData->vertexPointX.size(); i++) {
                vertexPoints.emplace_back(symData->vertexPointX[i], symData->vertexPointY[i], symData->vertexPointZ[i]);
            }
        } else if (!symData->vertexPointQX.empty()) {
            for (size_t i = 0; i < symData->vertexPointQX.size(); i++) {
                float x = bboxMinX_ + (symData->vertexPointQX[i] + 0.5f) * quantStep_;
                float y = bboxMinY_ + (symData->vertexPointQY[i] + 0.5f) * quantStep_;
                float z = bboxMinZ_ + (symData->vertexPointQZ[i] + 0.5f) * quantStep_;
                vertexPoints.emplace_back(x, y, z);
            }
        }

        // 导入符号数据到 SymbolReadOperator 内部队列；M2c native 只让 PPMC 跳过
        // legacy import（PPMC 有专属 paper-table reader），其他算法仍用 symData。
        if (!(std::getenv("PMSF_PAPER_NATIVE") != nullptr &&
              dynamic_cast<PPMCSymbolReadOperator*>(symbolReadOperator_.get()) != nullptr)) {
            symbolReadOperator_->importFromParquetExtended(
                symData->facetSymbols,
                symData->edgeSymbols,
                symData->vertexSymbols,
                points,
                vertexPoints);
        }

        // 从内部队列按 BFS 顺序消费符号并重建
        symbolReadOperator_->collectFromDecoded(seed_, vertices_, halfedges_, facets_);

        // 重建
        reconstructOperator_->reconstruct(vertices_, halfedges_, facets_);
    }

    if (parquetV2VerboseLogs()) {
        std::cout << "[Columnar] 第 " << currentRound_ << " 轮重建完成:"
                  << " 顶点调整=" << vertices_.size()
                  << " 边合并=" << halfedges_.size()
                  << " 面片分裂=" << facets_.size() << std::endl;
    }

    currentRound_++;
}

// ============================================================================
// 导出网格
// ============================================================================

std::shared_ptr<MCGAL::Mesh> ColumnarDeserializeOperator::exportMesh() {
    return mesh_;
}

// ============================================================================
// Tier 2 (2026-05-09)：V2 紧凑模式读取
// 字节布局见 ColumnarSerializeOperator::writeV2CompactBin 注释。
// ============================================================================
namespace {

inline std::uint32_t rdU32LE(const std::uint8_t*& p) {
    std::uint32_t v = (std::uint32_t)p[0] | ((std::uint32_t)p[1] << 8)
                    | ((std::uint32_t)p[2] << 16) | ((std::uint32_t)p[3] << 24);
    p += 4;
    return v;
}
inline std::int32_t rdI32LE(const std::uint8_t*& p) {
    return static_cast<std::int32_t>(rdU32LE(p));
}
inline std::uint16_t rdU16LE(const std::uint8_t*& p) {
    std::uint16_t v = (std::uint16_t)p[0] | ((std::uint16_t)p[1] << 8);
    p += 2;
    return v;
}
inline float rdF32LE(const std::uint8_t*& p) {
    std::uint32_t u = rdU32LE(p);
    float f; std::memcpy(&f, &u, 4);
    return f;
}
inline std::uint8_t rdU8(const std::uint8_t*& p) { return *p++; }
inline std::uint32_t rdVarU32(const std::uint8_t*& p, const std::uint8_t* pEnd) {
    std::uint32_t x = 0;
    int shift = 0;
    while (p < pEnd && shift <= 28) {
        const std::uint8_t b = *p++;
        x |= static_cast<std::uint32_t>(b & 0x7fu) << shift;
        if ((b & 0x80u) == 0) return x;
        shift += 7;
    }
    return x;
}
inline std::int32_t unzigzagI32(std::uint32_t x) {
    return static_cast<std::int32_t>((x >> 1) ^ (~(x & 1u) + 1u));
}
inline std::int32_t rdVarI32(const std::uint8_t*& p, const std::uint8_t* pEnd) {
    return unzigzagI32(rdVarU32(p, pEnd));
}

bool logBoundaryOwnerManifestBlob(const std::uint8_t* data, int32_t len) {
    if (!data || len < 8 || data[0] != 'P' || data[1] != 'B' ||
        data[2] != 'O' || data[3] != '1') {
        return false;
    }
    const std::uint8_t* p = data + 4;
    const std::uint8_t* pEnd = data + len;
    const std::uint32_t coordCount = rdVarU32(p, pEnd);
    const std::uint32_t groupCount = rdVarU32(p, pEnd);
    const std::uint32_t boundaryCoordCount = rdVarU32(p, pEnd);
    const std::uint32_t componentCount = rdVarU32(p, pEnd);
    if (p > pEnd) return false;

    std::vector<std::uint32_t> boundaryCoordIds;
    boundaryCoordIds.reserve(boundaryCoordCount);
    std::uint32_t prevCoord = 0;
    for (std::uint32_t i = 0; i < boundaryCoordCount; ++i) {
        prevCoord += rdVarU32(p, pEnd);
        if (prevCoord >= coordCount) return false;
        boundaryCoordIds.push_back(prevCoord);
    }

    std::vector<std::uint32_t> componentCoordCounts(componentCount, 0);
    for (std::uint32_t i = 0; i < boundaryCoordCount; ++i) {
        const std::uint32_t componentId = rdVarU32(p, pEnd);
        if (componentId >= componentCount) return false;
        ++componentCoordCounts[componentId];
    }

    std::uint64_t componentIncidentRefs = 0;
    std::uint32_t maxComponentCoords = 0;
    for (std::uint32_t c = 0; c < componentCount; ++c) {
        const std::uint32_t ownerGroup = rdVarU32(p, pEnd);
        const std::uint32_t coordCountInComponent = rdVarU32(p, pEnd);
        const std::uint32_t incidentCount = rdVarU32(p, pEnd);
        if (ownerGroup >= groupCount || coordCountInComponent != componentCoordCounts[c]) {
            return false;
        }
        maxComponentCoords = std::max(maxComponentCoords, coordCountInComponent);
        std::uint32_t prevGroup = 0;
        for (std::uint32_t i = 0; i < incidentCount; ++i) {
            prevGroup += rdVarU32(p, pEnd);
            if (prevGroup >= groupCount) return false;
            ++componentIncidentRefs;
        }
    }

    const std::uint32_t manifestGroupCount = rdVarU32(p, pEnd);
    std::uint64_t groupRefs = 0;
    for (std::uint32_t g = 0; g < manifestGroupCount; ++g) {
        const std::uint32_t groupId = rdVarU32(p, pEnd);
        const std::uint32_t refCount = rdVarU32(p, pEnd);
        if (groupId >= groupCount) return false;
        std::uint32_t prevBoundaryIndex = 0;
        for (std::uint32_t i = 0; i < refCount; ++i) {
            prevBoundaryIndex += rdVarU32(p, pEnd);
            if (prevBoundaryIndex >= boundaryCoordCount) return false;
            ++groupRefs;
        }
    }
    if (p > pEnd) return false;

    std::cout << "[PMSF_BOS1_OWNER_MANIFEST]"
              << " decoded=1"
              << " owner_blob_bytes=" << len
              << " coord_count=" << coordCount
              << " groups=" << groupCount
              << " boundary_coords=" << boundaryCoordCount
              << " components=" << componentCount
              << " component_incident_refs=" << componentIncidentRefs
              << " group_refs=" << groupRefs
              << " max_component_coords=" << maxComponentCoords
              << std::endl;
    return true;
}

std::string shellQuoteDec(const std::filesystem::path& p) {
    std::string s = p.string();
    std::string out = "'";
    for (char c : s) {
        if (c == '\'') out += "'\\''";
        else out.push_back(c);
    }
    out += "'";
    return out;
}

std::filesystem::path edgeBreakerToolDec(const char* exeName) {
    if (const char* dir = std::getenv("PMSF_EDGEBREAKER_DIR")) {
        std::filesystem::path base(dir);
        std::filesystem::path p = base / exeName;
        if (std::filesystem::exists(p)) return p;
        p = base / (std::string(exeName) + ".exe");
        if (std::filesystem::exists(p)) return p;
    }
    std::filesystem::path cwd = std::filesystem::current_path();
    for (const auto& root : {cwd, cwd.parent_path()}) {
        std::filesystem::path p = root / "deps" / "EdgeBreaker" / "src" / exeName;
        if (std::filesystem::exists(p)) return p;
        p = root / "deps" / "EdgeBreaker" / "src" / (std::string(exeName) + ".exe");
        if (std::filesystem::exists(p)) return p;
    }
    return {};
}

int quantizeCoord(float x, float min, float step) {
    const double t = (static_cast<double>(x) - static_cast<double>(min)) /
                     static_cast<double>(step);
    return static_cast<int>(std::floor(t + 1e-4));
}

bool parseOff(const std::filesystem::path& path,
              std::vector<std::array<int, 3>>& qverts,
              std::vector<std::array<int, 3>>& faces,
              float bboxMinX,
              float bboxMinY,
              float bboxMinZ,
              float quantStep) {
    std::ifstream ifs(path);
    if (!ifs) return false;
    std::string magic;
    ifs >> magic;
    if (magic != "OFF") return false;
    int nv = 0, nf = 0, ne = 0;
    ifs >> nv >> nf >> ne;
    if (nv <= 0 || nf < 0) return false;
    qverts.clear();
    qverts.reserve(static_cast<size_t>(nv));
    for (int i = 0; i < nv; ++i) {
        float x = 0, y = 0, z = 0;
        ifs >> x >> y >> z;
        qverts.push_back({quantizeCoord(x, bboxMinX, quantStep),
                          quantizeCoord(y, bboxMinY, quantStep),
                          quantizeCoord(z, bboxMinZ, quantStep)});
    }
    faces.clear();
    faces.reserve(static_cast<size_t>(nf));
    for (int i = 0; i < nf; ++i) {
        int d = 0, a = 0, b = 0, c = 0;
        ifs >> d >> a >> b >> c;
        if (!ifs || d != 3 || a < 0 || b < 0 || c < 0 ||
            a >= nv || b >= nv || c >= nv) {
            return false;
        }
        faces.push_back({a, b, c});
    }
    return true;
}

int findClosestQ(const std::vector<std::array<int, 3>>& qverts,
                 const std::array<int, 3>& target,
                 int forbidden) {
    int best = -1;
    long long bestDist = std::numeric_limits<long long>::max();
    for (int i = 0; i < static_cast<int>(qverts.size()); ++i) {
        if (i == forbidden) continue;
        const long long d =
            std::llabs(static_cast<long long>(qverts[i][0]) - target[0]) +
            std::llabs(static_cast<long long>(qverts[i][1]) - target[1]) +
            std::llabs(static_cast<long long>(qverts[i][2]) - target[2]);
        if (d < bestDist) {
            bestDist = d;
            best = i;
        }
    }
    return best;
}

template <typename GroupData>
bool decodeNativeEdgeBreakerBase(const std::vector<std::uint8_t>& beb,
                                 GroupData& data,
                                 float bboxMinX,
                                 float bboxMinY,
                                 float bboxMinZ,
                                 float quantStep) {
    pmsf_eb::DecodeResult dec = pmsf_eb::decodeEdgeBreaker(beb);
    if (!dec.ok || dec.verts.size() < 2) return false;
    std::vector<std::array<int, 3>> qverts(dec.verts.size());
    for (size_t i = 0; i < dec.verts.size(); ++i)
        qverts[i] = {dec.verts[i][0], dec.verts[i][1], dec.verts[i][2]};

    const int seed0 = findClosestQ(
        qverts, {data.seedV0QX, data.seedV0QY, data.seedV0QZ}, -1);
    const int seed1 = findClosestQ(
        qverts, {data.seedV1QX, data.seedV1QY, data.seedV1QZ}, seed0);
    if (seed0 < 0 || seed1 < 0 || seed0 == seed1) return false;

    // The non-seed base vertices must be re-labelled in the SAME order the
    // encoder used (Morton order of the quantized coords, see
    // ColumnarSerializeOperator mortonVertexOrder path). The decoder's base
    // vertex ENUMERATION order == vid order == seed-channel ordinal space, so
    // this is what makes the per-group seed ordinals resolve to the right
    // vertices (base meshes have no duplicate quantized coords, verified).
    auto spreadMortonBits = [](std::uint32_t value) -> std::uint64_t {
        std::uint64_t result = 0;
        for (unsigned bit = 0; bit < 21; ++bit)
            result |= static_cast<std::uint64_t>((value >> bit) & 1u) << (3u * bit);
        return result;
    };
    auto mortonOf = [&](int idx) -> std::uint64_t {
        return spreadMortonBits(static_cast<std::uint32_t>(qverts[idx][0])) |
               (spreadMortonBits(static_cast<std::uint32_t>(qverts[idx][1])) << 1) |
               (spreadMortonBits(static_cast<std::uint32_t>(qverts[idx][2])) << 2);
    };
    std::vector<int> nonSeed;
    nonSeed.reserve(qverts.size());
    for (int i = 0; i < static_cast<int>(qverts.size()); ++i)
        if (i != seed0 && i != seed1) nonSeed.push_back(i);
    std::sort(nonSeed.begin(), nonSeed.end(), [&](int a, int b) {
        const std::uint64_t ma = mortonOf(a), mb = mortonOf(b);
        if (ma != mb) return ma < mb;
        if (qverts[a] != qverts[b]) return qverts[a] < qverts[b];
        return a < b;
    });
    std::vector<int> remap(qverts.size(), -1);
    remap[static_cast<size_t>(seed0)] = 0;
    remap[static_cast<size_t>(seed1)] = 1;
    int next = 2;
    for (int idx : nonSeed) remap[static_cast<size_t>(idx)] = next++;

    data.numVertices = static_cast<int>(qverts.size()) - 2;
    data.numFaces = static_cast<int>(dec.faces.size());
    data.vertexQX.assign(static_cast<size_t>(data.numVertices), 0);
    data.vertexQY.assign(static_cast<size_t>(data.numVertices), 0);
    data.vertexQZ.assign(static_cast<size_t>(data.numVertices), 0);
    for (int old = 0; old < static_cast<int>(qverts.size()); ++old) {
        const int id = remap[static_cast<size_t>(old)];
        if (id < 2) continue;
        const size_t dst = static_cast<size_t>(id - 2);
        data.vertexQX[dst] = qverts[static_cast<size_t>(old)][0];
        data.vertexQY[dst] = qverts[static_cast<size_t>(old)][1];
        data.vertexQZ[dst] = qverts[static_cast<size_t>(old)][2];
    }
    data.vertexX.resize(data.vertexQX.size());
    data.vertexY.resize(data.vertexQY.size());
    data.vertexZ.resize(data.vertexQZ.size());
    for (size_t i = 0; i < data.vertexQX.size(); ++i) {
        data.vertexX[i] = bboxMinX + (data.vertexQX[i] + 0.5f) * quantStep;
        data.vertexY[i] = bboxMinY + (data.vertexQY[i] + 0.5f) * quantStep;
        data.vertexZ[i] = bboxMinZ + (data.vertexQZ[i] + 0.5f) * quantStep;
    }
    data.faceDegrees.assign(dec.faces.size(), 3);
    data.faceVertices.clear();
    data.faceVertices.reserve(dec.faces.size() * 3);
    for (const auto& f : dec.faces) {
        data.faceVertices.push_back(remap[static_cast<size_t>(f[0])]);
        data.faceVertices.push_back(remap[static_cast<size_t>(f[1])]);
        data.faceVertices.push_back(remap[static_cast<size_t>(f[2])]);
    }
    return true;
}

template <typename GroupData>
bool decodeEdgeBreakerBase(const std::vector<std::uint8_t>& beb,
                           GroupData& data,
                           float bboxMinX,
                           float bboxMinY,
                           float bboxMinZ,
                           float quantStep) {
    const std::filesystem::path ebdc = edgeBreakerToolDec("ebdc");
    if (ebdc.empty() || beb.empty()) return false;
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    std::filesystem::path tmp =
        std::filesystem::temp_directory_path() /
        ("pmsf_ebdc_" + std::to_string(static_cast<long long>(stamp)));
    std::error_code ec;
    std::filesystem::create_directories(tmp, ec);
    if (ec) return false;
    const std::filesystem::path bebPath = tmp / "base.beb";
    const std::filesystem::path offPath = tmp / "base.off";
    {
        std::ofstream ofs(bebPath, std::ios::binary);
        if (!ofs) {
            std::filesystem::remove_all(tmp, ec);
            return false;
        }
        ofs.write(reinterpret_cast<const char*>(beb.data()),
                  static_cast<std::streamsize>(beb.size()));
    }
    const std::string cmd =
        shellQuoteDec(ebdc) + " " + shellQuoteDec(bebPath) + " " +
        shellQuoteDec(offPath) + " >/dev/null 2>&1";
    if (std::system(cmd.c_str()) != 0) {
        std::filesystem::remove_all(tmp, ec);
        return false;
    }
    std::vector<std::array<int, 3>> qverts;
    std::vector<std::array<int, 3>> faces;
    const bool parsed = parseOff(offPath, qverts, faces,
                                 bboxMinX, bboxMinY, bboxMinZ, quantStep);
    std::filesystem::remove_all(tmp, ec);
    if (!parsed || qverts.size() < 2) return false;

    const int seed0 = findClosestQ(
        qverts, {data.seedV0QX, data.seedV0QY, data.seedV0QZ}, -1);
    const int seed1 = findClosestQ(
        qverts, {data.seedV1QX, data.seedV1QY, data.seedV1QZ}, seed0);
    if (seed0 < 0 || seed1 < 0 || seed0 == seed1) return false;

    std::vector<int> remap(qverts.size(), -1);
    remap[static_cast<size_t>(seed0)] = 0;
    remap[static_cast<size_t>(seed1)] = 1;
    int next = 2;
    for (int i = 0; i < static_cast<int>(qverts.size()); ++i) {
        if (remap[static_cast<size_t>(i)] < 0) remap[static_cast<size_t>(i)] = next++;
    }

    data.numVertices = static_cast<int>(qverts.size()) - 2;
    data.numFaces = static_cast<int>(faces.size());
    data.vertexQX.assign(static_cast<size_t>(data.numVertices), 0);
    data.vertexQY.assign(static_cast<size_t>(data.numVertices), 0);
    data.vertexQZ.assign(static_cast<size_t>(data.numVertices), 0);
    for (int old = 0; old < static_cast<int>(qverts.size()); ++old) {
        const int id = remap[static_cast<size_t>(old)];
        if (id < 2) continue;
        const size_t dst = static_cast<size_t>(id - 2);
        data.vertexQX[dst] = qverts[static_cast<size_t>(old)][0];
        data.vertexQY[dst] = qverts[static_cast<size_t>(old)][1];
        data.vertexQZ[dst] = qverts[static_cast<size_t>(old)][2];
    }
    data.vertexX.resize(data.vertexQX.size());
    data.vertexY.resize(data.vertexQY.size());
    data.vertexZ.resize(data.vertexQZ.size());
    for (size_t i = 0; i < data.vertexQX.size(); ++i) {
        data.vertexX[i] = bboxMinX + (data.vertexQX[i] + 0.5f) * quantStep;
        data.vertexY[i] = bboxMinY + (data.vertexQY[i] + 0.5f) * quantStep;
        data.vertexZ[i] = bboxMinZ + (data.vertexQZ[i] + 0.5f) * quantStep;
    }
    data.faceDegrees.assign(faces.size(), 3);
    data.faceVertices.clear();
    data.faceVertices.reserve(faces.size() * 3);
    for (const auto& f : faces) {
        data.faceVertices.push_back(remap[static_cast<size_t>(f[0])]);
        data.faceVertices.push_back(remap[static_cast<size_t>(f[1])]);
        data.faceVertices.push_back(remap[static_cast<size_t>(f[2])]);
    }
    return true;
}

bool decodeQuantCoordBlock(const std::uint8_t* data,
                           int32_t len,
                           int32_t expectedCount,
                           std::vector<int32_t>& qx,
                           std::vector<int32_t>& qy,
                           std::vector<int32_t>& qz) {
    if (!data || len < 5) return false;
    const std::uint8_t* p = data;
    const std::uint8_t* pEnd = data + len;
    if (p[0] != 'Q' || p[1] != 'C' || p[2] != 'B' || p[3] != '1') return false;
    p += 4;
    const int32_t count = static_cast<int32_t>(rdVarU32(p, pEnd));
    if (count < 0 || (expectedCount >= 0 && count != expectedCount)) return false;
    qx.assign(static_cast<size_t>(count), 0);
    qy.assign(static_cast<size_t>(count), 0);
    qz.assign(static_cast<size_t>(count), 0);
    int prev = 0;
    for (int32_t i = 0; i < count; ++i) {
        prev += rdVarI32(p, pEnd);
        qx[static_cast<size_t>(i)] = prev;
    }
    prev = 0;
    for (int32_t i = 0; i < count; ++i) {
        prev += rdVarI32(p, pEnd);
        qy[static_cast<size_t>(i)] = prev;
    }
    prev = 0;
    for (int32_t i = 0; i < count; ++i) {
        prev += rdVarI32(p, pEnd);
        qz[static_cast<size_t>(i)] = prev;
    }
    return p <= pEnd;
}

bool unpackFixedBits(const std::uint8_t*& p,
                     const std::uint8_t* pEnd,
                     int bits,
                     int32_t count,
                     std::vector<int32_t>& values) {
    if (bits < 0 || bits > 31 || count < 0) return false;
    values.assign(static_cast<size_t>(count), 0);
    if (bits == 0) return true;
    const std::size_t needed =
        (static_cast<std::size_t>(count) * static_cast<std::size_t>(bits) + 7u) / 8u;
    if (p + needed > pEnd) return false;
    for (int32_t i = 0; i < count; ++i) {
        std::uint32_t v = 0;
        const std::size_t baseBit = static_cast<std::size_t>(i) * static_cast<std::size_t>(bits);
        for (int b = 0; b < bits; ++b) {
            const std::size_t bit = baseBit + static_cast<std::size_t>(b);
            const std::uint8_t byte = p[bit / 8u];
            if ((byte >> (bit % 8u)) & 1u) {
                v |= (1u << b);
            }
        }
        values[static_cast<size_t>(i)] = static_cast<int32_t>(v);
    }
    p += needed;
    return true;
}

template <typename GroupData>
bool decodePartitionBaseBlob(const std::uint8_t* data,
                             int32_t len,
                             GroupData& out) {
    if (!data || len < 8) return false;
    const std::uint8_t* p = data;
    const std::uint8_t* pEnd = data + len;
    if (p[0] != 'P' || p[1] != 'B' || p[2] != 'M' || p[3] != '1') return false;
    p += 4;
    const std::uint8_t flags = rdU8(p);
    const bool allTri = (flags & 0x1) != 0;
    const bool deltaIndices = (flags & 0x2) != 0;
    const bool signedCoordDeltas = (flags & 0x4) != 0;
    const int32_t localCount = static_cast<int32_t>(rdVarU32(p, pEnd));
    const int32_t numFaces = static_cast<int32_t>(rdVarU32(p, pEnd));
    const int32_t seed0Local = static_cast<int32_t>(rdVarU32(p, pEnd));
    const int32_t seed1Local = static_cast<int32_t>(rdVarU32(p, pEnd));
    if (localCount <= 0 || seed0Local < 0 || seed1Local < 0 ||
        seed0Local >= localCount || seed1Local >= localCount) {
        return false;
    }

    std::vector<int32_t> localCoordIds(static_cast<size_t>(localCount), 0);
    int32_t prevCoordId = 0;
    for (int32_t i = 0; i < localCount; ++i) {
        prevCoordId += signedCoordDeltas
                           ? rdVarI32(p, pEnd)
                           : static_cast<int32_t>(rdVarU32(p, pEnd));
        localCoordIds[static_cast<size_t>(i)] = prevCoordId;
    }

    out.seedV0CoordId = localCoordIds[static_cast<size_t>(seed0Local)];
    out.seedV1CoordId = localCoordIds[static_cast<size_t>(seed1Local)];
    out.numVertices = localCount;
    out.numFaces = numFaces;
    out.faceVerticesAreCoordIds = true;
    out.vertexCoordIds.clear();
    out.vertexSharedPositions.clear();

    if (allTri) {
        out.faceDegrees.assign(static_cast<size_t>(numFaces), 3);
    } else {
        const int32_t degreeCount = static_cast<int32_t>(rdVarU32(p, pEnd));
        if (degreeCount < 0) return false;
        out.faceDegrees.resize(static_cast<size_t>(degreeCount));
        for (int32_t i = 0; i < degreeCount; ++i) {
            out.faceDegrees[static_cast<size_t>(i)] = rdVarI32(p, pEnd);
        }
    }

    const int32_t cornerCount = static_cast<int32_t>(rdVarU32(p, pEnd));
    std::vector<int32_t> localFaceVertices;
    if (deltaIndices) {
        localFaceVertices.assign(static_cast<size_t>(cornerCount), 0);
        if (allTri && cornerCount == numFaces * 3) {
            for (int corner = 0; corner < 3; ++corner) {
                int prev = 0;
                for (int f = 0; f < numFaces; ++f) {
                    prev += rdVarI32(p, pEnd);
                    localFaceVertices[static_cast<size_t>(3 * f + corner)] = prev;
                }
            }
        } else {
            int prev = 0;
            for (int32_t i = 0; i < cornerCount; ++i) {
                prev += rdVarI32(p, pEnd);
                localFaceVertices[static_cast<size_t>(i)] = prev;
            }
        }
    } else {
        const int bits = static_cast<int>(rdU8(p));
        if (!unpackFixedBits(p, pEnd, bits, cornerCount, localFaceVertices)) return false;
    }
    out.faceVertices.clear();
    out.faceVertices.reserve(localFaceVertices.size());
    for (int32_t local : localFaceVertices) {
        if (local < 0 || local >= localCount) return false;
        out.faceVertices.push_back(localCoordIds[static_cast<size_t>(local)]);
    }
    return p <= pEnd;
}

}  // namespace

bool ColumnarDeserializeOperator::readPartitionedBaseMeshTables(const std::string& baseName) {
    const std::string superPath = pmsf::paperPartitionedBaseTablePath(baseName);
    if (std::filesystem::exists(superPath)) {
        auto superTable = readParquetFile(superPath);
        if (!superTable || superTable->num_rows() <= 0) return false;
        auto modeCol = superTable->GetColumnByName(pmsf::kParquetBaseModeColumn);
        auto blobCol = superTable->GetColumnByName(pmsf::kParquetBaseBlobColumn);
        if (!modeCol || !blobCol || blobCol->num_chunks() == 0) {
            std::cerr << "[Columnar] partitioned base superblob missing columns" << std::endl;
            return false;
        }
        auto ownerCol = superTable->GetColumnByName(pmsf::kParquetBoundaryOwnerBlobColumn);
        if (ownerCol && ownerCol->num_chunks() > 0) {
            auto ownerA = std::static_pointer_cast<arrow::BinaryArray>(ownerCol->chunk(0));
            if (ownerA->length() > 0 && !ownerA->IsNull(0)) {
                int32_t ownerLen = 0;
                const std::uint8_t* ownerPtr = ownerA->GetValue(0, &ownerLen);
                if (!logBoundaryOwnerManifestBlob(ownerPtr, ownerLen)) {
                    std::cerr << "[Columnar] invalid boundary owner manifest blob"
                              << std::endl;
                    return false;
                }
            }
        }
        auto modeA = std::static_pointer_cast<arrow::Int32Array>(modeCol->chunk(0));
        auto blobA = std::static_pointer_cast<arrow::BinaryArray>(blobCol->chunk(0));
        if (modeA->Value(0) != 2) {
            std::cerr << "[Columnar] unsupported partitioned base superblob mode: "
                      << modeA->Value(0) << std::endl;
            return false;
        }
        int32_t len = 0;
        const std::uint8_t* ptr = blobA->GetValue(0, &len);
        std::vector<std::uint8_t> unwrappedBaseBlob;
        if (ptr && len >= 12 && ptr[0] == 'P' && ptr[1] == 'B' &&
            ptr[2] == 'Z' && ptr[3] == '1') {
            const std::uint32_t rawLen =
                static_cast<std::uint32_t>(ptr[4]) |
                (static_cast<std::uint32_t>(ptr[5]) << 8) |
                (static_cast<std::uint32_t>(ptr[6]) << 16) |
                (static_cast<std::uint32_t>(ptr[7]) << 24);
            const std::uint32_t compLen =
                static_cast<std::uint32_t>(ptr[8]) |
                (static_cast<std::uint32_t>(ptr[9]) << 8) |
                (static_cast<std::uint32_t>(ptr[10]) << 16) |
                (static_cast<std::uint32_t>(ptr[11]) << 24);
            if (rawLen == 0 || compLen == 0 ||
                12ull + static_cast<unsigned long long>(compLen) !=
                    static_cast<unsigned long long>(len)) {
                std::cerr << "[Columnar] invalid partitioned base zlib wrapper"
                          << std::endl;
                return false;
            }
            unwrappedBaseBlob.resize(rawLen);
            uLongf outLen = static_cast<uLongf>(rawLen);
            const int zret = uncompress(unwrappedBaseBlob.data(), &outLen,
                                        ptr + 12, static_cast<uLong>(compLen));
            if (zret != Z_OK || outLen != rawLen) {
                std::cerr << "[Columnar] partitioned base zlib unwrap failed rc="
                          << zret << std::endl;
                return false;
            }
            ptr = unwrappedBaseBlob.data();
            len = static_cast<int32_t>(unwrappedBaseBlob.size());
        } else if (ptr && len >= 20 && ptr[0] == 'P' && ptr[1] == 'B' &&
                   ptr[2] == 'Z' && ptr[3] == '2') {
            const std::uint32_t rawLen =
                static_cast<std::uint32_t>(ptr[4]) |
                (static_cast<std::uint32_t>(ptr[5]) << 8) |
                (static_cast<std::uint32_t>(ptr[6]) << 16) |
                (static_cast<std::uint32_t>(ptr[7]) << 24);
            const std::uint32_t splitOffset =
                static_cast<std::uint32_t>(ptr[8]) |
                (static_cast<std::uint32_t>(ptr[9]) << 8) |
                (static_cast<std::uint32_t>(ptr[10]) << 16) |
                (static_cast<std::uint32_t>(ptr[11]) << 24);
            const std::uint32_t coordCompLen =
                static_cast<std::uint32_t>(ptr[12]) |
                (static_cast<std::uint32_t>(ptr[13]) << 8) |
                (static_cast<std::uint32_t>(ptr[14]) << 16) |
                (static_cast<std::uint32_t>(ptr[15]) << 24);
            const std::uint32_t groupCompLen =
                static_cast<std::uint32_t>(ptr[16]) |
                (static_cast<std::uint32_t>(ptr[17]) << 8) |
                (static_cast<std::uint32_t>(ptr[18]) << 16) |
                (static_cast<std::uint32_t>(ptr[19]) << 24);
            if (rawLen == 0 || splitOffset == 0 || splitOffset >= rawLen ||
                coordCompLen == 0 || groupCompLen == 0 ||
                20ull + static_cast<unsigned long long>(coordCompLen) +
                    static_cast<unsigned long long>(groupCompLen) !=
                    static_cast<unsigned long long>(len)) {
                std::cerr << "[Columnar] invalid partitioned base split zlib wrapper"
                          << std::endl;
                return false;
            }
            unwrappedBaseBlob.resize(rawLen);
            uLongf coordOutLen = static_cast<uLongf>(splitOffset);
            const int coordRet =
                uncompress(unwrappedBaseBlob.data(), &coordOutLen,
                           ptr + 20, static_cast<uLong>(coordCompLen));
            uLongf groupOutLen = static_cast<uLongf>(rawLen - splitOffset);
            const int groupRet =
                uncompress(unwrappedBaseBlob.data() + splitOffset, &groupOutLen,
                           ptr + 20 + coordCompLen,
                           static_cast<uLong>(groupCompLen));
            if (coordRet != Z_OK || groupRet != Z_OK ||
                coordOutLen != splitOffset || groupOutLen != rawLen - splitOffset) {
                std::cerr << "[Columnar] partitioned base split zlib unwrap failed rc="
                          << coordRet << "," << groupRet << std::endl;
                return false;
            }
            ptr = unwrappedBaseBlob.data();
            len = static_cast<int32_t>(unwrappedBaseBlob.size());
        } else if (ptr && len >= 12 && ptr[0] == 'P' && ptr[1] == 'B' &&
                   ptr[2] == 'Z' && ptr[3] == '3') {
#ifdef PMSF_HAS_ZSTD
            const std::uint32_t rawLen =
                static_cast<std::uint32_t>(ptr[4]) |
                (static_cast<std::uint32_t>(ptr[5]) << 8) |
                (static_cast<std::uint32_t>(ptr[6]) << 16) |
                (static_cast<std::uint32_t>(ptr[7]) << 24);
            const std::uint32_t compLen =
                static_cast<std::uint32_t>(ptr[8]) |
                (static_cast<std::uint32_t>(ptr[9]) << 8) |
                (static_cast<std::uint32_t>(ptr[10]) << 16) |
                (static_cast<std::uint32_t>(ptr[11]) << 24);
            if (rawLen == 0 || compLen == 0 ||
                12ull + static_cast<unsigned long long>(compLen) !=
                    static_cast<unsigned long long>(len)) {
                std::cerr << "[Columnar] invalid partitioned base zstd wrapper"
                          << std::endl;
                return false;
            }
            unwrappedBaseBlob.resize(rawLen);
            const std::size_t outLen =
                ZSTD_decompress(unwrappedBaseBlob.data(), unwrappedBaseBlob.size(),
                                ptr + 12, compLen);
            if (ZSTD_isError(outLen) || outLen != rawLen) {
                std::cerr << "[Columnar] partitioned base zstd unwrap failed"
                          << std::endl;
                return false;
            }
            ptr = unwrappedBaseBlob.data();
            len = static_cast<int32_t>(unwrappedBaseBlob.size());
#else
            std::cerr << "[Columnar] zstd partitioned base wrapper unsupported by this build"
                      << std::endl;
            return false;
#endif
        }
        const bool isPBS2 = ptr && len >= 8 && ptr[0] == 'P' && ptr[1] == 'B' &&
                            ptr[2] == 'S' && ptr[3] == '2';
        const bool isPBS4 = ptr && len >= 8 && ptr[0] == 'P' && ptr[1] == 'B' &&
                            ptr[2] == 'S' && ptr[3] == '4';
        const bool isPBS6 = ptr && len >= 8 && ptr[0] == 'P' && ptr[1] == 'B' &&
                            ptr[2] == 'S' && ptr[3] == '6';
        const bool isPBS7 = ptr && len >= 8 && ptr[0] == 'P' && ptr[1] == 'B' &&
                            ptr[2] == 'S' && ptr[3] == '7';
        const bool isPBS8 = ptr && len >= 8 && ptr[0] == 'P' && ptr[1] == 'B' &&
                            ptr[2] == 'S' && ptr[3] == '8';
        const bool isPBS9 = ptr && len >= 8 && ptr[0] == 'P' && ptr[1] == 'B' &&
                            ptr[2] == 'S' && ptr[3] == '9';
        const bool isPBSA = ptr && len >= 8 && ptr[0] == 'P' && ptr[1] == 'B' &&
                            ptr[2] == 'S' && ptr[3] == 'A';
        if (!isPBS2 && !isPBS4 && !isPBS6 && !isPBS7 && !isPBS8 && !isPBS9 &&
            !isPBSA) {
            std::cerr << "[Columnar] invalid partitioned base superblob" << std::endl;
            return false;
        }
        const std::uint8_t* p = ptr + 4;
        const std::uint8_t* pEnd = ptr + len;
        const int32_t coordCount = static_cast<int32_t>(rdVarU32(p, pEnd));
        const int32_t groupCount = static_cast<int32_t>(rdVarU32(p, pEnd));
        if (coordCount <= 0 || groupCount < 0) return false;
        sharedVertexQX_.assign(static_cast<size_t>(coordCount), 0);
        sharedVertexQY_.assign(static_cast<size_t>(coordCount), 0);
        sharedVertexQZ_.assign(static_cast<size_t>(coordCount), 0);
        sharedVertexX_.clear();
        sharedVertexY_.clear();
        sharedVertexZ_.clear();
        int prev = 0;
        for (int32_t i = 0; i < coordCount; ++i) {
            prev += rdVarI32(p, pEnd);
            sharedVertexQX_[static_cast<size_t>(i)] = prev;
        }
        prev = 0;
        for (int32_t i = 0; i < coordCount; ++i) {
            prev += rdVarI32(p, pEnd);
            sharedVertexQY_[static_cast<size_t>(i)] = prev;
        }
        prev = 0;
        for (int32_t i = 0; i < coordCount; ++i) {
            prev += rdVarI32(p, pEnd);
            sharedVertexQZ_[static_cast<size_t>(i)] = prev;
        }

        baseMeshData_.clear();
        baseMeshData_.reserve(static_cast<size_t>(groupCount));
        sharedBaseMeshCoordinates_ = true;
        auto fillSeedQForData = [&](GroupBaseMeshData& data) {
            auto fillSeedQ = [&](int32_t cid, int32_t& qx, int32_t& qy, int32_t& qz) {
                if (cid >= 0 && static_cast<size_t>(cid) < sharedVertexQX_.size()) {
                    qx = sharedVertexQX_[static_cast<size_t>(cid)];
                    qy = sharedVertexQY_[static_cast<size_t>(cid)];
                    qz = sharedVertexQZ_[static_cast<size_t>(cid)];
                }
            };
            fillSeedQ(data.seedV0CoordId, data.seedV0QX, data.seedV0QY, data.seedV0QZ);
            fillSeedQ(data.seedV1CoordId, data.seedV1QX, data.seedV1QY, data.seedV1QZ);
        };
        if (isPBS8) {
            const int32_t boundaryCount = static_cast<int32_t>(rdVarU32(p, pEnd));
            if (boundaryCount < 0) return false;
            std::vector<int32_t> boundaryCoordIds(static_cast<size_t>(boundaryCount), 0);
            int32_t prevBoundaryCoord = 0;
            for (int32_t i = 0; i < boundaryCount; ++i) {
                prevBoundaryCoord += static_cast<int32_t>(rdVarU32(p, pEnd));
                if (prevBoundaryCoord < 0 || prevBoundaryCoord >= coordCount) return false;
                boundaryCoordIds[static_cast<size_t>(i)] = prevBoundaryCoord;
            }

            for (int32_t i = 0; i < groupCount; ++i) {
                GroupBaseMeshData data{};
                data.groupId = static_cast<int>(rdVarU32(p, pEnd));
                data.numFaces = static_cast<int>(rdVarU32(p, pEnd));
                const std::uint8_t flags = rdU8(p);
                const bool allTri = (flags & 0x1) != 0;
                const bool deltaRefs = (flags & 0x2) != 0;
                const bool signedInterior = (flags & 0x4) != 0;
                const bool signedBoundary = (flags & 0x8) != 0;
                const int32_t interiorCount = static_cast<int32_t>(rdVarU32(p, pEnd));
                const int32_t boundaryLocalCount = static_cast<int32_t>(rdVarU32(p, pEnd));
                const int32_t seed0Ref = static_cast<int32_t>(rdVarU32(p, pEnd));
                const int32_t seed1Ref = static_cast<int32_t>(rdVarU32(p, pEnd));
                const int32_t localCount = interiorCount + boundaryLocalCount;
                if (data.numFaces < 0 || interiorCount < 0 || boundaryLocalCount < 0 ||
                    localCount <= 0 || seed0Ref < 0 || seed1Ref < 0 ||
                    seed0Ref >= localCount || seed1Ref >= localCount) {
                    return false;
                }

                std::vector<int32_t> localCoordIds(static_cast<size_t>(localCount), 0);
                int32_t prevCoord = 0;
                for (int32_t j = 0; j < interiorCount; ++j) {
                    prevCoord += signedInterior
                                     ? rdVarI32(p, pEnd)
                                     : static_cast<int32_t>(rdVarU32(p, pEnd));
                    if (prevCoord < 0 || prevCoord >= coordCount) return false;
                    localCoordIds[static_cast<size_t>(j)] = prevCoord;
                }
                int32_t prevBoundaryIndex = 0;
                for (int32_t j = 0; j < boundaryLocalCount; ++j) {
                    prevBoundaryIndex += signedBoundary
                                             ? rdVarI32(p, pEnd)
                                             : static_cast<int32_t>(rdVarU32(p, pEnd));
                    if (prevBoundaryIndex < 0 || prevBoundaryIndex >= boundaryCount) return false;
                    localCoordIds[static_cast<size_t>(interiorCount + j)] =
                        boundaryCoordIds[static_cast<size_t>(prevBoundaryIndex)];
                }

                data.seedV0CoordId = localCoordIds[static_cast<size_t>(seed0Ref)];
                data.seedV1CoordId = localCoordIds[static_cast<size_t>(seed1Ref)];
                data.faceVerticesAreCoordIds = true;
                data.numVertices = coordCount;
                if (allTri) {
                    data.faceDegrees.assign(static_cast<size_t>(data.numFaces), 3);
                } else {
                    const int32_t degreeCount = static_cast<int32_t>(rdVarU32(p, pEnd));
                    if (degreeCount != data.numFaces) return false;
                    data.faceDegrees.resize(static_cast<size_t>(degreeCount));
                    for (int32_t f = 0; f < degreeCount; ++f) {
                        const int32_t storedDegree = rdVarI32(p, pEnd);
                        if (storedDegree == 0) return false;
                        data.faceDegrees[static_cast<size_t>(f)] = storedDegree;
                    }
                }

                const int32_t cornerCount = static_cast<int32_t>(rdVarU32(p, pEnd));
                if (cornerCount < 0) return false;
                std::vector<int32_t> localFaceRefs;
                if (deltaRefs) {
                    localFaceRefs.assign(static_cast<size_t>(cornerCount), 0);
                    if (allTri && cornerCount == data.numFaces * 3) {
                        for (int corner = 0; corner < 3; ++corner) {
                            int32_t prevRef = 0;
                            for (int f = 0; f < data.numFaces; ++f) {
                                prevRef += rdVarI32(p, pEnd);
                                localFaceRefs[static_cast<size_t>(3 * f + corner)] = prevRef;
                            }
                        }
                    } else {
                        int32_t prevRef = 0;
                        for (int32_t c = 0; c < cornerCount; ++c) {
                            prevRef += rdVarI32(p, pEnd);
                            localFaceRefs[static_cast<size_t>(c)] = prevRef;
                        }
                    }
                } else {
                    const int bits = static_cast<int>(rdU8(p));
                    if (!unpackFixedBits(p, pEnd, bits, cornerCount, localFaceRefs)) {
                        return false;
                    }
                }

                int64_t expectedCorners = 0;
                for (int32_t degree : data.faceDegrees) {
                    expectedCorners += std::abs(degree);
                }
                if (expectedCorners != cornerCount) return false;
                data.faceVertices.clear();
                data.faceVertices.reserve(localFaceRefs.size());
                for (int32_t ref : localFaceRefs) {
                    if (ref < 0 || ref >= localCount) return false;
                    data.faceVertices.push_back(localCoordIds[static_cast<size_t>(ref)]);
                }

                if (targetGroupId_ >= 0 && data.groupId != targetGroupId_) {
                    continue;
                }
                fillSeedQForData(data);
                baseMeshData_.push_back(std::move(data));
            }
        } else if (isPBS9 || isPBSA) {
            for (int32_t i = 0; i < groupCount; ++i) {
                GroupBaseMeshData data{};
                data.groupId = static_cast<int>(rdVarU32(p, pEnd));
                data.numFaces = static_cast<int>(rdVarU32(p, pEnd));
                data.seedV0CoordId = static_cast<int32_t>(rdVarU32(p, pEnd));
                data.seedV1CoordId = static_cast<int32_t>(rdVarU32(p, pEnd));
                const std::uint8_t degreeMode = rdU8(p);
                if (data.numFaces < 0 || data.seedV0CoordId < 0 || data.seedV1CoordId < 0 ||
                    data.seedV0CoordId >= coordCount || data.seedV1CoordId >= coordCount ||
                    degreeMode > 2) {
                    return false;
                }
                data.faceVerticesAreCoordIds = true;
                data.numVertices = coordCount;
                data.faceDegrees.assign(static_cast<size_t>(data.numFaces), 3);
                int64_t expectedCorners = static_cast<int64_t>(data.numFaces) * 3;

                if (degreeMode == 1) {
                    const int32_t nonTriCount = static_cast<int32_t>(rdVarU32(p, pEnd));
                    if (nonTriCount < 0 || nonTriCount > data.numFaces) return false;
                    int32_t prevFaceIndex = 0;
                    for (int32_t j = 0; j < nonTriCount; ++j) {
                        prevFaceIndex += static_cast<int32_t>(rdVarU32(p, pEnd));
                        if (prevFaceIndex < 0 || prevFaceIndex >= data.numFaces) return false;
                        const int32_t degree = static_cast<int32_t>(rdVarU32(p, pEnd)) + 1;
                        if (degree <= 0) return false;
                        data.faceDegrees[static_cast<size_t>(prevFaceIndex)] = degree;
                        expectedCorners += static_cast<int64_t>(degree) - 3;
                    }
                } else if (degreeMode == 2) {
                    const int32_t degreeCount = static_cast<int32_t>(rdVarU32(p, pEnd));
                    if (degreeCount != data.numFaces) return false;
                    expectedCorners = 0;
                    for (int32_t f = 0; f < degreeCount; ++f) {
                        const int32_t degree = static_cast<int32_t>(rdVarU32(p, pEnd)) + 1;
                        if (degree <= 0) return false;
                        data.faceDegrees[static_cast<size_t>(f)] = degree;
                        expectedCorners += degree;
                    }
                }

                const int32_t cornerCount = static_cast<int32_t>(rdVarU32(p, pEnd));
                if (cornerCount < 0 || expectedCorners != cornerCount) return false;
                data.faceVertices.assign(static_cast<size_t>(cornerCount), 0);
                int32_t prevAnchor = 0;
                std::size_t out = 0;
                std::vector<int32_t> prevFaceVerts;
                for (int32_t f = 0; f < data.numFaces; ++f) {
                    const int32_t degree = data.faceDegrees[static_cast<std::size_t>(f)];
                    const int32_t anchorDelta = static_cast<int32_t>(rdVarU32(p, pEnd));
                    prevAnchor += anchorDelta;
                    if (prevAnchor < 0 || prevAnchor >= coordCount) return false;
                    const bool reusePrevFace = isPBSA && anchorDelta == 0 &&
                                               prevFaceVerts.size() ==
                                                   static_cast<std::size_t>(degree);
                    std::vector<int32_t> faceVerts;
                    faceVerts.reserve(static_cast<std::size_t>(degree));
                    data.faceVertices[out++] = prevAnchor;
                    faceVerts.push_back(prevAnchor);
                    for (int32_t j = 1; j < degree; ++j) {
                        const int32_t coord =
                            reusePrevFace
                                ? prevFaceVerts[static_cast<std::size_t>(j)] +
                                      rdVarI32(p, pEnd)
                                : prevAnchor + static_cast<int32_t>(rdVarU32(p, pEnd));
                        if (coord < 0 || coord >= coordCount) return false;
                        data.faceVertices[out++] = coord;
                        faceVerts.push_back(coord);
                    }
                    prevFaceVerts.swap(faceVerts);
                }
                if (out != data.faceVertices.size()) return false;
                if (targetGroupId_ >= 0 && data.groupId != targetGroupId_) {
                    continue;
                }
                fillSeedQForData(data);
                baseMeshData_.push_back(std::move(data));
            }
        } else
        if (isPBS4 || isPBS6 || isPBS7) {
            for (int32_t i = 0; i < groupCount; ++i) {
                GroupBaseMeshData data{};
                data.groupId = static_cast<int>(rdVarU32(p, pEnd));
                data.numFaces = static_cast<int>(rdVarU32(p, pEnd));
                data.seedV0CoordId = static_cast<int32_t>(rdVarU32(p, pEnd));
                data.seedV1CoordId = static_cast<int32_t>(rdVarU32(p, pEnd));
                if (data.numFaces < 0 || data.seedV0CoordId < 0 || data.seedV1CoordId < 0) {
                    return false;
                }
                data.faceVerticesAreCoordIds = true;
                data.numVertices = coordCount;
                int32_t degreeCount = data.numFaces;
                std::uint8_t degreeMode = 2;
                if (isPBS6) {
                    degreeMode = rdU8(p);
                }
                data.faceDegrees.assign(static_cast<size_t>(degreeCount), 0);
                int64_t expectedCorners = 0;
                if (isPBS7) {
                    const int32_t storedDegreeCount = static_cast<int32_t>(rdVarU32(p, pEnd));
                    if (storedDegreeCount != data.numFaces) return false;
                    for (int32_t f = 0; f < degreeCount; ++f) {
                        const int32_t degree = static_cast<int32_t>(rdVarU32(p, pEnd)) + 1;
                        if (degree <= 0) return false;
                        data.faceDegrees[static_cast<size_t>(f)] = degree;
                        expectedCorners += degree;
                    }
                } else if (isPBS6 && degreeMode == 0) {
                    for (int32_t f = 0; f < degreeCount; ++f) {
                        data.faceDegrees[static_cast<size_t>(f)] = 3;
                    }
                    expectedCorners = static_cast<int64_t>(degreeCount) * 3;
                } else if (isPBS6 && degreeMode == 1) {
                    const std::size_t signBytes =
                        (static_cast<std::size_t>(degreeCount) + 7u) / 8u;
                    if (p + signBytes > pEnd) return false;
                    for (int32_t f = 0; f < degreeCount; ++f) {
                        const std::uint8_t b = p[static_cast<std::size_t>(f) / 8u];
                        const bool neg = ((b >> (static_cast<std::size_t>(f) % 8u)) & 1u) != 0;
                        data.faceDegrees[static_cast<size_t>(f)] = neg ? -3 : 3;
                    }
                    p += signBytes;
                    expectedCorners = static_cast<int64_t>(degreeCount) * 3;
                } else {
                    if (!isPBS6) {
                        degreeCount = static_cast<int32_t>(rdVarU32(p, pEnd));
                        if (degreeCount != data.numFaces) return false;
                        data.faceDegrees.assign(static_cast<size_t>(degreeCount), 0);
                    } else if (degreeMode != 2) {
                        return false;
                    } else {
                        const int32_t storedDegreeCount = static_cast<int32_t>(rdVarU32(p, pEnd));
                        if (storedDegreeCount != data.numFaces) return false;
                    }
                    for (int32_t f = 0; f < degreeCount; ++f) {
                        const int32_t storedDegree = rdVarI32(p, pEnd);
                        const int32_t degree = std::abs(storedDegree);
                        if (degree <= 0) return false;
                        data.faceDegrees[static_cast<size_t>(f)] = storedDegree;
                        expectedCorners += degree;
                    }
                }
                const int32_t cornerCount =
                    (isPBS6 && degreeMode != 2)
                        ? static_cast<int32_t>(expectedCorners)
                        : static_cast<int32_t>(rdVarU32(p, pEnd));
                if (cornerCount < 0 || expectedCorners != cornerCount) return false;
                data.faceVertices.assign(static_cast<size_t>(cornerCount), 0);
                if (isPBS7) {
                    int32_t prevAnchor = 0;
                    std::size_t out = 0;
                    for (int32_t f = 0; f < degreeCount; ++f) {
                        const int32_t degree = data.faceDegrees[static_cast<std::size_t>(f)];
                        prevAnchor += static_cast<int32_t>(rdVarU32(p, pEnd));
                        if (prevAnchor < 0 || prevAnchor >= coordCount) return false;
                        data.faceVertices[out++] = prevAnchor;
                        for (int32_t j = 1; j < degree; ++j) {
                            const int32_t coord = prevAnchor + static_cast<int32_t>(rdVarU32(p, pEnd));
                            if (coord < 0 || coord >= coordCount) return false;
                            data.faceVertices[out++] = coord;
                        }
                    }
                    if (out != data.faceVertices.size()) return false;
                } else {
                    int32_t prevCoord = 0;
                    for (int32_t c = 0; c < cornerCount; ++c) {
                        prevCoord += rdVarI32(p, pEnd);
                        if (prevCoord < 0 || prevCoord >= coordCount) return false;
                        data.faceVertices[static_cast<size_t>(c)] = prevCoord;
                    }
                }
                if (targetGroupId_ >= 0 && data.groupId != targetGroupId_) {
                    continue;
                }
                fillSeedQForData(data);
                baseMeshData_.push_back(std::move(data));
            }
        } else {
            for (int32_t i = 0; i < groupCount; ++i) {
                const int gid = static_cast<int>(rdVarU32(p, pEnd));
                const int32_t groupLen = static_cast<int32_t>(rdVarU32(p, pEnd));
                if (groupLen <= 0 || p + groupLen > pEnd) return false;
                if (targetGroupId_ >= 0 && gid != targetGroupId_) {
                    p += groupLen;
                    continue;
                }
                GroupBaseMeshData data{};
                data.groupId = gid;
                data.faceVerticesAreCoordIds = true;
                if (!decodePartitionBaseBlob(p, groupLen, data)) {
                    std::cerr << "[Columnar] partitioned base superblob group decode failed for group "
                              << gid << std::endl;
                    return false;
                }
                p += groupLen;
                fillSeedQForData(data);
                baseMeshData_.push_back(std::move(data));
            }
        }
        if (parquetV2VerboseLogs()) {
            std::cout << "[Columnar] partitioned base superblob: vertices="
                      << sharedVertexQX_.size()
                      << " groups=" << baseMeshData_.size() << std::endl;
        }
        return true;
    }

    const std::string verticesPath = pmsf::paperVerticesTablePath(baseName);
    const std::string partitionsPath = pmsf::paperPartitionsTablePath(baseName);
    if (!std::filesystem::exists(verticesPath) || !std::filesystem::exists(partitionsPath)) {
        return false;
    }

    auto vertexTable = readParquetFile(verticesPath);
    auto partitionTable = readParquetFile(partitionsPath);
    if (!vertexTable || !partitionTable) return false;

    auto firstCol = vertexTable->GetColumnByName(pmsf::kParquetFirstCoordIdColumn);
    auto countCol = vertexTable->GetColumnByName(pmsf::kParquetCountColumn);
    auto blobCol = vertexTable->GetColumnByName(pmsf::kParquetCoordBlobColumn);
    if (!firstCol || !countCol || !blobCol || blobCol->num_chunks() == 0) {
        std::cerr << "[Columnar] partitioned base vertices table missing columns" << std::endl;
        return false;
    }
    auto firstA = std::static_pointer_cast<arrow::Int32Array>(firstCol->chunk(0));
    auto countA = std::static_pointer_cast<arrow::Int32Array>(countCol->chunk(0));
    auto blobA = std::static_pointer_cast<arrow::BinaryArray>(blobCol->chunk(0));

    sharedVertexQX_.clear();
    sharedVertexQY_.clear();
    sharedVertexQZ_.clear();
    sharedVertexX_.clear();
    sharedVertexY_.clear();
    sharedVertexZ_.clear();

    for (int64_t row = 0; row < vertexTable->num_rows(); ++row) {
        const int32_t first = firstA->Value(row);
        const int32_t count = countA->Value(row);
        int32_t len = 0;
        const auto* ptr = blobA->GetValue(row, &len);
        std::vector<int32_t> qx, qy, qz;
        if (!decodeQuantCoordBlock(ptr, len, count, qx, qy, qz)) {
            std::cerr << "[Columnar] partitioned base coord blob decode failed at row "
                      << row << std::endl;
            return false;
        }
        if (first < 0 || count < 0) return false;
        const size_t needed = static_cast<size_t>(first + count);
        if (sharedVertexQX_.size() < needed) {
            sharedVertexQX_.resize(needed);
            sharedVertexQY_.resize(needed);
            sharedVertexQZ_.resize(needed);
        }
        for (int32_t i = 0; i < count; ++i) {
            const size_t dst = static_cast<size_t>(first + i);
            sharedVertexQX_[dst] = qx[static_cast<size_t>(i)];
            sharedVertexQY_[dst] = qy[static_cast<size_t>(i)];
            sharedVertexQZ_[dst] = qz[static_cast<size_t>(i)];
        }
    }

    auto groupCol = partitionTable->GetColumnByName(pmsf::kParquetGroupIdColumn);
    auto numVertCol = partitionTable->GetColumnByName(pmsf::kParquetNumBaseVerticesColumn);
    auto numFaceCol = partitionTable->GetColumnByName(pmsf::kParquetNumBaseFacesColumn);
    auto seed0Col = partitionTable->GetColumnByName(pmsf::kParquetSeedV0CoordIdColumn);
    auto seed1Col = partitionTable->GetColumnByName(pmsf::kParquetSeedV1CoordIdColumn);
    auto modeCol = partitionTable->GetColumnByName(pmsf::kParquetBaseModeColumn);
    auto baseBlobCol = partitionTable->GetColumnByName(pmsf::kParquetBaseBlobColumn);
    if (!groupCol || !numVertCol || !numFaceCol || !seed0Col || !seed1Col ||
        !modeCol || !baseBlobCol || baseBlobCol->num_chunks() == 0) {
        std::cerr << "[Columnar] partitioned base partitions table missing columns" << std::endl;
        return false;
    }

    auto groupA = std::static_pointer_cast<arrow::Int32Array>(groupCol->chunk(0));
    auto numVertA = std::static_pointer_cast<arrow::Int32Array>(numVertCol->chunk(0));
    auto numFaceA = std::static_pointer_cast<arrow::Int32Array>(numFaceCol->chunk(0));
    auto seed0A = std::static_pointer_cast<arrow::Int32Array>(seed0Col->chunk(0));
    auto seed1A = std::static_pointer_cast<arrow::Int32Array>(seed1Col->chunk(0));
    auto modeA = std::static_pointer_cast<arrow::Int32Array>(modeCol->chunk(0));
    auto baseBlobA = std::static_pointer_cast<arrow::BinaryArray>(baseBlobCol->chunk(0));

    baseMeshData_.clear();
    sharedBaseMeshCoordinates_ = true;
    for (int64_t row = 0; row < partitionTable->num_rows(); ++row) {
        const int gid = groupA->Value(row);
        if (targetGroupId_ >= 0 && gid != targetGroupId_) continue;
        if (modeA->Value(row) != 1) {
            std::cerr << "[Columnar] unsupported partition base mode: "
                      << modeA->Value(row) << std::endl;
            return false;
        }

        GroupBaseMeshData data{};
        data.groupId = gid;
        data.numVertices = numVertA->Value(row);
        data.numFaces = numFaceA->Value(row);
        data.seedV0CoordId = seed0A->Value(row);
        data.seedV1CoordId = seed1A->Value(row);
        data.faceVerticesAreCoordIds = true;

        int32_t len = 0;
        const auto* ptr = baseBlobA->GetValue(row, &len);
        if (!decodePartitionBaseBlob(ptr, len, data)) {
            std::cerr << "[Columnar] partitioned base blob decode failed for group "
                      << gid << std::endl;
            return false;
        }
        auto fillSeedQ = [&](int32_t cid, int32_t& qx, int32_t& qy, int32_t& qz) {
            if (cid >= 0 && static_cast<size_t>(cid) < sharedVertexQX_.size()) {
                qx = sharedVertexQX_[static_cast<size_t>(cid)];
                qy = sharedVertexQY_[static_cast<size_t>(cid)];
                qz = sharedVertexQZ_[static_cast<size_t>(cid)];
            }
        };
        fillSeedQ(data.seedV0CoordId, data.seedV0QX, data.seedV0QY, data.seedV0QZ);
        fillSeedQ(data.seedV1CoordId, data.seedV1QX, data.seedV1QY, data.seedV1QZ);
        baseMeshData_.push_back(std::move(data));
    }

    if (parquetV2VerboseLogs()) {
        std::cout << "[Columnar] partitioned base: vertices=" << sharedVertexQX_.size()
                  << " groups=" << baseMeshData_.size() << std::endl;
    }
    return true;
}

bool ColumnarDeserializeOperator::initPaperV2Native(
    const std::string& compactBinPath,
    std::vector<char> nativeBlob,
    std::string* error) {
    auto fail = [&](const std::string& message) {
        if (error) *error = message;
        return false;
    };
    if (nativeBlob.empty()) {
        return fail("native payload is empty");
    }

    // Reset the same transient decode state init() would reset.
    currentRound_ = 0;
    origTotalVertices_ = 0;
    origTotalFaces_ = 0;
    baseMeshData_.clear();
    symbolData_.clear();
    boundaryGraph_.reset();

    // Compact base payload is the single source of truth: header, base mesh,
    // codec version, blob layout, quantization grid and original mesh counts.
    if (!readV2CompactBin(compactBinPath)) {
        return fail("v2 compact base payload parse failed: " + compactBinPath);
    }

    rcBlob_ = std::move(nativeBlob);
    // Native payloads written by PaperSerializeOperator v2 are raw (the bridge
    // strips the SBC1 wrapper at encode time); tolerate wrapped blobs too so a
    // canonical residual blob could be fed in unchanged.
    if (!unwrapSymbolBlobIfNeeded(rcBlob_)) {
        rcBlob_.clear();
        return fail("native symbol blob envelope is invalid");
    }
    rcBlobOffset_ = 0;
    rcBlobInitialized_ = false;
    if (error) error->clear();
    return true;
}

bool ColumnarDeserializeOperator::initPaperV2Typed(
    const std::string& compactBinPath,
    pmsf::PMSFSymbolTable symbols,
    std::string* error) {
    auto fail = [&](const std::string& message) {
        if (error) *error = message;
        return false;
    };
    currentRound_ = 0;
    origTotalVertices_ = 0;
    origTotalFaces_ = 0;
    baseMeshData_.clear();
    symbolData_.clear();
    boundaryGraph_.reset();
    if (!readV2CompactBin(compactBinPath)) {
        return fail("v2 compact base payload parse failed: " + compactBinPath);
    }
    paperV2Typed_ = true;
    rcBlob_.clear();
    rcBlobOffset_ = 0;
    rcBlobInitialized_ = true;
    setPMSFSymbolTable(std::move(symbols));
    if (auto* ppmcReader =
            dynamic_cast<PPMCSymbolReadOperator*>(symbolReadOperator_.get())) {
        ppmcReader->setPaperTypedTableMode(true);
        ppmcReader->setMesh(mesh_);
    }
    if (error) error->clear();
    return true;
}

bool ColumnarDeserializeOperator::readV2CompactBin(const std::string& path) {
    pmsf::PMSFCompactBinEnvelope envelope;
    if (!pmsf::readPMSFCompactBinEnvelopeFile(
            path, "[Columnar]", envelope)) {
        return false;
    }
    return readV2CompactPayloadBytes(
        envelope.payload,
        envelope.rawLen,
        envelope.compLen,
        path);
}

bool ColumnarDeserializeOperator::readV2CompactBinPayload(
    const std::vector<std::uint8_t>& compactBin,
    const std::string& sourceLabel,
    bool quietMagicMismatch) {
    pmsf::PMSFCompactBinEnvelope envelope;
    if (!pmsf::readPMSFCompactBinEnvelopeBytes(
            compactBin, sourceLabel, "[Columnar]", envelope)) {
        return false;
    }

    return readV2CompactPayloadBytes(
        envelope.payload,
        envelope.rawLen,
        envelope.compLen,
        sourceLabel,
        quietMagicMismatch);
}

bool ColumnarDeserializeOperator::readV2CompactPayloadBytes(
    const std::vector<std::uint8_t>& payload,
    std::uint32_t rawLen,
    std::uint32_t compLen,
    const std::string& sourceLabel,
    bool quietMagicMismatch) {
    (void)sourceLabel;
    const std::uint8_t* p    = payload.data();
    const std::uint8_t* pEnd = payload.data() + rawLen;

    // magic "V2CP"
    if (rawLen < 8 || p[0] != 'V' || p[1] != '2' || p[2] != 'C' || p[3] != 'P') {
        if (!quietMagicMismatch) {
            std::cerr << "[Columnar] compact bin magic mismatch (expected V2CP)"
                      << std::endl;
        }
        return false;
    }
    p += 4;
    const auto compactHeader =
        pmsf::readPaperCompactPayloadHeader(p, /*hasSecondFlagByte=*/true);
    const bool hasQuant = compactHeader.hasQuant;
    const bool hasSeg = compactHeader.hasSeg;
    const bool compactU16 = compactHeader.compactU16;
    const bool triFaces = compactHeader.triFaces;
    const bool varintBase = compactHeader.varintBase;
    const bool edgeBreakerBase = compactHeader.edgeBreakerBase;
    const bool nativeEdgeBreakerBase = compactHeader.nativeEdgeBreakerBase;
    const bool splitResidualBase = compactHeader.splitResidualBase;
    const bool varintFaceDegrees = (compactHeader.flagBits2 & 0x04) != 0;
    codecVersion_ = compactHeader.codecVersion;
    blobLayout_ = compactHeader.blobLayout;
    canonicalSourceLayout_ = compactHeader.canonicalSourceLayout;
    origTotalVertices_ = compactHeader.origTotalVertices;
    origTotalFaces_ = compactHeader.origTotalFaces;

    // Phase S0（R13）：按产物自描述再实例化存储方案。旧产物无该字段 ⇒ 原始方案。
    // 名字未注册 / 版本不符一律**报错返回**，不静默回落（设计文档 §3.3）。
    {
        std::string why;
        if (!::pmsf::activateStorageSchemeFromArtifact(
                compactHeader.storageSchemeName, compactHeader.storageSchemeVersion, &why)) {
            std::cerr << "[Columnar] 存储方案无法激活：" << why << std::endl;
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
    header_.setEnableTriangulation((compactHeader.flagBits & 0x20) != 0);
    header_.setEnableConnectivityPredictionFaces(
        (compactHeader.flagBits & 0x40) != 0);
    header_.setEnableConnectivityPredictionEdges(
        (compactHeader.flagBits & 0x80) != 0);
    header_.setEnableTriangleMeshBalance(
        (compactHeader.flagBits2 & 0x01) != 0);
    header_.setEnableCurvaturePrediction(
        (compactHeader.flagBits2 & 0x02) != 0);

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
    const int32_t nGroups = rdI32LE(p);
    baseMeshData_.clear();
    baseMeshData_.reserve(nGroups);
    for (int g = 0; g < nGroups; ++g) {
        GroupBaseMeshData data{};
        data.groupId     = rdI32LE(p);
        data.numVertices = rdI32LE(p);
        data.numFaces    = rdI32LE(p);
        if (nativeEdgeBreakerBase) {
            if (!hasQuant) {
                std::cerr << "[Columnar] native EdgeBreaker compact base requires quantization" << std::endl;
                return false;
            }
            data.seedV0QX = rdI32LE(p); data.seedV0QY = rdI32LE(p); data.seedV0QZ = rdI32LE(p);
            data.seedV1QX = rdI32LE(p); data.seedV1QY = rdI32LE(p); data.seedV1QZ = rdI32LE(p);
            data.seedV0X = bboxMinX_ + (data.seedV0QX + 0.5f) * quantStep_;
            data.seedV0Y = bboxMinY_ + (data.seedV0QY + 0.5f) * quantStep_;
            data.seedV0Z = bboxMinZ_ + (data.seedV0QZ + 0.5f) * quantStep_;
            data.seedV1X = bboxMinX_ + (data.seedV1QX + 0.5f) * quantStep_;
            data.seedV1Y = bboxMinY_ + (data.seedV1QY + 0.5f) * quantStep_;
            data.seedV1Z = bboxMinZ_ + (data.seedV1QZ + 0.5f) * quantStep_;
            const int32_t ebLen = rdI32LE(p);
            if (ebLen <= 0 || p + ebLen > pEnd) {
                std::cerr << "[Columnar] invalid native EdgeBreaker base payload" << std::endl;
                return false;
            }
            std::vector<std::uint8_t> beb(p, p + ebLen);
            p += ebLen;
            if (!decodeNativeEdgeBreakerBase(beb, data,
                                             bboxMinX_, bboxMinY_, bboxMinZ_, quantStep_)) {
                std::cerr << "[Columnar] native EdgeBreaker base decode failed" << std::endl;
                return false;
            }
            baseMeshData_.push_back(std::move(data));
            continue;
        }
        if (edgeBreakerBase) {
            if (!hasQuant) {
                std::cerr << "[Columnar] EdgeBreaker compact base requires quantization" << std::endl;
                return false;
            }
            data.seedV0QX = rdI32LE(p); data.seedV0QY = rdI32LE(p); data.seedV0QZ = rdI32LE(p);
            data.seedV1QX = rdI32LE(p); data.seedV1QY = rdI32LE(p); data.seedV1QZ = rdI32LE(p);
            data.seedV0X = bboxMinX_ + (data.seedV0QX + 0.5f) * quantStep_;
            data.seedV0Y = bboxMinY_ + (data.seedV0QY + 0.5f) * quantStep_;
            data.seedV0Z = bboxMinZ_ + (data.seedV0QZ + 0.5f) * quantStep_;
            data.seedV1X = bboxMinX_ + (data.seedV1QX + 0.5f) * quantStep_;
            data.seedV1Y = bboxMinY_ + (data.seedV1QY + 0.5f) * quantStep_;
            data.seedV1Z = bboxMinZ_ + (data.seedV1QZ + 0.5f) * quantStep_;
            const int32_t ebLen = rdI32LE(p);
            if (ebLen <= 0 || p + ebLen > pEnd) {
                std::cerr << "[Columnar] invalid EdgeBreaker base payload" << std::endl;
                return false;
            }
            std::vector<std::uint8_t> beb(p, p + ebLen);
            p += ebLen;
            if (!decodeEdgeBreakerBase(beb, data,
                                       bboxMinX_, bboxMinY_, bboxMinZ_, quantStep_)) {
                std::cerr << "[Columnar] EdgeBreaker base decode failed" << std::endl;
                return false;
            }
            baseMeshData_.push_back(std::move(data));
            continue;
        }
        if (hasQuant) {
            if (varintBase) {
                data.seedV0QX = static_cast<int>(rdVarU32(p, pEnd));
                data.seedV0QY = static_cast<int>(rdVarU32(p, pEnd));
                data.seedV0QZ = static_cast<int>(rdVarU32(p, pEnd));
                data.seedV1QX = static_cast<int>(rdVarU32(p, pEnd));
                data.seedV1QY = static_cast<int>(rdVarU32(p, pEnd));
                data.seedV1QZ = static_cast<int>(rdVarU32(p, pEnd));
            } else if (compactU16) {
                data.seedV0QX = rdU16LE(p); data.seedV0QY = rdU16LE(p); data.seedV0QZ = rdU16LE(p);
                data.seedV1QX = rdU16LE(p); data.seedV1QY = rdU16LE(p); data.seedV1QZ = rdU16LE(p);
            } else {
                data.seedV0QX = rdI32LE(p); data.seedV0QY = rdI32LE(p); data.seedV0QZ = rdI32LE(p);
                data.seedV1QX = rdI32LE(p); data.seedV1QY = rdI32LE(p); data.seedV1QZ = rdI32LE(p);
            }
            // seed 浮点：从量化反算（与 readBaseMeshTable 行为一致）
            data.seedV0X = bboxMinX_ + (data.seedV0QX + 0.5f) * quantStep_;
            data.seedV0Y = bboxMinY_ + (data.seedV0QY + 0.5f) * quantStep_;
            data.seedV0Z = bboxMinZ_ + (data.seedV0QZ + 0.5f) * quantStep_;
            data.seedV1X = bboxMinX_ + (data.seedV1QX + 0.5f) * quantStep_;
            data.seedV1Y = bboxMinY_ + (data.seedV1QY + 0.5f) * quantStep_;
            data.seedV1Z = bboxMinZ_ + (data.seedV1QZ + 0.5f) * quantStep_;
            const int vlen = data.numVertices;
            if (vlen > 0) {
                data.vertexQX.resize(vlen);
                data.vertexQY.resize(vlen);
                data.vertexQZ.resize(vlen);
                if (splitResidualBase) {
                    int prevQX = data.seedV1QX;
                    for (int i = 0; i < vlen; ++i) {
                        prevQX += rdVarI32(p, pEnd);
                        data.vertexQX[i] = prevQX;
                    }
                    int prevQY = data.seedV1QY;
                    for (int i = 0; i < vlen; ++i) {
                        prevQY += rdVarI32(p, pEnd);
                        data.vertexQY[i] = prevQY;
                    }
                    int prevQZ = data.seedV1QZ;
                    for (int i = 0; i < vlen; ++i) {
                        prevQZ += rdVarI32(p, pEnd);
                        data.vertexQZ[i] = prevQZ;
                    }
                } else {
                    int prevQX = data.seedV1QX;
                    int prevQY = data.seedV1QY;
                    int prevQZ = data.seedV1QZ;
                    for (int i = 0; i < vlen; ++i) {
                        if (varintBase) {
                            prevQX += rdVarI32(p, pEnd);
                            prevQY += rdVarI32(p, pEnd);
                            prevQZ += rdVarI32(p, pEnd);
                            data.vertexQX[i] = prevQX;
                            data.vertexQY[i] = prevQY;
                            data.vertexQZ[i] = prevQZ;
                        } else if (compactU16) {
                            data.vertexQX[i] = rdU16LE(p);
                            data.vertexQY[i] = rdU16LE(p);
                            data.vertexQZ[i] = rdU16LE(p);
                        } else {
                            data.vertexQX[i] = rdI32LE(p);
                            data.vertexQY[i] = rdI32LE(p);
                            data.vertexQZ[i] = rdI32LE(p);
                        }
                    }
                }
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
            data.seedV0X = rdF32LE(p); data.seedV0Y = rdF32LE(p); data.seedV0Z = rdF32LE(p);
            data.seedV1X = rdF32LE(p); data.seedV1Y = rdF32LE(p); data.seedV1Z = rdF32LE(p);
            const int vlen = data.numVertices;
            if (vlen > 0) {
                data.vertexX.resize(vlen);
                data.vertexY.resize(vlen);
                data.vertexZ.resize(vlen);
                for (int i = 0; i < vlen; ++i) {
                    data.vertexX[i] = rdF32LE(p);
                    data.vertexY[i] = rdF32LE(p);
                    data.vertexZ[i] = rdF32LE(p);
                }
            }
        }
        if (triFaces) {
            data.faceDegrees.assign(static_cast<size_t>(data.numFaces), 3);
        } else if (varintFaceDegrees) {
            const int32_t nDegrees = static_cast<int32_t>(rdVarU32(p, pEnd));
            data.faceDegrees.resize(nDegrees);
            for (int i = 0; i < nDegrees; ++i) data.faceDegrees[i] = static_cast<int>(rdVarI32(p, pEnd));
        } else {
            const int32_t nDegrees = rdI32LE(p);
            data.faceDegrees.resize(nDegrees);
            for (int i = 0; i < nDegrees; ++i) data.faceDegrees[i] = rdI32LE(p);
        }
        const int32_t nFaceVerts = rdI32LE(p);
        data.faceVertices.resize(nFaceVerts);
        if (splitResidualBase && triFaces && nFaceVerts == data.numFaces * 3) {
            for (int corner = 0; corner < 3; ++corner) {
                int prevFaceVertex = 0;
                for (int f = 0; f < data.numFaces; ++f) {
                    prevFaceVertex += rdVarI32(p, pEnd);
                    data.faceVertices[static_cast<std::size_t>(3 * f + corner)] = prevFaceVertex;
                }
            }
        } else {
            int prevFaceVertex = 0;
            for (int i = 0; i < nFaceVerts; ++i) {
                if (varintBase) {
                    prevFaceVertex += rdVarI32(p, pEnd);
                    data.faceVertices[i] = prevFaceVertex;
                } else {
                    data.faceVertices[i] = compactU16 ? rdU16LE(p) : rdI32LE(p);
                }
            }
        }

        baseMeshData_.push_back(std::move(data));
    }

    if (p > pEnd) {
        std::cerr << "[Columnar] compact bin 读越界" << std::endl;
        return false;
    }
    if (parquetV2VerboseLogs()) {
        std::cout << "[Columnar] compact bin: rawLen=" << rawLen
                  << " compLen=" << compLen
                  << " round=" << compactHeader.round
                  << " groups=" << nGroups
                  << std::endl;
    }
    return true;
}
