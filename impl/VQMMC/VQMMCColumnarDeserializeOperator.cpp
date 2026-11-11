/*
 * VQMMCColumnarDeserializeOperator 实现 — 见 header 注释。
 *
 * 主流程：
 *   1) 读 4 个 parquet 表（metadata / base_mesh / symbols / vq）。
 *   2) 在内存里**还原 inner 二进制 buffer**，与 VQMMC + BasicSerializeOperator
 *      原本会写到 .bin 的字节布局逐字节一致（base mesh 段 + 3 段 zlib block
 *      with [origSize, compSize, data] 头 + 同顺序的 vq 数据）。
 *   3) 把还原 buffer 注入 inner VQMMCDeserializeOperator，剩下的解压全权委托。
 *
 * 这样做让我们 column-native 的写入端真实存在多列 schema（用户 / SQL 工具直接
 * 投影），同时不需要重写 `MCGAL::readBaseMeshWithQuantization` /
 * `VQMMCSymbolReadOperator` 等几百行验证过的解码代码。
 */
#include "VQMMCColumnarDeserializeOperator.h"
#include "storage/CompactSegmentDirectory.h"

#include "VQMMCDeserializeOperator.h"
#include "VQMMC/common/VQMMCQuantization.h"
#include "common/AlphabetStreamCodec.h"
#include "BufferUtils.h"
#include "Configuration.h"
#include "EntropyCodec.h"
#include "header/PMSFHeader.h"
#include "PMSFPayloadAdapterRegistry.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef COLUMNAR_SUPPORT
#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/reader.h>
#endif

namespace {

std::string stripParquetExt(const std::string& p) {
    std::string base = p;
    auto slash = base.find_last_of("/\\");
    auto dot = base.rfind('.');
    if (dot != std::string::npos && (slash == std::string::npos || dot > slash)) {
        base = base.substr(0, dot);
    }
    return base;
}

#ifdef COLUMNAR_SUPPORT
bool fileExists(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    return f.good();
}

std::shared_ptr<arrow::Table> readParquetTable(const std::string& path) {
    auto fr = arrow::io::ReadableFile::Open(path);
    if (!fr.ok()) {
        std::cerr << "[VQMMC-PV2] open failed: " << path << " " << fr.status().ToString() << std::endl;
        return nullptr;
    }
    auto file = fr.ValueOrDie();
    std::unique_ptr<parquet::arrow::FileReader> reader;
    auto st = parquet::arrow::OpenFile(file, arrow::default_memory_pool()).Value(&reader);
    if (!st.ok()) {
        std::cerr << "[VQMMC-PV2] reader open failed: " << st.ToString() << std::endl;
        return nullptr;
    }
    std::shared_ptr<arrow::Table> table;
    st = reader->ReadTable(&table);
    if (!st.ok()) {
        std::cerr << "[VQMMC-PV2] readTable failed: " << st.ToString() << std::endl;
        return nullptr;
    }
    auto combined = table->CombineChunks(arrow::default_memory_pool());
    if (!combined.ok()) {
        std::cerr << "[VQMMC-PV2] combine chunks failed: "
                  << combined.status().ToString() << std::endl;
        return nullptr;
    }
    return combined.ValueOrDie();
}

// R8：同一张表也可能来自 `_compact.bin` 的段（内存字节）。
// 字节与磁盘上的 parquet 完全同构（写入端用同一套 writer properties），
// 故这里只是把"从文件读"换成"从 BufferReader 读"，其余完全一致。
std::shared_ptr<arrow::Table> readParquetTableFromBytes(
    const std::vector<std::uint8_t>& bytes, const std::string& label) {
    if (bytes.empty()) return nullptr;
    auto buf = std::make_shared<arrow::Buffer>(bytes.data(),
                                               static_cast<int64_t>(bytes.size()));
    auto file = std::make_shared<arrow::io::BufferReader>(buf);
    std::unique_ptr<parquet::arrow::FileReader> reader;
    auto st = parquet::arrow::OpenFile(file, arrow::default_memory_pool()).Value(&reader);
    if (!st.ok()) {
        std::cerr << "[VQMMC-PV2] segment reader open failed: " << label << " "
                  << st.ToString() << std::endl;
        return nullptr;
    }
    std::shared_ptr<arrow::Table> table;
    st = reader->ReadTable(&table);
    if (!st.ok()) {
        std::cerr << "[VQMMC-PV2] segment readTable failed: " << label << " "
                  << st.ToString() << std::endl;
        return nullptr;
    }
    auto combined = table->CombineChunks(arrow::default_memory_pool());
    if (!combined.ok()) {
        std::cerr << "[VQMMC-PV2] combine segment chunks failed: " << label << " "
                  << combined.status().ToString() << std::endl;
        return nullptr;
    }
    return combined.ValueOrDie();
}

// 取一个 scalar 列在 row=0 的值
template <typename T, typename ArrowArrT>
T scalar(const std::shared_ptr<arrow::Table>& t, const char* name, T def) {
    (void)def;
    if (!t) {
        throw std::runtime_error(std::string("[VQMMC-PV2] missing table for ") + name);
    }
    auto col = t->GetColumnByName(name);
    if (!col || col->num_chunks() != 1) {
        throw std::runtime_error(std::string("[VQMMC-PV2] invalid scalar column: ") + name);
    }
    auto arr = std::dynamic_pointer_cast<ArrowArrT>(col->chunk(0));
    if (!arr || arr->length() != 1 || arr->IsNull(0)) {
        throw std::runtime_error(std::string("[VQMMC-PV2] invalid scalar value: ") + name);
    }
    return static_cast<T>(arr->Value(0));
}

// 读 string scalar
std::string scalarString(const std::shared_ptr<arrow::Table>& t, const char* name) {
    auto col = t->GetColumnByName(name);
    if (!col || col->num_chunks() == 0) return "";
    auto arr = std::dynamic_pointer_cast<arrow::StringArray>(col->chunk(0));
    if (!arr || arr->length() == 0) return "";
    return arr->GetString(0);
}

// 读 list<int32>，row 默认 0
std::vector<int> listInt(const std::shared_ptr<arrow::Table>& t, const char* name, int64_t row = 0) {
    if (!t) throw std::runtime_error(std::string("[VQMMC-PV2] missing table for ") + name);
    auto col = t->GetColumnByName(name);
    if (!col || col->num_chunks() != 1) {
        throw std::runtime_error(std::string("[VQMMC-PV2] invalid list column: ") + name);
    }
    auto la = std::dynamic_pointer_cast<arrow::ListArray>(col->chunk(0));
    if (!la || row < 0 || row >= la->length() || la->IsNull(row)) {
        throw std::runtime_error(std::string("[VQMMC-PV2] invalid list row: ") + name);
    }
    auto values = std::dynamic_pointer_cast<arrow::Int32Array>(la->values());
    if (!values) throw std::runtime_error(std::string("[VQMMC-PV2] invalid list type: ") + name);
    int64_t off = la->value_offset(row);
    int64_t len = la->value_length(row);
    if (off < 0 || len < 0 || off > values->length() ||
        len > values->length() - off) {
        throw std::runtime_error(std::string("[VQMMC-PV2] list bounds invalid: ") + name);
    }
    std::vector<int> out(len);
    for (int64_t i = 0; i < len; ++i) out[i] = values->Value(off + i);
    return out;
}

// 读 list<int16>，row 默认 0
std::vector<int> listInt16(const std::shared_ptr<arrow::Table>& t, const char* name, int64_t row = 0) {
    if (!t) throw std::runtime_error(std::string("[VQMMC-PV2] missing table for ") + name);
    auto col = t->GetColumnByName(name);
    if (!col || col->num_chunks() != 1) {
        throw std::runtime_error(std::string("[VQMMC-PV2] invalid list column: ") + name);
    }
    auto la = std::dynamic_pointer_cast<arrow::ListArray>(col->chunk(0));
    if (!la || row < 0 || row >= la->length() || la->IsNull(row)) {
        throw std::runtime_error(std::string("[VQMMC-PV2] invalid list row: ") + name);
    }
    auto values = std::dynamic_pointer_cast<arrow::Int16Array>(la->values());
    if (!values) throw std::runtime_error(std::string("[VQMMC-PV2] invalid list type: ") + name);
    int64_t off = la->value_offset(row);
    int64_t len = la->value_length(row);
    if (off < 0 || len < 0 || off > values->length() ||
        len > values->length() - off) {
        throw std::runtime_error(std::string("[VQMMC-PV2] list bounds invalid: ") + name);
    }
    std::vector<int> out(len);
    for (int64_t i = 0; i < len; ++i) out[i] = static_cast<int>(values->Value(off + i));
    return out;
}

// 读 list<int8>，row 默认 0
std::vector<int> listInt8(const std::shared_ptr<arrow::Table>& t, const char* name, int64_t row = 0) {
    if (!t) throw std::runtime_error(std::string("[VQMMC-PV2] missing table for ") + name);
    auto col = t->GetColumnByName(name);
    if (!col || col->num_chunks() != 1) {
        throw std::runtime_error(std::string("[VQMMC-PV2] invalid list column: ") + name);
    }
    auto la = std::dynamic_pointer_cast<arrow::ListArray>(col->chunk(0));
    if (!la || row < 0 || row >= la->length() || la->IsNull(row)) {
        throw std::runtime_error(std::string("[VQMMC-PV2] invalid list row: ") + name);
    }
    auto values = std::dynamic_pointer_cast<arrow::Int8Array>(la->values());
    if (!values) throw std::runtime_error(std::string("[VQMMC-PV2] invalid list type: ") + name);
    int64_t off = la->value_offset(row);
    int64_t len = la->value_length(row);
    if (off < 0 || len < 0 || off > values->length() ||
        len > values->length() - off) {
        throw std::runtime_error(std::string("[VQMMC-PV2] list bounds invalid: ") + name);
    }
    std::vector<int> out(len);
    for (int64_t i = 0; i < len; ++i) out[i] = static_cast<int>(values->Value(off + i));
    return out;
}

// 读 binary scalar (row=0)
std::vector<uint8_t> binaryRow0(const std::shared_ptr<arrow::Table>& t, const char* name) {
    if (!t) throw std::runtime_error(std::string("[VQMMC-PV2] missing table for ") + name);
    auto col = t->GetColumnByName(name);
    if (!col || col->num_chunks() != 1) {
        throw std::runtime_error(std::string("[VQMMC-PV2] invalid binary column: ") + name);
    }
    auto arr = std::dynamic_pointer_cast<arrow::BinaryArray>(col->chunk(0));
    if (!arr || arr->length() != 1 || arr->IsNull(0)) {
        throw std::runtime_error(std::string("[VQMMC-PV2] invalid binary value: ") + name);
    }
    auto sv = arr->GetView(0);
    return std::vector<uint8_t>(sv.data(), sv.data() + sv.size());
}

struct VQPMSFPayloadRound {
    int cbSize = 0;
    int numIdx = 0;
    int cbBits = 0;
    int idxBits = 0;
    float cMin[3] = {0.0f, 0.0f, 0.0f};
    float cStep[3] = {1.0f, 1.0f, 1.0f};
    std::vector<int> cqx;
    std::vector<int> cqy;
    std::vector<int> cqz;
    std::vector<int> indices;
    bool hasCodebook = false;
    bool hasIndices = false;
};

struct VQPackedPayloadBlock {
    int origBytes = 0;
    int compBytes = 0;
    std::vector<std::uint8_t> bytes;
    bool valid = false;
};

struct VQMMCMetadataPayload {
    int numRounds = 0;
    bool enableCompress = false;
    bool enableQuantization = true;
    bool enableSegmentation = false;
    bool enableAdjustment = false;
    int numVertices = 0;
    int numFaces = 0;
    int quantBits = 0;
    float bboxMin[3] = {0.0f, 0.0f, 0.0f};
    float quantStep = 1.0f;
    int connBits = 0;
    int facetOrig = 0;
    int facetComp = 0;
    int edgeOrig = 0;
    int edgeComp = 0;
    bool valid = false;
};

struct VQMMCBaseMeshPayload {
    int seedV0[3] = {0, 0, 0};
    int seedV1[3] = {0, 0, 0};
    std::vector<int> vertexQX;
    std::vector<int> vertexQY;
    std::vector<int> vertexQZ;
    std::vector<int> faceVertices;
    std::vector<int> faceDegrees;
    bool valid = false;
};

void requirePayloadBytes(const std::vector<std::uint8_t>& bytes,
                         std::size_t off,
                         std::size_t count,
                         const char* what) {
    if (off > bytes.size() || count > bytes.size() - off) {
        throw std::runtime_error(std::string("[VQMMC-PV2] truncated ") + what);
    }
}

std::int32_t readI32LE(const std::vector<std::uint8_t>& bytes, std::size_t& off) {
    requirePayloadBytes(bytes, off, 4, "int32 payload");
    std::uint32_t u = static_cast<std::uint32_t>(bytes[off]) |
                      (static_cast<std::uint32_t>(bytes[off + 1]) << 8) |
                      (static_cast<std::uint32_t>(bytes[off + 2]) << 16) |
                      (static_cast<std::uint32_t>(bytes[off + 3]) << 24);
    off += 4;
    return static_cast<std::int32_t>(u);
}

std::uint16_t readU16LE(const std::vector<std::uint8_t>& bytes, std::size_t& off) {
    requirePayloadBytes(bytes, off, 2, "uint16 payload");
    std::uint16_t u = static_cast<std::uint16_t>(bytes[off]) |
                      (static_cast<std::uint16_t>(bytes[off + 1]) << 8);
    off += 2;
    return u;
}

float readF32LE(const std::vector<std::uint8_t>& bytes, std::size_t& off) {
    std::uint32_t u = static_cast<std::uint32_t>(readI32LE(bytes, off));
    float f = 0.0f;
    std::memcpy(&f, &u, sizeof(f));
    return f;
}

std::size_t checkedBytesForBits(std::size_t count, unsigned bits,
                                const char* what) {
    if (bits == 0 ||
        count > (std::numeric_limits<std::size_t>::max() - 7u) / bits) {
        throw std::runtime_error(std::string("[VQMMC-PV2] invalid bit size: ") + what);
    }
    return (count * bits + 7u) / 8u;
}

int normalizeStoredUInt16(int value, int bits, const char* what) {
    if (value < 0) {
        if (bits != 16) {
            throw std::runtime_error(std::string("[VQMMC-PV2] negative ") + what);
        }
        value += 65536;
    }
    if (bits < 1 || bits > 16 ||
        static_cast<std::uint32_t>(value) >= (1u << bits)) {
        throw std::runtime_error(std::string("[VQMMC-PV2] ") + what + " out of range");
    }
    return value;
}

const pmsf::PMSFSymbolPayloadRecord* findVQMMCPayload(
    const pmsf::PMSFSymbolPayloadTable* payloads,
    std::int32_t lodId,
    const std::string& payloadName) {
    if (!payloads) return nullptr;
    return pmsf::findPMSFPayload(*payloads, 0, 0, lodId, payloadName);
}

std::vector<VQPMSFPayloadRound> loadVQPMSFPayloadRounds(
    const pmsf::PMSFSymbolPayloadTable* payloads) {
    constexpr std::size_t kMaxRounds = 1u << 20;
    std::vector<VQPMSFPayloadRound> rounds;
    if (!payloads) return rounds;
    for (const auto& row : *payloads) {
        if (row.objectId != 0 || row.partitionId != 0 || row.lodId < 0) {
            continue;
        }
        if (row.payloadName != pmsf::kVQMMCPayloadCodebookName &&
            row.payloadName != pmsf::kVQMMCPayloadIndicesName) {
            continue;
        }
        if (row.lodId < 0 ||
            static_cast<std::size_t>(row.lodId) >= kMaxRounds) {
            throw std::runtime_error("[VQMMC-PV2] VQ payload round id is unreasonable");
        }
        if (static_cast<std::size_t>(row.lodId) >= rounds.size()) {
            rounds.resize(static_cast<std::size_t>(row.lodId) + 1);
        }
        auto& round = rounds[static_cast<std::size_t>(row.lodId)];
        std::size_t off = 0;
        if (row.payloadName == pmsf::kVQMMCPayloadCodebookName) {
            if (round.hasCodebook) {
                throw std::runtime_error("[VQMMC-PV2] duplicate codebook payload round");
            }
            if (row.payload.size() < 32) {
                throw std::runtime_error("[VQMMC-PV2] truncated codebook payload");
            }
            round.cbSize = readI32LE(row.payload, off);
            const int codebookCount = pmsf::vqmmcCodebookCount(round.cbSize);
            round.cbBits = readI32LE(row.payload, off);
            if (codebookCount > 0 && (round.cbBits < 2 || round.cbBits > 16)) {
                throw std::runtime_error("[VQMMC-PV2] invalid codebook quant bits");
            }
            if (row.payload.size() !=
                32 + static_cast<std::size_t>(codebookCount) * 6) {
                throw std::runtime_error(
                    "[VQMMC-PV2] inconsistent codebook payload size");
            }
            for (int axis = 0; axis < 3; ++axis) round.cMin[axis] = readF32LE(row.payload, off);
            for (int axis = 0; axis < 3; ++axis) round.cStep[axis] = readF32LE(row.payload, off);
            for (int axis = 0; axis < 3; ++axis) {
                if (!std::isfinite(round.cMin[axis]) ||
                    !std::isfinite(round.cStep[axis]) ||
                    (codebookCount > 0 && round.cStep[axis] <= 0.0f)) {
                    throw std::runtime_error("[VQMMC-PV2] invalid codebook scale");
                }
            }
            round.cqx.clear();
            round.cqy.clear();
            round.cqz.clear();
            round.cqx.reserve(codebookCount);
            round.cqy.reserve(codebookCount);
            round.cqz.reserve(codebookCount);
            for (int i = 0; i < codebookCount; ++i) {
                round.cqx.push_back(static_cast<int>(readU16LE(row.payload, off)));
                round.cqy.push_back(static_cast<int>(readU16LE(row.payload, off)));
                round.cqz.push_back(static_cast<int>(readU16LE(row.payload, off)));
            }
            round.hasCodebook = true;
        } else {
            if (round.hasIndices) {
                throw std::runtime_error("[VQMMC-PV2] duplicate index payload round");
            }
            if (row.payload.size() < 8) {
                throw std::runtime_error("[VQMMC-PV2] truncated index payload");
            }
            round.idxBits = readI32LE(row.payload, off);
            round.numIdx = readI32LE(row.payload, off);
            if (round.idxBits < 1 || round.idxBits > 16 || round.numIdx < 0) {
                throw std::runtime_error("[VQMMC-PV2] invalid index payload header");
            }
            const std::size_t expected =
                8u + static_cast<std::size_t>(round.numIdx) * 2u;
            if (expected != row.payload.size()) {
                throw std::runtime_error("[VQMMC-PV2] inconsistent index payload size");
            }
            round.indices.clear();
            round.indices.reserve(round.numIdx);
            for (int i = 0; i < round.numIdx; ++i) {
                round.indices.push_back(static_cast<int>(readU16LE(row.payload, off)));
            }
            round.hasIndices = true;
        }
    }
    while (!rounds.empty() && !rounds.back().hasCodebook && !rounds.back().hasIndices) {
        rounds.pop_back();
    }
    return rounds;
}

VQPackedPayloadBlock loadVQPackedPayloadBlock(
    const pmsf::PMSFSymbolPayloadTable* payloads,
    const std::string& payloadName) {
    VQPackedPayloadBlock out;
    const auto* row = findVQMMCPayload(payloads, 0, payloadName);
    if (!row) return out;
    if (row->payload.size() < 8) {
        throw std::runtime_error(
            std::string("[VQMMC-PV2] truncated packed symbol payload: ") +
            payloadName);
    }
    std::size_t off = 0;
    out.origBytes = readI32LE(row->payload, off);
    out.compBytes = readI32LE(row->payload, off);
    if (out.origBytes < 0 || out.compBytes == std::numeric_limits<int>::min()) {
        throw std::runtime_error("[VQMMC-PV2] invalid packed symbol sizes");
    }
    const std::size_t expected =
        out.compBytes < 0
            ? static_cast<std::size_t>(-out.compBytes)
            : out.compBytes > 0
                ? static_cast<std::size_t>(out.compBytes)
                : static_cast<std::size_t>(out.origBytes);
    if (row->payload.size() - off != expected) {
        throw std::runtime_error(
            std::string("[VQMMC-PV2] packed symbol size mismatch: ") + payloadName);
    }
    out.bytes.assign(row->payload.begin() + static_cast<std::ptrdiff_t>(off),
                     row->payload.end());
    out.valid = true;
    return out;
}

VQMMCMetadataPayload loadVQMMCMetadataPayload(
    const pmsf::PMSFSymbolPayloadTable* payloads) {
    VQMMCMetadataPayload out;
    const auto* row = findVQMMCPayload(payloads, 0, pmsf::kVQMMCPayloadMetadataName);
    if (!row) return out;
    if (row->payload.size() != 76) {
        throw std::runtime_error("[VQMMC-PV2] inconsistent metadata payload size");
    }
    std::size_t off = 0;
    const int version = readI32LE(row->payload, off);
    if (version != 1) return out;
    (void)readI32LE(row->payload, off);  // codec_version
    out.numRounds = readI32LE(row->payload, off);
    out.enableCompress = readI32LE(row->payload, off) != 0;
    out.enableQuantization = readI32LE(row->payload, off) != 0;
    out.enableSegmentation = readI32LE(row->payload, off) != 0;
    out.enableAdjustment = readI32LE(row->payload, off) != 0;
    out.numVertices = readI32LE(row->payload, off);
    out.numFaces = readI32LE(row->payload, off);
    out.quantBits = readI32LE(row->payload, off);
    out.bboxMin[0] = readF32LE(row->payload, off);
    out.bboxMin[1] = readF32LE(row->payload, off);
    out.bboxMin[2] = readF32LE(row->payload, off);
    out.quantStep = readF32LE(row->payload, off);
    out.connBits = readI32LE(row->payload, off);
    out.facetOrig = readI32LE(row->payload, off);
    out.facetComp = readI32LE(row->payload, off);
    out.edgeOrig = readI32LE(row->payload, off);
    out.edgeComp = readI32LE(row->payload, off);
    if (out.numRounds < 0 || out.numRounds > (1 << 20) ||
        out.numVertices < 2 || out.numFaces < 1 ||
        out.quantBits < 0 || out.quantBits > 16 ||
        out.connBits < 0 || out.connBits > 31 ||
        out.facetOrig < 0 || out.edgeOrig < 0 ||
        out.facetComp == std::numeric_limits<int>::min() ||
        out.edgeComp == std::numeric_limits<int>::min() ||
        !std::isfinite(out.quantStep) ||
        (out.enableQuantization && (out.quantBits < 2 || out.quantStep <= 0.0f))) {
        throw std::runtime_error("[VQMMC-PV2] invalid metadata values");
    }
    out.valid = true;
    return out;
}

VQMMCBaseMeshPayload loadVQMMCBaseMeshPayload(
    const pmsf::PMSFSymbolPayloadTable* payloads) {
    VQMMCBaseMeshPayload out;
    const auto* row = findVQMMCPayload(payloads, 0, pmsf::kVQMMCPayloadBaseMeshName);
    if (!row) return out;
    std::size_t off = 0;
    const int version = readI32LE(row->payload, off);
    if (version != 1) return out;
    for (int axis = 0; axis < 3; ++axis) out.seedV0[axis] = readI32LE(row->payload, off);
    for (int axis = 0; axis < 3; ++axis) out.seedV1[axis] = readI32LE(row->payload, off);
    const int vertexCount = readI32LE(row->payload, off);
    if (vertexCount < 0 ||
        static_cast<std::size_t>(vertexCount) >
            (row->payload.size() - off) / (3u * sizeof(std::int32_t))) {
        throw std::runtime_error("[VQMMC-PV2] invalid base vertex count");
    }
    out.vertexQX.reserve(static_cast<std::size_t>(vertexCount));
    out.vertexQY.reserve(static_cast<std::size_t>(vertexCount));
    out.vertexQZ.reserve(static_cast<std::size_t>(vertexCount));
    for (int i = 0; i < vertexCount; ++i) out.vertexQX.push_back(readI32LE(row->payload, off));
    for (int i = 0; i < vertexCount; ++i) out.vertexQY.push_back(readI32LE(row->payload, off));
    for (int i = 0; i < vertexCount; ++i) out.vertexQZ.push_back(readI32LE(row->payload, off));
    const int faceVertexCount = readI32LE(row->payload, off);
    if (faceVertexCount < 0 ||
        static_cast<std::size_t>(faceVertexCount) >
            (row->payload.size() - off) / sizeof(std::int32_t)) {
        throw std::runtime_error("[VQMMC-PV2] invalid base face vertex count");
    }
    out.faceVertices.reserve(static_cast<std::size_t>(faceVertexCount));
    for (int i = 0; i < faceVertexCount; ++i) {
        out.faceVertices.push_back(readI32LE(row->payload, off));
    }
    const int faceDegreeCount = readI32LE(row->payload, off);
    if (faceDegreeCount < 0 ||
        static_cast<std::size_t>(faceDegreeCount) >
            (row->payload.size() - off) / sizeof(std::int32_t)) {
        throw std::runtime_error("[VQMMC-PV2] invalid base face degree count");
    }
    out.faceDegrees.reserve(static_cast<std::size_t>(faceDegreeCount));
    for (int i = 0; i < faceDegreeCount; ++i) {
        out.faceDegrees.push_back(readI32LE(row->payload, off));
    }
    if (off != row->payload.size()) {
        throw std::runtime_error("[VQMMC-PV2] trailing bytes in base mesh payload");
    }
    out.valid = true;
    return out;
}

#endif  // COLUMNAR_SUPPORT
}  // namespace

VQMMCColumnarDeserializeOperator::~VQMMCColumnarDeserializeOperator() {
    // assembledBuffer_ 的所有权已在 init() 中通过 inner_->init(assembledBuffer_)
    // 交给 inner VQMMCDeserializeOperator。后者会在 deserializeBaseMesh 内部
    // `delete[] buffer_` 重建 buffer，之后 inner 自己负责最终释放。这里如果
    // 再 delete 会导致 double-free（见 VQMMCDeserializeOperator.cpp:71）。
    // 留空即可。
}

void VQMMCColumnarDeserializeOperator::setSeeds(std::vector<MCGAL::Halfedge*> seeds) {
    DeserializeOperator::setSeeds(seeds);
    if (inner_) inner_->setSeeds(seeds);
}

// region 谓词透传给内层算子。
void VQMMCColumnarDeserializeOperator::setQueryRegion(int regionId) {
    if (inner_) inner_->setQueryRegion(regionId);
}

// ============================================================================
// init()：4 表 → 重组 inner buffer
// ============================================================================

void VQMMCColumnarDeserializeOperator::init(std::string path) {
    clearPMSFSymbolContext();

#ifndef COLUMNAR_SUPPORT
    (void)path;
    std::cerr << "[VQMMC-PV2] Parquet support not compiled in" << std::endl;
    return;
#else
    const std::string baseName = stripParquetExt(path);

    std::shared_ptr<arrow::Table> metaTable, bmTable, symTable, vqTable;

    // 唯一规范形态优先：`<baseName>_compact.bin` 的通用段目录。
    {
        const std::string binPath = baseName + pmsf::kPaperCompactBinSuffix;
        pmsf::PMSFCompactBinEnvelope env;
        std::vector<std::string> names;
        std::vector<std::int32_t> codecIds;
        std::vector<std::uint32_t> offsets, lengths;
        if (fileExists(binPath) &&
            pmsf::readPMSFCompactBinEnvelopeFile(binPath, "[VQMMC-PV2]", env) &&
            pmsf::readCompactSegmentDirectory(env.payload, names, codecIds, offsets,
                                              lengths)) {
            for (std::size_t i = 0; i < names.size(); ++i) {
                std::vector<std::uint8_t> seg(env.payload.begin() + offsets[i],
                                              env.payload.begin() + offsets[i] + lengths[i]);
                auto table = readParquetTableFromBytes(seg, names[i]);
                if (names[i] == "metadata")       metaTable = table;
                else if (names[i] == "baseMesh")  bmTable   = table;
                else if (names[i] == "symbols")   symTable  = table;
                else if (names[i] == "vq")        vqTable   = table;
            }
            std::cout << "[VQMMC-PV2] read compact segment container: " << names.size()
                      << " segments" << std::endl;
        }
    }

    // 回落：降级互操作形态（各自的 parquet 表）。
    if (!metaTable) {
        const std::string metaPath = baseName + "_metadata.parquet";
        metaTable = fileExists(metaPath) ? readParquetTable(metaPath) : nullptr;
    }
    if (!bmTable) {
        const std::string bmPath = baseName + "_base_mesh.parquet";
        bmTable = fileExists(bmPath) ? readParquetTable(bmPath) : nullptr;
    }
    if (!symTable) {
        const std::string symPath = baseName + "_symbols.parquet";
        symTable = fileExists(symPath) ? readParquetTable(symPath) : nullptr;
    }
    if (!vqTable) {
        const std::string vqPath = baseName + "_vq.parquet";
        vqTable = fileExists(vqPath) ? readParquetTable(vqPath) : nullptr;
    }
    loadPMSFSymbolSidecarIfPresent(path);
    const auto* paperPayloads = paperSymbolPayloadTable();
    auto metaPayload = loadVQMMCMetadataPayload(paperPayloads);
    auto basePayload = loadVQMMCBaseMeshPayload(paperPayloads);
    auto payloadRounds = loadVQPMSFPayloadRounds(paperPayloads);
    auto facetPayload = loadVQPackedPayloadBlock(paperPayloads, "vqmmc_facet_symbols");
    auto edgePayload = loadVQPackedPayloadBlock(paperPayloads, "vqmmc_edge_symbols");
    if ((!metaTable && !metaPayload.valid) ||
        (!bmTable && !basePayload.valid) ||
        (!symTable && (!facetPayload.valid || !edgePayload.valid)) ||
        (!vqTable && payloadRounds.empty())) {
        std::cerr << "[VQMMC-PV2] missing parquet tables under base " << baseName << std::endl;
        return;
    }

    propagatePMSFSymbolContextTo(inner_);

    // ---- metadata ----
    int  numRounds   = metaTable ? scalar<int,  arrow::Int32Array>  (metaTable, "num_rounds",          0) : metaPayload.numRounds;
    bool eC          = metaTable ? scalar<bool, arrow::BooleanArray>(metaTable, "enable_compress",     false) : metaPayload.enableCompress;
    bool eQ          = metaTable ? scalar<bool, arrow::BooleanArray>(metaTable, "enable_quantization", true) : metaPayload.enableQuantization;
    bool eS          = metaTable ? scalar<bool, arrow::BooleanArray>(metaTable, "enable_segmentation", false) : metaPayload.enableSegmentation;
    bool eA          = metaTable ? scalar<bool, arrow::BooleanArray>(metaTable, "enable_adjustment",   false) : metaPayload.enableAdjustment;
    int  numV        = metaTable ? scalar<int,  arrow::Int32Array>  (metaTable, "num_vertices_base",   0) : metaPayload.numVertices;
    int  numF        = metaTable ? scalar<int,  arrow::Int32Array>  (metaTable, "num_faces_base",      0) : metaPayload.numFaces;
    int  qBits       = metaTable ? scalar<int,  arrow::Int32Array>  (metaTable, "quant_bits",          0) : metaPayload.quantBits;
    float bbX        = metaTable ? scalar<float,arrow::FloatArray>  (metaTable, "bbox_min_x",          0.f) : metaPayload.bboxMin[0];
    float bbY        = metaTable ? scalar<float,arrow::FloatArray>  (metaTable, "bbox_min_y",          0.f) : metaPayload.bboxMin[1];
    float bbZ        = metaTable ? scalar<float,arrow::FloatArray>  (metaTable, "bbox_min_z",          0.f) : metaPayload.bboxMin[2];
    float qStep      = metaTable ? scalar<float,arrow::FloatArray>  (metaTable, "quant_step",          1.f) : metaPayload.quantStep;
    int  connBits    = metaTable ? scalar<int,  arrow::Int32Array>  (metaTable, "conn_bits",           0) : metaPayload.connBits;
    int  facetOrig   = metaTable ? scalar<int,  arrow::Int32Array>  (metaTable, "facet_orig_size",     0) : metaPayload.facetOrig;
    int  facetComp   = metaTable ? scalar<int,  arrow::Int32Array>  (metaTable, "facet_comp_size",     0) : metaPayload.facetComp;
    int  edgeOrig    = metaTable ? scalar<int,  arrow::Int32Array>  (metaTable, "edge_orig_size",      0) : metaPayload.edgeOrig;
    int  edgeComp    = metaTable ? scalar<int,  arrow::Int32Array>  (metaTable, "edge_comp_size",      0) : metaPayload.edgeComp;
    if (!symTable) {
        facetOrig = facetPayload.origBytes;
        facetComp = facetPayload.compBytes;
        edgeOrig = edgePayload.origBytes;
        edgeComp = edgePayload.compBytes;
    }
    // Unified payload lod ids (including historical files) use collector
    // order. Compatibility VQ tables and Binary use reverse, decoder order.
    // Empty coarse rounds have no payload rows, but still have a Binary header.
    if (!vqTable) {
        // Each round has at least its facet-stream alignment byte. Validate
        // before resize; do not truncate extra payload rounds or trust a
        // corrupt, unbounded metadata count.
        if (numRounds < 0 || numRounds > facetOrig ||
            payloadRounds.size() > static_cast<std::size_t>(numRounds)) {
            throw std::runtime_error("[VQMMC-PV2] inconsistent payload round count");
        }
        payloadRounds.resize(static_cast<std::size_t>(numRounds));
        std::reverse(payloadRounds.begin(), payloadRounds.end());
    }

    // 还原 PMSFHeader（VQMMC main 在 decompress 阶段会调 deserializeHeader 拿到）
    header_ = PMSFHeader::Builder()
                  .setRound(numRounds)
                  .setEnableCompress(eC)
                  .setEnableQuantization(eQ)
                  .setEnableSegmentation(eS)
                  .setEnableAdjustMent(eA)
                  .build();

    // ---- base_mesh ----
    int seedV0[3] = {
        bmTable ? scalar<int, arrow::Int32Array>(bmTable, "seed_v0_qx", 0) : basePayload.seedV0[0],
        bmTable ? scalar<int, arrow::Int32Array>(bmTable, "seed_v0_qy", 0) : basePayload.seedV0[1],
        bmTable ? scalar<int, arrow::Int32Array>(bmTable, "seed_v0_qz", 0) : basePayload.seedV0[2],
    };
    int seedV1[3] = {
        bmTable ? scalar<int, arrow::Int32Array>(bmTable, "seed_v1_qx", 0) : basePayload.seedV1[0],
        bmTable ? scalar<int, arrow::Int32Array>(bmTable, "seed_v1_qy", 0) : basePayload.seedV1[1],
        bmTable ? scalar<int, arrow::Int32Array>(bmTable, "seed_v1_qz", 0) : basePayload.seedV1[2],
    };
    auto vqx = bmTable ? listInt16(bmTable, "vertex_qx") : basePayload.vertexQX;
    auto vqy = bmTable ? listInt16(bmTable, "vertex_qy") : basePayload.vertexQY;
    auto vqz = bmTable ? listInt16(bmTable, "vertex_qz") : basePayload.vertexQZ;
    auto fv  = bmTable ? listInt(bmTable, "face_vertices") : basePayload.faceVertices;
    auto fd  = bmTable ? listInt8(bmTable, "face_degrees") : basePayload.faceDegrees;
    if (numRounds < 0 || numRounds > (1 << 20) ||
        numV < 2 || numF < 1 ||
        (eQ && (qBits < 2 || qBits > 16 ||
                !std::isfinite(qStep) || qStep <= 0.0f)) ||
        (!eQ && (qBits < 0 || qBits > 16)) ||
        connBits < 0 || connBits > 31 ||
        static_cast<std::size_t>(numV - 2) != vqx.size() ||
        vqx.size() != vqy.size() || vqx.size() != vqz.size() ||
        fd.size() != static_cast<std::size_t>(numF) ||
        static_cast<std::size_t>(numF) >
            std::numeric_limits<std::size_t>::max() / 3u ||
        fv.size() != static_cast<std::size_t>(numF) * 3u) {
        throw std::runtime_error("[VQMMC-PV2] inconsistent base mesh dimensions");
    }
    if (eQ) {
        for (int* axis : {&seedV0[0], &seedV0[1], &seedV0[2],
                          &seedV1[0], &seedV1[1], &seedV1[2]}) {
            if (*axis < 0 ||
                static_cast<std::uint32_t>(*axis) >= (1u << qBits)) {
                throw std::runtime_error("[VQMMC-PV2] seed coordinate out of range");
            }
        }
        for (int& value : vqx) value = normalizeStoredUInt16(value, qBits, "base coordinate");
        for (int& value : vqy) value = normalizeStoredUInt16(value, qBits, "base coordinate");
        for (int& value : vqz) value = normalizeStoredUInt16(value, qBits, "base coordinate");
    }
    for (int degree : fd) {
        if (degree != 3) {
            throw std::runtime_error("[VQMMC-PV2] only triangular base meshes are supported");
        }
    }
    for (int vertex : fv) {
        if (vertex < 0 || vertex >= numV) {
            throw std::runtime_error("[VQMMC-PV2] base face index out of range");
        }
    }
    if (eQ && (connBits < 1 ||
               static_cast<std::uint32_t>(connBits) < 32 &&
               static_cast<std::uint64_t>(numV - 1) >=
                   (std::uint64_t{1} << connBits))) {
        throw std::runtime_error("[VQMMC-PV2] invalid base connectivity bit width");
    }

    // ---- symbols（facet/edge BINARY） ----
    auto facetBin = symTable ? binaryRow0(symTable, "facet_symbols")
                             : facetPayload.bytes;
    auto edgeBin  = symTable ? binaryRow0(symTable, "edge_symbols")
                             : edgePayload.bytes;
    if (facetOrig < 0 || edgeOrig < 0 ||
        facetComp == std::numeric_limits<int>::min() ||
        edgeComp == std::numeric_limits<int>::min()) {
        throw std::runtime_error("[VQMMC-PV2] invalid symbol block sizes");
    }
    const auto packedSize = [](int orig, int comp) -> std::size_t {
        return comp < 0
            ? static_cast<std::size_t>(-comp)
            : comp > 0
                ? static_cast<std::size_t>(comp)
                : static_cast<std::size_t>(orig);
    };
    const std::size_t facetEmit = packedSize(facetOrig, facetComp);
    const std::size_t edgeEmit = packedSize(edgeOrig, edgeComp);
    if (facetBin.size() != facetEmit || edgeBin.size() != edgeEmit) {
        throw std::runtime_error("[VQMMC-PV2] symbol payload size mismatch");
    }

    // ---- vq（每轮一行） ----
    if (vqTable && vqTable->num_rows() > (1 << 20)) {
        throw std::runtime_error("[VQMMC-PV2] VQ table has too many rows");
    }
    int  numVqRows       = vqTable
                               ? static_cast<int>(vqTable->num_rows())
                               : static_cast<int>(payloadRounds.size());
    auto getInt32Col = [&](const char* name) -> std::vector<int> {
        if (!vqTable) return {};
        auto col = vqTable->GetColumnByName(name);
        if (!col || col->num_chunks() != 1) {
            throw std::runtime_error(std::string("[VQMMC-PV2] invalid VQ column: ") + name);
        }
        auto arr = std::dynamic_pointer_cast<arrow::Int32Array>(col->chunk(0));
        if (!arr) {
            throw std::runtime_error(std::string("[VQMMC-PV2] invalid VQ column type: ") + name);
        }
        std::vector<int> r(arr->length());
        for (int64_t i = 0; i < arr->length(); ++i) {
            if (arr->IsNull(i)) {
                throw std::runtime_error(std::string("[VQMMC-PV2] null VQ value: ") + name);
            }
            r[i] = arr->Value(i);
        }
        return r;
    };
    auto getFloatCol = [&](const char* name) -> std::vector<float> {
        if (!vqTable) return {};
        auto col = vqTable->GetColumnByName(name);
        if (!col || col->num_chunks() != 1) {
            throw std::runtime_error(std::string("[VQMMC-PV2] invalid VQ column: ") + name);
        }
        auto arr = std::dynamic_pointer_cast<arrow::FloatArray>(col->chunk(0));
        if (!arr) {
            throw std::runtime_error(std::string("[VQMMC-PV2] invalid VQ column type: ") + name);
        }
        std::vector<float> r(arr->length());
        for (int64_t i = 0; i < arr->length(); ++i) {
            if (arr->IsNull(i)) {
                throw std::runtime_error(std::string("[VQMMC-PV2] null VQ value: ") + name);
            }
            r[i] = arr->Value(i);
        }
        return r;
    };
    auto roundIds = getInt32Col("round_id");
    auto cbSizes = getInt32Col("codebook_size");
    auto numIdx  = getInt32Col("num_indices");
    auto cbBits  = getInt32Col("cb_quant_bits");
    auto cmx = getFloatCol("cb_min_x"), cmy = getFloatCol("cb_min_y"), cmz = getFloatCol("cb_min_z");
    auto csx = getFloatCol("cb_step_x"), csy = getFloatCol("cb_step_y"), csz = getFloatCol("cb_step_z");
    auto idxBitsCol = getInt32Col("idx_bits");
    if (!vqTable) {
        roundIds.resize(payloadRounds.size());
        cbSizes.resize(payloadRounds.size());
        numIdx.resize(payloadRounds.size());
        cbBits.resize(payloadRounds.size());
        cmx.resize(payloadRounds.size());
        cmy.resize(payloadRounds.size());
        cmz.resize(payloadRounds.size());
        csx.resize(payloadRounds.size());
        csy.resize(payloadRounds.size());
        csz.resize(payloadRounds.size());
        idxBitsCol.resize(payloadRounds.size());
        for (std::size_t r = 0; r < payloadRounds.size(); ++r) {
            roundIds[r] = static_cast<int>(r);
            cbSizes[r] = payloadRounds[r].cbSize;
            numIdx[r] = payloadRounds[r].numIdx;
            cbBits[r] = payloadRounds[r].cbBits;
            cmx[r] = payloadRounds[r].cMin[0];
            cmy[r] = payloadRounds[r].cMin[1];
            cmz[r] = payloadRounds[r].cMin[2];
            csx[r] = payloadRounds[r].cStep[0];
            csy[r] = payloadRounds[r].cStep[1];
            csz[r] = payloadRounds[r].cStep[2];
            idxBitsCol[r] = payloadRounds[r].idxBits;
        }
    }
    if (numVqRows != numRounds) {
        throw std::runtime_error("[VQMMC-PV2] VQ row count does not match metadata");
    }
    const auto requireColumnRows = [&](const std::vector<int>& values,
                                       const char* name) {
        if (values.size() != static_cast<std::size_t>(numVqRows)) {
            throw std::runtime_error(std::string("[VQMMC-PV2] VQ column length mismatch: ") + name);
        }
    };
    const auto requireFloatRows = [&](const std::vector<float>& values,
                                      const char* name) {
        if (values.size() != static_cast<std::size_t>(numVqRows)) {
            throw std::runtime_error(std::string("[VQMMC-PV2] VQ column length mismatch: ") + name);
        }
    };
    requireColumnRows(roundIds, "round_id");
    requireColumnRows(cbSizes, "codebook_size");
    requireColumnRows(numIdx, "num_indices");
    requireColumnRows(cbBits, "cb_quant_bits");
    requireColumnRows(idxBitsCol, "idx_bits");
    requireFloatRows(cmx, "cb_min_x"); requireFloatRows(cmy, "cb_min_y");
    requireFloatRows(cmz, "cb_min_z"); requireFloatRows(csx, "cb_step_x");
    requireFloatRows(csy, "cb_step_y"); requireFloatRows(csz, "cb_step_z");

    std::size_t vqRequired = 0;
    const auto addSize = [&](std::size_t& total, std::size_t value,
                             const char* what) {
        if (value > std::numeric_limits<std::size_t>::max() - total) {
            throw std::runtime_error(std::string("[VQMMC-PV2] size overflow: ") + what);
        }
        total += value;
    };
    for (int r = 0; r < numVqRows; ++r) {
        if (roundIds[static_cast<std::size_t>(r)] != r) {
            throw std::runtime_error("[VQMMC-PV2] VQ round ids are not contiguous");
        }
        const int codebookCount = pmsf::vqmmcCodebookCount(cbSizes[r]);
        if (numIdx[r] < 0 || (codebookCount == 0 && numIdx[r] != 0)) {
            throw std::runtime_error("[VQMMC-PV2] invalid VQ round counts");
        }
        addSize(vqRequired, 8, "VQ round header");
        if (codebookCount == 0) {
            continue;
        }
        if (cbBits[r] < 2 || cbBits[r] > 16 ||
            idxBitsCol[r] < 1 || idxBitsCol[r] > 16 ||
            !std::isfinite(cmx[r]) || !std::isfinite(cmy[r]) ||
            !std::isfinite(cmz[r]) || !std::isfinite(csx[r]) ||
            !std::isfinite(csy[r]) || !std::isfinite(csz[r]) ||
            csx[r] <= 0.0f || csy[r] <= 0.0f || csz[r] <= 0.0f) {
            throw std::runtime_error("[VQMMC-PV2] invalid VQ round metadata");
        }
        int minimumIdxBits = 1;
        while ((1 << minimumIdxBits) < codebookCount) ++minimumIdxBits;
        if (idxBitsCol[r] < minimumIdxBits) {
            throw std::runtime_error("[VQMMC-PV2] insufficient VQ index bits");
        }
        const auto cqx = vqTable ? listInt16(vqTable, "cb_quantized_x", r)
                                 : payloadRounds[static_cast<std::size_t>(r)].cqx;
        const auto cqy = vqTable ? listInt16(vqTable, "cb_quantized_y", r)
                                 : payloadRounds[static_cast<std::size_t>(r)].cqy;
        const auto cqz = vqTable ? listInt16(vqTable, "cb_quantized_z", r)
                                 : payloadRounds[static_cast<std::size_t>(r)].cqz;
        const auto indices = vqTable ? listInt16(vqTable, "indices", r)
                                     : payloadRounds[static_cast<std::size_t>(r)].indices;
        if (cqx.size() != static_cast<std::size_t>(codebookCount) ||
            cqy.size() != static_cast<std::size_t>(codebookCount) ||
            cqz.size() != static_cast<std::size_t>(codebookCount) ||
            indices.size() != static_cast<std::size_t>(numIdx[r])) {
            throw std::runtime_error("[VQMMC-PV2] VQ list length mismatch");
        }
        for (int value : cqx) (void)normalizeStoredUInt16(value, cbBits[r], "codebook value");
        for (int value : cqy) (void)normalizeStoredUInt16(value, cbBits[r], "codebook value");
        for (int value : cqz) (void)normalizeStoredUInt16(value, cbBits[r], "codebook value");
        for (int value : indices) (void)normalizeStoredUInt16(value, idxBitsCol[r], "index value");
        addSize(vqRequired, 1 + 24, "VQ codebook header");
        addSize(vqRequired, checkedBytesForBits(
            static_cast<std::size_t>(codebookCount) * 3u,
            static_cast<unsigned>(cbBits[r]), "VQ codebook"), "VQ codebook");
        addSize(vqRequired, 1, "VQ index header");
        addSize(vqRequired, checkedBytesForBits(
            static_cast<std::size_t>(numIdx[r]),
            static_cast<unsigned>(idxBitsCol[r]), "VQ indices"), "VQ indices");
    }

    // ============================================================
    // 还原 inner buffer：
    //   1) base mesh 段（与 BasicSerializeOperator 写入端字节级一致）
    //   2) facet zlib block: [int origSize][int compSize][bytes]
    //   3) edge  zlib block: [int origSize][int compSize][bytes]
    //   4) vq    block: 与 VQMMCSymbolCollectOperator::exportToBuffer 完全一致
    //                   包装为 [int origSize][int compSize=0][bytes]
    // ============================================================

    const std::size_t headerBytes = 4u + 2u + (eS ? 4u : 0u);
    const auto legacyBitStreamBytes = [&](std::size_t count,
                                          unsigned bits,
                                          const char* what) {
        if (bits == 0 ||
            count > std::numeric_limits<std::size_t>::max() / bits) {
            throw std::runtime_error(std::string("[VQMMC-PV2] size overflow: ") + what);
        }
        // BasicSerializeOperator advances by floor(bits/8) and then always
        // consumes the current alignment byte.
        return (count * bits) / 8u + 1u;
    };
    std::size_t baseBytes = 0;
    if (eQ) {
        addSize(baseBytes, 1u + 12u + 4u + 4u + 4u, "quantized base header");
        addSize(baseBytes, legacyBitStreamBytes(
            static_cast<std::size_t>(numV) * 3u,
            static_cast<unsigned>(qBits), "base coordinates"), "base coordinates");
        addSize(baseBytes, 1u, "base connectivity bit width");
        addSize(baseBytes, legacyBitStreamBytes(
            fv.size(), static_cast<unsigned>(connBits), "base connectivity"),
            "base connectivity");
    } else {
        addSize(baseBytes, 8u, "non-quantized base header");
        addSize(baseBytes, static_cast<std::size_t>(numV) * 12u,
                "non-quantized base coordinates");
        addSize(baseBytes, fv.size() * sizeof(std::int32_t),
                "non-quantized base connectivity");
    }
    std::size_t exactSize = headerBytes;
    addSize(exactSize, baseBytes, "assembled base mesh");
    addSize(exactSize, 8u + facetEmit, "facet block");
    addSize(exactSize, 8u + edgeEmit, "edge block");
    addSize(exactSize, 8u + vqRequired, "VQ block");
    if (exactSize > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        throw std::runtime_error("[VQMMC-PV2] assembled buffer exceeds int protocol limit");
    }
    const std::size_t bufSize = exactSize + 16u;
    delete[] assembledBuffer_;
    assembledBuffer_ = new char[bufSize];
    std::memset(assembledBuffer_, 0, bufSize);
    std::size_t off = 0;

    // ----- 0) PMSFHeader 段（与 PMSFHeader::serialize 字节布局一致）。
    //   int round (4 bytes), 9 bools 位包到 1~2 byte（5..9 位写满 1 byte，
    //   第 9 位落到下一个字节的 bit0），随后 offset += 1 强制对齐。
    //   实际占用 = 4 + 1 + 1 = 6 byte（与 PMSFHeader::deserialize 逐字节对齐）。
    //   若开启 segmentation 还会再补 4 byte int(groupNumber)，VQMMC 不开启。
    {
        size_t hOff = off;
        ::writeInt(assembledBuffer_, hOff, header_.getRound());
        unsigned bitOff = 0;
        ::writeBool(assembledBuffer_, bitOff, hOff, header_.isEnableCompress());
        ::writeBool(assembledBuffer_, bitOff, hOff, header_.isEnableQuantization());
        ::writeBool(assembledBuffer_, bitOff, hOff, header_.isEnableSegmentation());
        ::writeBool(assembledBuffer_, bitOff, hOff, header_.isEnableAdjustment());
        ::writeBool(assembledBuffer_, bitOff, hOff, header_.isEnablePrediction());
        // bit 5: triangulationDisabled = !enableTriangulation（PMSFHeader::serialize 反向位）
        ::writeBool(assembledBuffer_, bitOff, hOff, !header_.isEnableTriangulation());
        // bits 6-8: 3 个 PPMC 风格预测开关。VQMMC 一般为 false。
        ::writeBool(assembledBuffer_, bitOff, hOff, header_.isEnableConnectivityPredictionFaces());
        ::writeBool(assembledBuffer_, bitOff, hOff, header_.isEnableConnectivityPredictionEdges());
        ::writeBool(assembledBuffer_, bitOff, hOff, header_.isEnableTriangleMeshBalance());
        hOff += 1;
        if (header_.isEnableSegmentation()) {
            ::writeInt(assembledBuffer_, hOff, header_.getGroupNumber());
        }
        off = hOff;
    }

    // ----- 1) base mesh 段 -----
    auto writeIntLE = [&](int v) {
        std::memcpy(assembledBuffer_ + off, &v, sizeof(int));
        off += sizeof(int);
    };
    auto writeCharByte = [&](unsigned char v) {
        assembledBuffer_[off++] = static_cast<char>(v);
    };
    auto writeFloatLE = [&](float v) {
        std::memcpy(assembledBuffer_ + off, &v, sizeof(float));
        off += sizeof(float);
    };
    // 直接使用 BufferUtils.h 的 writeBitsOff（big-endian bit packing，写整个 uint32 with OR
    // — 我们已 memset assembledBuffer_=0，确保 OR 安全）。

    // BasicSerializeOperator::serializeBaseMeshWithSeed 在 quant 模式下顺序：
    //   char(quantBits), Point(bboxMin), float(quantStep),
    //   int(numV), int(numF),
    //   bit-stream (seed v0, seed v1, then other vertices) using quantBits,
    //   align byte (off++),
    //   char(connBit),
    //   bit-stream face vertex ids using connBit,
    //   align byte (off++).
    if (eQ) {
        writeCharByte(static_cast<unsigned char>(qBits));
        writeFloatLE(bbX); writeFloatLE(bbY); writeFloatLE(bbZ);
        writeFloatLE(qStep);
    }
    writeIntLE(numV);
    writeIntLE(numF);

    if (eQ) {
        unsigned bitOff = 0;
        // seed v0
        for (int k = 0; k < 3; ++k)
            ::writeBitsOff(static_cast<uint32_t>(seedV0[k]), qBits, assembledBuffer_, bitOff, off);
        // seed v1
        for (int k = 0; k < 3; ++k)
            ::writeBitsOff(static_cast<uint32_t>(seedV1[k]), qBits, assembledBuffer_, bitOff, off);
        // 其余顶点
        for (size_t i = 0; i < vqx.size(); ++i) {
            ::writeBitsOff(static_cast<uint32_t>(vqx[i]), qBits, assembledBuffer_, bitOff, off);
            ::writeBitsOff(static_cast<uint32_t>(vqy[i]), qBits, assembledBuffer_, bitOff, off);
            ::writeBitsOff(static_cast<uint32_t>(vqz[i]), qBits, assembledBuffer_, bitOff, off);
        }
        // BasicSerializeOperator 中的 align：bitOffset = 0; dataOffset_++
        bitOff = 0;
        off++;
        // connBit
        writeCharByte(static_cast<unsigned char>(connBits));
        // 面顶点（仅三角网格）
        bitOff = 0;
        size_t fvIdx = 0;
        for (int f = 0; f < numF; ++f) {
            for (int k = 0; k < 3 && fvIdx < fv.size(); ++k) {
                ::writeBitsOff(static_cast<uint32_t>(fv[fvIdx++]), connBits,
                               assembledBuffer_, bitOff, off);
            }
        }
        // align
        bitOff = 0;
        off++;
    } else {
        // 非 quant 路径：把 vqx/y/z 当作 int*1e6 的浮点回填
        // BasicSerializer 写：writePoint(seed v0), writePoint(seed v1), 其余 writePoint(vit)
        auto writePointFloat = [&](float x, float y, float z) {
            writeFloatLE(x); writeFloatLE(y); writeFloatLE(z);
        };
        writePointFloat(seedV0[0] * 1e-6f, seedV0[1] * 1e-6f, seedV0[2] * 1e-6f);
        writePointFloat(seedV1[0] * 1e-6f, seedV1[1] * 1e-6f, seedV1[2] * 1e-6f);
        for (size_t i = 0; i < vqx.size(); ++i)
            writePointFloat(vqx[i] * 1e-6f, vqy[i] * 1e-6f, vqz[i] * 1e-6f);
        for (size_t i = 0; i < fv.size(); ++i) writeIntLE(fv[i]);
    }

    // ----- 2) facet zlib block 头 + payload -----
    writeIntLE(facetOrig);
    writeIntLE(facetComp);
    if (facetOrig > 0) {
        // P16：负 facetComp = RC 编码，长度取绝对值
        size_t emit = (facetComp < 0) ? static_cast<size_t>(-facetComp)
                     : (facetComp > 0 ? static_cast<size_t>(facetComp)
                                      : static_cast<size_t>(facetOrig));
        std::memcpy(assembledBuffer_ + off, facetBin.data(), emit);
        off += emit;
    }

    // ----- 3) edge zlib block -----
    writeIntLE(edgeOrig);
    writeIntLE(edgeComp);
    if (edgeOrig > 0) {
        size_t emit = edgeComp > 0 ? static_cast<size_t>(edgeComp)
                                   : static_cast<size_t>(edgeOrig);
        std::memcpy(assembledBuffer_ + off, edgeBin.data(), emit);
        off += emit;
    }

    // ----- 4) vq block：与 VQMMCSymbolCollectOperator::exportToBuffer 一致 -----
    // 用足够大的 vqBuf + size_t off，与 exportToBuffer 同样调用 BufferUtils.h 的
    // writeInt / writeChar / writeFloat / writeBitsOff，保证字节级一致。
    if (vqRequired > std::numeric_limits<std::size_t>::max() - 4u) {
        throw std::runtime_error("[VQMMC-PV2] VQ buffer size overflow");
    }
    const std::size_t vqBufSize = std::max<std::size_t>(1u, vqRequired + 4u);
    std::vector<char> vqBufVec(vqBufSize, 0);
    char* vqBuf = vqBufVec.data();
    size_t vqOff = 0;
    const auto ensureVqBytes = [&](std::size_t count) {
        if (vqOff > vqBufSize || count > vqBufSize - vqOff) {
            throw std::runtime_error("[VQMMC-PV2] VQ payload capacity exceeded");
        }
    };
    const auto writeVqInt = [&](int value) {
        ensureVqBytes(sizeof(int));
        ::writeInt(vqBuf, vqOff, value);
    };
    const auto writeVqChar = [&](unsigned char value) {
        ensureVqBytes(sizeof(unsigned char));
        ::writeChar(vqBuf, vqOff, value);
    };
    const auto writeVqFloat = [&](float value) {
        ensureVqBytes(sizeof(float));
        ::writeFloat(vqBuf, vqOff, value);
    };
    const auto writeVqBits = [&](std::uint32_t value, unsigned bits,
                                 unsigned& bitOffset) {
        (void)bits;
        // BufferUtils::writeBitsOff performs a 32-bit OR even when fewer than
        // four logical bytes remain in the stream; reserve that implementation
        // footprint while keeping vqRequired equal to the wire length.
        ensureVqBytes(sizeof(std::uint32_t));
        ::writeBitsOff(value, bits, vqBuf, bitOffset, vqOff);
    };

    for (int r = 0; r < numVqRows; ++r) {
        const int codebookCount = pmsf::vqmmcCodebookCount(cbSizes[r]);
        writeVqInt(cbSizes[r]);
        writeVqInt(numIdx[r]);
        if (cbSizes[r] == 0) continue;

        writeVqChar(static_cast<unsigned char>(cbBits[r]));
        writeVqFloat(cmx[r]); writeVqFloat(cmy[r]); writeVqFloat(cmz[r]);
        writeVqFloat(csx[r]); writeVqFloat(csy[r]); writeVqFloat(csz[r]);

        // codebook 量化值
        auto cqx = vqTable ? listInt16(vqTable, "cb_quantized_x", r)
                           : payloadRounds[static_cast<std::size_t>(r)].cqx;
        auto cqy = vqTable ? listInt16(vqTable, "cb_quantized_y", r)
                           : payloadRounds[static_cast<std::size_t>(r)].cqy;
        auto cqz = vqTable ? listInt16(vqTable, "cb_quantized_z", r)
                           : payloadRounds[static_cast<std::size_t>(r)].cqz;
        if (cqx.size() != static_cast<std::size_t>(codebookCount) ||
            cqy.size() != static_cast<std::size_t>(codebookCount) ||
            cqz.size() != static_cast<std::size_t>(codebookCount)) {
            throw std::runtime_error("[VQMMC-PV2] inconsistent codebook columns");
        }
        unsigned cb_bitOff = 0;
        for (int j = 0; j < codebookCount; ++j) {
            writeVqBits(static_cast<std::uint32_t>(
                            normalizeStoredUInt16(cqx[j], cbBits[r], "codebook value")),
                        static_cast<unsigned>(cbBits[r]), cb_bitOff);
            writeVqBits(static_cast<std::uint32_t>(
                            normalizeStoredUInt16(cqy[j], cbBits[r], "codebook value")),
                        static_cast<unsigned>(cbBits[r]), cb_bitOff);
            writeVqBits(static_cast<std::uint32_t>(
                            normalizeStoredUInt16(cqz[j], cbBits[r], "codebook value")),
                        static_cast<unsigned>(cbBits[r]), cb_bitOff);
        }
        if (cb_bitOff != 0) {
            ensureVqBytes(1);
            ++vqOff;
        }

        // idx_bits + indices
        writeVqChar(static_cast<unsigned char>(idxBitsCol[r]));
        unsigned idx_bitOff = 0;
        auto idxList = vqTable ? listInt16(vqTable, "indices", r)
                               : payloadRounds[static_cast<std::size_t>(r)].indices;
        for (int v : idxList) {
            writeVqBits(static_cast<std::uint32_t>(
                            normalizeStoredUInt16(v, idxBitsCol[r], "index value")),
                        static_cast<unsigned>(idxBitsCol[r]), idx_bitOff);
        }
        if (idx_bitOff != 0) {
            ensureVqBytes(1);
            ++vqOff;
        }
    }
    if (vqOff != vqRequired) {
        throw std::runtime_error("[VQMMC-PV2] VQ payload size calculation mismatch");
    }

    int vqOrig = static_cast<int>(vqOff);
    writeIntLE(vqOrig);
    writeIntLE(0);  // compSize = 0 → 透传，readBlock 直接 memcpy
    if (vqOrig > 0) {
        if (off > bufSize ||
            static_cast<std::size_t>(vqOrig) > bufSize - off) {
            throw std::runtime_error(
                "[VQMMC-PV2] assembled buffer capacity exceeded");
        }
        std::memcpy(assembledBuffer_ + off, vqBuf, vqOrig);
        off += vqOrig;
    }

    assembledSize_ = off;

    std::cout << "[VQMMC-PV2] reassembled inner buffer: " << assembledSize_
              << " bytes (rounds=" << numRounds
              << " base_v=" << numV
              << " base_f=" << numF
              << " facet_orig=" << facetOrig
              << " edge_orig=" << edgeOrig
              << " vq_raw=" << vqOrig << ")" << std::endl;

    // 注入 inner_
    inner_->init(assembledBuffer_);
    inner_->dataOffset_ = 0;
    inner_->totalSize = assembledSize_;

    // 自己把 inner_ 定位到 base mesh 起点。
    //
    // 通用解压主循环 (runDecompress) 的规则是「init() 没填 header_ 才调
    // deserializeHeader」；上面已还原出非零 round，于是主循环会跳过那一步，
    // 而 inner_->dataOffset_ 此时仍停在 header 段之前 —— deserializeBaseMesh
    // 会从一个错位的 offset 读 base mesh（实测越界到 9 MB 处直接段错误）。
    // 凡是在 init() 里自己还原 header 的解压器，都有义务同时把指针摆好。
    {
        // 直接复用下面那个 deserializeHeader：它先归零再让 inner_ 读一遍 header，
        // 读完后 inner_->dataOffset_ 正好落在 base mesh 起点。幂等，所以调用方
        // 之后再调一次也不会错位。
        PMSFHeader probe;
        deserializeHeader(probe);
    }
#endif  // COLUMNAR_SUPPORT
}

void VQMMCColumnarDeserializeOperator::deserializeHeader(PMSFHeader& header) {
    // 让 inner_ 真正从 assembled buffer 读 header（这样 inner_->dataOffset_ 被推进
    // 到 base mesh 起点，与下游 deserializeBaseMesh 期望对齐）。我们已经把
    // header 段按原编码写在 buffer 开头，这里走原 deserialize 路径即可。
    if (inner_) {
        inner_->dataOffset_ = 0;
        inner_->deserializeHeader(header);
        header_ = header;
    } else {
        header = header_;
    }
}

std::shared_ptr<MCGAL::Mesh> VQMMCColumnarDeserializeOperator::deserializeBaseMesh() {
    auto m = inner_->deserializeBaseMesh();
    mesh_ = m;
    return m;
}

void VQMMCColumnarDeserializeOperator::deserializeOneRound() {
    inner_->deserializeOneRound();
}

std::shared_ptr<MCGAL::Mesh> VQMMCColumnarDeserializeOperator::exportMesh() {
    return inner_->exportMesh();
}
