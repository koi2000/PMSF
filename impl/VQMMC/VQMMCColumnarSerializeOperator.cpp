/*
 * VQMMCColumnarSerializeOperator 实现 — 见同名 header 的注释。
 */

#include "VQMMCColumnarSerializeOperator.h"

#include "common/AlphabetStreamCodec.h"
#include "BufferUtils.h"
#include "EntropyCodec.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Vertex.h"
#include "Point.h"
#include "storage/PMSFSymbolStorageAdapter.h"
#include "storage/CompactSegmentDirectory.h"
#include "VQMMC/common/PMSFVectorQuantizedPayloadAdapter.h"

// 我们直接 dynamic_pointer_cast 拿到 VQMMC 的具体 collector，访问其
// queues。这条路径只在 impl/VQMMC 里被构造，因此 collector 一定是
// VQMMCSymbolCollectOperator 实例。
#include "VQMMCSymbolCollectOperator.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#ifdef COLUMNAR_SUPPORT
#include "storage/PMSFSymbolPayloadBlobWriter.h"
#include "storage/PMSFUnifiedArtifactWriter.h"
#include "PMSFPayloadAdapterRegistry.h"
#include <arrow/api.h>
#include <arrow/builder.h>
#include <arrow/io/api.h>
#include <parquet/arrow/writer.h>
#include <parquet/properties.h>
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

bool vqmmcSkipCompatTables() {
    return pmsf::pmsfStorageEnvEnabled("PMSF_VQMMC_SKIP_COMPAT_TABLES");
}

bool vqmmcSkipCompatSymbolsTable() {
    return vqmmcSkipCompatTables() ||
           pmsf::pmsfStorageEnvEnabled("PMSF_VQMMC_SKIP_COMPAT_SYMBOLS_TABLE");
}

bool vqmmcSkipCompatVqTable() {
    return vqmmcSkipCompatTables() ||
           pmsf::pmsfStorageEnvEnabled("PMSF_VQMMC_SKIP_COMPAT_VQ_TABLE");
}

bool vqmmcSkipCompatBaseTables() {
    return vqmmcSkipCompatTables() ||
           pmsf::pmsfStorageEnvEnabled("PMSF_VQMMC_SKIP_COMPAT_BASE_TABLES");
}

#ifdef COLUMNAR_SUPPORT
// 通用 Parquet writer：ZSTD level 3，统一 page size。
std::shared_ptr<parquet::WriterProperties> buildWriterProps(bool dictForLargeBlobs) {
    auto b = parquet::WriterProperties::Builder();
    b.compression(parquet::Compression::ZSTD);
    b.compression_level(3);
    b.data_pagesize(128 * 1024);
    b.write_batch_size(4096);
    b.enable_statistics();
    if (!dictForLargeBlobs) {
        b.disable_dictionary();
    }
    return b.build();
}

bool writeTable(const std::string& path,
                std::shared_ptr<arrow::Schema> schema,
                std::shared_ptr<arrow::Table> table,
                bool dictForLargeBlobs) {
    auto outR = arrow::io::FileOutputStream::Open(path);
    if (!outR.ok()) {
        std::cerr << "[VQMMC-PV2] open failed: " << path << std::endl;
        return false;
    }
    auto out = outR.ValueOrDie();
    auto wp = buildWriterProps(dictForLargeBlobs);
    auto ap = parquet::ArrowWriterProperties::Builder().store_schema()->build();

    std::unique_ptr<parquet::arrow::FileWriter> writer;
    auto st = parquet::arrow::FileWriter::Open(*schema, arrow::default_memory_pool(),
                                               out, wp, ap).Value(&writer);
    if (!st.ok()) {
        std::cerr << "[VQMMC-PV2] writer open failed: " << st.ToString() << std::endl;
        return false;
    }
    st = writer->WriteTable(*table, table->num_rows());
    if (!st.ok()) {
        std::cerr << "[VQMMC-PV2] writeTable failed: " << st.ToString() << std::endl;
        return false;
    }
    (void)writer->Close();
    (void)out->Close();
    std::cout << "[VQMMC-PV2] write: " << path << std::endl;
    return true;
}

// R8：把同一张表写进**内存**而不是磁盘。字节与 writeTable 完全同构
// （同 schema / 同 writer properties），供 `_compact.bin` 的通用段目录承载。
std::vector<std::uint8_t> writeTableToBytes(
    std::shared_ptr<arrow::Schema> schema,
    std::shared_ptr<arrow::Table> table,
    bool dictForLargeBlobs) {
    std::vector<std::uint8_t> outBytes;
    auto outR = arrow::io::BufferOutputStream::Create();
    if (!outR.ok()) {
        std::cerr << "[VQMMC-PV2] buffer stream create failed" << std::endl;
        return outBytes;
    }
    auto out = outR.ValueOrDie();
    auto wp = buildWriterProps(dictForLargeBlobs);
    auto ap = parquet::ArrowWriterProperties::Builder().store_schema()->build();

    std::unique_ptr<parquet::arrow::FileWriter> writer;
    auto st = parquet::arrow::FileWriter::Open(*schema, arrow::default_memory_pool(),
                                               out, wp, ap).Value(&writer);
    if (!st.ok()) {
        std::cerr << "[VQMMC-PV2] in-memory writer open failed: " << st.ToString() << std::endl;
        return outBytes;
    }
    st = writer->WriteTable(*table, table->num_rows());
    if (!st.ok()) {
        std::cerr << "[VQMMC-PV2] in-memory writeTable failed: " << st.ToString() << std::endl;
        return outBytes;
    }
    (void)writer->Close();
    auto bufR = out->Finish();
    if (!bufR.ok()) {
        return outBytes;
    }
    auto buf = bufR.ValueOrDie();
    outBytes.assign(buf->data(), buf->data() + buf->size());
    return outBytes;
}

bool collectCompactSegmentOrWriteFile(
    const std::string& path,
    std::shared_ptr<arrow::Schema> schema,
    std::shared_ptr<arrow::Table> table,
    bool dictForLargeBlobs,
    std::vector<pmsf::CompactSegment>& segs,
    const char* segmentName) {
    if (pmsf::interopExportEnabled()) {
        if (!writeTable(path, schema, table, dictForLargeBlobs)) return false;
    }
    pmsf::CompactSegment seg;
    seg.name = segmentName;
    seg.bytes = writeTableToBytes(schema, table, dictForLargeBlobs);
    if (seg.bytes.empty()) return false;
    segs.push_back(std::move(seg));
    return true;
}

/// 析构时把收集到的段写成 `<base>_compact.bin`（通用段目录，v6 容器）。
struct CompactSegmentsFlusher {
    std::vector<pmsf::CompactSegment>& segs;
    std::string baseName;
    int codecVersion = 0;
    int blobLayout = 0;
    std::int32_t totalVertices = 0;
    std::int32_t totalFaces = 0;
    bool hasQuant = false;
    std::int32_t quantBits = 0;
    float bboxMinX = 0.0f;
    float bboxMinY = 0.0f;
    float bboxMinZ = 0.0f;
    float quantStep = 0.0f;
    mutable bool done = false;
    ~CompactSegmentsFlusher() { flush(); }
    void flush() const {
        if (done || segs.empty()) return;
        done = true;
        pmsf::PaperCompactPayloadHeader hdr;
        hdr.hasQuant = hasQuant;
        hdr.flags = hasQuant ? 0x1 : 0;
        hdr.codecVersion = codecVersion;
        hdr.blobLayout = blobLayout;
        hdr.origTotalVertices = totalVertices;
        hdr.origTotalFaces = totalFaces;
        hdr.quantBits = quantBits;
        hdr.bboxMinX = bboxMinX;
        hdr.bboxMinY = bboxMinY;
        hdr.bboxMinZ = bboxMinZ;
        hdr.quantStep = quantStep;
        const auto payload = pmsf::buildCompactSegmentPayload(hdr, segs);
        std::vector<std::uint8_t> wrapped;
        if (!pmsf::wrapCompactBinPayload(payload, wrapped)) return;
        const std::string binPath = baseName + pmsf::kPaperCompactBinSuffix;
        std::ofstream ofs(binPath, std::ios::binary);
        ofs.write(reinterpret_cast<const char*>(wrapped.data()),
                  static_cast<std::streamsize>(wrapped.size()));
        std::cout << "[VQMMC-PV2] write: " << binPath
                  << " segments=" << segs.size();
        std::cout << std::endl;
    }
};
#endif  // COLUMNAR_SUPPORT

void appendI32LE(std::vector<uint8_t>& out, std::int32_t v) {
    const auto u = static_cast<std::uint32_t>(v);
    out.push_back(static_cast<uint8_t>(u & 0xffu));
    out.push_back(static_cast<uint8_t>((u >> 8) & 0xffu));
    out.push_back(static_cast<uint8_t>((u >> 16) & 0xffu));
    out.push_back(static_cast<uint8_t>((u >> 24) & 0xffu));
}

void appendU32LE(std::vector<uint8_t>& out, std::uint32_t u) {
    out.push_back(static_cast<uint8_t>(u & 0xffu));
    out.push_back(static_cast<uint8_t>((u >> 8) & 0xffu));
    out.push_back(static_cast<uint8_t>((u >> 16) & 0xffu));
    out.push_back(static_cast<uint8_t>((u >> 24) & 0xffu));
}

void appendF32LE(std::vector<uint8_t>& out, float v) {
    std::uint32_t u = 0;
    std::memcpy(&u, &v, sizeof(v));
    appendU32LE(out, u);
}

// 把 boolean 流（每元素一字节 0/1）按位打包后再 zlib 压缩，等价于
// VQMMCSymbolCollectOperator::exportToBuffer 在 facetSymbolBuffer / edgeSymbolBuffer
// 上做的事，但跳过中间 buffer 的复制。
//
// 返回（origBytes, compBytes, payload）：
//   - origBytes : 位压缩后的字节数（未 zlib 时的真实长度）
//   - compBytes : zlib 后的字节数；若 >= origBytes 则为 0（表示透传 origPacked）
//   - payload   : 实际写入的字节序列（zlib 输出 或 直接位压缩输出）
struct PackedSegment {
    int                  origBytes = 0;
    int                  compBytes = 0;  // 0 = 未压缩
    std::vector<uint8_t> payload;
};

std::vector<uint8_t> payloadFromPackedSegment(const PackedSegment& segment) {
    std::vector<uint8_t> out;
    out.reserve(8 + segment.payload.size());
    appendI32LE(out, segment.origBytes);
    appendI32LE(out, segment.compBytes);
    out.insert(out.end(), segment.payload.begin(), segment.payload.end());
    return out;
}

PackedSegment packBitStreamThenZlib(const std::vector<std::deque<char>>& roundQueues,
                                      bool allowRc = false) {
    // 与 VQMMCSymbolCollectOperator::exportToBuffer 完全一致的位序：
    //   * 反向遍历轮次（roundQueues.size()-1 → 0）
    //   * 每轮内部按 collect 时的 push_back 顺序写每比特
    //   * 每轮结束 +1 byte（对应 facetSymbolOffset++）
    //
    // 通过直接调用 BufferUtils.h 的 writeBitsOff（big-endian 跨字节 OR 写入）
    // 确保字节级与 exportToBuffer 一致。
    // P16 诊断：把符号序列 dump 出来，供 python 侧算多阶/条件熵，
    // 判断 zlib 的 0.187 bit/sym 还有多少余量。仅 env 门控，不改码流。
        std::size_t bitCount = 0;
    for (const auto& queue : roundQueues) {
        bitCount += queue.size();
    }
    const std::size_t estimatedBytes = (bitCount + 7) / 8 + roundQueues.size() + 8;
    std::vector<char> buf(std::max<std::size_t>(estimatedBytes, 1), 0);

    std::size_t byteOff = 0;
    for (int i = static_cast<int>(roundQueues.size()) - 1; i >= 0; --i) {
        unsigned bitOff = 0;
        for (char sym : roundQueues[i]) {
            ::writeBitsOff(static_cast<uint32_t>(sym & 1), 1, buf.data(),
                           bitOff, byteOff);
        }
        // 与 exportToBuffer 一致：facetSymbolOffset++（无条件）。
        byteOff++;
    }

    PackedSegment seg;
    seg.origBytes = static_cast<int>(byteOff);
    if (seg.origBytes <= 0) {
        seg.compBytes = 0;
        return seg;
    }

    // P16：facet 走自适应二元 RC（回环 md5 已验证等价）。
    // 默认开启；设 VQMMC_FACET_RC=0 可退回 zlib 以便对照。
    static const bool kFacetRcDefault =
        !(std::getenv("VQMMC_FACET_RC") && std::getenv("VQMMC_FACET_RC")[0] == '0');
    if (kFacetRcDefault && allowRc) {
        std::deque<uint8_t> bits;
        for (int i = 0; i < seg.origBytes; ++i) {
            const uint8_t byte = static_cast<uint8_t>(buf[static_cast<std::size_t>(i)]);
            for (int b = 7; b >= 0; --b) bits.push_back((byte >> b) & 1u);
        }
        std::vector<uint8_t> rc = pmsf::encodeAlphabetStream(bits, 2, nullptr);
        std::cerr << "[RC-DIAG] origBytes=" << seg.origBytes
                  << " bits=" << bits.size()
                  << " ones=" << std::count(bits.begin(), bits.end(), (uint8_t)1)
                  << " rc=" << rc.size() << std::endl;
        seg.compBytes = -static_cast<int>(rc.size());
        seg.payload.assign(rc.begin(), rc.end());
        return seg;
    }
    std::vector<uint8_t> compressed;
    if (zlibCompress(reinterpret_cast<const uint8_t*>(buf.data()),
                     static_cast<size_t>(seg.origBytes), compressed)
        && static_cast<int>(compressed.size()) < seg.origBytes) {
        seg.compBytes = static_cast<int>(compressed.size());
        seg.payload   = std::move(compressed);
    } else {
        seg.compBytes = 0;       // 透传
        seg.payload.assign(buf.data(), buf.data() + seg.origBytes);
    }
    return seg;
}

}  // namespace

// ============================================================================
// SerializeOperator 接口
// ============================================================================

void VQMMCColumnarSerializeOperator::init() {
    buffer_ = nullptr;
    dataOffset_ = 0;
}

void VQMMCColumnarSerializeOperator::serializeInt(int /*val*/) {
    // 列式版本不通过 buffer 写 int；接口保留为空。
}

void VQMMCColumnarSerializeOperator::serializeBaseMesh(std::shared_ptr<MCGAL::Mesh> /*mesh*/) {
    // VQMMC 必须用带 seed 的版本。
}

void VQMMCColumnarSerializeOperator::serializeBaseMeshWithSeed(
    std::shared_ptr<MCGAL::Mesh> mesh,
    std::vector<MCGAL::Halfedge*> seeds) {
    mesh->garbage_collection();
    baseMesh_.numVertices = static_cast<int>(mesh->size_of_vertices());
    baseMesh_.numFaces    = static_cast<int>(mesh->size_of_facets());

    // VQMMC 默认开启 quantization（impl 入口固定 setEnableQuantization(true)）。
    // 我们在 quant 模式下持久化 int 量化坐标；非 quant 模式作为 fallback 用
    // bbox=0 / step=1 把浮点直接转 int 存储。
    baseMesh_.nbQuantBits = enableQuantization_ ? mesh->i_nbQuantBits : 0;
    if (enableQuantization_) {
        baseMesh_.bboxMin[0] = mesh->bboxMin.x();
        baseMesh_.bboxMin[1] = mesh->bboxMin.y();
        baseMesh_.bboxMin[2] = mesh->bboxMin.z();
        baseMesh_.quantStep  = mesh->f_quantStep;
    }

    // 1) Seed 的两个端点先入列（id=0, 1，与 BasicSerializeOperator 一致）。
    int id = 0;
    MCGAL::Halfedge* seed = seeds[0];
    seed->vertex()->setVid(id++);
    seed->end_vertex()->setVid(id++);

    auto pointToInts = [&](const MCGAL::Point& p, int out[3]) {
        if (enableQuantization_) {
            MCGAL::PointInt pi = mesh->floatPosToInt(p);
            int upper = (1 << mesh->i_nbQuantBits) - 1;
            for (int k = 0; k < 3; ++k) {
                int v = pi[k];
                if (v < 0)     v = 0;
                if (v > upper) v = upper;
                out[k] = v;
            }
        } else {
            // 非 quant：把浮点转为 int 后粗存（仅作 fallback，VQMMC 默认不走）
            out[0] = static_cast<int>(p.x() * 1e6);
            out[1] = static_cast<int>(p.y() * 1e6);
            out[2] = static_cast<int>(p.z() * 1e6);
        }
    };
    pointToInts(seed->vertex()->point(),     baseMesh_.seedV0Q);
    pointToInts(seed->end_vertex()->point(), baseMesh_.seedV1Q);

    // 2) 其余顶点
    baseMesh_.vertexQX.reserve(baseMesh_.numVertices);
    baseMesh_.vertexQY.reserve(baseMesh_.numVertices);
    baseMesh_.vertexQZ.reserve(baseMesh_.numVertices);
    for (MCGAL::Vertex* vit : mesh->vertices()) {
        if (vit->isRemoved()) continue;
        if (vit->poolId() == seed->vertex()->poolId()) continue;
        if (vit->poolId() == seed->end_vertex()->poolId()) continue;
        int q[3];
        pointToInts(vit->point(), q);
        baseMesh_.vertexQX.push_back(q[0]);
        baseMesh_.vertexQY.push_back(q[1]);
        baseMesh_.vertexQZ.push_back(q[2]);
        vit->setVid(id++);
    }

    // 3) 每面顶点索引（任意度数；与 BasicSerializeOperator 同顺序）
    baseMesh_.connBits = enableQuantization_
        ? static_cast<int>(std::ceil(std::log2(baseMesh_.numVertices + 1)))
        : 0;
    baseMesh_.faceVertices.reserve(baseMesh_.numFaces * 3);
    baseMesh_.faceDegrees.reserve(baseMesh_.numFaces);
    for (MCGAL::Facet* fit : mesh->faces()) {
        if (fit->isRemoved()) continue;
        MCGAL::Halfedge* st = fit->proxyHalfedge();
        MCGAL::Halfedge* ed = st;
        int deg = 0;
        do {
            baseMesh_.faceVertices.push_back(st->vertex()->vid());
            ++deg;
            st = st->next();
        } while (st != ed);
        baseMesh_.faceDegrees.push_back(deg);
    }

    baseMesh_.valid = true;
}

// ============================================================================
// 4-表写出
// ============================================================================


// R8：段收集 + 可选文件导出。默认只收集（唯一规范形态是 `_compact.bin`）；
// 需要与外部工具互操作时置 PMSF_INTEROP_EXPORT=1 才额外落 Parquet。

void VQMMCColumnarSerializeOperator::serialize(std::string path) {

#ifndef COLUMNAR_SUPPORT
    (void)path;
    std::cerr << "[VQMMC-PV2] Parquet support not compiled in" << std::endl;
    return;
#else
    if (!baseMesh_.valid) {
        std::cerr << "[VQMMC-PV2] serializeBaseMeshWithSeed 未调用，base mesh 为空" << std::endl;
        return;
    }
    auto vqColl = std::dynamic_pointer_cast<VQMMCSymbolCollectOperator>(collector_);
    if (!vqColl) {
        std::cerr << "[VQMMC-PV2] collector 不是 VQMMCSymbolCollectOperator" << std::endl;
        return;
    }
    const auto& facetQ    = vqColl->facetSymbolQueues();
    const auto& edgeQ     = vqColl->edgeSymbolQueues();
    const auto& vqIndexQ  = vqColl->vqIndexQueues();
    const auto& codebookQ = vqColl->codebookQueues();

    const std::string baseName = stripParquetExt(path);

    // R8：唯一规范形态 `_compact.bin` 的段收集（见 CompactSegmentDirectory.h）
    std::vector<pmsf::CompactSegment> compactSegs;
    const CompactSegmentsFlusher compactFlusher{
        compactSegs, baseName, 4, 0,
        static_cast<std::int32_t>(baseMesh_.numVertices),
        static_cast<std::int32_t>(baseMesh_.numFaces),
        enableQuantization_ && baseMesh_.nbQuantBits > 0,
        static_cast<std::int32_t>(baseMesh_.nbQuantBits),
        baseMesh_.bboxMin[0],
        baseMesh_.bboxMin[1],
        baseMesh_.bboxMin[2],
        baseMesh_.quantStep};
    const int numRounds = static_cast<int>(facetQ.size());

    // ---- Pack facet/edge 位流 + zlib（与 exportToBuffer 字节级对齐） ----
    PackedSegment facetSeg = packBitStreamThenZlib(facetQ, /*allowRc=*/true);
    PackedSegment edgeSeg  = packBitStreamThenZlib(edgeQ);
    const bool emitPMSFPayloads = pmsf::unifiedStorageEnabled();
    const bool skipCompatSymbolsTable =
        emitPMSFPayloads && vqmmcSkipCompatSymbolsTable();
    const bool skipCompatVqTable =
        emitPMSFPayloads && vqmmcSkipCompatVqTable();
    const bool skipCompatBaseTables =
        emitPMSFPayloads && vqmmcSkipCompatBaseTables();
    std::vector<pmsf::PaperNamedBlob> paperBlobs;
    if (emitPMSFPayloads) {
        pmsf::PMSFSymbolPayloadTable vqPayloadRows;
        pmsf::appendVQMMCPMSFPayload(vqPayloadRows,
                                      0,
                                      pmsf::kVQMMCPayloadFacetSymbolsName,
                                      pmsf::kVQMMCPayloadBitstreamCodecName,
                                      payloadFromPackedSegment(facetSeg));
        pmsf::appendVQMMCPMSFPayload(vqPayloadRows,
                                      0,
                                      pmsf::kVQMMCPayloadEdgeSymbolsName,
                                      pmsf::kVQMMCPayloadBitstreamCodecName,
                                      payloadFromPackedSegment(edgeSeg));
        if (skipCompatBaseTables) {
            std::vector<std::uint8_t> metaPayload;
            appendI32LE(metaPayload, 1);  // payload format version
            appendI32LE(metaPayload, 3);  // VQMMC Parquet V2 codec_version
            appendI32LE(metaPayload, numRounds);
            appendI32LE(metaPayload, header_ && header_->isEnableCompress() ? 1 : 0);
            appendI32LE(metaPayload, header_ ? (header_->isEnableQuantization() ? 1 : 0)
                                             : (enableQuantization_ ? 1 : 0));
            appendI32LE(metaPayload, header_ && header_->isEnableSegmentation() ? 1 : 0);
            appendI32LE(metaPayload, header_ && header_->isEnableAdjustment() ? 1 : 0);
            appendI32LE(metaPayload, baseMesh_.numVertices);
            appendI32LE(metaPayload, baseMesh_.numFaces);
            appendI32LE(metaPayload, baseMesh_.nbQuantBits);
            appendF32LE(metaPayload, baseMesh_.bboxMin[0]);
            appendF32LE(metaPayload, baseMesh_.bboxMin[1]);
            appendF32LE(metaPayload, baseMesh_.bboxMin[2]);
            appendF32LE(metaPayload, baseMesh_.quantStep);
            appendI32LE(metaPayload, baseMesh_.connBits);
            appendI32LE(metaPayload, facetSeg.origBytes);
            appendI32LE(metaPayload, facetSeg.compBytes);
            appendI32LE(metaPayload, edgeSeg.origBytes);
            appendI32LE(metaPayload, edgeSeg.compBytes);
            pmsf::appendVQMMCPMSFPayload(vqPayloadRows,
                                          0,
                                          pmsf::kVQMMCPayloadMetadataName,
                                          pmsf::kVQMMCCompatPayloadCodecName,
                                          std::move(metaPayload));

            std::vector<std::uint8_t> basePayload;
            appendI32LE(basePayload, 1);  // payload format version
            for (int value : baseMesh_.seedV0Q) appendI32LE(basePayload, value);
            for (int value : baseMesh_.seedV1Q) appendI32LE(basePayload, value);
            appendI32LE(basePayload, static_cast<std::int32_t>(baseMesh_.vertexQX.size()));
            for (int value : baseMesh_.vertexQX) appendI32LE(basePayload, value);
            for (int value : baseMesh_.vertexQY) appendI32LE(basePayload, value);
            for (int value : baseMesh_.vertexQZ) appendI32LE(basePayload, value);
            appendI32LE(basePayload, static_cast<std::int32_t>(baseMesh_.faceVertices.size()));
            for (int value : baseMesh_.faceVertices) appendI32LE(basePayload, value);
            appendI32LE(basePayload, static_cast<std::int32_t>(baseMesh_.faceDegrees.size()));
            for (int value : baseMesh_.faceDegrees) appendI32LE(basePayload, value);
            pmsf::appendVQMMCPMSFPayload(vqPayloadRows,
                                          0,
                                          pmsf::kVQMMCPayloadBaseMeshName,
                                          pmsf::kVQMMCCompatPayloadCodecName,
                                          std::move(basePayload));
            addPaperSidecarValue(pmsf::kVQMMCCompatBaseTablesSkippedSideName, 1);
            addPaperSidecarValue(pmsf::kVQMMCMetadataPayloadPresentSideName, 1);
            addPaperSidecarValue(pmsf::kVQMMCBaseMeshPayloadPresentSideName, 1);
        } else {
            addPaperSidecarValue(pmsf::kVQMMCCompatBaseTablesSkippedSideName, 0);
            addPaperSidecarValue(pmsf::kVQMMCMetadataPayloadPresentSideName, 0);
            addPaperSidecarValue(pmsf::kVQMMCBaseMeshPayloadPresentSideName, 0);
        }
        pmsf::appendVQMMCPMSFPayloadRows(vqPayloadRows, vqIndexQ, codebookQ);
        pmsf::appendPMSFPayloadRows(paperBlobs, vqPayloadRows);
    }

    // ---- (1) metadata ----
    if (!skipCompatBaseTables) {
        auto schema = arrow::schema({
            arrow::field("variant",             arrow::utf8(),    false),
            arrow::field("codec_version",       arrow::int32(),   false),
            arrow::field("num_rounds",          arrow::int32(),   false),
            arrow::field("enable_compress",     arrow::boolean(), false),
            arrow::field("enable_quantization", arrow::boolean(), false),
            arrow::field("enable_segmentation", arrow::boolean(), false),
            arrow::field("enable_adjustment",   arrow::boolean(), false),
            arrow::field("num_vertices_base",   arrow::int32(),   false),
            arrow::field("num_faces_base",      arrow::int32(),   false),
            arrow::field("quant_bits",          arrow::int32(),   false),
            arrow::field("bbox_min_x",          arrow::float32(), false),
            arrow::field("bbox_min_y",          arrow::float32(), false),
            arrow::field("bbox_min_z",          arrow::float32(), false),
            arrow::field("quant_step",          arrow::float32(), false),
            arrow::field("conn_bits",           arrow::int32(),   false),
            arrow::field("facet_orig_size",     arrow::int32(),   false),
            arrow::field("facet_comp_size",     arrow::int32(),   false),
            arrow::field("edge_orig_size",      arrow::int32(),   false),
            arrow::field("edge_comp_size",      arrow::int32(),   false),
        });
        arrow::StringBuilder  variantB;
        arrow::Int32Builder   codecB, roundsB, nvB, nfB, qbB, connB,
                              foB, fcB, eoB, ecB;
        arrow::BooleanBuilder eC, eQ, eS, eA;
        arrow::FloatBuilder   bxB, byB, bzB, qsB;

        (void)variantB.Append(std::string("vqmmc"));
        (void)codecB.Append(3);                     // 3 = column-native VQMMC
        (void)roundsB.Append(numRounds);
        (void)eC.Append(header_ ? header_->isEnableCompress()       : false);
        (void)eQ.Append(header_ ? header_->isEnableQuantization()   : enableQuantization_);
        (void)eS.Append(header_ ? header_->isEnableSegmentation()   : false);
        (void)eA.Append(header_ ? header_->isEnableAdjustment()     : false);
        (void)nvB.Append(baseMesh_.numVertices);
        (void)nfB.Append(baseMesh_.numFaces);
        (void)qbB.Append(baseMesh_.nbQuantBits);
        (void)bxB.Append(baseMesh_.bboxMin[0]);
        (void)byB.Append(baseMesh_.bboxMin[1]);
        (void)bzB.Append(baseMesh_.bboxMin[2]);
        (void)qsB.Append(baseMesh_.quantStep);
        (void)connB.Append(baseMesh_.connBits);
        (void)foB.Append(facetSeg.origBytes);
        (void)fcB.Append(facetSeg.compBytes);
        (void)eoB.Append(edgeSeg.origBytes);
        (void)ecB.Append(edgeSeg.compBytes);

        std::shared_ptr<arrow::Array> a[19];
        (void)variantB.Finish(&a[0]);  (void)codecB.Finish(&a[1]);
        (void)roundsB.Finish(&a[2]);   (void)eC.Finish(&a[3]);
        (void)eQ.Finish(&a[4]);        (void)eS.Finish(&a[5]);
        (void)eA.Finish(&a[6]);        (void)nvB.Finish(&a[7]);
        (void)nfB.Finish(&a[8]);       (void)qbB.Finish(&a[9]);
        (void)bxB.Finish(&a[10]);      (void)byB.Finish(&a[11]);
        (void)bzB.Finish(&a[12]);      (void)qsB.Finish(&a[13]);
        (void)connB.Finish(&a[14]);    (void)foB.Finish(&a[15]);
        (void)fcB.Finish(&a[16]);      (void)eoB.Finish(&a[17]);
        (void)ecB.Finish(&a[18]);

        auto table = arrow::Table::Make(schema,
            {a[0],a[1],a[2],a[3],a[4],a[5],a[6],a[7],a[8],a[9],
             a[10],a[11],a[12],a[13],a[14],a[15],a[16],a[17],a[18]}, 1);
        if (!collectCompactSegmentOrWriteFile(baseName + "_metadata.parquet", schema, table, true, compactSegs, "metadata")) return;
    } else {
        std::cout << "[VQMMC-PV2] skip compat _metadata.parquet; "
                  << "using unified paper payload" << std::endl;
    }

    // ---- (2) base_mesh ----
    if (!skipCompatBaseTables) {
        // 量化坐标 / 面顶点 id / 面度数都 ≤ 16-bit（量化最多 12bit；base mesh
        // 顶点数对常见网格远小于 32K）。用 INT16 列存比 INT32 列存节省 50%
        // 字节，配合 Parquet 字典编码后 BPV 与原 inner buffer 接近。
        auto schema = arrow::schema({
            arrow::field("seed_v0_qx", arrow::int32(), false),
            arrow::field("seed_v0_qy", arrow::int32(), false),
            arrow::field("seed_v0_qz", arrow::int32(), false),
            arrow::field("seed_v1_qx", arrow::int32(), false),
            arrow::field("seed_v1_qy", arrow::int32(), false),
            arrow::field("seed_v1_qz", arrow::int32(), false),
            arrow::field("vertex_qx",  arrow::list(arrow::int16()), false),
            arrow::field("vertex_qy",  arrow::list(arrow::int16()), false),
            arrow::field("vertex_qz",  arrow::list(arrow::int16()), false),
            arrow::field("face_vertices", arrow::list(arrow::int32()), false),
            arrow::field("face_degrees",  arrow::list(arrow::int8()),  false),
        });
        arrow::Int32Builder s0x, s0y, s0z, s1x, s1y, s1z;
        auto vxVb = std::make_shared<arrow::Int16Builder>();
        arrow::ListBuilder vxLb(arrow::default_memory_pool(), vxVb);
        auto vyVb = std::make_shared<arrow::Int16Builder>();
        arrow::ListBuilder vyLb(arrow::default_memory_pool(), vyVb);
        auto vzVb = std::make_shared<arrow::Int16Builder>();
        arrow::ListBuilder vzLb(arrow::default_memory_pool(), vzVb);
        auto fvVb = std::make_shared<arrow::Int32Builder>();
        arrow::ListBuilder fvLb(arrow::default_memory_pool(), fvVb);
        auto fdVb = std::make_shared<arrow::Int8Builder>();
        arrow::ListBuilder fdLb(arrow::default_memory_pool(), fdVb);

        (void)s0x.Append(baseMesh_.seedV0Q[0]); (void)s0y.Append(baseMesh_.seedV0Q[1]); (void)s0z.Append(baseMesh_.seedV0Q[2]);
        (void)s1x.Append(baseMesh_.seedV1Q[0]); (void)s1y.Append(baseMesh_.seedV1Q[1]); (void)s1z.Append(baseMesh_.seedV1Q[2]);

        auto clamp16 = [](int v) -> int16_t {
            if (v > 32767) return 32767;
            if (v < -32768) return -32768;
            return static_cast<int16_t>(v);
        };
        (void)vxLb.Append(); for (int v : baseMesh_.vertexQX) (void)vxVb->Append(clamp16(v));
        (void)vyLb.Append(); for (int v : baseMesh_.vertexQY) (void)vyVb->Append(clamp16(v));
        (void)vzLb.Append(); for (int v : baseMesh_.vertexQZ) (void)vzVb->Append(clamp16(v));
        (void)fvLb.Append(); for (int v : baseMesh_.faceVertices) (void)fvVb->Append(v);
        (void)fdLb.Append(); for (int v : baseMesh_.faceDegrees)
            (void)fdVb->Append(static_cast<int8_t>(v));

        std::shared_ptr<arrow::Array> a[11];
        (void)s0x.Finish(&a[0]); (void)s0y.Finish(&a[1]); (void)s0z.Finish(&a[2]);
        (void)s1x.Finish(&a[3]); (void)s1y.Finish(&a[4]); (void)s1z.Finish(&a[5]);
        (void)vxLb.Finish(&a[6]); (void)vyLb.Finish(&a[7]); (void)vzLb.Finish(&a[8]);
        (void)fvLb.Finish(&a[9]); (void)fdLb.Finish(&a[10]);

        auto table = arrow::Table::Make(schema,
            {a[0],a[1],a[2],a[3],a[4],a[5],a[6],a[7],a[8],a[9],a[10]}, 1);
        if (!collectCompactSegmentOrWriteFile(baseName + "_base_mesh.parquet", schema, table, true, compactSegs, "baseMesh")) return;
    } else {
        std::cout << "[VQMMC-PV2] skip compat _base_mesh.parquet; "
                  << "using unified paper payload" << std::endl;
    }

    // ---- (3) symbols（facet + edge 各一列 BINARY） ----
    if (!skipCompatSymbolsTable) {
        auto schema = arrow::schema({
            arrow::field("facet_symbols", arrow::binary(), false),
            arrow::field("edge_symbols",  arrow::binary(), false),
        });
        arrow::BinaryBuilder fb, eb;
        if (facetSeg.compBytes != 0) {
            // P16：负值 = RC 编码，长度取绝对值（原判据 `>0` 会把负值误判成
            // 「未压缩」并去读 origBytes 字节 ⇒ 越界 + 错误的段大小）
            (void)fb.Append(facetSeg.payload.data(),
                            static_cast<int32_t>(facetSeg.compBytes < 0
                                                     ? -facetSeg.compBytes
                                                     : facetSeg.compBytes));
        } else {
            (void)fb.Append(facetSeg.payload.data(),
                            static_cast<int32_t>(facetSeg.origBytes));
        }
        if (edgeSeg.compBytes > 0) {
            (void)eb.Append(edgeSeg.payload.data(),
                            static_cast<int32_t>(edgeSeg.compBytes));
        } else {
            (void)eb.Append(edgeSeg.payload.data(),
                            static_cast<int32_t>(edgeSeg.origBytes));
        }
        std::shared_ptr<arrow::Array> fa, ea;
        (void)fb.Finish(&fa);
        (void)eb.Finish(&ea);
        auto table = arrow::Table::Make(schema, {fa, ea}, 1);
        if (!collectCompactSegmentOrWriteFile(baseName + "_symbols.parquet", schema, table, false, compactSegs, "symbols")) return;
    } else {
        std::cout << "[VQMMC-PV2] skip compat _symbols.parquet; "
                  << "using unified paper payload" << std::endl;
    }

    // ---- (4) vq（每轮一行：codebook 量化 + 索引） ----
    if (!skipCompatVqTable) {
        auto schema = arrow::schema({
            arrow::field("round_id",       arrow::int32(),   false),
            arrow::field("codebook_size",  arrow::int32(),   false),
            arrow::field("num_indices",    arrow::int32(),   false),
            arrow::field("cb_quant_bits",  arrow::int32(),   false),
            arrow::field("cb_min_x",       arrow::float32(), false),
            arrow::field("cb_min_y",       arrow::float32(), false),
            arrow::field("cb_min_z",       arrow::float32(), false),
            arrow::field("cb_step_x",      arrow::float32(), false),
            arrow::field("cb_step_y",      arrow::float32(), false),
            arrow::field("cb_step_z",      arrow::float32(), false),
            arrow::field("cb_quantized_x", arrow::list(arrow::int16()), false),
            arrow::field("cb_quantized_y", arrow::list(arrow::int16()), false),
            arrow::field("cb_quantized_z", arrow::list(arrow::int16()), false),
            arrow::field("idx_bits",       arrow::int32(),   false),
            arrow::field("indices",        arrow::list(arrow::int16()), false),
        });
        arrow::Int32Builder ridB, cbsB, niB, cbqB, idxBitsB;
        arrow::FloatBuilder cmxB, cmyB, cmzB, csxB, csyB, cszB;
        auto cqxVb = std::make_shared<arrow::Int16Builder>(); arrow::ListBuilder cqxLb(arrow::default_memory_pool(), cqxVb);
        auto cqyVb = std::make_shared<arrow::Int16Builder>(); arrow::ListBuilder cqyLb(arrow::default_memory_pool(), cqyVb);
        auto cqzVb = std::make_shared<arrow::Int16Builder>(); arrow::ListBuilder cqzLb(arrow::default_memory_pool(), cqzVb);
        auto idxVb = std::make_shared<arrow::Int16Builder>(); arrow::ListBuilder idxLb(arrow::default_memory_pool(), idxVb);

        const int codebookQuantBits = pmsf::vqmmcConfiguredCodebookQuantBits();
        const uint32_t cbQuantMax = (1u << codebookQuantBits) - 1u;

        // 与 exportToBuffer 同的 reverse 顺序（最旧轮在前）。
        for (int i = static_cast<int>(codebookQ.size()) - 1; i >= 0; --i) {
            const auto& cb  = codebookQ[i];
            const auto& idx = vqIndexQ[i];
            int cbSize  = static_cast<int>(cb.size());
            int numIdx  = static_cast<int>(idx.size());
            const int roundId = static_cast<int>(codebookQ.size()) - 1 - i;
            (void)ridB.Append(roundId);
            (void)cbsB.Append(pmsf::vqmmcSignedCodebookSize(
                cbSize, pmsf::vqmmcConfiguredAbsoluteGeometry()));
            (void)niB.Append(numIdx);

            float cMin[3] = { 0, 0, 0 };
            float cStep[3] = { 1, 1, 1 };
            if (cbSize > 0) {
                float lo[3] = { 1e30f, 1e30f, 1e30f };
                float hi[3] = { -1e30f, -1e30f, -1e30f };
                for (int j = 0; j < cbSize; ++j) {
                    float v[3] = {(float)cb[j].x(), (float)cb[j].y(), (float)cb[j].z()};
                    for (int k = 0; k < 3; ++k) {
                        if (v[k] < lo[k]) lo[k] = v[k];
                        if (v[k] > hi[k]) hi[k] = v[k];
                    }
                }
                for (int k = 0; k < 3; ++k) {
                    cMin[k]  = lo[k];
                    float r  = hi[k] - lo[k];
                    if (r <= 0) r = 1e-9f;
                    cStep[k] = r / static_cast<float>(cbQuantMax);
                }
            }
            (void)cbqB.Append(codebookQuantBits);
            (void)cmxB.Append(cMin[0]);  (void)cmyB.Append(cMin[1]);  (void)cmzB.Append(cMin[2]);
            (void)csxB.Append(cStep[0]); (void)csyB.Append(cStep[1]); (void)cszB.Append(cStep[2]);

            (void)cqxLb.Append(); (void)cqyLb.Append(); (void)cqzLb.Append();
            for (int j = 0; j < cbSize; ++j) {
                float v[3] = {(float)cb[j].x(), (float)cb[j].y(), (float)cb[j].z()};
                int q[3];
                for (int k = 0; k < 3; ++k) {
                    int qi = static_cast<int>(std::round((v[k] - cMin[k]) / cStep[k]));
                    if (qi < 0) qi = 0;
                    if (qi > static_cast<int>(cbQuantMax)) {
                        qi = static_cast<int>(cbQuantMax);
                    }
                    q[k] = qi;
                }
                (void)cqxVb->Append(static_cast<int16_t>(q[0]));
                (void)cqyVb->Append(static_cast<int16_t>(q[1]));
                (void)cqzVb->Append(static_cast<int16_t>(q[2]));
            }

            int idxBits = 1;
            if (cbSize > 1) {
                int k = 0;
                uint32_t v = static_cast<uint32_t>(cbSize - 1);
                while (v) { v >>= 1; ++k; }
                idxBits = std::max(1, k);
            }
            (void)idxBitsB.Append(idxBits);
            (void)idxLb.Append();
            for (uint16_t v : idx) {
                (void)idxVb->Append(static_cast<int16_t>(v));
            }
        }

        std::shared_ptr<arrow::Array> a[15];
        (void)ridB.Finish(&a[0]);  (void)cbsB.Finish(&a[1]);  (void)niB.Finish(&a[2]);
        (void)cbqB.Finish(&a[3]);
        (void)cmxB.Finish(&a[4]);  (void)cmyB.Finish(&a[5]);  (void)cmzB.Finish(&a[6]);
        (void)csxB.Finish(&a[7]);  (void)csyB.Finish(&a[8]);  (void)cszB.Finish(&a[9]);
        (void)cqxLb.Finish(&a[10]); (void)cqyLb.Finish(&a[11]); (void)cqzLb.Finish(&a[12]);
        (void)idxBitsB.Finish(&a[13]); (void)idxLb.Finish(&a[14]);

        auto table = arrow::Table::Make(schema,
            {a[0],a[1],a[2],a[3],a[4],a[5],a[6],a[7],a[8],a[9],a[10],a[11],a[12],a[13],a[14]},
            numRounds);
        if (!collectCompactSegmentOrWriteFile(baseName + "_vq.parquet", schema, table, true, compactSegs, "vq")) return;
    } else {
        std::cout << "[VQMMC-PV2] skip compat _vq.parquet; using unified paper payload"
                  << std::endl;
    }

    if (emitPMSFPayloads) {
        writeUnifiedPMSFArtifactsForSerializerAndNamedBlobs(
            baseName, std::move(paperBlobs), false);
    }

    std::cout << "[VQMMC-PV2] variant=vqmmc rounds=" << numRounds
              << " base_v=" << baseMesh_.numVertices
              << " base_f=" << baseMesh_.numFaces
              << " facet_orig=" << facetSeg.origBytes
              << "/comp=" << facetSeg.compBytes
              << " edge_orig=" << edgeSeg.origBytes
              << "/comp=" << edgeSeg.compBytes
              << std::endl;
#endif  // COLUMNAR_SUPPORT
}
