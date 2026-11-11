/*
 * POMARColumnarSerializeOperator 实现
 *
 * 先调用父类 `PomarColumnarSerializeOperator::serialize` 把完整 inner buffer 写进
 * 父类的 `buffer_` 与段边界字段（通过 skipFileWrite_=true 抑制文件输出），
 * 然后按段组装 4 张独立 parquet 表。
 *
 * 与 VQMMCColumnarSerializeOperator 的设计原则完全一致：
 *   - payload 列走 Parquet UNCOMPRESSED（熵编码后的字节再 ZSTD 几乎无增益），
 *     在需要的列（base_mesh_blob 等轻度可压缩段）才保留 ZSTD；
 *   - 默认 compact 模式把 metadata/base/cluster0/clusters 聚合到一张单行
 *     Parquet 表，避免四个单行文件的重复 footer/schema 开销；
 *   - codec_version = 2，与其他 Columnar 入口统一。
 */

#include "POMAR/compress/POMARColumnarSerializeOperator.h"
#include "storage/CompactSegmentDirectory.h"
#include "storage/PaperStreamTable.h"
#include "storage/PaperStreamWriterV2.h"
#include "POMAR/compress/PomarColumnarSymbolCollectOperator.h"

#include "header/PMSFHeader.h"
#include "storage/PMSFSymbolPayloadBlobWriter.h"
#include "storage/PMSFSymbolStorageAdapter.h"
#include "PMSFPayloadAdapterRegistry.h"
#include "POMAR/storage/PMSFPomarPayloadAdapter.h"
#include "storage/PMSFUnifiedArtifactWriter.h"

#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/writer.h>
#include <parquet/properties.h>

#include <cassert>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace {


std::shared_ptr<parquet::WriterProperties> makeWriterProps(bool compressPayload) {
    parquet::WriterProperties::Builder b;
    b.version(parquet::ParquetVersion::PARQUET_2_6);
    b.compression(compressPayload ? parquet::Compression::ZSTD
                                  : parquet::Compression::UNCOMPRESSED);
    if (compressPayload) {
        b.compression_level(3);
    }
    b.disable_dictionary();
    return b.build();
}

bool pomarDirectResidualEnabled() {
    const char* v = std::getenv("PMSF_POMAR_DIRECT_RESIDUAL");
    return v != nullptr && v[0] != '\0' && v[0] != '0';
}

bool pomarCompactParquetEnabled() {
    const char* v = std::getenv("PMSF_POMAR_COMPACT_PARQUET");
    return !(v != nullptr && v[0] == '0');
}

bool pomarCompactPayloadZstdEnabled() {
    const char* v = std::getenv("PMSF_POMAR_COMPACT_PAYLOAD_ZSTD");
    return v != nullptr && v[0] != '\0' && v[0] != '0';
}

void writeTableToFile(const std::shared_ptr<arrow::Table>& table,
                      const std::string& path,
                      bool compress) {
    auto file = arrow::io::FileOutputStream::Open(path).ValueOrDie();
    auto props = makeWriterProps(compress);
    auto st = parquet::arrow::WriteTable(*table, arrow::default_memory_pool(),
                                         file, /*chunk_size=*/1 << 16, props);
    if (!st.ok()) {
        throw std::runtime_error("POMAR parquet v2 write failed: " + st.ToString());
    }
    (void)file->Close();
}

std::shared_ptr<arrow::Buffer> makeBuf(const char* data, std::size_t len) {
    auto res = arrow::AllocateBuffer(static_cast<int64_t>(len));
    auto buf = std::move(res).ValueOrDie();
    std::memcpy(buf->mutable_data(), data, len);
    return std::move(buf);
}

void writeUnifiedPomarArtifacts(const SerializeOperator* serializer,
                                const std::shared_ptr<SymbolCollectOperator>& collector,
                                const std::string& stem,
                                const char* buf,
                                std::size_t baseEnd,
                                std::size_t c0End,
                                std::size_t metaEnd,
                                std::size_t bufSize) {
    pmsf::PMSFSymbolPayloadTable payloadRows =
        pmsf::buildPomarPayloadTable(
            pmsf::kPomarV2PayloadCodecName,
            buf,
            baseEnd,
            c0End,
            metaEnd,
            bufSize);
    std::vector<pmsf::PaperNamedBlob> namedBlobs;
    pmsf::appendPMSFPayloadRows(namedBlobs, payloadRows);
    if (serializer) {
        serializer->writeUnifiedPMSFArtifactsForSerializerAndNamedBlobs(
            stem, std::move(namedBlobs), false);
    } else {
        pmsf::writeUnifiedPMSFArtifactsForCollectorAndNamedBlobs(
            stem, collector, std::move(namedBlobs), false);
    }
}

}  // namespace

void POMARColumnarSerializeOperator::serialize(std::string path) {
    // ---- 1) 让父类照常走完整编码流程，但禁止写文件 ----
    setSkipFileWrite(true);
    PomarColumnarSerializeOperator::serialize(path);  // path 参数被忽略

    const char* buf = rawBuffer();
    const std::size_t bufSize = rawBufferSize();
    const std::size_t baseEnd  = rawBaseMeshEnd();
    const std::size_t c0End    = rawCluster0End();
    const std::size_t metaEnd  = rawClusterMetaEnd();
    const unsigned nbClusters  = rawNbClusters();

    if (bufSize == 0 || baseEnd > c0End || c0End > metaEnd || metaEnd > bufSize) {
        throw std::runtime_error("POMAR parquet v2 serialize: inner buffer offsets inconsistent");
    }

    // base mesh 段包含：header (5B) + 父类 serializeBaseMeshWithSeed 的内容。
    // 头 5B 与 VQMMC 走的 PMSFHeader 格式一致，这里一并保留到 base_mesh_blob，
    // 使解码端可直接 feed 给 inner->init(buffer)。
    const std::size_t baseBlobLen  = baseEnd;                       // 含 header
    const std::size_t cluster0Len  = c0End - baseEnd;
    const std::size_t metaLen      = metaEnd - c0End;
    const std::size_t restLen      = bufSize - metaEnd;

    // cluster sizes 用于切分 rest 段
    const auto& clusterSizes = rawClusterSizes();

    // 计算合法的输出路径 stem：path 可能是 ".../out.parquet"
    std::string stem = pmsf::paperStorageStripExtension(path);

    const std::string metaPath   = pmsf::paperMetadataTablePath(stem);
    const std::string baseMeshPath = pmsf::paperBaseMeshTablePath(stem);
    const std::string cluster0Path = pmsf::pomarCluster0TablePath(stem);
    const std::string clustersPath = pmsf::pomarClustersTablePath(stem);
    const std::string compactPath = pmsf::pomarCompatMainTablePath(stem);

    if (paperStream_) {
        // Keep the algorithm's native RC streams intact inside the shared v2
        // relations. They are named extension streams; no external files or
        // per-algorithm storage format are needed for replay.
        pmsf::PaperStreamTable table;
        auto& object = table.object;
        object.formatVersion = pmsf::kPaperStreamFormatVersionV2;
        object.algorithm = "pomar";
        const char* schema = std::getenv("PMSF_PAPER_SCHEMA");
        object.schemaMode = schema && std::string(schema) == "compact"
            ? pmsf::PaperSchemaMode::Compact : pmsf::PaperSchemaMode::Typed;
        object.payloadMode = pmsf::PaperPayloadMode::Typed;
        object.numPartitions = 1;
        object.numRounds = header_ ? header_->getRound() : 0;
        object.totalVertices = static_cast<std::int32_t>(rawOriginalVertices());
        object.totalFaces = static_cast<std::int32_t>(rawOriginalFaces());
        object.codecVersion = pmsf::kPomarCompatPayloadWrapperCodecVersion;
        object.blobLayout = pmsf::pomarCompatBlobLayout(pomarDirectResidualEnabled());
        object.baseMeshCodec = "pomar.v2";
        object.boundaryCodec.clear();
        // The current POMAR RC state spans several LODs. Advertise partition
        // access until the stream is split at actual RC restart points.
        object.raPrefixUnit = "partition";
        for (const char* name : {"F", "Et", "Ef", "V"}) {
            object.streams.push_back({name, pmsf::PaperCellType::Bytes, pmsf::PaperStreamEncoding::Plain,
                "pomar.v2", 0, true, pmsf::PaperCellShape::Scalar,
                pmsf::PaperStreamRelation::Connectivity, pmsf::PaperStreamSource::Payload});
        }
        for (const char* name : {"Cf", "Cv", "CfQ", "CvQ"}) {
            object.streams.push_back({name, pmsf::PaperCellType::Int32, pmsf::PaperStreamEncoding::Plain,
                "pomar.v2", 0, true, pmsf::PaperCellShape::Vec3,
                pmsf::PaperStreamRelation::Geometry, pmsf::PaperStreamSource::Payload});
        }
        for (const char* name : {pmsf::kPomarCluster0PayloadName, pmsf::kPomarMetaPayloadName, pmsf::kPomarRestPayloadName}) {
            object.streams.push_back({name, pmsf::PaperCellType::Bytes, pmsf::PaperStreamEncoding::Plain,
                "pomar.v2", 0, true, pmsf::PaperCellShape::Scalar,
                pmsf::PaperStreamRelation::Side, pmsf::PaperStreamSource::Payload});
        }
        pmsf::PaperPartitionRow partition;
        partition.numLods = object.numRounds;
        partition.baseMeshCodec = "pomar.v2";
        partition.baseMesh.assign(reinterpret_cast<const std::uint8_t*>(buf),
                                  reinterpret_cast<const std::uint8_t*>(buf) + baseEnd);
        if (collector_ && collector_->getMesh()) {
            const auto mesh = collector_->getMesh();
            partition.numVertices = static_cast<std::int32_t>(mesh->vertices().size());
            partition.numFaces = static_cast<std::int32_t>(mesh->faces().size());
            object.enableQuantization = partition.enableQuantization = true;
            object.quantBits = partition.quantBits = static_cast<std::int32_t>(mesh->i_nbQuantBits);
            object.quantStep = partition.quantStep = mesh->f_quantStep;
            for (int axis = 0; axis < 3; ++axis) object.bboxMin[axis] = partition.bboxMin[axis] = mesh->bboxMin[axis];
        }
        table.partitions.push_back(std::move(partition));
        for (int lod = 0; lod < object.numRounds; ++lod) {
            pmsf::PaperLodRow row;
            row.lodId = lod;
            if (lod == 0) {
                auto bytes = [&](std::size_t from, std::size_t to) {
                    return std::vector<std::uint8_t>(reinterpret_cast<const std::uint8_t*>(buf) + from,
                                                     reinterpret_cast<const std::uint8_t*>(buf) + to);
                };
                row.extension[pmsf::kPomarCluster0PayloadName] = bytes(baseEnd, c0End);
                row.extension[pmsf::kPomarMetaPayloadName] = bytes(c0End, metaEnd);
                row.extension[pmsf::kPomarRestPayloadName] = bytes(metaEnd, bufSize);
            }
            table.lods.push_back(std::move(row));
        }
        if (object.numRounds < 1) throw std::runtime_error("POMAR paper artifact requires at least one round");
        pmsf::PaperStreamWriterV2().write(table, stem + "_paper.parquet");
        releaseBufferNow();
        return;
    }

    const bool skipCompatTables =
        pmsf::unifiedStorageEnabled() && !keepBuffer_ && pmsf::pomarSkipCompatColumnarTables();
    if (skipCompatTables) {
        writeUnifiedPomarArtifacts(this, collector_, stem, buf, baseEnd, c0End, metaEnd, bufSize);
        std::cout << "[POMAR-PV2] skip compat parquet tables; using unified paper payload"
                  << std::endl;
        if (buffer_) {
            std::free(buffer_);
            buffer_ = nullptr;
            dataOffset_ = 0;
        }
        return;
    }

    if (pomarCompactParquetEnabled()) {
        auto schema = arrow::schema({
            arrow::field(pmsf::kPayloadWrapperFormatColumn,        arrow::utf8(), false),
            arrow::field(pmsf::kPayloadWrapperCodecVersionColumn, arrow::int32(), false),
            arrow::field(pmsf::kPayloadWrapperBlobLayoutColumn,   arrow::int32(), false),
            arrow::field(pmsf::kPayloadWrapperPayloadBytesColumn, arrow::int64(), false),
            arrow::field(pmsf::kPayloadWrapperPayloadColumn,       arrow::binary(), false),
        });

        arrow::StringBuilder formatB;
        arrow::Int32Builder cvB, blobB;
        arrow::Int64Builder payloadBytesB;
        arrow::BinaryBuilder payloadB;

        const auto wrapper = pmsf::makePMSFPayloadWrapperDescriptor(
            pmsf::kPomarCompatPayloadWrapperFormat,
            pmsf::kPomarCompatPayloadWrapperCodecVersion,
            pmsf::pomarCompatBlobLayout(pomarDirectResidualEnabled()),
            0,
            false,
            false,
            false,
            false,
            bufSize);
        (void)formatB.Append(wrapper.format);
        (void)cvB.Append(wrapper.codecVersion);
        (void)blobB.Append(wrapper.blobLayout);
        (void)payloadBytesB.Append(wrapper.payloadBytes);
        (void)payloadB.Append(buf, static_cast<int32_t>(bufSize));

        std::vector<std::shared_ptr<arrow::Array>> a(5);
        (void)formatB.Finish(&a[0]);
        (void)cvB.Finish(&a[1]);
        (void)blobB.Finish(&a[2]);
        (void)payloadBytesB.Finish(&a[3]);
        (void)payloadB.Finish(&a[4]);

        // Parquet 是**降级互操作导出**，默认不写（否则同一载荷写两遍、体积翻倍）。
        if (pmsf::interopExportEnabled()) {
            auto table = arrow::Table::Make(schema, a);
            writeTableToFile(table, compactPath, pomarCompactPayloadZstdEnabled());
        }

        // ---- R8：同一份载荷另存为唯一规范形态 `<stem>_compact.bin`（通用段目录）----
        // 段 = 载荷内部的天然切分（base / cluster0 / clusterMeta / clustersRest），
        // 目录格式本身在 src/storage/CompactSegmentDirectory.h，此处不写算法特判逻辑。
        {
            pmsf::PaperCompactPayloadHeader hdr;
            hdr.flags = (header_ && header_->isEnableQuantization()) ? 0x1 : 0x0;
            hdr.hasQuant = (hdr.flags & 0x1) != 0;
            hdr.round = header_ ? header_->getRound() : 0;
            hdr.groupNumber = 0;
            hdr.flagBits = 0;
            hdr.flagBits2 = 0;
            hdr.codecVersion = pmsf::kPomarCompatPayloadWrapperCodecVersion;
            hdr.blobLayout = wrapper.blobLayout;
            if (collector_) {
                if (auto mesh = collector_->getMesh()) {
                    hdr.origTotalVertices = static_cast<std::int32_t>(rawOriginalVertices());
                    hdr.origTotalFaces = static_cast<std::int32_t>(rawOriginalFaces());
                    // The POMAR inner codec always quantizes, independently
                    // of the generic config flag. Describe the actual grid.
                    hdr.flags |= 0x1;
                    hdr.hasQuant = true;
                    hdr.quantBits = static_cast<std::int32_t>(mesh->i_nbQuantBits);
                    hdr.bboxMinX = mesh->bboxMin.x();
                    hdr.bboxMinY = mesh->bboxMin.y();
                    hdr.bboxMinZ = mesh->bboxMin.z();
                    hdr.quantStep = mesh->f_quantStep;
                }
            }

            const auto seg = [&](const char* name, std::size_t from, std::size_t to) {
                pmsf::CompactSegment cs;
                cs.name = name;
                cs.codecId = hdr.blobLayout;
                cs.bytes.assign(reinterpret_cast<const std::uint8_t*>(buf) + from,
                                reinterpret_cast<const std::uint8_t*>(buf) + to);
                return cs;
            };
            std::vector<pmsf::CompactSegment> segs;
            segs.push_back(seg("base", 0, baseEnd));
            segs.push_back(seg("cluster0", baseEnd, c0End));
            segs.push_back(seg("clusterMeta", c0End, metaEnd));
            segs.push_back(seg("clustersRest", metaEnd, bufSize));

            const auto payload = pmsf::buildCompactSegmentPayload(hdr, segs);
            std::vector<std::uint8_t> wrapped;
            if (pmsf::wrapCompactBinPayload(payload, wrapped)) {
                const std::string binPath = stem + pmsf::kPaperCompactBinSuffix;
                std::ofstream ofs(binPath, std::ios::binary);
                ofs.write(reinterpret_cast<const char*>(wrapped.data()),
                          static_cast<std::streamsize>(wrapped.size()));
            }
        }

        std::cout << "[POMAR-PV2] write: " << compactPath << " (interop="
                  << (pmsf::interopExportEnabled() ? "on" : "off") << ")" << std::endl;
        std::cout << "[POMAR-PV2] variant=pomar compact=1 clusters=" << nbClusters
                  << " global_lod=" << rawNbGlobalLOD()
                  << " clustered_lod=" << rawNbClusteredLOD()
                  << " base=" << baseBlobLen
                  << " c0=" << cluster0Len
                  << " meta=" << metaLen
                  << " rest=" << restLen << std::endl;

        if (!keepBuffer_) {
            writeUnifiedPomarArtifacts(this, collector_, stem, buf, baseEnd, c0End, metaEnd, bufSize);
        }
        if (!keepBuffer_ && buffer_) {
            std::free(buffer_);
            buffer_ = nullptr;
            dataOffset_ = 0;
        }
        return;
    }

    // ---- 2) _metadata.parquet ----
    {
        auto schema = arrow::schema({
            arrow::field(pmsf::kPomarNumRoundsColumn,          arrow::int32(), false),
            arrow::field(pmsf::kPomarEnableCompressColumn,     arrow::boolean(), false),
            arrow::field(pmsf::kPomarEnableQuantizationColumn, arrow::boolean(), false),
            arrow::field(pmsf::kPomarEnableSegmentationColumn, arrow::boolean(), false),
            arrow::field(pmsf::kPomarEnableAdjustmentColumn,   arrow::boolean(), false),
            arrow::field(pmsf::kPomarEnablePredictionColumn,   arrow::boolean(), false),
            arrow::field(pmsf::kPomarNumClustersColumn,        arrow::int32(), false),
            arrow::field(pmsf::kPomarNbGlobalLODColumn,       arrow::int32(), false),
            arrow::field(pmsf::kPomarNbClusteredLODColumn,    arrow::int32(), false),
            arrow::field(pmsf::kPomarBaseMeshSizeColumn,      arrow::int32(), false),
            arrow::field(pmsf::kPomarCluster0SizeColumn,       arrow::int32(), false),
            arrow::field(pmsf::kPomarClusterMetaSizeColumn,   arrow::int32(), false),
            arrow::field(pmsf::kPomarClustersRestSizeColumn,  arrow::int32(), false),
            arrow::field(pmsf::kPayloadWrapperCodecVersionColumn,       arrow::int32(), false),
            arrow::field(pmsf::kPayloadWrapperBlobLayoutColumn,         arrow::int32(), false),  // 10 = POMAR_RC
            arrow::field(pmsf::kPomarRandomAccessColumn,       arrow::boolean(), false),
        });
        arrow::Int32Builder roundsB, nbClB, nbGlB, nbClLodB, baseSzB, c0SzB,
            metaSzB, restSzB, cvB, blobB;
        arrow::BooleanBuilder eC, eQ, eS, eA, eP, raB;

        (void)roundsB.Append(header_ ? header_->getRound() : 0);
        (void)eC.Append(header_ ? header_->isEnableCompress() : false);
        // The inner POMAR stream always stores quantized coordinates.
        (void)eQ.Append(true);
        (void)eS.Append(header_ ? header_->isEnableSegmentation() : false);
        (void)eA.Append(header_ ? header_->isEnableAdjustment() : false);
        (void)eP.Append(header_ ? header_->isEnablePrediction() : false);
        (void)nbClB.Append(static_cast<int>(nbClusters));
        (void)nbGlB.Append(static_cast<int>(rawNbGlobalLOD()));
        (void)nbClLodB.Append(static_cast<int>(rawNbClusteredLOD()));
        (void)baseSzB.Append(static_cast<int>(baseBlobLen));
        (void)c0SzB.Append(static_cast<int>(cluster0Len));
        (void)metaSzB.Append(static_cast<int>(metaLen));
        (void)restSzB.Append(static_cast<int>(restLen));
        (void)cvB.Append(pmsf::kPomarCompatPayloadWrapperCodecVersion);
        (void)blobB.Append(pmsf::pomarCompatBlobLayout(
            pomarDirectResidualEnabled()));
        (void)raB.Append(nbClusters > 1);  // random-access enabled iff >1 cluster

        std::shared_ptr<arrow::Array> a[16];
        (void)roundsB.Finish(&a[0]);
        (void)eC.Finish(&a[1]); (void)eQ.Finish(&a[2]); (void)eS.Finish(&a[3]);
        (void)eA.Finish(&a[4]); (void)eP.Finish(&a[5]);
        (void)nbClB.Finish(&a[6]); (void)nbGlB.Finish(&a[7]);
        (void)nbClLodB.Finish(&a[8]);
        (void)baseSzB.Finish(&a[9]); (void)c0SzB.Finish(&a[10]);
        (void)metaSzB.Finish(&a[11]); (void)restSzB.Finish(&a[12]);
        (void)cvB.Finish(&a[13]); (void)blobB.Finish(&a[14]);
        (void)raB.Finish(&a[15]);
        auto table = arrow::Table::Make(schema,
            {a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9],
             a[10], a[11], a[12], a[13], a[14], a[15]});
        writeTableToFile(table, metaPath, /*compress=*/true);
    }

    // ---- 3) _base_mesh.parquet ----
    {
        auto schema = arrow::schema({
            arrow::field(pmsf::kPomarBaseMeshBlobColumn, arrow::binary(), false),
        });
        arrow::BinaryBuilder bB;
        (void)bB.Append(buf, static_cast<int32_t>(baseBlobLen));
        std::shared_ptr<arrow::Array> a;
        (void)bB.Finish(&a);
        auto table = arrow::Table::Make(schema, {a});
        writeTableToFile(table, baseMeshPath, /*compress=*/true);
    }

    // ---- 4) _cluster0.parquet ----
    {
        auto schema = arrow::schema({
            arrow::field(pmsf::kPomarCluster0RCBlobColumn, arrow::binary(), false),
        });
        arrow::BinaryBuilder bB;
        (void)bB.Append(buf + baseEnd, static_cast<int32_t>(cluster0Len));
        std::shared_ptr<arrow::Array> a;
        (void)bB.Finish(&a);
        auto table = arrow::Table::Make(schema, {a});
        writeTableToFile(table, cluster0Path, /*compress=*/false);
    }

    // ---- 5) _clusters.parquet（单行聚合，降低 Parquet 行级开销） ----
    {
        auto schema = arrow::schema({
            arrow::field(pmsf::kPomarClusterMetaBlobColumn, arrow::binary(), false),
            arrow::field(pmsf::kPomarClustersBlobColumn,     arrow::binary(), false),
        });
        std::size_t cursor = metaEnd;
        for (unsigned c = 1; c < nbClusters; ++c) {
            unsigned sz = (c < clusterSizes.size()) ? clusterSizes[c] : 0u;
            if (cursor + sz > bufSize) {
                throw std::runtime_error("POMAR parquet v2: cluster " +
                                         std::to_string(c) + " out of bounds");
            }
            cursor += sz;
        }
        if (cursor != bufSize) {
            throw std::runtime_error("POMAR parquet v2: aggregated cluster size mismatch");
        }

        arrow::BinaryBuilder metaB, clustersB;
        (void)metaB.Append(buf + c0End, static_cast<int32_t>(metaLen));
        (void)clustersB.Append(buf + metaEnd, static_cast<int32_t>(restLen));

        std::shared_ptr<arrow::Array> aMeta, aClusters;
        (void)metaB.Finish(&aMeta);
        (void)clustersB.Finish(&aClusters);
        auto table = arrow::Table::Make(schema, {aMeta, aClusters});
        writeTableToFile(table, clustersPath, /*compress=*/false);
    }

    std::cout << "[POMAR-PV2] write: " << metaPath << ", " << baseMeshPath
              << ", " << cluster0Path << ", " << clustersPath << std::endl;
    std::cout << "[POMAR-PV2] variant=pomar clusters=" << nbClusters
              << " global_lod=" << rawNbGlobalLOD()
              << " clustered_lod=" << rawNbClusteredLOD()
              << " base=" << baseBlobLen
              << " c0=" << cluster0Len
              << " meta=" << metaLen
              << " rest=" << restLen << std::endl;

    if (!keepBuffer_) {
        writeUnifiedPomarArtifacts(this, collector_, stem, buf, baseEnd, c0End, metaEnd, bufSize);
    }

    // inner buffer 已按语义拆列输出，此时才真正释放父类的辅助缓冲 / buffer_
    // （父类 freeAuxBuffers() 被 skip；这里手动释放以复刻原析构顺序）。
    // v3-refactor 扩展：如果子类（POMARUnifiedSerializeOperator）设置了
    // keepBuffer_，则**推迟释放**，让子类在自己 serialize 结束时再读 buffer_
    // 写 `_v3_blob.parquet`，然后手动 releaseBufferNow()。
    if (!keepBuffer_) {
        if (buffer_) {
            std::free(buffer_);
            buffer_ = nullptr;
            dataOffset_ = 0;
        }
    }
    // freeAuxBuffers 是 private；p_dataMes_ / p_dataClusters_ 会随 dtor 走，
    // 但我们在父类 serialize 结束时可能已经把它们置 nullptr（b_randomAccess_=false 时），
    // 所以这里无需额外处理。
}

void POMARColumnarSerializeOperator::releaseBufferNow() {
    if (buffer_) {
        std::free(buffer_);
        buffer_ = nullptr;
        dataOffset_ = 0;
    }
}
