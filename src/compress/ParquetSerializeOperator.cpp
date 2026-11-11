/*
 * @Author: koi
 * @Description: Parquet格式序列化操作符实现
 */
#include "ParquetSerializeOperator.h"
#include "storage/PMSFPMSFLegacySingleFilePayloadAdapter.h"
#include "storage/PMSFSymbolStorageAdapter.h"
#include "BufferUtils.h"
#include "Configuration.h"
#include "EntropyCodec.h"
#include "Halfedge.h"
#include "MeshUtils.h"
#include "Point.h"
#include "PMSFPayloadAdapterRegistry.h"
#include "storage/PMSFUnifiedArtifactWriter.h"
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>
#include <arrow/api.h>
#include <arrow/io/api.h>
#include <arrow/builder.h>
#include <parquet/arrow/writer.h>
#include <parquet/properties.h>

void ParquetSerializeOperator::init() {
    buffer_ = new char[BUFFER_SIZE];
}

void ParquetSerializeOperator::serializeInt(int val) {
    writeInt(buffer_, dataOffset_, val);
}

void ParquetSerializeOperator::serializeBaseMesh(std::shared_ptr<MCGAL::Mesh> mesh) {}

void ParquetSerializeOperator::serializeBaseMeshWithSeed(
    std::shared_ptr<MCGAL::Mesh> mesh, std::vector<MCGAL::Halfedge*> seeds) {
    baseMeshStartOffset_ = dataOffset_;
    mesh->garbage_collection();
    unsigned i_nbVerticesBaseMesh = mesh->size_of_vertices();
    unsigned i_nbFacesBaseMesh = mesh->size_of_facets();
    if (enableQuantization_) {
        writeChar(buffer_, dataOffset_, mesh->i_nbQuantBits);
        writePoint(buffer_, dataOffset_, mesh->bboxMin);
        writeFloat(buffer_, dataOffset_, mesh->f_quantStep);
    }
    writeInt(buffer_, dataOffset_, i_nbVerticesBaseMesh);
    writeInt(buffer_, dataOffset_, i_nbFacesBaseMesh);
    int id = 0;
    unsigned i_bitOffset = 0;
    MCGAL::Halfedge* seed = seeds[0];
    seed->vertex()->setVid(id++);
    seed->end_vertex()->setVid(id++);
    if (enableQuantization_) {
        MCGAL::Point p = seed->vertex()->point();
        MCGAL::PointInt pi = mesh->floatPosToInt(p);
        for (unsigned i = 0; i < 3; ++i)
            writeBitsOff((uint32_t)pi[i], mesh->i_nbQuantBits, buffer_, i_bitOffset, dataOffset_);
        p = seed->end_vertex()->point();
        pi = mesh->floatPosToInt(p);
        for (unsigned i = 0; i < 3; ++i)
            writeBitsOff((uint32_t)pi[i], mesh->i_nbQuantBits, buffer_, i_bitOffset, dataOffset_);
    } else {
        writePoint(buffer_, dataOffset_, seed->vertex()->point());
        writePoint(buffer_, dataOffset_, seed->end_vertex()->point());
    }
    for (MCGAL::Vertex* vit : mesh->vertices()) {
        if (vit->isRemoved()) continue;
        if (vit->poolId() == seed->vertex()->poolId() ||
            vit->poolId() == seed->end_vertex()->poolId()) continue;
        MCGAL::Point point = vit->point();
        if (enableQuantization_) {
            MCGAL::PointInt p = mesh->floatPosToInt(point);
            for (unsigned i = 0; i < 3; ++i) {
                if (p[i] >= (1 << mesh->i_nbQuantBits)) p[i] = (1 << mesh->i_nbQuantBits) - 1;
                else if (p[i] < 0) p[i] = 0;
                writeBitsOff(p[i], mesh->i_nbQuantBits, buffer_, i_bitOffset, dataOffset_);
            }
        } else {
            writePoint(buffer_, dataOffset_, point);
        }
        vit->setVid(id++);
    }
    char connBit = 0;
    if (enableQuantization_) {
        i_bitOffset = 0;
        dataOffset_++;
        connBit = ceil(log2(i_nbVerticesBaseMesh + 1));
        writeChar(buffer_, dataOffset_, connBit);
    }
    for (MCGAL::Facet* fit : mesh->faces()) {
        if (fit->isRemoved()) continue;
        MCGAL::Halfedge* st = fit->proxyHalfedge();
        MCGAL::Halfedge* ed = st;
        do {
            if (enableQuantization_)
                writeBitsOff((uint32_t)st->vertex()->vid(), connBit, buffer_, i_bitOffset, dataOffset_);
            else
                writeInt(buffer_, dataOffset_, st->vertex()->vid());
            st = st->next();
        } while (st != ed);
    }
    if (enableQuantization_) {
        i_bitOffset = 0;
        dataOffset_++;
    }
    baseMeshEndOffset_ = dataOffset_;
}

void ParquetSerializeOperator::serialize(std::string path) {
    int symbolSize = collector_->exportToBuffer(buffer_ + dataOffset_, enableQuantization_, enableCompress_);
    size_t headerSize = baseMeshStartOffset_;
    size_t baseMeshSize = baseMeshEndOffset_ - baseMeshStartOffset_;
    size_t symbolDataSize = symbolSize;
    int totalBinarySize = dataOffset_ + symbolSize;
    const bool skipCompatFile = pmsf::legacySingleFileSkipCompatFile();
    addPaperSidecarValue(pmsf::kLegacySingleFileCompatFileSkippedSideName,
                         skipCompatFile ? 1 : 0);
    addPaperSidecarValue(pmsf::kLegacySingleFileBackendSideName, 1);  // 1 = parquet single-file

    if (skipCompatFile) {
        auto blobs = pmsf::legacySingleFilePayloadBlob(
            buffer_, static_cast<std::size_t>(totalBinarySize));
        writeUnifiedPMSFArtifactsForSerializerAndNamedBlobs(path, std::move(blobs));
        if (!pmsf::collectorHasPMSFSymbolTable(collector_)) {
            const std::string baseName =
                pmsf::stripOutputExtensionForPMSFArtifacts(path);
            writePMSFPayloadOnlyFallbackSidecarForSerializer(
                baseName,
                pmsf::kLegacySingleFileSidecarPrefix,
                pmsf::legacySingleFileFallbackSideValues(1));
        }
        std::cout << "[ParquetSerialize] skip compat main parquet; "
                  << "using unified paper payload: " << path << std::endl;
        return;
    }

    auto schema = arrow::schema({
        arrow::field(pmsf::kLegacySingleFileHeaderDataColumn, arrow::binary()),
        arrow::field(pmsf::kLegacySingleFileBaseMeshDataColumn, arrow::binary()),
        arrow::field(pmsf::kLegacySingleFileSymbolDataColumn, arrow::binary()),
        arrow::field(pmsf::kLegacySingleFileTotalBinaryDataColumn, arrow::binary()),
        arrow::field(pmsf::kParquetNumRoundsColumn, arrow::int32()),
        arrow::field(pmsf::kParquetEnableQuantizationColumn, arrow::boolean()),
        arrow::field(pmsf::kParquetEnableCompressColumn, arrow::boolean()),
        arrow::field(pmsf::kParquetEnablePredictionColumn, arrow::boolean()),
        arrow::field(pmsf::kLegacySingleFileHeaderSizeColumn, arrow::int32()),
        arrow::field(pmsf::kLegacySingleFileBaseMeshSizeColumn, arrow::int32()),
        arrow::field(pmsf::kLegacySingleFileSymbolDataSizeColumn, arrow::int32()),
    });

    arrow::BinaryBuilder hdrB, bmB, symB, totB;
    arrow::Int32Builder nrB, hsB, bsB, ssB;
    arrow::BooleanBuilder eqB, ecB, epB;

    (void)hdrB.Append(buffer_, (int32_t)headerSize);
    (void)bmB.Append(buffer_ + baseMeshStartOffset_, (int32_t)baseMeshSize);
    (void)symB.Append(buffer_ + dataOffset_, (int32_t)symbolDataSize);
    (void)totB.Append(buffer_, (int32_t)totalBinarySize);
    (void)nrB.Append(header_->getRound());
    (void)eqB.Append(enableQuantization_);
    (void)ecB.Append(enableCompress_);
    (void)epB.Append(enablePrediction_);
    (void)hsB.Append((int32_t)headerSize);
    (void)bsB.Append((int32_t)baseMeshSize);
    (void)ssB.Append((int32_t)symbolDataSize);

    std::shared_ptr<arrow::Array> a0,a1,a2,a3,a4,a5,a6,a7,a8,a9,a10;
    (void)hdrB.Finish(&a0); (void)bmB.Finish(&a1); (void)symB.Finish(&a2); (void)totB.Finish(&a3);
    (void)nrB.Finish(&a4); (void)eqB.Finish(&a5); (void)ecB.Finish(&a6); (void)epB.Finish(&a7);
    (void)hsB.Finish(&a8); (void)bsB.Finish(&a9); (void)ssB.Finish(&a10);

    auto table = arrow::Table::Make(schema, {a0,a1,a2,a3,a4,a5,a6,a7,a8,a9,a10});
    auto result = arrow::io::FileOutputStream::Open(path);
    if (!result.ok()) { std::cerr << "无法打开输出: " << path << std::endl; return; }
    auto outfile = result.ValueOrDie();
    auto wp = parquet::WriterProperties::Builder().compression(parquet::Compression::SNAPPY)->build();
    auto ap = parquet::ArrowWriterProperties::Builder().store_schema()->build();
    auto kv = arrow::KeyValueMetadata::Make({pmsf::kPMSFVersionMetadataKey,"algorithm"},{"1.0","PMSF-Default"});
    auto tw = table->ReplaceSchemaMetadata(kv);
    auto st = parquet::arrow::WriteTable(*tw, arrow::default_memory_pool(), outfile, 1024*1024, wp, ap);
    if (!st.ok()) { std::cerr << "写入失败: " << st.ToString() << std::endl; return; }
    (void)outfile->Close();
    writeUnifiedPMSFArtifactsForSerializer(path);
    std::cout << "[ParquetSerialize] 写入: " << path << " Total=" << totalBinarySize << "B\n";
}
