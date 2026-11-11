/*
 * @Author: mig-v2
 * @Description: Parquet V2 通用 BLOB 包装序列化器实现
 *
 * 文件布局：
 *   <basename>_metadata.parquet   1 行
 *       variant   STRING    "pomar" / "vqmmc" / ...
 *       num_rounds INT32
 *       enable_compress/quant/seg/adj BOOL
 *       payload_bytes INT64    （内部 buffer 长度，含 inner 自己的 int len 前缀）
 *       codec_version INT32    （本包装协议版本号，目前=1）
 *   <basename>_payload.parquet    1 行
 *       payload   BINARY   （直接存 inner serializer 产生的完整文件内容，即
 *                           [int32 inner_len][inner_buffer]）
 *
 *   这样反序列化只需要：
 *     1) 读 metadata 确认 variant；
 *     2) 读出 payload BLOB 写回一个临时 .bin（或直接 in-memory 喂给 inner
 *        的 init(path) / init(buffer)）；
 *     3) 交给对应的 DeserializeOperator 完成实际解压。
 */
#include "ColumnarBlobSerializeOperator.h"
#include "header/PMSFHeader.h"
#include "PMSFPayloadAdapterRegistry.h"
#include "storage/PMSFSymbolPayloadBlobWriter.h"
#include "storage/PMSFSymbolStorageAdapter.h"
#include "storage/ColumnarBlobOperators.h"
#include "storage/ColumnarBlobPMSFPayloadAdapter.h"
#include "storage/PMSFUnifiedArtifactWriter.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <unistd.h>

#include <arrow/api.h>
#include <arrow/builder.h>
#include <arrow/io/api.h>
#include <parquet/arrow/writer.h>
#include <parquet/properties.h>

namespace {

struct TempFileGuard {
    std::string path;
    explicit TempFileGuard(const std::string& base)
        : path(base + ".__v2blob.XXXXXX.bin") {
        const int fd = ::mkstemps(path.data(), 4);
        if (fd < 0) {
            throw std::runtime_error("[ColumnarBlob] failed to reserve temporary payload");
        }
        ::close(fd);
    }
    TempFileGuard(const TempFileGuard&) = delete;
    TempFileGuard& operator=(const TempFileGuard&) = delete;
    ~TempFileGuard() {
        if (!path.empty()) {
            std::remove(path.c_str());
        }
    }
};

// 读整个文件到 vector<char>
static bool readWholeFile(const std::string& path, std::vector<char>& out) {
    std::ifstream fin(path, std::ios::binary | std::ios::ate);
    if (!fin) return false;
    std::streamsize n = fin.tellg();
    if (n < 0) return false;
    fin.seekg(0, std::ios::beg);
    out.resize((size_t)n);
    if (!fin.read(out.data(), n)) return false;
    return true;
}

static std::shared_ptr<arrow::Schema> buildMetadataSchema() {
    return arrow::schema({
        arrow::field(pmsf::kColumnarBlobVariantColumn, arrow::utf8(), false),
        arrow::field(pmsf::kPayloadWrapperCodecVersionColumn, arrow::int32(), false),
        arrow::field(pmsf::kParquetNumRoundsColumn, arrow::int32(), false),
        arrow::field(pmsf::kParquetEnableCompressColumn, arrow::boolean(), false),
        arrow::field(pmsf::kParquetEnableQuantizationColumn, arrow::boolean(), false),
        arrow::field(pmsf::kParquetEnableSegmentationColumn, arrow::boolean(), false),
        arrow::field(pmsf::kParquetEnableAdjustmentColumn, arrow::boolean(), false),
        arrow::field(pmsf::kPayloadWrapperPayloadBytesColumn, arrow::int64(), false),
    });
}

static std::shared_ptr<arrow::Schema> buildPayloadSchema() {
    return arrow::schema({
        arrow::field(pmsf::kPayloadWrapperPayloadColumn, arrow::binary(), false),
    });
}

static std::shared_ptr<parquet::WriterProperties> buildWriterProperties() {
    auto b = parquet::WriterProperties::Builder();
    b.compression(parquet::Compression::ZSTD);
    // level 3：BLOB 列内 inner serializer 已做完熵编码，ZSTD 仅做 padding/0 压缩，
    // level 9 相对 3 仅多 <1% 收益但编码慢 3-5 倍。见 ColumnarSerializeOperator 同注释。
    b.compression_level(3);
    b.data_pagesize(128 * 1024);
    b.write_batch_size(4096);
    b.enable_statistics();
    return b.build();
}

static bool skipCompatColumnarBlobTables() {
    return pmsf::pmsfStorageEnvEnabled("PMSF_V2BLOB_SKIP_COMPAT_TABLES");
}

static void writeTable(const std::string& path,
                       std::shared_ptr<arrow::Schema> schema,
                       std::shared_ptr<arrow::Table> table) {
    auto outResult = arrow::io::FileOutputStream::Open(path);
    if (!outResult.ok()) {
        throw std::runtime_error("[ColumnarBlob] open output failed: " +
                                 outResult.status().ToString());
    }
    auto out = outResult.ValueOrDie();
    auto writerProps = buildWriterProperties();
    auto arrowProps = parquet::ArrowWriterProperties::Builder().store_schema()->build();

    auto writerResult = parquet::arrow::FileWriter::Open(
        *schema, arrow::default_memory_pool(), out, writerProps, arrowProps);
    if (!writerResult.ok()) {
        throw std::runtime_error("[ColumnarBlob] create writer failed: " +
                                 writerResult.status().ToString());
    }
    auto writer = std::move(writerResult).ValueOrDie();
    auto st = writer->WriteTable(*table, table->num_rows());
    if (!st.ok()) {
        throw std::runtime_error("[ColumnarBlob] write table failed: " + st.ToString());
    }
    st = writer->Close();
    if (!st.ok()) {
        throw std::runtime_error("[ColumnarBlob] close writer failed: " + st.ToString());
    }
    st = out->Close();
    if (!st.ok()) {
        throw std::runtime_error("[ColumnarBlob] close output failed: " + st.ToString());
    }
    std::cout << "[ColumnarBlob] 写入: " << path << std::endl;
}

static void writeUnifiedBlobArtifacts(const std::string& baseName,
                                      const std::string& variant,
                                      const std::vector<char>& payload,
                                      const SerializeOperator* serializer,
                                      const std::shared_ptr<SymbolCollectOperator>& collector) {
    const bool hasCollectorPaper =
        pmsf::collectorHasPMSFSymbolMapping(collector);
    pmsf::PMSFSymbolPayloadTable payloadRows;
    pmsf::appendColumnarBlobPayload(
        payloadRows, variant, payload.data(), payload.size());
    std::vector<pmsf::PaperNamedBlob> blobs =
        pmsf::paperNamedBlobsFromPayloadRows(payloadRows);
    if (serializer) {
        serializer->writeUnifiedPMSFArtifactsForSerializerAndNamedBlobs(
            baseName, std::move(blobs), false);
    } else {
        pmsf::writeUnifiedPMSFArtifactsForCollectorAndNamedBlobs(
            baseName, collector, std::move(blobs), false);
    }
    if (!hasCollectorPaper) {
        if (serializer) {
            serializer->writePMSFPayloadOnlyFallbackSidecarForSerializer(
                baseName,
                pmsf::parquetV2BlobSidecarPrefix(variant));
        } else {
            pmsf::writePMSFPayloadOnlyFallbackSidecarForCollector(
                baseName,
                collector,
                pmsf::parquetV2BlobSidecarPrefix(variant));
        }
    }
}

}  // namespace

void ColumnarBlobSerializeOperator::serialize(std::string path) {
    if (!inner_) {
        throw std::runtime_error("[ColumnarBlob] inner serializer is null");
    }

    // 1) 让 inner serializer 把二进制先写到临时文件
    const std::string baseName = pmsf::paperStorageStripExtension(path);
    TempFileGuard tempFile{baseName};
    inner_->serialize(tempFile.path);

    // 2) 读入整个 inner 产物
    std::vector<char> payload;
    if (!readWholeFile(tempFile.path, payload) || payload.empty()) {
        throw std::runtime_error("[ColumnarBlob] missing or empty inner payload: " +
                                 tempFile.path);
    }

    const bool skipCompatTables =
        pmsf::unifiedStorageEnabled() && skipCompatColumnarBlobTables();

    // 3) 构建 metadata 表
    if (!skipCompatTables) {
        auto schema = buildMetadataSchema();

        arrow::StringBuilder variantB;
        arrow::Int32Builder codecB, numRoundsB;
        arrow::BooleanBuilder enCompB, enQuantB, enSegB, enAdjB;
        arrow::Int64Builder payloadBytesB;

        int numRounds = 0;
        bool enComp = false, enQuant = false, enSeg = false, enAdj = false;
        if (header_) {
            numRounds = header_->getRound();
            enComp = header_->isEnableCompress();
            enQuant = header_->isEnableQuantization();
            enSeg = header_->isEnableSegmentation();
            enAdj = header_->isEnableAdjustment();
        }
        const auto wrapper = pmsf::makePMSFPayloadWrapperDescriptor(
            variant_,
            pmsf::kColumnarBlobWrapperCodecVersion,
            pmsf::kPMSFPayloadWrapperNoBlobLayout,
            numRounds,
            enComp,
            enQuant,
            enSeg,
            enAdj,
            payload.size());
        (void)variantB.Append(wrapper.format);
        (void)codecB.Append(wrapper.codecVersion);
        (void)numRoundsB.Append(wrapper.numRounds);
        (void)enCompB.Append(wrapper.enableCompress);
        (void)enQuantB.Append(wrapper.enableQuantization);
        (void)enSegB.Append(wrapper.enableSegmentation);
        (void)enAdjB.Append(wrapper.enableAdjustment);
        (void)payloadBytesB.Append(wrapper.payloadBytes);

        std::shared_ptr<arrow::Array> variantA, codecA, numRoundsA, enCompA, enQuantA, enSegA, enAdjA, payloadBytesA;
        (void)variantB.Finish(&variantA);
        (void)codecB.Finish(&codecA);
        (void)numRoundsB.Finish(&numRoundsA);
        (void)enCompB.Finish(&enCompA);
        (void)enQuantB.Finish(&enQuantA);
        (void)enSegB.Finish(&enSegA);
        (void)enAdjB.Finish(&enAdjA);
        (void)payloadBytesB.Finish(&payloadBytesA);

        auto table = arrow::Table::Make(
            schema,
            {variantA, codecA, numRoundsA, enCompA, enQuantA, enSegA, enAdjA, payloadBytesA},
            1);
        writeTable(pmsf::paperMetadataTablePath(baseName), schema, table);
    }

    // 4) 构建 payload 表
    if (!skipCompatTables) {
        auto schema = buildPayloadSchema();
        arrow::BinaryBuilder payloadB;
        (void)payloadB.Append(payload.data(), (int32_t)payload.size());

        std::shared_ptr<arrow::Array> payloadA;
        (void)payloadB.Finish(&payloadA);

        auto table = arrow::Table::Make(schema, {payloadA}, 1);
        writeTable(pmsf::paperPayloadWrapperTablePath(baseName), schema, table);
    } else {
        std::cout << "[ColumnarBlob] skip compat metadata/payload tables; "
                  << "using unified paper payload" << std::endl;
    }

    writeUnifiedBlobArtifacts(baseName, variant_, payload, this, collector_);

    std::cout << "[ColumnarBlob] variant=" << variant_
              << " payload=" << payload.size() << " bytes" << std::endl;
}
