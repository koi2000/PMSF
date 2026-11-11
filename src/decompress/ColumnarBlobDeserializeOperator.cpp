/*
 * @Author: mig-v2
 * @Description: Parquet V2 通用 BLOB 包装反序列化器实现
 */
#include "ColumnarBlobDeserializeOperator.h"
#include "storage/PMSFSymbolStorageAdapter.h"
#include "storage/ColumnarBlobPMSFPayloadAdapter.h"
#include "header/PMSFHeader.h"
#include "PMSFPayloadAdapterRegistry.h"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/reader.h>

namespace {

static bool fileExists(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    return f.good();
}

static std::shared_ptr<arrow::Table> readParquetFile(const std::string& path) {
    auto fileResult = arrow::io::ReadableFile::Open(path);
    if (!fileResult.ok()) {
        std::cerr << "[ColumnarBlob] 打开失败: " << path << " (" << fileResult.status().ToString() << ")" << std::endl;
        return nullptr;
    }
    auto infile = fileResult.ValueOrDie();

    std::unique_ptr<parquet::arrow::FileReader> reader;
    auto st = parquet::arrow::OpenFile(infile, arrow::default_memory_pool()).Value(&reader);
    if (!st.ok()) {
        std::cerr << "[ColumnarBlob] 解析失败: " << path << " (" << st.ToString() << ")" << std::endl;
        return nullptr;
    }
    std::shared_ptr<arrow::Table> table;
    st = reader->ReadTable(&table);
    if (!st.ok()) {
        std::cerr << "[ColumnarBlob] 读取 table 失败: " << path << std::endl;
        return nullptr;
    }
    return table;
}

static std::string payloadRecordToString(const pmsf::PMSFSymbolPayloadRecord& row) {
    if (row.payload.empty()) return {};
    return std::string(reinterpret_cast<const char*>(row.payload.data()),
                       reinterpret_cast<const char*>(row.payload.data()) + row.payload.size());
}

}  // namespace

void ColumnarBlobDeserializeOperator::init(std::string path) {
    clearPMSFSymbolContext();

    const std::string base = pmsf::paperStorageStripExtension(path);
    const std::string metaPath = pmsf::paperMetadataTablePath(base);
    const std::string payloadPath = pmsf::paperPayloadWrapperTablePath(base);
    loadPMSFSymbolSidecarIfPresent(path);

    // ---- 读 metadata 表 ----
    auto metaTable = fileExists(metaPath) ? readParquetFile(metaPath) : nullptr;
    if (!metaTable || metaTable->num_rows() < 1) {
        std::cerr << "[ColumnarBlob] metadata 表缺失或为空，尝试 unified payload: "
                  << metaPath << std::endl;
    }

    std::string variant = expectedVariant_;
    int numRounds = 0;
    bool enComp = false, enQuant = false, enSeg = false, enAdj = false;

    if (metaTable && metaTable->num_rows() >= 1) {
        auto getCol = [&](const std::string& name) -> std::shared_ptr<arrow::ChunkedArray> {
            return metaTable->GetColumnByName(name);
        };
        auto col = getCol(pmsf::kColumnarBlobVariantColumn);
        if (col && col->length() > 0) {
            auto a = std::static_pointer_cast<arrow::StringArray>(col->chunk(0));
            variant = a->GetString(0);
        }
        auto roundsCol = getCol("num_rounds");
        if (roundsCol && roundsCol->length() > 0) {
            auto a = std::static_pointer_cast<arrow::Int32Array>(roundsCol->chunk(0));
            numRounds = a->Value(0);
        }
        auto boolAt = [&](const std::string& n, bool& out) {
            auto c = getCol(n);
            if (c && c->length() > 0) {
                auto a = std::static_pointer_cast<arrow::BooleanArray>(c->chunk(0));
                out = a->Value(0);
            }
        };
        boolAt("enable_compress", enComp);
        boolAt("enable_quantization", enQuant);
        boolAt("enable_segmentation", enSeg);
        boolAt("enable_adjustment", enAdj);
        // 解码池容量估计所需的原始网格计数（无该列的旧产物保持 0 ⇒ 默认池）。
        auto intAt = [&](const std::string& n, int& out) {
            auto c = getCol(n);
            if (c && c->length() > 0) {
                auto a = std::static_pointer_cast<arrow::Int32Array>(c->chunk(0));
                out = a->Value(0);
            }
        };
        intAt("total_vertices", origTotalVertices_);
        intAt("total_faces", origTotalFaces_);
    }

    if (!expectedVariant_.empty() && variant != expectedVariant_) {
        std::cerr << "[ColumnarBlob] [WARN] variant 不匹配: expect='"
                  << expectedVariant_ << "' actual='" << variant << "'" << std::endl;
    }

    // 构造 header 供 header() 返回
    header_ = PMSFHeader::Builder()
                  .setRound(numRounds)
                  .setEnableCompress(enComp)
                  .setEnableQuantization(enQuant)
                  .setEnableSegmentation(enSeg)
                  .setEnableAdjustMent(enAdj)
                  .build();

    // ---- 读 payload 表 ----
    std::string payloadStr;
    auto payloadTable = fileExists(payloadPath) ? readParquetFile(payloadPath) : nullptr;
    if (payloadTable && payloadTable->num_rows() >= 1) {
        auto payloadCol = payloadTable->GetColumnByName(pmsf::kPayloadWrapperPayloadColumn);
        if (payloadCol && payloadCol->length() >= 1) {
            auto binArray = std::static_pointer_cast<arrow::BinaryArray>(payloadCol->chunk(0));
            payloadStr = binArray->GetString(0);
        }
    }
    if (payloadStr.empty()) {
        const pmsf::PMSFSymbolPayloadRecord* payloadRow = nullptr;
        if (!expectedVariant_.empty()) {
            payloadRow = findPMSFSymbolPayload(
                0, 0, 0, pmsf::parquetV2BlobPayloadName(expectedVariant_));
        }
        if (!payloadRow) {
            payloadRow = findFirstPMSFSymbolPayloadByRoleAndCodecSuffix(
                pmsf::PMSFPayloadRole::AlgorithmPayload,
                pmsf::kColumnarBlobPayloadCodecSuffix);
        }
        if (payloadRow) {
            payloadStr = payloadRecordToString(*payloadRow);
            if (variant.empty()) {
                variant = pmsf::parquetV2BlobVariantFromPayloadCodec(
                    payloadRow->codecName);
            }
        }
    }
    if (payloadStr.empty()) {
        std::cerr << "[ColumnarBlob] payload 表和 unified payload 均缺失: "
                  << payloadPath << std::endl;
        return;
    }

    // ---- 写临时 bin，交给 inner->init(path) ----
    tempBinaryPath_ = pmsf::paperColumnarBlobTempPayloadPath(base);
    {
        std::ofstream fout(tempBinaryPath_, std::ios::binary);
        fout.write(payloadStr.data(), (std::streamsize)payloadStr.size());
    }

    if (!inner_) {
        std::cerr << "[ColumnarBlob] inner deserializer 为空" << std::endl;
        return;
    }
    inner_->init(tempBinaryPath_);
    // 同步成员
    buffer_ = inner_->buffer_;
    dataOffset_ = inner_->dataOffset_;
    totalSize = inner_->totalSize;

    propagatePMSFSymbolContextTo(inner_);

    std::cout << "[ColumnarBlob] variant=" << variant
              << " payload=" << payloadStr.size() << " bytes"
              << " rounds=" << numRounds << std::endl;

    // 临时文件暂不立即删除（inner 可能仍在使用 buffer_，且 init 已经把数据
    // 读入内存），保留给最后 cleanup。
    std::remove(tempBinaryPath_.c_str());
}
