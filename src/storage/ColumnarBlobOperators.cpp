#include "storage/ColumnarBlobOperators.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include "header/PMSFHeader.h"
#include "storage/PMSFSymbolPayloadBlobWriter.h"
#include "storage/PMSFUnifiedArtifactWriter.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <arrow/adapters/orc/adapter.h>
#include <arrow/api.h>
#include <arrow/io/api.h>
#include <arrow/ipc/reader.h>
#include <arrow/ipc/writer.h>
#include <arrow/table.h>
#include <arrow/util/compression.h>

namespace pmsf {
namespace {

std::string tempPathFor(const std::string& path, ColumnarBlobFormat format) {
    return paperColumnarBlobTempPayloadPath(path, columnarBlobFormatName(format));
}

void removeUnifiedPMSFArtifactsForPath(const std::string& path) {
    const std::string base = stripOutputExtensionForPMSFArtifacts(path);
    removePaperSidecarAndPayloadBlobsForBase(base);
}

bool readWholeFile(const std::string& path, std::vector<char>& out) {
    std::ifstream fin(path, std::ios::binary | std::ios::ate);
    if (!fin) return false;
    std::streamsize n = fin.tellg();
    if (n < 0) return false;
    fin.seekg(0, std::ios::beg);
    out.resize(static_cast<std::size_t>(n));
    if (n == 0) return true;
    return static_cast<bool>(fin.read(out.data(), n));
}

bool fileExists(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    return f.good();
}

bool loadUnifiedColumnarPayload(const DeserializeOperator& op,
                                ColumnarBlobFormat format,
                                std::string& payload) {
    const auto* bytes = op.findPMSFSymbolPayloadBytes(
        0, 0, 0, columnarPayloadName(format));
    if (!bytes) {
        if (const auto* row = op.findPMSFSymbolPayloadByRole(
                PMSFPayloadRole::ColumnarContainer)) {
            if (row->codecName == columnarPayloadCodec(format) &&
                !row->payload.empty()) {
                bytes = &row->payload;
            }
        }
    }
    if (!bytes) return false;
    payload.assign(reinterpret_cast<const char*>(bytes->data()),
                   reinterpret_cast<const char*>(bytes->data()) + bytes->size());
    return true;
}

bool writeWholeFile(const std::string& path, const std::string& payload) {
    std::ofstream fout(path, std::ios::binary);
    if (!fout) return false;
    fout.write(payload.data(), static_cast<std::streamsize>(payload.size()));
    return static_cast<bool>(fout);
}

std::shared_ptr<arrow::Schema> blobSchema() {
    return arrow::schema({
        arrow::field(pmsf::kPayloadWrapperFormatColumn, arrow::utf8(), false),
        arrow::field(pmsf::kPayloadWrapperCodecVersionColumn, arrow::int32(), false),
        arrow::field(pmsf::kParquetNumRoundsColumn, arrow::int32(), false),
        arrow::field(pmsf::kParquetEnableCompressColumn, arrow::boolean(), false),
        arrow::field(pmsf::kParquetEnableQuantizationColumn, arrow::boolean(), false),
        arrow::field(pmsf::kParquetEnableSegmentationColumn, arrow::boolean(), false),
        arrow::field(pmsf::kParquetEnableAdjustmentColumn, arrow::boolean(), false),
        arrow::field(pmsf::kPayloadWrapperPayloadBytesColumn, arrow::int64(), false),
        arrow::field(pmsf::kPayloadWrapperPayloadColumn, arrow::binary(), false),
    });
}

std::shared_ptr<arrow::Table> makeBlobTable(ColumnarBlobFormat format,
                                            const std::shared_ptr<PMSFHeader>& header,
                                            const std::vector<char>& payload) {
    arrow::StringBuilder formatB;
    arrow::Int32Builder codecB;
    arrow::Int32Builder roundsB;
    arrow::BooleanBuilder compressB;
    arrow::BooleanBuilder quantB;
    arrow::BooleanBuilder segmentB;
    arrow::BooleanBuilder adjustB;
    arrow::Int64Builder payloadBytesB;
    arrow::BinaryBuilder payloadB;

    int rounds = 0;
    bool enableCompress = false;
    bool enableQuantization = false;
    bool enableSegmentation = false;
    bool enableAdjustment = false;
    if (header) {
        rounds = header->getRound();
        enableCompress = header->isEnableCompress();
        enableQuantization = header->isEnableQuantization();
        enableSegmentation = header->isEnableSegmentation();
        enableAdjustment = header->isEnableAdjustment();
    }

    const auto wrapper = makePMSFPayloadWrapperDescriptor(
        columnarBlobFormatName(format),
        kColumnarBlobWrapperCodecVersion,
        kPMSFPayloadWrapperNoBlobLayout,
        rounds,
        enableCompress,
        enableQuantization,
        enableSegmentation,
        enableAdjustment,
        payload.size());

    (void)formatB.Append(wrapper.format);
    (void)codecB.Append(wrapper.codecVersion);
    (void)roundsB.Append(wrapper.numRounds);
    (void)compressB.Append(wrapper.enableCompress);
    (void)quantB.Append(wrapper.enableQuantization);
    (void)segmentB.Append(wrapper.enableSegmentation);
    (void)adjustB.Append(wrapper.enableAdjustment);
    (void)payloadBytesB.Append(wrapper.payloadBytes);
    (void)payloadB.Append(payload.data(), static_cast<int32_t>(payload.size()));

    std::shared_ptr<arrow::Array> formatA;
    std::shared_ptr<arrow::Array> codecA;
    std::shared_ptr<arrow::Array> roundsA;
    std::shared_ptr<arrow::Array> compressA;
    std::shared_ptr<arrow::Array> quantA;
    std::shared_ptr<arrow::Array> segmentA;
    std::shared_ptr<arrow::Array> adjustA;
    std::shared_ptr<arrow::Array> payloadBytesA;
    std::shared_ptr<arrow::Array> payloadA;

    (void)formatB.Finish(&formatA);
    (void)codecB.Finish(&codecA);
    (void)roundsB.Finish(&roundsA);
    (void)compressB.Finish(&compressA);
    (void)quantB.Finish(&quantA);
    (void)segmentB.Finish(&segmentA);
    (void)adjustB.Finish(&adjustA);
    (void)payloadBytesB.Finish(&payloadBytesA);
    (void)payloadB.Finish(&payloadA);

    return arrow::Table::Make(blobSchema(),
                              {formatA, codecA, roundsA, compressA, quantA,
                               segmentA, adjustA, payloadBytesA, payloadA},
                              1);
}

bool writeArrowFile(const std::string& path, const std::shared_ptr<arrow::Table>& table) {
    auto outResult = arrow::io::FileOutputStream::Open(path);
    if (!outResult.ok()) {
        std::cerr << "[ArrowBlob] open output failed: " << path << " "
                  << outResult.status().ToString() << std::endl;
        return false;
    }
    auto out = outResult.ValueOrDie();

    arrow::ipc::IpcWriteOptions options = arrow::ipc::IpcWriteOptions::Defaults();
    auto codecResult = arrow::util::Codec::Create(arrow::Compression::ZSTD);
    if (codecResult.ok()) {
        auto codec = std::move(codecResult).ValueOrDie();
        options.codec = std::shared_ptr<arrow::util::Codec>(std::move(codec));
    }

    auto writerResult = arrow::ipc::MakeFileWriter(out.get(), table->schema(), options);
    if (!writerResult.ok()) {
        std::cerr << "[ArrowBlob] create writer failed: "
                  << writerResult.status().ToString() << std::endl;
        return false;
    }
    auto writer = writerResult.ValueOrDie();
    auto st = writer->WriteTable(*table);
    if (!st.ok()) {
        std::cerr << "[ArrowBlob] write table failed: " << st.ToString() << std::endl;
        return false;
    }
    st = writer->Close();
    if (!st.ok()) {
        std::cerr << "[ArrowBlob] close writer failed: " << st.ToString() << std::endl;
        return false;
    }
    (void)out->Close();
    return true;
}

bool writeOrcFile(const std::string& path, const std::shared_ptr<arrow::Table>& table) {
    auto outResult = arrow::io::FileOutputStream::Open(path);
    if (!outResult.ok()) {
        std::cerr << "[ORCBlob] open output failed: " << path << " "
                  << outResult.status().ToString() << std::endl;
        return false;
    }
    auto out = outResult.ValueOrDie();

    arrow::adapters::orc::WriteOptions options;
    options.compression = arrow::Compression::ZSTD;
    options.compression_strategy = arrow::adapters::orc::CompressionStrategy::kCompression;
    options.dictionary_key_size_threshold = 1.0;

    auto writerResult = arrow::adapters::orc::ORCFileWriter::Open(out.get(), options);
    if (!writerResult.ok()) {
        std::cerr << "[ORCBlob] create writer failed: "
                  << writerResult.status().ToString() << std::endl;
        return false;
    }
    auto writer = std::move(writerResult).ValueOrDie();
    auto st = writer->Write(*table);
    if (!st.ok()) {
        std::cerr << "[ORCBlob] write table failed: " << st.ToString() << std::endl;
        return false;
    }
    st = writer->Close();
    if (!st.ok()) {
        std::cerr << "[ORCBlob] close writer failed: " << st.ToString() << std::endl;
        return false;
    }
    (void)out->Close();
    return true;
}

std::shared_ptr<arrow::Table> readArrowFile(const std::string& path) {
    auto fileResult = arrow::io::ReadableFile::Open(path);
    if (!fileResult.ok()) {
        std::cerr << "[ArrowBlob] open input failed: " << path << " "
                  << fileResult.status().ToString() << std::endl;
        return nullptr;
    }
    auto file = fileResult.ValueOrDie();
    auto readerResult = arrow::ipc::RecordBatchFileReader::Open(file);
    if (!readerResult.ok()) {
        std::cerr << "[ArrowBlob] create reader failed: "
                  << readerResult.status().ToString() << std::endl;
        return nullptr;
    }
    auto reader = readerResult.ValueOrDie();
    std::vector<std::shared_ptr<arrow::RecordBatch>> batches;
    batches.reserve(reader->num_record_batches());
    for (int i = 0; i < reader->num_record_batches(); ++i) {
        auto batchResult = reader->ReadRecordBatch(i);
        if (!batchResult.ok()) {
            std::cerr << "[ArrowBlob] read batch failed: "
                      << batchResult.status().ToString() << std::endl;
            return nullptr;
        }
        batches.push_back(batchResult.ValueOrDie());
    }
    auto tableResult = arrow::Table::FromRecordBatches(reader->schema(), batches);
    if (!tableResult.ok()) {
        std::cerr << "[ArrowBlob] build table failed: "
                  << tableResult.status().ToString() << std::endl;
        return nullptr;
    }
    return tableResult.ValueOrDie();
}

std::shared_ptr<arrow::Table> readOrcFile(const std::string& path) {
    auto fileResult = arrow::io::ReadableFile::Open(path);
    if (!fileResult.ok()) {
        std::cerr << "[ORCBlob] open input failed: " << path << " "
                  << fileResult.status().ToString() << std::endl;
        return nullptr;
    }
    auto file = fileResult.ValueOrDie();
    auto readerResult =
        arrow::adapters::orc::ORCFileReader::Open(file, arrow::default_memory_pool());
    if (!readerResult.ok()) {
        std::cerr << "[ORCBlob] create reader failed: "
                  << readerResult.status().ToString() << std::endl;
        return nullptr;
    }
    auto reader = std::move(readerResult).ValueOrDie();
    auto tableResult = reader->Read();
    if (!tableResult.ok()) {
        std::cerr << "[ORCBlob] read table failed: "
                  << tableResult.status().ToString() << std::endl;
        return nullptr;
    }
    return tableResult.ValueOrDie();
}

bool extractPayload(const std::shared_ptr<arrow::Table>& table,
                    ColumnarBlobFormat expectedFormat,
                    std::string& payload) {
    if (!table || table->num_rows() < 1) {
        std::cerr << "[ColumnarBlob] empty storage table" << std::endl;
        return false;
    }

    auto formatCol = table->GetColumnByName(pmsf::kPayloadWrapperFormatColumn);
    if (formatCol && formatCol->length() > 0 && formatCol->num_chunks() > 0) {
        auto arr = std::static_pointer_cast<arrow::StringArray>(formatCol->chunk(0));
        std::string actual = arr->GetString(0);
        std::string expected = columnarBlobFormatName(expectedFormat);
        if (actual != expected) {
            std::cerr << "[ColumnarBlob] warning: format mismatch, expected "
                      << expected << " but file says " << actual << std::endl;
        }
    }

    auto payloadCol = table->GetColumnByName(pmsf::kPayloadWrapperPayloadColumn);
    if (!payloadCol || payloadCol->length() < 1 || payloadCol->num_chunks() < 1) {
        std::cerr << "[ColumnarBlob] payload column missing" << std::endl;
        return false;
    }
    for (int i = 0; i < payloadCol->num_chunks(); ++i) {
        if (payloadCol->chunk(i)->length() == 0) continue;
        auto bin = std::static_pointer_cast<arrow::BinaryArray>(payloadCol->chunk(i));
        payload = bin->GetString(0);
        return true;
    }
    std::cerr << "[ColumnarBlob] payload column empty" << std::endl;
    return false;
}

bool writeStorageFile(ColumnarBlobFormat format,
                      const std::string& path,
                      const std::shared_ptr<PMSFHeader>& header,
                      const std::vector<char>& payload) {
    auto table = makeBlobTable(format, header, payload);
    switch (format) {
        case ColumnarBlobFormat::ArrowIpc:
            return writeArrowFile(path, table);
        case ColumnarBlobFormat::ORC:
            return writeOrcFile(path, table);
    }
    return false;
}

std::shared_ptr<arrow::Table> readStorageFile(ColumnarBlobFormat format,
                                              const std::string& path) {
    switch (format) {
        case ColumnarBlobFormat::ArrowIpc:
            return readArrowFile(path);
        case ColumnarBlobFormat::ORC:
            return readOrcFile(path);
    }
    return nullptr;
}

}  // namespace

ColumnarBlobSerializeOperator::ColumnarBlobSerializeOperator(
    std::shared_ptr<SerializeOperator> inner, ColumnarBlobFormat format)
    : SerializeOperator(inner ? inner->collector_ : nullptr,
                        inner ? inner->enablePrediction_ : false,
                        inner ? inner->enableQuantization_ : false,
                        inner ? inner->enableCompress_ : false),
      inner_(std::move(inner)),
      format_(format) {}

void ColumnarBlobSerializeOperator::init() {
    if (inner_) inner_->init();
    if (inner_) header_ = inner_->header_;
}

void ColumnarBlobSerializeOperator::serializeInt(int val) {
    if (inner_) inner_->serializeInt(val);
}

void ColumnarBlobSerializeOperator::serializeHeader(PMSFHeader header) {
    if (inner_) inner_->serializeHeader(header);
    header_ = std::make_shared<PMSFHeader>(header);
}

void ColumnarBlobSerializeOperator::setHeader(std::shared_ptr<PMSFHeader> header) {
    if (inner_) inner_->setHeader(header);
    header_ = std::move(header);
}

void ColumnarBlobSerializeOperator::serializeBaseMesh(std::shared_ptr<MCGAL::Mesh> mesh) {
    if (inner_) inner_->serializeBaseMesh(std::move(mesh));
}

void ColumnarBlobSerializeOperator::serializeBaseMeshWithSeed(
    std::shared_ptr<MCGAL::Mesh> mesh,
    std::vector<MCGAL::Halfedge*> seeds) {
    if (inner_) inner_->serializeBaseMeshWithSeed(std::move(mesh), std::move(seeds));
}

void ColumnarBlobSerializeOperator::serialize(std::string path) {
    if (!inner_) {
        std::cerr << "[ColumnarBlob] inner serializer is null" << std::endl;
        return;
    }

    const std::string temp = tempPathFor(path, format_);
    inner_->serialize(temp);

    std::vector<char> payload;
    if (!readWholeFile(temp, payload)) {
        std::cerr << "[ColumnarBlob] read inner payload failed: " << temp << std::endl;
        return;
    }
    std::remove(temp.c_str());
    removeUnifiedPMSFArtifactsForPath(temp);

    const bool skipCompatFile =
        unifiedStorageEnabled() && columnarSkipCompatBlobFileRequested();
    if (!skipCompatFile && !writeStorageFile(format_, path, header_, payload)) {
        std::cerr << "[ColumnarBlob] write " << columnarBlobFormatName(format_)
                  << " storage failed: " << path << std::endl;
        return;
    } else if (skipCompatFile) {
        std::cout << "[ColumnarBlob] skip compat " << columnarBlobFormatName(format_)
                  << " file; using unified paper payload" << std::endl;
    }
    pmsf::PMSFSymbolPayloadTable payloadRows;
    pmsf::appendColumnarBlobPayload(
        payloadRows, format_, payload.data(), payload.size());
    std::vector<pmsf::PaperNamedBlob> blobs =
        pmsf::paperNamedBlobsFromPayloadRows(payloadRows);
    writeUnifiedPMSFArtifactsForSerializerAndNamedBlobs(path, std::move(blobs));
    if (!collectorHasPMSFSymbolMapping(collector_)) {
        writePMSFPayloadOnlyFallbackSidecarForSerializer(
            stripOutputExtensionForPMSFArtifacts(path),
            columnarSidecarPrefix(format_));
    }

    std::cout << "[ColumnarBlob] wrote " << columnarBlobFormatName(format_)
              << " file: " << path
              << " payload=" << payload.size() << " bytes" << std::endl;
}

ColumnarBlobDeserializeOperator::ColumnarBlobDeserializeOperator(
    std::shared_ptr<DeserializeOperator> inner, ColumnarBlobFormat format)
    : DeserializeOperator(inner ? inner->symbolReadOperator_ : nullptr,
                          inner ? inner->reconstructOperator_ : nullptr),
      inner_(std::move(inner)),
      format_(format) {}

void ColumnarBlobDeserializeOperator::syncFromInner() {
    if (!inner_) return;
    buffer_ = inner_->buffer_;
    dataOffset_ = inner_->dataOffset_;
    pointOffset_ = inner_->pointOffset_;
    totalSize = inner_->totalSize;
    mesh_ = inner_->mesh_;
    header_ = inner_->header();
}

void ColumnarBlobDeserializeOperator::init(std::string path) {
    clearPMSFSymbolContext();
    loadPMSFSymbolSidecarIfPresent(path);
    auto table = fileExists(path) ? readStorageFile(format_, path) : nullptr;
    std::string payload;
    if (table && !extractPayload(table, format_, payload)) {
        (void)loadUnifiedColumnarPayload(*this, format_, payload);
    } else if (!table) {
        (void)loadUnifiedColumnarPayload(*this, format_, payload);
    }
    if (payload.empty()) return;

    const std::string temp = tempPathFor(path, format_);
    if (!writeWholeFile(temp, payload)) {
        std::cerr << "[ColumnarBlob] write temp payload failed: " << temp << std::endl;
        return;
    }
    if (!inner_) {
        std::cerr << "[ColumnarBlob] inner deserializer is null" << std::endl;
        std::remove(temp.c_str());
        return;
    }

    inner_->init(temp);
    std::remove(temp.c_str());
    syncFromInner();
    propagatePMSFSymbolContextTo(inner_);
    header_ = PMSFHeader();

    std::cout << "[ColumnarBlob] read " << columnarBlobFormatName(format_)
              << " file: " << path
              << " payload=" << payload.size() << " bytes" << std::endl;
}

void ColumnarBlobDeserializeOperator::setSeeds(std::vector<MCGAL::Halfedge*> seeds) {
    seeds_ = seeds;
    if (inner_) inner_->setSeeds(std::move(seeds));
}

void ColumnarBlobDeserializeOperator::deserializeHeader(PMSFHeader& header) {
    if (!inner_) return;
    inner_->deserializeHeader(header);
    syncFromInner();
    header_ = header;
}

std::shared_ptr<MCGAL::Mesh> ColumnarBlobDeserializeOperator::deserializeBaseMesh() {
    if (!inner_) return nullptr;
    auto mesh = inner_->deserializeBaseMesh();
    syncFromInner();
    return mesh;
}

void ColumnarBlobDeserializeOperator::deserializeOneRound() {
    if (!inner_) return;
    inner_->deserializeOneRound();
    syncFromInner();
}

std::shared_ptr<MCGAL::Mesh> ColumnarBlobDeserializeOperator::exportMesh() {
    if (inner_) return inner_->exportMesh();
    return mesh_;
}

PMSFHeader ColumnarBlobDeserializeOperator::header() {
    return header_;
}

}  // namespace pmsf
