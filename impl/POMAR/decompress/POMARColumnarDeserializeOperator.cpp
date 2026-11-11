/*
 * POMARColumnarDeserializeOperator 实现
 *
 * init(path): 优先读取 compact 单文件 parquet；若不存在则兼容旧 4 表格式。
 * 读取后拼回单一 inner buffer（header + base mesh + cluster0 RC + cluster
 * meta + clusters rest RC）→ 通过 inner 的 `init(char* buffer)` 接管后续流程。
 */

#include "POMAR/decompress/POMARColumnarDeserializeOperator.h"
#include "storage/CompactSegmentDirectory.h"
#include "storage/PaperStreamReaderV2.h"

#include "POMAR/decompress/PomarColumnarDeserializeOperator.h"
#include "decompress/SymbolReadOperator.h"
#include "reconstruct/ReconstructOpertator.h"
#include "storage/PMSFSymbolPayloadBlobReader.h"
#include "storage/PMSFSymbolStorageAdapter.h"
#include "PMSFPayloadAdapterRegistry.h"
#include "POMAR/storage/PMSFPomarPayloadAdapter.h"

#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/reader.h>

#include <algorithm>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace {

bool fileExists(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    return f.good();
}

std::shared_ptr<arrow::Table> readTable(const std::string& path) {
    auto file = arrow::io::ReadableFile::Open(path).ValueOrDie();
    std::unique_ptr<parquet::arrow::FileReader> reader;
    auto st = parquet::arrow::OpenFile(file, arrow::default_memory_pool()).Value(&reader);
    if (!st.ok()) {
        throw std::runtime_error("POMAR-PV2 open fail: " + path + ": " + st.ToString());
    }
    std::shared_ptr<arrow::Table> table;
    st = reader->ReadTable(&table);
    if (!st.ok()) {
        throw std::runtime_error("POMAR-PV2 read fail: " + path + ": " + st.ToString());
    }
    return table;
}

std::string binaryCell(const std::shared_ptr<arrow::Table>& t, const char* name,
                       int64_t row = 0) {
    auto col = t->GetColumnByName(name);
    if (!col || col->num_chunks() == 0) return {};
    auto arr = std::dynamic_pointer_cast<arrow::BinaryArray>(col->chunk(0));
    if (!arr || row >= arr->length()) return {};
    auto sv = arr->GetString(row);
    return std::string(sv);
}

int32_t int32Cell(const std::shared_ptr<arrow::Table>& t, const char* name,
                  int64_t row = 0) {
    auto col = t->GetColumnByName(name);
    if (!col || col->num_chunks() == 0) return 0;
    auto arr = std::dynamic_pointer_cast<arrow::Int32Array>(col->chunk(0));
    if (!arr || row >= arr->length()) return 0;
    return arr->Value(row);
}

int64_t int64Cell(const std::shared_ptr<arrow::Table>& t, const char* name,
                  int64_t row = 0) {
    auto col = t->GetColumnByName(name);
    if (!col || col->num_chunks() == 0) return 0;
    auto arr = std::dynamic_pointer_cast<arrow::Int64Array>(col->chunk(0));
    if (!arr || row >= arr->length()) return 0;
    return arr->Value(row);
}

bool boolCell(const std::shared_ptr<arrow::Table>& t, const char* name,
              int64_t row = 0) {
    auto col = t->GetColumnByName(name);
    if (!col || col->num_chunks() == 0) return false;
    auto arr = std::dynamic_pointer_cast<arrow::BooleanArray>(col->chunk(0));
    if (!arr || row >= arr->length()) return false;
    return arr->Value(row);
}

std::string v3StemFromStem(const std::string& stem) {
    return pmsf::paperV3StemFromStem(stem);
}

}  // namespace

POMARColumnarDeserializeOperator::POMARColumnarDeserializeOperator(
    std::shared_ptr<SymbolReadOperator> reader,
    std::shared_ptr<ReconstructOperator> reconstruct)
    : DeserializeOperator(reader, reconstruct) {
    inner_ = std::make_shared<PomarColumnarDeserializeOperator>(reader, reconstruct);
}

POMARColumnarDeserializeOperator::~POMARColumnarDeserializeOperator() {
    delete[] assembledBuffer_;
    assembledBuffer_ = nullptr;
}

void POMARColumnarDeserializeOperator::init(std::string path) {
    clearPMSFSymbolContext();

    // 去掉可能的 .parquet 扩展名得到 stem
    std::string stem = pmsf::paperStorageStripExtension(path);

    if (paperStream_ || (path.size() >= 14 && path.compare(path.size() - 14, 14, "_paper.parquet") == 0)) {
        const std::string artifact = fileExists(path) ? path : stem + "_paper.parquet";
        pmsf::PaperStreamReaderV2 reader(artifact);
        const auto table = reader.readAll();
        if (reader.usedV1Fallback() || table.object.algorithm != "pomar" || table.partitions.size() != 1 || table.lods.empty()) {
            throw std::runtime_error("POMAR paper reader: wrong algorithm/version/partition count");
        }
        origTotalVertices_ = table.object.totalVertices;
        origTotalFaces_ = table.object.totalFaces;
        setenv("PMSF_POMAR_DIRECT_RESIDUAL", pmsf::pomarCompatBlobLayoutUsesDirectResidual(table.object.blobLayout) ? "1" : "0", 1);
        const auto& ext = table.lods.front().extension;
        const auto& base = table.partitions.front().baseMesh;
        if (base.empty() || !ext.count(pmsf::kPomarCluster0PayloadName) || !ext.count(pmsf::kPomarMetaPayloadName) || !ext.count(pmsf::kPomarRestPayloadName)) {
            throw std::runtime_error("POMAR paper reader: missing named native stream");
        }
        std::vector<const std::vector<std::uint8_t>*> parts{&base, &ext.at(pmsf::kPomarCluster0PayloadName),
            &ext.at(pmsf::kPomarMetaPayloadName), &ext.at(pmsf::kPomarRestPayloadName)};
        assembledSize_ = 0;
        for (const auto* part : parts) assembledSize_ += part->size();
        assembledBuffer_ = new char[assembledSize_ + 32]();
        std::size_t offset = 0;
        for (const auto* part : parts) {
            std::memcpy(assembledBuffer_ + offset, part->data(), part->size());
            offset += part->size();
        }
        inner_->init(assembledBuffer_);
        buffer_ = assembledBuffer_;
        dataOffset_ = 0;
        totalSize = assembledSize_;
        return;
    }

    loadPMSFSymbolSidecarIfPresent(path);
    const std::string v3Stem = v3StemFromStem(stem);
    std::string payloadStem = stem;
    auto namedBlobCandidate =
        pmsf::readFirstPaperNamedBlobTableCandidate(payloadStem);
    if (namedBlobCandidate.blobs.empty() && payloadStem != v3Stem) {
        payloadStem = v3Stem;
        namedBlobCandidate =
            pmsf::readFirstPaperNamedBlobTableCandidate(payloadStem);
    }
    if (!namedBlobCandidate.blobs.empty()) {
        auto payloadBytes = [&](const std::string& payloadName) -> std::string {
            const auto* bytes = findPMSFSymbolPayloadBytes(0, 0, 0, payloadName);
            if (!bytes) {
                bytes = findPMSFSymbolPayloadBytesByRole(
                    pmsf::inferPMSFPayloadRole(payloadName), 0, 0, 0);
            }
            if (!bytes) return {};
            return std::string(bytes->begin(), bytes->end());
        };
        auto payloadParts = pmsf::readPomarPayloadStringParts(payloadBytes);

        if (!pmsf::pomarPayloadStringPartsHaveRequired(payloadParts)) {
            const auto& namedBlobs = namedBlobCandidate.blobs;
            auto namedPayloadBytes = [&](const std::string& payloadName) {
                auto value = pmsf::findPMSFPayloadOrRawNamedBlobString(
                    namedBlobs, 0, 0, 0, payloadName);
                if (value.empty()) {
                    value = pmsf::findPMSFPayloadOrRawNamedBlobStringByRole(
                        namedBlobs, pmsf::inferPMSFPayloadRole(payloadName));
                }
                return value;
            };
            payloadParts = pmsf::readPomarPayloadStringParts(namedPayloadBytes);
        }
        if (pmsf::pomarPayloadStringPartsHaveRequired(payloadParts)) {
            const std::string metaPath = pmsf::paperMetadataTablePath(payloadStem);
            if (fileExists(metaPath)) {
                auto metaTable = readTable(metaPath);
                int blobLayout = int32Cell(metaTable, pmsf::kPayloadWrapperBlobLayoutColumn);
                setenv("PMSF_POMAR_DIRECT_RESIDUAL",
                       pmsf::pomarCompatBlobLayoutUsesDirectResidual(blobLayout)
                           ? "1"
                           : "0",
                       1);
            }

            const std::size_t totalLen =
                pmsf::pomarPayloadStringPartsSize(payloadParts) + 32;
            assembledBuffer_ = new char[totalLen];
            std::memset(assembledBuffer_, 0, totalLen);
            std::size_t off = 0;
            auto appendPart = [&](const std::string& part) {
                std::memcpy(assembledBuffer_ + off, part.data(), part.size());
                off += part.size();
            };
            appendPart(payloadParts.baseMesh);
            appendPart(payloadParts.cluster0);
            appendPart(payloadParts.meta);
            appendPart(payloadParts.rest);
            assembledSize_ = off;

            std::cout << "[POMAR-PV2] reassembled unified paper payload: "
                      << assembledSize_ << " bytes" << std::endl;

            inner_->init(assembledBuffer_);
            buffer_ = assembledBuffer_;
            dataOffset_ = 0;
            totalSize = assembledSize_;
            propagatePMSFSymbolContextTo(inner_);
            return;
        }
    }

    // ---- R8：唯一规范形态 `<stem>_compact.bin`（通用段目录）优先 ----
    // 段按目录顺序拼接即得与旧路径**逐字节相同**的内层缓冲，故下游无需任何改动。
    {
        const std::string binPath = stem + pmsf::kPaperCompactBinSuffix;
        pmsf::PMSFCompactBinEnvelope env;
        if (fileExists(binPath) &&
            pmsf::readPMSFCompactBinEnvelopeFile(binPath, "[POMAR-PV2]", env)) {
            std::vector<std::string> names;
            std::vector<std::int32_t> codecIds;
            std::vector<std::uint32_t> offsets, lengths;
            if (pmsf::readCompactSegmentDirectory(env.payload, names, codecIds,
                                                  offsets, lengths)) {
                std::size_t total = 0;
                for (auto len : lengths) total += len;

                pmsf::PaperCompactPayloadHeader hdr = [&] {
                    const std::uint8_t* q = env.payload.data() + 4;
                    return pmsf::readPaperCompactPayloadHeader(q, true);
                }();
                origTotalVertices_ = hdr.origTotalVertices;
                origTotalFaces_ = hdr.origTotalFaces;
                setenv("PMSF_POMAR_DIRECT_RESIDUAL",
                       pmsf::pomarCompatBlobLayoutUsesDirectResidual(hdr.blobLayout) ? "1"
                                                                                    : "0",
                       1);

                assembledBuffer_ = new char[total + 32];
                std::memset(assembledBuffer_, 0, total + 32);
                std::size_t off = 0;
                for (std::size_t i = 0; i < lengths.size(); ++i) {
                    std::memcpy(assembledBuffer_ + off,
                                env.payload.data() + offsets[i], lengths[i]);
                    off += lengths[i];
                }
                assembledSize_ = off;

                std::cout << "[POMAR-PV2] reassembled compact segment container: "
                          << assembledSize_ << " bytes / " << lengths.size()
                          << " segments" << std::endl;

                inner_->init(assembledBuffer_);
                buffer_ = assembledBuffer_;
                dataOffset_ = 0;
                totalSize = assembledSize_;
                propagatePMSFSymbolContextTo(inner_);
                return;
            }
        }
    }

    const std::string compactPath = pmsf::pomarCompatMainTablePath(stem);
    const bool compact = fileExists(compactPath);
    auto mTable = compact ? readTable(compactPath)
                          : readTable(pmsf::paperMetadataTablePath(stem));
    std::shared_ptr<arrow::Table> bmTable;
    std::shared_ptr<arrow::Table> c0Table;
    std::shared_ptr<arrow::Table> csTable;
    if (!compact) {
        bmTable = readTable(pmsf::paperBaseMeshTablePath(stem));
        c0Table = readTable(pmsf::pomarCluster0TablePath(stem));
        csTable = readTable(pmsf::pomarClustersTablePath(stem));
    }

    if (compact && mTable->GetColumnByName(pmsf::kPayloadWrapperPayloadColumn)) {
        int blobLayout = int32Cell(mTable, pmsf::kPayloadWrapperBlobLayoutColumn);
        setenv("PMSF_POMAR_DIRECT_RESIDUAL",
               pmsf::pomarCompatBlobLayoutUsesDirectResidual(blobLayout)
                   ? "1"
                   : "0",
               1);

        std::string payload = binaryCell(mTable, pmsf::kPayloadWrapperPayloadColumn);
        const int64_t expectedPayloadBytes = int64Cell(mTable, pmsf::kPayloadWrapperPayloadBytesColumn);
        if (expectedPayloadBytes > 0 &&
            expectedPayloadBytes != static_cast<int64_t>(payload.size())) {
            std::cerr << "[POMAR-PV2] warn: compact payload size mismatch "
                      << payload.size() << "/" << expectedPayloadBytes << std::endl;
        }

        assembledBuffer_ = new char[payload.size() + 32];
        std::memset(assembledBuffer_, 0, payload.size() + 32);
        std::memcpy(assembledBuffer_, payload.data(), payload.size());
        assembledSize_ = payload.size();

        std::cout << "[POMAR-PV2] reassembled compact inner payload: "
                  << assembledSize_ << " bytes" << std::endl;

        inner_->init(assembledBuffer_);
        buffer_ = assembledBuffer_;
        dataOffset_ = 0;
        totalSize = assembledSize_;
        return;
    }

    int numRounds      = int32Cell(mTable, pmsf::kPomarNumRoundsColumn);
    int baseMeshSize   = int32Cell(mTable, pmsf::kPomarBaseMeshSizeColumn);
    int cluster0Size   = int32Cell(mTable, pmsf::kPomarCluster0SizeColumn);
    int clusterMetaSz  = int32Cell(mTable, pmsf::kPomarClusterMetaSizeColumn);
    int clustersRestSz = int32Cell(mTable, pmsf::kPomarClustersRestSizeColumn);
    int blobLayout     = int32Cell(mTable, pmsf::kPayloadWrapperBlobLayoutColumn);

    // Layout 11 keeps the same range-coded container but stores geometry as
    // direct xyz residuals instead of Frenet-frame residuals. Force the inner
    // decoder to follow the file layout so old layout-10 files stay readable.
    setenv("PMSF_POMAR_DIRECT_RESIDUAL",
           pmsf::pomarCompatBlobLayoutUsesDirectResidual(blobLayout)
               ? "1"
               : "0",
           1);

    bool eC = boolCell(mTable, pmsf::kPomarEnableCompressColumn);
    bool eQ = boolCell(mTable, pmsf::kPomarEnableQuantizationColumn);
    bool eS = boolCell(mTable, pmsf::kPomarEnableSegmentationColumn);
    bool eA = boolCell(mTable, pmsf::kPomarEnableAdjustmentColumn);
    bool eP = boolCell(mTable, pmsf::kPomarEnablePredictionColumn);

    header_ = PMSFHeader::Builder()
                  .setRound(numRounds)
                  .setEnableCompress(eC)
                  .setEnableQuantization(eQ)
                  .setEnableSegmentation(eS)
                  .setEnableAdjustMent(eA)
                  .setEnablePrediction(eP)
                  .build();

    std::string baseBlob = compact ? binaryCell(mTable, pmsf::kPomarBaseMeshBlobColumn)
                                   : binaryCell(bmTable, pmsf::kPomarBaseMeshBlobColumn);
    std::string c0Blob   = compact ? binaryCell(mTable, pmsf::kPomarCluster0RCBlobColumn)
                                   : binaryCell(c0Table, pmsf::kPomarCluster0RCBlobColumn);

    if ((int)baseBlob.size() != baseMeshSize || (int)c0Blob.size() != cluster0Size) {
        std::cerr << "[POMAR-PV2] warn: size mismatch base=" << baseBlob.size()
                  << "/" << baseMeshSize << " c0=" << c0Blob.size()
                  << "/" << cluster0Size << std::endl;
    }

    std::string metaBlob;
    std::string restBlob;
    if (compact) {
        metaBlob = binaryCell(mTable, pmsf::kPomarClusterMetaBlobColumn);
        restBlob = binaryCell(mTable, pmsf::kPomarClustersBlobColumn);
    } else if (csTable->GetColumnByName(pmsf::kPomarClustersBlobColumn)) {
        metaBlob = binaryCell(csTable, pmsf::kPomarClusterMetaBlobColumn);
        restBlob = binaryCell(csTable, pmsf::kPomarClustersBlobColumn);
    } else {
        // 兼容旧格式：row 0 (id=-1) = meta，其余 id=1..N-1 = 各 cluster RC blob。
        auto idCol = csTable->GetColumnByName(pmsf::kPomarClusterIdColumn);
        auto bbCol = csTable->GetColumnByName(pmsf::kPomarCompatRCBlobColumn);
        auto idArr = std::dynamic_pointer_cast<arrow::Int32Array>(idCol->chunk(0));
        auto bbArr = std::dynamic_pointer_cast<arrow::BinaryArray>(bbCol->chunk(0));

        std::vector<std::pair<int, std::string>> clusterBlobs;
        clusterBlobs.reserve(idArr->length());
        for (int64_t r = 0; r < idArr->length(); ++r) {
            int id = idArr->Value(r);
            std::string blob(bbArr->GetString(r));
            if (id == -1) {
                metaBlob = std::move(blob);
            } else if (id >= 1) {
                clusterBlobs.emplace_back(id, std::move(blob));
            }
        }
        std::sort(clusterBlobs.begin(), clusterBlobs.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });
        for (auto& p : clusterBlobs) {
            restBlob += p.second;
        }
    }

    if ((int)metaBlob.size() != clusterMetaSz) {
        std::cerr << "[POMAR-PV2] warn: meta size mismatch " << metaBlob.size()
                  << "/" << clusterMetaSz << std::endl;
    }
    if ((int)restBlob.size() != clustersRestSz) {
        std::cerr << "[POMAR-PV2] warn: rest size mismatch " << restBlob.size()
                  << "/" << clustersRestSz << std::endl;
    }

    // 计算 assembled 总长：base + cluster0 + clusterMeta + rest
    std::size_t totalLen = static_cast<std::size_t>(baseMeshSize +
                                                    cluster0Size +
                                                    clusterMetaSz +
                                                    clustersRestSz) + 32;  // 余量
    assembledBuffer_ = new char[totalLen];
    std::memset(assembledBuffer_, 0, totalLen);

    std::size_t off = 0;
    std::memcpy(assembledBuffer_ + off, baseBlob.data(), baseBlob.size());
    off += baseBlob.size();
    std::memcpy(assembledBuffer_ + off, c0Blob.data(), c0Blob.size());
    off += c0Blob.size();
    std::memcpy(assembledBuffer_ + off, metaBlob.data(), metaBlob.size());
    off += metaBlob.size();
    std::memcpy(assembledBuffer_ + off, restBlob.data(), restBlob.size());
    off += restBlob.size();
    assembledSize_ = off;

    std::cout << "[POMAR-PV2] reassembled inner buffer: " << assembledSize_
              << " bytes (base=" << baseMeshSize
              << " c0=" << cluster0Size
              << " meta=" << clusterMetaSz
              << " rest=" << clustersRestSz
              << ")" << std::endl;

    // 把 assembled buffer 喂给 inner
    inner_->init(assembledBuffer_);
    // 同步基类指针 / 状态（DecompressPipeline 会访问 header_ 等）
    buffer_ = assembledBuffer_;
    dataOffset_ = 0;
    totalSize = assembledSize_;
    loadPMSFSymbolSidecarIfPresent(path);
    propagatePMSFSymbolContextTo(inner_);
}

void POMARColumnarDeserializeOperator::deserializeHeader(PMSFHeader& header) {
    inner_->deserializeHeader(header);
    header_ = header;
    dataOffset_ = inner_->dataOffset_;
}

std::shared_ptr<MCGAL::Mesh>
POMARColumnarDeserializeOperator::deserializeBaseMesh() {
    auto mesh = inner_->deserializeBaseMesh();
    mesh_ = mesh;
    dataOffset_ = inner_->dataOffset_;
    return mesh;
}

void POMARColumnarDeserializeOperator::deserializeOneRound() {
    inner_->deserializeOneRound();
    dataOffset_ = inner_->dataOffset_;
}

void POMARColumnarDeserializeOperator::setSeeds(std::vector<MCGAL::Halfedge*> seeds) {
    inner_->setSeeds(seeds);
    seeds_ = seeds;
}

// region 谓词透传给内层算子（它才是真正按 group 选种子的那层）。
void POMARColumnarDeserializeOperator::setQueryRegion(int regionId) {
    if (inner_) inner_->setQueryRegion(regionId);
}

std::shared_ptr<MCGAL::Mesh>
POMARColumnarDeserializeOperator::exportMesh() {
    return inner_->exportMesh();
}
