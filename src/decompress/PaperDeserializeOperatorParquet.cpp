#include "PaperDeserializeOperator.h"
#include "storage/PaperTypedLodDirectory.h"

#include "ColumnarDeserializeOperator.h"
#include "storage/PMSFSymbolStorageAdapter.h"
#include "storage/PaperBridgeV2.h"
#include "storage/PaperStreamReader.h"
#include "storage/PaperStreamSymbolAdapter.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <system_error>
#include <utility>

PaperDeserializeOperator::PaperDeserializeOperator(
    std::shared_ptr<SymbolReadOperator> symbolReadOperator,
    std::shared_ptr<ReconstructOperator> reconstructOperator)
    : DeserializeOperator(std::move(symbolReadOperator), std::move(reconstructOperator)),
      inner_(std::make_unique<ColumnarDeserializeOperator>(
          DeserializeOperator::symbolReadOperator_,
          DeserializeOperator::reconstructOperator_)) {}

PaperDeserializeOperator::~PaperDeserializeOperator() {
    if (!tempDir_.empty()) {
        std::error_code ec;
        std::filesystem::remove_all(tempDir_, ec);
    }
}

void PaperDeserializeOperator::init(std::string path) {
    const std::string baseName = pmsf::paperStorageStripExtension(path);
    const std::string suffix = "_paper.parquet";
    const bool exactArtifact = path.size() >= suffix.size() &&
        path.compare(path.size() - suffix.size(), suffix.size(), suffix) == 0;
    const std::string paperPath = exactArtifact ? path : baseName + suffix;

    // M4-3: dictionary-version dispatch. v2 artifacts go through the v2 reader;
    // everything else keeps the original v1 bridge path below, unchanged.
    pmsf::PaperStreamReaderV2 probe(paperPath);
    if (!probe.usedV1Fallback()) {
        initV2(probe);
        return;
    }

    pmsf::PaperStreamReader reader(paperPath);
    const pmsf::PaperStreamTable table = reader.readAll();
    if (table.partitions.empty()) {
        throw std::runtime_error("PaperDeserializeOperator: no partition in " + paperPath);
    }
    {
        const char* nativeEnv = std::getenv("PMSF_PAPER_NATIVE");
        const bool nativeRequested = nativeEnv && *nativeEnv && std::strcmp(nativeEnv, "0") != 0;
        if (nativeRequested && table.object.algorithm != "ppmc") {
            throw std::runtime_error(
                "[M2c] PMSF_PAPER_NATIVE is currently implemented only for PPMC; "
                "refusing to decode algorithm '" + table.object.algorithm +
                "' without the legacy bridge.");
        }
        if (table.object.algorithm == "ppmc" && !(nativeEnv && std::strcmp(nativeEnv, "0") == 0)) {
            // PPMC paper artifacts are bridge-free by default; make the whole
            // Columnar/PPMC stack use the native path for this process.
            setenv("PMSF_PAPER_NATIVE", "1", 0);
        }
    }
    const std::vector<std::uint8_t>& compactBytes = table.partitions[0].baseMesh;
    if (compactBytes.empty()) {
        throw std::runtime_error("PaperDeserializeOperator: empty base mesh BLOB");
    }

    pmsf::PMSFSymbolTable symbols;
    std::string error;
    if (!pmsf::paper_adapter::toPMSFSymbolTable(table, symbols, &error)) {
        throw std::runtime_error("PaperDeserializeOperator: symbol decode failed: " + error);
    }

    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    tempDir_ = (std::filesystem::temp_directory_path() /
                ("pmsf_paper_read_" + std::to_string(static_cast<long long>(stamp))))
                   .string();
    std::error_code ec;
    std::filesystem::create_directories(tempDir_, ec);
    if (ec) {
        throw std::runtime_error("PaperDeserializeOperator: cannot create temp dir");
    }
    const std::string legacyBase = tempDir_ + "/legacy";
    {
        std::ofstream ofs(pmsf::paperCompactBinPath(legacyBase), std::ios::binary);
        if (!ofs) {
            throw std::runtime_error("PaperDeserializeOperator: cannot write temp compact bin");
        }
        ofs.write(reinterpret_cast<const char*>(compactBytes.data()),
                  static_cast<std::streamsize>(compactBytes.size()));
    }
    // M2b pilot bridge: restore the legacy symbol tables next to the compact bin.
    // PMSF_PAPER_NATIVE=1 skips this and tries the pure symbol-table path (M2c).
    const bool nativeSymbolPath = [&]() {
        const char* env = std::getenv("PMSF_PAPER_NATIVE");
        if (env && *env && std::strcmp(env, "0") != 0) return true;
        // M2c PPMC is produced bridge-free by default; decode it natively.
        return table.object.algorithm == "ppmc";
    }();
    if (!nativeSymbolPath) {
        for (const auto& [suffix, bytes] : table.legacyFiles) {
            std::ofstream ofs(tempDir_ + "/legacy" + suffix, std::ios::binary);
            if (!ofs) {
                throw std::runtime_error("PaperDeserializeOperator: cannot write temp legacy file " + suffix);
            }
            ofs.write(reinterpret_cast<const char*>(bytes.data()),
                      static_cast<std::streamsize>(bytes.size()));
        }
    }

    if (nativeSymbolPath) {
        // M2c: set before init so readSymbolsTable() is skipped; set again
        // after init so the reader created during init receives the context.
        pmsf::PMSFSymbolTable initSymbols = symbols;
        inner_->setPMSFSymbolTable(std::move(initSymbols));
        inner_->init(legacyBase + ".parquet");
        inner_->setPMSFSymbolTable(std::move(symbols));
        if (auto* columnar = dynamic_cast<ColumnarDeserializeOperator*>(inner_.get())) {
            columnar->bindPPMCReaderMesh();
        }
    } else {
        inner_->setPMSFSymbolTable(std::move(symbols));
        inner_->init(legacyBase + ".parquet");
    }
    header_ = inner_->header();
}

void PaperDeserializeOperator::deserializeHeader(PMSFHeader& header) {
    // Paper init has already parsed the dictionary/base header. A typed base-only
    // prefix legitimately has zero rounds; the generic pipeline's binary-header
    // fallback must not dereference this operator's nonexistent binary buffer.
    header = this->header();
}

void PaperDeserializeOperator::initV2(pmsf::PaperStreamReaderV2& reader) {
    if (reader.usedV1Fallback()) {
        throw std::runtime_error("PaperDeserializeOperator: v2 dispatch received a v1 reader");
    }
    const int maxLod = queryMaxLod_;
    pmsf::PaperStreamTable table =
        maxLod >= 0 ? reader.readPrefix(0, maxLod) : reader.readAll();
    if (table.partitions.empty()) {
        throw std::runtime_error("PaperDeserializeOperator: v2 artifact has no partition");
    }
    if (table.partitions.size()==1) {
        const int available=table.partitions.front().numLods;
        const auto expected=static_cast<std::size_t>(maxLod<0?available:std::min(maxLod,available));
        if (available<0 || table.lods.size()!=expected)
            throw std::runtime_error("PaperDeserializeOperator: v2 requested LOD rows are missing");
    }
    if (table.object.payloadMode==pmsf::PaperPayloadMode::Typed && table.object.raPrefixUnit=="lod_block") {
        pmsf::requirePaperTypedLodDirectory(table);
    }
    std::cout << "PAPER_V2_READ granularity=" << reader.granularity()
              << " row_groups=" << reader.rowGroupsRead() << " total_row_groups=" << reader.numRowGroups()
              << " accounted_bytes=" << reader.bytesRead() << " footer_bytes=" << reader.footerBytes()
              << " lods=" << table.lods.size() << "\n";
    usedV2Path_ = true;
    v2LodCount_ = static_cast<std::int32_t>(table.lods.size());
    v2Table_ = std::move(table);
    v2OrigV_ = v2Table_.object.totalVertices;
    v2OrigF_ = v2Table_.object.totalFaces;

    header_.setRound(v2Table_.object.numRounds > 0
                         ? v2Table_.object.numRounds
                         : static_cast<int>(v2Table_.lods.size()));
    header_.setGroupNumber(0);
    header_.setEnableCompress(true);
    header_.setEnableQuantization(v2Table_.object.enableQuantization);
    header_.setEnableSegmentation(false);
    header_.setEnableAdjustment(false);

    // Keep the partition base BLOB in a temp compact bin, byte-identical to the
    // v1 bridge layout, so the M4-4 native round driver can reuse the existing
    // compact-base decoder without any legacy side files.
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    tempDir_ = (std::filesystem::temp_directory_path() /
                ("pmsf_paper_v2_read_" + std::to_string(static_cast<long long>(stamp))))
                   .string();
    std::error_code ec;
    std::filesystem::create_directories(tempDir_, ec);
    if (ec) {
        throw std::runtime_error("PaperDeserializeOperator: cannot create temp dir");
    }
    const std::string legacyBase = tempDir_ + "/legacy";
    {
        const auto& basePayload = v2Table_.partitions.front().baseMesh;
        std::ofstream ofs(pmsf::paperCompactBinPath(legacyBase), std::ios::binary);
        if (!ofs) {
            throw std::runtime_error(
                "PaperDeserializeOperator: cannot write temp v2 compact bin");
        }
        ofs.write(reinterpret_cast<const char*>(basePayload.data()),
                  static_cast<std::streamsize>(basePayload.size()));
    }

    const std::string compactPath = pmsf::paperCompactBinPath(legacyBase);

    v2Native_ = v2Table_.object.payloadMode == pmsf::PaperPayloadMode::Native;
    if (!v2Native_) {
        // M4-4 typed v2: hand the pre-decoded PMSFSymbolTable to the typed
        // per-round driver (PPMC today). The compact base payload above is the
        // single source of truth for header/base mesh/layout.
        pmsf::PMSFSymbolTable symbols;
        std::string error;
        if (!pmsf::paper_adapter::toPMSFSymbolTable(v2Table_, symbols, &error)) {
            throw std::runtime_error(
                "PaperDeserializeOperator: v2 symbol decode failed: " + error);
        }
        std::string typedError;
        if (!inner_->initPaperV2Typed(compactPath, std::move(symbols), &typedError)) {
            throw std::runtime_error(
                "PaperDeserializeOperator: typed v2 decode seeding failed: " + typedError);
        }
        v2Typed_ = true;
        // The compact base payload carries the algorithm header (prediction /
        // curvature / connectivity switch bits); the typed table directory owns
        // the round count. Merge both so the PPMC reader is configured exactly
        // like the canonical decode path.
        header_ = inner_->header();
        header_.setRound(static_cast<int>(v2Table_.lods.size()));
        return;
    }

    // M4-4 native decode: rebuild the algorithm-native symbol blob from the
    // PaperStream v2 payload directory (CONN header + per-decode-LOD slices) or
    // from the partition-level whole-frame payload, then seed the native decoder
    // with the compact base payload. No legacy side tables are read.
    std::vector<char> nativeBlob;
    const auto& partition = v2Table_.partitions.front();
    const auto frame = v2Table_.object.nativeFrame;
    if (frame == pmsf::PaperNativeFrame::ConnRound ||
        frame == pmsf::PaperNativeFrame::ClassIStream) {
        nativeBlob.assign(partition.connHeader.begin(), partition.connHeader.end());
        std::vector<const pmsf::PaperLodRow*> lods;
        lods.reserve(v2Table_.lods.size());
        for (const auto& lod : v2Table_.lods) lods.push_back(&lod);
        std::sort(lods.begin(), lods.end(),
                  [](const pmsf::PaperLodRow* a, const pmsf::PaperLodRow* b) {
                      return a->lodId < b->lodId;
                  });
        for (const auto* lod : lods) {
            nativeBlob.insert(nativeBlob.end(), lod->nativePayload.begin(),
                              lod->nativePayload.end());
        }
    } else {
        nativeBlob.assign(partition.nativePayload.begin(),
                          partition.nativePayload.end());
    }
    if (nativeBlob.empty()) {
        throw std::runtime_error("PaperDeserializeOperator: v2 native payload is empty");
    }

    std::string nativeError;
    if (!inner_->initPaperV2Native(compactPath, std::move(nativeBlob), &nativeError)) {
        throw std::runtime_error(
            "PaperDeserializeOperator: native v2 decode seeding failed: " + nativeError);
    }
    header_ = inner_->header();
}

std::shared_ptr<MCGAL::Mesh> PaperDeserializeOperator::deserializeBaseMesh() {
    if (usedV2Path_) {
        if (!v2Native_ && !v2Typed_) {
            throw std::runtime_error(
                "PaperDeserializeOperator: v2 decode mode is neither native nor typed");
        }
        mesh_ = inner_->deserializeBaseMesh();
        if (!mesh_) {
            throw std::runtime_error(
                "PaperDeserializeOperator: v2 base-mesh decode failed");
        }
        if (auto* columnar = dynamic_cast<ColumnarDeserializeOperator*>(inner_.get())) {
            columnar->bindPPMCReaderMesh();
        }
        return mesh_;
    }
    mesh_ = inner_->deserializeBaseMesh();
    if (auto* columnar = dynamic_cast<ColumnarDeserializeOperator*>(inner_.get())) {
        columnar->bindPPMCReaderMesh();
    }
    return mesh_;
}

void PaperDeserializeOperator::deserializeOneRound() {
    if (usedV2Path_) {
        if (!v2Native_ && !v2Typed_) {
            throw std::runtime_error(
                "PaperDeserializeOperator: v2 decode mode is neither native nor typed");
        }
        inner_->deserializeOneRound();
        mesh_ = inner_->exportMesh();
        return;
    }
    inner_->deserializeOneRound();
    mesh_ = inner_->exportMesh();
}

std::shared_ptr<MCGAL::Mesh> PaperDeserializeOperator::exportMesh() {
    if (usedV2Path_) return (v2Native_ || v2Typed_) ? inner_->exportMesh() : mesh_;
    return inner_->exportMesh();
}

PMSFHeader PaperDeserializeOperator::header() {
    if (usedV2Path_) return v2Native_ ? inner_->header() : header_;
    return inner_->header();
}

void PaperDeserializeOperator::setSeeds(std::vector<MCGAL::Halfedge*> seeds) {
    DeserializeOperator::setSeeds(std::move(seeds));
    inner_->setSeeds(seeds_);
}

void PaperDeserializeOperator::setQueryRegion(int regionId) {
    inner_->setQueryRegion(regionId);
}

void PaperDeserializeOperator::setQueryMaxLod(int maxLod) {
    queryMaxLod_ = maxLod;
    inner_->setQueryMaxLod(maxLod);
}

int PaperDeserializeOperator::queryMaxLod() const {
    if (usedV2Path_) return queryMaxLod_;
    return inner_->queryMaxLod();
}

bool PaperDeserializeOperator::supportsRandomAccess() const {
    return inner_->supportsRandomAccess();
}

void PaperDeserializeOperator::originalMeshCounts(int& v, int& f) const {
    if (usedV2Path_) {
        if (v2Native_) {
            inner_->originalMeshCounts(v, f);
            return;
        }
        v = v2OrigV_;
        f = v2OrigF_;
        return;
    }
    inner_->originalMeshCounts(v, f);
}

void PaperDeserializeOperator::clearPMSFSymbolContext() {
    DeserializeOperator::clearPMSFSymbolContext();
    inner_->clearPMSFSymbolContext();
}
