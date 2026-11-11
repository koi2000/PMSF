#include "PaperSerializeOperator.h"

#include "storage/PMSFSymbolStorageAdapter.h"
#include "storage/PaperBridgeV2.h"
#include "storage/PaperStreamCompactCodec.h"
#include "storage/PaperStreamSymbolAdapter.h"
#include "compress/PMSFCollectorLayoutRegistry.h"
#include "storage/PaperStreamWriter.h"

#include "compress/ConnectivityBlobContainer.h"

#include <zlib.h>
#ifdef PMSF_HAS_ZSTD
#include <zstd.h>
#endif

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace {

std::vector<std::uint8_t> readFileBytes(const std::filesystem::path& path) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) return {};
    ifs.seekg(0, std::ios::end);
    const std::streamoff size = ifs.tellg();
    if (size <= 0) return {};
    ifs.seekg(0, std::ios::beg);
    std::vector<std::uint8_t> bytes(static_cast<std::size_t>(size));
    ifs.read(reinterpret_cast<char*>(bytes.data()), size);
    return bytes;
}

/// @brief Capture the authoritative algorithm-native blob (the same bytes the
///        canonical columnar writer entropy-codes into rc_blob) straight from
///        the collector. The codec-4 residual/bit/lod tables are only segment
///        splits of this blob, so reassembling them would be redundant.
std::vector<std::uint8_t> captureNativeBlob(SymbolCollectOperator* collector,
                                            bool enableQuantization,
                                            bool enablePrediction,
                                            bool enableCompress,
                                            const std::vector<std::uint8_t>& fallback) {
    if (collector != nullptr) {
        std::unique_ptr<char[]> scratch(new char[BUFFER_SIZE]);
        const auto exported = pmsf::paperExportCollectorPayloadBuffer(
            collector, scratch.get(), enableQuantization, enablePrediction,
            enableCompress);
        if (exported.handled && exported.usedBytes > 0) {
            return std::vector<std::uint8_t>(
                reinterpret_cast<const std::uint8_t*>(scratch.get()),
                reinterpret_cast<const std::uint8_t*>(scratch.get()) +
                    exported.usedBytes);
        }
    }
    return fallback;
}

std::uint32_t readU32LEAt(const std::vector<std::uint8_t>& data, std::size_t off) {
    return static_cast<std::uint32_t>(data[off]) |
           (static_cast<std::uint32_t>(data[off + 1]) << 8) |
           (static_cast<std::uint32_t>(data[off + 2]) << 16) |
           (static_cast<std::uint32_t>(data[off + 3]) << 24);
}

/// @brief Strip the optional SBC1 zlib envelope (M2b symbol-blob compression).
///        When unwrapping is not possible the bytes are returned unchanged; the
///        v2 decoder tolerates a wrapped blob as whole-frame payload.
std::vector<std::uint8_t> unwrapSymbolBlobForNative(std::vector<std::uint8_t> bytes) {
    if (bytes.size() < 13 || bytes[0] != 'S' || bytes[1] != 'B' || bytes[2] != 'C' ||
        bytes[3] != '1') {
        return bytes;
    }
    const std::uint8_t codec = bytes[4];
    const std::uint32_t rawLen = readU32LEAt(bytes, 5);
    const std::uint32_t compLen = readU32LEAt(bytes, 9);
    if (codec != 1 || rawLen == 0 || compLen == 0 ||
        13ull + static_cast<unsigned long long>(compLen) != bytes.size()) {
        return bytes;
    }
    std::vector<std::uint8_t> raw(rawLen);
    uLongf outLen = static_cast<uLongf>(rawLen);
    const int zret = uncompress(reinterpret_cast<Bytef*>(raw.data()), &outLen,
                                reinterpret_cast<const Bytef*>(bytes.data() + 13),
                                static_cast<uLong>(compLen));
    if (zret != Z_OK || outLen != static_cast<uLongf>(rawLen)) {
        return bytes;  // keep wrapped; whole-frame decode path handles it
    }
    return raw;
}

bool isSbc1Blob(const std::vector<std::uint8_t>& bytes) {
    return bytes.size() >= 13 && bytes[0] == 'S' && bytes[1] == 'B' &&
           bytes[2] == 'C' && bytes[3] == '1';
}

/// @brief Wrap a raw symbol blob with the canonical SBC1 zlib envelope
///        (codec byte 1), mimicking ColumnarSerializeOperator's wrapper. Only
///        used when the canonical wrapped bytes are not directly available; if
///        wrapping would not shrink the blob the raw bytes are kept.
std::vector<std::uint8_t> wrapSbc1Zlib(const std::vector<std::uint8_t>& raw,
                                       bool& wrapped) {
    wrapped = false;
    if (raw.size() <= 64) return raw;
    uLongf cap = compressBound(static_cast<uLong>(raw.size()));
    std::vector<std::uint8_t> compressed(cap);
    uLongf compressedLen = cap;
    if (compress2(compressed.data(), &compressedLen, raw.data(),
                  static_cast<uLong>(raw.size()), Z_BEST_COMPRESSION) != Z_OK) {
        return raw;
    }
    compressed.resize(static_cast<std::size_t>(compressedLen));
    if (compressed.size() + 13u >= raw.size()) return raw;
    std::vector<std::uint8_t> out;
    out.reserve(13u + compressed.size());
    out.insert(out.end(), {'S', 'B', 'C', '1', 1});
    const std::uint32_t rawLen = static_cast<std::uint32_t>(raw.size());
    const std::uint32_t compLen = static_cast<std::uint32_t>(compressed.size());
    out.push_back(static_cast<std::uint8_t>(rawLen & 0xffu));
    out.push_back(static_cast<std::uint8_t>((rawLen >> 8) & 0xffu));
    out.push_back(static_cast<std::uint8_t>((rawLen >> 16) & 0xffu));
    out.push_back(static_cast<std::uint8_t>((rawLen >> 24) & 0xffu));
    out.push_back(static_cast<std::uint8_t>(compLen & 0xffu));
    out.push_back(static_cast<std::uint8_t>((compLen >> 8) & 0xffu));
    out.push_back(static_cast<std::uint8_t>((compLen >> 16) & 0xffu));
    out.push_back(static_cast<std::uint8_t>((compLen >> 24) & 0xffu));
    out.insert(out.end(), compressed.begin(), compressed.end());
    wrapped = true;
    return out;
}

/// @brief Wrap a raw symbol blob with the SBC1 zstd envelope (codec byte 2).
std::vector<std::uint8_t> wrapSbc1Zstd(const std::vector<std::uint8_t>& raw,
                                       bool& wrapped) {
    wrapped = false;
#ifdef PMSF_HAS_ZSTD
    if (raw.size() <= 64) return raw;
    std::vector<std::uint8_t> compressed(ZSTD_compressBound(raw.size()));
    const std::size_t compressedLen =
        ZSTD_compress(compressed.data(), compressed.size(), raw.data(), raw.size(), 22);
    if (ZSTD_isError(compressedLen) || compressedLen == 0 ||
        compressedLen + 13u >= raw.size()) {
        return raw;
    }
    compressed.resize(compressedLen);
    std::vector<std::uint8_t> out;
    out.reserve(13u + compressed.size());
    out.insert(out.end(), {'S', 'B', 'C', '1', 2});
    const std::uint32_t rawLen = static_cast<std::uint32_t>(raw.size());
    const std::uint32_t compLen = static_cast<std::uint32_t>(compressed.size());
    for (int i = 0; i < 4; ++i)
        out.push_back(static_cast<std::uint8_t>((rawLen >> (8 * i)) & 0xffu));
    for (int i = 0; i < 4; ++i)
        out.push_back(static_cast<std::uint8_t>((compLen >> (8 * i)) & 0xffu));
    out.insert(out.end(), compressed.begin(), compressed.end());
    wrapped = true;
    return out;
#else
    (void)raw;
    return raw;
#endif
}

/// @brief Build a v2 NATIVE payload table from the algorithm-native blob.
///        CONN blobs are split into per-decode-LOD slices (header kept at the
///        partition row) so prefix/RA can stop after the first k LODs; other
///        layouts are stored whole at the partition row.
bool buildNativeV2Table(pmsf::PaperStreamTable& out, std::vector<std::uint8_t> residualBytes,
                        int codecVersion, int blobLayout,
                        bool keepWrappedPayload = false) {
    if (residualBytes.empty() || out.partitions.empty()) return false;
    // M5-2 (S1): keep the canonical SBC1 bytes verbatim. The decoder's
    // initPaperV2Native() already unwraps codec 1/2; only raw blobs are eligible
    // for CONN per-LOD slicing.
    const bool wrappedInput = keepWrappedPayload && isSbc1Blob(residualBytes);
    std::vector<std::uint8_t> raw =
        wrappedInput ? std::move(residualBytes)
                     : unwrapSymbolBlobForNative(std::move(residualBytes));

    auto& object = out.object;
    object.formatVersion = pmsf::kPaperStreamFormatVersionV2;
    object.schemaMode = pmsf::PaperSchemaMode::Compact;
    object.payloadMode = pmsf::PaperPayloadMode::Native;
    object.codecVersion = codecVersion;
    object.blobLayout = blobLayout;
    object.pageIndex = false;
    auto& partition = out.partitions.front();

    const bool connLike = !wrappedInput && raw.size() >= 4 &&
                          std::memcmp(raw.data(), "CONN", 4) == 0;
    if (connLike) {
        pmsf::ConnectivityBlobReader reader(raw);
        const auto& header = reader.header();
        const int numRounds = static_cast<int>(header.num_rounds);
        const int numStreams = static_cast<int>(header.num_streams);
        if (numRounds <= 0 || numStreams <= 0 ||
            numRounds != object.numRounds) {
            return false;
        }
        const std::size_t headerBytes = header.headerByteSize();
        if (headerBytes > raw.size()) return false;
        std::vector<pmsf::PaperLodRow> lods;
        lods.reserve(static_cast<std::size_t>(numRounds));
        partition.payloadDirectory.clear();
        partition.nativePayload.clear();
        partition.connHeader.assign(raw.begin(), raw.begin() + headerBytes);
        std::uint64_t cursor = 0;
        for (int r = 0; r < numRounds; ++r) {
            const std::size_t begin =
                headerBytes + header.stream_offsets[static_cast<std::size_t>(r) * numStreams];
            const std::size_t end = headerBytes +
                                    header.stream_offsets[static_cast<std::size_t>(r + 1) *
                                                          numStreams];
            if (end < begin || end > raw.size()) return false;
            pmsf::PaperLodRow lod;
            lod.partitionId = partition.partitionId;
            lod.lodId = r;  // CONN builder rounds are already decoder order
            lod.nativePayload.assign(raw.begin() + begin, raw.begin() + end);
            lod.payloadByteOffset = cursor;
            lod.payloadByteLength = lod.nativePayload.size();
            partition.payloadDirectory.push_back(
                {r, -1, cursor, static_cast<std::uint64_t>(lod.nativePayload.size())});
            cursor += lod.nativePayload.size();
            lods.push_back(std::move(lod));
        }
        out.lods = std::move(lods);
        partition.numLods = numRounds;
        object.nativeFrame = pmsf::PaperNativeFrame::ConnRound;
        object.schemeVersion = header.scheme_version;
        object.raPrefixUnit = "lod";
        object.nativeStreamNames.clear();
        for (int s = 0; s < numStreams; ++s) {
            object.nativeStreamNames.push_back("s" + std::to_string(s));
        }
        return true;
    }

    // Whole-frame layout: keep one partition-level payload, one empty LOD row per
    // decode round (round count comes from the compact base header).
    out.lods.clear();
    partition.payloadDirectory.clear();
    partition.connHeader.clear();
    partition.nativePayload = std::move(raw);
    partition.numLods = object.numRounds;
    for (int r = 0; r < object.numRounds; ++r) {
        pmsf::PaperLodRow lod;
        lod.partitionId = partition.partitionId;
        lod.lodId = r;
        out.lods.push_back(std::move(lod));
    }
    object.nativeFrame = pmsf::PaperNativeFrame::Whole;
    object.schemeVersion = 0;
    object.raPrefixUnit = "partition";
    object.nativeStreamNames = {"symbols"};
    return true;
}

}  // namespace

void PaperSerializeOperator::serialize(std::string path) {
    const char* schemaOption = std::getenv("PMSF_PAPER_SCHEMA");
    const bool physicalTyped = schemaOption != nullptr && std::string(schemaOption) == "typed";
    if (physicalTyped && !pmsf::paperStreamV2Enabled()) {
        throw std::runtime_error("PaperSerializeOperator: physical typed relations require PaperStream v2");
    }
    if (!collector_) {
        throw std::runtime_error("PaperSerializeOperator: no collector");
    }
    const std::string baseName = pmsf::paperStorageStripExtension(path);
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const auto tempDir = std::filesystem::temp_directory_path() /
                         ("pmsf_paper_write_" + std::to_string(static_cast<long long>(stamp)));
    std::error_code ec;
    std::filesystem::create_directories(tempDir, ec);
    if (ec) {
        throw std::runtime_error("PaperSerializeOperator: cannot create temp dir");
    }

    try {
        const std::string legacyBase = (tempDir / "legacy").string();
        // Base BLOB source. The v1 bridge forces its legacy compact writer; for
        // v2 paper artifacts the canonical columnar writer mode must be used when
        // requested so the compact payload (and its base vertex order) is
        // byte-identical to the canonical `art_compact.bin` (M5-2 MMSE item5).
        const bool v2Bridge = pmsf::paperStreamV2Enabled();
        const char* baseSourceEnv = std::getenv("PMSF_PAPERSTREAM_V2_BASE_SOURCE");
        const bool canonicalBaseSource =
            v2Bridge && baseSourceEnv != nullptr &&
            std::string(baseSourceEnv) == "canonical";
        if (canonicalBaseSource) {
            if (std::getenv("PMSF_V2_COMPACT") == nullptr) {
                setenv("PMSF_V2_COMPACT", "1", /*overwrite=*/0);
            }
            setV2CompactMode(false);
        } else {
            setV2CompactMode(true);
        }
        ColumnarSerializeOperator::serialize(legacyBase + ".parquet");
        const auto compactBytes = readFileBytes(pmsf::paperCompactBinPath(legacyBase));
        if (compactBytes.empty()) {
            throw std::runtime_error(
                canonicalBaseSource
                    ? "PaperSerializeOperator: canonical compact bin missing "
                      "(PMSF_PAPERSTREAM_V2_BASE_SOURCE=canonical)"
                    : "PaperSerializeOperator: legacy compact bin missing");
        }

        const pmsf::PMSFSymbolTable* symbols = collector_->pmsfSymbolTable();
        const bool symbolsAvailable = symbols != nullptr && !symbols->empty();
        // M5-8: the v2 native path captures the authoritative collector blob
        // (rc_blob / CONN) directly and does not need a PMSFSymbolTable. Route-A
        // style collectors (RD canonical traversal) expose only that blob, so the
        // symbol-table requirement is kept for v1/typed paths only.
        const char* payloadEnvEarly = std::getenv("PMSF_PAPERSTREAM_V2_PAYLOAD");
        const bool forceTypedEarly =
            payloadEnvEarly != nullptr && std::string(payloadEnvEarly) == "typed";
        const bool v2NativeCapture =
            pmsf::paperStreamV2Enabled() && !forceTypedEarly;
        if (!symbolsAvailable && !v2NativeCapture) {
            throw std::runtime_error(
                "PaperSerializeOperator: collector does not expose a PMSFSymbolTable");
        }
        const std::int32_t fallbackRounds =
            header_ != nullptr ? static_cast<std::int32_t>(header_->getRound()) : 0;

        pmsf::PaperObjectRow object;
        object.algorithm = algorithmName_;
        object.schemaMode = pmsf::PaperSchemaMode::Compact;
        object.formatVersion = 1;
        object.numRounds = header_ ? header_->getRound() : 0;
        object.numPartitions = 1;
        object.totalVertices = originalInputVertices_ > 0 ? originalInputVertices_ : totalVertices_;
        object.totalFaces = originalInputFaces_ > 0 ? originalInputFaces_ : totalFaces_;
        object.enableQuantization = enableQuantization_;
        object.bboxMin[0] = 0.f;
        object.bboxMin[1] = 0.f;
        object.bboxMin[2] = 0.f;
        object.quantStep = 0.f;
        object.codecVersion = 4;

        std::vector<pmsf::PaperPartitionRow> partitions(1);
        partitions[0].objectId = 0;
        partitions[0].partitionId = 0;
        partitions[0].numVertices = totalVertices_;
        partitions[0].numFaces = totalFaces_;
        partitions[0].numLods =
            symbolsAvailable ? static_cast<std::int32_t>(symbols->size())
                             : fallbackRounds;
        partitions[0].enableQuantization = enableQuantization_;
        partitions[0].baseMesh = compactBytes;
        partitions[0].baseMeshCodec = "legacy_compact";

        pmsf::PaperStreamTable table;
        if (symbolsAvailable) {
            table = pmsf::paper_adapter::fromPMSFSymbolTable(object, partitions,
                                                             *symbols);
        } else {
            // v2 native capture: declaration-only table; payload comes from
            // captureNativeBlob()/lastSymbolsBlob_ below.
            table.object = object;
            table.partitions = partitions;
            table.object.numRounds = fallbackRounds;
        }
        if (table.object.numRounds <= 0) {
            table.object.numRounds =
                symbolsAvailable ? static_cast<std::int32_t>(symbols->size())
                                 : fallbackRounds;
        }
        // Paper contract stores geometry as quantized integers when possible;
        // keeping every removed vertex as 3 x float (12 B) dominates bpv.
        if (enableQuantization_ && collector_->mesh_ &&
            collector_->mesh_->f_quantStep > 0.f) {
            const auto& mesh = collector_->mesh_;
            const float step = mesh->f_quantStep;
            const float bx = mesh->bboxMin.x();
            const float by = mesh->bboxMin.y();
            const float bz = mesh->bboxMin.z();
            table.object.bboxMin[0] = bx;
            table.object.bboxMin[1] = by;
            table.object.bboxMin[2] = bz;
            table.object.quantStep = step;
            table.object.quantBits = static_cast<std::int32_t>(mesh->i_nbQuantBits);
            table.partitions[0].bboxMin[0] = bx;
            table.partitions[0].bboxMin[1] = by;
            table.partitions[0].bboxMin[2] = bz;
            table.partitions[0].quantStep = step;
            table.partitions[0].quantBits = static_cast<std::int32_t>(mesh->i_nbQuantBits);
            auto quantize = [&](const std::vector<float>& f, std::vector<std::int32_t>& q) {
                // Residual-only paper rows (e.g. PPMC C_r^f) already have q;
                // don't clear it when there are no float coordinates.
                if (f.empty()) return;
                q.clear();
                q.reserve(f.size());
                for (std::size_t i = 0; i + 2 < f.size(); i += 3) {
                    q.push_back(static_cast<std::int32_t>(
                        std::lround((f[i] - bx) / step - 0.5f)));
                    q.push_back(static_cast<std::int32_t>(
                        std::lround((f[i + 1] - by) / step - 0.5f)));
                    q.push_back(static_cast<std::int32_t>(
                        std::lround((f[i + 2] - bz) / step - 0.5f)));
                }
            };
            for (auto& lod : table.lods) {
                quantize(lod.CfF, lod.CfQ);
                quantize(lod.CvF, lod.CvQ);
                lod.CfF.clear();
                lod.CvF.clear();
                lod.countCf = static_cast<std::int32_t>(lod.CfQ.size() / 3);
                lod.countCv = static_cast<std::int32_t>(lod.CvQ.size() / 3);
            }
        }
        // M4-3 opt-in v2 bridge: write the v2 single-file artifact through
        // PaperStreamWriterV2. With PMSF_PAPERSTREAM_V2 unset/0 the original v1
        // path below runs unchanged (byte-identical v1 artifacts).
        if (pmsf::paperStreamV2Enabled()) {
            // Payload mode: native by default (real decoded round driver); the
            // typed compact schema remains available via
            // PMSF_PAPERSTREAM_V2_PAYLOAD=typed for schema experiments.
            const char* payloadEnv = std::getenv("PMSF_PAPERSTREAM_V2_PAYLOAD");
            const bool forceTyped =
                payloadEnv != nullptr && std::string(payloadEnv) == "typed";
            // M5-2 (S1): PMSF_PAPERSTREAM_V2_PAYLOAD_CODEC=sbc1|zstd keeps the
            // canonical SBC1 envelope bytes instead of stripping them. The decoder
            // already unwraps codec 1 (zlib) / 2 (zstd). Default = raw.
            const char* payloadCodecEnv =
                std::getenv("PMSF_PAPERSTREAM_V2_PAYLOAD_CODEC");
            const std::string payloadCodecName =
                payloadCodecEnv != nullptr ? std::string(payloadCodecEnv) : "raw";
            const bool wantWrapped = payloadCodecName == "sbc1" ||
                                     payloadCodecName == "zstd";
            std::uint8_t payloadCodec = 0;
            bool nativeOk = false;
            if (!forceTyped) {
                // lastSymbolsBlob_ is exactly the rc_blob the canonical columnar
                // writer produced (raw, or SBC1-wrapped when symbol-blob
                // compression is enabled). It is the authoritative native stream;
                // the codec-4 residual/bit/lod tables are only segment splits of it.
                std::vector<std::uint8_t> nativeBlob;
                bool keepWrapped = false;
                if (wantWrapped && isSbc1Blob(lastSymbolsBlob_)) {
                    // Byte-identical canonical wrapper (no recompression).
                    nativeBlob = lastSymbolsBlob_;
                    keepWrapped = true;
                    payloadCodec =
                        lastSymbolsBlob_.size() > 4 && lastSymbolsBlob_[4] == 2 ? 2 : 1;
                } else {
                    nativeBlob = captureNativeBlob(
                        collector_.get(), enableQuantization_, enablePrediction_,
                        enableCompress_, lastSymbolsBlob_);
                    if (wantWrapped && !nativeBlob.empty()) {
                        bool wrapped = false;
                        nativeBlob = payloadCodecName == "zstd"
                                         ? wrapSbc1Zstd(nativeBlob, wrapped)
                                         : wrapSbc1Zlib(nativeBlob, wrapped);
                        keepWrapped = wrapped;
                        payloadCodec = wrapped ? (payloadCodecName == "zstd" ? 2 : 1) : 0;
                    } else if (!symbolsAvailable && !nativeBlob.empty() &&
                               nativeBlob.size() >= 4 &&
                               std::memcmp(nativeBlob.data(), "CONN", 4) == 0) {
                        // Blob container cannot carry CONN per-LOD directories;
                        // wrap the canonical CONN bytes instead of splitting.
                        bool wrapped = false;
                        nativeBlob = wrapSbc1Zlib(nativeBlob, wrapped);
                        keepWrapped = wrapped;
                        payloadCodec = wrapped ? 1 : 0;
                    }
                }
                if (!nativeBlob.empty()) {
                    pmsf::PaperStreamTable native = table;
                    const bool built = buildNativeV2Table(
                        native, std::move(nativeBlob), getCodecVersion(),
                        getBlobLayout(), keepWrapped);
                    if (built) {
                        table = std::move(native);
                        nativeOk = true;
                    } else {
                        payloadCodec = 0;
                    }
                }
            }
            if (!nativeOk) {
                pmsf::finalizePaperStreamV2Table(table, getBlobLayout());
            }
            pmsf::PaperWriterV2Options v2Options;
            if (physicalTyped) {
                table.object.schemaMode = pmsf::PaperSchemaMode::Typed;
                // Only actual per-LOD data benefits from LOD row groups. A
                // whole native payload lives at the partition and cannot be
                // pruned by splitting empty LOD declarations into extra groups.
                const bool wholeNative = table.object.payloadMode == pmsf::PaperPayloadMode::Native &&
                    (table.object.nativeFrame == pmsf::PaperNativeFrame::Whole ||
                     table.object.nativeFrame == pmsf::PaperNativeFrame::NamedPayload);
                v2Options.lodsPerRowGroup = wholeNative ? 0 : 4;
                if (table.object.payloadMode == pmsf::PaperPayloadMode::Typed && !table.lods.empty()) {
                    table.object.raPrefixUnit = "lod_block";
                }
            }
            // M5-1 zero-overhead profile: only meaningful for native whole-frame
            // payloads (e.g. PPMC); columnar v2 remains the default for every
            // other layout and for the existing test matrix.
            if (const char* containerEnv =
                    std::getenv("PMSF_PAPERSTREAM_V2_CONTAINER");
                containerEnv != nullptr && std::string(containerEnv) == "blob") {
                v2Options.container = pmsf::PaperV2Container::Blob;
                v2Options.blobPayloadCodec = payloadCodec;
                if (const char* baseEnv = std::getenv("PMSF_PAPERSTREAM_V2_BASE");
                    baseEnv != nullptr && std::string(baseEnv) == "raw-v2cp") {
                    v2Options.blobBaseCodec = 1;
                }
            }
            pmsf::PaperStreamWriterV2 v2Writer;
            v2Writer.write(table, baseName + "_paper.parquet", v2Options);
            std::filesystem::remove_all(tempDir, ec);
            return;
        }

                // M2b pilot bridge: keep the small legacy symbol tables so the existing
        // codec-4 decoder can run unchanged.  The compact bin is already stored
        // in the partition BLOB, so skip it here.
        for (const auto& entry : std::filesystem::directory_iterator(tempDir)) {
            if (!entry.is_regular_file()) continue;
            const std::string name = entry.path().filename().string();
            if (name == "legacy_compact.bin") continue;
            // The paper partition BLOB already carries header+base mesh; do not
            // duplicate the legacy metadata/base_mesh/boundary tables.
            if (name == "legacy_metadata.parquet" ||
                name == "legacy_base_mesh.parquet" ||
                name == "legacy_vertices.parquet" ||
                name == "legacy_partitions.parquet" ||
                name == "legacy_boundary.parquet") {
                continue;
            }
            if (name.rfind("legacy", 0) != 0) continue;
            auto bytes = readFileBytes(entry.path());
            if (!bytes.empty()) table.legacyFiles[name.substr(6)] = std::move(bytes);
        }
        // The compact paper schema must not hide algorithm side channels in
        // `extensions`: all verified algorithms decode identically after they are
        // dropped. PMSF_PAPER_KEEP_EXTENSIONS=1 is a debug-only opt-out.
        const char* keepExtEnv = std::getenv("PMSF_PAPER_KEEP_EXTENSIONS");
        const bool keepExtensions =
            keepExtEnv && *keepExtEnv && std::string(keepExtEnv) != "0";
        if (!keepExtensions) {
            for (auto& lod : table.lods) lod.extension.clear();
        }
        pmsf::PaperStreamWriter writer;
        writer.write(table, baseName + "_paper.parquet");
    } catch (...) {
        std::filesystem::remove_all(tempDir, ec);
        throw;
    }
    std::filesystem::remove_all(tempDir, ec);
}
