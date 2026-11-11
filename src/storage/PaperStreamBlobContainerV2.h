/*
 * @file PaperStreamBlobContainerV2.h
 * @brief PaperStream v2 minimal "single-blob" container profile (M5-1).
 *
 * Motivation (PPMC M5-0 probe): the columnar v2 container costs ~3.2 KB of
 * dictionary/column/footer overhead on small artifacts (PPMC horse19k:
 * 60,006 B vs canonical 56,799 B) even though the payload bytes themselves are
 * already minimal (compact base BLOB + one native symbol BLOB). This profile
 * packs the same v2 semantics into ONE binary column of ONE row:
 *
 *   parquet [paper_blob BINARY] x 1 row
 *   paper_blob = "P2BL" u16 version=1 u16 flags
 *                u32 algorithmLen  [algorithm]
 *                i32 objectId, numRounds, totalVertices, totalFaces,
 *                    numPartitions, codecVersion, blobLayout, schemeVersion
 *                u8 payloadMode, u8 nativeFrame, u8 raPrefixUnit
 *                u32 boundaryCodecLen [boundaryCodec]
 *                u32 partitionCount
 *                per partition:
 *                  i32 partitionId, i32 numLods
 *                  u32 baseMeshLen [baseMesh bytes = canonical compact BLOB]
 *                  u32 nativeLen   [partition-level native payload]
 *
 * Scope: payloadMode=Native and nativeFrame=Whole|NamedPayload (whole-partition
 * payloads; no per-LOD directory). Per-LOD containers keep using the regular
 * columnar v2 schema. Randomized access granularity is honestly "partition"
 * (page-level is M6): readPrefix(maxLod) truncates the synthesized LOD rows but
 * the native payload is still the whole partition BLOB, exactly like the
 * existing Whole-frame columnar profile.
 *
 * Deterministic: fixed field order, no padding; same table => same bytes.
 * Fail-closed: bad magic/version/truncation/enum mismatch => false with error.
 */
#ifndef PMSF_STORAGE_PAPER_STREAM_BLOB_CONTAINER_V2_H
#define PMSF_STORAGE_PAPER_STREAM_BLOB_CONTAINER_V2_H

#include "storage/PaperStreamTable.h"

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <zlib.h>

namespace pmsf {
namespace paper_blob_v2 {

inline constexpr char kMagic[4] = {'P', '2', 'B', 'L'};
/// v1 = raw payload + envelope base (M5-1). v2 adds the two codec bytes:
/// payload codec (0=raw, 1=SBC1, 2=SBC1-zstd) and base codec (0=envelope,
/// 1=raw V2CP payload).
inline constexpr std::uint16_t kVersion = 2;
inline constexpr std::uint16_t kVersionLegacy = 1;

/// Payload codec bytes: the container stores the payload bytes verbatim; the
/// codec is provenance for decode/manifest. Decode always unwraps SBC1 if
/// present (existing `unwrapSymbolBlobIfNeeded` path).
inline constexpr std::uint8_t kPayloadCodecRaw = 0;
inline constexpr std::uint8_t kPayloadCodecSbc1 = 1;
inline constexpr std::uint8_t kPayloadCodecSbc1Zstd = 2;
/// M5-3: plain zlib/deflate payload stream (no SBC1 envelope); used by the QEM
/// multi-stream bundle to keep container framing minimal.
inline constexpr std::uint8_t kPayloadCodecZlibRaw = 3;
/// Base codec: 0 = compact bin envelope (canonical `art_compact.bin` bytes),
/// 1 = raw V2CP payload (envelope stripped; parser re-wraps with zlib).
inline constexpr std::uint8_t kBaseCodecEnvelope = 0;
inline constexpr std::uint8_t kBaseCodecRawV2cp = 1;
/// M5-3: no compact base at all (progressive payload carries its own base
/// stream); partition base length is 0.
inline constexpr std::uint8_t kBaseCodecNone = 2;

inline void putU16(std::vector<std::uint8_t>& out, std::uint16_t v) {
    out.push_back(static_cast<std::uint8_t>(v & 0xffu));
    out.push_back(static_cast<std::uint8_t>((v >> 8) & 0xffu));
}
inline void putU32(std::vector<std::uint8_t>& out, std::uint32_t v) {
    for (int i = 0; i < 4; ++i)
        out.push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xffu));
}
inline void putI32(std::vector<std::uint8_t>& out, std::int32_t v) {
    putU32(out, static_cast<std::uint32_t>(v));
}
inline void putBytes(std::vector<std::uint8_t>& out,
                     const std::vector<std::uint8_t>& bytes) {
    putU32(out, static_cast<std::uint32_t>(bytes.size()));
    out.insert(out.end(), bytes.begin(), bytes.end());
}
inline void putString(std::vector<std::uint8_t>& out, const std::string& s) {
    putU32(out, static_cast<std::uint32_t>(s.size()));
    out.insert(out.end(), s.begin(), s.end());
}

inline bool getU16(const std::vector<std::uint8_t>& in, std::size_t& off,
                   std::uint16_t& v) {
    if (off + 2 > in.size()) return false;
    v = static_cast<std::uint16_t>(in[off]) |
        static_cast<std::uint16_t>(static_cast<std::uint16_t>(in[off + 1]) << 8);
    off += 2;
    return true;
}
inline bool getU32(const std::vector<std::uint8_t>& in, std::size_t& off,
                   std::uint32_t& v) {
    if (off + 4 > in.size()) return false;
    v = static_cast<std::uint32_t>(in[off]) |
        (static_cast<std::uint32_t>(in[off + 1]) << 8) |
        (static_cast<std::uint32_t>(in[off + 2]) << 16) |
        (static_cast<std::uint32_t>(in[off + 3]) << 24);
    off += 4;
    return true;
}
inline bool getI32(const std::vector<std::uint8_t>& in, std::size_t& off,
                   std::int32_t& v) {
    std::uint32_t u = 0;
    if (!getU32(in, off, u)) return false;
    v = static_cast<std::int32_t>(u);
    return true;
}
inline bool getBytes(const std::vector<std::uint8_t>& in, std::size_t& off,
                     std::vector<std::uint8_t>& out) {
    std::uint32_t len = 0;
    if (!getU32(in, off, len)) return false;
    if (off + len > in.size()) return false;
    out.assign(in.begin() + static_cast<std::ptrdiff_t>(off),
               in.begin() + static_cast<std::ptrdiff_t>(off + len));
    off += len;
    return true;
}
inline bool getString(const std::vector<std::uint8_t>& in, std::size_t& off,
                      std::string& out) {
    std::vector<std::uint8_t> bytes;
    if (!getBytes(in, off, bytes)) return false;
    out.assign(bytes.begin(), bytes.end());
    return true;
}

inline std::uint8_t payloadModeCode(PaperPayloadMode mode) {
    return static_cast<std::uint8_t>(mode);  // 0=Typed,1=Native,2=Mixed
}
inline bool payloadModeFromCode(std::uint8_t code, PaperPayloadMode& out) {
    switch (code) {
        case 0: out = PaperPayloadMode::Typed; return true;
        case 1: out = PaperPayloadMode::Native; return true;
        case 2: out = PaperPayloadMode::Mixed; return true;
        default: return false;
    }
}
inline std::uint8_t nativeFrameCode(PaperNativeFrame frame) {
    return static_cast<std::uint8_t>(frame);  // ConnRound=0..NamedPayload=3
}
inline bool nativeFrameFromCode(std::uint8_t code, PaperNativeFrame& out) {
    if (code > 3) return false;
    out = static_cast<PaperNativeFrame>(code);
    return true;
}
inline std::uint8_t raUnitCode(const std::string& unit) {
    if (unit == "lod") return 0;
    if (unit == "lod_block") return 1;
    if (unit == "partition") return 2;
    return 2;
}
inline void raUnitFromCode(std::uint8_t code, std::string& out) {
    out = code == 0 ? "lod" : code == 1 ? "lod_block" : "partition";
}

}  // namespace paper_blob_v2

/// @brief Options for the single-blob container (M5-2 zero-overhead rows).
struct PaperBlobContainerOptions {
    std::uint8_t payloadCodec = paper_blob_v2::kPayloadCodecRaw;
    std::uint8_t baseCodec = paper_blob_v2::kBaseCodecEnvelope;
};

namespace paper_blob_v2 {

inline std::uint32_t readU32At(const std::vector<std::uint8_t>& in, std::size_t off) {
    return static_cast<std::uint32_t>(in[off]) |
           (static_cast<std::uint32_t>(in[off + 1]) << 8) |
           (static_cast<std::uint32_t>(in[off + 2]) << 16) |
           (static_cast<std::uint32_t>(in[off + 3]) << 24);
}

inline void appendU32(std::vector<std::uint8_t>& out, std::uint32_t v) {
    for (int i = 0; i < 4; ++i)
        out.push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xffu));
}

/// @brief Strip the compact-bin envelope, returning the raw V2CP payload.
///        Supports the default zlib envelope (extended zlib-codec flag too); a
///        brotli/bitctx envelope fails closed (opt-in raw-base mode is only used
///        with canonical zlib compact bins). No rangecoder dependency here: this
///        header is also compiled into storage-only test targets.
inline bool extractRawV2cpPayload(const std::vector<std::uint8_t>& envelopeBytes,
                                 std::vector<std::uint8_t>& rawPayload) {
    if (envelopeBytes.size() < 8) return false;
    std::uint32_t rawLen = readU32At(envelopeBytes, 0);
    const std::uint32_t compLen = readU32At(envelopeBytes, 4);
    constexpr std::uint32_t kExtendedFlag = 0x80000000u;
    const bool extended = (rawLen & kExtendedFlag) != 0;
    rawLen &= ~kExtendedFlag;
    if (rawLen == 0 || compLen == 0 ||
        envelopeBytes.size() != 8u + static_cast<std::size_t>(compLen)) {
        return false;
    }
    const std::uint8_t* compressed = envelopeBytes.data() + 8;
    std::size_t compressedLen = compLen;
    if (extended) {
        if (compressedLen < 1) return false;
        const std::uint8_t codec = compressed[0];
        if (codec != 1) return false;  // zlib only
        ++compressed;
        --compressedLen;
    }
    rawPayload.resize(rawLen);
    uLongf outLen = static_cast<uLongf>(rawLen);
    const int zret = uncompress(rawPayload.data(), &outLen, compressed,
                                static_cast<uLong>(compressedLen));
    if (zret != Z_OK || outLen != static_cast<uLongf>(rawLen)) {
        rawPayload.clear();
        return false;
    }
    return true;
}

/// @brief Re-wrap a raw V2CP payload into the canonical compact-bin envelope
///        (`[u32 rawLen][u32 compLen][zlib stream]`), which is what
///        `readV2CompactBin()`/`initPaperV2Native()` consume.
inline bool wrapRawV2cpPayload(const std::vector<std::uint8_t>& rawPayload,
                               std::vector<std::uint8_t>& envelopeBytes) {
    if (rawPayload.empty()) return false;
    uLongf cap = compressBound(static_cast<uLong>(rawPayload.size()));
    std::vector<std::uint8_t> compressed(cap);
    uLongf compressedLen = cap;
    const int zret = compress2(compressed.data(), &compressedLen, rawPayload.data(),
                               static_cast<uLong>(rawPayload.size()), Z_BEST_COMPRESSION);
    if (zret != Z_OK) return false;
    compressed.resize(static_cast<std::size_t>(compressedLen));
    envelopeBytes.clear();
    envelopeBytes.reserve(8 + compressed.size());
    appendU32(envelopeBytes, static_cast<std::uint32_t>(rawPayload.size()));
    appendU32(envelopeBytes, static_cast<std::uint32_t>(compressed.size()));
    envelopeBytes.insert(envelopeBytes.end(), compressed.begin(), compressed.end());
    return true;
}

}  // namespace paper_blob_v2

/**
 * @brief Serialize a whole-frame native v2 table into the single-blob profile.
 * @return empty string on contract violation, with `error` filled.
 */
inline std::vector<std::uint8_t> serializePaperStreamV2BlobContainer(
    const PaperStreamTable& table, std::string* error = nullptr,
    PaperBlobContainerOptions options = {}) {
    using namespace paper_blob_v2;
    auto fail = [&](const std::string& message) {
        if (error) *error = message;
        return std::vector<std::uint8_t>();
    };
    std::string validationError;
    if (!paperValidateStreamTable(table, &validationError)) {
        return fail("blob container: invalid v2 table: " + validationError);
    }
    if (table.object.payloadMode != PaperPayloadMode::Native) {
        return fail("blob container requires payloadMode=Native");
    }
    if (table.object.nativeFrame != PaperNativeFrame::Whole &&
        table.object.nativeFrame != PaperNativeFrame::NamedPayload) {
        return fail("blob container requires nativeFrame=Whole|NamedPayload");
    }
    if (table.partitions.empty()) return fail("blob container: no partitions");
    for (const auto& partition : table.partitions) {
        if (partition.nativePayload.empty()) {
            return fail("blob container: partition " +
                        std::to_string(partition.partitionId) +
                        " has no native payload");
        }
        if (!partition.payloadDirectory.empty() || !partition.connHeader.empty()) {
            return fail("blob container does not support per-LOD directories/CONN headers");
        }
    }

    if (options.payloadCodec > kPayloadCodecZlibRaw) {
        return fail("blob container: unsupported payload codec " +
                    std::to_string(options.payloadCodec));
    }
    if (options.baseCodec > kBaseCodecNone) {
        return fail("blob container: unsupported base codec " +
                    std::to_string(options.baseCodec));
    }
    std::uint8_t effectiveBaseCodec = options.baseCodec;
    std::vector<std::vector<std::uint8_t>> basePayloads;
    basePayloads.reserve(table.partitions.size());
    if (effectiveBaseCodec == kBaseCodecRawV2cp) {
        bool extractOk = true;
        for (const auto& partition : table.partitions) {
            std::vector<std::uint8_t> raw;
            if (!extractRawV2cpPayload(partition.baseMesh, raw)) {
                extractOk = false;
                break;
            }
            basePayloads.push_back(std::move(raw));
        }
        if (!extractOk) {
            // Non-zlib compact envelopes (e.g. bit-ctx RC) cannot be stripped by
            // this rangecoder-free storage helper. Degrade gracefully to the
            // canonical envelope bytes instead of failing the encode; decode is
            // unaffected (envelope is the default contract).
            effectiveBaseCodec = kBaseCodecEnvelope;
            basePayloads.clear();
        }
    }
    if (effectiveBaseCodec == kBaseCodecNone) {
        basePayloads.clear();
        for (std::size_t i = 0; i < table.partitions.size(); ++i) {
            basePayloads.emplace_back();
        }
    } else if (effectiveBaseCodec == kBaseCodecEnvelope) {
        for (const auto& partition : table.partitions) {
            if (partition.baseMesh.empty()) {
                return fail("blob container: empty compact base requires "
                            "baseCodec=None");
            }
            basePayloads.push_back(partition.baseMesh);
        }
    }

    std::vector<std::uint8_t> out;
    out.reserve(64 + table.object.algorithm.size() + table.object.boundaryCodec.size());
    out.insert(out.end(), kMagic, kMagic + 4);
    putU16(out, kVersion);
    putU16(out, 0);  // flags (reserved)
    out.push_back(options.payloadCodec);
    out.push_back(effectiveBaseCodec);
    putString(out, table.object.algorithm);
    putI32(out, table.object.objectId);
    putI32(out, table.object.numRounds);
    putI32(out, table.object.totalVertices);
    putI32(out, table.object.totalFaces);
    putI32(out, table.object.numPartitions);
    putI32(out, table.object.codecVersion);
    putI32(out, table.object.blobLayout);
    putI32(out, table.object.schemeVersion);
    out.push_back(payloadModeCode(table.object.payloadMode));
    out.push_back(nativeFrameCode(table.object.nativeFrame));
    out.push_back(raUnitCode(table.object.raPrefixUnit));
    putString(out, table.object.boundaryCodec);
    putU32(out, static_cast<std::uint32_t>(table.partitions.size()));
    for (std::size_t i = 0; i < table.partitions.size(); ++i) {
        const auto& partition = table.partitions[i];
        putI32(out, partition.partitionId);
        putI32(out, partition.numLods);
        putBytes(out, basePayloads[i]);
        putBytes(out, partition.nativePayload);
    }
    if (error) error->clear();
    return out;
}

/**
 * @brief Parse the single-blob profile back into a whole-frame native v2 table.
 *        Synthesizes one empty LOD row per decode round (num_lods contract of
 *        the v2 table; native whole-frame payload is partition-level).
 */
inline bool parsePaperStreamV2BlobContainer(
    const std::vector<std::uint8_t>& in, PaperStreamTable& table,
    std::string* error = nullptr, std::uint8_t* outPayloadCodec = nullptr,
    std::uint8_t* outBaseCodec = nullptr) {
    using namespace paper_blob_v2;
    auto fail = [&](const std::string& message) {
        if (error) *error = message;
        return false;
    };
    if (in.size() < 8 || std::memcmp(in.data(), kMagic, 4) != 0) {
        return fail("blob container: bad magic");
    }
    std::size_t off = 4;
    std::uint16_t version = 0;
    std::uint16_t flags = 0;
    if (!getU16(in, off, version) || !getU16(in, off, flags)) {
        return fail("blob container: truncated version/flags");
    }
    if (version != kVersion && version != kVersionLegacy) {
        return fail("blob container: unsupported version " + std::to_string(version));
    }
    std::uint8_t payloadCodec = kPayloadCodecRaw;
    std::uint8_t baseCodec = kBaseCodecEnvelope;
    if (version >= 2) {
        if (off + 2 > in.size()) return fail("blob container: truncated codecs");
        payloadCodec = in[off++];
        baseCodec = in[off++];
        if (payloadCodec > kPayloadCodecZlibRaw || baseCodec > kBaseCodecNone) {
            return fail("blob container: unsupported codec flags");
        }
    }
    if (outPayloadCodec) *outPayloadCodec = payloadCodec;
    if (outBaseCodec) *outBaseCodec = baseCodec;
    table = PaperStreamTable{};
    table.object.formatVersion = kPaperStreamFormatVersionV2;
    table.object.schemaMode = PaperSchemaMode::Compact;
    if (!getString(in, off, table.object.algorithm)) {
        return fail("blob container: truncated algorithm");
    }
    if (!getI32(in, off, table.object.objectId) ||
        !getI32(in, off, table.object.numRounds) ||
        !getI32(in, off, table.object.totalVertices) ||
        !getI32(in, off, table.object.totalFaces) ||
        !getI32(in, off, table.object.numPartitions) ||
        !getI32(in, off, table.object.codecVersion) ||
        !getI32(in, off, table.object.blobLayout) ||
        !getI32(in, off, table.object.schemeVersion)) {
        return fail("blob container: truncated header");
    }
    if (off + 3 > in.size()) return fail("blob container: truncated enums");
    if (!payloadModeFromCode(in[off++], table.object.payloadMode)) {
        return fail("blob container: bad payload mode");
    }
    if (!nativeFrameFromCode(in[off++], table.object.nativeFrame)) {
        return fail("blob container: bad native frame");
    }
    raUnitFromCode(in[off++], table.object.raPrefixUnit);
    if (!getString(in, off, table.object.boundaryCodec)) {
        return fail("blob container: truncated boundary codec");
    }
    std::uint32_t partitionCount = 0;
    if (!getU32(in, off, partitionCount)) {
        return fail("blob container: truncated partition count");
    }
    if (partitionCount == 0 || partitionCount > 65535u) {
        return fail("blob container: invalid partition count");
    }
    if (table.object.numPartitions != 0 &&
        table.object.numPartitions != static_cast<std::int32_t>(partitionCount)) {
        return fail("blob container: numPartitions/partition count mismatch");
    }
    table.object.numPartitions = static_cast<std::int32_t>(partitionCount);
    for (std::uint32_t i = 0; i < partitionCount; ++i) {
        PaperPartitionRow partition;
        std::vector<std::uint8_t> baseBytes;
        if (!getI32(in, off, partition.partitionId) ||
            !getI32(in, off, partition.numLods) ||
            !getBytes(in, off, baseBytes) ||
            !getBytes(in, off, partition.nativePayload)) {
            return fail("blob container: truncated partition " + std::to_string(i));
        }
        if (baseCodec == kBaseCodecNone) {
            if (!baseBytes.empty()) {
                return fail("blob container: baseCodec=None but base bytes present");
            }
            partition.baseMesh.clear();
        } else if (baseCodec == kBaseCodecEnvelope) {
            partition.baseMesh = std::move(baseBytes);
        } else if (!wrapRawV2cpPayload(baseBytes, partition.baseMesh)) {
            return fail("blob container: cannot re-wrap raw V2CP base payload");
        }
        if (partition.numLods < 0) return fail("blob container: negative numLods");
        if (partition.nativePayload.empty()) {
            return fail("blob container: empty native payload");
        }
        if (baseCodec != kBaseCodecNone && partition.baseMesh.empty()) {
            return fail("blob container: empty compact base payload");
        }
        partition.objectId = table.object.objectId;
        table.partitions.push_back(std::move(partition));
    }
    if (off != in.size()) return fail("blob container: trailing bytes");
    for (auto& partition : table.partitions) {
        // Compatibility with M5-3/M5-5 artifacts written before the helper fixed
        // numLods: a whole-frame native partition that omits numLods inherits the
        // object's decode-round count (the LOD rows are declarations only; the
        // payload stays partition-level).
        if (partition.numLods == 0 && table.object.numRounds > 0) {
            partition.numLods = table.object.numRounds;
        }
        for (std::int32_t lod = 0; lod < partition.numLods; ++lod) {
            PaperLodRow row;
            row.objectId = table.object.objectId;
            row.partitionId = partition.partitionId;
            row.lodId = lod;
            table.lods.push_back(std::move(row));
        }
    }
    std::string validationError;
    if (!paperValidateStreamTable(table, &validationError)) {
        return fail("blob container: parsed table invalid: " + validationError);
    }
    if (error) error->clear();
    return true;
}

}  // namespace pmsf

#endif  // PMSF_STORAGE_PAPER_STREAM_BLOB_CONTAINER_V2_H
