/*
 * @file PaperStreamMultiStreamV2.h
 * @brief PaperStream v2 multi-stream bundle (PSM2) + payload codecs (M5-3).
 *
 * Portable, dependency-light (zlib; optional zstd behind PMSF_HAS_ZSTD) helper
 * used by progressive codecs that already expose semantic byte streams and only
 * need a container-level mapping to PaperStream:
 *
 *   PSM2 bundle = magic "PSM2", version, stream count, QEM-style meta block,
 *                 then per stream (u8 id, u16 nameLen [name], u32 len [bytes]).
 *
 * The four QEM streams are stored verbatim:
 *   id 1 baseMesh, id 2 round, id 3 topology, id 4 geometry.
 * Streaming order is preserved; unknown ids are skipped by readers
 * (forward-compatible), duplicates/overlaps are rejected.
 *
 * Payload codecs (container-level only; stream bytes never change):
 *   0 None       - bundle bytes as-is
 *   1 Sbc1Zlib   - canonical SBC1 envelope, codec byte 1 (zlib)
 *   2 Sbc1Zstd   - canonical SBC1 envelope, codec byte 2 (zstd; PMSF_HAS_ZSTD)
 *   3 ZlibRaw    - plain zlib/deflate stream (no envelope); smallest framing
 *
 * The Arrow/Parquet artifact writer/reader lives in
 * `PaperStreamMultiStreamArtifactV2.h` so this header stays usable in
 * storage-only targets.
 */
#ifndef PMSF_STORAGE_PAPER_STREAM_MULTI_STREAM_V2_H
#define PMSF_STORAGE_PAPER_STREAM_MULTI_STREAM_V2_H

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <zlib.h>
#ifdef PMSF_HAS_ZSTD
#include <zstd.h>
#endif

namespace pmsf {
namespace paper_multistream {

inline constexpr char kBundleMagic[4] = {'P', 'S', 'M', '2'};
inline constexpr std::uint16_t kBundleVersion = 1;
inline constexpr std::uint16_t kMaxStreamCount = 64;

inline constexpr std::uint8_t kStreamBaseMesh = 1;
inline constexpr std::uint8_t kStreamRound = 2;
inline constexpr std::uint8_t kStreamTopology = 3;
inline constexpr std::uint8_t kStreamGeometry = 4;

inline constexpr std::uint32_t kQemFlagGeomF64 = 1u << 0;
/// Reserved for M6 optional provenance extension; M5 ignores it.
inline constexpr std::uint32_t kQemFlagHasProvenance = 1u << 1;

/// Payload codec ids stored in the P2BL container (payload bytes provenance).
enum class PayloadCodec : std::uint8_t {
    None = 0,
    Sbc1Zlib = 1,
    Sbc1Zstd = 2,
    ZlibRaw = 3,
    /// Alias kept for the QEM seam draft (`PayloadCodec::Zlib` == raw-zlib id 3).
    Zlib = ZlibRaw,
};

struct SeamStream {
    std::uint8_t id = 0;
    std::string name;
    std::vector<std::uint8_t> bytes;
};

/// QEM progressive metadata carried in the PSM2 bundle. Generic enough for other
/// multi-stream codecs, but validated with the QEM record-size rules.
struct QemBundleMeta {
    std::uint16_t qemVersion = 1;
    std::uint16_t idWidth = 2;    // 2 or 4 bytes per original vertex id
    std::uint16_t geomBytes = 4;  // 4 (f32) or 8 (f64) bytes per component
    std::uint16_t reserved = 0;
    std::uint32_t flags = 0;
    std::uint32_t originalVertices = 0;
    std::uint32_t originalFaces = 0;
    std::uint32_t baseVertices = 0;
    std::uint32_t baseFaces = 0;
    std::uint32_t roundCount = 0;
    std::uint32_t totalCollapses = 0;
};

struct QemPaperOptions {
    PayloadCodec payloadCodec = PayloadCodec::ZlibRaw;
};

// ---------------------------------------------------------------------------
// Little-endian helpers
// ---------------------------------------------------------------------------
inline void putU16(std::vector<std::uint8_t>& out, std::uint16_t v) {
    out.push_back(static_cast<std::uint8_t>(v & 0xffu));
    out.push_back(static_cast<std::uint8_t>((v >> 8) & 0xffu));
}
inline void putU32(std::vector<std::uint8_t>& out, std::uint32_t v) {
    for (int i = 0; i < 4; ++i)
        out.push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xffu));
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

// ---------------------------------------------------------------------------
// Payload codecs (container-level; stream bytes unchanged)
// ---------------------------------------------------------------------------
inline bool zlibDeflate(const std::vector<std::uint8_t>& raw,
                        std::vector<std::uint8_t>& compressed, std::string* error) {
    uLongf cap = compressBound(static_cast<uLong>(raw.size()));
    std::vector<std::uint8_t> buffer(cap);
    uLongf len = cap;
    const int zret = compress2(buffer.data(), &len, raw.data(),
                               static_cast<uLong>(raw.size()), Z_BEST_COMPRESSION);
    if (zret != Z_OK) {
        if (error) *error = "zlib compress2 failed";
        return false;
    }
    compressed.assign(buffer.begin(),
                      buffer.begin() + static_cast<std::ptrdiff_t>(len));
    return true;
}

inline bool zlibInflate(const std::vector<std::uint8_t>& compressed,
                        std::vector<std::uint8_t>& raw, std::string* error) {
    z_stream stream{};
    if (inflateInit(&stream) != Z_OK) {
        if (error) *error = "zlib inflateInit failed";
        return false;
    }
    std::vector<std::uint8_t> out;
    out.resize(1u << 16);
    stream.next_in = const_cast<Bytef*>(compressed.data());
    stream.avail_in = static_cast<uInt>(compressed.size());
    int ret = Z_OK;
    std::size_t used = 0;
    while (ret == Z_OK) {
        if (used == out.size()) out.resize(out.size() * 2);
        stream.next_out = out.data() + used;
        stream.avail_out = static_cast<uInt>(out.size() - used);
        ret = inflate(&stream, Z_NO_FLUSH);
        used = out.size() - stream.avail_out;
    }
    inflateEnd(&stream);
    if (ret != Z_STREAM_END) {
        if (error) *error = "zlib inflate failed";
        return false;
    }
    out.resize(used);
    raw = std::move(out);
    return true;
}

inline void wrapSbc1Envelope(std::vector<std::uint8_t> compressed,
                             std::uint8_t codec,
                             std::uint32_t rawLen,
                             std::vector<std::uint8_t>& wrapped) {
    wrapped.clear();
    wrapped.reserve(13u + compressed.size());
    wrapped.insert(wrapped.end(), {'S', 'B', 'C', '1', codec});
    putU32(wrapped, rawLen);
    putU32(wrapped, static_cast<std::uint32_t>(compressed.size()));
    wrapped.insert(wrapped.end(), compressed.begin(), compressed.end());
}

inline bool unwrapSbc1Envelope(const std::vector<std::uint8_t>& wrapped,
                               std::vector<std::uint8_t>& raw, std::string* error) {
    if (wrapped.size() < 13 || wrapped[0] != 'S' || wrapped[1] != 'B' ||
        wrapped[2] != 'C' || wrapped[3] != '1') {
        if (error) *error = "SBC1 envelope: bad magic";
        return false;
    }
    const std::uint8_t codec = wrapped[4];
    const std::uint32_t rawLen =
        static_cast<std::uint32_t>(wrapped[5]) |
        (static_cast<std::uint32_t>(wrapped[6]) << 8) |
        (static_cast<std::uint32_t>(wrapped[7]) << 16) |
        (static_cast<std::uint32_t>(wrapped[8]) << 24);
    const std::uint32_t compLen =
        static_cast<std::uint32_t>(wrapped[9]) |
        (static_cast<std::uint32_t>(wrapped[10]) << 8) |
        (static_cast<std::uint32_t>(wrapped[11]) << 16) |
        (static_cast<std::uint32_t>(wrapped[12]) << 24);
    if (rawLen == 0 || 13ull + compLen != wrapped.size()) {
        if (error) *error = "SBC1 envelope: bad lengths";
        return false;
    }
    const std::uint8_t* payload = wrapped.data() + 13;
    if (codec == 1) {
        raw.resize(rawLen);
        uLongf outLen = rawLen;
        const int zret = uncompress(raw.data(), &outLen, payload,
                                    static_cast<uLong>(compLen));
        if (zret != Z_OK || outLen != rawLen) {
            if (error) *error = "SBC1 envelope: zlib inflate failed";
            raw.clear();
            return false;
        }
        return true;
    }
    if (codec == 2) {
#ifdef PMSF_HAS_ZSTD
        raw.resize(rawLen);
        const std::size_t outLen =
            ZSTD_decompress(raw.data(), raw.size(), payload, compLen);
        if (ZSTD_isError(outLen) || outLen != rawLen) {
            if (error) *error = "SBC1 envelope: zstd decompress failed";
            raw.clear();
            return false;
        }
        return true;
#else
        if (error) *error = "SBC1 envelope: zstd codec not linked";
        return false;
#endif
    }
    if (error) *error = "SBC1 envelope: unknown codec";
    return false;
}

/// @brief Encode the PSM2 bundle bytes with the selected payload codec.
inline bool encodePayload(const std::vector<std::uint8_t>& raw, PayloadCodec codec,
                          std::vector<std::uint8_t>& payload, std::string* error) {
    switch (codec) {
        case PayloadCodec::None:
            payload = raw;
            return true;
        case PayloadCodec::ZlibRaw: {
            // Wire format (qem-repro contract): [u32 rawLen][u32 compLen][zlib].
            std::vector<std::uint8_t> compressed;
            if (!zlibDeflate(raw, compressed, error)) return false;
            payload.clear();
            payload.reserve(8 + compressed.size());
            putU32(payload, static_cast<std::uint32_t>(raw.size()));
            putU32(payload, static_cast<std::uint32_t>(compressed.size()));
            payload.insert(payload.end(), compressed.begin(), compressed.end());
            return true;
        }
        case PayloadCodec::Sbc1Zlib: {
            std::vector<std::uint8_t> compressed;
            if (!zlibDeflate(raw, compressed, error)) return false;
            wrapSbc1Envelope(std::move(compressed), 1,
                             static_cast<std::uint32_t>(raw.size()), payload);
            return true;
        }
        case PayloadCodec::Sbc1Zstd: {
#ifdef PMSF_HAS_ZSTD
            std::vector<std::uint8_t> compressed(ZSTD_compressBound(raw.size()));
            const std::size_t len = ZSTD_compress(compressed.data(), compressed.size(),
                                                  raw.data(), raw.size(), 22);
            if (ZSTD_isError(len)) {
                if (error) *error = "zstd compress failed";
                return false;
            }
            compressed.resize(len);
            wrapSbc1Envelope(std::move(compressed), 2,
                             static_cast<std::uint32_t>(raw.size()), payload);
            return true;
#else
            if (error) *error = "Sbc1Zstd payload codec requires PMSF_HAS_ZSTD";
            return false;
#endif
        }
    }
    if (error) *error = "unknown payload codec";
    return false;
}

/// @brief Decode the payload bytes back to the raw PSM2 bundle.
inline bool decodePayload(const std::vector<std::uint8_t>& payload,
                          PayloadCodec codec, std::vector<std::uint8_t>& raw,
                          std::string* error) {
    switch (codec) {
        case PayloadCodec::None:
            raw = payload;
            return true;
        case PayloadCodec::ZlibRaw: {
            if (payload.size() < 8) {
                if (error) *error = "zlib-raw payload: truncated header";
                return false;
            }
            const std::uint32_t rawLen =
                static_cast<std::uint32_t>(payload[0]) |
                (static_cast<std::uint32_t>(payload[1]) << 8) |
                (static_cast<std::uint32_t>(payload[2]) << 16) |
                (static_cast<std::uint32_t>(payload[3]) << 24);
            const std::uint32_t compLen =
                static_cast<std::uint32_t>(payload[4]) |
                (static_cast<std::uint32_t>(payload[5]) << 8) |
                (static_cast<std::uint32_t>(payload[6]) << 16) |
                (static_cast<std::uint32_t>(payload[7]) << 24);
            if (8ull + compLen != payload.size()) {
                if (error) *error = "zlib-raw payload: bad lengths";
                return false;
            }
            std::vector<std::uint8_t> compressed(payload.begin() + 8,
                                                payload.end());
            if (!zlibInflate(compressed, raw, error)) return false;
            if (raw.size() != rawLen) {
                if (error) *error = "zlib-raw payload: raw length mismatch";
                raw.clear();
                return false;
            }
            return true;
        }
        case PayloadCodec::Sbc1Zlib:
        case PayloadCodec::Sbc1Zstd:
            return unwrapSbc1Envelope(payload, raw, error);
    }
    if (error) *error = "unknown payload codec";
    return false;
}

// ---------------------------------------------------------------------------
// PSM2 bundle
// ---------------------------------------------------------------------------
inline std::vector<std::uint8_t> serializeBundle(
    const std::vector<SeamStream>& streams, const QemBundleMeta& meta,
    std::string* error = nullptr) {
    auto fail = [&](const std::string& message) {
        if (error) *error = message;
        return std::vector<std::uint8_t>();
    };
    if (streams.size() > kMaxStreamCount) return fail("PSM2: too many streams");
    std::vector<std::uint8_t> out;
    out.reserve(128);
    out.insert(out.end(), kBundleMagic, kBundleMagic + 4);
    putU16(out, kBundleVersion);
    putU16(out, static_cast<std::uint16_t>(streams.size()));
    putU16(out, meta.qemVersion);
    putU16(out, meta.idWidth);
    putU16(out, meta.geomBytes);
    putU16(out, meta.reserved);
    putU32(out, meta.flags);
    putU32(out, meta.originalVertices);
    putU32(out, meta.originalFaces);
    putU32(out, meta.baseVertices);
    putU32(out, meta.baseFaces);
    putU32(out, meta.roundCount);
    putU32(out, meta.totalCollapses);
    std::vector<std::uint8_t> seen;
    seen.reserve(streams.size());
    for (const auto& stream : streams) {
        if (stream.id == 0) return fail("PSM2: stream id 0 is reserved");
        for (std::uint8_t id : seen) {
            if (id == stream.id) return fail("PSM2: duplicate stream id");
        }
        seen.push_back(stream.id);
        if (stream.name.size() > 0xffffu) return fail("PSM2: stream name too long");
        out.push_back(stream.id);
        putU16(out, static_cast<std::uint16_t>(stream.name.size()));
        out.insert(out.end(), stream.name.begin(), stream.name.end());
        out.reserve(out.size() + 4 + stream.bytes.size());
        putU32(out, static_cast<std::uint32_t>(stream.bytes.size()));
        out.insert(out.end(), stream.bytes.begin(), stream.bytes.end());
    }
    return out;
}

inline bool parseBundle(const std::vector<std::uint8_t>& in,
                        std::vector<SeamStream>& streams, QemBundleMeta& meta,
                        std::string* error = nullptr) {
    auto fail = [&](const std::string& message) {
        if (error) *error = message;
        return false;
    };
    if (in.size() < 8 || std::memcmp(in.data(), kBundleMagic, 4) != 0) {
        return fail("PSM2: bad magic");
    }
    std::size_t off = 4;
    std::uint16_t version = 0;
    std::uint16_t streamCount = 0;
    if (!getU16(in, off, version) || !getU16(in, off, streamCount)) {
        return fail("PSM2: truncated version/count");
    }
    if (version != kBundleVersion) {
        return fail("PSM2: unsupported version " + std::to_string(version));
    }
    if (streamCount > kMaxStreamCount) return fail("PSM2: too many streams");
    if (!getU16(in, off, meta.qemVersion) || !getU16(in, off, meta.idWidth) ||
        !getU16(in, off, meta.geomBytes) || !getU16(in, off, meta.reserved) ||
        !getU32(in, off, meta.flags) || !getU32(in, off, meta.originalVertices) ||
        !getU32(in, off, meta.originalFaces) || !getU32(in, off, meta.baseVertices) ||
        !getU32(in, off, meta.baseFaces) || !getU32(in, off, meta.roundCount) ||
        !getU32(in, off, meta.totalCollapses)) {
        return fail("PSM2: truncated meta");
    }
    streams.clear();
    streams.reserve(streamCount);
    std::vector<std::uint8_t> seen;
    for (std::uint16_t i = 0; i < streamCount; ++i) {
        if (off + 1 > in.size()) return fail("PSM2: truncated stream id");
        SeamStream stream;
        stream.id = in[off++];
        if (stream.id == 0) return fail("PSM2: stream id 0 is reserved");
        std::uint16_t nameLen = 0;
        if (!getU16(in, off, nameLen)) return fail("PSM2: truncated stream name");
        if (off + nameLen > in.size()) return fail("PSM2: truncated stream name");
        stream.name.assign(in.begin() + static_cast<std::ptrdiff_t>(off),
                           in.begin() + static_cast<std::ptrdiff_t>(off + nameLen));
        off += nameLen;
        if (!getBytes(in, off, stream.bytes)) {
            return fail("PSM2: truncated stream bytes");
        }
        for (std::uint8_t id : seen) {
            if (id == stream.id) return fail("PSM2: duplicate stream id");
        }
        seen.push_back(stream.id);
        streams.push_back(std::move(stream));
    }
    if (off != in.size()) return fail("PSM2: trailing bytes");
    if (error) error->clear();
    return true;
}

/// @brief QEM record-size contract checks (metadata vs stream lengths).
inline bool validateQemBundle(const std::vector<SeamStream>& streams,
                              const QemBundleMeta& meta, std::string* error = nullptr) {
    auto fail = [&](const std::string& message) {
        if (error) *error = message;
        return false;
    };
    if (meta.idWidth != 2 && meta.idWidth != 4) return fail("PSM2: idWidth must be 2/4");
    if (meta.geomBytes != 4 && meta.geomBytes != 8) {
        return fail("PSM2: geomBytes must be 4/8");
    }
    const SeamStream* base = nullptr;
    const SeamStream* round = nullptr;
    const SeamStream* topo = nullptr;
    const SeamStream* geom = nullptr;
    for (const auto& stream : streams) {
        switch (stream.id) {
            case kStreamBaseMesh: base = &stream; break;
            case kStreamRound: round = &stream; break;
            case kStreamTopology: topo = &stream; break;
            case kStreamGeometry: geom = &stream; break;
            default: break;
        }
    }
    if (!base || !round || !topo || !geom) {
        return fail("PSM2: missing one of baseMesh/round/topology/geometry");
    }
    const std::uint64_t expectedBase =
        static_cast<std::uint64_t>(meta.baseVertices) * (meta.idWidth + 24ull) +
        static_cast<std::uint64_t>(meta.baseFaces) * 3ull * meta.idWidth;
    if (base->bytes.size() != expectedBase) {
        return fail("PSM2: baseMesh length does not match meta");
    }
    if (round->bytes.size() != static_cast<std::uint64_t>(meta.roundCount) * 4ull) {
        return fail("PSM2: roundStream length does not match roundCount");
    }
    std::uint64_t sumCollapses = 0;
    for (std::size_t i = 0; i + 4 <= round->bytes.size(); i += 4) {
        const std::uint32_t count =
            static_cast<std::uint32_t>(round->bytes[i]) |
            (static_cast<std::uint32_t>(round->bytes[i + 1]) << 8) |
            (static_cast<std::uint32_t>(round->bytes[i + 2]) << 16) |
            (static_cast<std::uint32_t>(round->bytes[i + 3]) << 24);
        sumCollapses += count;
    }
    if (sumCollapses != meta.totalCollapses) {
        return fail("PSM2: round counts do not sum to totalCollapses");
    }
    if (static_cast<std::uint64_t>(meta.baseFaces) + 2ull * meta.totalCollapses !=
        meta.originalFaces) {
        return fail("PSM2: baseFaces + 2*collapses != originalFaces");
    }
    if (geom->bytes.size() !=
        static_cast<std::uint64_t>(meta.totalCollapses) * 6ull * meta.geomBytes) {
        return fail("PSM2: geometry length does not match collapse/geom size");
    }
    (void)topo;
    if (error) error->clear();
    return true;
}

}  // namespace paper_multistream
}  // namespace pmsf

#endif  // PMSF_STORAGE_PAPER_STREAM_MULTI_STREAM_V2_H
