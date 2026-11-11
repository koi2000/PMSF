/*
 * @file PaperStreamCompactCodec.h
 * @brief Compact relation-bundle codecs for PaperStream v1 (M1b).
 *
 * The compact schema stores each logical relation as one BLOB column instead
 * of one Parquet column per stream.  The bundles are self-describing and are
 * decoded with the stream dictionary; they are NOT algorithm-opaque blobs.
 */
#ifndef PMSF_STORAGE_PAPER_STREAM_COMPACT_CODEC_H
#define PMSF_STORAGE_PAPER_STREAM_COMPACT_CODEC_H

#include "storage/PaperStreamTable.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>

namespace pmsf {

inline const char* paperSchemaModeName(PaperSchemaMode mode) {
    return mode == PaperSchemaMode::Compact ? "compact" : "typed";
}

inline PaperSchemaMode paperSchemaModeFromName(const std::string& name) {
    return name == "compact" ? PaperSchemaMode::Compact : PaperSchemaMode::Typed;
}

namespace paper_codec {

    inline void putU8(std::vector<std::uint8_t>& out, std::uint8_t v) {
        out.push_back(v);
    }

    inline void putU32(std::vector<std::uint8_t>& out, std::uint32_t v) {
        for (int i = 0; i < 4; ++i)
            out.push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xffu));
    }

    inline void putI32(std::vector<std::uint8_t>& out, std::int32_t v) {
        putU32(out, static_cast<std::uint32_t>(v));
    }

    inline void putF32(std::vector<std::uint8_t>& out, float v) {
        std::uint32_t bits = 0;
        std::memcpy(&bits, &v, sizeof(bits));
        putU32(out, bits);
    }

    inline void putBytes(std::vector<std::uint8_t>& out, const void* data, std::size_t size) {
        putU32(out, static_cast<std::uint32_t>(size));
        if (size == 0)
            return;
        const auto* p = static_cast<const std::uint8_t*>(data);
        out.insert(out.end(), p, p + size);
    }

    inline void putString(std::vector<std::uint8_t>& out, const std::string& text) {
        putBytes(out, text.data(), text.size());
    }

    inline void putVarintI32(std::vector<std::uint8_t>& out, std::int32_t v) {
        const auto u = static_cast<std::uint32_t>(v);
        std::uint32_t z = (u << 1) ^ static_cast<std::uint32_t>(v >> 31);
        while (z >= 0x80u) {
            out.push_back(static_cast<std::uint8_t>((z & 0x7fu) | 0x80u));
            z >>= 7;
        }
        out.push_back(static_cast<std::uint8_t>(z));
    }

    inline void putFixedAxis(std::vector<std::uint8_t>& out,
                             const std::vector<std::int32_t>& values) {
        std::int32_t mn = values.empty() ? 0 : values.front();
        std::int32_t mx = mn;
        for (std::int32_t v : values) { mn = std::min(mn, v); mx = std::max(mx, v); }
        std::uint32_t span = static_cast<std::uint32_t>(mx - mn);
        int bits = 1;
        while (bits < 32 && (1u << bits) <= span) ++bits;
        putI32(out, mn);
        putU8(out, static_cast<std::uint8_t>(bits));
        std::vector<std::uint8_t> packed;
        std::uint64_t acc = 0;
        int nbits = 0;
        for (std::int32_t v : values) {
            acc |= static_cast<std::uint64_t>(v - mn) << nbits;
            nbits += bits;
            while (nbits >= 8) {
                packed.push_back(static_cast<std::uint8_t>(acc & 0xffu));
                acc >>= 8;
                nbits -= 8;
            }
        }
        if (nbits > 0) packed.push_back(static_cast<std::uint8_t>(acc & 0xffu));
        putBytes(out, packed.data(), packed.size());
    }

    inline bool getVarintI32(const std::vector<std::uint8_t>& in, std::size_t& off,
                             std::int32_t& v) {
        std::uint32_t z = 0;
        int shift = 0;
        while (off < in.size() && shift <= 28) {
            const std::uint8_t byte = in[off++];
            z |= static_cast<std::uint32_t>(byte & 0x7fu) << shift;
            if ((byte & 0x80u) == 0) {
                v = static_cast<std::int32_t>((z >> 1) ^
                                              static_cast<std::uint32_t>(-static_cast<std::int32_t>(z & 1u)));
                return true;
            }
            shift += 7;
        }
        return false;
    }

    inline bool getU8(const std::vector<std::uint8_t>& in, std::size_t& off, std::uint8_t& v) {
        if (off + 1 > in.size())
            return false;
        v = in[off++];
        return true;
    }

    inline bool getU32(const std::vector<std::uint8_t>& in, std::size_t& off, std::uint32_t& v) {
        if (off + 4 > in.size())
            return false;
        v = 0;
        for (int i = 0; i < 4; ++i)
            v |= static_cast<std::uint32_t>(in[off + i]) << (8 * i);
        off += 4;
        return true;
    }

    inline bool getI32(const std::vector<std::uint8_t>& in, std::size_t& off, std::int32_t& v) {
        std::uint32_t u = 0;
        if (!getU32(in, off, u))
            return false;
        v = static_cast<std::int32_t>(u);
        return true;
    }

    inline bool getF32(const std::vector<std::uint8_t>& in, std::size_t& off, float& v) {
        std::uint32_t bits = 0;
        if (!getU32(in, off, bits))
            return false;
        std::memcpy(&v, &bits, sizeof(v));
        return true;
    }

    inline bool getBytes(const std::vector<std::uint8_t>& in, std::size_t& off, std::vector<std::uint8_t>& out) {
        std::uint32_t size = 0;
        if (!getU32(in, off, size))
            return false;
        if (off + size > in.size())
            return false;
        out.assign(in.begin() + static_cast<std::ptrdiff_t>(off), in.begin() + static_cast<std::ptrdiff_t>(off + size));
        off += size;
        return true;
    }

    inline bool getString(const std::vector<std::uint8_t>& in, std::size_t& off, std::string& out) {
        std::vector<std::uint8_t> bytes;
        if (!getBytes(in, off, bytes))
            return false;
        out.assign(bytes.begin(), bytes.end());
        return true;
    }

    inline bool getFixedAxis(const std::vector<std::uint8_t>& in, std::size_t& off,
                             int count, std::vector<std::int32_t>& out) {
        std::int32_t mn = 0;
        std::uint8_t bits = 0;
        std::vector<std::uint8_t> packed;
        if (!getI32(in, off, mn) || !getU8(in, off, bits) || bits == 0 || bits > 32 ||
            !getBytes(in, off, packed)) {
            return false;
        }
        out.resize(static_cast<std::size_t>(count));
        std::uint64_t acc = 0;
        int nbits = 0;
        std::size_t p = 0;
        for (int i = 0; i < count; ++i) {
            while (nbits < bits) {
                if (p >= packed.size()) return false;
                acc |= static_cast<std::uint64_t>(packed[p++]) << nbits;
                nbits += 8;
            }
            const std::uint32_t v = static_cast<std::uint32_t>(acc) &
                                    ((bits == 32) ? 0xffffffffu : ((1u << bits) - 1u));
            out[i] = mn + static_cast<std::int32_t>(v);
            acc >>= bits;
            nbits -= bits;
        }
        return true;
    }


    inline void putMagic(std::vector<std::uint8_t>& out, const char magic[4]) {
        out.insert(out.end(), magic, magic + 4);
        putU32(out, 1);
    }

    inline bool checkMagic(const std::vector<std::uint8_t>& in, std::size_t& off, const char magic[4]) {
        if (off + 8 > in.size())
            return false;
        if (std::memcmp(in.data() + off, magic, 4) != 0)
            return false;
        off += 4;
        std::uint32_t version = 0;
        if (!getU32(in, off, version) || version != 1)
            return false;
        return true;
    }

}  // namespace paper_codec

inline std::vector<std::uint8_t> packPaperPartitionHeader(const PaperPartitionRow& p) {
    using namespace paper_codec;
    std::vector<std::uint8_t> out;
    putMagic(out, "PPTH");
    putI32(out, p.seedV0);
    putI32(out, p.seedV1);
    putI32(out, p.numVertices);
    putI32(out, p.numFaces);
    putI32(out, p.numLods);
    putU8(out, p.enableQuantization ? 1 : 0);
    putI32(out, p.quantBits);
    putF32(out, p.bboxMin[0]);
    putF32(out, p.bboxMin[1]);
    putF32(out, p.bboxMin[2]);
    putF32(out, p.quantStep);
    putString(out, p.baseMeshCodec);
    return out;
}

inline bool unpackPaperPartitionHeader(const std::vector<std::uint8_t>& in, PaperPartitionRow& p, std::string* error = nullptr) {
    using namespace paper_codec;
    std::size_t off = 0;
    if (!checkMagic(in, off, "PPTH")) {
        if (error)
            *error = "partition header: bad magic/version";
        return false;
    }
    std::uint8_t quant = 0;
    if (!getI32(in, off, p.seedV0) || !getI32(in, off, p.seedV1) || !getI32(in, off, p.numVertices) || !getI32(in, off, p.numFaces) ||
        !getI32(in, off, p.numLods) || !getU8(in, off, quant) || !getI32(in, off, p.quantBits) || !getF32(in, off, p.bboxMin[0]) ||
        !getF32(in, off, p.bboxMin[1]) || !getF32(in, off, p.bboxMin[2]) || !getF32(in, off, p.quantStep) || !getString(in, off, p.baseMeshCodec)) {
        if (error)
            *error = "partition header: truncated";
        return false;
    }
    p.enableQuantization = quant != 0;
    return true;
}

inline std::vector<std::uint8_t> packPaperConnectivity(const PaperLodRow& lod) {
    using namespace paper_codec;
    std::vector<std::uint8_t> out;
    putMagic(out, "PCON");
    putI32(out, lod.countF);
    putI32(out, lod.countEt);
    putI32(out, lod.countEf);
    putI32(out, lod.countV);
    putBytes(out, lod.F.data(), lod.F.size());
    putBytes(out, lod.Et.data(), lod.Et.size());
    putBytes(out, lod.Ef.data(), lod.Ef.size());
    putBytes(out, lod.V.data(), lod.V.size());
    return out;
}

inline bool unpackPaperConnectivity(const std::vector<std::uint8_t>& in, PaperLodRow& lod, std::string* error = nullptr) {
    using namespace paper_codec;
    std::size_t off = 0;
    if (!checkMagic(in, off, "PCON")) {
        if (error)
            *error = "connectivity: bad magic/version";
        return false;
    }
    if (!getI32(in, off, lod.countF) || !getI32(in, off, lod.countEt) || !getI32(in, off, lod.countEf) || !getI32(in, off, lod.countV) ||
        !getBytes(in, off, lod.F) || !getBytes(in, off, lod.Et) || !getBytes(in, off, lod.Ef) || !getBytes(in, off, lod.V)) {
        if (error)
            *error = "connectivity: truncated";
        return false;
    }
    return true;
}

inline std::vector<std::uint8_t> packPaperGeometry(const PaperLodRow& lod) {
    using namespace paper_codec;
    std::vector<std::uint8_t> out;
    putMagic(out, "PGEO");
    const bool hasInts = !lod.CfQ.empty() || !lod.CvQ.empty();
    const bool hasFloats = !lod.CfF.empty() || !lod.CvF.empty();
    std::vector<std::uint8_t> qx, qy, qz, rx, ry, rz;
    if (hasInts) {
        // Per-axis first-order delta; zigzag varint.  Cf and Cv are separate streams.
        std::int32_t px = 0, py = 0, pz = 0;
        for (std::size_t i = 0; i + 2 < lod.CfQ.size(); i += 3) {
            putVarintI32(qx, lod.CfQ[i] - px); px = lod.CfQ[i];
            putVarintI32(qy, lod.CfQ[i + 1] - py); py = lod.CfQ[i + 1];
            putVarintI32(qz, lod.CfQ[i + 2] - pz); pz = lod.CfQ[i + 2];
        }
        std::int32_t sx = 0, sy = 0, sz = 0;
        for (std::size_t i = 0; i + 2 < lod.CvQ.size(); i += 3) {
            putVarintI32(rx, lod.CvQ[i] - sx); sx = lod.CvQ[i];
            putVarintI32(ry, lod.CvQ[i + 1] - sy); sy = lod.CvQ[i + 1];
            putVarintI32(rz, lod.CvQ[i + 2] - sz); sz = lod.CvQ[i + 2];
        }
    }
    const std::uint8_t flags = static_cast<std::uint8_t>((hasInts ? 1 : 0) |
                                                          (hasFloats ? 2 : 0) |
                                                          8);
    putU8(out, flags);
    putI32(out, lod.countCf);
    putI32(out, lod.countCv);
    putBytes(out, qx.data(), qx.size());
    putBytes(out, qy.data(), qy.size());
    putBytes(out, qz.data(), qz.size());
    putBytes(out, rx.data(), rx.size());
    putBytes(out, ry.data(), ry.size());
    putBytes(out, rz.data(), rz.size());
    putBytes(out, lod.CfF.data(), lod.CfF.size() * sizeof(float));
    putBytes(out, lod.CvF.data(), lod.CvF.size() * sizeof(float));
    return out;
}

inline bool unpackPaperGeometry(const std::vector<std::uint8_t>& in, PaperLodRow& lod, std::string* error = nullptr) {
    using namespace paper_codec;
    std::size_t off = 0;
    if (!checkMagic(in, off, "PGEO")) {
        if (error)
            *error = "geometry: bad magic/version";
        return false;
    }
    std::uint8_t flags = 0;
    if (!getU8(in, off, flags) || !getI32(in, off, lod.countCf) || !getI32(in, off, lod.countCv)) {
        if (error)
            *error = "geometry: truncated header";
        return false;
    }
    auto unpackFloats = [](const std::vector<std::uint8_t>& bytes) {
        std::vector<float> out(bytes.size() / sizeof(float));
        if (!out.empty())
            std::memcpy(out.data(), bytes.data(), out.size() * sizeof(float));
        return out;
    };
    if (flags & 8) {
        std::vector<std::uint8_t> fx, fy;
        if (flags & 32) {
            std::vector<std::uint8_t> fixed;
            if (!getBytes(in, off, fixed) || !getBytes(in, off, fx) || !getBytes(in, off, fy)) {
                if (error) *error = "geometry: truncated fixed payload";
                return false;
            }
            std::size_t foff = 0;
            std::vector<std::int32_t> cfx, cfy, cfz, cvx, cvy, cvz;
            if (!getFixedAxis(fixed, foff, lod.countCf, cfx) ||
                !getFixedAxis(fixed, foff, lod.countCf, cfy) ||
                !getFixedAxis(fixed, foff, lod.countCf, cfz) ||
                !getFixedAxis(fixed, foff, lod.countCv, cvx) ||
                !getFixedAxis(fixed, foff, lod.countCv, cvy) ||
                !getFixedAxis(fixed, foff, lod.countCv, cvz)) {
                if (error) *error = "geometry: bad fixed axis stream";
                return false;
            }
            lod.CfQ.resize(static_cast<std::size_t>(lod.countCf) * 3);
            lod.CvQ.resize(static_cast<std::size_t>(lod.countCv) * 3);
            for (int i = 0; i < lod.countCf; ++i) {
                lod.CfQ[3 * i] = cfx[i]; lod.CfQ[3 * i + 1] = cfy[i]; lod.CfQ[3 * i + 2] = cfz[i];
            }
            for (int i = 0; i < lod.countCv; ++i) {
                lod.CvQ[3 * i] = cvx[i]; lod.CvQ[3 * i + 1] = cvy[i]; lod.CvQ[3 * i + 2] = cvz[i];
            }
            lod.CfF = unpackFloats(fx);
            lod.CvF = unpackFloats(fy);
            return true;
        }
        std::vector<std::uint8_t> qx, qy, qz, rx, ry, rz;
        if (!getBytes(in, off, qx) || !getBytes(in, off, qy) || !getBytes(in, off, qz) ||
            !getBytes(in, off, rx) || !getBytes(in, off, ry) || !getBytes(in, off, rz) ||
            !getBytes(in, off, fx) || !getBytes(in, off, fy)) {
            if (error) *error = "geometry: truncated payload";
            return false;
        }
        const bool secondOrder = (flags & 16) != 0;
        auto decodeDelta = [&](const std::vector<std::uint8_t>& bytes, int count,
                               std::vector<std::int32_t>& out) {
            out.clear();
            out.reserve(static_cast<std::size_t>(count));
            std::size_t o = 0;
            std::int32_t prev = 0;
            std::int32_t prev2 = 0;
            for (int i = 0; i < count; ++i) {
                std::int32_t d = 0;
                if (!paper_codec::getVarintI32(bytes, o, d)) return false;
                const std::int32_t v = secondOrder ? (d + 2 * prev - prev2) : (prev + d);
                prev2 = prev;
                prev = v;
                out.push_back(v);
            }
            return true;
        };
        lod.CfF = unpackFloats(fx);
        lod.CvF = unpackFloats(fy);
        if ((flags & 1) == 0) return true;
        std::vector<std::int32_t> cfx, cfy, cfz, cvx, cvy, cvz;
        if (!decodeDelta(qx, lod.countCf, cfx) || !decodeDelta(qy, lod.countCf, cfy) ||
            !decodeDelta(qz, lod.countCf, cfz) || !decodeDelta(rx, lod.countCv, cvx) ||
            !decodeDelta(ry, lod.countCv, cvy) || !decodeDelta(rz, lod.countCv, cvz)) {
            if (error) *error = "geometry: bad delta varint";
            return false;
        }
        lod.CfQ.resize(static_cast<std::size_t>(lod.countCf) * 3);
        lod.CvQ.resize(static_cast<std::size_t>(lod.countCv) * 3);
        for (int i = 0; i < lod.countCf; ++i) {
            lod.CfQ[3 * i] = cfx[i];
            lod.CfQ[3 * i + 1] = cfy[i];
            lod.CfQ[3 * i + 2] = cfz[i];
        }
        for (int i = 0; i < lod.countCv; ++i) {
            lod.CvQ[3 * i] = cvx[i];
            lod.CvQ[3 * i + 1] = cvy[i];
            lod.CvQ[3 * i + 2] = cvz[i];
        }
        return true;
    }
    // Legacy geometry layout: 2 int streams + 2 float streams.
    std::vector<std::uint8_t> qx, qy, fx, fy;
    if (!getBytes(in, off, qx) || !getBytes(in, off, qy) || !getBytes(in, off, fx) || !getBytes(in, off, fy)) {
        if (error) *error = "geometry: truncated legacy payload";
        return false;
    }
    auto unpackInts = [](const std::vector<std::uint8_t>& bytes) {
        std::vector<std::int32_t> out(bytes.size() / sizeof(std::int32_t));
        if (!out.empty()) std::memcpy(out.data(), bytes.data(), out.size() * sizeof(std::int32_t));
        return out;
    };
    auto unpackVarints = [](const std::vector<std::uint8_t>& bytes) {
        std::vector<std::int32_t> out;
        std::size_t o = 0;
        while (o < bytes.size()) {
            std::int32_t v = 0;
            if (!paper_codec::getVarintI32(bytes, o, v)) return std::vector<std::int32_t>{};
            out.push_back(v);
        }
        return out;
    };
    if (flags & 4) {
        lod.CfQ = unpackVarints(qx);
        lod.CvQ = unpackVarints(qy);
    } else {
        lod.CfQ = unpackInts(qx);
        lod.CvQ = unpackInts(qy);
    }
    lod.CfF = unpackFloats(fx);
    lod.CvF = unpackFloats(fy);
    return true;
}

inline std::vector<std::uint8_t> packPaperExtensions(const PaperLodRow& lod) {
    using namespace paper_codec;
    std::vector<std::uint8_t> out;
    putMagic(out, "PEXT");
    putU32(out, static_cast<std::uint32_t>(lod.extension.size()));
    for (const auto& [name, bytes] : lod.extension) {
        putString(out, name);
        putBytes(out, bytes.data(), bytes.size());
    }
    return out;
}

inline bool unpackPaperExtensions(const std::vector<std::uint8_t>& in, PaperLodRow& lod, std::string* error = nullptr) {
    using namespace paper_codec;
    std::size_t off = 0;
    if (!checkMagic(in, off, "PEXT")) {
        if (error)
            *error = "extensions: bad magic/version";
        return false;
    }
    std::uint32_t count = 0;
    if (!getU32(in, off, count)) {
        if (error)
            *error = "extensions: truncated count";
        return false;
    }
    lod.extension.clear();
    for (std::uint32_t i = 0; i < count; ++i) {
        std::string name;
        std::vector<std::uint8_t> bytes;
        if (!getString(in, off, name) || !getBytes(in, off, bytes)) {
            if (error)
                *error = "extensions: truncated entry";
            return false;
        }
        lod.extension.emplace(std::move(name), std::move(bytes));
    }
    return true;
}

inline std::vector<std::uint8_t> packPaperBoundary(const PaperBoundaryRow& b) {
    using namespace paper_codec;
    std::vector<std::uint8_t> out;
    putMagic(out, "PBDY");
    putI32(out, b.partitionA);
    putI32(out, b.partitionB);
    putU32(out, static_cast<std::uint32_t>(b.sharedVertices.size()));
    for (std::int32_t v : b.sharedVertices)
        putI32(out, v);
    putU32(out, static_cast<std::uint32_t>(b.sharedEdges.size()));
    for (std::int32_t v : b.sharedEdges)
        putI32(out, v);
    putU32(out, static_cast<std::uint32_t>(b.stopVertices.size()));
    for (std::int32_t v : b.stopVertices)
        putI32(out, v);
    return out;
}

inline bool unpackPaperBoundary(const std::vector<std::uint8_t>& in, PaperBoundaryRow& b, std::string* error = nullptr) {
    using namespace paper_codec;
    std::size_t off = 0;
    if (!checkMagic(in, off, "PBDY")) {
        if (error)
            *error = "boundary: bad magic/version";
        return false;
    }
    if (!getI32(in, off, b.partitionA) || !getI32(in, off, b.partitionB)) {
        if (error)
            *error = "boundary: truncated header";
        return false;
    }
    auto readInts = [&](std::vector<std::int32_t>& out) {
        std::uint32_t count = 0;
        if (!getU32(in, off, count))
            return false;
        out.resize(count);
        for (std::uint32_t i = 0; i < count; ++i) {
            if (!getI32(in, off, out[i]))
                return false;
        }
        return true;
    };
    if (!readInts(b.sharedVertices) || !readInts(b.sharedEdges) || !readInts(b.stopVertices)) {
        if (error)
            *error = "boundary: truncated arrays";
        return false;
    }
    return true;
}

inline std::vector<std::uint8_t> packPaperLegacyBundle(
    const std::map<std::string, std::vector<std::uint8_t>>& files) {
    using namespace paper_codec;
    std::vector<std::uint8_t> out;
    putMagic(out, "PLEG");
    putU32(out, static_cast<std::uint32_t>(files.size()));
    for (const auto& [name, bytes] : files) {
        putString(out, name);
        putBytes(out, bytes.data(), bytes.size());
    }
    return out;
}

inline bool unpackPaperLegacyBundle(
    const std::vector<std::uint8_t>& in,
    std::map<std::string, std::vector<std::uint8_t>>& files,
    std::string* error = nullptr) {
    using namespace paper_codec;
    std::size_t off = 0;
    if (!checkMagic(in, off, "PLEG")) {
        if (error) *error = "legacy bundle: bad magic/version";
        return false;
    }
    std::uint32_t count = 0;
    if (!getU32(in, off, count)) {
        if (error) *error = "legacy bundle: truncated count";
        return false;
    }
    files.clear();
    for (std::uint32_t i = 0; i < count; ++i) {
        std::string name;
        std::vector<std::uint8_t> bytes;
        if (!getString(in, off, name) || !getBytes(in, off, bytes)) {
            if (error) *error = "legacy bundle: truncated entry";
            return false;
        }
        files.emplace(std::move(name), std::move(bytes));
    }
    return true;
}

}  // namespace pmsf

#endif  // PMSF_STORAGE_PAPER_STREAM_COMPACT_CODEC_H
