/*
 * @file PaperStreamCodecV2.h
 * @brief PaperStream v2 additive codec bundles (M4-1).
 *
 * ADDITIVE ONLY: the v1 bundles in storage/PaperStreamCompactCodec.h are kept
 * untouched (PLEG/PPTH v1/PCON/PGEO/PEXT/PBDY), because the M2b bridge
 * writer/reader still depend on them. This header adds NEW functions with the
 * `V2` suffix:
 *   - PPDL  native payload byte-range directory (conn-round frame)
 *   - PNPL  native payload slice/blob (self-describing frame/scope/offset/len)
 *   - PRND  typed per-round TLV bundle (mirrors CanonicalEncodedStream shape
 *           without depending on core/Symbol.h)
 *   - PPTH v2 partition header (v1 PPTH + payload directory + CONN header)
 *   - PGEO v2 strict reader for the axis-delta layout (the v1 reader stays as
 *          is; v2 rejects the legacy/fixed-axis/second-order layouts that never
 *          had an encoder)
 *
 * Wire-format details are in docs/memory/PHASE2_DESIGN.md §3.4.
 */
#ifndef PMSF_STORAGE_PAPER_STREAM_CODEC_V2_H
#define PMSF_STORAGE_PAPER_STREAM_CODEC_V2_H

#include "storage/PaperStreamCompactCodec.h"
#include "storage/PaperStreamTable.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace pmsf {

// v2-only byte helpers. The v1 codec header is kept untouched (no u64 and no
// version-parameterized magic helpers there), so the additive v2 codec brings
// its own helpers instead of modifying it.
namespace paper_codec_v2 {

inline void putU64(std::vector<std::uint8_t>& out, std::uint64_t v) {
    for (int i = 0; i < 8; ++i)
        out.push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xffu));
}

inline bool getU64(const std::vector<std::uint8_t>& in, std::size_t& off, std::uint64_t& v) {
    if (off + 8 > in.size())
        return false;
    v = 0;
    for (int i = 0; i < 8; ++i)
        v |= static_cast<std::uint64_t>(in[off + i]) << (8 * i);
    off += 8;
    return true;
}

inline void putMagicVersion(std::vector<std::uint8_t>& out, const char magic[4],
                            std::uint32_t version) {
    out.insert(out.end(), magic, magic + 4);
    paper_codec::putU32(out, version);
}

inline bool checkMagicVersion(const std::vector<std::uint8_t>& in, std::size_t& off,
                              const char magic[4], std::uint32_t version) {
    if (off + 8 > in.size())
        return false;
    if (std::memcmp(in.data() + off, magic, 4) != 0)
        return false;
    off += 4;
    std::uint32_t stored = 0;
    if (!paper_codec::getU32(in, off, stored) || stored != version)
        return false;
    return true;
}

}  // namespace paper_codec_v2

/// @brief PPTH container version for the v2 partition header.
inline constexpr std::uint32_t kPaperPartitionHeaderV2Version = 2;

// ---------------------------------------------------------------------------
// PPDL: native payload byte-range directory (conn-round frame)
// ---------------------------------------------------------------------------
// Layout: "PPDL" u32 version=1 | u32 count
//         x count: i32 lodId | i32 streamId | u64 offset | u64 length
inline std::vector<std::uint8_t> packPaperPayloadDirectoryV2(
    const std::vector<PaperPayloadSlice>& slices) {
    using namespace paper_codec;
    using namespace paper_codec_v2;
    std::vector<std::uint8_t> out;
    putMagic(out, "PPDL");
    putU32(out, static_cast<std::uint32_t>(slices.size()));
    for (const auto& slice : slices) {
        putI32(out, slice.lodId);
        putI32(out, slice.streamId);
        putU64(out, slice.offset);
        putU64(out, slice.length);
    }
    return out;
}

inline bool unpackPaperPayloadDirectoryV2(const std::vector<std::uint8_t>& in,
                                          std::vector<PaperPayloadSlice>& slices,
                                          std::string* error = nullptr) {
    using namespace paper_codec;
    using namespace paper_codec_v2;
    slices.clear();
    std::size_t off = 0;
    if (!checkMagic(in, off, "PPDL")) {
        if (error) *error = "payload directory: bad magic/version";
        return false;
    }
    std::uint32_t count = 0;
    if (!getU32(in, off, count)) {
        if (error) *error = "payload directory: truncated count";
        return false;
    }
    if (count > (in.size() - off) / 24) {
        if (error) *error = "payload directory: count exceeds payload";
        return false;
    }
    slices.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        PaperPayloadSlice slice;
        if (!getI32(in, off, slice.lodId) || !getI32(in, off, slice.streamId) ||
            !getU64(in, off, slice.offset) || !getU64(in, off, slice.length)) {
            if (error) *error = "payload directory: truncated entry";
            slices.clear();
            return false;
        }
        slices.push_back(slice);
    }
    if (off != in.size()) {
        if (error) *error = "payload directory: trailing bytes";
        slices.clear();
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// PNPL: native payload slice / blob
// ---------------------------------------------------------------------------
// Layout: "PNPL" u32 version=1 | u8 scope | u8 frame
//         | u64 payloadByteOffset | u64 payloadByteLength | u32 len | bytes
inline std::vector<std::uint8_t> packPaperNativePayloadV2(
    const std::vector<std::uint8_t>& payload, PaperPayloadScope scope,
    PaperNativeFrame frame, std::uint64_t payloadOffset, std::uint64_t payloadLength) {
    using namespace paper_codec;
    using namespace paper_codec_v2;
    std::vector<std::uint8_t> out;
    putMagic(out, "PNPL");
    putU8(out, static_cast<std::uint8_t>(scope));
    putU8(out, static_cast<std::uint8_t>(frame));
    putU64(out, payloadOffset);
    putU64(out, payloadLength);
    putBytes(out, payload.data(), payload.size());
    return out;
}

inline bool unpackPaperNativePayloadV2(const std::vector<std::uint8_t>& in,
                                       std::vector<std::uint8_t>& payload,
                                       PaperPayloadScope& scope, PaperNativeFrame& frame,
                                       std::uint64_t& payloadOffset,
                                       std::uint64_t& payloadLength,
                                       std::string* error = nullptr) {
    using namespace paper_codec;
    using namespace paper_codec_v2;
    payload.clear();
    std::size_t off = 0;
    if (!checkMagic(in, off, "PNPL")) {
        if (error) *error = "native payload: bad magic/version";
        return false;
    }
    std::uint8_t scopeRaw = 0;
    std::uint8_t frameRaw = 0;
    if (!getU8(in, off, scopeRaw) || !getU8(in, off, frameRaw) ||
        !getU64(in, off, payloadOffset) || !getU64(in, off, payloadLength) ||
        !getBytes(in, off, payload)) {
        if (error) *error = "native payload: truncated header/body";
        return false;
    }
    if (scopeRaw > static_cast<std::uint8_t>(PaperPayloadScope::Object)) {
        if (error) *error = "native payload: unknown scope";
        return false;
    }
    if (frameRaw > static_cast<std::uint8_t>(PaperNativeFrame::NamedPayload)) {
        if (error) *error = "native payload: unknown frame";
        return false;
    }
    scope = static_cast<PaperPayloadScope>(scopeRaw);
    frame = static_cast<PaperNativeFrame>(frameRaw);
    if (payloadLength != 0 && payload.size() != payloadLength) {
        if (error) *error = "native payload: byte count != payloadByteLength";
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// PRND: typed per-round bundle
// ---------------------------------------------------------------------------
// Layout: "PRND" u32 version=1 | i32 roundId
//   u32 numScalarStreams
//     x: u8 relation | u8 cellType | u16 nameLen | name
//        | u32 symbolCount | u32 alphabetBits | u32 byteLen | bytes
//   u32 numGeometry
//     x: u8 coordFormat | u32 pointCount | u32 vertexPointCount
//        | u32 pointLen | pointBytes | u32 vertexLen | vertexBytes
//   u32 numSideChannels
//     x: u8 cellType | u8 shape | u16 nameLen | name
//        | u32 cellCount | u32 alphabetBits | u32 byteLen | bytes
struct PaperRoundScalarStreamV2 {
    std::string name;
    PaperStreamRelation relation = PaperStreamRelation::Connectivity;
    PaperCellType cellType = PaperCellType::Int32;
    std::int32_t symbolCount = 0;
    std::int32_t alphabetBits = 0;
    std::vector<std::uint8_t> bytes;
};

struct PaperRoundGeometryV2 {
    std::int32_t pointCount = 0;
    std::int32_t vertexPointCount = 0;
    std::int8_t coordFormat = 0;
    std::vector<std::uint8_t> pointBytes;
    std::vector<std::uint8_t> vertexPointBytes;
};

struct PaperRoundSideChannelV2 {
    std::string name;
    PaperCellType cellType = PaperCellType::Int32;
    PaperCellShape shape = PaperCellShape::Scalar;
    std::int32_t cellCount = 0;
    std::int32_t alphabetBits = 0;
    std::vector<std::uint8_t> bytes;
};

struct PaperRoundBundleV2 {
    std::int32_t roundId = 0;
    std::vector<PaperRoundScalarStreamV2> scalarStreams;
    std::vector<PaperRoundGeometryV2> geometry;
    std::vector<PaperRoundSideChannelV2> sideChannels;
};

inline std::vector<std::uint8_t> packPaperRoundBundleV2(const PaperRoundBundleV2& bundle) {
    using namespace paper_codec;
    if (bundle.roundId < 0)
        throw std::runtime_error("PaperStream V2 PRND: negative roundId");
    std::set<std::string> seenNames;
    std::vector<std::uint8_t> out;
    putMagic(out, "PRND");
    putI32(out, bundle.roundId);
    putU32(out, static_cast<std::uint32_t>(bundle.scalarStreams.size()));
    for (const auto& stream : bundle.scalarStreams) {
        if (stream.name.empty() || stream.name.size() > 0xffffu)
            throw std::runtime_error("PaperStream V2 PRND: bad scalar name");
        if (!seenNames.insert(stream.name).second)
            throw std::runtime_error("PaperStream V2 PRND: duplicate name " + stream.name);
        if (static_cast<std::uint8_t>(stream.relation) >
                static_cast<std::uint8_t>(PaperStreamRelation::Partition) ||
            static_cast<std::uint8_t>(stream.cellType) >
                static_cast<std::uint8_t>(PaperCellType::Float))
            throw std::runtime_error("PaperStream V2 PRND: unknown scalar relation/cell");
        if (stream.symbolCount < 0 || stream.alphabetBits < 0)
            throw std::runtime_error("PaperStream V2 PRND: negative scalar count");
        putU8(out, static_cast<std::uint8_t>(stream.relation));
        putU8(out, static_cast<std::uint8_t>(stream.cellType));
        out.push_back(static_cast<std::uint8_t>(stream.name.size() & 0xffu));
        out.push_back(static_cast<std::uint8_t>((stream.name.size() >> 8) & 0xffu));
        out.insert(out.end(), stream.name.begin(), stream.name.end());
        putU32(out, static_cast<std::uint32_t>(stream.symbolCount));
        putU32(out, static_cast<std::uint32_t>(stream.alphabetBits));
        putBytes(out, stream.bytes.data(), stream.bytes.size());
    }
    putU32(out, static_cast<std::uint32_t>(bundle.geometry.size()));
    for (const auto& geometry : bundle.geometry) {
        if (geometry.pointCount < 0 || geometry.vertexPointCount < 0)
            throw std::runtime_error("PaperStream V2 PRND: negative geometry count");
        if (geometry.pointBytes.size() % 3 != 0 ||
            geometry.vertexPointBytes.size() % 3 != 0)
            throw std::runtime_error("PaperStream V2 PRND: geometry byte stream not xyz-aligned");
        putU8(out, static_cast<std::uint8_t>(geometry.coordFormat));
        putU32(out, static_cast<std::uint32_t>(geometry.pointCount));
        putU32(out, static_cast<std::uint32_t>(geometry.vertexPointCount));
        putBytes(out, geometry.pointBytes.data(), geometry.pointBytes.size());
        putBytes(out, geometry.vertexPointBytes.data(), geometry.vertexPointBytes.size());
    }
    putU32(out, static_cast<std::uint32_t>(bundle.sideChannels.size()));
    for (const auto& channel : bundle.sideChannels) {
        if (channel.name.empty() || channel.name.size() > 0xffffu)
            throw std::runtime_error("PaperStream V2 PRND: bad side name");
        if (!seenNames.insert(channel.name).second)
            throw std::runtime_error("PaperStream V2 PRND: duplicate name " + channel.name);
        if (static_cast<std::uint8_t>(channel.cellType) >
                static_cast<std::uint8_t>(PaperCellType::Float) ||
            static_cast<std::uint8_t>(channel.shape) >
                static_cast<std::uint8_t>(PaperCellShape::Vec3))
            throw std::runtime_error("PaperStream V2 PRND: unknown side cell/shape");
        if (channel.cellCount < 0 || channel.alphabetBits < 0)
            throw std::runtime_error("PaperStream V2 PRND: negative side count");
        putU8(out, static_cast<std::uint8_t>(channel.cellType));
        putU8(out, static_cast<std::uint8_t>(channel.shape));
        out.push_back(static_cast<std::uint8_t>(channel.name.size() & 0xffu));
        out.push_back(static_cast<std::uint8_t>((channel.name.size() >> 8) & 0xffu));
        out.insert(out.end(), channel.name.begin(), channel.name.end());
        putU32(out, static_cast<std::uint32_t>(channel.cellCount));
        putU32(out, static_cast<std::uint32_t>(channel.alphabetBits));
        putBytes(out, channel.bytes.data(), channel.bytes.size());
    }
    return out;
}

inline bool unpackPaperRoundBundleV2(const std::vector<std::uint8_t>& in,
                                     PaperRoundBundleV2& bundle,
                                     std::string* error = nullptr) {
    using namespace paper_codec;
    bundle = PaperRoundBundleV2{};
    std::size_t off = 0;
    if (!checkMagic(in, off, "PRND")) {
        if (error) *error = "round bundle: bad magic/version";
        return false;
    }
    std::uint32_t numScalar = 0;
    if (!getI32(in, off, bundle.roundId) || !getU32(in, off, numScalar)) {
        if (error) *error = "round bundle: truncated header";
        return false;
    }
    if (bundle.roundId < 0) {
        if (error) *error = "round bundle: negative roundId";
        return false;
    }
    auto readName = [&](std::string& name) {
        if (off + 2 > in.size()) return false;
        const std::size_t len = static_cast<std::size_t>(in[off]) |
                                (static_cast<std::size_t>(in[off + 1]) << 8);
        off += 2;
        if (len > in.size() - off) return false;
        name.assign(in.begin() + static_cast<std::ptrdiff_t>(off),
                    in.begin() + static_cast<std::ptrdiff_t>(off + len));
        off += len;
        return true;
    };
    if (numScalar > (in.size() - off) / 16) {
        if (error) *error = "round bundle: scalar count exceeds payload";
        return false;
    }
    std::vector<std::string> seenNames;
    bundle.scalarStreams.reserve(numScalar);
    for (std::uint32_t i = 0; i < numScalar; ++i) {
        std::uint8_t relationRaw = 0;
        std::uint8_t cellRaw = 0;
        if (!getU8(in, off, relationRaw) || !getU8(in, off, cellRaw)) {
            if (error) *error = "round bundle: truncated scalar header";
            return false;
        }
        if (relationRaw > static_cast<std::uint8_t>(PaperStreamRelation::Partition) ||
            cellRaw > static_cast<std::uint8_t>(PaperCellType::Float)) {
            if (error) *error = "round bundle: unknown scalar relation/cell";
            return false;
        }
        PaperRoundScalarStreamV2 stream;
        stream.relation = static_cast<PaperStreamRelation>(relationRaw);
        stream.cellType = static_cast<PaperCellType>(cellRaw);
        if (!readName(stream.name) || stream.name.empty()) {
            if (error) *error = "round bundle: bad scalar stream name";
            return false;
        }
        if (std::find(seenNames.begin(), seenNames.end(), stream.name) != seenNames.end()) {
            if (error) *error = "round bundle: duplicate stream name";
            return false;
        }
        seenNames.push_back(stream.name);
        std::uint32_t symbolCount = 0;
        std::uint32_t alphabetBits = 0;
        if (!getU32(in, off, symbolCount) || !getU32(in, off, alphabetBits) ||
            !getBytes(in, off, stream.bytes)) {
            if (error) *error = "round bundle: truncated scalar body";
            return false;
        }
        stream.symbolCount = static_cast<std::int32_t>(symbolCount);
        stream.alphabetBits = static_cast<std::int32_t>(alphabetBits);
        bundle.scalarStreams.push_back(std::move(stream));
    }
    std::uint32_t numGeometry = 0;
    if (!getU32(in, off, numGeometry) || numGeometry > (in.size() - off) / 12) {
        if (error) *error = "round bundle: bad geometry count";
        return false;
    }
    bundle.geometry.reserve(numGeometry);
    for (std::uint32_t i = 0; i < numGeometry; ++i) {
        std::uint8_t coordFormat = 0;
        std::uint32_t pointCount = 0;
        std::uint32_t vertexPointCount = 0;
        if (!getU8(in, off, coordFormat) || !getU32(in, off, pointCount) ||
            !getU32(in, off, vertexPointCount)) {
            if (error) *error = "round bundle: truncated geometry header";
            return false;
        }
        PaperRoundGeometryV2 geometry;
        geometry.coordFormat = static_cast<std::int8_t>(coordFormat);
        geometry.pointCount = static_cast<std::int32_t>(pointCount);
        geometry.vertexPointCount = static_cast<std::int32_t>(vertexPointCount);
        if (!getBytes(in, off, geometry.pointBytes) ||
            !getBytes(in, off, geometry.vertexPointBytes)) {
            if (error) *error = "round bundle: truncated geometry body";
            return false;
        }
        bundle.geometry.push_back(std::move(geometry));
    }
    std::uint32_t numSide = 0;
    if (!getU32(in, off, numSide) || numSide > (in.size() - off) / 16) {
        if (error) *error = "round bundle: bad side-channel count";
        return false;
    }
    bundle.sideChannels.reserve(numSide);
    for (std::uint32_t i = 0; i < numSide; ++i) {
        std::uint8_t cellRaw = 0;
        std::uint8_t shapeRaw = 0;
        if (!getU8(in, off, cellRaw) || !getU8(in, off, shapeRaw)) {
            if (error) *error = "round bundle: truncated side header";
            return false;
        }
        if (cellRaw > static_cast<std::uint8_t>(PaperCellType::Float) ||
            shapeRaw > static_cast<std::uint8_t>(PaperCellShape::Vec3)) {
            if (error) *error = "round bundle: unknown side cell/shape";
            return false;
        }
        PaperRoundSideChannelV2 channel;
        channel.cellType = static_cast<PaperCellType>(cellRaw);
        channel.shape = static_cast<PaperCellShape>(shapeRaw);
        if (!readName(channel.name) || channel.name.empty()) {
            if (error) *error = "round bundle: bad side-channel name";
            return false;
        }
        if (std::find(seenNames.begin(), seenNames.end(), channel.name) != seenNames.end()) {
            if (error) *error = "round bundle: duplicate side-channel name";
            return false;
        }
        seenNames.push_back(channel.name);
        std::uint32_t cellCount = 0;
        std::uint32_t alphabetBits = 0;
        if (!getU32(in, off, cellCount) || !getU32(in, off, alphabetBits) ||
            !getBytes(in, off, channel.bytes)) {
            if (error) *error = "round bundle: truncated side body";
            return false;
        }
        channel.cellCount = static_cast<std::int32_t>(cellCount);
        channel.alphabetBits = static_cast<std::int32_t>(alphabetBits);
        bundle.sideChannels.push_back(std::move(channel));
    }
    if (off != in.size()) {
        if (error) *error = "round bundle: trailing bytes";
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// PPTH v2: partition header (v1 fields + payload directory + CONN header copy)
// ---------------------------------------------------------------------------
inline std::vector<std::uint8_t> packPaperPartitionHeaderV2(
    const PaperPartitionRow& p) {
    using namespace paper_codec;
    using namespace paper_codec_v2;
    std::vector<std::uint8_t> out;
    putMagicVersion(out, "PPTH", kPaperPartitionHeaderV2Version);
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
    const std::vector<std::uint8_t> directory =
        packPaperPayloadDirectoryV2(p.payloadDirectory);
    putBytes(out, directory.data(), directory.size());
    putBytes(out, p.connHeader.data(), p.connHeader.size());
    return out;
}

inline bool unpackPaperPartitionHeaderV2(const std::vector<std::uint8_t>& in,
                                         PaperPartitionRow& p,
                                         std::string* error = nullptr) {
    using namespace paper_codec;
    using namespace paper_codec_v2;
    std::size_t off = 0;
    if (off + 8 > in.size()) {
        if (error) *error = "partition header v2: truncated";
        return false;
    }
    std::uint32_t storedVersion = 0;
    {
        std::size_t versionOff = 4;
        if (!getU32(in, versionOff, storedVersion)) {
            if (error) *error = "partition header v2: truncated version";
            return false;
        }
    }
    if (storedVersion != kPaperPartitionHeaderV2Version) {
        if (error) {
            *error = "partition header v2: unsupported version " +
                     std::to_string(storedVersion) +
                     (storedVersion == 1
                          ? " (v1 PPTH is not a v2 artifact; recompress with paper v2)"
                          : "");
        }
        return false;
    }
    if (!checkMagicVersion(in, off, "PPTH", kPaperPartitionHeaderV2Version)) {
        if (error) *error = "partition header v2: bad magic/version";
        return false;
    }
    std::uint8_t quant = 0;
    std::vector<std::uint8_t> directoryBytes;
    if (!getI32(in, off, p.seedV0) || !getI32(in, off, p.seedV1) ||
        !getI32(in, off, p.numVertices) || !getI32(in, off, p.numFaces) ||
        !getI32(in, off, p.numLods) || !getU8(in, off, quant) ||
        !getI32(in, off, p.quantBits) || !getF32(in, off, p.bboxMin[0]) ||
        !getF32(in, off, p.bboxMin[1]) || !getF32(in, off, p.bboxMin[2]) ||
        !getF32(in, off, p.quantStep) || !getString(in, off, p.baseMeshCodec) ||
        !getBytes(in, off, directoryBytes) || !getBytes(in, off, p.connHeader)) {
        if (error) *error = "partition header v2: truncated body";
        return false;
    }
    p.enableQuantization = quant != 0;
    if (!unpackPaperPayloadDirectoryV2(directoryBytes, p.payloadDirectory, error)) {
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// PGEO v2: strict axis-delta geometry (v1 writer format; v2 reader only)
// ---------------------------------------------------------------------------
// flags bit0 = quantized int32 residuals present
//       bit1 = float32 coordinates present
//       bit3 = REQUIRED axis-delta varint layout (the only encoder v1/v2 has).
inline std::vector<std::uint8_t> packPaperGeometryV2(const PaperLodRow& lod) {
    using namespace paper_codec;
    std::vector<std::uint8_t> out;
    putMagic(out, "PGEO");
    const bool hasInts = !lod.CfQ.empty() || !lod.CvQ.empty();
    const bool hasFloats = !lod.CfF.empty() || !lod.CvF.empty();
    std::vector<std::uint8_t> qx, qy, qz, rx, ry, rz;
    if (hasInts) {
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
                                                          (hasFloats ? 2 : 0) | 8);
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

inline bool unpackPaperGeometryV2(const std::vector<std::uint8_t>& in, PaperLodRow& lod,
                                  std::string* error = nullptr) {
    using namespace paper_codec;
    std::size_t off = 0;
    if (!checkMagic(in, off, "PGEO")) {
        if (error) *error = "geometry v2: bad magic/version";
        return false;
    }
    std::uint8_t flags = 0;
    if (!getU8(in, off, flags) || !getI32(in, off, lod.countCf) ||
        !getI32(in, off, lod.countCv)) {
        if (error) *error = "geometry v2: truncated header";
        return false;
    }
    if ((flags & 8u) == 0) {
        if (error) {
            *error = "geometry v2: legacy layout (flag bit3 clear) is unsupported";
        }
        return false;
    }
    if ((flags & ~8u & ~3u) != 0) {
        if (error) *error = "geometry v2: unknown layout flags " +
                            std::to_string(static_cast<int>(flags));
        return false;
    }
    if (lod.countCf < 0 || lod.countCv < 0) {
        if (error) *error = "geometry v2: negative count";
        return false;
    }
    auto unpackFloats = [](const std::vector<std::uint8_t>& bytes,
                           std::vector<float>& out) {
        if (bytes.size() % sizeof(float) != 0) return false;
        out.resize(bytes.size() / sizeof(float));
        if (!out.empty()) std::memcpy(out.data(), bytes.data(), out.size() * sizeof(float));
        return true;
    };
    std::vector<std::uint8_t> qx, qy, qz, rx, ry, rz, fx, fy;
    if (!getBytes(in, off, qx) || !getBytes(in, off, qy) || !getBytes(in, off, qz) ||
        !getBytes(in, off, rx) || !getBytes(in, off, ry) || !getBytes(in, off, rz) ||
        !getBytes(in, off, fx) || !getBytes(in, off, fy)) {
        if (error) *error = "geometry v2: truncated payload";
        return false;
    }
    if (off != in.size()) {
        if (error) *error = "geometry v2: trailing bytes";
        return false;
    }
    if (!unpackFloats(fx, lod.CfF) || !unpackFloats(fy, lod.CvF)) {
        if (error) *error = "geometry v2: float stream not multiple of 4 bytes";
        return false;
    }
    if ((flags & 1u) == 0) {
        lod.CfQ.clear();
        lod.CvQ.clear();
        return true;
    }
    auto decodeDelta = [&](const std::vector<std::uint8_t>& bytes, int count,
                           std::vector<std::int32_t>& out) {
        out.clear();
        out.reserve(static_cast<std::size_t>(count));
        std::size_t o = 0;
        std::int32_t prev = 0;
        for (int i = 0; i < count; ++i) {
            std::int32_t d = 0;
            if (!paper_codec::getVarintI32(bytes, o, d)) return false;
            const std::int32_t v = prev + d;
            prev = v;
            out.push_back(v);
        }
        return o == bytes.size();
    };
    std::vector<std::int32_t> cfx, cfy, cfz, cvx, cvy, cvz;
    if (!decodeDelta(qx, lod.countCf, cfx) || !decodeDelta(qy, lod.countCf, cfy) ||
        !decodeDelta(qz, lod.countCf, cfz) || !decodeDelta(rx, lod.countCv, cvx) ||
        !decodeDelta(ry, lod.countCv, cvy) || !decodeDelta(rz, lod.countCv, cvz)) {
        if (error) *error = "geometry v2: bad delta varint stream";
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

}  // namespace pmsf

#endif  // PMSF_STORAGE_PAPER_STREAM_CODEC_V2_H
