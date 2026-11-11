/*
 * @file PaperStreamSymbolAdapter.h
 * @brief Convert between the in-memory PMSFSymbolTable and PaperStream rows.
 *
 * This is the M2 seam: collectors already fill PaperRoundSymbols
 * (F/Et/Ef/V/Cf/Cv + side cells); the adapter turns those into the storage
 * table, and the reverse path reconstructs the exact PaperRoundSymbols (down
 * to the SideCell variant type) so the existing SymbolReadOperator can keep
 * consuming the same data.
 */
#ifndef PMSF_STORAGE_PAPER_STREAM_SYMBOL_ADAPTER_H
#define PMSF_STORAGE_PAPER_STREAM_SYMBOL_ADAPTER_H

#include "core/Symbol.h"
#include "rangecoder/rangecod.h"
#include "storage/PaperStreamTable.h"

#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace pmsf {
namespace paper_adapter {

inline void appendU32(std::vector<std::uint8_t>& out, std::uint32_t v) {
    for (int i = 0; i < 4; ++i) out.push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xffu));
}

inline bool readU32(const std::vector<std::uint8_t>& in, std::size_t& off, std::uint32_t& v) {
    if (off + 4 > in.size()) return false;
    v = 0;
    for (int i = 0; i < 4; ++i) v |= static_cast<std::uint32_t>(in[off + i]) << (8 * i);
    off += 4;
    return true;
}

inline void appendFloat(std::vector<std::uint8_t>& out, float f) {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &f, sizeof(bits));
    appendU32(out, bits);
}

inline bool readFloat(const std::vector<std::uint8_t>& in, std::size_t& off, float& f) {
    std::uint32_t bits = 0;
    if (!readU32(in, off, bits)) return false;
    std::memcpy(&f, &bits, sizeof(f));
    return true;
}

inline void appendVarint(std::vector<std::uint8_t>& out, std::uint32_t value) {
    while (value >= 0x80u) {
        out.push_back(static_cast<std::uint8_t>((value & 0x7fu) | 0x80u));
        value >>= 7;
    }
    out.push_back(static_cast<std::uint8_t>(value));
}

inline bool readVarint(const std::vector<std::uint8_t>& in, std::size_t& off,
                       std::uint32_t& value) {
    value = 0;
    int shift = 0;
    while (off < in.size() && shift <= 28) {
        const std::uint8_t byte = in[off++];
        value |= static_cast<std::uint32_t>(byte & 0x7fu) << shift;
        if ((byte & 0x80u) == 0) return true;
        shift += 7;
    }
    return false;
}

inline std::uint32_t zigzagEncode(std::int32_t value) {
    const auto u = static_cast<std::uint32_t>(value);
    return (u << 1) ^ static_cast<std::uint32_t>(value >> 31);
}

inline std::int32_t zigzagDecode(std::uint32_t value) {
    return static_cast<std::int32_t>((value >> 1) ^
                                     static_cast<std::uint32_t>(-static_cast<std::int32_t>(value & 1u)));
}

inline std::vector<std::uint8_t> encodeBoolContextCoder(
    const std::deque<SymbolCode>& values) {
    std::vector<std::uint8_t> out;
    out.push_back(3);
    appendVarint(out, static_cast<std::uint32_t>(values.size()));
    std::vector<char> buf(values.size() * 2 + 4096);
    std::size_t off = 0;
    rangecoder rc{};
    rc.p_data = buf.data();
    rc.p_dataOffset = &off;
    start_encoding(&rc, 0, 0);
    std::uint32_t c0[256];
    std::uint32_t c1[256];
    for (int i = 0; i < 256; ++i) { c0[i] = 1; c1[i] = 1; }
    int ctx = 0;
    for (SymbolCode v : values) {
        const int bit = v ? 1 : 0;
        const std::uint32_t tot = c0[ctx] + c1[ctx];
        const std::uint32_t syf = bit ? c1[ctx] : c0[ctx];
        const std::uint32_t lt = bit ? c0[ctx] : 0;
        encode_freq(&rc, static_cast<freq>(syf), static_cast<freq>(lt),
                    static_cast<freq>(tot));
        if (bit) ++c1[ctx]; else ++c0[ctx];
        if (c0[ctx] + c1[ctx] > 32768) { c0[ctx] = (c0[ctx] + 1) / 2; c1[ctx] = (c1[ctx] + 1) / 2; }
        ctx = ((ctx << 1) | bit) & 0xff;
    }
    done_encoding(&rc);
    out.insert(out.end(), reinterpret_cast<const std::uint8_t*>(buf.data()),
               reinterpret_cast<const std::uint8_t*>(buf.data()) + off);
    return out;
}

inline bool decodeBoolContextCoder(const std::vector<std::uint8_t>& bytes,
                                   std::uint32_t count, std::size_t off,
                                   std::deque<SymbolCode>& out) {
    std::vector<char> staging(bytes.begin() + static_cast<std::ptrdiff_t>(off),
                              bytes.end());
    std::size_t dataOff = 0;
    rangecoder rc{};
    rc.p_data = staging.data();
    rc.p_dataOffset = &dataOff;
    if (start_decoding(&rc) != 0) return false;
    std::uint32_t c0[256];
    std::uint32_t c1[256];
    for (int i = 0; i < 256; ++i) { c0[i] = 1; c1[i] = 1; }
    int ctx = 0;
    for (std::uint32_t i = 0; i < count; ++i) {
        const std::uint32_t tot = c0[ctx] + c1[ctx];
        const int ch = decode_culfreq(&rc, static_cast<freq>(tot));
        const int bit = ch >= static_cast<int>(c0[ctx]) ? 1 : 0;
        const std::uint32_t syf = bit ? c1[ctx] : c0[ctx];
        const std::uint32_t lt = bit ? c0[ctx] : 0;
        decode_update(&rc, static_cast<freq>(syf), static_cast<freq>(lt),
                      static_cast<freq>(tot));
        if (bit) ++c1[ctx]; else ++c0[ctx];
        if (c0[ctx] + c1[ctx] > 32768) { c0[ctx] = (c0[ctx] + 1) / 2; c1[ctx] = (c1[ctx] + 1) / 2; }
        ctx = ((ctx << 1) | bit) & 0xff;
        out.push_back(bit);
    }
    return true;
}

inline std::vector<std::uint8_t> encodeInt32Deque(const std::deque<SymbolCode>& values) {
    std::vector<std::uint8_t> out;
    bool allBool = true;
    for (SymbolCode v : values) {
        if (v != 0 && v != 1) {
            allBool = false;
            break;
        }
    }
    if (allBool && values.size() >= 8) {
        // mode 1: bit-packed 0/1 stream.  Core F/Et/Ef/V codes are mostly
        // boolean, so this is 8x smaller than one varint per symbol.
        out.push_back(1);
        appendVarint(out, static_cast<std::uint32_t>(values.size()));
        std::uint8_t cur = 0;
        int bits = 0;
        for (SymbolCode v : values) {
            if (v != 0) cur = static_cast<std::uint8_t>(cur | (1u << bits));
            if (++bits == 8) {
                out.push_back(cur);
                cur = 0;
                bits = 0;
            }
        }
        if (bits != 0) out.push_back(cur);
        auto contextCoded = encodeBoolContextCoder(values);
        return contextCoded.size() < out.size() ? std::move(contextCoded) : std::move(out);
    }
    // mode 0: zigzag varint
    out.push_back(0);
    out.reserve(out.size() + values.size());
    for (SymbolCode v : values) appendVarint(out, zigzagEncode(static_cast<std::int32_t>(v)));
    return out;
}

inline bool decodeInt32Deque(const std::vector<std::uint8_t>& bytes,
                             std::deque<SymbolCode>& out) {
    out.clear();
    if (bytes.empty()) return true;
    const std::uint8_t mode = bytes[0];
    std::size_t off = 1;
    if (mode == 0) {
        while (off < bytes.size()) {
            std::uint32_t v = 0;
            if (!readVarint(bytes, off, v)) return false;
            out.push_back(static_cast<SymbolCode>(zigzagDecode(v)));
        }
        return true;
    }
    if (mode == 1) {
        std::uint32_t count = 0;
        if (!readVarint(bytes, off, count)) return false;
        for (std::uint32_t i = 0; i < count; ++i) {
            const std::size_t byteIndex = off + (i >> 3);
            if (byteIndex >= bytes.size()) return false;
            out.push_back((bytes[byteIndex] >> (i & 7u)) & 1u);
        }
        return true;
    }
    if (mode == 3) {
        std::uint32_t count = 0;
        if (!readVarint(bytes, off, count)) return false;
        return decodeBoolContextCoder(bytes, count, off, out);
    }
    return false;
}

inline std::vector<std::uint8_t> encodePointDeque(const std::deque<MCGAL::Point>& values) {
    std::vector<std::uint8_t> out;
    out.reserve(values.size() * 3 * sizeof(float));
    for (const auto& p : values) {
        appendFloat(out, p.x());
        appendFloat(out, p.y());
        appendFloat(out, p.z());
    }
    return out;
}

inline bool decodePointDeque(const std::vector<std::uint8_t>& bytes,
                             std::deque<MCGAL::Point>& out) {
    if (bytes.size() % 12 != 0) return false;
    out.clear();
    std::size_t off = 0;
    while (off < bytes.size()) {
        float x = 0, y = 0, z = 0;
        if (!readFloat(bytes, off, x) || !readFloat(bytes, off, y) ||
            !readFloat(bytes, off, z)) {
            return false;
        }
        out.emplace_back(x, y, z);
    }
    return true;
}

inline std::vector<std::uint8_t> encodePointIntDeque(const std::deque<MCGAL::PointInt>& values) {
    std::vector<std::uint8_t> out;
    out.reserve(values.size() * 3 * sizeof(std::int32_t));
    for (const auto& p : values) {
        appendU32(out, static_cast<std::uint32_t>(p.x()));
        appendU32(out, static_cast<std::uint32_t>(p.y()));
        appendU32(out, static_cast<std::uint32_t>(p.z()));
    }
    return out;
}

inline bool decodePointIntDeque(const std::vector<std::uint8_t>& bytes,
                                std::deque<MCGAL::PointInt>& out) {
    if (bytes.size() % 12 != 0) return false;
    out.clear();
    std::size_t off = 0;
    while (off < bytes.size()) {
        std::uint32_t x = 0, y = 0, z = 0;
        if (!readU32(bytes, off, x) || !readU32(bytes, off, y) || !readU32(bytes, off, z)) {
            return false;
        }
        out.emplace_back(static_cast<int>(static_cast<std::int32_t>(x)),
                         static_cast<int>(static_cast<std::int32_t>(y)),
                         static_cast<int>(static_cast<std::int32_t>(z)));
    }
    return true;
}

inline std::vector<float> flattenPoints(const std::deque<MCGAL::Point>& values) {
    std::vector<float> out;
    out.reserve(values.size() * 3);
    for (const auto& p : values) {
        out.push_back(p.x());
        out.push_back(p.y());
        out.push_back(p.z());
    }
    return out;
}

inline std::vector<std::int32_t> flattenPointInts(const std::deque<MCGAL::PointInt>& values) {
    std::vector<std::int32_t> out;
    out.reserve(values.size() * 3);
    for (const auto& p : values) {
        out.push_back(p.x());
        out.push_back(p.y());
        out.push_back(p.z());
    }
    return out;
}

inline bool unflattenPoints(const std::vector<float>& values,
                            std::deque<MCGAL::Point>& out) {
    if (values.size() % 3 != 0) return false;
    out.clear();
    for (std::size_t i = 0; i + 2 < values.size(); i += 3) {
        out.emplace_back(values[i], values[i + 1], values[i + 2]);
    }
    return true;
}

inline bool unflattenPointInts(const std::vector<std::int32_t>& values,
                               std::deque<MCGAL::PointInt>& out) {
    if (values.size() % 3 != 0) return false;
    out.clear();
    for (std::size_t i = 0; i + 2 < values.size(); i += 3) {
        out.emplace_back(values[i], values[i + 1], values[i + 2]);
    }
    return true;
}

inline void appendSideCell(std::vector<std::uint8_t>& out, const SideCell& cell) {
    if (std::holds_alternative<bool>(cell)) {
        out.push_back(0);
        out.push_back(std::get<bool>(cell) ? 1 : 0);
    } else if (std::holds_alternative<std::int8_t>(cell)) {
        out.push_back(1);
        out.push_back(static_cast<std::uint8_t>(std::get<std::int8_t>(cell)));
    } else if (std::holds_alternative<std::int16_t>(cell)) {
        out.push_back(2);
        const auto v = static_cast<std::uint16_t>(std::get<std::int16_t>(cell));
        out.push_back(static_cast<std::uint8_t>(v & 0xffu));
        out.push_back(static_cast<std::uint8_t>((v >> 8) & 0xffu));
    } else if (std::holds_alternative<std::int32_t>(cell)) {
        out.push_back(3);
        appendU32(out, static_cast<std::uint32_t>(std::get<std::int32_t>(cell)));
    } else if (std::holds_alternative<std::uint16_t>(cell)) {
        out.push_back(4);
        const auto v = std::get<std::uint16_t>(cell);
        out.push_back(static_cast<std::uint8_t>(v & 0xffu));
        out.push_back(static_cast<std::uint8_t>((v >> 8) & 0xffu));
    } else if (std::holds_alternative<float>(cell)) {
        out.push_back(5);
        appendFloat(out, std::get<float>(cell));
    } else if (std::holds_alternative<MCGAL::Point>(cell)) {
        out.push_back(6);
        const auto& p = std::get<MCGAL::Point>(cell);
        appendFloat(out, p.x());
        appendFloat(out, p.y());
        appendFloat(out, p.z());
    } else if (std::holds_alternative<std::vector<std::uint8_t>>(cell)) {
        out.push_back(7);
        const auto& bytes = std::get<std::vector<std::uint8_t>>(cell);
        appendU32(out, static_cast<std::uint32_t>(bytes.size()));
        out.insert(out.end(), bytes.begin(), bytes.end());
    } else {
        throw std::runtime_error("PaperStream adapter: unsupported SideCell variant");
    }
}

inline std::vector<std::uint8_t> encodeSideCells(const std::vector<SideCell>& cells) {
    std::vector<std::uint8_t> out;
    for (const auto& cell : cells) appendSideCell(out, cell);
    return out;
}

inline bool decodeSideCells(const std::vector<std::uint8_t>& bytes,
                            std::vector<SideCell>& out,
                            std::string* error = nullptr) {
    out.clear();
    std::size_t off = 0;
    while (off < bytes.size()) {
        const std::uint8_t type = bytes[off++];
        switch (type) {
            case 0:
                if (off >= bytes.size()) { if (error) *error = "truncated bool"; return false; }
                out.emplace_back(bytes[off++] != 0);
                break;
            case 1:
                if (off >= bytes.size()) { if (error) *error = "truncated int8"; return false; }
                out.emplace_back(static_cast<std::int8_t>(bytes[off++]));
                break;
            case 2: {
                if (off + 2 > bytes.size()) { if (error) *error = "truncated int16"; return false; }
                const auto v = static_cast<std::uint16_t>(bytes[off] | (bytes[off + 1] << 8));
                off += 2;
                out.emplace_back(static_cast<std::int16_t>(v));
                break;
            }
            case 3: {
                std::uint32_t v = 0;
                if (!readU32(bytes, off, v)) { if (error) *error = "truncated int32"; return false; }
                out.emplace_back(static_cast<std::int32_t>(v));
                break;
            }
            case 4: {
                if (off + 2 > bytes.size()) { if (error) *error = "truncated uint16"; return false; }
                const auto v = static_cast<std::uint16_t>(bytes[off] | (bytes[off + 1] << 8));
                off += 2;
                out.emplace_back(v);
                break;
            }
            case 5: {
                float v = 0;
                if (!readFloat(bytes, off, v)) { if (error) *error = "truncated float"; return false; }
                out.emplace_back(v);
                break;
            }
            case 6: {
                float x = 0, y = 0, z = 0;
                if (!readFloat(bytes, off, x) || !readFloat(bytes, off, y) ||
                    !readFloat(bytes, off, z)) {
                    if (error) *error = "truncated Point";
                    return false;
                }
                out.emplace_back(MCGAL::Point(x, y, z));
                break;
            }
            case 7: {
                std::uint32_t size = 0;
                if (!readU32(bytes, off, size)) { if (error) *error = "truncated bytes size"; return false; }
                if (off + size > bytes.size()) { if (error) *error = "truncated bytes"; return false; }
                std::vector<std::uint8_t> data(bytes.begin() + static_cast<std::ptrdiff_t>(off),
                                               bytes.begin() + static_cast<std::ptrdiff_t>(off + size));
                off += size;
                out.emplace_back(std::move(data));
                break;
            }
            default:
                if (error) *error = "unknown SideCell type tag " + std::to_string(type);
                return false;
        }
    }
    return true;
}

inline bool hasStream(const PaperObjectRow& object, const std::string& name) {
    for (const auto& spec : object.streams) {
        if (spec.name == name) return true;
    }
    return false;
}

inline void ensureStream(PaperObjectRow& object, const std::string& name,
                         PaperCellType cellType, PaperStreamEncoding encoding,
                         const std::string& codec) {
    if (hasStream(object, name)) return;
    object.streams.push_back(PaperStreamSpec{name, cellType, encoding, codec, 0, true});
}

inline PaperStreamTable fromPMSFSymbolTable(const PaperObjectRow& object,
                                            const std::vector<PaperPartitionRow>& partitions,
                                            const PMSFSymbolTable& symbols) {
    PaperStreamTable table;
    table.object = object;
    table.partitions = partitions;
    if (table.object.numPartitions == 0) {
        table.object.numPartitions = static_cast<std::int32_t>(partitions.size());
    }
    ensureStream(table.object, "F", PaperCellType::Int32, PaperStreamEncoding::Plain, "paper_adapter");
    ensureStream(table.object, "Et", PaperCellType::Int32, PaperStreamEncoding::Plain, "paper_adapter");
    ensureStream(table.object, "Ef", PaperCellType::Int32, PaperStreamEncoding::Plain, "paper_adapter");
    ensureStream(table.object, "V", PaperCellType::Int32, PaperStreamEncoding::Plain, "paper_adapter");
    ensureStream(table.object, "Cf", PaperCellType::Float, PaperStreamEncoding::Plain, "paper_adapter");
    ensureStream(table.object, "Cv", PaperCellType::Float, PaperStreamEncoding::Plain, "paper_adapter");
    ensureStream(table.object, "CfQ", PaperCellType::Int32, PaperStreamEncoding::Plain, "paper_adapter");
    ensureStream(table.object, "CvQ", PaperCellType::Int32, PaperStreamEncoding::Plain, "paper_adapter");

    for (const auto& row : symbols) {
        PaperLodRow lod;
        lod.objectId = row.objectId;
        lod.partitionId = row.partitionId;
        lod.lodId = row.lodId;
        lod.F = encodeInt32Deque(row.delta.facetSymbols);
        lod.Et = encodeInt32Deque(row.delta.triangulationEdgeSymbols);
        lod.Ef = encodeInt32Deque(row.delta.flipEdgeSymbols);
        lod.V = encodeInt32Deque(row.delta.vertexSymbols);
        lod.countF = static_cast<std::int32_t>(row.delta.facetSymbols.size());
        lod.countEt = static_cast<std::int32_t>(row.delta.triangulationEdgeSymbols.size());
        lod.countEf = static_cast<std::int32_t>(row.delta.flipEdgeSymbols.size());
        lod.countV = static_cast<std::int32_t>(row.delta.vertexSymbols.size());
        lod.CfF = flattenPoints(row.delta.removedVertexCoords);
        lod.CvF = flattenPoints(row.delta.shiftedVertexCoords);
        lod.CfQ = flattenPointInts(row.delta.removedVertexResiduals);
        lod.CvQ = flattenPointInts(row.delta.shiftedVertexResiduals);
        lod.countCf = static_cast<std::int32_t>(
            !row.delta.removedVertexCoords.empty() ? row.delta.removedVertexCoords.size()
                                                  : row.delta.removedVertexResiduals.size());
        lod.countCv = static_cast<std::int32_t>(
            !row.delta.shiftedVertexCoords.empty() ? row.delta.shiftedVertexCoords.size()
                                                  : row.delta.shiftedVertexResiduals.size());
        for (const auto& [name, cells] : row.delta.side) {
            lod.extension[name] = encodeSideCells(cells);
            ensureStream(table.object, name, PaperCellType::Bytes,
                         PaperStreamEncoding::Plain, "sidecell");
            // A codec may store an already encoded geometry relation as a
            // named per-LOD stream. The declaration links its logical role
            // to the encoded values without duplicating them in a typed list.
            if (name.starts_with("paper_role/CfQ/")) {
                for (auto& spec:table.object.streams) {
                    if (spec.name=="CfQ") {
                        spec.codecName=name;
                        spec.relation=PaperStreamRelation::Geometry;
                        spec.source=PaperStreamSource::Payload;
                        spec.shape=PaperCellShape::Vec3;
                    } else if (spec.name==name) {
                        spec.relation=PaperStreamRelation::Geometry;
                        spec.source=PaperStreamSource::Payload;
                        spec.shape=PaperCellShape::Vec3;
                    }
                }
            }
        }
        table.lods.push_back(std::move(lod));
    }
    return table;
}

inline bool toPMSFSymbolTable(const PaperStreamTable& table,
                              PMSFSymbolTable& out,
                              std::string* error = nullptr) {
    out.clear();
    for (const auto& lod : table.lods) {
        PMSFSymbolRecord row;
        row.objectId = lod.objectId;
        row.partitionId = lod.partitionId;
        row.lodId = lod.lodId;
        if (!decodeInt32Deque(lod.F, row.delta.facetSymbols)) {
            if (error) *error = "bad F stream lod=" + std::to_string(lod.lodId);
            return false;
        }
        if (!decodeInt32Deque(lod.Et, row.delta.triangulationEdgeSymbols)) {
            if (error) *error = "bad Et stream lod=" + std::to_string(lod.lodId);
            return false;
        }
        if (!decodeInt32Deque(lod.Ef, row.delta.flipEdgeSymbols)) {
            if (error) *error = "bad Ef stream lod=" + std::to_string(lod.lodId);
            return false;
        }
        if (!decodeInt32Deque(lod.V, row.delta.vertexSymbols)) {
            if (error) *error = "bad V stream lod=" + std::to_string(lod.lodId);
            return false;
        }
        if (!unflattenPoints(lod.CfF, row.delta.removedVertexCoords) ||
            !unflattenPoints(lod.CvF, row.delta.shiftedVertexCoords) ||
            !unflattenPointInts(lod.CfQ, row.delta.removedVertexResiduals) ||
            !unflattenPointInts(lod.CvQ, row.delta.shiftedVertexResiduals)) {
            if (error) *error = "bad geometry stream";
            return false;
        }
        if (lod.countF!=static_cast<std::int32_t>(row.delta.facetSymbols.size()) ||
            lod.countEt!=static_cast<std::int32_t>(row.delta.triangulationEdgeSymbols.size()) ||
            lod.countEf!=static_cast<std::int32_t>(row.delta.flipEdgeSymbols.size()) ||
            lod.countV!=static_cast<std::int32_t>(row.delta.vertexSymbols.size()) ||
            lod.countCf!=static_cast<std::int32_t>(!row.delta.removedVertexCoords.empty()?row.delta.removedVertexCoords.size():row.delta.removedVertexResiduals.size()) ||
            lod.countCv!=static_cast<std::int32_t>(!row.delta.shiftedVertexCoords.empty()?row.delta.shiftedVertexCoords.size():row.delta.shiftedVertexResiduals.size())) {
            if (error) *error = "decoded typed role count mismatch at LOD="+std::to_string(lod.lodId);
            return false;
        }
        for (const auto& [name, bytes] : lod.extension) {
            std::vector<SideCell> cells;
            if (!decodeSideCells(bytes, cells, error)) return false;
            row.delta.side.emplace(name, std::move(cells));
        }
        out.push_back(std::move(row));
    }
    return true;
}

}  // namespace paper_adapter
}  // namespace pmsf

#endif  // PMSF_STORAGE_PAPER_STREAM_SYMBOL_ADAPTER_H
