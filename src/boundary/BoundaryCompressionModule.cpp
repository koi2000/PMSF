#include "boundary/BoundaryCompressionModule.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <zlib.h>

namespace pmsf {
namespace {

void putI32(FILE* f, int v) {
    std::int32_t x = static_cast<std::int32_t>(v);
    std::fwrite(&x, sizeof(x), 1, f);
}

bool getI32(FILE* f, int& v) {
    std::int32_t x = 0;
    if (std::fread(&x, sizeof(x), 1, f) != 1) return false;
    v = static_cast<int>(x);
    return true;
}

void appendVarU32(std::vector<unsigned char>& out, std::uint32_t v) {
    while (v >= 0x80) {
        out.push_back(static_cast<unsigned char>(v | 0x80));
        v >>= 7;
    }
    out.push_back(static_cast<unsigned char>(v));
}

bool readVarU32(const std::vector<unsigned char>& in, std::size_t& pos, std::uint32_t& v) {
    v = 0;
    int shift = 0;
    while (pos < in.size() && shift <= 28) {
        const unsigned char b = in[pos++];
        v |= static_cast<std::uint32_t>(b & 0x7F) << shift;
        if ((b & 0x80) == 0) return true;
        shift += 7;
    }
    return false;
}

}  // namespace



bool BoundaryCompressionModule::writePayloadSection(const std::string& path,
                                                    const std::vector<int>& payload,
                                                    std::size_t* outBytes) {
    std::vector<unsigned char> raw;
    raw.reserve(payload.size() * 2);
    for (int v : payload) {
        const std::uint32_t zig =
            (static_cast<std::uint32_t>(v) << 1) ^ static_cast<std::uint32_t>(v >> 31);
        appendVarU32(raw, zig);
    }
    uLongf compSize = compressBound(static_cast<uLong>(raw.size()));
    std::vector<unsigned char> comp(static_cast<std::size_t>(compSize));
    if (compress2(comp.data(), &compSize, raw.data(), static_cast<uLong>(raw.size()), 9) != Z_OK)
        return false;
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    std::fwrite("BND4", 1, 4, f);
    putI32(f, 4);
    putI32(f, static_cast<int>(payload.size()));
    putI32(f, static_cast<int>(raw.size()));
    putI32(f, static_cast<int>(compSize));
    std::fwrite(comp.data(), 1, static_cast<std::size_t>(compSize), f);
    const long pos = std::ftell(f);
    std::fclose(f);
    if (outBytes) *outBytes = static_cast<std::size_t>(pos);
    return true;
}

bool BoundaryCompressionModule::readPayloadSection(const std::string& path,
                                                   std::vector<int>& payload) {
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    char magic[4] = {};
    if (std::fread(magic, 1, 4, f) != 4) {
        std::fclose(f);
        return false;
    }
    int version = 0, count = 0, compSize = 0, rawSize32 = 0;
    if (std::memcmp(magic, "BND4", 4) == 0) {
        if (!getI32(f, version) || version != 4 || !getI32(f, count) || count < 0 ||
            !getI32(f, rawSize32) || rawSize32 < 0 || !getI32(f, compSize) || compSize < 0) {
            std::fclose(f);
            return false;
        }
        std::vector<unsigned char> comp(static_cast<std::size_t>(compSize));
        if (compSize > 0 && std::fread(comp.data(), 1, comp.size(), f) != comp.size()) {
            std::fclose(f);
            return false;
        }
        std::fclose(f);
        std::vector<unsigned char> raw(static_cast<std::size_t>(rawSize32));
        uLongf rawSize = static_cast<uLongf>(raw.size());
        if (uncompress(raw.data(), &rawSize, comp.data(), static_cast<uLong>(comp.size())) != Z_OK ||
            rawSize != raw.size())
            return false;
        payload.resize(static_cast<std::size_t>(count));
        std::size_t pos = 0;
        for (int i = 0; i < count; ++i) {
            std::uint32_t zig = 0;
            if (!readVarU32(raw, pos, zig)) return false;
            const std::int32_t v = static_cast<std::int32_t>((zig >> 1) ^
                                                             (0u - (zig & 1u)));
            payload[static_cast<std::size_t>(i)] = static_cast<int>(v);
        }
        return true;
    }
    if (std::memcmp(magic, "BND2", 4) == 0) {
        if (!getI32(f, version) || version != 2 || !getI32(f, count) || count < 0) {
            std::fclose(f);
            return false;
        }
        payload.resize(static_cast<std::size_t>(count));
        for (int i = 0; i < count; ++i) {
            if (!getI32(f, payload[static_cast<std::size_t>(i)])) {
                std::fclose(f);
                return false;
            }
        }
        std::fclose(f);
        return true;
    }
    if (std::memcmp(magic, "BND3", 4) != 0) {
        std::fclose(f);
        return false;
    }
    if (!getI32(f, version) || version != 3 || !getI32(f, count) || count < 0 ||
        !getI32(f, compSize) || compSize < 0) {
        std::fclose(f);
        return false;
    }
    std::vector<unsigned char> comp(static_cast<std::size_t>(compSize));
    if (compSize > 0 && std::fread(comp.data(), 1, comp.size(), f) != comp.size()) {
        std::fclose(f);
        return false;
    }
    std::fclose(f);
    std::vector<unsigned char> raw(static_cast<std::size_t>(count) * sizeof(std::int32_t));
    uLongf rawSize = static_cast<uLongf>(raw.size());
    if (uncompress(raw.data(), &rawSize, comp.data(), static_cast<uLong>(comp.size())) != Z_OK ||
        rawSize != raw.size())
        return false;
    payload.resize(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        std::int32_t v = 0;
        std::memcpy(&v, raw.data() + static_cast<std::size_t>(i) * sizeof(v), sizeof(v));
        payload[static_cast<std::size_t>(i)] = static_cast<int>(v);
    }
    return true;
}

}  // namespace pmsf
