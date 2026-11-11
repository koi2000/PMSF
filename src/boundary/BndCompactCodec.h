#pragma once

// M4 compact payload codec for mode71 boundary-stage records (v2).
// Byte-oriented varint/zigzag encoding; the resulting byte stream is packed
// into the existing int-vector boundary channel with a negative-length marker:
//   residuals[0] = -(byteCount), then bytes packed 4-per-int (little endian).

#include <cstddef>
#include <cstdint>
#include <vector>

namespace pmsf {
namespace bndid {

using BndBytes = std::vector<unsigned char>;

inline void putU(BndBytes& b, std::uint32_t v) {
    while (v >= 0x80u) {
        b.push_back(static_cast<unsigned char>((v & 0x7Fu) | 0x80u));
        v >>= 7;
    }
    b.push_back(static_cast<unsigned char>(v));
}

inline void putS(BndBytes& b, std::int32_t v) {
    const std::uint32_t zz = (static_cast<std::uint32_t>(v) << 1) ^
                             static_cast<std::uint32_t>(v >> 31);
    putU(b, zz);
}

struct ByteReader {
    const BndBytes& b;
    std::size_t pos = 0;
    bool ok = true;
    explicit ByteReader(const BndBytes& bytes) : b(bytes) {}
    std::uint32_t getU() {
        std::uint32_t v = 0;
        int shift = 0;
        while (true) {
            if (pos >= b.size() || shift > 28) {
                ok = false;
                return 0;
            }
            const unsigned char c = b[pos++];
            v |= static_cast<std::uint32_t>(c & 0x7Fu) << shift;
            if ((c & 0x80u) == 0) break;
            shift += 7;
        }
        return v;
    }
    std::int32_t getS() {
        const std::uint32_t zz = getU();
        return static_cast<std::int32_t>((zz >> 1) ^ (~(zz & 1u) + 1u));
    }
    unsigned char getByte() {
        if (pos >= b.size()) {
            ok = false;
            return 0;
        }
        return b[pos++];
    }
};

}  // namespace bndid
}  // namespace pmsf