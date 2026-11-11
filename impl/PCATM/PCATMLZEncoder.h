// SPDX-License-Identifier: Apache-2.0
//
// PCATMLZEncoder.h — thin zlib deflate/inflate wrapper for PCATM paper §4 LZ stream.
//
// Paper: "The vector is then compressed by some lossless compression technique.
// In our implementation we used an LZ encoder."
//
// Uses zlib deflate (raw, no gzip/zlib header) for maximum compression density.

#ifndef PMSF_COMMON_PCATM_LZ_ENCODER_H
#define PMSF_COMMON_PCATM_LZ_ENCODER_H

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

#include <zlib.h>

namespace pmsf::pcatm {

inline std::vector<uint8_t> lzDeflate(const uint8_t* data, size_t size, int level = 9) {
    if (size == 0) return {};
    uLongf bound = compressBound(static_cast<uLong>(size));
    std::vector<uint8_t> out(bound);
    uLongf destLen = bound;
    int rc = compress2(out.data(), &destLen, data, static_cast<uLong>(size), level);
    if (rc != Z_OK) {
        throw std::runtime_error(std::string("[PCATM LZ] deflate failed: ") +
                                 (rc == Z_MEM_ERROR ? "Z_MEM_ERROR" :
                                  rc == Z_BUF_ERROR ? "Z_BUF_ERROR" : "unknown"));
    }
    out.resize(destLen);
    return out;
}

inline std::vector<uint8_t> lzDeflate(const std::vector<uint8_t>& data, int level = 9) {
    return lzDeflate(data.data(), data.size(), level);
}

inline std::vector<uint8_t> lzInflate(const uint8_t* data, size_t size, size_t expectedUncompressed) {
    if (size == 0) return {};
    uLongf destLen = static_cast<uLong>(expectedUncompressed);
    std::vector<uint8_t> out(destLen);
    int rc = uncompress(out.data(), &destLen, data, static_cast<uLong>(size));
    if (rc == Z_BUF_ERROR) {
        // buffer too small, try larger
        out.resize(destLen);
        rc = uncompress(out.data(), &destLen, data, static_cast<uLong>(size));
    }
    if (rc != Z_OK) {
        throw std::runtime_error(std::string("[PCATM LZ] inflate failed: ") +
                                 (rc == Z_DATA_ERROR ? "Z_DATA_ERROR" :
                                  rc == Z_MEM_ERROR ? "Z_MEM_ERROR" :
                                  rc == Z_BUF_ERROR ? "Z_BUF_ERROR" : "unknown"));
    }
    out.resize(destLen);
    return out;
}

/// Pack bits into bytes (MSB first within each byte). Unused trailing bits = 0.
inline std::vector<uint8_t> packBits(const std::vector<uint8_t>& bits) {
    const size_t N = bits.size();
    const size_t numBytes = (N + 7) / 8;
    std::vector<uint8_t> out(numBytes, 0);
    for (size_t i = 0; i < N; ++i)
        if (bits[i]) out[i >> 3] |= (1u << (7u - (i & 7u)));
    return out;
}

/// Unpack bytes into bits.
inline std::vector<uint8_t> unpackBits(const uint8_t* packed, size_t numBits) {
    std::vector<uint8_t> bits(numBits, 0);
    for (size_t i = 0; i < numBits; ++i)
        bits[i] = (packed[i >> 3] >> (7u - (i & 7u))) & 1u;
    return bits;
}

}  // namespace pmsf::pcatm

#endif  // PMSF_COMMON_PCATM_LZ_ENCODER_H
