// SPDX-License-Identifier: Apache-2.0
//
// PCATMStrictAdaptiveResidual.h
//
// task-59 (B1-alt): paper-faithful entropy recoding of the PCATM strict
// displacement stream.  The displacement values are unchanged; only their
// entropy code changes (paper §4 says the displacement vectors are Huffman
// coded - this is the same data entropy-coded per round/axis with an adaptive
// binary bit-plane model instead of a single global varint+zlib blob).
//
// Bit-plane model (deterministic, identical on encoder and decoder):
//   for round r, axis a (x=0,y=1,z=2), value v:
//     z = zigzag(v)                     (uint32)
//     L = bit_length(z)                 (0 for z==0)
//     length  : 4-bit binary tree, node contexts per (r,a)
//     mantissa: bits L-2 .. 0, contexts per (r,a,k)
// Model index layout:
//   [0, numRounds*3*15)                 length tree nodes (15 per r,a)
//   [lenCount, lenCount+numRounds*3*14) mantissa bit positions (14 per r,a)
// All models are binary qsmodels (scale 10, init count 1<<9) driven by the
// framework range coder - exactly the same primitive the storage layer uses.
//
// Wire format (little-endian varints):
//   varint numRounds
//   varint perRoundCount[0..numRounds)
//   varint payloadBytes
//   payload bytes (range-coded)
//
// This header intentionally depends only on the range coder + plain int
// triples so it can be unit-tested standalone before wiring into the collector
// and reader.

#ifndef PMSF_COMPRESS_PCATM_STRICT_ADAPTIVE_RESIDUAL_H
#define PMSF_COMPRESS_PCATM_STRICT_ADAPTIVE_RESIDUAL_H

#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace pmsf {
namespace pcatmstrict {

namespace ard {

constexpr int kLenTreeModelsPerAxis = 15;      // binary tree over a 4-bit length
constexpr int kMantissaModelsPerAxis = 14;     // bits 13..0 below the top bit
constexpr int kLenBits = 4;
constexpr int kMaxMantissaBits = 14;           // z < 2^15 => at most 13 mantissa bits
constexpr int kScaleBits = 12;
constexpr int kRescale = 64;

struct ArdConfig {
    int scaleBits = kScaleBits;
    int rescale = kRescale;
};

inline int32_t zigzag32(int32_t v) {
    const uint32_t u = static_cast<uint32_t>(v) << 1;
    return static_cast<int32_t>(u ^ static_cast<uint32_t>(v >> 31));
}

inline int32_t unzigzag32(int32_t z) {
    const uint32_t u = static_cast<uint32_t>(z);
    return static_cast<int32_t>((u >> 1) ^ (~(u & 1u) + 1u));
}

inline void appendVarU32(std::vector<uint8_t>& out, uint32_t v) {
    while (v >= 0x80u) {
        out.push_back(static_cast<uint8_t>((v & 0x7fu) | 0x80u));
        v >>= 7;
    }
    out.push_back(static_cast<uint8_t>(v));
}

inline uint32_t readVarU32(const uint8_t* data,
                           std::size_t size,
                           std::size_t& off,
                           const char* what) {
    uint32_t v = 0;
    int shift = 0;
    while (true) {
        if (off >= size)
            throw std::runtime_error(std::string("[PCATM-ADAPT-RES] truncated ") + what);
        const uint8_t b = data[off++];
        v |= static_cast<uint32_t>(b & 0x7fu) << shift;
        if ((b & 0x80u) == 0)
            break;
        shift += 7;
        if (shift > 28)
            throw std::runtime_error(std::string("[PCATM-ADAPT-RES] bad varint ") + what);
    }
    return v;
}

inline int lenModelIndex(int numRounds, int round, int axis, int node) {
    (void)numRounds;
    (void)round;
    return axis * kLenTreeModelsPerAxis + node;
}

inline int mantissaModelIndexBase(int numRounds) {
    (void)numRounds;
    return 3 * kLenTreeModelsPerAxis;
}

inline int mantissaModelIndex(int numRounds, int round, int axis, int k) {
    (void)numRounds;
    (void)round;
    return mantissaModelIndexBase(numRounds) + axis * kMantissaModelsPerAxis + k;
}

inline void initModels(std::vector<qsmodel>& models, int numRounds, int compress,
                       const ArdConfig& cfg = ArdConfig{}) {
    (void)numRounds;
    models.resize(static_cast<std::size_t>(3) *
                  (kLenTreeModelsPerAxis + kMantissaModelsPerAxis));
    for (auto& m : models) {
        initqsmodel(&m, 2, cfg.scaleBits, cfg.rescale, nullptr, compress);
    }
}

inline void freeModels(std::vector<qsmodel>& models) {
    for (auto& m : models)
        deleteqsmodel(&m);
    models.clear();
}

inline void encodeBit(qsmodel& model, rangecoder& rc, int bit, const ArdConfig& cfg) {
    int syfreq = 0;
    int ltfreq = 0;
    qsgetfreq(&model, bit, &syfreq, &ltfreq);
    encode_shift(&rc, static_cast<freq>(syfreq), static_cast<freq>(ltfreq),
                 static_cast<freq>(cfg.scaleBits));
    qsupdate(&model, bit);
}

inline int decodeBit(qsmodel& model, rangecoder& rc, const ArdConfig& cfg) {
    const int cf = decode_culshift(&rc, cfg.scaleBits);
    const int sym = qsgetsym(&model, cf);
    int syfreq = 0;
    int ltfreq = 0;
    qsgetfreq(&model, sym, &syfreq, &ltfreq);
    decode_update(&rc, static_cast<freq>(syfreq), static_cast<freq>(ltfreq),
                  static_cast<freq>(1 << cfg.scaleBits));
    qsupdate(&model, sym);
    return sym;
}

inline uint32_t encodeValue(qsmodel* models,
                            int numRounds,
                            rangecoder& rc,
                            int round,
                            int axis,
                            int32_t value,
                            const ArdConfig& cfg = ArdConfig{}) {
    uint32_t z = static_cast<uint32_t>(zigzag32(value));
    int L = 0;
    while ((z >> L) != 0)
        ++L;
    if (L > 15)
        throw std::runtime_error("[PCATM-ADAPT-RES] residual magnitude out of range");
    int node = 0;
    for (int level = 0; level < kLenBits; ++level) {
        const int bit = (L >> (kLenBits - 1 - level)) & 1;
        encodeBit(models[lenModelIndex(numRounds, round, axis, node)], rc, bit, cfg);
        node = node * 2 + 1 + bit;
    }
    for (int k = L - 2; k >= 0; --k) {
        const int bit = static_cast<int>((z >> k) & 1u);
        encodeBit(models[mantissaModelIndex(numRounds, round, axis, k)], rc, bit, cfg);
    }
    return z;
}

inline int32_t decodeValue(qsmodel* models,
                           int numRounds,
                           rangecoder& rc,
                           int round,
                           int axis,
                           const ArdConfig& cfg = ArdConfig{}) {
    int node = 0;
    int L = 0;
    for (int level = 0; level < kLenBits; ++level) {
        const int bit = decodeBit(models[lenModelIndex(numRounds, round, axis, node)], rc, cfg);
        L = (L << 1) | bit;
        node = node * 2 + 1 + bit;
    }
    if (L == 0)
        return 0;
    if (L == 1)
        return unzigzag32(1);
    uint32_t z = 1u << (L - 1);
    for (int k = L - 2; k >= 0; --k) {
        const int bit = decodeBit(models[mantissaModelIndex(numRounds, round, axis, k)], rc, cfg);
        z |= static_cast<uint32_t>(bit) << k;
    }
    return unzigzag32(static_cast<int32_t>(z));
}

inline std::vector<uint8_t> encode(
    const std::vector<std::vector<std::array<int32_t, 3>>>& rounds,
    const ArdConfig& cfg = ArdConfig{}) {
    const int numRounds = static_cast<int>(rounds.size());
    std::size_t totalValues = 0;
    for (const auto& r : rounds)
        totalValues += r.size() * 3;
    std::vector<char> buf(totalValues * 2 + 65536);
    std::size_t rcOffset = 0;
    rangecoder rc;
    rc.p_data = buf.data();
    rc.p_dataOffset = &rcOffset;
    start_encoding(&rc, 0, 0);
    std::vector<qsmodel> models;
    initModels(models, numRounds, 1, cfg);
    for (int r = 0; r < numRounds; ++r) {
        const auto& res = rounds[static_cast<std::size_t>(r)];
        for (const auto& p : res) {
            for (int a = 0; a < 3; ++a)
                encodeValue(models.data(), numRounds, rc, r, a, p[static_cast<std::size_t>(a)], cfg);
        }
    }
    const std::size_t payloadBytes = done_encoding(&rc);
    freeModels(models);

    std::vector<uint8_t> out;
    appendVarU32(out, static_cast<uint32_t>(numRounds));
    for (const auto& r : rounds)
        appendVarU32(out, static_cast<uint32_t>(r.size()));
    appendVarU32(out, static_cast<uint32_t>(payloadBytes));
    out.insert(out.end(), reinterpret_cast<uint8_t*>(buf.data()),
               reinterpret_cast<uint8_t*>(buf.data()) + payloadBytes);
    return out;
}

inline std::vector<std::vector<std::array<int32_t, 3>>> decode(
    const uint8_t* data,
    std::size_t size,
    int expectedRounds,
    const ArdConfig& cfg = ArdConfig{}) {
    std::size_t off = 0;
    const int numRounds = static_cast<int>(readVarU32(data, size, off, "numRounds"));
    if (numRounds != expectedRounds)
        throw std::runtime_error("[PCATM-ADAPT-RES] round count mismatch");
    std::vector<std::vector<std::array<int32_t, 3>>> rounds(
        static_cast<std::size_t>(numRounds));
    for (int r = 0; r < numRounds; ++r) {
        const uint32_t count = readVarU32(data, size, off, "roundCount");
        if (count > (1u << 20))
            throw std::runtime_error("[PCATM-ADAPT-RES] residual count out of range");
        rounds[static_cast<std::size_t>(r)].resize(count);
    }
    const uint32_t payloadBytes = readVarU32(data, size, off, "payloadBytes");
    if (off + payloadBytes > size)
        throw std::runtime_error("[PCATM-ADAPT-RES] truncated payload");

    std::vector<char> staging(data + off, data + off + payloadBytes);
    std::size_t rcOffset = 0;
    rangecoder rc;
    rc.p_data = staging.data();
    rc.p_dataOffset = &rcOffset;
    if (start_decoding(&rc) != 0)
        throw std::runtime_error("[PCATM-ADAPT-RES] range start failed");
    std::vector<qsmodel> models;
    initModels(models, numRounds, 0, cfg);
    for (int r = 0; r < numRounds; ++r) {
        auto& res = rounds[static_cast<std::size_t>(r)];
        for (auto& p : res) {
            for (int a = 0; a < 3; ++a)
                p[static_cast<std::size_t>(a)] =
                    decodeValue(models.data(), numRounds, rc, r, a, cfg);
        }
    }
    done_decoding(&rc);
    freeModels(models);
    return rounds;
}

}  // namespace ard
}  // namespace pcatmstrict
}  // namespace pmsf

#endif  // PMSF_COMPRESS_PCATM_STRICT_ADAPTIVE_RESIDUAL_H
