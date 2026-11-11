// SPDX-License-Identifier: Apache-2.0
//
// PCATMStrictPMSFSymbolReadOperator.cpp

#include "common/strict/PCATMStrictPMSFSymbolReadOperator.h"
#include "common/strict/PCATMStrictAdaptiveResidual.h"
#include "common/PaperLayoutAliases.h"
#include "PCATM/PCATMPaperMode2.h"

#include "Point.h"
#include "BFSVersionManager.h"
#include "core/PaperProtocolHint.h"
#include "PCATM/PCATMLZEncoder.h"
#include "compress/ConnectivityBlobContainer.h"
#include "common/strict/PCATMStrictPMSFSymbolCollectOperator.h"
#include "RD/compress/RDSymbolCollectOperator.h"
#include "entropy/RangeCoderEntropyCoder.h"
#include "entropy/StaticHuffmanCoder.h"
#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"
#include "Facet.h"
#include "Global.h"
#include "Halfedge.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iostream>
#include <queue>
#include <span>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

bool isEnvOn(const char* name) {
    const char* v = std::getenv(name);
    return v && *v && std::strcmp(v, "0") != 0;
}

// PMSF_BPV_DIAG only: quantized face signature, used to diff encoder patch
// components against the decoder's color-recovered components.
inline std::string strictFaceSignature(MCGAL::Mesh* mesh, MCGAL::Facet* f) {
    if (!mesh || !f) return "?";
    MCGAL::Halfedge* start = f->proxyHalfedge();
    if (!start) return "?";
    std::vector<MCGAL::PointInt> pts;
    MCGAL::Halfedge* h = start;
    do {
        if (!h->vertex()) return "?";
        pts.push_back(mesh->floatPosToInt(h->vertex()->point()));
        h = h->next();
    } while (h && h != start);
    std::sort(pts.begin(), pts.end(),
              [](const MCGAL::PointInt& a, const MCGAL::PointInt& b) {
                  if (a.x() != b.x()) return a.x() < b.x();
                  if (a.y() != b.y()) return a.y() < b.y();
                  return a.z() < b.z();
              });
    std::string s;
    for (const auto& p : pts) {
        s += "(" + std::to_string(p.x()) + "," + std::to_string(p.y()) +
             "," + std::to_string(p.z()) + ")";
    }
    return s;
}

int ceilLog2Size(size_t n) {
    int bits = 0;
    size_t value = 1;
    while (value < n) {
        value <<= 1;
        ++bits;
    }
    return std::max(bits, 1);
}

int32_t readI32(std::span<const uint8_t> bytes, size_t& off, const char* field) {
    if (off + 4 > bytes.size()) {
        throw std::runtime_error(std::string("[PCATMStrictPaper decoder] truncated ") + field);
    }
    int32_t v = 0;
    std::memcpy(&v, bytes.data() + off, 4);
    off += 4;
    return v;
}

uint32_t readVarU32(const std::vector<uint8_t>& bytes, size_t& off, const char* field) {
    uint32_t value = 0;
    int shift = 0;
    while (true) {
        if (off >= bytes.size() || shift > 28) {
            throw std::runtime_error(std::string("[PCATMStrictPaper decoder] invalid varint ") +
                                     field);
        }
        const uint8_t b = bytes[off++];
        value |= static_cast<uint32_t>(b & 0x7fu) << shift;
        if ((b & 0x80u) == 0) return value;
        shift += 7;
    }
}

std::vector<uint8_t> decodeBitsContextRange(const uint8_t* payload,
                                            size_t payloadBytes,
                                            size_t numBits,
                                            int ctxBits = 8) {
    ctxBits = std::clamp(ctxBits, 0, 12);
    const int kCtxCount = 1 << ctxBits;
    constexpr int kLgTotf = 10;
    std::size_t off = 0;
    std::vector<char> staging(payload, payload + payloadBytes);
    rangecoder rc;
    rc.p_data = staging.data();
    rc.p_dataOffset = &off;
    if (start_decoding(&rc) != 0) {
        throw std::runtime_error("[PCATMStrictPaper decoder] context color range start failed");
    }

    std::vector<qsmodel> models(kCtxCount);
    for (auto& model : models) {
        initqsmodel(&model, 2, kLgTotf, 1 << (kLgTotf - 1), nullptr, 0);
    }

    std::vector<uint8_t> bits;
    bits.reserve(numBits);
    int ctx = 0;
    int prevSym = -1;
    const int ctxMask = kCtxCount - 1;
    for (size_t i = 0; i < numBits; ++i) {
        const int cf = decode_culshift(&rc, kLgTotf);
        const int sym = qsgetsym(&models[ctx], cf);
        int syfreq = 0;
        int ltfreq = 0;
        qsgetfreq(&models[ctx], sym, &syfreq, &ltfreq);
        decode_update(&rc,
                      static_cast<freq>(syfreq),
                      static_cast<freq>(ltfreq),
                      static_cast<freq>(1 << kLgTotf));
        qsupdate(&models[ctx], sym);
        bits.push_back(static_cast<uint8_t>(sym));
        // Match the encoder's lag-1 context update exactly: at step i the
        // model index was built from symbols s_0..s_{i-2}; the symbol s_{i-1}
        // is folded in only after coding step i (encodeBitsContextRange's
        // deriveCtx does `c = ctx; if (prev >= 0) ctx = (ctx<<1|prev)&mask`).
        // The previous immediate-update form desynchronised codec 8 by one
        // step, which is why auto-selected adaptive-ctx color streams decoded
        // to garbage (WS-B 2026-10-22).
        if (prevSym >= 0) ctx = ((ctx << 1) | prevSym) & ctxMask;
        prevSym = sym;
    }

    done_decoding(&rc);
    for (auto& model : models) deleteqsmodel(&model);
    return bits;
}

int32_t unzigzag32(uint32_t v) {
    return static_cast<int32_t>((v >> 1) ^ (~(v & 1u) + 1u));
}

std::vector<uint8_t> decodeColorLz(std::span<const uint8_t> sl) {
    size_t off = 0;
    const int32_t numBits = readI32(sl, off, "color.numBits");
    const int32_t packedBytes = readI32(sl, off, "color.packedBytes");
    const int32_t lzBytes = readI32(sl, off, "color.lzBytes");
    if (numBits < 0 || packedBytes < 0 || lzBytes < 0) {
        throw std::runtime_error("[PCATMStrictPaper decoder] negative color stream size");
    }
    if (off + static_cast<size_t>(lzBytes) > sl.size()) {
        throw std::runtime_error("[PCATMStrictPaper decoder] truncated color LZ payload");
    }
    const size_t expectedPacked = static_cast<size_t>(packedBytes);
    std::vector<uint8_t> packed = pmsf::pcatm::lzInflate(
        sl.data() + off, static_cast<size_t>(lzBytes), expectedPacked);
    if (packed.size() != expectedPacked) {
        throw std::runtime_error(
            "[PCATMStrictPaper decoder] inflated color size mismatch");
    }
    return pmsf::pcatm::unpackBits(packed.data(), static_cast<size_t>(numBits));
}

std::vector<uint8_t> decodeColorLzV3(std::span<const uint8_t> sl, int& colorMode) {
    size_t off = 0;
    colorMode = readI32(sl, off, "color.mode");
    const int32_t numSymbols = readI32(sl, off, "color.numSymbols");
    const int32_t bitsPerSymbol = readI32(sl, off, "color.bitsPerSymbol");
    const int32_t packedBytes = readI32(sl, off, "color.packedBytes");
    const int32_t lzBytes = readI32(sl, off, "color.lzBytes");
    if ((colorMode != 2 && colorMode != 4) || numSymbols < 0 ||
        (bitsPerSymbol != 1 && bitsPerSymbol != 2 &&
         bitsPerSymbol != -1 && bitsPerSymbol != -2 &&
         bitsPerSymbol != -3 && bitsPerSymbol != -4) ||
        packedBytes < 0 || lzBytes < 0) {
        throw std::runtime_error("[PCATMStrictPaper decoder] invalid sv=3 color header");
    }
    if (off + static_cast<size_t>(lzBytes) > sl.size()) {
        throw std::runtime_error("[PCATMStrictPaper decoder] truncated sv=3 color LZ payload");
    }
    if (bitsPerSymbol == -2) {
        std::vector<uint32_t> decoded;
        decoded.reserve(static_cast<size_t>(numSymbols));
        pmsf::RangeCoderEntropyCoder coder((colorMode == 4) ? 2 : 1);
        const size_t used = coder.decode(sl.data() + off,
                                         static_cast<size_t>(lzBytes),
                                         static_cast<size_t>(numSymbols),
                                         decoded);
        if (used > static_cast<size_t>(lzBytes) ||
            decoded.size() != static_cast<size_t>(numSymbols)) {
            throw std::runtime_error("[PCATMStrictPaper decoder] color range count mismatch");
        }
        std::vector<uint8_t> symbols;
        symbols.reserve(decoded.size());
        for (uint32_t sym : decoded) {
            if ((colorMode == 2 && sym > 1u) || (colorMode == 4 && sym > 3u)) {
                throw std::runtime_error("[PCATMStrictPaper decoder] color range symbol OOB");
            }
            symbols.push_back(static_cast<uint8_t>(sym));
        }
        return symbols;
    }
    if (bitsPerSymbol == -4) {
        if (static_cast<size_t>(lzBytes) != static_cast<size_t>(packedBytes)) {
            throw std::runtime_error("[PCATMStrictPaper decoder] invalid raw color size");
        }
        std::vector<uint8_t> packed(sl.begin() + static_cast<std::ptrdiff_t>(off),
                                    sl.begin() + static_cast<std::ptrdiff_t>(off + lzBytes));
        const int actualBitsPerSymbol = (colorMode == 4) ? 2 : 1;
        std::vector<uint8_t> bits = pmsf::pcatm::unpackBits(
            packed.data(), static_cast<size_t>(numSymbols * actualBitsPerSymbol));
        std::vector<uint8_t> symbols;
        symbols.reserve(static_cast<size_t>(numSymbols));
        for (int32_t i = 0; i < numSymbols; ++i) {
            if (actualBitsPerSymbol == 1) {
                symbols.push_back(bits[static_cast<size_t>(i)] & 1u);
            } else {
                const size_t bi = static_cast<size_t>(i) * 2;
                symbols.push_back(static_cast<uint8_t>(
                    ((bits[bi] & 1u) << 1) | (bits[bi + 1] & 1u)));
            }
        }
        return symbols;
    }
    std::vector<uint8_t> packed = pmsf::pcatm::lzInflate(
        sl.data() + off, static_cast<size_t>(lzBytes), static_cast<size_t>(packedBytes));
    if (bitsPerSymbol == -1) {
        std::vector<uint8_t> symbols;
        symbols.reserve(static_cast<size_t>(numSymbols));
        size_t rawOff = 0;
        while (symbols.size() < static_cast<size_t>(numSymbols)) {
            const uint32_t runLen = readVarU32(packed, rawOff, "color.rleRunLen");
            if (rawOff >= packed.size() || runLen == 0) {
                throw std::runtime_error("[PCATMStrictPaper decoder] invalid color RLE stream");
            }
            const uint8_t sym = packed[rawOff++];
            if ((colorMode == 2 && sym > 1u) || (colorMode == 4 && sym > 3u) ||
                symbols.size() + static_cast<size_t>(runLen) > static_cast<size_t>(numSymbols)) {
                throw std::runtime_error("[PCATMStrictPaper decoder] invalid color RLE symbol");
            }
            symbols.insert(symbols.end(), static_cast<size_t>(runLen), sym);
        }
        if (rawOff != packed.size()) {
            throw std::runtime_error("[PCATMStrictPaper decoder] trailing color RLE bytes");
        }
        return symbols;
    }
    if (bitsPerSymbol == -3) {
        std::vector<uint8_t> symbols;
        symbols.reserve(static_cast<size_t>(numSymbols));
        size_t rawOff = 0;
        while (rawOff < packed.size()) {
            const uint32_t zeroRun = readVarU32(packed, rawOff, "color.sparseZeroRun");
            if (symbols.size() + static_cast<size_t>(zeroRun) >=
                static_cast<size_t>(numSymbols)) {
                throw std::runtime_error("[PCATMStrictPaper decoder] invalid sparse color run");
            }
            symbols.insert(symbols.end(), static_cast<size_t>(zeroRun), 0u);
            if (rawOff >= packed.size()) {
                throw std::runtime_error("[PCATMStrictPaper decoder] truncated sparse color sym");
            }
            const uint8_t sym = packed[rawOff++];
            if ((colorMode == 2 && sym > 1u) || (colorMode == 4 && sym > 3u) ||
                sym == 0u) {
                throw std::runtime_error("[PCATMStrictPaper decoder] invalid sparse color sym");
            }
            symbols.push_back(sym);
        }
        if (symbols.size() < static_cast<size_t>(numSymbols)) {
            symbols.insert(symbols.end(),
                           static_cast<size_t>(numSymbols) - symbols.size(),
                           0u);
        }
        if (symbols.size() != static_cast<size_t>(numSymbols)) {
            throw std::runtime_error("[PCATMStrictPaper decoder] sparse color count mismatch");
        }
        return symbols;
    }
    std::vector<uint8_t> bits = pmsf::pcatm::unpackBits(
        packed.data(), static_cast<size_t>(numSymbols * bitsPerSymbol));
    std::vector<uint8_t> symbols;
    symbols.reserve(static_cast<size_t>(numSymbols));
    for (int32_t i = 0; i < numSymbols; ++i) {
        if (bitsPerSymbol == 1) {
            symbols.push_back(bits[static_cast<size_t>(i)] ? 1u : 0u);
        } else {
            const size_t j = static_cast<size_t>(i) * 2;
            symbols.push_back(static_cast<uint8_t>((bits[j] << 1) | bits[j + 1]));
        }
    }
    return symbols;
}

std::vector<std::vector<uint8_t>> decodeColorRoundsGlobal(
    std::span<const uint8_t> sl,
    int expectedRounds,
    std::vector<int>& modes) {
    size_t off = 0;
    const int32_t numRounds = readI32(sl, off, "globalColor.rounds");
    if (numRounds != expectedRounds || numRounds < 0) {
        throw std::runtime_error("[PCATMStrictPaper decoder] global color round mismatch");
    }
    modes.assign(static_cast<size_t>(numRounds), 4);
    std::vector<int32_t> counts(static_cast<size_t>(numRounds), 0);
    int32_t totalSymbols = 0;
    for (int32_t r = 0; r < numRounds; ++r) {
        const int32_t mode = readI32(sl, off, "globalColor.mode");
        const int32_t count = readI32(sl, off, "globalColor.count");
        if ((mode != 2 && mode != 4 && mode != 3) || count < 0) {
            throw std::runtime_error("[PCATMStrictPaper decoder] invalid global color round header");
        }
        modes[static_cast<size_t>(r)] = mode;
        counts[static_cast<size_t>(r)] = count;
        totalSymbols += count;
    }
    const bool allMode2 =
        !modes.empty() &&
        std::all_of(modes.begin(), modes.end(), [](int mode) { return mode == 2; });
    const int bitsPerSymbol = allMode2 ? 1 : 2;
    const int32_t encodedTotal = readI32(sl, off, "globalColor.total");
    const int32_t codec = readI32(sl, off, "globalColor.codec");
    const int32_t payloadBytes = readI32(sl, off, "globalColor.payloadBytes");
    if (encodedTotal != totalSymbols || payloadBytes < 0 ||
        off + static_cast<size_t>(payloadBytes) > sl.size()) {
        throw std::runtime_error("[PCATMStrictPaper decoder] invalid global color payload");
    }

    std::vector<uint32_t> flat;
    flat.reserve(static_cast<size_t>(totalSymbols));
    const uint8_t* payload = sl.data() + off;
    if (codec == 0 || codec == 1) {
        std::vector<uint8_t> packed;
        if (codec == 0) {
            packed.assign(payload, payload + payloadBytes);
        } else {
            const size_t expectedPacked =
                (static_cast<size_t>(totalSymbols) * static_cast<size_t>(bitsPerSymbol) + 7) / 8;
            packed = pmsf::pcatm::lzInflate(payload,
                                            static_cast<size_t>(payloadBytes),
                                            expectedPacked);
        }
        std::vector<uint8_t> bits = pmsf::pcatm::unpackBits(
            packed.data(),
            static_cast<size_t>(totalSymbols) * static_cast<size_t>(bitsPerSymbol));
        for (int32_t i = 0; i < totalSymbols; ++i) {
            if (bitsPerSymbol == 1) {
                flat.push_back(bits[static_cast<size_t>(i)] & 1u);
            } else {
                const size_t bi = static_cast<size_t>(i) * 2;
                flat.push_back(((bits[bi] & 1u) << 1) | (bits[bi + 1] & 1u));
            }
        }
    } else if (codec == 2) {
        pmsf::StaticHuffmanCoder coder;
        const size_t used = coder.decode(payload,
                                         static_cast<size_t>(payloadBytes),
                                         static_cast<size_t>(totalSymbols),
                                         flat);
        if (used > static_cast<size_t>(payloadBytes)) {
            throw std::runtime_error("[PCATMStrictPaper decoder] global color huffman overrun");
        }
    } else if (codec == 3) {
        pmsf::RangeCoderEntropyCoder coder(bitsPerSymbol);
        const size_t used = coder.decode(payload,
                                         static_cast<size_t>(payloadBytes),
                                         static_cast<size_t>(totalSymbols),
                                         flat);
        if (used > static_cast<size_t>(payloadBytes)) {
            throw std::runtime_error("[PCATMStrictPaper decoder] global color range overrun");
        }
    } else if (codec == 4 || codec == 5 || codec == 6) {
        size_t payloadOff = 0;
        const int32_t rawBytes = readI32(
            std::span<const uint8_t>(payload, static_cast<size_t>(payloadBytes)),
            payloadOff,
            codec == 4 ? "globalColor.rleRawBytes"
                       : (codec == 5 ? "globalColor.sparseRawBytes"
                                      : "globalColor.nzRunRawBytes"));
        if (rawBytes < 0 || payloadOff > static_cast<size_t>(payloadBytes)) {
            throw std::runtime_error("[PCATMStrictPaper decoder] invalid global color run header");
        }
        std::vector<uint8_t> rawRun = pmsf::pcatm::lzInflate(
            payload + payloadOff,
            static_cast<size_t>(payloadBytes) - payloadOff,
            static_cast<size_t>(rawBytes));
        size_t rawOff = 0;
        if (codec == 4) {
            while (flat.size() < static_cast<size_t>(totalSymbols)) {
                const uint32_t runLen = readVarU32(rawRun, rawOff, "globalColor.rleRun");
                const uint32_t sym = readVarU32(rawRun, rawOff, "globalColor.rleSym");
                if (runLen == 0 || sym >= (1u << bitsPerSymbol) ||
                    flat.size() + static_cast<size_t>(runLen) >
                        static_cast<size_t>(totalSymbols)) {
                    throw std::runtime_error("[PCATMStrictPaper decoder] invalid global color RLE");
                }
                flat.insert(flat.end(), static_cast<size_t>(runLen), sym);
            }
        } else if (codec == 5) {
            while (rawOff < rawRun.size()) {
                const uint32_t runLen = readVarU32(rawRun, rawOff, "globalColor.sparseZeroRun");
                const uint32_t sym = readVarU32(rawRun, rawOff, "globalColor.sparseSym");
                if (sym == 0u || sym >= (1u << bitsPerSymbol) ||
                    flat.size() + static_cast<size_t>(runLen) >=
                        static_cast<size_t>(totalSymbols)) {
                    throw std::runtime_error("[PCATMStrictPaper decoder] invalid global color sparse");
                }
                flat.insert(flat.end(), static_cast<size_t>(runLen), 0u);
                flat.push_back(sym);
            }
            if (flat.size() < static_cast<size_t>(totalSymbols)) {
                flat.insert(flat.end(),
                            static_cast<size_t>(totalSymbols) - flat.size(),
                            0u);
            }
        } else {
            while (rawOff < rawRun.size()) {
                const uint32_t zeroRun = readVarU32(rawRun, rawOff, "globalColor.nzZeroRun");
                const uint32_t runLen = readVarU32(rawRun, rawOff, "globalColor.nzRunLen");
                const uint32_t sym = readVarU32(rawRun, rawOff, "globalColor.nzSym");
                if (runLen == 0 || sym == 0u || sym >= (1u << bitsPerSymbol) ||
                    flat.size() + static_cast<size_t>(zeroRun) +
                            static_cast<size_t>(runLen) >
                        static_cast<size_t>(totalSymbols)) {
                    throw std::runtime_error("[PCATMStrictPaper decoder] invalid global color nz-run");
                }
                flat.insert(flat.end(), static_cast<size_t>(zeroRun), 0u);
                flat.insert(flat.end(), static_cast<size_t>(runLen), sym);
            }
            if (flat.size() < static_cast<size_t>(totalSymbols)) {
                flat.insert(flat.end(),
                            static_cast<size_t>(totalSymbols) - flat.size(),
                            0u);
            }
        }
        if (rawOff != rawRun.size() || flat.size() != static_cast<size_t>(totalSymbols)) {
            throw std::runtime_error("[PCATMStrictPaper decoder] global color run count mismatch");
        }
    } else if (codec == 7 || codec == 8) {
        int ctxBits = 8;
        const uint8_t* rangePayload = payload;
        size_t rangePayloadBytes = static_cast<size_t>(payloadBytes);
        if (codec == 8) {
            size_t ctxOff = 0;
            ctxBits = readI32(std::span<const uint8_t>(
                                  payload, static_cast<size_t>(payloadBytes)),
                              ctxOff,
                              "globalColor.ctxBits");
            if (ctxBits < 0 || ctxBits > 12 ||
                ctxOff > static_cast<size_t>(payloadBytes)) {
                throw std::runtime_error(
                    "[PCATMStrictPaper decoder] invalid adaptive ctx color payload");
            }
            rangePayload = payload + ctxOff;
            rangePayloadBytes = static_cast<size_t>(payloadBytes) - ctxOff;
        }
        std::vector<uint8_t> bits = decodeBitsContextRange(
            rangePayload,
            rangePayloadBytes,
            static_cast<size_t>(totalSymbols) * static_cast<size_t>(bitsPerSymbol),
            ctxBits);
        for (int32_t i = 0; i < totalSymbols; ++i) {
            if (bitsPerSymbol == 1) {
                flat.push_back(bits[static_cast<size_t>(i)] & 1u);
            } else {
                const size_t bi = static_cast<size_t>(i) * 2;
                flat.push_back(((bits[bi] & 1u) << 1) | (bits[bi + 1] & 1u));
            }
        }
    } else {
        throw std::runtime_error("[PCATMStrictPaper decoder] unknown global color codec");
    }
    if (flat.size() != static_cast<size_t>(totalSymbols)) {
        throw std::runtime_error("[PCATMStrictPaper decoder] global color count mismatch");
    }

    std::vector<std::vector<uint8_t>> rounds(static_cast<size_t>(numRounds));
    size_t cursor = 0;
    for (int32_t r = 0; r < numRounds; ++r) {
        auto& out = rounds[static_cast<size_t>(r)];
        out.reserve(static_cast<size_t>(counts[static_cast<size_t>(r)]));
        const uint32_t maxSym = modes[static_cast<size_t>(r)] == 2 ? 1u : 3u;  // mode 3/4 -> 2-bit
        for (int32_t i = 0; i < counts[static_cast<size_t>(r)]; ++i) {
            uint32_t sym = flat[cursor++];
            if (sym > maxSym) {
                throw std::runtime_error("[PCATMStrictPaper decoder] global color symbol OOB");
            }
            out.push_back(static_cast<uint8_t>(sym));
        }
    }
    return rounds;
}

// WS-B task-18: parse the mode-2 side-info v2 ("C2S2") appended after the
// per-round color stream. Returns the bit-packed record payload; empty when the
// tail magic is absent.
std::vector<uint8_t> decodeMode2SideInfo(std::span<const uint8_t> sl) {
    // WS-B task-24 (v5): locate the side container from the END of the round
    // stream via an explicit footer [sideLen u32][magic u32]. This is fully
    // decoupled from the color stream header, which in v4 did not describe the
    // actual per-round stream length for some candidate layouts.
    constexpr int32_t kFooterMagic = 0x544F4F46;  // "FOOT"
    if (sl.size() < 8) return {};
    size_t tail = sl.size() - 8;
    size_t o = tail;
    const int32_t sideLen = readI32(sl, o, "side.footerLen");
    const int32_t footerMagic = readI32(sl, o, "side.footerMagic");
    if (footerMagic != kFooterMagic) return {};
    if (sideLen < 12 ||
        static_cast<size_t>(sideLen) + 8 > sl.size()) {
        return {};
    }
    const size_t sideStart =
        sl.size() - 8 - static_cast<size_t>(sideLen);
    std::span<const uint8_t> side = sl.subspan(sideStart, static_cast<size_t>(sideLen));
    size_t o2 = 0;
    const int32_t magic = readI32(side, o2, "side.magic");
    if (magic != 0x33533243) return {};  // "C2S3"
    const int32_t rawBytes = readI32(side, o2, "side.rawBytes");
    const int32_t payloadLzBytes = readI32(side, o2, "side.payloadLzBytes");
    if (rawBytes < 0 || payloadLzBytes < 0 ||
        o2 + static_cast<size_t>(payloadLzBytes) > side.size()) {
        return {};
    }
    std::vector<uint8_t> payload = pmsf::pcatm::lzInflate(
        side.data() + o2, static_cast<size_t>(payloadLzBytes),
        static_cast<size_t>(rawBytes));
    if (payload.size() != static_cast<size_t>(rawBytes)) return {};
    if (isEnvOn("PMSF_PCATM_SIDE_DUMP")) {
        std::cerr << "[2c-side-parse] stream=" << sl.size()
                  << " sideLen=" << sideLen
                  << " sideStart=" << sideStart
                  << " rawBytes=" << rawBytes
                  << " plz=" << payloadLzBytes
                  << " payload=" << payload.size() << " hex=";
        for (std::size_t bi = 0;
             bi < std::min<std::size_t>(8, payload.size()); ++bi) {
            char buf[4];
            std::snprintf(buf, sizeof(buf), "%02x", payload[bi]);
            std::cerr << buf;
        }
        std::cerr << std::endl;
    }
    return payload;
}

// Sequential MSB-first bit reader over the side-info payload.
class SideBitReader {
  public:
    explicit SideBitReader(const std::vector<uint8_t>& data) : data_(data) {}
    uint32_t readBit() {
        if (bitPos_ >= data_.size() * 8) {
            throw std::runtime_error(
                "[PCATMStrictPaper decoder] mode-2 side bitstream underflow");
        }
        const uint32_t v =
            (data_[bitPos_ / 8] >> (7 - static_cast<int>(bitPos_ % 8))) & 1u;
        ++bitPos_;
        return v;
    }
    uint32_t readBits(int n) {
        uint32_t v = 0;
        for (int i = 0; i < n; ++i) v = (v << 1) | readBit();
        return v;
    }

  private:
    const std::vector<uint8_t>& data_;
    std::size_t bitPos_ = 0;
};

void decodeValenceRaw(std::span<const uint8_t> sl,
                      std::deque<uint8_t>& valCodes,
                      std::deque<uint8_t>& otherVals) {
    size_t off = 0;
    const int32_t numVal = readI32(sl, off, "valence.numVal");
    const int32_t numOther = readI32(sl, off, "valence.numOther");
    if (numVal < 0 || numOther < 0 ||
        off + static_cast<size_t>(numVal + numOther) > sl.size()) {
        throw std::runtime_error("[PCATMStrictPaper decoder] invalid valence side stream");
    }
    for (int32_t i = 0; i < numVal; ++i) {
        valCodes.push_back(sl[off++]);
    }
    for (int32_t i = 0; i < numOther; ++i) {
        otherVals.push_back(sl[off++]);
    }
}

void decodeBitsLzRaw(std::span<const uint8_t> sl, std::deque<uint8_t>& out) {
    size_t off = 0;
    const int32_t numBits = readI32(sl, off, "bits.numBits");
    const int32_t packedBytes = readI32(sl, off, "bits.packedBytes");
    const int32_t lzBytes = readI32(sl, off, "bits.lzBytes");
    if (numBits < 0 || packedBytes < 0 || lzBytes < 0 ||
        off + static_cast<size_t>(lzBytes) > sl.size()) {
        throw std::runtime_error("[PCATMStrictPaper decoder] invalid bit side stream");
    }
    std::vector<uint8_t> packed = pmsf::pcatm::lzInflate(
        sl.data() + off, static_cast<size_t>(lzBytes), static_cast<size_t>(packedBytes));
    std::vector<uint8_t> bits = pmsf::pcatm::unpackBits(
        packed.data(), static_cast<size_t>(numBits));
    for (uint8_t bit : bits) out.push_back(bit ? 1u : 0u);
}

std::vector<MCGAL::PointInt> decodeResidualHuffman(std::span<const uint8_t> sl) {
    size_t off = 0;
    const int32_t numResiduals = readI32(sl, off, "residual.numResiduals");
    const int32_t numCoords = readI32(sl, off, "residual.numCoords");
    const int32_t huffmanBytes = readI32(sl, off, "residual.huffmanBytes");
    if (numResiduals < 0 || numCoords < 0 || huffmanBytes < 0) {
        throw std::runtime_error("[PCATMStrictPaper decoder] negative residual stream size");
    }
    if (numCoords != numResiduals * 3) {
        throw std::runtime_error(
            "[PCATMStrictPaper decoder] residual coord count does not match vertex count");
    }
    if (off + static_cast<size_t>(huffmanBytes) > sl.size()) {
        throw std::runtime_error("[PCATMStrictPaper decoder] truncated residual Huffman payload");
    }

    std::vector<uint32_t> symbols;
    symbols.reserve(static_cast<size_t>(numCoords));
    pmsf::StaticHuffmanCoder coder;
    const size_t used = coder.decode(
        sl.data() + off,
        static_cast<size_t>(huffmanBytes),
        static_cast<size_t>(numCoords),
        symbols);
    if (used > static_cast<size_t>(huffmanBytes)) {
        throw std::runtime_error("[PCATMStrictPaper decoder] Huffman decoder over-read");
    }
    if (symbols.size() != static_cast<size_t>(numCoords)) {
        throw std::runtime_error("[PCATMStrictPaper decoder] Huffman symbol count mismatch");
    }

    std::vector<MCGAL::PointInt> residuals;
    residuals.reserve(static_cast<size_t>(numResiduals));
    for (size_t i = 0; i < symbols.size(); i += 3) {
        residuals.emplace_back(
            unzigzag32(symbols[i + 0]),
            unzigzag32(symbols[i + 1]),
            unzigzag32(symbols[i + 2]));
    }
    return residuals;
}

std::vector<std::vector<MCGAL::PointInt>> decodeResidualGlobalHuffman(
    std::span<const uint8_t> sl,
    int expectedRounds) {
    size_t off = 0;
    const int32_t numRounds = readI32(sl, off, "globalResidual.numRounds");
    if (numRounds != expectedRounds) {
        throw std::runtime_error(
            "[PCATMStrictPaper decoder] global residual round count mismatch");
    }

    std::vector<int32_t> counts;
    counts.reserve(static_cast<size_t>(numRounds));
    int32_t sumResiduals = 0;
    for (int32_t r = 0; r < numRounds; ++r) {
        int32_t count = readI32(sl, off, "globalResidual.roundCount");
        if (count < 0) {
            throw std::runtime_error("[PCATMStrictPaper decoder] negative global residual count");
        }
        counts.push_back(count);
        sumResiduals += count;
    }

    const int32_t totalResiduals = readI32(sl, off, "globalResidual.totalResiduals");
    const int32_t numCoords = readI32(sl, off, "globalResidual.numCoords");
    const int32_t huffmanBytes = readI32(sl, off, "globalResidual.huffmanBytes");
    if (totalResiduals != sumResiduals || numCoords != totalResiduals * 3 ||
        huffmanBytes < 0) {
        throw std::runtime_error("[PCATMStrictPaper decoder] invalid global residual header");
    }
    if (off + static_cast<size_t>(huffmanBytes) > sl.size()) {
        throw std::runtime_error(
            "[PCATMStrictPaper decoder] truncated global residual Huffman payload");
    }

    std::vector<uint32_t> symbols;
    symbols.reserve(static_cast<size_t>(numCoords));
    pmsf::StaticHuffmanCoder coder;
    const size_t used = coder.decode(
        sl.data() + off,
        static_cast<size_t>(huffmanBytes),
        static_cast<size_t>(numCoords),
        symbols);
    if (used > static_cast<size_t>(huffmanBytes) ||
        symbols.size() != static_cast<size_t>(numCoords)) {
        throw std::runtime_error(
            "[PCATMStrictPaper decoder] global Huffman symbol count mismatch");
    }

    std::vector<std::vector<MCGAL::PointInt>> perRound(static_cast<size_t>(numRounds));
    size_t cursor = 0;
    for (int32_t r = 0; r < numRounds; ++r) {
        auto& out = perRound[static_cast<size_t>(r)];
        out.reserve(static_cast<size_t>(counts[static_cast<size_t>(r)]));
        for (int32_t i = 0; i < counts[static_cast<size_t>(r)]; ++i) {
            out.emplace_back(
                unzigzag32(symbols[cursor + 0]),
                unzigzag32(symbols[cursor + 1]),
                unzigzag32(symbols[cursor + 2]));
            cursor += 3;
        }
    }
    return perRound;
}

std::vector<std::vector<MCGAL::PointInt>> decodeResidualGlobalVarintZlib(
    std::span<const uint8_t> sl,
    int expectedRounds) {
    size_t off = 0;
    const int32_t numRounds = readI32(sl, off, "globalResidual.numRounds");
    if (numRounds != expectedRounds) {
        throw std::runtime_error(
            "[PCATMStrictPaper decoder] global residual round count mismatch");
    }

    std::vector<int32_t> counts;
    counts.reserve(static_cast<size_t>(numRounds));
    int32_t sumResiduals = 0;
    for (int32_t r = 0; r < numRounds; ++r) {
        const int32_t count = readI32(sl, off, "globalResidual.roundCount");
        if (count < 0) {
            throw std::runtime_error("[PCATMStrictPaper decoder] negative global residual count");
        }
        counts.push_back(count);
        sumResiduals += count;
    }

    const int32_t totalResiduals = readI32(sl, off, "globalResidual.totalResiduals");
    const int32_t numCoords = readI32(sl, off, "globalResidual.numCoords");
    const int32_t rawBytes = readI32(sl, off, "globalResidual.rawBytes");
    const int32_t lzBytes = readI32(sl, off, "globalResidual.lzBytes");
    if (totalResiduals != sumResiduals || numCoords != totalResiduals * 3 ||
        rawBytes < 0 || lzBytes < 0) {
        throw std::runtime_error("[PCATMStrictPaper decoder] invalid global varint residual header");
    }
    if (off + static_cast<size_t>(lzBytes) > sl.size()) {
        throw std::runtime_error(
            "[PCATMStrictPaper decoder] truncated global varint residual payload");
    }

    std::vector<uint8_t> raw = pmsf::pcatm::lzInflate(
        sl.data() + off, static_cast<size_t>(lzBytes), static_cast<size_t>(rawBytes));
    std::vector<std::vector<MCGAL::PointInt>> perRound(static_cast<size_t>(numRounds));
    size_t rawOff = 0;
    int32_t decodedResiduals = 0;
    for (int32_t r = 0; r < numRounds; ++r) {
        auto& out = perRound[static_cast<size_t>(r)];
        out.reserve(static_cast<size_t>(counts[static_cast<size_t>(r)]));
        for (int32_t i = 0; i < counts[static_cast<size_t>(r)]; ++i) {
            const int32_t x = unzigzag32(readVarU32(raw, rawOff, "residual.x"));
            const int32_t y = unzigzag32(readVarU32(raw, rawOff, "residual.y"));
            const int32_t z = unzigzag32(readVarU32(raw, rawOff, "residual.z"));
            out.emplace_back(x, y, z);
            ++decodedResiduals;
        }
    }
    if (decodedResiduals != totalResiduals || rawOff != raw.size()) {
        throw std::runtime_error("[PCATMStrictPaper decoder] global varint residual count mismatch");
    }
    return perRound;
}

std::vector<std::vector<MCGAL::PointInt>> decodeResidualGlobalVarintByteRange(
    std::span<const uint8_t> sl,
    int expectedRounds) {
    size_t off = 0;
    const int32_t numRounds = readI32(sl, off, "globalResidual.numRounds");
    if (numRounds != expectedRounds) {
        throw std::runtime_error(
            "[PCATMStrictPaper decoder] global residual round count mismatch");
    }

    std::vector<int32_t> counts;
    counts.reserve(static_cast<size_t>(numRounds));
    int32_t sumResiduals = 0;
    for (int32_t r = 0; r < numRounds; ++r) {
        const int32_t count = readI32(sl, off, "globalResidual.roundCount");
        if (count < 0) {
            throw std::runtime_error("[PCATMStrictPaper decoder] negative global residual count");
        }
        counts.push_back(count);
        sumResiduals += count;
    }

    const int32_t totalResiduals = readI32(sl, off, "globalResidual.totalResiduals");
    const int32_t numCoords = readI32(sl, off, "globalResidual.numCoords");
    const int32_t rawBytes = readI32(sl, off, "globalResidual.rawBytes");
    const int32_t payloadBytes = readI32(sl, off, "globalResidual.rangeBytes");
    if (totalResiduals != sumResiduals || numCoords != totalResiduals * 3 ||
        rawBytes < 0 || payloadBytes < 0 ||
        off + static_cast<size_t>(payloadBytes) > sl.size()) {
        throw std::runtime_error(
            "[PCATMStrictPaper decoder] invalid global byte-range residual header");
    }

    std::vector<uint32_t> decodedBytes;
    pmsf::RangeCoderEntropyCoder coder(8);
    const size_t used = coder.decode(sl.data() + off,
                                     static_cast<size_t>(payloadBytes),
                                     static_cast<size_t>(rawBytes),
                                     decodedBytes);
    if (used > static_cast<size_t>(payloadBytes) ||
        decodedBytes.size() != static_cast<size_t>(rawBytes)) {
        throw std::runtime_error(
            "[PCATMStrictPaper decoder] global byte-range residual count mismatch");
    }

    std::vector<uint8_t> raw;
    raw.reserve(decodedBytes.size());
    for (uint32_t b : decodedBytes) raw.push_back(static_cast<uint8_t>(b & 0xffu));

    std::vector<std::vector<MCGAL::PointInt>> perRound(static_cast<size_t>(numRounds));
    size_t rawOff = 0;
    int32_t decodedResiduals = 0;
    for (int32_t r = 0; r < numRounds; ++r) {
        auto& out = perRound[static_cast<size_t>(r)];
        out.reserve(static_cast<size_t>(counts[static_cast<size_t>(r)]));
        for (int32_t i = 0; i < counts[static_cast<size_t>(r)]; ++i) {
            const int32_t x = unzigzag32(readVarU32(raw, rawOff, "residual.x"));
            const int32_t y = unzigzag32(readVarU32(raw, rawOff, "residual.y"));
            const int32_t z = unzigzag32(readVarU32(raw, rawOff, "residual.z"));
            out.emplace_back(x, y, z);
            ++decodedResiduals;
        }
    }
    if (decodedResiduals != totalResiduals || rawOff != raw.size()) {
        throw std::runtime_error(
            "[PCATMStrictPaper decoder] global byte-range residual varint mismatch");
    }
    return perRound;
}

std::vector<std::vector<MCGAL::PointInt>> decodeResidualGlobalAxisVarintZlib(
    std::span<const uint8_t> sl,
    int expectedRounds) {
    size_t off = 0;
    const int32_t numRounds = readI32(sl, off, "globalResidual.numRounds");
    if (numRounds != expectedRounds) {
        throw std::runtime_error(
            "[PCATMStrictPaper decoder] global residual round count mismatch");
    }

    std::vector<int32_t> counts;
    counts.reserve(static_cast<size_t>(numRounds));
    int32_t sumResiduals = 0;
    for (int32_t r = 0; r < numRounds; ++r) {
        const int32_t count = readI32(sl, off, "globalResidual.roundCount");
        if (count < 0) {
            throw std::runtime_error("[PCATMStrictPaper decoder] negative global residual count");
        }
        counts.push_back(count);
        sumResiduals += count;
    }

    const int32_t totalResiduals = readI32(sl, off, "globalResidual.totalResiduals");
    const int32_t numCoords = readI32(sl, off, "globalResidual.numCoords");
    const int32_t rawBytes = readI32(sl, off, "globalResidual.rawBytes");
    const int32_t lzBytes = readI32(sl, off, "globalResidual.lzBytes");
    if (totalResiduals != sumResiduals || numCoords != totalResiduals * 3 ||
        rawBytes < 0 || lzBytes < 0) {
        throw std::runtime_error("[PCATMStrictPaper decoder] invalid global axis varint residual header");
    }
    if (off + static_cast<size_t>(lzBytes) > sl.size()) {
        throw std::runtime_error(
            "[PCATMStrictPaper decoder] truncated global axis varint residual payload");
    }

    std::vector<uint8_t> raw = pmsf::pcatm::lzInflate(
        sl.data() + off, static_cast<size_t>(lzBytes), static_cast<size_t>(rawBytes));
    std::vector<MCGAL::PointInt> flat(static_cast<size_t>(totalResiduals));
    size_t rawOff = 0;
    for (int32_t i = 0; i < totalResiduals; ++i) {
        flat[static_cast<size_t>(i)] = MCGAL::PointInt(
            unzigzag32(readVarU32(raw, rawOff, "residual.x")), 0, 0);
    }
    for (int32_t i = 0; i < totalResiduals; ++i) {
        flat[static_cast<size_t>(i)] = MCGAL::PointInt(
            flat[static_cast<size_t>(i)].x(),
            unzigzag32(readVarU32(raw, rawOff, "residual.y")),
            0);
    }
    for (int32_t i = 0; i < totalResiduals; ++i) {
        flat[static_cast<size_t>(i)] = MCGAL::PointInt(
            flat[static_cast<size_t>(i)].x(),
            flat[static_cast<size_t>(i)].y(),
            unzigzag32(readVarU32(raw, rawOff, "residual.z")));
    }
    if (rawOff != raw.size()) {
        throw std::runtime_error("[PCATMStrictPaper decoder] trailing axis varint residual bytes");
    }

    std::vector<std::vector<MCGAL::PointInt>> perRound(static_cast<size_t>(numRounds));
    size_t cursor = 0;
    for (int32_t r = 0; r < numRounds; ++r) {
        auto& out = perRound[static_cast<size_t>(r)];
        const size_t count = static_cast<size_t>(counts[static_cast<size_t>(r)]);
        out.reserve(count);
        for (size_t i = 0; i < count; ++i) {
            out.push_back(flat[cursor++]);
        }
    }
    return perRound;
}

std::vector<std::vector<MCGAL::PointInt>> decodeResidualGlobalSplitAxisVarintZlib(
    std::span<const uint8_t> sl,
    int expectedRounds) {
    size_t off = 0;
    const int32_t numRounds = readI32(sl, off, "globalResidual.numRounds");
    if (numRounds != expectedRounds) {
        throw std::runtime_error(
            "[PCATMStrictPaper decoder] global residual round count mismatch");
    }

    std::vector<int32_t> counts;
    counts.reserve(static_cast<size_t>(numRounds));
    int32_t sumResiduals = 0;
    for (int32_t r = 0; r < numRounds; ++r) {
        const int32_t count = readI32(sl, off, "globalResidual.roundCount");
        if (count < 0) {
            throw std::runtime_error("[PCATMStrictPaper decoder] negative global residual count");
        }
        counts.push_back(count);
        sumResiduals += count;
    }

    const int32_t totalResiduals = readI32(sl, off, "globalResidual.totalResiduals");
    const int32_t numCoords = readI32(sl, off, "globalResidual.numCoords");
    const int32_t rawXBytes = readI32(sl, off, "globalResidual.rawXBytes");
    const int32_t lzXBytes = readI32(sl, off, "globalResidual.lzXBytes");
    const int32_t rawYBytes = readI32(sl, off, "globalResidual.rawYBytes");
    const int32_t lzYBytes = readI32(sl, off, "globalResidual.lzYBytes");
    const int32_t rawZBytes = readI32(sl, off, "globalResidual.rawZBytes");
    const int32_t lzZBytes = readI32(sl, off, "globalResidual.lzZBytes");
    if (totalResiduals != sumResiduals || numCoords != totalResiduals * 3 ||
        rawXBytes < 0 || lzXBytes < 0 || rawYBytes < 0 || lzYBytes < 0 ||
        rawZBytes < 0 || lzZBytes < 0) {
        throw std::runtime_error(
            "[PCATMStrictPaper decoder] invalid global split-axis varint residual header");
    }
    if (off + static_cast<size_t>(lzXBytes) + static_cast<size_t>(lzYBytes) +
            static_cast<size_t>(lzZBytes) > sl.size()) {
        throw std::runtime_error(
            "[PCATMStrictPaper decoder] truncated global split-axis varint residual payload");
    }

    std::vector<uint8_t> rawX = pmsf::pcatm::lzInflate(
        sl.data() + off, static_cast<size_t>(lzXBytes), static_cast<size_t>(rawXBytes));
    off += static_cast<size_t>(lzXBytes);
    std::vector<uint8_t> rawY = pmsf::pcatm::lzInflate(
        sl.data() + off, static_cast<size_t>(lzYBytes), static_cast<size_t>(rawYBytes));
    off += static_cast<size_t>(lzYBytes);
    std::vector<uint8_t> rawZ = pmsf::pcatm::lzInflate(
        sl.data() + off, static_cast<size_t>(lzZBytes), static_cast<size_t>(rawZBytes));

    std::vector<MCGAL::PointInt> flat(static_cast<size_t>(totalResiduals));
    size_t offX = 0;
    size_t offY = 0;
    size_t offZ = 0;
    for (int32_t i = 0; i < totalResiduals; ++i) {
        flat[static_cast<size_t>(i)] = MCGAL::PointInt(
            unzigzag32(readVarU32(rawX, offX, "residual.x")),
            unzigzag32(readVarU32(rawY, offY, "residual.y")),
            unzigzag32(readVarU32(rawZ, offZ, "residual.z")));
    }
    if (offX != rawX.size() || offY != rawY.size() || offZ != rawZ.size()) {
        throw std::runtime_error(
            "[PCATMStrictPaper decoder] trailing split-axis varint residual bytes");
    }

    std::vector<std::vector<MCGAL::PointInt>> perRound(static_cast<size_t>(numRounds));
    size_t cursor = 0;
    for (int32_t r = 0; r < numRounds; ++r) {
        auto& out = perRound[static_cast<size_t>(r)];
        const size_t count = static_cast<size_t>(counts[static_cast<size_t>(r)]);
        out.reserve(count);
        for (size_t i = 0; i < count; ++i) {
            out.push_back(flat[cursor++]);
        }
    }
    return perRound;
}

std::vector<std::vector<MCGAL::PointInt>> decodeResidualGlobalMeppAxisRange(
    std::span<const uint8_t> sl,
    int expectedRounds) {
    constexpr int32_t kMeppAxisWrapperMagic = -1101;
    constexpr int32_t kMeppAxisVariantRange = 0;
    constexpr int32_t kMeppAxisVariantVarintZlib = 1;

    size_t off = 0;
    int32_t numRounds = readI32(sl, off, "globalResidual.numRoundsOrMagic");
    if (numRounds == kMeppAxisWrapperMagic) {
        const int32_t variant = readI32(sl, off, "globalResidual.meppVariant");
        const int32_t payloadBytes = readI32(sl, off, "globalResidual.meppPayloadBytes");
        if (payloadBytes < 0 || off + static_cast<size_t>(payloadBytes) > sl.size()) {
            throw std::runtime_error(
                "[PCATMStrictPaper decoder] invalid MEPP axis wrapper payload");
        }
        std::span<const uint8_t> payload(sl.data() + off, static_cast<size_t>(payloadBytes));
        if (variant == kMeppAxisVariantVarintZlib) {
            return decodeResidualGlobalVarintZlib(payload, expectedRounds);
        }
        if (variant != kMeppAxisVariantRange) {
            throw std::runtime_error(
                "[PCATMStrictPaper decoder] unknown MEPP axis residual variant");
        }
        sl = payload;
        off = 0;
        numRounds = readI32(sl, off, "globalResidual.numRounds");
    }
    if (numRounds != expectedRounds) {
        throw std::runtime_error(
            "[PCATMStrictPaper decoder] global residual round count mismatch");
    }

    std::vector<int32_t> counts;
    counts.reserve(static_cast<size_t>(numRounds));
    int32_t sumResiduals = 0;
    for (int32_t r = 0; r < numRounds; ++r) {
        const int32_t count = readI32(sl, off, "globalResidual.roundCount");
        if (count < 0) {
            throw std::runtime_error("[PCATMStrictPaper decoder] negative global residual count");
        }
        counts.push_back(count);
        sumResiduals += count;
    }

    const int32_t totalResiduals = readI32(sl, off, "globalResidual.totalResiduals");
    const int32_t numCoords = readI32(sl, off, "globalResidual.numCoords");
    const int32_t axisCount = readI32(sl, off, "globalResidual.axisCount");
    if (totalResiduals != sumResiduals || numCoords != totalResiduals * 3 || axisCount != 3) {
        throw std::runtime_error(
            "[PCATMStrictPaper decoder] invalid MEPP axis range residual header");
    }

    std::array<std::vector<int32_t>, 3> axisValues;
    for (int axisIndex = 0; axisIndex < 3; ++axisIndex) {
        const int32_t mode = readI32(sl, off, "globalResidual.axisMode");
        const int32_t minValue = readI32(sl, off, "globalResidual.axisMin");
        const int32_t alphabetSize = readI32(sl, off, "globalResidual.axisAlphabet");
        const int32_t bits = readI32(sl, off, "globalResidual.axisBits");
        const int32_t codebookSize = readI32(sl, off, "globalResidual.axisCodebook");
        const int32_t payloadBytes = readI32(sl, off, "globalResidual.axisPayload");
        if ((mode != 0 && mode != 1) || alphabetSize <= 0 || bits <= 0 || bits > 16 ||
            codebookSize < 0 || payloadBytes < 0) {
            throw std::runtime_error(
                "[PCATMStrictPaper decoder] invalid MEPP axis residual metadata");
        }
        if ((mode == 0 && codebookSize != 0) || (mode == 1 && codebookSize != alphabetSize)) {
            throw std::runtime_error(
                "[PCATMStrictPaper decoder] inconsistent MEPP axis residual codebook");
        }
        if (off + static_cast<size_t>(codebookSize) * 4u +
                static_cast<size_t>(payloadBytes) > sl.size()) {
            throw std::runtime_error(
                "[PCATMStrictPaper decoder] truncated MEPP axis residual payload");
        }

        std::vector<int32_t> codebook;
        codebook.reserve(static_cast<size_t>(codebookSize));
        for (int32_t i = 0; i < codebookSize; ++i) {
            codebook.push_back(readI32(sl, off, "globalResidual.axisCodebookValue"));
        }

        std::vector<uint32_t> ranks;
        if (totalResiduals > 0) {
            pmsf::RangeCoderEntropyCoder coder(bits);
            const size_t used = coder.decode(sl.data() + off,
                                             static_cast<size_t>(payloadBytes),
                                             static_cast<size_t>(totalResiduals),
                                             ranks);
            if (used > static_cast<size_t>(payloadBytes) ||
                ranks.size() != static_cast<size_t>(totalResiduals)) {
                throw std::runtime_error(
                    "[PCATMStrictPaper decoder] MEPP axis residual count mismatch");
            }
        }
        off += static_cast<size_t>(payloadBytes);

        auto& values = axisValues[static_cast<size_t>(axisIndex)];
        values.reserve(static_cast<size_t>(totalResiduals));
        for (uint32_t rank : ranks) {
            if (rank >= static_cast<uint32_t>(alphabetSize)) {
                throw std::runtime_error(
                    "[PCATMStrictPaper decoder] MEPP axis residual rank OOB");
            }
            if (mode == 0) {
                values.push_back(static_cast<int32_t>(
                    static_cast<int64_t>(minValue) + static_cast<int64_t>(rank)));
            } else {
                if (rank >= codebook.size()) {
                    throw std::runtime_error(
                        "[PCATMStrictPaper decoder] MEPP axis residual codebook rank OOB");
                }
                values.push_back(codebook[rank]);
            }
        }
    }

    std::vector<std::vector<MCGAL::PointInt>> perRound(static_cast<size_t>(numRounds));
    size_t cursor = 0;
    for (int32_t r = 0; r < numRounds; ++r) {
        auto& out = perRound[static_cast<size_t>(r)];
        const size_t count = static_cast<size_t>(counts[static_cast<size_t>(r)]);
        out.reserve(count);
        for (size_t i = 0; i < count; ++i) {
            out.emplace_back(axisValues[0][cursor],
                             axisValues[1][cursor],
                             axisValues[2][cursor]);
            ++cursor;
        }
    }
    return perRound;
}

std::vector<std::vector<MCGAL::PointInt>> decodeResidualGlobalDenseRange(
    std::span<const uint8_t> sl,
    int expectedRounds) {
    size_t off = 0;
    const int32_t numRounds = readI32(sl, off, "globalResidual.numRounds");
    if (numRounds != expectedRounds) {
        throw std::runtime_error(
            "[PCATMStrictPaper decoder] global residual round count mismatch");
    }

    std::vector<int32_t> counts;
    counts.reserve(static_cast<size_t>(numRounds));
    int32_t sumResiduals = 0;
    for (int32_t r = 0; r < numRounds; ++r) {
        const int32_t count = readI32(sl, off, "globalResidual.roundCount");
        if (count < 0) {
            throw std::runtime_error("[PCATMStrictPaper decoder] negative global residual count");
        }
        counts.push_back(count);
        sumResiduals += count;
    }

    const int32_t totalResiduals = readI32(sl, off, "globalResidual.totalResiduals");
    const int32_t numCoords = readI32(sl, off, "globalResidual.numCoords");
    const int32_t alphabetSize = readI32(sl, off, "globalResidual.alphabetSize");
    const int32_t rcBytes = readI32(sl, off, "globalResidual.rcBytes");
    if (totalResiduals != sumResiduals || numCoords != totalResiduals * 3 ||
        alphabetSize < 0 || rcBytes < 0) {
        throw std::runtime_error("[PCATMStrictPaper decoder] invalid dense range residual header");
    }
    if (off + static_cast<size_t>(alphabetSize) * 4u + static_cast<size_t>(rcBytes) > sl.size()) {
        throw std::runtime_error(
            "[PCATMStrictPaper decoder] truncated dense range residual payload");
    }

    std::vector<uint32_t> codebook;
    codebook.reserve(static_cast<size_t>(alphabetSize));
    for (int32_t i = 0; i < alphabetSize; ++i) {
        codebook.push_back(static_cast<uint32_t>(readI32(sl, off, "globalResidual.codebook")));
    }

    std::vector<uint32_t> ranks;
    ranks.reserve(static_cast<size_t>(numCoords));
    pmsf::RangeCoderEntropyCoder coder(ceilLog2Size(std::max<int32_t>(alphabetSize, 1)));
    const size_t used = coder.decode(sl.data() + off,
                                     static_cast<size_t>(rcBytes),
                                     static_cast<size_t>(numCoords),
                                     ranks);
    if (used > static_cast<size_t>(rcBytes) ||
        ranks.size() != static_cast<size_t>(numCoords)) {
        throw std::runtime_error("[PCATMStrictPaper decoder] dense range residual count mismatch");
    }

    std::vector<std::vector<MCGAL::PointInt>> perRound(static_cast<size_t>(numRounds));
    size_t cursor = 0;
    for (int32_t r = 0; r < numRounds; ++r) {
        auto& out = perRound[static_cast<size_t>(r)];
        out.reserve(static_cast<size_t>(counts[static_cast<size_t>(r)]));
        for (int32_t i = 0; i < counts[static_cast<size_t>(r)]; ++i) {
            const uint32_t rx = ranks[cursor + 0];
            const uint32_t ry = ranks[cursor + 1];
            const uint32_t rz = ranks[cursor + 2];
            if (rx >= codebook.size() || ry >= codebook.size() || rz >= codebook.size()) {
                throw std::runtime_error("[PCATMStrictPaper decoder] dense range residual rank OOB");
            }
            out.emplace_back(
                unzigzag32(codebook[rx]),
                unzigzag32(codebook[ry]),
                unzigzag32(codebook[rz]));
            cursor += 3;
        }
    }
    return perRound;
}

}  // namespace

void PCATMStrictPMSFSymbolReadOperator::importFromRangeCodedBuffer(
    char* buffer,
    size_t& dataOffset,
    unsigned /*nbQuantBits*/) {

    constexpr size_t kArbitraryLargeView = static_cast<size_t>(1) << 32;
    std::span<const uint8_t> view(reinterpret_cast<const uint8_t*>(buffer + dataOffset),
                                  kArbitraryLargeView);

    pmsf::ConnectivityBlobReader rd(view);
    const auto& hdr = rd.header();
    orientedZRecovery_ =
        hdr.scheme_version == pmsf::kPCATMStrictPaperOrientedZVersion;
    if (!pmsf::paperBlobSchemeVersionIsPCATMStrictPaper(
            hdr.scheme_version)) {
        throw std::runtime_error(
            "[PCATMStrictPaper decoder] expected scheme_version=1..12 got " +
            std::to_string(hdr.scheme_version));
    }
    const auto spec = pmsf::pcatmStrictPaperConnectivityBlobSpec(
        static_cast<std::int32_t>(hdr.num_rounds),
        static_cast<std::int32_t>(hdr.scheme_version),
        pmsf::paperBlobStreamCountPCATMStrictPaperForScheme(
            hdr.scheme_version));
    pmsf::requirePaperConnectivityBlobHeaderMatchesSpec(
        hdr, spec, "PCATMStrictPaper");
    const bool hasTopoSideStreams =
        pmsf::paperBlobSchemeVersionPCATMStrictPaperHasTopologySideStreams(
            hdr.scheme_version);
    const bool colorOnly =
        pmsf::paperBlobSchemeVersionPCATMStrictPaperIsColorOnly(
            hdr.scheme_version);

    totalColorBits_ = 0;
    totalResiduals_ = 0;
    decodedRounds_ = static_cast<int>(hdr.num_rounds);
    rcDecodedColors_.clear();
    rcDecodedResiduals_.clear();

    rcDecodedValCodes_.clear();
    rcDecodedOtherVals_.clear();
    rcDecodedIsPoly_.clear();
    faceBitsPerRound_.assign(decodedRounds_, 0);
    colorModePerRound_.assign(decodedRounds_, 2);
    colorSymbolsPerRound_.assign(decodedRounds_, {});
    residualsPerRound_.assign(decodedRounds_, 0);
    mode2SidePerRound_.assign(static_cast<size_t>(decodedRounds_), {});

    const bool globalAdaptiveResidual =
        pmsf::paperBlobSchemeVersionPCATMStrictPaperUsesAdaptiveResidual(
            hdr.scheme_version);
    const bool globalHuffman =
        pmsf::paperBlobSchemeVersionPCATMStrictPaperUsesGlobalHuffmanResidual(
            hdr.scheme_version);
    const bool globalVarintResidual =
        pmsf::paperBlobSchemeVersionPCATMStrictPaperUsesGlobalVarintResidual(
            hdr.scheme_version);
    const bool globalDenseRangeResidual =
        pmsf::paperBlobSchemeVersionPCATMStrictPaperUsesDenseRangeResidual(
            hdr.scheme_version);
    const bool globalAxisVarintResidual =
        pmsf::paperBlobSchemeVersionPCATMStrictPaperUsesAxisVarintResidual(
            hdr.scheme_version);
    const bool globalSplitAxisVarintResidual =
        pmsf::paperBlobSchemeVersionPCATMStrictPaperUsesSplitAxisVarintResidual(
            hdr.scheme_version);
    const bool globalByteRangeResidual =
        pmsf::paperBlobSchemeVersionPCATMStrictPaperUsesByteRangeResidual(
            hdr.scheme_version);
    const bool globalMeppAxisRangeResidual =
        pmsf::paperBlobSchemeVersionPCATMStrictPaperUsesMeppAxisRangeResidual(
            hdr.scheme_version);
    std::vector<std::vector<MCGAL::PointInt>> globalResiduals;
    if (globalAdaptiveResidual) {
        const auto adaptStream =
            rd.getStream(0, pmsf::kPaperBlobStreamPCATMStrictPaperGeometry);
        auto adaptRounds = pmsf::pcatmstrict::ard::decode(
            adaptStream.data(), adaptStream.size(), decodedRounds_);
        globalResiduals.resize(static_cast<std::size_t>(decodedRounds_));
        for (int r = 0; r < decodedRounds_; ++r) {
            auto& dst = globalResiduals[static_cast<std::size_t>(r)];
            for (const auto& v : adaptRounds[static_cast<std::size_t>(r)])
                dst.emplace_back(v[0], v[1], v[2]);
        }
    } else if (globalHuffman) {
        globalResiduals = decodeResidualGlobalHuffman(
            rd.getStream(0, pmsf::kPaperBlobStreamPCATMStrictPaperGeometry),
            decodedRounds_);
    } else if (globalVarintResidual) {
        globalResiduals = decodeResidualGlobalVarintZlib(
            rd.getStream(0, pmsf::kPaperBlobStreamPCATMStrictPaperGeometry),
            decodedRounds_);
    } else if (globalByteRangeResidual) {
        globalResiduals =
            decodeResidualGlobalVarintByteRange(
                rd.getStream(0, pmsf::kPaperBlobStreamPCATMStrictPaperGeometry),
                decodedRounds_);
    } else if (globalDenseRangeResidual) {
        globalResiduals = decodeResidualGlobalDenseRange(
            rd.getStream(0, pmsf::kPaperBlobStreamPCATMStrictPaperGeometry),
            decodedRounds_);
    } else if (globalMeppAxisRangeResidual) {
        globalResiduals = decodeResidualGlobalMeppAxisRange(
            rd.getStream(0, pmsf::kPaperBlobStreamPCATMStrictPaperGeometry),
            decodedRounds_);
    } else if (globalAxisVarintResidual) {
        globalResiduals = decodeResidualGlobalAxisVarintZlib(
            rd.getStream(0, pmsf::kPaperBlobStreamPCATMStrictPaperGeometry),
            decodedRounds_);
    } else if (globalSplitAxisVarintResidual) {
        globalResiduals =
            decodeResidualGlobalSplitAxisVarintZlib(
                rd.getStream(0, pmsf::kPaperBlobStreamPCATMStrictPaperGeometry),
                decodedRounds_);
    }

    // WS-B task-18: the side-info 2-color mode needs per-round color streams
    // (1 bit/face on mode-2 rounds). The encoder writes them under
    // PMSF_PCATM_STRICT_2COLOR_SIDE; both sides must agree on the layout.
    const bool globalColor =
        pmsf::paperBlobSchemeVersionPCATMStrictPaperUsesGlobalColor(
            hdr.scheme_version) &&
        !isEnvOn("PMSF_PCATM_STRICT_2COLOR_SIDE");
    std::vector<std::vector<uint8_t>> globalColorRounds;
    std::vector<int> globalColorModes;
    if (globalColor) {
        globalColorRounds = decodeColorRoundsGlobal(
            rd.getStream(0, pmsf::kPaperBlobStreamPCATMStrictPaperColor),
            decodedRounds_,
            globalColorModes);
    }

    for (int r = 0; r < decodedRounds_; ++r) {
        int colorMode = 2;
        std::vector<uint8_t> colors;
        if (globalColor) {
            colorMode = globalColorModes[static_cast<size_t>(r)];
            colors = globalColorRounds[static_cast<size_t>(r)];
        } else {
            colors = (hasTopoSideStreams || colorOnly)
                         ? decodeColorLzV3(
                               rd.getStream(
                                   r, pmsf::kPaperBlobStreamPCATMStrictPaperColor),
                               colorMode)
                         : decodeColorLz(
                               rd.getStream(
                                   r, pmsf::kPaperBlobStreamPCATMStrictPaperColor));
        }
        if (!globalColor && isEnvOn("PMSF_PCATM_STRICT_2COLOR_SIDE") &&
            colorMode == 2) {
            mode2SidePerRound_[static_cast<size_t>(r)] = decodeMode2SideInfo(
                rd.getStream(r, pmsf::kPaperBlobStreamPCATMStrictPaperColor));
        }
        colorModePerRound_[static_cast<size_t>(r)] = colorMode;
        colorSymbolsPerRound_[static_cast<size_t>(r)] = colors;
        totalColorBits_ += static_cast<int>(colors.size());
        for (uint8_t c : colors) {
            rcDecodedColors_.push_back(c);
        }
        if (static_cast<int>(faceBitsPerRound_.size()) > r) {
            faceBitsPerRound_[r] = static_cast<int>(colors.size());
        }

        std::vector<MCGAL::PointInt> residuals =
            (globalAdaptiveResidual || globalHuffman || globalVarintResidual ||
             globalDenseRangeResidual || globalAxisVarintResidual ||
             globalSplitAxisVarintResidual || globalByteRangeResidual ||
             globalMeppAxisRangeResidual)
                ? globalResiduals[static_cast<size_t>(r)]
                : decodeResidualHuffman(
                      rd.getStream(
                          r, pmsf::kPaperBlobStreamPCATMStrictPaperGeometry));
        residualsPerRound_[static_cast<size_t>(r)] =
            static_cast<int>(residuals.size());
        totalResiduals_ += static_cast<int>(residuals.size());
        for (const auto& residual : residuals) {
            rcDecodedResiduals_.push_back(residual);
        }

        if (hasTopoSideStreams) {
            decodeValenceRaw(
                rd.getStream(r, pmsf::kPaperBlobStreamPCATMStrictPaperTopologyValence),
                rcDecodedValCodes_,
                rcDecodedOtherVals_);
            decodeBitsLzRaw(
                rd.getStream(r, pmsf::kPaperBlobStreamPCATMStrictPaperTopologyIsPoly),
                rcDecodedIsPoly_);
        }
    }

    const size_t blobSize = hdr.headerByteSize() +
                            (hdr.stream_offsets.empty() ? 0 : hdr.stream_offsets.back());
    dataOffset += blobSize;

    if (const char* env = std::getenv("PMSF_BPV_DIAG"); env && env[0] == '1') {
        std::cerr << "[PCATMStrictPaper decoder sv=" << hdr.scheme_version
                  << "] rounds=" << decodedRounds_
                  << " totalColorBits=" << totalColorBits_
                  << " totalResiduals=" << totalResiduals_
                  << " blobSize=" << blobSize
                  << " residualMode=" << (globalAdaptiveResidual
                                             ? "adaptive_bitplane"
                                         : (globalByteRangeResidual
                                             ? "global_varint_byte_range"
                                         : (globalMeppAxisRangeResidual
                                             ? "global_mepp_axis_range"
                                         : (globalSplitAxisVarintResidual
                                             ? "global_split_axis_varint_zlib"
                                         : (globalAxisVarintResidual ? "global_axis_varint_zlib"
                                         : (globalDenseRangeResidual ? "global_dense_range"
                                         : (globalVarintResidual ? "global_varint_zlib"
                                                                : (globalHuffman ? "global_huffman"
                                                                                 : "per_round"))))))))
                  << " topoSideStreams=" << (hasTopoSideStreams ? "yes" : "no")
                  << " colorOnly=" << (colorOnly ? "yes" : "no")
                  << std::endl;
    }
}

namespace {

void pushFanDiagonals(MCGAL::Facet* f,
                      int valence,
                      std::vector<MCGAL::Halfedge*>& halfedge) {
    if (!f || valence < 4) return;
    MCGAL::Halfedge* proxyH = f->proxyHalfedge();
    if (!proxyH || !proxyH->next() || !proxyH->next()->next()) return;
    MCGAL::Halfedge* hd = proxyH->next()->next();
    halfedge.push_back(hd);
    MCGAL::Halfedge* hdCur = hd;
    for (int i = 1; i <= valence - 4; ++i) {
        if (!hdCur || !hdCur->opposite()) break;
        MCGAL::Halfedge* hdOp = hdCur->opposite();
        MCGAL::Halfedge* h0Fan = hdOp->next();
        if (!h0Fan || !h0Fan->next()) break;
        MCGAL::Halfedge* hdNext = h0Fan->next();
        halfedge.push_back(hdNext);
        hdCur = hdNext;
    }
}

uint8_t colorOf(MCGAL::Facet* f,
                const std::unordered_map<MCGAL::Facet*, uint8_t>& colorByFace) {
    auto it = colorByFace.find(f);
    return it == colorByFace.end() ? 0u : it->second;
}

bool isColorOnlyChild(
    MCGAL::Facet* f,
    const std::unordered_map<MCGAL::Facet*, uint8_t>& colorByFace,
    const std::unordered_set<MCGAL::Facet*>& consumed) {
    if (!f || f->isRemoved() || consumed.count(f)) return false;
    const uint8_t c = colorOf(f, colorByFace);
    return c == 2u || c == 3u;
}

bool extendFanChainByColor(
    MCGAL::Facet* cur,
    MCGAL::Facet* prev,
    const std::unordered_map<MCGAL::Facet*, uint8_t>& colorByFace,
    const std::unordered_set<MCGAL::Facet*>& consumed,
    std::vector<MCGAL::Halfedge*>& diagonals,
    std::vector<MCGAL::Facet*>& childFaces,
    int depth) {
    if (!cur || depth > 64) return false;
    if (colorOf(cur, colorByFace) == 3u) return true;

    MCGAL::Halfedge* start = cur->proxyHalfedge();
    if (!start) return false;

    std::vector<MCGAL::Halfedge*> candidates;
    MCGAL::Halfedge* h = start;
    do {
        MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
        MCGAL::Facet* nextFace = opp ? opp->face() : nullptr;
        const bool alreadyInPath =
            std::find(childFaces.begin(), childFaces.end(), nextFace) != childFaces.end();
        if (nextFace != prev &&
            !alreadyInPath &&
            isColorOnlyChild(nextFace, colorByFace, consumed)) {
            candidates.push_back(h);
        }
        h = h ? h->next() : nullptr;
    } while (h && h != start);

    std::sort(candidates.begin(), candidates.end(),
              [](MCGAL::Halfedge* a, MCGAL::Halfedge* b) {
                  return a->poolId() < b->poolId();
              });

    for (MCGAL::Halfedge* cand : candidates) {
        MCGAL::Facet* nextFace = cand->opposite()->face();
        diagonals.push_back(cand);
        childFaces.push_back(nextFace);
        if (extendFanChainByColor(nextFace, cur, colorByFace, consumed,
                                  diagonals, childFaces, depth + 1)) {
            return true;
        }
        childFaces.pop_back();
        diagonals.pop_back();
    }
    return false;
}


std::vector<MCGAL::Vertex*> faceVertices(MCGAL::Facet* f) {
    if (!f) return {};
    return f->getVertices();
}

bool faceHasVertex(MCGAL::Facet* f, MCGAL::Vertex* v) {
    if (!f || !v) return false;
    for (MCGAL::Vertex* fv : f->getVertices()) {
        if (fv == v) return true;
    }
    return false;
}

std::vector<MCGAL::Facet*> adjacentFacesWithColor(
    const std::vector<MCGAL::Facet*>& faces,
    const std::unordered_map<MCGAL::Facet*, uint8_t>& colorByFace,
    uint8_t color,
    MCGAL::Vertex* requiredVertex) {
    std::vector<MCGAL::Facet*> out;
    std::unordered_set<MCGAL::Facet*> seen;
    for (MCGAL::Facet* f : faces) {
        MCGAL::Halfedge* start = f ? f->proxyHalfedge() : nullptr;
        if (!start) continue;
        MCGAL::Halfedge* h = start;
        do {
            MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
            MCGAL::Facet* nf = opp ? opp->face() : nullptr;
            auto it = colorByFace.find(nf);
            if (it != colorByFace.end() && it->second == color &&
                (!requiredVertex || faceHasVertex(nf, requiredVertex)) &&
                seen.insert(nf).second) {
                out.push_back(nf);
            }
            h = h ? h->next() : nullptr;
        } while (h && h != start);
    }
    return out;
}

bool patchFacesFormZPath(const std::unordered_set<MCGAL::Facet*>& faceSet,
                         MCGAL::Facet* zeroA,
                         MCGAL::Facet* zeroB) {
    if (faceSet.size() < 3 || !zeroA || !zeroB || zeroA == zeroB ||
        !faceSet.count(zeroA) || !faceSet.count(zeroB)) {
        return false;
    }
    std::unordered_set<MCGAL::Facet*> visited;
    std::unordered_map<MCGAL::Facet*, int> degree;
    std::queue<MCGAL::Facet*> q;
    visited.insert(*faceSet.begin());
    q.push(*faceSet.begin());
    int edgeCountTwice = 0;
    while (!q.empty()) {
        MCGAL::Facet* cur = q.front();
        q.pop();
        MCGAL::Halfedge* start = cur ? cur->proxyHalfedge() : nullptr;
        if (!start) continue;
        MCGAL::Halfedge* h = start;
        do {
            MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
            MCGAL::Facet* nf = opp ? opp->face() : nullptr;
            if (faceSet.count(nf)) {
                ++degree[cur];
                ++edgeCountTwice;
                if (!visited.count(nf)) {
                    visited.insert(nf);
                    q.push(nf);
                }
            }
            h = h ? h->next() : nullptr;
        } while (h && h != start);
    }
    if (visited.size() != faceSet.size() ||
        edgeCountTwice / 2 != static_cast<int>(faceSet.size()) - 1) {
        return false;
    }
    int leaves = 0;
    for (MCGAL::Facet* f : faceSet) {
        const int d = degree[f];
        if (d == 1) ++leaves;
        else if (d != 2) return false;
    }
    return leaves == 2 && degree[zeroA] == 1 && degree[zeroB] == 1;
}

bool patchFacesFormTree(const std::unordered_set<MCGAL::Facet*>& faceSet) {
    if (faceSet.empty()) return false;
    std::unordered_set<MCGAL::Facet*> visited;
    std::queue<MCGAL::Facet*> q;
    visited.insert(*faceSet.begin());
    q.push(*faceSet.begin());
    int edgeCountTwice = 0;
    while (!q.empty()) {
        MCGAL::Facet* cur = q.front();
        q.pop();
        MCGAL::Halfedge* start = cur ? cur->proxyHalfedge() : nullptr;
        if (!start) continue;
        MCGAL::Halfedge* h = start;
        do {
            MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
            MCGAL::Facet* nf = opp ? opp->face() : nullptr;
            if (faceSet.count(nf)) {
                ++edgeCountTwice;
                if (!visited.count(nf)) {
                    visited.insert(nf);
                    q.push(nf);
                }
            }
            h = h ? h->next() : nullptr;
        } while (h && h != start);
    }
    return visited.size() == faceSet.size() &&
           edgeCountTwice / 2 == static_cast<int>(faceSet.size()) - 1;
}

bool facesShareEdge(MCGAL::Facet* a, MCGAL::Facet* b) {
    if (!a || !b) return false;
    MCGAL::Halfedge* start = a->proxyHalfedge();
    if (!start) return false;
    MCGAL::Halfedge* h = start;
    do {
        MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
        if (opp && opp->face() == b) return true;
        h = h ? h->next() : nullptr;
    } while (h && h != start);
    return false;
}

MCGAL::PointInt quantizePoint(const MCGAL::Point& p,
                              const MCGAL::Point& bboxMin,
                              float quantStep) {
    return MCGAL::PointInt((p.x() - bboxMin.x()) / quantStep,
                           (p.y() - bboxMin.y()) / quantStep,
                           (p.z() - bboxMin.z()) / quantStep);
}

std::vector<MCGAL::Facet*> stableQuantizedFaceOrder(
    const std::shared_ptr<MCGAL::Mesh>& mesh,
    const MCGAL::Point& bboxMin,
    float quantStep) {
    struct FaceOrderRecord {
        MCGAL::Facet* face = nullptr;
        std::vector<MCGAL::PointInt> key;
    };
    auto makeKey = [&](MCGAL::Facet* f) {
        std::vector<MCGAL::PointInt> key;
        if (!f) return key;
        MCGAL::Halfedge* start = f->proxyHalfedge();
        if (!start) return key;
        key.reserve(static_cast<size_t>(std::max(f->facet_degree(), 0)));
        MCGAL::Halfedge* h = start;
        do {
            if (MCGAL::Vertex* v = h ? h->vertex() : nullptr) {
                key.push_back(quantizePoint(v->point(), bboxMin, quantStep));
            }
            h = h ? h->next() : nullptr;
        } while (h && h != start);
        std::sort(key.begin(), key.end());
        return key;
    };

    std::vector<FaceOrderRecord> records;
    if (!mesh) return {};
    records.reserve(mesh->size_of_facets());
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        records.push_back(FaceOrderRecord{f, makeKey(f)});
    }
    std::sort(records.begin(), records.end(),
              [](const FaceOrderRecord& a, const FaceOrderRecord& b) {
                  if (a.key.size() != b.key.size()) return a.key.size() < b.key.size();
                  return std::lexicographical_compare(
                      a.key.begin(), a.key.end(), b.key.begin(), b.key.end());
              });
    std::vector<MCGAL::Facet*> faces;
    faces.reserve(records.size());
    for (const FaceOrderRecord& record : records) faces.push_back(record.face);
    return faces;
}

uint64_t morton3D12(uint32_t x, uint32_t y, uint32_t z) {
    uint64_t code = 0;
    for (uint32_t i = 0; i < 12; ++i) {
        code |= ((uint64_t(x) >> i) & 1ull) << (3 * i);
        code |= ((uint64_t(y) >> i) & 1ull) << (3 * i + 1);
        code |= ((uint64_t(z) >> i) & 1ull) << (3 * i + 2);
    }
    return code;
}

std::vector<MCGAL::Facet*> mortonFaceOrder(
    const std::shared_ptr<MCGAL::Mesh>& mesh,
    const MCGAL::Point& bboxMin,
    float quantStep) {
    struct FaceOrderRecord {
        MCGAL::Facet* face = nullptr;
        uint64_t morton = 0;
        std::vector<MCGAL::PointInt> tie;
    };
    auto makeRecord = [&](MCGAL::Facet* f) {
        int64_t sx = 0, sy = 0, sz = 0;
        int n = 0;
        std::vector<MCGAL::PointInt> tie;
        if (f) tie.reserve(static_cast<size_t>(std::max(f->facet_degree(), 0)));
        if (f) {
            MCGAL::Halfedge* start = f->proxyHalfedge();
            MCGAL::Halfedge* h = start;
            while (h) {
                if (MCGAL::Vertex* v = h->vertex()) {
                    MCGAL::PointInt q = quantizePoint(v->point(), bboxMin, quantStep);
                    sx += q.x();
                    sy += q.y();
                    sz += q.z();
                    tie.push_back(q);
                    ++n;
                }
                h = h->next();
                if (h == start) break;
            }
        }
        std::sort(tie.begin(), tie.end());
        auto clamp12 = [](int64_t v) -> uint32_t {
            if (v < 0) return 0u;
            if (v > 4095) return 4095u;
            return static_cast<uint32_t>(v);
        };
        const uint64_t morton = (n <= 0)
            ? uint64_t(0)
            : morton3D12(clamp12(sx / n), clamp12(sy / n), clamp12(sz / n));
        return FaceOrderRecord{f, morton, std::move(tie)};
    };
    std::vector<FaceOrderRecord> records;
    if (!mesh) return {};
    records.reserve(mesh->size_of_facets());
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        records.push_back(makeRecord(f));
    }
    std::sort(records.begin(), records.end(),
              [](const FaceOrderRecord& a, const FaceOrderRecord& b) {
                  if (a.morton != b.morton) return a.morton < b.morton;
                  return std::lexicographical_compare(
                      a.tie.begin(), a.tie.end(), b.tie.begin(), b.tie.end());
              });
    std::vector<MCGAL::Facet*> faces;
    faces.reserve(records.size());
    for (const FaceOrderRecord& record : records) faces.push_back(record.face);
    return faces;
}

bool zerosAttachToBlueRunEndpoints(const std::vector<MCGAL::Facet*>& ones,
                                   MCGAL::Facet* zeroA,
                                   MCGAL::Facet* zeroB) {
    if (ones.empty() || !zeroA || !zeroB || zeroA == zeroB) return false;
    std::unordered_set<MCGAL::Facet*> oneSet(ones.begin(), ones.end());
    std::vector<MCGAL::Facet*> endpoints;
    for (MCGAL::Facet* f : ones) {
        int deg = 0;
        MCGAL::Halfedge* start = f ? f->proxyHalfedge() : nullptr;
        if (!start) continue;
        MCGAL::Halfedge* h = start;
        do {
            MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
            MCGAL::Facet* nf = opp ? opp->face() : nullptr;
            if (oneSet.count(nf)) ++deg;
            h = h ? h->next() : nullptr;
        } while (h && h != start);
        if (deg <= 1) endpoints.push_back(f);
    }
    if (ones.size() == 1) {
        return facesShareEdge(zeroA, ones.front()) &&
               facesShareEdge(zeroB, ones.front());
    }
    if (endpoints.size() != 2) return false;
    return (facesShareEdge(zeroA, endpoints[0]) &&
            facesShareEdge(zeroB, endpoints[1])) ||
           (facesShareEdge(zeroA, endpoints[1]) &&
            facesShareEdge(zeroB, endpoints[0]));
}

std::vector<MCGAL::Halfedge*> collectPatchTreeDiagonals(
    MCGAL::Facet* root,
    const std::unordered_set<MCGAL::Facet*>& patchFaces) {
    std::vector<MCGAL::Halfedge*> diagonals;
    if (!root) return diagonals;
    std::unordered_set<MCGAL::Facet*> visited;
    std::queue<MCGAL::Facet*> q;
    visited.insert(root);
    q.push(root);
    while (!q.empty()) {
        MCGAL::Facet* cur = q.front();
        q.pop();
        MCGAL::Halfedge* start = cur ? cur->proxyHalfedge() : nullptr;
        if (!start) continue;
        MCGAL::Halfedge* h = start;
        do {
            MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
            MCGAL::Facet* nf = opp ? opp->face() : nullptr;
            if (nf && patchFaces.count(nf) && !visited.count(nf)) {
                visited.insert(nf);
                q.push(nf);
                diagonals.push_back(h);
            }
            h = h ? h->next() : nullptr;
        } while (h && h != start);
    }
    if (visited.size() != patchFaces.size()) diagonals.clear();
    return diagonals;
}

}  // namespace

void PCATMStrictPMSFSymbolReadOperator::collectFromDecoded(
    MCGAL::Halfedge* seed,
    std::vector<MCGAL::Vertex*>& vertices,
    std::vector<MCGAL::Halfedge*>& halfedge,
    std::vector<MCGAL::Facet*>& facets) {
    (void)vertices;
    const int roundIdx = currentRoundIdx_;
    const bool colorOnlyRound =
        roundIdx >= 0 &&
        roundIdx < static_cast<int>(colorSymbolsPerRound_.size()) &&
        !colorSymbolsPerRound_[static_cast<size_t>(roundIdx)].empty() &&
        rcDecodedIsPoly_.empty();

    if (colorOnlyRound) {
        const auto& roundColors = colorSymbolsPerRound_[static_cast<size_t>(roundIdx)];
        const int colorMode = colorModePerRound_[static_cast<size_t>(roundIdx)];
        if (colorMode != 2 && colorMode != 4 && colorMode != 3) {
            throw std::runtime_error(
                "[PCATMStrictPaper decoder] unsupported color-only mode=" +
                std::to_string(colorMode));
        }

        std::vector<MCGAL::Facet*> facesInOrder;
        facesInOrder.reserve(roundColors.size());
        std::unordered_map<MCGAL::Facet*, uint8_t> colorByFace;
        colorByFace.reserve(roundColors.size() * 2 + 16);
        size_t colorCursor = 0;

        if (mesh_ && isEnvOn("PMSF_PCATM_STRICT_MORTON_FACE_ORDER")) {
            for (MCGAL::Facet* f :
                 mortonFaceOrder(mesh_, bboxMin_, f_quantStep_)) {
                f->normalize_proxy_to_min_position_vertex(bboxMin_, f_quantStep_);
                uint8_t c = colorCursor < roundColors.size() ? roundColors[colorCursor++] : 0u;
                facesInOrder.push_back(f);
                colorByFace[f] = c;
                f->setPCATMPatchTag(c);
            }
        } else if (mesh_ && (isEnvOn("PMSF_PCATM_STRICT_FACE_ORDER") ||
                      isEnvOn("PMSF_PCATM_STRICT_STABLE_FACE_ORDER"))) {
            for (MCGAL::Facet* f :
                 stableQuantizedFaceOrder(mesh_, bboxMin_, f_quantStep_)) {
                f->normalize_proxy_to_min_position_vertex(bboxMin_, f_quantStep_);
                uint8_t c = colorCursor < roundColors.size() ? roundColors[colorCursor++] : 0u;
                facesInOrder.push_back(f);
                colorByFace[f] = c;
                f->setPCATMPatchTag(c);
            }
        } else {
            std::queue<int> q;
            if (seed) q.push(seed->poolId());
            int currentVersion = MCGAL::BFSVersionManager::current_version++;
            while (!q.empty()) {
                int hid = q.front();
                q.pop();
                MCGAL::Halfedge* h =
                    MCGAL::contextPool.getHalfedgeByIndexInSubPool(DEFAULT_MESH_ID, hid);
                if (!h || h->isVisited(currentVersion) || h->isRemoved()) continue;
                h->setVisited(currentVersion);

                MCGAL::Facet* f = h->face();
                if (f && !f->isVisited(currentVersion)) {
                    f->setVisited(currentVersion);
                    f->normalize_proxy_to_min_position_vertex(bboxMin_, f_quantStep_);
                    uint8_t c = colorCursor < roundColors.size() ? roundColors[colorCursor++] : 0u;
                    facesInOrder.push_back(f);
                    colorByFace[f] = c;
                    f->setPCATMPatchTag(c);
                }

                MCGAL::Halfedge* hIt = h;
                do {
                    MCGAL::Halfedge* hOpp = hIt->opposite();
                    if (hOpp && !hOpp->isVisited(currentVersion)) q.push(hOpp->poolId());
                    if (hIt && !hIt->isVisited(currentVersion)) q.push(hIt->poolId());
                    hIt = hIt->next();
                } while (hIt != h && hIt != nullptr);
            }
        }

        if (colorCursor != roundColors.size() ||
            facesInOrder.size() != roundColors.size()) {
            throw std::runtime_error(
                "[PCATMStrictPaper decoder] color/face order mismatch round=" +
                std::to_string(roundIdx) + " colors=" +
                std::to_string(roundColors.size()) + " faces=" +
                std::to_string(facesInOrder.size()) + " consumed=" +
                std::to_string(colorCursor));
        }

        if (colorMode == 3) {
            // mode-3: single 2-bit stream. color 1 = Z core (geometric zeros),
            // color 2 = deg3/4 patch (same-color grouping). Residual order:
            // all Z patches first (facesInOrder blue order), then all deg3/4.
            std::unordered_set<MCGAL::Facet*> consumed;
            consumed.reserve(facesInOrder.size() + 16);
            int recoveredPatches = 0;
            int recoveredDiagonals = 0;

            // ---- Pass 1: Z (blue) patches ----
            for (MCGAL::Facet* f : facesInOrder) {
                auto it = colorByFace.find(f);
                if (it == colorByFace.end() || it->second != 1u || consumed.count(f))
                    continue;
                std::vector<MCGAL::Facet*> ones;
                std::queue<MCGAL::Facet*> cq;
                consumed.insert(f);
                cq.push(f);
                while (!cq.empty()) {
                    MCGAL::Facet* cur = cq.front();
                    cq.pop();
                    ones.push_back(cur);
                    MCGAL::Halfedge* start = cur ? cur->proxyHalfedge() : nullptr;
                    if (!start) continue;
                    MCGAL::Halfedge* h = start;
                    do {
                        MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
                        MCGAL::Facet* nf = opp ? opp->face() : nullptr;
                        auto nit = colorByFace.find(nf);
                        if (nit != colorByFace.end() && nit->second == 1u && !consumed.count(nf)) {
                            consumed.insert(nf);
                            cq.push(nf);
                        }
                        h = h ? h->next() : nullptr;
                    } while (h && h != start);
                }
                std::array<MCGAL::Facet*, 2> recoveredZeros =
                    pmsf::pcatm::paperMode2Zeros(ones, colorByFace, bboxMin_, f_quantStep_);
                if (recoveredZeros[0] == nullptr || recoveredZeros[1] == nullptr ||
                    consumed.count(recoveredZeros[0]) || consumed.count(recoveredZeros[1])) {
                    throw std::runtime_error(
                        "[PCATMStrictPaper decoder] mode=3 ambiguous/overlapping Z patch round=" +
                        std::to_string(roundIdx));
                }
                std::unordered_set<MCGAL::Facet*> patchFaceSet(ones.begin(), ones.end());
                patchFaceSet.insert(recoveredZeros[0]);
                patchFaceSet.insert(recoveredZeros[1]);
                if (patchFaceSet.size() != ones.size() + 2) {
                    throw std::runtime_error(
                        "[PCATMStrictPaper decoder] mode=3 Z patch face-set mismatch round=" +
                        std::to_string(roundIdx));
                }
                MCGAL::Facet* root = recoveredZeros[0];
                std::vector<MCGAL::Halfedge*> diagonals =
                    collectPatchTreeDiagonals(root, patchFaceSet);
                if (diagonals.size() + 1 != patchFaceSet.size()) {
                    throw std::runtime_error(
                        "[PCATMStrictPaper decoder] mode=3 Z patch tree mismatch round=" +
                        std::to_string(roundIdx));
                }
                if (rcDecodedResiduals_.empty()) {
                    throw std::runtime_error(
                        "[PCATMStrictPaper decoder] residual underflow in mode=3 Z recovery");
                }
                const int valence = static_cast<int>(patchFaceSet.size()) + 2;
                MCGAL::PointInt residual = rcDecodedResiduals_.front();
                rcDecodedResiduals_.pop_front();
                root->setSplittable();
                root->setPolyValence(valence);
                root->setRemovedVertexPosInt(residual);
                pmsf::markPaperStrictPatchProtocol(root);
                root->normalize_proxy_to_min_position_vertex(bboxMin_, f_quantStep_);
                facets.push_back(root);
                for (MCGAL::Halfedge* hd : diagonals) {
                    if (!hd || hd->isRemoved()) continue;
                    hd->setAdded();
                    halfedge.push_back(hd);
                    ++recoveredDiagonals;
                }
                consumed.insert(recoveredZeros[0]);
                consumed.insert(recoveredZeros[1]);
                ++recoveredPatches;
            }

            // ---- Pass 2: deg3/4 patches (color 2), same-color grouping ----
            for (MCGAL::Facet* f : facesInOrder) {
                auto it = colorByFace.find(f);
                if (it == colorByFace.end() || it->second != 2u || consumed.count(f))
                    continue;
                std::vector<MCGAL::Facet*> componentFaces;
                std::vector<MCGAL::Halfedge*> diagonals;
                std::queue<MCGAL::Facet*> cq;
                cq.push(f);
                consumed.insert(f);
                while (!cq.empty()) {
                    MCGAL::Facet* cur = cq.front();
                    cq.pop();
                    componentFaces.push_back(cur);
                    MCGAL::Halfedge* start = cur ? cur->proxyHalfedge() : nullptr;
                    if (!start) continue;
                    MCGAL::Halfedge* h = start;
                    do {
                        MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
                        MCGAL::Facet* nf = opp ? opp->face() : nullptr;
                        auto nit = colorByFace.find(nf);
                        if (nit != colorByFace.end() && nit->second == 2u && !consumed.count(nf)) {
                            consumed.insert(nf);
                            cq.push(nf);
                            diagonals.push_back(h);
                        }
                        h = h ? h->next() : nullptr;
                    } while (h && h != start);
                }
                if (rcDecodedResiduals_.empty()) {
                    throw std::runtime_error(
                        "[PCATMStrictPaper decoder] residual underflow in mode=3 deg3/4 recovery");
                }
                const int valence = static_cast<int>(componentFaces.size()) + 2;
                MCGAL::PointInt residual = rcDecodedResiduals_.front();
                rcDecodedResiduals_.pop_front();
                f->setSplittable();
                f->setPolyValence(valence);
                f->setRemovedVertexPosInt(residual);
                pmsf::markPaperStrictPatchProtocol(f);
                f->normalize_proxy_to_min_position_vertex(bboxMin_, f_quantStep_);
                facets.push_back(f);
                for (MCGAL::Halfedge* hd : diagonals) {
                    if (!hd || hd->isRemoved()) continue;
                    hd->setAdded();
                    halfedge.push_back(hd);
                    ++recoveredDiagonals;
                }
                ++recoveredPatches;
            }

            const int expectedPatches =
                (roundIdx >= 0 && roundIdx < static_cast<int>(residualsPerRound_.size()))
                    ? residualsPerRound_[static_cast<size_t>(roundIdx)]
                    : -1;
            if (const char* env = std::getenv("PMSF_BPV_DIAG"); env && env[0] == '1') {
                std::cerr << "[PCATMStrictPaper 3-color round=" << roundIdx
                          << "] faces=" << facesInOrder.size()
                          << " patches=" << recoveredPatches
                          << " expectedPatches=" << expectedPatches
                          << " diagonals=" << recoveredDiagonals << std::endl;
            }
            if (expectedPatches >= 0 && recoveredPatches != expectedPatches) {
                throw std::runtime_error(
                    "[PCATMStrictPaper decoder] mode=3 patch count mismatch round=" +
                    std::to_string(roundIdx) + " recovered=" +
                    std::to_string(recoveredPatches) + " expected=" +
                    std::to_string(expectedPatches));
            }
            currentRoundIdx_++;
            return;
        }

        if (colorMode == 2) {
            std::unordered_set<MCGAL::Facet*> consumed;
            // WS-B task-18: explicit side-info disambiguation. The per-round
            // color stream carries a "C2S2" section with one record per blue
            // component: merged flag, optional (P, S, membership) split info,
            // and one 4-bit candidate zero-pair index per patch.
            const std::vector<uint8_t>* sideInfo = nullptr;
            if (isEnvOn("PMSF_PCATM_STRICT_2COLOR_SIDE") && roundIdx >= 0 &&
                roundIdx < static_cast<int>(mode2SidePerRound_.size()) &&
                !mode2SidePerRound_[static_cast<size_t>(roundIdx)].empty()) {
                sideInfo = &mode2SidePerRound_[static_cast<size_t>(roundIdx)];
            }
            int oneFaces = 0;
            for (const auto& kv : colorByFace) {
                if (kv.second == 1u) ++oneFaces;
            }
            consumed.reserve(static_cast<size_t>(oneFaces) * 2 + 16);
            int recoveredPatches = 0;
            int recoveredDiagonals = 0;
            int recoveredPatchFaces = 0;
            if (isEnvOn("PMSF_PCATM_MODE2_DIAG")) {
                std::cerr << "[2c-faces-dec] round=" << roundIdx << " n="
                          << facesInOrder.size() << " head=";
                const int lim =
                    std::min<int>(8, static_cast<int>(facesInOrder.size()));
                for (int fi = 0; fi < lim; ++fi) {
                    std::array<std::int64_t, 3> k{};
                    pmsf::pcatm::mode2FaceCentroidQ(
                        facesInOrder[static_cast<size_t>(fi)], mesh_, k);
                    std::cerr << "(" << k[0] << "," << k[1] << "," << k[2]
                              << ")";
                }
                std::cerr << std::endl;
            }
            std::unordered_map<MCGAL::Facet*, int> sideOrderPos;
            sideOrderPos.reserve(facesInOrder.size() * 2);
            for (int i = 0; i < static_cast<int>(facesInOrder.size()); ++i)
                sideOrderPos[facesInOrder[static_cast<size_t>(i)]] = i;
            auto floodFillBlue = [&](MCGAL::Facet* seed) {
                std::vector<MCGAL::Facet*> out;
                std::queue<MCGAL::Facet*> cq;
                consumed.insert(seed);
                cq.push(seed);
                while (!cq.empty()) {
                    MCGAL::Facet* cur = cq.front();
                    cq.pop();
                    out.push_back(cur);
                    MCGAL::Halfedge* start = cur ? cur->proxyHalfedge() : nullptr;
                    if (!start) continue;
                    MCGAL::Halfedge* h = start;
                    do {
                        MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
                        MCGAL::Facet* nf = opp ? opp->face() : nullptr;
                        auto nit = colorByFace.find(nf);
                        if (nit != colorByFace.end() && nit->second == 1u &&
                            !consumed.count(nf)) {
                            consumed.insert(nf);
                            cq.push(nf);
                        }
                        h = h ? h->next() : nullptr;
                    } while (h && h != start);
                }
                return out;
            };
            auto recoverPatch = [&](
                const std::vector<MCGAL::Facet*>& ones,
                const std::array<MCGAL::Facet*, 2>& sideZeros) {
                std::array<MCGAL::Facet*, 2> recoveredZeros{nullptr, nullptr};
                if (sideZeros[0] != nullptr && sideZeros[1] != nullptr) {
                    // v7: the side stream carries the two zero face indices
                    // directly; the tree check below verifies ones+zeros is
                    // exactly the removed-vertex patch (n faces, n-1 diagonals,
                    // connected), so a wrong/desynced index fails closed.
                    recoveredZeros = {{sideZeros[0], sideZeros[1]}};
                } else if (orientedZRecovery_ &&
                           !isEnvOn("PMSF_PCATM_STRICT_2COLOR_LEGACY_MATCH")) {
                    recoveredZeros = pmsf::pcatm::paperMode2Zeros(
                        ones, colorByFace, bboxMin_, f_quantStep_);
                } else {
                    // Legacy sv4..11 used pure dual-path uniqueness, not
                    // oriented Z / minimum-angle recovery. Keep its semantics.
                    const auto zeros =
                        adjacentFacesWithColor(ones, colorByFace, 0u, nullptr);
                    int matches = 0;
                    for (size_t a = 0; a < zeros.size(); ++a) {
                        for (size_t b = a + 1; b < zeros.size(); ++b) {
                            std::unordered_set<MCGAL::Facet*> trial(
                                ones.begin(), ones.end());
                            trial.insert(zeros[a]);
                            trial.insert(zeros[b]);
                            if (trial.size() != ones.size() + 2 ||
                                !patchFacesFormZPath(trial, zeros[a], zeros[b]))
                                continue;
                            ++matches;
                            recoveredZeros = {zeros[a], zeros[b]};
                        }
                    }
                    if (matches != 1) recoveredZeros = {nullptr, nullptr};
                }
                if (recoveredZeros[0] == nullptr ||
                    recoveredZeros[1] == nullptr ||
                    consumed.count(recoveredZeros[0]) ||
                    consumed.count(recoveredZeros[1])) {
                    throw std::runtime_error(
                        "[PCATMStrictPaper decoder] mode=2 ambiguous/overlapping patch round=" +
                        std::to_string(roundIdx));
                }

                std::unordered_set<MCGAL::Facet*> patchFaceSet;
                std::vector<MCGAL::Halfedge*> diagonals;
                MCGAL::Facet* root = nullptr;
                std::vector<MCGAL::Facet*> selectedZeros;
                for (MCGAL::Facet* pf : ones) {
                    patchFaceSet.insert(pf);
                }
                patchFaceSet.insert(recoveredZeros[0]);
                patchFaceSet.insert(recoveredZeros[1]);
                if (patchFaceSet.size() != ones.size() + 2) {
                    root = nullptr;
                    patchFaceSet.clear();
                    diagonals.clear();
                    selectedZeros.clear();
                } else {
                    root = recoveredZeros[0];
                    selectedZeros = {
                        recoveredZeros[0], recoveredZeros[1]};
                    diagonals = collectPatchTreeDiagonals(root, patchFaceSet);
                    if (diagonals.size() + 1 != patchFaceSet.size()) {
                        root = nullptr;
                        patchFaceSet.clear();
                        diagonals.clear();
                        selectedZeros.clear();
                    }
                }
                if (!root) {
                    throw std::runtime_error(
                        "[PCATMStrictPaper decoder] mode=2 patch tree mismatch round=" +
                        std::to_string(roundIdx) + " zeroCandidates=" +
                        std::to_string(2));
                }
                if (rcDecodedResiduals_.empty()) {
                    throw std::runtime_error(
                        "[PCATMStrictPaper decoder] residual underflow in mode=2 recovery");
                }

                const int valence = static_cast<int>(patchFaceSet.size()) + 2;
                MCGAL::PointInt residual = rcDecodedResiduals_.front();
                rcDecodedResiduals_.pop_front();
                root->setSplittable();
                root->setPolyValence(valence);
                root->setRemovedVertexPosInt(residual);
                pmsf::markPaperStrictPatchProtocol(root);
                root->normalize_proxy_to_min_position_vertex(bboxMin_, f_quantStep_);
                facets.push_back(root);
                for (MCGAL::Halfedge* hd : diagonals) {
                    if (!hd || hd->isRemoved()) continue;
                    hd->setAdded();
                    halfedge.push_back(hd);
                    ++recoveredDiagonals;
                }
                for (MCGAL::Facet* zf : selectedZeros) consumed.insert(zf);
                recoveredPatchFaces += static_cast<int>(patchFaceSet.size());
                ++recoveredPatches;

            };
            if (sideInfo != nullptr) {
                SideBitReader br(*sideInfo);
                struct SideMergedComp {
                    int patchCount = 0;
                    std::vector<std::uint32_t> ords;
                };
                std::unordered_map<int, SideMergedComp> mergedByOrdinal;
                const uint32_t hasMerged = br.readBit();
                if (hasMerged != 0u) {
                    const uint32_t mergedCount = br.readBits(12);
                    for (uint32_t mi = 0; mi < mergedCount; ++mi) {
                        const uint32_t compOrdinal = br.readBits(12);
                        const uint32_t patchCount = br.readBits(3);
                        const uint32_t faceCount = br.readBits(6);
                        if (patchCount == 0u) {
                            throw std::runtime_error(
                                "[PCATMStrictPaper decoder] mode-2 merged record "
                                "with zero patches round=" +
                                std::to_string(roundIdx));
                        }
                        int pbits = 1;
                        while ((1 << pbits) < static_cast<int>(patchCount))
                            ++pbits;
                        SideMergedComp rec;
                        rec.patchCount = static_cast<int>(patchCount);
                        rec.ords.reserve(static_cast<size_t>(faceCount));
                        for (uint32_t fi = 0; fi < faceCount; ++fi)
                            rec.ords.push_back(br.readBits(pbits));
                        mergedByOrdinal[static_cast<int>(compOrdinal)] =
                            std::move(rec);
                    }
                }
                const std::uint32_t sidePatchCount = br.readBits(16);
                const std::uint32_t sideCandSum = br.readBits(32);
                if (isEnvOn("PMSF_PCATM_SIDE_DUMP")) {
                    std::cerr << "[2c-dump-hdr] round=" << roundIdx
                              << " patchCount=" << sidePatchCount
                              << " candSum=" << sideCandSum << std::endl;
                }
                const int expectedSidePatches =
                    (roundIdx >= 0 &&
                     roundIdx < static_cast<int>(residualsPerRound_.size()))
                        ? residualsPerRound_[static_cast<size_t>(roundIdx)]
                        : -1;
                if (static_cast<int>(sidePatchCount) != expectedSidePatches) {
                    throw std::runtime_error(
                        "[PCATMStrictPaper decoder] mode-2 side patchCount "
                        "mismatch round=" + std::to_string(roundIdx) +
                        " header=" + std::to_string(sidePatchCount) +
                        " expected=" + std::to_string(expectedSidePatches));
                }
                if (expectedSidePatches <= 0) {
                    throw std::runtime_error(
                        "[PCATMStrictPaper decoder] mode-2 side info without "
                        "expected patch count round=" +
                        std::to_string(roundIdx));
                }
                // v7 (task-36): direct zero-index side stream. The decoder
                // enumerates the same topology-only zero-candidate list as the
                // encoder (faces adjacent to the recovered blue run, ordered by
                // the round's face order), then reads the two indices. The
                // resulting patch is only accepted after the existing tree
                // check; the encoder falls a whole round back to mode 4 when a
                // patch is not patch-exact, so no bad stream is accepted here.
                long long readCandSum = 0;
                int readPatchCount = 0;
                std::unordered_set<MCGAL::Facet*> sideRoundFaces;
                sideRoundFaces.reserve(colorByFace.size() * 2);
                for (const auto& kv : colorByFace) sideRoundFaces.insert(kv.first);
                auto readSideZeros =
                    [&](const std::vector<MCGAL::Facet*>& ones) {
                        bool keysUnique = true;
                        const auto cands =
                            pmsf::pcatm::enumerateMode2ZeroCandidates(
                                ones, sideRoundFaces, mesh_, keysUnique);
                        if (!keysUnique) {
                            throw std::runtime_error(
                                "[PCATMStrictPaper decoder] mode-2 ambiguous "
                                "zero-candidate centroid key round=" +
                                std::to_string(roundIdx));
                        }
                        const int n = static_cast<int>(cands.size());
                        if (n < 2 || n > 64) {
                            throw std::runtime_error(
                                "[PCATMStrictPaper decoder] mode-2 zero "
                                "candidate list size round=" +
                                std::to_string(roundIdx) + " n=" +
                                std::to_string(n));
                        }
                        int bits = 0;
                        while ((1 << bits) < n) ++bits;
                        const std::uint32_t i0 =
                            bits > 0 ? br.readBits(bits) : 0u;
                        const std::uint32_t i1 =
                            bits > 0 ? br.readBits(bits) : 0u;
                        ++readPatchCount;
                        readCandSum += static_cast<long long>(n);
                        if (isEnvOn("PMSF_PCATM_SIDE_DUMP")) {
                            std::cerr << "[2c-dump-dec] round=" << roundIdx
                                      << " i=" << readPatchCount - 1
                                      << " ones=" << ones.size()
                                      << " cands=" << n
                                      << " idx0=" << i0
                                      << " idx1=" << i1 << std::endl;
                        }
                        if (i0 >= static_cast<std::uint32_t>(n) ||
                            i1 >= static_cast<std::uint32_t>(n) || i0 == i1) {
                            throw std::runtime_error(
                                "[PCATMStrictPaper decoder] mode-2 zero index "
                                "OOB round=" + std::to_string(roundIdx) +
                                " idx0=" + std::to_string(i0) + " idx1=" +
                                std::to_string(i1) + " candidates=" +
                                std::to_string(n));
                        }
                        return std::array<MCGAL::Facet*, 2>{
                            {cands[static_cast<std::size_t>(i0)],
                             cands[static_cast<std::size_t>(i1)]}};
                    };
                int sideIdxCursor = 0;
                int compOrdinal = 0;
                for (MCGAL::Facet* f : facesInOrder) {
                    auto it = colorByFace.find(f);
                    if (it == colorByFace.end() || it->second != 1u ||
                        consumed.count(f)) {
                        continue;
                    }
                    std::vector<MCGAL::Facet*> compFaces = floodFillBlue(f);
                    std::sort(compFaces.begin(), compFaces.end(),
                              [&](MCGAL::Facet* a, MCGAL::Facet* b) {
                                  return sideOrderPos[a] < sideOrderPos[b];
                              });
                    auto mit = mergedByOrdinal.find(compOrdinal);
                    if (mit != mergedByOrdinal.end()) {
                        const auto& rec = mit->second;
                        if (rec.ords.size() != compFaces.size()) {
                            throw std::runtime_error(
                                "[PCATMStrictPaper decoder] mode-2 merged record "
                                "face count mismatch round=" +
                                std::to_string(roundIdx));
                        }
                        std::vector<std::vector<MCGAL::Facet*>> groups(
                            static_cast<size_t>(rec.patchCount));
                        for (std::size_t fi = 0; fi < compFaces.size(); ++fi) {
                            const std::uint32_t ord = rec.ords[fi];
                            if (ord >= groups.size()) {
                                throw std::runtime_error(
                                    "[PCATMStrictPaper decoder] mode-2 merged "
                                    "ordinal OOB round=" +
                                    std::to_string(roundIdx));
                            }
                            groups[ord].push_back(compFaces[fi]);
                        }
                        for (const auto& g : groups) {
                            if (g.empty()) {
                                throw std::runtime_error(
                                    "[PCATMStrictPaper decoder] mode-2 merged "
                                    "split underflow round=" +
                                    std::to_string(roundIdx));
                            }
                            recoverPatch(g, readSideZeros(g));
                            ++sideIdxCursor;
                        }
                    } else {
                        recoverPatch(compFaces, readSideZeros(compFaces));
                        ++sideIdxCursor;
                    }
                    ++compOrdinal;
                }
                if (isEnvOn("PMSF_PCATM_MODE2_DIAG")) {
                    std::cerr << "[2c-read] round=" << roundIdx
                              << " patches=" << readPatchCount
                              << " candSum=" << readCandSum << std::endl;
                }
                if (sideIdxCursor != expectedSidePatches) {
                    throw std::runtime_error(
                        "[PCATMStrictPaper decoder] mode-2 side patch count "
                        "mismatch round=" +
                        std::to_string(roundIdx) + " consumed=" +
                        std::to_string(sideIdxCursor) + " expected=" +
                        std::to_string(expectedSidePatches));
                }
                if (static_cast<std::uint32_t>(readCandSum) != sideCandSum) {
                    throw std::runtime_error(
                        "[PCATMStrictPaper decoder] mode-2 candidate-table "
                        "checksum mismatch round=" +
                        std::to_string(roundIdx) + " header=" +
                        std::to_string(sideCandSum) + " decoded=" +
                        std::to_string(readCandSum));
                }
            } else {
                for (MCGAL::Facet* f : facesInOrder) {
                    auto it = colorByFace.find(f);
                    if (it == colorByFace.end() || it->second != 1u ||
                        consumed.count(f)) {
                        continue;
                    }
                    const std::vector<MCGAL::Facet*> ones = floodFillBlue(f);
                    recoverPatch(ones, {{nullptr, nullptr}});
                }
            }

            const int expectedPatches =
                (roundIdx >= 0 && roundIdx < static_cast<int>(residualsPerRound_.size()))
                    ? residualsPerRound_[static_cast<size_t>(roundIdx)]
                    : -1;
            if (const char* env = std::getenv("PMSF_BPV_DIAG"); env && env[0] == '1') {
                std::cerr << "[PCATMStrictPaper 2-color round=" << roundIdx
                          << "] faces=" << facesInOrder.size()
                          << " patches=" << recoveredPatches
                          << " expectedPatches=" << expectedPatches
                          << " diagonals=" << recoveredDiagonals
                          << " oneFaces=" << oneFaces
                          << " patchFaces=" << recoveredPatchFaces
                          << std::endl;
            }
            if (expectedPatches >= 0 && recoveredPatches != expectedPatches) {
                throw std::runtime_error(
                    "[PCATMStrictPaper decoder] mode=2 patch count mismatch round=" +
                    std::to_string(roundIdx) + " recovered=" +
                    std::to_string(recoveredPatches) + " expected=" +
                    std::to_string(expectedPatches));
            }
            currentRoundIdx_++;
            return;
        }

        std::unordered_set<MCGAL::Facet*> consumed;
        int coloredFaces = 0;
        for (const auto& kv : colorByFace) {
            if (kv.second != 0u) ++coloredFaces;
        }
        consumed.reserve(static_cast<size_t>(coloredFaces) * 2 + 16);
        if (const char* env = std::getenv("PMSF_BPV_DIAG"); env && env[0] == '1') {
            std::cerr << "[PCATMStrictPaper color-only-start round=" << roundIdx
                      << " mode=" << colorModePerRound_[static_cast<size_t>(roundIdx)]
                      << " colors=" << roundColors.size()
                      << " faces=" << facesInOrder.size()
                      << " coloredFaces=" << coloredFaces
                      << " expected=" << residualsPerRound_[static_cast<size_t>(roundIdx)]
                      << std::endl;
        }
        int recoveredPatches = 0;
        int recoveredDiagonals = 0;
        int recoveredColoredFaces = 0;
        for (MCGAL::Facet* f : facesInOrder) {
            auto it = colorByFace.find(f);
            if (it == colorByFace.end() || it->second == 0u || consumed.count(f)) continue;
            const uint8_t patchColor = it->second;

            std::vector<MCGAL::Facet*> componentFaces;
            std::vector<MCGAL::Halfedge*> diagonals;
            std::queue<MCGAL::Facet*> cq;
            cq.push(f);
            consumed.insert(f);
            while (!cq.empty()) {
                MCGAL::Facet* cur = cq.front();
                cq.pop();
                componentFaces.push_back(cur);

                MCGAL::Halfedge* start = cur ? cur->proxyHalfedge() : nullptr;
                if (!start) continue;
                MCGAL::Halfedge* h = start;
                do {
                    MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
                    MCGAL::Facet* nf = opp ? opp->face() : nullptr;
                    auto nit = colorByFace.find(nf);
                    if (nit != colorByFace.end() && nit->second == patchColor &&
                        !consumed.count(nf)) {
                        consumed.insert(nf);
                        cq.push(nf);
                        diagonals.push_back(h);
                    }
                    h = h ? h->next() : nullptr;
                } while (h && h != start);
            }

            if (rcDecodedResiduals_.empty()) {
                throw std::runtime_error(
                    "[PCATMStrictPaper decoder] residual underflow in color-only recovery");
            }
            const int valence = static_cast<int>(componentFaces.size()) + 2;
            MCGAL::PointInt residual = rcDecodedResiduals_.front();
            rcDecodedResiduals_.pop_front();
            f->setSplittable();
            f->setPolyValence(valence);
            f->setRemovedVertexPosInt(residual);
            pmsf::markPaperStrictPatchProtocol(f);
            facets.push_back(f);
            f->normalize_proxy_to_min_position_vertex(bboxMin_, f_quantStep_);
            for (MCGAL::Halfedge* hd : diagonals) {
                if (!hd || hd->isRemoved()) continue;
                hd->setAdded();
                halfedge.push_back(hd);
                ++recoveredDiagonals;
            }
            recoveredColoredFaces += static_cast<int>(componentFaces.size());
            ++recoveredPatches;
        }

        const int expectedPatches =
            (roundIdx >= 0 && roundIdx < static_cast<int>(residualsPerRound_.size()))
                ? residualsPerRound_[static_cast<size_t>(roundIdx)]
                : -1;
        if (const char* env = std::getenv("PMSF_BPV_DIAG"); env && env[0] == '1') {
            std::cerr << "[PCATMStrictPaper color-only round=" << roundIdx
                      << "] faces=" << facesInOrder.size()
                      << " patches=" << recoveredPatches
                      << " expectedPatches=" << expectedPatches
                      << " diagonals=" << recoveredDiagonals
                      << " coloredFaces=" << coloredFaces
                      << " recoveredColoredFaces=" << recoveredColoredFaces
                      << std::endl;
        }
        if (expectedPatches >= 0 && recoveredPatches != expectedPatches) {
            throw std::runtime_error(
                "[PCATMStrictPaper decoder] color-only patch count mismatch round=" +
                std::to_string(roundIdx) + " recovered=" +
                std::to_string(recoveredPatches) + " expected=" +
                std::to_string(expectedPatches));
        }
        if (recoveredColoredFaces != coloredFaces ||
            recoveredDiagonals != recoveredColoredFaces - recoveredPatches) {
            throw std::runtime_error(
                "[PCATMStrictPaper decoder] color-only component coverage mismatch round=" +
                std::to_string(roundIdx));
        }

        currentRoundIdx_++;
        return;
    }

    if (rcDecodedIsPoly_.empty() && !rcDecodedValCodes_.empty()) {
        throw std::runtime_error("[PCATMStrictPaper decoder] internal isPoly side stream missing");
    }
    if (rcDecodedIsPoly_.empty()) {
        throw std::runtime_error(
            "[PCATMStrictPaper decoder] color-only patch recovery is not implemented for sv=1/2. "
            "Re-encode with sv=3 mixed 2/4-color storage side streams.");
    }

    std::queue<int> localGateQueue;
    if (seed != nullptr) localGateQueue.push(seed->poolId());
    int currentVersion = MCGAL::BFSVersionManager::current_version++;

    while (!localGateQueue.empty()) {
        int hid = localGateQueue.front();
        localGateQueue.pop();
        MCGAL::Halfedge* h =
            MCGAL::contextPool.getHalfedgeByIndexInSubPool(DEFAULT_MESH_ID, hid);
        if (!h || h->isVisited(currentVersion) || h->isRemoved()) continue;
        h->setVisited(currentVersion);

        MCGAL::Facet* f = h->face();
        bool faceFirstVisit = f && !f->isVisited(currentVersion);
        if (faceFirstVisit) {
            f->setVisited(currentVersion);
            f->normalize_proxy_to_min_position_vertex(bboxMin_, f_quantStep_);

            if (rcDecodedIsPoly_.empty()) {
                throw std::runtime_error("[PCATMStrictPaper decoder] isPoly underflow");
            }
            uint8_t isPoly = rcDecodedIsPoly_.front();
            rcDecodedIsPoly_.pop_front();

            if (isPoly) {
                if (rcDecodedValCodes_.empty() || rcDecodedResiduals_.empty()) {
                    throw std::runtime_error("[PCATMStrictPaper decoder] valence/residual underflow");
                }
                uint8_t code = rcDecodedValCodes_.front();
                rcDecodedValCodes_.pop_front();
                int valence = 0;
                if (code == static_cast<uint8_t>(RDValenceCode::VAL_OTHER)) {
                    if (rcDecodedOtherVals_.empty()) {
                        throw std::runtime_error("[PCATMStrictPaper decoder] V_OTHER underflow");
                    }
                    valence = static_cast<int>(rcDecodedOtherVals_.front());
                    rcDecodedOtherVals_.pop_front();
                } else {
                    valence = 3 + static_cast<int>(code);
                }

                MCGAL::PointInt residual = rcDecodedResiduals_.front();
                rcDecodedResiduals_.pop_front();
                f->setSplittable();
                f->setPolyValence(valence);
                f->setRemovedVertexPosInt(residual);
                pmsf::markPaperStrictPatchProtocol(f);
                facets.push_back(f);
                f->normalize_proxy_to_min_position_vertex(bboxMin_, f_quantStep_);

                pushFanDiagonals(f, valence, halfedge);
            }
        }

        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp && !hOpp->isVisited(currentVersion)) localGateQueue.push(hOpp->poolId());
            if (hIt && !hIt->isVisited(currentVersion)) localGateQueue.push(hIt->poolId());
            hIt = hIt->next();
        } while (hIt != h && hIt != nullptr);
    }

    (void)roundIdx;
    currentRoundIdx_++;
}
