// SPDX-License-Identifier: Apache-2.0
//
// PCATMStrictPMSFSymbolCollectOperator.cpp

#include "common/strict/PCATMStrictPMSFSymbolCollectOperator.h"
#include "common/strict/PCATMStrictAdaptiveResidual.h"
#include "PCATM/PCATMPaperMode2.h"
#include "PCATM/boundary/PCATMPMSFBoundary.h"
#include "common/PaperLayoutAliases.h"
#include "core/PMSFBoundarySideChannel.h"
#include "core/PaperGeometrySideChannel.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include "BFSVersionManager.h"
#include "Configuration.h"
#include "ContextPool.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "PCATM/PCATMLZEncoder.h"
#include "Point.h"
#include "RD/compress/RDSymbolCollectOperator.h"
#include "compress/ConnectivityBlobContainer.h"
#include "compress/RangeCoderStreamEncoder.h"
#include "entropy/RangeCoderEntropyCoder.h"
#include "entropy/StaticHuffmanCoder.h"
#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iostream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

void appendI32(std::vector<uint8_t>& out, int32_t v) {
    const uint32_t u = static_cast<uint32_t>(v);
    out.push_back(static_cast<uint8_t>(u & 0xffu));
    out.push_back(static_cast<uint8_t>((u >> 8) & 0xffu));
    out.push_back(static_cast<uint8_t>((u >> 16) & 0xffu));
    out.push_back(static_cast<uint8_t>((u >> 24) & 0xffu));
}

void appendVarU32(std::vector<uint8_t>& out, uint32_t v) {
    while (v >= 0x80u) {
        out.push_back(static_cast<uint8_t>((v & 0x7fu) | 0x80u));
        v >>= 7;
    }
    out.push_back(static_cast<uint8_t>(v));
}

std::vector<uint8_t> zlibDeflateStrategy(const uint8_t* data, size_t size, int level, int strategy) {
    if (size == 0)
        return {};
    z_stream zs{};
    int rc = deflateInit2(&zs, level, Z_DEFLATED, MAX_WBITS, 9, strategy);
    if (rc != Z_OK) {
        throw std::runtime_error("[PCATMStrictPaper] zlib deflateInit2 failed");
    }

    uLongf bound = compressBound(static_cast<uLong>(size));
    std::vector<uint8_t> out(bound);
    zs.next_in = const_cast<Bytef*>(reinterpret_cast<const Bytef*>(data));
    zs.avail_in = static_cast<uInt>(size);
    zs.next_out = reinterpret_cast<Bytef*>(out.data());
    zs.avail_out = static_cast<uInt>(out.size());
    rc = deflate(&zs, Z_FINISH);
    if (rc != Z_STREAM_END) {
        deflateEnd(&zs);
        throw std::runtime_error("[PCATMStrictPaper] zlib deflate failed");
    }
    out.resize(zs.total_out);
    deflateEnd(&zs);
    return out;
}

std::vector<uint8_t> zlibDeflateBestStrategy(const std::vector<uint8_t>& data) {
    std::vector<uint8_t> best = pmsf::pcatm::lzDeflate(data, 9);
    const int strategies[] = {
        Z_FILTERED,
        Z_HUFFMAN_ONLY,
        Z_RLE,
        Z_FIXED,
    };
    for (int strategy : strategies) {
        std::vector<uint8_t> cur = zlibDeflateStrategy(data.data(), data.size(), 9, strategy);
        if (cur.size() < best.size())
            best = std::move(cur);
    }
    return best;
}

std::vector<uint8_t> encodeBitsContextRange(const std::vector<uint8_t>& bits, int ctxBits = 8) {
    ctxBits = std::clamp(ctxBits, 0, 12);
    const int kCtxCount = 1 << ctxBits;
    const std::size_t bufSize = std::max<std::size_t>(bits.size() * 2 + 4096, 4096);
    std::vector<char> buf(bufSize);

    std::deque<uint8_t> bitsDeque(bits.begin(), bits.end());
    int ctx = 0;
    const int ctxMask = kCtxCount - 1;
    auto deriveCtx = [&ctx, ctxMask](int prev) {
        int c = ctx;
        if (prev >= 0) {
            ctx = ((ctx << 1) | prev) & ctxMask;
        }
        return c;
    };

    std::size_t rcSize = pmsf::encodeContextAdaptiveUint8Queue(bitsDeque, 2, kCtxCount, deriveCtx, nullptr, 0, 0, buf.data(), bufSize);
    return std::vector<uint8_t>(reinterpret_cast<uint8_t*>(buf.data()), reinterpret_cast<uint8_t*>(buf.data()) + rcSize);
}

std::vector<uint8_t> dequeToVector(const std::deque<uint8_t>& q) {
    std::vector<uint8_t> out;
    out.reserve(q.size());
    for (uint8_t v : q)
        out.push_back(v ? 1u : 0u);
    return out;
}

uint32_t zigzag32(int32_t v) {
    return (static_cast<uint32_t>(v) << 1) ^ static_cast<uint32_t>(v >> 31);
}

bool isEnvOn(const char* name) {
    return pmsf::pmsfStorageEnvEnabled(name);
}

void requirePCATMStrictPaperExactIfRequested() {
    if (!pmsf::boundaryFlagOn(pmsf::pcatm_boundary::kStrictPaperExactSpec)) {
        return;
    }
    throw std::runtime_error("[PCATMStrictPaper] paper-exact PCATM path requested, but exact "
                             "Z-shape patch semantics are not implemented in this strict storage "
                             "path yet");
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

std::vector<uint8_t> encodeColorLz(const std::deque<uint8_t>& colorBits) {
    std::vector<uint8_t> bits = dequeToVector(colorBits);
    std::vector<uint8_t> packed = pmsf::pcatm::packBits(bits);
    std::vector<uint8_t> lz = pmsf::pcatm::lzDeflate(packed, 9);

    std::vector<uint8_t> out;
    appendI32(out, static_cast<int32_t>(bits.size()));
    appendI32(out, static_cast<int32_t>(packed.size()));
    appendI32(out, static_cast<int32_t>(lz.size()));
    out.insert(out.end(), lz.begin(), lz.end());
    return out;
}

std::vector<uint8_t> encodeColorLzV3(const std::deque<uint8_t>& colorBits, int colorMode) {
    const int bitsPerSymbol = (colorMode == 4) ? 2 : 1;
    std::vector<uint8_t> bits;
    bits.reserve(colorBits.size() * static_cast<size_t>(bitsPerSymbol));
    for (uint8_t raw : colorBits) {
        uint8_t sym = raw ? 1u : 0u;
        if (colorMode == 4) {
            bits.push_back((sym >> 1) & 1u);
            bits.push_back(sym & 1u);
        } else {
            bits.push_back(sym & 1u);
        }
    }
    std::vector<uint8_t> packed = pmsf::pcatm::packBits(bits);
    std::vector<uint8_t> lz = pmsf::pcatm::lzDeflate(packed, 9);

    std::vector<uint8_t> out;
    appendI32(out, static_cast<int32_t>(colorMode));
    appendI32(out, static_cast<int32_t>(colorBits.size()));
    appendI32(out, static_cast<int32_t>(bitsPerSymbol));
    appendI32(out, static_cast<int32_t>(packed.size()));
    appendI32(out, static_cast<int32_t>(lz.size()));
    out.insert(out.end(), lz.begin(), lz.end());
    return out;
}

std::vector<uint8_t> deriveMode4PatchSymbols(const std::deque<uint8_t>& isPolyBits, const std::deque<uint8_t>& edgeSyms) {
    std::vector<uint8_t> symbols;
    symbols.reserve(isPolyBits.size());
    for (size_t i = 0; i < isPolyBits.size(); ++i) {
        if (isPolyBits[i]) {
            symbols.push_back(1u);
            continue;
        }
        const size_t e = i * 3;
        const bool hasAdded = e + 2 < edgeSyms.size() && (edgeSyms[e] || edgeSyms[e + 1] || edgeSyms[e + 2]);
        if (!hasAdded) {
            symbols.push_back(0u);
        } else {
            const int addedCount =
                static_cast<int>(edgeSyms[e] ? 1 : 0) + static_cast<int>(edgeSyms[e + 1] ? 1 : 0) + static_cast<int>(edgeSyms[e + 2] ? 1 : 0);
            // 4-color decoder semantics:
            //   0 = white/nonpatch
            //   1 = patch root/polyFace
            //   2 = interior child triangle
            //   3 = last child triangle
            // In v0-fan topology, interior child faces have two added fan
            // diagonals; the last child has only the previous diagonal.
            symbols.push_back(static_cast<uint8_t>(addedCount <= 1 ? 3u : 2u));
        }
    }
    return symbols;
}

std::vector<uint8_t> encodeColorSymbolsLzV4(const std::vector<uint8_t>& symbols, int colorMode) {
    const int bitsPerSymbol = (colorMode == 4) ? 2 : 1;
    std::vector<uint8_t> bits;
    bits.reserve(symbols.size() * static_cast<size_t>(bitsPerSymbol));
    for (uint8_t raw : symbols) {
        uint8_t sym = raw & ((colorMode == 4) ? 3u : 1u);
        if (bitsPerSymbol == 2) {
            bits.push_back((sym >> 1) & 1u);
            bits.push_back(sym & 1u);
        } else {
            bits.push_back(sym & 1u);
        }
    }
    std::vector<uint8_t> packed = pmsf::pcatm::packBits(bits);
    std::vector<uint8_t> lz = pmsf::pcatm::lzDeflate(packed, 9);

    std::vector<uint8_t> bitpackedOut;
    appendI32(bitpackedOut, static_cast<int32_t>(colorMode));
    appendI32(bitpackedOut, static_cast<int32_t>(symbols.size()));
    appendI32(bitpackedOut, static_cast<int32_t>(bitsPerSymbol));
    appendI32(bitpackedOut, static_cast<int32_t>(packed.size()));
    appendI32(bitpackedOut, static_cast<int32_t>(lz.size()));
    bitpackedOut.insert(bitpackedOut.end(), lz.begin(), lz.end());

    std::vector<uint8_t> rawPackedOut;
    appendI32(rawPackedOut, static_cast<int32_t>(colorMode));
    appendI32(rawPackedOut, static_cast<int32_t>(symbols.size()));
    appendI32(rawPackedOut, -4);
    appendI32(rawPackedOut, static_cast<int32_t>(packed.size()));
    appendI32(rawPackedOut, static_cast<int32_t>(packed.size()));
    rawPackedOut.insert(rawPackedOut.end(), packed.begin(), packed.end());

    std::vector<uint8_t> rleRaw;
    for (size_t i = 0; i < symbols.size();) {
        const uint8_t sym = symbols[i] & ((colorMode == 4) ? 3u : 1u);
        size_t j = i + 1;
        while (j < symbols.size() && (symbols[j] & ((colorMode == 4) ? 3u : 1u)) == sym && j - i < 0xffffffffull) {
            ++j;
        }
        appendVarU32(rleRaw, static_cast<uint32_t>(j - i));
        rleRaw.push_back(sym);
        i = j;
    }
    std::vector<uint8_t> rleLz = pmsf::pcatm::lzDeflate(rleRaw, 9);
    std::vector<uint8_t> rleOut;
    appendI32(rleOut, static_cast<int32_t>(colorMode));
    appendI32(rleOut, static_cast<int32_t>(symbols.size()));
    appendI32(rleOut, -1);
    appendI32(rleOut, static_cast<int32_t>(rleRaw.size()));
    appendI32(rleOut, static_cast<int32_t>(rleLz.size()));
    rleOut.insert(rleOut.end(), rleLz.begin(), rleLz.end());

    std::vector<uint32_t> rangeSymbols;
    rangeSymbols.reserve(symbols.size());
    for (uint8_t raw : symbols) {
        rangeSymbols.push_back(raw & ((colorMode == 4) ? 3u : 1u));
    }
    pmsf::RangeCoderEntropyCoder colorCoder(bitsPerSymbol);
    std::vector<uint8_t> rcPayload;
    colorCoder.encode(rangeSymbols, rcPayload);
    std::vector<uint8_t> rangeOut;
    appendI32(rangeOut, static_cast<int32_t>(colorMode));
    appendI32(rangeOut, static_cast<int32_t>(symbols.size()));
    appendI32(rangeOut, -2);
    appendI32(rangeOut, static_cast<int32_t>(symbols.size()));
    appendI32(rangeOut, static_cast<int32_t>(rcPayload.size()));
    rangeOut.insert(rangeOut.end(), rcPayload.begin(), rcPayload.end());

    std::vector<uint8_t> sparseRaw;
    uint32_t zeroRun = 0;
    const uint8_t mask = (colorMode == 4) ? 3u : 1u;
    for (uint8_t raw : symbols) {
        const uint8_t sym = raw & mask;
        if (sym == 0u) {
            ++zeroRun;
            continue;
        }
        appendVarU32(sparseRaw, zeroRun);
        sparseRaw.push_back(sym);
        zeroRun = 0;
    }
    std::vector<uint8_t> sparseLz = pmsf::pcatm::lzDeflate(sparseRaw, 9);
    std::vector<uint8_t> sparseOut;
    appendI32(sparseOut, static_cast<int32_t>(colorMode));
    appendI32(sparseOut, static_cast<int32_t>(symbols.size()));
    appendI32(sparseOut, -3);
    appendI32(sparseOut, static_cast<int32_t>(sparseRaw.size()));
    appendI32(sparseOut, static_cast<int32_t>(sparseLz.size()));
    sparseOut.insert(sparseOut.end(), sparseLz.begin(), sparseLz.end());

    const std::vector<uint8_t>* best = &bitpackedOut;
    if (rawPackedOut.size() < best->size())
        best = &rawPackedOut;
    if (rleOut.size() < best->size())
        best = &rleOut;
    if (rangeOut.size() < best->size())
        best = &rangeOut;
    if (sparseOut.size() < best->size())
        best = &sparseOut;
    return *best;
}

std::vector<uint8_t>
encodeColorRoundsGlobal(const std::vector<std::vector<uint8_t>>& rounds, const std::vector<int>& modes, int* selectedCodec = nullptr) {
    std::vector<uint32_t> flat;
    size_t total = 0;
    for (const auto& r : rounds)
        total += r.size();
    flat.reserve(total);
    const bool allMode2 = !modes.empty() && std::all_of(modes.begin(), modes.end(), [](int mode) { return mode == 2; });
    const int bitsPerSymbol = allMode2 ? 1 : 2;
    for (size_t r = 0; r < rounds.size(); ++r) {
        const uint8_t mask = (modes[r] == 2) ? 1u : 3u;  // mode 3/4 -> 2-bit
        for (uint8_t sym : rounds[r])
            flat.push_back(sym & mask);
    }

    auto makeHeader = [&](int codec, size_t payloadSize) {
        std::vector<uint8_t> out;
        appendI32(out, static_cast<int32_t>(rounds.size()));
        for (size_t r = 0; r < rounds.size(); ++r) {
            appendI32(out, static_cast<int32_t>(modes[r]));
            appendI32(out, static_cast<int32_t>(rounds[r].size()));
        }
        appendI32(out, static_cast<int32_t>(flat.size()));
        appendI32(out, codec);
        appendI32(out, static_cast<int32_t>(payloadSize));
        return out;
    };

    std::vector<uint8_t> bits;
    bits.reserve(flat.size() * static_cast<size_t>(bitsPerSymbol));
    for (uint32_t sym : flat) {
        if (bitsPerSymbol == 1) {
            bits.push_back(static_cast<uint8_t>(sym & 1u));
        } else {
            bits.push_back(static_cast<uint8_t>((sym >> 1) & 1u));
            bits.push_back(static_cast<uint8_t>(sym & 1u));
        }
    }
    std::vector<uint8_t> packed = pmsf::pcatm::packBits(bits);

    std::vector<uint8_t> raw = makeHeader(0, packed.size());
    raw.insert(raw.end(), packed.begin(), packed.end());

    std::vector<uint8_t> packedLz = pmsf::pcatm::lzDeflate(packed, 9);
    std::vector<uint8_t> zlib = makeHeader(1, packedLz.size());
    zlib.insert(zlib.end(), packedLz.begin(), packedLz.end());

    pmsf::StaticHuffmanCoder huffCoder;
    std::vector<uint8_t> huffPayload;
    huffCoder.encode(flat, huffPayload);
    std::vector<uint8_t> huff = makeHeader(2, huffPayload.size());
    huff.insert(huff.end(), huffPayload.begin(), huffPayload.end());

    pmsf::RangeCoderEntropyCoder rangeCoder(bitsPerSymbol);
    std::vector<uint8_t> rangePayload;
    rangeCoder.encode(flat, rangePayload);
    std::vector<uint8_t> range = makeHeader(3, rangePayload.size());
    range.insert(range.end(), rangePayload.begin(), rangePayload.end());

    std::vector<uint8_t> rleRaw;
    for (size_t i = 0; i < flat.size();) {
        const uint32_t sym = flat[i];
        size_t j = i + 1;
        while (j < flat.size() && flat[j] == sym && j - i < 0xffffffffull) {
            ++j;
        }
        appendVarU32(rleRaw, static_cast<uint32_t>(j - i));
        appendVarU32(rleRaw, sym);
        i = j;
    }
    std::vector<uint8_t> rleLz = pmsf::pcatm::lzDeflate(rleRaw, 9);
    std::vector<uint8_t> rlePayload;
    appendI32(rlePayload, static_cast<int32_t>(rleRaw.size()));
    rlePayload.insert(rlePayload.end(), rleLz.begin(), rleLz.end());
    std::vector<uint8_t> rle = makeHeader(4, rlePayload.size());
    rle.insert(rle.end(), rlePayload.begin(), rlePayload.end());

    std::vector<uint8_t> sparseRaw;
    uint32_t zeroRun = 0;
    for (uint32_t sym : flat) {
        if (sym == 0u) {
            ++zeroRun;
            continue;
        }
        appendVarU32(sparseRaw, zeroRun);
        appendVarU32(sparseRaw, sym);
        zeroRun = 0;
    }
    std::vector<uint8_t> sparseLz = pmsf::pcatm::lzDeflate(sparseRaw, 9);
    std::vector<uint8_t> sparsePayload;
    appendI32(sparsePayload, static_cast<int32_t>(sparseRaw.size()));
    sparsePayload.insert(sparsePayload.end(), sparseLz.begin(), sparseLz.end());
    std::vector<uint8_t> sparse = makeHeader(5, sparsePayload.size());
    sparse.insert(sparse.end(), sparsePayload.begin(), sparsePayload.end());

    std::vector<uint8_t> nzRunRaw;
    for (size_t i = 0; i < flat.size();) {
        uint32_t z = 0;
        while (i < flat.size() && flat[i] == 0u && z < 0xffffffffu) {
            ++i;
            ++z;
        }
        if (i >= flat.size())
            break;
        const uint32_t sym = flat[i];
        uint32_t run = 0;
        while (i < flat.size() && flat[i] == sym && sym != 0u && run < 0xffffffffu) {
            ++i;
            ++run;
        }
        appendVarU32(nzRunRaw, z);
        appendVarU32(nzRunRaw, run);
        appendVarU32(nzRunRaw, sym);
    }
    std::vector<uint8_t> nzRunLz = pmsf::pcatm::lzDeflate(nzRunRaw, 9);
    std::vector<uint8_t> nzRunPayload;
    appendI32(nzRunPayload, static_cast<int32_t>(nzRunRaw.size()));
    nzRunPayload.insert(nzRunPayload.end(), nzRunLz.begin(), nzRunLz.end());
    std::vector<uint8_t> nzRun = makeHeader(6, nzRunPayload.size());
    nzRun.insert(nzRun.end(), nzRunPayload.begin(), nzRunPayload.end());

    std::vector<uint8_t> ctxRangePayload = encodeBitsContextRange(bits);
    std::vector<uint8_t> ctxRange = makeHeader(7, ctxRangePayload.size());
    ctxRange.insert(ctxRange.end(), ctxRangePayload.begin(), ctxRangePayload.end());

    std::vector<uint8_t> adaptiveCtxPayload;
    int adaptiveCtxBits = 8;
    for (int ctxBits : {4, 6, 8, 10, 12}) {
        std::vector<uint8_t> candidate = encodeBitsContextRange(bits, ctxBits);
        if (adaptiveCtxPayload.empty() || candidate.size() < adaptiveCtxPayload.size()) {
            adaptiveCtxPayload = std::move(candidate);
            adaptiveCtxBits = ctxBits;
        }
    }
    std::vector<uint8_t> adaptiveCtxBody;
    appendI32(adaptiveCtxBody, adaptiveCtxBits);
    adaptiveCtxBody.insert(adaptiveCtxBody.end(), adaptiveCtxPayload.begin(), adaptiveCtxPayload.end());
    std::vector<uint8_t> adaptiveCtxRange = makeHeader(8, adaptiveCtxBody.size());
    adaptiveCtxRange.insert(adaptiveCtxRange.end(), adaptiveCtxBody.begin(), adaptiveCtxBody.end());

    if (const char* env = std::getenv("PMSF_BPV_DIAG"); env && env[0] == '1') {
        size_t ones = 0;
        for (uint32_t sym : flat) {
            if (sym != 0u)
                ++ones;
        }
        std::cerr << "[PCATMStrictPaper color-codec-candidates]"
                  << " symbols=" << flat.size() << " ones=" << ones << " density=" << (flat.empty() ? 0.0 : double(ones) / double(flat.size()))
                  << " raw=" << raw.size() << " zlib=" << zlib.size() << " huff=" << huff.size() << " range=" << range.size() << " rle=" << rle.size()
                  << " sparse=" << sparse.size() << " nzRun=" << nzRun.size() << " ctxRange=" << ctxRange.size()
                  << " adaptiveCtxRange=" << adaptiveCtxRange.size() << " adaptiveCtxBits=" << adaptiveCtxBits << std::endl;
    }

    const bool forceRaw = isEnvOn("PMSF_PCATM_STRICT_FORCE_COLOR_RAW");
    const bool forceZlib = isEnvOn("PMSF_PCATM_STRICT_FORCE_COLOR_ZLIB");
    const bool forceHuffman = isEnvOn("PMSF_PCATM_STRICT_FORCE_COLOR_HUFFMAN");
    const bool forceRange = isEnvOn("PMSF_PCATM_STRICT_FORCE_COLOR_RANGE");
    const bool forceRle = isEnvOn("PMSF_PCATM_STRICT_FORCE_COLOR_RLE");
    const bool forceSparse = isEnvOn("PMSF_PCATM_STRICT_FORCE_COLOR_SPARSE");
    const bool forceNzRun = isEnvOn("PMSF_PCATM_STRICT_FORCE_COLOR_NZRUN");
    const bool forceCtxRange = isEnvOn("PMSF_PCATM_STRICT_FORCE_COLOR_CTXRANGE");
    const bool forceAdaptiveCtxRange = isEnvOn("PMSF_PCATM_STRICT_FORCE_COLOR_ADAPTIVE_CTXRANGE");
    const int forceCount = static_cast<int>(forceRaw) + static_cast<int>(forceZlib) + static_cast<int>(forceHuffman) + static_cast<int>(forceRange) +
                           static_cast<int>(forceRle) + static_cast<int>(forceSparse) + static_cast<int>(forceNzRun) +
                           static_cast<int>(forceCtxRange) + static_cast<int>(forceAdaptiveCtxRange);
    if (forceCount > 1) {
        throw std::runtime_error("[PCATMStrictPaper] multiple forced color codecs");
    }
    if (forceRaw) {
        if (selectedCodec)
            *selectedCodec = 0;
        return raw;
    }
    if (forceZlib) {
        if (selectedCodec)
            *selectedCodec = 1;
        return zlib;
    }
    if (forceHuffman) {
        if (selectedCodec)
            *selectedCodec = 2;
        return huff;
    }
    if (forceRange) {
        if (selectedCodec)
            *selectedCodec = 3;
        return range;
    }
    if (forceRle) {
        if (selectedCodec)
            *selectedCodec = 4;
        return rle;
    }
    if (forceSparse) {
        if (selectedCodec)
            *selectedCodec = 5;
        return sparse;
    }
    if (forceNzRun) {
        if (selectedCodec)
            *selectedCodec = 6;
        return nzRun;
    }
    if (forceCtxRange) {
        if (selectedCodec)
            *selectedCodec = 7;
        return ctxRange;
    }
    if (forceAdaptiveCtxRange) {
        if (selectedCodec)
            *selectedCodec = 8;
        return adaptiveCtxRange;
    }

    // WS-B task-18: with mixed per-round color modes (mode 2 => 1 bit/face,
    // mode 3/4 => 2 bit/face) the bit-oriented codecs (raw/zlib/ctxRange/
    // adaptiveCtxRange) would need a per-round bit-width format. Keep the blob
    // format unchanged instead: symbol-oriented codecs (2..6) encode the symbol
    // vector directly and the decoder already derives bitsPerSymbol from the
    // per-round modes header. So in mixed mode only consider those.
    const bool mixedModes =
        !modes.empty() && !allMode2 &&
        std::any_of(modes.begin(), modes.end(),
                    [](int mode) { return mode == 2; });
    const std::vector<uint8_t>* best = nullptr;
    int bestCodec = -1;
    auto consider = [&](const std::vector<uint8_t>& candidate, int codec) {
        if (best == nullptr || candidate.size() < best->size()) {
            best = &candidate;
            bestCodec = codec;
        }
    };
    if (mixedModes) {
        consider(huff, 2);
        consider(range, 3);
        consider(rle, 4);
        consider(sparse, 5);
        consider(nzRun, 6);
    } else {
        consider(raw, 0);
        consider(zlib, 1);
        consider(huff, 2);
        consider(range, 3);
        consider(rle, 4);
        consider(sparse, 5);
        consider(nzRun, 6);
        consider(ctxRange, 7);
        consider(adaptiveCtxRange, 8);
    }
    if (selectedCodec)
        *selectedCodec = bestCodec;
    return *best;
}

// WS-B task-18: per-round mode-2 side info = one 2-bit candidate zero-pair
// index per patch (decoder recovery order). Appended after the self-describing
// per-round color stream; the color decoder ignores trailing bytes.
std::vector<uint8_t> encodeMode2SideInfo(const std::vector<uint8_t>& payload) {
    // WS-B task-18 side-info v3 ("C2S3"): payload is the bit-packed merged-
    // component + 2-bit candidate-index record stream; it is LZ-compressed
    // before being appended after the self-describing round color stream.
    std::vector<uint8_t> out;
    appendI32(out, 0x33533243);  // "C2S3"
    appendI32(out, static_cast<int32_t>(payload.size()));
    const std::vector<uint8_t> lz = pmsf::pcatm::lzDeflate(payload, 9);
    appendI32(out, static_cast<int32_t>(lz.size()));
    out.insert(out.end(), lz.begin(), lz.end());
    return out;
}

std::vector<uint8_t> encodeValenceRaw(const std::deque<uint8_t>& valCodes, const std::deque<uint8_t>& otherVals) {
    std::vector<uint8_t> out;
    appendI32(out, static_cast<int32_t>(valCodes.size()));
    appendI32(out, static_cast<int32_t>(otherVals.size()));
    out.insert(out.end(), valCodes.begin(), valCodes.end());
    out.insert(out.end(), otherVals.begin(), otherVals.end());
    return out;
}

std::vector<uint8_t> encodeBitsLzRaw(const std::deque<uint8_t>& bitsIn) {
    std::vector<uint8_t> bits = dequeToVector(bitsIn);
    std::vector<uint8_t> packed = pmsf::pcatm::packBits(bits);
    std::vector<uint8_t> lz = pmsf::pcatm::lzDeflate(packed, 9);
    std::vector<uint8_t> out;
    appendI32(out, static_cast<int32_t>(bits.size()));
    appendI32(out, static_cast<int32_t>(packed.size()));
    appendI32(out, static_cast<int32_t>(lz.size()));
    out.insert(out.end(), lz.begin(), lz.end());
    return out;
}

std::vector<uint8_t> encodeResidualHuffman(const std::deque<MCGAL::PointInt>& residuals) {
    std::vector<uint32_t> symbols;
    symbols.reserve(residuals.size() * 3);
    for (const auto& r : residuals) {
        symbols.push_back(zigzag32(r.x()));
        symbols.push_back(zigzag32(r.y()));
        symbols.push_back(zigzag32(r.z()));
    }

    pmsf::StaticHuffmanCoder coder;
    std::vector<uint8_t> payload;
    coder.encode(symbols, payload);

    std::vector<uint8_t> out;
    appendI32(out, static_cast<int32_t>(residuals.size()));
    appendI32(out, static_cast<int32_t>(symbols.size()));
    appendI32(out, static_cast<int32_t>(payload.size()));
    out.insert(out.end(), payload.begin(), payload.end());
    return out;
}

std::vector<uint8_t> encodeResidualGlobalHuffman(const std::vector<std::deque<MCGAL::PointInt>>& residualRounds) {
    std::vector<uint32_t> symbols;
    std::vector<int32_t> residualCounts;
    residualCounts.reserve(residualRounds.size());

    int32_t totalResiduals = 0;
    for (const auto& residuals : residualRounds) {
        residualCounts.push_back(static_cast<int32_t>(residuals.size()));
        totalResiduals += static_cast<int32_t>(residuals.size());
        for (const auto& r : residuals) {
            symbols.push_back(zigzag32(r.x()));
            symbols.push_back(zigzag32(r.y()));
            symbols.push_back(zigzag32(r.z()));
        }
    }

    pmsf::StaticHuffmanCoder coder;
    std::vector<uint8_t> payload;
    coder.encode(symbols, payload);

    std::vector<uint8_t> out;
    appendI32(out, static_cast<int32_t>(residualCounts.size()));
    for (int32_t count : residualCounts)
        appendI32(out, count);
    appendI32(out, totalResiduals);
    appendI32(out, static_cast<int32_t>(symbols.size()));
    appendI32(out, static_cast<int32_t>(payload.size()));
    out.insert(out.end(), payload.begin(), payload.end());
    return out;
}

std::vector<uint8_t> encodeResidualGlobalVarintZlib(const std::vector<std::deque<MCGAL::PointInt>>& residualRounds) {
    std::vector<uint8_t> raw;
    std::vector<int32_t> residualCounts;
    residualCounts.reserve(residualRounds.size());

    int32_t totalResiduals = 0;
    for (const auto& residuals : residualRounds) {
        residualCounts.push_back(static_cast<int32_t>(residuals.size()));
        totalResiduals += static_cast<int32_t>(residuals.size());
        for (const auto& r : residuals) {
            appendVarU32(raw, zigzag32(r.x()));
            appendVarU32(raw, zigzag32(r.y()));
            appendVarU32(raw, zigzag32(r.z()));
        }
    }

    std::vector<uint8_t> lz = zlibDeflateBestStrategy(raw);

    std::vector<uint8_t> out;
    appendI32(out, static_cast<int32_t>(residualCounts.size()));
    for (int32_t count : residualCounts)
        appendI32(out, count);
    appendI32(out, totalResiduals);
    appendI32(out, totalResiduals * 3);
    appendI32(out, static_cast<int32_t>(raw.size()));
    appendI32(out, static_cast<int32_t>(lz.size()));
    out.insert(out.end(), lz.begin(), lz.end());
    return out;
}

std::vector<uint8_t> encodeResidualGlobalVarintByteRange(const std::vector<std::deque<MCGAL::PointInt>>& residualRounds) {
    std::vector<uint8_t> raw;
    std::vector<int32_t> residualCounts;
    residualCounts.reserve(residualRounds.size());

    int32_t totalResiduals = 0;
    for (const auto& residuals : residualRounds) {
        residualCounts.push_back(static_cast<int32_t>(residuals.size()));
        totalResiduals += static_cast<int32_t>(residuals.size());
        for (const auto& r : residuals) {
            appendVarU32(raw, zigzag32(r.x()));
            appendVarU32(raw, zigzag32(r.y()));
            appendVarU32(raw, zigzag32(r.z()));
        }
    }

    std::vector<uint32_t> bytes;
    bytes.reserve(raw.size());
    for (uint8_t b : raw)
        bytes.push_back(static_cast<uint32_t>(b));

    pmsf::RangeCoderEntropyCoder coder(8);
    std::vector<uint8_t> payload;
    coder.encode(bytes, payload);

    std::vector<uint8_t> out;
    appendI32(out, static_cast<int32_t>(residualCounts.size()));
    for (int32_t count : residualCounts)
        appendI32(out, count);
    appendI32(out, totalResiduals);
    appendI32(out, totalResiduals * 3);
    appendI32(out, static_cast<int32_t>(raw.size()));
    appendI32(out, static_cast<int32_t>(payload.size()));
    out.insert(out.end(), payload.begin(), payload.end());
    return out;
}

std::vector<uint8_t> encodeResidualGlobalAxisVarintZlib(const std::vector<std::deque<MCGAL::PointInt>>& residualRounds) {
    std::vector<MCGAL::PointInt> flat;
    std::vector<int32_t> residualCounts;
    residualCounts.reserve(residualRounds.size());

    int32_t totalResiduals = 0;
    for (const auto& residuals : residualRounds) {
        residualCounts.push_back(static_cast<int32_t>(residuals.size()));
        totalResiduals += static_cast<int32_t>(residuals.size());
        flat.insert(flat.end(), residuals.begin(), residuals.end());
    }

    std::vector<uint8_t> raw;
    raw.reserve(flat.size() * 3);
    for (const auto& r : flat)
        appendVarU32(raw, zigzag32(r.x()));
    for (const auto& r : flat)
        appendVarU32(raw, zigzag32(r.y()));
    for (const auto& r : flat)
        appendVarU32(raw, zigzag32(r.z()));

    std::vector<uint8_t> lz = pmsf::pcatm::lzDeflate(raw, 9);

    std::vector<uint8_t> out;
    appendI32(out, static_cast<int32_t>(residualCounts.size()));
    for (int32_t count : residualCounts)
        appendI32(out, count);
    appendI32(out, totalResiduals);
    appendI32(out, totalResiduals * 3);
    appendI32(out, static_cast<int32_t>(raw.size()));
    appendI32(out, static_cast<int32_t>(lz.size()));
    out.insert(out.end(), lz.begin(), lz.end());
    return out;
}

std::vector<uint8_t> encodeResidualGlobalSplitAxisVarintZlib(const std::vector<std::deque<MCGAL::PointInt>>& residualRounds) {
    std::vector<MCGAL::PointInt> flat;
    std::vector<int32_t> residualCounts;
    residualCounts.reserve(residualRounds.size());

    int32_t totalResiduals = 0;
    for (const auto& residuals : residualRounds) {
        residualCounts.push_back(static_cast<int32_t>(residuals.size()));
        totalResiduals += static_cast<int32_t>(residuals.size());
        flat.insert(flat.end(), residuals.begin(), residuals.end());
    }

    std::vector<uint8_t> rawX;
    std::vector<uint8_t> rawY;
    std::vector<uint8_t> rawZ;
    rawX.reserve(flat.size());
    rawY.reserve(flat.size());
    rawZ.reserve(flat.size());
    for (const auto& r : flat)
        appendVarU32(rawX, zigzag32(r.x()));
    for (const auto& r : flat)
        appendVarU32(rawY, zigzag32(r.y()));
    for (const auto& r : flat)
        appendVarU32(rawZ, zigzag32(r.z()));

    std::vector<uint8_t> lzX = pmsf::pcatm::lzDeflate(rawX, 9);
    std::vector<uint8_t> lzY = pmsf::pcatm::lzDeflate(rawY, 9);
    std::vector<uint8_t> lzZ = pmsf::pcatm::lzDeflate(rawZ, 9);

    std::vector<uint8_t> out;
    appendI32(out, static_cast<int32_t>(residualCounts.size()));
    for (int32_t count : residualCounts)
        appendI32(out, count);
    appendI32(out, totalResiduals);
    appendI32(out, totalResiduals * 3);
    appendI32(out, static_cast<int32_t>(rawX.size()));
    appendI32(out, static_cast<int32_t>(lzX.size()));
    appendI32(out, static_cast<int32_t>(rawY.size()));
    appendI32(out, static_cast<int32_t>(lzY.size()));
    appendI32(out, static_cast<int32_t>(rawZ.size()));
    appendI32(out, static_cast<int32_t>(lzZ.size()));
    out.insert(out.end(), lzX.begin(), lzX.end());
    out.insert(out.end(), lzY.begin(), lzY.end());
    out.insert(out.end(), lzZ.begin(), lzZ.end());
    return out;
}

std::vector<uint8_t> encodeResidualGlobalDenseRange(const std::vector<std::deque<MCGAL::PointInt>>& residualRounds) {
    std::vector<uint32_t> coordSymbols;
    std::vector<int32_t> residualCounts;
    residualCounts.reserve(residualRounds.size());

    int32_t totalResiduals = 0;
    for (const auto& residuals : residualRounds) {
        residualCounts.push_back(static_cast<int32_t>(residuals.size()));
        totalResiduals += static_cast<int32_t>(residuals.size());
        for (const auto& r : residuals) {
            coordSymbols.push_back(zigzag32(r.x()));
            coordSymbols.push_back(zigzag32(r.y()));
            coordSymbols.push_back(zigzag32(r.z()));
        }
    }

    std::vector<uint32_t> codebook = coordSymbols;
    std::sort(codebook.begin(), codebook.end());
    codebook.erase(std::unique(codebook.begin(), codebook.end()), codebook.end());

    std::unordered_map<uint32_t, uint32_t> rankByValue;
    rankByValue.reserve(codebook.size() * 2 + 1);
    for (size_t i = 0; i < codebook.size(); ++i) {
        rankByValue.emplace(codebook[i], static_cast<uint32_t>(i));
    }

    std::vector<uint32_t> ranks;
    ranks.reserve(coordSymbols.size());
    for (uint32_t s : coordSymbols) {
        ranks.push_back(rankByValue.at(s));
    }

    pmsf::RangeCoderEntropyCoder coder(ceilLog2Size(std::max<size_t>(codebook.size(), 1)));
    std::vector<uint8_t> payload;
    coder.encode(ranks, payload);

    std::vector<uint8_t> out;
    appendI32(out, static_cast<int32_t>(residualCounts.size()));
    for (int32_t count : residualCounts)
        appendI32(out, count);
    appendI32(out, totalResiduals);
    appendI32(out, static_cast<int32_t>(coordSymbols.size()));
    appendI32(out, static_cast<int32_t>(codebook.size()));
    appendI32(out, static_cast<int32_t>(payload.size()));
    for (uint32_t value : codebook) {
        appendI32(out, static_cast<int32_t>(value));
    }
    out.insert(out.end(), payload.begin(), payload.end());
    return out;
}

std::vector<uint8_t> encodeResidualGlobalMeppAxisRange(const std::vector<std::deque<MCGAL::PointInt>>& residualRounds) {
    constexpr int32_t kMeppAxisWrapperMagic = -1101;
    constexpr int32_t kMeppAxisVariantRange = 0;
    constexpr int32_t kMeppAxisVariantVarintZlib = 1;

    std::vector<MCGAL::PointInt> flat;
    std::vector<int32_t> residualCounts;
    residualCounts.reserve(residualRounds.size());

    int32_t totalResiduals = 0;
    for (const auto& residuals : residualRounds) {
        residualCounts.push_back(static_cast<int32_t>(residuals.size()));
        totalResiduals += static_cast<int32_t>(residuals.size());
        flat.insert(flat.end(), residuals.begin(), residuals.end());
    }

    std::array<std::vector<int32_t>, 3> axisValues;
    axisValues[0].reserve(flat.size());
    axisValues[1].reserve(flat.size());
    axisValues[2].reserve(flat.size());
    for (const auto& r : flat) {
        axisValues[0].push_back(r.x());
        axisValues[1].push_back(r.y());
        axisValues[2].push_back(r.z());
    }

    struct AxisPayload {
        int32_t mode = 0;  // 0: MEPP-style min/range, 1: per-axis dense codebook.
        int32_t minValue = 0;
        int32_t alphabetSize = 1;
        int32_t bits = 1;
        std::vector<int32_t> codebook;
        std::vector<uint8_t> payload;
    };

    auto encodeDense = [](const std::vector<int32_t>& values) {
        AxisPayload axis;
        axis.mode = 1;
        axis.codebook = values;
        std::sort(axis.codebook.begin(), axis.codebook.end());
        axis.codebook.erase(std::unique(axis.codebook.begin(), axis.codebook.end()), axis.codebook.end());
        axis.alphabetSize = static_cast<int32_t>(std::max<size_t>(axis.codebook.size(), 1));
        axis.bits = ceilLog2Size(static_cast<size_t>(axis.alphabetSize));
        if (axis.bits > 16) {
            throw std::runtime_error("[PCATMStrictPaper] MEPP axis dense residual alphabet exceeds range coder limit");
        }

        std::unordered_map<int32_t, uint32_t> rankByValue;
        rankByValue.reserve(axis.codebook.size() * 2 + 1);
        for (size_t i = 0; i < axis.codebook.size(); ++i) {
            rankByValue.emplace(axis.codebook[i], static_cast<uint32_t>(i));
        }

        std::vector<uint32_t> ranks;
        ranks.reserve(values.size());
        for (int32_t v : values)
            ranks.push_back(rankByValue.at(v));

        pmsf::RangeCoderEntropyCoder coder(axis.bits);
        coder.encode(ranks, axis.payload);
        return axis;
    };

    auto encodeAxis = [&](const std::vector<int32_t>& values) {
        if (values.empty()) {
            return AxisPayload{};
        }

        const auto [minIt, maxIt] = std::minmax_element(values.begin(), values.end());
        const int32_t minValue = *minIt;
        const int32_t maxValue = *maxIt;
        const int64_t alphabet64 = static_cast<int64_t>(maxValue) - static_cast<int64_t>(minValue) + 1;

        AxisPayload direct;
        bool directOk = alphabet64 > 0 && alphabet64 <= (1 << 16);
        if (directOk) {
            direct.mode = 0;
            direct.minValue = minValue;
            direct.alphabetSize = static_cast<int32_t>(alphabet64);
            direct.bits = ceilLog2Size(static_cast<size_t>(direct.alphabetSize));

            std::vector<uint32_t> ranks;
            ranks.reserve(values.size());
            for (int32_t v : values) {
                ranks.push_back(static_cast<uint32_t>(static_cast<int64_t>(v) - static_cast<int64_t>(minValue)));
            }

            pmsf::RangeCoderEntropyCoder coder(direct.bits);
            coder.encode(ranks, direct.payload);
        }

        AxisPayload dense = encodeDense(values);
        const size_t directBytes = directOk ? direct.payload.size() : std::numeric_limits<size_t>::max();
        const size_t denseBytes = dense.payload.size() + dense.codebook.size() * sizeof(int32_t);
        return directBytes <= denseBytes ? direct : dense;
    };

    std::array<AxisPayload, 3> axes = {
        encodeAxis(axisValues[0]),
        encodeAxis(axisValues[1]),
        encodeAxis(axisValues[2]),
    };

    std::vector<uint8_t> axisOut;
    appendI32(axisOut, static_cast<int32_t>(residualCounts.size()));
    for (int32_t count : residualCounts)
        appendI32(axisOut, count);
    appendI32(axisOut, totalResiduals);
    appendI32(axisOut, totalResiduals * 3);
    appendI32(axisOut, 3);
    for (const auto& axis : axes) {
        appendI32(axisOut, axis.mode);
        appendI32(axisOut, axis.minValue);
        appendI32(axisOut, axis.alphabetSize);
        appendI32(axisOut, axis.bits);
        appendI32(axisOut, static_cast<int32_t>(axis.codebook.size()));
        appendI32(axisOut, static_cast<int32_t>(axis.payload.size()));
        for (int32_t v : axis.codebook)
            appendI32(axisOut, v);
        axisOut.insert(axisOut.end(), axis.payload.begin(), axis.payload.end());
    }

    std::vector<uint8_t> varintOut = encodeResidualGlobalVarintZlib(residualRounds);
    const bool useVarintFallback = varintOut.size() <= axisOut.size();
    const std::vector<uint8_t>& payload = useVarintFallback ? varintOut : axisOut;

    std::vector<uint8_t> out;
    appendI32(out, kMeppAxisWrapperMagic);
    appendI32(out, useVarintFallback ? kMeppAxisVariantVarintZlib : kMeppAxisVariantRange);
    appendI32(out, static_cast<int32_t>(payload.size()));
    out.insert(out.end(), payload.begin(), payload.end());
    return out;
}

struct Paper4PatchComponent {
    MCGAL::Facet* root = nullptr;
    std::vector<MCGAL::Facet*> faces;
    uint8_t color = 1;
};

bool isInternalPatchEdge(MCGAL::Halfedge* h, int paperLevel) {
    if (!h)
        return false;
    MCGAL::Halfedge* opp = h->opposite();
    return (h->isAdded() && h->paperLevel() == paperLevel) || (opp && opp->isAdded() && opp->paperLevel() == paperLevel);
}

bool faceHasVertex(MCGAL::Facet* f, MCGAL::Vertex* v) {
    if (!f || !v)
        return false;
    for (MCGAL::Vertex* fv : f->getVertices()) {
        if (fv == v)
            return true;
    }
    return false;
}

bool facesAreAdjacent(MCGAL::Facet* a, MCGAL::Facet* b) {
    if (!a || !b)
        return false;
    MCGAL::Halfedge* start = a->proxyHalfedge();
    if (!start)
        return false;
    MCGAL::Halfedge* h = start;
    do {
        MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
        if (opp && opp->face() == b)
            return true;
        h = h ? h->next() : nullptr;
    } while (h && h != start);
    return false;
}

std::vector<MCGAL::Facet*> stableQuantizedFaceOrder(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    struct FaceOrderRecord {
        MCGAL::Facet* face = nullptr;
        std::vector<MCGAL::PointInt> key;
    };
    auto makeKey = [&](MCGAL::Facet* f) {
        std::vector<MCGAL::PointInt> key;
        if (!f)
            return key;
        MCGAL::Halfedge* start = f->proxyHalfedge();
        if (!start)
            return key;
        key.reserve(static_cast<size_t>(std::max(f->facet_degree(), 0)));
        MCGAL::Halfedge* h = start;
        do {
            if (MCGAL::Vertex* v = h ? h->vertex() : nullptr) {
                key.push_back(mesh->floatPosToInt(v->point()));
            }
            h = h ? h->next() : nullptr;
        } while (h && h != start);
        std::sort(key.begin(), key.end());
        return key;
    };

    std::vector<FaceOrderRecord> records;
    if (!mesh)
        return {};
    records.reserve(mesh->size_of_facets());
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved())
            continue;
        records.push_back(FaceOrderRecord{f, makeKey(f)});
    }
    std::sort(records.begin(), records.end(), [](const FaceOrderRecord& a, const FaceOrderRecord& b) {
        if (a.key.size() != b.key.size())
            return a.key.size() < b.key.size();
        return std::lexicographical_compare(a.key.begin(), a.key.end(), b.key.begin(), b.key.end());
    });
    std::vector<MCGAL::Facet*> faces;
    faces.reserve(records.size());
    for (const FaceOrderRecord& record : records)
        faces.push_back(record.face);
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

std::vector<MCGAL::Facet*> mortonFaceOrder(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    struct FaceOrderRecord {
        MCGAL::Facet* face = nullptr;
        uint64_t morton = 0;
        std::vector<MCGAL::PointInt> tie;
    };
    auto makeRecord = [&](MCGAL::Facet* f) {
        int64_t sx = 0, sy = 0, sz = 0;
        int n = 0;
        std::vector<MCGAL::PointInt> tie;
        if (f)
            tie.reserve(static_cast<size_t>(std::max(f->facet_degree(), 0)));
        if (f) {
            MCGAL::Halfedge* start = f->proxyHalfedge();
            MCGAL::Halfedge* h = start;
            while (h) {
                if (MCGAL::Vertex* v = h->vertex()) {
                    MCGAL::PointInt q = mesh->floatPosToInt(v->point());
                    sx += q.x();
                    sy += q.y();
                    sz += q.z();
                    tie.push_back(q);
                    ++n;
                }
                h = h->next();
                if (h == start)
                    break;
            }
        }
        std::sort(tie.begin(), tie.end());
        auto clamp12 = [](int64_t v) -> uint32_t {
            if (v < 0)
                return 0u;
            if (v > 4095)
                return 4095u;
            return static_cast<uint32_t>(v);
        };
        const uint64_t morton = (n <= 0) ? uint64_t(0) : morton3D12(clamp12(sx / n), clamp12(sy / n), clamp12(sz / n));
        return FaceOrderRecord{f, morton, std::move(tie)};
    };
    std::vector<FaceOrderRecord> records;
    if (!mesh)
        return {};
    records.reserve(mesh->size_of_facets());
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved())
            continue;
        records.push_back(makeRecord(f));
    }
    std::sort(records.begin(), records.end(), [](const FaceOrderRecord& a, const FaceOrderRecord& b) {
        if (a.morton != b.morton)
            return a.morton < b.morton;
        return std::lexicographical_compare(a.tie.begin(), a.tie.end(), b.tie.begin(), b.tie.end());
    });
    std::vector<MCGAL::Facet*> faces;
    faces.reserve(records.size());
    for (const FaceOrderRecord& record : records)
        faces.push_back(record.face);
    return faces;
}

bool patchFacesFormTree(const std::vector<MCGAL::Facet*>& faces) {
    if (faces.empty())
        return false;
    std::unordered_set<MCGAL::Facet*> faceSet(faces.begin(), faces.end());
    std::unordered_set<MCGAL::Facet*> visited;
    std::queue<MCGAL::Facet*> q;
    visited.insert(faces.front());
    q.push(faces.front());
    int edgeCountTwice = 0;
    while (!q.empty()) {
        MCGAL::Facet* cur = q.front();
        q.pop();
        MCGAL::Halfedge* start = cur ? cur->proxyHalfedge() : nullptr;
        if (!start)
            continue;
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
    return visited.size() == faceSet.size() && edgeCountTwice / 2 == static_cast<int>(faceSet.size()) - 1;
}

bool patchFacesFormZPath(const std::vector<MCGAL::Facet*>& faces,
                         MCGAL::Facet* zeroA,
                         MCGAL::Facet* zeroB);

// WS-B task-18: combinatorial (legacy) recovery of a 2-color patch. Given the
// blue set and the full color map, the decoder enumerates all adjacent zero-face
// pairs and requires exactly one pair that completes the blue run into a Z-path
// (patchFacesFormZPath). Returns true iff that unique pair is exactly
// {expectedA, expectedB}.
bool legacyZerosUnique(const std::vector<MCGAL::Facet*>& ones,
                       MCGAL::Facet* expectedA,
                       MCGAL::Facet* expectedB,
                       const std::unordered_map<MCGAL::Facet*, uint8_t>& colors) {
    if (ones.empty()) return false;
    std::unordered_set<MCGAL::Facet*> oneSet(ones.begin(), ones.end());
    std::vector<MCGAL::Facet*> zeros;
    std::unordered_set<MCGAL::Facet*> seen;
    for (MCGAL::Facet* f : ones) {
        if (!f) continue;
        for (MCGAL::Halfedge* hh : f->getHalfedges()) {
            if (!hh || hh->isRemoved()) continue;
            MCGAL::Halfedge* opp = hh->opposite();
            if (!opp || opp->isRemoved() || !opp->face()) continue;
            MCGAL::Facet* nf = opp->face();
            if (oneSet.count(nf) || seen.count(nf)) continue;
            auto cit = colors.find(nf);
            if (cit == colors.end() || cit->second != 0u) continue;
            seen.insert(nf);
            zeros.push_back(nf);
        }
    }
    int matches = 0;
    MCGAL::Facet* foundA = nullptr;
    MCGAL::Facet* foundB = nullptr;
    for (size_t a = 0; a < zeros.size(); ++a) {
        for (size_t b = a + 1; b < zeros.size(); ++b) {
            std::vector<MCGAL::Facet*> trial = ones;
            trial.push_back(zeros[a]);
            trial.push_back(zeros[b]);
            std::unordered_set<MCGAL::Facet*> dedup(trial.begin(), trial.end());
            if (dedup.size() != trial.size()) continue;
            if (!patchFacesFormZPath(trial, zeros[a], zeros[b])) continue;
            ++matches;
            foundA = zeros[a];
            foundB = zeros[b];
        }
    }
    if (matches != 1) return false;
    return (foundA == expectedA && foundB == expectedB) ||
           (foundA == expectedB && foundB == expectedA);
}

bool patchFacesFormZPath(const std::vector<MCGAL::Facet*>& faces, MCGAL::Facet* zeroA, MCGAL::Facet* zeroB) {
    if (faces.size() < 3 || !zeroA || !zeroB || zeroA == zeroB)
        return false;
    std::unordered_set<MCGAL::Facet*> faceSet(faces.begin(), faces.end());
    if (faceSet.size() != faces.size() || !faceSet.count(zeroA) || !faceSet.count(zeroB)) {
        return false;
    }
    std::unordered_set<MCGAL::Facet*> visited;
    std::unordered_map<MCGAL::Facet*, int> degree;
    std::queue<MCGAL::Facet*> q;
    visited.insert(faces.front());
    q.push(faces.front());
    int edgeCountTwice = 0;
    while (!q.empty()) {
        MCGAL::Facet* cur = q.front();
        q.pop();
        MCGAL::Halfedge* start = cur ? cur->proxyHalfedge() : nullptr;
        if (!start)
            continue;
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
    if (visited.size() != faceSet.size() || edgeCountTwice / 2 != static_cast<int>(faceSet.size()) - 1) {
        return false;
    }
    int leaves = 0;
    for (MCGAL::Facet* f : faces) {
        const int d = degree[f];
        if (d == 1)
            ++leaves;
        else if (d != 2)
            return false;
    }
    return leaves == 2 && degree[zeroA] == 1 && degree[zeroB] == 1;
}

bool facesShareEdge(MCGAL::Facet* a, MCGAL::Facet* b) {
    if (!a || !b)
        return false;
    MCGAL::Halfedge* start = a->proxyHalfedge();
    if (!start)
        return false;
    MCGAL::Halfedge* h = start;
    do {
        MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
        if (opp && opp->face() == b)
            return true;
        h = h ? h->next() : nullptr;
    } while (h && h != start);
    return false;
}

bool zerosAttachToBlueRunEndpoints(const std::vector<MCGAL::Facet*>& ones, MCGAL::Facet* zeroA, MCGAL::Facet* zeroB) {
    if (ones.empty() || !zeroA || !zeroB || zeroA == zeroB)
        return false;
    std::unordered_set<MCGAL::Facet*> oneSet(ones.begin(), ones.end());
    std::vector<MCGAL::Facet*> endpoints;
    for (MCGAL::Facet* f : ones) {
        int deg = 0;
        MCGAL::Halfedge* start = f ? f->proxyHalfedge() : nullptr;
        if (!start)
            continue;
        MCGAL::Halfedge* h = start;
        do {
            MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
            MCGAL::Facet* nf = opp ? opp->face() : nullptr;
            if (oneSet.count(nf))
                ++deg;
            h = h ? h->next() : nullptr;
        } while (h && h != start);
        if (deg <= 1)
            endpoints.push_back(f);
    }
    if (ones.size() == 1) {
        return facesShareEdge(zeroA, ones.front()) && facesShareEdge(zeroB, ones.front());
    }
    if (endpoints.size() != 2)
        return false;
    return (facesShareEdge(zeroA, endpoints[0]) && facesShareEdge(zeroB, endpoints[1])) ||
           (facesShareEdge(zeroA, endpoints[1]) && facesShareEdge(zeroB, endpoints[0]));
}

std::vector<MCGAL::Facet*> adjacentFacesWithColor(const std::vector<MCGAL::Facet*>& faces,
                                                  const std::unordered_map<MCGAL::Facet*, uint8_t>& colorByFace,
                                                  uint8_t color,
                                                  MCGAL::Vertex* requiredVertex) {
    std::vector<MCGAL::Facet*> out;
    std::unordered_set<MCGAL::Facet*> seen;
    for (MCGAL::Facet* f : faces) {
        MCGAL::Halfedge* start = f ? f->proxyHalfedge() : nullptr;
        if (!start)
            continue;
        MCGAL::Halfedge* h = start;
        do {
            MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
            MCGAL::Facet* nf = opp ? opp->face() : nullptr;
            auto it = colorByFace.find(nf);
            if (it != colorByFace.end() && it->second == color && (!requiredVertex || faceHasVertex(nf, requiredVertex)) && seen.insert(nf).second) {
                out.push_back(nf);
            }
            h = h ? h->next() : nullptr;
        } while (h && h != start);
    }
    return out;
}

struct Mode2RecoverabilityStats {
    int expectedPatches = 0;
    int oneComponents = 0;
    int zeroMismatch = 0;
    int treeMismatch = 0;
};

bool isMode2RecoverableRound(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Halfedge* seed, Mode2RecoverabilityStats* stats = nullptr) {
    if (!mesh || !seed)
        return false;
    std::queue<int> q;
    q.push(seed->poolId());
    int currentVersion = MCGAL::BFSVersionManager::current_version++;
    std::vector<MCGAL::Facet*> facesInOrder;
    std::unordered_map<MCGAL::Facet*, uint8_t> colorByFace;
    int expectedPatches = 0;

    while (!q.empty()) {
        int hid = q.front();
        q.pop();
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndexInSubPool(mesh->meshId(), hid);
        if (!h || h->isVisited(currentVersion) || h->isRemoved())
            continue;
        h->setVisited(currentVersion);
        MCGAL::Facet* f = h->face();
        if (f && !f->isVisited(currentVersion)) {
            f->setVisited(currentVersion);
            uint8_t c = (f->getTriangleColor() == MCGAL::Facet::BLUE) ? 1u : 0u;
            facesInOrder.push_back(f);
            colorByFace[f] = c;
            if (f->isSplittable())
                ++expectedPatches;
        }
        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp && !hOpp->isVisited(currentVersion))
                q.push(hOpp->poolId());
            if (hIt && !hIt->isVisited(currentVersion))
                q.push(hIt->poolId());
            hIt = hIt->next();
        } while (hIt != h && hIt != nullptr);
    }

    std::unordered_set<MCGAL::Facet*> consumed;
    int oneComponents = 0;
    for (MCGAL::Facet* f : facesInOrder) {
        auto it = colorByFace.find(f);
        if (it == colorByFace.end() || it->second != 1u || consumed.count(f))
            continue;
        ++oneComponents;
        std::vector<MCGAL::Facet*> ones;
        std::queue<MCGAL::Facet*> cq;
        consumed.insert(f);
        cq.push(f);
        while (!cq.empty()) {
            MCGAL::Facet* cur = cq.front();
            cq.pop();
            ones.push_back(cur);
            MCGAL::Halfedge* start = cur ? cur->proxyHalfedge() : nullptr;
            if (!start)
                continue;
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

        auto zeros = adjacentFacesWithColor(ones, colorByFace, 0u, nullptr);
        if (zeros.size() < 2 && stats)
            ++stats->zeroMismatch;
        bool ok = false;
        int validPairs = 0;
        for (size_t za = 0; za < zeros.size(); ++za) {
            for (size_t zb = za + 1; zb < zeros.size(); ++zb) {
                std::vector<MCGAL::Facet*> trial = ones;
                trial.push_back(zeros[za]);
                trial.push_back(zeros[zb]);
                std::unordered_set<MCGAL::Facet*> dedup(trial.begin(), trial.end());
                if (dedup.size() != trial.size())
                    continue;
                if (!patchFacesFormZPath(trial, zeros[za], zeros[zb]))
                    continue;
                ++validPairs;
                ok = true;
            }
        }
        if (validPairs != 1)
            ok = false;
        if (!ok) {
            if (stats) {
                if (zeros.size() >= 2)
                    ++stats->treeMismatch;
                stats->expectedPatches = expectedPatches;
                stats->oneComponents = oneComponents;
            }
            return false;
        }
    }
    if (stats) {
        stats->expectedPatches = expectedPatches;
        stats->oneComponents = oneComponents;
    }
    return oneComponents == expectedPatches;
}


bool decoderSelectionMatchesEncoderComponents(
    const std::vector<MCGAL::Facet*>& facesInOrder,
    const std::unordered_map<MCGAL::Facet*, uint8_t>& colorByFace,
    const std::unordered_map<MCGAL::Facet*, int>& compByFace,
    const MCGAL::Point& bboxMin,
    float quantStep,
    int expectedPatches) {
    std::unordered_set<MCGAL::Facet*> consumed;
    std::unordered_set<MCGAL::Facet*> selectedZeros;
    std::unordered_map<int, size_t> componentSizes;
    for (const auto& item : compByFace) ++componentSizes[item.second];
    std::unordered_set<int> recoveredComponents;
    int recovered = 0;
    for (MCGAL::Facet* f : facesInOrder) {
        auto it = colorByFace.find(f);
        if (it == colorByFace.end() || it->second != 1u || consumed.count(f))
            continue;

        std::vector<MCGAL::Facet*> ones;
        std::queue<MCGAL::Facet*> q;
        consumed.insert(f);
        q.push(f);
        while (!q.empty()) {
            MCGAL::Facet* cur = q.front();
            q.pop();
            ones.push_back(cur);
            MCGAL::Halfedge* start = cur ? cur->proxyHalfedge() : nullptr;
            if (!start)
                continue;
            MCGAL::Halfedge* h = start;
            do {
                MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
                MCGAL::Facet* nf = opp ? opp->face() : nullptr;
                auto nit = colorByFace.find(nf);
                if (nit != colorByFace.end() && nit->second == 1u && !consumed.count(nf)) {
                    consumed.insert(nf);
                    q.push(nf);
                }
                h = h ? h->next() : nullptr;
            } while (h && h != start);
        }

        std::array<MCGAL::Facet*, 2> zeros{{nullptr, nullptr}};
        if (isEnvOn("PMSF_PCATM_STRICT_2COLOR_LEGACY_MATCH")) {
            // Mirror the decoder's legacy combinatorial recovery exactly.
            std::vector<MCGAL::Facet*> zerosVec;
            std::unordered_set<MCGAL::Facet*> seenZero;
            for (MCGAL::Facet* f : ones) {
                for (MCGAL::Halfedge* hh : f->getHalfedges()) {
                    if (!hh || hh->isRemoved()) continue;
                    MCGAL::Halfedge* opp = hh->opposite();
                    if (!opp || opp->isRemoved() || !opp->face()) continue;
                    MCGAL::Facet* nf = opp->face();
                    auto nit = colorByFace.find(nf);
                    if (nit == colorByFace.end() || nit->second != 0u) continue;
                    if (seenZero.insert(nf).second) zerosVec.push_back(nf);
                }
            }
            int matches = 0;
            for (size_t a = 0; a < zerosVec.size(); ++a) {
                for (size_t b = a + 1; b < zerosVec.size(); ++b) {
                    std::vector<MCGAL::Facet*> trial = ones;
                    trial.push_back(zerosVec[a]);
                    trial.push_back(zerosVec[b]);
                    std::unordered_set<MCGAL::Facet*> dedup(trial.begin(), trial.end());
                    if (dedup.size() != trial.size()) continue;
                    if (!patchFacesFormZPath(trial, zerosVec[a], zerosVec[b])) continue;
                    ++matches;
                    zeros = {{zerosVec[a], zerosVec[b]}};
                }
            }
            if (matches != 1) zeros = {{nullptr, nullptr}};
        } else {
            zeros = pmsf::pcatm::paperMode2Zeros(
                ones, colorByFace, bboxMin, quantStep);
        }
        if (zeros[0] == nullptr || zeros[1] == nullptr ||
            selectedZeros.count(zeros[0]) != 0 ||
            selectedZeros.count(zeros[1]) != 0) {
            if (const char* env = std::getenv("PMSF_BPV_DIAG");
                env && env[0] == '1') {
                std::cerr << "[PCATMStrictPaper mode2-shape-mismatch]"
                          << " ones=" << ones.size()
                          << " reason=shared_geometry_matcher"
                          << std::endl;
            }
            return false;
        }
        int comp = -1;
        for (MCGAL::Facet* pf : ones) {
            const auto cit = compByFace.find(pf);
            if (cit == compByFace.end()) return false;
            if (comp < 0) {
                comp = cit->second;
            } else if (comp != cit->second) {
                return false;
            }
        }
        if (comp < 0) return false;
        for (MCGAL::Facet* zero : zeros) {
            const auto cit = compByFace.find(zero);
            if (cit == compByFace.end() || cit->second != comp) return false;
        }
        if (componentSizes[comp] != ones.size() + 2 ||
            !recoveredComponents.insert(comp).second) return false;
        selectedZeros.insert(zeros[0]);
        selectedZeros.insert(zeros[1]);
        ++recovered;
    }
    return recovered == expectedPatches;
}

// PMSF_BPV_DIAG only: quantized face signature, used to diff encoder patch
// components against the decoder's color-recovered components.
inline std::string strictEncFaceSignature(MCGAL::Mesh* mesh, MCGAL::Facet* f) {
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

void buildPaper4Round(std::shared_ptr<MCGAL::Mesh> mesh,
                      MCGAL::Halfedge* seed,
                      std::vector<uint8_t>& symbols,
                      std::vector<uint8_t>& mode2Symbols,
                      bool& mode2Recoverable,
                      std::deque<MCGAL::PointInt>& residuals,
                      std::deque<MCGAL::PointInt>& mode2Residuals,
                      std::vector<uint8_t>& mode3Symbols,
                      bool& mode3Recoverable,
                      std::deque<MCGAL::PointInt>& mode3Residuals,
                      std::vector<uint8_t>& mode2Side,
                      int encoderRoundIndex,
                      std::vector<uint8_t>* blueLessColorOut = nullptr) {
    symbols.clear();
    mode2Symbols.clear();
    mode2Side.clear();
    mode2Recoverable = false;
    residuals.clear();
    mode2Residuals.clear();
    mode3Symbols.clear();
    mode3Recoverable = false;
    mode3Residuals.clear();
    if (blueLessColorOut) blueLessColorOut->clear();
    if (!mesh || !seed)
        return;
    const int paperLevel = MCGAL::contextPool.levelFast();

    std::vector<Paper4PatchComponent> comps;
    std::unordered_map<MCGAL::Facet*, int> compByFace;

    for (MCGAL::Facet* root : mesh->faces()) {
        if (!root || root->isRemoved() || !root->isSplittable() || compByFace.find(root) != compByFace.end()) {
            continue;
        }
        const int compIdx = static_cast<int>(comps.size());
        comps.push_back(Paper4PatchComponent{});
        comps.back().root = root;

        std::queue<MCGAL::Facet*> q;
        q.push(root);
        compByFace[root] = compIdx;
        while (!q.empty()) {
            MCGAL::Facet* f = q.front();
            q.pop();
            comps.back().faces.push_back(f);
            MCGAL::Halfedge* start = f->proxyHalfedge();
            if (!start)
                continue;
            MCGAL::Halfedge* h = start;
            do {
                MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
                MCGAL::Facet* nf = opp ? opp->face() : nullptr;
                if (nf && !nf->isRemoved() && compByFace.find(nf) == compByFace.end() && isInternalPatchEdge(h, paperLevel)) {
                    compByFace[nf] = compIdx;
                    q.push(nf);
                }
                h = h ? h->next() : nullptr;
            } while (h && h != start);
        }
    }

    std::vector<std::unordered_set<int>> adj(comps.size());
    for (int i = 0; i < static_cast<int>(comps.size()); ++i) {
        for (MCGAL::Facet* f : comps[static_cast<size_t>(i)].faces) {
            MCGAL::Halfedge* start = f ? f->proxyHalfedge() : nullptr;
            if (!start)
                continue;
            MCGAL::Halfedge* h = start;
            do {
                MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
                MCGAL::Facet* nf = opp ? opp->face() : nullptr;
                auto it = compByFace.find(nf);
                if (it != compByFace.end() && it->second != i) {
                    adj[static_cast<size_t>(i)].insert(it->second);
                }
                h = h ? h->next() : nullptr;
            } while (h && h != start);
        }
    }

    std::vector<uint8_t> assigned(comps.size(), 0u);
    for (int step = 0; step < static_cast<int>(comps.size()); ++step) {
        int best = -1;
        int bestSat = -1;
        int bestDegree = -1;
        for (int i = 0; i < static_cast<int>(comps.size()); ++i) {
            if (assigned[static_cast<size_t>(i)])
                continue;
            bool seen[4] = {false, false, false, false};
            for (int nb : adj[static_cast<size_t>(i)]) {
                uint8_t c = comps[static_cast<size_t>(nb)].color;
                if (assigned[static_cast<size_t>(nb)] && c <= 3)
                    seen[c] = true;
            }
            int sat = 0;
            for (int c = 1; c <= 3; ++c) {
                if (seen[c])
                    ++sat;
            }
            const int degree = static_cast<int>(adj[static_cast<size_t>(i)].size());
            if (sat > bestSat || (sat == bestSat && degree > bestDegree)) {
                best = i;
                bestSat = sat;
                bestDegree = degree;
            }
        }
        if (best < 0)
            break;

        int costByColor[4] = {0, 0, 0, 0};
        for (int nb : adj[static_cast<size_t>(best)]) {
            if (assigned[static_cast<size_t>(nb)]) {
                ++costByColor[comps[static_cast<size_t>(nb)].color];
            }
        }
        uint8_t chosen = 1;
        for (uint8_t c = 2; c <= 3; ++c) {
            if (costByColor[c] < costByColor[chosen])
                chosen = c;
        }
        comps[static_cast<size_t>(best)].color = chosen;
        assigned[static_cast<size_t>(best)] = 1u;
    }

    auto conflictCountFor = [&](int idx, uint8_t color) {
        int cnt = 0;
        for (int nb : adj[static_cast<size_t>(idx)]) {
            if (comps[static_cast<size_t>(nb)].color == color)
                ++cnt;
        }
        return cnt;
    };
    auto totalConflicts = [&]() {
        int cnt = 0;
        for (int i = 0; i < static_cast<int>(comps.size()); ++i) {
            for (int nb : adj[static_cast<size_t>(i)]) {
                if (i < nb && comps[static_cast<size_t>(i)].color == comps[static_cast<size_t>(nb)].color) {
                    ++cnt;
                }
            }
        }
        return cnt;
    };

    int unresolvedConflicts = totalConflicts();
    for (int pass = 0; pass < 200 && unresolvedConflicts > 0; ++pass) {
        bool changed = false;
        for (int i = 0; i < static_cast<int>(comps.size()); ++i) {
            const int current = conflictCountFor(i, comps[static_cast<size_t>(i)].color);
            if (current == 0)
                continue;
            uint8_t bestColor = comps[static_cast<size_t>(i)].color;
            int bestCost = current;
            for (uint8_t c = 1; c <= 3; ++c) {
                const int cost = conflictCountFor(i, c);
                if (cost < bestCost) {
                    bestCost = cost;
                    bestColor = c;
                }
            }
            if (bestColor != comps[static_cast<size_t>(i)].color) {
                comps[static_cast<size_t>(i)].color = bestColor;
                changed = true;
            }
        }
        const int nextConflicts = totalConflicts();
        if (!changed || nextConflicts >= unresolvedConflicts)
            break;
        unresolvedConflicts = nextConflicts;
    }
    if (unresolvedConflicts > 0) {
        std::cerr << "[PCATMStrictPaper] WARN paper4 unresolved same-color adjacencies=" << unresolvedConflicts
                  << " (paper would skip one endpoint of these removals)" << std::endl;
    }

    for (const auto& comp : comps) {
        const uint8_t patchTag = static_cast<uint8_t>(comp.color & 0x3u);
        for (MCGAL::Facet* f : comp.faces) {
            if (f && !f->isRemoved())
                f->setPCATMPatchTag(patchTag);
        }
    }

    std::unordered_map<MCGAL::Facet*, uint8_t> mode2ByFace;
    bool validMode2 = !comps.empty();
    int mode2InvalidSmall = 0;
    int mode2InvalidDegree = 0;
    int mode2InvalidShape = 0;
    int mode2InvalidCross = 0;
    // WS-B task-18: explicit side-info disambiguation of the mode-2 external
    // zero pair (2-bit candidate index per patch) instead of a geometric
    // matcher. Only active under PMSF_PCATM_STRICT_2COLOR_SIDE=1.
    const bool sideInfoMode = isEnvOn("PMSF_PCATM_STRICT_2COLOR_SIDE");
    std::unordered_map<int, std::array<MCGAL::Facet*, 2>> compChosenZeros;
    std::unordered_map<int, int> mode2SideIndexByComp;
    // v7 (task-36): direct zero-index side stream. Per blue component the
    // encoder stores the indices of the two chosen zero faces inside the
    // topology-only candidate list, plus the candidate count / bit width used
    // for the per-round patchCount/candSum checksum. No pair-index table and no
    // assignment repair: the decoder maps the two indices back to faces and
    // validates the patch with its existing tree check.
    std::unordered_map<int, std::array<int, 2>> compZeroIdx;
    std::unordered_map<int, int> compZeroCandCount;
    std::unordered_map<int, int> compZeroBits;
    const bool constructive2Color =
        isEnvOn("PMSF_PCATM_STRICT_CONSTRUCTIVE_2COLOR") || sideInfoMode;
    if (!constructive2Color) {
        for (int i = 0; i < static_cast<int>(comps.size()); ++i) {
            const auto& comp = comps[static_cast<size_t>(i)];
            for (MCGAL::Facet* f : comp.faces) {
                mode2ByFace[f] =
                    (f && f->getTriangleColor() == MCGAL::Facet::BLUE) ? 1u : 0u;
            }
        }
    } else if (sideInfoMode) {
        // WS-B task-18: side-info 2-color assignment with conflict resolution.
        // Every patch needs >= 3 faces (valence >= 5) so it has a middle run and
        // two external zero triangles; the induced patch graph must be a path.
        // Patches stay independently recoverable only if no two patches have
        // edge-adjacent blues. The rare conflict (one per level in practice) is
        // resolved by reassigning the already-assigned neighbour with an
        // alternative zero pair that does not touch the failing patch's run.
        validMode2 = false;
        const int numComps = static_cast<int>(comps.size());
        int firstFailComp = -1;
        int firstFailK = -1;
        std::vector<std::unordered_map<MCGAL::Facet*,
                                      std::vector<MCGAL::Facet*>>>
            adjAll(static_cast<size_t>(std::max(numComps, 0)));
        for (int ci = 0; ci < numComps; ++ci) {
            const auto& faces = comps[static_cast<size_t>(ci)].faces;
            for (MCGAL::Facet* f : faces) {
                for (MCGAL::Halfedge* hh : f->getHalfedges()) {
                    if (!hh || hh->isRemoved()) continue;
                    MCGAL::Halfedge* opp = hh->opposite();
                    if (!opp || opp->isRemoved() || !opp->face()) continue;
                    MCGAL::Facet* nf = opp->face();
                    auto cit = compByFace.find(nf);
                    if (cit == compByFace.end() || cit->second != ci || nf == f)
                        continue;
                    auto& list = adjAll[static_cast<size_t>(ci)][f];
                    if (std::find(list.begin(), list.end(), nf) == list.end())
                        list.push_back(nf);
                }
            }
        }
        struct SideAssign {
            std::vector<MCGAL::Facet*> ones;
            MCGAL::Facet* z1 = nullptr;
            MCGAL::Facet* z2 = nullptr;
        };
        auto adjacentToOnes = [&](MCGAL::Facet* z,
                                  const std::unordered_set<MCGAL::Facet*>& oneSet) {
            for (MCGAL::Halfedge* hh : z->getHalfedges()) {
                MCGAL::Halfedge* opp = hh ? hh->opposite() : nullptr;
                if (opp && opp->face() && oneSet.count(opp->face())) return true;
            }
            return false;
        };
        auto structuralCandidatesFor = [&](int ci) {
            std::vector<SideAssign> out;
            const auto& faces = comps[static_cast<size_t>(ci)].faces;
            const int k = static_cast<int>(faces.size());
            if (k < 3) return out;
            const auto& adj = adjAll[static_cast<size_t>(ci)];
            for (int ai = 0; ai < k; ++ai) {
                for (int bi = ai + 1; bi < k; ++bi) {
                    MCGAL::Facet* z1 = faces[static_cast<size_t>(ai)];
                    MCGAL::Facet* z2 = faces[static_cast<size_t>(bi)];
                    std::vector<MCGAL::Facet*> ones;
                    for (int c2 = 0; c2 < k; ++c2) {
                        if (c2 == ai || c2 == bi) continue;
                        ones.push_back(faces[static_cast<size_t>(c2)]);
                    }
                    std::unordered_set<MCGAL::Facet*> oneSet(ones.begin(), ones.end());
                    std::unordered_set<MCGAL::Facet*> seen;
                    std::queue<MCGAL::Facet*> q;
                    q.push(ones.front());
                    seen.insert(ones.front());
                    while (!q.empty()) {
                        MCGAL::Facet* cur = q.front();
                        q.pop();
                        auto ait = adj.find(cur);
                        if (ait == adj.end()) continue;
                        for (MCGAL::Facet* nb : ait->second) {
                            if (oneSet.count(nb) && seen.insert(nb).second) q.push(nb);
                        }
                    }
                    if (seen.size() != oneSet.size()) continue;
                    if (!adjacentToOnes(z1, oneSet) || !adjacentToOnes(z2, oneSet))
                        continue;
                    out.push_back(SideAssign{ones, z1, z2});
                }
            }
            return out;
        };
        std::vector<std::vector<SideAssign>> sideCands(static_cast<size_t>(std::max(numComps, 0)));
        std::unordered_map<int, SideAssign> recs;
        bool assignedAll = !comps.empty();
        for (int ci = 0; ci < numComps; ++ci) {
            sideCands[static_cast<size_t>(ci)] = structuralCandidatesFor(ci);
            if (sideCands[static_cast<size_t>(ci)].empty()) {
                assignedAll = false;
                if (firstFailComp < 0) {
                    firstFailComp = ci;
                    firstFailK = static_cast<int>(
                        comps[static_cast<size_t>(ci)].faces.size());
                }
                break;
            }
            recs[ci] = sideCands[static_cast<size_t>(ci)][0];
        }
        validMode2 = assignedAll;
        // v7 (task-36): patch-exact side stream = two zero-candidate indices.
        // Candidate list = topology-only faces adjacent to the blue run, in the
        // round's insertion order; the decoder enumerates the same list from the
        // recovered blue run + facesInOrder, so no colour map / candidate-table
        // synchronisation (the v4-v6 failure mode) is involved.
        // Candidate allowed-set = ALL live faces of the current mesh, which is
        // exactly what the decoder sees as `colorByFace` for this round (the
        // decoder enumerates the whole mesh, not only the patch faces). Candidate
        // order is the quantized centroid key, bit-identical on both sides.
        std::unordered_set<MCGAL::Facet*> roundFaces;
        roundFaces.reserve(mesh->size_of_facets() * 2);
        for (MCGAL::Facet* f : mesh->faces()) {
            if (f && !f->isRemoved()) roundFaces.insert(f);
        }
        long long v7CandSum = 0;
        if (validMode2) {
            for (int ci = 0; ci < numComps; ++ci) {
                bool keysUnique = true;
                const auto cands = pmsf::pcatm::enumerateMode2ZeroCandidates(
                    recs[ci].ones, roundFaces, mesh, keysUnique);
                const int n = static_cast<int>(cands.size());
                if (!keysUnique) {
                    validMode2 = false;
                    if (firstFailComp < 0) firstFailComp = ci;
                    if (firstFailK < 0)
                        firstFailK = static_cast<int>(
                            comps[static_cast<size_t>(ci)].faces.size());
                    break;
                }
                const int i0 = pmsf::pcatm::mode2FindZeroCandidateIndex(
                    cands, recs[ci].z1);
                const int i1 = pmsf::pcatm::mode2FindZeroCandidateIndex(
                    cands, recs[ci].z2);
                if (n < 2 || n > 64 || i0 < 0 || i1 < 0 || i0 == i1) {
                    if (isEnvOn("PMSF_PCATM_MODE2_DIAG")) {
                        std::cerr << "[2c-v7-fail] level=" << paperLevel
                                  << " comp=" << ci << " n=" << n
                                  << " i0=" << i0 << " i1=" << i1 << std::endl;
                    }
                    validMode2 = false;
                    if (firstFailComp < 0) firstFailComp = ci;
                    if (firstFailK < 0)
                        firstFailK = static_cast<int>(
                            comps[static_cast<size_t>(ci)].faces.size());
                    break;
                }
                int bits = 0;
                while ((1 << bits) < n) ++bits;
                compZeroIdx[ci] = {{std::min(i0, i1), std::max(i0, i1)}};
                compZeroCandCount[ci] = n;
                compZeroBits[ci] = bits;
                v7CandSum += static_cast<long long>(n);
            }
        }
        if (validMode2) {
            for (int ci = 0; ci < numComps; ++ci) {
                for (MCGAL::Facet* f : comps[static_cast<size_t>(ci)].faces)
                    mode2ByFace[f] = 0u;
                for (MCGAL::Facet* f : recs[ci].ones) mode2ByFace[f] = 1u;
                compChosenZeros[ci] = {{recs[ci].z1, recs[ci].z2}};
            }
        }
        if (!validMode2) {
            mode2ByFace.clear();
            compZeroIdx.clear();
            compZeroCandCount.clear();
            compZeroBits.clear();
        }
        if (firstFailComp >= 0 && firstFailK < 0) firstFailK = 0;
        if (isEnvOn("PMSF_PCATM_MODE2_DIAG")) {
            std::cerr << "[2c-v7] level=" << paperLevel
                      << " comps=" << numComps
                      << " assigned=" << (validMode2 ? 1 : 0)
                      << " recs=" << recs.size()
                      << " candSum=" << v7CandSum
                      << " failComp=" << firstFailComp
                      << " failK=" << firstFailK << std::endl;
        }
    } else {
        // WS-B task-18: construct the paper §4 2-color Z-shape assignment such
        // that the decoder's own geometric matcher (paperMode2Zeros) recovers
        // exactly this patch. Valence must be >= 5 (>= 3 patch triangles): the
        // two external triangles are 0, the connected middle run is 1, and no
        // two patches may have edge-adjacent 1s (paper's alternating rule).
        // The map used for validation is the full live-face map (all 0 except
        // already-committed blues), i.e. exactly the decoder's colorByFace.
        validMode2 = false;
        std::unordered_map<MCGAL::Facet*, uint8_t> colorMap;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (f && !f->isRemoved()) colorMap[f] = 0u;
        }
        std::unordered_set<MCGAL::Facet*> usedBlues;
        const char* forceProbeEnv3 =
            std::getenv("PMSF_PCATM_STRICT_FORCE_MODE2_LEVELS");
        const bool forceProbeLevel3 =
            forceProbeEnv3 && *forceProbeEnv3 &&
            paperLevel < std::atoi(forceProbeEnv3);
        bool allAssigned = !comps.empty();
        // Upper-bound probe fallback: first structurally valid (but perhaps not
        // matcher-valid) blue assignment per component, preserving the
        // no-edge-adjacent-blues invariant so residual accounting stays exact.
        std::unordered_map<MCGAL::Facet*, uint8_t> fallbackAssignment;
        long long cConn = 0, cEnds = 0, cCross = 0, cZeros = 0, cShape = 0;
        long long cPairs = 0;
        int firstFailComp = -1, firstFailK = -1;
        for (int i = 0; i < static_cast<int>(comps.size()) && allAssigned; ++i) {
            const auto& faces = comps[static_cast<size_t>(i)].faces;
            const int k = static_cast<int>(faces.size());
            if (k < 3) { allAssigned = false; break; }
            std::unordered_map<MCGAL::Facet*, std::vector<MCGAL::Facet*>> adj;
            for (MCGAL::Facet* f : faces) {
                for (MCGAL::Halfedge* hh : f->getHalfedges()) {
                    if (!hh || hh->isRemoved()) continue;
                    MCGAL::Halfedge* opp = hh->opposite();
                    if (!opp || opp->isRemoved() || !opp->face()) continue;
                    MCGAL::Facet* nf = opp->face();
                    auto cit = compByFace.find(nf);
                    if (cit == compByFace.end() || cit->second != i || nf == f)
                        continue;
                    auto& list = adj[f];
                    if (std::find(list.begin(), list.end(), nf) == list.end())
                        list.push_back(nf);
                }
            }
            auto sharesEdgeWith = [&](MCGAL::Facet* a, MCGAL::Facet* b) {
                for (MCGAL::Halfedge* hh : a->getHalfedges()) {
                    MCGAL::Halfedge* opp = hh ? hh->opposite() : nullptr;
                    if (opp && opp->face() == b) return true;
                }
                return false;
            };
            bool assigned = false;
            for (int ai = 0; ai < k && !assigned; ++ai) {
                for (int bi = ai + 1; bi < k && !assigned; ++bi) {
                    MCGAL::Facet* z1 = faces[static_cast<size_t>(ai)];
                    MCGAL::Facet* z2 = faces[static_cast<size_t>(bi)];
                    std::vector<MCGAL::Facet*> ones;
                    ones.reserve(static_cast<size_t>(k - 2));
                    for (int ci = 0; ci < k; ++ci) {
                        if (ci == ai || ci == bi) continue;
                        ones.push_back(faces[static_cast<size_t>(ci)]);
                    }
                    if (ones.empty()) continue;
                    std::unordered_set<MCGAL::Facet*> oneSet(ones.begin(), ones.end());
                    std::unordered_set<MCGAL::Facet*> seen;
                    std::queue<MCGAL::Facet*> q;
                    q.push(ones.front());
                    seen.insert(ones.front());
                    while (!q.empty()) {
                        MCGAL::Facet* cur = q.front();
                        q.pop();
                        for (MCGAL::Facet* nb : adj[cur]) {
                            if (oneSet.count(nb) && seen.insert(nb).second) q.push(nb);
                        }
                    }
                    if (seen.size() != oneSet.size()) { ++cConn; continue; }
                    bool endsOk = false;
                    if (ones.size() == 1) {
                        // Pentagons: one middle triangle, the two external
                        // zeros each share an edge with it (paper §4).
                        endsOk = sharesEdgeWith(z1, ones.front()) &&
                                 sharesEdgeWith(z2, ones.front());
                    } else {
                        std::vector<MCGAL::Facet*> endpoints;
                        for (MCGAL::Facet* f : ones) {
                            int deg = 0;
                            for (MCGAL::Facet* nb : adj[f])
                                if (oneSet.count(nb)) ++deg;
                            if (deg <= 1) endpoints.push_back(f);
                        }
                        if (endpoints.size() == 2) {
                            endsOk =
                                (sharesEdgeWith(z1, endpoints[0]) &&
                                 sharesEdgeWith(z2, endpoints[1])) ||
                                (sharesEdgeWith(z1, endpoints[1]) &&
                                 sharesEdgeWith(z2, endpoints[0]));
                        }
                    }
                    if (!endsOk) { ++cEnds; continue; }
                    bool crossBlue = false;
                    for (MCGAL::Facet* f : ones) {
                        for (MCGAL::Halfedge* hh : f->getHalfedges()) {
                            MCGAL::Halfedge* opp = hh ? hh->opposite() : nullptr;
                            if (!opp || !opp->face()) continue;
                            MCGAL::Facet* nf = opp->face();
                            auto cit = compByFace.find(nf);
                            if (cit != compByFace.end() && cit->second == i) continue;
                            if (usedBlues.count(nf)) { crossBlue = true; break; }
                        }
                        if (crossBlue) break;
                    }
                    if (crossBlue) { ++cCross; continue; }
                    if (forceProbeLevel3 || sideInfoMode) {
                        for (MCGAL::Facet* f : ones) colorMap[f] = 1u;
                        for (MCGAL::Facet* f : ones) usedBlues.insert(f);
                        for (MCGAL::Facet* f : faces)
                            mode2ByFace[f] = (f == z1 || f == z2) ? 0u : 1u;
                        if (sideInfoMode) compChosenZeros[i] = {{z1, z2}};
                        assigned = true;
                        break;
                    }
                    if (fallbackAssignment.empty()) {
                        for (MCGAL::Facet* f : faces) {
                            fallbackAssignment[f] =
                                (f == z1 || f == z2) ? 0u : 1u;
                        }
                    }
                    for (MCGAL::Facet* f : ones) colorMap[f] = 1u;
                    colorMap[z1] = 0u;
                    colorMap[z2] = 0u;
                    bool okShape = false;
                    if (isEnvOn("PMSF_PCATM_STRICT_2COLOR_LEGACY_MATCH")) {
                        // Combinatorial recovery: the decoder enumerates all
                        // adjacent zero pairs and needs exactly one Z-path
                        // completion; validate with the same rule.
                        okShape = legacyZerosUnique(ones, z1, z2, colorMap);
                        if (!okShape) ++cZeros;
                        ++cPairs;
                    } else {
                        std::vector<MCGAL::Facet*> trial = ones;
                        trial.push_back(z1);
                        trial.push_back(z2);
                        const auto zeros = pmsf::pcatm::paperMode2Zeros(
                            ones, colorMap, mesh->bboxMin, mesh->f_quantStep);
                        const bool okZeros =
                            (zeros[0] == z1 && zeros[1] == z2) ||
                            (zeros[0] == z2 && zeros[1] == z1);
                        okShape = okZeros && patchFacesFormZPath(trial, z1, z2);
                        if (okZeros && !okShape) ++cShape;
                        if (!okZeros) ++cZeros;
                        ++cPairs;
                    }
                    if (!okShape) {
                        for (MCGAL::Facet* f : ones) colorMap[f] = 0u;
                        continue;
                    }
                    for (MCGAL::Facet* f : ones) usedBlues.insert(f);
                    for (MCGAL::Facet* f : faces)
                        mode2ByFace[f] = (f == z1 || f == z2) ? 0u : 1u;
                    assigned = true;
                }
            }
            if (!assigned) {
                allAssigned = false;
                if (firstFailComp < 0) {
                    firstFailComp = i;
                    firstFailK = k;
                }
                if (isEnvOn("PMSF_PCATM_MODE2_DIAG")) {
                    int pairTotal = 0, pairConn = 0, pairEnds = 0,
                        pairCross = 0, pairOk = 0;
                    for (int ai = 0; ai < k; ++ai) {
                        for (int bi = ai + 1; bi < k; ++bi) {
                            ++pairTotal;
                            MCGAL::Facet* z1 = faces[static_cast<size_t>(ai)];
                            MCGAL::Facet* z2 = faces[static_cast<size_t>(bi)];
                            std::vector<MCGAL::Facet*> ones2;
                            for (int ci = 0; ci < k; ++ci) {
                                if (ci == ai || ci == bi) continue;
                                ones2.push_back(faces[static_cast<size_t>(ci)]);
                            }
                            if (ones2.empty()) continue;
                            std::unordered_set<MCGAL::Facet*> oneSet2(ones2.begin(), ones2.end());
                            std::unordered_set<MCGAL::Facet*> seen2;
                            std::queue<MCGAL::Facet*> q2;
                            q2.push(ones2.front());
                            seen2.insert(ones2.front());
                            while (!q2.empty()) {
                                MCGAL::Facet* cur = q2.front();
                                q2.pop();
                                for (MCGAL::Facet* nb : adj[cur])
                                    if (oneSet2.count(nb) && seen2.insert(nb).second) q2.push(nb);
                            }
                            if (seen2.size() != oneSet2.size()) continue;
                            ++pairConn;
                            bool ends2 = false;
                            if (ones2.size() == 1) {
                                ends2 = sharesEdgeWith(z1, ones2.front()) &&
                                        sharesEdgeWith(z2, ones2.front());
                            } else {
                                std::vector<MCGAL::Facet*> eps;
                                for (MCGAL::Facet* f2 : ones2) {
                                    int d2 = 0;
                                    for (MCGAL::Facet* nb : adj[f2])
                                        if (oneSet2.count(nb)) ++d2;
                                    if (d2 <= 1) eps.push_back(f2);
                                }
                                if (eps.size() == 2)
                                    ends2 = (sharesEdgeWith(z1, eps[0]) &&
                                             sharesEdgeWith(z2, eps[1])) ||
                                            (sharesEdgeWith(z1, eps[1]) &&
                                             sharesEdgeWith(z2, eps[0]));
                            }
                            if (!ends2) continue;
                            ++pairEnds;
                            bool cross2 = false;
                            for (MCGAL::Facet* f2 : ones2) {
                                for (MCGAL::Halfedge* hh2 : f2->getHalfedges()) {
                                    MCGAL::Halfedge* opp2 = hh2 ? hh2->opposite() : nullptr;
                                    if (!opp2 || !opp2->face()) continue;
                                    MCGAL::Facet* nf2 = opp2->face();
                                    auto cit2 = compByFace.find(nf2);
                                    if (cit2 != compByFace.end() && cit2->second == i) continue;
                                    if (usedBlues.count(nf2)) { cross2 = true; break; }
                                }
                                if (cross2) break;
                            }
                            if (cross2) { ++pairCross; continue; }
                            ++pairOk;
                        }
                    }
                    std::cerr << "[2c-fail] level=" << paperLevel
                              << " comp=" << i << " k=" << k
                              << " pairs=" << pairTotal
                              << " conn=" << pairConn << " ends=" << pairEnds
                              << " cross=" << pairCross << " ok=" << pairOk
                              << " usedBlues=" << usedBlues.size() << std::endl;
                }
                break;
            }
        }
        validMode2 = allAssigned;
        if (sideInfoMode) {
            if (!validMode2 ||
                compChosenZeros.size() != comps.size()) {
                validMode2 = false;
            } else {
                std::unordered_map<MCGAL::Facet*, uint8_t> finalMap;
                for (MCGAL::Facet* f : mesh->faces()) {
                    if (f && !f->isRemoved()) finalMap[f] = 0u;
                }
                for (const auto& kv : mode2ByFace) finalMap[kv.first] = kv.second;
                bool sideOk = true;
                for (int ci = 0; ci < static_cast<int>(comps.size()); ++ci) {
                    auto zit = compChosenZeros.find(ci);
                    if (zit == compChosenZeros.end()) { sideOk = false; break; }
                    std::vector<MCGAL::Facet*> ones;
                    for (MCGAL::Facet* f : comps[static_cast<size_t>(ci)].faces) {
                        auto mit = mode2ByFace.find(f);
                        if (mit != mode2ByFace.end() && mit->second == 1u)
                            ones.push_back(f);
                    }
                    const auto cands = pmsf::pcatm::enumerateMode2ZeroPairs(
                        ones, finalMap, mesh);
                    const int idx = pmsf::pcatm::mode2FindPairIndex(
                        cands, zit->second[0], zit->second[1]);
                    if (idx < 0 || idx >= 4) { sideOk = false; break; }
                    mode2SideIndexByComp[ci] = idx;
                }
                if (!sideOk) {
                    validMode2 = false;
                    mode2SideIndexByComp.clear();
                }
                if (isEnvOn("PMSF_PCATM_MODE2_DIAG")) {
                    std::cerr << "[2c-side] level=" << paperLevel
                              << " comps=" << comps.size()
                              << " assigned=" << (allAssigned ? 1 : 0)
                              << " sideOk=" << (sideOk ? 1 : 0)
                              << " sideIndices=" << mode2SideIndexByComp.size()
                              << std::endl;
                }
            }
        }
        if (!validMode2) {
            // WS-B task-18 UPPER-BOUND PROBE (env-gated, NOT decodable): force
            // a structurally valid assignment so the mode-2 color cost can be
            // measured even though the decoder matcher is not satisfied.
            const char* forceEnv =
                std::getenv("PMSF_PCATM_STRICT_FORCE_MODE2_LEVELS");
            if (forceEnv && *forceEnv && paperLevel < std::atoi(forceEnv)) {
                // Only use the structurally valid fallback (no-adjacent-blues
                // preserving). If even that is unavailable, skip forcing.
                if (!fallbackAssignment.empty()) {
                    mode2ByFace = fallbackAssignment;
                    validMode2 = true;
                }
            }
        }
        if (!validMode2) mode2ByFace.clear();
        if (isEnvOn("PMSF_PCATM_MODE2_DIAG")) {
            std::cerr << "[2c-construct] level=" << paperLevel
                      << " comps=" << comps.size()
                      << " assigned=" << (allAssigned ? 1 : 0)
                      << " pairs=" << cPairs << " connFail=" << cConn
                      << " endsFail=" << cEnds << " crossFail=" << cCross
                      << " zerosFail=" << cZeros << " shapeFail=" << cShape
                      << " firstFailComp=" << firstFailComp
                      << " firstFailK=" << firstFailK << std::endl;
        }
    }

    Mode2RecoverabilityStats mode2Stats;

    std::unordered_set<int> emittedComps;
    std::unordered_set<int> emittedMode2Comps;
    int mode2OneComponents = 0;
    std::vector<MCGAL::Facet*> facesInOrder;
    facesInOrder.reserve(mesh->size_of_facets());
    std::unordered_map<MCGAL::Facet*, uint8_t> mode4ColorByFace;
    std::unordered_map<MCGAL::Facet*, uint8_t> emittedMode2ByFace;

    auto emitFace = [&](MCGAL::Facet* f) {
        facesInOrder.push_back(f);
        auto it = compByFace.find(f);
        if (it == compByFace.end()) {
            symbols.push_back(0u);
            mode2Symbols.push_back(0u);
            mode4ColorByFace[f] = 0u;
            emittedMode2ByFace[f] = 0u;
            if (f && !f->isRemoved())
                f->setPCATMPatchTag(0);
            return;
        }
        const int compIdx = it->second;
        const uint8_t mode4Sym = comps[static_cast<size_t>(compIdx)].color;
        symbols.push_back(mode4Sym);
        mode4ColorByFace[f] = mode4Sym;
        const uint8_t mode2Sym = validMode2 ? mode2ByFace[f] : 0u;
        mode2Symbols.push_back(mode2Sym);
        emittedMode2ByFace[f] = mode2Sym;
        if (mode2Sym == 1u && emittedMode2Comps.insert(compIdx).second) {
            ++mode2OneComponents;
        }
    };

    if (isEnvOn("PMSF_PCATM_STRICT_MORTON_FACE_ORDER")) {
        for (MCGAL::Facet* f : mortonFaceOrder(mesh)) {
            emitFace(f);
        }
    } else if (isEnvOn("PMSF_PCATM_STRICT_FACE_ORDER") || isEnvOn("PMSF_PCATM_STRICT_STABLE_FACE_ORDER")) {
        for (MCGAL::Facet* f : stableQuantizedFaceOrder(mesh)) {
            emitFace(f);
        }
    } else {
        std::queue<int> q;
        q.push(seed->poolId());
        int currentVersion = MCGAL::BFSVersionManager::current_version++;
        while (!q.empty()) {
            int hid = q.front();
            q.pop();
            MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndexInSubPool(mesh->meshId(), hid);
            if (!h || h->isVisited(currentVersion) || h->isRemoved())
                continue;
            h->setVisited(currentVersion);

            MCGAL::Facet* f = h->face();
            if (f && !f->isVisited(currentVersion)) {
                f->setVisited(currentVersion);
                emitFace(f);
            }

            MCGAL::Halfedge* hIt = h;
            do {
                MCGAL::Halfedge* hOpp = hIt->opposite();
                if (hOpp && !hOpp->isVisited(currentVersion))
                    q.push(hOpp->poolId());
                if (hIt && !hIt->isVisited(currentVersion))
                    q.push(hIt->poolId());
                hIt = hIt->next();
            } while (hIt != h && hIt != nullptr);
        }
    }

    // PMSF_BPV_DIAG only: dump encoder patch components for a given encoder
    // round so they can be diffed against the decoder's color-recovered ones.

    const char* forceProbeEnv2 =
        std::getenv("PMSF_PCATM_STRICT_FORCE_MODE2_LEVELS");
    const bool forceProbeLevel =
        forceProbeEnv2 && *forceProbeEnv2 &&
        paperLevel < std::atoi(forceProbeEnv2);
    if (validMode2 && !forceProbeLevel && !sideInfoMode &&
        !decoderSelectionMatchesEncoderComponents(
            facesInOrder,
            emittedMode2ByFace,
            compByFace,
            mesh->bboxMin,
            mesh->f_quantStep,
            static_cast<int>(comps.size()))) {
        ++mode2InvalidShape;
        validMode2 = false;
    }

    // Byte-neutral diagnostic (default off): categorise, per encoder patch
    // component, why the color-only decoder cannot reproduce the encoder's
    // component selection.  The decoder can only *seed* recovery from a BLUE
    // (mode2==1) face, so any splittable component with zero blue faces
    // (valence-3/4 patches, whose Z color sequence is all-zero) is invisible
    // to mode2 and forces recovered<expectedPatches.  Never mutates bytes.
    if (isEnvOn("PMSF_PCATM_MODE2_DIAG")) {
        std::unordered_map<int, int> blueByComp;
        std::unordered_map<int, int> sizeByComp;
        for (const auto& kv : compByFace) ++sizeByComp[kv.second];
        for (const auto& kv : emittedMode2ByFace) {
            auto cit = compByFace.find(kv.first);
            if (cit == compByFace.end()) continue;
            if (kv.second == 1u) ++blueByComp[cit->second];
        }
        int compsNoBlue = 0;
        int compsOk = 0;
        int compsOneBlue = 0;
        int compsBlueSizeMismatch = 0;
        for (int ci = 0; ci < static_cast<int>(comps.size()); ++ci) {
            const int blue = blueByComp.count(ci) ? blueByComp[ci] : 0;
            const int sz = sizeByComp.count(ci) ? sizeByComp[ci] : 0;
            if (blue == 0) {
                ++compsNoBlue;
            } else if (sz != blue + 2) {
                ++compsBlueSizeMismatch;
            } else {
                ++compsOk;
                if (blue == 1) ++compsOneBlue;
            }
        }
        // Is the blue-less (deg3/4) adjacency subgraph colorable with only two
        // colors?  If so, a single 2-bit mode-3 stream (Z-core=1, deg3/4 in
        // {2,3}) suffices with no extra blob stream.  Report the greedy chromatic
        // number of the deg3/4 subgraph (edges = comp adjacency among blue-less).
        {
            std::vector<int> blueless;
            std::unordered_map<int,int> pos;
            for (int ci = 0; ci < static_cast<int>(comps.size()); ++ci) {
                const int blue = blueByComp.count(ci) ? blueByComp[ci] : 0;
                if (blue == 0) { pos[ci] = static_cast<int>(blueless.size()); blueless.push_back(ci); }
            }
            std::vector<int> colorOf(blueless.size(), 0);
            int maxColor = 0; int adjEdges = 0;
            for (size_t idx = 0; idx < blueless.size(); ++idx) {
                const int ci = blueless[idx];
                bool used[8] = {false,false,false,false,false,false,false,false};
                for (int nb : adj[static_cast<size_t>(ci)]) {
                    auto it = pos.find(nb);
                    if (it == pos.end()) continue;        // neighbor is a Z-patch comp
                    ++adjEdges;
                    if (static_cast<size_t>(it->second) < idx) {
                        int c = colorOf[static_cast<size_t>(it->second)];
                        if (c >= 1 && c < 8) used[c] = true;
                    }
                }
                int c = 1; while (c < 8 && used[c]) ++c;
                colorOf[idx] = c; if (c > maxColor) maxColor = c;
            }
            std::cerr << "[PCATMStrictPaper deg34-chromatic level=" << paperLevel
                      << "] blueLessComps=" << blueless.size()
                      << " deg34AdjEdgesHalf=" << (adjEdges/2)
                      << " greedyColors=" << maxColor << std::endl;
        }
        if (blueLessColorOut) {
            blueLessColorOut->reserve(facesInOrder.size());
            for (MCGAL::Facet* f : facesInOrder) {
                auto cit = compByFace.find(f);
                uint8_t out = 0u;
                if (cit != compByFace.end()) {
                    const int ci = cit->second;
                    const int blue = blueByComp.count(ci) ? blueByComp[ci] : 0;
                    if (blue == 0)
                        out = comps[static_cast<size_t>(ci)].color;  // deg3/4 patch color
                }
                blueLessColorOut->push_back(out);
            }
        }
        std::cerr << "[PCATMStrictPaper mode2-diag level=" << paperLevel
                  << "] comps=" << comps.size()
                  << " noBlue_deg34_blind=" << compsNoBlue
                  << " blueOkPatches=" << compsOk
                  << " singletonBlue=" << compsOneBlue
                  << " blueSizeMismatch=" << compsBlueSizeMismatch
                  << std::endl;
    }

    int mode2BlueCrossEdges = 0;
    int mode2BlueCrossAddedEdges = 0;
    if (isEnvOn("PMSF_BPV_DIAG")) {
        for (MCGAL::Facet* f : facesInOrder) {
            auto fit = emittedMode2ByFace.find(f);
            auto fcomp = compByFace.find(f);
            if (fit == emittedMode2ByFace.end() || fit->second != 1u || fcomp == compByFace.end()) {
                continue;
            }
            MCGAL::Halfedge* start = f ? f->proxyHalfedge() : nullptr;
            if (!start)
                continue;
            MCGAL::Halfedge* h = start;
            do {
                MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
                MCGAL::Facet* nf = opp ? opp->face() : nullptr;
                auto nit = emittedMode2ByFace.find(nf);
                auto ncomp = compByFace.find(nf);
                if (nit != emittedMode2ByFace.end() && nit->second == 1u && ncomp != compByFace.end() && ncomp->second != fcomp->second &&
                    f->poolId() < nf->poolId()) {
                    ++mode2BlueCrossEdges;
                    if ((h && h->isAdded()) || (opp && opp->isAdded())) {
                        ++mode2BlueCrossAddedEdges;
                    }
                }
                h = h ? h->next() : nullptr;
            } while (h && h != start);
        }
    }

    auto appendResidualForComp = [&](int compIdx) {
        if (compIdx < 0 || compIdx >= static_cast<int>(comps.size()))
            return;
        if (!emittedComps.insert(compIdx).second)
            return;
        MCGAL::Facet* root = comps[static_cast<size_t>(compIdx)].root;
        // R11（D6）：原 PMSF_PCATM_STRICT_ABS_GEOM 分支已移除。
        // 该开关用"绝对位置"取代重心，会破坏 normalize_proxy_to_min_position_vertex
        // 所依赖的编解码逐位一致前提（见 DEFECT_TRACKING D6），且只在非默认
        // 实验路径上使用，故按 R11 删除而不是留"坏开关+豁免"。
        residuals.push_back(root->getRemovedVertexPosInt());
    };
    auto appendMode2ResidualForComp = [&](int compIdx) {
        if (compIdx < 0 || compIdx >= static_cast<int>(comps.size()))
            return;
        MCGAL::Facet* root = comps[static_cast<size_t>(compIdx)].root;
        mode2Residuals.push_back(root->getRemovedVertexPosInt());  // 同上（R11）
    };

    // Geometry is stored in exactly the order in which the color-only decoder
    // will recover patch components.  This is the key paper-storage invariant:
    // the color stream replaces the default facet/edge streams, so residuals
    // must be paired to color-recovered patches, not to encoder-local patch ids.
    std::unordered_set<MCGAL::Facet*> consumed4;
    for (MCGAL::Facet* f : facesInOrder) {
        auto cit = mode4ColorByFace.find(f);
        if (cit == mode4ColorByFace.end() || cit->second == 0u || consumed4.count(f)) {
            continue;
        }
        const uint8_t patchColor = cit->second;
        std::queue<MCGAL::Facet*> q;
        consumed4.insert(f);
        q.push(f);
        while (!q.empty()) {
            MCGAL::Facet* cur = q.front();
            q.pop();
            MCGAL::Halfedge* start = cur ? cur->proxyHalfedge() : nullptr;
            if (!start)
                continue;
            MCGAL::Halfedge* h = start;
            do {
                MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
                MCGAL::Facet* nf = opp ? opp->face() : nullptr;
                auto nit = mode4ColorByFace.find(nf);
                if (nit != mode4ColorByFace.end() && nit->second == patchColor && !consumed4.count(nf)) {
                    consumed4.insert(nf);
                    q.push(nf);
                }
                h = h ? h->next() : nullptr;
            } while (h && h != start);
        }
        auto compIt = compByFace.find(f);
        if (compIt != compByFace.end())
            appendResidualForComp(compIt->second);
    }

    if (validMode2) {
        if (sideInfoMode) {
            // Side-info v3 payload: header bit (merged list present), optional
            // merged records (component ordinal, patch count, face count and one
            // patch ordinal per blue face), then one 2-bit candidate zero-pair
            // index per patch in recovery order. encodeMode2SideInfo() LZ-
            // compresses the packed payload before appending it to the round's
            // color stream.
            std::unordered_map<MCGAL::Facet*, int> sideOrderPos;
            sideOrderPos.reserve(facesInOrder.size() * 2);
            for (int i = 0; i < static_cast<int>(facesInOrder.size()); ++i)
                sideOrderPos[facesInOrder[static_cast<size_t>(i)]] = i;
            std::unordered_set<MCGAL::Facet*> consumedSide;
            std::vector<std::vector<MCGAL::Facet*>> sideComps;
            for (MCGAL::Facet* f : facesInOrder) {
                auto cit = emittedMode2ByFace.find(f);
                if (cit == emittedMode2ByFace.end() || cit->second != 1u ||
                    consumedSide.count(f))
                    continue;
                std::vector<MCGAL::Facet*> compFaces;
                std::queue<MCGAL::Facet*> q;
                consumedSide.insert(f);
                q.push(f);
                while (!q.empty()) {
                    MCGAL::Facet* cur = q.front();
                    q.pop();
                    compFaces.push_back(cur);
                    MCGAL::Halfedge* start = cur ? cur->proxyHalfedge() : nullptr;
                    if (!start) continue;
                    MCGAL::Halfedge* h = start;
                    do {
                        MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
                        MCGAL::Facet* nf = opp ? opp->face() : nullptr;
                        auto nit = emittedMode2ByFace.find(nf);
                        if (nit != emittedMode2ByFace.end() && nit->second == 1u &&
                            !consumedSide.count(nf)) {
                            consumedSide.insert(nf);
                            q.push(nf);
                        }
                        h = h ? h->next() : nullptr;
                    } while (h && h != start);
                }
                std::sort(compFaces.begin(), compFaces.end(),
                          [&](MCGAL::Facet* a, MCGAL::Facet* b) {
                              return sideOrderPos[a] < sideOrderPos[b];
                          });
                sideComps.push_back(std::move(compFaces));
            }
            struct SideMergedRecord {
                std::uint32_t compOrdinal = 0;
                std::uint32_t patchCount = 0;
                std::uint32_t faceCount = 0;
                std::vector<std::uint32_t> ords;
            };
            std::vector<SideMergedRecord> mergedRecords;
            std::vector<std::vector<int>> patchOrders;
            bool sideRecordOverflow = false;
            for (int oi = 0; oi < static_cast<int>(sideComps.size()); ++oi) {
                const auto& compFaces = sideComps[static_cast<size_t>(oi)];
                if (compFaces.size() > 63u) {
                    sideRecordOverflow = true;
                    break;
                }
                std::vector<int> patchOrder;
                std::unordered_map<int, int> patchOrd;
                for (MCGAL::Facet* f : compFaces) {
                    auto cit = compByFace.find(f);
                    if (cit == compByFace.end()) continue;
                    if (patchOrd.find(cit->second) == patchOrd.end()) {
                        patchOrd[cit->second] = static_cast<int>(patchOrder.size());
                        patchOrder.push_back(cit->second);
                    }
                }
                const int P = static_cast<int>(patchOrder.size());
                if (P <= 0 || P > 7) {
                    sideRecordOverflow = true;
                    break;
                }
                patchOrders.push_back(patchOrder);
                if (P > 1) {
                    SideMergedRecord rec;
                    rec.compOrdinal = static_cast<std::uint32_t>(oi);
                    rec.patchCount = static_cast<std::uint32_t>(P);
                    rec.faceCount = static_cast<std::uint32_t>(compFaces.size());
                    for (MCGAL::Facet* f : compFaces) {
                        auto cit = compByFace.find(f);
                        rec.ords.push_back(static_cast<std::uint32_t>(
                            cit == compByFace.end() ? 0 : patchOrd[cit->second]));
                    }
                    mergedRecords.push_back(std::move(rec));
                }
            }
            if (sideRecordOverflow) {
                validMode2 = false;
                mode2ByFace.clear();
                mode2Side.clear();
            } else {
                std::vector<uint8_t> sideBits;
                sideBits.reserve(comps.size() * 2 + 64);
                auto putBit = [&](uint8_t b) { sideBits.push_back(b & 1u); };
                auto putBits = [&](uint32_t v, int n) {
                    for (int i = n - 1; i >= 0; --i) putBit((v >> i) & 1u);
                };
                putBit(mergedRecords.empty() ? 0u : 1u);
                if (!mergedRecords.empty()) {
                    putBits(static_cast<uint32_t>(mergedRecords.size()), 12);
                    for (const auto& mr : mergedRecords) {
                        putBits(mr.compOrdinal, 12);
                        putBits(mr.patchCount, 3);
                        putBits(mr.faceCount, 6);
                        int pbits = 1;
                        while ((1 << pbits) < static_cast<int>(mr.patchCount))
                            ++pbits;
                        for (std::uint32_t ord : mr.ords)
                            putBits(ord, pbits);
                    }
                }
                // v7 (task-36) payload: [hasMerged][merged records]
                // [patchCount u16][candSum u32] then per patch two indices into
                // the topology-only zero-candidate list (ceil(log2(nCand)) bits
                // each). LZ-compressed afterwards; the footer/patchCount/candSum
                // checksums of v5 are retained so any desync fails closed.
                struct SideIndexRec {
                    int onesCount = 0;
                    int candCount = 0;
                    int bits = 0;
                    int idx0 = 0;
                    int idx1 = 0;
                };
                std::vector<SideIndexRec> indexRecs;
                long long emitCandSum = 0;
                int emitPatchCount = 0;
                for (const auto& patchOrder : patchOrders) {
                    for (int ci : patchOrder) {
                        auto oit = compZeroIdx.find(ci);
                        if (oit == compZeroIdx.end()) {
                            sideRecordOverflow = true;
                            break;
                        }
                        SideIndexRec rec;
                        rec.bits = compZeroBits.at(ci);
                        rec.idx0 = oit->second[0];
                        rec.idx1 = oit->second[1];
                        rec.candCount = compZeroCandCount.at(ci);
                        int onesCount = 0;
                        for (MCGAL::Facet* f :
                             comps[static_cast<size_t>(ci)].faces) {
                            auto mit = emittedMode2ByFace.find(f);
                            if (mit != emittedMode2ByFace.end() &&
                                mit->second == 1u)
                                ++onesCount;
                        }
                        rec.onesCount = onesCount;
                        emitCandSum += static_cast<long long>(rec.candCount);
                        ++emitPatchCount;
                        indexRecs.push_back(rec);
                    }
                    if (sideRecordOverflow) break;
                }
                putBits(static_cast<std::uint32_t>(emitPatchCount), 16);
                putBits(static_cast<std::uint32_t>(emitCandSum), 32);
                int dumpIdx = 0;
                for (const auto& rec : indexRecs) {
                    if (rec.bits > 0) {
                        putBits(static_cast<std::uint32_t>(rec.idx0), rec.bits);
                        putBits(static_cast<std::uint32_t>(rec.idx1), rec.bits);
                    }
                    if (isEnvOn("PMSF_PCATM_SIDE_DUMP")) {
                        std::cerr << "[2c-dump-enc] level=" << paperLevel
                                  << " i=" << dumpIdx
                                  << " ones=" << rec.onesCount
                                  << " cands=" << rec.candCount
                                  << " idx0=" << rec.idx0
                                  << " idx1=" << rec.idx1 << std::endl;
                    }
                    ++dumpIdx;
                }
                if (sideRecordOverflow) {
                    validMode2 = false;
                    mode2ByFace.clear();
                    mode2Side.clear();
                    compZeroIdx.clear();
                } else {
                    if (isEnvOn("PMSF_PCATM_MODE2_DIAG")) {
                        std::cerr << "[2c-emit-v7] level=" << paperLevel
                                  << " patches=" << emitPatchCount
                                  << " candSum=" << emitCandSum
                                  << " sideBits=" << sideBits.size()
                                  << std::endl;
                    }
                    const std::vector<uint8_t> payload =
                        pmsf::pcatm::packBits(sideBits);
                    mode2Side = encodeMode2SideInfo(payload);
                    if (isEnvOn("PMSF_PCATM_SIDE_DUMP")) {
                        std::cerr << "[2c-side-bytes] level=" << paperLevel
                                  << " bits=" << sideBits.size()
                                  << " payload=" << payload.size()
                                  << " side=" << mode2Side.size()
                                  << " hex=";
                        for (std::size_t bi = 0;
                             bi < std::min<std::size_t>(8, payload.size()); ++bi) {
                            char buf[4];
                            std::snprintf(buf, sizeof(buf), "%02x", payload[bi]);
                            std::cerr << buf;
                        }
                        std::cerr << std::endl;
                    }
                    for (const auto& patchOrder : patchOrders) {
                        for (int ci : patchOrder) appendMode2ResidualForComp(ci);
                    }
                }
            }
        } else {
        std::unordered_set<MCGAL::Facet*> consumed2;
        std::unordered_set<int> emittedMode2ResidualComps;
        for (MCGAL::Facet* f : facesInOrder) {
            auto cit = emittedMode2ByFace.find(f);
            if (cit == emittedMode2ByFace.end() || cit->second != 1u || consumed2.count(f)) {
                continue;
            }
            std::queue<MCGAL::Facet*> q;
            consumed2.insert(f);
            q.push(f);
            while (!q.empty()) {
                MCGAL::Facet* cur = q.front();
                q.pop();
                MCGAL::Halfedge* start = cur ? cur->proxyHalfedge() : nullptr;
                if (!start)
                    continue;
                MCGAL::Halfedge* h = start;
                do {
                    MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
                    MCGAL::Facet* nf = opp ? opp->face() : nullptr;
                    auto nit = emittedMode2ByFace.find(nf);
                    if (nit != emittedMode2ByFace.end() && nit->second == 1u && !consumed2.count(nf)) {
                        consumed2.insert(nf);
                        q.push(nf);
                    }
                    h = h ? h->next() : nullptr;
                } while (h && h != start);
            }
            auto compIt = compByFace.find(f);
            if (compIt != compByFace.end() && emittedMode2ResidualComps.insert(compIt->second).second) {
                appendMode2ResidualForComp(compIt->second);
            }
        }
        }
    }


    // ------------------------------------------------------------------
    // mode-3 (PMSF_PCATM_MODE2 lever): a single 2-bit color stream that keeps
    //   1 = Z-core (blue, recovered geometrically like mode2),
    //   2 = deg3/4 patch face (recovered by mode4-style same-color grouping),
    //   0 = everything else (Z endpoints + untouched; endpoints re-derived).
    // deg3/4 patches are never mutually edge-adjacent, so a single color (2)
    // keeps them separable. This makes mixed rounds recoverable while paying
    // only ~1 nonzero symbol per Z patch instead of (valence-2).
    // ------------------------------------------------------------------
    {
        std::unordered_map<int, int> blueByComp3;
        for (const auto& kv : emittedMode2ByFace) {
            auto cit = compByFace.find(kv.first);
            if (cit == compByFace.end()) continue;
            if (kv.second == 1u) ++blueByComp3[cit->second];
        }
        int blueComps3 = 0;
        int blueLessComps3 = 0;
        for (int ci = 0; ci < static_cast<int>(comps.size()); ++ci) {
            if (blueByComp3.count(ci) && blueByComp3[ci] > 0) ++blueComps3;
            else ++blueLessComps3;
        }
        // Build the mode-3 symbol vector in facesInOrder.
        mode3Symbols.reserve(facesInOrder.size());
        std::unordered_map<MCGAL::Facet*, uint8_t> mode3ByFace;
        mode3ByFace.reserve(facesInOrder.size() * 2 + 16);
        for (MCGAL::Facet* f : facesInOrder) {
            uint8_t sym = 0u;
            auto mit = emittedMode2ByFace.find(f);
            if (mit != emittedMode2ByFace.end() && mit->second == 1u) {
                sym = 1u;  // Z core (blue)
            } else {
                auto cit = compByFace.find(f);
                if (cit != compByFace.end()) {
                    const int ci = cit->second;
                    const bool blueLess = !(blueByComp3.count(ci) && blueByComp3[ci] > 0);
                    if (blueLess) sym = 2u;  // deg3/4 patch face
                }
            }
            mode3Symbols.push_back(sym);
            mode3ByFace[f] = sym;
        }
        // (a) blue Z-patches must be geometrically recoverable: reuse the exact
        // decoder-selection matcher but only require the blue-containing comps.
        const bool blueGateOk =
            blueComps3 == 0 ||
            decoderSelectionMatchesEncoderComponents(
                facesInOrder, emittedMode2ByFace, compByFace,
                mesh->bboxMin, mesh->f_quantStep, blueComps3);
        // (b) simulate the decoder's color-2 (deg3/4) recovery and require it to
        // reproduce exactly the blue-less encoder comps (no cross-comp merge).
        bool deg34Ok = true;
        int recoveredDeg34 = 0;
        {
            std::unordered_set<MCGAL::Facet*> consumed2;
            for (MCGAL::Facet* f : facesInOrder) {
                auto it = mode3ByFace.find(f);
                if (it == mode3ByFace.end() || it->second != 2u || consumed2.count(f))
                    continue;
                std::vector<MCGAL::Facet*> group;
                std::queue<MCGAL::Facet*> q2;
                consumed2.insert(f);
                q2.push(f);
                int comp = -1;
                bool compOk = true;
                while (!q2.empty()) {
                    MCGAL::Facet* cur = q2.front();
                    q2.pop();
                    group.push_back(cur);
                    auto cc = compByFace.find(cur);
                    if (cc == compByFace.end()) { compOk = false; }
                    else if (comp < 0) comp = cc->second;
                    else if (comp != cc->second) compOk = false;
                    MCGAL::Halfedge* start = cur ? cur->proxyHalfedge() : nullptr;
                    if (!start) continue;
                    MCGAL::Halfedge* h = start;
                    do {
                        MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
                        MCGAL::Facet* nf = opp ? opp->face() : nullptr;
                        auto nit = mode3ByFace.find(nf);
                        if (nit != mode3ByFace.end() && nit->second == 2u && !consumed2.count(nf)) {
                            consumed2.insert(nf);
                            q2.push(nf);
                        }
                        h = h ? h->next() : nullptr;
                    } while (h && h != start);
                }
                // group must equal exactly one full encoder comp.
                if (!compOk || comp < 0) { deg34Ok = false; break; }
                size_t compSize = 0;
                for (const auto& kv : compByFace) if (kv.second == comp) ++compSize;
                if (compSize != group.size()) { deg34Ok = false; break; }
                ++recoveredDeg34;
            }
            if (recoveredDeg34 != blueLessComps3) deg34Ok = false;
        }
        if (blueGateOk && deg34Ok) {
            // Residuals in the exact decoder recovery order: all Z patches
            // (facesInOrder blue-component order) then all deg3/4 patches
            // (facesInOrder color-2-component order).
            std::unordered_set<MCGAL::Facet*> cZ;
            std::unordered_set<int> seenZComp;
            for (MCGAL::Facet* f : facesInOrder) {
                auto it = mode3ByFace.find(f);
                if (it == mode3ByFace.end() || it->second != 1u || cZ.count(f)) continue;
                std::queue<MCGAL::Facet*> q;
                cZ.insert(f); q.push(f);
                int comp = -1;
                while (!q.empty()) {
                    MCGAL::Facet* cur = q.front(); q.pop();
                    auto cc = compByFace.find(cur);
                    if (cc != compByFace.end() && comp < 0) comp = cc->second;
                    MCGAL::Halfedge* start = cur ? cur->proxyHalfedge() : nullptr;
                    if (!start) continue;
                    MCGAL::Halfedge* h = start;
                    do {
                        MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
                        MCGAL::Facet* nf = opp ? opp->face() : nullptr;
                        auto nit = mode3ByFace.find(nf);
                        if (nit != mode3ByFace.end() && nit->second == 1u && !cZ.count(nf)) {
                            cZ.insert(nf); q.push(nf);
                        }
                        h = h ? h->next() : nullptr;
                    } while (h && h != start);
                }
                if (comp >= 0 && seenZComp.insert(comp).second) {
                    MCGAL::Facet* root = comps[static_cast<size_t>(comp)].root;
                    mode3Residuals.push_back(root->getRemovedVertexPosInt());
                }
            }
            std::unordered_set<MCGAL::Facet*> cD;
            std::unordered_set<int> seenDComp;
            for (MCGAL::Facet* f : facesInOrder) {
                auto it = mode3ByFace.find(f);
                if (it == mode3ByFace.end() || it->second != 2u || cD.count(f)) continue;
                std::queue<MCGAL::Facet*> q;
                cD.insert(f); q.push(f);
                int comp = -1;
                while (!q.empty()) {
                    MCGAL::Facet* cur = q.front(); q.pop();
                    auto cc = compByFace.find(cur);
                    if (cc != compByFace.end() && comp < 0) comp = cc->second;
                    MCGAL::Halfedge* start = cur ? cur->proxyHalfedge() : nullptr;
                    if (!start) continue;
                    MCGAL::Halfedge* h = start;
                    do {
                        MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
                        MCGAL::Facet* nf = opp ? opp->face() : nullptr;
                        auto nit = mode3ByFace.find(nf);
                        if (nit != mode3ByFace.end() && nit->second == 2u && !cD.count(nf)) {
                            cD.insert(nf); q.push(nf);
                        }
                        h = h ? h->next() : nullptr;
                    } while (h && h != start);
                }
                if (comp >= 0 && seenDComp.insert(comp).second) {
                    MCGAL::Facet* root = comps[static_cast<size_t>(comp)].root;
                    mode3Residuals.push_back(root->getRemovedVertexPosInt());
                }
            }
            mode3Recoverable =
                mode3Symbols.size() == symbols.size() &&
                mode3Residuals.size() == comps.size();
        }
        if (isEnvOn("PMSF_PCATM_MODE2_DIAG")) {
            // Categorise why blue (Z) recovery fails: the decoder BFSes blue
            // connectivity; if two Z patches have edge-adjacent blue cores the
            // BFS merges them (ones span >1 comp) and recovery is impossible.
            std::unordered_set<MCGAL::Facet*> cz;
            int blueBfsComps = 0, multiComp = 0, nullZeros = 0, zerosWrongComp = 0, sizeBad = 0, okZ = 0;
            for (MCGAL::Facet* f : facesInOrder) {
                auto it = mode3ByFace.find(f);
                if (it == mode3ByFace.end() || it->second != 1u || cz.count(f)) continue;
                ++blueBfsComps;
                std::vector<MCGAL::Facet*> ones;
                std::queue<MCGAL::Facet*> q; cz.insert(f); q.push(f);
                int comp = -1; bool multi = false;
                while (!q.empty()) {
                    MCGAL::Facet* cur = q.front(); q.pop(); ones.push_back(cur);
                    auto cc = compByFace.find(cur);
                    if (cc != compByFace.end()) { if (comp < 0) comp = cc->second; else if (comp != cc->second) multi = true; }
                    MCGAL::Halfedge* st = cur ? cur->proxyHalfedge() : nullptr; if (!st) continue;
                    MCGAL::Halfedge* h = st;
                    do { MCGAL::Halfedge* o = h?h->opposite():nullptr; MCGAL::Facet* nf = o?o->face():nullptr;
                         auto nit = mode3ByFace.find(nf);
                         if (nit!=mode3ByFace.end() && nit->second==1u && !cz.count(nf)) { cz.insert(nf); q.push(nf); }
                         h = h?h->next():nullptr; } while (h && h!=st);
                }
                if (multi) { ++multiComp; continue; }
                auto zeros = pmsf::pcatm::paperMode2Zeros(ones, emittedMode2ByFace, mesh->bboxMin, mesh->f_quantStep);
                if (!zeros[0] || !zeros[1]) { ++nullZeros; continue; }
                bool zc = true; for (MCGAL::Facet* z : zeros) { auto cc = compByFace.find(z); if (cc==compByFace.end()||cc->second!=comp) zc=false; }
                if (!zc) { ++zerosWrongComp; continue; }
                size_t cs = 0; for (auto& kv : compByFace) if (kv.second==comp) ++cs;
                if (cs != ones.size()+2) { ++sizeBad; continue; }
                ++okZ;
            }
            std::cerr << "[PCATMStrictPaper mode3-Zfail level=" << paperLevel
                      << "] blueComps=" << blueComps3 << " blueBfsComps=" << blueBfsComps
                      << " multiCompMerge=" << multiComp << " nullZeros=" << nullZeros
                      << " zerosWrongComp=" << zerosWrongComp << " sizeBad=" << sizeBad
                      << " okZ=" << okZ << std::endl;
        }
        if (const char* env = std::getenv("PMSF_BPV_DIAG"); env && env[0] == '1') {
            std::cerr << "[PCATMStrictPaper mode3-build level=" << paperLevel
                      << "] blueComps=" << blueComps3
                      << " blueLessComps=" << blueLessComps3
                      << " blueGateOk=" << (blueGateOk ? 1 : 0)
                      << " deg34Ok=" << (deg34Ok ? 1 : 0)
                      << " recoverable=" << (mode3Recoverable ? 1 : 0)
                      << " residuals=" << mode3Residuals.size()
                      << " comps=" << comps.size() << std::endl;
        }
    }

    mode2Recoverable =
        validMode2 && mode2Symbols.size() == symbols.size() &&
        mode2Residuals.size() == comps.size() &&
        (sideInfoMode ? !mode2Side.empty()
                      : mode2OneComponents == static_cast<int>(comps.size()));
    if (const char* forceEnv = std::getenv("PMSF_PCATM_STRICT_FORCE_MODE2_LEVELS");
        forceEnv && *forceEnv && paperLevel < std::atoi(forceEnv)) {
        mode2Recoverable = true;
    }
    if (const char* env = std::getenv("PMSF_BPV_DIAG"); env && env[0] == '1') {
        std::cerr << "[PCATMStrictPaper mode2-candidate level=" << paperLevel << "] comps=" << comps.size() << " valid=" << (validMode2 ? 1 : 0)
                  << " oneComponents=" << mode2OneComponents << " small=" << mode2InvalidSmall << " degree=" << mode2InvalidDegree
                  << " shape=" << mode2InvalidShape << " cross=" << mode2InvalidCross << " expectedPatches=" << mode2Stats.expectedPatches
                  << " oneComponentsActual=" << mode2Stats.oneComponents << " zeroMismatch=" << mode2Stats.zeroMismatch
                  << " treeMismatch=" << mode2Stats.treeMismatch << " blueCrossEdges=" << mode2BlueCrossEdges
                  << " blueCrossAddedEdges=" << mode2BlueCrossAddedEdges << std::endl;
    }
}

}  // namespace

void PCATMStrictPMSFSymbolCollectOperator::collect(MCGAL::Halfedge* seed, const std::vector<MCGAL::Halfedge*>& gates) {
    // PCATM paper §5 uses the patch barycenter vc as the prediction point.
    // PCATMPaper/PCATMZColor inherit PCLTTMTag::collect(), which rewrites the
    // first-order barycenter residual into a second-order curvature residual.
    // Bypass that rewrite for the strict paper storage path.
    //
    // Keep PCLTTMTag's pre-BFS proxy normalization, because it affects the
    // deterministic face proxy carried into following simplification rounds.
    if (mesh_) {
        for (MCGAL::Facet* f : mesh_->faces()) {
            if (f && !f->isRemoved()) {
                f->normalize_proxy_to_min_position_vertex(mesh_->bboxMin, mesh_->f_quantStep);
            }
        }
    }
    PCATMColorSymbolCollectOperator::collect(seed, gates);
    strictColorSymbolsPerRound_.emplace_back();
    strictMode2SymbolsPerRound_.emplace_back();
    strictResidualQueues_.emplace_back();
    strictMode2ResidualQueues_.emplace_back();
    bool mode2Recoverable = false;
    bool mode3Recoverable = false;
    strictBlueLessColorPerRound_.emplace_back();
    strictMode3SymbolsPerRound_.emplace_back();
    strictMode3ResidualQueues_.emplace_back();
    strictMode2SidePerRound_.emplace_back();
    buildPaper4Round(mesh_, seed, strictColorSymbolsPerRound_.back(), strictMode2SymbolsPerRound_.back(), mode2Recoverable,
                     strictResidualQueues_.back(), strictMode2ResidualQueues_.back(),
                     strictMode3SymbolsPerRound_.back(), mode3Recoverable, strictMode3ResidualQueues_.back(),
                     strictMode2SidePerRound_.back(),
                     static_cast<int>(strictColorSymbolsPerRound_.size()) - 1,
                     &strictBlueLessColorPerRound_.back());
    if (isEnvOn("PMSF_PCATM_MODE2_DIAG")) {
        std::cerr << "[strict-collect] round="
                  << (static_cast<int>(strictColorSymbolsPerRound_.size()) - 1)
                  << " mode2Rec=" << (mode2Recoverable ? 1 : 0)
                  << " mainRes=" << strictResidualQueues_.back().size()
                  << " mode2Res=" << strictMode2ResidualQueues_.back().size()
                  << " mode3Res=" << strictMode3ResidualQueues_.back().size()
                  << " comps~" << strictMode2SymbolsPerRound_.back().size()
                  << " m2nz="
                  << ([&] {
                         long long nz = 0;
                         for (uint8_t v : strictMode2SymbolsPerRound_.back())
                             if (v != 0) ++nz;
                         return nz;
                     })()
                  << std::endl;
    }
    strictMode2RecoverablePerRound_.push_back(mode2Recoverable ? 1u : 0u);
    strictMode3RecoverablePerRound_.push_back(mode3Recoverable ? 1u : 0u);
    paperTableCache_.clear();
}

void PCATMStrictPMSFSymbolCollectOperator::collect(MCGAL::Halfedge* seed) {
    static const std::vector<MCGAL::Halfedge*> kEmpty;
    collect(seed, kEmpty);
}

void PCATMStrictPMSFSymbolCollectOperator::collect(std::vector<MCGAL::Halfedge*> seeds) {
    if (seeds.empty()) {
        throw std::runtime_error("[PCATMStrictPaper] collect(vector<seed>) called with empty seeds");
    }
    static const std::vector<MCGAL::Halfedge*> kEmpty;
    collect(seeds[0], kEmpty);
}

const pmsf::PMSFSymbolTable* PCATMStrictPMSFSymbolCollectOperator::pmsfSymbolTable() const {
    const pmsf::PMSFSymbolTable* base = PCATMPMSFSymbolCollectOperator::pmsfSymbolTable();
    if (!base || base->empty()) {
        return nullptr;
    }
    pmsf::copyPMSFSymbolTableWithIndexedRowAugmentation(base, paperTableCache_, [this](std::size_t r, pmsf::PMSFSymbolRecord& row) {
        if (r < strictColorSymbolsPerRound_.size()) {
            pmsf::replacePCATMStrictColorSymbols(row.delta, strictColorSymbolsPerRound_[r]);
        }
        if (r < strictMode2SymbolsPerRound_.size()) {
            pmsf::replacePCATMStrictMode2Symbols(row.delta, strictMode2SymbolsPerRound_[r]);
        }
        if (r < strictMode2RecoverablePerRound_.size()) {
            pmsf::appendPCATMStrictMode2Recoverable(row.delta, static_cast<std::int32_t>(strictMode2RecoverablePerRound_[r]));
        }
        pmsf::appendPCATMStrictCapabilitySideChannel(
            row.delta, static_cast<std::int32_t>(lastSchemeVersion_),
            static_cast<std::int32_t>((lastSchemeVersion_ >= kPCATMStrictPaperColorOnlySchemeVersion && lastTopologySideBytes_ == 0) ? 1 : 0),
            static_cast<std::int32_t>(lastTopologySideBytes_ > 0 ? 1 : 0), static_cast<std::int32_t>(lastTopologySideBytes_));
        pmsf::appendPCATMStrictStats(row.delta, static_cast<std::int32_t>(lastMode2Rounds_), static_cast<std::int32_t>(lastMode4Rounds_),
                                     static_cast<std::int32_t>(lastConnectivityBytes_), static_cast<std::int32_t>(lastGeometryBytes_),
                                     static_cast<std::int32_t>(lastContainerOverheadBytes_), static_cast<std::int32_t>(lastRawColorBits_),
                                     static_cast<std::int32_t>(lastRemovedVertices_), static_cast<std::int32_t>(lastDegree3_),
                                     static_cast<std::int32_t>(lastDegree4_), static_cast<std::int32_t>(lastDegree5_),
                                     static_cast<std::int32_t>(lastDegree6_), static_cast<std::int32_t>(lastDegreeOther_));
        if (r < strictResidualQueues_.size()) {
            for (const auto& q : strictResidualQueues_[r]) {
                pmsf::appendPCATMStrictResidualSideChannel(row.delta, q);
            }
        }
        if (r < strictMode2ResidualQueues_.size()) {
            for (const auto& q : strictMode2ResidualQueues_[r]) {
                pmsf::appendPCATMStrictMode2Residual(row.delta, q);
            }
        }
    });
    return paperTableCache_.empty() ? nullptr : &paperTableCache_;
}

int PCATMStrictPMSFSymbolCollectOperator::exportToBufferWithRangeCoder(char* buffer, bool isEnableQuantization) {
    if (buffer == nullptr)
        return 0;
    requirePCATMStrictPaperExactIfRequested();
    if (!enablePrediction_ || !isEnableQuantization) {
        throw std::runtime_error("[PCATMStrictPaper] export requires prediction + quantization.");
    }

    const int numRounds = static_cast<int>(strictColorSymbolsPerRound_.size());
    if (numRounds == 0) {
        throw std::runtime_error("[PCATMStrictPaper] no collected rounds; collect() was not called.");
    }
    if (static_cast<int>(strictResidualQueues_.size()) != numRounds || static_cast<int>(strictMode2ResidualQueues_.size()) != numRounds ||
        static_cast<int>(strictMode2SymbolsPerRound_.size()) != numRounds || static_cast<int>(strictMode2RecoverablePerRound_.size()) != numRounds ||
        static_cast<int>(strictMode3SymbolsPerRound_.size()) != numRounds || static_cast<int>(strictMode3RecoverablePerRound_.size()) != numRounds ||
        static_cast<int>(strictMode3ResidualQueues_.size()) != numRounds ||
        static_cast<int>(gateResidualQueues_.size()) != numRounds) {
        throw std::runtime_error("[PCATMStrictPaper] color/residual round count mismatch.");
    }

    // task-59 B1-alt: paper-faithful entropy recoding of the displacement stream.
    const bool useAdaptiveResidual =
        isEnvOn("PMSF_PCATM_STRICT_ADAPT_RESIDUAL");
    const bool useHuffmanResidual = isEnvOn("PMSF_PCATM_STRICT_HUFFMAN_RESIDUAL");
    const bool useDenseRangeResidual = !useHuffmanResidual && isEnvOn("PMSF_PCATM_STRICT_DENSE_RANGE_RESIDUAL");
    const bool useByteRangeResidual = !useHuffmanResidual && !useDenseRangeResidual && isEnvOn("PMSF_PCATM_STRICT_VARINT_BYTE_RANGE_RESIDUAL");
    const bool useMeppAxisRangeResidual =
        !useHuffmanResidual && !useDenseRangeResidual && !useByteRangeResidual && isEnvOn("PMSF_PCATM_STRICT_MEPP_AXIS_RANGE_RESIDUAL");
    const bool useSplitAxisVarintResidual = !useHuffmanResidual && !useDenseRangeResidual && !useByteRangeResidual && !useMeppAxisRangeResidual &&
                                            isEnvOn("PMSF_PCATM_STRICT_SPLIT_AXIS_VARINT_RESIDUAL");
    const bool useVarintResidual = !useHuffmanResidual && !useDenseRangeResidual && !useByteRangeResidual && !useMeppAxisRangeResidual &&
                                   !useSplitAxisVarintResidual && !isEnvOn("PMSF_PCATM_STRICT_AXIS_VARINT_RESIDUAL");
    const bool useAxisVarintResidual =
        !useHuffmanResidual && !useDenseRangeResidual && !useByteRangeResidual && !useSplitAxisVarintResidual && !useVarintResidual;
    const bool useGlobalHuffman = true;
    bool useGlobalColor = useAdaptiveResidual || useVarintResidual || useSplitAxisVarintResidual || useByteRangeResidual || useMeppAxisRangeResidual;
    const char* forceMode2Env = std::getenv("PMSF_PCATM_STRICT_FORCE_MODE2_LEVELS");
    const int forceMode2Levels =
        (forceMode2Env && *forceMode2Env) ? std::atoi(forceMode2Env) : 0;
    const bool sideInfoExport =
        isEnvOn("PMSF_PCATM_STRICT_2COLOR_SIDE");
    if (forceMode2Levels > 0 || sideInfoExport) {
        // mode-2 rounds need per-round color streams (1 bit/face); the side-info
        // scheme also transports per-patch candidate indices there.
        useGlobalColor = false;
    }
    const int schemeVersion =
        useAdaptiveResidual ? pmsf::kPCATMStrictPaperAdaptiveResidualVersion :
        useVarintResidual ? pmsf::kPCATMStrictPaperOrientedZVersion :
        useSplitAxisVarintResidual ?
            kPCATMStrictPaperColorOnlySplitAxisVarintResidualSchemeVersion :
            (useByteRangeResidual ?
                 kPCATMStrictPaperColorOnlyVarintByteRangeResidualSchemeVersion :
                 (useMeppAxisRangeResidual ?
                      kPCATMStrictPaperColorOnlyMeppAxisRangeResidualSchemeVersion :
                      (useGlobalColor ?
                           kPCATMStrictPaperColorOnlyGlobalColorVarintResidualSchemeVersion :
                           (useHuffmanResidual ?
                                kPCATMStrictPaperColorOnlySchemeVersion :
                                (useDenseRangeResidual ? kPCATMStrictPaperColorOnlyDenseRangeResidualSchemeVersion :
                                                         (useAxisVarintResidual ? kPCATMStrictPaperColorOnlyAxisVarintResidualSchemeVersion :
                                                                                  kPCATMStrictPaperColorOnlyVarintResidualSchemeVersion))))));

    pmsf::ConnectivityBlobBuilder cbb(pmsf::pcatmStrictPaperConnectivityBlobSpec(numRounds, schemeVersion));

    int sumColor = 0;
    int sumGeom = 0;
    int sumValence = 0;
    int sumIsPoly = 0;
    int rawColorBits = 0;
    int removedVertices = 0;
    int degree3 = 0;
    int degree4 = 0;
    int degree5 = 0;
    int degree6 = 0;
    int degreeOther = 0;
    int mode2Rounds = 0;
    int mode4Rounds = 0;
    int colorCodec = -1;
    std::unordered_set<uint32_t> residualAlphabet;

    std::vector<std::deque<MCGAL::PointInt>> residualRounds;
    if (useGlobalHuffman)
        residualRounds.reserve(numRounds);
    std::vector<std::vector<uint8_t>> selectedColorRounds;
    std::vector<int> selectedColorModes;
    if (useGlobalColor) {
        selectedColorRounds.reserve(numRounds);
        selectedColorModes.reserve(numRounds);
    }

    // Keep the same encoder-round reversal convention as PCATMPure/PCATMPaper:
    // decoder level 0 consumes the last simplification round first.
    for (int i = 0; i < numRounds; ++i) {
        const int er = numRounds - 1 - i;

        int roundDeg3 = 0;
        int roundDeg4 = 0;
        for (uint8_t code : gateValenceQueues_[er]) {
            if (code == 0)
                ++roundDeg3;
            else if (code == 1)
                ++roundDeg4;
        }
        const bool allowMode2 =
            schemeVersion == pmsf::kPCATMStrictPaperOrientedZVersion &&
            isEnvOn("PMSF_PCATM_STRICT_ENABLE_MODE2") &&
            !isEnvOn("PMSF_PCATM_STRICT_DISABLE_MODE2");
        const bool recoverableMode2 =
            er < static_cast<int>(strictMode2RecoverablePerRound_.size()) && strictMode2RecoverablePerRound_[static_cast<size_t>(er)] != 0u;
        // mode-3 lever: single 2-bit stream (Z=1 geometric, deg3/4=2 grouped).
        const bool allowMode3 =
            schemeVersion == pmsf::kPCATMStrictPaperOrientedZVersion &&
            isEnvOn("PMSF_PCATM_MODE2") &&
            !isEnvOn("PMSF_PCATM_STRICT_DISABLE_MODE2");
        const bool recoverableMode3 =
            er < static_cast<int>(strictMode3RecoverablePerRound_.size()) && strictMode3RecoverablePerRound_[static_cast<size_t>(er)] != 0u;
        int colorMode = (allowMode3 && recoverableMode3) ? 3
                              : ((allowMode2 && recoverableMode2) ? 2 : 4);
        if (forceMode2Levels > 0 && er < forceMode2Levels) {
            colorMode = 2;
        }
        if (const char* env = std::getenv("PMSF_BPV_DIAG"); env && env[0] == '1') {
            std::cerr << "[PCATMStrictPaper round-mode er=" << er << " dr=" << i << "] deg3=" << roundDeg3 << " deg4=" << roundDeg4
                      << " recoverableMode2=" << (recoverableMode2 ? 1 : 0)
                      << " recoverableMode3=" << (recoverableMode3 ? 1 : 0)
                      << " selectedMode=" << colorMode << std::endl;
        }
        if (colorMode == 4)
            ++mode4Rounds;
        else
            ++mode2Rounds;

        std::vector<uint8_t> colorSymbols;
        if (colorMode == 4) {
            colorSymbols = strictColorSymbolsPerRound_[static_cast<size_t>(er)];
        } else if (colorMode == 3) {
            colorSymbols = strictMode3SymbolsPerRound_[static_cast<size_t>(er)];
        } else {
            colorSymbols = strictMode2SymbolsPerRound_[static_cast<size_t>(er)];
        }
        if (useGlobalColor) {
            selectedColorRounds.push_back(colorSymbols);
            selectedColorModes.push_back(colorMode);
        } else {
            auto color = encodeColorSymbolsLzV4(colorSymbols, colorMode);
            if (sideInfoExport && colorMode == 2 &&
                er >= 0 && er < static_cast<int>(strictMode2SidePerRound_.size())) {
                // v5 fix: strictMode2SidePerRound_ already holds the encoded
                // "C2S3" container from buildPaper4Round; appending it directly
                // avoids the v4 double-wrap (which made the decoder inflate the
                // outer container and receive the inner container as payload).
                const std::vector<uint8_t>& sideBytes =
                    strictMode2SidePerRound_[static_cast<size_t>(er)];
                const int32_t sideLen = static_cast<int32_t>(sideBytes.size());
                color.insert(color.end(), sideBytes.begin(), sideBytes.end());
                // WS-B task-24 (v5): explicit footer so the decoder locates the
                // side container from the END of the round stream instead of
                // trusting the color stream header's length field (v4 root cause:
                // the two differed by 23 B for candidate-dependent layouts).
                appendI32(color, sideLen);
                appendI32(color, 0x544F4F46);  // "FOOT"
            }
            sumColor += static_cast<int>(color.size());
            cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamPCATMStrictPaperColor, color);
        }
        rawColorBits += static_cast<int>(colorSymbols.size()) * ((colorMode == 2) ? 1 : 2);  // mode3/4 => 2-bit
        if (const char* env = std::getenv("PMSF_BPV_DIAG"); env && env[0] == '1') {
            int nz = 0;
            for (uint8_t sym : colorSymbols) {
                if (sym != 0u)
                    ++nz;
            }
            std::cerr << "[PCATMStrictPaper color-round er=" << er << " dr=" << i << "] mode=" << colorMode << " symbols=" << colorSymbols.size()
                      << " nonzero=" << nz << " density=" << (colorSymbols.empty() ? 0.0 : double(nz) / double(colorSymbols.size())) << std::endl;
        }

        for (uint8_t code : gateValenceQueues_[er]) {
            if (code == 0)
                ++degree3;
            else if (code == 1)
                ++degree4;
            else if (code == 2)
                ++degree5;
            else if (code == 3)
                ++degree6;
            else
                ++degreeOther;
        }

        const auto& residualQueue =
            (colorMode == 2) ? strictMode2ResidualQueues_[static_cast<size_t>(er)]
          : (colorMode == 3) ? strictMode3ResidualQueues_[static_cast<size_t>(er)]
          : strictResidualQueues_[static_cast<size_t>(er)];
        removedVertices += static_cast<int>(residualQueue.size());
        for (const auto& r : residualQueue) {
            residualAlphabet.insert(zigzag32(r.x()));
            residualAlphabet.insert(zigzag32(r.y()));
            residualAlphabet.insert(zigzag32(r.z()));
        }
        if (useGlobalHuffman) {
            residualRounds.push_back(residualQueue);
            if (i > 0) {
                cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamPCATMStrictPaperGeometry, std::vector<uint8_t>{});
            }
        } else {
            auto geom = encodeResidualHuffman(residualQueue);
            sumGeom += static_cast<int>(geom.size());
            cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamPCATMStrictPaperGeometry, geom);
        }
    }

    if (useGlobalHuffman) {
        std::vector<uint8_t> geom;
        if (useAdaptiveResidual) {
            std::vector<std::vector<std::array<int32_t, 3>>> adaptRounds;
            adaptRounds.reserve(residualRounds.size());
            for (const auto& q : residualRounds) {
                std::vector<std::array<int32_t, 3>> vals;
                vals.reserve(q.size());
                for (const auto& r : q)
                    vals.push_back({r.x(), r.y(), r.z()});
                adaptRounds.push_back(std::move(vals));
            }
            geom = pmsf::pcatmstrict::ard::encode(adaptRounds);
        } else {
        geom = useHuffmanResidual ?
                        encodeResidualGlobalHuffman(residualRounds) :
                        (useDenseRangeResidual ?
                             encodeResidualGlobalDenseRange(residualRounds) :
                             (useByteRangeResidual ?
                                  encodeResidualGlobalVarintByteRange(residualRounds) :
                                  (useMeppAxisRangeResidual ?
                                       encodeResidualGlobalMeppAxisRange(residualRounds) :
                                       (useSplitAxisVarintResidual ? encodeResidualGlobalSplitAxisVarintZlib(residualRounds) :
                                                                     (useAxisVarintResidual ? encodeResidualGlobalAxisVarintZlib(residualRounds) :
                                                                                              encodeResidualGlobalVarintZlib(residualRounds))))));
        }
        sumGeom = static_cast<int>(geom.size());
        cbb.appendStreamBytes(0, pmsf::kPaperBlobStreamPCATMStrictPaperGeometry, geom);
    }
    if (useGlobalColor) {
        auto color = encodeColorRoundsGlobal(selectedColorRounds, selectedColorModes, &colorCodec);
        sumColor = static_cast<int>(color.size());
        cbb.appendStreamBytes(0, pmsf::kPaperBlobStreamPCATMStrictPaperColor, color);
        for (int i = 1; i < numRounds; ++i) {
            cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamPCATMStrictPaperColor, std::vector<uint8_t>{});
        }
    }

    auto blob = cbb.finalize();
    std::memcpy(buffer, blob.data(), blob.size());

    lastConnectivityBytes_ = sumColor;
    lastGeometryBytes_ = sumGeom;
    lastContainerOverheadBytes_ = static_cast<int>(blob.size()) - sumColor - sumGeom - sumValence - sumIsPoly;
    lastRawColorBits_ = rawColorBits;
    lastRemovedVertices_ = removedVertices;
    lastDegree3_ = degree3;
    lastDegree4_ = degree4;
    lastDegree5_ = degree5;
    lastDegree6_ = degree6;
    lastDegreeOther_ = degreeOther;
    lastSchemeVersion_ = schemeVersion;
    lastTopologySideBytes_ = sumValence + sumIsPoly;
    lastMode2Rounds_ = mode2Rounds;
    lastMode4Rounds_ = mode4Rounds;

    // Byte-neutral measurement (default off): quantify the realistic color
    // saving of converting rounds to mode2 under a per-round blue-less-patch
    // (deg3/4) budget T.  For each T we build the same mixed 2-bit global color
    // stream the real path would, but with every round whose blue-less patch
    // count <= T switched to mode2, and we report both the resulting color
    // bytes and the number of deg3/4 patches that would need side-info.  Never
    // changes the real blob.
    if (useGlobalColor && isEnvOn("PMSF_PCATM_MODE2_DIAG")) {
        std::vector<int> degPerDR(numRounds, 0);
        std::vector<int> blueOnesPerDR(numRounds, 0);
        for (int i = 0; i < numRounds; ++i) {
            const int er = numRounds - 1 - i;
            int d = 0;
            for (uint8_t code : gateValenceQueues_[static_cast<size_t>(er)]) {
                if (code == 0 || code == 1) ++d;  // deg3 or deg4
            }
            degPerDR[static_cast<size_t>(i)] = d;
            int ones = 0;
            for (uint8_t sym : strictMode2SymbolsPerRound_[static_cast<size_t>(er)])
                if (sym == 1u) ++ones;
            blueOnesPerDR[static_cast<size_t>(i)] = ones;
        }
        for (int T : {0, 1, 2, 5, 10, 20, 50, 100, 1000000}) {
            std::vector<std::vector<uint8_t>> rr;
            std::vector<int> mm;
            rr.reserve(numRounds); mm.reserve(numRounds);
            int converted = 0; long sidePatches = 0;
            for (int i = 0; i < numRounds; ++i) {
                const int er = numRounds - 1 - i;
                const bool toMode2 = (degPerDR[static_cast<size_t>(i)] <= T);
                if (toMode2) {
                    rr.push_back(strictMode2SymbolsPerRound_[static_cast<size_t>(er)]);
                    mm.push_back(2);
                    ++converted;
                    sidePatches += degPerDR[static_cast<size_t>(i)];
                } else {
                    rr.push_back(strictColorSymbolsPerRound_[static_cast<size_t>(er)]);
                    mm.push_back(4);
                }
            }
            int cc = -1;
            auto col = encodeColorRoundsGlobal(rr, mm, &cc);
            // Side-info lower-bound estimate: 1 bit (val3/4) + ceil(log2 faces)
            // index per blue-less patch is pessimistic; use ~2 bytes/patch.
            const long sideBytesEst = sidePatches * 2;
            std::cerr << "[PCATMStrictPaper mode2-sweep T=" << T
                      << "] convertedRounds=" << converted
                      << " colorLZ=" << col.size()
                      << " realColorLZ=" << sumColor
                      << " colorSaving=" << (static_cast<long>(sumColor) - static_cast<long>(col.size()))
                      << " sidePatches=" << sidePatches
                      << " sideBytesEst=" << sideBytesEst
                      << " netSaving=" << (static_cast<long>(sumColor) - static_cast<long>(col.size()) - sideBytesEst)
                      << " codec=" << cc << std::endl;
        }
        // Single-stream "mode3" measurement: one 2-bit color stream with
        // Z-core=1 (recovered geometrically), deg3/4=2 (recovered by mode4-style
        // grouping; deg3/4 patches are never mutually adjacent -> one color is
        // enough), 0 elsewhere.  This is the realistic recoverable encoding.
        if (static_cast<int>(strictBlueLessColorPerRound_.size()) == numRounds) {
            std::vector<std::vector<uint8_t>> rr;
            std::vector<int> mm;
            rr.reserve(numRounds); mm.reserve(numRounds);
            for (int i = 0; i < numRounds; ++i) {
                const int er = numRounds - 1 - i;
                const auto& m2 = strictMode2SymbolsPerRound_[static_cast<size_t>(er)];
                const auto& blc = strictBlueLessColorPerRound_[static_cast<size_t>(er)];
                std::vector<uint8_t> sym(m2.size(), 0u);
                for (size_t k = 0; k < m2.size(); ++k) {
                    if (m2[k] == 1u) sym[k] = 1u;
                    else if (k < blc.size() && blc[k] != 0u) sym[k] = 2u;
                }
                rr.push_back(std::move(sym));
                mm.push_back(4);
            }
            int cc = -1;
            auto m3 = encodeColorRoundsGlobal(rr, mm, &cc);
            const long saving = static_cast<long>(sumColor) - static_cast<long>(m3.size());
            std::cerr << "[PCATMStrictPaper mode3-single] colorLZ=" << m3.size()
                      << " realColorLZ=" << sumColor
                      << " saving=" << saving
                      << " savingBPV=" << (saving * 8.0 / 48484.0)
                      << " codec=" << cc << std::endl;
        }
        // Direct measurement of the irreducible deg3/4 connectivity cost: encode
        // (globally, mode4) a color stream that keeps ONLY the blue-less (deg3/4)
        // patch colors.  Any correct all-mode2 scheme must re-transmit at least
        // this much side-info to recover those patches, since the blue stream
        // omits them entirely.  If S_deg34 ~ colorSaving, the net win is ~0.
        if (static_cast<int>(strictBlueLessColorPerRound_.size()) == numRounds) {
            std::vector<std::vector<uint8_t>> rr;
            std::vector<int> mm;
            rr.reserve(numRounds); mm.reserve(numRounds);
            for (int i = 0; i < numRounds; ++i) {
                const int er = numRounds - 1 - i;
                rr.push_back(strictBlueLessColorPerRound_[static_cast<size_t>(er)]);
                mm.push_back(4);
            }
            int cc = -1;
            auto deg = encodeColorRoundsGlobal(rr, mm, &cc);
            std::cerr << "[PCATMStrictPaper deg34-cost] S_deg34_bytes=" << deg.size()
                      << " allMode2ColorLZ_floor=" << 0
                      << " (compare to realColorLZ=" << sumColor
                      << ", all-mode2 color saving=25916-ish): side-info >= S_deg34"
                      << " codec=" << cc << std::endl;
        }
    }

    if (const char* env = std::getenv("PMSF_BPV_DIAG"); env && env[0] == '1') {
        std::cerr << "[PCATMStrictPaper bpv sv=" << schemeVersion << "] colorLZ=" << sumColor << " residualHuffman=" << sumGeom
                  << " valenceSide=" << sumValence << " isPolySide=" << sumIsPoly << " containerOverhead=" << lastContainerOverheadBytes_
                  << " totalBlob=" << blob.size() << " rawColorBits=" << rawColorBits << " colorCodec=" << colorCodec
                  << " removed=" << removedVertices << " rounds=" << numRounds << " residualMode="
                  << (useAdaptiveResidual ? "adaptive_bitplane" :
                          (useHuffmanResidual ?
                          "global_huffman" :
                          (useDenseRangeResidual ?
                               "global_dense_range" :
                               (useByteRangeResidual ?
                                    "global_varint_byte_range" :
                                    (useMeppAxisRangeResidual ?
                                         "global_mepp_axis_range" :
                                         (useSplitAxisVarintResidual ? "global_split_axis_varint_zlib" :
                                                                       (useAxisVarintResidual ? "global_axis_varint_zlib" : "global_varint_zlib")))))))
                  << " deg3=" << degree3 << " deg4=" << degree4 << " deg5=" << degree5 << " deg6=" << degree6 << " degOther=" << degreeOther
                  << " mode2Rounds=" << mode2Rounds << " mode4Rounds=" << mode4Rounds << " residualAlphabet=" << residualAlphabet.size() << std::endl;
    }

    return static_cast<int>(blob.size());
}

int PCATMStrictPMSFSymbolCollectOperator::exportToBuffer(char* /*buffer*/, bool /*enableQuantization*/, bool /*enableCompress*/) {
    throw std::runtime_error("[PCATMStrictPaper] non-RC export is not implemented.");
}
