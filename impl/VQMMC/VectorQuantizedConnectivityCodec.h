// SPDX-License-Identifier: Apache-2.0
//
// Framework-level physical codec for vector-quantized progressive residual
// streams. VQ-style algorithms should provide the symbolic queues and codebooks;
// this file handles adaptive bit streams, local topology streams, index streams,
// codebook quantization, and the 4-stream connectivity blob container.

#ifndef PMSF_COMPRESS_VECTOR_QUANTIZED_CONNECTIVITY_CODEC_H
#define PMSF_COMPRESS_VECTOR_QUANTIZED_CONNECTIVITY_CODEC_H

#include "Configuration.h"
#include "compress/ConnectivityBlobContainer.h"
#include "BFSVersionManager.h"
#include "BufferUtils.h"
#include "ContextPool.h"
#include "EntropyCodec.h"
#include "Facet.h"
#include "Global.h"
#include "Halfedge.h"
#include "Point.h"
#include "core/PaperProtocolHint.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂
#include "Vertex.h"
#include "VQMMC/common/VQMMCQuantization.h"
#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iostream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace pmsf {

enum class VQConnectivityStorageMode {
    Classic = 1,
    Valence = 3,
    Compact = 5,
    PaperFan = 6,
};

struct VQStreamSizes {
    std::size_t stream0 = 0;
    std::size_t stream1 = 0;
    std::size_t indices = 0;
    std::size_t codebook = 0;
    std::size_t blobBytes = 0;
};

struct VectorQuantizedDecodedBlob {
    int schemeVersion = kPaperBlobSchemeVersionCodec001;
    int numRounds = 0;
    std::vector<std::deque<char>> facetSyms;
    std::vector<std::deque<char>> edgeSyms;
    std::vector<std::deque<uint8_t>> faceCodes;
    std::vector<std::deque<uint8_t>> otherVals;
    std::vector<std::deque<uint8_t>> localEdgeSyms;
    std::vector<std::deque<uint8_t>> splitValences;
    std::vector<std::deque<uint8_t>> rootDiagSlots;
    std::vector<std::deque<uint16_t>> indices;
    std::vector<std::vector<MCGAL::Point>> codebooks;
    std::vector<uint8_t> absoluteGeometry;
};

struct VectorQuantizedSymbolRound {
    std::deque<char> facetSyms;
    std::deque<char> edgeSyms;
    std::deque<uint8_t> faceCodes;
    std::deque<uint8_t> otherVals;
    std::deque<uint8_t> localEdgeSyms;
    std::deque<uint8_t> splitValences;
    std::deque<uint8_t> splitOtherVals;
    std::deque<uint8_t> rootDiagSlots;
    std::deque<uint16_t> indices;
    std::vector<MCGAL::Facet*> splitFacets;
    std::vector<MCGAL::Point> codebook;
    unsigned facesVisited = 0;
    unsigned edgesVisited = 0;
    unsigned splittableFaces = 0;
};

inline std::vector<uint8_t> encodeVQAdaptiveCharBits(const std::deque<char>& syms) {
    std::vector<char> buf(std::max<std::size_t>(syms.size() / 4 + 1024, 4096));
    std::size_t off = 0;
    rangecoder rc;
    rc.p_data = buf.data();
    rc.p_dataOffset = &off;
    start_encoding(&rc, 0, 0);

    qsmodel model;
    initqsmodel(&model, 2, 10, 1 << 9, nullptr, 1);
    for (char s : syms) {
        const int sym = s & 1;
        int syfreq = 0;
        int ltfreq = 0;
        qsgetfreq(&model, sym, &syfreq, &ltfreq);
        encode_shift(&rc, syfreq, ltfreq, 10);
        qsupdate(&model, sym);
    }
    const std::size_t rcSize = done_encoding(&rc);
    deleteqsmodel(&model);

    std::vector<uint8_t> out(4);
    const int32_t n = static_cast<int32_t>(syms.size());
    std::memcpy(out.data(), &n, 4);
    out.insert(out.end(),
               reinterpret_cast<uint8_t*>(buf.data()),
               reinterpret_cast<uint8_t*>(buf.data()) + rcSize);
    return out;
}

inline std::deque<char> decodeVQAdaptiveCharBits(const uint8_t* data, std::size_t size) {
    std::deque<char> out;
    if (size < 4) return out;
    int32_t n = 0;
    std::memcpy(&n, data, 4);
    if (n <= 0) return out;

    rangecoder rc;
    rc.p_data = const_cast<char*>(reinterpret_cast<const char*>(data + 4));
    std::size_t off = 0;
    rc.p_dataOffset = &off;
    start_decoding(&rc);

    qsmodel model;
    initqsmodel(&model, 2, 10, 1 << 9, nullptr, 0);
    out.resize(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        const freq totf = decode_culshift(&rc, 10);
        const int sym = qsgetsym(&model, static_cast<int>(totf));
        int syfreq = 0;
        int ltfreq = 0;
        qsgetfreq(&model, sym, &syfreq, &ltfreq);
        decode_update_shift(&rc, syfreq, ltfreq, 10);
        qsupdate(&model, sym);
        out[static_cast<std::size_t>(i)] = static_cast<char>(sym);
    }
    deleteqsmodel(&model);
    return out;
}

inline std::vector<uint8_t> packVQBits(const std::deque<uint8_t>& syms) {
    std::vector<uint8_t> raw((syms.size() + 7) / 8, 0);
    for (std::size_t i = 0; i < syms.size(); ++i) {
        if (syms[i]) raw[i >> 3] |= static_cast<uint8_t>(1u << (i & 7));
    }
    return raw;
}

inline std::vector<uint8_t> packVQBits(const std::deque<char>& syms) {
    std::vector<uint8_t> raw((syms.size() + 7) / 8, 0);
    for (std::size_t i = 0; i < syms.size(); ++i) {
        if (syms[i] & 1) raw[i >> 3] |= static_cast<uint8_t>(1u << (i & 7));
    }
    return raw;
}

inline std::deque<char> unpackVQCharBits(const uint8_t* data,
                                         std::size_t size,
                                         int32_t n) {
    std::deque<char> out;
    if (n <= 0) return out;
    out.resize(static_cast<std::size_t>(n));
    for (int32_t i = 0; i < n; ++i) {
        const std::size_t byteId = static_cast<std::size_t>(i) >> 3;
        if (byteId < size) {
            out[static_cast<std::size_t>(i)] =
                static_cast<char>((data[byteId] >> (i & 7)) & 1);
        }
    }
    return out;
}

inline std::deque<uint8_t> unpackVQBits(const uint8_t* data,
                                        std::size_t size,
                                        int32_t n) {
    std::deque<uint8_t> out;
    if (n <= 0) return out;
    out.resize(static_cast<std::size_t>(n));
    for (int32_t i = 0; i < n; ++i) {
        const std::size_t byteId = static_cast<std::size_t>(i) >> 3;
        if (byteId < size) {
            out[static_cast<std::size_t>(i)] =
                static_cast<uint8_t>((data[byteId] >> (i & 7)) & 1);
        }
    }
    return out;
}

inline std::vector<uint8_t> encodeVQHybridCharBits(const std::deque<char>& syms) {
    std::vector<uint8_t> rcFull = encodeVQAdaptiveCharBits(syms);
    std::vector<uint8_t> rc;
    if (rcFull.size() > 4) rc.assign(rcFull.begin() + 4, rcFull.end());

    std::vector<uint8_t> raw = packVQBits(syms);
    std::vector<uint8_t> z;
    const bool zOk = zlibCompress(raw.data(), raw.size(), z) && z.size() < raw.size();

    const std::vector<uint8_t>* payload = &rc;
    uint8_t mode = 0;
    if (zOk && z.size() < payload->size()) {
        payload = &z;
        mode = 1;
    }
    if (raw.size() < payload->size()) {
        payload = &raw;
        mode = 2;
    }

    std::vector<uint8_t> out(5);
    const int32_t n = static_cast<int32_t>(syms.size());
    std::memcpy(out.data(), &n, 4);
    out[4] = mode;
    out.insert(out.end(), payload->begin(), payload->end());
    return out;
}

inline std::deque<char> decodeVQHybridCharBits(const uint8_t* data,
                                               std::size_t size) {
    std::deque<char> out;
    if (size < 5) return out;
    int32_t n = 0;
    std::memcpy(&n, data, 4);
    if (n <= 0) return out;
    const uint8_t mode = data[4];
    const uint8_t* payload = data + 5;
    const std::size_t payloadSize = size - 5;

    if (mode == 0) {
        std::vector<uint8_t> full(4 + payloadSize);
        std::memcpy(full.data(), &n, 4);
        if (payloadSize > 0) std::memcpy(full.data() + 4, payload, payloadSize);
        return decodeVQAdaptiveCharBits(full.data(), full.size());
    }
    if (mode == 1) {
        std::vector<uint8_t> raw;
        const std::size_t rawSize = (static_cast<std::size_t>(n) + 7) / 8;
        if (zlibDecompress(payload, payloadSize, raw, rawSize)) {
            return unpackVQCharBits(raw.data(), raw.size(), n);
        }
        return out;
    }
    if (mode == 2) return unpackVQCharBits(payload, payloadSize, n);
    return out;
}

inline std::vector<uint8_t> encodeVQFaceCodes(
    const std::deque<uint8_t>& codes,
    const std::deque<uint8_t>& otherVals) {
    std::vector<char> buf(std::max<std::size_t>(
        codes.size() * 2 + otherVals.size() + 1024, 4096));
    std::size_t off = 0;
    rangecoder rc;
    rc.p_data = buf.data();
    rc.p_dataOffset = &off;
    start_encoding(&rc, 0, 0);

    constexpr int kAlphabetSize = 8;
    constexpr int kLgTotf = 10;
    qsmodel models[4];
    for (auto& model : models) {
        initqsmodel(&model, kAlphabetSize, kLgTotf, 1 << 9, nullptr, 1);
    }
    qsmodel escapeModel;
    initqsmodel(&escapeModel, 256, 10, 1 << 9, nullptr, 1);

    int prev = -1;
    std::size_t otherCursor = 0;
    int32_t otherEmitted = 0;
    for (uint8_t rawSym : codes) {
        const int sym = std::min<int>(rawSym, 5);
        const int ctx = (prev == -1) ? 0
                      : (prev == 0)  ? 1
                      : (prev == 5)  ? 3
                                     : 2;
        int syfreq = 0;
        int ltfreq = 0;
        qsgetfreq(&models[ctx], sym, &syfreq, &ltfreq);
        encode_shift(&rc, syfreq, ltfreq, kLgTotf);
        qsupdate(&models[ctx], sym);

        if (sym == 5) {
            int esym = 0;
            if (otherCursor < otherVals.size()) {
                esym = static_cast<int>(otherVals[otherCursor++]);
            }
            int esyfreq = 0;
            int eltfreq = 0;
            qsgetfreq(&escapeModel, esym, &esyfreq, &eltfreq);
            encode_shift(&rc, esyfreq, eltfreq, 10);
            qsupdate(&escapeModel, esym);
            otherEmitted++;
        }
        prev = sym;
    }

    const std::size_t rcSize = done_encoding(&rc);
    for (auto& model : models) deleteqsmodel(&model);
    deleteqsmodel(&escapeModel);

    if (otherCursor != otherVals.size()) {
        std::cerr << "[VQ codec] WARN: unused other-valence escapes "
                  << otherCursor << "/" << otherVals.size() << std::endl;
    }

    std::vector<uint8_t> out(8);
    const int32_t n = static_cast<int32_t>(codes.size());
    std::memcpy(out.data(), &n, 4);
    std::memcpy(out.data() + 4, &otherEmitted, 4);
    out.insert(out.end(),
               reinterpret_cast<uint8_t*>(buf.data()),
               reinterpret_cast<uint8_t*>(buf.data()) + rcSize);
    return out;
}

inline void decodeVQFaceCodes(const uint8_t* data,
                              std::size_t size,
                              std::deque<uint8_t>& codes,
                              std::deque<uint8_t>& otherVals) {
    codes.clear();
    otherVals.clear();
    if (size < 8) return;
    int32_t n = 0;
    int32_t numOther = 0;
    std::memcpy(&n, data, 4);
    std::memcpy(&numOther, data + 4, 4);
    if (n <= 0) return;

    rangecoder rc;
    rc.p_data = const_cast<char*>(reinterpret_cast<const char*>(data + 8));
    std::size_t off = 0;
    rc.p_dataOffset = &off;
    start_decoding(&rc);

    constexpr int kAlphabetSize = 8;
    constexpr int kLgTotf = 10;
    qsmodel models[4];
    for (auto& model : models) {
        initqsmodel(&model, kAlphabetSize, kLgTotf, 1 << 9, nullptr, 0);
    }
    qsmodel escapeModel;
    initqsmodel(&escapeModel, 256, 10, 1 << 9, nullptr, 0);

    codes.resize(static_cast<std::size_t>(n));
    int prev = -1;
    int otherSeen = 0;
    for (int i = 0; i < n; ++i) {
        const int ctx = (prev == -1) ? 0
                      : (prev == 0)  ? 1
                      : (prev == 5)  ? 3
                                     : 2;
        int syfreq = 0;
        int ltfreq = 0;
        const freq totf = decode_culshift(&rc, kLgTotf);
        int sym = qsgetsym(&models[ctx], static_cast<int>(totf));
        qsgetfreq(&models[ctx], sym, &syfreq, &ltfreq);
        decode_update_shift(&rc, syfreq, ltfreq, kLgTotf);
        qsupdate(&models[ctx], sym);
        sym = std::min(sym, 5);
        codes[static_cast<std::size_t>(i)] = static_cast<uint8_t>(sym);

        if (sym == 5) {
            int esyfreq = 0;
            int eltfreq = 0;
            const freq etotf = decode_culshift(&rc, 10);
            const int esym = qsgetsym(&escapeModel, static_cast<int>(etotf));
            qsgetfreq(&escapeModel, esym, &esyfreq, &eltfreq);
            decode_update_shift(&rc, esyfreq, eltfreq, 10);
            qsupdate(&escapeModel, esym);
            otherVals.push_back(static_cast<uint8_t>(esym));
            otherSeen++;
        }
        prev = sym;
    }
    done_decoding(&rc);
    for (auto& model : models) deleteqsmodel(&model);
    deleteqsmodel(&escapeModel);

    if (otherSeen != numOther) {
        std::cerr << "[VQ codec] WARN: other-valence count mismatch header="
                  << numOther << " decoded=" << otherSeen << std::endl;
    }
}

inline std::vector<uint8_t> encodeVQLocalEdgeMasksPayload(
    const std::deque<uint8_t>& syms,
    const std::deque<uint8_t>& faceCodes) {
    const std::size_t numMasks = (syms.size() + 2) / 3;
    std::vector<char> buf(std::max<std::size_t>(numMasks * 2 + 1024, 4096));
    std::size_t off = 0;
    rangecoder rc;
    rc.p_data = buf.data();
    rc.p_dataOffset = &off;
    start_encoding(&rc, 0, 0);

    qsmodel models[4];
    for (auto& model : models) initqsmodel(&model, 8, 10, 1 << 9, nullptr, 1);

    int prevMask = 0;
    for (std::size_t i = 0; i < numMasks; ++i) {
        int mask = 0;
        for (int b = 0; b < 3; ++b) {
            const std::size_t pos = i * 3 + static_cast<std::size_t>(b);
            if (pos < syms.size() && syms[pos]) mask |= (1 << b);
        }
        const int faceCode = i < faceCodes.size() ? static_cast<int>(faceCodes[i]) : 0;
        const int ctx = (faceCode == 0 ? 0 : 1) + (prevMask == 0 ? 0 : 2);
        int syfreq = 0;
        int ltfreq = 0;
        qsgetfreq(&models[ctx], mask, &syfreq, &ltfreq);
        encode_shift(&rc, syfreq, ltfreq, 10);
        qsupdate(&models[ctx], mask);
        prevMask = mask;
    }

    const std::size_t rcSize = done_encoding(&rc);
    for (auto& model : models) deleteqsmodel(&model);

    std::vector<uint8_t> out;
    out.insert(out.end(),
               reinterpret_cast<uint8_t*>(buf.data()),
               reinterpret_cast<uint8_t*>(buf.data()) + rcSize);
    return out;
}

inline std::deque<uint8_t> decodeVQLocalEdgeMasksPayload(
    const uint8_t* data,
    std::size_t size,
    int32_t n,
    const std::deque<uint8_t>& faceCodes) {
    std::deque<uint8_t> out;
    if (n <= 0) return out;
    const std::size_t numMasks = (static_cast<std::size_t>(n) + 2) / 3;

    rangecoder rc;
    rc.p_data = const_cast<char*>(reinterpret_cast<const char*>(data));
    std::size_t off = 0;
    rc.p_dataOffset = &off;
    start_decoding(&rc);

    qsmodel models[4];
    for (auto& model : models) initqsmodel(&model, 8, 10, 1 << 9, nullptr, 0);

    out.resize(static_cast<std::size_t>(n));
    int prevMask = 0;
    for (std::size_t i = 0; i < numMasks; ++i) {
        const int faceCode = i < faceCodes.size() ? static_cast<int>(faceCodes[i]) : 0;
        const int ctx = (faceCode == 0 ? 0 : 1) + (prevMask == 0 ? 0 : 2);
        int syfreq = 0;
        int ltfreq = 0;
        const freq totf = decode_culshift(&rc, 10);
        const int mask = qsgetsym(&models[ctx], static_cast<int>(totf));
        qsgetfreq(&models[ctx], mask, &syfreq, &ltfreq);
        decode_update_shift(&rc, syfreq, ltfreq, 10);
        qsupdate(&models[ctx], mask);
        for (int b = 0; b < 3; ++b) {
            const std::size_t pos = i * 3 + static_cast<std::size_t>(b);
            if (pos < out.size()) out[pos] = static_cast<uint8_t>((mask >> b) & 1);
        }
        prevMask = mask;
    }
    done_decoding(&rc);
    for (auto& model : models) deleteqsmodel(&model);
    (void)size;
    return out;
}

inline std::vector<uint8_t> encodeVQLocalEdgeHybrid(
    const std::deque<uint8_t>& syms,
    const std::deque<uint8_t>& faceCodes) {
    std::vector<uint8_t> rc = encodeVQLocalEdgeMasksPayload(syms, faceCodes);
    std::vector<uint8_t> raw = packVQBits(syms);
    std::vector<uint8_t> z;
    const bool zOk = zlibCompress(raw.data(), raw.size(), z) && z.size() < raw.size();

    const std::vector<uint8_t>* payload = &rc;
    uint8_t mode = 0;
    if (zOk && z.size() < payload->size()) {
        payload = &z;
        mode = 1;
    }
    if (raw.size() < payload->size()) {
        payload = &raw;
        mode = 2;
    }

    std::vector<uint8_t> out(5);
    const int32_t n = static_cast<int32_t>(syms.size());
    std::memcpy(out.data(), &n, 4);
    out[4] = mode;
    out.insert(out.end(), payload->begin(), payload->end());
    return out;
}

inline std::deque<uint8_t> decodeVQLocalEdgeMasks(
    const uint8_t* data,
    std::size_t size,
    const std::deque<uint8_t>& faceCodes,
    bool hybrid) {
    std::deque<uint8_t> out;
    if (size < 4) return out;
    int32_t n = 0;
    std::memcpy(&n, data, 4);
    if (n <= 0) return out;

    if (!hybrid) return decodeVQLocalEdgeMasksPayload(data + 4, size - 4, n, faceCodes);
    if (size < 5) return out;

    const uint8_t mode = data[4];
    const uint8_t* payload = data + 5;
    const std::size_t payloadSize = size - 5;
    if (mode == 0) return decodeVQLocalEdgeMasksPayload(payload, payloadSize, n, faceCodes);
    if (mode == 1) {
        std::vector<uint8_t> raw;
        const std::size_t rawSize = (static_cast<std::size_t>(n) + 7) / 8;
        if (zlibDecompress(payload, payloadSize, raw, rawSize)) {
            return unpackVQBits(raw.data(), raw.size(), n);
        }
        return out;
    }
    if (mode == 2) return unpackVQBits(payload, payloadSize, n);
    return out;
}

inline std::vector<uint8_t> encodeVQSplitTopology(
    const std::deque<uint8_t>& valences,
    const std::deque<uint8_t>& otherVals,
    const std::deque<uint8_t>& rootSlots) {
    std::vector<char> buf(std::max<std::size_t>(
        valences.size() * 3 + otherVals.size() + 1024, 4096));
    std::size_t off = 0;
    rangecoder rc;
    rc.p_data = buf.data();
    rc.p_dataOffset = &off;
    start_encoding(&rc, 0, 0);

    qsmodel valModels[4];
    for (auto& model : valModels) initqsmodel(&model, 8, 10, 1 << 9, nullptr, 1);
    qsmodel slotModels[4];
    for (auto& model : slotModels) initqsmodel(&model, 4, 10, 1 << 9, nullptr, 1);
    qsmodel escapeModel;
    initqsmodel(&escapeModel, 256, 10, 1 << 9, nullptr, 1);

    int prevVal = -1;
    std::size_t otherCursor = 0;
    std::size_t slotCursor = 0;
    int32_t otherEmitted = 0;
    int32_t slotEmitted = 0;

    for (uint8_t rawCode : valences) {
        const int code = std::min<int>(rawCode, 4);
        const int ctx = (prevVal == -1) ? 0
                      : (prevVal == 0)  ? 1
                      : (prevVal == 4)  ? 3
                                     : 2;
        int syfreq = 0;
        int ltfreq = 0;
        qsgetfreq(&valModels[ctx], code, &syfreq, &ltfreq);
        encode_shift(&rc, syfreq, ltfreq, 10);
        qsupdate(&valModels[ctx], code);

        int actualValence = code + 3;
        if (code == 4) {
            int esym = 0;
            if (otherCursor < otherVals.size()) {
                esym = static_cast<int>(otherVals[otherCursor++]);
            }
            actualValence = esym;
            int esyfreq = 0;
            int eltfreq = 0;
            qsgetfreq(&escapeModel, esym, &esyfreq, &eltfreq);
            encode_shift(&rc, esyfreq, eltfreq, 10);
            qsupdate(&escapeModel, esym);
            otherEmitted++;
        }

        if (actualValence >= 4) {
            int slot = 3;
            if (slotCursor < rootSlots.size()) {
                slot = std::min<int>(rootSlots[slotCursor++], 3);
            }
            const int slotCtx = std::min(std::max(actualValence - 4, 0), 3);
            int sy = 0;
            int lt = 0;
            qsgetfreq(&slotModels[slotCtx], slot, &sy, &lt);
            encode_shift(&rc, sy, lt, 10);
            qsupdate(&slotModels[slotCtx], slot);
            slotEmitted++;
        }
        prevVal = code;
    }

    const std::size_t rcSize = done_encoding(&rc);
    for (auto& model : valModels) deleteqsmodel(&model);
    for (auto& model : slotModels) deleteqsmodel(&model);
    deleteqsmodel(&escapeModel);

    if (otherCursor != otherVals.size() || slotCursor != rootSlots.size()) {
        std::cerr << "[VQ codec] WARN topology cursor mismatch other="
                  << otherCursor << "/" << otherVals.size()
                  << " slots=" << slotCursor << "/" << rootSlots.size()
                  << std::endl;
    }

    std::vector<uint8_t> out(12);
    const int32_t n = static_cast<int32_t>(valences.size());
    std::memcpy(out.data(), &n, 4);
    std::memcpy(out.data() + 4, &otherEmitted, 4);
    std::memcpy(out.data() + 8, &slotEmitted, 4);
    out.insert(out.end(),
               reinterpret_cast<uint8_t*>(buf.data()),
               reinterpret_cast<uint8_t*>(buf.data()) + rcSize);
    return out;
}

inline void decodeVQSplitTopology(const uint8_t* data,
                                  std::size_t size,
                                  std::deque<uint8_t>& valences,
                                  std::deque<uint8_t>& rootSlots) {
    valences.clear();
    rootSlots.clear();
    if (size < 12) return;
    int32_t n = 0;
    int32_t numOther = 0;
    int32_t numSlots = 0;
    std::memcpy(&n, data, 4);
    std::memcpy(&numOther, data + 4, 4);
    std::memcpy(&numSlots, data + 8, 4);
    if (n <= 0) return;

    rangecoder rc;
    rc.p_data = const_cast<char*>(reinterpret_cast<const char*>(data + 12));
    std::size_t off = 0;
    rc.p_dataOffset = &off;
    start_decoding(&rc);

    qsmodel valModels[4];
    for (auto& model : valModels) initqsmodel(&model, 8, 10, 1 << 9, nullptr, 0);
    qsmodel slotModels[4];
    for (auto& model : slotModels) initqsmodel(&model, 4, 10, 1 << 9, nullptr, 0);
    qsmodel escapeModel;
    initqsmodel(&escapeModel, 256, 10, 1 << 9, nullptr, 0);

    valences.resize(static_cast<std::size_t>(n));
    int prevVal = -1;
    int otherSeen = 0;
    int slotSeen = 0;
    for (int i = 0; i < n; ++i) {
        const int ctx = (prevVal == -1) ? 0
                      : (prevVal == 0)  ? 1
                      : (prevVal == 4)  ? 3
                                     : 2;
        int syfreq = 0;
        int ltfreq = 0;
        const freq totf = decode_culshift(&rc, 10);
        int code = qsgetsym(&valModels[ctx], static_cast<int>(totf));
        qsgetfreq(&valModels[ctx], code, &syfreq, &ltfreq);
        decode_update_shift(&rc, syfreq, ltfreq, 10);
        qsupdate(&valModels[ctx], code);
        code = std::min(code, 4);

        int actualValence = code + 3;
        if (code == 4) {
            int esyfreq = 0;
            int eltfreq = 0;
            const freq etotf = decode_culshift(&rc, 10);
            const int esym = qsgetsym(&escapeModel, static_cast<int>(etotf));
            qsgetfreq(&escapeModel, esym, &esyfreq, &eltfreq);
            decode_update_shift(&rc, esyfreq, eltfreq, 10);
            qsupdate(&escapeModel, esym);
            actualValence = esym;
            otherSeen++;
        }
        valences[static_cast<std::size_t>(i)] =
            static_cast<uint8_t>(std::clamp(actualValence, 0, 255));

        if (actualValence >= 4) {
            const int slotCtx = std::min(std::max(actualValence - 4, 0), 3);
            int sf = 0;
            int lf = 0;
            const freq stotf = decode_culshift(&rc, 10);
            const int slot = qsgetsym(&slotModels[slotCtx], static_cast<int>(stotf));
            qsgetfreq(&slotModels[slotCtx], slot, &sf, &lf);
            decode_update_shift(&rc, sf, lf, 10);
            qsupdate(&slotModels[slotCtx], slot);
            rootSlots.push_back(static_cast<uint8_t>(std::min(slot, 3)));
            slotSeen++;
        }
        prevVal = code;
    }

    done_decoding(&rc);
    for (auto& model : valModels) deleteqsmodel(&model);
    for (auto& model : slotModels) deleteqsmodel(&model);
    deleteqsmodel(&escapeModel);

    if (otherSeen != numOther || slotSeen != numSlots) {
        std::cerr << "[VQ codec] WARN topology count mismatch other="
                  << otherSeen << "/" << numOther
                  << " slots=" << slotSeen << "/" << numSlots << std::endl;
    }
}

inline int vqIndexBitsForCodebookSize(int codebookSize) {
    int idxBits = 0;
    uint32_t v = static_cast<uint32_t>(std::max(codebookSize - 1, 0));
    while (v) {
        v >>= 1;
        ++idxBits;
    }
    return std::max(idxBits, 1);
}

inline VectorQuantizedSymbolRound collectVectorQuantizedConnectivitySymbols(
    int meshId,
    MCGAL::Halfedge* seed,
    const std::unordered_map<MCGAL::Facet*, uint16_t>& facetToIndex,
    const std::vector<MCGAL::Point>& codebook) {
    VectorQuantizedSymbolRound round;
    round.codebook = codebook;

    std::queue<int> gateQueue;
    if (seed != nullptr) gateQueue.push(seed->poolId());
    const int currentVersion = MCGAL::BFSVersionManager::current_version++;

    while (!gateQueue.empty()) {
        const int hid = gateQueue.front();
        gateQueue.pop();
        MCGAL::Halfedge* h =
            MCGAL::contextPool.getHalfedgeByIndexInSubPool(meshId, hid);
        if (h == nullptr || h->isVisited(currentVersion) || h->isRemoved()) {
            continue;
        }
        h->setVisited(currentVersion);
        round.edgesVisited++;
        round.edgeSyms.push_back(h->isAdded() ? 1 : 0);

        MCGAL::Facet* f = h->face();
        if (f != nullptr && !f->isVisited(currentVersion)) {
            f->setVisited(currentVersion);
            round.facesVisited++;
            const bool isSplit = f->isSplittable();
            round.facetSyms.push_back(isSplit ? 1 : 0);

            if (isSplit) {
                const int polyValence = f->getPolyValence();
                if (polyValence >= 3 && polyValence <= 6) {
                    round.faceCodes.push_back(static_cast<uint8_t>(polyValence - 2));
                } else {
                    round.faceCodes.push_back(5);
                    round.otherVals.push_back(
                        static_cast<uint8_t>(std::clamp(polyValence, 0, 255)));
                }
            } else {
                round.faceCodes.push_back(0);
            }

            MCGAL::Halfedge* hLocal = h;
            for (int k = 0; k < 3 && hLocal != nullptr; ++k) {
                round.localEdgeSyms.push_back(hLocal->isAdded() ? 1 : 0);
                hLocal = hLocal->next();
            }

            if (isSplit) {
                round.splitFacets.push_back(f);
                round.splittableFaces++;
                const int polyValence = f->getPolyValence();
                if (polyValence >= 3 && polyValence <= 6) {
                    round.splitValences.push_back(static_cast<uint8_t>(polyValence - 3));
                } else {
                    round.splitValences.push_back(4);
                    round.splitOtherVals.push_back(
                        static_cast<uint8_t>(std::clamp(polyValence, 0, 255)));
                }
                if (polyValence >= 4) {
                    uint8_t diagSlot = 3;
                    MCGAL::Halfedge* hDiag = h;
                    for (int k = 0; k < 3 && hDiag != nullptr; ++k) {
                        if (hDiag->isAdded()) {
                            diagSlot = static_cast<uint8_t>(k);
                            break;
                        }
                        hDiag = hDiag->next();
                    }
                    round.rootDiagSlots.push_back(diagSlot);
                }

                auto it = facetToIndex.find(f);
                round.indices.push_back(
                    it != facetToIndex.end() ? it->second : static_cast<uint16_t>(0));
            }
        }

        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp != nullptr && !hOpp->isVisited(currentVersion)) {
                gateQueue.push(hOpp->poolId());
            }
            if (hIt != nullptr && !hIt->isVisited(currentVersion)) {
                gateQueue.push(hIt->poolId());
            }
            hIt = hIt->next();
        } while (hIt != h);
    }

    return round;
}

inline int writeLegacyVectorQuantizedConnectivityBuffer(
    char* buffer,
    const std::vector<std::deque<char>>& facetSymbolQueues,
    const std::vector<std::deque<char>>& edgeSymbolQueues,
    const std::vector<std::deque<uint16_t>>& indexQueues,
    const std::vector<std::vector<MCGAL::Point>>& codebookQueues,
    bool absoluteGeometry) {
    std::size_t offset = 0;

    std::vector<char> facetBuffer(BUFFER_SIZE / 6, 0);
    std::vector<char> edgeBuffer(BUFFER_SIZE / 6, 0);
    std::size_t facetOffset = 0;
    std::size_t edgeOffset = 0;

    for (int i = static_cast<int>(facetSymbolQueues.size()) - 1; i >= 0; --i) {
        unsigned bitOffset = 0;
        for (char sym : facetSymbolQueues[static_cast<std::size_t>(i)]) {
            writeBitsOff(static_cast<uint32_t>(sym), 1, facetBuffer.data(),
                         bitOffset, facetOffset);
        }
        facetOffset++;
    }

    for (int i = static_cast<int>(edgeSymbolQueues.size()) - 1; i >= 0; --i) {
        unsigned bitOffset = 0;
        for (char sym : edgeSymbolQueues[static_cast<std::size_t>(i)]) {
            writeBitsOff(static_cast<uint32_t>(sym), 1, edgeBuffer.data(),
                         bitOffset, edgeOffset);
        }
        edgeOffset++;
    }

    std::vector<char> vqBuffer(BUFFER_SIZE / 3, 0);
    std::size_t vqOffset = 0;
    const int codebookQuantBits = vqmmcConfiguredCodebookQuantBits();
    const uint32_t codebookQuantMax = (1u << codebookQuantBits) - 1u;

    for (int i = static_cast<int>(codebookQueues.size()) - 1; i >= 0; --i) {
        const auto& codebook = codebookQueues[static_cast<std::size_t>(i)];
        const auto& indices = indexQueues[static_cast<std::size_t>(i)];
        const int cbSize = static_cast<int>(codebook.size());
        writeInt(vqBuffer.data(), vqOffset, absoluteGeometry ? -cbSize : cbSize);
        writeInt(vqBuffer.data(), vqOffset, static_cast<int>(indices.size()));
        if (cbSize == 0) continue;

        float cMin[3] = {
            std::numeric_limits<float>::max(),
            std::numeric_limits<float>::max(),
            std::numeric_limits<float>::max()};
        float cMax[3] = {
            -std::numeric_limits<float>::max(),
            -std::numeric_limits<float>::max(),
            -std::numeric_limits<float>::max()};
        for (const auto& point : codebook) {
            const float v[3] = {
                static_cast<float>(point.x()),
                static_cast<float>(point.y()),
                static_cast<float>(point.z())};
            for (int k = 0; k < 3; ++k) {
                cMin[k] = std::min(cMin[k], v[k]);
                cMax[k] = std::max(cMax[k], v[k]);
            }
        }

        float cStep[3];
        for (int k = 0; k < 3; ++k) {
            float range = cMax[k] - cMin[k];
            if (range <= 0.0f) range = 1e-9f;
            cStep[k] = range / static_cast<float>(codebookQuantMax);
        }
        writeChar(vqBuffer.data(), vqOffset,
                  static_cast<unsigned char>(codebookQuantBits));
        for (float value : cMin) writeFloat(vqBuffer.data(), vqOffset, value);
        for (float value : cStep) writeFloat(vqBuffer.data(), vqOffset, value);

        unsigned cbBitOffset = 0;
        for (const auto& point : codebook) {
            const float v[3] = {
                static_cast<float>(point.x()),
                static_cast<float>(point.y()),
                static_cast<float>(point.z())};
            for (int k = 0; k < 3; ++k) {
                int q = static_cast<int>(std::round((v[k] - cMin[k]) / cStep[k]));
                q = std::clamp(q, 0, static_cast<int>(codebookQuantMax));
                writeBitsOff(static_cast<uint32_t>(q), codebookQuantBits,
                             vqBuffer.data(), cbBitOffset, vqOffset);
            }
        }
        if (cbBitOffset != 0) vqOffset++;

        const int idxBits = vqIndexBitsForCodebookSize(cbSize);
        writeChar(vqBuffer.data(), vqOffset, static_cast<unsigned char>(idxBits));
        unsigned idxBitOffset = 0;
        for (uint16_t index : indices) {
            writeBitsOff(static_cast<uint32_t>(index), idxBits, vqBuffer.data(),
                         idxBitOffset, vqOffset);
        }
        if (idxBitOffset != 0) vqOffset++;
    }

    auto writeBlock = [&](const char* data, int origSize) {
        std::vector<uint8_t> compressed;
        int compSize = 0;
        if (origSize > 0 &&
            zlibCompress(reinterpret_cast<const uint8_t*>(data),
                         static_cast<std::size_t>(origSize), compressed) &&
            static_cast<int>(compressed.size()) < origSize) {
            compSize = static_cast<int>(compressed.size());
        }
        writeInt(buffer, offset, origSize);
        writeInt(buffer, offset, compSize);
        if (origSize > 0) {
            if (compSize > 0) {
                std::memcpy(buffer + offset, compressed.data(),
                            static_cast<std::size_t>(compSize));
                offset += static_cast<std::size_t>(compSize);
            } else {
                std::memcpy(buffer + offset, data, static_cast<std::size_t>(origSize));
                offset += static_cast<std::size_t>(origSize);
            }
        }
        return compSize > 0 ? compSize : origSize;
    };

    const int facetEmit = writeBlock(facetBuffer.data(), static_cast<int>(facetOffset));
    const int edgeEmit = writeBlock(edgeBuffer.data(), static_cast<int>(edgeOffset));
    const int vqEmit = writeBlock(vqBuffer.data(), static_cast<int>(vqOffset));

    std::cout << "[VQMMC serialize] facetOrig=" << facetOffset
              << " facetComp=" << facetEmit
              << " edgeOrig=" << edgeOffset
              << " edgeComp=" << edgeEmit
              << " vqOrig=" << vqOffset
              << " vqComp=" << vqEmit
              << " total=" << offset << std::endl;

    return static_cast<int>(offset);
}

inline std::vector<uint8_t> encodeVQIndices(const std::deque<uint16_t>& indices,
                                            int codebookSize) {
    if (indices.empty() || codebookSize <= 1) {
        std::vector<uint8_t> out(12, 0);
        const int32_t k = static_cast<int32_t>(codebookSize);
        const int32_t n = static_cast<int32_t>(indices.size());
        const int32_t b = 1;
        std::memcpy(out.data(), &k, 4);
        std::memcpy(out.data() + 4, &n, 4);
        std::memcpy(out.data() + 8, &b, 4);
        return out;
    }

    const int idxBits = vqIndexBitsForCodebookSize(codebookSize);
    std::vector<char> buf(std::max<std::size_t>(
        indices.size() * static_cast<std::size_t>(idxBits) / 4 + 4096, 16384));
    std::size_t off = 0;
    rangecoder rc;
    rc.p_data = buf.data();
    rc.p_dataOffset = &off;
    start_encoding(&rc, 0, 0);

    std::vector<qsmodel> models(static_cast<std::size_t>(idxBits));
    for (int b = 0; b < idxBits; ++b) {
        initqsmodel(&models[static_cast<std::size_t>(b)], 2, 10, 1 << 9, nullptr, 1);
    }

    for (uint16_t idx : indices) {
        for (int b = 0; b < idxBits; ++b) {
            const int bit = (idx >> (idxBits - 1 - b)) & 1;
            int syfreq = 0;
            int ltfreq = 0;
            qsgetfreq(&models[static_cast<std::size_t>(b)], bit, &syfreq, &ltfreq);
            encode_shift(&rc, syfreq, ltfreq, 10);
            qsupdate(&models[static_cast<std::size_t>(b)], bit);
        }
    }
    const std::size_t rcSize = done_encoding(&rc);
    for (int b = 0; b < idxBits; ++b) deleteqsmodel(&models[static_cast<std::size_t>(b)]);

    std::vector<uint8_t> out(12);
    const int32_t k = static_cast<int32_t>(codebookSize);
    const int32_t n = static_cast<int32_t>(indices.size());
    const int32_t bits = static_cast<int32_t>(idxBits);
    std::memcpy(out.data(), &k, 4);
    std::memcpy(out.data() + 4, &n, 4);
    std::memcpy(out.data() + 8, &bits, 4);
    out.insert(out.end(),
               reinterpret_cast<uint8_t*>(buf.data()),
               reinterpret_cast<uint8_t*>(buf.data()) + rcSize);
    return out;
}

inline std::deque<uint16_t> decodeVQIndices(const uint8_t* data, std::size_t size) {
    std::deque<uint16_t> out;
    if (size < 12) return out;
    int32_t k = 0;
    int32_t n = 0;
    int32_t idxBits = 0;
    std::memcpy(&k, data, 4);
    std::memcpy(&n, data + 4, 4);
    std::memcpy(&idxBits, data + 8, 4);
    if (n <= 0 || k <= 1) {
        if (n > 0) out.resize(static_cast<std::size_t>(n), 0);
        return out;
    }

    rangecoder rc;
    rc.p_data = const_cast<char*>(reinterpret_cast<const char*>(data + 12));
    std::size_t off = 0;
    rc.p_dataOffset = &off;
    start_decoding(&rc);

    std::vector<qsmodel> models(static_cast<std::size_t>(idxBits));
    for (int b = 0; b < idxBits; ++b) {
        initqsmodel(&models[static_cast<std::size_t>(b)], 2, 10, 1 << 9, nullptr, 0);
    }

    out.resize(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        uint16_t idx = 0;
        for (int b = 0; b < idxBits; ++b) {
            const freq totf = decode_culshift(&rc, 10);
            const int bit = qsgetsym(&models[static_cast<std::size_t>(b)],
                                     static_cast<int>(totf));
            int syfreq = 0;
            int ltfreq = 0;
            qsgetfreq(&models[static_cast<std::size_t>(b)], bit, &syfreq, &ltfreq);
            decode_update_shift(&rc, syfreq, ltfreq, 10);
            qsupdate(&models[static_cast<std::size_t>(b)], bit);
            idx = static_cast<uint16_t>((idx << 1) | bit);
        }
        out[static_cast<std::size_t>(i)] = idx;
    }
    for (int b = 0; b < idxBits; ++b) deleteqsmodel(&models[static_cast<std::size_t>(b)]);
    return out;
}

inline std::vector<uint8_t> encodeVQCodebook(const std::vector<MCGAL::Point>& codebook,
                                             bool predictiveGeometry) {
    const int quantBits = vqmmcConfiguredCodebookQuantBits();
    const uint32_t quantMax = (1u << quantBits) - 1u;

    const int cbSize = static_cast<int>(codebook.size());
    std::vector<uint8_t> out(24, 0);
    if (cbSize == 0) {
        std::memcpy(out.data() + 20, &cbSize, 4);
        return out;
    }

    float cMin[3] = {1e30f, 1e30f, 1e30f};
    float cMax[3] = {-1e30f, -1e30f, -1e30f};
    for (const auto& p : codebook) {
        const float v[3] = {static_cast<float>(p.x()),
                            static_cast<float>(p.y()),
                            static_cast<float>(p.z())};
        for (int k = 0; k < 3; ++k) {
            if (v[k] < cMin[k]) cMin[k] = v[k];
            if (v[k] > cMax[k]) cMax[k] = v[k];
        }
    }
    float cStep[3];
    for (int k = 0; k < 3; ++k) {
        const float range = cMax[k] - cMin[k];
        cStep[k] = (range > 0.0f) ? range / static_cast<float>(quantMax) : 1e-9f;
    }
    std::memcpy(out.data(), cMin, 12);
    std::memcpy(out.data() + 12, cStep, 12);

    std::vector<char> buf(std::max<std::size_t>(
        static_cast<std::size_t>(cbSize) * 3 *
                static_cast<std::size_t>((quantBits + 7) / 8) +
            4096,
        32768));
    std::size_t off = 0;
    rangecoder rc;
    rc.p_data = buf.data();
    rc.p_dataOffset = &off;
    start_encoding(&rc, 0, 0);

    auto encQuantizedCoord = [&](float val, float min, float step) {
        int q = static_cast<int>(std::round((val - min) / step));
        q = std::clamp(q, 0, static_cast<int>(quantMax));
        encode_shift(&rc, 1, static_cast<freq>(q), static_cast<freq>(quantBits));
    };

    for (const auto& p : codebook) {
        const float coords[3] = {p.x(), p.y(), p.z()};
        for (int k = 0; k < 3; ++k) encQuantizedCoord(coords[k], cMin[k], cStep[k]);
    }

    const std::size_t rcSize = done_encoding(&rc);
    const int32_t storedCbSize = predictiveGeometry ? cbSize : -cbSize;
    const int32_t storedQuantBits = quantBits;
    const std::size_t base = out.size();
    out.resize(base + 8 + rcSize);
    std::memcpy(out.data() + base, &storedCbSize, 4);
    std::memcpy(out.data() + base + 4, &storedQuantBits, 4);
    if (rcSize > 0) std::memcpy(out.data() + base + 8, buf.data(), rcSize);
    return out;
}

inline std::vector<MCGAL::Point> decodeVQCodebook(const uint8_t* data,
                                                  std::size_t size) {
    if (size < 24) return {};
    float cMin[3];
    float cStep[3];
    std::memcpy(cMin, data, 12);
    std::memcpy(cStep, data + 12, 12);
    if (size < 32) return {};
    int32_t cbSize = 0;
    int32_t quantBits = 0;
    std::memcpy(&cbSize, data + 24, 4);
    std::memcpy(&quantBits, data + 28, 4);
    if (cbSize < 0) cbSize = -cbSize;
    if (cbSize <= 0) return {};

    rangecoder rc;
    rc.p_data = const_cast<char*>(reinterpret_cast<const char*>(data + 32));
    std::size_t off = 0;
    rc.p_dataOffset = &off;
    start_decoding(&rc);

    auto dequant = [&](float min, float step, int bits) -> float {
        bits = std::max(2, std::min(16, bits));
        const freq totf = decode_culshift(&rc, bits);
        decode_update_shift(&rc, 1, totf, bits);
        return min + static_cast<float>(totf) * step;
    };

    std::vector<MCGAL::Point> codebook(static_cast<std::size_t>(cbSize));
    for (int i = 0; i < cbSize; ++i) {
        float coord[3];
        for (int k = 0; k < 3; ++k) coord[k] = dequant(cMin[k], cStep[k], quantBits);
        codebook[static_cast<std::size_t>(i)] =
            MCGAL::Point(coord[0], coord[1], coord[2]);
    }
    return codebook;
}

inline bool decodeVQCodebookAbsoluteMode(const uint8_t* data, std::size_t size) {
    if (size < 28) return false;
    int32_t cbSize = 0;
    std::memcpy(&cbSize, data + 24, 4);
    return cbSize < 0;
}

inline std::vector<uint8_t> writeVectorQuantizedConnectivityBlob(
    int blobLayout,
    VQConnectivityStorageMode mode,
    const std::vector<std::deque<char>>& facetSymbolQueues,
    const std::vector<std::deque<char>>& edgeSymbolQueues,
    const std::vector<std::deque<uint8_t>>& faceCodeQueues,
    const std::vector<std::deque<uint8_t>>& otherValQueues,
    const std::vector<std::deque<uint8_t>>& localEdgeSymbolQueues,
    const std::vector<std::deque<uint8_t>>& splitValenceQueues,
    const std::vector<std::deque<uint8_t>>& splitOtherValQueues,
    const std::vector<std::deque<uint8_t>>& rootDiagSlotQueues,
    const std::vector<std::deque<uint16_t>>& vqIndexQueues,
    const std::vector<std::vector<MCGAL::Point>>& codebookQueues,
    bool predictiveGeometry,
    VQStreamSizes* sizes = nullptr) {
    const int numRounds = static_cast<int>(facetSymbolQueues.size());
    pmsf::ConnectivityBlobBuilder cbb(
        pmsf::vectorQuantizedConnectivityBlobSpec(
            blobLayout,
            static_cast<std::int32_t>(mode),
            numRounds));

    VQStreamSizes localSizes;
    for (int i = 0; i < numRounds; ++i) {
        const int er = numRounds - 1 - i;
        std::vector<uint8_t> s0 =
            mode == VQConnectivityStorageMode::Valence
                ? encodeVQFaceCodes(faceCodeQueues[er], otherValQueues[er])
                : encodeVQAdaptiveCharBits(facetSymbolQueues[er]);
        std::vector<uint8_t> s1 =
            mode == VQConnectivityStorageMode::PaperFan
                ? encodeVQSplitTopology(splitValenceQueues[er],
                                        splitOtherValQueues[er],
                                        rootDiagSlotQueues[er])
                : (mode == VQConnectivityStorageMode::Valence
                       ? encodeVQLocalEdgeHybrid(localEdgeSymbolQueues[er],
                                                 faceCodeQueues[er])
                       : (mode == VQConnectivityStorageMode::Compact
                              ? encodeVQHybridCharBits(edgeSymbolQueues[er])
                              : encodeVQAdaptiveCharBits(edgeSymbolQueues[er])));
        std::vector<uint8_t> s2 =
            encodeVQIndices(vqIndexQueues[er],
                            static_cast<int>(codebookQueues[er].size()));
        std::vector<uint8_t> s3 =
            encodeVQCodebook(codebookQueues[er], predictiveGeometry);

        localSizes.stream0 += s0.size();
        localSizes.stream1 += s1.size();
        localSizes.indices += s2.size();
        localSizes.codebook += s3.size();

        cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamVectorQuantizedTopology0, s0);
        cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamVectorQuantizedTopology1, s1);
        cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamVectorQuantizedIndices, s2);
        cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamVectorQuantizedCodebook, s3);
    }

    std::vector<uint8_t> blob = cbb.finalize();
    localSizes.blobBytes = blob.size();
    if (sizes) *sizes = localSizes;
    return blob;
}

inline VectorQuantizedDecodedBlob readVectorQuantizedConnectivityBlob(
    const std::vector<uint8_t>& blob,
    int expectedBlobLayout) {
    pmsf::ConnectivityBlobReader reader(blob);
    const auto spec = pmsf::vectorQuantizedConnectivityBlobSpec(
        expectedBlobLayout,
        static_cast<std::int32_t>(reader.header().scheme_version),
        static_cast<std::int32_t>(reader.header().num_rounds));
    pmsf::requirePaperConnectivityBlobHeaderMatchesSpec(
        reader.header(), spec, "vector-quantized");

    VectorQuantizedDecodedBlob decoded;
    decoded.schemeVersion = static_cast<int>(reader.header().scheme_version);
    if (!pmsf::paperBlobSchemeVersionIsVectorQuantized(
            decoded.schemeVersion)) {
        throw std::runtime_error(
            "vector-quantized decoder unsupported scheme_version=" +
            std::to_string(decoded.schemeVersion));
    }

    decoded.numRounds = reader.numRounds();
    if (pmsf::paperBlobSchemeVersionVectorQuantizedUsesFacetEdgeStreams(
            decoded.schemeVersion)) {
        decoded.facetSyms.resize(static_cast<std::size_t>(decoded.numRounds));
        if (pmsf::paperBlobSchemeVersionVectorQuantizedUsesSplitTopologyStream(
                decoded.schemeVersion)) {
            decoded.splitValences.resize(static_cast<std::size_t>(decoded.numRounds));
            decoded.rootDiagSlots.resize(static_cast<std::size_t>(decoded.numRounds));
        } else {
            decoded.edgeSyms.resize(static_cast<std::size_t>(decoded.numRounds));
        }
    } else {
        decoded.faceCodes.resize(static_cast<std::size_t>(decoded.numRounds));
        decoded.otherVals.resize(static_cast<std::size_t>(decoded.numRounds));
        decoded.localEdgeSyms.resize(static_cast<std::size_t>(decoded.numRounds));
    }
    decoded.indices.resize(static_cast<std::size_t>(decoded.numRounds));
    decoded.codebooks.resize(static_cast<std::size_t>(decoded.numRounds));
    decoded.absoluteGeometry.resize(static_cast<std::size_t>(decoded.numRounds), 0);

    for (int r = 0; r < decoded.numRounds; ++r) {
        auto s0 = reader.getStream(r, pmsf::kPaperBlobStreamVectorQuantizedTopology0);
        auto s1 = reader.getStream(r, pmsf::kPaperBlobStreamVectorQuantizedTopology1);
        if (decoded.schemeVersion ==
            pmsf::kPaperBlobSchemeVersionVectorQuantizedV1) {
            decoded.facetSyms[static_cast<std::size_t>(r)] =
                decodeVQAdaptiveCharBits(s0.data(), s0.size());
            decoded.edgeSyms[static_cast<std::size_t>(r)] =
                decodeVQAdaptiveCharBits(s1.data(), s1.size());
        } else if (pmsf::paperBlobSchemeVersionVectorQuantizedUsesHybridEdgeStream(
                       decoded.schemeVersion)) {
            decoded.facetSyms[static_cast<std::size_t>(r)] =
                decodeVQAdaptiveCharBits(s0.data(), s0.size());
            decoded.edgeSyms[static_cast<std::size_t>(r)] =
                decodeVQHybridCharBits(s1.data(), s1.size());
        } else if (pmsf::paperBlobSchemeVersionVectorQuantizedUsesSplitTopologyStream(
                       decoded.schemeVersion)) {
            decoded.facetSyms[static_cast<std::size_t>(r)] =
                decodeVQAdaptiveCharBits(s0.data(), s0.size());
            decodeVQSplitTopology(
                s1.data(), s1.size(),
                decoded.splitValences[static_cast<std::size_t>(r)],
                decoded.rootDiagSlots[static_cast<std::size_t>(r)]);
        } else {
            decodeVQFaceCodes(
                s0.data(), s0.size(),
                decoded.faceCodes[static_cast<std::size_t>(r)],
                decoded.otherVals[static_cast<std::size_t>(r)]);
            decoded.localEdgeSyms[static_cast<std::size_t>(r)] =
                decodeVQLocalEdgeMasks(
                    s1.data(), s1.size(),
                    decoded.faceCodes[static_cast<std::size_t>(r)],
                    pmsf::paperBlobSchemeVersionVectorQuantizedUsesContextLocalEdges(
                        decoded.schemeVersion));
        }

        auto s2 = reader.getStream(r, pmsf::kPaperBlobStreamVectorQuantizedIndices);
        decoded.indices[static_cast<std::size_t>(r)] =
            decodeVQIndices(s2.data(), s2.size());
        auto s3 = reader.getStream(r, pmsf::kPaperBlobStreamVectorQuantizedCodebook);
        decoded.absoluteGeometry[static_cast<std::size_t>(r)] =
            decodeVQCodebookAbsoluteMode(s3.data(), s3.size()) ? 1 : 0;
        decoded.codebooks[static_cast<std::size_t>(r)] =
            decodeVQCodebook(s3.data(), s3.size());
    }

    return decoded;
}

inline void pushVQFanDiagonals(MCGAL::Halfedge* entry,
                               int rootDiagSlot,
                               int valence,
                               std::vector<MCGAL::Halfedge*>& halfedges) {
    if (!entry || valence < 4 || rootDiagSlot < 0 || rootDiagSlot > 2) return;
    MCGAL::Halfedge* hd = entry;
    for (int i = 0; i < rootDiagSlot && hd != nullptr; ++i) {
        hd = hd->next();
    }
    if (!hd) return;
    halfedges.push_back(hd);

    MCGAL::Halfedge* hdCur = hd;
    for (int i = 1; i <= valence - 4; ++i) {
        if (!hdCur || !hdCur->opposite()) break;
        MCGAL::Halfedge* hdOp = hdCur->opposite();
        MCGAL::Halfedge* h0Fan = hdOp ? hdOp->next() : nullptr;
        if (!h0Fan || !h0Fan->next()) break;
        MCGAL::Halfedge* hdNext = h0Fan->next();
        halfedges.push_back(hdNext);
        hdCur = hdNext;
    }
}

inline void applyVectorQuantizedDecodedRound(
    int schemeVersion,
    int currentRound,
    const VectorQuantizedDecodedBlob& decoded,
    MCGAL::Halfedge* seed,
    std::vector<MCGAL::Vertex*>& vertices,
    std::vector<MCGAL::Halfedge*>& halfedges,
    std::vector<MCGAL::Facet*>& facets) {
    if (!seed) return;

    const auto& codebook =
        decoded.codebooks[static_cast<std::size_t>(currentRound)];
    const auto& indices =
        decoded.indices[static_cast<std::size_t>(currentRound)];
    const bool absoluteGeometry =
        currentRound < static_cast<int>(decoded.absoluteGeometry.size()) &&
        decoded.absoluteGeometry[static_cast<std::size_t>(currentRound)] != 0;

    auto setResidualFromIndex = [&](MCGAL::Facet* f, std::size_t indexId) {
        if (!f || indexId >= indices.size()) return false;
        const uint16_t idx = indices[indexId];
        MCGAL::Point residual(0, 0, 0);
        if (static_cast<int>(idx) < static_cast<int>(codebook.size())) {
            residual = codebook[static_cast<std::size_t>(idx)];
        }
        if (absoluteGeometry) {
            pmsf::setPaperProtocolHint(f, pmsf::kPaperBlobLayoutCodec021);
        }
        f->setRemovedVertexPos(residual);
        facets.push_back(f);
        return true;
    };

    if (pmsf::paperBlobSchemeVersionVectorQuantizedUsesFaceCodeTopology(
            schemeVersion)) {
        const auto& faceCodes =
            decoded.faceCodes[static_cast<std::size_t>(currentRound)];
        const auto& otherVals =
            decoded.otherVals[static_cast<std::size_t>(currentRound)];
        const auto& edgeSyms =
            decoded.localEdgeSyms[static_cast<std::size_t>(currentRound)];

        std::queue<int> gateQ;
        gateQ.push(seed->poolId());
        const int currentVersion = MCGAL::BFSVersionManager::current_version++;

        std::size_t fi = 0;
        std::size_t oi = 0;
        std::size_t ei = 0;
        std::size_t ii = 0;
        while (!gateQ.empty()) {
            const int hid = gateQ.front();
            gateQ.pop();
            MCGAL::Halfedge* h =
                MCGAL::contextPool.getHalfedgeByIndexInSubPool(DEFAULT_MESH_ID, hid);
            if (h->isVisited(currentVersion) || h->isRemoved()) continue;
            h->setVisited(currentVersion);

            MCGAL::Facet* f = h->face();
            if (!f->isVisited(currentVersion)) {
                f->setVisited(currentVersion);
                const uint8_t faceCode = fi < faceCodes.size() ? faceCodes[fi] : 0;
                ++fi;
                if (faceCode != 0) {
                    f->setSplittable();
                    int polyValence = 0;
                    if (faceCode >= 1 && faceCode <= 4) {
                        polyValence = static_cast<int>(faceCode) + 2;
                    } else if (faceCode == 5) {
                        polyValence = oi < otherVals.size() ? static_cast<int>(otherVals[oi++]) : 0;
                    }
                    if (polyValence > 0) f->setPolyValence(polyValence);
                    if (setResidualFromIndex(f, ii)) ++ii;
                }

                MCGAL::Halfedge* hLocal = h;
                for (int k = 0; k < 3 && hLocal != nullptr; ++k) {
                    if (ei < edgeSyms.size() && edgeSyms[ei]) {
                        hLocal->setAdded();
                        halfedges.push_back(hLocal);
                    }
                    ++ei;
                    hLocal = hLocal->next();
                }
            }

            MCGAL::Halfedge* hIt = h;
            do {
                MCGAL::Halfedge* hOpp = hIt->opposite();
                if (hOpp != nullptr && !hOpp->isVisited(currentVersion)) {
                    gateQ.push(hOpp->poolId());
                }
                if (!hIt->isVisited(currentVersion)) gateQ.push(hIt->poolId());
                hIt = hIt->next();
            } while (hIt != h);
        }
        (void)vertices;
        return;
    }

    if (pmsf::paperBlobSchemeVersionVectorQuantizedUsesSplitTopologyStream(
            schemeVersion)) {
        const auto& facetSyms =
            decoded.facetSyms[static_cast<std::size_t>(currentRound)];
        const auto& splitVals =
            decoded.splitValences[static_cast<std::size_t>(currentRound)];
        const auto& rootSlots =
            decoded.rootDiagSlots[static_cast<std::size_t>(currentRound)];

        std::queue<int> gateQ;
        gateQ.push(seed->poolId());
        const int currentVersion = MCGAL::BFSVersionManager::current_version++;

        std::size_t fi = 0;
        std::size_t vi = 0;
        std::size_t si = 0;
        std::size_t ii = 0;
        while (!gateQ.empty()) {
            const int hid = gateQ.front();
            gateQ.pop();
            MCGAL::Halfedge* h =
                MCGAL::contextPool.getHalfedgeByIndexInSubPool(DEFAULT_MESH_ID, hid);
            if (h->isVisited(currentVersion) || h->isRemoved()) continue;
            h->setVisited(currentVersion);

            MCGAL::Facet* f = h->face();
            if (!f->isVisited(currentVersion)) {
                f->setVisited(currentVersion);
                if (fi < facetSyms.size() && facetSyms[fi]) {
                    const int valence = vi < splitVals.size() ? static_cast<int>(splitVals[vi++]) : 3;
                    f->setSplittable();
                    f->setPolyValence(valence);
                    if (valence >= 4) {
                        const int slot = si < rootSlots.size() ? static_cast<int>(rootSlots[si++]) : 3;
                        pushVQFanDiagonals(h, slot, valence, halfedges);
                    }
                    if (setResidualFromIndex(f, ii)) ++ii;
                }
                ++fi;
            }

            MCGAL::Halfedge* hIt = h;
            do {
                MCGAL::Halfedge* hOpp = hIt->opposite();
                if (hOpp != nullptr && !hOpp->isVisited(currentVersion)) {
                    gateQ.push(hOpp->poolId());
                }
                if (!hIt->isVisited(currentVersion)) gateQ.push(hIt->poolId());
                hIt = hIt->next();
            } while (hIt != h);
        }
        (void)vertices;
        return;
    }

    const auto& facetSyms =
        decoded.facetSyms[static_cast<std::size_t>(currentRound)];
    const auto& edgeSyms =
        decoded.edgeSyms[static_cast<std::size_t>(currentRound)];

    std::queue<int> gateQ;
    gateQ.push(seed->poolId());
    const int currentVersion = MCGAL::BFSVersionManager::current_version++;

    std::size_t fi = 0;
    std::size_t ei = 0;
    std::size_t ii = 0;
    while (!gateQ.empty()) {
        const int hid = gateQ.front();
        gateQ.pop();
        MCGAL::Halfedge* h =
            MCGAL::contextPool.getHalfedgeByIndexInSubPool(DEFAULT_MESH_ID, hid);
        if (h->isVisited(currentVersion) || h->isRemoved()) continue;
        h->setVisited(currentVersion);

        if (ei < edgeSyms.size() && edgeSyms[ei]) {
            h->setAdded();
            halfedges.push_back(h);
        }
        ++ei;

        MCGAL::Facet* f = h->face();
        if (!f->isVisited(currentVersion)) {
            f->setVisited(currentVersion);
            if (fi < facetSyms.size() && facetSyms[fi]) {
                f->setSplittable();
                if (setResidualFromIndex(f, ii)) ++ii;
            }
            ++fi;
        }

        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp != nullptr && !hOpp->isVisited(currentVersion)) {
                gateQ.push(hOpp->poolId());
            }
            if (!hIt->isVisited(currentVersion)) gateQ.push(hIt->poolId());
            hIt = hIt->next();
        } while (hIt != h);
    }
    (void)vertices;
}

}  // namespace pmsf

#endif  // PMSF_COMPRESS_VECTOR_QUANTIZED_CONNECTIVITY_CODEC_H
