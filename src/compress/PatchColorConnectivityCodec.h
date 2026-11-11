// SPDX-License-Identifier: Apache-2.0
//
// Framework-level codec for the common patch/color connectivity stream used by
// PCATM-style paper reproductions.
//
// Layout:
//   stream 0: 5-symbol valence code + 4 adaptive contexts + V_OTHER escape
//   stream 1: adaptive binary is-poly bit stream
//   stream 2: adaptive binary patch/color bit stream
//   stream 3: signed 3D residuals encoded as ZigZag bit planes

#ifndef PMSF_COMPRESS_PATCH_COLOR_CONNECTIVITY_CODEC_H
#define PMSF_COMPRESS_PATCH_COLOR_CONNECTIVITY_CODEC_H

#include "Configuration.h"
#include "ConnectivityBlobContainer.h"
#include "Point.h"
#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"
#include "storage/PaperBlobLayoutRegistry.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <deque>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace pmsf {

struct PatchColorStreamSizes {
    int valence = 0;
    int isPoly = 0;
    int color = 0;
    int residual = 0;
    int blobBytes = 0;

    int payloadBytes() const { return valence + isPoly + color + residual; }
};

struct PatchColorDecodedBlob {
    int numRounds = 0;
    std::size_t blobSize = 0;
    std::vector<int> faceBitsPerRound;
    std::deque<uint8_t> valenceCodes;
    std::deque<uint8_t> otherValences;
    std::deque<uint8_t> isPolyBits;
    std::deque<uint8_t> colorBits;
    std::deque<MCGAL::PointInt> residuals;
};

namespace detail {

inline int valence5Context(int prev) {
    if (prev < 0) return 0;
    if (prev == 0 || prev == 1) return 1;
    if (prev == 2 || prev == 3) return 2;
    return 3;
}

inline int32_t readI32(std::span<const uint8_t> bytes,
                       std::size_t offset,
                       const char* field) {
    if (offset + 4 > bytes.size()) {
        throw std::runtime_error(std::string("patch/color stream truncated: ") + field);
    }
    int32_t v = 0;
    std::memcpy(&v, bytes.data() + offset, 4);
    return v;
}

inline uint32_t zigzag32(int32_t v) {
    return (static_cast<uint32_t>(v) << 1) ^ static_cast<uint32_t>(v >> 31);
}

inline int32_t unzigzag32(uint32_t v) {
    return static_cast<int32_t>((v >> 1) ^ -(static_cast<int32_t>(v & 1u)));
}

}  // namespace detail

inline std::vector<uint8_t> encodeValence5WithContext(
    const std::deque<uint8_t>& valCodes,
    const std::deque<uint8_t>& otherVals,
    uint8_t otherSymbol = 4) {
    constexpr int kLgTotf = 10;
    constexpr int kRescale = 1 << 9;
    constexpr int kNumCtx = 4;
    constexpr int kAlphabetSize = 8;
    const std::size_t bufSize =
        std::max<std::size_t>(BUFFER_SIZE / 12, valCodes.size() * 2 + 1024);
    std::vector<char> buf(bufSize);
    std::size_t off = 0;

    rangecoder rc;
    rc.p_data = buf.data();
    rc.p_dataOffset = &off;
    start_encoding(&rc, 0, 0);

    qsmodel models[kNumCtx];
    for (auto& model : models) {
        initqsmodel(&model, kAlphabetSize, kLgTotf, kRescale, nullptr, 1);
    }
    qsmodel escapeModel;
    initqsmodel(&escapeModel, 256, kLgTotf, kRescale, nullptr, 1);

    int prev = -1;
    std::size_t escCursor = 0;
    for (uint8_t rawSym : valCodes) {
        const int sym = static_cast<int>(rawSym);
        qsmodel* model = &models[detail::valence5Context(prev)];
        int syfreq = 0;
        int ltfreq = 0;
        qsgetfreq(model, sym, &syfreq, &ltfreq);
        encode_shift(&rc, syfreq, ltfreq, kLgTotf);
        qsupdate(model, sym);

        if (rawSym == otherSymbol) {
            const uint8_t esc =
                escCursor < otherVals.size() ? otherVals[escCursor++] : 0;
            int esyfreq = 0;
            int eltfreq = 0;
            qsgetfreq(&escapeModel, esc, &esyfreq, &eltfreq);
            encode_shift(&rc, esyfreq, eltfreq, kLgTotf);
            qsupdate(&escapeModel, esc);
        }
        prev = sym;
    }

    const std::size_t rcSize = done_encoding(&rc);
    for (auto& model : models) deleteqsmodel(&model);
    deleteqsmodel(&escapeModel);

    std::vector<uint8_t> out(8);
    const int32_t numPoly = static_cast<int32_t>(valCodes.size());
    const int32_t numOther = static_cast<int32_t>(otherVals.size());
    std::memcpy(out.data(), &numPoly, 4);
    std::memcpy(out.data() + 4, &numOther, 4);
    out.insert(out.end(),
               reinterpret_cast<uint8_t*>(buf.data()),
               reinterpret_cast<uint8_t*>(buf.data()) + rcSize);
    return out;
}

inline std::vector<uint8_t> encodeAdaptiveBits(const std::deque<uint8_t>& bits) {
    constexpr int kLgTotf = 10;
    constexpr int kRescale = 1 << 9;
    std::vector<char> buf(std::max<std::size_t>(bits.size() / 4 + 1024, 4096));
    std::size_t off = 0;

    rangecoder rc;
    rc.p_data = buf.data();
    rc.p_dataOffset = &off;
    start_encoding(&rc, 0, 0);

    qsmodel model;
    initqsmodel(&model, 2, kLgTotf, kRescale, nullptr, 1);
    for (uint8_t bit : bits) {
        const int sym = bit ? 1 : 0;
        int syfreq = 0;
        int ltfreq = 0;
        qsgetfreq(&model, sym, &syfreq, &ltfreq);
        encode_shift(&rc, syfreq, ltfreq, kLgTotf);
        qsupdate(&model, sym);
    }

    const std::size_t rcSize = done_encoding(&rc);
    deleteqsmodel(&model);

    std::vector<uint8_t> out(4);
    const int32_t numBits = static_cast<int32_t>(bits.size());
    std::memcpy(out.data(), &numBits, 4);
    out.insert(out.end(),
               reinterpret_cast<uint8_t*>(buf.data()),
               reinterpret_cast<uint8_t*>(buf.data()) + rcSize);
    return out;
}

inline std::vector<uint8_t> encodeResidualBitplane(
    const std::deque<MCGAL::PointInt>& residuals,
    int bpBits) {
    std::vector<char> buf(BUFFER_SIZE / 3);
    std::size_t off = 0;

    rangecoder rc;
    rc.p_data = buf.data();
    rc.p_dataOffset = &off;
    start_encoding(&rc, 0, 0);

    std::vector<qsmodel> bpFirst1(static_cast<std::size_t>(bpBits));
    std::vector<qsmodel> bpAfter1(static_cast<std::size_t>(bpBits));
    for (int b = 0; b < bpBits; ++b) {
        initqsmodel(&bpFirst1[static_cast<std::size_t>(b)], 2, 10, 1 << 9, nullptr, 1);
        initqsmodel(&bpAfter1[static_cast<std::size_t>(b)], 2, 10, 1 << 9, nullptr, 1);
    }

    auto encodeCoord = [&](int32_t coord) {
        const uint32_t z = detail::zigzag32(coord);
        bool seenOne = false;
        for (int b = bpBits - 1; b >= 0; --b) {
            const int bit = static_cast<int>((z >> b) & 1u);
            qsmodel* model = seenOne ? &bpAfter1[static_cast<std::size_t>(b)]
                                     : &bpFirst1[static_cast<std::size_t>(b)];
            int syfreq = 0;
            int ltfreq = 0;
            qsgetfreq(model, bit, &syfreq, &ltfreq);
            encode_shift(&rc, syfreq, ltfreq, 10);
            qsupdate(model, bit);
            if (bit) seenOne = true;
        }
    };

    for (const auto& res : residuals) {
        encodeCoord(res.x());
        encodeCoord(res.y());
        encodeCoord(res.z());
    }

    const std::size_t rcSize = done_encoding(&rc);
    for (int b = 0; b < bpBits; ++b) {
        deleteqsmodel(&bpFirst1[static_cast<std::size_t>(b)]);
        deleteqsmodel(&bpAfter1[static_cast<std::size_t>(b)]);
    }

    std::vector<uint8_t> out(4);
    const int32_t numCoords = static_cast<int32_t>(residuals.size() * 3);
    std::memcpy(out.data(), &numCoords, 4);
    out.insert(out.end(),
               reinterpret_cast<uint8_t*>(buf.data()),
               reinterpret_cast<uint8_t*>(buf.data()) + rcSize);
    return out;
}

inline void decodeValence5WithContext(std::span<const uint8_t> stream,
                                      std::deque<uint8_t>& valCodes,
                                      std::deque<uint8_t>& otherVals,
                                      uint8_t otherSymbol = 4) {
    if (stream.size() < 8) {
        throw std::runtime_error("patch/color valence stream too small");
    }
    const int32_t numPoly = detail::readI32(stream, 0, "valence.numPoly");
    const int32_t numOther = detail::readI32(stream, 4, "valence.numOther");
    if (numPoly < 0 || numOther < 0) {
        throw std::runtime_error("patch/color valence stream has negative count");
    }

    std::vector<char> staging(stream.begin() + 8, stream.end());
    std::size_t off = 0;
    rangecoder rc;
    rc.p_data = staging.data();
    rc.p_dataOffset = &off;
    if (start_decoding(&rc) != 0) {
        throw std::runtime_error("patch/color valence RC start_decoding failed");
    }

    constexpr int kLgTotf = 10;
    constexpr int kRescale = 1 << 9;
    constexpr int kNumCtx = 4;
    constexpr int kAlphabetSize = 8;
    qsmodel models[kNumCtx];
    for (auto& model : models) {
        initqsmodel(&model, kAlphabetSize, kLgTotf, kRescale, nullptr, 0);
    }
    qsmodel escapeModel;
    initqsmodel(&escapeModel, 256, kLgTotf, kRescale, nullptr, 0);

    int prev = -1;
    int otherSeen = 0;
    for (int32_t i = 0; i < numPoly; ++i) {
        qsmodel* model = &models[detail::valence5Context(prev)];
        const int ltval = decode_culshift(&rc, kLgTotf);
        const int sym = qsgetsym(model, ltval);
        int syfreq = 0;
        int ltfreq = 0;
        qsgetfreq(model, sym, &syfreq, &ltfreq);
        decode_update(&rc, syfreq, ltfreq, 1 << kLgTotf);
        qsupdate(model, sym);
        valCodes.push_back(static_cast<uint8_t>(sym));

        if (sym == static_cast<int>(otherSymbol)) {
            const int eltval = decode_culshift(&rc, kLgTotf);
            const int esc = qsgetsym(&escapeModel, eltval);
            int esyfreq = 0;
            int eltfreq = 0;
            qsgetfreq(&escapeModel, esc, &esyfreq, &eltfreq);
            decode_update(&rc, esyfreq, eltfreq, 1 << kLgTotf);
            qsupdate(&escapeModel, esc);
            otherVals.push_back(static_cast<uint8_t>(esc));
            ++otherSeen;
        }
        prev = sym;
    }

    done_decoding(&rc);
    for (auto& model : models) deleteqsmodel(&model);
    deleteqsmodel(&escapeModel);
    (void)otherSeen;
    (void)numOther;
}

inline std::deque<uint8_t> decodeAdaptiveBits(std::span<const uint8_t> stream,
                                              int* numBitsOut = nullptr) {
    if (stream.size() < 4) {
        throw std::runtime_error("patch/color binary stream too small");
    }
    const int32_t numBits = detail::readI32(stream, 0, "binary.numBits");
    if (numBits < 0) {
        throw std::runtime_error("patch/color binary stream has negative count");
    }
    if (numBitsOut) *numBitsOut = numBits;

    std::vector<char> staging(stream.begin() + 4, stream.end());
    std::size_t off = 0;
    rangecoder rc;
    rc.p_data = staging.data();
    rc.p_dataOffset = &off;
    if (start_decoding(&rc) != 0) {
        throw std::runtime_error("patch/color binary RC start_decoding failed");
    }

    qsmodel model;
    initqsmodel(&model, 2, 10, 1 << 9, nullptr, 0);
    std::deque<uint8_t> out;
    for (int32_t i = 0; i < numBits; ++i) {
        const int ltval = decode_culshift(&rc, 10);
        const int sym = qsgetsym(&model, ltval);
        int syfreq = 0;
        int ltfreq = 0;
        qsgetfreq(&model, sym, &syfreq, &ltfreq);
        decode_update(&rc, syfreq, ltfreq, 1 << 10);
        qsupdate(&model, sym);
        out.push_back(sym ? 1 : 0);
    }
    done_decoding(&rc);
    deleteqsmodel(&model);
    return out;
}

inline std::deque<MCGAL::PointInt> decodeResidualBitplane(
    std::span<const uint8_t> stream,
    int bpBits) {
    if (stream.size() < 4) {
        throw std::runtime_error("patch/color residual stream too small");
    }
    const int32_t numCoords = detail::readI32(stream, 0, "residual.numCoords");
    if (numCoords < 0 || numCoords % 3 != 0) {
        throw std::runtime_error("patch/color residual stream has invalid count");
    }

    std::vector<char> staging(stream.begin() + 4, stream.end());
    std::size_t off = 0;
    rangecoder rc;
    rc.p_data = staging.data();
    rc.p_dataOffset = &off;
    if (start_decoding(&rc) != 0) {
        throw std::runtime_error("patch/color residual RC start_decoding failed");
    }

    std::vector<qsmodel> bpFirst1(static_cast<std::size_t>(bpBits));
    std::vector<qsmodel> bpAfter1(static_cast<std::size_t>(bpBits));
    for (int b = 0; b < bpBits; ++b) {
        initqsmodel(&bpFirst1[static_cast<std::size_t>(b)], 2, 10, 1 << 9, nullptr, 0);
        initqsmodel(&bpAfter1[static_cast<std::size_t>(b)], 2, 10, 1 << 9, nullptr, 0);
    }

    auto decodeCoord = [&]() -> int32_t {
        uint32_t z = 0;
        bool seenOne = false;
        for (int b = bpBits - 1; b >= 0; --b) {
            qsmodel* model = seenOne ? &bpAfter1[static_cast<std::size_t>(b)]
                                     : &bpFirst1[static_cast<std::size_t>(b)];
            const int ltval = decode_culshift(&rc, 10);
            const int sym = qsgetsym(model, ltval);
            int syfreq = 0;
            int ltfreq = 0;
            qsgetfreq(model, sym, &syfreq, &ltfreq);
            decode_update(&rc, syfreq, ltfreq, 1 << 10);
            qsupdate(model, sym);
            z = (z << 1) | static_cast<uint32_t>(sym);
            if (sym) seenOne = true;
        }
        return detail::unzigzag32(z);
    };

    std::deque<MCGAL::PointInt> residuals;
    for (int32_t coordsConsumed = 0; coordsConsumed < numCoords; coordsConsumed += 3) {
        residuals.emplace_back(decodeCoord(), decodeCoord(), decodeCoord());
    }

    done_decoding(&rc);
    for (int b = 0; b < bpBits; ++b) {
        deleteqsmodel(&bpFirst1[static_cast<std::size_t>(b)]);
        deleteqsmodel(&bpAfter1[static_cast<std::size_t>(b)]);
    }
    return residuals;
}

inline PatchColorStreamSizes writePatchColorConnectivityBlob(
    char* buffer,
    int blobLayout,
    int schemeVersion,
    const std::vector<std::deque<uint8_t>>& valenceQueues,
    const std::vector<std::deque<uint8_t>>& otherValQueues,
    const std::vector<std::deque<uint8_t>>& isPolyQueues,
    const std::vector<std::deque<uint8_t>>& colorQueues,
    const std::vector<std::deque<MCGAL::PointInt>>& residualQueues,
    int bpBits) {
    const int numRounds = static_cast<int>(valenceQueues.size());
    if (static_cast<int>(otherValQueues.size()) != numRounds ||
        static_cast<int>(isPolyQueues.size()) != numRounds ||
        static_cast<int>(colorQueues.size()) != numRounds ||
        static_cast<int>(residualQueues.size()) != numRounds) {
        throw std::runtime_error("patch/color blob queues are not aligned by round");
    }

    ConnectivityBlobBuilder builder(
        patchColorConnectivityBlobSpec(blobLayout, schemeVersion, numRounds));
    PatchColorStreamSizes sizes;
    for (int i = 0; i < numRounds; ++i) {
        const int er = numRounds - 1 - i;
        auto valence = encodeValence5WithContext(valenceQueues[er], otherValQueues[er]);
        sizes.valence += static_cast<int>(valence.size());
        builder.appendStreamBytes(i, kPaperBlobStreamPatchColorValence, valence);

        auto isPoly = encodeAdaptiveBits(isPolyQueues[er]);
        sizes.isPoly += static_cast<int>(isPoly.size());
        builder.appendStreamBytes(i, kPaperBlobStreamPatchColorIsPoly, isPoly);

        auto color = encodeAdaptiveBits(colorQueues[er]);
        sizes.color += static_cast<int>(color.size());
        builder.appendStreamBytes(i, kPaperBlobStreamPatchColorColor, color);

        auto residual = encodeResidualBitplane(residualQueues[er], bpBits);
        sizes.residual += static_cast<int>(residual.size());
        builder.appendStreamBytes(i, kPaperBlobStreamPatchColorResidual, residual);
    }

    auto blob = builder.finalize();
    std::memcpy(buffer, blob.data(), blob.size());
    sizes.blobBytes = static_cast<int>(blob.size());
    return sizes;
}

inline PatchColorDecodedBlob readPatchColorConnectivityBlob(
    std::span<const uint8_t> blobView,
    int expectedBlobLayout,
    int expectedSchemeVersion,
    int bpBits,
    const char* codecName) {
    ConnectivityBlobReader reader(blobView);
    const auto& hdr = reader.header();
    const auto spec = patchColorConnectivityBlobSpec(
        expectedBlobLayout,
        expectedSchemeVersion,
        static_cast<std::int32_t>(hdr.num_rounds));
    requirePaperConnectivityBlobHeaderMatchesSpec(hdr, spec, codecName);

    PatchColorDecodedBlob decoded;
    decoded.numRounds = static_cast<int>(hdr.num_rounds);
    decoded.faceBitsPerRound.assign(decoded.numRounds, 0);

    for (int r = 0; r < decoded.numRounds; ++r) {
        decodeValence5WithContext(reader.getStream(r, kPaperBlobStreamPatchColorValence),
                                  decoded.valenceCodes,
                                  decoded.otherValences);

        int faceBits = 0;
        auto isPoly = decodeAdaptiveBits(
            reader.getStream(r, kPaperBlobStreamPatchColorIsPoly), &faceBits);
        decoded.faceBitsPerRound[r] = faceBits;
        decoded.isPolyBits.insert(decoded.isPolyBits.end(), isPoly.begin(), isPoly.end());

        auto colors = decodeAdaptiveBits(reader.getStream(r, kPaperBlobStreamPatchColorColor));
        decoded.colorBits.insert(decoded.colorBits.end(), colors.begin(), colors.end());

        auto residuals = decodeResidualBitplane(
            reader.getStream(r, kPaperBlobStreamPatchColorResidual), bpBits);
        decoded.residuals.insert(decoded.residuals.end(),
                                 residuals.begin(),
                                 residuals.end());
    }

    decoded.blobSize = hdr.headerByteSize() +
                       (hdr.stream_offsets.empty() ? 0 : hdr.stream_offsets.back());
    return decoded;
}

}  // namespace pmsf

#endif  // PMSF_COMPRESS_PATCH_COLOR_CONNECTIVITY_CODEC_H
