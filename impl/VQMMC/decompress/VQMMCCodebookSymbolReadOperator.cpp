// SPDX-License-Identifier: Apache-2.0
//
// VQMMCCodebookSymbolReadOperator.cpp — VQMMC codebook protocol decoder.
//
// Accepts BlobLayout=23 (CODEC_023) in two physical forms:
//   - 3 streams: legacy RDTag-equivalent valence + isPolyBit + residual.
//   - 4 streams: vector-quantized connectivity blob with codebook + indices.

#include "VQMMCCodebookSymbolReadOperator.h"
#include "common/PaperLayoutAliases.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂
#include "VQMMC/common/PMSFVectorQuantizedPayloadAdapter.h"

#include "compress/ConnectivityBlobContainer.h"
#include "RD/compress/RDSymbolCollectOperator.h"
#include "Point.h"

#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"

#include <cstdint>
#include <cstring>
#include <iostream>
#include <span>
#include <stdexcept>
#include <vector>

void VQMMCCodebookSymbolReadOperator::importFromRangeCodedBuffer(
    char* buffer, size_t& dataOffset, unsigned nbQuantBits) {

    constexpr size_t kLargeView = static_cast<size_t>(1) << 32;
    std::span<const uint8_t> view(
        reinterpret_cast<const uint8_t*>(buffer + dataOffset), kLargeView);

    pmsf::ConnectivityBlobReader rd(view);
    const auto& hdr = rd.header();
    const auto legacySpec = pmsf::vqmmcLegacyCodebookConnectivityBlobSpec(
        static_cast<std::int32_t>(hdr.num_rounds));
    const size_t blobSize = hdr.headerByteSize() +
                            (hdr.stream_offsets.empty() ? 0 : hdr.stream_offsets.back());

    useVectorQuantizedBlob_ = false;
    currentVectorQuantizedRound_ = 0;
    decodedVectorQuantizedBlob_ = pmsf::VectorQuantizedDecodedBlob{};

    if (pmsf::paperBlobStreamCountVQMMCUsesVectorQuantizedBlob(
            hdr.num_streams)) {
        std::vector<uint8_t> blob(blobSize);
        std::memcpy(blob.data(), buffer + dataOffset, blobSize);
        decodedVectorQuantizedBlob_ = pmsf::readVectorQuantizedConnectivityBlob(
            blob, legacySpec.blobLayout);
        useVectorQuantizedBlob_ = true;
        dataOffset += blobSize;
        return;
    }

    if (!pmsf::paperBlobStreamCountVQMMCUsesLegacyMain(hdr.num_streams)) {
        throw std::runtime_error(
            "[VQMMC-CB decoder] expected num_streams=" +
            std::to_string(pmsf::kPaperBlobStreamCountVQMMCLegacyMain) + " or " +
            std::to_string(pmsf::kPaperBlobStreamCountVectorQuantizedBlob) + " got " +
            std::to_string(hdr.num_streams));
    }
    const auto physicalSpec = pmsf::makePaperConnectivityBlobSpec(
        legacySpec.blobLayout,
        static_cast<std::int32_t>(hdr.scheme_version),
        static_cast<std::int32_t>(hdr.num_rounds),
        pmsf::kPaperBlobStreamCountVQMMCLegacyMain);
    pmsf::requirePaperConnectivityBlobHeaderMatchesSpec(
        hdr, physicalSpec, "VQMMC-CB");
    if (pmsf::vqmmcPMSFSymbolTableMarksBlobPayloadStreamsPresent(
            pmsfSymbolTable())) {
        throw std::runtime_error(
            "[VQMMC-CB decoder] paper sidecar marks blob-level codebook/index "
            "streams as present, but the physical blob has only the legacy "
            "3-stream valence/isPoly/residual layout.");
    }

    const int numRounds = static_cast<int>(hdr.num_rounds);
    rcUsingResidualLocal_ = header_.isEnablePrediction() && header_.isEnableQuantization();
    if (!rcUsingResidualLocal_) {
        throw std::runtime_error(
            "[VQMMC-CB decoder] expects enablePrediction && enableQuantization");
    }
    const int BP_BITS = static_cast<int>(nbQuantBits) + 1;
    faceBitsPerRound_.assign(numRounds, 0);

    constexpr int lg_totf = 10;
    constexpr int rescale = 1 << 9;

    for (int r = 0; r < numRounds; ++r) {
        // stream 0: valence
        {
            auto sl = rd.getStream(r, pmsf::kPaperBlobStreamVQMMCLegacyValence);
            if (sl.size() < 8)
                throw std::runtime_error("[VQMMC-CB decoder] stream 0 too small");
            int32_t numPoly = 0, numOther = 0;
            std::memcpy(&numPoly,  sl.data(),     4);
            std::memcpy(&numOther, sl.data() + 4, 4);

            std::vector<char> rcStaging(sl.begin() + 8, sl.end());
            size_t rcOffset = 0;
            rangecoder rc;
            rc.p_data = rcStaging.data();
            rc.p_dataOffset = &rcOffset;
            if (start_decoding(&rc) != 0)
                throw std::runtime_error("[VQMMC-CB decoder] valence RC start failed");

            constexpr int alphabet_size = 8;
            qsmodel ms[4];
            for (int i = 0; i < 4; ++i)
                initqsmodel(&ms[i], alphabet_size, lg_totf, rescale, nullptr, 0);
            qsmodel esc;
            initqsmodel(&esc, 256, 10, 1 << 9, nullptr, 0);

            auto deriveCtx = [](int prev) {
                if (prev < 0) return 0;
                if (prev == (int)RDValenceCode::VAL_3 || prev == (int)RDValenceCode::VAL_4) return 1;
                if (prev == (int)RDValenceCode::VAL_5 || prev == (int)RDValenceCode::VAL_6) return 2;
                return 3;
            };

            int prev = -1;
            for (int k = 0; k < numPoly; ++k) {
                int ctx = deriveCtx(prev);
                int syfreq, ltfreq;
                int ltval = decode_culshift(&rc, lg_totf);
                int sym = qsgetsym(&ms[ctx], ltval);
                qsgetfreq(&ms[ctx], sym, &syfreq, &ltfreq);
                decode_update(&rc, syfreq, ltfreq, 1 << lg_totf);
                qsupdate(&ms[ctx], sym);
                rcDecodedValCodes_.push_back((uint8_t)sym);
                if (sym == (int)RDValenceCode::VAL_OTHER) {
                    int eltval = decode_culshift(&rc, 10);
                    int esym = qsgetsym(&esc, eltval);
                    int esyfreq, eltfreq;
                    qsgetfreq(&esc, esym, &esyfreq, &eltfreq);
                    decode_update(&rc, esyfreq, eltfreq, 1 << 10);
                    qsupdate(&esc, esym);
                    rcDecodedOtherVals_.push_back((uint8_t)esym);
                }
                prev = sym;
            }
            done_decoding(&rc);
            for (auto& m : ms) deleteqsmodel(&m);
            deleteqsmodel(&esc);
        }

        // stream 1: isPolyBit
        {
            auto sl = rd.getStream(r, pmsf::kPaperBlobStreamVQMMCLegacyIsPoly);
            if (sl.size() < 4)
                throw std::runtime_error("[VQMMC-CB decoder] isPoly too small");
            int32_t numFaceBits = 0;
            std::memcpy(&numFaceBits, sl.data(), 4);
            faceBitsPerRound_[r] = numFaceBits;

            std::vector<char> rcStaging(sl.begin() + 4, sl.end());
            size_t rcOffset = 0;
            rangecoder rc;
            rc.p_data = rcStaging.data();
            rc.p_dataOffset = &rcOffset;
            if (start_decoding(&rc) != 0)
                throw std::runtime_error("[VQMMC-CB decoder] isPoly start failed");
            qsmodel m;
            initqsmodel(&m, 2, lg_totf, rescale, nullptr, 0);
            for (int k = 0; k < numFaceBits; ++k) {
                int syfreq, ltfreq;
                int ltval = decode_culshift(&rc, lg_totf);
                int sym = qsgetsym(&m, ltval);
                qsgetfreq(&m, sym, &syfreq, &ltfreq);
                decode_update(&rc, syfreq, ltfreq, 1 << lg_totf);
                qsupdate(&m, sym);
                rcDecodedIsPoly_.push_back(sym ? 1 : 0);
            }
            done_decoding(&rc);
            deleteqsmodel(&m);
        }

        // stream 2: residual (bit-plane)
        {
            auto sl = rd.getStream(r, pmsf::kPaperBlobStreamVQMMCLegacyResidual);
            if (sl.size() < 4)
                throw std::runtime_error("[VQMMC-CB decoder] residual too small");
            int32_t numCoords = 0;
            std::memcpy(&numCoords, sl.data(), 4);

            std::vector<char> rcStaging(sl.begin() + 4, sl.end());
            size_t rcOffset = 0;
            rangecoder rc;
            rc.p_data = rcStaging.data();
            rc.p_dataOffset = &rcOffset;
            if (start_decoding(&rc) != 0)
                throw std::runtime_error("[VQMMC-CB decoder] residual start failed");

            std::vector<qsmodel> bpFirst1(BP_BITS), bpAfter1(BP_BITS);
            for (int b = 0; b < BP_BITS; ++b) {
                initqsmodel(&bpFirst1[b], 2, 10, 1 << 9, nullptr, 0);
                initqsmodel(&bpAfter1[b], 2, 10, 1 << 9, nullptr, 0);
            }
            auto decBP = [&]() -> int32_t {
                uint32_t z = 0;
                bool seenOne = false;
                for (int b = BP_BITS - 1; b >= 0; --b) {
                    qsmodel* mp = seenOne ? &bpAfter1[b] : &bpFirst1[b];
                    int syfreq, ltfreq;
                    int ltval = decode_culshift(&rc, 10);
                    int sym = qsgetsym(mp, ltval);
                    qsgetfreq(mp, sym, &syfreq, &ltfreq);
                    decode_update(&rc, syfreq, ltfreq, 1 << 10);
                    qsupdate(mp, sym);
                    z = (z << 1) | (uint32_t)sym;
                    if (sym) seenOne = true;
                }
                return (int32_t)((z >> 1) ^ -(int32_t)(z & 1));
            };
            int coordsConsumed = 0;
            while (coordsConsumed < numCoords) {
                int32_t rx = decBP(), ry = decBP(), rz = decBP();
                rcDecodedResiduals_.push_back(MCGAL::PointInt(rx, ry, rz));
                coordsConsumed += 3;
            }
            done_decoding(&rc);
            for (int b = 0; b < BP_BITS; ++b) {
                deleteqsmodel(&bpFirst1[b]); deleteqsmodel(&bpAfter1[b]);
            }
        }
    }

    dataOffset += blobSize;
}

void VQMMCCodebookSymbolReadOperator::collectFromDecoded(
    MCGAL::Halfedge* seed,
    std::vector<MCGAL::Vertex*>& vertices,
    std::vector<MCGAL::Halfedge*>& halfedge,
    std::vector<MCGAL::Facet*>& facets) {
    if (!useVectorQuantizedBlob_) {
        RDTagSymbolReadOperator::collectFromDecoded(seed, vertices, halfedge, facets);
        return;
    }
    if (currentVectorQuantizedRound_ >= decodedVectorQuantizedBlob_.numRounds) {
        return;
    }
    pmsf::applyVectorQuantizedDecodedRound(
        decodedVectorQuantizedBlob_.schemeVersion,
        currentVectorQuantizedRound_,
        decodedVectorQuantizedBlob_,
        seed,
        vertices,
        halfedge,
        facets);
    ++currentVectorQuantizedRound_;
}
