// SPDX-License-Identifier: Apache-2.0
//
// PCATMPureSymbolReadOperator.cpp — see header for design contract.
//
// 复用 RDTag sv=1 的 3 段流解码逻辑，仅 blob_layout 校验 18→21。
// scheme_version 仅支持 sv=1（PR-K3 落地）；未来若需 sv=2 multi-context isPoly
// 可参考 RDTag 实现扩展。

#include "PCATM/decompress/PCATMPureSymbolReadOperator.h"
#include "common/PaperLayoutAliases.h"
#include "core/PaperProtocolHint.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂

#include "compress/ConnectivityBlobContainer.h"
#include "RD/compress/RDSymbolCollectOperator.h"  // RDValenceCode
#include "Facet.h"
#include "Halfedge.h"
#include "Point.h"

#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <queue>
#include <span>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace {

void markPCATMPurePaperFacets(std::vector<MCGAL::Facet*>& facets) {
    pmsf::markPaperStrictPatchProtocolForEach(facets);
}

std::vector<MCGAL::Halfedge*> collectActualPurePatchTreeDiagonals(
    MCGAL::Facet* root,
    int targetValence,
    const std::unordered_set<MCGAL::Facet*>& roundRoots) {
    std::vector<MCGAL::Halfedge*> diagonals;
    if (!root || root->isRemoved() || targetValence <= 3) {
        return diagonals;
    }

    const std::size_t targetFaces = static_cast<std::size_t>(targetValence - 2);
    std::unordered_set<MCGAL::Facet*> visited;
    visited.reserve(targetFaces * 2 + 8);
    std::queue<MCGAL::Facet*> q;
    visited.insert(root);
    q.push(root);

    while (!q.empty() && visited.size() < targetFaces) {
        MCGAL::Facet* cur = q.front();
        q.pop();
        MCGAL::Halfedge* start = cur ? cur->proxyHalfedge() : nullptr;
        if (!start) continue;
        MCGAL::Halfedge* h = start;
        do {
            MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
            MCGAL::Facet* nf = opp ? opp->face() : nullptr;
            if (nf && !nf->isRemoved() && !visited.count(nf) &&
                (h->isAdded() || (opp && opp->isAdded())) &&
                (!roundRoots.count(nf) || nf == root)) {
                visited.insert(nf);
                q.push(nf);
                diagonals.push_back(h);
                if (visited.size() >= targetFaces) break;
            }
            h = h ? h->next() : nullptr;
        } while (h && h != start);
    }

    if (visited.size() != targetFaces) {
        diagonals.clear();
    }
    return diagonals;
}

}  // namespace

void PCATMPureSymbolReadOperator::importFromRangeCodedBuffer(
    char* buffer,
    size_t& dataOffset,
    unsigned nbQuantBits) {

    constexpr size_t kArbitraryLargeView = static_cast<size_t>(1) << 32;
    std::span<const uint8_t> view(reinterpret_cast<const uint8_t*>(buffer + dataOffset),
                                   kArbitraryLargeView);

    pmsf::ConnectivityBlobReader rd(view);
    const auto& hdr = rd.header();
    if (!pmsf::paperBlobSchemeVersionIsPCATMPure(hdr.scheme_version)) {
        throw std::runtime_error(
            "[PCATMPure decoder] expected scheme_version=1 (PR-K3) or 2 (PR-K4), got " +
            std::to_string(hdr.scheme_version));
    }
    const auto spec = pmsf::pcatmPureConnectivityBlobSpec(
        static_cast<std::int32_t>(hdr.num_rounds),
        static_cast<std::int32_t>(hdr.scheme_version));
    pmsf::requirePaperConnectivityBlobHeaderMatchesSpec(hdr, spec, "PCATMPure");

    const int numRounds = static_cast<int>(hdr.num_rounds);
    const int sv = static_cast<int>(hdr.scheme_version);

    rcUsingResidualLocal_ = header_.isEnablePrediction() && header_.isEnableQuantization();
    if (!rcUsingResidualLocal_) {
        throw std::runtime_error(
            "[PCATMPure decoder] expects enablePrediction && enableQuantization");
    }
    const int BP_BITS = static_cast<int>(nbQuantBits) + 1;

    faceBitsPerRound_.assign(numRounds, 0);

    constexpr int lg_totf = 10;
    constexpr int rescale = 1 << 9;

    for (int r = 0; r < numRounds; ++r) {
        // ---- stream 0: valence (mirror RDTag/PCLTTMTag) ----
        {
            auto sl = rd.getStream(r, pmsf::kPaperBlobStreamPCATMPureValence);
            if (sl.size() < 8) throw std::runtime_error("[PCATMPure decoder] stream 0 too small");
            int32_t numPoly = 0, numOther = 0;
            std::memcpy(&numPoly,  sl.data(),     4);
            std::memcpy(&numOther, sl.data() + 4, 4);

            std::vector<char> rcStaging(sl.begin() + 8, sl.end());
            size_t rcOffset = 0;
            rangecoder rc;
            rc.p_data = rcStaging.data();
            rc.p_dataOffset = &rcOffset;
            if (start_decoding(&rc) != 0) {
                throw std::runtime_error("[PCATMPure decoder] valence RC start_decoding failed");
            }

            constexpr int alphabet_size = 8;
            qsmodel ms[4];
            for (int i = 0; i < 4; ++i) {
                initqsmodel(&ms[i], alphabet_size, lg_totf, rescale, nullptr, 0);
            }
            qsmodel esc;
            initqsmodel(&esc, 256, 10, 1 << 9, nullptr, 0);

            auto deriveCtx = [](int prev) {
                if (prev < 0) return 0;
                if (prev == (int)RDValenceCode::VAL_3 || prev == (int)RDValenceCode::VAL_4) return 1;
                if (prev == (int)RDValenceCode::VAL_5 || prev == (int)RDValenceCode::VAL_6) return 2;
                return 3;
            };

            int prev = -1, otherSeen = 0;
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
                    otherSeen++;
                }
                prev = sym;
            }
            done_decoding(&rc);
            for (auto& m : ms) deleteqsmodel(&m);
            deleteqsmodel(&esc);
            (void)otherSeen; (void)numOther;
        }

        // ---- stream 1: isPolyBit (sv=1: 2-元 RC; sv=2: 8-context RC, PR-K4) ----
        if (sv == pmsf::kPaperBlobSchemeVersionPCATMPure) {
            auto sl = rd.getStream(r, pmsf::kPaperBlobStreamPCATMPureIsPoly);
            if (sl.size() < 4) throw std::runtime_error("[PCATMPure decoder sv=1] isPoly too small");
            int32_t numFaceBits = 0;
            std::memcpy(&numFaceBits, sl.data(), 4);
            faceBitsPerRound_[r] = numFaceBits;

            std::vector<char> rcStaging(sl.begin() + 4, sl.end());
            size_t rcOffset = 0;
            rangecoder rc;
            rc.p_data = rcStaging.data();
            rc.p_dataOffset = &rcOffset;
            if (start_decoding(&rc) != 0) {
                throw std::runtime_error("[PCATMPure decoder sv=1] isPoly start_decoding failed");
            }
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
        } else /* sv == kPaperBlobSchemeVersionPCATMPureV2 (PR-K4 multi-context RC) */ {
            constexpr int kHistoryBits = 3;
            constexpr int kNumCtx = 1 << kHistoryBits;
            constexpr int kHistoryMask = kNumCtx - 1;

            auto sl = rd.getStream(r, pmsf::kPaperBlobStreamPCATMPureIsPoly);
            if (sl.size() < 4) throw std::runtime_error("[PCATMPure decoder sv=2] isPoly too small");
            int32_t numFaceBits = 0;
            std::memcpy(&numFaceBits, sl.data(), 4);
            faceBitsPerRound_[r] = numFaceBits;

            std::vector<char> rcStaging(sl.begin() + 4, sl.end());
            size_t rcOffset = 0;
            rangecoder rc;
            rc.p_data = rcStaging.data();
            rc.p_dataOffset = &rcOffset;
            if (start_decoding(&rc) != 0) {
                throw std::runtime_error("[PCATMPure decoder sv=2] isPoly multi-ctx start_decoding failed");
            }
            std::vector<qsmodel> models(kNumCtx);
            for (int i = 0; i < kNumCtx; ++i) {
                initqsmodel(&models[i], 2, lg_totf, rescale, nullptr, 0);
            }
            int history = 0;
            for (int k = 0; k < numFaceBits; ++k) {
                qsmodel* mp = &models[history & kHistoryMask];
                int syfreq, ltfreq;
                int ltval = decode_culshift(&rc, lg_totf);
                int sym = qsgetsym(mp, ltval);
                qsgetfreq(mp, sym, &syfreq, &ltfreq);
                decode_update(&rc, syfreq, ltfreq, 1 << lg_totf);
                qsupdate(mp, sym);
                rcDecodedIsPoly_.push_back(sym ? 1 : 0);
                history = ((history << 1) | sym) & kHistoryMask;
            }
            done_decoding(&rc);
            for (auto& m : models) deleteqsmodel(&m);
        }

        // ---- stream 2: residual (bit-plane, mirror RDTag/PCLTTMTag) ----
        {
            auto sl = rd.getStream(r, pmsf::kPaperBlobStreamPCATMPureResidual);
            if (sl.size() < 4) throw std::runtime_error("[PCATMPure decoder] residual too small");
            int32_t numCoords = 0;
            std::memcpy(&numCoords, sl.data(), 4);

            std::vector<char> rcStaging(sl.begin() + 4, sl.end());
            size_t rcOffset = 0;
            rangecoder rc;
            rc.p_data = rcStaging.data();
            rc.p_dataOffset = &rcOffset;
            if (start_decoding(&rc) != 0) {
                throw std::runtime_error("[PCATMPure decoder] residual start_decoding failed");
            }
            std::vector<qsmodel> bpFirst1(BP_BITS), bpAfter1(BP_BITS);
            for (int b = 0; b < BP_BITS; ++b) {
                initqsmodel(&bpFirst1[b], 2, 10, 1 << 9, nullptr, 0);
                initqsmodel(&bpAfter1[b], 2, 10, 1 << 9, nullptr, 0);
            }
            auto decResidualBP = [&]() -> int32_t {
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
                int32_t rx = decResidualBP();
                int32_t ry = decResidualBP();
                int32_t rz = decResidualBP();
                rcDecodedResiduals_.push_back(MCGAL::PointInt(rx, ry, rz));
                coordsConsumed += 3;
            }
            done_decoding(&rc);
            for (int b = 0; b < BP_BITS; ++b) {
                deleteqsmodel(&bpFirst1[b]);
                deleteqsmodel(&bpAfter1[b]);
            }
        }
    }

    const size_t blobSize = hdr.headerByteSize() +
                            (hdr.stream_offsets.empty() ? 0 : hdr.stream_offsets.back());
    dataOffset += blobSize;

    const char* env = std::getenv("PMSF_BPV_DIAG");
    if (env && env[0] == '1') {
        std::cerr << "[PCATMPure decoder sv=" << sv << "] rounds=" << numRounds
                  << " totalIsPoly=" << rcDecodedIsPoly_.size()
                  << " totalValCodes=" << rcDecodedValCodes_.size()
                  << " totalResiduals=" << rcDecodedResiduals_.size()
                  << " blobSize=" << blobSize << std::endl;
    }
}

void PCATMPureSymbolReadOperator::collectFromDecoded(
    MCGAL::Halfedge* seed,
    std::vector<MCGAL::Vertex*>& vertices,
    std::vector<MCGAL::Halfedge*>& halfedge,
    std::vector<MCGAL::Facet*>& facets) {
    const std::size_t oldFacetCount = facets.size();
    const std::size_t oldHalfedgeCount = halfedge.size();
    RDTagSymbolReadOperator::collectFromDecoded(seed, vertices, halfedge, facets);
    std::vector<MCGAL::Halfedge*> parentFanCandidates;
    if (halfedge.size() > oldHalfedgeCount) {
        parentFanCandidates.assign(
            halfedge.begin() + static_cast<std::ptrdiff_t>(oldHalfedgeCount),
            halfedge.end());
    }
    std::vector<MCGAL::Facet*> roundFacets;
    if (facets.size() > oldFacetCount) {
        roundFacets.assign(facets.begin() + static_cast<std::ptrdiff_t>(oldFacetCount),
                           facets.end());
    }
    markPCATMPurePaperFacets(roundFacets);

    std::unordered_set<MCGAL::Facet*> roundRoots(roundFacets.begin(), roundFacets.end());
    halfedge.resize(oldHalfedgeCount);
    std::size_t parentFanCursor = 0;
    for (MCGAL::Facet* f : roundFacets) {
        if (!f) continue;
        const int expectedDiagonals = std::max(0, f->getPolyValence() - 3);
        auto diagonals =
            collectActualPurePatchTreeDiagonals(f, f->getPolyValence(), roundRoots);
        if (diagonals.empty() && expectedDiagonals > 0 &&
            parentFanCursor + static_cast<std::size_t>(expectedDiagonals) <=
                parentFanCandidates.size()) {
            diagonals.assign(parentFanCandidates.begin() + static_cast<std::ptrdiff_t>(parentFanCursor),
                             parentFanCandidates.begin() + static_cast<std::ptrdiff_t>(
                                 parentFanCursor + static_cast<std::size_t>(expectedDiagonals)));
        }
        parentFanCursor += static_cast<std::size_t>(expectedDiagonals);
        for (MCGAL::Halfedge* h : diagonals) {
            if (!h || h->isRemoved()) continue;
            if (!h->isAdded()) h->setAdded();
            if (h->opposite() && !h->opposite()->isAdded()) h->opposite()->setAdded();
            halfedge.push_back(h);
        }
    }
}
