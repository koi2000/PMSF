/*
 * @file        PCATMColorSymbolReadOperator.cpp
 * @brief       PCATM 2-color decoder 实现（CP-3.3 β+1 落地版）。
 *
 * 与 collector 字节级对称：解 ConnectivityBlobContainer v3 的 5 个子流，
 * 投影回父类 RDGateOnly v3 协议的 4 个解码队列 + 本类 color 队列，
 * 让父类 collectFromDecoded 接管 BFS / mesh 重建。
 */

#include "PCATM/decompress/PCATMColorSymbolReadOperator.h"
#include "common/PaperLayoutAliases.h"

#include "compress/ConnectivityBlobContainer.h"
#include "RD/compress/RDSymbolCollectOperator.h"  // RDValenceCode
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂

#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"
#include "Point.h"

#include <cstring>
#include <iostream>
#include <span>
#include <stdexcept>

void PCATMColorSymbolReadOperator::importFromRangeCodedBuffer(
    char* buffer,
    size_t& dataOffset,
    unsigned nbQuantBits) {

    // ---- 1) 解 ConnectivityBlobContainer header ----
    // 与 ValencePatchOnly 同款：不知道 blob 总长，用大 span，让 reader 内部 sentinel 截断。
    constexpr size_t kArbitraryLargeView = static_cast<size_t>(1) << 32;
    std::span<const uint8_t> view(reinterpret_cast<const uint8_t*>(buffer + dataOffset),
                                   kArbitraryLargeView);

    pmsf::ConnectivityBlobReader rd(view);
    const auto& hdr = rd.header();
    const auto spec = pmsf::pcatmColorConnectivityBlobSpec(
        static_cast<std::int32_t>(hdr.num_rounds));
    pmsf::requirePaperConnectivityBlobHeaderMatchesSpec(
        hdr, spec, "PCATMColor");

    const int numRounds = static_cast<int>(hdr.num_rounds);

    // ---- 2) Quantization invariant ----
    rcUsingResidualLocal_ = header_.isEnablePrediction() && header_.isEnableQuantization();
    if (!rcUsingResidualLocal_) {
        throw std::runtime_error(
            "[PCATMColor decoder] expects enablePrediction && enableQuantization");
    }
    const int BP_BITS = static_cast<int>(nbQuantBits) + 1;

    faceBitsPerRound_.assign(numRounds, 0);

    constexpr int lg_totf = 10;
    constexpr int rescale = 1 << 9;

    // ---- 3) round-major 解 5 段流 ----
    for (int r = 0; r < numRounds; ++r) {
        // ---- stream 0: valence (5 元 + 4-context + V_OTHER inline escape) ----
        // 字节布局: [int32 numPoly][int32 numOther][bytes RC]
        {
            auto sl = rd.getStream(r, pmsf::kPaperBlobStreamPCATMColorValence);
            if (sl.size() < 8) {
                throw std::runtime_error("[PCATMColor decoder] stream 0 too small");
            }
            int32_t numPoly = 0, numOther = 0;
            std::memcpy(&numPoly,  sl.data(),     4);
            std::memcpy(&numOther, sl.data() + 4, 4);

            std::vector<char> rcStaging(sl.begin() + 8, sl.end());
            size_t rcOffset = 0;
            rangecoder rc;
            rc.p_data = rcStaging.data();
            rc.p_dataOffset = &rcOffset;
            if (start_decoding(&rc) != 0) {
                throw std::runtime_error(
                    "[PCATMColor decoder] valence RC start_decoding failed at round=" +
                    std::to_string(r));
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
                if (prev == (int)RDValenceCode::VAL_3 ||
                    prev == (int)RDValenceCode::VAL_4) return 1;
                if (prev == (int)RDValenceCode::VAL_5 ||
                    prev == (int)RDValenceCode::VAL_6) return 2;
                return 3;
            };

            int prev = -1;
            int otherSeen = 0;
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

            if (otherSeen != numOther) {
                std::cerr << "[PCATMColor decoder] WARN round=" << r
                          << " V_OTHER count mismatch (header=" << numOther
                          << " decoded=" << otherSeen << ")" << std::endl;
            }
        }

        // ---- stream 1: isPoly (2 元 RC) ----
        // 字节布局: [int32 numFaceBits][bytes RC]
        {
            auto sl = rd.getStream(r, pmsf::kPaperBlobStreamPCATMColorIsPoly);
            if (sl.size() < 4) {
                throw std::runtime_error("[PCATMColor decoder] stream 1 too small");
            }
            int32_t numFaceBits = 0;
            std::memcpy(&numFaceBits, sl.data(), 4);
            faceBitsPerRound_[r] = numFaceBits;

            std::vector<char> rcStaging(sl.begin() + 4, sl.end());
            size_t rcOffset = 0;
            rangecoder rc;
            rc.p_data = rcStaging.data();
            rc.p_dataOffset = &rcOffset;
            if (start_decoding(&rc) != 0) {
                throw std::runtime_error("[PCATMColor decoder] isPoly start_decoding failed");
            }
            qsmodel m;
            initqsmodel(&m, 2, 10, 1 << 9, nullptr, 0);
            for (int k = 0; k < numFaceBits; ++k) {
                int syfreq, ltfreq;
                int ltval = decode_culshift(&rc, 10);
                int sym = qsgetsym(&m, ltval);
                qsgetfreq(&m, sym, &syfreq, &ltfreq);
                decode_update(&rc, syfreq, ltfreq, 1 << 10);
                qsupdate(&m, sym);
                rcDecodedIsPoly_.push_back((uint8_t)sym);
            }
            done_decoding(&rc);
            deleteqsmodel(&m);
        }

        // ---- stream 2: edgeSym (2 元 RC, 3 bit/face) ----
        {
            auto sl = rd.getStream(r, pmsf::kPaperBlobStreamPCATMColorEdgeSym);
            if (sl.size() < 4) {
                throw std::runtime_error("[PCATMColor decoder] stream 2 too small");
            }
            int32_t numEdgeSyms = 0;
            std::memcpy(&numEdgeSyms, sl.data(), 4);

            std::vector<char> rcStaging(sl.begin() + 4, sl.end());
            size_t rcOffset = 0;
            rangecoder rc;
            rc.p_data = rcStaging.data();
            rc.p_dataOffset = &rcOffset;
            if (start_decoding(&rc) != 0) {
                throw std::runtime_error("[PCATMColor decoder] edgeSym start_decoding failed");
            }
            qsmodel m;
            initqsmodel(&m, 2, 10, 1 << 9, nullptr, 0);
            for (int k = 0; k < numEdgeSyms; ++k) {
                int syfreq, ltfreq;
                int ltval = decode_culshift(&rc, 10);
                int sym = qsgetsym(&m, ltval);
                qsgetfreq(&m, sym, &syfreq, &ltfreq);
                decode_update(&rc, syfreq, ltfreq, 1 << 10);
                qsupdate(&m, sym);
                rcDecodedEdgeSyms_.push_back((uint8_t)sym);
            }
            done_decoding(&rc);
            deleteqsmodel(&m);
        }

        // ---- stream 3: color (2 元 RC) ----
        {
            auto sl = rd.getStream(r, pmsf::kPaperBlobStreamPCATMColorColor);
            if (sl.size() < 4) {
                throw std::runtime_error("[PCATMColor decoder] stream 3 too small");
            }
            int32_t numColorBits = 0;
            std::memcpy(&numColorBits, sl.data(), 4);

            std::vector<char> rcStaging(sl.begin() + 4, sl.end());
            size_t rcOffset = 0;
            rangecoder rc;
            rc.p_data = rcStaging.data();
            rc.p_dataOffset = &rcOffset;
            if (start_decoding(&rc) != 0) {
                throw std::runtime_error("[PCATMColor decoder] color start_decoding failed");
            }
            qsmodel m;
            initqsmodel(&m, 2, 10, 1 << 9, nullptr, 0);
            for (int k = 0; k < numColorBits; ++k) {
                int syfreq, ltfreq;
                int ltval = decode_culshift(&rc, 10);
                int sym = qsgetsym(&m, ltval);
                qsgetfreq(&m, sym, &syfreq, &ltfreq);
                decode_update(&rc, syfreq, ltfreq, 1 << 10);
                qsupdate(&m, sym);
                rcDecodedColors_.push_back((uint8_t)sym);
            }
            done_decoding(&rc);
            deleteqsmodel(&m);
        }

        // ---- stream 4: residual (bit-plane RC) ----
        {
            auto sl = rd.getStream(r, pmsf::kPaperBlobStreamPCATMColorResidual);
            if (sl.size() < 4) {
                throw std::runtime_error("[PCATMColor decoder] stream 4 too small");
            }
            int32_t numCoords = 0;
            std::memcpy(&numCoords, sl.data(), 4);

            std::vector<char> rcStaging(sl.begin() + 4, sl.end());
            size_t rcOffset = 0;
            rangecoder rc;
            rc.p_data = rcStaging.data();
            rc.p_dataOffset = &rcOffset;
            if (start_decoding(&rc) != 0) {
                throw std::runtime_error("[PCATMColor decoder] residual start_decoding failed");
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

    // dataOffset 推进
    const size_t blobSize = hdr.headerByteSize() +
                            (hdr.stream_offsets.empty() ? 0 : hdr.stream_offsets.back());
    dataOffset += blobSize;

    const char* env = std::getenv("PMSF_BPV_DIAG");
    if (env && env[0] == '1') {
        std::cerr << "[PCATMColor decoder] rounds=" << numRounds
                  << " totalIsPoly=" << rcDecodedIsPoly_.size()
                  << " totalValCodes=" << rcDecodedValCodes_.size()
                  << " totalOtherVals=" << rcDecodedOtherVals_.size()
                  << " totalEdgeSyms=" << rcDecodedEdgeSyms_.size()
                  << " totalColors=" << rcDecodedColors_.size()
                  << " totalResiduals=" << rcDecodedResiduals_.size()
                  << " blobSize=" << blobSize << std::endl;
    }
}
