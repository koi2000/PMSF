// SPDX-License-Identifier: Apache-2.0
//
// MMSEDualRingSymbolReadOperator.cpp — MMSE dual-ring protocol decoder.
//
// Accepts BlobLayout=22. sv=1 is the historical RDTag-compatible stream;
// sv=2 carries the experimental geometry mode stream; sv=3 is the adaptive
// RDTag-compatible residual stream; sv=4 is adaptive fixed dual-ring residual;
// sv=8 is the mixed-stream variant with multi-context global isPoly.

#include "MMSEDualRingSymbolReadOperator.h"
#include "common/PaperLayoutAliases.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂

#include "BFSVersionManager.h"
#include "ContextPool.h"
#include "Facet.h"
#include "Halfedge.h"
#include "compress/ConnectivityBlobContainer.h"
#include "compress/ResidualAxisContext.h"
#include "predict/MMSEPredictor.h"
#include "RD/compress/RDSymbolCollectOperator.h"
#include "Vertex.h"
#include "Point.h"

#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"

#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <queue>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {


}  // namespace

void MMSEDualRingSymbolReadOperator::importFromRangeCodedBuffer(
    char* buffer, size_t& dataOffset, unsigned nbQuantBits) {

    // Strict paper-aligned path: flat binary blob with magic "MMSE"
    if (std::memcmp(buffer + dataOffset, "MMSE", 4) == 0) {
        auto ri=[&](size_t o)->int32_t{int32_t v;std::memcpy(&v,buffer+dataOffset+o,4);return v;};
        if (ri(4) != pmsf::kPaperBlobLayoutMMSEStrict) {
            throw std::runtime_error(
                "[MMSE-DR dec] expected strict blob_layout=" +
                std::to_string(pmsf::kPaperBlobLayoutMMSEStrict) + " got " +
                std::to_string(ri(4)));
        }
        if (ri(8) != pmsf::kPaperBlobSchemeVersionMMSEStrict) {
            throw std::runtime_error(
                "[MMSE-DR dec] expected strict scheme_version=" +
                std::to_string(pmsf::kPaperBlobSchemeVersionMMSEStrict) +
                " got " + std::to_string(ri(8)));
        }
        int nr=ri(12),tc=ri(16),cs=ri(20),tpf=ri(24),rs=ri(28);(void)tc;(void)tpf;
        std::vector<int32_t> cpr(nr);for(int i=0;i<nr;++i)cpr[i]=ri(32+i*4);
        size_t hdr=32+nr*4,base=dataOffset;
        const uint8_t*csb=reinterpret_cast<const uint8_t*>(buffer+base+hdr);
        const uint8_t*rsb=csb+cs;

        std::vector<std::deque<uint8_t>> allCodes;
        {std::vector<char> st(csb,csb+cs);size_t o2=0;rangecoder rc;rc.p_data=st.data();rc.p_dataOffset=&o2;
         if(start_decoding(&rc)!=0)throw std::runtime_error("[MMSE-DR dec] code RC");
         qsmodel m;initqsmodel(&m,5,10,1<<9,nullptr,0);
         auto fs=[](int s)->uint8_t{return s==0?0:(uint8_t)(s+2);};
         for(int r=0;r<nr;++r){std::deque<uint8_t> rcodes;for(int k=0;k<cpr[r];++k){int sf,lf;int lv=decode_culshift(&rc,10);int s=qsgetsym(&m,lv);qsgetfreq(&m,s,&sf,&lf);decode_update(&rc,sf,lf,1<<10);qsupdate(&m,s);rcodes.push_back(fs(s));}allCodes.push_back(std::move(rcodes));}
         done_decoding(&rc);deleteqsmodel(&m);}

        std::vector<std::deque<MCGAL::PointInt>> allRes;
        {std::vector<char> st(rsb,rsb+rs);size_t o2=0;rangecoder rc;rc.p_data=st.data();rc.p_dataOffset=&o2;
         if(start_decoding(&rc)!=0)throw std::runtime_error("[MMSE-DR dec] res RC");
         int bp=(int)nbQuantBits+1;std::vector<qsmodel> f1(bp),a1(bp);
         for(int b=0;b<bp;++b){initqsmodel(&f1[b],2,10,1<<9,nullptr,0);initqsmodel(&a1[b],2,10,1<<9,nullptr,0);}
         auto dbp=[&]()->int32_t{uint32_t z=0;bool so=false;for(int b=bp-1;b>=0;--b){qsmodel*mp=so?&a1[b]:&f1[b];int sf,lf;int lv=decode_culshift(&rc,10);int s=qsgetsym(mp,lv);qsgetfreq(mp,s,&sf,&lf);decode_update(&rc,sf,lf,1<<10);qsupdate(mp,s);z=(z<<1)|(uint32_t)s;if(s)so=true;}return(int32_t)((z>>1)^-(int32_t)(z&1));};
         for(int r=0;r<nr;++r){std::deque<MCGAL::PointInt> rres;for(uint8_t c:allCodes[r]){if(c!=0)rres.push_back(MCGAL::PointInt(dbp(),dbp(),dbp()));}allRes.push_back(std::move(rres));}
         done_decoding(&rc);for(int b=0;b<bp;++b){deleteqsmodel(&f1[b]);deleteqsmodel(&a1[b]);}}

        rcDecodedIsPoly_.clear();rcDecodedValCodes_.clear();rcDecodedOtherVals_.clear();rcDecodedResiduals_.clear();faceBitsPerRound_.clear();
        for(int r=0;r<nr;++r){auto&codes=allCodes[r];auto&res=allRes[r];size_t ri2=0;std::deque<uint8_t> ip,vc;std::deque<MCGAL::PointInt> pt;
         for(uint8_t c:codes){if(c==0){ip.push_back(0);}else{ip.push_back(1);int valence=(int)c;RDValenceCode rdc;switch(valence){case 3:rdc=RDValenceCode::VAL_3;break;case 4:rdc=RDValenceCode::VAL_4;break;case 5:rdc=RDValenceCode::VAL_5;break;case 6:rdc=RDValenceCode::VAL_6;break;default:rdc=RDValenceCode::VAL_OTHER;break;}vc.push_back((uint8_t)rdc);if(ri2<res.size())pt.push_back(res[ri2++]);}}
         rcDecodedIsPoly_.insert(rcDecodedIsPoly_.end(),ip.begin(),ip.end());rcDecodedValCodes_.insert(rcDecodedValCodes_.end(),vc.begin(),vc.end());rcDecodedResiduals_.insert(rcDecodedResiduals_.end(),pt.begin(),pt.end());faceBitsPerRound_.push_back((int32_t)codes.size());}
        dataOffset+=hdr+cs+rs;
        return;
    }

    constexpr size_t kLargeView = static_cast<size_t>(1) << 32;
    std::span<const uint8_t> view(
        reinterpret_cast<const uint8_t*>(buffer + dataOffset), kLargeView);

    pmsf::ConnectivityBlobReader rd(view);
    const auto& hdr = rd.header();
    if (!pmsf::paperBlobSchemeVersionIsMMSEDualRing(hdr.scheme_version)) {
        throw std::runtime_error(
            "[MMSE-DR decoder] expected scheme_version=1/2/3/4/5/6/7/8, got " +
            std::to_string(hdr.scheme_version));
    }
    const auto spec = pmsf::mmseDualRingConnectivityBlobSpec(
        static_cast<std::int32_t>(hdr.num_rounds),
        static_cast<std::int32_t>(hdr.scheme_version),
        pmsf::paperBlobStreamCountMMSEDualRingForScheme(hdr.scheme_version));
    pmsf::requirePaperConnectivityBlobHeaderMatchesSpec(hdr, spec, "MMSE-DR");
    const int sv = static_cast<int>(hdr.scheme_version);
    // P8 / D1（sv 日落）：门禁只发射 V3 ⇒ 这两项恒 false（原为 sv==V2 / sv==V4）。
    rcDecodedModes_.clear();

    const int numRounds = static_cast<int>(hdr.num_rounds);
    // P1 (2026-10-22): per-round MMSE paper predictor mode lists (parsed below
    // from the self-describing residual-stream flag 0x20000000).
    rcDecodedPredModeQueues_.assign(static_cast<std::size_t>(numRounds),
                                    std::vector<std::uint8_t>{});
    rcDecodedPredClusterSizes_.assign(static_cast<std::size_t>(numRounds), 0);
    rcPredModeStreamPresent_ = false;
    rcUsingResidualLocal_ = header_.isEnablePrediction() && header_.isEnableQuantization();
    if (!rcUsingResidualLocal_) {
        throw std::runtime_error(
            "[MMSE-DR decoder] expects enablePrediction && enableQuantization");
    }
    const int BP_BITS = static_cast<int>(nbQuantBits) + 1;

    faceBitsPerRound_.assign(numRounds, 0);

    constexpr int lg_totf = 10;
    constexpr int rescale = 1 << 9;

    // ---- Decode each round: mirror of RDTag reader ----
    for (int r = 0; r < numRounds; ++r) {
        // stream 0: valence
        {
            auto sl = rd.getStream(
                r, pmsf::kPaperBlobStreamMMSEDualRingValence);
            if (sl.size() < 8)
                throw std::runtime_error("[MMSE-DR decoder] stream 0 too small");
            int32_t numPoly = 0, numOther = 0;
            std::memcpy(&numPoly,  sl.data(),     4);
            std::memcpy(&numOther, sl.data() + 4, 4);

            std::vector<char> rcStaging(sl.begin() + 8, sl.end());
            size_t rcOffset = 0;
            rangecoder rc;
            rc.p_data = rcStaging.data();
            rc.p_dataOffset = &rcOffset;
            if (start_decoding(&rc) != 0)
                throw std::runtime_error("[MMSE-DR decoder] valence RC start failed");

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

        // stream 1: isPolyBit
        {
            auto sl = rd.getStream(
                r, pmsf::kPaperBlobStreamMMSEDualRingIsPoly);
            if (sl.size() < 4)
                throw std::runtime_error("[MMSE-DR decoder] isPoly too small");
            int32_t numFaceBits = 0;
            std::memcpy(&numFaceBits, sl.data(), 4);
            faceBitsPerRound_[r] = numFaceBits;

            std::vector<char> rcStaging(sl.begin() + 4, sl.end());
            size_t rcOffset = 0;
            rangecoder rc;
            rc.p_data = rcStaging.data();
            rc.p_dataOffset = &rcOffset;
            if (start_decoding(&rc) != 0)
                throw std::runtime_error("[MMSE-DR decoder] isPoly start failed");

            // P8 / D1（sv 日落）：原为 useIsPolyContext(sv==V5) ? 多上下文模型 : 单模型，
            // V5 已日落 ⇒ 只保留门禁（V3）走的单模型分支。
            qsmodel m;
            initqsmodel(&m, 2, lg_totf, rescale, nullptr, 0);
            int history = 0;
            for (int k = 0; k < numFaceBits; ++k) {
                int syfreq, ltfreq;
                int ltval = decode_culshift(&rc, lg_totf);
                qsmodel* mp = &m;
                int sym = qsgetsym(mp, ltval);
                qsgetfreq(mp, sym, &syfreq, &ltfreq);
                decode_update(&rc, syfreq, ltfreq, 1 << lg_totf);
                qsupdate(mp, sym);
                rcDecodedIsPoly_.push_back(sym ? 1 : 0);
            }
            done_decoding(&rc);
            deleteqsmodel(&m);
        }

        // P8 / D1：原 `if (sv == V2) { decodeMode4 ... }` 已删除（mode 流只属 sv=2）。

        // stream 2/3: residual (bit-plane)
        {
            // V3 用 plain residual 流（原为 sv==V2 ? WithMode : plain，恒取 plain）。
            const int residualStream = pmsf::kPaperBlobStreamMMSEDualRingResidual;
            auto sl = rd.getStream(r, residualStream);
            if (sl.size() < 4)
                throw std::runtime_error("[MMSE-DR decoder] residual too small");
            std::uint32_t rawNumCoords = 0;
            std::memcpy(&rawNumCoords, sl.data(), 4);
            const bool axisCtx = (rawNumCoords & 0x80000000u) != 0;
            const bool hashCtx = (rawNumCoords & 0x40000000u) != 0;
            // P1 (2026-10-22): MMSE paper predictor stack — per-cluster 2-bit
            // mode stream appended after the 8-byte residual header.
            const bool predModeStream = (rawNumCoords & 0x20000000u) != 0;
            int32_t numCoords = static_cast<std::int32_t>(rawNumCoords & 0x1fffffffu);
            // V3 残差头校验（原为 V3‖V4‖V5 —— V3 是门禁本体，其余已日落）。
            if (sv == pmsf::kPaperBlobSchemeVersionMMSEDualRingV3) {
                if (sl.size() < 8) {
                    throw std::runtime_error("[MMSE-DR decoder sv=3] residual header too small");
                }
                if (numCoords < 0 || (numCoords % 3) != 0) {
                    throw std::runtime_error("[MMSE-DR decoder sv=3] invalid residual count");
                }
                const int numResiduals = numCoords / 3;
                const int bits[3] = {
                    static_cast<int>(sl[4]),
                    static_cast<int>(sl[5]),
                    static_cast<int>(sl[6]),
                };
                const int residualDropBits = static_cast<int>(sl[7]);
                // P1: parse the optional per-cluster predictor mode stream.
                std::size_t residualHeaderBytes = 8;
                if (predModeStream) {
                    if (sl.size() < 9) {
                        throw std::runtime_error(
                            "[MMSE-DR decoder sv=3] predictor mode header truncated");
                    }
                    const std::uint8_t modeSpec = sl[8];
                    if (modeSpec != 0xFFu) {
                        // Fixed mode across the whole round (single cluster).
                        if (modeSpec >= 4) {
                            throw std::runtime_error(
                                "[MMSE-DR decoder sv=3] invalid fixed predictor mode");
                        }
                        rcDecodedPredModeQueues_[static_cast<std::size_t>(r)] =
                            std::vector<std::uint8_t>{modeSpec};
                        rcDecodedPredClusterSizes_[static_cast<std::size_t>(r)] =
                            numResiduals > 0 ? numResiduals : 1;
                        rcPredModeStreamPresent_ = true;
                        residualHeaderBytes = 9;
                    } else {
                        if (sl.size() < 11) {
                            throw std::runtime_error(
                                "[MMSE-DR decoder sv=3] predictor cluster header truncated");
                        }
                        std::uint16_t predClusterSize = 0;
                        std::memcpy(&predClusterSize, sl.data() + 9, 2);
                        if (predClusterSize == 0) {
                            throw std::runtime_error(
                                "[MMSE-DR decoder sv=3] invalid predictor cluster size");
                        }
                        const std::size_t numClusters =
                            (static_cast<std::size_t>(numResiduals) +
                             predClusterSize - 1) /
                            predClusterSize;
                        const std::size_t modeBytes = (numClusters + 3) / 4;
                        if (sl.size() < 11 + modeBytes) {
                            throw std::runtime_error(
                                "[MMSE-DR decoder sv=3] predictor mode table truncated");
                        }
                        std::vector<std::uint8_t> roundModes(numClusters, 0);
                        for (std::size_t c = 0; c < numClusters; ++c) {
                            const std::uint8_t packed = sl[11 + (c >> 2)];
                            roundModes[c] = static_cast<std::uint8_t>(
                                (packed >> (2 * static_cast<unsigned>(c & 3))) & 0x3u);
                        }
                        rcDecodedPredModeQueues_[static_cast<std::size_t>(r)] =
                            std::move(roundModes);
                        rcDecodedPredClusterSizes_[static_cast<std::size_t>(r)] =
                            static_cast<std::int32_t>(predClusterSize);
                        rcPredModeStreamPresent_ = true;
                        residualHeaderBytes = 11 + modeBytes;
                    }
                }
                const int offsets[3] = {0, bits[0], bits[0] + bits[1]};
                const int numCtx = bits[0] + bits[1] + bits[2];
                std::vector<uint32_t> zc[3];
                std::vector<unsigned char> seen[3];
                for (int axis = 0; axis < 3; ++axis) {
                    zc[axis].assign(static_cast<size_t>(numResiduals), 0);
                    seen[axis].assign(static_cast<size_t>(numResiduals), 0);
                }
                if (numResiduals > 0 && numCtx > 0) {
                    std::vector<char> rcStaging(sl.begin() + residualHeaderBytes, sl.end());
                    size_t rcOffset = 0;
                    rangecoder rc;
                    rc.p_data = rcStaging.data();
                    rc.p_dataOffset = &rcOffset;
                    if (start_decoding(&rc) != 0) {
                        throw std::runtime_error("[MMSE-DR decoder sv=3] residual start failed");
                    }
                    std::vector<qsmodel> bpFirst1(numCtx), bpAfter1(numCtx);
                    for (int i = 0; i < numCtx; ++i) {
                        initqsmodel(&bpFirst1[i], 2, 10, 1 << 9, nullptr, 0);
                        initqsmodel(&bpAfter1[i], 2, 10, 1 << 9, nullptr, 0);
                    }
                    std::vector<qsmodel> bpYFirst1, bpYAfter1, bpZFirst1, bpZAfter1;
                    std::vector<uint8_t> axisBucketX, axisBucketXY;
                    std::vector<uint16_t> hashY, hashXY;
                    constexpr int kHashY = 1;
                    constexpr int kHashZ = 32;
                    if (hashCtx) {
                        const int yModels = bits[1] * kHashY;
                        const int zModels = bits[2] * kHashZ;
                        bpYFirst1.resize(yModels); bpYAfter1.resize(yModels);
                        bpZFirst1.resize(zModels); bpZAfter1.resize(zModels);
                        for (int i = 0; i < yModels; ++i) {
                            initqsmodel(&bpYFirst1[i], 2, 10, 1 << 9, nullptr, 0);
                            initqsmodel(&bpYAfter1[i], 2, 10, 1 << 9, nullptr, 0);
                        }
                        for (int i = 0; i < zModels; ++i) {
                            initqsmodel(&bpZFirst1[i], 2, 10, 1 << 9, nullptr, 0);
                            initqsmodel(&bpZAfter1[i], 2, 10, 1 << 9, nullptr, 0);
                        }
                        hashY.assign(static_cast<size_t>(numResiduals), 0);
                        hashXY.assign(static_cast<size_t>(numResiduals), 0);
                    } else if (axisCtx) {
                        const int yModels = bits[1] * pmsf::kResidualAxisBucketCount;
                        const int zModels = bits[2] * pmsf::kResidualAxisBucketCount;
                        bpYFirst1.resize(yModels); bpYAfter1.resize(yModels);
                        bpZFirst1.resize(zModels); bpZAfter1.resize(zModels);
                        for (int i = 0; i < yModels; ++i) {
                            initqsmodel(&bpYFirst1[i], 2, 10, 1 << 9, nullptr, 0);
                            initqsmodel(&bpYAfter1[i], 2, 10, 1 << 9, nullptr, 0);
                        }
                        for (int i = 0; i < zModels; ++i) {
                            initqsmodel(&bpZFirst1[i], 2, 10, 1 << 9, nullptr, 0);
                            initqsmodel(&bpZAfter1[i], 2, 10, 1 << 9, nullptr, 0);
                        }
                        axisBucketX.assign(static_cast<size_t>(numResiduals), 0);
                        axisBucketXY.assign(static_cast<size_t>(numResiduals), 0);
                    }
                    for (int axis = 0; axis < 3; ++axis) {
                        if (hashCtx && axis == 1) {
                            for (int i = 0; i < numResiduals; ++i) {
                                const std::uint32_t zz = zc[0][static_cast<size_t>(i)];
                                const std::int32_t rx = static_cast<std::int32_t>(
                                    (zz >> 1) ^ -(std::int32_t)(zz & 1u));
                                const uint32_t ux = static_cast<uint32_t>(rx);
                                hashY[static_cast<size_t>(i)] = static_cast<uint16_t>(
                                    (ux * 2654435761u) & (kHashY - 1));
                            }
                        } else if (hashCtx && axis == 2) {
                            for (int i = 0; i < numResiduals; ++i) {
                                const std::uint32_t zx = zc[0][static_cast<size_t>(i)];
                                const std::uint32_t zy = zc[1][static_cast<size_t>(i)];
                                const std::int32_t rx = static_cast<std::int32_t>(
                                    (zx >> 1) ^ -(std::int32_t)(zx & 1u));
                                const std::int32_t ry = static_cast<std::int32_t>(
                                    (zy >> 1) ^ -(std::int32_t)(zy & 1u));
                                const uint32_t ux = static_cast<uint32_t>(rx);
                                const uint32_t uy = static_cast<uint32_t>(ry);
                                hashXY[static_cast<size_t>(i)] = static_cast<uint16_t>(
                                    ((ux * 73856093u) ^ (uy * 19349663u)) & (kHashZ - 1));
                            }
                        } else if (axisCtx && axis == 1) {
                            for (int i = 0; i < numResiduals; ++i) {
                                const std::uint32_t zz = zc[0][static_cast<size_t>(i)];
                                const std::int32_t rx = static_cast<std::int32_t>(
                                    (zz >> 1) ^ -(std::int32_t)(zz & 1u));
                                axisBucketX[static_cast<size_t>(i)] =
                                    static_cast<uint8_t>(pmsf::residualAxisBucket(rx));
                            }
                        } else if (axisCtx && axis == 2) {
                            for (int i = 0; i < numResiduals; ++i) {
                                const std::uint32_t zx = zc[0][static_cast<size_t>(i)];
                                const std::uint32_t zy = zc[1][static_cast<size_t>(i)];
                                const std::int32_t rx = static_cast<std::int32_t>(
                                    (zx >> 1) ^ -(std::int32_t)(zx & 1u));
                                const std::int32_t ry = static_cast<std::int32_t>(
                                    (zy >> 1) ^ -(std::int32_t)(zy & 1u));
                                const int bx = pmsf::residualAxisBucket(rx);
                                const int by = pmsf::residualAxisBucket(ry);
                                axisBucketXY[static_cast<size_t>(i)] = static_cast<uint8_t>(
                                    bx * pmsf::kResidualAxisBucketCount + by);
                            }
                        }
                        const int width = bits[axis];
                        if (width == 0) continue;
                        for (int b = width - 1; b >= 0; --b) {
                            const int mi = offsets[axis] + b;
                            for (int i = 0; i < numResiduals; ++i) {
                                qsmodel* mp = nullptr;
                                if (hashCtx && axis == 1) {
                                    const int idx = b * kHashY +
                                                    hashY[static_cast<size_t>(i)];
                                    mp = seen[axis][static_cast<size_t>(i)]
                                             ? &bpYAfter1[idx] : &bpYFirst1[idx];
                                } else if (hashCtx && axis == 2) {
                                    const int idx = b * kHashZ +
                                                    hashXY[static_cast<size_t>(i)];
                                    mp = seen[axis][static_cast<size_t>(i)]
                                             ? &bpZAfter1[idx] : &bpZFirst1[idx];
                                } else if (axisCtx && axis == 1) {
                                    const int idx = b * pmsf::kResidualAxisBucketCount +
                                                    axisBucketX[static_cast<size_t>(i)];
                                    mp = seen[axis][static_cast<size_t>(i)]
                                             ? &bpYAfter1[idx] : &bpYFirst1[idx];
                                } else if (axisCtx && axis == 2) {
                                    const int idx = b * pmsf::kResidualAxisBucketCount +
                                                    axisBucketX[static_cast<size_t>(i)];
                                    mp = seen[axis][static_cast<size_t>(i)]
                                             ? &bpZAfter1[idx] : &bpZFirst1[idx];
                                } else {
                                    mp = seen[axis][static_cast<size_t>(i)]
                                             ? &bpAfter1[mi] : &bpFirst1[mi];
                                }
                                int ltval = decode_culshift(&rc, 10);
                                int sym = qsgetsym(mp, ltval);
                                int sf, lf;
                                qsgetfreq(mp, sym, &sf, &lf);
                                decode_update(&rc, sf, lf, 1 << 10);
                                qsupdate(mp, sym);
                                if (sym) {
                                    seen[axis][static_cast<size_t>(i)] = 1;
                                    zc[axis][static_cast<size_t>(i)] |= (1u << b);
                                }
                            }
                        }
                    }
                    done_decoding(&rc);
                    for (int i = 0; i < numCtx; ++i) {
                        deleteqsmodel(&bpFirst1[i]);
                        deleteqsmodel(&bpAfter1[i]);
                    }
                    for (auto& m : bpYFirst1) deleteqsmodel(&m);
                    for (auto& m : bpYAfter1) deleteqsmodel(&m);
                    for (auto& m : bpZFirst1) deleteqsmodel(&m);
                    for (auto& m : bpZAfter1) deleteqsmodel(&m);
                }
                const int32_t scale = residualDropBits > 0 ? (1 << residualDropBits) : 1;
                for (int i = 0; i < numResiduals; ++i) {
                    const int32_t rx = static_cast<int32_t>(
                        (zc[0][static_cast<size_t>(i)] >> 1) ^
                        -(int32_t)(zc[0][static_cast<size_t>(i)] & 1));
                    const int32_t ry = static_cast<int32_t>(
                        (zc[1][static_cast<size_t>(i)] >> 1) ^
                        -(int32_t)(zc[1][static_cast<size_t>(i)] & 1));
                    const int32_t rz = static_cast<int32_t>(
                        (zc[2][static_cast<size_t>(i)] >> 1) ^
                        -(int32_t)(zc[2][static_cast<size_t>(i)] & 1));
                    rcDecodedResiduals_.push_back(MCGAL::PointInt(rx * scale,
                                                                  ry * scale,
                                                                  rz * scale));
                }
                continue;
            }

            std::vector<char> rcStaging(sl.begin() + 4, sl.end());
            size_t rcOffset = 0;
            rangecoder rc;
            rc.p_data = rcStaging.data();
            rc.p_dataOffset = &rcOffset;
            if (start_decoding(&rc) != 0)
                throw std::runtime_error("[MMSE-DR decoder] residual start failed");

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

    const size_t blobSize = hdr.headerByteSize() +
                            (hdr.stream_offsets.empty() ? 0 : hdr.stream_offsets.back());
    dataOffset += blobSize;

    const char* env = std::getenv("PMSF_BPV_DIAG");
    if (env && env[0] == '1') {
        std::cerr << "[MMSE-DR decoder sv=" << sv << "] rounds=" << numRounds
                  << " totalValCodes=" << rcDecodedValCodes_.size()
                  << " totalModes=" << rcDecodedModes_.size()
                  << " totalResiduals=" << rcDecodedResiduals_.size()
                  << " blobSize=" << blobSize << std::endl;
    }
}

void MMSEDualRingSymbolReadOperator::collectFromDecoded(
    MCGAL::Halfedge* seed,
    std::vector<MCGAL::Vertex*>& vertices,
    std::vector<MCGAL::Halfedge*>& halfedge,
    std::vector<MCGAL::Facet*>& facets) {
    // P11（2026-09-24）：本方法原有 152 行自有实现，属 sv=2/v=4 的**双环**解码
    // 路径。该路径在门禁下**已证不可达**（P11 §66.3 插桩实测：在旧体首行插
    // fprintf+abort，门禁配置直跑 `D_RC=0` 且探针未触发；把条件强制改为 false
    // 的阳性对照则 `POS_RC=134` 且探针命中 ⇒ 判据可失败）。故收敛为纯委托。
    //
    // 委托目标即门禁路径：PCLTTM 的 tag 解码器（原 early-return 分支原样保留）。
    // P16 插桩：确认本 override 是否真的被调用（纯 stderr，不改行为）。

    // ------------------------------------------------------------------
    // P1 (2026-10-22, WS-B4): install the MMSE paper predictor replay round.
    //
    // When the residual stream carried the per-cluster mode table (flag
    // 0x20000000), the PCLTTM-compatible BFS below calls the predictor hook
    // (src/predict/RDPredictor.h::rdCurvaturePrediction) once per polyFace in
    // deterministic BFS order; the hook consumes the mode list we install here.
    // The encoder's post-pass (`applyMMSEPredictorStack`) used the identical
    // order to decide the modes and write `encoded = r1 - correction`, so the
    // reconstruction below is bit-symmetric.
    // ------------------------------------------------------------------
    pmsf::mmse_pred::clearReplayState();
    if (rcPredModeStreamPresent_) {
        const int round = currentRoundIdx_;
        if (round >= 0 &&
            static_cast<std::size_t>(round) < rcDecodedPredModeQueues_.size() &&
            !rcDecodedPredModeQueues_[static_cast<std::size_t>(round)].empty() &&
            rcDecodedPredClusterSizes_[static_cast<std::size_t>(round)] > 0) {
            pmsf::mmse_pred::setQuantContext(bboxMin_, f_quantStep_);
            pmsf::mmse_pred::ReplayState& st = pmsf::mmse_pred::replayState();
            st.enabled = true;
            st.clusterSize = rcDecodedPredClusterSizes_[static_cast<std::size_t>(round)];
            st.modes = &rcDecodedPredModeQueues_[static_cast<std::size_t>(round)];
            st.callIndex = 0;
        }
    }
    PCLTTMTagSymbolReadOperator::collectFromDecoded(seed, vertices, halfedge, facets);
    pmsf::mmse_pred::clearReplayState();
    pmsf::mmse_pred::clearQuantContext();
}
