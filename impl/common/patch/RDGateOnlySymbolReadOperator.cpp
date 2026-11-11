/*
 * @file RDGateOnlySymbolReadOperator.cpp
 * @brief RD Phase B decoder — 对称消费 RDGateOnlySymbolCollectOperator v3 协议。
 *
 * 见 `src/compress/RDGateOnlySymbolCollectOperator.h` 的完整协议描述。
 */
#include "common/patch/RDGateOnlySymbolReadOperator.h"
#include "BFSVersionManager.h"
#include "BufferUtils.h"
#include "Configuration.h"
#include "Facet.h"
#include "Halfedge.h"
#include "RD/compress/RDSymbolCollectOperator.h"  // RDValenceCode 枚举
#include "core.h"

#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <queue>

void RDGateOnlySymbolReadOperator::importFromRangeCodedBuffer(char* buffer,
                                                              size_t& dataOffset,
                                                              unsigned nbQuantBits) {
    // Header 解析（严格对齐 RDGateOnlySymbolCollectOperator::exportToBufferWithRangeCoder v3）
    int rcValSize = readInt(buffer, dataOffset);
    int rcIsPolySize = readInt(buffer, dataOffset);
    int rcEdgeSize = readInt(buffer, dataOffset);
    int rcPointSize = readInt(buffer, dataOffset);
    int numFaceBits = readInt(buffer, dataOffset);
    int numGates = readInt(buffer, dataOffset);
    int numEdgeSyms = readInt(buffer, dataOffset);
    int numPointCoords = readInt(buffer, dataOffset);
    int numRounds = readInt(buffer, dataOffset);

    faceBitsPerRound_.resize(numRounds, 0);
    for (int i = 0; i < numRounds; i++) {
        faceBitsPerRound_[i] = readInt(buffer, dataOffset);
    }

    // numEdgeSyms 必须等于 numFaceBits*3（每 face 3 个 edge-sym bit）。
    if (numEdgeSyms != numFaceBits * 3) {
        std::cerr << "[RDGateOnly decoder] WARN numEdgeSyms=" << numEdgeSyms
                  << " != numFaceBits*3=" << (numFaceBits * 3)
                  << " (header inconsistent; v2 stream?)" << std::endl;
    }

    // rcUsingResidualLocal_ 必须与 encoder 的同名判定一致：
    // encoder 要求 enablePrediction && isEnableQuantization 才走 bit-plane 残差（否则抛）。
    rcUsingResidualLocal_ = header_.isEnablePrediction() && header_.isEnableQuantization();
    if (!rcUsingResidualLocal_) {
        throw std::runtime_error(
            "[RDGateOnly decoder] expects enablePrediction && enableQuantization "
            "(bit-plane residual path). metadata.parquet flags inconsistent.");
    }

    // ---- valence RC 段（含 V_OTHER inline escape）----
    {
        size_t rcOffset = 0;
        rangecoder rc;
        rc.p_data = buffer + dataOffset;
        rc.p_dataOffset = &rcOffset;
        start_decoding(&rc);

        qsmodel valModel;
        qsmodel escapeModel;
        initqsmodel(&valModel, kNumValenceCodes, 10, 1 << 9, nullptr, 0);
        initqsmodel(&escapeModel, 256, 10, 1 << 9, nullptr, 0);

        for (int k = 0; k < numGates; k++) {
            int syfreq, ltfreq;
            ltfreq = decode_culshift(&rc, 10);
            int sym = qsgetsym(&valModel, ltfreq);
            qsgetfreq(&valModel, sym, &syfreq, &ltfreq);
            decode_update(&rc, syfreq, ltfreq, 1 << 10);
            qsupdate(&valModel, sym);
            rcDecodedValCodes_.push_back((uint8_t)sym);

            if (sym == (int)RDValenceCode::VAL_OTHER) {
                int esyfreq, eltfreq;
                eltfreq = decode_culshift(&rc, 10);
                int esym = qsgetsym(&escapeModel, eltfreq);
                qsgetfreq(&escapeModel, esym, &esyfreq, &eltfreq);
                decode_update(&rc, esyfreq, eltfreq, 1 << 10);
                qsupdate(&escapeModel, esym);
                rcDecodedOtherVals_.push_back((uint8_t)esym);
            }
        }

        done_decoding(&rc);
        deleteqsmodel(&valModel);
        deleteqsmodel(&escapeModel);
    }
    dataOffset += rcValSize;

    // ---- is-polyFace RC 段（2 元 adaptive）----
    {
        size_t rcOffset = 0;
        rangecoder rc;
        rc.p_data = buffer + dataOffset;
        rc.p_dataOffset = &rcOffset;
        start_decoding(&rc);

        qsmodel isPolyModel;
        initqsmodel(&isPolyModel, 2, 10, 1 << 9, nullptr, 0);

        for (int k = 0; k < numFaceBits; k++) {
            int syfreq, ltfreq;
            ltfreq = decode_culshift(&rc, 10);
            int sym = qsgetsym(&isPolyModel, ltfreq);
            qsgetfreq(&isPolyModel, sym, &syfreq, &ltfreq);
            decode_update(&rc, syfreq, ltfreq, 1 << 10);
            qsupdate(&isPolyModel, sym);
            rcDecodedIsPoly_.push_back((uint8_t)sym);
        }

        done_decoding(&rc);
        deleteqsmodel(&isPolyModel);
    }
    dataOffset += rcIsPolySize;

    // ---- ★ v3 NEW — edge-sym RC 段（2 元 adaptive，per polyFace 3 bit）----
    {
        size_t rcOffset = 0;
        rangecoder rc;
        rc.p_data = buffer + dataOffset;
        rc.p_dataOffset = &rcOffset;
        start_decoding(&rc);

        qsmodel edgeModel;
        initqsmodel(&edgeModel, 2, 10, 1 << 9, nullptr, 0);

        for (int k = 0; k < numEdgeSyms; k++) {
            int syfreq, ltfreq;
            ltfreq = decode_culshift(&rc, 10);
            int sym = qsgetsym(&edgeModel, ltfreq);
            qsgetfreq(&edgeModel, sym, &syfreq, &ltfreq);
            decode_update(&rc, syfreq, ltfreq, 1 << 10);
            qsupdate(&edgeModel, sym);
            rcDecodedEdgeSyms_.push_back((uint8_t)sym);
        }

        done_decoding(&rc);
        deleteqsmodel(&edgeModel);
    }
    dataOffset += rcEdgeSize;

    // ---- point RC 段（bit-plane 残差；verbatim Valence 路径）----
    {
        size_t rcOffset = 0;
        rangecoder rc;
        rc.p_data = buffer + dataOffset;
        rc.p_dataOffset = &rcOffset;
        start_decoding(&rc);

        const int BP_BITS = (int)nbQuantBits + 1;
        std::vector<qsmodel> bpFirst1(BP_BITS);
        std::vector<qsmodel> bpAfter1(BP_BITS);
        for (int b = 0; b < BP_BITS; b++) {
            initqsmodel(&bpFirst1[b], 2, 10, 1 << 9, nullptr, 0);
            initqsmodel(&bpAfter1[b], 2, 10, 1 << 9, nullptr, 0);
        }

        auto decResidualBP = [&]() -> int32_t {
            uint32_t z = 0;
            bool seenOne = false;
            for (int b = BP_BITS - 1; b >= 0; b--) {
                qsmodel* m = seenOne ? &bpAfter1[b] : &bpFirst1[b];
                int syfreq, ltfreq;
                ltfreq = decode_culshift(&rc, 10);
                int sym = qsgetsym(m, ltfreq);
                qsgetfreq(m, sym, &syfreq, &ltfreq);
                decode_update(&rc, syfreq, ltfreq, 1 << 10);
                qsupdate(m, sym);
                z = (z << 1) | (uint32_t)sym;
                if (sym) seenOne = true;
            }
            return (int32_t)((z >> 1) ^ -(int32_t)(z & 1));
        };

        int coordsConsumed = 0;
        while (coordsConsumed < numPointCoords) {
            int32_t rx = decResidualBP();
            int32_t ry = decResidualBP();
            int32_t rz = decResidualBP();
            rcDecodedResiduals_.push_back(MCGAL::PointInt(rx, ry, rz));
            coordsConsumed += 3;
        }

        done_decoding(&rc);
        for (int b = 0; b < BP_BITS; b++) {
            deleteqsmodel(&bpFirst1[b]);
            deleteqsmodel(&bpAfter1[b]);
        }
    }
    dataOffset += rcPointSize;

    currentRoundIdx_ = 0;

    const char* envDiag = std::getenv("PMSF_RD_DIAG");
    if (envDiag && envDiag[0] == '1') {
        std::cerr << "[RDGateOnly decoder] imported: numFaceBits=" << numFaceBits
                  << " numGates=" << numGates << " numEdgeSyms=" << numEdgeSyms
                  << " numRounds=" << numRounds
                  << " numOtherVals=" << rcDecodedOtherVals_.size() << std::endl;
    }
}

void RDGateOnlySymbolReadOperator::collectFromDecoded(MCGAL::Halfedge* seed,
                                                      std::vector<MCGAL::Vertex*>& vertices,
                                                      std::vector<MCGAL::Halfedge*>& halfedge,
                                                      std::vector<MCGAL::Facet*>& facets) {
    // Phase B v3 协议：每 polyFace 发 3 个 edge-sym bit；decoder 端读到 1 的 halfedge
    // → setAdded() + push 到 halfedge 列表，BasicReconstructOperator::merge_face 会
    // join_face 把对角线合回 polygon，再 insert_point 在 polygon 上 fan-out（v0-fan 反演）。
    (void)vertices;

    const int roundIdx = currentRoundIdx_;

    // perf（2026-11-10）：同 RDTag，BFS 队列改预分配 vector + head 游标 FIFO。
    std::vector<int> localGateQueue;
    localGateQueue.reserve(4096);
    if (seed != nullptr) {
        localGateQueue.push_back(seed->poolId());
    }
    std::size_t localGateHead = 0;
    int current_version = MCGAL::BFSVersionManager::current_version++;

    int bfsFaceIdx = 0;
    int polyFacesThisRound = 0;

    // 本轮预期访问的 face 数（用于边界检查）
    int expectedFaceBits = (roundIdx < (int)faceBitsPerRound_.size())
                               ? faceBitsPerRound_[roundIdx]
                               : -1;

    while (localGateHead < localGateQueue.size()) {
        int hid = localGateQueue[localGateHead++];
        MCGAL::Halfedge* h =
            MCGAL::contextPool.getHalfedgeByIndexInSubPool(DEFAULT_MESH_ID, hid);
        if (h == nullptr || h->isVisited(current_version) || h->isRemoved()) {
            continue;
        }
        h->setVisited(current_version);

        MCGAL::Facet* f = h->face();
        bool faceFirstVisit = (f != nullptr) && !f->isVisited(current_version);
        if (faceFirstVisit) {
            f->setVisited(current_version);

            if (rcDecodedIsPoly_.empty()) {
                std::cerr << "[RDGateOnly decoder] is-poly stream underflow at round "
                          << roundIdx << " bfsFaceIdx=" << bfsFaceIdx << std::endl;
                break;
            }
            uint8_t isPolyBit = rcDecodedIsPoly_.front();
            rcDecodedIsPoly_.pop_front();


            int emittedValence = -1;
            MCGAL::PointInt residual(0, 0, 0);

            if (isPolyBit) {
                if (rcDecodedValCodes_.empty()) {
                    std::cerr << "[RDGateOnly decoder] valence stream underflow at round "
                              << roundIdx << std::endl;
                    break;
                }
                uint8_t code = rcDecodedValCodes_.front();
                rcDecodedValCodes_.pop_front();

                int valence;
                if (code == (uint8_t)RDValenceCode::VAL_OTHER) {
                    if (rcDecodedOtherVals_.empty()) {
                        std::cerr << "[RDGateOnly decoder] V_OTHER escape stream underflow "
                                  << "at round " << roundIdx << std::endl;
                        break;
                    }
                    valence = (int)rcDecodedOtherVals_.front();
                    rcDecodedOtherVals_.pop_front();
                } else {
                    valence = 3 + (int)code;  // V_3=0 → 3, V_4=1 → 4, V_5=2 → 5, V_6=3 → 6
                }

                if (rcDecodedResiduals_.empty()) {
                    std::cerr << "[RDGateOnly decoder] residual stream underflow at round "
                              << roundIdx << std::endl;
                    break;
                }
                residual = rcDecodedResiduals_.front();
                rcDecodedResiduals_.pop_front();

                f->setSplittable();
                f->setPolyValence(valence);
                f->setRemovedVertexPosInt(residual);
                facets.push_back(f);
                polyFacesThisRound++;
                emittedValence = valence;
                (void)valence;  // 警告抑制；valence 已写入 face 的 polyValence 字段
            }

            // ★ v3 — 消费 3 个 edge-sym bit（每 face 都消费，与 encoder 严格对称）。
            //   encoder 端对每个 face 以 BFS 入口 `h` 为起点沿 next() 走 3 次，
            //   每条 halfedge 各发 1 bit isAdded()。decoder 端这里同样从 `h` 起
            //   next() 三步消费。bit=1 表示该 halfedge 是 v0-fan triangulation 的
            //   对角线 → setAdded() + push 到 halfedge 列表，让
            //   BasicReconstructOperator::merge_face 调 join_face 把它合回 polygon。
            //   v0-fan 的 N-3 条对角线分布在：
            //     - 1 条在 polyFace 自己内（fan_triangle_0 的 closing edge）
            //     - N-4 条在邻接 fan-child face 内（每个 fan_triangle_i 各 1 条）
            //   所以必须每 face 都消费 edge-sym 才能找全所有对角线。
            //   完全对称 ValenceSymbolReadOperator::collectFromDecoded:324-337。
            {
                MCGAL::Halfedge* hit = h;
                for (int k = 0; k < 3 && hit != nullptr; ++k) {
                    if (rcDecodedEdgeSyms_.empty()) {
                        std::cerr << "[RDGateOnly decoder] edge-sym stream underflow "
                                  << "at round " << roundIdx
                                  << " bfsFaceIdx=" << bfsFaceIdx
                                  << " k=" << k << std::endl;
                        break;
                    }
                    uint8_t esym = rcDecodedEdgeSyms_.front();
                    rcDecodedEdgeSyms_.pop_front();
                    if (esym) {
                        hit->setAdded();
                        halfedge.push_back(hit);
                    }
                    hit = hit->next();
                }
            }

            bfsFaceIdx++;
        }

        // BFS neighbor enqueue（严格对称 encoder）
        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp != nullptr && !hOpp->isVisited(current_version)) {
                localGateQueue.push_back(hOpp->poolId());
            }
            if (hIt != nullptr && !hIt->isVisited(current_version)) {
                localGateQueue.push_back(hIt->poolId());
            }
            hIt = hIt->next();
        } while (hIt != h && hIt != nullptr);
    }


    if (expectedFaceBits >= 0 && bfsFaceIdx != expectedFaceBits) {
        std::cerr << "[RDGateOnly decoder] WARN round=" << roundIdx
                  << " bfsFaceIdx=" << bfsFaceIdx
                  << " expectedFaceBits=" << expectedFaceBits
                  << " (BFS diverge; mesh state broken upstream?)" << std::endl;
    }

    const char* envDiag = std::getenv("PMSF_RD_DIAG");
    if (envDiag && envDiag[0] == '1') {
        std::cerr << "[RDGateOnly-dec round=" << roundIdx
                  << "] faces_bfs=" << bfsFaceIdx
                  << " polyFaces=" << polyFacesThisRound
                  << " expectedFaceBits=" << expectedFaceBits
                  << std::endl;
    }

    currentRoundIdx_++;
}
