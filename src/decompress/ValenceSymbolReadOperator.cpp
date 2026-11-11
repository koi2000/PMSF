/*
 * @Description: Valence-code symbol reader 实现（与 ValenceSymbolCollectOperator 对称）
 */
#include "ValenceSymbolReadOperator.h"
#include "BFSVersionManager.h"
#include "BufferUtils.h"
#include "Configuration.h"
#include "Facet.h"
#include "Halfedge.h"
#include "MeshUtils.h"
#include "compress/ValenceSymbolCollectOperator.h"
#include "core.h"

#include <cassert>
#include <cstdint>
#include <cstring>
#include <iostream>

void ValenceSymbolReadOperator::collect(MCGAL::Halfedge* /*seed*/,
                                        char* /*buffer*/,
                                        size_t& /*dataOffset*/,
                                        std::vector<MCGAL::Vertex*>& /*vertices*/,
                                        std::vector<MCGAL::Halfedge*>& /*halfedge*/,
                                        std::vector<MCGAL::Facet*>& /*facets*/) {
    // 目前只实现 range-coded 路径；plain 路径预留给未来非预测版本（与 exportToBuffer 对称）。
    assert(false && "ValenceSymbolReadOperator: only range-coded path is supported");
}

void ValenceSymbolReadOperator::collect(std::vector<MCGAL::Halfedge*> /*seeds*/,
                                        char* /*buffer*/,
                                        size_t& /*dataOffset*/,
                                        std::vector<MCGAL::Vertex*>& /*vertices*/,
                                        std::vector<MCGAL::Halfedge*>& /*halfedge*/,
                                        std::vector<MCGAL::Facet*>& /*facets*/) {
    assert(false && "ValenceSymbolReadOperator: only range-coded path is supported");
}

void ValenceSymbolReadOperator::importFromRangeCodedBuffer(char* buffer,
                                                           size_t& dataOffset,
                                                           unsigned nbQuantBits) {
    // header 字段顺序必须与 ValenceSymbolCollectOperator::exportToBufferWithRangeCoder 对齐
    int compCodeSize = readInt(buffer, dataOffset);
    int compEdgeSize = readInt(buffer, dataOffset);
    int compVertexSize = 0;
    if (header_.isEnableAdjustment()) {
        compVertexSize = readInt(buffer, dataOffset);
    }
    int compPointSize = readInt(buffer, dataOffset);

    int numCodes = readInt(buffer, dataOffset);
    int numEdgeSymbols = readInt(buffer, dataOffset);
    int numVertexSymbols = 0;
    if (header_.isEnableAdjustment()) {
        numVertexSymbols = readInt(buffer, dataOffset);
    }
    int numPointCoords = readInt(buffer, dataOffset);
    (void)numCodes;  // 实际码数由 codesPerRound[] 求和获得；保留只为对称

    // 每轮码字数（用于按 level 切换码字模型）—— 与编码端 header 对齐。
    int numRounds = readInt(buffer, dataOffset);
    std::vector<int> codesPerRound(numRounds, 0);
    for (int i = 0; i < numRounds; i++) {
        codesPerRound[i] = readInt(buffer, dataOffset);
    }

    // ---- 码字段：与编码端对称，按 level 奇偶切换模型 ----
    // 导出循环是从 queue index 大到小（B_n, A_n, B_{n-1}, …）；
    // 因此解码也按 i 从 numRounds-1 到 0 的顺序推进，每轮 codesPerRound[i] 个码。
    {
        size_t rcOffset = 0;
        rangecoder rc;
        rc.p_data = buffer + dataOffset;
        rc.p_dataOffset = &rcOffset;
        start_decoding(&rc);

        qsmodel modelDecim, modelClean;
        initqsmodel(&modelDecim, (int)ValenceCode::NUM_CODES, 10, 1 << 9, nullptr, 0);
        initqsmodel(&modelClean, (int)ValenceCode::NUM_CODES, 10, 1 << 9, nullptr, 0);

        for (int i = numRounds - 1; i >= 0; i--) {
            qsmodel* m = (i % 2 == 0) ? &modelDecim : &modelClean;
            int cnt = codesPerRound[i];
            for (int k = 0; k < cnt; k++) {
                int syfreq, ltfreq;
                ltfreq = decode_culshift(&rc, 10);
                int sym = qsgetsym(m, ltfreq);
                qsgetfreq(m, sym, &syfreq, &ltfreq);
                decode_update(&rc, syfreq, ltfreq, 1 << 10);
                qsupdate(m, sym);
                rcDecodedCodes_.push_back((uint8_t)sym);
            }
        }
        done_decoding(&rc);
        deleteqsmodel(&modelDecim);
        deleteqsmodel(&modelClean);
    }
    dataOffset += compCodeSize;

    // ---- edge sym ----
    {
        size_t rcOffset = 0;
        rangecoder rc;
        rc.p_data = buffer + dataOffset;
        rc.p_dataOffset = &rcOffset;
        start_decoding(&rc);

        qsmodel model;
        initqsmodel(&model, 2, 10, 1 << 9, nullptr, 0);
        for (int i = 0; i < numEdgeSymbols; i++) {
            int syfreq, ltfreq;
            ltfreq = decode_culshift(&rc, 10);
            int sym = qsgetsym(&model, ltfreq);
            qsgetfreq(&model, sym, &syfreq, &ltfreq);
            decode_update(&rc, syfreq, ltfreq, 1 << 10);
            qsupdate(&model, sym);
            rcDecodedEdgeSyms_.push_back((uint8_t)sym);
        }
        done_decoding(&rc);
        deleteqsmodel(&model);
    }
    dataOffset += compEdgeSize;

    // ---- vertex sym（Adjustment）----
    if (header_.isEnableAdjustment()) {
        size_t rcOffset = 0;
        rangecoder rc;
        rc.p_data = buffer + dataOffset;
        rc.p_dataOffset = &rcOffset;
        start_decoding(&rc);

        qsmodel model;
        initqsmodel(&model, 2, 10, 1 << 9, nullptr, 0);
        for (int i = 0; i < numVertexSymbols; i++) {
            int syfreq, ltfreq;
            ltfreq = decode_culshift(&rc, 10);
            int sym = qsgetsym(&model, ltfreq);
            qsgetfreq(&model, sym, &syfreq, &ltfreq);
            decode_update(&rc, syfreq, ltfreq, 1 << 10);
            qsupdate(&model, sym);
            rcDecodedVertexSyms_.push_back((uint8_t)sym);
        }
        done_decoding(&rc);
        deleteqsmodel(&model);
        dataOffset += compVertexSize;
    }

    // ---- point 数据（与 Optimized 完全一致的 ZigZag + 位面 / adaptive）----
    {
        size_t rcOffset = 0;
        rangecoder rc;
        rc.p_data = buffer + dataOffset;
        rc.p_dataOffset = &rcOffset;
        start_decoding(&rc);

        bool usePrediction = header_.isEnablePrediction();
        bool useQuant = header_.isEnableQuantization();

        const bool useResidualBitPlane = usePrediction && useQuant;
        rcUsingResidual_ = useResidualBitPlane;
        const bool useAdaptiveModel = !useResidualBitPlane && useQuant && nbQuantBits <= 16;

        const int BP_BITS = (int)nbQuantBits + 1;
        std::vector<qsmodel> bpFirst1;
        std::vector<qsmodel> bpAfter1;
        if (useResidualBitPlane) {
            bpFirst1.resize(BP_BITS);
            bpAfter1.resize(BP_BITS);
            for (int b = 0; b < BP_BITS; b++) {
                initqsmodel(&bpFirst1[b], 2, 10, 1 << 9, nullptr, 0);
                initqsmodel(&bpAfter1[b], 2, 10, 1 << 9, nullptr, 0);
            }
        }
        qsmodel coordModel;
        if (useAdaptiveModel) {
            initqsmodel(&coordModel, 1u << nbQuantBits, 18, 1 << 17, nullptr, 0);
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
            // ZigZag 反解
            int32_t r = (int32_t)((z >> 1) ^ -(int32_t)(z & 1));
            return r;
        };

        int coordsConsumed = 0;
        while (coordsConsumed < numPointCoords) {
            if (useResidualBitPlane) {
                int32_t rx = decResidualBP();
                int32_t ry = decResidualBP();
                int32_t rz = decResidualBP();
                rcDecodedResiduals_.push_back(MCGAL::PointInt(rx, ry, rz));
                coordsConsumed += 3;
            } else if (useQuant) {
                MCGAL::PointInt pi;
                for (int j = 0; j < 3; j++) {
                    int v;
                    if (useAdaptiveModel) {
                        int syfreq, ltfreq;
                        ltfreq = decode_culshift(&rc, 18);
                        v = qsgetsym(&coordModel, ltfreq);
                        qsgetfreq(&coordModel, v, &syfreq, &ltfreq);
                        decode_update(&rc, syfreq, ltfreq, 1 << 18);
                        qsupdate(&coordModel, v);
                    } else {
                        uint32_t ltfreq2 = decode_culshift(&rc, (freq)nbQuantBits);
                        v = (int)ltfreq2;
                        decode_update(&rc, 1, v, (freq)nbQuantBits);
                    }
                    pi[j] = v;
                }
                MCGAL::Point p(
                    bboxMin_[0] + (pi.x() + 0.5f) * f_quantStep_,
                    bboxMin_[1] + (pi.y() + 0.5f) * f_quantStep_,
                    bboxMin_[2] + (pi.z() + 0.5f) * f_quantStep_);
                rcDecodedPoints_.push_back(p);
                coordsConsumed += 3;
            } else {
                float coords[3];
                for (int j = 0; j < 3; j++) {
                    uint16_t low = decode_short(&rc);
                    uint16_t high = decode_short(&rc);
                    uint32_t raw = (uint32_t)low | ((uint32_t)high << 16);
                    std::memcpy(&coords[j], &raw, sizeof(float));
                }
                rcDecodedPoints_.push_back(MCGAL::Point(coords[0], coords[1], coords[2]));
                coordsConsumed += 3;
            }
        }

        done_decoding(&rc);
        if (useAdaptiveModel) deleteqsmodel(&coordModel);
        if (useResidualBitPlane) {
            for (int b = 0; b < BP_BITS; b++) {
                deleteqsmodel(&bpFirst1[b]);
                deleteqsmodel(&bpAfter1[b]);
            }
        }
    }
    dataOffset += compPointSize;
}

void ValenceSymbolReadOperator::collectFromDecoded(MCGAL::Halfedge* seed,
                                                   std::vector<MCGAL::Vertex*>& vertices,
                                                   std::vector<MCGAL::Halfedge*>& halfedge,
                                                   std::vector<MCGAL::Facet*>& facets) {
    // 对称消费 ValenceSymbolCollectOperator::collectOneRound 的新码流协议
    // （B.0 emission-frequency fix，2026-05-06）：
    //   * 不再在每个 halfedge 出队时消费 edge sym
    //   * face 首访时：
    //       1) 消费 1 个 valence code
    //       2) 若 code != NULL_PATCH：以 h 起沿 next() 走一圈，消费 3 个
    //          edge sym；每个 bit=1 的 halfedge setAdded 并压入 halfedge 列表
    //       3) POLY_V_*：另从 residual/point 队列取一个 removedVertexPos
    // BFS 顺序与编码端完全一致（PCLTTM non-Valence 路径已验证同步性）。

    // 清空临时 BFS 队列
    std::queue<int> empty;
    std::swap(gateQueue_, empty);
    gateQueue_.push(seed->poolId());
    int current_version = MCGAL::BFSVersionManager::current_version++;

    size_t codesBeforeRound = rcDecodedCodes_.size();
    size_t edgesBeforeRound = rcDecodedEdgeSyms_.size();
    int faceVisitCount = 0;
    int codeReadCount = 0;
    int edgeReadCount = 0;

    while (!gateQueue_.empty()) {
        int hid = gateQueue_.front();
        gateQueue_.pop();
        MCGAL::Halfedge* h =
            MCGAL::contextPool.getHalfedgeByIndexInSubPool(DEFAULT_MESH_ID, hid);
        if (h == nullptr || h->isVisited(current_version) || h->isRemoved()) {
            continue;
        }
        h->setVisited(current_version);

        MCGAL::Facet* f = h->face();
        bool faceFirstVisit = !f->isVisited(current_version);
        if (faceFirstVisit) {
            f->setVisited(current_version);
            faceVisitCount++;
            codeReadCount++;
            assert(!rcDecodedCodes_.empty());
            ValenceCode code = (ValenceCode)rcDecodedCodes_.front();
            rcDecodedCodes_.pop_front();

            bool isPoly = (code == ValenceCode::POLY_VAL_3 || code == ValenceCode::POLY_VAL_4 ||
                           code == ValenceCode::POLY_VAL_5 || code == ValenceCode::POLY_VAL_6 ||
                           code == ValenceCode::POLY_VAL_OTHER);
            if (isPoly) {
                markFacetSplittableThisRound(f);
                int v = ValenceSymbolCollectOperator::decodePolyValence(code);
                f->setPolyValence(v);
                if (rcUsingResidual_) {
                    assert(!rcDecodedResiduals_.empty());
                    f->setRemovedVertexPosInt(rcDecodedResiduals_.front());
                    rcDecodedResiduals_.pop_front();
                } else {
                    assert(!rcDecodedPoints_.empty());
                    f->setRemovedVertexPos(rcDecodedPoints_.front());
                    rcDecodedPoints_.pop_front();
                }
                facets.push_back(f);
            }
            // code == NULL_PATCH / FAN_CHILD: 无需额外 point / splittable 动作。
            // FAN_CHILD 的 splittable 保持为 false，join_face 会在 merge_face 阶段
            // 根据 gprev->face()->isSplittable() 分支选取正确的保留面（见 MeshUtils.h join_face）。

            // 消费 3 个 edge sym（仅非 NULL_PATCH face）——必须与 encoder 的
            // "以 h 起沿 next() 3 halfedges" 顺序完全对齐。
            if (code != ValenceCode::NULL_PATCH) {
                MCGAL::Halfedge* hit = h;
                for (int k = 0; k < 3 && hit != nullptr; ++k) {
                    assert(!rcDecodedEdgeSyms_.empty());
                    uint8_t esym = rcDecodedEdgeSyms_.front();
                    rcDecodedEdgeSyms_.pop_front();
                    edgeReadCount++;
                    if (esym) {
                        hit->setAdded();
                        halfedge.push_back(hit);
                    }
                    hit = hit->next();
                }
            }
        }

        // Adjustment
        if (header_.isEnableAdjustment() && !h->vertex()->isVisited(current_version)) {
            h->vertex()->setVisited(current_version);
            assert(!rcDecodedVertexSyms_.empty());
            uint8_t vsym = rcDecodedVertexSyms_.front();
            rcDecodedVertexSyms_.pop_front();
            if (vsym) {
                h->vertex()->setAdjusted();
                vertices.push_back(h->vertex());
                assert(!rcDecodedPoints_.empty());
                h->vertex()->setVertexPoint(rcDecodedPoints_.front());
                rcDecodedPoints_.pop_front();
            }
        }

        // BFS neighbor enqueue（与 Optimized / 编码端完全一致）
        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp != nullptr && !hOpp->isVisited(current_version)) {
                gateQueue_.push(hOpp->poolId());
            }
            if (hIt != nullptr && !hIt->isVisited(current_version)) {
                gateQueue_.push(hIt->poolId());
            }
            hIt = hIt->next();
        } while (hIt != h && hIt != nullptr);
    }

}
