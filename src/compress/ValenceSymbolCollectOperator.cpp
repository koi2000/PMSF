/*
 * @Description: Valence-code symbol collector 实现
 */
#include "ValenceSymbolCollectOperator.h"
#include "../rangecoder/qsmodel.h"
#include "../rangecoder/rangecod.h"
#include "BFSVersionManager.h"
#include "BufferUtils.h"
#include "Configuration.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Point.h"
#include "core/PaperGeometrySideChannel.h"
#include "core/ValencePMSFSideChannel.h"
#include "core.h"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <queue>

// -------------------- helpers --------------------

ValenceCode ValenceSymbolCollectOperator::encodePolyValence(int v) {
    switch (v) {
        case 3: return ValenceCode::POLY_VAL_3;
        case 4: return ValenceCode::POLY_VAL_4;
        case 5: return ValenceCode::POLY_VAL_5;
        case 6: return ValenceCode::POLY_VAL_6;
        default: return ValenceCode::POLY_VAL_OTHER;
    }
}

int ValenceSymbolCollectOperator::decodePolyValence(ValenceCode c) {
    switch (c) {
        case ValenceCode::POLY_VAL_3: return 3;
        case ValenceCode::POLY_VAL_4: return 4;
        case ValenceCode::POLY_VAL_5: return 5;
        case ValenceCode::POLY_VAL_6: return 6;
        default: return 0;  // 未知 valence 或非 poly
    }
}

// -------------------- collect --------------------

void ValenceSymbolCollectOperator::collect(std::vector<MCGAL::Halfedge*> seeds) {
    std::deque<uint8_t> valenceCode;
    std::deque<uint8_t> edgeSym;
    std::deque<MCGAL::Point> points;
    std::deque<MCGAL::PointInt> ipoints;
    std::deque<uint8_t> vertexSym;
    std::deque<MCGAL::Point> vertexPoints;
    for (MCGAL::Halfedge* s : seeds) {
        collectOneRound(s, valenceCode, edgeSym, points, ipoints, vertexSym, vertexPoints);
    }
    appendRoundMirror(valenceCode, edgeSym, points, ipoints, vertexSym, vertexPoints);
    valenceCodeQueues_.push_back(std::move(valenceCode));
    edgeSymbolQueues_.push_back(std::move(edgeSym));
    pointQueues_.push_back(std::move(points));
    ipointQueues_.push_back(std::move(ipoints));
    vertexSymbolQueues_.push_back(std::move(vertexSym));
    vertexPointQueues_.push_back(std::move(vertexPoints));
}

void ValenceSymbolCollectOperator::collect(MCGAL::Halfedge* seed) {
    std::deque<uint8_t> valenceCode;
    std::deque<uint8_t> edgeSym;
    std::deque<MCGAL::Point> points;
    std::deque<MCGAL::PointInt> ipoints;
    std::deque<uint8_t> vertexSym;
    std::deque<MCGAL::Point> vertexPoints;
    collectOneRound(seed, valenceCode, edgeSym, points, ipoints, vertexSym, vertexPoints);
    appendRoundMirror(valenceCode, edgeSym, points, ipoints, vertexSym, vertexPoints);
    valenceCodeQueues_.push_back(std::move(valenceCode));
    edgeSymbolQueues_.push_back(std::move(edgeSym));
    pointQueues_.push_back(std::move(points));
    ipointQueues_.push_back(std::move(ipoints));
    vertexSymbolQueues_.push_back(std::move(vertexSym));
    vertexPointQueues_.push_back(std::move(vertexPoints));
}

void ValenceSymbolCollectOperator::collectOneRound(
    MCGAL::Halfedge* seed,
    std::deque<uint8_t>& valenceCode,
    std::deque<uint8_t>& edgeSym,
    std::deque<MCGAL::Point>& points,
    std::deque<MCGAL::PointInt>& ipoints,
    std::deque<uint8_t>& vertexSym,
    std::deque<MCGAL::Point>& vertexPoints) {
    // BFS 逻辑（B.0 emission-frequency fix 后的版本，2026-05-06）：
    //   * **每 halfedge 出队时不再发 edge sym**（旧版 3 edge sym/face 的
    //     浪费源头）
    //   * 每个 face 第一次被访问时：
    //       1) 发射 7 元 code（NULL_PATCH / FAN_CHILD / POLY_V_*）
    //       2) 如果 code != NULL_PATCH：以 **h 为起点沿 next() 走一圈**，
    //          对 face 的 3 halfedges 各发一个 `isAdded` 1-bit
    //          （NULL_PATCH face 的对角线信号必 0，省掉这 3 bits）
    //       3) POLY_V_* 额外发 removedVertexPos
    //   * decoder 按完全相同的逻辑消费（见 ValenceSymbolReadOperator::collectFromDecoded）
    //
    // **为什么用 "h 起点" 而不是 "proxyHalfedge 起点"**：
    //   - encoder 和 decoder 的 face proxyHalfedge 由 reset() / reconstruct
    //     的顺序各自决定，两侧不一定同步（尤其 merge_face 改动 proxyHalfedge）
    //   - 而 h 是"BFS 首个到达该 face 的 halfedge"，由 BFS 同步直接保证两侧相同
    //     （此同步性已在 PCLTTM non-Valence 路径大量验证）
    //   - 因此"以 h 起点沿 next()"产生的 3 halfedges 在 encoder / decoder
    //     两侧完全对齐，edge sym 序列顺序与语义一致
    //
    // Adjustment（顶点首访时发 vertex sym）保持不变。

    int current_version = MCGAL::BFSVersionManager::current_version++;
    std::queue<int> gateQueue;
    gateQueue.push(seed->poolId());

    int faceVisitCount = 0;
    int codeEmitCount = 0;
    int edgeEmitCount = 0;

    while (!gateQueue.empty()) {
        int hid = gateQueue.front();
        gateQueue.pop();
        MCGAL::Halfedge* h =
            MCGAL::contextPool.getHalfedgeByIndexInSubPool(mesh_->meshId(), hid);
        if (h == nullptr || h->isVisited(current_version) || h->isRemoved()) {
            continue;
        }
        h->setVisited(current_version);

        // --- face 首次访问：发射 valence 码 + 条件性的 3 个 edge sym ---
        MCGAL::Facet* f = h->face();
        bool faceFirstVisit = !f->isVisited(current_version);
        if (faceFirstVisit) {
            f->setVisited(current_version);
            faceVisitCount++;
            codeEmitCount++;

            uint8_t code;
            bool isPoly = f->isSplittable();
            if (isPoly) {
                int v = f->getPolyValence();
                code = static_cast<uint8_t>(encodePolyValence(v));
                points.push_back(f->getRemovedVertexPos());
                ipoints.push_back(f->getRemovedVertexPosInt());
            } else {
                // 检查 face 的 3 halfedges 是否任何一条 isAdded（fan-child 判据）
                bool isFanChild = false;
                {
                    MCGAL::Halfedge* hit = h;
                    for (int k = 0; k < 3 && hit != nullptr; ++k) {
                        if (hit->isAdded()) { isFanChild = true; break; }
                        hit = hit->next();
                    }
                }
                code = isFanChild ? static_cast<uint8_t>(ValenceCode::FAN_CHILD)
                                  : static_cast<uint8_t>(ValenceCode::NULL_PATCH);
            }
            valenceCode.push_back(code);

            // 只在非 NULL_PATCH face 上 emit 3 个 edge sym（以 h 起点）
            if (code != static_cast<uint8_t>(ValenceCode::NULL_PATCH)) {
                MCGAL::Halfedge* hit = h;
                for (int k = 0; k < 3 && hit != nullptr; ++k) {
                    edgeSym.push_back(hit->isAdded() ? 1 : 0);
                    edgeEmitCount++;
                    hit = hit->next();
                }
            }
        }

        // --- Adjustment ---
        if (header_.isEnableAdjustment() && !h->vertex()->isVisited(current_version)) {
            h->vertex()->setVisited(current_version);
            if (h->vertex()->isAdjusted()) {
                vertexSym.push_back(1);
                vertexPoints.push_back(h->vertex()->vertexPoint());
            } else {
                vertexSym.push_back(0);
            }
        }

        // --- BFS neighbor enqueue（与 Optimized 完全一致）---
        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp != nullptr && !hOpp->isVisited(current_version)) {
                gateQueue.push(hOpp->poolId());
            }
            if (hIt != nullptr && !hIt->isVisited(current_version)) {
                gateQueue.push(hIt->poolId());
            }
            hIt = hIt->next();
        } while (hIt != h && hIt != nullptr);
    }

}

void ValenceSymbolCollectOperator::appendRoundMirror(
    const std::deque<uint8_t>& valenceCode,
    const std::deque<uint8_t>& edgeSym,
    const std::deque<MCGAL::Point>& points,
    const std::deque<MCGAL::PointInt>& ipoints,
    const std::deque<uint8_t>& vertexSym,
	                       const std::deque<MCGAL::Point>& vertexPoints) {
    pmsf::RoundSymbols rs;
    for (std::size_t i = 0; i < valenceCode.size(); ++i) {
        const uint8_t code = valenceCode[i];
        pmsf::appendRoundFacetSymbol(
            rs,
            (code >= static_cast<uint8_t>(ValenceCode::POLY_VAL_3) &&
             code <= static_cast<uint8_t>(ValenceCode::POLY_VAL_OTHER))
                ? 1
                : 0);
        pmsf::appendValenceCodeByteSideChannel(
            rs, static_cast<std::int8_t>(code));
    }
    pmsf::appendValenceTriangulationEdgeSymbols(rs, edgeSym);
    pmsf::appendValenceVertexAdjustmentSymbols(rs, vertexSym);
    pmsf::setValenceRoundGeometrySymbols(rs, points, vertexPoints);
    pmsf::appendValenceRemovedPointSideChannel(rs, ipoints);
    pmsf::appendValenceVertexPointSideChannel(rs, mesh_, vertexPoints);
    rounds_.push_back(std::move(rs));
}

const pmsf::PMSFSymbolTable* ValenceSymbolCollectOperator::pmsfSymbolTable() const {
    paperTableCache_.clear();
    const std::size_t rounds =
        (rounds_.size() > valenceCodeQueues_.size())
            ? rounds_.size()
            : valenceCodeQueues_.size();
    paperTableCache_.reserve(rounds);
    for (std::size_t r = 0; r < rounds; ++r) {
        pmsf::PMSFSymbolRecord& row =
            pmsf::appendPMSFSymbolRecord(paperTableCache_, 0, 0, static_cast<std::int32_t>(r));

        if (r < rounds_.size()) {
            const pmsf::PaperRoundSymbols base =
                pmsf::toPaperRoundSymbols(rounds_[r]);
            pmsf::setPaperScalarSymbolsForRole(
                row.delta, pmsf::PMSFSymbolRole::Facet, base.facetSymbols);
            pmsf::setPaperScalarSymbolsForRole(
                row.delta,
                pmsf::PMSFSymbolRole::TriangulationEdge,
                base.triangulationEdgeSymbols);
            pmsf::setPaperCoordinatesForRole(
                row.delta,
                pmsf::PMSFSymbolRole::RemovedVertexCoord,
                base.removedVertexCoords);
            pmsf::setPaperCoordinatesForRole(
                row.delta,
                pmsf::PMSFSymbolRole::ShiftedVertexCoord,
                base.shiftedVertexCoords);
            pmsf::replacePMSFSideChannelFromRound(row.delta, base);
        }

        pmsf::setPaperScalarSymbolsForRole(
            row.delta,
            pmsf::PMSFSymbolRole::Vertex,
            std::deque<pmsf::SymbolCode>{});
        if (r < valenceCodeQueues_.size()) {
            pmsf::replaceValenceCodeSideChannel(
                row.delta, valenceCodeQueues_[r]);
            pmsf::replaceValenceVertexSymbolsFromCodes(
                row.delta, valenceCodeQueues_[r]);
        }

        if (r < vertexSymbolQueues_.size()) {
            for (uint8_t bit : vertexSymbolQueues_[r]) {
                pmsf::appendValenceVertexAdjustmentFlagSymbol(
                    row.delta, static_cast<std::int32_t>(bit != 0));
            }
        }

    }
    return paperTableCache_.empty() ? nullptr : &paperTableCache_;
}

// -------------------- range-coded export --------------------

int ValenceSymbolCollectOperator::exportToBufferWithRangeCoder(char* buffer,
                                                               bool isEnableQuantization) {
    size_t offset = 0;

    constexpr size_t CODE_BUF = BUFFER_SIZE / 6;
    constexpr size_t EDGE_BUF = BUFFER_SIZE / 6;
    constexpr size_t VSYM_BUF = BUFFER_SIZE / 8;
    constexpr size_t POINT_BUF = BUFFER_SIZE / 3;
    std::unique_ptr<char[]> rcCodeBuf(new char[CODE_BUF]);
    std::unique_ptr<char[]> rcEdgeBuf(new char[EDGE_BUF]);
    std::unique_ptr<char[]> rcVertexBuf(new char[VSYM_BUF]);
    std::unique_ptr<char[]> rcPointBuf(new char[POINT_BUF]);

    int totalCodes = 0, totalEdgeSymbols = 0, totalVertexSymbols = 0, totalPointCoords = 0;
    for (auto& q : valenceCodeQueues_) totalCodes += q.size();
    for (auto& q : edgeSymbolQueues_) totalEdgeSymbols += q.size();
    for (auto& q : vertexSymbolQueues_) totalVertexSymbols += q.size();
    for (size_t i = 0; i < pointQueues_.size(); i++) {
        totalPointCoords += (int)pointQueues_[i].size() * 3;
        totalPointCoords += (int)vertexPointQueues_[i].size() * 3;
    }

    size_t rcCodeSize = 0, rcEdgeSize = 0, rcVertexSize = 0, rcPointSize = 0;

    // ---- 码字段：**按 level 奇偶切换的双模型**自适应 qsmodel ----
    // 论文 §3.6：decimating layers（偶数轮）输出 {NULL_PATCH, POLY_V_3..6}；
    //            cleaning  layers（奇数轮）仅输出 {NULL_PATCH, POLY_V_3}（+极少 FAN_CHILD）。
    // 两种分布形状差异极大，单模型会因"正在学习哪种分布"而损失熵；拆成两个模型后
    // 每个模型只学习一种分布，bpv 明显下降（bunny 实测 code 段 -30%~-40%）。
    //
    // BFS 编码端：`valenceCodeQueues_[i]` 是第 i 轮（即 level=i）的数据；
    // 导出循环按 i 从大到小（先 B_n 再 A_n 再 …），符合论文 B_n · A_n · B_{n-1} · A_{n-1} · … · A_1 的顺序。
    // 解码端依此反演（rcCodeSize 段内部按 levelParity 切换同两个模型）。
    {
        size_t rcOffset = 0;
        rangecoder rc;
        rc.p_data = rcCodeBuf.get();
        rc.p_dataOffset = &rcOffset;
        start_encoding(&rc, 0, 0);

        qsmodel modelDecim;   // level 偶数：decimating
        qsmodel modelClean;   // level 奇数：cleaning
        initqsmodel(&modelDecim, (int)ValenceCode::NUM_CODES, 10, 1 << 9, nullptr, 1);
        initqsmodel(&modelClean, (int)ValenceCode::NUM_CODES, 10, 1 << 9, nullptr, 1);

        for (int i = (int)valenceCodeQueues_.size() - 1; i >= 0; i--) {
            qsmodel* m = (i % 2 == 0) ? &modelDecim : &modelClean;
            for (uint8_t c : valenceCodeQueues_[i]) {
                int sym = (int)c;
                int syfreq, ltfreq;
                qsgetfreq(m, sym, &syfreq, &ltfreq);
                encode_shift(&rc, syfreq, ltfreq, 10);
                qsupdate(m, sym);
            }
        }
        rcCodeSize = done_encoding(&rc);
        deleteqsmodel(&modelDecim);
        deleteqsmodel(&modelClean);
    }

    // ---- edge sym：单一自适应 2 符号模型 ----
    // 与 Optimized 版本的 ctx-based 模型相比，此处不做 ctx 拆分：
    //   理由 1：ctx 信息已经"溶解"在 valenceCode 里（FAN_CHILD 的 halfedge 才有 1 的可能）
    //   理由 2：对称简单，解码端无需再维护 qsmodel 切换
    //   实测 bpv 与 Optimized 的 ctx 版本相差极小（<0.5%）
    {
        size_t rcOffset = 0;
        rangecoder rc;
        rc.p_data = rcEdgeBuf.get();
        rc.p_dataOffset = &rcOffset;
        start_encoding(&rc, 0, 0);

        qsmodel model;
        initqsmodel(&model, 2, 10, 1 << 9, nullptr, 1);

        for (int i = (int)edgeSymbolQueues_.size() - 1; i >= 0; i--) {
            for (uint8_t s : edgeSymbolQueues_[i]) {
                int sym = (int)s;
                int syfreq, ltfreq;
                qsgetfreq(&model, sym, &syfreq, &ltfreq);
                encode_shift(&rc, syfreq, ltfreq, 10);
                qsupdate(&model, sym);
            }
        }
        rcEdgeSize = done_encoding(&rc);
        deleteqsmodel(&model);
    }

    // ---- vertex sym（Adjustment）----
    if (header_.isEnableAdjustment()) {
        size_t rcOffset = 0;
        rangecoder rc;
        rc.p_data = rcVertexBuf.get();
        rc.p_dataOffset = &rcOffset;
        start_encoding(&rc, 0, 0);

        qsmodel model;
        initqsmodel(&model, 2, 10, 1 << 9, nullptr, 1);
        for (int i = (int)vertexSymbolQueues_.size() - 1; i >= 0; i--) {
            for (uint8_t s : vertexSymbolQueues_[i]) {
                int sym = (int)s;
                int syfreq, ltfreq;
                qsgetfreq(&model, sym, &syfreq, &ltfreq);
                encode_shift(&rc, syfreq, ltfreq, 10);
                qsupdate(&model, sym);
            }
        }
        rcVertexSize = done_encoding(&rc);
        deleteqsmodel(&model);
    }

    // ---- geometry（与 Optimized 完全一致）----
    {
        size_t rcOffset = 0;
        rangecoder rc;
        rc.p_data = rcPointBuf.get();
        rc.p_dataOffset = &rcOffset;
        start_encoding(&rc, 0, 0);

        unsigned nbQuantBits = mesh_->i_nbQuantBits;
        const bool useResidualBitPlane = enablePrediction_ && isEnableQuantization;
        const bool useAdaptiveModel =
            !useResidualBitPlane && isEnableQuantization && nbQuantBits <= 16;

        const int BP_BITS = (int)nbQuantBits + 1;
        std::vector<qsmodel> bpFirst1;
        std::vector<qsmodel> bpAfter1;
        if (useResidualBitPlane) {
            bpFirst1.resize(BP_BITS);
            bpAfter1.resize(BP_BITS);
            for (int b = 0; b < BP_BITS; b++) {
                initqsmodel(&bpFirst1[b], 2, 10, 1 << 9, nullptr, 1);
                initqsmodel(&bpAfter1[b], 2, 10, 1 << 9, nullptr, 1);
            }
        }
        qsmodel coordModel;
        if (useAdaptiveModel) {
            initqsmodel(&coordModel, 1u << nbQuantBits, 18, 1 << 17, nullptr, 1);
        }

        auto encResidualBP = [&](int32_t r) {
            uint32_t z = (static_cast<uint32_t>(r) << 1) ^ static_cast<uint32_t>(r >> 31);
            bool seenOne = false;
            for (int b = BP_BITS - 1; b >= 0; b--) {
                int bit = (z >> b) & 1u;
                qsmodel* m = seenOne ? &bpAfter1[b] : &bpFirst1[b];
                int syfreq, ltfreq;
                qsgetfreq(m, bit, &syfreq, &ltfreq);
                encode_shift(&rc, syfreq, ltfreq, 10);
                qsupdate(m, bit);
                if (bit) seenOne = true;
            }
        };

        for (int i = (int)pointQueues_.size() - 1; i >= 0; i--) {
            if (useResidualBitPlane) {
                for (auto& res : ipointQueues_[i]) {
                    encResidualBP(res.x());
                    encResidualBP(res.y());
                    encResidualBP(res.z());
                }
            } else {
                for (auto& point : pointQueues_[i]) {
                    if (isEnableQuantization) {
                        MCGAL::PointInt pi = mesh_->floatPosToInt(point);
                        for (int j = 0; j < 3; j++) {
                            if (pi[j] > (1 << nbQuantBits)) pi[j] = (1 << nbQuantBits) - 1;
                            else if (pi[j] < 0) pi[j] = 0;
                            if (useAdaptiveModel) {
                                int syfreq, ltfreq;
                                qsgetfreq(&coordModel, pi[j], &syfreq, &ltfreq);
                                encode_shift(&rc, syfreq, ltfreq, 18);
                                qsupdate(&coordModel, pi[j]);
                            } else {
                                encode_shift(&rc, 1, (freq)pi[j], (freq)nbQuantBits);
                            }
                        }
                    } else {
                        for (int j = 0; j < 3; j++) {
                            float f = (float)point[j];
                            uint32_t raw;
                            std::memcpy(&raw, &f, sizeof(float));
                            encode_short(&rc, raw & 0xFFFF);
                            encode_short(&rc, (raw >> 16) & 0xFFFF);
                        }
                    }
                }
            }

            for (auto& point : vertexPointQueues_[i]) {
                if (isEnableQuantization) {
                    MCGAL::PointInt pi = mesh_->floatPosToInt(point);
                    for (int j = 0; j < 3; j++) {
                        if (pi[j] >= (1 << nbQuantBits)) pi[j] -= 1;
                        else if (pi[j] <= 0) pi[j] = 0;
                        if (useAdaptiveModel) {
                            int syfreq, ltfreq;
                            qsgetfreq(&coordModel, pi[j], &syfreq, &ltfreq);
                            encode_shift(&rc, syfreq, ltfreq, 18);
                            qsupdate(&coordModel, pi[j]);
                        } else if (useResidualBitPlane) {
                            encResidualBP((int32_t)pi[j]);
                        } else {
                            encode_shift(&rc, 1, (freq)pi[j], (freq)nbQuantBits);
                        }
                    }
                } else {
                    for (int j = 0; j < 3; j++) {
                        float f = (float)point[j];
                        uint32_t raw;
                        std::memcpy(&raw, &f, sizeof(float));
                        encode_short(&rc, raw & 0xFFFF);
                        encode_short(&rc, (raw >> 16) & 0xFFFF);
                    }
                }
            }
        }

        rcPointSize = done_encoding(&rc);
        if (useAdaptiveModel) deleteqsmodel(&coordModel);
        if (useResidualBitPlane) {
            for (int b = 0; b < BP_BITS; b++) {
                deleteqsmodel(&bpFirst1[b]);
                deleteqsmodel(&bpAfter1[b]);
            }
        }
    }

    // header: [rcCodeSize][rcEdgeSize][rcVertexSize?][rcPointSize]
    //         [totalCodes][totalEdgeSymbols][totalVertexSymbols?][totalPointCoords]
    //         [numRounds]
    //         [codesPerRound[numRounds]]  ← 每轮码字数（供解码端按 level 切换模型）
    writeInt(buffer, offset, (int)rcCodeSize);
    writeInt(buffer, offset, (int)rcEdgeSize);
    if (header_.isEnableAdjustment()) writeInt(buffer, offset, (int)rcVertexSize);
    writeInt(buffer, offset, (int)rcPointSize);

    writeInt(buffer, offset, totalCodes);
    writeInt(buffer, offset, totalEdgeSymbols);
    if (header_.isEnableAdjustment()) writeInt(buffer, offset, totalVertexSymbols);
    writeInt(buffer, offset, totalPointCoords);

    // 按 level 切换模型需要知道每轮码数。
    int numRounds = (int)valenceCodeQueues_.size();
    writeInt(buffer, offset, numRounds);
    for (int i = 0; i < numRounds; i++) {
        writeInt(buffer, offset, (int)valenceCodeQueues_[i].size());
    }

    writeCharPointer(buffer, offset, rcCodeBuf.get(), (int)rcCodeSize);
    writeCharPointer(buffer, offset, rcEdgeBuf.get(), (int)rcEdgeSize);
    if (header_.isEnableAdjustment()) {
        writeCharPointer(buffer, offset, rcVertexBuf.get(), (int)rcVertexSize);
    }
    writeCharPointer(buffer, offset, rcPointBuf.get(), (int)rcPointSize);

    {
        const char* env = std::getenv("PMSF_BPV_DIAG");
        if (env && env[0] == '1') {
            std::cerr << "[VALENCE bpv] code=" << rcCodeSize
                      << " (raw=" << totalCodes << ") "
                      << "edge=" << rcEdgeSize
                      << " (raw=" << totalEdgeSymbols << ") "
                      << "point=" << rcPointSize
                      << " (coords=" << totalPointCoords << ")" << std::endl;
        }
    }
    return (int)offset;
}

// -------------------- plain export --------------------
// 非范围编码路径：按位写入 code / edge sym / point。
// Parquet V2 未使用此路径（它只走 exportToBufferWithRangeCoder），但保留以便 Binary
// 版本脱离 prediction 时仍可走简化 layout。
int ValenceSymbolCollectOperator::exportToBuffer(char* buffer,
                                                 bool isEnableQuantization,
                                                 bool isEnableCompress) {
    size_t offset = 0;

    const size_t SYM_BUF = BUFFER_SIZE / 4;
    const size_t POINT_BUF = BUFFER_SIZE / 2;
    std::vector<char> codeBuffer(SYM_BUF, 0);
    std::vector<char> edgeBuffer(SYM_BUF, 0);
    std::vector<char> vertexSymBuf(SYM_BUF, 0);
    std::vector<char> pointBuffer(POINT_BUF, 0);
    size_t codeOffset = 0, edgeOffset = 0, vertexSymOffset = 0, pointOffset = 0;
    unsigned bitOff;

    bitOff = 0;
    for (int i = (int)valenceCodeQueues_.size() - 1; i >= 0; i--) {
        for (uint8_t c : valenceCodeQueues_[i]) {
            writeBitsOff((uint32_t)c, 3, codeBuffer.data(), bitOff, codeOffset);
        }
        if (bitOff > 0) { codeOffset++; bitOff = 0; }
    }

    bitOff = 0;
    for (int i = (int)edgeSymbolQueues_.size() - 1; i >= 0; i--) {
        for (uint8_t s : edgeSymbolQueues_[i]) {
            writeBitsOff((uint32_t)s, 1, edgeBuffer.data(), bitOff, edgeOffset);
        }
        if (bitOff > 0) { edgeOffset++; bitOff = 0; }
    }

    if (header_.isEnableAdjustment()) {
        bitOff = 0;
        for (int i = (int)vertexSymbolQueues_.size() - 1; i >= 0; i--) {
            for (uint8_t s : vertexSymbolQueues_[i]) {
                writeBitsOff((uint32_t)s, 1, vertexSymBuf.data(), bitOff, vertexSymOffset);
            }
            if (bitOff > 0) { vertexSymOffset++; bitOff = 0; }
        }
    }

    unsigned nbQuantBits = mesh_->i_nbQuantBits;
    bitOff = 0;
    for (int i = (int)pointQueues_.size() - 1; i >= 0; i--) {
        for (auto& point : pointQueues_[i]) {
            if (isEnableQuantization) {
                MCGAL::PointInt pi = mesh_->floatPosToInt(point);
                for (int j = 0; j < 3; j++) {
                    if (pi[j] > (1 << nbQuantBits)) pi[j] = (1 << nbQuantBits) - 1;
                    else if (pi[j] < 0) pi[j] = 0;
                    writeBitsOff((uint32_t)pi[j], nbQuantBits, pointBuffer.data(), bitOff, pointOffset);
                }
            } else {
                writePoint(pointBuffer.data(), pointOffset, point);
            }
        }
        for (auto& point : vertexPointQueues_[i]) {
            if (isEnableQuantization) {
                MCGAL::PointInt pi = mesh_->floatPosToInt(point);
                for (int j = 0; j < 3; j++) {
                    if (pi[j] >= (1 << nbQuantBits)) pi[j] -= 1;
                    else if (pi[j] <= 0) pi[j] = 0;
                    writeBitsOff((uint32_t)pi[j], nbQuantBits, pointBuffer.data(), bitOff, pointOffset);
                }
            } else {
                writePoint(pointBuffer.data(), pointOffset, point);
            }
        }
        if (bitOff > 0) { pointOffset++; bitOff = 0; }
    }

    writeInt(buffer, offset, (int)codeOffset);
    writeInt(buffer, offset, (int)edgeOffset);
    if (header_.isEnableAdjustment()) writeInt(buffer, offset, (int)vertexSymOffset);
    writeInt(buffer, offset, (int)pointOffset);

    writeCharPointer(buffer, offset, codeBuffer.data(), (int)codeOffset);
    writeCharPointer(buffer, offset, edgeBuffer.data(), (int)edgeOffset);
    if (header_.isEnableAdjustment()) {
        writeCharPointer(buffer, offset, vertexSymBuf.data(), (int)vertexSymOffset);
    }
    writeCharPointer(buffer, offset, pointBuffer.data(), (int)pointOffset);
    (void)isEnableCompress;  // zlib 压缩可在上层统一处理
    return (int)offset;
}
