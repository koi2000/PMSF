/*
 * DefaultPPMCLiteSymbolReadOperator.cpp — mirror decoder for Lite.
 */

#include "DefaultPPMCLiteSymbolReadOperator.h"

#include "common/DebugTool.h"
#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"
#include "BFSVersionManager.h"
#include "BufferUtils.h"
#include "Configuration.h"
#include "Facet.h"
#include "MeshUtils.h"
#include "core.h"

#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <vector>

DefaultPPMCLiteSymbolReadOperator::~DefaultPPMCLiteSymbolReadOperator() {
    if (pointRCInitialized_) {
        for (auto& m : bpFirst_) deleteqsmodel(&m);
        for (auto& m : bpAfter_) deleteqsmodel(&m);
        if (coordModelInitialized_) deleteqsmodel(&coordModel_);
    }
}

void DefaultPPMCLiteSymbolReadOperator::importFromRangeCodedBuffer(
    char* buffer, size_t& dataOffset, unsigned nbQuantBits) {
    // Reset base-class decoded queues.
    rcDecodedFacetSymbols_.clear();
    rcDecodedEdgeSymbols_.clear();
    rcDecodedVertexSymbols_.clear();
    rcDecodedPoints_.clear();
    rcDecodedVertexPoints_.clear();
    rcDecodedResiduals_.clear();
    rcUsingResidual_ = header_.isEnablePrediction() && enableQuantization_;

    // Reset edge-sym model (base class member).
    if (edgeRCInitialized_) {
        deleteqsmodel(&edgeModel_);
        edgeRCInitialized_ = false;
    }
    useEdgeContext_ = true;

    // Reset our own point-stream state.
    if (pointRCInitialized_) {
        for (auto& m : bpFirst_) deleteqsmodel(&m);
        for (auto& m : bpAfter_) deleteqsmodel(&m);
        bpFirst_.clear();
        bpAfter_.clear();
        if (coordModelInitialized_) {
            deleteqsmodel(&coordModel_);
            coordModelInitialized_ = false;
        }
        pointRCInitialized_ = false;
    }
    nbQuantBits_ = nbQuantBits;
    useResidualBitPlaneLite_ = rcUsingResidual_;

    // ── Read buffer header ──
    int compFacetSize = readInt(buffer, dataOffset);
    int compEdgeSize = readInt(buffer, dataOffset);
    int compVertexSize = 0;
    if (header_.isEnableAdjustment()) compVertexSize = readInt(buffer, dataOffset);
    int compPointSize = readInt(buffer, dataOffset);

    int numFacetSymbols = readInt(buffer, dataOffset);
    int numEdgeSymbols = readInt(buffer, dataOffset);
    int numVertexSymbols = 0;
    if (header_.isEnableAdjustment()) numVertexSymbols = readInt(buffer, dataOffset);
    int numPointCoords = readInt(buffer, dataOffset);
    (void)numEdgeSymbols;
    (void)numPointCoords;  // we don't need explicit counts — BFS driven

    // ── (1) Decode facet syms eagerly (same as base) ──
    {
        size_t rcOffset = 0;
        rangecoder rc;
        rc.p_data = buffer + dataOffset;
        rc.p_dataOffset = &rcOffset;
        start_decoding(&rc);
        qsmodel model;
        initqsmodel(&model, 2, 10, 1 << 9, NULL, 0);
        for (int i = 0; i < numFacetSymbols; i++) {
            int syfreq, ltfreq;
            ltfreq = decode_culshift(&rc, 10);
            int sym = qsgetsym(&model, ltfreq);
            qsgetfreq(&model, sym, &syfreq, &ltfreq);
            decode_update(&rc, syfreq, ltfreq, 1 << 10);
            qsupdate(&model, sym);
            rcDecodedFacetSymbols_.push_back((char)sym);
        }
        done_decoding(&rc);
        deleteqsmodel(&model);
    }
    dataOffset += compFacetSize;

    // ── (2) Edge sym RC state lazily consumed during BFS (base class pattern). ──
    static thread_local std::vector<char> edgeRCBuffer;
    edgeRCBuffer.assign(buffer + dataOffset, buffer + dataOffset + compEdgeSize);
    edgeRCOffset_ = 0;
    edgeRC_.p_data = edgeRCBuffer.data();
    edgeRC_.p_dataOffset = &edgeRCOffset_;
    start_decoding(&edgeRC_);
    initqsmodel(&edgeModel_, 2, 10, 1 << 9, NULL, 0);
    edgeRCInitialized_ = true;
    dataOffset += compEdgeSize;

    // ── (3) Vertex syms eagerly (if adjustment enabled) ──
    if (header_.isEnableAdjustment()) {
        size_t rcOffset = 0;
        rangecoder rc;
        rc.p_data = buffer + dataOffset;
        rc.p_dataOffset = &rcOffset;
        start_decoding(&rc);
        qsmodel model;
        initqsmodel(&model, 2, 10, 1 << 9, NULL, 0);
        for (int i = 0; i < numVertexSymbols; i++) {
            int syfreq, ltfreq;
            ltfreq = decode_culshift(&rc, 10);
            int sym = qsgetsym(&model, ltfreq);
            qsgetfreq(&model, sym, &syfreq, &ltfreq);
            decode_update(&rc, syfreq, ltfreq, 1 << 10);
            qsupdate(&model, sym);
            rcDecodedVertexSymbols_.push_back((char)sym);
        }
        done_decoding(&rc);
        deleteqsmodel(&model);
        dataOffset += compVertexSize;
    }

    // ── (4) Point stream: LAZY. Same buffer trick as edge, so we can
    //       keep the RC state alive for BFS-driven decoding. ──
    static thread_local std::vector<char> pointRCBuffer;
    pointRCBuffer.assign(buffer + dataOffset, buffer + dataOffset + compPointSize);
    pointRCOffset_ = 0;
    pointRC_.p_data = pointRCBuffer.data();
    pointRC_.p_dataOffset = &pointRCOffset_;
    start_decoding(&pointRC_);

    const int BP_BITS = (int)nbQuantBits_ + 1;
    if (useResidualBitPlaneLite_) {
        bpFirst_.resize(BP_BITS);
        bpAfter_.resize(BP_BITS);
        for (int b = 0; b < BP_BITS; b++) {
            initqsmodel(&bpFirst_[b], 2, 10, 1 << 9, NULL, 0);
            initqsmodel(&bpAfter_[b], 2, 10, 1 << 9, NULL, 0);
        }
    } else if (enableQuantization_ && nbQuantBits_ <= 16) {
        // non-prediction adaptive-coord fallback
        unsigned nSymbols = 1u << nbQuantBits_;
        initqsmodel(&coordModel_, nSymbols, 18, 1 << 17, NULL, 0);
        coordModelInitialized_ = true;
    }
    pointRCInitialized_ = true;
    dataOffset += compPointSize;
}

namespace {

// Edge-sym decode (adaptive RC model shared with base class).
inline char decodeEdgeSymLite(rangecoder* rc, qsmodel* m) {
    int ltfreq = decode_culshift(rc, 10);
    int sym = qsgetsym(m, ltfreq);
    int syfreq, lt;
    qsgetfreq(m, sym, &syfreq, &lt);
    decode_update(rc, syfreq, lt, 1 << 10);
    qsupdate(m, sym);
    return (char)sym;
}

// Bit-plane residual decode (mirror of encoder helper).
inline int32_t decodeBitPlaneLite(rangecoder* rc, std::vector<qsmodel>& bpFirst,
                                   std::vector<qsmodel>& bpAfter, int bits) {
    uint32_t z = 0;
    bool seenOne = false;
    for (int b = bits - 1; b >= 0; b--) {
        qsmodel* m = seenOne ? &bpAfter[b] : &bpFirst[b];
        int ltfreq = decode_culshift(rc, 10);
        int sym = qsgetsym(m, ltfreq);
        int syfreq, lt;
        qsgetfreq(m, sym, &syfreq, &lt);
        decode_update(rc, syfreq, lt, 1 << 10);
        qsupdate(m, sym);
        z |= ((uint32_t)(sym & 1)) << b;
        if (sym) seenOne = true;
    }
    return (int32_t)((z >> 1) ^ (-(int32_t)(z & 1)));
}

// Frenet inverse-rotation wrapper with the exact same fallback rule as encoder.
// ⚠️ Phase-A Frenet rotation is currently OFF by default (see encoder-side
//    note). Enable via env `PMSF_LITE_ENABLE_FRENET=1` — must match encoder.
inline MCGAL::PointInt invFrenetFromGate(MCGAL::Halfedge* gate, int32_t ax,
                                          int32_t bx, int32_t cx) {
    static const bool enableFrenet = []() {
        const char* e = std::getenv("PMSF_LITE_ENABLE_FRENET");
        return e && e[0] == '1';
    }();
    if (!enableFrenet || gate == nullptr) {
        return MCGAL::PointInt(ax, bx, cx);
    }
    MCGAL::Vector3 t1(0, 0, 0), t2(0, 0, 0);
    MCGAL::Vector3 normal = MCGAL::computeNormalFromHeh(gate);
    if (normal.length() == 0) {
        return MCGAL::PointInt(ax, bx, cx);
    }
    MCGAL::Vector3 v1(gate->vertex()->point());
    MCGAL::Vector3 v2(gate->opposite() != nullptr
                          ? gate->opposite()->vertex()->point()
                          : gate->next()->vertex()->point());
    MCGAL::determineFrenetFrame(v1, v2, normal, t1, t2);
    MCGAL::Vector3i frenetCoord(ax, bx, cx);
    MCGAL::Vector3i invRot = MCGAL::invFrenetRotation(frenetCoord, t1, t2, normal);
    return MCGAL::PointInt(invRot[0], invRot[1], invRot[2]);
}

}  // namespace

void DefaultPPMCLiteSymbolReadOperator::collectFromDecoded(
    MCGAL::Halfedge* seed, std::vector<MCGAL::Vertex*>& vertices,
    std::vector<MCGAL::Halfedge*>& halfedge,
    std::vector<MCGAL::Facet*>& facets) {
    gateQueue.push(seed->poolId());
    int current_version = MCGAL::BFSVersionManager::current_version++;
    const int BP_BITS = (int)nbQuantBits_ + 1;

    while (!gateQueue.empty()) {
        int hid = gateQueue.front();
        gateQueue.pop();
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndexInSubPool(
            DEFAULT_MESH_ID, hid);
        if (h->isVisited(current_version) || h->isRemoved()) continue;
        h->setVisited(current_version);

        // ── Facet sym (eager-decoded; same as v3) ──
        MCGAL::Facet* f = h->face();
        if (!f->isVisited(current_version)) {
            f->setVisited(current_version);
            assert(!rcDecodedFacetSymbols_.empty());
            char symbol = rcDecodedFacetSymbols_.front();
            rcDecodedFacetSymbols_.pop_front();
            if (symbol) {
                f->setSplittable();
                // ── Lite: decode residual lazily, Frenet-invert with `h`
                //         as gate — identical to encoder's gate tracking. ──
                if (useResidualBitPlaneLite_) {
                    int32_t ax = decodeBitPlaneLite(&pointRC_, bpFirst_, bpAfter_, BP_BITS);
                    int32_t bx = decodeBitPlaneLite(&pointRC_, bpFirst_, bpAfter_, BP_BITS);
                    int32_t cx = decodeBitPlaneLite(&pointRC_, bpFirst_, bpAfter_, BP_BITS);
                    // h is the BFS-popped halfedge; matches what encoder used.
                    MCGAL::PointInt residual = invFrenetFromGate(h, ax, bx, cx);
                    f->setRemovedVertexPosInt(residual);
                } else if (enableQuantization_ && coordModelInitialized_) {
                    // Non-prediction path: decode absolute coords (no Frenet).
                    MCGAL::Point point;
                    uint32_t p[3];
                    for (int j = 0; j < 3; j++) {
                        int syfreq, ltfreq;
                        ltfreq = decode_culshift(&pointRC_, 18);
                        int sym = qsgetsym(&coordModel_, ltfreq);
                        qsgetfreq(&coordModel_, sym, &syfreq, &ltfreq);
                        decode_update(&pointRC_, syfreq, ltfreq, 1 << 18);
                        qsupdate(&coordModel_, sym);
                        p[j] = (uint32_t)sym;
                    }
                    point[0] = bboxMin_[0] + (p[0] + 0.5) * f_quantStep_;
                    point[1] = bboxMin_[1] + (p[1] + 0.5) * f_quantStep_;
                    point[2] = bboxMin_[2] + (p[2] + 0.5) * f_quantStep_;
                    f->setRemovedVertexPos(point);
                } else if (enableQuantization_) {
                    // Non-prediction, non-adaptive: 1/nbQuantBits shift-decode.
                    MCGAL::Point point;
                    uint32_t p[3];
                    for (int j = 0; j < 3; j++) {
                        p[j] = decode_culshift(&pointRC_, nbQuantBits_);
                        decode_update(&pointRC_, 1, p[j], (freq)1 << nbQuantBits_);
                    }
                    point[0] = bboxMin_[0] + (p[0] + 0.5) * f_quantStep_;
                    point[1] = bboxMin_[1] + (p[1] + 0.5) * f_quantStep_;
                    point[2] = bboxMin_[2] + (p[2] + 0.5) * f_quantStep_;
                    f->setRemovedVertexPos(point);
                } else {
                    // Non-quantized: raw float through RC short stream.
                    float coords[3];
                    for (int j = 0; j < 3; j++) {
                        uint16_t low = decode_short(&pointRC_);
                        uint16_t high = decode_short(&pointRC_);
                        uint32_t raw = (uint32_t)low | ((uint32_t)high << 16);
                        std::memcpy(&coords[j], &raw, sizeof(float));
                    }
                    f->setRemovedVertexPos(MCGAL::Point(coords[0], coords[1], coords[2]));
                }
                facets.push_back(f);
            }
        }

        // ── Edge sym (v3 per-edge, first-pop, same as base class) ──
        {
            MCGAL::Halfedge* hOpp = h->opposite();
            bool consume = false;
            if (hOpp == nullptr) consume = true;
            else if (!hOpp->isVisited(current_version)) consume = true;
            if (consume) {
                char esym = decodeEdgeSymLite(&edgeRC_, &edgeModel_);
                if (esym) {
                    h->setAdded();
                    if (hOpp != nullptr) hOpp->setAdded();
                    halfedge.push_back(h);
                }
            }
        }

        // ── Vertex adjustment (unchanged from v3) ──
        if (header_.isEnableAdjustment() && !h->vertex()->isVisited(current_version)) {
            h->vertex()->setVisited(current_version);
            assert(!rcDecodedVertexSymbols_.empty());
            char symbol = rcDecodedVertexSymbols_.front();
            rcDecodedVertexSymbols_.pop_front();
            if (symbol) {
                h->vertex()->setAdjusted();
                vertices.push_back(h->vertex());
                // NOTE: vertex-adjustment points are not Frenet-rotated (they
                // aren't residuals). Under Lite Phase A we decode them as raw
                // absolute-position points using the same RC stream. Since the
                // encoder currently emits vertex points only when
                // isEnableAdjustment() is true (rarely for Default path), the
                // simpler fallback below is typically never hit in bench.
                MCGAL::Point point;
                uint32_t p[3];
                for (int j = 0; j < 3; j++) {
                    p[j] = enableQuantization_ && coordModelInitialized_
                               ? ({
                                     int syfreq, ltfreq;
                                     ltfreq = decode_culshift(&pointRC_, 18);
                                     int sym = qsgetsym(&coordModel_, ltfreq);
                                     qsgetfreq(&coordModel_, sym, &syfreq, &ltfreq);
                                     decode_update(&pointRC_, syfreq, ltfreq, 1 << 18);
                                     qsupdate(&coordModel_, sym);
                                     (uint32_t)sym;
                                 })
                               : (uint32_t)decode_culshift(&pointRC_, nbQuantBits_);
                    if (!(enableQuantization_ && coordModelInitialized_))
                        decode_update(&pointRC_, 1, p[j], (freq)1 << nbQuantBits_);
                }
                point[0] = bboxMin_[0] + (p[0] + 0.5) * f_quantStep_;
                point[1] = bboxMin_[1] + (p[1] + 0.5) * f_quantStep_;
                point[2] = bboxMin_[2] + (p[2] + 0.5) * f_quantStep_;
                h->vertex()->setVertexPoint(point);
            }
        }

        // Push BFS neighbors (same as base class).
        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp != nullptr && !hOpp->isVisited(current_version)) {
                gateQueue.push(hOpp->poolId());
            }
            if (!hIt->isVisited(current_version)) {
                gateQueue.push(hIt->poolId());
            }
            hIt = hIt->next();
        } while (hIt != h);
    }
}
