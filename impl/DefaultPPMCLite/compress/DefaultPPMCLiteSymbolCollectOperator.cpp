/*
 * DefaultPPMCLiteSymbolCollectOperator.cpp
 *
 * Phase-A variant: same v3 per-edge BFS as Optimized, but rotates the
 * quantized residual through a Frenet local frame (built from the BFS
 * gate halfedge's face normal) before bit-plane RC encoding.
 */

#include "DefaultPPMCLiteSymbolCollectOperator.h"
#include "core/PMSFBoundarySideChannel.h"
#include "core/DefaultCodecPMSFSideChannel.h"
#include "DefaultPPMCLiteSideChannel.h"
#include "core/PaperGeometrySideChannel.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include "common/DebugTool.h"
#include "compress/RangeCoderStreamEncoder.h"
#include "BFSVersionManager.h"
#include "BufferUtils.h"
#include "Configuration.h"
#include "Facet.h"
#include "Halfedge.h"
#include "MeshUtils.h"
#include "Point.h"
#include "core.h"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <queue>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

bool defaultPPMCLiteEnvOn(const char* name) {
    return pmsf::pmsfStorageEnvEnabled(name);
}

void requireDefaultPPMCLitePaperExactIfRequested() {
    if (!pmsf::defaultPPMCLitePaperExactRequired()) return;
    throw std::runtime_error(
        "[DefaultPPMCLite] requested paper-exact PPMC replay, but this Lite "
        "path is an optimized transport variant with unverified curvature "
        "prediction / paper-exact replay semantics.");
}

}  // namespace

namespace {
bool liteFrenetEnabled() {
    const char* e = std::getenv("PMSF_LITE_ENABLE_FRENET");
    return e && e[0] == '1';
}
}  // namespace

int DefaultPPMCLiteSymbolCollectOperator::exportToBuffer(
    char* buffer, bool enableQuantization, bool enableCompress) {
    if (buffer) {
        requireDefaultPPMCLitePaperExactIfRequested();
    }
    return OptimizedSymbolCollectOperator::exportToBuffer(
        buffer, enableQuantization, enableCompress);
}

void DefaultPPMCLiteSymbolCollectOperator::collect(std::vector<MCGAL::Halfedge*> seeds) {
    collect(seeds[0]);
}

void DefaultPPMCLiteSymbolCollectOperator::collect(MCGAL::Halfedge* seed) {
    // ── Same v3 per-edge BFS as OptimizedSymbolCollectOperator::collect
    // ── plus one extra parallel queue: the Frenet-rotated quantized
    //    residual for each split face. We compute the rotation HERE
    //    (collect-time) rather than at export-time because halfedge
    //    pointers stored across rounds may reference halfedges mutated
    //    by later rounds' eliminate operations.
    std::deque<char> facetSym;
    std::deque<char> edgeSym;
    std::deque<char> edgeSymCtx;
    std::deque<char> vertexSym;
    std::deque<MCGAL::Point> points;
    std::deque<MCGAL::Point> vertexPoints;
    std::deque<MCGAL::PointInt> ipoints;
    std::deque<MCGAL::Vector3i> rotatedRes;  // parallel to ipoints
    std::queue<int> gateQueue;
    gateQueue.push(seed->poolId());
    int current_version = MCGAL::BFSVersionManager::current_version++;

    // Frenet-rotation helper (uses `h` — the BFS-popped halfedge — as gate,
    // same as decoder). Returns raw (x,y,z) residual if the basis can't be
    // built (nullptr gate / degenerate normal); decoder uses the identical
    // fallback rule so round-trip is exact.
    //
    // ⚠️ Current status: Phase-A Frenet rotation produces byte-symmetric
    // bit-stream but the encoder/decoder Frenet bases are NOT bit-exact
    // on all meshes (different mesh states between encode-time and
    // decode-time result in slightly different face-normals and halfedge
    // endpoints → different basis → residual does not round-trip to the
    // exact integer). Disabling by default lets this class ship as a
    // correct Lite pipeline while Phase-A geometry symmetry is iterated
    // on. Enable via env `PMSF_LITE_ENABLE_FRENET=1`.
    const bool enableFrenet = liteFrenetEnabled();
    auto rotateFrenet = [enableFrenet](const MCGAL::PointInt& r, MCGAL::Halfedge* gate)
        -> MCGAL::Vector3i {
        if (!enableFrenet || gate == nullptr) {
            return MCGAL::Vector3i(r.x(), r.y(), r.z());
        }
        MCGAL::Vector3 t1(0, 0, 0), t2(0, 0, 0);
        MCGAL::Vector3 normal = MCGAL::computeNormalFromHeh(gate);
        if (normal.length() == 0) {
            return MCGAL::Vector3i(r.x(), r.y(), r.z());
        }
        MCGAL::Vector3 v1(gate->vertex()->point());
        MCGAL::Vector3 v2(gate->opposite() != nullptr
                              ? gate->opposite()->vertex()->point()
                              : gate->next()->vertex()->point());
        MCGAL::determineFrenetFrame(v1, v2, normal, t1, t2);
        MCGAL::Vector3i distI(r.x(), r.y(), r.z());
        return MCGAL::frenetRotation(distI, t1, t2, normal);
    };

    while (!gateQueue.empty()) {
        int hid = gateQueue.front();
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndexInSubPool(mesh_->meshId(), hid);
        gateQueue.pop();
        if (h->isVisited(current_version) || h->isRemoved()) {
            continue;
        }
        h->setVisited(current_version);

        // ===== facet sym =====
        if (!h->face()->isVisited(current_version)) {
            MCGAL::Facet* f = h->face();
            f->setVisited(current_version);
            unsigned sym = f->isSplittable();
            facetSym.push_back(sym);
            if (sym) {
                MCGAL::Point rmved = f->getRemovedVertexPos();
                MCGAL::PointInt resi = f->getRemovedVertexPosInt();
                points.push_back(rmved);
                ipoints.push_back(resi);
                // Pre-rotate residual now (collect-time) since halfedge
                // pointers are only valid within the current BFS pass.
                rotatedRes.push_back(rotateFrenet(resi, h));
            }
        }

        // ===== edge sym（first-pop per-edge）=====
        MCGAL::Halfedge* hOpp = h->opposite();
        bool emit = false;
        if (hOpp == nullptr) {
            emit = true;
        } else if (!hOpp->isVisited(current_version)) {
            emit = true;
        }
        if (emit) {
            edgeSymCtx.push_back((char)1);
            bool added = h->isAdded() || (hOpp && hOpp->isAdded());
            edgeSym.push_back(added ? 1 : 0);
        }

        if (header_.isEnableAdjustment() && !h->vertex()->isVisited(current_version)) {
            h->vertex()->setVisited(current_version);
            if (h->vertex()->isAdjusted()) {
                vertexSym.push_back(1);
                vertexPoints.push_back(h->vertex()->vertexPoint());
            } else {
                vertexSym.push_back(0);
            }
        }

        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp2 = hIt->opposite();
            if (hOpp2 != nullptr && !hOpp2->isVisited(current_version)) {
                gateQueue.push(hOpp2->poolId());
            }
            if (hIt != nullptr && !hIt->isVisited(current_version)) {
                gateQueue.push(hIt->poolId());
            }
            hIt = hIt->next();
        } while (hIt != h);
    }

    // Push into base-class queues (protected members). We MUST preserve
    // the exact order the base class uses, so ColumnarSerializeOperator's
    // table builders still consume them correctly.
    facetSymbolQueues.push_back(facetSym);
    edgeSymbolQueues.push_back(edgeSym);
    edgeSymContextQueues.push_back(edgeSymCtx);
    ipointQueues.push_back(ipoints);
    pointQueues.push_back(points);
    vertexSymbolQueues.push_back(vertexSym);
    vertexPointQueues.push_back(vertexPoints);
    rotatedResidualQueues_.push_back(rotatedRes);

    pmsf::RoundSymbols rs;
    pmsf::setDefaultCodecPaperRoundSymbols(
        rs, facetSym, edgeSym, vertexSym, points, vertexPoints);
    pmsf::appendDefaultPPMCLiteRemovedPointSideChannel(rs, ipoints);
    pmsf::appendDefaultPPMCLiteVertexPointSideChannel(rs, mesh_, vertexPoints);
    rounds_.push_back(std::move(rs));
}

const pmsf::PMSFSymbolTable* DefaultPPMCLiteSymbolCollectOperator::pmsfSymbolTable() const {
    const pmsf::PMSFSymbolTable* base = OptimizedSymbolCollectOperator::pmsfSymbolTable();
    if (!base || base->empty()) {
        return nullptr;
    }
    const pmsf::SymbolCode geometryBasis = liteFrenetEnabled() ? 1 : 0;
    pmsf::copyPMSFSymbolTableWithRowAugmentation(
        base, paperTableCache_, [geometryBasis](pmsf::PMSFSymbolRecord& row) {
        pmsf::appendDefaultPPMCLiteStorageSideChannel(
            row.delta, static_cast<std::int32_t>(geometryBasis));
    });
    return paperTableCache_.empty() ? nullptr : &paperTableCache_;
}

int DefaultPPMCLiteSymbolCollectOperator::exportToBufferWithRangeCoder(
    char* buffer, bool isEnableQuantization) {
    if (buffer) {
        requireDefaultPPMCLitePaperExactIfRequested();
    }
    // The facet / edge / vertex streams are encoded exactly like the base
    // class — v3 per-edge single-model RC. The only semantic difference
    // is the GEOMETRY stream, where residuals (ipointQueues) get
    // Frenet-rotated before bit-plane encoding.
    //
    // The buffer layout is BINARY-identical to the base class's layout
    // (same header ints + same 4 blocks in the same order). Columnar
    // dispatches on `blob_layout == CODEC_006` to pick the right
    // decoder, so we can't break layout.

    size_t offset = 0;

    constexpr size_t SYM_BUF_SIZE = BUFFER_SIZE / 6;
    constexpr size_t POINT_BUF_SIZE = BUFFER_SIZE / 3;

    char* rcFacetBuf = new char[SYM_BUF_SIZE];
    char* rcEdgeBuf = new char[SYM_BUF_SIZE];
    char* rcVertexBuf = new char[SYM_BUF_SIZE];
    char* rcPointBuf = new char[POINT_BUF_SIZE];
    std::memset(rcFacetBuf, 0, SYM_BUF_SIZE);
    std::memset(rcEdgeBuf, 0, SYM_BUF_SIZE);
    std::memset(rcVertexBuf, 0, SYM_BUF_SIZE);
    std::memset(rcPointBuf, 0, POINT_BUF_SIZE);

    int totalFacetSymbols = 0;
    int totalEdgeSymbols = 0;
    int totalVertexSymbols = 0;
    int totalPointCoords = 0;
    for (auto& q : facetSymbolQueues) totalFacetSymbols += (int)q.size();
    for (auto& q : edgeSymbolQueues) totalEdgeSymbols += (int)q.size();
    for (auto& q : vertexSymbolQueues) totalVertexSymbols += (int)q.size();
    for (size_t i = 0; i < pointQueues.size(); i++) {
        totalPointCoords += (int)(pointQueues[i].size() * 3);
        totalPointCoords += (int)(vertexPointQueues[i].size() * 3);
    }

    size_t rcFacetSize = 0, rcEdgeSize = 0, rcVertexSize = 0, rcPointSize = 0;

    auto mergeCharQueuesReversed = [](const std::vector<std::deque<char>>& queues) {
        std::deque<uint8_t> out;
        for (int i = static_cast<int>(queues.size()) - 1; i >= 0; --i) {
            for (char sym : queues[i]) {
                out.push_back(static_cast<uint8_t>(sym));
            }
        }
        return out;
    };

    // ── (1) facet sym RC (single adaptive model, same as v3) ──
    {
        auto syms = mergeCharQueuesReversed(facetSymbolQueues);
        rcFacetSize = pmsf::encodeUint8Queue(syms, 2, rcFacetBuf, SYM_BUF_SIZE);
    }

    // ── (2) edge sym RC (single adaptive model, same as v3 per-edge) ──
    {
        auto syms = mergeCharQueuesReversed(edgeSymbolQueues);
        rcEdgeSize = pmsf::encodeUint8Queue(syms, 2, rcEdgeBuf, SYM_BUF_SIZE);
    }

    // ── (3) vertex sym RC (optional) ──
    if (header_.isEnableAdjustment()) {
        auto syms = mergeCharQueuesReversed(vertexSymbolQueues);
        rcVertexSize = pmsf::encodeUint8Queue(syms, 2, rcVertexBuf, SYM_BUF_SIZE);
    }

    // ── (4) GEOMETRY: bit-plane with Frenet-rotated residuals ──
    //
    // This is the sole BPV optimization in Lite Phase A. For each split
    // face, we look up its BFS gate halfedge (recorded in collect()),
    // build a Frenet basis (t1,t2,normal) from the face's normal and the
    // gate halfedge's edge vector, rotate the stored residual through
    // `MCGAL::frenetRotation(...)` into (α,β,γ), and feed those into the
    // SAME bit-plane RC that v3 uses. Because frenet/invFrenet are
    // bijective (proven in PPMC), the decoder can recover the original
    // residual exactly.
    //
    // Invariant: pointQueues/ipointQueues/rotatedResidualQueues_ are all
    // populated in the same BFS order for split faces → indexing by
    // round `i` and within-round position is aligned across the three.
    {
        size_t rcOffset = 0;
        unsigned nbQuantBits = mesh_->i_nbQuantBits;
        const bool useResidualBitPlane = enablePrediction_ && isEnableQuantization;
        const bool useAdaptiveModel =
            !useResidualBitPlane && isEnableQuantization && nbQuantBits <= 16;

        const int BP_BITS = static_cast<int>(nbQuantBits) + 1;

        if (useResidualBitPlane) {
            std::deque<MCGAL::PointInt> allResiduals;
            for (int i = (int)pointQueues.size() - 1; i >= 0; i--) {
                const bool haveRot =
                    (i < (int)rotatedResidualQueues_.size()) &&
                    (rotatedResidualQueues_[i].size() == ipointQueues[i].size());
                if (haveRot) {
                    for (auto& rot : rotatedResidualQueues_[i]) {
                        allResiduals.emplace_back(rot[0], rot[1], rot[2]);
                    }
                } else {
                    for (const auto& res : ipointQueues[i]) {
                        allResiduals.emplace_back(
                            static_cast<int32_t>(res.x()),
                            static_cast<int32_t>(res.y()),
                            static_cast<int32_t>(res.z()));
                    }
                }

                for (const auto& point : vertexPointQueues[i]) {
                    MCGAL::PointInt pi = mesh_->floatPosToInt(point);
                    for (int j = 0; j < 3; ++j) {
                        if (pi[j] >= (1 << nbQuantBits)) pi[j] = pi[j] - 1;
                        else if (pi[j] <= 0) pi[j] = 0;
                    }
                    allResiduals.push_back(pi);
                }
            }
            rcPointSize = pmsf::encodeResidualBitPlaneQueue(
                allResiduals, BP_BITS, rcPointBuf, SYM_BUF_SIZE);
        } else {
            pmsf::MultiModelRangeEncoder encoder(rcPointBuf, &rcOffset);
            int coordModel = -1;
            if (useAdaptiveModel) {
                coordModel = encoder.addModel(
                    {1 << nbQuantBits, 18, 1 << 17});
            }

            for (int i = (int)pointQueues.size() - 1; i >= 0; i--) {
                for (auto& point : pointQueues[i]) {
                    if (isEnableQuantization) {
                        MCGAL::PointInt pi = mesh_->floatPosToInt(point);
                        for (int j = 0; j < 3; j++) {
                            if (pi[j] > (1 << nbQuantBits))
                                pi[j] = (1 << nbQuantBits) - 1;
                            else if (pi[j] < 0)
                                pi[j] = 0;
                            if (useAdaptiveModel) {
                                encoder.encode(coordModel, pi[j]);
                            } else {
                                encoder.encodeRawSymbol(pi[j], nbQuantBits);
                            }
                        }
                    } else {
                        for (int j = 0; j < 3; j++) {
                            float f = static_cast<float>(point[j]);
                            uint32_t raw;
                            std::memcpy(&raw, &f, sizeof(float));
                            encoder.encodeRawShort(static_cast<uint16_t>(raw & 0xFFFF));
                            encoder.encodeRawShort(static_cast<uint16_t>((raw >> 16) & 0xFFFF));
                        }
                    }
                }

                for (auto& point : vertexPointQueues[i]) {
                    if (isEnableQuantization) {
                        MCGAL::PointInt pi = mesh_->floatPosToInt(point);
                        for (int j = 0; j < 3; j++) {
                            if (pi[j] >= (1 << nbQuantBits)) pi[j] = pi[j] - 1;
                            else if (pi[j] <= 0) pi[j] = 0;
                            if (useAdaptiveModel) {
                                encoder.encode(coordModel, pi[j]);
                            } else {
                                encoder.encodeRawSymbol(pi[j], nbQuantBits);
                            }
                        }
                    } else {
                        for (int j = 0; j < 3; j++) {
                            float f = static_cast<float>(point[j]);
                            uint32_t raw;
                            std::memcpy(&raw, &f, sizeof(float));
                            encoder.encodeRawShort(static_cast<uint16_t>(raw & 0xFFFF));
                            encoder.encodeRawShort(static_cast<uint16_t>((raw >> 16) & 0xFFFF));
                        }
                    }
                }
            }

            rcPointSize = static_cast<int>(encoder.finish());
        }
    }

    // ── Write buffer header + 4 blocks (identical layout to v3) ──
    writeInt(buffer, offset, (int)rcFacetSize);
    writeInt(buffer, offset, (int)rcEdgeSize);
    if (header_.isEnableAdjustment()) writeInt(buffer, offset, (int)rcVertexSize);
    writeInt(buffer, offset, (int)rcPointSize);

    writeInt(buffer, offset, totalFacetSymbols);
    writeInt(buffer, offset, totalEdgeSymbols);
    if (header_.isEnableAdjustment()) writeInt(buffer, offset, totalVertexSymbols);
    writeInt(buffer, offset, totalPointCoords);

    writeCharPointer(buffer, offset, rcFacetBuf, (int)rcFacetSize);
    writeCharPointer(buffer, offset, rcEdgeBuf, (int)rcEdgeSize);
    if (header_.isEnableAdjustment())
        writeCharPointer(buffer, offset, rcVertexBuf, (int)rcVertexSize);
    writeCharPointer(buffer, offset, rcPointBuf, (int)rcPointSize);

    {
        const char* env = std::getenv("PMSF_BPV_DIAG");
        if (env && env[0] == '1') {
            std::cerr << "[PCLTTM bpv][lite] facet=" << rcFacetSize
                      << " (raw=" << totalFacetSymbols << ") "
                      << "edge=" << rcEdgeSize
                      << " (raw=" << totalEdgeSymbols << ") "
                      << "point=" << rcPointSize
                      << " (coords=" << totalPointCoords << ")" << std::endl;
        }
    }

    delete[] rcFacetBuf;
    delete[] rcEdgeBuf;
    delete[] rcVertexBuf;
    delete[] rcPointBuf;
    return (int)offset;
}
