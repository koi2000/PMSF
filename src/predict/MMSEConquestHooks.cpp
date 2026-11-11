// SPDX-License-Identifier: Apache-2.0
//
// MMSEConquestHooks.cpp — paper §2.2/§2.3 predictor maths on the coarse graph.
//
// See MMSEConquestHooks.h for the symmetry contract.  Everything here works in
// the integer cell domain on the *coarse graph of the current layer*:
//   encoder: finalizeEncoderLayer() runs right after `decimateLayer` returned
//            (engine mesh == coarse mesh);
//   decoder: beginDecoderLayer() snapshots the layer-start mesh (which IS the
//            coarse mesh of that layer).
//
// The coarse graph adjacency is built from **face loops** (see
// buildCoarseAdjacency) rather than from vertex umbrella walks: face loops are
// identical on both sides, while proxy/halfedge-based walks can terminate early
// when a proxy points at a merged halfedge and then the encoder and decoder
// predict different secondary rings (measured: decoded topology/geometry FAIL
// before this fix).
//
// Cell-domain dual-ring (paper eq.(3)):
//   u    = mean of the gate ring cells
//   for each ring vertex l: c_l = #coarse neighbours of l; v_l = c_l + 1
//                           (paper's v_l counts the removed center j)
//   Su   = sum_l 1/v_l^2 * ( v_l*x_l - sum_{k in coarse N_l} x_k )
//   w    = 1 + alpha * sum_l 1/v_l^2 ;  xhat = (u + alpha*Su) / w
// MMSE LS (paper eq.(4)-(8)): S = 1-ring ∪ 2-ring on the coarse graph,
//   R_kl = rho^{d_kl}, C_k = rho^{d_kj}, w = R^{-1} C, xhat = sum w_k x_k.

#include "predict/MMSEConquestHooks.h"
#include "conquest/ConquestDecodeLayer.h"

#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Point.h"
#include "Vertex.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <vector>

namespace pmsf {
namespace mmse_pred {

namespace {

constexpr int kMaxFaceSteps = 512;

void solveDense(double* R, double* C, int n) {
    for (int col = 0; col < n; ++col) {
        int pivot = col;
        double best = std::fabs(R[static_cast<std::size_t>(col) * n + col]);
        for (int row = col + 1; row < n; ++row) {
            const double v = std::fabs(R[static_cast<std::size_t>(row) * n + col]);
            if (v > best) { best = v; pivot = row; }
        }
        if (best < 1e-12) {
            for (int i = 0; i < n; ++i) C[i] = 0.0;
            return;
        }
        if (pivot != col) {
            for (int k = 0; k < n; ++k) {
                std::swap(R[static_cast<std::size_t>(col) * n + k],
                          R[static_cast<std::size_t>(pivot) * n + k]);
            }
            std::swap(C[col], C[pivot]);
        }
        const double inv = 1.0 / R[static_cast<std::size_t>(col) * n + col];
        for (int row = col + 1; row < n; ++row) {
            const double f = R[static_cast<std::size_t>(row) * n + col] * inv;
            if (f == 0.0) continue;
            for (int k = col; k < n; ++k) {
                R[static_cast<std::size_t>(row) * n + k] -=
                    f * R[static_cast<std::size_t>(col) * n + k];
            }
            C[row] -= f * C[col];
        }
    }
    for (int row = n - 1; row >= 0; --row) {
        double acc = C[row];
        for (int k = row + 1; k < n; ++k) {
            acc -= R[static_cast<std::size_t>(row) * n + k] * C[k];
        }
        const double diag = R[static_cast<std::size_t>(row) * n + row];
        C[row] = (std::fabs(diag) < 1e-12) ? 0.0 : acc / diag;
    }
}

inline void addEdge(std::unordered_map<std::uint32_t, std::vector<std::uint32_t>>& adj,
                    std::uint32_t a, std::uint32_t b) {
    std::vector<std::uint32_t>& list = adj[a];
    for (std::uint32_t x : list) {
        if (x == b) return;
    }
    list.push_back(b);
}

}  // namespace

// ---------------------------------------------------------------------------
// Coarse-graph adjacency snapshot (face-loop based; symmetric by construction)
// ---------------------------------------------------------------------------
void MMSECQuestGeomState::buildCoarseAdjacency() const {
    adj_.clear();
    if (mesh_ == nullptr) return;
    for (MCGAL::Facet* f : mesh_->faces()) {
        if (f == nullptr || f->isRemoved()) continue;
        MCGAL::Halfedge* st = f->proxyHalfedge();
        if (st == nullptr) continue;
        MCGAL::Halfedge* it = st;
        for (int steps = 0; steps < kMaxFaceSteps; ++steps) {
            MCGAL::Vertex* a = it->vertex();
            MCGAL::Vertex* b = it->end_vertex();
            if (a != nullptr && b != nullptr && a != b && !a->isRemoved() && !b->isRemoved()) {
                const std::uint32_t ia = static_cast<std::uint32_t>(a->poolId());
                const std::uint32_t ib = static_cast<std::uint32_t>(b->poolId());
                addEdge(adj_, ia, ib);
                addEdge(adj_, ib, ia);
            }
            MCGAL::Halfedge* nxt = it->next();
            if (nxt == nullptr || nxt == st) break;
            it = nxt;
        }
    }
}

// ---------------------------------------------------------------------------
// Gate context + candidates
// ---------------------------------------------------------------------------
bool MMSECQuestGeomState::prepareGate(
    const std::vector<std::uint32_t>& ringIds,
    std::uint32_t centerId,
    const std::unordered_set<std::uint32_t>& excludedExtra,
    const double bboxMin[3], double step,
    GateCtx& ctx) const {
    ctx.ringVerts.clear();
    ctx.ringCells.clear();
    ctx.excluded = excludedExtra;
    ctx.excluded.insert(centerId);
    if (ringIds.empty() || mesh_ == nullptr) return false;
    for (std::uint32_t id : ringIds) {
        MCGAL::Vertex* v = MCGAL::contextPool.getVertexByIndexInSubPool(
            meshId_, static_cast<int>(id));
        if (v == nullptr || v->isRemoved()) continue;
        ctx.ringVerts.push_back(v);
        const MCGAL::Point p = v->point();
        ctx.ringCells.emplace_back(cellOf(p.x(), bboxMin[0], step),
                                   cellOf(p.y(), bboxMin[1], step),
                                   cellOf(p.z(), bboxMin[2], step));
    }
    return !ctx.ringCells.empty();
}

void MMSECQuestGeomState::dualRingCells(const GateCtx& ctx,
                                        const double bboxMin[3], double step,
                                        int out05[3], int out1[3]) const {
    out05[0] = out05[1] = out05[2] = 0;
    out1[0] = out1[1] = out1[2] = 0;
    const std::size_t n = ctx.ringCells.size();
    if (n == 0) return;
    double ux = 0, uy = 0, uz = 0;
    for (const auto& c : ctx.ringCells) {
        ux += c.x(); uy += c.y(); uz += c.z();
    }
    ux /= static_cast<double>(n);
    uy /= static_cast<double>(n);
    uz /= static_cast<double>(n);

    double sux = 0, suy = 0, suz = 0, wsum = 0.0;
    for (std::size_t i = 0; i < ctx.ringVerts.size(); ++i) {
        const std::uint32_t id = static_cast<std::uint32_t>(ctx.ringVerts[i]->poolId());
        // c_l = coarse valence (the center j is not in the coarse graph); the
        // paper's v_l = c_l + 1 counts j.
        std::vector<std::uint32_t> nbrs;
        if (const std::vector<std::uint32_t>* list = coarseAdj(id)) {
            for (std::uint32_t nb : *list) {
                if (ctx.excluded.count(nb) != 0) continue;
                nbrs.push_back(nb);
            }
        }
        const int cl = static_cast<int>(nbrs.size());
        const int vl = cl + 1;
        const double w = 1.0 / (static_cast<double>(vl) * static_cast<double>(vl));
        double skx = 0, sky = 0, skz = 0;
        for (std::uint32_t nb : nbrs) {
            MCGAL::Vertex* v = MCGAL::contextPool.getVertexByIndexInSubPool(
                meshId_, static_cast<int>(nb));
            if (v == nullptr) continue;
            const MCGAL::Point p = v->point();
            skx += cellOf(p.x(), bboxMin[0], step);
            sky += cellOf(p.y(), bboxMin[1], step);
            skz += cellOf(p.z(), bboxMin[2], step);
        }
        const MCGAL::PointInt& xl = ctx.ringCells[i];
        sux += w * (static_cast<double>(vl) * xl.x() - skx);
        suy += w * (static_cast<double>(vl) * xl.y() - sky);
        suz += w * (static_cast<double>(vl) * xl.z() - skz);
        wsum += w;
    }
    for (int pass = 0; pass < 2; ++pass) {
        const double alpha = (pass == 0) ? 0.5 : 1.0;
        const double w = 1.0 + alpha * wsum;
        if (w <= 0.0) continue;
        int* dst = (pass == 0) ? out05 : out1;
        dst[0] = static_cast<int>(roundHalfAway((ux + alpha * sux) / w));
        dst[1] = static_cast<int>(roundHalfAway((uy + alpha * suy) / w));
        dst[2] = static_cast<int>(roundHalfAway((uz + alpha * suz) / w));
    }
}

void MMSECQuestGeomState::lsCells(const GateCtx& ctx,
                                  const double bboxMin[3], double step,
                                  int out[3]) const {
    out[0] = out[1] = out[2] = 0;
    const std::size_t n1 = ctx.ringVerts.size();
    if (n1 < 3 || !lsEnabled_) return;

    // S = 1-ring ∪ 2-ring on the coarse graph (adjacency snapshot).
    std::vector<std::uint32_t> ids;
    std::vector<MCGAL::PointInt> cells;
    std::vector<int> distJ;
    for (std::size_t i = 0; i < n1; ++i) {
        ids.push_back(static_cast<std::uint32_t>(ctx.ringVerts[i]->poolId()));
        cells.push_back(ctx.ringCells[i]);
        distJ.push_back(1);
    }
    for (std::size_t i = 0; i < n1; ++i) {
        const std::vector<std::uint32_t>* list = coarseAdj(ids[i]);
        if (list == nullptr) continue;
        for (std::uint32_t nb : *list) {
            if (ctx.excluded.count(nb) != 0) continue;
            bool inS = false;
            for (std::uint32_t s : ids) {
                if (s == nb) { inS = true; break; }
            }
            if (inS) continue;
            MCGAL::Vertex* v = MCGAL::contextPool.getVertexByIndexInSubPool(
                meshId_, static_cast<int>(nb));
            if (v == nullptr) continue;
            ids.push_back(nb);
            const MCGAL::Point p = v->point();
            cells.emplace_back(cellOf(p.x(), bboxMin[0], step),
                               cellOf(p.y(), bboxMin[1], step),
                               cellOf(p.z(), bboxMin[2], step));
            distJ.push_back(2);
        }
    }
    const int n = static_cast<int>(ids.size());
    if (n < 3 || n > 32) return;

    std::vector<double> R(static_cast<std::size_t>(n) * n, 0.0);
    std::vector<double> C(static_cast<std::size_t>(n), 0.0);
    constexpr int kMaxDepth = 5;
    auto distance = [&](int ai, int bi) -> int {
        const std::uint32_t a = ids[static_cast<std::size_t>(ai)];
        const std::uint32_t b = ids[static_cast<std::size_t>(bi)];
        if (a == b) return 0;
        std::vector<std::uint32_t> frontier{a};
        std::vector<std::uint32_t> visited{a};
        for (int depth = 1; depth <= kMaxDepth; ++depth) {
            std::vector<std::uint32_t> next;
            for (std::uint32_t u : frontier) {
                const std::vector<std::uint32_t>* list = coarseAdj(u);
                if (list == nullptr) continue;
                for (std::uint32_t v : *list) {
                    if (ctx.excluded.count(v) != 0) continue;
                    if (v == b) return depth;
                    bool seen = false;
                    for (std::uint32_t w : visited) {
                        if (w == v) { seen = true; break; }
                    }
                    if (!seen) {
                        visited.push_back(v);
                        next.push_back(v);
                    }
                }
            }
            frontier.swap(next);
            if (frontier.empty()) break;
        }
        return kMaxDepth;
    };
    for (int i = 0; i < n; ++i) {
        C[static_cast<std::size_t>(i)] =
            std::pow(kConquestRho, static_cast<double>(distJ[static_cast<std::size_t>(i)]));
        for (int k = 0; k < n; ++k) {
            const int d = distance(i, k);
            R[static_cast<std::size_t>(i) * n + k] =
                std::pow(kConquestRho, static_cast<double>(d));
        }
    }
    constexpr double kRidge = 1e-9;
    for (int i = 0; i < n; ++i) R[static_cast<std::size_t>(i) * n + i] += kRidge;
    solveDense(R.data(), C.data(), n);
    double wsum = 0, wabs = 0, wmax = 0;
    for (int i = 0; i < n; ++i) {
        wsum += C[static_cast<std::size_t>(i)];
        wabs += std::fabs(C[static_cast<std::size_t>(i)]);
        wmax = std::max(wmax, std::fabs(C[static_cast<std::size_t>(i)]));
    }
    if (!(wsum > 0.5 && wsum < 1.5) || wabs > 4.0 || wmax > 2.0) return;
    double x = 0, y = 0, z = 0;
    for (int i = 0; i < n; ++i) {
        const double wi = C[static_cast<std::size_t>(i)];
        x += wi * cells[static_cast<std::size_t>(i)].x();
        y += wi * cells[static_cast<std::size_t>(i)].y();
        z += wi * cells[static_cast<std::size_t>(i)].z();
    }
    out[0] = static_cast<int>(roundHalfAway(x));
    out[1] = static_cast<int>(roundHalfAway(y));
    out[2] = static_cast<int>(roundHalfAway(z));
}

void MMSECQuestGeomState::markFineRingPositions(const std::vector<std::uint32_t>& centers) {
    fineKnown_.assign(fineTopology_.vertices.size(), true);
    for (auto center : centers) fineKnown_.at(fineTopology_.poolToIndex.at(center)) = false;
}

bool MMSECQuestGeomState::fineRingLsCells(std::uint32_t centerId, const double bboxMin[3],
                                        double step, int out[3]) const {
    const auto center = fineTopology_.poolToIndex.at(centerId);
    const auto model = fineRingWorkspace_.model(fineTopology_.adjacency, fineKnown_, center);
    auto& stats = fineRingStats_;
    ++stats.evaluated;
    stats.maxSupport = std::max(stats.maxSupport, model.support.size());
    stats.largeSupport += model.support.size() > 32;
    stats.distance4Pairs += std::count(model.pairDistances.begin(), model.pairDistances.end(), 4) / 2;
    const auto weights = solveFineRingWeights(model);
    if (!weights) { ++stats.unavailable; return false; }
    stats.minAbsPivot = std::min(stats.minAbsPivot, weights->minAbsPivot);
    double prediction[3] = {}, weightSum = 0., weightL1 = 0.;
    for (std::size_t i = 0; i < model.support.size(); ++i) {
        auto* vertex = fineTopology_.vertices[model.support[i]];
        if (!vertex || vertex->isRemoved()) throw std::runtime_error("MMSE fine ring: known position is not live");
        const auto point = vertex->point();
        const double weight = weights->coefficients[i];
        weightSum += weight;
        weightL1 += std::abs(weight);
        stats.maxAbsWeight = std::max(stats.maxAbsWeight, std::abs(weight));
        const double xyz[] = {point.x(), point.y(), point.z()};
        for (int axis = 0; axis < 3; ++axis) prediction[axis] += weight * cellOf(xyz[axis],bboxMin[axis],step);
    }
    stats.maxWeightL1 = std::max(stats.maxWeightL1, weightL1);
    stats.minWeightSum = std::min(stats.minWeightSum, weightSum);
    stats.maxWeightSum = std::max(stats.maxWeightSum, weightSum);
    for (int axis = 0; axis < 3; ++axis) {
        prediction[axis] = roundHalfAway(prediction[axis]);
        if (!std::isfinite(prediction[axis]) || prediction[axis] < std::numeric_limits<int>::min() ||
            prediction[axis] > std::numeric_limits<int>::max()) { ++stats.unavailable; return false; }
    }
    for (int axis = 0; axis < 3; ++axis) out[axis] = static_cast<int>(prediction[axis]);
    return true;
}

bool MMSECQuestGeomState::computeCandidates(
    const std::vector<std::uint32_t>& ringIds,
    std::uint32_t centerId,
    const std::unordered_set<std::uint32_t>& excludedExtra,
    const double bboxMin[3], double step,
    std::array<std::array<int, 3>, kConquestModeCount>& outCell) const {
    for (int m = 0; m < kConquestModeCount; ++m) {
        outCell[static_cast<std::size_t>(m)][0] = 0;
        outCell[static_cast<std::size_t>(m)][1] = 0;
        outCell[static_cast<std::size_t>(m)][2] = 0;
    }
    GateCtx ctx;
    if (!prepareGate(ringIds, centerId, excludedExtra, bboxMin, step, ctx)) return !fineRingPrediction_;

    double sx = 0, sy = 0, sz = 0;
    for (const auto& c : ctx.ringCells) {
        sx += c.x(); sy += c.y(); sz += c.z();
    }
    const double cnt = static_cast<double>(ctx.ringCells.size());
    outCell[kConquestModeBarycentric][0] = static_cast<int>(roundHalfAway(sx / cnt));
    outCell[kConquestModeBarycentric][1] = static_cast<int>(roundHalfAway(sy / cnt));
    outCell[kConquestModeBarycentric][2] = static_cast<int>(roundHalfAway(sz / cnt));

    int d05[3], d1[3];
    dualRingCells(ctx, bboxMin, step, d05, d1);
    for (int k = 0; k < 3; ++k) {
        outCell[kConquestModeDualRing05][static_cast<std::size_t>(k)] = d05[k];
        outCell[kConquestModeDualRing1][static_cast<std::size_t>(k)] = d1[k];
    }
    int ls[3] = {};
    bool available = true;
    if (fineRingPrediction_) available = fineRingLsCells(centerId, bboxMin, step, ls);
    else lsCells(ctx, bboxMin, step, ls);
    for (int k = 0; k < 3; ++k) {
        outCell[kConquestModeMMSELS][static_cast<std::size_t>(k)] = ls[k];
    }
    return available;
}

void MMSECQuestGeomState::computeModePrediction(
    int mode,
    const std::vector<std::uint32_t>& ringIds,
    std::uint32_t centerId,
    const std::unordered_set<std::uint32_t>& excludedExtra,
    const double bboxMin[3], double step,
    int out[3]) const {
    out[0] = out[1] = out[2] = 0;
    GateCtx ctx;
    if (!prepareGate(ringIds, centerId, excludedExtra, bboxMin, step, ctx)) return;
    if (mode == kConquestModeBarycentric) {
        double sx = 0, sy = 0, sz = 0;
        for (const auto& c : ctx.ringCells) {
            sx += c.x(); sy += c.y(); sz += c.z();
        }
        const double cnt = static_cast<double>(ctx.ringCells.size());
        out[0] = static_cast<int>(roundHalfAway(sx / cnt));
        out[1] = static_cast<int>(roundHalfAway(sy / cnt));
        out[2] = static_cast<int>(roundHalfAway(sz / cnt));
        return;
    }
    if (mode == kConquestModeDualRing05 || mode == kConquestModeDualRing1) {
        int d05[3], d1[3];
        dualRingCells(ctx, bboxMin, step, d05, d1);
        const int* src = (mode == kConquestModeDualRing05) ? d05 : d1;
        for (int k = 0; k < 3; ++k) out[k] = src[k];
        return;
    }
    if (mode == kConquestModeMMSELS) {
        if (fineRingPrediction_) {
            if (!fineRingLsCells(centerId, bboxMin, step, out))
                throw std::runtime_error("MMSE fine ring: unavailable mode 3 selected by stream");
        } else lsCells(ctx, bboxMin, step, out);
        return;
    }
}

// ---------------------------------------------------------------------------
// Encoder post-layer pass
// ---------------------------------------------------------------------------
void MMSECQuestGeomState::finalizeEncoderLayer(
    std::vector<std::array<int, 3>>& deltas,
    std::vector<std::uint8_t>& modes) {
    modes.clear();
    layerModes_.clear();
    lastModeHist_[0] = lastModeHist_[1] = lastModeHist_[2] = lastModeHist_[3] = 0;
    const std::size_t n = std::min(records_.size(), deltas.size());
    if (n == 0) return;
    if (recursiveTopology_) {
        if (records_.size() != deltas.size()) throw std::runtime_error("topology predictor: gate/residual record counts differ");
        finalizeRecursiveEncoderLayer(deltas, modes);
        return;
    }

    // The engine mesh is now the coarse mesh of this layer: snapshot its graph.
    buildCoarseAdjacency();

    std::vector<std::array<int, 3>> target(n);
    for (std::size_t i = 0; i < n; ++i) {
        for (int k = 0; k < 3; ++k) {
            target[i][static_cast<std::size_t>(k)] =
                records_[i].delta[k] + records_[i].cellMean[k];
        }
    }

    const std::unordered_set<std::uint32_t> noExtra;
    const std::size_t chunk = static_cast<std::size_t>(clusterSize_);
    for (std::size_t begin = 0; begin < n; begin += chunk) {
        const std::size_t end = std::min(n, begin + chunk);
        double err[kConquestModeCount] = {0, 0, 0, 0};
        std::vector<std::array<std::array<int, 3>, kConquestModeCount>> cand(end - begin);
        for (std::size_t i = begin; i < end; ++i) {
            computeCandidates(records_[i].ringIds, records_[i].centerId, noExtra,
                              records_[i].bboxMin, records_[i].step, cand[i - begin]);
            for (int m = 0; m < kConquestModeCount; ++m) {
                for (int k = 0; k < 3; ++k) {
                    const double d =
                        static_cast<double>(target[i][static_cast<std::size_t>(k)] -
                                            cand[i - begin][static_cast<std::size_t>(m)]
                                                          [static_cast<std::size_t>(k)]);
                    err[m] += d * d;
                }
            }
        }
        int best = 0;
        for (int m = 1; m < kConquestModeCount; ++m) {
            if (err[m] < err[best]) best = m;
        }
        modes.push_back(static_cast<std::uint8_t>(best));
        ++lastModeHist_[best];
        for (std::size_t i = begin; i < end; ++i) {
            for (int k = 0; k < 3; ++k) {
                deltas[i][static_cast<std::size_t>(k)] =
                    target[i][static_cast<std::size_t>(k)] -
                    cand[i - begin][static_cast<std::size_t>(best)]
                                   [static_cast<std::size_t>(k)];
            }
        }
    }
    layerModes_ = modes;
}

// Connectivity-only partitions use the fine layer's rooted graph. It is
// snapshotted before removal on the encoder, after topology-only insertion on
// the decoder. Allocation ids are only lookup keys, never ordering inputs.
TopologyBisectionResult MMSECQuestGeomState::partitionTopology(
    const std::vector<std::uint32_t>& centers) const {
    std::vector<std::size_t> vertices;
    vertices.reserve(centers.size());
    for (auto center : centers) {
        const auto it = fineTopology_.poolToIndex.find(center);
        if (it == fineTopology_.poolToIndex.end()) throw std::runtime_error("topology predictor: center is outside rooted layer component");
        vertices.push_back(it->second);
    }
    return recursiveTopologyBisection(fineTopology_.adjacency, fineTopology_.meshBoundary,
                                     vertices, topologyThreshold_, topologySeed_);
}

void MMSECQuestGeomState::prepareDecoderTopology(
    MCGAL::Halfedge* seed, const std::vector<std::uint32_t>& centers) {
    fineTopology_ = snapshotConquestLayerTopology(seed);
    std::size_t alive = 0;
    for (auto* vertex : mesh_->vertices()) if (vertex && !vertex->isRemoved()) ++alive;
    if (alive != fineTopology_.vertices.size()) throw std::runtime_error("topology predictor: disconnected layer is not covered by the rooted protocol");
    topologyPartition_ = partitionTopology(centers);
    if (fineRingPrediction_) markFineRingPositions(centers);
    if ((!replayModes_ && !topologyPartition_.clusters.empty()) ||
        (replayModes_ && replayModes_->size() != topologyPartition_.clusters.size())) {
        throw std::runtime_error("topology predictor: derived cluster count differs from arithmetic mode table");
    }
}

void MMSECQuestGeomState::finalizeRecursiveEncoderLayer(
    std::vector<std::array<int, 3>>& deltas, std::vector<std::uint8_t>& modes) {
    buildCoarseAdjacency();
    std::vector<std::uint32_t> centers;
    for (const auto& record : records_) centers.push_back(record.centerId);
    topologyPartition_ = partitionTopology(centers);
    if (fineRingPrediction_) markFineRingPositions(centers);
    const std::unordered_set<std::uint32_t> noExtra;
    std::vector<std::array<int, 3>> targets(records_.size());
    std::vector<std::array<int, 3>> selectedPredictions;
    if (fineRingPrediction_) selectedPredictions.resize(records_.size());
    for (std::size_t i = 0; i < records_.size(); ++i)
        for (int axis = 0; axis < 3; ++axis) targets[i][axis] = records_[i].delta[axis] + records_[i].cellMean[axis];
    lastTargetCellSignature_ = conquestGeometryCellSignature(targets);
    for (const auto& members : topologyPartition_.clusters) {
        double error[kConquestModeCount] = {};
        bool mode3Available = true;
        std::vector<std::array<std::array<int, 3>, kConquestModeCount>> candidates(members.size());
        for (std::size_t local = 0; local < members.size(); ++local) {
            const auto i = members[local];
            const auto& record = records_[i];
            bool available = computeCandidates(record.ringIds, record.centerId, noExtra,
                                               record.bboxMin, record.step, candidates[local]);
            if (fineRingPrediction_ && available) {
                for (int axis = 0; axis < 3; ++axis) {
                    const auto residual = std::int64_t(targets[i][axis])-candidates[local][kConquestModeMMSELS][axis];
                    if (residual < std::numeric_limits<int>::min() || residual > std::numeric_limits<int>::max()) {
                        ++fineRingStats_.unavailable;
                        available = false;
                        break;
                    }
                }
            }
            mode3Available &= available;
            for (int mode = 0; mode < kConquestModeCount; ++mode) for (int axis = 0; axis < 3; ++axis) {
                const double difference = static_cast<double>(targets[i][axis]) - candidates[local][mode][axis];
                const double squared = difference * difference;
                error[mode] += squared;
                if (fineRingPrediction_ && available) {
                    if (mode == kConquestModeBarycentric) fineRingStats_.barycentricSseOnSolvable += squared;
                    if (mode == kConquestModeMMSELS) fineRingStats_.fineRingSse += squared;
                }
            }
        }
        int best = 0;
        const int eligible = mode3Available ? kConquestModeCount : kConquestModeMMSELS;
        for (int mode = 1; mode < eligible; ++mode) if (error[mode] < error[best]) best = mode;
        modes.push_back(static_cast<std::uint8_t>(best));
        ++lastModeHist_[best];
        if (fineRingPrediction_) {
            fineRingStats_.rejectedClusters += !mode3Available;
            fineRingStats_.selectedGates += best == kConquestModeMMSELS ? members.size() : 0;
            fineRingStats_.selectedSse += error[best];
        }
        for (std::size_t local = 0; local < members.size(); ++local) {
            if (fineRingPrediction_) selectedPredictions[members[local]] = candidates[local][best];
            for (int axis = 0; axis < 3; ++axis) deltas[members[local]][axis] = targets[members[local]][axis] - candidates[local][best][axis];
        }
    }
    if (fineRingPrediction_) fineRingStats_.selectedPredictionSignature = conquestGeometryCellSignature(selectedPredictions);
    layerModes_ = modes;
}

// ---------------------------------------------------------------------------
// Decoder replay
// ---------------------------------------------------------------------------
void MMSECQuestGeomState::predictDecoder(const PatchView& p, double out[3]) const {
    int cells[3];
    predictDecoderCells(p, cells);
    for (int axis = 0; axis < 3; ++axis) out[axis] = toWorldCell(cells[axis], p.bboxMin[axis], p.quantStep);
}

void MMSECQuestGeomState::predictDecoderCells(const PatchView& p, int out[3]) const {
    out[0] = out[1] = out[2] = 0;
    const std::size_t idx = callIndex_++;
    int mode = kConquestModeBarycentric;
    if (replayModes_ != nullptr && !replayModes_->empty()) {
        std::size_t cluster = static_cast<std::size_t>(replayClusterSize_) > 0
                                  ? idx / static_cast<std::size_t>(replayClusterSize_)
                                  : 0;
        if (recursiveTopology_) {
            if (idx >= topologyPartition_.clusterForData.size()) throw std::runtime_error("topology predictor: insertion index out of range");
            cluster = topologyPartition_.clusterForData[idx];
            if (cluster >= replayModes_->size()) throw std::runtime_error("topology predictor: cluster index out of mode stream");
        } else if (cluster >= replayModes_->size()) cluster = replayModes_->size() - 1;
        mode = static_cast<int>((*replayModes_)[cluster]);
    }
    std::vector<std::uint32_t> ringIds(static_cast<std::size_t>(p.neighborCount));
    for (int i = 0; i < p.neighborCount; ++i) {
        ringIds[static_cast<std::size_t>(i)] = p.neighborIds ? p.neighborIds[i] : 0u;
    }
    int cand[3];
    computeModePrediction(mode, ringIds, p.centerId, insertedCenters_,
                          p.bboxMin, p.quantStep, cand);
    for (int k = 0; k < 3; ++k) out[k] = cand[k];
    if (fineRingPrediction_) {
        fineRingStats_.selectedGates += mode == kConquestModeMMSELS;
        for (auto cell : cand) foldConquestGeometryCell(fineRingStats_.selectedPredictionSignature, cell);
    }
    // This center is now part of the reconstruction; later gates in the same
    // layer must exclude it from their coarse-graph walks.
    insertedCenters_.insert(p.centerId);
}

}  // namespace mmse_pred
}  // namespace pmsf
