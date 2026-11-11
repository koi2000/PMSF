// SPDX-License-Identifier: Apache-2.0
//
// MMSEConquestHooks.h — MMSE paper §2.2/§2.3 geometry prediction on the
// valence-conquest backbone (P3-B2).
//
// The paper (Lee et al., ICIP 2011) predicts each removed vertex with
//   mode 0: dual-ring eq.(3) with alpha = 0  == barycentric/umbrella prediction
//   mode 1: dual-ring eq.(3) with alpha = 0.5
//   mode 2: dual-ring eq.(3) with alpha = 1.0
//   mode 3: MMSE linear LS prediction (eq.(4)-(8), rho = 0.997)
// and picks, per cluster, the mode minimising the sum of squared prediction
// residuals (§2.3 eq.(9)).
//
// ---------------------------------------------------------------------------
// Symmetry contract (the non-obvious part)
// ---------------------------------------------------------------------------
// The predictor is evaluated on the **coarse mesh of the layer** (all of the
// layer's removed vertices gone), NOT on the intra-layer progressive state:
// a conquest layer removes an independent set, so each removed vertex j sees a
// coarse polygon ring, but the *secondary* ring of a ring vertex l in the
// encoder's intra-layer mesh contains vertices that will only be removed later,
// while the decoder's intra-layer mesh still/again contains earlier-inserted
// vertices.  Both sides therefore evaluate predictions on the coarse graph:
//   * encoder: after `decimateLayer` the engine's mesh IS the coarse mesh; the
//     H3 hook only returns the plain mean during the run (so the engine records
//     `delta_mean = cell(target) - cell(mean)`), records each gate's ring ids,
//     and `finalizeEncoderLayer()` re-computes all candidates on the coarse
//     mesh, rewrites residuals and emits the per-cluster mode ids.
//   * decoder: during `decodeLayer` insertion the hook excludes the current
//     center (`PatchView::centerId`) and every vertex inserted earlier in the
//     same layer (the hook records center ids itself) — the remaining graph is
//     exactly the coarse graph.
// All arithmetic is done in the **integer cell domain** (`cellOf` mirrors the
// default H4 quantizer exactly), so sums/means are order-independent and the
// two sides are bit-exact.  `toWorldCell()` maps an integer prediction back to
// the world so the engine's own `toCell()` recovers it (`toCell(toWorld(q))=q`).
//
// B1 (previous step) used only the barycentric hook; this file replaces it with
// the full paper predictor stack (B2).  Everything remains algorithm-parameter
// driven through env vars, and the canonical `algorithm=MMSE` path is untouched.

#ifndef PMSF_PREDICT_MMSE_CONQUEST_HOOKS_H
#define PMSF_PREDICT_MMSE_CONQUEST_HOOKS_H

#include "ContextPool.h"
#include "Global.h"
#include "Mesh.h"
#include "common/BoundedRingTraversal.h"  // pmsf::boundedRing
#include "scheme/EngineHooks.h"
#include "conquest/ConquestLayerTopology.h"
#include "segmentation/RecursiveTopologyBisection.h"
#include "predict/FineRingMMSE.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <unordered_set>
#include <vector>

namespace pmsf {
namespace mmse_pred {

// Paper §2.2/§2.3 mode ids (2-bit stream values).
enum : int {
    kConquestModeBarycentric = 0,
    kConquestModeDualRing05  = 1,
    kConquestModeDualRing1   = 2,
    kConquestModeMMSELS      = 3,
    kConquestModeCount       = 4,
};

/// PMSF_MMSE_CLUSTER: vertices per mode-decision cluster (paper §2.1's threshold
/// is not tabulated; BFS-order chunks are the minimal decoder-derivable version).
inline int conquestClusterSizeFromEnv(int fallback = 32) {
    const char* env = std::getenv("PMSF_MMSE_CLUSTER");
    if (env == nullptr || env[0] == '\0') return fallback;
    const int v = std::atoi(env);
    if (v < 1) return fallback;
    return v > 4096 ? 4096 : v;
}

/// PMSF_MMSE_LS=0 disables paper mode 3 (development knob; default on).
inline bool conquestLsEnabledFromEnv() {
    const char* env = std::getenv("PMSF_MMSE_LS");
    if (env == nullptr || env[0] == '\0') return true;
    return env[0] != '0';
}

// ---------------------------------------------------------------------------
// Default H4 quantizer formulas, replicated exactly (mmse_cq leaves H4 default).
// ---------------------------------------------------------------------------
inline int cellOf(double c, double min, double step) {
    int q = static_cast<int>(std::ceil((c - min) / step)) - 1;
    if (q == -1) q = 0;
    return q;
}

inline double toWorldCell(int q, double min, double step) {
    return min + (static_cast<double>(q) + 0.5) * step;
}

inline double roundHalfAway(double v) {
    return (v >= 0.0) ? std::floor(v + 0.5) : std::ceil(v - 0.5);
}

// ---------------------------------------------------------------------------
// Geometry state (shared by the collector and the reconstructor).
// ---------------------------------------------------------------------------
class MMSECQuestGeomState final : public GeometryPredictor {
  public:
    // --- configuration -----------------------------------------------------
    void setRecursiveTopology(bool enabled, int threshold = 32, std::uint64_t seed = 1) {
        recursiveTopology_ = enabled;
        topologyThreshold_ = std::max(1, threshold);
        topologySeed_ = seed;
    }
    void setFineRingPrediction(bool enabled) { fineRingPrediction_ = enabled; }
    struct FineRingStats {
        std::size_t evaluated = 0, unavailable = 0, largeSupport = 0, maxSupport = 0, distance4Pairs = 0;
        std::size_t selectedGates = 0, rejectedClusters = 0;
        double minAbsPivot = std::numeric_limits<double>::infinity();
        double maxAbsWeight = 0., maxWeightL1 = 0., minWeightSum = std::numeric_limits<double>::infinity(),
               maxWeightSum = -std::numeric_limits<double>::infinity();
        double barycentricSseOnSolvable = 0., fineRingSse = 0., selectedSse = 0.;
        std::uint64_t selectedPredictionSignature = 1469598103934665603ull;
    };
    const FineRingStats& fineRingStats() const { return fineRingStats_; }

    void snapshotEncoderTopology(MCGAL::Halfedge* seed) {
        if (recursiveTopology_) {
            fineTopology_ = snapshotConquestLayerTopology(seed);
            std::size_t alive = 0;
            for (auto* vertex : mesh_->vertices()) if (vertex && !vertex->isRemoved()) ++alive;
            if (alive != fineTopology_.vertices.size()) {
                throw std::runtime_error("topology predictor: rooted component does not cover all live vertices");
            }
        }
    }
    void prepareDecoderTopology(MCGAL::Halfedge* seed, const std::vector<std::uint32_t>& centers);
    const TopologyBisectionResult& lastTopologyPartition() const { return topologyPartition_; }
    std::uint64_t lastTopologySignature() const { return fineTopology_.signature; }
    std::uint64_t lastTargetCellSignature() const { return lastTargetCellSignature_; }
    std::size_t lastTopologyVertices() const { return fineTopology_.vertices.size(); }
    std::size_t lastTopologyEdges() const {
        std::size_t count = 0;
        for (const auto& neighbors : fineTopology_.adjacency) count += neighbors.size();
        return count / 2;
    }
    void setClusterSize(int n) {
        clusterSize_ = std::max(1, n);
    }
    int clusterSize() const {
        return clusterSize_;
    }
    void setLsEnabled(bool on) {
        lsEnabled_ = on;
    }

    /// @brief Encoder side: geometry of the run + the layer record workspace.
    void setEncoderMesh(MCGAL::Mesh* mesh, int meshId) {
        mesh_ = mesh;
        meshId_ = meshId;
        encoder_ = true;
        records_.clear();
        pendingDelta_ = 0;
    }

    /// @brief Decoder side: current reconstruction mesh + per-layer replay data.
    void setDecoderMesh(MCGAL::Mesh* mesh, int meshId) {
        mesh_ = mesh;
        meshId_ = meshId;
        encoder_ = false;
    }

    void beginEncoderLayer() {
        records_.clear();
        pendingDelta_ = 0;
        layerModes_.clear();
        topologyPartition_ = {};
        fineRingStats_ = {};
        lastTargetCellSignature_ = 1469598103934665603ull;
    }

    /// @brief Decoder: install one layer's mode stream before `decodeLayer`.
    void beginDecoderLayer(const std::vector<std::uint8_t>* modes, int clusterSize) {
        replayModes_ = modes;
        replayClusterSize_ = std::max(1, clusterSize);
        callIndex_ = 0;
        insertedCenters_.clear();
        fineRingStats_ = {};
        // The layer-start mesh IS the coarse mesh of this layer; snapshot its
        // graph so predictions match the encoder's post-layer pass exactly.
        buildCoarseAdjacency();
    }

    // --- encoder API -------------------------------------------------------
    /// The engine calls this once per removed/inserted vertex, right before it
    /// computes/uses the residual.
    ///   encoder: return the plain 1-ring mean (same as the engine default) and
    ///            record the gate for the post-layer predictor pass;
    ///   decoder: return the world prediction of the transmitted mode.
    void predict(const PatchView& p, double out[3]) const override {
        if (!encoder_) {
            predictDecoder(p, out);
            return;
        }
        const double mean[3] = {meanComponent(p, 0), meanComponent(p, 1), meanComponent(p, 2)};
        GateRec rec;
        rec.centerId = p.centerId;
        rec.ringIds.resize(static_cast<std::size_t>(p.neighborCount));
        for (int i = 0; i < p.neighborCount; ++i) {
            rec.ringIds[static_cast<std::size_t>(i)] = p.neighborIds ? p.neighborIds[i] : 0u;
        }
        rec.meanWorld[0] = mean[0];
        rec.meanWorld[1] = mean[1];
        rec.meanWorld[2] = mean[2];
        for (int k = 0; k < 3; ++k) {
            rec.cellMean[k] = cellOf(mean[k], p.bboxMin[k], p.quantStep);
        }
        rec.bboxMin[0] = p.bboxMin[0];
        rec.bboxMin[1] = p.bboxMin[1];
        rec.bboxMin[2] = p.bboxMin[2];
        rec.step = p.quantStep;
        records_.push_back(std::move(rec));
        // Index of the record whose engine residual is about to arrive.
        pendingDelta_ = records_.size() - 1;
        out[0] = mean[0];
        out[1] = mean[1];
        out[2] = mean[2];
    }

    /// @brief Encoder sink callback: the engine's quantized residual for the
    /// most recently predicted gate (`delta = cell(target) - cell(mean)`).
    void onEncoderDelta(const int delta[3]) {
        if (records_.empty() || pendingDelta_ >= records_.size()) return;
        GateRec& rec = records_[pendingDelta_++];
        for (int k = 0; k < 3; ++k) rec.delta[k] = delta[k];
    }

    /// @brief Post-layer pass: recompute every residual with the paper
    /// predictor stack, decide one mode per cluster, rewrite `deltas`.
    void finalizeEncoderLayer(std::vector<std::array<int, 3>>& deltas,
                              std::vector<std::uint8_t>& modes);

    // --- decoder API -------------------------------------------------------
    /// Decoder: prediction for the current insertion, in world coordinates.
    void predictDecoder(const PatchView& p, double out[3]) const;
    void predictDecoderCells(const PatchView& p, int out[3]) const;
    bool predictQuantized(const PatchView& p, int out[3]) const override {
        if (encoder_ || !recursiveTopology_) return false;
        predictDecoderCells(p, out);
        return true;
    }


    int lastModeHist(int mode) const {
        return (mode >= 0 && mode < kConquestModeCount) ? lastModeHist_[mode] : 0;
    }

  private:
    struct GateRec {
        std::uint32_t centerId = 0;
        std::vector<std::uint32_t> ringIds;
        double meanWorld[3] = {0, 0, 0};
        int cellMean[3] = {0, 0, 0};
        int delta[3] = {0, 0, 0};
        double bboxMin[3] = {0, 0, 0};
        double step = 0.0;
    };

    static double meanComponent(const PatchView& p, int k) {
        double s = 0.0;
        for (int i = 0; i < p.neighborCount; ++i) s += p.neighborPos[3 * i + k];
        return (p.neighborCount > 0) ? s / static_cast<double>(p.neighborCount) : 0.0;
    }

    MCGAL::Vertex* vertexFromPoolId(std::uint32_t id) const {
        return MCGAL::contextPool.getVertexByIndexInSubPool(
            meshId_, static_cast<int>(id));
    }

    /// @brief Snapshot the **coarse graph adjacency** of the current mesh.
    ///
    /// Built from the active **faces** (each face loop contributes its edges),
    /// never from `Vertex::proxyHalfedge()`: face loops are the one connectivity
    /// view that is identical on the encoder's post-layer coarse mesh and on the
    /// decoder's layer-start mesh.  Proxy/halfedge walks can stop early on
    /// halfedges whose faces were merged by decimation/insertion, which made the
    /// encoder and decoder predict different secondary rings (measured: decoded
    /// topology/geometry FAIL).
    void buildCoarseAdjacency() const;
    const std::vector<std::uint32_t>* coarseAdj(std::uint32_t id) const {
        auto it = adj_.find(id);
        return (it == adj_.end()) ? nullptr : &it->second;
    }
    mutable std::unordered_map<std::uint32_t, std::vector<std::uint32_t>> adj_;

    struct GateCtx {
        std::vector<MCGAL::Vertex*> ringVerts;
        std::vector<MCGAL::PointInt> ringCells;
        std::unordered_set<std::uint32_t> excluded;
    };
    bool prepareGate(const std::vector<std::uint32_t>& ringIds,
                     std::uint32_t centerId,
                     const std::unordered_set<std::uint32_t>& excludedExtra,
                     const double bboxMin[3], double step,
                     GateCtx& ctx) const;
    void dualRingCells(const GateCtx& ctx, const double bboxMin[3], double step,
                       int out05[3], int out1[3]) const;
    void lsCells(const GateCtx& ctx, const double bboxMin[3], double step,
                 int out[3]) const;

    /// @brief All four candidate predictions (cell domain) for one gate on the
    /// coarse graph.  `ringIds` are the gate polygon ring vertices.
    bool computeCandidates(
        const std::vector<std::uint32_t>& ringIds,
        std::uint32_t centerId,
        const std::unordered_set<std::uint32_t>& excludedExtra,
        const double bboxMin[3], double step,
        std::array<std::array<int, 3>, kConquestModeCount>& outCell) const;
    /// @brief Decoder fast path: only the transmitted mode's prediction.
    void computeModePrediction(int mode,
                               const std::vector<std::uint32_t>& ringIds,
                               std::uint32_t centerId,
                               const std::unordered_set<std::uint32_t>& excludedExtra,
                               const double bboxMin[3], double step,
                               int out[3]) const;

    void markFineRingPositions(const std::vector<std::uint32_t>& centers);
    bool fineRingLsCells(std::uint32_t centerId, const double bboxMin[3], double step, int out[3]) const;

    TopologyBisectionResult partitionTopology(const std::vector<std::uint32_t>& centers) const;
    void finalizeRecursiveEncoderLayer(std::vector<std::array<int, 3>>& deltas,
                                       std::vector<std::uint8_t>& modes);

    void computeLs(const std::vector<std::uint32_t>& ringIds,
                   const std::unordered_set<std::uint32_t>& excludedExtra,
                   double bboxMin[3], double step,
                   int out[3]) const;

    MCGAL::Mesh* mesh_ = nullptr;
    int meshId_ = 0;
    mutable bool encoder_ = true;
    mutable std::vector<GateRec> records_;
    mutable std::size_t pendingDelta_ = 0;

    // decoder replay state (mutated through the const hook)
    mutable const std::vector<std::uint8_t>* replayModes_ = nullptr;
    mutable int replayClusterSize_ = 32;
    mutable std::size_t callIndex_ = 0;
    mutable std::unordered_set<std::uint32_t> insertedCenters_;

    // encoder layer workspace / diagnostics
    mutable std::vector<std::uint8_t> layerModes_;
    mutable int lastModeHist_[kConquestModeCount] = {0, 0, 0, 0};
    bool recursiveTopology_ = false;
    bool fineRingPrediction_ = false;
    std::vector<bool> fineKnown_;
    mutable FineRingWorkspace fineRingWorkspace_;
    mutable FineRingStats fineRingStats_;
    int topologyThreshold_ = 32;
    std::uint64_t topologySeed_ = 1;
    ConquestLayerTopology fineTopology_;
    TopologyBisectionResult topologyPartition_;
    std::uint64_t lastTargetCellSignature_ = 1469598103934665603ull;
    int clusterSize_ = 32;
    bool lsEnabled_ = true;
};

}  // namespace mmse_pred
}  // namespace pmsf

#endif  // PMSF_PREDICT_MMSE_CONQUEST_HOOKS_H
