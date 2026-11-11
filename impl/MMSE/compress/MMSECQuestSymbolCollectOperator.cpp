// SPDX-License-Identifier: Apache-2.0
//
// MMSECQuestSymbolCollectOperator.cpp — see the header for the blob layout.
//
// The traversal / connectivity coding is the framework's verified valence-conquest
// engine (paper [2]); this file is the MMSE wrapper that
//   1. runs one engine layer per framework round,
//   2. records the engine's mean-prediction residual (`delta = cell(target) -
//      cell(mean)`) through a GeometrySink,
//   3. after the layer, has the MMSE hooks recompute the residual with the
//      paper §2.2 predictor stack on the coarse mesh and decide one mode per
//      cluster (paper §2.3),
//   4. writes [u32 trailerLen][MMT1 trailer][CQST v2] with layers reversed
//      (encoding order is fine->coarse, decoding must replay coarse->fine).

#include "MMSE/compress/MMSECQuestSymbolCollectOperator.h"
#include "MMSE/MMSELayerSymbols.h"

#include "Configuration.h"
#include "conquest/ConquestBlobFormat.h"
#include "conquest/FrontierConquestEngine.h"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <cstring>
#include <utility>
#include <iostream>
#include <zlib.h>

namespace pmsf {

namespace {

/// Collects the engine's quantized residual per removed vertex and forwards it
/// to the MMSE geometry state (which pairs it with the pending gate record).
class MMSECQuestDeltaSink final : public pmsf::GeometrySink {
  public:
    MMSECQuestDeltaSink(std::vector<std::array<int, 3>>* out,
                        mmse_pred::MMSECQuestGeomState* state)
        : out_(out), state_(state) {}
    void onRemovedVertex(const int cellDelta[3]) override {
        // Keep the layer's delta vector sized/aligned; the post-layer MMSE pass
        // rewrites the values in place (residual for the chosen mode).
        if (out_ != nullptr) {
            out_->push_back({cellDelta[0], cellDelta[1], cellDelta[2]});
        }
        if (state_ != nullptr) state_->onEncoderDelta(cellDelta);
    }

  private:
    std::vector<std::array<int, 3>>* out_;
    mmse_pred::MMSECQuestGeomState* state_;
};

void putU32(std::vector<std::uint8_t>& out, std::uint32_t v) {
    out.push_back(static_cast<std::uint8_t>(v & 0xffu));
    out.push_back(static_cast<std::uint8_t>((v >> 8) & 0xffu));
    out.push_back(static_cast<std::uint8_t>((v >> 16) & 0xffu));
    out.push_back(static_cast<std::uint8_t>((v >> 24) & 0xffu));
}

}  // namespace

std::size_t MMSECQuestSymbolCollectOperator::totalModes() const {
    std::size_t n = 0;
    for (const auto& v : layerModes_) n += v.size();
    return n;
}

void MMSECQuestSymbolCollectOperator::init(std::shared_ptr<MCGAL::Mesh> mesh,
                                           std::vector<MCGAL::Halfedge*> seeds,
                                           PMSFHeader header) {
    mesh_ = mesh;
    SymbolCollectOperator::init(mesh_, seeds, header);
    layerModes_.clear();
    computeGridFromMesh();
}

// Isotropic single step = bbox longest edge / 2^bits.  When the framework
// quantizer is present we *用框架那一把尺子* exactly like the conquest codec —
// the container stores the base mesh with it, so both sides must share it.
void MMSECQuestSymbolCollectOperator::computeGridFromMesh() {
    stream_.layers.clear();
    if (gridForced_ || !mesh_) return;
    double lo[3] = {0, 0, 0}, hi[3] = {0, 0, 0};
    bool first = true;
    for (MCGAL::Vertex* v : mesh_->vertices()) {
        if (!v || v->isRemoved()) continue;
        const double p[3] = {v->x(), v->y(), v->z()};
        for (int k = 0; k < 3; ++k) {
            if (first) {
                lo[k] = hi[k] = p[k];
            } else {
                lo[k] = std::min(lo[k], p[k]);
                hi[k] = std::max(hi[k], p[k]);
            }
        }
        first = false;
    }
    if (first) {
        grid_ = pmsf::FrontierConquestEngine::QuantGrid{};
        return;
    }
    if (mesh_->enableQuantization && mesh_->f_quantStep > 0.0f) {
        grid_.bboxMin[0] = (double)mesh_->bboxMin[0];
        grid_.bboxMin[1] = (double)mesh_->bboxMin[1];
        grid_.bboxMin[2] = (double)mesh_->bboxMin[2];
        grid_.step = (double)mesh_->f_quantStep;
        quantBits_ = mesh_->i_nbQuantBits;
    } else {
        double extent = 0.0;
        for (int k = 0; k < 3; ++k) extent = std::max(extent, hi[k] - lo[k]);
        for (int k = 0; k < 3; ++k) grid_.bboxMin[k] = lo[k];
        grid_.step = extent / (double)(1u << quantBits_);
    }
    stream_.bboxMin[0] = grid_.bboxMin[0];
    stream_.bboxMin[1] = grid_.bboxMin[1];
    stream_.bboxMin[2] = grid_.bboxMin[2];
    stream_.step = grid_.step;
    snapMeshToCells();
}

// Snap the working mesh to the same world the decoder gets back: the container
// stores the base mesh as quantized cells and the decoder reconstructs
// `bboxMin + (q + 0.5) * step` in **float** arithmetic (see
// ColumnarDeserializeOperator).  Mirroring this formula bit-for-bit is what
// makes the encoder's neighbour coordinates identical to the decoder's.
void MMSECQuestSymbolCollectOperator::snapMeshToCells() {
    if (!mesh_ || !(grid_.step > 0.0)) return;
    for (MCGAL::Vertex* v : mesh_->vertices()) {
        if (!v || v->isRemoved()) continue;
        MCGAL::PointInt q = mesh_->floatPosToInt(v->point());
        const int bits = (int)mesh_->i_nbQuantBits;
        if (bits > 0 && bits < 31) {
            const int32_t maxV = (1 << bits) - 1;
            for (int k = 0; k < 3; ++k) {
                if (q[k] < 0) q[k] = 0;
                if (q[k] > maxV) q[k] = maxV;
            }
        }
        double w[3];
        for (int k = 0; k < 3; ++k) {
            const float bm = (float)grid_.bboxMin[k];
            const float st = (float)grid_.step;
            w[k] = (double)(bm + ((float)q[k] + 0.5f) * st);
        }
        v->setPoint(MCGAL::Point(w[0], w[1], w[2]));
    }
}

void MMSECQuestSymbolCollectOperator::collect(std::vector<MCGAL::Halfedge*> seeds) {
    if (!seeds.empty()) collect(seeds[0]);
}

void MMSECQuestSymbolCollectOperator::collect(MCGAL::Halfedge* seed) {
    if (!seed || !mesh_) return;

    if (!geom_) geom_ = std::make_shared<mmse_pred::MMSECQuestGeomState>();
    geom_->setRecursiveTopology(recursiveTopology_, topologyThreshold_, topologySeed_);
    geom_->setFineRingPrediction(fineRingPrediction_);
    geom_->setClusterSize(mmse_pred::conquestClusterSizeFromEnv(32));
    // The recursive wire protocol always evaluates the four paper modes;
    // decoder reconstruction must not depend on a process-level LS toggle.
    geom_->setLsEnabled(recursiveTopology_ ? true : mmse_pred::conquestLsEnabledFromEnv());
    geom_->setEncoderMesh(mesh_.get(), mesh_->meshId());
    geom_->beginEncoderLayer();
    geom_->snapshotEncoderTopology(seed);

    auto hooks = std::make_shared<pmsf::EngineHooks>();
    hooks->geometryPredictor = geom_;

    pmsf::FrontierConquestEngine engine(hooks);
    engine.setQuantGrid(grid_);

    conquest_blob::LayerRecord layer;
    // Seed is recorded by coordinates: pool ids are not comparable across sides.
    if (MCGAL::Vertex* a = seed->end_vertex()) {
        layer.seedPos[0][0] = a->x();
        layer.seedPos[0][1] = a->y();
        layer.seedPos[0][2] = a->z();
    }
    if (MCGAL::Halfedge* o = seed->opposite()) {
        if (MCGAL::Vertex* b = o->end_vertex()) {
            layer.seedPos[1][0] = b->x();
            layer.seedPos[1][1] = b->y();
            layer.seedPos[1][2] = b->z();
        }
    }

    pmsf::ConquestState st;
    MMSECQuestDeltaSink sink(&layer.deltas, geom_.get());
    engine.decimateLayer(mesh_.get(), st, seed, &layer.symbols, &sink);

    // Post-layer MMSE pass: recompute residuals with the paper predictor stack
    // on the now-coarse mesh and decide one mode per cluster.
    std::vector<std::uint8_t> modes;
    geom_->finalizeEncoderLayer(layer.deltas, modes);
    if (recursiveTopology_) {
        const auto& partition = geom_->lastTopologyPartition();
        std::cout << "MMSE_TOPOLOGY encoder layer=" << stream_.layers.size()
                  << " vertices=" << geom_->lastTopologyVertices() << " edges=" << geom_->lastTopologyEdges()
                  << " gates=" << layer.deltas.size() << " clusters=" << partition.clusters.size()
                  << " graph=" << geom_->lastTopologySignature() << " membership=" << partition.signature
                  << " target_cells=" << geom_->lastTargetCellSignature()
                  << " closed_fallbacks=" << partition.closedBoundaryFallbacks << "\n";
    }
    if (fineRingPrediction_) {
        const auto& stats = geom_->fineRingStats();
        std::cout << "MMSE_FINE_RING encoder layer=" << stream_.layers.size()
                  << " evaluated=" << stats.evaluated << " unavailable=" << stats.unavailable
                  << " large_support=" << stats.largeSupport << " max_support=" << stats.maxSupport
                  << " distance4_pairs=" << stats.distance4Pairs << " min_abs_pivot=" << stats.minAbsPivot
                  << " max_abs_weight=" << stats.maxAbsWeight << " max_weight_l1=" << stats.maxWeightL1
                  << " min_weight_sum=" << stats.minWeightSum << " max_weight_sum=" << stats.maxWeightSum
                  << " barycentric_sse_on_solvable=" << stats.barycentricSseOnSolvable
                  << " fine_ring_sse=" << stats.fineRingSse << " selected_sse=" << stats.selectedSse
                  << " rejected_clusters=" << stats.rejectedClusters << " selected_gates=" << stats.selectedGates
                  << " mode0=" << geom_->lastModeHist(0) << " mode1=" << geom_->lastModeHist(1)
                  << " mode2=" << geom_->lastModeHist(2) << " mode3=" << geom_->lastModeHist(3)
                  << " prediction_signature=" << stats.selectedPredictionSignature << "\n";
    }
    layerModes_.push_back(std::move(modes));

    if (!gridForced_) stream_.step = grid_.step;
    stream_.layers.push_back(std::move(layer));
}

int MMSECQuestSymbolCollectOperator::exportToBufferWithRangeCoder(
    char* buffer, bool /*enableQuantization*/) {
    if (buffer == nullptr) return -1;

    // Layer order must be reversed: this collector runs fine -> coarse, the
    // decoder replays coarse -> fine (same rule as the conquest codec).
    conquest_blob::Stream out;
    out.step = stream_.step;
    out.bboxMin[0] = stream_.bboxMin[0];
    out.bboxMin[1] = stream_.bboxMin[1];
    out.bboxMin[2] = stream_.bboxMin[2];
    out.layers.assign(stream_.layers.rbegin(), stream_.layers.rend());
    out.signedBitplanes = paperEntropy_;
    out.bitplaneContext = bitplaneContext_;

    // Trailer (per reversed layer): gateCount, clusterSize, packed 2-bit modes.
    std::vector<std::uint8_t> trailer;
    trailer.push_back('M');
    trailer.push_back('M');
    trailer.push_back('T');
    const unsigned version = fineRingPrediction_ ? 4u : (recursiveTopology_ ? 3u : (paperEntropy_ ? 2u : 1u));
    trailer.push_back(static_cast<std::uint8_t>('0'+version));
    putU32(trailer, version);
    putU32(trailer, static_cast<std::uint32_t>(out.layers.size()));
    if (recursiveTopology_) {
        putU32(trailer, static_cast<std::uint32_t>(topologyThreshold_));
        putU32(trailer, static_cast<std::uint32_t>(topologySeed_));
        putU32(trailer, static_cast<std::uint32_t>(topologySeed_ >> 32));
    }
    const int clusterSize = geom_ ? geom_->clusterSize() : 32;
    for (std::size_t li = 0; li < out.layers.size(); ++li) {
        // layerModes_ is in encoder order; out.layers is reversed.
        const std::size_t srcIdx = out.layers.size() - 1 - li;
        const std::vector<std::uint8_t> empty;
        const std::vector<std::uint8_t>& modes =
            (srcIdx < layerModes_.size()) ? layerModes_[srcIdx] : empty;
        putU32(trailer, static_cast<std::uint32_t>(out.layers[li].deltas.size()));
        putU32(trailer, static_cast<std::uint32_t>(recursiveTopology_ ? modes.size() : clusterSize));
        if (paperEntropy_) {
            const auto arithmetic = mmse_symbols::encodeModes(modes);
            putU32(trailer, static_cast<std::uint32_t>(arithmetic.size()));
            trailer.insert(trailer.end(), arithmetic.begin(), arithmetic.end());
            continue;
        }
        const std::size_t modeBytes = (modes.size() + 3) / 4;
        putU32(trailer, static_cast<std::uint32_t>(modeBytes));
        const std::size_t base = trailer.size();
        trailer.resize(base + modeBytes, 0);
        for (std::size_t c = 0; c < modes.size(); ++c) {
            trailer[base + (c >> 2)] |= static_cast<std::uint8_t>(
                (modes[c] & 0x3u) << (2 * static_cast<unsigned>(c & 3)));
        }
    }

    const std::size_t headerBytes = 4 + trailer.size();
    if (headerBytes + 64 >= (std::size_t)BUFFER_SIZE) return -1;
    // [u32 trailerLen][trailer][CQST ...]
    const std::uint32_t trailerLen = static_cast<std::uint32_t>(trailer.size());
    std::memcpy(buffer, &trailerLen, 4);
    std::memcpy(buffer + 4, trailer.data(), trailer.size());

    const int cqstBytes = conquest_blob::writeStream(
        out, buffer + headerBytes, (std::size_t)BUFFER_SIZE - headerBytes);
    if (cqstBytes < 0) return -1;
    return static_cast<int>(headerBytes) + cqstBytes;
}

int MMSECQuestSymbolCollectOperator::exportToBuffer(char* /*buffer*/,
                                                    bool /*enableQuantization*/,
                                                    bool /*enableCompress*/) {
    return 0;
}

const pmsf::PMSFSymbolTable* MMSECQuestSymbolCollectOperator::pmsfSymbolTable() const {
    paperTable_.clear();
    if (paperRelations_) {
        if (!recursiveTopology_ || !paperEntropy_ || layerModes_.size()!=stream_.layers.size())
            throw std::runtime_error("MMSE paper relations require complete recursive-topology layers");
        for (std::size_t index=0;index<stream_.layers.size();++index) {
            mmse_symbols::LayerMetadata info;
            info.encoderRound=static_cast<std::uint32_t>(index);
            info.rounds=static_cast<std::uint32_t>(stream_.layers.size());
            info.threshold=static_cast<std::uint32_t>(topologyThreshold_);
            info.seed=topologySeed_;info.context=bitplaneContext_;info.fineRingPrediction=fineRingPrediction_;
            paperTable_.push_back(mmse_symbols::encodeLayer(stream_,layerModes_[index],info));
        }
    } else {
        pmsf::appendPMSFSymbolRecord(paperTable_, 0, 0, 0);
    }
    return &paperTable_;
}

}  // namespace pmsf
