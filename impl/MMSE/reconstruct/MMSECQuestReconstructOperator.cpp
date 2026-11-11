// SPDX-License-Identifier: Apache-2.0
//
// MMSECQuestReconstructOperator.cpp — orchestration only (see header).
// The decode entity stays `pmsf::decodeLayer` (the verified conquest mirror);
// this class only recovers the layer seed, installs the MMSE mode stream and
// forwards the same EngineHooks instance used by the encoder.

#include "MMSE/reconstruct/MMSECQuestReconstructOperator.h"

#include "MMSE/decompress/MMSECQuestSymbolReadOperator.h"
#include "conquest/ConquestMeshPrimitives.h"

#include <cstdio>
#include <cstdlib>
#include <iostream>

namespace pmsf {

MMSECQuestReconstructOperator::MMSECQuestReconstructOperator(
    std::shared_ptr<MMSECQuestSymbolReadOperator> reader,
    std::shared_ptr<mmse_pred::MMSECQuestGeomState> state)
    : reader_(std::move(reader)), state_(std::move(state)) {
    hooks_.geometryPredictor = state_;
}

void MMSECQuestReconstructOperator::reconstruct(MCGAL::Halfedge* /*halfedge*/) {
    // Single-halfedge entry is unused: layers are delimited by the symbol stream.
}

void MMSECQuestReconstructOperator::reconstruct(
    std::vector<MCGAL::Vertex*>& /*vertices*/,
    std::vector<MCGAL::Halfedge*>& /*halfedge*/,
    std::vector<MCGAL::Facet*>& /*facets*/) {
    if (failed_ || !reader_ || !mesh_ || !state_) {
        failed_ = true;
        return;
    }
    // `ReconstructOperator::init()` is non-virtual, so the framework's init()
    // call only sets the base mesh_ member.  Bind the geometry hook to the
    // decoder mesh lazily on the first reconstruct() call.
    if (!decoderMeshBound_) {
        state_->setDecoderMesh(mesh_.get(), mesh_->meshId());
        decoderMeshBound_ = true;
    }
    const conquest_blob::LayerRecord* layer = reader_->takeNextLayer();
    if (!layer) {
        // Layer count exhausted: normal tail for fixed-round decoding.
        return;
    }
    const std::size_t layerIdx = reader_->layerCursor() - 1;

    // Seed recovered by coordinates (pool ids are not comparable across sides).
    MCGAL::Vertex* s0 = nearestVertex(mesh_.get(), layer->seedPos[0]);
    MCGAL::Vertex* s1 = nearestVertex(mesh_.get(), layer->seedPos[1]);
    MCGAL::Halfedge* seed = halfedgeBetween(s1, s0);
    if (!seed) {
        std::cerr << "[mmse_cq reconstruct] seed halfedge not found (layer " << layerIdx << ")\n";
        failed_ = true;
        return;
    }

    const conquest_blob::Stream& s = reader_->stream();
    FrontierConquestEngine::QuantGrid grid;
    grid.bboxMin[0] = s.bboxMin[0];
    grid.bboxMin[1] = s.bboxMin[1];
    grid.bboxMin[2] = s.bboxMin[2];
    grid.step = s.step;

    // The recursive protocol reconstructs all connectivity before partitioning
    // and recovering geometry. Coarse predictor adjacency is snapshotted first.
    state_->setRecursiveTopology(reader_->usesRecursiveTopology(), reader_->topologyThreshold(), reader_->topologySeed());
    state_->setFineRingPrediction(reader_->usesFineRingPrediction());
    state_->beginDecoderLayer(reader_->modesForLayer(layerIdx),
                              reader_->clusterSizeForLayer(layerIdx));

    if (reader_->usesRecursiveTopology()) {
        std::vector<ConquestDeferredGeometry> geometry;
        last_ = decodeLayer(mesh_.get(), st_, seed, layer->symbols, layer->deltas, grid,
                            hooks_, nullptr, &geometry);
        if (!last_.ok || geometry.size() != layer->deltas.size()) {
            throw std::runtime_error("MMSE topology-only reconstruction failed or omitted geometry records");
        }
        std::vector<std::uint32_t> centers;
        for (const auto& record : geometry) centers.push_back(static_cast<std::uint32_t>(record.center->poolId()));
        state_->prepareDecoderTopology(seed, centers);
        const auto& partition = state_->lastTopologyPartition();
        const auto restored = restoreDeferredConquestGeometry(geometry, grid, hooks_);
        std::cout << "MMSE_TOPOLOGY decoder layer=" << layerIdx
                  << " vertices=" << state_->lastTopologyVertices() << " edges=" << state_->lastTopologyEdges()
                  << " gates=" << geometry.size() << " clusters=" << partition.clusters.size()
                  << " graph=" << state_->lastTopologySignature() << " membership=" << partition.signature
                  << " target_cells=" << restored.cellSignature << " negative_predictions=" << restored.negativePredictions
                  << " closed_fallbacks=" << partition.closedBoundaryFallbacks << "\n";
        if (reader_->usesFineRingPrediction()) {
            const auto& stats = state_->fineRingStats();
            std::cout << "MMSE_FINE_RING decoder layer=" << layerIdx
                      << " evaluated=" << stats.evaluated << " unavailable=" << stats.unavailable
                      << " selected_gates=" << stats.selectedGates
                      << " prediction_signature=" << stats.selectedPredictionSignature << "\n";
        }
    } else {
    last_ = decodeLayer(mesh_.get(), st_, seed, layer->symbols, layer->deltas, grid,
                        hooks_, nullptr);
    }
    totalInserted_ += last_.inserted;
    if (!last_.ok) failed_ = true;
    if (!loggedOnce_) {
        loggedOnce_ = true;
    }
}

}  // namespace pmsf
