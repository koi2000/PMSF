// SPDX-License-Identifier: Apache-2.0
//
// RDCanonicalReconstructOperator.cpp — 见头文件。每轮一层，调 decodeLayer。
#include "RD/reconstruct/RDCanonicalReconstructOperator.h"

#include <cstdio>

namespace rd_canon {

void RDCanonicalReconstructOperator::reconstruct(MCGAL::Halfedge* /*halfedge*/) {
    // 单半边入口本 codec 不用（层由符号流界定，入口走向量版）。
}

QuantGrid RDCanonicalReconstructOperator::gridFromMesh() const {
    QuantGrid g;
    if (!mesh_) return g;
    g.bboxMin[0] = static_cast<double>(mesh_->bboxMin[0]);
    g.bboxMin[1] = static_cast<double>(mesh_->bboxMin[1]);
    g.bboxMin[2] = static_cast<double>(mesh_->bboxMin[2]);
    g.step = static_cast<double>(mesh_->f_quantStep);
    g.quantBits = static_cast<int>(mesh_->i_nbQuantBits);
    if (g.quantBits <= 0 || g.quantBits > 20) g.quantBits = 12;
    return g;
}

MCGAL::Halfedge* RDCanonicalReconstructOperator::locateSeed(
    const RDCanonicalSymbolReadOperator::LayerRecord& layer) {
    if (!layer.hasSeed || !mesh_) return nullptr;
    const QuantGrid grid = gridFromMesh();
    int q0[3] = {layer.seedQ[0], layer.seedQ[1], layer.seedQ[2]};  // tgt(seed)
    int q1[3] = {layer.seedQ[3], layer.seedQ[4], layer.seedQ[5]};  // tgt(opp(seed))
    MCGAL::Vertex* s0 = vertexByCell(mesh_.get(), grid, q0);
    MCGAL::Vertex* s1 = vertexByCell(mesh_.get(), grid, q1);
    if (s0 == nullptr || s1 == nullptr) {
        std::fprintf(stderr,
                     "[RD-CANON-DEC] seed vertices not found q0=(%d,%d,%d) "
                     "q1=(%d,%d,%d)\n",
                     q0[0], q0[1], q0[2], q1[0], q1[1], q1[2]);
        return nullptr;
    }
    // halfedgeBetween(s1, s0) 返回 tgt == s0 的那条 ⇒ 与 encoder 的 seed 同向。
    return halfedgeBetween(s1, s0);
}

void RDCanonicalReconstructOperator::reconstruct(
    std::vector<MCGAL::Vertex*>& /*vertices*/,
    std::vector<MCGAL::Halfedge*>& /*halfedge*/,
    std::vector<MCGAL::Facet*>& /*facets*/) {
    if (failed_ || !reader_ || !mesh_) {
        failed_ = true;
        return;
    }
    const auto* layer = reader_->takeNextLayer();
    const int round = roundIdx_++;
    if (layer == nullptr) return;  // 固定轮数解码的正常收尾
    if (layer->syms.empty()) {
        std::fprintf(stderr, "[RD-CANON-DEC] r=%d empty layer\n", round);
        return;
    }
    const QuantGrid grid = gridFromMesh();
    MCGAL::Halfedge* seed = locateSeed(*layer);
    if (seed == nullptr) {
        failed_ = true;
        std::fprintf(stderr, "[RD-CANON-DEC] r=%d FAIL seed-locate\n", round);
        return;
    }
    if (st_.vertexFlag.empty() && round == 0) {
        std::fprintf(stderr,
                     "[RD-CANON-DEC] grid step=%.17g bboxMin=(%.17g,%.17g,%.17g) "
                     "bits=%d\n",
                     grid.step, grid.bboxMin[0], grid.bboxMin[1], grid.bboxMin[2],
                     grid.quantBits);
    }
    const DecodeLayerResult r = decodeLayer(mesh_.get(), st_, seed, grid,
                                            layer->syms, layer->residuals);
    totalInserted_ += r.inserted;
    if (!r.ok) failed_ = true;
    std::fprintf(stderr,
                 "[RD-CANON-DEC] r=%d enc_layer=%d syms=%zu null=%d inserted=%d "
                 "residuals=%zu ok=%d\n",
                 round, reader_->layerCount() - 1 - round, layer->syms.size(),
                 r.nullPatches, r.inserted, layer->residuals.size(),
                 r.ok ? 1 : 0);
}

}  // namespace rd_canon
