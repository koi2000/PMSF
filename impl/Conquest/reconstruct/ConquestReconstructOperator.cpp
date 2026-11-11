/*
 * @file ConquestReconstructOperator.cpp
 * @brief `ConquestReconstructOperator.h` 的实现 —— 只做编排，不做解码。
 */
#include "Conquest/reconstruct/ConquestReconstructOperator.h"

#include "Conquest/decompress/ConquestSymbolReadOperator.h"
#include "conquest/ConquestMeshPrimitives.h"

#include <cmath>
#include <cstdio>

namespace pmsf {

ConquestReconstructOperator::ConquestReconstructOperator(std::shared_ptr<ConquestSymbolReadOperator> reader)
    : reader_(std::move(reader)) {}

void ConquestReconstructOperator::reconstruct(MCGAL::Halfedge* /*halfedge*/) {
    // 单半边入口本 codec 不用（层由符号流界定，入口走下面的向量版）。
}

void ConquestReconstructOperator::reconstruct(std::vector<MCGAL::Vertex*>& /*vertices*/,
                                              std::vector<MCGAL::Halfedge*>& /*halfedge*/,
                                              std::vector<MCGAL::Facet*>& /*facets*/) {
    if (failed_ || !reader_ || !mesh_) {
        failed_ = true;
        return;
    }
    const conquest_blob::LayerRecord* layer = reader_->takeNextLayer();
    if (!layer) {
        // 层数用尽：本层的符号流不存在。对**固定轮数**的解压这是正常收尾
        // （轮数由 header 给出，可能多于实际写出的层数）。
        return;
    }

    // ---- 种子：按坐标找回（池号在两端不可比）----------------------------
    // ⚠️ 次序：`halfedgeBetween(a, b)` 返回的是 **tgt 为 b** 的那条。
    // 记录里 `seedPos[0] = tgt(seed)`、`seedPos[1] = tgt(opp(seed))`，
    // 所以要还原 seed 必须传 `(seedPos[1] 的顶点, seedPos[0] 的顶点)`。
    MCGAL::Vertex* s0 = nearestVertex(mesh_.get(), layer->seedPos[0]);
    MCGAL::Vertex* s1 = nearestVertex(mesh_.get(), layer->seedPos[1]);
    MCGAL::Halfedge* seed = halfedgeBetween(s1, s0);
    if (!seed) {
        std::cerr << "[Conquest reconstruct] **找不到本层种子半边**（第 " << reader_->layerCursor() - 1 << " 层）\n";
        failed_ = true;
        return;
    }

    const conquest_blob::Stream& s = reader_->stream();
    FrontierConquestEngine::QuantGrid grid;
    grid.bboxMin[0] = s.bboxMin[0];
    grid.bboxMin[1] = s.bboxMin[1];
    grid.bboxMin[2] = s.bboxMin[2];
    grid.step = s.step;

    if (!loggedOnce_ && std::getenv("PMSF_CONQUEST_BASEQC")) {
        // 自洽判据（不需要与编码端配对）：base mesh 若真是"量化号写出去、读回来
        // 再 intPosToFloat"，则每个顶点的 p 都应满足 intPosToFloat(floatPosToInt(p)) == p。
        int bad = 0, tot = 0;
        double worst = 0.0;
        for (MCGAL::Vertex* v : mesh_->vertices()) {
            if (!v || v->isRemoved())
                continue;
            ++tot;
            const MCGAL::Point p0 = v->point();
            const MCGAL::Point p1 = mesh_->intPosToFloat(mesh_->floatPosToInt(p0));
            const double dx = p1.x() - p0.x(), dy = p1.y() - p0.y(), dz = p1.z() - p0.z();
            const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (d > 0.0)
                ++bad;
            if (d > worst)
                worst = d;
        }
        std::fprintf(stderr, "[BASEQC] base mesh 顶点=%d 不满足往返=%d 最大往返差=%.17g\n", tot, bad, worst);
        loggedOnce_ = true;
    }
        std::FILE* pt = nullptr;
    if (const char* pf = std::getenv("PMSF_CONQUEST_PRED_DEC")) {
        if (std::FILE* d = std::fopen(pf, "a")) {
            // 逆序重放：解码端的第 k 层 = 编码端的第 (N-1-k) 层，这里只跑单层时无需处理。
            pt = d;
        }
    }
    last_ = decodeLayer(mesh_.get(), st_, seed, layer->symbols, layer->deltas, grid,
                        hooks_ ? *hooks_ : defaultEngineHooks(), pt);
    if (pt)
        std::fclose(pt);
    totalInserted_ += last_.inserted;
        if (!last_.ok)
        failed_ = true;
}

}  // namespace pmsf
