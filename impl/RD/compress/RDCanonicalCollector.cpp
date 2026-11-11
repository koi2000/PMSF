// SPDX-License-Identifier: Apache-2.0
//
// RDCanonicalCollector.cpp — task-43 route A 编码端。
// 实现见头文件；sv=71 三流 framing 见 RDCanonicalBlobFormat.h。
#include "RD/compress/RDCanonicalCollector.h"

#include "RD/RDCanonicalBlobFormat.h"
#include "compress/ConnectivityBlobContainer.h"
#include "storage/PaperBlobLayoutRegistry.h"

#include "Configuration.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace rd_canon {

void RDCanonicalCollector::init(std::shared_ptr<MCGAL::Mesh> mesh,
                                std::vector<MCGAL::Halfedge*> seeds,
                                PMSFHeader header) {
    SymbolCollectOperator::init(mesh, seeds, header);
    frameworkSeed_ = seeds.empty() ? nullptr : seeds.front();
    layers_.clear();
    prepareGridAndSnap();
    std::fprintf(stderr,
                 "[RD-CANON] init quantBits=%d quantStep=%.17g bboxMin=(%.17g,%.17g,%.17g)\n",
                 grid_.quantBits, grid_.step, grid_.bboxMin[0], grid_.bboxMin[1],
                 grid_.bboxMin[2]);
}

void RDCanonicalCollector::prepareGridAndSnap() {
    if (!mesh_) return;
    grid_.quantBits = static_cast<int>(mesh_->i_nbQuantBits);
    if (grid_.quantBits <= 0 || grid_.quantBits > 20) grid_.quantBits = 12;
    grid_.bboxMin[0] = static_cast<double>(mesh_->bboxMin[0]);
    grid_.bboxMin[1] = static_cast<double>(mesh_->bboxMin[1]);
    grid_.bboxMin[2] = static_cast<double>(mesh_->bboxMin[2]);
    grid_.step = static_cast<double>(mesh_->f_quantStep);
    if (!(grid_.step > 0.0)) {
        // 兜底：按包围盒最长边现算（正常路径 preInitPreparation 已设好）。
        double lo[3] = {1e300, 1e300, 1e300}, hi[3] = {-1e300, -1e300, -1e300};
        bool first = true;
        for (MCGAL::Vertex* v : mesh_->vertices()) {
            if (!v || v->isRemoved()) continue;
            const double p[3] = {v->x(), v->y(), v->z()};
            for (int k = 0; k < 3; ++k) {
                lo[k] = std::min(lo[k], p[k]);
                hi[k] = std::max(hi[k], p[k]);
            }
            first = false;
        }
        if (first) return;
        double extent = 0.0;
        for (int k = 0; k < 3; ++k) {
            extent = std::max(extent, hi[k] - lo[k]);
            grid_.bboxMin[k] = lo[k];
        }
        grid_.step = extent / static_cast<double>(1u << grid_.quantBits);
        mesh_->f_quantStep = static_cast<float>(grid_.step);
        for (int k = 0; k < 3; ++k) mesh_->bboxMin[k] = static_cast<float>(lo[k]);
    }
    if (!(grid_.step > 0.0)) return;
    // 吸附到**解码端 base mesh 读回的那个世界**（float 公式，逐位一致）。
    for (MCGAL::Vertex* v : mesh_->vertices()) {
        if (!v || v->isRemoved()) continue;
        const MCGAL::PointInt q = mesh_->floatPosToInt(v->point());
        int qq[3];
        const int maxV = quantMaxCell(grid_);
        for (int k = 0; k < 3; ++k) {
            qq[k] = q[k];
            if (qq[k] < 0) qq[k] = 0;
            if (qq[k] > maxV) qq[k] = maxV;
        }
        v->setPoint(MCGAL::Point(cellCenter(grid_, 0, qq[0]),
                                 cellCenter(grid_, 1, qq[1]),
                                 cellCenter(grid_, 2, qq[2])));
    }
}

MCGAL::Halfedge* RDCanonicalCollector::pickLayerSeed(MCGAL::Halfedge* hint) {
    auto alive = [](MCGAL::Halfedge* h) -> bool {
        if (h == nullptr || h->isRemoved()) return false;
        if (h->vertex() == nullptr || h->vertex()->isRemoved()) return false;
        if (h->end_vertex() == nullptr || h->end_vertex()->isRemoved()) return false;
        if (h->face() == nullptr || h->face()->isRemoved()) return false;
        return true;
    };
    if (alive(hint)) return hint;
    if (alive(frameworkSeed_)) return frameworkSeed_;
    return canonicalSeed(mesh_.get(), grid_);
}

void RDCanonicalCollector::recordSeed(LayerRecord& layer,
                                      MCGAL::Halfedge* seed) const {
    if (seed == nullptr) return;
    MCGAL::Vertex* a = seed->end_vertex();       // tgt(seed)
    MCGAL::Vertex* b = seed->vertex();           // tgt(opp(seed))
    if (a == nullptr || b == nullptr) return;
    layer.hasSeed = true;
    layer.seedQ[0] = cellFromPos(grid_, 0, a->x());
    layer.seedQ[1] = cellFromPos(grid_, 1, a->y());
    layer.seedQ[2] = cellFromPos(grid_, 2, a->z());
    layer.seedQ[3] = cellFromPos(grid_, 0, b->x());
    layer.seedQ[4] = cellFromPos(grid_, 1, b->y());
    layer.seedQ[5] = cellFromPos(grid_, 2, b->z());
}

void RDCanonicalCollector::runOneLayer(MCGAL::Halfedge* seed) {
    LayerRecord layer;
    const int round = static_cast<int>(layers_.size());
    MCGAL::Halfedge* use = pickLayerSeed(seed);
    if (use == nullptr) {
        layers_.push_back(std::move(layer));
        std::fprintf(stderr, "[RD-CANON] r=%d NO-SEED (empty layer)\n", round);
        return;
    }
    recordSeed(layer, use);
    State st;
    const EncodeLayerResult r =
        encodeLayer(mesh_.get(), st, use, grid_, layer.syms, layer.residuals);
    if (layer.syms.size() !=
        static_cast<std::size_t>(r.removed + r.nullPatches)) {
        std::fprintf(stderr,
                     "[RD-CANON] r=%d WARN syms=%zu removed=%d null=%d\n", round,
                     layer.syms.size(), r.removed, r.nullPatches);
    }
    std::fprintf(stderr,
                 "[RD-CANON] r=%d pops=%d removed=%d null=%d residuals=%zu\n",
                 round, r.pops, r.removed, r.nullPatches, layer.residuals.size());
    layers_.push_back(std::move(layer));
}

void RDCanonicalCollector::collect(MCGAL::Halfedge* seed) {
    runOneLayer(seed);
}

void RDCanonicalCollector::collect(MCGAL::Halfedge* seed,
                                   const std::vector<MCGAL::Halfedge*>& gates) {
    (void)gates;  // route A 的遍历自管理 gate queue，不用框架 hint
    runOneLayer(seed);
}

void RDCanonicalCollector::collect(std::vector<MCGAL::Halfedge*> seeds) {
    runOneLayer(seeds.empty() ? nullptr : seeds[0]);
}

int RDCanonicalCollector::exportToBufferWithRangeCoder(char* buffer,
                                                       bool enableQuantization) {
    if (buffer == nullptr) return 0;
    if (!enableQuantization) {
        throw std::runtime_error(
            "[RDCanon] sv=71 requires quantization (integer-domain residuals)");
    }
    if (layers_.empty()) {
        std::fprintf(stderr, "[RD-CANON] export: no layers\n");
        return 0;
    }
    const int numRounds = static_cast<int>(layers_.size());
    pmsf::ConnectivityBlobBuilder cbb(pmsf::rdTagConnectivityBlobSpec(
        numRounds, kCanonBlobSchemeVersion, kCanonStreamCount));
    long long valBytes = 0, resBytes = 0, seedBytes = 0;
    long long symTotal = 0, nullTotal = 0, resTotal = 0;
    long long nonEmptyLayers = 0;
    std::vector<std::uint8_t> s0, s1, s2;
    for (int r = 0; r < numRounds; ++r) {
        const LayerRecord& layer = layers_[static_cast<std::size_t>(r)];
        encodeSymbolStream(layer.syms, s0);
        encodeResidualStream(layer.residuals, s1);
        encodeSeedStream(layer.hasSeed, layer.seedQ, layer.seedQ + 3, s2);
        if (!layer.syms.empty()) ++nonEmptyLayers;
        for (int sym : layer.syms) {
            if (sym == 4)
                ++nullTotal;
            else
                ++symTotal;
        }
        resTotal += static_cast<long long>(layer.residuals.size());
        valBytes += static_cast<long long>(s0.size());
        resBytes += static_cast<long long>(s1.size());
        seedBytes += static_cast<long long>(s2.size());
        cbb.appendStreamBytes(r, kCanonStreamSymbols, s0);
        cbb.appendStreamBytes(r, kCanonStreamResiduals, s1);
        cbb.appendStreamBytes(r, kCanonStreamSeed, s2);
    }
    const std::vector<std::uint8_t> blob = cbb.finalize();
    if (blob.size() > static_cast<std::size_t>(BUFFER_SIZE)) {
        throw std::runtime_error("[RDCanon] blob exceeds BUFFER_SIZE");
    }
    std::memcpy(buffer, blob.data(), blob.size());
    const long long containerBytes =
        static_cast<long long>(blob.size()) - valBytes - resBytes - seedBytes;
    std::fprintf(stderr,
                 "[RD-CANON] sv=%d rounds=%d streams=%d layers_nonempty=%lld "
                 "valence_syms=%lld null_patch=%lld residuals=%lld "
                 "conn_rc_bytes=%lld residual_rc_bytes=%lld seed_bytes=%lld "
                 "container_bytes=%lld blob_bytes=%zu\n",
                 kCanonBlobSchemeVersion, numRounds, kCanonStreamCount,
                 nonEmptyLayers, symTotal, nullTotal, resTotal, valBytes,
                 resBytes, seedBytes, containerBytes, blob.size());
    return static_cast<int>(blob.size());
}

int RDCanonicalCollector::exportToBuffer(char* /*buffer*/,
                                         bool /*enableQuantization*/,
                                         bool /*enableCompress*/) {
    throw std::runtime_error(
        "[RDCanon] exportToBuffer (non-RC) is not implemented.");
}

}  // namespace rd_canon
