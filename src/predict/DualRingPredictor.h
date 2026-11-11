/*
 * @file DualRingPredictor.h
 * @brief MMSE-style 1-ring + alpha-weighted 2-ring residual predictor.
 *
 * Valette, Chaine, Prost 2009 MMSE §3: predict the face-centre insert
 * position as
 *   uj  = mean(1-ring neighbours)
 *   Su  = Σ_{vl ∈ 1-ring} 1/|vl|² · (|vl|·xl − Σ_{vk ∈ 2-ring(vl), vk ∉ 1-ring} xk)
 *   w   = 1 + α · Σ_{vl ∈ 1-ring} 1/|vl|²
 *   p̂   = (uj + α · Su) / w
 *
 * α = 0 reduces to a plain barycentric predictor (ParallelogramPredictor
 * over a face rather than an edge).
 *
 * Note on the Predictor interface: the hook takes a Halfedge* at its gate,
 * we predict the position of the removed vertex inside `h->face()`.
 *
 * Byte-identical with impl/MMSE/MMSEPostProcessor.cpp::dualRingPrediction
 * because the computation is the same float-ordered summation.
 */
#ifndef PMSF_PREDICT_DUAL_RING_PREDICTOR_H
#define PMSF_PREDICT_DUAL_RING_PREDICTOR_H

#include "Facet.h"
#include "common/BoundedRingTraversal.h"
#include "Halfedge.h"
#include "Point.h"
#include "Predictor.h"
#include "Vertex.h"

#include <vector>

namespace pmsf {

class DualRingPredictor final : public Predictor {
public:
    explicit DualRingPredictor(double alpha = 0.5) : alpha_(alpha) {}

    MCGAL::Point predict(MCGAL::Halfedge* h) const override {
        if (h == nullptr || h->face() == nullptr) return MCGAL::Point(0, 0, 0);
        return predictForFacet(h->face());
    }

    /// Direct API for callers that have a facet pointer (MMSE's postprocess
    /// iterates facets not halfedges).
    MCGAL::Point predictForFacet(MCGAL::Facet* facet) const {
        std::vector<MCGAL::Vertex*> neighbours;
        MCGAL::Halfedge* st = facet->proxyHalfedge();
        if (st == nullptr) return MCGAL::Point(0, 0, 0);
            // ── PHASE66 物品2：**步数上限 + 断链保护** ─────────────────────
            // 与 RDPredictor.h / Facet.cpp 同族（无界环走，能 OOM 整机）。
            // 正常面环必闭合 ⇒ 上限永不触发 ⇒ 门禁逐字节不变。
        constexpr int kMaxRingSteps = 256;
        MCGAL::Halfedge* it = st;
        for (int steps = 0; steps < kMaxRingSteps; ++steps) {
            MCGAL::Vertex* v = it->vertex();
            if (v && !v->isRemoved()) neighbours.push_back(v);
            it = it->next();
            if (it == nullptr || it == st) break;  // 回到起点（正常）或断链（异常）
        }
        if (neighbours.empty()) return MCGAL::Point(0, 0, 0);

        // Primary ring: uj = mean of 1-ring
        MCGAL::Point uj(0, 0, 0);
        for (MCGAL::Vertex* vl : neighbours) uj = uj + vl->point();
        uj = uj / static_cast<int>(neighbours.size());
        if (alpha_ < 1e-6) return uj;

        // Secondary ring weighted sum.
        MCGAL::Point secondary(0, 0, 0);
        double w_sum = 0.0;
        for (MCGAL::Vertex* vl : neighbours) {
            if (vl->proxyHalfedge() != nullptr && vl->proxyHalfedge()->isRemoved()) continue;
            std::vector<MCGAL::Vertex*> vl_neigh;
            for (MCGAL::Halfedge* he : pmsf::boundedRing(vl)) {
                MCGAL::Vertex* vk = he->end_vertex();
                if (vk && !vk->isRemoved()) vl_neigh.push_back(vk);
            }
            const int val = static_cast<int>(vl_neigh.size());
            if (val == 0) continue;
            const double weight = 1.0 / (static_cast<double>(val) * val);
            w_sum += weight;
            MCGAL::Point xl = vl->point();
            MCGAL::Point neighbourSum(0, 0, 0);
            for (MCGAL::Vertex* vk : vl_neigh) {
                bool in1Ring = false;
                for (MCGAL::Vertex* nb : neighbours) {
                    if (vk == nb) { in1Ring = true; break; }
                }
                if (!in1Ring) neighbourSum = neighbourSum + vk->point();
            }
            MCGAL::Point scaled(xl.x() * val, xl.y() * val, xl.z() * val);
            MCGAL::Point term = scaled - neighbourSum;
            secondary = secondary + MCGAL::Point(term.x() * weight,
                                                  term.y() * weight,
                                                  term.z() * weight);
        }
        const double w = 1.0 + alpha_ * w_sum;
        MCGAL::Point alphaSec(secondary.x() * alpha_,
                              secondary.y() * alpha_,
                              secondary.z() * alpha_);
        MCGAL::Point result = uj + alphaSec;
        return result / w;
    }

    double alpha() const { return alpha_; }

private:
    double alpha_;
};

}  // namespace pmsf

#endif  // PMSF_PREDICT_DUAL_RING_PREDICTOR_H
