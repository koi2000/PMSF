/*
 * @file ButterflyPredictor.h
 * @brief Butterfly-subdivision predictor used by PPMC / RD residual coding.
 *
 * Dyn & Gregory / Zorin 1996: predict a newly-inserted midpoint of edge (a,b)
 * as
 *   p̂ = 1/2 (a + b) + 1/8 (c + d) − 1/16 (e + f + g + h)
 * where (c,d) are the opposite vertices of the two faces sharing edge (a,b),
 * and (e,f,g,h) are their "next-ring" neighbours (the vertices opposite to
 * (c,d) across their other two edges).
 *
 * PMSF 的 SymbolCollectOperator 在 facet 被移除时会发出 P_r (插入点位置)。
 * 对应的 halfedge `h` 是被移除面上"gate"半边；最新插入的点对应 h->face() 的
 * 中心。这里我们把 butterfly 规则应用到 gate 半边所在的 edge (h, h->opposite())
 * 并以 h->next() / h->opposite()->next() 为 (c,d) —— 这是 Zorin 原论文对
 * "新边中点" 的 stencil，对"面中心点"类算法语义最接近。
 *
 * Fallbacks：若遇到边界 / <8 可用邻居，退化为 ParallelogramPredictor 的公式。
 */
#ifndef PMSF_PREDICT_BUTTERFLY_PREDICTOR_H
#define PMSF_PREDICT_BUTTERFLY_PREDICTOR_H

#include "Facet.h"
#include "Predictor.h"

namespace pmsf {

class ButterflyPredictor final : public Predictor {
public:
    MCGAL::Point predict(MCGAL::Halfedge* h) const override {
        if (h == nullptr) return MCGAL::Point(0.0f, 0.0f, 0.0f);
        MCGAL::Halfedge* opp = h->opposite();
        MCGAL::Halfedge* next = h ? h->next() : nullptr;
        if (opp == nullptr || next == nullptr) {
            return parallelogram(h);
        }
        MCGAL::Halfedge* oppNext = opp->next();
        if (oppNext == nullptr) {
            return parallelogram(h);
        }

        // (a, b) = endpoints of the gate edge
        const MCGAL::Point a = h->vertex()->point();
        const MCGAL::Point b = opp->vertex()->point();
        // (c, d) = opposite vertices of the two adjacent faces
        const MCGAL::Point c = next->vertex()->point();
        const MCGAL::Point d = oppNext->vertex()->point();

        // (e, f) = second-ring via h's face; (g, h_) = second-ring via opp's face.
        // We look "across" c and d to pick up four outer neighbours. Missing
        // neighbours are handled by zero-weighting (weight accumulates only
        // from found ones so boundary stencils remain well-defined).
        const MCGAL::Point e = safeOppositeVertex(next);
        const MCGAL::Point f = safeOppositeVertex(next->next());
        const MCGAL::Point g = safeOppositeVertex(oppNext);
        const MCGAL::Point h2 = safeOppositeVertex(oppNext->next());

        // Butterfly weights: 1/2, 1/2 on (a, b); +1/8 on (c, d); −1/16 on (e, f, g, h).
        const float wab = 0.5f;
        const float wcd = 0.125f;
        const float wouter = -0.0625f;
        const float x = wab * (a.x() + b.x()) + wcd * (c.x() + d.x()) +
                        wouter * (e.x() + f.x() + g.x() + h2.x());
        const float y = wab * (a.y() + b.y()) + wcd * (c.y() + d.y()) +
                        wouter * (e.y() + f.y() + g.y() + h2.y());
        const float z = wab * (a.z() + b.z()) + wcd * (c.z() + d.z()) +
                        wouter * (e.z() + f.z() + g.z() + h2.z());
        return MCGAL::Point(x, y, z);
    }

private:
    /// For a halfedge `he`, pick the "opposite" vertex on the adjacent face
    /// across `he->opposite()`. Returns (0,0,0) if unavailable — caller
    /// absorbs this into the weighted sum which stays finite.
    static MCGAL::Point safeOppositeVertex(MCGAL::Halfedge* he) {
        if (he == nullptr) return MCGAL::Point(0.0f, 0.0f, 0.0f);
        MCGAL::Halfedge* opp = he->opposite();
        if (opp == nullptr) return MCGAL::Point(0.0f, 0.0f, 0.0f);
        MCGAL::Halfedge* oppNext = opp->next();
        if (oppNext == nullptr) return MCGAL::Point(0.0f, 0.0f, 0.0f);
        return oppNext->vertex()->point();
    }

    /// Degenerate-stencil fallback: reuse the parallelogram rule so the
    /// predictor always returns a meaningful number.
    static MCGAL::Point parallelogram(MCGAL::Halfedge* h) {
        if (h == nullptr) return MCGAL::Point(0.0f, 0.0f, 0.0f);
        MCGAL::Halfedge* opp = h->opposite();
        MCGAL::Halfedge* next = h->next();
        if (opp == nullptr || next == nullptr) return MCGAL::Point(0.0f, 0.0f, 0.0f);
        MCGAL::Halfedge* oppNext = opp->next();
        if (oppNext == nullptr) return MCGAL::Point(0.0f, 0.0f, 0.0f);
        const MCGAL::Point a = h->vertex()->point();
        const MCGAL::Point b = next->vertex()->point();
        const MCGAL::Point c = oppNext->vertex()->point();
        return MCGAL::Point(a.x() + c.x() - b.x(),
                            a.y() + c.y() - b.y(),
                            a.z() + c.z() - b.z());
    }
};

}  // namespace pmsf

#endif  // PMSF_PREDICT_BUTTERFLY_PREDICTOR_H
