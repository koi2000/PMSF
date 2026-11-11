/*
 * @file ParallelogramPredictor.h
 * @brief Touma–Gotsman 1998 parallelogram prediction (shared baseline for
 *        MMSE / PCLTTM / PCATM).
 *
 * Given a removed vertex adjacent to halfedge `h`, its predicted position
 * is p̂ = a + c − b, where (a, b, c) are three 1-ring neighbours forming a
 * local triangle. When the gate has fewer than 3 usable neighbours we fall
 * back to the face-barycenter.
 *
 * The exact choice of (a, b, c) matters for byte identity: we take the
 * gate's source vertex, the gate's next's source vertex, and the gate's
 * opposite's next's source vertex — a stable choice that does not depend
 * on the halfedge pool ordering.
 */
#ifndef PMSF_PREDICT_PARALLELOGRAM_PREDICTOR_H
#define PMSF_PREDICT_PARALLELOGRAM_PREDICTOR_H

#include "Facet.h"
#include "Predictor.h"

namespace pmsf {

class ParallelogramPredictor final : public Predictor {
public:
    MCGAL::Point predict(MCGAL::Halfedge* h) const override {
        if (h == nullptr) return MCGAL::Point(0.0f, 0.0f, 0.0f);
        MCGAL::Halfedge* opp = h->opposite();
        MCGAL::Halfedge* next = h->next();
        if (opp == nullptr || next == nullptr) return baryFallback(h);
        MCGAL::Halfedge* oppNext = opp->next();
        if (oppNext == nullptr) return baryFallback(h);
        const MCGAL::Point a = h->vertex()->point();
        const MCGAL::Point b = next->vertex()->point();
        const MCGAL::Point c = oppNext->vertex()->point();
        return MCGAL::Point(a.x() + c.x() - b.x(),
                            a.y() + c.y() - b.y(),
                            a.z() + c.z() - b.z());
    }

private:
    static MCGAL::Point baryFallback(MCGAL::Halfedge* h) {
        MCGAL::Facet* f = h->face();
        if (f == nullptr) return MCGAL::Point(0.0f, 0.0f, 0.0f);
        // Average the face's vertex positions — mirrors the classic
        // "centroid prediction" fallback used by several of the papers.
        float sx = 0, sy = 0, sz = 0;
        int n = 0;
            // ── PHASE66 物品2：**步数上限 + 断链保护** ─────────────────────
            // 与 RDPredictor.h / Facet.cpp 同族（无界环走，能 OOM 整机）。
            // 正常面环必闭合 ⇒ 上限永不触发 ⇒ 门禁逐字节不变。
        constexpr int kMaxRingSteps = 256;
        MCGAL::Halfedge* it = h;
        for (int steps = 0; steps < kMaxRingSteps; ++steps) {
            const MCGAL::Point p = it->vertex()->point();
            sx += p.x();
            sy += p.y();
            sz += p.z();
            ++n;
            it = it->next();
            if (it == nullptr || it == h) break;   // 回到起点（正常）或断链（异常）
        }
        if (n == 0) return MCGAL::Point(0.0f, 0.0f, 0.0f);
        return MCGAL::Point(sx / n, sy / n, sz / n);
    }
};

}  // namespace pmsf

#endif  // PMSF_PREDICT_PARALLELOGRAM_PREDICTOR_H
