/*
 * @file DualParallelogramPredictor.h
 * @brief PCATM 的双平行四边形预测 —— 两个相邻面的 parallelogram 预测平均。
 *
 * 论文 PCATM (Lee et al. 2012) §3.2 用面中心点 q 与 1-环邻居组成两个"virtual
 * parallelogram"，取它们预测的平均作为更鲁棒的中心点预测：
 *
 *   p̂ = 1/2 ( (a + c − b)  +  (a' + c' − b') )
 *
 * 其中 (a, b, c) 来自 h->face() 的 1-环，(a', b', c') 来自 opposite()->face() 的 1-环。
 *
 * 若任一面不可达（边界 / 半边 null），退化为单 parallelogram。
 */
#ifndef PMSF_PREDICT_DUAL_PARALLELOGRAM_PREDICTOR_H
#define PMSF_PREDICT_DUAL_PARALLELOGRAM_PREDICTOR_H

#include "ParallelogramPredictor.h"
#include "Predictor.h"

namespace pmsf {

class DualParallelogramPredictor final : public Predictor {
public:
    MCGAL::Point predict(MCGAL::Halfedge* h) const override {
        if (h == nullptr) return MCGAL::Point(0.0f, 0.0f, 0.0f);
        MCGAL::Point p1 = single_.predict(h);
        if (h->opposite() == nullptr) return p1;
        MCGAL::Point p2 = single_.predict(h->opposite());
        return MCGAL::Point(0.5f * (p1.x() + p2.x()),
                            0.5f * (p1.y() + p2.y()),
                            0.5f * (p1.z() + p2.z()));
    }

private:
    ParallelogramPredictor single_;
};

}  // namespace pmsf

#endif  // PMSF_PREDICT_DUAL_PARALLELOGRAM_PREDICTOR_H
