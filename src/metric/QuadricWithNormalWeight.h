/*
 * @file QuadricWithNormalWeight.h
 * @brief QEM 误差 × 法向差加权 —— APTM / feature-preserving 的常用变体。
 *
 * Garland 1998 / Cheng et al. 2006 在标准 QEM 之上引入"normal-angle weighting"：
 * 边 (u, v) 如果两端法向接近，代价降低；法向差大（尖锐特征）代价升高。
 * 这保留几何特征，同时牺牲一点点平坦区域的重建精度。
 *
 *   cost(h) = QEM(h) * (1 + α * (1 − n_u · n_v))
 *
 * α 默认 2.0（论文 APTM §4）。调用者可构造时覆盖。
 */
#ifndef PMSF_METRIC_QUADRIC_WITH_NORMAL_WEIGHT_H
#define PMSF_METRIC_QUADRIC_WITH_NORMAL_WEIGHT_H

#include "ErrorMetric.h"
#include "QuadricError.h"

#include <memory>

namespace pmsf {

class QuadricWithNormalWeight final : public ErrorMetric {
public:
    QuadricWithNormalWeight(QuadricAccessor accessor, double alpha = 2.0)
        : base_(std::move(accessor)), alpha_(alpha) {}

    double cost(MCGAL::Halfedge* h) const override {
        const double q = base_.cost(h);
        MCGAL::Vector3 nu = vertexNormal(h->vertex());
        MCGAL::Vector3 nv = vertexNormal(h->opposite()->vertex());
        const double nun = static_cast<double>(nu.length());
        const double nvn = static_cast<double>(nv.length());
        if (nun < 1e-12 || nvn < 1e-12) return q;
        const double dot = static_cast<double>(nu | nv) / (nun * nvn);
        const double clamped = dot < -1.0 ? -1.0 : (dot > 1.0 ? 1.0 : dot);
        return q * (1.0 + alpha_ * (1.0 - clamped));
    }

    void onAfterEliminate(MCGAL::Halfedge* collapsed) override {
        base_.onAfterEliminate(collapsed);
    }

private:
    static MCGAL::Vector3 vertexNormal(MCGAL::Vertex* v) {
        MCGAL::Vector3 n(0, 0, 0);
        int count = 0;
        for (MCGAL::Halfedge* he : v->halfedges()) {
            if (he == nullptr || he->face() == nullptr || he->face()->isRemoved()) continue;
            n = n + he->face()->computeNormal();
            ++count;
        }
        if (count > 0) n = n / count;
        return n;
    }

    QuadricError base_;
    double alpha_;
};

}  // namespace pmsf

#endif  // PMSF_METRIC_QUADRIC_WITH_NORMAL_WEIGHT_H
