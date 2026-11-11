// SPDX-License-Identifier: Apache-2.0
//
// BarycenterPredictor.h
//
// Quantized polygon-barycenter predictor used by PCATM/PCLTTM/RD-style
// vertex-removal streams.  The important API for strict PCATM is the integer
// path: encoder and decoder must both average quantized boundary vertices,
// then add/subtract the residual in the same integer domain.

#ifndef PMSF_PREDICT_BARYCENTER_PREDICTOR_H
#define PMSF_PREDICT_BARYCENTER_PREDICTOR_H

#include "Predictor.h"
#include "predict/RDPredictor.h"

namespace pmsf {

class BarycenterPredictor final : public Predictor {
public:
    void init(std::shared_ptr<MCGAL::Mesh> mesh) override {
        mesh_ = std::move(mesh);
    }

    MCGAL::Point predict(MCGAL::Halfedge* h) const override {
        MCGAL::Facet* f = h ? h->face() : nullptr;
        if (!f || !mesh_) return MCGAL::Point(0.0f, 0.0f, 0.0f);
        return MCGAL::rdIntToFloatPoint(
            predictQuantized(f), mesh_->bboxMin, mesh_->f_quantStep);
    }

    MCGAL::PointInt predictQuantized(MCGAL::Facet* f) const {
        return MCGAL::rdBarycenterQuantized(f, mesh_);
    }

    MCGAL::PointInt residualQuantized(MCGAL::Facet* f,
                                      const MCGAL::PointInt& ground) const {
        const MCGAL::PointInt p = predictQuantized(f);
        return MCGAL::PointInt(ground.x() - p.x(),
                               ground.y() - p.y(),
                               ground.z() - p.z());
    }

    MCGAL::PointInt reconstructQuantized(MCGAL::Facet* f,
                                         const MCGAL::PointInt& residual) const {
        const MCGAL::PointInt p = predictQuantized(f);
        return MCGAL::PointInt(residual.x() + p.x(),
                               residual.y() + p.y(),
                               residual.z() + p.z());
    }

    MCGAL::Point reconstructPoint(MCGAL::Facet* f,
                                  const MCGAL::PointInt& residual) const {
        if (!mesh_) return MCGAL::Point(0.0f, 0.0f, 0.0f);
        return MCGAL::rdIntToFloatPoint(
            reconstructQuantized(f, residual), mesh_->bboxMin, mesh_->f_quantStep);
    }

private:
    std::shared_ptr<MCGAL::Mesh> mesh_;
};

}  // namespace pmsf

#endif  // PMSF_PREDICT_BARYCENTER_PREDICTOR_H
