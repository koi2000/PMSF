/*
 * @file Predictor.h
 * @brief Pluggable vertex-position / symbol-value predictor.
 *
 * Separates "what is the predicted value of the next inserted point?" from
 * "how do I traverse / collect symbols?". Today parallelogram / Frenet /
 * PCA-basis / anchor-relative predictors are inlined in each paper's
 * SymbolCollectOperator. A reusable Predictor hook lets new algorithms
 * plug only the prediction rule and reuse the standard collector.
 *
 * The predictor outputs a residual (ground truth − prediction) that the
 * collector writes into a Parquet-V3 side-channel ("residual"). At decode
 * time the reconstructor predicts again and adds the residual back.
 *
 * Phase 10 addition — not hooked into any existing code path.
 */
#ifndef PMSF_PREDICT_PREDICTOR_H
#define PMSF_PREDICT_PREDICTOR_H

#include "Halfedge.h"
#include "Mesh.h"
#include "Point.h"

#include <memory>

namespace pmsf {

class Predictor {
public:
    virtual ~Predictor() = default;

    virtual void init(std::shared_ptr<MCGAL::Mesh> /*mesh*/) {}

    /// Predict the position of the removed vertex associated with `h` / `f`.
    /// Implementations may consult the local 1-ring, the face's barycenter,
    /// a Frenet frame at the gate edge, a PCA basis on a patch, etc.
    virtual MCGAL::Point predict(MCGAL::Halfedge* h) const = 0;

    /// Compute the residual r = ground − prediction. Default does point
    /// subtraction; predictors that want to encode in a rotated basis
    /// (e.g. Frenet) should override.
    virtual MCGAL::Point residual(MCGAL::Halfedge* h, const MCGAL::Point& ground) const {
        const MCGAL::Point p = predict(h);
        return MCGAL::Point(ground.x() - p.x(), ground.y() - p.y(), ground.z() - p.z());
    }

    /// Inverse: prediction + residual = reconstructed ground truth. Mirrors
    /// `residual` and must round-trip exactly for byte-identity.
    virtual MCGAL::Point reconstruct(MCGAL::Halfedge* h, const MCGAL::Point& residual) const {
        const MCGAL::Point p = predict(h);
        return MCGAL::Point(p.x() + residual.x(), p.y() + residual.y(), p.z() + residual.z());
    }
};

}  // namespace pmsf

#endif  // PMSF_PREDICT_PREDICTOR_H
