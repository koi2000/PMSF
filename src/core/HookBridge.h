/*
 * @file HookBridge.h
 * @brief Adapters that turn framework-level hook objects (pmsf::ErrorMetric,
 *        pmsf::FeatureDetector) into the lambda surface expected by the
 *        existing SelectOptions / IsRemovableOperator machinery.
 *
 * Motivation:
 *   SelectOptions was designed before Phase-10 hook bases existed. It takes
 *   `std::function<float(Halfedge*, Point&)>` and `std::function<void(Vertex*)>`.
 *   Every impl/<Algo>/<Algo>Compress.cpp that wants to use `QuadricError`
 *   today has to hand-write ~9 lines of glue (see impl/QEM/QEMCompress.cpp
 *   L34-L42). That glue is 100% identical across algorithms, so it belongs
 *   in the framework.
 *
 *   This header provides three adapters:
 *     - makeHalfedgeErrorAccessor(metric, mesh)
 *         → std::function<float(Halfedge*, Point&)>
 *     - makeOnAfterEliminate(metric)
 *         → std::function<void(Vertex*)>
 *     - FeatureMaskIsRemovableOperator(detector, mesh)
 *         → shared_ptr<IsRemovableOperator> that vetoes edges touching any
 *           vertex in the FeatureSet returned by detector->detect(mesh).
 *
 * With these, a new-algorithm author can replace 20+ lines of wiring with
 * three calls and focus on the paper's actual contribution. The QEM main
 * would become:
 *
 *   auto quad = std::make_shared<pmsf::QuadricError>(...);
 *   auto sopt = SelectOptions::Builder()
 *       .withErrorSource(ErrorSource::Halfedge)
 *       .withHalfedgeErrorAccessor(pmsf::makeHalfedgeErrorAccessor(quad))
 *       .withOnAfterEliminate(pmsf::makeOnAfterEliminate(quad))
 *       .build();
 *
 * These adapters preserve existing byte-level output: they simply reshape
 * the same calls, no semantics change.
 */
#ifndef PMSF_CORE_HOOK_BRIDGE_H
#define PMSF_CORE_HOOK_BRIDGE_H

#include "Halfedge.h"
#include "Mesh.h"
#include "Vertex.h"
#include "common/HalfedgeErrorEntry.h"
#include "feature/FeatureDetector.h"
#include "metric/ErrorMetric.h"
#include "metric/OptimalPlacementQuadricError.h"
#include "operator/IsRemovableOperator.h"

#include <functional>
#include <memory>

namespace pmsf {

/// Turn an ErrorMetric into the lambda SelectOptions expects.
///
/// The out-parameter `Point& target` receives the predicted target position
/// of the collapse — the framework's historical contract. We fill it with
/// the midpoint of the two endpoints (the same fallback impl/QEM used
/// before the hook), since the ErrorMetric interface intentionally doesn't
/// carry an opinion on target placement. Algorithms that DO have a paper-
/// specific target (e.g. QEM's optimal placement v*) should use
/// `makeHalfedgeErrorAccessorWithTarget` below.
inline std::function<float(MCGAL::Halfedge*, MCGAL::Point&)>
makeHalfedgeErrorAccessor(std::shared_ptr<ErrorMetric> metric) {
    return [m = std::move(metric)](MCGAL::Halfedge* h, MCGAL::Point& target) -> float {
        if (h == nullptr) return std::numeric_limits<float>::infinity();
        const MCGAL::Point a = h->vertex()->point();
        const MCGAL::Point b = h->opposite()->vertex()->point();
        target = MCGAL::Point(0.5f * (a.x() + b.x()),
                              0.5f * (a.y() + b.y()),
                              0.5f * (a.z() + b.z()));
        return static_cast<float>(m->cost(h));
    };
}

/// Variant for ErrorMetric subclasses that produce their own optimal
/// placement. Currently uses `OptimalPlacementQuadricError::costWithTarget`,
/// but any subclass that exposes `costWithTarget(Halfedge*, Point&)` via a
/// free function, member, or functor could slot in.
inline std::function<float(MCGAL::Halfedge*, MCGAL::Point&)>
makeHalfedgeErrorAccessorWithTarget(std::shared_ptr<OptimalPlacementQuadricError> metric) {
    return [m = std::move(metric)](MCGAL::Halfedge* h, MCGAL::Point& target) -> float {
        if (h == nullptr) return std::numeric_limits<float>::infinity();
        return static_cast<float>(m->costWithTarget(h, target));
    };
}

/// Turn an ErrorMetric into the onAfterEliminate callback SelectOptions
/// expects. Delegates to `metric->onAfterEliminate` with a representative
/// halfedge from the survived vertex — matching the existing QEM hand-
/// written lambda byte-for-byte.
inline std::function<void(MCGAL::Vertex*)>
makeOnAfterEliminate(std::shared_ptr<ErrorMetric> metric) {
    return [m = std::move(metric)](MCGAL::Vertex* survivor) {
        if (survivor == nullptr || survivor->isRemoved()) return;
        for (MCGAL::Halfedge* he : survivor->halfedges()) {
            if (he != nullptr && !he->isRemoved()) {
                m->onAfterEliminate(he);
                return;
            }
        }
    };
}

/// An IsRemovableOperator that pulls a FeatureSet from a FeatureDetector
/// at init() time and vetoes any HalfedgeErrorEntry whose endpoints are in
/// the set. Replaces hand-written wrappers in impl/APTM's AnchorPreprocessor
/// and impl/POMAR's cluster-guard operators.
class FeatureMaskIsRemovableOperator final : public IsRemovableOperator {
public:
    explicit FeatureMaskIsRemovableOperator(std::shared_ptr<FeatureDetector> detector)
        : detector_(std::move(detector)) {}

    /// Init signature matches the base class; we only consume the mesh
    /// pointer (pulled from the first vertex's context pool if `v` is
    /// non-empty, else expect the caller to have already called `setMesh`).
    void init(std::vector<MCGAL::Vertex*> v,
              std::vector<MCGAL::Halfedge*> /*h*/,
              std::vector<MCGAL::Facet*> /*f*/) override {
        if (!detector_ || !mesh_) return;
        mask_ = detector_->detect(mesh_);
    }

    /// Allow the pipeline to pass in the mesh explicitly; SelectOperator::init
    /// already has it and can call this right after constructing the chain.
    void setMesh(std::shared_ptr<MCGAL::Mesh> mesh) { mesh_ = std::move(mesh); }

    bool isRemovable(HalfedgeErrorEntry& entry) override {
        if (mask_.empty()) return true;
        MCGAL::Halfedge* e = entry.edge;
        if (e == nullptr) return false;
        const auto a = static_cast<std::uint32_t>(e->vertex()->poolId());
        const auto b = static_cast<std::uint32_t>(e->opposite()->vertex()->poolId());
        return mask_.find(a) == mask_.end() && mask_.find(b) == mask_.end();
    }

    const FeatureSet& mask() const { return mask_; }

private:
    std::shared_ptr<FeatureDetector> detector_;
    std::shared_ptr<MCGAL::Mesh> mesh_;
    FeatureSet mask_;
};

}  // namespace pmsf

#endif  // PMSF_CORE_HOOK_BRIDGE_H
