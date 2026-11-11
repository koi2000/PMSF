/*
 * @file ErrorMetric.h
 * @brief Pluggable edge/vertex cost functor.
 *
 * Today, each algorithm embeds its cost computation inside its
 * SelectOperator subclass (QEM's quadric error, RD's tag-priority rules,
 * POMAR's priority selector, etc.). Extracting it into a pure functor lets
 * new papers inject just a cost rule without subclassing any selector.
 *
 * A SelectOperator (framework- or user-written) queries `cost(halfedge)`
 * and orders candidates accordingly. Zero allocation on the hot path:
 * implementations should store their precomputed state (quadric matrices,
 * curvature tensors, etc.) via PropertyHandleWrapper, not inside the
 * functor.
 *
 * Phase 10 addition — nothing in the existing pipeline depends on this
 * header until an impl/ main opts in via
 * CompressPipeline::Builder::withErrorMetric<T>(). Legacy algorithms keep
 * computing cost inside their own selectors, so bpv/error stays unchanged.
 */
#ifndef PMSF_METRIC_ERROR_METRIC_H
#define PMSF_METRIC_ERROR_METRIC_H

#include "Halfedge.h"
#include "Mesh.h"

#include <limits>
#include <memory>

namespace pmsf {

/// Pure functor returning the "cost of collapsing / removing" a halfedge.
/// Lower cost = higher priority.
///
/// Canonical examples:
///   - QuadricError: returns v^T Q v where Q is the sum of the endpoint
///     quadrics (Garland–Heckbert 1997).
///   - HausdorffError: one-sided discrete Hausdorff estimate to the
///     pre-simplification mesh.
///   - CurvatureError: principal-curvature-magnitude × edge-length.
///   - RDLagrangianError: distortion + λ · bits, where bits is estimated
///     from an entropy coder probe.
///   - VQDistortionError: ||p − codebook[k]||² for the nearest entry k.
///
/// Algorithms can also inherit and provide bespoke cost rules for full
/// paper fidelity (e.g. RD's tag-based 6-level priority).
class ErrorMetric {
public:
    virtual ~ErrorMetric() = default;

    /// Initialise any precomputed state attached to `mesh`.
    virtual void init(std::shared_ptr<MCGAL::Mesh> /*mesh*/) {}

    /// Cost of collapsing / removing around `h`. Must be deterministic:
    /// both compress and decompress rely on a stable ordering.
    virtual double cost(MCGAL::Halfedge* h) const = 0;

    /// Cost with an explicit target position (e.g. the optimal placement of a
    /// collapsed edge). Metrics that do not depend on the target — every
    /// stateless or connectivity-driven metric — keep the default, which just
    /// ignores `target` and delegates to cost(h).
    ///
    /// Phase 1（2026-09-15）：提升为基类虚函数，使 H2 `errorMetric()` hook 能
    /// 统一装配 `SelectOptions::withHalfedgeErrorAccessor`，不必在框架里
    /// `dynamic_cast` 到具体度量类型。
    virtual double costWithTarget(MCGAL::Halfedge* h, MCGAL::Point& /*target*/) const {
        return cost(h);
    }

    /// Optional hook: update precomputed state after an elimination.
    /// QEM needs this (endpoint quadrics change). For stateless metrics
    /// it's a no-op.
    virtual void onAfterEliminate(MCGAL::Halfedge* /*collapsed*/) {}

    /// @brief 该度量是否需要在每次消除后收到刷新回调。
    ///
    /// Phase 1（2026-09-15）：框架的 H2 装配只在本函数返回 true 时才安装
    /// `SelectOptions::withOnAfterEliminate` —— 因为「装不装这个回调」本身
    /// 就是可观测行为（它会改变 SelectOptions 的字段，进而影响选择器的
    /// 拷贝/判空路径）。无状态度量返回 false 即可保持与历史实现一致。
    virtual bool needsAfterEliminateRefresh() const { return false; }

    /// Sentinel meaning "this edge must not be picked" — collapse operators
    /// should treat it as +infinity.
    static constexpr double kInvalid = std::numeric_limits<double>::infinity();
};

}  // namespace pmsf

#endif  // PMSF_METRIC_ERROR_METRIC_H
