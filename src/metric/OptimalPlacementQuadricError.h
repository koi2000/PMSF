/*
 * @file OptimalPlacementQuadricError.h
 * @brief Garland–Heckbert QEM with the original paper's optimal-placement
 *        (det-based) target + midpoint fallback. Framework-level reusable
 *        promotion of impl/QEM/QEMQuadric.h's core math.
 *
 * The existing `QuadricError` (src/metric/QuadricError.h) evaluates the
 * summed quadric at the edge midpoint — fine for byte-identity with the
 * simplest baseline, but not competitive with the paper. QEM's actual
 * contribution is the optimal placement v* that minimises v^T Q v:
 *
 *       ⎡ q0 q1 q2 ⎤ ⎡ x ⎤     ⎡ q3 ⎤
 *       ⎜ q1 q4 q5 ⎟ ⎜ y ⎟  =  ⎜ q6 ⎟
 *       ⎣ q2 q5 q7 ⎦ ⎣ z ⎦     ⎣ q8 ⎦
 *
 * Solved via Cramer's rule (3 determinants) when the system is
 * non-degenerate (|det| > ε). Fallback: pick the endpoint / midpoint with
 * the smallest error.
 *
 * Why a separate class: this hook requires user-provided
 *   (a) a `SymetricMatrix` accessor to the per-vertex quadric attribute;
 *   (b) an OUT-parameter target point to be consumed by SelectOptions'
 *       halfedgeErrorAccessor lambda (via HookBridge).
 * The simpler `QuadricError` can't do (b). This class extends ErrorMetric
 * with a `costWithTarget(...)` method; impl/ authors call it through the
 * hook bridge `makeHalfedgeErrorAccessorWithTarget(...)` below.
 *
 * Byte-identity with impl/QEM/QEMQuadric.h::calculateQuadricError is
 * preserved (same float ordering, same determinant form, same fallback
 * tie-breaking). Once QEMCompress.cpp switches to this header, its
 * in-main SelectOptions lambda can be replaced with a single bridge call.
 */
#ifndef PMSF_METRIC_OPTIMAL_PLACEMENT_QUADRIC_ERROR_H
#define PMSF_METRIC_OPTIMAL_PLACEMENT_QUADRIC_ERROR_H

#include "ErrorMetric.h"
#include "Halfedge.h"
#include "Point.h"
#include "Vertex.h"
#include "symetric_matrix.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <functional>

namespace pmsf {

/// Accessor into a per-vertex SymetricMatrix property. The algorithm's main
/// function registers the attribute (via ContextPool) and provides a lambda
/// that returns a reference to the quadric for a given vertex.
using SymetricMatrixAccessor = std::function<SymetricMatrix&(MCGAL::Vertex*)>;

/// Optional hook for post-collapse quadric refresh (recomputes the survivor
/// and its 1-ring). Left empty by default so stateless reuse stays trivial.
using SymetricMatrixRefresher = std::function<void(MCGAL::Vertex* /*survivor*/)>;

class OptimalPlacementQuadricError final : public ErrorMetric {
public:
    OptimalPlacementQuadricError(SymetricMatrixAccessor accessor,
                                 SymetricMatrixRefresher refresher = {})
        : accessor_(std::move(accessor)), refresher_(std::move(refresher)) {}

    /// ErrorMetric interface: cost WITHOUT target. Use costWithTarget when
    /// the caller needs the optimal placement (normal use).
    double cost(MCGAL::Halfedge* h) const override {
        MCGAL::Point tmp;
        return costWithTarget(h, tmp);
    }

    /// 带 optimal placement 的二次误差度量：每次消除后必须刷新 Q 矩阵。
    bool needsAfterEliminateRefresh() const override { return true; }

    /// Classic QEM collapse cost + optimal placement. Matches
    /// impl/QEM/QEMQuadric.h::calculateQuadricError byte-for-byte when the
    /// same accessor is passed.
    double costWithTarget(MCGAL::Halfedge* edge, MCGAL::Point& target) const override {
        // ── PHASE64：**边界半边保护**（修 dragon 上的段错误） ──────────────
        // 实测（附 83/84）：dragon（开边界网格，87 万面）上 QEM 与 RD 都在
        // `edge->opposite()->vertex()` 处 SIGSEGV。`opposite()` 在**边界半边**上
        // 为空指针，而这里（以及框架 accessor）都只防了 `e == nullptr`。
        //
        // 语义：边界半边**不可坍缩** ⇒ 代价取**无穷**，与框架 accessor 里
        // `if (e == nullptr) return infinity;` 的既有约定一致。
        //
        // ⚠️ 字节中性：bunny/horse 上这条路径**从未触发**（它们没有使
        // `opposite()` 为空的候选半边走到这里）⇒ 加了检查对已通过的门禁格
        // 应当**零影响**。门禁若出现任何 DIFF，说明该路径其实被走到过，必须重查。
        if (edge == nullptr || edge->vertex() == nullptr) {
            return std::numeric_limits<double>::infinity();
        }
        MCGAL::Halfedge* const opp = edge->opposite();
        if (opp == nullptr || opp->vertex() == nullptr) {
            return std::numeric_limits<double>::infinity();
        }
        MCGAL::Vertex* v0 = edge->vertex();
        MCGAL::Vertex* v1 = opp->vertex();
        SymetricMatrix q = accessor_(v0) + accessor_(v1);
        const double det = q.Det(0, 1, 2, 1, 4, 5, 2, 5, 7);
        auto vertexError = [&](const SymetricMatrix& qq, double x, double y, double z) {
            return qq[0] * x * x + 2 * qq[1] * x * y + 2 * qq[2] * x * z + 2 * qq[3] * x
                 + qq[4] * y * y + 2 * qq[5] * y * z + 2 * qq[6] * y
                 + qq[7] * z * z + 2 * qq[8] * z + qq[9];
        };
        if (std::fabs(det) > 1e-6) {
            target[0] = -1.0 / det * q.Det(1, 2, 3, 4, 5, 6, 5, 7, 8);
            target[1] =  1.0 / det * q.Det(0, 2, 3, 1, 5, 6, 2, 7, 8);
            target[2] = -1.0 / det * q.Det(0, 1, 3, 1, 4, 6, 2, 5, 8);
            return vertexError(q, target[0], target[1], target[2]);
        }
        // Degenerate: pick endpoint / midpoint with the smallest error.
        MCGAL::Point pt0 = v0->point();
        MCGAL::Point pt1 = v1->point();
        MCGAL::Point pt2 = (pt0 + pt1) / 2;
        const double e0 = vertexError(q, pt0[0], pt0[1], pt0[2]);
        const double e1 = vertexError(q, pt1[0], pt1[1], pt1[2]);
        const double e2 = vertexError(q, pt2[0], pt2[1], pt2[2]);
        double best = std::min(e0, std::min(e1, e2));
        if (e0 == best) target = pt0;
        if (e1 == best) target = pt1;
        if (e2 == best) target = pt2;
        return best;
    }

    void onAfterEliminate(MCGAL::Halfedge* collapsed) override {
        if (refresher_ && collapsed != nullptr) {
            refresher_(collapsed->vertex());
        }
    }

private:
    SymetricMatrixAccessor accessor_;
    SymetricMatrixRefresher refresher_;
};

}  // namespace pmsf

#endif  // PMSF_METRIC_OPTIMAL_PLACEMENT_QUADRIC_ERROR_H
