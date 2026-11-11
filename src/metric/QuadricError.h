/*
 * @file QuadricError.h
 * @brief Garland–Heckbert quadric error metric.
 *
 * Reproduces QEM's hot-path cost computation as a reusable framework hook.
 * The SymetricMatrix storage convention matches impl/QEM/QEMCompress.cpp
 * (stored on vertices as vpQuadrics via PropertyHandleWrapper); the accessor
 * is injected through a user-supplied lambda so this header does not pull
 * in any algorithm-specific attribute.
 *
 * Phase 10 addition. Not hooked into the existing QEM code path — it
 * parallels the current computation so that a Phase-13 rewrite of QEM can
 * swap to this hook with identical output.
 */
#ifndef PMSF_METRIC_QUADRIC_ERROR_H
#define PMSF_METRIC_QUADRIC_ERROR_H

#include "ErrorMetric.h"
#include "Point.h"

#include <array>
#include <functional>

namespace pmsf {

/// 4×4 symmetric quadric stored as 10 doubles (a..j), encoding the plane
/// fundamental. Q(v) = v^T M v with v = (x, y, z, 1).
struct Quadric {
    std::array<double, 10> m{};

    /// Q(p) = p^T M p for p homogeneous.
    double evaluate(const MCGAL::Point& p) const {
        const double x = p.x(), y = p.y(), z = p.z();
        // Matching the classical expansion in Garland–Heckbert 1997.
        return m[0] * x * x + 2 * m[1] * x * y + 2 * m[2] * x * z + 2 * m[3] * x
             + m[4] * y * y + 2 * m[5] * y * z + 2 * m[6] * y
             + m[7] * z * z + 2 * m[8] * z
             + m[9];
    }

    Quadric operator+(const Quadric& o) const {
        Quadric r;
        for (size_t i = 0; i < 10; ++i) r.m[i] = m[i] + o.m[i];
        return r;
    }
};

/// Injectable accessor: given a vertex, return the quadric attached to it.
/// Algorithms supply a lambda that reads from their vpQuadrics property.
using QuadricAccessor = std::function<const Quadric&(MCGAL::Vertex*)>;

class QuadricError final : public ErrorMetric {
public:
    explicit QuadricError(QuadricAccessor accessor)
        : accessor_(std::move(accessor)) {}

    double cost(MCGAL::Halfedge* h) const override {
        const Quadric& qa = accessor_(h->vertex());
        const Quadric& qb = accessor_(h->opposite()->vertex());
        const Quadric qsum = qa + qb;
        // Classical QEM: evaluate sum-quadric at the midpoint. A full
        // implementation would solve ∂Q/∂v = 0 for the optimal placement;
        // the midpoint form matches impl/QEM's historical choice and keeps
        // bpv/error identical after migration.
        const MCGAL::Point mid(
            0.5f * (h->vertex()->point().x() + h->opposite()->vertex()->point().x()),
            0.5f * (h->vertex()->point().y() + h->opposite()->vertex()->point().y()),
            0.5f * (h->vertex()->point().z() + h->opposite()->vertex()->point().z()));
        return qsum.evaluate(mid);
    }

private:
    QuadricAccessor accessor_;
};

}  // namespace pmsf

#endif  // PMSF_METRIC_QUADRIC_ERROR_H
