/*
 * @file EdgeLengthError.h
 * @brief Simplest concrete ErrorMetric — squared edge length.
 *
 * Useful as the default for prototyping new algorithms before a paper's
 * real cost is implemented, and as the reference target for unit tests.
 */
#ifndef PMSF_METRIC_EDGE_LENGTH_ERROR_H
#define PMSF_METRIC_EDGE_LENGTH_ERROR_H

#include "ErrorMetric.h"
#include "Point.h"

namespace pmsf {

class EdgeLengthError final : public ErrorMetric {
public:
    double cost(MCGAL::Halfedge* h) const override {
        const MCGAL::Point& a = h->vertex()->point();
        const MCGAL::Point& b = h->opposite()->vertex()->point();
        const float dx = a.x() - b.x();
        const float dy = a.y() - b.y();
        const float dz = a.z() - b.z();
        return static_cast<double>(dx * dx + dy * dy + dz * dz);
    }
};

}  // namespace pmsf

#endif  // PMSF_METRIC_EDGE_LENGTH_ERROR_H
