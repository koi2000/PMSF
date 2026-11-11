/*
 * @Author: koi
 * @Date: 2025-08-29 10:38:31
 * @Description: Manifold check for interior (non-boundary) vertices.
 *               Rejects boundary vertices, delegates core logic to
 *               UnifiedManifoldIsRemovableOperator::checkManifoldViolation.
 */
#include "ManifoldIsRemovableOperator.h"
#include "common/BoundedRingTraversal.h"
#include "UnifiedManifoldIsRemovableOperator.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Vertex.h"

bool ManifoldIsRemovableOperator::isRemovable(HalfedgeErrorEntry& entry) {
    MCGAL::Halfedge* h = entry.edge;
    MCGAL::Vertex* v = h->end_vertex();

    // 热路径：优先读缓存的 degree；失效时才重算。
    int deg = v->is_degree_dirty() ? v->vertex_degree() : v->vertex_degree_cached();
    if (v->isConquered() || deg <= 2 || deg >= 8)
        return false;

    std::vector<MCGAL::Vertex*> polyVertices;
    polyVertices.reserve(deg * 2);

    for (MCGAL::Halfedge* hit : pmsf::boundedRing(v)) {
        if (hit->isBoundary() || hit->opposite()->isBoundary()) {
            return false;
        }
        // Walk face boundary collecting all non-center vertices.
        // Supports triangular and polygonal faces.
        MCGAL::Halfedge* edge = hit->next();
        while (edge->end_vertex() != v) {
            polyVertices.push_back(edge->vertex());
            edge = edge->next();
        }
    }

    return !UnifiedManifoldIsRemovableOperator::checkManifoldViolation(v, polyVertices, true);
}
