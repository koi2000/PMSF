/*
 * @file QEMQuadric.h
 * @brief QEM attribute-registration glue. The optimal-placement math
 *        (calculateQuadricError + onAfterEliminate) has been promoted to
 *        src/metric/OptimalPlacementQuadricError.h as a framework-level
 *        hook; this header now owns only the ContextPool attribute
 *        bootstrap (`registerQuadricProperties`) which references the
 *        OpenMesh-style `VPropHandleT<SymetricMatrix>` handle bundle.
 *
 * The retained helper is paper-specific only in its choice of attribute
 * names and initialisation policy (accumulate face quadrics, then
 * precompute per-halfedge costs). The math itself is the framework's.
 *
 * Byte-identical with the pre-migration impl/QEM/QEMCompress.cpp because
 * the register helper calls into pmsf::OptimalPlacementQuadricError via
 * the same determinant code path.
 */
#ifndef PMSF_IMPL_QEM_QEM_QUADRIC_H
#define PMSF_IMPL_QEM_QEM_QUADRIC_H

#include "ContextPool.h"
#include "common/BoundedRingTraversal.h"
#include "Facet.h"
#include "Global.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Vertex.h"
#include "metric/OptimalPlacementQuadricError.h"
#include "symetric_matrix.h"

#include <memory>

namespace qem_impl {

/// Bundle of per-mesh property handles used across the QEM compress
/// pipeline. Declared at the caller's scope; passed by reference.
struct PropertyHandles {
    OpenMesh::VPropHandleT<SymetricMatrix> vpQuadrics;
    OpenMesh::HPropHandleT<double> epError;
    OpenMesh::HPropHandleT<MCGAL::Point> epTargetPoints;
    OpenMesh::HPropHandleT<bool> epDirty;
};

/// Accessor lambda factory — returns the per-vertex SymetricMatrix from
/// the ContextPool attribute. Passed to the framework
/// `OptimalPlacementQuadricError` hook.
inline pmsf::SymetricMatrixAccessor
makeQuadricAccessor(PropertyHandles& h, int meshId) {
    return [&h, meshId](MCGAL::Vertex* v) -> SymetricMatrix& {
        return MCGAL::contextPool.property(meshId, h.vpQuadrics, v);
    };
}

/// Refresher lambda factory — recomputes the quadric on the survivor and
/// its 1-ring after a collapse. Mirrors the historical
/// impl/QEM/QEMCompress.cpp onAfterEliminate closure byte-for-byte.
inline pmsf::SymetricMatrixRefresher
makeQuadricRefresher(PropertyHandles& h, int meshId) {
    return [&h, meshId](MCGAL::Vertex* survivor) {
        if (survivor == nullptr || survivor->isRemoved()) return;
        auto recomputeQ = [&](MCGAL::Vertex* v) {
            if (v == nullptr || v->isRemoved()) return;
            SymetricMatrix& q = MCGAL::contextPool.property(meshId, h.vpQuadrics, v);
            q.Clear();
            for (MCGAL::Halfedge* he : pmsf::boundedRing(v)) {
                if (he == nullptr || he->isRemoved()) continue;
                MCGAL::Facet* f = he->face();
                if (f == nullptr || f->isRemoved()) continue;
                MCGAL::Vector3 n = f->computeNormal();
                double a = n.x(), b = n.y(), c = n.z();
                double d = -(v->point().dot(n));
                q += SymetricMatrix(a, b, c, d);
            }
        };
        recomputeQ(survivor);
        for (MCGAL::Halfedge* he : pmsf::boundedRing(survivor)) {
            if (he == nullptr || he->isRemoved()) continue;
            recomputeQ(he->end_vertex());
        }
    };
}

/// Per-mesh registration: attach the property handles, zero per-vertex
/// quadrics, accumulate face quadrics, then precompute per-halfedge errors
/// via the framework hook. Byte-identical with the pre-promotion code.
inline void registerQuadricProperties(PropertyHandles& h,
                                      std::shared_ptr<MCGAL::Mesh> mesh) {
    const int meshId = mesh->meshId();
    MCGAL::contextPool.add_property(meshId, h.vpQuadrics);
    MCGAL::contextPool.add_property(meshId, h.epError);
    MCGAL::contextPool.add_property(meshId, h.epTargetPoints);
    MCGAL::contextPool.add_property(meshId, h.epDirty);
    for (MCGAL::Vertex* vertex : mesh->vertices())
        MCGAL::contextPool.property(meshId, h.vpQuadrics, vertex).Clear();
    for (MCGAL::Facet* facet : mesh->faces()) {
        MCGAL::Vertex* vh0 = facet->proxyHalfedge()->vertex();
        MCGAL::Vertex* vh1 = facet->proxyHalfedge()->next()->vertex();
        MCGAL::Vertex* vh2 = facet->proxyHalfedge()->next()->next()->vertex();
        const auto& n = facet->computeNormal();
        SymetricMatrix q(n.x(), n.y(), n.z(), -(vh0->point().dot(n)));
        MCGAL::contextPool.property(meshId, h.vpQuadrics, vh0) += q;
        MCGAL::contextPool.property(meshId, h.vpQuadrics, vh1) += q;
        MCGAL::contextPool.property(meshId, h.vpQuadrics, vh2) += q;
    }
    // Seed per-halfedge error/target caches via the framework hook so we
    // get the exact same numbers the legacy inline code produced.
    pmsf::OptimalPlacementQuadricError metric(makeQuadricAccessor(h, meshId));
    MCGAL::Point ptResult;
    for (MCGAL::Facet* face : mesh->faces()) {
        MCGAL::Halfedge* st = face->proxyHalfedge();
        MCGAL::Halfedge* ed = st;
        do {
            const double dError = metric.costWithTarget(st, ptResult);
            MCGAL::contextPool.property(meshId, h.epError, st) = dError;
            MCGAL::contextPool.property(meshId, h.epTargetPoints, st) = ptResult;
            st = st->next();
        } while (st != ed);
    }
}

}  // namespace qem_impl

#endif  // PMSF_IMPL_QEM_QEM_QUADRIC_H
