/*
 * @file RDErrorMetric.h
 * @brief RD's tag-based 6-level priority error metric (Phase 13b extract).
 *
 * Extracted verbatim from impl/RD/RDCompress.cpp:20-58. Byte-identical
 * semantics preserved: same VertexTag / PLUS / MINUS enum, same 6-level
 * priority ladder keyed on (tag, incident-in-processed) pairs.
 *
 * Phase 13b+ addition: `buildSelectOptions()` factory that assembles the
 * SelectOptions::Builder call chain (cost functor, property-registration
 * lambda, the four IsRemovable operators). This is the fully mechanical
 * boilerplate — the same Builder chain every paper's compress main writes —
 * so moving it out of the RD main does not hide algorithm semantics, just
 * shifts declaration. Output byte-identical.
 *
 * The helper takes property handles by reference so the main's lambda
 * closures still bind to the same storage location used for
 * `withRegisterProperties` → `calculateError`.
 */
#ifndef PMSF_IMPL_RD_RD_ERROR_METRIC_H
#define PMSF_IMPL_RD_RD_ERROR_METRIC_H

#include "ContextPool.h"
#include "common/BoundedRingTraversal.h"
#include "DegreeSixIsRemovableOperator.h"
#include "DegreeThreeIsRemovableOperator.h"
#include "Global.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Point.h"
#include "Vertex.h"
#include "core/HookBridge.h"
#include "metric/ErrorMetric.h"
#include "operator/BasicIsRemovableOperator.h"
#include "operator/UnifiedManifoldIsRemovableOperator.h"
#include "options/SelectOptions.h"
#include "selection/ErrorSource.h"

#include <memory>

namespace rd_impl {

enum VertexTag { UNTAGGED = 0, PLUS = 1, MINUS = 2 };

struct PropertyHandles {
    OpenMesh::VPropHandleT<int> vpTag;
    OpenMesh::HPropHandleT<double> epError;
};

inline int countIncidentEdgesInProcessed(MCGAL::Vertex* v) {
    int count = 0;
    for (MCGAL::Halfedge* hit : pmsf::boundedRing(v))
        if (hit->face()->isProcessed()) count++;
    return count;
}

inline double calculateError(const PropertyHandles& h, MCGAL::Halfedge* he) {
    MCGAL::Vertex* v_from = he->vertex();
    MCGAL::Vertex* v_to = he->next()->vertex();
    MCGAL::Vertex* v_front = he->next()->next()->vertex();
    bool from_null = false;
    int tag_from = MCGAL::contextPool.property(h.vpTag, v_from);
    int tag_to = MCGAL::contextPool.property(h.vpTag, v_to);
    int incident_front = countIncidentEdgesInProcessed(v_front);
    bool has_low_valence = (incident_front == 2 || incident_front == 3);
    if (tag_from == PLUS && tag_to == MINUS && !from_null && has_low_valence) return 6;
    if (tag_from == MINUS && tag_to == PLUS && !from_null && has_low_valence) return 6;
    if (tag_from == PLUS && tag_to == MINUS && from_null && has_low_valence) return 5;
    if (tag_from == MINUS && tag_to == PLUS && from_null && has_low_valence) return 5;
    if ((tag_from == MINUS && tag_to == MINUS) || (tag_from == PLUS && tag_to == PLUS)) {
        int incident_from = countIncidentEdgesInProcessed(v_from);
        int incident_to = countIncidentEdgesInProcessed(v_to);
        if (!from_null && (incident_from == 2 || incident_from == 3)
            && (incident_to == 2 || incident_to == 3)) return 4;
        if ( from_null && (incident_from == 2 || incident_from == 3)
            && (incident_to == 2 || incident_to == 3)) return 3;
    }
    if (!from_null) return 2;
    return 1;
}

inline void registerProperties(PropertyHandles& h, std::shared_ptr<MCGAL::Mesh> mesh) {
    MCGAL::contextPool.add_property(mesh->meshId(), h.vpTag);
    MCGAL::contextPool.add_property(mesh->meshId(), h.epError);
    for (MCGAL::Vertex* v : mesh->vertices())
        MCGAL::contextPool.property(mesh->meshId(), h.vpTag, v) = UNTAGGED;
}

/// Phase-13b+ Task-1: RD 的 tag-priority 作为 pmsf::ErrorMetric 子类对外暴露，
/// 让 RDCompress.cpp 通过 Builder::withErrorMetric 流程注入算法语义。
/// cost() 与 registerProperties/calculateError 完全同源。
class RDErrorMetricImpl final : public pmsf::ErrorMetric {
public:
    explicit RDErrorMetricImpl(PropertyHandles& h) : h_(h) {}
    double cost(MCGAL::Halfedge* he) const override { return calculateError(h_, he); }
private:
    PropertyHandles& h_;
};

/// Phase 13b+: full SelectOptions assembly. The caller owns `h` (the
/// closures below capture it by reference), so its lifetime must cover
/// the pipeline run.
///
/// Session 3 update: the cost-accessor lambda is now produced by the
/// framework `makeHalfedgeErrorAccessor(metric)` bridge instead of being
/// hand-written — same output, centralises the glue.
inline std::shared_ptr<SelectOptions> buildSelectOptions(
    PropertyHandles& h,
    std::shared_ptr<pmsf::ErrorMetric> metric = nullptr) {
    if (!metric) metric = std::make_shared<RDErrorMetricImpl>(h);
    return SelectOptions::Builder()
        .withErrorSource(ErrorSource::Halfedge)
        .withHalfedgeErrorAccessor(pmsf::makeHalfedgeErrorAccessor(metric))
        .withRegisterProperties([&h](std::shared_ptr<MCGAL::Mesh> m) {
            registerProperties(h, m); })
        .withIsRemovableOperator(std::make_shared<BasicIsRemovableOperator>())
        .withIsRemovableOperator(std::make_shared<UnifiedManifoldIsRemovableOperator>())
        .withIsRemovableOperator(std::make_shared<DegreeThreeIsRemovableOperator>())
        .withIsRemovableOperator(std::make_shared<DegreeSixIsRemovableOperator>())
        .build();
}

}  // namespace rd_impl

#endif  // PMSF_IMPL_RD_RD_ERROR_METRIC_H
