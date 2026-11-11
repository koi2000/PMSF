// EdgeBreakerMeshAdapter.h — turns a live MCGAL base mesh into the vertex/face
// arrays consumed by the native EdgeBreaker codec, with a DETERMINISTIC face
// ordering so the encoder (base connectivity blob) and the segmented facet->
// partition channel builder derive the SAME EB traversal order.
//
// The ordering is a pure function of the mesh: live faces in container order,
// vertex ids by first-encounter while walking each face's proxy halfedge. The
// EB traversal order (visit order) is then intrinsic to the connectivity and
// start corner 0, independent of vertex-id VALUES, so two independent callers
// on the same mesh obtain identical visit orders.
#ifndef PMSF_EDGE_BREAKER_MESH_ADAPTER_H
#define PMSF_EDGE_BREAKER_MESH_ADAPTER_H

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "EdgeBreakerCodec.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Vertex.h"

namespace pmsf_eb {

struct EbMeshPlan {
    std::vector<Vec3i> verts;                 // vertex id -> quantized coord
    std::vector<Tri> faces;                   // container order; ids first-seen
    std::vector<MCGAL::Facet*> facesInOrder;  // container index -> Facet*
    bool ok = false;
    bool allTriangles = true;
};

// baseMeshQuantBits: clamp bound for coords (match ColumnarSerializeOperator).
// wantCoords=false skips coordinate computation (channel builder only needs
// topology to derive the visit order).
inline EbMeshPlan buildEbPlan(MCGAL::Mesh* mesh, int baseMeshQuantBits,
                              bool wantCoords) {
    EbMeshPlan plan;
    if (!mesh) return plan;
    auto clampq = [&](std::int32_t v) -> std::int32_t {
        if (v < 0) return 0;
        if (baseMeshQuantBits > 0 && baseMeshQuantBits < 31) {
            const std::int32_t maxV = (1 << baseMeshQuantBits) - 1;
            if (v > maxV) return maxV;
        }
        return v;
    };
    std::unordered_map<MCGAL::Vertex*, int> vid;
    vid.reserve(mesh->faces().size() * 2 + 8);
    auto idOf = [&](MCGAL::Vertex* v) -> int {
        auto it = vid.find(v);
        if (it != vid.end()) return it->second;
        const int id = static_cast<int>(plan.verts.size());
        vid.emplace(v, id);
        if (wantCoords) {
            MCGAL::PointInt pi = mesh->floatPosToInt(v->point());
            plan.verts.push_back(Vec3i{clampq(pi[0]), clampq(pi[1]), clampq(pi[2])});
        } else {
            plan.verts.push_back(Vec3i{0, 0, 0});
        }
        return id;
    };
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        MCGAL::Halfedge* st = f->proxyHalfedge();
        MCGAL::Halfedge* ed = st;
        int ids[3];
        int d = 0;
        bool bad = false;
        do {
            if (d >= 3) { bad = true; break; }
            MCGAL::Vertex* vtx = st->vertex();
            if (!vtx) { bad = true; break; }
            ids[d++] = idOf(vtx);
            st = st->next();
        } while (st != ed);
        if (bad || d != 3) { plan.allTriangles = false; return plan; }
        plan.faces.push_back(Tri{ids[0], ids[1], ids[2]});
        plan.facesInOrder.push_back(f);
    }
    plan.ok = !plan.faces.empty();
    return plan;
}

}  // namespace pmsf_eb
#endif  // PMSF_EDGE_BREAKER_MESH_ADAPTER_H
