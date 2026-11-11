#pragma once

#include "ContextPool.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Vertex.h"

#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

namespace pmsf {

struct SeamCutResult {
    std::shared_ptr<MCGAL::Mesh> mesh;
    std::unordered_map<int, int> originOfPoolId;  // seam vertex poolId -> original poolId
    std::vector<MCGAL::Halfedge*> seeds;
    int groupCount = 0;
};

/// Split a facet-group-annotated mesh into independent per-group components by
/// duplicating vertices on group boundaries.  Every original face appears once;
/// each group copy references its own vertex copies.  originOfPoolId lets the
/// decoder merge surviving seam copies back by original vertex id.
class SeamCutOperator {
  public:
    static SeamCutResult run(const std::shared_ptr<MCGAL::Mesh>& input,
                             const std::vector<MCGAL::Halfedge*>& seeds);
};

class SeamMergeOperator {
  public:
    /// Merge a seam-cut mesh back into one mesh.  Vertices present in originOfPoolId
    /// are deduplicated by origin id; vertices added by reconstruction (not in the
    /// map) stay unique.
    static std::shared_ptr<MCGAL::Mesh> run(
        const std::shared_ptr<MCGAL::Mesh>& seamMesh,
        const std::unordered_map<int, int>& originOfPoolId);

    /// Position-based fallback merge (no origin ids): merge boundary vertices that
    /// have identical quantized positions. Used by the Phase-2 seam-cut proof of
    /// concept before origin-id serialization exists.
    static std::shared_ptr<MCGAL::Mesh> mergeByPosition(
        const std::shared_ptr<MCGAL::Mesh>& seamMesh,
        bool exactFloat = false);
};

}  // namespace pmsf
