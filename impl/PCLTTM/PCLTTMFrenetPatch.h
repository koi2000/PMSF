// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "PCLTTM/PCLTTMFrenetGeometry.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Vertex.h"
#include <memory>
#include <unordered_map>
#include <vector>

namespace pmsf::pclttm_frenet {

// Lifetime spans one framework run; collector consumes the elimination data
// in the same order as the existing valence/residual symbols. No facet or
// vertex identities are serialized.
using PendingPatches = std::unordered_map<const MCGAL::Facet*,PatchCode>;

inline Cell toCell(const MCGAL::PointInt& p) { return {p.x(),p.y(),p.z()}; }

inline std::vector<Cell> patchCells(MCGAL::Facet* face,MCGAL::Mesh& mesh,bool restoreAdjustments) {
    auto* start=face?face->proxyHalfedge():nullptr;
    if(!start)throw std::runtime_error("PCLTTM Frenet patch has no boundary");
    auto* edge=start;
    std::vector<Cell> ring;
    do {
        if(!edge||edge->face()!=face||!edge->vertex()||ring.size()==6)
            throw std::runtime_error("PCLTTM Frenet patch is not a closed valence 3–6 polygon");
        auto* vertex=edge->vertex();
        const auto p=restoreAdjustments&&vertex->isAdjusted()?vertex->vertexPoint():vertex->point();
        ring.push_back(toCell(mesh.floatPosToInt(p)));
        edge=edge->next();
    } while(edge!=start);
    return ring;
}
} // namespace pmsf::pclttm_frenet
