/*
 * @file BFSTraversal.h
 * @brief Deterministic BFS driver shared by compress collectors and decompress readers.
 *
 * Before Phase 9 every SymbolCollectOperator/SymbolReadOperator implemented
 * its own BFS walk over half-edges keyed off MCGAL::BFSVersionManager. The
 * walks were almost but not quite identical, creating a latent risk that the
 * compress-side and decompress-side orders drift out of sync and silently
 * corrupt the bitstream.
 *
 * This driver centralizes the walk. A visitor callback decides what (if
 * anything) to record per visited halfedge / facet / vertex. The three
 * canonical walks used by the existing Basic collector (facet-pass,
 * edge-pass, vertex-pass) are exposed as named helpers so that the refactor
 * to make BasicSymbolCollectOperator a thin visitor is mechanical and
 * byte-identical — enforced by the Phase 9 regression gate.
 *
 * Zero allocations on the hot path: the caller provides a reusable
 * GateQueue. Version ids are consumed from MCGAL::BFSVersionManager exactly
 * like the pre-refactor code.
 */
#ifndef PMSF_CORE_BFS_TRAVERSAL_H
#define PMSF_CORE_BFS_TRAVERSAL_H

#include "BFSVersionManager.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Vertex.h"

#include <functional>
#include <memory>
#include <queue>

namespace pmsf {

/// Reusable gate queue. Each driver call begins with the seed's poolId and
/// fans out through opposite()/next() — identical to the pre-Phase-9 code.
using GateQueue = std::queue<int>;

/// Visitor interface: the driver calls these as it visits entities. Returning
/// false is never a short-circuit — the BFS always completes — but it is a
/// hook for the visitor to record a per-entity decision (e.g. "not splittable
/// on this facet" → the collector pushes a '0' symbol).
///
/// Only the methods the visitor actually needs should be overridden; the
/// defaults are no-ops.
struct BFSVisitor {
    virtual ~BFSVisitor() = default;

    /// Called once per halfedge the first time it is visited under the
    /// current version. Guaranteed: !h->isRemoved() && !h->isVisited(version)
    /// before the call; h->setVisited(version) is performed by the driver
    /// *after* this returns so visitor code may inspect visited state on
    /// neighbours.
    virtual void onHalfedge(MCGAL::Halfedge* /*h*/, int /*version*/) {}

    /// Called the first time this facet is seen under the current version.
    virtual void onFacetFirstVisit(MCGAL::Facet* /*f*/, int /*version*/) {}

    /// Called the first time this vertex is seen under the current version.
    virtual void onVertexFirstVisit(MCGAL::Vertex* /*v*/, int /*version*/) {}
};

/// Run one BFS pass over the mesh starting from `seed`. Uses a fresh
/// BFSVersionManager tag (consuming one version from the global counter),
/// exactly like the legacy collectors do. Returns the version id used so
/// visitors that want to re-probe can do so.
///
/// The visitor may record arbitrary state; the driver itself is stateless.
int runBFS(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Halfedge* seed, BFSVisitor& visitor);

/// Three canonical passes used by the fixed-shape S_r emission. These are
/// exactly the three loops inlined in BasicSymbolCollectOperator::collect;
/// centralising them lets every future collector reuse them and guarantees
/// compress/decompress walk-order parity.
///
/// Each pass calls the provided per-entity callback during the entity's
/// first visit. Callbacks may push into RoundSymbols or a side-channel.

/// Facet pass: for every facet seen under this walk, invoke `onFacet`.
/// Historically this is where F_r and P_r (inserted-point positions) are
/// emitted. The walk order is deterministic and stable across compress /
/// decompress so long as both sides use the same helper.
void runFacetPass(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Halfedge* seed,
                  std::function<void(MCGAL::Facet*)> onFacet);

/// Edge pass: for every halfedge seen under this walk (one per halfedge,
/// twice per geometric edge — matches legacy emission), invoke `onHalfedge`.
void runEdgePass(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Halfedge* seed,
                 std::function<void(MCGAL::Halfedge*)> onHalfedge);

/// Vertex pass (only emitted when header.isEnableAdjustment() is true today):
/// for each vertex reached via a halfedge visit, invoke `onVertex` exactly
/// once. Historical V_r / VP_r emission.
void runVertexPass(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Halfedge* seed,
                   std::function<void(MCGAL::Vertex*)> onVertex);

}  // namespace pmsf

#endif  // PMSF_CORE_BFS_TRAVERSAL_H
