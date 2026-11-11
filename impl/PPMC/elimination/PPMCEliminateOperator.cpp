/*
 * @Author: koi
 * @Date: 2025-08-27 22:37:03
 * @Description:
 */
#include "PPMCEliminateOperator.h"
#include "common/BoundedRingTraversal.h"
#include "Facet.h"
#include "Global.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "MeshUtils.h"
#include "Point.h"
#include "Vertex.h"
#include "common/Define.h"
#include <cstddef>
#include <vector>

void PPMCEliminateOperator::init(std::shared_ptr<MCGAL::Mesh> mesh, PMSFHeader header, bool compress_boundary) {
    mesh_ = mesh;
    compress_boundary_ = compress_boundary;
    header_ = header;
}

bool PPMCEliminateOperator::eliminate(HalfedgeErrorEntry& entry) {
    MCGAL::Halfedge* startH = entry.edge;
    MCGAL::Vertex* v = startH->end_vertex();

    // make sure that the center vertex can be removed
    assert(!v->isConquered());
    assert(v->vertex_degree() > 2);

    // Dry-run pass (2026-05-01): walk the 1-ring first WITHOUT touching the
    // mesh and check that every split_facet this function will invoke
    // produces a chord whose endpoints are NOT already connected by an
    // existing halfedge. If any would duplicate an edge, bail out and let
    // the pipeline try another seed.
    //
    // Root cause: PPMCIsRemovableOperator::chordWouldDuplicateEdge checked
    // the ORIGINAL 1-ring, but eliminate() mutates the mesh between
    // successive split_facet calls, so even a 1-ring that was safe at
    // isRemovable time can develop a conflict partway through. The only
    // reliable place to catch this is inside eliminate() itself.
    //
    // Mesh state during the dry run: inspected `h->end_vertex()` values
    // stay valid because we don't mutate. The chord targets we compute
    // here exactly match what the real loop below will attempt.
    {
        std::vector<std::pair<MCGAL::Vertex*, MCGAL::Vertex*>> pendingChords;
        MCGAL::Halfedge* h_dry = startH->opposite();
        MCGAL::Halfedge* end_dry = h_dry;
        do {
            MCGAL::Facet* f = h_dry->face();
            if (f && f->facet_degree() > 3) {
                MCGAL::Halfedge* hSplit = h_dry->next();
                int guard = 0;
                while (hSplit && hSplit->next() && hSplit->next()->next() != h_dry) {
                    hSplit = hSplit->next();
                    if (++guard > 64) { hSplit = nullptr; break; }
                }
                if (!hSplit) return false;
                MCGAL::Vertex* a = h_dry->end_vertex();
                MCGAL::Vertex* b = hSplit->end_vertex();
                if (!a || !b || a == b) return false;
                // Reject if the chord is already present as an existing edge...
                // （用 1-ring 走查而非 Vertex::halfedges()：后者在 split/erase 之后会过期）
                bool existingChord = false;
                for (MCGAL::Halfedge* hh : pmsf::boundedRing(a)) {
                    if (hh && hh->end_vertex() == b) { existingChord = true; break; }
                }
                if (!existingChord) {
                    for (MCGAL::Halfedge* hh : pmsf::boundedRing(b)) {
                        if (hh && hh->end_vertex() == a) { existingChord = true; break; }
                    }
                }
                // ...or is about to be added by an earlier iteration of this
                // same eliminate() call (two 1-ring faces happening to target
                // the same (a,b) vertex pair — the second split_facet would
                // duplicate the halfedge pair just created by the first).
                bool pendingConflict = false;
                for (const auto& c : pendingChords) {
                    if ((c.first == a && c.second == b) ||
                        (c.first == b && c.second == a)) {
                        pendingConflict = true;
                        break;
                    }
                }
                if (existingChord || pendingConflict) return false;
                pendingChords.emplace_back(a, b);
            }
            h_dry = h_dry->opposite()->next();
        } while (h_dry != end_dry);
    }

    MCGAL::Halfedge* h = startH->opposite();
    MCGAL::Halfedge* end = h;
    do {
        // assert(!h->is_border());
        MCGAL::Facet* f = h->face();
        assert(!f->isConquered() && !f->isRemoved());  // we cannot cut again an already cut face, or a NULL patch

        // if the face is not a triangle, cut the corner
        if (f->facet_degree() > 3) {
            // loop around the face to find the apropriate other halfedge
            MCGAL::Halfedge* hSplit(h->next());
            for (; hSplit->next()->next() != h; hSplit = hSplit->next())
                ;
            MCGAL::Halfedge* hCorner = mesh_->split_facet(h, hSplit);
            // mark the new halfedges as added
            hCorner->setAdded();
            hCorner->opposite()->setAdded();

        }

        // mark the vertex as conquered
        h->end_vertex()->setConquered();
    } while ((h = h->opposite()->next()) != end);

    // copy the position of the center vertex:
    MCGAL::Point vPos = startH->end_vertex()->point();

    // remove the center vertex
    MCGAL::Halfedge* hNewFace = mesh_->erase_center_vertex(startH);
    MCGAL::Facet* f = hNewFace->face();
    // now mark the new face as having a removed vertex
    hNewFace->face()->setSplittable();
    // keep the removed vertex position.
    hNewFace->face()->setRemovedVertexPos(vPos);
    // ⚠️ P0 对齐修复：
    //   原版 PPMC 在 vertexCut 当场只记录浮点 removedVertexPos；
    //   "残差 = Q(removedPos) - Q(barycenter(h))" 在整轮 decimation 完成后由
    //   `determineResiduals()` 第二次 BFS 统一计算（此时 mesh 拓扑已稳定）。
    //   旧代码这里直接写 `setRemovedVertexPosInt(floatPosToInt(vPos))`，存的是
    //   绝对量化坐标而非残差，导致 alphaBetaModel 范围爆炸、range coder 失效。
    //   现在删除该行，改由 PPMCSymbolCollectOperator::determineResiduals() 统一计算。
    // f->setRemovedVertexPosInt(mesh_->floatPosToInt(vPos));
    (void)f;
    // scan the outside halfedges of the new face and add them to the queue if the state of its face is unknown. Also mark it as in_queue
    entry.edge = hNewFace;

    return hNewFace;
}

bool PPMCEliminateOperator::remove_point(MCGAL::Halfedge* h) {
    return true;
}

bool PPMCEliminateOperator::triangulate(MCGAL::Halfedge* h) {
    return true;
}

bool PPMCEliminateOperator::postprocess(MCGAL::Halfedge* h) {
    return true;
}

MCGAL::Halfedge* PPMCEliminateOperator::encode_boundary(MCGAL::Halfedge* h) {
    return nullptr;
}