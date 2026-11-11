/*
 * @Author: koi
 * @Date: 2025-08-28 21:51:18
 * @Description:
 */

#include "BasicCollapseIsRemovableOperator.h"
#include "Facet.h"
#include "Halfedge.h"
#include "MeshUtils.h"

bool BasicCollapseIsRemovableOperator::isRemovable(HalfedgeErrorEntry& entry) {
    MCGAL::Halfedge* halfedge = entry.edge;
    if (!halfedge->b_collapseOk || halfedge->isRemoved() || halfedge->vertex()->isRemoved() || halfedge->end_vertex()->isRemoved()) {
        return false;
    }
    if(halfedge->isBoundary()) {
        return false;
    }
    // if(!MCGAL::new_is_collapse_ok(halfedge)) {
    //     halfedge->b_collapseOk = false;
    //     return false;
    // }
    return true;
}
