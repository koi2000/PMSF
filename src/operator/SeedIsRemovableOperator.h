/*
 * @Author: koi
 * @Date: 2025-09-03 21:29:50
 * @Description:
 */
/*
 * @Author: koi
 * @Date: 2025-08-25 22:29:32
 * @Description:
 */
#ifndef SEED_ISREOVABLEOPERATOR_H
#define SEED_ISREOVABLEOPERATOR_H

#include "Halfedge.h"
#include "Vertex.h"
#include "operator/IsRemovableOperator.h"
#include <cstddef>
#include <vector>

class SeedIsRemovableOperator : public IsRemovableOperator {
  public:
    SeedIsRemovableOperator() = default;

    void init(std::vector<MCGAL::Vertex*> v, std::vector<MCGAL::Halfedge*> h, std::vector<MCGAL::Facet*> f) {
        this->seeds = h;
    };

    bool isRemovable(HalfedgeErrorEntry& entry) {
        MCGAL::Halfedge* halfedge = entry.edge;
        for (MCGAL::Halfedge* seed : seeds) {
            if (seed->vertex()->poolId() == halfedge->vertex()->poolId() || seed->end_vertex()->poolId() == halfedge->end_vertex()->poolId() ||
                seed->end_vertex()->poolId() == halfedge->vertex()->poolId() || seed->vertex()->poolId() == halfedge->end_vertex()->poolId()) {
                return false;
            }
            // if (seed->vertex()->poolId() == halfedge->vertex()->poolId() && seed->end_vertex()->poolId() == halfedge->end_vertex()->poolId()) {
            //     return false;
            // }
        }
        return true;
    };

    std::vector<MCGAL::Halfedge*> seeds;
};

#endif