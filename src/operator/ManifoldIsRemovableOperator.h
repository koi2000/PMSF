/*
 * @Author: koi
 * @Date: 2025-08-28 21:51:09
 * @Description:
 */
#ifndef MANIFOLD_ISREOVABLEOPERATOR_H
#define MANIFOLD_ISREOVABLEOPERATOR_H

#include "Halfedge.h"
#include "Vertex.h"
#include "operator/IsRemovableOperator.h"

class ManifoldIsRemovableOperator : public IsRemovableOperator {
  public:
    ManifoldIsRemovableOperator() = default;

    bool isRemovable(HalfedgeErrorEntry& entry);
};

#endif
