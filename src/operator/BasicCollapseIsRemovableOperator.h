/*
 * @Author: koi
 * @Date: 2025-08-28 21:51:09
 * @Description:
 */
#pragma once

#include "Halfedge.h"
#include "Vertex.h"
#include "operator/IsRemovableOperator.h"

class BasicCollapseIsRemovableOperator : public IsRemovableOperator {
  public:
  BasicCollapseIsRemovableOperator() = default;

    bool isRemovable(HalfedgeErrorEntry& entry);
};
