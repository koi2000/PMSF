/*
 * @Author: koi
 * @Date: 2025-08-28 21:51:09
 * @Description:
 */
#ifndef BASIC_ISREOVABLEOPERATOR_H
#define BASIC_ISREOVABLEOPERATOR_H

#include "Halfedge.h"
#include "Vertex.h"
#include "operator/IsRemovableOperator.h"

class BasicIsRemovableOperator : public IsRemovableOperator {
  public:
    BasicIsRemovableOperator() = default;

    bool isRemovable(HalfedgeErrorEntry& entry);
};

#endif