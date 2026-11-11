/*
 * Phase 2 重构：同 DegreeThree，统一到 src/operator/Level2DegreeNIsRemovableOperator.h。
 */
#pragma once
#include "operator/Level2DegreeNIsRemovableOperator.h"

class DegreeSixIsRemovableOperator : public pmsf::Level2DegreeSixIsRemovableOperator {
  public:
    using Level2DegreeSixIsRemovableOperator::Level2DegreeSixIsRemovableOperator;
};
