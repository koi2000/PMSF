/*
 * Phase 2 重构：历史上 MMSE/RD/VQMMC 三份副本字节相同（md5 相等），
 * 已上提到 src/operator/Level2DegreeNIsRemovableOperator.h。
 * 本文件保留为 thin shim，让 impl/xxx main 不用改 include 路径。
 */
#pragma once
#include "operator/Level2DegreeNIsRemovableOperator.h"

class DegreeThreeIsRemovableOperator : public pmsf::Level2DegreeThreeIsRemovableOperator {
  public:
    using Level2DegreeThreeIsRemovableOperator::Level2DegreeThreeIsRemovableOperator;
};
