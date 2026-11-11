/*
 * @file PreQuantizePreprocessor.cpp
 * @brief Phase 5：顶点坐标量化 PreProcessOperator 实现。
 */
#include "PreQuantizePreprocessor.h"

#include "Mesh.h"

namespace pmsf {

void PreQuantizePreprocessor::process(std::shared_ptr<MCGAL::Mesh> mesh) {
    if (!mesh) return;
    mesh->quantizeVertexPositions();
}

}  // namespace pmsf
