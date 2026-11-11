/*
 * @Author: koi
 * @Date: 2025-08-28 21:51:09
 * @Description:
 */
#ifndef UNIFIEDMANIFOLD_ISREOVABLEOPERATOR_H
#define UNIFIEDMANIFOLD_ISREOVABLEOPERATOR_H

#include "Halfedge.h"
#include "Vertex.h"
#include "operator/IsRemovableOperator.h"
#include <vector>

class UnifiedManifoldIsRemovableOperator : public IsRemovableOperator {
  public:
    UnifiedManifoldIsRemovableOperator() = default;

    bool isRemovable(HalfedgeErrorEntry& entry);

    /**
     * @param center    the vertex to be removed
     * @param poly      boundary vertices of the resulting polygon, in traversal order
     * @param isClosed  true for interior vertices (closed cycle),
     *                  false for boundary vertices (open path)
     * @return true if removal WOULD violate manifold
     */
    static bool checkManifoldViolation(MCGAL::Vertex* center,
                                       const std::vector<MCGAL::Vertex*>& poly,
                                       bool isClosed);

  private:
    static bool checkManifoldViolationRaw(MCGAL::Vertex* center,
                                          MCGAL::Vertex* const* poly,
                                          unsigned n,
                                          bool isClosed);
};

#endif
