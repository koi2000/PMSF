#pragma once

#include "Global.h"
#include "Halfedge.h"
#include "Vertex.h"
#include "operator/IsRemovableOperator.h"

class DegreeThreeIsRemovableOperator : public IsRemovableOperator {
  public:
    DegreeThreeIsRemovableOperator() = default;

    void init(std::vector<MCGAL::Vertex*> v, std::vector<MCGAL::Halfedge*> h, std::vector<MCGAL::Facet*> f) {
        this->seeds = h;
    };

    bool isRemovable(HalfedgeErrorEntry& entry) {
        // Cleaning conquest（奇数层）：接受所有候选，具体是否为 degree-3
        // 已由 `DegreeSixIsRemovableOperator` 反过来过滤（偶数层才拒绝非 3）。
        // 但这里其实是 decimating 层（level%2==0）使用的：放开所有。
        // ⚠️ 2026-05-04 性能优化：levelFast 无锁路径。
        if (MCGAL::contextPool.levelFast() % 2 == 0) {
          return true;
        }
        MCGAL::Vertex* v = entry.edge->end_vertex();
        // 热路径：优先读缓存；仅当缓存失效时才触发一次 O(deg) 重算。
        int deg = v->is_degree_dirty() ? v->vertex_degree() : v->vertex_degree_cached();
        return deg == 3;
    }
    std::vector<MCGAL::Halfedge*> seeds;
};