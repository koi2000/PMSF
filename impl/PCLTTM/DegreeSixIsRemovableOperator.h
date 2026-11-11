#pragma once
#include "Global.h"
#include "Halfedge.h"
#include "Vertex.h"
#include "operator/IsRemovableOperator.h"

class DegreeSixIsRemovableOperator : public IsRemovableOperator {
  public:
    DegreeSixIsRemovableOperator() = default;

    void init(std::vector<MCGAL::Vertex*> v, std::vector<MCGAL::Halfedge*> h, std::vector<MCGAL::Facet*> f) {
        this->seeds = h;
    };

    bool isRemovable(HalfedgeErrorEntry& entry) {
        // Cleaning conquest（奇数层）：DegreeThreeIsRemovable 已负责 valence==3 过滤，
        // 这里放开即可。
        // ⚠️ 2026-05-04 性能优化：levelFast 无锁路径。
        if (MCGAL::contextPool.levelFast() % 2 == 1) {
            return true;
        }
        MCGAL::Vertex* v = entry.edge->end_vertex();
        // 热路径：优先读缓存；仅当缓存失效时才触发一次 O(deg) 重算。
        int deg = v->is_degree_dirty() ? v->vertex_degree() : v->vertex_degree_cached();
        return deg > 3 && deg <= 6;
    }

    std::vector<MCGAL::Halfedge*> seeds;
};