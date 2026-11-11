#pragma once

#include "Global.h"
#include "Halfedge.h"
#include "Vertex.h"
#include "operator/IsRemovableOperator.h"

/**
 * Alliez-Desbrun 奇数 cleaning pass：只移除 valence == 3 的顶点。
 *
 * 修复：旧实现偶数层直接 return true（等于不过滤），奇数层只收 degree==3；
 *        这与 D6 的旧实现配合会让奇/偶 pass 在实质上合并，valence 分布不再被
 *        "decimating + cleaning" 两种节奏压缩，熵分布变差。
 *
 * 现在语义：
 *   偶数 pass 直接 return true（由 D6 过滤）；
 *   奇数 pass 只放行 degree == 3。
 */
class DegreeThreeIsRemovableOperator : public IsRemovableOperator {
  public:
    DegreeThreeIsRemovableOperator() = default;

    void init(std::vector<MCGAL::Vertex*> v, std::vector<MCGAL::Halfedge*> h, std::vector<MCGAL::Facet*> f) {
        this->seeds = h;
    };

    bool isRemovable(HalfedgeErrorEntry& entry) {
        // ⚠️ 2026-05-04 性能优化：levelFast 无锁路径。
        int level = MCGAL::contextPool.levelFast();
        // 偶数 decimating pass：D6 负责过滤；本算子放行
        if ((level % 2) == 0) {
            return true;
        }
        MCGAL::Vertex* v = entry.edge->end_vertex();
        if (v == nullptr || v->isRemoved())
            return false;
        int deg = v->is_degree_dirty() ? v->vertex_degree() : v->vertex_degree_cached();
        return deg == 3;
    }
    std::vector<MCGAL::Halfedge*> seeds;
};
