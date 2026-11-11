#pragma once
#include "Global.h"
#include "Halfedge.h"
#include "Vertex.h"
#include "operator/IsRemovableOperator.h"

/**
 * APTM / Alliez-Desbrun 的 valence-driven conquest：
 *   偶数轮（even decimating pass）—— 移除 valence ≤ 6 的顶点；
 *   奇数轮（odd cleaning pass） —— 只移除 valence == 3 的顶点（由 DegreeThreeIsRemovableOperator 负责）。
 *
 * 修复：
 *   旧实现里条件是 `degree > 3 && degree <= 6`，实际上把 degree==3 的偶数层顶点
 *   误判为不可移除——但偶数 pass 本来就是 "≤6"，degree==2/3 也应通过。同时
 *   `level % 2 == 1` 的奇数 pass 里直接 return true（无条件通过），这会让本应
 *   只删 valence==3 的奇数 pass 把任何通过 D3 自己筛选的顶点也放行 → 实质上
 *   等价于两层都"全收"，完全丢掉了 valence-driven 的语义。
 *
 *   现在改为：**只在偶数 pass 启用本算子**，按 "2 ≤ degree ≤ 6" 过滤；奇数 pass
 *   直接返回 true 不拦截（由同时注册的 DegreeThreeIsRemovableOperator 把关）。
 */
class DegreeSixIsRemovableOperator : public IsRemovableOperator {
  public:
    DegreeSixIsRemovableOperator() = default;

    void init(std::vector<MCGAL::Vertex*> v, std::vector<MCGAL::Halfedge*> h, std::vector<MCGAL::Facet*> f) {
        this->seeds = h;
    };

    bool isRemovable(HalfedgeErrorEntry& entry) {
        // ⚠️ 2026-05-04 性能优化：levelFast 无锁路径。
        int level = MCGAL::contextPool.levelFast();
        // 奇数 cleaning pass：不在本算子负责范围内，直接放行（交给 D3 算子）
        if ((level % 2) == 1) {
            return true;
        }
        MCGAL::Vertex* v = entry.edge->end_vertex();
        if (v == nullptr || v->isRemoved())
            return false;
        int deg = v->is_degree_dirty() ? v->vertex_degree() : v->vertex_degree_cached();
        // 偶数 decimating pass：valence ≤ 6（论文 §4.1）。degree<2 属于退化孤立点，不收。
        return deg >= 2 && deg <= 6;
    }

    std::vector<MCGAL::Halfedge*> seeds;
};
