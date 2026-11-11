#pragma once
#include "Global.h"
#include "Halfedge.h"
#include "Vertex.h"
#include "ContextPool.h"
#include "operator/IsRemovableOperator.h"

#include <cstdlib>
#include <cstring>

/**
 * PCATM 论文 §4 对顶点度数的描述：
 *   - 独立集（MIS）保证：同轮里被删的顶点两两不相邻；
 *   - 统计上优先选 degree ∈ {5, 6} 的顶点（4/2-coloring 覆盖率最高），
 *     但对 degree ∈ [3, 10] 的其它顶点同样可以合法删除，论文表 1 中
 *     bit/vertex 就包含了这类"非典型 degree"情形。
 *
 * 早期实现只允许 degree ∈ {5, 6}，在圆角稠密区域每轮能删的顶点极少，
 * bpv 偏离论文值较多，也会让 triangulate_general 频繁处理度 7-10 的面。
 * 这里放宽到 [3, 10]（上界 10 与论文实测一致——再高的 polygon 几何质量
 * 和熵收益都很差），交由 MIS + Manifold operator 做其它合法性检查。
 */
class DegreeSixIsRemovableOperator : public IsRemovableOperator {
  public:
    DegreeSixIsRemovableOperator() = default;

    void init(std::vector<MCGAL::Vertex*> v, std::vector<MCGAL::Halfedge*> h, std::vector<MCGAL::Facet*> f) {
        this->seeds = h;
    };

    bool isRemovable(HalfedgeErrorEntry& entry) {
        if (entry.edge == nullptr) return false;
        MCGAL::Vertex* vr = entry.edge->end_vertex();
        if (vr == nullptr) return false;
        // 使用 cached degree，避免每次 select 都走 vertex_degree() 的
        // 懒重算 O(deg) 路径。PMSF::pcatm{,Columnar}Compress 在入口处
        // 已经调 mesh->recomputeAllVertexDegrees()，后续 elimination 由
        // Vertex::dec/incCachedDegree 维护，故这里只读缓存即可。
        const char* strict = std::getenv("PMSF_PCATM_USE_STRICT_PAPER");
        const char* twoColorOnly = std::getenv("PMSF_PCATM_STRICT_TWO_COLOR_ONLY");
        const char* enableMode2 = std::getenv("PMSF_PCATM_STRICT_ENABLE_MODE2");
        const char* disableLevelSwitch = std::getenv("PMSF_PCATM_STRICT_DISABLE_LEVEL_SWITCH");
        const char* allowLowDegree = std::getenv("PMSF_PCATM_STRICT_ALLOW_LOW_DEGREE");
        const char* aggressive = std::getenv("PMSF_PCATM_AGGRESSIVE_REMOVABLE");
        const bool strictPaper =
            strict && *strict && std::strcmp(strict, "0") != 0;
        const bool aggressiveRemovable =
            aggressive && *aggressive && std::strcmp(aggressive, "0") != 0;
        const bool allowLow =
            allowLowDegree && *allowLowDegree && std::strcmp(allowLowDegree, "0") != 0;
        const bool strictTwoColor =
            strictPaper &&
            twoColorOnly && *twoColorOnly && std::strcmp(twoColorOnly, "0") != 0;
        const bool levelSwitchDisabled =
            disableLevelSwitch && *disableLevelSwitch &&
            std::strcmp(disableLevelSwitch, "0") != 0;
        const bool mode2Requested =
            enableMode2 && *enableMode2 && std::strcmp(enableMode2, "0") != 0;
        const bool strictLevelSwitch =
            strictPaper && mode2Requested && !levelSwitchDisabled;
        int d = (strictTwoColor || strictLevelSwitch)
                    ? eraseCenterValence(entry.edge)
                    : (vr->is_degree_dirty() ? vr->vertex_degree() : vr->vertex_degree_cached());
        // BPV-oriented aggressive mode: keep only the basic topology cap used by
        // UnifiedManifoldIsRemovableOperator.  This lets each round's greedy MIS
        // delete substantially more vertices and reduces the number of color
        // levels that must be transmitted.
        if (strictPaper && aggressiveRemovable) {
            return d >= 3 && d < 17;
        }
        // 论文里 2-coloring 对 d∈{5,6} 最优，4-coloring 对 d∈{3,4} 最优；
        // 取并集 [3, 10]，交给 MIS + Manifold 做最终过滤。
        //
        // BlobLayout=25 当前是严格 2-color storage（一 triangle 一 bit），没有
        // 4-color fallback。论文明确说明 quadrilateral cannot be encoded by the
        // 2-color technique，因此 strict 2-color 复现默认禁止 degree 3/4。
        // 需要对照旧数据时可设置 PMSF_PCATM_STRICT_ALLOW_LOW_DEGREE=1。
        if (strictTwoColor && !allowLow) {
            return d >= 5 && d <= 10;
        }
        if (strictLevelSwitch && !allowLow && (MCGAL::contextPool.levelFast() % 2 == 0)) {
            return d >= 5 && d <= 10;
        }
        return d >= 3 && d <= 10;
    }

    std::vector<MCGAL::Halfedge*> seeds;

  private:
    static int eraseCenterValence(MCGAL::Halfedge* h) {
        if (!h || !h->next() || !h->next()->opposite()) return 0;
        int degree = 1;
        MCGAL::Halfedge* g = h->next()->opposite();
        int guard = 0;
        while (g && g != h) {
            ++degree;
            if (!g->next() || !g->next()->opposite()) return 0;
            g = g->next()->opposite();
            if (++guard > 128) return 0;
        }
        return degree;
    }
};
