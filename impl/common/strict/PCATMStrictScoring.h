/*
 * @file PCATMStrictScoring.h
 * @brief PCATM 严格复现（`PMSF_PCATM_USE_STRICT_PAPER`）用的候选打分与误差函数。
 *
 * ## 为什么这两个函数住在这里
 *
 * 它们原先以 `static` 私有成员的形式长在 `src/selection/HalfedgeSelectionQueue.h`
 * 与 `src/selection/GreedySelector.cpp` 里，靠
 * `getenv("PMSF_PCATM_USE_STRICT_PAPER")` 决定要不要走。那是**算法分支硬编码
 * 在框架层**，违反「`src/` 不认识任何算法」。
 *
 * 现在框架只认 `pmsf::SelectionPolicy` 的三个算法无关开关（允许新增半边 /
 * 候选打分 / 误差函数），PCATM 在插件侧把这两段装进去 —— 代码没变，只是
 * 归位到了它本来就属于的层。
 *
 * ## 两个函数的关系
 *
 * 形式相近但**不是同一个东西**，刻意没有合并：
 *   - `strictError()` 交给 selection queue 的 `computeError`，参与堆排序；
 *     激进模式下返回 `d*1024 + liveDegree`（小度数优先 ⇒ 独立集更大、
 *     每 patch 的颜色符号更少）。
 *   - `strictCandidateScore()` 交给 `GreedySelector::select` 在**一个面的环上**
 *     挑消除哪个顶点，返回**越大越优先**（调用方用 `>` 比较）。
 *
 * 合并会改变两处的数值语义，进而改变产物字节。
 */
#pragma once

#include "core/SelectionPolicy.h"

#include "Halfedge.h"
#include "Vertex.h"

#include <cstdlib>
#include <cstring>

namespace pmsf {
namespace pcatm_detail {

/// @brief 「被消除中心顶点的度数」：绕 end_vertex 一环的live 半边数。
///        与框架侧同名helper保持逐字相同（它是打分公式的一部分，不能各写各的）。
inline int removedCenterValence(MCGAL::Halfedge* h) {
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

inline bool envOn(const char* name) {
    const char* v = std::getenv(name);
    return v && *v && std::strcmp(v, "0") != 0;
}

/// @brief 严格模式是否开启（`PMSF_PCATM_USE_STRICT_PAPER`）。
inline bool strictPaperEnabled() { return envOn("PMSF_PCATM_USE_STRICT_PAPER"); }

/// @brief 激进可消除（`PMSF_PCATM_AGGRESSIVE_REMOVABLE`，严格模式下的子开关）。
inline bool aggressiveEnabled() {
    return strictPaperEnabled() && envOn("PMSF_PCATM_AGGRESSIVE_REMOVABLE");
}

/// @brief 交给 `HalfedgeSelectionQueue::computeError` 的误差函数。
inline float strictError(MCGAL::Halfedge* edge) {
    const int d = removedCenterValence(edge);
    if (aggressiveEnabled()) {
        MCGAL::Vertex* v = edge ? edge->end_vertex() : nullptr;
        int liveDegree = d;
        if (v && !v->isRemoved()) {
            liveDegree = v->is_degree_dirty() ? v->vertex_degree() : v->vertex_degree_cached();
        }
        // Min-heap: low valence first approximates a larger greedy
        // independent set and also emits fewer color symbols per patch.
        return static_cast<float>(d * 1024 + liveDegree);
    }
    int score = 0;
    if (d == 5 || d == 6) {
        score += 10000;
    } else if (d >= 7 && d <= 10) {
        score += 7000;
    } else if (d == 3 || d == 4) {
        score += 1000;
    }
    score -= std::abs(d - 6) * 16;
    MCGAL::Vertex* v = edge ? edge->end_vertex() : nullptr;
    if (v && !v->isRemoved()) {
        score += v->is_degree_dirty() ? v->vertex_degree() : v->vertex_degree_cached();
    }
    return -static_cast<float>(score);
}

/// @brief 交给 `GreedySelector::select` 的候选打分（越大越优先）。
inline float strictCandidateScore(MCGAL::Halfedge* h) {
    const int d = removedCenterValence(h);
    if (d <= 0) return -1000000.0f;
    int score = 0;
    if (d == 5 || d == 6) {
        score += 10000;
    } else if (d >= 7 && d <= 10) {
        score += 7000;
    } else if (d == 3 || d == 4) {
        score += 1000;
    }
    score -= std::abs(d - 6) * 16;
    MCGAL::Vertex* v = h ? h->end_vertex() : nullptr;
    if (v && !v->isRemoved()) {
        score += v->is_degree_dirty() ? v->vertex_degree() : v->vertex_degree_cached();
    }
    return static_cast<float>(score);
}

/// @brief 按环境变量装配策略。未开严格模式 ⇒ 返回默认策略（= 框架默认行为）。
inline SelectionPolicy makeSelectionPolicy() {
    SelectionPolicy policy;
    if (!strictPaperEnabled()) return policy;
    policy.errorOverride = [](MCGAL::Halfedge* e, MCGAL::Point&) { return strictError(e); };
    policy.candidateScore = [](MCGAL::Halfedge* e) { return strictCandidateScore(e); };
    // 激进模式下允许本轮新增的半边参与候选。
    policy.allowAddedHalfedges = aggressiveEnabled();
    return policy;
}

}  // namespace pcatm_detail
}  // namespace pmsf
