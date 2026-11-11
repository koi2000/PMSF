#pragma once
// perf（2026-11-09 QEM 实测）：`for (Halfedge* h : *v)` 走 MCGAL 迭代器，每步要
// find_prev（非三角面 O(deg)）并带状态机。很多判定只依赖 1-ring 的「集合」和
// 「任一违例」的布尔结果，与顺序无关 ⇒ 用 O(1) 步进 h = h->opposite()->next()
// 遍历（集合相同、顺序相反）；异常环（未闭合）返回 Broken，调用方回退旧路径。
#include "Halfedge.h"
#include "Vertex.h"

namespace pmsf {

enum class RingWalkResult {
    Completed,
    EarlyStop,
    Broken,
};

/// fn 返回 false 表示提前结束（EarlyStop）。
template <typename Fn>
inline RingWalkResult walkOutgoingRing(MCGAL::Vertex* v, Fn&& fn, int maxSteps = 4096) {
    if (!v) return RingWalkResult::Broken;
    MCGAL::Halfedge* start = v->proxyHalfedge();
    if (!start) return RingWalkResult::Broken;
    MCGAL::Halfedge* h = start;
    int guard = 0;
    for (;;) {
        if (!fn(h)) return RingWalkResult::EarlyStop;
        MCGAL::Halfedge* opp = h->opposite();
        if (!opp) return RingWalkResult::Broken;
        h = opp->next();
        if (!h || ++guard > maxSteps) return RingWalkResult::Broken;
        if (h == start) return RingWalkResult::Completed;
    }
}

}  // namespace pmsf