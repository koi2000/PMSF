// SPDX-License-Identifier: Apache-2.0
//
// BoundedRingTraversal.h —— D17 硬化：**零分配**的有界顶点环遍历。
//
// 背景：MCGAL 的 `for (MCGAL::Halfedge* h : pmsf::boundedRing(v))` 走 `Vertex::halfedge_iterator`，
// 从 `proxyHalfedge_` 出发沿 step_cw（`h->find_prev()->opposite()`）/
// step_ccw（`h->opposite()->next()`）绕圈，**只在走回起点时终止**。
// ⇒ 环若"成环但不闭合"（实测 strict 路径有 51 个这样的顶点），**死循环**。
//
// 第一版实现返回 `std::vector`，逐字复刻步进逻辑 ⇒ 字节中性（全量回归 60 格
// 逐字节相同），**但每次调用构造 vector+set**，在
// `PriorityErrorSelector` / `UnifiedManifoldIsRemovableOperator` 等热循环里
// 造成 **1.4–2.6× 的 compress_time_ms 回归**，已回退。
//
// 本版改为**零分配迭代器**：形状与 MCGAL 的 `halfedge_iterator` 一致
// （`begin()/end()`、`operator++`、`operator!=`、`operator*`），
// 内部只有一个步数计数器 + 起点指针，**不构造任何容器**。
// 终止条件 = 走回起点 **或** 步数超过上限（不闭合的环由上限兜底）。
//
// 用法：`for (MCGAL::Halfedge* h : pmsf::boundedRing(v))` —— 对良构的环
// 产出与 `for (h : *v)` **完全相同的序列**（因此字节中性）；
// 对不闭合的环**有界终止**而非挂死。
// 面遍历（`for (h : *f)`）不需要换——面是有限表。

#ifndef PMSF_COMMON_BOUNDED_RING_TRAVERSAL_H
#define PMSF_COMMON_BOUNDED_RING_TRAVERSAL_H

#include "Halfedge.h"
#include "Vertex.h"

namespace pmsf {

// D17 根因（2026-09-21 读码定位）：`deps/MCGAL/include/Facet.h:398` 的
// `Halfedge::find_prev()` 对**非三角面**走的是
//     while (g->next() != this) g = g->next();
// —— **无界循环**。因此任何把 `find_prev()` 当作一步的"有界"走法都白搭：
// 外层上限包不住它内部的循环（实测：strict 网格上 `vertexRingCloses` 不终止）。
//
// `boundedFindPrev` 复刻它的语义但**自己计步**：
//   三角面走快捷路径（`next()->next()`，O(1)）；
//   非三角面沿 `next()` 走，**最多 maxSteps 步**，走回起点即返回，超限返回 nullptr。
// 全部在调用方实现，**不碰 deps/MCGAL**。
inline MCGAL::Halfedge* boundedFindPrev(MCGAL::Halfedge* h, int maxSteps = 64) {
    if (!h) return nullptr;
    MCGAL::Facet* f = h->face();
    if (f && f->facet_degree() == 3) {
        MCGAL::Halfedge* nx = h->next();
        return nx ? nx->next() : nullptr;   // 与 MCGAL 快捷路径一致
    }
    MCGAL::Halfedge* g = h;
    for (int i = 0; i < maxSteps; ++i) {
        MCGAL::Halfedge* nx = g->next();
        if (!nx) return nullptr;
        if (nx == h) return g;              // 走回起点 ⇒ 正常的 prev
        g = nx;
    }
    return nullptr;                         // 超限 ⇒ 该面 next() 链不闭合
}

class BoundedRingIterator {
  public:
    BoundedRingIterator() = default;
    BoundedRingIterator(MCGAL::Halfedge* cur, MCGAL::Halfedge* start, int maxSteps)
        : cur_(cur), start_(start), maxSteps_(maxSteps) {}

    MCGAL::Halfedge* operator*() const { return cur_; }

    BoundedRingIterator& operator++() {
        if (!cur_ || !start_) { cur_ = nullptr; return *this; }
        if (++steps_ > maxSteps_) { cur_ = nullptr; return *this; }
        MCGAL::Halfedge* nxt = nullptr;
        if (first_) {
            first_ = false;
            nxt = stepCw(cur_);
            if (!nxt) { ccw_ = true; nxt = stepCcw(start_); }
        } else if (!ccw_) {
            nxt = stepCw(cur_);
            if (!nxt) { ccw_ = true; nxt = stepCcw(start_); }
        } else {
            nxt = stepCcw(cur_);
        }
        cur_ = (nxt == start_) ? nullptr : nxt;
        return *this;
    }

    bool operator!=(const BoundedRingIterator& o) const { return cur_ != o.cur_; }

  private:
    static MCGAL::Halfedge* stepCw(MCGAL::Halfedge* h) {
        if (!h) return nullptr;
        MCGAL::Halfedge* n2 = boundedFindPrev(h, 64);   // D17：**有界**版，不用 find_prev
        return n2 ? n2->opposite() : nullptr;
    }
    static MCGAL::Halfedge* stepCcw(MCGAL::Halfedge* h) {
        if (!h) return nullptr;
        MCGAL::Halfedge* opp = h->opposite();
        return opp ? opp->next() : nullptr;
    }
    MCGAL::Halfedge* cur_ = nullptr;
    MCGAL::Halfedge* start_ = nullptr;
    int maxSteps_ = 256;
    int steps_ = 0;
    bool first_ = true;
    bool ccw_ = false;
};

class BoundedRing {
  public:
    BoundedRing(MCGAL::Vertex* v, int maxSteps) : v_(v), maxSteps_(maxSteps) {}
    BoundedRingIterator begin() const {
        if (!v_ || v_->isRemoved()) return BoundedRingIterator();
        return BoundedRingIterator(v_->proxyHalfedge(), v_->proxyHalfedge(), maxSteps_);
    }
    BoundedRingIterator end() const { return BoundedRingIterator(); }

  private:
    MCGAL::Vertex* v_ = nullptr;
    int maxSteps_ = 256;
};

inline BoundedRing boundedRing(MCGAL::Vertex* v, int maxSteps = 256) {
    return BoundedRing(v, maxSteps);
}

// D17「建环处保证闭合」的**判据**：从 `proxyHalfedge_` 出发绕一圈，
// 若能**走回起点**则环闭合；若因**步数上限**而终止则不闭合。
// 只读，不改任何东西 —— 供调用方在建点/建边之后自检。
inline bool vertexRingCloses(MCGAL::Vertex* v, int maxSteps = 256) {
    if (!v || v->isRemoved()) return true;
    MCGAL::Halfedge* start = v->proxyHalfedge();
    if (!start) return true;              // 无起点不算"不闭合"
    auto step_cw = [](MCGAL::Halfedge* h) -> MCGAL::Halfedge* {
        if (!h) return nullptr;
        MCGAL::Halfedge* n2 = boundedFindPrev(h, 64);
        return n2 ? n2->opposite() : nullptr;
    };
    auto step_ccw = [](MCGAL::Halfedge* h) -> MCGAL::Halfedge* {
        if (!h) return nullptr;
        MCGAL::Halfedge* opp = h->opposite();
        return opp ? opp->next() : nullptr;
    };
    MCGAL::Halfedge* cur = start;
    bool first = true, ccw = false;
    for (int i = 0; i < maxSteps; ++i) {
        MCGAL::Halfedge* nxt = nullptr;
        if (first) {
            first = false;
            nxt = step_cw(cur);
            if (!nxt) { ccw = true; nxt = step_ccw(start); }
        } else if (!ccw) {
            nxt = step_cw(cur);
            if (!nxt) { ccw = true; nxt = step_ccw(start); }
        } else {
            nxt = step_ccw(cur);
        }
        if (nxt == start) return true;    // 走回起点 ⇒ 闭合
        if (!nxt) return true;            // 走到边界 ⇒ 开扇，不算不闭合
        cur = nxt;
    }
    return false;                          // 步数用尽仍未回起点 ⇒ 不闭合
}

}  // namespace pmsf

#endif  // PMSF_COMMON_BOUNDED_RING_TRAVERSAL_H
