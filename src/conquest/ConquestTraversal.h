/*
 * @file ConquestTraversal.h
 * @brief 前沿遍历的**中性别名与纯函数**（编码引擎与解码镜像**唯一来源**）。
 *
 * **为什么单列一个头**：编码端（`FrontierConquestEngine.cpp`）与解码端
 * （`ConquestDecodeLayer.cpp`）必须用**逐字相同**的取邻、边界判定与
 * `nullPatchTail`/`findType` —— 只要有一处悄悄分家，两侧的 sign 传播就会
 * 不同，进而 `findType` 退化、拓扑分岔（本库已因此吃过多次亏）。
 * 曾经它们是两份复制粘贴；本文件把它们收成一份。
 *
 * **本文件不认识任何具体算法名**（lint 纪律）；`Conquest` 是机制名,不是方案名。
 */
#ifndef PMSF_CONQUEST_CONQUEST_TRAVERSAL_H
#define PMSF_CONQUEST_CONQUEST_TRAVERSAL_H

#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Vertex.h"

#include "conquest/FrontierConquestEngine.h"  // ConquestState

#include <cstdint>
#include <deque>

namespace pmsf {

// ---- 中性别名（参考实现用的是另一套网格库的命名）--------------------------
inline MCGAL::Vertex* tgt(MCGAL::Halfedge* h) {
    return h ? h->end_vertex() : nullptr;
}
inline MCGAL::Halfedge* nxt(MCGAL::Halfedge* h) {
    return h ? h->next() : nullptr;
}
inline MCGAL::Halfedge* opp(MCGAL::Halfedge* h) {
    return h ? h->opposite() : nullptr;
}

/// @brief 面的前驱。
///
/// **不能直接信任 `Facet::facet_degree()`**：它是**存量计数器**，而参考实现的
/// `find_prev()` 用 `facet_degree() == 3` 决定走不走三角面快捷路径
/// （`next()->next()`）。只要某个面的 degree 在某次拓扑改动后没被刷新，
/// 快捷路径就会在**真多边形**上返回**静默错误**的 prev —— 无崩溃、无告警，
/// 只是环游少走一格。（这条是 P13.1 第 1 层分歧的候选根因之一，虽然后来
/// 证明真正的根因是 degree 缓存，但这个信任也不该保留。）
/// 环游版在三角面上与快捷路径**逐字等价**，在多边形上才是真 prev。
inline MCGAL::Halfedge* prv(MCGAL::Halfedge* h) {
    if (!h)
        return nullptr;
    MCGAL::Halfedge* g = h;
    for (int i = 0; i < 512; ++i) {
        MCGAL::Halfedge* n = g->next();
        if (!n)
            return nullptr;
        if (n == h)
            return g;
        g = n;
    }
    return nullptr;  // 环不闭合（D17 类），有界退出而不是挂死
}

/// @brief 一条边是否为**边界边**。CGAL 的语义是**对称**的：
/// 「本侧无面 **或** 对侧无面」。只看一侧会在「本侧是边界、对侧不是」时漏判。
inline bool isBorderEdge(MCGAL::Halfedge* h) {
    if (!h)
        return true;
    for (int side = 0; side < 2; ++side) {
        MCGAL::Halfedge* e = (side == 0) ? h : h->opposite();
        if (!e || e->isRemoved())
            return true;
        MCGAL::Facet* f = e->face();
        if (!f || f->isRemoved())
            return true;
    }
    return false;
}

// 面/顶点标志位取值（与 ConquestState 的语义对应）
constexpr std::uint8_t kFFree = 0, kFConquered = 1, kFToBeRemoved = 2, kFTemp = 3;
constexpr std::uint8_t kVMoved = 2;  // ToBeRemoved

/// @brief 绕 `v` 的**出边**，收集 `target()` 即得其一环邻居。
///
/// ⚠️ **遍历次序必须与 MCGAL 原生 `halfedge_iterator` 一致：先 cw 后 ccw。**
///
/// 这不是风格问题，是**语义**问题：环不闭合时（D17 已知问题 —— strict 路径上
/// 实测有 51 个顶点"成环但不闭合"），纯 ccw 走法访问到的顶点集合与原生迭代器
/// **不同** ⇒ 流形判据 `circHits` 给出不同答案 ⇒ 放行/拒绝删除不同 ⇒ 拓扑分岔。
/// 这正是「迁移版与验证器分岔」的根因：验证器用的是复刻原生次序的
/// `pmsf::boundedRing`，而本文件第一版写成了纯 ccw。
///
/// 同一个顶点必须有三处一致：本函数、`isBorderVertex`、以及 MCGAL 自己的
/// `recompute_degree`（走原生迭代器）。三者不一致就会出现"同一个顶点
/// 三处算出的环不一样"。
inline int ringNeighbors(MCGAL::Vertex* v, MCGAL::Halfedge** out, int cap) {
    int n = 0;
    MCGAL::Halfedge* start = v ? v->proxyHalfedge() : nullptr;
    if (!start)
        return 0;
    auto stepCw = [](MCGAL::Halfedge* h) -> MCGAL::Halfedge* {
        MCGAL::Halfedge* p = prv(h);
        return p ? p->opposite() : nullptr;
    };
    auto stepCcw = [](MCGAL::Halfedge* h) -> MCGAL::Halfedge* {
        MCGAL::Halfedge* o = h ? h->opposite() : nullptr;
        return o ? o->next() : nullptr;
    };
    MCGAL::Halfedge* cur = start;
    bool first = true, ccw = false;
    for (int i = 0; i < cap; ++i) {
        out[n++] = cur;
        MCGAL::Halfedge* nx = nullptr;
        if (first || !ccw) {
            first = false;
            nx = stepCw(cur);
            if (!nx) {
                ccw = true;
                nx = stepCcw(start);
            }
        } else {
            nx = stepCcw(cur);
        }
        if (!nx) {
            --n;  // 走到边界：这一条不算
            break;
        }
        if (nx == start)
            break;
        cur = nx;
    }
    return n;
}

/// @brief 顶点 `target(h)` 的环上是否有边界边。
///
/// ⚠️ 注意是**顶点环**、不是面环。一个常见的写错法是沿 `h->next()` 绕一圈
/// （那是**面**），问的变成了"这个面有没有边界边" —— 完全是另一个问题。
/// 闭网格上两种写法都恒 false，所以**只有带边界网格才照得出这个错**。
///
// ---- H1 gate 出队（编码/解码共用；未激活时与旧 FIFO 路径逐字节等价）---------

/// @brief 合成 gate 的只读优先级视图。`front = tgt(nxt(h))`（本 gate 将要删除的
/// 前端点）、`back = tgt(h)`（gate 的另一端），与引擎 decimateLayer 的用法一致。
inline GatePriorityView gatePriorityView(MCGAL::Halfedge* h, const ConquestState& st) {
    GatePriorityView v;
    if (!h) return v;
    MCGAL::Vertex* front = tgt(nxt(h));
    MCGAL::Vertex* back = tgt(h);
    auto fill = [&](MCGAL::Vertex* v, std::uint8_t& sign, std::uint16_t& valence,
                    std::uint16_t& conq, std::uint8_t& conqOk) {
        if (!v) return;
        sign = st.sign(v);
        v->recompute_degree();
        valence = static_cast<std::uint16_t>(v->vertex_degree());
        MCGAL::Halfedge* hs[128];
        const int n = ringNeighbors(v, hs, 128);
        int c = 0;
        for (int i = 0; i < n; ++i) {
            MCGAL::Vertex* w = tgt(hs[i]);
            if (w && st.flag(w) == 1u) ++c;  // kFConquered
        }
        conq = static_cast<std::uint16_t>(c);
        conqOk = (c >= 2 && c <= 3) ? 1u : 0u;
    };
    fill(front, v.frontSign, v.frontValence, v.frontConquered, v.frontConqOk);
    fill(back, v.backSign, v.backValence, v.backConquered, v.backConqOk);
    return v;
}

/// @brief H1 出队。未激活或队列 <2 ⇒ `front()/pop_front()`（与旧路径逐字节等价）；
/// 激活 ⇒ 扫 deque 取 `higherPriority` 最优（默认 policy 恒 false ⇒ 取队首）。
inline MCGAL::Halfedge* popGate(std::deque<MCGAL::Halfedge*>& q,
                                const GatePriorityPolicy& policy,
                                const ConquestState& st) {
    if (q.empty()) return nullptr;
    if (!policy.isActive() || q.size() < 2) {
        MCGAL::Halfedge* h = q.front();
        q.pop_front();
        return h;
    }
    std::size_t best = 0;
    GatePriorityView bestView = gatePriorityView(q[0], st);
    for (std::size_t i = 1; i < q.size(); ++i) {
        const GatePriorityView v = gatePriorityView(q[i], st);
        if (policy.higherPriority(v, bestView)) {
            best = i;
            bestView = v;
        }
    }
    MCGAL::Halfedge* h = q[best];
    q.erase(q.begin() + static_cast<std::ptrdiff_t>(best));
    return h;
}

/// 用**同一套**圈遍历（`ringNeighbors`，先 cw 后 ccw），理由见那里。
inline bool isBorderVertex(MCGAL::Halfedge* h) {
    if (!h)
        return true;
    MCGAL::Vertex* v = tgt(h);
    if (!v)
        return true;
    MCGAL::Halfedge* hs[256];
    const int n = ringNeighbors(v, hs, 256);
    for (int i = 0; i < n; ++i)
        if (isBorderEdge(hs[i]))
            return true;
    return false;
}

/// @brief null-patch 的**尾部**：4 路 sign 传播 + 两个队列 push。
///
/// 参考实现里"manifold 违规"分支与"最终 else"分支是复制粘贴的孪生体，
/// 尾部**逐字相同** —— 故抽成一个函数两处共用。漏掉任何一半都会静默改变
/// sign 传播，进而让 `findType` 全面退化（见 decimateLayer 兜底分支的注释）。
inline void nullPatchTail(ConquestState& st, MCGAL::Halfedge* h, std::deque<MCGAL::Halfedge*>& q) {
    MCGAL::Vertex* vA = tgt(h);
    MCGAL::Vertex* vB = tgt(opp(h));
    MCGAL::Vertex* vF = tgt(nxt(h));
    const std::uint8_t sA = st.sign(vA), sB = st.sign(vB);
    if (sA == 1 && sB == 2)
        st.setSign(vF, 1);
    else if (sA == 2 && sB == 1)
        st.setSign(vF, 1);
    else if (sA == 1 && sB == 1)
        st.setSign(vF, 2);
    else if (sA == 2 && sB == 2)
        st.setSign(vF, 1);
    if (!isBorderEdge(nxt(h)))
        q.push_back(opp(nxt(h)));
    if (!isBorderEdge(prv(h)))
        q.push_back(opp(prv(h)));
}

// ---- 类型编码 --------------------------------------------------------------
// type = 4 * (valence - 3) + {1:(-,+), 2:(+,-), 3:(+,+), 4:(-,-)}
inline int findType(const ConquestState& st, MCGAL::Halfedge* h, int valence) {
    const std::uint8_t a = st.sign(tgt(h));
    const std::uint8_t b = st.sign(tgt(opp(h)));
    int k = 0;
    if (a == 2 && b == 1)
        k = 1;  // MINUS, PLUS
    else if (a == 1 && b == 2)
        k = 2;  // PLUS, MINUS
    else if (a == 1 && b == 1)
        k = 3;  // PLUS, PLUS
    else if (a == 2 && b == 2)
        k = 4;  // MINUS, MINUS
    if (k == 0)
        return 0;  // 未覆盖的组合 ⇒ 0（参考实现亦然）
    return 4 * (valence - 3) + k;
}

}  // namespace pmsf

#endif  // PMSF_CONQUEST_CONQUEST_TRAVERSAL_H
