// SPDX-License-Identifier: Apache-2.0
//
// RDCanonicalTraversal.h — task-43 route A：RD 论文 §2.1/§3.3 的
// **canonical deterministic frontier traversal（frontier FIFO gate queue +
// valence/null-patch 符号 + 解码端 replay）**，为 impl/RD 的 sv=71 码流服务。
//
// 为什么要它：默认 rd_tag 路径序列化 BFS 的 per-face isPoly locator（12.8 KB
// 结构成本）；论文的 gate queue 本身就是 locator —— 解码端按同一 traversal
// 在 base mesh 上重放即可，无需任何 patch 位置 side-info。
//
// 机制（Ahn et al. 2011 §2.1 建立在 Alliez–Desbrun valence-conquest 之上）：
//   1) 种子：一条 gate 半边；两端标记 CONQUERED（各层入口都重新标记 ⇒ 种子存活）。
//   2) FIFO pop gate h；front = tgt(nxt(h))。
//   3) front 可删（3 ≤ valence ≤ 6、非边界、未被本层征服、流形判据通过）
//        ⇒ 发 valence-3 符号，记录残差（Frenet inverse），erase_center_vertex
//          + retriangulate（按 sign 对 type 查表），前沿扩张。
//      不可删 ⇒ 发 null-patch 符号 4，sign 传播 + 前沿扩张。
//   4) 队列空 = 本层结束。
//   解码端：同一 gate 序列在 base mesh 上重放。valence ⇒ 先把 fan 区域并回
//   polygon（removeEdges 按 type 查表）→ 插中心点 → fan 三角化；null ⇒ 只扩张。
//
// 忠实性/来源说明：遍历机制逐条对照框架内已被逐层对拍的 A&D conquest 语义
// （`src/conquest/` 的 retriangulate / manifoldViolated / removeEdges /
// addCenterVertex / findType / sign 传播规则），**在 impl/RD 内重新实现**为 route A
// 的新代码；不 include / 不 alias conquest codec，也不复用其 blob 布局与几何编码。
// RD 侧的差异：几何为 RD 论文的局部 Frenet lifting（RDLift.h）+ 整数格残差，
// 由 sv=71 三流承载（见 RDCanonicalCollector / RDCanonicalBlobFormat.h）。
//
// 关键不变量（写死在代码里，改一处就要重跑 B2 六项）：
//   - 编码端/解码端的 sign 传播、type 查表、flag 标记必须逐字同源；
//   - 几何取环两端都用 ringNeighbors（同一个函数、同一环序规则）；
//   - 几何残差在**量化整数域**计算：位置 = pred_cell + d，重建 = intPosToFloat(d)；
//     预测子只影响熵，不影响可逆性。
#ifndef PMSF_IMPL_RD_RD_CANONICAL_TRAVERSAL_H
#define PMSF_IMPL_RD_RD_CANONICAL_TRAVERSAL_H

#include "ContextPool.h"
#include "Facet.h"
#include "Global.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Point.h"
#include "Vertex.h"

#include "RD/RDLift.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <functional>
#include <unordered_map>
#include <vector>

namespace rd_canon {

// ---- 中性别名 ---------------------------------------------------------------
inline MCGAL::Vertex* tgt(MCGAL::Halfedge* h) {
    return h ? h->end_vertex() : nullptr;
}
inline MCGAL::Halfedge* nxt(MCGAL::Halfedge* h) {
    return h ? h->next() : nullptr;
}
inline MCGAL::Halfedge* opp(MCGAL::Halfedge* h) {
    return h ? h->opposite() : nullptr;
}
/// 面的前驱：**不能**信 `Facet::facet_degree()`（存量计数，split/join 不维护）。
inline MCGAL::Halfedge* prv(MCGAL::Halfedge* h) {
    if (!h) return nullptr;
    MCGAL::Halfedge* g = h;
    for (int i = 0; i < 512; ++i) {
        MCGAL::Halfedge* n = g->next();
        if (!n) return nullptr;
        if (n == h) return g;
        g = n;
    }
    return nullptr;
}

/// 对称边界判定：本侧或对侧无面/已移除都算边界。
inline bool isBorderEdge(MCGAL::Halfedge* h) {
    if (!h) return true;
    for (int side = 0; side < 2; ++side) {
        MCGAL::Halfedge* e = (side == 0) ? h : h->opposite();
        if (!e || e->isRemoved()) return true;
        MCGAL::Facet* f = e->face();
        if (!f || f->isRemoved()) return true;
    }
    return false;
}

/// 面的真实环长（所有度数判断走它）。
inline int trueFaceDeg(MCGAL::Halfedge* h) {
    if (!h || !h->face() || h->face()->isRemoved()) return -1;
    int n = 0;
    MCGAL::Halfedge* it = h;
    for (int i = 0; i < 20000; ++i) {
        ++n;
        it = it->next();
        if (!it || it == h) break;
    }
    return n;
}

/// 绕 v 的出边环（先 cw 后 ccw，与 MCGAL 原生 halfedge_iterator 同一次序）。
/// 编码端 recordGeometry 与解码端插入后取环都用它 ⇒ 两端环集合/次序一致。
inline int ringNeighbors(MCGAL::Vertex* v, MCGAL::Halfedge** out, int cap) {
    int n = 0;
    MCGAL::Halfedge* start = v ? v->proxyHalfedge() : nullptr;
    if (!start) return 0;
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
            --n;
            break;
        }
        if (nx == start) break;
        cur = nx;
    }
    return n;
}

/// 顶点 `tgt(h)` 的环上是否有边界边（是**顶点环**，不是面环）。
inline bool isBorderVertex(MCGAL::Halfedge* h) {
    if (!h) return true;
    MCGAL::Vertex* v = tgt(h);
    if (!v) return true;
    MCGAL::Halfedge* hs[256];
    const int n = ringNeighbors(v, hs, 256);
    for (int i = 0; i < n; ++i)
        if (isBorderEdge(hs[i])) return true;
    return false;
}

// ---- 层内状态（**本地表**，不污染 MCGAL flag）--------------------------------
constexpr std::uint8_t kFFree = 0, kFConquered = 1, kFToBeRemoved = 2, kFTemp = 3;
constexpr std::uint8_t kVMoved = 2;  // ToBeRemoved

struct State {
    std::unordered_map<MCGAL::Vertex*, std::uint8_t> vertexFlag;
    std::unordered_map<MCGAL::Vertex*, std::uint8_t> vertexSign;  // 0=none 1=+ 2=-
    std::unordered_map<MCGAL::Facet*, std::uint8_t> facetFlag;

    std::uint8_t flag(MCGAL::Vertex* v) const {
        auto it = vertexFlag.find(v);
        return it == vertexFlag.end() ? 0u : it->second;
    }
    void setFlag(MCGAL::Vertex* v, std::uint8_t f) {
        vertexFlag[v] = f;
    }
    std::uint8_t sign(MCGAL::Vertex* v) const {
        auto it = vertexSign.find(v);
        return it == vertexSign.end() ? 0u : it->second;
    }
    /// 仅 NOSIGN 时写入（与论文/参考一致）。
    void setSign(MCGAL::Vertex* v, std::uint8_t s) {
        if (v == nullptr) return;
        if (sign(v) == 0u) vertexSign[v] = s;
    }
    /// 强制写 sign（解码端 removeEdges 的 sign 写入用）。
    void putSign(MCGAL::Vertex* v, std::uint8_t s) {
        if (v == nullptr) return;
        vertexSign[v] = s;
    }
    std::uint8_t fflag(MCGAL::Facet* f) const {
        auto it = facetFlag.find(f);
        return it == facetFlag.end() ? 0u : it->second;
    }
    void setFFlag(MCGAL::Facet* f, std::uint8_t v) {
        if (f == nullptr) return;
        facetFlag[f] = v;
    }
    void reset() {
        vertexFlag.clear();
        vertexSign.clear();
        facetFlag.clear();
    }
};

/// type = 4*(valence-3) + {1:(-,+), 2:(+,-), 3:(+,+), 4:(-,-)}；k==0 未覆盖。
inline int findType(const State& st, MCGAL::Halfedge* h, int valence) {
    const std::uint8_t a = st.sign(tgt(h));
    const std::uint8_t b = st.sign(tgt(opp(h)));
    int k = 0;
    if (a == 2 && b == 1)
        k = 1;
    else if (a == 1 && b == 2)
        k = 2;
    else if (a == 1 && b == 1)
        k = 3;
    else if (a == 2 && b == 2)
        k = 4;
    if (k == 0) return 0;
    return 4 * (valence - 3) + k;
}

/// null-patch 尾部：4 路 sign 传播 + 两个队列 push（两处 null 分支逐字共用）。
inline void nullPatchTail(State& st, MCGAL::Halfedge* h,
                          std::deque<MCGAL::Halfedge*>& q) {
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
    if (!isBorderEdge(nxt(h))) q.push_back(opp(nxt(h)));
    if (!isBorderEdge(prv(h))) q.push_back(opp(prv(h)));
}

// ---- 重三角化（编码端）------------------------------------------------------
// 逐支查表切分，新边连 tgt(h) 与 tgt(g)，与参考实现位精确一致。
inline void retriangulate(MCGAL::Mesh* mesh, State& st, MCGAL::Halfedge* ch,
                          int valence, int type) {
    MCGAL::Halfedge* h = ch;
    auto split = [&](MCGAL::Halfedge* a2, MCGAL::Halfedge* b2) -> MCGAL::Halfedge* {
        return mesh->split_facet(a2, b2);
    };
    MCGAL::Halfedge* g = nxt(h);
    if (type == 1 || type == 2 || type == 4) {
        st.setFFlag(h->face(), kFToBeRemoved);
        st.setSign(tgt(g), 1);
    } else if (type == 3) {
        st.setFFlag(h->face(), kFToBeRemoved);
        st.setSign(tgt(g), 2);
    } else if (type == 5 || type == 8) {
        st.setSign(tgt(g), 1);
        st.setSign(tgt(nxt(g)), 2);
        h = prv(h);
        h = split(h, g);
        st.setFFlag(h->face(), kFToBeRemoved);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
    } else if (type == 6 || type == 7) {
        st.setSign(tgt(g), 2);
        st.setSign(tgt(nxt(g)), 1);
        g = nxt(g);
        h = split(h, g);
        st.setFFlag(h->face(), kFToBeRemoved);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
    } else if (type == 9 || type == 12) {
        st.setSign(tgt(g), 1);
        st.setSign(tgt(nxt(g)), 2);
        st.setSign(tgt(nxt(nxt(g))), 1);
        h = prv(h);
        h = split(h, g);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
        g = nxt(nxt(h));
        h = split(h, g);
        st.setFFlag(h->face(), kFToBeRemoved);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
    } else if (type == 10) {
        st.setSign(tgt(g), 1);
        st.setSign(tgt(nxt(g)), 2);
        st.setSign(tgt(nxt(nxt(g))), 1);
        g = h;
        h = prv(prv(h));
        h = split(h, g);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
        g = nxt(h);
        h = prv(h);
        h = split(h, g);
        st.setFFlag(h->face(), kFToBeRemoved);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
    } else if (type == 11) {
        st.setSign(tgt(g), 2);
        st.setSign(tgt(nxt(g)), 1);
        st.setSign(tgt(nxt(nxt(g))), 2);
        g = nxt(g);
        h = split(h, g);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
        g = nxt(nxt(h));
        h = split(h, g);
        st.setFFlag(h->face(), kFToBeRemoved);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
    } else if (type == 13 || type == 16) {
        st.setSign(tgt(g), 1);
        st.setSign(tgt(nxt(g)), 2);
        st.setSign(tgt(nxt(nxt(g))), 1);
        st.setSign(tgt(nxt(nxt(nxt(g)))), 2);
        h = prv(h);
        h = split(h, g);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
        g = nxt(nxt(h));
        h = split(h, g);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
        g = nxt(nxt(h));
        h = split(h, g);
        st.setFFlag(h->face(), kFToBeRemoved);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
    } else if (type == 14 || type == 15) {
        st.setSign(tgt(g), 2);
        st.setSign(tgt(nxt(g)), 1);
        st.setSign(tgt(nxt(nxt(g))), 2);
        st.setSign(tgt(nxt(nxt(nxt(g)))), 1);
        g = nxt(g);
        h = split(h, g);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
        g = nxt(nxt(h));
        h = split(h, g);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
        g = nxt(nxt(h));
        h = split(h, g);
        st.setFFlag(h->face(), kFToBeRemoved);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
    }
    // type == 0：Find_Type 未覆盖组合（默认运行点不应出现；交给调用方判失败）。
}

/// 流形性判据（参考 `Is_Manifold_Property_Violated` 的忠实翻译；环心按 type 变）。
inline bool manifoldViolated(MCGAL::Halfedge* h, int type, int valence) {
    if (valence == 3) return false;
    std::vector<int> P(valence, -1);
    MCGAL::Halfedge* g = h;
    if (!g) return false;
    P[0] = tgt(g) ? static_cast<int>(tgt(g)->poolId()) : -1;
    g = nxt(g);
    for (int i = 1; i < valence; ++i) {
        if (!g) break;
        g = prv(opp(g));
        if (!g) break;
        MCGAL::Vertex* src = g->vertex();
        P[i] = src ? static_cast<int>(src->poolId()) : -1;
    }
    auto circHits = [&P](MCGAL::Halfedge* e, int k) -> bool {
        if (!e || k < 0 || k >= static_cast<int>(P.size()) || P[k] < 0) return false;
        MCGAL::Vertex* c = tgt(e);
        if (!c) return false;
        MCGAL::Halfedge* hs[256];
        const int n = ringNeighbors(c, hs, 256);
        for (int i = 0; i < n; ++i) {
            MCGAL::Vertex* nb = tgt(hs[i]);
            if (nb && static_cast<int>(nb->poolId()) == P[k]) return true;
        }
        return false;
    };
    if (valence == 4) {
        if (type == 5 || type == 8) return circHits(opp(h), 1);
        if (type == 6 || type == 7) return circHits(h, 2);
    } else if (valence == 5) {
        if (type == 9 || type == 12)
            return circHits(opp(h), 1) || circHits(nxt(opp(nxt(h))), 3);
        if (type == 10) return circHits(h, 3) || circHits(nxt(opp(nxt(h))), 3);
        if (type == 11) return circHits(h, 2) || circHits(opp(h), 2);
    } else if (valence == 6) {
        if (type == 13 || type == 16)
            return circHits(opp(h), 1) || circHits(opp(h), 3) ||
                   circHits(nxt(opp(nxt(h))), 3);
        if (type == 14 || type == 15)
            return circHits(h, 2) || circHits(h, 4) || circHits(nxt(opp(prv(h))), 2);
    }
    return false;
}

// ---- 解码端原语（fan 区域并回 polygon / 插中心点）---------------------------
/// `Remove_Edges` 的忠实直译（各 type 的 join 顺序逐行）；
/// 返回 false = 区域不再是那批三角形（编码/解码 traversal 分岔的信号）。
inline bool removeEdges(MCGAL::Mesh* mesh, MCGAL::Halfedge* h, int type) {
    if (!mesh || !h) return false;
    auto nx = [](MCGAL::Halfedge* e) { return e ? e->next() : nullptr; };
    auto pv = [](MCGAL::Halfedge* e) -> MCGAL::Halfedge* {
        if (!e) return nullptr;
        MCGAL::Halfedge* g = e;
        for (int i = 0; i < 512; ++i) {
            MCGAL::Halfedge* n = g->next();
            if (!n) return nullptr;
            if (n == e) return g;
            g = n;
        }
        return nullptr;
    };
    auto op = [](MCGAL::Halfedge* e) { return e ? e->opposite() : nullptr; };
    auto J = [&](MCGAL::Halfedge* e) -> MCGAL::Halfedge* {
        return (e && !e->isRemoved()) ? mesh->join_face(e) : nullptr;
    };
    if (type == 1 || type == 2 || type == 4 || type == 3) return true;
    if (type == 5 || type == 8) {
        MCGAL::Halfedge* g = J(pv(h));
        return g != nullptr || trueFaceDeg(h) == 3;
    }
    if (type == 6 || type == 7) {
        MCGAL::Halfedge* g = J(nx(h));
        return g != nullptr;
    }
    if (type == 9 || type == 12) {
        MCGAL::Halfedge* g = J(pv(h));
        if (!g) return false;
        g = J(nx(g));
        return g != nullptr;
    }
    if (type == 10) {
        MCGAL::Halfedge* g = J(op(nx(h)));
        if (!g) return false;
        g = J(g);
        return g != nullptr;
    }
    if (type == 11) {
        MCGAL::Halfedge* g = J(nx(h));
        if (!g) return false;
        g = J(pv(g));
        return g != nullptr;
    }
    if (type == 13 || type == 16) {
        MCGAL::Halfedge* g = J(op(pv(h)));
        if (!g) return false;
        g = J(g);
        if (!g) return false;
        g = J(g);
        return g != nullptr;
    }
    if (type == 14 || type == 15) {
        MCGAL::Halfedge* g = J(op(nx(h)));
        if (!g) return false;
        g = J(g);
        if (!g) return false;
        g = J(g);
        return g != nullptr;
    }
    return false;
}

/// 解码端 sign 写入（与编码端 retriangulate 各支的 sign 对应的**净效果**）。
inline void writeRemoveEdgesSigns(MCGAL::Halfedge* h, int type,
                                  const std::function<void(MCGAL::Vertex*, int)>& put) {
    if (!h) return;
    auto nx = [](MCGAL::Halfedge* e) { return e ? e->next() : nullptr; };
    auto S = [&](MCGAL::Halfedge* e, int sg) { put(tgt(e), sg); };
    const int PLUS = 1, MINUS = 2;
    if (type == 1 || type == 2 || type == 4)
        S(nx(h), PLUS);
    else if (type == 3)
        S(nx(h), MINUS);
    else if (type == 5 || type == 8) {
        S(nx(h), PLUS);
        S(nx(nx(h)), MINUS);
    } else if (type == 6 || type == 7) {
        S(nx(h), MINUS);
        S(nx(nx(h)), PLUS);
    } else if (type == 9 || type == 12 || type == 10) {
        S(nx(h), PLUS);
        S(nx(nx(h)), MINUS);
        S(nx(nx(nx(h))), PLUS);
    } else if (type == 11) {
        S(nx(h), MINUS);
        S(nx(nx(h)), PLUS);
        S(nx(nx(nx(h))), MINUS);
    } else if (type == 13 || type == 16) {
        S(nx(h), PLUS);
        S(nx(nx(h)), MINUS);
        S(nx(nx(nx(h))), PLUS);
        S(nx(nx(nx(nx(h)))), MINUS);
    } else if (type == 14 || type == 15) {
        S(nx(h), MINUS);
        S(nx(nx(h)), PLUS);
        S(nx(nx(nx(h))), MINUS);
        S(nx(nx(nx(nx(h)))), PLUS);
    }
}

/// `erase_center_vertex` 的逆：把多边形扇成一个中心点。
inline MCGAL::Vertex* addCenterVertex(MCGAL::Mesh* mesh, MCGAL::Halfedge* polyHe,
                                      std::vector<MCGAL::Facet*>* facesOut = nullptr) {
    std::vector<MCGAL::Halfedge*> e;
    {
        MCGAL::Halfedge* it = polyHe;
        for (int i = 0; i < 512; ++i) {
            e.push_back(it);
            it = it->next();
            if (!it || it == polyHe) break;
        }
    }
    const int n = static_cast<int>(e.size());
    if (n < 3) return nullptr;
    const int mid = mesh->meshId();
    MCGAL::Facet* originFace = polyHe->face();
    MCGAL::Vertex* c = MCGAL::contextPool.allocateVertexFromPool(mid, 0.f, 0.f, 0.f);
    mesh->add_vertex(c);
    std::vector<MCGAL::Halfedge*> X(n), Y(n);
    for (int i = 0; i < n; ++i) {
        MCGAL::Vertex* vi = e[i]->vertex();
        MCGAL::Vertex* vip1 = e[i]->end_vertex();
        X[i] = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(mid, vip1, c);
        Y[i] = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(mid, c, vi);
    }
    for (int i = 0; i < n; ++i) {
        const int j = (i + 1) % n;
        X[i]->setOpposite(Y[j]);
        Y[j]->setOpposite(X[i]);
        e[i]->setNext(X[i]);
        X[i]->setNext(Y[i]);
        Y[i]->setNext(e[i]);
    }
    for (int i = 0; i < n; ++i) {
        MCGAL::Vertex* srcX = X[i]->vertex();
        if (srcX != nullptr && srcX->proxyHalfedge() == nullptr)
            srcX->setProxyHalfedge(X[i]);
        if (srcX != nullptr) srcX->halfedges().push_back(X[i]);
        if (c->proxyHalfedge() == nullptr) c->setProxyHalfedge(Y[i]);
        c->halfedges().push_back(Y[i]);
    }
    for (int i = 0; i < n; ++i) {
        MCGAL::Facet* f =
            (i == 0) ? originFace : MCGAL::contextPool.allocateFaceFromPool(mid);
        f->reset(e[i]);
        if (i != 0) mesh->add_face(f);
        if (facesOut) facesOut->push_back(f);
    }
    return c;
}

/// 按量化格号精确定位顶点（解码端池号与编码端不可比）。
inline MCGAL::Vertex* vertexByCell(MCGAL::Mesh* mesh, const QuantGrid& g,
                                   const int q[3]) {
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        if (cellFromPos(g, 0, v->x()) == q[0] &&
            cellFromPos(g, 1, v->y()) == q[1] &&
            cellFromPos(g, 2, v->z()) == q[2])
            return v;
    }
    return nullptr;
}

/// 取一条 `tgt == b` 的绕 a 出边（顺序与参考一致，不能传反）。
inline MCGAL::Halfedge* halfedgeBetween(MCGAL::Vertex* a, MCGAL::Vertex* b) {
    if (!a || !b) return nullptr;
    MCGAL::Halfedge* s = a->proxyHalfedge();
    if (!s) return nullptr;
    MCGAL::Halfedge* it = s;
    for (int i = 0; i < 512; ++i) {
        if (it->end_vertex() == b) return it;
        MCGAL::Halfedge* o = it->opposite();
        if (!o) return nullptr;
        it = o->next();
        if (!it || it == s) return nullptr;
    }
    return nullptr;
}

/// canonical seed：全网格中「端点量化格坐标字典序最小」的边（无 rand，与进程
/// 历史无关）；tie-break 用 (q0,q1,poolId)。取该边属于某 live facet 的 proxy 半边。
inline MCGAL::Halfedge* canonicalSeed(MCGAL::Mesh* mesh, const QuantGrid& g) {
    if (!mesh) return nullptr;
    MCGAL::Halfedge* best = nullptr;
    long long bestKey[7] = {0, 0, 0, 0, 0, 0, 0};
    auto eval = [&](MCGAL::Halfedge* h) {
        if (h == nullptr || h->isRemoved()) return;
        MCGAL::Vertex* a = h->vertex();
        MCGAL::Vertex* b = h->end_vertex();
        if (a == nullptr || b == nullptr || a->isRemoved() || b->isRemoved()) return;
        long long qa[3] = {cellFromPos(g, 0, a->x()), cellFromPos(g, 1, a->y()),
                           cellFromPos(g, 2, a->z())};
        long long qb[3] = {cellFromPos(g, 0, b->x()), cellFromPos(g, 1, b->y()),
                           cellFromPos(g, 2, b->z())};
        // 规范化：字典序小的端点在前，便于比较对称边。
        bool swap = false;
        for (int k = 0; k < 3; ++k) {
            if (qa[k] != qb[k]) {
                swap = qb[k] < qa[k];
                break;
            }
        }
        long long key[7];
        if (!swap) {
            key[0] = qa[0]; key[1] = qa[1]; key[2] = qa[2];
            key[3] = qb[0]; key[4] = qb[1]; key[5] = qb[2];
        } else {
            key[0] = qb[0]; key[1] = qb[1]; key[2] = qb[2];
            key[3] = qa[0]; key[4] = qa[1]; key[5] = qa[2];
        }
        key[6] = h->poolId();
        auto less = [](const long long* x, const long long* y) {
            for (int k = 0; k < 7; ++k) {
                if (x[k] != y[k]) return x[k] < y[k];
            }
            return false;
        };
        if (best == nullptr || less(key, bestKey)) {
            best = h;
            for (int k = 0; k < 7; ++k) bestKey[k] = key[k];
        }
    };
    for (MCGAL::Facet* f : mesh->faces()) {
        if (f == nullptr || f->isRemoved()) continue;
        eval(f->proxyHalfedge());
    }
    return best;
}

// ---- 几何：两端同源的取环 / 预测 / Frenet lifting --------------------------
/// 一圈邻居的 (世界坐标, 量化格号) + 整数 barycenter。
struct RingGeom {
    int n = 0;
    double pos[3 * 256] = {0.0};
    int cell[256][3] = {{0, 0, 0}};
    int bary[3] = {0, 0, 0};
};

/// 取环 + 量化 + barycenter（两端必须用**同一函数**：取环次序与舍入完全一致）。
inline RingGeom ringGeometry(MCGAL::Vertex* v, const QuantGrid& g) {
    RingGeom rg;
    MCGAL::Halfedge* hs[256];
    const int n = ringNeighbors(v, hs, 256);
    rg.n = n;
    long long sum[3] = {0, 0, 0};
    for (int i = 0; i < n; ++i) {
        MCGAL::Vertex* w = tgt(hs[i]);
        for (int k = 0; k < 3; ++k) {
            const double c = (w == nullptr)
                                 ? 0.0
                                 : (k == 0 ? w->x() : (k == 1 ? w->y() : w->z()));
            rg.pos[3 * i + k] = c;
            const int q = cellFromPos(g, k, c);
            rg.cell[i][k] = q;
            sum[k] += q;
        }
    }
    if (n > 0) {
        for (int k = 0; k < 3; ++k) {
            const long long s = sum[k];
            rg.bary[k] = static_cast<int>((s + (long long)n / 2) / n);
        }
    }
    return rg;
}

/// 编码端：被删点 vdel 的残差（整数域，已过 Frenet forward）。
inline MCGAL::PointInt captureResidual(MCGAL::Vertex* vdel, const QuantGrid& g) {
    MCGAL::PointInt out(0, 0, 0);
    if (vdel == nullptr || !g.valid()) return out;
    const RingGeom rg = ringGeometry(vdel, g);
    int d[3];
    for (int k = 0; k < 3; ++k) {
        const double c = (k == 0 ? vdel->x() : (k == 1 ? vdel->y() : vdel->z()));
        d[k] = cellFromPos(g, k, c) - rg.bary[k];
    }
    const RdFrame f = frameFromRingPositions(rg.pos, rg.n, g);
    int x = d[0], y = d[1], z = d[2];
    if (f.valid) frameForward(x, y, z, f);
    out = MCGAL::PointInt(x, y, z);
    return out;
}

/// 解码端：把 encoded 残差逆变换并写回中心点 nv 的世界坐标。
inline void applyResidual(MCGAL::Vertex* nv, const QuantGrid& g,
                          const MCGAL::PointInt& encoded) {
    if (nv == nullptr || !g.valid()) return;
    const RingGeom rg = ringGeometry(nv, g);
    const RdFrame f = frameFromRingPositions(rg.pos, rg.n, g);
    int x = encoded.x(), y = encoded.y(), z = encoded.z();
    if (f.valid) frameInverse(x, y, z, f);
    int q[3] = {rg.bary[0] + x, rg.bary[1] + y, rg.bary[2] + z};
    nv->setPoint(MCGAL::Point(cellCenter(g, 0, q[0]), cellCenter(g, 1, q[1]),
                              cellCenter(g, 2, q[2])));
}

// ---- 编码端：一层 decimating conquest --------------------------------------
struct EncodeLayerResult {
    int removed = 0;
    int nullPatches = 0;
    int pops = 0;
};

/// 跑一层（从 seed 起的前沿征服），把连通性符号（0..3 = valence-3，4 = null）
/// 与每个被删点的几何残差按 emit 顺序写出。
inline EncodeLayerResult encodeLayer(MCGAL::Mesh* mesh, State& st,
                                     MCGAL::Halfedge* seed, const QuantGrid& g,
                                     std::vector<int>& syms,
                                     std::vector<MCGAL::PointInt>& residuals) {
    EncodeLayerResult res;
    std::deque<MCGAL::Halfedge*> q;
    st.reset();
    st.setFlag(tgt(seed), 1);
    st.setSign(tgt(seed), 1);
    st.setFlag(tgt(opp(seed)), 1);
    st.setSign(tgt(opp(seed)), 2);
    q.push_back(seed);
    auto emit = [&](int sym) {
        syms.push_back(sym);
        ++res.pops;
    };
    while (!q.empty()) {
        MCGAL::Halfedge* h = q.front();
        q.pop_front();
        if (h == nullptr) continue;
        const std::uint8_t ff = st.fflag(h->face());
        if (ff == kFConquered || ff == kFToBeRemoved) continue;
        MCGAL::Halfedge* nh = nxt(h);
        MCGAL::Vertex* vFront = tgt(nh);
        if (vFront == nullptr) continue;
        vFront->recompute_degree();
        const int valence = vFront->vertex_degree();
        const bool isBorder = isBorderVertex(nh);
        const bool conquered = (st.flag(vFront) != 0);
        const bool canDecimate = (valence >= 3 && valence <= 6 && !isBorder &&
                                  !conquered);
        if (canDecimate) {
            const int type = findType(st, h, valence);
            if (type != 0 && !manifoldViolated(h, type, valence)) {
                ++res.removed;
                emit(valence - 3);
                residuals.push_back(captureResidual(vFront, g));
                MCGAL::Halfedge* gg = h;
                mesh->erase_center_vertex(nxt(gg));
                gg = h;
                if (gg->face()) st.setFFlag(gg->face(), kFTemp);
                for (int j = 0; j < valence - 1; ++j) {
                    gg = nxt(gg);
                    if (!gg) break;
                    if (tgt(gg)) st.setFlag(tgt(gg), 1);
                    if (tgt(opp(gg))) st.setFlag(tgt(opp(gg)), 1);
                    if (!isBorderEdge(gg)) q.push_back(opp(gg));
                }
                retriangulate(mesh, st, h, valence, type);
                continue;
            }
            // 可删但 type/manifold 不通过 ⇒ null patch（落下去，与参考一致）。
            ++res.nullPatches;
            emit(4);
            if (h->face()) st.setFFlag(h->face(), kFConquered);
            st.setFlag(vFront, 1);
            nullPatchTail(st, h, q);
            continue;
        }
        // 兜底分支：不可删 ⇒ null patch（尾部与上支逐字相同）。
        ++res.nullPatches;
        emit(4);
        if (h->face()) st.setFFlag(h->face(), kFConquered);
        st.setFlag(vFront, 1);
        nullPatchTail(st, h, q);
    }
    return res;
}

// ---- 解码端：一层的逆（base mesh 上 replay）---------------------------------
struct DecodeLayerResult {
    bool ok = true;
    int inserted = 0;
    int nullPatches = 0;
    int consumed = 0;
};

inline DecodeLayerResult decodeLayer(MCGAL::Mesh* mesh, State& st,
                                     MCGAL::Halfedge* seed, const QuantGrid& g,
                                     const std::vector<int>& syms,
                                     const std::vector<MCGAL::PointInt>& residuals) {
    DecodeLayerResult res;
    std::deque<MCGAL::Halfedge*> q;
    st.reset();
    st.setFlag(tgt(seed), 1);
    st.setSign(tgt(seed), 1);
    st.setFlag(tgt(opp(seed)), 1);
    st.setSign(tgt(opp(seed)), 2);
    q.push_back(seed);
    std::size_t si = 0;
    std::size_t ri = 0;
    while (!q.empty()) {
        MCGAL::Halfedge* h = q.front();
        q.pop_front();
        if (h == nullptr) continue;
        const std::uint8_t ff = st.fflag(h->face());
        if (ff == kFConquered || ff == kFToBeRemoved) continue;
        if (si >= syms.size()) break;  // 本层符号耗尽 = 层结束
        const int s = syms[si++];
        MCGAL::Vertex* vFront = tgt(nxt(h));
        if (s >= 0 && s <= 3) {
            const int valence = s + 3;
            const int type = findType(st, h, valence);
            if (type == 0 || !removeEdges(mesh, h, type)) {
                res.ok = false;
                break;
            }
            writeRemoveEdgesSigns(h, type, [&](MCGAL::Vertex* v, int sg) {
                if (v) st.setSign(v, static_cast<std::uint8_t>(sg));
            });
            std::vector<MCGAL::Facet*> regionFaces;
            // 编码端 erase 之后多边形即 h->face()（inverseRetriangulate 在参考
            // 实现里是空操作，只为出参保留 ⇒ 这里直接取之，语义等价）。
            if (h->face()) regionFaces.push_back(h->face());
            if (h->face()) st.setFFlag(h->face(), kFToBeRemoved);
            MCGAL::Halfedge* gw = h;
            for (int j = 0; j < valence - 1; ++j) {
                gw = nxt(gw);
                if (!gw) break;
                if (tgt(gw)) st.setFlag(tgt(gw), 1);
                if (tgt(opp(gw))) st.setFlag(tgt(opp(gw)), 1);
                if (!isBorderEdge(gw)) q.push_back(opp(gw));
            }
            for (MCGAL::Facet* f : regionFaces) st.setFFlag(f, kFToBeRemoved);
            std::vector<MCGAL::Facet*> newFaces;
            MCGAL::Vertex* nv = addCenterVertex(mesh, h, &newFaces);
            for (MCGAL::Facet* f : newFaces) st.setFFlag(f, kFToBeRemoved);
            if (nv == nullptr) {
                res.ok = false;
                break;
            }
            if (ri < residuals.size()) {
                applyResidual(nv, g, residuals[ri++]);
            }
            st.setFlag(nv, 1);
            ++res.inserted;
        } else {
            if (h->face()) st.setFFlag(h->face(), kFConquered);
            st.setFlag(vFront, 1);
            nullPatchTail(st, h, q);
            ++res.nullPatches;
        }
    }
    res.consumed = static_cast<int>(si);
    return res;
}

}  // namespace rd_canon

#endif  // PMSF_IMPL_RD_RD_CANONICAL_TRAVERSAL_H
