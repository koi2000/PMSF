/*
 * @file ConquestMeshPrimitives.h
 * @brief 顶点删减式编解码的**网格原语**（框架原生，**算法无关**）。
 *
 * ⚠️ **验证状态（2026-09-22）**：这些原语已在
 * `tools/ConquestLayerRoundTripProbe.cpp` 的**层往返对拍**中逐层验证通过：
 *   cow_simp / fandisk / cow / bimba_original 四个网格
 *   **全部 `topoGeomOk=1`**（V/F 完全还原、规范面含几何全量对拍一致）；
 *   cow_simp **链式三层**同样全过。
 *
 * **单一来源**：`tools/ConquestMeshPrimitives.h` 只是本文件的转发，
 * 两个探针工具都经它使用同一份实现，**不得各自复制**。
 *
 * 本文件与任何具体算法无关：只提供"删点/插点"两侧的网格级原语，
 * 可调之处一律走 `scheme/EngineHooks.h` 的引擎级 hook。
 */
#ifndef PMSF_CONQUEST_MESH_PRIMITIVES_H
#define PMSF_CONQUEST_MESH_PRIMITIVES_H

#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Point.h"
#include "Vertex.h"
#include "Global.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <vector>

namespace pmsf {

/// @brief 面的**真实**环长。
///
/// ⚠️ 不要用 `Facet::facet_degree()`：它是**存储计数** `facet_degree_`（默认 3），
/// `split_facet` 与 `join_face` 都**不维护**它。本轮实测踩到：同一表达式读存储
/// 计数报 1、读真实环长报 0，一度被当成引擎分歧。所有度数判断一律走本函数。
inline int trueFaceDeg(MCGAL::Halfedge* h) {
    if (!h || !h->face() || h->face()->isRemoved())
        return -1;
    int n = 0;
    MCGAL::Halfedge* it = h;
    for (int i = 0; i < 20000; ++i) {   // 护栏调大：把"环长超出护栏"与"环真不闭合"分开
        ++n;
        it = it->next();
        if (!it || it == h)
            break;
    }
    return n;
}

/// @brief a、b 之间是否已存在边（沿 a 的有界出边环找 b）。
inline bool edgeExists(MCGAL::Vertex* a, MCGAL::Vertex* b) {
    if (!a || !b)
        return false;
    MCGAL::Halfedge* s = a->proxyHalfedge();
    if (!s)
        return false;
    MCGAL::Halfedge* it = s;
    for (int i = 0; i < 512; ++i) {
        if (it->end_vertex() == b || it->vertex() == b)
            return true;
        MCGAL::Halfedge* o = it->opposite();
        if (!o)
            break;
        it = o->next();
        if (!it || it == s)
            break;
    }
    return false;
}

/// @brief 逆重三角化：把被切碎的区域并回多边形。
///
/// 反复 `join_face` 掉「gate 所在面上、对面是三角形、**poolId 最大**」的那条边，
/// 直到 `face(gate)` 的真实度 == `valence`。
///
/// **不需要重放编码端 `retriangulate` 的导航链** —— 这是本判据最大的价值。
/// 依据：MCGAL 的半边池是**单调 bump 分配器**（从不回收），区域内的边在
/// 本区域的局部范围内必然比多边形原有边更晚分配，故池号最大者即内边。
///
/// @return 是否成功并回成度恰好为 `valence` 的多边形。
inline bool inverseRetriangulate(MCGAL::Mesh* mesh,
                                 MCGAL::Halfedge* gate,
                                 int valence,
                                 std::vector<MCGAL::Facet*>* absorbedOut = nullptr) {
    int guard = 0;
    while (trueFaceDeg(gate) >= 0 && trueFaceDeg(gate) < valence) {
        if (++guard > 512)
            return false;
        MCGAL::Halfedge* pick = nullptr;
        long long best = -1;
        MCGAL::Halfedge* it = gate;
        for (int i = 0; i < 512; ++i) {
            // **排除 gate 自身**：它是编码端 erase 之后多边形上的边界边
            // （`retriangulate` 以它为环心），join 掉就是把区域与外部合并。
            if (it == gate) {
                it = it->next();
                if (!it || it == gate)
                    break;
                continue;
            }
            MCGAL::Halfedge* o = it->opposite();
            if (o && o->face() && !o->face()->isRemoved() && trueFaceDeg(o) == 3) {
                const long long id = (long long)it->poolId();
                if (id > best) {
                    best = id;
                    pick = it;
                }
            }
            it = it->next();
            if (!it || it == gate)
                break;
        }
        if (!pick)
            return false;  // 找不到可 join 的边
        if (absorbedOut && pick->opposite())
            absorbedOut->push_back(pick->opposite()->face());  // 被并掉的那个面
                mesh->join_face(pick);
    }
    // 加入"没被并掉、留下来当多边形"的那一个：被并的 (valence-3) 个 + 这一个
    // = **valence-2 个**，恰等于编码端 `retriangulate` 标记的面数（逐支吻合）。
    if (absorbedOut && gate && gate->face())
        absorbedOut->push_back(gate->face());
    return trueFaceDeg(gate) == valence;
}

/// @brief `erase_center_vertex` 的逆（MCGAL 没有，自建）：把多边形扇成一个中心点。
///
/// 记多边形环为 e_0..e_{n-1}（e_i : v_i -> v_{i+1}）。插入中心点 c 后，
/// 三角形 T_i = (v_i, v_{i+1}, c)，其半边为 e_i -> X_i -> Y_i -> e_i，其中
///   X_i : v_{i+1} -> c ,  Y_i : c -> v_i ,  X_i.opposite = Y_{i+1}
///
/// 三处**非显然但必需**的写法（都是踩出来的）：
///   ① 用 `allocateHalfedgeFromPoolWithOutInit`。带配对的版本（`reset`）会在
///      目标点的环里搜可配对的半边并**断言** —— 而这里要手工把整个扇面接好。
///   ② 必须**手工补顶点登记**：`reset_without_init` 只设两个端点，不做 `reset`
///      里的 `setProxyHalfedge` 与 `halfedges().push_back`。不补则新点度为 0、
///      环遍历为空（V/F 已还原也照样是错的 —— 实测踩到）。
///   ③ 面必须用**裸分配**版 `allocateFaceFromPool(index)`。带 vs 的版本内部调
///      `f->reset(vts)`，会边走边 `Halfedge::reset` **自动配对**，撞上 ① 的断言。
///
/// @return 新建的中心点；多边形退化（n < 3）时返回 nullptr。
inline MCGAL::Vertex* addCenterVertex(MCGAL::Mesh* mesh, MCGAL::Halfedge* polyHe,
                                      std::vector<MCGAL::Facet*>* facesOut = nullptr) {
    std::vector<MCGAL::Halfedge*> e;
    {
        MCGAL::Halfedge* it = polyHe;
        for (int i = 0; i < 512; ++i) {
            e.push_back(it);
            it = it->next();
            if (!it || it == polyHe)
                break;
        }
    }
    const int n = (int)e.size();
    if (n < 3)
        return nullptr;
    const int mid = mesh->meshId();
    MCGAL::Facet* originFace = polyHe->face();

    MCGAL::Vertex* c = MCGAL::contextPool.allocateVertexFromPool(mid, 0.f, 0.f, 0.f);
    mesh->add_vertex(c);

    std::vector<MCGAL::Halfedge*> X(n), Y(n);
    for (int i = 0; i < n; ++i) {
        MCGAL::Vertex *vi = e[i]->vertex(), *vip1 = e[i]->end_vertex();
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
        MCGAL::Vertex* srcX = X[i]->vertex();  // v_{i+1}
        if (srcX->proxyHalfedge() == nullptr)
            srcX->setProxyHalfedge(X[i]);
        srcX->halfedges().push_back(X[i]);
        if (c->proxyHalfedge() == nullptr)
            c->setProxyHalfedge(Y[i]);
        c->halfedges().push_back(Y[i]);
    }
    for (int i = 0; i < n; ++i) {
        MCGAL::Facet* f = (i == 0) ? originFace : MCGAL::contextPool.allocateFaceFromPool(mid);
        f->reset(e[i]);
        if (i != 0)
            mesh->add_face(f);
        if (facesOut)
            facesOut->push_back(f);
    }
    return c;
}


/// @brief `Remove_Edges` 的**忠实直译**（参考实现 `Compression_Valence_Component.inl:9456`）。
///
/// 这是解码端"把切碎的区域并回多边形"的正解。与 `inverseRetriangulate` 的差别：
///
/// | | 本函数（参考做法） | inverseRetriangulate（我此前的做法） |
/// |---|---|---|
/// | 取位 | **跟着 `join_face` 返回的半边**走 prev/next | 在原多边形下标上算 m-1/0/2 |
/// | 起点 | 不需要旋转，`h` 就是原 gate | 需把环旋到"gate 的源顶点" |
/// | 锚点 | **完全不需要** | 依赖 origAnchor |
///
/// ⇒ 参考的做法不依赖"环的 0 号在哪"，也不需要 §21 引入的锚点旋转
/// （而那正是 §27/§30 反复出问题的地方）。
///
/// 逐支取位**逐行抄自参考**，不得凭构图推：
///   type 1,2,4 / 3 (V=3)：不 join（多边形本就是三角形）
///   type 5,8  (V=4)：prev(h) 起，1 刀
///   type 6,7  (V=4)：next(h) 起，1 刀
///   type 9,12 (V=5)：prev(h) 起，2 刀
///   type 10   (V=5)：opposite(next(h)) 起，2 刀
///   type 11   (V=5)：next(h) 起，2 刀
///   type 13,16(V=6)：opposite(prev(h)) 起，3 刀
///   type 14,15(V=6)：opposite(next(h)) 起，3 刀
inline bool removeEdges(MCGAL::Mesh* mesh, MCGAL::Halfedge* h, int type) {
    if (!mesh || !h)
        return false;
    auto nx = [](MCGAL::Halfedge* e) { return e ? e->next() : nullptr; };
    auto pv = [](MCGAL::Halfedge* e) {
        if (!e)
            return (MCGAL::Halfedge*)nullptr;
        MCGAL::Halfedge* g = e;
        for (int i = 0; i < 512; ++i) {
            MCGAL::Halfedge* n = g->next();
            if (!n)
                return (MCGAL::Halfedge*)nullptr;
            if (n == e)
                return g;
            g = n;
        }
        return (MCGAL::Halfedge*)nullptr;
    };
    auto op = [](MCGAL::Halfedge* e) { return e ? e->opposite() : nullptr; };
    auto J = [&](MCGAL::Halfedge* e) -> MCGAL::Halfedge* {
        return (e && !e->isRemoved()) ? mesh->join_face(e) : nullptr;
    };

    if (type == 1 || type == 2 || type == 4 || type == 3)
        return true;   // V=3：多边形本就是三角形，不 join
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
        g = J(nx(g));
        return g != nullptr;
    }
    if (type == 10) {
        MCGAL::Halfedge* g = J(op(nx(h)));
        g = J(g);
        return g != nullptr;
    }
    if (type == 11) {
        MCGAL::Halfedge* g = J(nx(h));
        g = J(pv(g));
        return g != nullptr;
    }
    if (type == 13 || type == 16) {
        MCGAL::Halfedge* g = J(op(pv(h)));
        g = J(g);
        g = J(g);
        return g != nullptr;
    }
    if (type == 14 || type == 15) {
        MCGAL::Halfedge* g = J(op(nx(h)));
        g = J(g);
        g = J(g);
        return g != nullptr;
    }
    return false;   // type == 0：不应出现
}


/// @brief `Remove_Edges` 里的 **sign 写入**（参考 `.inl:9456` 各支末尾，**同一函数内**）。
///
/// ⚠️ 与"并边"的关系：参考在 join 之后用 `g = h`（**原 gate**）直接写 sign ——
/// 因为它 join 的是 `prev(h)` / `next(h)` / `opposite(...)`，**`h` 本身不会被移除**
/// ⇒ **不需要任何锚点旋转**。这正是 §21/§27/§30 反复出问题的那个假设被消掉的地方。
inline void writeRemoveEdgesSigns(MCGAL::Halfedge* h, int type,
                                  const std::function<void(MCGAL::Vertex*, int)>& put) {
    if (!h)
        return;
    auto nx = [](MCGAL::Halfedge* e) { return e ? e->next() : nullptr; };
    auto tgt = [](MCGAL::Halfedge* e) { return e ? e->end_vertex() : nullptr; };
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

/// @brief 按坐标找回顶点（解码端的池号与编码端**不可比**，只能按坐标找）。
/// 取欧氏距离最近者；网格为空返回 nullptr。
inline MCGAL::Vertex* nearestVertex(MCGAL::Mesh* mesh, const double p[3]) {
    MCGAL::Vertex* best = nullptr;
    double bd = 1e300;
    if (!mesh)
        return nullptr;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved())
            continue;
        const MCGAL::Point& q = v->point();
        const double d = (q.x() - p[0]) * (q.x() - p[0]) + (q.y() - p[1]) * (q.y() - p[1]) +
                         (q.z() - p[2]) * (q.z() - p[2]);
        if (d < bd) {
            bd = d;
            best = v;
        }
    }
    return best;
}

/// @brief 取一条 `tgt == b` 的绕 `a` 出边。
///
/// ⚠️ **返回的是 tgt 为 `b` 的那条半边** —— 想还原一条种子半边 `seed`（其
/// `tgt(seed) == s0`、`tgt(opp(seed)) == s1`）必须传 `(s1 的顶点, s0 的顶点)`。
/// 传反了就拿到**反向**半边 ⇒ PLUS/MINUS 互换 ⇒ sign 传播全线不同 ⇒ 拓扑分岔。
/// （探针阶段实测：这一个反向就解释了当时全部症状，见蓝图 §9。）
inline MCGAL::Halfedge* halfedgeBetween(MCGAL::Vertex* a, MCGAL::Vertex* b) {
    if (!a || !b)
        return nullptr;
    MCGAL::Halfedge* s = a->proxyHalfedge();
    if (!s)
        return nullptr;
    MCGAL::Halfedge* it = s;
    for (int i = 0; i < 512; ++i) {
        if (it->end_vertex() == b)
            return it;
        MCGAL::Halfedge* o = it->opposite();
        if (!o)
            return nullptr;
        it = o->next();
        if (!it || it == s)
            return nullptr;
    }
    return nullptr;
}

}  // namespace pmsf

#endif  // PMSF_CONQUEST_MESH_PRIMITIVES_H
