// SPDX-License-Identifier: Apache-2.0
//
// tools/ConquestLayerRoundTripProbe.cpp —— P13.4 (C) 解码端**遍历本体**的往返探针。
//
// 与 CenterVertexRoundTripProbe 的分工：
//   那个验的是**原语**（一次 erase → split×k → join×k → add 的互逆性）；
//   本文件验的是**遍历**：一整层的 gate 队列能不能被逐符号倒着走回去。
//
// 结构（全部落盘，解码端从**快照**重启，不共享任何内存状态）：
//   ① 载入原网格 → dump 成规范 OFF（对照基准）
//   ② 载入原网格 → decimateLayer(一层) → dump 删减后网格 + 符号流
//   ③ 载入**删减后网格** → decodeLayer(符号流) → dump 结果
//   ④ ③ 与 ① 对拍：V / F / 顶点位置多重集
//
// ⚠️ ③ 必须从**落盘的快照**重启，不能复用 ② 留下的内存对象 ——
// 复用会让"解码端真的能独立重建"这件事无从检验（本库的仪表误判多半出在这类共享上）。
//
// 种子语义：编码端选定 seed 后，把它的两个端点**坐标**记下来；解码端按坐标
// 在快照网格里找回同一个半边。删减不改存活点的坐标，所以这条对得上。
// （真 codec 里种子是随码流传的，探针用坐标代替，只为免去写容器。）
#include "conquest/ConquestMeshPrimitives.h"
#include "conquest/ConquestDecodeLayer.h"
#include "Facet.h"
#include "Global.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Point.h"
#include "Vertex.h"
#include "conquest/FrontierConquestEngine.h"
#include "core/MeshRuntime.h"

#include <algorithm>
#include <cmath>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <functional>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

using pmsf::ConquestState;
using pmsf::trueFaceDeg;

// ---- 与引擎匿名命名空间逐字一致的中性别名（引擎里不可见，只能在此复刻）------
namespace {

inline MCGAL::Vertex* tgt(MCGAL::Halfedge* h) {
    return h ? h->end_vertex() : nullptr;
}
inline MCGAL::Halfedge* nxt(MCGAL::Halfedge* h) {
    return h ? h->next() : nullptr;
}
inline MCGAL::Halfedge* opp(MCGAL::Halfedge* h) {
    return h ? h->opposite() : nullptr;
}
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
    return nullptr;
}
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

constexpr std::uint8_t kFConquered = 1, kFToBeRemoved = 2, kFTemp = 3;

/// 与 FrontierConquestEngine.cpp 的 `findType` **逐字相同**（纯函数，只读 sign）。
inline int findType(const pmsf::ConquestState& st, MCGAL::Halfedge* h, int valence) {
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
    if (k == 0)
        return 0;
    return 4 * (valence - 3) + k;
}

/// 与 FrontierConquestEngine.cpp 的 `nullPatchTail` **逐字相同**。
/// 参考实现里"manifold 违规"与"最终 else"两支的尾部是复制粘贴的孪生体。
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

}  // namespace

// ---- §13 导航重放：确定性重三角化（下标表）---------------------------------
// 多边形用顶点序列 w[0..m-1] 表示（半边 i = w[i]→w[i+1]），**gate 恒在 0 号**。
// 由探针实测反推：split_facet(a,b) 切掉 (end(a), start(b), end(b))，
// 返回的新半边 = end(a)→end(b) = 新区多边形的 0 号。
static void splitCut(std::vector<MCGAL::Vertex*>& w, int aIdx, int bIdx,
                     std::vector<std::array<MCGAL::Vertex*, 3>>& tris) {
    const int m = (int)w.size();
    const int ea = (aIdx + 1) % m, sb = bIdx, eb = (bIdx + 1) % m;
    tris.push_back({w[ea], w[sb], w[eb]});
    std::vector<MCGAL::Vertex*> nw;
    nw.push_back(w[ea]);
    for (int k = eb;; k = (k + 1) % m) {
        nw.push_back(w[k]);
        if (k == aIdx)
            break;
    }
    w = nw;
}

/// 按 type 对多边形做确定性重三角化，返回三角形集合（顶点指针三元组，已排序）。
static std::vector<std::array<MCGAL::Vertex*, 3>> deterministicTriangulate(std::vector<MCGAL::Vertex*> w, int type) {
    std::vector<std::array<MCGAL::Vertex*, 3>> tris;
    auto S = [&](int aIdx, int bIdx) { splitCut(w, aIdx, bIdx, tris); };
    const int m0 = (int)w.size();
    if (type == 5 || type == 8) {
        S(m0 - 1, 1);
    } else if (type == 6 || type == 7 || type == 11 || type == 14 || type == 15) {
        S(0, 2);
        if (type == 11) { S(0, 2); }
        if (type == 14 || type == 15) { S(0, 2); S(0, 2); }
    } else if (type == 9 || type == 12 || type == 13 || type == 16) {
        S(m0 - 1, 1);
        S(0, 2);
        if (type == 13 || type == 16) { S(0, 2); }
    } else if (type == 10) {
        S((m0 - 2 + m0) % m0, 0);
        const int m1 = (int)w.size();
        S(m1 - 1, 1);
    }
    // ⚠️ **必须把最后剩下的多边形也算进去**：每次 splitCut 切掉一个三角形，
    // V 边形切 V-3 刀后剩 3 个顶点 —— 那**本身就是第 V-2 个区域三角形**。
    // 漏掉它会让重放恒少一个三角形（实测：真值区域 3 个、重放只有 2 个），
    // 判据因此永远不可能成立。
    if ((int)w.size() == 3)
        tris.push_back({w[0], w[1], w[2]});
    for (auto& tr : tris)
        std::sort(tr.begin(), tr.end());
    std::sort(tris.begin(), tris.end());
    return tris;
}

/// 与 §13.4 配套：取出 P 内部的三角形集合（三个顶点都在 P 上）。
static std::vector<std::array<MCGAL::Vertex*, 3>> trisInside(MCGAL::Mesh* mesh,
                                                            const std::vector<MCGAL::Vertex*>& poly) {
    std::vector<std::array<MCGAL::Vertex*, 3>> out;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved())
            continue;
        std::vector<MCGAL::Vertex*> vs;
        MCGAL::Halfedge* it = f->proxyHalfedge();
        for (int k = 0; k < 64 && it; ++k) {
            vs.push_back(it->vertex());
            it = it->next();
            if (!it || it == f->proxyHalfedge())
                break;
        }
        if (vs.size() != 3)
            continue;
        bool all = true;
        for (MCGAL::Vertex* v : vs) {
            bool in = false;
            for (MCGAL::Vertex* p : poly)
                if (p == v) {
                    in = true;
                    break;
                }
            if (!in) {
                all = false;
                break;
            }
        }
        if (all) {
            std::array<MCGAL::Vertex*, 3> tr = {vs[0], vs[1], vs[2]};
            std::sort(tr.begin(), tr.end());
            out.push_back(tr);
        }
    }
    std::sort(out.begin(), out.end());
    return out;
}

static std::vector<MCGAL::Vertex*> faceVerts(MCGAL::Halfedge* h) {
    std::vector<MCGAL::Vertex*> vs;
    MCGAL::Halfedge* it = h;
    for (int k = 0; k < 64 && it; ++k) {
        vs.push_back(it->vertex());
        it = it->next();
        if (!it || it == h)
            break;
    }
    return vs;
}

/// **§13.4 恢复（组合模型版）**：先在"三角形↔相邻三角形"邻接表上枚举候选区域
/// （纯整数运算，撤销 = 弹栈，**完全不碰网格**），对每个完整候选跑 §13.4 判定；
/// **选定之后**才对网格执行 join。这样根本不出现"撤销网格操作"（§14.2 的坑）。
static int gCompletion = 0, gCand = 0;
static bool recoverRegion(MCGAL::Mesh* mesh,
                          pmsf::ConquestState& st,
                          MCGAL::Halfedge* gate,
                          int valence,
                          int type,
                          const std::vector<std::array<double, 3>>* trueRing = nullptr,
                          MCGAL::Halfedge** polyAnchorOut = nullptr) {
    // ① 收集候选三角形（从 face(gate) 起做 4 层邻接 BFS）
    std::vector<MCGAL::Facet*> tris;
    std::map<MCGAL::Facet*, int> idx;
    {
        std::vector<MCGAL::Facet*> cur{gate->face()};
        idx[gate->face()] = 0;
        tris.push_back(gate->face());
        for (int lvl = 0; lvl < 4; ++lvl) {
            std::vector<MCGAL::Facet*> nxt;
            for (MCGAL::Facet* f : cur) {
                MCGAL::Halfedge* it = f->proxyHalfedge();
                for (int k = 0; k < 64 && it; ++k) {
                    MCGAL::Halfedge* o = it->opposite();
                    if (o && o->face() && !o->face()->isRemoved() && !idx.count(o->face())) {
                        idx[o->face()] = (int)tris.size();
                        tris.push_back(o->face());
                        nxt.push_back(o->face());
                    }
                    it = it->next();
                    if (!it || it == f->proxyHalfedge())
                        break;
                }
            }
            cur = nxt;
            if (tris.size() > 400)
                break;
        }
    }
    const int n = (int)tris.size();
    if (n < valence - 2)
        return false;

    // ② 邻接表：tri[i] 的三条边各自的对面三角形下标（-1 = 边界）
    std::vector<std::array<int, 3>> adj(n);
    std::vector<std::array<MCGAL::Halfedge*, 3>> edges(n);
    for (int i = 0; i < n; ++i) {
        MCGAL::Halfedge* it = tris[i]->proxyHalfedge();
        int k = 0;
        for (; k < 3 && it; ++k) {
            edges[i][k] = it;
            MCGAL::Halfedge* o = it->opposite();
            MCGAL::Facet* of = o ? o->face() : nullptr;
            auto f = of ? idx.find(of) : idx.end();
            adj[i][k] = (f == idx.end()) ? -1 : f->second;
            it = it->next();
            if (!it || it == tris[i]->proxyHalfedge())
                break;
        }
    }

    gCand = n;
    // ③ 回溯：从 tri[0] 起向外并，直到 |S| == valence-2
    std::vector<bool> inS(n, false);
    const int want = valence - 2;
    std::function<bool(int)> dfs = [&](int have) -> bool {
        if (have == want) {
            // ④ 判定：边界环 + 确定性重放
            std::vector<MCGAL::Vertex*> cyc;
            {
                // 边界环：从任一边界半边出发，每步取"从当前顶点出发的下一条边界边"。
                // ⚠️ 拐角处必须**绕顶点旋转**（`n->opposite()->next()`），
                // 不能按"同一三角形内相邻"配对 —— 后者只在直线段上成立。
                auto isInS = [&](MCGAL::Halfedge* e) {
                    if (!e)
                        return false;
                    MCGAL::Halfedge* o = e->opposite();
                    MCGAL::Facet* of = o ? o->face() : nullptr;
                    auto f = of ? idx.find(of) : idx.end();
                    return f != idx.end() && inS[f->second];
                };
                MCGAL::Halfedge* s0 = nullptr;
                for (int i = 0; i < n && !s0; ++i) {
                    if (!inS[i])
                        continue;
                    for (int k = 0; k < 3; ++k)
                        if (!isInS(edges[i][k])) {
                            s0 = edges[i][k];
                            break;
                        }
                }
                if (!s0)
                    return false;
                MCGAL::Halfedge* it = s0;
                for (int lim = 0; lim < 64; ++lim) {
                    cyc.push_back(it->vertex());
                    MCGAL::Halfedge* nx = it->next();
                    int rot = 0;
                    while (nx && isInS(nx) && ++rot < 64)
                        nx = nx->opposite() ? nx->opposite()->next() : nullptr;
                    if (!nx)
                        return false;
                    it = nx;
                    if (it == s0)
                        break;
                }
                if ((int)cyc.size() != valence)
                    return false;
                // ⚠️ **锚点对齐**：`deterministicTriangulate` 要求多边形 0 号
                // 对应 gate 的源顶点（编码端的取位 m-1/0/2 都是相对 gate 的）。
                // 从 `*bset.begin()` 起的环是任意起点的，必须旋到 gate->vertex()。
                {
                    MCGAL::Vertex* anchor = gate->vertex();
                    int pos = -1;
                    for (int k = 0; k < (int)cyc.size(); ++k)
                        if (cyc[k] == anchor)
                            pos = k;
                    if (pos < 0)
                        return false;
                    std::rotate(cyc.begin(), cyc.begin() + pos, cyc.end());
                }
            }
            // **盘性检查**：S 的所有顶点必须都落在边界环上。
            // 区域是"被删顶点的星"被重三角化后的产物，**没有内部顶点**；
            // 而连通集未必是盘 —— 3 个三角形可以绕一个顶点成"风车"，
            // 那个中心顶点不在边界环上。这样的 S 内边 join 起来是非流形操作，
            // 正是实测"环被打断"的来源（§19.3）。
            {
                for (int i = 0; i < n; ++i) {
                    if (!inS[i])
                        continue;
                    for (int k = 0; k < 3; ++k) {
                        MCGAL::Vertex* v = edges[i][k]->vertex();
                        if (std::find(cyc.begin(), cyc.end(), v) == cyc.end())
                            return false;
                    }
                }
            }
            ++gCompletion;
            auto exp = deterministicTriangulate(cyc, type);
            std::vector<std::array<MCGAL::Vertex*, 3>> got;
            for (int i = 0; i < n; ++i) {
                if (!inS[i])
                    continue;
                std::array<MCGAL::Vertex*, 3> tr = {edges[i][0]->vertex(), edges[i][1]->vertex(),
                                                    edges[i][2]->vertex()};
                std::sort(tr.begin(), tr.end());
                got.push_back(tr);
            }
            std::sort(got.begin(), got.end());
            return exp == got;
        }
        for (int i = 0; i < n; ++i) {
            if (inS[i])
                continue;
            bool touch = false;
            for (int k = 0; k < 3; ++k)
                if (adj[i][k] >= 0 && inS[adj[i][k]])
                    touch = true;
            if (!touch) {
                if (std::getenv("PMSF_CLR_DFSDBG") && have == 1)
                    std::fprintf(stderr, "[CLR] dfs have=%d i=%d adj=(%d,%d,%d) touch=0\n", have, i, adj[i][0],
                                 adj[i][1], adj[i][2]);
                continue;
            }
            if (std::getenv("PMSF_CLR_DFSDBG") && have == 1)
                std::fprintf(stderr, "[CLR] dfs have=%d 取 i=%d\n", have, i);
            inS[i] = true;
            if (dfs(have + 1))
                return true;
            inS[i] = false;
        }
        return false;
    };
    // ---- 判别实验（无条件）：真值区域的重放是否成立 ----
    // 把"下标表对 type 9 仍错"与"dfs 选了同样能过判据的错候选"一次分开。
    if (trueRing && !trueRing->empty()) {
        std::vector<MCGAL::Vertex*> cyc;
        for (const auto& q2 : *trueRing) {
            MCGAL::Vertex* best = nullptr;
            double bd = 1e300;
            for (int i = 0; i < n; ++i) {
                MCGAL::Halfedge* it2 = tris[i]->proxyHalfedge();
                for (int k = 0; k < 3 && it2; ++k) {
                    MCGAL::Vertex* v = it2->vertex();
                    const double d = (v->x() - q2[0]) * (v->x() - q2[0]) + (v->y() - q2[1]) * (v->y() - q2[1]) +
                                     (v->z() - q2[2]) * (v->z() - q2[2]);
                    if (d < bd) { bd = d; best = v; }
                    it2 = it2->next();
                    if (!it2 || it2 == tris[i]->proxyHalfedge()) break;
                }
            }
            if (best) cyc.push_back(best);
        }
        int pos = -1;
        for (int k = 0; k < (int)cyc.size(); ++k) if (cyc[k] == gate->vertex()) pos = k;
        if (pos >= 0) std::rotate(cyc.begin(), cyc.begin() + pos, cyc.end());
        std::vector<MCGAL::Vertex*> uniq;
        for (MCGAL::Vertex* v : cyc)
            if (std::find(uniq.begin(), uniq.end(), v) == uniq.end()) uniq.push_back(v);
        auto expT = deterministicTriangulate(cyc, type);
        auto gotT = trisInside(mesh, uniq);
        std::fprintf(stderr, "[CLR] 判别 val=%d type=%d 环 %zu 去重 %zu 锚 %d | 重放 %zu 真值 %zu 相等=%s\n",
                     valence, type, cyc.size(), uniq.size(), pos, expT.size(), gotT.size(),
                     (expT == gotT) ? "YES" : "NO");
    }
    inS[0] = true;
    {
        int touch1 = 0, adjTotal = 0;
        for (int i = 1; i < n; ++i)
            for (int k = 0; k < 3; ++k) {
                if (adj[i][k] >= 0) ++adjTotal;
                if (adj[i][k] == 0) ++touch1;
            }
        std::fprintf(stderr, "[CLR] recover 深1：n=%d want=%d 邻接条目=%d 与tri0相邻=%d\n", n, want, adjTotal,
                     touch1);
    }
    if (want == 1)
        return false;   // V=3 不该走这里
    if (!dfs(1)) {
        std::fprintf(stderr, "[CLR] recover 无解 候选=%d 完成=%d 期望=%d\n", gCand, gCompletion, want);
        // ---- 判别实验：把真值一环直接喂给 deterministicTriangulate ----
        if (trueRing && !trueRing->empty()) {
            std::vector<MCGAL::Vertex*> cyc;
            for (const auto& q2 : *trueRing) {
                MCGAL::Vertex* best = nullptr;
                double bd = 1e300;
                for (int i = 0; i < n; ++i) {
                    MCGAL::Halfedge* it2 = tris[i]->proxyHalfedge();
                    for (int k = 0; k < 3 && it2; ++k) {
                        MCGAL::Vertex* v = it2->vertex();
                        const double d = (v->x() - q2[0]) * (v->x() - q2[0]) +
                                         (v->y() - q2[1]) * (v->y() - q2[1]) +
                                         (v->z() - q2[2]) * (v->z() - q2[2]);
                        if (d < bd) {
                            bd = d;
                            best = v;
                        }
                        it2 = it2->next();
                        if (!it2 || it2 == tris[i]->proxyHalfedge())
                            break;
                    }
                }
                if (best)
                    cyc.push_back(best);
            }
            int pos = -1;
            for (int k = 0; k < (int)cyc.size(); ++k)
                if (cyc[k] == gate->vertex())
                    pos = k;
            if (pos >= 0)
                std::rotate(cyc.begin(), cyc.begin() + pos, cyc.end());
            const std::vector<std::array<double, 3>>* rp = trueRing;
            (void)rp;
            std::vector<MCGAL::Vertex*> uniq;
            for (MCGAL::Vertex* v : cyc)
                if (std::find(uniq.begin(), uniq.end(), v) == uniq.end())
                    uniq.push_back(v);
            auto expT = deterministicTriangulate(cyc, type);
            auto gotT = trisInside(mesh, uniq);
            std::fprintf(stderr,
                         "[CLR] **判别实验** 真值环 %zu 个（去重 %zu，锚点 pos=%d）\n"
                         "       重放三角 %zu 个 / 真值区域三角 %zu 个 / 相等=%s\n",
                         cyc.size(), uniq.size(), pos, expT.size(), gotT.size(),
                         (expT == gotT) ? "**是**" : "**否**");
        }
        return false;
    }

    // ⑤ 选定后**才**动网格：并掉 S 的全部内边（恰 valence-3 条）
    std::fprintf(stderr, "[CLR] recover 成功 候选=%d 完成=%d\n", gCand, gCompletion);
    // ⚠️ 不能用 ③ 的 `idx`/`adj` —— join 之后面对象会合并，下标立刻陈旧。
    // 改用**面指针集合**，每轮从当前面现算。
    std::set<MCGAL::Facet*> sset;
    for (int i = 0; i < n; ++i)
        if (inS[i])
            sset.insert(tris[i]);
    // 原 gate 的源顶点：join 会移除半边、也会让 `gate` 漂移，
    // 但 sign 写入是**相对原 gate** 的（编码端在 erase 后的多边形上取 `g = nxt(h)`），
    // 所以必须靠这个顶点把锚点找回来。
    MCGAL::Vertex* origAnchor = gate->vertex();
    MCGAL::Halfedge* walk = gate;
    int guard = 0;
    while (trueFaceDeg(walk) >= 0 && trueFaceDeg(walk) < valence) {
        if (++guard > 64) {
            std::fprintf(stderr, "[CLR]   ⑤ 早退：guard 超限（度=%d 期望=%d）\n", trueFaceDeg(walk), valence);
            return false;
        }
        MCGAL::Halfedge* pick = nullptr;
        MCGAL::Halfedge* it = walk;
        for (int k = 0; k < 64 && it; ++k) {
            MCGAL::Halfedge* o = it->opposite();
            if (o && o->face() && sset.count(o->face())) {
                // **排掉退化构型**：两个面共享的顶点必须恰为 2（就是这条边）。
                // 共享 ≥3 时 join 出来是非简单面，MCGAL 的重连会让环不再闭合
                // —— 正是 gate 8 实测的"一刀断"（§20.3）。
                auto vset = [](MCGAL::Facet* f, std::vector<MCGAL::Vertex*>& out) {
                    MCGAL::Halfedge* jt = f->proxyHalfedge();
                    for (int z = 0; z < 64 && jt; ++z) {
                        out.push_back(jt->vertex());
                        jt = jt->next();
                        if (!jt || jt == f->proxyHalfedge())
                            break;
                    }
                };
                std::vector<MCGAL::Vertex*> fa, fb;
                vset(it->face(), fa);
                vset(o->face(), fb);
                int shared = 0;
                for (MCGAL::Vertex* v : fa)
                    for (MCGAL::Vertex* w : fb)
                        if (v == w)
                            ++shared;
                if (shared != 2) {
                    std::fprintf(stderr, "[CLR]   跳过退化边 h%lld：两面共享 %d 个顶点\n",
                                 (long long)it->poolId(), shared);
                    it = it->next();
                    if (!it || it == gate)
                        break;
                    continue;
                }
                pick = it;
                break;
            }
            it = it->next();
            if (!it || it == walk)
                break;
        }
        if (!pick) {
            std::fprintf(stderr, "[CLR]   ⑤ 早退：当前面(度=%d)上没有对面在 S 里的边；sset 余 %zu\n",
                         trueFaceDeg(walk), sset.size());
            return false;
        }
        std::fprintf(stderr, "[CLR]   准备并刀 guard=%d 度=%d pick=h%lld %s\n", guard, trueFaceDeg(walk),
                     (long long)pick->poolId(), (pick == walk) ? "**就是当前锚点**" : "");
        // ⚠️ **join 后必须把锚点重挂**：`pick` 可能就是 `gate` 本身，
        // join 会把它标成已移除 —— 之后 `trueFaceDeg(gate)` 沿已移除半边走
        // 就得到垃圾（实测 20000），看起来像"环被打断"，实为**锚点失效**。
        // 这正是整个调查里最早那个症状（"第一刀 join 掉了 gate 自身"）的同一个根因。
        MCGAL::Halfedge* reanchor = pick->next();
        sset.erase(pick->opposite()->face());
        mesh->join_face(pick);
        if (reanchor && !reanchor->isRemoved())
            walk = reanchor;
        std::fprintf(stderr, "[CLR]   并完 guard=%d 度=%d\n", guard, trueFaceDeg(walk));
    }
    std::fprintf(stderr, "[CLR]   网格阶段完成 度=%d 期望=%d\n", trueFaceDeg(walk), valence);
    if (trueFaceDeg(walk) != valence)
        return false;

    // ---- 复刻编码端 `retriangulate` 的 **facet 标记** --------------------
    // 编码端每支的 `setFFlag(h->face())` / `setFFlag(opp(h)->face())` 逐次累计，
    // 净效果恰好是**全部 V-2 个区域三角形**（type 5/8 标 2、9..12 标 3、13..16 标 4）。
    // 解码端对应的面就是 S 里的那些 —— 它们在 join 里被并掉，但面对象仍在，
    // `st.fflag` 照读不误，正是"跳过侧"要的输入。
    if (std::getenv("PMSF_CLR_MARK_S"))
        for (int i = 0; i < n; ++i)
            if (inS[i])
                st.setFFlag(tris[i], kFToBeRemoved);

    // ---- 复刻编码端 `retriangulate` 的 **sign 写入** ----------------------
    // 位置与编码端对齐：编码端在 erase 之后、split 之前写；
    // 解码端对应的时刻就是"多边形已恢复、中心点未插回"的此刻。
    // `g = nxt(h)`，h 是**原 gate**（靠 origAnchor 找回它在多边形上的位置）。
    {
        MCGAL::Halfedge* p = nullptr;
        MCGAL::Halfedge* jt = walk;
        for (int z = 0; z < 64 && jt; ++z) {
            if (jt->vertex() == origAnchor) {
                p = jt;
                break;
            }
            jt = jt->next();
            if (!jt || jt == walk)
                break;
        }
        if (!p)
            return false;
        if (polyAnchorOut)
            *polyAnchorOut = p;   // 多边形上一条**活着的**半边（源顶点 = 原 gate 的源顶点）
        MCGAL::Halfedge* g2 = p->next();
        auto SG = [&](MCGAL::Halfedge* hh, int sg) {
            if (hh && tgt(hh))
                st.setSign(tgt(hh), (std::uint8_t)sg);
        };
        if (type == 1 || type == 2 || type == 4)
            SG(g2, 1);
        else if (type == 3)
            SG(g2, 2);
        else if (type == 5 || type == 8) {
            SG(g2, 1);
            SG(g2 ? g2->next() : nullptr, 2);
        } else if (type == 6 || type == 7) {
            SG(g2, 2);
            SG(g2 ? g2->next() : nullptr, 1);
        } else if (type == 9 || type == 12 || type == 10) {
            SG(g2, 1);
            SG(g2 ? g2->next() : nullptr, 2);
            SG(g2 && g2->next() ? g2->next()->next() : nullptr, 1);
        } else if (type == 11) {
            SG(g2, 2);
            SG(g2 ? g2->next() : nullptr, 1);
            SG(g2 && g2->next() ? g2->next()->next() : nullptr, 2);
        } else if (type == 13 || type == 16) {
            SG(g2, 1);
            SG(g2 ? g2->next() : nullptr, 2);
            SG(g2 && g2->next() ? g2->next()->next() : nullptr, 1);
            SG(g2 && g2->next() && g2->next()->next() ? g2->next()->next()->next() : nullptr, 2);
        } else if (type == 14 || type == 15) {
            SG(g2, 2);
            SG(g2 ? g2->next() : nullptr, 1);
            SG(g2 && g2->next() ? g2->next()->next() : nullptr, 2);
            SG(g2 && g2->next() && g2->next()->next() ? g2->next()->next()->next() : nullptr, 1);
        }
    }
    return true;
}

/// **真值版**逆重三角化：内边判据换成"对面三角形的三个顶点都在目标一环里"。
/// 仅用于**验证实验** —— 目标一环由原始网格独立查出，不来自任何编码端状态。
bool inverseRetriangulateWithRing(MCGAL::Mesh* mesh, MCGAL::Halfedge* gate, int valence,
                                  const std::vector<std::array<double, 3>>& ringSet) {
    auto inRing = [&](MCGAL::Vertex* v) {
        if (!v)
            return false;
        for (const auto& q : ringSet)
            if (v->x() == q[0] && v->y() == q[1] && v->z() == q[2])
                return true;
        return false;
    };
    int guard = 0;
    while (trueFaceDeg(gate) >= 0 && trueFaceDeg(gate) < valence) {
        if (++guard > 64)
            return false;
        MCGAL::Halfedge* pick = nullptr;
        MCGAL::Halfedge* it = gate;
        for (int i = 0; i < 256; ++i) {
            MCGAL::Halfedge* o = it->opposite();
            if (o && o->face() && !o->face()->isRemoved()) {
                MCGAL::Facet* f = o->face();
                std::vector<MCGAL::Vertex*> vs;
                MCGAL::Halfedge* jt = f->proxyHalfedge();
                for (int k = 0; k < 64 && jt; ++k) {
                    vs.push_back(jt->vertex());
                    jt = jt->next();
                    if (!jt || jt == f->proxyHalfedge())
                        break;
                }
                bool all = !vs.empty();
                for (MCGAL::Vertex* w : vs)
                    if (!inRing(w))
                        all = false;
                if (all) {
                    pick = it;
                    break;
                }
            }
            it = it->next();
            if (!it || it == gate)
                break;
        }
        if (!pick)
            return false;
        mesh->join_face(pick);
    }
    return trueFaceDeg(gate) == valence;
}

/// 绕 v 的有界环，写出邻居坐标与 id（次序无关：预测器取算术平均）。
int ringOf(MCGAL::Vertex* v, double* pos, std::uint32_t* ids, int cap) {
    int n = 0;
    MCGAL::Halfedge* s0 = v ? v->proxyHalfedge() : nullptr;
    if (!s0)
        return 0;
    MCGAL::Halfedge* it = s0;
    for (int i = 0; i < cap; ++i) {
        MCGAL::Vertex* w = it->end_vertex();
        pos[3 * n] = w ? w->x() : 0.0;
        pos[3 * n + 1] = w ? w->y() : 0.0;
        pos[3 * n + 2] = w ? w->z() : 0.0;
        ids[n] = w ? w->poolId() : 0u;
        ++n;
        MCGAL::Halfedge* o = it->opposite();
        if (!o)
            break;
        it = o->next();
        if (!it || it == s0)
            break;
    }
    return n;
}

/// 记录型几何汇：按**删除顺序**收下残差。真 codec 里这些进码流。
class RecordingSink : public pmsf::GeometrySink {
  public:
    std::vector<std::array<int, 3>> deltas;
    void onRemovedVertex(const int cellDelta[3]) override {
        deltas.push_back({cellDelta[0], cellDelta[1], cellDelta[2]});
    }
};

int livingV(MCGAL::Mesh* m) {
    int n = 0;
    for (MCGAL::Vertex* v : m->vertices())
        if (v && !v->isRemoved())
            ++n;
    return n;
}
int livingF(MCGAL::Mesh* m) {
    int n = 0;
    for (MCGAL::Facet* f : m->faces())
        if (f && !f->isRemoved())
            ++n;
    return n;
}

/// 按活面 dump 成 OFF（顶点索引重排）。解码端靠它从快照重启。
bool dumpOff(MCGAL::Mesh* mesh, const char* path) {
    std::map<MCGAL::Vertex*, int> idx;
    std::vector<MCGAL::Vertex*> vs;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved())
            continue;
        idx[v] = (int)vs.size();
        vs.push_back(v);
    }
    int nf = 0;
    for (MCGAL::Facet* f : mesh->faces())
        if (f && !f->isRemoved())
            ++nf;
    std::ofstream os(path);
    if (!os)
        return false;
    // ⚠️ **必须全精度**：默认 6 位有效数字的舍入，在包围盒边界附近足以
    // 让 `toCell` 的 `ceil` 翻格（实测：内存里 Δ格=0 的两侧，
    // 经 6 位 OFF 往返后变成差 1 格，凭空造出 5 项对不上）。
    os << std::setprecision(17);
    os << "OFF\n" << vs.size() << " " << nf << " 0\n";
    for (MCGAL::Vertex* v : vs) {
        const MCGAL::Point& p = v->point();
        os << p.x() << " " << p.y() << " " << p.z() << "\n";
    }
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved())
            continue;
        MCGAL::Halfedge* it = f->proxyHalfedge();
        std::vector<int> ids;
        for (int i = 0; i < 512 && it; ++i) {
            auto k = idx.find(it->vertex());
            if (k == idx.end())
                break;
            ids.push_back(k->second);
            it = it->next();
            if (!it || it == f->proxyHalfedge())
                break;
        }
        if (ids.size() < 3)
            continue;
        os << ids.size();
        for (int i : ids)
            os << " " << i;
        os << "\n";
    }
    return true;
}

// ---- 解码端遍历本体 --------------------------------------------------------
// decimateLayer 的镜像。**遍历结构完全相同**（同种子、同队列纪律、同前沿扩张），
// 差别只在动作：编码端按判据删，解码端按符号插。
//
// 关键：符号 4 把「H2 判据说不可删」「manifold 违规」「兜底」三种情况**合并**成
// 同一处理 —— 解码端**不需要**区分它们，这正是这套符号字母表成立的原因。
int decodeLayer(MCGAL::Mesh* mesh,
                ConquestState& st,
                MCGAL::Halfedge* seed,
                const std::vector<int>& syms,
                std::vector<std::string>* trace,
                const pmsf::FrontierConquestEngine::QuantGrid& grid,
                const std::vector<std::array<int, 3>>& deltas,
                std::vector<std::array<double, 3>>* frontSeqOut,
                std::vector<int>* symSeqOut = nullptr,
                std::vector<size_t>* pushSeqOut = nullptr,
                std::vector<std::string>* actSeqOut = nullptr,
                std::vector<std::array<double, 3>>* insPosOut = nullptr,
                const std::vector<std::vector<std::array<double, 3>>>* rings = nullptr) {
    // ---- **默认走 `src/conquest/` 版**（P13.4 (B)①，2026-09-22 搬迁）--------
    // 解码实体已落在 `src/conquest/ConquestDecodeLayer.cpp`，探针**不再另跑一份**：
    // 探针的职责是**验证 src 版**，自己存一份副本的话"验证"就名不副实
    // （本库反复栽在"复制粘贴后悄悄分家"上）。
    //
    // 只有设 `PMSF_CLR_LEGACYDEC=1` 才走下面那份**逐 gate 诊断原体**：它多产出
    // `frontSeq / symSeq / pushSeq / actSeq / insPos` 五个只服务对拍的序列，
    // 下游 ③.5 的 "gate 对齐" 对拍要用。**只在需要定位分岔时开。**
    //
    // 搬迁验收（2026-09-22）：两模式在 **4 网格 × 8 层链式**上 `RESULT` 逐字相同、
    // `clr_dec.off` **逐字节相同**。见蓝图 §79。
    if (!std::getenv("PMSF_CLR_LEGACYDEC")) {
        const pmsf::ConquestDecodeResult r = pmsf::decodeLayer(mesh, st, seed, syms, deltas, grid);
        std::fprintf(stderr, "[CLR] **src 版**：插回 %d 点，null-patch %d 次，符号 %zu/%zu，环不符 %d\n", r.inserted,
                     r.nullPatch, r.consumed, syms.size(), r.ringMismatch);
        return r.ok ? 0 : 1;
    }
    std::deque<MCGAL::Halfedge*> q;
    st.reset();
    st.setFlag(tgt(seed), 1);
    st.setSign(tgt(seed), 1);
    st.setFlag(tgt(opp(seed)), 1);
    st.setSign(tgt(opp(seed)), 2);
    q.push_back(seed);

    size_t si = 0;
    int inserted = 0, nullPatch = 0;
    std::vector<std::array<double, 3>> frontSeq;  // 本端每个 gate 的前端点位置
    std::vector<int> symSeq;                      // 本端每个 gate 读到的符号
    std::vector<size_t> pushSeq;                  // 本端每个 gate 的 push 条数
    std::vector<std::array<double, 3>> insPos;    // 本端每次插入的结果位置
    std::vector<std::string> actSeq;              // 本端每个 gate 的动作（DELETE/NULL）
    std::unordered_map<MCGAL::Halfedge*, const char*> prov;  // gate 来源：NUL / INS
    auto log = [&](const std::string& s) {
        if (trace)
            trace->push_back(s);
    };

    while (!q.empty()) {
        MCGAL::Halfedge* h = q.front();
        q.pop_front();
        if (!h)
            continue;
        // ⚠️ **不检查 `h->isRemoved()`**：编码端也不检查 —— 它只查
        // `st.fflag(h->face())`，对已移除半边读到的是陈旧但**确定**的值。
        // 在这里加一条比编码端更严的护栏，会让两侧不再同构，
        // 反而把"解码端与编码端逐字一样吗"这个问题搅浑（本轮踩过）。
        // ⚠️ flag 跳过是**必需**的：实测关掉它，第 4 个 gate 就
        // `inverseRetriangulate 失败`（区域不再是那批三角形）。
        const std::uint8_t ff = st.fflag(h->face());
        if (ff == kFConquered || ff == kFToBeRemoved) {
            if (std::getenv("PMSF_CLR_SKIPDBG") && si <= 12) {
                MCGAL::Vertex* sv = tgt(nxt(h));
                std::fprintf(stderr, "[SKIP] 跳过 h%lld fflag=%d 前端=(%.4f,%.4f,%.4f)\n",
                             (long long)h->poolId(), (int)ff, sv ? sv->x() : -9, sv ? sv->y() : -9,
                             sv ? sv->z() : -9);
            }
            continue;
        }

        if (si >= syms.size()) {
            std::fprintf(stderr, "[CLR] **符号流耗尽** 已用=%zu 插回=%d null=%d 队列长=%zu\n", si, inserted, nullPatch, q.size());
            int nIns = 0, nNul = 0, nFlagged = 0;
            for (MCGAL::Halfedge* e : q) {
                const char* pv = prov.count(e) ? prov[e] : "?";
                if (pv[0] == 'I')
                    ++nIns;
                else if (pv[0] == 'N')
                    ++nNul;
                const std::uint8_t f2 = e ? st.fflag(e->face()) : 0;
                if (f2 == kFConquered || f2 == kFToBeRemoved)
                    ++nFlagged;
            }
            std::fprintf(stderr, "[CLR]   残余来源：INS=%d NUL=%d 其他=%d；其中已被 flag 跳过=%d\n", nIns, nNul,
                         (int)q.size() - nIns - nNul, nFlagged);
            // 符号耗尽 = 本层结束。**不再当作错误** ——
            // 队列长度只是代理指标，完成定义要的是**网格本身对不对**。
            // 残余 gate 在真解码里不会出现（层边界由符号数界定）。
            std::fprintf(stderr, "[CLR] 符号耗尽 ⇒ 本层结束，残余 gate %zu 条未处理\n", q.size() + 1);
            break;
        }
        const int s = syms[si++];
        MCGAL::Halfedge* nh = nxt(h);
        MCGAL::Vertex* vFront = tgt(nh);
        (void)vFront;
        const size_t qBeforeGate = q.size();
        // 诊断：本 gate 处理前，队首（=下一个 gate）的前端点位置。
        // 与编码端 gate 轨迹同序号一一对应，可直接看出哪一步的 push 分了岔。
        if (std::getenv("PMSF_CLR_QDBG") && si <= 12) {
            MCGAL::Halfedge* qh = q.empty() ? nullptr : q.front();
            MCGAL::Vertex* qv = qh ? tgt(nxt(qh)) : nullptr;
            std::fprintf(stderr, "[Q] gate#%zu 本gate前端=(%.4f,%.4f,%.4f) 队首前端=%s\n", si - 1,
                         vFront ? vFront->x() : -9, vFront ? vFront->y() : -9, vFront ? vFront->z() : -9,
                         qv ? std::to_string(qv->x()).c_str() : "空");
        }

        if (s >= 0 && s <= 3) {
            const int valence = s + 3;
            (void)qBeforeGate;
            // 删点分支：前端点在解码端尚不存在，稍后由插回产生，位置在后面补记
            frontSeq.push_back({0, 0, 0});
            symSeq.push_back(s);
            // 编码端此处做的是「erase_center_vertex 后再 retriangulate」。
            // 解码端必须**严格逆序**：先把区域并回多边形，再跑标志/入队循环，
            // 最后才插回中心点 —— 因为编码端那个循环是在**多边形**上跑的
            // （erase 之后、retriangulate 之前）。
            std::vector<MCGAL::Facet*> regionFaces;
            bool invOk;
            if (std::getenv("PMSF_CLR_REMOVEDGES")) {
                // **§42.4**：Remove_Edges 的忠实直译（跟着 join 返回的半边走，
                // 不用锚点、不用下标表），再走 addCenterVertex 扇回来。
                const int gt = findType(st, h, valence);
                if (!pmsf::removeEdges(mesh, h, gt)) {
                    std::fprintf(stderr, "[CLR] removeEdges 失败 第%zu个gate valence=%d type=%d\n", si, valence, gt);
                    break;
                }
                // sign 写入：与并边同函数（参考的做法），**直接用原 gate h**。
                pmsf::writeRemoveEdgesSigns(h, gt, [&](MCGAL::Vertex* v, int sg) {
                    if (v)
                        st.setSign(v, (std::uint8_t)sg);
                });
            }
            // ⚠️ `h` 可能在 join 里被移除（与 gate 8 的锚点失效同一个坑）。
            // 先记下它的源顶点，join 之后在多边形上找回一条**活着的**半边。
            MCGAL::Vertex* gateAnchor = h ? h->vertex() : nullptr;
            MCGAL::Halfedge* polyAnchor = nullptr;
            if (std::getenv("PMSF_CLR_RECOVER")) {
                // **§13 导航重放实验**（env 门控；默认不走这条）。
                // ⚠️ 当前**未收敛**：回溯失败要撤销 join，而撤销用 split_facet
                // 会撞上 MCGAL 池分配的自动配对断言（搜索结果网格未精确还原）。
                // 下一步应改成**先在三角形邻接的组合模型上回溯**，选定后再对网格做 join。
                const int gtype = findType(st, h, valence);
                if (!recoverRegion(mesh, st, h, valence, gtype, [&]() -> const std::vector<std::array<double, 3>>* {
                        return (rings && inserted < (int)rings->size()) ? &(*rings)[inserted] : nullptr;
                    }(), &polyAnchor)) {
                    std::fprintf(stderr, "[CLR] recoverRegion 失败 第%zu个gate valence=%d type=%d\n", si, valence,
                                 gtype);
                    log("**recoverRegion 失败** sym=" + std::to_string(s));
                    break;
                }
            }
            // **§74.3 证伪实验**：PMSF_CLR_NORING=1 时跳过真值一环那一支，
            // 用来判定"它的副作用是否改变最终网格"。
            // **§76.2 无条件哨兵**：确认本块是否被进入（上一次条件块没打印，原因未定）。
            if (std::getenv("PMSF_CLR_RINGCMP"))
                std::fprintf(stderr, "[RING] 哨兵 进入删点分支后段 si=%zu inserted=%d rings=%p n=%zu\n", si,
                             inserted, (const void*)rings,
                             rings ? rings->size() : (size_t)0);
            // **§75.4 诊断**：并边之后、插点之前的**区域顶点集**，与真值一环并排。
            if (std::getenv("PMSF_CLR_RINGCMP") && rings && inserted < (int)rings->size() &&
                !(*rings)[inserted].empty()) {
                std::vector<std::array<double, 3>> region;
                MCGAL::Halfedge* it = h;
                for (int z = 0; z < 64 && it; ++z) {
                    MCGAL::Vertex* w = it->vertex();
                    if (w)
                        region.push_back({w->x(), w->y(), w->z()});
                    it = it->next();
                    if (!it || it == h)
                        break;
                }
                const auto& ring = (*rings)[inserted];
                int hit = 0;
                for (const auto& p2 : region)
                    for (const auto& q2 : ring)
                        if (std::fabs(p2[0] - q2[0]) < 1e-9 && std::fabs(p2[1] - q2[1]) < 1e-9 &&
                            std::fabs(p2[2] - q2[2]) < 1e-9) {
                            ++hit;
                            break;
                        }
                std::fprintf(stderr, "[RING] gate#%zu valence=%d 区域%zu 个点 / 真值一环 %zu 个点 / 命中 %d\n",
                             si - 1, valence, region.size(), ring.size(), hit);
                if (hit != (int)ring.size())
                    for (const auto& q2 : ring) {
                        int f = 0;
                        for (const auto& p2 : region)
                            if (std::fabs(p2[0] - q2[0]) < 1e-9 && std::fabs(p2[1] - q2[1]) < 1e-9 &&
                                std::fabs(p2[2] - q2[2]) < 1e-9)
                                f = 1;
                        if (!f)
                            std::fprintf(stderr, "[RING]   真值点 (%.4f,%.4f,%.4f) **不在区域里**\n", q2[0], q2[1],
                                         q2[2]);
                    }
            }
            if (std::getenv("PMSF_CLR_NORING")) {
                // **§74.3 证伪实验（修正版）**：`removeEdges` 已把区域变成多边形，
                // 这里**什么都不做** —— 用来判定"真值一环那一支是否多余"。
                // （第一版错在让它落进 else 的旧 poolId 路径，那是另一回事。）
                invOk = true;
            } else if (rings && inserted < (int)rings->size() && !(*rings)[inserted].empty()) {
                // **验证实验**：内边判据换成真值一环（由原始网格独立查出）。
                invOk = inverseRetriangulateWithRing(mesh, h, valence, (*rings)[inserted]);
            } else {
                const int degBefore = trueFaceDeg(h);
                invOk = pmsf::inverseRetriangulate(mesh, h, valence, &regionFaces);
                const int degAfter = trueFaceDeg(h);
                if (std::getenv("PMSF_CLR_ELSE"))
                    std::fprintf(stderr, "[ELSE] gate#%zu valence=%d 调用前真实度=%d 调用后=%d 吸收面=%zu %s\n",
                                 si - 1, valence, degBefore, degAfter, regionFaces.size(),
                                 (degBefore == degAfter && regionFaces.empty()) ? "**空操作**" : "**有副作用**");
            }
            if (false) {
                std::fprintf(stderr, "[CLR] **逆重三角化失败** 第%zu个gate sym=%d valence=%d 真值一环=%s\n", si, s,
                             valence,
                             (rings && inserted < (int)rings->size() && !(*rings)[inserted].empty()) ? "有" : "**无**");
                std::fprintf(stderr, "[CLR]   face(h)%s removed=%d 真实度=%d  proxy=%s\n", h->face() ? "" : "(空)",
                             (h->face() && h->face()->isRemoved()) ? 1 : 0, trueFaceDeg(h),
                             (h->face() && h->face()->proxyHalfedge() == h) ? "是h" : "非h");
                std::fprintf(stderr, "[CLR]   face(h) 各边: ");
                MCGAL::Halfedge* it = h;
                for (int i = 0; i < 64 && it; ++i) {
                    MCGAL::Halfedge* o = it->opposite();
                    std::fprintf(stderr, "[h%lld 对面度=%d%s] ", (long long)it->poolId(), o ? trueFaceDeg(o) : -9,
                                 (o && o->face() && !o->face()->isRemoved() && trueFaceDeg(o) == 3) ? "*" : "");
                    it = it->next();
                    if (!it || it == h)
                        break;
                }
                std::fprintf(stderr, "\n[CLR]   tgt(h)=%s tgt(next(h))=%s\n", tgt(h) ? std::to_string(tgt(h)->poolId()).c_str() : "空",
                             tgt(nxt(h)) ? std::to_string(tgt(nxt(h))->poolId()).c_str() : "空");
                log("**inverseRetriangulate 失败** sym=" + std::to_string(s));
                break;   // 不中断：走完对拍，拿到可比的拓扑数
            }
            // join 之后把 `h` 重挂到多边形上活着的半边。
            // ⚠️ 必须用 `recoverRegion` **内部**已找好的那条（它从活半边起步搜索）；
            // 从 `h` 自己开始搜是错的 —— `h` 已被 join 移除，走出来还是垃圾
            // （§27 的 ③ 就栽在这里，反而更差）。
            // 整体替换已撤（§30.3）：只让 push 用 polyAnchor，其余仍用原 h。
            if (false)
            // ⚠️ **默认关闭**：§27.2 的 A/B 显示这一步在这里**不必要且有害**
            // （插回 10→7 点、分岔下标不动）。保留为可复现的实验开关。
            if (gateAnchor && std::getenv("PMSF_CLR_REANCHOR")) {
                MCGAL::Halfedge* p = nullptr;
                MCGAL::Halfedge* jt = h;
                for (int z = 0; z < 64 && jt; ++z) {
                    if (!jt->isRemoved() && jt->vertex() == gateAnchor) {
                        p = jt;
                        break;
                    }
                    jt = jt->next();
                    if (!jt || jt == h)
                        break;
                }
                if (p)
                    h = p;
            }
            MCGAL::Halfedge* g = h;
            // **差分实验**：push 用 `polyAnchor`（活半边），标记/标志仍用原 `h`。
            MCGAL::Halfedge* gw = (polyAnchor && !std::getenv("PMSF_CLR_PUSH_USE_H")) ? polyAnchor : h;
            // 编码端此处先标 kFTemp，**随后 `retriangulate` 把它覆盖成
            // kFToBeRemoved**（每个 type 分支都写 h->face()）。解码端不跑
            // retriangulate（三角形已在），必须直接落到**净效果** kFToBeRemoved，
            // 否则这个面不会被后续 gate 跳过 ⇒ 队列比编码端长（实测 107 条）。
            if (g->face())
                st.setFFlag(g->face(), kFToBeRemoved);
            const size_t qBefore = q.size();
            (void)g;
            for (int j = 0; j < valence - 1; ++j) {
                gw = nxt(gw);
                if (!gw)
                    break;
                if (tgt(gw))
                    st.setFlag(tgt(gw), 1);
                if (tgt(opp(gw)))
                    st.setFlag(tgt(opp(gw)), 1);
                const bool be = isBorderEdge(gw);
                MCGAL::Halfedge* g = gw;
                if (std::getenv("PMSF_CLR_PUSHDBG") && si >= 6 && si <= 9) {
                    MCGAL::Halfedge* o = opp(g);
                    MCGAL::Facet* fo = o ? o->face() : nullptr;
                    std::fprintf(stderr,
                                 "[PUSH] gate#%zu j=%d g=h%lld g.rem=%d g.face=%p(f.rem=%d) "
                                 "o=h%lld o.rem=%d o.face=%p(f.rem=%d) isBorder=%d\n",
                                 si - 1, j, (long long)g->poolId(), (int)g->isRemoved(),
                                 (void*)(g->face()), (g->face() && g->face()->isRemoved()) ? 1 : 0,
                                 (long long)(o ? o->poolId() : -1), o ? (int)o->isRemoved() : -1, (void*)fo,
                                 (fo && fo->isRemoved()) ? 1 : 0, (int)be);
                }
                if (!be)
                    q.push_back(opp(g));
            }
            for (size_t k = qBefore; k < q.size(); ++k)
                prov[q[k]] = "INS";
            if (std::getenv("PMSF_CLR_PUSHSEQ") && si <= 9) {
                std::fprintf(stderr, "[SEQ] gate#%zu DELETE push %zu 条:", si - 1, q.size() - qBefore);
                for (size_t k = qBefore; k < q.size(); ++k) {
                    MCGAL::Vertex* w = tgt(nxt(q[k]));
                    std::fprintf(stderr, " (%.4f,%.4f,%.4f)", w ? w->x() : -9, w ? w->y() : -9, w ? w->z() : -9);
                }
                std::fprintf(stderr, "\n");
            }
            // 编码端 retriangulate 标记的 V-2 个面 = 它的**输出** = 解码端的**输入**
            // （就是刚被并回区域的那批面）。逐支标记数完全吻合，见 §9.5。
            // **差分实验**：早期遗留的"标记被吸收面"如今是否仍是分岔来源？
            // 用 env 关掉它，看"首次分岔下标"是否移动（差分判据，不靠猜）。
            if (!std::getenv("PMSF_CLR_NOMARK"))
                for (MCGAL::Facet* f : regionFaces)
                    st.setFFlag(f, kFToBeRemoved);
            std::vector<MCGAL::Facet*> newFaces;
            MCGAL::Vertex* nv = pmsf::addCenterVertex(mesh, h, &newFaces);
            // 对称假设：后续 gate 看到的是本端的**输出**面（addCenterVertex 新造的 V 个），
            // 编码端看到的是它自己的输出（V-2 个），两者都被标记才可能跳过一致。
            if (!std::getenv("PMSF_CLR_MARK_NONE"))
                for (MCGAL::Facet* f : newFaces)
                    st.setFFlag(f, kFToBeRemoved);
            if (!nv) {
                log("**addCenterVertex 失败** sym=" + std::to_string(s));
                break;
            }
            // ---- 几何重建：CRV = BC + Dist（整数域恒等，P13.2 已单独验过）----
            if (grid.valid() && inserted < (int)deltas.size()) {
                double nb[3 * 256];
                std::uint32_t ids[256];
                const int n = ringOf(nv, nb, ids, 256);
                if (n != valence)
                    std::fprintf(stderr, "[CLR] **环大小不符** 第%d次插入 valence=%d 但环=%d\n", inserted, valence,
                                 n);
                pmsf::PatchView pv;
                pv.neighborPos = nb;
                pv.neighborCount = n;
                pv.quantStep = grid.step;
                pv.bboxMin = grid.bboxMin;
                pv.neighborIds = ids;
                pmsf::FrontierConquestEngine tmp(nullptr);
                double pred[3];
                tmp.hooks().geometryOrDefault().predict(pv, pred);
                int d[3];
                tmp.hooks().quantizerOrDefault().inverseTransform(deltas[inserted].data(), d, pv);
                double p3[3];
                for (int k = 0; k < 3; ++k) {
                    const int cell = tmp.hooks().quantizerOrDefault().toCell(pred[k], grid.bboxMin[k], grid.step) + d[k];
                    p3[k] = tmp.hooks().quantizerOrDefault().toWorld(cell, grid.bboxMin[k], grid.step);
                }
                MCGAL::Point np3(p3[0], p3[1], p3[2]);
                // ⚠️ 必须是 `setPoint`。MCGAL 的 Vertex 把坐标存在 `v[0..2]`，
                // `point()`/`setPoint()`/`x()` 读写的是它；而 `setVertexPoint()`
                // 写的是**另一个字段** `vertexPoint_`，`x()` 根本不读 ——
                // 用了它几何就**静默地一点没写进去**（实测：前端点位置序列在
                // 第一次插入处跳到 0，而我自己打印的中间量 p3 却有值）。
                nv->setPoint(np3);
                            }
            st.setFlag(nv, 1);  // 编码端此点已不存在 ⇒ 对解码端等价于"已征服"
            frontSeq.back() = {nv->x(), nv->y(), nv->z()};
            insPos.push_back({nv->x(), nv->y(), nv->z()});
            pushSeq.push_back(q.size() - qBeforeGate);
            actSeq.push_back("DELETE");
            ++inserted;
            log("INS sym=" + std::to_string(s));
        } else {
            if (h->face())
                st.setFFlag(h->face(), kFConquered);
            st.setFlag(vFront, 1);
            {   // null 分支：前端点两侧都还在，直接读
                MCGAL::Vertex* fv = tgt(nxt(h));
                frontSeq.push_back(fv ? std::array<double, 3>{fv->x(), fv->y(), fv->z()}
                                      : std::array<double, 3>{0, 0, 0});
                symSeq.push_back(s);
            }
            const size_t qBefore2 = q.size();
            nullPatchTail(st, h, q);
            pushSeq.push_back(q.size() - qBefore2);
            actSeq.push_back("NULL");
            for (size_t k = qBefore2; k < q.size(); ++k)
                prov[q[k]] = "NUL";
            if (std::getenv("PMSF_CLR_PUSHSEQ") && si <= 9) {
                std::fprintf(stderr, "[SEQ] gate#%zu NULL   push %zu 条:", si - 1, q.size() - qBefore2);
                for (size_t k = qBefore2; k < q.size(); ++k) {
                    MCGAL::Vertex* w = tgt(nxt(q[k]));
                    std::fprintf(stderr, " (%.4f,%.4f,%.4f)", w ? w->x() : -9, w ? w->y() : -9, w ? w->z() : -9);
                }
                std::fprintf(stderr, "\n");
            }
            ++nullPatch;
            log("NUL");
        }
    }
    if (si != syms.size())
        log("**符号未消费完** 用=" + std::to_string(si) + "/" + std::to_string(syms.size()));
    std::fprintf(stderr, "[CLR] 解码端：插回 %d 点，null-patch %d 次，符号 %zu 个\n", inserted, nullPatch, si);
    if (frontSeqOut)
        *frontSeqOut = frontSeq;
    if (symSeqOut)
        *symSeqOut = symSeq;
    if (insPosOut)
        *insPosOut = insPos;
    if (pushSeqOut)
        *pushSeqOut = pushSeq;
    if (actSeqOut)
        *actSeqOut = actSeq;
    return 0;
}

// ---- 按坐标找回同一条半边 --------------------------------------------------
MCGAL::Vertex* nearestVertex(MCGAL::Mesh* mesh, const double p[3]) {
    MCGAL::Vertex* best = nullptr;
    double bd = 1e300;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved())
            continue;
        const MCGAL::Point& q = v->point();
        const double d = (q.x() - p[0]) * (q.x() - p[0]) + (q.y() - p[1]) * (q.y() - p[1]) + (q.z() - p[2]) * (q.z() - p[2]);
        if (d < bd) {
            bd = d;
            best = v;
        }
    }
    return best;
}

MCGAL::Halfedge* halfedgeBetween(MCGAL::Vertex* a, MCGAL::Vertex* b) {
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

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "用法: %s <mesh.off> <workdir>\n", argv[0]);
        return 1;
    }
    const std::string meshPath = argv[1], wd = argv[2];
    const std::string origOff = wd + "/clr_orig.off", postOff = wd + "/clr_post.off", decOff = wd + "/clr_dec.off", symFile = wd + "/clr_syms.txt";
    pmsf::ensureMeshRuntimeInitialized();

    double seedPos[2][3] = {{0, 0, 0}, {0, 0, 0}};
    std::vector<std::array<double, 3>> decSeq;   // 解码端逐 gate 的前端点位置
    std::vector<int> decSym;
    std::vector<size_t> decPush;
    std::vector<std::array<double, 3>> decIns;
    std::vector<std::string> decAct;
    std::vector<std::string> encKind;   // 编码端每 gate 的动作（DELETE/NULL）
    std::vector<size_t> encPush;        // 编码端每 gate 的 push 条数                     // 解码端逐 gate 读到的符号（由 decodeLayer 输出）

    // ---- ①② 编码端：dump 原网格 + 跑一层删减 ---------------------------------
    {
        auto mesh = std::make_shared<MCGAL::Mesh>(MCGAL::Mesh(meshPath));
        if (!dumpOff(mesh.get(), origOff.c_str())) {
            std::fprintf(stderr, "[CLR] dump 原网格失败\n");
            return 1;
        }
        std::fprintf(stderr, "[CLR] 原网格 V=%d F=%d\n", livingV(mesh.get()), livingF(mesh.get()));

        // 种子：取一条内部边（两侧都有面），两端点记坐标供解码端找回
        MCGAL::Halfedge* seed = nullptr;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved())
                continue;
            MCGAL::Halfedge* it = f->proxyHalfedge();
            for (int i = 0; i < 512 && it; ++i) {
                if (!isBorderEdge(it) && tgt(it) && tgt(opp(it))) {
                    seed = it;
                    break;
                }
                it = it->next();
                if (!it || it == f->proxyHalfedge())
                    break;
            }
            if (seed)
                break;
        }
        if (!seed) {
            std::fprintf(stderr, "[CLR] 找不到内部种子边\n");
            return 1;
        }
        for (int k = 0; k < 2; ++k) {
            MCGAL::Vertex* v = (k == 0) ? tgt(seed) : tgt(opp(seed));
            const MCGAL::Point& p = v->point();
            seedPos[k][0] = p.x();
            seedPos[k][1] = p.y();
            seedPos[k][2] = p.z();
        }
        std::fprintf(stderr, "[CLR] 种子 v%u -> v%u\n", tgt(seed)->poolId(), tgt(opp(seed))->poolId());

        pmsf::FrontierConquestEngine engine(nullptr);
        // 量化网格：包围盒 + 步长（全层一致，随码流传）
        pmsf::FrontierConquestEngine::QuantGrid grid;
        {
            double lo[3] = {1e300, 1e300, 1e300}, hi[3] = {-1e300, -1e300, -1e300};
            for (MCGAL::Vertex* v : mesh->vertices()) {
                if (!v || v->isRemoved())
                    continue;
                const double c[3] = {v->x(), v->y(), v->z()};
                for (int k = 0; k < 3; ++k) {
                    lo[k] = std::min(lo[k], c[k]);
                    hi[k] = std::max(hi[k], c[k]);
                }
            }
            double ext = 0;
            for (int k = 0; k < 3; ++k)
                ext = std::max(ext, hi[k] - lo[k]);
            for (int k = 0; k < 3; ++k)
                grid.bboxMin[k] = lo[k];
            grid.step = ext / 4096.0;
        }
        engine.setQuantGrid(grid);
        std::FILE* gtf = std::fopen((wd + "/clr_gatetrace_enc.txt").c_str(), "w");
        engine.setGateTrace(gtf);
        RecordingSink sink;
        ConquestState st;
        std::vector<int> syms;
        auto res = engine.decimateLayer(mesh.get(), st, seed, &syms, &sink);
        std::fprintf(stderr, "[CLR] 编码端：删 %d 点，发 %llu 符号（实际 %zu）\n", res.removedVertices, (unsigned long long)res.symbolsEmitted,
                     syms.size());

        if (!dumpOff(mesh.get(), postOff.c_str())) {
            std::fprintf(stderr, "[CLR] dump 删减后网格失败\n");
            return 1;
        }
        if (gtf)
            std::fclose(gtf);
        std::ofstream os(symFile);
        for (int s : syms)
            os << s << "\n";
        {
            std::ofstream gs(wd + "/clr_geom.txt");
            // ⚠️ **必须全精度**：`bboxMin`/`step` 是量化网格的定义，
            // 6 位有效数字的舍入会在边界处让 `toCell` 的 `ceil` 翻格
            // ——比较时用的是读回来的值，于是凭空造出"差 1 格"的假差额。
            gs << std::setprecision(17);
            gs << grid.step << " " << grid.bboxMin[0] << " " << grid.bboxMin[1] << " " << grid.bboxMin[2] << "\n";
            for (const auto& d : sink.deltas)
                gs << d[0] << " " << d[1] << " " << d[2] << "\n";
            std::fprintf(stderr, "[CLR] 几何残差 %zu 条（删除 %d 次）\n", sink.deltas.size(), res.removedVertices);
        }
        std::fprintf(stderr, "[CLR] 删减后网格 V=%d F=%d\n", livingV(mesh.get()), livingF(mesh.get()));
    }

    // ---- ②.5 从原始网格独立查出每次删除的真值一环（仅供验证实验）----------
    std::vector<std::vector<std::array<double, 3>>> rings;
    {
        auto ref = std::make_shared<MCGAL::Mesh>(MCGAL::Mesh(origOff));
        std::vector<std::array<double, 3>> trace;
        {
            std::ifstream is(wd + "/clr_gatetrace_enc.txt");
            double x, y, z;
            while (is >> x >> y >> z)
                trace.push_back({x, y, z});
        }
        std::vector<int> symsRef;
        {
            std::ifstream is(symFile);
            int s2;
            while (is >> s2)
                symsRef.push_back(s2);
        }
        for (size_t i = 0; i < symsRef.size() && i < trace.size(); ++i) {
            if (symsRef[i] > 3)
                continue;
            MCGAL::Vertex* v = nearestVertex(ref.get(), trace[i].data());
            std::vector<std::array<double, 3>> r;
            if (v) {
                MCGAL::Halfedge* s0 = v->proxyHalfedge();
                MCGAL::Halfedge* it = s0;
                for (int k = 0; k < 256 && it; ++k) {
                    MCGAL::Vertex* w = it->end_vertex();
                    if (w)
                        r.push_back({w->x(), w->y(), w->z()});
                    MCGAL::Halfedge* o = it->opposite();
                    if (!o)
                        break;
                    it = o->next();
                    if (!it || it == s0)
                        break;
                }
            }
            rings.push_back(r);
        }
        std::fprintf(stderr, "[CLR] 真值一环 %zu 组（删除 %d 次）\n", rings.size(), (int)rings.size());
    }

    // ---- ③ 解码端：从落盘快照重启 -------------------------------------------
    {
        auto mesh = std::make_shared<MCGAL::Mesh>(MCGAL::Mesh(postOff));
        // **载入后立即体检**：面环是否闭合。若此处就破损，问题在 dump/load，
        // 不在解码端 —— 前几步 NUL 不改拓扑，足以把责任分清。
        {
            int broken = 0, faces = 0;
            for (MCGAL::Facet* f : mesh->faces()) {
                if (!f || f->isRemoved())
                    continue;
                ++faces;
                MCGAL::Halfedge* s0 = f->proxyHalfedge();
                MCGAL::Halfedge* it = s0;
                bool closed = false;
                for (int i = 0; i < 512 && it; ++i) {
                    it = it->next();
                    if (!it || it == s0) {
                        closed = (it == s0);
                        break;
                    }
                }
                if (!closed)
                    ++broken;
            }
            std::fprintf(stderr, "[CLR] 载入后体检：活面 %d，环不闭合 %d\n", faces, broken);
        }
        std::vector<int> syms;
        {
            std::ifstream is(symFile);
            int s;
            while (is >> s)
                syms.push_back(s);
        }
        MCGAL::Vertex* a = nearestVertex(mesh.get(), seedPos[0]);
        MCGAL::Vertex* b = nearestVertex(mesh.get(), seedPos[1]);
        // ⚠️ 参数次序：`halfedgeBetween(x,y)` 返回的是 **tgt 为 y** 的那条半边
        // （它按 `it->end_vertex()==y` 搜）。编码端记的是 `seedPos[0]=tgt(seed)`，
        // 所以要还原 seed 必须传 `(b, a)`。实测传成 `(a,b)` 会拿到反向半边 ⇒
        // PLUS/MINUS 互换 ⇒ sign 传播全线不同 ⇒ gate 跳过集合不同 ⇒ 符号与 gate
        // 错位 ⇒ 插回位置错。**这一个反向就解释了 §9 的全部症状。**
        MCGAL::Halfedge* seed = halfedgeBetween(b, a);
        if (!seed) {
            std::fprintf(stderr, "[CLR] 解码端找不到种子半边\n");
            return 1;
        }
        pmsf::FrontierConquestEngine::QuantGrid grid;
        std::vector<std::array<int, 3>> deltas;
        {
            std::ifstream gs(wd + "/clr_geom.txt");
            gs >> grid.step >> grid.bboxMin[0] >> grid.bboxMin[1] >> grid.bboxMin[2];
            int a, b, c;
            while (gs >> a >> b >> c)
                deltas.push_back({a, b, c});
        }
        std::fprintf(stderr, "[CLR] 解码端种子：tgt(seed)=(%.4f,%.4f,%.4f) 期望=(%.4f,%.4f,%.4f)\n",
                     tgt(seed) ? tgt(seed)->x() : -9, tgt(seed) ? tgt(seed)->y() : -9,
                     tgt(seed) ? tgt(seed)->z() : -9, seedPos[0][0], seedPos[0][1], seedPos[0][2]);
        std::fprintf(stderr, "[CLR] 解码端种子：tgt(opp)=(%.4f,%.4f,%.4f) 期望=(%.4f,%.4f,%.4f)\n",
                     tgt(opp(seed)) ? tgt(opp(seed))->x() : -9, tgt(opp(seed)) ? tgt(opp(seed))->y() : -9,
                     tgt(opp(seed)) ? tgt(opp(seed))->z() : -9, seedPos[1][0], seedPos[1][1], seedPos[1][2]);
        ConquestState st;
        std::vector<std::string> trace;
        const int rc = decodeLayer(mesh.get(), st, seed, syms, &trace, grid, deltas, &decSeq, &decSym, &decPush, &decAct, &decIns, &rings);
        if (rc != 0) {
            std::fprintf(stderr, "[CLR] decodeLayer 失败 rc=%d，尾部 5 条轨迹：\n", rc);
            for (size_t i = trace.size() > 5 ? trace.size() - 5 : 0; i < trace.size(); ++i)
                std::fprintf(stderr, "   %s\n", trace[i].c_str());
            return 2;
        }
        if (!dumpOff(mesh.get(), decOff.c_str())) {
            std::fprintf(stderr, "[CLR] dump 解码结果失败\n");
            return 1;
        }
    }

    // ---- ③.5 逐 gate 前端点位置序列对拍（验证 gate 对齐）------------------
    {
        std::vector<std::array<double, 3>> enc;
        {
            // ⚠️ **按行读**：trace 现在第 6 个字段起是 push 序列的 `(x,y,z)` 元组，
            // 按 token 读会在元组处解析失败、循环在第一行就停
            // （踩过：读数变成"编码端 1 条"，后面的并排全都没跑）。
            std::ifstream is(wd + "/clr_gatetrace_enc.txt");
            std::string line;
            while (std::getline(is, line)) {
                std::istringstream ls(line);
                double x, y, z;
                std::string kind;
                size_t npush;
                if (!(ls >> x >> y >> z >> kind >> npush))
                    continue;
                enc.push_back({x, y, z});
                encKind.push_back(kind);
                encPush.push_back(npush);
            }
        }
        double stp = 0;
        {
            std::ifstream gs(wd + "/clr_geom.txt");
            gs >> stp;
        }
        std::fprintf(stderr, "[CLR] gate 序列：编码端 %zu 条 / 解码端 %zu 条%s\n", enc.size(), decSeq.size(),
                     decSeq.empty()
                         ? "  ← 默认模式走 src 版，**没有**逐 gate 诊断序列；要看这里请设 PMSF_CLR_LEGACYDEC=1"
                         : "");
        const size_t n2 = std::min(enc.size(), decSeq.size());
        {   // 位移扫描：定位错位是不是"整体差一格"这类**系统性**偏移
            auto eq = [&](size_t i, size_t j) {
                if (i >= enc.size() || j >= decSeq.size())
                    return false;
                const double dx = enc[i][0] - decSeq[j][0], dy = enc[i][1] - decSeq[j][1],
                             dz = enc[i][2] - decSeq[j][2];
                return std::sqrt(dx * dx + dy * dy + dz * dz) <= stp * 0.5 + 1e-12;
            };
            std::fprintf(stderr, "[CLR] 位移扫描（dec[j] 对 enc[j+shift] 的吻合数 / %zu）：\n", decSeq.size());
            for (int sh = -3; sh <= 3; ++sh) {
                size_t c3 = 0;
                for (size_t j = 0; j < decSeq.size(); ++j) {
                    const long long i = (long long)j + sh;
                    if (i >= 0 && eq((size_t)i, j))
                        ++c3;
                }
                std::fprintf(stderr, "[CLR]   shift=%+d : %zu\n", sh, c3);
            }
        }
        size_t firstBad = (size_t)-1, badCount = 0;
        double worst = 0;
        for (size_t i = 0; i < n2; ++i) {
            const double dx = enc[i][0] - decSeq[i][0], dy = enc[i][1] - decSeq[i][1],
                         dz = enc[i][2] - decSeq[i][2];
            const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (d > stp * 0.5 + 1e-12) {
                {
            std::vector<std::array<double, 3>> encDel;
            for (size_t i = 0; i < enc.size() && i < encKind.size(); ++i)
                if (encKind[i] == "DELETE")
                    encDel.push_back(enc[i]);
            std::fprintf(stderr, "[CLR] **重插点并排**：编码端删 %zu 次 / 解码端插 %zu 次\n", encDel.size(),
                         decIns.size());
            double stp2 = 0, bmin2[3] = {0, 0, 0};
            {
                std::ifstream gs(wd + "/clr_geom.txt");
                gs >> stp2 >> bmin2[0] >> bmin2[1] >> bmin2[2];
            }
            auto cl2 = [&](double x, int k) { return (long long)std::ceil((x - bmin2[k]) / stp2) - 1; };
            int over = 0;
            for (size_t i = 0; i < encDel.size() && i < decIns.size(); ++i) {
                const double d = std::sqrt((encDel[i][0] - decIns[i][0]) * (encDel[i][0] - decIns[i][0]) +
                                           (encDel[i][1] - decIns[i][1]) * (encDel[i][1] - decIns[i][1]) +
                                           (encDel[i][2] - decIns[i][2]) * (encDel[i][2] - decIns[i][2]));
                if (d > stp2 * 0.5 + 1e-12)
                    ++over;
                if (i < 20 || d > stp2 * 0.5 + 1e-12)
                    std::fprintf(stderr,
                                 "[CLR]   #%zu 原(%lld,%lld,%lld) 重建(%lld,%lld,%lld) Δ格=(%lld,%lld,%lld) 距离=%.3g 半步%s\n",
                                 i, cl2(encDel[i][0], 0), cl2(encDel[i][1], 1), cl2(encDel[i][2], 2),
                                 cl2(decIns[i][0], 0), cl2(decIns[i][1], 1), cl2(decIns[i][2], 2),
                                 cl2(decIns[i][0], 0) - cl2(encDel[i][0], 0),
                                 cl2(decIns[i][1], 1) - cl2(encDel[i][1], 1),
                                 cl2(decIns[i][2], 2) - cl2(encDel[i][2], 2), d / stp2,
                                 (d > stp2 * 0.5 + 1e-12) ? " **超差**" : "");
            }
            std::fprintf(stderr, "[CLR] **重插点超差 %d / %zu**\n", over, encDel.size());
        }
        for (size_t i = 0; i < 8 && i < encPush.size() && i < decPush.size(); ++i)
            std::fprintf(stderr, "[CMP] gate#%zu 编码端 %s push=%zu | 解码端 %s push=%zu %s\n", i,
                         encKind[i].c_str(), encPush[i], decAct[i].c_str(), decPush[i],
                         (encKind[i] == decAct[i] && encPush[i] == decPush[i]) ? "OK" : "**不等**");
        if (firstBad == (size_t)-1)
                    firstBad = i;
                ++badCount;
                worst = std::max(worst, d);
            }
        }
        if (firstBad == (size_t)-1)
            std::fprintf(stderr, "[CLR] **前端点序列逐位一致**（%zu 个 gate，全部在半步内）\n", n2);
        else
            std::fprintf(stderr, "[CLR] **前端点序列首次分岔于第 %zu 个 gate**（共 %zu/%zu 个不符，最大偏离 %.6g，半步 %.6g）\n",
                         firstBad, badCount, n2, worst, stp * 0.5);
    }

    // ---- ④ 对拍：规范面结构（量化后）--------------------------------------
    // 顶点按位置比较会漏掉拓扑；按**面的规范形式**比较才能同时覆盖
    // A1（V/F）、A2（几何）、A3（拓扑）。
    // 量化后比较：存活点在两侧都是原始值，新插点已是 dequantize(cell)，
    // 再量化一次得到同一个 cell —— 这正是正确性条件本身。
    {
        double step = 0, bmin[3] = {0, 0, 0};
        {
            std::ifstream gs(wd + "/clr_geom.txt");
            gs >> step >> bmin[0] >> bmin[1] >> bmin[2];
        }
        // ⚠️ **必须用编解码自己那一套量化器** `ceil((x-min)/step) - 1`，
        // 而不是 `floor((x-min)/step + 0.5)`（四舍五入）—— 两者是不同的量化器，
        // 在格边界附近会给出**不同的格号**。实测：重建点与原始点相差 < 1 个半步
        // （几何本身是对的），但用四舍五入去比会让 155 项落到相邻格、误判为不等。
        auto qcell = [&](double x, int k) {
            long long c = (long long)std::ceil((x - bmin[k]) / step) - 1;
            return c < 0 ? 0LL : c;
        };

        struct Loaded {
            int V = 0, F = 0;
            std::vector<std::array<long long, 3>> faces;  // 规范形式
        };
        auto loadRaw = [&](const std::string& path, std::vector<std::array<double, 3>>& raw) {
            std::ifstream is(path);
            std::string tok;
            int V, F, d;
            is >> tok >> V >> F >> d;
            raw.resize(V);
            for (int i = 0; i < V; ++i)
                is >> raw[i][0] >> raw[i][1] >> raw[i][2];
        };
        auto load = [&](const std::string& path) {
            Loaded L;
            std::ifstream is(path);
            std::string tok;
            is >> tok >> L.V >> L.F;
            int dummy;
            is >> dummy;
            std::vector<std::array<long long, 3>> pts(L.V);
            for (int i = 0; i < L.V; ++i) {
                double x, y, z;
                is >> x >> y >> z;
                pts[i] = {qcell(x, 0), qcell(y, 1), qcell(z, 2)};
            }
            for (int i = 0; i < L.F; ++i) {
                int n;
                is >> n;
                std::vector<std::array<long long, 3>> fv;
                for (int k = 0; k < n; ++k) {
                    int id;
                    is >> id;
                    if (id >= 0 && id < L.V)
                        fv.push_back(pts[id]);
                }
                std::sort(fv.begin(), fv.end());
                for (const auto& v : fv) {
                    L.faces.push_back(v);
                }
            }
            std::sort(L.faces.begin(), L.faces.end());
            return L;
        };
        Loaded a = load(origOff), b = load(decOff);
        // ---- §47.3 判据：同格 或 相邻格且实际距离 < 1 个半步 ----
        // 两个顶点相距不足一个量化格时，**量化本身就无法区分它们**；
        // 要求"格号全等"是在要求量化器做它做不到的事。
        // 故改用**带容差的面匹配**：A 中每个面能在 B 中找到未匹配的、三个顶点
        // 逐对满足（|Δ格号|≤1 逐维 且 世界坐标距离 < 1 个半步）的面。
        {
            std::vector<std::array<double, 3>> ra4, rb4;
            loadRaw(origOff, ra4);
            loadRaw(decOff, rb4);
            auto cellsOf = [&](const std::vector<std::array<double, 3>>& raw) {
                std::vector<std::array<long long, 3>> out(raw.size());
                for (size_t i = 0; i < raw.size(); ++i)
                    for (int k = 0; k < 3; ++k)
                        out[i][k] = qcell(raw[i][k], k);
                return out;
            };
            auto ca = cellsOf(ra4), cb = cellsOf(rb4);
            auto loadFaces = [&](const std::string& path, const std::vector<std::array<double, 3>>& raw,
                                 const std::vector<std::array<long long, 3>>& cells) {
                std::ifstream is(path);
                std::string tok;
                int V, F, d;
                is >> tok >> V >> F >> d;
                std::vector<std::array<double, 3>> pts(V);
                for (int i = 0; i < V; ++i)
                    is >> pts[i][0] >> pts[i][1] >> pts[i][2];
                std::vector<std::array<int, 3>> fs;
                for (int i = 0; i < F; ++i) {
                    int n, id;
                    is >> n;
                    std::array<int, 3> f{-1, -1, -1};
                    for (int k = 0; k < n; ++k) {
                        is >> id;
                        if (k < 3)
                            f[k] = id;
                    }
                    if (n == 3)
                        fs.push_back(f);
                }
                (void)raw;
                (void)cells;
                return std::make_pair(std::move(pts), std::move(fs));
            };
            auto pa = loadFaces(origOff, ra4, ca), pb = loadFaces(decOff, rb4, cb);
            // **§53.3 最终判据**：两侧顶点用同一个 `toCell` 量化后落在**同一格**
            // （Δ格 == 0）即记为相等。世界坐标距离**不作判据**，只作旁证输出。
            //
            // 理由（两条本库纪律在此合流）：
            //  · §46.3「验的判据与被验的东西必须用同一套量化器」；
            //  · §9.3 「判据不仅要会失败，失败的原因还必须是真原因」。
            // 用"世界距离 < 某个数"作判据，等于要求量化器做它做不到的事：
            // `toWorld(q)=min+(q+0.5)*step` 给格中心、`toCell` 用 ceil、
            // 下边界还有 `q==-1` 夹到 `0` 的夹取，三者叠加使格内距离可达(并略超)0.5*step。
            auto within = [&](const std::array<double, 3>& p, const std::array<long long, 3>& cp,
                              const std::array<double, 3>& q, const std::array<long long, 3>& cq) {
                (void)p;
                (void)q;
                return cp[0] == cq[0] && cp[1] == cq[1] && cp[2] == cq[2];
            };
            std::vector<bool> used(pb.second.size(), false);
            size_t matchedFaces = 0;
            for (const auto& fa : pa.second) {
                for (size_t j = 0; j < pb.second.size(); ++j) {
                    if (used[j])
                        continue;
                    const auto& fb = pb.second[j];
                    // 逐对匹配三个顶点（3! 种排法里找一个全满足的）
                    int perm[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
                    bool ok = false;
                    for (auto& pm : perm) {
                        bool all = true;
                        for (int k = 0; k < 3; ++k)
                            if (!within(pa.first[fa[k]], ca[fa[k]], pb.first[fb[pm[k]]], cb[fb[pm[k]]]))
                                all = false;
                        if (all) {
                            ok = true;
                            break;
                        }
                    }
                    if (ok) {
                        used[j] = true;
                        ++matchedFaces;
                        break;
                    }
                }
            }
            std::fprintf(stderr, "[CLR] **容差面匹配**（同格或相邻格且 <1 半步）：%zu / %zu 个面匹配 ⇒ %s\n",
                         matchedFaces, pa.second.size(),
                         (matchedFaces == pa.second.size() && pa.second.size() == pb.second.size()) ? "**全等**"
                                                                                                   : "**仍有差额**");
            // 把未匹配的面逐条打出：三个顶点的格号 + 世界坐标 + 与对方最近面的逐点距离
            int shown = 0;
            for (size_t i = 0; i < pa.second.size() && shown < 4; ++i) {
                bool any = false;
                for (size_t j = 0; j < pb.second.size(); ++j)
                    if (!used[j])
                        any = true;
                (void)any;
                // 该面是否被匹配过：重算一次（贪心匹配里没有反查表，这里按"能否找到搭档"判）
                bool hasPartner = false;
                for (size_t j = 0; j < pb.second.size() && !hasPartner; ++j) {
                    const auto& fb = pb.second[j];
                    int perm[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
                    for (auto& pm : perm) {
                        bool all = true;
                        for (int k = 0; k < 3; ++k)
                            if (!within(pa.first[pa.second[i][k]], ca[pa.second[i][k]],
                                        pb.first[fb[pm[k]]], cb[fb[pm[k]]]))
                                all = false;
                        if (all) {
                            hasPartner = true;
                            break;
                        }
                    }
                }
                if (hasPartner)
                    continue;
                ++shown;
                std::fprintf(stderr, "[CLR] 未匹配面 #%zu:\n", i);
                for (int k = 0; k < 3; ++k) {
                    const int ia = pa.second[i][k];
                    // 在 B 里找最近顶点，报格号差与该点世界距离
                    double bd = 1e300;
                    int ib = -1;
                    for (size_t j = 0; j < pb.first.size(); ++j) {
                        double d = 0;
                        for (int c = 0; c < 3; ++c) {
                            const double t = pa.first[ia][c] - pb.first[j][c];
                            d += t * t;
                        }
                        if (d < bd) {
                            bd = d;
                            ib = (int)j;
                        }
                    }
                    std::fprintf(stderr,
                                 "[CLR]   原格号=(%lld,%lld,%lld) 解格号=(%lld,%lld,%lld) 世界距离=%.6g（半步=%.6g）%s\n",
                                 ca[ia][0], ca[ia][1], ca[ia][2], cb[ib][0], cb[ib][1], cb[ib][2], std::sqrt(bd),
                                 step * 0.5,
                                 (std::sqrt(bd) <= step * 0.5 + 1e-12) ? "OK" : "**超差**");
                }
            }
        }
        // 只看"三个角都是存活点"的面 —— 存活点两侧都在且位置相同，
        // 所以这部分**纯测拓扑**，与几何重建无关。用来把两类错误分开。
        {
            std::vector<std::array<double, 3>> ra2, rb2;
            loadRaw(origOff, ra2);
            loadRaw(decOff, rb2);
            auto key = [&](const std::array<double, 3>& p,
                           const std::vector<std::array<double, 3>>& set) -> bool {
                for (const auto& q : set)
                    if (p[0] == q[0] && p[1] == q[1] && p[2] == q[2])
                        return true;
                return false;
            };
            auto survOnly = [&](const std::string& path, const std::vector<std::array<double, 3>>& ra3,
                                const std::vector<std::array<double, 3>>& rb3) {
                std::ifstream is(path);
                std::string tok;
                int V, F, d;
                is >> tok >> V >> F >> d;
                std::vector<std::array<std::array<long long, 3>, 3>> faces;
                std::vector<std::array<double, 3>> pts(V);
                for (int i = 0; i < V; ++i)
                    is >> pts[i][0] >> pts[i][1] >> pts[i][2];
                for (int i = 0; i < F; ++i) {
                    int n;
                    is >> n;
                    std::vector<std::array<double, 3>> fv;
                    for (int k = 0; k < n; ++k) {
                        int id;
                        is >> id;
                        fv.push_back(pts[id]);
                    }
                    const bool allSurv = std::all_of(fv.begin(), fv.end(), [&](const std::array<double, 3>& p) {
                        return key(p, ra3) && key(p, rb3);
                    });
                    if (allSurv && n == 3) {
                        std::array<std::array<long long, 3>, 3> f;
                        for (int k = 0; k < 3; ++k)
                            f[k] = {qcell(fv[k][0], 0), qcell(fv[k][1], 1), qcell(fv[k][2], 2)};
                        std::sort(f.begin(), f.end());
                        faces.push_back(f);
                    }
                }
                std::sort(faces.begin(), faces.end());
                return faces;
            };
            auto sa = survOnly(origOff, ra2, rb2), sb = survOnly(decOff, ra2, rb2);
            size_t c2 = 0, i3 = 0, j3 = 0;
            while (i3 < sa.size() && j3 < sb.size()) {
                if (sa[i3] == sb[j3]) {
                    ++c2;
                    ++i3;
                    ++j3;
                } else if (sa[i3] < sb[j3])
                    ++i3;
                else
                    ++j3;
            }
            std::fprintf(stderr, "[CLR] **纯拓扑**（三角均为存活点）：原=%zu 解=%zu 交集=%zu ⇒ 拓扑%s\n", sa.size(),
                         sb.size(), c2, (sa.size() == sb.size() && c2 == sa.size()) ? "**一致**" : "**不一致**");
        }
        // 面度直方图：解码端若留下未切分的多边形，这里立刻看得见
        auto hist = [](const std::string& path) {
            std::ifstream is(path);
            std::string tok;
            int V, F, d;
            is >> tok >> V >> F >> d;
            std::map<int, int> h;
            for (int i = 0; i < V; ++i) {
                double x, y, z;
                is >> x >> y >> z;
            }
            for (int i = 0; i < F; ++i) {
                int n;
                is >> n;
                h[n]++;
                for (int k = 0; k < n; ++k) {
                    int id;
                    is >> id;
                }
            }
            return h;
        };
        auto ha = hist(origOff), hb = hist(decOff);
        std::fprintf(stderr, "[CLR] 面度直方图  原:");
        for (auto& kv : ha)
            std::fprintf(stderr, " %d度×%d", kv.first, kv.second);
        std::fprintf(stderr, "\n[CLR] 面度直方图  解:");
        for (auto& kv : hb)
            std::fprintf(stderr, " %d度×%d", kv.first, kv.second);
        std::fprintf(stderr, "\n");
        // 多重集交集大小：定位"差多少"而不是只报"不等"
        size_t common = 0, i2 = 0, j2 = 0;
        while (i2 < a.faces.size() && j2 < b.faces.size()) {
            if (a.faces[i2] == b.faces[j2]) {
                ++common;
                ++i2;
                ++j2;
            } else if (a.faces[i2] < b.faces[j2])
                ++i2;
            else
                ++j2;
        }
        std::fprintf(stderr, "[CLR] 规范面项 交集=%zu / 原=%zu 解=%zu\n", common, a.faces.size(), b.faces.size());
        // **逐条打出差额**：多重集差集（原−解 与 解−原），各取前几条，
        // 并给出该顶点在两侧的原始坐标，判定是"恰落格边界"还是"真算错"。
        {
            std::vector<std::array<long long, 3>> onlyA, onlyB;
            {
                size_t i4 = 0, j4 = 0;
                while (i4 < a.faces.size() && j4 < b.faces.size()) {
                    if (a.faces[i4] == b.faces[j4]) {
                        ++i4;
                        ++j4;
                    } else if (a.faces[i4] < b.faces[j4])
                        onlyA.push_back(a.faces[i4++]);
                    else
                        onlyB.push_back(b.faces[j4++]);
                }
                for (; i4 < a.faces.size(); ++i4)
                    onlyA.push_back(a.faces[i4]);
                for (; j4 < b.faces.size(); ++j4)
                    onlyB.push_back(b.faces[j4]);
            }
            std::fprintf(stderr, "[CLR] 差额：原独有 %zu 项 / 解独有 %zu 项\n", onlyA.size(), onlyB.size());
            std::vector<std::array<double, 3>> ra3, rb3;
            loadRaw(origOff, ra3);
            loadRaw(decOff, rb3);
            auto dump = [&](const char* tag, const std::vector<std::array<long long, 3>>& v,
                            const std::vector<std::array<double, 3>>& raw) {
                for (size_t i = 0; i < v.size() && i < 6; ++i) {
                    std::fprintf(stderr, "[CLR]   %s 格号=(%lld,%lld,%lld)\n", tag, v[i][0], v[i][1], v[i][2]);
                    // 在原始坐标里找与之格号最接近的点，打印其真实坐标与距离
                    double best = 1e300;
                    std::array<double, 3> bp{0, 0, 0};
                    for (const auto& p : raw) {
                        double d = 0;
                        for (int k = 0; k < 3; ++k) {
                            const double c = (double)v[i][k] * step + bmin[k];
                            d += (p[k] - c) * (p[k] - c);
                        }
                        if (d < best) {
                            best = d;
                            bp = p;
                        }
                    }
                    std::fprintf(stderr, "[CLR]      最近真实点=(%.6f,%.6f,%.6f) 距离=%.6g（半步=%.6g）\n", bp[0], bp[1],
                                 bp[2], std::sqrt(best), step * 0.5);
                }
            };
            dump("原独有", onlyA, ra3);
            dump("解独有", onlyB, rb3);
        }
        // 最近邻偏离：区分"几何错"与"拓扑错"。几何对 ⇒ 每个原顶点都能在
        // 解码网格里找到 ≤ 1 个半步的对应点。
        {
            std::vector<std::array<double, 3>> ra, rb;
            loadRaw(origOff, ra);
            loadRaw(decOff, rb);
            double worst = 0, sum = 0;
            for (const auto& p : ra) {
                double best = 1e300;
                for (const auto& q : rb) {
                    const double dx = p[0] - q[0], dy = p[1] - q[1], dz = p[2] - q[2];
                    best = std::min(best, dx * dx + dy * dy + dz * dz);
                }
                best = std::sqrt(best);
                worst = std::max(worst, best);
                sum += best;
            }
            int hit = 0;
            for (const auto& p : ra) {
                double best = 1e300;
                for (const auto& q : rb) {
                    const double dx = p[0] - q[0], dy = p[1] - q[1], dz = p[2] - q[2];
                    best = std::min(best, dx * dx + dy * dy + dz * dz);
                }
                if (std::sqrt(best) <= step * 0.5 + 1e-12)
                    ++hit;
            }
            std::fprintf(stderr, "[CLR] 最近邻偏离：最大=%.6g 平均=%.6g（半步=%.6g）\n", worst, sum / ra.size(),
                         step * 0.5);
            std::fprintf(stderr, "[CLR] 逐点命中：%d/%zu 个原顶点能在半步内找到对应（预期 %zu）\n", hit, ra.size(),
                         ra.size() - 34);
            // **§51.4 判别**：分别数"精确对应（距离 0）"与"半步内但非 0"。
            // 幸存点两侧应当逐位相同 ⇒ 精确对应数应 ≈ 114（148−34）。
            int exact = 0, nearOnly = 0;
            for (const auto& p : ra) {
                double best = 1e300;
                for (const auto& q2 : rb) {
                    const double dx = p[0] - q2[0], dy = p[1] - q2[1], dz = p[2] - q2[2];
                    best = std::min(best, dx * dx + dy * dy + dz * dz);
                }
                const double d = std::sqrt(best);
                if (d == 0.0)
                    ++exact;
                else if (d <= step * 0.5 + 1e-12)
                    ++nearOnly;
            }
            std::fprintf(stderr,
                         "[CLR] **§51.4 判别**：精确对应（距离0）=%d（幸存点预期 %zu）；"
                         "半步内但非 0 = %d；半步外 = %zu\n",
                         exact, ra.size() - 34, nearOnly, ra.size() - exact - nearOnly);
        }
        const bool vfOk = (a.V == b.V && a.F == b.F);
        const bool topoOk = (a.faces == b.faces);
        std::printf("RESULT V原=%d V解=%d F原=%d F解=%d vfOk=%d topoGeomOk=%d\n", a.V, b.V, a.F, b.F,
                    (int)vfOk, (int)topoOk);
    }
    return 0;
}
