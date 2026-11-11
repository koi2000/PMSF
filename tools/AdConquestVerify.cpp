// SPDX-License-Identifier: Apache-2.0
//
// tools/AdConquestVerify.cpp —— P13.1 独立验证器：在 MCGAL 上重跑 AD
// (Alliez & Desbrun 2001) 的 decimation conquest，产出**连通性符号序列**，
// 与 MEPP2 oracle 对拍（tools/mepp2_reference/oracle_diff.py）。
//
// 判据 = **符号序列一致**，不是字节一致（熵编码器不同，字节必然不同）。
//
// 参考实现：reference/MEPP2/.../Compression_Valence/Compression_Valence_Component.inl
//   Decimation_Conquest   .inl:1342-2282
//   Find_Type             .inl:6755-6842
//   Is_Manifold_Property_Violated .inl:6843-7081
//   Retriangulation       .inl:7820-8225
// 蒸馏笔记见 docs/refactor/P13_1_ORACLE_BASELINE_2026-09-22.md
//
// **不参与框架主构建**：本文件只被显式 target 编译（见 CMakeLists 中的注释），
// 不影响九算法产物与默认路径。
//
// 用法： AdConquestVerify <mesh.off> <out_symbols.txt>

#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Point.h"
#include "Vertex.h"

#include "common/BoundedRingTraversal.h"  // pmsf::boundedRing（D17 的有界顶点环）
#include "core/MeshRuntime.h"             // pmsf::ensureMeshRuntimeInitialized()

#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <memory>
#include <set>
#include <string>
#include <unistd.h>
#include <unordered_map>
#include <vector>

namespace {

static bool g_trace = false;
static int g_maxV = 0;
static float g_bbox[6] = {0, 0, 0, 0, 0, 0};
static double g_bboxSpan = 0.0;  // <=0 表示跑到收敛；>0 时复刻 oracle 的停止规则
static long long g_symIdx = 0;
static long long g_illegalSplits = 0;

// 上一次 manifold 判定的现场（见 manifoldViolated 末尾的记账说明）。
static MCGAL::Halfedge* g_lastH = nullptr;
static int g_lastType = -1;
static int g_lastVal = -1;
static std::vector<int> g_lastP;
static std::vector<std::pair<int, int>> g_lastWalk;  // P 构造时逐步的 (source,target)
static bool g_lastViolated = false;
static long long g_lastSym = -1;
static long long g_manifoldCalls = 0;
static long long g_manifoldHits = 0;

// ---- 状态（用局部 map，不污染 MCGAL 自身的 tag/proxy 状态）------------------
// VREMOVED = 参考实现的 TO_BE_REMOVED。**Regulation 需要它**：
// 它的 Phase A 只打标记，真正的删点在 Phase B 按顶点顺序做（文档 §12.3）。
enum VFlag { VFREE = 0, VCONQUERED = 1, VREMOVED = 2 };
enum Vsig { NOSIGN = 0, PLUS = 1, MINUS = 2 };
enum FFlag { FFREE = 0, FCONQUERED = 1, FTO_BE_REMOVED = 2, FTEMP = 3 };

struct State {
    std::unordered_map<MCGAL::Vertex*, int> vflag;
    std::unordered_map<MCGAL::Vertex*, int> vsign;
    std::unordered_map<MCGAL::Facet*, int> fflag;

    int flag(MCGAL::Vertex* v) const {
        auto it = vflag.find(v);
        return it == vflag.end() ? VFREE : it->second;
    }
    void setFlag(MCGAL::Vertex* v, int f) {
        vflag[v] = f;
    }
    int sign(MCGAL::Vertex* v) const {
        auto it = vsign.find(v);
        return it == vsign.end() ? NOSIGN : it->second;
    }
    void setSign(MCGAL::Vertex* v, int s) {
        if (sign(v) == NOSIGN)
            vsign[v] = s;  // 参考实现：仅在 NOSIGN 时写入
    }
    int fflagOf(MCGAL::Facet* f) const {
        auto it = fflag.find(f);
        return it == fflag.end() ? FFREE : it->second;
    }
    void setFFlag(MCGAL::Facet* f, int v) {
        fflag[f] = v;
    }

    // 复刻参考实现的 `Init()`（.inl:1190）—— **每一次 Decimation_Conquest
    // 调用开头都会调它**，把三种 flag 全部复位。漏掉这一步会让第 2 层及以后
    // 一个点都删不掉（顶点全停留在上一层的 CONQUERED 上），表现为
    // "round 1: removed=0"、产物止步于第 0 层。
    // 注意 `vertex_Seed_Edge` **不**复位：它只在初始 setup 时写一次
    // （.inl:371/374），故每层都用**同一条种子边**（本工具的 seed 复用是对的）。
    void reset() {
        vflag.clear();
        vsign.clear();
        fflag.clear();
    }
};

// CGAL/BGL 的 target(h) == MCGAL 的 h->end_vertex()；source(h) == h->vertex()。
inline MCGAL::Vertex* target(MCGAL::Halfedge* h) {
    return h ? h->end_vertex() : nullptr;
}
inline MCGAL::Halfedge* next_(MCGAL::Halfedge* h) {
    return h ? h->next() : nullptr;
}
inline MCGAL::Halfedge* opp(MCGAL::Halfedge* h) {
    return h ? h->opposite() : nullptr;
}

// ---- P13.2 几何通路（规格逐行读自参考实现；文档 §17/§19）------------------
static int g_qbits = 10;
static bool g_geo = false;
static long long g_delCount = 0;
static long long g_distMax[3] = {0, 0, 0};
static double g_distSum2 = 0.0;
static std::vector<double> g_reconPos;  // 被删点的重建坐标 = dequantize(CRV)

// 量化：Q = (int)(ceil((x-min)/step)) - 1; 若为 -1 则夹到 0。（.inl:6256-6264）
inline int Qz(double c, double mn, double step) {
    int q = (int)std::ceil((c - mn) / step) - 1;
    if (q == -1)
        q = 0;
    return q;
}
// 反量化：x = min + (Q + 0.5) * step（取格中心）。（.inl:6294-6296）
inline double Dqz(int q, double mn, double step) {
    return mn + ((double)q + 0.5) * step;
}
// `Barycenter_Patch_Before_Removal`（.inl:7756-7782）：**被删点邻居位置的算术平均**，
// 无面积加权。参考实现绕 target(next(g)) 的 around-target 圈取
// target(opposite(*vh_it))；boundedRing(v)（出边圈）的 target(out) 是同一组邻居。
inline void barycenterOf(MCGAL::Vertex* v, double out[3]) {
    double x = 0, y = 0, z = 0;
    int n = 0;
    for (MCGAL::Halfedge* o : pmsf::boundedRing(v)) {
        MCGAL::Vertex* nb = target(o);
        if (!nb)
            continue;
        x += nb->x();
        y += nb->y();
        z += nb->z();
        ++n;
    }
    if (n == 0) {
        out[0] = out[1] = out[2] = 0;
        return;
    }
    out[0] = x / n;
    out[1] = y / n;
    out[2] = z / n;
}
// 探测：MCGAL `find_prev` 对**非三角面**走无界 while。若我们在非三角面上调用它，
// 就可能挂死。这里显式报告"在非三角面上调用了 prev_"，把假设变成可观测。
inline long long g_prevOnNonTri = 0;
inline long long g_prevBrokenRing = 0;
inline int g_lastNonTriDegree = -1;
inline MCGAL::Halfedge* prev_(MCGAL::Halfedge* h) {
    if (!h)
        return nullptr;
    // ⚠️ 2026-09-22：**不再信任 `Facet::facet_degree()`**，无条件走环游。
    //
    // 起因（第 1 层首个分歧的决定现场，见文档 §9）：绕前端点的 P-walk 在
    // valence=5 时**提前一圈闭合**（源顶点 5666→5670→5676→5677→**5666**），
    // 少走了一个邻居 ⇒ Points_index 失准 ⇒ manifold 判据放行了本该拒绝的删除。
    //
    // MCGAL 的 `find_prev()`（Facet.h:398）用 `face_->facet_degree() == 3` 做
    // 分支：真则走三角面快捷路径 `next()->next()`，假才走环游。而 `facet_degree_`
    // 是**存量计数器**（Facet.h:380，默认 3；`Facet::reset` 时按环重算）。
    // 只要某个面在 `erase_center_vertex` 之后 degree 没被刷新，
    // 快捷路径就会在一个**真多边形**上返回**静默错误**的 prev ——
    // 没有崩溃、没有告警，只是 P-walk 少走一格。
    //
    // 环游版在三角面上与快捷路径**逐字等价**（闭环三角形走 2 步即回到自身），
    // 在多边形上才是真 prev ⇒ 无条件用环游**不改变任何良构情形的语义**。
    //
    // ⚠️ **本假设已被实测证伪**（同日）：改成无条件环游后，失败点与 P-walk
    // **逐字节相同**（同样 sym#4008、同样 P=[5666,5670,5676,5677,5666]）。
    // ⇒ 本次失败**不是** `facet_degree_` 失准造成的。改动保留（语义等价、
    // 且消掉了 `find_prev` 对存量计数器的信任），但**不要**把它当成修复。
    // 真实卡点在顶点环自身，见文档 §9。
    MCGAL::Facet* f = h->face();
    if (f && !f->isRemoved() && f->facet_degree() != 3)
        ++g_prevOnNonTri;
    MCGAL::Halfedge* g = h;
    for (int i = 0; i < 512; ++i) {
        MCGAL::Halfedge* n = g->next();
        if (!n)
            return nullptr;
        if (n == h)
            return g;
        g = n;
    }
    ++g_prevBrokenRing;
    return nullptr;
}

// ---- 边界判定 ------------------------------------------------------------
// ⚠️ 2026-09-22：这两个函数**此前都是按语义猜的**，从未逐行读过参考实现。
// 用带边界网格一测就崩（文档 §14.3）。现按 .inl:6729-6749 与 CGAL 的
// `is_border_edge` 语义重写，两处都是**实质改动**：
//
//   ① `isBorderEdge` 此前**只看了 opposite 一侧**；CGAL 的
//      `is_border_edge(e)` = `is_border(e) || is_border(opposite(e))`，
//      是**对称**的。只看一侧 ⇒ 当 h 自己的面是边界而 opposite 不是时漏判。
//   ② `isBorderVertex` 此前写的是"沿 `h->next()` 走一圈"—— 那是**面**环，
//      问的是"这个面有没有边界边"；参考实现要的是**顶点**环：
//      `Halfedge_around_target_circulator(target(h))`，问的是
//      "**target(h) 这个顶点**的环上有没有边界边"。**两者是完全不同的问题。**
//      闭网格上两者恰好都恒 false ⇒ 一直没暴露。
//
// 边是无向的：`is_border_edge` 对一条边的两个方向同真同假。
// `boundedRing(v)` 产出边，参考实现用入边圈 —— 二者覆盖**同一组无向边**，故等价。
inline bool isBorderEdge(MCGAL::Halfedge* h) {
    if (!h)
        return true;
    // 两侧都看（CGAL is_border_edge 的对称语义）
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
inline bool isBorderVertex(MCGAL::Halfedge* h) {
    // 参考实现 Is_Border_Vertex（.inl:6729-6749）：绕 **target(h)** 的顶点圈。
    if (!h)
        return true;
    MCGAL::Vertex* v = target(h);
    if (!v)
        return true;
    for (MCGAL::Halfedge* o : pmsf::boundedRing(v))
        if (isBorderEdge(o))
            return true;
    return false;
}

// ---- Find_Type (.inl:6755)：type = 4*(valence-3) + {1:(-,+),2:(+,-),3:(+,+),4:(-,-)} ---
int findType(const State& S, MCGAL::Halfedge* h, int valence) {
    const int a = S.sign(target(h));       // sign of target(h)
    const int b = S.sign(target(opp(h)));  // sign of target(opposite(h))
    int k = 0;
    if (a == MINUS && b == PLUS)
        k = 1;
    else if (a == PLUS && b == MINUS)
        k = 2;
    else if (a == PLUS && b == PLUS)
        k = 3;
    else if (a == MINUS && b == MINUS)
        k = 4;
    if (k == 0) {
        static long long dbg = 0;
        return 0;  // 参考实现里未覆盖的组合 ⇒ type 0
    }
    return 4 * (valence - 3) + k;
}

// ---- Is_Manifold_Property_Violated (.inl:6843) ----------------------------
// **默认运行点下唯一生效的判据**（Normal_flipping / Use_metric 均 false）。
//
// 注意（我第一版蒸馏写错过，见文档 §3.8 更正）：Points_index **不是**边界点表，
// 而是**函数内当场从前端点环上构造**的顶点编号数组（逆时针）：
//     Points_index[0] = VN[target(next(h))]
//     g = next(next(h)); for i in 1..valence-1: g = prev(opposite(g)); Points_index[i] = VN[source(g)]
// 检查 = "新边的端点是否与环上某个已有顶点重合"（重合 ⇒ 新边已存在 ⇒ 破坏流形性）。
// **对闭网格完全生效。**
//
// 各 type 检查的 Points_index 下标（逐条读自 .inl:6890-7081）：
//   4: {5,8}->{1}   {6,7}->{2}
//   5: {9,12}->{1,3}  10->{3,3}  11->{2,2}
//   6: {13,16}->{1,3,3}  {14,15}->{2,4,2}
// ⚠️ 2026-09-22 重写。**上一版有三处系统性错误**，是 fandisk 首个分歧
// （oracle=4 / candidate=3）与 cow_simp 断言的共同根因。逐条对照 .inl:6843-7077：
//
//   ① `Points_index[0]` 参考实现是 `VN[target(h)]`，上一版写成 `VN[target(next(h))]`；
//   ② 绕行起点参考实现是 `g = next(h)`，上一版多走了一步（`next(next(h))`）；
//   ③ 判断子用的是**「环心」的邻居**：`VN[target(opposite(*Hvc))]`，而 `*Hvc` 是
//      around-**target** circulator。上一版写 `Hvc->vertex()`，在 around-**source**
//      的 `boundedRing` 里那恰好是**环心自己** ⇒ 判据退化成"环心 == P[k]"，
//      几乎恒假 ⇒ **漏判** ⇒ 该发 null patch(4) 的地方发了 delete ⇒ 后续
//      重三角化切出已存在的对角线 ⇒ MCGAL reset 断言。
//   ④ 环心**按 type 不同**，不是恒为 `target(opposite(h))`：6/7、10、11、14/15 走
//      `target(h)`。
//
// 参考实现逐 type 的判据（.inl:6871-7072，circulate 一律意为
// "绕该顶点一圈，看是否有邻居 == P[k]"）：
//
//   valence 4: type 5/8  -> circ(target(opp(h))) ∋ P[1]
//              type 6/7  -> circ(target(h))      ∋ P[2]
//   valence 5: type 9/12 -> circ(target(opp(h))) ∋ P[1];
//                           circ(target(next(opp(next(h))))) ∋ P[3]
//              type 10   -> circ(target(h))      ∋ P[3];
//                           circ(target(next(opp(next(h))))) ∋ P[3]
//              type 11   -> circ(target(h))      ∋ P[2];
//                           circ(target(opp(h))) ∋ P[2]
//   valence 6: type 13/16 -> circ(target(opp(h))) ∋ P[1];
//                            circ(target(opp(h))) ∋ P[3];
//                            circ(target(next(opp(next(h))))) ∋ P[3]
//              type 14/15 -> circ(target(h))      ∋ P[2];
//                            circ(target(h))      ∋ P[4];
//                            circ(target(next(opp(prev(h))))) ∋ P[2]
bool manifoldViolated(MCGAL::Halfedge* h, int type, int valence) {
    if (valence == 3)
        return false;  // 不插新边，恒安全

    // ---- 顶点环健康度探针（文档 §9 写明的下一步）----------------------------
    // 四数一比即可区分两种完全不同的病因：
    //   cached≠fresh            ⇒ 「缓存陈旧」——degree_dirty_ 没被置位
    //   ringBounded≠ringPrev    ⇒ 「两种环遍历互相矛盾」⇒ 环本身不一致
    //   两者都等于 valence 却仍出错 ⇒ 环和 degree 都自洽，问题在别处
    if (g_trace) {
        MCGAL::Vertex* ctr = target(next_(h));
        if (ctr) {
            // ⚠️ 注意：`vertex_degree()` **只在脏时**才重算，脏位为 0 时它返回的就是缓存，
            // 所以"cached==fresh"**不能**用来判定缓存是否正确（我第一版探针就犯了这个错）。
            // 真正有效的判据是：**强制** `recompute_degree()`（走 MCGAL 自己的无界环
            // 迭代器做一次真实计数），再与缓存比。
            const bool dirty = ctr->is_degree_dirty();
            const int cached = ctr->vertex_degree_cached();
            ctr->recompute_degree();  // 强制真实重算（会清脏位）
            const int fresh = ctr->vertex_degree_cached();
            int ringBounded = 0;
            for (MCGAL::Halfedge* o : pmsf::boundedRing(ctr)) {
                (void)o;
                ++ringBounded;
            }
            int ringPrev = 0;
            MCGAL::Halfedge* g0 = next_(h);
            if (g0) {
                MCGAL::Halfedge* w = g0;
                for (int i = 0; i < 64; ++i) {
                    MCGAL::Halfedge* nx = prev_(opp(w));
                    if (!nx || nx == g0)
                        break;
                    w = nx;
                    ++ringPrev;
                }
                ++ringPrev;  // 回到起点算一环
            }
            if (cached != fresh || ringBounded != valence || ringPrev != valence)
                std::fprintf(stderr,
                             "RING_PROBE sym#%lld val=%d cached=%d fresh=%d dirty=%d "
                             "ringBounded=%d ringPrev=%d center=v%d\n",
                             g_symIdx, valence, cached, fresh, (int)dirty, ringBounded, ringPrev, ctr->poolId());
        }
    }

    // 1) Points_index（.inl:6860-6869）—— 注意 P[0] 取 target(h)，不是 target(next(h))
    std::vector<int> P(valence, -1);
    MCGAL::Halfedge* g = h;
    if (!g)
        return false;
    P[0] = target(g) ? target(g)->poolId() : -1;
    g = next_(g);  // g 现在指向中心点（被删的那个顶点）
    g_lastWalk.clear();
    for (int i = 0; g && i < valence; ++i)
        g_lastWalk.push_back({(int)(g->vertex() ? g->vertex()->poolId() : -1), (int)(target(g) ? target(g)->poolId() : -1)});
    for (int i = 1; i < valence; ++i) {
        if (!g)
            break;
        g = prev_(opp(g));  // 绕中心点逆时针推进
        if (!g)
            break;
        // 参考实现取 target(opposite(g))；source(g) 与之等价。
        MCGAL::Vertex* src = g->vertex();
        P[i] = src ? src->poolId() : -1;
        g_lastWalk.push_back({(int)(src ? src->poolId() : -1), (int)(target(g) ? target(g)->poolId() : -1)});
    }

    // 2) `circ(e) ∋ k`：绕 target(e) 一圈，看是否有邻接顶点的编号 == P[k]
    //    （对应参考实现的 CGAL_For_all(Hvc, Hvc_end) { VN[target(opposite(*Hvc))] }）
    auto circHits = [&P](MCGAL::Halfedge* e, int k) -> bool {
        if (!e || k < 0 || k >= (int)P.size() || P[k] < 0)
            return false;
        MCGAL::Vertex* c = target(e);
        if (!c)
            return false;
        // boundedRing 产出**出边**，其 target 正是 around-target 圈的
        // target(opposite(*Hvc))。
        for (MCGAL::Halfedge* out : pmsf::boundedRing(c)) {
            MCGAL::Vertex* nb = target(out);
            if (nb && nb->poolId() == P[k])
                return true;
        }
        return false;
    };

    // 3) 逐 type 判据（严格照抄上表）
    bool res = false;
    if (valence == 4) {
        if (type == 5 || type == 8)
            res = circHits(opp(h), 1);
        else if (type == 6 || type == 7)
            res = circHits(h, 2);
    } else if (valence == 5) {
        if (type == 9 || type == 12) {
            MCGAL::Halfedge* e2 = next_(opp(next_(h)));
            res = circHits(opp(h), 1) || circHits(e2, 3);
        } else if (type == 10) {
            MCGAL::Halfedge* e2 = next_(opp(next_(h)));
            res = circHits(h, 3) || circHits(e2, 3);
        } else if (type == 11)
            res = circHits(h, 2) || circHits(opp(h), 2);
    } else if (valence == 6) {
        if (type == 13 || type == 16) {
            MCGAL::Halfedge* e2 = next_(opp(next_(h)));
            res = circHits(opp(h), 1) || circHits(opp(h), 3) || circHits(e2, 3);
        } else if (type == 14 || type == 15) {
            MCGAL::Halfedge* e2 = next_(opp(prev_(h)));
            res = circHits(h, 2) || circHits(h, 4) || circHits(e2, 2);
        }
    }

    // 记账：把**本次判定的完整现场**留下。若这次判 false（放行删除）而随后
    // 切分非法，`doSplit` 会把这份现场打出来 —— 这比"事后从调用点反推"可靠，
    // 是本轮三次反推全错之后换的做法。
    g_lastH = h;
    g_lastType = type;
    g_lastVal = valence;
    g_lastP = P;
    g_lastViolated = res;
    g_lastSym = g_symIdx;
    ++g_manifoldCalls;
    if (res)
        ++g_manifoldHits;
    return res;
}

// null-patch 的**尾部**（参考实现里"manifold 违规"分支与"最终 else"分支是复制粘贴的
// 孪生体，尾部逐字相同）：4 路 sign 传播 + **两个额外的队列 push**。
// 先前只实现了 2 路传播、且**完全漏掉了两个 push** —— 后者会让遍历前沿从第一个
// null patch 起就分叉，是 #133 分岔的根因。
inline void nullPatchTail(State& S, MCGAL::Halfedge* h, std::deque<MCGAL::Halfedge*>& Q) {
    MCGAL::Vertex* vA = target(h);
    MCGAL::Vertex* vB = target(opp(h));
    MCGAL::Vertex* vF = target(next_(h));
    const int sA = S.sign(vA), sB = S.sign(vB);
    if (sA == PLUS && sB == MINUS)
        S.setSign(vF, PLUS);
    else if (sA == MINUS && sB == PLUS)
        S.setSign(vF, PLUS);
    else if (sA == PLUS && sB == PLUS)
        S.setSign(vF, MINUS);
    else if (sA == MINUS && sB == MINUS)
        S.setSign(vF, PLUS);

    if (!isBorderEdge(next_(h)))
        Q.push_back(opp(next_(h)));
    if (!isBorderEdge(prev_(h)))
        Q.push_back(opp(prev_(h)));
}

// 参考实现的前置条件：`split_face(h,g)` 要求 target(h) 与 target(g) 之间**尚无边**
// （CGAL insert_tip 语义：新边连 target(h)–target(g)）。违反 ⇒ MCGAL reset 断言。
// 这里在调用前主动探测，把"哪个 case、哪两个顶点"打出来 —— 比等断言好定位。
inline bool splitWouldBeIllegal(MCGAL::Halfedge* h, MCGAL::Halfedge* g) {
    MCGAL::Vertex* a = target(h);
    MCGAL::Vertex* b = target(g);
    if (!a || !b || a == b)
        return true;
    // **必须用有界环**：MCGAL 的 `Vertex::find_halfedge` 走 `for (hit : *this)`，
    // 而该迭代器沿 cw/ccw 绕圈、**只在走回起点时终止** ⇒ 环不闭合即死循环（D17）。
    // 这是我自己的探针引入的挂死：cow_simp 的"挂死"是在加了本探针之后才出现的。
    for (MCGAL::Halfedge* hit : pmsf::boundedRing(a)) {
        if (hit && hit->end_vertex() == b)
            return true;
    }
    return false;
}

// 有界打印一个面的环（h 起，沿 next 走），形如 `v1->v2->v3->v1`。
inline void dumpFaceRing(MCGAL::Halfedge* h) {
    if (!h) {
        std::fprintf(stderr, "(null halfedge)");
        return;
    }
    MCGAL::Halfedge* it = h;
    for (int i = 0; i < 64; ++i) {
        if (!it) {
            std::fprintf(stderr, " -> NULL");
            return;
        }
        std::fprintf(stderr, "%sv%d->v%d", i ? " " : "", it->vertex() ? it->vertex()->poolId() : -1,
                     it->end_vertex() ? it->end_vertex()->poolId() : -1);
        it = it->next();
        if (it == h) {
            std::fprintf(stderr, " (closed after %d)", i + 1);
            return;
        }
    }
    std::fprintf(stderr, " (NOT CLOSED in 64)");
}

// 本次 retriangulate 内第几次 split（1 起）。失败时要能说出"三步里的第几步"。
static int g_splitSeq = 0;

inline MCGAL::Halfedge* doSplit(MCGAL::Mesh* mesh, MCGAL::Halfedge* h, MCGAL::Halfedge* g, int type, int valence) {
    ++g_splitSeq;
    if (splitWouldBeIllegal(h, g)) {
        std::fprintf(stderr,
                     "SPLIT_ILLEGAL type=%d val=%d split#%d  target(h)=v%d target(g)=v%d  "
                     "(already adjacent)\n",
                     type, valence, g_splitSeq, target(h) ? target(h)->poolId() : -1, target(g) ? target(g)->poolId() : -1);
        std::fprintf(stderr, "  h  = v%d->v%d   g = v%d->v%d\n", h && h->vertex() ? h->vertex()->poolId() : -1, target(h) ? target(h)->poolId() : -1,
                     g && g->vertex() ? g->vertex()->poolId() : -1, target(g) ? target(g)->poolId() : -1);
        std::fprintf(stderr, "  face(h) ring: ");
        dumpFaceRing(h);
        std::fprintf(stderr, "\n  face(g) ring: ");
        dumpFaceRing(g);
        std::fprintf(stderr, "\n");
        // **上一次 manifold 判定的现场** —— 这次非法切分是谁放行的。
        std::fprintf(stderr, "  last manifold verdict: sym#%lld type=%d val=%d violated=%d P=[", g_lastSym, g_lastType, g_lastVal,
                     (int)g_lastViolated);
        for (size_t i = 0; i < g_lastP.size(); ++i)
            std::fprintf(stderr, "%s%d", i ? "," : "", g_lastP[i]);
        std::fprintf(stderr, "]  (manifold: calls=%lld hits=%lld)\n", g_manifoldCalls, g_manifoldHits);
        std::fprintf(stderr, "  P-walk (source->target per step):");
        for (size_t i = 0; i < g_lastWalk.size(); ++i)
            std::fprintf(stderr, " [%d]v%d->v%d", (int)i, g_lastWalk[i].first, g_lastWalk[i].second);
        std::fprintf(stderr, "\n");
        // 判定时的**中心点**（= 前端点 = target(next(gate))）及其**真实环**。
        // P[] 是绕它走出来的；两者一比就知道 P 是不是走歪了。
        if (g_lastH) {
            MCGAL::Vertex* c = target(next_(g_lastH));
            std::fprintf(stderr, "  gate=v%d->v%d  center=%s  ring=[", g_lastH->vertex() ? g_lastH->vertex()->poolId() : -1,
                         target(g_lastH) ? target(g_lastH)->poolId() : -1, c ? std::to_string(c->poolId()).c_str() : "null");
            if (c) {
                int n = 0;
                for (MCGAL::Halfedge* o : pmsf::boundedRing(c)) {
                    std::fprintf(stderr, "%s%d", n++ ? "," : "", target(o) ? target(o)->poolId() : -1);
                }
            }
            std::fprintf(stderr, "] deg=%d\n", c ? c->vertex_degree() : -1);
        }
        std::fflush(stderr);
        ++g_illegalSplits;
    }
    // ⚠️ 这里是**真实切分**。8868cd3 引入本探针时误写成 `return doSplit(...)` 自递归
    // —— 尾调用被 -O3 优化成死循环，让 cow_simp 的"挂死"看起来像引擎缺陷，
    // 实际是我自己的探针。**探针包装体里绝不能出现被包装函数自己的名字。**
    // 不在此处补 degree 失效：**调用方不再依赖 degree 缓存**
    // （主循环读 valence 前一律 `recompute_degree()`，见那里的说明）。
    // 曾试过"切分后只失效新边两端 + 删点前失效环上顶点"——**不够**：
    // 仍有顶点被漏掉，sym#4008 的 cached=5/fresh=4 就是这么来的。
    return mesh->split_facet(h, g);
}

// ---- Retriangulation (.inl:7820)：只做 **sign 传播 + split_face + 标记** ----
// facet_normal / facet_Component_Number / Patch_Index 与连通性符号流无关，略去
// （几何通路是 P13.2）。每个 case 的 split 导航严格照抄参考实现。
void retriangulate(MCGAL::Mesh* mesh, State& S, MCGAL::Halfedge* ch, int valence, int type) {
    MCGAL::Halfedge* h = ch;
    MCGAL::Halfedge* g = next_(h);
    g_splitSeq = 0;
    if (std::getenv("PMSF_ADC_DBG")) {
        std::fprintf(stderr, "RETRI type=%d val=%d ch=v%d->v%d  face ring: ", type, valence, h->vertex() ? h->vertex()->poolId() : -1,
                     target(h) ? target(h)->poolId() : -1);
        dumpFaceRing(h);
        std::fprintf(stderr, "\n");
        std::fflush(stderr);
    }

    if (type == 1 || type == 2 || type == 4) {
        S.setFFlag(h->face(), FTO_BE_REMOVED);
        S.setSign(target(g), PLUS);
    } else if (type == 3) {
        S.setFFlag(h->face(), FTO_BE_REMOVED);
        S.setSign(target(g), MINUS);
    } else if (type == 5 || type == 8) {
        S.setSign(target(g), PLUS);
        S.setSign(target(next_(g)), MINUS);
        h = prev_(h);
        h = doSplit(mesh, h, g, type, valence);
        S.setFFlag(h->face(), FTO_BE_REMOVED);
        S.setFFlag(opp(h)->face(), FTO_BE_REMOVED);
    } else if (type == 6 || type == 7) {
        S.setSign(target(g), MINUS);
        S.setSign(target(next_(g)), PLUS);
        g = next_(g);
        h = doSplit(mesh, h, g, type, valence);
        S.setFFlag(h->face(), FTO_BE_REMOVED);
        S.setFFlag(opp(h)->face(), FTO_BE_REMOVED);
    } else if (type == 9 || type == 12) {
        S.setSign(target(g), PLUS);
        S.setSign(target(next_(g)), MINUS);
        S.setSign(target(next_(next_(g))), PLUS);
        h = prev_(h);
        h = doSplit(mesh, h, g, type, valence);
        S.setFFlag(opp(h)->face(), FTO_BE_REMOVED);
        g = next_(next_(h));
        h = doSplit(mesh, h, g, type, valence);
        S.setFFlag(h->face(), FTO_BE_REMOVED);
        S.setFFlag(opp(h)->face(), FTO_BE_REMOVED);
    } else if (type == 10) {
        S.setSign(target(g), PLUS);
        S.setSign(target(next_(g)), MINUS);
        S.setSign(target(next_(next_(g))), PLUS);
        g = h;
        h = prev_(prev_(h));
        h = doSplit(mesh, h, g, type, valence);
        S.setFFlag(opp(h)->face(), FTO_BE_REMOVED);
        g = next_(h);
        h = prev_(h);
        h = doSplit(mesh, h, g, type, valence);
        S.setFFlag(h->face(), FTO_BE_REMOVED);
        S.setFFlag(opp(h)->face(), FTO_BE_REMOVED);
    } else if (type == 11) {
        S.setSign(target(g), MINUS);
        S.setSign(target(next_(g)), PLUS);
        S.setSign(target(next_(next_(g))), MINUS);
        g = next_(g);
        h = doSplit(mesh, h, g, type, valence);
        S.setFFlag(opp(h)->face(), FTO_BE_REMOVED);
        g = next_(next_(h));
        h = doSplit(mesh, h, g, type, valence);
        S.setFFlag(h->face(), FTO_BE_REMOVED);
        S.setFFlag(opp(h)->face(), FTO_BE_REMOVED);
    } else if (type == 13 || type == 16) {
        S.setSign(target(g), PLUS);
        S.setSign(target(next_(g)), MINUS);
        S.setSign(target(next_(next_(g))), PLUS);
        S.setSign(target(next_(next_(next_(g)))), MINUS);
        h = prev_(h);
        h = doSplit(mesh, h, g, type, valence);
        S.setFFlag(opp(h)->face(), FTO_BE_REMOVED);
        g = next_(next_(h));
        h = doSplit(mesh, h, g, type, valence);
        S.setFFlag(opp(h)->face(), FTO_BE_REMOVED);
        g = next_(next_(h));
        h = doSplit(mesh, h, g, type, valence);
        S.setFFlag(h->face(), FTO_BE_REMOVED);
        S.setFFlag(opp(h)->face(), FTO_BE_REMOVED);
    } else if (type == 14 || type == 15) {
        S.setSign(target(g), MINUS);
        S.setSign(target(next_(g)), PLUS);
        S.setSign(target(next_(next_(g))), MINUS);
        S.setSign(target(next_(next_(next_(g)))), PLUS);
        g = next_(g);
        h = doSplit(mesh, h, g, type, valence);
        S.setFFlag(opp(h)->face(), FTO_BE_REMOVED);
        g = next_(next_(h));
        h = doSplit(mesh, h, g, type, valence);
        S.setFFlag(opp(h)->face(), FTO_BE_REMOVED);
        g = next_(next_(h));
        h = doSplit(mesh, h, g, type, valence);
        S.setFFlag(h->face(), FTO_BE_REMOVED);
        S.setFFlag(opp(h)->face(), FTO_BE_REMOVED);
    }
    // type == 0：Find_Type 未覆盖的组合。参考实现下不应出现；出现即说明
    // sign 传播与参考不符 —— 记录下来，便于定位。
    else {
        std::fprintf(stderr, "[AdConquest] WARN: unhandled type=0 valence=%d\n", valence);
    }
}

// ---- 一轮 Decimation_Conquest（.inl:1342）---------------------------------
// 返回本轮删掉的顶点数；符号追加到 out。
int decimationConquest(MCGAL::Mesh* mesh, State& S, MCGAL::Halfedge* seedFirst, std::vector<int>& out) {
    int Number_vertices = 0;
    std::deque<MCGAL::Halfedge*> Q;

    S.reset();  // == 参考实现的 Init()（.inl:1373 在每次调用开头调它）

    S.setFlag(target(seedFirst), VCONQUERED);
    S.setSign(target(seedFirst), PLUS);
    S.setFlag(target(opp(seedFirst)), VCONQUERED);
    S.setSign(target(opp(seedFirst)), MINUS);
    Q.push_back(seedFirst);

    int guard = 0;
    const int kGuardMax = 4000000;
    while (!Q.empty()) {
        if (++guard > kGuardMax) {
            std::fprintf(stderr, "[AdConquest] guard tripped at %d pops\n", guard);
            break;
        }
        MCGAL::Halfedge* h = Q.front();
        Q.pop_front();
        if (!h)
            continue;
        if (g_trace) {
            std::fprintf(stderr, "PHASE pop ok\n");
            std::fflush(stderr);
        }

        const int fflag = S.fflagOf(h->face());
        if (fflag == FCONQUERED || fflag == FTO_BE_REMOVED)
            continue;

        MCGAL::Halfedge* nh = next_(h);
        MCGAL::Vertex* vFront = target(nh);
        if (!vFront)
            continue;
        // 🔴 第 1 层分歧的**根因**（2026-09-22，一次运行内测实）：
        // 参考实现算的是 `out_degree(target(next(h)))` —— CGAL 的 out_degree
        // **每次都现算**。而 MCGAL 的 `vertex_degree()`（Vertex.cpp:8）只在
        // `degree_dirty_` 为真时才 `recompute_degree()`，否则直接返回缓存
        // `vertex_degree_`。`Mesh::split_facet` / `erase_center_vertex`
        // **都不维护**该缓存，`Vertex.h:184` 明确把维护责任交给调用方
        // （「拓扑修改点通过 incCachedDegree/decCachedDegree 或 invalidate_degree
        // 显式维护」）。我这份迁移既用了缓存、又没人替它置脏。
        //
        // 实测（sym#4008，中心点 v5671）：
        //     cached=5  dirty=0   ← vertex_degree() 返回的
        //     fresh =4            ← 强制 recompute_degree() 后的真实环长
        //     ringBounded=4  ringPrev=4  ← 两种独立环遍历也说是 4
        // ⇒ **valence 被系统性高估** ⇒ P[] 多一格、type 算错 ⇒ manifold 判据
        // 失去意义 ⇒ 放行非法删除 ⇒ 切出已存在的对角线 ⇒ reset 断言。
        // 第 0 层网格干净所以看不出来；**跨层累积**后才暴露。
        //
        // 修法：照参考实现**每次现算**（不用缓存）。这与"不改 deps/MCGAL"一致
        // —— 契约本来就要求调用方维护，我们选择不依赖该缓存。
        vFront->recompute_degree();
        const int valence = vFront->vertex_degree();

        const bool freeFront = (S.flag(vFront) == VFREE);
        const bool inRange = (valence >= 3) && (valence <= 6);
        const bool notBorder = !isBorderVertex(nh);

        if (freeFront && inRange && notBorder) {
            const int type = findType(S, h, valence);
            const bool violated = manifoldViolated(h, type, valence);
            if (!violated) {
                // —— 删除顶点 ——
                ++Number_vertices;
                if (g_trace) {
                    std::fprintf(stderr, "ENGINE_TRACE #%lld sym=%d val=%d type=%d\n", g_symIdx, valence - 3, valence, type);
                    std::fflush(stderr);
                }
                ++g_symIdx;
                out.push_back(valence - 3);
                MCGAL::Halfedge* g = h;
                // ---- P13.2 几何：CRV / barycenter / Dist（**整数域**）----
                // 参考实现（.inl:1531-1558 / 1607-1617）：
                //   CRV  = 量化(被删点位置)；BC = 量化(barycenter(被删点))
                //   Dist = CRV - BC          ← 整数残差，进 InterGeometry
                // 解码侧 CRV = BC + Dist **精确还原** ⇒ 重建位置 = dequantize(CRV)
                // ⇒ **barycenter/Frenet/range-offset 都只影响熵，不影响重建精度**。
                // 这里记录 dequantize(CRV) 与 |Dist|，用于验证
                //   ① 全量往返误差 ≈ 量化误差；② 预测确实把残差压小了。
                if (g_geo && g_bboxSpan > 0) {
                    const double step = g_bboxSpan / (double)(1 << g_qbits);
                    const double crvd[3] = {(double)vFront->x(), (double)vFront->y(), (double)vFront->z()};
                    double bcd[3];
                    barycenterOf(vFront, bcd);
                    for (int k = 0; k < 3; ++k) {
                        const int crv = Qz(crvd[k], g_bbox[k], step);
                        const int bc = Qz(bcd[k], g_bbox[k], step);
                        const long long d = (long long)crv - (long long)bc;
                        const long long ad = d < 0 ? -d : d;
                        if (ad > g_distMax[k])
                            g_distMax[k] = ad;
                        g_distSum2 += (double)d * (double)d;
                        g_reconPos.push_back(Dqz(crv, g_bbox[k], step));
                    }
                    ++g_delCount;
                }
                if (g_trace) {
                    std::fprintf(stderr, "PHASE erase_center_vertex begin\n");
                    std::fflush(stderr);
                }
                mesh->erase_center_vertex(next_(g));
                if (g_trace) {
                    std::fprintf(stderr, "PHASE erase_center_vertex end\n");
                    std::fflush(stderr);
                }
                g = h;
                if (g->face())
                    S.setFFlag(g->face(), FTEMP);
                for (int j = 0; j < valence - 1; ++j) {
                    g = next_(g);
                    if (!g)
                        break;
                    if (target(g))
                        S.setFlag(target(g), VCONQUERED);
                    if (target(opp(g)))
                        S.setFlag(target(opp(g)), VCONQUERED);
                    if (!isBorderEdge(g))
                        Q.push_back(opp(g));
                }
                // 传 h（= 参考实现里的 g，因为 .inl:1579 有一句 `g = h;` 把 g 拨回来）。
                // 我一度误改成传循环结束后的 g —— 那是我只看了调用行、没看**紧邻的
                // 重新赋值行**造成的假修正（实测把首个分歧从 #1289 打回 #11）。此处已改回。
                if (g_trace) {
                    std::fprintf(stderr, "PHASE retriangulate begin\n");
                    std::fflush(stderr);
                }
                retriangulate(mesh, S, h, valence, type);
                if (g_trace) {
                    std::fprintf(stderr, "PHASE retriangulate end (prevOnNonTri=%lld lastDeg=%d)\n", g_prevOnNonTri, g_lastNonTriDegree);
                    std::fflush(stderr);
                }
            } else {
                // —— NULL patch ——
                out.push_back(4);
                if (g_trace) {
                    std::fprintf(stderr, "ENGINE_TRACE #%lld sym=NULL(manifold) val=%d type=%d\n", g_symIdx, valence, type);
                    std::fflush(stderr);
                }
                ++g_symIdx;
                if (h->face())
                    S.setFFlag(h->face(), FCONQUERED);
                S.setFlag(vFront, VCONQUERED);
                nullPatchTail(S, h, Q);
            }
        } else {
            // 边界点分支（闭网格不进入）。符号 5/6 的实现见文档待办。
            if (g_trace) {
                std::fprintf(stderr, "ENGINE_TRACE #%lld sym=4(border) val=%d\n", g_symIdx, valence);
                std::fflush(stderr);
            }
            ++g_symIdx;
            out.push_back(4);
            if (h->face())
                S.setFFlag(h->face(), FCONQUERED);
            S.setFlag(vFront, VCONQUERED);
            nullPatchTail(S, h, Q);
        }
    }
    return Number_vertices;
}

// ---- Regulation（.inl:8890-9210）------------------------------------------
// 与 Decimation 的差异（**逐行读出**，文档 §12）：
//   · 只处理 `valence == 3` 的前端点（不是 3..6）；type 恒为 1
//   · **不做 manifold 判据** —— 参考实现里 `Manifold_condition` 那行是**注释掉的**
//     （.inl:8959-8960）。这是"清理"pass 的语义：只收割度为 3 的顶点，不插新边，
//     所以本来也不会破坏流形性。
//   · 字母表是**2 元**：0 = 标记删除，1 = null
//     （`Un_Regulation` 用 `Adaptive_Data_Model Connectivity(2)`，.inl:2291）
//   · **两段式**：Phase A 只**打标记**，真正的 `remove_center_vertex` 全部推迟到
//     Phase B，按**顶点迭代顺序**执行。这是与 Decimation 最大的结构差异。
int regulation(MCGAL::Mesh* mesh, State& S, MCGAL::Halfedge* seedFirst, std::vector<int>& out) {
    int Number_vertices = 0;
    std::deque<MCGAL::Halfedge*> Q;

    S.reset();  // == Init()（.inl:8890 后紧跟 Init(_pMesh)）
    S.setFlag(target(seedFirst), VCONQUERED);
    S.setFlag(target(opp(seedFirst)), VCONQUERED);
    Q.push_back(seedFirst);

    // ---- Phase A：遍历 + 只打标记 ----
    while (!Q.empty()) {
        MCGAL::Halfedge* h = Q.front();
        Q.pop_front();
        if (!h)
            continue;
        MCGAL::Halfedge* nh = next_(h);
        MCGAL::Vertex* vFront = target(nh);
        if (!vFront)
            continue;
        vFront->recompute_degree();  // 不信任 degree 缓存（与 Decimation 同理）
        const int valence = vFront->vertex_degree();

        const int ff = S.fflagOf(h->face());
        if (ff == FCONQUERED || ff == FTO_BE_REMOVED)
            continue;

        if (S.flag(vFront) == VFREE && valence == 3 && !isBorderVertex(nh)) {
            ++Number_vertices;
            out.push_back(0);  // 符号 0 = 删除

            // P13.2 几何：**必须在 Phase A 记录**。参考实现是在 Phase A 的
            // 删除分支里算 CRV/BC/Dist 的（.inl:9006-9071），此时网格还是
            // **本轮 Phase B 删点之前**的状态；若改到 Phase B 记录，
            // 前面几个 victim 已被删掉 ⇒ barycenter 会不一致。
            // （漏掉这一处曾导致重建缺 1695 个点，NN 假性恶化到 60 个半格。）
            if (g_geo && g_bboxSpan > 0) {
                const double step = g_bboxSpan / (double)(1 << g_qbits);
                const double crvd[3] = {(double)vFront->x(), (double)vFront->y(), (double)vFront->z()};
                double bcd[3];
                barycenterOf(vFront, bcd);
                for (int k = 0; k < 3; ++k) {
                    const int crv = Qz(crvd[k], g_bbox[k], step);
                    const int bc = Qz(bcd[k], g_bbox[k], step);
                    const long long d = (long long)crv - (long long)bc;
                    const long long ad = d < 0 ? -d : d;
                    if (ad > g_distMax[k])
                        g_distMax[k] = ad;
                    g_distSum2 += (double)d * (double)d;
                    g_reconPos.push_back(Dqz(crv, g_bbox[k], step));
                }
                ++g_delCount;
            }

            MCGAL::Halfedge* g = nh;
            S.setFlag(target(g), VREMOVED);
            S.setFFlag(g->face(), FTO_BE_REMOVED);

            g = prev_(opp(g));
            S.setFlag(target(opp(g)), VCONQUERED);
            S.setFFlag(g->face(), FTO_BE_REMOVED);
            if (!isBorderEdge(prev_(g))) {
                MCGAL::Halfedge* h1 = opp(prev_(g));
                S.setFFlag(h1->face(), FCONQUERED);
                S.setFlag(target(next_(h1)), VCONQUERED);
                if (!isBorderEdge(next_(h1)))
                    Q.push_back(opp(next_(h1)));
                if (!isBorderEdge(prev_(h1)))
                    Q.push_back(opp(prev_(h1)));
            }

            g = prev_(opp(g));
            S.setFlag(target(opp(g)), VCONQUERED);
            S.setFFlag(g->face(), FTO_BE_REMOVED);
            if (!isBorderEdge(prev_(g))) {
                MCGAL::Halfedge* h2 = opp(prev_(g));
                S.setFFlag(h2->face(), FCONQUERED);
                S.setFlag(target(next_(h2)), VCONQUERED);
                if (!isBorderEdge(next_(h2)))
                    Q.push_back(opp(next_(h2)));
                if (!isBorderEdge(prev_(h2)))
                    Q.push_back(opp(prev_(h2)));
            }
        } else {
            out.push_back(1);  // 符号 1 = null
            S.setFFlag(h->face(), FCONQUERED);
            S.setFlag(vFront, VCONQUERED);
            if (!isBorderEdge(nh))
                Q.push_back(opp(nh));
            if (!isBorderEdge(prev_(h)))
                Q.push_back(opp(prev_(h)));
        }
    }

    // ---- Phase B：按**顶点迭代顺序**删掉所有 TO_BE_REMOVED ----
    // 先快照：erase_center_vertex 可能改动 mesh->vertices()。
    std::vector<MCGAL::Vertex*> victims;
    for (MCGAL::Vertex* v : mesh->vertices())
        if (v && !v->isRemoved() && S.flag(v) == VREMOVED)
            victims.push_back(v);
    for (MCGAL::Vertex* v : victims) {
        if (v->isRemoved())
            continue;
        // 参考实现用 `halfedge(vh)` —— CGAL 语义是"target == v 的一条半边"。
        // MCGAL 的 `boundedRing(v)` 给出边，取第一条的 opposite 即为入边。
        MCGAL::Halfedge* in = nullptr;
        for (MCGAL::Halfedge* o : pmsf::boundedRing(v)) {
            in = opp(o);
            break;
        }
        if (in && target(in) == v)
            mesh->erase_center_vertex(in);
    }
    return Number_vertices;
}

int livingVertices(MCGAL::Mesh* mesh) {
    int n = 0;
    for (MCGAL::Vertex* v : mesh->vertices())
        if (v && !v->isRemoved())
            ++n;
    return n;
}

// ---- 网格健康度自检（**独立于符号流**的判据）------------------------------
// 为什么需要它：符号流**只编码 valence**（`valence-3` / null 的 4），
// **不编码 `type`**，更不编码新边的具体取位。所以 `Retriangulation` 的切分
// 导航即使错了，第 0 层的符号序列也**可以完全相等** —— 那样"层 0 全等"
// 就是**弱相等**，第 1 层会立刻分岔（当前正是这个症状）。
//
// 闭三角网格有两条**不依赖 oracle**的硬约束：
//   ① 每个面都是三角形（facet_degree == 3）；
//   ② V - E + F = 2（Euler，亏格 0）。等价地 F = 2V - 4、E = 3V - 6。
// 只要切分导航错了，①或②**必有一条被破坏**。这比对着 oracle 猜快得多。
struct MeshHealth {
    int V = 0, E = 0, F = 0;
    int nonTriFaces = 0;
    int euler() const {
        return V - E + F;
    }
};

MeshHealth meshHealth(MCGAL::Mesh* mesh) {
    MeshHealth h;
    for (MCGAL::Vertex* v : mesh->vertices())
        if (v && !v->isRemoved())
            ++h.V;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved())
            continue;
        ++h.F;
        if (f->facet_degree() != 3)
            ++h.nonTriFaces;
    }
    // 每条**无向边**数一次。MCGAL 没有全局 halfedge 迭代器，故从顶点环数：
    // 每个顶点的出边环里，只当归一化的 (min,max) 未被计入时才算。
    // 用 poolId 归一化（两端必须是同一个对象的两端）。
    std::set<std::pair<int, int>> seen;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved())
            continue;
        for (MCGAL::Halfedge* o : pmsf::boundedRing(v)) {
            MCGAL::Vertex* w = target(o);
            if (!w)
                continue;
            int a = v->poolId(), b = w->poolId();
            if (a > b)
                std::swap(a, b);
            seen.insert({a, b});
        }
    }
    h.E = (int)seen.size();
    return h;
}

// 中止时也能拿到**已经产出的**符号（否则断言一触发就什么都看不到）。
// 这是本轮教训的直接产物：oracle 侧的 dump 曾经"补丁没打上但脚本返回成功"，
// 这一侧则是"断言一触发，文件都没写"，两种都让中间结果白白丢掉。
struct DumpHook {
    static std::vector<int>* syms;
    static const char* path;
    static void write() {
        if (!syms || !path)
            return;
        FILE* fp = std::fopen(path, "w");
        if (!fp)
            return;
        std::fprintf(fp, "symbols");
        for (int v : *syms)
            std::fprintf(fp, " %d", v);
        std::fprintf(fp, "\n");
        std::fclose(fp);
    }
    static void onAbort(int) {
        write();
        _exit(134);
    }
};
std::vector<int>* DumpHook::syms = nullptr;
const char* DumpHook::path = nullptr;

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr,
                     "用法: %s <mesh.off> <out_symbols.txt> [seed_target_vertex_id]\n"
                     "  seed_target_vertex_id = 种子半边的 end_vertex 的 poolId。\n"
                     "  不给则取第 0 个面的最后一条半边（**与 oracle 未必一致**，\n"
                     "  见 docs/refactor/P13_1_ORACLE_BASELINE_2026-09-22.md 第二号移植风险）。\n",
                     argv[0]);
        return 1;
    }
    // 必须在建 Mesh **之前**做进程级运行时初始化（幂等）。
    // 不做就段错误 —— 池环境变量**不足以**替代它（踩过：裸链接探针 segfault）。
    pmsf::ensureMeshRuntimeInitialized();

    auto mesh = std::make_shared<MCGAL::Mesh>(MCGAL::Mesh(std::string(argv[1])));

    // 种子：oracle 实测为"文件第 0 个面 circulator 的第三条半边"（target = 该面第 0 个顶点）。
    // 见文档「第二号移植风险」。这里取第 0 个面的最后一条半边。
    MCGAL::Facet* f0 = nullptr;
    for (MCGAL::Facet* f : mesh->faces())
    // 种子：**显式优先**。oracle 的种子是"第 0 个面 circulator 的第三条半边"，
    // 但那依赖库的表示顺序 —— 实测 MCGAL 与 CGAL **不一致**（MCGAL 给出
    // v6036->v6041，oracle 是 v5844->v6041）。故允许显式指定 end_vertex 的 poolId。
    // ---- 种子：**坐标优先**（与 oracle 的 PMSF_MEPP2_ORACLE_SEED dump 直接对接）----
    // 坐标是**表示顺序无关**的，这正是文档里「第二号移植风险」的正解：
    // MCGAL 与 CGAL 的顶点/半边编号顺序不同（实测 MCGAL v6036->v6041，
    // oracle v5844->v6041）⇒ 用编号当种子永远对不齐，用坐标才对得齐。
    //   PMSF_ADC_SEED_A = oracle 的 seed_a = 坐标(target(First_halfedge))
    //   PMSF_ADC_SEED_B = oracle 的 seed_b = 坐标(target(opposite(First_halfedge)))
    // MCGAL 的 Point 是 **float**（Point.h:37），故按**最近顶点**匹配，不要求逐位相等。
    auto parseXYZ = [](const char* s, double o[3]) -> bool { return s && std::sscanf(s, "%lf %lf %lf", &o[0], &o[1], &o[2]) == 3; };
    MCGAL::Halfedge* seed = nullptr;
    if (argc >= 4) {
        const int want = std::atoi(argv[3]);
        for (MCGAL::Halfedge* h : *f0)
            if (h->end_vertex() && h->end_vertex()->poolId() == want) {
                seed = h;
                break;
            }
        if (!seed) {
            std::fprintf(stderr, "[AdConquest] seed vertex %d not in face0 ring\n", want);
            return 1;
        }
    } else {
        for (MCGAL::Halfedge* h : *f0)
            seed = h;  // 兜底：最后一条
    }
    if (!seed) {
        std::fprintf(stderr, "no seed\n");
        return 1;
    }

    // 前端点 = target(next(seed))，**它的 valence 决定第一个符号**。
    // 打出来是为了能直接与 oracle 的 ORACLE_TRACE 首行对齐。
    {
        MCGAL::Halfedge* nh = seed->next();
        MCGAL::Vertex* vf = nh ? nh->end_vertex() : nullptr;
        std::fprintf(stderr, "[AdConquest] seed: v%d -> v%d   front=v%d valence=%d\n", seed->vertex() ? seed->vertex()->poolId() : -1,
                     seed->end_vertex() ? seed->end_vertex()->poolId() : -1, vf ? vf->poolId() : -1, vf ? vf->vertex_degree() : -1);
    }

    g_trace = (std::getenv("PMSF_ADC_DBG") != nullptr);
    g_geo = (std::getenv("PMSF_ADC_GEO") != nullptr);
    if (const char* qb = std::getenv("PMSF_ADC_QBITS"))
        g_qbits = std::atoi(qb);
    if (const char* mv = std::getenv("PMSF_ADC_MAXV"))
        g_maxV = std::atoi(mv);
    State S;
    std::vector<int> syms;
    DumpHook::syms = &syms;
    DumpHook::path = argv[2];
    std::signal(SIGABRT, DumpHook::onAbort);
    std::signal(SIGSEGV, DumpHook::onAbort);
    // 量化用的包围盒必须在**原始输入网格**上算（参考实现 .inl:575-589
    // 在 NumberComponents 循环里算一次）。在 decimation 之前抓一份。
    {
        float mn[3] = {1e30f, 1e30f, 1e30f}, mx3[3] = {-1e30f, -1e30f, -1e30f};
        for (MCGAL::Vertex* v : mesh->vertices()) {
            if (!v || v->isRemoved())
                continue;
            const float c[3] = {v->x(), v->y(), v->z()};
            for (int k = 0; k < 3; ++k) {
                mn[k] = std::min(mn[k], c[k]);
                mx3[k] = std::max(mx3[k], c[k]);
            }
        }
        for (int k = 0; k < 3; ++k) {
            g_bbox[k] = mn[k];
            g_bbox[3 + k] = mx3[k];
        }
        // 参考实现用 float 相减得 span，这里保持一致
        g_bboxSpan = std::max({(double)(mx3[0] - mn[0]), (double)(mx3[1] - mn[1]), (double)(mx3[2] - mn[2])});
        // ⚠️ 必须打**全精度**：我用 %.6g 打出来、再在对比脚本里硬编码，
        // 结果格点错开整整一个 step，两组点**完全不相交**（NN = 2.000 半格）。
        // 那是**对比脚本的错**、不是引擎的错 —— 仪表又骗了我一次。
        std::fprintf(stderr, "[AdConquest] bbox(orig)=x[%.17g,%.17g] y[%.17g,%.17g] z[%.17g,%.17g] span=%.17g\n", (double)g_bbox[0],
                     (double)g_bbox[3], (double)g_bbox[1], (double)g_bbox[4], (double)g_bbox[2], (double)g_bbox[5], g_bboxSpan);
    }
    int prevV = livingVertices(mesh.get());
    const int chi0 = meshHealth(mesh.get()).euler();  // 初始 Euler 特征（开网格不是 2）
    std::fprintf(stderr, "[AdConquest] V0=%d chi0=%d\n", prevV, chi0);
    // 外层循环 = 参考实现的 `Simplification`（.inl:3797）：
    //   do { Decimation_Conquest(); Regulation(); } while (顶点数还在变)
    // 两者各自发射符号、各自记账 ⇒ oracle dump 的"层"是 **Decimation 与 Regulation 交替**。
    // （层 0=Decimation#0, 层 1=Regulation#0, 层 2=Decimation#1, …）
    for (int round = 0; round < 200; ++round) {
        const int removedDec = decimationConquest(mesh.get(), S, seed, syms);
        const int removedReg = regulation(mesh.get(), S, seed, syms);
        const int removed = removedDec + removedReg;
        const int nowV = livingVertices(mesh.get());
        // 网格健康度（**独立于符号流**的判据，见 meshHealth 的说明）。
        // 符号流只编码 valence、不编码 type 与新边取位 ⇒ Retriangulation 错位
        // 时符号序列**可以全等**，但 V-E+F 或"全三角"**必有一条被破坏**。
        const MeshHealth mh = meshHealth(mesh.get());
        // ⚠️ 不是 `euler() == 2`：那只对**闭**网格成立。开网格（带边界）的 χ 不是 2
        // （实测删掉一个面的 cow_open 是 χ=1）；写成 2 会把正确结果误报成 BROKEN
        // —— 我就这么误报过一次。正确的不变量是 **χ 保持不变**（删一个顶点时
        // 它的扇面同步消失，V 与 F 同步变化，χ 守恒）。
        const bool okEuler = (mh.euler() == chi0);
        const bool okTri = (mh.nonTriFaces == 0);
        std::fprintf(stderr,
                     "[AdConquest] round %d: removed=%d V=%d syms=%zu | V=%d E=%d F=%d "
                     "euler=%d(%s) nonTri=%d%s\n",
                     round, removed, nowV, syms.size(), mh.V, mh.E, mh.F, mh.euler(), okEuler ? "== chi0 OK" : "**CHANGED**", mh.nonTriFaces,
                     okTri ? "" : "  **NON-TRI**");
        if (!okEuler || !okTri)
            std::fprintf(stderr,
                         "[AdConquest] ^^^ 网格健康度在第 %d 轮后**已被破坏**（chi0=%d）⇒ "
                         "Retriangulation 的切分导航与参考不一致"
                         "（符号流看不出来，因为它不编码 type 与新边取位）\n",
                         round, chi0);
        // 复刻参考实现的停止规则 `if(Current_Number < NVertices) break;`
        // （Simplification, .inl:3860 附近）。有了它才能让引擎停在 oracle 的
        // **同一运行点**，进而比对几何 —— P13.2 的对拍前提。
        if (g_maxV > 0 && nowV < g_maxV)
            break;
        if (nowV == prevV)
            break;
        prevV = nowV;
    }

    FILE* fp = std::fopen(argv[2], "w");
    if (!fp) {
        std::fprintf(stderr, "cannot write %s\n", argv[2]);
        return 1;
    }
    std::fprintf(fp, "symbols");
    for (int s : syms)
        std::fprintf(fp, " %d", s);
    std::fprintf(fp, "\n");
    std::fclose(fp);
    std::fprintf(stderr, "[AdConquest] wrote %zu symbols -> %s\n", syms.size(), argv[2]);
    std::fprintf(stderr, "[AdConquest] illegal splits pre-detected = %lld\n", g_illegalSplits);

    // P13.2 对拍前提：把**存活顶点坐标**导出，与 oracle 的 out.obj 逐点比。
    // AD 只**删**点、不移动存活点 ⇒ 两边存活点的坐标应当一致
    // （注意 MCGAL 的 Point 是 float、CGAL 是 double，故按容差比，不逐位比）。
    // ---- P13.2 第一步：量化 → 反量化（规格见文档 §17，逐行读自参考实现）----
    // 只做量化，**不做** barycenter 预测、**不加**残差 ⇒ 结果不会等于 oracle 的
    // out.obj（那边还叠加了预测残差），但给出一个**上界**：
    // 若偏差与量化半格同量级，说明量化主导；若显著更小，说明预测贡献可观测。
    // 步长是**各向同性**的（包围盒最长边 / 2^Qbit），不是逐轴量化。
    // ---- P13.2 全量重建：被删点用 dequantize(CRV)，存活点用量化后的坐标 ----
    // 解码器重建出的就是这一组点（存活点从未被预测，被删点 = dequantize(BC+Dist)
    // = dequantize(CRV)）。与**原始网格**逐点比即可验证几何通路。
    if (g_geo && g_bboxSpan > 0) {
        const double step = g_bboxSpan / (double)(1 << g_qbits);
        std::fprintf(stderr,
                     "[AdConquest] geo: deleted=%lld distinct=%d |dist|max=(%lld,%lld,%lld) "
                     "rms=%.4f (quantized units) step=%.6g\n",
                     g_delCount, (int)(g_reconPos.size() / 3), g_distMax[0], g_distMax[1], g_distMax[2],
                     g_delCount > 0 ? std::sqrt(g_distSum2 / (double)(g_delCount * 3)) : 0.0, step);
        if (const char* rp = std::getenv("PMSF_ADC_RECON")) {
            FILE* rf = std::fopen(rp, "w");
            if (rf) {
                for (size_t i = 0; i + 2 < g_reconPos.size(); i += 3)
                    std::fprintf(rf, "%.17g %.17g %.17g\n", g_reconPos[i], g_reconPos[i + 1], g_reconPos[i + 2]);
                // 存活点：解码器把它们放在**量化后**的位置
                for (MCGAL::Vertex* v : mesh->vertices()) {
                    if (!v || v->isRemoved())
                        continue;
                    std::fprintf(rf, "%.17g %.17g %.17g\n", Dqz(Qz(v->x(), g_bbox[0], step), g_bbox[0], step),
                                 Dqz(Qz(v->y(), g_bbox[1], step), g_bbox[1], step), Dqz(Qz(v->z(), g_bbox[2], step), g_bbox[2], step));
                }
                std::fclose(rf);
            }
        }
    }
    return 0;
}
