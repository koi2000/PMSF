#pragma once

#include "Halfedge.h"
#include "common/BoundedRingTraversal.h"

#include <array>
#include "Point.h"
#include "Vertex.h"
#include "operator/IsRemovableOperator.h"

namespace {
/// @brief 参考 WVM 变体开关（进程内不变，缓存一次；原先每次候选都 getenv）。
inline bool ppmcWvmRefEnabled() {
    static const bool enabled = [] {
        const char* e = std::getenv("PMSF_PPMC_WVM_REF");
        return e && e[0] == '1';
    }();
    return enabled;
}
}  // namespace

class PPMCIsRemovableOperator : public IsRemovableOperator {
  public:
    PPMCIsRemovableOperator() = default;

    void init(std::vector<MCGAL::Vertex*> v, std::vector<MCGAL::Halfedge*> h, std::vector<MCGAL::Facet*> f) {
        this->seeds = h;
    };

    /// @brief `boundedRing(v)` 的快速等价：只需判断"v 的 1-ring 里是否有指向 target
    /// 的 outgoing 半边"。逐位语义等价（同一个 ring 集合），但省掉 BoundedRingIterator
    /// 状态机 + boundedFindPrev（非三角面时 O(deg)）——perf 实测 chord/WVM 的 ring
    /// 走查是 select 阶段的主要 self 时间。坏环用 guard 上限兜底（正常闭合网格不触发）。
    static bool ringHasTarget(MCGAL::Vertex* v, MCGAL::Vertex* target) {
        if (!v || !target) return false;
        MCGAL::Halfedge* h = v->proxyHalfedge();
        if (!h) return false;
        MCGAL::Halfedge* const start = h;
        int guard = 0;
        do {
            if (h->end_vertex() == target) return true;
            MCGAL::Halfedge* opp = h->opposite();
            if (!opp) return false;
            h = opp->next();
            if (!h) return false;
            if (++guard > 4096) return false;
        } while (h != start);
        return false;
    }

    /**
     * Test if a vertex removal will violate the manifold property of a mesh.
     * \return true if it will else false.
     *
     * Legacy Peng & Kuo 2005 1-ring-self-intersection check (opposite-edge
     * revisit inside the same 1-ring) was replaced by the current
     * face-sharing test below —— 它更宽松，匹配 PPMC 论文的 Maglo et al. 2013 实现。
     */
    bool willViolateManifold(MCGAL::Halfedge* const* polygon, int i_degree) const {
        // 参考实现 mymeshCompTests.cpp:27-59 的忠实移植（A/B：PMSF_PPMC_WVM_REF=1）。
        //
        // polygon[i] 是**从中心 v 出发**的半边 v->u_i（heh_oneRing 的约定）。
        // 对每个 u_i 遍历其邻居 w：若 w 也是 patch 顶点 u_j，则检查
        //    j == i_prev 且 face(polygon[i]) 非三角 → 顶点切除会破坏流形；
        //    j == i_next 且 face(polygon[i]->opposite()) 非三角 → 同上。
        // perf：该函数在 select 热路径上每候选调用一次，getenv 曾是可见开销；
        // 开关进程内不变，缓存一次（语义：env 值首字符为 '1' 才启用参考 WVM）。
        static const bool wvmRef = [] {
            const char* refEnv = std::getenv("PMSF_PPMC_WVM_REF");
            return refEnv && refEnv[0] == '1';
        }();
        if (!wvmRef) {
            // 旧版保守启发式（默认，保持既有字节/行为）。
            for (int i = 0; i < i_degree; ++i) {
                MCGAL::Halfedge* it = polygon[i];
                int deg = it->face()->facet_degree();
                if (deg == 3) continue;
                for (int j = 0; j < i_degree; j++) {
                    MCGAL::Halfedge* jt = polygon[j];
                    if (i == j) continue;
                    if (it->face() == jt->opposite()->face() || it->face() == jt->face()) {
                        if (ringHasTarget(it->end_vertex(), jt->end_vertex())) return true;
                    }
                }
            }
            return false;
        }

        const unsigned n = static_cast<unsigned>(i_degree);
        for (unsigned i = 0; i < n; ++i) {
            MCGAL::Halfedge* pi = polygon[i];
            if (!pi || !pi->end_vertex()) continue;
            MCGAL::Vertex* ui = pi->end_vertex();
            const unsigned i_prev = (i == 0) ? n - 1 : i - 1;
            const unsigned i_next = (i == n - 1) ? 0 : i + 1;
            // 遍历 u_i 的邻居 w（= u_i 出发半边的终点）。
            for (MCGAL::Halfedge* hh : pmsf::boundedRing(ui)) {
                if (!hh || !hh->end_vertex()) continue;
                MCGAL::Vertex* w = hh->end_vertex();
                for (unsigned j = 0; j < n; ++j) {
                    if (!polygon[j] || polygon[j]->end_vertex() != w) continue;
                    // 方向修正：MCGAL 的环迭代方向与 CGAL Halfedge_around_vertex_circulator
                    // 相反（MCGAL: find_prev()->opposite()；CGAL: next()->opposite()），
                    // 因此参考实现里的 i_prev / i_next 配对需要互换。
                    if ((j == i_next && pi->face()->facet_degree() != 3) ||
                        (j == i_prev && pi->opposite()->face()->facet_degree() != 3)) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    /**
     * Pre-flight check for the chord that PPMCEliminateOperator::eliminate
     * will introduce via Mesh::split_facet(h, hSplit) on a non-triangular
     * 1-ring face. Returns true when the chord endpoints are already
     * connected by an existing edge — in which case calling split_facet
     * would duplicate a halfedge and trip MCGAL::Halfedge::reset's
     * `h->opposite() == NULL` assertion.
     *
     * This is the pre-check the legacy willViolateManifold implementation
     * was missing: it only verified same-face neighbors in v's 1-ring,
     * whereas the real failure mode is an edge that connects the chord
     * endpoints through a face outside v's 1-ring.
     */
    bool chordWouldDuplicateEdge(MCGAL::Halfedge* const* polygon, int i_degree) const {
        for (int pi = 0; pi < i_degree; ++pi) {
            MCGAL::Halfedge* h = polygon[pi];
            MCGAL::Facet* f = h->face();
            if (f == nullptr) continue;
            if (f->facet_degree() <= 3) continue;  // eliminate() will not split
            // Mirror the chord-target search inside PPMCEliminateOperator::eliminate:
            //   hSplit starts at h->next(); advance while hSplit->next()->next() != h.
            MCGAL::Halfedge* hSplit = h->next();
            int guard = 0;
            while (hSplit && hSplit->next() && hSplit->next()->next() != h) {
                hSplit = hSplit->next();
                if (++guard > 64) { hSplit = nullptr; break; }  // malformed face
            }
            if (!hSplit) return true;                         // conservative
            MCGAL::Vertex* a = h->end_vertex();
            MCGAL::Vertex* b = hSplit->end_vertex();
            if (a == nullptr || b == nullptr || a == b) return true;
            // Existing edge a<->b would cause duplicate-halfedge assertion.
            // NOTE: 不能依赖 Vertex::halfedges()（历史遗留的存量 vector，在
            // split_facet / erase_center_vertex 之后会过期 → 漏检 → 造出 3 面共享一条边
            // 的非流形拓扑，后续轮次表现为"两条 1-ring 面都想切同一条弦"）。
            // 这里改用与真实拓扑一致的 1-ring 走查。
            if (ringHasTarget(a, b)) return true;
            if (ringHasTarget(b, a)) return true;
        }
        return false;
    }

    bool isRemovable(HalfedgeErrorEntry& entry) {
        if (entry.edge == nullptr || entry.edge->opposite() == nullptr) {
            return false;
        }
        if (entry.edge->vertex() == nullptr || entry.edge->end_vertex() == nullptr ||
            entry.edge->vertex()->isBoundary() || entry.edge->end_vertex()->isBoundary()) {
            return false;
        }
        MCGAL::Vertex* v = entry.edge->end_vertex();
        // PPMC 论文（Maglo et al. 2013, Peng & Kuo 2005）要求 vertex_degree > 2 保证
        // erase_center_vertex 后仍为合法多边形；上界选 16 以支持高阶顶点（bunny/armadillo 上
        // 会有 degree 12~13 的"极点"，过去限 < 8 会漏掉约 30% 的可消元顶点）。
        // 注：16 已覆盖 Stanford/Thingi10K 常见 manifold 网格的 P99 degree，更高会显著
        // 拖慢 willViolateManifold 的 O(n^2) 检查。
        const int deg = v->vertex_degree();
        // perf（2026-11-08 perf record 实测：isRemovable 7.5% cycles；其中两个
        // vector 的堆分配是 malloc/free ~7.5% 的主要来源）。1-ring 用栈数组收集：
        // deg 上界 16 已由条件保证；异常环（超过 16）保守拒绝。
        if (!v->isConquered() && deg > 2 && deg <= 16) {
            std::array<MCGAL::Halfedge*, 16> heh_oneRing;
            int ringN = 0;
            bool ringOk = false;
            // perf（2026-11-08 bunny 实测 isRemovable 7.4% cycles）：默认旧版 WVM 是
            // 顺序无关的两两判据，chord 检查也是布尔存在性 => 1-ring 可以用
            // O(1) 步进 h = h->opposite()->next() 收集（集合相同、顺序相反），
            // 省掉 BoundedRingIterator 状态机 + boundedFindPrev（非三角面 O(deg)）。
            // 环异常/未在 64 步内闭合时退回 boundedRing；参考 WVM 变体依赖环序，也走旧路。
            if (!ppmcWvmRefEnabled()) {
                MCGAL::Halfedge* h = v->proxyHalfedge();
                if (h) {
                    MCGAL::Halfedge* const start = h;
                    int guard = 0;
                    ringOk = true;
                    do {
                        if (h->opposite() == nullptr || h->face() == nullptr ||
                            h->opposite()->face() == nullptr) {
                            return false;
                        }
                        if (ringN >= static_cast<int>(heh_oneRing.size())) return false;
                        heh_oneRing[static_cast<std::size_t>(ringN++)] = h;
                        MCGAL::Halfedge* opp = h->opposite();
                        h = opp->next();
                        if (h == nullptr || ++guard > 64) {
                            ringOk = false;
                            break;
                        }
                    } while (h != start);
                }
            }
            if (!ringOk) {
                ringN = 0;
                for (MCGAL::Halfedge* hit : pmsf::boundedRing(v)) {
                    if (hit == nullptr || hit->opposite() == nullptr || hit->face() == nullptr ||
                        hit->opposite()->face() == nullptr) {
                        return false;
                    }
                    if (ringN >= static_cast<int>(heh_oneRing.size())) return false;
                    heh_oneRing[static_cast<std::size_t>(ringN++)] = hit;
                }
            }

            // Legacy 参考：isConvex / removalError 判据在 mymesh 原版里曾被用来
            // 收紧可删除顶点集；我们沿用 `!willViolateManifold` 一项即可匹配论文行为。
            bool b_ret = !willViolateManifold(heh_oneRing.data(), ringN);

            // P0 fix (2026-05-01): reject vertices whose removal would make
            // PPMCEliminateOperator::eliminate call split_facet with a chord
            // that duplicates an existing edge — the root cause of the
            // `h->opposite() == NULL` abort reported on Stanford bunny.
            // willViolateManifold only checks 1-ring-internal conflicts;
            // external conflicts (chord endpoints connected through a face
            // outside the 1-ring) slip through and crash the pool allocator.
            if (b_ret && chordWouldDuplicateEdge(heh_oneRing.data(), ringN)) {
                return false;
            }
            return b_ret;
        }
        return false;
    }

    std::vector<MCGAL::Halfedge*> seeds;
};