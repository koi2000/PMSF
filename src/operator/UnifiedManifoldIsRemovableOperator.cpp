/*
 * @Author: koi
 * @Date: 2025-08-29 11:12:36
 * @Description: Complete manifold-preserving removability check.
 *
 * Complexity: O(n × k) where n = polygon boundary size, k = average vertex degree.
 * Previous implementation was O(n² × k) due to brute-force pair enumeration.
 *
 * Checks performed:
 *   1. Vertex uniqueness — a duplicate vertex in the boundary means a pinched
 *      (non-manifold) one-ring.
 *   2. Link condition (chord-free) — an edge between two non-adjacent boundary
 *      vertices would become a non-manifold edge after removal, because the
 *      resulting polygon would need that edge for triangulation while it already
 *      belongs to an existing face.
 */
#include "UnifiedManifoldIsRemovableOperator.h"
#include "common/BoundedRingTraversal.h"
#include "common/FastRingWalk.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Point.h"
#include "Vertex.h"
#include <cstring>
#include <unordered_set>
#include <vector>

namespace {

bool touchesTopologyBoundary(MCGAL::Vertex* v) {
    if (!v) return false;
    // perf：该检查每个候选都跑；先 O(1) 环走，异常环回退 boundedRing。
    // 语义：环上任一半边无 opposite，或其在面内的前驱无 opposite 则视为边界。
    auto pred = [](MCGAL::Halfedge* h) -> bool {
        if (h->opposite() == nullptr) return false;
        MCGAL::Halfedge* prev = h->find_prev();
        if (prev && !prev->opposite()) return false;
        return true;
    };
    using pmsf::RingWalkResult;
    const RingWalkResult r = pmsf::walkOutgoingRing(v, pred);
    if (r == RingWalkResult::EarlyStop) return true;
    if (r == RingWalkResult::Broken) {
        for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
            if (!h) continue;
            if (!h->opposite()) return true;
            MCGAL::Halfedge* prev = h->find_prev();
            if (prev && !prev->opposite()) return true;
        }
        return false;
    }
    return false;
}
}

// 公开 API 让 conquest 主循环 dump 内部统计

bool UnifiedManifoldIsRemovableOperator::isRemovable(HalfedgeErrorEntry& entry) {
    MCGAL::Halfedge* h = entry.edge;
    MCGAL::Vertex* v = h->end_vertex();
    // 是否放行「本轮新增的」半边 —— 由算法通过 SelectionPolicy 声明，
    // 框架不认识任何算法（见 core/SelectionPolicy.h）。
    const bool allowAdded = policy_.allowAddedHalfedges;

    // 快速度数剪枝：polyVertices 的规模 ~= degree(v)。
    // 若 degree < 3 或 >= 17 直接拒绝，避免走后续 O(deg * k) 逻辑。
    {
        int deg = v->is_degree_dirty() ? v->vertex_degree() : v->vertex_degree_cached();
        if (deg < 3 || deg >= 17) {
            return false;
        }
    }

    // perf：两个 1-ring 扫描只做 OR 判定；先 O(1) 环走，异常环回退。
    auto addedPred = [&](MCGAL::Halfedge* hit) -> bool {
        if (!allowAdded && hit->isAdded()) return false;
        return true;
    };
    using pmsf::RingWalkResult;
    auto runRingCheck = [&](MCGAL::Vertex* rv) -> bool {
        const RingWalkResult rw = pmsf::walkOutgoingRing(rv, addedPred);
        if (rw == RingWalkResult::EarlyStop) return false;
        if (rw == RingWalkResult::Broken) {
            for (MCGAL::Halfedge* hit : *rv) {
                if (!addedPred(hit)) return false;
            }
        }
        return true;
    };
    if (!runRingCheck(h->vertex())) {
        return false;
    }
    if (pmsf::walkOutgoingRing(v, addedPred) == RingWalkResult::EarlyStop) {
        return false;
    }

    bool isBoundary = v->isBoundary() || touchesTopologyBoundary(v);
    if (isBoundary) {
        v->setBoundary();
    }
    MCGAL::Vertex* polyVertices[17];
    unsigned polySize = 0;
    auto appendPolyVertex = [&](MCGAL::Vertex* pv) -> bool {
        if (polySize >= 17) return false;
        polyVertices[polySize++] = pv;
        return true;
    };
    auto rejectPolySize = [&]() {
        return false;
    };

    if (isBoundary) {
        // Walk to the boundary-start face.
        // st ends up as a halfedge ?→v in the last face (CW direction) before boundary.
        MCGAL::Halfedge* st = h;
        int guard = 0;
        while (st && st->next() && st->next()->opposite() != nullptr) {
            st = st->next()->opposite();
            if (++guard > 2048) return false;
        }
        if (!st || !st->next()) return false;
        st = st->find_prev();
        if (!st) return false;

        if (!appendPolyVertex(st->vertex())) return rejectPolySize();
        while (true) {
            if (!st || !st->face()) return false;
            if ((!allowAdded && st->isAdded()) || st->face()->isSplittable()) {
                return false;
            }
            if (!appendPolyVertex(st->end_vertex())) return rejectPolySize();
            if (!st->next() || st->next()->opposite() == nullptr) {
                break;
            }
            st = st->next()->opposite()->next();
            if (++guard > 4096) return false;
        }
    } else {
        // Interior vertex — general approach that handles any face degree.
        // For each outgoing halfedge from v, walk the face boundary collecting
        // vertices that are NOT v.  This naturally supports triangles, quads, etc.
        auto collectFromOut = [&](MCGAL::Halfedge* outH) -> bool {
            MCGAL::Halfedge* edge = outH->next();
            while (edge->end_vertex() != v) {
                if (!allowAdded && edge->isAdded()) return false;
                if (!appendPolyVertex(edge->vertex())) return false;
                edge = edge->next();
            }
            return true;
        };
        polySize = 0;
        const RingWalkResult rw = pmsf::walkOutgoingRing(
            v, [&](MCGAL::Halfedge* outH) -> bool { return collectFromOut(outH); });
        if (rw == RingWalkResult::EarlyStop) {
            if (polySize >= 17) return rejectPolySize();
            return false;
        }
        if (rw == RingWalkResult::Broken) {
            polySize = 0;
            for (MCGAL::Halfedge* outH : pmsf::boundedRing(v)) {
                if (!collectFromOut(outH)) {
                    if (polySize >= 17) return rejectPolySize();
                    return false;
                }
            }
        }
    }

    const unsigned n = polySize;
    // 原实现 n >= 8 过严，会遗漏大量 degree 8~16 的合法内部顶点。
    // 放宽到 17：覆盖 Stanford/Thingi10K/armadillo 上 P99 顶点度数，
    // 与几篇原始论文的标准 (<= 16) 对齐。超过则 checkManifoldViolation 的
    // O(n × k) 检查开销开始显著，且这类极点多属于退化 mesh。
    if (n < 3 || n >= 17) {
        return false;
    }

    bool violation = checkManifoldViolationRaw(v, polyVertices, n, !isBoundary);
    return !violation;
}

bool UnifiedManifoldIsRemovableOperator::checkManifoldViolation(MCGAL::Vertex* center,
                                                                const std::vector<MCGAL::Vertex*>& poly,
                                                                bool isClosed) {
    return checkManifoldViolationRaw(center, poly.data(),
                                     static_cast<unsigned>(poly.size()), isClosed);
}

bool UnifiedManifoldIsRemovableOperator::checkManifoldViolationRaw(MCGAL::Vertex* center,
                                                                   MCGAL::Vertex* const* poly,
                                                                   unsigned n,
                                                                   bool isClosed) {
    if (n < 3)
        return true;

    // ---- Check 1: vertex uniqueness ----
    // A duplicate means the one-ring is pinched at that vertex, which is
    // already non-manifold or would become so after vertex removal.
    // 对于 n <= 16 的小集合，线性扫描比 unordered_set 快 (节省 hash 与 alloc)。
    constexpr unsigned SMALL_N = 17;
    MCGAL::Vertex* smallSet[SMALL_N];
    for (unsigned i = 0; i < n; i++) {
        MCGAL::Vertex* p = poly[i];
        for (unsigned j = 0; j < i; j++) {
            if (smallSet[j] == p) {
                return true;
            }
        }
        smallSet[i] = p;
    }

    // perf：inSet 在 O(n * k) 环检查里是内层热点；先用 64-bit bloom 过滤，
    // 只有 bit 命中才做线性比较（bit 未命中 ⇒ 必定不在集合，精确）。
    unsigned long long polyBloom = 0ULL;
    for (unsigned i = 0; i < n; i++) {
        polyBloom |= 1ULL << (static_cast<unsigned>(poly[i]->poolId()) & 63u);
    }
    auto inSet = [&](MCGAL::Vertex* q) -> bool {
        if ((polyBloom & (1ULL << (static_cast<unsigned>(q->poolId()) & 63u))) == 0ULL) {
            return false;
        }
        for (unsigned j = 0; j < n; j++) {
            if (smallSet[j] == q) return true;
        }
        return false;
    };

    // ---- Check 2: link condition (no chord edges) ----
    // For every polygon vertex v_i, inspect its 1-ring neighbors.
    // If any neighbor w satisfies:
    //   - w != center
    //   - w is NOT adjacent to v_i in the polygon boundary
    //   - w IS a polygon vertex
    // then there is a "chord" edge that would become non-manifold.
    //
    // isClosed = true  → polygon is v_0 v_1 ... v_{n-1} v_0 (cycle)
    // isClosed = false → polygon is v_0 v_1 ... v_{n-1}      (path)
    for (unsigned i = 0; i < n; i++) {
        MCGAL::Vertex* vi = poly[i];

        MCGAL::Vertex* v_prev = nullptr;
        MCGAL::Vertex* v_next = nullptr;
        if (isClosed) {
            v_prev = poly[(i + n - 1) % n];
            v_next = poly[(i + 1) % n];
        } else {
            if (i > 0)
                v_prev = poly[i - 1];
            if (i < n - 1)
                v_next = poly[i + 1];
        }

        auto pred = [&](MCGAL::Halfedge* he) -> bool {
            MCGAL::Vertex* w = he->end_vertex();
            if (w == center) return true;
            if (w == v_prev || w == v_next) return true;
            return !inSet(w);
        };
        using pmsf::RingWalkResult;
        const RingWalkResult rw = pmsf::walkOutgoingRing(vi, pred);
        if (rw == RingWalkResult::EarlyStop) return true;
        if (rw == RingWalkResult::Broken) {
            for (MCGAL::Halfedge* he : pmsf::boundedRing(vi)) {
                MCGAL::Vertex* w = he->end_vertex();
                if (w == center) continue;
                if (w == v_prev || w == v_next) continue;
                if (inSet(w)) return true;
            }
        }
    }

    return false;
}
