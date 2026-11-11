// SPDX-License-Identifier: Apache-2.0
//
// PCATMPaperEliminateOperator.cpp — see header for design contract.
//
// 实现路线（M1-b Step 2a）：
//
//   eliminate(entry):
//     1. adjustment（mirror 父类 PCATMZColor）
//     2. remove_point（mirror 父类，得到 N-gon polyFace + polyValence + residual）
//     3. normalize_proxy_to_min_position_vertex（与 decoder bit-exact 对齐前提）
//     4. setProtocolHint(20)  // CODEC_020
//     5. triangulate(h)        // ← 本类重写：真 Z-shape
//     6. postprocess（mirror 父类；no-op）
//
//   triangulate(h):
//     a. 收集 polygon ring polyVerts（顺序 = proxyHalfedge() 起 next() 走 N 圈）
//     b. 调 computeZShape(N, startColor=0, StartDir::Right) 拿对角线 + 三角形序列
//     c. 对每条 Diagonal{va, vb}：
//          h_a = polyVerts[va] 出发的 outgoing halfedge（即 polyVerts[va] → polyVerts[(va+1)%N]）
//          h_b = polyVerts[vb] 出发的 outgoing halfedge（即 polyVerts[vb] → polyVerts[(vb+1)%N]）
//        调 Mesh::split_facet(h_a, h_b) → 在 polyFace 上切出新对角线 (polyVerts[va+1] -> polyVerts[vb+1])
//        ⚠ split_facet 创建的对角线是 (h_a.end_vertex → h_b.end_vertex)
//          = (polyVerts[(va+1)%N] → polyVerts[(vb+1)%N])
//
//        但 computeZShape 的 Diagonal 语义是 "(polyVerts[va], polyVerts[vb])"：va, vb 是端点。
//        因此把 Diagonal{va, vb} 映射到 split_facet(h_a, h_b) 时，需要找
//          h_a = "入射到 polyVerts[va]" 的 halfedge → h_a.end_vertex = polyVerts[va] ✓
//          h_b = "入射到 polyVerts[vb]" 的 halfedge → h_b.end_vertex = polyVerts[vb] ✓
//        即 h_a.end_vertex = polyVerts[va] 而非 h_a.vertex = polyVerts[va]。
//
//        helper findOutgoingHalfedge(polyFace, fromVertex) 返回的是
//        "fromVertex 出发的" outgoing halfedge，i.e. h.vertex = fromVertex。
//        因此：
//          h_a_outgoing = findOutgoingHalfedge(polyFace, polyVerts[va])
//          h_a_split    = h_a_outgoing.prev()         // ← end_vertex = polyVerts[va]
//        但 polyFace 是 N-gon, halfedge 没有 prev() 方法；用 next() 走 N-1 步等价。
//        实际更简单：直接查 "入射到 polyVerts[va]" = polyVerts[(va-1+N)%N] 出发的 halfedge。
//
//   d. 给 split_facet 切出的每个 face 调 setTriangleColor(RED/BLUE) 和 setSplittable，
//      与 computeZShape 输出的 Triangle::color 严格对齐。
//      ⚠ split_facet 后哪个 face 对应哪个 Triangle 取决于切的顺序——
//        本实现按 computeZShape.diagonals 的索引顺序，每切一刀都把 "当前 split 切出的 fnew"
//        视为 next triangle（论文 zigzag 规则：每次切出一个 ear）。color 序列同步推进。
//      但更鲁棒的做法是：先全部切完所有 (N-3) 条对角线 → 得到 N-2 个三角形 face →
//        然后再扫一遍，每个 face 按其 polyVerts 顶点构成识别属于哪个 Triangle，
//        贴对应 color。
//
// 关键 invariant：每次 split_facet 后 polyFace 自身的 proxyHalfedge 可能改变，
// 但只要保证 ring 上 vertex pointer 不变（split_facet 只新增半边、不删 ring vertex），
// findOutgoingHalfedge 在新 face 上仍能正确找到 outgoing halfedge。
//
// 兜底：N <= 3 直接 setSplittable 不切，与父类 PCATMEliminate 一致。

#include "PCATMPaperEliminateOperator.h"
#include "common/BoundedRingTraversal.h"

#include "Facet.h"
#include "ContextPool.h"
#include "Global.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "MeshUtils.h"
#include "Point.h"
#include "Vertex.h"
#include "PCATM/AlternatingInvariantChecker.h"
#include "common/Define.h"
#include "PCATM/PCATMZShapeGeometry.h"
#include "operator/UnifiedManifoldIsRemovableOperator.h"
#include "predict/RDPredictor.h"  // rdBarycenterQuantized（已被父类用）

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <unordered_set>
#include <vector>

namespace {

bool isBoundaryEdge(int a, int b, int n);

bool isEnvOn(const char* name) {
    const char* v = std::getenv(name);
    return v && *v && std::strcmp(v, "0") != 0;
}

std::vector<int> paperTwoColorSequence(int numTriangles) {
    std::vector<int> colorSeq(numTriangles, 1);
    if (numTriangles <= 0) return colorSeq;

    // PCATM §4/Fig.2: the two external triangles are encoded as 0; the
    // middle Z-shaped run is encoded as 1.  Degree-3/4 patches cannot be
    // represented by this 2-color scheme; keep them all zero so strict audits
    // expose them via the low-degree histogram instead of pretending they are
    // recoverable 2-color patches.
    if (numTriangles <= 2) {
        std::fill(colorSeq.begin(), colorSeq.end(), 0);
        return colorSeq;
    }
    colorSeq.front() = 0;
    colorSeq.back() = 0;
    return colorSeq;
}

bool faceContainsVertex(MCGAL::Facet* f, MCGAL::Vertex* v) {
    if (!f || !v || f->isRemoved()) return false;
    for (MCGAL::Vertex* fv : f->getVertices()) {
        if (fv == v) return true;
    }
    return false;
}

std::vector<MCGAL::Vertex*> collectRemovalBoundaryRing(MCGAL::Halfedge* h) {
    std::vector<MCGAL::Vertex*> ring;
    if (!h || !h->vertex() || !h->next() || !h->next()->opposite()) return ring;
    ring.push_back(h->vertex());
    MCGAL::Halfedge* g = h->next()->opposite();
    int guard = 0;
    while (g && g != h) {
        if (!g->vertex()) return {};
        ring.push_back(g->vertex());
        if (!g->next() || !g->next()->opposite()) return {};
        g = g->next()->opposite();
        if (++guard > 128) return {};
    }
    return ring;
}

void rotateRingToQuantizedMin(std::vector<MCGAL::Vertex*>& ring,
                              const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (ring.empty() || !mesh) return;
    size_t best = 0;
    MCGAL::PointInt bestQ = mesh->floatPosToInt(ring[0]->point());
    for (size_t i = 1; i < ring.size(); ++i) {
        MCGAL::PointInt q = mesh->floatPosToInt(ring[i]->point());
        if (q < bestQ) {
            best = i;
            bestQ = q;
        }
    }
    if (best != 0) {
        std::rotate(ring.begin(), ring.begin() + static_cast<std::ptrdiff_t>(best), ring.end());
    }
}

MCGAL::Facet* externalFaceAcrossBoundaryEdge(MCGAL::Vertex* a,
                                             MCGAL::Vertex* b,
                                             MCGAL::Vertex* center) {
    if (!a || !b || !center) return nullptr;
    auto inspect = [&](MCGAL::Halfedge* hh) -> MCGAL::Facet* {
        if (!hh || hh->isRemoved()) return nullptr;
        MCGAL::Facet* f0 = hh->face();
        if (f0 && !f0->isRemoved() && !faceContainsVertex(f0, center)) return f0;
        MCGAL::Halfedge* opp = hh->opposite();
        MCGAL::Facet* f1 = opp ? opp->face() : nullptr;
        if (f1 && !f1->isRemoved() && !faceContainsVertex(f1, center)) return f1;
        return nullptr;
    };
    for (MCGAL::Halfedge* hh : pmsf::boundedRing(a)) {
        if (!hh || hh->isRemoved()) continue;
        if (hh->vertex() == a && hh->end_vertex() == b) {
            if (MCGAL::Facet* f = inspect(hh)) return f;
        }
    }
    for (MCGAL::Halfedge* hh : pmsf::boundedRing(b)) {
        if (!hh || hh->isRemoved()) continue;
        if (hh->vertex() == b && hh->end_vertex() == a) {
            if (MCGAL::Facet* f = inspect(hh)) return f;
        }
    }
    return nullptr;
}

bool strictTwoColorLevelEnabled() {
    if (!isEnvOn("PMSF_PCATM_USE_STRICT_PAPER")) return false;
    if (isEnvOn("PMSF_PCATM_STRICT_TWO_COLOR_ONLY")) return true;
    if (isEnvOn("PMSF_PCATM_STRICT_ALLOW_LOW_DEGREE")) return false;
    if (!isEnvOn("PMSF_PCATM_STRICT_ENABLE_MODE2")) return false;
    if (isEnvOn("PMSF_PCATM_STRICT_DISABLE_LEVEL_SWITCH")) return false;
    return (MCGAL::contextPool.levelFast() % 2) == 0;
}

int zShapeBoundaryBlueConflictsBeforeRemoval(
    const std::vector<MCGAL::Vertex*>& ring,
    MCGAL::Vertex* center,
    const pmsf::pcatm::ZShape& z,
    const std::vector<int>& colorSeq,
    int paperLevel) {
    int conflicts = 0;
    const int n = static_cast<int>(ring.size());
    for (int ti = 0; ti < static_cast<int>(z.triangles.size()); ++ti) {
        if (ti >= static_cast<int>(colorSeq.size()) ||
            colorSeq[static_cast<size_t>(ti)] == 0) {
            continue;
        }
        const auto& tri = z.triangles[static_cast<size_t>(ti)];
        const int ids[3] = {tri.v0, tri.v1, tri.v2};
        for (int e = 0; e < 3; ++e) {
            const int ia = ids[e];
            const int ib = ids[(e + 1) % 3];
            if (!isBoundaryEdge(ia, ib, n)) continue;
            MCGAL::Facet* nf = externalFaceAcrossBoundaryEdge(
                ring[static_cast<size_t>(ia)],
                ring[static_cast<size_t>(ib)],
                center);
            if (nf && nf->paperLevel() == paperLevel) {
                ++conflicts;
            }
        }
    }
    return conflicts;
}

bool canTriangulateStrictTwoColorWithoutBlueConflict(
    MCGAL::Halfedge* h,
    const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!strictTwoColorLevelEnabled()) return true;
    // WS-B task-18: when the strict collector constructs the 2-color assignment
    // itself (PMSF_PCATM_STRICT_CONSTRUCTIVE_2COLOR), elimination-time blue
    // colors are irrelevant; the collector validates adjacency/recoverability
    // post-hoc against the decoder matcher. Skip the encoder-side conflict veto
    // so the Z-shape triangulation is not starved of removals.
    if (isEnvOn("PMSF_PCATM_STRICT_CONSTRUCTIVE_2COLOR") ||
        isEnvOn("PMSF_PCATM_STRICT_2COLOR_SIDE")) {
        return true;
    }
    MCGAL::Vertex* center = h ? h->end_vertex() : nullptr;
    std::vector<MCGAL::Vertex*> ring = collectRemovalBoundaryRing(h);
    rotateRingToQuantizedMin(ring, mesh);
    const int n = static_cast<int>(ring.size());
    if (!center || n < 3) return false;
    if (n < 5) {
        const int paperLevel = MCGAL::contextPool.levelFast();
        for (int i = 0; i < n; ++i) {
            MCGAL::Facet* nf = externalFaceAcrossBoundaryEdge(
                ring[static_cast<size_t>(i)],
                ring[static_cast<size_t>((i + 1) % n)],
                center);
            if (nf && nf->paperLevel() == paperLevel) return false;
        }
        return true;
    }
    const std::vector<int> colorSeq = paperTwoColorSequence(n - 2);
    const auto canonical =
        pmsf::pcatm::computeZShape(n, 0, pmsf::pcatm::StartDir::Right);
    const int paperLevel = MCGAL::contextPool.levelFast();
    const int rightConflicts =
        zShapeBoundaryBlueConflictsBeforeRemoval(
            ring, center, canonical, colorSeq, paperLevel);
    return rightConflicts == 0;
}

bool hasLiveEdgeBetween(MCGAL::Vertex* a, MCGAL::Vertex* b) {
    if (!a || !b || a == b) return false;
    for (MCGAL::Halfedge* hh : pmsf::boundedRing(a)) {
        if (!hh || hh->isRemoved() || hh->vertex() != a) continue;
        if (hh->end_vertex() == b) return true;
    }
    for (MCGAL::Halfedge* hh : pmsf::boundedRing(b)) {
        if (!hh || hh->isRemoved() || hh->vertex() != b) continue;
        if (hh->end_vertex() == a) return true;
    }
    return false;
}

bool hasRegisteredHalfedgeBetween(MCGAL::Vertex* a, MCGAL::Vertex* b) {
    if (!a || !b || a == b) return false;
    // Halfedge::reset() scans the append-only Vertex::halfedges() registry,
    // not only the current live ring.  A removed historical halfedge is
    // therefore still unsafe: allocating the reverse halfedge can pair with
    // it and then hit the allocator's duplicate-opposite assertion.
    for (MCGAL::Halfedge* hh : a->halfedges()) {
        if (hh && hh->end_vertex() == b) return true;
    }
    for (MCGAL::Halfedge* hh : b->halfedges()) {
        if (hh && hh->end_vertex() == a) return true;
    }
    return false;
}

bool strictRemovalTopologyPreflight(MCGAL::Halfedge* h) {
    if (!h || h->isRemoved() || !h->vertex() || h->vertex()->isRemoved() ||
        !h->next() || !h->face() || !h->opposite() ||
        !h->end_vertex() || h->end_vertex()->isRemoved()) {
        return false;
    }

    MCGAL::Vertex* center = h->end_vertex();
    const std::vector<MCGAL::Vertex*> ring = collectRemovalBoundaryRing(h);
    if (ring.size() < 3 || ring.size() >= 17) {
        return false;
    }
    for (MCGAL::Vertex* v : ring) {
        if (!v || v->isRemoved() || v == center) {
            return false;
        }
    }

    // The polygon produced by erase_center_vertex must be a simple link.
    // This rejects pinched one-rings and pre-existing chords before any
    // topology mutation can make split_facet() allocate a duplicate edge.
    if (UnifiedManifoldIsRemovableOperator::checkManifoldViolation(
            center, ring, true)) {
        return false;
    }

    const int n = static_cast<int>(ring.size());
    const auto checkZ = [&](const pmsf::pcatm::ZShape& z) {
        if (!z.valid || static_cast<int>(z.triangles.size()) != n - 2 ||
            static_cast<int>(z.diagonals.size()) != n - 3) {
            return false;
        }

        std::vector<std::pair<int, int>> seen;
        seen.reserve(z.diagonals.size());
        for (const auto& diag : z.diagonals) {
            if (diag.va < 0 || diag.va >= n || diag.vb < 0 || diag.vb >= n ||
                diag.va == diag.vb) {
                return false;
            }

            const int a = std::min(diag.va, diag.vb);
            const int b = std::max(diag.va, diag.vb);
            if (isBoundaryEdge(a, b, n)) {
                return false;
            }
            if (std::find(seen.begin(), seen.end(), std::make_pair(a, b)) !=
                seen.end()) {
                return false;
            }
            seen.emplace_back(a, b);

            if (hasLiveEdgeBetween(ring[static_cast<size_t>(a)],
                                   ring[static_cast<size_t>(b)])) {
                return false;
            }
            if (hasRegisteredHalfedgeBetween(
                    ring[static_cast<size_t>(a)],
                    ring[static_cast<size_t>(b)])) {
                return false;
            }
        }
        return true;
    };

    // Strict paper mode has one wire-visible canonical Z phase.  Checking an
    // unselected mirror/fan candidate here would reject a valid candidate
    // before mutation and would make the encoder depend on a geometry that the
    // decoder is not allowed to guess.
    return checkZ(pmsf::pcatm::computeZShape(
        n, 0, pmsf::pcatm::StartDir::Right));
}

MCGAL::Halfedge* incomingHalfedgeTo(MCGAL::Facet* f, MCGAL::Vertex* v) {
    if (!f || !v || !f->proxyHalfedge()) return nullptr;
    MCGAL::Halfedge* st = f->proxyHalfedge();
    MCGAL::Halfedge* it = st;
    int guard = 0;
    do {
        if (!it) break;
        if (it->end_vertex() == v) return it;
        it = it->next();
        if (++guard > 128) break;
    } while (it != st && it != nullptr);
    return nullptr;
}

int faceDegree(MCGAL::Facet* f) {
    if (!f || !f->proxyHalfedge()) return 0;
    int n = 0;
    MCGAL::Halfedge* st = f->proxyHalfedge();
    MCGAL::Halfedge* it = st;
    int guard = 0;
    do {
        if (!it) break;
        ++n;
        it = it->next();
        if (++guard > 128) break;
    } while (it != st && it != nullptr);
    return n;
}

bool faceMatchesTriangle(MCGAL::Facet* f,
                         const std::vector<MCGAL::Vertex*>& polyVerts,
                         const pmsf::pcatm::Triangle& tri) {
    if (!f || f->isRemoved() || faceDegree(f) != 3) return false;
    std::unordered_set<MCGAL::Vertex*> want;
    want.insert(polyVerts[static_cast<size_t>(tri.v0)]);
    want.insert(polyVerts[static_cast<size_t>(tri.v1)]);
    want.insert(polyVerts[static_cast<size_t>(tri.v2)]);
    if (want.size() != 3) return false;
    for (MCGAL::Vertex* fv : f->getVertices()) {
        if (!want.count(fv)) return false;
    }
    return true;
}

bool isBoundaryEdge(int a, int b, int n) {
    if (n <= 0) return false;
    const int d = std::abs(a - b);
    return d == 1 || d == n - 1;
}

MCGAL::Halfedge* boundaryHalfedgeForEdge(MCGAL::Facet* polyFace,
                                         MCGAL::Vertex* a,
                                         MCGAL::Vertex* b) {
    if (!polyFace || !a || !b || !polyFace->proxyHalfedge()) return nullptr;
    MCGAL::Halfedge* st = polyFace->proxyHalfedge();
    MCGAL::Halfedge* it = st;
    int guard = 0;
    do {
        if (!it) break;
        if ((it->vertex() == a && it->end_vertex() == b) ||
            (it->vertex() == b && it->end_vertex() == a)) {
            return it;
        }
        it = it->next();
        if (++guard > 128) break;
    } while (it != st && it != nullptr);
    return nullptr;
}

int zShapeBoundaryBlueConflicts(MCGAL::Facet* polyFace,
                                const std::vector<MCGAL::Vertex*>& polyVerts,
                                const pmsf::pcatm::ZShape& z,
                                const std::vector<int>& colorSeq,
                                int paperLevel) {
    int conflicts = 0;
    const int n = static_cast<int>(polyVerts.size());
    for (int ti = 0; ti < static_cast<int>(z.triangles.size()); ++ti) {
        if (ti >= static_cast<int>(colorSeq.size()) ||
            colorSeq[static_cast<size_t>(ti)] == 0) {
            continue;
        }
        const auto& tri = z.triangles[static_cast<size_t>(ti)];
        const int ids[3] = {tri.v0, tri.v1, tri.v2};
        for (int e = 0; e < 3; ++e) {
            const int a = ids[e];
            const int b = ids[(e + 1) % 3];
            if (!isBoundaryEdge(a, b, n)) continue;
            MCGAL::Halfedge* h = boundaryHalfedgeForEdge(
                polyFace,
                polyVerts[static_cast<size_t>(a)],
                polyVerts[static_cast<size_t>(b)]);
            MCGAL::Facet* nf = (h && h->opposite()) ? h->opposite()->face() : nullptr;
            if (nf && nf != polyFace && !nf->isRemoved() &&
                nf->paperLevel() == paperLevel) {
                ++conflicts;
            }
        }
    }
    return conflicts;
}

}  // namespace

bool PCATMPaperEliminateOperator::eliminate(HalfedgeErrorEntry& entry) {
    MCGAL::Halfedge* h = entry.edge;
    if (h == nullptr) return false;

    // WS-B (task-18): paper §4 2-color Z-shape needs every patch of the round to
    // have a BLUE (middle-triangle) core, i.e. removed valence >= 5. The
    // framework's MinRemovedValenceIsRemovableOperator is ineffective on the
    // strict path (degree cache / selection chain), so enforce it here when the
    // env is set. Env-gated: default path untouched.
    int requiredMinValence = 0;
    if (const char* mvEnv = std::getenv("PMSF_PCATM_STRICT_ENFORCE_MIN_VALENCE");
        mvEnv && *mvEnv) {
        requiredMinValence = std::atoi(mvEnv);
    }
    // Phase mode: only the first N pipeline levels (which are the LAST
    // serialized rounds, i.e. the large meshes that dominate the color stream)
    // are restricted to high-valence removals so their patches are all
    // 2-color Z-shape capable; later levels remove the remaining vertices
    // unrestricted (4-color). The phase is a prefix because serialization is
    // level-reversed.
    if (const char* phaseEnv = std::getenv("PMSF_PCATM_STRICT_2COLOR_PHASE");
        phaseEnv && *phaseEnv && requiredMinValence == 0) {
        const int phase = std::atoi(phaseEnv);
        if (phase > 0 && MCGAL::contextPool.levelFast() < phase) {
            requiredMinValence = 5;
        }
    }
    if (requiredMinValence > 0) {
        MCGAL::Vertex* removed = h->end_vertex();
        int degree = 0;
        if (removed) {
            for (MCGAL::Halfedge* hh : removed->halfedges()) {
                if (hh && !hh->isRemoved() && hh->face() && !hh->face()->isRemoved())
                    ++degree;
            }
        }
        if (degree < requiredMinValence) {
            return false;
        }
    }

    if (!strictRemovalTopologyPreflight(h)) {
        return false;
    }
    if (!canTriangulateStrictTwoColorWithoutBlueConflict(h, mesh_)) {
        return false;
    }

    // 1) adjustment（mirror PCATMZColor / PCATMEliminate）
    if (header_.isEnableAdjustment()) {
        h->vertex()->setAdjusted();
        h->vertex()->setVertexPoint(h->vertex()->point());
        h->vertex()->setPoint(entry.p);
    }

    // 2) remove_point（mirror PCATMZColor —— 量化整数 barycenter）
    CHECK_TRUE(this->remove_point(h));

    // 3) normalize polyFace.proxyHalfedge 到量化整数字典序最小（与 decoder bit-exact 对齐）
    MCGAL::Facet* polyFace = h->face();
    if (polyFace) {
        polyFace->normalize_proxy_to_min_position_vertex(
            mesh_->bboxMin, mesh_->f_quantStep);
        polyFace->setProtocolHint(20);  // CODEC_020 (BlobLayout=20)
    }

    // 4) triangulate —— 本类重写：真 Z-shape
    CHECK_TRUE(this->triangulate(h));

    // 5) 可选全局 alternating invariant 校验（论文 §4：BLUE-BLUE 不共享 edge）。
    // 默认关闭，避免热路径额外全 mesh 扫描；需要验证 encoder Z-shape 时开启。
    if (const char* env = std::getenv("PMSF_PCATMPAPER_ASSERT_ALTERNATING")) {
        if (env[0] == '1') {
            pmsf::pcatm::AlternatingInvariantChecker checker;
            if (!checker.check(mesh_.get())) {
                throw std::runtime_error(
                    "[PCATMPaper] alternating invariant violation count=" +
                    std::to_string(checker.violations().size()));
            }
        }
    }

    // 6) postprocess（no-op，与父类一致）
    CHECK_TRUE(PCATMEliminateOperator::postprocess(h));
    return true;
}

// ----------------------------------------------------------------------------
// triangulate —— 真 Z-shape 实现
// ----------------------------------------------------------------------------

bool PCATMPaperEliminateOperator::triangulate(MCGAL::Halfedge* h) {
    if (!h) return false;
    MCGAL::Facet* polyFace = h->face();
    if (!polyFace) return false;

    // 收集 ring。顺序 = proxyHalfedge() 起 next() 走 N 圈（与 normalize 后 decoder 端一致）。
    std::vector<MCGAL::Vertex*> polyVerts = collectPolygonRing(polyFace);
    const int N = static_cast<int>(polyVerts.size());

    // 已经是三角形或更小（不应发生但安全兜底）：直接标 splittable，无几何变化。
    if (N <= 3) {
        polyFace->setSplittable();
        polyFace->setPaperLevel(MCGAL::contextPool.levelFast());
        polyFace->setPCATMPatchTag(1);
        // 单 triangle face 也贴一个 color（按 startColor=0）以满足 alternating 约束。
        applyColor(polyFace, 0);
        return true;
    }

    const bool strictPaper = isEnvOn("PMSF_PCATM_USE_STRICT_PAPER");
    const std::vector<int> strictColorSeq =
        strictPaper ? paperTwoColorSequence(N - 2) : std::vector<int>{};
    auto z = pmsf::pcatm::computeZShape(
        N, /*startColor=*/0, pmsf::pcatm::StartDir::Right);
    if (strictPaper) {
        const int paperLevel = MCGAL::contextPool.levelFast();
        const int conflicts =
            zShapeBoundaryBlueConflicts(
                polyFace, polyVerts, z, strictColorSeq, paperLevel);
        if (conflicts != 0) {
            std::cerr << "[PCATMPaper] canonical Z has " << conflicts
                      << " blue boundary conflicts (N=" << N << ")"
                      << std::endl;
        }
    }
    if (!z.valid || static_cast<int>(z.triangles.size()) != N - 2) {
        std::cerr << "[PCATMPaper] triangulate WARN: computeZShape triangles="
                  << z.triangles.size() << " expected " << (N - 2) << " (N=" << N << ")"
                  << std::endl;
        return false;
    }

    const bool forceFanFallback = isEnvOn("PMSF_PCATM_STRICT_FAN_FALLBACK");
    if (!forceFanFallback) {
        const int rootPolyValence = polyFace->getPolyValence();
        const MCGAL::Point rootRemovedPos = polyFace->getRemovedVertexPos();
        const MCGAL::PointInt rootRemovedPosInt = polyFace->getRemovedVertexPosInt();
        const uint8_t rootProtocolHint = polyFace->getProtocolHint();
        std::vector<MCGAL::Facet*> patchFaces;
        patchFaces.push_back(polyFace);

        for (const auto& diag : z.diagonals) {
            MCGAL::Vertex* va = polyVerts[static_cast<size_t>(diag.va)];
            MCGAL::Vertex* vb = polyVerts[static_cast<size_t>(diag.vb)];
            MCGAL::Facet* splitFace = nullptr;
            for (MCGAL::Facet* candidate : patchFaces) {
                if (candidate && !candidate->isRemoved() &&
                    faceDegree(candidate) > 3 &&
                    faceContainsVertex(candidate, va) &&
                    faceContainsVertex(candidate, vb)) {
                    splitFace = candidate;
                    break;
                }
            }
            if (!splitFace) {
                std::cerr << "[PCATMPaper] true Z-shape failed: no split face for diag "
                          << diag.va << "-" << diag.vb << " N=" << N << std::endl;
                return false;
            }

            MCGAL::Halfedge* ha = incomingHalfedgeTo(splitFace, va);
            MCGAL::Halfedge* hb = incomingHalfedgeTo(splitFace, vb);
            if (!ha || !hb || ha == hb) {
                std::cerr << "[PCATMPaper] true Z-shape failed: no incoming halfedges for diag "
                          << diag.va << "-" << diag.vb << " N=" << N << std::endl;
                return false;
            }

            MCGAL::Halfedge* hd = mesh_->split_facet(ha, hb);
            if (!hd || !hd->opposite()) {
                std::cerr << "[PCATMPaper] true Z-shape failed: split_facet returned null N="
                          << N << std::endl;
                return false;
            }
            hd->setAdded();
            hd->opposite()->setAdded();
            const int paperLevel = MCGAL::contextPool.levelFast();
            hd->setPaperLevel(paperLevel);
            hd->opposite()->setPaperLevel(paperLevel);
            MCGAL::Facet* fNew = hd->opposite()->face();
            if (fNew) patchFaces.push_back(fNew);
        }

        std::vector<int> colorSeq;
        if (strictPaper) {
            colorSeq = strictColorSeq;
        } else {
            colorSeq.resize(static_cast<size_t>(N - 2));
            for (int i = 0; i < N - 2; ++i) colorSeq[static_cast<size_t>(i)] = z.triangles[i].color;
        }

        int coloredCount = 0;
        std::unordered_set<MCGAL::Facet*> coloredFaces;
        const int paperLevel = MCGAL::contextPool.levelFast();
        for (int i = 0; i < N - 2; ++i) {
            MCGAL::Facet* matched = nullptr;
            for (MCGAL::Facet* f : patchFaces) {
                if (coloredFaces.count(f)) continue;
                if (faceMatchesTriangle(f, polyVerts, z.triangles[static_cast<size_t>(i)])) {
                    matched = f;
                    break;
                }
            }
            if (!matched) {
                std::cerr << "[PCATMPaper] true Z-shape failed: color binding missing tri="
                          << i << " N=" << N << std::endl;
                return false;
            }
            if (i == 0) {
                matched->setPolyValence(rootPolyValence);
                matched->setRemovedVertexPos(rootRemovedPos);
                matched->setRemovedVertexPosInt(rootRemovedPosInt);
                matched->setProtocolHint(rootProtocolHint);
                matched->setSplittable();
            }
            matched->setPCATMPatchTag(1);
            applyColor(matched, colorSeq[static_cast<size_t>(i)]);
            matched->setPaperLevel(paperLevel);
            matched->setProtocolHint(rootProtocolHint);
            coloredFaces.insert(matched);
            ++coloredCount;
        }

        if (coloredCount == N - 2) {
            return true;
        }
        std::cerr << "[PCATMPaper] true Z-shape failed: colored=" << coloredCount
                  << "/" << (N - 2) << " N=" << N << std::endl;
        return false;
    }

    // Diagnostic compatibility path: old fan triangulation + paper colors.
    // Kept only behind PMSF_PCATM_STRICT_FAN_FALLBACK=1 for bisecting regressions.

    // 记录 polyFace 在父类 fan triangulate 前的 proxyHalfedge，用于后续 fan face 收集。
    // 父类 fan 的 triangulation 入口是 proxyHalfedge()（由 normalize 确定），其 vertex
    // = polyVerts[0] 就是 fan apex。
    MCGAL::Vertex* fanApex = polyVerts[0];

    // 调父类 fan triangulate —— 稳定路径，不崩。
    // 父类行为：在 polyFace 上做 v0-fan，切出 N-3 条对角线 (v0 → v2), (v0 → v3), ..., (v0 → v_{N-2})。
    // polyFace 自身保留为第一个 triangle (v0, v1, v2)；其余 N-3 个 triangle 是新 face。
    if (!PCATMEliminateOperator::triangulate(h)) {
        std::cerr << "[PCATMPaper] triangulate ERROR: 父类 fan triangulate 失败 (N="
                  << N << ")" << std::endl;
        return false;
    }

    // 构建 color 序列。
    std::vector<int> colorSeq(N - 2, 0);
    if (isEnvOn("PMSF_PCATM_USE_STRICT_PAPER")) {
        colorSeq = paperTwoColorSequence(N - 2);
    } else if (z.valid && static_cast<int>(z.triangles.size()) == N - 2) {
        for (int i = 0; i < N - 2; ++i) {
            colorSeq[i] = z.triangles[i].color & 1;
        }
    } else {
        for (int i = 0; i < N - 2; ++i) {
            colorSeq[i] = i & 1;
        }
    }

    // ============================================================================
    // 收集 fan 切出的 N-2 个 triangle face：从 fanApex 出发的所有 outgoing halfedges
    // 指向 polyVerts[1..N-1] 中的某一个；它们各对应一个 fan triangle。
    // fan 顺序：triangle_i = (v0, v_{i+1}, v_{i+2}) for i = 0..N-3
    //   = face 上 halfedge ring: v0 → v_{i+1} → v_{i+2} → v0
    //   = fanApex 出发指向 v_{i+1} 的 halfedge 所在的 face
    // ============================================================================
    std::vector<MCGAL::Facet*> fanFaces;
    fanFaces.reserve(N - 2);
    for (int i = 0; i < N - 2; ++i) {
        // 要找 fanApex → polyVerts[i+1] 的 outgoing halfedge
        MCGAL::Vertex* target = polyVerts[i + 1];
        MCGAL::Facet* foundFace = nullptr;
        if (fanApex) {
            for (auto it = fanApex->begin(); it != fanApex->end(); ++it) {
                MCGAL::Halfedge* hh = *it;
                if (!hh || hh->isRemoved()) continue;
                if (hh->end_vertex() != target) continue;
                MCGAL::Facet* f = hh->face();
                if (!f || f->isRemoved()) continue;
                // 校验 face 的第三个顶点 = polyVerts[i+2]（fan 三角形 v0, v_{i+1}, v_{i+2}）
                // 由于 face ring = hh → hh.next() → hh.next().next() → hh，
                // hh.next().end_vertex() 应 = polyVerts[i+2]
                if (hh->next() && hh->next()->end_vertex() == polyVerts[i + 2]) {
                    foundFace = f;
                    break;
                }
            }
        }
        fanFaces.push_back(foundFace);
    }

    // 给每个 fan face 贴 color。未找到的 face 保持默认 RED（不致命）。
    int coloredCount = 0;
    for (int i = 0; i < N - 2; ++i) {
        if (fanFaces[i]) {
            fanFaces[i]->setPCATMPatchTag(1);
            applyColor(fanFaces[i], colorSeq[i]);
            coloredCount++;
        }
    }

    // WS-B fix (PCATM strict color-only decoder): the fan-fallback path is the
    // one actually used by the strict configuration
    // (PMSF_PCATM_STRICT_FAN_FALLBACK=1). Without marking the fan diagonals as
    // "added patch edges" the strict collector's patch-component builder sees
    // every patch as a single triangle (comp size 1) while the true patch
    // valence is N. The decoder then rebuilds a valence-3 patch, i.e. wrong
    // inserted-vertex prediction and wrong topology for every valence>3 patch.
    // Marking the internal fan diagonals (both halfedges) and their paper level
    // lets PCATMStrictPMSFSymbolCollectOperator group the whole N-2 face patch,
    // exactly as the true Z-shape branch already does for its split_facet
    // diagonals. Encoder-side only; stream layout/bytes of the strict path are
    // unchanged except for the now-correct per-patch face grouping.
    {
        const int patchLevel = MCGAL::contextPool.levelFast();
        std::unordered_set<MCGAL::Facet*> fanSet;
        fanSet.reserve(fanFaces.size());
        for (MCGAL::Facet* f : fanFaces) {
            if (f) fanSet.insert(f);
        }
        int markedEdges = 0;
        for (MCGAL::Facet* f : fanFaces) {
            if (!f) continue;
            for (MCGAL::Halfedge* hh : f->getHalfedges()) {
                if (!hh || hh->isRemoved()) continue;
                MCGAL::Halfedge* opp = hh->opposite();
                if (!opp || opp->isRemoved() || !opp->face()) continue;
                if (!fanSet.count(opp->face())) continue;
                // Parent PCATMEliminateOperator::triangulate already marks its
                // split diagonals as added, but never stamps paperLevel; an
                // already-added edge must still get the current level so that
                // isInternalPatchEdge() accepts it.
                hh->setAdded();
                opp->setAdded();
                hh->setPaperLevel(patchLevel);
                opp->setPaperLevel(patchLevel);
                ++markedEdges;
            }
        }
        if (const char* env = std::getenv("PMSF_BPV_DIAG"); env && env[0] == '1') {
            std::fprintf(stderr,
                         "[pcatm-fan-patch] N=%d faces=%d marked=%d level=%d\n",
                         N, static_cast<int>(fanFaces.size()), markedEdges,
                         patchLevel);
        }
    }

    if (coloredCount != N - 2) {
        // 非致命警告：部分 face 未着色。encoder 仍可继续；默认 color=RED（0）。
        static int warnCount = 0;
        if (warnCount < 5) {
            std::cerr << "[PCATMPaper] triangulate WARN: color binding matched="
                      << coloredCount << "/" << (N - 2) << " (N=" << N << ")" << std::endl;
            warnCount++;
        }
    }

    return true;
}


// ----------------------------------------------------------------------------
// helpers
// ----------------------------------------------------------------------------

std::vector<MCGAL::Vertex*> PCATMPaperEliminateOperator::collectPolygonRing(
    MCGAL::Facet* polyFace) {
    std::vector<MCGAL::Vertex*> verts;
    if (!polyFace || !polyFace->proxyHalfedge()) return verts;
    verts.reserve(8);
    MCGAL::Halfedge* st = polyFace->proxyHalfedge();
    MCGAL::Halfedge* it = st;
    int guard = 0;
    do {
        if (!it) break;
        verts.push_back(it->vertex());
        it = it->next();
        if (++guard > 64) break;  // 防御性兜底，N<=10 实际，64 足够
    } while (it != st && it != nullptr);
    return verts;
}

MCGAL::Halfedge* PCATMPaperEliminateOperator::findOutgoingHalfedge(
    MCGAL::Facet* polyFace, MCGAL::Vertex* fromVertex) {
    if (!polyFace || !fromVertex || !polyFace->proxyHalfedge()) return nullptr;
    // 在 polyFace 当前 ring 上找 outgoing halfedge of fromVertex。
    MCGAL::Halfedge* st = polyFace->proxyHalfedge();
    MCGAL::Halfedge* it = st;
    int guard = 0;
    do {
        if (!it) break;
        if (it->vertex() == fromVertex) return it;
        it = it->next();
        if (++guard > 64) break;
    } while (it != st && it != nullptr);
    return nullptr;
}

void PCATMPaperEliminateOperator::applyColor(MCGAL::Facet* face, int colorBit) {
    if (!face) return;
    face->setTriangleColor(colorBit & 1
                           ? MCGAL::Facet::TriangleColorFlag::BLUE
                           : MCGAL::Facet::TriangleColorFlag::RED);
}
