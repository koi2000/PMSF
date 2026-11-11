/*
 * @Author: koi
 * @Date: 2025-09-02 15:45:26
 * @Description:
 */
#include "reconstruct/ReconstructToggles.h"
#include "BasicReconstructOperator.h"
#include "boundary/BoundaryTopologyPatchCodec.h"
#include "boundary/BoundaryIdRegistry.h"
#include "common/CanonicalTieBreakKey.h"
#include "ContextPool.h"
#include "Facet.h"
#include "Mesh.h"
#include "MeshUtils.h"
#include "Vertex.h"
#include "predict/BarycenterPredictor.h"
#include "predict/RDPredictor.h"
#include "predict/DualRingPredictor.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "common/CachedEnv.h"

namespace {

bool enableFanCollapseReplay();
bool enableSafeCenterInsert();
bool enableSkipBothSplittableReplay();
bool enableCanonicalKeepReplay();
bool enablePatchComponentReplay();
bool enablePreventFoldedJoinReplay();
bool reconReasonProfileLogs();

// [PHASE59 诊断，默认关闭] join 成功时对「保留面/被删面是否在本轮 facets 集里」计数。
std::unordered_set<MCGAL::Facet*>* g_dbgFacetSet = nullptr;
long g_dbgJoinKeptInSet = 0;
long g_dbgJoinRemovedInSet = 0;
long g_dbgJoinNeitherInSet = 0;
long g_dbgJoinBothInSet = 0;
int patchFacetSortMode();

// [PHASE59 诊断，默认关闭] 「保留哪一面」规则的因果实验开关。
//
// 现状（mode 0）用**瞬态标记** `gFace->isSplittable()` 决定保留谁；而编码端
// `Mesh::erase_center_vertex` 保留的是**结构决定**的 `h->face()`。本组开关用来
// 测量这一条规则到底影响什么。`PMSF_RECON_KEEP_MODE` ∈ {0,1,2,3}：
//   0 = 原规则（默认，不设环境变量时走这条）
//   1 = 恒保留 hFace   2 = 恒保留 gFace
//   3 = 优先保留本轮 patch 面（承载 removedVertexPos、会被 insert_point 拆的那个）
const std::unordered_set<MCGAL::Facet*>* g_reconPatchFacetSet = nullptr;

int reconKeepMode() {
    static const int mode = [] {
        const char* env = PMSF_CACHED_ENV("PMSF_RECON_KEEP_MODE");
        return env && env[0] != '\0' ? std::atoi(env) : 0;
    }();
    return mode;
}

struct JoinReplayStats {
    int noTopologyH = 0;
    int noVertexH = 0;
    int noEndH = 0;
    int noNextH = 0;
    int noOppH = 0;
    int hRemoved = 0;
    int noTopologyG = 0;
    int noVertexG = 0;
    int noEndG = 0;
    int noNextG = 0;
    int noOppG = 0;
    int fanCollapseSkip = 0;
    int prevMissing = 0;
    int prevMismatch = 0;
    int endpointMissing = 0;
    int faceMissing = 0;
    int bothSplittableSkip = 0;
    int foldedJoinSkip = 0;
    int missingKeep = 0;
    int success = 0;
    int keptH = 0;
    int keptG = 0;
};

thread_local JoinReplayStats* activeJoinReplayStats = nullptr;

bool hasStrictPaperFacet(const std::vector<MCGAL::Facet*>& facets) {
    for (MCGAL::Facet* f : facets) {
        if (f && f->getProtocolHint() == 20)
            return true;
    }
    return false;
}

MCGAL::Facet*
mergeOneStrictPaperDiagonal(MCGAL::Facet* root, const std::vector<MCGAL::Halfedge*>& candidateDiagonals, MCGAL::Facet** mergedNeighbor) {
    if (!root || root->isRemoved())
        return nullptr;
    if (mergedNeighbor)
        *mergedNeighbor = nullptr;
    for (MCGAL::Halfedge* h : candidateDiagonals) {
        if (!h || h->isRemoved() || !h->opposite() || h->opposite()->isRemoved()) {
            continue;
        }
        MCGAL::Facet* fa = h->face();
        MCGAL::Facet* fb = h->opposite()->face();
        if (!fa || !fb || fa->isRemoved() || fb->isRemoved())
            continue;
        // 不变量（勿"修"）：候选两侧同面（fa == fb）时 join_face 会崩，但
        // **简单地 `if (fa == fb) continue;` 更糟** —— 实测崩溃消失，代价是
        // 5444 零面积面 / 3917 重复面 / 4318 非流形边（MeshAudit）。那次合并是
        // **必需的**，跳过只是把"响亮的崩溃"换成"静默的坏网格"，而顶点数照样对
        // （A1 看不出来）⇒ 对 R7 而言更糟。根因在更上游：两侧同面本身说明
        // 解码端重建序列已与编码端发散（与 D4 同一家族），须在上游定位。
        if (fa != root && fb != root)
            continue;
        if (mergedNeighbor)
            *mergedNeighbor = (fa == root) ? fb : fa;
        const int polyValence = root->getPolyValence();
        const auto removedPos = root->getRemovedVertexPos();
        const auto removedPosInt = root->getRemovedVertexPosInt();
        const uint8_t protocolHint = root->getProtocolHint();
        MCGAL::Halfedge* keptHalfedge = MCGAL::join_face(fa == root ? h : h->opposite());
        MCGAL::Facet* kept = keptHalfedge ? keptHalfedge->face() : nullptr;
        if (!kept || kept->isRemoved())
            return nullptr;
        kept->setPolyValence(polyValence);
        kept->setRemovedVertexPos(removedPos);
        kept->setRemovedVertexPosInt(removedPosInt);
        kept->setProtocolHint(protocolHint);
        if (!kept->isSplittable())
            kept->setSplittable();
        return kept;
    }
    return nullptr;
}

bool isAliveHalfedge(MCGAL::Halfedge* h) {
    return h && !h->isRemoved() && h->vertex() && h->end_vertex() && h->next() && h->face() && !h->face()->isRemoved();
}

bool hasTopologyReplayData(MCGAL::Halfedge* h) {
    return h && h->vertex() && h->end_vertex() && h->next() && h->opposite();
}

bool hasClosedInsertionCycle(MCGAL::Halfedge* start) {
    if (!start || !start->vertex() || !start->end_vertex()) {
        return false;
    }
    MCGAL::Halfedge* h = start;
    for (int steps = 0; steps < 100000; ++steps) {
        if (!h || !h->vertex() || !h->end_vertex() || !h->next()) {
            return false;
        }
        h = h->next();
        if (h == start)
            return steps >= 2;
    }
    return false;
}

// ── P32-4：**D18 三个存活方向各布一探针**（默认关闭，PMSF_D18_TRI_PROBE=1） ──
// P30/P31 已把「边界 / 环拼接 / 保面拼接 / 中心点扇」四条线用数据关闭，
// 剩下三个候选：① join_face 内部重排、② reset_without_init + 面复用、③ insert_point。
// 三者都以同一个可观测症状出现：**某个面的 next() 环不足 3 条半边**。
// 这里放一个共用的度数检查，在上述三处的**出口**各调一次。
int d18FaceDegree(MCGAL::Facet* f) {
    if (!f || f->isRemoved())
        return -1;
    MCGAL::Halfedge* h = f->proxyHalfedge();
    if (!h)
        return -1;
    MCGAL::Halfedge* it = h;
    for (int k = 0; k < 64; ++k) {
        MCGAL::Halfedge* nx = it->next();
        if (nx == nullptr || nx == h)
            return k + 1;
        it = nx;
    }
    return 64;
}
void d18TriProbe(const char* site, MCGAL::Facet* f) {
    const char* env = PMSF_CACHED_ENV("PMSF_D18_TRI_PROBE");
    if (!env || env[0] != '1')
        return;
    const int d = d18FaceDegree(f);
    if (d >= 3 || d < 0)
        return;
    static int seen[3] = {0, 0, 0};
    const int idx = (site[0] == 'S') ? 0 : (site[0] == 'R' ? 1 : 2);   // splice/reset/insert
    if (seen[idx] >= 20)
        return;
    ++seen[idx];
    std::fprintf(stderr, "[D18-probe-7] site=%s DEGENERATE face degree=%d face=%p\n",
                 site, d, static_cast<void*>(f));
}

MCGAL::Halfedge* findInsertionProxy(MCGAL::Facet* facet) {
    if (!facet)
        return nullptr;
    MCGAL::Halfedge* h = facet->proxyHalfedge();
    auto tryProxy = [&](MCGAL::Halfedge* candidate) -> MCGAL::Halfedge* {
        if (!hasClosedInsertionCycle(candidate)) return nullptr;
        // Optional diagnostic guard: the fallback scans neighbouring halfedges
        // and can otherwise return a cycle of a *different* facet, which makes
        // insert_point split the wrong face.  Strict mode only accepts a cycle
        // belonging to the original facet.
        if (PMSF_CACHED_ENV("PMSF_INS_PROXY_STRICT_FACE") && candidate->face() != facet)
            return nullptr;
        return candidate;
    };
    if (MCGAL::Halfedge* p = tryProxy(h))
        return p;
    if (h && h->face()) {
        if (MCGAL::Halfedge* p = tryProxy(h->face()->proxyHalfedge()))
            return p;
    }
    if (h) {
        MCGAL::Halfedge* cur = h;
        for (int i = 0; i < 8 && cur; ++i) {
            if (MCGAL::Halfedge* p = tryProxy(cur))
                return p;
            cur = cur->next();
        }
    }
    if (h && h->opposite()) {
        MCGAL::Halfedge* op = h->opposite();
        if (MCGAL::Halfedge* p = tryProxy(op))
            return p;
        if (op->face()) {
            if (MCGAL::Halfedge* p = tryProxy(op->face()->proxyHalfedge()))
                return p;
        }
        MCGAL::Halfedge* cur = op;
        for (int i = 0; i < 8 && cur; ++i) {
            if (MCGAL::Halfedge* p = tryProxy(cur))
                return p;
            cur = cur->next();
        }
    }
    return nullptr;
}

MCGAL::Halfedge* boundaryHalfedgeFromVertex(const std::vector<MCGAL::Halfedge*>& boundary, MCGAL::Vertex* v) {
    if (!v)
        return nullptr;
    for (MCGAL::Halfedge* h : boundary) {
        if (h && !h->isRemoved() && h->vertex() == v)
            return h;
    }
    return nullptr;
}

bool collapseV0FanToPolygon(MCGAL::Facet* root, int targetDegree) {
    if (!root || root->isRemoved() || targetDegree <= 3 || root->facet_degree() >= targetDegree) {
        return false;
    }
    MCGAL::Halfedge* start = root->proxyHalfedge();
    if (!isAliveHalfedge(start) || start->face() != root)
        return false;
    MCGAL::Halfedge* h1 = start->next();
    if (!isAliveHalfedge(h1))
        return false;
    MCGAL::Halfedge* diag = h1->next();
    if (!isAliveHalfedge(diag) || !isAliveHalfedge(diag->opposite()))
        return false;

    std::vector<MCGAL::Halfedge*> boundary;
    std::vector<MCGAL::Halfedge*> internal;
    std::vector<MCGAL::Facet*> removedFaces;
    boundary.reserve(static_cast<size_t>(targetDegree));
    internal.reserve(static_cast<size_t>((targetDegree - 3) * 2));
    removedFaces.reserve(static_cast<size_t>(targetDegree - 3));
    boundary.push_back(start);
    boundary.push_back(h1);

    std::unordered_set<MCGAL::Facet*> seenRemovedFaces;
    for (int i = 0; i < targetDegree - 3; ++i) {
        if (!isAliveHalfedge(diag))
            return false;
        MCGAL::Halfedge* op = diag->opposite();
        if (!isAliveHalfedge(op))
            return false;
        MCGAL::Facet* fanFace = op->face();
        if (!fanFace || fanFace->isRemoved())
            return false;

        internal.push_back(diag);
        internal.push_back(op);
        if (fanFace != root && seenRemovedFaces.insert(fanFace).second) {
            removedFaces.push_back(fanFace);
        }

        MCGAL::Halfedge* edge = op->next();
        if (!isAliveHalfedge(edge))
            return false;
        boundary.push_back(edge);
        if (i == targetDegree - 4) {
            MCGAL::Halfedge* last = edge->next();
            if (!isAliveHalfedge(last))
                return false;
            boundary.push_back(last);
        } else {
            diag = edge->next();
        }
    }

    if (boundary.size() != static_cast<size_t>(targetDegree))
        return false;
    std::unordered_set<MCGAL::Halfedge*> seenBoundary;
    seenBoundary.reserve(boundary.size() * 2 + 1);
    for (size_t i = 0; i < boundary.size(); ++i) {
        MCGAL::Halfedge* h = boundary[i];
        MCGAL::Halfedge* next = boundary[(i + 1) % boundary.size()];
        if (!isAliveHalfedge(h) || !isAliveHalfedge(next))
            return false;
        if (!seenBoundary.insert(h).second)
            return false;
        if (h->end_vertex() != next->vertex())
            return false;
    }
    std::unordered_set<MCGAL::Halfedge*> seenInternal;
    seenInternal.reserve(internal.size() * 2 + 1);
    for (MCGAL::Halfedge* h : internal) {
        if (!isAliveHalfedge(h) || seenBoundary.count(h) != 0 || !seenInternal.insert(h).second) {
            return false;
        }
    }

    for (MCGAL::Halfedge* h : internal) {
        MCGAL::Vertex* v = h->vertex();
        if (v && v->proxyHalfedge() == h) {
            if (MCGAL::Halfedge* replacement = boundaryHalfedgeFromVertex(boundary, v)) {
                v->setProxyHalfedge(replacement);
            }
        }
    }
    for (size_t i = 0; i < boundary.size(); ++i) {
        boundary[i]->setNext(boundary[(i + 1) % boundary.size()]);
    }
    for (MCGAL::Halfedge* h : internal) {
        h->setRemoved();
    }
    for (MCGAL::Facet* f : removedFaces) {
        f->setRemoved();
    }
    root->reset_without_init(boundary[0]);
    if (!root->isSplittable())
        root->setSplittable();
    return root->facet_degree() == targetDegree;
}

bool isRestoredPolyFace(MCGAL::Facet* f) {
    return f && !f->isRemoved() && f->getPolyValence() > 3 && f->facet_degree() >= f->getPolyValence();
}

MCGAL::Halfedge* findPrevInBoundedRing(MCGAL::Halfedge* h) {
    if (!hasTopologyReplayData(h))
        return nullptr;
    MCGAL::Facet* face = h->face();
    const int degree = face ? face->facet_degree() : 0;
    const int limit = std::min(100000, std::max(16, degree + 8));
    MCGAL::Halfedge* prev = h;
    for (int i = 0; i < limit; ++i) {
        MCGAL::Halfedge* next = prev->next();
        if (!next)
            return nullptr;
        if (next == h)
            return prev;
        prev = next;
    }
    return nullptr;
}

MCGAL::Halfedge* findAddedEdgeOnFaceBoundary(MCGAL::Facet* f) {
    if (!f || f->isRemoved() || !f->proxyHalfedge())
        return nullptr;
    MCGAL::Halfedge* start = f->proxyHalfedge();
    MCGAL::Halfedge* h = start;
    for (int guard = 0; h && guard < 100000; ++guard) {
        MCGAL::Halfedge* opp = h->opposite();
        if (opp && !opp->isRemoved() && opp->face() && opp->face() != f && !opp->face()->isRemoved() && (h->isAdded() || opp->isAdded())) {
            return h;
        }
        h = h->next();
        if (h == start)
            break;
    }
    return nullptr;
}

struct EdgePtrKey {
    MCGAL::Vertex* a = nullptr;
    MCGAL::Vertex* b = nullptr;

    bool operator==(const EdgePtrKey& other) const {
        return a == other.a && b == other.b;
    }
};

struct EdgePtrKeyHash {
    std::size_t operator()(const EdgePtrKey& key) const {
        std::uintptr_t av = reinterpret_cast<std::uintptr_t>(key.a);
        std::uintptr_t bv = reinterpret_cast<std::uintptr_t>(key.b);
        std::size_t h = std::hash<std::uintptr_t>{}(av);
        h ^= std::hash<std::uintptr_t>{}(bv) + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
        return h;
    }
};

EdgePtrKey undirectedEdgeKey(MCGAL::Vertex* a, MCGAL::Vertex* b) {
    std::uintptr_t av = reinterpret_cast<std::uintptr_t>(a);
    std::uintptr_t bv = reinterpret_cast<std::uintptr_t>(b);
    if (bv < av)
        std::swap(a, b);
    return EdgePtrKey{a, b};
}

bool mergeWouldCreateFoldedBoundary(MCGAL::Halfedge* h, MCGAL::Halfedge* hprev, MCGAL::Halfedge* gprev) {
    if (!h || !hprev || !gprev || !h->opposite())
        return true;
    MCGAL::Halfedge* g = h->opposite();
    MCGAL::Halfedge* hNext = h->next();
    MCGAL::Halfedge* gNext = g->next();
    if (!hNext || !gNext)
        return true;

    auto mergedNext = [&](MCGAL::Halfedge* cur) -> MCGAL::Halfedge* {
        if (cur == hprev)
            return gNext;
        if (cur == gprev)
            return hNext;
        return cur ? cur->next() : nullptr;
    };

    std::unordered_set<MCGAL::Vertex*> seenVertices;
    std::unordered_set<EdgePtrKey, EdgePtrKeyHash> seenEdges;
    seenVertices.reserve(32);
    seenEdges.reserve(32);

    MCGAL::Halfedge* start = hprev;
    MCGAL::Halfedge* cur = start;
    for (int guard = 0; cur && guard < 100000; ++guard) {
        if (cur == h || cur == g || !cur->vertex() || !cur->end_vertex()) {
            return true;
        }
        if (!seenVertices.insert(cur->vertex()).second) {
            return true;
        }
        EdgePtrKey key = undirectedEdgeKey(cur->vertex(), cur->end_vertex());
        if (!seenEdges.insert(key).second) {
            return true;
        }
        cur = mergedNext(cur);
        if (cur == start)
            return false;
    }
    return true;
}

static MCGAL::Mesh* g_reconTraceMesh = nullptr;

bool joinFaceForTopologyReplay(MCGAL::Halfedge* h) {
    if (pmsf::forceNoJoinReplay()) return false;
    if (PMSF_CACHED_ENV("PMSF_RECON_NO_JOIN_REPLAY")) return false;
    JoinReplayStats* stats = activeJoinReplayStats;
    if (!hasTopologyReplayData(h)) {
        if (stats) {
            ++stats->noTopologyH;
            if (!h || !h->vertex())
                ++stats->noVertexH;
            if (!h || !h->end_vertex())
                ++stats->noEndH;
            if (!h || !h->next())
                ++stats->noNextH;
            if (!h || !h->opposite())
                ++stats->noOppH;
        }
        return false;
    }
    if (h->isRemoved()) {
        if (stats)
            ++stats->hRemoved;
        return false;
    }
    MCGAL::Halfedge* g = h->opposite();
    if (!hasTopologyReplayData(g)) {
        if (stats) {
            ++stats->noTopologyG;
            if (!g || !g->vertex())
                ++stats->noVertexG;
            if (!g || !g->end_vertex())
                ++stats->noEndG;
            if (!g || !g->next())
                ++stats->noNextG;
            if (!g || !g->opposite())
                ++stats->noOppG;
        }
        return false;
    }
    if (enableFanCollapseReplay() && (isRestoredPolyFace(h->face()) || isRestoredPolyFace(g->face()))) {
        if (stats)
            ++stats->fanCollapseSkip;
        return false;
    }

    MCGAL::Halfedge* hprev = findPrevInBoundedRing(h);
    MCGAL::Halfedge* gprev = findPrevInBoundedRing(g);
    if (!hprev || !gprev) {
        if (stats)
            ++stats->prevMissing;
        return false;
    }
    if (hprev->next() != h || gprev->next() != g) {
        if (stats)
            ++stats->prevMismatch;
        return false;
    }
    if (!h->next() || !g->next() || !h->vertex() || !g->vertex()) {
        if (stats)
            ++stats->endpointMissing;
        return false;
    }
    MCGAL::Facet* hFace = hprev->face();
    MCGAL::Facet* gFace = gprev->face();
    if (!hFace || !gFace) {
        if (stats)
            ++stats->faceMissing;
        return false;
    }
    if (const char* jrp = PMSF_CACHED_ENV("PMSF_JOIN_REQUIRE_PATCH"); jrp && jrp[0]) {
        const int mode = std::atoi(jrp);
        const bool hPoly = hFace->getPolyValence() > 3;
        const bool gPoly = gFace->getPolyValence() > 3;
        const bool hPatch = hPoly || hFace->isSplittable();
        const bool gPatch = gPoly || gFace->isSplittable();
        bool keep = true;
        if (mode == 1) keep = hPatch || gPatch;
        else if (mode == 2) keep = hPoly || gPoly;
        else if (mode == 3) keep = hPoly && gPoly;
        else if (mode == 4) keep = (hPoly && gFace->isSplittable()) || (gPoly && hFace->isSplittable());
        else if (mode == 5) keep = hFace->getPolyValence() > 0 || gFace->getPolyValence() > 0;
        else if (mode == 6) keep = hFace->getPolyValence() > 0 && gFace->getPolyValence() > 0;
        if (!keep) return false;
    }
    if (enableSkipBothSplittableReplay() && hFace->isSplittable() && gFace->isSplittable()) {
        if (stats)
            ++stats->bothSplittableSkip;
        return false;
    }
    if (enablePreventFoldedJoinReplay() && mergeWouldCreateFoldedBoundary(h, hprev, gprev)) {
        if (stats)
            ++stats->foldedJoinSkip;
        return false;
    }

    bool keepGFace = gFace->isSplittable();
    if (enableCanonicalKeepReplay() && hFace->isSplittable() == gFace->isSplittable()) {
        keepGFace = gFace->poolId() < hFace->poolId();
    }
    // [PHASE59 诊断，默认关闭] 规则因果实验：mode 0 时本块整体不生效。
    if (const int keepMode = reconKeepMode(); keepMode != 0) {
        if (keepMode == 1) {
            keepGFace = false;
        } else if (keepMode == 2) {
            keepGFace = true;
        } else if (keepMode == 3 && g_reconPatchFacetSet != nullptr) {
            const bool hIn = g_reconPatchFacetSet->count(hFace) != 0;
            const bool gIn = g_reconPatchFacetSet->count(gFace) != 0;
            if (hIn != gIn)
                keepGFace = gIn;
        }
    }
    MCGAL::Halfedge* keptHalfedge = keepGFace ? gprev : hprev;
    MCGAL::Facet* removedFace = keepGFace ? hFace : gFace;
    MCGAL::Facet* keptFace = keepGFace ? gFace : hFace;
    if (!keptHalfedge || !removedFace || !keptFace) {
        if (stats)
            ++stats->missingKeep;
        return false;
    }

    MCGAL::Halfedge* hNext = h->next();
    MCGAL::Halfedge* gNext = g->next();
    // ── P30 物品3：**D18 源头守卫**（默认关闭，PMSF_D18_SPLICE_PROBE=1 开） ──
    // 四级探针已把 D18 的坏结构钉成 **digon**：某面只有两条半边 {hprev, gprev}
    // 且互为 next。而本处拼接 `hprev->setNext(gNext); gprev->setNext(hNext);`
    // 恰好会把 {hprev, gprev} 缝成 2-环 **当且仅当** gNext==gprev 且 hNext==hprev
    // —— 那意味着 h/g 各自所在的环**本来就只有 2 条半边**。这里因此是
    // 「谁写坏的」唯一候选点，故在写之前设哨：四个指针一出现退化重合
    // （含 hprev==gprev 这种「两条半边同前驱」），立刻打出顶点号。
    // 纯诊断：不改任何写指针的顺序与内容。
    if (const char* env = PMSF_CACHED_ENV("PMSF_D18_SPLICE_PROBE"); env && env[0] == '1') {
        const bool degenerate = (hNext == hprev) || (gNext == gprev) ||
                                (hprev == gprev) || (hNext == gNext);
        if (degenerate) {
            static int spliceProbe = 0;
            if (spliceProbe < 40) {
                ++spliceProbe;
                auto vid = [](MCGAL::Halfedge* e) {
                    return (e && e->vertex()) ? e->vertex()->poolId() : -1;
                };
                auto vn = [](MCGAL::Halfedge* e) {
                    MCGAL::Halfedge* nx = e ? e->next() : nullptr;
                    return (nx && nx->vertex()) ? nx->vertex()->poolId() : -1;
                };
                std::fprintf(stderr,
                    "[D18-probe-5] DEGENERATE SPLICE: h(%d->%d) g(%d->%d) "
                    "hprev(%d->%d) gprev(%d->%d) hNext(%d->%d) gNext(%d->%d) "
                    "flags[hNext==hprev:%d gNext==gprev:%d hprev==gprev:%d hNext==gNext:%d]\n",
                    vid(h), vn(h), vid(g), vn(g), vid(hprev), vn(hprev), vid(gprev), vn(gprev),
                    vid(hNext), vn(hNext), vid(gNext), vn(gNext),
                    (hNext == hprev) ? 1 : 0, (gNext == gprev) ? 1 : 0,
                    (hprev == gprev) ? 1 : 0, (hNext == gNext) ? 1 : 0);
            }
        }
    }
    if (const char* jt = PMSF_CACHED_ENV("PMSF_JOIN_TRACE"); jt && jt[0]) {
        if (FILE* fp = pmsf::bfsDumpFile(jt)) {
            static long joinPreSeq = 0;
            auto ringOf = [&](MCGAL::Facet* f) {
                std::vector<std::array<int, 3>> r;
                if (!f) return r;
                MCGAL::Halfedge* h0 = f->proxyHalfedge();
                MCGAL::Halfedge* hh = h0; int guard = 0;
                do {
                    if (hh && hh->vertex()) {
                        auto q = g_reconTraceMesh->floatPosToInt(hh->vertex()->point());
                        r.push_back({(int)q.x(), (int)q.y(), (int)q.z()});
                    }
                    hh = hh->next();
                } while (hh && hh != h0 && ++guard < 64);
                std::sort(r.begin(), r.end());
                return r;
            };
            auto hR = ringOf(hFace), gR = ringOf(gFace);
            std::fprintf(fp, "PREJOIN seq=%ld hRing=", joinPreSeq++);
            for (auto& p : hR) std::fprintf(fp, " %d,%d,%d", p[0], p[1], p[2]);
            std::fprintf(fp, " gRing=");
            for (auto& p : gR) std::fprintf(fp, " %d,%d,%d", p[0], p[1], p[2]);
            std::fputc('\n', fp);
        }
    }
    hprev->setNext(gNext);
    gprev->setNext(hNext);
    d18TriProbe("Splice", keptFace);
    g->setRemoved();
    h->setRemoved();
    if (h->vertex()->proxyHalfedge() == h) {
        h->vertex()->setProxyHalfedge(hprev);
    }
    if (g->vertex()->proxyHalfedge() == g) {
        g->vertex()->setProxyHalfedge(gprev);
    }
    if (g_dbgFacetSet != nullptr) {
        const bool kIn = g_dbgFacetSet->count(keptFace) != 0;
        const bool rIn = g_dbgFacetSet->count(removedFace) != 0;
        if (kIn)
            ++g_dbgJoinKeptInSet;
        if (rIn)
            ++g_dbgJoinRemovedInSet;
        if (!kIn && !rIn)
            ++g_dbgJoinNeitherInSet;
        if (kIn && rIn)
            ++g_dbgJoinBothInSet;
    }
    removedFace->setRemoved();
    keptFace->reset_without_init(keptHalfedge);
    if (const char* jt = PMSF_CACHED_ENV("PMSF_JOIN_TRACE"); jt && jt[0]) {
        if (FILE* fp = pmsf::bfsDumpFile(jt)) {
            static long joinSeq = 0;
            std::fprintf(fp, "JOIN seq=%ld kept=%d keepG=%d pv=%d ring=", joinSeq++,
                         keptFace->poolId(), keepGFace ? 1 : 0, keptFace->getPolyValence());
            std::vector<std::array<int, 3>> ring;
            MCGAL::Halfedge* h0 = keptHalfedge;
            MCGAL::Halfedge* hh = h0; int guard = 0;
            do {
                if (hh && hh->vertex()) {
                    if (g_reconTraceMesh) {
                        auto q = g_reconTraceMesh->floatPosToInt(hh->vertex()->point());
                        ring.push_back({(int)q.x(), (int)q.y(), (int)q.z()});
                    } else {
                        MCGAL::Point p = hh->vertex()->point();
                        ring.push_back({(int)p[0], (int)p[1], (int)p[2]});
                    }
                }
                hh = hh->next();
            } while (hh && hh != h0 && ++guard < 64);
            std::sort(ring.begin(), ring.end());
            for (auto& p : ring) std::fprintf(fp, " %d,%d,%d", p[0], p[1], p[2]);
            std::fputc('\n', fp);
        }
    }
    if (stats) {
        ++stats->success;
        if (keepGFace) {
            ++stats->keptG;
        } else {
            ++stats->keptH;
        }
    }
    return true;
}

MCGAL::Point reconstructBarycenterPointNoAlloc(MCGAL::Facet* facet, const MCGAL::PointInt& residual, const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!facet || !mesh || !facet->proxyHalfedge()) {
        return MCGAL::Point(0.0f, 0.0f, 0.0f);
    }
    int64_t sx = 0;
    int64_t sy = 0;
    int64_t sz = 0;
    int n = 0;
    MCGAL::Halfedge* start = facet->proxyHalfedge();
    MCGAL::Halfedge* h = start;
    int guard = 0;
    do {
        if (h && h->vertex()) {
            MCGAL::PointInt pi = mesh->floatPosToInt(h->vertex()->point());
            sx += pi.x();
            sy += pi.y();
            sz += pi.z();
            ++n;
        }
        h = h ? h->next() : nullptr;
    } while (h && h != start && ++guard < 100000);
    if (n == 0) {
        return MCGAL::Point(0.0f, 0.0f, 0.0f);
    }
    MCGAL::PointInt q(static_cast<int>(sx / n) + residual.x(), static_cast<int>(sy / n) + residual.y(), static_cast<int>(sz / n) + residual.z());
    return MCGAL::rdIntToFloatPoint(q, mesh->bboxMin, mesh->f_quantStep);
}

std::vector<MCGAL::Halfedge*> collectClosedRing(MCGAL::Halfedge* start) {
    std::vector<MCGAL::Halfedge*> ring;
    if (!start || !start->vertex() || !start->end_vertex())
        return ring;
    MCGAL::Halfedge* h = start;
    ring.reserve(8);
    for (int guard = 0; guard < 8192; ++guard) {
        if (!h || !h->vertex() || !h->end_vertex() || !h->next()) {
            ring.clear();
            return ring;
        }
        ring.push_back(h);
        h = h->next();
        if (h == start)
            return ring;
    }
    ring.clear();
    return ring;
}

MCGAL::Halfedge* createCenterVertexFromClosedRing(const std::shared_ptr<MCGAL::Mesh>& mesh, MCGAL::Halfedge* start, const MCGAL::Point& point) {
    if (!mesh || !start || !start->face())
        return nullptr;
    std::vector<MCGAL::Halfedge*> ring = collectClosedRing(start);
    const int n = static_cast<int>(ring.size());
    if (n < 3)
        return nullptr;
    for (int i = 0; i < n; ++i) {
        MCGAL::Halfedge* h = ring[static_cast<size_t>(i)];
        MCGAL::Halfedge* next = ring[static_cast<size_t>((i + 1) % n)];
        if (!h || !next || h->end_vertex() != next->vertex())
            return nullptr;
    }

    const int meshId = mesh->meshId();
    MCGAL::Vertex* center = MCGAL::contextPool.allocateVertexFromPool(meshId, point);
    center->setGroupId(start->end_vertex() ? start->end_vertex()->groupId() : 0);
    mesh->vertices().push_back(center);
    // V3 ID contract: this is an interior-vertex restoration path as well; register
    // its I-ID (round + forward ordinal) exactly like insert_point does.
    if (pmsf::bndid::dec().on) pmsf::bndid::decRegisterInterior(center);

    std::vector<MCGAL::Halfedge*> toCenter(static_cast<size_t>(n), nullptr);
    std::vector<MCGAL::Halfedge*> fromCenter(static_cast<size_t>(n), nullptr);
    for (int i = 0; i < n; ++i) {
        MCGAL::Vertex* boundaryVertex = ring[static_cast<size_t>(i)]->end_vertex();
        toCenter[static_cast<size_t>(i)] = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(meshId, boundaryVertex, center);
        fromCenter[static_cast<size_t>(i)] = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(meshId, center, boundaryVertex);
        toCenter[static_cast<size_t>(i)]->setOpposite(fromCenter[static_cast<size_t>(i)]);
        fromCenter[static_cast<size_t>(i)]->setOpposite(toCenter[static_cast<size_t>(i)]);
    }
    center->setProxyHalfedge(fromCenter[0]);

    for (int i = 0; i < n; ++i) {
        const int prev = (i + n - 1) % n;
        ring[static_cast<size_t>(i)]->setNext(toCenter[static_cast<size_t>(i)]);
        toCenter[static_cast<size_t>(i)]->setNext(fromCenter[static_cast<size_t>(prev)]);
        fromCenter[static_cast<size_t>(prev)]->setNext(ring[static_cast<size_t>(i)]);
    }

    MCGAL::Facet* origin = start->face();
    origin->reset_without_init(ring[0]);
    d18TriProbe("Reset", origin);
    for (int i = 1; i < n; ++i) {
        MCGAL::Facet* face = MCGAL::contextPool.allocateFaceFromPool(meshId, ring[static_cast<size_t>(i)]);
        mesh->add_face(face);
    }
    // ── P31 B1：**扇 vs 拼接，谁生出 digon？**（默认关闭，PMSF_D18_FAN_PROBE=1） ──
    // P30 五级探针已证：坏结构是 digon，且在**保面拼接**处可见（40 次签名一致），
    // 但那处的第一条不变量说明 digon **到达时已成 2-环** ⇒ 拼接只是传播。
    // 解码路径上写 setNext 的**只有两处**：本处的中心点扇、以及保面拼接。
    // 于是只需问一句：**扇刚做完时，新建的这些面里有没有 <3 条半边的？**
    //   有 ⇒ digon 生于扇（`ring` 里有退化条目）；
    //   无 ⇒ 生于别处，扇与拼接都可排除，范围再收一档。
    // 纯诊断：不改任何写指针。
    if (const char* env = PMSF_CACHED_ENV("PMSF_D18_FAN_PROBE"); env && env[0] == '1') {
        static int fanProbe = 0;
        auto degree = [](MCGAL::Halfedge* h) {
            int d = 0;
            MCGAL::Halfedge* it = h;
            for (int k = 0; k < 64 && it; ++k) {
                ++d;
                MCGAL::Halfedge* nx = it->next();
                if (nx == nullptr || nx == h) break;
                it = nx;
            }
            return d;
        };
        const int d0 = degree(ring[0]);
        if (d0 < 3 && fanProbe < 40) {
            ++fanProbe;
            std::fprintf(stderr,
                "[D18-probe-6] DEGENERATE FAN: n=%d originFaceDegree=%d "
                "ring0=(%d->%d) => digon is born IN THE FAN (degenerate ring entry)\n",
                n, d0,
                ring[0]->vertex() ? ring[0]->vertex()->poolId() : -1,
                ring[0]->end_vertex() ? ring[0]->end_vertex()->poolId() : -1);
        } else if (fanProbe == 0) {
            static int okProbe = 0;
            if (++okProbe == 1)
                std::fprintf(stderr,
                    "[D18-probe-6] first fan is healthy: n=%d originFaceDegree=%d "
                    "(digon NOT born here => look elsewhere)\n", n, d0);
        }
    }
    pmsf::applyBoundaryTopologyAtVertex(mesh, center, pmsf::pendingBoundaryTopologyRound());
    return fromCenter[0];
}

bool reconstructProfileLogs() {
    static const bool enabled = [] {
        const char* env = PMSF_CACHED_ENV("PMSF_PROFILE");
        return env && env[0] != '\0' && std::strcmp(env, "0") != 0 && std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
    }();
    return enabled;
}

bool enableFanCollapseReplay() {
    static const bool enabled = [] {
        const char* env = PMSF_CACHED_ENV("PMSF_RECON_FAN_COLLAPSE");
        return env && env[0] != '\0' && std::strcmp(env, "0") != 0 && std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
    }();
    return enabled;
}

bool enableSafeCenterInsert() {
    static const bool enabled = [] {
        const char* env = PMSF_CACHED_ENV("PMSF_RECON_SAFE_INSERT");
        return env && env[0] != '\0' && std::strcmp(env, "0") != 0 && std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
    }();
    return enabled;
}

bool enableSkipBothSplittableReplay() {
    static const bool enabled = [] {
        const char* env = PMSF_CACHED_ENV("PMSF_RECON_SKIP_BOTH_SPLITTABLE");
        return env && env[0] != '\0' && std::strcmp(env, "0") != 0 && std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
    }();
    return enabled;
}

bool enableCanonicalKeepReplay() {
    static const bool enabled = [] {
        const char* env = PMSF_CACHED_ENV("PMSF_RECON_CANONICAL_KEEP");
        return env && env[0] != '\0' && std::strcmp(env, "0") != 0 && std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
    }();
    return enabled;
}

bool enablePatchComponentReplay() {
    static const bool enabled = [] {
        const char* env = PMSF_CACHED_ENV("PMSF_RECON_PATCH_COMPONENT");
        return env && env[0] != '\0' && std::strcmp(env, "0") != 0 && std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
    }();
    return enabled;
}

bool enablePreventFoldedJoinReplay() {
    static const bool enabled = [] {
        const char* env = PMSF_CACHED_ENV("PMSF_RECON_PREVENT_FOLDED_JOIN");
        return env && env[0] != '\0' && std::strcmp(env, "0") != 0 && std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
    }();
    return enabled;
}

bool reconReasonProfileLogs() {
    static const bool enabled = [] {
        const char* env = PMSF_CACHED_ENV("PMSF_RECON_REASON_PROFILE");
        return env && env[0] != '\0' && std::strcmp(env, "0") != 0 && std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
    }();
    return enabled;
}

int patchFacetSortMode() {
    static const int mode = [] {
        const char* env = PMSF_CACHED_ENV("PMSF_RECON_PATCH_SORT");
        if (!env || env[0] == '\0')
            return 0;
        if (std::strcmp(env, "asc") == 0 || std::strcmp(env, "1") == 0)
            return 1;
        if (std::strcmp(env, "desc") == 0 || std::strcmp(env, "2") == 0)
            return 2;
        return 0;
    }();
    return mode;
}

}  // namespace

void BasicReconstructOperator::reconstruct(MCGAL::Halfedge* halfedge) {}

void BasicReconstructOperator::reconstruct(std::vector<MCGAL::Vertex*>& vertices,
                                           std::vector<MCGAL::Halfedge*>& halfedge,
                                           std::vector<MCGAL::Facet*>& facets) {
    int count = 0;
    for (MCGAL::Vertex* v : vertices) {
        v->setPoint(v->vertexPoint());
    }
    g_reconTraceMesh = mesh_.get();

    if (hasStrictPaperFacet(facets)) {
        using Clock = std::chrono::steady_clock;
        const bool profile = reconstructProfileLogs();
        auto t0 = Clock::now();
        std::unordered_map<MCGAL::Facet*, std::vector<MCGAL::Halfedge*>> diagonalsByFace;
        diagonalsByFace.reserve(halfedge.size() * 2 + 16);
        for (MCGAL::Halfedge* h : halfedge) {
            if (!h)
                continue;
            if (MCGAL::Facet* f = h->face(); f && !f->isRemoved()) {
                diagonalsByFace[f].push_back(h);
            }
            MCGAL::Halfedge* opp = h->opposite();
            if (opp) {
                if (MCGAL::Facet* f = opp->face(); f && !f->isRemoved()) {
                    diagonalsByFace[f].push_back(opp);
                }
            }
        }
        auto tIndexed = Clock::now();
        double mergeMs = 0.0;
        double insertMs = 0.0;
        for (MCGAL::Facet* f : facets) {
            if (!f || f->getProtocolHint() != 20)
                continue;
            MCGAL::Facet* root = f;
            const int targetDegree = root->getPolyValence();
            while (root && !root->isRemoved() && root->facet_degree() < targetDegree) {
                auto it = diagonalsByFace.find(root);
                if (it == diagonalsByFace.end())
                    break;
                MCGAL::Facet* oldRoot = root;
                MCGAL::Facet* mergedNeighbor = nullptr;
                auto tm = Clock::now();
                MCGAL::Facet* next = mergeOneStrictPaperDiagonal(root, it->second, &mergedNeighbor);
                if (profile) {
                    mergeMs += std::chrono::duration<double, std::milli>(Clock::now() - tm).count();
                }
                if (!next)
                    break;
                auto appendCandidates = [&](MCGAL::Facet* from) {
                    if (!from || from == next)
                        return;
                    auto srcIt = diagonalsByFace.find(from);
                    if (srcIt == diagonalsByFace.end() || srcIt->second.empty())
                        return;
                    auto& dst = diagonalsByFace[next];
                    dst.insert(dst.end(), srcIt->second.begin(), srcIt->second.end());
                };
                appendCandidates(oldRoot);
                appendCandidates(mergedNeighbor);
                root = next;
            }
            auto ti = Clock::now();
            insert_point(root);
            d18TriProbe("Insert", root);
            if (profile) {
                insertMs += std::chrono::duration<double, std::milli>(Clock::now() - ti).count();
            }
        }
        if (profile) {
            auto tDone = Clock::now();
            const double indexMs = std::chrono::duration<double, std::milli>(tIndexed - t0).count();
            const double totalMs = std::chrono::duration<double, std::milli>(tDone - t0).count();
            std::fprintf(stderr,
                         "[RECON_PROFILE] strict_paper index=%.3fms merge=%.3fms "
                         "insert=%.3fms total=%.3fms diagonals=%zu facets=%zu\n",
                         indexMs, mergeMs, insertMs, totalMs, halfedge.size(), facets.size());
        }
        return;
    }

    int collapsedFans = 0;
    if (enableFanCollapseReplay()) {
        for (MCGAL::Facet* f : facets) {
            if (!f || f->isRemoved())
                continue;
            const int targetDegree = f->getPolyValence();
            if (targetDegree > 3 && collapseV0FanToPolygon(f, targetDegree)) {
                ++collapsedFans;
            }
        }
    }
    if (collapsedFans > 0 && reconstructProfileLogs()) {
        std::fprintf(stderr, "[RECON_PROFILE] fan_collapse facets=%d\n", collapsedFans);
    }

    if (enablePatchComponentReplay()) {
        const bool profileReplay = reconstructProfileLogs();
        JoinReplayStats joinStats;
        if (profileReplay && reconReasonProfileLogs()) {
            activeJoinReplayStats = &joinStats;
        }
        int patchAttempts = 0;
        int patchJoined = 0;
        int patchInsertable = 0;
        int missingInsertionProxyLogged = 0;
        std::vector<MCGAL::Facet*> patchFacets = facets;
        const int sortMode = patchFacetSortMode();
        if (sortMode == 1) {
            std::sort(patchFacets.begin(), patchFacets.end(),
                      [](MCGAL::Facet* a, MCGAL::Facet* b) { return (a ? a->poolId() : -1) < (b ? b->poolId() : -1); });
        } else if (sortMode == 2) {
            std::sort(patchFacets.begin(), patchFacets.end(),
                      [](MCGAL::Facet* a, MCGAL::Facet* b) { return (a ? a->poolId() : -1) > (b ? b->poolId() : -1); });
        }
        for (MCGAL::Facet* f : patchFacets) {
            if (!f || f->isRemoved())
                continue;
            int guard = 0;
            while (MCGAL::Halfedge* h = findAddedEdgeOnFaceBoundary(f)) {
                ++patchAttempts;
                if (joinFaceForTopologyReplay(h)) {
                    ++patchJoined;
                } else {
                    break;
                }
                if (++guard > 100000)
                    break;
            }
            if (PMSF_CACHED_ENV("PMSF_RECON_FORCE_POLY_JOIN")) {
                int forceGuard = 0;
                while (f && !f->isRemoved() && f->getPolyValence() > f->facet_degree() &&
                       ++forceGuard < 1024) {
                    MCGAL::Halfedge* chosen = nullptr;
                    MCGAL::Halfedge* h0 = f->proxyHalfedge();
                    MCGAL::Halfedge* hh = h0; int ringGuard = 0;
                    do {
                        MCGAL::Halfedge* opp = hh ? hh->opposite() : nullptr;
                        if (opp && !opp->isRemoved() && opp->face() && opp->face() != f &&
                            !opp->face()->isRemoved()) {
                            chosen = hh;
                            break;
                        }
                        hh = hh ? hh->next() : nullptr;
                    } while (hh && hh != h0 && ++ringGuard < 64);
                    if (!chosen || !joinFaceForTopologyReplay(chosen)) break;
                }
            }
            if (profileReplay) {
                if (findInsertionProxy(f)) {
                    ++patchInsertable;
                } else if (missingInsertionProxyLogged < 8) {
                    MCGAL::Halfedge* proxy = f->proxyHalfedge();
                    std::fprintf(stderr,
                                 "[RECON_PROFILE] patch_missing_insert_proxy face=%d "
                                 "removed=%d degree=%d target=%d proxy=%d "
                                 "proxyRemoved=%d proxyFace=%d proxyOppFace=%d\n",
                                 f->poolId(), f->isRemoved() ? 1 : 0, f->facet_degree(), f->getPolyValence(), proxy ? proxy->poolId() : -1,
                                 proxy && proxy->isRemoved() ? 1 : 0, (proxy && proxy->face()) ? proxy->face()->poolId() : -1,
                                 (proxy && proxy->opposite() && proxy->opposite()->face()) ? proxy->opposite()->face()->poolId() : -1);
                    ++missingInsertionProxyLogged;
                }
            }
            insert_point(f);
        d18TriProbe("Insert", f);
            d18TriProbe("Insert", f);
        }
        if (activeJoinReplayStats == &joinStats) {
            activeJoinReplayStats = nullptr;
        }
        if (profileReplay) {
            std::fprintf(stderr,
                         "[RECON_PROFILE] patch_component halfedges=%zu attempts=%d "
                         "joined=%d facets=%zu insertable=%d\n",
                         halfedge.size(), patchAttempts, patchJoined, facets.size(), patchInsertable);
            if (reconReasonProfileLogs()) {
                int groupId = -1;
                if (!facets.empty() && facets.front())
                    groupId = facets.front()->groupId();
                std::fprintf(stderr,
                             "[RECON_REASON] group=%d success=%d keptH=%d keptG=%d "
                             "noTopoH=%d(noV=%d noEnd=%d noNext=%d noOpp=%d) "
                             "hRemoved=%d noTopoG=%d(noV=%d noEnd=%d noNext=%d noOpp=%d) "
                             "fanCollapse=%d prevMissing=%d prevMismatch=%d "
                             "endpointMissing=%d faceMissing=%d bothSplittableSkip=%d "
                             "foldedJoinSkip=%d missingKeep=%d\n",
                             groupId, joinStats.success, joinStats.keptH, joinStats.keptG, joinStats.noTopologyH, joinStats.noVertexH,
                             joinStats.noEndH, joinStats.noNextH, joinStats.noOppH, joinStats.hRemoved, joinStats.noTopologyG, joinStats.noVertexG,
                             joinStats.noEndG, joinStats.noNextG, joinStats.noOppG, joinStats.fanCollapseSkip, joinStats.prevMissing,
                             joinStats.prevMismatch, joinStats.endpointMissing, joinStats.faceMissing, joinStats.bothSplittableSkip,
                             joinStats.foldedJoinSkip, joinStats.missingKeep);
            }
        }
        return;
    }

    // [PHASE59 诊断，默认关闭] insert 前 facet 的度数直方图 + join 前/后对比。
    const bool degHist = PMSF_CACHED_ENV("PMSF_RECON_DEGREE_HIST") != nullptr;
    long splittableCombo[4] = {0, 0, 0, 0};  // 索引 = isSplittable(h) * 2 + isSplittable(g)
    auto dumpDegHist = [&](const char* tag, const std::vector<MCGAL::Facet*>& fs) {
        if (!degHist)
            return;
        int hist[32] = {0};
        int removed = 0;
        long sum = 0;
        for (MCGAL::Facet* f : fs) {
            if (!f || f->isRemoved()) {
                ++removed;
                continue;
            }
            const int d = f->facet_degree();
            if (d >= 0 && d < 32)
                ++hist[d];
            sum += d;
        }
        std::fprintf(stderr, "[RECON_DEG] %s n=%zu removed=%zu sumDeg=%ld |", tag, fs.size(), static_cast<std::size_t>(removed), sum);
        for (int d = 0; d < 12; ++d)
            std::fprintf(stderr, " %d:%d", d, hist[d]);
        std::fprintf(stderr, "\n");
        std::fflush(stderr);
    };
    dumpDegHist("before_join", facets);
    std::unordered_set<MCGAL::Facet*> facetSet;
    long survivorInSet = 0;
    if (degHist) {
        for (MCGAL::Facet* f : facets) {
            if (f && !f->isRemoved())
                facetSet.insert(f);
        }
        if (mesh_ != nullptr) {
            std::size_t marked = 0, liveF = 0;
            for (MCGAL::Facet* f : mesh_->faces()) {
                if (!f || f->isRemoved())
                    continue;
                ++liveF;
                if (f->isSplittable())
                    ++marked;
            }
            std::fprintf(stderr, "[RECON_DEG] pre_join liveF=%zu splittableFlagged=%zu inFacets=%zu\n", static_cast<std::size_t>(liveF),
                         static_cast<std::size_t>(marked), facetSet.size());
            std::fflush(stderr);
        }
    }

    if (degHist) {
        g_dbgFacetSet = &facetSet;
        g_dbgJoinKeptInSet = g_dbgJoinRemovedInSet = 0;
        g_dbgJoinNeitherInSet = g_dbgJoinBothInSet = 0;
    }
    int mergeAttempts = 0;
    int mergedFaces = 0;
    int bothSplittableMerges = 0;
    const bool profileReplay = reconstructProfileLogs();
    JoinReplayStats joinStats;
    if (profileReplay && reconReasonProfileLogs()) {
        activeJoinReplayStats = &joinStats;
    }
    // [PHASE59 诊断，默认关闭] mode 3 需要知道本轮哪些面是 patch 面。
    std::unordered_set<MCGAL::Facet*> keepPatchSet;
    if (reconKeepMode() == 3) {
        keepPatchSet.reserve(facets.size() * 2 + 16);
        for (MCGAL::Facet* f : facets) {
            if (f)
                keepPatchSet.insert(f);
        }
        g_reconPatchFacetSet = &keepPatchSet;
    }
    for (MCGAL::Halfedge* hit : halfedge) {
        if (!hit || hit->isRemoved())
            continue;
        ++mergeAttempts;
        if (profileReplay && hit->opposite() && hit->face() && hit->opposite()->face() && hit->face()->isSplittable() &&
            hit->opposite()->face()->isSplittable()) {
            ++bothSplittableMerges;
        }
        // [PHASE59 诊断，默认关闭] join 两侧的 isSplittable 组合分布。
        if (degHist && hit->face() && hit->opposite() && hit->opposite()->face()) {
            const bool hs = hit->face()->isSplittable();
            const bool gs = hit->opposite()->face()->isSplittable();
            ++splittableCombo[(hs ? 1 : 0) * 2 + (gs ? 1 : 0)];
        }
        if (degHist) {
            MCGAL::Facet* hF = hit->face();
            MCGAL::Facet* gF = hit->opposite() ? hit->opposite()->face() : nullptr;
            const bool keepG = degHist && gF && gF->isSplittable();
            MCGAL::Facet* survivor = keepG ? gF : hF;
            if (survivor && facetSet.count(survivor))
                ++survivorInSet;
        }
        if (joinFaceForTopologyReplay(hit)) {
            ++mergedFaces;
        }
        count++;
    }
    if (activeJoinReplayStats == &joinStats) {
        activeJoinReplayStats = nullptr;
    }
    g_reconPatchFacetSet = nullptr;
    if (degHist) {
        std::fprintf(stderr, "[RECON_DEG] join_set keptInSet=%ld removedInSet=%ld neither=%ld both=%ld\n", g_dbgJoinKeptInSet, g_dbgJoinRemovedInSet,
                     g_dbgJoinNeitherInSet, g_dbgJoinBothInSet);
        g_dbgFacetSet = nullptr;
    }
    dumpDegHist("after_join", facets);
    if (degHist && mesh_ != nullptr) {
        std::size_t liveF = 0;
        for (MCGAL::Facet* f : mesh_->faces()) {
            if (f && !f->isRemoved())
                ++liveF;
        }
        std::fprintf(stderr, "[RECON_DEG] after_join_liveF=%zu (joins=%d)\n", static_cast<std::size_t>(liveF), mergedFaces);
        std::fprintf(stderr, "[RECON_DEG] survivorInFacetSet=%ld / joins=%d\n", survivorInSet, mergedFaces);
        std::fprintf(stderr, "[RECON_DEG] join_splittable h1g1=%ld h1g0=%ld h0g1=%ld h0g0=%ld\n", splittableCombo[3], splittableCombo[2],
                     splittableCombo[1], splittableCombo[0]);
        std::fflush(stderr);
    }
    count = 0;
    int insertableFacets = 0;
    int missingInsertionProxyLogged = 0;
    for (MCGAL::Facet* f : facets) {
        MCGAL::Halfedge* insertionProxy = profileReplay ? findInsertionProxy(f) : nullptr;
        if (profileReplay) {
            if (insertionProxy) {
                ++insertableFacets;
            } else if (missingInsertionProxyLogged < 8) {
                MCGAL::Halfedge* proxy = f ? f->proxyHalfedge() : nullptr;
                std::fprintf(stderr,
                             "[RECON_PROFILE] missing_insert_proxy face=%d removed=%d "
                             "degree=%d target=%d proxy=%d proxyRemoved=%d "
                             "proxyFace=%d proxyOppFace=%d\n",
                             f ? f->poolId() : -1, f && f->isRemoved() ? 1 : 0, f ? f->facet_degree() : -1, f ? f->getPolyValence() : -1,
                             proxy ? proxy->poolId() : -1, proxy && proxy->isRemoved() ? 1 : 0,
                             (proxy && proxy->face()) ? proxy->face()->poolId() : -1,
                             (proxy && proxy->opposite() && proxy->opposite()->face()) ? proxy->opposite()->face()->poolId() : -1);
                ++missingInsertionProxyLogged;
            }
        }
        insert_point(f);
        d18TriProbe("Insert", f);
        count++;
    }
    if (profileReplay) {
        std::fprintf(stderr,
                     "[RECON_PROFILE] replay halfedges=%zu attempts=%d joined=%d "
                     "bothSplittable=%d facets=%zu insertable=%d\n",
                     halfedge.size(), mergeAttempts, mergedFaces, bothSplittableMerges, facets.size(), insertableFacets);
        if (reconReasonProfileLogs()) {
            int groupId = -1;
            if (!facets.empty() && facets.front()) {
                groupId = facets.front()->groupId();
            } else if (!halfedge.empty() && halfedge.front() && halfedge.front()->face()) {
                groupId = halfedge.front()->face()->groupId();
            }
            std::fprintf(stderr,
                         "[RECON_REASON] group=%d success=%d keptH=%d keptG=%d "
                         "noTopoH=%d(noV=%d noEnd=%d noNext=%d noOpp=%d) "
                         "hRemoved=%d noTopoG=%d(noV=%d noEnd=%d noNext=%d noOpp=%d) "
                         "fanCollapse=%d "
                         "prevMissing=%d prevMismatch=%d endpointMissing=%d "
                         "faceMissing=%d bothSplittableSkip=%d "
                         "foldedJoinSkip=%d missingKeep=%d\n",
                         groupId, joinStats.success, joinStats.keptH, joinStats.keptG, joinStats.noTopologyH, joinStats.noVertexH, joinStats.noEndH,
                         joinStats.noNextH, joinStats.noOppH, joinStats.hRemoved, joinStats.noTopologyG, joinStats.noVertexG, joinStats.noEndG,
                         joinStats.noNextG, joinStats.noOppG, joinStats.fanCollapseSkip, joinStats.prevMissing, joinStats.prevMismatch,
                         joinStats.endpointMissing, joinStats.faceMissing, joinStats.bothSplittableSkip, joinStats.foldedJoinSkip,
                         joinStats.missingKeep);
        }
    }
}

void BasicReconstructOperator::merge_face(MCGAL::Halfedge* halfedge) {
    if (!halfedge || halfedge->isRemoved()) {
        return;
    }
    joinFaceForTopologyReplay(halfedge);
}

/**
 * 插入中心点。
 * - 非预测路径：face 上的 removedVertexPos 已是原始浮点坐标，直接使用。
 * - 预测路径：face 上保存的是 Vector3i 残差；应用一阶 barycentric 预测：
 *       pred_barycenter = barycenterQuantized(polygon)
 *       vrPosInt       = residual + pred_barycenter
 *   再按 mesh 的 bboxMin + quantStep 反量化出浮点 Point。
 *   barycenter 的计算方式与编码端 VertexRemovalEliminateOperator::remove_point 完全对称。
 *
 * 注意：曾加过二阶曲率预测 `rdCurvaturePrediction`（论文公式 (2)），逻辑上
 * 需要邻居顶点的 r1 属性。但在串行"BFS 解码即 insert_point"的路径里，
 * 邻居的 r1 尚未被写入（都是 0），此时 pred_curvature ≡ 0，带来的只是
 * 属性访问开销与 decoder-side `rd_r1` attach 不存在时的 segfault 风险
 * （PCATM Parquet V2 解压器不走 RD/QEM 的属性注册）。故这里只保留一阶
 * barycentric 预测，与 PCATM 论文 §5 的"predicted point = mean of polygon
 * neighbors"完全对齐。若将来为 RD 引入真正的二阶 wavelet 预测，请在另一
 * 个 ReconstructOperator（例如 RDReconstructOperator）中实现，不要污染
 * BasicReconstructOperator 的通用路径。
 */
struct InsPointStats {
    long calls = 0;
    long nullFacet = 0;
    long nullProxy = 0;
    long done = 0;
    std::map<int, std::array<long, 3>> byGid;  // calls, nullProxy, done
    ~InsPointStats() {
        if (!PMSF_CACHED_ENV("PMSF_INS_STATS")) return;
        std::fprintf(stderr, "[ins-stats] calls=%ld nullFacet=%ld nullProxy=%ld done=%ld\n",
                     calls, nullFacet, nullProxy, done);
        for (const auto& kv : byGid)
            std::fprintf(stderr, "[ins-stats] gid=%d calls=%ld nullProxy=%ld done=%ld\n",
                         kv.first, kv.second[0], kv.second[1], kv.second[2]);
        std::fflush(stderr);
    }
};
static InsPointStats g_insPointStats;

void BasicReconstructOperator::insert_point(MCGAL::Facet* facet) {
    const bool insStats = PMSF_CACHED_ENV("PMSF_INS_STATS") != nullptr;
    auto statGid = [&]() -> int { return facet ? facet->groupId() : -1; };
        if (insStats) {
        ++g_insPointStats.calls;
        ++g_insPointStats.byGid[statGid()][0];
    }
    if (!facet) {
        if (insStats) ++g_insPointStats.nullFacet;
        return;
    }
    MCGAL::Halfedge* insertionProxy = findInsertionProxy(facet);
    if (!insertionProxy) {
        if (insStats) {
            ++g_insPointStats.nullProxy;
            ++g_insPointStats.byGid[statGid()][1];
        }
        return;
    }
    if (insStats) ++g_insPointStats.byGid[statGid()][2];
        if (facet->proxyHalfedge() != insertionProxy) {
        facet->proxyHalfedge(insertionProxy);
    }
    // [PHASE59 诊断，默认关闭] 每次插点的「多边形环 + 残差 + 落点」三元组。
    // 用来回答「解码端第 4 轮那 2 个编码端从未出现过的坐标是从哪个多边形算出来的」。
    // 注意：下面三条分支各自带 `return`，所以本 dump 必须由 lambda 在每条出口上调用。
    const std::size_t nvBefore = (mesh_ != nullptr) ? mesh_->vertices().size() : 0;
    auto dumpInsert = [&]() {
        // V3: register the interior-inserted vertex with its I-ID (round + forward ordinal).
        if (pmsf::bndid::dec().on && mesh_ != nullptr &&
            mesh_->vertices().size() > nvBefore)
            pmsf::bndid::decRegisterInterior(mesh_->vertices()[nvBefore]);
    };
    if (mesh_ != nullptr && facet->getProtocolHint() == 20) {
        facet->normalize_proxy_to_min_position_vertex(mesh_->bboxMin, mesh_->f_quantStep);
    }
    const char* dualRingEnv = PMSF_CACHED_ENV("PMSF_RECON_DUAL_RING");
    const bool useDualRing = dualRingEnv && *dualRingEnv && dualRingEnv[0] != '0';
    double dualRingAlpha = 0.5;
    if (const char* a = PMSF_CACHED_ENV("PMSF_RECON_DUAL_RING_ALPHA")) {
        const double v = std::strtod(a, nullptr);
        if (v > 0.0 && v <= 1.0) dualRingAlpha = v;
    }
    if (enablePrediction_ && mesh_ != nullptr) {
        if (!mesh_->enableQuantization) {
            MCGAL::Point pred = MCGAL::barycenter(facet->proxyHalfedge());
            if (useDualRing) {
                pmsf::DualRingPredictor dualPredictor(dualRingAlpha);
                pred = dualPredictor.predictForFacet(facet);
            }
            MCGAL::Point residual = facet->getRemovedVertexPos();
            MCGAL::Point p(pred.x() + residual.x(), pred.y() + residual.y(), pred.z() + residual.z());
            if (!enableSafeCenterInsert() || !createCenterVertexFromClosedRing(mesh_, facet->proxyHalfedge(), p)) {
                mesh_->create_center_vertex_without_init(facet->proxyHalfedge(), p);
            }
            if (mesh_->vertices().size() > nvBefore)
                pmsf::applyBoundaryTopologyAtVertex(mesh_, mesh_->vertices()[nvBefore],
                                                    pmsf::pendingBoundaryTopologyRound());
            dumpInsert();
            return;
        }
        MCGAL::PointInt residual = facet->getRemovedVertexPosInt();
        MCGAL::Point p;
        static const bool absRemovedPos = (PMSF_CACHED_ENV("PMSF_ABS_REMOVED_POS") != nullptr);
        if (absRemovedPos) {
            p = MCGAL::rdIntToFloatPoint(residual, mesh_->bboxMin, mesh_->f_quantStep);
        } else if (useDualRing) {
            pmsf::DualRingPredictor dualPredictor(dualRingAlpha);
            MCGAL::Point pred = dualPredictor.predictForFacet(facet);
            MCGAL::PointInt predInt = mesh_->floatPosToInt(pred);
            MCGAL::PointInt q(predInt.x() + residual.x(),
                              predInt.y() + residual.y(),
                              predInt.z() + residual.z());
            p = MCGAL::rdIntToFloatPoint(q, mesh_->bboxMin, mesh_->f_quantStep);
        } else {
            p = reconstructBarycenterPointNoAlloc(facet, residual, mesh_);
        }
        if (!enableSafeCenterInsert() || !createCenterVertexFromClosedRing(mesh_, facet->proxyHalfedge(), p)) {
            mesh_->create_center_vertex_without_init(facet->proxyHalfedge(), p);
        }
        if (mesh_->vertices().size() > nvBefore)
            pmsf::applyBoundaryTopologyAtVertex(mesh_, mesh_->vertices()[nvBefore],
                                                pmsf::pendingBoundaryTopologyRound());
        dumpInsert();
        return;
    }
    if (!enableSafeCenterInsert() || !createCenterVertexFromClosedRing(mesh_, facet->proxyHalfedge(), facet->getRemovedVertexPos())) {
        mesh_->create_center_vertex_without_init(facet->proxyHalfedge(), facet->getRemovedVertexPos());
    }
    if (mesh_->vertices().size() > nvBefore)
        pmsf::applyBoundaryTopologyAtVertex(mesh_, mesh_->vertices()[nvBefore],
                                            pmsf::pendingBoundaryTopologyRound());
    dumpInsert();
}
