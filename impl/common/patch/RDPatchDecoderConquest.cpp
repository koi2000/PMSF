// SPDX-License-Identifier: Apache-2.0
//
// RDPatchDecoderConquest.cpp — PR-M4.b (2026-05-12)
//
// 实施策略：mirror src/core/RDPatchPipeline.cpp 的 runConquest 主循环 + helper
// （pickLongestEdge / snapshotRing / pushNewGates / pushNullPatchNeighbors），
// 但 pop 后不调 IsRemovable，而是 read decodedSymbols[symIdx]。
//
// **不对称问题（M5 之前修）**：encoder forward 在 finest mesh 上选 first_gate；
// decoder 在 base mesh 上选 first_gate。两条 first_gate 不同 → priority queue
// 起点不同 → sym 序列消费偏移。表现为 round-trip V_out != V_in。
// 但 decoder 主循环本身的 mirror 逻辑在此 session 实装并验证可跑。

#include "common/patch/RDPatchDecoderConquest.h"
#include "common/BoundedRingTraversal.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <array>
#include <functional>
#include <iterator>
#include <limits>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "MeshUtils.h"  // MCGAL::barycenter
#include "Vertex.h"

#include "common/patch/RDPatchGateQueue.h"
#include "common/PCLTTMTagTable.h"  // PR-R: tag evolution helper
#include "common/PMSFRetriangulation.h"
#include "predict/RDPredictor.h"  // rdBarycenterQuantized

namespace pmsf {

namespace {

struct QKey {
    int x = 0;
    int y = 0;
    int z = 0;
    bool operator==(const QKey& o) const {
        return x == o.x && y == o.y && z == o.z;
    }
};

struct QKeyHash {
    size_t operator()(const QKey& k) const {
        uint64_t h = static_cast<uint32_t>(k.x);
        h = h * 1315423911u + static_cast<uint32_t>(k.y);
        h = h * 2654435761u + static_cast<uint32_t>(k.z);
        return static_cast<size_t>(h);
    }
};


// D17 硬化：**有界顶点环遍历**。
// MCGAL 的 `for (h : *v)` 沿 step_cw / step_ccw 绕圈，**只在走回起点时终止**
// ⇒ 环不闭合时**死循环**（实测 strict 路径有 51 个这样的顶点）。
// 本函数**逐字复刻** MCGAL 的步进逻辑（cw 优先、失败转 ccw），
// 但加 `seen` 集合 + 步数上限：**对良构的环产出与 MCGAL 完全相同的序列**
// ⇒ 默认路径字节不变；对不闭合的环**有界返回**而非挂死。
inline std::vector<MCGAL::Halfedge*> boundedVertexRing(MCGAL::Vertex* v,
                                                       int maxSteps = 256) {
    std::vector<MCGAL::Halfedge*> out;
    if (!v || v->isRemoved()) return out;
    MCGAL::Halfedge* start = v->proxyHalfedge();
    if (!start) return out;
    auto step_cw = [](MCGAL::Halfedge* h) -> MCGAL::Halfedge* {
        if (!h) return nullptr;
        MCGAL::Halfedge* n2 = h->find_prev();
        return n2 ? n2->opposite() : nullptr;
    };
    auto step_ccw = [](MCGAL::Halfedge* h) -> MCGAL::Halfedge* {
        if (!h) return nullptr;
        MCGAL::Halfedge* opp = h->opposite();
        return opp ? opp->next() : nullptr;
    };
    std::set<MCGAL::Halfedge*> seen;
    MCGAL::Halfedge* cur = start;
    bool first = true, ccw = false;
    int guard = 0;
    while (cur && guard++ < maxSteps) {
        if (!seen.insert(cur).second) break;   // 环路重复 ⇒ 不闭合，有界退出
        out.push_back(cur);
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
        if (nxt == start) break;
        cur = nxt;
    }
    return out;
}

inline bool envOn(const char* name) {
    const char* v = std::getenv(name);
    return v && *v && std::strcmp(v, "0") != 0;
}

int envInt(const char* name, int fallback) {
    const char* v = std::getenv(name);
    if (!v || !*v) return fallback;
    return std::max(1, std::atoi(v));
}

int envIntAllowZero(const char* name, int fallback) {
    const char* v = std::getenv(name);
    if (!v || !*v) return fallback;
    return std::max(0, std::atoi(v));
}

// PR-R (2026-05-12) decoder helpers (mirror encoder src/core/RDPatchPipeline.cpp)
inline RDPatchVertexTag mcgalTagToRDPatch(MCGAL::Vertex::TagFlag t) {
    switch (t) {
        case MCGAL::Vertex::Plus:  return RDPatchVertexTag::Plus;
        case MCGAL::Vertex::Minus: return RDPatchVertexTag::Minus;
        default:                   return RDPatchVertexTag::Untagged;
    }
}

inline RDPatchVertexTag tagOfVertex(MCGAL::Vertex* v) {
    if (!v) return RDPatchVertexTag::Untagged;
    return mcgalTagToRDPatch(v->getTag());
}

std::vector<MCGAL::Halfedge*> boundedVertexHalfedges(MCGAL::Vertex* v, int maxSteps);

inline int countProcessedFaces(MCGAL::Vertex* v) {
    if (!v || v->isRemoved()) return 0;
    int n = 0;
    for (MCGAL::Halfedge* he : boundedVertexHalfedges(v, 64)) {
        if (!he || he->isRemoved()) continue;
        MCGAL::Facet* f = he->face();
        if (f && !f->isRemoved() && f->isProcessed()) ++n;
    }
    return n;
}

inline bool isVertexSaturated(MCGAL::Vertex* v) {
    const int n = countProcessedFaces(v);
    static const char* envSat = std::getenv("PMSF_RDPATCH_SAT_RANGE");
    static const bool relaxed = envSat && std::strcmp(envSat, "relaxed") == 0;
    if (relaxed) return n >= 1 && n <= 4;
    return n == 2 || n == 3;
}

inline int computeRealBucket(MCGAL::Halfedge* h,
                             const std::set<MCGAL::Facet*>& nullPatchFaces,
                             bool forceSixBucket = false) {
    if (!h) return 6;
    static const char* envPRR = std::getenv("PMSF_RDPATCH_PR_R");
    static const bool prRActive = envPRR && envPRR[0] == '1';
    if (!forceSixBucket && !prRActive) {
        return 5;  // PR-Q default
    }
    MCGAL::Vertex* vSrc = h->vertex();
    MCGAL::Vertex* vDst = h->end_vertex();
    const RDPatchVertexTag tSrc = tagOfVertex(vSrc);
    const RDPatchVertexTag tDst = tagOfVertex(vDst);
    const bool satSrc = isVertexSaturated(vSrc);
    const bool satDst = isVertexSaturated(vDst);
    MCGAL::Facet* face = h->face();
    const RDPatchNullFlag nf =
        (face && nullPatchFaces.count(face)) ? RDPatchNullFlag::Null : RDPatchNullFlag::NonNull;
    return RDPatchGateQueue::computeBucket(tSrc, tDst, nf, satSrc, satDst);
}

// mirror encoder src/core/RDPatchPipeline.cpp::pickLongestEdge
MCGAL::Halfedge* pickLongestEdge(std::shared_ptr<MCGAL::Mesh> mesh) {
    float maxLen = 0.0f;
    MCGAL::Halfedge* best = nullptr;
    for (MCGAL::Facet* facet : mesh->faces()) {
        if (!facet || facet->isRemoved()) continue;
        for (MCGAL::Halfedge* he : *facet) {
            if (!he || he->isRemoved()) continue;
            const float len = he->length();
            if (len > maxLen) {
                maxLen = len;
                best = he;
            }
        }
    }
    return best;
}

// mirror encoder snapshotRing —— vertex 1-ring 顺序由 vertex iter 决定
struct RingSnapshot {
    std::vector<MCGAL::Vertex*> ringVerts;
};

RingSnapshot snapshotRing(MCGAL::Vertex* center) {
    RingSnapshot s;
    if (!center) return s;
    s.ringVerts.reserve(8);
    std::unordered_set<MCGAL::Vertex*> dedup;
    for (MCGAL::Halfedge* hit : boundedVertexRing(center)) {
        if (!hit) continue;
        MCGAL::Vertex* rv = nullptr;
        if (hit->vertex() && hit->vertex() != center) {
            rv = hit->vertex();
        } else if (hit->end_vertex() && hit->end_vertex() != center) {
            rv = hit->end_vertex();
        } else if (hit->opposite()) {
            MCGAL::Vertex* ov = hit->opposite()->vertex();
            MCGAL::Vertex* oe = hit->opposite()->end_vertex();
            rv = (ov && ov != center) ? ov : ((oe && oe != center) ? oe : nullptr);
        }
        if (rv && rv != center && dedup.insert(rv).second) {
            s.ringVerts.push_back(rv);
        }
    }
    return s;
}

// PR-R (2026-05-12) mirror encoder pushNewGates：用真实 6-bucket
void pushNewGates(const RingSnapshot& snap,
                  RDPatchGateQueue& q,
                  std::set<MCGAL::Halfedge*>& alreadyEnqueued,
                  std::set<MCGAL::Facet*>& facesEnqueued,
                  const std::set<MCGAL::Facet*>& nullPatchFaces,
                  bool forceSixBucket = false) {
    const int N = static_cast<int>(snap.ringVerts.size());
    if (N < 2) return;
    for (int i = 0; i < N; ++i) {
        MCGAL::Vertex* u = snap.ringVerts[i];
        if (!u || u->isRemoved()) continue;
        for (int j = 0; j < N; ++j) {
            if (i == j) continue;
            MCGAL::Vertex* w = snap.ringVerts[j];
            if (!w || w->isRemoved()) continue;
            MCGAL::Halfedge* h = u->find_halfedge(w);
            if (!h || h->isRemoved()) continue;
            if (alreadyEnqueued.count(h)) continue;
            MCGAL::Facet* hf = h->face();
            if (hf && facesEnqueued.count(hf)) continue;
            alreadyEnqueued.insert(h);
            if (hf) facesEnqueued.insert(hf);
            const int bucket = computeRealBucket(h, nullPatchFaces, forceSixBucket);
            q.push(h, bucket);
        }
    }
}

// PR-R (2026-05-12) mirror encoder pushNullPatchNeighbors
void pushNullPatchNeighbors(MCGAL::Halfedge* gateH,
                            RDPatchGateQueue& q,
                            std::set<MCGAL::Halfedge*>& alreadyEnqueued,
                            std::set<MCGAL::Facet*>& facesEnqueued,
                            const std::set<MCGAL::Facet*>& nullPatchFaces,
                            bool forceSixBucket = false) {
    if (!gateH || gateH->isRemoved()) return;
    MCGAL::Facet* face = gateH->face();
    if (!face || face->isRemoved()) return;
    for (MCGAL::Halfedge* he : *face) {
        if (!he || he->isRemoved()) continue;
        if (he == gateH) continue;
        MCGAL::Halfedge* nb = he->opposite();
        if (!nb || nb->isRemoved()) continue;
        if (alreadyEnqueued.count(nb)) continue;
        MCGAL::Facet* nbFace = nb->face();
        if (nbFace && facesEnqueued.count(nbFace)) continue;
        alreadyEnqueued.insert(nb);
        if (nbFace) facesEnqueued.insert(nbFace);
        const int bucket = computeRealBucket(nb, nullPatchFaces, forceSixBucket);
        q.push(nb, bucket);
    }
}

bool sameUndirectedEdge(MCGAL::Halfedge* h,
                        MCGAL::Vertex* a,
                        MCGAL::Vertex* b) {
    if (!h || !a || !b) return false;
    return (h->vertex() == a && h->end_vertex() == b) ||
           (h->vertex() == b && h->end_vertex() == a);
}

inline MCGAL::Halfedge* inputBoundaryFromGate(MCGAL::Halfedge* h) {
    if (!h || h->isRemoved() || !h->next() || !h->next()->next()) return nullptr;
    return h->next()->next();
}

inline MCGAL::Halfedge* gateFromInputBoundary(MCGAL::Halfedge* boundary) {
    if (!boundary || boundary->isRemoved() || !boundary->next()) return nullptr;
    return boundary->next();
}

std::vector<MCGAL::Halfedge*> boundedFaceHalfedges(MCGAL::Facet* f, int maxSteps = 64);

void markGateInputTags(MCGAL::Halfedge* h) {
    MCGAL::Halfedge* input = inputBoundaryFromGate(h);
    if (!input || input->isRemoved()) return;
    if (input->vertex() && !input->vertex()->isRemoved()) {
        input->vertex()->setTag(MCGAL::Vertex::Plus);
    }
    if (input->end_vertex() && !input->end_vertex()->isRemoved()) {
        input->end_vertex()->setTag(MCGAL::Vertex::Minus);
    }
}

void applyStrictNullGateTag(MCGAL::Halfedge* h) {
    if (!h || h->isRemoved() || !h->end_vertex()) return;
    MCGAL::Halfedge* input = inputBoundaryFromGate(h);
    if (!input || input->isRemoved() || !input->vertex() || !input->end_vertex()) return;
    MCGAL::Vertex* front = h->end_vertex();
    if (!front || front->isRemoved() || front->getTag() != MCGAL::Vertex::Untagged) return;
    const bool bothPlus =
        input->vertex()->getTag() == MCGAL::Vertex::Plus &&
        input->end_vertex()->getTag() == MCGAL::Vertex::Plus;
    front->setTag(bothPlus ? MCGAL::Vertex::Minus : MCGAL::Vertex::Plus);
}

void applyFig9TagsToRing(const std::vector<MCGAL::Vertex*>& ringVerts) {
    const int n = static_cast<int>(ringVerts.size());
    if (n < 3 || n > 6) return;
    const int tagLeft = ringVerts.front()
                            ? static_cast<int>(ringVerts.front()->getTag())
                            : static_cast<int>(MCGAL::Vertex::Plus);
    const int tagRight = ringVerts.back()
                             ? static_cast<int>(ringVerts.back()->getTag())
                             : static_cast<int>(MCGAL::Vertex::Minus);
    const RetriangulationResult r = paperRetriangulateFig9(n, tagLeft, tagRight);
    for (int idx : r.plusTagIdx) {
        if (idx >= 0 && idx < n && ringVerts[static_cast<size_t>(idx)]) {
            ringVerts[static_cast<size_t>(idx)]->setTag(MCGAL::Vertex::Plus);
        }
    }
    for (int idx : r.minusTagIdx) {
        if (idx >= 0 && idx < n && ringVerts[static_cast<size_t>(idx)]) {
            ringVerts[static_cast<size_t>(idx)]->setTag(MCGAL::Vertex::Minus);
        }
    }
}

void setMepp2TagIfUnset(MCGAL::Vertex* v, int tag) {
    if (!v || v->isRemoved() || v->getTag() != MCGAL::Vertex::Untagged) return;
    if (tag == static_cast<int>(MCGAL::Vertex::Plus)) {
        v->setTag(MCGAL::Vertex::Plus);
    } else if (tag == static_cast<int>(MCGAL::Vertex::Minus)) {
        v->setTag(MCGAL::Vertex::Minus);
    }
}

int mepp2TypeFromInputBoundary(MCGAL::Halfedge* input, int valence) {
    if (!input || input->isRemoved() || !input->vertex() || !input->end_vertex()) {
        return 0;
    }
    return paperMepp2TypeFromBoundaryTags(
        valence,
        static_cast<int>(input->vertex()->getTag()),
        static_cast<int>(input->end_vertex()->getTag()));
}

bool applyMepp2BoundaryTagsFromInput(MCGAL::Halfedge* input, int valence) {
    if (!input || input->isRemoved() || valence < 3 || valence > 6) return false;
    MCGAL::Facet* f = input->face();
    if (!f || f->isRemoved()) return false;
    const int type = mepp2TypeFromInputBoundary(input, valence);
    if (type == 0) return false;
    for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
        if (h && h->vertex() && !h->vertex()->isRemoved()) h->vertex()->setConquered();
    }
    MCGAL::Halfedge* g = input->next();
    for (int tag : paperMepp2InteriorTags(type)) {
        if (!g || g->isRemoved() || !g->end_vertex()) break;
        setMepp2TagIfUnset(g->end_vertex(), tag);
        g = g->next();
    }
    return true;
}

std::vector<MCGAL::Halfedge*> boundedFaceHalfedges(MCGAL::Facet* f, int maxSteps) {
    std::vector<MCGAL::Halfedge*> out;
    if (!f || f->isRemoved() || !f->proxyHalfedge()) return out;
    MCGAL::Halfedge* start = f->proxyHalfedge();
    MCGAL::Halfedge* h = start;
    int guard = 0;
    do {
        if (!h || h->isRemoved()) return {};
        out.push_back(h);
        h = h->next();
        ++guard;
    } while (h && h != start && guard < maxSteps);
    if (h != start) out.clear();
    return out;
}

MCGAL::Halfedge* boundedPrevInFace(MCGAL::Halfedge* h) {
    if (!h || h->isRemoved() || !h->face() || h->face()->isRemoved()) return nullptr;
    MCGAL::Halfedge* p = h->find_prev();
    if (!p || p->isRemoved() || p->face() != h->face()) return nullptr;
    return p;
}

MCGAL::Halfedge* faceHalfedgeBetweenOriented(MCGAL::Facet* f,
                                             MCGAL::Vertex* a,
                                             MCGAL::Vertex* b) {
    if (!f || f->isRemoved() || !a || !b) return nullptr;
    for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
        if (h && !h->isRemoved() && h->vertex() == a && h->end_vertex() == b) {
            return h;
        }
    }
    return nullptr;
}

RingSnapshot snapshotFaceRing(MCGAL::Facet* face) {
    RingSnapshot s;
    if (!face || face->isRemoved()) return s;
    for (MCGAL::Halfedge* h : boundedFaceHalfedges(face)) {
        if (h && h->vertex() && !h->vertex()->isRemoved()) {
            s.ringVerts.push_back(h->vertex());
        }
    }
    return s;
}

bool faceHasVertexOutsideSet(MCGAL::Facet* f,
                             const std::set<MCGAL::Vertex*>& verts) {
    if (!f || f->isRemoved()) return false;
    for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
        if (!h || h->isRemoved() || !h->vertex()) continue;
        if (!verts.count(h->vertex())) return true;
    }
    return false;
}

std::vector<MCGAL::Halfedge*> boundedVertexHalfedges(MCGAL::Vertex* v, int maxSteps = 64) {
    std::vector<MCGAL::Halfedge*> out;
    if (!v || v->isRemoved() || !v->proxyHalfedge()) return out;
    std::unordered_set<MCGAL::Halfedge*> seen;
    MCGAL::Halfedge* start = v->proxyHalfedge();
    MCGAL::Halfedge* cur = start;
    bool switchedToCcw = false;
    int guard = 0;
    auto stepCw = [](MCGAL::Halfedge* h) -> MCGAL::Halfedge* {
        if (!h || h->isRemoved()) return nullptr;
        MCGAL::Halfedge* prev = h->find_prev();
        if (!prev || prev->isRemoved()) return nullptr;
        MCGAL::Halfedge* opp = prev->opposite();
        return (opp && !opp->isRemoved()) ? opp : nullptr;
    };
    auto stepCcw = [](MCGAL::Halfedge* h) -> MCGAL::Halfedge* {
        if (!h || h->isRemoved()) return nullptr;
        MCGAL::Halfedge* opp = h->opposite();
        if (!opp || opp->isRemoved()) return nullptr;
        MCGAL::Halfedge* next = opp->next();
        return (next && !next->isRemoved()) ? next : nullptr;
    };
    while (cur && guard++ < maxSteps) {
        if (!seen.insert(cur).second) break;
        out.push_back(cur);
        MCGAL::Halfedge* next = switchedToCcw ? stepCcw(cur) : stepCw(cur);
        if (!next) {
            if (switchedToCcw) break;
            switchedToCcw = true;
            next = stepCcw(start);
        }
        if (!next || next == start) break;
        cur = next;
    }
    return out;
}

MCGAL::Halfedge* findHalfedgeBetweenBounded(MCGAL::Vertex* a, MCGAL::Vertex* b) {
    if (!a || !b || a->isRemoved() || b->isRemoved()) return nullptr;
    for (MCGAL::Halfedge* h : boundedVertexHalfedges(a)) {
        if (h && h->end_vertex() == b) return h;
    }
    return nullptr;
}

MCGAL::Halfedge* findExteriorHalfedgeBetween(MCGAL::Vertex* a,
                                             MCGAL::Vertex* b) {
    if (!a || !b || a->isRemoved() || b->isRemoved()) return nullptr;
    MCGAL::Halfedge* ab = findHalfedgeBetweenBounded(a, b);
    MCGAL::Halfedge* ba = findHalfedgeBetweenBounded(b, a);
    MCGAL::Halfedge* candidates[4] = {
        ab,
        ab ? ab->opposite() : nullptr,
        ba,
        ba ? ba->opposite() : nullptr,
    };
    for (MCGAL::Halfedge* h : candidates) {
        if (!h || h->isRemoved()) continue;
        MCGAL::Facet* f = h->face();
        if (f && !f->isRemoved() && !f->isProcessed()) return h;
    }
    for (MCGAL::Halfedge* h : candidates) {
        if (!h || h->isRemoved()) continue;
        MCGAL::Facet* f = h->face();
        if (f && !f->isRemoved()) return h;
    }
    return nullptr;
}

MCGAL::Halfedge* findExteriorHalfedgeBetween(MCGAL::Vertex* a,
                                             MCGAL::Vertex* b,
                                             const std::set<MCGAL::Vertex*>& patchBoundary) {
    if (!a || !b || a->isRemoved() || b->isRemoved()) return nullptr;
    MCGAL::Halfedge* ab = findHalfedgeBetweenBounded(a, b);
    MCGAL::Halfedge* ba = findHalfedgeBetweenBounded(b, a);
    MCGAL::Halfedge* candidates[4] = {
        ab,
        ab ? ab->opposite() : nullptr,
        ba,
        ba ? ba->opposite() : nullptr,
    };
    for (MCGAL::Halfedge* h : candidates) {
        if (!h || h->isRemoved()) continue;
        MCGAL::Facet* f = h->face();
        if (f && !f->isRemoved() && faceHasVertexOutsideSet(f, patchBoundary)) return h;
    }
    return findExteriorHalfedgeBetween(a, b);
}

MCGAL::Halfedge* findExteriorHalfedgeBetweenOnMesh(
    std::shared_ptr<MCGAL::Mesh> mesh,
    MCGAL::Vertex* a,
    MCGAL::Vertex* b,
    const std::set<MCGAL::Vertex*>& patchBoundary) {
    if (!mesh || !a || !b || a->isRemoved() || b->isRemoved()) return nullptr;
    MCGAL::Halfedge* fast = findExteriorHalfedgeBetween(a, b, patchBoundary);
    if (fast) return fast;
    const char* allowScanEnv = std::getenv("PMSF_RDPATCH_ALLOW_MESH_SCAN");
    if (!allowScanEnv || std::atoi(allowScanEnv) == 0) return nullptr;
    MCGAL::Halfedge* fallback = nullptr;
    MCGAL::Halfedge* unprocessedFallback = nullptr;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
            if (!h || h->isRemoved()) continue;
            if (!sameUndirectedEdge(h, a, b)) continue;
            if (!fallback) fallback = h;
            if (!f->isProcessed() && !unprocessedFallback) unprocessedFallback = h;
            if (faceHasVertexOutsideSet(f, patchBoundary)) return h;
        }
    }
    return unprocessedFallback ? unprocessedFallback : fallback;
}

void pushGateIfUsable(MCGAL::Halfedge* h,
                      RDPatchGateQueue& q,
                      std::set<MCGAL::Halfedge*>& alreadyEnqueued,
                      std::set<MCGAL::Facet*>& facesEnqueued,
                      const std::set<MCGAL::Facet*>& nullPatchFaces,
                      bool forceSixBucket = false) {
    if (!h || h->isRemoved()) return;
    MCGAL::Facet* f = h->face();
    if (!f || f->isRemoved()) return;
    if (f->isProcessed()) return;
    if (alreadyEnqueued.count(h)) return;
    if (facesEnqueued.count(f)) return;
    alreadyEnqueued.insert(h);
    facesEnqueued.insert(f);
    q.push(h, computeRealBucket(h, nullPatchFaces, forceSixBucket));
}

void pushGateFifoIfUsable(MCGAL::Halfedge* h,
                          RDPatchGateQueue& q,
                          std::set<MCGAL::Halfedge*>& alreadyEnqueued,
                          std::set<MCGAL::Facet*>& facesEnqueued) {
    if (!h || h->isRemoved()) return;
    MCGAL::Facet* f = h->face();
    if (!f || f->isRemoved()) return;
    if (f->isProcessed()) return;
    if (alreadyEnqueued.count(h)) return;
    if (facesEnqueued.count(f)) return;
    alreadyEnqueued.insert(h);
    facesEnqueued.insert(f);
    q.push(h, 5);
}

void pushPaperDecimatingGates(const std::vector<MCGAL::Vertex*>& ringVerts,
                              MCGAL::Vertex* inputA,
                              MCGAL::Vertex* inputB,
                              std::shared_ptr<MCGAL::Mesh> mesh,
                              RDPatchGateQueue& q,
                              std::set<MCGAL::Halfedge*>& alreadyEnqueued,
                              std::set<MCGAL::Facet*>& facesEnqueued,
                              const std::set<MCGAL::Facet*>& nullPatchFaces,
                              bool forceSixBucket = false) {
    const int n = static_cast<int>(ringVerts.size());
    if (n < 3) return;
    std::set<MCGAL::Vertex*> boundary;
    for (MCGAL::Vertex* v : ringVerts) {
        if (v && !v->isRemoved()) boundary.insert(v);
    }
    for (int i = 0; i < n; ++i) {
        MCGAL::Vertex* a = ringVerts[static_cast<size_t>(i)];
        MCGAL::Vertex* b = ringVerts[static_cast<size_t>((i + 1) % n)];
        if (!a || !b || a->isRemoved() || b->isRemoved()) continue;
        if ((a == inputA && b == inputB) || (a == inputB && b == inputA)) continue;
        pushGateIfUsable(gateFromInputBoundary(
                             findExteriorHalfedgeBetweenOnMesh(mesh, a, b, boundary)),
                         q, alreadyEnqueued, facesEnqueued, nullPatchFaces, forceSixBucket);
    }
}

std::vector<MCGAL::Halfedge*> collectPaperDecimatingGatesFromPolygon(
    MCGAL::Facet* patchFace,
    MCGAL::Vertex* inputA,
    MCGAL::Vertex* inputB) {
    std::vector<MCGAL::Halfedge*> out;
    if (!patchFace || patchFace->isRemoved()) return out;
    for (MCGAL::Halfedge* boundary : boundedFaceHalfedges(patchFace)) {
        if (!boundary || boundary->isRemoved()) continue;
        MCGAL::Vertex* a = boundary->vertex();
        MCGAL::Vertex* b = boundary->end_vertex();
        if (!a || !b || a->isRemoved() || b->isRemoved()) continue;
        if ((a == inputA && b == inputB) || (a == inputB && b == inputA)) continue;
        MCGAL::Halfedge* exteriorBoundary = boundary->opposite();
        if (!exteriorBoundary || exteriorBoundary->isRemoved()) continue;
        MCGAL::Facet* exteriorFace = exteriorBoundary->face();
        if (!exteriorFace || exteriorFace->isRemoved() || exteriorFace->isProcessed()) continue;
        MCGAL::Halfedge* gate = gateFromInputBoundary(exteriorBoundary);
        if (gate && !gate->isRemoved()) out.push_back(gate);
    }
    return out;
}

void pushPaperCleaningGates(const std::vector<MCGAL::Vertex*>& ringVerts,
                            std::shared_ptr<MCGAL::Mesh> mesh,
                            RDPatchGateQueue& q,
                            std::set<MCGAL::Halfedge*>& alreadyEnqueued,
                            std::set<MCGAL::Facet*>& facesEnqueued,
                            std::set<MCGAL::Facet*>& facesProcessedInPhase,
                            const std::set<MCGAL::Facet*>& nullPatchFaces,
                            bool forceSixBucket = false) {
    const int n = static_cast<int>(ringVerts.size());
    if (n < 3) return;
    std::set<MCGAL::Vertex*> boundary;
    for (MCGAL::Vertex* v : ringVerts) {
        if (v && !v->isRemoved()) boundary.insert(v);
    }
    for (int i = 0; i < n; ++i) {
        MCGAL::Vertex* a = ringVerts[static_cast<size_t>(i)];
        MCGAL::Vertex* b = ringVerts[static_cast<size_t>((i + 1) % n)];
        if (!a || !b || a->isRemoved() || b->isRemoved()) continue;
        MCGAL::Halfedge* exteriorGate = findExteriorHalfedgeBetweenOnMesh(mesh, a, b, boundary);
        MCGAL::Facet* exterior = exteriorGate ? exteriorGate->face() : nullptr;
        if (!exterior || exterior->isRemoved()) continue;
        exterior->setProcessedFlag();
        facesProcessedInPhase.insert(exterior);
        for (MCGAL::Halfedge* he : boundedFaceHalfedges(exterior)) {
            if (!he || he->isRemoved() || he == exteriorGate) continue;
            pushGateIfUsable(gateFromInputBoundary(he->opposite()),
                             q, alreadyEnqueued, facesEnqueued, nullPatchFaces, forceSixBucket);
        }
    }
}

void pushPaperNullPatchNeighbors(MCGAL::Halfedge* gateH,
                                 RDPatchGateQueue& q,
                                 std::set<MCGAL::Halfedge*>& alreadyEnqueued,
                                 std::set<MCGAL::Facet*>& facesEnqueued,
                                 const std::set<MCGAL::Facet*>& nullPatchFaces,
                                 bool forceSixBucket = false) {
    if (!gateH || gateH->isRemoved()) return;
    MCGAL::Facet* face = gateH->face();
    if (!face || face->isRemoved()) return;
    MCGAL::Halfedge* input = inputBoundaryFromGate(gateH);
    if (!input || input->isRemoved() || input->face() != face) return;
    MCGAL::Halfedge* he = input->next();
    int guard = 0;
    while (he && he != input && guard++ < 8) {
        if (he->isRemoved()) {
            he = he->next();
            continue;
        }
        pushGateIfUsable(gateFromInputBoundary(he->opposite()),
                         q, alreadyEnqueued, facesEnqueued, nullPatchFaces, forceSixBucket);
        he = he->next();
    }
}

void seedAllLivingFaces(std::shared_ptr<MCGAL::Mesh> mesh,
                        RDPatchGateQueue& q,
                        std::set<MCGAL::Halfedge*>& alreadyEnqueued,
                        std::set<MCGAL::Facet*>& facesEnqueued,
                        const std::set<MCGAL::Facet*>& nullPatchFaces,
                        bool forceSixBucket = false) {
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || facesEnqueued.count(f)) continue;
        MCGAL::Halfedge* best = nullptr;
        for (MCGAL::Halfedge* h : *f) {
            if (!h || h->isRemoved()) continue;
            best = h;
            break;
        }
        if (!best || alreadyEnqueued.count(best)) continue;
        alreadyEnqueued.insert(best);
        facesEnqueued.insert(f);
        q.push(best, computeRealBucket(best, nullPatchFaces, forceSixBucket));
    }
}

int livingVertexCount(std::shared_ptr<MCGAL::Mesh> m) {
    int n = 0;
    for (MCGAL::Vertex* v : m->vertices()) {
        if (v && !v->isRemoved()) ++n;
    }
    return n;
}

int livingFaceCount(std::shared_ptr<MCGAL::Mesh> m) {
    int n = 0;
    if (!m) return n;
    for (MCGAL::Facet* f : m->faces()) {
        if (f && !f->isRemoved()) ++n;
    }
    return n;
}

void resetLivingFaceProcessedFlags(std::shared_ptr<MCGAL::Mesh> mesh) {
    if (!mesh) return;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (f && !f->isRemoved()) f->setUnProcessed();
    }
}

void resetLivingVertexConqueredFlags(std::shared_ptr<MCGAL::Mesh> mesh) {
    if (!mesh) return;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (v && !v->isRemoved()) v->setUnConquered();
    }
}

// 闭环预测可行性实测用累加器（风险 3）：闭环残差 = vQuant − decBaryQ。
struct ClResidAccum {
    long long n = 0;
    long long sumAbs = 0;
    long long varintBytes = 0;
    int maxAbs = 0;
};
ClResidAccum g_clResidAccum;
long long g_absqPathCount = 0;
long long g_residFallbackCount = 0;
long long g_posMismatch = 0;
long long g_ccvSiteA = 0;  // insertDecodedVertexOnFace:1485
long long g_ccvSiteB = 0;  // gate loop gates=0
long long g_ccvSiteC = 0;  // gate loop gates=1

double triArea2(const MCGAL::Point& a, const MCGAL::Point& b, const MCGAL::Point& c) {
    const double abx = static_cast<double>(b.x()) - a.x();
    const double aby = static_cast<double>(b.y()) - a.y();
    const double abz = static_cast<double>(b.z()) - a.z();
    const double acx = static_cast<double>(c.x()) - a.x();
    const double acy = static_cast<double>(c.y()) - a.y();
    const double acz = static_cast<double>(c.z()) - a.z();
    const double cx = aby * acz - abz * acy;
    const double cy = abz * acx - abx * acz;
    const double cz = abx * acy - aby * acx;
    return std::sqrt(cx * cx + cy * cy + cz * cz);
}

MCGAL::Point safeInteriorInsertPoint(MCGAL::Halfedge* faceProxy,
                                     const MCGAL::Point& proposed) {
    if (!faceProxy) return proposed;
    std::vector<MCGAL::Point> pts;
    MCGAL::Halfedge* cur = faceProxy;
    do {
        if (cur && cur->vertex() && !cur->vertex()->isRemoved()) {
            pts.push_back(cur->vertex()->point());
        }
        cur = cur ? cur->next() : nullptr;
    } while (cur && cur != faceProxy);
    if (pts.size() < 3) return proposed;

    bool nearBoundary = false;
    constexpr double kArea2Eps = 1e-18;
    constexpr double kDist2Eps = 1e-20;
    for (size_t i = 0; i < pts.size(); ++i) {
        const MCGAL::Point& a = pts[i];
        const MCGAL::Point& b = pts[(i + 1) % pts.size()];
        if (triArea2(a, b, proposed) < kArea2Eps) {
            nearBoundary = true;
            break;
        }
        const double dx = static_cast<double>(proposed.x()) - a.x();
        const double dy = static_cast<double>(proposed.y()) - a.y();
        const double dz = static_cast<double>(proposed.z()) - a.z();
        if (dx * dx + dy * dy + dz * dz < kDist2Eps) {
            nearBoundary = true;
            break;
        }
    }
    if (!nearBoundary) return proposed;

    MCGAL::Point bary = MCGAL::barycenter(faceProxy);
    constexpr double kPullToBary = 0.01;
    return MCGAL::Point(
        static_cast<float>((1.0 - kPullToBary) * proposed.x() + kPullToBary * bary.x()),
        static_cast<float>((1.0 - kPullToBary) * proposed.y() + kPullToBary * bary.y()),
        static_cast<float>((1.0 - kPullToBary) * proposed.z() + kPullToBary * bary.z()));
}

void nudgeVertexOutOfLine(MCGAL::Vertex* v,
                          const MCGAL::Point& a,
                          const MCGAL::Point& b) {
    if (!v) return;
    double ex = static_cast<double>(b.x()) - a.x();
    double ey = static_cast<double>(b.y()) - a.y();
    double ez = static_cast<double>(b.z()) - a.z();
    double ax = 1.0, ay = 0.0, az = 0.0;
    if (std::fabs(ex) > std::fabs(ey) && std::fabs(ex) > std::fabs(ez)) {
        ax = 0.0; ay = 1.0; az = 0.0;
    }
    double nx = ey * az - ez * ay;
    double ny = ez * ax - ex * az;
    double nz = ex * ay - ey * ax;
    double len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (len < 1e-20) {
        nx = 0.0; ny = 0.0; nz = 1.0; len = 1.0;
    }
    constexpr double kNudge = 5e-5;
    v->setPoint(static_cast<float>(v->x() + kNudge * nx / len),
                static_cast<float>(v->y() + kNudge * ny / len),
                static_cast<float>(v->z() + kNudge * nz / len));
}

int repairDegenerateFaces(std::shared_ptr<MCGAL::Mesh> mesh) {
    // 归因验证开关（门控，默认关）。用途：判定 26 轮下 A2 差的那 6 个顶点
    // 是否由本函数挪动所致。
    // **结论：不是。** 实测关掉后 A2 仍是 34828/34834（与开着一模一样），
    // 且两种情况下 MeshAudit 都是 zero_area_faces=0 —— 即本函数在这些运行里
    // 根本没修过任何东西。保留该开关是为了让后来者能一键复验这条否定结论。
    if (envOn("PMSF_RDPATCH_NO_DEGEN_REPAIR")) return 0;
    int totalRepaired = 0;
    for (int pass = 0; pass < 6; ++pass) {
        int repairedThisPass = 0;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
            std::vector<MCGAL::Vertex*> verts;
            MCGAL::Halfedge* h = f->proxyHalfedge();
            MCGAL::Halfedge* cur = h;
            do {
                if (cur && cur->vertex() && !cur->vertex()->isRemoved()) {
                    verts.push_back(cur->vertex());
                }
                cur = cur ? cur->next() : nullptr;
            } while (cur && cur != h);
            if (verts.size() < 3) continue;
            bool bad = false;
            for (size_t i = 1; i + 1 < verts.size(); ++i) {
                if (triArea2(verts[0]->point(), verts[i]->point(), verts[i + 1]->point()) < 1e-10) {
                    bad = true;
                    nudgeVertexOutOfLine(verts[i + 1], verts[0]->point(), verts[i]->point());
                }
            }
            if (bad) ++repairedThisPass;
        }
        totalRepaired += repairedThisPass;
        if (repairedThisPass == 0) break;
    }
    return totalRepaired;
}

int symbolValence(const RDPatchSymbol& s) {
    switch (s.kind) {
        case RDPatchSymbolKind::V_3: return 3;
        case RDPatchSymbolKind::V_4: return 4;
        case RDPatchSymbolKind::V_5: return 5;
        case RDPatchSymbolKind::V_6: return 6;
        case RDPatchSymbolKind::V_OTHER: return static_cast<int>(s.escape_valence);
        case RDPatchSymbolKind::NULL_PATCH: return -1;
    }
    return -1;
}

int faceDegree(MCGAL::Facet* f) {
    if (!f || f->isRemoved() || !f->proxyHalfedge()) return 0;
    int n = 0;
    MCGAL::Halfedge* start = f->proxyHalfedge();
    MCGAL::Halfedge* h = start;
    do {
        if (!h || h->isRemoved()) return 0;
        ++n;
        h = h->next();
    } while (h && h != start && n < 256);
    return (h == start) ? n : 0;
}

struct PatchCandidate {
    std::vector<MCGAL::Facet*> faces;
    std::vector<MCGAL::Halfedge*> internalEdges;
    std::vector<MCGAL::Halfedge*> boundaryEdges;
};

bool validateTriangulatedPatch(const std::vector<MCGAL::Facet*>& faces,
                               int valence,
                               PatchCandidate& out) {
    if (valence < 3 || valence > 8) return false;
    if (static_cast<int>(faces.size()) != valence - 2) return false;
    std::set<MCGAL::Facet*> faceSet(faces.begin(), faces.end());
    if (static_cast<int>(faceSet.size()) != valence - 2) return false;

    std::set<MCGAL::Vertex*> boundaryVerts;
    std::vector<MCGAL::Halfedge*> boundaryEdges;
    std::vector<MCGAL::Halfedge*> internalEdges;

    for (MCGAL::Facet* f : faces) {
        if (!f || f->isRemoved() || faceDegree(f) != 3) return false;
        MCGAL::Halfedge* start = f->proxyHalfedge();
        MCGAL::Halfedge* h = start;
        int guard = 0;
        do {
            if (!h || h->isRemoved() || !h->vertex() || !h->end_vertex()) return false;
            MCGAL::Halfedge* opp = h->opposite();
            MCGAL::Facet* of = opp ? opp->face() : nullptr;
            if (of && faceSet.count(of)) {
                if (reinterpret_cast<std::uintptr_t>(h) <
                    reinterpret_cast<std::uintptr_t>(opp)) {
                    internalEdges.push_back(h);
                }
            } else {
                boundaryEdges.push_back(h);
                boundaryVerts.insert(h->vertex());
                boundaryVerts.insert(h->end_vertex());
            }
            h = h->next();
            ++guard;
        } while (h && h != start && guard < 16);
        if (h != start) return false;
    }

    if (static_cast<int>(boundaryEdges.size()) != valence) return false;
    if (static_cast<int>(boundaryVerts.size()) != valence) return false;
    if (static_cast<int>(internalEdges.size()) != valence - 3) return false;
    for (MCGAL::Vertex* v : boundaryVerts) {
        int incidentBoundary = 0;
        for (MCGAL::Halfedge* e : boundaryEdges) {
            if (e->vertex() == v || e->end_vertex() == v) ++incidentBoundary;
        }
        if (incidentBoundary != 2) return false;
    }

    out.faces = faces;
    out.internalEdges = std::move(internalEdges);
    out.boundaryEdges = std::move(boundaryEdges);
    return true;
}

bool qLess(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Vertex* a, MCGAL::Vertex* b) {
    if (!mesh || !a || !b) return a < b;
    MCGAL::PointInt qa = mesh->floatPosToInt(a->point());
    MCGAL::PointInt qb = mesh->floatPosToInt(b->point());
    if (qa[0] != qb[0]) return qa[0] < qb[0];
    if (qa[1] != qb[1]) return qa[1] < qb[1];
    return qa[2] < qb[2];
}

std::vector<MCGAL::Vertex*> orderedPatchBoundaryRing(const PatchCandidate& patch) {
    std::vector<MCGAL::Vertex*> ring;
    const int n = static_cast<int>(patch.boundaryEdges.size());
    if (n < 3) return ring;
    std::vector<char> used(static_cast<size_t>(n), 0);
    MCGAL::Halfedge* first = patch.boundaryEdges.front();
    if (!first || !first->vertex() || !first->end_vertex()) return {};
    ring.push_back(first->vertex());
    MCGAL::Vertex* cur = first->end_vertex();
    used[0] = 1;
    for (int step = 1; step < n; ++step) {
        ring.push_back(cur);
        bool advanced = false;
        for (int i = 1; i < n; ++i) {
            if (used[static_cast<size_t>(i)]) continue;
            MCGAL::Halfedge* e = patch.boundaryEdges[static_cast<size_t>(i)];
            if (!e || !e->vertex() || !e->end_vertex()) continue;
            if (e->vertex() == cur) {
                cur = e->end_vertex();
                used[static_cast<size_t>(i)] = 1;
                advanced = true;
                break;
            }
        }
        if (!advanced) return {};
    }
    return (cur == ring.front()) ? ring : std::vector<MCGAL::Vertex*>{};
}

void rotateRingToMinQuant(std::shared_ptr<MCGAL::Mesh> mesh,
                          std::vector<MCGAL::Vertex*>& ring) {
    if (ring.empty()) return;
    size_t best = 0;
    for (size_t i = 1; i < ring.size(); ++i) {
        if (qLess(mesh, ring[i], ring[best])) best = i;
    }
    std::rotate(ring.begin(), ring.begin() + static_cast<std::ptrdiff_t>(best), ring.end());
}

int patchInputOffset(std::shared_ptr<MCGAL::Mesh> mesh,
                     const PatchCandidate& patch,
                     MCGAL::Vertex* inputA,
                     MCGAL::Vertex* inputB) {
    if (!inputA || !inputB) return -1;
    std::vector<MCGAL::Vertex*> ring = orderedPatchBoundaryRing(patch);
    if (ring.empty()) return -1;
    rotateRingToMinQuant(mesh, ring);
    const int n = static_cast<int>(ring.size());
    for (int i = 0; i < n; ++i) {
        MCGAL::Vertex* a = ring[static_cast<size_t>(i)];
        MCGAL::Vertex* b = ring[static_cast<size_t>((i + 1) % n)];
        if (a == inputA && b == inputB) return i & 0x7;
        if (a == inputB && b == inputA) return (i & 0x7) | 0x8;
    }
    return -1;
}

bool patchMatchesPaperDiagonals(std::shared_ptr<MCGAL::Mesh> mesh,
                                const PatchCandidate& patch,
                                int valence,
                                bool strictRD) {
    if (!strictRD || valence < 4 || valence > 6) return true;
    std::vector<MCGAL::Vertex*> ring = orderedPatchBoundaryRing(patch);
    if (static_cast<int>(ring.size()) != valence) return false;
    rotateRingToMinQuant(mesh, ring);

    const int tagLeft = ring.front()
                            ? static_cast<int>(ring.front()->getTag())
                            : static_cast<int>(MCGAL::Vertex::Plus);
    const int tagRight = ring.back()
                             ? static_cast<int>(ring.back()->getTag())
                             : static_cast<int>(MCGAL::Vertex::Minus);
    const RetriangulationResult expected = paperRetriangulateFig9(valence, tagLeft, tagRight);
    if (static_cast<int>(expected.diagonals.size()) != valence - 3) return false;

    auto ringIndex = [&](MCGAL::Vertex* v) -> int {
        for (int i = 0; i < valence; ++i) {
            if (ring[static_cast<std::size_t>(i)] == v) return i;
        }
        return -1;
    };
    std::set<std::pair<int, int>> actual;
    for (MCGAL::Halfedge* h : patch.internalEdges) {
        if (!h || h->isRemoved() || !h->vertex() || !h->end_vertex()) continue;
        int a = ringIndex(h->vertex());
        int b = ringIndex(h->end_vertex());
        if (a < 0 || b < 0) return false;
        if (a > b) std::swap(a, b);
        actual.insert({a, b});
    }
    std::set<std::pair<int, int>> want;
    for (const auto& d : expected.diagonals) {
        int a = d[0];
        int b = d[1];
        if (a > b) std::swap(a, b);
        want.insert({a, b});
    }
    return actual == want;
}

bool findPatchContainingFace(MCGAL::Facet* start,
                             int valence,
                             PatchCandidate& out,
                             std::shared_ptr<MCGAL::Mesh> mesh = nullptr,
                             MCGAL::Vertex* inputA = nullptr,
                             MCGAL::Vertex* inputB = nullptr,
                             int desiredInputOffset = -1,
                             bool strictRD = false) {
    if (!start || start->isRemoved() || valence < 3 || valence > 8) return false;
    if (valence == 3) {
        return validateTriangulatedPatch({start}, valence, out);
    }

    const int targetFaces = valence - 2;
    std::vector<MCGAL::Facet*> pool;
    std::set<MCGAL::Facet*> seen;
    auto addFace = [&](MCGAL::Facet* f) {
        if (!f || f->isRemoved() || f == start || faceDegree(f) != 3) return;
        if (seen.insert(f).second && pool.size() < 16) pool.push_back(f);
    };
    auto addNeighbors = [&](MCGAL::Facet* f) {
        if (!f || f->isRemoved()) return;
        for (MCGAL::Halfedge* h : *f) {
            if (!h || h->isRemoved() || !h->opposite()) continue;
            addFace(h->opposite()->face());
        }
    };

    addNeighbors(start);
    const size_t oneRingCount = pool.size();
    if (desiredInputOffset < 0) {
        for (size_t i = 0; i < oneRingCount && pool.size() < 16; ++i) {
            addNeighbors(pool[i]);
        }
    }
    bool haveFallback = false;
    PatchCandidate fallback;
    auto acceptCandidate = [&](const std::vector<MCGAL::Facet*>& cand) {
        PatchCandidate pc;
        if (!validateTriangulatedPatch(cand, valence, pc)) return false;
        if (envOn("PMSF_RDPATCH_DECODER_DEBUG_FIRST")) {
            std::vector<MCGAL::Vertex*> ring = orderedPatchBoundaryRing(pc);
            rotateRingToMinQuant(mesh, ring);
            std::cerr << "      candidate valence=" << valence << " ring=";
            for (size_t ri = 0; ri < ring.size(); ++ri) {
                MCGAL::PointInt q = mesh ? mesh->floatPosToInt(ring[ri]->point()) : MCGAL::PointInt();
                if (ri) std::cerr << ";";
                std::cerr << q[0] << "," << q[1] << "," << q[2];
            }
            std::cerr << " diag=" << (patchMatchesPaperDiagonals(mesh, pc, valence, strictRD) ? 1 : 0)
                      << "\n";
        }
        if (!patchMatchesPaperDiagonals(mesh, pc, valence, strictRD)) return false;
        const int off = (inputA && inputB) ? patchInputOffset(mesh, pc, inputA, inputB) : -1;
        if (inputA && inputB && off < 0) {
            return false;
        }
        if (!haveFallback) {
            fallback = pc;
            haveFallback = true;
        }
        if (desiredInputOffset < 0) {
            out = pc;
            return true;
        }
        if (envOn("PMSF_RDPATCH_DECODER_DEBUG_FIRST")) {
            std::vector<MCGAL::Vertex*> ring = orderedPatchBoundaryRing(pc);
            rotateRingToMinQuant(mesh, ring);
            std::cerr << "      patch_candidate desired=" << (desiredInputOffset & 0x0f)
                      << " off=" << off << " ring=";
            for (size_t ri = 0; ri < ring.size(); ++ri) {
                MCGAL::PointInt q = mesh ? mesh->floatPosToInt(ring[ri]->point()) : MCGAL::PointInt();
                if (ri) std::cerr << ";";
                std::cerr << q[0] << "," << q[1] << "," << q[2];
            }
            std::cerr << "\n";
        }
        if (off == (desiredInputOffset & 0x0f)) {
            out = pc;
            return true;
        }
        return false;
    };

    if (targetFaces == 2) {
        for (MCGAL::Facet* a : pool) {
            std::vector<MCGAL::Facet*> cand{start, a};
            if (acceptCandidate(cand)) return true;
        }
        if (haveFallback) out = fallback;
        return haveFallback;
    }
    if (targetFaces == 3) {
        for (size_t i = 0; i < pool.size(); ++i) {
            for (size_t j = i + 1; j < pool.size(); ++j) {
                std::vector<MCGAL::Facet*> cand{start, pool[i], pool[j]};
                if (acceptCandidate(cand)) return true;
            }
        }
        if (haveFallback) out = fallback;
        return haveFallback;
    }
    if (targetFaces == 4) {
        for (size_t i = 0; i < pool.size(); ++i) {
            for (size_t j = i + 1; j < pool.size(); ++j) {
                for (size_t k = j + 1; k < pool.size(); ++k) {
                    std::vector<MCGAL::Facet*> cand{start, pool[i], pool[j], pool[k]};
                    if (acceptCandidate(cand)) return true;
                }
            }
        }
        if (haveFallback) out = fallback;
        return haveFallback;
    }
    return false;
}

MCGAL::Facet* mergePatchToPolygon(const PatchCandidate& patch,
                                  int valence) {
    if (patch.faces.empty()) return nullptr;
    MCGAL::Facet* poly = patch.faces.front();
    for (int joins = 0; joins < valence - 3; ++joins) {
        MCGAL::Halfedge* best = nullptr;
        int bestScore = 1000000;
        for (MCGAL::Halfedge* h : patch.internalEdges) {
            if (!h || h->isRemoved() || !h->opposite() || h->opposite()->isRemoved()) {
                continue;
            }
            MCGAL::Facet* fa = h->face();
            MCGAL::Facet* fb = h->opposite()->face();
            if (!fa || !fb || fa->isRemoved() || fb->isRemoved()) continue;
            const int da = faceDegree(fa);
            const int db = faceDegree(fb);
            if (da <= 0 || db <= 0) continue;
            const int score = std::min(da, db) * 100 + std::max(da, db);
            if (score < bestScore) {
                bestScore = score;
                best = h;
            }
        }
        if (!best) break;
        MCGAL::Halfedge* merged = MCGAL::join_face(best);
        if (merged && merged->face() && !merged->face()->isRemoved()) {
            poly = merged->face();
        }
    }
    if (poly && !poly->isRemoved() && faceDegree(poly) == valence) return poly;
    for (MCGAL::Facet* f : patch.faces) {
        if (f && !f->isRemoved() && faceDegree(f) == valence) return f;
    }
    return nullptr;
}

MCGAL::Facet* mergePatchForSymbol(MCGAL::Facet* gateFace,
                                  const RDPatchSymbol& sym,
                                  bool force = false,
                                  std::shared_ptr<MCGAL::Mesh> mesh = nullptr,
                                  MCGAL::Vertex* inputA = nullptr,
                                  MCGAL::Vertex* inputB = nullptr,
                                  int desiredInputOffset = -1,
                                  bool strictRD = false) {
    if (!force && !envOn("PMSF_RDPATCH_DECODER_PATCH_MERGE")) return gateFace;
    const int valence = symbolValence(sym);
    if (!gateFace || gateFace->isRemoved() || valence < 3) return gateFace;
    if (valence == 3) return gateFace;

    std::vector<MCGAL::Facet*> starts;
    starts.push_back(gateFace);
    if (inputA && inputB) {
        MCGAL::Halfedge* ab = findHalfedgeBetweenBounded(inputA, inputB);
        MCGAL::Halfedge* ba = findHalfedgeBetweenBounded(inputB, inputA);
        MCGAL::Halfedge* candidates[4] = {
            ab,
            ab ? ab->opposite() : nullptr,
            ba,
            ba ? ba->opposite() : nullptr,
        };
        for (MCGAL::Halfedge* h : candidates) {
            MCGAL::Facet* f = h ? h->face() : nullptr;
            if (!f || f->isRemoved()) continue;
            bool exists = false;
            for (MCGAL::Facet* s : starts) {
                if (s == f) {
                    exists = true;
                    break;
                }
            }
            if (!exists) starts.push_back(f);
        }
    }
    PatchCandidate patch;
    bool found = false;
    for (MCGAL::Facet* start : starts) {
        if (findPatchContainingFace(start, valence, patch,
                                    mesh, inputA, inputB, desiredInputOffset,
                                    strictRD)) {
            found = true;
            break;
        }
    }
    if (!found) return gateFace;
    MCGAL::Facet* poly = mergePatchToPolygon(patch, valence);
    return (poly && !poly->isRemoved()) ? poly : gateFace;
}

MCGAL::Facet* firstUsableFace(std::shared_ptr<MCGAL::Mesh> mesh) {
    if (!mesh) return nullptr;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (f && !f->isRemoved() && f->proxyHalfedge()) return f;
    }
    return nullptr;
}

int livingFaceRank(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Facet* target) {
    if (!mesh || !target || target->isRemoved()) return -1;
    int rank = 0;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        if (f == target) return rank;
        ++rank;
    }
    return -1;
}

bool vertexMatchesQ(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Vertex* v,
                    int qx, int qy, int qz) {
    if (!mesh || !v || v->isRemoved()) return false;
    MCGAL::PointInt q = mesh->floatPosToInt(v->point());
    return q[0] == qx && q[1] == qy && q[2] == qz;
}

MCGAL::Halfedge* findHalfedgeByInputQ(std::shared_ptr<MCGAL::Mesh> mesh,
                                      const std::vector<int>& q) {
    if (!mesh || q.size() < 6) return nullptr;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
            if (!h || h->isRemoved()) continue;
            if (vertexMatchesQ(mesh, h->vertex(), q[0], q[1], q[2]) &&
                vertexMatchesQ(mesh, h->end_vertex(), q[3], q[4], q[5])) {
                return h;
            }
        }
    }
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
            if (!h || h->isRemoved()) continue;
            if (vertexMatchesQ(mesh, h->vertex(), q[3], q[4], q[5]) &&
                vertexMatchesQ(mesh, h->end_vertex(), q[0], q[1], q[2])) {
                return h;
            }
        }
    }
    return nullptr;
}

MCGAL::Halfedge* findHalfedgeByInputQ(std::shared_ptr<MCGAL::Mesh> mesh,
                                      const std::array<int, 6>& q) {
    if (!mesh) return nullptr;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
            if (!h || h->isRemoved()) continue;
            if (vertexMatchesQ(mesh, h->vertex(), q[0], q[1], q[2]) &&
                vertexMatchesQ(mesh, h->end_vertex(), q[3], q[4], q[5])) {
                return h;
            }
        }
    }
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
            if (!h || h->isRemoved()) continue;
            if (vertexMatchesQ(mesh, h->vertex(), q[3], q[4], q[5]) &&
                vertexMatchesQ(mesh, h->end_vertex(), q[0], q[1], q[2])) {
                return h;
            }
        }
    }
    return nullptr;
}

bool gateMatchesInputQ(std::shared_ptr<MCGAL::Mesh> mesh,
                       MCGAL::Halfedge* gate,
                       const std::array<int, 6>& q) {
    if (!mesh || !gate || gate->isRemoved()) return false;
    MCGAL::Halfedge* input = inputBoundaryFromGate(gate);
    if (!input || input->isRemoved()) return false;
    return vertexMatchesQ(mesh, input->vertex(), q[0], q[1], q[2]) &&
           vertexMatchesQ(mesh, input->end_vertex(), q[3], q[4], q[5]);
}

MCGAL::Halfedge* popGateByInputQ(RDPatchGateQueue& queue,
                                 std::shared_ptr<MCGAL::Mesh> mesh,
                                 const std::array<int, 6>& q,
                                 int* outBucket = nullptr) {
    std::vector<std::pair<int, MCGAL::Halfedge*>> stash;
    stash.reserve(queue.size());
    int matchedBucket = -1;
    MCGAL::Halfedge* matched = nullptr;
    while (!queue.empty()) {
        int bucket = -1;
        MCGAL::Halfedge* h = queue.popHighestPriority(&bucket);
        if (!h) continue;
        if (!matched && gateMatchesInputQ(mesh, h, q)) {
            matched = h;
            matchedBucket = bucket;
            continue;
        }
        stash.emplace_back(bucket, h);
    }
    for (const auto& it : stash) {
        queue.push(it.second, it.first);
    }
    if (outBucket) *outBucket = matchedBucket;
    return matched;
}

bool samePointApprox(const MCGAL::Point& a, const MCGAL::Point& b) {
    const double dx = static_cast<double>(a.x()) - b.x();
    const double dy = static_cast<double>(a.y()) - b.y();
    const double dz = static_cast<double>(a.z()) - b.z();
    return dx * dx + dy * dy + dz * dz < 1e-18;
}

MCGAL::Vertex* insertedCenterVertex(MCGAL::Halfedge* newH,
                                    const MCGAL::Point& insertPoint) {
    if (!newH) return nullptr;
    MCGAL::Vertex* a = newH->vertex();
    MCGAL::Vertex* b = newH->end_vertex();
    if (a && !a->isRemoved() && samePointApprox(a->point(), insertPoint)) return a;
    if (b && !b->isRemoved() && samePointApprox(b->point(), insertPoint)) return b;
    RingSnapshot ra = snapshotRing(a);
    RingSnapshot rb = snapshotRing(b);
    if (a && !a->isRemoved() && !ra.ringVerts.empty() &&
        (!b || b->isRemoved() || rb.ringVerts.empty() ||
         ra.ringVerts.size() <= rb.ringVerts.size())) {
        return a;
    }
    if (b && !b->isRemoved()) return b;
    return a;
}

// D15 修复辅助：把新顶点的 halfedges() 列表补上。
// 背景：`deps/MCGAL` 的 `create_center_vertex_without_init` 建了顶点与半边，
// 但**不填该顶点自己的 `halfedges()` 列表**（函数末尾"collect all the halfedge"
// 那段是被注释掉的）——这正是名字里 "without_init" 缺的那一步。
// 实测：解码端插入的顶点全部 `halfedges()` 为空（1 轮 16146、26 轮 34786，
// 恰等于插入次数）。deps/MCGAL 禁改，故在调用方补。
// 走法：绕顶点一圈的出边为 h -> h->opposite()->next()；只有绕回起点（闭合扇）
// 才写入，避免写入半圈。
// D15 真正的根因（本轮定位）：`Vertex::begin()` 用的是 **`proxyHalfedge_`**，
// 不是 `halfedges_`：
//     halfedge_iterator begin() const { return halfedge_iterator(proxyHalfedge_); }
// 而 `create_center_vertex_without_init` 建完顶点后**既不填 `halfedges_`，
// 也不设 `proxyHalfedge_`** ⇒ 新顶点的 `proxyHalfedge_` 恒为 nullptr
// ⇒ `for (h : *v)` 一圈都走不出来（实测 16146 个新顶点全部如此）。
// 修法（调用方，deps/MCGAL 禁改）：把返回值交给新顶点当 proxyHalfedge。
// 返回的是 `oppo_new`，其 `vertex()` 正是新顶点，且是一条**出边** ⇒ 正是所需。
long long g_d17NonClosing = 0;
long long g_d17Checked = 0;
long long g_d17MergeBroke = 0;
long long g_d17Fixed = 0;

void repairNewVertexProxyHalfedge(MCGAL::Halfedge* returnedHe) {
    if (!returnedHe) return;
    MCGAL::Vertex* v = returnedHe->vertex();
    if (!v || v->isRemoved()) return;
    if (v->proxyHalfedge() == nullptr) {
        v->setProxyHalfedge(returnedHe);
    }
    // D17「建环处保证闭合」：设完起点后**自检**该环是否闭合。
    // 只在不闭合时才计数/报警 ⇒ 默认路径（环恒闭合）**行为完全不变**。
    ++g_d17Checked;
    if (!pmsf::vertexRingCloses(v)) {
        ++g_d17NonClosing;
    }
}

void repairNewVertexHalfedgeRing(MCGAL::Halfedge* anyOutgoing) {
    if (!anyOutgoing) return;
    MCGAL::Vertex* v = anyOutgoing->vertex();
    if (!v || v->isRemoved() || !v->halfedges().empty()) return;
    std::vector<MCGAL::Halfedge*> fan;
    MCGAL::Halfedge* start = anyOutgoing;
    MCGAL::Halfedge* cur = start;
    do {
        fan.push_back(cur);
        cur = cur->opposite() ? cur->opposite()->next() : nullptr;
    } while (cur && cur != start && fan.size() < 128);
    if (cur == start) {
        v->halfedges() = fan;
    }
}


// D17 闭合定位（局部、便宜）：检查**某个面环上**所有顶点的环是否闭合，
// 返回不闭合的个数。O(度数)，不是 O(V)，可安全地挂在每次改接操作前后。
// 门控 PMSF_D17_LOCAL=1。
bool d17LocalEnabled() {
    static const bool v = [] {
        const char* e = std::getenv("PMSF_D17_LOCAL");
        return e && e[0] == '1';
    }();
    return v;
}

int d17CountBadOnFace(MCGAL::Facet* f) {
    if (!f || f->isRemoved()) return 0;
    int bad = 0;
    for (MCGAL::Halfedge* hh : pmsf::boundedRing(f ? f->proxyHalfedge() ? f->proxyHalfedge()->vertex() : nullptr : nullptr)) {
        (void)hh;
    }
    // 上面那种写法不对（面不是顶点）；直接遍历面的半边取顶点：
    bad = 0;
    MCGAL::Halfedge* start = f->proxyHalfedge();
    if (!start) return 0;
    MCGAL::Halfedge* cur = start;
    int guard = 0;
    do {
        MCGAL::Vertex* v = cur ? cur->vertex() : nullptr;
        if (v && !v->isRemoved() && !pmsf::vertexRingCloses(v)) ++bad;
        cur = cur ? cur->next() : nullptr;
    } while (cur && cur != start && ++guard < 256);
    return bad;
}


// D17 修复（调用方；deps/MCGAL 禁改）：`mergePatchForSymbol` 会破坏环闭合
// （实测那之后某顶点的环不再走回起点）。修法：合并在**本面**上进行，
/// 本面的 `next()` 环是良构的，因此面上的每条半边都是一条**可靠的出边**。
// 对面上每个"环不闭合"的顶点，把它的 `proxyHalfedge_` 改设到本面上那条出边，
// 再复检；仍不闭合则保留原值（不引入更坏的状态）。
// 因普通路径 `mergePatchForSymbol` 是 no-op、根本不会破坏闭合，本修复只在
// 真的发生破坏时才动作 ⇒ **对默认路径零影响**。
int repairRingClosureOnFace(MCGAL::Facet* f) {
    if (!f || f->isRemoved()) return 0;
    MCGAL::Halfedge* start = f->proxyHalfedge();
    if (!start) return 0;
    int fixed_ = 0;
    MCGAL::Halfedge* cur = start;
    int guard = 0;
    do {
        MCGAL::Vertex* v = cur ? cur->vertex() : nullptr;
        if (v && !v->isRemoved() && !pmsf::vertexRingCloses(v)) {
            MCGAL::Halfedge* saved = v->proxyHalfedge();
            v->setProxyHalfedge(cur);
            if (pmsf::vertexRingCloses(v)) ++fixed_;
            else v->setProxyHalfedge(saved);   // 没修好就还原，不引入更坏状态
        }
        cur = cur ? cur->next() : nullptr;
    } while (cur && cur != start && ++guard < 256);
    return fixed_;
}

int traceOpLimit() {
    if (const char* e = std::getenv("PMSF_PCLTTM_TRACE_OPS")) {
        int n = std::atoi(e);
        if (n > 0) return n;
    }
    return 0;
}

void traceDecoderGate(std::shared_ptr<MCGAL::Mesh> mesh,
                      int symbolIndex,
                      int phase,
                      const char* kind,
                      MCGAL::Halfedge* gate,
                      int valence) {
    const int lim = traceOpLimit();
    if (lim <= 0 || symbolIndex >= lim || !mesh) return;
    auto qstr = [&](MCGAL::Vertex* v) {
        if (!v || v->isRemoved()) return std::string("nil");
        MCGAL::PointInt q = mesh->floatPosToInt(v->point());
        return std::to_string(q[0]) + "," + std::to_string(q[1]) + "," +
               std::to_string(q[2]);
    };
    MCGAL::Halfedge* input = inputBoundaryFromGate(gate);
    std::fprintf(stderr,
                 "[PCLTTM trace dec gate] si=%d phase=%d kind=%s valence=%d input=(%s)-(%s)\n",
                 symbolIndex, phase, kind, valence,
                 input ? qstr(input->vertex()).c_str() : "nil",
                 input ? qstr(input->end_vertex()).c_str() : "nil");
}

void traceDecoderOp(std::shared_ptr<MCGAL::Mesh> mesh,
                    int opIndex,
                    int phase,
                    int valence,
                    MCGAL::Vertex* inputA,
                    MCGAL::Vertex* inputB,
                    const std::vector<MCGAL::Vertex*>& ringVerts) {
    const int lim = traceOpLimit();
    if (lim <= 0 || opIndex >= lim || !mesh) return;
    auto qstr = [&](MCGAL::Vertex* v) {
        if (!v || v->isRemoved()) return std::string("nil");
        MCGAL::PointInt q = mesh->floatPosToInt(v->point());
        return std::to_string(q[0]) + "," + std::to_string(q[1]) + "," +
               std::to_string(q[2]);
    };
    std::fprintf(stderr,
                 "[PCLTTM trace dec] op=%d phase=%d valence=%d input=(%s)-(%s) ring=",
                 opIndex, phase, valence, qstr(inputA).c_str(), qstr(inputB).c_str());
    for (size_t i = 0; i < ringVerts.size(); ++i) {
        std::fprintf(stderr, "%s%s", i ? ";" : "", qstr(ringVerts[i]).c_str());
    }
    std::fprintf(stderr, "\n");
}

bool insertDecodedVertexOnFace(std::shared_ptr<MCGAL::Mesh> mesh,
                               MCGAL::Facet* face,
                               const std::vector<int>& resX,
                               const std::vector<int>& resY,
                               const std::vector<int>& resZ,
                               int residualIndex,
                               const QKey* absQ = nullptr,
                               MCGAL::Vertex** insertedVertex = nullptr) {
    if (insertedVertex) *insertedVertex = nullptr;
    if (!mesh || !face || face->isRemoved()) return false;
    MCGAL::Halfedge* faceProxy = face->proxyHalfedge();
    if (!faceProxy) return false;
    MCGAL::Point insertPoint;
    if (absQ) {
        MCGAL::PointInt vQ(absQ->x, absQ->y, absQ->z);
        insertPoint = mesh->intPosToFloat(vQ);
        // P5.1 判别器（门控 PMSF_RDPATCH_GEOM_DBG=1，默认关）：
        // 打印 absQ 分支实际拿到的值，与编码端同 residualIndex 的 vvAbsQX/Y/Z 对拍
        // —— 判"数值本身错"还是"传了但没被采用"。
        {
            static const bool kDbg = [] {
                const char* e = std::getenv("PMSF_RDPATCH_GEOM_DBG");
                return e && e[0] == '1';
            }();
            static int dbgN = 0;
            if (kDbg && dbgN < 8) {
                std::fprintf(stderr,
                             "  [absqdbg] resIdx=%d absQ=(%d,%d,%d) -> pt=(%.4f,%.4f,%.4f)\n",
                             residualIndex, absQ->x, absQ->y, absQ->z,
                             insertPoint.x(), insertPoint.y(), insertPoint.z());
                ++dbgN;
            }
        }
    } else if (residualIndex >= 0 &&
        residualIndex < static_cast<int>(resX.size()) &&
        residualIndex < static_cast<int>(resY.size()) &&
        residualIndex < static_cast<int>(resZ.size())) {
        MCGAL::PointInt baryQ = MCGAL::rdBarycenterQuantized(face, mesh);
        MCGAL::PointInt vQ(baryQ[0] + resX[static_cast<size_t>(residualIndex)],
                           baryQ[1] + resY[static_cast<size_t>(residualIndex)],
                           baryQ[2] + resZ[static_cast<size_t>(residualIndex)]);
        // P5.1 判别器：门控 PMSF_RDPATCH_GEOM_DBG=1（默认关）。
        // 打印每次插入的 (faceId, residualIndex, baryQ, residual, vQ)，用于判定
        // 重建坐标为何溢出原包围盒（残差是否被加到了错的面）。
        {
            static const bool kDbg = [] {
                const char* e = std::getenv("PMSF_RDPATCH_GEOM_DBG");
                return e && e[0] == '1';
            }();
            static int dbgN = 0;
            if (kDbg && dbgN < 8) {
                std::fprintf(stderr,
                             "  [geomdbg] face=%d resIdx=%d baryQ=(%d,%d,%d) "
                             "res=(%d,%d,%d) vQ=(%d,%d,%d)\n",
                             face->poolId(), residualIndex,
                             baryQ[0], baryQ[1], baryQ[2],
                             resX[static_cast<size_t>(residualIndex)],
                             resY[static_cast<size_t>(residualIndex)],
                             resZ[static_cast<size_t>(residualIndex)],
                             vQ[0], vQ[1], vQ[2]);
                ++dbgN;
            }
        }
        insertPoint = mesh->intPosToFloat(vQ);
    } else {
        insertPoint = MCGAL::barycenter(faceProxy);
    }
    if (!absQ) {
        insertPoint = safeInteriorInsertPoint(faceProxy, insertPoint);
    }
    MCGAL::Halfedge* newH = mesh->create_center_vertex_without_init(faceProxy, insertPoint);
    ++g_ccvSiteA;
    repairNewVertexProxyHalfedge(newH);
    repairNewVertexHalfedgeRing(newH);
    if (!newH) return false;
    // P5.1 最后一环：**回读**新顶点实际落点，与传入的 insertPoint 比对。
    // 数据链已全部验证正确（absQ 逐项全等），若这里也不等，则装配函数未采用传入坐标。
    {
        static const bool kDbg = [] {
            const char* e = std::getenv("PMSF_RDPATCH_GEOM_DBG");
            return e && e[0] == '1';
        }();
        static int dbgN = 0;
        if (kDbg && dbgN < 8) {
            MCGAL::Vertex* nv = insertedCenterVertex(newH, insertPoint);
            if (nv) {
                const MCGAL::Point ap = nv->point();
                std::fprintf(stderr,
                             "  [readback] want=(%.5f,%.5f,%.5f) got=(%.5f,%.5f,%.5f)"
                             "  |d|=%.6f\n",
                             insertPoint.x(), insertPoint.y(), insertPoint.z(),
                             ap.x(), ap.y(), ap.z(),
                             std::max({std::fabs(ap.x() - insertPoint.x()),
                                       std::fabs(ap.y() - insertPoint.y()),
                                       std::fabs(ap.z() - insertPoint.z())}));
            }
            ++dbgN;
        }
    }
    if (insertedVertex) {
        *insertedVertex = insertedCenterVertex(newH, insertPoint);
    }
    return true;
}

MCGAL::Facet* livingFaceByRank(std::shared_ptr<MCGAL::Mesh> mesh, int rank) {
    if (!mesh || rank < 0) return nullptr;
    int cur = 0;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        if (cur == rank) return f;
        ++cur;
    }
    return nullptr;
}

std::unordered_map<QKey, MCGAL::Vertex*, QKeyHash>
buildQVertexMap(std::shared_ptr<MCGAL::Mesh> mesh) {
    std::unordered_map<QKey, MCGAL::Vertex*, QKeyHash> out;
    if (!mesh) return out;
    out.reserve(mesh->vertices().size() * 2 + 16);
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        MCGAL::PointInt q = mesh->floatPosToInt(v->point());
        out[{q[0], q[1], q[2]}] = v;
    }
    return out;
}

MCGAL::Vertex* findQVertexWithTolerance(
    std::unordered_map<QKey, MCGAL::Vertex*, QKeyHash>& qToVertex,
    const QKey& key) {
    auto exact = qToVertex.find(key);
    if (exact != qToVertex.end() && exact->second && !exact->second->isRemoved()) {
        return exact->second;
    }
    for (int radius = 1; radius <= 1; ++radius) {
        for (int dx = -radius; dx <= radius; ++dx) {
            for (int dy = -radius; dy <= radius; ++dy) {
                for (int dz = -radius; dz <= radius; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    if (std::abs(dx) + std::abs(dy) + std::abs(dz) > radius) continue;
                    auto it = qToVertex.find({key.x + dx, key.y + dy, key.z + dz});
                    if (it != qToVertex.end() && it->second && !it->second->isRemoved()) {
                        return it->second;
                    }
                }
            }
        }
    }
    return nullptr;
}

std::vector<MCGAL::Vertex*> ringVerticesByCoords(
    const std::vector<unsigned char>& ringSizes,
    const std::vector<int>& ringQX,
    const std::vector<int>& ringQY,
    const std::vector<int>& ringQZ,
    size_t ringIndex,
    size_t coordOffset,
    std::unordered_map<QKey, MCGAL::Vertex*, QKeyHash>& qToVertex) {
    std::vector<MCGAL::Vertex*> ring;
    if (ringIndex >= ringSizes.size()) return ring;
    const int n = static_cast<int>(ringSizes[ringIndex]);
    ring.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        const size_t idx = coordOffset + static_cast<size_t>(i);
        if (idx >= ringQX.size() || idx >= ringQY.size() || idx >= ringQZ.size()) {
            ring.clear();
            return ring;
        }
        MCGAL::Vertex* v = findQVertexWithTolerance(
            qToVertex, {ringQX[idx], ringQY[idx], ringQZ[idx]});
        if (!v) {
            ring.clear();
            return ring;
        }
        ring.push_back(v);
    }
    return ring;
}

bool faceVerticesWithin(MCGAL::Facet* f, const std::set<MCGAL::Vertex*>& verts) {
    if (!f || f->isRemoved() || !f->proxyHalfedge()) return false;
    MCGAL::Halfedge* start = f->proxyHalfedge();
    MCGAL::Halfedge* h = start;
    int guard = 0;
    do {
        if (!h || h->isRemoved() || !verts.count(h->vertex())) return false;
        h = h->next();
        ++guard;
    } while (h && h != start && guard < 256);
    return h == start;
}

void markPatchInteriorFacesProcessedFromRing(
    const std::vector<MCGAL::Vertex*>& ring,
    MCGAL::Vertex* insertedCenter,
    std::set<MCGAL::Facet*>& facesProcessed) {
    std::set<MCGAL::Vertex*> patchVerts;
    for (MCGAL::Vertex* v : ring) {
        if (v && !v->isRemoved()) patchVerts.insert(v);
    }
    if (insertedCenter && !insertedCenter->isRemoved()) {
        patchVerts.insert(insertedCenter);
    }
    if (patchVerts.empty()) return;
    std::set<MCGAL::Facet*> seen;
    for (MCGAL::Vertex* v : patchVerts) {
        for (MCGAL::Halfedge* hv : boundedVertexHalfedges(v)) {
            MCGAL::Facet* f = hv ? hv->face() : nullptr;
            if (!f || f->isRemoved() || !seen.insert(f).second) continue;
            if (!faceVerticesWithin(f, patchVerts)) continue;
            f->setProcessedFlag();
            facesProcessed.insert(f);
        }
    }
}

MCGAL::Facet* mergePatchByRing(std::shared_ptr<MCGAL::Mesh> mesh,
                               const std::vector<MCGAL::Vertex*>& ring,
                               int valence,
                               bool allowSubsetSearch = false,
                               const std::function<bool(const PatchCandidate&)>& acceptPatch = {}) {
    if (!mesh || static_cast<int>(ring.size()) != valence || valence < 3) return nullptr;
    std::set<MCGAL::Vertex*> ringSet(ring.begin(), ring.end());

    std::vector<MCGAL::Facet*> faces;
    faces.reserve(static_cast<size_t>(std::max(1, valence - 2)));
    for (MCGAL::Facet* f : mesh->faces()) {
        if (f && !f->isRemoved() && faceDegree(f) == valence && faceVerticesWithin(f, ringSet)) return f;
        if (f && !f->isRemoved() && faceDegree(f) == 3 && faceVerticesWithin(f, ringSet)) faces.push_back(f);
    }
    PatchCandidate patch;
    if (!validateTriangulatedPatch(faces, valence, patch)) {
        if (!allowSubsetSearch) return nullptr;
        const int targetFaces = valence - 2;
        if (static_cast<int>(faces.size()) < targetFaces || faces.size() > 24) {
            return nullptr;
        }
        std::vector<MCGAL::Facet*> chosen;
        chosen.reserve(static_cast<size_t>(targetFaces));
        bool found = false;
        std::function<bool(int)> dfs = [&](int nextIdx) -> bool {
            if (static_cast<int>(chosen.size()) == targetFaces) {
                PatchCandidate cand;
                if (!validateTriangulatedPatch(chosen, valence, cand)) return false;
                patch = std::move(cand);
                found = true;
                return true;
            }
            for (int i = nextIdx; i < static_cast<int>(faces.size()); ++i) {
                chosen.push_back(faces[static_cast<size_t>(i)]);
                if (dfs(i + 1)) return true;
                chosen.pop_back();
            }
            return false;
        };
        if (!dfs(0) || !found) return nullptr;
    }
    if (acceptPatch && !acceptPatch(patch)) return nullptr;
    return mergePatchToPolygon(patch, valence);
}

MCGAL::Facet* mergePatchByFaceRanks(std::shared_ptr<MCGAL::Mesh> mesh,
                                    const std::vector<int>& ranks,
                                    int valence) {
    if (!mesh || ranks.empty() || valence < 3) return nullptr;
    std::vector<MCGAL::Facet*> faces;
    faces.reserve(ranks.size());
    for (int rank : ranks) {
        MCGAL::Facet* f = livingFaceByRank(mesh, rank);
        if (!f || f->isRemoved()) return nullptr;
        faces.push_back(f);
    }
    if (valence > 3 && !faces.empty()) {
        bool allSame = true;
        for (MCGAL::Facet* f : faces) {
            if (f != faces.front()) {
                allSame = false;
                break;
            }
        }
        if (allSame && faceDegree(faces.front()) == valence) return faces.front();
    }
    if (valence == 3) return faces.front();
    PatchCandidate patch;
    if (!validateTriangulatedPatch(faces, valence, patch)) return nullptr;
    return mergePatchToPolygon(patch, valence);
}

bool appendMepp2PatchFace(MCGAL::Halfedge* h,
                          std::vector<MCGAL::Facet*>& faces) {
    if (!h || h->isRemoved()) return false;
    MCGAL::Facet* f = h->face();
    if (!f || f->isRemoved()) return false;
    if (std::find(faces.begin(), faces.end(), f) != faces.end()) return false;
    faces.push_back(f);
    return true;
}

bool appendMepp2PatchFaceAcross(MCGAL::Halfedge* h,
                                std::vector<MCGAL::Facet*>& faces) {
    if (!h || h->isRemoved() || !h->opposite() || h->opposite()->isRemoved()) {
        return false;
    }
    return appendMepp2PatchFace(h->opposite(), faces);
}

bool discoverMepp2PatchFromInput(MCGAL::Halfedge* input,
                                 int valence,
                                 PatchCandidate& patch,
                                 int& typeOut) {
    typeOut = 0;
    if (!input || input->isRemoved() || valence < 3 || valence > 6) return false;
    if (!input->face() || input->face()->isRemoved()) return false;
    const int type = mepp2TypeFromInputBoundary(input, valence);
    if (type == 0) return false;

    std::vector<MCGAL::Facet*> faces;
    faces.reserve(static_cast<size_t>(valence - 2));
    if (!appendMepp2PatchFace(input, faces)) return false;

    auto opp = [](MCGAL::Halfedge* h) -> MCGAL::Halfedge* {
        return (h && h->opposite() && !h->opposite()->isRemoved()) ? h->opposite() : nullptr;
    };

    bool ok = true;
    if (valence == 3) {
        ok = true;
    } else if (type == 5 || type == 8) {
        ok = appendMepp2PatchFaceAcross(boundedPrevInFace(input), faces);
    } else if (type == 6 || type == 7) {
        ok = appendMepp2PatchFaceAcross(input->next(), faces);
    } else if (type == 9 || type == 12) {
        MCGAL::Halfedge* g = opp(boundedPrevInFace(input));
        ok = appendMepp2PatchFace(g, faces);
        g = ok ? opp(g ? g->next() : nullptr) : nullptr;
        ok = ok && appendMepp2PatchFace(g, faces);
    } else if (type == 10) {
        MCGAL::Halfedge* g = opp(input->next());
        ok = appendMepp2PatchFace(g, faces);
        g = ok ? opp(boundedPrevInFace(g)) : nullptr;
        ok = ok && appendMepp2PatchFace(g, faces);
    } else if (type == 11) {
        ok = appendMepp2PatchFaceAcross(input->next(), faces) &&
             appendMepp2PatchFaceAcross(boundedPrevInFace(input), faces);
    } else if (type == 13 || type == 16) {
        MCGAL::Halfedge* g = opp(boundedPrevInFace(input));
        ok = appendMepp2PatchFace(g, faces);
        ok = ok && appendMepp2PatchFaceAcross(g ? g->next() : nullptr, faces);
        ok = ok && appendMepp2PatchFaceAcross(boundedPrevInFace(g), faces);
    } else if (type == 14 || type == 15) {
        MCGAL::Halfedge* g = opp(input->next());
        ok = appendMepp2PatchFace(g, faces);
        ok = ok && appendMepp2PatchFaceAcross(g ? g->next() : nullptr, faces);
        ok = ok && appendMepp2PatchFaceAcross(boundedPrevInFace(g), faces);
    } else {
        ok = false;
    }
    if (!ok) return false;
    PatchCandidate tmp;
    if (!validateTriangulatedPatch(faces, valence, tmp)) return false;
    patch = std::move(tmp);
    typeOut = type;
    return true;
}

MCGAL::Halfedge* joinMepp2Face(MCGAL::Halfedge* h) {
    if (!h || h->isRemoved() || !h->opposite() || h->opposite()->isRemoved()) {
        return nullptr;
    }
    if (!h->face() || h->face()->isRemoved() ||
        !h->opposite()->face() || h->opposite()->face()->isRemoved()) {
        return nullptr;
    }
    return MCGAL::join_face(h);
}

MCGAL::Facet* mergeMepp2PatchFromInput(MCGAL::Halfedge* input,
                                       int type,
                                       int valence) {
    if (!input || input->isRemoved() || !input->face() || input->face()->isRemoved()) {
        return nullptr;
    }
    if (valence == 3) {
        input->face()->proxyHalfedge(input);
        return input->face();
    }

    auto opp = [](MCGAL::Halfedge* h) -> MCGAL::Halfedge* {
        return (h && h->opposite() && !h->opposite()->isRemoved()) ? h->opposite() : nullptr;
    };

    MCGAL::Halfedge* g = nullptr;
    switch (type) {
        case 5:
        case 8:
            g = joinMepp2Face(boundedPrevInFace(input));
            if (!g) return nullptr;
            break;
        case 6:
        case 7:
            g = joinMepp2Face(input->next());
            if (!g) return nullptr;
            break;
        case 9:
        case 12:
            g = joinMepp2Face(boundedPrevInFace(input));
            if (!g) return nullptr;
            g = joinMepp2Face(g ? g->next() : nullptr);
            if (!g) return nullptr;
            break;
        case 10:
            g = joinMepp2Face(opp(input->next()));
            if (!g) return nullptr;
            g = joinMepp2Face(g);
            if (!g) return nullptr;
            break;
        case 11:
            g = joinMepp2Face(input->next());
            if (!g) return nullptr;
            g = joinMepp2Face(boundedPrevInFace(g));
            if (!g) return nullptr;
            break;
        case 13:
        case 16:
            g = joinMepp2Face(opp(boundedPrevInFace(input)));
            if (!g) return nullptr;
            g = joinMepp2Face(g);
            if (!g) return nullptr;
            g = joinMepp2Face(g);
            if (!g) return nullptr;
            break;
        case 14:
        case 15:
            g = joinMepp2Face(opp(input->next()));
            if (!g) return nullptr;
            g = joinMepp2Face(g);
            if (!g) return nullptr;
            g = joinMepp2Face(g);
            if (!g) return nullptr;
            break;
        default:
            return nullptr;
    }

    MCGAL::Facet* poly = nullptr;
    if (input && !input->isRemoved() && input->face() && !input->face()->isRemoved()) {
        poly = input->face();
    } else if (g && !g->isRemoved() && g->face() && !g->face()->isRemoved()) {
        poly = g->face();
    }
    if (poly) poly->proxyHalfedge(input);
    return poly && faceDegree(poly) == valence ? poly : nullptr;
}

RDPatchDecoderResult runDirectRankReplay(
    std::shared_ptr<MCGAL::Mesh> mesh,
    const std::vector<RDPatchSymbol>& decodedSymbols,
    const std::vector<int>& resX,
    const std::vector<int>& resY,
    const std::vector<int>& resZ,
    const std::vector<int>& faceRanks,
    const std::vector<unsigned char>& ringSizes = {},
    const std::vector<int>& ringQX = {},
    const std::vector<int>& ringQY = {},
    const std::vector<int>& ringQZ = {},
    const std::vector<int>& absQX = {},
    const std::vector<int>& absQY = {},
    const std::vector<int>& absQZ = {}) {
    RDPatchDecoderResult R;
    R.numSymsAvailable = static_cast<int>(decodedSymbols.size());
    R.numVerticesIn = livingVertexCount(mesh);
    const bool useRingSideInfo = !ringSizes.empty();
    if (!mesh || (faceRanks.empty() && !useRingSideInfo)) {
        R.numVerticesOut = R.numVerticesIn;
        return R;
    }
    const bool useResiduals = !resX.empty()
                              && resX.size() == resY.size()
                              && resX.size() == resZ.size();
    const bool useAbsQ = !absQX.empty()
                         && absQX.size() == absQY.size()
                         && absQX.size() == absQZ.size();
    int expectedSingleRanks = 0;
    for (const RDPatchSymbol& s : decodedSymbols) {
        if (s.kind != RDPatchSymbolKind::NULL_PATCH) ++expectedSingleRanks;
    }
    const bool flattenedPatchRanks =
        !useRingSideInfo && static_cast<int>(faceRanks.size()) > expectedSingleRanks;
    size_t rankCursor = 0;
    size_t ringCursor = 0;
    size_t ringCoordCursor = 0;
    auto qToVertex = buildQVertexMap(mesh);
    const std::vector<int> emptyResiduals;
    for (size_t i = 0; i < decodedSymbols.size(); ++i) {
        const RDPatchSymbol& sym = decodedSymbols[i];
        ++R.numSymbolsTotal;
                if (sym.kind == RDPatchSymbolKind::NULL_PATCH) {
            ++R.numNullPopsConsumed;
            continue;
        }
        const int residualIndex = R.numVValPopsConsumed;
                MCGAL::Facet* f = nullptr;
        std::vector<int> patchRanksForDebug;
        std::vector<MCGAL::Vertex*> ringForDebug;
        if (useRingSideInfo) {
            const int valence = symbolValence(sym);
            const size_t coordStart = ringCoordCursor;
            ringForDebug = ringVerticesByCoords(ringSizes, ringQX, ringQY, ringQZ,
                                                ringCursor, coordStart, qToVertex);
                        if (ringCursor < ringSizes.size()) {
                ringCoordCursor += static_cast<size_t>(ringSizes[ringCursor]);
            }
            ++ringCursor;
            f = mergePatchByRing(mesh, ringForDebug, valence);
            if (!f || faceDegree(f) != valence) {
                if (residualIndex % 1000 == 0) {
                    std::cerr << "      ring_replay_stop idx=" << residualIndex
                              << " valence=" << valence
                              << " ring_sz=" << ringForDebug.size()
                              << " face_degree=" << faceDegree(f) << "\n";
                }
                break;
            }
        } else if (flattenedPatchRanks) {
            const int valence = symbolValence(sym);
            const int n = std::max(1, valence - 2);
            std::vector<int> patchRanks;
            patchRanks.reserve(static_cast<size_t>(n));
            for (int k = 0; k < n && rankCursor < faceRanks.size(); ++k) {
                patchRanks.push_back(faceRanks[rankCursor++]);
            }
            patchRanksForDebug = patchRanks;
            f = mergePatchByFaceRanks(mesh, patchRanks, valence);
        } else if (residualIndex < static_cast<int>(faceRanks.size())) {
            f = livingFaceByRank(mesh, faceRanks[static_cast<size_t>(residualIndex)]);
        }
        if (!f) {
            if (useRingSideInfo || !faceRanks.empty()) {
                if (residualIndex % 1000 == 0) {
                    std::cerr << "      sideinfo_replay_stop idx=" << residualIndex
                              << " valence=" << symbolValence(sym)
                              << " flat_ranks=" << (flattenedPatchRanks ? 1 : 0)
                              << " ring_mode=" << (useRingSideInfo ? 1 : 0)
                              << " mesh_faces=" << (mesh ? mesh->faces().size() : 0)
                              << " patch_ranks=[";
                    for (size_t pi = 0; pi < patchRanksForDebug.size(); ++pi) {
                        if (pi) std::cerr << ",";
                        std::cerr << patchRanksForDebug[pi];
                    }
                    std::cerr << "]\n";
                }
                break;
            }
            f = firstUsableFace(mesh);
        }
        const int beforeDegree = faceDegree(f);
        if (f && !flattenedPatchRanks && !useRingSideInfo) {
            f = mergePatchForSymbol(f, sym, true);
        }
        QKey decodedQKey{};
        bool hasDecodedQKey = false;
        if ((useRingSideInfo || useAbsQ) && f && !f->isRemoved() && f->proxyHalfedge()) {
            if (useAbsQ && residualIndex < static_cast<int>(absQX.size())) {
                decodedQKey = {absQX[static_cast<size_t>(residualIndex)],
                               absQY[static_cast<size_t>(residualIndex)],
                               absQZ[static_cast<size_t>(residualIndex)]};
                hasDecodedQKey = true;
            } else if (useResiduals && residualIndex < static_cast<int>(resX.size())) {
                MCGAL::PointInt baryQ = MCGAL::rdBarycenterQuantized(f, mesh);
                decodedQKey = {baryQ[0] + resX[static_cast<size_t>(residualIndex)],
                               baryQ[1] + resY[static_cast<size_t>(residualIndex)],
                               baryQ[2] + resZ[static_cast<size_t>(residualIndex)]};
                hasDecodedQKey = true;
            } else {
                MCGAL::PointInt q = mesh->floatPosToInt(MCGAL::barycenter(f->proxyHalfedge()));
                decodedQKey = {q[0], q[1], q[2]};
                hasDecodedQKey = true;
            }
        }
                MCGAL::Vertex* insertedVertex = nullptr;
        if (!insertDecodedVertexOnFace(mesh, f,
                                       useResiduals ? resX : emptyResiduals,
                                       useResiduals ? resY : emptyResiduals,
                                       useResiduals ? resZ : emptyResiduals,
                                       residualIndex,
                                       hasDecodedQKey ? &decodedQKey : nullptr,
                                       &insertedVertex)) {
            break;
        }
        ++R.numVValPopsConsumed;
        ++R.numAddedVertices;
        if (useRingSideInfo) {
            if (insertedVertex && !insertedVertex->isRemoved() && hasDecodedQKey) {
                qToVertex[decodedQKey] = insertedVertex;
            }
        }
    }
    R.numVerticesOut = livingVertexCount(mesh);
    if (!useRingSideInfo) {
        repairDegenerateFaces(mesh);
    }
    return R;
}

}  // namespace

RDPatchDecoderResult runDirectRDPatchReplay(
    std::shared_ptr<MCGAL::Mesh> mesh,
    const std::vector<RDPatchSymbol>& decodedSymbols,
    const std::vector<int>& resX,
    const std::vector<int>& resY,
    const std::vector<int>& resZ,
    const std::vector<int>& faceRanks,
    const std::vector<unsigned char>& ringSizes,
    const std::vector<int>& ringQX,
    const std::vector<int>& ringQY,
    const std::vector<int>& ringQZ,
    const std::vector<int>& absQX,
    const std::vector<int>& absQY,
    const std::vector<int>& absQZ) {
    return runDirectRankReplay(mesh, decodedSymbols, resX, resY, resZ, faceRanks,
                               ringSizes, ringQX, ringQY, ringQZ,
                               absQX, absQY, absQZ);
}

RDPatchDecoderResult runFaceIdRDPatchReplay(
    std::shared_ptr<MCGAL::Mesh> mesh,
    const std::vector<RDPatchSymbol>& decodedSymbols,
    const std::vector<int>& resX,
    const std::vector<int>& resY,
    const std::vector<int>& resZ,
    const std::vector<int>& faceIds) {
    RDPatchDecoderResult R;
    R.numSymsAvailable = static_cast<int>(decodedSymbols.size());
    if (!mesh) return R;
    R.numVerticesIn = livingVertexCount(mesh);
    int vIdx = 0;
    for (const RDPatchSymbol& sym : decodedSymbols) {
        ++R.numSymbolsTotal;
        if (sym.kind == RDPatchSymbolKind::NULL_PATCH) { ++R.numNullPopsConsumed; continue; }
        if (vIdx >= static_cast<int>(faceIds.size())) break;
        MCGAL::Facet* f = nullptr;
        for (MCGAL::Facet* cand : mesh->faces()) {
            if (cand && !cand->isRemoved() && cand->poolId() == faceIds[static_cast<size_t>(vIdx)]) {
                f = cand;
                break;
            }
        }
        if (!f) break;
        f = mergePatchForSymbol(f, sym, true);
        if (!f || f->isRemoved()) break;
        MCGAL::PointInt baryQ = MCGAL::rdBarycenterQuantized(f, mesh);
        QKey q{baryQ[0] + (vIdx < static_cast<int>(resX.size()) ? resX[static_cast<size_t>(vIdx)] : 0),
               baryQ[1] + (vIdx < static_cast<int>(resY.size()) ? resY[static_cast<size_t>(vIdx)] : 0),
               baryQ[2] + (vIdx < static_cast<int>(resZ.size()) ? resZ[static_cast<size_t>(vIdx)] : 0)};
        MCGAL::Vertex* inserted = nullptr;
        if (!insertDecodedVertexOnFace(mesh, f, resX, resY, resZ, vIdx, &q, &inserted)) break;
        ++vIdx;
        ++R.numAddedVertices;
        ++R.numVValPopsConsumed;
    }
    R.numVerticesOut = livingVertexCount(mesh);
    return R;
}

RDPatchDecoderResult runFaceRankRDPatchReplay(
    std::shared_ptr<MCGAL::Mesh> mesh,
    const std::vector<RDPatchSymbol>& decodedSymbols,
    const std::vector<int>& resX,
    const std::vector<int>& resY,
    const std::vector<int>& resZ,
    const std::vector<int>& faceRanks) {
    return runDirectRankReplay(mesh, decodedSymbols, resX, resY, resZ, faceRanks,
                               /*ringSizes=*/{}, /*ringQX=*/{}, /*ringQY=*/{},
                               /*ringQZ=*/{}, /*absQX=*/{}, /*absQY=*/{}, /*absQZ=*/{});
}

RDPatchDecoderResult runPointRDPatchReplay(
    std::shared_ptr<MCGAL::Mesh> mesh,
    const std::vector<RDPatchSymbol>& decodedSymbols,
    const std::vector<int>& resX,
    const std::vector<int>& resY,
    const std::vector<int>& resZ,
    const std::vector<int>& absQX,
    const std::vector<int>& absQY,
    const std::vector<int>& absQZ) {
    RDPatchDecoderResult R;
    R.numSymsAvailable = static_cast<int>(decodedSymbols.size());
    if (!mesh) return R;
    R.numVerticesIn = livingVertexCount(mesh);
    const bool useAbsQ = !absQX.empty() && absQX.size() == absQY.size() &&
                         absQX.size() == absQZ.size();
    int vIdx = 0;
    for (const RDPatchSymbol& sym : decodedSymbols) {
        ++R.numSymbolsTotal;
        if (sym.kind == RDPatchSymbolKind::NULL_PATCH) { ++R.numNullPopsConsumed; continue; }
        if (!useAbsQ || vIdx >= static_cast<int>(absQX.size())) break;
        const QKey q{absQX[static_cast<size_t>(vIdx)], absQY[static_cast<size_t>(vIdx)],
                     absQZ[static_cast<size_t>(vIdx)]};
        const MCGAL::Point target = mesh->intPosToFloat(MCGAL::PointInt(q.x, q.y, q.z));
        MCGAL::Facet* best = nullptr;
        double bestArea = -1.0;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
            MCGAL::Point a = f->proxyHalfedge()->vertex()->point();
            MCGAL::Point b = f->proxyHalfedge()->next()->vertex()->point();
            MCGAL::Point c = f->proxyHalfedge()->next()->next()->vertex()->point();
            auto cross = [](const MCGAL::Point& u, const MCGAL::Point& v) {
                return MCGAL::Point(u.y()*v.z()-u.z()*v.y(), u.z()*v.x()-u.x()*v.z(),
                                    u.x()*v.y()-u.y()*v.x());
            };
            MCGAL::Point ab(b.x()-a.x(), b.y()-a.y(), b.z()-a.z());
            MCGAL::Point ac(c.x()-a.x(), c.y()-a.y(), c.z()-a.z());
            MCGAL::Point n = cross(ab, ac);
            const double n2 = n.x()*n.x()+n.y()*n.y()+n.z()*n.z();
            if (n2 < 1e-24) continue;
            auto bary = [&](const MCGAL::Point& p, const MCGAL::Point& u, const MCGAL::Point& v) {
                auto cr = [](const MCGAL::Point& x, const MCGAL::Point& y) {
                    return MCGAL::Point(x.y()*y.z()-x.z()*y.y(), x.z()*y.x()-x.x()*y.z(),
                                        x.x()*y.y()-x.y()*y.x());
                };
                MCGAL::Point ap(p.x()-a.x(), p.y()-a.y(), p.z()-a.z());
                MCGAL::Point c1 = cr(u, ap), c2 = cr(v, ap);
                return std::array<double,3>{c1.x()*n.x()+c1.y()*n.y()+c1.z()*n.z(),
                                            c2.x()*n.x()+c2.y()*n.y()+c2.z()*n.z(),
                                            n2 - (c1.x()*n.x()+c1.y()*n.y()+c1.z()*n.z()) -
                                                 (c2.x()*n.x()+c2.y()*n.y()+c2.z()*n.z())};
            };
            auto w = bary(target, ab, ac);
            if (w[0] >= -1e-12 && w[1] >= -1e-12 && w[2] >= -1e-12) {
                best = f;
                break;
            }
            const double area = n2;
            if (area > bestArea) { bestArea = area; best = f; }
        }
        if (!best) break;
        MCGAL::Vertex* inserted = nullptr;
        if (!insertDecodedVertexOnFace(mesh, best, resX, resY, resZ, vIdx, &q, &inserted)) break;
        ++vIdx;
        ++R.numAddedVertices;
        ++R.numVValPopsConsumed;
    }
    R.numVerticesOut = livingVertexCount(mesh);
    return R;
}

RDPatchDecoderResult runRDPatchDecoderConquest(
    std::shared_ptr<MCGAL::Mesh> mesh,
    const std::vector<RDPatchSymbol>& decodedSymbols,
    const std::vector<int>& resX,
    const std::vector<int>& resY,
    const std::vector<int>& resZ,
    const std::vector<int>& faceRanks,
    const std::vector<unsigned char>& ringSizes,
    const std::vector<int>& ringQX,
    const std::vector<int>& ringQY,
    const std::vector<int>& ringQZ,
    const std::vector<int>& absQX,
    const std::vector<int>& absQY,
    const std::vector<int>& absQZ,
    const std::vector<int>& firstGateQ,
    const std::vector<unsigned char>& inputOffsets,
    const std::vector<std::array<int, 6>>& symbolInputQs,
    bool forcePMSFBoundaryGates,
    bool forceStrictRDConquest,
    const std::vector<std::array<int, 9>>& faceQ,
    const std::vector<unsigned char>& faceRingSizes,
    const std::vector<int>& faceRingQX,
    const std::vector<int>& faceRingQY,
    const std::vector<int>& faceRingQZ,
    const std::vector<unsigned char>& faceSide) {
    RDPatchDecoderResult R;
    auto strictFallbackFace = [&](int symIndex, bool allowAny = true) -> MCGAL::Halfedge* {
        if (!mesh) return nullptr;
        int vOrd = 0;
        for (int i = 0; i < symIndex && i < static_cast<int>(decodedSymbols.size()); ++i) {
            if (decodedSymbols[static_cast<size_t>(i)].kind != RDPatchSymbolKind::NULL_PATCH) ++vOrd;
        }
        std::size_t ringOff = 0;
        int ringN = 0;
        if (vOrd < static_cast<int>(faceRingSizes.size())) {
            for (int i = 0; i < vOrd; ++i) ringOff += faceRingSizes[static_cast<std::size_t>(i)];
            ringN = faceRingSizes[static_cast<std::size_t>(vOrd)];
        }
        const bool haveRing = ringN > 0 &&
                              ringOff + static_cast<std::size_t>(ringN) <= faceRingQX.size();
        auto ringMatches = [&](MCGAL::Vertex* v, int k) {
            return v && !v->isRemoved() &&
                   vertexMatchesQ(mesh, v,
                                  faceRingQX[ringOff + static_cast<std::size_t>(k)],
                                  faceRingQY[ringOff + static_cast<std::size_t>(k)],
                                  faceRingQZ[ringOff + static_cast<std::size_t>(k)]);
        };
        auto faceMatches = [&](MCGAL::Facet* f) {
            if (!f || f->isRemoved()) return false;
            MCGAL::Halfedge* h = f->proxyHalfedge();
            if (haveRing) {
                int matched = 0;
                for (int k = 0; k < ringN && h; ++k) {
                    bool found = false;
                    for (int j = 0; j < ringN; ++j) {
                        if (ringMatches(h->vertex(), j)) { found = true; break; }
                    }
                    if (!found) return false;
                    ++matched;
                    h = h->next();
                }
                return matched == ringN;
            }
            if (vOrd >= static_cast<int>(faceQ.size())) return false;
            const auto& loc = faceQ[static_cast<std::size_t>(vOrd)];
            bool anyNonZero = false;
            for (int k = 0; k < 9; ++k) if (loc[k] != 0) anyNonZero = true;
            if (!anyNonZero) return false;
            int matched = 0;
            for (int k = 0; k < 3 && h; ++k) {
                MCGAL::Vertex* v = h->vertex();
                if (!v || v->isRemoved()) return false;
                bool found = false;
                for (int j = 0; j < 3; ++j) {
                    if (vertexMatchesQ(mesh, v, loc[j * 3], loc[j * 3 + 1], loc[j * 3 + 2])) { found = true; break; }
                }
                if (found) ++matched;
                h = h->next();
            }
            return matched >= 3;
        };
        auto alignedHalfedge = [&](MCGAL::Facet* f) -> MCGAL::Halfedge* {
            if (!f || f->isRemoved()) return nullptr;
            MCGAL::Halfedge* h = f->proxyHalfedge();
            MCGAL::Halfedge* b = h;
            do {
                if (!h || h->isRemoved()) break;
                if (haveRing && ringMatches(h->vertex(), 0)) return h;
                if (!haveRing && vOrd < static_cast<int>(faceQ.size())) {
                    const auto& loc = faceQ[static_cast<std::size_t>(vOrd)];
                    if (vertexMatchesQ(mesh, h->vertex(), loc[0], loc[1], loc[2])) return h;
                }
                h = h ? h->next() : nullptr;
            } while (h && h != b);
            return f->proxyHalfedge();
        };
        for (MCGAL::Facet* f : mesh->faces()) {
            if (faceMatches(f)) return alignedHalfedge(f);
        }
        if (!allowAny) return nullptr;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved()) continue;
            MCGAL::Halfedge* h = f->proxyHalfedge();
            if (h && !h->isRemoved()) return h;
        }
        return nullptr;
    };
    R.numSymsAvailable = static_cast<int>(decodedSymbols.size());
    if (!mesh || decodedSymbols.empty()) {
        return R;
    }
    if (!faceRanks.empty() || !ringSizes.empty()) {
        return runDirectRankReplay(mesh, decodedSymbols, resX, resY, resZ, faceRanks,
                                   ringSizes, ringQX, ringQY, ringQZ,
                                   absQX, absQY, absQZ);
    }
    R.numVerticesIn = livingVertexCount(mesh);
    if (R.numVerticesIn == 0) {
        std::fprintf(stderr, "[RDPatchDecoderConquest] empty mesh\n");
        return R;
    }
    const bool useResiduals = !resX.empty()
                              && resX.size() == resY.size()
                              && resX.size() == resZ.size();

    // first_gate: 与 encoder 同规则在 base mesh 上选 longest_edge
    // ⚠️ M4.b 已知不对称：encoder 在 finest mesh 上选；此处在 base mesh 上选。
    // M5 之前需要写 metadata first_gate 量化坐标对（见 RDPatchDecoderConquest.h
    // 注释）。现在两端 first_gate 不一致，导致 sym 消费偏移。
    const bool usePMSFBoundaryGates =
        forceStrictRDConquest || forcePMSFBoundaryGates || envOn("PMSF_RDPATCH_PAPER_GATES");
    const bool forceSixBucket = forceStrictRDConquest;
    MCGAL::Halfedge* firstGate = findHalfedgeByInputQ(mesh, firstGateQ);
    const bool hasSerializedFirstGate = firstGate != nullptr;
    if (firstGate && usePMSFBoundaryGates) {
        MCGAL::Halfedge* activeGate = gateFromInputBoundary(firstGate);
        if (activeGate) firstGate = activeGate;
    }
    if (!firstGate) {
        firstGate = pickLongestEdge(mesh);
        if (usePMSFBoundaryGates) firstGate = gateFromInputBoundary(firstGate);
    }
    if (!firstGate) {
        std::fprintf(stderr, "[RDPatchDecoderConquest] no first_gate found\n");
        return R;
    }
    if (forceStrictRDConquest) {
        markGateInputTags(firstGate);
    }

    RDPatchGateQueue queue;
    std::set<MCGAL::Halfedge*> enqueued;
    std::set<MCGAL::Facet*> facesEnqueued;
    std::set<MCGAL::Facet*> facesProcessedInPhase;
    // PR-R (2026-05-12) mirror encoder: mesh-wide null patch face set
    std::set<MCGAL::Facet*> nullPatchFaces;

    auto seedQueue = [&]() {
        if ((usePMSFBoundaryGates || hasSerializedFirstGate) &&
            firstGate && !firstGate->isRemoved() &&
            firstGate->face() && !firstGate->face()->isRemoved()) {
            queue.push(firstGate, computeRealBucket(firstGate, nullPatchFaces, forceSixBucket));
            enqueued.insert(firstGate);
            if (firstGate->face()) facesEnqueued.insert(firstGate->face());
        } else {
            seedAllLivingFaces(mesh, queue, enqueued, facesEnqueued, nullPatchFaces,
                               forceSixBucket);
        }
    };
    seedQueue();

    int phase = 0;  // DEC=0 / CLN=1
    bool progressInCurrentCycle = false;
    int  cyclesCompleted = 0;
    int  step = 0;
    int  symIdx = 0;
    const int maxStepVertexFactor =
        envInt("PMSF_RDPATCH_DECODER_VERTEX_STEP_FACTOR", 4);
    const int maxSteps =
        R.numSymsAvailable + R.numVerticesIn * maxStepVertexFactor;  // 兜底

    auto isCleanV3 = [](RDPatchSymbolKind k) {
        return k == RDPatchSymbolKind::V_3;
    };
    bool hasSerializedPhase = false;
    int maxSerializedLayer = 0;
    for (const RDPatchSymbol& s : decodedSymbols) {
        if (s.phase == 1) {
            hasSerializedPhase = true;
        }
        maxSerializedLayer = std::max(maxSerializedLayer, static_cast<int>(s.layer));
    }
    const int strictMaxCycles = forceStrictRDConquest
                                    ? std::max(1, maxSerializedLayer + 1)
                                    : 1;

    // PR-R helper: V_v emit 后把 ring 周围 incident face setProcessedFlag + 给
    // ring 顶点 applyTagEvolution（mirror encoder 严格同序）。
    auto onVvEmittedRing = [](const RingSnapshot& snap) {
        for (MCGAL::Vertex* rv : snap.ringVerts) {
            if (!rv || rv->isRemoved()) continue;
            for (MCGAL::Halfedge* he : pmsf::boundedRing(rv)) {
                if (!he || he->isRemoved()) continue;
                MCGAL::Facet* f = he->face();
                if (f && !f->isRemoved()) f->setProcessedFlag();
            }
        }
        const int N = static_cast<int>(snap.ringVerts.size());
        if (N >= 3) {
            pclttm::Tag tagL = static_cast<pclttm::Tag>(snap.ringVerts[0]->getTag());
            pclttm::Tag tagR = static_cast<pclttm::Tag>(snap.ringVerts[N-1]->getTag());
            std::vector<pclttm::TagEvolutionRecord> trace;
            pclttm::applyTagEvolution(
                snap.ringVerts, tagL, tagR, pclttm::AnchorKind::V0First,
                [](MCGAL::Vertex* v) { return static_cast<pclttm::Tag>(v->getTag()); },
                [](MCGAL::Vertex* v, pclttm::Tag t) {
                    v->setTag(static_cast<MCGAL::Vertex::TagFlag>(t));
                },
                [](MCGAL::Vertex* v) -> int { return v ? v->poolId() : -1; },
                trace);
        }
    };

    auto orientPaperV3Gate = [&](const RDPatchSymbol& sym,
                                 MCGAL::Halfedge*& gate,
                                 MCGAL::Facet*& face,
                                 MCGAL::Vertex*& inputA,
                                 MCGAL::Vertex*& inputB,
                                 int desiredInputOffset) {
        if (!usePMSFBoundaryGates || symbolValence(sym) != 3 ||
            desiredInputOffset < 0 || !gate || gate->isRemoved() ||
            !inputA || !inputB) {
            return;
        }
        const bool desiredCanonicalForward = (desiredInputOffset & 0x8) == 0;
        const bool currentCanonicalForward = qLess(mesh, inputA, inputB);
        if (desiredCanonicalForward == currentCanonicalForward) return;

        MCGAL::Halfedge* input = inputBoundaryFromGate(gate);
        MCGAL::Halfedge* altInput = input ? input->opposite() : nullptr;
        MCGAL::Halfedge* altGate = gateFromInputBoundary(altInput);
        MCGAL::Facet* altFace = altGate ? altGate->face() : nullptr;
        if (!altGate || altGate->isRemoved() || !altFace || altFace->isRemoved()) return;
        gate = altGate;
        face = altFace;
        MCGAL::Halfedge* altBoundary = inputBoundaryFromGate(gate);
        if (altBoundary && altBoundary->vertex() && altBoundary->end_vertex()) {
            inputA = altBoundary->vertex();
            inputB = altBoundary->end_vertex();
        }
    };

    while (step < maxSteps) {
        // Strict replay fallback before phase switching: keep consuming the
        // serialized V symbols even when the boundary queue went empty.
        if (queue.empty() && forceStrictRDConquest && symIdx >= 0 &&
            symIdx < static_cast<int>(decodedSymbols.size())) {
            if (decodedSymbols[static_cast<size_t>(symIdx)].kind ==
                RDPatchSymbolKind::NULL_PATCH) {
                ++symIdx;
                ++R.numSymbolsTotal;
                ++R.numNullPopsConsumed;
                continue;
            }
            for (MCGAL::Facet* f : mesh->faces()) {
                if (!f || f->isRemoved()) continue;
                MCGAL::Halfedge* pH = f->proxyHalfedge();
                if (!pH || pH->isRemoved()) continue;
                queue.push(pH, 1);
                break;
            }
        }
        // Phase 切换
        if (queue.empty()) {
            if (phase == 0) {
                // D17 闭合定位（门控 PMSF_D17_PHASE=1）：在 DEC→CLN 相位切换处
                // 普查一次"环不闭合"数，用于把破坏闭合的操作**二分**到
                // decimating 阶段还是 cleaning 阶段。
                {
                    static const bool kD17Phase = [] {
                        const char* e = std::getenv("PMSF_D17_PHASE");
                        return e && e[0] == '1';
                    }();
                    if (kD17Phase) {
                        long long live = 0, bad = 0;
                        for (MCGAL::Vertex* vv : mesh->vertices()) {
                            if (!vv || vv->isRemoved()) continue;
                            ++live;
                            if (!pmsf::vertexRingCloses(vv)) ++bad;
                        }
                        std::fprintf(stderr,
                                     "  [D17phase] DEC 相位结束: live=%lld 环不闭合=%lld\n",
                                     live, bad);
                    }
                }
                phase = 1;
                                enqueued.clear();
                facesEnqueued.clear();
                facesProcessedInPhase.clear();
                nullPatchFaces.clear();
                if (forceStrictRDConquest) {
                    resetLivingFaceProcessedFlags(mesh);
                    if (usePMSFBoundaryGates && !forceStrictRDConquest) {
                        resetLivingVertexConqueredFlags(mesh);
                    }
                }
                seedQueue();
                continue;
            } else {
                cyclesCompleted++;
                R.numDecCycles = cyclesCompleted;
                                if (!progressInCurrentCycle) break;
                // PR-O mirror encoder maxCycles=1：encoder 限制 1 cycle，decoder 同
                const int kMaxCycles = strictMaxCycles;
                if (cyclesCompleted >= kMaxCycles) {
                                        break;
                }
                phase = 0;
                progressInCurrentCycle = false;
                enqueued.clear();
                facesEnqueued.clear();
                facesProcessedInPhase.clear();
                nullPatchFaces.clear();
                if (forceStrictRDConquest) {
                    resetLivingFaceProcessedFlags(mesh);
                    if (usePMSFBoundaryGates && !forceStrictRDConquest) {
                        resetLivingVertexConqueredFlags(mesh);
                    }
                }
                seedQueue();
                continue;
            }
        }

        if (symIdx >= R.numSymsAvailable) {
            R.exhaustedSymbols = true;
                        break;
        }

        if (!forceStrictRDConquest &&
            hasSerializedPhase &&
            decodedSymbols[static_cast<size_t>(symIdx)].phase <= 1 &&
            decodedSymbols[static_cast<size_t>(symIdx)].phase != phase) {
            phase = decodedSymbols[static_cast<size_t>(symIdx)].phase;
            enqueued.clear();
            facesEnqueued.clear();
            facesProcessedInPhase.clear();
            nullPatchFaces.clear();
            queue.clear();
            if (forceStrictRDConquest) {
                resetLivingFaceProcessedFlags(mesh);
                if (usePMSFBoundaryGates && !forceStrictRDConquest) {
                    resetLivingVertexConqueredFlags(mesh);
                }
            }
            seedQueue();
            if (queue.empty()) break;
        }

        int popBucket = -1;
        MCGAL::Halfedge* h = nullptr;
        const bool forceDirectGate = envOn("PMSF_RDPATCH_FORCE_SYMBOL_GATE");
        if (usePMSFBoundaryGates &&
            symIdx >= 0 &&
            symIdx < static_cast<int>(symbolInputQs.size())) {
            const auto& desiredQ = symbolInputQs[static_cast<size_t>(symIdx)];
            h = popGateByInputQ(queue, mesh, desiredQ, &popBucket);
            if (!h && forceDirectGate) {
                h = findHalfedgeByInputQ(mesh, desiredQ);
            }
        }
        if (!h) {
            h = queue.popHighestPriority(&popBucket);
        }
        if (forceStrictRDConquest && symIdx >= 0 &&
            symIdx < static_cast<int>(decodedSymbols.size()) &&
            decodedSymbols[static_cast<size_t>(symIdx)].kind !=
                RDPatchSymbolKind::NULL_PATCH) {
            if (symIdx < static_cast<int>(symbolInputQs.size())) {
                MCGAL::Halfedge* e = findHalfedgeByInputQ(mesh, symbolInputQs[static_cast<size_t>(symIdx)]);
                if (e) {
                    unsigned char side = (symIdx < static_cast<int>(faceSide.size()))
                                             ? faceSide[static_cast<size_t>(symIdx)]
                                             : 0;
                    if (const char* flip = std::getenv("PMSF_RDSTRICT_FLIP_SIDE")) {
                        if (std::atoi(flip) == symIdx) side ^= 1;
                    }
                    MCGAL::Halfedge* chosen = side ? e->opposite() : e;
                    if (!chosen || chosen->isRemoved() || !chosen->face() || chosen->face()->isRemoved()) {
                        chosen = side ? e : e->opposite();
                    }
                    if (chosen && !chosen->isRemoved() && chosen->face() && !chosen->face()->isRemoved()) {
                        h = chosen;
                    }
                }
            }
            if (!faceRingSizes.empty()) {
                if (MCGAL::Halfedge* locH = strictFallbackFace(symIdx, false)) {
                    h = locH;
                }
            } else if (!faceQ.empty() && !h) {
                if (MCGAL::Halfedge* locH = strictFallbackFace(symIdx, false)) {
                    h = locH;
                }
            }
        }
        // Strict replay fallback: when the boundary queue is exhausted but the
        // next serialized symbol is a V-code, pick any alive unprocessed face so
        // the symbol is consumed. Geometry still comes from absQ; this keeps the
        // removal/insert count aligned instead of silently dropping the symbol.
        if (!h && forceStrictRDConquest && symIdx >= 0 &&
            symIdx < static_cast<int>(decodedSymbols.size()) &&
            decodedSymbols[static_cast<size_t>(symIdx)].kind !=
                RDPatchSymbolKind::NULL_PATCH) {
            for (MCGAL::Facet* f : mesh->faces()) {
                if (!f || f->isRemoved()) {
                    continue;
                }
                MCGAL::Halfedge* pH = f->proxyHalfedge();
                if (!pH || pH->isRemoved()) continue;
                h = pH;
                break;
            }
        }
        if (!h || h->isRemoved()) {
            if (forceStrictRDConquest && symIdx >= 0 &&
                symIdx < static_cast<int>(decodedSymbols.size()) &&
                decodedSymbols[static_cast<size_t>(symIdx)].kind ==
                    RDPatchSymbolKind::NULL_PATCH) {
                ++symIdx;
                ++R.numSymbolsTotal;
                ++R.numNullPopsConsumed;
            }
            continue;
        }

        MCGAL::Facet* gateFace = h->face();
        // Strict replay must consume every symbol; encoder boundary-gate walk
        // may revisit a face that this decoder already marked processed.
        if (!forceStrictRDConquest && gateFace &&
            facesProcessedInPhase.count(gateFace)) {
            continue;
        }

        if (popBucket >= 1 && popBucket <= 6) {
            R.bucketHits[popBucket - 1]++;
        }
        ++step;
        R.numSteps = step;
        
        // 拿 sym 之前不 consume；按 phase 决定如何消费
        const RDPatchSymbol& sym = decodedSymbols[symIdx];

        if (phase == 0) {
            // DEC: 总是 consume 一个 sym（NULL_PATCH 或 V_v）
            traceDecoderGate(mesh, symIdx, phase,
                             sym.kind == RDPatchSymbolKind::NULL_PATCH ? "N" : "V",
                             h, -1);
            ++symIdx;
            R.numSymbolsTotal++;
            if (sym.kind == RDPatchSymbolKind::NULL_PATCH) {
                                R.numNullPopsConsumed++;
                if (gateFace) facesProcessedInPhase.insert(gateFace);
                // PR-R mirror encoder：标记 face 为 null + face::isProcessed
                if (gateFace) {
                    gateFace->setProcessedFlag();
                    nullPatchFaces.insert(gateFace);
                }
                if (usePMSFBoundaryGates) {
                                        pushPaperNullPatchNeighbors(h, queue, enqueued, facesEnqueued, nullPatchFaces,
                                                forceSixBucket);
                } else {
                    pushNullPatchNeighbors(h, queue, enqueued, facesEnqueued, nullPatchFaces,
                                           forceSixBucket);
                }
                            } else {
                // V_v：在 face 内 add center vertex
                R.numVValPopsConsumed++;
                if (!gateFace || gateFace->isRemoved()) {
                    if (gateFace) facesProcessedInPhase.insert(gateFace);
                    continue;
                }
                const int vIndex = R.numVValPopsConsumed - 1;
                if (vIndex >= 0 && vIndex < static_cast<int>(faceRanks.size())) {
                    MCGAL::Facet* ranked =
                        livingFaceByRank(mesh, faceRanks[static_cast<size_t>(vIndex)]);
                    if (ranked) gateFace = ranked;
                }
                MCGAL::Vertex* inputBoundaryA = nullptr;
                MCGAL::Vertex* inputBoundaryB = nullptr;
                if (h && h->next() && h->next()->next()) {
                    MCGAL::Halfedge* inputBoundary = h->next()->next();
                    inputBoundaryA = inputBoundary->vertex();
                    inputBoundaryB = inputBoundary->end_vertex();
                }
                const int desiredInputOffset =
                    (usePMSFBoundaryGates && vIndex >= 0 &&
                     vIndex < static_cast<int>(inputOffsets.size()))
                        ? static_cast<int>(inputOffsets[static_cast<size_t>(vIndex)])
                        : -1;
                orientPaperV3Gate(sym, h, gateFace, inputBoundaryA, inputBoundaryB,
                                  desiredInputOffset);
                                const int d17Before = d17LocalEnabled() ? d17CountBadOnFace(gateFace) : 0;
                gateFace = mergePatchForSymbol(gateFace, sym, usePMSFBoundaryGates,
                                               mesh, inputBoundaryA, inputBoundaryB,
                                               desiredInputOffset,
                                               forceStrictRDConquest);
                if (gateFace) {
                    const int d17Fixed = repairRingClosureOnFace(gateFace);
                    if (d17Fixed > 0) ++g_d17Fixed;
                }
                if (d17LocalEnabled()) {
                    const int d17After = d17CountBadOnFace(gateFace);
                    if (d17After > d17Before) {
                        ++g_d17MergeBroke;
                        if (g_d17MergeBroke <= 6) {
                            std::fprintf(stderr,
                                         "  [D17local] **mergePatchForSymbol 破坏闭合** step=%d "
                                         "before=%d after=%d\n", step, d17Before, d17After);
                        }
                    }
                }
                if (!gateFace || gateFace->isRemoved()) {
                    continue;
                }
                if (usePMSFBoundaryGates) {
                    gateFace->normalize_proxy_to_min_position_vertex(mesh->bboxMin, mesh->f_quantStep);
                }
                                MCGAL::Halfedge* faceProxy = gateFace->proxyHalfedge();
                if (!faceProxy) faceProxy = h;
                RingSnapshot polygonSnap = snapshotFaceRing(gateFace);
                // P5.5 诊断（门控，默认关）：解码端 gateFace 的环 + 其重心，
                // 与编码端 [encRing] 对拍，判"顶点集合是否相同"（重心只取决于
                // 集合，与朝向/起点无关）。
                if (envOn("PMSF_RDPATCH_RING_DBG") && R.numAddedVertices < 5) {
                    MCGAL::PointInt db = MCGAL::rdBarycenterQuantized(gateFace, mesh);
                    std::fprintf(stderr, "  [decRing] sym=%d decBary=(%d,%d,%d) ring=",
                                 R.numAddedVertices, db[0], db[1], db[2]);
                    for (MCGAL::Vertex* rv : polygonSnap.ringVerts) {
                        if (!rv) continue;
                        MCGAL::PointInt qv = mesh->floatPosToInt(rv->point());
                        std::fprintf(stderr, "(%d,%d,%d)", qv[0], qv[1], qv[2]);
                    }
                    std::fprintf(stderr, " deg=%d\n", faceDegree(gateFace));
                }
                // P5.5 判决性探针（临时，门控，默认关）：
                // "编码端的环顶点在解码端此刻的网格里**存在吗**"。
                // 不存在 ⇒ 编码端的多边形活在 fine 网格上，而解码端此刻手里是
                // coarse 网格 + 已插入点，**不是同一个网格** ⇒ A 路（解码端自行
                // 推出 encBaryQ）结构性不可达，absQ 这类与网格状态无关的绝对量
                // 是唯一稳健的预测子。
                // 存在 ⇒ 假设被推翻，问题回到"候选 patch 池为何取不到它"。
                // 探测步可配：逆向重放下第 k 步需要的是编码端第 N-1-k 个符号的
                // polyFace。判定"前提只对最后一个符号成立"还是"整条链都成立"，
                // 这决定逆向重放是可实现还是不可实现。
                // static：避免每次插入都做一遍 getenv（这是热循环）。默认无 probe
                // 时整段被跳过，零开销。
                std::vector<MCGAL::Halfedge*> paperDecimatingGates;
                if (usePMSFBoundaryGates) {
                    paperDecimatingGates =
                        collectPaperDecimatingGatesFromPolygon(gateFace, inputBoundaryA,
                                                               inputBoundaryB);
                }
                MCGAL::Point insertPoint;
                // P5.1 修复：本分支（usePMSFBoundaryGates=0 时解码端**实际**的
                // 插入点）此前完全不看 absQ —— 位置一律由「解码端自己重建出的
                // gateFace 的多边形重心 + 残差」推出。一旦 gateFace 与编码端的
                // polygonFace 不同源，重心就不同源，位置随之全错：实测 A2 命中
                // 恒等于 V_base（只有 base mesh 顶点落地），外部 Hausdorff 0.039
                // vs ppmc 同网格 3.4e-05。
                // 侧信息流里原本就带着该顶点的**绝对量化坐标**（与重心无关），
                // 只是从没人读它。有则优先采用——位置自此不再依赖选面是否正确；
                // 没有任何字节被新增或改写，只是让已有的值真正生效。
                if (vIndex >= 0 && vIndex < static_cast<int>(absQX.size()) &&
                    vIndex < static_cast<int>(absQY.size()) &&
                    vIndex < static_cast<int>(absQZ.size())) {
                    MCGAL::PointInt vQ(absQX[static_cast<size_t>(vIndex)],
                                       absQY[static_cast<size_t>(vIndex)],
                                       absQZ[static_cast<size_t>(vIndex)]);
                    insertPoint = mesh->intPosToFloat(vQ);
                    ++g_absqPathCount;
                    // ★ 闭环预测可行性实测（风险 3）：闭环残差 = vQuant − decBaryQ。
                    // 解码端此刻两样都有（absQ 来自侧信道，decBaryQ 自己算得出）。
                    // 统计它做成 zigzag+varint 需要多少字节，与现行残差流对比，
                    // 即可判定闭环方案能否把 bpv 压进 1.20。
                    // **必须门控**：这段每次插入都要算一次 rdBarycenterQuantized，
                    // 是实打实的额外开销，不能留在默认路径上。
                    static const bool kClResidMeasure = [] {
                        const char* e = std::getenv("PMSF_RDPATCH_CLRESID_MEASURE");
                        return e && e[0] == '1';
                    }();
                    if (kClResidMeasure) {
                        const MCGAL::PointInt cbq =
                            MCGAL::rdBarycenterQuantized(gateFace, mesh);
                        const int32_t dx = vQ[0] - cbq[0];
                        const int32_t dy = vQ[1] - cbq[1];
                        const int32_t dz = vQ[2] - cbq[2];
                        g_clResidAccum.n++;
                        g_clResidAccum.sumAbs += std::abs(dx) + std::abs(dy) + std::abs(dz);
                        g_clResidAccum.maxAbs = std::max(
                            g_clResidAccum.maxAbs,
                            std::max(std::abs(dx), std::max(std::abs(dy), std::abs(dz))));
                        auto vlen = [](int32_t v) {
                            uint32_t z = (static_cast<uint32_t>(v) << 1) ^
                                         static_cast<uint32_t>(v >> 31);
                            int n = 1;
                            while (z >= 0x80u) { z >>= 7; ++n; }
                            return n;
                        };
                        g_clResidAccum.varintBytes +=
                            vlen(dx) + vlen(dy) + vlen(dz);
                    }
                } else if (useResiduals && vIndex >= 0 &&
                           vIndex < (int)resX.size()) {
                    // D14：落到**残差兜底**的插入 —— 位置 = 解码端重心 + 残差，
                    // 与 absQ 无关，必然是错的。深轮数那几个坏顶点最可能出自这里。
                    ++g_residFallbackCount;
                    // PR-M5 sv=2: vertex_quant = face_bary_quant + residual
                    MCGAL::PointInt baryQ = MCGAL::rdBarycenterQuantized(gateFace, mesh);
                    MCGAL::PointInt vQ(baryQ[0] + resX[static_cast<size_t>(vIndex)],
                                       baryQ[1] + resY[static_cast<size_t>(vIndex)],
                                       baryQ[2] + resZ[static_cast<size_t>(vIndex)]);
                    insertPoint = mesh->intPosToFloat(vQ);
                } else {
                    // M4.b 占位：face barycenter
                    insertPoint = MCGAL::barycenter(faceProxy);
                }
                                insertPoint = safeInteriorInsertPoint(faceProxy, insertPoint);
                                MCGAL::Halfedge* newH =
                    mesh->create_center_vertex_without_init(faceProxy, insertPoint);
                ++g_ccvSiteB;
                repairNewVertexProxyHalfedge(newH);
                repairNewVertexHalfedgeRing(newH);
                                MCGAL::Vertex* centerV = nullptr;
                if (newH && usePMSFBoundaryGates && !mesh->vertices().empty()) {
                    centerV = mesh->vertices().back();
                }
                if (!centerV) centerV = insertedCenterVertex(newH, insertPoint);
                if (centerV) {
                    R.numAddedVertices++;
                    progressInCurrentCycle = true;
                    // D15 诊断（门控，默认关）：**新插入的顶点是否被接进 halfedge 结构**。
                    // 实测 gates=0 路径下，插入的顶点全部 `halfedges()` 为空
                    // （1 轮 16146 个、26 轮 34786 个，恰等于插入次数）。
                                        // D14：顶点最终位置是否就是请求的 insertPoint？
                    // absQ 路径下 insertPoint = intPosToFloat(absQ)，而 absQ ∈ origQ，
                    // 所以这里若对不上，说明位置在"请求 → 落点"之间被改过。
                }
                if (gateFace) facesProcessedInPhase.insert(gateFace);

                if (centerV) {
                    RingSnapshot snap = usePMSFBoundaryGates ? polygonSnap : snapshotRing(centerV);
                    if (forceStrictRDConquest && usePMSFBoundaryGates) {
                        applyFig9TagsToRing(snap.ringVerts);
                    }
                    traceDecoderOp(mesh, R.numVValPopsConsumed - 1, phase,
                                   symbolValence(sym), inputBoundaryA, inputBoundaryB,
                                   snap.ringVerts);
                                        if (usePMSFBoundaryGates) {
                        for (MCGAL::Halfedge* gate : paperDecimatingGates) {
                            pushGateIfUsable(gate, queue, enqueued, facesEnqueued,
                                             nullPatchFaces, forceSixBucket);
                        }
                    } else {
                        // PR-R: 标记 ring 周围 face::isProcessed + applyTagEvolution
                        onVvEmittedRing(snap);
                        pushNewGates(snap, queue, enqueued, facesEnqueued, nullPatchFaces,
                                     forceSixBucket);
                    }
                                    }
            }
        } else {
            // CLN: 只期望 V_3 sym
            if (isCleanV3(sym.kind)) {
                traceDecoderGate(mesh, symIdx, phase, "V",
                                 h, -1);
                ++symIdx;
                R.numSymbolsTotal++;
                R.numVValPopsConsumed++;
                if (!gateFace || gateFace->isRemoved()) {
                    if (gateFace) facesProcessedInPhase.insert(gateFace);
                    continue;
                }
                const int vIndex = R.numVValPopsConsumed - 1;
                if (vIndex >= 0 && vIndex < static_cast<int>(faceRanks.size())) {
                    MCGAL::Facet* ranked =
                        livingFaceByRank(mesh, faceRanks[static_cast<size_t>(vIndex)]);
                    if (ranked) gateFace = ranked;
                }
                MCGAL::Vertex* inputBoundaryA = nullptr;
                MCGAL::Vertex* inputBoundaryB = nullptr;
                if (h && h->next() && h->next()->next()) {
                    MCGAL::Halfedge* inputBoundary = h->next()->next();
                    inputBoundaryA = inputBoundary->vertex();
                    inputBoundaryB = inputBoundary->end_vertex();
                }
                const int desiredInputOffset =
                    (usePMSFBoundaryGates && vIndex >= 0 &&
                     vIndex < static_cast<int>(inputOffsets.size()))
                        ? static_cast<int>(inputOffsets[static_cast<size_t>(vIndex)])
                        : -1;
                orientPaperV3Gate(sym, h, gateFace, inputBoundaryA, inputBoundaryB,
                                  desiredInputOffset);
                const int d17Before = d17LocalEnabled() ? d17CountBadOnFace(gateFace) : 0;
                gateFace = mergePatchForSymbol(gateFace, sym, usePMSFBoundaryGates,
                                               mesh, inputBoundaryA, inputBoundaryB,
                                               desiredInputOffset,
                                               forceStrictRDConquest);
                if (gateFace) {
                    const int d17Fixed = repairRingClosureOnFace(gateFace);
                    if (d17Fixed > 0) ++g_d17Fixed;
                }
                if (d17LocalEnabled()) {
                    const int d17After = d17CountBadOnFace(gateFace);
                    if (d17After > d17Before) {
                        ++g_d17MergeBroke;
                        if (g_d17MergeBroke <= 6) {
                            std::fprintf(stderr,
                                         "  [D17local] **mergePatchForSymbol 破坏闭合** step=%d "
                                         "before=%d after=%d\n", step, d17Before, d17After);
                        }
                    }
                }
                if (!gateFace || gateFace->isRemoved()) {
                    continue;
                }
                if (usePMSFBoundaryGates) {
                    gateFace->normalize_proxy_to_min_position_vertex(mesh->bboxMin, mesh->f_quantStep);
                }
                MCGAL::Halfedge* faceProxy = gateFace->proxyHalfedge();
                if (!faceProxy) faceProxy = h;
                RingSnapshot polygonSnap = snapshotFaceRing(gateFace);
                MCGAL::Point insertPoint;
                // D14 修复：**CLN 相位的插入此前完全不看 absQ** —— 位置一律由
                // 「解码端重心 + 残差」推出，而解码端重心与编码端不同源 ⇒ 位置错。
                // 编码端 vvAbsQX/Y/Z 是在**公共 emit 路径**上按符号 push 的
                //（RDPatchPipeline.cpp:1656），CLN 相位的符号同样有对应条目，
                // 所以这里可与 DEC 相位一样直接取用；索引口径也一致，
                // 两处都用 R.numVValPopsConsumed - 1。
                const int clnIdx = R.numVValPopsConsumed - 1;
                if (clnIdx >= 0 && clnIdx < static_cast<int>(absQX.size()) &&
                    clnIdx < static_cast<int>(absQY.size()) &&
                    clnIdx < static_cast<int>(absQZ.size())) {
                    MCGAL::PointInt vQ(absQX[static_cast<size_t>(clnIdx)],
                                       absQY[static_cast<size_t>(clnIdx)],
                                       absQZ[static_cast<size_t>(clnIdx)]);
                    insertPoint = mesh->intPosToFloat(vQ);
                } else if (useResiduals && clnIdx >= 0 &&
                           clnIdx < (int)resX.size()) {
                    MCGAL::PointInt baryQ = MCGAL::rdBarycenterQuantized(gateFace, mesh);
                    MCGAL::PointInt vQ(baryQ[0] + resX[static_cast<size_t>(clnIdx)],
                                       baryQ[1] + resY[static_cast<size_t>(clnIdx)],
                                       baryQ[2] + resZ[static_cast<size_t>(clnIdx)]);
                    insertPoint = mesh->intPosToFloat(vQ);
                } else {
                    insertPoint = MCGAL::barycenter(faceProxy);
                }
                insertPoint = safeInteriorInsertPoint(faceProxy, insertPoint);
                MCGAL::Halfedge* newH =
                    mesh->create_center_vertex_without_init(faceProxy, insertPoint);
                ++g_ccvSiteC;
                repairNewVertexProxyHalfedge(newH);
                repairNewVertexHalfedgeRing(newH);
                MCGAL::Vertex* centerV = nullptr;
                if (newH && usePMSFBoundaryGates && !mesh->vertices().empty()) {
                    centerV = mesh->vertices().back();
                }
                if (!centerV) centerV = insertedCenterVertex(newH, insertPoint);
                if (centerV) {
                    R.numAddedVertices++;
                    progressInCurrentCycle = true;
                }
                if (gateFace) facesProcessedInPhase.insert(gateFace);
                if (centerV) {
                    RingSnapshot snap = usePMSFBoundaryGates ? polygonSnap : snapshotRing(centerV);
                    if (forceStrictRDConquest && usePMSFBoundaryGates) {
                        applyFig9TagsToRing(snap.ringVerts);
                    }
                    traceDecoderOp(mesh, R.numVValPopsConsumed - 1, phase,
                                   symbolValence(sym), inputBoundaryA, inputBoundaryB,
                                   snap.ringVerts);
                    if (usePMSFBoundaryGates) {
                        (void)inputBoundaryA;
                        (void)inputBoundaryB;
                        pushPaperCleaningGates(snap.ringVerts, mesh, queue, enqueued, facesEnqueued,
                                               facesProcessedInPhase, nullPatchFaces,
                                               forceSixBucket);
                    } else {
                        // PR-R: mirror encoder
                        onVvEmittedRing(snap);
                        pushNewGates(snap, queue, enqueued, facesEnqueued, nullPatchFaces,
                                     forceSixBucket);
                    }
                }
            } else {
                if (forceStrictRDConquest && sym.kind == RDPatchSymbolKind::NULL_PATCH) {
                    traceDecoderGate(mesh, symIdx, phase, "N", h, -1);
                    ++symIdx;
                    R.numSymbolsTotal++;
                    R.numNullPopsConsumed++;
                    if (gateFace) {
                        facesProcessedInPhase.insert(gateFace);
                        gateFace->setProcessedFlag();
                        nullPatchFaces.insert(gateFace);
                    }
                } else {
                    // PR-Q (2026-05-12) mirror encoder：CLN phase non-V_3 face 不消费 sym
                    // 但仍要 push 邻居扩散，让 conquest 能 reach valence-3 顶点。
                    if (gateFace) facesProcessedInPhase.insert(gateFace);
                    // PR-R: legacy CLN non-V_3 face 不发 sym，但也算 processed。
                    if (gateFace) gateFace->setProcessedFlag();
                }
                if (usePMSFBoundaryGates) {
                    pushPaperNullPatchNeighbors(h, queue, enqueued, facesEnqueued, nullPatchFaces,
                                                forceSixBucket);
                } else {
                    pushNullPatchNeighbors(h, queue, enqueued, facesEnqueued, nullPatchFaces,
                                           forceSixBucket);
                }
            }
        }
    }

    if (step >= maxSteps) {
        R.reachedMaxSteps = true;
        std::fprintf(stderr,
                     "[RDPatchDecoderConquest] WARNING: reached max_steps=%d, "
                     "abort（V_in=%d, added=%d, sym_consumed=%d/%d）\n",
                     maxSteps, R.numVerticesIn, R.numAddedVertices,
                     R.numSymbolsTotal, R.numSymsAvailable);
    }

    if (!usePMSFBoundaryGates && symIdx < R.numSymsAvailable) {
        for (; symIdx < R.numSymsAvailable; ++symIdx) {
            const RDPatchSymbol& sym = decodedSymbols[static_cast<size_t>(symIdx)];
            ++R.numSymbolsTotal;
            if (sym.kind == RDPatchSymbolKind::NULL_PATCH) {
                ++R.numNullPopsConsumed;
                continue;
            }
            const int residualIndex = R.numVValPopsConsumed;
            MCGAL::Facet* f = nullptr;
            if (residualIndex < static_cast<int>(faceRanks.size())) {
                f = livingFaceByRank(mesh, faceRanks[static_cast<size_t>(residualIndex)]);
            }
            if (!f) f = firstUsableFace(mesh);
            // P5.1：absQ 通道此前**从未被使用**——本调用点第 6 参是 residualIndex，
            // absQ 一直走默认 nullptr ⇒ 即使侧信息到了也只能走 barycenter。
            // 现在把侧信息里的绝对量化坐标传进去；命中该分支会完全绕过 barycenter
            // （insertDecodedVertexOnFace 内 absQ 分支：insertPoint = intPosToFloat(vQ)）。
            const QKey* absQPtr = nullptr;
            QKey absQVal{};
            if (residualIndex < static_cast<int>(absQX.size()) &&
                residualIndex < static_cast<int>(absQY.size()) &&
                residualIndex < static_cast<int>(absQZ.size())) {
                absQVal = QKey{absQX[static_cast<size_t>(residualIndex)],
                               absQY[static_cast<size_t>(residualIndex)],
                               absQZ[static_cast<size_t>(residualIndex)]};
                absQPtr = &absQVal;
            }
            if (!insertDecodedVertexOnFace(mesh, f, resX, resY, resZ, residualIndex,
                                           absQPtr)) {
                break;
            }
            ++R.numVValPopsConsumed;
            ++R.numAddedVertices;
        }
    }

    R.numVerticesOut = livingVertexCount(mesh);
    repairDegenerateFaces(mesh);
    if (g_d17Checked > 0) {
        std::fprintf(stderr, "  [D17] 建环自检执行 %lld 次，其中不闭合 %lld 次\n",
                     g_d17Checked, g_d17NonClosing);
        g_d17Checked = 0;
    }
    if (g_d17NonClosing > 0) {
        std::fprintf(stderr, "  [D17] 本轮建环处检测到**不闭合**的环：%lld 个\n",
                     g_d17NonClosing);
        g_d17NonClosing = 0;
    }
    if (g_absqPathCount || g_residFallbackCount) {
        std::fprintf(stderr,
                     "  [D14] 本轮插入：absQ 路径=%lld  残差兜底=%lld  落点≠请求=%lld\n",
                     g_absqPathCount, g_residFallbackCount, g_posMismatch);
        g_absqPathCount = 0;
        g_residFallbackCount = 0;
        g_posMismatch = 0;
        std::fprintf(stderr,
                     "  [D14] 建点次数：OnFace(A)=%lld  gateB=%lld  gateC=%lld\n",
                     g_ccvSiteA, g_ccvSiteB, g_ccvSiteC);
        g_ccvSiteA = g_ccvSiteB = g_ccvSiteC = 0;
    }
    if (g_clResidAccum.n > 0) {
        std::fprintf(stderr,
                     "  [clResid] n=%lld  avg|r|/comp=%.2f  max|r|=%d  "
                     "varint 合计=%lld B  => %.3f B/符号\n",
                     g_clResidAccum.n,
                     (double)g_clResidAccum.sumAbs / (3.0 * (double)g_clResidAccum.n),
                     g_clResidAccum.maxAbs, g_clResidAccum.varintBytes,
                     (double)g_clResidAccum.varintBytes / (double)g_clResidAccum.n);
        g_clResidAccum = ClResidAccum{};
    }
    return R;
}

RDPatchPipelineResult rebuildRDPatchResultInStrictDecodeOrder(
    std::shared_ptr<MCGAL::Mesh> mesh,
    const RDPatchPipelineResult& forward) {
    struct InputKey {
        std::array<int, 6> q{};
        bool operator==(const InputKey& o) const { return q == o.q; }
    };
    struct InputKeyHash {
        size_t operator()(const InputKey& k) const {
            uint64_t h = 1469598103934665603ULL;
            for (int v : k.q) {
                h ^= static_cast<uint32_t>(v);
                h *= 1099511628211ULL;
            }
            return static_cast<size_t>(h);
        }
    };
    struct PendingRemoval {
        RDPatchSymbol sym{};
        std::array<int, 6> inputQ{};
        std::vector<QKey> ringQ;
        int rx = 0;
        int ry = 0;
        int rz = 0;
        int ax = 0;
        int ay = 0;
        int az = 0;
        unsigned char inputOffset = 0;
        int forwardIndex = -1;
        bool used = false;
    };
    struct EdgeKey {
        QKey a{};
        QKey b{};
        bool operator==(const EdgeKey& o) const { return a == o.a && b == o.b; }
    };
    struct EdgeKeyHash {
        size_t operator()(const EdgeKey& e) const {
            QKeyHash h;
            return h(e.a) ^ (h(e.b) + 0x9e3779b97f4a7c15ULL + (h(e.a) << 6) + (h(e.a) >> 2));
        }
    };
    auto qKeyLessLocal = [](const QKey& a, const QKey& b) {
        if (a.x != b.x) return a.x < b.x;
        if (a.y != b.y) return a.y < b.y;
        return a.z < b.z;
    };
    auto makeEdgeKey = [&](QKey a, QKey b) {
        if (qKeyLessLocal(b, a)) std::swap(a, b);
        return EdgeKey{a, b};
    };
    auto ringSetKeyFromQ = [&](std::vector<QKey> qs) {
        std::sort(qs.begin(), qs.end(), qKeyLessLocal);
        std::string key;
        key.reserve(qs.size() * 36);
        for (const QKey& q : qs) {
            key += std::to_string(q.x);
            key.push_back(',');
            key += std::to_string(q.y);
            key.push_back(',');
            key += std::to_string(q.z);
            key.push_back(';');
        }
        return key;
    };
    auto ringSetKeyFromVertices = [&](const std::vector<MCGAL::Vertex*>& ring) {
        std::vector<QKey> qs;
        qs.reserve(ring.size());
        for (MCGAL::Vertex* v : ring) {
            if (!v || v->isRemoved()) return std::string();
            MCGAL::PointInt q = mesh->floatPosToInt(v->point());
            qs.push_back({q[0], q[1], q[2]});
        }
        return ringSetKeyFromQ(std::move(qs));
    };

    RDPatchPipelineResult out;
    out.mesh = mesh;
    out.numVerticesIn = forward.numVerticesIn;
    out.numVerticesOut = forward.numVerticesOut;
    out.leakyFactors = forward.leakyFactors;
    if (!mesh) return out;
    mesh->resetState();
    mesh->invalidateAllVertexDegrees();
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (v && !v->isRemoved()) v->clearTag();
    }

    std::vector<PendingRemoval> pending;
    pending.reserve(forward.vvResidualsX.size());
    std::unordered_map<InputKey, std::vector<int>, InputKeyHash> byInput;
    std::unordered_map<EdgeKey, std::vector<int>, EdgeKeyHash> byRingEdge;
    std::unordered_map<std::string, std::vector<int>> byRingSet;
    struct RingAdjacencyNeed {
        int ownerIdx = -1;
        int ownerForwardIndex = -1;
        QKey prev{};
        QKey next{};
    };
    std::unordered_map<QKey, std::vector<RingAdjacencyNeed>, QKeyHash> byRingVertex;
    size_t ringCoord = 0;
    int vCursor = 0;
    for (size_t si = 0; si < forward.symbols.size(); ++si) {
        const RDPatchSymbol& s = forward.symbols[si];
        if (s.kind == RDPatchSymbolKind::NULL_PATCH) continue;
        if (vCursor >= static_cast<int>(forward.vvResidualsX.size()) ||
            vCursor >= static_cast<int>(forward.vvResidualsY.size()) ||
            vCursor >= static_cast<int>(forward.vvResidualsZ.size()) ||
            vCursor >= static_cast<int>(forward.vvRingSizes.size())) {
            break;
        }
        PendingRemoval r;
        r.sym = s;
        r.forwardIndex = vCursor;
        if (si < forward.symbolInputQs.size()) r.inputQ = forward.symbolInputQs[si];
        r.rx = forward.vvResidualsX[static_cast<size_t>(vCursor)];
        r.ry = forward.vvResidualsY[static_cast<size_t>(vCursor)];
        r.rz = forward.vvResidualsZ[static_cast<size_t>(vCursor)];
        if (vCursor < static_cast<int>(forward.vvAbsQX.size()) &&
            vCursor < static_cast<int>(forward.vvAbsQY.size()) &&
            vCursor < static_cast<int>(forward.vvAbsQZ.size())) {
            r.ax = forward.vvAbsQX[static_cast<size_t>(vCursor)];
            r.ay = forward.vvAbsQY[static_cast<size_t>(vCursor)];
            r.az = forward.vvAbsQZ[static_cast<size_t>(vCursor)];
        }
        if (vCursor < static_cast<int>(forward.vvInputOffsets.size())) {
            r.inputOffset = forward.vvInputOffsets[static_cast<size_t>(vCursor)];
        }
        const int n = static_cast<int>(forward.vvRingSizes[static_cast<size_t>(vCursor)]);
        r.ringQ.reserve(static_cast<size_t>(n));
        for (int k = 0; k < n; ++k) {
            const size_t qi = ringCoord + static_cast<size_t>(k);
            if (qi < forward.vvRingQX.size() &&
                qi < forward.vvRingQY.size() &&
                qi < forward.vvRingQZ.size()) {
                r.ringQ.push_back({forward.vvRingQX[qi],
                                   forward.vvRingQY[qi],
                                   forward.vvRingQZ[qi]});
            }
        }
        ringCoord += static_cast<size_t>(n);
        InputKey key{r.inputQ};
        const int recIndex = static_cast<int>(pending.size());
        byInput[key].push_back(recIndex);
        const int rn = static_cast<int>(r.ringQ.size());
        if (rn > 0) {
            byRingSet[ringSetKeyFromQ(r.ringQ)].push_back(recIndex);
            for (int i = 0; i < rn; ++i) {
                byRingEdge[makeEdgeKey(r.ringQ[static_cast<size_t>(i)],
                                       r.ringQ[static_cast<size_t>((i + 1) % rn)])]
                    .push_back(recIndex);
                byRingVertex[r.ringQ[static_cast<size_t>(i)]].push_back(
                    RingAdjacencyNeed{
                        recIndex,
                        r.forwardIndex,
                        r.ringQ[static_cast<size_t>((i + rn - 1) % rn)],
                        r.ringQ[static_cast<size_t>((i + 1) % rn)]});
            }
        }
        pending.push_back(std::move(r));
        ++vCursor;
    }

    auto qToVertex = buildQVertexMap(mesh);
    std::array<int, 6> watchEdgeQ{};
    bool hasWatchEdge = false;
    if (const char* e = std::getenv("PMSF_RDSTRICT_BACKWARD_WATCH_EDGE")) {
        int a = 0, b = 0, c = 0, d = 0, ee = 0, f = 0;
        if (std::sscanf(e, "%d,%d,%d,%d,%d,%d", &a, &b, &c, &d, &ee, &f) == 6 ||
            std::sscanf(e, "%d:%d:%d:%d:%d:%d", &a, &b, &c, &d, &ee, &f) == 6) {
            watchEdgeQ = {a, b, c, d, ee, f};
            hasWatchEdge = true;
        }
    }
    auto watchedEdgeExists = [&]() {
        if (!hasWatchEdge) return false;
        MCGAL::Vertex* a = findQVertexWithTolerance(
            qToVertex, {watchEdgeQ[0], watchEdgeQ[1], watchEdgeQ[2]});
        MCGAL::Vertex* b = findQVertexWithTolerance(
            qToVertex, {watchEdgeQ[3], watchEdgeQ[4], watchEdgeQ[5]});
        if (!a || !b || a->isRemoved() || b->isRemoved()) return false;
        MCGAL::Halfedge* ab = findHalfedgeBetweenBounded(a, b);
        MCGAL::Halfedge* ba = findHalfedgeBetweenBounded(b, a);
        return (ab && !ab->isRemoved()) || (ba && !ba->isRemoved());
    };
    bool watchEdgeLive = watchedEdgeExists();
    auto currentInputKey = [&](MCGAL::Halfedge* gate, InputKey& key) -> bool {
        MCGAL::Halfedge* input = inputBoundaryFromGate(gate);
        if (!input || input->isRemoved() || !input->vertex() || !input->end_vertex()) {
            return false;
        }
        MCGAL::PointInt a = mesh->floatPosToInt(input->vertex()->point());
        MCGAL::PointInt b = mesh->floatPosToInt(input->end_vertex()->point());
        key.q = {a[0], a[1], a[2], b[0], b[1], b[2]};
        return true;
    };
    auto reversedKey = [](const InputKey& key) {
        InputKey r;
        r.q = {key.q[3], key.q[4], key.q[5], key.q[0], key.q[1], key.q[2]};
        return r;
    };
    auto ringForRecord = [&](const PendingRemoval& rec) {
        std::vector<MCGAL::Vertex*> ring;
        ring.reserve(rec.ringQ.size());
        for (const QKey& q : rec.ringQ) {
            MCGAL::Vertex* v = findQVertexWithTolerance(qToVertex, q);
            if (!v || v->isRemoved()) {
                ring.clear();
                return ring;
            }
            ring.push_back(v);
        }
        return ring;
    };
    auto patchRingMatchesRecord = [&](const PatchCandidate& patch,
                                      const PendingRemoval& rec,
                                      int valence) {
        std::vector<MCGAL::Vertex*> ring = orderedPatchBoundaryRing(patch);
        if (static_cast<int>(ring.size()) != valence ||
            static_cast<int>(rec.ringQ.size()) != valence) {
            return false;
        }
        if (envOn("PMSF_RDSTRICT_BACKWARD_LOOSE_RING")) {
            std::multiset<std::tuple<int, int, int>> actual;
            std::multiset<std::tuple<int, int, int>> expected;
            for (int i = 0; i < valence; ++i) {
                MCGAL::PointInt q = mesh->floatPosToInt(ring[static_cast<size_t>(i)]->point());
                actual.insert({q[0], q[1], q[2]});
                const QKey& e = rec.ringQ[static_cast<size_t>(i)];
                expected.insert({e.x, e.y, e.z});
            }
            return actual == expected;
        }
        std::vector<QKey> actual;
        actual.reserve(static_cast<size_t>(valence));
        for (int i = 0; i < valence; ++i) {
            MCGAL::PointInt q = mesh->floatPosToInt(ring[static_cast<size_t>(i)]->point());
            actual.push_back({q[0], q[1], q[2]});
        }
        auto qNear = [](const QKey& a, const QKey& b) {
            return std::abs(a.x - b.x) <= 1 &&
                   std::abs(a.y - b.y) <= 1 &&
                   std::abs(a.z - b.z) <= 1;
        };
        auto matchesWithStep = [&](int shift, int step) {
            for (int i = 0; i < valence; ++i) {
                int j = shift + step * i;
                j %= valence;
                if (j < 0) j += valence;
                if (!qNear(actual[static_cast<size_t>(i)],
                           rec.ringQ[static_cast<size_t>(j)])) {
                    return false;
                }
            }
            return true;
        };
        for (int shift = 0; shift < valence; ++shift) {
            if (matchesWithStep(shift, 1) || matchesWithStep(shift, -1)) {
                return true;
            }
        }
        return false;
    };
    auto printRingQ = [](const char* label, const std::vector<QKey>& ring) {
        std::fprintf(stderr, "%s", label);
        for (size_t i = 0; i < ring.size(); ++i) {
            const QKey& q = ring[i];
            std::fprintf(stderr, "%s%d,%d,%d", i ? ";" : "", q.x, q.y, q.z);
        }
        std::fprintf(stderr, "\n");
    };
    auto patchRingQ = [&](const PatchCandidate& patch) {
        std::vector<QKey> outQ;
        std::vector<MCGAL::Vertex*> ring = orderedPatchBoundaryRing(patch);
        outQ.reserve(ring.size());
        for (MCGAL::Vertex* v : ring) {
            if (!v || v->isRemoved()) {
                outQ.clear();
                return outQ;
            }
            MCGAL::PointInt q = mesh->floatPosToInt(v->point());
            outQ.push_back({q[0], q[1], q[2]});
        }
        return outQ;
    };
    auto printMismatchRingsAtGate = [&](PendingRemoval& rec,
                                        MCGAL::Facet* start,
                                        MCGAL::Halfedge* gateInput,
                                        MCGAL::Vertex* inputA,
                                        MCGAL::Vertex* inputB) {
        if (!envOn("PMSF_RDSTRICT_BACKWARD_TRACE_RING_MISMATCH")) return;
        const int valence = symbolValence(rec.sym);
        PatchCandidate patch;
        int mepp2Type = 0;
        bool found = false;
        if (gateInput && !gateInput->isRemoved()) {
            found = discoverMepp2PatchFromInput(gateInput, valence, patch, mepp2Type);
        }
        (void)start;
        (void)inputA;
        (void)inputB;
        if (!found) return;
        std::fprintf(stderr,
                     "    ring mismatch detail forward=%d valence=%d mepp2Type=%d\n",
                     rec.forwardIndex, valence, mepp2Type);
        printRingQ("    patchRing=", patchRingQ(patch));
        printRingQ("    recordRing=", rec.ringQ);
    };
    auto patchConsumesProtectedBoundary = [&](const PatchCandidate& patch,
                                              const PendingRemoval& rec) {
        for (MCGAL::Halfedge* h : patch.internalEdges) {
            if (!h || h->isRemoved() || !h->vertex() || !h->end_vertex()) continue;
            MCGAL::PointInt a = mesh->floatPosToInt(h->vertex()->point());
            MCGAL::PointInt b = mesh->floatPosToInt(h->end_vertex()->point());
            const EdgeKey edge = makeEdgeKey({a[0], a[1], a[2]}, {b[0], b[1], b[2]});
            auto it = byRingEdge.find(edge);
            if (it == byRingEdge.end()) continue;
            for (int idx : it->second) {
                if (idx < 0 || idx >= static_cast<int>(pending.size())) continue;
                const PendingRemoval& other = pending[static_cast<size_t>(idx)];
                if (other.used) continue;
                if (other.forwardIndex < rec.forwardIndex) return true;
            }
        }
        return false;
    };
    auto patchPreservesLowerBoundaryAdjacency = [&](const PatchCandidate& patch,
                                                    const PendingRemoval& rec) {
        if (!envOn("PMSF_RDSTRICT_BACKWARD_PROTECT_ADJ")) return true;
        std::vector<MCGAL::Vertex*> boundary = orderedPatchBoundaryRing(patch);
        std::set<MCGAL::Vertex*> boundarySet(boundary.begin(), boundary.end());
        const QKey insertedQ{rec.ax, rec.ay, rec.az};
        auto checkNeed = [&](const RingAdjacencyNeed& need) {
            if (need.ownerIdx < 0 || need.ownerIdx >= static_cast<int>(pending.size())) {
                return true;
            }
            const PendingRemoval& owner = pending[static_cast<size_t>(need.ownerIdx)];
            if (owner.used || need.ownerForwardIndex >= rec.forwardIndex) return true;
            const QKey adjacent[2] = {need.prev, need.next};
            for (const QKey& aq : adjacent) {
                MCGAL::Vertex* av = findQVertexWithTolerance(qToVertex, aq);
                if (av && !av->isRemoved() && !boundarySet.count(av)) {
                    return false;
                }
            }
            return true;
        };
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (std::abs(dx) + std::abs(dy) + std::abs(dz) > 1) continue;
                    auto it = byRingVertex.find(
                        {insertedQ.x + dx, insertedQ.y + dy, insertedQ.z + dz});
                    if (it == byRingVertex.end()) continue;
                    for (const RingAdjacencyNeed& need : it->second) {
                        if (!checkNeed(need)) return false;
                    }
                }
            }
        }
        return true;
    };
    auto findDecoderPatchAtGate = [&](int valence,
                                      MCGAL::Facet* start,
                                      MCGAL::Halfedge* gateInput,
                                      MCGAL::Vertex* inputA,
                                      MCGAL::Vertex* inputB,
                                      PatchCandidate& patch) {
        if (!start || start->isRemoved() || valence < 3) return false;
        const bool disableMepp2Join =
            envOn("PMSF_PCLTTM_DISABLE_MEPP2_JOIN") ||
            envOn("PMSF_RDSTRICT_BACKWARD_DISABLE_MEPP2_JOIN");
        if (!disableMepp2Join && gateInput && !gateInput->isRemoved()) {
            PatchCandidate mepp2Patch;
            int mepp2Type = 0;
            if (discoverMepp2PatchFromInput(gateInput, valence,
                                            mepp2Patch, mepp2Type)) {
                patch = std::move(mepp2Patch);
                return true;
            }
        }
        const bool strictDiag =
            !envOn("PMSF_RDSTRICT_BACKWARD_DISABLE_DIAG_CHECK");
        std::vector<MCGAL::Facet*> starts;
        starts.push_back(start);
        if (inputA && inputB) {
            MCGAL::Halfedge* ab = findHalfedgeBetweenBounded(inputA, inputB);
            MCGAL::Halfedge* ba = findHalfedgeBetweenBounded(inputB, inputA);
            MCGAL::Halfedge* hs[4] = {
                ab,
                ab ? ab->opposite() : nullptr,
                ba,
                ba ? ba->opposite() : nullptr,
            };
            for (MCGAL::Halfedge* h : hs) {
                MCGAL::Facet* f = h ? h->face() : nullptr;
                if (!f || f->isRemoved()) continue;
                bool exists = false;
                for (MCGAL::Facet* s : starts) {
                    if (s == f) {
                        exists = true;
                        break;
                    }
                }
                if (!exists) starts.push_back(f);
            }
        }
        for (MCGAL::Facet* s : starts) {
            PatchCandidate pc;
            if (!findPatchContainingFace(s, valence, pc, mesh,
                                         inputA, inputB, -1, strictDiag)) {
                continue;
            }
            patch = std::move(pc);
            return true;
        }
        return false;
    };
    auto tryRecordAtGate = [&](PendingRemoval& rec,
                               MCGAL::Facet* start,
                               MCGAL::Halfedge* gateInput,
                               MCGAL::Vertex* inputA,
                               MCGAL::Vertex* inputB,
                               MCGAL::Facet** outFace) {
        if (outFace) *outFace = nullptr;
        const int valence = symbolValence(rec.sym);
        const bool disableMepp2Join =
            envOn("PMSF_PCLTTM_DISABLE_MEPP2_JOIN") ||
            envOn("PMSF_RDSTRICT_BACKWARD_DISABLE_MEPP2_JOIN");
        if (!disableMepp2Join && gateInput && !gateInput->isRemoved()) {
            PatchCandidate mepp2Patch;
            int mepp2Type = 0;
            if (discoverMepp2PatchFromInput(gateInput, valence,
                                            mepp2Patch, mepp2Type) &&
                patchRingMatchesRecord(mepp2Patch, rec, valence) &&
                !patchConsumesProtectedBoundary(mepp2Patch, rec) &&
                patchPreservesLowerBoundaryAdjacency(mepp2Patch, rec)) {
                MCGAL::Facet* merged =
                    mergeMepp2PatchFromInput(gateInput, mepp2Type, valence);
                if (merged && !merged->isRemoved() && faceDegree(merged) == valence) {
                    if (outFace) *outFace = merged;
                    return true;
                }
                return false;
            }
        }
        PatchCandidate patch;
        if (!findDecoderPatchAtGate(valence, start, gateInput, inputA, inputB, patch)) return false;
        if (!patchRingMatchesRecord(patch, rec, valence)) return false;
        if (patchConsumesProtectedBoundary(patch, rec)) return false;
        if (!patchPreservesLowerBoundaryAdjacency(patch, rec)) return false;
        MCGAL::Facet* merged = mergePatchToPolygon(patch, valence);
        if (!merged || merged->isRemoved() || faceDegree(merged) != valence) return false;
        if (outFace) *outFace = merged;
        return true;
    };
    std::vector<MCGAL::Vertex*> matchedRing;
    auto rejectReasonAtGate = [&](PendingRemoval& rec,
                                  MCGAL::Facet* start,
                                  MCGAL::Halfedge* gateInput,
                                  MCGAL::Vertex* inputA,
                                  MCGAL::Vertex* inputB) -> const char* {
        const int valence = symbolValence(rec.sym);
        const bool disableMepp2Join =
            envOn("PMSF_PCLTTM_DISABLE_MEPP2_JOIN") ||
            envOn("PMSF_RDSTRICT_BACKWARD_DISABLE_MEPP2_JOIN");
        if (!disableMepp2Join && gateInput && !gateInput->isRemoved()) {
            PatchCandidate mepp2Patch;
            int mepp2Type = 0;
            if (discoverMepp2PatchFromInput(gateInput, valence,
                                            mepp2Patch, mepp2Type)) {
                if (!patchRingMatchesRecord(mepp2Patch, rec, valence)) return "mepp2_ring";
                if (patchConsumesProtectedBoundary(mepp2Patch, rec)) return "mepp2_protected";
                if (!patchPreservesLowerBoundaryAdjacency(mepp2Patch, rec)) return "mepp2_adj";
                return "mepp2_merge";
            }
        }
        PatchCandidate patch;
        if (!findDecoderPatchAtGate(valence, start, gateInput, inputA, inputB, patch)) {
            return "no_patch";
        }
        if (!patchRingMatchesRecord(patch, rec, valence)) return "ring";
        if (patchConsumesProtectedBoundary(patch, rec)) return "protected";
        if (!patchPreservesLowerBoundaryAdjacency(patch, rec)) return "adj";
        return "merge";
    };
    const bool backwardDecAcceptsV3 = envOn("PMSF_RDSTRICT_BACKWARD_DEC_V3");
    auto phaseAccepts = [backwardDecAcceptsV3](int phase, int valence) {
        const int decMin = backwardDecAcceptsV3 ? 3 : 4;
        return (phase == 0 && valence >= decMin && valence <= 6) ||
               (phase == 1 && valence == 3);
    };
    auto appendNull = [&](int phase,
                          int bucket,
                          unsigned short layer,
                          const std::array<int, 6>* inputQ = nullptr) {
        RDPatchSymbol s;
        s.kind = RDPatchSymbolKind::NULL_PATCH;
        s.escape_valence = 0;
        s.phase = static_cast<unsigned char>(phase);
        s.bucket = bucket;
        s.layer = layer;
        out.symbols.push_back(s);
        out.symbolInputQs.push_back(inputQ ? *inputQ : std::array<int, 6>{});
        ++out.numNullPatches;
    };
    auto appendRemoval = [&](const PendingRemoval& rec,
                             int phase,
                             int bucket,
                             unsigned short layer,
                             MCGAL::Facet* gateFace = nullptr,
                             const std::vector<MCGAL::Vertex*>* replayRing = nullptr,
                             MCGAL::Facet* seedFace = nullptr,
                             int seedRank = -1) {
        RDPatchSymbol s = rec.sym;
        s.phase = static_cast<unsigned char>(phase);
        s.bucket = bucket;
        s.layer = layer;
        out.symbols.push_back(s);
        out.vvResidualsX.push_back(rec.rx);
        out.vvResidualsY.push_back(rec.ry);
        out.vvResidualsZ.push_back(rec.rz);
        out.vvAbsQX.push_back(rec.ax);
        out.vvAbsQY.push_back(rec.ay);
        out.vvAbsQZ.push_back(rec.az);
        out.vvInputOffsets.push_back(rec.inputOffset);
        unsigned char ringN = 0;
        if (replayRing && !replayRing->empty()) {
            for (MCGAL::Vertex* rv : *replayRing) {
                if (!rv || rv->isRemoved()) continue;
                MCGAL::PointInt rq = mesh->floatPosToInt(rv->point());
                out.vvRingQX.push_back(rq[0]);
                out.vvRingQY.push_back(rq[1]);
                out.vvRingQZ.push_back(rq[2]);
                ++ringN;
            }
        } else {
            for (const QKey& q : rec.ringQ) {
                out.vvRingQX.push_back(q.x);
                out.vvRingQY.push_back(q.y);
                out.vvRingQZ.push_back(q.z);
                ++ringN;
            }
        }
        out.vvRingSizes.push_back(ringN);
        out.symbolInputQs.push_back(rec.inputQ);
        out.vvDecodeFaceRanks.push_back(rec.forwardIndex);
        {
            std::array<int, 9> fq{};
            unsigned char ringN = 0;
            if (gateFace && !gateFace->isRemoved()) {
                MCGAL::Halfedge* h = gateFace->proxyHalfedge();
                MCGAL::Halfedge* b = h;
                int k = 0;
                do {
                    if (!h || h->isRemoved()) break;
                    MCGAL::Vertex* v = h->vertex();
                    if (v && !v->isRemoved()) {
                        MCGAL::PointInt q = mesh->floatPosToInt(v->point());
                        if (k < 3) {
                            fq[static_cast<std::size_t>(k * 3 + 0)] = q[0];
                            fq[static_cast<std::size_t>(k * 3 + 1)] = q[1];
                            fq[static_cast<std::size_t>(k * 3 + 2)] = q[2];
                        }
                        out.vvFaceRingQX.push_back(q[0]);
                        out.vvFaceRingQY.push_back(q[1]);
                        out.vvFaceRingQZ.push_back(q[2]);
                        ++ringN;
                        ++k;
                    }
                    h = h ? h->next() : nullptr;
                } while (h && h != b && ringN < 255);
            }
            out.vvFaceQ.push_back(fq);
            out.vvFaceRingSizes.push_back(ringN);
        }
        {
            MCGAL::Halfedge* inputEdge = findHalfedgeByInputQ(mesh, rec.inputQ);
            unsigned char side = 0;
            if (inputEdge && gateFace) {
                side = (inputEdge->face() == gateFace) ? 0 : 1;
            }
            out.vvFaceSide.push_back(side);
            out.vvFaceRank.push_back(seedFace ? seedFace->poolId() : -1);
        }
        ++out.numDecimatedTotal;
    };

    const int startVertices = livingVertexCount(mesh);
    auto findRecordCompatibleInput = [&](const PendingRemoval& rec) -> MCGAL::Halfedge* {
        const int valence = symbolValence(rec.sym);
        std::vector<MCGAL::Vertex*> ring = ringForRecord(rec);
        if (static_cast<int>(ring.size()) != valence) return nullptr;
        for (int i = 0; i < valence; ++i) {
            MCGAL::Vertex* a = ring[static_cast<std::size_t>(i)];
            MCGAL::Vertex* b = ring[static_cast<std::size_t>((i + 1) % valence)];
            MCGAL::Halfedge* candidates[4] = {
                findHalfedgeBetweenBounded(a, b),
                findHalfedgeBetweenBounded(b, a),
                nullptr,
                nullptr,
            };
            candidates[2] = candidates[0] ? candidates[0]->opposite() : nullptr;
            candidates[3] = candidates[1] ? candidates[1]->opposite() : nullptr;
            for (MCGAL::Halfedge* input : candidates) {
                if (!input || input->isRemoved()) continue;
                PatchCandidate patch;
                int mepp2Type = 0;
                if (discoverMepp2PatchFromInput(input, valence, patch, mepp2Type) &&
                    patchRingMatchesRecord(patch, rec, valence)) {
                    return input;
                }
            }
        }
        return nullptr;
    };
    auto chooseBaseReachableSeedInput = [&]() -> MCGAL::Halfedge* {
        std::vector<int> order(pending.size());
        for (int i = 0; i < static_cast<int>(pending.size()); ++i) order[static_cast<size_t>(i)] = i;
        std::sort(order.begin(), order.end(), [&](int a, int b) {
            return pending[static_cast<size_t>(a)].forwardIndex >
                   pending[static_cast<size_t>(b)].forwardIndex;
        });
        for (int idx : order) {
            if (idx < 0 || idx >= static_cast<int>(pending.size())) continue;
            const PendingRemoval& rec = pending[static_cast<size_t>(idx)];
            const int valence = symbolValence(rec.sym);
            std::vector<MCGAL::Vertex*> ring = ringForRecord(rec);
            if (static_cast<int>(ring.size()) != valence) continue;
            if (MCGAL::Halfedge* compatible = findRecordCompatibleInput(rec)) {
                return compatible;
            }
            MCGAL::Halfedge* input = findHalfedgeByInputQ(mesh, rec.inputQ);
            MCGAL::Halfedge* active = input ? gateFromInputBoundary(input) : nullptr;
            if (active && active->face() && !active->face()->isRemoved()) return input;
        }
        return nullptr;
    };

    std::vector<int> forwardFirstGateQ = {
        forward.firstGateV0Q[0], forward.firstGateV0Q[1], forward.firstGateV0Q[2],
        forward.firstGateV1Q[0], forward.firstGateV1Q[1], forward.firstGateV1Q[2],
    };
    MCGAL::Halfedge* seedInput = envOn("PMSF_RDSTRICT_BACKWARD_FORWARD_SEED")
                                     ? nullptr
                                     : chooseBaseReachableSeedInput();
    const bool foundDecodeSeed = seedInput != nullptr;
    if (!seedInput) seedInput = findHalfedgeByInputQ(mesh, forwardFirstGateQ);
    MCGAL::Halfedge* firstGate = seedInput ? gateFromInputBoundary(seedInput) : nullptr;
    if (!firstGate) firstGate = seedInput;
    const bool foundForwardFirstGate = firstGate != nullptr;
    if (!firstGate) {
        firstGate = pickLongestEdge(mesh);
        firstGate = gateFromInputBoundary(firstGate);
    }
    if (!firstGate) return out;
    {
        MCGAL::Halfedge* seed = seedInput ? seedInput : firstGate;
        MCGAL::PointInt a = mesh->floatPosToInt(seed->vertex()->point());
        MCGAL::PointInt b = mesh->floatPosToInt(seed->end_vertex()->point());
        out.firstGateV0Q[0] = a[0];
        out.firstGateV0Q[1] = a[1];
        out.firstGateV0Q[2] = a[2];
        out.firstGateV1Q[0] = b[0];
        out.firstGateV1Q[1] = b[1];
        out.firstGateV1Q[2] = b[2];
        out.layerSeedQs.push_back({a[0], a[1], a[2], b[0], b[1], b[2]});
    }
    markGateInputTags(firstGate);
    if (envOn("PMSF_RDSTRICT_BACKWARD_DEBUG")) {
        std::fprintf(stderr,
                     "[RD_STRICT backward debug] firstGateFromForward=%d decodeSeed=%d\n",
                     foundForwardFirstGate ? 1 : 0,
                     foundDecodeSeed ? 1 : 0);
    }

    RDPatchGateQueue queue;
    std::set<MCGAL::Halfedge*> enqueued;
    std::set<MCGAL::Facet*> facesEnqueued;
    std::set<MCGAL::Facet*> facesProcessedInPhase;
    std::set<MCGAL::Facet*> nullPatchFaces;
    const bool singleSeed = envOn("PMSF_PCLTTM_SINGLE_SEED");
    const bool rebuildForceSixBucket =
        envOn("PMSF_RDSTRICT_BACKWARD_FORCE_6BUCKET") ||
        envOn("PMSF_RDPATCH_USE_6BUCKET");
    auto seedQueue = [&]() {
        queue.clear();
        enqueued.clear();
        facesEnqueued.clear();
        if (!singleSeed) {
            seedAllLivingFaces(mesh, queue, enqueued, facesEnqueued,
                               nullPatchFaces, rebuildForceSixBucket);
            return;
        }
        if (!firstGate || firstGate->isRemoved() ||
            !firstGate->face() || firstGate->face()->isRemoved()) {
            firstGate = pickLongestEdge(mesh);
            firstGate = gateFromInputBoundary(firstGate);
        }
        if (firstGate && firstGate->face() && !firstGate->face()->isRemoved()) {
            queue.push(firstGate,
                       computeRealBucket(firstGate, nullPatchFaces,
                                         rebuildForceSixBucket));
            enqueued.insert(firstGate);
            facesEnqueued.insert(firstGate->face());
        }
    };
    seedQueue();

    int phase = 0;
    bool progressInCurrentCycle = false;
    int cyclesCompleted = 0;
    int step = 0;
    int directMatches = 0;
    int ringMatches = 0;
    int noCandidatePops = 0;
    int candidateMissPops = 0;
    bool printedBackwardDebug = false;
    const int maxSteps = std::max<int>(
        {1000,
         livingFaceCount(mesh) * envInt("PMSF_RDSTRICT_BACKWARD_FACE_STEPS", 4),
         static_cast<int>(forward.symbols.size()) * envInt("PMSF_RDSTRICT_BACKWARD_SYM_STEPS", 4) +
             static_cast<int>(pending.size()) * envInt("PMSF_RDSTRICT_BACKWARD_V_STEPS", 8)});

    while (step < maxSteps && out.numDecimatedTotal < static_cast<int>(pending.size())) {
        if (queue.empty()) {
            if (phase == 0) {
                phase = 1;
            } else {
                ++cyclesCompleted;
                out.numDecCycles = cyclesCompleted;
                if (!progressInCurrentCycle ||
                    cyclesCompleted >= envInt("PMSF_RDSTRICT_BACKWARD_MAX_CYCLES", 1)) break;
                phase = 0;
                progressInCurrentCycle = false;
            }
            enqueued.clear();
            facesEnqueued.clear();
            facesProcessedInPhase.clear();
            nullPatchFaces.clear();
            resetLivingFaceProcessedFlags(mesh);
            seedQueue();
            continue;
        }

        int popBucket = -1;
        MCGAL::Halfedge* h = queue.popHighestPriority(&popBucket);
        if (!h || h->isRemoved()) continue;
        MCGAL::Facet* gateFace = h->face();
        if (gateFace && facesProcessedInPhase.count(gateFace)) continue;
        ++step;
        out.numSteps = step;
        if (popBucket >= 1 && popBucket <= 6) ++out.bucketHits[popBucket - 1];

        MCGAL::Vertex* inputA = nullptr;
        MCGAL::Vertex* inputB = nullptr;
        MCGAL::Halfedge* input = inputBoundaryFromGate(h);
        if (input) {
            inputA = input->vertex();
            inputB = input->end_vertex();
        }
        PendingRemoval* matched = nullptr;
        MCGAL::Facet* patchFace = nullptr;
        MCGAL::Facet* patchSeedFace = nullptr;
        int patchSeedRank = -1;
        auto ringSeedFace = [&](const std::vector<MCGAL::Vertex*>& ring) -> MCGAL::Facet* {
            const int n = static_cast<int>(ring.size());
            for (int i = 0; i < n; ++i) {
                MCGAL::Halfedge* h = findHalfedgeBetweenBounded(ring[static_cast<size_t>(i)],
                                                                ring[static_cast<size_t>((i + 1) % n)]);
                if (h && h->face() && !h->face()->isRemoved()) return h->face();
            }
            return nullptr;
        };
        auto appendCandidate = [](std::vector<int>& dst, int idx) {
            if (std::find(dst.begin(), dst.end(), idx) == dst.end()) dst.push_back(idx);
        };
        std::vector<int> candidates;
        InputKey key;
        bool haveInputKey = false;
        if (currentInputKey(h, key)) {
            haveInputKey = true;
            const InputKey rev = reversedKey(key);
            if (auto it = byInput.find(key); it != byInput.end()) {
                for (int idx : it->second) appendCandidate(candidates, idx);
            }
            if (!(rev == key)) {
                if (auto it = byInput.find(rev); it != byInput.end()) {
                    for (int idx : it->second) appendCandidate(candidates, idx);
                }
            }
            EdgeKey ek = makeEdgeKey({key.q[0], key.q[1], key.q[2]},
                                     {key.q[3], key.q[4], key.q[5]});
            if (auto it = byRingEdge.find(ek); it != byRingEdge.end()) {
                for (int idx : it->second) appendCandidate(candidates, idx);
            }
        }
        if (envOn("PMSF_RDSTRICT_BACKWARD_RINGSET_CANDIDATES")) {
            for (int valence = 3; valence <= 6; ++valence) {
                if (!phaseAccepts(phase, valence)) continue;
                PatchCandidate patch;
                if (!findDecoderPatchAtGate(valence, gateFace, input,
                                            inputA, inputB, patch)) {
                    continue;
                }
                const std::string ringKey =
                    ringSetKeyFromVertices(orderedPatchBoundaryRing(patch));
                if (ringKey.empty()) continue;
                if (auto it = byRingSet.find(ringKey); it != byRingSet.end()) {
                    for (int idx : it->second) appendCandidate(candidates, idx);
                }
            }
        }
        for (int idx : candidates) {
            PendingRemoval& rec = pending[static_cast<size_t>(idx)];
            if (rec.used) continue;
            const int valence = symbolValence(rec.sym);
            if (!phaseAccepts(phase, valence)) continue;
            MCGAL::Facet* merged = nullptr;
            if (!tryRecordAtGate(rec, gateFace, input, inputA, inputB, &merged)) continue;
            matched = &rec;
            patchFace = merged;
            patchSeedFace = gateFace;
            patchSeedRank = -1;
            ++directMatches;
            break;
        }
        if (!matched && !printedBackwardDebug &&
            envOn("PMSF_RDSTRICT_BACKWARD_DEBUG") && !candidates.empty()) {
            printedBackwardDebug = true;
            std::fprintf(stderr,
                         "[RD_STRICT backward debug] step=%d phase=%d bucket=%d candidates=%zu input=(%d,%d,%d)-(%d,%d,%d)\n",
                         step, phase, popBucket, candidates.size(),
                         key.q[0], key.q[1], key.q[2], key.q[3], key.q[4], key.q[5]);
            int shown = 0;
            for (int idx : candidates) {
                if (idx < 0 || idx >= static_cast<int>(pending.size())) continue;
                PendingRemoval& rec = pending[static_cast<size_t>(idx)];
                if (rec.used) continue;
                const int valence = symbolValence(rec.sym);
                MCGAL::Facet* dbgFace = nullptr;
                const bool foundLocal =
                    tryRecordAtGate(rec, gateFace, input, inputA, inputB, &dbgFace);
                std::fprintf(stderr,
                             "  cand idx=%d valence=%d tryRecord=%d reason=%s "
                             "faceDegree=%d ringN=%zu\n",
                             idx, valence, foundLocal ? 1 : 0,
                             foundLocal ? "ok" :
                                 rejectReasonAtGate(rec, gateFace, input, inputA, inputB),
                             faceDegree(dbgFace),
                             rec.ringQ.size());
                if (!foundLocal) {
                    printMismatchRingsAtGate(rec, gateFace, input, inputA, inputB);
                }
                if (++shown >= 8) break;
            }
        }
        if (!matched) {
            if (candidates.empty()) {
                ++noCandidatePops;
            } else {
                ++candidateMissPops;
            }
        }
        if (!matched && envOn("PMSF_RDSTRICT_BACKWARD_GLOBAL_RING")) {
            std::vector<int> ringCandidates = candidates;
            std::sort(ringCandidates.begin(), ringCandidates.end(),
                      [&](int a, int b) {
                          if (a < 0 || a >= static_cast<int>(pending.size())) return false;
                          if (b < 0 || b >= static_cast<int>(pending.size())) return true;
                          return pending[static_cast<size_t>(a)].forwardIndex >
                                 pending[static_cast<size_t>(b)].forwardIndex;
                      });
            for (int idx : ringCandidates) {
                PendingRemoval& rec = pending[static_cast<size_t>(idx)];
                if (rec.used) continue;
                const int valence = symbolValence(rec.sym);
                if (!phaseAccepts(phase, valence)) continue;
                std::vector<MCGAL::Vertex*> ring = ringForRecord(rec);
                if (static_cast<int>(ring.size()) != valence) continue;
                MCGAL::Facet* seed = ringSeedFace(ring);
                const int seedRank = seed ? livingFaceRank(mesh, seed) : -1;
                MCGAL::Facet* merged =
                    mergePatchByRing(mesh, ring, valence, false,
                                     [&](const PatchCandidate& patch) {
                                         return !patchConsumesProtectedBoundary(patch, rec) &&
                                                patchPreservesLowerBoundaryAdjacency(patch, rec);
                                     });
                if (!merged || merged->isRemoved() || faceDegree(merged) != valence) continue;
                matched = &rec;
                matchedRing = ring;
                patchSeedFace = seed;
                patchSeedRank = seedRank;
                patchFace = merged;
                ++ringMatches;
                break;
            }
        }
        const int smallGlobalThreshold =
            envIntAllowZero("PMSF_RDSTRICT_BACKWARD_SMALL_GLOBAL_RING_THRESHOLD", 1024);
        const int remainingCount =
            static_cast<int>(pending.size()) - out.numDecimatedTotal;
        if (!matched && remainingCount <= smallGlobalThreshold) {
            std::vector<int> remainingIdx;
            remainingIdx.reserve(pending.size());
            for (int i = 0; i < static_cast<int>(pending.size()); ++i) {
                if (!pending[static_cast<size_t>(i)].used) remainingIdx.push_back(i);
            }
            std::sort(remainingIdx.begin(), remainingIdx.end(),
                      [&](int a, int b) {
                          return pending[static_cast<size_t>(a)].forwardIndex >
                                 pending[static_cast<size_t>(b)].forwardIndex;
                      });
            for (int idx : remainingIdx) {
                PendingRemoval& rec = pending[static_cast<size_t>(idx)];
                if (rec.used) continue;
                const int valence = symbolValence(rec.sym);
                if (!phaseAccepts(phase, valence)) continue;
                std::vector<MCGAL::Vertex*> ring = ringForRecord(rec);
                if (static_cast<int>(ring.size()) != valence) continue;
                const bool allowSubsetSearch =
                    remainingCount <= envInt("PMSF_RDSTRICT_BACKWARD_RING_SUBSET_TAIL", 8);
                MCGAL::Facet* seed = ringSeedFace(ring);
                const int seedRank = seed ? livingFaceRank(mesh, seed) : -1;
                MCGAL::Facet* merged =
                    mergePatchByRing(mesh, ring, valence, allowSubsetSearch,
                                     [&](const PatchCandidate& patch) {
                                         return !patchConsumesProtectedBoundary(patch, rec) &&
                                                patchPreservesLowerBoundaryAdjacency(patch, rec);
                                     });
                if (!merged || merged->isRemoved() || faceDegree(merged) != valence) continue;
                matched = &rec;
                matchedRing = ring;
                patchSeedFace = seed;
                patchSeedRank = seedRank;
                patchFace = merged;
                ++ringMatches;
                break;
            }
        }
        if (!matched && envOn("PMSF_RDSTRICT_BACKWARD_FULL_SCAN")) {
            for (PendingRemoval& rec : pending) {
                if (rec.used) continue;
                const int valence = symbolValence(rec.sym);
                if (!phaseAccepts(phase, valence)) continue;
                MCGAL::Facet* merged = nullptr;
                if (!tryRecordAtGate(rec, gateFace, input, inputA, inputB, &merged)) continue;
                matched = &rec;
                matchedRing.clear();
                patchSeedFace = gateFace;
                patchSeedRank = -1;
                patchFace = merged;
                break;
            }
        }

        if (!matched || !patchFace) {
            appendNull(phase,
                       popBucket,
                       static_cast<unsigned short>(cyclesCompleted),
                       haveInputKey ? &key.q : nullptr);
            if (gateFace) {
                facesProcessedInPhase.insert(gateFace);
                gateFace->setProcessedFlag();
                nullPatchFaces.insert(gateFace);
            }
            applyStrictNullGateTag(h);
            pushPaperNullPatchNeighbors(h, queue, enqueued, facesEnqueued,
                                        nullPatchFaces, rebuildForceSixBucket);
            continue;
        }

        MCGAL::Halfedge* paperInputOnPatch =
            faceHalfedgeBetweenOriented(patchFace, inputA, inputB);
        if (paperInputOnPatch) {
            patchFace->proxyHalfedge(paperInputOnPatch);
        } else {
            patchFace->normalize_proxy_to_min_position_vertex(mesh->bboxMin, mesh->f_quantStep);
        }
        RingSnapshot polygonSnap = snapshotFaceRing(patchFace);
        std::vector<MCGAL::Halfedge*> paperDecimatingGates;
        if (phase == 0) {
            paperDecimatingGates =
                collectPaperDecimatingGatesFromPolygon(patchFace, inputA, inputB);
        }
        if (!applyMepp2BoundaryTagsFromInput(paperInputOnPatch, symbolValence(matched->sym))) {
            applyFig9TagsToRing(polygonSnap.ringVerts);
        }
        PendingRemoval emitted = *matched;
        if (!envOn("PMSF_RDSTRICT_BACKWARD_PRESERVE_RESIDUALS")) {
            MCGAL::PointInt emitBaryQ = MCGAL::rdBarycenterQuantized(patchFace, mesh);
            emitted.rx = emitted.ax - emitBaryQ[0];
            emitted.ry = emitted.ay - emitBaryQ[1];
            emitted.rz = emitted.az - emitBaryQ[2];
        }
        QKey absQ{matched->ax, matched->ay, matched->az};
        MCGAL::Vertex* inserted = nullptr;
        const bool ok = insertDecodedVertexOnFace(mesh, patchFace,
                                                  out.vvResidualsX,
                                                  out.vvResidualsY,
                                                  out.vvResidualsZ,
                                                  -1,
                                                  &absQ,
                                                  &inserted);
        if (!ok || !inserted) {
            appendNull(phase,
                       popBucket,
                       static_cast<unsigned short>(cyclesCompleted),
                       haveInputKey ? &key.q : nullptr);
            if (gateFace) facesProcessedInPhase.insert(gateFace);
            continue;
        }
        matched->used = true;
        qToVertex[absQ] = inserted;
        inserted->setConquered();
        appendRemoval(emitted, phase, popBucket, static_cast<unsigned short>(cyclesCompleted), patchFace, nullptr, patchSeedFace, patchSeedRank);
        if (hasWatchEdge) {
            const bool insertedA =
                emitted.ax == watchEdgeQ[0] &&
                emitted.ay == watchEdgeQ[1] &&
                emitted.az == watchEdgeQ[2];
            const bool insertedB =
                emitted.ax == watchEdgeQ[3] &&
                emitted.ay == watchEdgeQ[4] &&
                emitted.az == watchEdgeQ[5];
            if (insertedA || insertedB) {
                std::fprintf(stderr,
                             "[RD_STRICT watch vertex] emittedForward=%d valence=%d "
                             "inserted endpoint=%c\n",
                             emitted.forwardIndex,
                             symbolValence(emitted.sym),
                             insertedA ? 'A' : 'B');
            }
            const bool nowLive = watchedEdgeExists();
            if (watchEdgeLive != nowLive) {
                std::fprintf(stderr,
                             "[RD_STRICT watch edge] emittedForward=%d valence=%d "
                             "edge %s after insertedQ=(%d,%d,%d)\n",
                             emitted.forwardIndex,
                             symbolValence(emitted.sym),
                             nowLive ? "appeared" : "disappeared",
                             emitted.ax, emitted.ay, emitted.az);
            }
            watchEdgeLive = nowLive;
        }
        progressInCurrentCycle = true;
        if (gateFace && !gateFace->isRemoved()) {
            gateFace->setProcessedFlag();
            facesProcessedInPhase.insert(gateFace);
        }
        markPatchInteriorFacesProcessedFromRing(polygonSnap.ringVerts,
                                                inserted,
                                                facesProcessedInPhase);
        if (phase == 0) {
            for (MCGAL::Halfedge* gate : paperDecimatingGates) {
                pushGateIfUsable(gate, queue, enqueued, facesEnqueued,
                                 nullPatchFaces, rebuildForceSixBucket);
            }
        } else {
            pushPaperCleaningGates(polygonSnap.ringVerts, mesh, queue, enqueued,
                                   facesEnqueued, facesProcessedInPhase,
                                   nullPatchFaces, rebuildForceSixBucket);
        }
    }

    out.numVerticesOut = startVertices;
    out.numVerticesIn = startVertices + out.numDecimatedTotal;
    if (out.numDecimatedTotal != static_cast<int>(pending.size())) {
        int rem3 = 0, rem4 = 0, rem5 = 0, rem6 = 0, remOther = 0;
        int minForward = std::numeric_limits<int>::max();
        int maxForward = -1;
        for (const PendingRemoval& rec : pending) {
            if (rec.used) continue;
            minForward = std::min(minForward, rec.forwardIndex);
            maxForward = std::max(maxForward, rec.forwardIndex);
            switch (symbolValence(rec.sym)) {
                case 3: ++rem3; break;
                case 4: ++rem4; break;
                case 5: ++rem5; break;
                case 6: ++rem6; break;
                default: ++remOther; break;
            }
        }
        const int remTotal = rem3 + rem4 + rem5 + rem6 + remOther;
        if (envOn("PMSF_RDSTRICT_BACKWARD_REMAIN_DIAG") &&
            remTotal <= envInt("PMSF_RDSTRICT_BACKWARD_REMAIN_DIAG_LIMIT", 8)) {
            for (const PendingRemoval& rec : pending) {
                if (rec.used) continue;
                const int valence = symbolValence(rec.sym);
                std::vector<MCGAL::Vertex*> ring = ringForRecord(rec);
                std::set<MCGAL::Vertex*> ringSet(ring.begin(), ring.end());
                int polygonCount = 0;
                int triInsideCount = 0;
                std::vector<MCGAL::Facet*> triInside;
                for (MCGAL::Facet* f : mesh->faces()) {
                    if (!f || f->isRemoved()) continue;
                    const int deg = faceDegree(f);
                    if (deg == valence && faceVerticesWithin(f, ringSet)) ++polygonCount;
                    if (deg == 3 && faceVerticesWithin(f, ringSet)) {
                        ++triInsideCount;
                        if (triInside.size() < 32) triInside.push_back(f);
                    }
                }
                bool subsetPossible = false;
                if (static_cast<int>(triInside.size()) >= valence - 2 &&
                    triInside.size() <= 24) {
                    std::vector<MCGAL::Facet*> chosen;
                    chosen.reserve(static_cast<size_t>(valence - 2));
                    std::function<bool(int)> dfs = [&](int nextIdx) -> bool {
                        if (static_cast<int>(chosen.size()) == valence - 2) {
                            PatchCandidate cand;
                            return validateTriangulatedPatch(chosen, valence, cand);
                        }
                        for (int i = nextIdx; i < static_cast<int>(triInside.size()); ++i) {
                            chosen.push_back(triInside[static_cast<size_t>(i)]);
                            if (dfs(i + 1)) return true;
                            chosen.pop_back();
                        }
                        return false;
                    };
                    subsetPossible = dfs(0);
                }
                MCGAL::Vertex* ia = findQVertexWithTolerance(
                    qToVertex, {rec.inputQ[0], rec.inputQ[1], rec.inputQ[2]});
                MCGAL::Vertex* ib = findQVertexWithTolerance(
                    qToVertex, {rec.inputQ[3], rec.inputQ[4], rec.inputQ[5]});
                MCGAL::Halfedge* input = findHalfedgeBetweenBounded(ia, ib);
                PatchCandidate mepp2Patch;
                int mepp2Type = 0;
                const bool mepp2Ok =
                    input && discoverMepp2PatchFromInput(input, valence,
                                                         mepp2Patch, mepp2Type);
                std::fprintf(stderr,
                             "[RD_STRICT backward remain] idx=%d valence=%d "
                             "ring=%zu/%zu polygon=%d triInside=%d subset=%d "
                             "input=%d mepp2=%d type=%d q0=(%d,%d,%d) q1=(%d,%d,%d)\n",
                             rec.forwardIndex,
                             valence,
                             ring.size(), rec.ringQ.size(),
                             polygonCount,
                             triInsideCount,
                             subsetPossible ? 1 : 0,
                             input ? 1 : 0,
                             mepp2Ok ? 1 : 0,
                             mepp2Type,
                             rec.inputQ[0], rec.inputQ[1], rec.inputQ[2],
                             rec.inputQ[3], rec.inputQ[4], rec.inputQ[5]);
            }
        }
        std::fprintf(stderr,
                     "[RD_STRICT backward emit] WARNING emitted %d/%zu removals "
                     "in decode order; direct=%d ring=%d noCand=%d candMiss=%d "
                     "remaining v3=%d v4=%d v5=%d v6=%d other=%d forwardRange=%d..%d steps=%d/%d\n",
                     out.numDecimatedTotal, pending.size(),
                     directMatches, ringMatches, noCandidatePops, candidateMissPops,
                     rem3, rem4, rem5, rem6, remOther,
                     minForward == std::numeric_limits<int>::max() ? -1 : minForward,
                     maxForward, step, maxSteps);
    }
    return out;
}

namespace {

RDPatchPipelineResult filterRemainingStrictForwardRecords(
    const RDPatchPipelineResult& src,
    const std::vector<int>& usedForwardIndices) {
    std::vector<char> used(src.vvResidualsX.size(), 0);
    for (int idx : usedForwardIndices) {
        if (idx >= 0 && idx < static_cast<int>(used.size())) {
            used[static_cast<std::size_t>(idx)] = 1;
        }
    }

    RDPatchPipelineResult out;
    out.mesh = src.mesh;
    out.numVerticesIn = src.numVerticesIn;
    out.numVerticesOut = src.numVerticesOut;
    out.firstGateV0Q[0] = src.firstGateV0Q[0];
    out.firstGateV0Q[1] = src.firstGateV0Q[1];
    out.firstGateV0Q[2] = src.firstGateV0Q[2];
    out.firstGateV1Q[0] = src.firstGateV1Q[0];
    out.firstGateV1Q[1] = src.firstGateV1Q[1];
    out.firstGateV1Q[2] = src.firstGateV1Q[2];
    out.layerSeedQs = src.layerSeedQs;
    out.leakyFactors = src.leakyFactors;

    std::size_t ringCursor = 0;
    int vCursor = 0;
    const bool symbolInputsBySymbol = src.symbolInputQs.size() == src.symbols.size();
    for (std::size_t si = 0; si < src.symbols.size(); ++si) {
        const RDPatchSymbol& sym = src.symbols[si];
        if (sym.kind == RDPatchSymbolKind::NULL_PATCH) continue;
        const int cur = vCursor++;
        const int ringN = (cur >= 0 && cur < static_cast<int>(src.vvRingSizes.size()))
                              ? static_cast<int>(src.vvRingSizes[static_cast<std::size_t>(cur)])
                              : 0;
        if (cur >= 0 && cur < static_cast<int>(used.size()) &&
            !used[static_cast<std::size_t>(cur)]) {
            out.symbols.push_back(sym);
            if (symbolInputsBySymbol && si < src.symbolInputQs.size()) {
                out.symbolInputQs.push_back(src.symbolInputQs[si]);
            } else if (cur < static_cast<int>(src.symbolInputQs.size())) {
                out.symbolInputQs.push_back(src.symbolInputQs[static_cast<std::size_t>(cur)]);
            }
            out.vvResidualsX.push_back(src.vvResidualsX[static_cast<std::size_t>(cur)]);
            out.vvResidualsY.push_back(src.vvResidualsY[static_cast<std::size_t>(cur)]);
            out.vvResidualsZ.push_back(src.vvResidualsZ[static_cast<std::size_t>(cur)]);
            if (cur < static_cast<int>(src.vvAbsQX.size()) &&
                cur < static_cast<int>(src.vvAbsQY.size()) &&
                cur < static_cast<int>(src.vvAbsQZ.size())) {
                out.vvAbsQX.push_back(src.vvAbsQX[static_cast<std::size_t>(cur)]);
                out.vvAbsQY.push_back(src.vvAbsQY[static_cast<std::size_t>(cur)]);
                out.vvAbsQZ.push_back(src.vvAbsQZ[static_cast<std::size_t>(cur)]);
            }
            if (cur < static_cast<int>(src.vvInputOffsets.size())) {
                out.vvInputOffsets.push_back(src.vvInputOffsets[static_cast<std::size_t>(cur)]);
            }
            out.vvRingSizes.push_back(static_cast<unsigned char>(ringN));
            for (int k = 0; k < ringN; ++k) {
                const std::size_t qi = ringCursor + static_cast<std::size_t>(k);
                if (qi < src.vvRingQX.size() && qi < src.vvRingQY.size() &&
                    qi < src.vvRingQZ.size()) {
                    out.vvRingQX.push_back(src.vvRingQX[qi]);
                    out.vvRingQY.push_back(src.vvRingQY[qi]);
                    out.vvRingQZ.push_back(src.vvRingQZ[qi]);
                }
            }
            ++out.numDecimatedTotal;
        }
        ringCursor += static_cast<std::size_t>(std::max(ringN, 0));
    }
    if (!out.symbolInputQs.empty()) {
        const auto& q = out.symbolInputQs.front();
        out.firstGateV0Q[0] = q[0];
        out.firstGateV0Q[1] = q[1];
        out.firstGateV0Q[2] = q[2];
        out.firstGateV1Q[0] = q[3];
        out.firstGateV1Q[1] = q[4];
        out.firstGateV1Q[2] = q[5];
        out.layerSeedQs.clear();
        out.layerSeedQs.push_back(q);
    }
    return out;
}

RDPatchPipelineResult extractStrictForwardLayerRecords(
    const RDPatchPipelineResult& src,
    unsigned short wantedLayer) {
    RDPatchPipelineResult out;
    out.mesh = src.mesh;
    out.numVerticesIn = src.numVerticesIn;
    out.numVerticesOut = src.numVerticesOut;
    out.leakyFactors = src.leakyFactors;
    if (wantedLayer < src.layerSeedQs.size()) {
        const auto& q = src.layerSeedQs[static_cast<std::size_t>(wantedLayer)];
        out.layerSeedQs.push_back(q);
        out.firstGateV0Q[0] = q[0];
        out.firstGateV0Q[1] = q[1];
        out.firstGateV0Q[2] = q[2];
        out.firstGateV1Q[0] = q[3];
        out.firstGateV1Q[1] = q[4];
        out.firstGateV1Q[2] = q[5];
    } else {
        out.firstGateV0Q[0] = src.firstGateV0Q[0];
        out.firstGateV0Q[1] = src.firstGateV0Q[1];
        out.firstGateV0Q[2] = src.firstGateV0Q[2];
        out.firstGateV1Q[0] = src.firstGateV1Q[0];
        out.firstGateV1Q[1] = src.firstGateV1Q[1];
        out.firstGateV1Q[2] = src.firstGateV1Q[2];
        out.layerSeedQs = src.layerSeedQs;
    }

    std::size_t ringCursor = 0;
    int vCursor = 0;
    const bool symbolInputsBySymbol = src.symbolInputQs.size() == src.symbols.size();
    for (std::size_t si = 0; si < src.symbols.size(); ++si) {
        const RDPatchSymbol& sym = src.symbols[si];
        const bool include = sym.layer == wantedLayer;
        if (include) {
            out.symbols.push_back(sym);
            if (symbolInputsBySymbol && si < src.symbolInputQs.size()) {
                out.symbolInputQs.push_back(src.symbolInputQs[si]);
            } else {
                out.symbolInputQs.push_back(std::array<int, 6>{});
            }
            if (sym.kind == RDPatchSymbolKind::NULL_PATCH) {
                ++out.numNullPatches;
            }
        }
        if (sym.kind == RDPatchSymbolKind::NULL_PATCH) continue;

        const int cur = vCursor++;
        const int ringN = (cur >= 0 && cur < static_cast<int>(src.vvRingSizes.size()))
                              ? static_cast<int>(src.vvRingSizes[static_cast<std::size_t>(cur)])
                              : 0;
        if (include) {
            if (cur < static_cast<int>(src.vvResidualsX.size()) &&
                cur < static_cast<int>(src.vvResidualsY.size()) &&
                cur < static_cast<int>(src.vvResidualsZ.size())) {
                out.vvResidualsX.push_back(src.vvResidualsX[static_cast<std::size_t>(cur)]);
                out.vvResidualsY.push_back(src.vvResidualsY[static_cast<std::size_t>(cur)]);
                out.vvResidualsZ.push_back(src.vvResidualsZ[static_cast<std::size_t>(cur)]);
            }
            if (cur < static_cast<int>(src.vvAbsQX.size()) &&
                cur < static_cast<int>(src.vvAbsQY.size()) &&
                cur < static_cast<int>(src.vvAbsQZ.size())) {
                out.vvAbsQX.push_back(src.vvAbsQX[static_cast<std::size_t>(cur)]);
                out.vvAbsQY.push_back(src.vvAbsQY[static_cast<std::size_t>(cur)]);
                out.vvAbsQZ.push_back(src.vvAbsQZ[static_cast<std::size_t>(cur)]);
            }
            if (cur < static_cast<int>(src.vvInputOffsets.size())) {
                out.vvInputOffsets.push_back(src.vvInputOffsets[static_cast<std::size_t>(cur)]);
            }
            out.vvRingSizes.push_back(static_cast<unsigned char>(ringN));
            for (int k = 0; k < ringN; ++k) {
                const std::size_t qi = ringCursor + static_cast<std::size_t>(k);
                if (qi < src.vvRingQX.size() &&
                    qi < src.vvRingQY.size() &&
                    qi < src.vvRingQZ.size()) {
                    out.vvRingQX.push_back(src.vvRingQX[qi]);
                    out.vvRingQY.push_back(src.vvRingQY[qi]);
                    out.vvRingQZ.push_back(src.vvRingQZ[qi]);
                }
            }
            ++out.numDecimatedTotal;
        }
        ringCursor += static_cast<std::size_t>(std::max(ringN, 0));
    }
    return out;
}

}  // namespace

RDPatchPipelineResult rebuildRDPatchResultInSplitStrictDecodeOrder(
    std::shared_ptr<MCGAL::Mesh> mesh,
    const RDPatchPipelineResult& forward) {
    RDPatchPipelineResult combined;
    combined.mesh = mesh;
    combined.numVerticesIn = forward.numVerticesIn;
    combined.numVerticesOut = forward.numVerticesOut;
    combined.leakyFactors = forward.leakyFactors;
    if (!mesh || forward.numDecimatedTotal <= 0) return combined;

    std::vector<RDPatchPipelineResult> rounds;
    rounds.reserve(16);
    if (envOn("PMSF_PCLTTM_LAYERWISE_REBUILD")) {
        int maxLayer = -1;
        for (const RDPatchSymbol& s : forward.symbols) {
            maxLayer = std::max(maxLayer, static_cast<int>(s.layer));
        }
        int total = 0;
        bool ok = maxLayer >= 0;
        for (int li = maxLayer; li >= 0 && ok; --li) {
            RDPatchPipelineResult layerInput =
                extractStrictForwardLayerRecords(forward, static_cast<unsigned short>(li));
            if (layerInput.numDecimatedTotal <= 0) continue;
            RDPatchPipelineResult layerRemaining = layerInput;
            const int maxSubRounds = envInt("PMSF_PCLTTM_SPLIT_REBUILD_MAX_SUBROUNDS", 32);
            int layerTotal = 0;
            std::vector<RDPatchPipelineResult> layerRounds;
            layerRounds.reserve(4);
            for (int sub = 0; sub < maxSubRounds &&
                 layerRemaining.numDecimatedTotal > 0; ++sub) {
                RDPatchPipelineResult round =
                    rebuildRDPatchResultInStrictDecodeOrder(mesh, layerRemaining);
                if (round.numDecimatedTotal <= 0 || round.vvDecodeFaceRanks.empty()) break;
                layerTotal += round.numDecimatedTotal;
                std::vector<int> used = round.vvDecodeFaceRanks;
                round.vvDecodeFaceRanks.clear();
                layerRounds.push_back(std::move(round));
                layerRemaining = filterRemainingStrictForwardRecords(layerRemaining, used);
            }
            if (layerTotal != layerInput.numDecimatedTotal) {
                ok = false;
                                break;
            }
            total += layerTotal;
            rounds.insert(rounds.end(),
                          std::make_move_iterator(layerRounds.begin()),
                          std::make_move_iterator(layerRounds.end()));
                    }
        if (!ok || total != forward.numDecimatedTotal) {
            combined.numDecimatedTotal = total;
            return combined;
        }
    }
    RDPatchPipelineResult remaining = forward;
    int total = 0;
    if (rounds.empty()) {
        const int maxSubRounds = envInt("PMSF_PCLTTM_SPLIT_REBUILD_MAX_SUBROUNDS", 32);
        for (int i = 0; i < maxSubRounds && remaining.numDecimatedTotal > 0; ++i) {
            RDPatchPipelineResult round =
                rebuildRDPatchResultInStrictDecodeOrder(mesh, remaining);
            if (round.numDecimatedTotal <= 0 || round.vvDecodeFaceRanks.empty()) break;
            total += round.numDecimatedTotal;
            std::vector<int> used = round.vvDecodeFaceRanks;
            round.vvDecodeFaceRanks.clear();
            rounds.push_back(std::move(round));
            remaining = filterRemainingStrictForwardRecords(remaining, used);
            if (envOn("PMSF_PCLTTM_SPLIT_REBUILD_VERBOSE")) {
                std::fprintf(stderr,
                             "[PCLTTMStrict] split rebuild subround %d removed=%d remaining=%d\n",
                             i + 1,
                             rounds.back().numDecimatedTotal,
                             remaining.numDecimatedTotal);
            }
        }
        if (rounds.empty()) {
            combined.numDecimatedTotal = total;
            return combined;
        }
    }

    const std::size_t nLayers = rounds.size();
    combined.layerSeedQs.assign(nLayers, std::array<int, 6>{});
    combined.numDecCycles = static_cast<int>(nLayers);
    for (std::size_t ri = 0; ri < rounds.size(); ++ri) {
        const unsigned short layer =
            static_cast<unsigned short>(nLayers - 1 - ri);
        RDPatchPipelineResult& round = rounds[ri];
        std::array<int, 6> seed{};
        if (!round.layerSeedQs.empty()) {
            seed = round.layerSeedQs.front();
        } else if (!round.symbolInputQs.empty()) {
            seed = round.symbolInputQs.front();
        }
        combined.layerSeedQs[static_cast<std::size_t>(layer)] = seed;
        if (ri == 0) {
            combined.firstGateV0Q[0] = seed[0];
            combined.firstGateV0Q[1] = seed[1];
            combined.firstGateV0Q[2] = seed[2];
            combined.firstGateV1Q[0] = seed[3];
            combined.firstGateV1Q[1] = seed[4];
            combined.firstGateV1Q[2] = seed[5];
        }

        std::size_t vCursor = 0;
        for (std::size_t si = 0; si < round.symbols.size(); ++si) {
            RDPatchSymbol s = round.symbols[si];
            s.phase = 0;
            s.bucket = -1;
            s.layer = layer;
            combined.symbols.push_back(s);
            if (si < round.symbolInputQs.size()) {
                combined.symbolInputQs.push_back(round.symbolInputQs[si]);
            } else {
                combined.symbolInputQs.push_back(std::array<int, 6>{});
            }
            if (s.kind == RDPatchSymbolKind::NULL_PATCH) continue;
            if (vCursor < round.vvResidualsX.size()) {
                combined.vvResidualsX.push_back(round.vvResidualsX[vCursor]);
                combined.vvResidualsY.push_back(round.vvResidualsY[vCursor]);
                combined.vvResidualsZ.push_back(round.vvResidualsZ[vCursor]);
            }
            if (vCursor < round.vvAbsQX.size() &&
                vCursor < round.vvAbsQY.size() &&
                vCursor < round.vvAbsQZ.size()) {
                combined.vvAbsQX.push_back(round.vvAbsQX[vCursor]);
                combined.vvAbsQY.push_back(round.vvAbsQY[vCursor]);
                combined.vvAbsQZ.push_back(round.vvAbsQZ[vCursor]);
            }
            if (vCursor < round.vvInputOffsets.size()) {
                combined.vvInputOffsets.push_back(round.vvInputOffsets[vCursor]);
            }
            const int ringN = vCursor < round.vvRingSizes.size()
                                  ? static_cast<int>(round.vvRingSizes[vCursor])
                                  : 0;
            combined.vvRingSizes.push_back(static_cast<unsigned char>(ringN));
            std::size_t ringStart = 0;
            for (std::size_t r = 0; r < vCursor && r < round.vvRingSizes.size(); ++r) {
                ringStart += static_cast<std::size_t>(round.vvRingSizes[r]);
            }
            for (int k = 0; k < ringN; ++k) {
                const std::size_t qi = ringStart + static_cast<std::size_t>(k);
                if (qi < round.vvRingQX.size() && qi < round.vvRingQY.size() &&
                    qi < round.vvRingQZ.size()) {
                    combined.vvRingQX.push_back(round.vvRingQX[qi]);
                    combined.vvRingQY.push_back(round.vvRingQY[qi]);
                    combined.vvRingQZ.push_back(round.vvRingQZ[qi]);
                }
            }
            ++vCursor;
            ++combined.numDecimatedTotal;
        }
        combined.numNullPatches += round.numNullPatches;
        combined.numSteps += round.numSteps;
        for (int i = 0; i < 6; ++i) combined.bucketHits[i] += round.bucketHits[i];
    }
    total = combined.numDecimatedTotal;
    if (total != forward.numDecimatedTotal) {
        const int remainingTail = forward.numDecimatedTotal - total;
        const int sparseTailLimit =
            envInt("PMSF_PCLTTM_SPARSE_TAIL_REBUILD_LIMIT", 32);
        if (remainingTail > 0 && remainingTail <= sparseTailLimit &&
            remaining.numDecimatedTotal == remainingTail) {
            combined.locatorRingPrefix = total;
            std::size_t ringCursor = 0;
            int vCursor = 0;
            for (std::size_t si = 0; si < remaining.symbols.size(); ++si) {
                RDPatchSymbol s = remaining.symbols[si];
                if (s.kind == RDPatchSymbolKind::NULL_PATCH) continue;
                s.phase = 0;
                s.bucket = -1;
                s.layer = 0;
                combined.symbols.push_back(s);
                if (si < remaining.symbolInputQs.size()) {
                    combined.symbolInputQs.push_back(remaining.symbolInputQs[si]);
                } else {
                    combined.symbolInputQs.push_back(std::array<int, 6>{});
                }
                if (vCursor < static_cast<int>(remaining.vvResidualsX.size())) {
                    combined.vvResidualsX.push_back(
                        remaining.vvResidualsX[static_cast<std::size_t>(vCursor)]);
                    combined.vvResidualsY.push_back(
                        remaining.vvResidualsY[static_cast<std::size_t>(vCursor)]);
                    combined.vvResidualsZ.push_back(
                        remaining.vvResidualsZ[static_cast<std::size_t>(vCursor)]);
                }
                if (vCursor < static_cast<int>(remaining.vvAbsQX.size()) &&
                    vCursor < static_cast<int>(remaining.vvAbsQY.size()) &&
                    vCursor < static_cast<int>(remaining.vvAbsQZ.size())) {
                    combined.vvAbsQX.push_back(
                        remaining.vvAbsQX[static_cast<std::size_t>(vCursor)]);
                    combined.vvAbsQY.push_back(
                        remaining.vvAbsQY[static_cast<std::size_t>(vCursor)]);
                    combined.vvAbsQZ.push_back(
                        remaining.vvAbsQZ[static_cast<std::size_t>(vCursor)]);
                }
                if (vCursor < static_cast<int>(remaining.vvInputOffsets.size())) {
                    combined.vvInputOffsets.push_back(
                        remaining.vvInputOffsets[static_cast<std::size_t>(vCursor)]);
                }
                const int ringN = vCursor < static_cast<int>(remaining.vvRingSizes.size())
                                      ? static_cast<int>(remaining.vvRingSizes[
                                            static_cast<std::size_t>(vCursor)])
                                      : 0;
                combined.vvRingSizes.push_back(static_cast<unsigned char>(ringN));
                for (int k = 0; k < ringN; ++k) {
                    const std::size_t qi = ringCursor + static_cast<std::size_t>(k);
                    if (qi < remaining.vvRingQX.size() &&
                        qi < remaining.vvRingQY.size() &&
                        qi < remaining.vvRingQZ.size()) {
                        combined.vvRingQX.push_back(remaining.vvRingQX[qi]);
                        combined.vvRingQY.push_back(remaining.vvRingQY[qi]);
                        combined.vvRingQZ.push_back(remaining.vvRingQZ[qi]);
                    }
                }
                ringCursor += static_cast<std::size_t>(std::max(ringN, 0));
                ++vCursor;
                ++combined.numDecimatedTotal;
            }
            combined.numVerticesIn = forward.numVerticesIn;
            combined.numVerticesOut = forward.numVerticesOut;
                    }
    }
    return combined;
}

RDPatchPipelineResult rebuildRDPatchResultInReverseRingOrder(
    std::shared_ptr<MCGAL::Mesh> mesh,
    const RDPatchPipelineResult& forward) {
    struct PendingRemoval {
        RDPatchSymbol sym{};
        std::array<int, 6> inputQ{};
        std::vector<QKey> ringQ;
        int rx = 0, ry = 0, rz = 0;
        int ax = 0, ay = 0, az = 0;
        unsigned char inputOffset = 0;
    };

    RDPatchPipelineResult out;
    out.mesh = mesh;
    out.numVerticesIn = forward.numVerticesIn;
    out.numVerticesOut = forward.numVerticesOut;
    out.leakyFactors = forward.leakyFactors;
    out.firstGateV0Q[0] = forward.firstGateV0Q[0];
    out.firstGateV0Q[1] = forward.firstGateV0Q[1];
    out.firstGateV0Q[2] = forward.firstGateV0Q[2];
    out.firstGateV1Q[0] = forward.firstGateV1Q[0];
    out.firstGateV1Q[1] = forward.firstGateV1Q[1];
    out.firstGateV1Q[2] = forward.firstGateV1Q[2];
    out.layerSeedQs = forward.layerSeedQs;
    if (!mesh) return out;

    std::vector<PendingRemoval> records;
    records.reserve(forward.vvResidualsX.size());
    size_t ringCoord = 0;
    int vCursor = 0;
    for (size_t si = 0; si < forward.symbols.size(); ++si) {
        const RDPatchSymbol& s = forward.symbols[si];
        if (s.kind == RDPatchSymbolKind::NULL_PATCH) continue;
        if (vCursor >= static_cast<int>(forward.vvResidualsX.size()) ||
            vCursor >= static_cast<int>(forward.vvResidualsY.size()) ||
            vCursor >= static_cast<int>(forward.vvResidualsZ.size()) ||
            vCursor >= static_cast<int>(forward.vvRingSizes.size())) {
            break;
        }
        PendingRemoval rec;
        rec.sym = s;
        rec.sym.phase = 0;
        rec.sym.layer = 0;
        rec.sym.bucket = -1;
        if (si < forward.symbolInputQs.size()) rec.inputQ = forward.symbolInputQs[si];
        rec.rx = forward.vvResidualsX[static_cast<size_t>(vCursor)];
        rec.ry = forward.vvResidualsY[static_cast<size_t>(vCursor)];
        rec.rz = forward.vvResidualsZ[static_cast<size_t>(vCursor)];
        if (vCursor < static_cast<int>(forward.vvAbsQX.size()) &&
            vCursor < static_cast<int>(forward.vvAbsQY.size()) &&
            vCursor < static_cast<int>(forward.vvAbsQZ.size())) {
            rec.ax = forward.vvAbsQX[static_cast<size_t>(vCursor)];
            rec.ay = forward.vvAbsQY[static_cast<size_t>(vCursor)];
            rec.az = forward.vvAbsQZ[static_cast<size_t>(vCursor)];
        }
        if (vCursor < static_cast<int>(forward.vvInputOffsets.size())) {
            rec.inputOffset = forward.vvInputOffsets[static_cast<size_t>(vCursor)];
        }
        const int n = static_cast<int>(forward.vvRingSizes[static_cast<size_t>(vCursor)]);
        rec.ringQ.reserve(static_cast<size_t>(std::max(n, 0)));
        for (int k = 0; k < n; ++k) {
            const size_t qi = ringCoord + static_cast<size_t>(k);
            if (qi < forward.vvRingQX.size() &&
                qi < forward.vvRingQY.size() &&
                qi < forward.vvRingQZ.size()) {
                rec.ringQ.push_back({forward.vvRingQX[qi],
                                     forward.vvRingQY[qi],
                                     forward.vvRingQZ[qi]});
            }
        }
        ringCoord += static_cast<size_t>(std::max(n, 0));
        records.push_back(std::move(rec));
        ++vCursor;
    }

    mesh->resetState();
    mesh->invalidateAllVertexDegrees();
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (v && !v->isRemoved()) v->clearTag();
    }
    auto qToVertex = buildQVertexMap(mesh);

    auto ringForRecord = [&](const PendingRemoval& rec) {
        std::vector<MCGAL::Vertex*> ring;
        ring.reserve(rec.ringQ.size());
        for (const QKey& q : rec.ringQ) {
            MCGAL::Vertex* v = findQVertexWithTolerance(qToVertex, q);
            if (!v || v->isRemoved()) {
                ring.clear();
                return ring;
            }
            ring.push_back(v);
        }
        return ring;
    };
    auto locatePatchFaces = [&](const std::vector<MCGAL::Vertex*>& ring,
                                int valence,
                                std::vector<int>& patchRanks,
                                MCGAL::Facet*& patchFace) {
        patchRanks.clear();
        patchFace = nullptr;
        if (static_cast<int>(ring.size()) != valence || valence < 3) return false;
        std::set<MCGAL::Vertex*> ringSet(ring.begin(), ring.end());
        std::vector<MCGAL::Facet*> patchFaces;
        MCGAL::Facet* polygonFace = nullptr;
        const bool singleRankSideInfo = envOn("PMSF_PCLTTM_SINGLE_RANKS");
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved() || !faceVerticesWithin(f, ringSet)) continue;
            const int deg = faceDegree(f);
            if (deg == valence) {
                polygonFace = f;
                break;
            }
            if (deg == 3) {
                patchFaces.push_back(f);
            }
        }
        if (polygonFace) {
            const int rank = livingFaceRank(mesh, polygonFace);
            if (rank < 0) return false;
            patchRanks.assign(
                static_cast<size_t>(singleRankSideInfo ? 1 : std::max(1, valence - 2)),
                rank);
            patchFace = polygonFace;
            return true;
        }
        PatchCandidate patch;
        if (!validateTriangulatedPatch(patchFaces, valence, patch)) return false;
        patchRanks.reserve(patch.faces.size());
        for (MCGAL::Facet* f : patch.faces) {
            const int rank = livingFaceRank(mesh, f);
            if (rank < 0) return false;
            patchRanks.push_back(rank);
        }
        if (singleRankSideInfo && !patchRanks.empty()) {
            patchRanks.resize(1);
        } else {
            std::sort(patchRanks.begin(), patchRanks.end());
        }
        patchFace = mergePatchToPolygon(patch, valence);
        return patchFace && !patchFace->isRemoved() && faceDegree(patchFace) == valence;
    };

    int failed = 0;
    int opIndex = 0;
    for (auto it = records.rbegin(); it != records.rend(); ++it) {
        const int valence = symbolValence(it->sym);
        std::vector<MCGAL::Vertex*> ring = ringForRecord(*it);
        MCGAL::Facet* patchFace = nullptr;
        std::vector<int> patchRanks;
        if (!locatePatchFaces(ring, valence, patchRanks, patchFace)) {
            if (failed == 0) {
                std::fprintf(stderr,
                             "[RD_STRICT reverse ring order] first failure op=%d "
                             "valence=%d ring=%zu expected=%zu\n",
                             opIndex, valence, ring.size(), it->ringQ.size());
            }
            ++failed;
            break;
        }
        QKey absQ{it->ax, it->ay, it->az};
        MCGAL::Vertex* inserted = nullptr;
        if (!insertDecodedVertexOnFace(mesh, patchFace,
                                       out.vvResidualsX,
                                       out.vvResidualsY,
                                       out.vvResidualsZ,
                                       -1,
                                       &absQ,
                                       &inserted) ||
            !inserted) {
            if (failed == 0) {
                std::fprintf(stderr,
                             "[RD_STRICT reverse ring order] first failure op=%d "
                             "reason=insert valence=%d patchDegree=%d ranks=%zu\n",
                             opIndex, valence, faceDegree(patchFace),
                             patchRanks.size());
            }
            ++failed;
            break;
        }
        qToVertex[absQ] = inserted;
        out.symbols.push_back(it->sym);
        out.symbolInputQs.push_back(it->inputQ);
        out.vvResidualsX.push_back(it->rx);
        out.vvResidualsY.push_back(it->ry);
        out.vvResidualsZ.push_back(it->rz);
        out.vvAbsQX.push_back(it->ax);
        out.vvAbsQY.push_back(it->ay);
        out.vvAbsQZ.push_back(it->az);
        out.vvInputOffsets.push_back(it->inputOffset);
        out.vvDecodeFaceRanks.insert(out.vvDecodeFaceRanks.end(),
                                     patchRanks.begin(), patchRanks.end());
        out.vvRingSizes.push_back(static_cast<unsigned char>(it->ringQ.size()));
        for (const QKey& q : it->ringQ) {
            out.vvRingQX.push_back(q.x);
            out.vvRingQY.push_back(q.y);
            out.vvRingQZ.push_back(q.z);
        }
        ++out.numDecimatedTotal;
        ++opIndex;
    }
    out.numVerticesIn = out.numVerticesOut + out.numDecimatedTotal;
    out.numDecCycles = 1;
    if (failed || out.numDecimatedTotal != static_cast<int>(records.size())) {
        std::fprintf(stderr,
                     "[RD_STRICT reverse ring order] WARNING emitted %d/%zu "
                     "rank-located removals failed=%d\n",
                     out.numDecimatedTotal, records.size(), failed);
    }
    return out;
}

}  // namespace pmsf
