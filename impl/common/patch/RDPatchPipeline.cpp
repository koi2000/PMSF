// SPDX-License-Identifier: Apache-2.0
//
// RDPatchPipeline.cpp —— PR-M1 (2026-05-12) RD 论文 §2 patch-level conquest
// 主循环驱动实装。
//
// 详见 RDPatchPipeline.h 头文件契约 + docs/paper_alignment/PR_M_RD_PATCH_DESIGN.md。
//
// 整体流程（PR-M1 简化版）：
//
//   1. load mesh from .off
//   2. PMSF/contextPool init（与 RDTag 路径同源），attach kVPropRDR1
//   3. operator init: select(NULL —— 我们不需要 select，conquest 自己驱动) +
//      eliminate(RDTagEliminateOperator) + isRemovable 链 (BasicIsRemovable +
//      UnifiedManifoldIsRemovable) —— 其中 isRemovable 通过 SeedIsRemovable 风格
//      链显式判，跳过 SelectOperator 抽象
//   4. first_gate = mesh.longest_edge() —— 与 RD baseline customSeedSelector 一致
//   5. gate_queue.push(first_gate, bucket=5)
//   6. main loop:
//        while !queue.empty() OR (cycle 内 phase 还未都跑过):
//          h = queue.pop()
//          if phase == DEC:
//              front_v = h->end_vertex()
//              if 3 <= deg <= 6 and is_removable(h):
//                  emit V_v + erase + push 新 boundary edges
//              else:
//                  emit NULL_PATCH（PR-M1 不真改 mesh，仅记录）
//          else: # CLN
//              if deg == 3 and is_removable(h):
//                  emit V_3 + erase + push
//          if queue empty: switch phase; check 1 cycle no-progress → break
//   7. dump diag
//
// 关键 invariants（M1 阶段简化）：
//   - vertex tag 全部视为 Untagged（→ bucket = 5/6 only）
//   - patch null-flag 全部视为 NonNull（→ bucket = 5）
//   - 不做 incident-in-processed 跟踪 —— saturated 永远 false
//   即 PR-M1 的所有 push 都进 bucket 5。priority API 仍完整保留以便 M2/M3 接入。

#include "RDPatchPipeline.h"
#include "common/BoundedRingTraversal.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <set>
#include <string>
#include <unordered_set>

#include "ContextPool.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "core/MeshRuntime.h"
#include "Point.h"
#include "Vertex.h"
#include "MeshUtils.h"

#include "common/HalfedgeErrorEntry.h"
#include "common/PCLTTMTagTable.h"
#include "common/PMSFRetriangulation.h"
#include "PCLTTM/boundary/PCLTTMPMSFBoundary.h"
#include "predict/CurvaturePredictor.h"
#include "common/patch/RDPatchGateQueue.h"
#include "RD/elimination/RDTagEliminateOperator.h"
#include "header/PMSFHeader.h"
#include "operator/BasicIsRemovableOperator.h"
#include "operator/UnifiedManifoldIsRemovableOperator.h"
#include "predict/RDPredictor.h"  // kVPropRDR1


namespace pmsf {


// P10 阶段 0：前沿集合驱动纯规则遍历的卡死率统计（只读）。
long long g_p10Pops = 0;
long long g_p10BusyPops = 0;       // 队列非空的 pop 次数
long long g_p10Stalls = 0;         // 队列非空但前沿为空的次数 = 纯规则当场卡死
long long g_p10QueueSum = 0;       // 队列长度累计
long long g_p10FrontierSum = 0;    // 队列中落在前沿上的 gate 数累计
long long g_p10FirstStallStep = -1;
// 卡死发生在本轮第几步（最多记 16 个）。**必须能区分 step=0 的仪器假象**：
// 轮首尚未处理任何面 ⇒ 所有 countProcessedFaces 全为 0 ⇒ 前沿按定义为空。
// 而轮首那条 gate 是**外部给定的种子**（不需要从前沿推出），故 step=0 的"卡死"
// 不是真卡死。只有 step>0 的卡死才是纯规则的实质障碍。
long long g_p10StallSteps[16] = {};
int g_p10StallStepCount = 0;
// 疑似卡死点上的**全网格前沿穷举**结果（把"队列假象"与"真卡死"分开）：
long long g_p10FalseStalls = 0;          // 队列里没有，但全网格前沿仍有 gate ⇒ 纯规则可动
long long g_p10TrueStalls = 0;           // 全网格前沿为空 ⇒ 纯规则真的无着法
long long g_p10MeshFrontierAtStall = 0;  // 疑似卡死点上全网格前沿 gate 数累计
long long g_p10MeshUsableAtStall = 0;    // 同上，但只数 face 未处理的**可用** gate
// 真卡死点的逐事件留证（合计值会与假卡死点混淆，故单独存）。
long long g_p10TrueStallStep[16] = {};
long long g_p10TrueStallFrontier[16] = {};
long long g_p10TrueStallUsable[16] = {};
long long g_p10TrueStallQueue[16] = {};
int g_p10TrueStallCount = 0;

// P10 阶段 0 续：**真的跑前沿驱动规则**时的计数（PMSF_P10_RULE=1 才动）。
long long g_p10RulePops = 0;          // 规则模式下的 pop 总数
long long g_p10RuleFallbacks = 0;     // 前沿无可用 gate ⇒ 回落队列首的次数
long long g_p10RuleFallbackAt[16] = {};
int g_p10RuleFallbackSteps = 0;
// 队列构成分解（规则模式下累计）：用来回答"回落率高是因为可用 gate 少，
// 还是因为前沿判据把可用 gate 也排除了"——这两者的修法完全不同。
long long g_p10RuleFallbacksDec = 0;   // 其中 DEC 相位
long long g_p10RuleFallbacksCln = 0;   // 其中 CLN 相位
long long g_p10RuleQueueSum = 0;         // 队列长度累计
long long g_p10RuleUsableSum = 0;        // 其中 face 未处理（可用）累计
long long g_p10RuleUsableFrontSum = 0;   // 其中**既可用又在前沿上**累计

namespace {


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

inline void requirePaperFig9ExactTableIfRequested() {
    static const bool required = pmsf::boundaryFlagOn(pmsf::pclttm_boundary::kFig9ExactSpec);
    if (required) {
        throw std::runtime_error(
            "[PMSFRetriangulation] PMSF_PCLTTM_REQUIRE_FIG9_EXACT=1, "
            "but the literal Alliez-Desbrun Fig.9 64-entry table has not "
            "been imported; current code only provides the shared executable "
            "model fallback");
    }
}

inline void parseCurvatureLambdaEnv(int& la, int& lb, int& lg) {
    la = lb = lg = 1;
    const char* env = std::getenv("PMSF_RDPATCH_LAMBDA");
    if (!env || !*env) return;
    if (std::strcmp(env, "0") == 0 || std::strcmp(env, "1") == 0 || std::strcmp(env, "2") == 0) {
        la = lb = lg = env[0] - '0';
        return;
    }
    int a = 1, b = 1, c = 1;
    if (std::sscanf(env, "%d,%d,%d", &a, &b, &c) == 3 ||
        std::sscanf(env, "%d:%d:%d", &a, &b, &c) == 3) {
        la = std::clamp(a, 0, 2);
        lb = std::clamp(b, 0, 2);
        lg = std::clamp(c, 0, 2);
    }
}

inline bool isValidGate(MCGAL::Halfedge* h) {
    return h && !h->isRemoved() && h->face() && !h->face()->isRemoved() &&
           h->vertex() && !h->vertex()->isRemoved() &&
           h->end_vertex() && !h->end_vertex()->isRemoved();
}

inline MCGAL::Halfedge* inputBoundaryFromGate(MCGAL::Halfedge* h) {
    if (!h || h->isRemoved() || !h->next() || !h->next()->next()) return nullptr;
    return h->next()->next();
}

inline MCGAL::Halfedge* gateFromInputBoundary(MCGAL::Halfedge* boundary) {
    if (!boundary || boundary->isRemoved() || !boundary->next()) return nullptr;
    return boundary->next();
}

// PR-R (2026-05-12) helpers：vertex tag / face nullFlag / saturated 检查，
// 用于 6-bucket priority queue 的 computeBucket 真实填参。
// 双端 mirror：encoder/decoder 都用同样规则。
//
// vertex tag 来源：MCGAL::Vertex::TagFlag (Untagged/Plus/Minus)
// face nullFlag 来源：mesh-wide std::set<Facet*> nullPatchFaces_（encoder/decoder 各自维护）
// saturated 定义：vertex 1-ring 中 face::isProcessed() 数量 ∈ [2, 3]
//   ("two or three incident edges in remeshed region" 论文 §2.2 step 1)

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

// 数 vertex 1-ring 中 face::isProcessed() 数量。论文"incident edges in remeshed region"
// 实际是数已被 conquest 处理的 face（incident edge 数 ≈ incident face 数 in manifold mesh）。
inline int countProcessedFaces(MCGAL::Vertex* v) {
    if (!v || v->isRemoved()) return 0;
    int n = 0;
    for (MCGAL::Halfedge* he : boundedVertexRing(v)) {
        if (!he || he->isRemoved()) continue;
        MCGAL::Facet* f = he->face();
        if (f && !f->isRemoved() && f->isProcessed()) ++n;
    }
    return n;
}

// P10 阶段 0：**前沿判据**（纯函数，只读网格状态）。
// gate 落在"已处理/未处理交界"上 ⟺ 两端顶点的 `countProcessedFaces` 不同。
// 该判据与 P9 判决使用的逐顶点判据逐字相同；两端都可从各自网格独立算出。
inline bool isOnFrontier(MCGAL::Halfedge* h) {
    if (!h || h->isRemoved()) return false;
    MCGAL::Vertex* va = h->vertex();
    MCGAL::Vertex* vb = h->end_vertex();
    if (!va || !vb || va->isRemoved() || vb->isRemoved()) return false;
    return countProcessedFaces(va) != countProcessedFaces(vb);
}

inline bool isVertexSaturated(MCGAL::Vertex* v) {
    const int n = countProcessedFaces(v);
    // PR-R: 论文是 [2, 3]。env PMSF_RDPATCH_SAT_RANGE=relaxed 放宽到 [1, 4]
    // 让 bucket 1/2 多命中。实测命中率：[2,3] 几乎无命中；[1,4] 让 bucket 1/2 真激活。
    static const char* envSat = std::getenv("PMSF_RDPATCH_SAT_RANGE");
    static const bool relaxed = envSat && std::strcmp(envSat, "relaxed") == 0;
    if (relaxed) return n >= 1 && n <= 4;
    return n == 2 || n == 3;
}

bool frontPatchTouchesProcessedFace(MCGAL::Vertex* v) {
    if (!v || v->isRemoved()) return true;
    for (MCGAL::Halfedge* he : boundedVertexRing(v)) {
        if (!he || he->isRemoved()) continue;
        MCGAL::Facet* f = he->face();
        if (f && !f->isRemoved() && f->isProcessed()) return true;
    }
    return false;
}

// PR-R bucket 计算：传入 halfedge h，自动查 src/dst tag、saturated、face null 状态。
// 返回 6-bucket 编号 1..6。
//
// **PR-R 实测结论**：完整 6-bucket 在本 conquest 实装下 TotalBPV 略恶化
// （bunny 27.61 vs PR-Q 27.36, +0.9%）。原因：(a) ConBPV 已 98% Shannon 下界
// 不可降；(b) 6-bucket 分散 push 顺序 → 删点效率低 → V_base 大 → BaseBPV 多。
// **default 关闭 PR-R 行为**，等价于 PR-Q（push 用固定 bucket 5）。
// env PMSF_RDPATCH_PR_R=1 启用 PR-R 真实 6-bucket（用于 ablation 对比）。
inline int computeRealBucket(MCGAL::Halfedge* h,
                             const std::set<MCGAL::Facet*>& nullPatchFaces,
                             bool forceSixBucket = false) {
    if (!h) return 6;
    const char* env6 = std::getenv("PMSF_RDPATCH_USE_6BUCKET");
    if (!forceSixBucket && (!env6 || env6[0] != '1')) return 5;
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

// P10 阶段 0 续：**前沿驱动的纯规则选门**（真的改变遍历，用于在构造出的序列上
// 重跑卡死测量 —— 此前测的是既有 push 路径上的前沿行为，走的不是同一条路）。
//
// 候选集（纯函数，只读网格状态）：
//     { gate ∈ 队列 : 位于前沿（两端 countProcessedFaces 不同）
//                     ∧ 可用（face 存在、未移除、未处理） }
// 序：bucket 升序；同 bucket 按**已处理侧端点的量化位置**字典序。
//     只用已处理侧，是因为前端点位置在选门时**尚未解码** —— 这是 P10.1 里
//     真正可解码的 tie-break 的最小候选；用前端点位置会做出解码端算不出的序。
//
// 前沿为空 ⇒ 返回 nullptr（调用方回落队列首并计数）。**回落次数就是"若要变成
// 纯规则还需多少例外通道"**。
inline MCGAL::Halfedge* p10PickFrontierGate(MCGAL::Mesh* mesh,
                                            RDPatchGateQueue& q,
                                            const std::set<MCGAL::Facet*>& nullPatchFaces,
                                            bool forceSixBucket,
                                            const std::set<MCGAL::Facet*>& handled,
                                            long long* outTotal = nullptr,
                                            long long* outUsable = nullptr,
                                            long long* outUsableFrontier = nullptr) {
    MCGAL::Halfedge* best = nullptr;
    int bestBucket = 0;
    long long bestKey[3] = {0, 0, 0};
    long long nTotal = 0, nUsable = 0, nUsableFrontier = 0;
    q.forEachEntry([&](MCGAL::Halfedge* h) {
        if (!h || h->isRemoved()) return false;
        ++nTotal;
        MCGAL::Facet* f = h->face();
        // 可用判据 = **本相位尚未处理过该面**，与主循环的 dedup 同源
        // （`facesProcessedInPhase.count(gateFace)`）。**不要**用 `f->isProcessed()`：
        // 那个标志是"被 patch 内部标记过"，与"这条 gate 是否还有活干"不是一回事，
        // 用它会把队列里绝大多数条目误判为不可用（实测 0.1%）。
        if (!f || f->isRemoved() || handled.count(f)) return false;
        ++nUsable;
        if (!isOnFrontier(h)) return false;
        ++nUsableFrontier;
        MCGAL::Vertex* va = h->vertex();
        MCGAL::Vertex* vb = h->end_vertex();
        if (!va || !vb) return false;
        // 已处理侧端点 = 已处理面数较大者（相等时不可能：前沿判据已排除）
        const int ca = countProcessedFaces(va);
        const int cb = countProcessedFaces(vb);
        MCGAL::Vertex* vProc = (ca > cb) ? va : vb;
        const MCGAL::PointInt qi = mesh->floatPosToInt(vProc->point());
        const int bucket = computeRealBucket(h, nullPatchFaces, forceSixBucket);
        const long long key[3] = {qi[0], qi[1], qi[2]};
        if (best == nullptr || bucket < bestBucket ||
            (bucket == bestBucket && (key[0] < bestKey[0] ||
                                      (key[0] == bestKey[0] &&
                                       (key[1] < bestKey[1] ||
                                        (key[1] == bestKey[1] && key[2] < bestKey[2])))))) {
            best = h;
            bestBucket = bucket;
            bestKey[0] = key[0];
            bestKey[1] = key[1];
            bestKey[2] = key[2];
        }
        return false;
    });
    if (outTotal) *outTotal += nTotal;
    if (outUsable) *outUsable += nUsable;
    if (outUsableFrontier) *outUsableFrontier += nUsableFrontier;
    return best;
}

// 把 valence 映射到 RDPatch alphabet。论文：3..6 → V_3..V_6；其他 → V_OTHER + escape。
RDPatchSymbolKind valenceToKind(int v) {
    switch (v) {
        case 3: return RDPatchSymbolKind::V_3;
        case 4: return RDPatchSymbolKind::V_4;
        case 5: return RDPatchSymbolKind::V_5;
        case 6: return RDPatchSymbolKind::V_6;
        default: return RDPatchSymbolKind::V_OTHER;
    }
}

// 找 mesh 中所有 face 上"几何最长"的 halfedge。与 PMSF::rdColumnarCompress 的
// customSeedSelector 同源（以 length() 为度量；同长情况下取最早遍历到的）。
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

// PR-M1 简化：收集被删 vertex 的 1-ring，erase+retriangulate 后从这些 vertex
// 之间找 halfedge 作为新 gate push。这不是论文 step 7 的严格 priority push，
// 但对"跑通到 base mesh"的验收目标足够。M2 起会按论文严格化。
struct RingSnapshot {
    std::vector<MCGAL::Vertex*> ringVerts;
};

RingSnapshot snapshotRing(MCGAL::Vertex* center) {
    RingSnapshot s;
    if (!center) return s;
    s.ringVerts.reserve(8);
    auto liveOutgoing = [&](MCGAL::Halfedge* h) {
        return h && !h->isRemoved() && h->vertex() == center &&
               h->end_vertex() && h->end_vertex() != center && h->opposite();
    };
    MCGAL::Halfedge* start = liveOutgoing(center->proxyHalfedge())
                                 ? center->proxyHalfedge()
                                 : nullptr;
    if (!start) {
        for (MCGAL::Halfedge* hit : boundedVertexRing(center)) {
            if (liveOutgoing(hit)) {
                start = hit;
                break;
            }
        }
    }
    if (start) {
        std::unordered_set<MCGAL::Halfedge*> seenHalfedges;
        MCGAL::Halfedge* h = start;
        for (int guard = 0; h && guard < 256; ++guard) {
            if (!liveOutgoing(h) || !seenHalfedges.insert(h).second) break;
            s.ringVerts.push_back(h->end_vertex());
            MCGAL::Halfedge* next = h->opposite() ? h->opposite()->next() : nullptr;
            if (next == start) break;
            h = next;
        }
    }
    const int expectedDegree = center->is_degree_dirty()
                                   ? center->vertex_degree()
                                   : center->vertex_degree_cached();
    if (expectedDegree >= 3 &&
        static_cast<int>(s.ringVerts.size()) == expectedDegree) {
        return s;
    }

    // Fallback for stale/non-manifold vertex halfedge rings. This preserves the
    // historical behavior but should not be used on the closed manifold APTM path.
    s.ringVerts.clear();
    std::unordered_set<MCGAL::Vertex*> dedup;
    for (MCGAL::Halfedge* hit : boundedVertexRing(center)) {
        if (!liveOutgoing(hit)) continue;
        MCGAL::Vertex* rv = hit->end_vertex();
        if (rv && dedup.insert(rv).second) {
            s.ringVerts.push_back(rv);
        }
    }
    return s;
}

// O(N²) all-pair gate push：确保 conquest 充分探索 mesh，快速收敛。
// BFS-only O(N) push 虽改善 ConBPV (-0.34) 但严重拖慢收敛(24→100轮) 且 GeoBPV +1.5。
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

bool sameUndirectedEdge(MCGAL::Halfedge* h,
                        MCGAL::Vertex* a,
                        MCGAL::Vertex* b) {
    if (!h || !a || !b) return false;
    return (h->vertex() == a && h->end_vertex() == b) ||
           (h->vertex() == b && h->end_vertex() == a);
}

std::vector<MCGAL::Halfedge*> boundedFaceHalfedges(MCGAL::Facet* f, int maxSteps = 64) {
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

MCGAL::Halfedge* findFaceHalfedgeBetween(MCGAL::Facet* f,
                                         MCGAL::Vertex* a,
                                         MCGAL::Vertex* b) {
    if (!f || !a || !b || f->isRemoved()) return nullptr;
    for (MCGAL::Halfedge* he : boundedFaceHalfedges(f)) {
        if (!he || he->isRemoved()) continue;
        if (sameUndirectedEdge(he, a, b)) return he;
    }
    return nullptr;
}

MCGAL::Halfedge* findExteriorHalfedgeBetween(MCGAL::Vertex* a,
                                             MCGAL::Vertex* b) {
    if (!a || !b || a->isRemoved() || b->isRemoved()) return nullptr;
    MCGAL::Halfedge* ab = a->find_halfedge(b);
    MCGAL::Halfedge* ba = b->find_halfedge(a);
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

bool faceHasVertexOutsideSet(MCGAL::Facet* f,
                             const std::set<MCGAL::Vertex*>& verts) {
    if (!f || f->isRemoved()) return false;
    for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
        if (!h || h->isRemoved() || !h->vertex()) continue;
        if (!verts.count(h->vertex())) return true;
    }
    return false;
}

MCGAL::Halfedge* findExteriorHalfedgeBetween(MCGAL::Vertex* a,
                                             MCGAL::Vertex* b,
                                             const std::set<MCGAL::Vertex*>& patchBoundary) {
    if (!a || !b || a->isRemoved() || b->isRemoved()) return nullptr;
    MCGAL::Halfedge* ab = a->find_halfedge(b);
    MCGAL::Halfedge* ba = b->find_halfedge(a);
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

void markPatchInteriorFacesProcessed(const std::vector<MCGAL::Vertex*>& ringVerts,
                                     std::set<MCGAL::Facet*>& facesProcessedInPhase) {
    std::set<MCGAL::Vertex*> boundary;
    for (MCGAL::Vertex* v : ringVerts) {
        if (v && !v->isRemoved()) boundary.insert(v);
    }
    if (boundary.empty()) return;
    std::set<MCGAL::Facet*> seen;
    const int n = static_cast<int>(ringVerts.size());
    for (int i = 0; i < n; ++i) {
        MCGAL::Vertex* a = ringVerts[static_cast<size_t>(i)];
        MCGAL::Vertex* b = ringVerts[static_cast<size_t>((i + 1) % n)];
        if (!a || !b || a->isRemoved() || b->isRemoved()) continue;
        MCGAL::Halfedge* ab = a->find_halfedge(b);
        MCGAL::Halfedge* ba = b->find_halfedge(a);
        MCGAL::Halfedge* candidates[4] = {
            ab,
            ab ? ab->opposite() : nullptr,
            ba,
            ba ? ba->opposite() : nullptr,
        };
        for (MCGAL::Halfedge* h : candidates) {
            MCGAL::Facet* f = h ? h->face() : nullptr;
            if (!f || f->isRemoved() || !seen.insert(f).second) continue;
            if (faceHasVertexOutsideSet(f, boundary)) continue;
            f->setProcessedFlag();
            facesProcessedInPhase.insert(f);
        }
    }
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

// Paper Figure 10(a): after deleting a valence-v patch, push the v-1 output
// gates on the patch boundary except the input edge. The gate is oriented into
// the exterior face through the opposite halfedge, just like null-patch
// propagation pushes neighboring faces.
std::string traceQuantVertex(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Vertex* v) {
    if (!mesh || !v || v->isRemoved()) return "nil";
    MCGAL::PointInt q = mesh->floatPosToInt(v->point());
    return std::to_string(q[0]) + "," + std::to_string(q[1]) + "," +
           std::to_string(q[2]);
}

void pushPaperDecimatingGates(std::shared_ptr<MCGAL::Mesh> mesh,
                              const std::vector<MCGAL::Vertex*>& ringVerts,
                              MCGAL::Vertex* inputA,
                              MCGAL::Vertex* inputB,
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
        if ((a == inputA && b == inputB) || (a == inputB && b == inputA)) {
            continue;
        }
        MCGAL::Halfedge* gate =
            gateFromInputBoundary(findExteriorHalfedgeBetween(a, b, boundary));
        if (std::getenv("PMSF_PCLTTM_TRACE_PUSH")) {
            MCGAL::Facet* gf = gate ? gate->face() : nullptr;
            std::fprintf(stderr,
                         "[PCLTTM trace enc push] edge=(%s)-(%s) gate_input=(%s)-(%s) "
                         "usable=%d faceProcessed=%d already=%d faceQueued=%d\n",
                         traceQuantVertex(mesh, a).c_str(),
                         traceQuantVertex(mesh, b).c_str(),
                         gate && inputBoundaryFromGate(gate)
                             ? traceQuantVertex(mesh, inputBoundaryFromGate(gate)->vertex()).c_str()
                             : "nil",
                         gate && inputBoundaryFromGate(gate)
                             ? traceQuantVertex(mesh, inputBoundaryFromGate(gate)->end_vertex()).c_str()
                             : "nil",
                         gate && !gate->isRemoved() && gf && !gf->isRemoved() ? 1 : 0,
                         gf && gf->isProcessed() ? 1 : 0,
                         alreadyEnqueued.count(gate) ? 1 : 0,
                         gf && facesEnqueued.count(gf) ? 1 : 0);
        }
        pushGateIfUsable(gate, q, alreadyEnqueued, facesEnqueued, nullPatchFaces,
                         forceSixBucket);
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

// Paper Figure 10(b): cleaning conquest flags each exterior face adjacent to
// the deleted valence-3 patch boundary as conquered and pushes the two other
// gates of that exterior face.
void pushPaperCleaningGates(const std::vector<MCGAL::Vertex*>& ringVerts,
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
        MCGAL::Halfedge* exteriorGate = findExteriorHalfedgeBetween(a, b, boundary);
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

MCGAL::Vector3 polygonNormal(const std::vector<MCGAL::Vertex*>& ringVerts) {
    MCGAL::Vector3 n(0, 0, 0);
    const int sz = static_cast<int>(ringVerts.size());
    if (sz < 3) return n;
    for (int i = 0; i < sz; ++i) {
        MCGAL::Vertex* a = ringVerts[static_cast<size_t>(i)];
        MCGAL::Vertex* b = ringVerts[static_cast<size_t>((i + 1) % sz)];
        if (!a || !b || a->isRemoved() || b->isRemoved()) continue;
        MCGAL::Vector3 va(a->point());
        MCGAL::Vector3 vb(b->point());
        n = n + (va % vb);
    }
    if (n.length() == 0) return n;
    return n.normalize();
}

// PR-S (2026-05-12) diagnostic：分别计数 isRemovable 拒绝的原因。

// 重写 isVertexRemovable 内联检查，分原因计数。模拟 BasicIsRemovableOperator + UnifiedManifold。

// 跑可移除性检查链 Basic + UnifiedManifold（与 RD baseline 装配同源）。
// 跑可移除性检查链 Basic + UnifiedManifold（与 RD baseline 装配同源）。
// PR-S (2026-05-12)：env PMSF_RDPATCH_BYPASS_ISADDED=1 时绕过 isAdded 检查 +
// 直接复用 UnifiedManifold 的 polygon 构造 + manifold violation check。
// 理由：isAdded 标记新 retriangulated halfedge，BFS-style baseline 用它隔离删点
// 区域；但 RDPatch 是 patch-based conquest，论文 Fig. 2 允许相邻 patch 立即处理。
// 实测：单轮删点率从 17% → ?（待测）。
bool isVertexRemovableBypassIsAdded(MCGAL::Halfedge* h) {
    if (!h || h->isRemoved()) return false;
    MCGAL::Vertex* vSrc = h->vertex();
    MCGAL::Vertex* vDst = h->end_vertex();
    if (!vSrc || !vDst || vSrc->isRemoved() || vDst->isRemoved()) return false;
    if (vSrc->isCollapsed() || vDst->isCollapsed()) return false;
    if (!vDst->isRemovable()) return false;
    if (h->face()->isRemoved()) return false;

    // 度数剪枝
    int deg = vDst->is_degree_dirty() ? vDst->vertex_degree() : vDst->vertex_degree_cached();
    if (deg < 3 || deg >= 17) return false;

    // 构造 1-ring polygon（mirror UnifiedManifold 内部代码，但跳过所有 isAdded 检查）
    bool isBoundary = vDst->isBoundary();
    std::vector<MCGAL::Vertex*> polyVertices;
    polyVertices.reserve(deg);
    if (isBoundary) {
        MCGAL::Halfedge* st = h;
        while (st->next()->opposite() != nullptr) st = st->next()->opposite();
        st = st->find_prev();
        polyVertices.push_back(st->vertex());
        while (true) {
            if (st->face()->isSplittable()) return false;  // 保留 splittable 检查
            polyVertices.push_back(st->end_vertex());
            if (st->next()->opposite() == nullptr) break;
            st = st->next()->opposite()->next();
        }
    } else {
        for (MCGAL::Halfedge* outH : boundedVertexRing(vDst)) {
            MCGAL::Halfedge* edge = outH->next();
            while (edge->end_vertex() != vDst) {
                polyVertices.push_back(edge->vertex());
                edge = edge->next();
            }
        }
    }
    int n = polyVertices.size();
    if (n < 3 || n >= 17) return false;
    return !UnifiedManifoldIsRemovableOperator::checkManifoldViolation(
        vDst, polyVertices, !isBoundary);
}

// PR-T (2026-05-12)：默认 bypass isAdded（论文对齐），env PMSF_RDPATCH_LEGACY_ISADDED=1
// 退回旧 BasicIsRemovable + UnifiedManifold 链。
bool isVertexRemovable(MCGAL::Halfedge* h,
                       BasicIsRemovableOperator& basic,
                       UnifiedManifoldIsRemovableOperator& unifiedManifold) {
    // 论文 §2.2 真正的 isRemovable 规则极简：
    //   1. front vertex valence ∈ [3..6]（DEC）或 ==3（CLN）—— conquest 主循环已检查
    //   2. polygon 拓扑 valid（无 chord edge / duplicate）—— UnifiedManifold check
    // 没有"isAdded 隔离"——那是仓库 baseline 的 BFS-style 残留，让每轮删点率
    // 从论文 36% 降到 17%。
    //
    // **决策**（用户要求"推进激进删点方案"）：default 走 bypass 路径。
    //   - 单轮删点率 44%（≥ 论文 36%）
    //   - 每 V_v NULL 数 2.23（接近论文 ~3）
    //   - ConBPV 6.99 → 5.45（仓库最低）
    //   - GeoBPV 19.5 → 20.4（+4.7%）/ V_base 191 → 527（fan triangulation 副作用）
    //   - TotalBPV 27.36 → 27.85（+1.8% 略恶化，但 ConBPV 论文对齐方向正确）
    //
    // env hooks：
    //   PMSF_RDPATCH_LEGACY_ISADDED=1 → 退回 BasicIsRemovable + UnifiedManifold (PR-Q 行为)
    //   都不设 → default = bypass (PR-T)
    const char* envLegacy = std::getenv("PMSF_RDPATCH_LEGACY_ISADDED");
    const bool useLegacy = envLegacy && envLegacy[0] == '1';
    if (useLegacy) {
        if (!h || h->isRemoved()) return false;
        HalfedgeErrorEntry entry(h);
        if (!basic.isRemovable(entry)) return false;
        if (!unifiedManifold.isRemovable(entry)) return false;
        return true;
    }
    return isVertexRemovableBypassIsAdded(h);
}

// PR-R (2026-05-12)：NULL_PATCH 后 push 邻居 face，bucket 由 computeRealBucket 决定
// （之前 PR-P 用固定 bucket 5 NonNull）。
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
                        int phase,
                        bool forceSixBucket = false) {
    (void)phase;
    (void)nullPatchFaces;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        MCGAL::Halfedge* fallback = nullptr;
        for (MCGAL::Halfedge* h : *f) {
            if (!isValidGate(h)) continue;
            if (!fallback) fallback = h;
            break;
        }
        MCGAL::Halfedge* best = fallback;
        if (best) pushGateIfUsable(best, q, alreadyEnqueued, facesEnqueued,
                                   nullPatchFaces, forceSixBucket);
    }
}

void resetLivingFaceProcessedFlags(std::shared_ptr<MCGAL::Mesh> mesh) {
    for (MCGAL::Facet* f : mesh->faces()) {
        if (f && !f->isRemoved()) f->setUnProcessed();
    }
}

void resetLivingVertexConqueredFlags(std::shared_ptr<MCGAL::Mesh> mesh) {
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (v && !v->isRemoved()) v->setUnConquered();
    }
}

void markGateVerticesConquered(MCGAL::Halfedge* h) {
    if (!h || h->isRemoved()) return;
    MCGAL::Halfedge* input = inputBoundaryFromGate(h);
    if (!input || input->isRemoved()) return;
    if (input->vertex() && !input->vertex()->isRemoved()) input->vertex()->setConquered();
    if (input->end_vertex() && !input->end_vertex()->isRemoved()) {
        input->end_vertex()->setConquered();
    }
}

void markPatchBoundaryVerticesConquered(const std::vector<MCGAL::Vertex*>& ringVerts) {
    for (MCGAL::Vertex* v : ringVerts) {
        if (v && !v->isRemoved()) v->setConquered();
    }
}

void applyPaperNullGateTag(MCGAL::Halfedge* h) {
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

void setPaperTagIfUnset(MCGAL::Vertex* v, int tag) {
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

void stampMepp2InteriorTags(MCGAL::Halfedge* input, int type) {
    if (!input || input->isRemoved()) return;
    MCGAL::Halfedge* g = input->next();
    for (int tag : paperMepp2InteriorTags(type)) {
        if (!g || g->isRemoved() || !g->end_vertex()) break;
        setPaperTagIfUnset(g->end_vertex(), tag);
        g = g->next();
    }
}

bool mepp2SplitFace(std::shared_ptr<MCGAL::Mesh> mesh,
                    MCGAL::Halfedge*& h,
                    MCGAL::Halfedge* g) {
    if (!mesh || !h || !g || h->isRemoved() || g->isRemoved()) return false;
    if (!h->face() || h->face()->isRemoved() || h->face() != g->face()) return false;
    MCGAL::Vertex* a = h->end_vertex();
    MCGAL::Vertex* b = g->end_vertex();
    MCGAL::Halfedge* hn = mesh->split_facet(h, g);
    if (!hn || !hn->opposite()) return false;
    hn->setAdded();
    hn->opposite()->setAdded();
    if (a && !a->isRemoved() && !a->is_degree_dirty()) a->incCachedDegree(1);
    if (b && !b->isRemoved() && !b->is_degree_dirty()) b->incCachedDegree(1);
    h = hn;
    return true;
}

bool applyMepp2Retriangulation(std::shared_ptr<MCGAL::Mesh> mesh,
                               MCGAL::Halfedge* input,
                               int valence) {
    if (!mesh || !input || input->isRemoved() || valence < 3 || valence > 6) return false;
    if (!input->face() || input->face()->isRemoved()) return false;
    const int type = mepp2TypeFromInputBoundary(input, valence);
    if (type == 0) return false;

    stampMepp2InteriorTags(input, type);
    if (valence == 3) return true;

    MCGAL::Halfedge* h = input;
    MCGAL::Halfedge* g = input->next();
    if (!g || g->isRemoved()) return false;

    switch (type) {
        case 5:
        case 8:
            h = h->find_prev();
            return mepp2SplitFace(mesh, h, g);
        case 6:
        case 7:
            g = g->next();
            return mepp2SplitFace(mesh, h, g);
        case 9:
        case 12:
            h = h->find_prev();
            if (!mepp2SplitFace(mesh, h, g)) return false;
            g = h->next() ? h->next()->next() : nullptr;
            return mepp2SplitFace(mesh, h, g);
        case 10:
            g = h;
            h = h->find_prev();
            if (!h) return false;
            h = h->find_prev();
            if (!mepp2SplitFace(mesh, h, g)) return false;
            g = h->next();
            h = h->find_prev();
            return mepp2SplitFace(mesh, h, g);
        case 11:
            g = g->next();
            if (!mepp2SplitFace(mesh, h, g)) return false;
            g = h->next() ? h->next()->next() : nullptr;
            return mepp2SplitFace(mesh, h, g);
        case 13:
        case 16:
            h = h->find_prev();
            if (!mepp2SplitFace(mesh, h, g)) return false;
            g = h->next() ? h->next()->next() : nullptr;
            if (!mepp2SplitFace(mesh, h, g)) return false;
            g = h->next() ? h->next()->next() : nullptr;
            return mepp2SplitFace(mesh, h, g);
        case 14:
        case 15:
            g = g->next();
            if (!mepp2SplitFace(mesh, h, g)) return false;
            g = h->next() ? h->next()->next() : nullptr;
            if (!mepp2SplitFace(mesh, h, g)) return false;
            g = h->next() ? h->next()->next() : nullptr;
            return mepp2SplitFace(mesh, h, g);
        default:
            return false;
    }
}

int traceOpLimit() {
    if (const char* e = std::getenv("PMSF_PCLTTM_TRACE_OPS")) {
        int n = std::atoi(e);
        if (n > 0) return n;
    }
    return 0;
}

bool shouldTraceOp(int opIndex) {
    const int lim = traceOpLimit();
    if (lim <= 0 || opIndex < 0) return false;
    int start = 0;
    if (const char* e = std::getenv("PMSF_PCLTTM_TRACE_START")) {
        start = std::max(0, std::atoi(e));
    }
    return opIndex >= start && opIndex < start + lim;
}

void traceEncoderOp(std::shared_ptr<MCGAL::Mesh> mesh,
                    int opIndex,
                    int phase,
                    int valence,
                    MCGAL::Vertex* inputA,
                    MCGAL::Vertex* inputB,
                    const std::vector<MCGAL::Vertex*>& ringVerts) {
    if (!shouldTraceOp(opIndex) || !mesh) return;
    auto qstr = [&](MCGAL::Vertex* v) {
        if (!v || v->isRemoved()) return std::string("nil");
        MCGAL::PointInt q = mesh->floatPosToInt(v->point());
        return std::to_string(q[0]) + "," + std::to_string(q[1]) + "," +
               std::to_string(q[2]);
    };
    std::fprintf(stderr,
                 "[PCLTTM trace enc] op=%d phase=%d valence=%d input=(%s)-(%s) ring=",
                 opIndex, phase, valence, qstr(inputA).c_str(), qstr(inputB).c_str());
    for (size_t i = 0; i < ringVerts.size(); ++i) {
        std::fprintf(stderr, "%s%s", i ? ";" : "", qstr(ringVerts[i]).c_str());
    }
    std::fprintf(stderr, "\n");
}

void traceEncoderGate(std::shared_ptr<MCGAL::Mesh> mesh,
                      int symbolIndex,
                      int phase,
                      const char* kind,
                      MCGAL::Halfedge* gate,
                      int valence) {
    if (!shouldTraceOp(symbolIndex) || !mesh) return;
    auto qstr = [&](MCGAL::Vertex* v) {
        if (!v || v->isRemoved()) return std::string("nil");
        MCGAL::PointInt q = mesh->floatPosToInt(v->point());
        return std::to_string(q[0]) + "," + std::to_string(q[1]) + "," +
               std::to_string(q[2]);
    };
    MCGAL::Halfedge* input = inputBoundaryFromGate(gate);
    std::fprintf(stderr,
                 "[PCLTTM trace enc gate] si=%d phase=%d kind=%s valence=%d input=(%s)-(%s)\n",
                 symbolIndex, phase, kind, valence,
                 input ? qstr(input->vertex()).c_str() : "nil",
                 input ? qstr(input->end_vertex()).c_str() : "nil");
}

void recordSymbolInputQ(std::shared_ptr<MCGAL::Mesh> mesh,
                        RDPatchPipelineResult& R,
                        MCGAL::Halfedge* gate) {
    std::array<int, 6> q{};
    MCGAL::Halfedge* input = inputBoundaryFromGate(gate);
    if (mesh && input && input->vertex() && input->end_vertex()) {
        MCGAL::PointInt a = mesh->floatPosToInt(input->vertex()->point());
        MCGAL::PointInt b = mesh->floatPosToInt(input->end_vertex()->point());
        q = {a[0], a[1], a[2], b[0], b[1], b[2]};
    }
    R.symbolInputQs.push_back(q);
}

int livingFaceRank(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Facet* target) {
    if (!mesh || !target || target->isRemoved()) return -1;
    int rank = 0;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        if (f == target) return rank;
        ++rank;
    }
    return -1;
}

int localFaceDegree(MCGAL::Facet* f) {
    return static_cast<int>(boundedFaceHalfedges(f).size());
}

bool localFaceVerticesWithin(MCGAL::Facet* f,
                             const std::set<MCGAL::Vertex*>& verts) {
    if (!f || f->isRemoved()) return false;
    for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
        if (!h || !h->vertex() || !verts.count(h->vertex())) return false;
    }
    return true;
}

std::vector<int> patchFaceRanksAfterRetriangulation(
    std::shared_ptr<MCGAL::Mesh> mesh,
    const std::vector<MCGAL::Vertex*>& ringVerts,
    int valence) {
    std::vector<int> ranks;
    if (!mesh || static_cast<int>(ringVerts.size()) != valence || valence < 3) {
        return ranks;
    }
    std::set<MCGAL::Vertex*> ringSet;
    for (MCGAL::Vertex* v : ringVerts) {
        if (!v || v->isRemoved()) return {};
        ringSet.insert(v);
    }
    if (static_cast<int>(ringSet.size()) != valence) return {};

    MCGAL::Facet* polygonFace = nullptr;
    std::vector<int> triRanks;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !localFaceVerticesWithin(f, ringSet)) continue;
        const int deg = localFaceDegree(f);
        if (deg == valence) {
            polygonFace = f;
            break;
        }
        if (deg == 3) {
            const int rank = livingFaceRank(mesh, f);
            if (rank >= 0) triRanks.push_back(rank);
        }
    }
    if (polygonFace) {
        const int rank = livingFaceRank(mesh, polygonFace);
        if (rank >= 0) {
            ranks.assign(static_cast<size_t>(std::max(1, valence - 2)), rank);
        }
        return ranks;
    }
    if (static_cast<int>(triRanks.size()) == valence - 2) {
        std::sort(triRanks.begin(), triRanks.end());
        return triRanks;
    }
    return {};
}

// 主入口：driving conquest loop。
RDPatchPipelineResult runConquest(std::shared_ptr<MCGAL::Mesh> mesh,
                                  const RDPatchPipelineConfig& cfg) {
    RDPatchPipelineResult R;
    R.mesh = mesh;

    // P5.5 诊断用：滚动保留最后 5 个符号的 (valence, encBary, ringQ)。
    struct RingDebugRec {
        int valence;
        int bx, by, bz;
        std::vector<std::array<int, 3>> ringQ;
        std::vector<std::array<int, 3>> polyQ;
    };
    std::vector<RingDebugRec> tailRings;

    // 计入 V_in。注意 mesh.vertices() 包含已 removed 顶点占位；用 living count。
    auto livingVertexCount = [](std::shared_ptr<MCGAL::Mesh> m) {
        int n = 0;
        for (MCGAL::Vertex* v : m->vertices()) {
            if (v && !v->isRemoved()) ++n;
        }
        return n;
    };
    R.numVerticesIn = livingVertexCount(mesh);
    if (R.numVerticesIn == 0) {
        std::fprintf(stderr, "[RDPatchPipeline] ERROR: empty mesh\n");
        return R;
    }
    const bool strictRD = cfg.strictRDConquest;
    const bool strictPMSFStorage = cfg.strictRDConquest || cfg.strictPCLTTMConquest;
    const bool usePMSFBoundaryGates = cfg.usePMSFBoundaryGates || cfg.strictRDConquest;
    const bool forceSixBucket = cfg.strictRDConquest;
    std::set<std::array<int, 3>> strictAllowedRemovalQs(
        cfg.strictAllowedRemovalQs.begin(), cfg.strictAllowedRemovalQs.end());
        const bool recordFaceRanks = envOn("PMSF_RDPATCH_WRITE_FACE_RANKS");

    // ---- operator init ----
    auto eliminate = std::make_shared<RDTagEliminateOperator>();
    PMSFHeader header = PMSFHeader::Builder()
                            .setRound(1)
                            .setEnableCompress(true)
                            .setEnableQuantization(true)
                            .setEnableAdjustMent(false)
                            .build();
    eliminate->init(mesh, header, /*compress_boundary=*/false);

    BasicIsRemovableOperator         basicChk;
    UnifiedManifoldIsRemovableOperator unifiedManifoldChk;

    // ---- first gate ----
    MCGAL::Halfedge* firstGate = pickLongestEdge(mesh);
    if (usePMSFBoundaryGates) firstGate = gateFromInputBoundary(firstGate);
    if (!firstGate) {
        std::fprintf(stderr, "[RDPatchPipeline] ERROR: no first_gate found\n");
        return R;
    }
    // Store the paper input boundary of the first gate. The active gate itself
    // can become an internal/removed edge after the layer's first patch
    // retriangulation; the input boundary is the stable edge the paper conquest
    // uses to recover the active gate on the decoder side.
    {
        MCGAL::Halfedge* seedInput = usePMSFBoundaryGates
                                         ? inputBoundaryFromGate(firstGate)
                                         : firstGate;
        if (!seedInput) seedInput = firstGate;
        MCGAL::PointInt q0 = mesh->floatPosToInt(seedInput->vertex()->point());
        MCGAL::PointInt q1 = mesh->floatPosToInt(seedInput->end_vertex()->point());
        R.firstGateV0Q[0] = q0[0]; R.firstGateV0Q[1] = q0[1]; R.firstGateV0Q[2] = q0[2];
        R.firstGateV1Q[0] = q1[0]; R.firstGateV1Q[1] = q1[1]; R.firstGateV1Q[2] = q1[2];
        R.layerSeedQs.push_back({q0[0], q0[1], q0[2], q1[0], q1[1], q1[2]});
    }

    RDPatchGateQueue queue;
    std::set<MCGAL::Halfedge*> enqueued;
    // facesEnqueued: per-phase 已被 push 的 face 集合（与 enqueued 配对，去重粒度
    // 提到 face 级别，避免同一 face 的 3 条 halfedge 全 push 后产生 3× 重复 step）。
    std::set<MCGAL::Facet*> facesEnqueued;
    // Phase 内 face-level 去重：每次 pop 一条 gate 后取其 face，若该 face 已在
    // 此 phase 内被处理过（emit 过 symbol），则 silently skip 不重发。论文 §2.2
    // 隐含语义：每个 patch 在每个 phase 内只处理一次。phase 切换时清空。
    std::set<MCGAL::Facet*> facesProcessedInPhase;
    // PR-R (2026-05-12)：mesh-wide null patch face set。emit NULL_PATCH 后插入；
    // 后续 push 邻居 face 时检查 face 是否在此 set → 决定 bucket 1/3/5 vs 2/4/6
    // (论文 §2.2 "null patch effects")。decoder mirror。
    std::set<MCGAL::Facet*> nullPatchFaces;

    if (usePMSFBoundaryGates) {
        resetLivingVertexConqueredFlags(mesh);
    }
    queue.push(firstGate, computeRealBucket(firstGate, nullPatchFaces, forceSixBucket));
    enqueued.insert(firstGate);
    if (firstGate->face()) facesEnqueued.insert(firstGate->face());
    if (strictPMSFStorage && firstGate->vertex() && firstGate->end_vertex()) {
        MCGAL::Halfedge* seedInput = usePMSFBoundaryGates
                                         ? inputBoundaryFromGate(firstGate)
                                         : firstGate;
        if (!seedInput) seedInput = firstGate;
        seedInput->vertex()->setTag(MCGAL::Vertex::Plus);
        seedInput->end_vertex()->setTag(MCGAL::Vertex::Minus);
        markGateVerticesConquered(firstGate);
    }
    if (cfg.strictPCLTTMConquest && !strictRD && !envOn("PMSF_PCLTTM_SINGLE_SEED")) {
        queue.clear();
        enqueued.clear();
        facesEnqueued.clear();
        resetLivingFaceProcessedFlags(mesh);
        seedAllLivingFaces(mesh, queue, enqueued, facesEnqueued, nullPatchFaces, 0,
                           forceSixBucket);
    }

    // Phase: DEC=0 / CLN=1
    int phase = 0;
    bool progressInCurrentCycle = false;
    int  cyclesCompleted = 0;

    const int maxSteps = R.numVerticesIn * cfg.maxStepsMultiplier;
    int step = 0;

    auto emit = [&](RDPatchSymbolKind kind, unsigned char escape, int bucket) {
        RDPatchSymbol s;
        s.kind = kind;
        s.escape_valence = escape;
        s.phase = static_cast<unsigned char>(phase);
        s.bucket = bucket;
        s.layer = static_cast<unsigned short>(cyclesCompleted);
        R.symbols.push_back(s);
    };

    while (step < maxSteps) {
        // Phase 切换 / 退出条件
        if (queue.empty()) {
            if (phase == 0) {
                // DEC → CLN
                phase = 1;
                                // CLN phase 开始时重新 seed first_gate（论文：每个 phase 内
                // queue 用同一 first_gate 重启；状态可继承）。简化：再 push
                // first_gate 如果它仍 valid。
                if (!isValidGate(firstGate)) {
                    firstGate = pickLongestEdge(mesh);
                    if (usePMSFBoundaryGates) firstGate = gateFromInputBoundary(firstGate);
                }
                if (isValidGate(firstGate)) {
                    enqueued.clear();
                    facesEnqueued.clear();
                    facesProcessedInPhase.clear();
                    nullPatchFaces.clear();
                    if (strictPMSFStorage) {
                        resetLivingFaceProcessedFlags(mesh);
                        if (usePMSFBoundaryGates && !strictRD) {
                            resetLivingVertexConqueredFlags(mesh);
                        }
                    }
                    if (cfg.strictPCLTTMConquest && !strictRD && !envOn("PMSF_PCLTTM_SINGLE_SEED")) {
                        seedAllLivingFaces(mesh, queue, enqueued, facesEnqueued, nullPatchFaces,
                                           phase, forceSixBucket);
                    } else {
                        queue.push(firstGate, computeRealBucket(firstGate, nullPatchFaces,
                                                               forceSixBucket));
                        enqueued.insert(firstGate);
                        if (firstGate->face()) facesEnqueued.insert(firstGate->face());
                        if (usePMSFBoundaryGates) markGateVerticesConquered(firstGate);
                    }
                }
                continue;
            } else {
                // CLN → DEC：完成一个 full cycle
                cyclesCompleted++;
                R.numDecCycles = cyclesCompleted;
                                if (!progressInCurrentCycle) break;  // base mesh 收敛
                // PR-O：限制 maxCycles。实测 cycle 2/3 全是 NULL（cycle 1 已删尽
                // 可删点；剩下的只是重走 mesh 确认）。多轮压缩走外层 multi-round。
                if (cyclesCompleted >= cfg.maxCycles) {
                                        break;
                }
                phase = 0;
                progressInCurrentCycle = false;
                if (!isValidGate(firstGate)) {
                    firstGate = pickLongestEdge(mesh);
                    if (usePMSFBoundaryGates) firstGate = gateFromInputBoundary(firstGate);
                }
                if (isValidGate(firstGate)) {
                    enqueued.clear();
                    facesEnqueued.clear();
                    facesProcessedInPhase.clear();
                    nullPatchFaces.clear();
                    if (strictPMSFStorage) {
                        resetLivingFaceProcessedFlags(mesh);
                        if (usePMSFBoundaryGates && !strictRD) {
                            resetLivingVertexConqueredFlags(mesh);
                        }
                    }
                    if (cfg.strictPCLTTMConquest && !strictRD && !envOn("PMSF_PCLTTM_SINGLE_SEED")) {
                        seedAllLivingFaces(mesh, queue, enqueued, facesEnqueued, nullPatchFaces,
                                           phase, forceSixBucket);
                    } else {
                        queue.push(firstGate, computeRealBucket(firstGate, nullPatchFaces,
                                                               forceSixBucket));
                        enqueued.insert(firstGate);
                        if (firstGate->face()) facesEnqueued.insert(firstGate->face());
                        if (usePMSFBoundaryGates) markGateVerticesConquered(firstGate);
                    }
                }
                continue;
            }
        }

        int popBucket = -1;
        // P9 阶段 0（只评估不实现）：**tie 率**测量。
        // 论文 RD.md:343 说 gate 顺序应由"同一套确定性优先级规则"从当前网格状态
        // 推出，"若存在非确定性 tie-breaking，则还需要存 tie-break 信息或 gate 顺序"。
        // 因此关键量是：**每次 pop 时，同一 bucket 里还有多少条候补**——
        // 若长期 >1，则顺序不能只靠 bucket 优先级决定，必须有确定的 tie-break 规则，
        // 否则两端无法推出同一顺序。
        // P10 阶段 0（只读探测）：**前沿集合驱动纯规则遍历的卡死率**。
        //
        // P10 的要旨是让 pop 成为"当前网格状态"的纯函数：候选集 = 前沿集合
        //   F = { gate : gate 两端顶点的 countProcessedFaces 不同 }
        // （与 P9 判决同一判据），按确定性优先级规则取一条。若 F 在某时刻为空而
        // 队列仍有待处理 gate，则该纯规则**当场卡死**（无合法着法）。
        //
        // 本探针不改遍历、不改产物：每次 pop 前数一遍"队列里有多少条落在前沿上"，
        // 记录 F=∅ 且队列非空的次数。只读 queue + 只读 mesh 状态。
        static const bool kP10Diag = [] {
            const char* e = std::getenv("PMSF_P10_FRONTIER");
            return e && e[0] == '1';
        }();
        if (kP10Diag) {
            std::size_t inQueue = 0, onFrontier = 0;
            queue.forEachEntry([&](MCGAL::Halfedge* qh) {
                if (!qh || qh->isRemoved()) return false;
                ++inQueue;
                if (isOnFrontier(qh)) ++onFrontier;
                return false;
            });
            ++g_p10Pops;
            g_p10QueueSum += static_cast<long long>(inQueue);
            g_p10FrontierSum += static_cast<long long>(onFrontier);
            if (inQueue > 0) {
                ++g_p10BusyPops;
                if (onFrontier == 0 && step > 0) {
                    // **关键判别**：上一步数的是"前沿 ∩ 队列"。纯规则的候选集是
                    // **整个前沿**（可从网格状态独立重建），可能包含从未入队的 gate。
                    // 故此处对全网格做一次穷举扫描：若前沿上仍有 gate，则纯规则
                    // 当场**有合法着法**（不是卡死）；只有全网格前沿为空才是真卡死。
                    // 只在"疑似卡死"点扫描（每轮 ≤2 次），代价可忽略。
                    // 两个计数：
                    //   meshFrontier  = 前沿 gate 总数（不限可用性）
                    //   meshUsable    = 前沿 ∧ **可用**（face 未处理）—— 与
                    //                   pushGateIfUsable 的 `if (f->isProcessed())
                    //                   return;` 同判据。**这才是纯规则真正的合法
                    //                   着法集合**：只数总数会把"不可用的前沿 gate"
                    //                   误算成可用，从而假报不卡死。
                    long long meshFrontier = 0, meshUsable = 0;
                    for (MCGAL::Facet* mf : mesh->faces()) {
                        if (!mf || mf->isRemoved()) continue;
                        for (MCGAL::Halfedge* mh : *mf) {
                            if (!mh || mh->isRemoved()) continue;
                            if (!isOnFrontier(mh)) continue;
                            ++meshFrontier;
                            MCGAL::Facet* uf = mh->face();
                            // 可用判据**必须**与主循环 dedup 同源（facesProcessedInPhase），
                            // 不能用 f->isProcessed() —— 后者会把绝大多数条目误判为不可用
                            // （实测 0.1%），从而把"前沿其实有活干"误报成"真卡死"。
                            if (uf && !uf->isRemoved() &&
                                !facesProcessedInPhase.count(uf))
                                ++meshUsable;
                        }
                    }
                    g_p10MeshFrontierAtStall += meshFrontier;
                    g_p10MeshUsableAtStall += meshUsable;
                    if (meshUsable > 0) {
                        ++g_p10FalseStalls;   // 队列假象：前沿另有**可用** gate
                    } else {
                        ++g_p10TrueStalls;    // 前沿无可用 gate ⇒ 真卡死
                        // **逐事件留证**：合计值会把"假卡死点"的计数混进来，
                        // 无法回答"真卡死点上前沿到底是什么样"。故单独记。
                        if (g_p10TrueStallCount < 16) {
                            g_p10TrueStallStep[g_p10TrueStallCount] = step;
                            g_p10TrueStallFrontier[g_p10TrueStallCount] = meshFrontier;
                            g_p10TrueStallUsable[g_p10TrueStallCount] = meshUsable;
                            g_p10TrueStallQueue[g_p10TrueStallCount] = (long long)inQueue;
                            ++g_p10TrueStallCount;
                        }
                    }
                }
                if (onFrontier == 0) {
                    ++g_p10Stalls;
                    if (g_p10FirstStallStep < 0) g_p10FirstStallStep = step;
                    if (g_p10StallStepCount < 16) {
                        g_p10StallSteps[g_p10StallStepCount++] = step;
                    }
                }
            }
        }
        // P10 阶段 0 续：**前沿驱动纯规则**（env 打开时才改变遍历）。
        // 默认关闭 ⇒ 走原来的 popHighestPriority，零行为变化。
        static const bool kP10Rule = [] {
            const char* e = std::getenv("PMSF_P10_RULE");
            return e && e[0] == '1';
        }();
        MCGAL::Halfedge* h = nullptr;
        if (kP10Rule) {
            MCGAL::Halfedge* pick = p10PickFrontierGate(
                mesh.get(), queue, nullPatchFaces, forceSixBucket, facesProcessedInPhase,
                &g_p10RuleQueueSum, &g_p10RuleUsableSum, &g_p10RuleUsableFrontSum);
            if (pick != nullptr) {
                // 按 bucket 取出时才知道它属于哪个 bucket —— 用队列自身移除保持其余次序
                queue.removeEntry(pick);
                popBucket = computeRealBucket(pick, nullPatchFaces, forceSixBucket);
                h = pick;
            } else {
                // 前沿无可用 gate：回落到 push 历史队首，并计数（= 需要例外通道的量）
                ++g_p10RuleFallbacks;
                if (phase == 0) ++g_p10RuleFallbacksDec; else ++g_p10RuleFallbacksCln;
                if (step > 0 && g_p10RuleFallbackSteps < 16) {
                    g_p10RuleFallbackAt[g_p10RuleFallbackSteps++] = step;
                }
                h = queue.popHighestPriority(&popBucket);
            }
            ++g_p10RulePops;
        } else {
            h = queue.popHighestPriority(&popBucket);
        }
                if (!h || h->isRemoved()) continue;

        // Face-level dedup（per-phase）：每个 patch 在一个 phase 内只 emit 一次 symbol。
        // dedup 命中的 pop 不计 step（避免 max_steps 兜底误中断）。
        MCGAL::Facet* gateFace = h->face();
        if (gateFace && facesProcessedInPhase.count(gateFace)) {
            continue;
        }

        if (popBucket >= 1 && popBucket <= 6) {
            R.bucketHits[popBucket - 1]++;
        }
        ++step;
        R.numSteps = step;

        if (cfg.progressLogEvery > 0 && step % cfg.progressLogEvery == 0) {
            std::fprintf(stderr,
                         "[RDPatchPipeline] step=%d phase=%s queue=%zu "
                         "decimated=%d null=%d\n",
                         step, (phase == 0 ? "DEC" : "CLN"),
                         queue.size(), R.numDecimatedTotal,
                         R.numNullPatches);
        }

        MCGAL::Vertex* frontV = h->end_vertex();
        if (!frontV || frontV->isRemoved() || frontV->isCollapsed()) {
            continue;
        }
        const int valence = frontV->vertex_degree();
        bool strictAllowedRemoval = true;
        if (strictRD && cfg.strictUseAllowedRemovalFilter) {
            MCGAL::PointInt q = mesh->floatPosToInt(frontV->point());
            strictAllowedRemoval =
                strictAllowedRemovalQs.count({q[0], q[1], q[2]}) != 0;
        }

        bool removable = false;
        const bool frontVertexFree = !usePMSFBoundaryGates || !frontV->isConquered();
        if (phase == 0 && frontVertexFree) {
            // PR-Q (2026-05-12) 探索：论文 §2.2 DEC 阶段只删 valence 4..6（不删 3）。
            //   理由：valence 3 顶点留给 CLN 集中清理，长程上可能多删点。
            //   **实测 multi-round bench (bunny, target_V=50)**：
            //     DEC v=3..6 (PR-P 旧): rounds=60, V_base=191, TotalBPV=27.36
            //     DEC v=4..6 (PR-Q):    rounds=75, V_base=278, TotalBPV=27.71 (恶化)
            //   论文 PGS 比 A&D 长程多 5%（仅在 toy mesh 上），但本仓库 conquest
            //   实装的 NULL 占比远高于论文，DEC v=4..6 的 trade-off 反向：
            //   单轮少删 → 多 round → container overhead 累计反向恶化。
            //   **decision**：默认保留 v=3..6（PR-P 行为）；env PMSF_RDPATCH_DEC_V4_6=1
            //   切换论文 v=4..6 模式（用于 ablation 对比）。
            const char* envV4_6 = std::getenv("PMSF_RDPATCH_DEC_V4_6");
            const int decMin = (strictRD || (envV4_6 && envV4_6[0] == '1')) ? 4 : 3;
            const bool allowProcessedPatchDelete =
                envOn("PMSF_RDSTRICT_ALLOW_PROCESSED_PATCH_DELETE");
            int decMax = 6;
            if (strictRD) {
                const char* decMaxEnv = std::getenv("PMSF_RDSTRICT_DEC_MAX_VALENCE");
                if (decMaxEnv && decMaxEnv[0] != '\0') {
                    decMax = std::clamp(std::atoi(decMaxEnv), decMin, 6);
                }
            }
            if (valence >= decMin && valence <= decMax &&
                strictAllowedRemoval &&
                (!strictRD || allowProcessedPatchDelete ||
                 !frontPatchTouchesProcessedFace(frontV))) {
                removable = isVertexRemovable(h, basicChk, unifiedManifoldChk);
            }
        } else if (frontVertexFree) {
            // CLN: valence == 3 且 isRemovable（不变）
            const bool allowProcessedPatchDelete =
                envOn("PMSF_RDSTRICT_ALLOW_PROCESSED_PATCH_DELETE");
            if (valence == 3 &&
                strictAllowedRemoval &&
                (!strictRD || allowProcessedPatchDelete ||
                 !frontPatchTouchesProcessedFace(frontV))) {
                removable = isVertexRemovable(h, basicChk, unifiedManifoldChk);
            }
        }

        if (removable) {
            // emit V_v + erase + push
            const RDPatchSymbolKind kind = valenceToKind(valence);
            const unsigned char escape =
                (kind == RDPatchSymbolKind::V_OTHER)
                    ? static_cast<unsigned char>(std::min(valence, 255))
                    : 0;

            // 在 erase 之前抓 1-ring 作 push 候选源
            RingSnapshot snap = snapshotRing(frontV);
            const int symbolIndexBeforeRemoval = static_cast<int>(R.symbols.size());
            traceEncoderGate(mesh, symbolIndexBeforeRemoval, phase, "V", h, valence);
            recordSymbolInputQ(mesh,R,h);
            MCGAL::Vertex* inputBoundaryA = nullptr;
            MCGAL::Vertex* inputBoundaryB = nullptr;
            MCGAL::Vertex* inputGateThird = nullptr;
            MCGAL::Halfedge* inputBoundaryH = nullptr;
            if (MCGAL::Halfedge* inputBoundary = inputBoundaryFromGate(h)) {
                inputBoundaryH = inputBoundary;
                inputBoundaryA = inputBoundary->vertex();
                inputBoundaryB = inputBoundary->end_vertex();
                if (gateFace) {
                    for (MCGAL::Halfedge* gh : *gateFace) {
                        MCGAL::Vertex* gv = gh ? gh->vertex() : nullptr;
                        if (gv && gv != inputBoundaryA && gv != inputBoundaryB) {
                            inputGateThird = gv;
                            break;
                        }
                    }
                }
            }

            // PR-V curvature + PR-U paper retriangulation
            // Step 1: snapshot ring + vQuant BEFORE removal
            MCGAL::PointInt vQuant = mesh->floatPosToInt(frontV->point());

            const char* fanEnv=std::getenv("PMSF_RDPATCH_FAN_RETRI");
            bool useFan=fanEnv&&fanEnv[0]=='1';
            int emitX=0, emitY=0, emitZ=0;
            MCGAL::Facet* decodeFaceForSideInfo = gateFace;
            MCGAL::Facet* polyFaceForDebug = nullptr;
            unsigned char inputOffset = 0;

            if(useFan){
                // Fan path: original flow (residual from triangle face)
                MCGAL::PointInt baryQuant = MCGAL::rdBarycenterQuantized(gateFace, mesh);
                emitX=vQuant[0]-baryQuant[0]; emitY=vQuant[1]-baryQuant[1]; emitZ=vQuant[2]-baryQuant[2];
                HalfedgeErrorEntry entry(h,h->end_vertex()->point(),0.0f,0);
                if(!eliminate->eliminate(entry)){emit(RDPatchSymbolKind::NULL_PATCH,0,popBucket);R.numNullPatches++;if(gateFace)facesProcessedInPhase.insert(gateFace);continue;}
                decodeFaceForSideInfo = entry.edge ? entry.edge->face() : gateFace;
                int N=(int)snap.ringVerts.size();if(N>=3){pclttm::Tag tL=static_cast<pclttm::Tag>(snap.ringVerts[0]->getTag()),tR=static_cast<pclttm::Tag>(snap.ringVerts[N-1]->getTag());std::vector<pclttm::TagEvolutionRecord> tr;pclttm::applyTagEvolution(snap.ringVerts,tL,tR,pclttm::AnchorKind::V0First,[](MCGAL::Vertex* v){return static_cast<pclttm::Tag>(v->getTag());},[](MCGAL::Vertex* v,pclttm::Tag t){v->setTag(static_cast<MCGAL::Vertex::TagFlag>(t));},[](MCGAL::Vertex* v)->int{return v?v->poolId():-1;},tr);}
                if(gateFace)facesProcessedInPhase.insert(gateFace);if(entry.edge&&entry.edge->face())facesProcessedInPhase.insert(entry.edge->face());
            }else{
                // Paper path: remove_point first, THEN compute residual from polygon
                // This ensures encoder & decoder use the SAME polygon barycenter.
                if(!eliminate->remove_point(h)){emit(RDPatchSymbolKind::NULL_PATCH,0,popBucket);R.numNullPatches++;if(gateFace)facesProcessedInPhase.insert(gateFace);continue;}
                MCGAL::Facet* polyFace=h->face();
                polyFaceForDebug = polyFace;
                if(polyFace){
                    if (strictPMSFStorage && inputBoundaryH &&
                        !inputBoundaryH->isRemoved() &&
                        inputBoundaryH->face() == polyFace) {
                        polyFace->proxyHalfedge(inputBoundaryH);
                    }
                    if(!strictPMSFStorage){
                        polyFace->normalize_proxy_to_min_position_vertex(mesh->bboxMin,mesh->f_quantStep);
                    }
                }

                // Collect polygon boundary vertices (clockwise order = same as decoder)
                std::vector<MCGAL::Vertex*> pv;if(polyFace){auto* px=polyFace->proxyHalfedge();if(px){auto* c=px;do{pv.push_back(c->vertex());c=c->next();}while(c!=px);}}
                int V=(int)pv.size();
                if(V>0&&inputBoundaryA&&inputBoundaryB){for(int oi=0;oi<V;++oi){auto* a=pv[(size_t)oi];auto* b=pv[(size_t)((oi+1)%V)];if(a==inputBoundaryA&&b==inputBoundaryB){inputOffset=(unsigned char)oi;break;}if(a==inputBoundaryB&&b==inputBoundaryA){inputOffset=(unsigned char)(oi|0x8);break;}}}
                if(V>0&&inputGateThird){
                    const int oi=static_cast<int>(inputOffset&0x7);
                    unsigned char side=0;
                    if(oi>=0&&oi<V){
                        MCGAL::Vertex* after=pv[(size_t)((oi+2)%V)];
                        MCGAL::Vertex* before=pv[(size_t)((oi+V-1)%V)];
                        if(inputGateThird==after)side=1;
                        else if(inputGateThird==before)side=2;
                        else side=3;
                    }
                    inputOffset=static_cast<unsigned char>(inputOffset|(side<<4));
                }

                // Compute residual from POLYGON barycenter (not triangle!)
                MCGAL::PointInt baryQuant = MCGAL::rdBarycenterQuantized(polyFace, mesh);
                int r1x=vQuant[0]-baryQuant[0], r1y=vQuant[1]-baryQuant[1], r1z=vQuant[2]-baryQuant[2];
                emitX=r1x;emitY=r1y;emitZ=r1z;

                if(cfg.useFrenetResiduals){
                    MCGAL::Vector3 normal = polygonNormal(pv);
                    if(normal.length()!=0 && inputBoundaryA && inputBoundaryB){
                        MCGAL::Vector3 t1,t2;
                        MCGAL::determineFrenetFrame(MCGAL::Vector3(inputBoundaryA->point()),
                                                     MCGAL::Vector3(inputBoundaryB->point()),
                                                     normal,t1,t2);
	                        if(t1.length()!=0 && t2.length()!=0){
	                            const bool directFrenet = envOn("PMSF_PCLTTM_DIRECT_FRENET");
	                            if(directFrenet){
	                                MCGAL::Vector3 residualVec(r1x,r1y,r1z);
	                                emitX=(int)std::lround(residualVec | t1);
	                                emitY=(int)std::lround(residualVec | t2);
	                                emitZ=(int)std::lround(residualVec | normal);
	                            }else{
	                                MCGAL::Vector3i residual(r1x,r1y,r1z);
	                                MCGAL::Vector3i fr=MCGAL::frenetRotation(residual,t1,t2,normal);
	                                emitX=fr[0];emitY=fr[1];emitZ=fr[2];
	                            }
	                        }
                    }
                }

                // Curvature prediction using polygon boundary vertices
                const char* noCurv = std::getenv("PMSF_RDPATCH_NO_CURVATURE");
                if(!cfg.useFrenetResiduals && (!noCurv || noCurv[0]!='1')){
                    std::vector<MCGAL::PointInt*> nr; nr.reserve(pv.size());
                    for(auto* rv:pv){
                        if(rv&&!rv->isRemoved()){auto& s=MCGAL::contextPool.property<MCGAL::PointInt>(DEFAULT_MESH_ID,MCGAL::kVPropRDR1,rv);nr.push_back(&s);}
                        else nr.push_back(nullptr);
                    }
                    int la=1, lb=1, lg=1;
                    parseCurvatureLambdaEnv(la, lb, lg);
                    auto rc=curvatureEncodeFixed(r1x,r1y,r1z,pv,nr,la,lb,lg);
                    emitX=rc[0];emitY=rc[1];emitZ=rc[2];
                    R.leakyFactors=packLeakyFactors(la,lb,lg);
                }
                // Store raw first-order residual on all active polygon neighbors.
                // This mirrors PCLTTMTag/RDTag's Phase 3 curvature timing:
                // later patches predict from already-conquered neighboring
                // residuals, while removed center vertices are not queried.
                for(auto* rv:pv){
                    if(!rv||rv->isRemoved())continue;
                    auto& s=MCGAL::contextPool.property<MCGAL::PointInt>(
                        DEFAULT_MESH_ID,MCGAL::kVPropRDR1,rv);
                    s=MCGAL::PointInt(r1x,r1y,r1z);
                }

                std::vector<MCGAL::Halfedge*> preSplitPaperDecimatingGates;
                if (strictRD && phase == 0) {
                    preSplitPaperDecimatingGates =
                        collectPaperDecimatingGatesFromPolygon(polyFace,
                                                               inputBoundaryA,
                                                               inputBoundaryB);
                }

                // Paper triangulation (after residual computation)
                RetriangulationResult paperR;
                bool usedMepp2Retriangulation = false;
                if (strictPMSFStorage && polyFace && inputBoundaryH &&
                    !inputBoundaryH->isRemoved() && inputBoundaryH->face() == polyFace) {
                    usedMepp2Retriangulation =
                        applyMepp2Retriangulation(mesh, inputBoundaryH, V);
                    decodeFaceForSideInfo = inputBoundaryH->face();
                }
                if (!strictPMSFStorage) {
                    paperR = paperRetriangulate(V);
                } else if (!usedMepp2Retriangulation) {
                    requirePaperFig9ExactTableIfRequested();
                    paperR = paperRetriangulateFig9(
                        V,
                        pv.empty() ? static_cast<int>(MCGAL::Vertex::Plus)
                                   : static_cast<int>(pv.front()->getTag()),
                        pv.empty() ? static_cast<int>(MCGAL::Vertex::Minus)
                                   : static_cast<int>(pv.back()->getTag()));
                }
                if(polyFace&&!usedMepp2Retriangulation&&!paperR.diagonals.empty()){
                    auto fH=[&](MCGAL::Facet* f,MCGAL::Vertex* v)->MCGAL::Halfedge*{if(!f||!v)return nullptr;auto* s=f->proxyHalfedge();if(!s)return nullptr;auto* it=s;do{if(it->end_vertex()==v)return it;it=it->next();}while(it!=s);return nullptr;};
                    auto cV=[](MCGAL::Facet* f)->int{if(!f)return 0;auto* s=f->proxyHalfedge();if(!s)return 0;int n=0;auto* it=s;do{n++;it=it->next();}while(it!=s);return n;};
                    MCGAL::Facet* rem=polyFace;
                    for(auto& d:paperR.diagonals){if(d[0]>=V||d[1]>=V)continue;auto* hA=fH(rem,pv[d[0]]);auto* hB=fH(rem,pv[d[1]]);if(!hA||!hB)continue;auto* hn=mesh->split_facet(hA,hB);hn->setAdded();hn->opposite()->setAdded();pv[d[0]]->incCachedDegree(1);pv[d[1]]->incCachedDegree(1);auto* fA=hA->face();auto* fB=hn->opposite()->face();rem=(cV(fA)>3)?fA:fB;}
                    decodeFaceForSideInfo = rem;
                    if (inputBoundaryA && inputBoundaryB) {
                        bool foundInputFace = false;
                        for (MCGAL::Facet* ff : mesh->faces()) {
                            if (!ff || ff->isRemoved() || !ff->proxyHalfedge()) continue;
                            MCGAL::Halfedge* start = ff->proxyHalfedge();
                            MCGAL::Halfedge* he = start;
                            int guard = 0;
                            do {
                                if (he && !he->isRemoved() &&
                                    he->vertex() == inputBoundaryA &&
                                    he->end_vertex() == inputBoundaryB &&
                                    he->face() && !he->face()->isRemoved()) {
                                    decodeFaceForSideInfo = he->face();
                                    foundInputFace = true;
                                    break;
                                }
                                he = he ? he->next() : nullptr;
                            } while (he && he != start && ++guard < 32);
                            if (foundInputFace) {
                                break;
                            }
                        }
                    }
                } else {
                    decodeFaceForSideInfo = polyFace;
                }
                if (!usedMepp2Retriangulation) {
                    for(int idx:paperR.plusTagIdx)if(idx>=0&&idx<V&&pv[idx]&&pv[idx]->getTag()==MCGAL::Vertex::Untagged)pv[idx]->setTag(MCGAL::Vertex::Plus);
                    for(int idx:paperR.minusTagIdx)if(idx>=0&&idx<V&&pv[idx]&&pv[idx]->getTag()==MCGAL::Vertex::Untagged)pv[idx]->setTag(MCGAL::Vertex::Minus);
                }
                if(gateFace)facesProcessedInPhase.insert(gateFace);if(polyFace&&polyFace->proxyHalfedge())facesProcessedInPhase.insert(polyFace->proxyHalfedge()->face());

                // Default side information follows the decoder-visible polygon
                // after removal. The opt-in pre-removal path is a topology
                // diagnostic for exact original one-ring replay.
                if (!envOn("PMSF_RDPATCH_STORE_PRE_REMOVAL_RING")) {
                    snap.ringVerts = std::move(pv);
                }
                if (strictRD && phase == 0) {
                    for (MCGAL::Halfedge* gate : preSplitPaperDecimatingGates) {
                        pushGateIfUsable(gate, queue, enqueued, facesEnqueued,
                                         nullPatchFaces, forceSixBucket);
                    }
                }
            }
            emit(kind,escape,popBucket);R.numDecimatedTotal++;R.vvResidualsX.push_back(emitX);R.vvResidualsY.push_back(emitY);R.vvResidualsZ.push_back(emitZ);R.vvAbsQX.push_back(vQuant[0]);R.vvAbsQY.push_back(vQuant[1]);R.vvAbsQZ.push_back(vQuant[2]);
            {
                std::array<int, 9> faceQ{};
                if (decodeFaceForSideInfo && !decodeFaceForSideInfo->isRemoved()) {
                    MCGAL::Halfedge* scan = decodeFaceForSideInfo->proxyHalfedge();
                    for (int k = 0; k < 3 && scan && !scan->isRemoved(); ++k) {
                        MCGAL::Vertex* vv = scan->vertex();
                        if (vv && !vv->isRemoved()) {
                            MCGAL::PointInt q = mesh->floatPosToInt(vv->point());
                            faceQ[static_cast<std::size_t>(k * 3 + 0)] = q[0];
                            faceQ[static_cast<std::size_t>(k * 3 + 1)] = q[1];
                            faceQ[static_cast<std::size_t>(k * 3 + 2)] = q[2];
                        }
                        scan = scan->next();
                    }
                }
                R.vvFaceQ.push_back(faceQ);
                unsigned char ringN = 0;
                if (decodeFaceForSideInfo && !decodeFaceForSideInfo->isRemoved()) {
                    MCGAL::Halfedge* scan2 = decodeFaceForSideInfo->proxyHalfedge();
                    MCGAL::Halfedge* begin2 = scan2;
                    do {
                        if (!scan2 || scan2->isRemoved()) break;
                        MCGAL::Vertex* rv = scan2->vertex();
                        if (rv && !rv->isRemoved()) {
                            MCGAL::PointInt q = mesh->floatPosToInt(rv->point());
                            R.vvFaceRingQX.push_back(q[0]);
                            R.vvFaceRingQY.push_back(q[1]);
                            R.vvFaceRingQZ.push_back(q[2]);
                            ++ringN;
                        }
                        scan2 = scan2->next();
                    } while (scan2 && scan2 != begin2 && ringN < 255);
                }
                R.vvFaceRingSizes.push_back(ringN);
            }
            R.vvRemovedVertexIds.push_back(frontV?frontV->poolId():-1);
            R.vvInputOffsets.push_back(inputOffset);
            traceEncoderOp(mesh, R.numDecimatedTotal - 1, phase, valence,
                           inputBoundaryA, inputBoundaryB, snap.ringVerts);
            size_t storedRingVerts = 0;
            for(MCGAL::Vertex* rv:snap.ringVerts){
                if(!rv)continue;
                MCGAL::PointInt qv=mesh->floatPosToInt(rv->point());
                R.vvRingQX.push_back(qv[0]);R.vvRingQY.push_back(qv[1]);R.vvRingQZ.push_back(qv[2]);
                R.vvRingVertexIds.push_back(rv->poolId());
                ++storedRingVerts;
            }
            // P5.5 诊断（门控，默认关）：把编码端**实际用来算残差的那个多边形**
            // 连同其重心打出来，供与解码端 gateFace 的环逐项对拍——
            // 判定两者的**顶点集合**是否相同（重心只取决于集合，与朝向无关）。
            if (envOn("PMSF_RDPATCH_RING_DBG") && R.numDecimatedTotal <= 5) {
                std::fprintf(stderr, "  [encRing] sym=%d val=%d encBary=(%d,%d,%d) ring=",
                             R.numDecimatedTotal - 1, valence,
                             vQuant[0] - emitX, vQuant[1] - emitY, vQuant[2] - emitZ);
                for (MCGAL::Vertex* rv : snap.ringVerts) {
                    if (!rv) continue;
                    MCGAL::PointInt qv = mesh->floatPosToInt(rv->point());
                    std::fprintf(stderr, "(%d,%d,%d)", qv[0], qv[1], qv[2]);
                }
                std::fprintf(stderr, " | polyFace=");
                if (polyFaceForDebug && !polyFaceForDebug->isRemoved()) {
                    for (MCGAL::Halfedge* ph : *polyFaceForDebug) {
                        MCGAL::Vertex* pvv = ph ? ph->vertex() : nullptr;
                        if (!pvv || pvv->isRemoved()) continue;
                        MCGAL::PointInt qv = mesh->floatPosToInt(pvv->point());
                        std::fprintf(stderr, "(%d,%d,%d)", qv[0], qv[1], qv[2]);
                    }
                }
                std::fprintf(stderr, "\n");
            }
            // P5.5 逆向重放对拍用：滚动保留**最后** 5 个符号的环。
            // 逆向重放下解码端第 k 步对应编码端第 N-1-k 步，要对的是编码端
            // 的**尾部**而不是头部；正向的 [encRing] 打的是头部。
            if (envOn("PMSF_RDPATCH_RING_DBG")) {
                // ★ 用 **polyFace**（编码端真正用来算残差/重心的那个面）的顶点集，
                // 不是 snap.ringVerts —— 二者若不相等，之前几轮探针测的都是错的
                // 对象。这里同时记录两者以便直接对拍。
                std::vector<std::array<int, 3>> ringQ;
                for (MCGAL::Vertex* rv : snap.ringVerts) {
                    if (!rv) continue;
                    MCGAL::PointInt qv = mesh->floatPosToInt(rv->point());
                    ringQ.push_back({qv[0], qv[1], qv[2]});
                }
                std::vector<std::array<int, 3>> polyQ;
                if (polyFaceForDebug && !polyFaceForDebug->isRemoved()) {
                    for (MCGAL::Halfedge* ph : *polyFaceForDebug) {
                        MCGAL::Vertex* pvv = ph ? ph->vertex() : nullptr;
                        if (!pvv || pvv->isRemoved()) continue;
                        MCGAL::PointInt qv = mesh->floatPosToInt(pvv->point());
                        polyQ.push_back({qv[0], qv[1], qv[2]});
                    }
                }
                tailRings.push_back({valence, vQuant[0] - emitX, vQuant[1] - emitY,
                                     vQuant[2] - emitZ, ringQ, polyQ});
                if (tailRings.size() > 5) tailRings.erase(tailRings.begin());
            }
            R.vvRingSizes.push_back(static_cast<unsigned char>(std::min<size_t>(storedRingVerts, 255)));
            if(recordFaceRanks){
                const bool groupRanks = envOn("PMSF_RDPATCH_GROUP_FACE_RANKS");
                if(groupRanks){
                    std::vector<int> patchRanks =
                        patchFaceRanksAfterRetriangulation(mesh, snap.ringVerts, valence);
                    if(!patchRanks.empty()){
                        R.vvDecodeFaceRanks.insert(R.vvDecodeFaceRanks.end(),
                                                   patchRanks.begin(),
                                                   patchRanks.end());
                    }else{
                        R.vvDecodeFaceRanks.push_back(livingFaceRank(mesh,decodeFaceForSideInfo));
                    }
                }else{
                    R.vvDecodeFaceRanks.push_back(livingFaceRank(mesh,decodeFaceForSideInfo));
                }
            }progressInCurrentCycle=true;
            if (cfg.maxRemovalsPerRun > 0 &&
                R.numDecimatedTotal >= cfg.maxRemovalsPerRun) {
                                break;
            }
            if(usePMSFBoundaryGates){
                markPatchBoundaryVerticesConquered(snap.ringVerts);
                markPatchInteriorFacesProcessed(snap.ringVerts, facesProcessedInPhase);
            }
            if(!usePMSFBoundaryGates){
                for(MCGAL::Vertex* rv:snap.ringVerts){if(!rv||rv->isRemoved())continue;for(MCGAL::Halfedge* he:*rv){if(!he||he->isRemoved())continue;MCGAL::Facet* f=he->face();if(f&&!f->isRemoved())f->setProcessedFlag();}}
            }
            if(usePMSFBoundaryGates){
                if(phase==0){
                    if (!strictRD) {
                        pushPaperDecimatingGates(mesh,snap.ringVerts,inputBoundaryA,inputBoundaryB,
                                                 queue,enqueued,facesEnqueued,nullPatchFaces,
                                                 forceSixBucket);
                    }
                }else{
                    pushPaperCleaningGates(snap.ringVerts,queue,enqueued,facesEnqueued,
                                           facesProcessedInPhase,nullPatchFaces,
                                           forceSixBucket);
                }
            }else{
                pushNewGates(snap,queue,enqueued,facesEnqueued,nullPatchFaces,forceSixBucket);
            }
        } else {
            // Strict PCLTTM follows the paper: decimating emits {3..6,N}, and
            // cleaning emits {3,N}. Legacy RD_PATCH keeps CLN NULL silent for
            // backward-compatible BPV experiments.
            if (phase == 0 || strictPMSFStorage) {
                traceEncoderGate(mesh, static_cast<int>(R.symbols.size()), phase, "N", h, valence);
                recordSymbolInputQ(mesh,R,h);
                emit(RDPatchSymbolKind::NULL_PATCH, 0, popBucket);
                R.numNullPatches++;
            }
            if (gateFace) facesProcessedInPhase.insert(gateFace);
            // PR-R (2026-05-12)：标记 face 为 null patch + face::isProcessed()。
            // null patch face 加入 nullPatchFaces set 让后续 push 时能识别为 null。
            if (gateFace) {
                gateFace->setProcessedFlag();
                if (phase == 0 || strictPMSFStorage) nullPatchFaces.insert(gateFace);
            }
            if (strictPMSFStorage) {
                applyPaperNullGateTag(h);
            }
            // PR-Q (2026-05-12)：DEC + CLN phase 都 push 邻居（双端 mirror，固定行为）。
            //   - DEC: 必须 push（防死锁）
            //   - CLN: 也 push，让 conquest reach 整个 mesh（找 valence-3 顶点）。
            //     不能用 env 切换：encoder/decoder 必须严格 mirror，否则 round-trip 崩。
            //   实测 multi-round bench (bunny): TotalBPV 几乎无差（CLN push 主要
            //   增加少量 container overhead，但删点效率无大变化）。
            if (usePMSFBoundaryGates) {
                pushPaperNullPatchNeighbors(h, queue, enqueued, facesEnqueued, nullPatchFaces,
                                            forceSixBucket);
            } else {
                pushNullPatchNeighbors(h, queue, enqueued, facesEnqueued, nullPatchFaces,
                                       forceSixBucket);
            }
        }
    }

    if (step >= maxSteps) {
        R.reachedMaxSteps = true;
        std::fprintf(stderr,
                     "[RDPatchPipeline] WARNING: reached max_steps=%d, "
                     "abort（mesh V_in=%d, decimated=%d）\n",
                     maxSteps, R.numVerticesIn, R.numDecimatedTotal);
    }

    R.numVerticesOut = livingVertexCount(mesh);

    
    if (g_p10RulePops > 0) {
        // **在构造出的真实序列上**的卡死/回落测量。这是阶段 0 声明的头号风险
        // （此前测的是既有 push 路径上的前沿行为），本行才是规则自己走出来的序列。
        std::fprintf(stderr,
                     "  [P10.R] 规则模式 pop=%lld  前沿无可用 gate 而回落=%lld (%.4f%%)  "
                     "样本(前16)=",
                     g_p10RulePops, g_p10RuleFallbacks,
                     g_p10RulePops ? 100.0 * static_cast<double>(g_p10RuleFallbacks) /
                                         static_cast<double>(g_p10RulePops)
                                   : 0.0);
        for (int i = 0; i < g_p10RuleFallbackSteps; ++i) {
            std::fprintf(stderr, "%lld ", g_p10RuleFallbackAt[i]);
        }
        std::fprintf(stderr, "\n");
        std::fprintf(stderr,
                     "  [P10.Rc] 回落按相位拆分：DEC=%lld  CLN=%lld\n",
                     g_p10RuleFallbacksDec, g_p10RuleFallbacksCln);
        std::fprintf(stderr,
                     "  [P10.Rb] 队列构成（累计/每次 pop 均）：队列=%.2f  其中**可用**=%.2f (%.1f%%)  "
                     "其中**既可用又在前沿**=%.2f (%.2f%% of 队列)\n",
                     g_p10RulePops ? static_cast<double>(g_p10RuleQueueSum) /
                                         static_cast<double>(g_p10RulePops)
                                   : 0.0,
                     g_p10RulePops ? static_cast<double>(g_p10RuleUsableSum) /
                                         static_cast<double>(g_p10RulePops)
                                   : 0.0,
                     g_p10RuleQueueSum ? 100.0 * static_cast<double>(g_p10RuleUsableSum) /
                                             static_cast<double>(g_p10RuleQueueSum)
                                       : 0.0,
                     g_p10RulePops ? static_cast<double>(g_p10RuleUsableFrontSum) /
                                         static_cast<double>(g_p10RulePops)
                                   : 0.0,
                     g_p10RuleQueueSum ? 100.0 * static_cast<double>(g_p10RuleUsableFrontSum) /
                                             static_cast<double>(g_p10RuleQueueSum)
                                       : 0.0);
        g_p10RulePops = g_p10RuleFallbacks = 0;
        g_p10RuleFallbackSteps = 0;
        g_p10RuleQueueSum = g_p10RuleUsableSum = g_p10RuleUsableFrontSum = 0;
        g_p10RuleFallbacksDec = g_p10RuleFallbacksCln = 0;
    }

    if (g_p10Pops > 0) {
        // P10 阶段 0 判据：**卡死率**。若 g_p10Stalls > 0，说明存在这样的时刻——
        // 队列里还有待处理 gate，但**没有一条落在前沿上** ⇒ "只从前沿取 gate"
        // 的纯规则在该时刻无合法着法（当场卡死）。这是 P10 可行性的第一道门。
        std::fprintf(stderr,
                     "  [P10.0] pop=%lld  队列非空次数=%lld  前沿为空且队列非空=**%lld** (%.4f%%)  "
                     "首次卡死 step=%lld\n",
                     g_p10Pops, g_p10BusyPops, g_p10Stalls,
                     g_p10BusyPops ? 100.0 * static_cast<double>(g_p10Stalls) /
                                         static_cast<double>(g_p10BusyPops)
                                   : 0.0,
                     g_p10FirstStallStep);
        {
            long long at0 = 0, mid = 0;
            for (int i = 0; i < g_p10StallStepCount; ++i) {
                if (g_p10StallSteps[i] == 0) ++at0; else ++mid;
            }
            std::fprintf(stderr,
                         "  [P10.0a] 卡死 step 分布：step==0（轮首仪器假象，种子外给，不算）= %lld  "
                         "step>0（**真卡死**）= %lld  样本(前16)=",
                         at0, mid);
            for (int i = 0; i < g_p10StallStepCount; ++i) {
                std::fprintf(stderr, "%lld ", g_p10StallSteps[i]);
            }
            std::fprintf(stderr, "\n");
        }
        std::fprintf(stderr,
                     "  [P10.0c] 疑似卡死点全网格穷举：前沿另有**可用** gate（非卡死）= %lld  "
                     "前沿无可用 gate（**真卡死**）= %lld  前沿 gate 数合计=%lld  "
                     "其中可用=%lld\n",
                     g_p10FalseStalls, g_p10TrueStalls, g_p10MeshFrontierAtStall,
                     g_p10MeshUsableAtStall);
        for (int i = 0; i < g_p10TrueStallCount; ++i) {
            std::fprintf(stderr,
                         "  [P10.0d] 真卡死#%d step=%lld  队列长度=%lld  "
                         "全网格前沿 gate=%lld  其中可用=%lld\n",
                         i, g_p10TrueStallStep[i], g_p10TrueStallQueue[i],
                         g_p10TrueStallFrontier[i], g_p10TrueStallUsable[i]);
        }
        g_p10FalseStalls = g_p10TrueStalls = g_p10MeshFrontierAtStall = 0;
        g_p10MeshUsableAtStall = 0;
        g_p10TrueStallCount = 0;
        std::fprintf(stderr,
                     "  [P10.0b] 队列均长=%.2f  其中落在前沿上的均数=%.2f (%.1f%%)\n",
                     g_p10Pops ? static_cast<double>(g_p10QueueSum) /
                                     static_cast<double>(g_p10Pops)
                               : 0.0,
                     g_p10Pops ? static_cast<double>(g_p10FrontierSum) /
                                     static_cast<double>(g_p10Pops)
                               : 0.0,
                     g_p10QueueSum ? 100.0 * static_cast<double>(g_p10FrontierSum) /
                                         static_cast<double>(g_p10QueueSum)
                                   : 0.0);
        g_p10Pops = g_p10BusyPops = g_p10Stalls = g_p10QueueSum = g_p10FrontierSum = 0;
        g_p10FirstStallStep = -1;
        g_p10StallStepCount = 0;
    }

        // P5.5 逆向重放对拍：打印编码端**尾部** 5 个符号的环。逆向重放下解码端
    // 第 k 步对应编码端第 N-1-k 步，要对的是尾部；正向的 [encRing] 打的是头部。
    if (envOn("PMSF_RDPATCH_RING_DBG")) {
        int tailIdx = R.numDecimatedTotal - static_cast<int>(tailRings.size());
        for (const RingDebugRec& rec : tailRings) {
            std::fprintf(stderr,
                         "  [encTail] sym=%d val=%d encBary=(%d,%d,%d) ring=", tailIdx,
                         rec.valence, rec.bx, rec.by, rec.bz);
            for (const auto& q : rec.ringQ) {
                std::fprintf(stderr, "(%d,%d,%d)", q[0], q[1], q[2]);
            }
            std::fprintf(stderr, " | polyFace=");
            for (const auto& q : rec.polyQ) {
                std::fprintf(stderr, "(%d,%d,%d)", q[0], q[1], q[2]);
            }
            std::fprintf(stderr, " ringEqPoly=%s", (rec.polyQ == rec.ringQ) ? "Y" : "N");
            // 关键判据：该环在**编码结束时的最终网格**上是否是一个面。
            // 编码端最后一个符号的删除按定义应把它的环变成最终网格上的一个面
            // ⇒ 这里应当至少命中 1。若为 0，说明"环快照 ≠ 实际生成的多边形"
            // 或"之后被重新三角化"，A 路的前提在真实数据上不成立。
            std::set<std::array<int, 3>> want(rec.ringQ.begin(), rec.ringQ.end());
            int exact = 0, superset = 0;
            for (MCGAL::Facet* f : mesh->faces()) {
                if (!f || f->isRemoved()) continue;
                std::set<std::array<int, 3>> fv;
                for (MCGAL::Halfedge* hh : *f) {
                    MCGAL::Vertex* vv = hh ? hh->vertex() : nullptr;
                    if (!vv || vv->isRemoved()) continue;
                    const MCGAL::PointInt q = mesh->floatPosToInt(vv->point());
                    fv.insert({q[0], q[1], q[2]});
                }
                if (fv == want) ++exact;
                bool sup = true;
                for (const auto& w : want) {
                    if (!fv.count(w)) { sup = false; break; }
                }
                if (sup) ++superset;
            }
            std::fprintf(stderr, "  [encTailFace] 恰同顶点集的面=%d 包含它的面=%d\n",
                         exact, superset);
            ++tailIdx;
        }
    }

    return R;
}

}  // namespace

RDPatchPipelineResult runRDPatchEncoderOnMesh(std::shared_ptr<MCGAL::Mesh> mesh,
                                              const RDPatchPipelineConfig& cfg) {
    // 调用方负责 PMSF / contextPool 已初始化。这里只负责 conquest 主循环。
    return runConquest(mesh, cfg);
}

RDPatchPipelineResult runRDPatchEncoderPR_M1(const std::string& inputPath,
                                             const RDPatchPipelineConfig& cfg) {
    // PMSF 构造触发 contextPool init —— 与 baseline impl main 路径一致。
    pmsf::ensureMeshRuntimeInitialized();

    // attach kVPropRDR1 vertex 属性（与 RDTag 路径同源；PR-M1 不严格需要，但 RDTag
    // eliminate 内部 read/write rd_r1 时可能调，attach 保险）。
    MCGAL::contextPool.attachVertexProperty<MCGAL::PointInt>(
        DEFAULT_MESH_ID, MCGAL::kVPropRDR1);

    auto mesh = std::make_shared<MCGAL::Mesh>(MCGAL::Mesh(inputPath));

    return runConquest(mesh, cfg);
}

}  // namespace pmsf
