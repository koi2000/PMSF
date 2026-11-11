/*
 * @Author: koi
 * @Date: 2025-08-29 21:13:56
 * @Description:
 */

#include "OptimizedSymbolCollectOperator.h"
#include "boundary/BoundaryIdRegistry.h"
#include "common/CanonicalTieBreakKey.h"
#include "boundary/BndTraversalKey.h"
#include "entropy/StreamEntropyCoder.h"
#include "../common/DebugTool.h"
#include "../rangecoder/qsmodel.h"
#include "../rangecoder/rangecod.h"
#include "BFSVersionManager.h"
#include "BufferUtils.h"
#include "Configuration.h"
#include "Facet.h"
#include "Halfedge.h"
#include "MeshUtils.h"
#include "Point.h"
#include "core/DefaultCodecPMSFSideChannel.h"
#include "core/PaperGeometrySideChannel.h"
#include "core.h"
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <queue>
#include <utility>
#include <vector>

bool OptimizedSymbolCollectOperator::inGroup(MCGAL::Halfedge* h) const {
    if (groupFilter_ < 0) return h != nullptr;
    if (!h) return false;
    if (h->face()) return h->face()->groupId() == groupFilter_;
    return h->vertex() && h->vertex()->groupId() == groupFilter_;
}

namespace {
bool segmentedFaceLocalEdgeBits() {
    static const bool enabled = [] {
        const char* env = std::getenv("PMSF_SEGMENTED_DEFAULT_FACE_EDGE_BITS");
        return env && env[0] != '\0' && std::strcmp(env, "0") != 0 &&
               std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
    }();
    return enabled;
}

bool segmentedDefaultAbsGeometry() {
    static const bool enabled = [] {
        const char* env = std::getenv("PMSF_SEGMENTED_DEFAULT_ABS_GEOM");
        return env && env[0] != '\0' && std::strcmp(env, "0") != 0 &&
               std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
    }();
    return enabled;
}

bool segmentedSortRemainingFaces() {
    static const bool enabled = [] {
        const char* env = std::getenv("PMSF_SEGMENTED_SORT_REMAINING_FACES");
        return env && env[0] != '\0' && std::strcmp(env, "0") != 0 &&
               std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
    }();
    return enabled;
}

bool segmentedSortGateNeighbors() {
    static const bool enabled = [] {
        const char* env = std::getenv("PMSF_SEGMENTED_SORT_GATE_NEIGHBORS");
        return env && env[0] != '\0' && std::strcmp(env, "0") != 0 &&
               std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
    }();
    return enabled;
}

// [PHASE61 物品 1 ①，env 门控，默认关闭] 分区 BFS 起点漂出本组时的兜底。
//
// 缺陷（实测，N=32 / `PMSF_NO_BOUNDARY_PROTECT=1`）：`collect(seed)` 开头的
// `if (!inGroup(seed)) return;` 会让**整组**在该轮一个符号都不发（收尾扫描
// 也被一起跳过，因为 return 在函数最前面）。编码端的 `seed` 是
// `exportSeeds()` 抓的那条 `Halfedge*`，其 `face()->groupId()` 会随消除漂移；
// 解码端的 seed 是从通道里的**顶点对**在基网上重新解析出来的 —— 同一条边，
// 但可能选中另一侧半边。两侧于是**在不同轮次**丢掉同一组 ⇒ 该组的符号流
// 从第一次丢轮起整体错位 ⇒ 队列提前耗尽（`!rcDecodedFacetSymbols_.empty()`）。
//
// 开启本开关后，seed 不在本组时不再放弃整组，而是让 BFS 空转、由函数末尾的
// 收尾扫描按 `groupId` 覆盖本组全部活面 —— 与解码端**同规则**。分段路径下
// `canonicalSegmentedOrder()` 恒真（`groupFilter_ >= 0`），收尾扫描按
// `lessFacetKey` 的**几何序**排，gate 邻居亦然，故两侧顺序天然一致。
bool segmentedSeedOutOfGroupTailScan() {
    static const bool enabled = [] {
        const char* env = std::getenv("PMSF_SEG_SEED_OOG_TAILSCAN");
        return env && env[0] != '\0' && std::strcmp(env, "0") != 0 &&
               std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
    }();
    return enabled;
}

bool lessVertexKey(const std::shared_ptr<MCGAL::Mesh>& mesh,
                   MCGAL::Vertex* a,
                   MCGAL::Vertex* b) {
    if (a == b) return false;
    if (!a) return false;
    if (!b) return true;
    if (mesh && mesh->enableQuantization) {
        MCGAL::PointInt qa = mesh->floatPosToInt(a->point());
        MCGAL::PointInt qb = mesh->floatPosToInt(b->point());
        if (qa.x() != qb.x()) return qa.x() < qb.x();
        if (qa.y() != qb.y()) return qa.y() < qb.y();
        if (qa.z() != qb.z()) return qa.z() < qb.z();
    } else {
        const MCGAL::Point& pa = a->point();
        const MCGAL::Point& pb = b->point();
        if (pa.x() != pb.x()) return pa.x() < pb.x();
        if (pa.y() != pb.y()) return pa.y() < pb.y();
        if (pa.z() != pb.z()) return pa.z() < pb.z();
    }
    if (const char* tie = std::getenv("PMSF_STABLE_TOPO_TIE"); tie && tie[0] == '1') {
        const std::uint64_t ra = pmsf::stableTopoRank(mesh, a);
        const std::uint64_t rb = pmsf::stableTopoRank(mesh, b);
        if (ra != rb) return ra < rb;
    }
    if (const char* tie = std::getenv("PMSF_TOPOLOGY_ONLY_TIE"); tie && tie[0] == '1') {
        auto ka = pmsf::topologyOnlyTieKey(mesh, a);
        auto kb = pmsf::topologyOnlyTieKey(mesh, b);
        if (ka != kb) return ka < kb;
    }
    if (const char* tie = std::getenv("PMSF_CANONICAL_TOPO_TIE"); tie && tie[0] == '1') {
        auto ka = pmsf::topologyTieKey(mesh, a);
        auto kb = pmsf::topologyTieKey(mesh, b);
        if (ka != kb) return ka < kb;
    }
    return a->poolId() < b->poolId();
}

bool lessHalfedgeKey(const std::shared_ptr<MCGAL::Mesh>& mesh,
                     MCGAL::Halfedge* a,
                     MCGAL::Halfedge* b) {
    if (a == b) return false;
    if (!a) return false;
    if (!b) return true;
    if (lessVertexKey(mesh, a->vertex(), b->vertex())) return true;
    if (lessVertexKey(mesh, b->vertex(), a->vertex())) return false;
    if (lessVertexKey(mesh, a->end_vertex(), b->end_vertex())) return true;
    if (lessVertexKey(mesh, b->end_vertex(), a->end_vertex())) return false;
    if (const char* tie = std::getenv("PMSF_CANONICAL_TOPO_TIE"); tie && tie[0] == '1') {
        auto fa = pmsf::faceTieKey(mesh, a ? a->face() : nullptr);
        auto fb = pmsf::faceTieKey(mesh, b ? b->face() : nullptr);
        if (fa != fb) return fa < fb;
    }
    return a->poolId() < b->poolId();
}

MCGAL::Halfedge* canonicalFaceHalfedge(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                       MCGAL::Halfedge* h) {
    if (!h) return nullptr;
    MCGAL::Halfedge* best = h;
    MCGAL::Halfedge* cur = h->next();
    int guard = 0;
    while (cur && cur != h && ++guard < 128) {
        if (lessHalfedgeKey(mesh, best, cur)) best = cur;
        cur = cur->next();
    }
    return best;
}

bool lessFacetKey(const std::shared_ptr<MCGAL::Mesh>& mesh,
                  MCGAL::Facet* a,
                  MCGAL::Facet* b) {
    if (a == b) return false;
    if (!a) return false;
    if (!b) return true;
    MCGAL::Halfedge* ah = canonicalFaceHalfedge(mesh, a->proxyHalfedge());
    MCGAL::Halfedge* bh = canonicalFaceHalfedge(mesh, b->proxyHalfedge());
    if (lessHalfedgeKey(mesh, ah, bh)) return true;
    if (lessHalfedgeKey(mesh, bh, ah)) return false;
    if (const char* tie = std::getenv("PMSF_CANONICAL_TOPO_TIE"); tie && tie[0] == '1') {
        auto ka = pmsf::faceTieKey(mesh, a);
        auto kb = pmsf::faceTieKey(mesh, b);
        if (ka != kb) return ka < kb;
    }
    return a->poolId() < b->poolId();
}

}  // namespace

// 分区路径（`groupFilter_ >= 0`，只由 `SegmentedOptimizedSymbolCollectOperator`
// 设置）的 BFS 必须与 mesh 的**遍历序**解耦。
//
// PHASE59 实测（bunny，单遍，N=8）：编码端 `mesh_` 是原网格、解码端是逐轮重建出的
// 网格，两者的半边/面 **pool 分配序不同**，而 BFS 有两处直接按遍历序取材 ——
// 「门邻居入队序」与「剩余面尾扫序」。于是同一个 BFS 在两侧走到不同分支，符号流
// 不再自洽（N=8 实测 34379/59957，正确为 34834/69664）。N=4 没暴露是因为每轮每组
// 的 BFS 恰好覆盖满，尾扫为空操作。
//
// 两项都换成按**几何键**排序（`lessHalfedgeKey` / `lessFacetKey`；量化坐标并列时
// 才回退 `poolId()`），两侧得到同一序列。非分区路径两项恒为 false ⇒ 默认产物
// 逐字节不变。
bool OptimizedSymbolCollectOperator::canonicalRemainingFaces() const {
    return segmentedSortRemainingFaces() || canonicalSegmentedOrder();
}

bool OptimizedSymbolCollectOperator::canonicalGateNeighbors() const {
    return segmentedSortGateNeighbors() || canonicalSegmentedOrder();
}

void OptimizedSymbolCollectOperator::collect(std::vector<MCGAL::Halfedge*> seeds) {
    collect(seeds[0]);
}

void OptimizedSymbolCollectOperator::collectFromFaces(
    MCGAL::Halfedge* seed,
    const std::vector<MCGAL::Facet*>& faces) {
    const std::vector<MCGAL::Facet*>* previous = collectFacesOverride_;
    collectFacesOverride_ = &faces;
    collect(seed);
    collectFacesOverride_ = previous;
}

void OptimizedSymbolCollectOperator::collect(MCGAL::Halfedge* seed) {
    if (!seed) return;
    pmsf::resetStableTopoRankCache();
    // [PHASE61 物品 1 ①，env 门控，默认关闭] 见 segmentedSeedOutOfGroupTailScan()。
    if (!inGroup(seed) && !segmentedSeedOutOfGroupTailScan()) return;
    if (canonicalGateNeighbors()) {
        if (MCGAL::Halfedge* canonicalSeed = canonicalFaceHalfedge(mesh_, seed)) {
            seed = canonicalSeed;
        }
    }
    std::deque<char> facetSym;
    std::deque<char> edgeSym;
    std::deque<char> edgeSymCtx;  // 与 edgeSym 平行；codec_version=3 下 ctx 由 packCtx 派生
    std::deque<char> vertexSym;
    std::deque<MCGAL::Point> points;
    std::deque<MCGAL::Point> vertexPoints;
    std::deque<MCGAL::PointInt> ipoints;
    std::queue<int> gateQueue;
    const bool stableBfs = canonicalGateNeighbors();
    auto halfedgeById = [&](int hid) -> MCGAL::Halfedge* {
        return MCGAL::contextPool.getHalfedgeByIndexInSubPool(mesh_->meshId(), hid);
    };
    // [PERF, byte-neutral] 稳定分段 BFS 的优先队列原先存 halfedge id：比较器每次
    // 都要 halfedgeById×2 + lessHalfedgeKey（4 个顶点 floatPosToInt + 跨 18M 池的
    // 散列解引用），单次比较 ~25ns、全程数千万次 ⇒ 这是分段 collect 相对 N=1 FIFO
    // 暴涨的主因。改为**入队时把比较键算一次**随元素存入堆：几何键把 (qx,qy,qz)
    // 打包进一个 uint64（量化坐标恒 ≥0 且各 ≤ quantBits 位），顶点/半边 poolId 作
    // 次级 tiebreak —— 字段同序复刻 lessHalfedgeKey 的量化分支，故弹出序逐位不变，
    // 产物字节不变。堆内比较退化为连续内存上的整型比较，无解引用、无 floatPosToInt。
    // 仅在量化开启且 quantBits ≤ 20（打包不溢出）时启用（分段路径常态）；否则
    // 回退旧的 id 优先队列（逐位不变）。
    const bool topoTie = std::getenv("PMSF_CANONICAL_TOPO_TIE") != nullptr;
    const bool fastKey = !topoTie && stableBfs && mesh_ && mesh_->enableQuantization &&
                         mesh_->i_nbQuantBits <= 20;
    struct GateEntry {
        std::uint64_t gv, gev;
        std::int32_t vp, evp, hid;
    };
    auto packGeom = [](const MCGAL::PointInt& q) -> std::uint64_t {
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(q.x())) << 42) |
               (static_cast<std::uint64_t>(static_cast<std::uint32_t>(q.y())) << 21) |
               static_cast<std::uint64_t>(static_cast<std::uint32_t>(q.z()));
    };
    auto makeEntry = [&](MCGAL::Halfedge* h) -> GateEntry {
        MCGAL::Vertex* v = h->vertex();
        MCGAL::Vertex* ev = h->end_vertex();
        GateEntry e;
        e.gv = packGeom(mesh_->floatPosToInt(v->point()));
        e.gev = packGeom(mesh_->floatPosToInt(ev->point()));
        e.vp = v->poolId();
        e.evp = ev->poolId();
        e.hid = h->poolId();
        return e;
    };
    auto entryLess = [](const GateEntry& a, const GateEntry& b) -> bool {
        if (a.gv != b.gv) return a.gv < b.gv;
        if (a.vp != b.vp) return a.vp < b.vp;
        if (a.gev != b.gev) return a.gev < b.gev;
        if (a.evp != b.evp) return a.evp < b.evp;
        return a.hid < b.hid;
    };
    // 与旧 stableGateCmp 同义：堆顶 = 按 lessHalfedgeKey 的**最小**元素。
    auto fastCmp = [&](const GateEntry& lhs, const GateEntry& rhs) {
        return entryLess(rhs, lhs);
    };
    std::priority_queue<GateEntry, std::vector<GateEntry>, decltype(fastCmp)>
        fastQueue(fastCmp);
    auto stableGateCmp = [&](int lhs, int rhs) {
        return lessHalfedgeKey(mesh_, halfedgeById(rhs), halfedgeById(lhs));
    };
    std::priority_queue<int, std::vector<int>, decltype(stableGateCmp)>
        stableGateQueue(stableGateCmp);
    const bool bndV3Traversal = std::getenv("PMSF_BND_V3") != nullptr &&
                                    std::getenv("PMSF_BND_V3_CANON_TRAV") != nullptr;
    std::priority_queue<pmsf::bndid::TraversalEntry, std::vector<pmsf::bndid::TraversalEntry>,
                        pmsf::bndid::TraversalCmp>
        bndV3Queue{pmsf::bndid::TraversalCmp{}};
    auto pushGate = [&](MCGAL::Halfedge* h) {
        if (!h) return;
        if (bndV3Traversal) {
            bndV3Queue.push(pmsf::bndid::makeTraversalEntry(mesh_.get(), h));
            return;
        }
        if (fastKey) {
            fastQueue.push(makeEntry(h));
        } else if (stableBfs) {
            stableGateQueue.push(h->poolId());
        } else {
            gateQueue.push(h->poolId());
        }
    };
    auto hasGate = [&]() -> bool {
        if (bndV3Traversal) return !bndV3Queue.empty();
        if (fastKey) return !fastQueue.empty();
        return stableBfs ? !stableGateQueue.empty() : !gateQueue.empty();
    };
    auto popGate = [&]() -> int {
        if (bndV3Traversal) {
            int hid = bndV3Queue.top().hid;
            bndV3Queue.pop();
            return hid;
        }
        if (fastKey) {
            int hid = fastQueue.top().hid;
            fastQueue.pop();
            return hid;
        }
        if (!stableBfs) {
            int hid = gateQueue.front();
            gateQueue.pop();
            return hid;
        }
        int hid = stableGateQueue.top();
        stableGateQueue.pop();
        return hid;
    };
    // [PHASE61 物品 1 ①，env 门控，默认关闭] 曾试过「开开关时不推 seed」以消除
    // 两侧 seed 半边方向不一致带来的顺序差 —— **实测更差**（N=32/no-protect：
    // dec d1 由 589 掉到 571，enc 590；round 1 的一致度反而下降），已回退。
    pmsf::bndid::resetTraversalKeyCaches();
    pushGate(seed);
    int current_version = MCGAL::BFSVersionManager::current_version++;
    int fcount = 0;
    int ecount = 0;
    int addedBoundaryCount = 0;
    std::vector<MCGAL::Facet*> faces;
    std::vector<MCGAL::Halfedge*> halfedges;
    const bool faceLocalEdgeBits = segmentedFaceLocalEdgeBits() && groupFilter_ >= 0;

    // codec_version=3 规则：per-edge 编码，每条无向 edge 只发 1 bit。
    // 发射时机：halfedge 首次 pop 时（此时其 opposite 尚未被 visit）。当
    // opposite 已经 visit 过（= 第二次 pop）则跳过 —— 该 edge 的 bit 已经
    // 在 opposite 首次 pop 时编码过。边界半边（opposite==nullptr）立即发。
    //
    // isAdded 含义约定：对一条 edge 的两条 halfedge，只要其中任一 isAdded()
    // 就视为该 edge "added"。编码端在首次 pop 时 OR 一下两侧状态；解码端
    // 读到 bit=1 时同步给两条 halfedge 都打 setAdded()。
    //
    // 该规则与 v2 不同：v2 是 per-halfedge，每个 halfedge pop 都写 1 bit。
    // v3 直接对半 —— 总 bit 数 ≈ v2 的一半（每条无向 edge 只发 1 次）。
    // 依赖不变式：encoder 与 decoder 共享同一 gateQueue / 同一 setVisited
    //   推送规则（与 v2 一致），因此对 `hOpp->isVisited(current_version)` 在
    //   两端结果一致，从而对"首次 / 二次 pop"的分类对称。
    //
    // ⚠️ header.enableTriangulation=false（--no-triangulate flag）在当前实现中
    //    **不启用任何 emit skip** —— 仅保留 header bit 供未来 PPMC-style polygon
    //    保留路径使用。原先实验的 "skip-both-unsplittable"（second-pop）和
    //    "skip-if-h-face-unsplittable"（first-pop, Rule H）两种 BPV 优化规则，
    //    在 default 路径的 decoder 侧都会因 **M_r 的 encoder/decoder 构造路径
    //    产生的 halfedge pop 顺序 asymmetry** 而失步（encoder eliminate + triangulate
    //    vs decoder merge_face + create_center_vertex_without_init 在 halfedge
    //    `next()` cycle 的 ordering 上并非字节级对称）。
    //
    //    具体讨论与观察见 docs/PPMC_RD_DEFAULT_BPV_SESSION_SUMMARY.md：
    //      * facet sym bit stream 两端完全一致（face visit order 相同）；
    //      * edge sym bit stream 在中后期轮次小量失步（≤15 bit）→ RC 流 desync →
    //        decoder std::bad_alloc @ round ≥ 6。
    //    真正解决需要 RD-style gate-conquest（见 tools/DefaultRDColumnarCompress.cpp
    //    已落地的 Phase A），将 edge sym 收敛为 per-polyFace 的 valence code，
    //    彻底避开 per-halfedge 的 first/second-pop 分类问题。
    //
    //    因此本开关在 encoder/decoder 两端一律忽略：两种 header 值走相同的
    //    first-pop + emit-every-edge 规则，保证 round-trip。
    // (enableTriangulation=false 的 BPV 改进目标已由 Phase A RD 工具 takeover)

    auto drainQueue = [&]() {
    while (hasGate()) {
        int hid = popGate();
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndexInSubPool(mesh_->meshId(), hid);
        if (!h || !inGroup(h) || h->isVisited(current_version) || h->isRemoved()) {
            continue;
        }
        h->setVisited(current_version);

        // ===== facet sym =====
        if (!h->face()->isVisited(current_version)) {
            MCGAL::Facet* f = h->face();
            f->setVisited(current_version);
            unsigned sym = f->isSplittable();
            facetSym.push_back(sym);
            if (sym) {
                MCGAL::Point rmved = f->getRemovedVertexPos();
                points.push_back(rmved);
                const MCGAL::PointInt rmvQ = f->getRemovedVertexPosInt();
                ipoints.push_back(rmvQ);
                                if (pmsf::bndid::enc().on) {
                    if (MCGAL::Vertex* rv = pmsf::bndid::encFacetRemovedVertex(f))
                        pmsf::bndid::encAssignInteriorId(rv);
                }
                fcount++;
                faces.push_back(f);
            }
            if (faceLocalEdgeBits) {
                MCGAL::Halfedge* start = canonicalFaceHalfedge(mesh_, h);
                std::vector<MCGAL::Halfedge*> faceRing;
                MCGAL::Halfedge* hit = start;
                int guard = 0;
                do {
                    if (!hit) break;
                    faceRing.push_back(hit);
                    hit = hit->next();
                } while (hit && hit != start && ++guard < 64);
                auto keyVertex = [&](MCGAL::Vertex* v) {
                    auto q = mesh_->floatPosToInt(v->point());
                    const int deg = v->is_degree_dirty() ? v->vertex_degree() : v->vertex_degree_cached();
                    auto a = std::array<int, 4>{q.x() >> 2, q.y() >> 2, q.z() >> 2, deg};
                    return a;
                };
                auto edgeKey = [&](MCGAL::Halfedge* e) {
                    auto pa = keyVertex(e->vertex());
                    auto pb = keyVertex(e->end_vertex());
                    if (pb < pa) std::swap(pa, pb);
                    return std::array<int, 8>{pa[0], pa[1], pa[2], pa[3], pb[0], pb[1], pb[2], pb[3]};
                };
                std::sort(faceRing.begin(), faceRing.end(),
                          [&](MCGAL::Halfedge* a, MCGAL::Halfedge* b) { return edgeKey(a) < edgeKey(b); });
                for (MCGAL::Halfedge* e : faceRing) {
                    bool added = e->isAdded();
                    edgeSym.push_back(added ? 1 : 0);
                    if (added) {
                        if (!e->opposite() || !inGroup(e->opposite())) {
                            ++addedBoundaryCount;
                        }
                        halfedges.push_back(e);
                        ++ecount;
                    }
                }
            }
        }

        // ===== edge sym =====
        MCGAL::Halfedge* hOpp = h->opposite();
        bool emit = false;
        if (!faceLocalEdgeBits) {
            if (hOpp == nullptr || !inGroup(hOpp)) {
                emit = true;  // 边界半边：single face，立即 emit
            } else if (!hOpp->isVisited(current_version)) {
                emit = true;  // 首次 pop 规则（first-pop rule）—— 编解码对称
            }
        }
        // else: 二次 pop，本 edge 的 bit 已在 opposite 首次 pop 时发过，跳过

        if (emit) {
            edgeSymCtx.push_back((char)1);
            bool added = h->isAdded() || (hOpp && inGroup(hOpp) && hOpp->isAdded());
            if (added) {
                if (hOpp == nullptr || !inGroup(hOpp)) {
                    ++addedBoundaryCount;
                }
                edgeSym.push_back(1);
                halfedges.push_back(h);
                ecount++;
            } else {
                edgeSym.push_back(0);
            }
        }

        if (header_.isEnableAdjustment() && !h->vertex()->isVisited(current_version)) {
            h->vertex()->setVisited(current_version);
            if (h->vertex()->isAdjusted()) {
                vertexSym.push_back(1);
                vertexPoints.push_back(h->vertex()->vertexPoint());
            } else {
                vertexSym.push_back(0);
            }
        }

        std::vector<MCGAL::Halfedge*> neighborHalfedges;
        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp != nullptr && inGroup(hOpp) && !hOpp->isVisited(current_version)) {
                neighborHalfedges.push_back(hOpp);
            }
            if (hIt != nullptr && inGroup(hIt) && !hIt->isVisited(current_version)) {
                neighborHalfedges.push_back(hIt);
            }
            hIt = hIt->next();
        } while (hIt != h);
        if (canonicalGateNeighbors()) {
            std::sort(neighborHalfedges.begin(), neighborHalfedges.end(),
                      [&](MCGAL::Halfedge* a, MCGAL::Halfedge* b) {
                          return lessHalfedgeKey(mesh_, a, b);
                      });
        }
        for (MCGAL::Halfedge* nh : neighborHalfedges) {
            pushGate(nh);
        }
    }
    };
    drainQueue();
    if (mesh_ || collectFacesOverride_ != nullptr) {
        std::vector<MCGAL::Facet*> remainingFaces;
        if (collectFacesOverride_ != nullptr) {
            remainingFaces.reserve(collectFacesOverride_->size());
            for (MCGAL::Facet* f : *collectFacesOverride_) {
                if (!f || f->isRemoved() || f->isVisited(current_version)) {
                    continue;
                }
                if (groupFilter_ >= 0 && f->groupId() != groupFilter_) {
                    continue;
                }
                remainingFaces.push_back(f);
            }
        } else {
            for (MCGAL::Facet* f : mesh_->faces()) {
                if (!f || f->isRemoved() || f->isVisited(current_version)) {
                    continue;
                }
                if (groupFilter_ >= 0 && f->groupId() != groupFilter_) {
                    continue;
                }
                remainingFaces.push_back(f);
            }
        }
        if (canonicalRemainingFaces()) {
            std::sort(remainingFaces.begin(), remainingFaces.end(),
                      [&](MCGAL::Facet* a, MCGAL::Facet* b) {
                          return lessFacetKey(mesh_, a, b);
                      });
        }
        for (MCGAL::Facet* f : remainingFaces) {
            if (!f || f->isRemoved() || f->isVisited(current_version)) {
                continue;
            }
            if (groupFilter_ >= 0 && f->groupId() != groupFilter_) {
                continue;
            }
            MCGAL::Halfedge* h = f->proxyHalfedge();
            if (canonicalGateNeighbors()) {
                h = canonicalFaceHalfedge(mesh_, h);
            }
            if (!h || !inGroup(h)) continue;
            pushGate(h);
            drainQueue();
        }
    }
#ifdef DEBUG_DUMP
    std::cout << "ecount: " << ecount << std::endl;
    std::cout << "fcount: " << fcount << std::endl;
    DEBUG_DUMP_HALFEDGES(halfedges, "./newsegment/ce_new_halfedges.txt");
    DEBUG_DUMP_FACETS(faces, "./newsegment/ce_new_facets.txt");
    DEBUG_DUMP_CONTAINER(edgeSym, "./newsegment/ce_edge_sym.txt");
    DEBUG_DUMP_CONTAINER(facetSym, "./newsegment/ce_facet_sym.txt");
#endif
    facetSymbolQueues.push_back(facetSym);
    edgeSymbolQueues.push_back(edgeSym);
    edgeSymContextQueues.push_back(edgeSymCtx);
    ipointQueues.push_back(ipoints);
    pointQueues.push_back(points);
    vertexSymbolQueues.push_back(vertexSym);
    vertexPointQueues.push_back(vertexPoints);

    pmsf::RoundSymbols rs;
    pmsf::setDefaultCodecPaperRoundSymbols(
        rs, facetSym, edgeSym, vertexSym, points, vertexPoints);
    pmsf::appendOptimizedRemovedPointSideChannel(rs, ipoints);
    pmsf::appendOptimizedVertexPointSideChannel(rs, mesh_, vertexPoints);
    pmsf::appendOptimizedStorageSideChannel(
        rs, static_cast<std::int32_t>(faceLocalEdgeBits ? 2 : 1), static_cast<std::int32_t>(faceLocalEdgeBits ? 1 : 0),
        static_cast<std::int32_t>(stableBfs ? 1 : 0), static_cast<std::int32_t>(canonicalRemainingFaces() ? 1 : 0),
        static_cast<std::int32_t>(segmentedDefaultAbsGeometry() ? 1 : 0), static_cast<std::int32_t>(header_.isEnableAdjustment() ? 1 : 0),
        static_cast<std::int32_t>(edgeSymCtx.empty() ? 0 : 1), static_cast<std::int32_t>(groupFilter_));
    rounds_.push_back(std::move(rs));

    // [PHASE59 诊断，默认关闭] 把本轮标记的 halfedge/facet 以**端点量化坐标**写出，
    // 供与解码端 `item.halfedges` 逐项对拍（poolId 两端不可比，坐标可比）。

    const char* dumpEnv = std::getenv("PMSF_NOTRI_DUMP");
    if (dumpEnv && dumpEnv[0] == '1') {
        int roundIdx = (int)facetSymbolQueues.size() - 1;
        char fname[128];
        std::snprintf(fname, sizeof(fname), "/tmp/notri/enc_r%02d.txt", roundIdx);
        FILE* fp = std::fopen(fname, "w");
        if (fp) {
            std::fprintf(fp, "facetSym(%zu): ", facetSymbolQueues.back().size());
            for (char c : facetSymbolQueues.back()) std::fprintf(fp, "%d", (int)c);
            std::fprintf(fp, "\nedgeSym(%zu): ", edgeSymbolQueues.back().size());
            for (char c : edgeSymbolQueues.back()) std::fprintf(fp, "%d", (int)c);
            std::fprintf(fp, "\nedgeTargets(%zu): ", halfedges.size());
            for (MCGAL::Halfedge* h : halfedges) {
                auto va = h ? h->vertex() : nullptr;
                auto vb = h ? h->end_vertex() : nullptr;
                auto ka = va ? mesh_->floatPosToInt(va->point()) : MCGAL::PointInt(0, 0, 0);
                auto kb = vb ? mesh_->floatPosToInt(vb->point()) : MCGAL::PointInt(0, 0, 0);
                std::fprintf(fp, "%d,%d,%d>%d,%d,%d;",
                             ka[0], ka[1], ka[2], kb[0], kb[1], kb[2]);
            }
            std::fprintf(fp, "\n");
            std::fclose(fp);
        }
    }
}

int OptimizedSymbolCollectOperator::exportToBuffer(char* buffer, bool isEnableQuantization, bool isEnableCompress) {
    size_t offset = 0;
    char* facetSymbolOutBlock = nullptr;
    char* edgeSymbolOutBlock = nullptr;
    char* vertexSymbolOutBlock = nullptr;
    char* pointOutBlock = nullptr;
    // 记录两个meta，symbol的大小以及point的大小
    char* facetSymbolBuffer = new char[BUFFER_SIZE / 6];
    char* edgeSymbolBuffer = new char[BUFFER_SIZE / 6];
    char* vertexSymbolBuffer = new char[BUFFER_SIZE / 6];
    size_t facetSymbolOffset = 0;
    size_t edgeSymbolOffset = 0;
    size_t vertexSymbolOffset = 0;
    char* pointBuffer = new char[BUFFER_SIZE / 3];
    size_t pointOffset = 0;
    int compressFacetSymbolSize = 0;
    int compressEdgeSymbolSize = 0;
    int compressVertexSymbolSize = 0;
    int compressPointSize = 0;
    unsigned i_bitOffset = 0;
    for (int i = (int)facetSymbolQueues.size() - 1; i >= 0; i--) {
        const auto& facetSym = facetSymbolQueues[i];
        i_bitOffset = 0;
        for (char sym : facetSym) {
            if (isEnableQuantization) {
                writeBitsOff(sym, 1, facetSymbolBuffer, i_bitOffset, facetSymbolOffset);
            } else {
                writeChar(facetSymbolBuffer, facetSymbolOffset, sym);
            }
        }
        if (isEnableQuantization) {
            facetSymbolOffset++;
        }
    }

    for (int i = (int)edgeSymbolQueues.size() - 1; i >= 0; i--) {
        const auto& edgeSym = edgeSymbolQueues[i];
        i_bitOffset = 0;
        for (char sym : edgeSym) {
            if (isEnableQuantization) {
                writeBitsOff(sym, 1, edgeSymbolBuffer, i_bitOffset, edgeSymbolOffset);
            } else {
                writeChar(edgeSymbolBuffer, edgeSymbolOffset, sym);
            }
        }
        if (isEnableQuantization) {
            edgeSymbolOffset++;
        }
    }

    for (int i = (int)vertexSymbolQueues.size() - 1; i >= 0; i--) {
        const auto& vertexSym = vertexSymbolQueues[i];
        i_bitOffset = 0;
        for (char sym : vertexSym) {
            if (isEnableQuantization) {
                writeBitsOff(sym, 1, vertexSymbolBuffer, i_bitOffset, vertexSymbolOffset);
            } else {
                writeChar(vertexSymbolBuffer, vertexSymbolOffset, sym);
            }
        }
        if (isEnableQuantization) {
            vertexSymbolOffset++;
        }
    }

    for (int i = (int)pointQueues.size() - 1; i >= 0; i--) {
        const auto& points = pointQueues[i];
        const auto& vertexPoints = vertexPointQueues[i];
        const auto& ipoints = ipointQueues[i];
        i_bitOffset = 0;
        for (size_t pi = 0; pi < points.size(); pi++) {
            MCGAL::Point point = points[pi];
            MCGAL::PointInt ipoint = ipoints[pi];
            if (isEnableQuantization) {
                MCGAL::PointInt piq = mesh_->floatPosToInt(point);
                for (int j = 0; j < 3; j++) {
                    if (piq[j] > (1 << mesh_->i_nbQuantBits)) {
                        piq[j] = (1 << mesh_->i_nbQuantBits) - 1;
                    } else if (piq[j] < 0) {
                        piq[j] = 0;
                    }
                    writeBitsOff((uint32_t)piq[j], mesh_->i_nbQuantBits, pointBuffer, i_bitOffset, pointOffset);
                }
            } else {
                writePoint(pointBuffer, pointOffset, point);
            }
        }
        for (size_t vpi = 0; vpi < vertexPoints.size(); vpi++) {
            MCGAL::Point point = vertexPoints[vpi];
            if (isEnableQuantization) {
                MCGAL::PointInt piq = mesh_->floatPosToInt(point);
                for (int j = 0; j < 3; j++) {
                    if (piq[j] >= (1 << mesh_->i_nbQuantBits)) {
                        piq[j] = piq[j] - 1;
                    } else if (piq[j] <= 0) {
                        piq[j] = 0;
                    }
                    writeBitsOff((uint32_t)piq[j], mesh_->i_nbQuantBits, pointBuffer, i_bitOffset, pointOffset);
                }
            } else {
                writePoint(pointBuffer, pointOffset, point);
            }
        }
        if (isEnableQuantization) {
            pointOffset++;
        }
    }

    if (isEnableCompress) {
        serializeCharPointer(facetSymbolBuffer, facetSymbolOffset, facetSymbolOutBlock, compressFacetSymbolSize);
        serializeCharPointer(edgeSymbolBuffer, edgeSymbolOffset, edgeSymbolOutBlock, compressEdgeSymbolSize);
        if (header_.isEnableAdjustment()) {
            serializeCharPointer(vertexSymbolBuffer, vertexSymbolOffset, vertexSymbolOutBlock, compressVertexSymbolSize);
        }
        serializeCharPointer(pointBuffer, pointOffset, pointOutBlock, compressPointSize);
        writeInt(buffer, offset, compressFacetSymbolSize);
        writeInt(buffer, offset, compressEdgeSymbolSize);
        if (header_.isEnableAdjustment()) {
            writeInt(buffer, offset, compressVertexSymbolSize);
        }
        writeCharPointer(buffer, offset, facetSymbolOutBlock, compressFacetSymbolSize);
        writeCharPointer(buffer, offset, edgeSymbolOutBlock, compressEdgeSymbolSize);
        if (header_.isEnableAdjustment()) {
            writeCharPointer(buffer, offset, vertexSymbolOutBlock, compressVertexSymbolSize);
        }
        writeCharPointer(buffer, offset, pointOutBlock, compressPointSize);
    } else {
        writeInt(buffer, offset, facetSymbolOffset);
        writeInt(buffer, offset, edgeSymbolOffset);
        if (header_.isEnableAdjustment()) {
            writeInt(buffer, offset, vertexSymbolOffset);
        }
        writeCharPointer(buffer, offset, facetSymbolBuffer, facetSymbolOffset);
        writeCharPointer(buffer, offset, edgeSymbolBuffer, edgeSymbolOffset);
        if (header_.isEnableAdjustment()) {
            writeCharPointer(buffer, offset, vertexSymbolBuffer, vertexSymbolOffset);
        }
        writeCharPointer(buffer, offset, pointBuffer, pointOffset);
    }
    return offset;
}

int OptimizedSymbolCollectOperator::exportToBufferWithRangeCoder(char* buffer, bool isEnableQuantization) {
    size_t offset = 0;

    constexpr size_t SYM_BUF_SIZE = BUFFER_SIZE / 6;
    constexpr size_t POINT_BUF_SIZE = BUFFER_SIZE / 3;

    char* rcFacetBuf = new char[SYM_BUF_SIZE];
    char* rcEdgeBuf = new char[SYM_BUF_SIZE];
    char* rcVertexBuf = new char[SYM_BUF_SIZE];
    char* rcPointBuf = new char[POINT_BUF_SIZE];

    int totalFacetSymbols = 0;
    int totalEdgeSymbols = 0;
    int totalVertexSymbols = 0;
    int totalPointCoords = 0;
    if (const char* dump = std::getenv("PMSF_IPQ_ENC")) {
        if (FILE* fp = pmsf::bfsDumpFile(dump)) {
            std::fprintf(fp, "Q group=%d", groupFilter_);
            for (int i = (int)ipointQueues.size() - 1; i >= 0; i--) {
                for (auto& res : ipointQueues[static_cast<std::size_t>(i)])
                    std::fprintf(fp, " %d,%d,%d", res.x(), res.y(), res.z());
            }
            std::fputc('\n', fp);
        }
    }

    for (auto& q : facetSymbolQueues) totalFacetSymbols += q.size();
    for (auto& q : edgeSymbolQueues) totalEdgeSymbols += q.size();
    for (auto& q : vertexSymbolQueues) totalVertexSymbols += q.size();
    for (size_t i = 0; i < pointQueues.size(); i++) {
        totalPointCoords += pointQueues[i].size() * 3;
        totalPointCoords += vertexPointQueues[i].size() * 3;
    }
    size_t rcFacetSize = 0;
    size_t rcEdgeSize = 0;
    size_t rcVertexSize = 0;
    size_t rcPointSize = 0;

    // --- Encode facet symbols (binary 0/1) with adaptive qsmodel ---
    // 注：曾尝试按"符号序号→桶"的多上下文（N_BUCKET=4），实测 bunny 上仅省 1 字节，
    //     说明 facet sym 在不同 round 的分布波动小，单一自适应模型已接近熵下限。
    //
    // Phase S2（与 rd/pclttm 批同形）：方案可注入本条流的编码器。注入分支把
    // 同一符号序列（同样的轮次倒序）交给注入 coder，字节写进同一 size 槽
    // （rcFacetSize/rcFacetBuf）；未注入时下面**原有代码一字不动**。
    if (auto injectedFacet = ::pmsf::injectedStreamCoder("facet")) {
        std::vector<std::uint32_t> syms;
        syms.reserve(totalFacetSymbols);
        for (int i = (int)facetSymbolQueues.size() - 1; i >= 0; i--) {
            for (char sym : facetSymbolQueues[i]) {
                syms.push_back(static_cast<std::uint32_t>(static_cast<uint8_t>(sym)));
            }
        }
        std::vector<std::uint8_t> blob;
        injectedFacet->reset();
        injectedFacet->encode(syms, blob);
        if (blob.size() > SYM_BUF_SIZE) {
            throw std::runtime_error(
                "[OptimizedCollect] injected facet stream exceeds rc buffer");
        }
        if (!blob.empty()) std::memcpy(rcFacetBuf, blob.data(), blob.size());
        rcFacetSize = blob.size();
    } else {
        size_t rcOffset = 0;
        rangecoder rc;
        rc.p_data = rcFacetBuf;
        rc.p_dataOffset = &rcOffset;
        start_encoding(&rc, 0, 0);

        qsmodel model;
        initqsmodel(&model, 2, 10, 1 << 9, NULL, 1);

        for (int i = (int)facetSymbolQueues.size() - 1; i >= 0; i--) {
            for (char sym : facetSymbolQueues[i]) {
                int syfreq, ltfreq;
                qsgetfreq(&model, (int)sym, &syfreq, &ltfreq);
                encode_shift(&rc, syfreq, ltfreq, 10);
                qsupdate(&model, (int)sym);
            }
        }

        rcFacetSize = done_encoding(&rc);
        deleteqsmodel(&model);
    }

    // --- Encode edge symbols (binary 0/1) with adaptive qsmodel ---
    // codec_version=3：edge sym 只在 h->face() 可分裂时才进入此流；单 ctx 足够。
    // Phase S2：注入分支（同 facet 流的约定，符号序列为轮次倒序的 0/1）。
    if (auto injectedEdge = ::pmsf::injectedStreamCoder("edge")) {
        std::vector<std::uint32_t> syms;
        syms.reserve(totalEdgeSymbols);
        for (int i = (int)edgeSymbolQueues.size() - 1; i >= 0; i--) {
            for (char sym : edgeSymbolQueues[i]) {
                syms.push_back(static_cast<std::uint32_t>(static_cast<uint8_t>(sym)));
            }
        }
        std::vector<std::uint8_t> blob;
        injectedEdge->reset();
        injectedEdge->encode(syms, blob);
        if (blob.size() > SYM_BUF_SIZE) {
            throw std::runtime_error(
                "[OptimizedCollect] injected edge stream exceeds rc buffer");
        }
        if (!blob.empty()) std::memcpy(rcEdgeBuf, blob.data(), blob.size());
        rcEdgeSize = blob.size();
    } else {
        size_t rcOffset = 0;
        rangecoder rc;
        rc.p_data = rcEdgeBuf;
        rc.p_dataOffset = &rcOffset;
        start_encoding(&rc, 0, 0);

        qsmodel model;
        initqsmodel(&model, 2, 10, 1 << 9, NULL, 1);

        for (int i = (int)edgeSymbolQueues.size() - 1; i >= 0; i--) {
            for (char sym : edgeSymbolQueues[i]) {
                int syfreq, ltfreq;
                qsgetfreq(&model, (int)sym, &syfreq, &ltfreq);
                encode_shift(&rc, syfreq, ltfreq, 10);
                qsupdate(&model, (int)sym);
            }
        }

        rcEdgeSize = done_encoding(&rc);
        deleteqsmodel(&model);
    }

    // --- Encode vertex symbols (binary 0/1) with adaptive qsmodel ---
    if (header_.isEnableAdjustment()) {
        // Phase S2：注入分支（同 facet/edge 的约定）。
        if (auto injectedVertex = ::pmsf::injectedStreamCoder("vertex")) {
            std::vector<std::uint32_t> syms;
            syms.reserve(totalVertexSymbols);
            for (int i = (int)vertexSymbolQueues.size() - 1; i >= 0; i--) {
                for (char sym : vertexSymbolQueues[i]) {
                    syms.push_back(static_cast<std::uint32_t>(static_cast<uint8_t>(sym)));
                }
            }
            std::vector<std::uint8_t> blob;
            injectedVertex->reset();
            injectedVertex->encode(syms, blob);
            if (blob.size() > SYM_BUF_SIZE) {
                throw std::runtime_error(
                    "[OptimizedCollect] injected vertex stream exceeds rc buffer");
            }
            if (!blob.empty()) std::memcpy(rcVertexBuf, blob.data(), blob.size());
            rcVertexSize = blob.size();
        } else {
        size_t rcOffset = 0;
        rangecoder rc;
        rc.p_data = rcVertexBuf;
        rc.p_dataOffset = &rcOffset;
        start_encoding(&rc, 0, 0);

        qsmodel model;
        initqsmodel(&model, 2, 10, 1 << 9, NULL, 1);

        for (int i = (int)vertexSymbolQueues.size() - 1; i >= 0; i--) {
            for (char sym : vertexSymbolQueues[i]) {
                int syfreq, ltfreq;
                qsgetfreq(&model, (int)sym, &syfreq, &ltfreq);
                encode_shift(&rc, syfreq, ltfreq, 10);
                qsupdate(&model, (int)sym);
            }
        }

        rcVertexSize = done_encoding(&rc);
        deleteqsmodel(&model);
        }
    }

    // --- Encode point data with range coder ---
    // 两种路径：
    //   (a) enablePrediction_ + enableQuantization → 残差 ZigZag + 每位面独立二元上下文 (论文 bit-plane coding 的简化骨架)
    //   (b) 否则 → 原坐标 4096-ary 自适应模型（旧行为，向后兼容）
    {
        size_t rcOffset = 0;
        rangecoder rc;
        rc.p_data = rcPointBuf;
        rc.p_dataOffset = &rcOffset;
        start_encoding(&rc, 0, 0);

        unsigned nbQuantBits = mesh_->i_nbQuantBits;
        const bool useResidualBitPlane = enablePrediction_ && isEnableQuantization;
        const bool usePredictiveFloat = enablePrediction_ && !isEnableQuantization;
        const bool useAdaptiveModel = !useResidualBitPlane && isEnableQuantization && nbQuantBits <= 16;

        // P18.2 PHASE59 物品 1 / 物品 3：**几何编码器选路打点**。
        //
        // 背景：`enablePrediction_` 的默认值是 false，由**序列化器**在
        // `exportToBuffer*` 之前 `setEnablePrediction(true)` 顶上
        // （`BasicSerializeOperator.cpp:259`）。而分段 collector 的内层
        // 实例是在 `ensureCollectors()`（`init()` 里）**就**构造好的，
        // `SegmentedOptimizedSymbolCollectOperator` 又**没有覆写**
        // `setEnablePrediction` ⇒ 序列化器那句只落在**外层包装**上。
        // 若如此，分段路径的坐标编码器与不分段路径**不是同一个**。
        //
        // 这是**读源码**得出的怀疑（本项目读源码错过三次），故加打点实测。
        // 默认静默；`PMSF_SEG_PRED_TRACE=1` 时每个 collector 打一行。

        // 位面模型：每个位面独立 binary qsmodel。位数 = nbQuantBits + 1（1 位符号 + nbQuantBits 幅值位）
        // 注：残差幅值最坏可达 2^nbQuantBits - 1，ZigZag 后最多 nbQuantBits + 1 位。
        // 注：曾尝试按轴 (x,y,z) 拆分为 3 套独立模型，实测 bunny 上几乎无收益（<0.1%）甚至略劣，
        //     原因是模型学习速度被样本量的 1/3 拖慢；故保留单组共享模型。
        //
        // 论文 Sect. 4.1 cluster 划分：
        //   - cluster A "first-1"：从 MSB 向下扫，直到遇到第一个 '1' 之前的所有 '0' 位。
        //     这部分极度偏向 '0'（高位很少被点亮），用一组按位面索引的 qsmodel 学习。
        //   - cluster B "after-1"：第一个 '1' 之后的位，分布更接近均匀，用另一组按位面索引的 qsmodel。
        //   两 cluster 各 BP_BITS 个位面模型，合计 2 * BP_BITS。
        const int BP_BITS = (int)nbQuantBits + 1;

        // Phase S2（与 rd/pclttm 批同形）：方案可注入本条流的编码器。
        // 注入分支只覆盖 bit-plane 残差模式（其余模式两侧都回落原有代码）。
        // 展平约定：轮次倒序，每轮先 facet 残差（ipointQueues，absGeometry 开关
        // 的取值与原实现逐项一致），再 vertex 附加点（同样的 clamp 规则），
        // 每个分量 zigzag 有符号→无符号，共 totalPointCoords 个 uint32。
        // 字节写进同一 size 槽（rcPointSize/rcPointBuf）。默认分支一字不动。
        bool pointInjected = false;
        if (useResidualBitPlane) {
            if (auto injectedPoint = ::pmsf::injectedStreamCoder("point")) {
                std::vector<std::uint32_t> syms;
                syms.reserve(static_cast<std::size_t>(totalPointCoords));
                auto zigzag = [](std::int32_t v) -> std::uint32_t {
                    return (static_cast<std::uint32_t>(v) << 1) ^
                           static_cast<std::uint32_t>(v >> 31);
                };
                for (int i = (int)pointQueues.size() - 1; i >= 0; i--) {
                    const bool absGeometry = segmentedDefaultAbsGeometry();
                    const auto& points = pointQueues[i];
                    std::size_t pi = 0;
                    for (auto& res : ipointQueues[i]) {
                        MCGAL::PointInt encoded = res;
                        if (absGeometry && pi < points.size() && mesh_) {
                            encoded = mesh_->floatPosToInt(points[pi]);
                        }
                        syms.push_back(zigzag(static_cast<std::int32_t>(encoded.x())));
                        syms.push_back(zigzag(static_cast<std::int32_t>(encoded.y())));
                        syms.push_back(zigzag(static_cast<std::int32_t>(encoded.z())));
                        ++pi;
                    }
                    for (auto& point : vertexPointQueues[i]) {
                        MCGAL::PointInt pv = mesh_->floatPosToInt(point);
                        for (int j = 0; j < 3; j++) {
                            if (pv[j] >= (1 << nbQuantBits))
                                pv[j] = pv[j] - 1;
                            else if (pv[j] <= 0)
                                pv[j] = 0;
                            syms.push_back(zigzag(static_cast<std::int32_t>(pv[j])));
                        }
                    }
                }
                std::vector<std::uint8_t> blob;
                injectedPoint->reset();
                injectedPoint->encode(syms, blob);
                if (blob.size() > POINT_BUF_SIZE) {
                    throw std::runtime_error(
                        "[OptimizedCollect] injected point stream exceeds rc buffer");
                }
                if (!blob.empty()) std::memcpy(rcPointBuf, blob.data(), blob.size());
                rcPointSize = blob.size();
                pointInjected = true;
            }
        }
        if (!pointInjected) {
        std::vector<qsmodel> bpModelsFirst1;  // cluster A：未出现 '1' 之前的位
        std::vector<qsmodel> bpModelsAfter1;  // cluster B：出现 '1' 之后的位
        if (useResidualBitPlane) {
            bpModelsFirst1.resize(BP_BITS);
            bpModelsAfter1.resize(BP_BITS);
            for (int b = 0; b < BP_BITS; b++) {
                initqsmodel(&bpModelsFirst1[b], 2, 10, 1 << 9, NULL, 1);
                initqsmodel(&bpModelsAfter1[b], 2, 10, 1 << 9, NULL, 1);
            }
        }

        qsmodel coordModel;
        if (useAdaptiveModel) {
            unsigned nSymbols = 1u << nbQuantBits;
            initqsmodel(&coordModel, nSymbols, 18, 1 << 17, NULL, 1);
        }

        // P18.2 阶段 59 物品 1 后半判据「每组的符号流可被独立重放自洽」：
        // `PMSF_QSMODEL_REPLAY=1` 时把喂给 coordModel 的符号序列录下来，
        // 本轮结束前用**一个全新模型**重放同一序列。若干净模型也失衡 ⇒
        // 序列本身非法（编码器问题）；若重放通过 ⇒ 流自洽，此前的失衡
        // 只能来自「模型被越界符号污染」。默认关闭，不影响任何行为。
        const bool replayEnabled = std::getenv("PMSF_QSMODEL_REPLAY") != nullptr;
        std::vector<int> coordSymDump;
        auto coordUpdate = [&](int sym) {
            if (replayEnabled) coordSymDump.push_back(sym);
            qsupdate(&coordModel, sym);
        };

        // P18.2 PHASE59 附 17 补：**位面路径的同款重放**。
        //
        // 附 17 把分段路径的坐标编码器从自适应 4096-ary 改成了位面残差
        // （与不分段路径同源），代价是原来的 `[QSREPLAY]` 探针不再触发 ——
        // 物品 1 判据的后半「每组的符号流可被独立重放自洽」因此**失去覆盖**。
        // 这里补上等价探针：把喂给每个位面模型（2 个 cluster × BP_BITS 个位面）
        // 的 0/1 序列录下来，本轮结束前各自喂给**同参数的全新模型**。
        // 干净模型若同样触发 `dorescale` 的致命检查 ⇒ 序列本身非法；
        // 重放通过 ⇒ 该组该位面的符号流自洽。
        // 默认关闭；`PMSF_QSMODEL_REPLAY=1` 时生效。
        std::vector<std::vector<int>> bpSymDump;
        if (replayEnabled) {
            bpSymDump.assign(static_cast<std::size_t>(BP_BITS) * 2, {});
        }
        auto bpUpdate = [&](qsmodel* m, int bit) {
            if (replayEnabled) {
                const bool isAfter1 = (m >= bpModelsAfter1.data() &&
                                       m < bpModelsAfter1.data() + bpModelsAfter1.size());
                const std::size_t b = static_cast<std::size_t>(m - (isAfter1 ? bpModelsAfter1.data()
                                                                             : bpModelsFirst1.data()));
                bpSymDump[isAfter1 ? BP_BITS + b : b].push_back(bit);
            }
            qsupdate(m, bit);
        };

        qsmodel byteModel;
        if (usePredictiveFloat) {
            initqsmodel(&byteModel, 256, 12, 1 << 11, NULL, 1);
        }

        auto encodeResidualBitPlane = [&](int32_t r) {
            uint32_t z = (static_cast<uint32_t>(r) << 1) ^ static_cast<uint32_t>(r >> 31);
            bool seenOne = false;
            for (int b = BP_BITS - 1; b >= 0; b--) {
                int bit = (z >> b) & 1u;
                qsmodel* m = seenOne ? &bpModelsAfter1[b] : &bpModelsFirst1[b];
                int syfreq, ltfreq;
                qsgetfreq(m, bit, &syfreq, &ltfreq);
                encode_shift(&rc, syfreq, ltfreq, 10);
                bpUpdate(m, bit);
                if (bit) seenOne = true;
            }
        };

        auto encodeFloatWithByteModel = [&](float value) {
            uint32_t raw = 0;
            memcpy(&raw, &value, sizeof(float));
            for (int byteIdx = 0; byteIdx < 4; ++byteIdx) {
                int sym = static_cast<int>((raw >> (byteIdx * 8)) & 0xFFu);
                int syfreq, ltfreq;
                qsgetfreq(&byteModel, sym, &syfreq, &ltfreq);
                encode_shift(&rc, syfreq, ltfreq, 12);
                qsupdate(&byteModel, sym);
            }
        };

        for (int i = (int)pointQueues.size() - 1; i >= 0; i--) {
            // facet 点流
            if (useResidualBitPlane) {
                // 默认使用 ipointQueues 中保存的残差（Vector3i），由
                // VertexRemovalEliminateOperator 预测后存入。segmented default
                // 可切换为绝对量化坐标，避免分区重建时 barycenter 上下文漂移导致
                // 几何误还原；仍复用 residual bit-plane 编码，不改变流结构。
                const bool absGeometry = segmentedDefaultAbsGeometry();
                const auto& points = pointQueues[i];
                std::size_t pi = 0;
                for (auto& res : ipointQueues[i]) {
                    MCGAL::PointInt encoded = res;
                    if (absGeometry && pi < points.size() && mesh_) {
                        encoded = mesh_->floatPosToInt(points[pi]);
                    }
                    encodeResidualBitPlane(encoded.x());
                    encodeResidualBitPlane(encoded.y());
                    encodeResidualBitPlane(encoded.z());
                    ++pi;
                }
            } else {
                for (auto& point : pointQueues[i]) {
                    if (isEnableQuantization) {
                        MCGAL::PointInt pi = mesh_->floatPosToInt(point);
                        for (int j = 0; j < 3; j++) {
                            // P18.2 阶段 59 根因修复：**这里是 off-by-one**。
                            //
                            // `useAdaptiveModel` 分支下模型字母表是
                            //     nSymbols = 1u << nbQuantBits      （见上面 :864）
                            // 合法符号区间是 **[0, nSymbols-1]**。而这里原先写
                            //     `if (pi[j] > (1 << nbQuantBits))`  ⇒ 恰好等于
                            //     `1 << nbQuantBits` 的值**不被钳位**，直接进了
                            //     `qsupdate(&coordModel, pi[j])`。
                            //
                            // `qsupdate` 内是 `m->newf[sym] += m->incr;`，没有边界
                            // 检查；`newf` 是 malloc(n+1) ⇒ `newf[4096]` 落在一块
                            // **合法但不在统计范围内**的内存上（dorescale 只统计
                            // 0..n-1）。这次增量因此**从总量里凭空消失**，破坏
                            // 「sum(newf[0..n-1]) == cf[n]」不变量，下一次 dorescale
                            // 就报 `BUG: rescaling left N` 并 exit(1)。
                            //
                            // 实证（PMSF_QSMODEL_DIAG）：
                            //   [QSDIAG-OOB] qsupdate sym=4096 n=4096 (out of range)
                            //   [QSDIAG] n=4096 sum_newf=132410 cf_n=262144
                            // （sum 比 cf[n] 少了约一半，正是持续丢增量所致。）
                            //
                            // 同文件另一条路径（下面 :958 附近）用的是 `>=`，是对的；
                            // 这里漏了等号。改成 `>=` 后 4096 被钳到 4095，
                            // 而 4095 本来就合法、不受影响。
                            if (pi[j] >= (1 << nbQuantBits))
                                pi[j] = (1 << nbQuantBits) - 1;
                            else if (pi[j] < 0)
                                pi[j] = 0;
                            if (useAdaptiveModel) {
                                int syfreq, ltfreq;
                                qsgetfreq(&coordModel, pi[j], &syfreq, &ltfreq);
                                encode_shift(&rc, syfreq, ltfreq, 18);
                                coordUpdate(pi[j]);
                            } else {
                                encode_shift(&rc, 1, (freq)pi[j], (freq)nbQuantBits);
                            }
                        }
                    } else if (usePredictiveFloat) {
                        for (int j = 0; j < 3; j++) {
                            encodeFloatWithByteModel(static_cast<float>(point[j]));
                        }
                    } else {
                        for (int j = 0; j < 3; j++) {
                            float f = (float)point[j];
                            uint32_t raw;
                            memcpy(&raw, &f, sizeof(float));
                            encode_short(&rc, raw & 0xFFFF);
                            encode_short(&rc, (raw >> 16) & 0xFFFF);
                        }
                    }
                }
            }

            // 顶点附加点流（Adjustment）—— 维持原有"原坐标"编码，不走残差，避免改动量太大
            for (auto& point : vertexPointQueues[i]) {
                if (isEnableQuantization) {
                    MCGAL::PointInt pi = mesh_->floatPosToInt(point);
                    for (int j = 0; j < 3; j++) {
                        if (pi[j] >= (1 << nbQuantBits))
                            pi[j] = pi[j] - 1;
                        else if (pi[j] <= 0)
                            pi[j] = 0;
                        if (useAdaptiveModel) {
                            int syfreq, ltfreq;
                            qsgetfreq(&coordModel, pi[j], &syfreq, &ltfreq);
                            encode_shift(&rc, syfreq, ltfreq, 18);
                            qsupdate(&coordModel, pi[j]);
                        } else if (useResidualBitPlane) {
                            // 顶点附加点此处无预测，用位面编码"直接值"等价于普通熵编码
                            // 为保持编解码对称，采用与 residual 相同的 ZigZag + 位面路径，传入 pi[j] 直接作为"残差=绝对值"
                            encodeResidualBitPlane((int32_t)pi[j]);
                        } else {
                            encode_shift(&rc, 1, (freq)pi[j], (freq)nbQuantBits);
                        }
                    }
                } else if (usePredictiveFloat) {
                    for (int j = 0; j < 3; j++) {
                        encodeFloatWithByteModel(static_cast<float>(point[j]));
                    }
                } else {
                    for (int j = 0; j < 3; j++) {
                        float f = (float)point[j];
                        uint32_t raw;
                        memcpy(&raw, &f, sizeof(float));
                        encode_short(&rc, raw & 0xFFFF);
                        encode_short(&rc, (raw >> 16) & 0xFFFF);
                    }
                }
            }
        }

        rcPointSize = done_encoding(&rc);

        if (useAdaptiveModel) {
            // P18.2 阶段 59 物品 1 后半判据：独立重放。
            //
            // 判据逻辑（goal 的方法论 a）：把上面录下的符号序列喂给一个
            // **全新模型**。序列若非法（含越界符号），重放也会失衡并在
            // dorescale 处 exit(1) ⇒ 进程 rc=1；序列若自洽，重放跑完并
            // 打印 `replay=OK`。先打 out_of_range 计数，这样即使重放
            // 把进程打死，也已经有数字可读。
            if (replayEnabled) {
                const unsigned replayAlphabet = 1u << nbQuantBits;
                int outOfRange = 0;
                long long maxSym = -1;
                for (int s : coordSymDump) {
                    if (s < 0 || static_cast<unsigned>(s) >= replayAlphabet) {
                        ++outOfRange;
                    }
                    if (s > maxSym) maxSym = s;
                }
                std::fprintf(stderr,
                             "[QSREPLAY] symbols=%zu alphabet=%u max_sym=%lld "
                             "out_of_range=%d (pre-replay)\n",
                             coordSymDump.size(), replayAlphabet, maxSym,
                             outOfRange);
                std::fflush(stderr);

                qsmodel replayModel;
                initqsmodel(&replayModel, replayAlphabet, 18, 1 << 17, NULL, 1);
                for (int s : coordSymDump) {
                    qsupdate(&replayModel, s);
                }
                deleteqsmodel(&replayModel);
                std::fprintf(stderr,
                             "[QSREPLAY] replay=OK (clean model held its invariant)\n");
                std::fflush(stderr);
            }
            deleteqsmodel(&coordModel);
        }
        if (usePredictiveFloat) {
            deleteqsmodel(&byteModel);
        }
        if (useResidualBitPlane) {
            // P18.2 PHASE59 附 17 补：位面路径的**独立重放**自检。
            // 序列若非法，下面的重放会自己触发 dorescale 的致命检查、
            // 进程以 rc=1 死掉 —— 那本身就是判据要的信号。
            if (replayEnabled) {
                std::size_t total = 0;
                for (const auto& seq : bpSymDump) total += seq.size();
                for (std::size_t i = 0; i < bpSymDump.size(); ++i) {
                    qsmodel m;
                    initqsmodel(&m, 2, 10, 1 << 9, NULL, 1);
                    for (int s : bpSymDump[i]) {
                        qsupdate(&m, s);
                    }
                    deleteqsmodel(&m);
                }
                std::fprintf(stderr,
                             "[QSREPLAY-BP] models=%zu symbols=%zu "
                             "replay=OK (every bit-plane stream replayed clean)\n",
                             bpSymDump.size(), total);
                std::fflush(stderr);
            }
            for (int b = 0; b < BP_BITS; b++) {
                deleteqsmodel(&bpModelsFirst1[b]);
                deleteqsmodel(&bpModelsAfter1[b]);
            }
        }
        }
    }

    // --- Write header: compressed sizes + symbol counts ---
    writeInt(buffer, offset, (int)rcFacetSize);
    writeInt(buffer, offset, (int)rcEdgeSize);
    if (header_.isEnableAdjustment()) {
        writeInt(buffer, offset, (int)rcVertexSize);
    }
    writeInt(buffer, offset, (int)rcPointSize);

    writeInt(buffer, offset, totalFacetSymbols);
    writeInt(buffer, offset, totalEdgeSymbols);
    if (header_.isEnableAdjustment()) {
        writeInt(buffer, offset, totalVertexSymbols);
    }
    writeInt(buffer, offset, totalPointCoords);

    // --- Write compressed data blocks ---
    writeCharPointer(buffer, offset, rcFacetBuf, (int)rcFacetSize);
    writeCharPointer(buffer, offset, rcEdgeBuf, (int)rcEdgeSize);
    if (header_.isEnableAdjustment()) {
        writeCharPointer(buffer, offset, rcVertexBuf, (int)rcVertexSize);
    }
    writeCharPointer(buffer, offset, rcPointBuf, (int)rcPointSize);

#ifdef DEBUG
    // 调试信息：打印各段压缩后大小，便于验证 bpv 来源与各项优化效果
    std::cerr << "[RC sizes] facet=" << rcFacetSize
              << " (raw=" << totalFacetSymbols << ") "
              << "edge=" << rcEdgeSize
              << " (raw=" << totalEdgeSymbols << ") "
              << "point=" << rcPointSize
              << " (coords=" << totalPointCoords << ")" << std::endl;
#endif
    // bpv 对齐诊断：按 PMSF_BPV_DIAG=1 打印各段压缩后大小，
    // 便于外部 eval 核对连通性 / 几何占比。标签刻意中性（不带算法名）。
    {
        const char* env = std::getenv("PMSF_BPV_DIAG");
        if (env && env[0] == '1') {
            std::cerr << "[bpv] facet=" << rcFacetSize
                      << " (raw=" << totalFacetSymbols << ") "
                      << "edge=" << rcEdgeSize
                      << " (raw=" << totalEdgeSymbols << ") "
                      << "point=" << rcPointSize
                      << " (coords=" << totalPointCoords << ")" << std::endl;
        }
    }

    delete[] rcFacetBuf;
    delete[] rcEdgeBuf;
    delete[] rcVertexBuf;
    delete[] rcPointBuf;

    return (int)offset;
}
