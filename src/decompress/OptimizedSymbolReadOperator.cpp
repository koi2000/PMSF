/*
 * @Author: koi
 * @Date: 2025-08-31 22:33:59
 * @Description:
 */
/*
 * @Author: koi
 * @Date: 2025-08-29 21:13:56
 * @Description:
 */

#include "OptimizedSymbolReadOperator.h"
#include "common/CanonicalTieBreakKey.h"
#include "boundary/BndTraversalKey.h"
#include "entropy/StreamEntropyCoder.h"

#include "predict/RDPredictor.h"  // MCGAL::rdIntToFloatPoint
#include "../common/DebugTool.h"
#include "../rangecoder/qsmodel.h"
#include "../rangecoder/rangecod.h"
#include "BFSVersionManager.h"
#include "BufferUtils.h"
#include "Configuration.h"
#include "Facet.h"
#include "MeshUtils.h"
#include "core.h"
#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <deque>
#include <queue>
#include <vector>
#include <sys/types.h>

bool OptimizedSymbolReadOperator::inGroup(MCGAL::Halfedge* h) const {
    if (groupId < 0) return h != nullptr;
    if (!h) return false;
    if (h->face()) return h->face()->groupId() == groupId;
    return h->vertex() && h->vertex()->groupId() == groupId;
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

// [PHASE61 物品 1 ①，env 门控，默认关闭] `OptimizedSymbolCollectOperator` 里
// `segmentedSeedOutOfGroupTailScan()` 的严格镜像 —— 编解码必须同开同关，
// 否则两侧丢组的轮次不同，该组符号流整体错位。理由见那边的注释。
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
    return a->poolId() < b->poolId();
}

bool lessHalfedgeKey(const std::shared_ptr<MCGAL::Mesh>& mesh,
                     MCGAL::Halfedge* a,
                     MCGAL::Halfedge* b) {
    if (a == b) return false;
    if (!a) return false;
    if (!b) return true;
    // ⚠️ 原写法 4 次 `lessVertexKey`，每次**两**个操作数都重算 floatPosToInt。
    // 量化开启时（分段路径的常态）把 4 个顶点的键各取一次并记忆化；
    // 未开量化时**原样回退**到 `lessVertexKey` 的浮点分支 —— 两条路径的
    // 比较次序与分支逐条照抄，故结果与原先逐位相同。
    if (mesh && mesh->enableQuantization) {
        const MCGAL::PointInt qav  = mesh->floatPosToInt(a->vertex()->point());
        const MCGAL::PointInt qbv  = mesh->floatPosToInt(b->vertex()->point());
        const MCGAL::PointInt qaev = mesh->floatPosToInt(a->end_vertex()->point());
        const MCGAL::PointInt qbev = mesh->floatPosToInt(b->end_vertex()->point());
        auto lt = [&](const MCGAL::PointInt& qa, const MCGAL::PointInt& qb,
                      MCGAL::Vertex* va, MCGAL::Vertex* vb) {
            if (va == vb) return false;
            if (!va) return false;
            if (!vb) return true;
            if (qa.x() != qb.x()) return qa.x() < qb.x();
            if (qa.y() != qb.y()) return qa.y() < qb.y();
            if (qa.z() != qb.z()) return qa.z() < qb.z();
            if (const char* tie = std::getenv("PMSF_STABLE_TOPO_TIE"); tie && tie[0] == '1') {
                const std::uint64_t ra = pmsf::stableTopoRank(mesh, va);
                const std::uint64_t rb = pmsf::stableTopoRank(mesh, vb);
                if (ra != rb) return ra < rb;
            }
            if (const char* tie = std::getenv("PMSF_TOPOLOGY_ONLY_TIE"); tie && tie[0] == '1') {
                auto ka = pmsf::topologyOnlyTieKey(mesh, va);
                auto kb = pmsf::topologyOnlyTieKey(mesh, vb);
                if (ka != kb) return ka < kb;
            }
            if (const char* tie = std::getenv("PMSF_CANONICAL_TOPO_TIE"); tie && tie[0] == '1') {
                auto ka = pmsf::topologyTieKey(mesh, va);
                auto kb = pmsf::topologyTieKey(mesh, vb);
                if (ka != kb) return ka < kb;
            }
            return va->poolId() < vb->poolId();
        };
        if (lt(qav, qbv, a->vertex(), b->vertex())) return true;
        if (lt(qbv, qav, b->vertex(), a->vertex())) return false;
        if (lt(qaev, qbev, a->end_vertex(), b->end_vertex())) return true;
        if (lt(qbev, qaev, b->end_vertex(), a->end_vertex())) return false;
        if (const char* tie = std::getenv("PMSF_CANONICAL_TOPO_TIE"); tie && tie[0] == '1') {
            auto fa = pmsf::faceTieKey(mesh, a ? a->face() : nullptr);
            auto fb = pmsf::faceTieKey(mesh, b ? b->face() : nullptr);
            if (fa != fb) return fa < fb;
        }
        return a->poolId() < b->poolId();
    }
    if (lessVertexKey(mesh, a->vertex(), b->vertex())) return true;
    if (lessVertexKey(mesh, b->vertex(), a->vertex())) return false;
    if (lessVertexKey(mesh, a->end_vertex(), b->end_vertex())) return true;
    if (lessVertexKey(mesh, b->end_vertex(), a->end_vertex())) return false;
    if (const char* tie = std::getenv("PMSF_TOPOLOGY_ONLY_TIE"); tie && tie[0] == '1') {
        auto fa = pmsf::faceTieKey(mesh, a ? a->face() : nullptr);
        auto fb = pmsf::faceTieKey(mesh, b ? b->face() : nullptr);
        if (fa != fb) return fa < fb;
    }
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
}

// 见 OptimizedSymbolCollectOperator.cpp 的同名定义：分区 BFS 与 mesh 遍历序解耦。
bool OptimizedSymbolReadOperator::canonicalRemainingFaces() const {
    return segmentedSortRemainingFaces() || canonicalSegmentedOrder();
}

bool OptimizedSymbolReadOperator::canonicalGateNeighbors() const {
    return segmentedSortGateNeighbors() || canonicalSegmentedOrder();
}

void OptimizedSymbolReadOperator::collect(MCGAL::Halfedge* seed,
                                          char* buffer,
                                          size_t& dataOffset,
                                          std::vector<MCGAL::Vertex*>& vertices,
                                          std::vector<MCGAL::Halfedge*>& halfedge,
                                          std::vector<MCGAL::Facet*>& facets) {
    if (canonicalGateNeighbors()) {
        if (MCGAL::Halfedge* canonicalSeed = canonicalFaceHalfedge(mesh_, seed)) {
            seed = canonicalSeed;
        }
    }
#ifdef DEBUG
    std::deque<char> facetSymbol;
    std::deque<char> edgeSymbol;
    std::deque<MCGAL::Point> points;
#endif
    unsigned f_bitOffset = 0;
    unsigned e_bitOffset = 0;
    unsigned v_bitOffset = 0;
    unsigned p_bitOffset = 0;

    auto enqueueNeighbors = [&](MCGAL::Halfedge* h, int version) {
        std::vector<MCGAL::Halfedge*> neighborHalfedges;
        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp != nullptr && !hOpp->isVisited(version)) {
                neighborHalfedges.push_back(hOpp);
            }
            if (!hIt->isVisited(version)) {
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
            if (nh) gateQueue.push(nh->poolId());
        }
    };

    auto readQuantizedPoint = [&]() -> MCGAL::Point {
        MCGAL::Point point;
        uint32_t p[3];
        for (int i = 0; i < 3; i++) {
            p[i] = readBitsOff(12, buffer, p_bitOffset, offsets[2]);
        }
        point[0] = bboxMin_[0] + (p[0] + 0.5) * f_quantStep_;
        point[1] = bboxMin_[1] + (p[1] + 0.5) * f_quantStep_;
        point[2] = bboxMin_[2] + (p[2] + 0.5) * f_quantStep_;
        return point;
    };

    const bool stableBfs = canonicalGateNeighbors();
    auto halfedgeById = [&](int hid) -> MCGAL::Halfedge* {
        return MCGAL::contextPool.getHalfedgeByIndexInSubPool(DEFAULT_MESH_ID, hid);
    };
    // [PERF, byte-neutral] 与编码端 collector 同构的「入队时算一次比较键」优化。
    // 几何键把 (qx,qy,qz) 打包进 uint64，poolId 作次级 tiebreak，字段同序复刻
    // lessHalfedgeKey 的量化分支 ⇒ 弹出序逐位不变 ⇒ 重建网格不变。仅量化开启且
    // quantBits ≤ 20 时启用；否则回退旧的 id 优先队列（逐位不变）。
    const bool fastKey = !std::getenv("PMSF_CANONICAL_TOPO_TIE") && stableBfs &&
                         mesh_ && mesh_->enableQuantization && mesh_->i_nbQuantBits <= 20;
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
    pmsf::bndid::resetTraversalKeyCaches();
    pushGate(seed);
    int current_version = MCGAL::BFSVersionManager::current_version++;
    auto drainQueue = [&]() {
    std::vector<MCGAL::Halfedge*> neighborHalfedges;
    while (hasGate()) {
        int hid = popGate();
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndexInSubPool(DEFAULT_MESH_ID, hid);

        if (h->isVisited(current_version) || h->isRemoved()) {
            continue;
        }
        h->setVisited(current_version);

        // ===== 先解 facet sym（与编码端顺序对齐）=====
        MCGAL::Facet* f = h->face();
        if (!f->isVisited(current_version)) {
            char symbol = enableQuantization_
                ? readBitsOff(1, buffer, f_bitOffset, dataOffset)
                : readChar(buffer, dataOffset);
#ifdef DEBUG
            facetSymbol.push_back(symbol);
#endif
            f->setVisited(current_version);
            if (symbol) {
                markFacetSplittableThisRound(f);
                MCGAL::Point point = enableQuantization_
                    ? readQuantizedPoint()
                    : readPoint(buffer, offsets[2]);
                f->setRemovedVertexPos(point);
                facets.push_back(f);
            }
        }

        // ===== edge sym：与编码端 codec_version=3 规则对称（per-edge, 首次 pop 消费）=====
        MCGAL::Halfedge* hOpp = h->opposite();
        bool consume = false;
        if (hOpp == nullptr) {
            consume = true;
        } else if (!hOpp->isVisited(current_version)) {
            consume = true;  // 首次 pop
        }
        if (consume) {
            char esymbol = enableQuantization_
                ? readBitsOff(1, buffer, e_bitOffset, offsets[0])
                : readChar(buffer, offsets[0]);
#ifdef DEBUG
            edgeSymbol.push_back(esymbol);
#endif
            if (esymbol) {
                h->setAdded();
                if (hOpp != nullptr) hOpp->setAdded();
                halfedge.push_back(h);
            }
        }

        if (header_.isEnableAdjustment() && !h->vertex()->isVisited(current_version)) {
            char symbol = enableQuantization_
                ? readBitsOff(1, buffer, v_bitOffset, offsets[1])
                : readChar(buffer, offsets[1]);
            h->vertex()->setVisited(current_version);
            if (symbol) {
                h->vertex()->setAdjusted();
                vertices.push_back(h->vertex());
                MCGAL::Point point = enableQuantization_
                    ? readQuantizedPoint()
                    : readPoint(buffer, offsets[2]);
                h->vertex()->setVertexPoint(point);
            }
        }

        enqueueNeighbors(h, current_version);
    }
    };
    drainQueue();
    if (mesh_) {
        std::vector<MCGAL::Facet*> remainingFaces;
        for (MCGAL::Facet* f : mesh_->faces()) {
            if (!f || f->isRemoved() || f->isVisited(current_version)) {
                continue;
            }
            if (groupId >= 0 && f->groupId() != groupId) {
                continue;
            }
            remainingFaces.push_back(f);
        }
        if (canonicalRemainingFaces()) {
            std::sort(remainingFaces.begin(), remainingFaces.end(),
                      [&](MCGAL::Facet* a, MCGAL::Facet* b) {
                          return lessFacetKey(mesh_, a, b);
                      });
        }
        for (MCGAL::Facet* f : remainingFaces) {
            MCGAL::Halfedge* h = f->proxyHalfedge();
            if (canonicalGateNeighbors()) {
                h = canonicalFaceHalfedge(mesh_, h);
            }
            if (!h || !inGroup(h)) continue;
            gateQueue.push(h->poolId());
            drainQueue();
        }
    }

    if (enableQuantization_) {
        dataOffset++;
        offsets[0]++;
        offsets[1]++;
        offsets[2]++;
    }
#ifdef DEBUG_DUMP
    DEBUG_DUMP_CONTAINER(edgeSymbol, "./newsegment/de_edge_sym.txt");
    DEBUG_DUMP_CONTAINER(facetSymbol, "./newsegment/de_facet_sym.txt");
    DEBUG_DUMP_HALFEDGES(halfedge, "./newsegment/de_new_halfedges.txt");
    DEBUG_DUMP_FACETS(facets, "./newsegment/de_new_facets.txt");
// int i = 9;
#endif
}

void OptimizedSymbolReadOperator::collect(std::vector<MCGAL::Halfedge*> seeds,
                                          char* buffer,
                                          size_t& dataOffset,
                                          std::vector<MCGAL::Vertex*>& vertices,
                                          std::vector<MCGAL::Halfedge*>& halfedge,
                                          std::vector<MCGAL::Facet*>& facets) {}

void OptimizedSymbolReadOperator::importFromRangeCodedBuffer(char* buffer, size_t& dataOffset, unsigned nbQuantBits) {
    rcDecodedFacetSymbols_.clear();
    rcDecodedEdgeSymbols_.clear();
    rcDecodedVertexSymbols_.clear();
    rcDecodedPoints_.clear();
    rcDecodedVertexPoints_.clear();
    rcDecodedResiduals_.clear();
    rcUsingResidual_ = header_.isEnablePrediction() && enableQuantization_;
    if (edgeInjected_) {
        edgeInjected_ = false;
        injectedEdgeSyms_.clear();
    }
    if (edgeRCInitialized_) {
        deleteqsmodel(&edgeModel_);
        edgeRCInitialized_ = false;
    }
    // codec_version=3 开启 edge sym 的 "skip-when-face-not-splittable" 规则；
    // 解码端总是按相同规则处理。
    useEdgeContext_ = true;

    int compFacetSize = readInt(buffer, dataOffset);
    int compEdgeSize = readInt(buffer, dataOffset);
    int compVertexSize = 0;
    if (header_.isEnableAdjustment()) {
        compVertexSize = readInt(buffer, dataOffset);
    }
    int compPointSize = readInt(buffer, dataOffset);

    int numFacetSymbols = readInt(buffer, dataOffset);
    int numEdgeSymbols = readInt(buffer, dataOffset);
    int numVertexSymbols = 0;
    if (header_.isEnableAdjustment()) {
        numVertexSymbols = readInt(buffer, dataOffset);
    }
    int numPointCoords = readInt(buffer, dataOffset);

    // --- Decode facet symbols (binary 0/1) with adaptive qsmodel ---
    // 注：曾尝试 N_BUCKET=4 桶上下文，编解码对齐后仅省 1 字节，故回退单模型以降低复杂度。
    //
    // Phase S2：注入分支。编码侧注入时本段字节由注入 coder 产生，符号总数
    // 仍取自有格式头里的 numFacetSymbols（同序），解出后同样进
    // rcDecodedFacetSymbols_。默认分支一字不动。
    if (auto injectedFacet = ::pmsf::injectedStreamCoder("facet")) {
        std::vector<std::uint32_t> syms;
        injectedFacet->reset();
        injectedFacet->decode(
            reinterpret_cast<const std::uint8_t*>(buffer + dataOffset),
            static_cast<std::size_t>(compFacetSize),
            static_cast<std::size_t>(numFacetSymbols), syms);
        for (std::uint32_t s : syms) {
            rcDecodedFacetSymbols_.push_back(static_cast<char>(s));
        }
    } else {
        size_t rcOffset = 0;
        rangecoder rc;
        rc.p_data = buffer + dataOffset;
        rc.p_dataOffset = &rcOffset;
        start_decoding(&rc);

        qsmodel model;
        initqsmodel(&model, 2, 10, 1 << 9, NULL, 0);

        for (int i = 0; i < numFacetSymbols; i++) {
            int syfreq, ltfreq;
            ltfreq = decode_culshift(&rc, 10);
            int sym = qsgetsym(&model, ltfreq);
            qsgetfreq(&model, sym, &syfreq, &ltfreq);
            decode_update(&rc, syfreq, ltfreq, 1 << 10);
            qsupdate(&model, sym);
            rcDecodedFacetSymbols_.push_back((char)sym);
        }

        done_decoding(&rc);
        deleteqsmodel(&model);
    }
    dataOffset += compFacetSize;

    // --- Edge symbols：保留 range coder 状态，由 collectFromDecoded 在 BFS 中按上下文逐位解码 ---
    // 注：edge sym 总数 = numEdgeSymbols；compEdgeSize 为 RC 后的字节数。
    // 我们不在此预解码，因此 dataOffset 直接前推 compEdgeSize；
    // 同时记录 edge RC 起点指针让 BFS 阶段使用（指向预先 memcpy 的副本，避免 buffer_ 后续被改）。
    //
    // Phase S2：注入分支 —— 编码侧注入时本段字节由注入 coder 产生，这里整体
    // 解出到 injectedEdgeSyms_，BFS 的 decodeEdgeSym 与 decodeAllEdgeSymbols
    // 改从该队列取（edgeRC_ 保持未初始化）。默认分支一字不动。
    if (auto injectedEdge = ::pmsf::injectedStreamCoder("edge")) {
        std::vector<std::uint32_t> syms;
        injectedEdge->reset();
        injectedEdge->decode(
            reinterpret_cast<const std::uint8_t*>(buffer + dataOffset),
            static_cast<std::size_t>(compEdgeSize),
            static_cast<std::size_t>(numEdgeSymbols), syms);
        for (std::uint32_t s : syms) injectedEdgeSyms_.push_back(static_cast<char>(s));
        edgeInjected_ = true;
    } else {
    edgeRCStorage_.assign(buffer + dataOffset, buffer + dataOffset + compEdgeSize);
    edgeRCOffset_ = 0;
    edgeRC_.p_data = edgeRCStorage_.data();
    edgeRC_.p_dataOffset = &edgeRCOffset_;
    start_decoding(&edgeRC_);
    initqsmodel(&edgeModel_, 2, 10, 1 << 9, NULL, 0);
    edgeRCInitialized_ = true;
    }
    dataOffset += compEdgeSize;

    // --- Decode vertex symbols (binary 0/1) with adaptive qsmodel ---
    if (header_.isEnableAdjustment()) {
        // Phase S2：注入分支（同 facet 流的约定）。
        if (auto injectedVertex = ::pmsf::injectedStreamCoder("vertex")) {
            std::vector<std::uint32_t> syms;
            injectedVertex->reset();
            injectedVertex->decode(
                reinterpret_cast<const std::uint8_t*>(buffer + dataOffset),
                static_cast<std::size_t>(compVertexSize),
                static_cast<std::size_t>(numVertexSymbols), syms);
            for (std::uint32_t s : syms) {
                rcDecodedVertexSymbols_.push_back(static_cast<char>(s));
            }
        } else {
        size_t rcOffset = 0;
        rangecoder rc;
        rc.p_data = buffer + dataOffset;
        rc.p_dataOffset = &rcOffset;
        start_decoding(&rc);

        qsmodel model;
        initqsmodel(&model, 2, 10, 1 << 9, NULL, 0);

        for (int i = 0; i < numVertexSymbols; i++) {
            int syfreq, ltfreq;
            ltfreq = decode_culshift(&rc, 10);
            int sym = qsgetsym(&model, ltfreq);
            qsgetfreq(&model, sym, &syfreq, &ltfreq);
            decode_update(&rc, syfreq, ltfreq, 1 << 10);
            qsupdate(&model, sym);
            rcDecodedVertexSymbols_.push_back((char)sym);
        }

        done_decoding(&rc);
        deleteqsmodel(&model);
        }
        dataOffset += compVertexSize;
    }

    // --- Decode point data with range coder ---
    {
        size_t rcOffset = 0;
        rangecoder rc;
        rc.p_data = buffer + dataOffset;
        rc.p_dataOffset = &rcOffset;
        start_decoding(&rc);

        const bool useResidualBitPlane = rcUsingResidual_;
        const bool usePredictiveFloat = header_.isEnablePrediction() && !enableQuantization_;
        const bool useAdaptiveModel = !useResidualBitPlane && enableQuantization_ && nbQuantBits <= 16;

        // Phase S2：注入分支（与编码侧对称；仅覆盖 bit-plane 残差模式，
        // 其它模式两侧都回落原有代码）。展平约定：numPointCoords 个 zigzag
        // uint32，每 3 个一组成 PointInt，顺序与编码侧相同（轮次倒序，
        // 每轮先 facet 残差后 vertex 附加点）。默认分支一字不动。
        if (useResidualBitPlane) {
            if (auto injectedPoint = ::pmsf::injectedStreamCoder("point")) {
                std::vector<std::uint32_t> syms;
                injectedPoint->reset();
                injectedPoint->decode(
                    reinterpret_cast<const std::uint8_t*>(buffer + dataOffset),
                    static_cast<std::size_t>(compPointSize),
                    static_cast<std::size_t>(numPointCoords), syms);
                auto unzigzag = [](std::uint32_t zz) -> std::int32_t {
                    return static_cast<std::int32_t>(zz >> 1) ^
                           -static_cast<std::int32_t>(zz & 1u);
                };
                for (std::size_t k = 0; k + 2 < syms.size(); k += 3) {
                    rcDecodedResiduals_.push_back(MCGAL::PointInt(
                        unzigzag(syms[k]), unzigzag(syms[k + 1]),
                        unzigzag(syms[k + 2])));
                }
                dataOffset += compPointSize;
                return;
            }
        }

        const int BP_BITS = (int)nbQuantBits + 1;
        // 与编码端对称：cluster A "first-1" + cluster B "after-1"，每 cluster 各 BP_BITS 个独立位面模型。
        std::vector<qsmodel> bpModelsFirst1;
        std::vector<qsmodel> bpModelsAfter1;
        if (useResidualBitPlane) {
            bpModelsFirst1.resize(BP_BITS);
            bpModelsAfter1.resize(BP_BITS);
            for (int b = 0; b < BP_BITS; b++) {
                initqsmodel(&bpModelsFirst1[b], 2, 10, 1 << 9, NULL, 0);
                initqsmodel(&bpModelsAfter1[b], 2, 10, 1 << 9, NULL, 0);
            }
        }

        qsmodel coordModel;
        if (useAdaptiveModel) {
            unsigned nSymbols = 1u << nbQuantBits;
            initqsmodel(&coordModel, nSymbols, 18, 1 << 17, NULL, 0);
        }

        qsmodel byteModel;
        if (usePredictiveFloat) {
            initqsmodel(&byteModel, 256, 12, 1 << 11, NULL, 0);
        }

        auto decodeResidualBitPlane = [&]() -> int32_t {
            uint32_t z = 0;
            bool seenOne = false;
            for (int b = BP_BITS - 1; b >= 0; b--) {
                qsmodel* m = seenOne ? &bpModelsAfter1[b] : &bpModelsFirst1[b];
                int ltfreq = decode_culshift(&rc, 10);
                int sym = qsgetsym(m, ltfreq);
                int syfreq, lt;
                qsgetfreq(m, sym, &syfreq, &lt);
                decode_update(&rc, syfreq, lt, 1 << 10);
                qsupdate(m, sym);
                z |= ((uint32_t)(sym & 1)) << b;
                if (sym) seenOne = true;
            }
            return (int32_t)((z >> 1) ^ (-(int32_t)(z & 1)));
        };

        auto decodeFloatWithByteModel = [&]() -> float {
            uint32_t raw = 0;
            for (int byteIdx = 0; byteIdx < 4; ++byteIdx) {
                int ltfreq = decode_culshift(&rc, 12);
                int sym = qsgetsym(&byteModel, ltfreq);
                int syfreq, lt;
                qsgetfreq(&byteModel, sym, &syfreq, &lt);
                decode_update(&rc, syfreq, lt, 1 << 12);
                qsupdate(&byteModel, sym);
                raw |= static_cast<uint32_t>(sym & 0xFF) << (byteIdx * 8);
            }
            float value = 0.0f;
            memcpy(&value, &raw, sizeof(float));
            return value;
        };

        // 每个"点"（无论 facet 点还是 vertex 点）由 3 个分量组成。
        // 注意：在残差位面路径下，**facet 点**被编码成残差，**vertex 点**（adjustment）仍按绝对值位面编码。
        // 因为当前 RD 压缩流程 isEnableAdjustment()==false，所以此处只需处理 facet 点残差；
        // 若将来开启 adjustment，解码端亦保持"前 N 个是 facet 残差，后 M 个是 vertex 绝对值"的顺序。
        int numPoints = numPointCoords / 3;
        for (int i = 0; i < numPoints; i++) {
            if (useResidualBitPlane) {
                int32_t rx = decodeResidualBitPlane();
                int32_t ry = decodeResidualBitPlane();
                int32_t rz = decodeResidualBitPlane();
                rcDecodedResiduals_.push_back(MCGAL::PointInt(rx, ry, rz));
            } else if (enableQuantization_) {
                MCGAL::Point point;
                uint32_t p[3];
                for (int j = 0; j < 3; j++) {
                    if (useAdaptiveModel) {
                        int syfreq, ltfreq;
                        ltfreq = decode_culshift(&rc, 18);
                        int sym = qsgetsym(&coordModel, ltfreq);
                        qsgetfreq(&coordModel, sym, &syfreq, &ltfreq);
                        decode_update(&rc, syfreq, ltfreq, 1 << 18);
                        qsupdate(&coordModel, sym);
                        p[j] = (uint32_t)sym;
                    } else {
                        p[j] = decode_culshift(&rc, nbQuantBits);
                        decode_update(&rc, 1, p[j], (freq)1 << nbQuantBits);
                    }
                }
                point[0] = bboxMin_[0] + (p[0] + 0.5) * f_quantStep_;
                point[1] = bboxMin_[1] + (p[1] + 0.5) * f_quantStep_;
                point[2] = bboxMin_[2] + (p[2] + 0.5) * f_quantStep_;
                rcDecodedPoints_.push_back(point);
            } else if (usePredictiveFloat) {
                float coords[3];
                for (int j = 0; j < 3; j++) {
                    coords[j] = decodeFloatWithByteModel();
                }
                rcDecodedPoints_.push_back(MCGAL::Point(coords[0], coords[1], coords[2]));
            } else {
                float coords[3];
                for (int j = 0; j < 3; j++) {
                    uint16_t low = decode_short(&rc);
                    uint16_t high = decode_short(&rc);
                    uint32_t raw = (uint32_t)low | ((uint32_t)high << 16);
                    memcpy(&coords[j], &raw, sizeof(float));
                }
                rcDecodedPoints_.push_back(MCGAL::Point(coords[0], coords[1], coords[2]));
            }
        }

        done_decoding(&rc);

        if (useAdaptiveModel) {
            deleteqsmodel(&coordModel);
        }
        if (usePredictiveFloat) {
            deleteqsmodel(&byteModel);
        }
        if (useResidualBitPlane) {
            for (int b = 0; b < BP_BITS; b++) {
                deleteqsmodel(&bpModelsFirst1[b]);
                deleteqsmodel(&bpModelsAfter1[b]);
            }
        }
    }
    if (const char* dump = std::getenv("PMSF_IPQ_DEC")) {
        static int importIdx = 0;
        if (FILE* fp = pmsf::bfsDumpFile(dump)) {
            std::fprintf(fp, "Q import=%d", importIdx++);
            for (auto& res : rcDecodedResiduals_)
                std::fprintf(fp, " %d,%d,%d", res.x(), res.y(), res.z());
            std::fputc('\n', fp);
        }
    }
    dataOffset += compPointSize;
}

void OptimizedSymbolReadOperator::decodeAllEdgeSymbols(int count) {
    // Phase S2：注入分支 —— 符号已在 import 时整体解出到 injectedEdgeSyms_。
    if (edgeInjected_) {
        rcDecodedEdgeSymbols_.clear();
        for (int i = 0; i < count && !injectedEdgeSyms_.empty(); ++i) {
            rcDecodedEdgeSymbols_.push_back(injectedEdgeSyms_.front());
            injectedEdgeSyms_.pop_front();
        }
        return;
    }
    if (!edgeRCInitialized_ || count <= 0) {
        return;
    }
    rcDecodedEdgeSymbols_.clear();
    for (int i = 0; i < count; ++i) {
        int ltfreq = decode_culshift(&edgeRC_, 10);
        int sym = qsgetsym(&edgeModel_, ltfreq);
        int syfreq, lt;
        qsgetfreq(&edgeModel_, sym, &syfreq, &lt);
        decode_update(&edgeRC_, syfreq, lt, 1 << 10);
        qsupdate(&edgeModel_, sym);
        rcDecodedEdgeSymbols_.push_back(static_cast<char>(sym));
    }
}

void OptimizedSymbolReadOperator::collectFromDecoded(MCGAL::Halfedge* seed,
                                                     std::vector<MCGAL::Vertex*>& vertices,
                                                     std::vector<MCGAL::Halfedge*>& halfedge,
                                                     std::vector<MCGAL::Facet*>& facets) {
    pmsf::resetStableTopoRankCache();
    // 空轮不驱动 BFS。
    //
    // 编码端按 header.round 记录全部轮次，包括「本轮零消除」的空轮 —— 这类轮次
    // 的 facet symbol 流长度为 0。但解码端的 BFS 由「种子 + 每轮 gate」驱动，
    // 空轮的种子 gate 依然存在，于是会去访问一个面并消费 facet symbol，越界断言
    // （`!rcDecodedFacetSymbols_.empty()`；实测某 anchor 密集网格的 round 0：
    //  declared=0 consumed=0 faceVisits=1）。
    //
    // facet symbol 与「被访问的面」一一对应，因此「facet 流为空」等价于
    // 「本轮没有任何面被访问」等价于「本轮无事可做」。直接返回即可，既不改变
    // 非空轮的行为（它们的流非空），也不改变重建结果（空轮本就没有可重建的信息）。
    if (rcDecodedFacetSymbols_.empty()) {
        return;
    }

    if (!seed) return;
    // [PHASE61 物品 1 ①，env 门控，默认关闭] 与编码端同规则，必须同开同关。
    if (!inGroup(seed) && !segmentedSeedOutOfGroupTailScan()) return;
    if (canonicalGateNeighbors()) {
        if (MCGAL::Halfedge* canonicalSeed = canonicalFaceHalfedge(mesh_, seed)) {
            seed = canonicalSeed;
        }
    }
    const bool stableBfs = canonicalGateNeighbors();
    auto halfedgeById = [&](int hid) -> MCGAL::Halfedge* {
        return MCGAL::contextPool.getHalfedgeByIndexInSubPool(DEFAULT_MESH_ID, hid);
    };
    // [PERF, byte-neutral] 与编码端 collector 同构的「入队时算一次比较键」优化。
    // 几何键把 (qx,qy,qz) 打包进 uint64，poolId 作次级 tiebreak，字段同序复刻
    // lessHalfedgeKey 的量化分支 ⇒ 弹出序逐位不变 ⇒ 重建网格不变。仅量化开启且
    // quantBits ≤ 20 时启用；否则回退旧的 id 优先队列（逐位不变）。
    const bool fastKey = !std::getenv("PMSF_CANONICAL_TOPO_TIE") && stableBfs &&
                         mesh_ && mesh_->enableQuantization && mesh_->i_nbQuantBits <= 20;
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
    // [PHASE61 物品 1 ①] 「不推 seed」的镜像实验已回退，见编码端同处注释。
    pmsf::bndid::resetTraversalKeyCaches();
    pushGate(seed);
    int current_version = MCGAL::BFSVersionManager::current_version++;

    // 诊断计数器（PMSF_NOTRI_DIAG=1 时打印）
    int dbg_faceVisits = 0;
    int dbg_facetSymConsumed = 0;
    int dbg_edgeSymConsumed = 0;
    int dbg_splittableSeen = 0;
    int dbg_skipBothUnsplittableHits = 0;
    // 全量 dump（PMSF_NOTRI_DUMP=1）
    std::string dbg_facetSymTrace;
    std::string dbg_edgeSymTrace;
    const char* dumpEnv = std::getenv("PMSF_NOTRI_DUMP");
    const bool dumpOn = dumpEnv && dumpEnv[0] == '1';

    // edge sym 实时解码 lambda（单二元模型；v3 按 "跳过不可分裂 face" 规则）。
    // Phase S2：注入路径从 import 时解出的队列取符号。
    auto decodeEdgeSym = [&]() -> char {
        if (edgeInjected_) {
            char s = injectedEdgeSyms_.front();
            injectedEdgeSyms_.pop_front();
            return s;
        }
        int ltfreq = decode_culshift(&edgeRC_, 10);
        int sym = qsgetsym(&edgeModel_, ltfreq);
        int syfreq, lt;
        qsgetfreq(&edgeModel_, sym, &syfreq, &lt);
        decode_update(&edgeRC_, syfreq, lt, 1 << 10);
        qsupdate(&edgeModel_, sym);
        return (char)sym;
    };

    // codec_version=3：edge sym 仅在 h->face()->isSplittable() 时进入流，
    //   per-halfedge 粒度但"跳过不可分裂 face"。
    // codec_version<=2：legacy per-halfedge 流，每次 pop 都消费 1 bit。
    const bool skipFaceBased = (codecVersion_ == 0 || codecVersion_ >= 3);
    const bool faceLocalEdgeBits = segmentedFaceLocalEdgeBits() && groupId >= 0;
    // [QEM binary adjustment fix] 编码端把本轮的 point 流展平成
    //   「facet 残差块 → vertex adjustment 块」两段；解码端若在 BFS 中
    // 交替 pop，会在两者都存在时错位（首个受害路径：QEM binary 的
    // adjustment=true；表现为插入顶点全体偏移 ~2e-3）。
    // 本函数把 adjustment 的 pop 推迟到本轮 facet 残差全部消费之后（末尾）。
    std::deque<MCGAL::Vertex*> deferredAdjustVertices;
    // ⚠️ enableTriangulation=false 在当前实现中**不影响 edge sym 解码逻辑** ——
    //    encoder 端也忽略此 header bit，两端均走 plain first-pop 规则。详细原因
    //    见 src/compress/OptimizedSymbolCollectOperator.cpp 顶部注释。

    auto drainQueue = [&]() {
    // PHASE63 物品1（R2 第二刀）：`neighborHalfedges` 原先**每次循环新建**，
    // 每次 2~4 个 push_back ⇒ 每轮 2~3 次 malloc/free；解码要跑百万轮
    //（armadillo 解压热点里 `malloc` 9.01%），这里是主要来源之一。
    // 提到循环外 + `clear()`（保留 capacity）⇒ 分配次数 O(轮数) → O(1)。
    // 每轮内容仍从头重建、**逐元素与顺序都不变** ⇒ 产物逐字节不变。
    std::vector<MCGAL::Halfedge*> neighborHalfedges;
    while (hasGate()) {
        int hid = popGate();
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndexInSubPool(DEFAULT_MESH_ID, hid);

        MCGAL::Facet* f = h ? h->face() : nullptr;
        if (!h || !inGroup(h) || h->isVisited(current_version) || h->isRemoved() ||
            !f || f->isRemoved()) {
            continue;
        }
        h->setVisited(current_version);

        // ===== 先解 facet sym，确保 face.splittable 在下一步 edge 解码前就位 =====
        if (!f->isVisited(current_version)) {
            f->setVisited(current_version);
            dbg_faceVisits++;

            assert(!rcDecodedFacetSymbols_.empty());
            char symbol = rcDecodedFacetSymbols_.front();
            rcDecodedFacetSymbols_.pop_front();
            dbg_facetSymConsumed++;
            if (symbol) dbg_splittableSeen++;
            if (dumpOn) dbg_facetSymTrace.push_back(symbol ? '1' : '0');

            if (symbol) {
                if (f->isUnsplittable()) {
                    f->resetSplittableFlag();
                }
                if (!f->isSplittable()) {
                    f->setSplittable();
                }
                if (rcUsingResidual_) {
                    assert(!rcDecodedResiduals_.empty());
                    MCGAL::PointInt res = rcDecodedResiduals_.front();
                    rcDecodedResiduals_.pop_front();
                    f->setRemovedVertexPosInt(res);
                } else {
                    assert(!rcDecodedPoints_.empty());
                    MCGAL::Point point = rcDecodedPoints_.front();
                    rcDecodedPoints_.pop_front();
                    f->setRemovedVertexPos(point);
                }
                facets.push_back(f);
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
                    char esymbol;
                    if (useEdgeContext_) {
                        esymbol = decodeEdgeSym();
                    } else {
                        assert(!rcDecodedEdgeSymbols_.empty());
                        esymbol = rcDecodedEdgeSymbols_.front();
                        rcDecodedEdgeSymbols_.pop_front();
                    }
                    dbg_edgeSymConsumed++;
                    if (dumpOn) dbg_edgeSymTrace.push_back(esymbol ? '1' : '0');
                    if (esymbol) {
                        if (!e->isAdded()) {
                            e->setAdded();
                        }
                        halfedge.push_back(e);
                    }
                }
            }
        }

        // ===== edge sym =====
        // codec_version=3：per-edge，首次 pop 时消费 1 bit；二次 pop 跳过。
        // codec_version<=2：legacy per-halfedge，每次 pop 都消费 1 bit。
        MCGAL::Halfedge* hOppForEdge = h->opposite();
        bool consume = true;
        if (faceLocalEdgeBits) {
            consume = false;
        } else if (skipFaceBased) {
            if (hOppForEdge == nullptr || !inGroup(hOppForEdge)) {
                consume = true;
            } else if (hOppForEdge->isVisited(current_version)) {
                consume = false;  // 二次 pop（first-pop 规则下已消费过）
            }
        }
        if (consume) {
            char esymbol;
            if (useEdgeContext_) {
                esymbol = decodeEdgeSym();
            } else {
                assert(!rcDecodedEdgeSymbols_.empty());
                esymbol = rcDecodedEdgeSymbols_.front();
                rcDecodedEdgeSymbols_.pop_front();
            }
            dbg_edgeSymConsumed++;
            if (dumpOn) dbg_edgeSymTrace.push_back(esymbol ? '1' : '0');
            if (esymbol) {
                if (!h->isAdded()) {
                    h->setAdded();
                }
                MCGAL::Halfedge* hOpp = h->opposite();
                if (skipFaceBased && hOpp != nullptr && inGroup(hOpp) && !hOpp->isAdded()) {
                    hOpp->setAdded();  // v3：两侧 halfedge 共用 added 标记
                }
                halfedge.push_back(h);
            }
        }

        if (header_.isEnableAdjustment() && !h->vertex()->isVisited(current_version)) {
            h->vertex()->setVisited(current_version);
            assert(!rcDecodedVertexSymbols_.empty());
            char symbol = rcDecodedVertexSymbols_.front();
            rcDecodedVertexSymbols_.pop_front();
            if (symbol) {
                h->vertex()->setAdjusted();
                vertices.push_back(h->vertex());
                // 推迟到本轮 facet 残差消费完之后再 pop（见函数末尾），
                // 与编码端「facet 残差块 → adjustment 块」的写序一致。
                deferredAdjustVertices.push_back(h->vertex());
            }
        }

        neighborHalfedges.clear();
        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp != nullptr && inGroup(hOpp) && !hOpp->isVisited(current_version)) {
                neighborHalfedges.push_back(hOpp);
            }
            if (inGroup(hIt) && !hIt->isVisited(current_version)) {
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
    if (mesh_ || decodedFacesOverride_ != nullptr) {
        std::vector<MCGAL::Facet*> remainingFaces;
        if (decodedFacesOverride_ != nullptr) {
            remainingFaces.reserve(decodedFacesOverride_->size());
            for (MCGAL::Facet* f : *decodedFacesOverride_) {
                if (!f || f->isRemoved() || f->isVisited(current_version)) {
                    continue;
                }
                if (groupId >= 0 && f->groupId() != groupId) {
                    continue;
                }
                remainingFaces.push_back(f);
            }
        } else {
            for (MCGAL::Facet* f : mesh_->faces()) {
                if (!f || f->isRemoved() || f->isVisited(current_version)) {
                    continue;
                }
                if (groupId >= 0 && f->groupId() != groupId) {
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
            if (groupId >= 0 && f->groupId() != groupId) {
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

    // [QEM binary adjustment fix] 本轮 facet 残差已全部 pop 完，再按 BFS 顺序
    // 消费 adjustment 点（编码端同轮先写 facet 残差、后写 adjustment）。
    for (MCGAL::Vertex* v : deferredAdjustVertices) {
        if (!v) continue;
        if (rcUsingResidual_) {
            assert(!rcDecodedResiduals_.empty());
            MCGAL::PointInt res = rcDecodedResiduals_.front();
            rcDecodedResiduals_.pop_front();
            v->setVertexPoint(
                MCGAL::rdIntToFloatPoint(res, mesh_->bboxMin, mesh_->f_quantStep));
        } else {
            assert(!rcDecodedPoints_.empty());
            MCGAL::Point point = rcDecodedPoints_.front();
            rcDecodedPoints_.pop_front();
            v->setVertexPoint(point);
        }
    }

    if (dumpOn) {
        static int s_decRoundIdx = 0;
        char fname[128];
        std::snprintf(fname, sizeof(fname), "/tmp/notri/dec_r%02d.txt", s_decRoundIdx++);
        FILE* fp = std::fopen(fname, "w");
        if (fp) {
            std::fprintf(fp, "facetSym(%zu): %s\nedgeSym(%zu): %s\n",
                         dbg_facetSymTrace.size(), dbg_facetSymTrace.c_str(),
                         dbg_edgeSymTrace.size(), dbg_edgeSymTrace.c_str());
            std::fprintf(fp, "edgeTargets(%zu): ", halfedge.size());
            for (MCGAL::Halfedge* h : halfedge) {
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
    if (const char* lo = std::getenv("PMSF_FACET_LEFTOVER"); lo && lo[0] == '1') {
        static int call = 0;
        std::fprintf(stderr, "[facet-left] call=%d left=%zu consumed=%d\n",
                     call++, rcDecodedFacetSymbols_.size(), dbg_facetSymConsumed);
    }
}

void OptimizedSymbolReadOperator::collectFromDecodedFromFaces(
    MCGAL::Halfedge* seed,
    const std::vector<MCGAL::Facet*>& faces,
    std::vector<MCGAL::Vertex*>& vertices,
    std::vector<MCGAL::Halfedge*>& halfedge,
    std::vector<MCGAL::Facet*>& facets) {
    const std::vector<MCGAL::Facet*>* previous = decodedFacesOverride_;
    decodedFacesOverride_ = &faces;
    collectFromDecoded(seed, vertices, halfedge, facets);
    decodedFacesOverride_ = previous;
}

// ============================================================================
// Parquet 多表方案：从结构化符号数据导入内部队列
// ============================================================================

void OptimizedSymbolReadOperator::importFromParquet(
    const std::vector<bool>& facetSymbols,
    const std::vector<bool>& edgeSymbols,
    const std::vector<bool>& vertexSymbols,
    const std::vector<MCGAL::Point>& points) {

    rcDecodedFacetSymbols_.clear();
    rcDecodedEdgeSymbols_.clear();
    rcDecodedVertexSymbols_.clear();
    rcDecodedPoints_.clear();
    rcDecodedVertexPoints_.clear();

    // 将 bool 向量转换为 char 队列（与 importFromRangeCodedBuffer 输出格式一致）
    for (bool s : facetSymbols) {
        rcDecodedFacetSymbols_.push_back(s ? 1 : 0);
    }
    for (bool s : edgeSymbols) {
        rcDecodedEdgeSymbols_.push_back(s ? 1 : 0);
    }
    for (bool s : vertexSymbols) {
        rcDecodedVertexSymbols_.push_back(s ? 1 : 0);
    }
    // 点坐标直接导入
    for (auto& p : points) {
        rcDecodedPoints_.push_back(p);
    }
}

void OptimizedSymbolReadOperator::importResidualsFromParquet(
    const std::vector<MCGAL::PointInt>& residuals) {
    rcDecodedResiduals_.clear();
    for (const auto& r : residuals) {
        rcDecodedResiduals_.push_back(r);
    }
    rcUsingResidual_ = true;
}
