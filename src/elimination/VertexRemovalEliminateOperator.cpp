/*
 * @Author: koi
 * @Date: 2025-08-27 22:37:03
 * @Description:
 */
#include "VertexRemovalEliminateOperator.h"
#include "boundary/BoundaryTopologyStats.h"
#include "boundary/BoundaryIdRegistry.h"
#include "common/BoundaryPatchCodec.h"
#include "boundary/BoundaryTopologyPatch.h"
#include "common/BoundedRingTraversal.h"
#include "Facet.h"
#include "Global.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "MeshUtils.h"
#include "Point.h"
#include "Vertex.h"
#include "common/Define.h"
#include "common/HalfedgeErrorEntry.h"
#include "operator/BoundaryIsRemovableOperator.h"
#include "operator/UnifiedManifoldIsRemovableOperator.h"
#include "predict/RDPredictor.h"
#include "common/CanonicalTieBreakKey.h"
#include <algorithm>
#include <array>
#include <climits>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <set>
#include <vector>

namespace {

// perf（2026-11-09 QEM 实测）：以下诊断 env 在消除/选择热循环里被逐次调用
// （单次 QEM 压缩合计 ~60 万次 getenv，每次线性扫 environ）。开关在进程内不变，
// 缓存一次；诊断用法不变（启动前设置 env 仍生效）。
inline bool boundaryTopologyStatsOn() {
    static const bool on = std::getenv("PMSF_BOUNDARY_TOPOLOGY_STATS") != nullptr;
    return on;
}
inline const char* ownedBoundaryManifestPath() {
    static const char* v = std::getenv("PMSF_OWNED_BOUNDARY_MANIFEST");
    return v;
}
inline bool absRemovedPosEnabled() {
    static const bool on = std::getenv("PMSF_ABS_REMOVED_POS") != nullptr;
    return on;
}
inline const char* btpOrderEncPath() {
    static const char* v = std::getenv("PMSF_BTP_ORDER_ENC");
    return v;
}
inline bool triDropDupPolyEnabled() {
    static const bool on = std::getenv("PMSF_TRI_DROP_DUP_POLY") != nullptr;
    return on;
}

bool quantizedRemovalWouldDegenerate(MCGAL::Mesh* mesh, MCGAL::Halfedge* h) {
    if (!mesh || !h || !h->vertex() || !h->end_vertex()) return false;
    MCGAL::Vertex* center = h->end_vertex();
    MCGAL::Vertex* survivor = h->vertex();
    if (!center || center->isRemoved() || !survivor || survivor->isRemoved()) return false;
    std::vector<MCGAL::Vertex*> ring;
    for (MCGAL::Halfedge* rh : pmsf::boundedRing(center)) {
        if (!rh || rh->isRemoved() || rh->vertex() != center) continue;
        MCGAL::Vertex* rv = rh->end_vertex();
        if (!rv || rv->isRemoved()) continue;
        ring.push_back(rv);
    }
    if (ring.size() < 3) return false;
    auto qi = [&](MCGAL::Vertex* v) { return mesh->floatPosToInt(v->point()); };
    auto cross2 = [](const MCGAL::PointInt& a, const MCGAL::PointInt& b, const MCGAL::PointInt& c) {
        const long long abx = static_cast<long long>(b.x()) - a.x();
        const long long aby = static_cast<long long>(b.y()) - a.y();
        const long long abz = static_cast<long long>(b.z()) - a.z();
        const long long acx = static_cast<long long>(c.x()) - a.x();
        const long long acy = static_cast<long long>(c.y()) - a.y();
        const long long acz = static_cast<long long>(c.z()) - a.z();
        const long long cx = aby * acz - abz * acy;
        const long long cy = abz * acx - abx * acz;
        const long long cz = abx * acy - aby * acx;
        return cx * cx + cy * cy + cz * cz;
    };
    const MCGAL::PointInt qs = qi(survivor);
    for (MCGAL::Vertex* rv : ring) {
        const MCGAL::PointInt q = qi(rv);
        if (q.x() == qs.x() && q.y() == qs.y() && q.z() == qs.z()) return true;
    }
    if (std::getenv("PMSF_QUANT_GUARD_AREA")) {
        for (size_t i = 0; i < ring.size(); ++i) {
            const MCGAL::PointInt q1 = qi(ring[i]);
            const MCGAL::PointInt q2 = qi(ring[(i + 1) % ring.size()]);
            if (cross2(qs, q1, q2) == 0) return true;
        }
    }
    return false;
}

bool removalWouldViolateLinkCondition(MCGAL::Halfedge* h) {
    if (!h || !h->next() || !h->end_vertex()) {
        return true;
    }

    MCGAL::Vertex* center = h->end_vertex();
    std::vector<MCGAL::Vertex*> poly;
    poly.reserve(16);
    poly.push_back(h->vertex());

    bool closed = true;
    MCGAL::Halfedge* cur = h;
    for (int guard = 0; guard < 4096; ++guard) {
        MCGAL::Halfedge* next = cur ? cur->next() : nullptr;
        if (!next || !next->end_vertex()) {
            return true;
        }
        MCGAL::Vertex* boundaryVertex = next->end_vertex();
        if (boundaryVertex != center && boundaryVertex != poly.front()) {
            poly.push_back(boundaryVertex);
        }

        MCGAL::Halfedge* opp = next->opposite();
        if (!opp) {
            closed = false;
            break;
        }
        cur = opp;
        if (cur == h) {
            break;
        }
    }

    if (poly.size() < 3) {
        return true;
    }
    return UnifiedManifoldIsRemovableOperator::checkManifoldViolation(center, poly, closed);
}


// [QEM binary adjustment fix] 解码端 BasicReconstructOperator::reconstruct 会先
// 把 adjusted 顶点 setPoint(vertexPoint()) 再拆点，随后用还原后的环算预测。
// 编码端 residual 必须在同一状态上预测，否则 survivor 的 optimal placement
// 位移会平摊到每个被插入顶点上（QEM binary 实测 ~5e-4 的残差偏移）。
MCGAL::PointInt adjustedRestoreBarycenter(
    MCGAL::Facet* face, const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!face || !mesh) return MCGAL::PointInt(0, 0, 0);
    auto verts = MCGAL::rdCollectPolygonVerts(face);
    if (verts.empty()) return MCGAL::PointInt(0, 0, 0);
    int64_t sx = 0, sy = 0, sz = 0;
    int n = 0;
    for (MCGAL::Vertex* v : verts) {
        if (v == nullptr) continue;
        const MCGAL::Point p = v->isAdjusted() ? v->vertexPoint() : v->point();
        const MCGAL::PointInt pi = mesh->floatPosToInt(p);
        sx += pi.x();
        sy += pi.y();
        sz += pi.z();
        ++n;
    }
    if (n == 0) return MCGAL::PointInt(0, 0, 0);
    return MCGAL::PointInt(static_cast<int>(sx / n),
                           static_cast<int>(sy / n),
                           static_cast<int>(sz / n));
}
}  // namespace

void VertexRemovalEliminateOperator::init(std::shared_ptr<MCGAL::Mesh> mesh, PMSFHeader header, bool compress_boundary) {
    mesh_ = mesh;
    compress_boundary_ = compress_boundary;
    header_ = header;
}

bool VertexRemovalEliminateOperator::eliminate(HalfedgeErrorEntry& entry) {
    MCGAL::Halfedge*& h = entry.edge;
    if (h == nullptr)
        return false;
    if (std::getenv("PMSF_QUANT_DEGENERATE_GUARD")) {
        const char* maxRoundEnv = std::getenv("PMSF_QUANT_GUARD_MAX_ROUND");
        const int maxRound = maxRoundEnv ? std::atoi(maxRoundEnv) : INT_MAX;
        if (pmsf::currentCompressRoundForGuard <= maxRound &&
            quantizedRemovalWouldDegenerate(mesh_.get(), h))
            return false;
    }
    // if (compress_boundary_ && h->isBoundary()) {
    //     CHECK_TRUE(encode_boundary(h));
    //     return true;
    // }
    if (header_.isEnableAdjustment()) {
        h->vertex()->setAdjusted();
        h->vertex()->setVertexPoint(h->vertex()->point());
        h->vertex()->setPoint(entry.p);
    }
    CHECK_TRUE(!removalWouldViolateLinkCondition(h));
    CHECK_TRUE(remove_point(h));
    CHECK_TRUE(triangulate(h));
    if (std::getenv("PMSF_RESIDUAL_AFTER_TRIANG") && hasPendingRemovedPos_ &&
        header_.isEnablePrediction() && header_.isEnableQuantization() &&
        h && h->face() && h->face()->proxyHalfedge()) {
        MCGAL::Halfedge* patch = h->face()->proxyHalfedge();
        h->face()->setRemovedVertexPosInt(
            computeRemovedVertexResidualInt(h, patch, MCGAL::Point(), pendingRemovedPosInt_));
    }
    hasPendingRemovedPos_ = false;
    pendingRemovedVertex_ = nullptr;
    CHECK_TRUE(postprocess(h));
    entry.edge = h->face()->proxyHalfedge();
    // CHECK_TRUE(encode_boundary(h));
    return true;
}

int VertexRemovalEliminateOperator::computeRemovedValence(
    MCGAL::Halfedge* /*h*/,
    MCGAL::Vertex* removedVertex) {
    if (!removedVertex) return 0;
    return removedVertex->is_degree_dirty()
               ? removedVertex->vertex_degree()
               : removedVertex->vertex_degree_cached();
}

MCGAL::Vector3i VertexRemovalEliminateOperator::computeRemovedVertexResidualInt(
    MCGAL::Halfedge* /*h*/,
    MCGAL::Halfedge* hface,
    const MCGAL::Point& /*vrPoint*/,
    const MCGAL::PointInt& vrPos) {
    if (absRemovedPosEnabled()) {
        return MCGAL::Vector3i(vrPos.x(), vrPos.y(), vrPos.z());
    }
    MCGAL::PointInt predPos;
    if (header_.isEnableAdjustment()) {
        predPos = adjustedRestoreBarycenter(hface ? hface->face() : nullptr, mesh_);
    } else {
        predPos = MCGAL::rdBarycenterQuantized(hface->face(), mesh_);
    }
    MCGAL::Vector3i residual(vrPos.x() - predPos.x(),
                             vrPos.y() - predPos.y(),
                             vrPos.z() - predPos.z());
        return residual;
}

MCGAL::Point VertexRemovalEliminateOperator::computeStoredVertexPoint(
    MCGAL::Halfedge* /*h*/,
    MCGAL::Halfedge* hface,
    const MCGAL::Point& vrPoint) {
    if (header_.isEnablePrediction() && !header_.isEnableQuantization()) {
        MCGAL::Point pred = MCGAL::barycenter(hface->face()->proxyHalfedge());
        return MCGAL::Point(vrPoint.x() - pred.x(),
                            vrPoint.y() - pred.y(),
                            vrPoint.z() - pred.z());
    }
    return vrPoint;
}

void VertexRemovalEliminateOperator::recordRoundPatch(MCGAL::Facet* patchFace) {
    if (!patchFace) return;
    if (inCleaningPhase_) {
        roundCleaning_.push_back(patchFace);
    } else {
        roundDecimated_.push_back(patchFace);
    }
}

bool VertexRemovalEliminateOperator::remove_point(MCGAL::Halfedge* h) {
    if (!h)
        return false;
    MCGAL::Vertex* removedVertex = h->end_vertex();
    if (!removedVertex || removedVertex->isRemoved())
        return false;
    {
        auto& guard = pmsf::anchorGuardedVertices();
        if (!guard.empty() && guard.count(removedVertex))
            return false;
    }
    h->vertex()->setCollapsed();

    MCGAL::PointInt vrPos = mesh_->floatPosToInt(removedVertex->point());
    MCGAL::Point vrPoint = removedVertex->point();
    pendingRemovedPosInt_ = vrPos;
    pendingRemovedVertex_ = removedVertex;
    hasPendingRemovedPos_ = true;
    int removedValence = computeRemovedValence(h, removedVertex);

    if (const char* orderEnc = btpOrderEncPath()) {
        std::vector<int> fg;
        for (MCGAL::Halfedge* hit : pmsf::boundedRing(removedVertex)) {
            if (hit && hit->face() && !hit->face()->isRemoved()) fg.push_back(hit->face()->groupId());
        }
        std::set<int> u(fg.begin(), fg.end());
        if (FILE* of = std::fopen(orderEnc, "a")) {
            auto q = mesh_->floatPosToInt(removedVertex->point());
            std::fprintf(of, "%d %d %d %d %d\n", pmsf::boundaryTopologyRound(),
                         u.size() > 1 ? 1 : 0, q.x(), q.y(), q.z());
            std::fclose(of);
        }
    }

    if (boundaryTopologyStatsOn() ||
        pmsf::boundaryTopologyPatchEnabled()) {
        auto& st = pmsf::boundaryTopologyStats();
        ++st.totalRemovals;
        std::vector<int> fanGids;
        bool outer = false;
        for (MCGAL::Halfedge* hit : pmsf::boundedRing(removedVertex)) {
            if (!hit) continue;
            if (hit->isBoundary()) outer = true;
            if (hit->face() && !hit->face()->isRemoved()) fanGids.push_back(hit->face()->groupId());
        }
        if (outer) ++st.outerBoundaryRemovals;
        std::set<int> uniq(fanGids.begin(), fanGids.end());
        if (std::getenv("PMSF_BOUNDARY_OWNED_GIDS") && uniq.size() > 1) {
            const int owner = fanGids.front();
            for (MCGAL::Halfedge* hit : pmsf::boundedRing(removedVertex)) {
                if (!hit) continue;
                if (hit->face() && !hit->face()->isRemoved())
                    hit->face()->setGroupId(owner);
                if (hit->end_vertex() && !hit->end_vertex()->isRemoved())
                    hit->end_vertex()->setGroupId(owner);
            }
            removedVertex->setGroupId(owner);
        }
        if (uniq.size() > 1 && pmsf::boundaryTopologyPatchEnabled()) {
            pmsf::BoundaryTopologyOp op;
            op.round = pmsf::boundaryTopologyRound();
            op.q = mesh_->floatPosToInt(removedVertex->point());
            // Store gids sorted by canonical triangle key so the decoder can
            // reconstruct the same assignment without storing ring rotation.
            std::vector<std::pair<std::array<int, 9>, int>> keyed;
            for (MCGAL::Halfedge* hit : pmsf::boundedRing(removedVertex)) {
                if (!hit || !hit->face() || hit->face()->isRemoved()) continue;
                std::array<int, 9> key{};
                int n = 0;
                MCGAL::Halfedge* h0 = hit->face()->proxyHalfedge();
                MCGAL::Halfedge* hh = h0;
                int guard = 0;
                do {
                    if (hh && hh->vertex() && n < 3) {
                        auto q = mesh_->floatPosToInt(hh->vertex()->point());
                        key[static_cast<std::size_t>(n * 3)] = q.x();
                        key[static_cast<std::size_t>(n * 3 + 1)] = q.y();
                        key[static_cast<std::size_t>(n * 3 + 2)] = q.z();
                        ++n;
                    }
                    hh = hh ? hh->next() : nullptr;
                } while (hh && hh != h0 && ++guard < 64);
                if (n != 3) continue;
                std::array<std::array<int, 3>, 3> tri{};
                for (int i = 0; i < 3; ++i)
                    tri[static_cast<std::size_t>(i)] = {key[static_cast<std::size_t>(i * 3)],
                                                         key[static_cast<std::size_t>(i * 3 + 1)],
                                                         key[static_cast<std::size_t>(i * 3 + 2)]};
                std::sort(tri.begin(), tri.end());
                for (int i = 0; i < 3; ++i)
                    for (int j = 0; j < 3; ++j)
                        key[static_cast<std::size_t>(i * 3 + j)] = tri[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
                keyed.push_back({key, hit->face()->groupId()});
            }
            std::sort(keyed.begin(), keyed.end(),
                      [](const auto& a, const auto& b) { return a.first < b.first; });
            for (const auto& kv : keyed) op.gids.push_back(kv.second);
            pmsf::boundaryTopologyOps().push_back(std::move(op));
        }
        if (uniq.size() > 1) {
            ++st.mixedRemovals;
            st.mixedRingFaces += fanGids.size();
            std::uint64_t trans = 0;
            for (std::size_t i = 0; i < fanGids.size(); ++i) {
                if (i == 0 || fanGids[i] != fanGids[i - 1]) ++trans;
            }
            if (trans > 1) --trans;  // first run is the dominant gid
            st.mixedTransitions += trans;
            if (trans > st.maxTransitions) st.maxTransitions = trans;
        }
    }

    std::vector<MCGAL::Vertex*> ringVerts;
    if (shouldMaintainRingDegreeCache()) {
        ringVerts.reserve(8);
        for (MCGAL::Halfedge* hit : pmsf::boundedRing(removedVertex)) {
            if (hit && hit->opposite()) {
                MCGAL::Vertex* rv = hit->opposite()->vertex();
                if (rv && rv != removedVertex) {
                    ringVerts.push_back(rv);
                }
            }
        }
    }

    // ================= OWNED-BOUNDARY（env 门控，默认关闭）=================
    // 根因：释放分区边界后，被消的「边界顶点」其一环面片跨多个 groupId；
    // erase_center_vertex 把整环并进一个 owner 组，非 owner 组的面被销毁且解码端
    // 无法重建 => 回环失步。本块在**合面之前**把跨组一环中每个三角面的
    // (3 端点量化坐标, 原 gid) 记入 manifest 边信道；解码端据此把重建出的一环面
    // 改回原 gid，使非 owner 组的面得以被其自身符号流重建。
    // 本块只读/只写边信道，不改消除控制流 => 符号流逐字节不变。
    if (const char* manifestPath = ownedBoundaryManifestPath()) {
        {
            bool obClosed = true;
            std::vector<MCGAL::Facet*> obRing;
            std::set<int> obGids;
            for (MCGAL::Halfedge* hit : pmsf::boundedRing(removedVertex)) {
                if (!hit || !hit->face()) { obClosed = false; break; }
                obRing.push_back(hit->face());
                obGids.insert(hit->face()->groupId());
            }
            if (obClosed && obGids.size() > 1 && !obRing.empty()) {
                if (FILE* mf = std::fopen(manifestPath, "a")) {
                    for (MCGAL::Facet* f : obRing) {
                        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                        std::vector<std::array<int, 3>> vs;
                        MCGAL::Halfedge* h0 = f->proxyHalfedge();
                        MCGAL::Halfedge* hh = h0;
                        int guard = 0;
                        do {
                            if (!hh || !hh->vertex() || ++guard > 32) break;
                            MCGAL::PointInt p = mesh_->floatPosToInt(hh->vertex()->point());
                            vs.push_back({p.x(), p.y(), p.z()});
                            hh = hh->next();
                        } while (hh != nullptr && hh != h0);
                        if (vs.size() != 3) continue;
                        std::sort(vs.begin(), vs.end());
                        std::fprintf(mf, "%d %d %d %d %d %d %d %d %d %d\n",
                                     f->groupId(), vs[0][0], vs[0][1], vs[0][2],
                                     vs[1][0], vs[1][1], vs[1][2], vs[2][0], vs[2][1], vs[2][2]);
                    }
                    std::fclose(mf);
                }
            }
        }
    }

    // ================= THROWAWAY DIAG(env gated, default off, byte-neutral)====
    // For every CLOSED one-ring spanning >1 group, per ring triangle report
    // (origGid, ownerGid=min(ring), acrossGid = gid of the face across the
    // non-spoke edge (a,b)). Count whether origGid == acrossGid. Read-only,
    // no control-flow change => symbol stream byte-identical.
    
    // [PHASE61 物品1① **测量计数器**，默认关闭]
    //   - `closed`  扇区闭合（无边界半边）；`open` 触到基网边界。
    //   - 闭合的再分 `uniform`（环内面 gid 一致 ⇒ 附 34 的可逆条件）
    //     与 `mixed`（多 gid ⇒ 合面时**销毁** gid，`facesMixed` 就是被销毁的面数）。
    // 它回答的是：**「gid 一致性」这一个约束还挡着多少顶点、代价是多少次纠正**。
    // 只读计数器，不改变任何控制流 ⇒ 默认路径零字节变化。
    
    h->end_vertex()->setRemoved();
    MCGAL::Halfedge* hface = mesh_->erase_center_vertex(h);

    // [PHASE61 诊断，默认关闭] 被消顶点「环」记录 —— 编码端的洞识别**基准**。
    //
    // 解码端 `BasicReconstructOperator::insert_point` 会把自己认定的洞环写进
    // (PMSF_SEG_INS_DUMP dump 已移除)；但「认定得对不对」需要编码端同一时刻的环做基准。
    // 本信道输出 `erase_center_vertex` **之后、`triangulate` 之前**的多边形环
    // （正是解码端 insert 时要拆的那个），按**量化坐标**排序，因此跨进程可比。
    //
    // 键：环的量化坐标多重集；值：被消顶点的量化坐标（`->` 之后）。
    
    if (shouldMaintainRingDegreeCache()) {
        for (MCGAL::Vertex* rv : ringVerts) {
            if (rv && !rv->isRemoved() && !rv->is_degree_dirty()) {
                rv->decCachedDegree(1);
            }
        }
    }

    h->setFace(hface->face());
    hface->face()->setRemovedVertexPos(computeStoredVertexPoint(h, hface, vrPoint));
    hface->face()->setPolyValence(removedValence);
    hface->face()->setRemovedVertexPosInt(
        computeRemovedVertexResidualInt(h, hface, vrPoint, vrPos));
    pmsf::bndid::encNoteFacetRemovedVertex(hface->face(), removedVertex);

    recordRoundPatch(hface->face());
    return true;
}


bool VertexRemovalEliminateOperator::triangulate(MCGAL::Halfedge* h) {
    if (!h)
        return false;
    int groupId = h->face()->groupId();
    MCGAL::Facet* polyFace = h->face();
    // if (!polyFace || polyFace->isRemoved())
    //     return false;

    std::vector<MCGAL::Vertex*> polyVerts = polyFace->getVertices();
    std::vector<MCGAL::Halfedge*> polyHalfedges = polyFace->getHalfedges();
    int n = static_cast<int>(polyVerts.size());
        if (n < 4) {
        h->face()->setSplittable();
        return true;
    }
    h->face()->setSplittable();

    // ⚠️ 关于 enableTriangulation=false：
    //    最初的设计意图是让 VertexRemoval 完全跳过 fan-triangulation，产生 polygon 中间态。
    //    但这样做会破坏 Default 路径的关键不变式：
    //      (a) BasicIsRemovableOperator 用 `h->isAdded()` 判断"该顶点邻域最近是否
    //          因 split 生成过 halfedge"——跳过 fan 后这些 flag 消失，selector
    //          会反复选中 polygon 内部顶点做 `erase_center_vertex`，最终陷入死循环。
    //      (b) GreedySelector::select 依赖 `face.isConquered` / 顶点级标记来跳过已
    //          处理区域；若不 fan-triangulate，这些标记覆盖不到 polygon 内部顶点。
    //    PPMC 路径另走 `setProcessed()` / 二次 BFS，完全不同 plumbing。
    //
    //    因此我们保留 fan-triangulation（保障 mesh 拓扑不变式与历史行为等价），
    //    "enableTriangulation=false" 只影响 **encoder/decoder 对 edge sym 的
    //    second-pop + skip-both-unsplittable 规则**（见 OptimizedSymbolCollectOperator）。
    //    这是一个折中方案，BPV 收益来自跳过 unsplittable-unsplittable 内部边的 sym
    //    发射，而非来自 fan 消失。
    //
    //    TODO：后续如要进一步跟进 PPMC 的 vertexCut + setProcessed 方案，需要整套 port
    //    （见 impl/PPMC/PPMCColumnarCompress.cpp 的 pipeline 组装）。
    h = polyFace->proxyHalfedge();
    MCGAL::Halfedge* p1 = nullptr;
    MCGAL::Halfedge* p2 = nullptr;

    
    
    for (int i = 0; i < n - 2; ++i) {
        if (i == 0) {
            MCGAL::Halfedge* h0 = polyHalfedges[0];
            MCGAL::Halfedge* h1 = polyHalfedges[1];
            MCGAL::Vertex* v1 = polyVerts[2];
            MCGAL::Vertex* v2 = polyVerts[0];
            MCGAL::Halfedge* hnew = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(mesh_->meshId(), v1, v2);
            p1 = hnew;
            hnew->setAdded();
            h0->setNext(h1);
            h1->setNext(hnew);
            hnew->setNext(h0);
            polyFace->reset(h0);
        } else if (i == n - 3) {
            MCGAL::Halfedge* h0 = polyHalfedges[n - 2];
            MCGAL::Halfedge* h1 = polyHalfedges[n - 1];
            MCGAL::Vertex* v1 = polyVerts[0];
            MCGAL::Vertex* v2 = polyVerts[n - 2];
            MCGAL::Halfedge* hnew = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(mesh_->meshId(), v1, v2);
            // p2 = hnew;
            hnew->setOpposite(p1);
            p1->setOpposite(hnew);
            hnew->setAdded();
            h0->setNext(h1);
            h1->setNext(hnew);
            hnew->setNext(h0);
            MCGAL::Facet* newFace = MCGAL::contextPool.allocateFaceFromPool(mesh_->meshId(), h0);
            newFace->setGroupId(groupId);
            mesh_->add_face(newFace);
        } else {
            MCGAL::Vertex* v0 = polyVerts[0];
            MCGAL::Vertex* v1 = polyVerts[i + 1];
            MCGAL::Vertex* v2 = polyVerts[i + 2];
            MCGAL::Halfedge* h0 = polyHalfedges[i + 1];
            MCGAL::Halfedge* h1 = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(mesh_->meshId(), v2, v0);
            MCGAL::Halfedge* h2 = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(mesh_->meshId(), v0, v1);
            // h1->setOpposite(h2);
            h2->setOpposite(p1);
            p1->setOpposite(h2);
            p1 = h1;

            h1->setAdded();
            h2->setAdded();
            h0->setNext(h1);
            h1->setNext(h2);
            h2->setNext(h0);
            MCGAL::Facet* newFace = MCGAL::contextPool.allocateFaceFromPool(mesh_->meshId(), h0);
            newFace->setGroupId(groupId);
            mesh_->add_face(newFace);
        }
    }
    if (polyFace && !polyFace->isRemoved()) {
        auto pv = polyFace->getVertices();
        if (pv.size() == 3 && triDropDupPolyEnabled()) {
            std::array<MCGAL::Vertex*, 3> want{pv[0], pv[1], pv[2]};
            std::sort(want.begin(), want.end());
            for (MCGAL::Facet* f : mesh_->faces()) {
                if (!f || f->isRemoved() || !f->proxyHalfedge() || f == polyFace) continue;
                auto fv = f->getVertices();
                if (fv.size() != 3) continue;
                std::array<MCGAL::Vertex*, 3> have{fv[0], fv[1], fv[2]};
                std::sort(have.begin(), have.end());
                if (have == want) {
                    pmsf::removeFaceForPatch(mesh_, f);
                                        break;
                }
            }
        }
    }
    // p1->setOpposite(p2);
    // p2->setOpposite(p1);

    // 度数增量维护：fan-triangulation 的对角线连接 v0 和 v_2 .. v_{n-2}，共 n-3 条对角线。
    // v0 获得 (n-3) 条新边；v_2 .. v_{n-2} 各获得 1 条新边；v_1 与 v_{n-1} 不变。
    // 每条对角线是一对半边，但只表示一条无向边，因此两端点 degree 各 +1。
    if (n >= 4) {
        MCGAL::Vertex* v0 = polyVerts[0];
        if (v0 && !v0->isRemoved() && !v0->is_degree_dirty()) {
            v0->incCachedDegree(n - 3);
        }
        for (int i = 2; i <= n - 2; i++) {
            MCGAL::Vertex* vi = polyVerts[i];
            if (vi && !vi->isRemoved() && !vi->is_degree_dirty()) {
                vi->incCachedDegree(1);
            }
        }
    }

    return true;
}

bool VertexRemovalEliminateOperator::postprocess(MCGAL::Halfedge* h) {
    return true;
}

static void setNewGroupId(MCGAL::Facet* f) {
    for (auto it = f->halfedges_begin(); it != f->halfedges_end(); it++) {
        if (!(*it)->isBoundary()) {
            f->setGroupId((*it)->face()->groupId());
            break;
        }
    }
}

/**
 * 后面需要单独验证是否正确
 */
// 问题：如果下下个边界在多边形上，就会导致边界出错
MCGAL::Halfedge* VertexRemovalEliminateOperator::encode_boundary(MCGAL::Halfedge* h) {
    // if (!compress_boundary_ || !h->isBoundary()) {
    //     return true;
    // }
    MCGAL::Halfedge* boundary = h;
    MCGAL::Halfedge* nxtBoundary = h->next_boundary();
    MCGAL::Vertex* v1 = h->vertex();
    MCGAL::Vertex* v2 = nxtBoundary->end_vertex();
    int gid1 = h->face()->groupId();
    int gid2 = h->opposite()->face()->groupId();
    remove_point(h);
    // 最重要的是锚定两个点，锚定住出点
    // 如何锚定，先把中心点删除，然后找出点，记住出点和记录点不一样
    //
    MCGAL::Facet* newFace = h->face();
    MCGAL::Halfedge* newBoundary = nullptr;
    for (MCGAL::Halfedge* hit : pmsf::boundedRing(v1)) {
        if (hit->face() == newFace) {
            newBoundary = hit;
            break;
        }
    }
    while (true) {
        newBoundary->setBoundary();
        if (newBoundary->end_vertex() == v2) {
            break;
        }
        newBoundary = newBoundary->next();
    }
    MCGAL::Halfedge* st = newFace->proxyHalfedge();
    MCGAL::Halfedge* ed = st;
    do {
        if (!st->isBoundary()) {
            groupId = st->opposite()->face()->groupId();
            break;
        }
        st = st->next();
    } while (st != ed);
    h->face()->setGroupId(groupId);
    triangulate(h);
    return nullptr;
    // MCGAL::Halfedge* newBoundary = nullptr;
    // for (MCGAL::Halfedge* hit : v1->halfedges()) {
    //     if (hit->end_vertex() == v2) {
    //         newBoundary = hit;
    //         break;
    //     }
    // }
    // if (newBoundary->isAdded()) {
    //     newBoundary->setBoundary();
    //     newBoundary->face()->setSplittable();
    //     newBoundary->face()->setRemovedVertexPos(h->end_vertex());
    //     newBoundary->opposite()->face()->setSplittable();
    //     newBoundary->opposite()->face()->setRemovedVertexPos(h->end_vertex());
    //     // setNewGroupId(newBoundary->face());
    //     // setNewGroupId(newBoundary->opposite()->face());
    // } else {
    //     MCGAL::Facet* f = newBoundary->face();
    //     for (auto it = f->halfedges_begin(); it != f->halfedges_end(); it++) {
    //         if((*it)->face()->groupId() != (*it)->opposite()->face()->groupId()) {
    //             (*it)->setBoundary();
    //             (*it)->opposite()->setBoundary();
    //         }
    //     }
    // }
    // return true;
}

// ============================================================================
// Phase 0 · Step 0.4 —— round-list hook 实现。
// 对 baseline binary / Columnar 的回归门禁零影响：旧 collector 不调用这四个
// 方法，因此即便 roundDecimated_ / roundCleaning_ 累积再多也不会进入任何
// 字节流。
// ============================================================================

std::vector<MCGAL::Facet*> VertexRemovalEliminateOperator::takeRoundDecimatedPatches() {
    std::vector<MCGAL::Facet*> out;
    out.swap(roundDecimated_);
    return out;
}

std::vector<MCGAL::Facet*> VertexRemovalEliminateOperator::takeRoundCleaningPatches() {
    std::vector<MCGAL::Facet*> out;
    out.swap(roundCleaning_);
    return out;
}

void VertexRemovalEliminateOperator::resetRoundAccumulators() {
    roundDecimated_.clear();
    roundCleaning_.clear();
    // inCleaningPhase_ **不**在这里复位：每轮内部可能由 caller 多次翻转
    // （先 decim，后 cleaning）。caller 负责对 setInCleaningPhase 的生命期管理。
}

void VertexRemovalEliminateOperator::setInCleaningPhase(bool inCleaning) {
    inCleaningPhase_ = inCleaning;
}
