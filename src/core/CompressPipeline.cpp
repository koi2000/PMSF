/*
 * @file CompressPipeline.cpp
 * @brief pmsf::runCompress / runDecompress 实现。
 *
 * 重要：本文件是 Phase 1 重构的**唯一行为聚合点**。每一步的顺序、判断条件
 * 必须与 src/PMSF.cpp 老 xxxCompress / xxxDecompress 方法的"并集"完全一致，
 * 否则绿色格子的 BPV / Hausdorff 会变 —— 参见 docs/refactor/baseline/README.md。
 */
#include "core/CompressPipeline.h"
#include "boundary/BoundaryCompressionModule.h"
#include "boundary/BoundaryCompactStorage.h"
#include "boundary/BoundaryTopologyStats.h"
#include "boundary/BoundaryIdRegistry.h"
#include "boundary/BndCompactCodec.h"
#include "boundary/BoundaryTopologyPatch.h"
#include "boundary/BoundaryTopologyPatchCodec.h"
#include "common/BfsGroupLabel.h"
#include "common/GroupAnchorTraversal.h"
#include "common/BoundedRingTraversal.h"
#include "core/MeshRuntime.h"
#include "segmentation/SegmentOperator.h"
#include "common/Graph.h"
#include "boundary/SeamCutOperator.h"
#include "boundary/SingleBoundaryCollapse.h"
#include "boundary/SingleCopyBoundaryStage.h"
#include "boundary/SingleCopyBoundaryCompact.h"
#include "boundary/SingleCopyBoundaryAnchor.h"
#include "common/MeshSplitUtils.h"  // P18.2 阶段 A：分区阶段（此前框架完全没有这一环）
#include "common/BoundaryPatchCodec.h"

#include <chrono>
#include <csignal>
#include <cstdlib>

static bool pmsfDumpRoundSelected(const char* env, int round) {
    if (!env || !env[0]) return true;
    const std::string s(env);
    std::size_t p = 0;
    while (p <= s.size()) {
        const std::size_t comma = s.find(',', p);
        const std::string tok = s.substr(p, comma == std::string::npos ? std::string::npos : comma - p);
        if (!tok.empty() && std::atoi(tok.c_str()) == round) return true;
        if (comma == std::string::npos) break;
        p = comma + 1;
    }
    return false;
}


namespace pmsf {
namespace {
inline MCGAL::Halfedge* findCanonicalGroupSeedLocal(MCGAL::Mesh* mesh, int groupId);
inline MCGAL::Halfedge* findLiveSeedBetweenLocal(MCGAL::Mesh* mesh,
                                                MCGAL::Vertex* v0,
                                                MCGAL::Vertex* v1,
                                                int groupId);
}  // namespace
}  // namespace pmsf
#include <algorithm>
#include <array>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Mesh.h"

#include "ContextPool.h"
#include "codebook/Codebook.h"
#include "common/HalfedgeErrorEntry.h"
#include "compress/BasicSerializeOperator.h"
#include "compress/OptimizedSymbolCollectOperator.h"
#include "core/HookBridge.h"
#include "elimination/EliminateOperator.h"
#include "entropy/EntropyCoder.h"
#include "feature/FeatureDetector.h"
#include "metric/ErrorMetric.h"
#include "postprocess/EdgeFlipPostProcessOperator.h"
#include "postprocess/EdgeFlipCandidateBfs.h"
#include "common/BoundaryOrderUtils.h"
#include "common/GroupBoundaryEnumeration.h"
#include "elimination/BoundaryVertexRemovalEliminateOperator.h"
#include "postprocess/PaperPostProcessOperator.h"
#include "operator/GroupFootholdIsRemovableOperator.h"  // PHASE61 物品 1 ①
#include "operator/BoundaryIsRemovableOperator.h"  // shared boundary freeze set


#include "operator/IsRemovableOperator.h"
#include "operator/IsFlipOperator.h"
#include "operator/SeedIsRemovableOperator.h"
#include "operator/VisualTopologyIsRemovableOperator.h"
#include "predict/Predictor.h"
#include "selection/GreedySelector.h"
#include "selection/IndependentSetSelector.h"
#include "selection/SelectOperator.h"

#ifdef COLUMNAR_SUPPORT
#    include "compress/ColumnarSerializeOperator.h"
#    include "compress/PaperSerializeOperator.h"
#    include "compress/UnifiedMultiTableSerializeOperator.h"
#endif

namespace pmsf {

int currentCompressRoundForGuard = 0;

FILE* bfsDumpFile(const char* envValue) {
    static FILE* fp = nullptr;
    if (!fp && envValue && envValue[0]) fp = std::fopen(envValue, "w");
    return fp;
}

namespace {

bool envOn(const char* name) {
    const char* v = std::getenv(name);
    return v && *v && std::string(v) != "0";
}

int envInt(const char* name, int fallback) {
    const char* v = std::getenv(name);
    if (!v || !*v) return fallback;
    const int x = std::atoi(v);
    return x > 0 ? x : fallback;
}

/// Like envInt but honours explicit 0 (used where 0 is a meaningful "disable" value;
/// envInt would silently fall back and made earlier zero-op diagnostics invalid).
int envIntAllowZero(const char* name, int fallback) {
    const char* v = std::getenv(name);
    if (!v || !*v) return fallback;
    return std::atoi(v);
}

/// Per-round boundary maxOps: PMSF_SC_BOUNDARY_MAXOPS_LIST is a comma list; a value
/// >= 0 overrides the global PMSF_SC_BOUNDARY_MAXOPS for that round.  Used to give
/// early rounds a large boundary budget while keeping late rounds conservative.
// Adaptive boundary budgets (no user-visible K): a per-round cap vector produced by
// the verification-driven search (scripts/tests/adaptive_search.py).  The vector is
// consumed here as the topology-candidate upper bound; the topology still decides
// whether any candidate exists each round.  Artifacts self-describe via nOps.
inline const std::vector<int>* adaptiveBoundaryBudgets() {
    static thread_local std::vector<int> budgets;
    static thread_local bool loaded = false;
    if (loaded) return budgets.empty() ? nullptr : &budgets;
    loaded = true;
    std::string text;
    if (const char* f = std::getenv("PMSF_SC_BOUNDARY_BUDGET_FILE")) {
        std::ifstream in(f);
        if (in) {
            std::stringstream ss;
            ss << in.rdbuf();
            text = ss.str();
        }
    } else if (const char* j = std::getenv("PMSF_SC_BOUNDARY_ADAPTIVE_BUDGETS")) {
        text = j;
    }
    if (text.empty()) return nullptr;
    // Accept either a JSON object {"budgets":[...]} or a bare comma list.
    std::size_t lb = text.find('[');
    std::size_t rb = text.find(']', lb == std::string::npos ? 0 : lb);
    if (lb != std::string::npos && rb != std::string::npos && rb > lb) {
        std::string seq = text.substr(lb + 1, rb - lb - 1);
        std::stringstream ss(seq);
        std::string tok;
        while (std::getline(ss, tok, ',')) {
            std::size_t a = tok.find_first_not_of(" \t\n\r");
            if (a == std::string::npos) continue;
            std::size_t b = tok.find_last_not_of(" \t\n\r");
            budgets.push_back(std::atoi(tok.substr(a, b - a + 1).c_str()));
        }
    }
    return budgets.empty() ? nullptr : &budgets;
}

int boundaryMaxOpsForRound(int round, int fallback = 8) {
    if (const std::vector<int>* ab = adaptiveBoundaryBudgets()) {
        if (round >= 0 && round < static_cast<int>(ab->size()))
            return std::max(0, (*ab)[round]);
    }
    // K-free mode: the topology decides how many boundary collapses are safe/possible
    // in a round; the stage loop stops by itself when no eligible candidate remains.
    if (std::getenv("PMSF_SC_BOUNDARY_KFREE")) {
        const int cap = envIntAllowZero("PMSF_SC_BOUNDARY_KFREE_CAP", 100000);
        return cap > 0 ? cap : 100000;
    }
    const int global = envIntAllowZero("PMSF_SC_BOUNDARY_MAXOPS", fallback);
    const char* v = std::getenv("PMSF_SC_BOUNDARY_MAXOPS_LIST");
    if (!v || !*v || round < 0) return global;
    const std::string rs(v);
    int idx = 0;
    std::size_t p = 0;
    while (p <= rs.size()) {
        const std::size_t comma = rs.find(',', p);
        const std::string tok =
            rs.substr(p, comma == std::string::npos ? std::string::npos : comma - p);
        if (idx == round) {
            if (!tok.empty() && tok != "-") {
                const int x = std::atoi(tok.c_str());
                if (x >= 0) return x;
            }
            return global;
        }
        if (comma == std::string::npos) break;
        p = comma + 1;
        ++idx;
    }
    return global;
}

double triangleArea2(const MCGAL::Point& a, const MCGAL::Point& b, const MCGAL::Point& c) {
    const double ux = static_cast<double>(b.x()) - a.x();
    const double uy = static_cast<double>(b.y()) - a.y();
    const double uz = static_cast<double>(b.z()) - a.z();
    const double vx = static_cast<double>(c.x()) - a.x();
    const double vy = static_cast<double>(c.y()) - a.y();
    const double vz = static_cast<double>(c.z()) - a.z();
    const double cx = uy * vz - uz * vy;
    const double cy = uz * vx - ux * vz;
    const double cz = ux * vy - uy * vx;
    return cx * cx + cy * cy + cz * cz;
}

int repairDegenerateTriangleGeometry(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!mesh) return 0;
    const double area2Threshold = 1e-30;
    const float epsFactor = std::getenv("PMSF_REPAIR_EPS_FACTOR")
                               ? std::max(1e-9f, static_cast<float>(std::atof(std::getenv("PMSF_REPAIR_EPS_FACTOR"))))
                               : 0.01f;
    const float epsMin = std::getenv("PMSF_REPAIR_EPS_MIN")
                             ? static_cast<float>(std::atof(std::getenv("PMSF_REPAIR_EPS_MIN")))
                             : 1e-8f;
    const float eps = std::max(mesh->f_quantStep * epsFactor, epsMin);
    int totalRepaired = 0;
    for (int pass = 0; pass < 8; ++pass) {
        int repairedThisPass = 0;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved()) continue;
            std::vector<MCGAL::Vertex*> vs = f->getVertices();
            if (vs.size() < 3 || !vs[0]) continue;
            MCGAL::Vertex* moveVertex = nullptr;
            MCGAL::Point movePoint;
            MCGAL::Point edgeA;
            MCGAL::Point edgeB;
            for (std::size_t i = 1; i + 1 < vs.size(); ++i) {
                if (!vs[i] || !vs[i + 1]) continue;
                const MCGAL::Point p0 = vs[0]->point();
                const MCGAL::Point p1 = vs[i]->point();
                const MCGAL::Point p2 = vs[i + 1]->point();
                if (triangleArea2(p0, p1, p2) > area2Threshold) continue;

                auto dist2 = [](const MCGAL::Point& a, const MCGAL::Point& b) {
                    const double dx = static_cast<double>(a.x()) - b.x();
                    const double dy = static_cast<double>(a.y()) - b.y();
                    const double dz = static_cast<double>(a.z()) - b.z();
                    return dx * dx + dy * dy + dz * dz;
                };
                moveVertex = vs[i + 1];
                movePoint = p2;
                edgeA = p0;
                edgeB = p1;
                constexpr double zeroEdgeThreshold = 1e-24;
                if (dist2(p0, p1) <= zeroEdgeThreshold) {
                    moveVertex = vs[i];
                    movePoint = p1;
                    edgeA = p0;
                    edgeB = p2;
                } else if (dist2(p1, p2) <= zeroEdgeThreshold) {
                    moveVertex = vs[i + 1];
                    movePoint = p2;
                    edgeA = p1;
                    edgeB = p0;
                } else if (dist2(p2, p0) <= zeroEdgeThreshold) {
                    moveVertex = vs[0];
                    movePoint = p0;
                    edgeA = p2;
                    edgeB = p1;
                }
                break;
            }
            if (!moveVertex) continue;

            const double ex = static_cast<double>(edgeB.x()) - edgeA.x();
            const double ey = static_cast<double>(edgeB.y()) - edgeA.y();
            const double ez = static_cast<double>(edgeB.z()) - edgeA.z();
            double dx = 0.0;
            double dy = -ez;
            double dz = ey;
            if (dx * dx + dy * dy + dz * dz <= 1e-30) {
                dx = -ey;
                dy = ex;
                dz = 0.0;
            }
            double len = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (len <= 1e-30) {
                dx = 1.0;
                dy = 0.0;
                dz = 0.0;
                len = 1.0;
            }
            const float scale = eps * static_cast<float>(pass + 1);
            moveVertex->setPoint(MCGAL::Point(
                movePoint.x() + static_cast<float>(scale * dx / len),
                movePoint.y() + static_cast<float>(scale * dy / len),
                movePoint.z() + static_cast<float>(scale * dz / len)));
            ++repairedThisPass;
        }
        totalRepaired += repairedThisPass;
        if (repairedThisPass == 0) break;
    }
    return totalRepaired;
}

int removedCenterValence(MCGAL::Halfedge* h) {
    if (!h || !h->next() || !h->next()->opposite()) return 0;
    int degree = 1;
    MCGAL::Halfedge* g = h->next()->opposite();
    int guard = 0;
    while (g && g != h) {
        ++degree;
        if (!g->next() || !g->next()->opposite()) return 0;
        g = g->next()->opposite();
        if (++guard > 128) return 0;
    }
    return degree;
}

int liveVertexDegree(MCGAL::Vertex* v) {
    if (!v || v->isRemoved()) return 0;
    return v->is_degree_dirty() ? v->vertex_degree() : v->vertex_degree_cached();
}

int candidateNeighborCount(MCGAL::Vertex* v, const std::unordered_set<int>& candidates) {
    if (!v) return 0;
    int n = 0;
    for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
        if (!h || h->isRemoved()) continue;
        MCGAL::Vertex* nb = h->end_vertex();
        if (nb && !nb->isRemoved() && candidates.count(nb->poolId())) ++n;
    }
    return n;
}

void assignBfsLocalVertexIds(const std::shared_ptr<MCGAL::Mesh>& mesh,
                             MCGAL::Halfedge* seed) {
    if (!mesh) return;
    (void)seed;
    int nextId = 0;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        v->setVid(nextId++);
    }
}

std::array<long long, 6> canonicalHalfedgeKey(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                              MCGAL::Halfedge* h) {
    auto vertexKey = [&](MCGAL::Vertex* v) -> std::array<long long, 3> {
        if (!v) return {LLONG_MIN, LLONG_MIN, LLONG_MIN};
        if (mesh && mesh->enableQuantization) {
            MCGAL::PointInt q = mesh->floatPosToInt(v->point());
            return {static_cast<long long>(q[0]),
                    static_cast<long long>(q[1]),
                    static_cast<long long>(q[2])};
        }
        constexpr double kScale = 100000000.0;
        return {static_cast<long long>(std::llround(static_cast<double>(v->x()) * kScale)),
                static_cast<long long>(std::llround(static_cast<double>(v->y()) * kScale)),
                static_cast<long long>(std::llround(static_cast<double>(v->z()) * kScale))};
    };
    const auto a = vertexKey(h ? h->vertex() : nullptr);
    const auto b = vertexKey(h ? h->end_vertex() : nullptr);
    return {a[0], a[1], a[2], b[0], b[1], b[2]};
}

void canonicalizeTraversalProxies(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!mesh) return;
    // [诊断] PMSF_SKIP_CANON=1 整段跳过，用于验证某算法是否依赖 proxy canonical 化。
    static const bool skipAll = (std::getenv("PMSF_SKIP_CANON") != nullptr);
    if (skipAll) return;
    // perf（2026-11-09 QEM 实测）：某些算法的码流不依赖 vertex proxy 的 canonical
    // 化（QEM 由 compressOptionsTuner 声明 PMSF_SKIP_VERTEX_CANON=1；PPMC 沿用旧名）。
    // 面 proxy 仍然 canonicalize（跳过会改变 BFS 起点/码流，已验证 QEM sha 变化）。
    const bool skipVertCanon = [] {
        const char* e = std::getenv("PMSF_SKIP_VERTEX_CANON");
        if (e == nullptr) e = std::getenv("PMSF_PPMC_NO_VERTEX_CANON");
        return e && *e && *e != '0';
    }();

    // 旧路径（非量化 / 量化坐标超出打包范围时回退）：与历史实现逐字节等价。
    auto canonicalizeLegacy = [&]() {
        const std::size_t nv = mesh->size_of_vertices();
        static const std::array<long long, 3> kNullKey{LLONG_MIN, LLONG_MIN, LLONG_MIN};
        std::vector<std::array<long long, 3>> vkey(nv, kNullKey);
        if (mesh->enableQuantization) {
            for (MCGAL::Vertex* v : mesh->vertices()) {
                if (!v || v->isRemoved()) continue;
                const std::size_t id = static_cast<std::size_t>(v->poolId());
                if (id >= nv) continue;
                MCGAL::PointInt q = mesh->floatPosToInt(v->point());
                vkey[id] = {static_cast<long long>(q[0]),
                            static_cast<long long>(q[1]),
                            static_cast<long long>(q[2])};
            }
        } else {
            constexpr double kScale = 100000000.0;
            for (MCGAL::Vertex* v : mesh->vertices()) {
                if (!v || v->isRemoved()) continue;
                const std::size_t id = static_cast<std::size_t>(v->poolId());
                if (id >= nv) continue;
                vkey[id] = {
                    static_cast<long long>(std::llround(static_cast<double>(v->x()) * kScale)),
                    static_cast<long long>(std::llround(static_cast<double>(v->y()) * kScale)),
                    static_cast<long long>(std::llround(static_cast<double>(v->z()) * kScale))};
            }
        }
        auto vertexKeyOf = [&](MCGAL::Vertex* v) -> const std::array<long long, 3>& {
            if (!v) return kNullKey;
            const std::size_t id = static_cast<std::size_t>(v->poolId());
            return (id < nv) ? vkey[id] : kNullKey;
        };
        auto halfedgeKeyLess = [&](MCGAL::Halfedge* lhs, MCGAL::Halfedge* rhs) {
            const auto& l0 = vertexKeyOf(lhs ? lhs->vertex() : nullptr);
            const auto& r0 = vertexKeyOf(rhs ? rhs->vertex() : nullptr);
            for (int i = 0; i < 3; ++i) {
                if (l0[i] != r0[i]) return l0[i] < r0[i];
            }
            const auto& l1 = vertexKeyOf(lhs ? lhs->end_vertex() : nullptr);
            const auto& r1 = vertexKeyOf(rhs ? rhs->end_vertex() : nullptr);
            for (int i = 0; i < 3; ++i) {
                if (l1[i] != r1[i]) return l1[i] < r1[i];
            }
            return false;
        };
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved()) continue;
            MCGAL::Halfedge* best = nullptr;
            for (MCGAL::Halfedge* h : *f) {
                if (!h || h->isRemoved() || !h->vertex() || h->vertex()->isRemoved()) continue;
                if (!best || halfedgeKeyLess(best, h)) {
                    best = h;
                }
            }
            if (best) f->setProxyHalfedge(best);
        }
        if (!skipVertCanon) {
            for (MCGAL::Vertex* v : mesh->vertices()) {
                if (!v || v->isRemoved()) continue;
                MCGAL::Halfedge* best = nullptr;
                for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
                    if (!h || h->isRemoved() || h->vertex() != v ||
                        !h->end_vertex() || h->end_vertex()->isRemoved()) {
                        continue;
                    }
                    if (!best || halfedgeKeyLess(best, h)) {
                        best = h;
                    }
                }
                if (best) v->setProxyHalfedge(best);
            }
        }
    };

    if (!mesh->enableQuantization) {
        canonicalizeLegacy();
        return;
    }

    // perf（2026-11-08 bunny 实测 canonicalize 25.7 ms / 11.8% cycles）：量化 key
    // 打包成 64-bit（3×17 位 + 1 偏移，0 保留给“缺失/已删除”哨兵，语义等价于原
    // 实现的 LLONG_MIN 最小 key）。face/vertex 环比较从“6 个 long long 字典序”
    // 降为“两次 uint64 比较”，vkey 内存 24B→8B；坐标超出 17 位范围时整体回退旧路径。
    const std::size_t nv = mesh->size_of_vertices();
    std::vector<unsigned long long> vpack(nv, 0ULL);
    bool inRange = true;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        const std::size_t id = static_cast<std::size_t>(v->poolId());
        if (id >= nv) continue;
        const MCGAL::PointInt q = mesh->floatPosToInt(v->point());
        if (q[0] < 0 || q[1] < 0 || q[2] < 0 ||
            q[0] > 131071 || q[1] > 131071 || q[2] > 131071) {
            inRange = false;
            break;
        }
        vpack[id] = 1ULL + ((static_cast<unsigned long long>(q[0]) << 34) |
                            (static_cast<unsigned long long>(q[1]) << 17) |
                            static_cast<unsigned long long>(q[2]));
    }
    if (!inRange) {
        canonicalizeLegacy();
        return;
    }
    auto packOf = [&](MCGAL::Vertex* v) -> unsigned long long {
        if (!v) return 0ULL;
        const std::size_t id = static_cast<std::size_t>(v->poolId());
        return (id < nv) ? vpack[id] : 0ULL;
    };
    auto halfedgeKeyLess = [&](MCGAL::Halfedge* lhs, MCGAL::Halfedge* rhs) {
        const unsigned long long l0 = packOf(lhs ? lhs->vertex() : nullptr);
        const unsigned long long r0 = packOf(rhs ? rhs->vertex() : nullptr);
        if (l0 != r0) return l0 < r0;
        return packOf(lhs ? lhs->end_vertex() : nullptr) <
               packOf(rhs ? rhs->end_vertex() : nullptr);
    };
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        MCGAL::Halfedge* best = nullptr;
        for (MCGAL::Halfedge* h : *f) {
            if (!h || h->isRemoved() || !h->vertex() || h->vertex()->isRemoved()) continue;
            if (!best || halfedgeKeyLess(best, h)) {
                best = h;
            }
        }
        if (best) f->setProxyHalfedge(best);
    }
    if (!skipVertCanon) {
        for (MCGAL::Vertex* v : mesh->vertices()) {
            if (!v || v->isRemoved()) continue;
            MCGAL::Halfedge* best = nullptr;
            for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
                if (!h || h->isRemoved() || h->vertex() != v ||
                    !h->end_vertex() || h->end_vertex()->isRemoved()) {
                    continue;
                }
                if (!best || halfedgeKeyLess(best, h)) {
                    best = h;
                }
            }
            if (best) v->setProxyHalfedge(best);
        }
    }
}
class EdgeFlipProtectedPatchIsRemovableOperator final : public IsRemovableOperator {
  public:
    explicit EdgeFlipProtectedPatchIsRemovableOperator(
        std::shared_ptr<std::unordered_set<int>> protectedVertexIds)
        : protectedVertexIds_(std::move(protectedVertexIds)) {}

    bool isRemovable(HalfedgeErrorEntry& entry) override {
        MCGAL::Halfedge* h = entry.edge;
        if (!h || !protectedVertexIds_ || protectedVertexIds_->empty()) return true;
        return !touchesProtectedOneRing(h->end_vertex()) &&
               !touchesProtectedOneRing(h->vertex());
    }

  private:
    bool isProtected(MCGAL::Vertex* v) const {
        return v && !v->isRemoved() &&
               protectedVertexIds_->count(v->poolId()) > 0;
    }

    bool touchesProtectedOneRing(MCGAL::Vertex* v) const {
        if (!v || v->isRemoved()) return false;
        if (isProtected(v)) return true;
        for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
            if (!h || h->isRemoved()) continue;
            if (isProtected(h->end_vertex())) return true;
        }
        return false;
    }

    std::shared_ptr<std::unordered_set<int>> protectedVertexIds_;
};

}  // namespace

// -----------------------------------------------------------------------
// 预置工厂（仅供 PMSF.cpp 里 default 类算法复用）
// -----------------------------------------------------------------------

CompressPipelineConfig CompressPipelineConfig::defaultBinary() {
    CompressPipelineConfig cfg;
    cfg.collectorFactory = []() -> std::shared_ptr<SymbolCollectOperator> {
        return std::make_shared<OptimizedSymbolCollectOperator>();
    };
    cfg.serializerFactory = [](std::shared_ptr<SymbolCollectOperator> c,
                               const CompressOptions& o) -> std::shared_ptr<SerializeOperator> {
        return std::make_shared<BasicSerializeOperator>(
            c, o.isEnablePrediction(), o.isEnableQuantization(), o.isEnableCompress());
    };
    cfg.seedStrategy = CompressPipelineConfig::SeedStrategy::SelectSeed;
    // 历史默认：多数算法（default / qem / parquetV2 / ppmc）在每次 eliminate 成功后
    // 调用 select->update(entry)。只有 PCATM / RD / APTM / MMSE / PCLTTM 的老实现里
    // 这行被注释掉。为最大限度与老代码的字节级对齐，默认打开 update；不需要的算法
    // 在 cfg 中显式关闭即可。
    cfg.updateAfterEliminate = true;
    cfg.serializerNeedsExplicitHeader = true;
    cfg.serializerWantsSharedHeader = false;
    return cfg;
}

CompressPipelineConfig CompressPipelineConfig::defaultColumnar() {
    CompressPipelineConfig cfg;
    cfg.collectorFactory = []() -> std::shared_ptr<SymbolCollectOperator> {
        return std::make_shared<OptimizedSymbolCollectOperator>();
    };
#ifdef COLUMNAR_SUPPORT
    cfg.serializerFactory = [](std::shared_ptr<SymbolCollectOperator> c,
                               const CompressOptions& o) -> std::shared_ptr<SerializeOperator> {
        return std::make_shared<ColumnarSerializeOperator>(
            c, o.isEnablePrediction(), o.isEnableQuantization(), o.isEnableCompress());
    };
#else
    cfg.serializerFactory = [](std::shared_ptr<SymbolCollectOperator>,
                               const CompressOptions&) -> std::shared_ptr<SerializeOperator> {
        throw std::runtime_error("Columnar serializer requested but COLUMNAR_SUPPORT not defined");
    };
#endif
    cfg.seedStrategy = CompressPipelineConfig::SeedStrategy::SelectSeed;
    cfg.updateAfterEliminate = true;  // 与 defaultBinary 对齐
    cfg.serializerNeedsExplicitHeader = false;
    cfg.serializerWantsSharedHeader = true;
    return cfg;
}

CompressPipelineConfig CompressPipelineConfig::defaultPaper(const std::string& algorithmName) {
    CompressPipelineConfig cfg = defaultColumnar();
#ifdef COLUMNAR_SUPPORT
    cfg.serializerFactory = [algorithmName](std::shared_ptr<SymbolCollectOperator> c,
                                            const CompressOptions& o) -> std::shared_ptr<SerializeOperator> {
        return std::make_shared<PaperSerializeOperator>(
            c, o.isEnablePrediction(), o.isEnableQuantization(), o.isEnableCompress(),
            algorithmName);
    };
#endif
    return cfg;
}

CompressPipelineConfig CompressPipelineConfig::defaultUnified() {
    // V3 与 V2 的 cfg 差异仅在 serializerFactory —— V3 替换为
    // UnifiedMultiTableSerializeOperator。V3 writer 继承 V2，默认情况下
    // 若未注册任何 side channel / blob，产出物与 V2 的 4 表字节级兼容
    // （仅文件名后缀 `_v3_` 不同）。
    CompressPipelineConfig cfg = defaultColumnar();
#ifdef COLUMNAR_SUPPORT
    cfg.serializerFactory = [](std::shared_ptr<SymbolCollectOperator> c,
                               const CompressOptions& o) -> std::shared_ptr<SerializeOperator> {
        return std::make_shared<UnifiedMultiTableSerializeOperator>(
            c, o.isEnablePrediction(), o.isEnableQuantization(), o.isEnableCompress());
    };
#endif
    return cfg;
}

// -----------------------------------------------------------------------
// 路线 (a) 第二遍：把「顶点序号 → 分区」表套用到全网格
// -----------------------------------------------------------------------
//
// 第一遍在**基网**上分区、沿坍缩记录回填，得到全网格每个顶点的分区归属；
// 本函数把它**写回网格**（facet 的 groupId）并导出每组的 seed。
//
// 两个必须对齐的约定（编解码两侧各自重算时也须一致）：
//   1. 索引是**顶点序号**（`mesh->vertices()` 的迭代序），不是 poolId ——
//      跨网格 poolId 不对应（附 15 实测 `applied=0 missing=69664`）。
//   2. 每组 seed = 该组**第一个被遍历到的 facet** 的 `proxyHalfedge()`。
//      用遍历序而不是「任取一个」，是为了让它**可重算**。
static std::vector<MCGAL::Halfedge*> applySegmentGroupTable(
    const std::shared_ptr<MCGAL::Mesh>& mesh,
    const std::vector<int>& table) {
    std::unordered_map<int, int> poolToOrd;
    {
        int ord = 0;
        for (MCGAL::Vertex* v : mesh->vertices()) {
            poolToOrd[v->poolId()] = ord++;
        }
    }
    std::map<int, MCGAL::Halfedge*> firstSeed;
    int assigned = 0;
    int unassigned = 0;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (f->isRemoved()) continue;
        int gid = -1;
        for (MCGAL::Vertex* v : f->getVertices()) {
            if (v == nullptr) continue;
            auto p = poolToOrd.find(v->poolId());
            if (p == poolToOrd.end()) continue;
            const std::size_t ord = static_cast<std::size_t>(p->second);
            if (ord >= table.size()) continue;
            const int g = table[ord];
            if (g >= 0) {
                gid = g;
                break;
            }
        }
        if (gid < 0) {
            ++unassigned;
            continue;
        }
        f->setGroupId(gid);
        ++assigned;
        if (firstSeed.find(gid) == firstSeed.end()) {
            firstSeed[gid] = f->proxyHalfedge();
        }
    }
    int maxG = -1;
    for (const auto& kv : firstSeed) {
        if (kv.first > maxG) maxG = kv.first;
    }
    // 稠密向量：`seeds[i]` 必须就是分区 i 的 seed ——
    // `SegmentedOptimizedSymbolCollectOperator::ensureCollectors` 按**下标**
    // 给内层 collector 设 `groupFilter_`，下标错位 = 收错组的符号。
    std::vector<MCGAL::Halfedge*> seeds(static_cast<std::size_t>(maxG + 1), nullptr);
    for (const auto& kv : firstSeed) {
        seeds[static_cast<std::size_t>(kv.first)] = kv.second;
    }
    if (std::getenv("PMSF_SEG_TABLE")) {
        std::fprintf(stderr,
                     "[SEG_TABLE_APPLY] assigned=%d unassigned=%d groups=%zu "
                     "table=%zu\n",
                     assigned, unassigned, firstSeed.size(), table.size());
        std::fflush(stderr);
    }
    return seeds;
}

// -----------------------------------------------------------------------
// runCompress —— 通用压缩主循环
// -----------------------------------------------------------------------

static std::shared_ptr<MCGAL::Mesh> cloneMeshForPipelinePreflip(
    const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!mesh) return nullptr;
    char path[] = "/tmp/pmsf_edgeflip_preflip_XXXXXX.off";
    const int fd = mkstemps(path, 4);
    if (fd < 0) return nullptr;
    close(fd);
    mesh->dumpto_oldtype(path);
    auto copy = std::make_shared<MCGAL::Mesh>(path);
    std::remove(path);
    if (!copy) return nullptr;
    copy->bboxMin = mesh->bboxMin; copy->bboxMax = mesh->bboxMax;
    copy->f_quantStep = mesh->f_quantStep; copy->i_nbQuantBits = mesh->i_nbQuantBits;
    copy->enableQuantization = mesh->enableQuantization;
    copy->setMeshId(mesh->meshId());
    return copy;
}


static int countPairedHalfedges(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    int n = 0;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do { if (h && !h->isRemoved() && h->opposite() != nullptr) ++n; h = h->next(); } while (h && h != h0 && ++guard < 64);
    }
    return n;
}

static std::vector<std::array<int, 9>> g_sharedPatchFillFaces;

static int countBoundaryHalfedges(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    int n = 0;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do { if (h && !h->isRemoved() && h->opposite() == nullptr) ++n; h = h->next(); } while (h && h != h0 && ++guard < 64);
    }
    return n;
}

static std::shared_ptr<MCGAL::Mesh> cloneMeshExactForTrial(
    const std::shared_ptr<MCGAL::Mesh>& mesh,
    std::unordered_map<int, MCGAL::Vertex*>* outMap) {
    if (!mesh) return nullptr;
    auto out = std::make_shared<MCGAL::Mesh>();
    out->setMeshId(mesh->meshId());
    out->bboxMin = mesh->bboxMin; out->bboxMax = mesh->bboxMax;
    out->f_quantStep = mesh->f_quantStep; out->i_nbQuantBits = mesh->i_nbQuantBits;
    out->enableQuantization = mesh->enableQuantization;
    std::unordered_map<int, MCGAL::Vertex*> vmap;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        MCGAL::Vertex* nv = MCGAL::contextPool.allocateVertexFromPool(out->meshId(), v->point());
        if (!nv) return nullptr;
        nv->halfedges().clear();
        nv->setGroupId(v->groupId());
        out->add_vertex(nv);
        vmap[v->poolId()] = nv;
    }
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        std::vector<MCGAL::Vertex*> vs;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do { if (!h || !h->vertex()) break; auto it = vmap.find(h->vertex()->poolId()); if (it == vmap.end()) break; vs.push_back(it->second); h = h->next(); } while (h && h != h0 && ++guard < 64);
        if (vs.size() < 3) continue;
        MCGAL::Facet* nf = MCGAL::contextPool.allocateFaceFromPool(out->meshId(), vs);
        if (!nf) return nullptr;
        nf->setGroupId(f->groupId());
        out->add_face(nf);
    }
    for (MCGAL::Facet* f : out->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do { if (h) { if (h->opposite() == nullptr) h->setBoundary(); else { h->setNotBoundary(); h->opposite()->setNotBoundary(); } } h = h->next(); } while (h && h != h0 && ++guard < 64);
    }
    if (outMap) *outMap = std::move(vmap);
    return out;
}


static std::shared_ptr<MCGAL::Mesh> cloneMeshForTrialFreshPool(
    const std::shared_ptr<MCGAL::Mesh>& mesh,
    std::unordered_map<int, MCGAL::Vertex*>* outMap,
    int* outPoolId) {
    if (!mesh) return nullptr;
    int liveV = 0, liveF = 0;
    for (MCGAL::Vertex* v : mesh->vertices()) if (v && !v->isRemoved()) ++liveV;
    for (MCGAL::Facet* f : mesh->faces()) if (f && !f->isRemoved()) ++liveF;
    // Generous pool: the conservative terminal stage trial may allocate on failed
    // candidate attempts before rolling them back; only live elements are serialized.
    const int poolId = MCGAL::contextPool.registerPool(liveV * 32 + 4096,
                                                      liveF * 128 + 16384,
                                                      liveF * 16 + 4096);
    if (poolId < 0) return nullptr;
    if (outPoolId) *outPoolId = poolId;
    auto out = std::make_shared<MCGAL::Mesh>();
    out->setMeshId(poolId);
    out->bboxMin = mesh->bboxMin; out->bboxMax = mesh->bboxMax;
    out->f_quantStep = mesh->f_quantStep; out->i_nbQuantBits = mesh->i_nbQuantBits;
    out->enableQuantization = mesh->enableQuantization;
    std::unordered_map<int, MCGAL::Vertex*> vmap;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        MCGAL::Vertex* nv = MCGAL::contextPool.allocateVertexFromPool(out->meshId(), v->point());
        if (!nv) return out;
        nv->halfedges().clear();
        nv->setGroupId(v->groupId());
        out->add_vertex(nv);
        vmap[v->poolId()] = nv;
    }
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        std::vector<MCGAL::Vertex*> vs;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (!h || !h->vertex()) break;
            auto it = vmap.find(h->vertex()->poolId());
            if (it == vmap.end()) break;
            vs.push_back(it->second);
            h = h->next();
        } while (h && h != h0 && ++guard < 64);
        if (vs.size() < 3) continue;
        MCGAL::Facet* nf = MCGAL::contextPool.allocateFaceFromPool(out->meshId(), vs);
        if (!nf) return out;
        nf->setGroupId(f->groupId());
        out->add_face(nf);
    }
    if (outMap) *outMap = std::move(vmap);
    return out;
}

static bool isClosedTriangleManifold(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!mesh) return false;
    std::map<std::pair<MCGAL::Vertex*, MCGAL::Vertex*>, int> edgeCount;
    std::set<std::array<MCGAL::Vertex*, 3>> triSet;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Vertex* fv[3];
        int n = 0;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && h->vertex() && n < 3) fv[n++] = h->vertex();
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
        if (n != 3) return false;
        std::array<MCGAL::Vertex*, 3> tri{fv[0], fv[1], fv[2]};
        std::sort(tri.begin(), tri.end());
        if (!triSet.insert(tri).second) return false;
        for (int k = 0; k < 3; ++k) {
            MCGAL::Vertex* a = fv[k];
            MCGAL::Vertex* b = fv[(k + 1) % 3];
            if (!a || !b || a == b) return false;
            if (a > b) std::swap(a, b);
            ++edgeCount[{a, b}];
        }
    }
    for (const auto& kv : edgeCount)
        if (kv.second != 2) return false;
    return true;
}

static bool meshTopologyValid(const std::shared_ptr<MCGAL::Mesh>& mesh,
                              int* outLiveV = nullptr,
                              int* outLiveF = nullptr,
                              int* outBadEdges = nullptr,
                              int onlyGid = -1,
                              int* outDupFaces = nullptr) {
    if (!mesh) return false;
    int liveV = 0, liveF = 0;
    for (MCGAL::Vertex* v : mesh->vertices()) if (v && !v->isRemoved()) ++liveV;
    std::map<std::pair<MCGAL::Vertex*, MCGAL::Vertex*>, int> edges;
    std::set<std::array<MCGAL::Vertex*, 3>> tris;
    bool ok = true;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        if (onlyGid >= 0 && f->groupId() != onlyGid) continue;
        ++liveF;
        MCGAL::Vertex* fv[3];
        int n = 0;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && h->vertex() && n < 3) fv[n++] = h->vertex();
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
        if (n != 3) {
            ok = false;
            continue;
        }
        for (int k = 0; k < 3; ++k)
            if (!fv[k] || fv[k]->isRemoved()) {
                ok = false;
            }
        std::array<MCGAL::Vertex*, 3> tri{fv[0], fv[1], fv[2]};
        std::sort(tri.begin(), tri.end());
        if (!tris.insert(tri).second) ok = false;
        for (int k = 0; k < 3; ++k) {
            MCGAL::Vertex* a = fv[k];
            MCGAL::Vertex* b = fv[(k + 1) % 3];
            if (!a || !b || a == b) { ok = false; continue; }
            if (a > b) std::swap(a, b);
            ++edges[{a, b}];
        }
    }
    int bad = 0;
    int hist[6] = {0, 0, 0, 0, 0, 0};
    for (const auto& kv : edges) {
        ++hist[std::min(5, kv.second)];
        if (kv.second != 2) ++bad;
    }
    if (bad) ok = false;
    if (outLiveV) *outLiveV = liveV;
    if (outLiveF) *outLiveF = liveF;
    if (outBadEdges) *outBadEdges = bad;
    if (outDupFaces) *outDupFaces = static_cast<int>(liveF) - static_cast<int>(tris.size());
    return ok;
}

static std::shared_ptr<MCGAL::Mesh> rebuildMeshFromFaceListForFinalOp(
    const std::shared_ptr<MCGAL::Mesh>& mesh,
    const std::vector<int>& groups) {
    if (!mesh) return nullptr;
    char path[] = "/tmp/pmsf_face_rebuild_XXXXXX.off";
    const int fd = mkstemps(path, 4);
    if (fd < 0) return nullptr;
    close(fd);
    mesh->dumpto_oldtype(path);
    auto out = std::make_shared<MCGAL::Mesh>(path);
    std::remove(path);
    if (!out) return nullptr;
    out->bboxMin = mesh->bboxMin; out->bboxMax = mesh->bboxMax;
    out->f_quantStep = mesh->f_quantStep; out->i_nbQuantBits = mesh->i_nbQuantBits;
    out->enableQuantization = mesh->enableQuantization;
    out->setMeshId(mesh->meshId());
    size_t gi = 0;
    for (MCGAL::Facet* f : out->faces()) {
        if (!f || f->isRemoved()) continue;
        if (gi < groups.size()) f->setGroupId(groups[gi]);
        ++gi;
    }
    for (MCGAL::Vertex* v : out->vertices()) {
        if (!v || v->isRemoved()) continue;
        for (MCGAL::Halfedge* h : v->halfedges()) {
            if (h && !h->isRemoved() && h->face() && !h->face()->isRemoved()) {
                v->setGroupId(h->face()->groupId());
                break;
            }
        }
    }
    return out;
}

void runCompress(CompressOptions& options, const CompressPipelineConfig& cfg) {
    const auto fnStartT = std::chrono::steady_clock::now();
    pmsf::bndid::encEnable();
    pmsf::bndid::encReset();
    // Multi-round boundary staging relies on encoder/decoder agreeing on the
    // quantized grid everywhere.  The quantization step is maxRange/2^bits, so a
    // bbox-max vertex has in-memory q == 2^bits while the base writer (and hence
    // the decoder) clamps it to 2^bits-1.  That one-quantum disagreement only
    // shows up when a stage-modified region contains bbox-extreme vertices and
    // then breaks the following round replay, so clamp the encoder grid when the
    // experimental multi-round path is requested.
    if (std::getenv("PMSF_SC_ALLOW_MULTIROUND") != nullptr &&
        std::getenv("PMSF_QUANT_MAX_CLAMP") == nullptr) {
        setenv("PMSF_QUANT_MAX_CLAMP", "1", 1);
    }
    // Multi-round boundary staging can destroy the halfedge between a group's
    // exported seed vertices (a stage collapse rewires the local fan), while the
    // decoder re-derives the seed every round from those two vertices.  Refresh the
    // encoder seeds from the same (already surviving) endpoint pair each round so
    // both sides pick the same BFS root (`findLiveSeedBetweenLocal` mirrors the
    // decoder's `findLiveHalfedgeInMesh` search order and group filter).
    if (std::getenv("PMSF_SC_ALLOW_MULTIROUND") != nullptr &&
        std::getenv("PMSF_SEG_REFRESH_SEEDS") == nullptr) {
        setenv("PMSF_SEG_REFRESH_SEEDS", "1", 1);
    }
    // A multi-round boundary schedule implies the group-stage codec: when the caller
    // provides only ALLOW_MULTIROUND + BOUNDARY_ROUNDS (+ optional MAXOPS), turn on the
    // stage switches so the user does not have to pass four independent env vars.
    if (std::getenv("PMSF_SC_ALLOW_MULTIROUND") != nullptr &&
        std::getenv("PMSF_SC_BOUNDARY_ROUNDS") != nullptr &&
        std::getenv("PMSF_SC_BOUNDARY_STAGE") == nullptr &&
        std::getenv("PMSF_SC_BOUNDARY_GROUP_STAGE") == nullptr &&
        std::getenv("PMSF_SC_BOUNDARY_ANCHOR_STAGE") == nullptr &&
        std::getenv("PMSF_SC_BOUNDARY_COMPACT_STAGE") == nullptr) {
        setenv("PMSF_SC_BOUNDARY_STAGE", "1", 1);
        setenv("PMSF_SC_BOUNDARY_GROUP_STAGE", "1", 1);
    }
    // Segment playback can interleave boundary stages with edge flips.  Two encoder
    // changes are then required:
    //
    // 1. the coordinate-symbol flip format, so the segmented decoder can undo each
    //    flip by endpoint coordinates instead of the un-reproducible replay edge
    //    order (see BOUNDARY_COMPRESSION_DECISIONS.md D490/D497);
    // 2. a boundary scope lock: flipping at/next to group seams makes the split
    //    ordering and groupId tie-breaks of the interior replay diverge from the
    //    encoder (D509).  Keeping flips strictly inside a group keeps the replay
    //    unambiguous, which makes the flip+boundary interleave decode exactly
    //    (D513).  Scope 2 = boundary vertices + their one-ring neighbours.
    if (std::getenv("PMSF_SC_ALLOW_MULTIROUND") != nullptr &&
        std::getenv("PMSF_EDGEFLIP_ENABLE") != nullptr) {
        // One bit per all-edge, in canonical (endpoint-q sorted) order.  The flip
        // operator locks the two triangles (and their ring) of every flipped edge,
        // so marked edges are pairwise independent; the decoder rebuilds the same
        // canonical edge list from its (topologically identical) round state and
        // simply re-flips the marked edges.  No per-flip coordinates and no shared
        // BFS root are needed (PMSF_EDGEFLIP_BITS_CANONICAL, D518).
        if (std::getenv("PMSF_EDGEFLIP_BITS_ONLY") == nullptr) {
            setenv("PMSF_EDGEFLIP_BITS_ONLY", "1", 1);
        }
        if (std::getenv("PMSF_EDGEFLIP_BITS_CANONICAL") == nullptr) {
            setenv("PMSF_EDGEFLIP_BITS_CANONICAL", "1", 1);
        }
        if (std::getenv("PMSF_EDGEFLIP_LOCK_BOUNDARY") == nullptr) {
            setenv("PMSF_EDGEFLIP_LOCK_BOUNDARY", "2", 1);
        }
    }
    // Mixed-group fans are the other staging side effect the decoder cannot infer:
    // it inherits one groupId per split facet, so any elimination of a vertex whose
    // fan spans two groups corrupts the reconstruction labels.  Guard it whenever
    // the multi-round path is requested (this is also correct for the final-round
    // path but kept behind the same experimental switch for byte-compatibility).
    if (std::getenv("PMSF_SC_ALLOW_MULTIROUND") != nullptr &&
        std::getenv("PMSF_SC_FAN_GROUP_GUARD") == nullptr) {
        setenv("PMSF_SC_FAN_GROUP_GUARD", "1", 1);
    }
    // PRE / mixed round ordering changes the local fan history enough that the
    // loop-locator payload can go stale across several staged rounds; the compact
    // q-locator payload (len 18) addresses the boundary edge by endpoint q and is
    // order-independent.  Enable it automatically for any PRE ordering.
    if (std::getenv("PMSF_SC_ALLOW_MULTIROUND") != nullptr &&
        (std::getenv("PMSF_SC_BOUNDARY_PRE") || std::getenv("PMSF_SC_BOUNDARY_PRE_ROUNDS")) &&
        std::getenv("PMSF_SC_QLOC") == nullptr) {
        setenv("PMSF_SC_QLOC", "1", 1);
    }
    // NOTE: set PMSF_SC_QLOC=0 to force the topology-anchored loop locator (len 7)
    // for PRE/mixed as well; the q-locator is then not written at all.
    const bool boundaryModuleEnabled = [] {
        const char* v = std::getenv("PMSF_BOUNDARY_MODULE");
        return v && v[0] == '1';
    }();
    if (boundaryModuleEnabled) {
        // Optional independent module: reuse the proven pre-pass boundary patch,
        // but store its payload in the companion section instead of inventing a
        // second mesh format.  Only active when the module is enabled.
        setenv("PMSF_SHARED_BOUNDARY_PATCH", "1", 1);
        setenv("PMSF_SHARED_BOUNDARY_PATCH_ALLOW_LARGE", "1", 1);
    }
    static std::vector<int> g_boundaryStorageCounts;
    static std::vector<unsigned char> g_boundaryStorageBits;
    static std::vector<int> g_boundaryStorageResiduals;
    static int g_boundaryStorageMode = 0;
    static std::vector<int> g_bfsAnchorResiduals;
    static std::vector<int> g_groupAnchorResiduals;
    static std::vector<std::vector<int>> g_scBoundaryRoundData;
    // Per-round boundary ordering: 1 = stage BEFORE the round interior elimination
    // (PRE), 0 = after (POST).  Only needed for the mixed-order payload (mode 66).
    static std::vector<int> g_scRoundOrdering;
    // V3 (mode71) effective stage order: what the encoder ACTUALLY executed, which
    // differs from the requested order for POST rounds (e.g. requested BIF at POST
    // executes I,F,B = IFB; requested BFI skips boundary and runs flip early = FIB).
    static std::vector<int> g_scEffectiveRoundOrdering;
    // boundary v2 diagnostics: fixed loop anchors chosen at round 0
    static thread_local std::vector<std::pair<int, MCGAL::Vertex*>> g_bcFixedAnchors;
    static std::vector<pmsf::ScAnchorDef> g_scAnchors;
    static std::vector<std::array<int, 6>> g_scRoundSeedQ;
    // V3: BFS seed vertex pair actually used for each round's interior collect,
    // recorded at collect time and written to the mode71 payload as stable IDs.
    static std::vector<std::array<MCGAL::Vertex*, 2>> g_scRoundSeedVerts;
    static std::vector<int> g_groupSplitResiduals;
    g_groupSplitResiduals.clear();
    g_boundaryStorageCounts.clear();
    g_boundaryStorageBits.clear();
    g_boundaryStorageResiduals.clear();
    g_boundaryStorageMode = 0;
    std::shared_ptr<MCGAL::Mesh> preFlipBaseForSerialize;
    if (envOn("PMSF_EDGEFLIP_ENABLE")) {
        int rounds = 1;
        if (const char* r = std::getenv("PMSF_EDGEFLIP_ROUNDS")) {
            const int v = std::atoi(r);
            if (v > 0) rounds = v;
        }
        options.forceEnableEdgeFlipPostProcess(true, rounds);
    }
    // 框架职责：确保 MCGAL ContextPool 已初始化。池容量按需估计（标定乘数
    // 收敛在 MeshRuntime 一处，env `PMSF_POOL_*` 仍可显式覆盖）——巨型固定池
    // 的 `new Halfedge[N]` 逐元素构造曾是进程级隐藏常数（18M ≈ +600ms）。
    // prescan 拿到的输入规模同时还要传给序列化器（写 compact header 的
    // origTotalVertices/origTotalFaces），供解码端按需注册池。
    int inV = 0, inF = 0;
    if (prescanOffMeshCounts(options.getPath(), inV, inF)) {
        ensureMeshRuntimeInitializedForMeshSize(inV, inF);
    } else {
        ensureMeshRuntimeInitialized();
    }
    // ⚠️ 2026-05-04 性能插桩：阶段计时（chrono 累加）。
    //   打开方式：环境变量 PMSF_PROFILE=1 启用。开销极低（每阶段两次 steady_clock::now()
    //   + 一个 double 累加），O(rounds × stages) ≈ 百次调用，对 bunny ~1 微秒级。
    //   结束时打印到 stderr：
    //     [PROFILE] load_mesh=XX.X ms select_init=X.X init=X.X
    //               select_loop=XX.X eliminate_loop=XX.X update=XX.X
    //               preprocess=XX.X postprocess=XX.X
    //               collect=XX.X serialize=XX.X total=XX.X
    const bool profileOn = []() {
        const char* e = std::getenv("PMSF_PROFILE");
        return e && *e && *e != '0';
    }();
    using clk = std::chrono::steady_clock;
    auto now = [] { return clk::now(); };
    auto ms = [](clk::duration d) {
        return std::chrono::duration<double, std::milli>(d).count();
    };
    clk::duration t_load{}, t_preInit{}, t_selectInit{}, t_eliminateInit{},
                  t_preprocess{}, t_selectLoop{}, t_eliminateLoop{}, t_updateLoop{},
                  t_collectLoop{}, t_selectReset{}, t_postprocess{}, t_serialize{};
    auto t_total_start = now();
    if (std::getenv("PMSF_PERF_PHASE") != nullptr) {
        std::fprintf(stderr, "[PERF_PHASE] pre_profile_init=%.1f ms\n",
                     std::chrono::duration<double, std::milli>(t_total_start - fnStartT)
                         .count());
    }

    // ---------- 1) header ----------
    auto headerBuilder = PMSFHeader::Builder()
                             .setRound(options.getRound())
                             .setEnableCompress(options.isEnableCompress())
                             .setEnableQuantization(options.isEnableQuantization())
                             .setEnableSegmentation(options.isEnableSegmentation())
                             .setEnableAdjustMent(options.isEnableAdjustment())
                             .setEnableTriangulation(options.isEnableTriangulation())
                             .setEnableConnectivityPredictionFaces(
                                 options.isEnableConnectivityPredictionFaces())
                             .setEnableConnectivityPredictionEdges(
                                 options.isEnableConnectivityPredictionEdges())
                             .setEnableTriangleMeshBalance(
                                 options.isEnableTriangleMeshBalance());
    // header 里的 prediction 位必须与**序列化器实际走的编码路径**一致。
    //
    // BasicSerializeOperator::serialize() 在 enablePrediction（= options 里的
    // prediction）为真时走 range-coder 导出；解码端读的就是 header 这一位，决定
    // 走不走同一个导入。历史上这一位只在 cfg.headerCarryPrediction 为真时才写，
    // 于是「options 开 prediction + cfg 没开 headerCarryPrediction」的组合会产出
    // **自相矛盾的产物**：编码端写 RC 流、header 却说不预测、解码端按老格式读 ——
    // 不报错，只是从某一轮起解出的网格开始膨胀（实测：第 12 轮起顶点数从
    // 34834 一路涨到 37 万）。
    //
    // 所以判据收敛成一条：options 开了 prediction，header 就必须如实记下来。
    if (cfg.headerCarryPrediction || options.isEnablePrediction()) {
        headerBuilder.setEnablePrediction(options.isEnablePrediction());
    }
    PMSFHeader header = headerBuilder.build();

    // ---------- 2) load mesh ----------
    auto t0 = now();
    std::shared_ptr<MCGAL::Mesh> mesh =
        std::make_shared<MCGAL::Mesh>(MCGAL::Mesh(options.getPath()));
    if (const char* splitMode = std::getenv("PMSF_TEST_BOUNDARY_SPLIT")) {
        const bool wantBoundary = std::string(splitMode).rfind("boundary", 0) == 0;
        if (std::string(splitMode) == "boundary_codec") {
            MCGAL::Halfedge* be = nullptr;
            for (MCGAL::Facet* f : mesh->faces()) {
                if (!f || f->isRemoved()) continue;
                for (MCGAL::Halfedge* h : *f) {
                    if (h && !h->isRemoved() && h->opposite() == nullptr && h->face()) { be = h; break; }
                }
                if (be) break;
            }
            if (!be) { std::cerr << "[codec] no boundary edge" << std::endl; std::exit(0); }
            MCGAL::Point p1; for (int k=0;k<3;++k) p1[k]=(be->vertex()->point()[k]+be->end_vertex()->point()[k])*0.5f;
            MCGAL::Vertex* nv1 = nullptr;
            pmsf::splitBoundaryEdgeInsertVertex(mesh, be, p1, &nv1);
            MCGAL::Point removed = nv1 ? nv1->point() : p1;
            MCGAL::Halfedge* ne = nullptr;
            if (nv1) pmsf::collapseBoundaryVertex3Robust(mesh, nv1, &ne);
            MCGAL::Vertex* nv2 = nullptr;
            if (ne) pmsf::splitBoundaryEdgeInsertVertex(mesh, ne, removed, &nv2);
            mesh->checkMesh();
            mesh->dumpto_oldtype("/tmp/split_test.off");
            std::cerr << "[codec] V=" << mesh->size_of_vertices() << " F=" << mesh->size_of_facets()
                      << " ne=" << ne << " nv2=" << nv2 << std::endl;
            std::exit(0);
        }
        if (std::string(splitMode) == "boundary_fan_codec") {
            MCGAL::Halfedge* be = nullptr;
            for (MCGAL::Facet* f : mesh->faces()) {
                if (!f || f->isRemoved()) continue;
                for (MCGAL::Halfedge* h : *f) {
                    if (h && !h->isRemoved() && h->opposite() == nullptr && h->face()) { be = h; break; }
                }
                if (be) break;
            }
            if (!be) { std::cerr << "[fan-codec] no boundary edge" << std::endl; std::exit(0); }
            MCGAL::Point p1; for (int k=0;k<3;++k) p1[k]=(be->vertex()->point()[k]+be->end_vertex()->point()[k])*0.5f;
            MCGAL::Vertex* nv = nullptr;
            pmsf::splitBoundaryEdgeInsertVertex(mesh, be, p1, &nv);
            pmsf::BoundaryFanPatch fan;
            if (nv) pmsf::captureBoundaryFanPatch(mesh, nv, fan);
            MCGAL::Halfedge* ne = nullptr;
            if (nv) pmsf::collapseBoundaryVertex3Robust(mesh, nv, &ne);
            pmsf::captureBoundaryPostFaces(mesh, fan);
            bool ok = pmsf::applyBoundaryFanPatch(mesh, fan);
            canonicalizeTraversalProxies(mesh);
            mesh->checkMesh();
            mesh->dumpto_oldtype("/tmp/split_test.off");
            std::cerr << "[fan-codec] V=" << mesh->size_of_vertices() << " F=" << mesh->size_of_facets()
                      << " orig=" << fan.origFaces.size() << " post=" << fan.postFaces.size()
                      << " ok=" << (ok?1:0) << std::endl;
            std::exit(0);
        }
        MCGAL::Halfedge* target = nullptr;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved()) continue;
            for (MCGAL::Halfedge* h : *f) {
                if (!h || h->isRemoved() || !h->face()) continue;
                if (wantBoundary) {
                    if (h->opposite() == nullptr && h->face()->facet_degree() == 3) { target = h; break; }
                } else if (h->opposite() && !h->opposite()->isRemoved() &&
                           h->opposite()->face() && h->face()->facet_degree() == 3 &&
                           h->opposite()->face()->facet_degree() == 3) {
                    target = h; break;
                }
            }
            if (target) break;
        }
        if (target) {
            MCGAL::Point pos;
            for (int k = 0; k < 3; ++k)
                pos[k] = (target->vertex()->point()[k] + target->end_vertex()->point()[k]) * 0.5f;
            MCGAL::Vertex* nv = nullptr;
            if (wantBoundary) {
                pmsf::splitBoundaryEdgeInsertVertex(mesh, target, pos, &nv);
            } else {
                pmsf::splitInteriorEdgeInsertVertex(mesh, target, pos, &nv);
            }
            if (std::string(splitMode) == "boundary_rt2" && nv) {
                pmsf::collapseBoundaryVertex3Robust(mesh, nv);
            }
            if (std::string(splitMode) == "boundary_roundtrip" && nv) {
                MCGAL::Halfedge* av = nullptr;
                for (MCGAL::Halfedge* hh : nv->halfedges()) {
                    if (hh && hh->opposite() == nullptr) { av = hh; break; }
                }
                if (av) mesh->new_collapse(av);
            }
            mesh->checkMesh();
            mesh->dumpto_oldtype("/tmp/split_test.off");
            std::cerr << "[split-test] mode=" << splitMode << " V=" << mesh->size_of_vertices()
                      << " F=" << mesh->size_of_facets() << std::endl;
        } else {
            std::cerr << "[split-test] no target" << std::endl;
        }
        std::exit(0);
    }

    mesh->enableQuantization = options.isEnableQuantization();
    t_load += now() - t0;

    // ---------- 3) preInitPreparation ----------
    // 例如 PPMC 的 mesh->quantizeVertexPositions() 必须发生在 SelectOperator::init
    // 之前，因为 select 初始化会读取顶点坐标构建误差优先队列。
    t0 = now();
    if (cfg.preInitPreparation) {
        cfg.preInitPreparation(mesh, header, options);
    }

    // ---------- 3b) context-level 初始化 + vertex degree 预计算 ----------
    if (cfg.trackContextLevel) {
        MCGAL::contextPool.setMetadata("level", MCGAL::ContextValue(0));
    }
    if (cfg.recomputeVertexDegreesOnce) {
        mesh->recomputeAllVertexDegrees();
    }
    t_preInit += now() - t0;

    // ---------- 4) operators ----------
    auto select = options.getSelect();
    auto eliminate = options.getEliminate();
    auto collector = cfg.collectorFactory();
    auto serializer = cfg.serializerFactory(collector, options);
    std::shared_ptr<Graph> boundaryGraph;
    serializer->setEnableEdgeFlipPostProcess(options.isEnableEdgeFlipPostProcess());
    if (options.isEnableEdgeFlipPostProcess()) {
        setenv("PMSF_EDGEFLIP_STABLE_BFS", "1", 1);
    }
    serializer->setEdgeFlipPostProcessConfig(
        options.getEdgeFlipPostProcessRounds(),
        options.getEdgeFlipPostProcessLongEdgeRatio(),
        options.getEdgeFlipPostProcessMinImprovementRatio());
    std::shared_ptr<std::unordered_set<int>> edgeFlipProtectedVertexIds;
    if (options.isEnableEdgeFlipPostProcess() &&
        !std::getenv("PMSF_EDGEFLIP_NO_PROTECT")) {
        edgeFlipProtectedVertexIds = std::make_shared<std::unordered_set<int>>();
        select->addIsRemovableOperator(
            std::make_shared<EdgeFlipProtectedPatchIsRemovableOperator>(
                edgeFlipProtectedVertexIds));
    }
    auto seedIsRemovableOperator = std::make_shared<SeedIsRemovableOperator>();
    if (envOn("PMSF_VISUAL_TOPOLOGY_GUARD") ||
        envOn("PMSF_VISUAL_ISFLIP_GUARD")) {
        const auto mode = eliminate->usesEdgeCollapseMode()
                              ? VisualTopologyIsRemovableOperator::Mode::EdgeCollapse
                              : VisualTopologyIsRemovableOperator::Mode::VertexRemoval;
        select->addIsRemovableOperator(
            std::make_shared<VisualTopologyIsRemovableOperator>(mode));
    }
    if (envOn("PMSF_VISUAL_ISFLIP_GUARD") &&
        dynamic_cast<GreedySelector*>(select.get()) == nullptr) {
        select->addIsRemovableOperator(std::make_shared<IsFlipOperator>());
    }

    // B.1 Sprint 2 probe：runCompress 可选并行 collector，只做 diag 统计，
    // 不参与 serialize（见 CompressPipelineConfig::probeCollectorFactory 注释）。
    std::shared_ptr<SymbolCollectOperator> probeCollector;
    if (cfg.probeCollectorFactory) {
        probeCollector = cfg.probeCollectorFactory();
    }

    auto preProcessOperator = options.getPreProcessOperator();
    auto postProcessOperator = options.getPostProcessOperator();

    // ---------- 5) optional property registration (QEM quadrics etc) ----------
    if (select->options_.getRegisterFunc()) {
        select->options_.getRegisterFunc()(mesh);
    }

    // ---------- 6) select->init ----------
    t0 = now();
    select->init(mesh);
    if (cfg.refreshVertexDegreesEachRound) {
        select->setRefreshDegreesEachRound(true);
    }
    t_selectInit += now() - t0;

    // ---------- 7) options.preProcess (position=Once) ----------
    // PCATM 历史行为：在 select init 之后、eliminate init 之前调用一次。
    t0 = now();
    if (cfg.preProcessPosition == CompressPipelineConfig::PreProcessPosition::Once
        && preProcessOperator) {
        preProcessOperator->process(mesh);
    }
    t_preprocess += now() - t0;

    // ---------- 8) eliminate->init ----------
    t0 = now();
    eliminate->init(mesh, header);
    t_eliminateInit += now() - t0;


    // ---------- 9) seed 落位 ----------
    MCGAL::Halfedge* seed = nullptr;
    MCGAL::Point p;
    HalfedgeErrorEntry entry(seed, p, 0, 0);

    switch (cfg.seedStrategy) {
        case CompressPipelineConfig::SeedStrategy::SelectSeed: {
            if (cfg.customSeedSelector) {
                select->selectSeed(seed, cfg.customSeedSelector);
            } else {
                select->selectSeed(seed);
            }
            entry.edge = seed;
            break;
        }
        case CompressPipelineConfig::SeedStrategy::SelectThenExtract: {
            select->select(entry);
            seed = entry.edge;
            break;
        }
    }

    // ---------- 10) 分区阶段（P18.2 阶段 A）+ collector / seed IsRemovable 挂接 ----------
    //
    // **分区阶段是框架此前完全没有的一环。** 在加这个块之前，全树没有任何地方
    // 构造/调用 `SegmentOperator` ⇒ 每个 facet 的 `groupId` 恒为默认 -1
    // （deps/MCGAL/include/Halfedge.h:248），而 collector 拿到的 seeds 恒为
    // 写死的单元素 `{seed}`。任何按 groupId 过滤的 collector 于是看到**空视图**，
    // 却仍用整网格的字母表编码 ⇒ 符号流不自洽 ⇒ range coder 模型漂移
    // （表现为 qsmodel 的 "BUG: rescaling ..."）。
    // 归因全文见 docs/refactor/P18.2_PARTITION_STAGE_ATTRIBUTION.md。
    //
    // 这也解释了 CompressPipeline.h 顶部注释为什么把 `segment` 排除在 runCompress
    // 之外（「嵌套 per-segment 循环」）—— 那是**阶段 B** 的事；本块只做阶段 A
    // （产出真 groupId + 真 seeds）。
    //
    // **默认路径零字节变化**：`cfg.segmentOperatorFactory` 为空 ⇒ 下面 `seeds`
    // 仍是 `{seed}`，三个 init 收到的实参逐一相同。
    //
    // P18.2 路线 (a) 两遍改造把这一块分成三种模式：
    //   读表（`segmentGroupTableIn`）  → 分组由第一遍算好，这里只**套用**
    //   写表（`segmentGroupTableOut`） → 在这里**不分区**（分区对象是基网，
    //                                    不是本处的全网格），留到轮循环之后
    //   都不设                          → 现有行为：就地分区（阶段 A 原样）
    std::vector<MCGAL::Halfedge*> seeds{seed};
    if (cfg.segmentGroupTableIn) {
        seeds = applySegmentGroupTable(mesh, *cfg.segmentGroupTableIn);
    } else if (!cfg.segmentGroupTableOut && cfg.segmentOperatorFactory) {
        if (auto segmentOperator = cfg.segmentOperatorFactory()) {
            std::shared_ptr<MCGAL::Mesh> segmentedMesh = mesh;
            segmentOperator->segment(segmentedMesh);
            auto exportedSeeds = segmentOperator->exportSeeds();
            if (!exportedSeeds.empty()) {
                seeds = std::move(exportedSeeds);
            }
            boundaryGraph = segmentOperator->exportGraph();
#ifdef COLUMNAR_SUPPORT
            if (boundaryGraph) {
                if (auto* columnar = dynamic_cast<ColumnarSerializeOperator*>(serializer.get())) {
                    columnar->setBoundaryGraph(boundaryGraph);
                }
            }
#endif
            // 单分区（N=1）：分区器那条**唯一**的 seed 是全网格上任取的一个
            // facet 的 proxyHalfedge，它**不受消除保护** ⇒ 跑到基网时早被移除，
            // 其 base `vid` = -1 ⇒ seed 通道写出 (-1,-1) ⇒ 解码端无从套用、只能
            // 另算一个**活** seed ⇒ 两侧 BFS 起点不同 ⇒ 每面 splittable 位错配
            // （实测：轮 0 计数相同但 20 个面不同），逐轮放大 ⇒ 解码塌成
            // 11,653/16,325。单分区在语义上**就是整网格**，故 seed 可任取 ——
            // 取**主循环 seed**：它是框架恒保护的那条（survives，base vid=0/1），
            // 与解码端重算规则一致，故回环精确。N≥2 分支（seeds.size()>1）不受影响。
            if (seeds.size() == 1 && seed) {
                seeds[0] = seed;
            }
        }
    }

    // [group anchor BFS test, env gated] Can group labels be derived from one
    // seed face per group by deterministic BFS?  Diagnostic only.
    if (std::getenv("PMSF_GROUP_BFS_TEST") && mesh) {
        std::unordered_map<MCGAL::Facet*, int> label;
        std::deque<MCGAL::Facet*> dq;
        std::unordered_map<int, MCGAL::Facet*> firstFace;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved()) continue;
            const int g = f->groupId();
            if (g < 0 || firstFace.count(g)) continue;
            firstFace[g] = f;
        }
        for (auto& kv : firstFace) {
            if (!kv.second || kv.second->isRemoved()) continue;
            label[kv.second] = kv.first;
            dq.push_back(kv.second);
        }
        int assigned = 0, mismatch = 0;
        while (!dq.empty()) {
            MCGAL::Facet* f = dq.front(); dq.pop_front();
            const int g = label[f];
            MCGAL::Halfedge* h0 = f->proxyHalfedge();
            MCGAL::Halfedge* h = h0;
            int guard = 0;
            do {
                if (h && h->opposite() && h->opposite()->face()) {
                    MCGAL::Facet* nb = h->opposite()->face();
                    if (!nb->isRemoved() && !label.count(nb)) {
                        label[nb] = g;
                        dq.push_back(nb);
                    }
                }
                h = h ? h->next() : nullptr;
            } while (h && h != h0 && ++guard < 64);
        }
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved()) continue;
            ++assigned;
            auto it = label.find(f);
            if (it == label.end() || it->second != f->groupId()) ++mismatch;
        }
        std::fprintf(stderr, "[group-bfs-test] faces=%d assigned=%d mismatch=%d groups=%zu\n",
                     assigned, (int)label.size(), mismatch, firstFace.size());
    }

    // [segmentation-as-BFS prototype, env gated] Relabel facets by multi-source
    // BFS from the per-group seeds, so group labels are derivable from the seed
    // channel alone (no group RLE needed in the ideal case).
    if (std::getenv("PMSF_SEG_BFS_LABELS") && mesh && !seeds.empty()) {
        std::unordered_map<MCGAL::Facet*, int> label;
        std::deque<MCGAL::Facet*> dq;
        for (std::size_t g = 0; g < seeds.size(); ++g) {
            MCGAL::Halfedge* s = seeds[g];
            MCGAL::Facet* f = (s && s->face() && !s->face()->isRemoved()) ? s->face() : nullptr;
            if (!f) {
                for (MCGAL::Facet* cand : mesh->faces()) {
                    if (cand && !cand->isRemoved() && cand->groupId() == static_cast<int>(g)) {
                        f = cand;
                        break;
                    }
                }
            }
            if (!f || label.count(f)) continue;
            label[f] = static_cast<int>(g);
            dq.push_back(f);
        }
        while (!dq.empty()) {
            MCGAL::Facet* f = dq.front(); dq.pop_front();
            const int g = label[f];
            MCGAL::Halfedge* h0 = f->proxyHalfedge();
            MCGAL::Halfedge* h = h0;
            int guard = 0;
            do {
                if (h && h->opposite() && h->opposite()->face()) {
                    MCGAL::Facet* nb = h->opposite()->face();
                    if (!nb->isRemoved() && !label.count(nb)) {
                        label[nb] = g;
                        dq.push_back(nb);
                    }
                }
                h = h ? h->next() : nullptr;
            } while (h && h != h0 && ++guard < 64);
        }
        int mismatch = 0, assigned = 0;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved()) continue;
            ++assigned;
            auto it = label.find(f);
            if (it == label.end()) { ++mismatch; continue; }
            if (it->second != f->groupId()) ++mismatch;
            f->setGroupId(it->second);
        }
        std::fprintf(stderr, "[seg-bfs] groups=%zu faces=%d mismatch_vs_original=%d\n",
                     seeds.size(), assigned, mismatch);
    }

    // [per-round BFS group labels, env gated] One permanent anchor per group.
    std::vector<MCGAL::Vertex*> bfsAnchors;
    auto bfsRelabelGroups = [&](const std::shared_ptr<MCGAL::Mesh>& m) {
        if (!m || bfsAnchors.empty()) return;
        std::unordered_map<MCGAL::Facet*, int> label;
        std::deque<MCGAL::Facet*> dq;
        for (std::size_t g = 0; g < bfsAnchors.size(); ++g) {
            MCGAL::Vertex* a = bfsAnchors[g];
            if (!a || a->isRemoved()) continue;
            MCGAL::Facet* f = nullptr;
            for (MCGAL::Halfedge* h : a->halfedges())
                if (h && h->face() && !h->face()->isRemoved()) { f = h->face(); break; }
            if (!f || label.count(f)) continue;
            label[f] = static_cast<int>(g);
            dq.push_back(f);
        }
        while (!dq.empty()) {
            MCGAL::Facet* f = dq.front(); dq.pop_front();
            const int g = label[f];
            MCGAL::Halfedge* h0 = f->proxyHalfedge();
            MCGAL::Halfedge* h = h0;
            int guard = 0;
            do {
                if (h && h->opposite() && h->opposite()->face()) {
                    MCGAL::Facet* nb = h->opposite()->face();
                    if (!nb->isRemoved() && !label.count(nb)) {
                        label[nb] = g;
                        dq.push_back(nb);
                    }
                }
                h = h ? h->next() : nullptr;
            } while (h && h != h0 && ++guard < 64);
        }
        for (MCGAL::Facet* f : m->faces()) {
            if (!f || f->isRemoved()) continue;
            auto it = label.find(f);
            if (it != label.end()) f->setGroupId(it->second);
        }
    };
    if (std::getenv("PMSF_SEG_BFS_ROUND") && mesh) {
        const std::size_t G = seeds.size();
        bfsAnchors.assign(G, nullptr);
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
            const int g = f->groupId();
            if (g < 0 || static_cast<std::size_t>(g) >= G || bfsAnchors[static_cast<std::size_t>(g)]) continue;
            bfsAnchors[static_cast<std::size_t>(g)] = f->proxyHalfedge()->vertex();
        }
        auto& frozen = pmsf::sharedBoundaryFrozenVertices();
        frozen.clear();
        int n = 0;
        for (MCGAL::Vertex* a : bfsAnchors)
            if (a) { frozen.insert(a); ++n; }
        std::fprintf(stderr, "[seg-bfs-round] groups=%zu anchors=%d\n", G, n);
        g_bfsAnchorResiduals.clear();
        g_bfsAnchorResiduals.push_back(static_cast<int>(bfsAnchors.size()));
        for (MCGAL::Vertex* a : bfsAnchors) {
            if (a) {
                auto q = mesh->floatPosToInt(a->point());
                g_bfsAnchorResiduals.push_back(q.x());
                g_bfsAnchorResiduals.push_back(q.y());
                g_bfsAnchorResiduals.push_back(q.z());
            } else {
                g_bfsAnchorResiduals.push_back(-1);
                g_bfsAnchorResiduals.push_back(-1);
                g_bfsAnchorResiduals.push_back(-1);
            }
        }
    }

    // [boundary topology compression prototype 2, env gated]
    // Collapse group-boundary edges directly (lossy boundary), then compress the
    // simplified mesh.  Unlike the eliminate-based pre-pass this uses the mesh
    // link-condition collapse and avoids boundary-vertex triangulation state.
    if (std::getenv("PMSF_BOUNDARY_COLLAPSE") && mesh) {
        if (const auto st0 = pmsf::measureBoundaryStorage(mesh); st0.segmentCount >= 0)
            std::fprintf(stderr, "[boundary-topology] before segments=%d boundaryV=%d\n",
                         st0.segmentCount, st0.boundaryVertexCount);
        const char* maxEnv = std::getenv("PMSF_BOUNDARY_COLLAPSE_MAX");
        const int maxOps = maxEnv ? std::max(0, std::atoi(maxEnv)) : 100000;
        int removed = 0;
        for (int iter = 0; iter < maxOps; ++iter) {
            bool collapsed = false;
            for (MCGAL::Facet* f : mesh->faces()) {
                if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                MCGAL::Halfedge* h0 = f->proxyHalfedge();
                MCGAL::Halfedge* h = h0;
                int guard = 0;
                do {
                    if (h && !h->isRemoved() && h->face() && h->opposite() &&
                        h->opposite()->face() && !h->opposite()->face()->isRemoved() &&
                        h->face()->groupId() != h->opposite()->face()->groupId() &&
                        h->end_vertex() && !h->end_vertex()->isRemoved() &&
                        h->vertex() && !h->vertex()->isRemoved() &&
                        mesh->new_is_collapse_ok(h)) {
                        const int owner = std::min(h->face()->groupId(),
                                                    h->opposite()->face()->groupId());
                        MCGAL::Vertex* survivor = h->end_vertex();
                        mesh->new_collapse(h);
                        if (survivor) survivor->setGroupId(owner);
                        ++removed;
                        collapsed = true;
                        break;
                    }
                    h = h ? h->next() : nullptr;
                } while (h && h != h0 && ++guard < 64);
                if (collapsed) break;
            }
            if (!collapsed) break;
        }
        int liveV = 0, liveF = 0;
        for (MCGAL::Vertex* v : mesh->vertices()) if (v && !v->isRemoved()) ++liveV;
        for (MCGAL::Facet* f : mesh->faces()) if (f && !f->isRemoved()) ++liveF;
        std::fprintf(stderr, "[boundary-collapse] removed=%d V=%d F=%d\n", removed, liveV, liveF);
        if (const auto st1 = pmsf::measureBoundaryStorage(mesh); st1.segmentCount >= 0)
            std::fprintf(stderr, "[boundary-topology] after segments=%d boundaryV=%d\n",
                         st1.segmentCount, st1.boundaryVertexCount);
    }
    // [group anchors, env gated] One permanent vertex per group, chosen right
    // after segmentation and frozen for all subsequent stages/rounds.
    std::vector<MCGAL::Vertex*> groupAnchors;
    if (std::getenv("PMSF_GROUP_ANCHORS") && mesh) {
        groupAnchors = pmsf::collectOneAnchorPerGroup(mesh.get());
        auto& frozen = pmsf::sharedBoundaryFrozenVertices();
        auto& guarded = pmsf::anchorGuardedVertices();
        const bool freezeAnchors = std::getenv("PMSF_GROUP_ANCHORS_NO_FREEZE") == nullptr;
        const bool guardAnchors = std::getenv("PMSF_GROUP_ANCHORS_FREEZE_GUARD") != nullptr;
        guarded.clear();
        int n = 0;
        if (guardAnchors) {
            for (MCGAL::Vertex* a : groupAnchors)
                if (a) { guarded.insert(a); ++n; }
        } else if (freezeAnchors) {
            frozen.clear();
            for (MCGAL::Vertex* a : groupAnchors)
                if (a) { frozen.insert(a); ++n; }
        }
        int traversed = 0, segments = 0, coveredGroups = 0;
        for (std::size_t g = 0; g < groupAnchors.size(); ++g) {
            MCGAL::Vertex* a = groupAnchors[g];
            if (!a) continue;
            const auto edges = pmsf::walkAllGroupBoundaryEdges(mesh.get(), a, static_cast<int>(g));
            traversed += static_cast<int>(edges.size());
            if (!edges.empty()) { ++segments; ++coveredGroups; }
        }
        std::set<std::pair<int, int>> uniqueEdges;
        for (std::size_t g = 0; g < groupAnchors.size(); ++g) {
            MCGAL::Vertex* a = groupAnchors[g];
            if (!a) continue;
            for (const auto& e : pmsf::walkAllGroupBoundaryEdges(mesh.get(), a, static_cast<int>(g)))
                uniqueEdges.insert(e);
        }
        g_groupAnchorResiduals.clear();
        g_groupAnchorResiduals.push_back(static_cast<int>(groupAnchors.size()));
        for (MCGAL::Vertex* a : groupAnchors) {
            if (a) {
                auto q = mesh->floatPosToInt(a->point());
                g_groupAnchorResiduals.push_back(q.x());
                g_groupAnchorResiduals.push_back(q.y());
                g_groupAnchorResiduals.push_back(q.z());
            } else {
                g_groupAnchorResiduals.push_back(-1);
                g_groupAnchorResiduals.push_back(-1);
                g_groupAnchorResiduals.push_back(-1);
            }
        }
        int totalBoundaryEdges = 0;
        std::set<std::pair<int, int>> allBoundaryEdges;
        for (const auto& seg : pmsf::collectGroupBoundarySegments(mesh.get())) {
            totalBoundaryEdges += static_cast<int>(seg.edges.size());
            for (MCGAL::Halfedge* h : seg.edges)
                if (h && h->vertex() && h->end_vertex()) {
                    int a = h->vertex()->poolId();
                    int b = h->end_vertex()->poolId();
                    if (a > b) std::swap(a, b);
                    allBoundaryEdges.insert({a, b});
                }
        }
        int uniqueCovered = 0;
        for (const auto& e : uniqueEdges) if (allBoundaryEdges.count(e)) ++uniqueCovered;
        std::fprintf(stderr,
                     "[group-anchors] groups=%zu anchors=%d groupsWithAccumulatedSegments=%d "
                     "coveredGroups=%d boundaryEdgesFromAnchors=%d uniqueCovered=%d/%d "
                     "uniqueCoverage=%.1f%% frozen=%zu\n",
                     groupAnchors.size(), n, segments, coveredGroups, traversed,
                     uniqueCovered, static_cast<int>(allBoundaryEdges.size()),
                     allBoundaryEdges.empty()
                         ? 0.0
                         : 100.0 * static_cast<double>(uniqueCovered) /
                               static_cast<double>(allBoundaryEdges.size()),
                     pmsf::sharedBoundaryFrozenVertices().size());
    }

    // [boundary topology compression prototype, env gated]
    // Permanently simplify the group boundary before the progressive rounds,
    // then compress the simplified mesh normally.  This is a lossy-boundary
    // prototype: the decoded mesh equals the pre-simplified input.
    if (std::getenv("PMSF_BOUNDARY_PRESIMPLIFY") && eliminate) {
        const char* maxEnv = std::getenv("PMSF_BOUNDARY_PRESIMPLIFY_MAX");
        const int maxOps = maxEnv ? std::max(0, std::atoi(maxEnv)) : 100000;
        int removed = 0;
        {
            int grouped = 0, total = 0, mixedV = 0;
            for (MCGAL::Facet* f : mesh->faces()) {
                if (!f || f->isRemoved()) continue;
                ++total;
                if (f->groupId() >= 0) ++grouped;
            }
            for (MCGAL::Vertex* v : mesh->vertices()) {
                if (!v || v->isRemoved()) continue;
                std::set<int> gs;
                for (MCGAL::Halfedge* rh : pmsf::boundedRing(v))
                    if (rh && rh->face() && !rh->face()->isRemoved()) gs.insert(rh->face()->groupId());
                if (gs.size() > 1) ++mixedV;
            }
            std::fprintf(stderr, "[boundary-presimplify] pre grouped=%d/%d mixedV=%d\n", grouped, total, mixedV);
        }
        for (int pass = 0; pass < 64 && removed < maxOps; ++pass) {
            std::vector<MCGAL::Halfedge*> cands;
            std::set<MCGAL::Vertex*> picked;
            for (MCGAL::Facet* f : mesh->faces()) {
                if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                MCGAL::Halfedge* h0 = f->proxyHalfedge();
                MCGAL::Halfedge* hh = h0;
                int guard = 0;
                do {
                    if (hh && hh->end_vertex() && !hh->end_vertex()->isRemoved()) {
                        MCGAL::Vertex* v = hh->end_vertex();
                        if (picked.insert(v).second) {
                            std::set<int> gids;
                            bool outer = false;
                            for (MCGAL::Halfedge* rh : pmsf::boundedRing(v)) {
                                if (!rh) continue;
                                if (rh->isBoundary()) outer = true;
                                if (rh->face() && !rh->face()->isRemoved())
                                    gids.insert(rh->face()->groupId());
                            }
                            if (gids.size() > 1) cands.push_back(hh);
                        }
                    }
                    hh = hh ? hh->next() : nullptr;
                } while (hh && hh != h0 && ++guard < 64);
            }
            if (pass == 0) std::fprintf(stderr, "[boundary-presimplify] pass0 cands=%zu\n", cands.size());
            if (cands.empty()) break;
            std::fprintf(stderr, "[boundary-presimplify] pass=%d cands=%zu removed=%d\n", pass, cands.size(), removed);
            int passRemoved = 0;
            for (MCGAL::Halfedge* h : cands) {
                if (!h || h->isRemoved() || !h->end_vertex() || h->end_vertex()->isRemoved()) continue;
                HalfedgeErrorEntry pe(h, MCGAL::Point(0, 0, 0), 0, 0);
                if (eliminate->eliminate(pe)) { ++removed; ++passRemoved; }
                if (removed >= maxOps) break;
            }
            if (passRemoved == 0) break;
        }
        mesh->garbage_collection();
        // Refresh seeds that pointed at removed boundary vertices: keep the
        // same group index but take the first surviving face of that group.
        for (std::size_t g = 0; g < seeds.size(); ++g) {
            MCGAL::Halfedge* s = seeds[g];
            bool dead = !s || s->isRemoved() || !s->face() || s->face()->isRemoved() ||
                        !s->vertex() || s->vertex()->isRemoved() ||
                        !s->end_vertex() || s->end_vertex()->isRemoved();
            if (!dead) continue;
            MCGAL::Halfedge* repl = nullptr;
            for (MCGAL::Facet* f : mesh->faces()) {
                if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                if (f->groupId() == static_cast<int>(g)) { repl = f->proxyHalfedge(); break; }
            }
            seeds[g] = repl;
        }
        // Drop elimination bookkeeping produced by the pre-pass; the main
        // progressive rounds must start from the simplified mesh as a clean
        // initial state.
        eliminate->init(mesh, header);
        int liveV = 0, liveF = 0;
        for (MCGAL::Vertex* v : mesh->vertices()) if (v && !v->isRemoved()) ++liveV;
        for (MCGAL::Facet* f : mesh->faces()) if (f && !f->isRemoved()) ++liveF;
        std::fprintf(stderr, "[boundary-presimplify] removed=%d V=%d F=%d\n", removed, liveV, liveF);
    }

    if (std::getenv("PMSF_BOUNDARY_STORAGE_STATS")) {
        const auto st = pmsf::measureBoundaryStorage(mesh);
        const double ratio = st.naiveBits > 0
                                 ? static_cast<double>(st.compactBits) /
                                       static_cast<double>(st.naiveBits)
                                 : 0.0;
        std::fprintf(stderr,
                     "[boundary-storage] groups=%d segments=%d boundaryV=%d boundaryE=%d "
                     "quantBits=%u naiveBits=%zu compactBits=%zu ratio=%.6f "
                     "anchorBits=%zu flags=%zu\n",
                     st.groupCount, st.segmentCount, st.boundaryVertexCount,
                     st.boundaryEdgeCount, st.quantBits, st.naiveBits, st.compactBits,
                     ratio, st.anchorBits, st.bitFlags);
    }
    if (std::getenv("PMSF_SEAM_TEST")) {
        mesh->dumpto_oldtype("/tmp/seam_input.off");
        auto cut = pmsf::SeamCutOperator::run(mesh, seeds);
        if (!cut.mesh) {
            std::cerr << "[seam-test] cut failed" << std::endl;
            std::exit(1);
        }
        if (std::getenv("PMSF_SEAM_CODEC_TEST")) {
            cut.mesh->dumpto_oldtype("/tmp/seam_codec_before.off");
            const auto bedges = pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(cut.mesh);
            bool collapsed = false;
            bool restored = false;
            for (MCGAL::Halfedge* h : bedges) {
                if (!h || h->isRemoved() || h->opposite() != nullptr) continue;
                MCGAL::Vertex* v = h->end_vertex();
                if (!v || v->isRemoved()) continue;
                const MCGAL::Point pos = v->point();
                MCGAL::Halfedge* newEdge = nullptr;
                if (!pmsf::collapseBoundaryVertex3Robust(cut.mesh, v, &newEdge)) continue;
                collapsed = true;
                if (newEdge) {
                    MCGAL::Vertex* nv = nullptr;
                    MCGAL::Halfedge* split = pmsf::splitBoundaryEdgeInsertVertex(cut.mesh, newEdge, pos, &nv);
                    restored = split && nv;
                }
                break;
            }
            cut.mesh->dumpto_oldtype("/tmp/seam_codec_after.off");
            std::cerr << "[seam-codec] collapsed=" << (collapsed?1:0)
                      << " restored=" << (restored?1:0)
                      << " V=" << cut.mesh->size_of_vertices()
                      << " F=" << cut.mesh->size_of_facets() << std::endl;
            std::exit(0);
        }
        if (std::getenv("PMSF_SEAM_FAN_CHEAP_TEST")) {
            cut.mesh->dumpto_oldtype("/tmp/fan_cheap_before.off");
            const auto bedges = pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(cut.mesh);
            std::cerr << "[fan-cheap] bedges=" << bedges.size() << std::endl;
            bool ok = false;
            int k = 0;
            int probes = 0;
            for (MCGAL::Halfedge* h : bedges) {
                if (!h || h->isRemoved() || h->opposite() != nullptr) continue;
                MCGAL::Vertex* v = h->end_vertex();
                if (!v || v->isRemoved()) continue;
                pmsf::BoundaryFanPatch probe;
                if (!pmsf::captureBoundaryFanPatch(cut.mesh, v, probe)) continue;
                ++probes;
                if (std::getenv("PMSF_SEAM_FAN_CHEAP_K3") && probe.origFaces.size() != 3) continue;
                if (probe.origFaces.size() < 2) continue;
                std::cerr << "[fan-cheap] probe fan=" << probe.origFaces.size() << std::endl;
                int gid = probe.origGroups.empty() ? 0 : probe.origGroups.front();
                MCGAL::Point removed, a, b;
                MCGAL::Halfedge* edge = nullptr;
                MCGAL::Point apexPos;
                if (!pmsf::removeBoundaryVertexFanCheap(cut.mesh, v, k, removed, a, b, &edge, &apexPos)) continue;
                bool restored = pmsf::restoreBoundaryVertexFanCheap(cut.mesh, edge, removed, k, gid, &apexPos);
                ok = restored;
                std::cerr << "[fan-cheap] k=" << k << " restored=" << (restored?1:0) << std::endl;
                break;
            }
            std::cerr << "[fan-cheap] probes=" << probes << std::endl;
            cut.mesh->dumpto_oldtype("/tmp/fan_cheap_after.off");
            std::cerr << "[fan-cheap] ok=" << (ok?1:0) << " V=" << cut.mesh->size_of_vertices()
                      << " F=" << cut.mesh->size_of_facets() << std::endl;
            std::exit(0);
        }

        if (std::getenv("PMSF_SEAM_FAN_TEST")) {
            cut.mesh->dumpto_oldtype("/tmp/seam_fan_before.off");
            const auto bedges = pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(cut.mesh);
            pmsf::BoundaryFanPatch fan;
            bool removed = false;
            for (MCGAL::Halfedge* h : bedges) {
                if (!h || h->isRemoved() || h->opposite() != nullptr) continue;
                MCGAL::Vertex* v = h->end_vertex();
                if (!v || v->isRemoved()) continue;
                pmsf::BoundaryFanPatch cand;
                if (!pmsf::captureBoundaryFanPatch(cut.mesh, v, cand)) continue;
                if (cand.origFaces.size() < 2) continue;
                fan = cand;
                removed = pmsf::removeBoundaryVertexFan(cut.mesh, v, fan);
                break;
            }
            bool restored = removed && pmsf::applyBoundaryFanPatch(cut.mesh, fan);
            cut.mesh->dumpto_oldtype("/tmp/seam_fan_after.off");
            std::cerr << "[seam-fan] removed=" << (removed?1:0)
                      << " restored=" << (restored?1:0)
                      << " orig=" << fan.origFaces.size()
                      << " post=" << fan.postFaces.size()
                      << " V=" << cut.mesh->size_of_vertices()
                      << " F=" << cut.mesh->size_of_facets() << std::endl;
            std::exit(0);
        }
        if (std::getenv("PMSF_SEAM_SHARED_TEST")) {
            cut.mesh->dumpto_oldtype("/tmp/seam_shared_cut1.off");
            auto shared = pmsf::SeamMergeOperator::mergeByPosition(cut.mesh);
            if (!shared) { std::cerr << "[seam-shared] merge failed" << std::endl; std::exit(1); }
            shared->dumpto_oldtype("/tmp/seam_shared_merged.off");
            auto cut2 = pmsf::SeamCutOperator::run(shared, {});
            if (!cut2.mesh) { std::cerr << "[seam-shared] recut failed" << std::endl; std::exit(1); }
            cut2.mesh->dumpto_oldtype("/tmp/seam_shared_cut2.off");
            std::cerr << "[seam-shared] cut1 V=" << cut.mesh->size_of_vertices()
                      << " F=" << cut.mesh->size_of_facets()
                      << " merged V=" << shared->size_of_vertices()
                      << " F=" << shared->size_of_facets()
                      << " cut2 V=" << cut2.mesh->size_of_vertices()
                      << " F=" << cut2.mesh->size_of_facets() << std::endl;
            std::exit(0);
        }
        auto merged = pmsf::SeamMergeOperator::run(cut.mesh, cut.originOfPoolId);
        if (!merged) {
            std::cerr << "[seam-test] merge failed" << std::endl;
            std::exit(1);
        }
        merged->dumpto_oldtype("/tmp/seam_merged.off");
        std::cerr << "[seam-test] inputV=" << mesh->size_of_vertices()
                  << " inputF=" << mesh->size_of_facets()
                  << " cutV=" << cut.mesh->size_of_vertices()
                  << " cutF=" << cut.mesh->size_of_facets()
                  << " mergedV=" << merged->size_of_vertices()
                  << " mergedF=" << merged->size_of_facets()
                  << " seeds=" << cut.seeds.size() << std::endl;
        std::exit(0);
    }

    if (const char* scTest = std::getenv("PMSF_SC_BOUNDARY_TEST")) {
        const int rc = pmsf::runSingleBoundaryRoundTripTest(mesh, std::atoi(scTest));
        std::exit(rc);
    }
    if (const char* scPos = std::getenv("PMSF_SC_BOUNDARY_POS_TEST")) {
        const int rc = pmsf::runSingleBoundaryPositionalRoundTripTest(mesh, std::atoi(scPos));
        std::exit(rc);
    }
    if (const char* scCompact = std::getenv("PMSF_SC_BOUNDARY_COMPACT_TEST")) {
        const int rc = pmsf::runSingleBoundaryCompactRoundTripTest(mesh, std::atoi(scCompact));
        std::exit(rc);
    }
    if (const char* scAnchor = std::getenv("PMSF_SC_BOUNDARY_ANCHOR_TEST")) {
        const int rc = pmsf::runAnchorRoundTripTest(mesh, std::atoi(scAnchor));
        std::exit(rc);
    }

    std::vector<int> preComplexFanResiduals;
    int preComplexFanCount = 0;
    if (std::getenv("PMSF_SEAM_COMPRESS")) {
        auto cut = pmsf::SeamCutOperator::run(mesh, seeds);
        if (!cut.mesh || cut.seeds.empty()) {
            std::cerr << "[seam-compress] cut failed" << std::endl;
            std::exit(1);
        }
        mesh = cut.mesh;
        seeds = cut.seeds;
        seed = seeds[0];
        boundaryGraph = nullptr;
        if (std::getenv("PMSF_SEAM_PRE_COMPLEX_FAN")) {
            std::vector<int> fanResiduals;
            int applied = 0;
            const int cap = std::atoi(std::getenv("PMSF_SEAM_PRE_COMPLEX_FAN_MAX") ? std::getenv("PMSF_SEAM_PRE_COMPLEX_FAN_MAX") : "64");
            for (int iter = 0; iter < cap; ++iter) {
                mesh->garbage_collection();
                std::unordered_map<int,int> inc;
                for (MCGAL::Facet* f : mesh->faces()) {
                    if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                    std::unordered_set<int> seen;
                    MCGAL::Halfedge* h0 = f->proxyHalfedge();
                    MCGAL::Halfedge* h = h0; int guard = 0;
                    do { if (h && h->vertex() && seen.insert(h->vertex()->poolId()).second) ++inc[h->vertex()->poolId()]; h = h->next(); } while (h && h != h0 && ++guard < 64);
                }
                MCGAL::Vertex* chosen = nullptr;
                for (MCGAL::Halfedge* h : pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(mesh)) {
                    if (!h || h->isRemoved() || h->opposite() != nullptr) continue;
                    MCGAL::Vertex* v = h->end_vertex();
                    if (!v || v->isRemoved()) continue;
                    if (seed && seed->vertex() && seed->end_vertex() && (v == seed->vertex() || v == seed->end_vertex())) continue;
                    if (inc[v->poolId()] >= 3) { chosen = v; break; }
                }
                if (!chosen) break;
                int k = 0; MCGAL::Point removed, a, b, apex; MCGAL::Halfedge* e = nullptr;
                if (!pmsf::removeBoundaryVertexFanManual(mesh, chosen, k, removed, a, b, &e, &apex)) break;
                if (!e || k < 3) break;
                const auto qv = mesh->floatPosToInt(removed);
                const auto qa = mesh->floatPosToInt(a);
                const auto qb = mesh->floatPosToInt(b);
                auto qe = mesh->floatPosToInt(apex);
                auto qev = mesh->floatPosToInt(e->vertex()->point());
                const int apexSide = (qe == qev) ? 1 : -1;
                const int gid = e->face() ? e->face()->groupId() : 0;
                fanResiduals.insert(fanResiduals.end(), {k, qv[0], qv[1], qv[2], qa[0], qa[1], qa[2], qb[0], qb[1], qb[2], apexSide, gid});
                ++applied;
            }
            if (applied > 0) {
                std::vector<int> payload;
                payload.push_back(applied);
                payload.insert(payload.end(), fanResiduals.begin(), fanResiduals.end());
                preComplexFanResiduals = payload;
                preComplexFanCount = applied;
            }
            if (std::getenv("PMSF_SEAM_DEBUG"))
                std::cerr << "[pre-complex-fan] applied=" << applied << std::endl;
        }
        if (serializer && !std::getenv("PMSF_SEAM_SHARED_BASE") &&
            std::getenv("PMSF_SEAM_BASE_DEDUP"))
            serializer->setSeamCutBase(true);
        if (select) {
            if (select->options_.getRegisterFunc()) {
                select->options_.getRegisterFunc()(mesh);
            }
            select->init(mesh);
        }
        if (eliminate) eliminate->init(mesh, header);
        if (std::getenv("PMSF_SEAM_DEBUG")) {
            std::cerr << "[seam-compress] cutV=" << mesh->size_of_vertices()
                      << " cutF=" << mesh->size_of_facets()
                      << " groups=" << cut.groupCount << std::endl;
        }
    }

    // 独立旁证（P18.2 阶段 A）：分区阶段到底有没有真的给 facet 写 groupId、
    // 判据：groups == 请求的分区数，unassigned == 0，seeds.size() == groups。

    // [PHASE59 诊断，默认关闭] 分区 seed 身份探针（step 10，全网格）。

    collector->init(mesh, seeds, header);
    if (probeCollector) probeCollector->init(mesh, seeds, header);
    // P18.2 路线 (a)：**分区种子不进消除决策**。
    //
    // `SeedIsRemovableOperator::isRemovable` 会把「与任一 seed 半边共端点」的
    // 边一律判为不可移除（`SeedIsRemovableOperator.h:28-40`）。于是
    // **G 个种子 ≠ 1 个种子** ⇒ 消除顺序改变 ⇒ 基网不同。
    //
    // 附 16 实测：摘掉它之后两遍的 `_base_mesh.parquet` **仍然**不同
    // （121218 vs 121174 字节；摘之前是 121555）⇒ 它只是**次要**原因。主因是
    // `BoundaryIsRemovableOperator::check`（`BoundaryIsRemovableOperator.h:55-79`）
    // 直接用 `groupId` 判分区边界 —— **分组参与消除决策，这是论文的设计意图，
    // 不是 bug**。故「两侧各自重算分组」整条路不可达（分组 → 消除 → 基网 →
    // 分组，成环）；本改动仍然保留，因为它让「分区 seeds 不干扰消除」
    // 这件事成为可分辨的、独立的变量。
    //
    // 默认路径零影响：该路径下 `seeds` 就是 `{seed}`，两者逐元素相同。
    // N=1 的旧分区路径同样不受影响（只在 `segmentGroupTableIn` 非空时改）。
    // P18.2 PHASE59 附 19：**主循环 seed 必须**在保护集里 —— 这是框架一直依赖
    // 的不变式，而接上分区阶段之后被打破了。
    //
    // 打破的路径：`segmentOperatorFactory` 非空时，上面把 `seeds` 换成了
    // `segmentOperator->exportSeeds()`。若 `seedProtection` 直接取 `seeds`，
    // 保护的就成了**分区器的** seed 而不是主循环 `selectSeed()` 的那一条 ——
    // 实测两者是**不同的半边**：
    //     主循环 seed   hid=175592 vA=32640 vB=9645
    //     exportSeeds[0] hid=77522  vA=26050 vB=1507
    // ⇒ 主循环 seed 边**在第 0 轮就被消除**，此后每轮
    // `collector->collect(seed)` 都是从一个**已死的半边**起 BFS，而
    // `:1407` 把这条 seed 写进基网表、解码端从一条**活的**半边起 BFS
    // ⇒ 两侧从第 0 轮就分道扬镳（实测解码 26,341/32,506 vs 原 34,834/69,664）。
    //
    // 修法只做一件事：把主循环 seed **并进**保护集，别的一概不动。
    // 实测（附 19）：`efoss segments:1` 由 26,341/32,506 变为 **34,834/69,664 精确**，
    // 且与 `default segments:1` **逐字节相同**（md5 d3215950…）；
    // `segments:2` 的产物**逐字节不变**（md5 09f92ef1…）⇒ 不会回归分区路径。
    // ⚠️ 保护集**恒为单元素 `{seed}`**，与 `seeds` / `segmentGroupTableIn` 无关。
    // 第一版写成「`seeds` 再并进 `seed`」是**错的** —— 那会把
    // `segmentGroupTableIn` 路径从 `{seed}` 变成 `seeds ∪ {seed}`（9 条），
    // 实测 `segments:2` 产物 md5 由 bbff038f… 变成 a61ee62c…（**已变**，不是
    // 附 16 说的「不变」）。故回到 `{seed}`：这条路与改动前**逐元素相同**。
    // PHASE61 物品 1 ①：**测量用**开关（env 门控，默认关闭）。
    //
    // 附 30 实测：摘掉 BoundaryIsRemovableOperator 后基网 8238→593、BPV 55.53→35.26
    // （进带），但 decode abort —— 3 个分区在基网上消失、符号流无落脚点。
    // 假说：**不必保护整条分区边界，只要保证每组在基网上有一个立足点**即可。
    // 本开关把保护集从主循环单 seed 换成**每组的 seed**（G 条），
    // 实测它能否既让基网塌下去、又让回环成立。
    std::vector<MCGAL::Halfedge*> seedProtection{seed};
    if (std::getenv("PMSF_PROTECT_GROUP_SEEDS") != nullptr && seeds.size() > 1) {
        seedProtection = seeds;
    }
    // P18.2 PHASE59：**分区 seed 也必须进保护集**，理由与上面主循环 seed 完全相同。
    //
    // 实测（原分区 seed 身份探针，已移除）：分区遍的两个 seed 半边到轮循环结束时
    //      g=0 q=(2005,1656,2953)/(1984,1655,2936) removedA=1 removedB=1 hremoved=1
    //      g=1 q=(604,3367,524)/(610,3330,524)   removedA=1 removedB=1 hremoved=1
    // **两条都被消除**（对照：主循环 seed 受保护后 removed=0）。
    // 而解码端的 seed 是 BFS 的根（`OptimizedSymbolReadOperator::collectFromDecoded`
    // 里 `pushGate(seed)`），根不对 ⇒ 遍历序整体错位 ⇒ 重建不出顶点。
    //
    // 判据取 `seeds.size() > 1`：默认路径 `seeds` 恒为 `{seed}`（size 1），
    // 单遍 `segmentOperatorFactory` 路径也不进这里，故**默认路径逐字节不变**。
    if (seeds.size() > 1) {
        for (MCGAL::Halfedge* s : seeds) {
            if (s)
                seedProtection.push_back(s);
        }
    }
    seedIsRemovableOperator->init({}, seedProtection, {});
    select->addIsRemovableOperator(seedIsRemovableOperator);
    // PHASE61 物品 1 ①：**每组一个代表面**，禁止消掉它（env 门控，默认关闭）。
    // 见 GroupFootholdIsRemovableOperator.h 的实测背景。
    if (std::getenv("PMSF_GROUP_FOOTHOLD") != nullptr && seeds.size() > 1) {
        std::vector<MCGAL::Facet*> footholds(seeds.size(), nullptr);
        for (MCGAL::Facet* f : mesh->faces()) {
            if (f == nullptr || f->isRemoved()) continue;
            const int g = f->groupId();
            if (g < 0 || static_cast<std::size_t>(g) >= footholds.size()) continue;
            if (footholds[static_cast<std::size_t>(g)] == nullptr) {
                footholds[static_cast<std::size_t>(g)] = f;
            }
        }
        auto footholdOp = std::make_shared<GroupFootholdIsRemovableOperator>();
        footholdOp->setFootholds(std::move(footholds));
        select->addIsRemovableOperator(footholdOp);
    }

    // ---------- 10b) cfg.featureDetector（Phase A.3 partial wiring） ----------
    // 当算法主函数通过 Builder::withFeatureDetector(...) 提供了 FeatureDetector
    // 时，框架自动：
    //   1. 在 original mesh 上跑 detect() 一次（per-算法的 detector 自行决定
    //      是否缓存；FeatureSet 是廉价的 unordered_set<uint32_t>）；
    //   2. 把结果挂到 FeatureMaskIsRemovableOperator 上并追加到 SelectOperator
    //      的可移除算子链末端 —— 这意味着"落在特征集内的顶点塌缩会被 veto"。
    // 既有算法（AnchorPreprocessor、cluster-guard 等）保持不动，因为它们在
    // CompressOptions.preProcessOperator 链里已经手动注入了 FeatureMaskIs
    // RemovableOperator 或等价的 setNotRemovable 语义；这里只是给 **没接入** 的
    // 新算法一条零 LOC 捷径。
    if (cfg.featureDetector) {
        auto featGuard = std::make_shared<FeatureMaskIsRemovableOperator>(cfg.featureDetector);
        featGuard->setMesh(mesh);
        featGuard->init({}, {}, {});
        select->addIsRemovableOperator(featGuard);
    }

    // ---------- 11) beforeCompressLoop（PPMC 下传 prediction flag 等） ----------
    if (cfg.beforeCompressLoop) {
        cfg.beforeCompressLoop(mesh, header, collector, options);
    }
    // header 至此定稿（算法可能刚改写它）。序列化器里「必须与 header 一致」的
    // 开关在这一刻对齐 —— 解码端读 header 决定导入路径，序列化器必须同源。
    if (serializer) {
        serializer->onHeaderFinalized(header);
    }

    // ---------- 12) 独立集选择器（PCATM 专属） ----------
    IndependentSetSelector misSelector;
    if (cfg.enableIndependentSet) {
        misSelector.init(mesh);
    }

    // ---------- 13) 主循环 ----------
    // B.1 gate-level conquest 支持：每轮聚合 eliminate 产生的 delta gates
    // （每次 eliminate(entry) 后 entry.edge 被 VertexRemovalEliminateOperator
    //  重置为 `h->face()->proxyHalfedge()`，即本轮 polyFace 的 proxy handle）。
    // 这份 gate 列表作为 hint 传给 collector；BFS-based collector 默认忽略，
    // 只有 gate-level collector（B.1 的 RDSymbolCollectOperator）会用到。
    std::vector<MCGAL::Halfedge*> perRoundGates;
    std::vector<int> edgeFlipRoundCounts;
    std::vector<std::pair<int, int>> edgeFlipSymbols;
    std::vector<EdgeFlipPostProcessOperator::CoordSymbol> edgeFlipCoordSymbols;
    std::vector<int> edgeFlipRoundBitCounts;
    std::vector<int> boundaryRoundBitCounts;
    std::vector<unsigned char> boundaryRoundBits;
    std::vector<int> boundaryRoundResiduals;
    if (!preComplexFanResiduals.empty()) {
        boundaryRoundBitCounts.assign(static_cast<size_t>(std::max(1, options.getRound())), 0);
        boundaryRoundBits.clear();
        boundaryRoundResiduals = preComplexFanResiduals;
    }
    std::vector<unsigned char> edgeFlipRoundBits;
    const int interleavedFlipRoundsLimit = options.getEdgeFlipPostProcessRounds();
    const bool dumpPipelineRounds = envOn("PMSF_PIPELINE_DEBUG_DUMP");
    const bool dumpEdgeFlipRounds =
        dumpPipelineRounds || envOn("PMSF_EDGEFLIP_DEBUG_DUMP");
    // P18.2 路线 (a) 第 3 步前置：**坍缩记录**。
    //
    // 目标是「把基网的分区精确回填到全网格」。附 11 我曾声称
    // `contextPool.operations` 已记着坍缩关系，**附 11b 实测证伪**
    // （该容器在本路径上 `total_ops=0`）。故这里自己记：
    // 每次 `eliminate` 成功后，记下**被移除顶点 → 存活顶点**。
    //
    // 记录点必须覆盖**全部三处** eliminate 调用（:790/:838/:854）；
    // 漏一处就会静默少记，回填时表现为「部分面没有 groupId」。
    // `PMSF_SEG_COLLAPSE=1` 开启（默认关闭，零行为影响）。
    //
    // P18.2 路线 (a)：**写表的那一遍必须自动开**（`segmentGroupTableOut` 非空）
    // —— 否则记录为空，回填全落空。实测过：忘了这条时
    // `recorded=0 removed_vertices=28893 MISMATCH`，而表写出来只剩存活的 5941 个。
    const bool segCollapseTrace =
        std::getenv("PMSF_SEG_COLLAPSE") != nullptr ||
        (cfg.segmentGroupTableOut != nullptr);
    std::unordered_map<int, int> segCollapseMap;  // removedVertexPoolId → survivorPoolId
    // ⚠️ 端点必须在 `eliminate` **之前**取。
    //
    // 第一版是「eliminate 之后拿 entry.edge 再取端点」—— **实测 recorded=0**
    // （自检：removed_vertices=27717）。原因：`entry.edge` 会被 eliminate
    // **重置**成 polyFace->proxyHalfedge()（见 SymbolCollectOperator.h 的
    // gate 注释），故之后它已不是被坍缩的那条边。
    // 这正是「想当然」会做错、而自检能立刻抓住的那类错误。
    auto recordCollapse = [&](MCGAL::Vertex* a, MCGAL::Vertex* b) {
        if (!segCollapseTrace) return;
        if (a != nullptr && a->isRemoved() && b != nullptr) {
            segCollapseMap[a->poolId()] = b->poolId();
        } else if (b != nullptr && b->isRemoved() && a != nullptr) {
            segCollapseMap[b->poolId()] = a->poolId();
        }
    };

    if (std::getenv("PMSF_BOUNDARY_PRE_REMOVE")) {
        const auto preEdges = pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(mesh);
        MCGAL::Vertex* chosen = nullptr;
        for (MCGAL::Halfedge* h : preEdges) {
            if (!h || h->isRemoved() || h->opposite() != nullptr) continue;
            MCGAL::Vertex* v = h->end_vertex();
            if (!v || v->isRemoved() || v->vertex_degree() != 3) continue;
            if (seed && seed->vertex() && seed->end_vertex() &&
                (v == seed->vertex() || v == seed->end_vertex())) continue;
            int boundaryIncident = 0;
            int incidentFaces = 0;
            bool strictOk = true;
            for (MCGAL::Halfedge* vh : pmsf::boundedRing(v)) {
                if (!vh || vh->isRemoved() || !vh->face()) { strictOk = false; continue; }
                ++incidentFaces;
                if (vh->opposite() == nullptr) ++boundaryIncident;
                if (vh->face()->facet_degree() != 3) strictOk = false;
            }
            if (!strictOk || boundaryIncident != 2 || incidentFaces != 2) continue;
            chosen = v;
            break;
        }
        MCGAL::Halfedge* newEdge = nullptr;
        MCGAL::PointInt removedInt(0, 0, 0);
        MCGAL::PointInt edgeAInt(0, 0, 0);
        MCGAL::PointInt edgeBInt(0, 0, 0);
        if (chosen) {
            removedInt = mesh->floatPosToInt(chosen->point());
            pmsf::collapseBoundaryVertex3Robust(mesh, chosen, &newEdge);
            if (newEdge && newEdge->vertex() && newEdge->end_vertex()) {
                edgeAInt = mesh->floatPosToInt(newEdge->vertex()->point());
                edgeBInt = mesh->floatPosToInt(newEdge->end_vertex()->point());
            }
        }
        const auto postEdges = pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(mesh);
        std::vector<unsigned char> decisions(postEdges.size(), 0);
        if (newEdge) {
            for (size_t ei = 0; ei < postEdges.size(); ++ei) {
                if (postEdges[ei] == newEdge) { decisions[ei] = 1; break; }
            }
        }
        const int rounds = std::max(1, options.getRound());
        boundaryRoundBitCounts.assign(static_cast<size_t>(rounds), 0);
        if (!decisions.empty()) boundaryRoundBitCounts[0] = static_cast<int>(decisions.size());
        boundaryRoundBits.assign(decisions.size(), 0);
        for (size_t bi = 0; bi < decisions.size(); ++bi) boundaryRoundBits[bi] = decisions[bi];
        boundaryRoundResiduals.clear();
        if (newEdge) {
            boundaryRoundResiduals = {edgeAInt.x(), edgeAInt.y(), edgeAInt.z(),
                                      edgeBInt.x(), edgeBInt.y(), edgeBInt.z(),
                                      removedInt.x(), removedInt.y(), removedInt.z()};
        }
        if (std::getenv("PMSF_BOUNDARY_DEBUG"))
            std::cerr << "[boundary-pre] pre=" << preEdges.size() << " post=" << postEdges.size()
                      << " chosen=" << (chosen ? 1 : 0) << " newEdge=" << (newEdge ? 1 : 0) << std::endl;
    }

    auto runOrderBitsBoundaryAtEnd = [&](int roundIdx) {
        std::vector<unsigned char> bits;
        std::vector<int> residuals;
        mesh->garbage_collection();
        auto bedges = pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(mesh);
        auto incidentCount = [&](MCGAL::Vertex* vv) {
            int n = 0;
            for (MCGAL::Facet* f : mesh->faces()) {
                if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                MCGAL::Halfedge* h0 = f->proxyHalfedge();
                MCGAL::Halfedge* hh = h0;
                int gg = 0;
                do { if (hh && hh->vertex() == vv) { ++n; break; } hh = hh->next(); } while (hh && hh != h0 && ++gg < 64);
            }
            return n;
        };
        if (std::getenv("PMSF_SEAM_COMPLEX_FAN") && roundIdx == 0) {
            for (size_t ci = 0; ci < bedges.size(); ++ci) {
                MCGAL::Halfedge* ch = bedges[ci];
                if (!ch || ch->isRemoved() || ch->opposite() != nullptr) continue;
                MCGAL::Vertex* cv = ch->end_vertex();
                if (!cv || cv->isRemoved()) continue;
                if (incidentCount(cv) == 3) {
                    std::swap(bedges[0], bedges[ci]);
                    break;
                }
            }
        }
        for (MCGAL::Halfedge* h : bedges) {
            if (!h || h->isRemoved() || h->opposite() != nullptr) continue;
            MCGAL::Vertex* v = h->end_vertex();
            if (!v || v->isRemoved()) continue;
            if (seed && seed->vertex() && seed->end_vertex() &&
                (v == seed->vertex() || v == seed->end_vertex())) continue;
            const int k = incidentCount(v);
            if (k < 2) continue;
            if (k > 2 && (!std::getenv("PMSF_SEAM_COMPLEX_FAN") || roundIdx != 0)) continue;
            if (k > 3) continue;  // complex-fan path currently validated for k=3 only
            if (k == 3) {
                int incident = 0;
                for (MCGAL::Facet* f : mesh->faces()) {
                    if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                    MCGAL::Halfedge* h0 = f->proxyHalfedge();
                    MCGAL::Halfedge* hh = h0;
                    int gg = 0;
                    do { if (hh && hh->vertex() == v) { ++incident; break; } hh = hh->next(); } while (hh && hh != h0 && ++gg < 64);
                }
                if (incident != k) continue;
            }
            const MCGAL::PointInt oldInt = mesh->floatPosToInt(v->point());
            MCGAL::Point removed, a, b, apexPos;
            MCGAL::Halfedge* newEdge = nullptr;
            int appliedK = k;
            int sign = 1;
            if (k == 2) {
                if (!pmsf::collapseBoundaryVertex3Robust(mesh, v, &newEdge)) continue;
                if (!newEdge) continue;
                const MCGAL::PointInt ai = mesh->floatPosToInt(newEdge->vertex()->point());
                const MCGAL::PointInt bi = mesh->floatPosToInt(newEdge->end_vertex()->point());
                removed = mesh->intPosToFloat(oldInt);
                a = mesh->intPosToFloat(ai); b = mesh->intPosToFloat(bi);
            } else {
                std::unordered_map<int, MCGAL::Vertex*> vmap;
                auto trial = cloneMeshExactForTrial(mesh, &vmap);
                if (!trial) continue;
                auto itV = vmap.find(v->poolId());
                if (itV == vmap.end()) continue;
                const int preB = countBoundaryHalfedges(mesh);
                if (std::getenv("PMSF_SEAM_DEBUG")) std::cerr << "[clone-quality] meshPaired=" << countPairedHalfedges(mesh) << " trialPaired=" << countPairedHalfedges(trial) << std::endl;
                int tk = 0; MCGAL::Point tr, ta, tb, tap; MCGAL::Halfedge* te = nullptr;
                const bool trialOk = pmsf::removeBoundaryVertexFanCheap(trial, itV->second, tk, tr, ta, tb, &te, &tap);
                const int trialPost = countBoundaryHalfedges(trial);
                if (std::getenv("PMSF_SEAM_DEBUG")) std::cerr << "[complex-trial] preB=" << preB << " k=" << appliedK << " ok=" << (trialOk?1:0) << " tk=" << tk << " postB=" << trialPost << std::endl;
                if (!trialOk) continue;
                if (tk != appliedK) continue;
                if (trialPost != preB - 1) continue;
                if (!pmsf::removeBoundaryVertexFanCheap(mesh, v, appliedK, removed, a, b, &newEdge, &apexPos)) continue;
                if (countBoundaryHalfedges(mesh) != preB - 1) continue;
                auto qa = mesh->floatPosToInt(apexPos);
                auto qv = mesh->floatPosToInt(newEdge->vertex()->point());
                sign = (qa == qv) ? 1 : -1;
            }
            const auto order = pmsf::collectBoundaryEdgesInOrder(mesh);
            int edgePos = -1;
            for (size_t oi = 0; oi < order.size(); ++oi) {
                if (order[oi] == newEdge) { edgePos = static_cast<int>(oi); break; }
            }
            if (edgePos < 0) continue;
            bits.assign(order.size(), 0);
            bits[static_cast<size_t>(edgePos)] = 1;
            const MCGAL::PointInt ai = mesh->floatPosToInt(newEdge->vertex()->point());
            const MCGAL::PointInt bi = mesh->floatPosToInt(newEdge->end_vertex()->point());
            const MCGAL::PointInt mid((ai.x() + bi.x()) / 2, (ai.y() + bi.y()) / 2, (ai.z() + bi.z()) / 2);
            residuals.push_back(sign * appliedK);
            residuals.push_back(oldInt.x() - mid.x());
            residuals.push_back(oldInt.y() - mid.y());
            residuals.push_back(oldInt.z() - mid.z());
            break;
        }
        boundaryRoundBitCounts.push_back(static_cast<int>(bits.size()));
        boundaryRoundBits.insert(boundaryRoundBits.end(), bits.begin(), bits.end());
        boundaryRoundResiduals.insert(boundaryRoundResiduals.end(), residuals.begin(), residuals.end());
        if (std::getenv("PMSF_SEAM_DEBUG"))
            std::cerr << "[seam-boundary-orderbits-round-end] round=" << roundIdx
                      << " boundaryEdges=" << bits.size() << std::endl;
    };
    auto runOrderedBoundaryAtEnd = [&](int roundIdx) {
        std::vector<unsigned char> bits;
        std::vector<int> residuals;
        static std::set<std::array<int, 3>> spacingQ;
        if (const char* endRounds = std::getenv("PMSF_SEAM_BOUNDARY_END_ROUNDS")) {
            const std::string rs(endRounds);
            bool enabled = false;
            std::size_t pos = 0;
            while (pos <= rs.size()) {
                const std::size_t comma = rs.find(',', pos);
                const std::string tok = rs.substr(
                    pos, comma == std::string::npos ? std::string::npos : comma - pos);
                if (!tok.empty() && std::atoi(tok.c_str()) == roundIdx) { enabled = true; break; }
                if (comma == std::string::npos) break;
                pos = comma + 1;
            }
            if (!enabled) {
                if (std::getenv("PMSF_SEAM_GC_DISABLED_ROUNDS")) mesh->garbage_collection();
                boundaryRoundBitCounts.push_back(0);
                return;
            }
        }
        mesh->garbage_collection();
        const auto bedges = pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(mesh);
        const auto gidByPos = pmsf::buildVertexPosGroups(mesh);
        for (MCGAL::Halfedge* h : bedges) {
            if (!h || h->isRemoved() || h->opposite() != nullptr) continue;
            MCGAL::Vertex* v = h->end_vertex();
            if (!v || v->isRemoved()) continue;
            if (seed && seed->vertex() && seed->end_vertex() &&
                (v == seed->vertex() || v == seed->end_vertex())) continue;
            if (!pmsf::isSeamBoundaryVertex(gidByPos, mesh, v)) continue;
            std::vector<MCGAL::Vertex*> spacingRing;
            spacingRing.push_back(v);
            std::set<MCGAL::Vertex*> depth1;
            for (MCGAL::Halfedge* rh : pmsf::boundedRing(v))
                if (rh && rh->end_vertex() && !rh->end_vertex()->isRemoved())
                    depth1.insert(rh->end_vertex());
            for (MCGAL::Vertex* d1 : depth1) spacingRing.push_back(d1);
            for (MCGAL::Vertex* d1 : depth1)
                for (MCGAL::Halfedge* rh : pmsf::boundedRing(d1))
                    if (rh && rh->end_vertex() && !rh->end_vertex()->isRemoved() &&
                        rh->end_vertex() != v)
                        spacingRing.push_back(rh->end_vertex());
            bool spacingConflict = false;
            for (MCGAL::Vertex* rv : spacingRing) {
                const auto rq = mesh->floatPosToInt(rv->point());
                if (spacingQ.count({rq.x(), rq.y(), rq.z()})) { spacingConflict = true; break; }
            }
            if (spacingConflict) continue;
            const MCGAL::PointInt oldInt = mesh->floatPosToInt(v->point());
            MCGAL::Halfedge* newEdge = nullptr;
            if (!pmsf::collapseBoundaryVertex3Robust(mesh, v, &newEdge)) continue;
            if (!newEdge || !newEdge->vertex() || !newEdge->end_vertex()) continue;
            const auto postEdges = pmsf::edgeflip::collectMeshBoundaryEdgesSorted(mesh);
            int edgeIndex = -1;
            for (size_t ei = 0; ei < postEdges.size(); ++ei) {
                if (postEdges[ei] == newEdge) { edgeIndex = static_cast<int>(ei); break; }
            }
            if (edgeIndex < 0) continue;
            const MCGAL::PointInt ai = mesh->floatPosToInt(newEdge->vertex()->point());
            const MCGAL::PointInt bi = mesh->floatPosToInt(newEdge->end_vertex()->point());
            if (std::getenv("PMSF_SEAM_DEBUG"))
                std::cerr << "[seam-boundary-round-end-op] round=" << roundIdx
                          << " edgeIndex=" << edgeIndex << " postEdges=" << postEdges.size()
                          << " V=" << mesh->size_of_vertices()
                          << " a=" << ai.x() << "," << ai.y() << "," << ai.z()
                          << " b=" << bi.x() << "," << bi.y() << "," << bi.z()
                          << " old=" << oldInt.x() << "," << oldInt.y() << "," << oldInt.z()
                          << " gid=" << (newEdge->face() ? newEdge->face()->groupId() : -1)
                          << " c=" << (newEdge->next() && newEdge->next()->end_vertex()
                                            ? (std::to_string(newEdge->next()->end_vertex()->poolId()))
                                            : std::string("-"))
                          << std::endl;
            const MCGAL::PointInt mid((ai.x() + bi.x()) / 2, (ai.y() + bi.y()) / 2, (ai.z() + bi.z()) / 2);
            bits.push_back(1);
            // mode13 stride 9: target edge endpoints (a,b) + removed position residual
            residuals.push_back(ai.x());
            residuals.push_back(ai.y());
            residuals.push_back(ai.z());
            residuals.push_back(bi.x());
            residuals.push_back(bi.y());
            residuals.push_back(bi.z());
            residuals.push_back(newEdge->face() ? newEdge->face()->groupId() : -1);
            residuals.push_back(oldInt.x() - mid.x());
            residuals.push_back(oldInt.y() - mid.y());
            residuals.push_back(oldInt.z() - mid.z());
            for (MCGAL::Vertex* rv : spacingRing) {
                if (!rv) continue;
                const auto rq = mesh->floatPosToInt(rv->point());
                spacingQ.insert({rq.x(), rq.y(), rq.z()});
            }
            for (MCGAL::Vertex* ev : {newEdge->vertex(), newEdge->end_vertex()}) {
                if (!ev) continue;
                const auto eq = mesh->floatPosToInt(ev->point());
                spacingQ.insert({eq.x(), eq.y(), eq.z()});
            }
            if (std::getenv("PMSF_SEAM_REBUILD_RINGS_AFTER_OP")) {
                pmsf::rebuildVertexHalfedgeRings(mesh);
            }
            // A boundary collapse can leave one edge with two unpaired halfedges
            // (a topological hole) while the serialized base mesh is re-paired by
            // the decoder; keep encoder/decoder pairing identical.
            pmsf::repairHalfedgePairing(mesh);
            if (newEdge->next() && newEdge->next()->end_vertex()) {
                const auto cq = mesh->floatPosToInt(newEdge->next()->end_vertex()->point());
                spacingQ.insert({cq.x(), cq.y(), cq.z()});
            }
            break;
        }
        boundaryRoundBitCounts.push_back(static_cast<int>(bits.size()));
        boundaryRoundBits.insert(boundaryRoundBits.end(), bits.begin(), bits.end());
        boundaryRoundResiduals.insert(boundaryRoundResiduals.end(), residuals.begin(), residuals.end());
        if (std::getenv("PMSF_SEAM_DEBUG"))
            std::cerr << "[seam-boundary-round-end] round=" << roundIdx
                      << " applied=" << bits.size() << " F=" << mesh->size_of_facets() << std::endl;
    };
    if (std::getenv("PMSF_BOUNDARY_ENUM_TEST") && mesh && seed) {
        mesh->garbage_collection();
        pmsf::rebuildVertexHalfedgeRings(mesh);
        const auto all = pmsf::edgeflip::collectBoundaryEdgesSeedBfs(mesh, seed);
        std::set<int> fullIds;
        MCGAL::Halfedge* first = nullptr;
        int gA = -1, gB = -1;
        for (MCGAL::Halfedge* h : all) {
            if (!h || h->isRemoved() || !h->opposite()) continue;
            if (!h->face() || !h->opposite()->face()) continue;
            if (h->face()->isRemoved() || h->opposite()->face()->isRemoved()) continue;
            if (h->face()->groupId() == h->opposite()->face()->groupId()) continue;
            fullIds.insert(h->poolId());
            if (!first) { first = h; gA = h->face()->groupId(); gB = h->opposite()->face()->groupId(); }
        }
        if (first) {
            MCGAL::Vertex* anchor = first->vertex();
            int ga = std::min(gA, gB), gb = std::max(gA, gB);
            MCGAL::Halfedge* start = pmsf::findAnchorHalfedge(mesh.get(), anchor, ga, gb);
            if (!start) start = pmsf::findAnchorHalfedge(mesh.get(), anchor, gb, ga);
            std::vector<MCGAL::Halfedge*> loop;
            if (start) loop = pmsf::enumerateGroupBoundaryChain(mesh.get(), start, start->face()->groupId(), start->opposite()->face()->groupId());
            auto edgeKey = [](MCGAL::Halfedge* h) { int a = h->vertex() ? h->vertex()->poolId() : -1; int b = h->end_vertex() ? h->end_vertex()->poolId() : -2; if (a > b) std::swap(a, b); return std::make_pair(a, b); };
            std::set<std::pair<int,int>> fullEdges;
            for (MCGAL::Halfedge* h : all) {
                if (!h || h->isRemoved() || !h->opposite() || !h->face() || !h->opposite()->face()) continue;
                if (h->face()->isRemoved() || h->opposite()->face()->isRemoved()) continue;
                int x = h->face()->groupId(), y = h->opposite()->face()->groupId();
                if (x == y) continue;
                if (std::min(x,y) == ga && std::max(x,y) == gb) fullEdges.insert(edgeKey(h));
            }
            std::set<std::pair<int,int>> loopEdges;
            for (MCGAL::Halfedge* h : loop) loopEdges.insert(edgeKey(h));
            int outside = 0;
            for (auto& e : loopEdges) if (!fullEdges.count(e)) ++outside;
            int missing = 0;
            for (auto& e : fullEdges) if (!loopEdges.count(e)) ++missing;
        }
    }

    if (std::getenv("PMSF_BOUNDARY_ENUM_ALL") && mesh && seed) {
        mesh->garbage_collection();
        pmsf::rebuildVertexHalfedgeRings(mesh);
        const auto allEdges = pmsf::edgeflip::collectBoundaryEdgesSeedBfs(mesh, seed);
        auto halfKey = [](MCGAL::Halfedge* h) {
            int a = h && h->vertex() ? h->vertex()->poolId() : -1;
            int b = h && h->end_vertex() ? h->end_vertex()->poolId() : -2;
            if (a > b) std::swap(a, b);
            return std::make_pair(a, b);
        };
        struct PairGroup { std::vector<MCGAL::Halfedge*> edges; };
        std::map<std::pair<int,int>, PairGroup> byPair;
        for (MCGAL::Halfedge* h : allEdges) {
            if (!h || h->isRemoved() || !h->opposite() || !h->face() || !h->opposite()->face()) continue;
            if (h->face()->isRemoved() || h->opposite()->face()->isRemoved()) continue;
            int a = h->face()->groupId(), b = h->opposite()->face()->groupId();
            if (a == b) continue;
            byPair[{std::min(a,b), std::max(a,b)}].edges.push_back(h);
        }
        int totalPairs = 0, totalComps = 0, totalLoops = 0, totalBad = 0, totalMissing = 0, totalOutside = 0;
        for (auto& [pair, pg] : byPair) {
            ++totalPairs;
            std::map<int, std::vector<MCGAL::Halfedge*>> adj;
            for (MCGAL::Halfedge* h : pg.edges) {
                if (h->vertex()) adj[h->vertex()->poolId()].push_back(h);
                if (h->end_vertex()) adj[h->end_vertex()->poolId()].push_back(h);
            }
            std::set<MCGAL::Halfedge*> seen;
            std::set<std::pair<int,int>> allPairKeys;
            for (MCGAL::Halfedge* h : pg.edges) allPairKeys.insert(halfKey(h));
            for (MCGAL::Halfedge* seedEdge : pg.edges) {
                if (seen.count(seedEdge)) continue;
                ++totalComps;
                std::vector<MCGAL::Halfedge*> comp;
                std::deque<MCGAL::Halfedge*> q;
                q.push_back(seedEdge);
                seen.insert(seedEdge);
                while (!q.empty()) {
                    MCGAL::Halfedge* cur = q.front(); q.pop_front(); comp.push_back(cur);
                    for (int vp : {cur->vertex() ? cur->vertex()->poolId() : -1, cur->end_vertex() ? cur->end_vertex()->poolId() : -1}) {
                        auto it = adj.find(vp);
                        if (it == adj.end()) continue;
                        for (MCGAL::Halfedge* nx : it->second) {
                            if (seen.insert(nx).second) q.push_back(nx);
                        }
                    }
                }
                std::set<std::pair<int,int>> compKeys;
                for (MCGAL::Halfedge* h : comp) compKeys.insert(halfKey(h));
                MCGAL::Vertex* anchor = seedEdge->vertex();
                int ga = pair.first, gb = pair.second;
                MCGAL::Halfedge* start = pmsf::findAnchorHalfedge(mesh.get(), anchor, ga, gb);
                if (!start) start = pmsf::findAnchorHalfedge(mesh.get(), anchor, gb, ga);
                if (!start) { ++totalBad; continue; }
                auto loop = pmsf::enumerateGroupBoundaryChain(mesh.get(), start, start->face()->groupId(), start->opposite()->face()->groupId());
                ++totalLoops;
                std::set<std::pair<int,int>> loopKeys;
                for (MCGAL::Halfedge* h : loop) loopKeys.insert(halfKey(h));
                int outside = 0, missing = 0;
                for (auto& k : loopKeys) if (!compKeys.count(k)) ++outside;
                for (auto& k : compKeys) if (!loopKeys.count(k)) ++missing;
                if (outside || missing) {
                    ++totalBad; totalMissing += missing; totalOutside += outside;
                }
            }
        }
    }

    if (std::getenv("PMSF_BOUNDARY_ENUM_SEGMENTS") && mesh && seed) {
        mesh->garbage_collection();
        pmsf::rebuildVertexHalfedgeRings(mesh);
        const auto allEdges = pmsf::edgeflip::collectBoundaryEdgesSeedBfs(mesh, seed);
        auto halfKey = [](MCGAL::Halfedge* h) {
            int a = h && h->vertex() ? h->vertex()->poolId() : -1;
            int b = h && h->end_vertex() ? h->end_vertex()->poolId() : -2;
            if (a > b) std::swap(a, b);
            return std::make_pair(a, b);
        };
        std::map<std::pair<int,int>, std::vector<MCGAL::Halfedge*>> byPair;
        for (MCGAL::Halfedge* h : allEdges) {
            if (!h || h->isRemoved() || !h->opposite() || !h->face() || !h->opposite()->face()) continue;
            if (h->face()->isRemoved() || h->opposite()->face()->isRemoved()) continue;
            int a = h->face()->groupId(), b = h->opposite()->face()->groupId();
            if (a == b) continue;
            byPair[{std::min(a,b), std::max(a,b)}].push_back(h);
        }
        int totalPairs = 0, totalSegs = 0, totalBad = 0, totalMissing = 0, totalOutside = 0;
        for (auto& [pair, pg] : byPair) {
            ++totalPairs;
            std::unordered_map<int, std::vector<MCGAL::Halfedge*>> incident;
            for (MCGAL::Halfedge* h : pg) {
                if (h->vertex()) incident[h->vertex()->poolId()].push_back(h);
                if (h->end_vertex()) incident[h->end_vertex()->poolId()].push_back(h);
            }
            std::set<MCGAL::Halfedge*> visited;
            for (MCGAL::Halfedge* seedEdge : pg) {
                if (visited.count(seedEdge)) continue;
                ++totalSegs;
                std::vector<MCGAL::Halfedge*> seg;
                std::deque<MCGAL::Halfedge*> q;
                q.push_back(seedEdge); visited.insert(seedEdge);
                while (!q.empty()) {
                    MCGAL::Halfedge* cur = q.front(); q.pop_front(); seg.push_back(cur);
                    for (int vp : {cur->vertex() ? cur->vertex()->poolId() : -1, cur->end_vertex() ? cur->end_vertex()->poolId() : -1}) {
                        auto it = incident.find(vp);
                        if (it == incident.end() || it->second.size() != 2) continue;
                        for (MCGAL::Halfedge* nx : it->second) {
                            if (nx != cur && visited.insert(nx).second) q.push_back(nx);
                        }
                    }
                }
                std::set<std::pair<int,int>> segKeys;
                for (MCGAL::Halfedge* h : seg) segKeys.insert(halfKey(h));
                MCGAL::Vertex* anchor = nullptr;
                for (MCGAL::Vertex* candV : {seedEdge->vertex(), seedEdge->end_vertex()}) {
                    if (!candV) continue;
                    auto incIt = incident.find(candV->poolId());
                    if (incIt != incident.end() && incIt->second.size() == 2) { anchor = candV; break; }
                }
                if (!anchor) anchor = seedEdge->vertex();
                int ga = pair.first, gb = pair.second;
                MCGAL::Halfedge* start = nullptr;
                if (anchor) {
                    for (MCGAL::Halfedge* hh : anchor->halfedges()) {
                        if (!hh || hh->isRemoved() || !hh->face() || !hh->opposite() || !hh->opposite()->face()) continue;
                        int x = hh->face()->groupId(), y = hh->opposite()->face()->groupId();
                        if (std::min(x,y) != ga || std::max(x,y) != gb) continue;
                        if (segKeys.count(halfKey(hh))) { start = hh; break; }
                    }
                }
                if (!start) start = seedEdge;
                if (!start->face() || !start->opposite()) { ++totalBad; continue; }
                auto chain = pmsf::enumerateGroupBoundaryChain(mesh.get(), start, start->face()->groupId(), start->opposite()->face()->groupId());
                std::set<std::pair<int,int>> chainKeys;
                for (MCGAL::Halfedge* h : chain) chainKeys.insert(halfKey(h));
                int outside = 0, missing = 0;
                for (auto& k : chainKeys) if (!segKeys.count(k)) ++outside;
                for (auto& k : segKeys) if (!chainKeys.count(k)) ++missing;
                if (outside || missing) {
                    ++totalBad; totalMissing += missing; totalOutside += outside;
                }
            }
        }
    }

    if (std::getenv("PMSF_BOUNDARY_ENUM_WALKSEG") && mesh && seed) {
        mesh->garbage_collection();
        pmsf::rebuildVertexHalfedgeRings(mesh);
        const auto allEdges = pmsf::edgeflip::collectBoundaryEdgesSeedBfs(mesh, seed);
        auto halfKey = [](MCGAL::Halfedge* h) {
            int a = h && h->vertex() ? h->vertex()->poolId() : -1;
            int b = h && h->end_vertex() ? h->end_vertex()->poolId() : -2;
            if (a > b) std::swap(a, b);
            return std::make_pair(a, b);
        };
        std::map<std::pair<int,int>, std::vector<MCGAL::Halfedge*>> byPair;
        for (MCGAL::Halfedge* h : allEdges) {
            if (!h || h->isRemoved() || !h->opposite() || !h->face() || !h->opposite()->face()) continue;
            if (h->face()->isRemoved() || h->opposite()->face()->isRemoved()) continue;
            int a = h->face()->groupId(), b = h->opposite()->face()->groupId();
            if (a == b) continue;
            byPair[{std::min(a,b), std::max(a,b)}].push_back(h);
        }
        int totalSegs = 0, totalBad = 0, totalOverlap = 0, totalMissing = 0, totalOutside = 0;
        for (auto& [pair, pg] : byPair) {
            std::unordered_map<int, std::vector<MCGAL::Halfedge*>> incident;
            for (MCGAL::Halfedge* h : pg) {
                if (h->vertex()) incident[h->vertex()->poolId()].push_back(h);
                if (h->end_vertex()) incident[h->end_vertex()->poolId()].push_back(h);
            }
            std::set<MCGAL::Halfedge*> visited;
            for (MCGAL::Halfedge* seedEdge : pg) {
                if (visited.count(seedEdge)) continue;
                ++totalSegs;
                int ga = seedEdge->face()->groupId();
                int gb = seedEdge->opposite()->face()->groupId();
                auto chain = pmsf::enumerateGroupBoundaryChain(mesh.get(), seedEdge, ga, gb);
                std::set<std::pair<int,int>> chainKeys;
                for (MCGAL::Halfedge* h : chain) chainKeys.insert(halfKey(h));
                for (MCGAL::Halfedge* h : chain) if (visited.count(h)) ++totalOverlap;
                for (MCGAL::Halfedge* h : chain) visited.insert(h);
                MCGAL::Vertex* anchor = nullptr;
                for (MCGAL::Halfedge* h : chain) {
                    for (MCGAL::Vertex* candV : {h->vertex(), h->end_vertex()}) {
                        if (!candV) continue;
                        auto it = incident.find(candV->poolId());
                        if (it != incident.end() && it->second.size() == 2) { anchor = candV; break; }
                    }
                    if (anchor) break;
                }
                if (!anchor) anchor = chain.empty() ? seedEdge->vertex() : chain.front()->vertex();
                MCGAL::Halfedge* start = nullptr;
                if (anchor) {
                    for (MCGAL::Halfedge* hh : anchor->halfedges()) {
                        if (!hh || hh->isRemoved() || !hh->face() || !hh->opposite() || !hh->opposite()->face()) continue;
                        int x = hh->face()->groupId(), y = hh->opposite()->face()->groupId();
                        if (std::min(x,y) != pair.first || std::max(x,y) != pair.second) continue;
                        if (chainKeys.count(halfKey(hh))) { start = hh; break; }
                    }
                }
                if (!start) { ++totalBad; continue; }
                auto chain2 = pmsf::enumerateGroupBoundaryChain(mesh.get(), start, start->face()->groupId(), start->opposite()->face()->groupId());
                std::set<std::pair<int,int>> chain2Keys;
                for (MCGAL::Halfedge* h : chain2) chain2Keys.insert(halfKey(h));
                int outside = 0, missing = 0;
                for (auto& k : chain2Keys) if (!chainKeys.count(k)) ++outside;
                for (auto& k : chainKeys) if (!chain2Keys.count(k)) ++missing;
                if (outside || missing) {
                    ++totalBad; totalMissing += missing; totalOutside += outside;
                }
            }
        }
    }

    std::vector<int> sharedPatchAllRecords;
    std::vector<int> sharedBoundaryAnchorRecords;
    std::set<int> sharedAnchoredGroups;
    bool sharedAnchorsComputed = false;
    int sharedPatchAllCount = 0;
    auto applySharedBoundaryPatch = [&](int roundTag) -> bool {
        if (!(std::getenv("PMSF_SHARED_BOUNDARY_PATCH") && mesh && eliminate && seed)) return false;
        if (!std::getenv("PMSF_SHARED_BOUNDARY_PATCH_ALLOW_LARGE")) {
            const int patchSegments = options.getSegmentationOption().getNumSegments();
            const int patchRounds = options.getRound();
            if (patchSegments > 4 || patchRounds > 7) return false;
            if (patchSegments == 2 && patchRounds > 5) return false;
            if (patchSegments <= 1) return false;
        }
        // Anchor graph is recorded lazily per touched group (see op success
        // path).  A full per-group scan is only used when explicitly requested.
        if (!sharedAnchorsComputed && std::getenv("PMSF_SHARED_BOUNDARY_ANCHOR_ALL")) {
            mesh->garbage_collection();
            const auto anchors = pmsf::collectGroupBoundaryAnchors(mesh.get());
            for (const auto& [gid, anchor] : anchors) {
                if (sharedAnchoredGroups.count(gid)) continue;
                sharedBoundaryAnchorRecords.push_back(gid);
                if (anchor) {
                    const auto q = mesh->floatPosToInt(anchor->point());
                    sharedBoundaryAnchorRecords.push_back(q.x());
                    sharedBoundaryAnchorRecords.push_back(q.y());
                    sharedBoundaryAnchorRecords.push_back(q.z());
                } else {
                    sharedBoundaryAnchorRecords.push_back(-1);
                    sharedBoundaryAnchorRecords.push_back(-1);
                    sharedBoundaryAnchorRecords.push_back(-1);
                }
                sharedAnchoredGroups.insert(gid);
            }
        }
        sharedAnchorsComputed = true;
        mesh->garbage_collection();
        const int spMaxOps = std::atoi(std::getenv("PMSF_SHARED_BOUNDARY_PATCH_MAX") ? std::getenv("PMSF_SHARED_BOUNDARY_PATCH_MAX") : "1");
        std::vector<int> spRecordList;
        int spAppliedCount = 0;
        for (int spOp = 0; spOp < spMaxOps; ++spOp) {
        const auto spCands = pmsf::edgeflip::collectBoundaryEdgesSeedBfs(mesh, seed);
        int spTried = 0;
        const int spTryLimit = std::getenv("PMSF_SHARED_BOUNDARY_PATCH_TRIES")
                                   ? std::max(1, std::atoi(std::getenv("PMSF_SHARED_BOUNDARY_PATCH_TRIES")))
                                   : 8;
        bool spOpDone = false;
        const int spPick = (std::getenv("PMSF_SHARED_BOUNDARY_PATCH_MULTI") || std::getenv("PMSF_SHARED_BOUNDARY_PATCH_PER_ROUND")) ? -1 : (std::getenv("PMSF_SHARED_BOUNDARY_PATCH_PICK") ? std::atoi(std::getenv("PMSF_SHARED_BOUNDARY_PATCH_PICK")) : 4);
        int spCandIdx = 0;
        for (MCGAL::Halfedge* sh : spCands) {
            const int spMyIdx = spCandIdx++;
            if (spPick >= 0 && spMyIdx != spPick) continue;
            if (spOpDone || spTried >= spTryLimit) break;
            if (!sh || sh->isRemoved() || !sh->opposite()) continue;
            if (!sh->face() || !sh->opposite()->face()) continue;
            if (sh->face()->isRemoved() || sh->opposite()->face()->isRemoved()) continue;
            if (sh->face()->groupId() == sh->opposite()->face()->groupId()) continue;
            MCGAL::Vertex* sv = sh->end_vertex();
            if (!sv || sv->isRemoved()) continue;
            if (seed->vertex() && seed->end_vertex() && (sv == seed->vertex() || sv == seed->end_vertex())) continue;
            const int sd = sv->vertex_degree();
            if (sd < 4 || sd > 10) continue;
            std::vector<MCGAL::Vertex*> ringV;
            std::vector<int> ringG;
            bool ringOk = true;
            for (MCGAL::Halfedge* rh : pmsf::boundedRing(sv)) {
                if (!rh || rh->isRemoved() || !rh->face() || rh->face()->isRemoved() ||
                    rh->vertex() != sv || !rh->end_vertex() || rh->end_vertex()->isRemoved()) { ringOk = false; break; }
                ringV.push_back(rh->end_vertex());
                ringG.push_back(rh->face()->groupId());
            }
            const int d = static_cast<int>(ringV.size());
            if (!ringOk || d < 4 || d > 12) continue;
            {
                auto& frozen = pmsf::sharedBoundaryFrozenVertices();
                bool ringFrozen = frozen.count(sv) > 0;
                for (MCGAL::Vertex* rv : ringV) if (rv && frozen.count(rv)) ringFrozen = true;
                if (ringFrozen) continue;
            }
            std::set<MCGAL::Vertex*> ringSet(ringV.begin(), ringV.end());
            if (static_cast<int>(ringSet.size()) != d) continue;
            std::set<int> ringGroups; for (int g : ringG) ringGroups.insert(g);
            if (ringGroups.size() < 2) continue;
            if (std::getenv("PMSF_BOUNDARY_DEBUG")) {
                std::cerr << "[shared-boundary-patch-cand] idx=" << spMyIdx << " d=" << d << " deg=" << sd << " shG=" << (sh->face()?sh->face()->groupId():-1) << " oppG=" << (sh->opposite()&&sh->opposite()->face()?sh->opposite()->face()->groupId():-1) << " seq=";
                for (int g : ringG) std::cerr << g << ",";
                std::cerr << " trans=";
                for (int i = 0; i < d; ++i) if (ringG[(i + d - 1) % d] != ringG[i]) std::cerr << i << ",";
                std::cerr << std::endl;
            }
            std::vector<std::pair<MCGAL::Facet*, int>> multiOverrideFaces;
            if (std::getenv("PMSF_SHARED_BOUNDARY_PATCH_MULTI_HOMOG") && ringGroups.size() > 2) {
                std::set<MCGAL::Facet*> neigh;
                for (MCGAL::Facet* f : mesh->faces()) {
                    if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                    bool touchesV = false, touchesRing = false;
                    MCGAL::Halfedge* fh0 = f->proxyHalfedge();
                    MCGAL::Halfedge* fh = fh0; int fg = 0;
                    do {
                        if (fh && fh->vertex()) {
                            if (fh->vertex() == sv) touchesV = true;
                            if (ringSet.count(fh->vertex())) touchesRing = true;
                        }
                        fh = fh->next();
                    } while (fh && fh != fh0 && ++fg < 64);
                    if (touchesRing && !touchesV) neigh.insert(f);
                }
                for (MCGAL::Facet* f : neigh) {
                    multiOverrideFaces.push_back({f, f->groupId()});
                    f->setGroupId(ringG[0]);
                }
                for (MCGAL::Vertex* rv : ringV) if (rv) rv->setGroupId(ringG[0]);
            }
            const int opAnchorGidA = (sh && sh->face()) ? sh->face()->groupId() : -1;
            const int opAnchorGidB = (sh && sh->opposite() && sh->opposite()->face())
                                         ? sh->opposite()->face()->groupId()
                                         : -1;
            std::set<MCGAL::Facet*> beforeFaces;
            for (MCGAL::Facet* f : mesh->faces()) if (f && !f->isRemoved()) beforeFaces.insert(f);
            const MCGAL::PointInt vInt = mesh->floatPosToInt(sv->point());
            std::vector<MCGAL::PointInt> ringQ;
            ringQ.reserve(ringV.size());
            for (MCGAL::Vertex* rv : ringV) ringQ.push_back(mesh->floatPosToInt(rv->point()));
            if (std::getenv("PMSF_SHARED_BOUNDARY_PATCH_VALIDATE")) {
                auto faceSignature = [](const std::shared_ptr<MCGAL::Mesh>& m) {
                    std::vector<std::array<int, 9>> sig;
                    for (MCGAL::Facet* f : m->faces()) {
                        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                        std::array<std::array<int, 3>, 3> vs{};
                        int n = 0;
                        MCGAL::Halfedge* h0 = f->proxyHalfedge(); MCGAL::Halfedge* hh = h0; int guard = 0;
                        do {
                            if (hh && hh->vertex() && n < 3) {
                                auto q = m->floatPosToInt(hh->vertex()->point());
                                vs[static_cast<size_t>(n++)] = {q.x(), q.y(), q.z()};
                            }
                            hh = hh->next();
                        } while (hh && hh != h0 && ++guard < 64);
                        if (n != 3) continue;
                        std::sort(vs.begin(), vs.end());
                        std::array<int, 9> row{};
                        for (int k = 0; k < 3; ++k) for (int c = 0; c < 3; ++c) row[static_cast<size_t>(3 * k + c)] = vs[static_cast<size_t>(k)][static_cast<size_t>(c)];
                        sig.push_back(row);
                    }
                    std::sort(sig.begin(), sig.end());
                    return sig;
                };
                const auto sigBefore = faceSignature(mesh);
                std::unordered_map<int, MCGAL::Vertex*> trialMap;
                auto trial = cloneMeshExactForTrial(mesh, &trialMap);
                if (!trial) { ++spTried; continue; }
                auto trialIt = trialMap.find(sv->poolId());
                if (trialIt == trialMap.end()) { ++spTried; continue; }
                int trialD = 0;
                MCGAL::Point trialRemoved;
                std::vector<MCGAL::Halfedge*> trialRingH;
                std::vector<MCGAL::Vertex*> trialRingV;
                std::vector<int> trialRingG;
                if (!pmsf::removeSharedGroupBoundaryVertexManual(trial, trialIt->second, trialD, trialRemoved,
                                                                 trialRingH, trialRingV, trialRingG)) {
                    ++spTried;
                    continue;
                }
                {
                    bool trialValid = true;
                    std::set<std::array<int, 9>> faceKeys;
                    for (MCGAL::Facet* tf : trial->faces()) {
                        if (!tf || tf->isRemoved() || !tf->proxyHalfedge()) continue;
                        std::array<std::array<int, 3>, 3> vs{};
                        int n = 0;
                        MCGAL::Halfedge* th0 = tf->proxyHalfedge(); MCGAL::Halfedge* th = th0; int tg = 0;
                        do {
                            if (th && th->vertex() && n < 3) {
                                auto q = trial->floatPosToInt(th->vertex()->point());
                                vs[static_cast<size_t>(n++)] = {q.x(), q.y(), q.z()};
                            }
                            th = th->next();
                        } while (th && th != th0 && ++tg < 64);
                        if (n != 3) { trialValid = false; break; }
                        std::sort(vs.begin(), vs.end());
                        std::array<int, 9> key{};
                        for (int k = 0; k < 3; ++k) for (int c = 0; c < 3; ++c) key[static_cast<size_t>(3 * k + c)] = vs[static_cast<size_t>(k)][static_cast<size_t>(c)];
                        if (!faceKeys.insert(key).second) { trialValid = false; break; }
                        MCGAL::Halfedge* thit = th0; int tg2 = 0;
                        do {
                            if (thit && !thit->isRemoved() && thit->opposite()) {
                                auto* op = thit->opposite();
                                if (op->opposite() != thit || op->vertex() != thit->end_vertex() || thit->vertex() != op->end_vertex()) {
                                    trialValid = false; break;
                                }
                            }
                            thit = thit->next();
                        } while (thit && thit != th0 && ++tg2 < 64);
                        if (!trialValid) break;
                    }
                    if (trialValid) {
                        for (MCGAL::Vertex* tv : trial->vertices()) {
                            if (!tv || tv->isRemoved()) continue;
                            if (!pmsf::vertexRingCloses(tv)) { trialValid = false; break; }
                        }
                    }
                    if (!trialValid) { ++spTried; continue; }
                }
                std::vector<MCGAL::PointInt> trialRingQ;
                for (MCGAL::Vertex* tv : trialRingV) trialRingQ.push_back(trial->floatPosToInt(tv->point()));
                const MCGAL::PointInt trialVPos = trial->floatPosToInt(trialRemoved);
                if (!pmsf::restoreSharedGroupBoundaryVertexManual(trial, trialD, trialVPos, trialRingQ, trialRingG)) {
                    ++spTried;
                    continue;
                }
                if (faceSignature(trial) != sigBefore) { ++spTried; continue; }
            }
            if (std::getenv("PMSF_SHARED_BOUNDARY_PATCH_MULTI")) {
                int trans = 0;
                for (int i = 0; i < d; ++i) if (ringG[static_cast<size_t>((i + d - 1) % d)] != ringG[static_cast<size_t>(i)]) ++trans;
                if (trans < 3) continue;
                if (std::getenv("PMSF_BOUNDARY_DEBUG"))
                    std::cerr << "[shared-boundary-multi] candidate idx=" << spMyIdx << " d=" << d << " runs=" << trans << std::endl;
            }
            if (std::getenv("PMSF_SHARED_BOUNDARY_STD_ELIM")) {
                HalfedgeErrorEntry entry(sh);
                if (!eliminate->eliminate(entry)) { ++spTried; continue; }
            } else {
                int manualD = 0;
                MCGAL::Point removedPos;
                std::vector<MCGAL::Halfedge*> manualRingH;
                std::vector<MCGAL::Vertex*> manualRingV;
                std::vector<int> manualRingG;
                if (!pmsf::removeSharedGroupBoundaryVertexManual(mesh, sv, manualD, removedPos,
                                                                 manualRingH, manualRingV, manualRingG)) {
                    ++spTried;
                    continue;
                }
                ringV = manualRingV;
                ringG = manualRingG;
            }
            std::vector<MCGAL::Facet*> fill;
            for (MCGAL::Facet* f : mesh->faces()) {
                if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                MCGAL::Vertex* fv[3]; int n = 0; bool ok = true;
                MCGAL::Halfedge* h0 = f->proxyHalfedge(); MCGAL::Halfedge* hh = h0; int guard = 0;
                do {
                    if (!hh || !hh->vertex()) { ok = false; break; }
                    if (n < 3) fv[n++] = hh->vertex();
                    hh = hh->next();
                } while (hh && hh != h0 && ++guard < 64);
                if (!ok || n != 3) continue;
                if (!ringSet.count(fv[0]) || !ringSet.count(fv[1]) || !ringSet.count(fv[2])) continue;
                fill.push_back(f);
            }
            if (fill.empty()) { ++spTried; continue; }
            g_sharedPatchFillFaces.clear();
            for (MCGAL::Facet* f : fill) {
                MCGAL::Vertex* fv[3]; int n = 0;
                MCGAL::Halfedge* h0 = f->proxyHalfedge(); MCGAL::Halfedge* hh = h0; int guard = 0;
                do { if (hh && hh->vertex() && n < 3) fv[n++] = hh->vertex(); hh = hh->next(); } while (hh && hh != h0 && ++guard < 64);
                if (n != 3) continue;
                std::array<int, 9> tri{};
                for (int k = 0; k < 3; ++k) {
                    auto it = std::find(ringV.begin(), ringV.end(), fv[k]);
                    if (it != ringV.end()) {
                        const auto& q = ringQ[static_cast<size_t>(it - ringV.begin())];
                        tri[static_cast<size_t>(3 * k)] = q.x();
                        tri[static_cast<size_t>(3 * k + 1)] = q.y();
                        tri[static_cast<size_t>(3 * k + 2)] = q.z();
                    }
                }
                g_sharedPatchFillFaces.push_back(tri);
            }
            // This patch is a pre-pass, not a progressive symbol: clear the splittable-face
            // payload left by eliminate so the symbol collector does not replay it.
            for (MCGAL::Facet* pf : mesh->faces()) {
                if (!pf || pf->isRemoved()) continue;
                pf->resetSplittableFlag();
                pf->setUnProcessed();
                pf->setRemovedVertexPos(MCGAL::Point(0, 0, 0));
                pf->setRemovedVertexPosInt(MCGAL::PointInt(0, 0, 0));
                MCGAL::Halfedge* ph0 = pf->proxyHalfedge();
                MCGAL::Halfedge* ph = ph0;
                int pguard = 0;
                do {
                    if (ph && !ph->isRemoved()) ph->setOriginal();
                    ph = ph->next();
                } while (ph && ph != ph0 && ++pguard < 64);
            }
            for (MCGAL::Vertex* rv : ringV) if (rv) rv->resetState();
            if (std::getenv("PMSF_SHARED_BOUNDARY_PATCH_HOMOGENIZE")) {
                std::set<int> patchGroups(ringG.begin(), ringG.end());
                if (patchGroups.size() > 2) {
                    for (MCGAL::Vertex* rv : ringV) if (rv) rv->setGroupId(ringG[0]);
                }
            }
            int opInterface = -1, opBoundaryIndex = -1;
            {
                auto edgeKey = [](MCGAL::Halfedge* h) {
                    int a = h && h->vertex() ? h->vertex()->poolId() : -1;
                    int b = h && h->end_vertex() ? h->end_vertex()->poolId() : -2;
                    if (a > b) std::swap(a, b);
                    return std::make_pair(a, b);
                };
                const auto segs = pmsf::collectGroupBoundarySegments(mesh.get());
                const auto targetKey = edgeKey(sh);
                for (size_t si = 0; si < segs.size(); ++si) {
                    for (size_t ei = 0; ei < segs[si].edges.size(); ++ei) {
                        if (edgeKey(segs[si].edges[ei]) == targetKey) {
                            opInterface = static_cast<int>(si);
                            opBoundaryIndex = static_cast<int>(ei);
                            break;
                        }
                    }
                    if (opInterface >= 0) break;
                }
            }
            std::vector<int> rec;
            rec.push_back(roundTag);
            rec.push_back(opInterface);
            rec.push_back(opBoundaryIndex);
            rec.push_back(d);
            rec.push_back(vInt.x()); rec.push_back(vInt.y()); rec.push_back(vInt.z());
            for (int i = 0; i < d; ++i) {
                const MCGAL::PointInt q = ringQ[static_cast<size_t>(i)];
                rec.push_back(q.x() - vInt.x());
                rec.push_back(q.y() - vInt.y());
                rec.push_back(q.z() - vInt.z());
                rec.push_back(ringG[static_cast<size_t>(i)]);
            }
            rec.push_back(static_cast<int>(multiOverrideFaces.size()));
            for (auto& ov : multiOverrideFaces) {
                std::array<std::array<int, 3>, 3> vs{};
                MCGAL::Facet* f = ov.first;
                MCGAL::Halfedge* fh0 = f->proxyHalfedge(); MCGAL::Halfedge* fh = fh0; int fg = 0;
                int k = 0;
                do {
                    if (fh && fh->vertex() && k < 3) {
                        auto q = mesh->floatPosToInt(fh->vertex()->point());
                        vs[static_cast<size_t>(k++)] = {q.x(), q.y(), q.z()};
                    }
                    fh = fh->next();
                } while (fh && fh != fh0 && ++fg < 64);
                std::sort(vs.begin(), vs.end());
                for (int i = 0; i < 3; ++i) for (int c = 0; c < 3; ++c) rec.push_back(vs[static_cast<size_t>(i)][static_cast<size_t>(c)]);
                rec.push_back(ov.second);
            }
            spRecordList.insert(spRecordList.end(), rec.begin(), rec.end());
            for (MCGAL::Vertex* rv : ringV) pmsf::sharedBoundaryFrozenVertices().insert(rv);
            if (!std::getenv("PMSF_SHARED_BOUNDARY_PATCH_NO_ANCHOR")) {
                const int gidA = opAnchorGidA;
                const int gidB = opAnchorGidB;
                for (int gid : {gidA, gidB}) {
                    if (gid < 0 || !sharedAnchoredGroups.insert(gid).second) continue;
                    MCGAL::Vertex* anchor = sv;
                    for (size_t ri = 0; ri < ringG.size(); ++ri) {
                        if (ringG[ri] == gid && ringV[ri]) { anchor = ringV[ri]; break; }
                    }
                    sharedBoundaryAnchorRecords.push_back(gid);
                    if (anchor) {
                        const auto q = mesh->floatPosToInt(anchor->point());
                        sharedBoundaryAnchorRecords.push_back(q.x());
                        sharedBoundaryAnchorRecords.push_back(q.y());
                        sharedBoundaryAnchorRecords.push_back(q.z());
                    } else {
                        sharedBoundaryAnchorRecords.push_back(-1);
                        sharedBoundaryAnchorRecords.push_back(-1);
                        sharedBoundaryAnchorRecords.push_back(-1);
                    }
                }
            }
            const char* extLock = std::getenv("PMSF_SHARED_BOUNDARY_PATCH_EXTEND_LOCK");
            int lockDepth = 0;
            if (extLock) {
                lockDepth = std::max(1, std::atoi(extLock));
            } else if (std::getenv("PMSF_SHARED_BOUNDARY_PATCH_PER_ROUND")) {
                lockDepth = options.getSegmentationOption().getNumSegments() <= 2 ? 2 : 1;
            }
            if (lockDepth > 0) {
                std::set<MCGAL::Vertex*> firstRing;
                for (MCGAL::Vertex* rv : ringV) {
                    if (!rv) continue;
                    for (MCGAL::Halfedge* hh : pmsf::boundedRing(rv)) {
                        if (hh && hh->end_vertex()) firstRing.insert(hh->end_vertex());
                    }
                }
                for (MCGAL::Vertex* fv : firstRing) pmsf::sharedBoundaryFrozenVertices().insert(fv);
                if (lockDepth >= 2) {
                    for (MCGAL::Vertex* fv : firstRing) {
                        if (!fv) continue;
                        for (MCGAL::Halfedge* hh : pmsf::boundedRing(fv)) {
                            if (hh && hh->end_vertex()) pmsf::sharedBoundaryFrozenVertices().insert(hh->end_vertex());
                        }
                    }
                }
            }
            ++spAppliedCount;
            spOpDone = true;
            if (std::getenv("PMSF_BOUNDARY_DEBUG"))
                std::cerr << "[shared-boundary-patch] op=" << spOp << " d=" << d << " fill=" << fill.size() << " rec=" << rec.size() << std::endl;
            break;
        }
        if (!spOpDone) break;
        }
        if (spAppliedCount > 0) {
            sharedPatchAllCount += spAppliedCount;
            sharedPatchAllRecords.insert(sharedPatchAllRecords.end(), spRecordList.begin(), spRecordList.end());
            if (collector) collector->init(mesh, seeds, header);
            if (select) select->init(mesh);
            if (eliminate) eliminate->init(mesh, header);
            if (std::getenv("PMSF_BOUNDARY_DEBUG")) {
                mesh->dumpto_oldtype("/tmp/shared_patch_input.off");
                std::cerr << "[shared-boundary-patch] appliedThisCall=" << spAppliedCount
                          << " totalOps=" << sharedPatchAllCount << std::endl;
            }
        } else if (std::getenv("PMSF_BOUNDARY_DEBUG")) {
            std::cerr << "[shared-boundary-patch] skipped" << std::endl;
        }
        return spAppliedCount > 0;
    };
    const int patchAfterRound = std::getenv("PMSF_SHARED_BOUNDARY_PATCH_AFTER_ROUND")
                                    ? std::atoi(std::getenv("PMSF_SHARED_BOUNDARY_PATCH_AFTER_ROUND"))
                                    : -1;
    const bool patchPerRound = std::getenv("PMSF_SHARED_BOUNDARY_PATCH_PER_ROUND") != nullptr;
    std::set<int> patchRoundSet;
    if (const char* roundsEnv = std::getenv("PMSF_SHARED_BOUNDARY_PATCH_ROUNDS")) {
        std::string rs(roundsEnv);
        size_t pos = 0;
        while (pos <= rs.size()) {
            size_t comma = rs.find(',', pos);
            std::string tok = rs.substr(pos, comma == std::string::npos ? std::string::npos : comma - pos);
            if (!tok.empty()) patchRoundSet.insert(std::atoi(tok.c_str()));
            if (comma == std::string::npos) break;
            pos = comma + 1;
        }
    }
    if ((!patchPerRound || std::getenv("PMSF_SHARED_BOUNDARY_PATCH_PREPASS_TOO")) && patchAfterRound < 0)
        applySharedBoundaryPatch(-1);
    if (boundaryModuleEnabled && sharedPatchAllCount > 0) {
        // The pre-pass can delete a vertex that happens to be a segmentation
        // seed.  Replacing such seeds with an alive proxy of the same group is
        // required: an invalid seed makes the segmented BFS/group channel lose
        // whole patches in the decoder even though the patched mesh is valid.
        int repairedSeeds = 0;
        for (std::size_t gi = 0; gi < seeds.size(); ++gi) {
            MCGAL::Halfedge* s = seeds[gi];
            if (s && !s->isRemoved() && s->vertex() && !s->vertex()->isRemoved() &&
                s->end_vertex() && !s->end_vertex()->isRemoved())
                continue;
            MCGAL::Halfedge* replacement = nullptr;
            for (MCGAL::Facet* f : mesh->faces()) {
                if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                if (f->groupId() != static_cast<int>(gi)) continue;
                replacement = f->proxyHalfedge();
                break;
            }
            if (replacement) {
                seeds[gi] = replacement;
                ++repairedSeeds;
            }
        }
        if (seed && (seed->isRemoved() || !seed->vertex() || seed->vertex()->isRemoved()) &&
            !seeds.empty()) {
            seed = seeds[0];
        }
        if (std::getenv("PMSF_BOUNDARY_DEBUG"))
            std::fprintf(stderr, "[boundary-module] repaired_seeds=%d\n", repairedSeeds);
    }
    // Optional module: the pre-pass may delete vertices exactly on group
    // boundaries.  The old group ids are inherited by the simplified patched
    // mesh, which is not a valid segmentation of the new mesh and makes the
    // segmented BFS/group channel disagree with the decoder.  Re-segment the
    // patched mesh before collecting symbols; the standard path (module off)
    // is untouched.
    if (std::getenv("PMSF_BOUNDARY_MODULE_RESEGMENT") && boundaryModuleEnabled &&
        sharedPatchAllCount > 0 && cfg.segmentOperatorFactory) {
        if (auto resegment = cfg.segmentOperatorFactory()) {
            resegment->segment(mesh);
            auto resegSeeds = resegment->exportSeeds();
            if (!resegSeeds.empty()) {
                seeds = std::move(resegSeeds);
                if (seeds[0]) seed = seeds[0];
            }
            boundaryGraph = resegment->exportGraph();
        }
    }

    auto applySharedBoundary = [&]() -> int {
        int sbRemoved = 0;
        int sbCandCount = 0;
        const int sbMax = std::atoi(std::getenv("PMSF_SHARED_BOUNDARY_MAX") ? std::getenv("PMSF_SHARED_BOUNDARY_MAX") : "4");
        for (int sbIt = 0; sbIt < sbMax; ++sbIt) {
            const auto sbCands = pmsf::edgeflip::collectBoundaryEdgesSeedBfs(mesh, seed);
            sbCandCount = static_cast<int>(sbCands.size());
            bool sbDid = false;
            std::vector<int> sbPicks;
            if (const char* pstr = std::getenv("PMSF_SHARED_BOUNDARY_PICK")) {
                std::string sp(pstr);
                size_t pos = 0;
                while (pos <= sp.size()) {
                    size_t comma = sp.find(',', pos);
                    std::string tok = sp.substr(pos, comma == std::string::npos ? std::string::npos : comma - pos);
                    if (!tok.empty()) sbPicks.push_back(std::atoi(tok.c_str()));
                    if (comma == std::string::npos) break;
                    pos = comma + 1;
                }
            }
            const int sbPick = sbPicks.empty() ? -1 : sbPicks[static_cast<size_t>(std::min<int>(sbIt, static_cast<int>(sbPicks.size()) - 1))];
            int sbIdx = 0;
            for (MCGAL::Halfedge* sh : sbCands) {
                const int sbMyIdx = sbIdx++;
                if (sbPick >= 0 && sbMyIdx != sbPick) continue;
                if (!sh || sh->isRemoved() || !sh->opposite()) continue;
                if (!sh->face() || !sh->opposite()->face()) continue;
                if (sh->face()->isRemoved() || sh->opposite()->face()->isRemoved()) continue;
                if (sh->face()->groupId() == sh->opposite()->face()->groupId()) continue;
                MCGAL::Vertex* sv = sh->end_vertex();
                if (!sv || sv->isRemoved()) continue;
                if (seed->vertex() && seed->end_vertex() && (sv == seed->vertex() || sv == seed->end_vertex())) continue;
                const int sd = sv->vertex_degree();
                if (sd < 3 || sd > 7) continue;
                if (std::getenv("PMSF_BOUNDARY_DEBUG")) {
                    std::set<int> gs;
                    for (MCGAL::Halfedge* rh : sv->halfedges()) if (rh && !rh->isRemoved() && rh->face() && !rh->face()->isRemoved()) gs.insert(rh->face()->groupId());
                    auto q = mesh->floatPosToInt(sv->point());
                    std::cerr << "[shared-boundary-cand] idx=" << sbMyIdx << " v=" << q.x() << "," << q.y() << "," << q.z() << " deg=" << sd << " seq=";
                    for (MCGAL::Halfedge* rh : pmsf::boundedRing(sv)) {
                        if (!rh || rh->isRemoved()) continue;
                        std::cerr << (rh->face() && !rh->face()->isRemoved() ? rh->face()->groupId() : -1) << ",";
                    }
                    std::cerr << " groups=";
                    for (int g : gs) std::cerr << g << ",";
                    std::cerr << " vGroup=" << sv->groupId()
                              << " shFaceG=" << (sh->face() ? sh->face()->groupId() : -99)
                              << " shOppG=" << (sh->opposite() && sh->opposite()->face() ? sh->opposite()->face()->groupId() : -99)
                              << std::endl;
                }
                if (std::getenv("PMSF_SHARED_BOUNDARY_ORIENT")) {
                    for (MCGAL::Halfedge* alt : sv->halfedges()) {
                        if (!alt || alt == sh || alt->isRemoved() || !alt->opposite()) continue;
                        if (alt->end_vertex() != sv) continue;
                        if (!alt->face() || !alt->opposite()->face()) continue;
                        if (alt->face()->isRemoved() || alt->opposite()->face()->isRemoved()) continue;
                        if (alt->face()->groupId() == alt->opposite()->face()->groupId()) continue;
                        sh = alt; break;
                    }
                }
                HalfedgeErrorEntry sEntry(sh);
                if (eliminate->eliminate(sEntry)) { ++sbRemoved; sbDid = true; break; }
            }
            if (!sbDid) break;
        }
        if (std::getenv("PMSF_SEAM_DEBUG") || std::getenv("PMSF_BOUNDARY_DEBUG"))
            std::cerr << "[shared-boundary-test] candidates=" << sbCandCount << " removed=" << sbRemoved << std::endl;
        return sbRemoved;
    };
    const int sbAfterRound = std::getenv("PMSF_SHARED_BOUNDARY_AFTER_ROUND")
                                 ? std::atoi(std::getenv("PMSF_SHARED_BOUNDARY_AFTER_ROUND"))
                                 : -1;
    if (std::getenv("PMSF_SHARED_BOUNDARY_TEST") && sbAfterRound < 0) applySharedBoundary();

    const bool boundaryStageEnabled = std::getenv("PMSF_BOUNDARY_STAGE") != nullptr;
    int boundaryStageMax = 8;
    if (const char* bsMax = std::getenv("PMSF_BOUNDARY_STAGE_MAX"))
        boundaryStageMax = std::max(0, std::atoi(bsMax));
    bool sharedPatchAfterInterior = false;
    if (const char* order = std::getenv("PMSF_ROUND_STAGE_ORDER")) {
        const std::string o(order);
        const std::size_t pb = o.find("boundary");
        const std::size_t pi = o.find("interior");
        if (pb != std::string::npos && pi != std::string::npos)
            sharedPatchAfterInterior = pb > pi;
    }
    bool boundaryStageBeforeInterior = true;
    if (const char* order = std::getenv("PMSF_ROUND_STAGE_ORDER")) {
        std::string o(order);
        const std::size_t pb = o.find("boundary");
        const std::size_t pi = o.find("interior");
        if (pb != std::string::npos && pi != std::string::npos)
            boundaryStageBeforeInterior = pb < pi;
    }
    // Explicit 3-stage order for the boundary-v2 multi-round path:
    //   IFB = interior -> flip -> boundary   (default POST layout)
    //   BIF = boundary -> interior -> flip   (boundary PRE layout)
    //   IBF = interior -> boundary -> flip   (new: boundary moves before flip)
    // F-first permutations (FIB/BFI/FBI) are not wired yet and fall back to IFB.
    int scV2StageOrder = 0;
    if (const char* o = std::getenv("PMSF_SC_V2_STAGE_ORDER")) {
        std::string s(o);
        for (auto& c : s)
            if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');
        if (s == "IFB" || s == "INTERIOR_FLIP_BOUNDARY") {
            scV2StageOrder = 0;
        } else if (s == "BIF" || s == "BOUNDARY_INTERIOR_FLIP") {
            scV2StageOrder = 1;
        } else if (s == "IBF" || s == "INTERIOR_BOUNDARY_FLIP") {
            scV2StageOrder = 2;
        } else if (s == "BFI" || s == "BOUNDARY_FLIP_INTERIOR") {
            scV2StageOrder = 3;
        } else if (s == "FIB" || s == "FLIP_INTERIOR_BOUNDARY") {
            scV2StageOrder = 4;
        } else if (s == "FBI" || s == "FLIP_BOUNDARY_INTERIOR") {
            scV2StageOrder = 5;
        } else {
            std::fprintf(stderr,
                         "[sc-stage-order] unsupported order '%s'; supported: IFB, BIF, IBF "
                         "(falling back to IFB)\n",
                         o);
            scV2StageOrder = 0;
        }
    }
    const bool scV2OrderEnv = std::getenv("PMSF_SC_V2_STAGE_ORDER") != nullptr;
    auto scRoundListContains = [](const char* csv, int round) -> bool {
        if (!csv) return false;
        const std::string rs(csv);
        std::size_t p = 0;
        while (p <= rs.size()) {
            const std::size_t comma = rs.find(',', p);
            const std::string tok =
                rs.substr(p, comma == std::string::npos ? std::string::npos : comma - p);
            if (!tok.empty() && std::atoi(tok.c_str()) == round) return true;
            if (comma == std::string::npos) break;
            p = comma + 1;
        }
        return false;
    };
    // Base per-round order id: 0=IFB, 1=BIF, 2=IBF, 3=BFI, 4=FIB, 5=FBI.
    // PRE_ROUNDS selects BIF rounds, IFB_ROUNDS selects IFB rounds, everything
    // else uses PMSF_SC_V2_STAGE_ORDER, so one run can mix all six layouts.
    auto scBaseOrderForRound = [&](int round) -> int {
        // Explicit per-round order list has the highest priority:
        //   PMSF_SC_V2_STAGE_ORDER_LIST=0,2,1,5,...  (one id per round, -1 = default)
        if (const char* ol = std::getenv("PMSF_SC_V2_STAGE_ORDER_LIST")) {
            const std::string rs(ol);
            int idx = 0;
            std::size_t p = 0;
            while (p <= rs.size()) {
                const std::size_t comma = rs.find(',', p);
                const std::string tok =
                    rs.substr(p, comma == std::string::npos ? std::string::npos : comma - p);
                if (!tok.empty() && idx == round) {
                    const int v = std::atoi(tok.c_str());
                    if (v >= 0 && v <= 5) return v;
                    break;
                }
                if (comma == std::string::npos) break;
                p = comma + 1;
                ++idx;
            }
        }
        if (!scV2OrderEnv) return 0;
        if (scRoundListContains(std::getenv("PMSF_SC_BOUNDARY_PRE_ROUNDS"), round))
            return 1;
        if (scRoundListContains(std::getenv("PMSF_SC_V2_STAGE_ORDER_IFB_ROUNDS"), round))
            return 0;
        return scV2StageOrder;
    };
    // "boundary runs before the interior elimination" for BIF(1) and BFI(3).
    auto scPreForRound = [&](int round) -> bool {
        if (scV2OrderEnv) {
            const int ro = scBaseOrderForRound(round);
            return ro == 1 || ro == 3;
        }
        if (std::getenv("PMSF_SC_BOUNDARY_PRE")) return true;
        const char* pr = std::getenv("PMSF_SC_BOUNDARY_PRE_ROUNDS");
        if (!pr) return false;
        const std::string rs(pr);
        std::size_t p = 0;
        while (p <= rs.size()) {
            const std::size_t comma = rs.find(',', p);
            const std::string tok = rs.substr(
                p, comma == std::string::npos ? std::string::npos : comma - p);
            if (!tok.empty() && std::atoi(tok.c_str()) == round) return true;
            if (comma == std::string::npos) break;
            p = comma + 1;
        }
        return false;
    };
    auto scRoundOrderFor = [&](int round) -> int {
        if (!scV2OrderEnv) return scPreForRound(round) ? 1 : 0;
        return scBaseOrderForRound(round);
    };
    auto runBoundaryStage = [&](int round) -> int {
        if (!boundaryStageEnabled || !eliminate || !mesh) return 0;
        int removed = 0;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
            MCGAL::Halfedge* h0 = f->proxyHalfedge();
            MCGAL::Halfedge* h = h0;
            int guard = 0;
            do {
                if (h && !h->isRemoved() && h->face() && h->opposite() &&
                    h->opposite()->face() && !h->opposite()->face()->isRemoved() &&
                    h->face()->groupId() != h->opposite()->face()->groupId()) {
                    MCGAL::Vertex* v = h->end_vertex();
                    bool frozen = false;
                    if (v) {
                        auto& fz = pmsf::sharedBoundaryFrozenVertices();
                        frozen = fz.count(v) > 0;
                    }
                    if (v && !v->isRemoved() && !frozen) {
                        const int deg = v->is_degree_dirty() ? v->vertex_degree()
                                                             : v->vertex_degree_cached();
                        if (deg >= 4 && deg <= 10) {
                            HalfedgeErrorEntry be(h, MCGAL::Point(0, 0, 0), 0, 0);
                            if (eliminate->eliminate(be)) {
                                ++removed;
                                if (removed >= boundaryStageMax) return removed;
                                break;
                            }
                        }
                    }
                }
                h = h ? h->next() : nullptr;
            } while (h && h != h0 && ++guard < 64);
            if (removed >= boundaryStageMax) break;
        }
        return removed;
    };

    auto runGroupBoundarySplitStage = [&](int round) -> int {
        if (!std::getenv("PMSF_GROUP_BOUNDARY_SPLIT") || !mesh) return 0;
        int maxOps = 1;
        if (const char* m = std::getenv("PMSF_GROUP_BOUNDARY_SPLIT_MAX"))
            maxOps = std::max(0, std::min(32, std::atoi(m)));
        int triesLimit = 64;
        if (const char* t = std::getenv("PMSF_GROUP_BOUNDARY_SPLIT_TRIES"))
            triesLimit = std::max(1, std::atoi(t));
        const auto& frozen = pmsf::sharedBoundaryFrozenVertices();
        const bool lossy = std::getenv("PMSF_GROUP_BOUNDARY_SPLIT_LOSSY") != nullptr;
        int applied = 0;
        for (int op = 0; op < maxOps; ++op) {
            std::vector<MCGAL::Vertex*> cands;
            for (MCGAL::Facet* f : mesh->faces()) {
                if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                MCGAL::Halfedge* h0 = f->proxyHalfedge();
                MCGAL::Halfedge* h = h0;
                int guard = 0;
                do {
                    if (h && !h->isRemoved() && h->face() && h->opposite() &&
                        h->opposite()->face() && !h->opposite()->face()->isRemoved() &&
                        h->face()->groupId() != h->opposite()->face()->groupId()) {
                        MCGAL::Vertex* v = h->end_vertex();
                        if (v && !v->isRemoved() && frozen.count(v) == 0) {
                            const int deg = v->is_degree_dirty() ? v->vertex_degree()
                                                                 : v->vertex_degree_cached();
                            if (deg >= 4 && deg <= 12 &&
                                !(seed && seed->vertex() && seed->end_vertex() &&
                                  (v == seed->vertex() || v == seed->end_vertex()))) {
                                bool touchesSeed = false;
                                std::vector<int> fanG;
                                int fanD = 0;
                                if (seed && !seed->isRemoved()) {
                                    for (MCGAL::Halfedge* rh : pmsf::boundedRing(v)) {
                                        if (rh == seed || rh == seed->opposite()) { touchesSeed = true; break; }
                                        if (rh && rh->face() && !rh->face()->isRemoved()) {
                                            fanG.push_back(rh->face()->groupId());
                                            ++fanD;
                                        }
                                    }
                                } else {
                                    for (MCGAL::Halfedge* rh : pmsf::boundedRing(v)) {
                                        if (rh && rh->face() && !rh->face()->isRemoved()) {
                                            fanG.push_back(rh->face()->groupId());
                                            ++fanD;
                                        }
                                    }
                                }
                                std::vector<int> transitions;
                                for (int fi = 0; fi < fanD; ++fi)
                                    if (fanG[static_cast<std::size_t>(fi)] !=
                                        fanG[static_cast<std::size_t>((fi + fanD - 1) % fanD)])
                                        transitions.push_back(fi);
                                bool simplePath = false;
                                if (transitions.size() == 2 && (deg == 4 || deg == 5)) {
                                    const int a = transitions[0];
                                    const int b = transitions[1];
                                    const int d1 = (b - a + deg) % deg;
                                    const int d2 = deg - d1;
                                    simplePath = (d1 >= 2 && d2 >= 2);
                                }
                                if (!touchesSeed && fanD == deg && simplePath)
                                    cands.push_back(v);
                            }
                        }
                    }
                    h = h ? h->next() : nullptr;
                } while (h && h != h0 && ++guard < 64);
            }
            std::sort(cands.begin(), cands.end());
            cands.erase(std::unique(cands.begin(), cands.end()), cands.end());
            bool done = false;
            int tried = 0;
            for (MCGAL::Vertex* v : cands) {
                if (done || tried >= triesLimit) break;
                ++tried;
                int d = 0;
                MCGAL::Point removed;
                std::vector<MCGAL::Halfedge*> ringH;
                std::vector<MCGAL::Vertex*> ringV;
                std::vector<int> ringG;
                if (!pmsf::removeSharedGroupBoundaryVertexManual(
                        mesh, v, d, removed, ringH, ringV, ringG))
                    continue;
                auto vq = mesh->floatPosToInt(removed);
                {
                    std::set<MCGAL::Vertex*> ringSet(ringV.begin(), ringV.end());
                    for (int rep = 0; rep < 8; ++rep) {
                        std::map<std::pair<MCGAL::Vertex*, MCGAL::Vertex*>,
                                 std::vector<MCGAL::Facet*>> edgeFaces;
                        for (MCGAL::Facet* lf : mesh->faces()) {
                            if (!lf || lf->isRemoved() || !lf->proxyHalfedge()) continue;
                            MCGAL::Vertex* lv[3];
                            int ln = 0;
                            MCGAL::Halfedge* lh0 = lf->proxyHalfedge();
                            MCGAL::Halfedge* lh = lh0;
                            int lg = 0;
                            do {
                                if (lh && lh->vertex() && ln < 3) lv[ln++] = lh->vertex();
                                lh = lh ? lh->next() : nullptr;
                            } while (lh && lh != lh0 && ++lg < 64);
                            if (ln != 3) continue;
                            bool touches = false;
                            for (int k = 0; k < 3; ++k)
                                if (ringSet.count(lv[k])) { touches = true; break; }
                            if (!touches) continue;
                            for (int k = 0; k < 3; ++k) {
                                MCGAL::Vertex* a = lv[k];
                                MCGAL::Vertex* b = lv[(k + 1) % 3];
                                if (!a || !b || a == b) continue;
                                if (a > b) std::swap(a, b);
                                edgeFaces[{a, b}].push_back(lf);
                            }
                        }
                        MCGAL::Facet* victim = nullptr;
                        for (auto& kv : edgeFaces) {
                            if (kv.second.size() > 2) { victim = kv.second.back(); break; }
                        }
                        if (!victim) break;
                        pmsf::removeFaceForPatch(mesh, victim);
                        if (std::getenv("PMSF_BOUNDARY_DEBUG"))
                            std::fprintf(stderr, "[group-boundary-split] repaired surplus face round=%d\n", round);
                    }
                }
                if (!lossy) {
                    g_groupSplitResiduals.push_back(round);
                    g_groupSplitResiduals.push_back(d);
                    g_groupSplitResiduals.push_back(vq.x());
                    g_groupSplitResiduals.push_back(vq.y());
                    g_groupSplitResiduals.push_back(vq.z());
                    for (int ri = 0; ri < d; ++ri) {
                        const auto rq = mesh->floatPosToInt(ringV[static_cast<std::size_t>(ri)]->point());
                        g_groupSplitResiduals.push_back(rq.x() - vq.x());
                        g_groupSplitResiduals.push_back(rq.y() - vq.y());
                        g_groupSplitResiduals.push_back(rq.z() - vq.z());
                        g_groupSplitResiduals.push_back(ringG[static_cast<std::size_t>(ri)]);
                    }
                }
                ++applied;
                done = true;
                break;
            }
            if (!done) break;
        }
        if (applied > 0) {
            // Manual boundary surgery is a pre-pass, not a progressive symbol:
            // clear the splittable/removed-vertex payload so the round collector
            // does not replay it as an interior vertex removal.
            for (MCGAL::Facet* pf : mesh->faces()) {
                if (!pf || pf->isRemoved()) continue;
                pf->resetSplittableFlag();
                pf->setUnProcessed();
                pf->setRemovedVertexPos(MCGAL::Point(0, 0, 0));
                pf->setRemovedVertexPosInt(MCGAL::PointInt(0, 0, 0));
                MCGAL::Halfedge* ph0 = pf->proxyHalfedge();
                MCGAL::Halfedge* ph = ph0;
                int pguard = 0;
                do {
                    if (ph && !ph->isRemoved()) ph->setOriginal();
                    ph = ph->next();
                } while (ph && ph != ph0 && ++pguard < 64);
            }
            for (MCGAL::Vertex* rv : mesh->vertices())
                if (rv && !rv->isRemoved()) rv->resetState();
            if (collector) collector->init(mesh, seeds, header);
            if (select) select->init(mesh);
            if (eliminate) eliminate->init(mesh, header);
            if (std::getenv("PMSF_BOUNDARY_DEBUG"))
                std::fprintf(stderr, "[group-boundary-split] round=%d applied=%d totalResiduals=%zu\n",
                             round, applied, g_groupSplitResiduals.size());
        }
        return applied;
    };

    std::vector<std::pair<MCGAL::Vertex*, MCGAL::Vertex*>> segSeedPairs;
    if (std::getenv("PMSF_SEG_REFRESH_SEEDS") && !seeds.empty()) {
        for (MCGAL::Halfedge* h : seeds) {
            if (h && h->vertex() && h->end_vertex())
                segSeedPairs.emplace_back(h->vertex(), h->end_vertex());
            else
                segSeedPairs.emplace_back(nullptr, nullptr);
        }
    }
    if (std::getenv("PMSF_SC_BOUNDARY_STAGE") ||
        std::getenv("PMSF_SC_BOUNDARY_COMPACT_STAGE") ||
        std::getenv("PMSF_SC_BOUNDARY_ANCHOR_STAGE") ||
        std::getenv("PMSF_SC_BOUNDARY_GROUP_STAGE") ||
        std::getenv("PMSF_SC_BOUNDARY_AUTO"))
        g_scBoundaryRoundData.assign(static_cast<std::size_t>(std::max(0, options.getRound())),
                                     std::vector<int>{});
    g_scRoundOrdering.assign(static_cast<std::size_t>(std::max(0, options.getRound())), 0);
    g_scEffectiveRoundOrdering.assign(static_cast<std::size_t>(std::max(0, options.getRound())), 0);
    static thread_local std::vector<std::pair<int, MCGAL::Vertex*>> g_bcV2Anchors;
    static thread_local std::vector<pmsf::BoundaryV2Round> g_bcV2Rounds;
    // Boundary-v2 (mode 70) opt-in: explicit env, or the AUTO helper (requires the
    // validated multi-round + flip + boundary-round envelope).
    const bool bcV2Auto = std::getenv("PMSF_BC_V2_AUTO") != nullptr &&
                          std::getenv("PMSF_SC_ALLOW_MULTIROUND") != nullptr &&
                          std::getenv("PMSF_EDGEFLIP_ENABLE") != nullptr &&
                          std::getenv("PMSF_SC_BOUNDARY_ROUNDS") != nullptr;
    const bool bcV2Enabled =
        std::getenv("PMSF_SC_BOUNDARY_V2") != nullptr || bcV2Auto;
    const bool conservativeAnchors =
        std::getenv("PMSF_BND_BASE_STAGE") != nullptr && mesh;
    if ((bcV2Enabled || conservativeAnchors) && mesh) {
        g_bcV2Anchors = pmsf::collectFixedLoopAnchors(mesh.get());
        auto& frozenV2 = pmsf::sharedBoundaryFrozenVertices();
        auto& guardedV2 = pmsf::anchorGuardedVertices();
        for (auto& ag : g_bcV2Anchors) {
            if (!ag.second || ag.second->isRemoved()) continue;
            ag.second->setAnchor();
            frozenV2.insert(ag.second);
            guardedV2.insert(ag.second);
        }
        g_bcV2Rounds.assign(static_cast<std::size_t>(std::max(0, options.getRound())),
                            pmsf::BoundaryV2Round{});
        std::fprintf(stderr, "[bc-v2] anchors=%zu rounds=%d\n", g_bcV2Anchors.size(),
                     options.getRound());
    }
    auto bcV2RoundSelected = [&](int r) -> bool {
        const char* only = std::getenv("PMSF_SC_BOUNDARY_ROUNDS");
        if (!only || !only[0]) return true;
        const std::string rs(only);
        std::size_t p = 0;
        while (p <= rs.size()) {
            const std::size_t comma = rs.find(',', p);
            const std::string tok =
                rs.substr(p, comma == std::string::npos ? std::string::npos : comma - p);
            if (!tok.empty() && std::atoi(tok.c_str()) == r) return true;
            if (comma == std::string::npos) break;
            p = comma + 1;
        }
        return false;
    };
    g_scRoundSeedQ.assign(static_cast<std::size_t>(std::max(0, options.getRound())),
                          std::array<int, 6>{-1, -1, -1, -1, -1, -1});
    g_scRoundSeedVerts.assign(static_cast<std::size_t>(std::max(0, options.getRound())),
                              std::array<MCGAL::Vertex*, 2>{nullptr, nullptr});
    if (std::getenv("PMSF_SC_BOUNDARY_ANCHOR_STAGE") && mesh) {
        g_scAnchors = pmsf::collectGroupAnchors(mesh.get());
        auto& frozen = pmsf::sharedBoundaryFrozenVertices();
        auto& guarded = pmsf::anchorGuardedVertices();
        int live = 0;
        const bool freezeAnchors = std::getenv("PMSF_SC_NO_ANCHOR_FREEZE") == nullptr;
        for (auto& d : g_scAnchors) {
            if (!d.v) continue;
            if (freezeAnchors) {
                frozen.insert(d.v);
                guarded.insert(d.v);
            }
            ++live;
        }
        std::fprintf(stderr, "[sc-anchors] collected=%zu live=%d frozen=%zu\n",
                     g_scAnchors.size(), live, frozen.size());
    }
    if (const char* apf = std::getenv("PMSF_SC_ENC_ALL_PREFLAGS"); apf) {
        // (diagnostic) clear once; round-loop appends per-round pre-state flags
        std::fopen(apf, "w");
    }
    if (const char* og = std::getenv("PMSF_SC_ENC_ORIG_GID_DIR")) {
        if (FILE* gf = std::fopen((std::string(og) + "/orig_gid.txt").c_str(), "w")) {
            for (MCGAL::Facet* ff : mesh->faces()) {
                if (!ff || ff->isRemoved() || !ff->proxyHalfedge()) continue;
                auto vv = pmsf::sbdetail::faceVertices(ff);
                if (!vv[0]) continue;
                std::vector<std::array<int, 3>> qs;
                for (int k = 0; k < 3; ++k) {
                    const auto q = mesh->floatPosToInt(vv[k]->point());
                    { const int mq = (mesh->i_nbQuantBits > 0) ? ((1 << mesh->i_nbQuantBits) - 1) : (1 << 30); qs.push_back({std::clamp(q.x(), 0, mq), std::clamp(q.y(), 0, mq), std::clamp(q.z(), 0, mq)}); }
                }
                std::sort(qs.begin(), qs.end());
                std::fprintf(gf, "%d %d %d %d %d %d %d %d %d %d\n", ff->groupId(), qs[0][0],
                             qs[0][1], qs[0][2], qs[1][0], qs[1][1], qs[1][2], qs[2][0],
                             qs[2][1], qs[2][2]);
            }
            std::fclose(gf);
        }
    }
        // Boundary-v2 stage invoked at an explicit point in the round order.
        // Interior elimination and collector->collect have already run when this
        // is called; the caller decides whether it goes before or after the flip
        // pass (IBF vs IFB) or before the interior (BIF, handled by the PRE block).
        auto runV2StageNow = [&](int stageRound) {
            if (!bcV2Enabled || !mesh) return;
            if (std::getenv("PMSF_SC_BOUNDARY_SKIP_STAGE") != nullptr) return;
            if (stageRound < 0 || stageRound >= static_cast<int>(g_bcV2Rounds.size())) return;
            if (!bcV2RoundSelected(stageRound)) return;
            if (scPreForRound(stageRound)) return;
            const int stagePreGroupBoundaryEdges = pmsf::countGroupBoundaryEdges(mesh.get());
            const int maxOps = boundaryMaxOpsForRound(stageRound);
            std::set<MCGAL::Vertex*> protectedVerts;
            for (auto& ag : g_bcV2Anchors)
                if (ag.second) protectedVerts.insert(ag.second);
            for (MCGAL::Halfedge* h : seeds) {
                if (!h) continue;
                if (h->vertex()) protectedVerts.insert(h->vertex());
                if (h->end_vertex()) protectedVerts.insert(h->end_vertex());
            }
            if (seed) {
                if (seed->vertex()) protectedVerts.insert(seed->vertex());
                if (seed->end_vertex()) protectedVerts.insert(seed->end_vertex());
            }
            bool runRealStage = true;
            if (std::getenv("PMSF_SC_V2_PREFLIGHT") && !std::getenv("PMSF_SC_V2_STRICT_FAIL") &&
                !pmsf::scV2PreflightStage(mesh, stageRound, maxOps, protectedVerts, g_bcV2Anchors)) {
                runRealStage = false;
            }
            pmsf::BoundaryV2Round roundOut;
            int ops = 0;
            if (runRealStage) {
                ops = pmsf::runSingleCopyBoundaryStageV2(
                    mesh, stageRound, maxOps, protectedVerts, g_bcV2Anchors, roundOut);
                if (ops < 0) {
                    const bool strictFail = std::getenv("PMSF_SC_V2_STRICT_FAIL") != nullptr;
                    if (!strictFail && pmsf::scV2RollbackRound(mesh, roundOut)) {
                        std::fprintf(stderr,
                                     "[bc-v2-stage] round=%d post-enumeration failed, "
                                     "rolled back %zu ops -> round unstaged\n",
                                     stageRound, roundOut.ops.size());
                        roundOut = pmsf::BoundaryV2Round();
                        ops = 0;
                    } else {
                        throw std::runtime_error("boundary v2 stage failed");
                    }
                }
            }
            g_bcV2Rounds[static_cast<std::size_t>(stageRound)] = std::move(roundOut);
            canonicalizeTraversalProxies(mesh);
            g_bcV2Rounds[static_cast<std::size_t>(stageRound)].groupBoundaryEdges =
                stagePreGroupBoundaryEdges;
            if (stageRound < static_cast<int>(g_scRoundOrdering.size())) {
                const int roReq = scRoundOrderFor(stageRound);
                g_scRoundOrdering[static_cast<std::size_t>(stageRound)] = roReq;
                g_scEffectiveRoundOrdering[static_cast<std::size_t>(stageRound)] =
                    (roReq == 5) ? 5 : (roReq == 2) ? 2 : ((roReq == 3 || roReq == 4) ? 4 : 0);
            }
            if (std::getenv("PMSF_SC_DEBUG"))
                std::fprintf(stderr,
                             "[bc-v2-stage] round=%d order=%d ops=%d edges=%d bits=%zu\n", stageRound,
                             scV2StageOrder, ops,
                             g_bcV2Rounds[static_cast<std::size_t>(stageRound)].edgeCount,
                             g_bcV2Rounds[static_cast<std::size_t>(stageRound)].bits.size());
        };

        auto runFlipPass = [&](int flipRound) {
            if (!options.isEnableEdgeFlipPostProcess() ||
                std::getenv("PMSF_EDGEFLIP_POST_ONLY") != nullptr)
                return;
            char flipRoundBeforePath[512];
            char flipRoundAfterPath[512];
            if (dumpEdgeFlipRounds) {
                std::filesystem::create_directories("build/aaaaresult/compress/flip");
                std::snprintf(flipRoundBeforePath, sizeof(flipRoundBeforePath),
                              "build/aaaaresult/compress/flip/round_%03d_before.off", flipRound);
                std::snprintf(flipRoundAfterPath, sizeof(flipRoundAfterPath),
                              "build/aaaaresult/compress/flip/round_%03d_after.off", flipRound);
                mesh->dumpto_oldtype(flipRoundBeforePath);
            }
            int flipsThisRound = 0;
            bool enableThisRound = true;
            if (std::getenv("PMSF_EDGEFLIP_LAST_ONLY")) {
                enableThisRound = (flipRound == options.getRound() - 1);
            }
            // Arbitrary flip-round schedule: the artifact records per-round flip
            // counts, so the decoder does not need this env to undo the flips.
            if (const char* flipRounds = std::getenv("PMSF_EDGEFLIP_ROUND_LIST")) {
                enableThisRound = false;
                const std::string rs(flipRounds);
                std::size_t p = 0;
                while (p <= rs.size()) {
                    const std::size_t comma = rs.find(',', p);
                    const std::string tok =
                        rs.substr(p, comma == std::string::npos ? std::string::npos : comma - p);
                    if (!tok.empty() && std::atoi(tok.c_str()) == flipRound) {
                        enableThisRound = true;
                        break;
                    }
                    if (comma == std::string::npos) break;
                    p = comma + 1;
                }
            }
            if (enableThisRound) {
                EdgeFlipPostProcessOperator::Config efCfg;
                efCfg.rounds = 1;
                efCfg.longEdgeRatio = options.getEdgeFlipPostProcessLongEdgeRatio();
                efCfg.minImprovementRatio =
                    options.getEdgeFlipPostProcessMinImprovementRatio();
                if (const char* v = std::getenv("PMSF_EDGEFLIP_LONG_RATIO")) {
                    efCfg.longEdgeRatio = std::atof(v);
                }
                if (const char* v = std::getenv("PMSF_EDGEFLIP_MIN_IMPROVEMENT")) {
                    efCfg.minImprovementRatio = std::atof(v);
                }
                efCfg.maxFlipsPerRoundPass =
                    envInt("PMSF_EDGEFLIP_MAX_PER_ROUND", -1);
                if (std::getenv("PMSF_EDGEFLIP_STORE_PREFLIP_BASE") &&
                    !preFlipBaseForSerialize) {
                    preFlipBaseForSerialize = cloneMeshForPipelinePreflip(mesh);
                }
                if (const char* ef = std::getenv("PMSF_EDGEFLIP_ENCFLAGS_DIR"); ef && mesh) {
                    std::filesystem::create_directories(ef);
                    char fpath[512];
                    std::snprintf(fpath, sizeof(fpath), "%s/enc_flags_%03d.txt", ef, flipRound);
                    if (FILE* ff = std::fopen(fpath, "w")) {
                        const int mq = (mesh->i_nbQuantBits > 0)
                                           ? ((1 << mesh->i_nbQuantBits) - 1)
                                           : (1 << 30);
                        for (MCGAL::Facet* ffac : mesh->faces()) {
                            if (!ffac || ffac->isRemoved() || !ffac->proxyHalfedge()) continue;
                            std::vector<std::array<int, 3>> qs;
                            for (MCGAL::Halfedge* hh : {ffac->proxyHalfedge(),
                                                       ffac->proxyHalfedge()->next(),
                                                       ffac->proxyHalfedge()->next()->next()}) {
                                if (!hh || !hh->vertex()) continue;
                                const auto q = mesh->floatPosToInt(hh->vertex()->point());
                                qs.push_back({std::clamp(q.x(), 0, mq), std::clamp(q.y(), 0, mq),
                                              std::clamp(q.z(), 0, mq)});
                            }
                            if (qs.size() != 3) continue;
                            std::sort(qs.begin(), qs.end());
                            int amask = 0;
                            std::vector<std::pair<std::array<int, 3>, MCGAL::Vertex*>> items;
                            for (MCGAL::Halfedge* hh : {ffac->proxyHalfedge(),
                                                       ffac->proxyHalfedge()->next(),
                                                       ffac->proxyHalfedge()->next()->next()}) {
                                if (!hh || !hh->vertex()) continue;
                                const auto q = mesh->floatPosToInt(hh->vertex()->point());
                                items.push_back({{std::clamp(q.x(), 0, mq), std::clamp(q.y(), 0, mq),
                                                  std::clamp(q.z(), 0, mq)},
                                                 hh->vertex()});
                            }
                            std::sort(items.begin(), items.end(),
                                      [](const auto& a, const auto& b) { return a.first < b.first; });
                            for (int pos = 0; pos < 3; ++pos) {
                                for (MCGAL::Halfedge* hh : {ffac->proxyHalfedge(),
                                                            ffac->proxyHalfedge()->next(),
                                                            ffac->proxyHalfedge()->next()->next()}) {
                                    if (hh && hh->vertex() == items[pos].second &&
                                        hh->isAdded())
                                        amask |= (1 << pos);
                                }
                            }
                            std::fprintf(ff, "%d %d %d %d %d %d %d %d %d %d %d %d %d\n",
                                         ffac->groupId(), ffac->isSplittable() ? 1 : 0,
                                         ffac->isUnsplittable() ? 1 : 0, qs[0][0], qs[0][1],
                                         qs[0][2], qs[1][0], qs[1][1], qs[1][2], qs[2][0],
                                         qs[2][1], qs[2][2], amask);
                        }
                        std::fclose(ff);
                    }
                }
                EdgeFlipPostProcessOperator edgeFlipOp(efCfg);
                if (seed && seed->vertex() && seed->end_vertex()) {
                    edgeFlipOp.setProtectedSeedVertexIds(seed->vertex()->poolId(),
                                                         seed->end_vertex()->poolId());
                    edgeFlipOp.setSeedHalfedge(seed);
                }
                edgeFlipOp.process(mesh);
                canonicalizeTraversalProxies(mesh);
                const auto& counts = edgeFlipOp.roundFlipCounts();
                if (!counts.empty()) flipsThisRound = counts.front();
                const auto& ids = edgeFlipOp.flipSymbols();
                edgeFlipSymbols.insert(edgeFlipSymbols.end(), ids.begin(), ids.end());
                const auto& coords = edgeFlipOp.flipCoordSymbols();
                edgeFlipCoordSymbols.insert(edgeFlipCoordSymbols.end(),
                                            coords.begin(),
                                            coords.end());
                const auto& bitCounts = edgeFlipOp.roundCandidateBitCounts();
                edgeFlipRoundBitCounts.insert(edgeFlipRoundBitCounts.end(),
                                              bitCounts.begin(),
                                              bitCounts.end());
                const auto& bits = edgeFlipOp.roundCandidateBits();
                edgeFlipRoundBits.insert(edgeFlipRoundBits.end(),
                                         bits.begin(),
                                         bits.end());
                if (std::getenv("PMSF_EDGEFLIP_DEBUG"))
                    std::fprintf(stderr, "[ef-round-bits] flipRound=%d bitCounts=%zu bits=%zu totalRounds=%zu\n",
                                 flipRound, bitCounts.size(), bits.size(), edgeFlipRoundBitCounts.size());
                if (edgeFlipProtectedVertexIds) {
                    for (int vid : edgeFlipOp.protectedVertexIds()) {
                        edgeFlipProtectedVertexIds->insert(vid);
                    }
                }
            } else {
                edgeFlipRoundBitCounts.push_back(0);
            }
            edgeFlipRoundCounts.push_back(flipsThisRound);
            if (dumpEdgeFlipRounds) {
                mesh->dumpto_oldtype(flipRoundAfterPath);
            }

            if (dumpPipelineRounds) {
                std::filesystem::create_directories("build/aaaaresult/compress/rounds");
                char roundPostFlipPath[512];
                std::snprintf(roundPostFlipPath, sizeof(roundPostFlipPath),
                              "build/aaaaresult/compress/rounds/round_%03d_postflip.off", flipRound);
                mesh->dumpto_oldtype(roundPostFlipPath);
            }
        };

    for (int i = 0; i < options.getRound(); i++) {
        pmsf::boundaryTopologyRound() = i;
        if (i < static_cast<int>(g_scEffectiveRoundOrdering.size())) {
            const int ro0 = scRoundOrderFor(i);
            g_scEffectiveRoundOrdering[static_cast<std::size_t>(i)] =
                (ro0 == 5) ? 5 : (ro0 == 2) ? 2 : ((ro0 == 3 || ro0 == 4) ? 4 : 0);
        }
        // [experiment, env-gated] Decoder resets element flags every round via
        // resetRoundElementState; the encoder only resets the subset reachable from
        // live facets (selector init -> Mesh::resetState).  With boundary stages that
        // churn/erase facets, elements enclosed only by removed facets keep stale
        // added/splittable flags and drift the interior replay a few rounds later.
        // Full reset makes encoder/decoder flag state symmetric.
        if (i > 0 && std::getenv("PMSF_SC_ENC_FULL_ROUND_RESET"))
            pmsf::resetRoundElementState(mesh);
        if (const char* apf = std::getenv("PMSF_SC_ENC_ALL_PREFLAGS"); apf) {
            if (FILE* ff = std::fopen(apf, i == 0 ? "w" : "a")) {
                std::fprintf(ff, "ROUND %d\n", i);
                for (MCGAL::Facet* ffac : mesh->faces()) {
                    if (!ffac || ffac->isRemoved() || !ffac->proxyHalfedge()) continue;
                    const auto vs = pmsf::sbdetail::faceVertices(ffac);
                    if (!vs[0]) continue;
                    std::vector<std::array<int, 3>> qs;
                    for (int k = 0; k < 3; ++k) {
                        const auto q = mesh->floatPosToInt(vs[k]->point());
                        { const int mq = (mesh->i_nbQuantBits > 0) ? ((1 << mesh->i_nbQuantBits) - 1) : (1 << 30); qs.push_back({std::clamp(q.x(), 0, mq), std::clamp(q.y(), 0, mq), std::clamp(q.z(), 0, mq)}); }
                    }
                    std::sort(qs.begin(), qs.end());
                    int amask = 0;
                    {
                        const auto fvs = pmsf::sbdetail::faceVertices(ffac);
                        for (int k = 0; k < 3; ++k) {
                            MCGAL::Vertex* fv = fvs[k];
                            if (!fv) continue;
                            MCGAL::PointInt fq = mesh->floatPosToInt(fv->point());
                            std::array<int, 3> key = {fq.x(), fq.y(), fq.z()};
                            int pos = -1;
                            for (int pp = 0; pp < 3; ++pp)
                                if (qs[static_cast<std::size_t>(pp)] == key) { pos = pp; break; }
                            if (pos < 0) continue;
                            for (MCGAL::Halfedge* hh : {ffac->proxyHalfedge(), ffac->proxyHalfedge()->next(),
                                                       ffac->proxyHalfedge()->next()->next()}) {
                                if (hh && hh->vertex() == fv) {
                                    if (hh->isAdded()) amask |= (1 << pos);
                                    break;
                                }
                            }
                        }
                    }
                    std::fprintf(ff, "%d %d %d %d %d %d %d %d %d %d %d %d %d\n", ffac->groupId(),
                                 ffac->isSplittable() ? 1 : 0, ffac->isUnsplittable() ? 1 : 0,
                                 qs[0][0], qs[0][1], qs[0][2], qs[1][0], qs[1][1], qs[1][2],
                                 qs[2][0], qs[2][1], qs[2][2], amask);
                }
                std::fclose(ff);
            }
        }
        if (boundaryStageBeforeInterior) {
            const int bRemoved = runBoundaryStage(i);
            if (bRemoved > 0 && std::getenv("PMSF_BOUNDARY_DEBUG"))
                std::fprintf(stderr, "[boundary-stage] round=%d before removed=%d\n", i, bRemoved);
        }
        if (bcV2Enabled && scPreForRound(i) && bcV2RoundSelected(i) && mesh &&
            std::getenv("PMSF_SC_BOUNDARY_SKIP_STAGE") == nullptr &&
            i < static_cast<int>(g_bcV2Rounds.size())) {
            const int preGroupBoundaryEdges = pmsf::countGroupBoundaryEdges(mesh.get());
            const int maxOps = boundaryMaxOpsForRound(i);
            std::set<MCGAL::Vertex*> protectedVerts;
            for (auto& ag : g_bcV2Anchors)
                if (ag.second) protectedVerts.insert(ag.second);
            for (MCGAL::Halfedge* h : seeds) {
                if (!h) continue;
                if (h->vertex()) protectedVerts.insert(h->vertex());
                if (h->end_vertex()) protectedVerts.insert(h->end_vertex());
            }
            if (seed) {
                if (seed->vertex()) protectedVerts.insert(seed->vertex());
                if (seed->end_vertex()) protectedVerts.insert(seed->end_vertex());
            }
            bool runRealStage = true;
            if (std::getenv("PMSF_SC_V2_PREFLIGHT") && !std::getenv("PMSF_SC_V2_STRICT_FAIL") &&
                !pmsf::scV2PreflightStage(mesh, i, maxOps, protectedVerts, g_bcV2Anchors)) {
                runRealStage = false;
            }
            pmsf::BoundaryV2Round roundOut;
            int ops = 0;
            if (runRealStage) {
                int batches = 1;
                if (const char* be = std::getenv("PMSF_SC_BOUNDARY_BATCHES")) {
                    const int b = std::atoi(be);
                    if (b > 0) batches = b;
                }
                if (batches > 1) {
                    // Several small stage calls inside one round, re-enumerating and
                    // re-canonicalizing between batches.  The decoder consumes the
                    // recorded op list, so artifacts remain self-describing.
                    for (int b = 0; b < batches; ++b) {
                        pmsf::BoundaryV2Round one;
                        const int o = pmsf::runSingleCopyBoundaryStageV2(
                            mesh, i, maxOps, protectedVerts, g_bcV2Anchors, one);
                        if (o <= 0) break;
                        roundOut.ops.insert(roundOut.ops.end(), one.ops.begin(), one.ops.end());
                        ops += o;
                        canonicalizeTraversalProxies(mesh);
                    }
                } else {
                    ops = pmsf::runSingleCopyBoundaryStageV2(
                        mesh, i, maxOps, protectedVerts, g_bcV2Anchors, roundOut);
                }
                if (ops < 0) {
                    const bool strictFail = std::getenv("PMSF_SC_V2_STRICT_FAIL") != nullptr;
                    if (!strictFail && pmsf::scV2RollbackRound(mesh, roundOut)) {
                        std::fprintf(stderr,
                                     "[bc-v2-stage-pre] round=%d post-enumeration failed, "
                                     "rolled back %zu ops -> round unstaged\n",
                                     i, roundOut.ops.size());
                        roundOut = pmsf::BoundaryV2Round();
                        ops = 0;
                    } else {
                        throw std::runtime_error("boundary v2 stage failed");
                    }
                }
            }
            g_bcV2Rounds[static_cast<std::size_t>(i)] = std::move(roundOut);
            canonicalizeTraversalProxies(mesh);
            g_bcV2Rounds[static_cast<std::size_t>(i)].groupBoundaryEdges =
                preGroupBoundaryEdges;
            if (i < static_cast<int>(g_scRoundOrdering.size()))
                g_scRoundOrdering[static_cast<std::size_t>(i)] = scRoundOrderFor(i);
            if (std::getenv("PMSF_SC_DEBUG"))
                std::fprintf(stderr,
                             "[bc-v2-stage-pre] round=%d ops=%d edges=%d bits=%zu\n", i, ops,
                             g_bcV2Rounds[static_cast<std::size_t>(i)].edgeCount,
                             g_bcV2Rounds[static_cast<std::size_t>(i)].bits.size());
        }
        if (!bcV2Enabled && scPreForRound(i) &&
            (std::getenv("PMSF_SC_BOUNDARY_ANCHOR_STAGE") ||
             std::getenv("PMSF_SC_BOUNDARY_GROUP_STAGE")) &&
            std::getenv("PMSF_SC_BOUNDARY_SKIP_STAGE") == nullptr &&
            i < static_cast<int>(g_scBoundaryRoundData.size())) {
            bool runPreStage = true;
            if (const char* onlyRounds = std::getenv("PMSF_SC_BOUNDARY_ROUNDS")) {
                runPreStage = false;
                const std::string rs(onlyRounds);
                std::size_t p = 0;
                while (p <= rs.size()) {
                    const std::size_t comma = rs.find(',', p);
                    const std::string tok = rs.substr(
                        p, comma == std::string::npos ? std::string::npos : comma - p);
                    if (!tok.empty() && std::atoi(tok.c_str()) == i) { runPreStage = true; break; }
                    if (comma == std::string::npos) break;
                    p = comma + 1;
                }
            }
            if (runPreStage) {
                std::set<MCGAL::Vertex*> protectedVerts;
                for (MCGAL::Vertex* a : groupAnchors)
                    if (a) protectedVerts.insert(a);
                for (MCGAL::Halfedge* h : seeds) {
                    if (!h) continue;
                    if (h->vertex()) protectedVerts.insert(h->vertex());
                    if (h->end_vertex()) protectedVerts.insert(h->end_vertex());
                }
                if (seed) {
                    if (seed->vertex()) protectedVerts.insert(seed->vertex());
                    if (seed->end_vertex()) protectedVerts.insert(seed->end_vertex());
                }
                for (const auto& d : g_scAnchors)
                    if (d.v) protectedVerts.insert(d.v);
                const int maxOps = boundaryMaxOpsForRound(i);
                std::vector<int> roundData;
                const bool preGroup =
                    std::getenv("PMSF_SC_BOUNDARY_GROUP_STAGE") != nullptr;
                const int effectiveOps =
                    preGroup ? pmsf::scFindSafeGroupMaxOps(mesh.get(), i, maxOps, protectedVerts)
                             : maxOps;
                const int appliedOps =
                    preGroup ? pmsf::runSingleCopyBoundaryEncoderStageGroup(
                                   mesh, i, effectiveOps, protectedVerts, roundData)
                             : pmsf::runSingleCopyBoundaryEncoderStageAnchor(
                                   mesh, i, maxOps, protectedVerts, g_scAnchors, roundData);
                // PRE order: collect happens after this stage, record the seed used.
                if (seed && seed->vertex() && seed->end_vertex() && !seed->isRemoved() &&
                    !seed->vertex()->isRemoved() && !seed->end_vertex()->isRemoved() &&
                    static_cast<std::size_t>(i) < g_scRoundSeedQ.size()) {
                    const int maxQ = (1 << mesh->i_nbQuantBits) - 1;
                    auto cq = [&](int x) { return std::clamp(x, 0, maxQ); };
                    const auto p0 = mesh->floatPosToInt(seed->vertex()->point());
                    const auto p1 = mesh->floatPosToInt(seed->end_vertex()->point());
                    g_scRoundSeedQ[static_cast<std::size_t>(i)] = {
                        cq(p0.x()), cq(p0.y()), cq(p0.z()), cq(p1.x()), cq(p1.y()), cq(p1.z())};
                }
                g_scBoundaryRoundData[static_cast<std::size_t>(i)] = std::move(roundData);
                if (i < static_cast<int>(g_scRoundOrdering.size())) {
                    g_scRoundOrdering[static_cast<std::size_t>(i)] = 1;
                    const int roPre = scRoundOrderFor(i);
                    g_scEffectiveRoundOrdering[static_cast<std::size_t>(i)] =
                        (roPre == 3 || roPre == 4 || roPre == 5) ? 3 : 1;
                }
                pmsf::canonicalizeBoundaryMeshOrder(mesh);
                if (std::getenv("PMSF_SC_DEBUG"))
                    std::fprintf(stderr,
                                 "[sc-boundary-stage-pre] round=%d ops=%d resInts=%zu\n", i,
                                 appliedOps,
                                 g_scBoundaryRoundData[static_cast<std::size_t>(i)].size());
            }
        }

        if (i == 1 && std::getenv("PMSF_ANCHOR_ONERING_LOCK") && !groupAnchors.empty()) {
            const bool guardLock = std::getenv("PMSF_ANCHOR_ONERING_LOCK_GUARD") != nullptr;
            auto& frozen = pmsf::sharedBoundaryFrozenVertices();
            auto& guarded = pmsf::anchorGuardedVertices();
            auto& target = guardLock ? guarded : frozen;
            int locked = 0;
            for (MCGAL::Vertex* a : groupAnchors) {
                if (!a || a->isRemoved()) continue;
                target.insert(a);
                for (MCGAL::Halfedge* h : pmsf::boundedRing(a)) {
                    if (h && h->end_vertex() && !h->end_vertex()->isRemoved()) {
                        target.insert(h->end_vertex());
                        ++locked;
                    }
                }
            }
            std::fprintf(stderr, "[anchor-one-ring] locked=%d frozen=%zu guarded=%zu\n",
                         locked, frozen.size(), guarded.size());
        }
        if (std::getenv("PMSF_SEG_BFS_ROUND")) {
            bfsRelabelGroups(mesh);
            if (std::getenv("PMSF_SEG_LABEL_HASH") && mesh) {
                unsigned long long h = 1469598103934665603ull;
                auto mix = [&](long long v) { h ^= (unsigned long long)v; h *= 1099511628211ull; };
                for (MCGAL::Facet* f : mesh->faces()) {
                    if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                    std::vector<std::array<int, 3>> vs;
                    MCGAL::Halfedge* h0 = f->proxyHalfedge();
                    MCGAL::Halfedge* hh = h0; int guard = 0;
                    do {
                        if (hh && hh->vertex()) {
                            auto q = mesh->floatPosToInt(hh->vertex()->point());
                            vs.push_back({q.x(), q.y(), q.z()});
                        }
                        hh = hh ? hh->next() : nullptr;
                    } while (hh && hh != h0 && ++guard < 64);
                    std::sort(vs.begin(), vs.end());
                    for (auto& t : vs) { mix(t[0]); mix(t[1]); mix(t[2]); }
                    mix(f->groupId());
                }
                std::fprintf(stderr, "[seg-label-hash] enc round=%d hash=%llu\n", i,
                             (unsigned long long)h);
            }
        }
        pmsf::currentCompressRoundForGuard = i;
        if (std::getenv("PMSF_BOUNDARY_SELECT_STAGE")) {
            const std::string rs = std::to_string(i);
            setenv("PMSF_BOUNDARY_SELECT_ROUND", rs.c_str(), 1);
        }
        if (cfg.trackContextLevel) {
            MCGAL::contextPool.setMetadata("level", MCGAL::ContextValue(i));
        }
        if (cfg.beforeRound) {
            cfg.beforeRound(i, mesh);
        }
        const int patchOnlyRound = std::getenv("PMSF_SHARED_BOUNDARY_PATCH_ONLY_ROUND")
                                      ? std::atoi(std::getenv("PMSF_SHARED_BOUNDARY_PATCH_ONLY_ROUND"))
                                      : -1;
        const bool patchRoundSelected = patchRoundSet.empty() || patchRoundSet.count(i) > 0;
        if (patchPerRound && !sharedPatchAfterInterior && patchRoundSelected &&
            (patchOnlyRound < 0 || patchOnlyRound == i))
            applySharedBoundaryPatch(i);
        else if (patchAfterRound == i) applySharedBoundaryPatch(i);
        if (std::getenv("PMSF_SHARED_BOUNDARY_TEST") && sbAfterRound == i) applySharedBoundary();
        // 历史 APTM 行为：pre-process 每轮前跑一次（anchor 重新标注）
        if (cfg.preProcessPosition == CompressPipelineConfig::PreProcessPosition::EveryRound
            && preProcessOperator) {
            auto tp = now();
            preProcessOperator->process(mesh);
            t_preprocess += now() - tp;
        }

        if (cfg.enableIndependentSet) {
            misSelector.clear();
        }

        {
            const int roPreStage = scRoundOrderFor(i);
            // F-first orders run the flip pass before the interior elimination.
            if (roPreStage == 3 || roPreStage == 4 || roPreStage == 5) runFlipPass(i);
            // FBI: boundary stage between the flip and the interior elimination.
            if (roPreStage == 5) runV2StageNow(i);
        }
        perRoundGates.clear();
        // Phase 0 · Step 0.4 —— 在 round loop 顶部重置 eliminator 的 per-round
        // patch 累加器。基类默认 no-op，对 Binary / Columnar baseline 零影响；
        // 仅 Valence-Patch-Only 等 Phase 1+ collector 会调用对应的 take*() 取走。
        eliminate->resetRoundAccumulators();
        int roundEliminationCount = 0;

        const bool mixedBoundary = std::getenv("PMSF_SEAM_MIXED_BOUNDARY") != nullptr;
        const bool orderedLongRoundsOk =
            options.getRound() <= 6 || std::getenv("PMSF_SEAM_BOUNDARY_ALLOW_LONG_ROUNDS");
        if (!std::getenv("PMSF_SEAM_BOUNDARY_ROUND_END") &&
            std::getenv("PMSF_SEAM_BOUNDARY_ORDER_BITS") && std::getenv("PMSF_SEAM_COMPRESS")) {
            std::vector<unsigned char> bits;
            std::vector<int> residuals;
            mesh->garbage_collection();
            const auto bedges = pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(mesh);
            for (MCGAL::Halfedge* h : bedges) {
                if (!h || h->isRemoved() || h->opposite() != nullptr) continue;
                MCGAL::Vertex* v = h->end_vertex();
                if (!v || v->isRemoved()) continue;
                if (seed && seed->vertex() && seed->end_vertex() &&
                    (v == seed->vertex() || v == seed->end_vertex())) continue;
                const MCGAL::PointInt oldInt = mesh->floatPosToInt(v->point());
                const int orderedVPid = v->poolId();
                MCGAL::Halfedge* newEdge = nullptr;
                if (!pmsf::collapseBoundaryVertex3Robust(mesh, v, &newEdge)) continue;
                if (!newEdge) continue;
                const auto order = pmsf::collectBoundaryEdgesInOrder(mesh);
                int edgePos = -1;
                for (size_t oi = 0; oi < order.size(); ++oi) {
                    if (order[oi] == newEdge) { edgePos = static_cast<int>(oi); break; }
                }
                if (edgePos < 0) continue;
                bits.assign(order.size(), 0);
                bits[static_cast<size_t>(edgePos)] = 1;
                const MCGAL::PointInt ai = mesh->floatPosToInt(newEdge->vertex()->point());
                const MCGAL::PointInt bi = mesh->floatPosToInt(newEdge->end_vertex()->point());
                const MCGAL::PointInt mid((ai.x() + bi.x()) / 2,
                                          (ai.y() + bi.y()) / 2,
                                          (ai.z() + bi.z()) / 2);
                residuals.push_back(oldInt.x() - mid.x());
                residuals.push_back(oldInt.y() - mid.y());
                residuals.push_back(oldInt.z() - mid.z());
                break;
            }
            boundaryRoundBitCounts.push_back(static_cast<int>(bits.size()));
            boundaryRoundBits.insert(boundaryRoundBits.end(), bits.begin(), bits.end());
            boundaryRoundResiduals.insert(boundaryRoundResiduals.end(), residuals.begin(), residuals.end());
            if (std::getenv("PMSF_SEAM_DEBUG"))
                std::cerr << "[seam-boundary-orderbits] round=" << i
                          << " boundaryEdges=" << bits.size()
                          << " V=" << mesh->size_of_vertices() << " F=" << mesh->size_of_facets() << std::endl;
        } else if (!std::getenv("PMSF_SEAM_BOUNDARY_ROUND_END") &&
                   ((std::getenv("PMSF_SEAM_BOUNDARY_ORDERED") && !mixedBoundary) ||
             (mixedBoundary && i > 0)) && std::getenv("PMSF_SEAM_COMPRESS") &&
            orderedLongRoundsOk) {
            std::vector<unsigned char> bits;
            std::vector<int> residuals;
            int applied = 0;
            const char* ordMaxEnv = std::getenv("PMSF_SEAM_BOUNDARY_ORDERED_MAXOPS");
            const int ordMaxOps = ordMaxEnv ? std::max(1, std::atoi(ordMaxEnv)) : 1;
            while (applied < ordMaxOps) {
            if (std::getenv("PMSF_SEAM_BOUNDARY_CANON")) pmsf::canonicalizeMeshOrder(mesh);
            else mesh->garbage_collection();
            const auto bedges = pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(mesh);
            bool did = false;
            for (MCGAL::Halfedge* h : bedges) {
                if (!h || h->isRemoved() || h->opposite() != nullptr) continue;
                MCGAL::Vertex* v = h->end_vertex();
                if (!v || v->isRemoved()) continue;
                if (seed && seed->vertex() && seed->end_vertex() &&
                    (v == seed->vertex() || v == seed->end_vertex())) continue;
                std::vector<MCGAL::Facet*> fan;
                for (MCGAL::Halfedge* hv : pmsf::boundedRing(v)) {
                    if (hv && hv->face() && !hv->face()->isRemoved()) fan.push_back(hv->face());
                }
                if (fan.size() != 2) continue;
                int origVidx = -1;
                for (size_t vi = 0; vi < mesh->vertices().size(); ++vi) {
                    if (mesh->vertices()[vi] == v) { origVidx = static_cast<int>(vi); break; }
                }
                if (origVidx < 0) continue;
                std::vector<int> faceIdx;
                for (size_t fi = 0; fi < mesh->faces().size(); ++fi) {
                    if (mesh->faces()[fi] == fan[0] || mesh->faces()[fi] == fan[1])
                        faceIdx.push_back(static_cast<int>(fi));
                }
                if (faceIdx.size() != 2) continue;
                if (faceIdx[0] > faceIdx[1]) std::swap(faceIdx[0], faceIdx[1]);
                const MCGAL::PointInt oldInt = mesh->floatPosToInt(v->point());
                const int orderedVPid = v->poolId();
                const int orderedF0Pid = fan[0] ? fan[0]->poolId() : -1;
                const int orderedF1Pid = fan[1] ? fan[1]->poolId() : -1;
                std::vector<int> orderedHalfPids;
                try { if (std::getenv("PMSF_SEAM_BOUNDARY_RESTORE_HALF_IDS")) {
                    for (MCGAL::Facet* ff : fan) {
                        MCGAL::Halfedge* hh0 = ff ? ff->proxyHalfedge() : nullptr;
                        MCGAL::Halfedge* hh = hh0;
                        int gg = 0;
                        do { if (hh) orderedHalfPids.push_back(hh->poolId()); hh = hh ? hh->next() : nullptr; } while (hh && hh != hh0 && ++gg < 8);
                    }
                } } catch (...) {}
                MCGAL::Halfedge* newEdge = nullptr;
                if (!pmsf::collapseBoundaryVertex3Robust(mesh, v, &newEdge)) continue;
                if (!newEdge || !newEdge->vertex() || !newEdge->end_vertex()) continue;
                const auto postEdges = pmsf::edgeflip::collectMeshBoundaryEdgesSorted(mesh);
                int edgeIndex = -1;
                for (size_t ei = 0; ei < postEdges.size(); ++ei) {
                    if (postEdges[ei] == newEdge) { edgeIndex = static_cast<int>(ei); break; }
                }
                if (edgeIndex < 0) continue;
                const MCGAL::PointInt ai = mesh->floatPosToInt(newEdge->vertex()->point());
                const MCGAL::PointInt bi = mesh->floatPosToInt(newEdge->end_vertex()->point());
                const MCGAL::PointInt mid((ai.x() + bi.x()) / 2, (ai.y() + bi.y()) / 2, (ai.z() + bi.z()) / 2);
                bits.push_back(1);
                residuals.push_back(edgeIndex);
                residuals.push_back(oldInt.x() - mid.x());
                residuals.push_back(oldInt.y() - mid.y());
                residuals.push_back(oldInt.z() - mid.z());
                if (!std::getenv("PMSF_SEAM_BOUNDARY_ORDERED_NOVERTEX") &&
                    !std::getenv("PMSF_SEAM_MIXED_BOUNDARY")) {
                    residuals.push_back(origVidx);
                    if (std::getenv("PMSF_SEAM_BOUNDARY_RESTORE_IDS"))
                        residuals.push_back(orderedVPid);
                }
                if (!std::getenv("PMSF_SEAM_BOUNDARY_ORDERED_NOFACE") &&
                    !std::getenv("PMSF_SEAM_MIXED_BOUNDARY") &&
                    !std::getenv("PMSF_SEAM_BOUNDARY_ORDERED_NOVERTEX")) {
                    residuals.push_back(faceIdx[0]);
                    residuals.push_back(faceIdx[1]);
                    if (std::getenv("PMSF_SEAM_BOUNDARY_RESTORE_FACE_IDS")) {
                        residuals.push_back(orderedF0Pid);
                        residuals.push_back(orderedF1Pid);
                    }
                    if (std::getenv("PMSF_SEAM_BOUNDARY_RESTORE_HALF_IDS")) {
                        for (int hp : orderedHalfPids) residuals.push_back(hp);
                    }
                }
                ++applied;
                if (std::getenv("PMSF_SEAM_BOUNDARY_CANON")) pmsf::canonicalizeMeshOrder(mesh);
                did = true;
                break;
            }
            if (!did) break;
            }
            boundaryRoundBitCounts.push_back(static_cast<int>(bits.size()));
            boundaryRoundBits.insert(boundaryRoundBits.end(), bits.begin(), bits.end());
            boundaryRoundResiduals.insert(boundaryRoundResiduals.end(), residuals.begin(), residuals.end());
            if (std::getenv("PMSF_SEAM_DEBUG"))
                std::cerr << "[seam-boundary-ordered] round=" << i << " applied=" << applied
                          << " V=" << mesh->size_of_vertices() << " F=" << mesh->size_of_facets() << std::endl;
        } else if (((std::getenv("PMSF_SEAM_BOUNDARY") && !mixedBoundary) ||
                    (mixedBoundary && i == 0)) &&
                   std::getenv("PMSF_SEAM_COMPRESS") &&
                   (i == 0 || std::getenv("PMSF_SEAM_BOUNDARY_ALL_ROUNDS")) &&
                   (!std::getenv("PMSF_SEAM_BOUNDARY_MAX_ROUND") ||
                    i <= std::atoi(std::getenv("PMSF_SEAM_BOUNDARY_MAX_ROUND")))) {
            std::vector<int> residuals;
            int applied = 0;
            const int maxOps = std::getenv("PMSF_SEAM_BOUNDARY_MAXOPS")
                                   ? std::max(1, std::atoi(std::getenv("PMSF_SEAM_BOUNDARY_MAXOPS")))
                                   : 256;
            std::set<std::array<int, 3>> rejectedCoords;
            auto badCounts = [&](int& dangling, int& dupFaces) {
                dangling = 0;
                dupFaces = 0;
                std::map<std::array<int, 9>, int> seen;
                for (MCGAL::Facet* lf : mesh->faces()) {
                    if (!lf || lf->isRemoved() || !lf->proxyHalfedge()) continue;
                    MCGAL::Vertex* lv[3];
                    int ln = 0;
                    MCGAL::Halfedge* lh0 = lf->proxyHalfedge();
                    MCGAL::Halfedge* lh = lh0;
                    int lg = 0;
                    do {
                        if (lh && lh->vertex()) {
                            if (lh->vertex()->isRemoved()) ++dangling;
                            if (ln < 3) lv[ln++] = lh->vertex();
                        }
                        lh = lh ? lh->next() : nullptr;
                    } while (lh && lh != lh0 && ++lg < 64);
                    if (ln != 3 || dangling) continue;
                    std::array<int, 9> key{};
                    for (int k = 0; k < 3; ++k) {
                        auto q = mesh->floatPosToInt(lv[k]->point());
                        key[static_cast<std::size_t>(3 * k)] = q.x();
                        key[static_cast<std::size_t>(3 * k + 1)] = q.y();
                        key[static_cast<std::size_t>(3 * k + 2)] = q.z();
                    }
                    std::sort(key.begin(), key.end());
                    if (++seen[key] > 1) ++dupFaces;
                }
            };
            const auto gidByPos = pmsf::buildVertexPosGroups(mesh);
            while (applied < maxOps) {
                const auto bedges = pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(mesh);
                bool did = false;
                for (MCGAL::Halfedge* h : bedges) {
                    if (!h || h->isRemoved() || h->opposite() != nullptr) continue;
                    MCGAL::Vertex* v = h->end_vertex();
                    if (!v || v->isRemoved()) continue;
                    if (seed && seed->vertex() && seed->end_vertex() &&
                        (v == seed->vertex() || v == seed->end_vertex())) continue;
                    // Only seam-cut boundary vertices are exactly restorable by
                    // mergeByPosition; collapsing true open-mesh boundary vertices
                    // cannot be undone. Skip them.
                    if (!pmsf::isSeamBoundaryVertex(gidByPos, mesh, v)) continue;
                    const MCGAL::PointInt oldInt = mesh->floatPosToInt(v->point());
                    if (rejectedCoords.count({oldInt.x(), oldInt.y(), oldInt.z()})) continue;
                    MCGAL::Halfedge* newEdge = nullptr;
                    if (!pmsf::collapseBoundaryVertex3Robust(mesh, v, &newEdge)) continue;
                    if (!newEdge || !newEdge->vertex() || !newEdge->end_vertex()) continue;
                    const auto postEdges = pmsf::edgeflip::collectMeshBoundaryEdgesSorted(mesh);
                    int edgeIndex = -1;
                    for (size_t pei = 0; pei < postEdges.size(); ++pei) {
                        if (postEdges[pei] == newEdge) { edgeIndex = static_cast<int>(pei); break; }
                    }
                    if (edgeIndex < 0) continue;
                    const MCGAL::PointInt ai = mesh->floatPosToInt(newEdge->vertex()->point());
                    const MCGAL::PointInt bi = mesh->floatPosToInt(newEdge->end_vertex()->point());
                    const MCGAL::PointInt mid((ai.x() + bi.x()) / 2,
                                              (ai.y() + bi.y()) / 2,
                                              (ai.z() + bi.z()) / 2);
                    residuals.push_back(ai.x());
                    residuals.push_back(ai.y());
                    residuals.push_back(ai.z());
                    residuals.push_back(bi.x());
                    residuals.push_back(bi.y());
                    residuals.push_back(bi.z());
                    residuals.push_back(newEdge->face() ? newEdge->face()->groupId() : -1);
                    residuals.push_back(oldInt.x() - mid.x());
                    residuals.push_back(oldInt.y() - mid.y());
                    residuals.push_back(oldInt.z() - mid.z());
                    ++applied;
                    did = true;
                    break;
                }
                if (!did) break;
            }
            // Keep encoder/decoder halfedge pairing identical after all collapses
            // of this round (per-op repair would be O(rounds*ops*mesh)).
            if (applied > 0) pmsf::repairHalfedgePairing(mesh);
            std::vector<unsigned char> bits(static_cast<size_t>(applied), 1);
            boundaryRoundBitCounts.push_back(static_cast<int>(bits.size()));
            boundaryRoundBits.insert(boundaryRoundBits.end(), bits.begin(), bits.end());
            boundaryRoundResiduals.insert(boundaryRoundResiduals.end(), residuals.begin(), residuals.end());
            if (std::getenv("PMSF_SEAM_DEBUG")) {
                std::cerr << "[seam-boundary] round=" << i << " applied=" << applied
                          << " V=" << mesh->size_of_vertices()
                          << " F=" << mesh->size_of_facets() << std::endl;
            }
        } else if (((std::getenv("PMSF_SEAM_BOUNDARY") && !mixedBoundary) || mixedBoundary) &&
                   std::getenv("PMSF_SEAM_COMPRESS")) {
            // Keep one boundary round slot per round so decoder round mapping stays aligned.
            boundaryRoundBitCounts.push_back(0);
        }

        // P1 Spike：统计 gate-queue pop 次数（= valence sym + null-patch sym 总和）
        //   enable via env PMSF_RD_PATCH_SPIKE=1。零字节影响（仅计数 + 末尾打印）。
        static const bool kRdPatchSpike = []() {
            const char* e = std::getenv("PMSF_RD_PATCH_SPIKE");
            return e && e[0] == '1';
        }();
        int roundSelectCalls = 0;
        int roundIsValid = 0;      // select 返回 true 但未成功 eliminate（= null-patch 候选）
        // Encoder-side guard: never remove a vertex whose fan mixes facet groups.
        // The decoder inherits one groupId from the split facet, so a mixed fan would
        // make its reconstruction groups differ from the encoder's original faces.
        const bool scFanGroupGuard = std::getenv("PMSF_SC_FAN_GROUP_GUARD") != nullptr;
        auto scFanGroupUniform = [](MCGAL::Vertex* v) -> bool {
            if (!v || v->isRemoved()) return false;
            int g = -1;
            bool any = false;
            for (MCGAL::Halfedge* h : v->halfedges()) {
                if (!h || h->isRemoved() || !h->face() || h->face()->isRemoved()) continue;
                const int fg = h->face()->groupId();
                if (!any) { g = fg; any = true; }
                else if (fg != g) return false;
            }
            return any;
        };
        const bool batchMIS = cfg.enableIndependentSet && envOn("PMSF_PCATM_BATCH_MIS");
        if (batchMIS) {
            struct BatchCandidate {
                BatchCandidate() : entry(nullptr) {}
                HalfedgeErrorEntry entry;
                MCGAL::Vertex* center = nullptr;
                int valence = 0;
                int liveDegree = 0;
                int conflictDegree = 0;
            };

            int maxBatchPasses = 1;
            if (const char* refillEnv = std::getenv("PMSF_PCATM_BATCH_REFILL_PASSES")) {
                maxBatchPasses = std::max(1, std::atoi(refillEnv));
            }

            auto runBatchPass = [&](int passIdx) {
                std::unordered_map<int, BatchCandidate> byVertex;
                int passSelectCalls = 0;
                int passRejected = 0;
                int passEliminated = 0;
                while (true) {
                    auto ts = now();
                    bool ok = select->select(entry);
                    t_selectLoop += now() - ts;
                    if (!ok) break;
                    ++roundSelectCalls;
                    ++passSelectCalls;
                    if (!entry.edge || !entry.edge->end_vertex()) {
                        ++roundIsValid;
                        ++passRejected;
                        continue;
                    }
                    if (scFanGroupGuard && !scFanGroupUniform(entry.edge->end_vertex())) {
                        ++passRejected;
                        continue;
                    }
                    MCGAL::Vertex* center = entry.edge->end_vertex();
                    const int vid = center->poolId();
                    BatchCandidate cand;
                    cand.entry = entry;
                    cand.center = center;
                    cand.valence = removedCenterValence(entry.edge);
                    cand.liveDegree = liveVertexDegree(center);
                    auto it = byVertex.find(vid);
                    if (it == byVertex.end() ||
                        std::tie(cand.valence, cand.liveDegree, cand.entry.error) <
                            std::tie(it->second.valence,
                                     it->second.liveDegree,
                                     it->second.entry.error)) {
                        byVertex[vid] = cand;
                    }
                }

                std::unordered_set<int> candidateIds;
                candidateIds.reserve(byVertex.size() * 2 + 16);
                for (const auto& kv : byVertex) candidateIds.insert(kv.first);

                std::vector<BatchCandidate> candidates;
                candidates.reserve(byVertex.size());
                for (auto& kv : byVertex) {
                    kv.second.conflictDegree =
                        candidateNeighborCount(kv.second.center, candidateIds);
                    candidates.push_back(kv.second);
                }
                const bool sortByConflict = envOn("PMSF_PCATM_BATCH_MIS_SORT_CONFLICT");
                std::sort(candidates.begin(), candidates.end(),
                          [sortByConflict](const BatchCandidate& a, const BatchCandidate& b) {
                              if (sortByConflict) {
                                  return std::make_tuple(a.conflictDegree,
                                                         a.valence,
                                                         a.liveDegree,
                                                         a.entry.error,
                                                         a.center->poolId()) <
                                         std::make_tuple(b.conflictDegree,
                                                         b.valence,
                                                         b.liveDegree,
                                                         b.entry.error,
                                                         b.center->poolId());
                              }
                              return std::make_tuple(a.valence,
                                                     a.liveDegree,
                                                     a.conflictDegree,
                                                     a.entry.error,
                                                     a.center->poolId()) <
                                     std::make_tuple(b.valence,
                                                     b.liveDegree,
                                                     b.conflictDegree,
                                                     b.entry.error,
                                                     b.center->poolId());
                          });

                std::vector<HalfedgeErrorEntry> chosen;
                chosen.reserve(candidates.size());
                for (const auto& cand : candidates) {
                    if (!misSelector.canRemove(cand.entry)) {
                        ++roundIsValid;
                        ++passRejected;
                        continue;
                    }
                    chosen.push_back(cand.entry);
                    misSelector.markRemovedAndNeighbors(cand.center);
                }

                for (auto& selectedEntry : chosen) {
                    // ---- 消除前**重新校验**（D4）----
                    //
                    // `chosen` 是在**同一批开始前**一次性筛出来的快照，但同批里
                    // 先消除的候选会改变网格：后一个候选的半边可能已被移除、
                    // 误差已失效、拓扑前提不再成立。按快照硬消会让编码端产出的
                    // 消除序列**在按序重放时不自洽** —— 解码端严格按符号顺序重放，
                    // 就会撞 `join_face` 的 `!gprev->isRemoved()` 断言
                    // （实测 PMSF_PCATM_BATCH_MIS=1 压得出、解不开）。
                    //
                    // 这里退回队列而不是硬消，编解码两侧的序列才一致。
                    {
                        HalfedgeErrorEntry recheck = selectedEntry;
                        if (recheck.edge == nullptr || recheck.edge->isRemoved() ||
                            recheck.edge->vertex() == nullptr || recheck.edge->vertex()->isRemoved() ||
                            recheck.edge->end_vertex() == nullptr || recheck.edge->end_vertex()->isRemoved()) {
                            ++passRejected;
                            continue;
                        }
                    }
                    auto te = now();
                    MCGAL::Vertex* cA1 = selectedEntry.edge ? selectedEntry.edge->vertex() : nullptr;
                    MCGAL::Vertex* cB1 = selectedEntry.edge ? selectedEntry.edge->end_vertex() : nullptr;
                    bool eok = eliminate->eliminate(selectedEntry);
                    t_eliminateLoop += now() - te;
                    if (eok) {
                        recordCollapse(cA1, cB1);  // P18.2 坍缩记录（第 1/3 处）
                        if (selectedEntry.edge != nullptr) perRoundGates.push_back(selectedEntry.edge);
                        ++roundEliminationCount;
                        ++passEliminated;
                        if (cfg.updateAfterEliminate) {
                            auto tu = now();
                            select->update(selectedEntry);
                            t_updateLoop += now() - tu;
                        }
                    } else {
                        ++roundIsValid;
                        ++passRejected;
                    }
                }

                if (const char* env = std::getenv("PMSF_BPV_DIAG"); env && env[0] == '1') {
                    std::fprintf(stderr,
                        "[PCATM_BATCH_MIS] r=%d pass=%d popped=%d unique=%zu chosen=%zu eliminated=%d rejected=%d\n",
                        i, passIdx, passSelectCalls, byVertex.size(), chosen.size(),
                        passEliminated, passRejected);
                }
                return passEliminated;
            };

            int passEliminated = runBatchPass(0);
            for (int pass = 1; pass < maxBatchPasses && passEliminated > 0; ++pass) {
                auto ts = now();
                select->init(mesh);
                t_selectLoop += now() - ts;
                passEliminated = runBatchPass(pass);
            }
        } else {
            while (true) {
                auto ts = now();
                bool ok = select->select(entry);
                t_selectLoop += now() - ts;
                if (!ok) break;
                if (kRdPatchSpike) { roundSelectCalls++; }
                if (scFanGroupGuard && entry.edge &&
                    !scFanGroupUniform(entry.edge->end_vertex())) {
                    continue;
                }
                if (cfg.enableIndependentSet) {
                    if (!misSelector.canRemove(entry)) {
                        if (kRdPatchSpike) roundIsValid++;  // 这里只算 IS 拒绝（和 null-patch 语义近）
                        continue;
                    }
                    MCGAL::Vertex* removedCandidate =
                        entry.edge ? entry.edge->end_vertex() : nullptr;
                    MCGAL::Vertex* cA2 = entry.edge ? entry.edge->vertex() : nullptr;
                    MCGAL::Vertex* cB2 = entry.edge ? entry.edge->end_vertex() : nullptr;
                    auto te = now();
                    bool eok = eliminate->eliminate(entry);
                    t_eliminateLoop += now() - te;
                    if (eok) {
                        recordCollapse(cA2, cB2);  // P18.2 坍缩记录（第 2/3 处）
                        misSelector.markRemoved(removedCandidate);
                        if (entry.edge != nullptr) perRoundGates.push_back(entry.edge);
                        roundEliminationCount++;
                        if (cfg.updateAfterEliminate) {
                            auto tu = now();
                            select->update(entry);
                            t_updateLoop += now() - tu;
                        }
                    } else if (kRdPatchSpike) {
                        roundIsValid++;  // eliminate 失败 → null-patch
                    }
                } else {
                    MCGAL::Vertex* cA3 = entry.edge ? entry.edge->vertex() : nullptr;
                    MCGAL::Vertex* cB3 = entry.edge ? entry.edge->end_vertex() : nullptr;
                    auto te = now();
                    bool eok = eliminate->eliminate(entry);
                    t_eliminateLoop += now() - te;
                    if (eok) {
                        recordCollapse(cA3, cB3);  // P18.2 坍缩记录（第 3/3 处）
                        if (entry.edge != nullptr) perRoundGates.push_back(entry.edge);
                        roundEliminationCount++;
                        if (cfg.updateAfterEliminate) {
                            auto tu = now();
                            select->update(entry);
                            t_updateLoop += now() - tu;
                        }
                    } else if (kRdPatchSpike) {
                        roundIsValid++;
                    }
                }
            }
        }
        if (kRdPatchSpike) {
            std::fprintf(stderr,
                "[RD_PATCH_SPIKE] r=%d  select_pop=%d  eliminated=%d  null_patch=%d  "
                "(null/pop=%.1f%%)\n",
                i, roundSelectCalls, roundEliminationCount, roundIsValid,
                roundSelectCalls > 0 ? 100.0 * roundIsValid / roundSelectCalls : 0.0);
        }

        // PPMC 风格 collector 不能处理空轮（i_lenGeom>0 assert）；提供 opt-in
        // 早停以避免崩溃，并把实际跑过的轮数同步到 header（解码端读 header.round
        // 即可正确终止 deserializeOneRound 循环）。其它算法保持 stopOnEmptyRound=false
        // 的历史行为（跑满 R 轮，包括空轮）以维持 SHA256 baseline。
        if (const char* rdbg = std::getenv("PMSF_ROUND_DBG"); rdbg && *rdbg && *rdbg != '0') {
            std::size_t liveV = 0, liveF = 0, patchF = 0, nonTriF = 0, maxDeg = 0;
            for (MCGAL::Vertex* v : mesh->vertices()) {
                if (v && !v->isRemoved()) ++liveV;
            }
            for (MCGAL::Facet* fc : mesh->faces()) {
                if (!fc || fc->isRemoved()) continue;
                ++liveF;
                const std::size_t d = fc->facet_degree();
                if (d > 3) ++nonTriF;
                if (d > maxDeg) maxDeg = d;
                if (fc->isSplittable()) ++patchF;
            }
            // 非流形边诊断：统计被 >2 个面共享的边（以及 Euler 特征）。
            std::size_t nonManifoldEdges = 0, uniqueEdges = 0;
            {
                std::unordered_map<std::uint64_t, std::uint32_t> edgeCount;
                for (MCGAL::Facet* fc : mesh->faces()) {
                    if (!fc || fc->isRemoved() || !fc->proxyHalfedge()) continue;
                    MCGAL::Halfedge* p0 = fc->proxyHalfedge();
                    MCGAL::Halfedge* p = p0;
                    int g2 = 0;
                    do {
                        if (p && p->vertex() && p->end_vertex()) {
                            const std::int64_t ua = p->vertex()->poolId();
                            const std::int64_t ub = p->end_vertex()->poolId();
                            const std::uint64_t lo = static_cast<std::uint64_t>(std::min(ua, ub));
                            const std::uint64_t hi = static_cast<std::uint64_t>(std::max(ua, ub));
                            const std::uint64_t key = (lo << 32) | hi;
                            ++edgeCount[key];
                        }
                        p = p ? p->next() : nullptr;
                    } while (p && p != p0 && ++g2 < 4096);
                }
                uniqueEdges = edgeCount.size();
                for (const auto& kv : edgeCount) {
                    if (kv.second > 2) ++nonManifoldEdges;
                }
            }
            std::fprintf(stderr,
                         "[ROUND_DBG] r=%d removed=%d liveV=%zu liveF=%zu patchF=%zu nonTriF=%zu maxDeg=%zu nonManifoldEdges=%zu euler=%lld\n",
                         i, roundEliminationCount, liveV, liveF, patchF, nonTriF, maxDeg,
                         nonManifoldEdges,
                         static_cast<long long>(static_cast<long long>(liveV) -
                                                static_cast<long long>(uniqueEdges) +
                                                static_cast<long long>(liveF)));
        }
        if (cfg.stopOnEmptyRound && roundEliminationCount == 0) {
            header.setRound(i);
            options.setRound(i);
            t_postprocess += now() - now();  // dummy 收尾
            break;
        }

        // postProcess 历史位置：select 阶段结束之后、collector->collect 之前。
        auto tpp = now();
        if (cfg.afterRound) {
            cfg.afterRound(i, mesh);
        } else if (cfg.postProcessPosition
                       == CompressPipelineConfig::PostProcessPosition::EveryRound
                   && postProcessOperator) {
            postProcessOperator->process(mesh);
        }
        t_postprocess += now() - tpp;
        auto tc = now();
        // P18.2 阶段 B：分区推进（外层 round 由本循环持有，内层 group 由
        // collector 的 collectRound 负责）。
        //
        // **默认路径逐字节不变**：seeds.size() <= 1（即未分区）时，
        // 下面走 `collect(seed, perRoundGates)` 原调用；即便走 collectRound，
        // 基类实现也只是退回 collect(seed)。分区 collector 覆写它。
        // 判据是「本管线配了分区阶段吗」，**不是** seeds.size() > 1：
        // N=1 的分区同样必须走真 seeds（见 SegmentedOptimized::collectRound
        // 的注释 —— 用主循环 seed 会 BFS 错起点，N=1 实测 BUG: rescaling）。
        // [PHASE59 诊断，默认关闭] 本轮的**活**顶点/面数（collect 时刻）。
        // facetSym 是「被 BFS 访问到」的面数，不等于活面数；回环定位需要后者。
        // [PHASE59 回环诊断，默认关闭] 把本轮 collect 时刻的**整网格活状态**写出
        // （顶点量化坐标 + 每个活面的顶点坐标集与 splittable 标记）。`PMSF_SEG_HE_DUMP`
        // 只写「被标记的子集」，无法区分「网格不同」与「标记不同」；本信道是前者。
        if (std::getenv("PMSF_BOUNDARY_COMPRESS") && boundaryGraph) {
            const auto candidates =
                pmsf::edgeflip::collectBoundaryEdgesSeedBfs(mesh, seed);
            std::vector<unsigned char> decisions(candidates.size(), 0);
            int removed = 0;
            for (size_t ci = 0; ci < candidates.size(); ++ci) {
                if (removed >= 1) break;  // conservative first step
                MCGAL::Halfedge* h = candidates[ci];
                if (!h || h->isRemoved() || !h->opposite()) continue;
                MCGAL::Vertex* v = h->end_vertex();
                if (!v || v->isRemoved()) continue;
                if (v->vertex_degree() < 3 || v->vertex_degree() > 7) continue;
                if (seed && seed->vertex() && seed->end_vertex() &&
                    (v == seed->vertex() || v == seed->end_vertex())) continue;
                HalfedgeErrorEntry entry(h);
                if (eliminate->eliminate(entry)) {
                    decisions[ci] = 1;
                    ++removed;
                }
            }
            boundaryRoundBitCounts.push_back(static_cast<int>(candidates.size()));
            boundaryRoundBits.insert(boundaryRoundBits.end(), decisions.begin(), decisions.end());
            if (std::getenv("PMSF_BOUNDARY_DEBUG")) {
                std::cerr << "[boundary-compress] cand=" << candidates.size()
                          << " removed=" << removed << std::endl;
            }
        }

        if (std::getenv("PMSF_BOUNDARY_REMOVE") && !std::getenv("PMSF_BOUNDARY_PRE_REMOVE") &&
            !std::getenv("PMSF_BOUNDARY_COMPRESS")) {
            const auto preEdges = pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(mesh);
            MCGAL::Vertex* chosen = nullptr;
            for (MCGAL::Halfedge* h : preEdges) {
                if (!h || h->isRemoved() || h->opposite() != nullptr) continue;
                MCGAL::Vertex* v = h->end_vertex();
                if (!v || v->isRemoved() || v->vertex_degree() != 3) continue;
                if (seed && seed->vertex() && seed->end_vertex() &&
                    (v == seed->vertex() || v == seed->end_vertex())) continue;
                bool touchesSeedFace = false;
                if (seed && seed->face()) {
                    for (MCGAL::Halfedge* sh : *seed->face()) {
                        if (sh && (sh->vertex() == v || sh->end_vertex() == v)) { touchesSeedFace = true; break; }
                    }
                }
                if (touchesSeedFace) continue;
                chosen = v;
                break;
            }
            MCGAL::Halfedge* newEdge = nullptr;
            MCGAL::PointInt removedInt(0, 0, 0);
            if (chosen) {
                removedInt = mesh->floatPosToInt(chosen->point());
                pmsf::collapseBoundaryVertex3Robust(mesh, chosen, &newEdge);
            }
            canonicalizeTraversalProxies(mesh);
            const auto postEdges = pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(mesh);
            std::vector<unsigned char> decisions(postEdges.size(), 0);
            std::vector<int> residuals;
            if (newEdge) {
                for (size_t ei = 0; ei < postEdges.size(); ++ei) {
                    if (postEdges[ei] == newEdge) { decisions[ei] = 1; break; }
                }
                residuals.push_back(removedInt.x());
                residuals.push_back(removedInt.y());
                residuals.push_back(removedInt.z());
            }
            boundaryRoundBitCounts.push_back(static_cast<int>(postEdges.size()));
            boundaryRoundBits.insert(boundaryRoundBits.end(), decisions.begin(), decisions.end());
            boundaryRoundResiduals.insert(boundaryRoundResiduals.end(), residuals.begin(), residuals.end());
            if (std::getenv("PMSF_BOUNDARY_DEBUG")) {
                std::cerr << "[boundary-remove-pre] preEdges=" << preEdges.size()
                          << " postEdges=" << postEdges.size()
                          << " chosen=" << (chosen ? 1 : 0)
                          << " newEdge=" << (newEdge ? 1 : 0) << std::endl;
            }
        }

        if (std::getenv("PMSF_BOUNDARY_ELIM")) {
            const auto preEdges = pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(mesh);
            MCGAL::Vertex* chosen = nullptr;
            MCGAL::Halfedge* chosenH = nullptr;
            for (MCGAL::Halfedge* h : preEdges) {
                if (!h || h->isRemoved() || h->opposite() != nullptr) continue;
                MCGAL::Vertex* v = h->end_vertex();
                if (!v || v->isRemoved() || v->vertex_degree() != 3) continue;
                if (seed && seed->vertex() && seed->end_vertex() &&
                    (v == seed->vertex() || v == seed->end_vertex())) continue;
                chosen = v; chosenH = h; break;
            }
            if (chosenH) {
                HalfedgeErrorEntry entry(chosenH);
                const bool ok = eliminate->eliminate(entry);
                if (std::getenv("PMSF_BOUNDARY_DEBUG"))
                    std::cerr << "[boundary-elim] chosen=" << (chosen?1:0) << " ok=" << (ok?1:0) << std::endl;
            }
        }



        if (sharedPatchAfterInterior && patchPerRound && patchRoundSelected &&
            (patchOnlyRound < 0 || patchOnlyRound == i))
            applySharedBoundaryPatch(i);
        if (!boundaryStageBeforeInterior) {
            const int bRemoved = runBoundaryStage(i);
            if (bRemoved > 0 && std::getenv("PMSF_BOUNDARY_DEBUG"))
                std::fprintf(stderr, "[boundary-stage] round=%d after removed=%d\n", i, bRemoved);
        }
        if (std::getenv("PMSF_SC_CANON_FACES")) pmsf::scCanonicalFaceSort(mesh.get());
        if (cfg.segmentOperatorFactory) {
            if (std::getenv("PMSF_SEG_REFRESH_SEEDS") && !segSeedPairs.empty()) {
                for (std::size_t gi = 0; gi < seeds.size() && gi < segSeedPairs.size(); ++gi) {
                    MCGAL::Vertex* va = segSeedPairs[gi].first;
                    MCGAL::Vertex* vb = segSeedPairs[gi].second;
                    if (!va || !vb) continue;
                    if (MCGAL::Halfedge* live = findLiveSeedBetweenLocal(mesh.get(), va, vb,
                                                                        static_cast<int>(gi)))
                        seeds[gi] = live;
                }
                if (!seeds.empty() && seeds[0]) seed = seeds[0];
            }
            if (std::getenv("PMSF_SEG_CANONICAL_SEEDS")) {
                for (std::size_t gi = 0; gi < seeds.size(); ++gi) {
                    MCGAL::Halfedge* canonical =
                        findCanonicalGroupSeedLocal(mesh.get(), static_cast<int>(gi));
                    if (canonical) seeds[gi] = canonical;
                }
                if (!seeds.empty() && seeds[0]) seed = seeds[0];
            }
            std::vector<std::uint8_t> activeMask(seeds.size(), 1);
            if (i < static_cast<int>(g_scRoundSeedVerts.size()) && seed && seed->vertex() &&
                seed->end_vertex() && !seed->isRemoved() && !seed->vertex()->isRemoved() &&
                !seed->end_vertex()->isRemoved()) {
                g_scRoundSeedVerts[static_cast<std::size_t>(i)] = {seed->vertex(),
                                                                   seed->end_vertex()};
            }
            if (std::getenv("PMSF_BND_DECOUPLED") && mesh) {
                // Boundary-scope lock: freeze the group-boundary vertices and their
                // one-ring from interior elimination, so boundary-stage ops and interior
                // ops act on disjoint regions (commute).  Rebuild every round.
                static thread_local std::unordered_set<MCGAL::Vertex*> g_decoupledGuarded;
                auto& guard = pmsf::anchorGuardedVertices();
                for (MCGAL::Vertex* gv : g_decoupledGuarded) guard.erase(gv);
                g_decoupledGuarded.clear();
                std::unordered_set<MCGAL::Vertex*> bverts;
                for (MCGAL::Facet* f : mesh->faces()) {
                    if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                    for (MCGAL::Halfedge* h : *f) {
                        if (!h || h->isRemoved()) continue;
                        MCGAL::Halfedge* opp = h->opposite();
                        const bool isBoundary =
                            !opp || opp->isRemoved() || !opp->face() ||
                            opp->face()->groupId() != f->groupId();
                        if (!isBoundary) continue;
                        if (h->vertex() && !h->vertex()->isRemoved())
                            bverts.insert(h->vertex());
                        if (h->end_vertex() && !h->end_vertex()->isRemoved())
                            bverts.insert(h->end_vertex());
                    }
                }
                std::unordered_set<MCGAL::Vertex*> region;
                for (MCGAL::Vertex* v : bverts) {
                    if (!v || v->isRemoved()) continue;
                    region.insert(v);
                    for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
                        if (h && h->end_vertex() && !h->end_vertex()->isRemoved())
                            region.insert(h->end_vertex());
                    }
                }
                for (MCGAL::Vertex* v : region) {
                    guard.insert(v);
                    g_decoupledGuarded.insert(v);
                }
            }
            collector->collectRound(seed, seeds, activeMask, i);
        } else {
            const auto tcan0 = std::chrono::steady_clock::now();
            if (std::getenv("PMSF_PPMC_SKIP_CANON") == nullptr) {
                canonicalizeTraversalProxies(mesh);
            }
            if (const char* tp = std::getenv("PMSF_PPMC_TPROF"); tp && *tp && *tp != '0') {
                std::fprintf(stderr, "[CANON_TPROF] round=%d ms=%.1f\n", i,
                             std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - tcan0).count());
            }
            collector->collect(seed, perRoundGates);  // 新 overload：默认回退到 collect(seed)
        }
        if (probeCollector) probeCollector->collect(seed, perRoundGates);
        t_collectLoop += now() - tc;


        if (std::getenv("PMSF_BOUNDARY_PATCH_ROUND")) {
            const auto preEdges = pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(mesh);
            MCGAL::Vertex* chosen = nullptr;
            for (MCGAL::Halfedge* h : preEdges) {
                if (!h || h->isRemoved() || h->opposite() != nullptr) continue;
                MCGAL::Vertex* v = h->end_vertex();
                if (!v || v->isRemoved() || v->vertex_degree() != 3) continue;
                if (seed && seed->vertex() && seed->end_vertex() &&
                    (v == seed->vertex() || v == seed->end_vertex())) continue;
                chosen = v; break;
            }
            pmsf::BoundaryCollapsePatch patch;
            MCGAL::Halfedge* newEdge = nullptr;
            std::vector<int> residuals;
            if (chosen && pmsf::captureBoundaryCollapse3(mesh, chosen, patch)) {
                pmsf::collapseBoundaryVertex3Robust(mesh, chosen, &newEdge);
            }
            canonicalizeTraversalProxies(mesh);
            const auto postEdges = pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(mesh);
            std::vector<unsigned char> bits(postEdges.size(), 0);
            if (newEdge) {
                for (size_t ei = 0; ei < postEdges.size(); ++ei) {
                    if (postEdges[ei] == newEdge) { bits[ei] = 1; break; }
                }
                auto pushPoint = [&](const MCGAL::Point& pt) {
                    const auto q = pmsf::patchInt(mesh, pt);
                    residuals.push_back(q[0]); residuals.push_back(q[1]); residuals.push_back(q[2]);
                };
                pushPoint(patch.removed);
                pushPoint(patch.a); pushPoint(patch.b); pushPoint(patch.c);
                pushPoint(patch.f0ProxyA); pushPoint(patch.f0ProxyB);
                pushPoint(patch.f1ProxyA); pushPoint(patch.f1ProxyB);
                residuals.push_back(patch.g0); residuals.push_back(patch.g1);
            }
            boundaryRoundBitCounts.push_back(static_cast<int>(bits.size()));
            boundaryRoundBits.insert(boundaryRoundBits.end(), bits.begin(), bits.end());
            boundaryRoundResiduals.insert(boundaryRoundResiduals.end(), residuals.begin(), residuals.end());
            if (std::getenv("PMSF_BOUNDARY_DEBUG"))
                std::cerr << "[boundary-patch-round] pre=" << preEdges.size()
                          << " post=" << postEdges.size()
                          << " chosen=" << (chosen ? 1 : 0)
                          << " newEdge=" << (newEdge ? 1 : 0) << std::endl;
        }

        if (dumpPipelineRounds) {
            std::filesystem::create_directories("build/aaaaresult/compress/rounds");
            char roundPath[512];
            std::snprintf(roundPath, sizeof(roundPath),
                          "build/aaaaresult/compress/rounds/round_%03d_preflip.off", i);
            mesh->dumpto_oldtype(roundPath);
        }

        if (bcV2Enabled && scRoundOrderFor(i) == 2) runV2StageNow(i);
        {
            const int roFlip = scRoundOrderFor(i);
            const bool flipEarly = (roFlip == 3 || roFlip == 4 || roFlip == 5);
            if (!flipEarly) runFlipPass(i);
        }
        if (std::getenv("PMSF_SEAM_BOUNDARY_ROUND_END") &&
            std::getenv("PMSF_SEAM_BOUNDARY_ORDERED") &&
            std::getenv("PMSF_SEAM_COMPRESS")) {
            runOrderedBoundaryAtEnd(i);
        }
        if (std::getenv("PMSF_SEAM_BOUNDARY_ROUND_END") &&
            std::getenv("PMSF_SEAM_BOUNDARY_ORDER_BITS") &&
            std::getenv("PMSF_SEAM_COMPRESS")) {
            runOrderBitsBoundaryAtEnd(i);
        }

        // Boundary compression as the last stage of the round (after interior
        // and flip).  Decoder restores it before undoing the round.
        {
            bool runSplitThisRound = std::getenv("PMSF_GROUP_BOUNDARY_SPLIT") != nullptr;
            if (const char* onlyRound = std::getenv("PMSF_GROUP_BOUNDARY_SPLIT_ROUND"))
                runSplitThisRound = runSplitThisRound && i == std::atoi(onlyRound);
            if (runSplitThisRound) {
                const int gsRemoved = runGroupBoundarySplitStage(i);
                if (gsRemoved > 0 && std::getenv("PMSF_BOUNDARY_DEBUG"))
                    std::fprintf(stderr, "[group-boundary-split] round=%d removed=%d\n", i,
                                 gsRemoved);
            }
        }

        // Single-copy group-boundary stage (interim positional-patch prototype).
        // Runs after interior + flip (decoder restores it before undoing the round).
        const bool scAutoStage = std::getenv("PMSF_SC_BOUNDARY_AUTO") != nullptr;
        bool runScBoundaryStage = std::getenv("PMSF_SC_BOUNDARY_STAGE") != nullptr ||
                                  std::getenv("PMSF_SC_BOUNDARY_COMPACT_STAGE") != nullptr ||
                                  std::getenv("PMSF_SC_BOUNDARY_ANCHOR_STAGE") != nullptr ||
                                  std::getenv("PMSF_SC_BOUNDARY_GROUP_STAGE") != nullptr ||
                                  scAutoStage;
        if (runScBoundaryStage) {
            if (const char* onlyRounds = std::getenv("PMSF_SC_BOUNDARY_ROUNDS")) {
                runScBoundaryStage = false;
                const std::string rs(onlyRounds);
                std::size_t p = 0;
                while (p <= rs.size()) {
                    const std::size_t comma = rs.find(',', p);
                    const std::string tok = rs.substr(
                        p, comma == std::string::npos ? std::string::npos : comma - p);
                    if (!tok.empty() && std::atoi(tok.c_str()) == i) { runScBoundaryStage = true; break; }
                    if (comma == std::string::npos) break;
                    p = comma + 1;
                }
            }
            if (scAutoStage && std::getenv("PMSF_SC_BOUNDARY_ROUNDS") == nullptr &&
                i != options.getRound() - 1)
                runScBoundaryStage = false;
            if (runScBoundaryStage && scAutoStage &&
                std::getenv("PMSF_SC_REQUIRE_VALIDATED_SEGMENTS") != nullptr) {
                // Optional strict mode: only run AUTO on the originally validated segment
                // counts.  By default AUTO relies on the preflight trial + post-stage
                // verification gate, which are configuration-aware and safe for other
                // segment counts too (verified on segments 8/16).
                const int autoSegments = options.getSegmentationOption().getNumSegments();
                if (autoSegments != 4 && autoSegments != 32 && autoSegments != 1) {
                    if (std::getenv("PMSF_SC_DEBUG") || std::getenv("PMSF_BOUNDARY_DEBUG"))
                        std::fprintf(stderr,
                                     "[sc-auto-guard] segments=%d skipped "
                                     "(PMSF_SC_REQUIRE_VALIDATED_SEGMENTS set)\n",
                                     autoSegments);
                    runScBoundaryStage = false;
                }
            }
            // Multi-round boundary staging is still experimental: mid-stream stages can
            // corrupt later-round reconstruction (facet order / group replay).  Refuse
            // silently-corrupt artifacts unless the caller explicitly opts in.
            if (runScBoundaryStage && i != options.getRound() - 1 &&
                std::getenv("PMSF_SC_ALLOW_MULTIROUND") == nullptr) {
                if (std::getenv("PMSF_BOUNDARY_DEBUG") || std::getenv("PMSF_SC_DEBUG"))
                    std::fprintf(stderr,
                                 "[sc-multi-round-guard] round=%d skipped (set "
                                 "PMSF_SC_ALLOW_MULTIROUND=1 to enable experimental multi-round "
                                 "boundary staging)\n",
                                 i);
                runScBoundaryStage = false;
            }
            if (runScBoundaryStage && scAutoStage) {
                // AUTO guards:
                // - tiny coarse bases: unstable tiny boundary loops;
                // - huge coarse bases (very few rounds): the per-op fan retriangulation
                //   is O(F) per op, so a large stage would cost minutes; skip instead of
                //   burning time (AXts stay neutral, not corrupt).
                std::size_t autoLiveV = 0;
                for (MCGAL::Vertex* av : mesh->vertices())
                    if (av && !av->isRemoved()) ++autoLiveV;
                if (autoLiveV < 1000) runScBoundaryStage = false;
                if (autoLiveV > 20000) runScBoundaryStage = false;
            }
        }
        if (bcV2Enabled) {
            const int roStageLate = scRoundOrderFor(i);
            if (roStageLate == 0 || roStageLate == 1 || roStageLate == 4) runV2StageNow(i);
        }

        if (std::getenv("PMSF_BC_V2_DEBUG") && bcV2Enabled && mesh) {
            {
                char gp[512];
                std::snprintf(gp, sizeof(gp), "/tmp/bc_v2_enc_round_%03d.gids", i);
                if (FILE* gf = std::fopen(gp, "w")) {
                    const int mq = (mesh->i_nbQuantBits > 0) ? ((1 << mesh->i_nbQuantBits) - 1)
                                                             : (1 << 30);
                    for (MCGAL::Facet* ff : mesh->faces()) {
                        if (!ff || ff->isRemoved() || !ff->proxyHalfedge()) continue;
                        const auto fv = pmsf::sbdetail::faceVertices(ff);
                        if (!fv[0]) continue;
                        std::vector<std::array<int, 3>> qs;
                        for (int k = 0; k < 3; ++k) {
                            const auto q = mesh->floatPosToInt(fv[k]->point());
                            qs.push_back({std::clamp(q.x(), 0, mq), std::clamp(q.y(), 0, mq),
                                          std::clamp(q.z(), 0, mq)});
                        }
                        std::sort(qs.begin(), qs.end());
                        std::fprintf(gf, "%d %d %d %d %d %d %d %d %d %d\n", ff->groupId(),
                                     qs[0][0], qs[0][1], qs[0][2], qs[1][0], qs[1][1],
                                     qs[1][2], qs[2][0], qs[2][1], qs[2][2]);
                    }
                    std::fclose(gf);
                }
            }
            {
                char op[512];
                std::snprintf(op, sizeof(op), "/tmp/bc_v2_enc_end_%03d.off", i);
                mesh->dumpto_oldtype(op);
                std::snprintf(op, sizeof(op), "/tmp/bc_v2_enc_end_%03d.gids", i);
                if (FILE* gf = std::fopen(op, "w")) {
                    const int mq = (mesh->i_nbQuantBits > 0) ? ((1 << mesh->i_nbQuantBits) - 1)
                                                             : (1 << 30);
                    for (MCGAL::Facet* ff : mesh->faces()) {
                        if (!ff || ff->isRemoved() || !ff->proxyHalfedge()) continue;
                        const auto fv = pmsf::sbdetail::faceVertices(ff);
                        if (!fv[0]) continue;
                        std::vector<std::array<int, 3>> qs;
                        for (int k = 0; k < 3; ++k) {
                            const auto q = mesh->floatPosToInt(fv[k]->point());
                            qs.push_back({std::clamp(q.x(), 0, mq), std::clamp(q.y(), 0, mq),
                                          std::clamp(q.z(), 0, mq)});
                        }
                        std::sort(qs.begin(), qs.end());
                        std::fprintf(gf, "%d %d %d %d %d %d %d %d %d %d\n", ff->groupId(),
                                     qs[0][0], qs[0][1], qs[0][2], qs[1][0], qs[1][1],
                                     qs[1][2], qs[2][0], qs[2][1], qs[2][2]);
                    }
                    std::fclose(gf);
                }
            }
            std::size_t dbgLiveV = 0, dbgLiveF = 0;
            for (MCGAL::Vertex* v : mesh->vertices())
                if (v && !v->isRemoved()) ++dbgLiveV;
            for (MCGAL::Facet* f : mesh->faces())
                if (f && !f->isRemoved()) ++dbgLiveF;
            std::fprintf(stderr, "[bc-v2-roundend] round=%d V=%zu F=%zu", i, dbgLiveV, dbgLiveF);
            for (const auto& ag : g_bcV2Anchors) {
                std::vector<MCGAL::Halfedge*> dbgLoop;
                pmsf::walkLoopFromFixedAnchor(mesh.get(), ag.second, ag.first, dbgLoop);
                std::fprintf(stderr, " g%d=%zu", ag.first, dbgLoop.size());
            }
            std::fprintf(stderr, "\n");
        }
        if (std::getenv("PMSF_SC_DEBUG")) {
            std::size_t liveV = 0, liveF = 0;
            for (MCGAL::Vertex* v : mesh->vertices())
                if (v && !v->isRemoved()) ++liveV;
            for (MCGAL::Facet* f : mesh->faces())
                if (f && !f->isRemoved()) ++liveF;
            std::fprintf(stderr, "[enc-round-state] round=%d liveV=%zu liveF=%zu\n", i, liveV,
                         liveF);
        }
        if (std::getenv("PMSF_BC_ANCHOR_DIAG") && mesh) {
            static thread_local std::vector<std::pair<int, MCGAL::Vertex*>> diagAnchors;
            static thread_local bool diagAnchorsReady = false;
            if (i == 0) {
                diagAnchors = pmsf::collectFixedLoopAnchors(mesh.get());
                g_bcFixedAnchors = diagAnchors;
                diagAnchorsReady = !diagAnchors.empty();
                // Diagnostic anchor persistence: freeze the chosen vertices so the
                // forward simplification keeps them alive.
                for (auto& ag : diagAnchors) {
                    if (!ag.second || ag.second->isRemoved()) continue;
                    ag.second->setAnchor();
                    pmsf::sharedBoundaryFrozenVertices().insert(ag.second);
                }
                const char* af = std::getenv("PMSF_BC_ANCHOR_FILE");
                const char* apath = (af && af[0]) ? af : "/tmp/bc_anchors_q.txt";
                if (FILE* f = std::fopen(apath, "w")) {
                    for (auto& ag : diagAnchors) {
                        const auto q = mesh->floatPosToInt(ag.second->point());
                        std::fprintf(f, "%d %d %d %d\n", ag.first, q.x(), q.y(), q.z());
                    }
                    std::fclose(f);
                }
            }
            if (diagAnchorsReady) {
                pmsf::dumpFixedAnchorLoopSequences(mesh.get(), diagAnchors, "enc", i);
            }
            {
                char ecp[512];
                std::snprintf(ecp, sizeof(ecp), "/tmp/bc_enc_round_%03d.off", i);
                mesh->dumpto_oldtype(ecp);
            }
        }
        if (const char* epf = std::getenv("PMSF_SC_ENC_POSTFLAGS"); epf) {
            if (FILE* ff = std::fopen(epf, i == 0 ? "w" : "a")) {
                std::fprintf(ff, "ROUND %d\n", i);
                for (MCGAL::Facet* ffac : mesh->faces()) {
                    if (!ffac || ffac->isRemoved() || !ffac->proxyHalfedge()) continue;
                    const auto vs = pmsf::sbdetail::faceVertices(ffac);
                    if (!vs[0]) continue;
                    std::vector<std::array<int, 3>> qs;
                    for (int k = 0; k < 3; ++k) {
                        const auto q = mesh->floatPosToInt(vs[k]->point());
                        { const int mq = (mesh->i_nbQuantBits > 0) ? ((1 << mesh->i_nbQuantBits) - 1) : (1 << 30); qs.push_back({std::clamp(q.x(), 0, mq), std::clamp(q.y(), 0, mq), std::clamp(q.z(), 0, mq)}); }
                    }
                    std::sort(qs.begin(), qs.end());
                    int amask = 0;
                    {
                        const auto fvs = pmsf::sbdetail::faceVertices(ffac);
                        for (int k = 0; k < 3; ++k) {
                            MCGAL::Vertex* fv = fvs[k];
                            if (!fv) continue;
                            MCGAL::PointInt fq = mesh->floatPosToInt(fv->point());
                            std::array<int, 3> key = {fq.x(), fq.y(), fq.z()};
                            int pos = -1;
                            for (int pp = 0; pp < 3; ++pp)
                                if (qs[static_cast<std::size_t>(pp)] == key) { pos = pp; break; }
                            if (pos < 0) continue;
                            for (MCGAL::Halfedge* hh : {ffac->proxyHalfedge(), ffac->proxyHalfedge()->next(),
                                                       ffac->proxyHalfedge()->next()->next()}) {
                                if (hh && hh->vertex() == fv) {
                                    if (hh->isAdded()) amask |= (1 << pos);
                                    break;
                                }
                            }
                        }
                    }
                    std::fprintf(ff, "%d %d %d %d %d %d %d %d %d %d %d %d %d\n", ffac->groupId(),
                                 ffac->isSplittable() ? 1 : 0, ffac->isUnsplittable() ? 1 : 0,
                                 qs[0][0], qs[0][1], qs[0][2], qs[1][0], qs[1][1], qs[1][2],
                                 qs[2][0], qs[2][1], qs[2][2], amask);
                }
                std::fclose(ff);
            }
        }
        auto tr = now();
        if (cfg.reseedEachRound) {
            select->reseedLive();
        }
        select->reset();
        t_selectReset += now() - tr;

        if (std::getenv("PMSF_SEAM_CANON_ROUNDS") && std::getenv("PMSF_SEAM_COMPRESS"))
            pmsf::canonicalizeMeshOrder(mesh);
        if (const char* gd = std::getenv("PMSF_SC_ENC_ROUND_GID_DIR")) {
            char gpath[512];
            std::snprintf(gpath, sizeof(gpath), "%s/enc_gid_%03d.txt", gd, i);
            if (FILE* gf = std::fopen(gpath, "w")) {
                for (MCGAL::Facet* ff : mesh->faces()) {
                    if (!ff || ff->isRemoved() || !ff->proxyHalfedge()) continue;
                    auto vv = pmsf::sbdetail::faceVertices(ff);
                    if (!vv[0]) continue;
                    std::vector<std::array<int, 3>> qs;
                    for (int k = 0; k < 3; ++k) {
                        const auto q = mesh->floatPosToInt(vv[k]->point());
                        { const int mq = (mesh->i_nbQuantBits > 0) ? ((1 << mesh->i_nbQuantBits) - 1) : (1 << 30); qs.push_back({std::clamp(q.x(), 0, mq), std::clamp(q.y(), 0, mq), std::clamp(q.z(), 0, mq)}); }
                    }
                    std::sort(qs.begin(), qs.end());
                    std::fprintf(gf, "%d %d %d %d %d %d %d %d %d %d\n", ff->groupId(),
                                 qs[0][0], qs[0][1], qs[0][2], qs[1][0], qs[1][1], qs[1][2],
                                 qs[2][0], qs[2][1], qs[2][2]);
                }
                for (MCGAL::Vertex* vv : mesh->vertices()) {
                    if (!vv || vv->isRemoved()) continue;
                    const auto q = mesh->floatPosToInt(vv->point());
                    std::fprintf(gf, "V %d %d %d %d\n", vv->groupId(), q.x(), q.y(), q.z());
                }
                std::fclose(gf);
            }
        }
        if (std::getenv("PMSF_ROUND_DEDUP_FACES")) {
            const int dded = pmsf::dedupIdenticalFaces(mesh);
            if (dded > 0 && std::getenv("PMSF_BOUNDARY_DEBUG"))
                std::fprintf(stderr, "[round-dedup] round=%d removed=%d\n", i, dded);
        }
    }

    // ---------- 13.5) conservative terminal boundary stage ----------
    //
    // Conservative boundary compression (L0): boundary simplification happens ONCE
    // on a clean trial clone of the terminal/base mesh, AFTER all progressive rounds.
    // The decoder just loads the resulting base mesh — there is NO inverse op.  The
    // trial clone has fully reset element flags and is fully validated before being
    // accepted; if the result is not a valid closed manifold delta (boundary-edge
    // count changed), the clone is discarded and the original base is kept (same
    // format, no mode switch).  The writer later verifies the complete artifact.
    if (std::getenv("PMSF_BND_BASE_STAGE") != nullptr && mesh) {
        pmsf::resetRoundElementState(mesh);
        canonicalizeTraversalProxies(mesh);
        MCGAL::Vertex* termSeedV0 = (seed && seed->vertex()) ? seed->vertex() : nullptr;
        MCGAL::Vertex* termSeedV1 = (seed && seed->end_vertex()) ? seed->end_vertex() : nullptr;
        std::unordered_map<int, MCGAL::Vertex*> trialVmap;
        int trialPoolId = -1;
        auto trial = cloneMeshForTrialFreshPool(mesh, &trialVmap, &trialPoolId);
        if (trial) {
            pmsf::resetRoundElementState(trial);
            canonicalizeTraversalProxies(trial);
            auto mapV = [&](MCGAL::Vertex* v) -> MCGAL::Vertex* {
                if (!v) return nullptr;
                auto it = trialVmap.find(v->poolId());
                return it == trialVmap.end() ? nullptr : it->second;
            };
            auto countMeshBoundaryEdges = [&](const std::shared_ptr<MCGAL::Mesh>& m) {
                std::size_t count = 0;
                for (MCGAL::Facet* f : m->faces()) {
                    if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                    for (MCGAL::Halfedge* h : *f) {
                        if (!h || h->isRemoved()) continue;
                        MCGAL::Halfedge* opp = h->opposite();
                        if (!opp || opp->isRemoved()) ++count;
                    }
                }
                return count;
            };
            std::vector<std::pair<int, MCGAL::Vertex*>> anchors;
            for (const auto& ag : g_bcV2Anchors) {
                MCGAL::Vertex* tv = mapV(ag.second);
                if (tv && !tv->isRemoved()) anchors.emplace_back(ag.first, tv);
            }
            if (anchors.empty()) anchors = pmsf::collectFixedLoopAnchors(trial.get());
            MCGAL::Vertex* trialSeedV0 = mapV(termSeedV0);
            MCGAL::Vertex* trialSeedV1 = mapV(termSeedV1);
            if (!anchors.empty()) {
                std::set<MCGAL::Vertex*> protectedVerts;
                for (auto& ag : anchors)
                    if (ag.second) protectedVerts.insert(ag.second);
                if (trialSeedV0) protectedVerts.insert(trialSeedV0);
                if (trialSeedV1) protectedVerts.insert(trialSeedV1);
                int maxOps = 256;
                if (const char* mo = std::getenv("PMSF_BND_BASE_MAXOPS")) maxOps = std::atoi(mo);
                const std::size_t boundaryBefore = countMeshBoundaryEdges(trial);
                auto terminalTopologyOk = [&](const std::shared_ptr<MCGAL::Mesh>& m,
                                              std::size_t expectedBoundary) {
                    std::map<std::pair<MCGAL::Vertex*, MCGAL::Vertex*>, int> edgeCount;
                    std::set<std::array<MCGAL::Vertex*, 3>> triSet;
                    for (MCGAL::Facet* f : m->faces()) {
                        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                        MCGAL::Vertex* fv[3];
                        int n = 0;
                        MCGAL::Halfedge* h0 = f->proxyHalfedge();
                        MCGAL::Halfedge* h = h0;
                        int guard = 0;
                        do {
                            if (h && h->vertex() && n < 3) fv[n++] = h->vertex();
                            h = h ? h->next() : nullptr;
                        } while (h && h != h0 && ++guard < 64);
                        if (n != 3) return false;
                        std::array<MCGAL::Vertex*, 3> tri{fv[0], fv[1], fv[2]};
                        std::sort(tri.begin(), tri.end());
                        if (!triSet.insert(tri).second) return false;
                        for (int k = 0; k < 3; ++k) {
                            MCGAL::Vertex* a = fv[k];
                            MCGAL::Vertex* b = fv[(k + 1) % 3];
                            if (!a || !b || a == b || a->isRemoved() || b->isRemoved())
                                return false;
                            if (a > b) std::swap(a, b);
                            ++edgeCount[{a, b}];
                        }
                    }
                    std::size_t boundaryEdges = 0;
                    for (const auto& kv : edgeCount) {
                        if (kv.second > 2) return false;
                        if (kv.second == 1) ++boundaryEdges;
                    }
                    return boundaryEdges == expectedBoundary;
                };
                const bool preflightOk = pmsf::scV2PreflightStage(
                    trial, options.getRound(), maxOps, protectedVerts, anchors);
                int applied = 0;
                bool valid = false;
                if (preflightOk) {
                    pmsf::BoundaryV2Round terminalRound;
                    applied = pmsf::runSingleCopyBoundaryStageV2(
                        trial, options.getRound(), maxOps, protectedVerts, anchors, terminalRound);
                    if (applied < 0) {
                        if (pmsf::scV2RollbackRound(trial, terminalRound)) applied = 0;
                        else applied = -1;
                    }
                    valid = (applied >= 0) && terminalTopologyOk(trial, boundaryBefore);
                }
                if (valid && applied > 0) {
                    mesh = trial;
                    MCGAL::Halfedge* newSeed = nullptr;
                    if (trialSeedV0 && trialSeedV1) {
                        newSeed = trialSeedV0->find_halfedge(trialSeedV1);
                        if (newSeed && newSeed->vertex() != trialSeedV0 && newSeed->opposite())
                            newSeed = newSeed->opposite();
                    }
                    if (!newSeed) {
                        for (MCGAL::Facet* f : mesh->faces()) {
                            if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                            newSeed = f->proxyHalfedge();
                            break;
                        }
                    }
                    if (newSeed) seed = newSeed;
                    std::size_t liveV = 0, liveF = 0;
                    for (MCGAL::Vertex* v : mesh->vertices())
                        if (v && !v->isRemoved()) ++liveV;
                    for (MCGAL::Facet* f : mesh->faces())
                        if (f && !f->isRemoved()) ++liveF;
                    std::fprintf(stderr,
                                 "[bnd-base-stage] ACCEPT pool=%d anchors=%zu applied=%d boundaryEdges=%zu liveV=%zu liveF=%zu\n",
                                 trialPoolId, anchors.size(), applied, boundaryBefore, liveV, liveF);
                } else {
                    std::fprintf(stderr,
                                 "[bnd-base-stage] REJECT anchors=%zu preflight=%d applied=%d (base unchanged)\n",
                                 anchors.size(), preflightOk ? 1 : 0, applied);
                }
            } else {
                std::fprintf(stderr, "[bnd-base-stage] no anchors -> base unchanged\n");
            }
        }
    }

    // ---------- 14) beforeSerialize（POMAR V2 的 LOD partition 等） ----------
    t0 = now();
    if (cfg.beforeSerialize) {
        cfg.beforeSerialize(collector, serializer, mesh, seed);
    }

    if (std::getenv("PMSF_BC_ANCHOR_DIAG") && mesh) {
        const auto hs = pmsf::groupBoundaryEnumHashesAllLoops(mesh.get());
        for (const auto& gh : hs)
            std::fprintf(stderr, "[bc-diag-final-enc] g=%d hash=%llx\n", gh.first, gh.second);
        mesh->dumpto_oldtype("/tmp/bc_enc_final.off");
    }
    if (std::getenv("PMSF_EDGEFLIP_POST_ONLY") &&
        options.isEnableEdgeFlipPostProcess()) {
        if (!preFlipBaseForSerialize) {
            preFlipBaseForSerialize = cloneMeshForPipelinePreflip(mesh);
        }
        EdgeFlipPostProcessOperator::Config efCfg;
        efCfg.rounds = 1;
        efCfg.longEdgeRatio = options.getEdgeFlipPostProcessLongEdgeRatio();
        efCfg.minImprovementRatio = options.getEdgeFlipPostProcessMinImprovementRatio();
        if (const char* v = std::getenv("PMSF_EDGEFLIP_LONG_RATIO")) efCfg.longEdgeRatio = std::atof(v);
        if (const char* v = std::getenv("PMSF_EDGEFLIP_MIN_IMPROVEMENT")) efCfg.minImprovementRatio = std::atof(v);
        efCfg.maxFlipsPerRoundPass = envInt("PMSF_EDGEFLIP_MAX_PER_ROUND", -1);
        EdgeFlipPostProcessOperator edgeFlipOp(efCfg);
        edgeFlipOp.process(mesh);
        canonicalizeTraversalProxies(mesh);
        const auto& postCounts = edgeFlipOp.roundCandidateBitCounts();
        edgeFlipRoundBitCounts.insert(edgeFlipRoundBitCounts.end(),
                                      postCounts.begin(), postCounts.end());
        const auto& postBits = edgeFlipOp.roundCandidateBits();
        edgeFlipRoundBits.insert(edgeFlipRoundBits.end(), postBits.begin(), postBits.end());
        const auto& postFlips = edgeFlipOp.roundFlipCounts();
        edgeFlipRoundCounts.push_back(postFlips.empty() ? 0 : postFlips.front());
    }
    if (std::getenv("PMSF_EDGEFLIP_CLEAN_BASE") && !preFlipBaseForSerialize) {
        auto cleanBase = cloneMeshForPipelinePreflip(mesh);
        if (cleanBase) {
            if (seed && seed->vertex() && seed->end_vertex()) {
                auto samePoint = [](const MCGAL::Point& a, const MCGAL::Point& b) {
                    const double dx = static_cast<double>(a.x()) - b.x();
                    const double dy = static_cast<double>(a.y()) - b.y();
                    const double dz = static_cast<double>(a.z()) - b.z();
                    return dx*dx + dy*dy + dz*dz < 1e-10;
                };
                MCGAL::Vertex* nv0 = nullptr;
                MCGAL::Vertex* nv1 = nullptr;
                for (MCGAL::Vertex* v : cleanBase->vertices()) {
                    if (!v || v->isRemoved()) continue;
                    if (!nv0 && samePoint(v->point(), seed->vertex()->point())) nv0 = v;
                    if (!nv1 && samePoint(v->point(), seed->end_vertex()->point())) nv1 = v;
                }
                if (nv0 && nv1) {
                    if (MCGAL::Halfedge* h = nv0->find_halfedge(nv1)) seed = h;
                }
            }
            mesh = cleanBase;
        }
    }
    if (std::getenv("PMSF_EDGEFLIP_STORE_PREFLIP_BASE") && preFlipBaseForSerialize) {
        auto samePoint = [](const MCGAL::Point& a, const MCGAL::Point& b) {
            const double dx = static_cast<double>(a.x()) - b.x();
            const double dy = static_cast<double>(a.y()) - b.y();
            const double dz = static_cast<double>(a.z()) - b.z();
            return dx*dx + dy*dy + dz*dz < 1e-10;
        };
        if (seed && seed->vertex() && seed->end_vertex()) {
            MCGAL::Vertex* oldV0 = seed->vertex();
            MCGAL::Vertex* oldV1 = seed->end_vertex();
            MCGAL::Vertex* nv0 = nullptr;
            MCGAL::Vertex* nv1 = nullptr;
            for (MCGAL::Vertex* v : preFlipBaseForSerialize->vertices()) {
                if (!v || v->isRemoved()) continue;
                if (!nv0 && samePoint(v->point(), oldV0->point())) nv0 = v;
                if (!nv1 && samePoint(v->point(), oldV1->point())) nv1 = v;
            }
            if (nv0 && nv1) {
                if (MCGAL::Halfedge* h = nv0->find_halfedge(nv1)) seed = h;
            }
        }
        mesh = preFlipBaseForSerialize;
    }
    // ---------- 15) serialize ----------
    serializer->init();
    if (inV > 0 && inF > 0) {
        // 必须在 init() 之后：init() 会复位成员状态。
        serializer->setOriginalMeshCounts(inV, inF);
    }
    if (cfg.serializerWantsSharedHeader) {
        serializer->setHeader(std::make_shared<PMSFHeader>(header));
    }
    if (cfg.serializerNeedsExplicitHeader) {
        serializer->serializeHeader(header);
    }
    if (envOn("PMSF_EDGEFLIP_DEBUG")) {
        std::fprintf(stderr,
                     "[edgeflip-debug][compress] rounds=%zu idSyms=%zu coordSyms=%zu bitRounds=%zu bits=%zu\n",
                     edgeFlipRoundCounts.size(),
                     edgeFlipSymbols.size(),
                     edgeFlipCoordSymbols.size(),
                     edgeFlipRoundBitCounts.size(),
                     edgeFlipRoundBits.size());
    }
    if (std::getenv("PMSF_BOUNDARY_FAN_PATCH")) {
        const auto preEdges = pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(mesh);
        MCGAL::Vertex* chosen = nullptr;
        MCGAL::Halfedge* chosenEdge = nullptr;
        pmsf::BoundaryFanPatch fan;
        for (MCGAL::Halfedge* h : preEdges) {
            if (!h || h->isRemoved() || h->opposite() != nullptr) continue;
            MCGAL::Vertex* v = h->end_vertex();
            if (!v || v->isRemoved()) continue;
            if (v->vertex_degree() < 3 || v->vertex_degree() > 7) continue;
            pmsf::BoundaryFanPatch cand;
            if (!pmsf::captureBoundaryFanPatch(mesh, v, cand)) continue;
            chosen = v; chosenEdge = h; fan = cand; break;
        }
        MCGAL::Halfedge* newEdge = nullptr;
        std::vector<int> residuals;
        if (chosen && chosenEdge && fan.valid) {
            chosen->setRemoved();
            MCGAL::Halfedge* hole = mesh->erase_center_vertex(chosenEdge);
            if (hole) {
                eliminate->triangulate(hole);
                pmsf::captureBoundaryPostFaces(mesh, fan);
            }
        }
        canonicalizeTraversalProxies(mesh);
        const auto postEdges = pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(mesh);
        std::vector<unsigned char> bits(1, 0);
        if (fan.valid && !fan.postFaces.empty()) {
            bits[0] = 1;
            residuals.push_back(static_cast<int>(fan.origFaces.size()));
            residuals.push_back(static_cast<int>(fan.postFaces.size()));
            residuals.push_back(static_cast<int>(fan.origGroups.size()));
            residuals.push_back(static_cast<int>(fan.boundaryEdges.size()));
            auto pushPoint = [&](const MCGAL::Point& pt) {
                const auto q = pmsf::patchInt(mesh, pt);
                residuals.push_back(q[0]); residuals.push_back(q[1]); residuals.push_back(q[2]);
            };
            pushPoint(fan.removed);
            for (const auto& tri : fan.origFaces) for (const auto& pt : tri) pushPoint(pt);
            for (const auto& tri : fan.postFaces) for (const auto& pt : tri) pushPoint(pt);
            for (int g : fan.origGroups) residuals.push_back(g);
            for (const auto& be : fan.boundaryEdges) { pushPoint(be.first); pushPoint(be.second); }
        }
        const int rounds = std::max(1, options.getRound());
        boundaryRoundBitCounts.assign(static_cast<size_t>(rounds), 0);
        if (!bits.empty()) boundaryRoundBitCounts.back() = static_cast<int>(bits.size());
        boundaryRoundBits.assign(bits.size(), 0);
        for (size_t bi = 0; bi < bits.size(); ++bi) boundaryRoundBits[bi] = bits[bi];
        boundaryRoundResiduals = residuals;
        if (std::getenv("PMSF_BOUNDARY_DEBUG"))
            std::cerr << "[boundary-fan-patch] pre=" << preEdges.size()
                      << " post=" << postEdges.size()
                      << " orig=" << fan.origFaces.size()
                      << " postFaces=" << fan.postFaces.size()
                      << " newEdge=" << (newEdge ? 1 : 0) << std::endl;
    }

    if (std::getenv("PMSF_BOUNDARY_FINAL_PATCH")) {
        const auto preEdges = pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(mesh);
        MCGAL::Vertex* chosen = nullptr;
        for (MCGAL::Halfedge* h : preEdges) {
            if (!h || h->isRemoved() || h->opposite() != nullptr) continue;
            MCGAL::Vertex* v = h->end_vertex();
            if (!v || v->isRemoved() || v->vertex_degree() != 3) continue;
            chosen = v; break;
        }
        pmsf::BoundaryCollapsePatch patch;
        MCGAL::Halfedge* newEdge = nullptr;
        std::vector<int> residuals;
        if (chosen && pmsf::captureBoundaryCollapse3(mesh, chosen, patch)) {
            pmsf::collapseBoundaryVertex3Robust(mesh, chosen, &newEdge);
        }
        canonicalizeTraversalProxies(mesh);
        const auto postEdges = pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(mesh);
        std::vector<unsigned char> bits(postEdges.size(), 0);
        if (newEdge) {
            for (size_t ei = 0; ei < postEdges.size(); ++ei) {
                if (postEdges[ei] == newEdge) { bits[ei] = 1; break; }
            }
            auto pushPoint = [&](const MCGAL::Point& pt) {
                const auto q = pmsf::patchInt(mesh, pt);
                residuals.push_back(q[0]); residuals.push_back(q[1]); residuals.push_back(q[2]);
            };
            pushPoint(patch.removed);
            pushPoint(patch.a); pushPoint(patch.b); pushPoint(patch.c);
            pushPoint(patch.f0ProxyA); pushPoint(patch.f0ProxyB);
            pushPoint(patch.f1ProxyA); pushPoint(patch.f1ProxyB);
            residuals.push_back(patch.g0); residuals.push_back(patch.g1);
        }
        const int rounds = std::max(1, options.getRound());
        boundaryRoundBitCounts.assign(static_cast<size_t>(rounds), 0);
        if (!bits.empty()) boundaryRoundBitCounts.back() = static_cast<int>(bits.size());
        boundaryRoundBits.assign(bits.size(), 0);
        for (size_t bi = 0; bi < bits.size(); ++bi) boundaryRoundBits[bi] = bits[bi];
        boundaryRoundResiduals = residuals;
        if (std::getenv("PMSF_BOUNDARY_DEBUG"))
            std::cerr << "[boundary-final-patch] pre=" << preEdges.size()
                      << " post=" << postEdges.size()
                      << " chosen=" << (chosen ? 1 : 0)
                      << " newEdge=" << (newEdge ? 1 : 0) << std::endl;
    }

    if (std::getenv("PMSF_BOUNDARY_FINAL_REMOVE")) {
        const auto preEdges = pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(mesh);
        MCGAL::Vertex* chosen = nullptr;
        for (MCGAL::Halfedge* h : preEdges) {
            if (!h || h->isRemoved() || h->opposite() != nullptr) continue;
            MCGAL::Vertex* v = h->end_vertex();
            if (!v || v->isRemoved() || v->vertex_degree() != 3) continue;
            chosen = v;
            break;
        }
        MCGAL::Halfedge* newEdge = nullptr;
        MCGAL::PointInt removedInt(0, 0, 0);
        MCGAL::PointInt edgeAInt(0, 0, 0);
        MCGAL::PointInt edgeBInt(0, 0, 0);
        if (chosen) {
            removedInt = mesh->floatPosToInt(chosen->point());
            pmsf::collapseBoundaryVertex3Robust(mesh, chosen, &newEdge);
            if (newEdge && newEdge->vertex() && newEdge->end_vertex()) {
                edgeAInt = mesh->floatPosToInt(newEdge->vertex()->point());
                edgeBInt = mesh->floatPosToInt(newEdge->end_vertex()->point());
            }
        }
        canonicalizeTraversalProxies(mesh);
        const auto postEdges = pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(mesh);
        std::vector<unsigned char> bits(postEdges.size(), 0);
        std::vector<int> residuals;
        if (newEdge) {
            for (size_t ei = 0; ei < postEdges.size(); ++ei) {
                if (postEdges[ei] == newEdge) { bits[ei] = 1; break; }
            }
            residuals.push_back(edgeAInt.x()); residuals.push_back(edgeAInt.y()); residuals.push_back(edgeAInt.z());
            residuals.push_back(edgeBInt.x()); residuals.push_back(edgeBInt.y()); residuals.push_back(edgeBInt.z());
            residuals.push_back(removedInt.x()); residuals.push_back(removedInt.y()); residuals.push_back(removedInt.z());
        }
        const int rounds = std::max(1, options.getRound());
        boundaryRoundBitCounts.assign(static_cast<size_t>(rounds), 0);
        if (!bits.empty()) boundaryRoundBitCounts[0] = static_cast<int>(bits.size());
        boundaryRoundBits.assign(bits.size(), 0);
        for (size_t bi = 0; bi < bits.size(); ++bi) boundaryRoundBits[bi] = bits[bi];
        boundaryRoundResiduals = residuals;
        if (std::getenv("PMSF_BOUNDARY_DEBUG"))
            std::cerr << "[boundary-final-remove] pre=" << preEdges.size()
                      << " post=" << postEdges.size()
                      << " chosen=" << (chosen ? 1 : 0)
                      << " newEdge=" << (newEdge ? 1 : 0) << std::endl;
    } else if (std::getenv("PMSF_BOUNDARY_SHIFT")) {

        const auto bverts = pmsf::edgeflip::collectMeshBoundaryVerticesCanonical(mesh);
        const auto bedges = pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(mesh);
        std::unordered_map<MCGAL::Vertex*, std::vector<MCGAL::Vertex*>> adj;
        for (MCGAL::Halfedge* h : bedges) {
            if (!h || h->isRemoved()) continue;
            adj[h->vertex()].push_back(h->end_vertex());
            adj[h->end_vertex()].push_back(h->vertex());
        }
        std::vector<unsigned char> bits(bverts.size(), 0);
        std::vector<int> residuals;
        int shifted = 0;
        for (size_t vi = 0; vi < bverts.size(); ++vi) {
            MCGAL::Vertex* v = bverts[vi];
            auto it = adj.find(v);
            if (!v || v->isRemoved() || it == adj.end() || it->second.size() < 2) continue;
            MCGAL::Point target;
            for (int k = 0; k < 3; ++k)
                target[k] = (it->second[0]->point()[k] + it->second[1]->point()[k]) * 0.5f;
            const MCGAL::PointInt oldInt = mesh->floatPosToInt(v->point());
            const MCGAL::PointInt newInt = mesh->floatPosToInt(target);
            const int dr[3] = {oldInt.x() - newInt.x(), oldInt.y() - newInt.y(), oldInt.z() - newInt.z()};
            if (dr[0] == 0 && dr[1] == 0 && dr[2] == 0) continue;
            v->setVertexPoint(target);
            bits[vi] = 1;
            residuals.push_back(dr[0]); residuals.push_back(dr[1]); residuals.push_back(dr[2]);
            ++shifted;
        }
        const int rounds = std::max(1, options.getRound());
        boundaryRoundBitCounts.assign(static_cast<size_t>(rounds), 0);
        if (!bits.empty()) boundaryRoundBitCounts.back() = static_cast<int>(bits.size());
        boundaryRoundBits.assign(bits.size(), 0);
        for (size_t bi = 0; bi < bits.size(); ++bi) boundaryRoundBits[bi] = bits[bi];
        boundaryRoundResiduals = residuals;
        if (std::getenv("PMSF_BOUNDARY_DEBUG"))
            std::cerr << "[boundary-shift-final] verts=" << bverts.size()
                      << " shifted=" << shifted << std::endl;
    }

    if (std::getenv("PMSF_SEAM_FINAL_BOUNDARY") && std::getenv("PMSF_SEAM_COMPRESS")) {
        if (std::getenv("PMSF_SEAM_DEBUG")) std::cerr << "[seam-final-base] V=" << mesh->size_of_vertices() << " F=" << mesh->size_of_facets() << std::endl;
        mesh->garbage_collection();
        if (std::getenv("PMSF_SEAM_DEBUG")) std::cerr << "[seam-final-gc] V=" << mesh->size_of_vertices() << " F=" << mesh->size_of_facets() << std::endl;
        std::vector<unsigned char> bits;
        std::vector<int> residuals;
        int applied = 0;
        const int maxOps = 256;
        while (applied < maxOps) {
            const auto bedges = pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(mesh);
            bool did = false;
            for (MCGAL::Halfedge* h : bedges) {
                if (!h || h->isRemoved() || h->opposite() != nullptr) continue;
                MCGAL::Vertex* v = h->end_vertex();
                if (!v || v->isRemoved()) continue;
                if (seed && seed->vertex() && seed->end_vertex() &&
                    (v == seed->vertex() || v == seed->end_vertex())) continue;
                int origIndex = -1;
                for (size_t vi = 0; vi < mesh->vertices().size(); ++vi) {
                    if (mesh->vertices()[vi] == v) { origIndex = static_cast<int>(vi); break; }
                }
                if (origIndex < 0) continue;
                int gid = 0;
                for (MCGAL::Halfedge* hv : pmsf::boundedRing(v)) {
                    if (hv && hv->face()) { gid = hv->face()->groupId(); break; }
                }
                const MCGAL::PointInt oldInt = mesh->floatPosToInt(v->point());
                MCGAL::Halfedge* newEdge = nullptr;
                if (!pmsf::collapseBoundaryVertex3Robust(mesh, v, &newEdge)) continue;
                if (!newEdge || !newEdge->vertex() || !newEdge->end_vertex()) continue;
                const auto postEdges = pmsf::edgeflip::collectMeshBoundaryEdgesSorted(mesh);
                int edgeIndex = -1;
                for (size_t ei = 0; ei < postEdges.size(); ++ei) {
                    if (postEdges[ei] == newEdge) { edgeIndex = static_cast<int>(ei); break; }
                }
                if (edgeIndex < 0) continue;
                const MCGAL::PointInt ai = mesh->floatPosToInt(newEdge->vertex()->point());
                const MCGAL::PointInt bi = mesh->floatPosToInt(newEdge->end_vertex()->point());
                const MCGAL::PointInt mid((ai.x() + bi.x()) / 2, (ai.y() + bi.y()) / 2, (ai.z() + bi.z()) / 2);
                bits.push_back(1);
                residuals.push_back(edgeIndex);
                residuals.push_back(oldInt.x() - mid.x());
                residuals.push_back(oldInt.y() - mid.y());
                residuals.push_back(oldInt.z() - mid.z());
                residuals.push_back(origIndex);
                residuals.push_back(gid);
                ++applied;
                did = true;
                break;
            }
            if (!did) break;
        }
        boundaryRoundBitCounts.assign(1, static_cast<int>(bits.size()));
        boundaryRoundBits.assign(bits.begin(), bits.end());
        boundaryRoundResiduals = residuals;
        if (std::getenv("PMSF_SEAM_DEBUG")) {
            std::cerr << "[seam-final-boundary] applied=" << applied
                      << " V=" << mesh->size_of_vertices() << " F=" << mesh->size_of_facets() << std::endl;
        }
    }


    if (std::getenv("PMSF_SEAM_FINAL_COMPLEX_FAN") && std::getenv("PMSF_SEAM_COMPRESS")) {
        std::unordered_map<int, int> inc;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
            std::unordered_set<int> seen;
            MCGAL::Halfedge* h0 = f->proxyHalfedge();
            MCGAL::Halfedge* h = h0; int guard = 0;
            do { if (h && h->vertex() && seen.insert(h->vertex()->poolId()).second) ++inc[h->vertex()->poolId()]; h = h->next(); } while (h && h != h0 && ++guard < 64);
        }
        std::vector<MCGAL::Vertex*> cand;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
            MCGAL::Halfedge* h0 = f->proxyHalfedge(); MCGAL::Halfedge* h = h0; int guard = 0;
            do {
                if (h && !h->isRemoved() && h->opposite() == nullptr && h->end_vertex()) {
                    MCGAL::Vertex* v = h->end_vertex();
                    if (v && !v->isRemoved() && inc[v->poolId()] == 3) {
                        if (!seed || !seed->vertex() || !seed->end_vertex() || (v != seed->vertex() && v != seed->end_vertex())) cand.push_back(v);
                    }
                }
                h = h->next();
            } while (h && h != h0 && ++guard < 64);
        }
        std::sort(cand.begin(), cand.end(), [&](MCGAL::Vertex* a, MCGAL::Vertex* b) { return inc[a->poolId()] > inc[b->poolId()]; });
        cand.erase(std::unique(cand.begin(), cand.end()), cand.end());
        for (MCGAL::Vertex* v : cand) {
            int k = 0; MCGAL::Point removed, a, b, apex; MCGAL::Halfedge* e = nullptr;
            std::vector<int> faceIdx, facePid; int vIdx = -1, vPid = -1;
            if (!pmsf::removeBoundaryVertexFanManual(mesh, v, k, removed, a, b, &e, &apex, &faceIdx, &facePid, &vIdx, &vPid)) continue;
            if (!e) continue;
            const auto qv = mesh->floatPosToInt(removed);
            const auto qa = mesh->floatPosToInt(a);
            const auto qb = mesh->floatPosToInt(b);
            auto qe = mesh->floatPosToInt(apex);
            auto qev = mesh->floatPosToInt(e->vertex()->point());
            const int apexSide = (qe == qev) ? 1 : -1;
            const int gid = e->face() ? e->face()->groupId() : 0;
            boundaryRoundBitCounts.assign(1, 0);
            boundaryRoundBits.clear();
            boundaryRoundResiduals = {k, qv[0], qv[1], qv[2], qa[0], qa[1], qa[2], qb[0], qb[1], qb[2], apexSide, gid, vIdx, vPid, k};
            for (int i = 0; i < k; ++i) {
                boundaryRoundResiduals.push_back(i < static_cast<int>(faceIdx.size()) ? faceIdx[static_cast<size_t>(i)] : -1);
                boundaryRoundResiduals.push_back(i < static_cast<int>(facePid.size()) ? facePid[static_cast<size_t>(i)] : -1);
            }
            if (std::getenv("PMSF_SEAM_DEBUG")) std::cerr << "[final-complex-fan] k=" << k << " gid=" << gid << std::endl;
            break;
        }
    }

    if (std::getenv("PMSF_SEAM_FINAL_COMPLEX_FAN") && !boundaryRoundResiduals.empty()) {
        std::vector<int> groups;
        for (MCGAL::Facet* f : mesh->faces())
            if (f && !f->isRemoved()) groups.push_back(f->groupId());
        MCGAL::Point seedA, seedB;
        const bool hasSeed = seed && seed->vertex() && seed->end_vertex();
        if (hasSeed) { seedA = seed->vertex()->point(); seedB = seed->end_vertex()->point(); }
        auto rebuilt = rebuildMeshFromFaceListForFinalOp(mesh, groups);
        if (rebuilt) {
            if (hasSeed) {
                auto same = [](const MCGAL::Point& x, const MCGAL::Point& y) {
                    return std::abs(x.x() - y.x()) < 1e-9 &&
                           std::abs(x.y() - y.y()) < 1e-9 &&
                           std::abs(x.z() - y.z()) < 1e-9;
                };
                MCGAL::Vertex* na = nullptr;
                MCGAL::Vertex* nb = nullptr;
                for (MCGAL::Vertex* v : rebuilt->vertices()) {
                    if (!v || v->isRemoved()) continue;
                    if (!na && same(v->point(), seedA)) na = v;
                    if (!nb && same(v->point(), seedB)) nb = v;
                }
                if (na && nb) { if (MCGAL::Halfedge* h = na->find_halfedge(nb)) seed = h; }
            }
            mesh = rebuilt;
        }
    }

    if (sharedPatchAllCount > 0) {
        std::vector<int> sharedPatchPayload;
        sharedPatchPayload.push_back(2);  // mode23 payload version 2: anchor graph + ops
        sharedPatchPayload.push_back(static_cast<int>(sharedBoundaryAnchorRecords.size()));
        sharedPatchPayload.insert(sharedPatchPayload.end(), sharedBoundaryAnchorRecords.begin(), sharedBoundaryAnchorRecords.end());
        sharedPatchPayload.push_back(sharedPatchAllCount);
        sharedPatchPayload.insert(sharedPatchPayload.end(), sharedPatchAllRecords.begin(), sharedPatchAllRecords.end());
        boundaryRoundBitCounts.assign(static_cast<size_t>(std::max(1, options.getRound())), 0);
        boundaryRoundBits.clear();
        boundaryRoundResiduals = sharedPatchPayload;
        if (std::getenv("PMSF_BOUNDARY_DEBUG"))
            std::cerr << "[shared-boundary-patch] finalOps=" << sharedPatchAllCount
                      << " payload=" << sharedPatchPayload.size() << std::endl;
        if (boundaryModuleEnabled) {
            const std::string sectionPath = pmsf::BoundaryCompressionModule::sectionPathForArtifact(
                options.getOutputPath());
            std::size_t sectionBytes = 0;
            if (pmsf::BoundaryCompressionModule::writePayloadSection(sectionPath,
                                                                     sharedPatchPayload,
                                                                     &sectionBytes)) {
                std::fprintf(stderr, "[boundary-module] payload section=%s ops=%d bytes=%zu\n",
                             sectionPath.c_str(), sharedPatchAllCount, sectionBytes);
                // The module owns the boundary payload.  Keep the standard
                // boundary section empty so the bytes are not stored twice; the
                // decoder applies the companion payload after all rounds.
                boundaryRoundBitCounts.assign(
                    static_cast<std::size_t>(std::max(1, options.getRound())), 0);
                boundaryRoundBits.clear();
                boundaryRoundResiduals.clear();
            } else {
                std::fprintf(stderr, "[boundary-module] failed to write payload %s\n",
                             sectionPath.c_str());
            }
        }
    }


    if (const char* bsMode = std::getenv("PMSF_BOUNDARY_STORAGE_MODE"); bsMode && bsMode[0]) {
        const int mode = std::atoi(bsMode);
        if (mode == 30 || mode == 31) {
            const auto payload = (mode == 30)
                                     ? pmsf::buildCompactBoundaryPayload(mesh, options.getRound())
                                     : pmsf::buildNaiveBoundaryPayload(mesh, options.getRound());
            g_boundaryStorageMode = mode;
            g_boundaryStorageCounts = payload.roundBitCounts;
            g_boundaryStorageBits = payload.bits;
            g_boundaryStorageResiduals = payload.residuals;
            std::fprintf(stderr,
                         "[boundary-storage-mode] mode=%d mesh=V%zu/F%zu rounds=%zu bits=%zu residuals=%zu\n",
                         mode, mesh->size_of_vertices(), mesh->size_of_facets(),
                         payload.roundBitCounts.size(), payload.bits.size(),
                         payload.residuals.size());
            if (mode == 30) {
                pmsf::setCompactBoundaryPayloadForVerify(payload.bits, payload.residuals);
                const auto vst = pmsf::verifyCompactBoundaryPayload(mesh);
                std::fprintf(stderr,
                             "[boundary-compact-enc-verify] segments=%d verified=%d failed=%d "
                             "anchorMissing=%d walkMismatch=%d fallback=%d\n",
                             vst.segments, vst.verified, vst.failed, vst.anchorMissing,
                             vst.walkMismatch, vst.fallbackSegments);
            }
        }
    }

    if (std::getenv("PMSF_BASE_DEDUP_FACES")) {
        std::map<std::array<MCGAL::Vertex*, 3>, MCGAL::Facet*> seen;
        std::vector<MCGAL::Facet*> dupFaces;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
            MCGAL::Vertex* vs[3];
            int n = 0;
            MCGAL::Halfedge* h0 = f->proxyHalfedge();
            MCGAL::Halfedge* h = h0;
            int guard = 0;
            do {
                if (h && h->vertex() && n < 3) vs[n++] = h->vertex();
                h = h ? h->next() : nullptr;
            } while (h && h != h0 && ++guard < 64);
            if (n != 3) continue;
            std::array<MCGAL::Vertex*, 3> key{vs[0], vs[1], vs[2]};
            std::sort(key.begin(), key.end());
            auto it = seen.find(key);
            if (it == seen.end()) {
                seen[key] = f;
            } else {
                dupFaces.push_back(f);
            }
        }
        for (MCGAL::Facet* f : dupFaces) pmsf::removeFaceForPatch(mesh, f);
    }
    if (std::getenv("PMSF_BASE_REPAIR_DUP")) {
        int repaired = 0;
        for (int pass = 0; pass < 5000; ++pass) {
            std::map<std::pair<MCGAL::Vertex*, MCGAL::Vertex*>,
                     std::vector<MCGAL::Facet*>> dir;
            for (MCGAL::Facet* f : mesh->faces()) {
                if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                MCGAL::Vertex* vs[3];
                int n = 0;
                MCGAL::Halfedge* h0 = f->proxyHalfedge();
                MCGAL::Halfedge* h = h0;
                int guard = 0;
                do {
                    if (h && h->vertex() && n < 3) vs[n++] = h->vertex();
                    h = h ? h->next() : nullptr;
                } while (h && h != h0 && ++guard < 64);
                if (n != 3) continue;
                for (int k = 0; k < 3; ++k) {
                    MCGAL::Vertex* a = vs[k];
                    MCGAL::Vertex* b = vs[(k + 1) % 3];
                    if (a && b) dir[{a, b}].push_back(f);
                }
            }
            MCGAL::Facet* victim = nullptr;
            for (auto& kv : dir) {
                if (kv.second.size() > 1) { victim = kv.second.back(); break; }
            }
            if (!victim) break;
            pmsf::removeFaceForPatch(mesh, victim);
            ++repaired;
        }
        std::fprintf(stderr, "[base-repair-dup] removedFaces=%d\n", repaired);
    }
    pmsf::printBoundaryTopologyStats();
    if (const char* fg = std::getenv("PMSF_SC_ENC_FINAL_GID")) {
        if (FILE* ff = std::fopen(fg, "w")) {
            for (MCGAL::Facet* ffac : mesh->faces()) {
                if (!ffac || ffac->isRemoved() || !ffac->proxyHalfedge()) continue;
                const auto vs = pmsf::sbdetail::faceVertices(ffac);
                if (!vs[0]) continue;
                std::vector<std::array<int, 3>> qs;
                for (int k = 0; k < 3; ++k) {
                    const auto q = mesh->floatPosToInt(vs[k]->point());
                    qs.push_back({q.x(), q.y(), q.z()});
                }
                std::sort(qs.begin(), qs.end());
                    int amask = 0;
                    {
                        const auto fvs = pmsf::sbdetail::faceVertices(ffac);
                        for (int k = 0; k < 3; ++k) {
                            MCGAL::Vertex* fv = fvs[k];
                            if (!fv) continue;
                            MCGAL::PointInt fq = mesh->floatPosToInt(fv->point());
                            std::array<int, 3> key = {fq.x(), fq.y(), fq.z()};
                            int pos = -1;
                            for (int pp = 0; pp < 3; ++pp)
                                if (qs[static_cast<std::size_t>(pp)] == key) { pos = pp; break; }
                            if (pos < 0) continue;
                            for (MCGAL::Halfedge* hh : {ffac->proxyHalfedge(), ffac->proxyHalfedge()->next(),
                                                       ffac->proxyHalfedge()->next()->next()}) {
                                if (hh && hh->vertex() == fv) {
                                    if (hh->isAdded()) amask |= (1 << pos);
                                    break;
                                }
                            }
                        }
                    }
                std::fprintf(ff, "%d %d %d %d %d %d %d %d %d %d\n", ffac->groupId(), qs[0][0],
                             qs[0][1], qs[0][2], qs[1][0], qs[1][1], qs[1][2], qs[2][0],
                             qs[2][1], qs[2][2]);
            }
            std::fclose(ff);
        }
    }

    // V9 canonical bit stream: the payload stores one bit count per *flip round*.
    // When flips become impossible before the last round (small meshes, e.g. fandisk
    // after ~14 rounds), the trailing rounds have no entry.  The decoder maps payload
    // round r to compress round r (undoing from the last round), so pad with explicit
    // zero-count rounds up to the full round count - otherwise the decoded flips would
    // be replayed on the wrong rounds.
    if (!edgeFlipRoundBitCounts.empty() && !edgeFlipRoundBits.empty() &&
        edgeFlipCoordSymbols.empty() &&
        std::getenv("PMSF_EDGEFLIP_BITS_CANONICAL") != nullptr &&
        edgeFlipRoundBitCounts.size() < static_cast<std::size_t>(options.getRound())) {
        edgeFlipRoundBitCounts.resize(static_cast<std::size_t>(options.getRound()), 0);
    }
    serializer->setEdgeFlipBitPayload(edgeFlipRoundBitCounts, edgeFlipRoundBits);

    bool scHasAnyRoundData = false;
    for (const auto& rd : g_scBoundaryRoundData)
        if (!rd.empty()) { scHasAnyRoundData = true; break; }
    if ((std::getenv("PMSF_SC_BOUNDARY_STAGE") ||
         std::getenv("PMSF_SC_BOUNDARY_COMPACT_STAGE") ||
         std::getenv("PMSF_SC_BOUNDARY_ANCHOR_STAGE") ||
         std::getenv("PMSF_SC_BOUNDARY_GROUP_STAGE") ||
         std::getenv("PMSF_SC_BOUNDARY_AUTO")) &&
        !g_scBoundaryRoundData.empty() &&
        (scHasAnyRoundData || std::getenv("PMSF_SC_BOUNDARY_AUTO") == nullptr)) {
        std::vector<std::vector<int>> dataOut = g_scBoundaryRoundData;
        const bool scNoPayload = std::getenv("PMSF_SC_NO_PAYLOAD") != nullptr;
        if (scNoPayload) dataOut.clear();
        int scMode = 61;
        if (std::getenv("PMSF_SC_BOUNDARY_GROUP_STAGE") ||
            std::getenv("PMSF_SC_BOUNDARY_AUTO")) {
            // 65 = same payload layout as 64, but the per-round boundary stage is
            // applied BEFORE the interior elimination on the encoder (reverse order
            // on the decoder: undo interior first, then apply the inverse stage).
            scMode = std::getenv("PMSF_SC_BOUNDARY_PRE_ROUNDS")
                         ? 66
                         : (std::getenv("PMSF_SC_BOUNDARY_PRE") ? 65 : 64);
        } else if (std::getenv("PMSF_SC_BOUNDARY_ANCHOR_STAGE") && !g_scAnchors.empty()) {
            // The base serializer runs garbage_collection() later; compact the
            // vertex array now so base indices refer to the final serialized order.
            if (std::getenv("PMSF_SC_NO_EARLY_GC") == nullptr)
                mesh->garbage_collection();
            MCGAL::Vertex* seedV = (seed && seed->vertex()) ? seed->vertex() : nullptr;
            MCGAL::Vertex* seedE = (seed && seed->end_vertex()) ? seed->end_vertex() : nullptr;
            if (std::getenv("PMSF_BC_ANCHOR_DIAG") && !g_bcFixedAnchors.empty()) {
                std::vector<pmsf::ScAnchorDef> anch;
                anch.reserve(g_bcFixedAnchors.size());
                for (auto& ag : g_bcFixedAnchors) {
                    pmsf::ScAnchorDef d;
                    d.v = ag.second;
                    d.group = ag.first;
                    anch.push_back(d);
                }
                const auto bidx = pmsf::anchorBaseIndicesWithSeed(mesh, anch, seedV, seedE);
                const char* af = std::getenv("PMSF_BC_ANCHOR_FILE");
                const char* apath = (af && af[0]) ? af : "/tmp/bc_anchors_q.txt";
                char bpath[512];
                std::snprintf(bpath, sizeof(bpath), "%s.base", apath);
                if (FILE* bf = std::fopen(bpath, "w")) {
                    int bad = 0;
                    for (std::size_t k = 0; k < anch.size(); ++k) {
                        if (bidx[k] < 0) ++bad;
                        std::fprintf(bf, "%d %d\n", anch[k].group, bidx[k]);
                    }
                    std::fclose(bf);
                    std::fprintf(stderr, "[bc-anchor-base] anchors=%zu bad=%d file=%s\n",
                                 anch.size(), bad, bpath);
                }
            }
            const auto baseIdx = pmsf::anchorBaseIndicesWithSeed(mesh, g_scAnchors, seedV, seedE);
            bool allOk = true;
            for (int bi : baseIdx) if (bi < 0) allOk = false;
            {
                if (!allOk)
                    std::fprintf(stderr, "[sc-anchor-payload] WARN some anchors not in base\n");
                std::vector<int> section;
                section.push_back(static_cast<int>(g_scAnchors.size()));
                for (std::size_t k = 0; k < g_scAnchors.size(); ++k) {
                    section.push_back(baseIdx[k]);
                    section.push_back(g_scAnchors[k].group);
                }
                int seedRecs = 0;
                for (const auto& sq : g_scRoundSeedQ) if (sq[0] >= 0) ++seedRecs;
                section.push_back(seedRecs);
                for (std::size_t r = 0; r < g_scRoundSeedQ.size(); ++r) {
                    const auto& sq = g_scRoundSeedQ[r];
                    if (sq[0] < 0) continue;
                    section.push_back(static_cast<int>(r));
                    for (int k = 0; k < 6; ++k) section.push_back(sq[k]);
                }
                std::vector<int> merged = section;
                merged.insert(merged.end(), dataOut.back().begin(), dataOut.back().end());
                dataOut.back() = std::move(merged);
                scMode = 63;
                if (std::getenv("PMSF_SC_DEBUG")) {
                    std::fprintf(stderr, "[sc-anchor-payload] anchors=%zu sectionInts=%zu mode=%d\n",
                                 g_scAnchors.size(), section.size(), scMode);
                    for (std::size_t k = 0; k < g_scAnchors.size(); ++k) {
                        const auto q = pmsf::scAnchorQ(mesh.get(), g_scAnchors[k].v);
                        std::fprintf(stderr, "  encAnchor[%zu] base=%d group=%d q=(%d,%d,%d)\n", k,
                                     baseIdx[k], g_scAnchors[k].group, q[0], q[1], q[2]);
                    }
                }
            }
        } else if (std::getenv("PMSF_SC_BOUNDARY_COMPACT_STAGE")) {
            scMode = 62;
        }
        if (scMode == 66) {
            for (std::size_t r = 0; r < dataOut.size(); ++r) {
                if (dataOut[r].empty()) continue;
                const int ord = (r < g_scRoundOrdering.size()) ? g_scRoundOrdering[r] : 0;
                dataOut[r].insert(dataOut[r].begin(), ord ? 1 : 0);
            }
        }
        boundaryRoundBitCounts.assign(dataOut.size(), 0);
        boundaryRoundBits.clear();
        boundaryRoundResiduals.clear();
        for (std::size_t r = 0; r < dataOut.size(); ++r) {
            const auto& data = dataOut[r];
            const int n = static_cast<int>(data.size());
            boundaryRoundBitCounts[r] = n;
            boundaryRoundBits.insert(boundaryRoundBits.end(), static_cast<std::size_t>(std::max(0, n)),
                                     static_cast<unsigned char>(1));
            boundaryRoundResiduals.insert(boundaryRoundResiduals.end(), data.begin(), data.end());
        }
        if (std::getenv("PMSF_SC_BOUNDARY_NO_PAYLOAD")) {
            if (std::getenv("PMSF_SC_DEBUG"))
                std::fprintf(stderr, "[sc-nopayload] residualInts=%zu (payload omitted)\n",
                             boundaryRoundResiduals.size());
        } else {
            if (std::getenv("PMSF_BC_ANCHOR_DIAG") && !g_bcFixedAnchors.empty() && mesh) {
                std::vector<pmsf::ScAnchorDef> anch;
                anch.reserve(g_bcFixedAnchors.size());
                for (auto& ag : g_bcFixedAnchors) {
                    pmsf::ScAnchorDef d;
                    d.v = ag.second;
                    d.group = ag.first;
                    anch.push_back(d);
                }
                MCGAL::Vertex* seedV = (seed && seed->vertex()) ? seed->vertex() : nullptr;
                MCGAL::Vertex* seedE = (seed && seed->end_vertex()) ? seed->end_vertex() : nullptr;
                const auto bidx = pmsf::anchorBaseIndicesWithSeed(mesh, anch, seedV, seedE);
                const char* af = std::getenv("PMSF_BC_ANCHOR_FILE");
                const char* apath = (af && af[0]) ? af : "/tmp/bc_anchors_q.txt";
                char bpath[512];
                std::snprintf(bpath, sizeof(bpath), "%s.base", apath);
                if (FILE* bf = std::fopen(bpath, "w")) {
                    int bad = 0;
                    for (std::size_t k = 0; k < anch.size(); ++k) {
                        if (bidx[k] < 0) ++bad;
                        std::fprintf(bf, "%d %d\n", anch[k].group, bidx[k]);
                    }
                    std::fclose(bf);
                    std::fprintf(stderr, "[bc-anchor-base] anchors=%zu bad=%d file=%s\n",
                                 anch.size(), bad, bpath);
                }
            }
            serializer->setBoundaryBitPayload(boundaryRoundBitCounts, boundaryRoundBits,
                                              boundaryRoundResiduals, scMode);
        }
    } else if (std::getenv("PMSF_GROUP_BOUNDARY_SPLIT") && !g_groupSplitResiduals.empty()) {
        boundaryRoundBitCounts.assign(static_cast<std::size_t>(std::max(1, options.getRound())), 0);
        boundaryRoundBits.clear();
        boundaryRoundResiduals = g_groupSplitResiduals;
        serializer->setBoundaryBitPayload(boundaryRoundBitCounts, boundaryRoundBits,
                                          boundaryRoundResiduals, 50);
    } else if (std::getenv("PMSF_GROUP_ANCHORS") &&
        !std::getenv("PMSF_GROUP_ANCHORS_NO_PAYLOAD") &&
        !std::getenv("PMSF_SEG_BFS_ROUND") &&
        !g_groupAnchorResiduals.empty() &&
        !(pmsf::boundaryTopologyPatchEnabled() && !pmsf::boundaryTopologyOps().empty())) {
        boundaryRoundBitCounts.assign(static_cast<std::size_t>(std::max(1, options.getRound())), 0);
        boundaryRoundBits.clear();
        boundaryRoundResiduals = g_groupAnchorResiduals;
        serializer->setBoundaryBitPayload(boundaryRoundBitCounts, boundaryRoundBits,
                                          boundaryRoundResiduals, 42);
    } else if (std::getenv("PMSF_SEG_BFS_ROUND") && !g_bfsAnchorResiduals.empty()) {
        boundaryRoundBitCounts.assign(static_cast<std::size_t>(std::max(1, options.getRound())), 0);
        boundaryRoundBits.clear();
        boundaryRoundResiduals = g_bfsAnchorResiduals;
        serializer->setBoundaryBitPayload(boundaryRoundBitCounts, boundaryRoundBits,
                                          boundaryRoundResiduals, 42);
    } else if (pmsf::boundaryTopologyPatchEnabled() && !pmsf::boundaryTopologyOps().empty()) {
        const auto& ops = pmsf::boundaryTopologyOps();
        std::vector<pmsf::BoundaryTopologyPatchOp> patchOps;
        patchOps.reserve(ops.size());
        for (const auto& op : ops) {
            pmsf::BoundaryTopologyPatchOp po;
            po.round = op.round;
            po.qx = op.q.x();
            po.qy = op.q.y();
            po.qz = op.q.z();
            po.gids = op.gids;
            patchOps.push_back(std::move(po));
        }
        auto patchBits = pmsf::encodeBoundaryTopologyPatchBits(patchOps);
        if (std::getenv("PMSF_BOUNDARY_DEBUG")) {
            std::vector<pmsf::BoundaryTopologyPatchOp> back;
            const bool ok = pmsf::parseBoundaryTopologyPatchBits(patchBits, back);
            int mismatch = 0;
            auto keyLess = [](const auto& a, const auto& b) {
                if (a.qx != b.qx) return a.qx < b.qx;
                if (a.qy != b.qy) return a.qy < b.qy;
                if (a.qz != b.qz) return a.qz < b.qz;
                return a.round < b.round;
            };
            auto sortedIn = patchOps;
            std::sort(sortedIn.begin(), sortedIn.end(), keyLess);
            if (ok && back.size() == sortedIn.size()) {
                for (std::size_t i = 0; i < back.size(); ++i) {
                    if (back[i].round != sortedIn[i].round || back[i].qx != sortedIn[i].qx ||
                        back[i].qy != sortedIn[i].qy || back[i].qz != sortedIn[i].qz ||
                        back[i].gids != sortedIn[i].gids) ++mismatch;
                }
            }
            std::fprintf(stderr, "[btp-roundtrip-sorted] mismatch=%d\n", mismatch);
            {
                std::vector<int> hist(64, 0);
                for (const auto& op : patchOps) if (op.round >= 0 && op.round < 64) ++hist[static_cast<std::size_t>(op.round)];
                std::fprintf(stderr, "[btp-hist]");
                for (int r = 0; r < 64; ++r) if (hist[static_cast<std::size_t>(r)]) std::fprintf(stderr, " %d:%d", r, hist[static_cast<std::size_t>(r)]);
                std::fprintf(stderr, "\n");
            }
            for (const auto& b : back) {
                if (b.round == 29) {
                    std::fprintf(stderr, "[btp-first-r29] q=%d,%d,%d d=%zu\n", b.qx, b.qy, b.qz, b.gids.size());
                    break;
                }
            }
            std::fprintf(stderr,
                         "[btp-roundtrip] ok=%d in=%zu out=%zu mismatch=%d firstRound29=%d@%d,%d,%d d=%zu\n",
                         ok, patchOps.size(), back.size(), mismatch,
                         patchOps.empty() ? -1 : patchOps.front().round,
                         patchOps.empty() ? 0 : patchOps.front().qx,
                         patchOps.empty() ? 0 : patchOps.front().qy,
                         patchOps.empty() ? 0 : patchOps.front().qz,
                         patchOps.empty() ? 0 : patchOps.front().gids.size());
        }
        boundaryRoundBitCounts.assign(static_cast<std::size_t>(std::max(1, options.getRound())), 0);
        boundaryRoundBitCounts[0] = static_cast<int>(patchBits.size());
        boundaryRoundBits = std::move(patchBits);
        boundaryRoundResiduals.assign(1, static_cast<int>(ops.size()));
        std::fprintf(stderr,
                     "[boundary-topology-patch] ops=%zu patchBits=%zu bytes=%.1f\n",
                     ops.size(), boundaryRoundBits.size(),
                     static_cast<double>(boundaryRoundBits.size()) / 8.0);
        serializer->setBoundaryBitPayload(boundaryRoundBitCounts, boundaryRoundBits,
                                          boundaryRoundResiduals, 41);
    } else if (g_boundaryStorageMode != 0) {
        serializer->setBoundaryBitPayload(g_boundaryStorageCounts, g_boundaryStorageBits,
                                          g_boundaryStorageResiduals, g_boundaryStorageMode);
    } else if (std::getenv("PMSF_SEAM_BOUNDARY_ROUND_END")) {
        serializer->setBoundaryBitPayload(boundaryRoundBitCounts, boundaryRoundBits,
                                          boundaryRoundResiduals, 13);
    } else {
        serializer->setBoundaryBitPayload(boundaryRoundBitCounts, boundaryRoundBits,
                                          boundaryRoundResiduals,
                                          (std::getenv("PMSF_SHARED_BOUNDARY_PATCH") && !std::getenv("PMSF_BOUNDARY_MODULE")) ? 23 : std::getenv("PMSF_SEAM_PRE_COMPLEX_FAN") ? 22 : std::getenv("PMSF_SEAM_FINAL_COMPLEX_FAN") ? 21 : std::getenv("PMSF_SEAM_BOUNDARY_ORDER_BITS") ? 20 : std::getenv("PMSF_SEAM_MIXED_BOUNDARY") ? 12 : std::getenv("PMSF_SEAM_BOUNDARY_ORDERED") ? 11 : std::getenv("PMSF_SEAM_FINAL_BOUNDARY") ? 10 : std::getenv("PMSF_SEAM_BOUNDARY") ? 13 : (std::getenv("PMSF_BOUNDARY_FAN_PATCH") ? 7 : (std::getenv("PMSF_BOUNDARY_FINAL_PATCH") ? 6 : (std::getenv("PMSF_BOUNDARY_PATCH_ROUND") ? 5 : ((std::getenv("PMSF_BOUNDARY_PRE_REMOVE") || std::getenv("PMSF_BOUNDARY_FINAL_REMOVE")) ? 2 : (std::getenv("PMSF_BOUNDARY_SHIFT") ? 1 : 0))))));
    }
    bool bndV3Written = false;
    if (bcV2Enabled && mesh && !g_bcV2Rounds.empty() && pmsf::bndid::envEnabled()) {
        MCGAL::Vertex* seedV = (seed && seed->vertex()) ? seed->vertex() : nullptr;
        MCGAL::Vertex* seedE = (seed && seed->end_vertex()) ? seed->end_vertex() : nullptr;
        // Base index map mirroring the default BasicSerializeOperator order:
        // seedV, seedE, then mesh->vertices() live order.
        std::unordered_map<MCGAL::Vertex*, int> basePos;
        auto addBase = [&](MCGAL::Vertex* v) {
            if (!v || v->isRemoved()) return;
            if (basePos.find(v) != basePos.end()) return;
            basePos[v] = static_cast<int>(basePos.size());
        };
        addBase(seedV);
        addBase(seedE);
        for (MCGAL::Vertex* v : mesh->vertices()) addBase(v);
        const int R = options.getRound();
        // anchors (group, base index; -1 = unresolved -> payload rejected)
        std::vector<std::pair<int, int>> anchRecs;
        bool anchorsOk = true;
        for (auto& ag : g_bcV2Anchors) {
            auto it = basePos.find(ag.second);
            if (it == basePos.end()) {
                anchorsOk = false;
                anchRecs.emplace_back(ag.first, -1);
            } else {
                anchRecs.emplace_back(ag.first, it->second);
            }
        }
        // Per-round BFS seed records (stable IDs): restores the exact collect root
        // used by the encoder for each round's interior replay.
        std::vector<int> seedRecs;
        for (int r = 0; r < R; ++r) {
            if (r >= static_cast<int>(g_scRoundSeedVerts.size())) break;
            const auto& sv = g_scRoundSeedVerts[static_cast<std::size_t>(r)];
            if (!sv[0] || !sv[1] || sv[0]->isRemoved() || sv[1]->isRemoved()) continue;
            pmsf::bndid::Id id0, id1;
            if (!pmsf::bndid::encResolve(sv[0], basePos, id0)) continue;
            if (!pmsf::bndid::encResolve(sv[1], basePos, id1)) continue;
            seedRecs.push_back(r);
            seedRecs.push_back(id0.kind); seedRecs.push_back(id0.a); seedRecs.push_back(id0.b);
            seedRecs.push_back(id1.kind); seedRecs.push_back(id1.a); seedRecs.push_back(id1.b);
        }
        bool allResolved = anchorsOk;
        auto resolveId = [&](MCGAL::Vertex* v, int& kind, int& a, int& b) -> bool {
            pmsf::bndid::Id id;
            if (!pmsf::bndid::encResolve(v, basePos, id)) return false;
            kind = id.kind; a = id.a; b = id.b;
            return true;
        };
        int unresolved = 0;

        // ---- compact byte payload (v2) ----
        pmsf::bndid::BndBytes v3b;
        auto putU = [&](std::uint32_t v) { pmsf::bndid::putU(v3b, v); };
        auto putS = [&](std::int32_t v) { pmsf::bndid::putS(v3b, v); };
        auto putId = [&](int kind, int a, int b) {
            v3b.push_back(static_cast<unsigned char>(kind < 0 ? 3 : (kind & 3)));
            putU(static_cast<std::uint32_t>(std::max(0, b)));
            if (kind > 0) putU(static_cast<std::uint32_t>(std::max(0, a)));
        };
        // Self-describing schedule flags:
        //   bit0 = decoupled boundary-first decode,
        //   bit1 = per-op ring certificates (third-vertex IDs) follow each op,
        //   bit2 = per-op explicit patch topology follows each op.
        const bool v3Certs = std::getenv("PMSF_BND_V3_CERTS") != nullptr;
        const bool v3AllExplicit = std::getenv("PMSF_BND_V3_EXPLICIT_ALL") != nullptr;
        std::set<std::pair<int, int>> v3ExplicitOps;
        if (const char* eo = std::getenv("PMSF_BND_V3_EXPLICIT_OPS")) {
            const std::string es(eo);
            std::size_t p = 0;
            while (p < es.size()) {
                const std::size_t c = es.find(',', p);
                const std::string tok = es.substr(p, c == std::string::npos ? std::string::npos : c - p);
                const std::size_t colon = tok.find(':');
                if (colon != std::string::npos)
                    v3ExplicitOps.emplace(std::atoi(tok.substr(0, colon).c_str()),
                                          std::atoi(tok.substr(colon + 1).c_str()));
                if (c == std::string::npos) break;
                p = c + 1;
            }
        }
        std::set<std::pair<int, int>> v3CertOps;
        if (const char* co = std::getenv("PMSF_BND_V3_CERTS_OPS")) {
            const std::string cs(co);
            std::size_t p = 0;
            while (p < cs.size()) {
                const std::size_t cc = cs.find(',', p);
                const std::string tok = cs.substr(p, cc == std::string::npos ? std::string::npos : cc - p);
                const std::size_t colon = tok.find(':');
                if (colon != std::string::npos)
                    v3CertOps.emplace(std::atoi(tok.substr(0, colon).c_str()),
                                      std::atoi(tok.substr(colon + 1).c_str()));
                if (cc == std::string::npos) break;
                p = cc + 1;
            }
        }
        const bool v3CertsAll = std::getenv("PMSF_BND_V3_CERTS") != nullptr;
        const bool v3CertsSel = !v3CertOps.empty();
        const bool v3Explicit = v3AllExplicit || !v3ExplicitOps.empty();
        const bool v3Njr = std::getenv("PMSF_BND_V3_NJR") != nullptr;
        const bool v3V4 = std::getenv("PMSF_BND_V4") != nullptr;
        int v3Strat = 0;
        if (const char* stEnv = std::getenv("PMSF_BND_V3_DECODE_STRAT"))
            v3Strat = std::atoi(stEnv) & 3;
        std::set<int> v3OrderRounds;
        if (const char* ors = std::getenv("PMSF_BND_V3_ORDER_ROUNDS")) {
            const std::string oss(ors);
            std::size_t p = 0;
            while (p < oss.size()) {
                const std::size_t cc = oss.find(',', p);
                const std::string tok =
                    oss.substr(p, cc == std::string::npos ? std::string::npos : cc - p);
                if (!tok.empty()) v3OrderRounds.insert(std::atoi(tok.c_str()));
                if (cc == std::string::npos) break;
                p = cc + 1;
            }
        }
        const bool v3RoundMeta = std::getenv("PMSF_BND_V3_ROUND_RETRY") != nullptr ||
                                 !v3OrderRounds.empty();
        const unsigned char v3FlagsByte =
            static_cast<unsigned char>((std::getenv("PMSF_BND_DECOUPLED") ? 1 : 0) |
                                       ((v3CertsAll || v3CertsSel) ? 2 : 0) | (v3Explicit ? 4 : 0) |
                                       (v3Njr ? 8 : 0) | (v3V4 ? 16 : 0) |
                                       (static_cast<unsigned char>((v3Strat & 3) << 5)) |
                                       (v3RoundMeta ? 128 : 0));
        v3b.push_back(v3FlagsByte);
        // Anchor table (one fixed anchor per group, chosen after segmentation):
        // sorted by base index, baseIdx delta-coded + group uvar.  The decoder uses
        // these for the anchor-driven boundary-loop traversal check (no coordinates).
        putU(static_cast<std::uint32_t>(anchRecs.size()));
        {
            std::vector<std::pair<int, int>> sortedAnchors = anchRecs;
            std::sort(sortedAnchors.begin(), sortedAnchors.end(),
                      [](const std::pair<int, int>& x, const std::pair<int, int>& y) {
                          return x.second < y.second;
                      });
            int prevB = 0;
            for (auto& ar : sortedAnchors) {
                putU(static_cast<std::uint32_t>(std::max(0, ar.first)));
                const int delta = ar.second - prevB;
                putU(static_cast<std::uint32_t>(std::max(0, delta)));
                prevB = ar.second;
            }
        }
        putU(static_cast<std::uint32_t>(R));
        // Seeds: emit only rounds where the seed ID pair CHANGED (decoder carries the
        // last record forward); usually the BFS root is stable for many rounds.
        std::vector<int> seedChangeRec;  // [round, id0(k,a,b), id1(k,a,b)] per change
        {
            int lastS[6] = {-1, -1, -1, -1, -1, -1};
            for (std::size_t si = 0; si + 6 < seedRecs.size(); si += 7) {
                const int* cur = &seedRecs[si + 1];
                bool same = true;
                for (int k = 0; k < 6; ++k)
                    if (cur[k] != lastS[k]) { same = false; break; }
                if (same) continue;
                seedChangeRec.push_back(seedRecs[si]);
                for (int k = 0; k < 6; ++k) { seedChangeRec.push_back(cur[k]); lastS[k] = cur[k]; }
            }
        }
        putU(static_cast<std::uint32_t>(seedChangeRec.size() / 7));
        for (std::size_t si = 0; si + 6 < seedChangeRec.size(); si += 7) {
            putU(static_cast<std::uint32_t>(seedChangeRec[si]));
            putId(seedChangeRec[si + 1], seedChangeRec[si + 2], seedChangeRec[si + 3]);
            putId(seedChangeRec[si + 4], seedChangeRec[si + 5], seedChangeRec[si + 6]);
        }
        if (v3Explicit) {
            // Explicit-op bitmap: only listed ops carry the (large) per-op patch arrays.
            if (v3AllExplicit) {
                putU(0xFFFFFFFFu);
            } else {
                putU(static_cast<std::uint32_t>(v3ExplicitOps.size()));
                for (const auto& kv : v3ExplicitOps) {
                    putU(static_cast<std::uint32_t>(std::max(0, kv.first)));
                    putU(static_cast<std::uint32_t>(std::max(0, kv.second)));
                }
            }
        }
        if (v3CertsAll || v3CertsSel) {
            if (v3CertsAll) {
                putU(0xFFFFFFFFu);
            } else {
                putU(static_cast<std::uint32_t>(v3CertOps.size()));
                for (const auto& kv : v3CertOps) {
                    putU(static_cast<std::uint32_t>(std::max(0, kv.first)));
                    putU(static_cast<std::uint32_t>(std::max(0, kv.second)));
                }
            }
        }
        if (v3RoundMeta) {
            putU(static_cast<std::uint32_t>(v3OrderRounds.size()));
            for (int orr : v3OrderRounds) putU(static_cast<std::uint32_t>(std::max(0, orr)));
        }
        int expWritten = 0, expFallback = 0;
        for (int r = 0; r < R; ++r) {
            int ord = 0;
            if (std::getenv("PMSF_SC_V2_STAGE_ORDER_LIST") != nullptr) {
                ord = (r < static_cast<int>(g_scEffectiveRoundOrdering.size()))
                          ? g_scEffectiveRoundOrdering[static_cast<std::size_t>(r)]
                          : 0;
            } else {
                ord = (r < static_cast<int>(g_scRoundOrdering.size()))
                          ? g_scRoundOrdering[static_cast<std::size_t>(r)]
                          : 0;
            }
            const pmsf::BoundaryV2Round* rr =
                (r < static_cast<int>(g_bcV2Rounds.size())) ? &g_bcV2Rounds[static_cast<std::size_t>(r)] : nullptr;
            const std::size_t nOps = rr ? rr->ops.size() : 0;
            putU(static_cast<std::uint32_t>(std::max(0, ord)));
            putU(static_cast<std::uint32_t>(nOps));
            for (std::size_t oi = 0; oi < nOps; ++oi) {
                const auto& op = rr->ops[oi];
                // Vertex refs: stable ID when resolvable, else kind=3 + quantized position
                // (position fallback for decode-side resolution; never used for locating loops).
                auto putVertexRef = [&](MCGAL::Vertex* v) {
                    int kind = -1, a = 0, b = 0;
                    if (v && resolveId(v, kind, a, b)) {
                        putId(kind, a, b);
                        return;
                    }
                    ++unresolved;
                    if (!v) {
                        v3b.push_back(3);
                        putU(0);
                        putU(0);
                        putU(0);
                        return;
                    }
                    v3b.push_back(3);
                    const auto q = mesh->floatPosToInt(v->point());
                    putU(static_cast<std::uint32_t>(std::max(0, q.x())));
                    putU(static_cast<std::uint32_t>(std::max(0, q.y())));
                    putU(static_cast<std::uint32_t>(std::max(0, q.z())));
                };
                putVertexRef(op.a);
                putVertexRef(op.b);
                putS(op.group);
                putS(op.gB);
                const std::uint32_t kcode =
                    (static_cast<std::uint32_t>(std::max(0, std::min(63, op.kA))) & 63u) |
                    ((static_cast<std::uint32_t>(std::max(0, std::min(63, op.kB))) & 63u) << 6) |
                    ((static_cast<std::uint32_t>(std::max(0, std::min(15, op.orient))) & 15u) << 12);
                putU(kcode);
                const int mq = (mesh->i_nbQuantBits > 0) ? ((1 << mesh->i_nbQuantBits) - 1) : (1 << 30);
                for (int k = 0; k < 3; ++k)
                    putU(static_cast<std::uint32_t>(std::clamp(op.removedQ[k], 0, mq)));
                if (v3RoundMeta && v3OrderRounds.count(r) != 0) {
                    const bool ranksOk = op.vRank >= 0 &&
                                         static_cast<int>(op.faceRanks.size()) ==
                                             static_cast<int>(op.kA + op.kB);
                    putS(ranksOk ? op.vRank : -1);
                    if (ranksOk)
                        for (int rk : op.faceRanks) putS(rk);
                    else
                        for (int k = 0; k < op.kA + op.kB; ++k) putS(-1);
                }
                if (v3CertsAll || v3CertOps.count({r, static_cast<int>(oi)}) != 0) {
                    auto putThird = [&](MCGAL::Vertex* tv) {
                        int tk = -1, ta = 0, tb = 0;
                        if (!tv || tv->isRemoved() || !resolveId(tv, tk, ta, tb)) {
                            v3b.push_back(3);  // unresolved certificate (1 byte only)
                            return;
                        }
                        putId(tk, ta, tb);
                    };
                    putThird(op.thirdA);
                    putThird(op.thirdB);
                }
                const bool wantExplicit =
                    v3AllExplicit || v3ExplicitOps.count({r, static_cast<int>(oi)}) != 0;
                if (v3Explicit && wantExplicit) {
                    // Build a per-op local vertex dictionary: each unique vertex gets one
                    // record (stable ID / position fallback / removed placeholder); faces
                    // then reference local indices.  V4 format keeps explicit patches small.
                    std::unordered_map<MCGAL::Vertex*, int> localIdx;
                    std::vector<MCGAL::Vertex*> uniq;
                    std::vector<int> uniqIsRemoved;
                    auto addVertex = [&](MCGAL::Vertex* v, bool isRemoved) -> int {
                        auto it = localIdx.find(v);
                        if (it != localIdx.end()) return it->second;
                        const int idx = static_cast<int>(uniq.size());
                        uniq.push_back(v);
                        uniqIsRemoved.push_back(isRemoved ? 1 : 0);
                        localIdx[v] = idx;
                        return idx;
                    };
                    std::vector<std::array<int, 4>> origRec, addRec;
                    auto buildFaces = [&](const std::vector<std::array<MCGAL::Vertex*, 3>>& tris,
                                          const std::vector<int>& gids,
                                          std::vector<std::array<int, 4>>& out) -> bool {
                        out.clear();
                        out.reserve(tris.size());
                        for (std::size_t ti = 0; ti < tris.size(); ++ti) {
                            std::array<int, 4> rec{};
                            rec[0] = (ti < gids.size()) ? gids[ti] : op.group;
                            for (int k = 0; k < 3; ++k) {
                                MCGAL::Vertex* fv = tris[ti][static_cast<std::size_t>(k)];
                                rec[1 + k] = addVertex(fv, fv == op.removedVertex);
                            }
                            out.push_back(rec);
                        }
                        return true;
                    };
                    const bool okOrig = buildFaces(op.expOrig, op.expOrigGid, origRec);
                    const bool okAdd = buildFaces(op.expAdd, op.expAddGid, addRec);
                    if (okOrig && okAdd && !addRec.empty()) {
                        ++expWritten;
                        if (v3V4) {
                            putU(static_cast<std::uint32_t>(uniq.size()));
                            for (std::size_t vi = 0; vi < uniq.size(); ++vi) {
                                MCGAL::Vertex* vv = uniq[vi];
                                if (uniqIsRemoved[vi]) {
                                    putU(3);  // removed-vertex placeholder (this op's S-ID)
                                    continue;
                                }
                                if (!vv) return;  // cannot happen: null vertices are placeholders
                                const auto q = mesh->floatPosToInt(vv->point());
                                int kind = -1, aa = 0, bb = 0;
                                if (resolveId(vv, kind, aa, bb)) {
                                    if (kind == 1) {
                                        putU(2);
                                        putU(static_cast<std::uint32_t>(kind));
                                        putU(static_cast<std::uint32_t>(std::max(0, bb)));
                                        putU(static_cast<std::uint32_t>(std::max(0, aa)));
                                        putU(static_cast<std::uint32_t>(std::max(0, q.x())));
                                        putU(static_cast<std::uint32_t>(std::max(0, q.y())));
                                        putU(static_cast<std::uint32_t>(std::max(0, q.z())));
                                    } else {
                                        putU(0);
                                        putU(static_cast<std::uint32_t>(kind));
                                        putU(static_cast<std::uint32_t>(std::max(0, bb)));
                                        if (kind > 0)
                                            putU(static_cast<std::uint32_t>(std::max(0, aa)));
                                    }
                                } else {
                                    putU(1);
                                    putU(static_cast<std::uint32_t>(std::max(0, q.x())));
                                    putU(static_cast<std::uint32_t>(std::max(0, q.y())));
                                    putU(static_cast<std::uint32_t>(std::max(0, q.z())));
                                }
                            }
                            putU(static_cast<std::uint32_t>(origRec.size()));
                            for (const auto& rec : origRec) {
                                putS(rec[0]);
                                putU(static_cast<std::uint32_t>(rec[1]));
                                putU(static_cast<std::uint32_t>(rec[2]));
                                putU(static_cast<std::uint32_t>(rec[3]));
                            }
                            putU(static_cast<std::uint32_t>(addRec.size()));
                            for (const auto& rec : addRec) {
                                putS(rec[0]);
                                putU(static_cast<std::uint32_t>(rec[1]));
                                putU(static_cast<std::uint32_t>(rec[2]));
                                putU(static_cast<std::uint32_t>(rec[3]));
                            }
                        } else {
                            // Legacy per-face tag format (kept for A/B checks).
                            auto writeFace = [&](const std::array<int, 22>& rec) {
                                putS(rec[0]);
                                for (int k = 0; k < 3; ++k) {
                                    const int* vr = &rec[1 + 7 * k];
                                    const int tag = vr[0];
                                    putU(static_cast<std::uint32_t>(std::max(0, tag)));
                                    if (tag == 0 || tag == 2) {
                                        putU(static_cast<std::uint32_t>(std::max(0, vr[1])));
                                        putU(static_cast<std::uint32_t>(std::max(0, vr[3])));
                                        if (vr[1] > 0)
                                            putU(static_cast<std::uint32_t>(std::max(0, vr[2])));
                                    }
                                    if (tag == 1 || tag == 2) {
                                        putU(static_cast<std::uint32_t>(std::max(0, vr[4])));
                                        putU(static_cast<std::uint32_t>(std::max(0, vr[5])));
                                        putU(static_cast<std::uint32_t>(std::max(0, vr[6])));
                                    }
                                }
                            };
                            std::vector<std::array<int, 22>> expOrigRec, expAddRec;
                            auto buildLegacy = [&](const std::vector<std::array<MCGAL::Vertex*, 3>>& tris,
                                                   const std::vector<int>& gids,
                                                   std::vector<std::array<int, 22>>& out) -> bool {
                                out.clear();
                                out.reserve(tris.size());
                                for (std::size_t ti = 0; ti < tris.size(); ++ti) {
                                    std::array<int, 22> rec{};
                                    rec[0] = (ti < gids.size()) ? gids[ti] : op.group;
                                    for (int k = 0; k < 3; ++k) {
                                        MCGAL::Vertex* fv = tris[ti][static_cast<std::size_t>(k)];
                                        const auto q = fv ? mesh->floatPosToInt(fv->point())
                                                          : MCGAL::PointInt(0, 0, 0);
                                        int* vr = &rec[1 + 7 * k];
                                        if (fv == op.removedVertex) {
                                            vr[1] = 2;
                                            vr[2] = r;
                                            vr[3] = static_cast<int>(oi);
                                        } else {
                                            int kind = -1, aa = 0, bb = 0;
                                            if (!resolveId(fv, kind, aa, bb)) return false;
                                            if (kind == 1) {
                                                vr[0] = 2;
                                                vr[1] = kind;
                                                vr[2] = aa;
                                                vr[3] = bb;
                                                vr[4] = q.x();
                                                vr[5] = q.y();
                                                vr[6] = q.z();
                                            } else {
                                                vr[0] = 0;
                                                vr[1] = kind;
                                                vr[2] = aa;
                                                vr[3] = bb;
                                            }
                                        }
                                    }
                                    out.push_back(rec);
                                }
                                return true;
                            };
                            if (!buildLegacy(op.expOrig, op.expOrigGid, expOrigRec) ||
                                !buildLegacy(op.expAdd, op.expAddGid, expAddRec)) {
                                // fall back to empty arrays (should not happen for selective ops)
                                putU(0);
                                putU(0);
                            } else {
                                putU(static_cast<std::uint32_t>(expOrigRec.size()));
                                for (const auto& rec : expOrigRec) writeFace(rec);
                                putU(static_cast<std::uint32_t>(expAddRec.size()));
                                for (const auto& rec : expAddRec) writeFace(rec);
                            }
                        }
                    } else {
                        ++expFallback;
                    }
                }
                // Ring certificate omitted: experiments show the late-round failures
                // are element-creation/pool drift, not pair ambiguity; keeping certs
                // costs bytes, so the compact payload omits them.  Writer verification
                // still rejects any artifact that cannot decode.
                (void)0;
            }
            if (v3RoundMeta)
                putU(static_cast<std::uint32_t>(
                    (rr && rr->groupBoundaryEdges >= 0) ? (rr->groupBoundaryEdges + 1) : 0));
        }

        if (std::getenv("PMSF_V3_PRINT_NOPS")) {
            std::fprintf(stderr, "[bnd-v3-nops]");
            for (int rr2 = 0; rr2 < R; ++rr2) {
                const pmsf::BoundaryV2Round* rp =
                    (rr2 < static_cast<int>(g_bcV2Rounds.size()))
                        ? &g_bcV2Rounds[static_cast<std::size_t>(rr2)]
                        : nullptr;
                std::fprintf(stderr, " %d", rp ? static_cast<int>(rp->ops.size()) : 0);
            }
            std::fprintf(stderr, "\n");
        }
        if (allResolved) {
            std::vector<int> v3;
            v3.push_back(-static_cast<int>(v3b.size()));  // negative length => compact v2 bytes
            std::size_t bi = 0;
            while (bi < v3b.size()) {
                int w = 0;
                for (int k = 0; k < 4; ++k) {
                    if (bi + static_cast<std::size_t>(k) < v3b.size())
                        w |= static_cast<int>(v3b[bi + static_cast<std::size_t>(k)]) << (8 * k);
                }
                v3.push_back(w);
                bi += 4;
            }
            std::vector<int> v3Counts(static_cast<std::size_t>(std::max(0, R)), 0);
            boundaryRoundBitCounts = std::move(v3Counts);
            boundaryRoundBits.clear();
            boundaryRoundResiduals = std::move(v3);
            serializer->setBoundaryBitPayload(boundaryRoundBitCounts, boundaryRoundBits,
                                              boundaryRoundResiduals, 71);
            bndV3Written = true;
            std::fprintf(stderr,
                         "[bnd-v3-payload] rounds=%d checksumMode=skeleton-v2 payloadBytes=%zu residuals=%zu\n",
                         R, v3b.size(), boundaryRoundResiduals.size());
        } else {
            std::fprintf(stderr,
                         "[bnd-v3-payload] WARN unresolved anchors/ops -> fallback to mode70 (anchorsOk=%d unresolved=%d)\n",
                         anchorsOk ? 1 : 0, unresolved);
        }

    }
    if (bcV2Enabled && mesh && !g_bcV2Rounds.empty() && !bndV3Written) {
        std::vector<int> v2Counts(static_cast<std::size_t>(std::max(0, options.getRound())), 0);
        std::vector<unsigned char> v2Bits;
        std::vector<int> v2Residuals;
        std::vector<pmsf::ScAnchorDef> anchDefs;
        anchDefs.reserve(g_bcV2Anchors.size());
        for (auto& ag : g_bcV2Anchors) {
            pmsf::ScAnchorDef d;
            d.v = ag.second;
            d.group = ag.first;
            anchDefs.push_back(d);
        }
        MCGAL::Vertex* seedV = (seed && seed->vertex()) ? seed->vertex() : nullptr;
        MCGAL::Vertex* seedE = (seed && seed->end_vertex()) ? seed->end_vertex() : nullptr;
        const auto baseIdx = pmsf::anchorBaseIndicesWithSeed(mesh, anchDefs, seedV, seedE);
        v2Residuals.push_back(static_cast<int>(anchDefs.size()));
        if (std::getenv("PMSF_BC_V2_DEBUG") || std::getenv("PMSF_BC_V2_ANCHORS")) {
            std::fprintf(stderr, "[bc-v2-anchors-payload] n=%zu\n", anchDefs.size());
            for (std::size_t ai = 0; ai < anchDefs.size(); ++ai) {
                const auto aq = (anchDefs[ai].v && !anchDefs[ai].v->isRemoved())
                                    ? mesh->floatPosToInt(anchDefs[ai].v->point())
                                    : MCGAL::PointInt(-1, -1, -1);
                std::fprintf(stderr, "[bc-v2-anchors-payload] i=%zu g=%d q=(%d,%d,%d) baseIdx=%d\n",
                             ai, anchDefs[ai].group, aq.x(), aq.y(), aq.z(), baseIdx[ai]);
            }
        }
        int badAnchor = 0;
        for (std::size_t k = 0; k < anchDefs.size(); ++k) {
            if (baseIdx[k] < 0) ++badAnchor;
            v2Residuals.push_back(anchDefs[k].group);
            v2Residuals.push_back(baseIdx[k]);
        }
        std::size_t nonEmpty = 0;
        for (std::size_t r = 0; r < g_bcV2Rounds.size() && r < v2Counts.size(); ++r) {
            const auto& rr = g_bcV2Rounds[r];
            if (rr.ops.empty() || rr.edgeCount <= 0) continue;
            ++nonEmpty;
            v2Counts[r] = rr.edgeCount;
            v2Bits.insert(v2Bits.end(), rr.bits.begin(), rr.bits.end());
            const int marker = (r < g_scRoundOrdering.size()) ? g_scRoundOrdering[r] : 0;
            v2Residuals.push_back(marker);
            for (const auto& op : rr.ops) {
                v2Residuals.push_back(op.kA);
                v2Residuals.push_back(op.kB);
                v2Residuals.push_back(op.rq[0]);
                v2Residuals.push_back(op.rq[1]);
                v2Residuals.push_back(op.rq[2]);
                v2Residuals.push_back(op.orient);
            }
        }
        boundaryRoundBitCounts = std::move(v2Counts);
        boundaryRoundBits = std::move(v2Bits);
        boundaryRoundResiduals = std::move(v2Residuals);
        serializer->setBoundaryBitPayload(boundaryRoundBitCounts, boundaryRoundBits,
                                          boundaryRoundResiduals, 70);
        std::fprintf(stderr,
                     "[bc-v2-payload] anchors=%zu badAnchor=%d nonEmptyRounds=%zu bits=%zu "
                     "residualInts=%zu\n",
                     anchDefs.size(), badAnchor, nonEmpty, boundaryRoundBits.size(),
                     boundaryRoundResiduals.size());
    }
    if (envOn("PMSF_EDGEFLIP_COORD_PAYLOAD") && !envOn("PMSF_EDGEFLIP_BITS_ONLY")) {
        if (!edgeFlipCoordSymbols.empty()) {
            serializer->setEdgeFlipPayload(edgeFlipRoundCounts, edgeFlipCoordSymbols);
        } else {
            serializer->setEdgeFlipPayload(edgeFlipRoundCounts, edgeFlipSymbols);
        }
    }
    // ---------- 14c) P18.2 路线 (a) 的落点勘察：**基网分区** ----------
    //
    // 走到这里 `mesh` **就是基网**（轮循环已全部跑完），也正是解码端
    // `deserializeBaseMesh()` 之后手上的那个网格对象。所以这里是路线 (a)
    // 「两侧各自重算 seeds/groupId」唯一可能的公共对象 ——
    // 而 **step 10 的分区跑的是全网格**（实测 34,834 顶点），
    // 与这里的基网（实测 7,081 顶点）不是同一个东西：
    //
    //     编码端 step 10 : 34,834 顶点（全网格）
    //     基网（此处）    :  7,081 顶点   ← 解码端 round 1 手上的
    //
    // 本块先只做**勘察**：在真实的管线内、在正确的时刻，对基网跑一次
    // `segment()`，报出组数与未赋值面数。目的是验证「编码端在这里能拿到
    // 基网并分区」这一前提**在真实代码路径上**成立（此前的验证用的是
    // 单独导出的网格，不在管线内）。默认关闭，零行为影响。
    //
    // 注意：本块**不改变**任何已收集的符号 —— 那是路线 (a) 的两遍改造
    // 才能做的事（见 docs/refactor/P18.2_PARTITION_STAGE_ATTRIBUTION.md 附 8）。
    // 触发条件有两个：写表（路线 (a) 第一遍）或勘察探针（env）。
    // 两者共用同一次 `segment()` + 回填 —— 跑两次会让第二次从被第一次
    // 弄脏的 processed 标志出发（附 9a 的重置实验证明那会让 unassigned≠0）。
    const bool segTableOut = (cfg.segmentGroupTableOut != nullptr);
    const bool segBaseProbe = (std::getenv("PMSF_SEG_BASE_MESH") != nullptr);
    if (cfg.segmentOperatorFactory && (segTableOut || segBaseProbe)) {
        if (auto baseSeg = cfg.segmentOperatorFactory()) {
            std::shared_ptr<MCGAL::Mesh> baseMesh = mesh;
            baseSeg->segment(baseMesh);
            std::unordered_set<int> groups;
            int unassigned = 0;
            for (MCGAL::Facet* f : mesh->faces()) {
                if (f->groupId() < 0) {
                    ++unassigned;
                } else {
                    groups.insert(f->groupId());
                }
            }
            // 附 11 声称「`contextPool.operations` 已保留坍缩记录、可用」。
            // 那是我**读源码**得出的结论 —— 本阶段读源码得出的结论错过三次，
            // 故这里**实测**一次，下一轮才敢据它设计传播。
            std::size_t opLevels = MCGAL::contextPool.operations.size();
            std::size_t opTotal = 0;
            std::size_t opWithVr = 0;
            for (const auto& lvl : MCGAL::contextPool.operations) {
                opTotal += lvl.size();
                for (const auto& kv : lvl) {
                    if (kv.second.vr != nullptr) ++opWithVr;
                }
            }
            std::fprintf(stderr,
                         "[SEG_BASE] base_mesh_facets=%zu base_mesh_vertices=%zu "
                         "groups=%zu seeds=%zu unassigned=%d\n",
                         static_cast<std::size_t>(mesh->size_of_facets()),
                         static_cast<std::size_t>(mesh->size_of_vertices()),
                         groups.size(), baseSeg->exportSeeds().size(), unassigned);
            std::fprintf(stderr,
                         "[SEG_OPS] operation_levels=%zu total_ops=%zu "
                         "ops_with_vr=%zu\n",
                         opLevels, opTotal, opWithVr);

            // P18.2：坍缩记录的**自检** —— 记录数 vs 实际被移除的顶点数。
            // 两者不等就说明三处 eliminate 有漏记（记录点不全），
            // 或 `isRemoved()` 判据与 eliminate 的实际行为不一致。
            std::size_t removedVerts = 0;
            for (MCGAL::Vertex* v : mesh->vertices()) {
                if (v->isRemoved()) ++removedVerts;
            }
            std::fprintf(stderr,
                         "[SEG_COLLAPSE] recorded=%zu removed_vertices=%zu %s\n",
                         segCollapseMap.size(), removedVerts,
                         (segCollapseMap.size() == removedVerts) ? "MATCH" : "MISMATCH");
            std::fflush(stderr);

            // P18.2 路线 (a) 第 3 步：**消费记录做回填**。
            //
            // 基网上存活面的 groupId 是已知的（上面刚跑完 segment()）。
            // 回填 = 沿 `segCollapseMap`（被移除顶点 → 存活顶点）把
            // **被移除顶点**追到某个存活顶点，从而拿到它所属的分区。
            // 这就是两遍改造里「把基网分组映射回全网格」那一步。
            //
            // 自检：**覆盖率** —— 若某个被移除顶点沿链走不到任何已分区顶点，
            // 回填就会漏，表现为「部分面没有 groupId」。
            std::unordered_map<int, int> vertGroup;  // 存活顶点 → 所属分区
            for (MCGAL::Facet* f : mesh->faces()) {
                if (f->isRemoved()) continue;
                for (MCGAL::Vertex* v : f->getVertices()) {
                    if (v != nullptr) vertGroup[v->poolId()] = f->groupId();
                }
            }

            std::size_t resolved = 0;
            std::size_t unresolved = 0;
            std::size_t chainTotal = 0;
            for (const auto& kv : segCollapseMap) {
                int cur = kv.second;  // 从存活端出发
                bool found = false;
                for (int guard = 0; guard < 64; ++guard) {
                    if (vertGroup.count(cur) != 0) {
                        found = true;
                        chainTotal += static_cast<std::size_t>(guard);
                        break;
                    }
                    auto nx = segCollapseMap.find(cur);
                    if (nx == segCollapseMap.end()) break;
                    cur = nx->second;
                }
                if (found) {
                    ++resolved;
                } else {
                    ++unresolved;
                }
            }
            std::fprintf(stderr,
                         "[SEG_BACKFILL] vertices_with_group=%zu resolved=%zu "
                         "unresolved=%zu avg_chain=%.2f\n",
                         vertGroup.size(), resolved, unresolved,
                         resolved ? (double)chainTotal / (double)resolved : 0.0);
            std::fflush(stderr);

            // ---- 把回填结果**真正应用**到一份新载入的网格上 ----
            //
            // 这是第 3 步的最后一段：不是「算出映射」，而是「用映射给网格
            // 写上 groupId」。新网格与本次运行的网格 poolId 一一对应
            // （同输入、同确定性），故可直接按 poolId 套用。
            //
            // 自检：**每个面都必须拿到 groupId**。漏一个，两遍改造里
            // 那个面就没有分区归属 —— 这正是要在这里当场发现的事。
            for (const auto& kv : segCollapseMap) {
                int cur = kv.second;
                for (int guard = 0; guard < 64; ++guard) {
                    auto direct = vertGroup.find(cur);
                    if (direct != vertGroup.end()) {
                        vertGroup[kv.first] = direct->second;
                        break;
                    }
                    auto nx = segCollapseMap.find(cur);
                    if (nx == segCollapseMap.end()) break;
                    cur = nx->second;
                }
            }

            // ⚠️ 跨网格必须用**序号**对应，不能用 poolId。
            //
            // 实测教训：第一版直接按 poolId 套用，结果 `applied=0 missing=69664`
            // —— 第二个网格的顶点是**另一段 pool 偏移**，poolId 对不上。
            // 两份网格由同一个文件确定性载入 ⇒ 顶点**序号**才是一一对应的。
            std::unordered_map<int, int> ordGroup;  // 顶点序号 → 分区
            int ordCount = 0;
            {
                int ord = 0;
                for (MCGAL::Vertex* v : mesh->vertices()) {
                    auto it = vertGroup.find(v->poolId());
                    if (it != vertGroup.end()) ordGroup[ord] = it->second;
                    ++ord;
                }
                ordCount = ord;
            }

            // ---- 路线 (a) 第一遍的产物：**顶点序号 → 分区** 表 ----
            //
            // 这是两遍改造之间唯一的通道。长度取全网格顶点数，未归属写 -1
            // （不写 0 —— 0 是合法分区号，混同就丢了一个分区）。
            if (segTableOut) {
                auto& table = *cfg.segmentGroupTableOut;
                table.assign(static_cast<std::size_t>(ordCount), -1);
                std::size_t filled = 0;
                for (const auto& kv : ordGroup) {
                    if (kv.first < 0 || kv.first >= ordCount) continue;
                    table[static_cast<std::size_t>(kv.first)] = kv.second;
                    ++filled;
                }
                std::fprintf(stderr,
                             "[SEG_TABLE_WRITE] vertices=%d filled=%zu "
                             "unfilled=%zu groups=%zu\n",
                             ordCount, filled,
                             static_cast<std::size_t>(ordCount) - filled,
                             baseSeg->exportSeeds().size());
                std::fflush(stderr);
            }

            if (segBaseProbe) {
            std::shared_ptr<MCGAL::Mesh> freshMesh =
                std::make_shared<MCGAL::Mesh>(MCGAL::Mesh(options.getPath()));
            std::unordered_map<int, int> freshPoolToOrd;
            {
                int ord = 0;
                for (MCGAL::Vertex* v : freshMesh->vertices()) {
                    freshPoolToOrd[v->poolId()] = ord++;
                }
            }
            std::size_t applied = 0;
            std::size_t missing = 0;
            for (MCGAL::Facet* f : freshMesh->faces()) {
                int gid = -1;
                for (MCGAL::Vertex* v : f->getVertices()) {
                    if (v == nullptr) continue;
                    auto p2o = freshPoolToOrd.find(v->poolId());
                    if (p2o == freshPoolToOrd.end()) continue;
                    auto g = ordGroup.find(p2o->second);
                    if (g != ordGroup.end()) {
                        gid = g->second;
                        break;
                    }
                }
                if (gid >= 0) {
                    f->setGroupId(gid);
                    ++applied;
                } else {
                    ++missing;
                }
            }
            // 分组是否**可用**：pass 2 要拿它去喂分段 collector，
            // 空组会让那个内层 collector 无符号可收（等于丢一整段）。
            std::unordered_map<int, std::size_t> groupFacetCount;
            for (MCGAL::Facet* f : freshMesh->faces()) {
                if (f->groupId() >= 0) ++groupFacetCount[f->groupId()];
            }
            std::size_t minG = static_cast<std::size_t>(-1);
            std::size_t maxG = 0;
            std::size_t emptyGroups = 0;
            for (const auto& kv : groupFacetCount) {
                minG = std::min(minG, kv.second);
                maxG = std::max(maxG, kv.second);
            }
            for (std::size_t g = 0; g < groupFacetCount.size(); ++g) {
                if (groupFacetCount.count(static_cast<int>(g)) == 0) ++emptyGroups;
            }
            std::fprintf(stderr,
                         "[SEG_APPLY] fresh_facets=%zu applied=%zu missing=%zu %s "
                         "groups=%zu min=%zu max=%zu empty=%zu\n",
                         static_cast<std::size_t>(freshMesh->size_of_facets()), applied, missing,
                         (missing == 0) ? "FULL-COVERAGE" : "GAPS",
                         groupFacetCount.size(),
                         (minG == static_cast<std::size_t>(-1)) ? 0 : minG,
                         maxG, emptyGroups);
            std::fflush(stderr);
            }  // if (segBaseProbe)
        }
    }


    // [PHASE59 诊断，默认关闭] 把分区 seed 的量化坐标 dump 出去，
    // 供解码端探针单独验证「seed 是否就是回环失败的根因」。

    if (std::getenv("PMSF_SEAM_SHARED_BASE") && std::getenv("PMSF_SEAM_COMPRESS")) {
        if (std::getenv("PMSF_SEAM_ROUNDTRIP_TEST")) {
            auto rtShared = pmsf::SeamMergeOperator::mergeByPosition(mesh, true);
            if (rtShared) {
                auto rtCut = pmsf::SeamCutOperator::run(rtShared, {});
                if (rtCut.mesh) rtCut.mesh->dumpto_oldtype("/tmp/shared_roundtrip.off");
            }
        }
        auto shared = pmsf::SeamMergeOperator::mergeByPosition(mesh, true);
        if (shared) {
            // Remap the serialization seed from cut mesh to shared base by position.
            MCGAL::Halfedge* remapped = nullptr;
            if (seed && seed->vertex() && seed->end_vertex()) {
                MCGAL::Halfedge* found0 = nullptr;
                MCGAL::Halfedge* found1 = nullptr;
                MCGAL::PointInt qa = mesh->floatPosToInt(seed->vertex()->point());
                MCGAL::PointInt qb = mesh->floatPosToInt(seed->end_vertex()->point());
                MCGAL::Vertex* va = nullptr;
                MCGAL::Vertex* vb = nullptr;
                for (MCGAL::Vertex* v : shared->vertices()) {
                    if (!v || v->isRemoved()) continue;
                    const MCGAL::PointInt q = shared->floatPosToInt(v->point());
                    if (!va && q == qa) va = v;
                    if (!vb && q == qb) vb = v;
                }
                if (va && vb) {
                    for (MCGAL::Halfedge* h : pmsf::boundedRing(va)) {
                        if (h && !h->isRemoved() && h->end_vertex() == vb) { remapped = h; break; }
                    }
                }
                (void)found0; (void)found1;
            }
            if (remapped) {
                mesh = shared;
                seed = remapped;
            }
        }
    }

    if (std::getenv("PMSF_BOUNDARY_DEBUG") && !g_sharedPatchFillFaces.empty()) {
        int foundFill = 0;
        for (const auto& tri : g_sharedPatchFillFaces) {
            std::array<std::array<int,3>,3> want{};
            for (int k = 0; k < 3; ++k) want[static_cast<size_t>(k)] = {tri[static_cast<size_t>(3*k)], tri[static_cast<size_t>(3*k+1)], tri[static_cast<size_t>(3*k+2)]};
            std::sort(want.begin(), want.end());
            for (MCGAL::Facet* f : mesh->faces()) {
                if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                std::array<std::array<int,3>,3> have{};
                int n = 0;
                MCGAL::Halfedge* h0 = f->proxyHalfedge(); MCGAL::Halfedge* hh = h0; int guard = 0;
                do {
                    if (hh && hh->vertex() && n < 3) {
                        auto q = mesh->floatPosToInt(hh->vertex()->point());
                        have[static_cast<size_t>(n++)] = {q.x(), q.y(), q.z()};
                    }
                    hh = hh->next();
                } while (hh && hh != h0 && ++guard < 64);
                if (n != 3) continue;
                std::sort(have.begin(), have.end());
                if (have == want) { ++foundFill; break; }
            }
        }
        std::cerr << "[shared-boundary-patch-base] storedFill=" << g_sharedPatchFillFaces.size()
                  << " foundInBase=" << foundFill << std::endl;
        mesh->dumpto_oldtype("/tmp/shared_patch_base.off");
    }

    serializer->serializeBaseMeshWithSeed(mesh, {seed});
    serializer->serialize(options.getOutputPath());
    t_serialize += now() - t0;

    // B.1 Sprint 2 probe：触发 probe 的 summary 打印（复用其 exportToBufferWithRangeCoder
    // 里的 PMSF_RD_DIAG 代码）。buffer 不被使用，可以传 nullptr（RDSymbolCollectOperator
    // 实现已保证 exportTo* 不访问 buffer）。
    if (probeCollector) {
        probeCollector->exportToBufferWithRangeCoder(nullptr, false);
    }

    if (profileOn) {
        auto t_total = now() - t_total_start;
        std::fprintf(stderr,
            "[PROFILE] load=%.1fms preInit=%.1f sel_init=%.1f elim_init=%.1f "
            "preprocess=%.1f postproc=%.1f "
            "select=%.1f eliminate=%.1f update=%.1f "
            "collect=%.1f sel_reset=%.1f serialize=%.1f total=%.1f\n",
            ms(t_load), ms(t_preInit), ms(t_selectInit), ms(t_eliminateInit),
            ms(t_preprocess), ms(t_postprocess),
            ms(t_selectLoop), ms(t_eliminateLoop), ms(t_updateLoop),
            ms(t_collectLoop), ms(t_selectReset), ms(t_serialize),
            ms(t_total));
    }

    // [perf 诊断, env PMSF_TEARDOWN_DIAG] profile 之后仍有 ~29 ms 才返回：逐个
    // 显式释放大对象，定位到底是谁的析构（保持默认行为不变，仅诊断路径提前 reset）。
}

// -----------------------------------------------------------------------
// runDecompress —— 通用解压主循环
// -----------------------------------------------------------------------

namespace {
// Mirror of pmsf::findCanonicalGroupHalfedgeInMesh (impl/common/SegmentedRangeCodedDecode.cpp)
// used only to align encoder per-group seeds with the decoder's canonical fallback.
inline MCGAL::Halfedge* findCanonicalGroupSeedLocal(MCGAL::Mesh* mesh, int groupId) {
    if (!mesh || groupId < 0) return nullptr;
    MCGAL::Halfedge* best = nullptr;
    std::array<double, 6> bestKey{};
    auto qkey = [&](MCGAL::Halfedge* h) -> std::array<double, 6> {
        if (mesh->enableQuantization) {
            MCGAL::PointInt a = mesh->floatPosToInt(h->vertex()->point());
            MCGAL::PointInt b = mesh->floatPosToInt(h->end_vertex()->point());
            return {static_cast<double>(a.x()), static_cast<double>(a.y()),
                    static_cast<double>(a.z()), static_cast<double>(b.x()),
                    static_cast<double>(b.y()), static_cast<double>(b.z())};
        }
        const MCGAL::Point& a = h->vertex()->point();
        const MCGAL::Point& b = h->end_vertex()->point();
        return {a.x(), a.y(), a.z(), b.x(), b.y(), b.z()};
    };
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge() || f->groupId() != groupId) continue;
        MCGAL::Halfedge* h = f->proxyHalfedge();
        MCGAL::Halfedge* start = h;
        int guard = 0;
        do {
            if (h && !h->isRemoved() && h->vertex() && h->end_vertex()) {
                const std::array<double, 6> key = qkey(h);
                if (!best || key < bestKey) { best = h; bestKey = key; }
            }
            h = h ? h->next() : nullptr;
        } while (h && h != start && ++guard <= 8192);
    }
    return best;
}
}  // namespace

namespace {
inline MCGAL::Halfedge* findLiveSeedBetweenLocal(MCGAL::Mesh* mesh,
                                                MCGAL::Vertex* v0,
                                                MCGAL::Vertex* v1,
                                                int groupId) {
    if (!mesh) return nullptr;
    auto halfedgeMatches = [&](MCGAL::Halfedge* h, MCGAL::Vertex* a, MCGAL::Vertex* b) {
        if (!h || h->isRemoved() || !h->vertex() || !h->end_vertex()) return false;
        if (groupId >= 0 && h->face() && !h->face()->isRemoved() &&
            h->face()->groupId() != groupId)
            return false;
        return (!a || h->vertex() == a) && (!b || h->end_vertex() == b);
    };
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        if (groupId >= 0 && f->groupId() != groupId) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (halfedgeMatches(h, v0, v1)) return h;
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
    }
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        if (groupId >= 0 && f->groupId() != groupId) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (halfedgeMatches(h, v0, nullptr)) return h;
            if (halfedgeMatches(h, v1, nullptr)) return h;
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
    }
    return nullptr;
}
}  // namespace

std::shared_ptr<MCGAL::Mesh> runDecompress(
    DecompressOptions& options,
    std::function<void(const PMSFHeader&,
                       std::shared_ptr<DeserializeOperator>,
                       std::shared_ptr<MCGAL::Mesh>)>
        afterBaseMeshHook) {
    pmsf::bndid::decEnable();
    pmsf::bndid::decReset();
    if (const char* dbgAlarm = std::getenv("PMSF_DEBUG_ALARM")) {
        std::signal(SIGALRM, [](int) { std::abort(); });
        ::alarm(static_cast<unsigned>(std::max(1, std::atoi(dbgAlarm))));
    }
    auto deserializer = options.getDeserializeOperator();
    deserializer->init(options.getPath());

    // Columnar init() 已在内部填好 header_；Binary init() 只填 buffer_。
    // header() 默认实现返回 header_，若 round==0 说明尚未被 parquetV2 的 init 填上，
    // 走 binary 的 deserializeHeader(buffer_) 路径。两种路径产出的 header 语义一致。
    PMSFHeader header = deserializer->header();
    if (header.getRound() == 0) {
        deserializer->deserializeHeader(header);
    }

    // 池容量按原始网格规模注册（metadata 的 total_vertices/total_faces；
    // 读不到的后端回落内置默认）。必须在 deserializeBaseMesh() 之前 ——
    // 基网解码是第一个真正从池分配的动作。
    int origV = 0, origF = 0;
    deserializer->originalMeshCounts(origV, origF);
    if (origV > 0 && origF > 0) {
        ensureMeshRuntimeInitializedForMeshSize(origV, origF);
    } else {
        ensureMeshRuntimeInitialized();
    }

    std::shared_ptr<MCGAL::Mesh> mesh = deserializer->deserializeBaseMesh();
    // Self-describing mode: mode71 payload enables the ID registry without env.
    if (deserializer && deserializer->isBoundaryV3Payload()) pmsf::bndid::decEnable(true);
    if (mesh) pmsf::bndid::decInitFromMesh(mesh.get());
    if (mesh && std::getenv("PMSF_SEAM_INFER_VGID") && std::getenv("PMSF_SEAM_COMPRESS")) {
        const int inferChanged = pmsf::inferVertexGroupsFromFaces(mesh);
        if (std::getenv("PMSF_SEAM_DEBUG"))
            std::cerr << "[seam-infer-vgid] changed=" << inferChanged << std::endl;
    }
    if (pmsf::hasCompactBoundaryPayloadForVerify() && mesh) {
        const auto vst = pmsf::verifyCompactBoundaryPayload(mesh);
        std::fprintf(stderr,
                     "[boundary-compact-base-verify] segments=%d verified=%d failed=%d "
                     "anchorMissing=%d walkMismatch=%d fallback=%d\n",
                     vst.segments, vst.verified, vst.failed, vst.anchorMissing,
                     vst.walkMismatch, vst.fallbackSegments);
    }


    if (afterBaseMeshHook) {
        afterBaseMeshHook(header, deserializer, mesh);
    }

    const bool dumpPipelineRounds =
        envOn("PMSF_PIPELINE_DEBUG_DUMP") || envOn("PMSF_EDGEFLIP_DEBUG_DUMP");
    if (dumpPipelineRounds) {
        std::filesystem::create_directories("build/aaaaresult/decompress/rounds");
    }
    // max_lod 谓词：解码跑到第 k 轮为止。各轮在产物内首尾相接（见
    // tests/test_truncation_prefix.cpp A5），所以「跑前 k 轮」读的正是前缀字节，
    // 且解出的网格 = 全量解码在第 k 轮结束时的中间状态 —— 这就是「截断即前缀」。
    int round = header.getRound();
    const int lodCap = deserializer->queryMaxLod();
    if (lodCap >= 0 && lodCap < round) {
        round = lodCap;
    }
    for (int ri = 0; ri < round; ++ri) {
        char beforePath[512];
        if (dumpPipelineRounds) {
            std::snprintf(beforePath, sizeof(beforePath),
                          "build/aaaaresult/decompress/rounds/round_%03d_before.off", ri);
            mesh->dumpto_oldtype(beforePath);
        }
        // 复位上一轮残留的元素状态。用框架版的完整复位而不是
        // MCGAL::Mesh::resetState()：后者只覆盖「活着的面」触及的元素，已移除的
        // 面包裹的元素不复位，它们的 addedFlag_/splittableFlag_ 跨轮残留，下一轮
        // 解码 BFS 再走到就撞 assert。详见 MeshRuntime.h 的说明。
        pmsf::bndid::decSetRound(round - 1 - ri);
        pmsf::resetRoundElementState(mesh);
        deserializer->deserializeOneRound();
        mesh = deserializer->exportMesh();
        if (std::getenv("PMSF_SEAM_CANON_ROUNDS") && std::getenv("PMSF_SEAM_COMPRESS") && mesh)
            pmsf::canonicalizeMeshOrder(mesh);
        if (std::getenv("PMSF_SEAM_BOUNDARY_GCDEC") && std::getenv("PMSF_SEAM_COMPRESS") && mesh)
            mesh->garbage_collection();
        if (std::getenv("PMSF_DEC_ROUND_VALIDATE") && mesh) {
            int nonTri = 0, dangling = 0, dup = 0;
            std::vector<std::vector<MCGAL::PointInt>> badFaces;
            std::map<std::array<int, 9>, int> seen;
            for (MCGAL::Facet* f : mesh->faces()) {
                if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                std::vector<MCGAL::PointInt> qs;
                MCGAL::Halfedge* h0 = f->proxyHalfedge();
                MCGAL::Halfedge* h = h0;
                int guard = 0;
                do {
                    if (h && h->vertex()) {
                        if (h->vertex()->isRemoved()) ++dangling;
                        qs.push_back(mesh->floatPosToInt(h->vertex()->point()));
                    }
                    h = h->next();
                } while (h && h != h0 && ++guard < 64);
                if (qs.size() != 3) {
                    ++nonTri;
                    if (badFaces.size() < 5) badFaces.push_back(qs);
                    continue;
                }
                std::array<int, 9> key{};
                for (int k = 0; k < 3; ++k) {
                    key[3*k] = qs[k].x(); key[3*k+1] = qs[k].y(); key[3*k+2] = qs[k].z();
                }
                std::sort(key.begin(), key.end());
                if (++seen[key] > 1) ++dup;
            }
            std::cerr << "[dec-round-validate] ri=" << ri << " V=" << mesh->size_of_vertices()
                      << " F=" << mesh->size_of_facets() << " nonTri=" << nonTri
                      << " dangling=" << dangling << " dup=" << dup << std::endl;
            for (size_t bi = 0; bi < badFaces.size(); ++bi) {
                std::cerr << "  bad" << bi << ":";
                for (const auto& q : badFaces[bi]) std::cerr << " (" << q.x() << "," << q.y() << "," << q.z() << ")";
                std::cerr << std::endl;
            }
        }
        if (std::getenv("PMSF_SEAM_INFER_VGID_ROUNDS") && std::getenv("PMSF_SEAM_COMPRESS") && mesh)
            pmsf::inferVertexGroupsFromFaces(mesh);
                char afterPath[512];
        if (dumpPipelineRounds) {
            std::snprintf(afterPath, sizeof(afterPath),
                          "build/aaaaresult/decompress/rounds/round_%03d_after.off", ri);
            mesh->dumpto_oldtype(afterPath);
        }
    }

    // Decoupled boundary schedule: apply all boundary inverses now (after every
    // interior round has been undone), in reverse encode-round order.
    if (deserializer && deserializer->isBoundaryV3Payload()) {
        deserializer->finalizeBoundaryInverses(round);
        mesh = deserializer->exportMesh();
    }

    if (envOn("PMSF_BOUNDARY_MODULE") && !options.getPath().empty()) {
        std::vector<int> boundaryPayload;
        const std::string bndPath =
            pmsf::BoundaryCompressionModule::sectionPathForArtifact(options.getPath());
        if (pmsf::BoundaryCompressionModule::readPayloadSection(bndPath, boundaryPayload)) {
            const bool applied = pmsf::applyBoundaryInversePayload(mesh, boundaryPayload);
            if (std::getenv("PMSF_BOUNDARY_DEBUG"))
                std::fprintf(stderr, "[boundary-module] inverse payload=%zu applied=%d path=%s\n",
                             boundaryPayload.size(), applied ? 1 : 0, bndPath.c_str());
        }
    }

    if (envOn("PMSF_REPAIR_DEGENERATE")) {
        const int repaired = repairDegenerateTriangleGeometry(mesh);
        if (repaired > 0) {
            std::fprintf(stderr,
                         "[runDecompress] repaired_degenerate_faces=%d\n",
                         repaired);
        }
    }

    if (std::getenv("PMSF_SEAM_DEBUG"))
        std::cerr << "[seam-merge-begin] V=" << mesh->size_of_vertices()
                  << " F=" << mesh->size_of_facets() << std::endl;
    if (std::getenv("PMSF_SEAM_MERGE")) {
        auto merged = pmsf::SeamMergeOperator::mergeByPosition(mesh);
        if (std::getenv("PMSF_SEAM_DEBUG"))
            std::cerr << "[seam-merge-end]" << std::endl;
        if (!merged) {
            std::cerr << "[seam-merge] merge failed" << std::endl;
            std::exit(1);
        }
        if (std::getenv("PMSF_SEAM_DEBUG")) {
            std::cerr << "[seam-merge] preV=" << mesh->size_of_vertices()
                      << " preF=" << mesh->size_of_facets()
                      << " mergedV=" << merged->size_of_vertices()
                      << " mergedF=" << merged->size_of_facets() << std::endl;
        }
        mesh = merged;
    }

    // Paper-specific reconstruction postprocessor (explicit opt-in only).
    if (const char* pp = std::getenv("PMSF_PAPER_POSTPROCESS"); pp && *pp) {
        pmsf::PaperPostProcessOperator::apply(mesh, pp);
    }

    if (!options.getOutputPath().empty()) {
        mesh->dumpto_oldtype(options.getOutputPath());
    }
    return mesh;
}

// ---------------------------------------------------------------------------
// Phase 5: CompressPipeline::Builder —— 流畅接口 fold 到 CompressPipelineConfig
// ---------------------------------------------------------------------------

CompressPipeline::Builder::Builder() = default;

CompressPipeline::Builder& CompressPipeline::Builder::withBackend(Backend b) {
    backend_ = b;
    switch (b) {
        case Backend::Columnar: base_ = CompressPipelineConfig::defaultColumnar(); break;
        case Backend::Unified: base_ = CompressPipelineConfig::defaultUnified(); break;
        case Backend::Paper: base_ = CompressPipelineConfig::defaultPaper(); break;
        case Backend::Binary:
        default:                 base_ = CompressPipelineConfig::defaultBinary();    break;
    }
    return *this;
}

CompressPipeline::Builder& CompressPipeline::Builder::withCollectorFactory(
    CompressPipelineConfig::CollectorFactory f) {
    base_.collectorFactory = std::move(f);
    return *this;
}

CompressPipeline::Builder& CompressPipeline::Builder::withSerializerFactory(
    CompressPipelineConfig::SerializerFactory f) {
    base_.serializerFactory = std::move(f);
    return *this;
}

CompressPipeline::Builder& CompressPipeline::Builder::withPreInitPreprocessor(
    std::shared_ptr<PreProcessOperator> op) {
    if (op) preInitPreprocessors_.push_back(std::move(op));
    return *this;
}

CompressPipeline::Builder& CompressPipeline::Builder::withPreprocessor(
    std::shared_ptr<PreProcessOperator> op) {
    if (op) preprocessors_.push_back(std::move(op));
    return *this;
}

CompressPipeline::Builder& CompressPipeline::Builder::withRoundPreprocessor(
    std::shared_ptr<RoundPreProcessor> op) {
    if (op) roundPreprocessors_.push_back(std::move(op));
    return *this;
}

CompressPipeline::Builder& CompressPipeline::Builder::withUpdateAfterEliminate(bool v) {
    base_.updateAfterEliminate = v;
    return *this;
}
CompressPipeline::Builder& CompressPipeline::Builder::withEnableIndependentSet(bool v) {
    base_.enableIndependentSet = v;
    return *this;
}
CompressPipeline::Builder& CompressPipeline::Builder::withRecomputeVertexDegreesOnce(bool v) {
    base_.recomputeVertexDegreesOnce = v;
    return *this;
}
CompressPipeline::Builder& CompressPipeline::Builder::withHeaderCarryPrediction(bool v) {
    base_.headerCarryPrediction = v;
    return *this;
}
CompressPipeline::Builder& CompressPipeline::Builder::withSeedStrategy(
    CompressPipelineConfig::SeedStrategy s) {
    base_.seedStrategy = s;
    return *this;
}
CompressPipeline::Builder& CompressPipeline::Builder::withCustomSeedSelector(
    std::function<MCGAL::Halfedge*(std::shared_ptr<MCGAL::Mesh>)> f) {
    base_.customSeedSelector = std::move(f);
    return *this;
}
CompressPipeline::Builder& CompressPipeline::Builder::withBeforeCompressLoop(
    decltype(CompressPipelineConfig::beforeCompressLoop) f) {
    base_.beforeCompressLoop = std::move(f);
    return *this;
}
CompressPipeline::Builder& CompressPipeline::Builder::withAfterRound(
    decltype(CompressPipelineConfig::afterRound) f) {
    base_.afterRound = std::move(f);
    return *this;
}
CompressPipeline::Builder& CompressPipeline::Builder::withBeforeSerialize(
    decltype(CompressPipelineConfig::beforeSerialize) f) {
    base_.beforeSerialize = std::move(f);
    return *this;
}

// ---- Phase 10/13+ 算法 hook 注入 ----
CompressPipeline::Builder& CompressPipeline::Builder::withErrorMetric(std::shared_ptr<ErrorMetric> m) {
    base_.errorMetric = std::move(m);
    return *this;
}
CompressPipeline::Builder& CompressPipeline::Builder::withPredictor(std::shared_ptr<Predictor> p) {
    base_.predictor = std::move(p);
    return *this;
}
CompressPipeline::Builder& CompressPipeline::Builder::withEntropyCoder(std::shared_ptr<EntropyCoder> e) {
    base_.entropyCoder = std::move(e);
    return *this;
}
CompressPipeline::Builder& CompressPipeline::Builder::withCodebook(std::shared_ptr<Codebook> c) {
    base_.codebook = std::move(c);
    return *this;
}
CompressPipeline::Builder& CompressPipeline::Builder::withFeatureDetector(std::shared_ptr<FeatureDetector> f) {
    base_.featureDetector = std::move(f);
    return *this;
}

// Phase A.5: declarative Capability-set setters.
CompressPipeline::Builder& CompressPipeline::Builder::withCapability(Capability c) {
    base_.capabilities.add(c);
    return *this;
}
CompressPipeline::Builder& CompressPipeline::Builder::withCapabilities(CapabilitySet s) {
    base_.capabilities |= s;
    return *this;
}

CompressPipelineConfig CompressPipeline::Builder::build() const {
    CompressPipelineConfig cfg = base_;

    // Fold preInitPreprocessors 到 preInitPreparation 链
    if (!preInitPreprocessors_.empty()) {
        auto prev = cfg.preInitPreparation;
        auto ops = preInitPreprocessors_;
        cfg.preInitPreparation = [prev, ops](std::shared_ptr<MCGAL::Mesh> mesh,
                                              PMSFHeader& header,
                                              const CompressOptions& options) {
            for (auto& op : ops) op->process(mesh);
            if (prev) prev(mesh, header, options);
        };
    }

    // Fold preprocessors 到 beforeCompressLoop 链
    if (!preprocessors_.empty()) {
        auto prev = cfg.beforeCompressLoop;
        auto ops = preprocessors_;
        cfg.beforeCompressLoop = [prev, ops](std::shared_ptr<MCGAL::Mesh> mesh,
                                              PMSFHeader& header,
                                              std::shared_ptr<SymbolCollectOperator> collector,
                                              const CompressOptions& options) {
            for (auto& op : ops) op->process(mesh);
            if (prev) prev(mesh, header, collector, options);
        };
    }

    // Fold roundPreprocessors 到 beforeRound 链
    if (!roundPreprocessors_.empty()) {
        auto prev = cfg.beforeRound;
        auto ops = roundPreprocessors_;
        cfg.beforeRound = [prev, ops](int i, std::shared_ptr<MCGAL::Mesh> mesh) {
            for (auto& op : ops) op->beforeRound(i, mesh);
            if (prev) prev(i, mesh);
        };
        cfg.trackContextLevel = true;  // ContextLevelTracking 自动启用 meta
    }

    return cfg;
}

}  // namespace pmsf
