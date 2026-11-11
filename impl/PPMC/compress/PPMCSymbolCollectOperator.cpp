/*
 * @Author: koi
 * @Date: 2025-08-29 21:13:56
 * @Description:
 */

#include "PPMC/compress/PPMCSymbolCollectOperator.h"
#include "common/BoundedRingTraversal.h"
#include "PMSFPayloadAdapterRegistry.h"
#include "storage/PMSFSymbolStorageAdapter.h"
#include "common/DebugTool.h"
#include "compress/RangeCoderStreamEncoder.h"
#include "BFSVersionManager.h"
#include "BufferUtils.h"
#include "Configuration.h"
#include "Facet.h"
#include "Global.h"
#include "Halfedge.h"
#include "MeshUtils.h"
#include "Point.h"
#include "core.h"
#include "predict/FrenetResidualCodec.h"
#include "PPMC/PPMCFrenetRotation.h"
#include <algorithm>
#include <cassert>
#include <cmath>   // P2.a 对齐：long double overloads of fabs / std::fabs
#include <cstring>
#include <cstddef>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <zlib.h>

#define INV_ALPHA 2

// perf（2026-11-08 bunny 实测）：PMSF_PPMC_TPROF 诊断检查原先在
// determineGeometrySym 里**每个几何符号一次 getenv**（34,712 次/次压缩），
// 缓存成进程内常量；诊断用法不变（启动前设 env）。
inline bool ppmcTprofEnabled() {
    static const bool enabled = [] {
        const char* e = std::getenv("PMSF_PPMC_TPROF");
        return e && *e && *e != '0';
    }();
    return enabled;
}

// [perf 诊断, env PMSF_PPMC_TPROF=1] determineGeometrySym 子阶段累计耗时(ms)/次数。
static double gPpmcTFrame = 0.0, gPpmcTLap = 0.0, gPpmcTRot = 0.0, gPpmcTResidualStore = 0.0;
static long gPpmcNGeom = 0;

namespace {

static int ppmcQsmodelRescale() {
    const char* e = std::getenv("PMSF_PPMC_QS_RESCALE");
    if (e && e[0]) {
        int v = std::atoi(e);
        if (v >= 6 && v <= 20) return 1 << v;
    }
    return 1 << 17;
}

// ─── P3.b 浮点判据 helpers（回退到原版 PPMC mymeshComp.cpp 的 fabs 比较） ─────
// 编/解码两端走完全相同浮点代码 + 同一 BFS 顺序，结果 bit-identical。

bool ppmcMultiComponentSymbols() {
    const char* env = std::getenv("PMSF_PPMC_MULTI_COMPONENT_SYMBOLS");
    return env && *env && std::string(env) != "0";
}

bool ppmcSymbolDebugLogs() {
    static const bool enabled = [] {
        const char* env = std::getenv("PMSF_SEGMENTED_PPMC_DEBUG");
        return env && *env && std::string(env) != "0";
    }();
    return enabled;
}

bool ppmcEnvOn(const char* name) {
    return pmsf::pmsfStorageEnvEnabled(name);
}

void requirePPMCPaperExactIfRequested() {
    if (!pmsf::boundaryFlagOn(pmsf::ppmc_boundary::kPaperExactSpec)) return;
    throw std::runtime_error(
        "[PPMC] requested paper-exact replay, but curvature prediction is "
        "still disabled/fail-fast until the encoder and decoder formulas are "
        "proved symmetric.");
}

struct PPMCEdgeTraversalDebugStats {
    std::size_t edgeVisits = 0;
    std::size_t coded = 0;
    std::size_t noCode = 0;
    std::size_t noCodeBothUnsplit = 0;
    std::size_t noCodeBoundaryUnsplit = 0;
    std::size_t sameGroupOpp = 0;
    std::size_t noSameGroupOpp = 0;
    std::size_t original = 0;
    std::size_t inserted = 0;
    std::size_t boundaryEnqueues = 0;
    std::size_t componentSeeds = 0;
};

struct PPMCFaceTraversalDebugStats {
    std::size_t faceVisits = 0;
    std::size_t split = 0;
    std::size_t unsplit = 0;
    std::size_t residuals = 0;
    std::size_t componentSeeds = 0;
    std::uint64_t sequenceHash = 1469598103934665603ull;
};

unsigned faceDegreeFromGate(MCGAL::Halfedge* gate);

int ppmcTraceLimit() {
    static const int limit = [] {
        const char* env = std::getenv("PMSF_SEGMENTED_PPMC_TRACE_LIMIT");
        if (!env || !*env) return 0;
        return std::max(0, std::atoi(env));
    }();
    return limit;
}

std::uint64_t mixDebugHash(std::uint64_t h, std::uint64_t v) {
    h ^= v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
    h *= 1099511628211ull;
    return h;
}

std::uint64_t signedCoordBits(int v) {
    return static_cast<std::uint64_t>(static_cast<std::int64_t>(v)) ^
           0x8000000000000000ull;
}

std::uint64_t faceRingDebugHash(MCGAL::Mesh* mesh,
                                MCGAL::Halfedge* gate,
                                bool split) {
    std::uint64_t h = split ? 0x73504c4954544142ull : 0x554e53504c495454ull;
    if (!gate) return h;
    h = mixDebugHash(h, static_cast<std::uint64_t>(faceDegreeFromGate(gate)));
    h = mixDebugHash(h, gate->face() ? static_cast<std::uint64_t>(gate->face()->groupId()) : ~0ull);
    MCGAL::Halfedge* it = gate;
    int guard = 0;
    while (it && ++guard <= 4096) {
        if (it->vertex() && mesh) {
            MCGAL::PointInt q = mesh->floatPosToInt(it->vertex()->point());
            h = mixDebugHash(h, signedCoordBits(q.x()));
            h = mixDebugHash(h, signedCoordBits(q.y()));
            h = mixDebugHash(h, signedCoordBits(q.z()));
        } else {
            h = mixDebugHash(h, 0xffffffffffffffffull);
        }
        if (!it->next()) {
            h = mixDebugHash(h, 0x4f50454e5f504154ull);
            break;
        }
        it = it->next();
        if (it == gate) break;
    }
    if (guard > 4096) h = mixDebugHash(h, 0x42524f4b454eull);
    return h;
}

void traceFaceDebug(const char* side,
                    int group,
                    int op,
                    std::size_t index,
                    MCGAL::Mesh* mesh,
                    MCGAL::Halfedge* gate,
                    bool split,
                    float surface) {
    const int limit = ppmcTraceLimit();
    if (limit <= 0 || index >= static_cast<std::size_t>(limit)) return;
    std::ostringstream verts;
    MCGAL::Halfedge* it = gate;
    int guard = 0;
    while (it && ++guard <= 32) {
        if (it->vertex() && mesh) {
            MCGAL::PointInt q = mesh->floatPosToInt(it->vertex()->point());
            verts << q.x() << "," << q.y() << "," << q.z();
        } else {
            verts << "null";
        }
        if (!it->next()) {
            verts << "|open";
            break;
        }
        it = it->next();
        if (it == gate) break;
        verts << ";";
    }
    std::cerr << "[PMSF_PPMC_FACE_TRACE] " << side
              << " group=" << group
              << " op=" << op
              << " idx=" << index
              << " poolId=" << (gate ? gate->poolId() : -1)
              << " face_gid=" << (gate && gate->face() ? gate->face()->groupId() : -1)
              << " degree=" << faceDegreeFromGate(gate)
              << " split=" << (split ? 1 : 0)
              << " surface=" << surface
              << " ring_hash=" << faceRingDebugHash(mesh, gate, split)
              << " verts=" << verts.str()
              << "\n";
}

/// 面积均值预测判据：判断"当前面积 f_faceSurface 是否更接近 split 类均值"。
///   返回 true → 预测 split；false → 预测 not-split。
/// 与原版 `(avgSplit != 0 && fabs(s - avgSplit) <= fabs(s - avgUnsplit)) || avgUnsplit == 0` 一致。
inline bool predictSplitByAreaFloat(float f_faceSurface,
                                     float avgSplit, float avgUnsplit) {
    if ((avgSplit != 0.0f && std::fabs(f_faceSurface - avgSplit) <= std::fabs(f_faceSurface - avgUnsplit))
        || avgUnsplit == 0.0f)
        return true;
    return false;
}

/// 边长度均值预测判据：判断"当前 edgeLen 是否更接近 original 类均值"。
///   返回 true → 预测 original；false → 预测 inserted。
inline bool predictOriginalByLenFloat(float f_edgeLen,
                                       float avgOrig, float avgIns) {
    if ((avgOrig != 0.0f && std::fabs(f_edgeLen - avgOrig) <= std::fabs(f_edgeLen - avgIns))
        || avgIns == 0.0f)
        return true;
    return false;
}

/// Triangle-mesh 邻域 balance 预测的"两类比例接近"判据（与原版 fabs 比较语义一致）。
///   原版：|cntSplit/total - cntUnsplit/total| < 0.1f
inline bool faceClassRatiosClose(unsigned cntSplit, unsigned cntUnsplit) {
    float total = (float)(cntSplit + cntUnsplit);
    if (total == 0.0f) return false;
    return std::fabs((float)cntSplit / total - (float)cntUnsplit / total) < 0.1f;
}

inline float safeEdgeLen(MCGAL::Halfedge* h) {
    if (!h) return 0.0f;
    if (h->opposite()) return MCGAL::edgeLen(h);
    if (!h->vertex() || !h->end_vertex()) return 0.0f;
    MCGAL::Vector3 d = h->vertex()->point() - h->end_vertex()->point();
    return d.length();
}

unsigned faceDegreeFromGate(MCGAL::Halfedge* gate) {
    if (!gate) return 0;
    unsigned degree = 0;
    MCGAL::Halfedge* h = gate;
    while (h) {
        ++degree;
        if (!h->next()) {
            ++degree;
            break;
        }
        h = h->next();
        if (h == gate) break;
    }
    return degree;
}

int ppmcResidualSnapBits() {
    const char* env = std::getenv("PMSF_PPMC_RESIDUAL_SNAP_BITS");
    if (!env || !*env) return 0;
    return std::max(0, std::min(12, std::atoi(env)));
}

int snapSignedResidual(int v, int bits) {
    if (bits <= 0) return v;
    const int step = 1 << bits;
    const int half = step >> 1;
    if (v >= 0) return ((v + half) / step) * step;
    return -(((-v + half) / step) * step);
}

bool ppmcSplitZlibOperationCodecEnabled() {
    const char* env = std::getenv("PMSF_PPMC_OPERATION_CODEC");
    if (!env || !*env) return false;
    const std::string value(env);
    return value == "split_zlib" || value == "poc2";
}

void putU32LE(std::vector<char>& out, std::uint32_t v) {
    for (int i = 0; i < 4; ++i) {
        out.push_back(static_cast<char>((v >> (8 * i)) & 0xffu));
    }
}

void putU32LEToBuffer(char* out, std::size_t& off, std::uint32_t v) {
    for (int i = 0; i < 4; ++i) {
        out[off++] = static_cast<char>((v >> (8 * i)) & 0xffu);
    }
}

std::uint32_t zigzag32Local(std::int32_t v) {
    return (static_cast<std::uint32_t>(v) << 1) ^
           static_cast<std::uint32_t>(v >> 31);
}

void putVarU32(std::vector<char>& out, std::uint32_t v) {
    while (v >= 0x80u) {
        out.push_back(static_cast<char>((v & 0x7fu) | 0x80u));
        v >>= 7;
    }
    out.push_back(static_cast<char>(v));
}

void putVarI32(std::vector<char>& out, std::int32_t v) {
    putVarU32(out, zigzag32Local(v));
}

bool zlibMaybeCompress(const std::vector<char>& raw, std::vector<char>& payload) {
    payload.clear();
    if (raw.empty()) {
        payload.push_back(0);
        putU32LE(payload, 0);
        putU32LE(payload, 0);
        return true;
    }
    uLongf cap = compressBound(static_cast<uLong>(raw.size()));
    std::vector<Bytef> compressed(static_cast<std::size_t>(cap));
    uLongf compressedLen = cap;
    const int zret = compress2(compressed.data(), &compressedLen,
                               reinterpret_cast<const Bytef*>(raw.data()),
                               static_cast<uLong>(raw.size()),
                               Z_BEST_COMPRESSION);
    const bool useCompressed =
        zret == Z_OK && compressedLen < static_cast<uLongf>(raw.size());
    payload.push_back(useCompressed ? 1 : 0);
    putU32LE(payload, static_cast<std::uint32_t>(raw.size()));
    putU32LE(payload, useCompressed
                          ? static_cast<std::uint32_t>(compressedLen)
                          : static_cast<std::uint32_t>(raw.size()));
    if (useCompressed) {
        payload.insert(payload.end(),
                       reinterpret_cast<const char*>(compressed.data()),
                       reinterpret_cast<const char*>(compressed.data() + compressedLen));
    } else {
        payload.insert(payload.end(), raw.begin(), raw.end());
    }
    return true;
}
}  // namespace

bool PPMCSymbolCollectOperator::inGroup(MCGAL::Halfedge* h) const {
    if (groupFilter_ < 0) {
        return h != nullptr;
    }
    if (!h) return false;
    if (h->face()) {
        return h->face()->groupId() == groupFilter_;
    }
    return h->vertex() && h->vertex()->groupId() == groupFilter_;
}

MCGAL::Halfedge* PPMCSymbolCollectOperator::findNextUnprocessedFaceSeed() const {
    if (!mesh_) return nullptr;
    for (MCGAL::Facet* f : mesh_->faces()) {
        if (!f || f->isRemoved() || f->isProcessed() || !f->proxyHalfedge()) continue;
        if (groupFilter_ >= 0 && f->groupId() != groupFilter_) continue;
        return f->proxyHalfedge();
    }
    return nullptr;
}

MCGAL::Halfedge* PPMCSymbolCollectOperator::findNextUnprocessedHalfedgeSeed() const {
    if (!mesh_) return nullptr;
    for (MCGAL::Facet* f : mesh_->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        if (groupFilter_ >= 0 && f->groupId() != groupFilter_) continue;
        MCGAL::Halfedge* start = f->proxyHalfedge();
        MCGAL::Halfedge* h = start;
        int guard = 0;
        do {
            if (h && !h->isProcessed() && inGroup(h)) return h;
            h = h ? h->next() : nullptr;
        } while (h && h != start && ++guard <= 2048);
    }
    return nullptr;
}

void PPMCSymbolCollectOperator::collect(std::vector<MCGAL::Halfedge*> seeds) {
    collect(seeds[0]);
}

/**
 * Update the average surfaces of the faces with an removed vertex and the others.
 */
// ⚠️ P3.b 对齐（2026-05-01）：回退到原版 PPMC 浮点增量均值 `avg = (avg*(n-1) + f)/n`。
//   编/解码两端走完全相同浮点代码 + 同一 BFS 顺序 → bit-identical。
void PPMCSymbolCollectOperator::updateAvgSurfaces(bool b_split, float f_faceSurface) {
    if (b_split) {
        i_nbFacesWithCenterRemoved++;
        f_avgSurfaceFaceWithCenterRemoved = (f_avgSurfaceFaceWithCenterRemoved
                                              * (i_nbFacesWithCenterRemoved - 1) + f_faceSurface)
                                             / i_nbFacesWithCenterRemoved;
    } else {
        i_nbFacesWithoutCenterRemoved++;
        f_avgSurfaceFaceWithoutCenterRemoved = (f_avgSurfaceFaceWithoutCenterRemoved
                                                 * (i_nbFacesWithoutCenterRemoved - 1) + f_faceSurface)
                                                / i_nbFacesWithoutCenterRemoved;
    }
}

/**
 * Return the degree of a vertex without counting the new edges.
 */
inline unsigned vertexDegreeNotNew(MCGAL::Vertex* vh) {
    unsigned i_degree = 0;

    for (MCGAL::Halfedge* hit : pmsf::boundedRing(vh)) {
        if (!hit->isNew())
            i_degree++;
    }

    return i_degree;
}

/**
 * Return the average laplacian vector of the vertices of a face.
 */
MCGAL::PointInt PPMCSymbolCollectOperator::avgLaplacianVect(MCGAL::Halfedge* heh_gate) {
    // 确定性 1-ring 聚合：从活面集合一遍构建 (ΣQ(neighbour), non-new 计数)。
    // 参考实现的 1-ring 走法依赖 Vertex::proxyHalfedge 的环，而解码端重建后的
    // 顶点 proxy 可能指向重建前的旧环 → 编/解码 ring 计数/邻居不一致（±1）。
    // 用面集合构建、整数求和（顺序无关），两侧只要面集合相同结果就逐位相同。
    //
    // perf（2026-11-08）：原实现用 unordered_map<Vertex*, vector<Vertex*>>，每轮
    // 要分配数千个小 vector 并做数万次 hash；本实现按 poolId 平铺成 4 个 int 数组。
    // Laplacian 只需要 ΣQ(neighbour) 与计数，因此结果与原实现**逐位相同**。
    if (!curvatureNeighborCacheValid_) {
        const int n = static_cast<int>(mesh_->size_of_vertices());
        const int nClamped = (n < 0) ? 0 : n;
        curvSumQX_.assign(static_cast<std::size_t>(nClamped), 0);
        curvSumQY_.assign(static_cast<std::size_t>(nClamped), 0);
        curvSumQZ_.assign(static_cast<std::size_t>(nClamped), 0);
        curvCnt_.assign(static_cast<std::size_t>(nClamped), 0);
        std::vector<int> posQX(static_cast<std::size_t>(nClamped), 0);
        std::vector<int> posQY(static_cast<std::size_t>(nClamped), 0);
        std::vector<int> posQZ(static_cast<std::size_t>(nClamped), 0);
        for (MCGAL::Vertex* v : mesh_->vertices()) {
            if (!v || v->isRemoved()) continue;
            const int id = v->poolId();
            if (id < 0 || id >= nClamped) continue;
            const MCGAL::PointInt q = mesh_->floatPosToInt(v->point());
            posQX[static_cast<std::size_t>(id)] = q[0];
            posQY[static_cast<std::size_t>(id)] = q[1];
            posQZ[static_cast<std::size_t>(id)] = q[2];
        }
        for (MCGAL::Facet* f : mesh_->faces()) {
            if (!f || f->isRemoved()) continue;
            MCGAL::Halfedge* h0 = f->proxyHalfedge();
            if (!h0) continue;
            MCGAL::Halfedge* h = h0;
            int guard = 0;
            do {
                if (h && h->vertex() && h->end_vertex() && !h->isNew()) {
                    const int src = h->vertex()->poolId();
                    const int dst = h->end_vertex()->poolId();
                    if (src >= 0 && src < nClamped && dst >= 0 && dst < nClamped) {
                        const std::size_t si = static_cast<std::size_t>(src);
                        const std::size_t di = static_cast<std::size_t>(dst);
                        curvSumQX_[si] += posQX[di];
                        curvSumQY_[si] += posQY[di];
                        curvSumQZ_[si] += posQZ[di];
                        curvCnt_[si] += 1;
                    }
                }
                h = h ? h->next() : nullptr;
            } while (h && h != h0 && ++guard < 4096);
        }
        // 位置缓存即为下一阶段的 centerQ 来源；存入成员以便查询阶段使用。
        curvPosQX_ = std::move(posQX);
        curvPosQY_ = std::move(posQY);
        curvPosQZ_ = std::move(posQZ);
        curvatureNeighborCacheValid_ = true;
    }

    const int n = static_cast<int>(curvCnt_.size());
    MCGAL::Halfedge* heh = heh_gate;
    MCGAL::PointInt sumLaplacian;
    do {
        MCGAL::Vertex* center = heh->end_vertex();
        MCGAL::PointInt laplacian;
        int deg = 0;
        const int id = center ? center->poolId() : -1;
        if (id >= 0 && id < n) {
            const std::size_t si = static_cast<std::size_t>(id);
            deg = curvCnt_[si];
            if (deg > 0) {
                const int cx = curvPosQX_[si], cy = curvPosQY_[si], cz = curvPosQZ_[si];
                // Σ(Q(u) − Q(center)) ≡ ΣQ(u) − deg·Q(center)（整数加法可交换）
                laplacian = MCGAL::PointInt(curvSumQX_[si] - deg * cx,
                                            curvSumQY_[si] - deg * cy,
                                            curvSumQZ_[si] - deg * cz);
            }
        }
        const int degSafe = (deg == 0) ? 1 : deg;
        sumLaplacian = sumLaplacian + MCGAL::PointInt(laplacian[0] / degSafe,
                                                     laplacian[1] / degSafe,
                                                     laplacian[2] / degSafe);
        heh = heh->next();
    } while (heh != heh_gate);

    const int faceDeg = static_cast<int>(heh_gate->face()->facet_degree());
    return MCGAL::PointInt(sumLaplacian[0] / faceDeg, sumLaplacian[1] / faceDeg,
                           sumLaplacian[2] / faceDeg);
}

/**
 * Determine the geometry symbols.
 *
 * P0 对齐：
 *   1. 输入 `fh->getRemovedVertexPosInt()` 此时是由 `determineResiduals()` 计算好的
 *      残差 `Q(removedPos) - Q(barycenter(h))`（**不是**绝对量化坐标）；
 *   2. USE_BIJECTION 已在 Configuration.h 全局开启，必须用 MCGAL 风格的 Frenet 旋转
 *      把残差从局部 (x,y,z) 投影到 (α=t1, β=t2, γ=normal) 三轴；
 *   3. CGAL 残留符号（`Vector`, `VectorInt`, `CGAL::NULL_VECTOR`, `computeNormal(heh)`）
 *      在 PMSF 框架中并不存在，需替换为 `MCGAL::Vector3` / `MCGAL::Vector3i` /
 *      `MCGAL::computeNormalFromHeh(heh)` / `MCGAL::determineFrenetFrame(p1,p2,...)`。
 */
void PPMCSymbolCollectOperator::determineGeometrySym(MCGAL::Halfedge* heh_gate, MCGAL::Facet* fh) {
    static const bool tprof = ppmcTprofEnabled();
    // perf（2026-11-08 收敛）：诊断计时原先**无条件**每个几何符号取 5 次
    // steady_clock（34.7k 符号 ≈ 4–5 ms）。改为仅在 PMSF_PPMC_TPROF 开启时取时。
    auto tnow = [] { return std::chrono::steady_clock::now(); };
    auto tms = [](const std::chrono::steady_clock::time_point& a,
                  const std::chrono::steady_clock::time_point& b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
    };
    std::chrono::steady_clock::time_point tg0{}, tg1{}, tg2{}, tg3{}, tg4{};
    if (tprof) tg0 = tnow();
#ifdef USE_BIJECTION
    // 确定性 gate-anchored 坐标系（编/解码两侧对同一 gate 得到逐位相同的帧）。
    // D28 fix: build the frame from the integer quantization lattice so the
    // encoder and decoder frames are bit-identical (raw-float vs dequantized
    // float positions differ in the last mantissa bits and used to flip a few
    // decoded residuals after the double-precision rotation round-trip).
    pmsf::FrenetFrame frame = ppmc::ppmcHalfedgeFrameQuantized(heh_gate, mesh_.get());
#endif
    if (tprof) tg1 = tnow();

    MCGAL::PointInt distQuant;
    if (b_useCurvaturePrediction) {
        // 参考实现 §4.2 式(2)：g = r + lap(face)/INV_ALPHA（INV_ALPHA=2，
        // 分量除法按 C 语义向零截断）。avgLaplacianVect 内部两处除法必须
        // 除以真实度数（MCGAL::PointInt::operator/ 忽略除数、固定除 2）。
        MCGAL::PointInt lap = avgLaplacianVect(heh_gate);
        MCGAL::PointInt lapDiv(lap[0] / INV_ALPHA, lap[1] / INV_ALPHA, lap[2] / INV_ALPHA);
        distQuant = fh->getRemovedVertexPosInt() + lapDiv;
    } else {
        distQuant = fh->getRemovedVertexPosInt();
    }
    if (tprof) tg2 = tnow();

#ifdef USE_BIJECTION
    MCGAL::Vector3i v3i(distQuant[0], distQuant[1], distQuant[2]);
    const double fT1[3] = {static_cast<double>(frame.t1[0]), static_cast<double>(frame.t1[1]),
                           static_cast<double>(frame.t1[2])};
    const double fT2[3] = {static_cast<double>(frame.t2[0]), static_cast<double>(frame.t2[1]),
                           static_cast<double>(frame.t2[2])};
    const double fN[3] = {static_cast<double>(frame.normal[0]), static_cast<double>(frame.normal[1]),
                          static_cast<double>(frame.normal[2])};
    // 参考实现注明 "Double precision needed for the frenet rotation
    // computations"：MCGAL 的 float 版本对较大残差（曲率预测后）掉 ±1，
    // 破坏解码对称性；这里用 double 全精度双射。
    MCGAL::Vector3i frenetCoord = ppmc::frenetRotationD(v3i, fT1, fT2, fN);
#endif

    if (tprof) tg3 = tnow();
    // Store the geometry symbols.
#ifdef USE_BIJECTION
    geometrySym[i_curDecimationId].push_back(MCGAL::PointInt(frenetCoord[0], frenetCoord[1], frenetCoord[2]));
#else
    geometrySym[i_curDecimationId].push_back(distQuant);
#endif
    if (tprof) {
        tg4 = tnow();
        gPpmcTFrame += tms(tg0, tg1);
        gPpmcTLap += tms(tg1, tg2);
        gPpmcTRot += tms(tg2, tg3);
        gPpmcTResidualStore += tms(tg3, tg4);
        ++gPpmcNGeom;
    }
}

void PPMCSymbolCollectOperator::collect(MCGAL::Halfedge* seed) {
    // [perf 诊断, env PMSF_PPMC_TPROF=1] 记录 collect() 各子阶段耗时。
    static const bool tprof = ppmcTprofEnabled();
    auto tnow = [] { return std::chrono::steady_clock::now(); };
    auto tms = [](const std::chrono::steady_clock::time_point& a,
                  const std::chrono::steady_clock::time_point& b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
    };
    auto t0 = tnow();
    // [诊断] 网格状态 dump 移到 lift(false) 之后（= 该轮编码时刻的解码端期望状态）。
    resetCurvatureNeighborCache();
    this->seed = seed;
    boundaryScanFacets_.clear();
    // ⚠️ P3.e 对齐（2026-05-14）：与原版 PPMC `decimationStep()` 对齐——
    //   消元后只有被移除顶点的面是 Splittable，其余 face 必须标为 Unsplittable。
    //   若保持 Unknown，InsertedEdgeCodingStep 的 b_toCode 判据会把 Unknown
    //   当作"需编码"（isUnsplittable=false），导致编码端边符号数多于解码端
    //   （解码端 undecimationStep 已将所有 face 标为 Splittable/Unsplittable，
    //   无 Unknown 状态），多余符号会被下一轮 face BFS 消费 → 码流错位。
    for (auto* f : mesh_->faces()) {
        if (!f->isConquered())
            f->setUnsplittable();
    }
    const auto t1 = tnow();
    // ⚠️ P0 对齐：先做一遍 BFS 计算并存储残差到每个 splittable face，
    //   后续 RemovedVertexCodingStep 内的 determineGeometrySym 才能读到正确的小残差。
    //   PPMCEliminateOperator 已经只设置浮点 RemovedVertexPos，整数残差由这里统一写入。
    determineResiduals();
    const auto t2 = tnow();
    if (b_useLiftingScheme)
        lift(false);
    const auto t3 = tnow();
    RemovedVertexCodingStep();
    const auto t4 = tnow();
    InsertedEdgeCodingStep();
    const auto t5 = tnow();
    if (tprof) {
        std::fprintf(stderr,
                     "[PPMC_TPROF] op=%d flags=%.1f detRes=%.1f lift=%.1f removedCoding=%.1f insertedCoding=%.1f total=%.1f | geomN=%ld frame=%.1f lap=%.1f rot=%.1f store=%.1f\n",
                     i_curDecimationId, tms(t0, t1), tms(t1, t2), tms(t2, t3), tms(t3, t4),
                     tms(t4, t5), tms(t0, t5), gPpmcNGeom, gPpmcTFrame, gPpmcTLap,
                     gPpmcTRot, gPpmcTResidualStore);
        gPpmcTFrame = gPpmcTLap = gPpmcTRot = gPpmcTResidualStore = 0.0;
        gPpmcNGeom = 0;
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// ⚠️ P3.e 对齐（2026-05-14）：与原版 PPMC `mymeshLifting.cpp::lift()` 对齐。
//   编码端：在 determineResiduals 之后 RemovedVertexCoding 之前，对每个顶点的
//   邻近 splittable face 残差求和，散布到顶点位置（减少残差距量 → 降低几何熵）。
//   解码端对应 unlift 在 PPMCSymbolReadOperator 中。
// ═════════════════════════════════════════════════════════════════════════════
void PPMCSymbolCollectOperator::lift(bool b_unlift) {
    for (auto* v : mesh_->vertices())
        v->resetState();

    gateQueue.push(seed->poolId());

    while (!gateQueue.empty()) {
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndex(DEFAULT_MESH_ID, gateQueue.front());
        gateQueue.pop();
        if (!inGroup(h)) {
            continue;
        }

        // 原版 CGAL `h->vertex()` 是半边**终点**；MCGAL 的 vertex() 是起点，
        // 终点是 end_vertex()。用 vertex() 会让洪泛退化成 ~3 顶点的链
        // （入队的是终点一侧星，conquered 却打在起点上），且与解码端
        // unlift 的链不一致 → 顶点位置无法还原。必须用 end_vertex()。
        MCGAL::Vertex* vh = h->end_vertex();
        if (vh->isConquered())
            continue;
        vh->setConquered();

        // 邻居入队
        MCGAL::Halfedge* hIt = h->next();
        int cg = 0;
        while (hIt->opposite() && hIt->opposite() != h) {
            if (inGroup(hIt) && !hIt->end_vertex()->isConquered())
                gateQueue.push(hIt->poolId());
            hIt = hIt->opposite()->next();
            if (++cg > 2048) {
                fprintf(stderr, "[PPMC ENC LIFT] BROKEN CYCLE at poolId=%d\n", h->poolId());
                break;
            }
        }

        // 累加邻近 splittable face 的残差
        MCGAL::PointInt liftAcc(0, 0, 0);
        int nNeighbours = 0;
        hIt = h;
        do {
            MCGAL::Facet* f = hIt->face();
            if (f && f->isSplittable()) {
                liftAcc = liftAcc + f->getRemovedVertexPosInt();
            }
            nNeighbours++;
            MCGAL::Halfedge* next = hIt->next();
            hIt = next ? next->opposite() : nullptr;
        } while (hIt && hIt != h);
        if (nNeighbours == 0) {
            continue;
        }

        MCGAL::PointInt quantPos = mesh_->floatPosToInt(vh->point());

        if (b_unlift)
            liftAcc = MCGAL::PointInt(-liftAcc[0], -liftAcc[1], -liftAcc[2]);

        // newPos = quantPos + lift / (degree * INV_GAMMA)
        // INV_GAMMA = 2，除 degree 后整除
        int div = nNeighbours * 2;  // INV_GAMMA
        MCGAL::PointInt delta(liftAcc[0] / div, liftAcc[1] / div, liftAcc[2] / div);
        MCGAL::PointInt newPosInt(quantPos[0] + delta[0],
                                  quantPos[1] + delta[1],
                                  quantPos[2] + delta[2]);

        // LIFTING_NB_ADDITIONAL_BITS_GEOMETRY = 1：保序不溢出。
        // 越界（正常网格不会，仅贴 bboxMin/Max 平面的顶点可能）时**跳过该顶点**
        // 而不是钳位：钳位不可逆，解码端 unlift 会过冲。跳过判定只依赖
        // quantPos + delta，两侧相同 → 编解码严格对称（解码端同处跳过）。
        const int maxVal = 1 << (mesh_->i_nbQuantBits + 1);
        bool outOfRange = false;
        for (int i = 0; i < 3; ++i) {
            if (newPosInt[i] < 0 || newPosInt[i] >= maxVal) {
                outOfRange = true;
                break;
            }
        }
        if (outOfRange)
            continue;

        vh->setPoint(mesh_->intPosToFloat(newPosInt));
    }
}

/**
 * 与原版 PPMC `mymeshComp.cpp::determineResiduals()` 对齐：
 *   以 seed 为起点 BFS，对每个 splittable face 计算
 *     residual = Q(removedVertexPos) - Q(barycenter(face_via_h))
 *   并写回 facet 的 RemovedVertexPosInt。barycenter 必须传入与解码端一致的 halfedge
 *   起点，因此使用从 gateQueue 弹出的 h 而非 facet->proxyHalfedge()——这与
 *   RemovedVertexCodingStep / decode 端 BFS 顺序天然一致。
 *
 * 复用了与 RemovedVertexCodingStep 相同的 face processed flag，因此本函数末尾必须
 * 重置 processedFlag，避免下一阶段 BFS 全部跳过。
 */
void PPMCSymbolCollectOperator::determineResiduals() {
    for (MCGAL::Facet* fit : mesh_->faces()) {
        fit->resetProcessedFlag();
    }

    while (!gateQueue.empty()) gateQueue.pop();
    gateQueue.push(seed->poolId());

    while (true) {
        if (gateQueue.empty()) {
            if (!ppmcMultiComponentSymbols()) break;
            MCGAL::Halfedge* nextSeed = findNextUnprocessedFaceSeed();
            if (!nextSeed) break;
            gateQueue.push(nextSeed->poolId());
        }
        int hIndex = gateQueue.front();
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndex(DEFAULT_MESH_ID, hIndex);
        gateQueue.pop();
        if (!inGroup(h)) {
            continue;
        }

        if (!h || !h->face()) continue;
        MCGAL::Facet* f = h->face();
        if (f->isProcessed())
            continue;

        // Mark processed.
        f->setProcessedFlag();

        // BFS 邻居入队
        MCGAL::Halfedge* hIt = h;
        while (hIt) {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp && hOpp->face() && inGroup(hOpp) && !hOpp->face()->isProcessed())
                gateQueue.push(hOpp->poolId());
            if (!hIt->next()) break;
            hIt = hIt->next();
            if (hIt == h) break;
        }

        if (f->isSplittable()) {
            // residual = Q(removedPos) - Q(barycenter(h))
            // ⚠️ P3.b 对齐（2026-05-01）：回退到原版浮点 barycenter + floatPosToInt。
            //   P2.a-extend 的 `barycenterIntQuant`（int64 trunc 累加）消除了 ULP 级
            //   浮点差异，但与原版 PPMC 的 `CGAL::ORIGIN + Σv/d` 浮点累加语义不同；
            //   原版 PPMC 编/解码两端走完全相同浮点代码 + 同一 BFS 顺序，浮点累加
            //   顺序天然一致，结果 bit-identical。故回到原版语义最简单、最稳定。
            MCGAL::PointInt removedQ = mesh_->floatPosToInt(f->getRemovedVertexPos());
            MCGAL::PointInt baryQ = mesh_->floatPosToInt(MCGAL::barycenter(h));
            f->setRemovedVertexPosInt(removedQ - baryQ);
        }
    }

    // 重置 processed flag，让 RemovedVertexCodingStep 能再 BFS 一遍同样顺序。
    for (MCGAL::Facet* fit : mesh_->faces()) {
        fit->resetProcessedFlag();
    }
}

void PPMCSymbolCollectOperator::RemovedVertexCodingStep() {
    // Encode the type of operation on one bit.
    for (MCGAL::Facet* fit : mesh_->faces()) {
        fit->resetProcessedFlag();
    }

    while (!gateQueue.empty()) gateQueue.pop();
    // Add the first halfedge to the queue.
    gateQueue.push(seed->poolId());

    // Resize the vectors to add the current conquest symbols.
    geometrySym.push_back(std::deque<MCGAL::PointInt>());
    connectFaceSym.push_back(std::deque<std::pair<unsigned, unsigned>>());

    f_avgSurfaceFaceWithCenterRemoved = 0.0f;
    f_avgSurfaceFaceWithoutCenterRemoved = 0.0f;
    i_nbFacesWithCenterRemoved = 0;
    i_nbFacesWithoutCenterRemoved = 0;
    i_nbGoodPredictions = 0;

    printf("Removed vertex coding begining.\n");
    PPMCFaceTraversalDebugStats faceDebugStats;
    const bool faceDebugEnabled = ppmcSymbolDebugLogs() || ppmcTraceLimit() > 0;
#ifdef DEBUG
    std::vector<int> correctionVec;
    std::vector<MCGAL::Facet*> facets;
#endif
    while (true) {
        if (gateQueue.empty()) {
            if (!ppmcMultiComponentSymbols()) break;
            MCGAL::Halfedge* nextSeed = findNextUnprocessedFaceSeed();
            if (!nextSeed) break;
            gateQueue.push(nextSeed->poolId());
            ++faceDebugStats.componentSeeds;
        }
        int hIndex = gateQueue.front();
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndex(DEFAULT_MESH_ID, hIndex);
        gateQueue.pop();
        if (!inGroup(h)) {
            continue;
        }

        if (!h || !h->face()) continue;
        MCGAL::Facet* f = h->face();

        // If the face is already processed, pick the next halfedge:
        if (f->isProcessed())
            continue;
        boundaryScanFacets_.push_back(f);

        // Determine face symbol.
        unsigned sym, symPred;
        bool b_split = f->isSplittable();
        // ⚠️ P3.b 对齐：浮点 face surface（fan Heron 三角面积和，原版 PPMC 语义）。
        float f_faceSurface = MCGAL::faceSurface(h);

        // Connectivity prediction.
        // ─── 论文 Peng & Kuo 2005 Sect. 3.2 连接性预测 ─────────────────────────
        // 先尝试 triangle-mesh 邻域 balance 预测（仅在当前 face 是三角形且两类历史比例
        // 接近 0.5 时启用），否则回退到面积均值预测。symPred=1 表示预测与真实相符。
        bool predictDone = false;
        const unsigned faceDegree = faceDegreeFromGate(h);
        // ⚠️ 参考实现对齐：balance 预测**不限制面度数**（原版只看两类历史比例是否接近），
        // 我们此前多加的 `faceDegree == 3` 限制会让 patch 多边形全部退化到面积预测，
        // 实测 horse19k 第 1 轮预测命中率 65.6% vs 参考 81.6%（conn +332 B）。
        if (b_useTriangleMeshConnectivityPredictionFaces) {
            if (faceClassRatiosClose(i_nbFacesWithCenterRemoved, i_nbFacesWithoutCenterRemoved)) {
                int i_neighborBalance = 0;
                // P3.b: cycle-guard — 首次断裂时 abort 便于定位漂移起点。
                MCGAL::Halfedge* hnb = h;
                int cg = 0;
                while (hnb) {
                    MCGAL::Halfedge* hnbOpp = hnb->opposite();
                    MCGAL::Facet* fn = hnbOpp ? hnbOpp->face() : nullptr;
                    if (fn && fn->isProcessed()) {
                        if (fn->isSplittable())
                            i_neighborBalance++;
                        else if (fn->isUnsplittable())
                            i_neighborBalance--;
                    }
                    if (!hnb->next()) break;
                    hnb = hnb->next();
                    if (++cg > 1024) {
                        fprintf(stderr, "[PPMC ENC] BROKEN CYCLE in triangle balance, deg=%u, poolId=%d\n",
                                faceDegree, h->poolId());
                        std::abort();
                    }
                    if (hnb == h) break;
                }

                // 与原版一致：balance>=0 预测 not-split；balance<0 预测 split。
                if ((i_neighborBalance >= 0 && !b_split) ||
                    (i_neighborBalance < 0 && b_split)) {
                    symPred = 1;
                    i_nbGoodPredictions++;
                } else {
                    symPred = 0;
                }
                predictDone = true;
            }
        }

        if (!predictDone) {
            // 面积均值预测（浮点 fabs 比较，与原版 mymeshComp.cpp L386 完全一致）。
            bool pSplit = predictSplitByAreaFloat(f_faceSurface,
                                                  f_avgSurfaceFaceWithCenterRemoved,
                                                  f_avgSurfaceFaceWithoutCenterRemoved);
            if (pSplit == b_split) {
                symPred = 1;
                i_nbGoodPredictions++;
            } else {
                symPred = 0;
            }
        }

        // No connectivity prediction.
        sym = b_split ? 1 : 0;

        // Update the average surfaces.
        updateAvgSurfaces(b_split, f_faceSurface);
        if (faceDebugEnabled) {
            ++faceDebugStats.faceVisits;
            if (b_split) {
                ++faceDebugStats.split;
                ++faceDebugStats.residuals;
            } else {
                ++faceDebugStats.unsplit;
            }
            const std::uint64_t ringHash = faceRingDebugHash(mesh_.get(), h, b_split);
            faceDebugStats.sequenceHash =
                mixDebugHash(faceDebugStats.sequenceHash, ringHash);
            traceFaceDebug("encode",
                           groupFilter_,
                           i_curDecimationId,
                           faceDebugStats.faceVisits - 1,
                           mesh_.get(),
                           h,
                           b_split,
                           f_faceSurface);
        }

        // Push the symbols.
        connectFaceSym[i_curDecimationId].push_back(std::pair<unsigned, unsigned>(sym, symPred));

        // Determine the geometry symbol.
        if (b_split) {
            determineGeometrySym(h, f);
#ifdef DEBUG
            facets.push_back(f);
            correctionVec.push_back(f->getRemovedVertexPosInt()[0]);
            correctionVec.push_back(f->getRemovedVertexPosInt()[1]);
            correctionVec.push_back(f->getRemovedVertexPosInt()[2]);
#endif
        }

        // Mark the face as processed.
        f->setProcessedFlag();

        // Add the other halfedges to the queue — with cycle-guard (P3.b).
        MCGAL::Halfedge* hIt = h;
        int cg = 0;
        while (hIt) {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            // assert(!hOpp->is_border());
            if (hOpp && hOpp->face() && inGroup(hOpp) && !hOpp->face()->isProcessed())
                gateQueue.push(hOpp->poolId());
            if (!hIt->next()) break;
            hIt = hIt->next();
            if (++cg > 1024) {
                fprintf(stderr, "[PPMC ENC] BROKEN CYCLE in neighbor push, deg=%u, poolId=%d\n",
                        faceDegree, h->poolId());
                std::abort();
            }
            if (hIt == h) break;
        }
    }
#ifdef DEBUG
    // DEBUG_DUMP_CONTAINER(correctionVec, "./encode/correctionVec.txt");
    DEBUG_DUMP_FACETS(facets, "./encode/facets.txt");
#endif

    printf("Removed vertex coding completed.\n");
    if (faceDebugEnabled) {
        std::cerr << "[PMSF_PPMC_FACE_DEBUG] encode"
                  << " group=" << groupFilter_
                  << " op=" << i_curDecimationId
                  << " face_visits=" << faceDebugStats.faceVisits
                  << " split=" << faceDebugStats.split
                  << " unsplit=" << faceDebugStats.unsplit
                  << " residuals=" << faceDebugStats.residuals
                  << " component_seeds=" << faceDebugStats.componentSeeds
                  << " sequence_hash=" << faceDebugStats.sequenceHash
                  << " prediction_good=" << i_nbGoodPredictions
                  << "\n";
    }
    float f_nb = i_nbFacesWithCenterRemoved > i_nbFacesWithoutCenterRemoved ? i_nbFacesWithCenterRemoved : i_nbFacesWithoutCenterRemoved;

    unsigned usePrediction = (float)i_nbGoodPredictions / mesh_->size_of_facets() > f_nb / mesh_->size_of_facets() ? 1 : 0;
    facesConnectPredictionUsed.push_back(usePrediction);
}

/**
 * Update the average lengths of the original and inserted edges.
 */
// ⚠️ P3.b 对齐：回退到原版浮点增量均值。
void PPMCSymbolCollectOperator::updateAvgEdgeLen(bool b_original, float f_edgeLen) {
    if (b_original) {
        i_nbOriginalEdges++;
        f_avgOriginalEdgesLength = (f_avgOriginalEdgesLength
                                     * (i_nbOriginalEdges - 1) + f_edgeLen)
                                    / i_nbOriginalEdges;
    } else {
        i_nbInsertedEdges++;
        f_avgInsertedEdgesLength = (f_avgInsertedEdgesLength
                                     * (i_nbInsertedEdges - 1) + f_edgeLen)
                                    / i_nbInsertedEdges;
    }
}

void PPMCSymbolCollectOperator::InsertedEdgeCodingStep() {
    while (!gateQueue.empty()) gateQueue.pop();
    // Add the first halfedge to the queue.
    gateQueue.push(seed->poolId());

    // ⚠️ P3.b 对齐：回退到浮点累加器 f_avgInsertedEdgesLength / f_avgOriginalEdgesLength。
    f_avgInsertedEdgesLength = 0.0f;
    f_avgOriginalEdgesLength = 0.0f;
    i_nbInsertedEdges = 0;
    i_nbOriginalEdges = 0;
    i_nbGoodPredictions = 0;

    // Resize the vector to add the current conquest symbols.
    connectEdgeSym.push_back(std::deque<std::pair<unsigned, unsigned>>());
    printf("Inserted edge coding begining.\n");
    int edgeSize = 0;
    PPMCEdgeTraversalDebugStats debugStats;
#ifdef DEBUG
    std::vector<MCGAL::Halfedge*> halfedges;
#endif
    auto enqueuePendingBoundaryEdges = [&]() -> int {
        int added = 0;
        for (MCGAL::Facet* f : boundaryScanFacets_) {
            if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
            MCGAL::Halfedge* start = f->proxyHalfedge();
            MCGAL::Halfedge* e = start;
            int guard = 0;
            do {
                if (e && e->face() == f && !e->isProcessed() && !e->opposite() && inGroup(e)) {
                    gateQueue.push(e->poolId());
                    ++added;
                    ++debugStats.boundaryEnqueues;
                }
                e = e ? e->next() : nullptr;
            } while (e && e != start && ++guard <= 2048);
        }
        return added;
    };

    while (true) {
        if (gateQueue.empty() && enqueuePendingBoundaryEdges() == 0) {
            if (!ppmcMultiComponentSymbols()) break;
            MCGAL::Halfedge* nextSeed = findNextUnprocessedHalfedgeSeed();
            if (!nextSeed) break;
            gateQueue.push(nextSeed->poolId());
            ++debugStats.componentSeeds;
        }
        int hIndex = gateQueue.front();
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndex(DEFAULT_MESH_ID, hIndex);
        gateQueue.pop();
        if (!h || !h->face()) {
            continue;
        }
        if (!inGroup(h)) {
            continue;
        }

        // Test if the edge has already been conquered.
        if (h->isProcessed())
            continue;
        edgeSize++;
        ++debugStats.edgeVisits;
        // Mark the halfedge as processed.
        h->setProcessed();
        if (h->opposite() && inGroup(h->opposite())) {
            h->opposite()->setProcessed();
        }

        // Add the other halfedges to the queue — with cycle-guard (P3.b).
        MCGAL::Halfedge* hIt = h->next();
        int cg = 0;
        while (hIt && hIt->opposite() && hIt->opposite() != h &&
               (groupFilter_ < 0 || inGroup(hIt->opposite()))) {
            if (inGroup(hIt) && !hIt->isProcessed())
                gateQueue.push(hIt->poolId());
            hIt = hIt->opposite()->next();
            if (++cg > 2048) {
                fprintf(stderr, "[PPMC ENC] BROKEN CYCLE in InsertedEdgeCodingStep neighbor walk, poolId=%d\n",
                        h->poolId());
                std::abort();
            }
        }
        // Don't write a symbol if the two faces of an egde are unsplitable.
        const bool sameGroupOpp =
            h->opposite() && h->opposite()->face() &&
            (groupFilter_ < 0 || inGroup(h->opposite()));
        if (sameGroupOpp) {
            ++debugStats.sameGroupOpp;
        } else {
            ++debugStats.noSameGroupOpp;
        }
        bool b_toCode = sameGroupOpp
                            ? (h->face()->isSplittable() ||
                               h->opposite()->face()->isSplittable())
                            : h->face()->isSplittable();

        // Determine the edge symbol.
        unsigned sym, symPred;
        bool b_original = h->isOriginal();
        // ⚠️ P3.b 对齐：浮点 edge length（原版 PPMC `MyMesh::edgeLen(heh)` 语义）。
        float f_edgeLen = safeEdgeLen(h);

        // Connectivity prediction：predictOriginal == b_original ? symPred=1 : 0。
        bool pOrig = predictOriginalByLenFloat(f_edgeLen,
                                                f_avgOriginalEdgesLength,
                                                f_avgInsertedEdgesLength);
        if (pOrig == b_original) {
            symPred = 1;
            i_nbGoodPredictions++;
        } else {
            symPred = 0;
        }

        if (b_original) {
            sym = 0;
            ++debugStats.original;
        } else {
            sym = 1;
            ++debugStats.inserted;
#ifdef DEBUG
            halfedges.push_back(h);
#endif
        }

        // Update the average edge lengths.
        updateAvgEdgeLen(b_original, f_edgeLen);

        // Store the symbol if needed.
        if (b_toCode) {
            ++debugStats.coded;
            connectEdgeSym[i_curDecimationId].push_back(std::pair<unsigned, unsigned>(sym, symPred));
        } else {
            ++debugStats.noCode;
            if (sameGroupOpp) {
                ++debugStats.noCodeBothUnsplit;
            } else {
                ++debugStats.noCodeBoundaryUnsplit;
            }
        }
    }

    printf("Inserted edge coding completed.\n");

    float f_nb = i_nbOriginalEdges > i_nbInsertedEdges ? i_nbOriginalEdges : i_nbInsertedEdges;

    unsigned usePrediction =
        edgeSize > 0 && (float)i_nbGoodPredictions * 2 / edgeSize > f_nb * 2 / edgeSize
            ? 1
            : 0;
    edgesConnectPredictionUsed.push_back(usePrediction);
    if (ppmcSymbolDebugLogs()) {
        std::cerr << "[PMSF_PPMC_EDGE_DEBUG] encode"
                  << " group=" << groupFilter_
                  << " op=" << i_curDecimationId
                  << " edge_visits=" << debugStats.edgeVisits
                  << " edge_symbols=" << debugStats.coded
                  << " no_code=" << debugStats.noCode
                  << " no_code_both_unsplit=" << debugStats.noCodeBothUnsplit
                  << " no_code_boundary_unsplit=" << debugStats.noCodeBoundaryUnsplit
                  << " same_group_opp=" << debugStats.sameGroupOpp
                  << " no_same_group_opp=" << debugStats.noSameGroupOpp
                  << " original=" << debugStats.original
                  << " inserted=" << debugStats.inserted
                  << " boundary_enqueues=" << debugStats.boundaryEnqueues
                  << " component_seeds=" << debugStats.componentSeeds
                  << " prediction_good=" << i_nbGoodPredictions
                  << " prediction_used=" << usePrediction
                  << "\n";
    }

#ifdef DEBUG
    DEBUG_DUMP_HALFEDGES(halfedges, "./encode/halfedges.txt");
#endif

    i_curDecimationId++;  // Increment the current decimation operation id.
}

int PPMCSymbolCollectOperator::exportToBuffer(char* buffer, bool isEnableQuantization, bool isEnableCompress) {
    requirePPMCPaperExactIfRequested();
    size_t offset = 0;
    int i_nbDecimations = i_curDecimationId - 1;
    lastEncodedByteBreakdown_ = EncodedByteBreakdown{};
    rangeCoder.p_data = buffer;
    rangeCoder.p_dataOffset = &offset;
    // Encode in REVERSE order (round N-1,...,0).  The decoder reads forward
    // (round 0,1,...,N-1), so the buffer order is: last elimination → first.
    // This is intentional: decoder round k reads the symbols for undoing
    // elimination round (N-1-k), which is the correct reconstruction order.
    while (true) {
        if (i_nbDecimations < 0)
            break;
        encodeRemovedVertices(i_nbDecimations);
        encodeInsertedEdges(i_nbDecimations);
        i_nbDecimations--;
    }
    std::cout << "[PPMC_RC_BREAKDOWN]"
              << " total_bytes=" << offset
              << " facet_symbol_bytes=" << lastEncodedByteBreakdown_.facetSymbolBytes()
              << " residual_bytes=" << lastEncodedByteBreakdown_.residualBytes
              << " removed_connectivity_bytes=" << lastEncodedByteBreakdown_.removedConnectivityBytes
              << " inserted_connectivity_bytes=" << lastEncodedByteBreakdown_.insertedConnectivityBytes
              << " removed_stream_bytes=" << lastEncodedByteBreakdown_.removedStreamBytes
              << " inserted_stream_bytes=" << lastEncodedByteBreakdown_.insertedStreamBytes
              << " operations=" << lastEncodedByteBreakdown_.operations
              << std::endl;
    return offset;
}

const pmsf::PMSFSymbolPayloadTable*
PPMCSymbolCollectOperator::paperSymbolPayloadTable() const {
    if (paperPayloadTableCacheDecimationId_ == i_curDecimationId &&
        paperPayloadTableCacheGroupFilter_ == groupFilter_) {
        return &paperPayloadTableCache_;
    }
    paperPayloadTableCache_.clear();
    paperPayloadTableCacheDecimationId_ = i_curDecimationId;
    paperPayloadTableCacheGroupFilter_ = groupFilter_;
    if (i_curDecimationId <= 0) {
        return &paperPayloadTableCache_;
    }

    // 与下方 encodeToBuffer 的 MeasScratch 同理（P15）：`std::vector<char>
    // scratch(BUFFER_SIZE)` 是值初始化，每次调用都把 600 MB 清零；本函数按
    // decimationId 缓存失效 ⇒ **每轮调用一次**（15 轮 = 9 GB memset，占
    // PPMC binary 编码约一半指令数）。scratch 是纯输出草稿：
    // exportOperationToBuffer 顺序写入、只回读自己写过的 `bytes` 字节
    // （见下方 appendPPMCOperationPayload 的 length 参数），故可进程内复用。
    struct PayloadScratch {
        char* buf = nullptr;
        PayloadScratch() : buf(new char[BUFFER_SIZE]) {}
    };
    static thread_local PayloadScratch payloadScratch;
    char* const scratchBuf = payloadScratch.buf;
    auto* self = const_cast<PPMCSymbolCollectOperator*>(this);
    for (int op = 0; op < i_curDecimationId; ++op) {
        if (!hasOperationData(op)) {
            continue;
        }
        const int bytes = self->exportOperationToBuffer(op, scratchBuf);
        if (bytes <= 0) {
            continue;
        }

        pmsf::appendPPMCOperationPayload(
            paperPayloadTableCache_,
            scratchBuf,
            static_cast<std::size_t>(bytes),
            groupFilter_ >= 0 ? groupFilter_ : 0,
            static_cast<std::int32_t>(op));
    }
    return &paperPayloadTableCache_;
}

bool PPMCSymbolCollectOperator::hasOperationData(int operationId) const {
    if (operationId < 0 || operationId >= i_curDecimationId) {
        return false;
    }
    return operationId < static_cast<int>(connectFaceSym.size()) &&
           operationId < static_cast<int>(connectEdgeSym.size()) &&
           operationId < static_cast<int>(geometrySym.size()) &&
           !connectFaceSym[operationId].empty() &&
           !connectEdgeSym[operationId].empty() &&
           !geometrySym[operationId].empty();
}

int PPMCSymbolCollectOperator::exportOperationToBuffer(int operationId, char* buffer) {
    requirePPMCPaperExactIfRequested();
    if (!hasOperationData(operationId)) {
        return 0;
    }
    if (ppmcSplitZlibOperationCodecEnabled()) {
        const int splitBytes = exportOperationToSplitZlibBuffer(operationId, buffer);
        if (splitBytes > 0) {
            return splitBytes;
        }
    }
    size_t offset = 0;
    lastEncodedByteBreakdown_ = EncodedByteBreakdown{};
    rangeCoder.p_data = buffer;
    rangeCoder.p_dataOffset = &offset;
    encodeRemovedVertices(static_cast<unsigned>(operationId));
    encodeInsertedEdges(static_cast<unsigned>(operationId));
    return static_cast<int>(offset);
}

int PPMCSymbolCollectOperator::exportOperationToSplitZlibBuffer(int operationId, char* buffer) {
    if (!buffer || !hasOperationData(operationId)) {
        return 0;
    }

    const unsigned op = static_cast<unsigned>(operationId);
    std::deque<std::pair<unsigned, unsigned>>& connSym = connectFaceSym[op];
    std::deque<MCGAL::PointInt>& geomSym = geometrySym[op];
    std::deque<std::pair<unsigned, unsigned>>& edgeSym = connectEdgeSym[op];
    if (connSym.empty() || geomSym.empty() || edgeSym.empty()) {
        return 0;
    }

    const bool facePredictionUsed =
        b_useConnectivityPredictionFaces && facesConnectPredictionUsed[op];
    const bool edgePredictionUsed =
        b_useConnectivityPredictionEdges && edgesConnectPredictionUsed[op];

    const std::size_t faceConnCap =
        std::max<std::size_t>(1024, connSym.size() * 8 + 1024);
    std::vector<char> faceConn(faceConnCap);
    std::deque<uint8_t> faceSyms;
    for (const auto& s : connSym) {
        faceSyms.push_back(static_cast<uint8_t>(
            facePredictionUsed ? s.second : s.first));
    }
    const std::size_t faceBytes = pmsf::encodeUint8Queue(
        faceSyms, 2, faceConn.data(), faceConnCap);
    faceConn.resize(faceBytes);

    const std::size_t edgeConnCap =
        std::max<std::size_t>(1024, edgeSym.size() * 8 + 1024);
    std::vector<char> edgeConn(edgeConnCap);
    std::deque<uint8_t> edgeSyms;
    for (const auto& s : edgeSym) {
        edgeSyms.push_back(static_cast<uint8_t>(
            edgePredictionUsed ? s.second : s.first));
    }
    const std::size_t edgeBytes = pmsf::encodeUint8Queue(
        edgeSyms, 2, edgeConn.data(), edgeConnCap);
    edgeConn.resize(edgeBytes);

    std::vector<char> residualRaw;
    residualRaw.reserve(8 + geomSym.size() * 6);
    putVarU32(residualRaw, static_cast<std::uint32_t>(geomSym.size()));
    for (const MCGAL::PointInt& v : geomSym) putVarI32(residualRaw, v.x());
    for (const MCGAL::PointInt& v : geomSym) putVarI32(residualRaw, v.y());
    for (const MCGAL::PointInt& v : geomSym) putVarI32(residualRaw, v.z());
    std::vector<char> residualPayload;
    zlibMaybeCompress(residualRaw, residualPayload);

    const std::size_t headerBytes = 4 + 1 + 1 + 4 + 4 + 4 + 4 + 4;
    const std::size_t totalBytes = headerBytes + faceConn.size() +
                                   residualPayload.size() + edgeConn.size();
    if (totalBytes > static_cast<std::size_t>(BUFFER_SIZE)) {
        return 0;
    }

    std::size_t out = 0;
    buffer[out++] = 'P';
    buffer[out++] = 'O';
    buffer[out++] = 'C';
    buffer[out++] = '2';
    buffer[out++] = static_cast<char>(facePredictionUsed ? 1 : 0);
    buffer[out++] = static_cast<char>(edgePredictionUsed ? 1 : 0);
    putU32LEToBuffer(buffer, out, static_cast<std::uint32_t>(faceConn.size()));
    putU32LEToBuffer(buffer, out, static_cast<std::uint32_t>(residualPayload.size()));
    putU32LEToBuffer(buffer, out, static_cast<std::uint32_t>(edgeConn.size()));
    putU32LEToBuffer(buffer, out, static_cast<std::uint32_t>(connSym.size()));
    putU32LEToBuffer(buffer, out, static_cast<std::uint32_t>(edgeSym.size()));
    std::memcpy(buffer + out, faceConn.data(), faceConn.size());
    out += faceConn.size();
    std::memcpy(buffer + out, residualPayload.data(), residualPayload.size());
    out += residualPayload.size();
    std::memcpy(buffer + out, edgeConn.data(), edgeConn.size());
    out += edgeConn.size();

    return static_cast<int>(out);
}

/**
 * Encode an inserted edge list.
 */
void PPMCSymbolCollectOperator::encodeInsertedEdges(unsigned i_operationId) {
    std::deque<std::pair<unsigned, unsigned>>& symbols = connectEdgeSym[i_operationId];
    assert(!symbols.empty());

    const bool predictionUsed = b_useConnectivityPredictionEdges
                                    ? edgesConnectPredictionUsed[i_operationId]
                                    : false;

    pmsf::MultiModelRangeEncoder encoder(rangeCoder.p_data, rangeCoder.p_dataOffset);
    encoder.encodeRawBit(predictionUsed ? 1 : 0);
    const int connectModel = encoder.addModel({2, 10, 1 << 9});

    for (const auto& s : symbols) {
        const unsigned sym = predictionUsed ? s.second : s.first;
        encoder.encode(connectModel, static_cast<int>(sym));
    }

    const unsigned i_size = static_cast<unsigned>(encoder.finish());
    lastEncodedByteBreakdown_.insertedConnectivityBytes += i_size;
    lastEncodedByteBreakdown_.insertedStreamBytes += i_size;
}

/**
 * Encode the geometry and the connectivity of a removed vertex list.
 */
void PPMCSymbolCollectOperator::encodeRemovedVertices(unsigned i_operationId) {
    std::deque<std::pair<unsigned, unsigned>>& connSym = connectFaceSym[i_operationId];
    std::deque<MCGAL::PointInt>& geomSym = geometrySym[i_operationId];

    if (const int snapBits = ppmcResidualSnapBits(); snapBits > 0) {
        for (MCGAL::PointInt& v : geomSym) {
            v[0] = snapSignedResidual(v[0], snapBits);
            v[1] = snapSignedResidual(v[1], snapBits);
            v[2] = snapSignedResidual(v[2], snapBits);
        }
    }

    const bool predictionUsed = b_useConnectivityPredictionFaces
                                    ? facesConnectPredictionUsed[i_operationId]
                                    : false;

    const unsigned i_lenGeom = geomSym.size();
    const unsigned i_lenConn = connSym.size();
    assert(i_lenGeom > 0);
    assert(i_lenConn > 0);

    // Determine the min and max values for the geometry coding.
    alphaBetaMin = geomSym[0].x();
    int alphaBetaMax = geomSym[0].x();
#ifdef USE_BIJECTION
    gammaMin = geomSym[0].z();
    int gammaMax = geomSym[0].z();
#endif

    for (unsigned i = 0; i < i_lenGeom; ++i) {
        const MCGAL::PointInt& v = geomSym[i];

        if (v.x() < alphaBetaMin) alphaBetaMin = v.x();
        if (v.y() < alphaBetaMin) alphaBetaMin = v.y();
#ifdef USE_BIJECTION
        if (v.z() < gammaMin) gammaMin = v.z();
#else
        if (v.z() < alphaBetaMin) alphaBetaMin = v.z();
#endif

        if (v.x() > alphaBetaMax) alphaBetaMax = v.x();
        if (v.y() > alphaBetaMax) alphaBetaMax = v.y();
#ifdef USE_BIJECTION
        if (v.z() > gammaMax) gammaMax = v.z();
#else
        if (v.z() > alphaBetaMax) alphaBetaMax = v.z();
#endif
    }

    const unsigned alphaBetaRange = std::max(alphaBetaMax - alphaBetaMin + 1, 2);
#ifdef USE_BIJECTION
    const unsigned gammaRange = std::max(gammaMax - gammaMin + 1, 2);
#endif

    assert(alphaBetaMin >= -(1 << 14) && alphaBetaMin <= (1 << 14));
    assert(alphaBetaRange < (1 << 14));
#ifdef USE_BIJECTION
    assert(gammaMin >= -(1 << 14) && gammaMin <= (1 << 14));
    assert(gammaRange < (1 << 14));
#endif

    // Main encoder: header + interleaved connectivity/geometry.
    pmsf::MultiModelRangeEncoder encoder(rangeCoder.p_data, rangeCoder.p_dataOffset);
    encoder.encodeRawBit(predictionUsed ? 1 : 0);
    encoder.encodeRawShort(static_cast<uint16_t>(alphaBetaMin));
    encoder.encodeRawShort(static_cast<uint16_t>(alphaBetaRange));
#ifdef USE_BIJECTION
    encoder.encodeRawShort(static_cast<uint16_t>(gammaMin));
    encoder.encodeRawShort(static_cast<uint16_t>(gammaRange));
#endif

    const int connectModel = encoder.addModel({2, 10, 1 << 9});
    const int qsRescale = ppmcQsmodelRescale();
    const int alphaBetaModel = encoder.addModel(
        {static_cast<int>(alphaBetaRange), 18, qsRescale});
#ifdef USE_BIJECTION
    const int gammaModel = encoder.addModel(
        {static_cast<int>(gammaRange), 18, qsRescale});
#endif

    // Measurement encoder: connectivity only, same model parameters.
    //
    // ── P15（2026-09-24）：600 MB 缓冲由「每次调用零初始化」改为**进程内复用** ──
    // 本函数在 `while (true) { … i_nbDecimations--; }` 里**每轮调用一次**
    // （见 `encodeToBuffer` 的 N-1 … 0 反向循环）。原写法
    // `std::vector<char> measBuf(BUFFER_SIZE)` 会**每轮**把 600 MB 清零
    // （`vector` 的定长构造是值初始化）—— 50 轮即 30 GB 的 memset，
    // 正是 perf 里 `__memset_avx2_unaligned_erms` 9.76% 与内核态 ~20% 的来源。
    //
    // 复用之后：**同一块内存，只在首次触碰时由内核清零一次**。
    // 安全性论证：`measEnc` 从 `measOff = 0` 起**顺序写入**，只回读自己写过的
    // 字节；缓冲是纯输出草稿。判据：门禁 60/60 逐字节相同（硬约束 1）。
    struct MeasScratch {
        char* buf = nullptr;
        MeasScratch() : buf(new char[BUFFER_SIZE]) {}
    };
    static thread_local MeasScratch measScratch;
    char* const measBuf = measScratch.buf;
    size_t measOff = 0;
    pmsf::MultiModelRangeEncoder measEnc(measBuf, &measOff);
    const int connectModelMes = measEnc.addModel({2, 10, 1 << 9});

    unsigned k = 0;
    for (unsigned i = 0; i < i_lenConn; ++i) {
        const unsigned sym = predictionUsed ? connSym[i].second : connSym[i].first;
        const bool b_split = connSym[i].first;

        encoder.encode(connectModel, static_cast<int>(sym));
        measEnc.encode(connectModelMes, static_cast<int>(sym));

        if (b_split) {
            const MCGAL::PointInt& v = geomSym[k];
            for (unsigned j = 0; j < 3; ++j) {
#ifdef USE_BIJECTION
                if (j < 2) {
                    encoder.encode(alphaBetaModel,
                                   static_cast<int>(v[j] - alphaBetaMin));
                } else {
                    encoder.encode(gammaModel,
                                   static_cast<int>(v[j] - gammaMin));
                }
#else
                encoder.encode(alphaBetaModel,
                               static_cast<int>(v[j] - alphaBetaMin));
#endif
            }
            k++;
        }
    }

    const unsigned i_size = static_cast<unsigned>(encoder.finish());
    const unsigned i_sizeConn = static_cast<unsigned>(measEnc.finish());
    lastEncodedByteBreakdown_.removedConnectivityBytes += i_sizeConn;
    lastEncodedByteBreakdown_.removedStreamBytes += i_size;
    lastEncodedByteBreakdown_.residualBytes +=
        (i_size >= i_sizeConn) ? (i_size - i_sizeConn) : 0;
    lastEncodedByteBreakdown_.operations += 1;
}
