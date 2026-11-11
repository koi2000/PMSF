// SPDX-License-Identifier: Apache-2.0
//
// RDCQuestHooks.h — task-19 P2b：RD on conquest engine 的引擎 hook 实现。
//
// 接缝（与 `src/conquest/FrontierConquestEngine` + `ConquestDecodeLayer` 共用）：
//   H4 ResidualQuantizer：引擎 `recordGeometry()` 里
//       delta = toCell(original) − toCell(pred)，然后 `transform(delta, out, PatchView)`；
//     解码端 `decodeLayer()` 用**同一个 PatchView** 调 `inverseTransform`，再
//       `cell = toCell(pred) + d`。
//   ⇒ H4 只要**整数域严格可逆**，几何就逐格无损（引擎其余部分已逐层验证）。
//
// 本文件实现论文 Ahn et al. 2011 §3.1 Eq.(3) 的**局部 Frenet 系投影**，但用
// **可逆整数 lifting**（而非 float 旋转+round；后者在 rd_tag 路径已实测会沿预测链
// 累积 ~39 quant unit，见 AUDIT §4）：
//   1) 由 PatchView 的有序邻居位置构造 (n, t1, t2)：
//      n = Newell 法向；t1 = (P[0]−centroid) 投影到切平面并归一化；t2 = n × t1；
//   2) R = Rz(α)Ry(β)Rx(γ)（Euler ZYX），每个 2D 旋转 = 3 个 lifting shear
//      `u += round(a·v)`（a=(c−1)/s, b=s）⇒ 整数逐位可逆；
//      近 π 先做 R(θ)=N·R(θ−π) 归一化（否则 a→∞ 放大 rounding）。
//
// 对称性：PatchView 的邻居顺序/坐标在编解码两端由**同一函数**（`ringNeighbors`，
// 且编码端显式做 base-mesh 往返对齐末位）生成；conquest 几何本身逐格无损
// ⇒ 解码端重建位置 = 编码端吸附后位置 ⇒ frame 数值逐位一致 ⇒ transform 可逆。
//
// env：`PMSF_RD_CQ_GEOM` = native（A/B 对照，恒等）/ frame（默认，本 hook）。
#ifndef PMSF_RD_HOOKS_RD_CQUEST_HOOKS_H
#define PMSF_RD_HOOKS_RD_CQUEST_HOOKS_H

#include "scheme/EngineHooks.h"

#include "Configuration.h"
#include "ContextPool.h"
#include "Mesh.h"
#include "conquest/ConquestBlobFormat.h"
#include "conquest/ConquestTraversal.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <array>
#include <functional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace pmsf {
namespace rd_cq {

enum class GeomMode { Native = 0, Frame = 1 };

inline GeomMode resolveGeomMode() {
    const char* v = std::getenv("PMSF_RD_CQ_GEOM");
    if (v == nullptr || *v == '\0') return GeomMode::Frame;  // rd_cq 默认开
    if (std::strcmp(v, "native") == 0) return GeomMode::Native;
    return GeomMode::Frame;
}

/// H3 开关：**默认关**（env `PMSF_RD_CQ_H3=1` 才开）。
/// 归因（AUDIT §12）：论文 Eq.(2) 的 r_i 需要 2-ring，而 decoder 预测时刻的网格是
/// 「removeEdges + addCenterVertex」后的**临时 fine 图**，只有 c 的 1-ring 与 encoder
/// 的 coarse 图对称；2-ring 在 patch 内部不同 ⇒ 预测分叉。且即便忽略对称性，H3 的
/// 预测在本 walkthrough 上也不降熵（见 §12 实测）。保留实现供未来 Arch-2（encoder
/// 端 decode 模拟）再试。
inline bool h3EnabledFromEnv() {
    const char* v = std::getenv("PMSF_RD_CQ_H3");
    if (v == nullptr || *v == '\0') return false;
    return std::strcmp(v, "0") != 0;
}

// ---- 局部 Frenet 系（与 APTM hook 同一公式，便于交叉验证）------------------
struct RdFrame {
    double n[3] = {0.0, 0.0, 1.0};
    double t1[3] = {1.0, 0.0, 0.0};
    double t2[3] = {0.0, 1.0, 0.0};
    bool valid = false;
};

/// frame 变体（全部**对环起点/方向不敏感**的确定性规则，双端可复算）：
///   lex    = 法向 cross(a,b)（a,b = 量化格字典序最小两个邻居）+ t1 沿 a
///   newell = Newell 法向（用 cross(a,b) 定半球符号）+ t1 沿 a
///   far    = cross(a,b) 法向 + t1 沿「离 centroid 最远的环邻居」（距离规则，
///            起点/方向无关；同距用格坐标字典序 tie-break）
inline int resolveFrameVariant() {
    const char* v = std::getenv("PMSF_RD_CQ_FRAME");
    if (v == nullptr || *v == '\0') return 1;  // 默认 newell（实测最优且对称）
    if (std::strcmp(v, "lex") == 0) return 0;
    if (std::strcmp(v, "far") == 0) return 2;
    return 1;
}

inline RdFrame frameFromPatch(const PatchView& p) {
    // ★ 三角平衡对称性：frame 必须**对环的起点/绕行方向不敏感**。
    //   引擎 encoder 用 `ringNeighbors(vdel)`（删前）、decoder 用 `ringNeighbors(nv)`
    //   （插后），两者只保证**同一组邻居**，起点与方向可能不同；默认算术平均
    //   预测子是次序无关的，所以 r-1 层验证通过。但 frame 的 t1 若取 P[0] 就会
    //   随起点变化 ⇒ transform/inverse 的基不同 ⇒ 几何漂移（实测 coord_max 1.75e-3）。
    //   修法：用**量化格坐标字典序最小**的两个邻居做 canonical anchor，
    //   n = normalize(cross(a, b))、t1 = normalize(a - (a·n)n)、t2 = n×t1
    //   ⇒ 只依赖邻居集合，与起点/方向无关。
    RdFrame f;
    const int N = p.neighborCount;
    if (N < 3 || p.neighborPos == nullptr) return f;
    const double* P = p.neighborPos;
    // 引擎的量化公式（与 ResidualQuantizer::toCell 默认实现同源）：
    //   q = ceil((c - min)/step) - 1，-1 夹到 0。
    auto cell = [&](int i, int k) -> long long {
        const double c = P[3 * i + k];
        int q = static_cast<int>(std::ceil((c - p.bboxMin[k]) / p.quantStep)) - 1;
        if (q == -1) q = 0;
        return static_cast<long long>(q);
    };
    // 选 canonical anchor：按 (x,y,z) 字典序，取最小与次小的**不同格**邻居。
    int i0 = -1, i1 = -1;
    for (int i = 0; i < N; ++i) {
        if (i0 < 0) { i0 = i; continue; }
        const long long ax = cell(i, 0), ay = cell(i, 1), az = cell(i, 2);
        const long long bx = cell(i0, 0), by = cell(i0, 1), bz = cell(i0, 2);
        const bool less = (ax < bx) || (ax == bx && (ay < by || (ay == by && az < bz)));
        if (less) { i1 = i0; i0 = i; }
        else if (i1 < 0) i1 = i;
        else {
            const long long cx = cell(i1, 0), cy = cell(i1, 1), cz = cell(i1, 2);
            const bool less1 = (ax < cx) || (ax == cx && (ay < cy || (ay == cy && az < cz)));
            if (less1) i1 = i;
        }
    }
    if (i0 < 0 || i1 < 0) return f;
    double c[3] = {0.0, 0.0, 0.0};
    for (int i = 0; i < N; ++i) {
        c[0] += P[3 * i + 0];
        c[1] += P[3 * i + 1];
        c[2] += P[3 * i + 2];
    }
    c[0] /= N; c[1] /= N; c[2] /= N;
    // far 变体：t1 anchor 换成「离 centroid 最远」的环邻居（距离与起点/方向无关）。
    {
        const int variant = resolveFrameVariant();
        if (variant == 2) {
            int best = i0;
            double bestD = -1.0;
            for (int i = 0; i < N; ++i) {
                const double dx = P[3 * i + 0] - c[0];
                const double dy = P[3 * i + 1] - c[1];
                const double dz = P[3 * i + 2] - c[2];
                const double d = dx * dx + dy * dy + dz * dz;
                // 同距用格坐标字典序 tie-break（确定性）。
                if (d > bestD + 1e-18) {
                    bestD = d; best = i;
                } else if (std::fabs(d - bestD) <= 1e-18 && i < best) {
                    best = i;
                }
            }
            i0 = best;
        }
    }
    double a[3] = {P[3 * i0 + 0] - c[0], P[3 * i0 + 1] - c[1], P[3 * i0 + 2] - c[2]};
    double b[3] = {P[3 * i1 + 0] - c[0], P[3 * i1 + 1] - c[1], P[3 * i1 + 2] - c[2]};
    double n[3] = {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                   a[0] * b[1] - a[1] * b[0]};
    if (resolveFrameVariant() == 1) {
        // Newell 法向（best-fit 平面）；符号由 cross(a,b) 定（双端同一规则）。
        double nw[3] = {0.0, 0.0, 0.0};
        for (int i = 0; i < N; ++i) {
            const int j = (i + 1) % N;
            const double ax = P[3 * i + 0] - c[0], ay = P[3 * i + 1] - c[1],
                         az = P[3 * i + 2] - c[2];
            const double bx = P[3 * j + 0] - c[0], by = P[3 * j + 1] - c[1],
                         bz = P[3 * j + 2] - c[2];
            nw[0] += ay * bz - az * by;
            nw[1] += az * bx - ax * bz;
            nw[2] += ax * by - ay * bx;
        }
        const double nwl = std::sqrt(nw[0] * nw[0] + nw[1] * nw[1] + nw[2] * nw[2]);
        if (nwl > 1e-12) {
            const double sgn = (nw[0] * n[0] + nw[1] * n[1] + nw[2] * n[2]) < 0.0 ? -1.0 : 1.0;
            n[0] = sgn * nw[0]; n[1] = sgn * nw[1]; n[2] = sgn * nw[2];
        }
    }
    double nl = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    if (nl <= 1e-12) {
        // 退化（a、b 近共线）：Newell 法向。
        for (int i = 0; i < N; ++i) {
            const int j = (i + 1) % N;
            const double ax = P[3 * i + 0] - c[0], ay = P[3 * i + 1] - c[1],
                         az = P[3 * i + 2] - c[2];
            const double bx = P[3 * j + 0] - c[0], by = P[3 * j + 1] - c[1],
                         bz = P[3 * j + 2] - c[2];
            n[0] += ay * bz - az * by;
            n[1] += az * bx - ax * bz;
            n[2] += ax * by - ay * bx;
        }
        nl = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        if (nl <= 1e-12) return f;
    }
    f.n[0] = n[0] / nl; f.n[1] = n[1] / nl; f.n[2] = n[2] / nl;
    // t1 = a 去法向分量后归一化（a 由 canonical anchor 定义 ⇒ 双端一致）。
    double dot = a[0] * f.n[0] + a[1] * f.n[1] + a[2] * f.n[2];
    double t1[3] = {a[0] - dot * f.n[0], a[1] - dot * f.n[1], a[2] - dot * f.n[2]};
    double tl = std::sqrt(t1[0] * t1[0] + t1[1] * t1[1] + t1[2] * t1[2]);
    if (tl <= 1e-12) return f;
    f.t1[0] = t1[0] / tl; f.t1[1] = t1[1] / tl; f.t1[2] = t1[2] / tl;
    f.t2[0] = f.n[1] * f.t1[2] - f.n[2] * f.t1[1];
    f.t2[1] = f.n[2] * f.t1[0] - f.n[0] * f.t1[2];
    f.t2[2] = f.n[0] * f.t1[1] - f.n[1] * f.t1[0];
    f.valid = true;
    return f;
}

// ---- 可逆整数 lifting 的 2D / 3D 旋转 --------------------------------------
inline int rdLiftRound(double v) { return static_cast<int>(std::llround(v)); }

/// 2D rotation [[c,-s],[s,c]] 的 3-lift 分解；近 π 先做取负归一化（见 AUDIT §4）。
inline void rdLift2dForward(int& u, int& v, double c, double s) {
    if (c < 0.0) {
        u = -u;
        v = -v;
        c = -c;
        s = -s;
    }
    if (std::fabs(s) < 1e-15) return;
    const double a = (c - 1.0) / s;
    u += rdLiftRound(a * static_cast<double>(v));
    v += rdLiftRound(s * static_cast<double>(u));
    u += rdLiftRound(a * static_cast<double>(v));
}

inline void rdLift2dInverse(int& u, int& v, double c, double s) {
    if (c < 0.0) {
        u = -u;
        v = -v;
        c = -c;
        s = -s;
    }
    if (std::fabs(s) < 1e-15) return;
    const double a = (c - 1.0) / s;
    u -= rdLiftRound(a * static_cast<double>(v));
    v -= rdLiftRound(s * static_cast<double>(u));
    u -= rdLiftRound(a * static_cast<double>(v));
}

inline void rdEulerZYX(const RdFrame& f, double& al, double& be, double& ga) {
    double sBeta = -f.t2[0];  // r20 = -sin(beta)，R 的行 = (n, t1, t2)
    if (sBeta > 1.0) sBeta = 1.0;
    if (sBeta < -1.0) sBeta = -1.0;
    be = std::asin(sBeta);
    if (std::fabs(std::cos(be)) > 1e-8) {
        ga = std::atan2(f.t2[1], f.t2[2]);  // r21, r22
        al = std::atan2(f.t1[0], f.n[0]);   // r10, r00
    } else {
        ga = 0.0;
        al = std::atan2(-f.n[1], f.t1[1]);
    }
}

inline void rdFrameForward(int& x, int& y, int& z, const RdFrame& f) {
    double al = 0.0, be = 0.0, ga = 0.0;
    rdEulerZYX(f, al, be, ga);
    rdLift2dForward(y, z, std::cos(ga), std::sin(ga));   // Rx(gamma)
    rdLift2dForward(x, z, std::cos(be), -std::sin(be));  // Ry(beta)
    rdLift2dForward(x, y, std::cos(al), std::sin(al));   // Rz(alpha)
}

inline void rdFrameInverse(int& x, int& y, int& z, const RdFrame& f) {
    double al = 0.0, be = 0.0, ga = 0.0;
    rdEulerZYX(f, al, be, ga);
    rdLift2dInverse(x, y, std::cos(al), std::sin(al));
    rdLift2dInverse(x, z, std::cos(be), -std::sin(be));
    rdLift2dInverse(y, z, std::cos(ga), std::sin(ga));
}

// ---- H1：论文 §3.2 prioritized gate selection（gate-queue 次序）------------
//
// 6 级 priority（论文 §3.2；null-patch 相关 2/4/6 级在本 removability 链下
// 不可达，实测 null_patch=0，故只实现 6/4/2）：
//   6 = ⊕⊖ gate，且 ⊖ 端点有 2–3 条邻边在 conquered region；
//   4 = ⊕⊕/⊖⊖ gate，两端都满足 2–3 条 conquered 邻边；
//   2 = 其余。
// 引擎在 encoder/decoder **两侧**用同一策略出队（decodeLayer 镜像），符号流
// 因此仍由同一 gate 序列产生/消费。
/// 默认 **关**：H1 的 encoder/decoder gate 顺序镜像尚未对齐（PGS=1 时
/// decode 进程在 8+ 分钟内 hung，且 encoder artifact 比 FIFO 差 +53 B）。
/// 保留实现供后续调试（H3 落地后再试），不进入 rd_cq 默认路径。
inline bool rdcqPgsEnabled() {
    const char* v = std::getenv("PMSF_RD_CQ_PGS");
    if (v == nullptr || *v == '\0') return false;
    return std::strcmp(v, "0") != 0;
}

class RDPgsGatePriority final : public GatePriorityPolicy {
  public:
    bool isActive() const override { return rdcqPgsEnabled(); }

    bool higherPriority(const GatePriorityView& a,
                        const GatePriorityView& b) const override {
        return pgsLevel(a) > pgsLevel(b);
    }

  private:
    static int pgsLevel(const GatePriorityView& v) {
        if (v.frontSign == 0 || v.backSign == 0) return 2;
        const bool opposite = (v.frontSign != v.backSign);
        if (opposite) {
            // ⊖ 端点（sign==2）满足 low-conquered（2–3 条）。
            const bool minusOk = (v.frontSign == 2 && v.frontConqOk) ||
                                 (v.backSign == 2 && v.backConqOk);
            return minusOk ? 6 : 2;
        }
        return (v.frontConqOk && v.backConqOk) ? 4 : 2;
    }
};


// ---- H4：ResidualQuantizer -------------------------------------------------
class RDFrenetQuantizer final : public ResidualQuantizer {
  public:
    /// H3 用：encoder 侧记录 (centerId, ringIds, targetCell) 的回调（可空）。
    std::function<void(const PatchView&, const int[3])> onTransform;
    /// frameOn=false 时 transform/inverse 均为恒等（只借它做 H3 记录）。
    explicit RDFrenetQuantizer(bool frameOn = true) : frameOn_(frameOn) {}

    /// 论文 Eq.(3)：残差投影到局部 Frenet 系（可逆整数 lifting ⇒ 几何逐格无损）。
    void transform(const int in[3], int out[3], const PatchView& p) const override {
        if (onTransform) onTransform(p, in);
        const RdFrame f = frameFromPatch(p);
        if (!f.valid) {
            out[0] = in[0];
            out[1] = in[1];
            out[2] = in[2];
            return;
        }
        if (!frameOn_) {
            out[0] = in[0];
            out[1] = in[1];
            out[2] = in[2];
            return;
        }
        int x = in[0], y = in[1], z = in[2];
        rdFrameForward(x, y, z, f);
        out[0] = x;
        out[1] = y;
        out[2] = z;
    }

    void inverseTransform(const int in[3], int out[3],
                          const PatchView& p) const override {
        const RdFrame f = frameFromPatch(p);
        if (!f.valid) {
            out[0] = in[0];
            out[1] = in[1];
            out[2] = in[2];
            return;
        }
        if (!frameOn_) {
            out[0] = in[0];
            out[1] = in[1];
            out[2] = in[2];
            return;
        }
        int x = in[0], y = in[1], z = in[2];
        rdFrameInverse(x, y, z, f);
        out[0] = x;
        out[1] = y;
        out[2] = z;
    }

  private:
    bool frameOn_ = true;
};

// ---- H3：论文 §3.1 Eq.(2) curvature prediction（coarse-graph 对称）---------
//
// 对称契约（与 MMSE H3 同一思路，难点不在公式而在"两端看到同一个图"）：
//  * encoder：引擎运行期 H3 返回默认 mean（引擎只记 delta_mean）；H4 hook 额外
//    记录每个 gate 的 (centerId, ringIds, targetCell)。**一层结束后**（mesh 正好是
//    该层的 coarse 网格）`finalizeLayer()` 在 coarse 图上重算论文预测并覆盖该层
//    delta（同层内已删点不可见）。
//  * decoder：每次插入前，用当前网格**去掉本中心与同层已插入中心**后的图
//    （= coarse 网格）重算同一预测。整数 cell 域求和/取均值 ⇒ 与遍历顺序无关、
//    双端位精确。
//  * r_i = cell(v_i) − cell(v̂_i)，v̂_i = i 的粗图一环均值（把中心 c 用 v̂_c 代）；
//    r̂ = mean_i r_i；pred_cell = cell(barycenter(ring c)) + r̂；
//    delta = cell(target) − pred_cell。这就是论文 Eq.(2) 的二阶（curvature）预测。
inline int rdCellOf(double c, double min, double step) {
    int q = static_cast<int>(std::ceil((c - min) / step)) - 1;
    if (q == -1) q = 0;
    return q;
}
inline double rdToWorldCell(int q, double min, double step) {
    return min + (static_cast<double>(q) + 0.5) * step;
}
inline double rdRoundHalfAway(double v) {
    return (v >= 0.0) ? std::floor(v + 0.5) : std::ceil(v - 0.5);
}
inline double rdVertexCoord(MCGAL::Vertex* v, int k) {
    return (k == 0) ? v->x() : (k == 1 ? v->y() : v->z());
}
inline MCGAL::Vertex* rdcqVertexById(std::uint32_t id) {
    return MCGAL::contextPool.getVertexByIndexInSubPool(
        DEFAULT_MESH_ID, static_cast<int>(id));
}

/// H3 的 coarse-graph 预测（cell 域）。`ringIds`/`ringPos` 为环序；
/// `excluded` = 本层已插入中心（decoder）或空（encoder）。返回 false 表示无法算。
inline bool rdcqCurvaturePredCell(const std::vector<std::uint32_t>& ringIds,
                                  const std::vector<std::array<double, 3>>& ringPos,
                                  std::uint32_t centerId,
                                  const std::unordered_set<std::uint32_t>& excluded,
                                  const double bboxMin[3], double step, int predCell[3]) {
    const std::size_t n = ringIds.size();
    if (n == 0 || ringPos.size() != n || step <= 0.0) return false;
    long long bary[3] = {0, 0, 0};
    int ringCell[256][3];
    if (n > 256) return false;
    for (std::size_t i = 0; i < n; ++i) {
        for (int k = 0; k < 3; ++k) {
            ringCell[i][k] = rdCellOf(ringPos[i][k], bboxMin[k], step);
            bary[k] += ringCell[i][k];
        }
    }
    int baryCell[3];
    for (int k = 0; k < 3; ++k)
        baryCell[k] = static_cast<int>(rdRoundHalfAway((double)bary[k] / (double)n));

    double rsum[3] = {0.0, 0.0, 0.0};
    std::size_t rcnt = 0;
    char ringDet[2048];
    ringDet[0] = '\0';
    std::size_t ringDetLen = 0;
    for (std::size_t i = 0; i < n; ++i) {
        MCGAL::Vertex* vi = rdcqVertexById(ringIds[i]);
        if (!vi || vi->isRemoved()) continue;
        long long sum[3] = {0, 0, 0};
        int cnt = 0;
        MCGAL::Halfedge* hs[64];
        const int hn = ringNeighbors(vi, hs, 64);
        for (int j = 0; j < hn; ++j) {
            MCGAL::Vertex* w = tgt(hs[j]);
            if (!w || w->isRemoved()) continue;
            if (w->poolId() == centerId) continue;
            if (!excluded.empty() && excluded.count(w->poolId())) continue;
            for (int k = 0; k < 3; ++k)
                sum[k] += rdCellOf(rdVertexCoord(w, k), bboxMin[k], step);
            ++cnt;
        }
        if (cnt == 0) continue;
        // 中心 c 的替代：coarse 图上 c 不在，但它在 i 的一环里（i 是 c 的环邻居
        // ⇒ 相邻）⇒ 用 v̂_c = baryCell 补一格（双端同一规则）。
        for (int k = 0; k < 3; ++k)
            sum[k] += baryCell[k];
        ++cnt;
        int vhat3[3] = {0, 0, 0};
        for (int k = 0; k < 3; ++k) {
            vhat3[k] = static_cast<int>(
                rdRoundHalfAway((double)sum[k] / (double)cnt));
            rsum[k] += (double)(ringCell[i][k] - vhat3[k]);
        }
        ++rcnt;
        if (ringDetLen < sizeof(ringDet) - 128) {
            ringDetLen += (std::size_t)std::snprintf(
                ringDet + ringDetLen, sizeof(ringDet) - ringDetLen,
                " [%d,%d,%d cnt=%d vhat=%d,%d,%d nbrs=%d]", ringCell[i][0],
                ringCell[i][1], ringCell[i][2], cnt, vhat3[0], vhat3[1],
                vhat3[2], hn);
        }
    }
    for (int k = 0; k < 3; ++k) {
        predCell[k] = baryCell[k] +
                      static_cast<int>(rdRoundHalfAway(rsum[k] / (double)rcnt));
    }
    return true;
}

/// H3/H4 共用的 encoder/decoder 状态（per-layer 记录）。
class RDCQuestGeomState {
  public:
    bool encoderMode = true;
    bool h3Enabled = true;

    // ---- encoder：per-gate 记录（H4 hook 写）----
    struct EncGate {
        std::uint32_t centerId = 0;
        std::vector<std::uint32_t> ringIds;
        int targetCell[3] = {0, 0, 0};
    };
    std::vector<EncGate> encGates;
    std::vector<std::pair<std::size_t, std::size_t>> layerRanges;

    // ---- decoder：同层已插入中心 ----
    std::unordered_set<std::uint32_t> insertedCenters;

    // encoder：H4.transform 里记录（此时 vdel 还在网格里 ⇒ 能取 target 位置）。
    void recordEncoderGate(const PatchView& p, const int /*deltaMean*/[3]) {
        EncGate g;
        g.centerId = p.centerId;
        g.ringIds.assign(p.neighborIds, p.neighborIds + p.neighborCount);
        MCGAL::Vertex* v = rdcqVertexById(p.centerId);
        for (int k = 0; k < 3; ++k)
            g.targetCell[k] = rdCellOf(rdVertexCoord(v, k), p.bboxMin[k], p.quantStep);
        encGates.push_back(std::move(g));
    }
    void beginCollectLayer() { layerRanges.emplace_back(encGates.size(), encGates.size()); }
    void endCollectLayer() {
    }

    // decoder：一层开始（由 reconstructor 子类调用）。
    void beginDecoderLayer() { insertedCenters.clear(); }

    /// encoder：一层结束后在 coarse 图上重算论文预测并覆盖该层 delta。
    void finalizeLayer(conquest_blob::LayerRecord& layer, const conquest_blob::Stream& stream,
                       std::size_t layerIdx) {
        const std::size_t b = layerRanges[layerIdx].first;
        const std::size_t e = layerRanges[layerIdx].second;
        const double* mn = stream.bboxMin;
        const double step = stream.step;
        for (std::size_t k = 0; k < layer.deltas.size(); ++k) {
            const EncGate& g = encGates[b + k];
            std::vector<std::uint32_t> ids;
            std::vector<std::array<double, 3>> pos;
            for (std::uint32_t id : g.ringIds) {
                MCGAL::Vertex* v = rdcqVertexById(id);
                ids.push_back(id);
                pos.push_back({v->x(), v->y(), v->z()});
            }
            int predCell[3];
            int delta[3];
            for (int k2 = 0; k2 < 3; ++k2)
                delta[k2] = g.targetCell[k2] - predCell[k2];
            // frame 变换：用 coarse 环位置（decoder 插入时看到的就是这组位置；
            // canonical frame 对环序不敏感）。
            double nb[3 * 256];
            std::uint32_t nbIds[256];
            const std::size_t nn = ids.size() < 256 ? ids.size() : 256;
            for (std::size_t i = 0; i < nn; ++i) {
                nb[3 * i + 0] = pos[i][0];
                nb[3 * i + 1] = pos[i][1];
                nb[3 * i + 2] = pos[i][2];
                nbIds[i] = ids[i];
            }
            if (resolveGeomMode() == GeomMode::Native) {
                // H4 关：decoder 的 inverseTransform 也是恒等 ⇒ 存原始 delta。
                layer.deltas[k] = {delta[0], delta[1], delta[2]};
                continue;
            }
            PatchView pv;
            pv.neighborPos = nb;
            pv.neighborCount = static_cast<int>(nn);
            pv.quantStep = step;
            pv.bboxMin = mn;
            pv.neighborIds = nbIds;
            pv.centerId = g.centerId;
            RDFrenetQuantizer q;
            int out[3];
            q.transform(delta, out, pv);
            layer.deltas[k] = {out[0], out[1], out[2]};
        }
    }

    /// decoder：插入前的论文预测（coarse 图）。返回 false ⇒ 让引擎用默认 mean。
    bool decoderPredict(const PatchView& p, double out[3]) {
        if (encoderMode || !h3Enabled) return false;
        std::vector<std::uint32_t> ids;
        std::vector<std::array<double, 3>> pos;
        for (int i = 0; i < p.neighborCount; ++i) {
            const std::uint32_t id = p.neighborIds[i];
            if (id == p.centerId) continue;
            if (insertedCenters.count(id)) continue;
            ids.push_back(id);
            const double* q = p.neighborPos + 3 * i;
            pos.push_back({q[0], q[1], q[2]});
        }
        if (ids.size() < 3) {
            insertedCenters.insert(p.centerId);
            return false;
        }
        int predCell[3];
        if (!rdcqCurvaturePredCell(ids, pos, p.centerId, insertedCenters,
                                   p.bboxMin, p.quantStep, predCell)) {
            insertedCenters.insert(p.centerId);
            return false;
        }
        const bool firstGateOfLayer = insertedCenters.empty();
        insertedCenters.insert(p.centerId);
        for (int k = 0; k < 3; ++k)
            out[k] = rdToWorldCell(predCell[k], p.bboxMin[k], p.quantStep);
        return true;
    }
};

/// H3 `GeometryPredictor`：encoder 期返回默认 mean（finalize 才用它），
/// decoder 期返回 coarse-graph curvature 预测。
class RDCQuestGeomPredictor final : public GeometryPredictor {
  public:
    explicit RDCQuestGeomPredictor(std::shared_ptr<RDCQuestGeomState> st)
        : st_(std::move(st)) {}
    void predict(const PatchView& p, double out[3]) const override {
        if (st_ && st_->decoderPredict(p, out)) return;
        GeometryPredictor::predict(p, out);  // 默认 = 一圈算术平均
    }

  private:
    std::shared_ptr<RDCQuestGeomState> st_;
};

/// 编码/解码共用的 hook 集合：H1 = PGS（默认开，env `PMSF_RD_CQ_PGS=0` 可关）；
/// H4 = 局部 Frenet（`PMSF_RD_CQ_GEOM=native` 可关）；H3/H5 留引擎默认。
inline EngineHooksPtr makeRDCQuestHooks(std::shared_ptr<RDCQuestGeomState> st = nullptr,
                                        bool encoderMode = true) {
    if (st) {
        st->encoderMode = encoderMode;
        st->h3Enabled = h3EnabledFromEnv();
    }
    const bool geomOn = resolveGeomMode() != GeomMode::Native;
    const bool pgsOn = rdcqPgsEnabled();
    const bool h3On = st && st->h3Enabled;
    if (!geomOn && !pgsOn && !h3On) return nullptr;
    auto hooks = std::make_shared<EngineHooks>();
    if (pgsOn) hooks->gatePriority = std::make_shared<RDPgsGatePriority>();
    if (geomOn || (h3On && encoderMode))
        hooks->residualQuantizer = std::make_shared<RDFrenetQuantizer>(geomOn);
    if (h3On) {
        // H3 的 encoder 期预测返回默认 mean；finalize 在层末重算并覆盖 delta。
        // 注意不要凭空创建 H4：`GEOM=native` 时 decoder 侧也必须是恒等变换。
        hooks->geometryPredictor = std::make_shared<RDCQuestGeomPredictor>(st);
        if (encoderMode && hooks->residualQuantizer) {
            auto q = std::static_pointer_cast<RDFrenetQuantizer>(
                hooks->residualQuantizer);
            q->onTransform = [st](const PatchView& p, const int in[3]) {
                st->recordEncoderGate(p, in);
            };
        }
    }
    return hooks;
}

}  // namespace rd_cq
}  // namespace pmsf

#endif  // PMSF_RD_HOOKS_RD_CQUEST_HOOKS_H
