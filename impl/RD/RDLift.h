// SPDX-License-Identifier: Apache-2.0
//
// RDLift.h — task-43 route A（canonical deterministic traversal + decoder replay）
// 的几何 lifting：RD 论文 §3.1 Eq.(3) 的局部 Frenet 系投影，用**可逆整数
// lifting** 实现（float 旋转 + round 会在预测链上累积漂移；整数 lifting 逐位可逆）。
//
// 本文件**只做纯数学**（不认识 MCGAL 类型）：输入是一圈邻居的世界坐标（环序，
// 两端由同一个 ringNeighbors 取得）+ 量化网格参数。编码端与解码端各自用**完全相同**
// 的输入（见 RDCanonicalTraversal.h 的 ringGeometryInputs）⇒ frame 逐位一致 ⇒
// transform / inverseTransform 严格互逆。
//
// 机制（与 RDCQuestHooks 的 H4 同一套，task-19 已证明可逆；这里剥掉 hook 接口与
// 诊断，只保留 frame 构造 + lifting，供 route A 的 conquest 遍历直接调用）：
//   1) canonical ring frame：
//        - anchor a = 量化格坐标字典序最小的环邻居（与起点/方向无关）；
//        - anchor b = 次小（且格坐标不同于 a）的环邻居；
//        - n = Newell 法向（best-fit 平面），符号由 cross(a,b) 定（双端同一规则）；
//        - t1 = (a - centroid) 去法向分量后归一化；t2 = n × t1。
//   2) R = Rz(α)Ry(β)Rx(γ)（Euler ZYX），每个 2D 旋转 = 3 个 lifting shear
//      `u += round(k·v)`；近 π 先做 R(θ)=N·R(θ−π) 归一化（否则 k→∞ 放大 round 误差）。
//
// 量化：`cellFromPos` 用 floor((c−min)/step) 并夹到 [0, 2^bits−1]；`cellCenter` 复刻
// 解码端 base-mesh 读回那条 **float** 公式（`bm + (q+0.5f)*step`），使两端世界逐位相同。
#ifndef PMSF_IMPL_RD_RD_LIFT_H
#define PMSF_IMPL_RD_RD_LIFT_H

#include <cmath>
#include <cstdint>

namespace rd_canon {

/// 量化网格：与框架 `Mesh::bboxMin / f_quantStep / i_nbQuantBits` 同源。
struct QuantGrid {
    double bboxMin[3] = {0.0, 0.0, 0.0};
    double step = 0.0;
    int quantBits = 12;
    bool valid() const {
        return step > 0.0;
    }
};

inline int quantMaxCell(const QuantGrid& g) {
    return (g.quantBits > 0 && g.quantBits < 31) ? ((1 << g.quantBits) - 1)
                                                 : 0x7fffffff;
}

/// 世界坐标 → 量化格号。floor + 夹持（与容器写 base mesh 的 clamp 同口径）。
inline int cellFromPos(const QuantGrid& g, int axis, double c) {
    int q = static_cast<int>(std::floor((c - g.bboxMin[axis]) / g.step));
    if (q < 0) q = 0;
    const int maxV = quantMaxCell(g);
    if (q > maxV) q = maxV;
    return q;
}

/// 量化格号 → 世界坐标。**必须与解码端 base-mesh 读回公式逐字一致**：
///   bboxMin + (q + 0.5f) * quantStep   （全程 float）
inline double cellCenter(const QuantGrid& g, int axis, int q) {
    return static_cast<double>(static_cast<float>(g.bboxMin[axis]) +
                               (static_cast<float>(q) + 0.5f) *
                                   static_cast<float>(g.step));
}

struct RdFrame {
    double n[3] = {0.0, 0.0, 0.0};
    double t1[3] = {0.0, 0.0, 0.0};
    double t2[3] = {0.0, 0.0, 0.0};
    bool valid = false;
};

/// 由一圈邻居世界坐标（环序，长度 N）构造 canonical ring frame。
/// @p P  长度 3*N 的邻居坐标（xyz 连续）
inline RdFrame frameFromRingPositions(const double* P, int N, const QuantGrid& g) {
    RdFrame f;
    if (N < 3 || P == nullptr || !g.valid()) return f;
    auto cell = [&](int i, int k) -> long long {
        return static_cast<long long>(cellFromPos(g, k, P[3 * i + k]));
    };
    // canonical anchor：按 (x,y,z) 字典序取最小与"次小且不同格"的邻居。
    int i0 = -1, i1 = -1;
    for (int i = 0; i < N; ++i) {
        if (i0 < 0) {
            i0 = i;
            continue;
        }
        const long long ax = cell(i, 0), ay = cell(i, 1), az = cell(i, 2);
        const long long bx = cell(i0, 0), by = cell(i0, 1), bz = cell(i0, 2);
        const bool less = (ax < bx) || (ax == bx && (ay < by || (ay == by && az < bz)));
        if (less) {
            i1 = i0;
            i0 = i;
        } else if (i1 < 0) {
            const bool different = (ax != bx) || (ay != by) || (az != bz);
            if (different) i1 = i;
        } else {
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
    c[0] /= N;
    c[1] /= N;
    c[2] /= N;
    double a[3] = {P[3 * i0 + 0] - c[0], P[3 * i0 + 1] - c[1], P[3 * i0 + 2] - c[2]};
    double b[3] = {P[3 * i1 + 0] - c[0], P[3 * i1 + 1] - c[1], P[3 * i1 + 2] - c[2]};
    double n[3] = {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                   a[0] * b[1] - a[1] * b[0]};
    // Newell 法向（best-fit 平面）；符号由 cross(a,b) 定（与环起点/方向无关）。
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
        const double sgn =
            (nw[0] * n[0] + nw[1] * n[1] + nw[2] * n[2]) < 0.0 ? -1.0 : 1.0;
        n[0] = sgn * nw[0];
        n[1] = sgn * nw[1];
        n[2] = sgn * nw[2];
    }
    double nl = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    if (nl <= 1e-12) return f;  // 退化
    f.n[0] = n[0] / nl;
    f.n[1] = n[1] / nl;
    f.n[2] = n[2] / nl;
    double dot = a[0] * f.n[0] + a[1] * f.n[1] + a[2] * f.n[2];
    double t1[3] = {a[0] - dot * f.n[0], a[1] - dot * f.n[1], a[2] - dot * f.n[2]};
    double tl = std::sqrt(t1[0] * t1[0] + t1[1] * t1[1] + t1[2] * t1[2]);
    if (tl <= 1e-12) return f;
    f.t1[0] = t1[0] / tl;
    f.t1[1] = t1[1] / tl;
    f.t1[2] = t1[2] / tl;
    f.t2[0] = f.n[1] * f.t1[2] - f.n[2] * f.t1[1];
    f.t2[1] = f.n[2] * f.t1[0] - f.n[0] * f.t1[2];
    f.t2[2] = f.n[0] * f.t1[1] - f.n[1] * f.t1[0];
    f.valid = true;
    return f;
}

// ---- 可逆整数 lifting 的 2D / 3D 旋转 --------------------------------------
inline int rdLiftRound(double v) {
    return static_cast<int>(std::llround(v));
}

/// 2D rotation [[c,-s],[s,c]] 的 3-lift 分解；近 π 先做取负归一化。
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
        ga = std::atan2(f.t2[1], f.t2[2]);
        al = std::atan2(f.t1[0], f.n[0]);
    } else {
        ga = 0.0;
        al = std::atan2(-f.n[1], f.t1[1]);
    }
}

inline void frameForward(int& x, int& y, int& z, const RdFrame& f) {
    double al = 0.0, be = 0.0, ga = 0.0;
    rdEulerZYX(f, al, be, ga);
    rdLift2dForward(y, z, std::cos(ga), std::sin(ga));   // Rx(gamma)
    rdLift2dForward(x, z, std::cos(be), -std::sin(be));  // Ry(beta)
    rdLift2dForward(x, y, std::cos(al), std::sin(al));   // Rz(alpha)
}

inline void frameInverse(int& x, int& y, int& z, const RdFrame& f) {
    double al = 0.0, be = 0.0, ga = 0.0;
    rdEulerZYX(f, al, be, ga);
    rdLift2dInverse(x, y, std::cos(al), std::sin(al));
    rdLift2dInverse(x, z, std::cos(be), -std::sin(be));
    rdLift2dInverse(y, z, std::cos(ga), std::sin(ga));
}

}  // namespace rd_canon

#endif  // PMSF_IMPL_RD_RD_LIFT_H
