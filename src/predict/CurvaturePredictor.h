// SPDX-License-Identifier: Apache-2.0
//
// CurvaturePredictor.h —— PR-V (2026-05-13)
// 论文 Ahn et al. 2011 §3.1 的 2nd-order curvature prediction。
//
// 公式（论文 Eq. 2-3）：
//   r_c   = v_c - v_hat_c                           (一阶 barycentric residual)
//   r_hat = (1/|N|) * Σ_{i∈N} r1_i                   (邻居一阶残差均值)
//   α'    = round(n · (r_c - λ_α · r_hat))            (法向投影)
//   β'    = round(t1 · (r_c - λ_β · r_hat))           (切向投影 1)
//   γ'    = round(t2 · (r_c - λ_γ · r_hat))           (切向投影 2)
//
// 其中 λ_{α,β,γ} ∈ {0, 0.5, 1}，per-layer 由 encoder 选择最优值。
// 双端共用：encoder/decoder 使用相同的 local frame 和 r_hat 计算，保证 deterministic。
//
// 详见 docs/paper_alignment/PR_U_PAPER_ALIGN_PLAN.md §4。

#ifndef PMSF_PREDICT_CURVATURE_PREDICTOR_H
#define PMSF_PREDICT_CURVATURE_PREDICTOR_H

#include <array>
#include <memory>
#include <vector>

namespace MCGAL {
class Vertex;
class Point;
class PointInt;
class Vector3;
class Mesh;
}  // namespace MCGAL

namespace pmsf {

/// 论文的局部坐标系 (normal, tangent1, tangent2)，orthonormal。
struct LocalFrame {
    double n[3];   // unit normal
    double t1[3];  // first tangent
    double t2[3];  // second tangent (n × t1)
};

/// 从 ring vertex 浮点坐标（顺时针环）计算局部坐标系。
/// 法向用 Newell's method；t1 为 centroid→v[0] 在切平面上的投影；t2 = n × t1。
LocalFrame computeLocalFrame(const std::vector<MCGAL::Vertex*>& ringVerts);

/// 曲率预测编码结果。
struct CurvatureEncodeResult {
    int alpha;        // 法向投影残差（round to int）
    int beta;         // 切向 1 投影残差
    int gamma;        // 切向 2 投影残差
    int lambdaIdx[3]; // leaky factor 索引：0→0, 1→0.5, 2→1
};

/// 曲率预测编码：给定顶点量化位置 + ring + 邻居 r1 值，
/// 返回投影后的 (α', β', γ') 残差 + 最优 leaky factor。
///
/// r_c_int  = (rc_x, rc_y, rc_z)，在量化整数坐标空间。
/// r1_neighbors  = ring 顶点对应的 kVPropRDR1 值，长度必须 = ringVerts.size()。
CurvatureEncodeResult curvatureEncode(
    int rc_x, int rc_y, int rc_z,
    const std::vector<MCGAL::Vertex*>& ringVerts,
    const std::vector<MCGAL::PointInt*>& r1Neighbors);

/// 曲率预测编码（固定 leaky factor 版本）。
/// 使用统一的 λ 值（不 per-vertex 优化），encoder/decoder 必须用相同 λ。
std::array<int, 3> curvatureEncodeFixed(
    int rc_x, int rc_y, int rc_z,
    const std::vector<MCGAL::Vertex*>& ringVerts,
    const std::vector<MCGAL::PointInt*>& r1Neighbors,
    int lambdaIdx0, int lambdaIdx1, int lambdaIdx2);

/// 曲率预测解码：给定 (α', β', γ') + leaky factor + ring + 邻居 r1，
/// 重建 r_c（在量化整数坐标空间）。
///
/// 返回 (rc_x, rc_y, rc_z) 的整数重建值。
std::array<int, 3> curvatureDecode(
    int alpha, int beta, int gamma,
    int lambdaIdx0, int lambdaIdx1, int lambdaIdx2,
    const std::vector<MCGAL::Vertex*>& ringVerts,
    const std::vector<MCGAL::PointInt*>& r1Neighbors);

/// 将 leaky factor 索引转换为实际 float 值。
inline double lambdaValue(int idx) {
    const double table[3] = {0.0, 0.5, 1.0};
    return table[idx >= 0 && idx <= 2 ? idx : 1];
}

/// 打包 3 个 leaky factor 索引为 1 byte: (λ_α << 4) | (λ_β << 2) | λ_γ
inline unsigned char packLeakyFactors(int la, int lb, int lg) {
    return static_cast<unsigned char>(
        ((la & 3) << 4) | ((lb & 3) << 2) | (lg & 3));
}

/// 解包 1 byte → 3 个 leaky factor 索引
inline void unpackLeakyFactors(unsigned char packed, int& la, int& lb, int& lg) {
    la = (packed >> 4) & 3;
    lb = (packed >> 2) & 3;
    lg = packed & 3;
}

}  // namespace pmsf

#endif  // PMSF_PREDICT_CURVATURE_PREDICTOR_H
