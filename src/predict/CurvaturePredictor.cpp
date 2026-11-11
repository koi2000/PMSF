// SPDX-License-Identifier: Apache-2.0
//
// CurvaturePredictor.cpp —— PR-V (2026-05-13)
// 论文 §3.1 2nd-order curvature prediction 实现。
// 所有浮点运算使用 double 精度，保证 encoder/decoder deterministic。

#include "predict/CurvaturePredictor.h"

#include <cmath>
#include <cstdio>

#include "Point.h"
#include "Vertex.h"

namespace pmsf {

LocalFrame computeLocalFrame(const std::vector<MCGAL::Vertex*>& ringVerts) {
    LocalFrame f{};
    const int N = static_cast<int>(ringVerts.size());
    if (N < 3) return f;

    // 统计有效顶点
    int validN = 0;
    for (auto* v : ringVerts) {
        if (v && !v->isRemoved()) ++validN;
    }
    if (validN < 3) return f;

    // Newell's method for robust normal
    double nx = 0.0, ny = 0.0, nz = 0.0;
    for (int i = 0; i < N; ++i) {
        MCGAL::Vertex* v0 = ringVerts[i];
        MCGAL::Vertex* v1 = ringVerts[(i + 1) % N];
        if (!v0 || v0->isRemoved() || !v1 || v1->isRemoved()) continue;
        double x0 = static_cast<double>(v0->point()[0]);
        double y0 = static_cast<double>(v0->point()[1]);
        double z0 = static_cast<double>(v0->point()[2]);
        double x1 = static_cast<double>(v1->point()[0]);
        double y1 = static_cast<double>(v1->point()[1]);
        double z1 = static_cast<double>(v1->point()[2]);
        nx += (y0 - y1) * (z0 + z1);
        ny += (z0 - z1) * (x0 + x1);
        nz += (x0 - x1) * (y0 + y1);
    }
    double len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (len < 1e-12) return f;
    nx /= len;
    ny /= len;
    nz /= len;
    f.n[0] = nx;
    f.n[1] = ny;
    f.n[2] = nz;

    // 计算 centroid
    double cx = 0.0, cy = 0.0, cz = 0.0;
    {
        int cnt = 0;
        for (auto* v : ringVerts) {
            if (!v || v->isRemoved()) continue;
            cx += static_cast<double>(v->point()[0]);
            cy += static_cast<double>(v->point()[1]);
            cz += static_cast<double>(v->point()[2]);
            ++cnt;
        }
        if (cnt > 0) {
            cx /= cnt;
            cy /= cnt;
            cz /= cnt;
        }
    }

    // t1: centroid → first valid vertex, projected to tangent plane
    double t1x = 0.0, t1y = 0.0, t1z = 0.0;
    for (auto* v : ringVerts) {
        if (!v || v->isRemoved()) continue;
        t1x = static_cast<double>(v->point()[0]) - cx;
        t1y = static_cast<double>(v->point()[1]) - cy;
        t1z = static_cast<double>(v->point()[2]) - cz;
        break;
    }
    double dot = t1x * nx + t1y * ny + t1z * nz;
    t1x -= dot * nx;
    t1y -= dot * ny;
    t1z -= dot * nz;
    len = std::sqrt(t1x * t1x + t1y * t1y + t1z * t1z);
    if (len < 1e-12) {
        // 退化：尝试用 world X/Y 构建切平面
        if (std::fabs(nz) < 0.9) {
            t1x = -ny;
            t1y = nx;
            t1z = 0.0;
        } else {
            t1x = 1.0;
            t1y = 0.0;
            t1z = 0.0;
            dot = t1x * nx;
            t1x -= dot * nx;
            t1y -= dot * ny;
            t1z -= dot * nz;
        }
        len = std::sqrt(t1x * t1x + t1y * t1y + t1z * t1z);
        if (len < 1e-12) return f;
    }
    f.t1[0] = t1x / len;
    f.t1[1] = t1y / len;
    f.t1[2] = t1z / len;

    // t2 = n × t1
    f.t2[0] = ny * f.t1[2] - nz * f.t1[1];
    f.t2[1] = nz * f.t1[0] - nx * f.t1[2];
    f.t2[2] = nx * f.t1[1] - ny * f.t1[0];

    return f;
}

// 3D dot product: double unit vector · int vector
inline double dot3(const double u[3], int x, int y, int z) {
    return u[0] * static_cast<double>(x) + u[1] * static_cast<double>(y)
           + u[2] * static_cast<double>(z);
}

// Round double to nearest int (ties toward zero for determinism)
inline int roundInt(double v) {
    return static_cast<int>(v > 0.0 ? v + 0.5 : v - 0.5);
}

CurvatureEncodeResult curvatureEncode(
    int rc_x, int rc_y, int rc_z,
    const std::vector<MCGAL::Vertex*>& ringVerts,
    const std::vector<MCGAL::PointInt*>& r1Neighbors) {

    CurvatureEncodeResult result{};
    result.lambdaIdx[0] = 1;  // default 0.5
    result.lambdaIdx[1] = 1;
    result.lambdaIdx[2] = 1;

    const int N = static_cast<int>(ringVerts.size());
    if (N < 3) {
        // 无 ring → 退回一阶 (x,y,z) 直通
        result.alpha = rc_x;
        result.beta  = rc_y;
        result.gamma = rc_z;
        return result;
    }

    LocalFrame frame = computeLocalFrame(ringVerts);

    // 计算 r_hat = mean(r1_neighbors)（整数向量，在量化坐标空间）
    double rhat_x = 0.0, rhat_y = 0.0, rhat_z = 0.0;
    {
        int cnt = 0;
        for (int i = 0; i < N && i < static_cast<int>(r1Neighbors.size()); ++i) {
            if (!r1Neighbors[i]) continue;
            rhat_x += static_cast<double>(r1Neighbors[i]->x());
            rhat_y += static_cast<double>(r1Neighbors[i]->y());
            rhat_z += static_cast<double>(r1Neighbors[i]->z());
            ++cnt;
        }
        if (cnt > 0) {
            rhat_x /= cnt;
            rhat_y /= cnt;
            rhat_z /= cnt;
        }
    }

    // r_c 在三个轴上的投影（float）
    double n_dot_rc  = dot3(frame.n,  rc_x, rc_y, rc_z);
    double t1_dot_rc = dot3(frame.t1, rc_x, rc_y, rc_z);
    double t2_dot_rc = dot3(frame.t2, rc_x, rc_y, rc_z);

    // r_hat 在三个轴上的投影（float）
    int rhat_xi = roundInt(rhat_x);
    int rhat_yi = roundInt(rhat_y);
    int rhat_zi = roundInt(rhat_z);
    double n_dot_rhat  = dot3(frame.n,  rhat_xi, rhat_yi, rhat_zi);
    double t1_dot_rhat = dot3(frame.t1, rhat_xi, rhat_yi, rhat_zi);
    double t2_dot_rhat = dot3(frame.t2, rhat_xi, rhat_yi, rhat_zi);

    // 每个轴独立选最优 leaky factor（最小化 |α'|）
    const double lambdaVals[3] = {0.0, 0.5, 1.0};

    auto bestLambda = [&](double proj_rc, double proj_rhat, int& outAlpha) -> int {
        int bestI = 1;  // default 0.5
        int bestAbs = 2147483647;
        for (int i = 0; i < 3; ++i) {
            double adj = proj_rc - lambdaVals[i] * proj_rhat;
            int a = roundInt(adj);
            int absA = a < 0 ? -a : a;
            if (absA < bestAbs) {
                bestAbs = absA;
                bestI = i;
                outAlpha = a;
            }
        }
        return bestI;
    };

    result.lambdaIdx[0] = bestLambda(n_dot_rc,  n_dot_rhat,  result.alpha);
    result.lambdaIdx[1] = bestLambda(t1_dot_rc, t1_dot_rhat, result.beta);
    result.lambdaIdx[2] = bestLambda(t2_dot_rc, t2_dot_rhat, result.gamma);

    return result;
}

std::array<int, 3> curvatureEncodeFixed(
    int rc_x, int rc_y, int rc_z,
    const std::vector<MCGAL::Vertex*>& ringVerts,
    const std::vector<MCGAL::PointInt*>& r1Neighbors,
    int lambdaIdx0, int lambdaIdx1, int lambdaIdx2) {

    const int N = static_cast<int>(ringVerts.size());
    if (N < 3) {
        return {rc_x, rc_y, rc_z};
    }

    LocalFrame frame = computeLocalFrame(ringVerts);

    // r_hat = mean(r1_neighbors)
    double rhat_x = 0.0, rhat_y = 0.0, rhat_z = 0.0;
    {
        int cnt = 0;
        for (int i = 0; i < N && i < static_cast<int>(r1Neighbors.size()); ++i) {
            if (!r1Neighbors[i]) continue;
            rhat_x += static_cast<double>(r1Neighbors[i]->x());
            rhat_y += static_cast<double>(r1Neighbors[i]->y());
            rhat_z += static_cast<double>(r1Neighbors[i]->z());
            ++cnt;
        }
        if (cnt > 0) { rhat_x /= cnt; rhat_y /= cnt; rhat_z /= cnt; }
    }
    int rhat_xi = roundInt(rhat_x);
    int rhat_yi = roundInt(rhat_y);
    int rhat_zi = roundInt(rhat_z);

    double n_dot_rc  = dot3(frame.n,  rc_x, rc_y, rc_z);
    double t1_dot_rc = dot3(frame.t1, rc_x, rc_y, rc_z);
    double t2_dot_rc = dot3(frame.t2, rc_x, rc_y, rc_z);

    double n_dot_rhat  = dot3(frame.n,  rhat_xi, rhat_yi, rhat_zi);
    double t1_dot_rhat = dot3(frame.t1, rhat_xi, rhat_yi, rhat_zi);
    double t2_dot_rhat = dot3(frame.t2, rhat_xi, rhat_yi, rhat_zi);

    const double la = lambdaValue(lambdaIdx0);
    const double lb = lambdaValue(lambdaIdx1);
    const double lg = lambdaValue(lambdaIdx2);

    int alpha = roundInt(n_dot_rc  - la * n_dot_rhat);
    int beta  = roundInt(t1_dot_rc - lb * t1_dot_rhat);
    int gamma = roundInt(t2_dot_rc - lg * t2_dot_rhat);

    return {alpha, beta, gamma};
}

std::array<int, 3> curvatureDecode(
    int alpha, int beta, int gamma,
    int lambdaIdx0, int lambdaIdx1, int lambdaIdx2,
    const std::vector<MCGAL::Vertex*>& ringVerts,
    const std::vector<MCGAL::PointInt*>& r1Neighbors) {

    const int N = static_cast<int>(ringVerts.size());
    if (N < 3) {
        return {alpha, beta, gamma};  // 一阶直通
    }

    LocalFrame frame = computeLocalFrame(ringVerts);

    // r_hat = mean(r1_neighbors)（与 encoder 相同）
    double rhat_x = 0.0, rhat_y = 0.0, rhat_z = 0.0;
    {
        int cnt = 0;
        for (int i = 0; i < N && i < static_cast<int>(r1Neighbors.size()); ++i) {
            if (!r1Neighbors[i]) continue;
            rhat_x += static_cast<double>(r1Neighbors[i]->x());
            rhat_y += static_cast<double>(r1Neighbors[i]->y());
            rhat_z += static_cast<double>(r1Neighbors[i]->z());
            ++cnt;
        }
        if (cnt > 0) {
            rhat_x /= cnt;
            rhat_y /= cnt;
            rhat_z /= cnt;
        }
    }
    int rhat_xi = roundInt(rhat_x);
    int rhat_yi = roundInt(rhat_y);
    int rhat_zi = roundInt(rhat_z);

    double n_dot_rhat  = dot3(frame.n,  rhat_xi, rhat_yi, rhat_zi);
    double t1_dot_rhat = dot3(frame.t1, rhat_xi, rhat_yi, rhat_zi);
    double t2_dot_rhat = dot3(frame.t2, rhat_xi, rhat_yi, rhat_zi);

    const double la = lambdaValue(lambdaIdx0);
    const double lb = lambdaValue(lambdaIdx1);
    const double lg = lambdaValue(lambdaIdx2);

    // 重建 r_c 在局部坐标系下的三个分量（float）
    double n_rc  = static_cast<double>(alpha) + la * n_dot_rhat;
    double t1_rc = static_cast<double>(beta)  + lb * t1_dot_rhat;
    double t2_rc = static_cast<double>(gamma) + lg * t2_dot_rhat;

    // 重建 r_c 在全局坐标下的分量
    double rx = n_rc * frame.n[0]  + t1_rc * frame.t1[0]  + t2_rc * frame.t2[0];
    double ry = n_rc * frame.n[1]  + t1_rc * frame.t1[1]  + t2_rc * frame.t2[1];
    double rz = n_rc * frame.n[2]  + t1_rc * frame.t1[2]  + t2_rc * frame.t2[2];

    return {roundInt(rx), roundInt(ry), roundInt(rz)};
}

}  // namespace pmsf
