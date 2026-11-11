// SPDX-License-Identifier: Apache-2.0
//
// KLTTransform.cpp —— PR-X (2026-05-13)
// 3×3 KLT（PCA）简化实现：Jacobi 特征分解 + 量化基存储。

#include "predict/KLTTransform.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace pmsf {

// Round to nearest int
static inline int roundInt(double v) {
    return static_cast<int>(v > 0.0 ? v + 0.5 : v - 0.5);
}

KLTBasis computeKLTBasis(const std::vector<int>& alpha,
                          const std::vector<int>& beta,
                          const std::vector<int>& gamma) {
    KLTBasis b{};
    b.valid = false;

    const int N = static_cast<int>(alpha.size());
    if (N < 4) return b;  // 样本太少，跳过 KLT

    // 1. 计算均值
    double ma = 0, mb = 0, mg = 0;
    for (int i = 0; i < N; ++i) {
        ma += alpha[i];
        mb += beta[i];
        mg += gamma[i];
    }
    ma /= N; mb /= N; mg /= N;

    // 2. 计算 3×3 协方差矩阵（对称）
    double c00 = 0, c01 = 0, c02 = 0, c11 = 0, c12 = 0, c22 = 0;
    for (int i = 0; i < N; ++i) {
        double da = alpha[i] - ma;
        double db = beta[i]  - mb;
        double dg = gamma[i] - mg;
        c00 += da * da; c01 += da * db; c02 += da * dg;
        c11 += db * db; c12 += db * dg;
        c22 += dg * dg;
    }
    double invN = 1.0 / (N - 1);
    c00 *= invN; c01 *= invN; c02 *= invN;
    c11 *= invN; c12 *= invN;
    c22 *= invN;

    // 3. 检查能量集中度（对角占优则跳过 KLT）
    double diagSum = c00 + c11 + c22;
    double offDiagSum = 2.0 * (std::fabs(c01) + std::fabs(c02) + std::fabs(c12));
    if (diagSum < 1e-12 || offDiagSum / diagSum < 0.15) {
        // 非对角能量 < 15% 对角能量 → KLT 无增益
        return b;
    }

    // 4. Jacobi 特征分解（3×3 对称矩阵，固定 10 次 sweep）
    double V[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    double A[3][3] = {{c00,c01,c02},{c01,c11,c12},{c02,c12,c22}};

    for (int sweep = 0; sweep < 10; ++sweep) {
        for (int p = 0; p < 2; ++p) {
            for (int q = p + 1; q < 3; ++q) {
                double apq = A[p][q];
                if (std::fabs(apq) < 1e-15) continue;
                double app = A[p][p], aqq = A[q][q];
                double theta = 0.5 * std::atan2(2.0 * apq, aqq - app);
                double c = std::cos(theta), s = std::sin(theta);

                // 更新 A
                A[p][p] = c*c*app + s*s*aqq - 2*c*s*apq;
                A[q][q] = s*s*app + c*c*aqq + 2*c*s*apq;
                A[p][q] = A[q][p] = 0;

                // 更新 A 的其他元素
                for (int r = 0; r < 3; ++r) {
                    if (r == p || r == q) continue;
                    double arp = A[r][p], arq = A[r][q];
                    A[r][p] = A[p][r] = c*arp - s*arq;
                    A[r][q] = A[q][r] = s*arp + c*arq;
                }

                // 更新 V
                for (int r = 0; r < 3; ++r) {
                    double vrp = V[r][p], vrq = V[r][q];
                    V[r][p] = c*vrp - s*vrq;
                    V[r][q] = s*vrp + c*vrq;
                }
            }
        }
    }

    // 5. 按特征值大小排序（降序）
    double ev[3] = {A[0][0], A[1][1], A[2][2]};
    int idx[3] = {0, 1, 2};
    if (ev[idx[0]] < ev[idx[1]]) std::swap(idx[0], idx[1]);
    if (ev[idx[0]] < ev[idx[2]]) std::swap(idx[0], idx[2]);
    if (ev[idx[1]] < ev[idx[2]]) std::swap(idx[1], idx[2]);

    // 检查能量集中比
    double eSum = ev[idx[0]] + ev[idx[1]] + ev[idx[2]];
    if (eSum < 1e-12 || ev[idx[0]] / eSum < 0.45) {
        // 第一分量能量 < 45% → KLT 效果不大
        return b;
    }

    // 输出排序后的特征向量
    for (int i = 0; i < 3; ++i) {
        int col = idx[i];
        b.vecs[i][0] = V[0][col];
        b.vecs[i][1] = V[1][col];
        b.vecs[i][2] = V[2][col];
        // 归一化
        double len = std::sqrt(b.vecs[i][0]*b.vecs[i][0]
                              + b.vecs[i][1]*b.vecs[i][1]
                              + b.vecs[i][2]*b.vecs[i][2]);
        if (len > 1e-12) {
            b.vecs[i][0] /= len;
            b.vecs[i][1] /= len;
            b.vecs[i][2] /= len;
        }
    }
    b.valid = true;
    return b;
}

void applyKLTForward(std::vector<int>& alpha, std::vector<int>& beta,
                     std::vector<int>& gamma, const KLTBasis& basis) {
    if (!basis.valid) return;
    const int N = static_cast<int>(alpha.size());
    for (int i = 0; i < N; ++i) {
        double a = alpha[i], b = beta[i], g = gamma[i];
        int na = roundInt(basis.vecs[0][0]*a + basis.vecs[0][1]*b + basis.vecs[0][2]*g);
        int nb = roundInt(basis.vecs[1][0]*a + basis.vecs[1][1]*b + basis.vecs[1][2]*g);
        int ng = roundInt(basis.vecs[2][0]*a + basis.vecs[2][1]*b + basis.vecs[2][2]*g);
        alpha[i] = na; beta[i] = nb; gamma[i] = ng;
    }
}

void applyKLTInverse(std::vector<int>& alpha, std::vector<int>& beta,
                     std::vector<int>& gamma, const KLTBasis& basis) {
    if (!basis.valid) return;
    const int N = static_cast<int>(alpha.size());
    for (int i = 0; i < N; ++i) {
        double a = alpha[i], b = beta[i], g = gamma[i];
        // 逆变换 = 转置 × r（特征向量矩阵是正交的）
        int na = roundInt(basis.vecs[0][0]*a + basis.vecs[1][0]*b + basis.vecs[2][0]*g);
        int nb = roundInt(basis.vecs[0][1]*a + basis.vecs[1][1]*b + basis.vecs[2][1]*g);
        int ng = roundInt(basis.vecs[0][2]*a + basis.vecs[1][2]*b + basis.vecs[2][2]*g);
        alpha[i] = na; beta[i] = nb; gamma[i] = ng;
    }
}

int packKLTBasis(const KLTBasis& basis, unsigned char* out, int maxLen) {
    if (!basis.valid || maxLen < 10) return 0;
    out[0] = 1;  // flag: KLT enabled
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            // 量化到 [-127, 127]
            int v = roundInt(basis.vecs[i][j] * 127.0);
            if (v < -127) v = -127;
            if (v > 127) v = 127;
            out[1 + i*3 + j] = static_cast<unsigned char>(v & 0xFF);
        }
    }
    return 10;
}

int unpackKLTBasis(const unsigned char* in, int inLen, KLTBasis& basis) {
    basis.valid = false;
    if (inLen < 1 || in[0] != 1) return 0;  // KLT not enabled
    if (inLen < 10) return 0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            int v = static_cast<int>(static_cast<signed char>(in[1 + i*3 + j]));
            basis.vecs[i][j] = static_cast<double>(v) / 127.0;
        }
    }
    // 施密特正交化（量化可能破坏正交性）
    for (int i = 0; i < 3; ++i) {
        for (int k = 0; k < i; ++k) {
            double dot = basis.vecs[i][0]*basis.vecs[k][0]
                        + basis.vecs[i][1]*basis.vecs[k][1]
                        + basis.vecs[i][2]*basis.vecs[k][2];
            basis.vecs[i][0] -= dot * basis.vecs[k][0];
            basis.vecs[i][1] -= dot * basis.vecs[k][1];
            basis.vecs[i][2] -= dot * basis.vecs[k][2];
        }
        double len = std::sqrt(basis.vecs[i][0]*basis.vecs[i][0]
                              + basis.vecs[i][1]*basis.vecs[i][1]
                              + basis.vecs[i][2]*basis.vecs[i][2]);
        if (len > 1e-12) {
            basis.vecs[i][0] /= len;
            basis.vecs[i][1] /= len;
            basis.vecs[i][2] /= len;
        }
    }
    basis.valid = true;
    return 10;
}

}  // namespace pmsf
