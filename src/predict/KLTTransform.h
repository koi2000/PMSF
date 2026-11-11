// SPDX-License-Identifier: Apache-2.0
//
// KLTTransform.h —— PR-X (2026-05-13)
// 论文 §3.2 topology-based KLT 的简化实现。
//
// 对 (α', β', γ') 残差做 per-round 3×3 KLT，将能量集中到少数分量。
// 若协方差矩阵非对角项显著（能量集中比 > 1.5），启用 KLT；
// 否则直通原残差（论文"encoder decides whether to use the transform or not"）。
//
// 双端共用：encoder 计算 basis 并量化存储，decoder 读回并做逆变换。

#ifndef PMSF_PREDICT_KLT_TRANSFORM_H
#define PMSF_PREDICT_KLT_TRANSFORM_H

#include <vector>

namespace pmsf {

struct KLTBasis {
    double vecs[3][3];  // 行优先：vecs[i] = 第 i 个特征向量（已归一化）
    bool valid;
};

/// 从残差向量计算 KLT 基（3×3 协方差矩阵的特征向量）。
/// 若样本数 < 4 或能量比 ≤ 1.5，返回 valid=false（直通）。
KLTBasis computeKLTBasis(const std::vector<int>& alpha,
                          const std::vector<int>& beta,
                          const std::vector<int>& gamma);

/// 前向 KLT：r_klt = basis^T × r_curv（in-place）
void applyKLTForward(std::vector<int>& alpha, std::vector<int>& beta,
                     std::vector<int>& gamma, const KLTBasis& basis);

/// 逆向 KLT：r_curv = basis × r_klt（in-place）
void applyKLTInverse(std::vector<int>& alpha, std::vector<int>& beta,
                     std::vector<int>& gamma, const KLTBasis& basis);

/// 打包 basis 为紧凑表示（9 分量各 1 字节有符号量化 + 1 字节 flag = ≤10 字节）。
/// 返回实际写入字节数。
int packKLTBasis(const KLTBasis& basis, unsigned char* out, int maxLen);

/// 解包紧凑表示 → KLTBasis。返回读取字节数，0 表示无效/未启用。
int unpackKLTBasis(const unsigned char* in, int inLen, KLTBasis& basis);

}  // namespace pmsf

#endif  // PMSF_PREDICT_KLT_TRANSFORM_H
