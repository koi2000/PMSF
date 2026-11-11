/*
 * @file StochasticRelaxationVQCodebook.h
 * @brief VQMMC 专用：Stochastic Relaxation（随机松弛）向量量化码本。
 *
 * 从 impl/VQMMC/VectorQuantizer.cpp 提炼到框架层。
 *
 * 论文（Chen-Chou 1990 "Stochastic Relaxation"，VQMMC 引用）对经典 LBG 的改进：
 *   1. 初始化：从训练数据中均匀采样 K 个 centroid。
 *   2. 每次迭代：
 *      - 对每个训练向量 x，以"温度 T"决定的 softmax 概率分布在 K 个 centroid
 *        之间分配软归属，而不是"硬"分给最近一个。
 *      - 用软归属的加权均值更新每个 centroid。
 *      - T 按 geometric schedule 递减：T ← T * cool_rate。
 *   3. 低温下软归属 ≈ 硬归属，收敛到 LBG 结果；高温下避免局部最优。
 *
 * 与 KMeansCodebook 相同接口（继承 pmsf::Codebook），impl/ 只需切类名。
 * 训练后 `encode()` 用纯硬归属（argmin），和 KMeans 一致。
 */
#ifndef PMSF_CODEBOOK_STOCHASTIC_RELAXATION_VQ_CODEBOOK_H
#define PMSF_CODEBOOK_STOCHASTIC_RELAXATION_VQ_CODEBOOK_H

#include "Codebook.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

namespace pmsf {

class StochasticRelaxationVQCodebook final : public Codebook {
public:
    struct Config {
        std::size_t k = 256;
        int max_iter = 64;
        double initial_temperature = 1.0;
        double cool_rate = 0.92;
        std::uint32_t seed = 0xC0DEBEEF;

        Config() = default;
    };

    explicit StochasticRelaxationVQCodebook(Config cfg) : cfg_(cfg) {}
    StochasticRelaxationVQCodebook() = default;
    explicit StochasticRelaxationVQCodebook(std::size_t k, std::uint32_t seed = 0xC0DEBEEF) {
        cfg_.k = k;
        cfg_.seed = seed;
    }

    void train(const std::vector<MCGAL::Point>& samples) override {
        if (samples.empty() || cfg_.k == 0) { centroids_.clear(); return; }
        std::mt19937 rng(cfg_.seed);

        // Forgy init.
        std::vector<std::size_t> idx(samples.size());
        for (std::size_t i = 0; i < samples.size(); ++i) idx[i] = i;
        std::shuffle(idx.begin(), idx.end(), rng);
        centroids_.clear();
        centroids_.reserve(cfg_.k);
        for (std::size_t i = 0; i < cfg_.k && i < samples.size(); ++i) {
            centroids_.push_back(samples[idx[i]]);
        }
        const std::size_t K = centroids_.size();

        double T = cfg_.initial_temperature;
        std::vector<double> acc_x(K, 0), acc_y(K, 0), acc_z(K, 0);
        std::vector<double> acc_w(K, 0);
        std::vector<double> weights(K, 0);

        for (int it = 0; it < cfg_.max_iter; ++it) {
            std::fill(acc_x.begin(), acc_x.end(), 0.0);
            std::fill(acc_y.begin(), acc_y.end(), 0.0);
            std::fill(acc_z.begin(), acc_z.end(), 0.0);
            std::fill(acc_w.begin(), acc_w.end(), 0.0);
            double total_distortion = 0.0;

            for (const auto& s : samples) {
                // Compute distances and softmax weights.
                double minD = std::numeric_limits<double>::infinity();
                for (std::size_t c = 0; c < K; ++c) {
                    const double d = distSq(s, centroids_[c]);
                    if (d < minD) minD = d;
                }
                double sumW = 0.0;
                const double invT = 1.0 / std::max(T, 1e-6);
                for (std::size_t c = 0; c < K; ++c) {
                    // Stabilised softmax: weight = exp(-(d - minD) / T).
                    const double d = distSq(s, centroids_[c]);
                    const double w = std::exp(-(d - minD) * invT);
                    weights[c] = w;
                    sumW += w;
                }
                if (sumW <= 0.0) continue;
                for (std::size_t c = 0; c < K; ++c) {
                    const double w = weights[c] / sumW;
                    acc_x[c] += w * s.x();
                    acc_y[c] += w * s.y();
                    acc_z[c] += w * s.z();
                    acc_w[c] += w;
                }
                total_distortion += minD;
            }

            for (std::size_t c = 0; c < K; ++c) {
                if (acc_w[c] > 1e-9) {
                    centroids_[c] = MCGAL::Point(
                        static_cast<float>(acc_x[c] / acc_w[c]),
                        static_cast<float>(acc_y[c] / acc_w[c]),
                        static_cast<float>(acc_z[c] / acc_w[c]));
                }
            }

            T *= cfg_.cool_rate;
            (void)total_distortion;  // surface for future logging
        }
    }

    std::uint16_t encode(const MCGAL::Point& p) const override {
        if (centroids_.empty()) return 0xFFFF;
        std::uint16_t best = 0;
        double best_d = std::numeric_limits<double>::infinity();
        for (std::size_t c = 0; c < centroids_.size(); ++c) {
            const double d = distSq(p, centroids_[c]);
            if (d < best_d) { best_d = d; best = static_cast<std::uint16_t>(c); }
        }
        return best;
    }

    MCGAL::Point reconstruct(std::uint16_t i) const override {
        if (i >= centroids_.size()) return MCGAL::Point(0, 0, 0);
        return centroids_[i];
    }

    const std::vector<MCGAL::Point>& centroids() const override { return centroids_; }
    std::size_t size() const override { return centroids_.size(); }

private:
    static double distSq(const MCGAL::Point& a, const MCGAL::Point& b) {
        const double dx = a.x() - b.x();
        const double dy = a.y() - b.y();
        const double dz = a.z() - b.z();
        return dx * dx + dy * dy + dz * dz;
    }

    Config cfg_;
    std::vector<MCGAL::Point> centroids_;
};

}  // namespace pmsf

#endif  // PMSF_CODEBOOK_STOCHASTIC_RELAXATION_VQ_CODEBOOK_H
