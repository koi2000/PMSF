/*
 * @file KMeansCodebook.h
 * @brief Lloyd's k-means codebook. Deterministic given a seed.
 *
 * Simple textbook implementation — not competitive with SR-VQ on the full
 * VQMMC benchmark, but adequate for MMSE-scale codebooks and provides a
 * clean reference for the framework's Codebook interface.
 *
 * VQMMC's Stochastic-Relaxation variant will live alongside as
 * StochasticRelaxationVQCodebook.h when we migrate impl/VQMMC/VectorQuantizer.cpp.
 */
#ifndef PMSF_CODEBOOK_KMEANS_CODEBOOK_H
#define PMSF_CODEBOOK_KMEANS_CODEBOOK_H

#include "Codebook.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

namespace pmsf {

class KMeansCodebook final : public Codebook {
public:
    KMeansCodebook(std::size_t k, std::uint32_t seed = 0xC0DEBEEF)
        : k_(k), seed_(seed) {}

    void train(const std::vector<MCGAL::Point>& samples) override {
        if (samples.empty() || k_ == 0) { centroids_.clear(); return; }
        // Forgy init — deterministic with our seed.
        std::mt19937 rng(seed_);
        std::vector<std::size_t> idx(samples.size());
        for (std::size_t i = 0; i < samples.size(); ++i) idx[i] = i;
        std::shuffle(idx.begin(), idx.end(), rng);
        centroids_.clear();
        centroids_.reserve(k_);
        for (std::size_t i = 0; i < k_ && i < samples.size(); ++i)
            centroids_.push_back(samples[idx[i]]);
        if (centroids_.size() < k_) k_ = centroids_.size();

        // Lloyd iterations — 32 is plenty for the codebook sizes the papers
        // actually use (64–1024 entries on Stanford meshes).
        constexpr int kMaxIter = 32;
        std::vector<std::uint32_t> assign(samples.size(), 0);
        for (int it = 0; it < kMaxIter; ++it) {
            bool changed = false;
            for (std::size_t s = 0; s < samples.size(); ++s) {
                std::uint32_t best = 0;
                double best_d = std::numeric_limits<double>::infinity();
                for (std::size_t c = 0; c < k_; ++c) {
                    const double d = distSq(samples[s], centroids_[c]);
                    if (d < best_d) { best_d = d; best = static_cast<std::uint32_t>(c); }
                }
                if (assign[s] != best) { assign[s] = best; changed = true; }
            }
            std::vector<double> sx(k_, 0), sy(k_, 0), sz(k_, 0);
            std::vector<std::size_t> cnt(k_, 0);
            for (std::size_t s = 0; s < samples.size(); ++s) {
                const auto c = assign[s];
                sx[c] += samples[s].x(); sy[c] += samples[s].y(); sz[c] += samples[s].z();
                ++cnt[c];
            }
            for (std::size_t c = 0; c < k_; ++c) {
                if (cnt[c] == 0) continue;
                centroids_[c] = MCGAL::Point(
                    static_cast<float>(sx[c] / cnt[c]),
                    static_cast<float>(sy[c] / cnt[c]),
                    static_cast<float>(sz[c] / cnt[c]));
            }
            if (!changed) break;
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

    std::size_t k_;
    std::uint32_t seed_;
    std::vector<MCGAL::Point> centroids_;
};

}  // namespace pmsf

#endif  // PMSF_CODEBOOK_KMEANS_CODEBOOK_H
