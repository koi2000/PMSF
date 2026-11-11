// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace pmsf::mmse_pred {

inline constexpr double kConquestRho = 0.997;

struct FineRingModel {
    std::vector<std::size_t> support; // rooted fine-graph indices, in ascending order
    std::vector<std::uint8_t> centerDistances;
    std::vector<std::uint8_t> pairDistances; // row-major square matrix
};

// Compute Eq. (7)/(8) distances on the restored fine graph. Vertices whose
// positions are not available may shorten paths, but never supply coordinates.
// All pairs can reach each other in <=4 steps via the center. Search for
// shorter paths without an arbitrary distance cap or a global n*n graph table.
class FineRingWorkspace {
  public:
    FineRingModel model(const std::vector<std::vector<std::size_t>>& graph,
                        const std::vector<bool>& known, std::size_t center) {
        if (known.size() != graph.size() || center >= graph.size() || known[center]) {
            throw std::runtime_error("MMSE fine ring: invalid position-availability mask or center");
        }
        if (stamp_.size() != graph.size()) {
            stamp_.assign(graph.size(), 0);
            distance_.resize(graph.size());
            generation_ = 0;
        }
        walk(graph, center, 2);
        FineRingModel result;
        for (auto index : queue_) if (known[index]) result.support.push_back(index);
        std::sort(result.support.begin(), result.support.end());
        for (auto index : result.support) result.centerDistances.push_back(distance_[index]);
        const auto n = result.support.size();
        if (n && n > result.pairDistances.max_size() / n) {
            throw std::runtime_error("MMSE fine ring: pair distance table too large");
        }
        result.pairDistances.resize(n * n);
        for (std::size_t row = 0; row < n; ++row) {
            walk(graph, result.support[row], 3);
            for (std::size_t col = row + 1; col < n; ++col) {
                const auto target = result.support[col];
                auto d = static_cast<std::uint8_t>(result.centerDistances[row] + result.centerDistances[col]);
                if (stamp_[target] == generation_) d = std::min(d, distance_[target]);
                result.pairDistances[row*n+col] = result.pairDistances[col*n+row] = d;
            }
        }
        return result;
    }

  private:
    void walk(const std::vector<std::vector<std::size_t>>& graph, std::size_t start, std::uint8_t limit) {
        if (++generation_ == 0) {
            std::fill(stamp_.begin(), stamp_.end(), 0);
            ++generation_;
        }
        queue_.clear();
        queue_.push_back(start);
        stamp_[start] = generation_;
        distance_[start] = 0;
        for (std::size_t head = 0; head < queue_.size(); ++head) {
            const auto v = queue_[head];
            if (distance_[v] >= limit) break;
            for (auto neighbor : graph[v]) {
                if (neighbor >= graph.size()) throw std::runtime_error("MMSE fine ring: invalid graph edge");
                if (stamp_[neighbor] == generation_) continue;
                stamp_[neighbor] = generation_;
                distance_[neighbor] = static_cast<std::uint8_t>(distance_[v]+1);
                queue_.push_back(neighbor);
            }
        }
    }

    std::vector<std::size_t> queue_;
    std::vector<std::uint32_t> stamp_;
    std::vector<std::uint8_t> distance_;
    std::uint32_t generation_ = 0;
};

struct FineRingWeights {
    std::vector<double> coefficients;
    double minAbsPivot = 0.;
};

// Eq. (6): partial pivoting on Rw=C, with the common variance cancelled.
// A graph-distance covariance need not be positive definite. There is no
// ridge, normalization or clipping; a numerically singular solve is reported
// as unavailable so the encoder cannot select mode 3 for that cluster.
inline std::optional<FineRingWeights> solveFineRingWeights(const FineRingModel& model) {
    const std::size_t n = model.support.size();
    if (!n) return std::nullopt;
    if (model.centerDistances.size() != n || model.pairDistances.size()/n != n ||
        model.pairDistances.size()%n != 0) {
        throw std::runtime_error("MMSE fine ring: inconsistent system dimensions");
    }
    double rhoPowers[5] = {1.};
    for (int d = 1; d <= 4; ++d) rhoPowers[d] = rhoPowers[d-1] * kConquestRho;
    std::vector<double> matrix;
    matrix.reserve(model.pairDistances.size());
    for (auto d : model.pairDistances) {
        if (d > 4) throw std::runtime_error("MMSE fine ring: pair outside distance bound");
        matrix.push_back(rhoPowers[d]);
    }
    FineRingWeights solution;
    auto& rhs = solution.coefficients;
    rhs.reserve(n);
    for (auto d : model.centerDistances) {
        if (d < 1 || d > 2) throw std::runtime_error("MMSE fine ring: support outside 1/2 ring");
        rhs.push_back(rhoPowers[d]);
    }
    // Original matrix entries are bounded by 1. Use a roundoff-scale pivot
    // threshold and no modification to the system for well-resolved pivots.
    const double tolerance = 16 * std::numeric_limits<double>::epsilon() * static_cast<double>(n);
    solution.minAbsPivot = std::numeric_limits<double>::infinity();
    for (std::size_t col = 0; col < n; ++col) {
        auto pivot = col;
        for (std::size_t row = col+1; row < n; ++row) {
            if (std::abs(matrix[row*n+col]) > std::abs(matrix[pivot*n+col])) pivot = row;
        }
        const double best = std::abs(matrix[pivot*n+col]);
        if (!std::isfinite(best) || best <= tolerance) return std::nullopt;
        solution.minAbsPivot = std::min(solution.minAbsPivot, best);
        if (pivot != col) {
            for (std::size_t k = col; k < n; ++k) std::swap(matrix[col*n+k],matrix[pivot*n+k]);
            std::swap(rhs[col],rhs[pivot]);
        }
        for (std::size_t row = col+1; row < n; ++row) {
            const double factor = matrix[row*n+col]/matrix[col*n+col];
            if (factor == 0.) continue;
            for (std::size_t k = col+1; k < n; ++k) matrix[row*n+k] -= factor*matrix[col*n+k];
            rhs[row] -= factor*rhs[col];
        }
    }
    for (std::size_t row = n; row-- > 0;) {
        for (std::size_t k = row+1; k < n; ++k) rhs[row] -= matrix[row*n+k]*rhs[k];
        rhs[row] /= matrix[row*n+row];
        if (!std::isfinite(rhs[row])) return std::nullopt;
    }
    return solution;
}
} // namespace pmsf::mmse_pred
