/*
 * @file Codebook.h
 * @brief Pluggable vector-quantization codebook, shared between MMSE / VQMMC
 *        and future VQ-based algorithms.
 *
 * Today MMSE (impl/MMSE/MMSEPostProcessor.cpp — 143 LOC) and VQMMC
 * (impl/VQMMC/VectorQuantizer.cpp — 443 LOC) each re-implement their own
 * training / lookup paths. Factoring the codebook concept into the
 * framework lets a new VQ algorithm compose training + lookup from
 * existing pieces and emit only its novelty (e.g. a new distortion
 * function or a new initialization scheme).
 *
 * The interface is intentionally 3-method-thin:
 *   - `train` consumes the observed vectors (positions / residuals);
 *   - `encode` finds the nearest entry for an input vector;
 *   - `reconstruct` returns the entry's centroid.
 *
 * Parquet-V3 persists the codebook in its 5th optional table (_codebook)
 * via `centroids()`.
 */
#ifndef PMSF_CODEBOOK_CODEBOOK_H
#define PMSF_CODEBOOK_CODEBOOK_H

#include "Point.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace pmsf {

class Codebook {
public:
    virtual ~Codebook() = default;

    /// Train or refresh the codebook from the full set of observed vectors.
    virtual void train(const std::vector<MCGAL::Point>& samples) = 0;

    /// Find the nearest entry index for `p`. Implementations may return
    /// 0xFFFF to signal "miss → fall back to raw encoding".
    virtual std::uint16_t encode(const MCGAL::Point& p) const = 0;

    /// Reconstruct the centroid for index `i`.
    virtual MCGAL::Point reconstruct(std::uint16_t i) const = 0;

    /// Expose centroids for serialization.
    virtual const std::vector<MCGAL::Point>& centroids() const = 0;

    /// Number of entries.
    virtual std::size_t size() const = 0;
};

}  // namespace pmsf

#endif  // PMSF_CODEBOOK_CODEBOOK_H
