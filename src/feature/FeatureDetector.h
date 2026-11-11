/*
 * @file FeatureDetector.h
 * @brief Pluggable per-mesh feature/anchor detector (curvature hysteresis,
 *        cluster boundaries, etc.).
 *
 * Generalizes impl/APTM/anchor_detection.h so that APTM-style curvature
 * hysteresis, POMAR-style cluster boundaries, or future "preserve sharp
 * creases" detectors can all be expressed as the same hook:
 *     FeatureDetector::detect(mesh) → set of "feature" vertex ids,
 *     which downstream IsRemovableOperator / Predictor consult.
 *
 * Phase 10 addition. Legacy APTM/POMAR keep their bespoke detectors until
 * Phase 13 migrates them.
 */
#ifndef PMSF_FEATURE_FEATURE_DETECTOR_H
#define PMSF_FEATURE_FEATURE_DETECTOR_H

#include "Mesh.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_set>

namespace pmsf {

/// Opaque vertex-id container, populated by detect(). Keyed by the mesh's
/// vertex pool id so consumers can look up O(1) whether a vertex is a
/// feature.
using FeatureSet = std::unordered_set<std::uint32_t>;

class FeatureDetector {
public:
    virtual ~FeatureDetector() = default;

    /// Detect features on a fresh mesh (usually the pre-simplification
    /// original). May be called repeatedly — implementations should cache
    /// per-mesh state internally if re-detection is expensive.
    virtual FeatureSet detect(std::shared_ptr<MCGAL::Mesh> mesh) = 0;
};

}  // namespace pmsf

#endif  // PMSF_FEATURE_FEATURE_DETECTOR_H
