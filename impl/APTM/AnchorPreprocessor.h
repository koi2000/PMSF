#pragma once

#include "Vertex.h"
#include "feature/CurvatureHysteresisDetector.h"
#include "preprocess/PreProcessOperator.h"
#include "ContextPool.h"
#include "Global.h"
#include <cstdio>
#include <memory>
#include <vector>

/**
 * APTMPreprocessor / AnchorPreprocessor
 *
 * Task 3 + hook-bridge migration: the Gaussian-curvature + hysteresis
 * detector has moved to `src/feature/CurvatureHysteresisDetector.h`. This
 * preprocessor is now a thin adapter that resolves detected poolIds back
 * to Vertex*, caches them on first invocation, and re-marks them
 * non-removable each round (the framework's per-round resetState() wipes
 * the flag).
 *
 * The legacy `impl/APTM/anchor_detection.h` (213 LOC) is now superseded by
 * the framework hook; AnchorPreprocessor owns only the per-round anchor-
 * reapplication policy, not the detection math. Byte-identical with the
 * legacy implementation because CurvatureHysteresisDetector is a 1:1 port
 * of AnchorDetection::detectAnchors.
 */
class AnchorPreprocessor : public PreProcessOperator {
  public:
    AnchorPreprocessor(double upper = 1.7, double lower = 1.2)
        : detector_(std::make_shared<pmsf::CurvatureHysteresisDetector>(upper, lower)) {}
    ~AnchorPreprocessor() = default;

    void process(std::shared_ptr<MCGAL::Mesh> mesh) override {
        if (!detected_) {
            const pmsf::FeatureSet ids = detector_->detect(mesh);
            anchors_.reserve(ids.size());
            const int meshId = mesh->meshId();
            for (std::uint32_t poolId : ids) {
                MCGAL::Vertex* v =
                    MCGAL::contextPool.getVertexByIndexInSubPool(meshId, static_cast<int>(poolId));
                if (v != nullptr) anchors_.push_back(v);
            }
            detected_ = true;
            std::fprintf(stderr, "[APTM] detected anchors: %zu\n", anchors_.size());
        }
        for (MCGAL::Vertex* v : anchors_) {
            if (v != nullptr && !v->isRemoved()) {
                v->setAnchor();
            }
        }
    }

    size_t anchorCount() const { return anchors_.size(); }

  private:
    std::shared_ptr<pmsf::CurvatureHysteresisDetector> detector_;
    bool detected_ = false;
    std::vector<MCGAL::Vertex*> anchors_;
};
