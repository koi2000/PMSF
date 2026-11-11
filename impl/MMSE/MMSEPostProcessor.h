#pragma once
#include "postprocess/PostProcessOperator.h"
#include "Facet.h"
#include "Point.h"
#include <vector>

/// MMSE post-processor: residual-encode the "removed vertex" position of
/// each splittable facet using a dual-ring prediction (framework's
/// pmsf::DualRingPredictor) + α blending between primary and secondary
/// ring terms.
///
/// The prediction math now lives in src/predict/DualRingPredictor.h as a
/// framework hook; this class owns only the MMSE-specific residual offset
/// and facet iteration policy.
class MMSEPostProcessor : public PostProcessOperator {
  public:
    MMSEPostProcessor(double alpha = 0.5) : alpha_(alpha) {}
    ~MMSEPostProcessor() = default;

    void process(std::shared_ptr<MCGAL::Mesh> mesh) override;

    void processWithGraph(std::shared_ptr<MCGAL::Mesh> mesh, std::shared_ptr<Graph> graph) override;

    void inverseProcess(std::shared_ptr<MCGAL::Mesh> mesh, std::vector<MCGAL::Facet*>& facets);

    double alpha() const { return alpha_; }

  private:
    double alpha_;
};
