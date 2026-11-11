#pragma once

#include "decompress/DeserializeOperator.h"
#include "VQMMCPostProcessor.h"
#include "VQMMCSymbolReadOperator.h"
#include "reconstruct/ReconstructOpertator.h"
#include "Mesh.h"
#include <memory>

class VQMMCDeserializeOperator : public DeserializeOperator {
  public:
    VQMMCDeserializeOperator(std::shared_ptr<VQMMCSymbolReadOperator> symbolReadOperator,
                              std::shared_ptr<ReconstructOperator> reconstructOperator,
                              std::shared_ptr<VQMMCPostProcessor> postProcessor)
        : DeserializeOperator(std::static_pointer_cast<SymbolReadOperator>(symbolReadOperator), reconstructOperator),
          vqSymbolReader_(symbolReadOperator),
          postProcessor_(postProcessor) {}

    ~VQMMCDeserializeOperator() = default;

    void setSeeds(std::vector<MCGAL::Halfedge*> seeds) override {
        seed_ = seeds[0];
    }

    std::shared_ptr<MCGAL::Mesh> deserializeBaseMesh() override;

    void deserializeOneRound() override;

    std::shared_ptr<MCGAL::Mesh> exportMesh() override { return mesh_; }

    void setStrictMode(bool v) { strictMode_ = v; }

  private:
    MCGAL::Halfedge* seed_ = nullptr;
    bool strictMode_ = false;
    std::shared_ptr<VQMMCSymbolReadOperator> vqSymbolReader_;
    std::shared_ptr<VQMMCPostProcessor> postProcessor_;

    std::vector<MCGAL::Vertex*> vertices_;
    std::vector<MCGAL::Halfedge*> halfedges_;
    std::vector<MCGAL::Facet*> facets_;
};
