#pragma once

#include "postprocess/PostProcessOperator.h"
#include "Facet.h"
#include "Point.h"
#include "Types.h"
#include <cstdint>
#include <unordered_map>
#include <vector>
#include <memory>

MCGAL::Point vqmmcBarycentricPrediction(MCGAL::Facet* facet);

class VQMMCPostProcessor : public PostProcessOperator {
  public:
    struct FacetTrace {
        MCGAL::Facet* facet = nullptr;
        int removedVertexId = -1;
        std::vector<int> boundaryVertexIds;
        MCGAL::Point actualPosition;
        MCGAL::Point sourcePrediction;
        uint16_t codebookIndex = 0;
    };

    struct RoundVQData {
        std::vector<MCGAL::Point> codebook;
        std::vector<PTTreeNode> progressiveTree;
        std::unordered_map<MCGAL::Facet*, uint16_t> facetToIndex;
        std::vector<FacetTrace> traces;
    };

    VQMMCPostProcessor(int codebookSize = 256) : codebookSize_(codebookSize) {}
    ~VQMMCPostProcessor() = default;

    void process(std::shared_ptr<MCGAL::Mesh> mesh) override;
    void processWithGraph(std::shared_ptr<MCGAL::Mesh> mesh, std::shared_ptr<Graph> graph) override;
    void inverseProcess(std::shared_ptr<MCGAL::Mesh> mesh, std::vector<MCGAL::Facet*>& facets);
    void finalizeDecoderEquivalentGeometry(
        const std::shared_ptr<MCGAL::Mesh>& mesh);
    void recordRemovedVertex(MCGAL::Facet* facet, int vertexId);

    const RoundVQData& latestRoundData() const { return roundData_.back(); }
    const std::vector<RoundVQData>& allRoundData() const { return roundData_; }
    int codebookSize() const { return codebookSize_; }

  private:
    int codebookSize_;
    std::vector<RoundVQData> roundData_;
    std::unordered_map<MCGAL::Facet*, int> removedVertexIds_;
};
