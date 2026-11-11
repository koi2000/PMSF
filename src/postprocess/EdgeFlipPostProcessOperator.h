#pragma once

#include "PostProcessOperator.h"
#include <array>
#include <set>
#include <string>
#include <utility>
#include <vector>

class EdgeFlipPostProcessOperator final : public PostProcessOperator {
  public:
    struct CoordSymbol {
        std::array<int, 3> a{};
        std::array<int, 3> b{};
        std::array<int, 3> oa{};
        std::array<int, 3> ob{};
        int f0Proxy = -1;
        int f1Proxy = -1;
        int vaProxy = -1;
        int vbProxy = -1;
        int vcProxy = -1;
        int vdProxy = -1;
    };

    struct Config {
        int rounds = 1;
        double longEdgeRatio = 1.35;
        double minImprovementRatio = 0.90;
        int maxFlipsPerRoundPass = -1;
        bool exportSnapshots = false;
        std::string snapshotDir;
    };

    EdgeFlipPostProcessOperator() = default;
    explicit EdgeFlipPostProcessOperator(const Config& cfg) : cfg_(cfg) {}

    void setSeedHalfedge(MCGAL::Halfedge* h) { seedHalfedge_ = h; }

    void setProtectedSeedVertexIds(int seedVid0, int seedVid1) {
        seedVid0_ = seedVid0;
        seedVid1_ = seedVid1;
    }

    void process(std::shared_ptr<MCGAL::Mesh> mesh) override;
    void processWithGraph(std::shared_ptr<MCGAL::Mesh> mesh, std::shared_ptr<Graph> graph) override;

    const std::vector<std::pair<int, int>>& flipSymbols() const {
        return flipSymbols_;
    }
    const std::set<std::pair<int, int>>& flipMarkedEdges() const {
        return flipMarkedEdges_;
    }
    const std::vector<CoordSymbol>& flipCoordSymbols() const {
        return flipCoordSymbols_;
    }
    const std::vector<int>& roundFlipCounts() const {
        return roundFlipCounts_;
    }
    const std::vector<int>& roundCandidateBitCounts() const {
        return roundCandidateBitCounts_;
    }
    const std::vector<unsigned char>& roundCandidateBits() const {
        return roundCandidateBits_;
    }
    const std::vector<int>& protectedVertexIds() const {
        return protectedVertexIds_;
    }

  private:
    Config cfg_;
    MCGAL::Halfedge* seedHalfedge_ = nullptr;
    int seedVid0_ = -1;
    int seedVid1_ = -1;
    std::vector<std::pair<int, int>> flipSymbols_;
    std::vector<CoordSymbol> flipCoordSymbols_;
    std::set<std::pair<int, int>> flipMarkedEdges_;
    std::vector<int> roundFlipCounts_;
    std::vector<int> roundCandidateBitCounts_;
    std::vector<unsigned char> roundCandidateBits_;
    std::vector<int> protectedVertexIds_;
};
