/*
 * @Author: koi
 * @Date: 2025-09-02 10:39:10
 * @Description:
 */
#ifndef BASIC_DESERIALIZE_OPERATOR_H
#define BASIC_DESERIALIZE_OPERATOR_H

#include "DeserializeOperator.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "decompress/SymbolReadOperator.h"
#include "postprocess/EdgeFlipPostProcessOperator.h"
#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

class BasicDeserializeOperator : public DeserializeOperator {
  public:
    BasicDeserializeOperator(std::shared_ptr<SymbolReadOperator> symbolReadOperator, std::shared_ptr<ReconstructOperator> reconstructOperator)
        : DeserializeOperator(symbolReadOperator, reconstructOperator) {}

    ~BasicDeserializeOperator() = default;

    // void init(std::string path) override;

    void setSeeds(std::vector<MCGAL::Halfedge*> seeds) override {
        seed = seeds[0];
    }

    std::shared_ptr<MCGAL::Mesh> deserializeBaseMesh() override;

    std::shared_ptr<MCGAL::Mesh> exportMesh() override;

    void deserializeOneRound() override;

    MCGAL::Halfedge* seed = nullptr;
    bool hasInterleavedEdgeFlipPayload_ = false;
    int edgeFlipDecodedRounds_ = 0;
    std::vector<int> edgeFlipRoundCounts_;
    std::vector<int> edgeFlipPrefix_;
    std::vector<std::pair<int, int>> edgeFlipSymbols_;
    std::vector<EdgeFlipPostProcessOperator::CoordSymbol> edgeFlipCoordSymbols_;
    std::vector<int> edgeFlipRoundBitCounts_;
    std::vector<int> edgeFlipRoundBitPrefix_;
    std::vector<unsigned char> edgeFlipRoundBits_;
};

#endif
