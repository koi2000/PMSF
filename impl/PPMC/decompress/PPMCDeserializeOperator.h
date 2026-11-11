#ifndef PPMC_DESERIALIZE_OPERATOR_H
#define PPMC_DESERIALIZE_OPERATOR_H

#include "decompress/DeserializeOperator.h"

class PPMCDeserializeOperator : public DeserializeOperator {
  public:
    PPMCDeserializeOperator(std::shared_ptr<SymbolReadOperator> symbolReadOperator, std::shared_ptr<ReconstructOperator> reconstructOperator)
        : DeserializeOperator(symbolReadOperator, reconstructOperator) {}
    ~PPMCDeserializeOperator() = default;

    void setSeeds(std::vector<MCGAL::Halfedge*> seeds) override {
        seed = seeds[0];
        if (seed) {
            seedV0 = seed->vertex();
            seedV1 = seed->end_vertex();
        }
    }

    std::shared_ptr<MCGAL::Mesh> deserializeBaseMesh() override;

    void deserializeOneRound() override;

    MCGAL::Halfedge* seed = nullptr;
    // P3 对齐：与原版 PPMC `vh_departureConquest[0/1]` 等价。
    // 顶点对象在 decimate/undecimate 中保持不变（seed 两端的顶点被保护），
    // 但 halfedge 对象会在 reconstruct 阶段重建，因此每轮解码开始前需要
    // 用 (seedV0, seedV1) 对重新查找 v0→v1 的 halfedge，不能复用旧指针。
    MCGAL::Vertex* seedV0 = nullptr;
    MCGAL::Vertex* seedV1 = nullptr;
};

#endif