#pragma once

#include "compress/SymbolCollectOperator.h"
#include "VQMMCPostProcessor.h"
#include <cstdint>
#include <deque>
#include <memory>
#include <vector>

class VQMMCSymbolCollectOperator : public SymbolCollectOperator {
  public:
    VQMMCSymbolCollectOperator(std::shared_ptr<VQMMCPostProcessor> postProcessor)
        : postProcessor_(postProcessor) {}

    void collect(MCGAL::Halfedge* seed) override;
    void collect(std::vector<MCGAL::Halfedge*> seeds) override;
    int exportToBuffer(char* buffer, bool enableQuantization = false, bool enableCompress = false) override;
    void refreshGeometryFromPostProcessor();

    // ---- column-native parquet 序列化用的只读访问 ----
    // 队列按 collect() 调用顺序追加：第 r 个元素 = 第 r 轮（最旧轮在前）。
    // Decode 端约定 reverse iterate（与 exportToBuffer 一致）。
    const std::vector<std::deque<char>>&         facetSymbolQueues()  const { return facetSymbolQueues_; }
    const std::vector<std::deque<char>>&         edgeSymbolQueues()   const { return edgeSymbolQueues_; }
    const std::vector<std::deque<uint16_t>>&     vqIndexQueues()      const { return vqIndexQueues_; }
    const std::vector<std::vector<MCGAL::Facet*>>& splitFacetQueues() const { return splitFacetQueues_; }
    const std::vector<std::vector<MCGAL::Point>>& codebookQueues()    const { return codebookQueues_; }
    const std::vector<std::deque<uint8_t>>&      faceCodeQueues()     const { return faceCodeQueues_; }
    const std::vector<std::deque<uint8_t>>&      otherValQueues()     const { return otherValQueues_; }
    const std::vector<std::deque<uint8_t>>&      localEdgeSymbolQueues() const { return localEdgeSymbolQueues_; }
    const std::vector<std::deque<uint8_t>>&      splitValenceQueues() const { return splitValenceQueues_; }
    const std::vector<std::deque<uint8_t>>&      splitOtherValQueues() const { return splitOtherValQueues_; }
    const std::vector<std::deque<uint8_t>>&      rootDiagSlotQueues() const { return rootDiagSlotQueues_; }
    const std::vector<pmsf::RoundSymbols>* rounds() const override { return &rounds_; }
    const pmsf::PMSFSymbolPayloadTable* paperSymbolPayloadTable() const override;

  private:
    std::shared_ptr<VQMMCPostProcessor> postProcessor_;

    std::vector<std::deque<char>> facetSymbolQueues_;
    std::vector<std::deque<char>> edgeSymbolQueues_;
    std::vector<std::deque<uint16_t>> vqIndexQueues_;
    std::vector<std::vector<MCGAL::Facet*>> splitFacetQueues_;
    std::vector<std::vector<MCGAL::Point>> codebookQueues_;
    std::vector<std::deque<uint8_t>> faceCodeQueues_;
    std::vector<std::deque<uint8_t>> otherValQueues_;
    std::vector<std::deque<uint8_t>> localEdgeSymbolQueues_;
    std::vector<std::deque<uint8_t>> splitValenceQueues_;
    std::vector<std::deque<uint8_t>> splitOtherValQueues_;
    std::vector<std::deque<uint8_t>> rootDiagSlotQueues_;
    std::vector<pmsf::RoundSymbols> rounds_;
    mutable pmsf::PMSFSymbolPayloadTable paperPayloadTableCache_;
    mutable bool paperPayloadTableCacheValid_ = false;
};
