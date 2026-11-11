/*
 * @Author: koi
 * @Date: 2025-08-29 21:11:03
 * @Description:
 */

#ifndef BASIC_SYMBOLCOLLECTOPERATOR_H
#define BASIC_SYMBOLCOLLECTOPERATOR_H

#include "SymbolCollectOperator.h"
#include <memory>
#include <vector>

class BasicSymbolCollectOperator : public SymbolCollectOperator {
  public:
    BasicSymbolCollectOperator() = default;

    void collect(MCGAL::Halfedge* seed) override;
    void collect(std::vector<MCGAL::Halfedge*> seed) override;
    int exportToBuffer(char* buffer, bool enableQuantization = false, bool enableCompress = false) override;

    // Parquet V2 结构化存储需要直接访问每轮符号与点流
    const std::vector<std::deque<MCGAL::Point>>& getPointQueues() const { return pointQueues; }
    const std::vector<std::deque<MCGAL::Point>>& getVertexPointQueues() const { return vertexPointQueues; }
    const std::vector<std::deque<MCGAL::PointInt>>& getIpointQueues() const { return ipointQueues; }
    const std::vector<std::deque<char>>& getFacetSymbolQueues() const { return facetSymbolQueues; }
    const std::vector<std::deque<char>>& getVertexSymbolQueues() const { return vertexSymbolQueues; }
    const std::vector<std::deque<char>>& getEdgeSymbolQueues() const { return edgeSymbolQueues; }

    /// @brief Phase A.1: RoundSymbols mirror. Populated alongside the legacy
    ///        6-parallel-deque state at the end of each collect() call. This
    ///        lets Parquet V3 and hook-based consumers iterate a typed
    ///        `pmsf::RoundSymbols` instead of 6 raw std::deques keyed by
    ///        position. The legacy deque getters continue to produce the same
    ///        bytes — this is an additive, read-only mirror.
    const std::vector<pmsf::RoundSymbols>* rounds() const override { return &rounds_; }

  private:
    void collectHalfedgeSymbol(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Halfedge* seed);
    void collectFacetSymbol(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Halfedge* seed);

    std::vector<std::deque<MCGAL::Point>> pointQueues;
    std::vector<std::deque<MCGAL::Point>> vertexPointQueues;
    std::vector<std::deque<MCGAL::PointInt>> ipointQueues;
    std::vector<std::deque<char>> facetSymbolQueues;
    std::vector<std::deque<char>> vertexSymbolQueues;
    std::vector<std::deque<char>> edgeSymbolQueues;

    /// @brief Phase A.1: additive RoundSymbols mirror. Filled at the end of
    ///        each collect() invocation by copying from the 6-deque state so
    ///        the bytes-out path (exportToBuffer) is untouched and the
    ///        byte-identical regression gates still hold.
    std::vector<pmsf::RoundSymbols> rounds_;
    
};

#endif