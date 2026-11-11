/*
 * @Author: koi
 * @Date: 2025-08-29 21:11:03
 * @Description:
 */

#ifndef OPTIMIZED_SYMBOLCOLLECTOPERATOR_H
#define OPTIMIZED_SYMBOLCOLLECTOPERATOR_H

#include "SymbolCollectOperator.h"
#include <memory>
#include <vector>

class OptimizedSymbolCollectOperator : public SymbolCollectOperator {
  public:
    OptimizedSymbolCollectOperator() = default;

    void collect(MCGAL::Halfedge* seed) override;
    void collectFromFaces(MCGAL::Halfedge* seed,
                          const std::vector<MCGAL::Facet*>& faces);
    void collect(std::vector<MCGAL::Halfedge*> seed) override;
    int exportToBuffer(char* buffer, bool enableQuantization = false, bool enableCompress = false) override;
    int exportToBufferWithRangeCoder(char* buffer, bool enableQuantization = false) override;
    bool supportsRangeCoderExport() const override { return true; }
    void setGroupFilter(int groupId) { groupFilter_ = groupId; }

  private:
    void collectHalfedgeSymbol(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Halfedge* seed);
    void collectFacetSymbol(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Halfedge* seed);
    bool inGroup(MCGAL::Halfedge* h) const;
    bool canonicalSegmentedOrder() const {
        if (std::getenv("PMSF_SEG_FIFO_BFS")) return false;
        return groupFilter_ >= 0;
    }
    bool canonicalRemainingFaces() const;
    bool canonicalGateNeighbors() const;

  public:
    // Parquet V2 多表方案需要直接访问内部队列
    const std::vector<std::deque<MCGAL::Point>>& getPointQueues() const { return pointQueues; }
    const std::vector<std::deque<MCGAL::Point>>& getVertexPointQueues() const { return vertexPointQueues; }
    const std::vector<std::deque<MCGAL::PointInt>>& getIpointQueues() const { return ipointQueues; }
    const std::vector<std::deque<char>>& getFacetSymbolQueues() const { return facetSymbolQueues; }
    const std::vector<std::deque<char>>& getVertexSymbolQueues() const { return vertexSymbolQueues; }
    const std::vector<std::deque<char>>& getEdgeSymbolQueues() const { return edgeSymbolQueues; }
    const std::vector<pmsf::RoundSymbols>* rounds() const override { return &rounds_; }

    // codec=4 A3 回退方案：从已解码的原始符号/坐标重新编码出 Class I blob。
    // 这些 setter 把多轮队列折叠成单轮队列；exportToBufferWithRangeCoder 会按
    // 同样的顺序重新熵编码，产出与解码端兼容的新 blob。
    void setFacetSymbolQueuesForReencode(std::vector<std::deque<char>> q) { facetSymbolQueues = std::move(q); }
    void setEdgeSymbolQueuesForReencode(std::vector<std::deque<char>> q) { edgeSymbolQueues = std::move(q); }
    void setVertexSymbolQueuesForReencode(std::vector<std::deque<char>> q) { vertexSymbolQueues = std::move(q); }
    void setPointQueuesForReencode(std::vector<std::deque<MCGAL::Point>> q) { pointQueues = std::move(q); }
    void setIpointQueuesForReencode(std::vector<std::deque<MCGAL::PointInt>> q) { ipointQueues = std::move(q); }
    void setVertexPointQueuesForReencode(std::vector<std::deque<MCGAL::Point>> q) { vertexPointQueues = std::move(q); }

  protected:
    // ⚠️ protected 而非 private：子类（如 DefaultPPMCLiteSymbolCollectOperator）
    // 需要在覆写的 collect() / exportToBufferWithRangeCoder() 里直接向这些队列
    // 推送数据或读取它们以做 Frenet 旋转等残差变换。语义完全不变。
    std::vector<std::deque<MCGAL::Point>> pointQueues;
    std::vector<std::deque<MCGAL::Point>> vertexPointQueues;
    std::vector<std::deque<MCGAL::PointInt>> ipointQueues;
    std::vector<std::deque<char>> facetSymbolQueues;
    std::vector<std::deque<char>> vertexSymbolQueues;
    std::vector<std::deque<char>> edgeSymbolQueues;
    std::vector<pmsf::RoundSymbols> rounds_;
    /// 与 edgeSymbolQueues 平行：每个 edge sym 对应的"上下文标签"。
    /// 当前定义：1 = 当前 halfedge 所在面被分裂(face.sym=1)，0 = 否。
    /// 仅用于 range-coder 编码端选择上下文模型；不进入序列化流，**解码端通过 BFS 重建状态时同步重算**。
    std::vector<std::deque<char>> edgeSymContextQueues;
    int groupFilter_ = -1;

  private:
    const std::vector<MCGAL::Facet*>* collectFacesOverride_ = nullptr;
};

#endif
