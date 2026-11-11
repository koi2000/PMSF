/*
 * @Author: koi
 * @Date: 2025-09-01 09:36:36
 * @Description:
 */

#pragma once

#include "SymbolReadOperator.h"
#include "Vertex.h"
#include "../rangecoder/qsmodel.h"
#include "../rangecoder/rangecod.h"
#include <cstddef>
#include <deque>
#include <memory>
#include <utility>
#include <vector>
#include "common/FixedQueue.h"

class OptimizedSymbolReadOperator : public SymbolReadOperator {
  public:
    OptimizedSymbolReadOperator() = default;

    void setMesh(std::shared_ptr<MCGAL::Mesh> mesh) { mesh_ = std::move(mesh); }

    void collect(MCGAL::Halfedge* seed,
                 char* buffer,
                 size_t& dataOffset,
                 std::vector<MCGAL::Vertex*>& vertices,
                 std::vector<MCGAL::Halfedge*>& halfedge,
                 std::vector<MCGAL::Facet*>& facets);

    void collect(std::vector<MCGAL::Halfedge*> seeds,
                 char* buffer,
                 size_t& dataOffset,
                 std::vector<MCGAL::Vertex*>& vertices,
                 std::vector<MCGAL::Halfedge*>& halfedge,
                 std::vector<MCGAL::Facet*>& facets);

    void importFromRangeCodedBuffer(char* buffer, size_t& dataOffset, unsigned nbQuantBits);

    /// @brief 把 edge RC 流中的全部 edge symbols 预解码到 rcDecodedEdgeSymbols_。
    /// 用于 codec=4 A3 在编码端测量 Parquet 原生列大小；不影响现有 BFS 解码路径。
    void decodeAllEdgeSymbols(int count);

    // codec=4 A3 访问器：读取已解码的原始符号/坐标序列
    const std::deque<char>& getRcDecodedFacetSymbols() const { return rcDecodedFacetSymbols_; }
    const std::deque<char>& getRcDecodedEdgeSymbols() const { return rcDecodedEdgeSymbols_; }
    const std::deque<char>& getRcDecodedVertexSymbols() const { return rcDecodedVertexSymbols_; }
    const std::deque<MCGAL::Point>& getRcDecodedPoints() const { return rcDecodedPoints_; }
    const std::deque<MCGAL::Point>& getRcDecodedVertexPoints() const { return rcDecodedVertexPoints_; }
    const std::deque<MCGAL::PointInt>& getRcDecodedResiduals() const { return rcDecodedResiduals_; }
    bool isRcUsingResidual() const { return rcUsingResidual_; }

    /// @brief 从 Parquet 解析的结构化符号数据导入内部队列
    void importFromParquet(const std::vector<bool>& facetSymbols,
                           const std::vector<bool>& edgeSymbols,
                           const std::vector<bool>& vertexSymbols,
                           const std::vector<MCGAL::Point>& points) override;

    /// @brief 从 Parquet 解析的结构化残差数据导入内部队列（RD 预测路径）
    void importResidualsFromParquet(const std::vector<MCGAL::PointInt>& residuals) override;

    void collectFromDecoded(MCGAL::Halfedge* seed,
                            std::vector<MCGAL::Vertex*>& vertices,
                            std::vector<MCGAL::Halfedge*>& halfedge,
                            std::vector<MCGAL::Facet*>& facets) override;
    void collectFromDecodedFromFaces(MCGAL::Halfedge* seed,
                                     const std::vector<MCGAL::Facet*>& faces,
                                     std::vector<MCGAL::Vertex*>& vertices,
                                     std::vector<MCGAL::Halfedge*>& halfedge,
                                     std::vector<MCGAL::Facet*>& facets);

  protected:
    // ⚠️ protected 而非 private：子类（如 DefaultPPMCLiteSymbolReadOperator）
    // 需要复用/扩展以下解码状态。语义完全不变。
    FixedQueue<int> gateQueue =  FixedQueue<int>(30000);

    std::deque<char> rcDecodedFacetSymbols_;

    std::deque<char> rcDecodedEdgeSymbols_;
    std::deque<char> rcDecodedVertexSymbols_;
    std::deque<MCGAL::Point> rcDecodedPoints_;
    std::deque<MCGAL::Point> rcDecodedVertexPoints_;
    // RD 预测路径下保存的"残差整数坐标"队列（ZigZag + 位面解码后得到）。
    // 与 rcDecodedPoints_ 互斥使用：prediction=true 时使用此队列。
    std::deque<MCGAL::PointInt> rcDecodedResiduals_;
    bool rcUsingResidual_ = false;

    // === 边符号上下文化解码：BFS 同步消费 ===
    // codec_version=3：edge sym 以 "per-halfedge 但跳过不可分裂 face" 的规则编码。
    // 解码端按同一规则判断是否需要从 RC 读一位。RC 本身是单二元模型。
    bool useEdgeContext_ = false;
    rangecoder edgeRC_{};
    size_t edgeRCOffset_ = 0;
    qsmodel edgeModel_{};
    bool edgeRCInitialized_ = false;
    std::vector<char> edgeRCStorage_;

    // Phase S2：注入方案接管 edge 流时，符号在 import 时整体解出到这个队列，
    // BFS 中的 decodeEdgeSym 与 decodeAllEdgeSymbols 改从这里取（edgeRC_ 不初始化）。
    bool edgeInjected_ = false;
    std::deque<char> injectedEdgeSyms_;

  private:
    bool inGroup(MCGAL::Halfedge* h) const;
    /// 分区路径（`groupId >= 0`，解码端在逐组轮循环里设置）的 BFS 必须与重建出的
    /// 网格的**遍历序**解耦 —— 与编码端 `OptimizedSymbolCollectOperator::
    /// canonicalSegmentedOrder()` 严格镜像；实测与理由见那边的注释。
    bool canonicalSegmentedOrder() const {
        if (std::getenv("PMSF_SEG_FIFO_BFS")) return false;
        return groupId >= 0;
    }
    bool canonicalRemainingFaces() const;
    bool canonicalGateNeighbors() const;
    std::shared_ptr<MCGAL::Mesh> mesh_;
    const std::vector<MCGAL::Facet*>* decodedFacesOverride_ = nullptr;
};
