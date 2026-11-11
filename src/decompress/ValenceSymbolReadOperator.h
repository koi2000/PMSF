/*
 * @Description: Valence-code symbol reader。
 * 布局完全对称于 ValenceSymbolCollectOperator::exportToBufferWithRangeCoder。
 *
 *    header: [rcCodeSize][rcEdgeSize][rcVertexSize?][rcPointSize]
 *            [totalCodes][totalEdgeSymbols][totalVertexSymbols?][totalPointCoords]
 *    data:   [rcCodeBlob][rcEdgeBlob][rcVertexBlob?][rcPointBlob]
 *
 * 消费流程：
 *   1) init() 建立 range coder 状态。
 *   2) importFromRangeCodedBuffer() 在 deserializeBaseMesh 之后一次性解出所有码、边 sym
 *      与 point/residual，缓存到队列。
 *   3) 每轮 deserializeOneRound() 调用 collectFromDecoded(seed)：
 *      * BFS 遍历（与编码端完全相同的顺序）
 *      * 每 halfedge 消费一个 edge bit；isAdded 的 halfedge 压入 halfedge 列表
 *      * face 首次访问消费一个 7 元码；POLY_V_* 设置 splittable + setRemovedVertexPos(Int)
 *      * Adjustment 可选
 *   4) Reconstruct 阶段（BasicReconstructOperator 等）调用 merge_face / insert_point。
 */
#pragma once

#include "SymbolReadOperator.h"
#include "../rangecoder/qsmodel.h"
#include "../rangecoder/rangecod.h"
#include "Vertex.h"
#include "compress/ValenceSymbolCollectOperator.h"
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <queue>
#include <vector>

class ValenceSymbolReadOperator : public SymbolReadOperator {
  public:
    ValenceSymbolReadOperator() = default;

    void collect(MCGAL::Halfedge* seed,
                 char* buffer,
                 size_t& dataOffset,
                 std::vector<MCGAL::Vertex*>& vertices,
                 std::vector<MCGAL::Halfedge*>& halfedge,
                 std::vector<MCGAL::Facet*>& facets) override;

    void collect(std::vector<MCGAL::Halfedge*> seeds,
                 char* buffer,
                 size_t& dataOffset,
                 std::vector<MCGAL::Vertex*>& vertices,
                 std::vector<MCGAL::Halfedge*>& halfedge,
                 std::vector<MCGAL::Facet*>& facets) override;

    void importFromRangeCodedBuffer(char* buffer, size_t& dataOffset, unsigned nbQuantBits) override;

    void collectFromDecoded(MCGAL::Halfedge* seed,
                            std::vector<MCGAL::Vertex*>& vertices,
                            std::vector<MCGAL::Halfedge*>& halfedge,
                            std::vector<MCGAL::Facet*>& facets) override;

  private:
    std::queue<int> gateQueue_;

    // 解码后的全量队列（BFS 顺序）
    std::deque<uint8_t> rcDecodedCodes_;
    std::deque<uint8_t> rcDecodedEdgeSyms_;
    std::deque<uint8_t> rcDecodedVertexSyms_;
    std::deque<MCGAL::Point> rcDecodedPoints_;
    std::deque<MCGAL::Point> rcDecodedVertexPoints_;
    std::deque<MCGAL::PointInt> rcDecodedResiduals_;

    // 预测路径：用 residual，否则用 Point
    bool rcUsingResidual_ = false;
};
