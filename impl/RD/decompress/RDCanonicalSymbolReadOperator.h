// SPDX-License-Identifier: Apache-2.0
//
// RDCanonicalSymbolReadOperator.h — task-43 route A 解码端的**符号读取器**（薄）。
//
// 职责：把 sv=71 三流 blob 按 RDCanonicalBlobFormat.h 的 framing 读回内存，
// 供 RDCanonicalReconstructOperator 逐层取走。**不做解码/不动网格**（解码实体
// 只有 RDCanonicalTraversal.h 里的 decodeLayer 一处）。
//
// 层序：blob 的 container round r = encoder 第 r 层（自然序写）；解码端第 i 轮
// 对应 encoder 第 (N-1-i) 层 ⇒ 解析时直接存进 decoder-order 数组，顺序消费。
#ifndef PMSF_IMPL_RD_DECOMPRESS_RD_CANONICAL_SYMBOL_READ_OPERATOR_H
#define PMSF_IMPL_RD_DECOMPRESS_RD_CANONICAL_SYMBOL_READ_OPERATOR_H

#include "decompress/SymbolReadOperator.h"

#include "Point.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace rd_canon {

class RDCanonicalSymbolReadOperator : public SymbolReadOperator {
  public:
    struct LayerRecord {
        std::vector<int> syms;
        std::vector<MCGAL::PointInt> residuals;
        bool hasSeed = false;
        int seedQ[6] = {0, 0, 0, 0, 0, 0};
    };

    RDCanonicalSymbolReadOperator() = default;
    ~RDCanonicalSymbolReadOperator() = default;

    void importFromRangeCodedBuffer(char* buffer, size_t& dataOffset,
                                    unsigned nbQuantBits) override;

    // 框架纯虚契约（本 layout 走"队列已装填"支，不会被调用）。
    void collect(std::vector<MCGAL::Halfedge*> seeds, char* buffer,
                 size_t& dataOffset, std::vector<MCGAL::Vertex*>& vertices,
                 std::vector<MCGAL::Halfedge*>& halfedge,
                 std::vector<MCGAL::Facet*>& facets) override;
    void collect(MCGAL::Halfedge* seed, char* buffer, size_t& dataOffset,
                 std::vector<MCGAL::Vertex*>& vertices,
                 std::vector<MCGAL::Halfedge*>& halfedge,
                 std::vector<MCGAL::Facet*>& facets) override;

    void collectFromDecoded(MCGAL::Halfedge* seed,
                            std::vector<MCGAL::Vertex*>& vertices,
                            std::vector<MCGAL::Halfedge*>& halfedge,
                            std::vector<MCGAL::Facet*>& facets) override;

    bool decodedOk() const {
        return decodedOk_;
    }
    int layerCount() const {
        return static_cast<int>(layers_.size());
    }
    std::size_t layerCursor() const {
        return layerIndex_;
    }
    /// 取走下一层（decoder order）；越界返回 nullptr（固定轮数解码的正常收尾）。
    const LayerRecord* takeNextLayer();

  private:
    std::vector<LayerRecord> layers_;
    std::size_t layerIndex_ = 0;
    bool decodedOk_ = false;
};

}  // namespace rd_canon

#endif  // PMSF_IMPL_RD_DECOMPRESS_RD_CANONICAL_SYMBOL_READ_OPERATOR_H
