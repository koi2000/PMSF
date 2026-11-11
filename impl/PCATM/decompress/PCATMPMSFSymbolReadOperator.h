// SPDX-License-Identifier: Apache-2.0
//
// PCATMPMSFSymbolReadOperator.h — PCATM 论文严格对齐协议 decoder (BlobLayout=20, sv=1)。
//
// 协议（sv=1, 4 段流 ConnectivityBlobContainer v3, magic='CONN'）：
//   stream 0 (valence):   5-元 4-context RC + V_OTHER inline escape
//   stream 1 (isPolyBit): 2-元 RC，per BFS face 1 bit
//   stream 2 (colorBits): 2-元 RC，per BFS face 1 bit (RED=0/BLUE=1) — alternating
//                         color per paper §4 invariant
//   stream 3 (residual):  bit-plane RC（含 Phase 3 Step 1 二阶 curvature predictor）
//
// 与 PCATMZColorSymbolReadOperator (BlobLayout=17) 的差异：
//   - blob_layout 校验 17 -> 20；
//   - collectFromDecoded 会把父类 v0-fan diagonal candidates 替换为当前 mesh
//     上沿 added/internal diagonals 收集的 actual patch-tree candidates，并给
//     polyFace 标记 protocolHint=20，让 reconstruct 进入 strict-paper/Z-shape
//     patch 合并分支；若 added 标记不可用，则回退父类 fan candidates。
//
// 详见 docs/paper_alignment/PCATM_PURE_PAPER_NEXT_SESSION_PROMPT.md §C PR-K2。

#ifndef PMSF_DECOMPRESS_PCATM_PAPER_SYMBOL_READ_OPERATOR_H
#define PMSF_DECOMPRESS_PCATM_PAPER_SYMBOL_READ_OPERATOR_H

#include "PCATM/decompress/PCATMZColorSymbolReadOperator.h"

#include <cstddef>
#include <vector>

namespace MCGAL {
class Facet;
class Halfedge;
class Vertex;
}  // namespace MCGAL

class PCATMPMSFSymbolReadOperator : public PCATMZColorSymbolReadOperator {
  public:
    PCATMPMSFSymbolReadOperator() = default;

    /// @brief 解析 BlobLayout=20 sv=1 的 ConnectivityBlobContainer v3 4 段流。
    ///        与 PCATMZColor(17) 字节布局完全相同，仅 blob_layout 校验不同。
    void importFromRangeCodedBuffer(char* buffer,
                                    size_t& dataOffset,
                                    unsigned nbQuantBits) override;

    void collectFromDecoded(MCGAL::Halfedge* seed,
                            std::vector<MCGAL::Vertex*>& vertices,
                            std::vector<MCGAL::Halfedge*>& halfedge,
                            std::vector<MCGAL::Facet*>& facets) override;
};

#endif  // PMSF_DECOMPRESS_PCATM_PAPER_SYMBOL_READ_OPERATOR_H
