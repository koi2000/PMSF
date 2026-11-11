// SPDX-License-Identifier: Apache-2.0
//
// PCATMPureSymbolReadOperator.h — PCATM 论文"信息论最小"协议 decoder (BlobLayout=21, sv=1)。
//
// 协议（sv=1, 3 段流 ConnectivityBlobContainer v3, magic='CONN'）：
//   stream 0 (valence):    5-元 4-context RC + V_OTHER inline escape
//   stream 1 (isPolyBit):  2-元 RC，per BFS face 1 bit
//   stream 2 (residual):   bit-plane RC（含 Phase 3 Step 1 二阶 curvature predictor）
//
// 字节布局与 RDTag (BlobLayout=18, sv=1) 完全相同，仅 blob_layout 校验号不同（18→21）。
//
// decoder 复用 RDTag/PCLTTMTag 的 symbol 消费与二阶 curvature predictor 还原，
// 但 collectFromDecoded 会把父类 v0-fan diagonal candidates 替换为当前 mesh
// 上沿 added/internal diagonals 收集的 actual patch-tree candidates，并给
// polyFace 标记 protocolHint=20，让 reconstruct 走 strict-paper/Z-shape patch
// 合并分支；若 added 标记不可用，则回退父类 fan candidates。
//
// 详见 docs/paper_alignment/PCATM_PURE_PAPER_NEXT_SESSION_PROMPT.md §C PR-K3。

#ifndef PMSF_DECOMPRESS_PCATM_PURE_SYMBOL_READ_OPERATOR_H
#define PMSF_DECOMPRESS_PCATM_PURE_SYMBOL_READ_OPERATOR_H

#include "RD/decompress/RDTagSymbolReadOperator.h"

#include <vector>

namespace MCGAL {
class Facet;
class Halfedge;
class Vertex;
}  // namespace MCGAL

class PCATMPureSymbolReadOperator : public RDTagSymbolReadOperator {
  public:
    PCATMPureSymbolReadOperator() = default;

    /// @brief 解析 BlobLayout=21 sv=1 的 ConnectivityBlobContainer v3 3 段流。
    ///        字节布局与 RDTag(18) 完全相同，仅 blob_layout 校验不同。
    void importFromRangeCodedBuffer(char* buffer,
                                    size_t& dataOffset,
                                    unsigned nbQuantBits) override;

    void collectFromDecoded(MCGAL::Halfedge* seed,
                            std::vector<MCGAL::Vertex*>& vertices,
                            std::vector<MCGAL::Halfedge*>& halfedge,
                            std::vector<MCGAL::Facet*>& facets) override;
};

#endif  // PMSF_DECOMPRESS_PCATM_PURE_SYMBOL_READ_OPERATOR_H
