// SPDX-License-Identifier: Apache-2.0
//
// PCLTTMTagSymbolReadOperator.h — PCLTTM Tag (path-B, BlobLayout=16, sv=3)
// decoder 端。
//
// sv=3 Phase 2：砍 edgeSym 流，decoder 按 polyFace 拓扑（proxyHalfedge + valence）
// 重建 N-3 条内部对角线 halfedge 列表，不再消费 edgeSym bit。默认保留历史
// v0-fan 链；`PMSF_PCLTTM_USE_TAG_TABLE_TRIANGULATION=1` 时改为跨 added
// internal edges 收集 patch faces/diagonals，以支持非 fan tag-table 几何。
//
// 协议（sv=3, 3 段流 ConnectivityBlobContainer v3, magic='CONN'）：
//   stream 0 (valence):       5-元 4-context RC + V_OTHER inline escape
//   stream 1 (fusedFaceSym):  4-元 {NotPoly=0, PolyU=1, PolyP=2, PolyM=3}
//   stream 2 (residual):      bit-plane RC
//
// 关键依赖：MCGAL Phase 2 新增 `Facet::reset_proxy_first(vs)` +
//   `ContextPool::allocateFaceFromPoolProxyFirst`，让 ColumnarDeserialize
//   重建 base_mesh 时把 proxyHalfedge 设为 vs[0]→vs[1]，对齐 encoder
//   Columnar 序列化时 faceVids[0] = encoder.proxyHalfedge.vertex 的语义。
//   这是默认 v0-fan 拓扑推断成立的前提；tag-table 模式还要求 encoder
//   用同一环境变量写出 added internal edges。
//
// sv 历史：
//   sv=1: deprecated (PR-3/PR-4 五段流)
//   sv=2: deprecated (Phase 1 四段流)
//   sv=3: Phase 2 落地（本协议）
//
// 详见 docs/paper_alignment/PHASE2_DETERMINISTIC_FAN_DESIGN.md。

#ifndef PMSF_DECOMPRESS_PCLTTM_TAG_SYMBOL_READ_OPERATOR_H
#define PMSF_DECOMPRESS_PCLTTM_TAG_SYMBOL_READ_OPERATOR_H

#include "PCATM/decompress/PCATMColorSymbolReadOperator.h"

#include <cstdint>
#include <deque>
#include <memory>

namespace MCGAL {
class Mesh;
class PointInt;
}  // namespace MCGAL

class PCLTTMTagSymbolReadOperator : public PCATMColorSymbolReadOperator {
  public:
    PCLTTMTagSymbolReadOperator() = default;

    /// @brief 解析 BlobLayout=16 sv=3 的 ConnectivityBlobContainer v3 3 段流。
    void importFromRangeCodedBuffer(char* buffer,
                                    size_t& dataOffset,
                                    unsigned nbQuantBits) override;

    void setPMSFSymbolContext(const pmsf::PMSFSymbolTable* symbols,
                              const pmsf::PMSFSymbolPayloadTable* payloads) override;

    /// @brief BFS + 按 polyFace 拓扑定位对角线（不消费 edgeSym 流）。
    void collectFromDecoded(MCGAL::Halfedge* seed,
                            std::vector<MCGAL::Vertex*>& vertices,
                            std::vector<MCGAL::Halfedge*>& halfedge,
                            std::vector<MCGAL::Facet*>& facets) override;

    unsigned currentFrenetDenominator() const {
        if (frenetDenominators_.empty()) return 0;
        if (currentRoundIdx_ <= 0) throw std::runtime_error("PCLTTM Frenet reconstruct called before collection");
        return frenetDenominators_.at(static_cast<std::size_t>(currentRoundIdx_-1));
    }

    const std::deque<MCGAL::Facet*>& decodedFacetQueue() const {
        return decodedFacetQueue_;
    }

  protected:
    bool expectFrenetRelations_ = false;
    std::vector<unsigned> frenetDenominators_;
    std::deque<MCGAL::Facet*> decodedFacetQueue_;
    std::deque<uint8_t> rcDecodedVertexTags_;
    std::deque<MCGAL::PointInt> rcDecodedFrenetCorrections_;
    bool rcUseFrenetResidual_ = false;
    bool rcUseFrenetExactResidual_ = false;
    int rcDecodedRoundCount_ = 0;
};

#endif  // PMSF_DECOMPRESS_PCLTTM_TAG_SYMBOL_READ_OPERATOR_H
