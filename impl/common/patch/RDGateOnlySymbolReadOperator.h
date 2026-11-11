/*
 * @file RDGateOnlySymbolReadOperator.h
 * @brief RD Phase B decoder — 对称消费 RDGateOnlySymbolCollectOperator v3 协议。
 *
 * 见 `src/compress/RDGateOnlySymbolCollectOperator.h` 的协议描述。
 *
 * 与父类 `ValenceSymbolReadOperator` 的差异：
 *   - importFromRangeCodedBuffer：解 4 段 RC 流（valence + isPoly + edge + residual），
 *     valence 段内联 V_OTHER escape。
 *   - collectFromDecoded：BFS from seed，face 首访时先读 isPoly bit；若 1 再读
 *     valence + 可选 V_OTHER escape + 3 个 edge-sym bit + residual。
 *   - edge-sym=1 的 halfedge → setAdded() + push 到 halfedges 列表，
 *     BasicReconstructOperator::merge_face 据此把 fan-triangulated 的 N-3 条对角线
 *     join_face 合回 polygon，再 insert_point 在 polygon 上做 fan-out（v0-fan 反演）。
 *
 * V_OTHER 解出的真实 valence 存入 `rcDecodedOtherVals_`，按 BFS 顺序消费。
 */
#pragma once

#include "decompress/ValenceSymbolReadOperator.h"
#include <deque>

class RDGateOnlySymbolReadOperator : public ValenceSymbolReadOperator {
  public:
    RDGateOnlySymbolReadOperator() = default;

    /// @brief 解析 BlobLayout=8 v3 header 并一次性解 4 段 RC 流。
    void importFromRangeCodedBuffer(char* buffer,
                                    size_t& dataOffset,
                                    unsigned nbQuantBits) override;

    /// @brief BFS from seed，按 is-polyFace bit 驱动消费 valence/edge/residual。
    void collectFromDecoded(MCGAL::Halfedge* seed,
                            std::vector<MCGAL::Vertex*>& vertices,
                            std::vector<MCGAL::Halfedge*>& halfedge,
                            std::vector<MCGAL::Facet*>& facets) override;

  protected:
    // Phase 1 · Step 1.2 把 v3 协议的 5 个解码队列改为 protected，让派生
    // `ValencePatchOnlySymbolReadOperator` 在 override importFromRangeCodedBuffer
    // 后能直接灌数据，复用本类已验证的 collectFromDecoded BFS。
    //
    // 解码缓冲（BFS 顺序队列）
    std::deque<uint8_t> rcDecodedIsPoly_;     // per-face 1 bit（所有轮累加）
    std::deque<uint8_t> rcDecodedValCodes_;   // is-poly=1 的 face 的 5 元 valence code
    std::deque<uint8_t> rcDecodedOtherVals_;  // V_OTHER escape 的真实 valence（uint8）
    std::deque<uint8_t> rcDecodedEdgeSyms_;   // ★ v3 — per-polyFace 3 bit edge-sym
    std::deque<MCGAL::PointInt> rcDecodedResiduals_;  // is-poly=1 的 face 的残差

    // 每轮 BFS 访问的 face 数（用于 per-round trace 同步）
    std::vector<int> faceBitsPerRound_;

    // 本地轮次计数器（每 collectFromDecoded 调用递增）
    int currentRoundIdx_ = 0;

    // 从 header 派生
    bool rcUsingResidualLocal_ = false;

    // 5 元 alphabet
    static constexpr int kNumValenceCodes = 5;
};
