/*
 * @file RDGateOnlySymbolCollectOperator.h
 * @brief RD Phase B encoder — true gate-only emission with BFS-symmetric protocol (C.1).
 *
 * 与 `RDSymbolCollectOperator` (Phase A，复用 Valence 7 元码 + 全 mesh BFS) 的核心区别：
 *
 *   - **BFS from seed，每 face 首访发 1 bit is-polyFace**（= 本轮 polyFace？）。
 *   - is-polyFace=1 时：emit 5 元 valence code（V_3..V_6, V_OTHER）
 *     + 若 V_OTHER inline 发 uint8 真实 valence（256 元 adaptive）
 *     + emit residual（verbatim Valence 的 bit-plane 路径）。
 *   - is-polyFace=0 时：只发 1 bit，跳过，不污染 valence/residual 流。
 *   - **没有 edge sym 段**：v0-fan triangulation 由 `VertexRemovalEliminateOperator`
 *     固定（`polyVerts[0]` 出发的 N-3 条对角线），decoder 端由
 *     `Mesh::create_center_vertex_without_init` 对称重建，无需 per-halfedge 1-bit。
 *
 * ### 2026-05-09 v3 协议（本文件，几何层修复）
 *
 * 旧协议（v1）：`for g in perRoundGates { emit 1 valence + 1 residual }`，按 selector
 * 顺序发码。V_OTHER 不发真实 valence（bug：decoder 不知道 fan 几个三角形）。
 *
 * v2 协议（已废弃）：BFS-symmetric + is-polyFace bit + V_OTHER escape，但**不发 edge-sym**。
 * 实测拓扑 round-trip ✓ 但几何 RMS 偏差 ×340（v0-fan 反演失败）。
 *
 * 新协议（v3，2026-05-09 第 3 次 session）：在 v2 基础上加回 per-polyFace edge-sym 流：
 *   - 从 seed BFS（与 ValenceSymbolReadOperator::collectFromDecoded 完全对称的
 *     halfedge 入队规则），face 首访时发码。
 *   - V_OTHER inline escape（与 v2 相同）。
 *   - **每个 face 增发 3 个 edge-sym bit**（poly + fan_child + null_patch 都发）：
 *     以 BFS 入口 halfedge `h` 为起点沿 `next()` 走三次，每条 halfedge 的 `isAdded()`
 *     标志各发 1 bit。这样 fan-triangulated 的 N-3 条对角线（polyFace 1 条 + 邻接
 *     fan-child face 各 1 条）全部被 edge-sym 标记。
 *     与 Phase A `ValenceSymbolCollectOperator::collectOneRound:158-166` 类似，
 *     但 Phase A 只在非 NULL_PATCH face 上发，而 v3 在所有 face 上发（实现简单且
 *     RC 对全 0 NULL_PATCH face 的 3 bit 几乎零开销）。
 *   - decoder 端读到这些 1-bit 的 halfedge 后调 `setAdded()` 并 push 到 `halfedge`
 *     列表，BasicReconstructOperator::merge_face 据此把 fan-triangulated 的 N-3 条
 *     对角线 join_face 合回 polygon，再 insert_point 在 polygon 上做 fan-out。
 *   - `gates` 参数仍保留（签名兼容）但从"驱动列表"降级为"断言参考"：
 *     `assert(gates.size() == Σ isPoly=1)`。
 *
 * ### v3.2 尝试（已弃，2026-05-09 第 4 次 session）
 *
 * v3.2：每 face 1 bit `hasAdded` + 仅 hasAdded=1 face 发 3 bit edge-sym。理论上避免
 * NULL_PATCH face 的 3 bit 浪费。**实测 bunny BPV 43.97 → 44.18（+0.21 倒退）**。
 * 信息论根因：v3 的 RC adaptive model 已经对 NULL_PATCH 的全 0 段近似最优编码（~0.066 bit/bit）；
 * 拆成两段后总熵不变，反而多付 RC 框架开销。**结论：要降 BPV 必须减少 raw 信息量
 * 而非改 RC 编码方式**。下一里程碑走 gate-conquest（让 decoder 推断 NULL_PATCH 而非
 * 接收）。
 *
 * ### BlobLayout=8 v3 Header
 *
 *   [rcValSize:int32]           valence RC 流（含 V_OTHER inline escape）
 *   [rcIsPolySize:int32]        is-polyFace bit RC 流
 *   [rcEdgeSize:int32]          ★ NEW v3 — edge-sym RC 流（per-polyFace 3 bit）
 *   [rcPointSize:int32]         residual RC 流
 *   [numFaceBits:int32]         累计 BFS 访问 face 数 = Σ faceBitsPerRound
 *   [numGates:int32]            is-poly=1 的 face 总数 = #valence syms = #residuals
 *   [numEdgeSyms:int32]         ★ NEW v3 — = numGates * 3
 *   [numPointCoords:int32]      = numGates * 3
 *   [numRounds:int32]
 *   [faceBitsPerRound:int32 × numRounds]
 *   [rcValBuf:rcValSize bytes]
 *   [rcIsPolyBuf:rcIsPolySize bytes]
 *   [rcEdgeBuf:rcEdgeSize bytes] ★ NEW v3
 *   [rcPointBuf:rcPointSize bytes]
 *
 * ### 与 Phase A 的并存
 *
 * - Phase A `DefaultRDColumnarCompress` → BlobLayout=7 (RD_GATE) → 走 Valence 解码路径
 * - Phase B `DefaultRDGateOnlyColumnar{Compress,Decompress}` → BlobLayout=8 (RD_GATE_ONLY)
 *   → 走 RDGateOnlySymbolReadOperator（round-trip ✓）
 *
 * 两条路径独立，互不影响。Phase A 的字节哈希 `6eff0565...` 不会因本 patch 改变。
 */
#ifndef RD_GATEONLY_SYMBOLCOLLECTOPERATOR_H
#define RD_GATEONLY_SYMBOLCOLLECTOPERATOR_H

#include "compress/ValenceSymbolCollectOperator.h"
#include <cstdint>
#include <deque>
#include <vector>

namespace MCGAL {
class Halfedge;
class Facet;
class Point;
class PointInt;
}  // namespace MCGAL

/// @brief RD Phase B true gate-only encoder (BFS-symmetric + is-polyFace bit + V_OTHER escape).
class RDGateOnlySymbolCollectOperator : public ValenceSymbolCollectOperator {
  public:
    RDGateOnlySymbolCollectOperator() = default;

    /// @brief Phase B 主入口：BFS from seed，忽略 gates 的顺序（仅用于断言）。
    /// CompressPipeline 在每轮 eliminate 完毕后调用此 overload。
    void collect(MCGAL::Halfedge* seed, const std::vector<MCGAL::Halfedge*>& gates) override;

    /// @brief Phase B 兼容入口。本类不接受无 gates 的 single-seed（上游必须传 gates）。
    /// 直接 throw，防止 PMSF Segmentation 旧路径或探针误用产生假数据。
    void collect(MCGAL::Halfedge* seed) override;
    void collect(std::vector<MCGAL::Halfedge*> seeds) override;

    /// @brief Phase B export：valence RC (含 V_OTHER escape) + is-polyFace RC + 残差位面。
    int exportToBufferWithRangeCoder(char* buffer, bool enableQuantization = false) override;

    /// @brief plain export（Binary 路径）—— Phase B 只走 Parquet V2，不实装。
    int exportToBuffer(char* buffer, bool enableQuantization = false, bool enableCompress = false) override;

    // -- 测量探针 accessor --
    int totalGatesEmitted() const {
        int t = 0;
        for (auto& q : gateValenceQueues_) t += (int)q.size();
        return t;
    }
    int totalFaceBitsEmitted() const {
        int t = 0;
        for (auto& q : gateIsPolyQueues_) t += (int)q.size();
        return t;
    }
    int totalEdgeSymsEmitted() const {
        int t = 0;
        for (auto& q : gateEdgeSymQueues_) t += (int)q.size();
        return t;
    }
    int totalRoundsEmitted() const { return (int)gateValenceQueues_.size(); }
    int lastRcValenceSize() const { return lastRcValenceSize_; }
    int lastRcIsPolySize() const { return lastRcIsPolySize_; }
    int lastRcEdgeSize() const { return lastRcEdgeSize_; }
    int lastRcPointSize() const { return lastRcPointSize_; }

    const pmsf::PMSFSymbolTable* pmsfSymbolTable() const override;

    // Canonical-path read-only accessors.
    const std::vector<std::deque<uint8_t>>& gateValenceQueues() const {
        return gateValenceQueues_;
    }
    const std::vector<std::deque<uint8_t>>& gateOtherValQueues() const {
        return gateOtherValQueues_;
    }
    const std::vector<std::deque<uint8_t>>& gateIsPolyQueues() const {
        return gateIsPolyQueues_;
    }
    const std::vector<std::deque<uint8_t>>& gateEdgeSymQueues() const {
        return gateEdgeSymQueues_;
    }
    const std::vector<std::deque<MCGAL::PointInt>>& gateResidualQueues() const {
        return gateResidualQueues_;
    }

  private:
    // 5 元 alphabet 的 #symbols
    static constexpr int kNumValenceCodes = 5;

  protected:
    // Phase 1 · Step 1.1 把 v3 协议的 5 个 per-round 队列改为 protected，
    // 让派生 `ValencePatchOnlySymbolCollectOperator` 能在 override collect()
    // 之后读取它们做"6 元 alphabet 合并 + 4-context RC"。无功能变化。
    //
    // gateIsPolyQueues_: 每轮 BFS 访问到的 face 各发 1 bit，长度 = 该轮 face 访问数。
    // gateValenceQueues_: 每轮 is-poly=1 的 face 的 5 元 valence 码，长度 = 该轮 polyFace 数。
    // gateOtherValQueues_: 每轮 sym=V_OTHER 的 face 的真实 valence（uint8），长度 = 该轮 V_OTHER 数。
    // gateEdgeSymQueues_: ★ v3 — 每轮 is-poly=1 的 face 的 3 个 edge-sym bit，
    //                     长度 = 该轮 polyFace 数 × 3。顺序：以 BFS 入口 halfedge h
    //                     起，沿 next() 三次每条发 1 bit isAdded()。
    // gateResidualQueues_: 每轮 is-poly=1 的 face 的 PointInt 残差，长度同 gateValenceQueues_。
    std::vector<std::deque<uint8_t>> gateIsPolyQueues_;
    std::vector<std::deque<uint8_t>> gateValenceQueues_;
    std::vector<std::deque<uint8_t>> gateOtherValQueues_;
    std::vector<std::deque<uint8_t>> gateEdgeSymQueues_;
    std::vector<std::deque<MCGAL::PointInt>> gateResidualQueues_;
    mutable pmsf::PMSFSymbolTable paperTableCache_{};

    virtual const char* pmsfChannelPrefix() const { return "rd_gate_only"; }

  private:

    // 最近一次 export 的 RC 段长度（供 PMSF_BPV_DIAG / 工具汇总打印）
    int lastRcValenceSize_ = 0;
    int lastRcIsPolySize_ = 0;
    int lastRcEdgeSize_ = 0;
    int lastRcPointSize_ = 0;
};

#endif  // RD_GATEONLY_SYMBOLCOLLECTOPERATOR_H
