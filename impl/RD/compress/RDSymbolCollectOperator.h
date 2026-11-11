/*
 * @file RDSymbolCollectOperator.h
 * @brief RD paper-aligned gate-conquest collector（Phase A）。
 *
 * ### 设计选择（Phase A）
 *
 * 本次实现采取**保守而可 round-trip** 的方案：直接继承 `ValenceSymbolCollectOperator`，
 * 共享 byte layout（同一 `exportToBufferWithRangeCoder` / `importFromRangeCodedBuffer`
 * 协议），仅通过**独立的 BlobLayout::RD_GATE (=7)** 把它和 VALENCE_RC 区分开。
 *
 * 这样做的理由：
 *   1. Valence 已经在 PMSF 全链路（encoder → Parquet → decoder → Hausdorff 校验）
 *      verified，复用它最大化"第一版就 round-trip"的概率。
 *   2. 后续 Phase B（真·gate-only BFS、Frenet 旋转残差、level 奇偶 cleaning-decimating
 *      码字拆分）可以**仅修改 RD 子类**，完全不触动 Valence。Default/Valence/PPMC/Lite
 *      的 baseline 保持字节级不变。
 *   3. 独立 blob_layout 保证 decoder 侧永远走 RDSymbolReadOperator 的入口；即使将来
 *      encoder 内部协议分叉，解码不会误入 Valence 路径。
 *
 * ### Phase A 行为
 *
 * - 收集：走 ValenceSymbolCollectOperator::collect() 的 BFS（每 face 首访发一个 7 元
 *   code，polyFace 携带 residual 残差 + 3 个 fan 边 sym）。码字 alphabet 仍是 Valence 的。
 *   虽然名字叫 "RDValenceCode"（保留供未来 Phase B 使用），Phase A 里用的是 Valence 同款。
 * - 导出：默认委托 base class 的 `exportToBufferWithRangeCoder`；若显式要求
 *   paper-exact RD，则 fail-fast，避免把 Phase A 兼容流误标成论文严格复现。
 * - BlobLayout：序列化器 stamps `BlobLayout::RD_GATE (7)`，解码时走 RDSymbolReadOperator。
 *
 * ### Phase B 拓展点（不在本次实现）
 *
 * - 真·gate-only：encoder 只对 polyFace 发 code，decoder 通过 gate 队列的 N-boundary
 *   expansion + 每条 boundary 1 bit（"邻 face 是否 polyFace"）重建 gate 集合。期待
 *   将 code 段从 ~0.95 symbol/face 压到 ~1 symbol/V（×10 倍缩减）。
 * - Frenet-rotated residuals：把残差映射到局部切平面 (α,β,γ)，进一步剪减高频位面。
 * - v=3 cleaning-layer 码字拆分（论文 §3.6 decimating/cleaning 奇偶层）。
 *
 * @see docs/RD_PAPER_ALIGNMENT_PROGRESS.md §B.1–B.3
 */
#ifndef RD_SYMBOLCOLLECTOPERATOR_H
#define RD_SYMBOLCOLLECTOPERATOR_H

#include "compress/ValenceSymbolCollectOperator.h"
#include <cstdint>
#include <vector>

namespace MCGAL {
class Halfedge;
class Facet;
}

/// @brief RD gate-level valence alphabet（5 元，为未来 Phase B 预留）。
/// 与 Valence 的 7 元 alphabet 的关键区别：Phase B 将不存在 NULL_PATCH / FAN_CHILD ——
/// gate-level encoder 只对本轮 delta polyFace 发 symbol。Phase A 保留 enum 并未使用，
/// 仍复用 Valence 的 7 元码以保证 round-trip。
enum class RDValenceCode : uint8_t {
    VAL_3     = 0,
    VAL_4     = 1,
    VAL_5     = 2,
    VAL_6     = 3,
    VAL_OTHER = 4,  // 包含 valence >= 7（稀有）
    NUM_CODES = 5
};

/// @brief RD paper 风格收集器（Phase A：复用 Valence 码流，与之 byte-compatible）。
///
/// 通过继承 `ValenceSymbolCollectOperator` 直接复用 BFS 收集 + RC 导出逻辑。
/// Phase A 唯一的差别在序列化侧：`ColumnarSerializeOperator` 会把本类的实例
/// stamp 为 `BlobLayout::RD_GATE=7`，确保解码端走 `RDSymbolReadOperator`。
class RDSymbolCollectOperator : public ValenceSymbolCollectOperator {
  public:
    RDSymbolCollectOperator() = default;

    // Phase A：收集仍复用父类 Valence 的 BFS；导出默认委托父类 RC 布局。
    // 仅在显式要求 paper-exact RD 时 fail-fast。
    // Phase B 将覆写 collect(seed, gates) 走 gate-only BFS，并调整
    // exportToBufferWithRangeCoder 的码字 + 残差模型。

    // Re-export base class's 2-arg `collect(seed, gates)` overload that
    // `ValenceSymbolCollectOperator` hides via name-hiding (it only overrides
    // the 1-arg + vector forms). Required by the PMSF_RD_DIAG probe path
    // (`PMSFUnified.cpp`) which still calls `collect(seed, perRoundGates)`.
    // Base impl forwards to `collect(seed)` so behaviour is unchanged.
    using SymbolCollectOperator::collect;

    int exportToBufferWithRangeCoder(char* buffer,
                                     bool enableQuantization = false) override;

    const pmsf::PMSFSymbolTable* pmsfSymbolTable() const override;

    // ---- probe 兼容 accessors ----
    // B.1 Sprint 2 probe 曾暴露 `totalGates / gatesPerRound / valenceHistogram`。
    // Phase A 保留静态入口以兼容调用方（tools/diag）。
    int totalGates() const {
        int t = 0;
        for (auto& q : getValenceCodeQueues()) t += (int)q.size();
        return t;
    }
    int totalRounds() const { return (int)getValenceCodeQueues().size(); }

    /// @brief 兼容 probe 旧接口：把 Valence 7 元码映射回 RD 5 元码用于 histogram。
    ///        non-POLY codes（NULL_PATCH / FAN_CHILD）映射到 VAL_OTHER。
    static RDValenceCode encodeValence(int valence);
    static int           decodeValence(RDValenceCode c);
};

#endif  // RD_SYMBOLCOLLECTOPERATOR_H
