/*
 * @file        ValencePatchOnlySymbolCollectOperator.h
 * @brief       Phase 1 · Step 1.1 — 方案 V (Valence-Patch-Only) encoder.
 * @author      koi
 * @date        2026-05-10
 *
 * 见 docs/CONNECTIVITY_BITSTREAM_IMPLEMENTATION_PLAN.md §1。
 *
 * 与父类 `RDGateOnlySymbolCollectOperator` 的字节级差异：
 *
 *   父类 (RD_GATE_ONLY=8 sv=v3) 的 4 段 RC 流：
 *     - rcVal:    5 元 alphabet (V_3..V_OTHER) + V_OTHER 256-元 escape
 *     - rcIsPoly: 2 元 (per BFS-visited face 1 bit)
 *     - rcEdge:   2 元 (per polyFace 3 bit)
 *     - rcPoint:  bit-plane (per polyFace 3 ZigZag)
 *
 *   方案 V (VALENCE_PATCH_ONLY=9) 的 3 段流，封装进 ConnectivityBlobContainer:
 *     - stream 0 (valence6): 6 元 alphabet (NULL_PATCH=0, V_3=1..V_6=4, V_OTHER=5)
 *                            + V_OTHER 256-元 escape；用 ContextAdaptiveRangeCoder
 *                            的 4-context 模型编码（first / prev=NULL / prev=V*
 *                            / prev=OTHER）。emit 长度 = #BFS-visited face；
 *                            合并掉父类的 rcVal+rcIsPoly 两段。
 *     - stream 1 (edgeSym):  per-polyFace 3 bit（保留父类 rcEdge 段以保证 v0-fan
 *                            几何 round-trip；v2 协议去 edge-sym 已实测 RMS×340 偏差）。
 *                            ContextAdaptiveRangeCoder 2 元 / 1-context（即 RC 普通模式）。
 *     - stream 2 (residual): bit-plane 残差（verbatim 父类 rcPoint 路径，调用
 *                            原有 bpFirst1/bpAfter1 模型）。
 *
 * BFS 顺序：完全 mirror 父类 collect(seed, gates) —— 同一个 BFSVersionManager
 * version、同一个 enqueue 规则（opposite 先入 / 自身后入 / 沿 next 三次）。
 *
 * 收益预期：
 *   - 合并 rcVal+rcIsPoly 后，"NULL_PATCH 1-bit" 进入 RC 自适应模型成为高频字节，
 *     不再独立成段；同时 4-context 比单 qsmodel 利用了"上一帧 patch 类型"的相关性。
 *   - 实测 BPV 应 ≤ 父类 RDGateOnly v3 的 44.19，目标 §1 KPI ≤ 5。
 *     （如果 BPV 没达标，根因可能在 edge 段或 residual 段，需要再迭代。）
 *
 * 子类约束：
 *   - 不接受无 gates 的 single-seed collect()（同父类直接 throw）。
 *   - exportToBufferWithRangeCoder 必须 enablePrediction && enableQuantization。
 *   - exportToBuffer (binary 路径) throw —— 与父类一致。
 */

#ifndef PMSF_COMPRESS_VALENCE_PATCH_ONLY_SYMBOL_COLLECT_OPERATOR_H
#define PMSF_COMPRESS_VALENCE_PATCH_ONLY_SYMBOL_COLLECT_OPERATOR_H

#include "common/patch/RDGateOnlySymbolCollectOperator.h"

#include <cstdint>
#include <vector>

namespace MCGAL {
class Halfedge;
class PointInt;
}  // namespace MCGAL

/// @brief 方案 V 6-元 alphabet。NULL_PATCH=0 是关键差异，把父类 rcIsPoly 段并入。
enum class ValencePatchCode : uint8_t {
    NULL_PATCH = 0,
    VAL_3      = 1,
    VAL_4      = 2,
    VAL_5      = 3,
    VAL_6      = 4,
    VAL_OTHER  = 5,
    NUM_CODES  = 6
};

/// @brief 方案 V context 模型：4 个 context（first / prev=NULL / prev=V* / prev=OTHER）。
/// 实现：alphabet_size=8（6 元 padding 到 8 的二进制上界，sym=6/7 永不出现），
/// **qsmodel lg_totf=10**（与父类 RDGateOnly rcVal/rcIsPoly/rcEdge/rcPoint 一致）。
/// 注意：`lg_totf` 是频率表总和的位数，与 alphabet 大小无数学耦合；设置过小
/// （例如 3）会让 adaptive 频率精度退化到 ~0.125 粒度，导致 valence6 段从
/// 信息论下界 ~30KB 膨胀到 ~130KB（bunny 上 BPV +14）。
constexpr int kValencePatchAlphabetBits = 3;  // alphabet_size = 8（实际只用 0..5）
constexpr int kValencePatchContextBits  = 2;  // context_count = 4

class ValencePatchOnlySymbolCollectOperator : public RDGateOnlySymbolCollectOperator {
  public:
    ValencePatchOnlySymbolCollectOperator() = default;

    /// @brief Phase 1 主入口：BFS from seed，gates 为 invariant 断言。
    void collect(MCGAL::Halfedge* seed, const std::vector<MCGAL::Halfedge*>& gates) override;

    /// @brief 不支持的入口：直接 throw（与父类一致）。
    void collect(MCGAL::Halfedge* seed) override;
    void collect(std::vector<MCGAL::Halfedge*> seeds) override;

    /// @brief 方案 V export：3 段流封装进 ConnectivityBlobContainer，
    ///        magic='CONN', layout=VALENCE_PATCH_ONLY (=9), sv=1。
    int exportToBufferWithRangeCoder(char* buffer,
                                     bool enableQuantization = false) override;

    /// @brief plain export 不支持。
    int exportToBuffer(char* buffer,
                       bool enableQuantization = false,
                       bool enableCompress = false) override;

    const pmsf::PMSFSymbolTable* pmsfSymbolTable() const override;

    // -- 测量探针 --
    int totalValenceSymbolsEmitted() const;
    int lastValence6StreamSize() const { return lastValence6Size_; }
    int lastEdgeStreamSize() const { return lastEdgeSize_; }
    int lastResidualStreamSize() const { return lastResidualSize_; }
    int lastBlobTotalSize() const { return lastBlobTotalSize_; }

  private:
    // 把父类 BFS 产出的 (gateIsPolyQueues_, gateValenceQueues_, gateOtherValQueues_)
    // 三段 raw 数据投影到 6-元 alphabet 序列（per round）。
    //
    // 投影规则（按 BFS 访问顺序）：
    //   - isPoly=0 → emit ValencePatchCode::NULL_PATCH (=0)；
    //   - isPoly=1 → 父类 5 元 (V_3..V_OTHER) → +1 映射成 (VAL_3..VAL_OTHER)。
    //
    // 输出长度等于 gateIsPolyQueues_[r].size() —— 每 BFS face 一个 symbol。
    static std::vector<uint8_t> project6Alphabet(
        const std::deque<uint8_t>& isPoly,
        const std::deque<uint8_t>& valenceCodes5);

    // 用 ContextAdaptiveRangeCoder 编码 6 元 + V_OTHER escape；返回 RC 字节流。
    static std::vector<uint8_t> encodeValence6WithContext(
        const std::vector<uint8_t>& codes6,
        const std::deque<uint8_t>& otherVals);

    // 用单 qsmodel (2 元 adaptive) 编码 edge-sym 段，verbatim 父类 rcEdge。
    static std::vector<uint8_t> encodeEdgeSyms(const std::deque<uint8_t>& edgeSyms);

    // bit-plane 残差编码 —— 这一段我们直接借用父类已经验证的实现路径：
    // 把残差 ZigZag → 沿 BP_BITS 位面 → bpFirst1/bpAfter1 各 1 个 qsmodel。
    // 调用方提供 BP_BITS（来自 mesh_->i_nbQuantBits + 1）。
    std::vector<uint8_t> encodeResidualBitplane(
        const std::deque<MCGAL::PointInt>& residuals,
        int BP_BITS);

  protected:
    const char* pmsfChannelPrefix() const override { return "valence_patch_only"; }

  private:
    int lastValence6Size_   = 0;
    int lastEdgeSize_       = 0;
    int lastResidualSize_   = 0;
    int lastBlobTotalSize_  = 0;
};

#endif  // PMSF_COMPRESS_VALENCE_PATCH_ONLY_SYMBOL_COLLECT_OPERATOR_H
