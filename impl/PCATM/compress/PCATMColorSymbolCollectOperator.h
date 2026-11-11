/*
 * @file        PCATMColorSymbolCollectOperator.h
 * @brief       PCATM 论文 2-color 编码 encoder（Cohen-Or et al. 2002）。
 * @author      koi
 * @date        2026-05-10
 *
 * ## CP-3.3 β+1 落地版（继承 RDGateOnly）
 *
 * 协议：BlobLayout::CODEC_014 (=14), scheme_version=3。
 * 5 段流封装进 ConnectivityBlobContainer v3：
 *   stream 0 (valence):  5 元 alphabet {V_3, V_4, V_5, V_6, V_OTHER} + 4-context
 *                        RC + V_OTHER inline uint8 escape；每 polyFace 1 sym。
 *   stream 1 (isPoly):   2 元 RC，每 BFS face 1 bit = face.isSplittable()。
 *   stream 2 (edgeSym):  2 元 RC，每 BFS face 3 bit = `h.isAdded()` (BFS 入口
 *                        halfedge 起 next() 三步)。decoder 据此 setAdded() +
 *                        push 到 halfedge 列表，merge_face 把对角线 join 回 polygon。
 *                        与 RDGateOnly v3 完全对称的对角线信号。
 *   stream 3 (color):    2 元 RC，每 BFS face 1 bit = (BLUE ? 1 : 0)。
 *                        来自 PCATMEliminateOperator Z-shape RED/BLUE 着色。
 *                        ★ CP-3.3 阶段：作 PCATM 论文协议忠实留位，未参与对角线推断；
 *                        未来优化路径（β+2）可用 color + valence + mesh 对称改造
 *                        推对角线、去 edgeSym。
 *   stream 4 (residual): bit-plane RC（per-polyFace 3 coord，verbatim 父类路径）。
 *
 * ## 继承关系
 *
 * 继承自 `RDGateOnlySymbolCollectOperator` —— 复用 v3 协议的 BFS、4 段流编码、
 * residual bit-plane 等成熟代码；**只增**一段 color 流（per BFS face 1 bit），
 * 把 PCATM Z-shape 着色从 mesh 上 extract 到字节流，保持 RDGateOnly 几何反演
 * 路径（v0-fan + edgeSym 标对角线）100% 兼容。
 *
 * ## scheme_version 历史
 *
 *   sv=1 (CP-3.2，commit 8b7e62c, deprecated): 3 段流 (valence, color, residual)。
 *       conquest 假设被推翻，无法解码。
 *   sv=2 (CP-3.3 探索, deprecated): 4 段流 +isPoly。decoder 仍无法做对角线推断
 *       （proxyHalfedge 不对称），未落地。
 *   sv=3 (CP-3.3 β+1 落地): 5 段流，继承 RDGateOnly v3 协议 + 加 color。
 *       BPV 实测 ≈ RDGateOnly 44.19 + color ~1 bpv ≈ 45 bpv。round-trip 保证。
 *
 * ## BFS 顺序对称性
 *
 * 完全复用父类 `RDGateOnlySymbolCollectOperator::collect(seed, gates)` 的 BFS。
 * collect() 在父类填完 4 个 protected gateXxxQueues_ 之后，**遍历同样 BFS 顺序**
 * 的 mesh face，extract `face.getTriangleColor()` 写入新 `gateColorQueues_`。
 * 因此 color 流顺序与 isPoly/edgeSym 流的"per BFS face"顺序严格对齐。
 */

#ifndef PMSF_COMPRESS_PCATM_COLOR_SYMBOL_COLLECT_OPERATOR_H
#define PMSF_COMPRESS_PCATM_COLOR_SYMBOL_COLLECT_OPERATOR_H

#include "common/patch/RDGateOnlySymbolCollectOperator.h"
#include "common/PaperLayoutAliases.h"
#include <cstdint>
#include <deque>
#include <vector>

namespace MCGAL {
class Halfedge;
class Facet;
}  // namespace MCGAL

#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂

/// @brief CODEC_014 protocol version。CP-3.3 β+1 落地版 = 3。
constexpr int kPCATMColorSchemeVersion =
    pmsf::kPaperBlobSchemeVersionPCATMColor;

class PCATMColorSymbolCollectOperator : public RDGateOnlySymbolCollectOperator {
  public:
    PCATMColorSymbolCollectOperator() = default;

    /// @brief 主入口：delegate 父类 BFS，然后**重新 BFS**（同规则）extract color。
    ///        实际实现里直接 override 父类整个 BFS 循环以一次走完，详见 .cpp。
    void collect(MCGAL::Halfedge* seed, const std::vector<MCGAL::Halfedge*>& gates) override;
    void collect(MCGAL::Halfedge* seed) override;
    void collect(std::vector<MCGAL::Halfedge*> seeds) override;

    /// @brief CP-3.3 sv=3 export：5 段流 ConnectivityBlobContainer v3。
    int exportToBufferWithRangeCoder(char* buffer, bool enableQuantization = false) override;

    /// @brief plain export 不支持。
    int exportToBuffer(char* buffer, bool enableQuantization = false, bool enableCompress = false) override;

    // -- 测量探针 --
    int lastColorStreamSize() const { return lastColorSize_; }
    int lastBlobTotalSize()    const { return lastBlobTotalSize_; }
    const pmsf::PMSFSymbolTable* pmsfSymbolTable() const override;

    /// 每轮 BFS 顺序的 color bit 流（per BFS face 1 bit, RED=0/BLUE=1）。
    /// 与父类 gateIsPolyQueues_ 等长，每 face 一一对应。
    const std::vector<std::deque<uint8_t>>& gateColorQueues() const {
        return gateColorQueues_;
    }

  protected:
    const char* pmsfChannelPrefix() const override { return "pcatm_color"; }

    std::vector<std::deque<uint8_t>> gateColorQueues_;

  private:
    int lastColorSize_     = 0;
    int lastBlobTotalSize_ = 0;
    mutable pmsf::PMSFSymbolTable paperTableCache_{};
};

#endif  // PMSF_COMPRESS_PCATM_COLOR_SYMBOL_COLLECT_OPERATOR_H
