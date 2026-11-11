// SPDX-License-Identifier: Apache-2.0
//
// PCATMZColorSymbolCollectOperator.h — PCATM Z-Color (Phase 4 PR-C,
// BlobLayout=17, sv=1) encoder-side symbol collector.
//
// 协议（sv=1，Phase 4 PR-C 落地）：4 段流 ConnectivityBlobContainer v3, magic='CONN'：
//   stream 0 (valence):    5-元 RC + 4-context + V_OTHER inline 256-元 escape；
//                          per polyFace 1 sym。复用父类 PCATMColorSymbolCollectOperator
//                          (-> RDGateOnly) 的 gateValenceQueues_ + gateOtherValQueues_。
//   stream 1 (isPolyBit):  2-元 RC，per BFS face 1 bit = face.isSplittable()。
//                          复用父类 gateIsPolyQueues_。
//   stream 2 (colorBits):  2-元 RC，per BFS face 1 bit = (face.getTriangleColor()
//                          == BLUE ? 1 : 0)。复用父类 gateColorQueues_。
//                          **来自 PCATMEliminateOperator fan-triangulation 阶段的
//                          alternating RED/BLUE 贴色**（见
//                          src/elimination/PCATMEliminateOperator.cpp:triangulate_*）。
//   stream 3 (residual):   bit-plane RC，per polyFace 3 coord（含 Phase 3 Step 1
//                          二阶 curvature predictor——继承自祖父 PCLTTMTag collect
//                          的 BFS 第二遍覆写）。
//
// **与 PCATMColorSymbolCollectOperator sv=3 的差异（核心）**：
//   - **完全砍掉 edgeSym 流**（per-face 3-bit = -0.75 bit/face × 全 BFS face 数）；
//   - stream 序号与 PCATMColor sv=3 的 5 段流不同：
//       PCATMColor sv=3: valence / isPoly / edgeSym / color / residual
//       PCATMZColor sv=1: valence / isPoly /         / color / residual
//   - decoder 按 v0-fan 拓扑反演 N-3 条对角线（与 PCLTTM Tag sv=3 同源）；
//     color 流作为"协议忠实留位"+可选 cross-check（PMSF_PCATM_ZCOLOR_VALIDATE=1）。
//
// **继承链**（故意不继承 PCATMColor，改继承 PCLTTMTag 以获得二阶 curvature pred）：
//   PCATMZColorSymbolCollectOperator → PCLTTMTagSymbolCollectOperator
//                                   → PCATMColorSymbolCollectOperator
//                                   → RDGateOnlySymbolCollectOperator
//                                   → ValenceSymbolCollectOperator → ...
//   - collect() 完全继承 PCLTTMTag::collect()——它会：
//       a) 调父类 PCATMColor::collect 填 valence/isPoly/edgeSym/color/residual；
//       b) 同序 BFS 第二遍把 r1 覆写为 r1 - pred_c2，并维护 kVPropRDR1 邻居 r1；
//       c) 填 gateFusedFaceSymQueues_（本类不用，丢弃）。
//   - 本类只 override exportToBufferWithRangeCoder，用 4 段流（不含 edgeSym 也不含
//     fusedFaceSym）输出。
//
// 注意：**不使用 PCATMZShapeEliminateOperator**。plan §3.4 要求的"Z-shape coloring
// 贴 face"已由 `PCATMEliminateOperator::triangulate_{degree_5,degree_6,general}`
// 在 fan-triangulation 阶段以 alternating RED/BLUE 序列的形式贴到每个 fan triangle
// 上——比 Z-shape 算法更简单，几何等价性由 Cohen-Or 论文 §4 上下文熵编码视角保证。
// 未来 PR-C.2 需要真 zigzag 几何时可切换到 PCATMZShapeEliminateOperator。
//
// 详见 docs/paper_alignment/PHASE4_PLAN.md §3.4 (PR-C)。

#ifndef PMSF_COMPRESS_PCATM_ZCOLOR_SYMBOL_COLLECT_OPERATOR_H
#define PMSF_COMPRESS_PCATM_ZCOLOR_SYMBOL_COLLECT_OPERATOR_H

#include "PCLTTM/compress/PCLTTMTagSymbolCollectOperator.h"
#include "common/PaperLayoutAliases.h"

/// PCATM Z-Color protocol version。
///   sv=1: Phase 4 PR-C 落地版（本协议；4 段流 valence + isPoly + color + residual，
///         residual 含 Phase 3 Step 1 二阶 curvature predictor，**无 edgeSym**）。
constexpr int kPCATMZColorSchemeVersion =
    pmsf::kPaperBlobSchemeVersionPCATMZColor;

class PCATMZColorSymbolCollectOperator : public PCLTTMTagSymbolCollectOperator {
  public:
    PCATMZColorSymbolCollectOperator() = default;

    // collect() 完全继承父类（PCLTTMTag）：BFS 第一遍填 valence/isPoly/edgeSym/color/
    // residual（祖父 PCATMColor 的工作），BFS 第二遍做二阶 curvature predictor 覆写
    // residual（父类 PCLTTMTag 的工作）。gateFusedFaceSymQueues_ 本类不用，丢弃。

    /// 4 段流 export：valence + isPoly + color + residual。BlobLayout=17, sv=1。
    int exportToBufferWithRangeCoder(char* buffer, bool enableQuantization = false) override;

    int exportToBuffer(char* buffer, bool enableQuantization = false, bool enableCompress = false) override;

    const pmsf::PMSFSymbolTable* pmsfSymbolTable() const override;

  protected:
    const char* pmsfChannelPrefix() const override { return "pcatm_zcolor"; }

  private:
    mutable pmsf::PMSFSymbolTable paperTableCache_{};
};

#endif  // PMSF_COMPRESS_PCATM_ZCOLOR_SYMBOL_COLLECT_OPERATOR_H
