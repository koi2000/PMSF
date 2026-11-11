// SPDX-License-Identifier: Apache-2.0
//
// PCATMPMSFSymbolCollectOperator.h — PCATM 论文严格对齐 (BlobLayout=20, sv=1)
// encoder-side symbol collector.
//
// 与 PCATMZColorSymbolCollectOperator 的关系（M1-b Step 2b）：
//   - **协议字节布局完全相同**：4 段流 ConnectivityBlobContainer v3
//       stream 0 (valence):   5-元 4-context RC + V_OTHER inline 256-元 escape
//       stream 1 (isPolyBit): 2-元 RC，per BFS face 1 bit
//       stream 2 (colorBits): 2-元 RC，per BFS face 1 bit (RED=0/BLUE=1)
//       stream 3 (residual):  bit-plane RC，含二阶 curvature predictor（祖父 PCLTTMTag 提供）
//   - **唯一差异**：blob_layout 字段 stamp 20 (CODEC_020) 而非 17 (CODEC_017)。
//     这让 deserializer 能区分两种协议；同时 4 段流的字节解码完全可复用。
//
// 为什么字节布局相同但需要新 BlobLayout？
//   - 协议号是 encoder/decoder 配对协商的唯一标识：
//     * BlobLayout=17 (CODEC_017)：encoder fan + alternating，decoder v0-fan 反演
//     * BlobLayout=20 (CODEC_020) ：encoder 真 Z-shape，decoder Z-shape 反演（M1-b Step 2c）
//   - 若用同 BlobLayout，PCATMPaperEliminate 的真 Z-shape 输出会被误读为 PCATMZColor 的
//     fan 输出，decoder v0-fan 反演会得到错误几何。
//   - 因此协议号必须区分，即使字节布局相同。
//
// 实施策略：
//   - 继承 PCATMZColorSymbolCollectOperator，只 override exportToBufferWithRangeCoder。
//   - override 内部直接复制父类逻辑（4 段流编码），把 ConnectivityBlobBuilder 的
//     blob_layout 参数改为 20 即可。代码量 ~50 行（vs 父类 ~100 行 utility helpers
//     都 reuse 不到，因为是 anonymous namespace）。
//
// 9 算法 baseline 零回归：
//   旧路径（含 DefaultPCATMZColorColumnarCompress）不实例化本类。

#ifndef PMSF_COMPRESS_PCATM_PAPER_SYMBOL_COLLECT_OPERATOR_H
#define PMSF_COMPRESS_PCATM_PAPER_SYMBOL_COLLECT_OPERATOR_H

#include "PCATM/compress/PCATMZColorSymbolCollectOperator.h"
#include "common/PaperLayoutAliases.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂

/// CODEC_020 protocol version。
///   sv=1: M1-b Step 2 落地版（4 段流 valence + isPolyBit + color + residual）。
constexpr int kPCATMPaperSchemeVersion =
    pmsf::kPaperBlobSchemeVersionPCATMPaper;
constexpr int kPCATMPaperBlobLayout    = pmsf::kPaperBlobLayoutPCATMPaper;

class PCATMPMSFSymbolCollectOperator : public PCATMZColorSymbolCollectOperator {
  public:
    PCATMPMSFSymbolCollectOperator() = default;

    // collect() 完全继承父类（PCLTTMTag → PCATMColor → RDGateOnly → ...）：
    //   BFS 第一遍填 valence/isPoly/edgeSym/color/residual（祖父 PCATMColor 工作），
    //   BFS 第二遍做二阶 curvature predictor 覆写 residual（父类 PCLTTMTag 工作）。
    //
    // 注意：collector 不知道 encoder 端是 fan 还是 Z-shape——它只忠实地把每个 face 的
    //   triangleColor / isSplittable / valence / residual 字段读出来写入字节流。
    //   PCATMPaperEliminateOperator 已经把每个 face 的 color 按 Z-shape 的 alternating
    //   规则贴好，本 collector 无需感知 Z-shape 几何。
    //
    // 因此本类**与 PCATMZColor 的差异仅在 export 时 stamp 的 blob_layout 字段**。

    /// 4 段流 export，BlobLayout=20, sv=1。字节布局与 PCATMZColor sv=1 完全相同，
    /// 仅 ConnectivityBlobContainer header 的 blob_layout 字段不同。
    int exportToBufferWithRangeCoder(char* buffer, bool enableQuantization = false) override;

    int exportToBuffer(char* buffer, bool enableQuantization = false, bool enableCompress = false) override;

  protected:
    const char* pmsfChannelPrefix() const override { return "pcatm_paper"; }
};

#endif  // PMSF_COMPRESS_PCATM_PAPER_SYMBOL_COLLECT_OPERATOR_H
