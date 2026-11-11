// SPDX-License-Identifier: Apache-2.0
//
// PCATMPureSymbolCollectOperator.h — PCATM 论文"信息论最小"协议 (BlobLayout=21, sv=1)
// encoder-side symbol collector. PR-K3 实装。
//
// 与 PCATMPMSFSymbolCollectOperator (BlobLayout=20) 的差异：
//   - **砍 color 段**：4 段流 → 3 段流（valence + isPolyBit + residual）。
//   - blob_layout 20 → 21。
//   - num_streams 4 → 3。
//
// 与 RDTagSymbolCollectOperator (BlobLayout=18) 的差异：
//   - 字节布局完全相同（都是 3 段流）。
//   - 仅 blob_layout 号不同（21 vs 18），供 decoder 区分 encoder eliminate 产出：
//       * RDTag: RDTagEliminateOperator (RD 算法)
//       * PCATMPure: PCATMPaperEliminateOperator (PCATM 算法 + vc barycenter)
//   - 注意：decoder 算法（父类 PCLTTMTag collectFromDecoded）完全相同，但协议号是
//     encoder/decoder 配对的唯一标识，用于验证 parquet 确实由 PCATM 主流程产出。
//
// BPV 预期（bunny V=34834，R=30）：
//   valence    ~ 2.32 bpv
//   isPolyBit  ~ 4.81 bpv
//   residual   ~18.41 bpv
//   connBPV   = 7.13 （= valence + isPolyBit）
//   totalBPV  = 25.54
//
// 详见 docs/paper_alignment/PCATM_PURE_PAPER_NEXT_SESSION_PROMPT.md §C PR-K3。

#ifndef PMSF_COMPRESS_PCATM_PURE_SYMBOL_COLLECT_OPERATOR_H
#define PMSF_COMPRESS_PCATM_PURE_SYMBOL_COLLECT_OPERATOR_H

#include "PCATM/compress/PCATMPMSFSymbolCollectOperator.h"
#include "common/PaperLayoutAliases.h"

/// CODEC_021 protocol version.
///   sv=1: PR-K3 落地版（3 段流 valence + isPolyBit + residual，砍 color）。
constexpr int kPCATMPureSchemeVersion =
    pmsf::kPaperBlobSchemeVersionPCATMPure;
constexpr int kPCATMPureBlobLayout    = pmsf::kPaperBlobLayoutPCATMPure;

class PCATMPureSymbolCollectOperator : public PCATMPMSFSymbolCollectOperator {
  public:
    PCATMPureSymbolCollectOperator() = default;

    // collect() 完全继承父类：BFS 遍历填 valence/isPoly/edgeSym/color/residual 队列。
    // 即使 color 队列被填，本 operator 的 exportRC 不会写出 color 段。

    /// 3 段流 export，BlobLayout=21, sv=1。字节布局与 RDTag sv=1 完全相同，
    /// 仅 ConnectivityBlobContainer header 的 blob_layout 字段不同。
    int exportToBufferWithRangeCoder(char* buffer, bool enableQuantization = false) override;

    const pmsf::PMSFSymbolTable* pmsfSymbolTable() const override;

  protected:
    const char* pmsfChannelPrefix() const override { return "pcatm_pure"; }

  private:
    mutable pmsf::PMSFSymbolTable paperTableCache_{};
};

#endif  // PMSF_COMPRESS_PCATM_PURE_SYMBOL_COLLECT_OPERATOR_H
