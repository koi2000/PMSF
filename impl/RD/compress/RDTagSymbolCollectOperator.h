// SPDX-License-Identifier: Apache-2.0
//
// RDTagSymbolCollectOperator.h — RD Tag (Phase 4 PR-B, BlobLayout=18)
// encoder-side symbol collector.
//
// 协议（多版本支持）：3 段流 ConnectivityBlobContainer v3, magic='CONN'：
//   stream 0 (valence):       5-元 RC + 4-context + V_OTHER inline 256-元 escape；
//                             per polyFace 1 sym。完全复用父类
//                             PCATMColorSymbolCollectOperator (-> RDGateOnly) 的
//                             gateValenceQueues_ + gateOtherValQueues_。
//   stream 1 (isPolyBit):
//      sv=1 (Phase 4 PR-B 默认)：2-元 1-context RC，per BFS face 1 bit。
//      sv=2 (Phase 5 PR-J1)：    2-元 history multi-context RC。
//                                早期 RLE 实验会增大 Bunny 码流，旧 env 名
//                                `PMSF_RDTAG_ISPOLY_RLE=1` 仅作为别名保留。
//                                sv=3/sv=4 下为避免版本组合歧义，stream 1 固定
//                                使用 sv=1 的 1-context RC。
//   stream 2 (residual):      bit-plane RC，per polyFace 3 coord。**复用 PCLTTM Tag
//                             Phase 3 Step 1 的二阶 curvature predictor**——本类
//                             继承 PCLTTMTagSymbolCollectOperator，其 collect()
//                             已在 BFS 第二遍把 r1 覆写为 r1 - pred_c2，并写邻居
//                             vertex 的 kVPropRDR1。
//   stream 3 (adjustment):    sv=5 only。QEM adjustment 旧 survivor position 相对
//                             当前 QEM placement 的量化残差；与 removed vertex 一样
//                             走 predict/residual，不再存绝对点。
//
// 与 PCLTTMTagSymbolCollectOperator 的差异（极小化）：
//   - 不写 stream 1 fusedFaceSym (4-元)；改写 stream 1 isPolyBit。
//   - 不读 vertex tag —— RD 算法不依赖 PCLTTM Vertex::TagFlag。
//
// 几何还原路径：完全复用 PCLTTM Tag 的 v0-fan 拓扑反演（base_mesh
// proxyHalfedge 量化整数字典序 normalize 已在 Phase 2 修通）。decoder
// 端 RDTagSymbolReadOperator 复用 PCLTTMTag 的 collectFromDecoded BFS，
// 仅在拆解 stream 1 时按 hdr.scheme_version 分支。
//
// 详见 docs/paper_alignment/{PHASE4_PLAN,PHASE5_PLAN}.md。

#ifndef PMSF_COMPRESS_RD_TAG_SYMBOL_COLLECT_OPERATOR_H
#define PMSF_COMPRESS_RD_TAG_SYMBOL_COLLECT_OPERATOR_H

#include "PCLTTM/compress/PCLTTMTagSymbolCollectOperator.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂

#include <cstdint>
#include <deque>
#include <vector>

namespace MCGAL {
class Halfedge;
class PointInt;
}  // namespace MCGAL

/// RD Tag protocol version。
///   sv=1: Phase 4 PR-B 落地版（默认；3 段流 valence + isPolyBit RC + residual，
///         residual 含 Phase 3 Step 1 二阶 curvature predictor）。
///   sv=2: Phase 5 PR-J1 落地版（env PMSF_RDTAG_ISPOLY_MULTICTX=1 或旧别名
///         PMSF_RDTAG_ISPOLY_RLE=1 触发；stream 1 换成 history multi-context RC）。
///   sv=3: stream 2 residual 改为 per-component adaptive bit-plane-major RC。
///   sv=4: sv=3 + signed residual LSB truncation
///         (env PMSF_RDTAG_RESIDUAL_DROP=N).
///   sv=5: QEM adjustment on，新增 stream 3：
///         adjusted vertex bitstream + old survivor position residual。
///   sv=6: QEM adjustment on + edgeSym topology guard，5 段流：
///         valence/isPoly/edgeSym/residual/adjustment。
///   sv=7: sv=3 + residuals stored in a local Frenet frame.
///   sv=8: sv=7 + exact global correction stream.
///   sv=9: sv=3 with valence/isPoly/residual encoded as global streams.
///   sv=10: sv=9 + residuals stored in a local Frenet frame.
///   sv=11: sv=9 + static-probability isPoly stream.
///   sv=12: sv=10 + static-probability isPoly stream.
///   sv=13: sv=9 + headerless per-round packed valence and delta-varint
///          face-count metadata.
///   sv=14: sv=10 + headerless per-round packed valence and delta-varint
///          face-count metadata.
///   sv=15: sv=13 + global history-context isPoly stream.
///   sv=16: sv=13-style global metadata + combined isPoly/valence stream.
///   sv=17: sv=13 + per-round static-probability global isPoly stream.
///   sv=18: sv=15 + residual bit-plane RC split into round buckets.
constexpr int kRDTagSchemeVersionV1 = pmsf::kPaperBlobSchemeVersionRDTagV1;
constexpr int kRDTagSchemeVersionV2 = pmsf::kPaperBlobSchemeVersionRDTagV2;
constexpr int kRDTagSchemeVersionV3 = pmsf::kPaperBlobSchemeVersionRDTagV3;
constexpr int kRDTagSchemeVersionV4 = pmsf::kPaperBlobSchemeVersionRDTagV4;
constexpr int kRDTagSchemeVersionV5 = pmsf::kPaperBlobSchemeVersionRDTagV5;
constexpr int kRDTagSchemeVersionV6 = pmsf::kPaperBlobSchemeVersionRDTagV6;
/// sv=6 的 edgeSym 编码器修正版（PHASE62 物品2）。见 PaperBlobLayoutRegistry.h
/// 里 `kPaperBlobSchemeVersionRDTagV6Ctx` 的说明：流布局与 V6 全等，
/// 只有 edgeSym 那一段的熵编码器不同。
constexpr int kRDTagSchemeVersionV6Ctx =
    pmsf::kPaperBlobSchemeVersionRDTagV6Ctx;
constexpr int kRDTagSchemeVersionV7 = pmsf::kPaperBlobSchemeVersionRDTagV7;
constexpr int kRDTagSchemeVersionV8 = pmsf::kPaperBlobSchemeVersionRDTagV8;
constexpr int kRDTagSchemeVersionV9 = pmsf::kPaperBlobSchemeVersionRDTagV9;
constexpr int kRDTagSchemeVersionV10 = pmsf::kPaperBlobSchemeVersionRDTagV10;
constexpr int kRDTagSchemeVersionV11 = pmsf::kPaperBlobSchemeVersionRDTagV11;
constexpr int kRDTagSchemeVersionV12 = pmsf::kPaperBlobSchemeVersionRDTagV12;
constexpr int kRDTagSchemeVersionV13 = pmsf::kPaperBlobSchemeVersionRDTagV13;
constexpr int kRDTagSchemeVersionV14 = pmsf::kPaperBlobSchemeVersionRDTagV14;
constexpr int kRDTagSchemeVersionV15 = pmsf::kPaperBlobSchemeVersionRDTagV15;
constexpr int kRDTagSchemeVersionV16 = pmsf::kPaperBlobSchemeVersionRDTagV16;
constexpr int kRDTagSchemeVersionV17 = pmsf::kPaperBlobSchemeVersionRDTagV17;
constexpr int kRDTagSchemeVersionV18 = pmsf::kPaperBlobSchemeVersionRDTagV18;

class RDTagSymbolCollectOperator : public PCLTTMTagSymbolCollectOperator {
  public:
    bool preservesLiftedBaseMesh() const override { return false; }
    bool hasGateLevelSymbolCheck() const override { return true; }

    RDTagSymbolCollectOperator() = default;

    // collect() 先继承父类（PCLTTMTag）的 RD_TAG 三段流收集；若 header 开启
    // adjustment，再额外按同一 seed BFS 收集 adjusted vertex bit + 旧点残差。
    void collect(MCGAL::Halfedge* seed, const std::vector<MCGAL::Halfedge*>& gates) override;
    void collect(MCGAL::Halfedge* seed) override;
    void collect(std::vector<MCGAL::Halfedge*> seeds) override;

    // 父类 collect() 语义：
    //   - 父类 BFS 第一遍 (PCATMColor) 填 valence/isPoly/edgeSym/color/residual；
    //   - 父类 BFS 第二遍 (PCLTTMTag) 同序生成 fusedFaceSym 队列 + 二阶 curvature
    //     predictor 覆写 gateResidualQueues_.back()。
    // export 时本类只用 stream 0 (valence) + stream 1 (isPoly, 不用 fusedFaceSym)
    // + stream 2 (residual)，丢弃 stream 中的 fusedFaceSym/edgeSym/color；QEM sv=6
    // 另写 edgeSym topology guard + adjustment residual。

    /// 3 段流 export：valence + isPolyBit + residual。BlobLayout=18。
    /// 默认 sv=1（与 Phase 4 PR-B 一致）；env PMSF_RDTAG_ISPOLY_MULTICTX=1
    /// 切到 sv=2（仅在 adaptive residual 未启用时生效）。
    int exportToBufferWithRangeCoder(char* buffer, bool enableQuantization = false) override;

    int exportToBuffer(char* buffer, bool enableQuantization = false, bool enableCompress = false) override;
    const pmsf::PMSFSymbolTable* pmsfSymbolTable() const override;

    // Canonical-path accessors (read-only views of protected per-round queues).
    const std::vector<std::deque<uint8_t>>& gateValenceQueues() const override {
        return gateValenceQueues_;
    }
    const std::vector<std::deque<uint8_t>>& gateOtherValQueues() const override {
        return gateOtherValQueues_;
    }
    const std::vector<std::deque<uint8_t>>& gateIsPolyQueues() const override {
        return gateIsPolyQueues_;
    }
    const std::vector<std::deque<MCGAL::PointInt>>& gateResidualQueues() const override {
        return gateResidualQueues_;
    }

  protected:
    const char* pmsfChannelPrefix() const override { return "rd_tag"; }

    std::vector<std::deque<uint8_t>> gateVertexAdjustSymbolQueues_;
    std::vector<std::deque<MCGAL::PointInt>> gateVertexAdjustResidualQueues_;
    mutable pmsf::PMSFSymbolTable paperTableCache_{};
};

#endif  // PMSF_COMPRESS_RD_TAG_SYMBOL_COLLECT_OPERATOR_H
