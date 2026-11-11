// SPDX-License-Identifier: Apache-2.0
//
// PCLTTMTagSymbolCollectOperator.h — PCLTTM Tag (path-B, BlobLayout=16, sv=2)
// encoder-side symbol collector.
//
// 协议（sv=2，阶段 1 fusedFaceSym 合流）：4 段流 ConnectivityBlobContainer v3, magic='CONN'：
//   stream 0 (valence):       5-元 RC + 4-context + V_OTHER inline 256-元 escape；
//                             per polyFace 1 sym。复用父类 PCATMColor (-> RDGateOnly) 的
//                             gateValenceQueues_ + gateOtherValQueues_。
//   stream 1 (fusedFaceSym):  4-元 alphabet {NotPoly=0, PolyUntagged=1, PolyPlus=2,
//                             PolyMinus=3}，adaptive 1-context qsmodel；per BFS face 1 sym
//                             （每个 BFS 访问的 face 都发，与父类 gateIsPolyQueues_ 同步频率）。
//                             合并旧 isPoly + 旧 vertexTag 两条流。
//   stream 2 (edgeSym):       2-元 RC，per face 3 bit，verbatim 父类。
//   stream 3 (residual):      bit-plane RC，per polyFace 3 coord，verbatim 父类。
//
// sv 历史：
//   sv=1: deprecated (PR-3/PR-4，5 段流 valence + isPoly + edgeSym + vertexTag + residual)。
//   sv=2: 阶段 1 落地（本协议）。
//
// 详见 docs/paper_alignment/PHASE1_FUSED_FACE_SYM_DESIGN.md。

#ifndef PMSF_COMPRESS_PCLTTM_TAG_SYMBOL_COLLECT_OPERATOR_H
#define PMSF_COMPRESS_PCLTTM_TAG_SYMBOL_COLLECT_OPERATOR_H

#include "PCATM/compress/PCATMColorSymbolCollectOperator.h"
#include "common/PaperLayoutAliases.h"
#include "PCLTTM/PCLTTMFrenetPatch.h"

#include <cstdint>
#include <cstdlib>
#include <deque>
#include <vector>

namespace MCGAL {
class Halfedge;
class Facet;
}  // namespace MCGAL

/// PCLTTM Tag protocol version。
///   sv=1: deprecated（PR-3/PR-4 落地版，5 段流 valence + isPoly + edgeSym + vertexTag + residual）；
///   sv=2: deprecated（Phase 1 落地版，4 段流，fusedFaceSym 4-元合流）；
///   sv=3: Phase 2 落地版（3 段流，decoder 按 v0-fan 拓扑 + base_mesh 修复后的
///         proxyHalfedge 对齐推对角线，砍 edgeSym 流）。
/// 详见 docs/paper_alignment/PHASE2_DETERMINISTIC_FAN_DESIGN.md。
constexpr int kPCLTTMTagSchemeVersion =
    pmsf::kPaperBlobSchemeVersionPCLTTMTag;

class PCLTTMTagSymbolCollectOperator : public PCATMColorSymbolCollectOperator {
  public:
    explicit PCLTTMTagSymbolCollectOperator(std::shared_ptr<pmsf::pclttm_frenet::PendingPatches> pending = {})
        : frenetPatches_(std::move(pending)) {}

    /// 主入口：调父类 BFS 收集 valence + isPoly + edgeSym + color + residual，
    /// 然后**额外**遍历 mesh BFS 同序生成 fusedFaceSym 流（per BFS face 1 sym, 4-元）。
    void collect(MCGAL::Halfedge* seed, const std::vector<MCGAL::Halfedge*>& gates) override;
    void collect(MCGAL::Halfedge* seed) override;
    void collect(std::vector<MCGAL::Halfedge*> seeds) override;

    /// 阶段 1 export：4 段流（valence + fusedFaceSym + edgeSym + residual）BlobLayout=16, sv=2。
    int exportToBufferWithRangeCoder(char* buffer, bool enableQuantization = false) override;

    /// plain export 不支持。
    int exportToBuffer(char* buffer, bool enableQuantization = false, bool enableCompress = false) override;

    // -- 测量探针 --
    int lastFusedFaceSymStreamSize() const { return lastFusedFaceSymSize_; }
    int totalFusedFaceSymsEmitted() const {
        int t = 0;
        for (auto& q : gateFusedFaceSymQueues_) t += (int)q.size();
        return t;
    }

    // 兼容旧调用（只是别名）
    int lastVertexTagStreamSize() const { return lastFusedFaceSymSize_; }
    int totalVertexTagsEmitted() const { return totalFusedFaceSymsEmitted(); }
    const pmsf::PMSFSymbolTable* pmsfSymbolTable() const override;

    // Canonical-path accessors.
    const std::vector<std::deque<uint8_t>>& gateFusedFaceSymQueues() const {
        return gateFusedFaceSymQueues_;
    }

  protected:
    const char* pmsfChannelPrefix() const override { return "pclttm_tag"; }

    /// 每轮 BFS 顺序的 fusedFaceSym 流（per BFS face 1 sym, 4-元 alphabet）：
    ///   0 = NotPoly（!isSplittable）
    ///   1 = PolyUntagged
    ///   2 = PolyPlus
    ///   3 = PolyMinus
    /// 与父类 gateIsPolyQueues_ 同步（每 face 1 sym），合并 isPoly + vertexTag。
    std::vector<std::deque<uint8_t>> gateFusedFaceSymQueues_;

    /// residual-facet alignment layer: per polyFace residual index -> Facet*.
    /// Populated in the same BFS order as gateResidualQueues_.
    std::vector<std::deque<MCGAL::Facet*>> gateFacetQueues_;

  public:
    const std::vector<std::deque<MCGAL::Facet*>>& gateFacetQueues() const {
        return gateFacetQueues_;
    }

  protected:

    /// （仅诊断用）每轮每 polyFace 的 pred_c2（论文 §3.1 公式 (2) 二阶预测）。
    /// 默认不填（避免内存开销）；当 env PMSF_RDTAG_LEAKY_DIAG=1 时第二遍 BFS
    /// 会 push_back 此队列，供后续 leaky-factor grid-search 诊断使用。
    /// 队列长度与 gateResidualQueues_ 同（per polyFace 1 项）。
    std::vector<std::deque<MCGAL::PointInt>> gatePredC2Queues_;

    /// （仅诊断用）每轮每 polyFace 的 face normal（论文 §3.1 公式 (3) Frenet 帧）。
    /// 同上，默认不填；env PMSF_RDTAG_LEAKY_DIAG=1 时填充。
    /// 用于诊断：Frenet 投影 (n, t1, t2) 后 Shannon bound 是否显著下降。
    /// 三个 float / polyFace。
    std::vector<std::deque<float>> gateFaceNormalXQueues_;
    std::vector<std::deque<float>> gateFaceNormalYQueues_;
    std::vector<std::deque<float>> gateFaceNormalZQueues_;
    /// proxyHalfedge gate vector (P - Q)，构造 t1 用。
    std::vector<std::deque<float>> gateGateVecXQueues_;
    std::vector<std::deque<float>> gateGateVecYQueues_;
    std::vector<std::deque<float>> gateGateVecZQueues_;

    /// RDTag sv=8 实验：Frenet 投影后的可逆 correction。
    /// 每项为 original_encoded - unproject(project(original_encoded))，decoder
    /// 在反投影后加回该 correction，从而保持 r1 与全局 residual 完全一致。
    std::vector<std::deque<MCGAL::PointInt>> gateFrenetCorrectionQueues_;

  private:
    std::vector<std::uint8_t> encodeFrenetRound(std::size_t encoderRound) const;
    std::shared_ptr<pmsf::pclttm_frenet::PendingPatches> frenetPatches_;
    std::vector<unsigned> frenetDenominators_;
    int lastFusedFaceSymSize_ = 0;
    mutable pmsf::PMSFSymbolTable paperTableCache_{};
};

#endif  // PMSF_COMPRESS_PCLTTM_TAG_SYMBOL_COLLECT_OPERATOR_H
