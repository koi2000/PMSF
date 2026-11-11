/*
 * @file        PCATMColorSymbolCollectOperator.cpp
 * @brief       PCATM 2-color encoder 实现（CP-3.3 β+1 落地版）。
 *
 * 继承自 RDGateOnlySymbolCollectOperator，复用 v3 协议的 BFS + 4 段流编码
 * （valence + isPoly + edgeSym + residual），并新增 1 段 color 流（per BFS face
 * 1 bit, RED=0/BLUE=1）。
 *
 * BlobLayout::CODEC_014 (=14), scheme_version=3。
 *
 * BFS 是父类 BFS 的逐字节副本（拓扑遍历相同），但额外把每访问到的 face 的
 * `getTriangleColor()` 写入 `gateColorQueues_`。这保证了 color 流与
 * gateIsPolyQueues_ 等"per BFS face"流的顺序严格对齐。
 */

#include "PCATM/compress/PCATMColorSymbolCollectOperator.h"
#include "common/ValenceStreamCodec.h"  // pmsf::encodeValence5（P3 抽取）

using pmsf::encodeValence5;
#include "common/PaperLayoutAliases.h"
#include "compress/BFSConquestCollector.h"
#include "compress/RangeCoderStreamEncoder.h"
#include "core/PMSFBoundarySideChannel.h"
#include "PCATM/boundary/PCATMPMSFBoundary.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂
#include "storage/PMSFSymbolStorageAdapter.h"

#include "compress/ConnectivityBlobContainer.h"
#include "Configuration.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Point.h"
#include "core.h"
#include "RD/compress/RDSymbolCollectOperator.h"  // RDValenceCode
#include "common/RDTagCanonicalCodec.h"  // pmsf::rd_tag::encodeBitstream2RC（P7 抽取）

#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <queue>
#include <stdexcept>
#include <string>

// =============================================================================
// collect entry points
// =============================================================================

void PCATMColorSymbolCollectOperator::collect(MCGAL::Halfedge* seed) {
    // 单 seed 不带 gates 的入口（CompressPipeline 的探针 / 兼容路径）：
    // 仍然 delegate 到带 gates 的版本，传一个空 gates。父类协议的 isPoly 数 =
    // gates.size() 断言会 fail 但只是 WARN，不阻塞。
    static const std::vector<MCGAL::Halfedge*> kEmpty;
    collect(seed, kEmpty);
}

void PCATMColorSymbolCollectOperator::collect(std::vector<MCGAL::Halfedge*> seeds) {
    if (seeds.empty()) {
        throw std::runtime_error(
            "[PCATMColor] collect(vector<seed>) called with empty seeds");
    }
    static const std::vector<MCGAL::Halfedge*> kEmpty;
    collect(seeds[0], kEmpty);
}

// =============================================================================
// collect(seed, gates) — 主 BFS（mirror RDGateOnly v3，外加 color 收集）
// =============================================================================

void PCATMColorSymbolCollectOperator::collect(MCGAL::Halfedge* seed,
                                              const std::vector<MCGAL::Halfedge*>& gates) {
    // 1) 复用父类 RDGateOnly 的完整 v3 协议 BFS，填好 isPoly/valence/edgeSym/residual。
    RDGateOnlySymbolCollectOperator::collect(seed, gates);

    // 2) 以**同样的 BFS 顺序**再遍历一次 mesh face，提取 RED/BLUE 颜色。
    //    mesh 状态未被修改，因此第二轮遍历与父类访问顺序完全一致。
    std::deque<uint8_t> colorBits;

    struct ColorVisitor : pmsf::BFSConquestVisitor {
        std::deque<uint8_t>* colorBits;
        void onFaceFirstVisit(MCGAL::Facet* f,
                              MCGAL::Halfedge* /*gate*/,
                              int /*idx*/) override {
            bool isBlue = (f->getTriangleColor() ==
                           MCGAL::Facet::TriangleColorFlag::BLUE);
            colorBits->push_back(isBlue ? 1 : 0);
        }
    };

    ColorVisitor visitor;
    visitor.colorBits = &colorBits;
    pmsf::runFaceConquestBFS(seed, gates, mesh_.get(), visitor);

    // 父类 collect 已经把本轮的 4 个队列压入 history；这里把 color 队列对齐到同一轮。
    gateColorQueues_.push_back(std::move(colorBits));
    paperTableCache_.clear();
}

// =============================================================================
// CP-3.3 sv=3 — 5 段流 RC 编码
// =============================================================================

namespace {

bool pcatmEnvOn(const char* name) {
    return pmsf::pmsfStorageEnvEnabled(name);
}

void requirePCATMExactIfRequested(const char* label) {
    if (!pmsf::boundaryFlagOn(pmsf::pcatm_boundary::kPaperExactSpec)) {
        return;
    }
    throw std::runtime_error(
        std::string(label) +
        " is not paper-exact: PCATM Z-shape / strict paper reconstruction "
        "is still unverified for this non-strict path.");
}

/// @brief stream 0: valence (5 元 + 4-context + V_OTHER inline escape)。
/// 字节布局: [int32 numPolyFaces][int32 numOther][bytes RC]
/// 4-context: 0=first, 1=prev V_3..V_4, 2=prev V_5..V_6, 3=prev V_OTHER。
}  // namespace

const pmsf::PMSFSymbolTable* PCATMColorSymbolCollectOperator::pmsfSymbolTable() const {
    const pmsf::PMSFSymbolTable* base = RDGateOnlySymbolCollectOperator::pmsfSymbolTable();
    if (!base || base->empty()) {
        return nullptr;
    }
    paperTableCache_ = *base;
    const std::string prefix = pmsfChannelPrefix();
    for (std::size_t r = 0; r < paperTableCache_.size() && r < gateColorQueues_.size(); ++r) {
        auto& row = paperTableCache_[r];
        pmsf::appendPCATMColorBoundarySideChannels(
            row.delta,
            prefix,
            static_cast<std::int32_t>(0),
            static_cast<std::int32_t>(1),
            static_cast<std::int32_t>(1),
            static_cast<std::int32_t>(0));
        pmsf::replacePCATMColorBitSymbols(
            row.delta, prefix, gateColorQueues_[r]);
    }
    return paperTableCache_.empty() ? nullptr : &paperTableCache_;
}

int PCATMColorSymbolCollectOperator::exportToBufferWithRangeCoder(char* buffer,
                                                                  bool isEnableQuantization) {
    if (buffer == nullptr) return 0;
    requirePCATMExactIfRequested("[PCATMColor]");
    if (!enablePrediction_ || !isEnableQuantization) {
        throw std::runtime_error(
            "[PCATMColor] exportRC requires enablePrediction && enableQuantization.");
    }

    const int numRounds = (int)gateValenceQueues_.size();
    if ((int)gateIsPolyQueues_.size()  != numRounds ||
        (int)gateEdgeSymQueues_.size() != numRounds ||
        (int)gateResidualQueues_.size()!= numRounds ||
        (int)gateColorQueues_.size()   != numRounds) {
        throw std::runtime_error(
            "[PCATMColor] inconsistent per-round queue sizes (sv=3 expects 5 streams "
            "all aligned)");
    }

    pmsf::ConnectivityBlobBuilder cbb(
        pmsf::pcatmColorConnectivityBlobSpec(numRounds));

    int sumValence = 0, sumIsPoly = 0, sumEdge = 0, sumColor = 0, sumRes = 0;
    const int BP_BITS = (int)mesh_->i_nbQuantBits + 1;

    // 轮次反转：encoder round (N-1-i) → builder round i（与 RDGateOnly / 方案 V 一致）
    for (int i = 0; i < numRounds; ++i) {
        int er = numRounds - 1 - i;

        auto rcVal = encodeValence5(gateValenceQueues_[er],
                                               gateOtherValQueues_[er]);
        sumValence += (int)rcVal.size();
        cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamPCATMColorValence, rcVal);

        // 不传流名 ⇒ 不走注入缝（PCATM 从来没有注入缝，见计划 §20.2）。
        auto rcIsPoly = pmsf::rd_tag::encodeBitstream2RC(gateIsPolyQueues_[er]);
        sumIsPoly += (int)rcIsPoly.size();
        cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamPCATMColorIsPoly, rcIsPoly);

        auto rcEdge = pmsf::rd_tag::encodeBitstream2RC(gateEdgeSymQueues_[er]);
        sumEdge += (int)rcEdge.size();
        cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamPCATMColorEdgeSym, rcEdge);

        auto rcColor = pmsf::rd_tag::encodeBitstream2RC(gateColorQueues_[er]);
        sumColor += (int)rcColor.size();
        cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamPCATMColorColor, rcColor);

        auto rcRes = pmsf::rd_tag::encodeResidualBitplane(gateResidualQueues_[er], BP_BITS);
        sumRes += (int)rcRes.size();
        cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamPCATMColorResidual, rcRes);
    }

    auto blob = cbb.finalize();
    std::memcpy(buffer, blob.data(), blob.size());

    lastColorSize_     = sumColor;
    lastBlobTotalSize_ = (int)blob.size();

    const char* envBpv = std::getenv("PMSF_BPV_DIAG");
    if (envBpv && envBpv[0] == '1') {
        int overhead = (int)blob.size() - sumValence - sumIsPoly - sumEdge - sumColor - sumRes;
        std::cerr << "[PCATMColor bpv sv=3] valence=" << sumValence
                  << " isPoly=" << sumIsPoly
                  << " edge=" << sumEdge
                  << " color=" << sumColor
                  << " residual=" << sumRes
                  << " containerOverhead=" << overhead
                  << " totalBlob=" << blob.size()
                  << " rounds=" << numRounds << std::endl;
    }
    return (int)blob.size();
}

int PCATMColorSymbolCollectOperator::exportToBuffer(char* /*buffer*/,
                                                    bool /*enableQuantization*/,
                                                    bool /*enableCompress*/) {
    throw std::runtime_error(
        "[PCATMColor] exportToBuffer (non-RC) not supported; use Columnar + range-coded blob.");
}
