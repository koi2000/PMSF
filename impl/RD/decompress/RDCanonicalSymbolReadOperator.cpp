// SPDX-License-Identifier: Apache-2.0
//
// RDCanonicalSymbolReadOperator.cpp — sv=71 三流 blob 解析（见头文件）。
#include "RD/decompress/RDCanonicalSymbolReadOperator.h"

#include "RD/RDCanonicalBlobFormat.h"
#include "RD/RDCanonicalPaperRa.h"
#include "compress/ConnectivityBlobContainer.h"
#include "storage/PaperBlobLayoutRegistry.h"

#include <cstdio>
#include <iostream>
#include <stdexcept>

namespace rd_canon {

void RDCanonicalSymbolReadOperator::importFromRangeCodedBuffer(
    char* buffer, size_t& dataOffset, unsigned /*nbQuantBits*/) {
    if (buffer == nullptr) {
        decodedOk_ = false;
        return;
    }
    // 框架不传真实长度（与 Conquest reader 同款）；容器解析只读 header，
    // 后续 getStream 的切片都在我们写出的 offsets 之内 ⇒ 用大 view 安全。
    constexpr size_t kArbitraryLargeView = static_cast<size_t>(1) << 32;
    std::span<const uint8_t> view(
        reinterpret_cast<const uint8_t*>(buffer + dataOffset),
        kArbitraryLargeView);
    pmsf::ConnectivityBlobReader rd(view);
    const auto& hdr = rd.header();
    // task-54：sv=71 = rounds 按 encoder 顺序（官方 fused / canonical columnar
    // artifact，decoder round i ← container R-1-i）；sv=72 = rounds 已按 decoder
    // 顺序（仅 page-RA profile 使用，decoder round i ← container i）。两者都必须
    // 接受；sv=72 的语义与生成见 RD/RDCanonicalPaperRa.h。
    const int schemeVersion = static_cast<int>(hdr.scheme_version);
    const bool decoderOrderRounds =
        (schemeVersion == rd_canon::kCanonConnSchemeDecoderOrder);
    if (schemeVersion != kCanonBlobSchemeVersion && !decoderOrderRounds) {
        throw std::runtime_error(
            "[RDCanon decoder] expected scheme_version=71 or 72, got " +
            std::to_string(schemeVersion));
    }
    if (static_cast<int>(hdr.num_streams) != kCanonStreamCount) {
        throw std::runtime_error("[RDCanon decoder] expected 3 streams, got " +
                                 std::to_string(hdr.num_streams));
    }
    const int numRounds = rd.numRounds();
    layers_.assign(static_cast<std::size_t>(numRounds), LayerRecord{});
    long long symTotal = 0, nullTotal = 0, resTotal = 0, seedLayers = 0;
    for (int r = 0; r < numRounds; ++r) {
        // sv=71：container 是 encoder 顺序 ⇒ 反序映射到 decoder 层序；
        // sv=72：container 已是 decoder 顺序 ⇒ 直接映射（page-RA prefix 语义）。
        const int di = decoderOrderRounds ? r : (numRounds - 1 - r);
        LayerRecord& layer = layers_[static_cast<std::size_t>(di)];
        decodeSymbolStream(rd.getStream(r, kCanonStreamSymbols), layer.syms);
        decodeResidualStream(rd.getStream(r, kCanonStreamResiduals),
                             layer.residuals);
        layer.hasSeed = decodeSeedStream(rd.getStream(r, kCanonStreamSeed),
                                         layer.seedQ, layer.seedQ + 3);
        if (layer.hasSeed) ++seedLayers;
        for (int sym : layer.syms) {
            if (sym == 4)
                ++nullTotal;
            else
                ++symTotal;
        }
        resTotal += static_cast<long long>(layer.residuals.size());
    }
    layerIndex_ = 0;
    decodedOk_ = true;
    std::fprintf(stderr,
                 "[RD-CANON-DEC] import sv=%d order=%s rounds=%d syms=%lld "
                 "null=%lld residuals=%lld seed_layers=%lld\n",
                 schemeVersion, decoderOrderRounds ? "decoder" : "encoder",
                 numRounds, symTotal, nullTotal, resTotal, seedLayers);
}

void RDCanonicalSymbolReadOperator::collect(
    std::vector<MCGAL::Halfedge*> /*seeds*/, char* /*buffer*/,
    size_t& /*dataOffset*/, std::vector<MCGAL::Vertex*>& /*vertices*/,
    std::vector<MCGAL::Halfedge*>& /*halfedge*/,
    std::vector<MCGAL::Facet*>& /*facets*/) {}

void RDCanonicalSymbolReadOperator::collect(
    MCGAL::Halfedge* /*seed*/, char* /*buffer*/, size_t& /*dataOffset*/,
    std::vector<MCGAL::Vertex*>& /*vertices*/,
    std::vector<MCGAL::Halfedge*>& /*halfedge*/,
    std::vector<MCGAL::Facet*>& /*facets*/) {}

void RDCanonicalSymbolReadOperator::collectFromDecoded(
    MCGAL::Halfedge* /*seed*/, std::vector<MCGAL::Vertex*>& /*vertices*/,
    std::vector<MCGAL::Halfedge*>& /*halfedge*/,
    std::vector<MCGAL::Facet*>& /*facets*/) {
    // 每轮真正的动作在 RDCanonicalReconstructOperator::reconstruct 里。
}

const RDCanonicalSymbolReadOperator::LayerRecord*
RDCanonicalSymbolReadOperator::takeNextLayer() {
    if (!decodedOk_ || layerIndex_ >= layers_.size()) return nullptr;
    return &layers_[layerIndex_++];
}

}  // namespace rd_canon
