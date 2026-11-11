// SPDX-License-Identifier: Apache-2.0
//
// PCATMPMSFSymbolReadOperator.cpp — see header for design contract.
//
// 实现策略：PCATM paper 与 PCATMZColor 共享同一个 4 段流物理协议；
// 差异仅在 blob_layout。解码由框架级 PatchColorConnectivityCodec 提供。

#include "PCATM/decompress/PCATMPMSFSymbolReadOperator.h"
#include "common/PaperLayoutAliases.h"

#include "BFSVersionManager.h"
#include "ContextPool.h"
#include "core/PaperProtocolHint.h"
#include "compress/PatchColorConnectivityCodec.h"
#include "Facet.h"
#include "Global.h"
#include "Halfedge.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <queue>
#include <span>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace {

void markPCATMPaperFacets(std::vector<MCGAL::Facet*>& facets) {
    pmsf::markPaperStrictPatchProtocolForEach(facets);
}

void consumeAndApplyPaperColorsByBfs(MCGAL::Halfedge* seed,
                                     int expectedFaceBits,
                                     std::deque<uint8_t>& colors) {
    if (expectedFaceBits <= 0) {
        return;
    }

    std::queue<int> q;
    if (seed != nullptr) {
        q.push(seed->poolId());
    }
    int currentVersion = MCGAL::BFSVersionManager::current_version++;
    int consumed = 0;

    while (!q.empty() && consumed < expectedFaceBits && !colors.empty()) {
        int hid = q.front();
        q.pop();
        MCGAL::Halfedge* h =
            MCGAL::contextPool.getHalfedgeByIndexInSubPool(DEFAULT_MESH_ID, hid);
        if (!h || h->isVisited(currentVersion) || h->isRemoved()) continue;
        h->setVisited(currentVersion);

        MCGAL::Facet* f = h->face();
        if (f && !f->isVisited(currentVersion)) {
            f->setVisited(currentVersion);
            uint8_t color = colors.front();
            colors.pop_front();
            f->setTriangleColor(color
                                    ? MCGAL::Facet::TriangleColorFlag::BLUE
                                    : MCGAL::Facet::TriangleColorFlag::RED);
            f->setPCATMPatchTag(color);
            ++consumed;
        }

        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt ? hIt->opposite() : nullptr;
            if (hOpp && !hOpp->isVisited(currentVersion)) q.push(hOpp->poolId());
            if (hIt && !hIt->isVisited(currentVersion)) q.push(hIt->poolId());
            hIt = hIt ? hIt->next() : nullptr;
        } while (hIt != h && hIt != nullptr);
    }

    while (consumed < expectedFaceBits && !colors.empty()) {
        colors.pop_front();
        ++consumed;
    }
}

std::vector<MCGAL::Halfedge*> collectActualPatchTreeDiagonals(
    MCGAL::Facet* root,
    int targetValence,
    const std::unordered_set<MCGAL::Facet*>& roundRoots) {
    std::vector<MCGAL::Halfedge*> diagonals;
    if (!root || root->isRemoved() || targetValence <= 3) {
        return diagonals;
    }

    const std::size_t targetFaces = static_cast<std::size_t>(targetValence - 2);
    std::unordered_set<MCGAL::Facet*> visited;
    visited.reserve(targetFaces * 2 + 8);
    std::queue<MCGAL::Facet*> q;
    visited.insert(root);
    q.push(root);

    while (!q.empty() && visited.size() < targetFaces) {
        MCGAL::Facet* cur = q.front();
        q.pop();
        MCGAL::Halfedge* start = cur ? cur->proxyHalfedge() : nullptr;
        if (!start) continue;
        MCGAL::Halfedge* h = start;
        do {
            MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
            MCGAL::Facet* nf = opp ? opp->face() : nullptr;
            if (nf && !nf->isRemoved() && !visited.count(nf) &&
                (h->isAdded() || (opp && opp->isAdded())) &&
                (!roundRoots.count(nf) || nf == root)) {
                visited.insert(nf);
                q.push(nf);
                diagonals.push_back(h);
                if (visited.size() >= targetFaces) break;
            }
            h = h ? h->next() : nullptr;
        } while (h && h != start);
    }

    if (visited.size() != targetFaces) {
        diagonals.clear();
    }
    return diagonals;
}

}  // namespace

void PCATMPMSFSymbolReadOperator::importFromRangeCodedBuffer(
    char* buffer,
    size_t& dataOffset,
    unsigned nbQuantBits) {

    constexpr size_t kArbitraryLargeView = static_cast<size_t>(1) << 32;
    std::span<const uint8_t> view(reinterpret_cast<const uint8_t*>(buffer + dataOffset),
                                   kArbitraryLargeView);

    rcUsingResidualLocal_ = header_.isEnablePrediction() && header_.isEnableQuantization();
    if (!rcUsingResidualLocal_) {
        throw std::runtime_error(
            "[PCATMPaper decoder] expects enablePrediction && enableQuantization");
    }
    const int BP_BITS = static_cast<int>(nbQuantBits) + 1;

    auto decoded = pmsf::readPatchColorConnectivityBlob(
        view,
        /*expectedBlobLayout=*/pmsf::kPaperBlobLayoutPCATMPaper,
        /*expectedSchemeVersion=*/pmsf::kPaperBlobSchemeVersionPCATMPaper,
        BP_BITS,
        "PCATMPaper");
    faceBitsPerRound_ = std::move(decoded.faceBitsPerRound);
    rcDecodedValCodes_.insert(rcDecodedValCodes_.end(),
                              decoded.valenceCodes.begin(),
                              decoded.valenceCodes.end());
    rcDecodedOtherVals_.insert(rcDecodedOtherVals_.end(),
                               decoded.otherValences.begin(),
                               decoded.otherValences.end());
    rcDecodedIsPoly_.insert(rcDecodedIsPoly_.end(),
                            decoded.isPolyBits.begin(),
                            decoded.isPolyBits.end());
    rcDecodedColors_.insert(rcDecodedColors_.end(),
                            decoded.colorBits.begin(),
                            decoded.colorBits.end());
    rcDecodedResiduals_.insert(rcDecodedResiduals_.end(),
                               decoded.residuals.begin(),
                               decoded.residuals.end());
    dataOffset += decoded.blobSize;

    const char* env = std::getenv("PMSF_BPV_DIAG");
    if (env && env[0] == '1') {
        std::cerr << "[PCATMPaper decoder sv=1] rounds=" << decoded.numRounds
                  << " totalIsPoly=" << rcDecodedIsPoly_.size()
                  << " totalValCodes=" << rcDecodedValCodes_.size()
                  << " totalColors=" << rcDecodedColors_.size()
                  << " totalResiduals=" << rcDecodedResiduals_.size()
                  << " blobSize=" << decoded.blobSize << std::endl;
    }
}

void PCATMPMSFSymbolReadOperator::collectFromDecoded(
    MCGAL::Halfedge* seed,
    std::vector<MCGAL::Vertex*>& vertices,
    std::vector<MCGAL::Halfedge*>& halfedge,
    std::vector<MCGAL::Facet*>& facets) {
    const int roundIdx = currentRoundIdx_;
    const int expectedFaceBits =
        (roundIdx >= 0 && roundIdx < static_cast<int>(faceBitsPerRound_.size()))
            ? faceBitsPerRound_[static_cast<std::size_t>(roundIdx)]
            : 0;
    consumeAndApplyPaperColorsByBfs(seed, expectedFaceBits, rcDecodedColors_);

    const std::size_t oldFacetCount = facets.size();
    const std::size_t oldHalfedgeCount = halfedge.size();
    PCLTTMTagSymbolReadOperator::collectFromDecoded(seed, vertices, halfedge, facets);
    std::vector<MCGAL::Halfedge*> parentFanCandidates;
    if (halfedge.size() > oldHalfedgeCount) {
        parentFanCandidates.assign(
            halfedge.begin() + static_cast<std::ptrdiff_t>(oldHalfedgeCount),
            halfedge.end());
    }
    std::vector<MCGAL::Facet*> roundFacets;
    if (facets.size() > oldFacetCount) {
        roundFacets.assign(facets.begin() + static_cast<std::ptrdiff_t>(oldFacetCount),
                           facets.end());
    }
    markPCATMPaperFacets(roundFacets);

    std::unordered_set<MCGAL::Facet*> roundRoots(roundFacets.begin(), roundFacets.end());
    halfedge.resize(oldHalfedgeCount);
    std::size_t parentFanCursor = 0;
    for (MCGAL::Facet* f : roundFacets) {
        if (!f) continue;
        const int expectedDiagonals = std::max(0, f->getPolyValence() - 3);
        auto diagonals = collectActualPatchTreeDiagonals(f, f->getPolyValence(), roundRoots);
        if (diagonals.empty() && expectedDiagonals > 0 &&
            parentFanCursor + static_cast<std::size_t>(expectedDiagonals) <=
                parentFanCandidates.size()) {
            diagonals.assign(parentFanCandidates.begin() + static_cast<std::ptrdiff_t>(parentFanCursor),
                             parentFanCandidates.begin() + static_cast<std::ptrdiff_t>(
                                 parentFanCursor + static_cast<std::size_t>(expectedDiagonals)));
        }
        parentFanCursor += static_cast<std::size_t>(expectedDiagonals);
        for (MCGAL::Halfedge* h : diagonals) {
            if (!h || h->isRemoved()) continue;
            h->setAdded();
            if (h->opposite()) h->opposite()->setAdded();
            halfedge.push_back(h);
        }
    }
}
