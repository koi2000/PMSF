#include "VQMMCSymbolCollectOperator.h"
#include "core/PMSFBoundarySideChannel.h"
#include "VQMMC/boundary/VQMMCPMSFBoundary.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include "VQMMC/VectorQuantizedConnectivityCodec.h"
#include "PMSFPayloadAdapterRegistry.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace {

bool vqmmcEnvEnabled(const char* name) {
    return pmsf::pmsfStorageEnvEnabled(name);
}

bool vqmmcSkipCompatTables() {
    return vqmmcEnvEnabled("PMSF_VQMMC_SKIP_COMPAT_TABLES");
}

void requireVQMMCPaperExactIfRequested() {
    if (!pmsf::boundaryFlagOn(pmsf::vqmmc_boundary::kPaperExactSpec)) return;
    throw std::runtime_error(
        "[VQMMC] requested paper-exact VQMMC replay, but the current "
        "impl path only exposes VQ payload/codebook metadata and does not "
        "prove full paper-exact connectivity + geometry replay.");
}

}  // namespace

void VQMMCSymbolCollectOperator::collect(std::vector<MCGAL::Halfedge*> seeds) {
    collect(seeds[0]);
}

void VQMMCSymbolCollectOperator::collect(MCGAL::Halfedge* seed) {
    const auto& roundData = postProcessor_->latestRoundData();
    auto round = pmsf::collectVectorQuantizedConnectivitySymbols(
        mesh_->meshId(), seed, roundData.facetToIndex, roundData.codebook);


    pmsf::RoundSymbols rs;
    rs.facetSymbols = pmsf::toPMSFSymbolCodeDeque(round.facetSyms);
    rs.edgeSymbols = pmsf::toPMSFSymbolCodeDeque(round.edgeSyms);
    const int cbSize = static_cast<int>(round.codebook.size());
    const int idxCount = static_cast<int>(round.indices.size());
    const int idxBits = pmsf::vqIndexBitsForCodebookSize(cbSize);
    pmsf::appendVQMMCPayloadContextSideMap(
        rs.side,
        cbSize,
        idxCount,
        pmsf::unifiedStorageEnabled() &&
                (vqmmcSkipCompatTables() ||
                 vqmmcEnvEnabled("PMSF_VQMMC_SKIP_COMPAT_SYMBOLS_TABLE"))
            ? 1
            : 0,
        pmsf::unifiedStorageEnabled() &&
                (vqmmcSkipCompatTables() ||
                 vqmmcEnvEnabled("PMSF_VQMMC_SKIP_COMPAT_VQ_TABLE"))
            ? 1
            : 0,
        pmsf::unifiedStorageEnabled() &&
                (vqmmcSkipCompatTables() ||
                 vqmmcEnvEnabled("PMSF_VQMMC_SKIP_COMPAT_BASE_TABLES"))
            ? 1
            : 0);
    pmsf::appendPMSFBoundarySideMapAbsentExactPair(
        rs.side, "vqmmc", "paper", pmsf::boundaryFlagI32(pmsf::vqmmc_boundary::kPaperExactSpec));
    for (const PTTreeNode& node : roundData.progressiveTree) {
        rs.side["vq_tree_node_id"].push_back(static_cast<std::int32_t>(node.nodeId));
        rs.side["vq_tree_left"].push_back(static_cast<std::int32_t>(node.leftChild));
        rs.side["vq_tree_right"].push_back(static_cast<std::int32_t>(node.rightChild));
        rs.side["vq_tree_depth"].push_back(static_cast<std::int32_t>(node.depth));
        rs.side["vq_tree_codeword"].push_back(node.codeword);
    }
    for (uint8_t code : round.faceCodes) {
        rs.side["vq_face_code"].push_back(static_cast<std::int8_t>(code));
    }
    for (uint8_t val : round.otherVals) {
        rs.side["vq_other_valence"].push_back(static_cast<std::int8_t>(val));
    }
    for (uint8_t bit : round.localEdgeSyms) {
        rs.side["vq_local_edge_symbol"].push_back(static_cast<bool>(bit != 0));
    }
    for (uint8_t val : round.splitValences) {
        rs.side["vq_split_valence"].push_back(static_cast<std::int8_t>(val));
    }
    for (uint8_t val : round.splitOtherVals) {
        rs.side["vq_split_other_valence"].push_back(static_cast<std::int8_t>(val));
    }
    for (uint8_t slot : round.rootDiagSlots) {
        rs.side["vq_root_diag_slot"].push_back(static_cast<std::int8_t>(slot));
    }
    rounds_.push_back(std::move(rs));

    facetSymbolQueues_.push_back(std::move(round.facetSyms));
    edgeSymbolQueues_.push_back(std::move(round.edgeSyms));
    faceCodeQueues_.push_back(std::move(round.faceCodes));
    otherValQueues_.push_back(std::move(round.otherVals));
    localEdgeSymbolQueues_.push_back(std::move(round.localEdgeSyms));
    splitValenceQueues_.push_back(std::move(round.splitValences));
    splitOtherValQueues_.push_back(std::move(round.splitOtherVals));
    rootDiagSlotQueues_.push_back(std::move(round.rootDiagSlots));
    vqIndexQueues_.push_back(std::move(round.indices));
    splitFacetQueues_.push_back(std::move(round.splitFacets));
    codebookQueues_.push_back(std::move(round.codebook));
    paperPayloadTableCache_.clear();
    paperPayloadTableCacheValid_ = false;
}

void VQMMCSymbolCollectOperator::refreshGeometryFromPostProcessor() {
    const auto& allRounds = postProcessor_->allRoundData();
    if (allRounds.size() != codebookQueues_.size() ||
        allRounds.size() != vqIndexQueues_.size() ||
        allRounds.size() != splitFacetQueues_.size()) {
        throw std::runtime_error(
            "[VQMMC] geometry refresh round count mismatch");
    }
    for (std::size_t round = 0; round < allRounds.size(); ++round) {
        const auto& data = allRounds[round];
        codebookQueues_[round] = data.codebook;
        if (vqIndexQueues_[round].size() != splitFacetQueues_[round].size()) {
            throw std::runtime_error(
                "[VQMMC] geometry refresh index/facet count mismatch");
        }
        for (std::size_t i = 0; i < splitFacetQueues_[round].size(); ++i) {
            auto it = data.facetToIndex.find(splitFacetQueues_[round][i]);
            if (it == data.facetToIndex.end()) {
                throw std::runtime_error(
                    "[VQMMC] geometry refresh missing facet index");
            }
            vqIndexQueues_[round][i] = it->second;
        }
    }
    paperPayloadTableCache_.clear();
    paperPayloadTableCacheValid_ = false;
}

const pmsf::PMSFSymbolPayloadTable*
VQMMCSymbolCollectOperator::paperSymbolPayloadTable() const {
    if (paperPayloadTableCacheValid_) {
        return paperPayloadTableCache_.empty() ? nullptr : &paperPayloadTableCache_;
    }
    if (vqIndexQueues_.empty() && codebookQueues_.empty()) {
        paperPayloadTableCacheValid_ = true;
        return nullptr;
    }

    paperPayloadTableCache_.clear();
    pmsf::appendVQMMCPMSFPayloadRows(paperPayloadTableCache_,
                                      vqIndexQueues_,
                                      codebookQueues_);
    paperPayloadTableCacheValid_ = true;

    return paperPayloadTableCache_.empty() ? nullptr : &paperPayloadTableCache_;
}

int VQMMCSymbolCollectOperator::exportToBuffer(char* buffer,
                                               bool /*isEnableQuantization*/,
                                               bool /*isEnableCompress*/) {
    if (buffer) {
        requireVQMMCPaperExactIfRequested();
    }
    auto usePredictiveResidual = []() -> bool {
        const char* absEnv = std::getenv("VQMMC_ABS_GEOM");
        if (absEnv && *absEnv && std::strcmp(absEnv, "0") != 0) {
            return false;
        }
        const char* env = std::getenv("VQMMC_USE_PREDICTIVE_RESIDUAL");
        if (env && *env) {
            return std::strcmp(env, "0") != 0;
        }
        return true;
    };
    return pmsf::writeLegacyVectorQuantizedConnectivityBuffer(
        buffer,
        facetSymbolQueues_,
        edgeSymbolQueues_,
        vqIndexQueues_,
        codebookQueues_,
        !usePredictiveResidual());
}
