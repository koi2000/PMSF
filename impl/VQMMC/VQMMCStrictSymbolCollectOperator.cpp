/*
 * VQMMCStrictSymbolCollectOperator — Range-Coder implementation.
 */
#include "VQMMCStrictSymbolCollectOperator.h"
#include "core/PMSFBoundarySideChannel.h"
#include "VQMMC/boundary/VQMMCPMSFBoundary.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include "VQMMC/VectorQuantizedConnectivityCodec.h"

#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>

namespace {

bool useMepp2Storage() {
    const char* env = std::getenv("VQMMC_MEPP2");
    return env && *env && std::strcmp(env, "0") != 0;
}

bool useValenceStorage() {
    const char* env = std::getenv("VQMMC_MEPP2_VALENCE");
    return env && *env && std::strcmp(env, "0") != 0;
}

bool usePaperConnectivityStorage() {
    const char* env = std::getenv("VQMMC_PAPER_CONN");
    return env && *env && std::strcmp(env, "0") != 0;
}

bool envEnabled(const char* name) {
    return pmsf::pmsfStorageEnvEnabled(name);
}

bool requirePaperExactReplay() {
    return pmsf::boundaryFlagOn(pmsf::vqmmc_boundary::kPaperExactSpec);
}

bool requirePaperConnectivityStorage() {
    return pmsf::boundaryFlagOn(pmsf::vqmmc_boundary::kPaperConnectivitySpec);
}

void requireStrictPaperCapabilitiesIfRequested(bool paperConnMode) {
    if (requirePaperExactReplay()) {
        throw std::runtime_error(
            "[VQMMC-Strict] requested paper-exact VQMMC replay, but BlobLayout=30 "
            "only proves the strict VQ payload/container boundary; full paper-exact "
            "connectivity and geometry replay are not implemented.");
    }
    if (requirePaperConnectivityStorage() && !paperConnMode) {
        throw std::runtime_error(
            "[VQMMC-Strict] paper connectivity storage was required, but "
            "VQMMC_PAPER_CONN is not enabled.");
    }
}

bool usePredictiveResidual() {
    const char* absEnv = std::getenv("VQMMC_ABS_GEOM");
    if (absEnv && *absEnv && std::strcmp(absEnv, "0") != 0) {
        return false;
    }
    const char* env = std::getenv("VQMMC_USE_PREDICTIVE_RESIDUAL");
    if (env && *env) return std::strcmp(env, "0") != 0;
    return true;
}

}  // namespace

const pmsf::PMSFSymbolTable* VQMMCStrictSymbolCollectOperator::pmsfSymbolTable() const {
    const pmsf::PMSFSymbolTable* base = SymbolCollectOperator::pmsfSymbolTable();
    if (!base || base->empty()) {
        return nullptr;
    }
    const bool mepp2 = useMepp2Storage();
    const bool paperConn = usePaperConnectivityStorage();
    const bool valence = mepp2 && useValenceStorage();
    const bool compact = mepp2 && !valence && !paperConn;
    for (auto& round : paperTableCache_) {
        auto& side = round.delta.side;
        side["vqmmc_strict_storage"].push_back(static_cast<std::int32_t>(1));
        side["vqmmc_strict_blob_layout"].push_back(
            static_cast<std::int32_t>(kVQMMCStrictBlobLayout));
        side["vqmmc_strict_container_v3"].push_back(static_cast<std::int32_t>(1));
        side["vqmmc_mepp2_storage_requested"].push_back(
            static_cast<std::int32_t>(mepp2 ? 1 : 0));
        side["vqmmc_paper_connectivity_storage"].push_back(
            static_cast<std::int32_t>(paperConn ? 1 : 0));
        side["vqmmc_valence_storage_requested"].push_back(
            static_cast<std::int32_t>(valence ? 1 : 0));
        side["vqmmc_compact_storage_requested"].push_back(
            static_cast<std::int32_t>(compact ? 1 : 0));
        side["vqmmc_predictive_residual"].push_back(
            static_cast<std::int32_t>(usePredictiveResidual() ? 1 : 0));
        pmsf::appendPMSFBoundarySideMapAbsentExactPair(
            side, "vqmmc", "paper", pmsf::boundaryFlagI32(pmsf::vqmmc_boundary::kPaperExactSpec));
        side["vqmmc_paper_connectivity_required"].push_back(
            static_cast<std::int32_t>(requirePaperConnectivityStorage() ? 1 : 0));
    }
    return paperTableCache_.empty() ? nullptr : &paperTableCache_;
}

int VQMMCStrictSymbolCollectOperator::exportToBuffer(char* buffer,
                                                      bool /*enableQuantization*/,
                                                      bool /*enableCompress*/) {
    if (!buffer) return 0;
    const int numRounds = (int)facetSymbolQueues().size();
    if (numRounds == 0) return 0;
    const bool mepp2Mode = useMepp2Storage();
    const bool paperConnMode = usePaperConnectivityStorage();
    const bool valenceMode = mepp2Mode && useValenceStorage();
    const bool compactMode = mepp2Mode && !valenceMode && !paperConnMode;
    requireStrictPaperCapabilitiesIfRequested(paperConnMode);

    const pmsf::VQConnectivityStorageMode mode =
        paperConnMode ? pmsf::VQConnectivityStorageMode::PaperFan
                      : (valenceMode ? pmsf::VQConnectivityStorageMode::Valence
                                     : (compactMode ? pmsf::VQConnectivityStorageMode::Compact
                                                    : pmsf::VQConnectivityStorageMode::Classic));
    pmsf::VQStreamSizes streamSizes;
    auto blob = pmsf::writeVectorQuantizedConnectivityBlob(
        kVQMMCStrictBlobLayout,
        mode,
        facetSymbolQueues(),
        edgeSymbolQueues(),
        faceCodeQueues(),
        otherValQueues(),
        localEdgeSymbolQueues(),
        splitValenceQueues(),
        splitOtherValQueues(),
        rootDiagSlotQueues(),
        vqIndexQueues(),
        codebookQueues(),
        usePredictiveResidual(),
        &streamSizes);


    if (blob.size() > (size_t)BUFFER_SIZE) {
        std::cerr << "[VQMMC-Strict] blob exceeds BUFFER_SIZE" << std::endl;
        return 0;
    }
    std::memcpy(buffer, blob.data(), blob.size());
    return (int)blob.size();
}
