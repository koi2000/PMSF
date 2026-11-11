// SPDX-License-Identifier: Apache-2.0
//
// VQMMCCodebookSymbolCollectOperator.cpp — VQMMC codebook protocol (BlobLayout=23).
//
// Inherits RDTagSymbolCollectOperator for the full BFS.
// Overrides export to write 3 streams with blob_layout=23.
// Full blob-level VQ codebook+indices integration is deferred; the unified
// paper sidecar records the current mode explicitly.

#include "VQMMCCodebookSymbolCollectOperator.h"
#include "common/ValenceStreamCodec.h"  // pmsf::encodeValence5（P3 抽取）
#include "common/AlphabetStreamCodec.h"  // pmsf::encodeAlphabetStream（P11 抽取）

using pmsf::encodeValence5;
#include "common/PaperLayoutAliases.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂
#include "core/PMSFBoundarySideChannel.h"
#include "VQMMC/boundary/VQMMCPMSFBoundary.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include "compress/ConnectivityBlobContainer.h"
#include "compress/RangeCoderStreamEncoder.h"
#include "Configuration.h"
#include "Mesh.h"
#include "Point.h"
#include "RD/compress/RDSymbolCollectOperator.h"
#include "PMSFPayloadAdapterRegistry.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

constexpr int kBufScratch = BUFFER_SIZE / 12;

bool vqmmcEnvEnabled(const char* name) {
    return pmsf::pmsfStorageEnvEnabled(name);
}

void requireVQMMCPaperExactIfRequested(const char* scope) {
    if (!pmsf::boundaryFlagOn(pmsf::vqmmc_boundary::kPaperExactSpec)) return;
    throw std::runtime_error(
        std::string(scope) +
        " requested paper-exact VQMMC replay, but the current codebook "
        "path keeps a byte-compatible RDTag transport plus VQ payload table; "
        "full paper-exact connectivity and geometry replay are not proven.");
}


std::vector<uint8_t> encodeResidualBP(const std::deque<MCGAL::PointInt>& residuals,
                                       int BP_BITS) {
    const size_t bufSize = std::max<size_t>(BUFFER_SIZE / 3, residuals.size() * 3 + 1024);
    std::vector<char> buf(bufSize);
    size_t rcSize = pmsf::encodeResidualBitPlaneQueue(residuals, BP_BITS, buf.data(), bufSize);

    std::vector<uint8_t> out;
    out.resize(4);
    int32_t numCoords = (int32_t)(residuals.size() * 3);
    std::memcpy(out.data(), &numCoords, 4);
    out.insert(out.end(), reinterpret_cast<uint8_t*>(buf.data()),
               reinterpret_cast<uint8_t*>(buf.data()) + rcSize);
    return out;
}

}  // namespace

const pmsf::PMSFSymbolTable* VQMMCCodebookSymbolCollectOperator::pmsfSymbolTable() const {
    const pmsf::PMSFSymbolTable* base = RDTagSymbolCollectOperator::pmsfSymbolTable();
    if (!base || base->empty()) {
        return nullptr;
    }
    pmsf::copyPMSFSymbolTableWithIndexedRowAugmentation(
        base,
        paperTableCache_,
        [&](std::size_t r, pmsf::PMSFSymbolRecord& row) {
        const int codebookSize = r < codebookQueues_.size()
                                     ? static_cast<int>(codebookQueues_[r].size())
                                     : 0;
        const int indexCount = r < vqIndexQueues_.size()
                                   ? static_cast<int>(vqIndexQueues_[r].size())
                                   : 0;
        pmsf::appendVQMMCCodebookPayloadStorage(
            row.delta, codebookSize, indexCount);
    });
    return paperTableCache_.empty() ? nullptr : &paperTableCache_;
}

const pmsf::PMSFSymbolPayloadTable*
VQMMCCodebookSymbolCollectOperator::paperSymbolPayloadTable() const {
    if (payloadTableCacheValid_) {
        return payloadTableCache_.empty() ? nullptr : &payloadTableCache_;
    }
    payloadTableCache_.clear();
    pmsf::appendVQMMCPMSFPayloadRows(payloadTableCache_,
                                      vqIndexQueues_,
                                      codebookQueues_);
    payloadTableCacheValid_ = true;

    return payloadTableCache_.empty() ? nullptr : &payloadTableCache_;
}

int VQMMCCodebookSymbolCollectOperator::exportToBufferWithRangeCoder(
    char* buffer, bool isEnableQuantization) {
    if (buffer == nullptr) return 0;
    requireVQMMCPaperExactIfRequested("[VQMMC-CB]");
    if (!enablePrediction_ || !isEnableQuantization) {
        throw std::runtime_error(
            "[VQMMC-CB] exportRC requires enablePrediction && enableQuantization.");
    }

    const int numRounds = (int)gateValenceQueues_.size();
    pmsf::ConnectivityBlobBuilder cbb(
        pmsf::vqmmcLegacyCodebookConnectivityBlobSpec(numRounds));

    int sumValence = 0, sumIsPoly = 0, sumRes = 0;
    const int BP_BITS = (int)mesh_->i_nbQuantBits + 1;

    for (int i = 0; i < numRounds; ++i) {
        int er = numRounds - 1 - i;

        auto rcVal = encodeValence5(gateValenceQueues_[er], gateOtherValQueues_[er]);
        sumValence += (int)rcVal.size();
        cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamVQMMCLegacyValence, rcVal);

        auto rcIsPoly = pmsf::encodeAlphabetStream(gateIsPolyQueues_[er], 2);
        sumIsPoly += (int)rcIsPoly.size();
        cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamVQMMCLegacyIsPoly, rcIsPoly);

        auto rcRes = encodeResidualBP(gateResidualQueues_[er], BP_BITS);
        sumRes += (int)rcRes.size();
        cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamVQMMCLegacyResidual, rcRes);
    }

    auto blob = cbb.finalize();
    std::memcpy(buffer, blob.data(), blob.size());

    const char* envBpv = std::getenv("PMSF_BPV_DIAG");
    if (envBpv && envBpv[0] == '1') {
        int overhead = (int)blob.size() - sumValence - sumIsPoly - sumRes;
        std::cerr << "[VQMMC-CB bpv] valence=" << sumValence
                  << " isPolyBit=" << sumIsPoly
                  << " residual=" << sumRes
                  << " containerOverhead=" << overhead
                  << " totalBlob=" << blob.size()
                  << " rounds=" << numRounds << std::endl;
    }
    return (int)blob.size();
}

int VQMMCCodebookSymbolCollectOperator::exportToBuffer(char*, bool, bool) {
    throw std::runtime_error("[VQMMC-CB] exportToBuffer (non-RC) not implemented.");
}
