// SPDX-License-Identifier: Apache-2.0
//
// PCATMZColorSymbolCollectOperator.cpp — see header for design contract.
//
// 实施策略（最小 diff）：
//   1. collect() 完全继承 PCLTTMTagSymbolCollectOperator，不重写；
//   2. exportToBufferWithRangeCoder 重写：只写 4 段流 (valence + isPoly + color
//      + residual)，砍 edgeSym (PCATMColor sv=3 中的 stream 2) 与 fusedFaceSym
//      (PCLTTMTag sv=3 中的 stream 1)。blob_layout=17, sv=1。
//
// 字节布局由框架级 PatchColorConnectivityCodec 提供：
//   stream 0: valence, stream 1: isPoly, stream 2: color, stream 3: residual.

#include "PCATM/compress/PCATMZColorSymbolCollectOperator.h"
#include "common/PaperLayoutAliases.h"
#include "core/PMSFBoundarySideChannel.h"
#include "PCATM/boundary/PCATMPMSFBoundary.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂
#include "storage/PMSFSymbolStorageAdapter.h"

#include "compress/PatchColorConnectivityCodec.h"
#include "Mesh.h"

#include <cstring>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

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
        " is not paper-exact: exact Z-shape / strict paper reconstruction "
        "is still unverified for this path.");
}

}  // namespace

// ============================================================================
// export — ConnectivityBlobBuilder v3 (4 段流，sv=1)
// ============================================================================
int PCATMZColorSymbolCollectOperator::exportToBufferWithRangeCoder(char* buffer,
                                                                    bool isEnableQuantization) {
    if (buffer == nullptr) return 0;
    requirePCATMExactIfRequested("[PCATMZColor]");
    if (!enablePrediction_ || !isEnableQuantization) {
        throw std::runtime_error(
            "[PCATMZColor] exportRC requires enablePrediction && enableQuantization.");
    }

    const int numRounds = (int)gateValenceQueues_.size();
    if ((int)gateIsPolyQueues_.size()   != numRounds ||
        (int)gateResidualQueues_.size() != numRounds ||
        (int)gateColorQueues_.size()    != numRounds) {
        throw std::runtime_error(
            "[PCATMZColor sv=1] inconsistent per-round queue sizes "
            "(expects valence/isPoly/color/residual aligned by round)");
    }

    const int BP_BITS = (int)mesh_->i_nbQuantBits + 1;
    auto sizes = pmsf::writePatchColorConnectivityBlob(
        buffer,
        /*blobLayout=*/pmsf::kPaperBlobLayoutPCATMZColor,
        kPCATMZColorSchemeVersion,
        gateValenceQueues_,
        gateOtherValQueues_,
        gateIsPolyQueues_,
        gateColorQueues_,
        gateResidualQueues_,
        BP_BITS);

    const char* envBpv = std::getenv("PMSF_BPV_DIAG");
    if (envBpv && envBpv[0] == '1') {
        const int overhead = sizes.blobBytes - sizes.payloadBytes();
        std::cerr << "[PCATMZColor bpv sv=1] valence=" << sizes.valence
                  << " isPolyBit=" << sizes.isPoly
                  << " colorBits=" << sizes.color
                  << " residual=" << sizes.residual
                  << " containerOverhead=" << overhead
                  << " totalBlob=" << sizes.blobBytes
                  << " rounds=" << numRounds << std::endl;
    }
    return sizes.blobBytes;
}

int PCATMZColorSymbolCollectOperator::exportToBuffer(char* /*buffer*/,
                                                      bool /*enableQuantization*/,
                                                      bool /*enableCompress*/) {
    throw std::runtime_error(
        "[PCATMZColor] exportToBuffer (non-RC) is not implemented.");
}

const pmsf::PMSFSymbolTable* PCATMZColorSymbolCollectOperator::pmsfSymbolTable() const {
    const pmsf::PMSFSymbolTable* base = PCATMColorSymbolCollectOperator::pmsfSymbolTable();
    if (!base || base->empty()) {
        return nullptr;
    }
    const std::string prefix = pmsfChannelPrefix();
    const bool strictPaper = prefix == "pcatm_paper";
    pmsf::copyPMSFSymbolTableWithRowAugmentation(
        base,
        paperTableCache_,
        [&prefix, strictPaper](pmsf::PMSFSymbolRecord& row) {
        pmsf::appendPCATMZColorPaperLayoutSideChannel(
            row.delta, prefix, strictPaper);
    });
    return paperTableCache_.empty() ? nullptr : &paperTableCache_;
}
