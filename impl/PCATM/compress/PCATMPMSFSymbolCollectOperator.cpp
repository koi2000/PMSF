// SPDX-License-Identifier: Apache-2.0
//
// PCATMPMSFSymbolCollectOperator.cpp — see header for design contract.
//
// 实现策略（M1-b Step 2b）：
//   collect 继承 PCATMZColor；export 只指定 CODEC_020 的 blob layout。
//   4 段流的 valence/isPoly/color/residual 编码由框架级
//   PatchColorConnectivityCodec 统一提供。

#include "PCATM/compress/PCATMPMSFSymbolCollectOperator.h"

#include "compress/PatchColorConnectivityCodec.h"
#include "Mesh.h"

#include <cstring>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

// ============================================================================
// export — ConnectivityBlobBuilder v3 (4 段流，sv=1, blob_layout=20)
// ============================================================================
int PCATMPMSFSymbolCollectOperator::exportToBufferWithRangeCoder(char* buffer,
                                                                   bool isEnableQuantization) {
    if (buffer == nullptr) return 0;
    if (!enablePrediction_ || !isEnableQuantization) {
        throw std::runtime_error(
            "[PCATMPaper] exportRC requires enablePrediction && enableQuantization.");
    }

    const int numRounds = (int)gateValenceQueues_.size();
    if ((int)gateIsPolyQueues_.size()   != numRounds ||
        (int)gateResidualQueues_.size() != numRounds ||
        (int)gateColorQueues_.size()    != numRounds) {
        throw std::runtime_error(
            "[PCATMPaper sv=1] inconsistent per-round queue sizes "
            "(expects valence/isPoly/color/residual aligned by round)");
    }

    const int BP_BITS = (int)mesh_->i_nbQuantBits + 1;
    auto sizes = pmsf::writePatchColorConnectivityBlob(
        buffer,
        kPCATMPaperBlobLayout,
        kPCATMPaperSchemeVersion,
        gateValenceQueues_,
        gateOtherValQueues_,
        gateIsPolyQueues_,
        gateColorQueues_,
        gateResidualQueues_,
        BP_BITS);

    const char* envBpv = std::getenv("PMSF_BPV_DIAG");
    if (envBpv && envBpv[0] == '1') {
        int overhead = sizes.blobBytes - sizes.payloadBytes();
        std::cerr << "[PCATMPaper bpv sv=1] valence=" << sizes.valence
                  << " isPolyBit=" << sizes.isPoly
                  << " colorBits=" << sizes.color
                  << " residual=" << sizes.residual
                  << " containerOverhead=" << overhead
                  << " totalBlob=" << sizes.blobBytes
                  << " rounds=" << numRounds << std::endl;
    }
    return sizes.blobBytes;
}

int PCATMPMSFSymbolCollectOperator::exportToBuffer(char* /*buffer*/,
                                                     bool /*enableQuantization*/,
                                                     bool /*enableCompress*/) {
    throw std::runtime_error(
        "[PCATMPaper] exportToBuffer (non-RC) is not implemented.");
}
