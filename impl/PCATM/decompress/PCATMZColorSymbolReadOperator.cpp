// SPDX-License-Identifier: Apache-2.0
//
// PCATMZColorSymbolReadOperator.cpp — see header for design contract.
//
// 与 PCLTTMTagSymbolReadOperator::importFromRangeCodedBuffer 的差异（最小化）：
//   - 期望 hdr.blob_layout = 17, hdr.scheme_version = 1, hdr.num_streams = 4。
//   - stream 1 解 2-元 isPolyBit（与 RDTag 同样）。
//   - stream 2 解 2-元 colorBits → rcDecodedColors_（留位用）。
//   - stream 3 解 residual bit-plane。
//   - **不填** rcDecodedVertexTags_。

#include "PCATM/decompress/PCATMZColorSymbolReadOperator.h"
#include "common/PaperLayoutAliases.h"

#include "compress/PatchColorConnectivityCodec.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂

#include <cstdlib>
#include <iostream>
#include <span>
#include <stdexcept>
#include <utility>

void PCATMZColorSymbolReadOperator::importFromRangeCodedBuffer(
    char* buffer,
    size_t& dataOffset,
    unsigned nbQuantBits) {

    constexpr size_t kArbitraryLargeView = static_cast<size_t>(1) << 32;
    std::span<const uint8_t> view(reinterpret_cast<const uint8_t*>(buffer + dataOffset),
                                   kArbitraryLargeView);

    rcUsingResidualLocal_ = header_.isEnablePrediction() && header_.isEnableQuantization();
    if (!rcUsingResidualLocal_) {
        throw std::runtime_error(
            "[PCATMZColor decoder] expects enablePrediction && enableQuantization");
    }
    const int BP_BITS = static_cast<int>(nbQuantBits) + 1;

    auto decoded = pmsf::readPatchColorConnectivityBlob(
        view,
        pmsf::kPaperBlobLayoutPCATMZColor,
        pmsf::kPaperBlobSchemeVersionPCATMZColor,
        BP_BITS,
        "PCATMZColor");
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
        std::cerr << "[PCATMZColor decoder sv=1] rounds=" << decoded.numRounds
                  << " totalIsPoly=" << rcDecodedIsPoly_.size()
                  << " totalValCodes=" << rcDecodedValCodes_.size()
                  << " totalColors=" << rcDecodedColors_.size()
                  << " totalResiduals=" << rcDecodedResiduals_.size()
                  << " blobSize=" << decoded.blobSize << std::endl;
    }
}
