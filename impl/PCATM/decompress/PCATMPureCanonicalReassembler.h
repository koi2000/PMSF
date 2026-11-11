// SPDX-License-Identifier: Apache-2.0
//
// PCATMPureCanonicalReassembler.h — rebuild a CODEC_021 sv=1/2
// ConnectivityBlobContainer from a decoded canonical PMSFSymbolStream.

#ifndef PMSF_DECOMPRESS_PCATM_PURE_CANONICAL_REASSEMBLER_H
#define PMSF_DECOMPRESS_PCATM_PURE_CANONICAL_REASSEMBLER_H

#include "compress/ConnectivityBlobContainer.h"
#include "common/PaperLayoutAliases.h"
#include "compress/PatchColorConnectivityCodec.h"
#include "core/Symbol.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace pmsf {

inline std::vector<std::uint8_t> reassemblePCATMPureBlobFromCanonical(
    const PMSFSymbolStream& stream,
    std::int32_t schemeVersion,
    int quantBits) {
    const int numRounds = static_cast<int>(stream.size());
    if (numRounds == 0) {
        return {};
    }
    const int BP_BITS = quantBits + 1;

    ConnectivityBlobBuilder builder(
        pcatmPureConnectivityBlobSpec(numRounds, schemeVersion));

    for (int r = 0; r < numRounds; ++r) {
        const PaperRoundSymbols& round = stream[r];

        std::deque<std::uint8_t> valCodes;
        valCodes.assign(round.facetSymbols.begin(),
                        round.facetSymbols.end());

        std::deque<std::uint8_t> otherVals;
        auto otherIt = round.side.find("pcatm_pure/valence_other");
        if (otherIt != round.side.end()) {
            for (const auto& cell : otherIt->second) {
                std::visit(
                    [&](const auto& v) {
                        using T = std::decay_t<decltype(v)>;
                        if constexpr (std::is_integral_v<T>) {
                            otherVals.push_back(
                                static_cast<std::uint8_t>(v));
                        }
                    },
                    cell);
            }
        }

        std::deque<std::uint8_t> isPolyBits;
        isPolyBits.assign(round.triangulationEdgeSymbols.begin(),
                          round.triangulationEdgeSymbols.end());

        std::deque<MCGAL::PointInt> residuals;
        residuals.assign(round.removedVertexResiduals.begin(),
                         round.removedVertexResiduals.end());

        auto valBytes = encodeValence5WithContext(valCodes, otherVals);
        auto isPolyBytes = encodeAdaptiveBits(isPolyBits);
        auto resBytes = encodeResidualBitplane(residuals, BP_BITS);

        builder.appendStreamBytes(
            r, kPaperBlobStreamPCATMPureValence, valBytes);
        builder.appendStreamBytes(
            r, kPaperBlobStreamPCATMPureIsPoly, isPolyBytes);
        builder.appendStreamBytes(
            r, kPaperBlobStreamPCATMPureResidual, resBytes);
    }

    return builder.finalize();
}

}  // namespace pmsf

#endif  // PMSF_DECOMPRESS_PCATM_PURE_CANONICAL_REASSEMBLER_H
