// SPDX-License-Identifier: Apache-2.0
//
// PatchColorCanonicalReassembler.h — rebuild a patch/color ConnectivityBlobContainer
// (CODEC_017 / CODEC_020 sv=1) from a decoded canonical PMSFSymbolStream.

#ifndef PMSF_DECOMPRESS_PATCH_COLOR_CANONICAL_REASSEMBLER_H
#define PMSF_DECOMPRESS_PATCH_COLOR_CANONICAL_REASSEMBLER_H

#include "compress/ConnectivityBlobContainer.h"
#include "compress/PatchColorConnectivityCodec.h"
#include "core/Symbol.h"
#include "storage/PaperBlobLayoutRegistry.h"

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace pmsf {

inline std::vector<std::uint8_t> reassemblePatchColorBlobFromCanonical(
    const PMSFSymbolStream& stream,
    std::int32_t blobLayout,
    std::int32_t schemeVersion,
    int quantBits) {
    const int numRounds = static_cast<int>(stream.size());
    if (numRounds == 0) {
        return {};
    }
    const int BP_BITS = quantBits + 1;

    ConnectivityBlobBuilder builder(
        patchColorConnectivityBlobSpec(blobLayout, schemeVersion, numRounds));

    for (int r = 0; r < numRounds; ++r) {
        const PaperRoundSymbols& round = stream[r];

        std::deque<std::uint8_t> valCodes;
        valCodes.assign(round.facetSymbols.begin(),
                        round.facetSymbols.end());

        std::deque<std::uint8_t> otherVals;
        auto otherIt = round.side.find("patch_color/valence_other");
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

        std::deque<std::uint8_t> colorBits;
        colorBits.assign(round.flipEdgeSymbols.begin(),
                         round.flipEdgeSymbols.end());

        std::deque<MCGAL::PointInt> residuals;
        residuals.assign(round.removedVertexResiduals.begin(),
                         round.removedVertexResiduals.end());

        auto valBytes = encodeValence5WithContext(valCodes, otherVals);
        auto isPolyBytes = encodeAdaptiveBits(isPolyBits);
        auto colorBytes = encodeAdaptiveBits(colorBits);
        auto resBytes = encodeResidualBitplane(residuals, BP_BITS);

        builder.appendStreamBytes(
            r, kPaperBlobStreamPatchColorValence, valBytes);
        builder.appendStreamBytes(
            r, kPaperBlobStreamPatchColorIsPoly, isPolyBytes);
        builder.appendStreamBytes(
            r, kPaperBlobStreamPatchColorColor, colorBytes);
        builder.appendStreamBytes(
            r, kPaperBlobStreamPatchColorResidual, resBytes);
    }

    return builder.finalize();
}

}  // namespace pmsf

#endif  // PMSF_DECOMPRESS_PATCH_COLOR_CANONICAL_REASSEMBLER_H
