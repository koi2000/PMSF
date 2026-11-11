// SPDX-License-Identifier: Apache-2.0
//
// PCATMColorCanonicalReassembler.h — rebuild a CODEC_014 sv=3
// ConnectivityBlobContainer from a decoded canonical PMSFSymbolStream.

#ifndef PMSF_DECOMPRESS_PCATM_COLOR_CANONICAL_REASSEMBLER_H
#define PMSF_DECOMPRESS_PCATM_COLOR_CANONICAL_REASSEMBLER_H

#include "compress/ConnectivityBlobContainer.h"
#include "common/PaperLayoutAliases.h"
#include "common/RDTagCanonicalCodec.h"
#include "core/Symbol.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace pmsf {

inline std::vector<std::uint8_t> reassemblePCATMColorV3BlobFromCanonical(
    const PMSFSymbolStream& stream, int quantBits) {
    const int numRounds = static_cast<int>(stream.size());
    if (numRounds == 0) {
        return {};
    }
    const int BP_BITS = quantBits + 1;

    ConnectivityBlobBuilder builder(
        pcatmColorConnectivityBlobSpec(numRounds));

    for (int r = 0; r < numRounds; ++r) {
        const PaperRoundSymbols& round = stream[r];

        std::deque<std::uint8_t> valCodes;
        valCodes.assign(round.facetSymbols.begin(),
                        round.facetSymbols.end());

        std::deque<std::uint8_t> otherVals;
        auto otherIt = round.side.find("pcatm_color/valence_other");
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

        std::deque<std::uint8_t> edgeSyms;
        edgeSyms.assign(round.vertexSymbols.begin(),
                        round.vertexSymbols.end());

        std::deque<std::uint8_t> colorBits;
        colorBits.assign(round.flipEdgeSymbols.begin(),
                         round.flipEdgeSymbols.end());

        std::deque<MCGAL::PointInt> residuals;
        residuals.assign(round.removedVertexResiduals.begin(),
                         round.removedVertexResiduals.end());

        auto valBytes = rd_tag::encodeValence5WithContext(valCodes, otherVals);
        auto isPolyBytes = rd_tag::encodeBitstream2RC(isPolyBits);
        auto edgeBytes = rd_tag::encodeBitstream2RC(edgeSyms);
        auto colorBytes = rd_tag::encodeBitstream2RC(colorBits);
        auto resBytes = rd_tag::encodeResidualBitplane(residuals, BP_BITS);

        builder.appendStreamBytes(
            r, kPaperBlobStreamPCATMColorValence, valBytes);
        builder.appendStreamBytes(
            r, kPaperBlobStreamPCATMColorIsPoly, isPolyBytes);
        builder.appendStreamBytes(
            r, kPaperBlobStreamPCATMColorEdgeSym, edgeBytes);
        builder.appendStreamBytes(
            r, kPaperBlobStreamPCATMColorColor, colorBytes);
        builder.appendStreamBytes(
            r, kPaperBlobStreamPCATMColorResidual, resBytes);
    }

    return builder.finalize();
}

}  // namespace pmsf

#endif  // PMSF_DECOMPRESS_PCATM_COLOR_CANONICAL_REASSEMBLER_H
