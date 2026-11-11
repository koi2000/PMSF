// SPDX-License-Identifier: Apache-2.0
//
// PCLTTMTagCanonicalReassembler.h — rebuild a CODEC_016 sv=3
// ConnectivityBlobContainer from a decoded canonical PMSFSymbolStream.

#ifndef PMSF_DECOMPRESS_PCLTTM_TAG_CANONICAL_REASSEMBLER_H
#define PMSF_DECOMPRESS_PCLTTM_TAG_CANONICAL_REASSEMBLER_H

#include "compress/ConnectivityBlobContainer.h"
#include "common/PaperLayoutAliases.h"
#include "PCLTTM/compress/PCLTTMTagCanonicalCodec.h"
#include "core/Symbol.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace pmsf {

inline std::vector<std::uint8_t> reassemblePCLTTMTagV3BlobFromCanonical(
    const PMSFSymbolStream& stream, int quantBits) {
    const int numRounds = static_cast<int>(stream.size());
    if (numRounds == 0) {
        return {};
    }
    const int BP_BITS = quantBits + 1;

    ConnectivityBlobBuilder builder(
        pclttmTagConnectivityBlobSpec(numRounds));

    for (int r = 0; r < numRounds; ++r) {
        const PaperRoundSymbols& round = stream[r];

        std::deque<std::uint8_t> fusedSyms;
        fusedSyms.assign(round.facetSymbols.begin(),
                         round.facetSymbols.end());

        std::deque<std::uint8_t> valCodes;
        auto valIt = round.side.find("pclttm_tag/valence");
        if (valIt != round.side.end()) {
            for (const auto& cell : valIt->second) {
                std::visit(
                    [&](const auto& v) {
                        using T = std::decay_t<decltype(v)>;
                        if constexpr (std::is_integral_v<T>) {
                            valCodes.push_back(
                                static_cast<std::uint8_t>(v));
                        }
                    },
                    cell);
            }
        }

        std::deque<std::uint8_t> otherVals;
        auto otherIt = round.side.find("pclttm_tag/valence_other");
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

        std::deque<MCGAL::PointInt> residuals;
        residuals.assign(round.removedVertexResiduals.begin(),
                         round.removedVertexResiduals.end());

        auto valBytes = pclttm_tag::encodeValence5WithContext(
            valCodes, otherVals);
        auto fusedBytes = pclttm_tag::encodeFusedFaceSym4RC(fusedSyms);
        auto resBytes = pclttm_tag::encodeResidualBitplane(
            residuals, BP_BITS);

        builder.appendStreamBytes(
            r, kPaperBlobStreamPCLTTMTagValence, valBytes);
        builder.appendStreamBytes(
            r, kPaperBlobStreamPCLTTMTagFusedFaceSym, fusedBytes);
        builder.appendStreamBytes(
            r, kPaperBlobStreamPCLTTMTagResidual, resBytes);
    }

    return builder.finalize();
}

}  // namespace pmsf

#endif  // PMSF_DECOMPRESS_PCLTTM_TAG_CANONICAL_REASSEMBLER_H
