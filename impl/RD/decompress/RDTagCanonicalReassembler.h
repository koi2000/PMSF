// SPDX-License-Identifier: Apache-2.0
//
// RDTagCanonicalReassembler.h — rebuild an RD_TAG sv=1 ConnectivityBlobContainer
// from a decoded canonical PMSFSymbolStream.

#ifndef PMSF_DECOMPRESS_RD_TAG_CANONICAL_REASSEMBLER_H
#define PMSF_DECOMPRESS_RD_TAG_CANONICAL_REASSEMBLER_H

#include "compress/ConnectivityBlobContainer.h"
#include "common/RDTagCanonicalCodec.h"
#include "RD/compress/RDTagSymbolCollectOperator.h"
#include "core/Symbol.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂

#include <algorithm>
#include <cstdint>
#include <deque>
#include <span>
#include <string>
#include <vector>

namespace pmsf {

inline std::vector<std::uint8_t> reassembleRDTagV1BlobFromCanonical(
    const PMSFSymbolStream& stream, int quantBits) {
    const int numRounds = static_cast<int>(stream.size());
    if (numRounds == 0) {
        return {};
    }
    const int BP_BITS = quantBits + 1;

    ConnectivityBlobBuilder builder(
        rdTagConnectivityBlobSpec(numRounds,
                                  kRDTagSchemeVersionV1,
                                  3));

    for (int r = 0; r < numRounds; ++r) {
        const PaperRoundSymbols& round = stream[r];

        std::deque<std::uint8_t> valCodes;
        valCodes.assign(round.facetSymbols.begin(),
                        round.facetSymbols.end());

        std::deque<std::uint8_t> isPolyBits;
        isPolyBits.assign(round.triangulationEdgeSymbols.begin(),
                          round.triangulationEdgeSymbols.end());

        std::deque<MCGAL::PointInt> residuals;
        residuals.assign(round.removedVertexResiduals.begin(),
                         round.removedVertexResiduals.end());

        std::deque<std::uint8_t> otherVals;
        auto it = round.side.find("rd_tag/valence_other");
        if (it != round.side.end()) {
            for (const auto& cell : it->second) {
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

        auto valBytes = rd_tag::encodeValence5WithContext(valCodes, otherVals);
        auto isPolyBytes = rd_tag::encodeBitstream2RC(isPolyBits);
        auto resBytes = rd_tag::encodeResidualBitplane(residuals, BP_BITS);

        builder.appendStreamBytes(r, 0, valBytes);
        builder.appendStreamBytes(r, 1, isPolyBytes);
        builder.appendStreamBytes(r, 2, resBytes);
    }

    return builder.finalize();
}

}  // namespace pmsf

#endif  // PMSF_DECOMPRESS_RD_TAG_CANONICAL_REASSEMBLER_H
