#ifndef PMSF_CORE_PAPER_EDGE_FLIP_SIDE_CHANNEL_H
#define PMSF_CORE_PAPER_EDGE_FLIP_SIDE_CHANNEL_H

#include "core/PMSFSideChannel.h"

#include <cstdint>

namespace pmsf {


template <typename PaperRoundSymbols>
inline void appendPaperEdgeFlipCandidateBitCount(PaperRoundSymbols& delta,
                                                 std::int32_t count)
{
    appendPaperI32SideChannel(delta, "edge_flip", "candidate_bit_count", count);
}


template <typename PaperRoundSymbols>
inline void appendPaperEdgeFlipRoundCount(PaperRoundSymbols& delta,
                                          std::int32_t count)
{
    appendPaperI32SideChannel(delta, "edge_flip", "round_count", count);
}


template <typename PaperRoundSymbols, typename CoordSymbol>
inline void appendPaperEdgeFlipCoordSymbol(PaperRoundSymbols& delta,
                                           const CoordSymbol& sym)
{
    for (int k = 0; k < 3; ++k) {
        appendPaperI32SideChannel(
            delta, "edge_flip", "a_q", static_cast<std::int32_t>(sym.a[k]));
        appendPaperI32SideChannel(
            delta, "edge_flip", "b_q", static_cast<std::int32_t>(sym.b[k]));
        appendPaperI32SideChannel(
            delta, "edge_flip", "oa_q", static_cast<std::int32_t>(sym.oa[k]));
        appendPaperI32SideChannel(
            delta, "edge_flip", "ob_q", static_cast<std::int32_t>(sym.ob[k]));
    }
}

template <typename SideMap>
inline void appendPaperEdgeFlipEndpointSideMap(SideMap& side,
                                               std::int32_t endpointA,
                                               std::int32_t endpointB)
{
    appendPaperI32SideMap(side, "edge_flip", "endpoint_a", endpointA);
    appendPaperI32SideMap(side, "edge_flip", "endpoint_b", endpointB);
}


template <typename PaperRoundSymbols, typename EndpointPair>
inline void appendPaperEdgeFlipEndpointPair(PaperRoundSymbols& delta,
                                            const EndpointPair& sym)
{
    appendPaperI32SideChannel(
        delta, "edge_flip", "endpoint_a", static_cast<std::int32_t>(sym.first));
    appendPaperI32SideChannel(
        delta, "edge_flip", "endpoint_b", static_cast<std::int32_t>(sym.second));
}

} // namespace pmsf

#endif // PMSF_CORE_PAPER_EDGE_FLIP_SIDE_CHANNEL_H
