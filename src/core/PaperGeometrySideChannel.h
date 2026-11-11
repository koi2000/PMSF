#ifndef PMSF_CORE_PAPER_GEOMETRY_SIDE_CHANNEL_H
#define PMSF_CORE_PAPER_GEOMETRY_SIDE_CHANNEL_H

#include "core/PMSFSideChannel.h"

#include <cstdint>
#include <string>

namespace pmsf {

template <typename SideMap, typename QuantizedPoint>
inline void appendPaperQuantizedPointSideMap(SideMap& side,
                                             const std::string& prefix,
                                             const QuantizedPoint& q)
{
    appendPaperI32SideMap(side, prefix, "qx", static_cast<std::int32_t>(q[0]));
    appendPaperI32SideMap(side, prefix, "qy", static_cast<std::int32_t>(q[1]));
    appendPaperI32SideMap(side, prefix, "qz", static_cast<std::int32_t>(q[2]));
}

template <typename PaperRoundSymbols, typename QuantizedPoint>
inline void appendPaperQuantizedPointSideChannel(PaperRoundSymbols& delta,
                                                 const std::string& prefix,
                                                 const QuantizedPoint& q)
{
    appendPaperI32SideChannel(delta, prefix, "qx", static_cast<std::int32_t>(q.x()));
    appendPaperI32SideChannel(delta, prefix, "qy", static_cast<std::int32_t>(q.y()));
    appendPaperI32SideChannel(delta, prefix, "qz", static_cast<std::int32_t>(q.z()));
}



} // namespace pmsf

#endif // PMSF_CORE_PAPER_GEOMETRY_SIDE_CHANNEL_H
