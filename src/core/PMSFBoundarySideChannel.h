#ifndef PMSF_CORE_PAPER_BOUNDARY_SIDE_CHANNEL_H
#define PMSF_CORE_PAPER_BOUNDARY_SIDE_CHANNEL_H

#include "core/PMSFSideChannel.h"
#include "core/Symbol.h"

#include <cstdint>
#include <string>

namespace pmsf {

inline std::string paperBoundarySideName(const std::string& prefix,
                                         const std::string& suffix)
{
    return prefix.empty() ? suffix : prefix + "_" + suffix;
}

inline void appendPMSFBoundarySideI32(PaperRoundSymbols& delta,
                                       const std::string& prefix,
                                       const std::string& suffix,
                                       std::int32_t value)
{
    appendPaperI32SideChannel(delta, paperBoundarySideName(prefix, suffix), value);
}

template <typename SideMap>
inline void appendPMSFBoundarySideMapI32(SideMap& side,
                                          const std::string& prefix,
                                          const std::string& suffix,
                                          std::int32_t value)
{
    appendPaperI32SideMap(side, paperBoundarySideName(prefix, suffix), value);
}

inline void appendPMSFBoundaryExactPair(PaperRoundSymbols& delta,
                                         const std::string& prefix,
                                         const std::string& stem,
                                         std::int32_t exactValue,
                                         std::int32_t requiredValue)
{
    appendPMSFBoundarySideI32(delta, prefix, stem + "_exact", exactValue);
    appendPMSFBoundarySideI32(delta,
                               prefix,
                               stem + "_exact_required",
                               requiredValue);
}

template <typename SideMap>
inline void appendPMSFBoundarySideMapExactPair(SideMap& side,
                                                const std::string& prefix,
                                                const std::string& stem,
                                                std::int32_t exactValue,
                                                std::int32_t requiredValue)
{
    appendPMSFBoundarySideMapI32(side, prefix, stem + "_exact", exactValue);
    appendPMSFBoundarySideMapI32(side,
                                  prefix,
                                  stem + "_exact_required",
                                  requiredValue);
}

inline void appendPMSFBoundaryAbsentExactPair(PaperRoundSymbols& delta,
                                               const std::string& prefix,
                                               const std::string& stem,
                                               std::int32_t requiredValue)
{
    appendPMSFBoundaryExactPair(delta, prefix, stem, 0, requiredValue);
}

template <typename SideMap>
inline void appendPMSFBoundarySideMapAbsentExactPair(
    SideMap& side,
    const std::string& prefix,
    const std::string& stem,
    std::int32_t requiredValue)
{
    appendPMSFBoundarySideMapExactPair(side, prefix, stem, 0, requiredValue);
}

} // namespace pmsf

#endif // PMSF_CORE_PAPER_BOUNDARY_SIDE_CHANNEL_H
