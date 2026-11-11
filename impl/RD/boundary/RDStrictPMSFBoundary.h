#ifndef PMSF_CORE_RD_STRICT_PAPER_BOUNDARY_H
#define PMSF_CORE_RD_STRICT_PAPER_BOUNDARY_H

#include "core/PMSFBoundarySideChannel.h"
#include "core/PMSFSideChannel.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include <cstdint>

namespace pmsf {

inline bool rdStrictPMSFBoundaryEnvOn(const char* name)
{
    return pmsfStorageEnvEnabled(name);
}

inline bool rdStrictPaperExactEnvRequired()
{
    return rdStrictPMSFBoundaryEnvOn("PMSF_RDSTRICT_REQUIRE_PAPER_EXACT");
}

inline bool rdStrictPaperInverseEnvRequired()
{
    return rdStrictPMSFBoundaryEnvOn("PMSF_RDSTRICT_REQUIRE_PAPER_INVERSE");
}

inline bool rdStrictGeometryReplayEnvRequired()
{
    return rdStrictPMSFBoundaryEnvOn("PMSF_RDSTRICT_REQUIRE_GEOMETRY_REPLAY_EXACT");
}

inline bool rdStrictPaperExactRequired()
{
    return rdStrictPaperExactEnvRequired() ||
           rdStrictPaperInverseEnvRequired() ||
           rdStrictGeometryReplayEnvRequired();
}

inline bool rdStrictPaperInverseRequired()
{
    return rdStrictPaperInverseEnvRequired() || rdStrictPaperExactEnvRequired();
}

inline bool rdStrictGeometryReplayRequired()
{
    return rdStrictGeometryReplayEnvRequired() || rdStrictPaperExactEnvRequired();
}

inline std::int32_t rdStrictPaperExactRequiredI32()
{
    return rdStrictPaperExactRequired() ? 1 : 0;
}

inline std::int32_t rdStrictPaperInverseRequiredI32()
{
    return rdStrictPaperInverseRequired() ? 1 : 0;
}

inline std::int32_t rdStrictGeometryReplayRequiredI32()
{
    return rdStrictGeometryReplayRequired() ? 1 : 0;
}

inline void appendRDStrictPMSFBoundarySideChannels(PaperRoundSymbols& delta)
{
    appendPMSFBoundarySideI32(delta,
                               "rd_strict",
                               "paper_inverse_transform",
                               0);
    appendPMSFBoundaryAbsentExactPair(delta,
                                       "rd_strict",
                                       "paper",
                                       rdStrictPaperExactRequiredI32());
    appendPMSFBoundarySideI32(delta,
                               "rd_strict",
                               "paper_inverse_required",
                               rdStrictPaperInverseRequiredI32());
    appendPMSFBoundarySideI32(delta,
                               "rd_strict",
                               "geometry_replay_exact",
                               0);
    appendPMSFBoundarySideI32(delta,
                               "rd_strict",
                               "geometry_replay_required",
                               rdStrictGeometryReplayRequiredI32());
}

inline void appendRDStrictStorageSideChannel(PaperRoundSymbols& delta,
                                             std::int32_t strictRDConquest,
                                             std::int32_t paperBoundaryGates,
                                             std::int32_t frenetResiduals,
                                             std::int32_t connectivityBytes,
                                             std::int32_t geometryBytes,
                                             std::int32_t sideInfoBytes,
                                             std::int32_t containerOverheadBytes,
                                             std::int32_t splitDecodeRounds)
{
    appendPaperI32SideChannel(
        delta, "rd_strict", "strict_rd_conquest", strictRDConquest);
    appendPaperI32SideChannel(
        delta, "rd_strict", "paper_boundary_gates", paperBoundaryGates);
    appendPaperI32SideChannel(
        delta, "rd_strict", "frenet_residuals", frenetResiduals);
    appendRDStrictPMSFBoundarySideChannels(delta);
    appendPaperI32SideChannel(
        delta, "rd_strict", "connectivity_bytes", connectivityBytes);
    appendPaperI32SideChannel(
        delta, "rd_strict", "geometry_bytes", geometryBytes);
    appendPaperI32SideChannel(
        delta, "rd_strict", "side_info_bytes", sideInfoBytes);
    appendPaperI32SideChannel(
        delta,
        "rd_strict",
        "container_overhead_bytes",
        containerOverheadBytes);
    appendPaperI32SideChannel(
        delta, "rd_strict", "split_decode_rounds", splitDecodeRounds);
}


} // namespace pmsf

#endif // PMSF_CORE_RD_STRICT_PAPER_BOUNDARY_H
