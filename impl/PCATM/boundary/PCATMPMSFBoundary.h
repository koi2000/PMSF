#ifndef PMSF_CORE_PCATM_PAPER_BOUNDARY_H
#define PMSF_CORE_PCATM_PAPER_BOUNDARY_H

#include "core/PMSFBoundaryComponent.h"
#include "core/PMSFBoundarySideChannel.h"
#include "core/PaperGeometrySideChannel.h"
#include "core/PMSFSideChannel.h"
#include "core/Symbol.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include <cstdint>
#include <string>

namespace pmsf {

namespace pcatm_boundary {

// R3/L1（PHASE63）：原先 9 个样板函数（`...EnvOn` 转发 + 4 个 `...Required`
// + 4 个 `...I32`）全是 `envOn(A) || envOn(B)` 与 `? 1 : 0`。机制已收进
// `core/PMSFBoundaryComponent.h`；此处只留 **env 名参数**。
//
// ⚠️ 短路顺序逐条照抄原式（4 条链都是「本开关 || PAPER_EXACT」的排列）。
const char* const kPaperExact[] = {
    "PMSF_PCATM_REQUIRE_PAPER_EXACT",
    "PMSF_PCATM_REQUIRE_Z_SHAPE_EXACT",
};
const char* const kZShapeExact[] = {
    "PMSF_PCATM_REQUIRE_Z_SHAPE_EXACT",
    "PMSF_PCATM_REQUIRE_PAPER_EXACT",
};
const char* const kStrictPaperExact[] = {
    "PMSF_PCATM_STRICT_REQUIRE_PAPER_EXACT",
    "PMSF_PCATM_STRICT_REQUIRE_Z_SHAPE_EXACT",
};
const char* const kStrictZShapeExact[] = {
    "PMSF_PCATM_STRICT_REQUIRE_Z_SHAPE_EXACT",
    "PMSF_PCATM_STRICT_REQUIRE_PAPER_EXACT",
};

constexpr BoundaryFlagSpec kPaperExactSpec{kPaperExact, 2};
constexpr BoundaryFlagSpec kZShapeExactSpec{kZShapeExact, 2};
constexpr BoundaryFlagSpec kStrictPaperExactSpec{kStrictPaperExact, 2};
constexpr BoundaryFlagSpec kStrictZShapeExactSpec{kStrictZShapeExact, 2};

}  // namespace pcatm_boundary


inline void appendPCATMColorBoundarySideChannels(
    PaperRoundSymbols& delta,
    const std::string& prefix,
    std::int32_t topologyMode,
    std::int32_t colorStreamPresent,
    std::int32_t edgeStreamPresent,
    std::int32_t strictPaperReconstruct)
{
    appendPaperI32SideChannel(delta, prefix, "topology_mode", topologyMode);
    appendPaperI32SideChannel(
        delta, prefix, "color_stream_present", colorStreamPresent);
    appendPaperI32SideChannel(
        delta, prefix, "edge_stream_present", edgeStreamPresent);
    appendPaperI32SideChannel(
        delta, prefix, "strict_paper_reconstruct", strictPaperReconstruct);
    appendPMSFBoundaryAbsentExactPair(
        delta, prefix, "paper", boundaryFlagI32(pcatm_boundary::kPaperExactSpec));
    appendPMSFBoundaryAbsentExactPair(
        delta, prefix, "z_shape", boundaryFlagI32(pcatm_boundary::kZShapeExactSpec));
}

template <typename PaperRoundSymbols>
inline void appendPCATMColorLayoutRequestSideChannel(
    PaperRoundSymbols& delta,
    const std::string& prefix,
    std::int32_t paperLayoutRequested,
    std::int32_t strictPaperReconstructUnverified)
{
    appendPaperI32SideChannel(
        delta, prefix, "paper_layout_requested", paperLayoutRequested);
    appendPaperI32SideChannel(
        delta,
        prefix,
        "strict_paper_reconstruct_unverified",
        strictPaperReconstructUnverified);
}


template <typename PaperRoundSymbols>
inline void appendPCATMZColorPaperLayoutSideChannel(PaperRoundSymbols& delta,
                                                    const std::string& prefix,
                                                    bool strictPaper)
{
    setPaperScalarSymbolsForRole(delta, PMSFSymbolRole::TriangulationEdge, PMSFSymbolCodeDeque{});
    const std::int32_t strict = strictPaper ? 1 : 0;
    appendPCATMColorBoundarySideChannels(delta, prefix, strict, 1, 0, strict);
    appendPCATMColorLayoutRequestSideChannel(delta, prefix, strict, strict);
}



template <typename PaperRoundSymbols>
inline void appendPCATMPureBoundary(PaperRoundSymbols& delta)
{
    setPaperScalarSymbolsForRole(delta, PMSFSymbolRole::TriangulationEdge, PMSFSymbolCodeDeque{});
    appendPaperI32SideChannel(delta, "pcatm_pure", "topology_mode", 2);
    appendPaperI32SideChannel(delta, "pcatm_pure", "color_stream_present", 0);
    appendPaperI32SideChannel(delta, "pcatm_pure", "edge_stream_present", 0);
    appendPaperI32SideChannel(delta, "pcatm_pure", "strict_paper_reconstruct", 1);
    appendPMSFBoundaryAbsentExactPair(
        delta, "pcatm_pure", "paper", boundaryFlagI32(pcatm_boundary::kPaperExactSpec));
    appendPMSFBoundaryAbsentExactPair(
        delta, "pcatm_pure", "z_shape", boundaryFlagI32(pcatm_boundary::kZShapeExactSpec));
    appendPaperI32SideChannel(
        delta, "pcatm_pure", "strict_paper_reconstruct_unverified", 1);
}


template <typename PaperRoundSymbols, typename Values>
inline void replacePCATMColorBitSymbols(PaperRoundSymbols& delta,
                                        const std::string& prefix,
                                        const Values& values)
{
    replacePaperI32SideChannel(delta, prefix, "color_bit", values);
}


template <typename PaperRoundSymbols, typename Values>
inline void replacePCATMStrictColorSymbols(PaperRoundSymbols& delta,
                                           const Values& values)
{
    replacePaperI32SideChannel(delta, "pcatm_strict", "color_symbol", values);
}


template <typename PaperRoundSymbols, typename Values>
inline void replacePCATMStrictMode2Symbols(PaperRoundSymbols& delta,
                                           const Values& values)
{
    replacePaperI32SideChannel(delta, "pcatm_strict", "mode2_symbol", values);
}

template <typename PaperRoundSymbols>
inline void appendPCATMStrictCapabilitySideChannel(
    PaperRoundSymbols& delta,
    std::int32_t schemeVersion,
    std::int32_t colorOnlyPatchRecovery,
    std::int32_t topologySideStreams,
    std::int32_t topologySideBytes)
{
    appendPaperI32SideChannel(delta, "pcatm_strict", "full_patch_recovery", 0);
    appendPaperI32SideChannel(delta, "pcatm_strict", "stream_validation", 1);
    appendPaperI32SideChannel(delta, "pcatm_strict", "scheme_version", schemeVersion);
    appendPaperI32SideChannel(
        delta,
        "pcatm_strict",
        "color_only_patch_recovery",
        colorOnlyPatchRecovery);
    appendPaperI32SideChannel(
        delta, "pcatm_strict", "topology_side_streams", topologySideStreams);
    appendPaperI32SideChannel(
        delta, "pcatm_strict", "topology_side_bytes", topologySideBytes);
    appendPMSFBoundaryAbsentExactPair(
        delta, "pcatm_strict", "z_shape", boundaryFlagI32(pcatm_boundary::kStrictZShapeExactSpec));
    appendPMSFBoundaryAbsentExactPair(
        delta, "pcatm_strict", "paper", boundaryFlagI32(pcatm_boundary::kStrictPaperExactSpec));
}



template <typename PaperRoundSymbols>
inline void appendPCATMStrictMode2Recoverable(PaperRoundSymbols& delta,
                                              std::int32_t mode2Recoverable)
{
    appendPaperI32SideChannel(
        delta, "pcatm_strict", "mode2_recoverable", mode2Recoverable);
}


template <typename PaperRoundSymbols>
inline void appendPCATMStrictStats(
    PaperRoundSymbols& delta,
    std::int32_t mode2Rounds,
    std::int32_t mode4Rounds,
    std::int32_t connectivityBytes,
    std::int32_t geometryBytes,
    std::int32_t containerOverheadBytes,
    std::int32_t rawColorBits,
    std::int32_t removedVertices,
    std::int32_t degree3,
    std::int32_t degree4,
    std::int32_t degree5,
    std::int32_t degree6,
    std::int32_t degreeOther)
{
    appendPaperI32SideChannel(delta, "pcatm_strict", "mode2_rounds", mode2Rounds);
    appendPaperI32SideChannel(delta, "pcatm_strict", "mode4_rounds", mode4Rounds);
    appendPaperI32SideChannel(
        delta, "pcatm_strict", "connectivity_bytes", connectivityBytes);
    appendPaperI32SideChannel(delta, "pcatm_strict", "geometry_bytes", geometryBytes);
    appendPaperI32SideChannel(
        delta,
        "pcatm_strict",
        "container_overhead_bytes",
        containerOverheadBytes);
    appendPaperI32SideChannel(delta, "pcatm_strict", "raw_color_bits", rawColorBits);
    appendPaperI32SideChannel(
        delta, "pcatm_strict", "removed_vertices", removedVertices);
    appendPaperI32SideChannel(delta, "pcatm_strict", "degree3", degree3);
    appendPaperI32SideChannel(delta, "pcatm_strict", "degree4", degree4);
    appendPaperI32SideChannel(delta, "pcatm_strict", "degree5", degree5);
    appendPaperI32SideChannel(delta, "pcatm_strict", "degree6", degree6);
    appendPaperI32SideChannel(delta, "pcatm_strict", "degree_other", degreeOther);
}

template <typename PaperRoundSymbols, typename QuantizedPoint>
inline void appendPCATMStrictResidualSideChannel(PaperRoundSymbols& delta,
                                                 const QuantizedPoint& q)
{
    appendPaperCoordinateForRole(
        delta, PMSFSymbolRole::RemovedVertexCoord,
        static_cast<double>(q[0]),
        static_cast<double>(q[1]),
        static_cast<double>(q[2]));
    appendPaperQuantizedPointSideChannel(delta, "pcatm_strict_residual", q);
}



template <typename PaperRoundSymbols, typename QuantizedPoint>
inline void appendPCATMStrictMode2Residual(PaperRoundSymbols& delta,
                                           const QuantizedPoint& q)
{
    appendPaperQuantizedPointSideChannel(
        delta, "pcatm_strict_mode2_residual", q);
}

} // namespace pmsf

#endif // PMSF_CORE_PCATM_PAPER_BOUNDARY_H
