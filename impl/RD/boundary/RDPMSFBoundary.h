#ifndef PMSF_CORE_RD_PAPER_BOUNDARY_H
#define PMSF_CORE_RD_PAPER_BOUNDARY_H

#include "core/PMSFBoundarySideChannel.h"
#include "core/PaperGeometrySideChannel.h"
#include "core/PMSFSideChannel.h"
#include "core/Symbol.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂
#include "storage/PMSFSymbolStorageAdapter.h"

#include <cstdint>
#include <string>

namespace pmsf {

inline constexpr const char* kRDPMSFStorageScope = "rd";
inline constexpr const char* kRDPaperPhaseAStorageScope = "rd_phase_a";
inline constexpr const char* kRDPaperBlobLayoutSideName = "blob_layout";

inline bool rdPMSFBoundaryEnvOn(const char* name)
{
    return pmsfStorageEnvEnabled(name);
}

inline bool rdPaperExactRequired()
{
    return rdPMSFBoundaryEnvOn("PMSF_RD_REQUIRE_PAPER_EXACT") ||
           rdPMSFBoundaryEnvOn("PMSF_RDTAG_REQUIRE_PAPER_EXACT") ||
           rdPMSFBoundaryEnvOn("PMSF_RD_REQUIRE_CURVATURE_EXACT") ||
           rdPMSFBoundaryEnvOn("PMSF_RD_REQUIRE_KLT_EXACT") ||
           rdPMSFBoundaryEnvOn("PMSF_RD_REQUIRE_PGS_EXACT");
}

inline bool rdPrioritizedGateSelectionExactRequired()
{
    return rdPMSFBoundaryEnvOn("PMSF_RD_REQUIRE_PGS_EXACT") ||
           rdPMSFBoundaryEnvOn("PMSF_RD_REQUIRE_PAPER_EXACT");
}

inline bool rdCurvaturePredictionExactRequired()
{
    return rdPMSFBoundaryEnvOn("PMSF_RD_REQUIRE_CURVATURE_EXACT") ||
           rdPMSFBoundaryEnvOn("PMSF_RD_REQUIRE_PAPER_EXACT");
}

inline bool rdKLTExactRequired()
{
    return rdPMSFBoundaryEnvOn("PMSF_RD_REQUIRE_KLT_EXACT") ||
           rdPMSFBoundaryEnvOn("PMSF_RD_REQUIRE_PAPER_EXACT");
}

inline std::int32_t rdPaperExactRequiredI32()
{
    return rdPaperExactRequired() ? 1 : 0;
}

inline std::int32_t rdPrioritizedGateSelectionExactRequiredI32()
{
    return rdPrioritizedGateSelectionExactRequired() ? 1 : 0;
}

inline std::int32_t rdCurvaturePredictionExactRequiredI32()
{
    return rdCurvaturePredictionExactRequired() ? 1 : 0;
}

inline std::int32_t rdKLTExactRequiredI32()
{
    return rdKLTExactRequired() ? 1 : 0;
}

inline void appendRDPMSFBoundaryExactPairs(PaperRoundSymbols& delta,
                                            const std::string& prefix)
{
    appendPMSFBoundaryAbsentExactPair(delta,
                                       prefix,
                                       "paper",
                                       rdPaperExactRequiredI32());
    appendPMSFBoundaryAbsentExactPair(
        delta,
        prefix,
        "prioritized_gate_selection",
        rdPrioritizedGateSelectionExactRequiredI32());
    appendPMSFBoundaryAbsentExactPair(delta,
                                       prefix,
                                       "curvature_prediction",
                                       rdCurvaturePredictionExactRequiredI32());
    appendPMSFBoundaryAbsentExactPair(delta,
                                       prefix,
                                       "klt",
                                       rdKLTExactRequiredI32());
}


inline void appendRDPhaseAStorageSideChannel(PaperRoundSymbols& delta)
{
    appendPaperI32SideChannel(
        delta, kRDPaperPhaseAStorageScope, "valence_compatible", 1);
    appendPaperI32SideChannel(delta, kRDPMSFStorageScope, "true_gate_only", 0);
    appendPaperI32SideChannel(delta, kRDPMSFStorageScope, "frenet_residuals", 0);
    appendPaperI32SideChannel(
        delta, kRDPMSFStorageScope, "cleaning_decimating_split", 0);
    appendPaperI32SideChannel(
        delta, kRDPMSFStorageScope, kRDPaperBlobLayoutSideName, kPaperBlobLayoutRDGate);
    appendRDPMSFBoundaryExactPairs(delta, kRDPaperPhaseAStorageScope);
}



template <typename SideMap, typename Values>
inline void replaceRDTagVertexAdjustSymbolSideMap(SideMap& side,
                                                  const Values& values)
{
    replacePaperI32SideMap(side, "rd_tag", "vertex_adjust_symbol", values);
}

template <typename PaperRoundSymbols, typename Values>
inline void replaceRDTagVertexAdjustSymbol(PaperRoundSymbols& delta,
                                           const Values& values)
{
    replacePaperI32SideChannel(delta, "rd_tag", "vertex_adjust_symbol", values);
}

template <typename PaperRoundSymbols>
inline void appendRDTagResidualCapabilityBoundary(
    PaperRoundSymbols& delta,
    std::int32_t geometryBasis,
    std::int32_t frenetExact,
    std::int32_t adaptiveResidual,
    std::int32_t globalStreams,
    std::int32_t residualDropBits)
{
    appendPaperI32SideChannel(delta, "rd_tag", "geometry_basis", geometryBasis);
    appendPaperI32SideChannel(delta, "rd_tag", "frenet_exact", frenetExact);
    appendPaperI32SideChannel(delta, "rd_tag", "adaptive_residual", adaptiveResidual);
    appendPaperI32SideChannel(delta, "rd_tag", "global_streams", globalStreams);
    appendPaperI32SideChannel(delta, "rd_tag", "edge_stream_present", 0);
    appendPaperI32SideChannel(delta, "rd_tag", "valence_stream_present", 1);
    appendPaperI32SideChannel(delta, "rd_tag", "ispoly_stream_present", 1);
    appendPaperI32SideChannel(delta, "rd_tag", "residual_drop_bits", residualDropBits);
    appendRDPMSFBoundaryExactPairs(delta, "rd_tag");
}

template <typename PaperRoundSymbols, typename QuantizedPoint>
inline void appendRDTagVertexAdjustResidualSideChannel(PaperRoundSymbols& delta,
                                                       const QuantizedPoint& q)
{
    appendPaperCoordinateForRole(
        delta, PMSFSymbolRole::ShiftedVertexCoord,
        static_cast<double>(q[0]),
        static_cast<double>(q[1]),
        static_cast<double>(q[2]));
    appendPaperQuantizedPointSideChannel(
        delta, "rd_tag_vertex_adjust_residual", q);
}



template <typename PaperRoundSymbols>
inline void appendRDGateValenceCodeSymbol(PaperRoundSymbols& delta,
                                          const std::string& prefix,
                                          std::int32_t code)
{
    appendPaperScalarSymbolForRole(delta, PMSFSymbolRole::Vertex, code);
    appendPaperI32SideChannel(delta, prefix, "valence_code", code);
}

template <typename PaperRoundSymbols>
inline void appendRDGateIsPolySymbol(PaperRoundSymbols& delta, bool isPoly)
{
    appendPaperScalarSymbolForRole(
        delta, PMSFSymbolRole::Facet, isPoly ? 1 : 0);
}

template <typename PaperRoundSymbols>
inline void appendRDGateTriangulationEdgeSymbol(PaperRoundSymbols& delta,
                                                bool edge)
{
    appendPaperScalarSymbolForRole(
        delta, PMSFSymbolRole::TriangulationEdge, edge ? 1 : 0);
}


template <typename PaperRoundSymbols>
inline void appendRDGateOtherValenceSymbol(PaperRoundSymbols& delta,
                                           const std::string& prefix,
                                           std::int32_t valence)
{
    appendPaperI32SideChannel(delta, prefix, "other_valence", valence);
}

template <typename PaperRoundSymbols, typename QuantizedPoint>
inline void appendRDGateResidualMirror(PaperRoundSymbols& delta,
                                       const std::string& prefix,
                                       const QuantizedPoint& q)
{
    appendPaperCoordinateForRole(
        delta, PMSFSymbolRole::RemovedVertexCoord,
        static_cast<double>(q[0]),
        static_cast<double>(q[1]),
        static_cast<double>(q[2]));
    appendPaperQuantizedPointSideChannel(delta, prefix + "_residual", q);
}

} // namespace pmsf

#endif // PMSF_CORE_RD_PAPER_BOUNDARY_H
