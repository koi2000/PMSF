#ifndef PMSF_CORE_PCLTTM_PAPER_BOUNDARY_H
#define PMSF_CORE_PCLTTM_PAPER_BOUNDARY_H

#include "core/PMSFBoundaryComponent.h"
#include "core/PMSFBoundarySideChannel.h"
#include "core/PaperGeometrySideChannel.h"
#include "core/PMSFSideChannel.h"
#include "core/Symbol.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include <cstdint>
#include <string>

namespace pmsf {

namespace pclttm_boundary {

// R3/L1（PHASE63）：原先 15 个样板函数全是 `envOn(A) || envOn(B) || ...`
// 与 `? 1 : 0`。机制已收进 `core/PMSFBoundaryComponent.h`；此处只留 **env 名**。
// ⚠️ 每条链的短路顺序**逐条照抄**原式 ⇒ 产物逐字节不变。
const char* const kPaperExact[] = {
    "PMSF_PCLTTM_REQUIRE_PAPER_EXACT",
    "PMSF_PCLTTM_REQUIRE_TOPOLOGY_REPLAY_EXACT",
    "PMSF_PCLTTM_REQUIRE_GEOMETRY_REPLAY_EXACT",
};
const char* const kTopologyReplay[] = {
    "PMSF_PCLTTM_REQUIRE_TOPOLOGY_REPLAY_EXACT",
    "PMSF_PCLTTM_REQUIRE_PAPER_EXACT",
};
const char* const kGeometryReplay[] = {
    "PMSF_PCLTTM_REQUIRE_GEOMETRY_REPLAY_EXACT",
    "PMSF_PCLTTM_REQUIRE_PAPER_EXACT",
};
const char* const kFig9Exact[] = {"PMSF_PCLTTM_REQUIRE_FIG9_EXACT"};
const char* const kAnyPaperOrFig9[] = {
    "PMSF_PCLTTM_REQUIRE_PAPER_EXACT",
    "PMSF_PCLTTM_REQUIRE_TOPOLOGY_REPLAY_EXACT",
    "PMSF_PCLTTM_REQUIRE_GEOMETRY_REPLAY_EXACT",
    "PMSF_PCLTTM_REQUIRE_FIG9_EXACT",
};

constexpr BoundaryFlagSpec kPaperExactSpec{kPaperExact, 3};
constexpr BoundaryFlagSpec kTopologyReplaySpec{kTopologyReplay, 2};
constexpr BoundaryFlagSpec kGeometryReplaySpec{kGeometryReplay, 2};
constexpr BoundaryFlagSpec kFig9ExactSpec{kFig9Exact, 1};
constexpr BoundaryFlagSpec kAnyPaperOrFig9Spec{kAnyPaperOrFig9, 4};

}  // namespace pclttm_boundary

inline void appendPCLTTMPMSFBoundaryExactPairs(PaperRoundSymbols& delta,
                                                const std::string& prefix)
{
    appendPMSFBoundaryAbsentExactPair(delta,
                                       prefix,
                                       "paper",
                                       boundaryFlagI32(pclttm_boundary::kPaperExactSpec));
    appendPMSFBoundaryAbsentExactPair(
        delta,
        prefix,
        "topology_replay",
        boundaryFlagI32(pclttm_boundary::kTopologyReplaySpec));
    appendPMSFBoundaryAbsentExactPair(
        delta,
        prefix,
        "geometry_replay",
        boundaryFlagI32(pclttm_boundary::kGeometryReplaySpec));
}


inline void appendPCLTTMStrictReplaySideChannel(
    PaperRoundSymbols& delta,
    const std::string& prefix,
    std::int32_t geometryMode,
    std::int32_t writeRanks,
    std::int32_t writeRings,
    std::int32_t rankReplay,
    std::int32_t ringReplay,
    std::int32_t rdReplay,
    std::int32_t reverseVOnly,
    std::int32_t reverseRanks,
    std::int32_t packAsIs)
{
    appendPaperI32SideChannel(delta, prefix, "geometry_mode", geometryMode);
    appendPaperI32SideChannel(delta, prefix, "fig9_executable_model", 1);
    appendPaperI32SideChannel(delta, prefix, "fig9_exact_table", 0);
    appendPaperI32SideChannel(
        delta, prefix, "fig9_exact_required", boundaryFlagI32(pclttm_boundary::kFig9ExactSpec));
    appendPCLTTMPMSFBoundaryExactPairs(delta, prefix);
    appendPaperI32SideChannel(delta, prefix, "write_ranks", writeRanks);
    appendPaperI32SideChannel(delta, prefix, "write_rings", writeRings);
    appendPaperI32SideChannel(delta, prefix, "rank_replay", rankReplay);
    appendPaperI32SideChannel(delta, prefix, "ring_replay", ringReplay);
    appendPaperI32SideChannel(delta, prefix, "rd_replay", rdReplay);
    appendPaperI32SideChannel(delta, prefix, "reverse_v_only", reverseVOnly);
    appendPaperI32SideChannel(delta, prefix, "reverse_ranks", reverseRanks);
    appendPaperI32SideChannel(delta, prefix, "pack_as_is", packAsIs);
}


template <typename SideMap, typename Values>
inline void replacePCLTTMTagFusedFaceSymbolSideMap(SideMap& side,
                                                   const Values& values)
{
    replacePaperI32SideMap(side, "pclttm_tag", "fused_face_symbol", values);
}

template <typename PaperRoundSymbols>
inline void clearPCLTTMTagInheritedTriangulationEdgeSymbols(
    PaperRoundSymbols& delta)
{
    setPaperScalarSymbolsForRole(delta, PMSFSymbolRole::TriangulationEdge, PMSFSymbolCodeDeque{});
}

template <typename PaperRoundSymbols, typename Values>
inline void appendPCLTTMTagRoundSchemaSideChannel(
    PaperRoundSymbols& delta,
    const Values& fusedFaceSymbols,
    std::int32_t geometryBasis,
    std::int32_t frenetExact,
    std::int32_t tagTableTriangulation)
{
    appendPaperI32SideChannel(delta, "pclttm_tag", "geometry_basis", geometryBasis);
    appendPaperI32SideChannel(delta, "pclttm_tag", "frenet_exact", frenetExact);
    appendPaperI32SideChannel(
        delta, "pclttm_tag", "table_triangulation", tagTableTriangulation);
    appendPaperI32SideChannel(delta, "pclttm_tag", "edge_stream_present", 0);
    appendPaperI32SideChannel(delta, "pclttm_tag", "fused_face_stream_present", 1);
    appendPaperI32SideChannel(delta, "pclttm_tag", "evolution_table", 2);
    appendPaperI32SideChannel(delta, "pclttm_tag", "fig9_executable_model", 1);
    appendPaperI32SideChannel(delta, "pclttm_tag", "fig9_exact_table", 0);
    appendPaperI32SideChannel(
        delta, "pclttm_tag", "fig9_exact_required",
        boundaryFlagI32(pclttm_boundary::kFig9ExactSpec));
    appendPMSFBoundaryAbsentExactPair(
        delta,
        "pclttm_tag",
        "paper",
        boundaryFlagI32(pclttm_boundary::kAnyPaperOrFig9Spec));
    appendPMSFBoundaryAbsentExactPair(
        delta,
        "pclttm_tag",
        "topology_replay",
        boundaryFlagI32(pclttm_boundary::kTopologyReplaySpec));
    appendPMSFBoundaryAbsentExactPair(
        delta,
        "pclttm_tag",
        "geometry_replay",
        boundaryFlagI32(pclttm_boundary::kGeometryReplaySpec));
    replacePaperI32SideChannel(
        delta, "pclttm_tag", "fused_face_symbol", fusedFaceSymbols);
}



template <typename PaperRoundSymbols, typename QuantizedPoint>
inline void appendPCLTTMTagPredC2Symbol(PaperRoundSymbols& delta,
                                        const QuantizedPoint& q)
{
    appendPaperQuantizedPointSideChannel(delta, "pclttm_tag_pred_c2", q);
}


template <typename PaperRoundSymbols, typename QuantizedPoint>
inline void appendPCLTTMTagFrenetCorrectionSymbol(PaperRoundSymbols& delta,
                                                  const QuantizedPoint& q)
{
    appendPaperQuantizedPointSideChannel(delta, "pclttm_tag_frenet_correction", q);
}

} // namespace pmsf

#endif // PMSF_CORE_PCLTTM_PAPER_BOUNDARY_H
