#ifndef PMSF_CORE_DEFAULT_CODEC_PAPER_SIDE_CHANNEL_H
#define PMSF_CORE_DEFAULT_CODEC_PAPER_SIDE_CHANNEL_H

#include "core/PMSFBoundarySideChannel.h"
#include "core/PaperGeometrySideChannel.h"
#include "core/PMSFSideChannel.h"
#include "core/Symbol.h"

#include <cstddef>
#include <cstdint>

namespace pmsf {

template <typename RoundSymbols,
          typename FacetSymbols,
          typename EdgeSymbols,
          typename VertexSymbols,
          typename Points,
          typename VertexPoints>
inline void setDefaultCodecPaperRoundSymbols(RoundSymbols& round,
                                             const FacetSymbols& facetSymbols,
                                             const EdgeSymbols& edgeSymbols,
                                             const VertexSymbols& vertexSymbols,
                                             const Points& points,
                                             const VertexPoints& vertexPoints)
{
    round.facetSymbols = toPMSFSymbolCodeDeque(facetSymbols);
    round.edgeSymbols = toPMSFSymbolCodeDeque(edgeSymbols);
    round.vertexSymbols = toPMSFSymbolCodeDeque(vertexSymbols);
    round.points = points;
    round.vertexPoints = vertexPoints;
}

template <typename RoundSymbols,
          typename FacetQueues,
          typename EdgeQueues,
          typename VertexQueues,
          typename PointQueues,
          typename VertexPointQueues>
inline void setDefaultCodecPaperRoundSymbolsFromQueues(
    RoundSymbols& round,
    std::size_t roundIndex,
    const FacetQueues& facetQueues,
    const EdgeQueues& edgeQueues,
    const VertexQueues& vertexQueues,
    const PointQueues& pointQueues,
    const VertexPointQueues& vertexPointQueues)
{
    round.facetSymbols = toPMSFSymbolCodeDeque(facetQueues[roundIndex]);
    if (roundIndex < edgeQueues.size()) {
        round.edgeSymbols = toPMSFSymbolCodeDeque(edgeQueues[roundIndex]);
    }
    if (roundIndex < vertexQueues.size()) {
        round.vertexSymbols = toPMSFSymbolCodeDeque(vertexQueues[roundIndex]);
    }
    if (roundIndex < pointQueues.size()) {
        round.points = pointQueues[roundIndex];
    }
    if (roundIndex < vertexPointQueues.size()) {
        round.vertexPoints = vertexPointQueues[roundIndex];
    }
}

template <typename RoundSymbols,
          typename FacetSymbols,
          typename EdgeSymbols,
          typename Points>
inline void setDefaultCodecTopologyRoundSymbols(RoundSymbols& round,
                                                const FacetSymbols& facetSymbols,
                                                const EdgeSymbols& edgeSymbols,
                                                const Points& points)
{
    round.facetSymbols = toPMSFSymbolCodeDeque(facetSymbols);
    round.edgeSymbols = toPMSFSymbolCodeDeque(edgeSymbols);
    round.points = points;
}


template <typename RoundSymbols, typename QuantizedPoints>
inline void appendBasicRemovedPointSideChannel(RoundSymbols& round,
                                               const QuantizedPoints& points)
{
    for (const auto& q : points) {
        appendPaperQuantizedPointSideChannel(round, "basic_point", q);
    }
}


template <typename RoundSymbols, typename MeshPtr, typename Points>
inline void appendBasicVertexPointSideChannel(RoundSymbols& round,
                                              const MeshPtr& mesh,
                                              const Points& points)
{
    if (!mesh) return;
    for (const auto& p : points) {
        appendPaperQuantizedPointSideChannel(
            round, "basic_vertex_point", mesh->floatPosToInt(p));
    }
}


template <typename RoundSymbols>
inline void appendBasicStorageSideChannel(RoundSymbols& round,
                                          std::int32_t adjustmentEnabled)
{
    appendPaperI32SideChannel(round, "basic", "topology_stream", 0);
    appendPaperI32SideChannel(
        round, "basic", "adjustment_enabled", adjustmentEnabled);
    appendPaperI32SideChannel(round, "basic", "group_filter", 0);
}


template <typename RoundSymbols, typename QuantizedPoints>
inline void appendOptimizedRemovedPointSideChannel(
    RoundSymbols& round,
    const QuantizedPoints& points)
{
    for (const auto& q : points) {
        appendPaperQuantizedPointSideChannel(round, "optimized_point", q);
    }
}


template <typename RoundSymbols, typename MeshPtr, typename Points>
inline void appendOptimizedVertexPointSideChannel(
    RoundSymbols& round,
    const MeshPtr& mesh,
    const Points& points)
{
    if (!mesh) return;
    for (const auto& p : points) {
        appendPaperQuantizedPointSideChannel(
            round, "optimized_vertex_point", mesh->floatPosToInt(p));
    }
}


template <typename RoundSymbols>
inline void appendOptimizedStorageSideChannel(
    RoundSymbols& round,
    std::int32_t topologyStream,
    std::int32_t faceLocalEdgeBits,
    std::int32_t stableGateNeighbors,
    std::int32_t sortRemainingFaces,
    std::int32_t absoluteGeometry,
    std::int32_t adjustmentEnabled,
    std::int32_t edgeContextPresent,
    std::int32_t groupFilter)
{
    appendPaperI32SideChannel(round, "optimized", "topology_stream", topologyStream);
    appendPaperI32SideChannel(
        round, "optimized", "face_local_edge_bits", faceLocalEdgeBits);
    appendPaperI32SideChannel(
        round, "optimized", "stable_gate_neighbors", stableGateNeighbors);
    appendPaperI32SideChannel(
        round, "optimized", "sort_remaining_faces", sortRemainingFaces);
    appendPaperI32SideChannel(
        round, "optimized", "absolute_geometry", absoluteGeometry);
    appendPaperI32SideChannel(
        round, "optimized", "adjustment_enabled", adjustmentEnabled);
    appendPaperI32SideChannel(
        round, "optimized", "edge_context_present", edgeContextPresent);
    appendPaperI32SideChannel(round, "optimized", "group_filter", groupFilter);
}





}  // namespace pmsf

#endif  // PMSF_CORE_DEFAULT_CODEC_PAPER_SIDE_CHANNEL_H
