#ifndef PMSF_CORE_VALENCE_PAPER_SIDE_CHANNEL_H
#define PMSF_CORE_VALENCE_PAPER_SIDE_CHANNEL_H

#include "core/PaperGeometrySideChannel.h"
#include "core/PMSFSideChannel.h"
#include "core/Symbol.h"

#include <cstddef>
#include <cstdint>
#include <string>

namespace pmsf {


template <typename RoundSymbols>
inline void appendValenceCodeByteSideChannel(RoundSymbols& round,
                                             std::int8_t code)
{
    appendPaperI8SideChannel(round, "valence", "code", code);
}

template <typename RoundSymbols, typename EdgeSymbols>
inline void appendValenceTriangulationEdgeSymbols(RoundSymbols& round,
                                                  const EdgeSymbols& edgeSymbols)
{
    for (const auto& value : edgeSymbols) {
        appendRoundTriangulationEdgeSymbol(
            round, static_cast<std::int32_t>(value != 0));
    }
}

template <typename RoundSymbols, typename VertexSymbols>
inline void appendValenceVertexAdjustmentSymbols(RoundSymbols& round,
                                                 const VertexSymbols& vertexSymbols)
{
    for (const auto& value : vertexSymbols) {
        appendRoundVertexSymbol(
            round, static_cast<std::int32_t>(value != 0));
    }
}

template <typename RoundSymbols, typename Points, typename VertexPoints>
inline void setValenceRoundGeometrySymbols(RoundSymbols& round,
                                           const Points& points,
                                           const VertexPoints& vertexPoints)
{
    round.points = points;
    round.vertexPoints = vertexPoints;
}


template <typename RoundSymbols, typename QuantizedPoints>
inline void appendValenceRemovedPointSideChannel(RoundSymbols& round,
                                                 const QuantizedPoints& points)
{
    for (const auto& q : points) {
        appendPaperQuantizedPointSideChannel(round, "valence_point", q);
    }
}


template <typename RoundSymbols, typename MeshPtr, typename Points>
inline void appendValenceVertexPointSideChannel(RoundSymbols& round,
                                                const MeshPtr& mesh,
                                                const Points& points)
{
    if (!mesh) return;
    for (const auto& p : points) {
        appendPaperQuantizedPointSideChannel(
            round, "valence_vertex_point", mesh->floatPosToInt(p));
    }
}

template <typename SideMap, typename Values>
inline void replaceValenceCodeSideMap(SideMap& side,
                                      const Values& values)
{
    replacePaperI32SideMap(side, "valence", "code", values);
}

template <typename PaperRoundSymbols, typename Values>
inline void replaceValenceCodeSideChannel(PaperRoundSymbols& delta,
                                          const Values& values)
{
    replacePaperI32SideChannel(delta, "valence", "code", values);
}

template <typename PaperRoundSymbols, typename Values>
inline void replaceValenceVertexSymbolsFromCodes(PaperRoundSymbols& delta,
                                                 const Values& values)
{
    auto* target = paperScalarSymbolDequeForRole(delta, PMSFSymbolRole::Vertex);
    if (target == nullptr) {
        return;
    }
    target->clear();
    for (const auto& value : values) {
        target->push_back(static_cast<std::int32_t>(value));
    }
}

template <typename SideMap, typename Values>
inline void replaceValencePatch6CodeSideMap(SideMap& side,
                                            const std::string& prefix,
                                            const Values& values)
{
    replacePaperI32SideMap(side, prefix, "valence_patch6_code", values);
}

template <typename PaperRoundSymbols, typename Values>
inline void replaceValencePatch6CodeSideChannel(PaperRoundSymbols& delta,
                                                const std::string& prefix,
                                                const Values& values)
{
    replacePaperI32SideChannel(delta, prefix, "valence_patch6_code", values);
}


template <typename PaperRoundSymbols>
inline void appendValenceVertexAdjustmentFlagSymbol(PaperRoundSymbols& delta,
                                                    std::int32_t flag)
{
    appendPaperI32SideChannel(delta, "valence", "vertex_adjustment_flag", flag);
}

}  // namespace pmsf

#endif  // PMSF_CORE_VALENCE_PAPER_SIDE_CHANNEL_H
