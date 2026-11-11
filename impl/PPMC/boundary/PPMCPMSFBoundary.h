#ifndef PMSF_CORE_PPMC_PAPER_BOUNDARY_H
#define PMSF_CORE_PPMC_PAPER_BOUNDARY_H

#include "core/PMSFBoundaryComponent.h"
#include "core/PMSFBoundarySideChannel.h"
#include "core/PaperGeometrySideChannel.h"
#include "core/PMSFSideChannel.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include <cstdint>

namespace pmsf {

namespace ppmc_boundary {

// R3/L1（PHASE63）：原先 4 个样板函数（2 个 `...Required` + 2 个 `...I32`）
// 全是 `envOn(A) || envOn(B)` 与 `? 1 : 0`。机制已收进
// `core/PMSFBoundaryComponent.h`；此处只留 **env 名参数**，短路顺序照抄原式。
const char* const kPaperExact[] = {
    "PMSF_PPMC_REQUIRE_PMSF_EXACT",
    "PMSF_PPMC_REQUIRE_CURVATURE_EXACT",
};
const char* const kCurvatureExact[] = {
    "PMSF_PPMC_REQUIRE_CURVATURE_EXACT",
    "PMSF_PPMC_REQUIRE_PMSF_EXACT",
};

constexpr BoundaryFlagSpec kPaperExactSpec{kPaperExact, 2};
constexpr BoundaryFlagSpec kCurvatureExactSpec{kCurvatureExact, 2};

}  // namespace ppmc_boundary

inline void appendPPMCPaperContext(PaperRoundSymbols& delta,
                                   std::int32_t segmented,
                                   std::int32_t connectivityPredictionFaces,
                                   std::int32_t connectivityPredictionEdges,
                                   std::int32_t triangleMeshPredictionFaces,
                                   std::int32_t curvaturePrediction,
                                   std::int32_t curvatureSymmetric,
                                   std::int32_t liftingScheme,
                                   std::int32_t liftingDecoderUnlift,
                                   std::int32_t adaptiveQuantization,
                                   std::int32_t groupFilter)
{
    if (segmented >= 0) {
        appendPaperI32SideChannel(delta, "ppmc", "segmented", segmented);
    }
    appendPaperI32SideChannel(
        delta, "ppmc", "connectivity_prediction_faces",
        connectivityPredictionFaces);
    appendPaperI32SideChannel(
        delta, "ppmc", "connectivity_prediction_edges",
        connectivityPredictionEdges);
    appendPaperI32SideChannel(
        delta, "ppmc", "triangle_mesh_prediction_faces",
        triangleMeshPredictionFaces);
    appendPaperI32SideChannel(
        delta, "ppmc", "curvature_prediction", curvaturePrediction);
    appendPaperI32SideChannel(
        delta, "ppmc", "curvature_symmetric", curvatureSymmetric);
    appendPMSFBoundaryAbsentExactPair(
        delta, "ppmc", "paper", boundaryFlagI32(ppmc_boundary::kPaperExactSpec));
    appendPMSFBoundaryAbsentExactPair(
        delta, "ppmc", "curvature_prediction",
        boundaryFlagI32(ppmc_boundary::kCurvatureExactSpec));
    appendPaperI32SideChannel(delta, "ppmc", "lifting_scheme", liftingScheme);
    appendPaperI32SideChannel(
        delta, "ppmc", "lifting_decoder_unlift", liftingDecoderUnlift);
    appendPaperI32SideChannel(
        delta, "ppmc", "adaptive_quantization", adaptiveQuantization);
    appendPaperI32SideChannel(delta, "ppmc", "group_filter", groupFilter);
}

template <typename FaceSymbols>
inline void appendPPMCPaperFaceSymbols(PaperRoundSymbols& delta,
                                       const FaceSymbols& faceSymbols)
{
    for (const auto& sym : faceSymbols) {
        appendPaperScalarSymbolForRole(
            delta,
            PMSFSymbolRole::Facet,
            static_cast<std::int32_t>(sym.first));
        appendPaperI32SideChannel(
            delta, "ppmc_face_symbol", "a",
            static_cast<std::int32_t>(sym.first));
        appendPaperI32SideChannel(
            delta, "ppmc_face_symbol", "b",
            static_cast<std::int32_t>(sym.second));
    }
}

template <typename EdgeSymbols>
inline void appendPPMCPaperEdgeSymbols(PaperRoundSymbols& delta,
                                       const EdgeSymbols& edgeSymbols)
{
    for (const auto& sym : edgeSymbols) {
        appendPaperScalarSymbolForRole(
            delta,
            PMSFSymbolRole::TriangulationEdge,
            static_cast<std::int32_t>(sym.first));
        appendPaperI32SideChannel(
            delta, "ppmc_edge_symbol", "a",
            static_cast<std::int32_t>(sym.first));
        appendPaperI32SideChannel(
            delta, "ppmc_edge_symbol", "b",
            static_cast<std::int32_t>(sym.second));
    }
}

template <typename GeometrySymbols>
inline void appendPPMCPaperGeometrySymbols(PaperRoundSymbols& delta,
                                           const GeometrySymbols& geometrySymbols)
{
    // PPMC geometry symbols are integer residuals (C_r^f), not absolute
    // coordinates.  Store them directly so the paper geometry codec can
    // delta/varint them instead of re-quantizing float coordinates.
    for (const auto& p : geometrySymbols) {
        delta.removedVertexResiduals.push_back(MCGAL::PointInt(p.x(), p.y(), p.z()));
    }
}

} // namespace pmsf

#endif // PMSF_CORE_PPMC_PAPER_BOUNDARY_H
