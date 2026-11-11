/*****************************************************************************
 * @Author: koi
 * @Description: DefaultPPMCLite collector 的 paper side-channel 辅助函数。
 * 阶段 F：从 src/core/DefaultCodecPMSFSideChannel.h 下沉到 impl/DefaultPPMCLite，
 *         避免 src/core 依赖 PPMC 算法边界头文件。
 *****************************************************************************/
#ifndef IMPL_DEFAULT_PPMC_LITE_SIDE_CHANNEL_H
#define IMPL_DEFAULT_PPMC_LITE_SIDE_CHANNEL_H

#include "core/Symbol.h"
#include "core/PaperGeometrySideChannel.h"
#include "core/PMSFSideChannel.h"
#include "core/PMSFBoundarySideChannel.h"
#include "PPMC/boundary/DefaultPPMCLitePMSFBoundary.h"

#include <cstdint>

namespace pmsf {


template <typename RoundSymbols, typename QuantizedPoints>
inline void appendDefaultPPMCLiteRemovedPointSideChannel(
    RoundSymbols& round,
    const QuantizedPoints& points)
{
    for (const auto& q : points) {
        appendPaperQuantizedPointSideChannel(
            round, "default_ppmc_lite_point", q);
    }
}


template <typename RoundSymbols, typename MeshPtr, typename Points>
inline void appendDefaultPPMCLiteVertexPointSideChannel(
    RoundSymbols& round,
    const MeshPtr& mesh,
    const Points& points)
{
    if (!mesh) return;
    for (const auto& p : points) {
        appendPaperQuantizedPointSideChannel(
            round, "default_ppmc_lite_vertex_point", mesh->floatPosToInt(p));
    }
}

template <typename PaperRoundSymbols>
inline void appendDefaultPPMCLiteStorageSideChannel(PaperRoundSymbols& delta,
                                                    std::int32_t geometryBasis)
{
    erasePMSFSideChannel(delta, "default_ppmc_lite_geometry_basis");
    erasePMSFSideChannel(delta, "default_ppmc_lite_area_prediction");
    erasePMSFSideChannel(delta, "default_ppmc_lite_adaptive_lod_rc_reset");
    appendPaperI32SideChannel(
        delta, "default_ppmc_lite", "geometry_basis", geometryBasis);
    appendPaperI32SideChannel(delta, "default_ppmc_lite", "area_prediction", 0);
    appendPaperI32SideChannel(
        delta, "default_ppmc_lite", "adaptive_lod_rc_reset", 0);
    appendPMSFBoundaryAbsentExactPair(
        delta, "default_ppmc_lite", "paper",
        defaultPPMCLitePaperExactRequiredI32());
    appendPMSFBoundaryAbsentExactPair(
        delta, "default_ppmc_lite", "curvature_prediction",
        defaultPPMCLiteCurvatureExactRequiredI32());
}


}  // namespace pmsf

#endif  // IMPL_DEFAULT_PPMC_LITE_SIDE_CHANNEL_H
