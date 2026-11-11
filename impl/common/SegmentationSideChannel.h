/*****************************************************************************
 * @Author: koi
 * @Description: Segmentation collector 的 paper side-channel 辅助函数。
 * 阶段 F：从 src/core/DefaultCodecPMSFSideChannel.h 下沉到 impl/common，
 *         避免 src/core 依赖 PPMC 算法边界头文件。
 *****************************************************************************/
#ifndef IMPL_COMMON_SEGMENTATION_SIDE_CHANNEL_H
#define IMPL_COMMON_SEGMENTATION_SIDE_CHANNEL_H

#include "core/Symbol.h"
#include "core/PaperGeometrySideChannel.h"
#include "core/PMSFSideChannel.h"
#include "core/PMSFBoundarySideChannel.h"
#include "PPMC/boundary/SegmentationPMSFBoundary.h"

#include <cstdint>

namespace pmsf {


template <typename PaperRoundSymbols, typename QuantizedPoints>
inline void appendSegmentationRemovedPointSideChannel(
    PaperRoundSymbols& delta,
    const QuantizedPoints& points)
{
    for (const auto& q : points) {
        appendPaperQuantizedPointSideChannel(delta, "segmentation_point", q);
    }
}

template <typename PaperRoundSymbols>
inline void appendSegmentationStorageSideChannel(PaperRoundSymbols& delta,
                                                 std::int32_t groupId)
{
    appendPaperI32SideChannel(delta, "segmentation", "topology_stream", 0);
    appendPaperI32SideChannel(delta, "segmentation", "group_id", groupId);
    appendPaperI32SideChannel(
        delta, "segmentation", "processed_face_guard", 1);
    appendPaperI32SideChannel(
        delta, "segmentation", "processed_edge_guard", 1);
    appendPaperI32SideChannel(delta, "segmentation", "geometry_basis", 0);
    appendPMSFBoundaryAbsentExactPair(
        delta, "segmentation", "paper", segmentationPaperExactRequiredI32());
}


}  // namespace pmsf

#endif  // IMPL_COMMON_SEGMENTATION_SIDE_CHANNEL_H
