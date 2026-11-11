#ifndef PMSF_CORE_APTM_PAPER_BOUNDARY_H
#define PMSF_CORE_APTM_PAPER_BOUNDARY_H

// R3/L1（PHASE63）：原先 10 个样板函数
//（`...EnvOn` 转发 + 3 个 `...EnvRequired` + 3 个 `...Required` + 3 个 `...I32`）
// 全是 `envOn(A) || envOn(B) || ...` 与 `? 1 : 0`。机制已收进
// `core/PMSFBoundaryComponent.h`；本文件按「算法侧只留参数声明」只保留 **env 名**。
//
// ⚠️ 短路顺序**逐条照抄**原式（`pmsfStorageEnvEnabled` 无副作用，
//    但顺序仍保持，避免日后加副作用时踩雷）：
//      PaperExactRequired        = PAPER_EXACT || POLAR || ANCHOR
//      PolarGeometryRequired     = POLAR || PAPER_EXACT
//      AnchorReconstructRequired = ANCHOR || PAPER_EXACT
// ⇒ 产物逐字节不变。

#include "core/PMSFBoundaryComponent.h"
#include "core/PMSFBoundarySideChannel.h"
#include "core/PMSFSideChannel.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include <cstddef>
#include <cstdint>

namespace pmsf {
namespace aptm_boundary {

const char* const kPaperExact[] = {
    "PMSF_APTM_REQUIRE_PAPER_EXACT",
    "PMSF_APTM_REQUIRE_POLAR_GEOMETRY",
    "PMSF_APTM_REQUIRE_ANCHOR_RECONSTRUCT",
};
const char* const kPolarGeometry[] = {
    "PMSF_APTM_REQUIRE_POLAR_GEOMETRY",
    "PMSF_APTM_REQUIRE_PAPER_EXACT",
};
const char* const kAnchorReconstruct[] = {
    "PMSF_APTM_REQUIRE_ANCHOR_RECONSTRUCT",
    "PMSF_APTM_REQUIRE_PAPER_EXACT",
};

constexpr BoundaryFlagSpec kPaperExactSpec{kPaperExact, 3};
constexpr BoundaryFlagSpec kPolarGeometrySpec{kPolarGeometry, 2};
constexpr BoundaryFlagSpec kAnchorReconstructSpec{kAnchorReconstruct, 2};

}  // namespace aptm_boundary


template <typename PaperRoundSymbols>
inline void appendAPTMConquestBoundary(PaperRoundSymbols& delta)
{
    appendPaperI32SideChannel(
        delta, "aptm_conquest", "wrapper_over_pclttm_strict", 1);
    appendPaperI32SideChannel(delta, "aptm_conquest", "locator_bridge", 1);
    appendPaperI32SideChannel(delta, "aptm", "anchor_aware_reconstruct", 0);
    appendPaperI32SideChannel(delta, "aptm", "anchor_reconstruct_exact", 0);
    appendPaperI32SideChannel(delta, "aptm", "polar_geometry", 0);
    appendPaperI32SideChannel(delta, "aptm", "polar_geometry_exact", 0);
    appendPaperI32SideChannel(delta, "aptm", "paper_exact_reconstruct", 0);
    appendPMSFBoundaryAbsentExactPair(
        delta, "aptm", "paper", boundaryFlagI32(aptm_boundary::kPaperExactSpec));
    appendPaperI32SideChannel(
        delta, "aptm", "anchor_reconstruct_required",
        boundaryFlagI32(aptm_boundary::kAnchorReconstructSpec));
    appendPaperI32SideChannel(
        delta, "aptm", "polar_geometry_required",
        boundaryFlagI32(aptm_boundary::kPolarGeometrySpec));
}

} // namespace pmsf

#endif // PMSF_CORE_APTM_PAPER_BOUNDARY_H
