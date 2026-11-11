#ifndef PMSF_CORE_MMSE_PAPER_BOUNDARY_H
#define PMSF_CORE_MMSE_PAPER_BOUNDARY_H

#include "core/PMSFBoundaryComponent.h"
#include "core/PMSFBoundarySideChannel.h"
#include "core/PMSFSideChannel.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include <cstdint>

namespace pmsf {
namespace mmse_boundary {

// R3/L1（PHASE63）：原先 7 个样板函数（`...EnvOn` 转发 + 3 个 `...Required`
// + 3 个 `...I32`）全是 `envOn(A) || ...` 与 `? 1 : 0`。机制已收进
// `core/PMSFBoundaryComponent.h`；本文件按「算法侧只留参数声明」只保留 **env 名**。
//
// ⚠️ 短路顺序逐条照抄原式：
//   StrictDualRingRequired     = STRICT_DUAL_RING
//   ModeDecisionExactRequired  = MODE_DECISION_EXACT
//   PaperExactRequired         = PAPER_EXACT || STRICT_DUAL_RING || MODE_DECISION_EXACT
// ⇒ 产物逐字节不变。

const char* const kStrictDualRing[] = {"PMSF_MMSE_REQUIRE_STRICT_DUAL_RING"};
const char* const kModeDecisionExact[] = {"PMSF_MMSE_REQUIRE_MODE_DECISION_EXACT"};
const char* const kPaperExact[] = {
    "PMSF_MMSE_REQUIRE_PAPER_EXACT",
    "PMSF_MMSE_REQUIRE_STRICT_DUAL_RING",
    "PMSF_MMSE_REQUIRE_MODE_DECISION_EXACT",
};

constexpr BoundaryFlagSpec kStrictDualRingSpec{kStrictDualRing, 1};
constexpr BoundaryFlagSpec kModeDecisionExactSpec{kModeDecisionExact, 1};
constexpr BoundaryFlagSpec kPaperExactSpec{kPaperExact, 3};

}  // namespace mmse_boundary

inline void appendMMSEBoundaryCapabilitySideChannels(
    PaperRoundSymbols& delta,
    std::int32_t dualRingPredictionPresent,
    std::int32_t modeDecisionPresent)
{
    appendPMSFBoundaryAbsentExactPair(
        delta, "mmse", "paper", boundaryFlagI32(mmse_boundary::kPaperExactSpec));
    appendPaperI32SideChannel(
        delta, "mmse", "dual_ring_prediction_present",
        dualRingPredictionPresent);
    appendPaperI32SideChannel(
        delta, "mmse", "strict_dual_ring_required",
        boundaryFlagI32(mmse_boundary::kStrictDualRingSpec));
    appendPaperI32SideChannel(
        delta, "mmse", "mode_decision_present", modeDecisionPresent);
    appendPMSFBoundaryAbsentExactPair(
        delta,
        "mmse",
        "mode_decision",
        boundaryFlagI32(mmse_boundary::kModeDecisionExactSpec));
}

inline void appendMMSEStorageContextSideChannels(
    PaperRoundSymbols& delta,
    float alpha,
    std::int32_t fixedDualRing,
    std::int32_t clusterModeDecision,
    std::int32_t strictStorageRequested)
{
    appendPaperF32SideChannel(delta, "mmse", "alpha", alpha);
    appendPaperI32SideChannel(
        delta, "mmse", "fixed_dual_ring", fixedDualRing);
    appendPaperI32SideChannel(
        delta, "mmse", "cluster_mode_decision", clusterModeDecision);
    appendPaperI32SideChannel(
        delta, "mmse", "strict_storage_requested",
        strictStorageRequested);
    appendPaperI32SideChannel(
        delta, "mmse", "strict_blob_layout",
        strictStorageRequested ? 27 : 0);
}

template <typename SideMap, typename Values>
inline void replaceMMSEPredictionModeSideMap(SideMap& side,
                                             const Values& values)
{
    replacePaperI32SideMap(side, "mmse", "prediction_mode", values);
}

template <typename PaperRoundSymbols, typename Values>
inline void replaceMMSEPredictionModeSymbols(PaperRoundSymbols& delta,
                                             const Values& values)
{
    replacePaperI32SideChannel(delta, "mmse", "prediction_mode", values);
}

} // namespace pmsf

#endif // PMSF_CORE_MMSE_PAPER_BOUNDARY_H
