#ifndef PMSF_CORE_SEGMENTATION_PAPER_BOUNDARY_H
#define PMSF_CORE_SEGMENTATION_PAPER_BOUNDARY_H

#include "storage/PMSFSymbolStorageAdapter.h"

#include <cstdint>

namespace pmsf {

inline bool segmentationPaperExactRequired()
{
    return pmsfStorageEnvEnabled("PMSF_SEGMENTATION_REQUIRE_PMSF_EXACT");
}

inline std::int32_t segmentationPaperExactRequiredI32()
{
    return segmentationPaperExactRequired() ? 1 : 0;
}

} // namespace pmsf

#endif // PMSF_CORE_SEGMENTATION_PAPER_BOUNDARY_H
