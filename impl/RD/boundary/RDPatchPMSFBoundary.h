#ifndef PMSF_CORE_RD_PATCH_PAPER_BOUNDARY_H
#define PMSF_CORE_RD_PATCH_PAPER_BOUNDARY_H

#include "storage/PMSFSymbolStorageAdapter.h"

#include <cstdint>
#include <string>

namespace pmsf {

inline bool rdPatchPaperExactRequired()
{
    return pmsfStorageEnvEnabled("PMSF_RDPATCH_REQUIRE_PMSF_EXACT");
}

inline bool rdPatchPaperExactRequiredForPrefix(const std::string& prefix)
{
    return prefix == "rd_patch" && rdPatchPaperExactRequired();
}

inline std::int32_t rdPatchPaperExactRequiredI32()
{
    return rdPatchPaperExactRequired() ? 1 : 0;
}

} // namespace pmsf

#endif // PMSF_CORE_RD_PATCH_PAPER_BOUNDARY_H
