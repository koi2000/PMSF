#ifndef PMSF_CORE_DEFAULT_PPMC_LITE_PAPER_BOUNDARY_H
#define PMSF_CORE_DEFAULT_PPMC_LITE_PAPER_BOUNDARY_H

#include "storage/PMSFSymbolStorageAdapter.h"

#include <cstdint>

namespace pmsf {

inline bool defaultPPMCLitePMSFBoundaryEnvOn(const char* name)
{
    return pmsfStorageEnvEnabled(name);
}

inline bool defaultPPMCLitePaperExactRequired()
{
    return defaultPPMCLitePMSFBoundaryEnvOn("PMSF_PPMC_REQUIRE_PMSF_EXACT") ||
           defaultPPMCLitePMSFBoundaryEnvOn("PMSF_PPMC_REQUIRE_CURVATURE_EXACT") ||
           defaultPPMCLitePMSFBoundaryEnvOn("PMSF_DEFAULT_PPMC_LITE_REQUIRE_PAPER_EXACT") ||
           defaultPPMCLitePMSFBoundaryEnvOn("PMSF_DEFAULT_PPMC_LITE_REQUIRE_CURVATURE_EXACT");
}

inline bool defaultPPMCLiteCurvatureExactRequired()
{
    return defaultPPMCLitePMSFBoundaryEnvOn("PMSF_PPMC_REQUIRE_CURVATURE_EXACT") ||
           defaultPPMCLitePMSFBoundaryEnvOn("PMSF_PPMC_REQUIRE_PMSF_EXACT") ||
           defaultPPMCLitePMSFBoundaryEnvOn("PMSF_DEFAULT_PPMC_LITE_REQUIRE_CURVATURE_EXACT") ||
           defaultPPMCLitePMSFBoundaryEnvOn("PMSF_DEFAULT_PPMC_LITE_REQUIRE_PAPER_EXACT");
}

inline std::int32_t defaultPPMCLitePaperExactRequiredI32()
{
    return defaultPPMCLitePaperExactRequired() ? 1 : 0;
}

inline std::int32_t defaultPPMCLiteCurvatureExactRequiredI32()
{
    return defaultPPMCLiteCurvatureExactRequired() ? 1 : 0;
}

} // namespace pmsf

#endif // PMSF_CORE_DEFAULT_PPMC_LITE_PAPER_BOUNDARY_H
