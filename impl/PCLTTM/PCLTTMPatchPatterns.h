// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "common/PMSFRetriangulation.h"
#include "conquest/ConquestPatchDiscovery.h"

namespace pmsf::pclttm_conquest {
inline const std::vector<ConquestPatchPattern>& patchPatterns() {
    static const auto result=[] {
        std::vector<ConquestPatchPattern> patterns(17);
        for(int n=3;n<=6;++n)for(int source=1;source<=2;++source)for(int target=1;target<=2;++target) {
            const auto descriptor=paperRetriangulateFig9Exact(n,source,target);
            patterns[paperMepp2TypeFromBoundaryTags(n,source,target)]={n,descriptor.triangles};
        }
        return patterns;
    }();
    return result;
}
} // namespace pmsf::pclttm_conquest
