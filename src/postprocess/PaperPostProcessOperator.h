#pragma once

#include "Mesh.h"

#include <memory>
#include <string>

namespace pmsf {

/// Paper-specific reconstruction postprocessors used by the decompress pipeline.
/// These are explicit, env-selected hooks; they never run by default.
class PaperPostProcessOperator {
  public:
    /// mode: "snap12" = snap coordinates to a 12-bit bbox grid;
    ///       "none"  = no-op.
    static int apply(const std::shared_ptr<MCGAL::Mesh>& mesh,
                     const std::string& mode);
};

}  // namespace pmsf
