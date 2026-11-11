#pragma once
// Independent, optional boundary-compression module.
// Standard codec sections are never modified: when enabled this module writes
// a separate boundary section (companion file <artifact>.bnd).
#include "Mesh.h"
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace pmsf {

class BoundaryCompressionModule {
  public:
    static constexpr const char* kMagic = "BND1";
    static constexpr int kVersion = 1;
    /// Versioned payload section (BND2): raw little-endian int32 payload.
    /// Used by the optional boundary patch mode; the standard artifact does not
    /// reference this section yet.
    static bool writePayloadSection(const std::string& path,
                                    const std::vector<int>& payload,
                                    std::size_t* outBytes = nullptr);
    static bool readPayloadSection(const std::string& path, std::vector<int>& payload);
    static std::string sectionPathForArtifact(const std::string& artifactPath) {
        return artifactPath + ".bnd";
    }
};

/// Apply the companion boundary-patch payload (mode23 v2 op list) to a fully
/// decoded mesh.  Used when the module owns the boundary payload and the
/// standard artifact carries none.
bool applyBoundaryInversePayload(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                 const std::vector<int>& payload);

}  // namespace pmsf
