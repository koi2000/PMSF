#pragma once

namespace pmsf {

// Payload-driven override: mode71 artifacts that were encoded with K-free boundary
// stages need the deterministic reconstruction path (no join replay).  The decoder
// sets this from the self-describing payload flag, so artifacts stay self-contained.
inline bool& forceNoJoinReplay() {
    static bool v = false;
    return v;
}

}  // namespace pmsf
