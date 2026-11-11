#pragma once
// Diagnostic: estimate the storage needed to make mixed-gid boundary
// collapses reversible.  Only counters; never affects the codec.
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace pmsf {

struct BoundaryTopologyStats {
    std::uint64_t mixedRemovals = 0;
    std::uint64_t mixedRingFaces = 0;
    std::uint64_t mixedTransitions = 0;
    std::uint64_t maxTransitions = 0;
    std::uint64_t outerBoundaryRemovals = 0;
    std::uint64_t totalRemovals = 0;
};

inline BoundaryTopologyStats& boundaryTopologyStats() {
    static BoundaryTopologyStats s;
    return s;
}

inline std::uint64_t estimateBoundaryPatchBits(const BoundaryTopologyStats& s) {
    // Conservative upper estimate per mixed vertex:
    //   dominant gid (6 bits) + ring size header (8 bits)
    //   + per transition: run length (8 bits) + gid (6 bits)
    return s.mixedRemovals * 14 + s.mixedTransitions * 14;
}

inline void printBoundaryTopologyStats() {
    const auto& s = boundaryTopologyStats();
    if (std::getenv("PMSF_BOUNDARY_TOPOLOGY_STATS") == nullptr) return;
    const std::uint64_t bits = estimateBoundaryPatchBits(s);
    std::fprintf(stderr,
                 "[boundary-topology-stats] removals=%llu mixed=%llu outer=%llu "
                 "ringFaces=%llu transitions=%llu maxTransitions=%llu "
                 "estBits=%llu estBytes=%.1f\n",
                 (unsigned long long)s.totalRemovals,
                 (unsigned long long)s.mixedRemovals,
                 (unsigned long long)s.outerBoundaryRemovals,
                 (unsigned long long)s.mixedRingFaces,
                 (unsigned long long)s.mixedTransitions,
                 (unsigned long long)s.maxTransitions,
                 (unsigned long long)bits, static_cast<double>(bits) / 8.0);
    std::fflush(stderr);
}

}  // namespace pmsf
