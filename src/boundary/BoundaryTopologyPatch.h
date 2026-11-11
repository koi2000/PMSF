#pragma once
// Encoder-side recording for boundary-topology gid patches.
// Kept out of the default path: only active when the env is set.
#include "Point.h"
#include <cstdlib>
#include <vector>

namespace pmsf {

struct BoundaryTopologyOp {
    int round = -1;
    MCGAL::PointInt q{0, 0, 0};
    std::vector<int> gids;
};

inline std::vector<BoundaryTopologyOp>& boundaryTopologyOps() {
    static std::vector<BoundaryTopologyOp> ops;
    return ops;
}

inline int& boundaryTopologyRound() {
    static int round = -1;
    return round;
}

inline bool boundaryTopologyPatchEnabled() {
    // perf（2026-11-09）：该检查在消除热路径被每轮/每候选调用，env 进程内不变。
    static const bool enabled = [] {
        const char* e = std::getenv("PMSF_BOUNDARY_TOPOLOGY_PATCH");
        return e && e[0] && e[0] != '0';
    }();
    return enabled;
}

}  // namespace pmsf
