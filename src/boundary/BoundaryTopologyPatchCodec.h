#pragma once
// Compact boundary-topology gid patch (experimental).
//
// The patch is stored in the existing boundary payload bit channel.
// Per mixed-gid boundary removal we store:
//   round (6 bits), center q (12*3 bits, delta-coded after sorting),
//   fan size d (5 bits), and d gids (5 bits each) sorted by the canonical
//   triangle key (sorted vertex coords).  The decoder re-sorts its
//   reconstructed fan faces by the same key, so no ring rotation is stored.
#include "Mesh.h"
#include <cstdint>
#include <memory>
#include <vector>

namespace pmsf {

struct BoundaryTopologyPatchOp {
    int round = 0;
    int qx = 0;
    int qy = 0;
    int qz = 0;
    std::vector<int> gids;
};

std::vector<unsigned char> encodeBoundaryTopologyPatchBits(
    const std::vector<BoundaryTopologyPatchOp>& ops);

bool parseBoundaryTopologyPatchBits(const std::vector<unsigned char>& bits,
                                    std::vector<BoundaryTopologyPatchOp>& ops);

int applyBoundaryTopologyPatchRound(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                    const std::vector<unsigned char>& bits,
                                    int round);

/// In-insert application: called right after a center vertex/fan has been
/// reconstructed, before the next symbol is consumed.
std::vector<BoundaryTopologyPatchOp>& pendingBoundaryTopologyOps();
int& pendingBoundaryTopologyRound();
bool applyBoundaryTopologyAtVertex(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                   MCGAL::Vertex* center,
                                   int round);

}  // namespace pmsf
