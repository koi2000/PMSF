#pragma once
// Boundary storage design prototype (measurement only).
//
// The compact representation stores ONE anchor vertex per group-boundary
// segment plus one direction bit; the remaining boundary is derived by
// walking the group-boundary graph.  The naive representation stores every
// boundary vertex (quantized xyz + group id) and every boundary edge endpoint.
// This module only measures the two representations and never changes the
// artifact format.
#include "Mesh.h"
#include <cstddef>
#include <memory>
#include <vector>

namespace pmsf {

struct BoundaryStorageStats {
    int groupCount = 0;
    int segmentCount = 0;
    int boundaryVertexCount = 0;
    int boundaryEdgeCount = 0;
    unsigned quantBits = 0;
    std::size_t naiveBits = 0;
    std::size_t compactBits = 0;
    std::size_t anchorBits = 0;
    std::size_t bitFlags = 0;
};

BoundaryStorageStats measureBoundaryStorage(const std::shared_ptr<MCGAL::Mesh>& mesh);

struct BoundaryBitPayload {
    std::vector<int> roundBitCounts;
    std::vector<unsigned char> bits;
    std::vector<int> residuals;
};

/// Compact boundary payload (mode 30): one anchor per segment + 1 bit
/// direction + length/checksum.  The anchor coordinates are delta coded.
BoundaryBitPayload buildCompactBoundaryPayload(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                               int rounds);

/// Naive boundary payload (mode 31): explicit boundary vertex/edge table.
/// Used only as the uncompressed-cost reference.
BoundaryBitPayload buildNaiveBoundaryPayload(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                             int rounds);

struct BoundaryVerifyStats {
    int segments = 0;
    int verified = 0;
    int failed = 0;
    int anchorMissing = 0;
    int walkMismatch = 0;
    int fallbackSegments = 0;
};

/// Store the parsed mode-30 payload for a later decoder-side verification
/// pass (called from OptimizedBasicDeserializeOperator).
void setCompactBoundaryPayloadForVerify(std::vector<unsigned char> bits,
                                        std::vector<int> residuals);
bool hasCompactBoundaryPayloadForVerify();

/// Derive boundaries from anchors and compare edge-count/checksum with the
/// encoded payload (order-independent checksum).
BoundaryVerifyStats verifyCompactBoundaryPayload(const std::shared_ptr<MCGAL::Mesh>& mesh);

}  // namespace pmsf
