/*
 * VQMMCStrictSymbolReadOperator — Range-Coder decoder.
 */
#include "VQMMCStrictSymbolReadOperator.h"

#include "BFSVersionManager.h"
#include "Configuration.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Point.h"

#include "VQMMCStrictSymbolCollectOperator.h"

#include <cstdint>
#include <cstring>
#include <vector>

void VQMMCStrictSymbolReadOperator::parseStrictBlob(char* buffer) {
    std::vector<uint8_t> blob(blobSize_);
    std::memcpy(blob.data(), buffer, blobSize_);

    decodedBlob_ =
        pmsf::readVectorQuantizedConnectivityBlob(blob, kVQMMCStrictBlobLayout);
    schemeVersion_ = decodedBlob_.schemeVersion;
    numRounds_ = decodedBlob_.numRounds;
    blobParsed_ = true;
    currentRound_ = 0;
}

void VQMMCStrictSymbolReadOperator::collect(
    MCGAL::Halfedge* seed, char* buffer, size_t& dataOffset,
    std::vector<MCGAL::Vertex*>& vertices,
    std::vector<MCGAL::Halfedge*>& halfedge,
    std::vector<MCGAL::Facet*>& facets) {

    if (!blobParsed_) parseStrictBlob(buffer);
    if (currentRound_ >= numRounds_) return;

    pmsf::applyVectorQuantizedDecodedRound(
        schemeVersion_,
        currentRound_,
        decodedBlob_,
        seed,
        vertices,
        halfedge,
        facets);
    dataOffset++;
    currentRound_++;
}

void VQMMCStrictSymbolReadOperator::collect(
    std::vector<MCGAL::Halfedge*> seeds, char* buffer, size_t& dataOffset,
    std::vector<MCGAL::Vertex*>& vertices,
    std::vector<MCGAL::Halfedge*>& halfedge,
    std::vector<MCGAL::Facet*>& facets) {
    if (!seeds.empty())
        collect(seeds[0], buffer, dataOffset, vertices, halfedge, facets);
}
