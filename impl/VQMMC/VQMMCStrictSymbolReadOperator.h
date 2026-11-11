/*
 * VQMMCStrictSymbolReadOperator
 * ------------------------------
 * Phase 1 decoder: Reads ConnectivityBlobContainer (BlobLayout=30) with
 * 4 RC-encoded streams per round, then runs BFS traversal using decoded data.
 */
#pragma once

#include "VQMMC/VectorQuantizedConnectivityCodec.h"
#include "VQMMCSymbolReadOperator.h"
#include <vector>

class VQMMCStrictSymbolReadOperator : public VQMMCSymbolReadOperator {
  public:
    VQMMCStrictSymbolReadOperator() : VQMMCSymbolReadOperator() {}

    void setBlobSize(size_t sz) { blobSize_ = sz; }

    void collect(MCGAL::Halfedge* seed, char* buffer, size_t& dataOffset,
                 std::vector<MCGAL::Vertex*>& vertices,
                 std::vector<MCGAL::Halfedge*>& halfedge,
                 std::vector<MCGAL::Facet*>& facets) override;

    void collect(std::vector<MCGAL::Halfedge*> seeds, char* buffer,
                 size_t& dataOffset,
                 std::vector<MCGAL::Vertex*>& vertices,
                 std::vector<MCGAL::Halfedge*>& halfedge,
                 std::vector<MCGAL::Facet*>& facets) override;

  private:
    bool blobParsed_ = false;
    int currentRound_ = 0;
    int numRounds_ = 0;
    int schemeVersion_ = 1;
    size_t blobSize_ = 0;

    pmsf::VectorQuantizedDecodedBlob decodedBlob_;

    void parseStrictBlob(char* buffer);
};
