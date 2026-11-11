#pragma once

#include "decompress/SymbolReadOperator.h"
#include "common/FixedQueue.h"
#include <cstddef>
#include <vector>

class VQMMCSymbolReadOperator : public SymbolReadOperator {
  public:
    VQMMCSymbolReadOperator() = default;

    void collect(MCGAL::Halfedge* seed,
                 char* buffer,
                 size_t& dataOffset,
                 std::vector<MCGAL::Vertex*>& vertices,
                 std::vector<MCGAL::Halfedge*>& halfedge,
                 std::vector<MCGAL::Facet*>& facets) override;

    void collect(std::vector<MCGAL::Halfedge*> seeds,
                 char* buffer,
                 size_t& dataOffset,
                 std::vector<MCGAL::Vertex*>& vertices,
                 std::vector<MCGAL::Halfedge*>& halfedge,
                 std::vector<MCGAL::Facet*>& facets) override;

    void setVQDataOffset(size_t offset) { vqDataOffset_ = offset; }
    void setStreamBounds(size_t facetEnd, size_t edgeEnd, size_t vqEnd) {
        facetEnd_ = facetEnd;
        edgeEnd_ = edgeEnd;
        vqEnd_ = vqEnd;
    }
    size_t vqDataOffset() const { return vqDataOffset_; }

  private:
    FixedQueue<int> gateQueue_ = FixedQueue<int>(30000);
    size_t vqDataOffset_ = 0;
    size_t facetEnd_ = 0;
    size_t edgeEnd_ = 0;
    size_t vqEnd_ = 0;
};
