/*
 * @Author: koi
 * @Date: 2025-09-01 09:36:36
 * @Description:
 */

#pragma once

#include "SymbolReadOperator.h"
#include "Vertex.h"
#include <memory>
#include <vector>

class MockSymbolReadOperator : public SymbolReadOperator {
  public:
    MockSymbolReadOperator() = default;

    void collect(MCGAL::Halfedge* seed,
                 char* buffer,
                 size_t& dataOffset,
                 std::vector<MCGAL::Vertex*>& vertices,
                 std::vector<MCGAL::Halfedge*>& halfedge,
                 std::vector<MCGAL::Facet*>& facets);

    void collect(std::vector<MCGAL::Halfedge*> seeds,
                 char* buffer,
                 size_t& dataOffset,
                 std::vector<MCGAL::Vertex*>& vertices,
                 std::vector<MCGAL::Halfedge*>& halfedge,
                 std::vector<MCGAL::Facet*>& facets);
};
