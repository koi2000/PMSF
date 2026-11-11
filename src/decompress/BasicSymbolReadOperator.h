/*
 * @Author: koi
 * @Date: 2025-09-01 09:36:36
 * @Description:
 */

#pragma once

#include "SymbolReadOperator.h"
#include "Vertex.h"
#include <cstddef>
#include <deque>
#include <memory>
#include <vector>
#include "common/FixedQueue.h"

class BasicSymbolReadOperator : public SymbolReadOperator {
  public:
    BasicSymbolReadOperator() = default;

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

    /// @brief 从 Parquet 结构化符号导入（单点流兼容接口）
    void importFromParquet(const std::vector<bool>& facetSymbols,
                           const std::vector<bool>& edgeSymbols,
                           const std::vector<bool>& vertexSymbols,
                           const std::vector<MCGAL::Point>& points) override;

    /// @brief 从 Parquet 结构化符号导入（双点流：facet points + vertex points）
    void importFromParquetExtended(const std::vector<bool>& facetSymbols,
                                   const std::vector<bool>& edgeSymbols,
                                   const std::vector<bool>& vertexSymbols,
                                   const std::vector<MCGAL::Point>& points,
                                   const std::vector<MCGAL::Point>& vertexPoints) override;

    void collectFromDecoded(MCGAL::Halfedge* seed,
                            std::vector<MCGAL::Vertex*>& vertices,
                            std::vector<MCGAL::Halfedge*>& halfedge,
                            std::vector<MCGAL::Facet*>& facets) override;

  private:
    FixedQueue<int> gateQueue =  FixedQueue<int>(30000);

    std::deque<char> parquetFacetSymbols_;
    std::deque<char> parquetEdgeSymbols_;
    std::deque<char> parquetVertexSymbols_;
    std::deque<MCGAL::Point> parquetPoints_;
    std::deque<MCGAL::Point> parquetVertexPoints_;
    bool useParquetMode_ = false;
    // void collectHalfedgeSymbol(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Halfedge* seed);
    // void collectFacetSymbol(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Halfedge* seed);
};
