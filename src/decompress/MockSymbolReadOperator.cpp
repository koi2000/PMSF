/*
 * @Author: koi
 * @Date: 2025-08-31 22:33:59
 * @Description:
 */

#include "MockSymbolReadOperator.h"
#include "BufferUtils.h"
#include "../rangecoder/qsmodel.h"
#include "../rangecoder/rangecod.h"
#include "BFSVersionManager.h"
#include "Configuration.h"
#include "Facet.h"
#include "MeshUtils.h"
#include "core.h"
#include <deque>
#include <sys/types.h>

void MockSymbolReadOperator::collect(MCGAL::Halfedge* seed,
                                      char* buffer,
                                      size_t& dataOffset,
                                      std::vector<MCGAL::Vertex*>& vertices,
                                      std::vector<MCGAL::Halfedge*>& halfedge,
                                      std::vector<MCGAL::Facet*>& facets) {
}

void MockSymbolReadOperator::collect(std::vector<MCGAL::Halfedge*> seeds,
                                      char* buffer,
                                      size_t& dataOffset,
                                      std::vector<MCGAL::Vertex*>& vertices,
                                      std::vector<MCGAL::Halfedge*>& halfedge,
                                      std::vector<MCGAL::Facet*>& facets) {}