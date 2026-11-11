/*
 * @Author: koi
 * @Date: 2025-08-31 22:33:59
 * @Description:
 */
/*
 * @Author: koi
 * @Date: 2025-08-29 21:13:56
 * @Description:
 */

#include "BasicSymbolReadOperator.h"
#include "../common/DebugTool.h"
#include "../rangecoder/qsmodel.h"
#include "../rangecoder/rangecod.h"
#include "BFSVersionManager.h"
#include "BufferUtils.h"
#include "Configuration.h"
#include "Facet.h"
#include "MeshUtils.h"
#include "core.h"
#include <cstdint>
#include <deque>
#include <sys/types.h>

void BasicSymbolReadOperator::collect(MCGAL::Halfedge* seed,
                                      char* buffer,
                                      size_t& dataOffset,
                                      std::vector<MCGAL::Vertex*>& vertices,
                                      std::vector<MCGAL::Halfedge*>& halfedge,
                                      std::vector<MCGAL::Facet*>& facets) {
#ifdef DEBUG
    std::deque<char> facetSymbol;
    std::deque<char> edgeSymbol;
    std::deque<MCGAL::Point> points;
#endif
    // std::queue<int> gateQueue;

    gateQueue.push(seed->poolId());
    int current_version = MCGAL::BFSVersionManager::current_version++;
    unsigned i_bitOffset = 0;
    unsigned p_bitOffset = 0;
    while (!gateQueue.empty()) {
        int hid = gateQueue.front();
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndexInSubPool(DEFAULT_MESH_ID, hid);
        gateQueue.pop();
        if (h->isVisited(current_version) || h->isRemoved()) {
            continue;
        }
        h->setVisited(current_version);

        if (!h->face()->isVisited(current_version)) {
            MCGAL::Facet* f = h->face();
            char symbol;
            if (enableQuantization_) {
                symbol = readBitsOff(1, buffer, i_bitOffset, dataOffset);
            } else {
                symbol = readChar(buffer, dataOffset);
            }
#ifdef DEBUG
            facetSymbol.push_back(symbol);
#endif
            f->setVisited(current_version);
            if (symbol) {
                markFacetSplittableThisRound(f);
                MCGAL::Point point;
                if (enableQuantization_) {
                    uint32_t p[3];
                    if (dataOffset == 326429) {
                        int i = 0;
                    }
                    for (int i = 0; i < 3; i++) {
                        p[i] = readBitsOff(12, buffer, p_bitOffset, offsets[0]);
                    }
                    MCGAL::PointInt pi = MCGAL::PointInt(p[0], p[1], p[2]);
                    point[0] = bboxMin_[0] + (p[0] + 0.5) * f_quantStep_;
                    point[1] = bboxMin_[1] + (p[1] + 0.5) * f_quantStep_;
                    point[2] = bboxMin_[2] + (p[2] + 0.5) * f_quantStep_;
                    // point = MCGAL::intPosToFloat(MCGAL::PointInt(p[0], p[1], p[2]), bboxMin_, f_quantStep_);
                } else {
                    point = readPoint(buffer, offsets[0]);
                }
                f->setRemovedVertexPos(point);
                facets.push_back(f);
            }
        }
        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp != nullptr && !hOpp->isVisited(current_version)) {
                gateQueue.push(hOpp->poolId());
            }
            if (hIt != nullptr && !hIt->isVisited(current_version)) {
                gateQueue.push(hIt->poolId());
            }
            hIt = hIt->next();
        } while (hIt != h);
    }

    current_version = MCGAL::BFSVersionManager::current_version++;
    gateQueue.push(seed->poolId());

    while (!gateQueue.empty()) {
        int hid = gateQueue.front();
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndexInSubPool(DEFAULT_MESH_ID, hid);
        gateQueue.pop();
        if (h->isVisited(current_version) || h->isRemoved()) {
            continue;
        }
        h->setVisited(current_version);
        char symbol;
        // if (!h->isVisited(current_version)) {
        if (enableQuantization_) {
            symbol = readBitsOff(1, buffer, i_bitOffset, dataOffset);
        } else {
            symbol = readChar(buffer, dataOffset);
        }
#ifdef DEBUG
        edgeSymbol.push_back(symbol);
#endif
        if (symbol) {
            h->setAdded();
            halfedge.push_back(h);
        }
        // }
        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp != nullptr && !hOpp->isVisited(current_version)) {
                gateQueue.push(hOpp->poolId());
            }
            if (hIt != nullptr && !hIt->isVisited(current_version)) {
                gateQueue.push(hIt->poolId());
            }
            hIt = hIt->next();
        } while (hIt != h);
    }
    if (header_.isEnableAdjustment()) {
        current_version = MCGAL::BFSVersionManager::current_version++;
        gateQueue.push(seed->poolId());

        while (!gateQueue.empty()) {
            int hid = gateQueue.front();
            MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndexInSubPool(DEFAULT_MESH_ID, hid);
            gateQueue.pop();
            if (h->isVisited(current_version)) {
                continue;
            }
            h->setVisited(current_version);
            if (!h->vertex()->isVisited(current_version)) {
                char symbol;
                if (enableQuantization_) {
                    symbol = readBitsOff(1, buffer, i_bitOffset, dataOffset);
                } else {
                    symbol = readChar(buffer, dataOffset);
                }
                h->vertex()->setVisited(current_version);
                if (symbol) {
                    h->vertex()->setAdjusted();
                    vertices.push_back(h->vertex());
                    MCGAL::Point point;
                    if (enableQuantization_) {
                        uint32_t p[3];
                        for (int i = 0; i < 3; i++) {
                            p[i] = readBitsOff(12, buffer, p_bitOffset, offsets[0]);
                        }
                        MCGAL::PointInt pi = MCGAL::PointInt(p[0], p[1], p[2]);
                        point[0] = bboxMin_[0] + (p[0] + 0.5) * f_quantStep_;
                        point[1] = bboxMin_[1] + (p[1] + 0.5) * f_quantStep_;
                        point[2] = bboxMin_[2] + (p[2] + 0.5) * f_quantStep_;
                        // point = MCGAL::intPosToFloat(MCGAL::PointInt(p[0], p[1], p[2]), bboxMin_, f_quantStep_);
                    } else {
                        point = readPoint(buffer, offsets[0]);
                    }
                    h->vertex()->setVertexPoint(point);
                }
            }

            MCGAL::Halfedge* hIt = h;

            do {
                MCGAL::Halfedge* hOpp = hIt->opposite();
                if (hOpp != nullptr && !hOpp->isVisited(current_version)) {
                    gateQueue.push(hOpp->poolId());
                }
                if (hIt != nullptr && !hIt->isVisited(current_version)) {
                    gateQueue.push(hIt->poolId());
                }
                hIt = hIt->next();
            } while (hIt != h);
        }
    }
    if (enableQuantization_) {
        dataOffset++;
        offsets[0]++;
    }
#ifdef DEBUG_DUMP
    DEBUG_DUMP_CONTAINER(edgeSymbol, "./newsegment/de_edge_sym.txt");
    DEBUG_DUMP_CONTAINER(facetSymbol, "./newsegment/de_facet_sym.txt");
    DEBUG_DUMP_HALFEDGES(halfedge, "./newsegment/de_new_halfedges.txt");
    DEBUG_DUMP_FACETS(facets, "./newsegment/de_new_facets.txt");
// int i = 9;
#endif
}

void BasicSymbolReadOperator::collect(std::vector<MCGAL::Halfedge*> seeds,
                                      char* buffer,
                                      size_t& dataOffset,
                                      std::vector<MCGAL::Vertex*>& vertices,
                                      std::vector<MCGAL::Halfedge*>& halfedge,
                                      std::vector<MCGAL::Facet*>& facets) {}

void BasicSymbolReadOperator::importFromParquet(const std::vector<bool>& facetSymbols,
                                                const std::vector<bool>& edgeSymbols,
                                                const std::vector<bool>& vertexSymbols,
                                                const std::vector<MCGAL::Point>& points) {
    parquetFacetSymbols_.clear();
    parquetEdgeSymbols_.clear();
    parquetVertexSymbols_.clear();
    parquetPoints_.clear();
    parquetVertexPoints_.clear();

    for (bool s : facetSymbols) parquetFacetSymbols_.push_back(s ? 1 : 0);
    for (bool s : edgeSymbols) parquetEdgeSymbols_.push_back(s ? 1 : 0);
    for (bool s : vertexSymbols) parquetVertexSymbols_.push_back(s ? 1 : 0);
    for (auto& p : points) parquetPoints_.push_back(p);

    useParquetMode_ = true;
}

void BasicSymbolReadOperator::importFromParquetExtended(const std::vector<bool>& facetSymbols,
                                                        const std::vector<bool>& edgeSymbols,
                                                        const std::vector<bool>& vertexSymbols,
                                                        const std::vector<MCGAL::Point>& points,
                                                        const std::vector<MCGAL::Point>& vertexPoints) {
    parquetFacetSymbols_.clear();
    parquetEdgeSymbols_.clear();
    parquetVertexSymbols_.clear();
    parquetPoints_.clear();
    parquetVertexPoints_.clear();

    for (bool s : facetSymbols) parquetFacetSymbols_.push_back(s ? 1 : 0);
    for (bool s : edgeSymbols) parquetEdgeSymbols_.push_back(s ? 1 : 0);
    for (bool s : vertexSymbols) parquetVertexSymbols_.push_back(s ? 1 : 0);
    for (auto& p : points) parquetPoints_.push_back(p);
    for (auto& p : vertexPoints) parquetVertexPoints_.push_back(p);

    useParquetMode_ = true;
}

void BasicSymbolReadOperator::collectFromDecoded(MCGAL::Halfedge* seed,
                                                 std::vector<MCGAL::Vertex*>& vertices,
                                                 std::vector<MCGAL::Halfedge*>& halfedge,
                                                 std::vector<MCGAL::Facet*>& facets) {
    if (!useParquetMode_) return;

    gateQueue.push(seed->poolId());
    int current_version = MCGAL::BFSVersionManager::current_version++;

    while (!gateQueue.empty()) {
        int hid = gateQueue.front();
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndexInSubPool(DEFAULT_MESH_ID, hid);
        gateQueue.pop();

        if (h->isVisited(current_version) || h->isRemoved()) continue;
        h->setVisited(current_version);

        if (!h->face()->isVisited(current_version)) {
            MCGAL::Facet* f = h->face();
            f->setVisited(current_version);

            char symbol = 0;
            if (!parquetFacetSymbols_.empty()) {
                symbol = parquetFacetSymbols_.front();
                parquetFacetSymbols_.pop_front();
            }
            if (symbol) {
                markFacetSplittableThisRound(f);
                MCGAL::Point point;
                if (!parquetPoints_.empty()) {
                    point = parquetPoints_.front();
                    parquetPoints_.pop_front();
                }
                f->setRemovedVertexPos(point);
                facets.push_back(f);
            }
        }

        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp != nullptr && !hOpp->isVisited(current_version)) gateQueue.push(hOpp->poolId());
            if (hIt != nullptr && !hIt->isVisited(current_version)) gateQueue.push(hIt->poolId());
            hIt = hIt->next();
        } while (hIt != h);
    }

    current_version = MCGAL::BFSVersionManager::current_version++;
    gateQueue.push(seed->poolId());

    while (!gateQueue.empty()) {
        int hid = gateQueue.front();
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndexInSubPool(DEFAULT_MESH_ID, hid);
        gateQueue.pop();

        if (h->isVisited(current_version) || h->isRemoved()) continue;
        h->setVisited(current_version);

        char symbol = 0;
        if (!parquetEdgeSymbols_.empty()) {
            symbol = parquetEdgeSymbols_.front();
            parquetEdgeSymbols_.pop_front();
        }
        if (symbol) {
            h->setAdded();
            halfedge.push_back(h);
        }

        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp != nullptr && !hOpp->isVisited(current_version)) gateQueue.push(hOpp->poolId());
            if (hIt != nullptr && !hIt->isVisited(current_version)) gateQueue.push(hIt->poolId());
            hIt = hIt->next();
        } while (hIt != h);
    }

    if (header_.isEnableAdjustment()) {
        current_version = MCGAL::BFSVersionManager::current_version++;
        gateQueue.push(seed->poolId());

        while (!gateQueue.empty()) {
            int hid = gateQueue.front();
            MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndexInSubPool(DEFAULT_MESH_ID, hid);
            gateQueue.pop();

            if (h->isVisited(current_version) || h->isRemoved()) continue;
            h->setVisited(current_version);

            if (!h->vertex()->isVisited(current_version)) {
                h->vertex()->setVisited(current_version);

                char symbol = 0;
                if (!parquetVertexSymbols_.empty()) {
                    symbol = parquetVertexSymbols_.front();
                    parquetVertexSymbols_.pop_front();
                }

                if (symbol) {
                    h->vertex()->setAdjusted();
                    vertices.push_back(h->vertex());

                    MCGAL::Point point;
                    if (!parquetVertexPoints_.empty()) {
                        point = parquetVertexPoints_.front();
                        parquetVertexPoints_.pop_front();
                    } else if (!parquetPoints_.empty()) {
                        // 兼容历史单点流文件：顶点点位可能仍拼接在主点流尾部
                        point = parquetPoints_.front();
                        parquetPoints_.pop_front();
                    }
                    h->vertex()->setVertexPoint(point);
                }
            }

            MCGAL::Halfedge* hIt = h;
            do {
                MCGAL::Halfedge* hOpp = hIt->opposite();
                if (hOpp != nullptr && !hOpp->isVisited(current_version)) gateQueue.push(hOpp->poolId());
                if (hIt != nullptr && !hIt->isVisited(current_version)) gateQueue.push(hIt->poolId());
                hIt = hIt->next();
            } while (hIt != h);
        }
    }

    useParquetMode_ = false;
}
