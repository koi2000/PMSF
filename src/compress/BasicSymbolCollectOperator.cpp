/*
 * @Author: koi
 * @Date: 2025-08-29 21:13:56
 * @Description:
 */

#include "BasicSymbolCollectOperator.h"
#include "../common/DebugTool.h"
#include "../rangecoder/qsmodel.h"
#include "../rangecoder/rangecod.h"
#include "BFSVersionManager.h"
#include "BufferUtils.h"
#include "Configuration.h"
#include "Facet.h"
#include "Halfedge.h"
#include "MeshUtils.h"
#include "Point.h"
#include "core/DefaultCodecPMSFSideChannel.h"
#include "core/PaperGeometrySideChannel.h"
#include "core.h"
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <vector>

void BasicSymbolCollectOperator::collect(std::vector<MCGAL::Halfedge*> seeds) {
    collect(seeds[0]);
}

void BasicSymbolCollectOperator::collect(MCGAL::Halfedge* seed) {
    std::deque<char> facetSym;
    std::deque<char> edgeSym;
    std::deque<char> vertexSym;
    std::deque<MCGAL::Point> points;
    std::deque<MCGAL::Point> vertexPoints;
    std::deque<MCGAL::PointInt> ipoints;
    std::queue<int> gateQueue;
    gateQueue.push(seed->poolId());
    int current_version = MCGAL::BFSVersionManager::current_version++;
    int fcount = 0;
    std::vector<MCGAL::Facet*> faces;
    std::vector<MCGAL::Halfedge*> halfedges;
    while (!gateQueue.empty()) {
        int hid = gateQueue.front();
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndexInSubPool(mesh_->meshId(), hid);
        gateQueue.pop();
        if (h->isVisited(current_version) || h->isRemoved()) {
            continue;
        }
        h->setVisited(current_version);

        if (!h->face()->isVisited(current_version)) {
            MCGAL::Facet* f = h->face();
            f->setVisited(current_version);
            unsigned sym = f->isSplittable();
            facetSym.push_back(sym);
            if (sym) {
                MCGAL::Point rmved = f->getRemovedVertexPos();
                points.push_back(rmved);
                ipoints.push_back(f->getRemovedVertexPosInt());
                fcount++;
                faces.push_back(f);
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

    gateQueue.push(seed->poolId());
    current_version = MCGAL::BFSVersionManager::current_version++;
    int ecount = 0;
    while (!gateQueue.empty()) {
        int hid = gateQueue.front();
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndexInSubPool(mesh_->meshId(), hid);
        gateQueue.pop();
        if (h->isVisited(current_version) || h->isRemoved()) {
            continue;
        }
        h->setVisited(current_version);
        // if (!h->opposite()->isVisited(current_version)) {
        if (h->isAdded()) {
            edgeSym.push_back(1);
            halfedges.push_back(h);
            ecount++;
        } else {
            edgeSym.push_back(0);
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
        gateQueue.push(seed->poolId());
        current_version = MCGAL::BFSVersionManager::current_version++;
        while (!gateQueue.empty()) {
            int hid = gateQueue.front();
            MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndexInSubPool(mesh_->meshId(), hid);
            gateQueue.pop();
            if (h->isVisited(current_version) || h->isRemoved()) {
                continue;
            }
            h->setVisited(current_version);

            if (!h->vertex()->isVisited(current_version)) {
                h->vertex()->setVisited(current_version);
                if (h->vertex()->isAdjusted()) {
                    vertexSym.push_back(1);
                    vertexPoints.push_back(h->vertex()->vertexPoint());
                } else {
                    vertexSym.push_back(0);
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
#ifdef DEBUG_DUMP
    std::cout << "ecount: " << ecount << std::endl;
    std::cout << "fcount: " << fcount << std::endl;
    DEBUG_DUMP_HALFEDGES(halfedges, "./newsegment/ce_new_halfedges.txt");
    DEBUG_DUMP_FACETS(faces, "./newsegment/ce_new_facets.txt");
    DEBUG_DUMP_CONTAINER(edgeSym, "./newsegment/ce_edge_sym.txt");
    DEBUG_DUMP_CONTAINER(facetSym, "./newsegment/ce_facet_sym.txt");
#endif
    facetSymbolQueues.push_back(facetSym);
    edgeSymbolQueues.push_back(edgeSym);
    ipointQueues.push_back(ipoints);
    pointQueues.push_back(points);
    vertexSymbolQueues.push_back(vertexSym);
    vertexPointQueues.push_back(vertexPoints);

    // Phase A.1: additive RoundSymbols mirror. The legacy 6-deque state above
    // is the source of truth; we just copy it into a typed RoundSymbols so
    // Parquet V3 / hook consumers can iterate uniformly. Byte-identical path
    // is not affected.
    pmsf::RoundSymbols rs;
    pmsf::setDefaultCodecPaperRoundSymbols(
        rs, facetSym, edgeSym, vertexSym, points, vertexPoints);
    pmsf::appendBasicRemovedPointSideChannel(rs, ipoints);
    pmsf::appendBasicVertexPointSideChannel(rs, mesh_, vertexPoints);
    pmsf::appendBasicStorageSideChannel(
        rs,
        static_cast<std::int32_t>(header_.isEnableAdjustment() ? 1 : 0));
    rounds_.push_back(std::move(rs));
}


int BasicSymbolCollectOperator::exportToBuffer(char* buffer, bool isEnableQuantization, bool isEnableCompress) {
    size_t offset = 0;
    char* symboloutBlock = nullptr;
    char* pointOutBlock = nullptr;
    // 记录两个meta，symbol的大小以及point的大小
    char* symbolBuffer = new char[BUFFER_SIZE / 3];
    size_t symbolOffset = 0;
    char* pointBuffer = new char[BUFFER_SIZE / 3];
    size_t pointOffset = 0;
    int compressSymbolSize = 0;
    int compressPointSize = 0;    if (isEnableCompress) {
        serializeCharPointer(symbolBuffer, symbolOffset, symboloutBlock, compressSymbolSize);
        serializeCharPointer(pointBuffer, pointOffset, pointOutBlock, compressPointSize);
        writeInt(buffer, offset, compressSymbolSize);
        // writeInt(buffer, offset, compressPointSize);
        writeCharPointer(buffer, offset, symboloutBlock, compressSymbolSize);
        writeCharPointer(buffer, offset, pointOutBlock, compressPointSize);
    } else {
        writeInt(buffer, offset, symbolOffset);
        // writeInt(buffer, offset, compressPointSize);
        writeCharPointer(buffer, offset, symbolBuffer, symbolOffset);
        writeCharPointer(buffer, offset, pointBuffer, pointOffset);
    }
    return offset;
}

