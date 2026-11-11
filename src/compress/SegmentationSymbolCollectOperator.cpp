/*
 * @Author: koi
 * @Date: 2025-09-18 17:31:27
 * @Description:
 */
/*
 * @Author: koi
 * @Date: 2025-08-29 21:13:56
 * @Description:
 */

#include "SegmentationSymbolCollectOperator.h"
#include "core/DefaultCodecPMSFSideChannel.h"
#include "core/PMSFBoundarySideChannel.h"
#include "core/PaperGeometrySideChannel.h"
#include "SegmentationSideChannel.h"
#include "BufferUtils.h"
#include "BFSVersionManager.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Point.h"
#include "storage/PMSFSymbolStorageAdapter.h"
#include <cstddef>
#include <cstdint>
#include <deque>
#include <iostream>
#include <memory>
#include <queue>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

void requireSegmentationPaperExactIfRequested(const char* scope) {
    if (!pmsf::segmentationPaperExactRequired()) {
        return;
    }
    throw std::runtime_error(
        std::string(scope) +
        " PMSF_SEGMENTATION_REQUIRE_PMSF_EXACT=1 requested, but "
        "segmentation paper-exact replay is not proved for this path");
}

}  // namespace

void SegmentationSymbolCollectOperator::collect(MCGAL::Halfedge* seed) {
    std::deque<char> facetSym;
    std::deque<char> edgeSym;
    std::deque<MCGAL::Point> points;
    std::deque<MCGAL::PointInt> ipoints;
    std::queue<int> gateQueue;
    // int groupId = seed->groupId();
    gateQueue.push(seed->poolId());
    int current_version = MCGAL::BFSVersionManager::current_version++;
                                                                                
    std::vector<MCGAL::Facet*> faces;
    std::vector<MCGAL::Halfedge*> halfedges;
    while (!gateQueue.empty()) {
        int hid = gateQueue.front();
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndexInSubPool(mesh_->meshId(), hid);
        gateQueue.pop();
        if (h->isVisited(current_version) || h->isRemoved() || (h->vertex()->groupId() != groupId && h->end_vertex()->groupId() != groupId)) {
            continue;
        }
        h->setVisited(current_version);

        if (!h->face()->isVisited(current_version)) {
            MCGAL::Facet* f = h->face();
            f->setVisited(current_version);
            unsigned sym = f->isSplittable();
            if (!h->face()->isProcessed()) {
                facetSym.push_back(sym);
                if (sym) {
                    MCGAL::Point rmved = f->getRemovedVertexPos();
                    points.push_back(rmved);
                    ipoints.push_back(f->getRemovedVertexPosInt());
                    fcount++;
                    faces.push_back(f);
                }
            } else {
                facetSym.push_back(0);
            }
            h->face()->setProcessedFlag();
        }
        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (!hOpp->isVisited(current_version)) {
                gateQueue.push(hOpp->poolId());
            }
            if (!hIt->isVisited(current_version)) {
                gateQueue.push(hIt->poolId());
            }
            hIt = hIt->opposite()->next();
        } while (hIt != h);
    }
    gateQueue.push(seed->poolId());
    current_version = MCGAL::BFSVersionManager::current_version++;
    // int ecount = 0;
    while (!gateQueue.empty()) {
        int hid = gateQueue.front();
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndexInSubPool(mesh_->meshId(), hid);
        gateQueue.pop();
        if (h->isVisited(current_version) || h->isRemoved() || (h->vertex()->groupId() != groupId && h->end_vertex()->groupId() != groupId)) {
            continue;
        }

        h->setVisited(current_version);
        // if (!h->opposite()->isVisited(current_version)) {
            if (h->isAdded()) {
                if (!h->isProcessed()) {
                    edgeSym.push_back(1);
                    ecount++;
                } else {
                    edgeSym.push_back(0);
                }
                // edgeSym.push_back(1);
                halfedges.push_back(h);
                
            } else {
                edgeSym.push_back(0);
            }
            h->setProcessed();
        // }
        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (!hOpp->isVisited(current_version)) {
                gateQueue.push(hOpp->poolId());
            }
            if (!hIt->isVisited(current_version)) {
                gateQueue.push(hIt->poolId());
            }
            hIt = hIt->opposite()->next();
        } while (hIt != h);
    }
    segmentFacetSymbolQueues[groupIdx].push_back(facetSym);
    segmentEdgeSymbolQueues[groupIdx].push_back(edgeSym);
    segmentIpointQueues[groupIdx].push_back(ipoints);
    segmentPointQueues[groupIdx].push_back(points);

    pmsf::PMSFSymbolRecord& row = pmsf::appendPMSFSymbolRecord(
        paperTable_, 0, groupIdx, static_cast<std::int32_t>(segmentFacetSymbolQueues[groupIdx].size() - 1));
    pmsf::setPaperScalarSymbolsForRole(row.delta, pmsf::PMSFSymbolRole::Facet, pmsf::toPMSFSymbolCodeDeque(facetSym));
    pmsf::setPaperScalarSymbolsForRole(row.delta, pmsf::PMSFSymbolRole::TriangulationEdge, pmsf::toPMSFSymbolCodeDeque(edgeSym));
    pmsf::setPaperCoordinatesForRole(row.delta, pmsf::PMSFSymbolRole::RemovedVertexCoord, points);
    pmsf::appendSegmentationRemovedPointSideChannel(row.delta, ipoints);
    pmsf::appendSegmentationStorageSideChannel(
        row.delta, static_cast<std::int32_t>(groupId));
}

void SegmentationSymbolCollectOperator::collect(std::vector<MCGAL::Halfedge*> seeds) {
    if (!inited) {
        inited = true;
        segmentFacetSymbolQueues.resize(seeds.size());
        segmentEdgeSymbolQueues.resize(seeds.size());
        segmentIpointQueues.resize(seeds.size());
        segmentPointQueues.resize(seeds.size());
    }
    fcount = 0;
    ecount = 0;
    groupIdx = 0;
    for (MCGAL::Halfedge* seed : seeds) {
        groupId = groupIdx;
        collect(seed);
        
        groupIdx++;
    }
    std::cout << "ecount: " << ecount << std::endl;
    std::cout << "fcount: " << fcount << std::endl;
}

int SegmentationSymbolCollectOperator::exportToBuffer(char* buffer, bool isEnableQuantization, bool isEnableCompress) {
    if (buffer) {
        requireSegmentationPaperExactIfRequested("[Segmentation]");
    }
    // =========================================================================
    // 数据布局（支持分区级 + 轮次级随机定位）
    //
    //   [header]
    //     int32 groupNumber          // 分区数量 G
    //     int32 roundNumber          // 压缩轮次 R（每个分区的轮次相同）
    //
    //   [groupStartOffsets] G * int32
    //     每个分区在整个 payload 中的数据起点（相对 payload 起点；payload 起点 = header 之后）
    //
    //   [groupRoundOffsets] G * R * int32
    //     对每个分区 g 依次写入 R 个累计偏移（相对分区 g 数据段的起点）。
    //     offsets[g][r] = 解压完"第 r+1 个 deserializeOneRound 调用"后，reader 的 dataOffset
    //     在 r 轴上，r=0 对应第一次解压（即写入顺序中最早写入的一轮，也是压缩时的"最后一轮"）
    //
    //   [data] 每分区独立连续存储；每个分区内按 round 倒序写入（与历史一致：最后一轮在最前）
    // =========================================================================
    const int G = groupIdx;                                           // 分区数
    const int R = (G > 0) ? (int)segmentFacetSymbolQueues[0].size()   // 每分区轮次数
                          : 0;
    // 头部 + 偏移表 占用字节
    const size_t headerBytes   = 2 * sizeof(int);
    const size_t startTblBytes = (size_t)G * sizeof(int);
    const size_t roundTblBytes = (size_t)G * (size_t)R * sizeof(int);
    const size_t metaBytes     = headerBytes + startTblBytes + roundTblBytes;

    // 先把 data 写到 metaBytes 之后的位置，记录偏移
    size_t offset = metaBytes;
    std::vector<int> groupStart(G, 0);
    std::vector<std::vector<int>> roundOff(G, std::vector<int>(R, 0));

    for (int gid = 0; gid < G; ++gid) {
        const size_t groupDataStart = offset;
        groupStart[gid] = (int)(groupDataStart - metaBytes);
        for (int writeIdx = 0, j = R - 1; j >= 0; --j, ++writeIdx) {
            unsigned i_bitOffset = 0;
            std::deque<char> facetSym = segmentFacetSymbolQueues[gid][j];
            std::deque<char> edgeSym  = segmentEdgeSymbolQueues[gid][j];
            std::deque<MCGAL::Point> points = segmentPointQueues[gid][j];
            while (!facetSym.empty()) {
                char sym = facetSym.front();
                facetSym.pop_front();
                writeChar(buffer, offset, sym);
                if (sym) {
                    MCGAL::Point point = points.front();
                    points.pop_front();
                    if (isEnableQuantization) {
                        MCGAL::PointInt pi = mesh_->floatPosToInt(point);
                        for (int i = 0; i < 3; ++i) {
                            writeBits((uint32_t)pi[i], mesh_->i_nbQuantBits, buffer, i_bitOffset, offset);
                        }
                    } else {
                        writePoint(buffer, offset, point);
                    }
                }
            }
            while (!edgeSym.empty()) {
                char sym = edgeSym.front();
                edgeSym.pop_front();
                writeChar(buffer, offset, sym);
            }
            roundOff[gid][writeIdx] = (int)(offset - groupDataStart);
        }
    }

    // 写回 header 与偏移表（meta 区域）
    size_t metaOffset = 0;
    writeInt(buffer, metaOffset, G);
    writeInt(buffer, metaOffset, R);
    for (int g = 0; g < G; ++g) {
        writeInt(buffer, metaOffset, groupStart[g]);
    }
    for (int g = 0; g < G; ++g) {
        for (int r = 0; r < R; ++r) {
            writeInt(buffer, metaOffset, roundOff[g][r]);
        }
    }
    // metaOffset 此时应等于 metaBytes
    (void)isEnableCompress;
    return (int)offset;
}
