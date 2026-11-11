/*
 * @Author: koi
 * @Date: 2025-09-19 15:46:28
 * @Description:
 */

#include "SegmentationSerializeOperator.h"
#include "BufferUtils.h"
#include "BFSVersionManager.h"
#include "Configuration.h"
#include "Halfedge.h"
#include "Point.h"
#include "Vertex.h"
#include "storage/PMSFUnifiedArtifactWriter.h"
#include <cstdint>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <vector>

void SegmentationSerializeOperator::init() {
    buffer_ = new char[BUFFER_SIZE];
}

void SegmentationSerializeOperator::serializeInt(int val) {
    writeInt(buffer_, dataOffset_, val);
}

void SegmentationSerializeOperator::calculateGroupSize(std::shared_ptr<MCGAL::Mesh> mesh,
                                                       std::vector<MCGAL::Vertex*>& vs,
                                                       MCGAL::Halfedge* seed,
                                                       std::set<MCGAL::Vertex*> vset,
                                                       int& idx) {
    // std::queue<int> gateQueue;
    // gateQueue.push(seed->poolId());
    // int current_version = MCGAL::BFSVersionManager::current_version++;
    // while (!gateQueue.empty()) {
    //     int hid = gateQueue.front();
    //     MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndexInSubPool(mesh->meshId(), hid);
    //     gateQueue.pop();
    //     if (h->isVisited(current_version) || h->isRemoved()) {
    //         continue;
    //     }
    //     h->setVisited(current_version);
    //     h->vertex()->setVid(idx++);
    //     if (h->vertex() != seed->vertex() || h->end_vertex() != seed->end_vertex()) {
    //         vs.push_back(h->vertex());
    //     }
    //     MCGAL::Halfedge* hIt = h;
    //     do {
    //         MCGAL::Halfedge* hOpp = hIt->opposite();
    //         if (!hOpp->isVisited(current_version) && hIt->vertex()->groupId() == hIt->end_vertex()->groupId()) {
    //             gateQueue.push(hOpp->poolId());
    //         }
    //         if (!hIt->isVisited(current_version) && hIt->vertex()->groupId() == hIt->end_vertex()->groupId()) {
    //             gateQueue.push(hIt->poolId());
    //         }
    //         hIt = hIt->next();
    //     } while (hIt != h);
    // }
    std::copy_if(mesh->vertices().begin(), mesh->vertices().end(), std::back_inserter(vs),
                 [&](MCGAL::Vertex* v) { return v->groupId() == idx && !vset.count(v); });
}

void SegmentationSerializeOperator::serializeBaseMesh(std::shared_ptr<MCGAL::Mesh> mesh) {}

void SegmentationSerializeOperator::serializeBaseMeshWithSeed(std::shared_ptr<MCGAL::Mesh> mesh, std::vector<MCGAL::Halfedge*> seeds) {
    mesh->garbage_collection();
    int i_nbVerticesBaseMesh = mesh->size_of_vertices();
    int i_nbFacesBaseMesh = mesh->size_of_facets();
    writeInt(buffer_, dataOffset_, i_nbVerticesBaseMesh);
    writeInt(buffer_, dataOffset_, i_nbFacesBaseMesh);

    // 先把seeds都写进去
    // 第一种，先写seeds，然后写每个group的数量，然后每个点
    int sidx = 0;
    int gidx = 0;
    int idx = seeds.size() * 2;
    std::vector<int> groupCnt;
    std::vector<std::vector<MCGAL::Vertex*>> vertexGroups;
    std::set<MCGAL::Vertex*> vset;
    for (MCGAL::Halfedge* seed : seeds) {
        vset.insert(seed->vertex());
        vset.insert(seed->end_vertex());
    }
    for (MCGAL::Halfedge* seed : seeds) {
        seed->vertex()->setVid(sidx++);
        seed->end_vertex()->setVid(sidx++);
        std::vector<MCGAL::Vertex*> vs;
        calculateGroupSize(mesh, vs, seed, vset, gidx);
        gidx++;
        groupCnt.push_back(vs.size());
        vertexGroups.push_back(vs);
    }
    int sum = 0;
    for (int cnt : groupCnt) {
        sum += cnt;
        writeInt(buffer_, dataOffset_, cnt);
    }
    
    for (MCGAL::Halfedge* seed : seeds) {
        writePoint(buffer_, dataOffset_, seed->vertex());
        writePoint(buffer_, dataOffset_, seed->end_vertex());
    }
    unsigned i_bitOffset = 0;
    for (std::vector<MCGAL::Vertex*>& vs : vertexGroups) {
        for (MCGAL::Vertex*& vit : vs) {
            // if (vit->vid() != -1) {
            //     std::cout << "error" << std::endl;
            //     continue;
            // }
            vit->setVid(idx++);
            writePoint(buffer_, dataOffset_, vit);
        }
    }
    for (MCGAL::Facet* fit : mesh->faces()) {
        if (fit->isRemoved()) {
            continue;
        }
        int i_faceDegree = 0;

        MCGAL::Halfedge* st = fit->proxyHalfedge();
        MCGAL::Halfedge* ed = st;
        do {
            writeInt(buffer_, dataOffset_, st->vertex()->vid());
            // writeBits((uint32_t)st->vertex()->vid(), connBit, buffer_, i_bitOffset, dataOffset_);
            st = st->next();
        } while (st != ed);
    }
}

void SegmentationSerializeOperator::serialize(std::string path) {
    int size = collector_->exportToBuffer(buffer_ + dataOffset_, enableQuantization_);
    char* outBlock = buffer_;
    int outBlockSize = dataOffset_ + size;
    if (enableCompress_) {
        serializeCharPointer(buffer_, dataOffset_ + size, outBlock, outBlockSize);
    }

    writeLengthPrefixedLegacySingleFile(path, outBlock,
                                        static_cast<std::size_t>(outBlockSize),
                                        0);
}
