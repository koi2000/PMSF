/*
 * @Author: koi
 * @Date: 2025-10-05 11:51:01
 * @Description:
 */
/*
 * @Author: koi
 * @Date: 2025-09-13 23:10:21
 * @Description:
 */
#include "PreDefinedVertexSegmentOperator.h"
#include "common/BoundedRingTraversal.h"
#include "BFSVersionManager.h"
#include "Facet.h"
#include "Global.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Vertex.h"
#include "common/Graph.h"
#include <algorithm>
#include <cassert>
#include <climits>
#include <cstdlib>
#include <map>
#include <memory>
#include <queue>
#include <random>
#include <set>
#include <vector>

#define RANDOM_SEED 3945

// 辅助函数：找到下一个边界边（基于顶点的版本）
static MCGAL::Halfedge* next_boundary_vertex(int inner, int outer, MCGAL::Halfedge* boundary) {
    MCGAL::Halfedge* nxt = boundary->next();
    if (nxt->isBoundary()) {
        if (nxt->vertex()->groupId() == inner && nxt->end_vertex()->groupId() == outer) {
            return nxt;
        }
    }
    nxt = boundary->next()->opposite()->next();
    if (nxt->isBoundary()) {
        if (nxt->vertex()->groupId() == inner && nxt->end_vertex()->groupId() == outer) {
            return nxt;
        }
    }
    for (MCGAL::Halfedge* hit : *boundary->end_vertex()) {
        if (hit->opposite()->isBoundary() && hit->isBoundary() && hit->vertex()->groupId() == inner && hit->end_vertex()->groupId() == outer) {
            return hit;
        }
    }
    return nullptr;
}

void PreDefinedVertexSegmentOperator::segment(std::shared_ptr<MCGAL::Mesh>& mesh) {
    this->mesh = mesh;
    markBoundary(mesh);
    // buildGraph();
}

void PreDefinedVertexSegmentOperator::markBoundary(std::shared_ptr<MCGAL::Mesh>& mesh) {
    // =========================================================================
    // 预分区算法（Vertex-based Region Growing + Balanced Post-merge）
    // 与 PreDefinedFacetSegmentOperator 同样的三步结构，区分点在于：
    //   - 单位是 vertex（面数相等的保证在 vertex 单位下近似成立，因为三角网格面数 ~ 2*顶点）
    //   - 扩张通过顶点邻接；种子 halfedge 要求 vertex/end_vertex 同 group
    // 目标：
    //   1) 最终分区数严格等于传入 number
    //   2) 各分区面数/顶点数尽可能接近 vsize / number
    //   3) seeds.size() == number
    // =========================================================================
    const int vsize = mesh->size_of_vertices();
    const int sampleNumber = number;
    if (sampleNumber <= 0 || vsize == 0) {
        return;
    }
    const int targetPerGroup = vsize / sampleNumber;

    std::vector<MCGAL::Vertex*> verticesVec;
    verticesVec.reserve(vsize);
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v->isRemoved()) {
            v->setGroupId(-1);
            verticesVec.push_back(v);
        }
    }
    const int alive = (int)verticesVec.size();
    std::vector<int> groupSize(sampleNumber, 0);

    const char* disableSpatialLowKEnv = std::getenv("PMSF_DISABLE_SPATIAL_LOWK_PARTITION");
    const bool useSpatialLowK = (sampleNumber == 2 || sampleNumber == 4) &&
                                !(disableSpatialLowKEnv && disableSpatialLowKEnv[0] != '\0' &&
                                  disableSpatialLowKEnv[0] != '0');

    if (useSpatialLowK) {
        // 低分区数下，按 pool 顺序采样的多源 BFS 很容易在 bunny 这类顶点顺序
        // 与空间位置弱相关的网格上产生碎片化边界，进而让 PPMC 的 boundary
        // vertex 保护几乎阻止所有删除。这里用空间最长轴递归二分，优先保证
        // 2/4 分区是连续的大块。
        std::vector<std::vector<MCGAL::Vertex*>> bins;
        bins.push_back(verticesVec);
        auto coord = [](MCGAL::Vertex* v, int axis) -> double {
            MCGAL::Point p = v->point();
            if (axis == 0) return p.x();
            if (axis == 1) return p.y();
            return p.z();
        };
        while ((int)bins.size() < sampleNumber) {
            int splitBin = -1;
            int splitAxis = 0;
            double bestExtent = -1.0;
            for (int i = 0; i < (int)bins.size(); ++i) {
                if (bins[i].size() <= 1) continue;
                double minV[3] = {coord(bins[i][0], 0), coord(bins[i][0], 1), coord(bins[i][0], 2)};
                double maxV[3] = {minV[0], minV[1], minV[2]};
                for (MCGAL::Vertex* v : bins[i]) {
                    for (int axis = 0; axis < 3; ++axis) {
                        const double c = coord(v, axis);
                        minV[axis] = std::min(minV[axis], c);
                        maxV[axis] = std::max(maxV[axis], c);
                    }
                }
                for (int axis = 0; axis < 3; ++axis) {
                    const double extent = maxV[axis] - minV[axis];
                    if (extent > bestExtent) {
                        bestExtent = extent;
                        splitBin = i;
                        splitAxis = axis;
                    }
                }
            }
            if (splitBin < 0) break;
            auto& bin = bins[splitBin];
            std::sort(bin.begin(), bin.end(), [&](MCGAL::Vertex* a, MCGAL::Vertex* b) {
                return coord(a, splitAxis) < coord(b, splitAxis);
            });
            const size_t mid = bin.size() / 2;
            std::vector<MCGAL::Vertex*> right(bin.begin() + mid, bin.end());
            bin.erase(bin.begin() + mid, bin.end());
            bins.push_back(std::move(right));
        }
        for (int g = 0; g < (int)bins.size() && g < sampleNumber; ++g) {
            for (MCGAL::Vertex* v : bins[g]) {
                v->setGroupId(g);
                groupSize[g]++;
            }
        }
        for (MCGAL::Vertex* v : verticesVec) {
            if (v->groupId() < 0) {
                v->setGroupId(0);
                groupSize[0]++;
            }
        }
    } else {

        // ---------- Step A: 均匀种子采样 ----------
        std::vector<MCGAL::Vertex*> seedVerts(sampleNumber, nullptr);
        for (int i = 0; i < sampleNumber; ++i) {
            int idx = (int)(((long long)i * alive) / sampleNumber);
            if (idx >= alive) idx = alive - 1;
            seedVerts[i] = verticesVec[idx];
        }
        {
            std::set<MCGAL::Vertex*> used;
            for (int i = 0; i < sampleNumber; ++i) {
                if (used.count(seedVerts[i])) {
                    for (MCGAL::Vertex* v : verticesVec) {
                        if (!used.count(v)) { seedVerts[i] = v; break; }
                    }
                }
                used.insert(seedVerts[i]);
            }
        }

        // ---------- Step B: 多源同步 BFS ----------
        std::vector<std::queue<int>> groupQueue(sampleNumber);
        // 使用 isProcessed() 是否已经被归属（避免重复扩张）
        // 注意：这里用 groupId != -1 作为 "已归属" 标记，不依赖 Vertex::setProcessed()
        // 因为 Vertex::setProcessed() 的语义可能被后续流程复用
        for (int g = 0; g < sampleNumber; ++g) {
            MCGAL::Vertex* v = seedVerts[g];
            v->setGroupId(g);
            groupQueue[g].push(v->poolId());
            groupSize[g] = 1;
        }
        int remaining = alive - sampleNumber;
        while (remaining > 0) {
            bool progressed = false;
            std::vector<int> order(sampleNumber);
            for (int g = 0; g < sampleNumber; ++g) order[g] = g;
            std::sort(order.begin(), order.end(), [&](int a, int b) {
                return groupSize[a] < groupSize[b];
            });
            for (int g : order) {
                if (groupQueue[g].empty()) continue;
                int vid = groupQueue[g].front();
                groupQueue[g].pop();
                MCGAL::Vertex* v = MCGAL::contextPool.getVertexByIndex(mesh->meshId(), vid);
                for (MCGAL::Halfedge* hit : pmsf::boundedRing(v)) {
                    MCGAL::Vertex* nb = hit->end_vertex();
                    if (nb->groupId() == -1 && !nb->isRemoved()) {
                        nb->setGroupId(g);
                        groupQueue[g].push(nb->poolId());
                        groupSize[g]++;
                        remaining--;
                        progressed = true;
                    }
                }
            }
            if (!progressed) {
                // 剩余孤立顶点：挂给邻居或最小分区
                for (MCGAL::Vertex* v : verticesVec) {
                    if (v->groupId() != -1 || v->isRemoved()) continue;
                    int pickGroup = -1;
                    for (MCGAL::Halfedge* hit : pmsf::boundedRing(v)) {
                        int ng = hit->end_vertex()->groupId();
                        if (ng >= 0) { pickGroup = ng; break; }
                    }
                    if (pickGroup < 0) {
                        pickGroup = 0;
                        for (int g = 1; g < sampleNumber; ++g) {
                            if (groupSize[g] < groupSize[pickGroup]) pickGroup = g;
                        }
                    }
                    v->setGroupId(pickGroup);
                    groupSize[pickGroup]++;
                    remaining--;
                }
                break;
            }
        }

        // ---------- Step C: 均衡 & 修复空分区 ----------
        for (int g = 0; g < sampleNumber; ++g) {
            if (groupSize[g] > 0) continue;
            int maxG = 0;
            for (int k = 1; k < sampleNumber; ++k) {
                if (groupSize[k] > groupSize[maxG]) maxG = k;
            }
            if (groupSize[maxG] <= 1) break;
            MCGAL::Vertex* newSeedV = nullptr;
            for (MCGAL::Vertex* v : verticesVec) {
                if (v->groupId() == maxG) { newSeedV = v; break; }
            }
            if (!newSeedV) break;
            int wantTake = std::max(1, std::min(targetPerGroup, groupSize[maxG] / 2));
            std::queue<int> bfs;
            newSeedV->setGroupId(g);
            groupSize[maxG]--;
            groupSize[g]++;
            bfs.push(newSeedV->poolId());
            int got = 1;
            while (got < wantTake && !bfs.empty()) {
                int vid = bfs.front();
                bfs.pop();
                MCGAL::Vertex* v = MCGAL::contextPool.getVertexByIndex(mesh->meshId(), vid);
                for (MCGAL::Halfedge* hit : pmsf::boundedRing(v)) {
                    if (got >= wantTake) break;
                    MCGAL::Vertex* nb = hit->end_vertex();
                    if (nb->groupId() == maxG) {
                        nb->setGroupId(g);
                        groupSize[maxG]--;
                        groupSize[g]++;
                        got++;
                        bfs.push(nb->poolId());
                    }
                }
            }
        }

        const int kMaxBalanceIters = 5;
        for (int iter = 0; iter < kMaxBalanceIters; ++iter) {
            bool changed = false;
            int maxG = 0, minG = 0;
            for (int g = 1; g < sampleNumber; ++g) {
                if (groupSize[g] > groupSize[maxG]) maxG = g;
                if (groupSize[g] < groupSize[minG]) minG = g;
            }
            if (groupSize[maxG] - groupSize[minG] <= std::max(1, targetPerGroup / 10)) break;
            for (MCGAL::Vertex* v : verticesVec) {
                if (v->groupId() != maxG) continue;
                if (groupSize[maxG] <= targetPerGroup) break;
                int bestNeighbour = -1;
                int bestSize = INT_MAX;
                for (MCGAL::Halfedge* hit : pmsf::boundedRing(v)) {
                    int ng = hit->end_vertex()->groupId();
                    if (ng != maxG && ng >= 0 && groupSize[ng] < bestSize && groupSize[ng] < targetPerGroup) {
                        bestNeighbour = ng;
                        bestSize = groupSize[ng];
                    }
                }
                if (bestNeighbour >= 0) {
                    v->setGroupId(bestNeighbour);
                    groupSize[maxG]--;
                    groupSize[bestNeighbour]++;
                    changed = true;
                }
            }
            if (!changed) break;
        }
    }

    // ---------- 构造 seeds: 每个分区找一条起止顶点同 group 的内部边 ----------
    seeds.clear();
    seeds.reserve(sampleNumber);
    std::vector<MCGAL::Halfedge*> groupSeed(sampleNumber, nullptr);
    for (MCGAL::Vertex* v : verticesVec) {
        int g = v->groupId();
        if (g < 0 || g >= sampleNumber) continue;
        if (groupSeed[g] != nullptr) continue;
        for (MCGAL::Halfedge* hit : pmsf::boundedRing(v)) {
            if (hit->vertex()->groupId() == g && hit->end_vertex()->groupId() == g) {
                groupSeed[g] = hit;
                break;
            }
        }
    }
    // 无内部边的小分区：退而求其次取任意一条 halfedge
    for (MCGAL::Vertex* v : verticesVec) {
        int g = v->groupId();
        if (g < 0 || g >= sampleNumber) continue;
        if (groupSeed[g] == nullptr) {
            for (MCGAL::Halfedge* hit : pmsf::boundedRing(v)) {
                groupSeed[g] = hit;
                break;
            }
        }
    }
    for (int g = 0; g < sampleNumber; ++g) {
        assert(groupSeed[g] != nullptr && "Vertex segment operator produced fewer groups than requested");
        seeds.push_back(groupSeed[g]);
    }
    assert((int)seeds.size() == sampleNumber && "Final seeds count must match requested number");

    // 标记边界边：连接不同组的顶点的边
    for (MCGAL::Vertex* vit : mesh->vertices()) {
        for (MCGAL::Halfedge* hit : vit->halfedges()) {
            if (hit->vertex()->groupId() != hit->end_vertex()->groupId() && hit->opposite() != nullptr) {
                hit->setBoundary();
                MCGAL::Halfedge* ohit = hit->opposite();
                ohit->setBoundary();
                hit->vertex()->setBoundary();
                hit->end_vertex()->setBoundary();

                hit->setOpposite(nullptr);

                ohit->setOpposite(nullptr);
            }
        }
    }
    for (MCGAL::Facet* f : mesh->faces()) {
        for (MCGAL::Halfedge* hit : *f) {
            if (hit->isBoundary()) {
                continue;
            }
            if (hit->vertex()->isBoundary() || hit->end_vertex()->isBoundary()) {
                hit->setBoundary();
            }
        }
    }

    // 重复边界顶点：为每个分区创建独立的副本
    // 第一步：收集所有边界顶点及其连接的分区
    std::map<MCGAL::Vertex*, std::set<int>> boundaryVertexGroups;
    for (MCGAL::Vertex* vit : mesh->vertices()) {
        if (!vit->isBoundary()) {
            continue;
        }
        std::set<int> connectedGroups;

        // 遍历所有连接到v的顶点，收集所有不同的groupId
        // 对于边界顶点v，收集所有通过半边连接到v的顶点的groupId
        // 首先添加v自己的groupId
        connectedGroups.insert(vit->groupId());

        for (MCGAL::Halfedge* hit : vit->halfedges()) {
            // 收集连接到v的顶点的groupId
            MCGAL::Vertex* other = (hit->vertex() == vit) ? hit->end_vertex() : hit->vertex();
            connectedGroups.insert(other->groupId());

            // 如果半边有face，也可以通过face的groupId来确定分区
            // if (hit->face() != nullptr && !hit->isBoundary()) {
            //     connectedGroups.insert(hit->face()->groupId());
            // }
        }

        // 只有当边界顶点连接到多个分区时才需要复制
        if (connectedGroups.size() > 1) {
            boundaryVertexGroups[vit] = connectedGroups;
        }
    }

    // 第二步：为每个边界顶点创建副本
    std::map<MCGAL::Vertex*, std::map<int, MCGAL::Vertex*>> vertexCopies;  // 原顶点 -> (groupId -> 副本顶点)
    for (auto& pair : boundaryVertexGroups) {
        MCGAL::Vertex* originalVertex = pair.first;
        const std::set<int>& groups = pair.second;

        // 为每个分区创建副本
        for (int groupId : groups) {
            MCGAL::Vertex* copy = MCGAL::contextPool.allocateVertexFromPool(mesh->meshId(), originalVertex->point());
            mesh->add_vertex(copy);
            copy->setGroupId(groupId);
            copy->setOriginVertexId(originalVertex->globalVertexId());
            copy->setBoundary();
            vertexCopies[originalVertex][groupId] = copy;
        }
    }

    for (int i = 0; i < seeds.size(); i++) {
        MCGAL::Halfedge* seed = seeds[i];
        std::queue<int> gateQueue;
        gateQueue.push(seed->poolId());
        int current_version = MCGAL::BFSVersionManager::current_version++;
        while (!gateQueue.empty()) {
            int hid = gateQueue.front();
            MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndexInSubPool(mesh->meshId(), hid);
            gateQueue.pop();
            if (h->isVisited(current_version) || h->isRemoved()) {
                continue;
            }
            h->setVisited(current_version);

            MCGAL::Halfedge* hIt = h;
            do {
                MCGAL::Halfedge* hOpp = hIt->opposite();
                if (hOpp == nullptr) {
                    if (vertexCopies.count(hIt->vertex())) {
                        hIt->setVertex(vertexCopies[hIt->vertex()][i]);
                    }
                    if (vertexCopies.count(hIt->end_vertex())) {
                        hIt->setEndVertex(vertexCopies[hIt->end_vertex()][i]);
                    }
                    hIt = hIt->next();
                    continue;
                }
                if (!hOpp->isVisited(current_version)) {
                    gateQueue.push(hOpp->poolId());
                }
                if (!hIt->isVisited(current_version)) {
                    gateQueue.push(hIt->poolId());
                }
                hIt = hIt->next();
            } while (hIt != h);
        }
    }

    // // 第三步：收集所有需要更新的半边（在修改之前先收集）
    // std::vector<MCGAL::Halfedge*> edgesToUpdate;
    // for (MCGAL::Vertex* vit : mesh->vertices()) {
    //     // 收集所有连接到边界顶点的半边
    //     for (MCGAL::Halfedge* hit : vit->halfedges()) {
    //         MCGAL::Vertex* vStart = hit->vertex();
    //         MCGAL::Vertex* vEnd = hit->end_vertex();
    //         // 如果起始顶点或结束顶点是边界顶点且有副本，则需要更新
    //         if ((vStart->isBoundary() && vertexCopies.count(vStart)) || (vEnd->isBoundary() && vertexCopies.count(vEnd))) {
    //             edgesToUpdate.push_back(hit);
    //         }
    //     }
    // }
    // // 第四步：更新所有半边，使它们指向正确的顶点副本
    // for (MCGAL::Halfedge* hit : edgesToUpdate) {
    //     MCGAL::Vertex* vStart = hit->vertex();
    //     MCGAL::Vertex* vEnd = hit->end_vertex();
    //     MCGAL::Vertex* newStart = vStart;
    //     MCGAL::Vertex* newEnd = vEnd;
    //     // 确定半边应该使用的groupId
    //     // 对于非边界边，优先使用face的groupId
    //     // 对于边界边，使用起始顶点的groupId
    //     int targetGroupId = -1;
    //     // 边界边：使用起始顶点所在的groupId
    //     targetGroupId = vStart->groupId();
    //     // 确定起始顶点应该使用的副本
    //     if (vStart->isBoundary() && vertexCopies.count(vStart)) {
    //         // 查找与targetGroupId匹配的副本
    //         if (vertexCopies[vStart].count(targetGroupId)) {
    //             newStart = vertexCopies[vStart][targetGroupId];
    //         } else {
    //             // 如果没有完全匹配的groupId，可能需要调整策略
    //             // 但这种情况应该很少发生，因为我们已经收集了所有相关的groupId
    //             // 为了安全，使用第一个可用的副本
    //             if (!vertexCopies[vStart].empty()) {
    //                 newStart = vertexCopies[vStart].begin()->second;
    //             }
    //         }
    //     }
    //     // 确定结束顶点应该使用的副本
    //     if (vEnd->isBoundary() && vertexCopies.count(vEnd)) {
    //         int endGroupId = targetGroupId;
    //         // 对于边界边，结束顶点在另一个分区
    //         if (hit->isBoundary()) {
    //             // 边界边连接两个分区，结束顶点应该在它自己的分区
    //             endGroupId = vEnd->groupId();
    //         } else {
    //             // 对于非边界边，结束顶点应该在同一个分区（与起始顶点相同）
    //             // 但如果结束顶点有多个副本，优先使用与face的groupId匹配的副本
    //             if (hit->face() != nullptr) {
    //                 endGroupId = hit->face()->groupId();
    //             } else {
    //                 endGroupId = targetGroupId;
    //             }
    //         }
    //         if (vertexCopies[vEnd].count(endGroupId)) {
    //             newEnd = vertexCopies[vEnd][endGroupId];
    //         } else {
    //             // 如果没有完全匹配的groupId，尝试使用targetGroupId
    //             if (vertexCopies[vEnd].count(targetGroupId)) {
    //                 newEnd = vertexCopies[vEnd][targetGroupId];
    //             } else if (!vertexCopies[vEnd].empty()) {
    //                 // 使用第一个可用的副本
    //                 newEnd = vertexCopies[vEnd].begin()->second;
    //             }
    //         }
    //     }
    //     // 更新半边的顶点指向
    //     if (newStart != vStart) {
    //         hit->setVertex(newStart);
    //     }
    //     if (newEnd != vEnd) {
    //         hit->setEndVertex(newEnd);
    //     }
    // }
}

void PreDefinedVertexSegmentOperator::buildGraph() {
    std::set<MCGAL::Vertex*> triPoints;
    for (MCGAL::Vertex* vit : mesh->vertices()) {
        std::set<int> cnt;
        for (MCGAL::Halfedge* hit : pmsf::boundedRing(vit)) {
            cnt.insert(hit->end_vertex()->groupId());
        }
        if (cnt.size() >= 3) {
            triPoints.insert(vit);
        }
    }
    for (MCGAL::Vertex* vit : triPoints) {
        for (MCGAL::Halfedge* hit : pmsf::boundedRing(vit)) {
            if (hit->isBoundary()) {
                MCGAL::Halfedge* st = hit;
                int inner = st->vertex()->groupId();
                int outer = st->end_vertex()->groupId();
                int dis = 0;
                do {
                    assert(st->isBoundary());
                    MCGAL::Halfedge* ed = st;
                    if (triPoints.count(st->end_vertex())) {
                        int i = 0;
                    }
                    st = next_boundary_vertex(inner, outer, st);

                    if (st == nullptr) {
                        g->addHEdge(inner, outer, {hit->vertex()->poolId(), hit->end_vertex()->poolId(), ed->end_vertex()->poolId()});
                    }
                    dis++;
                } while (st != nullptr);
            }
        }
    }
    g->setTriPoints(triPoints);
}

std::vector<MCGAL::Halfedge*> PreDefinedVertexSegmentOperator::exportSeeds() {
    return seeds;
}

std::shared_ptr<Graph> PreDefinedVertexSegmentOperator::exportGraph() {
    return g;
}
