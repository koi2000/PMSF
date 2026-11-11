#include "PreDefinedFacetSegmentOperator.h"
#include "common/BoundaryGraphDerive.h"
#include "common/BoundedRingTraversal.h"
#include "BFSVersionManager.h"
#include "Facet.h"
#include "Global.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "MeshUtils.h"
#include "Vertex.h"
#include "common/Graph.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <climits>
#include <cstdlib>
#include <map>
#include <memory>
#include <queue>
#include <random>
#include <set>
#include <vector>

#define RANDOM_SEED 345

void PreDefinedFacetSegmentOperator::segment(std::shared_ptr<MCGAL::Mesh>& mesh) {
    g = std::make_shared<Graph>();
    this->mesh = mesh;
    markBoundary(mesh);
    // buildGraph();
}

void PreDefinedFacetSegmentOperator::markBoundary(std::shared_ptr<MCGAL::Mesh>& mesh) {
    // =========================================================================
    // 预分区算法（Face-based Region Growing + Balanced Post-merge）
    // 目标：
    //   1) 最终分区数严格等于传入 number
    //   2) 各分区面数尽可能接近 fsize / number
    //   3) 种子 faces 的 proxyHalfedge 作为各分区的起点
    //
    // 算法分为三步：
    //   Step A: 均匀种子采样（按 poolId 等距离采 number 个种子，避免扎堆）
    //   Step B: 多源同步 BFS（所有种子同时向外扩张，天然得到 Voronoi-like 均衡分区）
    //   Step C: 均衡后处理
    //          - 若某分区远超目标面数，削掉其边界面给最小邻居
    //          - 若某分区被孤立/过小，将其整体并入最小邻居并重新生成一个新种子
    //          - 最终严格保证分区数 == number
    // =========================================================================
    const int fsize = mesh->size_of_facets();
    const int sampleNumber = number;
    if (sampleNumber <= 0 || fsize == 0) {
        return;
    }
    const int targetPerGroup = fsize / sampleNumber;

    // 重置 processed flag，避免外部残留状态干扰分区结果
    for (MCGAL::Facet* fit : mesh->faces()) {
        fit->resetProcessedFlag();
        fit->setGroupId(-1);
    }

    // ---------- Step A: 均匀种子采样 ----------
    // 按 mesh->faces() 顺序等距采样，对大多数 mesh 已能获得较均匀的空间分布；
    // 若等距采样的种子恰好落在一个已被其它种子扩张覆盖的区域，会在 Step C 中得到修复
    std::vector<MCGAL::Facet*> facesVec;
    facesVec.reserve(fsize);
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f->isRemoved()) {
            facesVec.push_back(f);
        }
    }
    const int alive = (int)facesVec.size();
    std::vector<int> groupSize(sampleNumber, 0);
    std::vector<MCGAL::Facet*> seedFaces(sampleNumber, nullptr);
    bool usedSpatialBipartition = false;

    const char* disableSpatialLowKEnv = std::getenv("PMSF_DISABLE_SPATIAL_LOWK_PARTITION");
    const bool useSpatialLowK = sampleNumber == 2 && alive > 1 &&
                                !(disableSpatialLowKEnv && disableSpatialLowKEnv[0] != '\0' &&
                                  disableSpatialLowKEnv[0] != '0');

    if (useSpatialLowK) {
        auto faceCentroid = [](MCGAL::Facet* f) {
            std::array<double, 3> c{0.0, 0.0, 0.0};
            int n = 0;
            for (MCGAL::Halfedge* h : *f) {
                MCGAL::Vertex* v = h ? h->vertex() : nullptr;
                if (!v) continue;
                c[0] += v->x();
                c[1] += v->y();
                c[2] += v->z();
                ++n;
            }
            if (n > 0) {
                c[0] /= n;
                c[1] /= n;
                c[2] /= n;
            }
            return c;
        };

        std::array<double, 3> minC{
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max()};
        std::array<double, 3> maxC{
            std::numeric_limits<double>::lowest(),
            std::numeric_limits<double>::lowest(),
            std::numeric_limits<double>::lowest()};
        std::vector<std::pair<std::array<double, 3>, MCGAL::Facet*>> ordered;
        ordered.reserve(facesVec.size());
        for (MCGAL::Facet* f : facesVec) {
            auto c = faceCentroid(f);
            for (int a = 0; a < 3; ++a) {
                minC[a] = std::min(minC[a], c[a]);
                maxC[a] = std::max(maxC[a], c[a]);
            }
            ordered.push_back({c, f});
        }
        int axis = 0;
        if (maxC[1] - minC[1] > maxC[axis] - minC[axis]) axis = 1;
        if (maxC[2] - minC[2] > maxC[axis] - minC[axis]) axis = 2;
        std::sort(ordered.begin(), ordered.end(), [&](const auto& lhs, const auto& rhs) {
            if (lhs.first[axis] != rhs.first[axis]) return lhs.first[axis] < rhs.first[axis];
            return lhs.second->poolId() < rhs.second->poolId();
        });
        const std::size_t split = ordered.size() / 2;
        for (std::size_t i = 0; i < ordered.size(); ++i) {
            const int gid = i < split ? 0 : 1;
            MCGAL::Facet* f = ordered[i].second;
            f->setGroupId(gid);
            f->setProcessedFlag();
            groupSize[gid]++;
            if (seedFaces[gid] == nullptr) seedFaces[gid] = f;
        }
        usedSpatialBipartition = seedFaces[0] != nullptr && seedFaces[1] != nullptr;
    }

    if (!usedSpatialBipartition) {
        // 等距抽样下标
        for (int i = 0; i < sampleNumber; ++i) {
            int idx = (int)(((long long)i * alive) / sampleNumber);
            if (idx >= alive) idx = alive - 1;
            seedFaces[i] = facesVec[idx];
        }
        // 去重：若等距抽样仍然撞车（极小 mesh 时），顺延选一个未被选中的面
        {
            std::set<MCGAL::Facet*> used;
            for (int i = 0; i < sampleNumber; ++i) {
                if (used.count(seedFaces[i])) {
                    for (MCGAL::Facet* f : facesVec) {
                        if (!used.count(f)) {
                            seedFaces[i] = f;
                            break;
                        }
                    }
                }
                used.insert(seedFaces[i]);
            }
        }

        // ---------- Step B: 多源同步 BFS ----------
        // groupSize[g] 记录当前分区 g 的面数；groupQueue[g] 为当前分区的扩张前沿
        std::vector<std::queue<int>> groupQueue(sampleNumber);
        for (int g = 0; g < sampleNumber; ++g) {
            MCGAL::Facet* f = seedFaces[g];
            f->setGroupId(g);
            f->setProcessedFlag();
            groupQueue[g].push(f->poolId());
            groupSize[g] = 1;
        }
        // 轮转式 BFS：每次遍历所有分区的前沿各扩一步，直到所有前沿为空或所有面被占用
        // 为了让面数较少的分区优先扩张，按 groupSize 升序处理
        int remaining = alive - sampleNumber;
        while (remaining > 0) {
            // 记录本轮是否有任何分区扩张出新面，用于检测整体死锁
            bool progressed = false;
            // 按当前分区大小升序处理：小的先扩，避免大分区吞并空间
            std::vector<int> order(sampleNumber);
            for (int g = 0; g < sampleNumber; ++g) order[g] = g;
            std::sort(order.begin(), order.end(), [&](int a, int b) {
                return groupSize[a] < groupSize[b];
            });
            for (int g : order) {
                if (groupQueue[g].empty()) continue;
                // 不再严格要求 <= target，Step C 会做均衡；这里允许适度超过以避免死锁
                int fid = groupQueue[g].front();
                groupQueue[g].pop();
                MCGAL::Facet* f = MCGAL::contextPool.getFacetByIndex(mesh->meshId(), fid);
                MCGAL::Halfedge* st = f->proxyHalfedge();
                MCGAL::Halfedge* hIt = st;
                do {
                    MCGAL::Halfedge* hOpp = hIt->opposite();
                    if (hOpp != nullptr && hOpp->face() != nullptr) {
                        MCGAL::Facet* nf = hOpp->face();
                        if (!nf->isProcessed() && !nf->isRemoved()) {
                            nf->setGroupId(g);
                            nf->setProcessedFlag();
                            groupQueue[g].push(nf->poolId());
                            groupSize[g]++;
                            remaining--;
                            progressed = true;
                        }
                    }
                    hIt = hIt->next();
                } while (hIt != st);
            }
            if (!progressed) {
                // 所有前沿都不能再扩张，但仍有面未被归属（不连通或孤岛）
                // 把剩余的未处理面按 poolId 顺序就近挂给任意邻居，若无邻居则挂给最小分区
                for (MCGAL::Facet* f : facesVec) {
                    if (f->isProcessed() || f->isRemoved()) continue;
                    int pickGroup = -1;
                    MCGAL::Halfedge* st = f->proxyHalfedge();
                    MCGAL::Halfedge* hIt = st;
                    do {
                        MCGAL::Halfedge* hOpp = hIt->opposite();
                        if (hOpp != nullptr && hOpp->face() != nullptr && hOpp->face()->isProcessed()) {
                            pickGroup = hOpp->face()->groupId();
                            break;
                        }
                        hIt = hIt->next();
                    } while (hIt != st);
                    if (pickGroup < 0) {
                        // 真正孤立的面：挂给当前最小的分区
                        pickGroup = 0;
                        for (int g = 1; g < sampleNumber; ++g) {
                            if (groupSize[g] < groupSize[pickGroup]) pickGroup = g;
                        }
                    }
                    f->setGroupId(pickGroup);
                    f->setProcessedFlag();
                    groupSize[pickGroup]++;
                    groupQueue[pickGroup].push(f->poolId());
                    remaining--;
                }
                break;
            }
        }
    }

    // ---------- Step C: 均衡后处理 ----------
    // 先修复"空分区"（极端情况下某个种子面被其它分区抢走后该分区为空）
    auto groupNeighbours = [&](int g, std::set<int>& out) {
        out.clear();
        for (MCGAL::Facet* f : facesVec) {
            if (f->groupId() != g) continue;
            MCGAL::Halfedge* st = f->proxyHalfedge();
            MCGAL::Halfedge* hIt = st;
            do {
                MCGAL::Halfedge* hOpp = hIt->opposite();
                if (hOpp != nullptr && hOpp->face() != nullptr) {
                    int ng = hOpp->face()->groupId();
                    if (ng != g && ng != -1) out.insert(ng);
                }
                hIt = hIt->next();
            } while (hIt != st);
        }
    };

    // 若存在空分区，从最大分区中"切出"一块面给它（保证最终分区数严格等于 sampleNumber）
    for (int g = 0; !usedSpatialBipartition && g < sampleNumber; ++g) {
        if (groupSize[g] > 0) continue;
        // 找最大分区
        int maxG = 0;
        for (int k = 1; k < sampleNumber; ++k) {
            if (groupSize[k] > groupSize[maxG]) maxG = k;
        }
        if (groupSize[maxG] <= 1) break;  // 几乎不可能：总面数 < sampleNumber
        // 从 maxG 中选一个 face 作为 g 的新种子并 BFS 抢占
        MCGAL::Facet* newSeedFace = nullptr;
        for (MCGAL::Facet* f : facesVec) {
            if (f->groupId() == maxG) {
                newSeedFace = f;
                break;
            }
        }
        if (!newSeedFace) break;
        // BFS 扩张直到 g 拿到大致 groupSize[maxG]/2 或 targetPerGroup
        int wantTake = std::max(1, std::min(targetPerGroup, groupSize[maxG] / 2));
        std::queue<int> bfs;
        bfs.push(newSeedFace->poolId());
        newSeedFace->setGroupId(g);
        groupSize[maxG]--;
        groupSize[g]++;
        int got = 1;
        while (got < wantTake && !bfs.empty()) {
            int fid = bfs.front();
            bfs.pop();
            MCGAL::Facet* f = MCGAL::contextPool.getFacetByIndex(mesh->meshId(), fid);
            MCGAL::Halfedge* st = f->proxyHalfedge();
            MCGAL::Halfedge* hIt = st;
            do {
                MCGAL::Halfedge* hOpp = hIt->opposite();
                if (hOpp != nullptr && hOpp->face() != nullptr) {
                    MCGAL::Facet* nf = hOpp->face();
                    if (nf->groupId() == maxG && got < wantTake) {
                        nf->setGroupId(g);
                        groupSize[maxG]--;
                        groupSize[g]++;
                        got++;
                        bfs.push(nf->poolId());
                    }
                }
                hIt = hIt->next();
            } while (hIt != st);
        }
    }

    // 均衡迭代：将远大于目标面数的分区边界面转移到最小的相邻分区
    // 迭代次数有上限，避免极端情况下的抖动
    const int kMaxBalanceIters = 5;
    for (int iter = 0; !usedSpatialBipartition && iter < kMaxBalanceIters; ++iter) {
        bool changed = false;
        // 找当前最大分区
        int maxG = 0, minG = 0;
        for (int g = 1; g < sampleNumber; ++g) {
            if (groupSize[g] > groupSize[maxG]) maxG = g;
            if (groupSize[g] < groupSize[minG]) minG = g;
        }
        if (groupSize[maxG] - groupSize[minG] <= std::max(1, targetPerGroup / 10)) break;

        // 从 maxG 的边界面中，把靠近 minG 或其它较小分区的面转移出去
        for (MCGAL::Facet* f : facesVec) {
            if (f->groupId() != maxG) continue;
            if (groupSize[maxG] <= targetPerGroup) break;
            int bestNeighbour = -1;
            int bestNeighbourSize = INT_MAX;
            MCGAL::Halfedge* st = f->proxyHalfedge();
            MCGAL::Halfedge* hIt = st;
            do {
                MCGAL::Halfedge* hOpp = hIt->opposite();
                if (hOpp != nullptr && hOpp->face() != nullptr) {
                    int ng = hOpp->face()->groupId();
                    if (ng != maxG && ng >= 0 && groupSize[ng] < bestNeighbourSize && groupSize[ng] < targetPerGroup) {
                        bestNeighbour = ng;
                        bestNeighbourSize = groupSize[ng];
                    }
                }
                hIt = hIt->next();
            } while (hIt != st);
            if (bestNeighbour >= 0) {
                f->setGroupId(bestNeighbour);
                groupSize[maxG]--;
                groupSize[bestNeighbour]++;
                changed = true;
            }
        }
        if (!changed) break;
    }

    // ---------- 构造 seeds：每个分区选一个内部面的 proxyHalfedge ----------
    seeds.clear();
    seeds.reserve(sampleNumber);
    std::vector<MCGAL::Halfedge*> groupSeed(sampleNumber, nullptr);
    for (MCGAL::Facet* f : facesVec) {
        int g = f->groupId();
        if (g < 0 || g >= sampleNumber) continue;
        if (groupSeed[g] != nullptr) continue;
        // 优先选内部面（邻居都在同一分区）
        bool interior = true;
        MCGAL::Halfedge* st = f->proxyHalfedge();
        MCGAL::Halfedge* hIt = st;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp != nullptr && hOpp->face() != nullptr && hOpp->face()->groupId() != g) {
                interior = false;
                break;
            }
            hIt = hIt->next();
        } while (hIt != st);
        if (interior) {
            groupSeed[g] = f->proxyHalfedge();
        }
    }
    // 无内部面的分区（小分区），回退到任意属于该分区的面
    for (MCGAL::Facet* f : facesVec) {
        int g = f->groupId();
        if (g < 0 || g >= sampleNumber) continue;
        if (groupSeed[g] == nullptr) groupSeed[g] = f->proxyHalfedge();
    }
    for (int g = 0; g < sampleNumber; ++g) {
        assert(groupSeed[g] != nullptr && "Segment operator produced fewer groups than requested");
        seeds.push_back(groupSeed[g]);
    }
    assert((int)seeds.size() == sampleNumber && "Final seeds count must match requested number");

    std::vector<MCGAL::Halfedge*> boundaryHalfedges;
    for (MCGAL::Facet* fit : mesh->faces()) {
        for (auto it = fit->halfedges_begin(); it != fit->halfedges_end(); it++) {
            MCGAL::Halfedge* htmp = (*it);
            htmp->vertex()->setGroupId(fit->groupId());
            htmp->end_vertex()->setGroupId(fit->groupId());
            if (htmp != nullptr && htmp->opposite() != nullptr && htmp->face()->groupId() != htmp->opposite()->face()->groupId()) {
                MCGAL::Halfedge* ohtmp = (*it)->opposite();
                htmp->setBoundary();
                htmp->vertex()->setBoundary();
                htmp->end_vertex()->setBoundary();
                ohtmp->setBoundary();
                // htmp->setOpposite(nullptr);
                // ohtmp->setOpposite(nullptr);
                boundaryHalfedges.push_back(htmp);
                boundaryHalfedges.push_back(ohtmp);
                // htmp->vertex()->setGroupId(-1);
                // htmp->end_vertex()->setGroupId(-1);
            }
        }
    }
    // Parquet V2 segmented PPMC does not consume the legacy boundary graph.
    // Skipping it avoids unbounded vertex-ring walks after partition cuts are marked.
    for (MCGAL::Halfedge* htmp : boundaryHalfedges) {
        htmp->setOpposite(nullptr);
    }
    // mesh->dumpto_oldtype("./segment/seg_1.off");
    // mesh->resetState();

    // 重复边界顶点：为每个分区创建独立的副本
    // 第一步：收集所有边界顶点及其连接的分区
    std::map<int, std::set<int>> boundaryVertexGroups;
    for (MCGAL::Vertex* vit : mesh->vertices()) {
        if (!vit->isBoundary()) {
            continue;
        }
        std::set<int> connectedGroups;

        // 遍历所有连接到v的顶点，收集所有不同的groupId
        // 对于边界顶点v，收集所有通过半边连接到v的顶点的groupId
        // 首先添加v自己的groupId
        // connectedGroups.insert(vit->groupId());

        for (MCGAL::Halfedge* hit : vit->halfedges()) {
            // 收集连接到v的顶点的groupId
            // MCGAL::Vertex* other = hit->end_vertex();
            connectedGroups.insert(hit->face()->groupId());

            // 如果半边有face，也可以通过face的groupId来确定分区
            // if (hit->face() != nullptr && !hit->isBoundary()) {
            //     connectedGroups.insert(hit->face()->groupId());
            // }
        }

        // 只有当边界顶点连接到多个分区时才需要复制
        if (connectedGroups.size() > 1) {
            boundaryVertexGroups[vit->poolId()] = connectedGroups;
        }
    }

    // 第二步：为每个边界顶点创建副本
    std::map<int, std::map<int, MCGAL::Vertex*>> vertexCopies;  // 原顶点 -> (groupId -> 副本顶点)
    for (auto& pair : boundaryVertexGroups) {
        MCGAL::Vertex* originalVertex = MCGAL::contextPool.getVertexByIndex(mesh->meshId(), pair.first);
        originalVertex->setRemoved();
        const std::set<int>& groups = pair.second;

        // 为每个分区创建副本
        for (int groupId : groups) {
            MCGAL::Vertex* copy = MCGAL::contextPool.allocateVertexFromPool(mesh->meshId(), originalVertex->point());
            mesh->add_vertex(copy);
            copy->setGroupId(groupId);
            copy->setOriginVertexId(originalVertex->globalVertexId());
            copy->setBoundary();
            vertexCopies[originalVertex->poolId()][groupId] = copy;
        }
    }

    auto copyForGroup = [&](MCGAL::Vertex* v, int groupId) -> MCGAL::Vertex* {
        if (v == nullptr || groupId < 0) {
            return v;
        }
        const int originalId = v->globalVertexId();
        auto vertexCopyIt = vertexCopies.find(originalId);
        if (vertexCopyIt == vertexCopies.end()) {
            return v;
        }
        auto groupCopyIt = vertexCopyIt->second.find(groupId);
        if (groupCopyIt != vertexCopyIt->second.end() && groupCopyIt->second != nullptr) {
            return groupCopyIt->second;
        }

        MCGAL::Vertex* originalVertex = MCGAL::contextPool.getVertexByIndex(mesh->meshId(), originalId);
        MCGAL::Vertex* copy = MCGAL::contextPool.allocateVertexFromPool(mesh->meshId(), originalVertex->point());
        mesh->add_vertex(copy);
        copy->setGroupId(groupId);
        copy->setOriginVertexId(originalVertex->globalVertexId());
        copy->setBoundary();
        vertexCopyIt->second[groupId] = copy;
        return copy;
    };

    for (MCGAL::Facet* face : facesVec) {
        if (face == nullptr || face->isRemoved()) {
            continue;
        }
        const int groupId = face->groupId();
        for (MCGAL::Halfedge* hit : *face) {
            MCGAL::Vertex* v0 = copyForGroup(hit->vertex(), groupId);
            MCGAL::Vertex* v1 = copyForGroup(hit->end_vertex(), groupId);
            if (v0 != hit->vertex()) {
                hit->setVertex(v0);
            }
            if (v1 != hit->end_vertex()) {
                hit->setEndVertex(v1);
            }
        }
    }

    for (MCGAL::Vertex* vertex : mesh->vertices()) {
        vertex->halfedges().clear();
        vertex->setProxyHalfedge(nullptr);
    }
    for (MCGAL::Facet* face : facesVec) {
        if (face == nullptr || face->isRemoved()) {
            continue;
        }
        for (MCGAL::Halfedge* hit : *face) {
            if (hit->vertex() == nullptr) {
                continue;
            }
            hit->vertex()->halfedges().push_back(hit);
            if (hit->vertex()->proxyHalfedge() == nullptr) {
                hit->vertex()->setProxyHalfedge(hit);
            }
        }
    }

    for (MCGAL::Facet* face : mesh->faces()) {
        if (face == nullptr || face->isRemoved()) {
            continue;
        }
        for (MCGAL::Halfedge* hit : *face) {
            assert(!hit->vertex()->isRemoved());
            assert(!hit->end_vertex()->isRemoved());
        }
    }
    for (MCGAL::Vertex* vertex : mesh->vertices()) {
        if (vertex->isRemoved()) {
            continue;
        }
        if (vertex->poolId() == 35518) {
            int i = 0;
        }
        assert(vertex->proxyHalfedge() != nullptr);
        for (MCGAL::Halfedge* hit : vertex->halfedges()) {
            assert(!hit->vertex()->isRemoved());
            assert(!hit->end_vertex()->isRemoved());
        }
    }
    // The graph is intentionally left empty in this path, so there is nothing to refactor.
}

void PreDefinedFacetSegmentOperator::buildGraph() {
    std::set<MCGAL::Vertex*> triPoints;
    for (MCGAL::Vertex* vit : mesh->vertices()) {
        std::set<int> cnt;
        for (MCGAL::Halfedge* hit : pmsf::boundedRing(vit)) {
            cnt.insert(hit->face()->groupId());
        }
        if (cnt.size() >= 3) {
            triPoints.insert(vit);
        }
    }
    for (MCGAL::Vertex* vit : triPoints) {
        for (MCGAL::Halfedge* hit : pmsf::boundedRing(vit)) {
            if (hit->isBoundary()) {
                // if (triPoints.count(hit->vertex) && triPoints.count(hit->end_vertex)) {
                //     g.addHEdge(hit->face->groupId, {hit->vertex->id, hit->end_vertex->id, hit->opposite->face->groupId, hit->end_vertex->id});
                //     continue;
                // }
                MCGAL::Halfedge* st = hit;
                int inner = st->face()->groupId();
                int outer = st->opposite()->face()->groupId();
                int dis = 0;
                do {
                    assert(st->isBoundary());
                    MCGAL::Halfedge* ed = st;
                    if (triPoints.count(st->end_vertex())) {
                        int i = 0;
                    }
                    st = next_boundary(inner, outer, st);

                    if (st == nullptr) {
                        g->addHEdge(inner, outer, {hit->vertex()->poolId(), hit->end_vertex()->poolId(), ed->end_vertex()->poolId()});
                    }
                    dis++;
                } while (st != nullptr);
                // g.addEdge({hit->vertex->id, hit->end_vertex->id, hit->opposite->face->groupId, st->end_vertex->id}, {st->end_vertex->id,
                // st->vertex->id, hit->face->groupId, hit->vertex->id});
            }
        }
    }
    g->setTriPoints(triPoints);
    if (std::getenv("PMSF_BOUNDARY_GRAPH_SCHEME") &&
        std::string(std::getenv("PMSF_BOUNDARY_GRAPH_SCHEME")) == "start_only") {
        g->setBoundaryScheme(BoundaryGraphScheme::StartOnly);
    }
    if (std::getenv("PMSF_BOUNDARY_GRAPH_VALIDATE_START_ONLY")) {
        int checked = 0;
        int mismatched = 0;
        for (auto& [groupA, neighbors] : g->graph()) {
            for (auto& [groupB, nodes] : neighbors) {
                for (const Node& legacy : nodes) {
                    Node derived;
                    ++checked;
                    if (!pmsf::deriveBoundaryTriple(mesh, groupA, groupB, legacy.st, derived) ||
                        derived.ed != legacy.ed || derived.stop != legacy.stop) {
                        ++mismatched;
                        if (mismatched <= 8) {
                            std::cerr << "[boundary-start-only] mismatch A=" << groupA
                                      << " B=" << groupB << " st=" << legacy.st
                                      << " legacy=(" << legacy.ed << "," << legacy.stop
                                      << ") derived=(" << derived.ed << "," << derived.stop
                                      << ")" << std::endl;
                        }
                    }
                }
            }
        }
        std::cerr << "[boundary-start-only] checked=" << checked
                  << " mismatched=" << mismatched << std::endl;
    }
}

std::vector<MCGAL::Halfedge*> PreDefinedFacetSegmentOperator::exportSeeds() {
    return seeds;
}

std::shared_ptr<Graph> PreDefinedFacetSegmentOperator::exportGraph() {
    return g;
}
