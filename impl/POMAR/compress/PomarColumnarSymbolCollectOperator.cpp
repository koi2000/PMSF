/*
 * @Author: pomar-refactor
 * @Description: Pomar 存储模块重构版 —— 符号收集器实现
 *
 * 本文件实现 PomarColumnarSymbolCollectOperator，负责：
 *   1. 基于 seed 半边 BFS 给所有顶点分配 cluster id；
 *   2. 按 LOD 从粗到细遍历 mesh，将 contextPool.operations 中的顶点分裂操作
 *      展开成 nbSplit / nbSkipsHeh / vertexSplit(2个) / geometry(2个) / geometryNormal(1个)
 *      五类符号；
 *   3. 保存到按 cluster 分桶的 deque 中，供 PomarColumnarSerializeOperator 进一步熵编码。
 *
 * 编码主体算法直接移植自旧版 PomarSerializeOperator::serialize() 的前半段，
 * 只是将"写 buffer"的动作剥离出去。
 */

#include "POMAR/compress/PomarColumnarSymbolCollectOperator.h"
#include "common/BoundedRingTraversal.h"
#include "core/PMSFBoundarySideChannel.h"
#include "core/PMSFSideChannel.h"

#include "common/RankedHalfedge.hpp"
#include "ContextPool.h"
#include "Facet.h"
#include "Halfedge.h"
#include "MeshUtils.h"
#include "PerformedOperation.h"
#include "Point.h"
#include "Vertex.h"
#include "predict/FrenetResidualCodec.h"
#include "PMSFPayloadAdapterRegistry.h"
#include "POMAR/storage/PMSFPomarPayloadAdapter.h"

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <cmath>
#include <cstdint>
#include <map>
#include <queue>
#include <utility>
#include <vector>

namespace {

unsigned liveVertexCount(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!mesh) return 0;
    unsigned count = 0;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (v && !v->isRemoved()) ++count;
    }
    return count;
}

bool pomarTopoCandidateEnabled() {
    static const bool enabled = [] {
        const char* v = std::getenv("PMSF_POMAR_TOPO_CANDIDATE");
        return v != nullptr && v[0] != '\0' && v[0] != '0';
    }();
    return enabled;
}

bool pomarDirectResidualEnabled() {
    static const bool enabled = [] {
        const char* v = std::getenv("PMSF_POMAR_DIRECT_RESIDUAL");
        return v != nullptr && v[0] != '\0' && v[0] != '0';
    }();
    return enabled;
}

}  // namespace

// ---- 框架入口 ----

void PomarColumnarSymbolCollectOperator::collect(std::vector<MCGAL::Halfedge*> seeds) {
    assert(!seeds.empty());
    collect(seeds[0]);
}

void PomarColumnarSymbolCollectOperator::computeLODPartition() {
    assert(mesh_ != nullptr);
    i_nbLODs_ = MCGAL::contextPool.operations.size();
    if (target_cluster_count_ > 0) {
        determineNbNonClusteredLODByTargetClusters(target_cluster_count_);
    } else {
        determineNbNonClusteredLOD(random_access_ratio_);
    }
}

void PomarColumnarSymbolCollectOperator::collect(MCGAL::Halfedge* seed) {
    assert(mesh_ != nullptr);

    // 注：顶点量化已经由 PomarColumnarSerializeOperator::serializeBaseMeshWithSeed 之前
    // 在 caller 侧完成，这里不再重复执行，以避免对 mesh 状态造成额外副作用。
    // mesh_->garbage_collection();
    // mesh_->quantizeVertexPositions();

    // 1. 计算 LOD 层数以及全局/局部 LOD 拆分（若尚未计算）
    if (i_nbLODs_ == 0) {
        computeLODPartition();
    }

    // 2. 给 0 号 cluster 用 seed 初始化
    clusterSeedHeh_.clear();
    clusterSeedHeh_.push_back(seed);
    i_nbClusters_ = 1;
    i_minClusterId_ = 0;

    nbSplitSymbols_.assign(1, {});
    nbSkipsHehSymbols_.assign(1, {});
    vertexSplitSymbols_.assign(1, {});
    geometrySymbols_.assign(1, {});
    geometryNormalSymbols_.assign(1, {});

    // 3. 逐 LOD 展开符号
    collectAllLODSymbols();

    const bool directResidual = pomarDirectResidualEnabled();
    paperTable_ = pmsf::buildLegacyOperationPMSFSymbolTable(
        "pomar",
        i_nbLODs_,
        i_nbClusters_,
        i_nbGlobalLOD_,
        i_nbClusteredLOD_,
        nbSplitSymbols_,
        nbSkipsHehSymbols_,
        vertexSplitSymbols_,
        geometrySymbols_,
        geometryNormalSymbols_,
        {},
        "pomar_v2",
        directResidual,
        true);
    for (auto& row : paperTable_) {
        pmsf::appendPomarV2ResidualMode(
            row.delta, directResidual, pomarTopoCandidateEnabled());
    }
}

int PomarColumnarSymbolCollectOperator::exportToBuffer(char* buffer,
                                                 bool /*enableQuantization*/,
                                                 bool /*enableCompress*/) {
    (void)buffer;
    pmsf::requireLegacyOperationPaperExactIfRequested(
        "pomar_v2",
        "[PomarSymbolCollectV2]");
    // Pomar 的熵编码 + 写文件由 PomarColumnarSerializeOperator 统一处理，
    // 此处保持空实现以满足抽象约束。
    return 0;
}

// ---- 内部实现 ----

void PomarColumnarSymbolCollectOperator::determineNbNonClusteredLOD(float p) {
    const unsigned i_nbLOD = MCGAL::contextPool.operations.size();
    const unsigned i_totalNbVertices = mesh_->size_of_vertices();
    unsigned i_nbInsertedVertices = 0;

    for (unsigned i = 0; i < i_nbLOD; ++i) {
        i_nbInsertedVertices += MCGAL::contextPool.operations[i_nbLOD - 1 - i].size();
        if ((float)i_nbInsertedVertices / i_totalNbVertices > p) {
            i_nbGlobalLOD_ = i + 1;
            i_nbClusteredLOD_ = i_nbLODs_ - i_nbGlobalLOD_;
            return;
        }
    }
    i_nbGlobalLOD_ = i_nbLOD;
    i_nbClusteredLOD_ = 0;
}

void PomarColumnarSymbolCollectOperator::determineNbNonClusteredLODByTargetClusters(
    unsigned targetClusters) {
    const unsigned i_nbLOD = MCGAL::contextPool.operations.size();
    const unsigned baseVertices = liveVertexCount(mesh_);
    if (i_nbLOD == 0 || targetClusters == 0) {
        i_nbGlobalLOD_ = 0;
        i_nbClusteredLOD_ = i_nbLOD;
        return;
    }

    unsigned insertedVertices = 0;
    unsigned bestGlobalLOD = 0;
    unsigned bestClusterCount = baseVertices;
    unsigned bestDiff = baseVertices > targetClusters
                            ? baseVertices - targetClusters
                            : targetClusters - baseVertices;

    for (unsigned i = 0; i < i_nbLOD; ++i) {
        insertedVertices += MCGAL::contextPool.operations[i_nbLOD - 1 - i].size();
        const unsigned candidateClusters = baseVertices + insertedVertices;
        const unsigned diff = candidateClusters > targetClusters
                                  ? candidateClusters - targetClusters
                                  : targetClusters - candidateClusters;
        if (diff < bestDiff ||
            (diff == bestDiff && candidateClusters <= targetClusters &&
             bestClusterCount > targetClusters)) {
            bestDiff = diff;
            bestGlobalLOD = i + 1;
            bestClusterCount = candidateClusters;
        }
    }

    i_nbGlobalLOD_ = bestGlobalLOD;
    i_nbClusteredLOD_ = i_nbLODs_ - i_nbGlobalLOD_;
}

void PomarColumnarSymbolCollectOperator::numberClusters(MCGAL::Halfedge* firstHeh) {
    for (MCGAL::Vertex* v_it : mesh_->vertices()) {
        v_it->setUnConquered();
    }

    std::queue<MCGAL::Halfedge*> hehQueue;
    hehQueue.push(firstHeh);

    unsigned i_clusterId = 1;
    while (!hehQueue.empty()) {
        MCGAL::Halfedge* hehEntry = hehQueue.front();
        hehQueue.pop();

        MCGAL::Vertex* vhTo = hehEntry->end_vertex();
        if (vhTo->isConquered()) {
            continue;
        }

        vhTo->setGroupId(i_clusterId++);
        clusterSeedHeh_.push_back(hehEntry);
        vhTo->setConquered();

        // 把邻接半边入队
        MCGAL::Halfedge* hehEntryOpp = hehEntry->opposite();
        MCGAL::Halfedge* hehIt = hehEntryOpp;
        do {
            hehIt = hehIt->find_prev();
            hehIt = hehIt->opposite();
            if (!hehIt->end_vertex()->isConquered()) {
                hehQueue.push(hehIt);
            }
        } while (hehIt != hehEntryOpp);
    }
    i_nbClusters_ = i_clusterId;
}

void PomarColumnarSymbolCollectOperator::markEdgeHalfedgesToNotCount(unsigned i_clusterId, MCGAL::Halfedge* heh) {
    if (heh->end_vertex()->groupId() == i_clusterId) {
        heh->b_dontCount = true;
    }
    MCGAL::Halfedge* hehOpp = heh->opposite();
    if (hehOpp->end_vertex()->groupId() == i_clusterId) {
        hehOpp->b_dontCount = true;
    }
}

void PomarColumnarSymbolCollectOperator::markEdgesToNotCount(unsigned i_clusterId, const MCGAL::PerformedOperation& op) {
    MCGAL::Halfedge* hehIt = op.v1->find_halfedge(op.vhPivotL);
    const MCGAL::Halfedge* hehEnd = op.v1->find_halfedge(op.vhPivotR);
    while (hehIt != hehEnd) {
        markEdgeHalfedgesToNotCount(i_clusterId, hehIt);
        hehIt = hehIt->next();
        markEdgeHalfedgesToNotCount(i_clusterId, hehIt);
        hehIt = hehIt->next();
        hehIt = hehIt->opposite();
    }
    markEdgeHalfedgesToNotCount(i_clusterId, hehIt);
}

void PomarColumnarSymbolCollectOperator::codeVertexSplit(unsigned i_clusterId,
                                                   MCGAL::Halfedge* heh,
                                                   MCGAL::PerformedOperation& op) {
    assert(op.v1 == heh->end_vertex());

    std::vector<MCGAL::Vertex*> vrNeighborsBefore;
    vrNeighborsBefore.reserve(op.v1 ? op.v1->halfedges().size() + 4 : 8);
    vrNeighborsBefore.push_back(op.v2);
    vrNeighborsBefore.push_back(op.v1);

    unsigned symb[2] = {0, 0};
    MCGAL::Halfedge* hehIt = heh;
    while (hehIt->vertex() != op.vhPivotL) {
        hehIt = hehIt->next();
        hehIt = hehIt->opposite();
        vrNeighborsBefore.push_back(hehIt->vertex());
        symb[0]++;
    }
    symb[0]--;

    hehIt = heh->opposite();
    while (hehIt->end_vertex() != op.vhPivotR) {
        hehIt = hehIt->find_prev();
        hehIt = hehIt->opposite();
        vrNeighborsBefore.push_back(hehIt->end_vertex());
        symb[1]++;
    }
    symb[1]--;

    vertexSplitSymbols_[i_clusterId].push_back(symb[0]);
    vertexSplitSymbols_[i_clusterId].push_back(symb[1]);

    // 计算重心作为预测位置
    MCGAL::Vector3 barycenter(0, 0, 0);
    for (auto* v : vrNeighborsBefore) {
        barycenter = barycenter + v->point();
    }
    barycenter = barycenter / vrNeighborsBefore.size();

    MCGAL::PointInt predPos = mesh_->floatPosToInt(barycenter);
    MCGAL::PointInt vrPos = mesh_->floatPosToInt(op.vr->point());
    MCGAL::PointInt residual = vrPos - predPos;
    MCGAL::Vector3i v3i(residual[0], residual[1], residual[2]);
    MCGAL::Vector3i toEncode = v3i;
    if (!pomarDirectResidualEnabled()) {
        auto isFinite3 = [](MCGAL::Vector3 n) {
            return std::isfinite(n[0]) && std::isfinite(n[1]) && std::isfinite(n[2]);
        };
        MCGAL::Vector3 n1raw = heh->face()->computeNormal();
        MCGAL::Vector3 n2raw = heh->opposite()->face()->computeNormal();
        if (!isFinite3(n1raw)) n1raw = MCGAL::Vector3(0, 0, 0);
        if (!isFinite3(n2raw)) n2raw = MCGAL::Vector3(0, 0, 0);
        MCGAL::Vector3 rawNormal = n1raw + n2raw;
        MCGAL::Vector3 normal;
        if (rawNormal.length() > 1e-6f) {
            normal = rawNormal.normalize();
        } else if (isFinite3(n1raw) && n1raw.length() > 1e-6f) {
            normal = n1raw.normalize();
        } else {
            normal = MCGAL::Vector3(0, 0, 1);
        }
        if (!isFinite3(normal)) {
            normal = MCGAL::Vector3(0, 0, 1);
        }

        MCGAL::Vector3 t1, t2;
        MCGAL::determineFrenetFrame(op.v1->point(), op.v2->point(), normal, t1, t2);
        if (!isFinite3(t1) || !isFinite3(t2)) {
            t1 = MCGAL::Vector3(1, 0, 0);
            t2 = MCGAL::Vector3(0, 1, 0);
            normal = MCGAL::Vector3(0, 0, 1);
        }
        toEncode = MCGAL::frenetRotation(v3i, t1, t2, normal);
    }
    const int kClamp = 1 << 14;
    for (int k = 0; k < 3; ++k) {
        int vk = toEncode[k];
        if (vk < -kClamp) vk = -kClamp;
        if (vk > kClamp) vk = kClamp;
        toEncode[k] = vk;
    }
    op.vr->setPoint(mesh_->intPosToFloat(vrPos));

    geometrySymbols_[i_clusterId].push_back(toEncode[0]);
    geometrySymbols_[i_clusterId].push_back(toEncode[1]);
    geometryNormalSymbols_[i_clusterId].push_back(toEncode[2]);

    markEdgesToNotCount(i_clusterId, op);
}

void PomarColumnarSymbolCollectOperator::collectAllLODSymbols() {
    for (unsigned i_LODrev = 0; i_LODrev < i_nbLODs_; ++i_LODrev) {
        // 当进入 clustered LOD 阶段时开始聚簇编号
        if (i_LODrev == i_nbGlobalLOD_) {
            numberClusters(clusterSeedHeh_[0]);
            i_minClusterId_ = 0;

            nbSkipsHehSymbols_.resize(i_nbClusters_);
            vertexSplitSymbols_.resize(i_nbClusters_);
            geometrySymbols_.resize(i_nbClusters_);
            geometryNormalSymbols_.resize(i_nbClusters_);
            nbSplitSymbols_.resize(i_nbClusters_);
        }

        const unsigned i_LOD = i_nbLODs_ - 1 - i_LODrev;
        std::queue<MCGAL::PerformedOperation> opToPerform;

        for (MCGAL::Vertex* v_it : mesh_->vertices()) {
            v_it->setUnConquered();
        }
        for (MCGAL::Facet* f_it : mesh_->faces()) {
            for (MCGAL::Halfedge* h_it : *f_it) {
                h_it->b_dontCount = false;
            }
        }

        for (unsigned i_clusterId = i_minClusterId_; i_clusterId < i_nbClusters_; ++i_clusterId) {
            std::queue<MCGAL::Halfedge*> hehQueue;
            hehQueue.push(clusterSeedHeh_[i_clusterId]);

            while (!hehQueue.empty()) {
                MCGAL::Halfedge* hehEntry = hehQueue.front();
                hehQueue.pop();

                MCGAL::Vertex* vhTo = hehEntry->end_vertex();
                if (vhTo->groupId() != i_clusterId || vhTo->isConquered()) {
                    continue;
                }

                // 默认复现 POMAR 的长度 rank；实验模式使用纯拓扑环顺序，
                // 避免解码端量化几何改变候选长度排序。
                std::priority_queue<RankedHalfedge> rankedHeh;
                std::vector<MCGAL::Halfedge*> topoHeh;
                MCGAL::Halfedge* hehIt = hehEntry;
                do {
                    hehIt = hehIt->next();
                    hehIt = hehIt->opposite();
                    if (pomarTopoCandidateEnabled()) {
                        topoHeh.push_back(hehIt);
                    } else {
                        rankedHeh.push(RankedHalfedge(hehIt));
                    }
                } while (hehIt != hehEntry);

                auto itLow = MCGAL::contextPool.operations[i_LOD].lower_bound(vhTo->poolId());
                auto itUp = MCGAL::contextPool.operations[i_LOD].upper_bound(vhTo->poolId());

                unsigned i_nbSplits = 0;
                unsigned i_nbSkipsHeh = 0;
                auto handleCandidate = [&](MCGAL::Halfedge* hehCandidate) {
                    if (!hehCandidate || hehCandidate->b_dontCount) {
                        return;
                    }

                    const MCGAL::Vertex* vhFrom = hehCandidate->vertex();
                    bool b_hasSplit = false;
                    for (auto it = itLow; it != itUp; ++it) {
                        MCGAL::PerformedOperation op = it->second;
                        if (op.v2 == vhFrom) {
                            opToPerform.push(op);
                            codeVertexSplit(i_clusterId, hehCandidate, op);
                            b_hasSplit = true;
                        }
                    }

                    if (b_hasSplit) {
                        nbSkipsHehSymbols_[i_clusterId].push_back(i_nbSkipsHeh);
                        i_nbSkipsHeh = 0;
                        i_nbSplits++;
                    } else {
                        i_nbSkipsHeh++;
                    }
                };
                if (pomarTopoCandidateEnabled()) {
                    for (MCGAL::Halfedge* candidate : topoHeh) {
                        handleCandidate(candidate);
                    }
                } else {
                    while (!rankedHeh.empty()) {
                        RankedHalfedge rh = rankedHeh.top();
                        rankedHeh.pop();
                        handleCandidate(rh.heh);
                    }
                }

                nbSplitSymbols_[i_clusterId].push_back(i_nbSplits);
                vhTo->setConquered();

                MCGAL::Halfedge* hehEntryOpp = hehEntry->opposite();
                hehIt = hehEntryOpp;
                do {
                    hehIt = hehIt->find_prev();
                    hehIt = hehIt->opposite();
                    hehQueue.push(hehIt);
                } while (hehIt != hehEntryOpp);
            }
        }

        // 在本 LOD 重放所有 split 把 mesh 恢复到下一轮所需状态
        assert(opToPerform.size() == MCGAL::contextPool.operations[i_LOD].size());
        while (!opToPerform.empty()) {
            MCGAL::PerformedOperation op = opToPerform.front();
            opToPerform.pop();
            MCGAL::vertexSplit(mesh_, op);
        }
        mesh_->resetState();

        // 释放当前 LOD 的操作记录
        MCGAL::contextPool.operations[i_LOD].clear();
    }
}
