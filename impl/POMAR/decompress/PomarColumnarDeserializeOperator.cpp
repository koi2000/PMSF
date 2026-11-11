/*
 * @Author: pomar-refactor
 * @Description: Pomar 存储模块重构版 —— 反序列化操作符实现
 */
#include "POMAR/decompress/PomarColumnarDeserializeOperator.h"
#include "common/BoundedRingTraversal.h"
#include "common/DebugTool.h"
#include "POMAR/decompress/PomarColumnarSymbolReadOperator.h"
#include "POMAR/reconstruct/PomarReconstructOperatorV2.h"

#include "common/RankedHalfedge.hpp"
#include "BufferUtils.h"
#include "Configuration.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "MeshUtils.h"
#include "PerformedOperation.h"
#include "Point.h"
#include "Vertex.h"
#include "predict/FrenetResidualCodec.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>

namespace {

bool pomarLegacyCandidateEnabled() {
    static const bool enabled = [] {
        const char* v = std::getenv("PMSF_POMAR_LEGACY_CANDIDATE");
        return v != nullptr && v[0] != '\0' && v[0] != '0';
    }();
    return enabled;
}

bool pomarTopoCandidateEnabled() {
    static const bool enabled = [] {
        const char* v = std::getenv("PMSF_POMAR_TOPO_CANDIDATE");
        return v != nullptr && v[0] != '\0' && v[0] != '0';
    }();
    return enabled;
}

/// 环游的**安全网**上界（D3）。
///
/// 编码端的两处环游（`PomarSerializeOperator.cpp` 的候选环 / 邻居环）是
/// **无上界**的 `do/while`，靠"绕回入口半边"终止。解码端曾给它们加了
/// `max(32, degree+8)` 的截断式守卫 —— 那**破坏了与编码端的对称性**：
/// 实测 LetterA 末轮 cluster 17 的邻居环需要 54 步（degree=17），旧守卫在 32 步
/// 就 break，少 push 了邻居 → 该顶点永不被征服 → 少 1 顶点 2 面（D3）。
///
/// 现在两侧语义一致（都走到绕回入口为止），这里保留的只是一个**远大于任何
/// 真实环**的兜底，并且**触发时必须出声**：环游被截断 = 解码发散，
/// 绝不能再像旧守卫那样静默 break。
constexpr unsigned kPomarRingSafetyLimit = 1u << 20;

bool pomarRingSafetyLimitHit(const char* which, unsigned clusterId,
                             unsigned degree, unsigned steps) {
    std::cerr << "[POMAR-V2] RING SAFETY LIMIT HIT (" << which << ") cluster="
              << clusterId << " degree=" << degree << " steps=" << steps
              << " —— 环游未绕回入口，解码结果不可信（见 DEFECT_TRACKING D3）\n";
    return true;
}

bool pomarDirectResidualEnabled() {
    static const bool enabled = [] {
        const char* v = std::getenv("PMSF_POMAR_DIRECT_RESIDUAL");
        return v != nullptr && v[0] != '\0' && v[0] != '0';
    }();
    return enabled;
}
}  // namespace

std::shared_ptr<PomarColumnarSymbolReadOperator> PomarColumnarDeserializeOperator::pomarReader() const {
    auto r = std::dynamic_pointer_cast<PomarColumnarSymbolReadOperator>(symbolReadOperator_);
    assert(r != nullptr && "PomarColumnarDeserializeOperator 要求传入 PomarColumnarSymbolReadOperator");
    return r;
}

std::shared_ptr<PomarReconstructOperatorV2> PomarColumnarDeserializeOperator::pomarReconstructor() const {
    auto r = std::dynamic_pointer_cast<PomarReconstructOperatorV2>(reconstructOperator_);
    assert(r != nullptr && "PomarColumnarDeserializeOperator 要求传入 PomarReconstructOperatorV2");
    return r;
}

// ---- deserializeBaseMesh ----

std::shared_ptr<MCGAL::Mesh> PomarColumnarDeserializeOperator::deserializeBaseMesh() {
    mesh_ = std::make_shared<MCGAL::Mesh>();
    mesh_->setMeshId(DEFAULT_MESH_ID);

    auto reader = pomarReader();
    reader->bindBuffer(buffer_, &dataOffset_);

    readHeaderAndBaseMesh();

    // 根据 header 中 cluster 数初始化（此时暂时只有 1 号 cluster）
    reader->resizeClusters(1);

    // 解码 0 号 cluster 的 i_nbGlobalLOD 层符号
    reader->decodeCluster(0, i_nbGlobalLOD_, (unsigned)mesh_->vertices().size());

    symbolIds_.assign(1, 0);
    splitIds_.assign(1, 0);
    globalLODConsumed_ = (i_nbGlobalLOD_ == 0);

    reconstructOperator_->init(mesh_);
    return mesh_;
}

void PomarColumnarDeserializeOperator::readHeaderAndBaseMesh() {
    // codec version
    unsigned i_codecVersion = readUChar(buffer_, dataOffset_);
    assert(i_codecVersion == 1);

    mesh_->bboxMin = readPoint(buffer_, dataOffset_);
    mesh_->f_quantStep = readFloat(buffer_, dataOffset_);

    unsigned i_bitOffset = 0;
    dataOffset_++;

    mesh_->i_nbQuantBits = readBits(4, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_) + 1;
    unsigned i_nbVerticesBaseMesh = readBits(16, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
    unsigned i_nbFacesBaseMesh = readBits(16, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
    unsigned i_nbBitsPerVertex = (unsigned)std::ceil(std::log((double)i_nbVerticesBaseMesh) / std::log(2.0));

    if (!MCGAL::contextPool.hasPool(mesh_->meshId())) {
        MCGAL::contextPool.registerPool(i_nbVerticesBaseMesh * 50, i_nbFacesBaseMesh * 150, i_nbFacesBaseMesh * 50);
    }

    for (unsigned i = 0; i < i_nbVerticesBaseMesh; ++i) {
        MCGAL::PointInt p;
        for (unsigned j = 0; j < 3; ++j) {
            p[j] = readBits(mesh_->i_nbQuantBits, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
        }
        MCGAL::Vertex* v = MCGAL::contextPool.allocateVertexFromPool(mesh_->meshId(), mesh_->intPosToFloat(p));
        mesh_->add_vertex(v);
    }

    MCGAL::Vertex* vhHehFirst[2] = {nullptr, nullptr};
    for (unsigned i = 0; i < i_nbFacesBaseMesh; ++i) {
        MCGAL::Vertex* vhs[3];
        for (unsigned j = 0; j < 3; ++j) {
            int idx = readBits(i_nbBitsPerVertex, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
            vhs[j] = mesh_->vertices()[idx];
        }
        MCGAL::Facet* f = MCGAL::contextPool.allocateFaceFromPool(mesh_->meshId(), {vhs[0], vhs[1], vhs[2]});
        mesh_->add_face(f);

        if (i == 0) {
            vhHehFirst[0] = vhs[0];
            vhHehFirst[1] = vhs[1];
        }
    }

    i_nbGlobalLOD_ = readBits(6, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
    i_nbClusteredLOD_ = readBits(6, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);

    if (i_bitOffset == 0) {
        dataOffset_--;
    }

    // bbox max
    mesh_->bboxMax = mesh_->bboxMin;
    for (MCGAL::Vertex* v_it : mesh_->vertices()) {
        mesh_->updateBboxMax(v_it->point());
    }
    clusterSeedHeh_.push_back(vhHehFirst[0]->find_halfedge(vhHehFirst[1]));
    }

// ---- deserializeOneRound ----

void PomarColumnarDeserializeOperator::deserializeOneRound() {
    auto reader = pomarReader();
    auto reconstructor = pomarReconstructor();

    // 阶段 1：推进一次 global LOD
    if (!globalLODConsumed_) {
                // reset mesh state
        for (MCGAL::Vertex* v_it : mesh_->vertices()) {
            v_it->setUnConquered();
        }
        for (MCGAL::Facet* f_it : mesh_->faces()) {
            for (MCGAL::Halfedge* h_it : *f_it) {
                h_it->b_dontCount = false;
            }
        }
        std::queue<MCGAL::PerformedOperation> opToPerform;
        refineCluster(0, opToPerform);
        reconstructor->applyOperations(opToPerform);

        curGlobalLOD_++;
        if (curGlobalLOD_ >= i_nbGlobalLOD_ || symbolIds_[0] >= reader->getNbSplitSymbols()[0].size()) {
            globalLODConsumed_ = true;
        }
        return;
    }

    // 阶段 2：进入 clustered 阶段前的一次性初始化
    if (i_nbClusteredLOD_ > 0 && clusterSizes_.empty()) {
                numberClusters();
        clusterVertexCounts_.assign(i_nbClusters_, 1);
        if (!clusterVertexCounts_.empty()) {
            clusterVertexCounts_[0] = 0;
        }
        
        reader->resizeClusters(i_nbClusters_);
        clusterSizes_.assign(i_nbClusters_, 0);
        readClusterSizes();
        
        // 把 cluster meta 灌入 reader
        // （readClusterSizes 内部已经把 bit widths / minMin 读到成员变量里，
        //  这里一次性同步给 reader）
        // 注：readClusterSizes 末尾已经调用过 reader->setClusterMeta()

        dataOffsetClusters_ = dataOffset_;

        // 完整解压时没有 ROI/seed 约束，所有 clustered 分块都必须回放到完整
        // LOD。旧路径从每个 base 顶点重复 BFS 计算 required LOD；在 Armadillo
        // 这类大模型上会产生巨量重复遍历，且最终仍等价于每个 cluster 解满。
        p_requiredNbLOD_.assign(i_nbClusters_, i_nbClusteredLOD_);
        if (!p_requiredNbLOD_.empty()) {
            p_requiredNbLOD_[0] = 0;
        }

        symbolIds_.resize(i_nbClusters_);
        splitIds_.resize(i_nbClusters_);

        // 逐个 cluster 解码
        for (unsigned i_clusterId = 1; i_clusterId < i_nbClusters_; ++i_clusterId) {
                        size_t offsetStart = dataOffset_;
            if (p_requiredNbLOD_[i_clusterId] > 0) {
                reader->decodeCluster(i_clusterId, p_requiredNbLOD_[i_clusterId], /*i_nbBaseVertices=*/1);
            }
            symbolIds_[i_clusterId] = 0;
            splitIds_[i_clusterId] = 0;
            dataOffset_ = offsetStart + clusterSizes_[i_clusterId];
        }
            }

    // 阶段 3：推进一次 clustered LOD
    if (curClusteredLOD_ >= i_nbClusteredLOD_) {
        return;  // 已完成
    }

    for (MCGAL::Vertex* v_it : mesh_->vertices()) {
        v_it->setUnConquered();
    }
    for (MCGAL::Facet* f_it : mesh_->faces()) {
        for (MCGAL::Halfedge* h_it : *f_it) {
            h_it->b_dontCount = false;
        }
    }

    std::queue<MCGAL::PerformedOperation> opToPerform;
    const auto& nbSplitSymbols = reader->getNbSplitSymbols();
    unsigned skippedZeroClusters = 0;
        for (unsigned i_clusterId = 1; i_clusterId < i_nbClusters_; ++i_clusterId) {
        if ((int)curClusteredLOD_ < (int)p_requiredNbLOD_[i_clusterId]) {
            const unsigned vertexCount = clusterVertexCounts_[i_clusterId];
            const unsigned symbolStart = symbolIds_[i_clusterId];
            const auto& symbols = nbSplitSymbols[i_clusterId];
            bool hasSplit = false;
            if (symbolStart + vertexCount <= symbols.size()) {
                for (unsigned i = 0; i < vertexCount; ++i) {
                    if (symbols[symbolStart + i] != 0) {
                        hasSplit = true;
                        break;
                    }
                }
                if (!hasSplit) {
                    symbolIds_[i_clusterId] += vertexCount;
                    skippedZeroClusters++;
                    continue;
                }
            }
                        const size_t beforeOps = opToPerform.size();
            refineCluster(i_clusterId, opToPerform);
            clusterVertexCounts_[i_clusterId] += static_cast<unsigned>(opToPerform.size() - beforeOps);
        }
    }
    // 末轮不变量（D3 的守卫）：编码端"每征服一个顶点写一个 nbSplitSymbol"，
    // 解码端"每征服一个顶点消费一个"。**所有轮跑完后**两者必须相等；
    // 不等 = 有顶点从未被征服，其 split 永不被施加 —— 这正是 D3 的表现
    // （末轮 cluster 17 少消费 1 个符号 → 少 1 顶点 2 面）。
    // 中间轮不检查：那时后续轮次尚未消费，累计值天然不等。
    if (curClusteredLOD_ + 1 >= i_nbClusteredLOD_) {
        long long unconsumedTotal = 0;
        // cluster 0 是 global cluster，其符号在 global 阶段消费、计数方式不同，从 1 起。
        for (unsigned c = 1; c < i_nbClusters_ && c < nbSplitSymbols.size(); ++c) {
            const long long encoded = static_cast<long long>(nbSplitSymbols[c].size());
            const long long consumed = static_cast<long long>(symbolIds_[c]);
            if (consumed != encoded) {
                unconsumedTotal += (encoded - consumed);
                std::cerr << "[POMAR-V2] UNCONSUMED cluster=" << c
                          << " consumed=" << consumed << " encoded=" << encoded
                          << " missing=" << (encoded - consumed) << '\n';
            }
        }
        if (unconsumedTotal != 0) {
            std::cerr << "[POMAR-V2] ❌ 末轮后仍有 " << unconsumedTotal
                      << " 个 split 符号未被消费 —— 解码顶点数将少于编码端"
                      << "（DEFECT_TRACKING D3）\n";
        }
    }
        reconstructor->applyOperations(opToPerform);

    curClusteredLOD_++;

    // P8/D12 对拍抓手（**默认关闭，零行为变化**）：把解码端从码流读出的
    // vertexSplitSymbols **值序列**落盘，与编码端同名文件 diff。
    //
    // 放在每轮末尾 ⇒ 文件被最后一轮覆写，留下的是"解码跑完全程之后"的序列。
    // env 未设置时该调用立即返回（不建文件、不写字节）。
    // 注意：dump 的是**本来就会算出来**的那份数据，不为 dump 额外计算任何东西。
    DUMP_SYMBOL_IF_ENV(reader->getVertexSplitSymbols(), "PMSF_POMAR_DUMP_DIR",
                       "decode_vertexSplitSymbols.txt");
}

// ---- refineCluster / decodeVertexSplit ----

void PomarColumnarDeserializeOperator::refineCluster(unsigned i_clusterId, std::queue<MCGAL::PerformedOperation>& opToPerform) {
    auto reader = pomarReader();
    auto& nbSplitSymbols = reader->getNbSplitSymbols();
    auto& nbSkipsHehSymbols = reader->getNbSkipsHehSymbols();

    std::queue<MCGAL::Halfedge*> hehQueue;
    MCGAL::Halfedge* seed = clusterSeedHeh_[i_clusterId];
    if (!seed) {
        return;
    }
    hehQueue.push(seed);

    // 复用缓冲：rankedHeh/ringCandidates/topoHeh 原先在 while 循环内**每访问
    // 一个顶点**重建一次（armadillo 解码的 malloc/free 合计 ~17% Ir 的主要
    // 来源之一）。提出循环 + 每轮 clear()，保留 capacity。
    // rankedHeh 用 vector+heap 算法复刻 std::priority_queue 的 push/top/pop
    // 语义（priority_queue 本身就是 vector+heap 操作，顺序逐位等价）。
    struct RankedHehHeap {
        std::vector<RankedHalfedge> v;
        void push(const RankedHalfedge& r) {
            v.push_back(r);
            std::push_heap(v.begin(), v.end());
        }
        const RankedHalfedge& top() const { return v.front(); }
        void pop() {
            std::pop_heap(v.begin(), v.end());
            v.pop_back();
        }
        bool empty() const { return v.empty(); }
        void clear() { v.clear(); }
    };
    RankedHehHeap rankedHeh;
    std::vector<MCGAL::Halfedge*> ringCandidates;
    std::vector<MCGAL::Halfedge*> topoHeh;

    while (!hehQueue.empty()) {
        MCGAL::Halfedge* hehEntry = hehQueue.front();
        hehQueue.pop();

        MCGAL::Vertex* vhTo = hehEntry->end_vertex();
        if (vhTo->groupId() != i_clusterId || vhTo->isConquered()) {
            continue;
        }

        MCGAL::Halfedge* hehIt = hehEntry;
        unsigned& i_symbolId = symbolIds_[i_clusterId];
        unsigned i_nbSplits = nbSplitSymbols[i_clusterId][i_symbolId];

        if (i_nbSplits > 0) {
            rankedHeh.clear();
            ringCandidates.clear();
            topoHeh.clear();
            const unsigned ringLimit = kPomarRingSafetyLimit;
            unsigned ringSteps = 0;
            do {
                hehIt = hehIt->next();
                hehIt = hehIt->opposite();
                if (pomarTopoCandidateEnabled()) {
                    topoHeh.push_back(hehIt);
                } else {
                    rankedHeh.push(RankedHalfedge(hehIt));
                }
                ringCandidates.push_back(hehIt);
                ringSteps++;
                if (ringSteps > ringLimit) {
                    pomarRingSafetyLimitHit("candidate", i_clusterId,
                                            static_cast<unsigned>(vhTo->halfedges().size()),
                                            ringSteps);
                    break;
                }
            } while (hehIt != hehEntry);

            size_t topoPos = 0;
            for (unsigned i_split = 0; i_split < i_nbSplits; ++i_split) {
                unsigned& i_splitId = splitIds_[i_clusterId];
                unsigned i_nbSkipsHehSymbol = nbSkipsHehSymbols[i_clusterId][i_splitId];

                unsigned i_skip = 0;
                if (pomarTopoCandidateEnabled()) {
                    MCGAL::Halfedge* fallbackCandidate = nullptr;
                    while (i_skip < i_nbSkipsHehSymbol && topoPos < topoHeh.size()) {
                        fallbackCandidate = topoHeh[topoPos++];
                        if (!fallbackCandidate->b_dontCount) {
                            i_skip++;
                        }
                    }
                    while (topoPos < topoHeh.size() && topoHeh[topoPos]->b_dontCount) {
                        fallbackCandidate = topoHeh[topoPos++];
                    }

                    MCGAL::Halfedge* chosen = nullptr;
                    bool recoveredCandidate = false;
                    if (topoPos < topoHeh.size()) {
                        chosen = topoHeh[topoPos++];
                    } else {
                        recoveredCandidate = true;
                    }
                    if (chosen && !isDecodeCandidateValid(chosen, i_clusterId, i_splitId)) {
                        chosen = nullptr;
                        recoveredCandidate = true;
                    }
                    while (!chosen && topoPos < topoHeh.size()) {
                        MCGAL::Halfedge* candidate = topoHeh[topoPos++];
                        if (!candidate->b_dontCount && isDecodeCandidateValid(candidate, i_clusterId, i_splitId)) {
                            chosen = candidate;
                        }
                    }
                    if (!chosen && fallbackCandidate && isDecodeCandidateValid(fallbackCandidate, i_clusterId, i_splitId)) {
                        chosen = fallbackCandidate;
                        recoveredCandidate = true;
                    }
                    if (!chosen) {
                        for (MCGAL::Halfedge* candidate : ringCandidates) {
                            if (isDecodeCandidateValid(candidate, i_clusterId, i_splitId)) {
                                chosen = candidate;
                                recoveredCandidate = true;
                                break;
                            }
                        }
                    }
                                        if (!chosen) {
                        i_splitId++;
                        continue;
                    }
                    MCGAL::PerformedOperation op = decodeVertexSplit(chosen, i_clusterId, i_splitId);
                    opToPerform.push(op);
                    i_splitId++;
                    continue;
                }

                if (pomarLegacyCandidateEnabled()) {
                    while (i_skip < i_nbSkipsHehSymbol && !rankedHeh.empty()) {
                        if (!rankedHeh.top().heh->b_dontCount) {
                            i_skip++;
                        }
                        rankedHeh.pop();
                    }
                    while (!rankedHeh.empty() && rankedHeh.top().heh->b_dontCount) {
                        rankedHeh.pop();
                    }
                    if (rankedHeh.empty()) {
                        i_splitId++;
                        continue;
                    }
                    MCGAL::PerformedOperation op =
                        decodeVertexSplit(rankedHeh.top().heh, i_clusterId, i_splitId);
                    rankedHeh.pop();
                    opToPerform.push(op);
                    i_splitId++;
                    continue;
                }

                MCGAL::Halfedge* fallbackCandidate = nullptr;
                while (i_skip < i_nbSkipsHehSymbol && !rankedHeh.empty()) {
                    fallbackCandidate = rankedHeh.top().heh;
                    if (!rankedHeh.top().heh->b_dontCount) {
                        i_skip++;
                    }
                    rankedHeh.pop();
                }
                while (!rankedHeh.empty() && rankedHeh.top().heh->b_dontCount) {
                    fallbackCandidate = rankedHeh.top().heh;
                    rankedHeh.pop();
                }

                MCGAL::Halfedge* chosen = nullptr;
                bool recoveredCandidate = false;
                if (!rankedHeh.empty()) {
                    chosen = rankedHeh.top().heh;
                    rankedHeh.pop();
                } else {
                    recoveredCandidate = true;
                }
                if (chosen && !isDecodeCandidateValid(chosen, i_clusterId, i_splitId)) {
                    chosen = nullptr;
                    recoveredCandidate = true;
                }
                while (!chosen && !rankedHeh.empty()) {
                    MCGAL::Halfedge* candidate = rankedHeh.top().heh;
                    rankedHeh.pop();
                    if (!candidate->b_dontCount && isDecodeCandidateValid(candidate, i_clusterId, i_splitId)) {
                        chosen = candidate;
                    }
                }
                if (!chosen && fallbackCandidate && isDecodeCandidateValid(fallbackCandidate, i_clusterId, i_splitId)) {
                    chosen = fallbackCandidate;
                    recoveredCandidate = true;
                }
                if (!chosen) {
                    for (MCGAL::Halfedge* candidate : ringCandidates) {
                        if (isDecodeCandidateValid(candidate, i_clusterId, i_splitId)) {
                            chosen = candidate;
                            recoveredCandidate = true;
                            break;
                        }
                    }
                }
                                if (!chosen) {
                    i_splitId++;
                    continue;
                }
                MCGAL::PerformedOperation op = decodeVertexSplit(chosen, i_clusterId, i_splitId);
                opToPerform.push(op);
                i_splitId++;
            }
        }

        vhTo->setConquered();
        MCGAL::Halfedge* hehEntryOpp = hehEntry->opposite();
        hehIt = hehEntryOpp;
        const unsigned walkLimit = kPomarRingSafetyLimit;
        unsigned walkSteps = 0;
        do {
            hehIt = hehIt->find_prev();
            hehIt = hehIt->opposite();
            walkSteps++;
            hehQueue.push(hehIt);
            if (walkSteps > walkLimit) {
                pomarRingSafetyLimitHit("neighbor", i_clusterId,
                                        static_cast<unsigned>(vhTo->halfedges().size()),
                                        walkSteps);
                break;
            }
        } while (hehIt != hehEntryOpp);

        i_symbolId++;
    }

}

bool PomarColumnarDeserializeOperator::isDecodeCandidateValid(MCGAL::Halfedge* heh,
                                                        unsigned i_clusterId,
                                                        unsigned i_splitId) {
    auto reader = pomarReader();
    auto& vertexSplitSymbols = reader->getVertexSplitSymbols();

    if (!heh || i_clusterId >= vertexSplitSymbols.size() ||
        2 * i_splitId + 1 >= vertexSplitSymbols[i_clusterId].size()) {
        return false;
    }
    MCGAL::Vertex* v1 = heh->end_vertex();
    if (!v1) {
        return false;
    }

    MCGAL::Halfedge* hehIt = heh;
    for (unsigned i = 0; i < vertexSplitSymbols[i_clusterId][2 * i_splitId] + 1; ++i) {
        if (!hehIt || !hehIt->next() || !hehIt->next()->opposite()) {
            return false;
        }
        hehIt = hehIt->next();
        hehIt = hehIt->opposite();
    }
    MCGAL::Vertex* pivotL = hehIt ? hehIt->vertex() : nullptr;
    if (!pivotL || !v1->find_halfedge(pivotL)) {
        return false;
    }

    hehIt = heh->opposite();
    for (unsigned i = 0; i < vertexSplitSymbols[i_clusterId][2 * i_splitId + 1] + 1; ++i) {
        if (!hehIt || !hehIt->find_prev() || !hehIt->find_prev()->opposite()) {
            return false;
        }
        hehIt = hehIt->find_prev();
        hehIt = hehIt->opposite();
    }
    MCGAL::Vertex* pivotR = hehIt ? hehIt->end_vertex() : nullptr;
    return pivotR && pivotR->find_halfedge(v1) != nullptr;
}

MCGAL::PerformedOperation PomarColumnarDeserializeOperator::decodeVertexSplit(MCGAL::Halfedge* heh,
                                                                         unsigned i_clusterId,
                                                                         unsigned i_splitId) {
    auto reader = pomarReader();
    auto& vertexSplitSymbols = reader->getVertexSplitSymbols();
    auto& geometrySymbols = reader->getGeometrySymbols();
    auto& geometryNormalSymbols = reader->getGeometryNormalSymbols();

    MCGAL::PerformedOperation op;
    op.v1 = heh->end_vertex();
    op.v2 = heh->vertex();

    std::vector<MCGAL::Vertex*> vrNeighborsBefore;
    vrNeighborsBefore.reserve(op.v1 ? op.v1->halfedges().size() + 4 : 8);
    vrNeighborsBefore.push_back(op.v2);
    vrNeighborsBefore.push_back(op.v1);

    MCGAL::Halfedge* hehIt = heh;
    for (unsigned i = 0; i < vertexSplitSymbols[i_clusterId][2 * i_splitId] + 1; ++i) {
        hehIt = hehIt->next();
        hehIt = hehIt->opposite();
        vrNeighborsBefore.push_back(hehIt->vertex());
    }
    op.vhPivotL = hehIt->vertex();
    assert(op.v1->find_halfedge(op.vhPivotL) != nullptr);

    hehIt = heh->opposite();
    for (unsigned i = 0; i < vertexSplitSymbols[i_clusterId][2 * i_splitId + 1] + 1; ++i) {
        hehIt = hehIt->find_prev();
        hehIt = hehIt->opposite();
        vrNeighborsBefore.push_back(hehIt->end_vertex());
    }
    op.vhPivotR = hehIt->end_vertex();

    // 计算重心 -> 预测位置
    MCGAL::Point barycenter(0, 0, 0);
    for (MCGAL::Vertex* it : vrNeighborsBefore) {
        barycenter = barycenter + it->point();
    }
    barycenter = barycenter / vrNeighborsBefore.size();

    MCGAL::Vector3i decodedFresnet(geometrySymbols[i_clusterId][2 * i_splitId],
                                   geometrySymbols[i_clusterId][2 * i_splitId + 1],
                                   geometryNormalSymbols[i_clusterId][i_splitId]);

    MCGAL::Vector3i residual = decodedFresnet;
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
        residual = MCGAL::invFrenetRotation(decodedFresnet, t1, t2, normal);
    }
    MCGAL::Vector3i predPos = mesh_->floatPosToInt(barycenter);
    MCGAL::Vector3i vrPos = predPos + residual;

    MCGAL::Vertex* vr = MCGAL::contextPool.allocateVertexFromPool(mesh_->meshId(), mesh_->intPosToFloat(vrPos));
    mesh_->add_vertex(vr);
    op.vr = vr;

    markEdgesToNotCount(i_clusterId, op);
    return op;
}

// ---- cluster meta ----

void PomarColumnarDeserializeOperator::readClusterSizes() {
    unsigned i_bitOffset = 0;
    dataOffset_++;

    unsigned i_nbBitsPerClusterSize = readBits(4, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
    for (unsigned i_clusterId = 1; i_clusterId < i_nbClusters_; ++i_clusterId) {
        clusterSizes_[i_clusterId] = readBits(i_nbBitsPerClusterSize, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
    }

    unsigned i_nbBitsNbSplits = readBits(3, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
    unsigned i_nbBitsNbSkips = readBits(3, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
    unsigned i_nbBitsVertexSplits = readBits(3, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
    unsigned i_nbBitsMinResidual = readBits(5, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
    unsigned i_nbBitsResidualRange = readBits(5, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
    unsigned i_nbBitsMinResidualNormal = readBits(5, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
    unsigned i_nbBitsResidualNormalRange = readBits(5, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);

    if (i_bitOffset == 0) {
        dataOffset_--;
    }

    int i_minMinResidual = readInt16(buffer_, dataOffset_);
    int i_minMinResidualNormal = readInt16(buffer_, dataOffset_);

    pomarReader()->setClusterMeta(i_nbBitsNbSplits, i_nbBitsNbSkips, i_nbBitsVertexSplits, i_nbBitsMinResidual, i_nbBitsResidualRange,
                                  i_nbBitsMinResidualNormal, i_nbBitsResidualNormalRange, i_minMinResidual, i_minMinResidualNormal);
}

void PomarColumnarDeserializeOperator::numberClusters() {
    for (MCGAL::Vertex* v_it : mesh_->vertices()) {
        v_it->setUnConquered();
    }

    std::queue<MCGAL::Halfedge*> hehQueue;
    hehQueue.push(clusterSeedHeh_[0]);

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

void PomarColumnarDeserializeOperator::setRequiredNbLOD(MCGAL::Vertex* vhInit, unsigned* p_requiredNbLOD) {
    std::queue<MCGAL::Vertex*> vhQueue;
    std::queue<int> requieredLODQueue;

    vhQueue.push(vhInit);
    requieredLODQueue.push(i_nbClusteredLOD_);

    while (!vhQueue.empty()) {
        MCGAL::Vertex* vh = vhQueue.front();
        unsigned requieredLOD = requieredLODQueue.front();
        vhQueue.pop();
        requieredLODQueue.pop();

        unsigned i_clusterId = vh->groupId();
        if (p_requiredNbLOD[i_clusterId] < requieredLOD) {
            p_requiredNbLOD[i_clusterId] = requieredLOD;
            unsigned nextRequiredLOD = requieredLOD > 0 ? requieredLOD - 1 : 0;
            for (MCGAL::Halfedge* vv_it : pmsf::boundedRing(vh)) {
                vhQueue.push(vv_it->end_vertex());
                requieredLODQueue.push(nextRequiredLOD);
            }
        }
    }
}

void PomarColumnarDeserializeOperator::markEdgesToNotCount(unsigned i_clusterId, const MCGAL::PerformedOperation& op) {
    MCGAL::Halfedge* hehIt = op.v1->find_halfedge(op.vhPivotL);
    const MCGAL::Halfedge* hehEnd = op.v1->find_halfedge(op.vhPivotR);
    if (!hehIt || !hehEnd) {
                return;
    }
    const unsigned walkLimit = std::max<unsigned>(32, static_cast<unsigned>(op.v1->halfedges().size()) * 4 + 16);
    unsigned walkSteps = 0;
    while (hehIt != hehEnd) {
        markEdgeHalfedgesToNotCount(i_clusterId, hehIt);
        hehIt = hehIt->next();
        markEdgeHalfedgesToNotCount(i_clusterId, hehIt);
        hehIt = hehIt->next();
        hehIt = hehIt->opposite();
        walkSteps++;
        if (walkSteps > walkLimit) {
                        return;
        }
    }
    markEdgeHalfedgesToNotCount(i_clusterId, hehIt);
}

void PomarColumnarDeserializeOperator::markEdgeHalfedgesToNotCount(unsigned i_clusterId, MCGAL::Halfedge* heh) {
    if (heh->end_vertex()->groupId() == i_clusterId) {
        heh->b_dontCount = true;
    }
    MCGAL::Halfedge* hehOpp = heh->opposite();
    if (hehOpp->end_vertex()->groupId() == i_clusterId) {
        hehOpp->b_dontCount = true;
    }
}
