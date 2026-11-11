/*
 * @Author: koi
 * @Date: 2025-09-02 10:39:21
 * @Description: 分区解压缩实现：新数据布局支持按分区 + 按轮次随机定位
 */
#include "SegmentationDeserializeOperator.h"
#include "common/BoundedRingTraversal.h"
#include "../common/DebugTool.h"
#include "BufferUtils.h"
#include "Configuration.h"
#include "Global.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "MeshUtils.h"
#include "Vertex.h"
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

std::shared_ptr<MCGAL::Mesh> SegmentationDeserializeOperator::deserializeBaseMesh() {
    // ---- 基网信息 ----
    unsigned i_nbVerticesBaseMesh = readInt(buffer_, dataOffset_);
    unsigned i_nbFacesBaseMesh = readInt(buffer_, dataOffset_);
    MCGAL::contextPool.registerPool(i_nbVerticesBaseMesh * 5, i_nbFacesBaseMesh * 15, i_nbFacesBaseMesh * 5);

    mesh_ = std::make_shared<MCGAL::Mesh>();
    mesh_->setMeshId(0);

    // 每个分区拥有的非种子顶点数量
    std::vector<int> groupSize = MCGAL::charPtrToVectorInt(buffer_, dataOffset_, header_.getGroupNumber());

    // 种子顶点（每分区 2 个）
    for (int i = 0; i < header_.getGroupNumber(); ++i) {
        MCGAL::Vertex* v0 = MCGAL::contextPool.allocateVertexFromPool(DEFAULT_MESH_ID, readPoint(buffer_, dataOffset_));
        MCGAL::Vertex* v1 = MCGAL::contextPool.allocateVertexFromPool(DEFAULT_MESH_ID, readPoint(buffer_, dataOffset_));
        v0->setGroupId(i);
        v1->setGroupId(i);
        mesh_->add_vertex(v0);
        mesh_->add_vertex(v1);
    }
    // 分区内其它顶点
    for (size_t j = 0; j < groupSize.size(); ++j) {
        for (int i = 0; i < groupSize[j]; ++i) {
            MCGAL::Vertex* vt = MCGAL::contextPool.allocateVertexFromPool(DEFAULT_MESH_ID, readPoint(buffer_, dataOffset_));
            vt->setGroupId((int)j);
            mesh_->add_vertex(vt);
        }
    }
    // 基网面
    for (unsigned i = 0; i < i_nbFacesBaseMesh; ++i) {
        std::vector<MCGAL::Vertex*> vts;
        for (int j = 0; j < 3; ++j) {
            int vertex_index = readInt(buffer_, dataOffset_);
            vts.push_back(mesh_->get_vertex(vertex_index));
        }
        MCGAL::Facet* face = MCGAL::contextPool.allocateFaceFromPool(DEFAULT_MESH_ID, vts);
        mesh_->add_face(face);
    }

    // ---- 读取新布局的元信息（header + 偏移表） ----
    // 新布局 header:
    //   int32 groupNumber
    //   int32 roundNumber
    //   G * int32 groupStartOffset  (相对 payload 起点)
    //   G * R * int32 roundOffset   (相对分区起点；累计偏移)
    groupNumber_ = readInt(buffer_, dataOffset_);
    roundNumber_ = readInt(buffer_, dataOffset_);
    if (groupNumber_ != header_.getGroupNumber()) {
        std::cerr << "[SegmentationDeserializeOperator] warning: payload groupNumber=" << groupNumber_
                  << " != header.groupNumber=" << header_.getGroupNumber() << std::endl;
    }

    groupStart_.assign(groupNumber_, 0);
    for (int g = 0; g < groupNumber_; ++g) {
        groupStart_[g] = (size_t)readInt(buffer_, dataOffset_);
    }
    groupRoundEnd_.assign(groupNumber_, std::vector<size_t>(roundNumber_, 0));
    for (int g = 0; g < groupNumber_; ++g) {
        for (int r = 0; r < roundNumber_; ++r) {
            groupRoundEnd_[g][r] = (size_t)readInt(buffer_, dataOffset_);
        }
    }
    // payloadStart 指向真正的数据段起点（紧跟 meta 之后），此时 dataOffset_ 正好位于该位置
    payloadStart_ = dataOffset_;
    // 把 groupStart / groupRoundEnd 转为"绝对字节偏移"，方便直接作为 reader 的 dataOffset
    for (int g = 0; g < groupNumber_; ++g) {
        groupStart_[g] += payloadStart_;
        for (int r = 0; r < roundNumber_; ++r) {
            groupRoundEnd_[g][r] += groupStart_[g];
        }
    }
    groupCurrentRound_.assign(groupNumber_, 0);

    // 为每个分区寻找种子 halfedge（v0->v1 方向）
    seeds.clear();
    seeds.reserve(groupNumber_);
    for (int i = 0; i < groupNumber_; ++i) {
        MCGAL::Vertex* v0 = mesh_->get_vertex(i * 2);
        MCGAL::Vertex* v1 = mesh_->get_vertex(i * 2 + 1);
        MCGAL::Halfedge* found = nullptr;
        for (MCGAL::Halfedge* hit : pmsf::boundedRing(v0)) {
            if (hit->end_vertex() == v1) { found = hit; break; }
        }
        seeds.push_back(found);
        std::string path = "./debug/seed" + std::to_string(i) + ".off";
        if (found != nullptr) {
            DEBUG_DUMP_VERTEX_ONE_RING(found->vertex(), path);
        }
    }

    reconstructOperator_->init(mesh_);
    return mesh_;
}

// ---------------------------------------------------------------------------
// 兼容旧行为：所有分区同步推进一轮
// ---------------------------------------------------------------------------
void SegmentationDeserializeOperator::deserializeOneRound() {
    for (int g = 0; g < groupNumber_; ++g) {
        deserializeOneRoundForGroup(g);
    }
}

bool SegmentationDeserializeOperator::deserializeOneRoundForGroup(int groupId) {
    if (groupId < 0 || groupId >= groupNumber_) return false;
    int cur = groupCurrentRound_[groupId];
    if (cur >= roundNumber_) return false;

    // 本轮读取范围：[startPos, endPos)
    size_t startPos = (cur == 0) ? groupStart_[groupId]
                                 : groupRoundEnd_[groupId][cur - 1];
    size_t endPos   = groupRoundEnd_[groupId][cur];

    std::vector<MCGAL::Vertex*> vertices;
    std::vector<MCGAL::Halfedge*> halfedges;
    std::vector<MCGAL::Facet*> facets;

    if (header_.isEnableQuantization()) {
        symbolReadOperator_->enableQuantization(mesh_->bboxMin, mesh_->f_quantStep);
    }
    symbolReadOperator_->groupId = groupId;
    MCGAL::Halfedge* seed = seeds[groupId];
    if (seed != nullptr && endPos > startPos) {
        size_t localOff = startPos;
        symbolReadOperator_->collect(seed, buffer_, localOff, vertices, halfedges, facets);
        // 容差：允许 reader 读完后的位置与 endPos 存在轻微差异（对于 Parquet / 位对齐情况）
        // 但理想情况下 localOff 应等于 endPos
        (void)endPos;
    }
    reconstructOperator_->reconstruct(vertices, halfedges, facets);
    groupCurrentRound_[groupId] = cur + 1;
    return true;
}

void SegmentationDeserializeOperator::decompressGroupToRound(int groupId, int targetRound) {
    if (groupId < 0 || groupId >= groupNumber_) return;
    if (targetRound < 0) targetRound = 0;
    if (targetRound > roundNumber_) targetRound = roundNumber_;
    while (groupCurrentRound_[groupId] < targetRound) {
        if (!deserializeOneRoundForGroup(groupId)) break;
    }
}

void SegmentationDeserializeOperator::decompressAllToRound(int targetRound) {
    if (targetRound < 0) targetRound = 0;
    if (targetRound > roundNumber_) targetRound = roundNumber_;
    // 按轮次展开：先所有分区推进到 1 轮、再 2 轮……保证"语义上的同步"
    for (int r = 1; r <= targetRound; ++r) {
        for (int g = 0; g < groupNumber_; ++g) {
            if (groupCurrentRound_[g] < r) {
                deserializeOneRoundForGroup(g);
            }
        }
    }
}

std::shared_ptr<MCGAL::Mesh> SegmentationDeserializeOperator::exportMesh() {
    return mesh_;
}
