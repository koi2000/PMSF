#include "PPMC/decompress/PPMCSegmentationDeserializeOperator.h"
#include "common/BoundedRingTraversal.h"

#include "BufferUtils.h"
#include "Configuration.h"
#include "Global.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "MeshUtils.h"
#include "Vertex.h"
#include <iostream>
#include <vector>

std::shared_ptr<MCGAL::Mesh> PPMCSegmentationDeserializeOperator::deserializeBaseMesh() {
    unsigned vertexCount = readInt(buffer_, dataOffset_);
    unsigned faceCount = readInt(buffer_, dataOffset_);
    MCGAL::contextPool.registerPool(vertexCount * 5, faceCount * 15, faceCount * 5);

    mesh_ = std::make_shared<MCGAL::Mesh>();
    mesh_->setMeshId(DEFAULT_MESH_ID);

    std::vector<int> groupSize = MCGAL::charPtrToVectorInt(buffer_, dataOffset_, header_.getGroupNumber());
    for (int i = 0; i < header_.getGroupNumber(); ++i) {
        MCGAL::Vertex* v0 = MCGAL::contextPool.allocateVertexFromPool(DEFAULT_MESH_ID, readPoint(buffer_, dataOffset_));
        MCGAL::Vertex* v1 = MCGAL::contextPool.allocateVertexFromPool(DEFAULT_MESH_ID, readPoint(buffer_, dataOffset_));
        v0->setGroupId(i);
        v1->setGroupId(i);
        mesh_->add_vertex(v0);
        mesh_->add_vertex(v1);
    }
    for (std::size_t group = 0; group < groupSize.size(); ++group) {
        for (int i = 0; i < groupSize[group]; ++i) {
            MCGAL::Vertex* vertex = MCGAL::contextPool.allocateVertexFromPool(DEFAULT_MESH_ID, readPoint(buffer_, dataOffset_));
            vertex->setGroupId(static_cast<int>(group));
            mesh_->add_vertex(vertex);
        }
    }
    for (unsigned i = 0; i < faceCount; ++i) {
        int degree = readInt(buffer_, dataOffset_);
        std::vector<MCGAL::Vertex*> vertices;
        vertices.reserve(degree);
        for (int j = 0; j < degree; ++j) {
            int vertexIndex = readInt(buffer_, dataOffset_);
            vertices.push_back(mesh_->get_vertex(vertexIndex));
        }
        MCGAL::Facet* face = MCGAL::contextPool.allocateFaceFromPool(DEFAULT_MESH_ID, vertices);
        mesh_->add_face(face);
    }

    groupNumber_ = readInt(buffer_, dataOffset_);
    roundNumber_ = readInt(buffer_, dataOffset_);
    if (groupNumber_ != header_.getGroupNumber()) {
        std::cerr << "[PPMCSegmentationDeserialize] warning: payload groupNumber="
                  << groupNumber_ << " != header.groupNumber=" << header_.getGroupNumber() << std::endl;
    }

    groupStart_.assign(groupNumber_, 0);
    for (int group = 0; group < groupNumber_; ++group) {
        groupStart_[group] = static_cast<size_t>(readInt(buffer_, dataOffset_));
    }
    groupRoundEnd_.assign(groupNumber_, std::vector<size_t>(roundNumber_, 0));
    for (int group = 0; group < groupNumber_; ++group) {
        for (int round = 0; round < roundNumber_; ++round) {
            groupRoundEnd_[group][round] = static_cast<size_t>(readInt(buffer_, dataOffset_));
        }
    }

    payloadStart_ = dataOffset_;
    for (int group = 0; group < groupNumber_; ++group) {
        groupStart_[group] += payloadStart_;
        for (int round = 0; round < roundNumber_; ++round) {
            groupRoundEnd_[group][round] += groupStart_[group];
        }
    }
    groupCurrentRound_.assign(groupNumber_, 0);

    seeds.clear();
    seeds.reserve(groupNumber_);
    for (int group = 0; group < groupNumber_; ++group) {
        MCGAL::Vertex* v0 = mesh_->get_vertex(group * 2);
        MCGAL::Vertex* v1 = mesh_->get_vertex(group * 2 + 1);
        MCGAL::Halfedge* found = nullptr;
        for (MCGAL::Halfedge* hit : pmsf::boundedRing(v0)) {
            if (hit->end_vertex() == v1) {
                found = hit;
                break;
            }
        }
        seeds.push_back(found);
    }

    reconstructOperator_->init(mesh_);
    return mesh_;
}
