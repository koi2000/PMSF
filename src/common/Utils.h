#pragma once

#include "Configuration.h"
#include "Facet.h"
#include "Global.h"
#include "Halfedge.h"
#include "Vertex.h"
#include "common/Graph.h"
#include <cassert>
#include <memory>
#include <vector>

static void parseGraph(std::shared_ptr<Graph> g, std::vector<MCGAL::Vertex*>& vertices, std::vector<MCGAL::Halfedge*>& halfedges) {
    for (auto& [groupId, mp] : g->graph()) {
        for (auto& [neighbour, nodes] : mp) {
            for (auto& node : nodes) {
                // node.st = vertexCopyMap[node.st][groupId]->poolId();
                // node.ed = vertexCopyMap[node.ed][groupId]->poolId();
                // node.stop = vertexCopyMap[node.stop][groupId]->poolId();
                MCGAL::Vertex* st = MCGAL::contextPool.getVertexByIndex(DEFAULT_MESH_ID, node.st);
                MCGAL::Vertex* ed = MCGAL::contextPool.getVertexByIndex(DEFAULT_MESH_ID, node.ed);
                MCGAL::Vertex* stop = MCGAL::contextPool.getVertexByIndex(DEFAULT_MESH_ID, node.stop);
                MCGAL::Halfedge* hit = st->find_halfedge(ed);
                assert(hit != nullptr);
                halfedges.push_back(hit);
                vertices.push_back(stop);
            }
        }
    }
}