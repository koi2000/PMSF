// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "Halfedge.h"
#include "Vertex.h"
#include "Facet.h"

#include <algorithm>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace pmsf {

/// Rooted combinatorial-map numbering, independent of geometry, allocation ids
/// and container order. The seed's direction and mesh orientation are the only
/// ordering inputs. Works on the component reachable by next/opposite walks.
struct ConquestLayerTopology {
    std::vector<MCGAL::Vertex*> vertices;
    std::unordered_map<std::uint32_t,std::size_t> poolToIndex;
    std::vector<std::vector<std::size_t>> adjacency;
    std::vector<bool> meshBoundary;
    std::uint64_t signature = 0;
};

inline ConquestLayerTopology snapshotConquestLayerTopology(MCGAL::Halfedge* seed) {
    if (!seed || !seed->face() || seed->face()->isRemoved()) {
        throw std::runtime_error("layer topology: seed has no live face");
    }
    ConquestLayerTopology out;
    std::unordered_set<MCGAL::Halfedge*> seen;
    std::deque<MCGAL::Halfedge*> pending{seed};
    auto vertexIndex=[&](MCGAL::Vertex* vertex) {
        if (!vertex || vertex->isRemoved()) throw std::runtime_error("layer topology: dead/null vertex");
        const auto id=static_cast<std::uint32_t>(vertex->poolId());
        const auto it=out.poolToIndex.find(id);
        if (it!=out.poolToIndex.end()) return it->second;
        const auto index=out.vertices.size();
        out.poolToIndex.emplace(id,index);out.vertices.push_back(vertex);
        out.adjacency.emplace_back();out.meshBoundary.push_back(false);
        return index;
    };
    while (!pending.empty()) {
        auto* edge=pending.front();pending.pop_front();
        if (!edge || !edge->face() || edge->face()->isRemoved() || !seen.insert(edge).second) continue;
        const auto a=vertexIndex(edge->vertex()),b=vertexIndex(edge->end_vertex());
        if (a==b) throw std::runtime_error("layer topology: zero-length topological edge");
        out.adjacency[a].push_back(b);out.adjacency[b].push_back(a);
        if (!edge->next()) throw std::runtime_error("layer topology: broken face loop");
        pending.push_back(edge->next());
        auto* opposite=edge->opposite();
        if (!opposite || !opposite->face() || opposite->face()->isRemoved()) {
            out.meshBoundary[a]=out.meshBoundary[b]=true;
        } else pending.push_back(edge->opposite());
    }
    // FNV over the rooted adjacency, never over pointer values or pool ids.
    std::uint64_t hash=1469598103934665603ull;
    auto fold=[&](std::uint64_t value) {for(int k=0;k<8;++k){hash^=(value>>(8*k))&255u;hash*=1099511628211ull;}};
    fold(out.vertices.size());
    for (std::size_t i=0;i<out.adjacency.size();++i) {
        auto& neighbors=out.adjacency[i];std::sort(neighbors.begin(),neighbors.end());
        neighbors.erase(std::unique(neighbors.begin(),neighbors.end()),neighbors.end());
        fold(i);fold(neighbors.size());for(auto id:neighbors)fold(id);fold(out.meshBoundary[i]);
    }
    out.signature=hash;
    return out;
}
} // namespace pmsf
