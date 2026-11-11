// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace pmsf {

struct TopologyBisectionResult {
    std::vector<std::vector<std::size_t>> clusters; // data-point indices, not vertex ids
    std::vector<std::size_t> clusterForData;
    std::size_t splitCount=0;
    std::size_t closedBoundaryFallbacks=0;
    std::uint64_t signature=0;
};

/// Connectivity-only recursive two-seed region growth. Regions are domains of
/// graph vertices; `dataVertices` is the set being coded at this refinement
/// layer. Only that set counts against the threshold. Surviving vertices remain
/// traversal support for graph distances. No geometry or side memberships are
/// used; callers must provide the same rooted topology numbering on both sides.
inline TopologyBisectionResult recursiveTopologyBisection(
    const std::vector<std::vector<std::size_t>>& graph,
    const std::vector<bool>& meshBoundary,
    const std::vector<std::size_t>& dataVertices,
    std::size_t threshold,std::uint64_t seed) {
    if (!threshold || graph.size()!=meshBoundary.size()) throw std::runtime_error("topology bisection: invalid threshold/boundary size");
    const auto n=graph.size();
    std::vector<std::size_t> dataAt(n,std::numeric_limits<std::size_t>::max());
    for(std::size_t i=0;i<dataVertices.size();++i) {
        const auto vertex=dataVertices[i];
        if(vertex>=n || dataAt[vertex]!=std::numeric_limits<std::size_t>::max()) throw std::runtime_error("topology bisection: duplicated/out-of-range data vertex");
        dataAt[vertex]=i;
    }
    for(std::size_t a=0;a<n;++a)for(auto b:graph[a]) {
        if(b>=n||b==a)throw std::runtime_error("topology bisection: invalid adjacency");
        if(std::find(graph[b].begin(),graph[b].end(),a)==graph[b].end()) throw std::runtime_error("topology bisection: non-symmetric adjacency");
    }
    TopologyBisectionResult out;out.clusterForData.resize(dataVertices.size());
    if(dataVertices.empty())return out;
    std::uint64_t randomState=seed;
    auto random=[&]() {
        std::uint64_t value=(randomState+=0x9e3779b97f4a7c15ull);
        value=(value^(value>>30))*0xbf58476d1ce4e5b9ull;
        value=(value^(value>>27))*0x94d049bb133111ebull;
        return value^(value>>31);
    };
    std::vector<std::vector<std::size_t>> pending(1);
    pending.front().resize(n);std::iota(pending.front().begin(),pending.front().end(),0);
    std::vector<std::uint64_t> regionMark(n,0);
    std::vector<int> owner(n,-1);std::uint64_t generation=0;
    while(!pending.empty()) {
        auto region=std::move(pending.back());pending.pop_back();
        std::size_t count=0;
        for(auto v:region) count+=dataAt[v]!=std::numeric_limits<std::size_t>::max();
        if(!count)continue;
        if(count<=threshold) {
            std::vector<std::size_t> members;
            for(auto v:region)if(dataAt[v]!=std::numeric_limits<std::size_t>::max())members.push_back(dataAt[v]);
            std::sort(members.begin(),members.end());
            const auto cluster=out.clusters.size();
            for(auto i:members)out.clusterForData[i]=cluster;
            out.clusters.push_back(std::move(members));
            continue;
        }
        ++generation;
        for(auto v:region){regionMark[v]=generation;owner[v]=-1;}
        std::vector<std::size_t> boundary;
        for(auto v:region) {
            bool isBoundary=meshBoundary[v];
            for(auto neighbor:graph[v]) if(regionMark[neighbor]!=generation){isBoundary=true;break;}
            if(isBoundary)boundary.push_back(v);
        }
        // The paper does not specify boundary seeds for a closed whole mesh.
        // An explicit deterministic fallback draws from the whole domain.
        if(boundary.size()<2){boundary=region;++out.closedBoundaryFallbacks;}
        const auto first=static_cast<std::size_t>(random()%boundary.size());
        auto second=static_cast<std::size_t>(random()%(boundary.size()-1));
        if(second>=first)++second;
        const auto a=boundary[first],b=boundary[second];
        std::vector<std::size_t> frontier[2]={{a},{b}};
        owner[a]=0;owner[b]=1;
        while(!frontier[0].empty()||!frontier[1].empty()) {
            for(int side=0;side<2;++side) {
                std::vector<std::size_t> next;
                // Each growth step is a complete topological distance shell.
                // Canonical vertex order resolves collisions deterministically.
                std::sort(frontier[side].begin(),frontier[side].end());
                for(auto v:frontier[side])for(auto neighbor:graph[v]) {
                    if(regionMark[neighbor]==generation&&owner[neighbor]<0){owner[neighbor]=side;next.push_back(neighbor);}
                }
                frontier[side]=std::move(next);
            }
        }
        std::vector<std::size_t> children[2];
        for(auto v:region) {
            if(owner[v]<0)throw std::runtime_error("topology bisection: disconnected domain cannot be covered by its two seeds");
            children[owner[v]].push_back(v);
        }
        if(children[0].empty()||children[1].empty())throw std::runtime_error("topology bisection: split did not shrink domain");
        ++out.splitCount;
        pending.push_back(std::move(children[1]));pending.push_back(std::move(children[0]));
    }
    std::uint64_t hash=1469598103934665603ull;
    for(auto cluster:out.clusterForData)for(int k=0;k<8;++k){hash^=(static_cast<std::uint64_t>(cluster)>>(8*k))&255u;hash*=1099511628211ull;}
    out.signature=hash;
    return out;
}
} // namespace pmsf
