// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "conquest/ConquestLayerTopology.h"
#include "conquest/ConquestPatchDiscovery.h"
#include <array>
#include <limits>

namespace pmsf {

struct ConquestReplayTrace {
    // Exact per-pop records: rooted vertex labels, face state, tags, vertex
    // states, queue size, and executed symbol (-1 for skipped gates). Vertex
    // labels are derived from topology, then insertion order, never pool IDs.
    using Event=std::array<std::uint64_t,12>;
    std::vector<Event> events;
    std::unordered_map<MCGAL::Vertex*,std::uint64_t> labels;
    void start(MCGAL::Halfedge* seed) {
        events.clear();labels.clear();
        const auto graph=snapshotConquestLayerTopology(seed);
        for(auto* vertex:graph.vertices)bind(vertex);
    }
    void bind(MCGAL::Vertex* vertex) {
        if(!vertex)throw std::runtime_error("conquest trace: null vertex");
        if(!labels.count(vertex))labels.emplace(vertex,labels.size());
    }
    void pop(MCGAL::Halfedge* gate,const ConquestState& st,std::size_t remaining) {
        auto id=[&](MCGAL::Vertex* vertex){return vertex?labels.at(vertex):std::numeric_limits<std::uint64_t>::max();};
        auto* source=gate->vertex();auto* target=gate->end_vertex();auto* front=tgt(nxt(gate));
        events.push_back({id(source),id(target),id(front),st.fflag(gate->face()),
            st.sign(source),st.sign(target),st.sign(front),st.flag(source),st.flag(target),st.flag(front),
            remaining,std::numeric_limits<std::uint64_t>::max()});
    }
    void symbol(int code) {events.back().back()=static_cast<std::uint64_t>(code);}
};

struct ConquestDecodeControl {
    const std::vector<ConquestPatchPattern>* patternsByType=nullptr;
    bool packed=false;
    std::size_t expandedSymbols=0;
    ConquestReplayTrace* trace=nullptr;
    std::vector<std::uint8_t>* feasibleMasks=nullptr; // simulation only; never serialized
};

// Reverse selection preserves the semantics of the actual next transmitted
// symbol. Only a suffix of a null run can be omitted: once an explicit null
// remains, earlier gates see that null rather than the future valence.
inline std::vector<int> packConquestNulls(const std::vector<int>& expanded,
                                        const std::vector<std::uint8_t>& feasible) {
    if(expanded.size()!=feasible.size())throw std::runtime_error("conquest packing: mask count differs");
    std::vector<int> reversed;reversed.reserve(expanded.size());
    int next=-1;
    for(std::size_t i=expanded.size();i-- >0;) {
        const int code=expanded[i];
        if(code<0||code>4)throw std::runtime_error("conquest packing: invalid symbol");
        if(code==4&&next>=0&&next<4&&(feasible[i]&(1u<<next))==0)continue;
        reversed.push_back(code);next=code;
    }
    return {reversed.rbegin(),reversed.rend()};
}
} // namespace pmsf
