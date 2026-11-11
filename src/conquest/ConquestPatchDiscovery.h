// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "conquest/FrontierConquestEngine.h"
#include "conquest/ConquestTraversal.h"
#include <array>
#include <stdexcept>
#include <vector>

namespace pmsf {

struct ConquestPatchPattern {
    int valence=0;
    std::vector<std::array<int,3>> triangles; // oriented polygon indices
};
struct ConquestDiscoveredPatch {
    std::vector<MCGAL::Facet*> faces;
    std::vector<MCGAL::Vertex*> boundary; // target(gate), next..., source(gate)
};

// Match an oriented triangle template across the current free faces. This is
// a pure topology query: no joins, tag writes, geometry, pool-id ordering or
// guesses about diagonals. An absent/reused/non-free face or vertex fails
// without changing the mesh or state.
inline bool discoverConquestPatch(MCGAL::Halfedge* gate,const ConquestState& state,
                                  const ConquestPatchPattern& pattern,
                                  ConquestDiscoveredPatch* output=nullptr) {
    const int n=pattern.valence;
    if(n<3||n>6||pattern.triangles.size()!=static_cast<std::size_t>(n-2)||!gate)
        return false;
    auto edgeIn=[](const std::array<int,3>& tri,int a,int b) {
        for(int i=0;i<3;++i)if(tri[i]==a&&tri[(i+1)%3]==b)return i;
        return -1;
    };
    int root=-1;
    for(std::size_t i=0;i<pattern.triangles.size();++i) {
        const auto& tri=pattern.triangles[i];
        for(auto v:tri)if(v<0||v>=n)return false;
        if(edgeIn(tri,n-1,0)>=0)root=static_cast<int>(i);
    }
    if(root<0)return false;
    struct Work {int face,from,to;MCGAL::Halfedge* edge;};
    std::vector<Work> pending{{root,n-1,0,gate}};
    ConquestDiscoveredPatch result;result.faces.resize(n-2);result.boundary.resize(n);
    auto assign=[&](int label,MCGAL::Vertex* vertex) {
        if(!vertex||vertex->isRemoved())return false;
        auto*& known=result.boundary[label];
        if(known)return known==vertex;
        if(std::find(result.boundary.begin(),result.boundary.end(),vertex)!=result.boundary.end())return false;
        known=vertex;return true;
    };
    for(std::size_t head=0;head<pending.size();++head) {
        const auto work=pending[head];auto* e=work.edge;
        if(!e||e->isRemoved()||!e->face()||e->face()->isRemoved()||state.fflag(e->face())!=kFFree)return false;
        auto* next=e->next();auto* last=next?next->next():nullptr;
        if(!next||!last||last->next()!=e||next->face()!=e->face()||last->face()!=e->face())return false;
        if(!assign(work.from,e->vertex())||!assign(work.to,e->end_vertex()))return false;
        const auto& tri=pattern.triangles[work.face];const int corner=edgeIn(tri,work.from,work.to);
        if(corner<0||!assign(tri[(corner+2)%3],next->end_vertex()))return false;
        if(result.faces[work.face]) {if(result.faces[work.face]!=e->face())return false;continue;}
        if(std::find(result.faces.begin(),result.faces.end(),e->face())!=result.faces.end())return false;
        result.faces[work.face]=e->face();
        auto* edge=e;
        for(int side=0;side<3;++side,edge=edge->next()) {
            const int a=tri[(corner+side)%3],b=tri[(corner+side+1)%3];
            int neighbor=-1;
            for(std::size_t j=0;j<pattern.triangles.size();++j)if(static_cast<int>(j)!=work.face&&edgeIn(pattern.triangles[j],b,a)>=0) {
                if(neighbor>=0)return false;
                neighbor=static_cast<int>(j);
            }
            if(neighbor>=0)pending.push_back({neighbor,b,a,edge->opposite()});
            else if((a+1)%n!=b)return false; // every external edge belongs to the polygon
        }
        if(pending.size()>1+3*pattern.triangles.size())return false;
    }
    if(std::find(result.faces.begin(),result.faces.end(),nullptr)!=result.faces.end()||
       std::find(result.boundary.begin(),result.boundary.end(),nullptr)!=result.boundary.end())return false;
    if(output)*output=std::move(result);
    return true;
}

} // namespace pmsf
