// SPDX-License-Identifier: Apache-2.0
#pragma once

// Garland & Heckbert 1997, PDF pp.3-5: additive initial-plane quadrics,
// initially selected edge/distance pairs, global minimum-cost contractions,
// optimal placement with segment/endpoints fallback, boundary planes.
// The legacy QEMRefStyleEngine remains an independent calibration protocol.
#include "QEMRefStyleEngine.h"  // value mesh and reversible record types only

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace pmsf::qem_paper1997 {
using Mesh = qem_refstyle::TriangleMesh;
using Point = qem_refstyle::Vec3d;
using Record = qem_refstyle::CollapseRecord;
using Quadric = std::array<double, 16>;

struct Options {
    int targetFaces = 250;
    double pairThreshold = 0.0;
    double boundaryWeight = 1000.0; // paper says large penalty; explicit parameter
    bool preventInversion = true;
    int lodBatchSize = 256; // presentation grouping only; selection stays sequential
};

struct Candidate {
    int survivor = -1;
    int victim = -1;
    Point target;
    double cost = 0.0;
    std::uint64_t versionU = 0, versionV = 0;
};

inline Quadric add(const Quadric& a, const Quadric& b) {
    Quadric out{};
    for (int i = 0; i < 16; ++i) out[i] = a[i] + b[i];
    return out;
}
inline Point minus(Point a, Point b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
inline Point plus(Point a, Point b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
inline Point scale(Point a, double s) { return {a.x*s,a.y*s,a.z*s}; }
inline double dot(Point a, Point b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
inline Point cross(Point a, Point b) {
    return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
inline double evaluate(const Quadric& q, Point p) {
    const double v[4]={p.x,p.y,p.z,1.0}; double result=0.0;
    for (int r=0;r<4;++r) for (int c=0;c<4;++c) result+=v[r]*q[r*4+c]*v[c];
    return result;
}
inline void addPlane(Quadric& q, Point normal, Point onPlane, double weight=1.0) {
    const double plane[4]={normal.x,normal.y,normal.z,-dot(normal,onPlane)};
    for (int r=0;r<4;++r) for (int c=0;c<4;++c) q[r*4+c]+=weight*plane[r]*plane[c];
}

/// Eq.(1), then the paper's singular-system segment optimization, then three
/// discrete candidates. Rank-deficient planar quadrics must not be inverted.
inline Point optimalTarget(const Quadric& q, Point a, Point b) {
    double matrix[3][4]; double norm=0;
    for (int r=0;r<3;++r) {
        for (int c=0;c<3;++c) { matrix[r][c]=q[r*4+c]; norm=std::max(norm,std::fabs(matrix[r][c])); }
        matrix[r][3]=-q[r*4+3];
    }
    bool singular=norm==0.0;
    for (int c=0;c<3 && !singular;++c) {
        int pivot=c;
        for (int r=c+1;r<3;++r) if (std::fabs(matrix[r][c])>std::fabs(matrix[pivot][c])) pivot=r;
        if (std::fabs(matrix[pivot][c])<=norm*1e-12) { singular=true; break; }
        for (int k=c;k<4;++k) std::swap(matrix[pivot][k],matrix[c][k]);
        const double divisor=matrix[c][c];
        for (int k=c;k<4;++k) matrix[c][k]/=divisor;
        for (int r=0;r<3;++r) if (r!=c) {
            const double factor=matrix[r][c];
            for (int k=c;k<4;++k) matrix[r][k]-=factor*matrix[c][k];
        }
    }
    if (!singular) {
        Point p{matrix[0][3],matrix[1][3],matrix[2][3]};
        if (std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)) return p;
    }
    const Point d=minus(b,a);
    const double dv[3]={d.x,d.y,d.z}, av[4]={a.x,a.y,a.z,1.0};
    double curvature=0.0,gradient=0.0;
    for (int r=0;r<3;++r) {
        for (int c=0;c<3;++c) curvature+=dv[r]*q[r*4+c]*dv[c];
        for (int c=0;c<4;++c) gradient+=dv[r]*q[r*4+c]*av[c];
    }
    if (curvature>std::max(1.0,norm)*1e-15) {
        const double t=std::clamp(-gradient/curvature,0.0,1.0);
        return plus(a,scale(d,t));
    }
    Point best=a; double value=evaluate(q,a);
    for (Point p : {b,scale(plus(a,b),0.5)}) {
        const double cost=evaluate(q,p);
        if (cost<value) { value=cost; best=p; }
    }
    return best;
}

class Engine {
    struct Vertex { Point position; Quadric q{}; std::set<int> faces,pairs; std::uint64_t version=0; bool active=true; };
    struct Face { std::array<int,3> corners; bool active=true; };
    struct Later {
        bool operator()(const Candidate& a,const Candidate& b) const {
            return std::tie(a.cost,a.survivor,a.victim)>std::tie(b.cost,b.survivor,b.victim);
        }
    };
  public:
    Engine(const Mesh& input, Options options) : options_(options) {
        if (options.targetFaces<1 || options.lodBatchSize<1 || !std::isfinite(options.pairThreshold) ||
            options.pairThreshold<0 || !std::isfinite(options.boundaryWeight) || options.boundaryWeight<0) {
            throw std::runtime_error("paper QEM: invalid target/batch/threshold/boundary weight");
        }
        vertices_.resize(input.vertices.size());
        for (std::size_t i=0;i<input.vertices.size();++i) {
            const auto p=input.vertices[i];
            if (!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)) throw std::runtime_error("paper QEM: nonfinite vertex");
            vertices_[i].position=p;
        }
        std::map<std::pair<int,int>,std::vector<int>> edges;
        for (std::size_t index=0;index<input.faces.size();++index) {
            const auto& f=input.faces[index];
            for (int v:f) if (v<0||static_cast<std::size_t>(v)>=vertices_.size()) throw std::runtime_error("paper QEM: face index out of range");
            if (f[0]==f[1]||f[1]==f[2]||f[0]==f[2]) throw std::runtime_error("paper QEM: repeated face corner");
            Point normal=cross(minus(input.vertices[f[1]],input.vertices[f[0]]),minus(input.vertices[f[2]],input.vertices[f[0]]));
            const double length=std::sqrt(dot(normal,normal));
            if (!(length>0)) throw std::runtime_error("paper QEM: zero-area input triangle");
            normal=scale(normal,1/length);
            faces_.push_back({f,true});
            for (int v:f) { vertices_[v].faces.insert(static_cast<int>(index)); addPlane(vertices_[v].q,normal,input.vertices[f[0]]); }
            for (int k=0;k<3;++k) {
                const int a=f[k],b=f[(k+1)%3];
                addPair(a,b); edges[std::minmax(a,b)].push_back(static_cast<int>(index));
            }
        }
        liveFaces_=static_cast<int>(input.faces.size());
        if (options.targetFaces>liveFaces_) throw std::runtime_error("paper QEM: target exceeds input faces");
        for (const auto& [edge,incident]:edges) if (incident.size()==1 && options.boundaryWeight>0) {
            const int a=edge.first,b=edge.second;
            const auto f=faces_[incident.front()].corners;
            Point faceNormal=cross(minus(input.vertices[f[1]],input.vertices[f[0]]),minus(input.vertices[f[2]],input.vertices[f[0]]));
            Point perpendicular=cross(minus(input.vertices[b],input.vertices[a]),faceNormal);
            const double length=std::sqrt(dot(perpendicular,perpendicular));
            if (length>0) {
                perpendicular=scale(perpendicular,1/length);
                addPlane(vertices_[a].q,perpendicular,input.vertices[a],options.boundaryWeight);
                addPlane(vertices_[b].q,perpendicular,input.vertices[a],options.boundaryWeight);
            }
        }
        // Only initial pairs are selected. Non-edge candidates remain valid
        // after their endpoints move farther than t; they are remapped on merge.
        if (options.pairThreshold>0) {
            const double threshold2=options.pairThreshold*options.pairThreshold;
            for (int a=0;a<static_cast<int>(vertices_.size());++a)
                for (int b=a+1;b<static_cast<int>(vertices_.size());++b)
                    if (dot(minus(input.vertices[a],input.vertices[b]),minus(input.vertices[a],input.vertices[b]))<threshold2) addPair(a,b);
        }
        for (int a=0;a<static_cast<int>(vertices_.size());++a)
            for (int b:vertices_[a].pairs) if (a<b) push(a,b);
    }

    bool select(Candidate& out) {
        if (liveFaces_<=options_.targetFaces) return false;
        while (!heap_.empty()) {
            const Candidate c=heap_.top(); heap_.pop();
            if (current(c)) { out=c; return true; }
        }
        stalled_=true; return false;
    }
    bool removable(const Candidate& c) const {
        if (!current(c)) return false;
        if (!options_.preventInversion) return true;
        std::set<int> incident=vertices_[c.survivor].faces;
        incident.insert(vertices_[c.victim].faces.begin(),vertices_[c.victim].faces.end());
        for (int fi:incident) {
            const auto f=faces_[fi].corners;
            const bool u=std::find(f.begin(),f.end(),c.survivor)!=f.end();
            const bool v=std::find(f.begin(),f.end(),c.victim)!=f.end();
            if (u&&v) continue;
            Point before[3],after[3];
            for (int k=0;k<3;++k) {
                before[k]=vertices_[f[k]].position;
                after[k]=(f[k]==c.survivor||f[k]==c.victim)?c.target:before[k];
            }
            const Point n0=cross(minus(before[1],before[0]),minus(before[2],before[0]));
            const Point n1=cross(minus(after[1],after[0]),minus(after[2],after[0]));
            if (!(dot(n1,n1)>0) || dot(n0,n1)<0) return false;
        }
        return true;
    }

    Record eliminate(const Candidate& c) {
        if (!current(c)) throw std::runtime_error("paper QEM: stale elimination");
        const int u=c.survivor,v=c.victim;
        Record record; record.survivor=u;record.victim=v;
        record.roundIndex=static_cast<long long>(operations_/options_.lodBatchSize)+1;
        const Point pu=vertices_[u].position,pv=vertices_[v].position;
        const double a[3]={pu.x,pu.y,pu.z},b[3]={pv.x,pv.y,pv.z};
        std::copy(a,a+3,record.survivorPosBefore);std::copy(b,b+3,record.victimPosBefore);
        const std::vector<int> incident(vertices_[v].faces.begin(),vertices_[v].faces.end());
        for (int fi:incident) {
            const auto f=faces_[fi].corners;
            for (int k=0;k<3;++k) if (f[k]==v) record.fanFaces.push_back({f[(k+1)%3],f[(k+2)%3]});
        }
        lastQBefore_=vertices_[u].q;lastQVictim_=vertices_[v].q;
        vertices_[u].q=add(vertices_[u].q,vertices_[v].q);
        lastQAfter_=vertices_[u].q;
        vertices_[u].position=c.target;
        affected_=vertices_[u].pairs;
        affected_.insert(vertices_[v].pairs.begin(),vertices_[v].pairs.end());
        affected_.erase(u);affected_.erase(v);
        for (int other:vertices_[v].pairs) {
            vertices_[other].pairs.erase(v);
            if (other!=u && vertices_[other].active) addPair(u,other);
        }
        vertices_[u].pairs.erase(v);vertices_[v].pairs.clear();
        for (int fi:incident) {
            auto& f=faces_[fi];
            for (int corner:f.corners) vertices_[corner].faces.erase(fi);
            for (int& corner:f.corners) if (corner==v) corner=u;
            if (f.corners[0]==f.corners[1]||f.corners[1]==f.corners[2]||f.corners[0]==f.corners[2]) {
                f.active=false;--liveFaces_;
            } else for (int corner:f.corners) vertices_[corner].faces.insert(fi);
        }
        vertices_[v].active=false;vertices_[v].faces.clear();
        ++vertices_[u].version;++vertices_[v].version;++operations_;
        return record;
    }

    void update(const Candidate& c) {
        const int u=c.survivor;
        // The quadric costs change only at u. Normal-validity checks can change
        // throughout the affected one-ring, so revive those rejected pairs too.
        std::set<std::pair<int,int>> candidates;
        affected_.insert(u);
        for (int a:affected_) if (vertices_[a].active)
            for (int b:vertices_[a].pairs) if (vertices_[b].active) candidates.insert(std::minmax(a,b));
        for (auto [a,b]:candidates) push(a,b);
    }

    Mesh snapshot(std::vector<int>* originalIds=nullptr) const {
        Mesh out;std::vector<int> ids(vertices_.size(),-1);
        if (originalIds) originalIds->clear();
        for (std::size_t v=0;v<vertices_.size();++v) if (vertices_[v].active) {
            ids[v]=static_cast<int>(out.vertices.size());out.vertices.push_back(vertices_[v].position);
            if (originalIds) originalIds->push_back(static_cast<int>(v));
        }
        for (const auto& f:faces_) if (f.active) out.faces.push_back({ids[f.corners[0]],ids[f.corners[1]],ids[f.corners[2]]});
        return out;
    }
    int liveFaceCount() const { return liveFaces_; }
    std::size_t operationCount() const { return operations_; }
    bool stalled() const { return stalled_; }
    const Quadric& lastBefore() const { return lastQBefore_; }
    const Quadric& lastVictim() const { return lastQVictim_; }
    const Quadric& lastAfter() const { return lastQAfter_; }
    const Quadric& quadric(int v) const { return vertices_.at(v).q; }
    bool active(int v) const { return vertices_.at(v).active; }
    int vertexCount() const { return static_cast<int>(vertices_.size()); }
    const Point& point(int v) const { return vertices_.at(v).position; }
    bool hasPair(int a,int b) const { return vertices_.at(a).pairs.count(b)!=0; }
  private:
    void addPair(int a,int b) { if (a!=b) { vertices_[a].pairs.insert(b);vertices_[b].pairs.insert(a); } }
    bool current(const Candidate& c) const {
        return vertices_[c.survivor].active&&vertices_[c.victim].active&&
            vertices_[c.survivor].pairs.count(c.victim)&&
            c.versionU==vertices_[c.survivor].version&&c.versionV==vertices_[c.victim].version;
    }
    void push(int a,int b) {
        if (a>b) std::swap(a,b);
        Candidate c;c.survivor=a;c.victim=b;
        const auto q=add(vertices_[a].q,vertices_[b].q);
        c.target=optimalTarget(q,vertices_[a].position,vertices_[b].position);
        c.cost=std::max(0.0,evaluate(q,c.target));
        c.versionU=vertices_[a].version;c.versionV=vertices_[b].version;
        if (std::isfinite(c.cost)&&std::isfinite(c.target.x)&&std::isfinite(c.target.y)&&std::isfinite(c.target.z)) heap_.push(c);
    }
    Options options_;
    std::vector<Vertex> vertices_;
    std::vector<Face> faces_;
    std::priority_queue<Candidate,std::vector<Candidate>,Later> heap_;
    std::set<int> affected_;
    int liveFaces_=0;
    std::size_t operations_=0;
    bool stalled_=false;
    Quadric lastQBefore_{},lastQVictim_{},lastQAfter_{};
};
}  // namespace pmsf::qem_paper1997
