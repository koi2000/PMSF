// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <stdexcept>

namespace pmsf::pclttm_frenet {

using Cell = std::array<std::int32_t,3>;
using Vec = std::array<double,3>;

inline Vec cross(const Vec& a,const Vec& b) {
    return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};
}
inline double dot(const Vec& a,const Vec& b) {return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
inline bool normalize(Vec& v) {
    const double norm=std::sqrt(dot(v,v));
    if (!(norm>1e-12))return false;
    for(auto& component:v)component/=norm;
    return true;
}

struct Frame {
    Vec barycenter{};
    std::array<Vec,3> axes{{{1,0,0},{0,1,0},{0,0,1}}};  // t1, t2, n
    bool degenerate=false;
};

// Section 5. In an oriented triangulated patch the area-weighted normal
// equals the sum of the oriented boundary cross products (diagonals cancel).
// The opt-in wire profile chooses the first distinct directed boundary edge
// in coordinate order so it can reconstruct the gate after merging the fan.
// The original paper derives it from its own conquest order.
inline Frame makeFrame(std::span<const Cell> ring) {
    if(ring.size()<3||ring.size()>6)throw std::runtime_error("PCLTTM Frenet requires a closed 3–6 vertex patch");
    Frame frame;
    Vec normal{};
    std::array<Cell,2> gate{};
    bool hasGate=false;
    for(std::size_t i=0;i<ring.size();++i) {
        const auto& a=ring[i];const auto& b=ring[(i+1)%ring.size()];
        Vec x{},y{};
        for(int k=0;k<3;++k) {
            // The mesh profile is bounded at q16; this also keeps all area
            // cross products and barycenter sums exactly representable.
            if(a[k]<0||a[k]>65536)throw std::runtime_error("PCLTTM Frenet ring coordinate outside q16 bound");
            frame.barycenter[k]+=a[k];x[k]=a[k];y[k]=b[k];
        }
        const auto area=cross(x,y);
        for(int k=0;k<3;++k)normal[k]+=area[k];
        const std::array<Cell,2> candidate{a,b};
        if(a!=b&&(!hasGate||candidate<gate)) {gate=candidate;hasGate=true;}
    }
    for(auto& component:frame.barycenter)component/=static_cast<double>(ring.size());
    if(!normalize(normal)||!hasGate) {frame.degenerate=true;return frame;}
    Vec tangent{};
    for(int k=0;k<3;++k)tangent[k]=double(gate[1][k])-gate[0][k];
    const auto alongNormal=dot(tangent,normal);
    for(int k=0;k<3;++k)tangent[k]-=alongNormal*normal[k];
    if(!normalize(tangent)) {frame.degenerate=true;return frame;}
    auto t2=cross(normal,tangent);
    if(!normalize(t2)) {frame.degenerate=true;return frame;}
    frame.axes={tangent,t2,normal};
    return frame;
}

inline std::int32_t rounded(double value) {
    value=std::round(value);
    if(!std::isfinite(value)||value<std::numeric_limits<std::int32_t>::min()||value>std::numeric_limits<std::int32_t>::max())
        throw std::runtime_error("PCLTTM Frenet coefficient outside int32");
    return static_cast<std::int32_t>(value);
}

inline Cell project(const Frame& frame,const Cell& target,unsigned denominator) {
    if(denominator!=1&&denominator!=2)throw std::runtime_error("invalid Frenet step denominator");
    Vec residual{};for(int k=0;k<3;++k)residual[k]=double(target[k])-frame.barycenter[k];
    Cell local{};for(int k=0;k<3;++k)local[k]=rounded(denominator*dot(residual,frame.axes[k]));
    return local;
}

inline Cell restore(const Frame& frame,const Cell& local,unsigned denominator) {
    if(denominator!=1&&denominator!=2)throw std::runtime_error("invalid Frenet step denominator");
    Cell target{};
    for(int k=0;k<3;++k) {
        double value=frame.barycenter[k];
        for(int axis=0;axis<3;++axis)value+=frame.axes[axis][k]*local[axis]/denominator;
        target[k]=rounded(value);
    }
    return target;
}

struct PatchCode {
    Cell wholeCell{}, halfCell{};
    bool wholeExact=false;
    bool degenerate=false;
};

inline PatchCode projectPatch(const Frame& frame,const Cell& target) {
    for(auto v:target)if(v<0||v>65536)throw std::runtime_error("PCLTTM Frenet target outside q16 bound");
    PatchCode code{project(frame,target,1),project(frame,target,2),false,frame.degenerate};
    code.wholeExact=restore(frame,code.wholeCell,1)==target;
    // Each orthonormal projection error is at most 1/4 grid cell at this
    // step. By Cauchy-Schwarz, each Cartesian error is < 1/2 grid cell.
    if(restore(frame,code.halfCell,2)!=target)
        throw std::runtime_error("PCLTTM Frenet half-cell projection failed exact recovery");
    return code;
}
} // namespace pmsf::pclttm_frenet
