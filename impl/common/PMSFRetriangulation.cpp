// SPDX-License-Identifier: Apache-2.0
//
// PMSFRetriangulation.cpp — PR-U + Phase 1 Figure 9 extension.
// Ahn et al. 2011 Fig.1 (valence-only) + shared executable model for
// Alliez & Desbrun 2001 Fig.9 (tag-driven). The latter is not yet a literal
// imported 64-entry table.

#include "common/PMSFRetriangulation.h"
#include "storage/PMSFSymbolStorageAdapter.h"
#include <algorithm>
#include <stdexcept>
#include <vector>

namespace pmsf {

RetriangulationResult paperRetriangulate(int valence) {
    RetriangulationResult r;
    switch (valence) {
        case 3:
            r.triangles  = {{0, 1, 2}};
            break;
        case 4:
            r.triangles  = {{0, 1, 2}, {0, 2, 3}};
            r.diagonals  = {{0, 2}};
            r.plusTagIdx  = {0, 2};
            r.minusTagIdx = {1, 3};
            break;
        case 5:
            r.triangles  = {{0, 1, 2}, {0, 2, 3}, {0, 3, 4}};
            r.diagonals  = {{0, 2}, {0, 3}};
            r.plusTagIdx  = {0, 2, 3};
            r.minusTagIdx = {1, 4};
            break;
        case 6:
            r.triangles  = {{0, 1, 2}, {2, 3, 4}, {4, 5, 0}, {0, 2, 4}};
            r.diagonals  = {{0, 2}, {2, 4}, {0, 4}};
            r.plusTagIdx  = {0, 2, 4};
            r.minusTagIdx = {1, 3, 5};
            break;
        default: break;
    }
    return r;
}

// =============================================================================
// Figure 9 tag-driven retriangulation model (Alliez & Desbrun 2001)
// =============================================================================
// Shared executable model maps (degree, tagLeft, tagRight) → retriangulation pattern.
// Plus=1 means diagonal to center; Minus=2 means no diagonal.
//
// Tag sequence generation (Figure 9 semantics):
//   ring[0] = tagLeft, ring[N-1] = tagRight
//   Middle vertices alternate Plus/Minus, anchored from the side
//   closest to the given endpoint.
//
// Retriangulation: fan from the first Plus vertex, with diagonals
// to all other non-adjacent Plus vertices.

namespace {

// Compute tag sequence for Figure 9
std::vector<int> computeFig9Tags(int N, int tagLeft, int tagRight) {
    std::vector<int> tags(N, 0);
    if (N < 3) return tags;

    tags[0] = tagLeft;
    tags[N - 1] = tagRight;

    if (N == 3) {
        // One middle vertex: tag opposite to whichever endpoint is more constrained
        // If endpoints agree (both Plus or both Minus), middle gets opposite
        // If endpoints differ, middle matches the anchor (left)
        tags[1] = (tagLeft == tagRight) ? (3 - tagLeft) : tagLeft;  // 3-x flips 1↔2
        return tags;
    }

    // N >= 4: alternating pattern from left to right
    // Assign middle vertices [1, N-2] alternating from tagLeft
    int cur = (tagLeft == 1) ? 2 : 1;  // start opposite to tagLeft
    for (int i = 1; i < N - 1; ++i) {
        tags[i] = cur;
        cur = 3 - cur;  // flip
    }
    return tags;
}

}  // namespace

RetriangulationResult paperRetriangulateFig9(int valence, int tagLeft, int tagRight) {
    RetriangulationResult r;
    if (valence < 3 || valence > 6) return r;

    // Step 1: Compute tag sequence
    auto tags = computeFig9Tags(valence, tagLeft, tagRight);

    // Step 2: Identify Plus/Minus vertices
    for (int i = 0; i < valence; ++i) {
        if (tags[i] == 1) r.plusTagIdx.push_back(i);
        else r.minusTagIdx.push_back(i);
    }

    if (r.plusTagIdx.empty()) {
        // All Minus — no center connections, degenerate. Fallback to fan from 0.
        r.plusTagIdx.push_back(0);
        for (int i = 0; i < valence; ++i)
            if (i != 0) r.minusTagIdx.push_back(i);
    }

    // Step 3: Generate a complete deterministic fan from the first Plus
    // vertex. This is the conservative executable form of the Figure 9
    // tag-driven table: endpoint/current tags determine the tag sequence and
    // therefore the fan anchor; the actual polygon split is then complete for
    // every degree 3..6. Earlier placeholder code emitted only a subset of the
    // required diagonals for anchors other than 0 and could leave unstable
    // N-gons in the conquest mesh.
    int anchor = r.plusTagIdx[0];
    for (int i = 1; i < valence - 1; ++i) {
        int b = (anchor + i) % valence;
        int c = (anchor + i + 1) % valence;
        r.triangles.push_back({anchor, b, c});
        if (i < valence - 2) {
            r.diagonals.push_back({anchor, c});
        }
    }

    return r;
}

RetriangulationResult paperRetriangulateFig9Exact(int valence,int sourceTag,int targetTag) {
    RetriangulationResult result;
    if(valence<3||valence>6||(sourceTag!=1&&sourceTag!=2)||(targetTag!=1&&targetTag!=2))
        return result;
    const bool evenFamily=(valence==4&&targetTag==1)||(valence==6&&targetTag==1);
    std::vector<int> interior;
    if(valence==3)interior={(sourceTag==1&&targetTag==1)?2:1};
    else if(valence==4)interior=evenFamily?std::vector<int>{2,1}:std::vector<int>{1,2};
    else if(valence==5)interior=(sourceTag==1&&targetTag==1)?std::vector<int>{2,1,2}:std::vector<int>{1,2,1};
    else interior=evenFamily?std::vector<int>{2,1,2,1}:std::vector<int>{1,2,1,2};
    const int tags[2]={targetTag,sourceTag};
    for(int i=0;i<valence;++i) {
        const int tag=i==0?tags[0]:i==valence-1?tags[1]:interior[i-1];
        (tag==1?result.plusTagIdx:result.minusTagIdx).push_back(i);
    }
    if(valence==4)result.diagonals=evenFamily?std::vector<std::array<int,2>>{{0,2}}:std::vector<std::array<int,2>>{{1,3}};
    if(valence==5) {
        if(sourceTag==1&&targetTag==1)result.diagonals={{0,2},{2,4}};
        else if(sourceTag==2&&targetTag==1)result.diagonals={{0,3},{1,3}};
        else result.diagonals={{1,3},{1,4}};
    }
    if(valence==6)result.diagonals=evenFamily?std::vector<std::array<int,2>>{{0,2},{0,4},{2,4}}:
        std::vector<std::array<int,2>>{{1,3},{1,5},{3,5}};
    // Derive oriented triangles from the literal boundary+diagonals, instead
    // of substituting a fan for the central-triangle hexagon cases.
    auto adjacent=[&](int a,int b) {
        if((a+1)%valence==b||(b+1)%valence==a)return true;
        return std::any_of(result.diagonals.begin(),result.diagonals.end(),[&](const auto& edge){
            return (edge[0]==a&&edge[1]==b)||(edge[1]==a&&edge[0]==b);
        });
    };
    for(int a=0;a<valence;++a)for(int b=a+1;b<valence;++b)for(int c=b+1;c<valence;++c)
        if(adjacent(a,b)&&adjacent(b,c)&&adjacent(c,a))result.triangles.push_back({a,b,c});
    return result;
}

int paperMepp2TypeFromBoundaryTags(int valence, int sourceTag, int targetTag) {
    const bool sourcePlus = sourceTag == 1;
    const bool sourceMinus = sourceTag == 2;
    const bool targetPlus = targetTag == 1;
    const bool targetMinus = targetTag == 2;
    if ((!sourcePlus && !sourceMinus) || (!targetPlus && !targetMinus)) return 0;

    int offset = 0;
    switch (valence) {
        case 3: offset = 1; break;
        case 4: offset = 5; break;
        case 5: offset = 9; break;
        case 6: offset = 13; break;
        default: return 0;
    }
    if (targetMinus && sourcePlus) return offset + 0;
    if (targetPlus && sourceMinus) return offset + 1;
    if (targetPlus && sourcePlus) return offset + 2;
    if (targetMinus && sourceMinus) return offset + 3;
    return 0;
}

std::vector<int> paperMepp2InteriorTags(int type) {
    switch (type) {
        case 1:
        case 2:
        case 4:
            return {1};
        case 3:
            return {2};
        case 5:
        case 8:
            return {1, 2};
        case 6:
        case 7:
            return {2, 1};
        case 9:
        case 10:
        case 12:
            return {1, 2, 1};
        case 11:
            return {2, 1, 2};
        case 13:
        case 16:
            return {1, 2, 1, 2};
        case 14:
        case 15:
            return {2, 1, 2, 1};
        default:
            return {};
    }
}

}  // namespace pmsf
