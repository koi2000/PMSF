#pragma once

// Boundary-stage storage V3 (skeleton): shared stable-ID registry.
//
// ID space (see docs/memory/BOUNDARY_STAGE_STORAGE_V3_DESIGN.md):
//   kind 0 = B-ID : base vertex order index (serialized base mesh order)
//   kind 1 = I-ID : interior-removal birth id (round, residual ordinal in round)
//   kind 2 = S-ID : stage-removal birth id (round, stage op index in round)
//   kind 3 reserved (stage-created vertices; not supported in V3.0)
//
// M1 validated (BOUNDARY_ID_CONTRACT_M1.md): decoder insert_point order == encoder
// residual emission order (forward), and serialized base order == decoder base order.
// The registry is used only when PMSF_BND_V3 is enabled on BOTH compress/decompress
// (skeleton mode); the final V3 artifact will carry the mode in its header instead.

#include "Mesh.h"
#include "Vertex.h"
#include "Facet.h"
#include "boundary/BoundaryTopologyPatch.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unordered_map>
#include <vector>

namespace pmsf {
namespace bndid {

inline bool envEnabled() {
    const char* e = std::getenv("PMSF_BND_V3");
    return e != nullptr && e[0] != '\0' && std::strcmp(e, "0") != 0;
}

struct Id {
    int kind = -1;
    int a = 0;
    int b = 0;
};

inline bool valid(const Id& id) { return id.kind >= 0 && id.kind <= 3; }

struct IdHash {
    std::size_t operator()(const Id& id) const {
        std::uint64_t k = static_cast<std::uint64_t>(id.kind);
        std::uint64_t a = static_cast<std::uint64_t>(static_cast<std::uint32_t>(id.a));
        std::uint64_t b = static_cast<std::uint64_t>(static_cast<std::uint32_t>(id.b));
        return std::hash<std::uint64_t>{}((k << 62) ^ (a * 0x9E3779B97F4A7C15ull) ^ b);
    }
};
struct IdEq {
    bool operator()(const Id& x, const Id& y) const {
        return x.kind == y.kind && x.a == y.a && x.b == y.b;
    }
};

// ---------------------------------------------------------------------------
// Encoder-side bookkeeping
// ---------------------------------------------------------------------------

struct EncoderState {
    bool on = false;
    std::unordered_map<MCGAL::Vertex*, Id> ids;              // removed vertices -> I/S-ID
    std::unordered_map<MCGAL::Facet*, MCGAL::Vertex*> facetRemoved;  // patch face -> removed vertex
    int round = -1;
    int counter = 0;
};

inline EncoderState& enc() {
    static EncoderState st;
    return st;
}

inline void encEnable() { enc().on = envEnabled(); }

inline void encReset() {
    enc().ids.clear();
    enc().facetRemoved.clear();
    enc().round = -1;
    enc().counter = 0;
}

// Called from VertexRemovalEliminateOperator::remove_point after the patch face got
// its removed-vertex position, so the collector can later link a residual emission
// back to the removed vertex pointer and assign its I-ID.
inline void encNoteFacetRemovedVertex(MCGAL::Facet* f, MCGAL::Vertex* v) {
    if (!enc().on || !f || !v) return;
    enc().facetRemoved[f] = v;
}

inline MCGAL::Vertex* encFacetRemovedVertex(MCGAL::Facet* f) {
    if (!enc().on || !f) return nullptr;
    auto it = enc().facetRemoved.find(f);
    return it == enc().facetRemoved.end() ? nullptr : it->second;
}

// Called at collector residual emission, in emission order.  The ordinal is the
// decoder-visible insertion ordinal (M1: forward mapping).
inline Id encAssignInteriorId(MCGAL::Vertex* v) {
    if (!enc().on || !v) return Id{};
    const int r = pmsf::boundaryTopologyRound();
    if (r != enc().round) {
        enc().round = r;
        enc().counter = 0;
    }
    Id id;
    id.kind = 1;
    id.a = r;
    id.b = enc().counter++;
    enc().ids[v] = id;
    return id;
}

inline void encNoteStageRemoved(MCGAL::Vertex* v, int round, int opIndex) {
    if (!enc().on || !v) return;
    Id id;
    id.kind = 2;
    id.a = round;
    id.b = opIndex;
    enc().ids[v] = id;
}

inline bool encLookup(MCGAL::Vertex* v, Id& out) {
    if (!enc().on || !v) return false;
    auto it = enc().ids.find(v);
    if (it == enc().ids.end()) return false;
    out = it->second;
    return true;
}

// Resolve any vertex to its V3 ID: base survivor -> B-ID (caller supplies the
// base index map since the exact serialized order includes seed handling), else
// look up the recorded I-ID/S-ID.
inline bool encResolve(MCGAL::Vertex* v, const std::unordered_map<MCGAL::Vertex*, int>& baseIdx,
                       Id& out) {
    if (!v) return false;
    auto it = baseIdx.find(v);
    if (it != baseIdx.end()) {
        out.kind = 0;
        out.a = 0;
        out.b = it->second;
        return true;
    }
    return encLookup(v, out);
}

// ---------------------------------------------------------------------------
// Decoder-side registry
// ---------------------------------------------------------------------------

struct DecoderState {
    bool on = false;
    std::unordered_map<Id, MCGAL::Vertex*, IdHash, IdEq> byId;
    std::unordered_map<MCGAL::Vertex*, Id> byVertex;
    std::vector<MCGAL::Vertex*> baseOrder;
    int round = -1;
    int counter = 0;
};

inline DecoderState& dec() {
    static DecoderState st;
    return st;
}

inline void decEnable(bool force = false) { dec().on = force || envEnabled(); }

inline void decReset() {
    dec().byId.clear();
    dec().byVertex.clear();
    dec().baseOrder.clear();
    dec().round = -1;
    dec().counter = 0;
}

inline void decRegister(const Id& id, MCGAL::Vertex* v) {
    if (!dec().on || !v || id.kind < 0) return;
    dec().byId[id] = v;
    dec().byVertex[v] = id;
}

// Base order == decoder mesh->vertices() creation order (M1 validated).
inline void decInitFromBase() {
    if (!dec().on) return;
    dec().baseOrder.clear();
    dec().byId.clear();
    dec().byVertex.clear();
}

inline void decInitFromMesh(MCGAL::Mesh* mesh) {
    if (!dec().on || !mesh) return;
    decReset();
    int idx = 0;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        dec().baseOrder.push_back(v);
        Id id;
        id.kind = 0;
        id.a = 0;
        id.b = idx++;
        decRegister(id, v);
    }
}

inline void decSetRound(int round) {
    dec().round = round;
    dec().counter = 0;
}

inline Id decRegisterInterior(MCGAL::Vertex* v) {
    if (!dec().on || !v || dec().round < 0) return Id{};
    Id id;
    id.kind = 1;
    id.a = dec().round;
    id.b = dec().counter++;
    decRegister(id, v);
    return id;
}

inline Id decRegisterStage(MCGAL::Vertex* v, int round, int opIndex) {
    if (!dec().on || !v) return Id{};
    Id id;
    id.kind = 2;
    id.a = round;
    id.b = opIndex;
    decRegister(id, v);
    return id;
}

inline MCGAL::Vertex* decGet(const Id& id) {
    if (!dec().on) return nullptr;
    auto it = dec().byId.find(id);
    return it == dec().byId.end() ? nullptr : it->second;
}

inline bool decLookup(const MCGAL::Vertex* v, Id& out) {
    if (!dec().on || !v) return false;
    auto it = dec().byVertex.find(const_cast<MCGAL::Vertex*>(v));
    if (it == dec().byVertex.end()) return false;
    out = it->second;
    return true;
}

}  // namespace bndid
}  // namespace pmsf