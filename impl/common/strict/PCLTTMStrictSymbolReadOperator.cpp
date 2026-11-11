// SPDX-License-Identifier: Apache-2.0
//
// PCLTTMStrictSymbolReadOperator.cpp

#include "common/strict/PCLTTMStrictSymbolReadOperator.h"
#include "common/BoundedRingTraversal.h"
#include "common/PaperLayoutAliases.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂

#include "compress/ConnectivityBlobContainer.h"
#include "common/strict/PCLTTMStrictSymbolCollectOperator.h"
#include "BufferUtils.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "MeshUtils.h"
#include "Point.h"
#include "Vertex.h"
#include "common/PCLTTMTagTable.h"
#include "common/patch/RDPatchGateQueue.h"
#include "common/PMSFRetriangulation.h"
#include "common/patch/RDPatchDecoderConquest.h"
#include "predict/RDPredictor.h"

#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace pmsf {
namespace {

enum : uint8_t {
    kSymNull = 0,
    kSymV3 = 1,
    kSymV4 = 2,
    kSymV5 = 3,
    kSymV6 = 4,
};


std::vector<MCGAL::Halfedge*> boundedFaceHalfedges(MCGAL::Facet* f,
                                                   int maxSteps = 64);

std::vector<MCGAL::Halfedge*> boundedVertexHalfedges(MCGAL::Vertex* v,
                                                     int maxSteps = 256);

inline MCGAL::Halfedge* inputBoundaryFromGate(MCGAL::Halfedge* h) {
    if (!h || h->isRemoved() || !h->next() || !h->next()->next()) return nullptr;
    return h->next()->next();
}

inline MCGAL::Halfedge* gateFromInputBoundary(MCGAL::Halfedge* boundary) {
    if (!boundary || boundary->isRemoved() || !boundary->next()) return nullptr;
    return boundary->next();
}

inline char* rangeCoderInput(std::span<const uint8_t> payload) {
    return const_cast<char*>(reinterpret_cast<const char*>(payload.data()));
}

int32_t readI32(std::span<const uint8_t> s, size_t& off, const char* what) {
    if (off + 4 > s.size()) {
        throw std::runtime_error(std::string("[PCLTTMStrict decoder] truncated ") + what);
    }
    int32_t v = 0;
    std::memcpy(&v, s.data() + off, 4);
    off += 4;
    return v;
}

uint16_t readU16(std::span<const uint8_t> s, size_t& off, const char* what) {
    if (off + 2 > s.size()) {
        throw std::runtime_error(std::string("[PCLTTMStrict decoder] truncated ") + what);
    }
    uint16_t v = 0;
    std::memcpy(&v, s.data() + off, 2);
    off += 2;
    return v;
}

uint32_t readVarU32(std::span<const uint8_t> s, size_t& off, const char* what) {
    uint32_t value = 0;
    int shift = 0;
    while (shift <= 28) {
        if (off >= s.size()) {
            throw std::runtime_error(std::string("[PCLTTMStrict decoder] truncated ") + what);
        }
        const uint8_t b = s[off++];
        value |= static_cast<uint32_t>(b & 0x7fu) << shift;
        if ((b & 0x80u) == 0) return value;
        shift += 7;
    }
    throw std::runtime_error(std::string("[PCLTTMStrict decoder] malformed varint ") + what);
}

int32_t readZigZagVarI32(std::span<const uint8_t> s, size_t& off, const char* what) {
    const uint32_t z = readVarU32(s, off, what);
    return static_cast<int32_t>((z >> 1) ^ (~(z & 1u) + 1u));
}

int ceilLog2Positive(int v) {
    int lg = 0;
    int p = 1;
    while (p < v) {
        p <<= 1;
        ++lg;
    }
    return lg;
}

RDPatchSymbolKind strictToKind(uint8_t s, bool cleaning) {
    if (s == kSymNull) return RDPatchSymbolKind::NULL_PATCH;
    if (cleaning) return RDPatchSymbolKind::V_3;
    switch (s) {
        case kSymV3: return RDPatchSymbolKind::V_3;
        case kSymV4: return RDPatchSymbolKind::V_4;
        case kSymV5: return RDPatchSymbolKind::V_5;
        case kSymV6: return RDPatchSymbolKind::V_6;
        default: break;
    }
    throw std::runtime_error("[PCLTTMStrict decoder] invalid strict symbol");
}

MCGAL::Vector3 polygonNormal(MCGAL::Facet* f) {
    MCGAL::Vector3 n(0, 0, 0);
    if (!f || f->isRemoved()) return n;
    MCGAL::Halfedge* start = f->proxyHalfedge();
    if (!start) return n;
    MCGAL::Halfedge* h = start;
    int guard = 0;
    do {
        MCGAL::Vertex* a = h ? h->vertex() : nullptr;
        MCGAL::Vertex* b = h ? h->end_vertex() : nullptr;
        if (a && b && !a->isRemoved() && !b->isRemoved()) {
            MCGAL::Vector3 va(a->point());
            MCGAL::Vector3 vb(b->point());
            n = n + (va % vb);
        }
        h = h ? h->next() : nullptr;
        ++guard;
    } while (h && h != start && guard < 256);
    if (n.length() == 0) return n;
    return n.normalize();
}

MCGAL::Halfedge* findLiveFaceGate(MCGAL::Facet* f) {
    if (!f || f->isRemoved()) return nullptr;
    MCGAL::Halfedge* h = f->proxyHalfedge();
    if (h && !h->isRemoved() && h->face() == f) return h;
    for (MCGAL::Halfedge* it : boundedFaceHalfedges(f)) {
        if (it && !it->isRemoved()) return it;
    }
    return nullptr;
}

bool strictUsesCartesianResiduals() {
    const char* e = std::getenv("PMSF_PCLTTM_GEOM_MODE");
    return !e || std::string(e) == "cartesian" || std::string(e) == "absolute";
}

bool strictUsesAbsoluteGeometry() {
    const char* e = std::getenv("PMSF_PCLTTM_GEOM_MODE");
    return e && std::string(e) == "absolute";
}

MCGAL::PointInt barycenterFromRingQ(const std::vector<std::array<int, 3>>& ringQ) {
    if (ringQ.empty()) return MCGAL::PointInt(0, 0, 0);
    int64_t sx = 0;
    int64_t sy = 0;
    int64_t sz = 0;
    for (const auto& q : ringQ) {
        sx += q[0];
        sy += q[1];
        sz += q[2];
    }
    const int64_t n = static_cast<int64_t>(ringQ.size());
    return MCGAL::PointInt(static_cast<int>(sx / n),
                           static_cast<int>(sy / n),
                           static_cast<int>(sz / n));
}

bool isWalkGate(MCGAL::Halfedge* h) {
    return h && !h->isRemoved() && h->face() && !h->face()->isRemoved() &&
           h->vertex() && !h->vertex()->isRemoved() &&
           h->end_vertex() && !h->end_vertex()->isRemoved();
}

RDPatchVertexTag mcgalTagToRDPatch(MCGAL::Vertex::TagFlag t) {
    switch (t) {
        case MCGAL::Vertex::Plus: return RDPatchVertexTag::Plus;
        case MCGAL::Vertex::Minus: return RDPatchVertexTag::Minus;
        default: return RDPatchVertexTag::Untagged;
    }
}

RDPatchVertexTag tagOfVertex(MCGAL::Vertex* v) {
    if (!v) return RDPatchVertexTag::Untagged;
    return mcgalTagToRDPatch(v->getTag());
}

int countProcessedFaces(MCGAL::Vertex* v) {
    if (!v || v->isRemoved()) return 0;
    int n = 0;
    for (MCGAL::Halfedge* he : boundedVertexHalfedges(v)) {
        if (!he || he->isRemoved()) continue;
        MCGAL::Facet* f = he->face();
        if (f && !f->isRemoved() && f->isProcessed()) ++n;
    }
    return n;
}

bool isVertexSaturated(MCGAL::Vertex* v) {
    const int n = countProcessedFaces(v);
    static const char* envSat = std::getenv("PMSF_RDPATCH_SAT_RANGE");
    static const bool relaxed = envSat && std::strcmp(envSat, "relaxed") == 0;
    if (relaxed) return n >= 1 && n <= 4;
    return n == 2 || n == 3;
}

int walkBucket(MCGAL::Halfedge* h,
               const std::set<MCGAL::Facet*>& nullPatchFaces) {
    const char* env6 = std::getenv("PMSF_RDPATCH_USE_6BUCKET");
    if (!env6 || env6[0] != '1') return 5;
    if (!h) return 6;
    MCGAL::Facet* f = h->face();
    const RDPatchNullFlag nf =
        (f && nullPatchFaces.count(f)) ? RDPatchNullFlag::Null : RDPatchNullFlag::NonNull;
    return RDPatchGateQueue::computeBucket(tagOfVertex(h->vertex()),
                                           tagOfVertex(h->end_vertex()),
                                           nf,
                                           isVertexSaturated(h->vertex()),
                                           isVertexSaturated(h->end_vertex()));
}

void pushWalkGate(MCGAL::Halfedge* h,
                  RDPatchGateQueue& q,
                  std::set<MCGAL::Halfedge*>& enqueued,
                  std::set<MCGAL::Facet*>& facesEnqueued,
                  const std::set<MCGAL::Facet*>& nullPatchFaces) {
    if (!isWalkGate(h) || enqueued.count(h)) return;
    MCGAL::Facet* f = h->face();
    if (f && f->isProcessed()) return;
    if (f && facesEnqueued.count(f)) return;
    enqueued.insert(h);
    if (f) facesEnqueued.insert(f);
    q.push(h, walkBucket(h, nullPatchFaces));
}

void seedWalkFaces(std::shared_ptr<MCGAL::Mesh> mesh,
                   RDPatchGateQueue& q,
                   std::set<MCGAL::Halfedge*>& enqueued,
                   std::set<MCGAL::Facet*>& facesEnqueued,
                   const std::set<MCGAL::Facet*>& nullPatchFaces) {
    q.clear();
    enqueued.clear();
    facesEnqueued.clear();
    if (!mesh) return;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        for (MCGAL::Halfedge* h : *f) {
            if (isWalkGate(h)) {
                pushWalkGate(h, q, enqueued, facesEnqueued, nullPatchFaces);
                break;
            }
        }
    }
}

void seedWalkSingleGate(MCGAL::Halfedge* seed,
                        RDPatchGateQueue& q,
                        std::set<MCGAL::Halfedge*>& enqueued,
                        std::set<MCGAL::Facet*>& facesEnqueued,
                        const std::set<MCGAL::Facet*>& nullPatchFaces) {
    q.clear();
    enqueued.clear();
    facesEnqueued.clear();
    if (!isWalkGate(seed)) return;
    MCGAL::Halfedge* input = inputBoundaryFromGate(seed);
    if (!input || input->isRemoved()) input = seed;
    if (input->vertex()) {
        input->vertex()->setTag(MCGAL::Vertex::Plus);
        input->vertex()->setConquered();
    }
    if (input->end_vertex()) {
        input->end_vertex()->setTag(MCGAL::Vertex::Minus);
        input->end_vertex()->setConquered();
    }
    pushWalkGate(seed, q, enqueued, facesEnqueued, nullPatchFaces);
}

void markWalkSeedInput(MCGAL::Halfedge* seed) {
    if (!isWalkGate(seed)) return;
    MCGAL::Halfedge* input = inputBoundaryFromGate(seed);
    if (!input || input->isRemoved()) input = seed;
    if (input->vertex() && !input->vertex()->isRemoved()) {
        input->vertex()->setTag(MCGAL::Vertex::Plus);
        input->vertex()->setConquered();
    }
    if (input->end_vertex() && !input->end_vertex()->isRemoved()) {
        input->end_vertex()->setTag(MCGAL::Vertex::Minus);
        input->end_vertex()->setConquered();
    }
}

bool sameQuant(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Vertex* v,
               int qx, int qy, int qz) {
    if (!mesh || !v || v->isRemoved()) return false;
    MCGAL::PointInt q = mesh->floatPosToInt(v->point());
    return q[0] == qx && q[1] == qy && q[2] == qz;
}

struct QuantKey {
    int x = 0;
    int y = 0;
    int z = 0;

    bool operator==(const QuantKey& o) const {
        return x == o.x && y == o.y && z == o.z;
    }
};

struct QuantKeyHash {
    size_t operator()(const QuantKey& k) const {
        uint64_t h = 1469598103934665603ull;
        auto mix = [&](int v) {
            uint32_t u = static_cast<uint32_t>(v);
            h ^= u;
            h *= 1099511628211ull;
        };
        mix(k.x);
        mix(k.y);
        mix(k.z);
        return static_cast<size_t>(h);
    }
};

using QuantVertexIndex =
    std::unordered_map<QuantKey, std::vector<MCGAL::Vertex*>, QuantKeyHash>;

QuantKey quantKeyFromQ(const std::array<int, 3>& q) {
    return QuantKey{q[0], q[1], q[2]};
}

QuantKey quantKeyFromVertex(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Vertex* v) {
    MCGAL::PointInt q = mesh->floatPosToInt(v->point());
    return QuantKey{q[0], q[1], q[2]};
}

void indexLiveVertex(std::shared_ptr<MCGAL::Mesh> mesh,
                     QuantVertexIndex& index,
                     MCGAL::Vertex* v) {
    if (!mesh || !v || v->isRemoved()) return;
    index[quantKeyFromVertex(mesh, v)].push_back(v);
}

QuantVertexIndex buildLiveVertexIndex(std::shared_ptr<MCGAL::Mesh> mesh) {
    QuantVertexIndex index;
    if (!mesh) return index;
    index.reserve(mesh->vertices().size() * 2 + 1);
    for (MCGAL::Vertex* v : mesh->vertices()) {
        indexLiveVertex(mesh, index, v);
    }
    return index;
}

MCGAL::Vertex* findLiveVertexByQIndexed(const QuantVertexIndex& index,
                                        const std::array<int, 3>& q) {
    auto findExact = [&](const QuantKey& key) -> MCGAL::Vertex* {
        auto it = index.find(key);
        if (it == index.end()) return nullptr;
        for (MCGAL::Vertex* v : it->second) {
            if (v && !v->isRemoved()) return v;
        }
        return nullptr;
    };
    const QuantKey exact = quantKeyFromQ(q);
    if (MCGAL::Vertex* v = findExact(exact)) return v;

    // Parquet base-mesh float round-trips can move extrema by one quantization
    // unit after floatPosToInt, especially at 0 / 2^bits boundaries. Ring side
    // info is still authoritative, so use a one-cell lookup tolerance only when
    // exact lookup failed.
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) continue;
                if (MCGAL::Vertex* v =
                        findExact(QuantKey{q[0] + dx, q[1] + dy, q[2] + dz})) {
                    return v;
                }
            }
        }
    }
    return nullptr;
}

MCGAL::Halfedge* findInputBoundaryBySeedQIndexed(
    const QuantVertexIndex& index,
    const std::array<int, 6>& seedQ) {
    std::array<int, 3> qa{seedQ[0], seedQ[1], seedQ[2]};
    std::array<int, 3> qb{seedQ[3], seedQ[4], seedQ[5]};
    MCGAL::Vertex* a = findLiveVertexByQIndexed(index, qa);
    MCGAL::Vertex* b = findLiveVertexByQIndexed(index, qb);
    if (!a || !b) return nullptr;
    for (MCGAL::Halfedge* h : boundedVertexHalfedges(a)) {
        if (h && !h->isRemoved() && h->end_vertex() == b && isWalkGate(h)) {
            return h;
        }
    }
    for (MCGAL::Halfedge* h : boundedVertexHalfedges(b)) {
        if (h && !h->isRemoved() && h->end_vertex() == a && isWalkGate(h)) {
            MCGAL::Halfedge* opp = h->opposite();
            return isWalkGate(opp) ? opp : h;
        }
    }
    return nullptr;
}

MCGAL::Halfedge* findInputBoundaryBySeedQ(std::shared_ptr<MCGAL::Mesh> mesh,
                                          const std::array<int, 6>& seedQ) {
    if (!mesh) return nullptr;
    MCGAL::Halfedge* reversedFallback = nullptr;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
            if (!isWalkGate(h)) continue;
            const bool forward =
                sameQuant(mesh, h->vertex(), seedQ[0], seedQ[1], seedQ[2]) &&
                sameQuant(mesh, h->end_vertex(), seedQ[3], seedQ[4], seedQ[5]);
            if (forward) return h;
            const bool reversed =
                sameQuant(mesh, h->vertex(), seedQ[3], seedQ[4], seedQ[5]) &&
                sameQuant(mesh, h->end_vertex(), seedQ[0], seedQ[1], seedQ[2]);
            if (reversed && !reversedFallback) {
                MCGAL::Halfedge* opp = h->opposite();
                reversedFallback = isWalkGate(opp) ? opp : h;
            }
        }
    }
    return reversedFallback;
}

MCGAL::Halfedge* pickWalkLongestEdge(std::shared_ptr<MCGAL::Mesh> mesh) {
    if (!mesh) return nullptr;
    float maxLen = 0.0f;
    MCGAL::Halfedge* best = nullptr;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
            if (!isWalkGate(h)) continue;
            const float len = h->length();
            if (!best || len > maxLen) {
                maxLen = len;
                best = gateFromInputBoundary(h);
            }
        }
    }
    return best;
}

void resetWalkConquestFlags(std::shared_ptr<MCGAL::Mesh> mesh, bool clearTags) {
    if (!mesh) return;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (f && !f->isRemoved()) f->setUnProcessed();
    }
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        v->setUnConquered();
        if (clearTags) v->clearTag();
    }
}

int traceOpLimit() {
    if (const char* e = std::getenv("PMSF_PCLTTM_TRACE_OPS")) {
        int n = std::atoi(e);
        if (n > 0) return n;
    }
    return 0;
}

bool shouldTraceOp(int opIndex) {
    const int lim = traceOpLimit();
    if (lim <= 0 || opIndex < 0) return false;
    int start = 0;
    if (const char* e = std::getenv("PMSF_PCLTTM_TRACE_START")) {
        start = std::max(0, std::atoi(e));
    }
    return opIndex >= start && opIndex < start + lim;
}

std::string qstr(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Vertex* v) {
    if (!mesh || !v || v->isRemoved()) return "nil";
    MCGAL::PointInt q = mesh->floatPosToInt(v->point());
    return std::to_string(q[0]) + "," + std::to_string(q[1]) + "," +
           std::to_string(q[2]);
}

void traceDecoderOp(std::shared_ptr<MCGAL::Mesh> mesh,
                    int opIndex,
                    int phase,
                    int valence,
                    MCGAL::Vertex* inputA,
                    MCGAL::Vertex* inputB,
                    const std::vector<MCGAL::Vertex*>& ringVerts) {
    if (!shouldTraceOp(opIndex) || !mesh) return;
    auto qstr = [&](MCGAL::Vertex* v) {
        if (!v || v->isRemoved()) return std::string("nil");
        MCGAL::PointInt q = mesh->floatPosToInt(v->point());
        return std::to_string(q[0]) + "," + std::to_string(q[1]) + "," +
               std::to_string(q[2]);
    };
    std::fprintf(stderr,
                 "[PCLTTM trace dec] op=%d phase=%d valence=%d input=(%s)-(%s) ring=",
                 opIndex, phase, valence, qstr(inputA).c_str(), qstr(inputB).c_str());
    for (size_t i = 0; i < ringVerts.size(); ++i) {
        std::fprintf(stderr, "%s%s", i ? ";" : "", qstr(ringVerts[i]).c_str());
    }
    std::fprintf(stderr, "\n");
}

void traceDecoderGate(std::shared_ptr<MCGAL::Mesh> mesh,
                      int symbolIndex,
                      int phase,
                      const char* kind,
                      MCGAL::Halfedge* gate,
                      int valence) {
    if (!shouldTraceOp(symbolIndex) || !mesh) return;
    auto qstr = [&](MCGAL::Vertex* v) {
        if (!v || v->isRemoved()) return std::string("nil");
        MCGAL::PointInt q = mesh->floatPosToInt(v->point());
        return std::to_string(q[0]) + "," + std::to_string(q[1]) + "," +
               std::to_string(q[2]);
    };
    MCGAL::Halfedge* input = inputBoundaryFromGate(gate);
    std::fprintf(stderr,
                 "[PCLTTM trace dec gate] si=%d phase=%d kind=%s valence=%d input=(%s)-(%s)",
                 symbolIndex, phase, kind, valence,
                 input ? qstr(input->vertex()).c_str() : "nil",
                 input ? qstr(input->end_vertex()).c_str() : "nil");
    std::fprintf(stderr, "\n");
}

void pushWalkNullNeighbors(MCGAL::Halfedge* gateH,
                           RDPatchGateQueue& q,
                           std::set<MCGAL::Halfedge*>& enqueued,
                           std::set<MCGAL::Facet*>& facesEnqueued,
                           const std::set<MCGAL::Facet*>& nullPatchFaces) {
    if (!isWalkGate(gateH)) return;
    MCGAL::Facet* face = gateH->face();
    if (!face || face->isRemoved()) return;
    MCGAL::Halfedge* input = inputBoundaryFromGate(gateH);
    if (!input || input->isRemoved() || input->face() != face) return;
    MCGAL::Halfedge* he = input->next();
    int guard = 0;
    while (he && he != input && guard++ < 8) {
        if (he->isRemoved()) {
            he = he->next();
            continue;
        }
        MCGAL::Halfedge* nb = he->opposite();
        pushWalkGate(gateFromInputBoundary(nb), q, enqueued, facesEnqueued, nullPatchFaces);
        he = he->next();
    }
}

void pushWalkCenterFaces(MCGAL::Vertex* center,
                         RDPatchGateQueue& q,
                         std::set<MCGAL::Halfedge*>& enqueued,
                         std::set<MCGAL::Facet*>& facesEnqueued,
                         const std::set<MCGAL::Facet*>& nullPatchFaces) {
    if (!center || center->isRemoved()) return;
    for (MCGAL::Halfedge* h : boundedVertexHalfedges(center)) {
        if (!h || h->isRemoved()) continue;
        if (h->face() && !h->face()->isRemoved()) pushWalkGate(h, q, enqueued, facesEnqueued, nullPatchFaces);
        if (h->opposite() && h->opposite()->face() && !h->opposite()->face()->isRemoved()) {
            pushWalkGate(h->opposite(), q, enqueued, facesEnqueued, nullPatchFaces);
        }
    }
}

int symbolValence(RDPatchSymbolKind k) {
    switch (k) {
        case RDPatchSymbolKind::V_3: return 3;
        case RDPatchSymbolKind::V_4: return 4;
        case RDPatchSymbolKind::V_5: return 5;
        case RDPatchSymbolKind::V_6: return 6;
        default: return 0;
    }
}

bool faceContainsVertex(MCGAL::Facet* f, MCGAL::Vertex* v) {
    if (!f || !v || f->isRemoved()) return false;
    for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
        if (!h || h->isRemoved()) continue;
        if (h->vertex() == v || h->end_vertex() == v) return true;
    }
    return false;
}

bool faceAllMinus(MCGAL::Facet* f) {
    if (!f || f->isRemoved()) return false;
    bool any = false;
    for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
        MCGAL::Vertex* v = h ? h->vertex() : nullptr;
        if (!v || v->isRemoved()) return false;
        any = true;
        if (v->getTag() != MCGAL::Vertex::Minus) return false;
    }
    return any;
}

bool boundaryHasTripleOrder(const std::vector<MCGAL::Vertex*>& boundary,
                            MCGAL::Vertex* a,
                            MCGAL::Vertex* b,
                            MCGAL::Vertex* c) {
    if (!a || !b || !c || boundary.size() < 3) return false;
    const int n = static_cast<int>(boundary.size());
    for (int i = 0; i < n; ++i) {
        if (boundary[static_cast<size_t>(i)] == a &&
            boundary[static_cast<size_t>((i + 1) % n)] == b &&
            boundary[static_cast<size_t>((i + 2) % n)] == c) {
            return true;
        }
    }
    return false;
}

bool sameUndirectedEdge(MCGAL::Halfedge* h,
                        MCGAL::Vertex* a,
                        MCGAL::Vertex* b) {
    if (!h || !a || !b) return false;
    return (h->vertex() == a && h->end_vertex() == b) ||
           (h->vertex() == b && h->end_vertex() == a);
}

std::vector<MCGAL::Facet*> anchorNeighbors(MCGAL::Facet* f, MCGAL::Vertex* anchor) {
    std::vector<MCGAL::Facet*> out;
    if (!f || !anchor || f->isRemoved()) return out;
    for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
        if (!h || h->isRemoved()) continue;
        if (h->vertex() != anchor && h->end_vertex() != anchor) continue;
        MCGAL::Halfedge* opp = h->opposite();
        MCGAL::Facet* nf = opp ? opp->face() : nullptr;
        if (nf && !nf->isRemoved() && nf != f && faceContainsVertex(nf, anchor)) {
            out.push_back(nf);
        }
    }
    return out;
}

struct PatchCandidate {
    std::vector<MCGAL::Facet*> faces;
    std::vector<MCGAL::Halfedge*> internal;
    std::vector<MCGAL::Vertex*> boundary;
    bool usedAltFanWalk = false;
};

int faceDegree(MCGAL::Facet* f);

MCGAL::Facet* mergePatchToPolygon(const PatchCandidate& cand,
                                  int valence);

bool hasBoundedFaceLoop(MCGAL::Facet* f, int maxSteps);

std::vector<MCGAL::Halfedge*> boundedFaceHalfedges(MCGAL::Facet* f, int maxSteps) {
    std::vector<MCGAL::Halfedge*> out;
    if (!f || f->isRemoved() || !f->proxyHalfedge()) return out;
    MCGAL::Halfedge* start = f->proxyHalfedge();
    MCGAL::Halfedge* h = start;
    int guard = 0;
    do {
        if (!h || h->isRemoved()) return {};
        out.push_back(h);
        h = h->next();
        ++guard;
    } while (h && h != start && guard < maxSteps);
    if (h != start) out.clear();
    return out;
}

MCGAL::Halfedge* boundedPrevInFace(MCGAL::Halfedge* h, int maxSteps = 256) {
    if (!h || h->isRemoved() || maxSteps <= 0) return nullptr;
    MCGAL::Halfedge* g = h;
    for (int guard = 0; guard < maxSteps; ++guard) {
        if (!g || g->isRemoved() || !g->next()) return nullptr;
        if (g->next() == h) return g;
        g = g->next();
    }
    return nullptr;
}

std::vector<MCGAL::Halfedge*> boundedVertexHalfedges(MCGAL::Vertex* v,
                                                     int maxSteps) {
    std::vector<MCGAL::Halfedge*> out;
    if (!v || v->isRemoved() || !v->proxyHalfedge() || maxSteps <= 0) return out;

    std::unordered_set<MCGAL::Halfedge*> seen;
    seen.reserve(static_cast<size_t>(maxSteps));
    auto append = [&](MCGAL::Halfedge* h) -> bool {
        if (!h || h->isRemoved()) return false;
        if (!seen.insert(h).second) return false;
        out.push_back(h);
        return true;
    };
    auto stepCw = [](MCGAL::Halfedge* h) -> MCGAL::Halfedge* {
        if (!h) return nullptr;
        MCGAL::Halfedge* prev = boundedPrevInFace(h);
        return prev ? prev->opposite() : nullptr;
    };
    auto stepCcw = [](MCGAL::Halfedge* h) -> MCGAL::Halfedge* {
        if (!h || !h->opposite()) return nullptr;
        return h->opposite()->next();
    };

    MCGAL::Halfedge* start = v->proxyHalfedge();
    append(start);

    MCGAL::Halfedge* h = start;
    for (int guard = 0; guard < maxSteps; ++guard) {
        h = stepCw(h);
        if (!h || h == start || seen.count(h)) break;
        append(h);
    }

    h = stepCcw(start);
    for (int guard = 0; guard < maxSteps; ++guard) {
        if (!h || h == start || seen.count(h)) break;
        append(h);
        h = stepCcw(h);
    }
    return out;
}

std::vector<MCGAL::Vertex*> orderedFaceVertices(MCGAL::Facet* f) {
    std::vector<MCGAL::Vertex*> out;
    for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
        if (!h->vertex()) return {};
        out.push_back(h->vertex());
    }
    return out;
}

MCGAL::Halfedge* faceHalfedgeEndingAt(MCGAL::Facet* f, MCGAL::Vertex* v) {
    if (!f || !v || f->isRemoved() || !f->proxyHalfedge()) return nullptr;
    for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
        if (h && !h->isRemoved() && h->end_vertex() == v) return h;
    }
    return nullptr;
}

MCGAL::Halfedge* faceHalfedgeBetweenOriented(MCGAL::Facet* f,
                                             MCGAL::Vertex* a,
                                             MCGAL::Vertex* b) {
    if (!f || !a || !b || f->isRemoved() || !f->proxyHalfedge()) return nullptr;
    for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
        if (h && !h->isRemoved() && h->vertex() == a && h->end_vertex() == b) {
            return h;
        }
    }
    return nullptr;
}

MCGAL::Halfedge* exteriorHalfedgeOnPolygonEdge(MCGAL::Facet* poly,
                                               MCGAL::Vertex* a,
                                               MCGAL::Vertex* b) {
    if (!poly || !a || !b) return nullptr;
    for (MCGAL::Halfedge* h : boundedFaceHalfedges(poly)) {
        if (!h || h->isRemoved()) continue;
        if (h->vertex() == a && h->end_vertex() == b) return h->opposite();
        if (h->vertex() == b && h->end_vertex() == a) return h->opposite();
    }
    return nullptr;
}

bool faceHasVertexOutsideSet(MCGAL::Facet* f,
                             const std::set<MCGAL::Vertex*>& verts) {
    if (!f || f->isRemoved()) return false;
    for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
        if (!h || h->isRemoved() || !h->vertex()) continue;
        if (!verts.count(h->vertex())) return true;
    }
    return false;
}

bool faceAllVerticesInSet(MCGAL::Facet* f,
                          const std::set<MCGAL::Vertex*>& verts) {
    if (!f || f->isRemoved()) return false;
    bool any = false;
    for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
        if (!h || h->isRemoved() || !h->vertex()) return false;
        any = true;
        if (!verts.count(h->vertex())) return false;
    }
    return any;
}

MCGAL::Halfedge* boundedFindHalfedge(MCGAL::Vertex* a,
                                     MCGAL::Vertex* b,
                                     int maxSteps = 256) {
    if (!a || !b || a->isRemoved() || b->isRemoved()) return nullptr;
    for (MCGAL::Halfedge* h : boundedVertexHalfedges(a, maxSteps)) {
        if (h && !h->isRemoved() && h->end_vertex() == b) return h;
    }
    return nullptr;
}

MCGAL::Halfedge* findExteriorHalfedgeBetween(MCGAL::Vertex* a,
                                             MCGAL::Vertex* b) {
    if (!a || !b || a->isRemoved() || b->isRemoved()) return nullptr;
    MCGAL::Halfedge* ab = boundedFindHalfedge(a, b);
    MCGAL::Halfedge* ba = boundedFindHalfedge(b, a);
    MCGAL::Halfedge* candidates[4] = {
        ab,
        ab ? ab->opposite() : nullptr,
        ba,
        ba ? ba->opposite() : nullptr,
    };
    for (MCGAL::Halfedge* h : candidates) {
        if (!h || h->isRemoved()) continue;
        MCGAL::Facet* f = h->face();
        if (f && !f->isRemoved() && !f->isProcessed()) return h;
    }
    for (MCGAL::Halfedge* h : candidates) {
        if (!h || h->isRemoved()) continue;
        MCGAL::Facet* f = h->face();
        if (f && !f->isRemoved()) return h;
    }
    return nullptr;
}

MCGAL::Halfedge* findExteriorHalfedgeBetween(MCGAL::Vertex* a,
                                             MCGAL::Vertex* b,
                                             const std::set<MCGAL::Vertex*>& patchVerts) {
    if (!a || !b || a->isRemoved() || b->isRemoved()) return nullptr;
    MCGAL::Halfedge* ab = boundedFindHalfedge(a, b);
    MCGAL::Halfedge* ba = boundedFindHalfedge(b, a);
    MCGAL::Halfedge* candidates[4] = {
        ab,
        ab ? ab->opposite() : nullptr,
        ba,
        ba ? ba->opposite() : nullptr,
    };
    for (MCGAL::Halfedge* h : candidates) {
        if (!h || h->isRemoved()) continue;
        MCGAL::Facet* f = h->face();
        if (f && !f->isRemoved() && faceHasVertexOutsideSet(f, patchVerts)) return h;
    }
    return findExteriorHalfedgeBetween(a, b);
}

void applyStrictBoundaryTags(std::vector<MCGAL::Vertex*>& ring) {
    const int n = static_cast<int>(ring.size());
    if (n < 3) return;
    auto paper = paperRetriangulateFig9(
        n,
        ring.front() ? static_cast<int>(ring.front()->getTag()) : 1,
        ring.back() ? static_cast<int>(ring.back()->getTag()) : 2);
    for (MCGAL::Vertex* v : ring) {
        if (v && !v->isRemoved()) v->setConquered();
    }
    for (int idx : paper.plusTagIdx) {
        if (idx >= 0 && idx < n && ring[static_cast<size_t>(idx)]) {
            MCGAL::Vertex* v = ring[static_cast<size_t>(idx)];
            if (v->getTag() == MCGAL::Vertex::Untagged) v->setTag(MCGAL::Vertex::Plus);
        }
    }
    for (int idx : paper.minusTagIdx) {
        if (idx >= 0 && idx < n && ring[static_cast<size_t>(idx)]) {
            MCGAL::Vertex* v = ring[static_cast<size_t>(idx)];
            if (v->getTag() == MCGAL::Vertex::Untagged) v->setTag(MCGAL::Vertex::Minus);
        }
    }
}

void setStrictTagIfUnset(MCGAL::Vertex* v, int tag) {
    if (!v || v->isRemoved() || v->getTag() != MCGAL::Vertex::Untagged) return;
    if (tag == static_cast<int>(MCGAL::Vertex::Plus)) {
        v->setTag(MCGAL::Vertex::Plus);
    } else if (tag == static_cast<int>(MCGAL::Vertex::Minus)) {
        v->setTag(MCGAL::Vertex::Minus);
    }
}

int mepp2TypeFromInputBoundary(MCGAL::Halfedge* input, int valence) {
    if (!input || input->isRemoved() || !input->vertex() || !input->end_vertex()) {
        return 0;
    }
    return paperMepp2TypeFromBoundaryTags(
        valence,
        static_cast<int>(input->vertex()->getTag()),
        static_cast<int>(input->end_vertex()->getTag()));
}

bool applyStrictBoundaryTagsFromPaperInput(MCGAL::Halfedge* input, int valence) {
    if (!input || input->isRemoved() || valence < 3 || valence > 6) return false;
    MCGAL::Facet* f = input->face();
    if (!f || f->isRemoved()) return false;
    const int type = mepp2TypeFromInputBoundary(input, valence);
    if (type == 0) return false;
    for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
        if (h && h->vertex() && !h->vertex()->isRemoved()) h->vertex()->setConquered();
    }
    MCGAL::Halfedge* g = input->next();
    for (int tag : paperMepp2InteriorTags(type)) {
        if (!g || g->isRemoved() || !g->end_vertex()) break;
        setStrictTagIfUnset(g->end_vertex(), tag);
        g = g->next();
    }
    return true;
}

void applyStrictNullGateTag(MCGAL::Halfedge* h) {
    if (!h || h->isRemoved() || !h->end_vertex()) return;
    MCGAL::Halfedge* input = inputBoundaryFromGate(h);
    if (!input || input->isRemoved() || !input->vertex() || !input->end_vertex()) return;
    MCGAL::Vertex* front = h->end_vertex();
    if (!front || front->isRemoved() || front->getTag() != MCGAL::Vertex::Untagged) return;
    const bool bothPlus =
        input->vertex()->getTag() == MCGAL::Vertex::Plus &&
        input->end_vertex()->getTag() == MCGAL::Vertex::Plus;
    front->setTag(bothPlus ? MCGAL::Vertex::Minus : MCGAL::Vertex::Plus);
}

void pushDecodedPaperDecimatingGates(std::shared_ptr<MCGAL::Mesh> mesh,
                                     const std::vector<MCGAL::Vertex*>& ring,
                                     MCGAL::Vertex* insertedCenter,
                                     MCGAL::Vertex* inputA,
                                     MCGAL::Vertex* inputB,
                                     RDPatchGateQueue& q,
                                     std::set<MCGAL::Halfedge*>& enqueued,
                                     std::set<MCGAL::Facet*>& facesEnqueued,
                                     const std::set<MCGAL::Facet*>& nullPatchFaces) {
    const int n = static_cast<int>(ring.size());
    if (n < 3) return;
    std::set<MCGAL::Vertex*> patchVerts;
    for (MCGAL::Vertex* v : ring) {
        if (v && !v->isRemoved()) patchVerts.insert(v);
    }
    if (insertedCenter && !insertedCenter->isRemoved()) {
        patchVerts.insert(insertedCenter);
    }
    for (int i = 0; i < n; ++i) {
        MCGAL::Vertex* a = ring[static_cast<size_t>(i)];
        MCGAL::Vertex* b = ring[static_cast<size_t>((i + 1) % n)];
        if (!a || !b || a->isRemoved() || b->isRemoved()) continue;
        if ((a == inputA && b == inputB) || (a == inputB && b == inputA)) continue;
        MCGAL::Halfedge* gate =
            gateFromInputBoundary(findExteriorHalfedgeBetween(a, b, patchVerts));
        if (std::getenv("PMSF_PCLTTM_TRACE_PUSH")) {
            MCGAL::Facet* gf = gate ? gate->face() : nullptr;
            MCGAL::Halfedge* in = gate ? inputBoundaryFromGate(gate) : nullptr;
            std::fprintf(stderr,
                         "[PCLTTM trace dec push] edge=(%s)-(%s) gate_input=(%s)-(%s) "
                         "usable=%d faceProcessed=%d already=%d faceQueued=%d\n",
                         qstr(mesh, a).c_str(), qstr(mesh, b).c_str(),
                         in ? qstr(mesh, in->vertex()).c_str() : "nil",
                         in ? qstr(mesh, in->end_vertex()).c_str() : "nil",
                         isWalkGate(gate) ? 1 : 0,
                         gf && gf->isProcessed() ? 1 : 0,
                         enqueued.count(gate) ? 1 : 0,
                         gf && facesEnqueued.count(gf) ? 1 : 0);
        }
        pushWalkGate(gate, q, enqueued, facesEnqueued, nullPatchFaces);
    }
}

std::vector<MCGAL::Halfedge*> collectDecodedPaperDecimatingGatesFromPolygon(
    MCGAL::Facet* patchFace,
    MCGAL::Vertex* inputA,
    MCGAL::Vertex* inputB) {
    std::vector<MCGAL::Halfedge*> out;
    if (!patchFace || patchFace->isRemoved()) return out;
    for (MCGAL::Halfedge* boundary : boundedFaceHalfedges(patchFace)) {
        if (!boundary || boundary->isRemoved()) continue;
        MCGAL::Vertex* a = boundary->vertex();
        MCGAL::Vertex* b = boundary->end_vertex();
        if (!a || !b || a->isRemoved() || b->isRemoved()) continue;
        if ((a == inputA && b == inputB) || (a == inputB && b == inputA)) continue;
        MCGAL::Halfedge* exteriorBoundary = boundary->opposite();
        if (!exteriorBoundary || exteriorBoundary->isRemoved()) continue;
        MCGAL::Facet* exteriorFace = exteriorBoundary->face();
        if (!exteriorFace || exteriorFace->isRemoved() || exteriorFace->isProcessed()) continue;
        MCGAL::Halfedge* gate = gateFromInputBoundary(exteriorBoundary);
        if (gate && !gate->isRemoved()) out.push_back(gate);
    }
    return out;
}

void pushDecodedPaperDecimatingGateList(std::shared_ptr<MCGAL::Mesh> mesh,
                                        const std::vector<MCGAL::Halfedge*>& gates,
                                        RDPatchGateQueue& q,
                                        std::set<MCGAL::Halfedge*>& enqueued,
                                        std::set<MCGAL::Facet*>& facesEnqueued,
                                        const std::set<MCGAL::Facet*>& nullPatchFaces,
                                        std::map<MCGAL::Halfedge*, std::array<int, 6>>*
                                            protectedPaperGates = nullptr,
                                        std::set<MCGAL::Facet*>*
                                            processedSkippedQueuedFaces = nullptr,
                                        std::set<MCGAL::Facet*>*
                                            processedAlreadyQueuedFaces = nullptr) {
    for (MCGAL::Halfedge* gate : gates) {
        MCGAL::Halfedge* in = gate ? inputBoundaryFromGate(gate) : nullptr;
        MCGAL::Facet* gf = gate ? gate->face() : nullptr;
        if (std::getenv("PMSF_PCLTTM_TRACE_PUSH")) {
            std::fprintf(stderr,
                         "[PCLTTM trace dec push] edge=(%s)-(%s) gate_input=(%s)-(%s) "
                         "usable=%d faceProcessed=%d already=%d faceQueued=%d\n",
                         in ? qstr(mesh, in->end_vertex()).c_str() : "nil",
                         in ? qstr(mesh, in->vertex()).c_str() : "nil",
                         in ? qstr(mesh, in->vertex()).c_str() : "nil",
                         in ? qstr(mesh, in->end_vertex()).c_str() : "nil",
                         isWalkGate(gate) ? 1 : 0,
                         gf && gf->isProcessed() ? 1 : 0,
                         enqueued.count(gate) ? 1 : 0,
                         gf && facesEnqueued.count(gf) ? 1 : 0);
        }
        if (gf && gf->isProcessed()) continue;
        if (!isWalkGate(gate)) continue;
        if (enqueued.count(gate)) {
            if (processedAlreadyQueuedFaces && gf && facesEnqueued.count(gf) &&
                !gf->isProcessed()) {
                gf->setProcessedFlag();
                processedAlreadyQueuedFaces->insert(gf);
            }
            continue;
        }
        const bool faceAlreadyQueued = gf && facesEnqueued.count(gf);
        if (faceAlreadyQueued) {
            if (!protectedPaperGates) {
                if (processedSkippedQueuedFaces && gf && !gf->isProcessed()) {
                    gf->setProcessedFlag();
                    processedSkippedQueuedFaces->insert(gf);
                }
                continue;
            }
            if (!in || !in->vertex() || !in->end_vertex()) continue;
            MCGAL::PointInt qa = mesh->floatPosToInt(in->vertex()->point());
            MCGAL::PointInt qb = mesh->floatPosToInt(in->end_vertex()->point());
            (*protectedPaperGates)[gate] =
                {qa[0], qa[1], qa[2], qb[0], qb[1], qb[2]};
        }
        enqueued.insert(gate);
        if (gf) facesEnqueued.insert(gf);
        q.push(gate, walkBucket(gate, nullPatchFaces));
    }
}

void markDecodedPatchInteriorFacesProcessed(const std::vector<MCGAL::Vertex*>& ring,
                                            MCGAL::Vertex* insertedCenter,
                                            std::set<MCGAL::Facet*>& facesProcessed) {
    std::set<MCGAL::Vertex*> patchVerts;
    for (MCGAL::Vertex* v : ring) {
        if (v && !v->isRemoved()) patchVerts.insert(v);
    }
    if (insertedCenter && !insertedCenter->isRemoved()) {
        patchVerts.insert(insertedCenter);
    }
    if (patchVerts.empty()) return;
    std::set<MCGAL::Facet*> seen;
    for (MCGAL::Vertex* v : patchVerts) {
        for (MCGAL::Halfedge* hv : boundedVertexHalfedges(v)) {
            MCGAL::Facet* f = hv ? hv->face() : nullptr;
            if (!f || f->isRemoved() || !seen.insert(f).second) continue;
            if (!faceAllVerticesInSet(f, patchVerts)) continue;
            f->setProcessedFlag();
            facesProcessed.insert(f);
        }
    }
}

void markStaleOneBoundaryFacesProcessed(const std::vector<MCGAL::Vertex*>& ring,
                                        std::set<MCGAL::Facet*>& facesProcessed) {
    std::set<MCGAL::Vertex*> boundary;
    for (MCGAL::Vertex* v : ring) {
        if (v && !v->isRemoved()) boundary.insert(v);
    }
    if (boundary.empty()) return;
    std::set<MCGAL::Facet*> seen;
    for (MCGAL::Vertex* bv : boundary) {
        for (MCGAL::Halfedge* hv : boundedVertexHalfedges(bv)) {
            MCGAL::Facet* f = hv ? hv->face() : nullptr;
            if (!f || f->isRemoved() || f->isProcessed() || !seen.insert(f).second) {
                continue;
            }
            int boundaryCount = 0;
            bool allNonBoundaryConquered = true;
            for (MCGAL::Halfedge* fh : boundedFaceHalfedges(f)) {
                MCGAL::Vertex* v = fh ? fh->vertex() : nullptr;
                if (!v || v->isRemoved()) {
                    allNonBoundaryConquered = false;
                    break;
                }
                if (boundary.count(v)) {
                    ++boundaryCount;
                } else if (!v->isConquered() && v->getTag() == MCGAL::Vertex::Untagged) {
                    allNonBoundaryConquered = false;
                    break;
                }
            }
            if (boundaryCount == 1 && allNonBoundaryConquered) {
                f->setProcessedFlag();
                facesProcessed.insert(f);
            }
        }
    }
}

void pushDecodedPaperCleaningGates(const std::vector<MCGAL::Vertex*>& ring,
                                   MCGAL::Facet* poly,
                                   RDPatchGateQueue& q,
                                   std::set<MCGAL::Halfedge*>& enqueued,
                                   std::set<MCGAL::Facet*>& facesEnqueued,
                                   std::set<MCGAL::Facet*>& facesProcessed,
                                   const std::set<MCGAL::Facet*>& nullPatchFaces) {
    const int n = static_cast<int>(ring.size());
    if (n < 3) return;
    for (int i = 0; i < n; ++i) {
        MCGAL::Vertex* a = ring[static_cast<size_t>(i)];
        MCGAL::Vertex* b = ring[static_cast<size_t>((i + 1) % n)];
        if (!a || !b || a->isRemoved() || b->isRemoved()) continue;
        MCGAL::Halfedge* exteriorGate = exteriorHalfedgeOnPolygonEdge(poly, a, b);
        MCGAL::Facet* exterior = exteriorGate ? exteriorGate->face() : nullptr;
        if (!exterior || exterior->isRemoved()) continue;
        exterior->setProcessedFlag();
        facesProcessed.insert(exterior);
        for (MCGAL::Halfedge* he : boundedFaceHalfedges(exterior)) {
            if (!he || he->isRemoved() || he == exteriorGate) continue;
            pushWalkGate(gateFromInputBoundary(he->opposite()), q, enqueued,
                         facesEnqueued, nullPatchFaces);
        }
    }
}

bool validatePatchCandidate(const std::vector<MCGAL::Facet*>& faces,
                            int valence,
                            MCGAL::Vertex* anchor,
                            const std::set<MCGAL::Facet*>* forbiddenFaces,
                            MCGAL::Halfedge* requiredBoundaryGate,
                            PatchCandidate& cand) {
    if (static_cast<int>(faces.size()) != valence - 2) return false;
    std::set<MCGAL::Facet*> faceSet(faces.begin(), faces.end());
    std::set<MCGAL::Vertex*> boundaryVerts;
    std::vector<MCGAL::Halfedge*> boundaryEdges;
    std::vector<MCGAL::Halfedge*> internalEdges;

    for (MCGAL::Facet* f : faces) {
        if (!f || f->isRemoved()) return false;
        if (forbiddenFaces && forbiddenFaces->count(f)) return false;
        auto halfedges = boundedFaceHalfedges(f);
        if (halfedges.empty()) return false;
        for (MCGAL::Halfedge* h : halfedges) {
            if (!h || h->isRemoved() || !h->vertex() || !h->end_vertex()) return false;
            MCGAL::Halfedge* opp = h->opposite();
            MCGAL::Facet* of = opp ? opp->face() : nullptr;
            if (of && faceSet.count(of)) {
                if (anchor && h->vertex() != anchor && h->end_vertex() != anchor) {
                    return false;
                }
                if (reinterpret_cast<std::uintptr_t>(h) <
                    reinterpret_cast<std::uintptr_t>(opp)) {
                    internalEdges.push_back(h);
                }
            } else {
                boundaryEdges.push_back(h);
                boundaryVerts.insert(h->vertex());
                boundaryVerts.insert(h->end_vertex());
            }
        }
    }

    if (static_cast<int>(boundaryEdges.size()) != valence) return false;
    if (requiredBoundaryGate) {
        bool hasGateBoundary = false;
        for (MCGAL::Halfedge* e : boundaryEdges) {
            if (sameUndirectedEdge(e,
                                   requiredBoundaryGate->vertex(),
                                   requiredBoundaryGate->end_vertex())) {
                hasGateBoundary = true;
                break;
            }
        }
        if (!hasGateBoundary) return false;
    }
    if (static_cast<int>(boundaryVerts.size()) != valence) return false;
    if (static_cast<int>(internalEdges.size()) != valence - 3) return false;
    for (MCGAL::Vertex* v : boundaryVerts) {
        int bdeg = 0;
        for (MCGAL::Halfedge* e : boundaryEdges) {
            if (e->vertex() == v || e->end_vertex() == v) ++bdeg;
        }
        if (bdeg != 2) return false;
    }
    std::map<MCGAL::Vertex*, MCGAL::Vertex*> nextBoundary;
    std::map<MCGAL::Vertex*, int> inDegree;
    for (MCGAL::Halfedge* e : boundaryEdges) {
        MCGAL::Vertex* a = e->vertex();
        MCGAL::Vertex* b = e->end_vertex();
        if (!a || !b || nextBoundary.count(a)) return false;
        nextBoundary[a] = b;
        ++inDegree[b];
    }
    for (MCGAL::Vertex* v : boundaryVerts) {
        if (!nextBoundary.count(v) || inDegree[v] != 1) return false;
    }
    MCGAL::Vertex* start = *boundaryVerts.begin();
    MCGAL::Vertex* cur = start;
    for (int i = 0; i < valence; ++i) {
        auto it = nextBoundary.find(cur);
        if (it == nextBoundary.end()) return false;
        cur = it->second;
        if (i + 1 < valence && cur == start) return false;
    }
    if (cur != start) return false;
    std::vector<MCGAL::Vertex*> boundary;
    boundary.reserve(static_cast<size_t>(valence));
    cur = start;
    for (int i = 0; i < valence; ++i) {
        boundary.push_back(cur);
        cur = nextBoundary[cur];
    }
    cand.faces = faces;
    cand.internal = internalEdges;
    cand.boundary = std::move(boundary);
    return true;
}

bool appendMepp2PatchFace(MCGAL::Halfedge* h,
                          std::vector<MCGAL::Facet*>& faces,
                          const std::set<MCGAL::Facet*>* forbiddenFaces) {
    if (!h || h->isRemoved()) return false;
    MCGAL::Facet* f = h->face();
    if (!f || f->isRemoved()) return false;
    if (forbiddenFaces && forbiddenFaces->count(f)) return false;
    if (std::find(faces.begin(), faces.end(), f) != faces.end()) return false;
    faces.push_back(f);
    return true;
}

bool appendMepp2PatchFaceAcross(MCGAL::Halfedge* h,
                                std::vector<MCGAL::Facet*>& faces,
                                const std::set<MCGAL::Facet*>* forbiddenFaces) {
    if (!h || h->isRemoved() || !h->opposite() || h->opposite()->isRemoved()) {
        return false;
    }
    return appendMepp2PatchFace(h->opposite(), faces, forbiddenFaces);
}

bool discoverMepp2PatchFromInput(MCGAL::Halfedge* input,
                                 int valence,
                                 const std::set<MCGAL::Facet*>* forbiddenFaces,
                                 PatchCandidate& cand,
                                 int& typeOut) {
    typeOut = 0;
    if (!input || input->isRemoved() || valence < 3 || valence > 6) return false;
    if (!input->face() || input->face()->isRemoved()) return false;
    const int type = mepp2TypeFromInputBoundary(input, valence);
    if (type == 0) return false;

    std::vector<MCGAL::Facet*> faces;
    faces.reserve(static_cast<size_t>(valence - 2));
    if (!appendMepp2PatchFace(input, faces, forbiddenFaces)) return false;

    auto prev = [](MCGAL::Halfedge* h) -> MCGAL::Halfedge* {
        return h ? boundedPrevInFace(h) : nullptr;
    };
    auto opp = [](MCGAL::Halfedge* h) -> MCGAL::Halfedge* {
        return (h && h->opposite() && !h->opposite()->isRemoved()) ? h->opposite() : nullptr;
    };

    bool ok = true;
    if (valence == 3) {
        ok = true;
    } else if (type == 5 || type == 8) {
        ok = appendMepp2PatchFaceAcross(prev(input), faces, forbiddenFaces);
    } else if (type == 6 || type == 7) {
        ok = appendMepp2PatchFaceAcross(input->next(), faces, forbiddenFaces);
    } else if (type == 9 || type == 12) {
        MCGAL::Halfedge* g = opp(prev(input));
        ok = appendMepp2PatchFace(g, faces, forbiddenFaces);
        g = ok ? opp(g ? g->next() : nullptr) : nullptr;
        ok = ok && appendMepp2PatchFace(g, faces, forbiddenFaces);
    } else if (type == 10) {
        MCGAL::Halfedge* g = opp(input->next());
        ok = appendMepp2PatchFace(g, faces, forbiddenFaces);
        g = ok ? opp(prev(g)) : nullptr;
        ok = ok && appendMepp2PatchFace(g, faces, forbiddenFaces);
    } else if (type == 11) {
        ok = appendMepp2PatchFaceAcross(input->next(), faces, forbiddenFaces) &&
             appendMepp2PatchFaceAcross(prev(input), faces, forbiddenFaces);
    } else if (type == 13 || type == 16) {
        MCGAL::Halfedge* g = opp(prev(input));
        ok = appendMepp2PatchFace(g, faces, forbiddenFaces);
        ok = ok && appendMepp2PatchFaceAcross(g ? g->next() : nullptr,
                                              faces, forbiddenFaces);
        ok = ok && appendMepp2PatchFaceAcross(prev(g), faces, forbiddenFaces);
    } else if (type == 14 || type == 15) {
        MCGAL::Halfedge* g = opp(input->next());
        ok = appendMepp2PatchFace(g, faces, forbiddenFaces);
        ok = ok && appendMepp2PatchFaceAcross(g ? g->next() : nullptr,
                                              faces, forbiddenFaces);
        ok = ok && appendMepp2PatchFaceAcross(prev(g), faces, forbiddenFaces);
    } else {
        ok = false;
    }
    if (!ok) return false;
    PatchCandidate tmp;
    if (!validatePatchCandidate(faces, valence, nullptr, forbiddenFaces, input, tmp)) {
        return false;
    }
    cand = std::move(tmp);
    typeOut = type;
    return true;
}

MCGAL::Halfedge* joinMepp2Face(MCGAL::Halfedge* h) {
    if (!h || h->isRemoved() || !h->opposite() || h->opposite()->isRemoved()) return nullptr;
    if (!h->face() || h->face()->isRemoved() ||
        !h->opposite()->face() || h->opposite()->face()->isRemoved()) {
        return nullptr;
    }
    return MCGAL::join_face(h);
}

MCGAL::Facet* mergeMepp2PatchFromInput(MCGAL::Halfedge* input,
                                       int type,
                                       int valence) {
    if (!input || input->isRemoved() || !input->face() || input->face()->isRemoved()) {
        return nullptr;
    }
    if (valence == 3) {
        input->face()->proxyHalfedge(input);
        return input->face();
    }

    auto prev = [](MCGAL::Halfedge* h) -> MCGAL::Halfedge* {
        return h ? boundedPrevInFace(h) : nullptr;
    };
    auto opp = [](MCGAL::Halfedge* h) -> MCGAL::Halfedge* {
        return (h && h->opposite() && !h->opposite()->isRemoved()) ? h->opposite() : nullptr;
    };

    MCGAL::Halfedge* g = nullptr;
    switch (type) {
        case 5:
        case 8:
            g = joinMepp2Face(prev(input));
            if (!g) return nullptr;
            break;
        case 6:
        case 7:
            g = joinMepp2Face(input->next());
            if (!g) return nullptr;
            break;
        case 9:
        case 12:
            g = joinMepp2Face(prev(input));
            if (!g) return nullptr;
            g = joinMepp2Face(g ? g->next() : nullptr);
            if (!g) return nullptr;
            break;
        case 10:
            g = joinMepp2Face(opp(input->next()));
            if (!g) return nullptr;
            g = joinMepp2Face(g);
            if (!g) return nullptr;
            break;
        case 11:
            g = joinMepp2Face(input->next());
            if (!g) return nullptr;
            g = joinMepp2Face(prev(g));
            if (!g) return nullptr;
            break;
        case 13:
        case 16:
            g = joinMepp2Face(opp(prev(input)));
            if (!g) return nullptr;
            g = joinMepp2Face(g);
            if (!g) return nullptr;
            g = joinMepp2Face(g);
            if (!g) return nullptr;
            break;
        case 14:
        case 15:
            g = joinMepp2Face(opp(input->next()));
            if (!g) return nullptr;
            g = joinMepp2Face(g);
            if (!g) return nullptr;
            g = joinMepp2Face(g);
            if (!g) return nullptr;
            break;
        default:
            return nullptr;
    }

    MCGAL::Facet* poly = nullptr;
    if (input && !input->isRemoved() && input->face() && !input->face()->isRemoved()) {
        poly = input->face();
    } else if (g && !g->isRemoved() && g->face() && !g->face()->isRemoved()) {
        poly = g->face();
    }
    if (poly) poly->proxyHalfedge(input);
    return poly && faceDegree(poly) == valence ? poly : nullptr;
}

int patchTagScore(const PatchCandidate& cand, int valence) {
    if (static_cast<int>(cand.boundary.size()) != valence) return -1;
    int best = -1;
    for (int rev = 0; rev < 2; ++rev) {
        for (int rot = 0; rot < valence; ++rot) {
            std::vector<MCGAL::Vertex*> ring;
            ring.reserve(static_cast<size_t>(valence));
            for (int i = 0; i < valence; ++i) {
                int idx = rev ? (rot - i) : (rot + i);
                idx %= valence;
                if (idx < 0) idx += valence;
                ring.push_back(cand.boundary[static_cast<size_t>(idx)]);
            }
            auto paper = paperRetriangulateFig9(
                valence,
                ring.front() ? static_cast<int>(ring.front()->getTag()) : 1,
                ring.back() ? static_cast<int>(ring.back()->getTag()) : 2);
            std::vector<int> expected(static_cast<size_t>(valence), 2);
            for (int idx : paper.plusTagIdx) {
                if (idx >= 0 && idx < valence) expected[static_cast<size_t>(idx)] = 1;
            }
            for (int idx : paper.minusTagIdx) {
                if (idx >= 0 && idx < valence) expected[static_cast<size_t>(idx)] = 2;
            }
            int score = 0;
            for (int i = 0; i < valence; ++i) {
                MCGAL::Vertex* v = ring[static_cast<size_t>(i)];
                const int tag = v ? static_cast<int>(v->getTag()) : 0;
                if (tag == expected[static_cast<size_t>(i)]) score += 2;
                else if (tag == 0) score += 1;
            }
            best = std::max(best, score);
        }
    }
    return best;
}

bool candidateHasInternalEdge(const PatchCandidate& cand,
                              MCGAL::Vertex* a,
                              MCGAL::Vertex* b) {
    if (!a || !b) return false;
    for (MCGAL::Halfedge* h : cand.internal) {
        if (sameUndirectedEdge(h, a, b)) return true;
    }
    return false;
}

bool candidateHasConqueredNonInput(const PatchCandidate& cand,
                                   MCGAL::Halfedge* requiredInput) {
    if (!requiredInput) return false;
    MCGAL::Vertex* a = requiredInput->vertex();
    MCGAL::Vertex* b = requiredInput->end_vertex();
    for (MCGAL::Vertex* v : cand.boundary) {
        if (v && v != a && v != b && v->isConquered()) return true;
    }
    return false;
}

int patchFig9DiagonalScore(const PatchCandidate& cand,
                           int valence,
                           MCGAL::Halfedge* requiredInput = nullptr,
                           int inputOffsetCode = -1) {
    if (static_cast<int>(cand.boundary.size()) != valence) return -1;
    int best = -1;
    for (int rev = 0; rev < 2; ++rev) {
        for (int rot = 0; rot < valence; ++rot) {
            std::vector<MCGAL::Vertex*> ring;
            ring.reserve(static_cast<size_t>(valence));
            for (int i = 0; i < valence; ++i) {
                int idx = rev ? (rot - i) : (rot + i);
                idx %= valence;
                if (idx < 0) idx += valence;
                ring.push_back(cand.boundary[static_cast<size_t>(idx)]);
            }
            if (requiredInput && inputOffsetCode >= 0) {
                const int off = inputOffsetCode & 0x7;
                const bool reversed = (inputOffsetCode & 0x8) != 0;
                if (off < 0 || off >= valence) continue;
                MCGAL::Vertex* a = ring[static_cast<size_t>(off)];
                MCGAL::Vertex* b = ring[static_cast<size_t>((off + 1) % valence)];
                const bool ok = reversed
                                    ? (a == requiredInput->end_vertex() &&
                                       b == requiredInput->vertex())
                                    : (a == requiredInput->vertex() &&
                                       b == requiredInput->end_vertex());
                if (!ok) continue;
            }
            auto paper = paperRetriangulateFig9(
                valence,
                ring.front() ? static_cast<int>(ring.front()->getTag()) : 1,
                ring.back() ? static_cast<int>(ring.back()->getTag()) : 2);
            int score = 0;
            for (const auto& d : paper.diagonals) {
                if (d[0] < 0 || d[1] < 0 || d[0] >= valence || d[1] >= valence) continue;
                if (candidateHasInternalEdge(cand,
                                             ring[static_cast<size_t>(d[0])],
                                             ring[static_cast<size_t>(d[1])])) {
                    score += 100;
                } else {
                    score -= 1000;
                }
            }
            score += patchTagScore(cand, valence);
            best = std::max(best, score);
        }
    }
    return best;
}

int patchInputOrientationScore(const PatchCandidate& cand,
                               MCGAL::Facet* start,
                               MCGAL::Halfedge* requiredInput) {
    if (!start || !requiredInput || cand.boundary.empty()) return 0;
    MCGAL::Vertex* a = requiredInput->vertex();
    MCGAL::Vertex* b = requiredInput->end_vertex();
    if (!a || !b) return 0;
    MCGAL::Vertex* third = nullptr;
    for (MCGAL::Vertex* v : orderedFaceVertices(start)) {
        if (v && v != a && v != b) {
            third = v;
            break;
        }
    }
    if (!third) return 0;
    const int n = static_cast<int>(cand.boundary.size());
    for (int i = 0; i < n; ++i) {
        MCGAL::Vertex* x = cand.boundary[static_cast<size_t>(i)];
        MCGAL::Vertex* y = cand.boundary[static_cast<size_t>((i + 1) % n)];
        if (x == a && y == b) {
            MCGAL::Vertex* after = cand.boundary[static_cast<size_t>((i + 2) % n)];
            MCGAL::Vertex* before = cand.boundary[static_cast<size_t>((i + n - 1) % n)];
            if (after == third) return 10000;
            if (before == third) return -10000;
        } else if (x == b && y == a) {
            MCGAL::Vertex* after = cand.boundary[static_cast<size_t>((i + n - 1) % n)];
            MCGAL::Vertex* before = cand.boundary[static_cast<size_t>((i + 2) % n)];
            if (after == third) return 10000;
            if (before == third) return -10000;
        }
    }
    return 0;
}

int patchStoredSideScore(const PatchCandidate& cand,
                         int valence,
                         MCGAL::Facet* start,
                         MCGAL::Halfedge* requiredInput,
                         int inputOffsetCode) {
    const int side = (inputOffsetCode >> 4) & 0x3;
    if (side == 0 || !start || !requiredInput ||
        static_cast<int>(cand.boundary.size()) != valence) {
        return 0;
    }
    MCGAL::Vertex* a = requiredInput->vertex();
    MCGAL::Vertex* b = requiredInput->end_vertex();
    if (!a || !b) return 0;
    MCGAL::Vertex* third = nullptr;
    for (MCGAL::Vertex* v : orderedFaceVertices(start)) {
        if (v && v != a && v != b) {
            third = v;
            break;
        }
    }
    if (!third) return 0;

    int best = -250;
    for (int rev = 0; rev < 2; ++rev) {
        for (int rot = 0; rot < valence; ++rot) {
            std::vector<MCGAL::Vertex*> ring;
            ring.reserve(static_cast<size_t>(valence));
            for (int i = 0; i < valence; ++i) {
                int idx = rev ? (rot - i) : (rot + i);
                idx %= valence;
                if (idx < 0) idx += valence;
                ring.push_back(cand.boundary[static_cast<size_t>(idx)]);
            }
            const int off = inputOffsetCode & 0x7;
            const bool reversed = (inputOffsetCode & 0x8) != 0;
            if (off < 0 || off >= valence) continue;
            MCGAL::Vertex* x = ring[static_cast<size_t>(off)];
            MCGAL::Vertex* y = ring[static_cast<size_t>((off + 1) % valence)];
            const bool inputOk = reversed
                                     ? (x == b && y == a)
                                     : (x == a && y == b);
            if (!inputOk) continue;
            MCGAL::Vertex* after = ring[static_cast<size_t>((off + 2) % valence)];
            MCGAL::Vertex* before =
                ring[static_cast<size_t>((off + valence - 1) % valence)];
            bool match = false;
            if (side == 1) {
                match = (third == after);
            } else if (side == 2) {
                match = (third == before);
            } else if (side == 3) {
                match = std::find(ring.begin(), ring.end(), third) == ring.end();
            }
            best = std::max(best, match ? 250 : -250);
        }
    }
    return best;
}

bool findFanPatchContaining(MCGAL::Facet* start,
                            int valence,
                            const std::set<MCGAL::Facet*>* forbiddenFaces,
                            MCGAL::Halfedge* requiredBoundaryGate,
                            int inputOffsetCode,
                            PatchCandidate& best,
                            const std::set<MCGAL::Vertex*>* forbiddenBoundaryVertices = nullptr,
                            std::shared_ptr<MCGAL::Mesh> meshForPointScore = nullptr,
                            MCGAL::PointInt* targetQ = nullptr) {
    if (!start || start->isRemoved() || valence < 3 || valence > 6) return false;
    if (forbiddenFaces && forbiddenFaces->count(start)) return false;
    if (valence == 3) {
        return validatePatchCandidate({start}, valence, nullptr, forbiddenFaces,
                                      requiredBoundaryGate, best);
    }

    const int targetFaces = valence - 2;
    auto pointTieBreakScore = [&](const PatchCandidate& cand) {
        const char* geomScoreEnv = std::getenv("PMSF_PCLTTM_GEOM_PATCH_SCORE");
        if (!geomScoreEnv || geomScoreEnv[0] != '1' ||
            !meshForPointScore || !targetQ || cand.boundary.empty()) return 0;
        int64_t sx = 0, sy = 0, sz = 0;
        int n = 0;
        for (MCGAL::Vertex* v : cand.boundary) {
            if (!v || v->isRemoved()) continue;
            MCGAL::PointInt q = meshForPointScore->floatPosToInt(v->point());
            sx += q[0];
            sy += q[1];
            sz += q[2];
            ++n;
        }
        if (n == 0) return 0;
        const int64_t dx = sx / n - (*targetQ)[0];
        const int64_t dy = sy / n - (*targetQ)[1];
        const int64_t dz = sz / n - (*targetQ)[2];
        const int64_t d = std::llabs(dx) + std::llabs(dy) + std::llabs(dz);
        return static_cast<int>(std::max<int64_t>(-1000000, -d));
    };
    const bool firstValidPatch = []() {
        const char* e = std::getenv("PMSF_PCLTTM_FIRST_VALID_PATCH");
        return e && e[0] == '1';
    }();
        const bool useTagScore = []() {
        const char* e = std::getenv("PMSF_PCLTTM_TAG_SCORE_PATCH");
        return e && e[0] == '1';
    }();
    int dfsNodes = 0;
    int fullCandidates = 0;
        auto tryPool = [&](const std::vector<MCGAL::Facet*>& pool,
                       bool allowEarlyPerfect) -> bool {
        if (static_cast<int>(pool.size()) < targetFaces) return false;
        std::vector<MCGAL::Facet*> chosen;
        chosen.reserve(static_cast<size_t>(targetFaces));
        chosen.push_back(start);
        bool foundLocal = false;
        int bestLocalScore = -1;
        PatchCandidate bestLocal;
        std::set<uint64_t> visitedMasks;
        auto adjacentToChosen = [&](MCGAL::Facet* f) {
            if (!f || f->isRemoved()) return false;
            for (MCGAL::Facet* c : chosen) {
                if (!c || c->isRemoved()) continue;
                for (MCGAL::Halfedge* h : boundedFaceHalfedges(c)) {
                    MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
                    if (opp && opp->face() == f) return true;
                }
            }
            return false;
        };
        auto maskForChosen = [&]() {
            uint64_t mask = 0;
            for (MCGAL::Facet* f : chosen) {
                for (int i = 0; i < static_cast<int>(pool.size()) && i < 63; ++i) {
                    if (pool[static_cast<size_t>(i)] == f) {
                        mask |= (uint64_t{1} << i);
                        break;
                    }
                }
            }
            return mask;
        };
        std::function<bool()> dfs = [&]() -> bool {
            ++dfsNodes;
            if (static_cast<int>(chosen.size()) == targetFaces) {
                ++fullCandidates;
                const uint64_t mask = maskForChosen();
                if (!visitedMasks.insert(mask).second) return false;
                PatchCandidate cand;
                if (validatePatchCandidate(chosen, valence, nullptr, forbiddenFaces,
                                           requiredBoundaryGate, cand)) {
                    if (forbiddenBoundaryVertices) {
                        bool touchesForbidden = false;
                        for (MCGAL::Vertex* v : cand.boundary) {
	                            if (v && forbiddenBoundaryVertices->count(v)) {
	                                touchesForbidden = true;
	                                break;
	                            }
	                        }
	                        if (touchesForbidden) return false;
	                    }
	                    const bool conqueredNonInput =
	                        candidateHasConqueredNonInput(cand, requiredBoundaryGate);
	                    const int score = patchFig9DiagonalScore(
	                                          cand, valence, requiredBoundaryGate,
	                                          -1) +
	                                      patchStoredSideScore(cand, valence, start,
	                                                           requiredBoundaryGate,
	                                                           inputOffsetCode) +
	                                      patchInputOrientationScore(cand, start,
	                                                                 requiredBoundaryGate) +
	                                      (useTagScore ? patchTagScore(cand, valence) * 10 : 0) +
	                                      pointTieBreakScore(cand) +
	                                      (conqueredNonInput ? -20000 : 0);
                    if (firstValidPatch) {
                        bestLocal = cand;
                        bestLocalScore = score;
                        foundLocal = true;
                        return true;
                    }
                    if (!foundLocal || score > bestLocalScore) {
                        bestLocal = cand;
                        bestLocalScore = score;
                        foundLocal = true;
                    }
                    const int perfect = (valence - 3) * 100;
	                    if (allowEarlyPerfect && !conqueredNonInput &&
	                        score >= perfect) return true;
                }
                return false;
            }
            for (int i = 0; i < static_cast<int>(pool.size()); ++i) {
                MCGAL::Facet* candidate = pool[static_cast<size_t>(i)];
                if (candidate == start) continue;
                if (std::find(chosen.begin(), chosen.end(), candidate) != chosen.end()) continue;
                if (!adjacentToChosen(candidate)) continue;
                chosen.push_back(pool[static_cast<size_t>(i)]);
                if (dfs() && allowEarlyPerfect) return true;
                chosen.pop_back();
            }
            return false;
        };
        dfs();
        if (foundLocal) {
            best = bestLocal;
            return true;
        }
        return false;
    };

    for (MCGAL::Halfedge* sh : boundedFaceHalfedges(start)) {
        MCGAL::Vertex* anchor = sh ? sh->vertex() : nullptr;
        if (!anchor || anchor->isRemoved()) continue;
        std::vector<MCGAL::Facet*> fanPool;
        std::deque<MCGAL::Facet*> bfs;
        std::set<MCGAL::Facet*> seen;
        fanPool.push_back(start);
        bfs.push_back(start);
        seen.insert(start);
        while (!bfs.empty() && static_cast<int>(fanPool.size()) < 12) {
            MCGAL::Facet* cur = bfs.front();
            bfs.pop_front();
            for (MCGAL::Halfedge* h : boundedFaceHalfedges(cur)) {
                if (!h || h->isRemoved()) continue;
                if (h->vertex() != anchor && h->end_vertex() != anchor) continue;
                MCGAL::Halfedge* opp = h->opposite();
                MCGAL::Facet* nf = opp ? opp->face() : nullptr;
                if (!nf || nf->isRemoved()) continue;
                if (forbiddenFaces && forbiddenFaces->count(nf)) continue;
                if (!seen.insert(nf).second) continue;
                fanPool.push_back(nf);
                bfs.push_back(nf);
                if (static_cast<int>(fanPool.size()) >= 12) break;
            }
        }
        if (tryPool(fanPool, true)) return true;
    }

    std::vector<MCGAL::Facet*> pool;
    std::deque<MCGAL::Facet*> bfs;
    std::set<MCGAL::Facet*> seen;
    pool.push_back(start);
    bfs.push_back(start);
    seen.insert(start);
    while (!bfs.empty() && static_cast<int>(pool.size()) < 18) {
        MCGAL::Facet* cur = bfs.front();
        bfs.pop_front();
        for (MCGAL::Halfedge* h : boundedFaceHalfedges(cur)) {
            MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
            MCGAL::Facet* nf = opp ? opp->face() : nullptr;
            if (!nf || nf->isRemoved()) continue;
            if (forbiddenFaces && forbiddenFaces->count(nf)) continue;
            if (!seen.insert(nf).second) continue;
            pool.push_back(nf);
            bfs.push_back(nf);
            if (static_cast<int>(pool.size()) >= 18) break;
        }
    }
    if (static_cast<int>(pool.size()) < targetFaces) return false;

    std::vector<MCGAL::Facet*> chosen;
    chosen.reserve(static_cast<size_t>(targetFaces));
    chosen.push_back(start);
    bool found = false;
    int bestScore = -1;
    std::set<uint64_t> visitedMasks;
    auto adjacentToChosen = [&](MCGAL::Facet* f) {
        if (!f || f->isRemoved()) return false;
        for (MCGAL::Facet* c : chosen) {
            if (!c || c->isRemoved()) continue;
            for (MCGAL::Halfedge* h : boundedFaceHalfedges(c)) {
                MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
                if (opp && opp->face() == f) return true;
            }
        }
        return false;
    };
    auto maskForChosen = [&]() {
        uint64_t mask = 0;
        for (MCGAL::Facet* f : chosen) {
            for (int i = 0; i < static_cast<int>(pool.size()) && i < 63; ++i) {
                if (pool[static_cast<size_t>(i)] == f) {
                    mask |= (uint64_t{1} << i);
                    break;
                }
            }
        }
        return mask;
    };
    std::function<bool()> dfs = [&]() -> bool {
        ++dfsNodes;
        if (static_cast<int>(chosen.size()) == targetFaces) {
            ++fullCandidates;
            const uint64_t mask = maskForChosen();
            if (!visitedMasks.insert(mask).second) return false;
            PatchCandidate cand;
            if (validatePatchCandidate(chosen, valence, nullptr, forbiddenFaces,
                                       requiredBoundaryGate, cand)) {
                if (forbiddenBoundaryVertices) {
                    bool touchesForbidden = false;
                    for (MCGAL::Vertex* v : cand.boundary) {
                        if (v && forbiddenBoundaryVertices->count(v)) {
                            touchesForbidden = true;
                            break;
                        }
                    }
	                    if (touchesForbidden) return false;
	                }
	                const bool conqueredNonInput =
	                    candidateHasConqueredNonInput(cand, requiredBoundaryGate);
	                const int score = useTagScore ? patchTagScore(cand, valence)
	                                              : patchFig9DiagonalScore(
	                                                    cand, valence, requiredBoundaryGate,
	                                                    -1);
	                const int finalScore =
	                    score +
	                    patchStoredSideScore(cand, valence, start,
	                                         requiredBoundaryGate, inputOffsetCode) +
	                    patchInputOrientationScore(cand, start, requiredBoundaryGate) +
	                    pointTieBreakScore(cand) +
	                    (conqueredNonInput ? -20000 : 0);
                if (firstValidPatch) {
                    best = cand;
                    bestScore = finalScore;
                    found = true;
                    return true;
                }
                if (!found || finalScore > bestScore) {
                    best = cand;
                    bestScore = finalScore;
                    found = true;
                }
                const int perfect = (valence - 3) * 100;
	                if (!useTagScore && !conqueredNonInput &&
	                    score >= perfect) return true;
            }
            return false;
        }
        for (int i = 1; i < static_cast<int>(pool.size()); ++i) {
            MCGAL::Facet* candidate = pool[static_cast<size_t>(i)];
            if (std::find(chosen.begin(), chosen.end(), candidate) != chosen.end()) continue;
            if (!adjacentToChosen(candidate)) continue;
            chosen.push_back(candidate);
            if (dfs() && !useTagScore) return true;
            chosen.pop_back();
        }
        return false;
    };
    dfs();
    return found;
}

bool discoverFig9FanWalk(MCGAL::Halfedge* gate,
                         int valence,
                         const std::set<MCGAL::Facet*>* forbiddenFaces,
                         MCGAL::Halfedge* requiredBoundaryGate,
                         PatchCandidate& patch) {
    if (!isWalkGate(gate) || valence < 3 || valence > 6) return false;
    MCGAL::Facet* start = gate->face();
    if (!start || start->isRemoved()) return false;
    if (forbiddenFaces && forbiddenFaces->count(start)) return false;
    MCGAL::Halfedge* input = inputBoundaryFromGate(gate);
    if (!input || input->isRemoved()) return false;
    MCGAL::Vertex* a = input->vertex();
    MCGAL::Vertex* b = input->end_vertex();
    if (!a || !b) return false;

    std::vector<MCGAL::Vertex*> tri = orderedFaceVertices(start);
    MCGAL::Vertex* c = nullptr;
    for (MCGAL::Vertex* v : tri) {
        if (v && v != a && v != b) {
            c = v;
            break;
        }
    }
    if (!c) return false;

    auto collectFan = [&](MCGAL::Vertex* anchor,
                          MCGAL::Vertex* firstPrev,
                          std::vector<MCGAL::Facet*>& outFaces) -> bool {
        outFaces.clear();
        outFaces.reserve(static_cast<size_t>(valence - 2));
        outFaces.push_back(start);
        MCGAL::Vertex* prev = firstPrev;
        MCGAL::Facet* cur = start;
        for (int idx = valence - 2; idx >= 2; --idx) {
            MCGAL::Halfedge* diag = nullptr;
            for (MCGAL::Halfedge* h : boundedFaceHalfedges(cur)) {
                if (sameUndirectedEdge(h, anchor, prev)) {
                    diag = h;
                    break;
                }
            }
            if (!diag || !diag->opposite()) return false;
            MCGAL::Facet* nf = diag->opposite()->face();
            if (!nf || nf->isRemoved()) return false;
            if (forbiddenFaces && forbiddenFaces->count(nf)) return false;
            if (std::find(outFaces.begin(), outFaces.end(), nf) != outFaces.end()) return false;
            MCGAL::Vertex* next = nullptr;
            for (MCGAL::Vertex* v : orderedFaceVertices(nf)) {
                if (v && v != anchor && v != prev) {
                    next = v;
                    break;
                }
            }
            if (!next) return false;
            outFaces.push_back(nf);
            cur = nf;
            prev = next;
        }
        return true;
    };

    std::vector<MCGAL::Facet*> faces;
    const bool anchorAtInputA = a && a->getTag() == MCGAL::Vertex::Plus;
    if (!collectFan(anchorAtInputA ? a : b, c, faces)) {
        if (!collectFan(anchorAtInputA ? b : a, c, faces)) return false;
    }
    if (!validatePatchCandidate(faces, valence, nullptr, forbiddenFaces,
                                requiredBoundaryGate, patch)) {
        return false;
    }
    const bool preferInputTripleOrder = []() {
        const char* e = std::getenv("PMSF_PCLTTM_PREFER_INPUT_TRIPLE_ORDER");
        return e && e[0] == '1';
    }();
    if (preferInputTripleOrder && valence > 4 &&
        !boundaryHasTripleOrder(patch.boundary, a, b, c)) {
        std::vector<MCGAL::Facet*> altFaces;
        PatchCandidate alt;
        if (collectFan(a, c, altFaces) &&
            validatePatchCandidate(altFaces, valence, nullptr, forbiddenFaces,
                                   requiredBoundaryGate, alt) &&
            boundaryHasTripleOrder(alt.boundary, a, b, c)) {
            patch = alt;
        }
    }
    if (valence == 4) {
        if (candidateHasConqueredNonInput(patch, requiredBoundaryGate)) {
            MCGAL::Vertex* altEdges[2][2] = {{a, c}, {b, c}};
            for (auto& ev : altEdges) {
                MCGAL::Halfedge* altDiag = nullptr;
                for (MCGAL::Halfedge* h : boundedFaceHalfedges(start)) {
                    if (sameUndirectedEdge(h, ev[0], ev[1])) {
                        altDiag = h;
                        break;
                    }
                }
                if (!altDiag || !altDiag->opposite()) continue;
                MCGAL::Facet* nf = altDiag->opposite()->face();
                if (!nf || nf->isRemoved()) continue;
                if (forbiddenFaces && forbiddenFaces->count(nf)) continue;
                PatchCandidate alt;
                if (!validatePatchCandidate({start, nf}, valence, nullptr, forbiddenFaces,
                                            requiredBoundaryGate, alt)) {
                    continue;
                }
                if (!candidateHasConqueredNonInput(alt, requiredBoundaryGate)) {
                    if (const char* markRejected =
                            std::getenv("PMSF_PCLTTM_MARK_REJECTED_FAN_FACES");
                        markRejected && markRejected[0] == '1') {
                        std::set<MCGAL::Facet*> altFaces(alt.faces.begin(), alt.faces.end());
                        for (MCGAL::Facet* rejected : patch.faces) {
                            if (rejected && !rejected->isRemoved() && !altFaces.count(rejected)) {
                                rejected->setProcessedFlag();
                            }
                        }
                    }
                    alt.usedAltFanWalk = true;
                    patch = alt;
                    return true;
                }
            }
            return false;
        }
    }
    const bool rejectConqueredLargeFan = []() {
        const char* e = std::getenv("PMSF_PCLTTM_REJECT_CONQUERED_LARGE_FAN");
        return e && e[0] == '1';
    }();
    if (rejectConqueredLargeFan && valence > 4 &&
        candidateHasConqueredNonInput(patch, requiredBoundaryGate)) {
        return false;
    }
    return true;
}

MCGAL::Vertex* findLiveVertexByQ(std::shared_ptr<MCGAL::Mesh> mesh,
                                 const std::array<int, 3>& q) {
    if (!mesh) return nullptr;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (sameQuant(mesh, v, q[0], q[1], q[2])) return v;
    }
    return nullptr;
}

MCGAL::Vertex* findLiveVertexByQ(std::shared_ptr<MCGAL::Mesh> mesh,
                                 const QuantVertexIndex* index,
                                 const std::array<int, 3>& q) {
    if (index) {
        if (MCGAL::Vertex* v = findLiveVertexByQIndexed(*index, q)) return v;
    }
    return findLiveVertexByQ(mesh, q);
}

bool candidateBoundaryMatchesRingQ(std::shared_ptr<MCGAL::Mesh> mesh,
                                   const PatchCandidate& cand,
                                   const std::vector<std::array<int, 3>>& ringQ) {
    const int n = static_cast<int>(ringQ.size());
    if (!mesh || static_cast<int>(cand.boundary.size()) != n) return false;
    const char* exactRingEnv = std::getenv("PMSF_PCLTTM_STRICT_EXACT_RING_Q");
    const bool exactRingQ = exactRingEnv && exactRingEnv[0] == '1';
    auto sameQuantNear = [&](MCGAL::Vertex* v, const std::array<int, 3>& q) {
        if (!v || v->isRemoved()) return false;
        MCGAL::PointInt p = mesh->floatPosToInt(v->point());
        if (exactRingQ) {
            return p[0] == q[0] && p[1] == q[1] && p[2] == q[2];
        }
        return std::abs(p[0] - q[0]) <= 1 &&
               std::abs(p[1] - q[1]) <= 1 &&
               std::abs(p[2] - q[2]) <= 1;
    };
    for (int rev = 0; rev < 2; ++rev) {
        for (int rot = 0; rot < n; ++rot) {
            bool ok = true;
            for (int i = 0; i < n; ++i) {
                int idx = rev ? (rot - i) : (rot + i);
                idx %= n;
                if (idx < 0) idx += n;
                MCGAL::Vertex* v = cand.boundary[static_cast<size_t>(idx)];
                const auto& q = ringQ[static_cast<size_t>(i)];
                if (!sameQuantNear(v, q)) {
                    ok = false;
                    break;
                }
            }
            if (ok) return true;
        }
    }
    return false;
}

bool sameQuantNear(std::shared_ptr<MCGAL::Mesh> mesh,
                   MCGAL::Vertex* v,
                   const std::array<int, 3>& q) {
    if (!mesh || !v || v->isRemoved()) return false;
    MCGAL::PointInt p = mesh->floatPosToInt(v->point());
    return std::abs(p[0] - q[0]) <= 1 &&
           std::abs(p[1] - q[1]) <= 1 &&
           std::abs(p[2] - q[2]) <= 1;
}

MCGAL::Halfedge* findRingOrderedHalfedge(std::shared_ptr<MCGAL::Mesh> mesh,
                                         MCGAL::Facet* f,
                                         const std::vector<std::array<int, 3>>& ringQ) {
    if (!mesh || !f || f->isRemoved() || ringQ.size() < 3) return nullptr;
    const size_t n = ringQ.size();
    for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
        if (!h || h->isRemoved() || !h->vertex() || !h->end_vertex()) continue;
        for (size_t i = 0; i < n; ++i) {
            const auto& a = ringQ[i];
            const auto& b = ringQ[(i + 1) % n];
            if (sameQuantNear(mesh, h->vertex(), a) &&
                sameQuantNear(mesh, h->end_vertex(), b)) {
                return h;
            }
        }
    }
    return nullptr;
}

MCGAL::Facet* discoverAndMergePatchFromRingQ(
    std::shared_ptr<MCGAL::Mesh> mesh,
    const QuantVertexIndex* vertexIndex,
    const std::vector<std::array<int, 3>>& ringQ,
    int valence,
    const std::set<MCGAL::Facet*>* forbiddenFaces,
    MCGAL::Halfedge* requiredInputBoundary,
    std::vector<MCGAL::Vertex*>& boundary,
    bool& merged,
    int traceIndex = -1,
    MCGAL::Facet* preferredFace = nullptr) {
    merged = false;
    boundary.clear();
    if (!mesh || static_cast<int>(ringQ.size()) != valence || valence < 3 || valence > 6) {
        if (shouldTraceOp(traceIndex)) {
            std::fprintf(stderr, "[PCLTTM trace ring fail] si=%d reason=bad_size\n", traceIndex);
        }
        return nullptr;
    }
    std::set<MCGAL::Vertex*> ringVerts;
    std::vector<MCGAL::Vertex*> ringOrdered;
    ringOrdered.reserve(ringQ.size());
    auto ringCandidateVertices = [&](const std::array<int, 3>& q)
        -> std::vector<MCGAL::Vertex*> {
        std::vector<MCGAL::Vertex*> out;
        std::set<MCGAL::Vertex*> seen;
        const char* exactRingEnv = std::getenv("PMSF_PCLTTM_STRICT_EXACT_RING_Q");
        const bool exactRingQ = exactRingEnv && exactRingEnv[0] == '1';
        auto usable = [&](MCGAL::Vertex* v) {
            return v && !v->isRemoved();
        };
        auto append = [&](MCGAL::Vertex* v) {
            if (!usable(v) || !seen.insert(v).second) return;
            out.push_back(v);
        };
        auto appendFromIndex = [&](const QuantKey& key) {
            if (!vertexIndex) return;
            auto it = vertexIndex->find(key);
            if (it == vertexIndex->end()) return;
            for (MCGAL::Vertex* v : it->second) {
                append(v);
            }
        };
        appendFromIndex(quantKeyFromQ(q));
        if (exactRingQ) return out;
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    appendFromIndex(QuantKey{q[0] + dx, q[1] + dy, q[2] + dz});
                }
            }
        }
        for (MCGAL::Vertex* v : mesh->vertices()) {
            if (!usable(v)) continue;
            MCGAL::PointInt p = mesh->floatPosToInt(v->point());
            if (std::abs(p[0] - q[0]) <= 1 &&
                std::abs(p[1] - q[1]) <= 1 &&
                std::abs(p[2] - q[2]) <= 1) {
                append(v);
            }
        }
        std::stable_sort(out.begin(), out.end(), [&](MCGAL::Vertex* a, MCGAL::Vertex* b) {
            auto dist = [&](MCGAL::Vertex* v) {
                MCGAL::PointInt p = mesh->floatPosToInt(v->point());
                return std::abs(p[0] - q[0]) + std::abs(p[1] - q[1]) +
                       std::abs(p[2] - q[2]);
            };
            return dist(a) < dist(b);
        });
        if (out.size() > 6) out.resize(6);
        return out;
    };
    std::vector<std::vector<MCGAL::Vertex*>> candidateLists;
    candidateLists.reserve(ringQ.size());
    for (const auto& q : ringQ) {
        candidateLists.push_back(ringCandidateVertices(q));
        if (candidateLists.back().empty()) {
            if (shouldTraceOp(traceIndex)) {
                std::fprintf(stderr,
                             "[PCLTTM trace ring fail] si=%d reason=missing_vertex q=%d,%d,%d\n",
                             traceIndex, q[0], q[1], q[2]);
            }
            return nullptr;
        }
    }
    std::vector<MCGAL::Vertex*> currentRing(ringQ.size(), nullptr);
    std::vector<MCGAL::Vertex*> bestRing;
    std::set<MCGAL::Vertex*> usedCandidates;
    int bestScore = std::numeric_limits<int>::min();
    const int targetFaces = valence - 2;
    auto hasEdgeBetween = [](MCGAL::Vertex* a, MCGAL::Vertex* b) {
        if (!a || !b) return false;
        MCGAL::Halfedge* ab = boundedFindHalfedge(a, b);
        MCGAL::Halfedge* ba = boundedFindHalfedge(b, a);
        return (ab && !ab->isRemoved()) || (ba && !ba->isRemoved());
    };
    auto quantDistance = [&](MCGAL::Vertex* v, const std::array<int, 3>& q) {
        if (!v) return 1000000;
        MCGAL::PointInt p = mesh->floatPosToInt(v->point());
        return std::abs(p[0] - q[0]) + std::abs(p[1] - q[1]) +
               std::abs(p[2] - q[2]);
    };
    auto scoreRing = [&](const std::vector<MCGAL::Vertex*>& ring) {
        int edgeScore = 0;
        int distPenalty = 0;
        int preferredScore = 0;
        std::set<MCGAL::Vertex*> ringSet(ring.begin(), ring.end());
        if (preferredFace && !preferredFace->isRemoved()) {
            for (MCGAL::Halfedge* h : boundedFaceHalfedges(preferredFace)) {
                if (h && h->vertex() && ringSet.count(h->vertex())) {
                    ++preferredScore;
                }
            }
        }
        const int n = static_cast<int>(ring.size());
        for (int i = 0; i < n; ++i) {
            if (hasEdgeBetween(ring[static_cast<size_t>(i)],
                               ring[static_cast<size_t>((i + 1) % n)])) {
                ++edgeScore;
            }
            distPenalty += quantDistance(ring[static_cast<size_t>(i)],
                                         ringQ[static_cast<size_t>(i)]);
        }
        return preferredScore * 10000 + edgeScore * 1000 - distPenalty;
    };
    auto ringPatchPoolSize = [&](const std::vector<MCGAL::Vertex*>& ring) {
        if (ring.size() != ringQ.size()) return 0;
        std::set<MCGAL::Vertex*> verts(ring.begin(), ring.end());
        if (verts.size() != ring.size()) return 0;
        auto localIsRingEdge = [&](MCGAL::Vertex* a, MCGAL::Vertex* b) {
            if (!a || !b) return false;
            const int n = static_cast<int>(ring.size());
            for (int i = 0; i < n; ++i) {
                MCGAL::Vertex* x = ring[static_cast<size_t>(i)];
                MCGAL::Vertex* y = ring[static_cast<size_t>((i + 1) % n)];
                if ((a == x && b == y) || (a == y && b == x)) return true;
            }
            return false;
        };
        int poolCount = 0;
        int allInsideCount = 0;
        std::unordered_set<MCGAL::Facet*> seen;
        std::vector<MCGAL::Facet*> candidates;
        for (MCGAL::Vertex* v : ring) {
            for (MCGAL::Halfedge* hv : boundedVertexHalfedges(v)) {
                MCGAL::Facet* f = hv ? hv->face() : nullptr;
                if (f && !f->isRemoved()) candidates.push_back(f);
            }
        }
        if (candidates.empty()) {
            for (MCGAL::Facet* f : mesh->faces()) candidates.push_back(f);
        }
        for (MCGAL::Facet* f : candidates) {
            if (!f || f->isRemoved()) continue;
            if (!seen.insert(f).second) continue;
            if (forbiddenFaces && forbiddenFaces->count(f)) continue;
            bool allInside = true;
            bool hasRingEdgeLocal = false;
            for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
                if (!h || !h->vertex() || !verts.count(h->vertex())) {
                    allInside = false;
                    break;
                }
                if (localIsRingEdge(h->vertex(), h->end_vertex())) {
                    hasRingEdgeLocal = true;
                }
            }
            if (allInside) {
                ++allInsideCount;
                if (hasRingEdgeLocal) ++poolCount;
            }
        }
        if (poolCount < targetFaces && allInsideCount <= 24) return allInsideCount;
        return poolCount;
    };
    std::vector<MCGAL::Vertex*> greedyRing;
    greedyRing.reserve(ringQ.size());
    std::set<MCGAL::Vertex*> greedyUsed;
    bool greedyOk = true;
    for (const auto& list : candidateLists) {
        MCGAL::Vertex* picked = nullptr;
        for (MCGAL::Vertex* v : list) {
            if (v && !greedyUsed.count(v)) {
                picked = v;
                break;
            }
        }
        if (!picked) {
            greedyOk = false;
            break;
        }
        greedyUsed.insert(picked);
        greedyRing.push_back(picked);
    }
    if (greedyOk && ringPatchPoolSize(greedyRing) >= targetFaces) {
        bestRing = greedyRing;
    }
    std::function<void(size_t)> chooseRing = [&](size_t idx) {
        if (idx == candidateLists.size()) {
            const int score = scoreRing(currentRing);
            if (score > bestScore) {
                bestScore = score;
                bestRing = currentRing;
            }
            return;
        }
        for (MCGAL::Vertex* v : candidateLists[idx]) {
            if (!v || usedCandidates.count(v)) continue;
            usedCandidates.insert(v);
            currentRing[idx] = v;
            chooseRing(idx + 1);
            currentRing[idx] = nullptr;
            usedCandidates.erase(v);
        }
    };
    const char* candidateMatchEnv =
        std::getenv("PMSF_PCLTTM_STRICT_CANDIDATE_RING_MATCH");
    const bool useCandidateMatch = candidateMatchEnv && candidateMatchEnv[0] == '1';
    if (bestRing.empty() && useCandidateMatch) {
        chooseRing(0);
    }
    if (bestRing.empty() && greedyOk) {
        bestRing = greedyRing;
    }
    if (bestRing.empty()) {
        if (shouldTraceOp(traceIndex)) {
            std::fprintf(stderr, "[PCLTTM trace ring fail] si=%d reason=no_unique_vertices\n",
                         traceIndex);
        }
        return nullptr;
    }
    ringOrdered = std::move(bestRing);
    for (MCGAL::Vertex* v : ringOrdered) ringVerts.insert(v);
    if (static_cast<int>(ringVerts.size()) != valence) {
        if (shouldTraceOp(traceIndex)) {
            std::fprintf(stderr, "[PCLTTM trace ring fail] si=%d reason=duplicate_vertices\n", traceIndex);
        }
        return nullptr;
    }

    auto isRingEdge = [&](MCGAL::Vertex* a, MCGAL::Vertex* b) {
        if (!a || !b) return false;
        const int n = static_cast<int>(ringOrdered.size());
        for (int i = 0; i < n; ++i) {
            MCGAL::Vertex* x = ringOrdered[static_cast<size_t>(i)];
            MCGAL::Vertex* y = ringOrdered[static_cast<size_t>((i + 1) % n)];
            if ((a == x && b == y) || (a == y && b == x)) return true;
        }
        return false;
    };
    auto faceHasRingEdge = [&](MCGAL::Facet* f) {
        for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
            if (h && isRingEdge(h->vertex(), h->end_vertex())) return true;
        }
        return false;
    };

    std::vector<MCGAL::Facet*> pool;
    std::vector<MCGAL::Facet*> allInsidePool;
    std::unordered_set<MCGAL::Facet*> seenFaces;
    std::vector<MCGAL::Facet*> candidateFaces;
    for (MCGAL::Vertex* v : ringOrdered) {
        for (MCGAL::Halfedge* hv : boundedVertexHalfedges(v)) {
            MCGAL::Facet* f = hv ? hv->face() : nullptr;
            if (f && !f->isRemoved()) candidateFaces.push_back(f);
        }
    }
    if (candidateFaces.empty()) {
        for (MCGAL::Facet* f : mesh->faces()) candidateFaces.push_back(f);
    }
    for (MCGAL::Facet* f : candidateFaces) {
        if (!f || f->isRemoved()) continue;
        if (!seenFaces.insert(f).second) continue;
        if (forbiddenFaces && forbiddenFaces->count(f)) continue;
        bool allInside = true;
        for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
            if (!h || !h->vertex() || !ringVerts.count(h->vertex())) {
                allInside = false;
                break;
            }
        }
        if (allInside) {
            allInsidePool.push_back(f);
            if (faceHasRingEdge(f)) pool.push_back(f);
        }
    }
    if (static_cast<int>(pool.size()) < targetFaces &&
        allInsidePool.size() <= 24) {
        pool = allInsidePool;
    }
    if (static_cast<int>(pool.size()) < targetFaces && !candidateFaces.empty()) {
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved()) continue;
            if (!seenFaces.insert(f).second) continue;
            if (forbiddenFaces && forbiddenFaces->count(f)) continue;
            bool allInside = true;
            for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
                if (!h || !h->vertex() || !ringVerts.count(h->vertex())) {
                    allInside = false;
                    break;
                }
            }
            if (allInside) {
                allInsidePool.push_back(f);
                if (faceHasRingEdge(f)) pool.push_back(f);
            }
        }
        if (static_cast<int>(pool.size()) < targetFaces &&
            allInsidePool.size() <= 24) {
            pool = allInsidePool;
        }
    }
    if (shouldTraceOp(traceIndex)) {
        std::fprintf(stderr,
                     "[PCLTTM trace ring] si=%d valence=%d pool=%zu target=%d\n",
                     traceIndex, valence, pool.size(), targetFaces);
    }
    if (static_cast<int>(pool.size()) < targetFaces) {
        if (shouldTraceOp(traceIndex)) {
            std::fprintf(stderr,
                         "[PCLTTM trace ring fail] si=%d reason=small_pool pool=%zu target=%d\n",
                         traceIndex, pool.size(), targetFaces);
        }
        return nullptr;
    }

    std::vector<MCGAL::Facet*> chosen;
    chosen.reserve(static_cast<size_t>(targetFaces));
    PatchCandidate best;
    bool found = false;
    int discoveredValence = valence;
    std::function<bool(int)> dfs = [&](int nextIdx) -> bool {
        if (static_cast<int>(chosen.size()) == targetFaces) {
            PatchCandidate cand;
            if (validatePatchCandidate(chosen, valence, nullptr, forbiddenFaces,
                                       requiredInputBoundary, cand) &&
                candidateBoundaryMatchesRingQ(mesh, cand, ringQ)) {
                best = cand;
                found = true;
                discoveredValence = valence;
                return true;
            }
            return false;
        }
        for (int i = nextIdx; i < static_cast<int>(pool.size()); ++i) {
            chosen.push_back(pool[static_cast<size_t>(i)]);
            if (dfs(i + 1)) return true;
            chosen.pop_back();
        }
        return false;
    };
    dfs(0);
    const char* allowPartialRingEnv =
        std::getenv("PMSF_PCLTTM_STRICT_ALLOW_PARTIAL_RING");
    if (!found && !pool.empty() && pool.size() <= 10 &&
        allowPartialRingEnv && allowPartialRingEnv[0] == '1') {
        const int maxPartialFaces =
            std::min<int>(targetFaces, static_cast<int>(pool.size()));
        for (int partialFaces = maxPartialFaces; partialFaces >= 1 && !found; --partialFaces) {
            std::vector<MCGAL::Facet*> partial;
            partial.reserve(static_cast<size_t>(partialFaces));
            std::function<bool(int)> partialDfs = [&](int nextIdx) -> bool {
                if (static_cast<int>(partial.size()) == partialFaces) {
                    PatchCandidate cand;
                    const int partialValence = partialFaces + 2;
                    if (validatePatchCandidate(partial, partialValence, nullptr,
                                               forbiddenFaces, nullptr, cand)) {
                        best = cand;
                        found = true;
                        discoveredValence = partialValence;
                        if (shouldTraceOp(traceIndex)) {
                            std::fprintf(stderr,
                                         "[PCLTTM trace ring partial] si=%d faces=%d valence=%d requested=%d\n",
                                         traceIndex, partialFaces, partialValence, valence);
                        }
                        return true;
                    }
                    return false;
                }
                for (int i = nextIdx; i < static_cast<int>(pool.size()); ++i) {
                    partial.push_back(pool[static_cast<size_t>(i)]);
                    if (partialDfs(i + 1)) return true;
                    partial.pop_back();
                }
                return false;
            };
            partialDfs(0);
        }
    }
    if (!found) {
        if (shouldTraceOp(traceIndex)) {
            std::fprintf(stderr,
                         "[PCLTTM trace ring fail] si=%d reason=no_candidate pool=%zu target=%d\n",
                         traceIndex, pool.size(), targetFaces);
        }
        return nullptr;
    }
    MCGAL::Facet* poly = mergePatchToPolygon(best, discoveredValence);
    if (!poly || poly->isRemoved() || faceDegree(poly) != discoveredValence) {
        if (shouldTraceOp(traceIndex)) {
            std::fprintf(stderr, "[PCLTTM trace ring fail] si=%d reason=merge_failed\n", traceIndex);
        }
        return nullptr;
    }
    if (MCGAL::Halfedge* ringGate = findRingOrderedHalfedge(mesh, poly, ringQ)) {
        poly->proxyHalfedge(ringGate);
    }
    merged = true;
    boundary = std::move(ringOrdered);
    return poly;
}

MCGAL::Facet* discoverAndMergePatchFromGate(MCGAL::Halfedge* gate,
                                            int valence,
                                            const std::set<MCGAL::Facet*>* forbiddenFaces,
                                            std::vector<MCGAL::Vertex*>& boundary,
                                            bool& merged,
                                            int inputOffsetCode = -1,
                                            const std::set<MCGAL::Vertex*>* forbiddenBoundaryVertices = nullptr,
                                            std::shared_ptr<MCGAL::Mesh> meshForPointScore = nullptr,
                                            MCGAL::PointInt* targetQ = nullptr,
                                            bool* usedAltFanWalkOut = nullptr) {
    merged = false;
    boundary.clear();
    if (usedAltFanWalkOut) *usedAltFanWalkOut = false;
    if (!isWalkGate(gate) || valence < 3 || valence > 6) return nullptr;
    MCGAL::Facet* start = gate->face();
    if (!start || start->isRemoved()) return nullptr;
    if (forbiddenFaces && forbiddenFaces->count(start)) return nullptr;
    MCGAL::Halfedge* requiredInputBoundary = inputBoundaryFromGate(gate);
    if (valence == 3) {
        PatchCandidate tri;
        if (!validatePatchCandidate({start}, valence, nullptr, forbiddenFaces,
                                    requiredInputBoundary, tri)) {
            return nullptr;
        }
        if (forbiddenBoundaryVertices) {
            for (MCGAL::Vertex* v : tri.boundary) {
                if (v && forbiddenBoundaryVertices->count(v)) return nullptr;
            }
        }
        boundary = orderedFaceVertices(start);
        return start;
    }

    PatchCandidate patch;
    bool foundPatch = false;
    const char* geomScoreEnv = std::getenv("PMSF_PCLTTM_GEOM_PATCH_SCORE");
    const bool preferPointScoredSearch = targetQ && geomScoreEnv && geomScoreEnv[0] == '1';
    const bool useStoredTopologyOffsets = []() {
        const char* e = std::getenv("PMSF_PCLTTM_USE_TOPO_OFFSETS");
        return e && *e && std::strcmp(e, "0") != 0;
    }();
    const char* disableMepp2Join = std::getenv("PMSF_PCLTTM_DISABLE_MEPP2_JOIN");
    if (!disableMepp2Join || !*disableMepp2Join ||
        std::strcmp(disableMepp2Join, "0") == 0) {
        int mepp2Type = 0;
        PatchCandidate mepp2Patch;
        if (discoverMepp2PatchFromInput(requiredInputBoundary, valence,
                                        forbiddenFaces, mepp2Patch, mepp2Type)) {
            MCGAL::Facet* poly =
                mergeMepp2PatchFromInput(requiredInputBoundary, mepp2Type, valence);
            if (poly && !poly->isRemoved() && faceDegree(poly) == valence) {
                merged = valence > 3;
                boundary = orderedFaceVertices(poly);
                return poly;
            }
            return nullptr;
        }
    }
    if (preferPointScoredSearch) {
        foundPatch = findFanPatchContaining(start, valence, forbiddenFaces,
                                            requiredInputBoundary, inputOffsetCode,
                                            patch, forbiddenBoundaryVertices,
                                            meshForPointScore, targetQ) ||
                     discoverFig9FanWalk(gate, valence, forbiddenFaces,
                                         requiredInputBoundary, patch);
    } else if (inputOffsetCode >= 0 && useStoredTopologyOffsets) {
        foundPatch = findFanPatchContaining(start, valence, forbiddenFaces,
                                            requiredInputBoundary, inputOffsetCode,
                                            patch, forbiddenBoundaryVertices,
                                            meshForPointScore, targetQ);
    } else if (const char* e = std::getenv("PMSF_PCLTTM_FIG9_ONLY");
               e && e[0] == '1') {
        foundPatch = discoverFig9FanWalk(gate, valence, forbiddenFaces,
                                         requiredInputBoundary, patch);
    } else {
        foundPatch = discoverFig9FanWalk(gate, valence, forbiddenFaces,
                                         requiredInputBoundary, patch) ||
                     findFanPatchContaining(start, valence, forbiddenFaces,
                                            requiredInputBoundary, inputOffsetCode,
                                            patch, forbiddenBoundaryVertices,
                                            meshForPointScore, targetQ);
    }
    if (!foundPatch) {
        return nullptr;
    }
    if (forbiddenBoundaryVertices) {
        for (MCGAL::Vertex* v : patch.boundary) {
            if (v && forbiddenBoundaryVertices->count(v)) return nullptr;
        }
    }
    MCGAL::Facet* poly = mergePatchToPolygon(patch, valence);
    if (!poly || poly->isRemoved() || faceDegree(poly) != valence) {
        return nullptr;
    }
    merged = true;
    if (usedAltFanWalkOut) *usedAltFanWalkOut = patch.usedAltFanWalk;
    boundary = orderedFaceVertices(poly);
    return poly;
}

bool gateCanDecodeAsVWithoutMerging(MCGAL::Halfedge* gate,
                                    int valence,
                                    const std::set<MCGAL::Facet*>* forbiddenFaces,
                                    int inputOffsetCode = -1,
                                    std::shared_ptr<MCGAL::Mesh> meshForPointScore = nullptr,
                                    MCGAL::PointInt* targetQ = nullptr) {
    if (!isWalkGate(gate) || valence < 3 || valence > 6) return false;
    MCGAL::Facet* start = gate->face();
    if (!start || start->isRemoved()) return false;
    if (forbiddenFaces && forbiddenFaces->count(start)) return false;
    MCGAL::Halfedge* requiredInputBoundary = inputBoundaryFromGate(gate);
    if (!requiredInputBoundary || requiredInputBoundary->isRemoved()) return false;

    PatchCandidate patch;
    if (valence == 3) {
        return validatePatchCandidate({start}, valence, nullptr, forbiddenFaces,
                                      requiredInputBoundary, patch);
    }

    int mepp2Type = 0;
    if (discoverMepp2PatchFromInput(requiredInputBoundary, valence,
                                    forbiddenFaces, patch, mepp2Type)) {
        return true;
    }
    if (discoverFig9FanWalk(gate, valence, forbiddenFaces,
                            requiredInputBoundary, patch)) {
        return true;
    }
    return findFanPatchContaining(start, valence, forbiddenFaces,
                                  requiredInputBoundary, inputOffsetCode, patch,
                                  nullptr, meshForPointScore, targetQ);
}

int faceDegree(MCGAL::Facet* f) {
    if (!f || f->isRemoved() || !f->proxyHalfedge()) return 0;
    int n = 0;
    MCGAL::Halfedge* start = f->proxyHalfedge();
    MCGAL::Halfedge* h = start;
    do {
        ++n;
        h = h ? h->next() : nullptr;
    } while (h && h != start && n < 256);
    return n;
}

bool hasBoundedFaceLoop(MCGAL::Facet* f, int maxSteps = 64) {
    if (!f || f->isRemoved() || !f->proxyHalfedge()) return false;
    MCGAL::Halfedge* start = f->proxyHalfedge();
    MCGAL::Halfedge* h = start;
    int n = 0;
    do {
        if (!h || h->isRemoved()) return false;
        h = h->next();
        ++n;
    } while (h && h != start && n < maxSteps);
    return h == start;
}

MCGAL::Facet* mergePatchToPolygon(const PatchCandidate& cand,
                                  int valence) {
    if (cand.faces.empty()) return nullptr;
    MCGAL::Facet* poly = cand.faces.front();
    std::vector<MCGAL::Halfedge*> remaining = cand.internal;
    int joins = 0;
    while (joins < valence - 3) {
        MCGAL::Halfedge* best = nullptr;
        int bestScore = 1000000;
        for (MCGAL::Halfedge* diag : remaining) {
            if (!diag || diag->isRemoved() || !diag->opposite() || diag->opposite()->isRemoved()) {
                continue;
            }
            MCGAL::Facet* fa = diag->face();
            MCGAL::Facet* fb = diag->opposite()->face();
            if (!fa || !fb || fa->isRemoved() || fb->isRemoved()) continue;
            const int da = faceDegree(fa);
            const int db = faceDegree(fb);
            if (da <= 0 || db <= 0) continue;
            const int score = std::min(da, db) * 100 + std::max(da, db);
            if (score < bestScore) {
                bestScore = score;
                best = diag;
            }
        }
        if (!best) break;
        MCGAL::Halfedge* ret = MCGAL::join_face(best);
        if (ret && ret->face() && !ret->face()->isRemoved()) {
            poly = ret->face();
        }
        ++joins;
    }
    if (poly && !poly->isRemoved() && faceDegree(poly) == valence) return poly;
    for (MCGAL::Facet* f : cand.faces) {
        if (f && !f->isRemoved() && faceDegree(f) == valence) return f;
    }
    return poly && !poly->isRemoved() ? poly : nullptr;
}

void refillLivingFaces(std::shared_ptr<MCGAL::Mesh> mesh, std::deque<MCGAL::Facet*>& q) {
    q.clear();
    if (!mesh) return;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (f && !f->isRemoved()) q.push_back(f);
    }
}

MCGAL::Facet* livingFaceByRank(std::shared_ptr<MCGAL::Mesh> mesh, int rank) {
    if (!mesh || rank < 0) return nullptr;
    int cur = 0;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        if (cur == rank) return f;
        ++cur;
    }
    return nullptr;
}

double pointPlaneDistance2Quant(std::shared_ptr<MCGAL::Mesh> mesh,
                                MCGAL::Facet* f,
                                MCGAL::PointInt q) {
    auto hs = boundedFaceHalfedges(f);
    if (!mesh || hs.size() != 3) return std::numeric_limits<double>::infinity();
    std::array<MCGAL::PointInt, 3> p{};
    for (int i = 0; i < 3; ++i) {
        p[static_cast<size_t>(i)] = mesh->floatPosToInt(hs[static_cast<size_t>(i)]->vertex()->point());
    }
    const double ax = p[1][0] - p[0][0];
    const double ay = p[1][1] - p[0][1];
    const double az = p[1][2] - p[0][2];
    const double bx = p[2][0] - p[0][0];
    const double by = p[2][1] - p[0][1];
    const double bz = p[2][2] - p[0][2];
    const double nx = ay * bz - az * by;
    const double ny = az * bx - ax * bz;
    const double nz = ax * by - ay * bx;
    const double n2 = nx * nx + ny * ny + nz * nz;
    if (n2 <= 0.0) return std::numeric_limits<double>::infinity();
    const double px = q[0] - p[0][0];
    const double py = q[1] - p[0][1];
    const double pz = q[2] - p[0][2];
    const double d = nx * px + ny * py + nz * pz;
    return (d * d) / n2;
}

bool projectedPointInTriangleQuant(std::shared_ptr<MCGAL::Mesh> mesh,
                                   MCGAL::Facet* f,
                                   MCGAL::PointInt q) {
    auto hs = boundedFaceHalfedges(f);
    if (!mesh || hs.size() != 3) return false;
    std::array<MCGAL::PointInt, 3> p{};
    for (int i = 0; i < 3; ++i) {
        p[static_cast<size_t>(i)] = mesh->floatPosToInt(hs[static_cast<size_t>(i)]->vertex()->point());
    }
    const double ax = p[1][0] - p[0][0];
    const double ay = p[1][1] - p[0][1];
    const double az = p[1][2] - p[0][2];
    const double bx = p[2][0] - p[0][0];
    const double by = p[2][1] - p[0][1];
    const double bz = p[2][2] - p[0][2];
    const double nx = ay * bz - az * by;
    const double ny = az * bx - ax * bz;
    const double nz = ax * by - ay * bx;
    int drop = 0;
    if (std::abs(ny) > std::abs(nx) && std::abs(ny) >= std::abs(nz)) drop = 1;
    else if (std::abs(nz) > std::abs(nx) && std::abs(nz) > std::abs(ny)) drop = 2;
    auto x2 = [&](MCGAL::PointInt v) -> double {
        return drop == 0 ? v[1] : v[0];
    };
    auto y2 = [&](MCGAL::PointInt v) -> double {
        return drop == 2 ? v[1] : v[2];
    };
    auto orient = [&](MCGAL::PointInt a,
                      MCGAL::PointInt b,
                      MCGAL::PointInt c) {
        return (x2(b) - x2(a)) * (y2(c) - y2(a)) -
               (y2(b) - y2(a)) * (x2(c) - x2(a));
    };
    const double area = orient(p[0], p[1], p[2]);
    if (std::abs(area) <= 0.0) return false;
    const double s0 = orient(p[0], p[1], q);
    const double s1 = orient(p[1], p[2], q);
    const double s2 = orient(p[2], p[0], q);
    const double eps = std::max(1.0, std::abs(area)) * 1e-9;
    if (area > 0.0) return s0 >= -eps && s1 >= -eps && s2 >= -eps;
    return s0 <= eps && s1 <= eps && s2 <= eps;
}

MCGAL::Facet* livingTriangleContainingQ(std::shared_ptr<MCGAL::Mesh> mesh,
                                        MCGAL::PointInt q,
                                        const std::set<MCGAL::Facet*>* forbiddenFaces) {
    if (!mesh) return nullptr;
    MCGAL::Facet* best = nullptr;
    double bestDist = std::numeric_limits<double>::infinity();
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        if (forbiddenFaces && forbiddenFaces->count(f)) continue;
        if (faceDegree(f) != 3) continue;
        if (!projectedPointInTriangleQuant(mesh, f, q)) continue;
        const double d2 = pointPlaneDistance2Quant(mesh, f, q);
        if (d2 < bestDist) {
            bestDist = d2;
            best = f;
        }
    }
    return best;
}

}  // namespace

static bool envEnabled(const char* name) {
    const char* e = std::getenv(name);
    return e && e[0] == '1';
}

static bool envEnabledDefault(const char* name, bool defaultValue) {
    const char* e = std::getenv(name);
    if (!e) return defaultValue;
    return e[0] != '0';
}

void PCLTTMStrictSymbolReadOperator::importFromRangeCodedBuffer(
    char* buffer, size_t& dataOffset, unsigned /*nbQuantBits*/) {
    constexpr size_t kArbitraryLargeView = static_cast<size_t>(1) << 32;
    std::span<const uint8_t> view(reinterpret_cast<const uint8_t*>(buffer + dataOffset),
                                  kArbitraryLargeView);

    ConnectivityBlobReader rd(view);
    const auto& hdr = rd.header();
    pmsf::requirePaperConnectivityBlobHeaderMatchesFamilySpec(
        hdr,
        pmsf::pclttmStrictConnectivityBlobFamilySpec(),
        "PCLTTMStrict");

    auto sl0 = rd.getStream(0, kPaperBlobStreamPCLTTMStrictConnectivity);
    size_t off = 0;
    const int32_t numPackedSyms = readI32(sl0, off, "numPackedSyms");
    const int32_t numNullBefore = readI32(sl0, off, "numNullBefore");
    const int32_t numNullAfter = readI32(sl0, off, "numNullAfter");
    const int32_t numRemoved = readI32(sl0, off, "numRemoved");
    const int32_t numA = readI32(sl0, off, "numA");
    const int32_t numB = readI32(sl0, off, "numB");
    const int32_t numLayers = readI32(sl0, off, "numLayers");
    if (numPackedSyms < 0 || numRemoved < 0 || numLayers <= 0) {
        throw std::runtime_error("[PCLTTMStrict decoder] invalid connectivity header");
    }

    layerSeedQs_.clear();
    layerSeedQs_.reserve(static_cast<size_t>(numLayers));
    std::vector<std::pair<int32_t, int32_t>> layerCounts(static_cast<size_t>(numLayers));
    int64_t packedCountCheck = 0;
    for (int i = 0; i < numLayers; ++i) {
        std::array<int, 6> seedQ{};
        for (int j = 0; j < 6; ++j) {
            seedQ[static_cast<size_t>(j)] = readI32(sl0, off, "layerSeedQ");
        }
        layerSeedQs_.push_back(seedQ);
        int32_t a = readI32(sl0, off, "layerA");
        int32_t b = readI32(sl0, off, "layerB");
        if (a < 0 || b < 0) {
            throw std::runtime_error("[PCLTTMStrict decoder] negative layer count");
        }
        layerCounts[static_cast<size_t>(i)] = {a, b};
        packedCountCheck += a + b;
    }
    if (packedCountCheck != numPackedSyms) {
        throw std::runtime_error("[PCLTTMStrict decoder] layer count does not match packed count");
    }
    packedNullInferenceAllowed_ = numNullAfter < numNullBefore;

    std::vector<uint8_t> packed;
    packed.reserve(static_cast<size_t>(numPackedSyms));
    if (numPackedSyms > 0) {
        auto payload = sl0.subspan(off);
        size_t rcOffset = 0;
        rangecoder rc;
        rc.p_data = rangeCoderInput(payload);
        rc.p_dataOffset = &rcOffset;
        if (start_decoding(&rc) < 0) {
            throw std::runtime_error("[PCLTTMStrict decoder] connectivity RC start failed");
        }
        qsmodel model;
        initqsmodel(&model, 5, 10, 1 << 9, nullptr, 0);
        for (int i = 0; i < numPackedSyms; ++i) {
            int lt = decode_culshift(&rc, 10);
            int sym = qsgetsym(&model, lt);
            int sf = 0, lf = 0;
            qsgetfreq(&model, sym, &sf, &lf);
            decode_update(&rc, sf, lf, 1 << 10);
            qsupdate(&model, sym);
            packed.push_back(static_cast<uint8_t>(sym));
        }
        done_decoding(&rc);
        deleteqsmodel(&model);
    }

    decodedSymbols_.clear();
    decodedSymbols_.reserve(static_cast<size_t>(numPackedSyms));
    size_t p = 0;
    for (int li = numLayers - 1; li >= 0; --li) {
        const auto [aCount, bCount] = layerCounts[static_cast<size_t>(li)];
        for (int i = 0; i < bCount; ++i) {
            uint8_t s = packed[p++];
            RDPatchSymbol out{};
            out.kind = strictToKind(s, true);
            out.phase = 1;
            out.bucket = -1;
            out.layer = static_cast<unsigned short>(li);
            decodedSymbols_.push_back(out);
        }
        for (int i = 0; i < aCount; ++i) {
            uint8_t s = packed[p++];
            RDPatchSymbol out{};
            out.kind = strictToKind(s, false);
            out.phase = 0;
            out.bucket = -1;
            out.layer = static_cast<unsigned short>(li);
            decodedSymbols_.push_back(out);
        }
    }

    auto sl1 = rd.getStream(0, kPaperBlobStreamPCLTTMStrictGeometry);
    size_t moff = 0;
    const int32_t geomLayers = readI32(sl1, moff, "geomLayers");
    if (geomLayers != numLayers) {
        throw std::runtime_error("[PCLTTMStrict decoder] geometry layer count mismatch");
    }
    struct GMeta { int32_t count, minX, rangeX, minY, rangeY, minZ, rangeZ; };
    std::vector<GMeta> metas;
    metas.reserve(static_cast<size_t>(geomLayers));
    for (int i = 0; i < geomLayers; ++i) {
        GMeta m{};
        m.count = readI32(sl1, moff, "geom.count");
        m.minX = readI32(sl1, moff, "geom.minX");
        m.rangeX = readI32(sl1, moff, "geom.rangeX");
        m.minY = readI32(sl1, moff, "geom.minY");
        m.rangeY = readI32(sl1, moff, "geom.rangeY");
        m.minZ = readI32(sl1, moff, "geom.minZ");
        m.rangeZ = readI32(sl1, moff, "geom.rangeZ");
        if (m.count < 0 || m.rangeX < 0 || m.rangeY < 0 || m.rangeZ < 0) {
            throw std::runtime_error("[PCLTTMStrict decoder] invalid geometry metadata");
        }
        metas.push_back(m);
    }

    decodedResidualsX_.clear();
    decodedResidualsY_.clear();
    decodedResidualsZ_.clear();
    decodedResidualsX_.reserve(static_cast<size_t>(numRemoved));
    decodedResidualsY_.reserve(static_cast<size_t>(numRemoved));
    decodedResidualsZ_.reserve(static_cast<size_t>(numRemoved));

    auto residualPayload = sl1.subspan(moff);
    size_t rcOffset = 0;
    rangecoder rc;
    rc.p_data = rangeCoderInput(residualPayload);
    rc.p_dataOffset = &rcOffset;
    if (start_decoding(&rc) < 0) {
        throw std::runtime_error("[PCLTTMStrict decoder] residual RC start failed");
    }

    auto decodeAxis = [&](int count, int mn, int range, std::vector<int>& out) {
        if (count <= 0) return;
        if (range <= 1) {
            out.insert(out.end(), static_cast<size_t>(count), mn);
            return;
        }
        const int lgTotf = std::max(10, ceilLog2Positive(range));
        const int rescale = 1 << std::min(15, std::max(1, lgTotf - 1));
        qsmodel model;
        initqsmodel(&model, range, lgTotf, rescale, nullptr, 0);
        for (int i = 0; i < count; ++i) {
            int lt = decode_culshift(&rc, lgTotf);
            int sym = qsgetsym(&model, lt);
            int sf = 0, lf = 0;
            qsgetfreq(&model, sym, &sf, &lf);
            decode_update(&rc, sf, lf, 1 << lgTotf);
            qsupdate(&model, sym);
            out.push_back(mn + sym);
        }
        deleteqsmodel(&model);
    };

    for (const GMeta& m : metas) {
        decodeAxis(m.count, m.minX, m.rangeX, decodedResidualsX_);
        decodeAxis(m.count, m.minY, m.rangeY, decodedResidualsY_);
        decodeAxis(m.count, m.minZ, m.rangeZ, decodedResidualsZ_);
    }
    done_decoding(&rc);
    if (static_cast<int32_t>(decodedResidualsX_.size()) != numRemoved ||
        decodedResidualsX_.size() != decodedResidualsY_.size() ||
        decodedResidualsX_.size() != decodedResidualsZ_.size()) {
        throw std::runtime_error("[PCLTTMStrict decoder] residual count mismatch");
    }

    decodedFaceRanks_.clear();
    decodedInputOffsets_.clear();
    decodedRingQs_.clear();
    decodedRingStartIndex_ = 0;
    decodedReconRingIds_.clear();
    decodedReconOriginalIds_.clear();
    decodedSymbolInputQs_.clear();
    decodedVInputQs_.clear();
    auto appendGroupedRankDeltas = [&](const std::vector<int32_t>& deltas,
                                       int32_t expectedRanks,
                                       const char* label) {
        if (expectedRanks < 0) {
            throw std::runtime_error(std::string("[PCLTTMStrict decoder] negative ") + label);
        }
        decodedFaceRanks_.reserve(decodedFaceRanks_.size() +
                                  static_cast<size_t>(expectedRanks));
        const size_t before = decodedFaceRanks_.size();
        size_t deltaCursor = 0;
        int32_t prevFirst = 0;
        for (const RDPatchSymbol& sym : decodedSymbols_) {
            if (sym.kind == RDPatchSymbolKind::NULL_PATCH) continue;
            if (decodedFaceRanks_.size() - before >= static_cast<size_t>(expectedRanks)) break;
            const int groupN = std::max(1, symbolValence(sym.kind) - 2);
            if (deltaCursor >= deltas.size()) break;
            int32_t prev = prevFirst + deltas[deltaCursor++];
            decodedFaceRanks_.push_back(prev);
            prevFirst = prev;
            for (int k = 1; k < groupN &&
                 decodedFaceRanks_.size() - before < static_cast<size_t>(expectedRanks); ++k) {
                if (deltaCursor >= deltas.size()) break;
                prev += deltas[deltaCursor++];
                decodedFaceRanks_.push_back(prev);
            }
        }
        if (decodedFaceRanks_.size() - before != static_cast<size_t>(expectedRanks) ||
            deltaCursor != deltas.size()) {
            throw std::runtime_error(std::string("[PCLTTMStrict decoder] rank group count mismatch in ") +
                                     label);
        }
    };
    auto decodeBitplaneRankDeltas = [&](std::span<const uint8_t> payload,
                                        int32_t n,
                                        int32_t bpBits,
                                        const char* label) {
        if (n < 0 || bpBits <= 0 || bpBits > 31) {
            throw std::runtime_error(std::string("[PCLTTMStrict decoder] invalid bitplane rank header in ") +
                                     label);
        }
        size_t rcOffset = 0;
        rangecoder rc;
        rc.p_data = rangeCoderInput(payload);
        rc.p_dataOffset = &rcOffset;
        if (start_decoding(&rc) < 0) {
            throw std::runtime_error(std::string("[PCLTTMStrict decoder] rank bitplane RC start failed in ") +
                                     label);
        }
        std::vector<qsmodel> bpFirst1(static_cast<size_t>(bpBits));
        std::vector<qsmodel> bpAfter1(static_cast<size_t>(bpBits));
        for (int b = 0; b < bpBits; ++b) {
            initqsmodel(&bpFirst1[static_cast<size_t>(b)], 2, 10, 1 << 9, nullptr, 0);
            initqsmodel(&bpAfter1[static_cast<size_t>(b)], 2, 10, 1 << 9, nullptr, 0);
        }
        std::vector<int32_t> deltas;
        deltas.reserve(static_cast<size_t>(n));
        for (int32_t i = 0; i < n; ++i) {
            uint32_t z = 0;
            bool seenOne = false;
            for (int b = bpBits - 1; b >= 0; --b) {
                qsmodel* model = seenOne ? &bpAfter1[static_cast<size_t>(b)]
                                         : &bpFirst1[static_cast<size_t>(b)];
                const int lt = decode_culshift(&rc, 10);
                const int bit = qsgetsym(model, lt);
                int sf = 0, lf = 0;
                qsgetfreq(model, bit, &sf, &lf);
                decode_update(&rc, sf, lf, 1 << 10);
                qsupdate(model, bit);
                if (bit) {
                    z |= (1u << b);
                    seenOne = true;
                }
            }
            deltas.push_back(static_cast<int32_t>((z >> 1) ^ (~(z & 1u) + 1u)));
        }
        done_decoding(&rc);
        for (int b = 0; b < bpBits; ++b) {
            deleteqsmodel(&bpFirst1[static_cast<size_t>(b)]);
            deleteqsmodel(&bpAfter1[static_cast<size_t>(b)]);
        }
        return deltas;
    };
    auto decodeVarintRankDeltas = [&](std::span<const uint8_t> payload,
                                      int32_t n,
                                      const char* label) {
        std::vector<int32_t> deltas;
        if (n < 0) {
            throw std::runtime_error(std::string("[PCLTTMStrict decoder] negative varint rank count in ") +
                                     label);
        }
        deltas.reserve(static_cast<size_t>(n));
        size_t voff = 0;
        for (int32_t i = 0; i < n; ++i) {
            deltas.push_back(readZigZagVarI32(payload, voff, label));
        }
        if (voff != payload.size()) {
            throw std::runtime_error(std::string("[PCLTTMStrict decoder] trailing varint rank bytes in ") +
                                     label);
        }
        return deltas;
    };
    auto appendAptmLocatorV1Ranks = [&](std::span<const uint8_t> s,
                                        size_t& off,
                                        const char* label) {
        const int32_t numGroups = readI32(s, off, "locator.group.count");
        const int32_t numRanks = readI32(s, off, "locator.rank.count");
        const int32_t mode = readI32(s, off, "locator.mode");
        const int32_t aux = readI32(s, off, "locator.aux");
        if (numGroups < 0 || numRanks < 0) {
            throw std::runtime_error("[PCLTTMStrict decoder] invalid APTM locator header");
        }
        int32_t inferredGroups = 0;
        for (const RDPatchSymbol& sym : decodedSymbols_) {
            if (sym.kind != RDPatchSymbolKind::NULL_PATCH) ++inferredGroups;
        }
        const int32_t expectedGroups =
            decodedRingStartIndex_ > 0 ? decodedRingStartIndex_ : inferredGroups;
        if (expectedGroups != numGroups) {
            throw std::runtime_error("[PCLTTMStrict decoder] APTM locator group count mismatch");
        }
        std::span<const uint8_t> payload = s.subspan(off);
        std::vector<int32_t> deltas;
        if (mode == 0) {
            deltas = decodeBitplaneRankDeltas(payload, numRanks, aux, label);
        } else if (mode == 1) {
            if (aux < 0 || static_cast<size_t>(aux) != payload.size()) {
                throw std::runtime_error("[PCLTTMStrict decoder] invalid APTM locator varint size");
            }
            deltas = decodeVarintRankDeltas(payload, numRanks, label);
        } else if (mode == 2) {
            if (aux < 0) {
                throw std::runtime_error("[PCLTTMStrict decoder] invalid APTM locator zlib size");
            }
            std::vector<uint8_t> raw;
            if (!zlibDecompress(payload.data(), payload.size(), raw,
                                static_cast<size_t>(aux))) {
                throw std::runtime_error("[PCLTTMStrict decoder] APTM locator zlib decode failed");
            }
            deltas = decodeVarintRankDeltas(
                std::span<const uint8_t>(raw.data(), raw.size()), numRanks, label);
        } else {
            throw std::runtime_error("[PCLTTMStrict decoder] unknown APTM locator mode");
        }
        appendGroupedRankDeltas(deltas, numRanks, label);
        off = s.size();
    };
    auto appendGroupBitplaneRanks = [&](std::span<const uint8_t> s,
                                        size_t& off,
                                        const char* label) {
        const int32_t n = readI32(s, off, label);
        const int32_t bpBits = readI32(s, off, "faceRank.group.bits");
        if (n < 0 || bpBits <= 0 || bpBits > 31) {
            throw std::runtime_error("[PCLTTMStrict decoder] invalid group bitplane rank header");
        }
        decodedFaceRanks_.reserve(decodedFaceRanks_.size() + static_cast<size_t>(n));
        auto payload = s.subspan(off);
        size_t rcOffset = 0;
        rangecoder rc;
        rc.p_data = rangeCoderInput(payload);
        rc.p_dataOffset = &rcOffset;
        if (start_decoding(&rc) < 0) {
            throw std::runtime_error("[PCLTTMStrict decoder] rank bitplane RC start failed");
        }
        std::vector<qsmodel> bpFirst1(static_cast<size_t>(bpBits));
        std::vector<qsmodel> bpAfter1(static_cast<size_t>(bpBits));
        for (int b = 0; b < bpBits; ++b) {
            initqsmodel(&bpFirst1[static_cast<size_t>(b)], 2, 10, 1 << 9, nullptr, 0);
            initqsmodel(&bpAfter1[static_cast<size_t>(b)], 2, 10, 1 << 9, nullptr, 0);
        }
        auto decodeDelta = [&]() {
            uint32_t z = 0;
            bool seenOne = false;
            for (int b = bpBits - 1; b >= 0; --b) {
                qsmodel* model = seenOne ? &bpAfter1[static_cast<size_t>(b)]
                                         : &bpFirst1[static_cast<size_t>(b)];
                const int lt = decode_culshift(&rc, 10);
                const int bit = qsgetsym(model, lt);
                int sf = 0, lf = 0;
                qsgetfreq(model, bit, &sf, &lf);
                decode_update(&rc, sf, lf, 1 << 10);
                qsupdate(model, bit);
                if (bit) {
                    z |= (1u << b);
                    seenOne = true;
                }
            }
            return static_cast<int32_t>((z >> 1) ^ (~(z & 1u) + 1u));
        };
        const size_t before = decodedFaceRanks_.size();
        int32_t prevFirst = 0;
        for (const RDPatchSymbol& sym : decodedSymbols_) {
            if (sym.kind == RDPatchSymbolKind::NULL_PATCH) continue;
            if (decodedFaceRanks_.size() - before >= static_cast<size_t>(n)) break;
            const int groupN = std::max(1, symbolValence(sym.kind) - 2);
            int32_t prev = prevFirst + decodeDelta();
            decodedFaceRanks_.push_back(prev);
            prevFirst = prev;
            for (int k = 1; k < groupN &&
                 decodedFaceRanks_.size() - before < static_cast<size_t>(n); ++k) {
                prev += decodeDelta();
                decodedFaceRanks_.push_back(prev);
            }
        }
        done_decoding(&rc);
        for (int b = 0; b < bpBits; ++b) {
            deleteqsmodel(&bpFirst1[static_cast<size_t>(b)]);
            deleteqsmodel(&bpAfter1[static_cast<size_t>(b)]);
        }
        if (decodedFaceRanks_.size() - before != static_cast<size_t>(n)) {
            throw std::runtime_error("[PCLTTMStrict decoder] face rank group count mismatch");
        }
        off = s.size();
    };
    if (pmsf::paperBlobStreamCountPCLTTMStrictHasOffsets(hdr.num_streams)) {
        auto sl2 = rd.getStream(0, kPaperBlobStreamPCLTTMStrictOffsets);
        if (sl2.size() >= 4) {
            size_t roff = 0;
            const int32_t markerOrCount = readI32(sl2, roff, "inputOffset.count");
            if (markerOrCount == -4) {
                const int32_t numOffsets = readI32(sl2, roff, "inputOffset.split.count");
                const int32_t offsetBytes = readI32(sl2, roff, "inputOffset.offsetBytes");
                const int32_t reverseBytes = readI32(sl2, roff, "inputOffset.reverseBytes");
                const int32_t sideBytes = readI32(sl2, roff, "inputOffset.sideBytes");
                if (numOffsets < 0 || offsetBytes < 0 || reverseBytes < 0 ||
                    sideBytes < 0 ||
                    roff + static_cast<size_t>(offsetBytes) +
                            static_cast<size_t>(reverseBytes) +
                            static_cast<size_t>(sideBytes) > sl2.size()) {
                    throw std::runtime_error("[PCLTTMStrict decoder] invalid split offset stream");
                }
                auto decodeSmallAlphabet = [&](std::span<const uint8_t> payload,
                                               int alphabet,
                                               const char* what) {
                    std::vector<uint8_t> out;
                    out.reserve(static_cast<size_t>(numOffsets));
                    if (numOffsets == 0) return out;
                    size_t rcOffset = 0;
                    rangecoder rc;
                    rc.p_data = rangeCoderInput(payload);
                    rc.p_dataOffset = &rcOffset;
                    if (start_decoding(&rc) < 0) {
                        throw std::runtime_error(std::string("[PCLTTMStrict decoder] ") +
                                                 what + " RC start failed");
                    }
                    qsmodel model;
                    initqsmodel(&model, alphabet, 10, 1 << 9, nullptr, 0);
                    for (int32_t i = 0; i < numOffsets; ++i) {
                        int lt = decode_culshift(&rc, 10);
                        int sym = qsgetsym(&model, lt);
                        int sf = 0, lf = 0;
                        qsgetfreq(&model, sym, &sf, &lf);
                        decode_update(&rc, sf, lf, 1 << 10);
                        qsupdate(&model, sym);
                        out.push_back(static_cast<uint8_t>(sym));
                    }
                    done_decoding(&rc);
                    deleteqsmodel(&model);
                    return out;
                };
                const auto offsetPayload =
                    sl2.subspan(roff, static_cast<size_t>(offsetBytes));
                roff += static_cast<size_t>(offsetBytes);
                const auto reversePayload =
                    sl2.subspan(roff, static_cast<size_t>(reverseBytes));
                roff += static_cast<size_t>(reverseBytes);
                const auto sidePayload =
                    sl2.subspan(roff, static_cast<size_t>(sideBytes));
                std::vector<uint8_t> offsets =
                    decodeSmallAlphabet(offsetPayload, 8, "offset");
                std::vector<uint8_t> reverses =
                    decodeSmallAlphabet(reversePayload, 2, "reverse");
                std::vector<uint8_t> sides =
                    decodeSmallAlphabet(sidePayload, 4, "side");
                if (offsets.size() != static_cast<size_t>(numOffsets) ||
                    reverses.size() != static_cast<size_t>(numOffsets) ||
                    sides.size() != static_cast<size_t>(numOffsets)) {
                    throw std::runtime_error("[PCLTTMStrict decoder] split offset count mismatch");
                }
                decodedInputOffsets_.reserve(static_cast<size_t>(numOffsets));
                for (int32_t i = 0; i < numOffsets; ++i) {
                    decodedInputOffsets_.push_back(
                        static_cast<uint8_t>((offsets[static_cast<size_t>(i)] & 0x07) |
                                             ((reverses[static_cast<size_t>(i)] & 0x01) << 3) |
                                             ((sides[static_cast<size_t>(i)] & 0x03) << 4)));
                }
            } else if (markerOrCount < 0) {
                throw std::runtime_error("[PCLTTMStrict decoder] negative input offset count");
            } else {
                const int32_t numOffsets = markerOrCount;
                decodedInputOffsets_.reserve(static_cast<size_t>(numOffsets));
                if (numOffsets > 0) {
                    auto payload = sl2.subspan(roff);
                    size_t rcOffset = 0;
                    rangecoder rc;
                    rc.p_data = rangeCoderInput(payload);
                    rc.p_dataOffset = &rcOffset;
                    if (start_decoding(&rc) < 0) {
                        throw std::runtime_error("[PCLTTMStrict decoder] offset RC start failed");
                    }
                    qsmodel model;
                    initqsmodel(&model, 16, 10, 1 << 9, nullptr, 0);
                    for (int32_t i = 0; i < numOffsets; ++i) {
                        int lt = decode_culshift(&rc, 10);
                        int sym = qsgetsym(&model, lt);
                        int sf = 0, lf = 0;
                        qsgetfreq(&model, sym, &sf, &lf);
                        decode_update(&rc, sf, lf, 1 << 10);
                        qsupdate(&model, sym);
                        decodedInputOffsets_.push_back(static_cast<uint8_t>(sym & 0x0f));
                    }
                    done_decoding(&rc);
                    deleteqsmodel(&model);
                }
            }
        }
    }
    if (pmsf::paperBlobStreamCountPCLTTMStrictHasReplay(hdr.num_streams)) {
        auto sl3 = rd.getStream(0, kPaperBlobStreamPCLTTMStrictReplay);
        if (sl3.size() >= 4) {
            size_t roff = 0;
            int32_t numRings = readI32(sl3, roff, "ring.count");
            const bool compactU16 = (numRings == -1);
            const bool deltaU16 = (numRings == -2);
            const bool faceRanks = (numRings == -3);
            const bool faceRanksVarint = (numRings == -8);
            const bool faceRanksGroupVarint = (numRings == -9);
            const bool faceRanksGroupBitplane = (numRings == -10);
            const bool aptmLocatorV1 = (numRings == -13);
            const bool aptmHybridLocator = (numRings == -14);
            const bool aptmHybridReconLocator = (numRings == -15);
            const bool aptmSparseRingTail = (numRings == -16);
            const bool reconRingIds = (numRings == -11 || numRings == -12);
            const bool reconRingIdsWithOriginalIds = (numRings == -12);
            const bool dictionaryU16 = (numRings == -5 || numRings == -6);
            const bool dictionaryWideIds = (numRings == -6);
            const bool dictionaryVarint = (numRings == -7);
            if (compactU16) {
                numRings = readI32(sl3, roff, "ring.compact.count");
            } else if (deltaU16) {
                numRings = readI32(sl3, roff, "ring.delta.count");
            } else if (dictionaryU16) {
                numRings = readI32(sl3, roff, "ring.dict.count");
            } else if (dictionaryVarint) {
                numRings = readI32(sl3, roff, "ring.varint.count");
            } else if (faceRanks) {
                const int32_t n = readI32(sl3, roff, "faceRank.count");
                if (n < 0) {
                    throw std::runtime_error("[PCLTTMStrict decoder] negative face rank count");
                }
                decodedFaceRanks_.reserve(static_cast<size_t>(n));
                for (int32_t i = 0; i < n; ++i) {
                    decodedFaceRanks_.push_back(readI32(sl3, roff, "faceRank.value"));
                }
                numRings = 0;
            } else if (faceRanksVarint) {
                const int32_t n = readI32(sl3, roff, "faceRank.varint.count");
                if (n < 0) {
                    throw std::runtime_error("[PCLTTMStrict decoder] negative face rank count");
                }
                decodedFaceRanks_.reserve(static_cast<size_t>(n));
                int32_t prev = 0;
                for (int32_t i = 0; i < n; ++i) {
                    prev += readZigZagVarI32(sl3, roff, "faceRank.varint.delta");
                    decodedFaceRanks_.push_back(prev);
                }
                numRings = 0;
            } else if (faceRanksGroupVarint) {
                const int32_t n = readI32(sl3, roff, "faceRank.group.count");
                if (n < 0) {
                    throw std::runtime_error("[PCLTTMStrict decoder] negative face rank count");
                }
                decodedFaceRanks_.reserve(static_cast<size_t>(n));
                int32_t prevFirst = 0;
                for (const RDPatchSymbol& sym : decodedSymbols_) {
                    if (sym.kind == RDPatchSymbolKind::NULL_PATCH) continue;
                    if (static_cast<int32_t>(decodedFaceRanks_.size()) >= n) break;
                    const int groupN = std::max(1, symbolValence(sym.kind) - 2);
                    int32_t prev = prevFirst + readZigZagVarI32(sl3, roff, "faceRank.group.first");
                    decodedFaceRanks_.push_back(prev);
                    prevFirst = prev;
                    for (int k = 1; k < groupN &&
                         static_cast<int32_t>(decodedFaceRanks_.size()) < n; ++k) {
                        prev += readZigZagVarI32(sl3, roff, "faceRank.group.delta");
                        decodedFaceRanks_.push_back(prev);
                    }
                }
                if (static_cast<int32_t>(decodedFaceRanks_.size()) != n) {
                    throw std::runtime_error("[PCLTTMStrict decoder] face rank group count mismatch");
                }
                numRings = 0;
            } else if (faceRanksGroupBitplane) {
                appendGroupBitplaneRanks(sl3, roff, "faceRank.group.count");
                numRings = 0;
            } else if (aptmLocatorV1) {
                appendAptmLocatorV1Ranks(sl3, roff, "aptm.locator.v1");
                numRings = 0;
            } else if (aptmSparseRingTail) {
                const int32_t ringStart = readI32(sl3, roff, "aptm.sparseRing.start");
                const int32_t ringBytes = readI32(sl3, roff, "aptm.sparseRing.bytes");
                if (ringStart < 0 || ringBytes < 0 ||
                    roff + static_cast<size_t>(ringBytes) > sl3.size()) {
                    throw std::runtime_error("[PCLTTMStrict decoder] invalid sparse ring tail header");
                }
                decodedRingStartIndex_ = ringStart;
                std::span<const uint8_t> ringSpan =
                    sl3.subspan(roff, static_cast<size_t>(ringBytes));
                roff += static_cast<size_t>(ringBytes);
                size_t ringOff = 0;
                const int32_t ringMarker =
                    readI32(ringSpan, ringOff, "aptm.sparseRing.marker");
                if (ringMarker != -7) {
                    throw std::runtime_error("[PCLTTMStrict decoder] invalid sparse ring tail payload");
                }
                auto readVarU32Sparse = [&](const char* what) {
                    uint32_t value = 0;
                    int shift = 0;
                    for (;;) {
                        if (ringOff >= ringSpan.size() || shift > 28) {
                            throw std::runtime_error(
                                std::string("[PCLTTMStrict decoder] truncated ") + what);
                        }
                        const uint8_t b = ringSpan[ringOff++];
                        value |= static_cast<uint32_t>(b & 0x7fu) << shift;
                        if ((b & 0x80u) == 0) break;
                        shift += 7;
                    }
                    return value;
                };
                auto readZigZagSparse = [&](const char* what) {
                    const uint32_t z = readVarU32Sparse(what);
                    return static_cast<int32_t>((z >> 1) ^
                                                (~(z & 1u) + 1u));
                };
                const int32_t ringCount = readI32(ringSpan, ringOff, "aptm.sparseRing.count");
                const int32_t dictN = readI32(ringSpan, ringOff, "aptm.sparseRing.dict.size");
                if (ringCount < 0 || dictN < 0) {
                    throw std::runtime_error("[PCLTTMStrict decoder] invalid sparse ring tail counts");
                }
                std::vector<std::array<int, 3>> dict;
                dict.reserve(static_cast<size_t>(dictN));
                std::array<int32_t, 3> prevQ{0, 0, 0};
                for (int32_t i = 0; i < dictN; ++i) {
                    std::array<int, 3> q{};
                    for (int axis = 0; axis < 3; ++axis) {
                        prevQ[static_cast<size_t>(axis)] +=
                            readZigZagSparse("aptm.sparseRing.dict.delta");
                        q[static_cast<size_t>(axis)] =
                            static_cast<int>(prevQ[static_cast<size_t>(axis)]);
                    }
                    dict.push_back(q);
                }
                std::vector<uint8_t> sizes;
                sizes.reserve(static_cast<size_t>(ringCount));
                for (int32_t i = 0; i < ringCount; ++i) {
                    if (ringOff >= ringSpan.size()) {
                        throw std::runtime_error("[PCLTTMStrict decoder] truncated sparse ring tail size");
                    }
                    const uint8_t n = ringSpan[ringOff++];
                    if (n > 16) {
                        throw std::runtime_error("[PCLTTMStrict decoder] invalid sparse ring tail size");
                    }
                    sizes.push_back(n);
                }
                decodedRingQs_.reserve(static_cast<size_t>(ringCount));
                for (int32_t i = 0; i < ringCount; ++i) {
                    std::vector<std::array<int, 3>> ring;
                    ring.reserve(sizes[static_cast<size_t>(i)]);
                    int32_t prev = 0;
                    for (uint8_t j = 0; j < sizes[static_cast<size_t>(i)]; ++j) {
                        prev += readZigZagSparse("aptm.sparseRing.id.delta");
                        if (prev < 0 || static_cast<size_t>(prev) >= dict.size()) {
                            throw std::runtime_error("[PCLTTMStrict decoder] sparse ring id out of range");
                        }
                        ring.push_back(dict[static_cast<size_t>(prev)]);
                    }
                    decodedRingQs_.push_back(std::move(ring));
                }
                numRings = 0;
            } else if (aptmHybridLocator || aptmHybridReconLocator) {
                const int32_t ringStart = readI32(sl3, roff, "aptm.hybrid.ringStart");
                const int32_t rankBytes = readI32(sl3, roff, "aptm.hybrid.rankBytes");
                const int32_t ringBytes = readI32(sl3, roff, "aptm.hybrid.ringBytes");
                if (ringStart < 0 || rankBytes < 0 || ringBytes < 0 ||
                    roff + static_cast<size_t>(rankBytes) +
                               static_cast<size_t>(ringBytes) > sl3.size()) {
                    throw std::runtime_error("[PCLTTMStrict decoder] invalid APTM hybrid locator header");
                }
                decodedRingStartIndex_ = ringStart;
                std::span<const uint8_t> rankSpan =
                    sl3.subspan(roff, static_cast<size_t>(rankBytes));
                roff += static_cast<size_t>(rankBytes);
                size_t rankOff = 0;
                const int32_t rankMarker = readI32(rankSpan, rankOff, "aptm.hybrid.rank.marker");
                if (rankMarker != -13) {
                    throw std::runtime_error("[PCLTTMStrict decoder] invalid APTM hybrid rank payload");
                }
                appendAptmLocatorV1Ranks(rankSpan, rankOff, "aptm.hybrid.rank");
                std::span<const uint8_t> ringSpan =
                    sl3.subspan(roff, static_cast<size_t>(ringBytes));
                roff += static_cast<size_t>(ringBytes);
                size_t ringOff = 0;
                const int32_t ringMarker = readI32(ringSpan, ringOff, "aptm.hybrid.ring.marker");
                auto readVarU32Hybrid = [&](const char* what) {
                    uint32_t value = 0;
                    int shift = 0;
                    for (;;) {
                        if (ringOff >= ringSpan.size() || shift > 28) {
                            throw std::runtime_error(
                                std::string("[PCLTTMStrict decoder] truncated ") + what);
                        }
                        const uint8_t b = ringSpan[ringOff++];
                        value |= static_cast<uint32_t>(b & 0x7fu) << shift;
                        if ((b & 0x80u) == 0) break;
                        shift += 7;
                    }
                    return value;
                };
                auto readZigZagHybrid = [&](const char* what) {
                    const uint32_t z = readVarU32Hybrid(what);
                    return static_cast<int32_t>((z >> 1) ^
                                                (~(z & 1u) + 1u));
                };
                if (aptmHybridReconLocator) {
                    if (ringMarker != -11) {
                        throw std::runtime_error(
                            "[PCLTTMStrict decoder] invalid APTM hybrid recon-ring payload");
                    }
                    const int32_t ringCount =
                        readI32(ringSpan, ringOff, "aptm.hybrid.recon.count");
                    if (ringCount < 0) {
                        throw std::runtime_error(
                            "[PCLTTMStrict decoder] invalid APTM hybrid recon-ring count");
                    }
                    std::vector<uint8_t> sizes;
                    sizes.reserve(static_cast<size_t>(ringCount));
                    for (int32_t i = 0; i < ringCount; ++i) {
                        if (ringOff >= ringSpan.size()) {
                            throw std::runtime_error(
                                "[PCLTTMStrict decoder] truncated APTM hybrid recon-ring size");
                        }
                        sizes.push_back(ringSpan[ringOff++]);
                    }
                    decodedReconRingIds_.reserve(static_cast<size_t>(ringCount));
                    for (int32_t i = 0; i < ringCount; ++i) {
                        std::vector<int> ring;
                        ring.reserve(sizes[static_cast<size_t>(i)]);
                        int32_t prev = 0;
                        for (uint8_t j = 0; j < sizes[static_cast<size_t>(i)]; ++j) {
                            prev += readZigZagHybrid("aptm.hybrid.recon.id.delta");
                            ring.push_back(static_cast<int>(prev));
                        }
                        decodedReconRingIds_.push_back(std::move(ring));
                    }
                } else {
                    if (ringMarker != -7) {
                        throw std::runtime_error("[PCLTTMStrict decoder] invalid APTM hybrid ring payload");
                    }
                    const int32_t ringCount = readI32(ringSpan, ringOff, "aptm.hybrid.ring.count");
                    const int32_t dictN = readI32(ringSpan, ringOff, "aptm.hybrid.ring.dict.size");
                    if (ringCount < 0 || dictN < 0) {
                        throw std::runtime_error("[PCLTTMStrict decoder] invalid APTM hybrid ring counts");
                    }
                    std::vector<std::array<int, 3>> dict;
                    dict.reserve(static_cast<size_t>(dictN));
                    std::array<int32_t, 3> prevQ{0, 0, 0};
                    for (int32_t i = 0; i < dictN; ++i) {
                        std::array<int, 3> q{};
                        for (int axis = 0; axis < 3; ++axis) {
                            prevQ[static_cast<size_t>(axis)] +=
                                readZigZagHybrid("aptm.hybrid.ring.dict.delta");
                            q[static_cast<size_t>(axis)] =
                                static_cast<int>(prevQ[static_cast<size_t>(axis)]);
                        }
                        dict.push_back(q);
                    }
                    std::vector<uint8_t> sizes;
                    sizes.reserve(static_cast<size_t>(ringCount));
                    for (int32_t i = 0; i < ringCount; ++i) {
                        if (ringOff >= ringSpan.size()) {
                            throw std::runtime_error("[PCLTTMStrict decoder] truncated APTM hybrid ring size");
                        }
                        const uint8_t n = ringSpan[ringOff++];
                        if (n > 16) {
                            throw std::runtime_error("[PCLTTMStrict decoder] invalid APTM hybrid ring size");
                        }
                        sizes.push_back(n);
                    }
                    decodedRingQs_.reserve(static_cast<size_t>(ringCount));
                    for (int32_t i = 0; i < ringCount; ++i) {
                        std::vector<std::array<int, 3>> ring;
                        ring.reserve(sizes[static_cast<size_t>(i)]);
                        int32_t prev = 0;
                        for (uint8_t j = 0; j < sizes[static_cast<size_t>(i)]; ++j) {
                            prev += readZigZagHybrid("aptm.hybrid.ring.id.delta");
                            if (prev < 0 || static_cast<size_t>(prev) >= dict.size()) {
                                throw std::runtime_error("[PCLTTMStrict decoder] APTM hybrid ring id out of range");
                            }
                            ring.push_back(dict[static_cast<size_t>(prev)]);
                        }
                        decodedRingQs_.push_back(std::move(ring));
                    }
                }
                numRings = 0;
            } else if (reconRingIds) {
                const int32_t n = readI32(sl3, roff, "reconRing.count");
                if (n < 0) {
                    throw std::runtime_error("[PCLTTMStrict decoder] negative recon ring count");
                }
                int32_t originalIdCount = 0;
                if (reconRingIdsWithOriginalIds) {
                    originalIdCount = readI32(sl3, roff, "reconRing.originalIdCount");
                    if (originalIdCount < 0) {
                        throw std::runtime_error(
                            "[PCLTTMStrict decoder] negative recon original-id count");
                    }
                }
                std::vector<uint8_t> sizes;
                sizes.reserve(static_cast<size_t>(n));
                for (int32_t i = 0; i < n; ++i) {
                    if (roff >= sl3.size()) {
                        throw std::runtime_error("[PCLTTMStrict decoder] truncated recon ring size");
                    }
                    sizes.push_back(sl3[roff++]);
                }
                decodedReconRingIds_.reserve(static_cast<size_t>(n));
                for (int32_t i = 0; i < n; ++i) {
                    std::vector<int> ring;
                    ring.reserve(sizes[static_cast<size_t>(i)]);
                    int32_t prev = 0;
                    for (uint8_t j = 0; j < sizes[static_cast<size_t>(i)]; ++j) {
                        prev += readZigZagVarI32(sl3, roff, "reconRing.idDelta");
                        ring.push_back(static_cast<int>(prev));
                    }
                    decodedReconRingIds_.push_back(std::move(ring));
                }
                if (reconRingIdsWithOriginalIds) {
                    decodedReconOriginalIds_.reserve(static_cast<size_t>(originalIdCount));
                    int32_t prev = 0;
                    for (int32_t i = 0; i < originalIdCount; ++i) {
                        prev += readZigZagVarI32(sl3, roff, "reconRing.originalIdDelta");
                        decodedReconOriginalIds_.push_back(static_cast<int>(prev));
                    }
                }
                numRings = 0;
            }
            if (numRings < 0) {
                throw std::runtime_error("[PCLTTMStrict decoder] negative ring count");
            }
            decodedRingQs_.reserve(static_cast<size_t>(numRings));
            if (faceRanks || faceRanksVarint || faceRanksGroupVarint ||
                faceRanksGroupBitplane || aptmLocatorV1 || aptmHybridLocator ||
                aptmHybridReconLocator ||
                reconRingIds) {
                // Stream 3 is used as a compact patch locator stream.
            } else if (dictionaryVarint) {
                auto readVarU32 = [&](const char* what) {
                    uint32_t value = 0;
                    int shift = 0;
                    for (;;) {
                        if (roff >= sl3.size() || shift > 28) {
                            throw std::runtime_error(
                                std::string("[PCLTTMStrict decoder] truncated ") + what);
                        }
                        const uint8_t b = sl3[roff++];
                        value |= static_cast<uint32_t>(b & 0x7fu) << shift;
                        if ((b & 0x80u) == 0) break;
                        shift += 7;
                    }
                    return value;
                };
                auto readZigZag = [&](const char* what) {
                    const uint32_t z = readVarU32(what);
                    return static_cast<int32_t>((z >> 1) ^
                                                (~(z & 1u) + 1u));
                };
                const int32_t dictN = readI32(sl3, roff, "ring.varint.dict.size");
                if (dictN < 0) {
                    throw std::runtime_error("[PCLTTMStrict decoder] negative ring dictionary size");
                }
                std::vector<std::array<int, 3>> dict;
                dict.reserve(static_cast<size_t>(dictN));
                std::array<int32_t, 3> prevQ{0, 0, 0};
                for (int32_t i = 0; i < dictN; ++i) {
                    std::array<int, 3> q{};
                    for (int axis = 0; axis < 3; ++axis) {
                        prevQ[static_cast<size_t>(axis)] +=
                            readZigZag("ring.varint.dict.delta");
                        q[static_cast<size_t>(axis)] =
                            static_cast<int>(prevQ[static_cast<size_t>(axis)]);
                    }
                    dict.push_back(q);
                }
                std::vector<uint8_t> sizes;
                sizes.reserve(static_cast<size_t>(numRings));
                for (int32_t i = 0; i < numRings; ++i) {
                    if (roff >= sl3.size()) {
                        throw std::runtime_error("[PCLTTMStrict decoder] truncated ring.varint.size");
                    }
                    uint8_t n = sl3[roff++];
                    if (n > 16) {
                        throw std::runtime_error("[PCLTTMStrict decoder] invalid varint ring size");
                    }
                    sizes.push_back(n);
                }
                for (int32_t i = 0; i < numRings; ++i) {
                    std::vector<std::array<int, 3>> ring;
                    ring.reserve(sizes[static_cast<size_t>(i)]);
                    int32_t prev = 0;
                    for (uint8_t j = 0; j < sizes[static_cast<size_t>(i)]; ++j) {
                        prev += readZigZag("ring.varint.id.delta");
                        if (prev < 0 || static_cast<size_t>(prev) >= dict.size()) {
                            throw std::runtime_error("[PCLTTMStrict decoder] ring dictionary id out of range");
                        }
                        ring.push_back(dict[static_cast<size_t>(prev)]);
                    }
                    decodedRingQs_.push_back(std::move(ring));
                }
            } else if (dictionaryU16) {
                const int32_t dictN = readI32(sl3, roff, "ring.dict.size");
                if (dictN < 0) {
                    throw std::runtime_error("[PCLTTMStrict decoder] negative ring dictionary size");
                }
                std::vector<std::array<int, 3>> dict;
                dict.reserve(static_cast<size_t>(dictN));
                for (int32_t i = 0; i < dictN; ++i) {
                    std::array<int, 3> q{};
                    for (int axis = 0; axis < 3; ++axis) {
                        q[static_cast<size_t>(axis)] =
                            static_cast<int>(readU16(sl3, roff, "ring.dict.q"));
                    }
                    dict.push_back(q);
                }
                std::vector<uint8_t> sizes;
                sizes.reserve(static_cast<size_t>(numRings));
                size_t totalIds = 0;
                for (int32_t i = 0; i < numRings; ++i) {
                    if (roff >= sl3.size()) {
                        throw std::runtime_error("[PCLTTMStrict decoder] truncated ring.dict.size");
                    }
                    uint8_t n = sl3[roff++];
                    if (n > 16) {
                        throw std::runtime_error("[PCLTTMStrict decoder] invalid dict ring size");
                    }
                    sizes.push_back(n);
                    totalIds += n;
                }
                (void)totalIds;
                for (int32_t i = 0; i < numRings; ++i) {
                    std::vector<std::array<int, 3>> ring;
                    ring.reserve(sizes[static_cast<size_t>(i)]);
                    for (uint8_t j = 0; j < sizes[static_cast<size_t>(i)]; ++j) {
                        uint32_t id = 0;
                        if (dictionaryWideIds) {
                            id = static_cast<uint32_t>(readI32(sl3, roff, "ring.dict.id32"));
                        } else {
                            id = static_cast<uint32_t>(readU16(sl3, roff, "ring.dict.id16"));
                        }
                        if (id >= dict.size()) {
                            throw std::runtime_error("[PCLTTMStrict decoder] ring dictionary id out of range");
                        }
                        ring.push_back(dict[static_cast<size_t>(id)]);
                    }
                    decodedRingQs_.push_back(std::move(ring));
                }
            } else if (deltaU16) {
                std::vector<uint8_t> sizes;
                sizes.reserve(static_cast<size_t>(numRings));
                size_t totalRingVerts = 0;
                for (int32_t i = 0; i < numRings; ++i) {
                    if (roff >= sl3.size()) {
                        throw std::runtime_error("[PCLTTMStrict decoder] truncated ring.delta.size");
                    }
                    uint8_t n = sl3[roff++];
                    if (n > 16) {
                        throw std::runtime_error("[PCLTTMStrict decoder] invalid delta ring size");
                    }
                    sizes.push_back(n);
                    totalRingVerts += n;
                }
                auto payload = sl3.subspan(roff);
                size_t rcOffset = 0;
                rangecoder rc;
                rc.p_data = rangeCoderInput(payload);
                rc.p_dataOffset = &rcOffset;
                if (start_decoding(&rc) < 0) {
                    throw std::runtime_error("[PCLTTMStrict decoder] ring delta RC start failed");
                }
                qsmodel mx, my, mz;
                initqsmodel(&mx, 65536, 16, 1 << 15, nullptr, 0);
                initqsmodel(&my, 65536, 16, 1 << 15, nullptr, 0);
                initqsmodel(&mz, 65536, 16, 1 << 15, nullptr, 0);
                std::array<uint16_t, 3> prev{0, 0, 0};
                auto decodeSym = [&](qsmodel& model, int axis) {
                    int lt = decode_culshift(&rc, 16);
                    int sym = qsgetsym(&model, lt);
                    int sf = 0, lf = 0;
                    qsgetfreq(&model, sym, &sf, &lf);
                    decode_update(&rc, sf, lf, 1 << 16);
                    qsupdate(&model, sym);
                    uint16_t v = static_cast<uint16_t>(
                        prev[static_cast<size_t>(axis)] + static_cast<uint16_t>(sym));
                    prev[static_cast<size_t>(axis)] = v;
                    return static_cast<int>(v);
                };
                for (int32_t i = 0; i < numRings; ++i) {
                    std::vector<std::array<int, 3>> ring;
                    ring.reserve(sizes[static_cast<size_t>(i)]);
                    for (uint8_t j = 0; j < sizes[static_cast<size_t>(i)]; ++j) {
                        std::array<int, 3> q{};
                        q[0] = decodeSym(mx, 0);
                        q[1] = decodeSym(my, 1);
                        q[2] = decodeSym(mz, 2);
                        ring.push_back(q);
                    }
                    decodedRingQs_.push_back(std::move(ring));
                }
                done_decoding(&rc);
                deleteqsmodel(&mx);
                deleteqsmodel(&my);
                deleteqsmodel(&mz);
                (void)totalRingVerts;
                roff = sl3.size();
            } else {
                for (int32_t i = 0; i < numRings; ++i) {
                int32_t n = 0;
                if (compactU16) {
                    if (roff >= sl3.size()) {
                        throw std::runtime_error("[PCLTTMStrict decoder] truncated ring.size");
                    }
                    n = static_cast<int32_t>(sl3[roff++]);
                } else {
                    n = readI32(sl3, roff, "ring.size");
                }
                if (n < 0 || n > 16) {
                    throw std::runtime_error("[PCLTTMStrict decoder] invalid ring size");
                }
                std::vector<std::array<int, 3>> ring;
                ring.reserve(static_cast<size_t>(n));
                for (int32_t j = 0; j < n; ++j) {
                    std::array<int, 3> q{};
                    if (compactU16) {
                        for (int axis = 0; axis < 3; ++axis) {
                            if (roff + sizeof(uint16_t) > sl3.size()) {
                                throw std::runtime_error("[PCLTTMStrict decoder] truncated ring.q");
                            }
                            uint16_t u = 0;
                            std::memcpy(&u, sl3.data() + roff, sizeof(uint16_t));
                            roff += sizeof(uint16_t);
                            q[static_cast<size_t>(axis)] = static_cast<int>(u);
                        }
                    } else {
                        q[0] = readI32(sl3, roff, "ring.x");
                        q[1] = readI32(sl3, roff, "ring.y");
                        q[2] = readI32(sl3, roff, "ring.z");
                    }
                    ring.push_back(q);
                }
                decodedRingQs_.push_back(std::move(ring));
                }
            }
        }
    }
    if (pmsf::paperBlobStreamCountPCLTTMStrictHasExtra(hdr.num_streams)) {
        auto sl4 = rd.getStream(0, kPaperBlobStreamPCLTTMStrictExtra);
        if (sl4.size() >= 4) {
            size_t soff = 0;
            const int32_t nOrMarker = readI32(sl4, soff, "symbolInput.count");
            if (nOrMarker == -1) {
                const int32_t n = readI32(sl4, soff, "vInput.count");
                if (n < 0) {
                    throw std::runtime_error("[PCLTTMStrict decoder] negative V input count");
                }
                decodedVInputQs_.reserve(static_cast<size_t>(n));
                for (int32_t i = 0; i < n; ++i) {
                    std::array<int, 6> q{};
                    for (int j = 0; j < 6; ++j) {
                        q[static_cast<size_t>(j)] =
                            static_cast<int>(readU16(sl4, soff, "vInput.q"));
                    }
                    decodedVInputQs_.push_back(q);
                }
            } else if (nOrMarker == -3) {
                const int32_t n = readI32(sl4, soff, "faceRank.count");
                if (n < 0) {
                    throw std::runtime_error("[PCLTTMStrict decoder] negative face rank count");
                }
                decodedFaceRanks_.reserve(decodedFaceRanks_.size() + static_cast<size_t>(n));
                for (int32_t i = 0; i < n; ++i) {
                    decodedFaceRanks_.push_back(readI32(sl4, soff, "faceRank.value"));
                }
            } else if (nOrMarker == -8) {
                const int32_t n = readI32(sl4, soff, "faceRank.varint.count");
                if (n < 0) {
                    throw std::runtime_error("[PCLTTMStrict decoder] negative face rank count");
                }
                decodedFaceRanks_.reserve(decodedFaceRanks_.size() + static_cast<size_t>(n));
                int32_t prev = 0;
                for (int32_t i = 0; i < n; ++i) {
                    prev += readZigZagVarI32(sl4, soff, "faceRank.varint.delta");
                    decodedFaceRanks_.push_back(prev);
                }
            } else if (nOrMarker == -9) {
                const int32_t n = readI32(sl4, soff, "faceRank.group.count");
                if (n < 0) {
                    throw std::runtime_error("[PCLTTMStrict decoder] negative face rank count");
                }
                decodedFaceRanks_.reserve(decodedFaceRanks_.size() + static_cast<size_t>(n));
                const size_t before = decodedFaceRanks_.size();
                int32_t prevFirst = 0;
                for (const RDPatchSymbol& sym : decodedSymbols_) {
                    if (sym.kind == RDPatchSymbolKind::NULL_PATCH) continue;
                    if (decodedFaceRanks_.size() - before >= static_cast<size_t>(n)) break;
                    const int groupN = std::max(1, symbolValence(sym.kind) - 2);
                    int32_t prev = prevFirst + readZigZagVarI32(sl4, soff, "faceRank.group.first");
                    decodedFaceRanks_.push_back(prev);
                    prevFirst = prev;
                    for (int k = 1; k < groupN &&
                         decodedFaceRanks_.size() - before < static_cast<size_t>(n); ++k) {
                        prev += readZigZagVarI32(sl4, soff, "faceRank.group.delta");
                        decodedFaceRanks_.push_back(prev);
                    }
                }
                if (decodedFaceRanks_.size() - before != static_cast<size_t>(n)) {
                    throw std::runtime_error("[PCLTTMStrict decoder] face rank group count mismatch");
                }
            } else if (nOrMarker == -10) {
                appendGroupBitplaneRanks(sl4, soff, "faceRank.group.count");
            } else if (nOrMarker == -13) {
                appendAptmLocatorV1Ranks(sl4, soff, "aptm.locator.v1.extra");
            } else {
                if (nOrMarker < 0) {
                    throw std::runtime_error("[PCLTTMStrict decoder] negative symbol input count");
                }
                decodedSymbolInputQs_.reserve(static_cast<size_t>(nOrMarker));
                for (int32_t i = 0; i < nOrMarker; ++i) {
                    std::array<int, 6> q{};
                    for (int j = 0; j < 6; ++j) {
                        q[static_cast<size_t>(j)] = readI32(sl4, soff, "symbolInput.q");
                    }
                    decodedSymbolInputQs_.push_back(q);
                }
            }
        }
    }

    const size_t blobSize = hdr.headerByteSize() +
                            (hdr.stream_offsets.empty() ? 0 : hdr.stream_offsets.back());
    dataOffset += blobSize;

    if (const char* e = std::getenv("PMSF_BPV_DIAG"); e && e[0] == '1') {
        std::cerr << "[PCLTTMStrict decoder] packed_syms=" << decodedSymbols_.size()
                  << " removed=" << numRemoved
                  << " residuals=" << decodedResidualsX_.size()
                  << " inputOffsets=" << decodedInputOffsets_.size()
                  << " rings=" << decodedRingQs_.size()
                  << " reconRingIds=" << decodedReconRingIds_.size()
                  << " reconOriginalIds=" << decodedReconOriginalIds_.size()
                  << " symbolInputs=" << decodedSymbolInputQs_.size()
                  << " vInputs=" << decodedVInputQs_.size()
                  << " nullBefore=" << numNullBefore
                  << " nullAfter=" << numNullAfter
                  << " A_syms=" << numA
                  << " B_syms=" << numB
                  << " layers=" << numLayers
                  << "\n";
    }
}

void PCLTTMStrictSymbolReadOperator::collectFromDecoded(
    MCGAL::Halfedge* seed,
    std::vector<MCGAL::Vertex*>& vertices,
    std::vector<MCGAL::Halfedge*>& halfedge,
    std::vector<MCGAL::Facet*>& facets) {
    (void)seed;
    vertices.clear();
    halfedge.clear();
    facets.clear();
    if (!mesh_ || decodedSymbols_.empty()) return;

    if (!envEnabled("PMSF_PCLTTM_STRICT_LEGACY_FACE_QUEUE")) {
        const bool autoSideInfoReplay =
            !envEnabled("PMSF_PCLTTM_DISABLE_AUTO_REPLAY");
        const bool forceRankReplay = envEnabled("PMSF_PCLTTM_USE_RANK_REPLAY");
        const bool forceRingReplay = envEnabled("PMSF_PCLTTM_USE_RING_REPLAY");
        const bool hasRankSideInfo = !decodedFaceRanks_.empty();
        const bool hasRingSideInfo = !decodedRingQs_.empty();
        const bool hasSparseRingTail =
            decodedRingStartIndex_ > 0 && hasRingSideInfo && !hasRankSideInfo &&
            decodedRingQs_.size() < decodedResidualsX_.size();
        const bool hasHybridReconIds =
            decodedRingStartIndex_ > 0 && !decodedReconRingIds_.empty();
        if (envEnabled("PMSF_PCLTTM_USE_POINT_REPLAY")) {
            if (!strictUsesAbsoluteGeometry()) {
                throw std::runtime_error(
                    "[PCLTTMStrict point replay] requires PMSF_PCLTTM_GEOM_MODE=absolute");
            }
            int added = 0;
            int failed = 0;
            for (size_t si = 0, ridx = 0; si < decodedSymbols_.size(); ++si) {
                const RDPatchSymbol& s = decodedSymbols_[si];
                if (s.kind == RDPatchSymbolKind::NULL_PATCH) continue;
                if (ridx >= decodedResidualsX_.size() ||
                    ridx >= decodedResidualsY_.size() ||
                    ridx >= decodedResidualsZ_.size()) {
                    break;
                }
                const int valence = symbolValence(s.kind);
                MCGAL::PointInt q(decodedResidualsX_[ridx],
                                  decodedResidualsY_[ridx],
                                  decodedResidualsZ_[ridx]);
                MCGAL::Facet* tri = livingTriangleContainingQ(mesh_, q, nullptr);
                MCGAL::Facet* f = nullptr;
                std::vector<MCGAL::Vertex*> boundary;
                bool merged = false;
                if (tri && valence == 3) {
                    f = tri;
                    boundary = orderedFaceVertices(f);
                } else if (tri) {
                    for (MCGAL::Halfedge* g : boundedFaceHalfedges(tri)) {
                        std::vector<MCGAL::Vertex*> candBoundary;
                        bool candMerged = false;
                        MCGAL::PointInt target = q;
                        MCGAL::Facet* cand = discoverAndMergePatchFromGate(
                            g, valence, nullptr, candBoundary, candMerged,
                            -1, nullptr, mesh_, &target);
                        if (cand && !cand->isRemoved() && faceDegree(cand) == valence) {
                            f = cand;
                            boundary = std::move(candBoundary);
                            merged = candMerged;
                            break;
                        }
                    }
                }
                (void)merged;
                if (!f || f->isRemoved() || faceDegree(f) != valence) {
                    ++failed;
                    break;
                }
                MCGAL::Halfedge* insertGate = findLiveFaceGate(f);
                if (!insertGate) {
                    ++failed;
                    break;
                }
                MCGAL::Halfedge* newH =
                    mesh_->create_center_vertex_without_init(insertGate, mesh_->intPosToFloat(q));
                if (!newH) {
                    ++failed;
                    break;
                }
                ++added;
                ++ridx;
            }
            if (const char* e = std::getenv("PMSF_BPV_DIAG"); e && e[0] == '1') {
                std::cerr << "[PCLTTMStrict point replay] added=" << added
                          << " failed=" << failed
                          << " symbols=" << decodedSymbols_.size()
                          << " V_out=" << mesh_->vertices().size()
                          << "\n";
            }
            return;
        }
        if (forceRankReplay && hasRankSideInfo &&
            (hasRingSideInfo || hasHybridReconIds) &&
            decodedRingStartIndex_ > 0) {
            const int rankPrefix = decodedRingStartIndex_;
            const bool useReconHybrid = hasHybridReconIds && !hasRingSideInfo;
            std::vector<RDPatchSymbol> prefixSymbols;
            prefixSymbols.reserve(static_cast<size_t>(rankPrefix));
            int prefixV = 0;
            for (const RDPatchSymbol& s : decodedSymbols_) {
                if (s.kind != RDPatchSymbolKind::NULL_PATCH && prefixV >= rankPrefix) break;
                prefixSymbols.push_back(s);
                if (s.kind != RDPatchSymbolKind::NULL_PATCH) ++prefixV;
            }
            auto sliceInts = [&](const std::vector<int>& src, int count) {
                const size_t n = std::min(src.size(), static_cast<size_t>(std::max(0, count)));
                return std::vector<int>(src.begin(), src.begin() + n);
            };
            std::vector<int> firstGateQ;
            firstGateQ.reserve(6);
            if (!layerSeedQs_.empty()) {
                for (int v : layerSeedQs_.front()) firstGateQ.push_back(v);
            }
            mesh_->resetState();
            mesh_->invalidateAllVertexDegrees();
            for (MCGAL::Vertex* v : mesh_->vertices()) {
                if (v && !v->isRemoved()) v->clearTag();
            }
            const std::vector<int> prefixX = sliceInts(decodedResidualsX_, rankPrefix);
            const std::vector<int> prefixY = sliceInts(decodedResidualsY_, rankPrefix);
            const std::vector<int> prefixZ = sliceInts(decodedResidualsZ_, rankPrefix);
            const std::vector<int> emptyAbsQ;
            const std::vector<int>& absQX =
                strictUsesAbsoluteGeometry() ? prefixX : emptyAbsQ;
            const std::vector<int>& absQY =
                strictUsesAbsoluteGeometry() ? prefixY : emptyAbsQ;
            const std::vector<int>& absQZ =
                strictUsesAbsoluteGeometry() ? prefixZ : emptyAbsQ;
            RDPatchDecoderResult rr = runRDPatchDecoderConquest(
                mesh_,
                prefixSymbols,
                prefixX,
                prefixY,
                prefixZ,
                decodedFaceRanks_,
                /*ringSizes=*/{},
                /*ringQX=*/{},
                /*ringQY=*/{},
                /*ringQZ=*/{},
                absQX,
                absQY,
                absQZ,
                firstGateQ,
                /*inputOffsets=*/{},
                /*symbolInputQs=*/{},
                /*forcePMSFBoundaryGates=*/true,
                /*forceStrictRDConquest=*/true);

            QuantVertexIndex liveVertexIndex = buildLiveVertexIndex(mesh_);
            std::vector<MCGAL::Vertex*> idToVertex;
            if (useReconHybrid) {
                idToVertex.reserve(mesh_->vertices().size() + decodedResidualsX_.size());
                for (MCGAL::Vertex* v : mesh_->vertices()) {
                    if (v && !v->isRemoved()) idToVertex.push_back(v);
                }
            }
            auto vertexForQ = [&](const std::array<int, 3>& q) -> MCGAL::Vertex* {
                if (MCGAL::Vertex* indexed =
                        findLiveVertexByQIndexed(liveVertexIndex, q)) {
                    return indexed;
                }
                for (MCGAL::Vertex* v : mesh_->vertices()) {
                    if (!v || v->isRemoved()) continue;
                    MCGAL::PointInt p = mesh_->floatPosToInt(v->point());
                    if (std::abs(p[0] - q[0]) <= 1 &&
                        std::abs(p[1] - q[1]) <= 1 &&
                        std::abs(p[2] - q[2]) <= 1) {
                        return v;
                    }
                }
                return nullptr;
            };
            auto ringGateForOrder =
                [&](MCGAL::Facet* f, const std::vector<MCGAL::Vertex*>& ring) {
                    if (!f || f->isRemoved() || ring.size() < 3) {
                        return static_cast<MCGAL::Halfedge*>(nullptr);
                    }
                    const int n = static_cast<int>(ring.size());
                    for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
                        if (!h || h->isRemoved()) continue;
                        for (int i = 0; i < n; ++i) {
                            MCGAL::Vertex* a = ring[static_cast<size_t>(i)];
                            MCGAL::Vertex* b = ring[static_cast<size_t>((i + 1) % n)];
                            if (h->vertex() == a && h->end_vertex() == b) return h;
                        }
                    }
                    return static_cast<MCGAL::Halfedge*>(nullptr);
                };
            auto boundaryMatchesVertices =
                [&](const PatchCandidate& cand,
                    const std::vector<MCGAL::Vertex*>& ring) {
                    const int n = static_cast<int>(ring.size());
                    if (static_cast<int>(cand.boundary.size()) != n) return false;
                    for (int rev = 0; rev < 2; ++rev) {
                        for (int rot = 0; rot < n; ++rot) {
                            bool ok = true;
                            for (int i = 0; i < n; ++i) {
                                int idx = rev ? (rot - i) : (rot + i);
                                idx %= n;
                                if (idx < 0) idx += n;
                                if (cand.boundary[static_cast<size_t>(idx)] !=
                                    ring[static_cast<size_t>(i)]) {
                                    ok = false;
                                    break;
                                }
                            }
                            if (ok) return true;
                        }
                    }
                    return false;
                };
            auto locateRingPatch =
                [&](const std::vector<MCGAL::Vertex*>& ring,
                    const std::vector<std::array<int, 3>>* ringQ,
                    int valence,
                    MCGAL::Facet*& polyFace) -> bool {
                    polyFace = nullptr;
                    if (static_cast<int>(ring.size()) != valence || valence < 3) return false;
                    std::set<MCGAL::Vertex*> ringSet(ring.begin(), ring.end());
                    if (static_cast<int>(ringSet.size()) != valence) return false;
                    auto faceWithinRing = [&](MCGAL::Facet* f) {
                        if (!f || f->isRemoved()) return false;
                        for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
                            if (!h || !h->vertex() || !ringSet.count(h->vertex())) {
                                return false;
                            }
                        }
                        return true;
                    };
                    std::vector<MCGAL::Facet*> pool;
                    std::unordered_set<MCGAL::Facet*> seen;
                    for (MCGAL::Vertex* v : ring) {
                        for (MCGAL::Halfedge* hv : boundedVertexHalfedges(v)) {
                            MCGAL::Facet* f = hv ? hv->face() : nullptr;
                            if (!f || f->isRemoved() || !seen.insert(f).second) continue;
                            if (!faceWithinRing(f)) continue;
                            if (faceDegree(f) == valence) {
                                polyFace = f;
                                if (MCGAL::Halfedge* g = ringGateForOrder(polyFace, ring)) {
                                    polyFace->proxyHalfedge(g);
                                }
                                return true;
                            }
                            if (faceDegree(f) == 3) pool.push_back(f);
                        }
                    }
                    if (static_cast<int>(pool.size()) < valence - 2) {
                        for (MCGAL::Facet* f : mesh_->faces()) {
                            if (!f || f->isRemoved() || !seen.insert(f).second) continue;
                            if (!faceWithinRing(f)) continue;
                            if (faceDegree(f) == valence) {
                                polyFace = f;
                                if (MCGAL::Halfedge* g = ringGateForOrder(polyFace, ring)) {
                                    polyFace->proxyHalfedge(g);
                                }
                                return true;
                            }
                            if (faceDegree(f) == 3) pool.push_back(f);
                        }
                    }
                    if (static_cast<int>(pool.size()) < valence - 2) return false;
                    std::vector<MCGAL::Facet*> chosen;
                    PatchCandidate best;
                    bool found = false;
                    std::function<bool(int)> dfs = [&](int nextIdx) {
                        if (static_cast<int>(chosen.size()) == valence - 2) {
                            PatchCandidate cand;
                            if (validatePatchCandidate(chosen, valence, nullptr, nullptr,
                                                       nullptr, cand) &&
                                (ringQ ? candidateBoundaryMatchesRingQ(mesh_, cand, *ringQ)
                                       : boundaryMatchesVertices(cand, ring))) {
                                best = cand;
                                found = true;
                                return true;
                            }
                            return false;
                        }
                        for (int i = nextIdx; i < static_cast<int>(pool.size()); ++i) {
                            chosen.push_back(pool[static_cast<size_t>(i)]);
                            if (dfs(i + 1)) return true;
                            chosen.pop_back();
                        }
                        return false;
                    };
                    if (!dfs(0)) return false;
                    polyFace = mergePatchToPolygon(best, valence);
                    if (!polyFace || polyFace->isRemoved() ||
                        faceDegree(polyFace) != valence) {
                        return false;
                    }
                    if (MCGAL::Halfedge* g = ringGateForOrder(polyFace, ring)) {
                        polyFace->proxyHalfedge(g);
                    }
                    return true;
                };

            int addedRing = 0;
            int failed = 0;
            int vIndex = 0;
            size_t ringIdx = 0;
            for (const RDPatchSymbol& s : decodedSymbols_) {
                if (s.kind == RDPatchSymbolKind::NULL_PATCH) continue;
                const int curV = vIndex++;
                if (curV < rankPrefix) continue;
                const int valence = symbolValence(s.kind);
                const size_t ringCount =
                    useReconHybrid ? decodedReconRingIds_.size()
                                   : decodedRingQs_.size();
                if (valence < 3 || ringIdx >= ringCount ||
                    curV >= static_cast<int>(decodedResidualsX_.size())) {
                    ++failed;
                    break;
                }
                std::vector<MCGAL::Vertex*> ring;
                const std::vector<std::array<int, 3>>* ringQ = nullptr;
                int missingRingVertex = -1;
                int missingReconId = -1;
                if (useReconHybrid) {
                    const auto& ringIds = decodedReconRingIds_[ringIdx++];
                    ring.reserve(ringIds.size());
                    for (int id : ringIds) {
                        if (id < 0 || static_cast<size_t>(id) >= idToVertex.size()) {
                            missingReconId = id;
                            ring.clear();
                            break;
                        }
                        MCGAL::Vertex* v = idToVertex[static_cast<size_t>(id)];
                        if (!v || v->isRemoved()) {
                            missingReconId = id;
                            ring.clear();
                            break;
                        }
                        ring.push_back(v);
                    }
                } else {
                    ringQ = &decodedRingQs_[ringIdx++];
                    ring.reserve(ringQ->size());
                    for (const auto& q : *ringQ) {
                        MCGAL::Vertex* v = vertexForQ(q);
                        if (!v) {
                            missingRingVertex = static_cast<int>(ring.size());
                            ring.clear();
                            break;
                        }
                        ring.push_back(v);
                    }
                }
                MCGAL::Facet* polyFace = nullptr;
                if (!locateRingPatch(ring, ringQ, valence, polyFace)) {
                    if (const char* e = std::getenv("PMSF_BPV_DIAG"); e && e[0] == '1') {
                        std::cerr << "[PCLTTMStrict hybrid fail] curV=" << curV
                                  << " valence=" << valence
                                  << " mode=" << (useReconHybrid ? "recon" : "q")
                                  << " foundRing=" << ring.size()
                                  << " missingRingVertex=" << missingRingVertex
                                  << " missingReconId=" << missingReconId
                                  << "\n";
                    }
                    ++failed;
                    break;
                }
                if (!useReconHybrid && ringQ) {
                    if (MCGAL::Halfedge* ringGate =
                            findRingOrderedHalfedge(mesh_, polyFace, *ringQ)) {
                        polyFace->proxyHalfedge(ringGate);
                    }
                }
                MCGAL::PointInt baryQ =
                    strictUsesAbsoluteGeometry() ? MCGAL::PointInt(0, 0, 0)
                                                 : (useReconHybrid
                                                        ? MCGAL::rdBarycenterQuantized(polyFace, mesh_)
                                                        : barycenterFromRingQ(*ringQ));
                MCGAL::PointInt vQ =
                    strictUsesAbsoluteGeometry()
                        ? MCGAL::PointInt(decodedResidualsX_[static_cast<size_t>(curV)],
                                          decodedResidualsY_[static_cast<size_t>(curV)],
                                          decodedResidualsZ_[static_cast<size_t>(curV)])
                        : MCGAL::PointInt(
                              baryQ[0] + decodedResidualsX_[static_cast<size_t>(curV)],
                              baryQ[1] + decodedResidualsY_[static_cast<size_t>(curV)],
                              baryQ[2] + decodedResidualsZ_[static_cast<size_t>(curV)]);
                MCGAL::Halfedge* insertGate = findLiveFaceGate(polyFace);
                MCGAL::Halfedge* newH =
                    insertGate ? mesh_->create_center_vertex_without_init(
                                     insertGate, mesh_->intPosToFloat(vQ))
                               : nullptr;
                MCGAL::Vertex* insertedCenter = newH ? newH->vertex() : nullptr;
                if (!insertedCenter || insertedCenter->isRemoved()) {
                    ++failed;
                    break;
                }
                if (useReconHybrid) {
                    idToVertex.push_back(insertedCenter);
                } else {
                    indexLiveVertex(mesh_, liveVertexIndex, insertedCenter);
                }
                ++addedRing;
            }
            if (const char* e = std::getenv("PMSF_BPV_DIAG"); e && e[0] == '1') {
                std::cerr << "[PCLTTMStrict hybrid rank/ring replay] rank_added="
                          << rr.numAddedVertices
                          << " ring_added=" << addedRing
                          << " failed=" << failed
                          << " rank_prefix=" << rankPrefix
                          << " rings=" << (useReconHybrid ? decodedReconRingIds_.size()
                                                          : decodedRingQs_.size())
                          << " mode=" << (useReconHybrid ? "recon" : "q")
                          << " V_out=" << mesh_->vertices().size()
                          << "\n";
            }
            return;
        }
        if ((envEnabled("PMSF_PCLTTM_USE_RECON_RING_REPLAY") ||
             (autoSideInfoReplay && !decodedReconRingIds_.empty())) &&
            !decodedReconRingIds_.empty()) {
            std::vector<MCGAL::Vertex*> idToVertex;
            idToVertex.reserve(mesh_->vertices().size() + decodedResidualsX_.size());
            std::unordered_map<MCGAL::Vertex*, int> vertexToReconId;
            vertexToReconId.reserve(mesh_->vertices().size() + decodedResidualsX_.size());
            for (MCGAL::Vertex* v : mesh_->vertices()) {
                if (v && !v->isRemoved()) {
                    vertexToReconId[v] = static_cast<int>(idToVertex.size());
                    idToVertex.push_back(v);
                }
            }
            auto dumpIds = [](const std::set<int>& ids) {
                std::string out;
                for (int id : ids) {
                    if (!out.empty()) out += ",";
                    out += std::to_string(id);
                }
                return out;
            };
            const bool verifyReconRing =
                envEnabled("PMSF_PCLTTM_VERIFY_RECON_RING") ||
                envEnabled("PMSF_PCLTTM_STOP_ON_RECON_DIVERGENCE");
            const bool stopOnReconDivergence =
                envEnabled("PMSF_PCLTTM_STOP_ON_RECON_DIVERGENCE");
            auto actualNeighborReconIds = [&](
                MCGAL::Vertex* center) {
                std::set<int> ids;
                if (!center || center->isRemoved()) return ids;
                for (MCGAL::Facet* f : mesh_->faces()) {
                    if (!f || f->isRemoved() || !faceContainsVertex(f, center)) continue;
                    for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
                        if (!h || h->isRemoved()) continue;
                        MCGAL::Vertex* vs[2] = {h->vertex(), h->end_vertex()};
                        for (MCGAL::Vertex* v : vs) {
                            if (!v || v == center || v->isRemoved()) continue;
                            auto it = vertexToReconId.find(v);
                            if (it != vertexToReconId.end()) ids.insert(it->second);
                        }
                    }
                }
                return ids;
            };
            auto canonicalEdge = [](int a, int b) {
                if (a > b) std::swap(a, b);
                return std::pair<int, int>(a, b);
            };
            auto dumpEdges = [](const std::set<std::pair<int, int>>& edges) {
                std::string out;
                for (const auto& e : edges) {
                    if (!out.empty()) out += ",";
                    out += std::to_string(e.first);
                    out += "-";
                    out += std::to_string(e.second);
                }
                return out;
            };
            auto expectedRingEdges = [&](const std::vector<int>& ids) {
                std::set<std::pair<int, int>> edges;
                if (ids.size() < 3) return edges;
                for (size_t i = 0; i < ids.size(); ++i) {
                    edges.insert(canonicalEdge(ids[i], ids[(i + 1) % ids.size()]));
                }
                return edges;
            };
            auto actualCenterFanEdges = [&](
                MCGAL::Vertex* center) {
                std::set<std::pair<int, int>> edges;
                if (!center || center->isRemoved()) return edges;
                for (MCGAL::Facet* f : mesh_->faces()) {
                    if (!f || f->isRemoved() || !faceContainsVertex(f, center)) continue;
                    std::vector<int> ids;
                    for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
                        MCGAL::Vertex* v = h ? h->vertex() : nullptr;
                        if (!v || v == center || v->isRemoved()) continue;
                        auto it = vertexToReconId.find(v);
                        if (it != vertexToReconId.end()) ids.push_back(it->second);
                    }
                    std::sort(ids.begin(), ids.end());
                    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
                    if (ids.size() == 2) edges.insert(canonicalEdge(ids[0], ids[1]));
                }
                return edges;
            };
            auto faceReconIds = [&](MCGAL::Facet* f) {
                std::vector<int> ids;
                if (!f || f->isRemoved()) return ids;
                for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
                    MCGAL::Vertex* v = h ? h->vertex() : nullptr;
                    auto it = v ? vertexToReconId.find(v) : vertexToReconId.end();
                    ids.push_back(it == vertexToReconId.end() ? -1 : it->second);
                }
                return ids;
            };
            auto edgeIndexKey = [](int a, int b) -> std::uint64_t {
                if (a > b) std::swap(a, b);
                return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(a)) << 32u) |
                       static_cast<std::uint32_t>(b);
            };
            std::unordered_map<std::uint64_t, std::vector<MCGAL::Facet*>> edgeFaceIndex;
            edgeFaceIndex.reserve(mesh_->faces().size() * 3 + decodedResidualsX_.size() * 4);
            auto indexFace = [&](MCGAL::Facet* f) {
                if (!f || f->isRemoved()) return;
                std::vector<int> ids = faceReconIds(f);
                const int n = static_cast<int>(ids.size());
                if (n < 3) return;
                for (int i = 0; i < n; ++i) {
                    const int a = ids[static_cast<size_t>(i)];
                    const int b = ids[static_cast<size_t>((i + 1) % n)];
                    if (a < 0 || b < 0 || a == b) continue;
                    edgeFaceIndex[edgeIndexKey(a, b)].push_back(f);
                }
            };
            for (MCGAL::Facet* f : mesh_->faces()) {
                indexFace(f);
            }
            auto dumpRingNeighborhood =
                [&](int opIndex, int valence, const std::vector<int>& ids) {
                    if (!envEnabled("PMSF_PCLTTM_TRACE_RECON_RING")) return;
                    std::set<int> ringSet(ids.begin(), ids.end());
                    std::fprintf(stderr,
                                 "[PCLTTM recon ring neighborhood] op=%d valence=%d\n",
                                 opIndex,
                                 valence);
                    for (size_t i = 0; i < ids.size(); ++i) {
                        const int a = ids[i];
                        const int b = ids[(i + 1) % ids.size()];
                        int edgeFaces = 0;
                        for (MCGAL::Facet* f : mesh_->faces()) {
                            if (!f || f->isRemoved()) continue;
                            std::vector<int> fids = faceReconIds(f);
                            bool hasA = false;
                            bool hasB = false;
                            for (int id : fids) {
                                hasA = hasA || id == a;
                                hasB = hasB || id == b;
                            }
                            if (hasA && hasB) ++edgeFaces;
                        }
                        std::fprintf(stderr,
                                     "  edge %d-%d live_faces=%d\n",
                                     a,
                                     b,
                                     edgeFaces);
                    }
                    int printed = 0;
                    for (MCGAL::Facet* f : mesh_->faces()) {
                        if (!f || f->isRemoved()) continue;
                        std::vector<int> fids = faceReconIds(f);
                        int hits = 0;
                        for (int id : fids) {
                            if (ringSet.count(id)) ++hits;
                        }
                        if (hits < 2) continue;
                        std::fprintf(stderr, "  face deg=%zu hits=%d ids=",
                                     fids.size(), hits);
                        for (size_t ii = 0; ii < fids.size(); ++ii) {
                            std::fprintf(stderr, "%s%d", ii ? "," : "", fids[ii]);
                        }
                        std::fprintf(stderr, "\n");
                        if (++printed >= 40) {
                            std::fprintf(stderr, "  ... truncated neighborhood\n");
                            break;
                        }
                    }
                };
            auto ringGateForOrder =
                [&](MCGAL::Facet* f, const std::vector<MCGAL::Vertex*>& ring) {
                    if (!f || f->isRemoved() || ring.size() < 3) return static_cast<MCGAL::Halfedge*>(nullptr);
                    const int n = static_cast<int>(ring.size());
                    for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
                        if (!h || h->isRemoved()) continue;
                        for (int i = 0; i < n; ++i) {
                            MCGAL::Vertex* a = ring[static_cast<size_t>(i)];
                            MCGAL::Vertex* b = ring[static_cast<size_t>((i + 1) % n)];
                            if (h->vertex() == a && h->end_vertex() == b) return h;
                        }
                    }
                    return static_cast<MCGAL::Halfedge*>(nullptr);
                };
            auto boundaryMatchesIds =
                [&](const PatchCandidate& cand, const std::vector<MCGAL::Vertex*>& ring) {
                    const int n = static_cast<int>(ring.size());
                    if (static_cast<int>(cand.boundary.size()) != n) return false;
                    for (int rev = 0; rev < 2; ++rev) {
                        for (int rot = 0; rot < n; ++rot) {
                            bool ok = true;
                            for (int i = 0; i < n; ++i) {
                                int idx = rev ? (rot - i) : (rot + i);
                                idx %= n;
                                if (idx < 0) idx += n;
                                if (cand.boundary[static_cast<size_t>(idx)] !=
                                    ring[static_cast<size_t>(i)]) {
                                    ok = false;
                                    break;
                                }
                            }
                            if (ok) return true;
                        }
                    }
                    return false;
                };
            auto locatePatchByReconIds =
                [&](const std::vector<int>& ids,
                    int valence,
                    int opIndex,
                    std::vector<MCGAL::Vertex*>& ring,
                    MCGAL::Facet*& polyFace) {
                    ring.clear();
                    polyFace = nullptr;
                    auto logFail = [&](const char* reason, int extra = -1) {
                        if (!envEnabled("PMSF_PCLTTM_TRACE_RECON_RING")) return;
                        std::fprintf(stderr,
                                     "[PCLTTM recon ring fail] op=%d reason=%s valence=%d ids=",
                                     opIndex, reason, valence);
                        for (size_t ii = 0; ii < ids.size(); ++ii) {
                            std::fprintf(stderr, "%s%d", ii ? "," : "", ids[ii]);
                        }
                        std::fprintf(stderr, " idToVertex=%zu extra=%d\n",
                                     idToVertex.size(), extra);
                    };
                    if (static_cast<int>(ids.size()) != valence || valence < 3) {
                        logFail("bad_size");
                        return false;
                    }
                    ring.reserve(ids.size());
                    std::set<MCGAL::Vertex*> ringSet;
                    for (int id : ids) {
                        if (id < 0 || static_cast<size_t>(id) >= idToVertex.size()) {
                            logFail("missing_id", id);
                            return false;
                        }
                        MCGAL::Vertex* v = idToVertex[static_cast<size_t>(id)];
                        if (!v || v->isRemoved()) {
                            logFail("removed_id", id);
                            return false;
                        }
                        ring.push_back(v);
                        ringSet.insert(v);
                    }
                    if (static_cast<int>(ringSet.size()) != valence) {
                        logFail("duplicate_ring");
                        return false;
                    }
                    auto faceWithinRing = [&](MCGAL::Facet* f) {
                        if (!f || f->isRemoved()) return false;
                        for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
                            if (!h || !h->vertex() || !ringSet.count(h->vertex())) {
                                return false;
                            }
                        }
                        return true;
                    };
                    std::vector<MCGAL::Facet*> pool;
                    std::vector<MCGAL::Facet*> allInsideFaces;
                    std::unordered_set<MCGAL::Facet*> seen;
                    auto rememberInsideFace = [&](MCGAL::Facet* f) {
                        if (!f || f->isRemoved()) return;
                        allInsideFaces.push_back(f);
                        if (faceDegree(f) == 3) pool.push_back(f);
                    };
                    auto considerInsideFace = [&](MCGAL::Facet* f) -> bool {
                        if (!f || f->isRemoved() || !seen.insert(f).second) return false;
                        if (!faceWithinRing(f)) return false;
                        if (faceDegree(f) == valence) {
                            polyFace = f;
                            if (MCGAL::Halfedge* g = ringGateForOrder(polyFace, ring)) {
                                polyFace->proxyHalfedge(g);
                            }
                            return true;
                        }
                        rememberInsideFace(f);
                        return false;
                    };
                    auto mergeInsideFacesToPolygon = [&]() -> bool {
                        if (allInsideFaces.size() < 2) return false;
                        auto refreshInside = [&]() {
                            std::vector<MCGAL::Facet*> cur;
                            for (MCGAL::Facet* f : mesh_->faces()) {
                                if (f && !f->isRemoved() && faceWithinRing(f)) {
                                    cur.push_back(f);
                                }
                            }
                            return cur;
                        };
                        std::vector<MCGAL::Facet*> cur = refreshInside();
                        for (int guard = 0; guard < valence * valence && cur.size() > 1; ++guard) {
                            std::set<MCGAL::Facet*> curSet(cur.begin(), cur.end());
                            MCGAL::Halfedge* joinEdge = nullptr;
                            for (MCGAL::Facet* f : cur) {
                                for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
                                    MCGAL::Facet* of = h && h->opposite() ? h->opposite()->face() : nullptr;
                                    if (of && curSet.count(of)) {
                                        joinEdge = h;
                                        break;
                                    }
                                }
                                if (joinEdge) break;
                            }
                            if (!joinEdge) break;
                            MCGAL::Halfedge* ret = MCGAL::join_face(joinEdge);
                            polyFace = ret && ret->face() ? ret->face() : nullptr;
                            cur = refreshInside();
                            for (MCGAL::Facet* f : cur) {
                                if (faceDegree(f) == valence) {
                                    polyFace = f;
                                    if (MCGAL::Halfedge* g = ringGateForOrder(polyFace, ring)) {
                                        polyFace->proxyHalfedge(g);
                                    }
                                    return true;
                                }
                            }
                        }
                        if (polyFace && !polyFace->isRemoved() && faceDegree(polyFace) == valence) {
                            if (MCGAL::Halfedge* g = ringGateForOrder(polyFace, ring)) {
                                polyFace->proxyHalfedge(g);
                            }
                            return true;
                        }
                        return false;
                    };
                    for (size_t ai = 0; ai < ids.size(); ++ai) {
                        for (size_t bi = ai + 1; bi < ids.size(); ++bi) {
                            auto it = edgeFaceIndex.find(
                                edgeIndexKey(ids[ai], ids[bi]));
                            if (it == edgeFaceIndex.end()) continue;
                            for (MCGAL::Facet* f : it->second) {
                                if (considerInsideFace(f)) return true;
                            }
                        }
                    }
                    for (MCGAL::Vertex* v : ring) {
                        for (MCGAL::Halfedge* hv : boundedVertexHalfedges(v)) {
                            if (considerInsideFace(hv ? hv->face() : nullptr)) return true;
                        }
                    }
                    if (static_cast<int>(pool.size()) < valence - 2) {
                        for (MCGAL::Facet* f : mesh_->faces()) {
                            if (considerInsideFace(f)) return true;
                        }
                    }
                    if (static_cast<int>(pool.size()) < valence - 2) {
                        if (mergeInsideFacesToPolygon()) return true;
                        dumpRingNeighborhood(opIndex, valence, ids);
                        logFail("small_pool", static_cast<int>(pool.size()));
                        return false;
                    }
                    std::vector<MCGAL::Facet*> chosen;
                    PatchCandidate best;
                    bool found = false;
                    std::function<bool(int)> dfs = [&](int nextIdx) {
                        if (static_cast<int>(chosen.size()) == valence - 2) {
                            PatchCandidate cand;
                            if (validatePatchCandidate(chosen, valence, nullptr, nullptr,
                                                       nullptr, cand) &&
                                boundaryMatchesIds(cand, ring)) {
                                best = cand;
                                found = true;
                                return true;
                            }
                            return false;
                        }
                        for (int i = nextIdx; i < static_cast<int>(pool.size()); ++i) {
                            chosen.push_back(pool[static_cast<size_t>(i)]);
                            if (dfs(i + 1)) return true;
                            chosen.pop_back();
                        }
                        return false;
                    };
                    if (!dfs(0)) {
                        if (mergeInsideFacesToPolygon()) return true;
                        dumpRingNeighborhood(opIndex, valence, ids);
                        logFail("no_candidate", static_cast<int>(pool.size()));
                        return false;
                    }
                    polyFace = mergePatchToPolygon(best, valence);
                    if (!polyFace || polyFace->isRemoved() || faceDegree(polyFace) != valence) {
                        logFail("merge_failed", faceDegree(polyFace));
                        return false;
                    }
                    if (MCGAL::Halfedge* g = ringGateForOrder(polyFace, ring)) {
                        polyFace->proxyHalfedge(g);
                    }
                    return true;
                };

            int added = 0;
            int failed = 0;
            size_t ridx = 0;
            for (size_t si = 0; si < decodedSymbols_.size(); ++si) {
                const RDPatchSymbol& s = decodedSymbols_[si];
                if (s.kind == RDPatchSymbolKind::NULL_PATCH) continue;
                const int valence = symbolValence(s.kind);
                if (valence < 3 || ridx >= decodedReconRingIds_.size() ||
                    ridx >= decodedResidualsX_.size()) {
                    ++failed;
                    break;
                }
                std::vector<MCGAL::Vertex*> ring;
                MCGAL::Facet* polyFace = nullptr;
                if (!locatePatchByReconIds(decodedReconRingIds_[ridx],
                                           valence, static_cast<int>(ridx), ring, polyFace)) {
                    ++failed;
                    break;
                }
                MCGAL::PointInt baryQ =
                    strictUsesAbsoluteGeometry()
                        ? MCGAL::PointInt(0, 0, 0)
                        : MCGAL::rdBarycenterQuantized(polyFace, mesh_);
                MCGAL::PointInt vQ =
                    strictUsesAbsoluteGeometry()
                        ? MCGAL::PointInt(decodedResidualsX_[ridx],
                                          decodedResidualsY_[ridx],
                                          decodedResidualsZ_[ridx])
                        : MCGAL::PointInt(baryQ[0] + decodedResidualsX_[ridx],
                                          baryQ[1] + decodedResidualsY_[ridx],
                                          baryQ[2] + decodedResidualsZ_[ridx]);
                MCGAL::Halfedge* insertGate = findLiveFaceGate(polyFace);
                const size_t faceStart = mesh_->faces().size();
                MCGAL::Halfedge* newH =
                    insertGate ? mesh_->create_center_vertex_without_init(
                                     insertGate, mesh_->intPosToFloat(vQ))
                               : nullptr;
                MCGAL::Vertex* insertedCenter = newH ? newH->vertex() : nullptr;
                if (!insertedCenter || insertedCenter->isRemoved()) {
                    ++failed;
                    break;
                }
                const int insertedReconId = static_cast<int>(idToVertex.size());
                vertexToReconId[insertedCenter] = insertedReconId;
                idToVertex.push_back(insertedCenter);
                indexFace(polyFace);
                for (size_t fi = faceStart; fi < mesh_->faces().size(); ++fi) {
                    indexFace(mesh_->faces()[fi]);
                }
                if (verifyReconRing) {
                    std::set<int> expected(decodedReconRingIds_[ridx].begin(),
                                           decodedReconRingIds_[ridx].end());
                    std::set<int> actual = actualNeighborReconIds(insertedCenter);
                    if (expected != actual) {
                        std::fprintf(stderr,
                                     "[PCLTTM recon ring divergence] op=%zu center=%d "
                                     "expected={%s} actual={%s}\n",
                                     ridx,
                                     insertedReconId,
                                     dumpIds(expected).c_str(),
                                     dumpIds(actual).c_str());
                        if (stopOnReconDivergence) {
                            ++failed;
                            break;
                        }
                    }
                    std::set<std::pair<int, int>> expectedFan =
                        expectedRingEdges(decodedReconRingIds_[ridx]);
                    std::set<std::pair<int, int>> actualFan =
                        actualCenterFanEdges(insertedCenter);
                    if (expectedFan != actualFan) {
                        std::fprintf(stderr,
                                     "[PCLTTM recon fan divergence] op=%zu center=%d "
                                     "expected={%s} actual={%s}\n",
                                     ridx,
                                     insertedReconId,
                                     dumpEdges(expectedFan).c_str(),
                                     dumpEdges(actualFan).c_str());
                        if (stopOnReconDivergence) {
                            ++failed;
                            break;
                        }
                    }
                }
                ++added;
                ++ridx;
            }
            if (decodedReconOriginalIds_.size() == idToVertex.size() &&
                !decodedReconOriginalIds_.empty()) {
                std::vector<MCGAL::Vertex*> ordered(idToVertex.size(), nullptr);
                bool ok = true;
                for (size_t reconId = 0; reconId < decodedReconOriginalIds_.size(); ++reconId) {
                    const int originalId = decodedReconOriginalIds_[reconId];
                    if (originalId < 0 ||
                        static_cast<size_t>(originalId) >= ordered.size() ||
                        ordered[static_cast<size_t>(originalId)] != nullptr) {
                        ok = false;
                        break;
                    }
                    ordered[static_cast<size_t>(originalId)] = idToVertex[reconId];
                }
                if (ok) {
                    for (MCGAL::Vertex* v : ordered) {
                        if (!v) {
                            ok = false;
                            break;
                        }
                    }
                }
                if (ok) {
                    mesh_->vertices() = std::move(ordered);
                } else if (const char* e = std::getenv("PMSF_BPV_DIAG"); e && e[0] == '1') {
                    std::cerr << "[PCLTTMStrict recon ring replay] original-id reorder skipped\n";
                }
            }
            if (const char* e = std::getenv("PMSF_BPV_DIAG"); e && e[0] == '1') {
                std::cerr << "[PCLTTMStrict recon ring replay] added=" << added
                          << " failed=" << failed
                          << " rings=" << decodedReconRingIds_.size()
                          << " V_out=" << mesh_->vertices().size()
                          << "\n";
            }
            return;
        }
        if (envEnabled("PMSF_PCLTTM_USE_EXACT_RING_REPLAY") && hasRingSideInfo) {
            QuantVertexIndex liveVertexIndex = buildLiveVertexIndex(mesh_);
            auto exactVertexForQ = [&](const std::array<int, 3>& q) -> MCGAL::Vertex* {
                if (MCGAL::Vertex* indexed =
                        findLiveVertexByQIndexed(liveVertexIndex, q)) {
                    return indexed;
                }
                for (MCGAL::Vertex* v : mesh_->vertices()) {
                    if (!v || v->isRemoved()) continue;
                    MCGAL::PointInt p = mesh_->floatPosToInt(v->point());
                    if (std::abs(p[0] - q[0]) <= 1 &&
                        std::abs(p[1] - q[1]) <= 1 &&
                        std::abs(p[2] - q[2]) <= 1) {
                        return v;
                    }
                }
                return nullptr;
            };
            auto locateExactRingPatch =
                [&](const std::vector<MCGAL::Vertex*>& ring,
                    const std::vector<std::array<int, 3>>& ringQ,
                    int valence,
                    MCGAL::Facet*& polyFace) -> bool {
                polyFace = nullptr;
                if (static_cast<int>(ring.size()) != valence || valence < 3) return false;
                std::set<MCGAL::Vertex*> ringSet(ring.begin(), ring.end());
                if (static_cast<int>(ringSet.size()) != valence) return false;
                auto faceWithinRing = [&](MCGAL::Facet* f) {
                    if (!f || f->isRemoved()) return false;
                    for (MCGAL::Halfedge* h : boundedFaceHalfedges(f)) {
                        if (!h || !h->vertex() || !ringSet.count(h->vertex())) {
                            return false;
                        }
                    }
                    return true;
                };
                std::vector<MCGAL::Facet*> pool;
                std::unordered_set<MCGAL::Facet*> seen;
                for (MCGAL::Vertex* v : ring) {
                    for (MCGAL::Halfedge* hv : boundedVertexHalfedges(v)) {
                        MCGAL::Facet* f = hv ? hv->face() : nullptr;
                        if (!f || f->isRemoved() || !seen.insert(f).second) continue;
                        if (!faceWithinRing(f)) continue;
                        if (faceDegree(f) == valence) {
                            polyFace = f;
                            return true;
                        }
                        if (faceDegree(f) == 3) pool.push_back(f);
                    }
                }
                if (static_cast<int>(pool.size()) < valence - 2) {
                    for (MCGAL::Facet* f : mesh_->faces()) {
                        if (!f || f->isRemoved() || !seen.insert(f).second) continue;
                        if (!faceWithinRing(f)) continue;
                        if (faceDegree(f) == valence) {
                            polyFace = f;
                            return true;
                        }
                        if (faceDegree(f) == 3) pool.push_back(f);
                    }
                }
                if (static_cast<int>(pool.size()) < valence - 2) return false;
                std::vector<MCGAL::Facet*> chosen;
                PatchCandidate best;
                bool found = false;
                std::function<bool(int)> dfs = [&](int nextIdx) {
                    if (static_cast<int>(chosen.size()) == valence - 2) {
                        PatchCandidate cand;
                        if (validatePatchCandidate(chosen, valence, nullptr, nullptr,
                                                   nullptr, cand) &&
                            candidateBoundaryMatchesRingQ(mesh_, cand, ringQ)) {
                            best = cand;
                            found = true;
                            return true;
                        }
                        return false;
                    }
                    for (int i = nextIdx; i < static_cast<int>(pool.size()); ++i) {
                        chosen.push_back(pool[static_cast<size_t>(i)]);
                        if (dfs(i + 1)) return true;
                        chosen.pop_back();
                    }
                    return false;
                };
                if (!dfs(0)) return false;
                polyFace = mergePatchToPolygon(best, valence);
                return polyFace && !polyFace->isRemoved() && faceDegree(polyFace) == valence;
            };

            int added = 0;
            int failed = 0;
            size_t ridx = 0;
            for (size_t si = 0; si < decodedSymbols_.size(); ++si) {
                const RDPatchSymbol& s = decodedSymbols_[si];
                if (s.kind == RDPatchSymbolKind::NULL_PATCH) continue;
                const int valence = symbolValence(s.kind);
                if (valence < 3 || ridx >= decodedRingQs_.size() ||
                    ridx >= decodedResidualsX_.size()) {
                    ++failed;
                    break;
                }
                std::vector<MCGAL::Vertex*> ring;
                ring.reserve(decodedRingQs_[ridx].size());
                for (const auto& q : decodedRingQs_[ridx]) {
                    MCGAL::Vertex* v = exactVertexForQ(q);
                    if (!v) {
                        ring.clear();
                        break;
                    }
                    ring.push_back(v);
                }
                MCGAL::Facet* polyFace = nullptr;
                if (!locateExactRingPatch(ring, decodedRingQs_[ridx], valence, polyFace)) {
                    ++failed;
                    break;
                }
                if (MCGAL::Halfedge* ringGate =
                        findRingOrderedHalfedge(mesh_, polyFace, decodedRingQs_[ridx])) {
                    polyFace->proxyHalfedge(ringGate);
                }
                MCGAL::PointInt baryQ =
                    strictUsesAbsoluteGeometry()
                        ? MCGAL::PointInt(0, 0, 0)
                        : barycenterFromRingQ(decodedRingQs_[ridx]);
                MCGAL::PointInt vQ =
                    strictUsesAbsoluteGeometry()
                        ? MCGAL::PointInt(decodedResidualsX_[ridx],
                                          decodedResidualsY_[ridx],
                                          decodedResidualsZ_[ridx])
                        : MCGAL::PointInt(baryQ[0] + decodedResidualsX_[ridx],
                                          baryQ[1] + decodedResidualsY_[ridx],
                                          baryQ[2] + decodedResidualsZ_[ridx]);
                MCGAL::Halfedge* insertGate = findLiveFaceGate(polyFace);
                MCGAL::Halfedge* newH =
                    insertGate ? mesh_->create_center_vertex_without_init(
                                     insertGate, mesh_->intPosToFloat(vQ))
                               : nullptr;
                MCGAL::Vertex* insertedCenter = newH ? newH->vertex() : nullptr;
                if (!insertedCenter || insertedCenter->isRemoved()) {
                    ++failed;
                    break;
                }
                indexLiveVertex(mesh_, liveVertexIndex, insertedCenter);
                ++added;
                ++ridx;
            }
            if (const char* e = std::getenv("PMSF_BPV_DIAG"); e && e[0] == '1') {
                std::cerr << "[PCLTTMStrict exact ring replay] added=" << added
                          << " failed=" << failed
                          << " rings=" << decodedRingQs_.size()
                          << " V_out=" << mesh_->vertices().size()
                          << "\n";
            }
            return;
        }
        if (forceRankReplay ||
            (autoSideInfoReplay && hasRankSideInfo && !hasRingSideInfo && !forceRingReplay)) {
            std::vector<int> firstGateQ;
            firstGateQ.reserve(6);
            if (!layerSeedQs_.empty()) {
                for (int v : layerSeedQs_.front()) firstGateQ.push_back(v);
            }
            mesh_->resetState();
            mesh_->invalidateAllVertexDegrees();
            for (MCGAL::Vertex* v : mesh_->vertices()) {
                if (v && !v->isRemoved()) v->clearTag();
            }
            const std::vector<int> emptyAbsQ;
            const std::vector<int>& absQX =
                strictUsesAbsoluteGeometry() ? decodedResidualsX_ : emptyAbsQ;
            const std::vector<int>& absQY =
                strictUsesAbsoluteGeometry() ? decodedResidualsY_ : emptyAbsQ;
            const std::vector<int>& absQZ =
                strictUsesAbsoluteGeometry() ? decodedResidualsZ_ : emptyAbsQ;
            RDPatchDecoderResult rr = runRDPatchDecoderConquest(
                mesh_,
                decodedSymbols_,
                decodedResidualsX_,
                decodedResidualsY_,
                decodedResidualsZ_,
                decodedFaceRanks_,
                /*ringSizes=*/{},
                /*ringQX=*/{},
                /*ringQY=*/{},
                /*ringQZ=*/{},
                absQX,
                absQY,
                absQZ,
                firstGateQ,
                decodedInputOffsets_,
                decodedSymbolInputQs_,
                /*forcePMSFBoundaryGates=*/true,
                /*forceStrictRDConquest=*/true);
            if (const char* e = std::getenv("PMSF_BPV_DIAG"); e && e[0] == '1') {
                std::cerr << "[PCLTTMStrict rank replay] added="
                          << rr.numAddedVertices
                          << " consumed=" << rr.numSymbolsTotal
                          << "/" << rr.numSymsAvailable
                          << " ranks=" << decodedFaceRanks_.size()
                          << " V_out=" << rr.numVerticesOut
                          << "\n";
            }
            return;
        }
        if (forceRingReplay ||
            (autoSideInfoReplay && hasRingSideInfo && !hasSparseRingTail)) {
            std::vector<unsigned char> ringSizes;
            std::vector<int> ringQX;
            std::vector<int> ringQY;
            std::vector<int> ringQZ;
            ringSizes.reserve(decodedRingQs_.size());
            for (const auto& ring : decodedRingQs_) {
                ringSizes.push_back(
                    static_cast<unsigned char>(std::min<size_t>(255, ring.size())));
                for (const auto& q : ring) {
                    ringQX.push_back(q[0]);
                    ringQY.push_back(q[1]);
                    ringQZ.push_back(q[2]);
                }
            }
            std::vector<int> firstGateQ;
            firstGateQ.reserve(6);
            if (!layerSeedQs_.empty()) {
                for (int v : layerSeedQs_.front()) firstGateQ.push_back(v);
            }
            mesh_->resetState();
            mesh_->invalidateAllVertexDegrees();
            for (MCGAL::Vertex* v : mesh_->vertices()) {
                if (v && !v->isRemoved()) v->clearTag();
            }
            RDPatchDecoderResult rr = runRDPatchDecoderConquest(
                mesh_,
                decodedSymbols_,
                decodedResidualsX_,
                decodedResidualsY_,
                decodedResidualsZ_,
                /*faceRanks=*/{},
                ringSizes,
                ringQX,
                ringQY,
                ringQZ,
                /*absQX=*/{},
                /*absQY=*/{},
                /*absQZ=*/{},
                firstGateQ,
                decodedInputOffsets_,
                decodedSymbolInputQs_,
                /*forcePMSFBoundaryGates=*/true,
                /*forceStrictRDConquest=*/true);
            if (const char* e = std::getenv("PMSF_BPV_DIAG"); e && e[0] == '1') {
                std::cerr << "[PCLTTMStrict ring replay] added="
                          << rr.numAddedVertices
                          << " consumed=" << rr.numSymbolsTotal
                          << "/" << rr.numSymsAvailable
                          << " rings=" << ringSizes.size()
                          << " V_out=" << rr.numVerticesOut
                          << "\n";
            }
            return;
        }
        if (envEnabled("PMSF_PCLTTM_USE_RD_REPLAY")) {
            std::vector<int> firstGateQ;
            firstGateQ.reserve(6);
            if (!layerSeedQs_.empty()) {
                for (int v : layerSeedQs_.front()) firstGateQ.push_back(v);
            }
            mesh_->resetState();
            mesh_->invalidateAllVertexDegrees();
            for (MCGAL::Vertex* v : mesh_->vertices()) {
                if (v && !v->isRemoved()) v->clearTag();
            }
            RDPatchDecoderResult rr = runRDPatchDecoderConquest(
                mesh_,
                decodedSymbols_,
                decodedResidualsX_,
                decodedResidualsY_,
                decodedResidualsZ_,
                /*faceRanks=*/{},
                /*ringSizes=*/{},
                /*ringQX=*/{},
                /*ringQY=*/{},
                /*ringQZ=*/{},
                /*absQX=*/{},
                /*absQY=*/{},
                /*absQZ=*/{},
                firstGateQ,
                decodedInputOffsets_,
                decodedSymbolInputQs_,
                /*forcePMSFBoundaryGates=*/true,
                /*forceStrictRDConquest=*/true);
            if (const char* e = std::getenv("PMSF_BPV_DIAG"); e && e[0] == '1') {
                std::cerr << "[PCLTTMStrict RD replay] added="
                          << rr.numAddedVertices
                          << " consumed=" << rr.numSymbolsTotal
                          << "/" << rr.numSymsAvailable
                          << " V_out=" << rr.numVerticesOut
                          << "\n";
            }
            return;
        }
        RDPatchGateQueue queue;
        std::set<MCGAL::Halfedge*> enqueued;
        std::set<MCGAL::Facet*> facesEnqueued;
        std::set<MCGAL::Facet*> facesProcessed;
        std::set<MCGAL::Facet*> nullPatchFaces;
        std::map<MCGAL::Halfedge*, std::array<int, 6>> protectedPaperGates;
        QuantVertexIndex liveVertexIndex = buildLiveVertexIndex(mesh_);
        const bool singleSeed = envEnabled("PMSF_PCLTTM_SINGLE_SEED");
        const bool reverseWalk = envEnabledDefault("PMSF_PCLTTM_STRICT_WALK_REVERSE", false);
        const RDPatchSymbol& firstSym = reverseWalk ? decodedSymbols_.back() : decodedSymbols_.front();
        unsigned short currentLayer = firstSym.layer;
        unsigned char currentPhase = firstSym.phase;
        auto activeSeedForLayer = [&](unsigned short layer) -> MCGAL::Halfedge* {
            MCGAL::Halfedge* seedInput = nullptr;
            if (layer < layerSeedQs_.size()) {
                seedInput = findInputBoundaryBySeedQIndexed(liveVertexIndex, layerSeedQs_[layer]);
                if (!seedInput) seedInput = findInputBoundaryBySeedQ(mesh_, layerSeedQs_[layer]);
                if (traceOpLimit() > 0) {
                    const auto& q = layerSeedQs_[layer];
                    std::fprintf(stderr,
                                 "[PCLTTM trace dec seed] layer=%u q=(%d,%d,%d)-(%d,%d,%d) found=%d\n",
                                 static_cast<unsigned>(layer),
                                 q[0], q[1], q[2], q[3], q[4], q[5],
                                 seedInput ? 1 : 0);
                }
            }
            if (!seedInput) seedInput = seed;
            const bool seedIsInputBoundary =
                envEnabledDefault("PMSF_PCLTTM_STRICT_SEED_IS_INPUT_BOUNDARY", true);
            MCGAL::Halfedge* active = seedIsInputBoundary
                                           ? gateFromInputBoundary(seedInput)
                                           : seedInput;
            if (!isWalkGate(active)) active = seedInput;
            if (!isWalkGate(active)) active = pickWalkLongestEdge(mesh_);
            return active;
        };
        resetWalkConquestFlags(mesh_, true);
        MCGAL::Halfedge* currentLayerSeed = activeSeedForLayer(currentLayer);
        if (!singleSeed) markWalkSeedInput(currentLayerSeed);
        if (singleSeed) {
            seedWalkSingleGate(currentLayerSeed, queue, enqueued, facesEnqueued, nullPatchFaces);
        } else {
            seedWalkFaces(mesh_, queue, enqueued, facesEnqueued, nullPatchFaces);
        }
        const bool mergeWalkPatches =
            !envEnabled("PMSF_PCLTTM_STRICT_DISABLE_FIG12_PATCH");

        size_t residualIdx = reverseWalk ? decodedResidualsX_.size() : 0;
        int inserted = 0;
        int consumed = 0;
        int mergedPatches = 0;
        int failedPatchDiscoveries = 0;
        int inferredPackedNulls = 0;
        int stalledSymbols = 0;
        int looseFallbacks = 0;
        int vInputLookups = 0;
        int vInputIndexedHits = 0;
        int vInputGateHits = 0;
        int skippedInvalidGates = 0;
        int skippedProcessedGates = 0;
        int emptyQueueReseeds = 0;
        int noGateResetRetries = 0;
        int noGateRetryStoredSi = -1;
        int noGateRetryCount = 0;
        struct DeferredReverseV {
            int storedSi = -1;
            size_t ridx = 0;
            int valence = 0;
            unsigned char phase = 0;
        };
        std::vector<DeferredReverseV> deferredReverse;
        struct SkippedNoGateV {
            int storedSi = -1;
            size_t ridx = 0;
            int valence = 0;
            unsigned char phase = 0;
        };
        std::vector<SkippedNoGateV> skippedNoGateV;
        int tailGeometryFallbacks = 0;
        int debugLimit = -1;
        if (const char* e = std::getenv("PMSF_PCLTTM_STRICT_DEC_LIMIT")) {
            debugLimit = std::atoi(e);
        }
        const int nSyms = static_cast<int>(decodedSymbols_.size());
        const int maxGatePops = std::max(10000, nSyms * 40);
        const bool useSparseRingTail =
            !reverseWalk && decodedRingStartIndex_ > 0 &&
            !decodedRingQs_.empty() && decodedFaceRanks_.empty();
        std::vector<int> decodedVValences;
        decodedVValences.reserve(decodedResidualsX_.size());
        for (const RDPatchSymbol& sym : decodedSymbols_) {
            if (sym.kind != RDPatchSymbolKind::NULL_PATCH) {
                decodedVValences.push_back(symbolValence(sym.kind));
            }
        }
        int progressEvery = 0;
        if (const char* e = std::getenv("PMSF_PCLTTM_PROGRESS_EVERY")) {
            progressEvery = std::max(0, std::atoi(e));
        }

        auto resetSection = [&](bool clearTags) {
            facesProcessed.clear();
            nullPatchFaces.clear();
            protectedPaperGates.clear();
            resetWalkConquestFlags(mesh_, clearTags);
            if (clearTags) {
                currentLayerSeed = activeSeedForLayer(currentLayer);
                if (!singleSeed) markWalkSeedInput(currentLayerSeed);
            }
            if (singleSeed) {
                seedWalkSingleGate(currentLayerSeed, queue, enqueued, facesEnqueued, nullPatchFaces);
            } else {
                seedWalkFaces(mesh_, queue, enqueued, facesEnqueued, nullPatchFaces);
            }
        };

        struct SlowLoopLogger {
            bool enabled = false;
            int startSi = 0;
            int& siRef;
            int startConsumed = 0;
            int& consumedRef;
            int startInserted = 0;
            int& insertedRef;
            RDPatchGateQueue& queueRef;
            std::chrono::steady_clock::time_point startClock;
            ~SlowLoopLogger() {
                if (!enabled) return;
                double threshold = 0.05;
                if (const char* e = std::getenv("PMSF_PCLTTM_SLOW_LOOP_SECONDS")) {
                    double v = std::atof(e);
                    if (v > 0.0) threshold = v;
                }
                const double elapsed =
                    std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - startClock).count();
                if (elapsed >= threshold) {
                    std::fprintf(stderr,
                                 "[PCLTTM slow loop] si=%d->%d consumed=%d->%d "
                                 "inserted=%d->%d queue=%zu elapsed=%.6f\n",
                                 startSi, siRef, startConsumed, consumedRef,
                                 startInserted, insertedRef, queueRef.size(), elapsed);
                }
            }
        };

        int si = 0;
        while (si < nSyms && consumed < maxGatePops) {
            SlowLoopLogger slowLoop{
                envEnabled("PMSF_PCLTTM_TRACE_SLOW_LOOP"),
                si,
                si,
                consumed,
                consumed,
                inserted,
                inserted,
                queue,
                std::chrono::steady_clock::now()
            };
            if (debugLimit >= 0 && inserted >= debugLimit) break;
            if (progressEvery > 0 && consumed > 0 && (consumed % progressEvery) == 0) {
                std::fprintf(stderr,
                             "[PCLTTM progress] consumed=%d inserted=%d si=%d queue=%zu "
                             "layer=%u phase=%u skipped_invalid=%d skipped_processed=%d\n",
                             consumed, inserted, si, queue.size(),
                             static_cast<unsigned>(currentLayer),
                             static_cast<unsigned>(currentPhase),
                             skippedInvalidGates, skippedProcessedGates);
            }
            const int storedSi = reverseWalk ? (nSyms - 1 - si) : si;
            const RDPatchSymbol& s =
                decodedSymbols_[static_cast<size_t>(storedSi)];
            if (useSparseRingTail && s.kind != RDPatchSymbolKind::NULL_PATCH &&
                residualIdx >= static_cast<size_t>(decodedRingStartIndex_)) {
                break;
            }
            if (s.layer != currentLayer || s.phase != currentPhase) {
                const bool layerChanged = s.layer != currentLayer;
                currentLayer = s.layer;
                currentPhase = s.phase;
                resetSection(layerChanged);
            }

            MCGAL::Halfedge* h = nullptr;
            MCGAL::Facet* gateFace = nullptr;
            const int inputSi = si;
            size_t sideInfoVIdx = static_cast<size_t>(-1);
            if (inputSi >= 0 && inputSi < static_cast<int>(decodedSymbolInputQs_.size())) {
                MCGAL::Halfedge* input =
                    findInputBoundaryBySeedQIndexed(
                        liveVertexIndex, decodedSymbolInputQs_[static_cast<size_t>(inputSi)]);
                if (!input) {
                    input = findInputBoundaryBySeedQ(
                        mesh_, decodedSymbolInputQs_[static_cast<size_t>(inputSi)]);
                }
                MCGAL::Halfedge* active = gateFromInputBoundary(input);
                if (isWalkGate(active)) {
                    h = active;
                    gateFace = h->face();
                }
            }
            if (!isWalkGate(h) && s.kind != RDPatchSymbolKind::NULL_PATCH &&
                !decodedVInputQs_.empty()) {
                ++vInputLookups;
                const size_t vInputIdx = reverseWalk
                                             ? (residualIdx == 0 ? decodedVInputQs_.size()
                                                                 : residualIdx - 1)
                                             : residualIdx;
                sideInfoVIdx = vInputIdx;
                if (vInputIdx < decodedVInputQs_.size()) {
                    MCGAL::Halfedge* input =
                        findInputBoundaryBySeedQIndexed(
                            liveVertexIndex, decodedVInputQs_[vInputIdx]);
                    if (input) ++vInputIndexedHits;
                    MCGAL::Halfedge* active = gateFromInputBoundary(input);
                    if (isWalkGate(active)) {
                        h = active;
                        gateFace = h->face();
                        ++vInputGateHits;
                    } else if (isWalkGate(input)) {
                        h = input;
                        gateFace = h->face();
                        ++vInputGateHits;
                    }
                }
            }
            bool triedEmptyReseed = false;
            size_t allMinusDeferrals = 0;
            const size_t maxAllMinusDeferrals = queue.size();
            size_t nextVDeferrals = 0;
            size_t maxNextVDeferrals = std::min<size_t>(queue.size(), 4);
            if (const char* e = std::getenv("PMSF_PCLTTM_NEXT_V_DEFERRAL_LIMIT")) {
                maxNextVDeferrals =
                    std::min<size_t>(queue.size(), std::max(0, std::atoi(e)));
            }
            while (!isWalkGate(h)) {
                if (queue.empty()) {
                    if (!triedEmptyReseed &&
                        envEnabledDefault("PMSF_PCLTTM_STRICT_RESEED_ON_EMPTY", true)) {
                        triedEmptyReseed = true;
                        ++emptyQueueReseeds;
                        seedWalkFaces(mesh_, queue, enqueued, facesEnqueued, nullPatchFaces);
                        continue;
                    }
                    break;
                }
                int bucket = -1;
                h = queue.popHighestPriority(&bucket);
                if (!isWalkGate(h)) {
                    ++skippedInvalidGates;
                    continue;
                }
                gateFace = h->face();
                bool protectedPaperGate = false;
                auto protectedIt = protectedPaperGates.find(h);
                if (protectedIt != protectedPaperGates.end()) {
                    MCGAL::Halfedge* protectedInput = inputBoundaryFromGate(h);
                    if (protectedInput && protectedInput->vertex() &&
                        protectedInput->end_vertex()) {
                        MCGAL::PointInt qa =
                            mesh_->floatPosToInt(protectedInput->vertex()->point());
                        MCGAL::PointInt qb =
                            mesh_->floatPosToInt(protectedInput->end_vertex()->point());
                        const auto& expected = protectedIt->second;
                        protectedPaperGate =
                            qa[0] == expected[0] && qa[1] == expected[1] &&
                            qa[2] == expected[2] && qb[0] == expected[3] &&
                            qb[1] == expected[4] && qb[2] == expected[5];
                    }
                    protectedPaperGates.erase(protectedIt);
                }
                if (!protectedPaperGate && gateFace &&
                    (facesProcessed.count(gateFace) || gateFace->isProcessed())) {
                    ++skippedProcessedGates;
                    h = nullptr;
                    gateFace = nullptr;
                    continue;
                }
                if (s.kind == RDPatchSymbolKind::NULL_PATCH &&
                    envEnabledDefault("PMSF_PCLTTM_DEFER_ALL_MINUS_NULL", false) &&
                    faceAllMinus(gateFace) && allMinusDeferrals < maxAllMinusDeferrals &&
                    !queue.empty()) {
                    ++allMinusDeferrals;
                    queue.push(h, walkBucket(h, nullPatchFaces));
                    h = nullptr;
                    gateFace = nullptr;
                    continue;
                }
                if (s.kind == RDPatchSymbolKind::NULL_PATCH &&
                    envEnabled("PMSF_PCLTTM_DEFER_NULL_MATCHING_NEXT_V") &&
                    nextVDeferrals < maxNextVDeferrals && !queue.empty()) {
                    int nextStoredSi = reverseWalk ? storedSi - 1 : storedSi + 1;
                    if (nextStoredSi >= 0 && nextStoredSi < nSyms) {
                        const RDPatchSymbol& nextSym =
                            decodedSymbols_[static_cast<size_t>(nextStoredSi)];
                        const int nextValence = symbolValence(nextSym.kind);
                        MCGAL::PointInt nextPatchTargetQ(0, 0, 0);
                        const bool nextHasTargetQ =
                            strictUsesAbsoluteGeometry() &&
                            residualIdx < decodedResidualsX_.size() &&
                            residualIdx < decodedResidualsY_.size() &&
                            residualIdx < decodedResidualsZ_.size();
                        if (nextHasTargetQ) {
                            nextPatchTargetQ =
                                MCGAL::PointInt(decodedResidualsX_[residualIdx],
                                                decodedResidualsY_[residualIdx],
                                                decodedResidualsZ_[residualIdx]);
                        }
                        const bool forbidProcessedPatch =
                            envEnabled("PMSF_PCLTTM_STRICT_FORBID_PROCESSED_PATCH");
                        MCGAL::Vertex* candidateFront = h ? h->end_vertex() : nullptr;
                        if (nextSym.layer == s.layer && nextSym.phase == s.phase &&
                            nextValence >= 4 &&
                            candidateFront && !candidateFront->isRemoved() &&
                            candidateFront->vertex_degree() == nextValence &&
                            gateCanDecodeAsVWithoutMerging(
                                h, nextValence,
                                forbidProcessedPatch ? &facesProcessed : nullptr,
                                -1, mesh_, nextHasTargetQ ? &nextPatchTargetQ : nullptr)) {
                            ++nextVDeferrals;
                            queue.push(h, walkBucket(h, nullPatchFaces));
                            h = nullptr;
                            gateFace = nullptr;
                            continue;
                        }
                    }
                }
                break;
            }
            if (!isWalkGate(h)) {
                if (s.kind != RDPatchSymbolKind::NULL_PATCH) {
                    if (envEnabled("PMSF_PCLTTM_RESET_ON_NOGATE_V")) {
                        if (noGateRetryStoredSi != storedSi) {
                            noGateRetryStoredSi = storedSi;
                            noGateRetryCount = 0;
                        }
                        const int maxNoGateRetries = []() {
                            if (const char* e =
                                    std::getenv("PMSF_PCLTTM_NOGATE_RESET_RETRIES")) {
                                return std::max(0, std::atoi(e));
                            }
                            return 1;
                        }();
                        if (noGateRetryCount < maxNoGateRetries) {
                            ++noGateRetryCount;
                            ++noGateResetRetries;
                            resetSection(false);
                            continue;
                        }
                    }
                    const int skippedValence = symbolValence(s.kind);
                    const size_t skippedRidx =
                        reverseWalk
                            ? (residualIdx == 0 ? decodedResidualsX_.size()
                                                : residualIdx - 1)
                            : residualIdx;
                    if (skippedValence >= 3 &&
                        skippedRidx < decodedResidualsX_.size()) {
                        skippedNoGateV.push_back(
                            SkippedNoGateV{storedSi, skippedRidx, skippedValence, s.phase});
                        if (envEnabled("PMSF_PCLTTM_TRACE_NOGATE_SKIP")) {
                            std::fprintf(stderr,
                                         "[PCLTTM no-gate skip] si=%d storedSi=%d ridx=%zu "
                                         "valence=%d layer=%u phase=%u\n",
                                         si, storedSi, skippedRidx, skippedValence,
                                         static_cast<unsigned>(s.layer),
                                         static_cast<unsigned>(s.phase));
                        }
                    }
                }
                ++si;
                stalledSymbols = 0;
                continue;
            }
            ++consumed;
            
            if (s.kind == RDPatchSymbolKind::NULL_PATCH) {
                const std::clock_t nullStartClock = std::clock();
                traceDecoderGate(mesh_, si, static_cast<int>(s.phase), "N", h, -1);
                if (gateFace) {
                    facesProcessed.insert(gateFace);
                    gateFace->setProcessedFlag();
                    nullPatchFaces.insert(gateFace);
                }
                applyStrictNullGateTag(h);
                pushWalkNullNeighbors(h, queue, enqueued, facesEnqueued, nullPatchFaces);
                ++si;
                stalledSymbols = 0;
                if (envEnabled("PMSF_PCLTTM_TRACE_SLOW_NULL")) {
                    const double elapsed =
                        static_cast<double>(std::clock() - nullStartClock) / CLOCKS_PER_SEC;
                    const double threshold = []() {
                        if (const char* e = std::getenv("PMSF_PCLTTM_SLOW_NULL_SECONDS")) {
                            double v = std::atof(e);
                            if (v > 0.0) return v;
                        }
                        return 0.005;
                    }();
                    if (elapsed >= threshold) {
                        std::fprintf(stderr,
                                     "[PCLTTM slow null] si=%d consumed=%d queue=%zu "
                                     "elapsed=%.6f\n",
                                     si - 1, consumed, queue.size(), elapsed);
                    }
                }
                continue;
            }

            auto markPackedNullAndRetry = [&]() -> bool {
                if (!packedNullInferenceAllowed_ &&
                    !envEnabled("PMSF_PCLTTM_STRICT_ALLOW_UNPACKED_NULL_INFERENCE")) {
                    return false;
                }
                if (gateFace) {
                    facesProcessed.insert(gateFace);
                    gateFace->setProcessedFlag();
                    nullPatchFaces.insert(gateFace);
                }
                pushWalkNullNeighbors(h, queue, enqueued, facesEnqueued, nullPatchFaces);
                ++inferredPackedNulls;
                ++stalledSymbols;
                return true;
            };
            auto consumeUnmatchedV = [&]() {
                ++failedPatchDiscoveries;
                if (reverseWalk) {
                    if (residualIdx > 0) --residualIdx;
                } else {
                    ++residualIdx;
                }
                ++si;
                stalledSymbols = 0;
            };

            const int valence = symbolValence(s.kind);
            if (valence < 3) {
                ++si;
                continue;
            }
            const bool traceSlowOp = envEnabled("PMSF_PCLTTM_TRACE_SLOW_OP");
            const double slowOpThreshold = []() {
                if (const char* e = std::getenv("PMSF_PCLTTM_SLOW_OP_SECONDS")) {
                    double v = std::atof(e);
                    if (v > 0.0) return v;
                }
                return 0.05;
            }();
            const std::clock_t opStartClock = std::clock();
            traceDecoderGate(mesh_, si, static_cast<int>(s.phase), "V", h, valence);
            MCGAL::Vertex* symbolInputA = nullptr;
            MCGAL::Vertex* symbolInputB = nullptr;
            if (MCGAL::Halfedge* symbolInput = inputBoundaryFromGate(h)) {
                symbolInputA = symbolInput->vertex();
                symbolInputB = symbolInput->end_vertex();
            }
            bool mergedPatch = false;
            std::vector<MCGAL::Vertex*> boundary;
            MCGAL::Halfedge* discoveryGate = h;
            if (envEnabled("PMSF_PCLTTM_DEC_OPPOSITE_PATCH")) {
                if (MCGAL::Halfedge* input = inputBoundaryFromGate(discoveryGate)) {
                    MCGAL::Halfedge* altInput = input->opposite();
                    MCGAL::Halfedge* altGate = gateFromInputBoundary(altInput);
                    if (isWalkGate(altGate)) discoveryGate = altGate;
                }
            }
            const size_t rankIdx = reverseWalk
                                       ? (residualIdx == 0 ? decodedFaceRanks_.size()
                                                           : residualIdx - 1)
                                       : residualIdx;
            const size_t offsetIdx = reverseWalk
                                         ? (residualIdx == 0 ? decodedInputOffsets_.size()
                                                             : residualIdx - 1)
                                         : residualIdx;
            const size_t ringIdx = reverseWalk
                                       ? (residualIdx == 0 ? decodedRingQs_.size()
                                                           : residualIdx - 1)
                                       : residualIdx;
            const bool ringSideInfoForCurrent =
                !useSparseRingTail && ringIdx < decodedRingQs_.size();
            const int inputOffsetCode =
                offsetIdx < decodedInputOffsets_.size()
                    ? static_cast<int>(decodedInputOffsets_[offsetIdx])
                    : -1;
            auto skipFailedReverseRing = [&]() -> bool {
                if (!reverseWalk || ringIdx >= decodedRingQs_.size()) return false;
                deferredReverse.push_back(
                    DeferredReverseV{storedSi, ringIdx, valence, s.phase});
                ++failedPatchDiscoveries;
                if (residualIdx > 0) --residualIdx;
                ++si;
                stalledSymbols = 0;
                return true;
            };
            MCGAL::Facet* rankedFallbackFace = nullptr;
            MCGAL::Halfedge* rankedFallbackGate = nullptr;
            if (shouldTraceOp(si)) {
                std::fprintf(stderr, "[PCLTTM trace dec offset] si=%d ridx=%zu off=%d\n",
                             si, offsetIdx, inputOffsetCode);
            }
            if (!envEnabled("PMSF_PCLTTM_STRICT_DISABLE_FACE_RANKS") &&
                rankIdx < decodedFaceRanks_.size()) {
                MCGAL::Facet* ranked =
                    livingFaceByRank(mesh_, decodedFaceRanks_[rankIdx]);
                MCGAL::Halfedge* rankedGate = findLiveFaceGate(ranked);
                if (ranked && rankedGate) {
                    rankedFallbackFace = ranked;
                    rankedFallbackGate = rankedGate;
                }
                if (ranked && sideInfoVIdx < decodedVInputQs_.size()) {
                    const auto& iq = decodedVInputQs_[sideInfoVIdx];
                    std::array<int, 3> qa{iq[0], iq[1], iq[2]};
                    std::array<int, 3> qb{iq[3], iq[4], iq[5]};
                    for (MCGAL::Halfedge* eh : boundedFaceHalfedges(ranked)) {
                        if (!isWalkGate(eh)) continue;
                        const bool forward = sameQuantNear(mesh_, eh->vertex(), qa) &&
                                             sameQuantNear(mesh_, eh->end_vertex(), qb);
                        const bool backward = sameQuantNear(mesh_, eh->vertex(), qb) &&
                                              sameQuantNear(mesh_, eh->end_vertex(), qa);
                        if (forward || backward) {
                            MCGAL::Halfedge* input = forward ? eh : eh->opposite();
                            MCGAL::Halfedge* gate = gateFromInputBoundary(input);
                            if (isWalkGate(gate)) rankedGate = gate;
                            break;
                        }
                    }
                }
                if (rankedGate) {
                    discoveryGate = rankedGate;
                    gateFace = ranked;
                }
            }
            MCGAL::Facet* f = nullptr;
            bool usedRingPatch = false;
            bool usedAltFanWalk = false;
            const bool forceRankPatch =
                envEnabled("PMSF_PCLTTM_STRICT_FORCE_FACE_RANK_PATCH");
            const bool hadRingPatchSideInfo =
                mergeWalkPatches && ringSideInfoForCurrent;
            size_t ridxForPoint = residualIdx;
            if (reverseWalk && residualIdx > 0) ridxForPoint = residualIdx - 1;
            MCGAL::PointInt patchTargetQ(0, 0, 0);
            const bool hasPatchTargetQ =
                strictUsesAbsoluteGeometry() && ridxForPoint < decodedResidualsX_.size() &&
                ridxForPoint < decodedResidualsY_.size() &&
                ridxForPoint < decodedResidualsZ_.size();
            if (hasPatchTargetQ) {
                patchTargetQ = MCGAL::PointInt(decodedResidualsX_[ridxForPoint],
                                               decodedResidualsY_[ridxForPoint],
                                               decodedResidualsZ_[ridxForPoint]);
            }
            if (!f && envEnabled("PMSF_PCLTTM_STRICT_GEOM_TRI_PATCH") &&
                strictUsesAbsoluteGeometry() &&
                valence >= 3 && valence <= 6 &&
                ridxForPoint < decodedResidualsX_.size()) {
                const bool forbidProcessedPatch =
                    envEnabled("PMSF_PCLTTM_STRICT_FORBID_PROCESSED_PATCH");
                MCGAL::Facet* byPoint =
                    livingTriangleContainingQ(mesh_, patchTargetQ,
                                              forbidProcessedPatch ? &facesProcessed : nullptr);
                if (byPoint && !byPoint->isRemoved() && hasBoundedFaceLoop(byPoint) &&
                    valence == 3) {
                    f = byPoint;
                    boundary = orderedFaceVertices(f);
                    if (MCGAL::Halfedge* g = findLiveFaceGate(f)) {
                        discoveryGate = g;
                        gateFace = f;
                        f->proxyHalfedge(g);
                    }
                    mergedPatch = false;
                    usedRingPatch = false;
                } else if (byPoint && !byPoint->isRemoved()) {
                    for (MCGAL::Halfedge* g : boundedFaceHalfedges(byPoint)) {
                        if (!isWalkGate(g)) continue;
                        std::vector<MCGAL::Vertex*> candidateBoundary;
                        bool candidateMerged = false;
                        bool candidateAltFan = false;
                        MCGAL::Facet* candidate =
                            discoverAndMergePatchFromGate(
                                g, valence,
                                forbidProcessedPatch ? &facesProcessed : nullptr,
                                candidateBoundary, candidateMerged,
                                inputOffsetCode, nullptr, mesh_, &patchTargetQ,
                                &candidateAltFan);
                        if (candidate && !candidate->isRemoved() &&
                            hasBoundedFaceLoop(candidate) &&
                            faceDegree(candidate) == valence) {
                            f = candidate;
                            boundary = std::move(candidateBoundary);
                            discoveryGate = g;
                            gateFace = byPoint;
                            mergedPatch = candidateMerged;
                            usedRingPatch = false;
                            usedAltFanWalk = candidateAltFan;
                            break;
                        }
                    }
                }
            }
            if (forceRankPatch && rankedFallbackFace &&
                !rankedFallbackFace->isRemoved() && hasBoundedFaceLoop(rankedFallbackFace)) {
                f = rankedFallbackFace;
                boundary = orderedFaceVertices(f);
                if (rankedFallbackGate && rankedFallbackGate->face() == f) {
                    f->proxyHalfedge(rankedFallbackGate);
                }
                mergedPatch = false;
                usedRingPatch = false;
                ++looseFallbacks;
            }
            if (!f && hadRingPatchSideInfo) {
                f = discoverAndMergePatchFromRingQ(mesh_, &liveVertexIndex,
                                                   decodedRingQs_[ringIdx],
                                                   valence, nullptr,
                                                   nullptr, boundary, mergedPatch, si,
                                                   rankedFallbackFace);
                usedRingPatch = (f != nullptr);
            }
            if (!f && hadRingPatchSideInfo &&
                envEnabled("PMSF_PCLTTM_STRICT_DEFER_RING_FIRST")) {
                if (skipFailedReverseRing()) continue;
            }
            if (!f && decodedRingQs_.empty() && rankedFallbackFace &&
                !rankedFallbackFace->isRemoved() && hasBoundedFaceLoop(rankedFallbackFace)) {
                f = rankedFallbackFace;
                boundary = orderedFaceVertices(f);
                if (rankedFallbackGate && rankedFallbackGate->face() == f) {
                    f->proxyHalfedge(rankedFallbackGate);
                }
                mergedPatch = false;
                usedRingPatch = false;
                ++looseFallbacks;
            }
            if (!f) {
                const bool forbidProcessedPatch =
                    envEnabled("PMSF_PCLTTM_STRICT_FORBID_PROCESSED_PATCH");
                f = mergeWalkPatches
                        ? discoverAndMergePatchFromGate(
                              discoveryGate, valence,
                              forbidProcessedPatch ? &facesProcessed : nullptr, boundary,
                              mergedPatch, inputOffsetCode, nullptr,
                              mesh_, hasPatchTargetQ ? &patchTargetQ : nullptr,
                              &usedAltFanWalk)
                        : discoveryGate->face();
            }
            if (!f && envEnabled("PMSF_PCLTTM_STRICT_ENABLE_LOOSE_PATCH")) {
                MCGAL::Facet* loose = rankedFallbackFace ? rankedFallbackFace
                                      : (discoveryGate ? discoveryGate->face() : nullptr);
                MCGAL::Halfedge* looseGate = rankedFallbackGate ? rankedFallbackGate
                                         : (discoveryGate ? discoveryGate : nullptr);
                if (loose && !loose->isRemoved() && hasBoundedFaceLoop(loose)) {
                    f = loose;
                    boundary = orderedFaceVertices(f);
                    if (looseGate && looseGate->face() == f) f->proxyHalfedge(looseGate);
                    mergedPatch = false;
                    usedRingPatch = false;
                    ++looseFallbacks;
                                    }
            }
                        if (!f && mergeWalkPatches) {
                if (skipFailedReverseRing()) continue;
                ++failedPatchDiscoveries;
                if (!markPackedNullAndRetry()) {
                    consumeUnmatchedV();
                    continue;
                }
                if (stalledSymbols > std::max(1000, static_cast<int>(mesh_->faces().size()) * 8)) {
                    ++si;
                    stalledSymbols = 0;
                }
                continue;  // Paper §4.2: packed null, retry the same V-code.
            }
            if (!f || f->isRemoved()) {
                if (skipFailedReverseRing()) continue;
                if (!markPackedNullAndRetry()) {
                    consumeUnmatchedV();
                    continue;
                }
                if (stalledSymbols > std::max(1000, static_cast<int>(mesh_->faces().size()) * 8)) {
                    ++si;
                    stalledSymbols = 0;
                }
                continue;
            }
            if (!hasBoundedFaceLoop(f)) {
                if (skipFailedReverseRing()) continue;
                ++failedPatchDiscoveries;
                if (!markPackedNullAndRetry()) {
                    consumeUnmatchedV();
                    continue;
                }
                if (stalledSymbols > std::max(1000, static_cast<int>(mesh_->faces().size()) * 8)) {
                    ++si;
                    stalledSymbols = 0;
                }
                continue;
            }
            MCGAL::Halfedge* ringInsertGate = nullptr;
            if (usedRingPatch && ringSideInfoForCurrent) {
                ringInsertGate = findRingOrderedHalfedge(
                    mesh_, f, decodedRingQs_[ringIdx]);
                if (ringInsertGate) f->proxyHalfedge(ringInsertGate);
            }
            if (!usedRingPatch) {
                MCGAL::Halfedge* paperInput = nullptr;
                if (symbolInputA && symbolInputB) {
                    paperInput = faceHalfedgeBetweenOriented(f, symbolInputA, symbolInputB);
                }
                if (!paperInput) {
                    if (MCGAL::Halfedge* input = inputBoundaryFromGate(discoveryGate)) {
                        paperInput = faceHalfedgeBetweenOriented(
                            f, input->vertex(), input->end_vertex());
                    }
                }
                if (paperInput) {
                    f->proxyHalfedge(paperInput);
                    ringInsertGate = paperInput;
                } else {
                    f->normalize_proxy_to_min_position_vertex(mesh_->bboxMin, mesh_->f_quantStep);
                }
                boundary = orderedFaceVertices(f);
            }
            if (boundary.empty()) boundary = orderedFaceVertices(f);
            MCGAL::Vertex* inputBoundaryA = nullptr;
            MCGAL::Vertex* inputBoundaryB = nullptr;
            if (usedRingPatch && inputOffsetCode >= 0 && !boundary.empty()) {
                const int n = static_cast<int>(boundary.size());
                const int off = inputOffsetCode & 0x7;
                const bool reversed = (inputOffsetCode & 0x8) != 0;
                if (off >= 0 && off < n) {
                    MCGAL::Vertex* a = boundary[static_cast<size_t>(off)];
                    MCGAL::Vertex* b = boundary[static_cast<size_t>((off + 1) % n)];
                    inputBoundaryA = reversed ? b : a;
                    inputBoundaryB = reversed ? a : b;
                }
            } else if (symbolInputA && symbolInputB) {
                inputBoundaryA = symbolInputA;
                inputBoundaryB = symbolInputB;
            } else if (MCGAL::Halfedge* input = inputBoundaryFromGate(discoveryGate)) {
                inputBoundaryA = input->vertex();
                inputBoundaryB = input->end_vertex();
            }
            if (!inputBoundaryA || !inputBoundaryB) {
                inputBoundaryA = symbolInputA;
                inputBoundaryB = symbolInputB;
            }
            if (MCGAL::Halfedge* paperInput =
                    faceHalfedgeBetweenOriented(f, inputBoundaryA, inputBoundaryB)) {
                f->proxyHalfedge(paperInput);
                ringInsertGate = paperInput;
                boundary = orderedFaceVertices(f);
            }
            const bool useRingPrediction =
                !strictUsesAbsoluteGeometry() && ringSideInfoForCurrent;
            MCGAL::PointInt baryQ = useRingPrediction
                                        ? barycenterFromRingQ(decodedRingQs_[ringIdx])
                                        : MCGAL::rdBarycenterQuantized(f, mesh_);
            size_t ridx = residualIdx;
            if (reverseWalk) {
                if (residualIdx == 0) break;
                ridx = residualIdx - 1;
            }
            MCGAL::Vector3i decoded(ridx < decodedResidualsX_.size() ? decodedResidualsX_[ridx] : 0,
                                    ridx < decodedResidualsY_.size() ? decodedResidualsY_[ridx] : 0,
                                    ridx < decodedResidualsZ_.size() ? decodedResidualsZ_[ridx] : 0);
            MCGAL::Vector3i cart = decoded;
            MCGAL::Vector3 n = polygonNormal(f);
            if (!strictUsesCartesianResiduals() && n.length() != 0 && inputBoundaryA && inputBoundaryB) {
                MCGAL::Vector3 t1, t2;
                MCGAL::determineFrenetFrame(MCGAL::Vector3(inputBoundaryA->point()),
                                            MCGAL::Vector3(inputBoundaryB->point()),
                                            n, t1, t2);
                if (t1.length() != 0 && t2.length() != 0) {
                    cart = MCGAL::invFrenetRotation(decoded, t1, t2, n);
                }
            }
            MCGAL::PointInt vQ = strictUsesAbsoluteGeometry()
                                      ? MCGAL::PointInt(decoded[0], decoded[1], decoded[2])
                                      : MCGAL::PointInt(baryQ[0] + cart[0],
                                                        baryQ[1] + cart[1],
                                                        baryQ[2] + cart[2]);
            auto fanWouldDegenerate = [&](const std::vector<MCGAL::Vertex*>& ring,
                                          MCGAL::PointInt centerQ) {
                if (ring.size() < 3) return true;
                const int nRing = static_cast<int>(ring.size());
                const int64_t cx0 = centerQ[0];
                const int64_t cy0 = centerQ[1];
                const int64_t cz0 = centerQ[2];
                for (int i = 0; i < nRing; ++i) {
                    MCGAL::Vertex* a = ring[static_cast<size_t>(i)];
                    MCGAL::Vertex* b = ring[static_cast<size_t>((i + 1) % nRing)];
                    if (!a || !b || a->isRemoved() || b->isRemoved()) return true;
                    MCGAL::PointInt aq = mesh_->floatPosToInt(a->point());
                    MCGAL::PointInt bq = mesh_->floatPosToInt(b->point());
                    const int64_t ux = static_cast<int64_t>(aq[0]) - cx0;
                    const int64_t uy = static_cast<int64_t>(aq[1]) - cy0;
                    const int64_t uz = static_cast<int64_t>(aq[2]) - cz0;
                    const int64_t vx = static_cast<int64_t>(bq[0]) - cx0;
                    const int64_t vy = static_cast<int64_t>(bq[1]) - cy0;
                    const int64_t vz = static_cast<int64_t>(bq[2]) - cz0;
                    const int64_t cx = uy * vz - uz * vy;
                    const int64_t cy = uz * vx - ux * vz;
                    const int64_t cz = ux * vy - uy * vx;
                    if (cx == 0 && cy == 0 && cz == 0) return true;
                }
                return false;
            };
            auto liveVertexExistsAtQ = [&](MCGAL::PointInt q) {
                QuantKey key{q[0], q[1], q[2]};
                auto it = liveVertexIndex.find(key);
                if (it == liveVertexIndex.end()) return false;
                for (MCGAL::Vertex* v : it->second) {
                    if (v && !v->isRemoved()) return true;
                }
                return false;
            };
            auto nudgeDecodedPoint = [&]() {
                const int delta[6][3] = {
                    {1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
                    {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
                };
                for (const auto& d : delta) {
                    MCGAL::PointInt cand(vQ[0] + d[0], vQ[1] + d[1], vQ[2] + d[2]);
                    if (!liveVertexExistsAtQ(cand) && !fanWouldDegenerate(boundary, cand)) {
                        vQ = cand;
                        return true;
                    }
                }
                return false;
            };
            if ((fanWouldDegenerate(boundary, vQ) || liveVertexExistsAtQ(vQ)) &&
                envEnabled("PMSF_PCLTTM_AVOID_DUP_Q")) {
                nudgeDecodedPoint();
            }
            if (fanWouldDegenerate(boundary, vQ) &&
                envEnabled("PMSF_PCLTTM_STRICT_ENABLE_LOOSE_PATCH")) {
                MCGAL::Facet* loose = discoveryGate ? discoveryGate->face() : nullptr;
                if (loose && !loose->isRemoved() && hasBoundedFaceLoop(loose)) {
                    f = loose;
                    boundary = orderedFaceVertices(f);
                    mergedPatch = false;
                    usedRingPatch = false;
                    ringInsertGate = nullptr;
                    baryQ = strictUsesAbsoluteGeometry()
                                ? MCGAL::PointInt(0, 0, 0)
                                : MCGAL::rdBarycenterQuantized(f, mesh_);
                    vQ = strictUsesAbsoluteGeometry()
                             ? MCGAL::PointInt(decoded[0], decoded[1], decoded[2])
                             : MCGAL::PointInt(baryQ[0] + cart[0],
                                               baryQ[1] + cart[1],
                                               baryQ[2] + cart[2]);
                    ++looseFallbacks;
                }
            }
            if (liveVertexExistsAtQ(vQ) &&
                !envEnabled("PMSF_PCLTTM_AVOID_DUP_Q") &&
                envEnabled("PMSF_PCLTTM_STRICT_ENABLE_LOOSE_PATCH")) {
                MCGAL::Facet* loose = discoveryGate ? discoveryGate->face() : nullptr;
                if (loose && !loose->isRemoved() && hasBoundedFaceLoop(loose)) {
                    f = loose;
                    boundary = orderedFaceVertices(f);
                    mergedPatch = false;
                    usedRingPatch = false;
                    ringInsertGate = nullptr;
                    baryQ = strictUsesAbsoluteGeometry()
                                ? MCGAL::PointInt(0, 0, 0)
                                : MCGAL::rdBarycenterQuantized(f, mesh_);
                    vQ = strictUsesAbsoluteGeometry()
                             ? MCGAL::PointInt(decoded[0], decoded[1], decoded[2])
                             : MCGAL::PointInt(baryQ[0] + cart[0],
                                               baryQ[1] + cart[1],
                                               baryQ[2] + cart[2]);
                    ++looseFallbacks;
                }
            }
            if (liveVertexExistsAtQ(vQ) &&
                envEnabled("PMSF_PCLTTM_AVOID_DUP_Q")) {
                nudgeDecodedPoint();
            }
            MCGAL::Halfedge* insertGate = ringInsertGate ? ringInsertGate : findLiveFaceGate(f);
            if (!insertGate) {
                if (!markPackedNullAndRetry()) {
                    consumeUnmatchedV();
                    continue;
                }
                if (stalledSymbols > std::max(1000, static_cast<int>(mesh_->faces().size()) * 8)) {
                    ++si;
                    stalledSymbols = 0;
                }
                continue;
            }
            std::vector<MCGAL::Halfedge*> preInsertPaperDecimatingGates;
            if (s.phase == 0) {
                preInsertPaperDecimatingGates =
                    collectDecodedPaperDecimatingGatesFromPolygon(
                        f, symbolInputA ? symbolInputA : inputBoundaryA,
                        symbolInputB ? symbolInputB : inputBoundaryB);
            }
            bool protectQueuedPaperGates = false;
            bool markQueuedPaperFacesProcessed = false;
            if (s.phase == 0 && valence == 3 &&
                envEnabled("PMSF_PCLTTM_REPLACE_STALE_FACE_GATES")) {
                const bool inputConquered =
                    inputBoundaryA && inputBoundaryB &&
                    !inputBoundaryA->isRemoved() && !inputBoundaryB->isRemoved() &&
                    inputBoundaryA->isConquered() && inputBoundaryB->isConquered();
                bool hasUnconqueredBoundary = false;
                bool allBoundaryConquered = true;
                for (MCGAL::Vertex* bv : boundary) {
                    if (!bv || bv->isRemoved()) continue;
                    if (!bv->isConquered()) {
                        hasUnconqueredBoundary = true;
                        allBoundaryConquered = false;
                    }
                }
                protectQueuedPaperGates = inputConquered && hasUnconqueredBoundary;
                markQueuedPaperFacesProcessed = allBoundaryConquered;
            }
            traceDecoderOp(mesh_, inserted, static_cast<int>(s.phase), valence,
                           inputBoundaryA, inputBoundaryB, boundary);
            MCGAL::Halfedge* tagInput =
                faceHalfedgeBetweenOriented(f, inputBoundaryA, inputBoundaryB);
            if (!applyStrictBoundaryTagsFromPaperInput(tagInput, valence)) {
                applyStrictBoundaryTags(boundary);
            }
            if (s.phase != 0) {
                pushDecodedPaperCleaningGates(boundary, f, queue, enqueued, facesEnqueued,
                                              facesProcessed, nullPatchFaces);
            }
            MCGAL::Halfedge* newH =
                mesh_->create_center_vertex_without_init(insertGate, mesh_->intPosToFloat(vQ));
            if (gateFace && !gateFace->isRemoved()) {
                facesProcessed.insert(gateFace);
                gateFace->setProcessedFlag();
            }
            MCGAL::Vertex* insertedCenter = newH ? newH->vertex() : nullptr;
            if (insertedCenter && !insertedCenter->isRemoved()) {
                indexLiveVertex(mesh_, liveVertexIndex, insertedCenter);
                for (MCGAL::Halfedge* hv : boundedVertexHalfedges(insertedCenter)) {
                    MCGAL::Facet* pf = hv ? hv->face() : nullptr;
                    if (pf && !pf->isRemoved()) {
                        pf->setProcessedFlag();
                        facesProcessed.insert(pf);
                    }
                }
                markDecodedPatchInteriorFacesProcessed(boundary, insertedCenter,
                                                       facesProcessed);
                if (usedAltFanWalk &&
                    envEnabledDefault("PMSF_PCLTTM_MARK_STALE_ONE_BOUNDARY", true)) {
                    markStaleOneBoundaryFacesProcessed(boundary, facesProcessed);
                }
            }
            if (s.phase == 0) {
                std::map<MCGAL::Halfedge*, std::array<int, 6>>* protectedGateSet =
                    protectQueuedPaperGates
                        ? &protectedPaperGates
                        : nullptr;
                std::set<MCGAL::Facet*>* processedQueuedFaces =
                    markQueuedPaperFacesProcessed
                        ? &facesProcessed
                        : nullptr;
                std::set<MCGAL::Facet*>* processedAlreadyQueuedFaces =
                    envEnabled("PMSF_PCLTTM_REPLACE_STALE_FACE_GATES")
                        ? &facesProcessed
                        : nullptr;
                pushDecodedPaperDecimatingGateList(mesh_, preInsertPaperDecimatingGates,
                                                   queue, enqueued, facesEnqueued,
                                                   nullPatchFaces, protectedGateSet,
                                                   processedQueuedFaces,
                                                   processedAlreadyQueuedFaces);
                if (preInsertPaperDecimatingGates.empty() &&
                    envEnabled("PMSF_PCLTTM_FALLBACK_POST_INSERT_PUSH")) {
                    pushDecodedPaperDecimatingGates(mesh_, boundary, insertedCenter,
                                                    symbolInputA ? symbolInputA : inputBoundaryA,
                                                    symbolInputB ? symbolInputB : inputBoundaryB,
                                                    queue, enqueued, facesEnqueued,
                                                    nullPatchFaces);
                }
            }
            if (insertedCenter) {
                ++inserted;
                if (mergedPatch) ++mergedPatches;
                insertedCenter->setConquered();
            }
            if (reverseWalk) --residualIdx;
            else ++residualIdx;
            ++si;
            stalledSymbols = 0;
            if (traceSlowOp) {
                const double elapsed =
                    static_cast<double>(std::clock() - opStartClock) / CLOCKS_PER_SEC;
                if (elapsed >= slowOpThreshold) {
                    std::fprintf(stderr,
                                 "[PCLTTM slow op] si=%d inserted=%d layer=%u phase=%u "
                                 "valence=%d merged=%d boundary=%zu elapsed=%.6f\n",
                                 si - 1, inserted, static_cast<unsigned>(s.layer),
                                 static_cast<unsigned>(s.phase), valence,
                                 mergedPatch ? 1 : 0, boundary.size(), elapsed);
                }
            }
        }
        auto tryDeferred = [&](const DeferredReverseV& d) -> bool {
            if (d.valence < 3 || d.ridx >= decodedRingQs_.size() ||
                d.ridx >= decodedResidualsX_.size()) {
                return false;
            }
            bool mergedPatch = false;
            std::vector<MCGAL::Vertex*> boundary;
            MCGAL::Facet* f = discoverAndMergePatchFromRingQ(
                mesh_, &liveVertexIndex, decodedRingQs_[d.ridx], d.valence,
                nullptr, nullptr, boundary, mergedPatch, d.storedSi,
                d.ridx < decodedFaceRanks_.size()
                    ? livingFaceByRank(mesh_, decodedFaceRanks_[d.ridx])
                    : nullptr);
            if ((!f || f->isRemoved() || !hasBoundedFaceLoop(f)) &&
                !envEnabled("PMSF_PCLTTM_STRICT_DISABLE_DEFERRED_RANK_FALLBACK") &&
                d.ridx < decodedFaceRanks_.size()) {
                f = livingFaceByRank(mesh_, decodedFaceRanks_[d.ridx]);
                if (f && !f->isRemoved() && hasBoundedFaceLoop(f)) {
                    boundary = orderedFaceVertices(f);
                    mergedPatch = false;
                    ++looseFallbacks;
                }
            }
            if (!f || f->isRemoved() || !hasBoundedFaceLoop(f)) return false;

            MCGAL::Halfedge* ringInsertGate =
                findRingOrderedHalfedge(mesh_, f, decodedRingQs_[d.ridx]);
            if (ringInsertGate) f->proxyHalfedge(ringInsertGate);
            if (boundary.empty()) boundary = orderedFaceVertices(f);
            MCGAL::Vertex* inputBoundaryA = nullptr;
            MCGAL::Vertex* inputBoundaryB = nullptr;
            const int inputOffsetCode =
                d.ridx < decodedInputOffsets_.size()
                    ? static_cast<int>(decodedInputOffsets_[d.ridx])
                    : -1;
            if (inputOffsetCode >= 0 && !boundary.empty()) {
                const int n = static_cast<int>(boundary.size());
                const int off = inputOffsetCode & 0x7;
                const bool reversed = (inputOffsetCode & 0x8) != 0;
                if (off >= 0 && off < n) {
                    MCGAL::Vertex* a = boundary[static_cast<size_t>(off)];
                    MCGAL::Vertex* b = boundary[static_cast<size_t>((off + 1) % n)];
                    inputBoundaryA = reversed ? b : a;
                    inputBoundaryB = reversed ? a : b;
                }
            }

            MCGAL::PointInt baryQ = strictUsesAbsoluteGeometry()
                                        ? MCGAL::PointInt(0, 0, 0)
                                        : barycenterFromRingQ(decodedRingQs_[d.ridx]);
            MCGAL::Vector3i decoded(decodedResidualsX_[d.ridx],
                                    decodedResidualsY_[d.ridx],
                                    decodedResidualsZ_[d.ridx]);
            MCGAL::Vector3i cart = decoded;
            MCGAL::Vector3 n = polygonNormal(f);
            if (!strictUsesCartesianResiduals() && n.length() != 0 &&
                inputBoundaryA && inputBoundaryB) {
                MCGAL::Vector3 t1, t2;
                MCGAL::determineFrenetFrame(MCGAL::Vector3(inputBoundaryA->point()),
                                            MCGAL::Vector3(inputBoundaryB->point()),
                                            n, t1, t2);
                if (t1.length() != 0 && t2.length() != 0) {
                    cart = MCGAL::invFrenetRotation(decoded, t1, t2, n);
                }
            }
            MCGAL::PointInt vQ = strictUsesAbsoluteGeometry()
                                      ? MCGAL::PointInt(decoded[0], decoded[1], decoded[2])
                                      : MCGAL::PointInt(baryQ[0] + cart[0],
                                                        baryQ[1] + cart[1],
                                                        baryQ[2] + cart[2]);
            MCGAL::Halfedge* insertGate = ringInsertGate ? ringInsertGate : findLiveFaceGate(f);
            if (!insertGate) return false;
            applyStrictBoundaryTags(boundary);
            MCGAL::Halfedge* newH =
                mesh_->create_center_vertex_without_init(insertGate, mesh_->intPosToFloat(vQ));
            MCGAL::Vertex* insertedCenter = newH ? newH->vertex() : nullptr;
            if (!insertedCenter || insertedCenter->isRemoved()) return false;
            indexLiveVertex(mesh_, liveVertexIndex, insertedCenter);
            for (MCGAL::Halfedge* hv : boundedVertexHalfedges(insertedCenter)) {
                MCGAL::Facet* pf = hv ? hv->face() : nullptr;
                if (pf && !pf->isRemoved()) {
                    pf->setProcessedFlag();
                    facesProcessed.insert(pf);
                }
            }
            markDecodedPatchInteriorFacesProcessed(boundary, insertedCenter,
                                                   facesProcessed);
            ++inserted;
            if (mergedPatch) ++mergedPatches;
            insertedCenter->setConquered();
            return true;
        };
        for (int pass = 0; pass < 32 && !deferredReverse.empty(); ++pass) {
            std::vector<DeferredReverseV> stillDeferred;
            stillDeferred.reserve(deferredReverse.size());
            int passInserted = 0;
            auto tryOneDeferred = [&](const DeferredReverseV& d) {
                if (tryDeferred(d)) {
                    ++passInserted;
                } else {
                    stillDeferred.push_back(d);
                }
            };
            if ((pass % 2) == 0) {
                for (const DeferredReverseV& d : deferredReverse) tryOneDeferred(d);
            } else {
                for (auto it = deferredReverse.rbegin(); it != deferredReverse.rend(); ++it) {
                    tryOneDeferred(*it);
                }
            }
            deferredReverse.swap(stillDeferred);
            if (passInserted == 0) break;
        }
        if (envEnabled("PMSF_PCLTTM_TRACE_DEFERRED") && !deferredReverse.empty()) {
            int n = 0;
            for (const DeferredReverseV& d : deferredReverse) {
                if (n++ >= 32) break;
                std::fprintf(stderr,
                             "[PCLTTM deferred remaining] storedSi=%d ridx=%zu valence=%d phase=%u\n",
                             d.storedSi, d.ridx, d.valence,
                             static_cast<unsigned>(d.phase));
                bool mergedPatch = false;
                std::vector<MCGAL::Vertex*> boundary;
                discoverAndMergePatchFromRingQ(
                    mesh_, &liveVertexIndex, decodedRingQs_[d.ridx], d.valence,
                    nullptr, nullptr, boundary, mergedPatch, d.storedSi);
            }
        }
        int sparseTailFallbacks = 0;
        if (useSparseRingTail &&
            residualIdx >= static_cast<size_t>(decodedRingStartIndex_) &&
            residualIdx < decodedResidualsX_.size()) {
            while (residualIdx < decodedResidualsX_.size()) {
                const size_t tailIdx =
                    residualIdx - static_cast<size_t>(decodedRingStartIndex_);
                if (tailIdx >= decodedRingQs_.size() ||
                    residualIdx >= decodedVValences.size()) {
                    break;
                }
                const int valence = decodedVValences[residualIdx];
                bool mergedPatch = false;
                std::vector<MCGAL::Vertex*> boundary;
                MCGAL::Facet* f = discoverAndMergePatchFromRingQ(
                    mesh_, &liveVertexIndex, decodedRingQs_[tailIdx], valence,
                    nullptr, nullptr, boundary, mergedPatch,
                    static_cast<int>(residualIdx), nullptr);
                if (!f || f->isRemoved() || !hasBoundedFaceLoop(f)) break;
                MCGAL::Halfedge* ringInsertGate =
                    findRingOrderedHalfedge(mesh_, f, decodedRingQs_[tailIdx]);
                if (ringInsertGate) f->proxyHalfedge(ringInsertGate);
                if (boundary.empty()) boundary = orderedFaceVertices(f);
                MCGAL::PointInt baryQ =
                    strictUsesAbsoluteGeometry()
                        ? MCGAL::PointInt(0, 0, 0)
                        : barycenterFromRingQ(decodedRingQs_[tailIdx]);
                MCGAL::PointInt vQ =
                    strictUsesAbsoluteGeometry()
                        ? MCGAL::PointInt(decodedResidualsX_[residualIdx],
                                          decodedResidualsY_[residualIdx],
                                          decodedResidualsZ_[residualIdx])
                        : MCGAL::PointInt(baryQ[0] + decodedResidualsX_[residualIdx],
                                          baryQ[1] + decodedResidualsY_[residualIdx],
                                          baryQ[2] + decodedResidualsZ_[residualIdx]);
                MCGAL::Halfedge* insertGate =
                    ringInsertGate ? ringInsertGate : findLiveFaceGate(f);
                if (!insertGate) break;
                applyStrictBoundaryTags(boundary);
                MCGAL::Halfedge* newH =
                    mesh_->create_center_vertex_without_init(insertGate,
                                                             mesh_->intPosToFloat(vQ));
                MCGAL::Vertex* insertedCenter = newH ? newH->vertex() : nullptr;
                if (!insertedCenter || insertedCenter->isRemoved()) break;
                indexLiveVertex(mesh_, liveVertexIndex, insertedCenter);
                for (MCGAL::Halfedge* hv : boundedVertexHalfedges(insertedCenter)) {
                    MCGAL::Facet* pf = hv ? hv->face() : nullptr;
                    if (pf && !pf->isRemoved()) {
                        pf->setProcessedFlag();
                        facesProcessed.insert(pf);
                    }
                }
                markDecodedPatchInteriorFacesProcessed(boundary, insertedCenter,
                                                       facesProcessed);
                insertedCenter->setConquered();
                ++inserted;
                if (mergedPatch) ++mergedPatches;
                ++sparseTailFallbacks;
                ++residualIdx;
            }
        }
        auto liveVertexExistsAtQGlobal = [&](MCGAL::PointInt q) {
            QuantKey key{q[0], q[1], q[2]};
            auto it = liveVertexIndex.find(key);
            if (it == liveVertexIndex.end()) return false;
            for (MCGAL::Vertex* v : it->second) {
                if (v && !v->isRemoved()) return true;
            }
            return false;
        };
        auto fanWouldDegenerateGlobal =
            [&](const std::vector<MCGAL::Vertex*>& ring, MCGAL::PointInt centerQ) {
                if (ring.size() < 3) return true;
                const int nRing = static_cast<int>(ring.size());
                const int64_t cx0 = centerQ[0];
                const int64_t cy0 = centerQ[1];
                const int64_t cz0 = centerQ[2];
                for (int i = 0; i < nRing; ++i) {
                    MCGAL::Vertex* a = ring[static_cast<size_t>(i)];
                    MCGAL::Vertex* b = ring[static_cast<size_t>((i + 1) % nRing)];
                    if (!a || !b || a->isRemoved() || b->isRemoved()) return true;
                    MCGAL::PointInt aq = mesh_->floatPosToInt(a->point());
                    MCGAL::PointInt bq = mesh_->floatPosToInt(b->point());
                    const int64_t ux = static_cast<int64_t>(aq[0]) - cx0;
                    const int64_t uy = static_cast<int64_t>(aq[1]) - cy0;
                    const int64_t uz = static_cast<int64_t>(aq[2]) - cz0;
                    const int64_t vx = static_cast<int64_t>(bq[0]) - cx0;
                    const int64_t vy = static_cast<int64_t>(bq[1]) - cy0;
                    const int64_t vz = static_cast<int64_t>(bq[2]) - cz0;
                    const int64_t nx = uy * vz - uz * vy;
                    const int64_t ny = uz * vx - ux * vz;
                    const int64_t nz = ux * vy - uy * vx;
                    if (nx == 0 && ny == 0 && nz == 0) return true;
                }
                return false;
            };
        auto insertTailByGeometry = [&](size_t ridx, int valence) -> bool {
            if (valence < 3 || ridx >= decodedResidualsX_.size() ||
                ridx >= decodedResidualsY_.size() ||
                ridx >= decodedResidualsZ_.size()) {
                return false;
            }
            struct TailCandidate {
                MCGAL::Facet* face = nullptr;
                MCGAL::PointInt q{0, 0, 0};
                double score = std::numeric_limits<double>::infinity();
                bool projectedInside = false;
            };
            TailCandidate best;
            for (MCGAL::Facet* f : mesh_->faces()) {
                if (!f || f->isRemoved() || faceDegree(f) != valence) continue;
                std::vector<MCGAL::Vertex*> boundary = orderedFaceVertices(f);
                if (static_cast<int>(boundary.size()) != valence) continue;
                MCGAL::PointInt baryQ =
                    strictUsesAbsoluteGeometry()
                        ? MCGAL::PointInt(0, 0, 0)
                        : MCGAL::rdBarycenterQuantized(f, mesh_);
                MCGAL::PointInt vQ(
                    strictUsesAbsoluteGeometry()
                        ? decodedResidualsX_[ridx]
                        : baryQ[0] + decodedResidualsX_[ridx],
                    strictUsesAbsoluteGeometry()
                        ? decodedResidualsY_[ridx]
                        : baryQ[1] + decodedResidualsY_[ridx],
                    strictUsesAbsoluteGeometry()
                        ? decodedResidualsZ_[ridx]
                        : baryQ[2] + decodedResidualsZ_[ridx]);
                if (liveVertexExistsAtQGlobal(vQ)) continue;
                if (fanWouldDegenerateGlobal(boundary, vQ)) continue;
                bool inside = false;
                double score = 0.0;
                if (valence == 3) {
                    inside = projectedPointInTriangleQuant(mesh_, f, vQ);
                    score = pointPlaneDistance2Quant(mesh_, f, vQ);
                    if (!std::isfinite(score)) continue;
                    if (!inside && best.projectedInside) continue;
                } else {
                    score = static_cast<double>(std::abs(decodedResidualsX_[ridx]) +
                                                std::abs(decodedResidualsY_[ridx]) +
                                                std::abs(decodedResidualsZ_[ridx]));
                }
                if (!best.face || (inside && !best.projectedInside) ||
                    (inside == best.projectedInside && score < best.score)) {
                    best.face = f;
                    best.q = vQ;
                    best.score = score;
                    best.projectedInside = inside;
                }
            }
            if (!best.face || best.face->isRemoved() || !hasBoundedFaceLoop(best.face)) {
                return false;
            }
            MCGAL::Halfedge* insertGate = findLiveFaceGate(best.face);
            if (!insertGate) return false;
            std::vector<MCGAL::Vertex*> boundary = orderedFaceVertices(best.face);
            applyStrictBoundaryTags(boundary);
            MCGAL::Halfedge* newH =
                mesh_->create_center_vertex_without_init(insertGate, mesh_->intPosToFloat(best.q));
            MCGAL::Vertex* insertedCenter = newH ? newH->vertex() : nullptr;
            if (!insertedCenter || insertedCenter->isRemoved()) return false;
            indexLiveVertex(mesh_, liveVertexIndex, insertedCenter);
            for (MCGAL::Halfedge* hv : boundedVertexHalfedges(insertedCenter)) {
                MCGAL::Facet* pf = hv ? hv->face() : nullptr;
                if (pf && !pf->isRemoved()) {
                    pf->setProcessedFlag();
                    facesProcessed.insert(pf);
                }
            }
            markDecodedPatchInteriorFacesProcessed(boundary, insertedCenter, facesProcessed);
            insertedCenter->setConquered();
            ++inserted;
            ++tailGeometryFallbacks;
            return true;
        };
        if (!reverseWalk &&
            envEnabledDefault("PMSF_PCLTTM_ENABLE_TAIL_GEOM_FALLBACK", true) &&
            residualIdx < decodedResidualsX_.size()) {
            std::sort(skippedNoGateV.begin(), skippedNoGateV.end(),
                      [](const SkippedNoGateV& a, const SkippedNoGateV& b) {
                          return a.ridx < b.ridx;
                      });
            size_t skippedCursor = 0;
            while (residualIdx < decodedResidualsX_.size()) {
                while (skippedCursor < skippedNoGateV.size() &&
                       skippedNoGateV[skippedCursor].ridx < residualIdx) {
                    ++skippedCursor;
                }
                int valence = 3;
                if (skippedCursor < skippedNoGateV.size() &&
                    skippedNoGateV[skippedCursor].ridx == residualIdx) {
                    valence = skippedNoGateV[skippedCursor].valence;
                    ++skippedCursor;
                }
                if (!insertTailByGeometry(residualIdx, valence)) break;
                ++residualIdx;
            }
        }
        if (envEnabled("PMSF_BPV_DIAG")) {
            std::cerr << "[PCLTTMStrict walk decoder] consumed=" << consumed
                      << " inserted=" << inserted
                      << " residual_cursor=" << residualIdx
                      << " merged_patches=" << mergedPatches
                      << " failed_patch_discoveries=" << failedPatchDiscoveries
                      << " deferred_remaining=" << deferredReverse.size()
                      << " inferred_nulls=" << inferredPackedNulls
                      << " loose_fallbacks=" << looseFallbacks
                      << " vinput_lookups=" << vInputLookups
                      << " vinput_indexed_hits=" << vInputIndexedHits
                      << " vinput_gate_hits=" << vInputGateHits
                      << " empty_queue_reseeds=" << emptyQueueReseeds
                      << " no_gate_reset_retries=" << noGateResetRetries
                      << " no_gate_v_skips=" << skippedNoGateV.size()
                      << " tail_geom_fallbacks=" << tailGeometryFallbacks
                      << " sparse_tail_fallbacks=" << sparseTailFallbacks
                      << " si=" << si
                      << " symbols=" << decodedSymbols_.size()
                      << "\n";
        }
        return;
    }

    std::deque<MCGAL::Facet*> q;
    refillLivingFaces(mesh_, q);
    size_t residualIdx = 0;
    int added = 0;
    int mergedPatches = 0;
    unsigned short currentLayer = decodedSymbols_.front().layer;
    unsigned char currentPhase = decodedSymbols_.front().phase;
    const bool diagOn = []() {
        const char* e = std::getenv("PMSF_BPV_DIAG");
        return e && e[0] == '1';
    }();
    const bool mergeFanPatches = []() {
        const char* e = std::getenv("PMSF_PCLTTM_STRICT_DEC_MERGE_PATCH");
        return e && e[0] == '1';
    }();
    int debugLimit = -1;
    if (const char* e = std::getenv("PMSF_PCLTTM_STRICT_DEC_LIMIT")) {
        debugLimit = std::atoi(e);
    }

    for (const RDPatchSymbol& s : decodedSymbols_) {
        if (debugLimit >= 0 && added >= debugLimit) break;
        if (s.kind == RDPatchSymbolKind::NULL_PATCH) continue;
        if (s.layer != currentLayer || s.phase != currentPhase) {
            currentLayer = s.layer;
            currentPhase = s.phase;
            refillLivingFaces(mesh_, q);
        }
        const int valence = symbolValence(s.kind);
        if (valence < 3) continue;

        MCGAL::Facet* f = nullptr;
        PatchCandidate patch;
        const bool usePatchMergeForThisSymbol = mergeFanPatches && valence < 6;
        while (!q.empty()) {
            MCGAL::Facet* cand = q.front();
            q.pop_front();
            if (!cand || cand->isRemoved() || !findLiveFaceGate(cand)) continue;
            if (!usePatchMergeForThisSymbol ||
                findFanPatchContaining(cand, valence, nullptr, nullptr, -1, patch)) {
                f = cand;
                break;
            }
        }
        if (!f) break;
                MCGAL::Facet* polyFace = f;
        if (usePatchMergeForThisSymbol) {
            polyFace = mergePatchToPolygon(patch, valence);
            if (!polyFace || polyFace->isRemoved()) break;
        }
        if (!hasBoundedFaceLoop(polyFace)) {
            std::cerr << "[PCLTTMStrict decoder] invalid polygon loop before insert"
                      << " added=" << added << " valence=" << valence << "\n";
            break;
        }
        if (faceDegree(polyFace) == valence && valence > 3) ++mergedPatches;
        MCGAL::Halfedge* h = findLiveFaceGate(polyFace);
        MCGAL::PointInt baryQ = MCGAL::rdBarycenterQuantized(polyFace, mesh_);
        MCGAL::Vector3i decoded(residualIdx < decodedResidualsX_.size() ? decodedResidualsX_[residualIdx] : 0,
                                residualIdx < decodedResidualsY_.size() ? decodedResidualsY_[residualIdx] : 0,
                                residualIdx < decodedResidualsZ_.size() ? decodedResidualsZ_[residualIdx] : 0);
        MCGAL::Vector3i cart = decoded;
        MCGAL::Vector3 n = polygonNormal(polyFace);
        if (!strictUsesCartesianResiduals() && n.length() != 0 && h && h->vertex() && h->end_vertex()) {
            MCGAL::Vector3 t1, t2;
            MCGAL::determineFrenetFrame(MCGAL::Vector3(h->vertex()->point()),
                                        MCGAL::Vector3(h->end_vertex()->point()),
                                        n, t1, t2);
            if (t1.length() != 0 && t2.length() != 0) {
                cart = MCGAL::invFrenetRotation(decoded, t1, t2, n);
            }
        }
        MCGAL::PointInt vQ = strictUsesAbsoluteGeometry()
                                  ? MCGAL::PointInt(decoded[0], decoded[1], decoded[2])
                                  : MCGAL::PointInt(baryQ[0] + cart[0],
                                                    baryQ[1] + cart[1],
                                                    baryQ[2] + cart[2]);
        MCGAL::Point pnt = mesh_->intPosToFloat(vQ);
                MCGAL::Halfedge* newH = usePatchMergeForThisSymbol
                                    ? mesh_->create_center_vertex_plus(h, pnt)
                                    : mesh_->create_center_vertex_without_init(h, pnt);
        if (newH && newH->end_vertex()) {
            ++added;
            MCGAL::Vertex* cv = newH->end_vertex();
            for (MCGAL::Halfedge* hv : pmsf::boundedRing(cv)) {
                if (hv && hv->face() && !hv->face()->isRemoved()) {
                    q.push_back(hv->face());
                }
            }
            if (diagOn && added % 5000 == 0) {
                std::cerr << "[PCLTTMStrict decoder] progress inserted=" << added
                          << " residuals=" << residualIdx + 1
                          << " layer=" << s.layer
                          << " phase=" << static_cast<int>(s.phase)
                          << " q=" << q.size()
                          << "\n";
            }
        }
        ++residualIdx;
    }

    if (const char* e = std::getenv("PMSF_BPV_DIAG"); e && e[0] == '1') {
        std::cerr << "[PCLTTMStrict decoder] inserted=" << added
                  << " residuals_used=" << residualIdx
                  << " target_symbols=" << decodedSymbols_.size()
                  << " merged_patches=" << mergedPatches
                  << "\n";
    }
}

void PCLTTMStrictSymbolReadOperator::collect(
    MCGAL::Halfedge* /*seed*/, char* /*buffer*/, size_t& /*dataOffset*/,
    std::vector<MCGAL::Vertex*>& /*vertices*/,
    std::vector<MCGAL::Halfedge*>& /*halfedge*/,
    std::vector<MCGAL::Facet*>& /*facets*/) {
    throw std::runtime_error("[PCLTTMStrict decoder] use importFromRangeCodedBuffer + collectFromDecoded");
}

void PCLTTMStrictSymbolReadOperator::collect(
    std::vector<MCGAL::Halfedge*> /*seeds*/, char* /*buffer*/, size_t& /*dataOffset*/,
    std::vector<MCGAL::Vertex*>& /*vertices*/,
    std::vector<MCGAL::Halfedge*>& /*halfedge*/,
    std::vector<MCGAL::Facet*>& /*facets*/) {
    throw std::runtime_error("[PCLTTMStrict decoder] segmented collect is not supported");
}

}  // namespace pmsf
