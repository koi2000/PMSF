/*
 * @Author: koi
 * @Date: 2025-09-02 10:39:21
 * @Description:
 */
#include "BasicDeserializeOperator.h"
#include "postprocess/EdgeFlipCandidateBfs.h"
#include "entropy/RangeCoderEntropyCoder.h"
#include "common/BoundedRingTraversal.h"
#include "../common/DebugTool.h"
#include "BufferUtils.h"
#include "Configuration.h"
#include "Global.h"
#include "Halfedge.h"
#include "MeshUtils.h"
#include "Vertex.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <deque>
#include <climits>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {
constexpr int kEdgeFlipSymbolMagic = 0x45464C50;  // "EFLP"
constexpr int kEdgeFlipSymbolFormatV2 = 2;
constexpr int kEdgeFlipSymbolFormatV3 = 3;
constexpr int kEdgeFlipSymbolFormatV4 = 4;
constexpr int kEdgeFlipSymbolFormatV5 = 5;
constexpr int kEdgeFlipSymbolFormatV8 = 8;
constexpr int kEdgeFlipSymbolFormatV6 = 6;
constexpr int kEdgeFlipSymbolFormatV7 = 7;
enum class EdgeFlipPayloadKind {
    None = 0,
    LegacyV1 = 1,
    InterleavedV2 = 2,
    InterleavedV3 = 3,
    InterleavedV4 = 4,
    InterleavedV5 = 5,
    InterleavedV6 = 6,
    InterleavedV7 = 7,
};

bool edgeFlipDumpEnabled() {
    static const bool enabled = [] {
        const char* env = std::getenv("PMSF_EDGEFLIP_DEBUG_DUMP");
        return env && env[0] != '\0' && std::strcmp(env, "0") != 0 &&
               std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
    }();
    return enabled;
}

MCGAL::Halfedge* findHalfedgeBetweenVertices(MCGAL::Vertex* va, MCGAL::Vertex* vb) {
    if (!va || !vb || va->isRemoved() || vb->isRemoved()) return nullptr;
    if (MCGAL::Halfedge* h = va->find_halfedge(vb)) {
        if (!h->isRemoved()) return h;
    }
    if (MCGAL::Halfedge* h = vb->find_halfedge(va)) {
        if (!h->isRemoved()) return h;
    }
    return nullptr;
}

void assignBfsLocalVertexIds(const std::shared_ptr<MCGAL::Mesh>& mesh,
                             MCGAL::Halfedge* seed) {
    if (!mesh) return;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        v->setVid(-1);
    }
    int nextId = 0;
    std::deque<MCGAL::Vertex*> q;
    auto enqueue = [&](MCGAL::Vertex* v) {
        if (!v || v->isRemoved()) return;
        if (v->vid() >= 0) return;
        v->setVid(nextId++);
        q.push_back(v);
    };
    if (seed) {
        enqueue(seed->vertex());
        enqueue(seed->end_vertex());
    }
    if (q.empty()) {
        for (MCGAL::Vertex* v : mesh->vertices()) {
            if (!v || v->isRemoved()) continue;
            enqueue(v);
            if (!q.empty()) break;
        }
    }
    auto bfs = [&]() {
        while (!q.empty()) {
            MCGAL::Vertex* cur = q.front();
            q.pop_front();
            for (MCGAL::Halfedge* h : pmsf::boundedRing(cur)) {
                if (!h || h->isRemoved()) continue;
                MCGAL::Vertex* nb = h->end_vertex();
                if (!nb || nb->isRemoved()) continue;
                if (nb->vid() >= 0) continue;
                nb->setVid(nextId++);
                q.push_back(nb);
            }
        }
    };
    bfs();
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        if (v->vid() >= 0) continue;
        enqueue(v);
        bfs();
    }
}

std::vector<MCGAL::Vertex*> buildLocalIdMap(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    int maxId = -1;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        if (v->vid() > maxId) maxId = v->vid();
    }
    if (maxId < 0) return {};
    std::vector<MCGAL::Vertex*> idToVertex(static_cast<size_t>(maxId + 1), nullptr);
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        const int id = v->vid();
        if (id >= 0 && id < static_cast<int>(idToVertex.size())) {
            idToVertex[static_cast<size_t>(id)] = v;
        }
    }
    return idToVertex;
}

MCGAL::Halfedge* findHalfedgeByVertexIds(const std::shared_ptr<MCGAL::Mesh>& mesh, int a, int b) {
    if (!mesh) return nullptr;
    if (a < 0 || b < 0) return nullptr;
    const auto& verts = mesh->vertices();
    if (a >= static_cast<int>(verts.size()) || b >= static_cast<int>(verts.size())) return nullptr;
    MCGAL::Vertex* va = verts[static_cast<size_t>(a)];
    MCGAL::Vertex* vb = verts[static_cast<size_t>(b)];
    return findHalfedgeBetweenVertices(va, vb);
}

MCGAL::Halfedge* findHalfedgeByPoolIds(const std::shared_ptr<MCGAL::Mesh>& mesh, int a, int b) {
    if (!mesh || a < 0 || b < 0) return nullptr;
    const int meshId = mesh->meshId();
    if (meshId >= 0 &&
        meshId < static_cast<int>(MCGAL::contextPool.vpoolSizes.size()) &&
        a < MCGAL::contextPool.vpoolSizes[static_cast<size_t>(meshId)] &&
        b < MCGAL::contextPool.vpoolSizes[static_cast<size_t>(meshId)]) {
        if (MCGAL::Halfedge* h = findHalfedgeBetweenVertices(
                MCGAL::contextPool.getVertexByIndexInSubPool(meshId, a),
                MCGAL::contextPool.getVertexByIndexInSubPool(meshId, b))) {
            return h;
        }
    }
    MCGAL::Vertex* va = nullptr;
    MCGAL::Vertex* vb = nullptr;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        if (v->poolId() == a) va = v;
        if (v->poolId() == b) vb = v;
        if (va && vb) break;
    }
    return findHalfedgeBetweenVertices(va, vb);
}

MCGAL::Halfedge* findHalfedgeByLocalIds(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                        const std::vector<MCGAL::Vertex*>& idToVertex,
                                        int a, int b) {
    if (!mesh) return nullptr;
    if (a < 0 || b < 0) return nullptr;
    if (a >= static_cast<int>(idToVertex.size()) || b >= static_cast<int>(idToVertex.size())) {
        return nullptr;
    }
    MCGAL::Vertex* va = idToVertex[static_cast<size_t>(a)];
    MCGAL::Vertex* vb = idToVertex[static_cast<size_t>(b)];
    return findHalfedgeBetweenVertices(va, vb);
}

MCGAL::Halfedge* findDirectedHalfedgeByVertices(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                                MCGAL::Vertex* from,
                                                MCGAL::Vertex* to) {
    if (!mesh || !from || !to) return nullptr;
    if (from->isRemoved() || to->isRemoved()) return nullptr;
    MCGAL::Halfedge* h = from->find_halfedge(to);
    return (h && !h->isRemoved()) ? h : nullptr;
}

bool coordMatch(const std::shared_ptr<MCGAL::Mesh>& mesh,
                MCGAL::Vertex* v,
                const std::array<int, 3>& p);

void undoOneFlip(const std::shared_ptr<MCGAL::Mesh>& mesh,
                 MCGAL::Halfedge* h,
                 const EdgeFlipPostProcessOperator::CoordSymbol& sym);

void undoOneFlip(const std::shared_ptr<MCGAL::Mesh>& mesh, MCGAL::Halfedge* h) {
    if (!mesh || !h || h->isRemoved()) return;
    EdgeFlipPostProcessOperator::CoordSymbol sym;
    undoOneFlip(mesh, h, sym);
}

void undoOneFlip(const std::shared_ptr<MCGAL::Mesh>& mesh,
                 MCGAL::Halfedge* h,
                 const EdgeFlipPostProcessOperator::CoordSymbol& sym) {
    if (!mesh || !h || h->isRemoved()) return;
    MCGAL::Halfedge* ho = h->opposite();
    if (!ho || ho->isRemoved()) return;
    if (!h->face() || !ho->face() || h->face()->isRemoved() || ho->face()->isRemoved()) { return; }
    if (h->face()->facet_degree() != 3 || ho->face()->facet_degree() != 3) { return; }
    if (!h->next() || !ho->next()) return;

    MCGAL::Vertex* d = h->vertex();
    MCGAL::Vertex* c = h->end_vertex();
    MCGAL::Vertex* sideH = h->next()->end_vertex();
    MCGAL::Vertex* sideHo = ho->next()->end_vertex();
    MCGAL::Vertex* a = sideH;
    MCGAL::Vertex* b = sideHo;
    bool swappedFaces = false;
    const bool hasCoordRoles =
        !(sym.oa[0] == INT_MIN && sym.oa[1] == INT_MIN && sym.oa[2] == INT_MIN &&
          sym.ob[0] == INT_MIN && sym.ob[1] == INT_MIN && sym.ob[2] == INT_MIN);
    if (hasCoordRoles) {
        const bool direct =
            coordMatch(mesh, sideH, sym.oa) && coordMatch(mesh, sideHo, sym.ob);
        const bool swapped =
            coordMatch(mesh, sideH, sym.ob) && coordMatch(mesh, sideHo, sym.oa);
        if (direct) {
            a = sideH;
            b = sideHo;
            swappedFaces = false;
        } else if (swapped) {
            a = sideHo;
            b = sideH;
            swappedFaces = true;
        } else {
            return;
        }
    }
    if (!a || !b || !c || !d) { return; }

    MCGAL::Halfedge* h_ca = findDirectedHalfedgeByVertices(mesh, c, a);
    MCGAL::Halfedge* h_ad = findDirectedHalfedgeByVertices(mesh, a, d);
    MCGAL::Halfedge* h_bc = findDirectedHalfedgeByVertices(mesh, b, c);
    MCGAL::Halfedge* h_db = findDirectedHalfedgeByVertices(mesh, d, b);
    if (!h_ca || !h_ad || !h_bc || !h_db) { return; }

    MCGAL::Facet* f0 = swappedFaces ? ho->face() : h->face();
    MCGAL::Facet* f1 = swappedFaces ? h->face() : ho->face();

    h->setVertex(a);
    h->setEndVertex(b);
    ho->setVertex(b);
    ho->setEndVertex(a);

    h->setNext(h_bc);
    h_bc->setNext(h_ca);
    h_ca->setNext(h);

    ho->setNext(h_ad);
    h_ad->setNext(h_db);
    h_db->setNext(ho);

    auto roleToHalfedge = [&](int role) -> MCGAL::Halfedge* {
        switch (role) {
            case 0: return h;
            case 1: return h_bc;
            case 2: return h_ca;
            case 3: return ho;
            case 4: return h_ad;
            case 5: return h_db;
            default: return nullptr;
        }
    };

    f0->reset(roleToHalfedge(sym.f0Proxy) ? roleToHalfedge(sym.f0Proxy) : h);
    f1->reset(roleToHalfedge(sym.f1Proxy) ? roleToHalfedge(sym.f1Proxy) : ho);

    if (MCGAL::Halfedge* p = roleToHalfedge(sym.vaProxy)) a->setProxyHalfedge(p);
    if (MCGAL::Halfedge* p = roleToHalfedge(sym.vbProxy)) b->setProxyHalfedge(p);
    if (MCGAL::Halfedge* p = roleToHalfedge(sym.vcProxy)) c->setProxyHalfedge(p);
    if (MCGAL::Halfedge* p = roleToHalfedge(sym.vdProxy)) d->setProxyHalfedge(p);
}

bool coordMatch(const std::shared_ptr<MCGAL::Mesh>& mesh,
                MCGAL::Vertex* v,
                const std::array<int, 3>& p) {
    if (!mesh) return false;
    if (!v || v->isRemoved()) return false;
    std::array<int, 3> q;
    if (mesh->enableQuantization) {
        MCGAL::PointInt qi = mesh->floatPosToInt(v->point());
        q = {qi[0], qi[1], qi[2]};
    } else {
        constexpr double kScale = 1000000.0;
        q = {static_cast<int>(std::llround(static_cast<double>(v->x()) * kScale)),
             static_cast<int>(std::llround(static_cast<double>(v->y()) * kScale)),
             static_cast<int>(std::llround(static_cast<double>(v->z()) * kScale))};
    }
    const int tol = mesh->enableQuantization ? 0 : 100;
    return std::abs(q[0] - p[0]) <= tol &&
           std::abs(q[1] - p[1]) <= tol &&
           std::abs(q[2] - p[2]) <= tol;
}

MCGAL::Halfedge* findHalfedgeByCoordSymbol(
    const std::shared_ptr<MCGAL::Mesh>& mesh,
    const EdgeFlipPostProcessOperator::CoordSymbol& sym) {
    if (!mesh) return nullptr;
    MCGAL::Halfedge* endpointOnly = nullptr;
    int endpointOnlyCount = 0;
    const bool hasOppSymbol =
        !(sym.oa[0] == INT_MIN && sym.oa[1] == INT_MIN && sym.oa[2] == INT_MIN &&
          sym.ob[0] == INT_MIN && sym.ob[1] == INT_MIN && sym.ob[2] == INT_MIN);
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        for (MCGAL::Halfedge* h : *f) {
            if (!h || h->isRemoved()) continue;
            MCGAL::Vertex* a = h->vertex();
            MCGAL::Vertex* b = h->end_vertex();
            if (!a || !b || a->isRemoved() || b->isRemoved()) continue;
            const bool direct = coordMatch(mesh, a, sym.a) && coordMatch(mesh, b, sym.b);
            const bool reverse = coordMatch(mesh, a, sym.b) && coordMatch(mesh, b, sym.a);
            if (!direct && !reverse) continue;
            MCGAL::Halfedge* oriented = direct ? h : h->opposite();
            if (!oriented) continue;
            endpointOnly = oriented;
            ++endpointOnlyCount;
            if (!hasOppSymbol) return oriented;
            MCGAL::Halfedge* ho = oriented->opposite();
            MCGAL::Vertex* oa = oriented->next() ? oriented->next()->end_vertex() : nullptr;
            MCGAL::Vertex* ob = (ho && ho->next()) ? ho->next()->end_vertex() : nullptr;
            const bool oppDirect = coordMatch(mesh, oa, sym.oa) && coordMatch(mesh, ob, sym.ob);
            const bool oppReverse = coordMatch(mesh, oa, sym.ob) && coordMatch(mesh, ob, sym.oa);
            if (!oppDirect && !oppReverse) continue;
            return oriented;
        }
    }
    if (hasOppSymbol) return nullptr;
    if (endpointOnlyCount == 1) return endpointOnly;
    return nullptr;
}

bool isTrianglePairFlippable(MCGAL::Halfedge* h) {
    if (!h || h->isRemoved()) return false;
    MCGAL::Halfedge* ho = h->opposite();
    if (!ho || ho->isRemoved()) return false;
    MCGAL::Facet* f0 = h->face();
    MCGAL::Facet* f1 = ho->face();
    if (!f0 || !f1 || f0->isRemoved() || f1->isRemoved()) return false;
    if (f0->facet_degree() != 3 || f1->facet_degree() != 3) return false;
    if (!h->next() || !h->next()->next() || !ho->next() || !ho->next()->next()) return false;
    MCGAL::Vertex* a = h->vertex();
    MCGAL::Vertex* b = h->end_vertex();
    MCGAL::Vertex* c = h->next()->end_vertex();
    MCGAL::Vertex* d = ho->next()->end_vertex();
    if (!a || !b || !c || !d) return false;
    if (a == b || a == c || a == d || b == c || b == d || c == d) return false;
    return true;
}

std::array<long long, 3> vertexKey(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                   MCGAL::Vertex* v) {
    if (!v) return {0, 0, 0};
    if (mesh && mesh->enableQuantization) {
        MCGAL::PointInt q = mesh->floatPosToInt(v->point());
        return {static_cast<long long>(q[0]),
                static_cast<long long>(q[1]),
                static_cast<long long>(q[2])};
    }
    constexpr double kScale = 100000000.0;
    return {static_cast<long long>(std::llround(static_cast<double>(v->x()) * kScale)),
            static_cast<long long>(std::llround(static_cast<double>(v->y()) * kScale)),
            static_cast<long long>(std::llround(static_cast<double>(v->z()) * kScale))};
}

bool vertexKeyLess(const std::array<long long, 3>& lhs,
                   const std::array<long long, 3>& rhs) {
    if (lhs[0] != rhs[0]) return lhs[0] < rhs[0];
    if (lhs[1] != rhs[1]) return lhs[1] < rhs[1];
    return lhs[2] < rhs[2];
}

std::array<long long, 12> replayCandidateKey(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                             MCGAL::Halfedge* h) {
    if (!h || !h->opposite() || !h->next() || !h->opposite()->next()) return {};
    auto e0 = vertexKey(mesh, h->vertex());
    auto e1 = vertexKey(mesh, h->end_vertex());
    auto o0 = vertexKey(mesh, h->next()->end_vertex());
    auto o1 = vertexKey(mesh, h->opposite()->next()->end_vertex());
    if (vertexKeyLess(e1, e0)) std::swap(e0, e1);
    if (vertexKeyLess(o1, o0)) std::swap(o0, o1);
    return {e0[0], e0[1], e0[2],
            e1[0], e1[1], e1[2],
            o0[0], o0[1], o0[2],
            o1[0], o1[1], o1[2]};
}

std::array<long long, 6> canonicalHalfedgeKey(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                              MCGAL::Halfedge* h) {
    auto a = vertexKey(mesh, h ? h->vertex() : nullptr);
    auto b = vertexKey(mesh, h ? h->end_vertex() : nullptr);
    return {a[0], a[1], a[2], b[0], b[1], b[2]};
}

void canonicalizeTraversalProxies(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!mesh) return;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Halfedge* best = nullptr;
        for (MCGAL::Halfedge* h : *f) {
            if (!h || h->isRemoved() || !h->vertex() || h->vertex()->isRemoved()) continue;
            if (!best || canonicalHalfedgeKey(mesh, best) < canonicalHalfedgeKey(mesh, h)) {
                best = h;
            }
        }
        if (best) f->setProxyHalfedge(best);
    }
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        MCGAL::Halfedge* best = nullptr;
        for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
            if (!h || h->isRemoved() || h->vertex() != v ||
                !h->end_vertex() || h->end_vertex()->isRemoved()) {
                continue;
            }
            if (!best || canonicalHalfedgeKey(mesh, best) < canonicalHalfedgeKey(mesh, h)) {
                best = h;
            }
        }
        if (best) v->setProxyHalfedge(best);
    }
}

std::vector<MCGAL::Halfedge*> collectReplayCandidates(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    struct ReplayCandidate {
        MCGAL::Halfedge* h = nullptr;
        std::array<long long, 12> key{};
    };
    std::vector<MCGAL::Halfedge*> out;
    if (!mesh) return out;
    std::vector<ReplayCandidate> candidates;
    candidates.reserve(mesh->size_of_facets() * 2 + 8);
    std::unordered_set<std::uint64_t> seen;
    seen.reserve(mesh->size_of_facets() * 4 + 8);
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        for (MCGAL::Halfedge* h : *f) {
            if (!isTrianglePairFlippable(h)) continue;
            MCGAL::Halfedge* ho = h->opposite();
            const int hid = h->poolId();
            const int oid = ho ? ho->poolId() : -1;
            if (hid < 0 || oid < 0) continue;
            const std::uint64_t key =
                (static_cast<std::uint64_t>(std::min(hid, oid)) << 32) ^
                static_cast<std::uint32_t>(std::max(hid, oid));
            if (!seen.insert(key).second) continue;
            ReplayCandidate item;
            item.h = h;
            item.key = replayCandidateKey(mesh, h);
            candidates.push_back(item);
        }
    }
    std::stable_sort(candidates.begin(), candidates.end(),
                     [](const ReplayCandidate& lhs, const ReplayCandidate& rhs) {
                         return lhs.key < rhs.key;
                     });
    out.reserve(candidates.size());
    for (const ReplayCandidate& item : candidates) {
        out.push_back(item.h);
    }
    return out;
}

EdgeFlipPayloadKind readEdgeFlipPayload(char* buffer,
                                        size_t& dataOffset,
                                        size_t totalSize,
                                        std::vector<int>& roundCounts,
                                        std::vector<std::pair<int, int>>& symbols,
                                        std::vector<EdgeFlipPostProcessOperator::CoordSymbol>& coordSymbols,
                                        std::vector<int>& roundBitCounts,
                                        std::vector<unsigned char>& roundBits) {
    roundCounts.clear();
    symbols.clear();
    coordSymbols.clear();
    roundBitCounts.clear();
    roundBits.clear();
    if (!buffer) return EdgeFlipPayloadKind::None;
    if (dataOffset + sizeof(int) * 2 > totalSize) return EdgeFlipPayloadKind::None;
    size_t probe = dataOffset;
    const int marker = readInt(buffer, probe);
    if (marker != kEdgeFlipSymbolMagic) return EdgeFlipPayloadKind::None;

    const int markerOrCount = readInt(buffer, probe);
    if (markerOrCount == kEdgeFlipSymbolFormatV2) {
        const int numRounds = readInt(buffer, probe);
        const int n = readInt(buffer, probe);
        if (numRounds < 0 || n < 0) return EdgeFlipPayloadKind::None;
        const size_t bytesNeeded = static_cast<size_t>(numRounds) * sizeof(int) +
                                   static_cast<size_t>(n) * sizeof(int) * 2;
        if (probe + bytesNeeded > totalSize) return EdgeFlipPayloadKind::None;

        roundCounts.assign(static_cast<size_t>(numRounds), 0);
        for (int i = 0; i < numRounds; ++i) {
            roundCounts[static_cast<size_t>(i)] = readInt(buffer, probe);
        }
        symbols.reserve(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i) {
            const int a = readInt(buffer, probe);
            const int b = readInt(buffer, probe);
            symbols.emplace_back(a, b);
        }
        dataOffset = probe;
        return EdgeFlipPayloadKind::InterleavedV2;
    }

    if (markerOrCount == kEdgeFlipSymbolFormatV3) {
        const int numRounds = readInt(buffer, probe);
        const int n = readInt(buffer, probe);
        if (numRounds < 0 || n < 0) return EdgeFlipPayloadKind::None;
        const size_t bytesNeeded = static_cast<size_t>(numRounds) * sizeof(int) +
                                   static_cast<size_t>(n) * sizeof(float) * 6;
        if (probe + bytesNeeded > totalSize) return EdgeFlipPayloadKind::None;

        roundCounts.assign(static_cast<size_t>(numRounds), 0);
        for (int i = 0; i < numRounds; ++i) {
            roundCounts[static_cast<size_t>(i)] = readInt(buffer, probe);
        }
        coordSymbols.reserve(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i) {
            EdgeFlipPostProcessOperator::CoordSymbol sym;
            sym.a[0] = readFloat(buffer, probe);
            sym.a[1] = readFloat(buffer, probe);
            sym.a[2] = readFloat(buffer, probe);
            sym.b[0] = readFloat(buffer, probe);
            sym.b[1] = readFloat(buffer, probe);
            sym.b[2] = readFloat(buffer, probe);
            sym.oa = {INT_MIN, INT_MIN, INT_MIN};
            sym.ob = {INT_MIN, INT_MIN, INT_MIN};
            coordSymbols.push_back(sym);
        }
        dataOffset = probe;
        return EdgeFlipPayloadKind::InterleavedV3;
    }

    if (markerOrCount == kEdgeFlipSymbolFormatV4) {
        const int numRounds = readInt(buffer, probe);
        const int n = readInt(buffer, probe);
        if (numRounds < 0 || n < 0) return EdgeFlipPayloadKind::None;
        const size_t bytesNeeded = static_cast<size_t>(numRounds) * sizeof(int) +
                                   static_cast<size_t>(n) * sizeof(int) * 12;
        if (probe + bytesNeeded > totalSize) return EdgeFlipPayloadKind::None;

        roundCounts.assign(static_cast<size_t>(numRounds), 0);
        for (int i = 0; i < numRounds; ++i) {
            roundCounts[static_cast<size_t>(i)] = readInt(buffer, probe);
        }
        coordSymbols.reserve(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i) {
            EdgeFlipPostProcessOperator::CoordSymbol sym;
            sym.a[0] = readInt(buffer, probe);
            sym.a[1] = readInt(buffer, probe);
            sym.a[2] = readInt(buffer, probe);
            sym.b[0] = readInt(buffer, probe);
            sym.b[1] = readInt(buffer, probe);
            sym.b[2] = readInt(buffer, probe);
            sym.oa[0] = readInt(buffer, probe);
            sym.oa[1] = readInt(buffer, probe);
            sym.oa[2] = readInt(buffer, probe);
            sym.ob[0] = readInt(buffer, probe);
            sym.ob[1] = readInt(buffer, probe);
            sym.ob[2] = readInt(buffer, probe);
            coordSymbols.push_back(sym);
        }
        dataOffset = probe;
        return EdgeFlipPayloadKind::InterleavedV4;
    }

    if (markerOrCount == kEdgeFlipSymbolFormatV5) {
        const int numRounds = readInt(buffer, probe);
        const int totalBits = readInt(buffer, probe);
        if (numRounds < 0 || totalBits < 0) return EdgeFlipPayloadKind::None;
        const int packedBytes = (totalBits + 7) / 8;
        const size_t bytesNeeded = static_cast<size_t>(numRounds) * sizeof(int) +
                                   static_cast<size_t>(packedBytes);
        if (probe + bytesNeeded > totalSize) return EdgeFlipPayloadKind::None;
        roundBitCounts.assign(static_cast<size_t>(numRounds), 0);
        for (int i = 0; i < numRounds; ++i) {
            roundBitCounts[static_cast<size_t>(i)] = readInt(buffer, probe);
        }
        std::vector<unsigned char> packed(static_cast<size_t>(packedBytes), 0);
        for (int i = 0; i < packedBytes; ++i) {
            packed[static_cast<size_t>(i)] =
                static_cast<unsigned char>(readChar(buffer, probe));
        }
        roundBits.assign(static_cast<size_t>(totalBits), 0);
        for (int i = 0; i < totalBits; ++i) {
            const unsigned char byte = packed[static_cast<size_t>(i >> 3)];
            roundBits[static_cast<size_t>(i)] =
                static_cast<unsigned char>((byte >> (i & 7)) & 0x1u);
        }
        dataOffset = probe;
        return EdgeFlipPayloadKind::InterleavedV5;
    }

    if (markerOrCount == kEdgeFlipSymbolFormatV8) {
        const int numRounds = readInt(buffer, probe);
        const int totalBits = readInt(buffer, probe);
        if (numRounds < 0 || totalBits < 0) return EdgeFlipPayloadKind::None;
        roundBitCounts.assign(static_cast<size_t>(numRounds), 0);
        for (int i = 0; i < numRounds; ++i) {
            roundBitCounts[static_cast<size_t>(i)] = readInt(buffer, probe);
        }
        const int codedSize = readInt(buffer, probe);
        if (codedSize < 0 || probe + static_cast<size_t>(codedSize) > totalSize) {
            return EdgeFlipPayloadKind::None;
        }
        std::vector<std::uint32_t> decoded;
        pmsf::RangeCoderEntropyCoder coder(1);
        coder.decode(reinterpret_cast<const std::uint8_t*>(buffer + probe),
                     static_cast<size_t>(codedSize), static_cast<size_t>(totalBits), decoded);
        probe += static_cast<size_t>(codedSize);
        roundBits.assign(static_cast<size_t>(totalBits), 0);
        for (int i = 0; i < totalBits && i < static_cast<int>(decoded.size()); ++i) {
            roundBits[static_cast<size_t>(i)] = static_cast<unsigned char>(decoded[static_cast<size_t>(i)] & 1u);
        }
        dataOffset = probe;
        return EdgeFlipPayloadKind::InterleavedV5;
    }

    if (markerOrCount == kEdgeFlipSymbolFormatV6 ||
        markerOrCount == kEdgeFlipSymbolFormatV7) {
        const int numRounds = readInt(buffer, probe);
        const int totalBits = readInt(buffer, probe);
        const int n = readInt(buffer, probe);
        if (numRounds < 0 || totalBits < 0 || n < 0) return EdgeFlipPayloadKind::None;
        const int packedBytes = (totalBits + 7) / 8;
        const int coordIntsPerSymbol =
            markerOrCount == kEdgeFlipSymbolFormatV7 ? 18 : 12;
        const size_t bytesNeeded = static_cast<size_t>(numRounds) * sizeof(int) * 2 +
                                   static_cast<size_t>(packedBytes) +
                                   static_cast<size_t>(n) * sizeof(int) *
                                       static_cast<size_t>(coordIntsPerSymbol);
        if (probe + bytesNeeded > totalSize) return EdgeFlipPayloadKind::None;
        roundBitCounts.assign(static_cast<size_t>(numRounds), 0);
        for (int i = 0; i < numRounds; ++i) {
            roundBitCounts[static_cast<size_t>(i)] = readInt(buffer, probe);
        }
        roundCounts.assign(static_cast<size_t>(numRounds), 0);
        for (int i = 0; i < numRounds; ++i) {
            roundCounts[static_cast<size_t>(i)] = readInt(buffer, probe);
        }
        std::vector<unsigned char> packed(static_cast<size_t>(packedBytes), 0);
        for (int i = 0; i < packedBytes; ++i) {
            packed[static_cast<size_t>(i)] =
                static_cast<unsigned char>(readChar(buffer, probe));
        }
        roundBits.assign(static_cast<size_t>(totalBits), 0);
        for (int i = 0; i < totalBits; ++i) {
            const unsigned char byte = packed[static_cast<size_t>(i >> 3)];
            roundBits[static_cast<size_t>(i)] =
                static_cast<unsigned char>((byte >> (i & 7)) & 0x1u);
        }
        coordSymbols.reserve(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i) {
            EdgeFlipPostProcessOperator::CoordSymbol sym;
            sym.a[0] = readInt(buffer, probe);
            sym.a[1] = readInt(buffer, probe);
            sym.a[2] = readInt(buffer, probe);
            sym.b[0] = readInt(buffer, probe);
            sym.b[1] = readInt(buffer, probe);
            sym.b[2] = readInt(buffer, probe);
            sym.oa[0] = readInt(buffer, probe);
            sym.oa[1] = readInt(buffer, probe);
            sym.oa[2] = readInt(buffer, probe);
            sym.ob[0] = readInt(buffer, probe);
            sym.ob[1] = readInt(buffer, probe);
            sym.ob[2] = readInt(buffer, probe);
            if (markerOrCount == kEdgeFlipSymbolFormatV7) {
                sym.f0Proxy = readInt(buffer, probe);
                sym.f1Proxy = readInt(buffer, probe);
                sym.vaProxy = readInt(buffer, probe);
                sym.vbProxy = readInt(buffer, probe);
                sym.vcProxy = readInt(buffer, probe);
                sym.vdProxy = readInt(buffer, probe);
            }
            coordSymbols.push_back(sym);
        }
        dataOffset = probe;
        return markerOrCount == kEdgeFlipSymbolFormatV7
                   ? EdgeFlipPayloadKind::InterleavedV7
                   : EdgeFlipPayloadKind::InterleavedV6;
    }

    const int n = markerOrCount;
    if (n < 0) return EdgeFlipPayloadKind::None;
    const size_t payloadBytes = static_cast<size_t>(n) * sizeof(int) * 2;
    if (probe + payloadBytes > totalSize) return EdgeFlipPayloadKind::None;

    symbols.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        const int a = readInt(buffer, probe);
        const int b = readInt(buffer, probe);
        symbols.emplace_back(a, b);
    }
    dataOffset = probe;
    return EdgeFlipPayloadKind::LegacyV1;
}

}  // namespace

// void BasicDeserializeOperator::init(std::string path) {
//     DeserializeOperator::init(path);
//     // int symbolSize = readInt(buffer_, dataOffset_);
//     // pointOffset_ = symbolSize + dataOffset_;
//     // symbolReadOperator_->init({pointOffset_});
// }

std::shared_ptr<MCGAL::Mesh> BasicDeserializeOperator::deserializeBaseMesh() {
    

    std::shared_ptr<MCGAL::Mesh> mesh;
    if (header_.isEnableQuantization()) {
        const size_t quantMetadataOffset = dataOffset_;
        mesh = MCGAL::readBaseMeshWithQuantization(DEFAULT_MESH_ID, buffer_, dataOffset_);
        if (mesh) {
            mesh->i_nbQuantBits =
                static_cast<unsigned char>(buffer_[quantMetadataOffset]);
        }
    } else {
        mesh = MCGAL::readBaseMesh(DEFAULT_MESH_ID, buffer_, dataOffset_);
    }

    // find seed
    MCGAL::Vertex* v0 = mesh->vertices()[0];
    MCGAL::Vertex* v1 = mesh->vertices()[1];
    for (MCGAL::Halfedge* hit : pmsf::boundedRing(v0)) {
        if (hit->end_vertex() == v1) {
            seed = hit;
            break;
        }
    }
    this->mesh_ = mesh;
    reconstructOperator_->init(mesh);

    std::vector<int> parsedRoundCounts;
    std::vector<std::pair<int, int>> parsedSymbols;
    std::vector<EdgeFlipPostProcessOperator::CoordSymbol> parsedCoordSymbols;
    std::vector<int> parsedRoundBitCounts;
    std::vector<unsigned char> parsedRoundBits;
    const EdgeFlipPayloadKind payloadKind =
        readEdgeFlipPayload(buffer_, dataOffset_, totalSize, parsedRoundCounts, parsedSymbols,
                            parsedCoordSymbols, parsedRoundBitCounts, parsedRoundBits);
    if (payloadKind == EdgeFlipPayloadKind::InterleavedV2) {
        hasInterleavedEdgeFlipPayload_ = true;
        edgeFlipDecodedRounds_ = 0;
        edgeFlipRoundCounts_ = std::move(parsedRoundCounts);
        edgeFlipSymbols_ = std::move(parsedSymbols);
        edgeFlipCoordSymbols_.clear();
        edgeFlipPrefix_.assign(edgeFlipRoundCounts_.size() + 1, 0);
        for (size_t i = 0; i < edgeFlipRoundCounts_.size(); ++i) {
            int c = edgeFlipRoundCounts_[i];
            if (c < 0) c = 0;
            edgeFlipPrefix_[i + 1] = edgeFlipPrefix_[i] + c;
        }
    } else if (payloadKind == EdgeFlipPayloadKind::InterleavedV3 ||
               payloadKind == EdgeFlipPayloadKind::InterleavedV4) {
        hasInterleavedEdgeFlipPayload_ = true;
        edgeFlipDecodedRounds_ = 0;
        edgeFlipRoundCounts_ = std::move(parsedRoundCounts);
        edgeFlipCoordSymbols_ = std::move(parsedCoordSymbols);
        edgeFlipSymbols_.clear();
        edgeFlipPrefix_.assign(edgeFlipRoundCounts_.size() + 1, 0);
        for (size_t i = 0; i < edgeFlipRoundCounts_.size(); ++i) {
            int c = edgeFlipRoundCounts_[i];
            if (c < 0) c = 0;
            edgeFlipPrefix_[i + 1] = edgeFlipPrefix_[i] + c;
        }
    } else if (payloadKind == EdgeFlipPayloadKind::InterleavedV5 ||
               payloadKind == EdgeFlipPayloadKind::InterleavedV6 ||
               payloadKind == EdgeFlipPayloadKind::InterleavedV7) {
        setenv("PMSF_EDGEFLIP_STABLE_BFS", "1", 1);
        hasInterleavedEdgeFlipPayload_ = true;
        edgeFlipDecodedRounds_ = 0;
        edgeFlipRoundCounts_ = std::move(parsedRoundCounts);
        edgeFlipCoordSymbols_ = std::move(parsedCoordSymbols);
        edgeFlipRoundBitCounts_ = std::move(parsedRoundBitCounts);
        edgeFlipRoundBits_ = std::move(parsedRoundBits);
        edgeFlipRoundBitPrefix_.assign(edgeFlipRoundBitCounts_.size() + 1, 0);
        for (size_t i = 0; i < edgeFlipRoundBitCounts_.size(); ++i) {
            int c = edgeFlipRoundBitCounts_[i];
            if (c < 0) c = 0;
            edgeFlipRoundBitPrefix_[i + 1] = edgeFlipRoundBitPrefix_[i] + c;
        }
        edgeFlipPrefix_.assign(edgeFlipRoundCounts_.size() + 1, 0);
        for (size_t i = 0; i < edgeFlipRoundCounts_.size(); ++i) {
            int c = edgeFlipRoundCounts_[i];
            if (c < 0) c = 0;
            edgeFlipPrefix_[i + 1] = edgeFlipPrefix_[i] + c;
        }
    } else if (payloadKind == EdgeFlipPayloadKind::LegacyV1) {
        if (edgeFlipDumpEnabled()) {
            std::filesystem::create_directories("build/aaaaresult/decompress/flip");
            mesh->dumpto_oldtype("build/aaaaresult/decompress/flip/undo_before.off");
        }
        for (int i = static_cast<int>(parsedSymbols.size()) - 1; i >= 0; --i) {
            const auto& s = parsedSymbols[static_cast<size_t>(i)];
            undoOneFlip(mesh, findHalfedgeByVertexIds(mesh, s.first, s.second));
        }
        if (edgeFlipDumpEnabled()) {
            mesh->dumpto_oldtype("build/aaaaresult/decompress/flip/undo_after.off");
        }
    }

    int symbolSize = readInt(buffer_, dataOffset_);
    pointOffset_ = symbolSize + dataOffset_;
    symbolReadOperator_->init({pointOffset_}, header_);

    if (header_.isEnableCompress()) {
        char* symbolBuffer = new char[BUFFER_SIZE];
        char* pointBuffer = new char[BUFFER_SIZE];
        int realSymbolSize = 0;
        int realPointSize = 0;
        deserializeCharPointer(buffer_, dataOffset_, symbolBuffer, realSymbolSize);
        deserializeCharPointer(buffer_, dataOffset_ + symbolSize, pointBuffer, realPointSize);
        delete[] buffer_;
        char* newBuffer = new char[realSymbolSize + realPointSize];
        std::memcpy(newBuffer, symbolBuffer, realSymbolSize);
        std::memcpy(newBuffer + realSymbolSize, pointBuffer, realPointSize);
        delete[] symbolBuffer;
        delete[] pointBuffer;
        buffer_ = newBuffer;
        dataOffset_ = 0;
        symbolReadOperator_->init({(size_t)realSymbolSize}, header_);
    }
    return mesh;
}

void BasicDeserializeOperator::deserializeOneRound() {
    static int s_round = 0;
    const bool dbg = []() {
        const char* v = std::getenv("PMSF_EDGEFLIP_DEBUG");
        return v && v[0] == '1';
    }();
    if (dbg) {
        std::fprintf(stderr, "[edgeflip-debug][basic] round=%d begin\n", s_round);
    }
    std::vector<std::pair<MCGAL::Halfedge*, EdgeFlipPostProcessOperator::CoordSymbol>>
        pendingEdgeFlipUndoTargets;
    int pendingEdgeFlipCompressRound = -1;
    const int interleavedRounds = !edgeFlipRoundBitCounts_.empty()
                                      ? static_cast<int>(edgeFlipRoundBitCounts_.size())
                                      : static_cast<int>(edgeFlipRoundCounts_.size());
    if (hasInterleavedEdgeFlipPayload_ &&
        edgeFlipDecodedRounds_ < interleavedRounds) {
        if (!edgeFlipRoundBitCounts_.empty()) {
            const int numRounds = static_cast<int>(edgeFlipRoundBitCounts_.size());
            const int compressRound = numRounds - 1 - edgeFlipDecodedRounds_;
            if (compressRound >= 0 &&
                compressRound + 1 < static_cast<int>(edgeFlipRoundBitPrefix_.size())) {
                pendingEdgeFlipCompressRound = compressRound;
                const int startBit = edgeFlipRoundBitPrefix_[static_cast<size_t>(compressRound)];
                const int endBit = edgeFlipRoundBitPrefix_[static_cast<size_t>(compressRound + 1)];
                const int expected = endBit - startBit;
                const std::vector<MCGAL::Halfedge*> replayEdges =
                    pmsf::edgeflip::collectEdgeFlipCandidatesBfs(mesh_);
                const int n = std::min(expected, static_cast<int>(replayEdges.size()));
                std::vector<std::pair<MCGAL::Halfedge*, EdgeFlipPostProcessOperator::CoordSymbol>>
                    undoTargets;
                if (!edgeFlipCoordSymbols_.empty() &&
                    compressRound + 1 < static_cast<int>(edgeFlipPrefix_.size())) {
                    const int startIdx = edgeFlipPrefix_[static_cast<size_t>(compressRound)];
                    const int endIdx = std::min(edgeFlipPrefix_[static_cast<size_t>(compressRound + 1)],
                                                static_cast<int>(edgeFlipCoordSymbols_.size()));
                    undoTargets.reserve(static_cast<size_t>(std::max(0, endIdx - startIdx)));
                    auto orientCandidate = [&](MCGAL::Halfedge* candidate,
                                               const EdgeFlipPostProcessOperator::CoordSymbol& sym)
                        -> MCGAL::Halfedge* {
                        auto oriented = [&](MCGAL::Halfedge* e) -> MCGAL::Halfedge* {
                            if (!e || !e->opposite() || !e->next() || !e->opposite()->next()) return nullptr;
                            if (!coordMatch(mesh_, e->vertex(), sym.a) ||
                                !coordMatch(mesh_, e->end_vertex(), sym.b)) return nullptr;
                            MCGAL::Vertex* oa = e->next()->end_vertex();
                            MCGAL::Vertex* ob = e->opposite()->next()->end_vertex();
                            const bool direct =
                                coordMatch(mesh_, oa, sym.oa) && coordMatch(mesh_, ob, sym.ob);
                            const bool swapped =
                                coordMatch(mesh_, oa, sym.ob) && coordMatch(mesh_, ob, sym.oa);
                            return (direct || swapped) ? e : nullptr;
                        };
                        if (MCGAL::Halfedge* h = oriented(candidate)) return h;
                        if (MCGAL::Halfedge* h = oriented(candidate ? candidate->opposite() : nullptr)) return h;
                        return findHalfedgeByCoordSymbol(mesh_, sym);
                    };
                    int symbolIdx = endIdx - 1;
                    for (int i = n - 1; i >= 0 && symbolIdx >= startIdx; --i) {
                        const int bitIdx = startBit + i;
                        if (bitIdx < 0 || bitIdx >= static_cast<int>(edgeFlipRoundBits_.size())) continue;
                        if (edgeFlipRoundBits_[static_cast<size_t>(bitIdx)] == 0) continue;
                        const auto& sym =
                            edgeFlipCoordSymbols_[static_cast<size_t>(symbolIdx--)];
                        undoTargets.emplace_back(
                            orientCandidate(replayEdges[static_cast<size_t>(i)], sym), sym);
                    }
                } else {
                    undoTargets.reserve(static_cast<size_t>(n));
                    for (int i = 0; i < n; ++i) {
                        const int bitIdx = startBit + i;
                        if (bitIdx < 0 || bitIdx >= static_cast<int>(edgeFlipRoundBits_.size())) break;
                        if (edgeFlipRoundBits_[static_cast<size_t>(bitIdx)] == 0) continue;
                        undoTargets.emplace_back(replayEdges[static_cast<size_t>(i)],
                                                 EdgeFlipPostProcessOperator::CoordSymbol{});
                    }
                }
                pendingEdgeFlipUndoTargets = std::move(undoTargets);
            }
            ++edgeFlipDecodedRounds_;
            if (dbg) {
                std::fprintf(stderr,
                             "[edgeflip-debug][basic] round=%d undo_done decoded=%d\n",
                             s_round, edgeFlipDecodedRounds_);
            }
        } else {
        const int numRounds = static_cast<int>(edgeFlipRoundCounts_.size());
        const int compressRound = numRounds - 1 - edgeFlipDecodedRounds_;
        if (compressRound >= 0 &&
            compressRound + 1 < static_cast<int>(edgeFlipPrefix_.size())) {
            pendingEdgeFlipCompressRound = compressRound;
            const int startIdx = edgeFlipPrefix_[static_cast<size_t>(compressRound)];
            const int endIdx = edgeFlipPrefix_[static_cast<size_t>(compressRound + 1)];

            char beforePath[512];
            char afterPath[512];
            if (edgeFlipDumpEnabled()) {
                std::filesystem::create_directories("build/aaaaresult/decompress/flip");
                std::snprintf(beforePath, sizeof(beforePath),
                              "build/aaaaresult/decompress/flip/undo_round_%03d_before.off",
                              compressRound);
                std::snprintf(afterPath, sizeof(afterPath),
                              "build/aaaaresult/decompress/flip/undo_round_%03d_after.off",
                              compressRound);
                mesh_->dumpto_oldtype(beforePath);
            }

            if (!edgeFlipCoordSymbols_.empty()) {
                const int symbolEnd =
                    std::min(endIdx, static_cast<int>(edgeFlipCoordSymbols_.size()));
                for (int i = symbolEnd - 1; i >= startIdx; --i) {
                    const auto& s = edgeFlipCoordSymbols_[static_cast<size_t>(i)];
                    undoOneFlip(mesh_, findHalfedgeByCoordSymbol(mesh_, s));
                }
            } else {
                assignBfsLocalVertexIds(mesh_, seed);
                const std::vector<MCGAL::Vertex*> idToVertex = buildLocalIdMap(mesh_);
                const int symbolEnd = std::min(endIdx, static_cast<int>(edgeFlipSymbols_.size()));
                for (int i = symbolEnd - 1; i >= startIdx; --i) {
                    const auto& s = edgeFlipSymbols_[static_cast<size_t>(i)];
                    MCGAL::Halfedge* h = findHalfedgeByPoolIds(mesh_, s.first, s.second);
                    if (!h) {
                        h = findHalfedgeByLocalIds(mesh_, idToVertex, s.first, s.second);
                    }
                    if (!h) h = findHalfedgeByVertexIds(mesh_, s.first, s.second);
                    undoOneFlip(mesh_, h);
                }
            }
            if (edgeFlipDumpEnabled()) {
                mesh_->dumpto_oldtype(afterPath);
            }
        }
        ++edgeFlipDecodedRounds_;
        if (dbg) {
            std::fprintf(stderr,
                         "[edgeflip-debug][basic] round=%d undo_done decoded=%d\n",
                         s_round, edgeFlipDecodedRounds_);
        }
        }
    }

    if (pendingEdgeFlipCompressRound >= 0) {
        char beforePath[512];
        char afterPath[512];
        if (edgeFlipDumpEnabled()) {
            std::filesystem::create_directories("build/aaaaresult/decompress/flip");
            std::snprintf(beforePath, sizeof(beforePath),
                          "build/aaaaresult/decompress/flip/undo_round_%03d_before.off",
                          pendingEdgeFlipCompressRound);
            std::snprintf(afterPath, sizeof(afterPath),
                          "build/aaaaresult/decompress/flip/undo_round_%03d_after.off",
                          pendingEdgeFlipCompressRound);
            mesh_->dumpto_oldtype(beforePath);
        }
        for (const auto& item : pendingEdgeFlipUndoTargets) {
            undoOneFlip(mesh_, item.first, item.second);
        }
        if (edgeFlipDumpEnabled()) {
            mesh_->dumpto_oldtype(afterPath);
        }
    }

    std::vector<MCGAL::Vertex*> vertices;
    std::vector<MCGAL::Halfedge*> halfedges;
    std::vector<MCGAL::Facet*> facets;
    if (header_.isEnableQuantization()) {
        symbolReadOperator_->enableQuantization(mesh_->bboxMin, mesh_->f_quantStep);
    }
    symbolReadOperator_->collect(seed, buffer_, dataOffset_, vertices, halfedges, facets);
    reconstructOperator_->reconstruct(vertices, halfedges, facets);
    if (pendingEdgeFlipCompressRound >= 0 && edgeFlipDumpEnabled()) {
        std::filesystem::create_directories("build/aaaaresult/decompress/rounds");
        char roundPath[512];
        std::snprintf(roundPath, sizeof(roundPath),
                      "build/aaaaresult/decompress/rounds/round_%03d_after_reconstruct.off",
                      pendingEdgeFlipCompressRound);
        mesh_->dumpto_oldtype(roundPath);
    }
    if (dbg) {
        std::fprintf(stderr, "[edgeflip-debug][basic] round=%d reconstruct_done\n", s_round);
    }
    ++s_round;
}

std::shared_ptr<MCGAL::Mesh> BasicDeserializeOperator::exportMesh() {
    return mesh_;
}
