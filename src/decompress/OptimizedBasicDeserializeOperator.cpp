/*
 * @Author: koi
 * @Date: 2025-09-02 10:39:21
 * @Description:
 */
#include "reconstruct/ReconstructToggles.h"
#include "OptimizedBasicDeserializeOperator.h"
#include "boundary/SingleCopyBoundaryStage.h"
#include "boundary/SingleCopyBoundaryCompact.h"
#include "boundary/SingleCopyBoundaryAnchor.h"
#include "postprocess/EdgeFlipCandidateBfs.h"
#include "common/MeshSplitUtils.h"
#include "boundary/BoundaryCompactStorage.h"
#include "boundary/BoundaryIdRegistry.h"
#include "boundary/BndCompactCodec.h"
#include "boundary/BoundaryTopologyPatchCodec.h"
#include "common/BfsGroupLabel.h"
#include "common/BoundaryPatchCodec.h"
#include "common/BoundaryOrderUtils.h"
#include "boundary/SeamCutOperator.h"
#include "entropy/RangeCoderEntropyCoder.h"

static void canonicalizeDecoderProxies(const std::shared_ptr<MCGAL::Mesh>& mesh);
static void canonicalizeDecoderProxiesDirected(const std::shared_ptr<MCGAL::Mesh>& mesh);
// Canonical quantized position of vertices recreated by the single-copy
// boundary inverse (position round-trip floatPosToInt(intPosToFloat(q)) is not
// always exactly q, so later inverse steps must key by the stored q).
static pmsf::SBCanonicalPosMap g_scCanonicalPos;
static std::vector<pmsf::ScAnchorDef> g_scDecodedAnchors;
static std::unordered_map<int, std::array<int, 6>> g_scSeedByRound;
static std::vector<MCGAL::Vertex*> g_scBaseVertexOrder;
#include "common/BoundedRingTraversal.h"
#include "../common/DebugTool.h"
#include "BufferUtils.h"
#include "Configuration.h"
#include "Global.h"
#include "Halfedge.h"
#include "MeshUtils.h"
#include "OptimizedSymbolReadOperator.h"
#include "Vertex.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
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
int g_bndV3RetryStrat = -1;  // round-retry strategy override (decoder-only)
constexpr int kEdgeFlipSymbolMagic = 0x45464C50;  // "EFLP"
constexpr int kEdgeFlipSymbolFormatV2 = 2;
constexpr int kEdgeFlipSymbolFormatV3 = 3;
constexpr int kEdgeFlipSymbolFormatV4 = 4;
constexpr int kEdgeFlipSymbolFormatV5 = 5;
constexpr int kEdgeFlipSymbolFormatV8 = 8;
constexpr int kEdgeFlipSymbolFormatV9 = 9;
constexpr int kBoundaryBitMagic = 0x31444E42;
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
    InterleavedV8 = 8,
    InterleavedV7 = 7,
    InterleavedV9 = 9,  // canonical (q-sorted) all-edge one-bit stream
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
    EdgeFlipPostProcessOperator::CoordSymbol sym;
    undoOneFlip(mesh, h, sym);
}

void undoOneFlip(const std::shared_ptr<MCGAL::Mesh>& mesh,
                 MCGAL::Halfedge* h,
                 const EdgeFlipPostProcessOperator::CoordSymbol& sym) {
    if (!mesh || !h || h->isRemoved()) return;
    MCGAL::Halfedge* ho = h->opposite();
    if (!ho || ho->isRemoved()) return;
    if (!h->face() || !ho->face() || h->face()->isRemoved() || ho->face()->isRemoved()) return;
    if (h->face()->facet_degree() != 3 || ho->face()->facet_degree() != 3) return;
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
    if (!a || !b || !c || !d) return;

    MCGAL::Halfedge* h_ca = findDirectedHalfedgeByVertices(mesh, c, a);
    MCGAL::Halfedge* h_ad = findDirectedHalfedgeByVertices(mesh, a, d);
    MCGAL::Halfedge* h_bc = findDirectedHalfedgeByVertices(mesh, b, c);
    MCGAL::Halfedge* h_db = findDirectedHalfedgeByVertices(mesh, d, b);
    if (!h_ca || !h_ad || !h_bc || !h_db) return;

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

// Locate a live halfedge by its two endpoint coordinates.  Used to undo a recorded
// edge flip when the replay-edge order is not reproducible (segmented decoder).

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
        return EdgeFlipPayloadKind::InterleavedV8;
    }

    if (markerOrCount == kEdgeFlipSymbolFormatV9) {
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
            roundBits[static_cast<size_t>(i)] =
                static_cast<unsigned char>(decoded[static_cast<size_t>(i)] & 1u);
        }
        dataOffset = probe;
        return EdgeFlipPayloadKind::InterleavedV9;
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


namespace {
bool readBoundaryBitPayload(const char* buffer, size_t totalSize, size_t& dataOffset,
                            std::vector<int>& roundBitCounts,
                            std::vector<unsigned char>& roundBits,
                            std::vector<int>& roundResiduals,
                            int& applyMode) {
    size_t probe = dataOffset;
    if (probe + sizeof(int) > totalSize) return false;
    const int magic = readInt(const_cast<char*>(buffer), probe);
    if (magic != kBoundaryBitMagic) return false;
    const int version = readInt(const_cast<char*>(buffer), probe);
    if (version < 1 || version > 4) return false;
    const int numRounds = readInt(const_cast<char*>(buffer), probe);
    if (version >= 3) applyMode = readInt(const_cast<char*>(buffer), probe);
    const int totalBits = readInt(const_cast<char*>(buffer), probe);
    if (numRounds < 0 || totalBits < 0) return false;
    roundBitCounts.assign(static_cast<size_t>(numRounds), 0);
    for (int i = 0; i < numRounds; ++i) roundBitCounts[static_cast<size_t>(i)] = readInt(const_cast<char*>(buffer), probe);
    const int codedSize = readInt(const_cast<char*>(buffer), probe);
    if (codedSize < 0 || probe + static_cast<size_t>(codedSize) > totalSize) return false;
    std::vector<std::uint32_t> decoded;
    pmsf::RangeCoderEntropyCoder coder(1);
    coder.decode(reinterpret_cast<const std::uint8_t*>(buffer + probe), static_cast<size_t>(codedSize),
                 static_cast<size_t>(totalBits), decoded);
    probe += static_cast<size_t>(codedSize);
    roundBits.assign(static_cast<size_t>(totalBits), 0);
    for (int i = 0; i < totalBits && i < static_cast<int>(decoded.size()); ++i) {
        roundBits[static_cast<size_t>(i)] = static_cast<unsigned char>(decoded[static_cast<size_t>(i)] & 1u);
    }
    if (version >= 2) {
        const int residualInts = readInt(const_cast<char*>(buffer), probe);
        if (residualInts < 0) return false;
        roundResiduals.assign(static_cast<size_t>(residualInts), 0);
        if (applyMode == 8 || applyMode == 23 || applyMode == 30 || applyMode == 31 ||
            applyMode == 40 || applyMode == 41 || applyMode == 42 || applyMode == 50 ||
            applyMode == 13 || applyMode == 61 || applyMode == 62 || applyMode == 63 ||
            applyMode == 64 || applyMode == 65 || applyMode == 66 || applyMode == 70) {
            for (int i = 0; i < residualInts; ++i) {
                std::uint32_t z = 0;
                int shift = 0;
                for (int guard = 0; guard < 5; ++guard) {
                    if (probe >= totalSize) return false;
                    const std::uint8_t b = static_cast<std::uint8_t>(buffer[probe++]);
                    z |= static_cast<std::uint32_t>(b & 0x7Fu) << shift;
                    if ((b & 0x80u) == 0) break;
                    shift += 7;
                }
                roundResiduals[static_cast<size_t>(i)] =
                    static_cast<int>((z >> 1) ^ static_cast<std::uint32_t>(-static_cast<int>(z & 1u)));
            }
        } else {
            if (probe + static_cast<size_t>(residualInts) * sizeof(int) > totalSize) return false;
            for (int i = 0; i < residualInts; ++i)
                roundResiduals[static_cast<size_t>(i)] = readInt(const_cast<char*>(buffer), probe);
        }
        if (version == 4) {
            // Expand the compact mode-64 stream back to the canonical len=7 layout
            // used by the inverse codec.
            std::vector<int> expanded;
            int compactOff = 0;
            for (int r = 0; r < numRounds; ++r) {
                const int compactCount = roundBitCounts[static_cast<size_t>(r)];
                if (compactCount < 0 || (compactCount % 7) != 0 ||
                    compactOff + compactCount > static_cast<int>(roundResiduals.size()))
                    return false;
                const int ops = compactCount / 7;
                roundBitCounts[static_cast<size_t>(r)] = ops * 8;
                for (int o = 0; o < ops; ++o) {
                    const int base = compactOff + o * 7;
                    const int loopIdx = roundResiduals[static_cast<size_t>(base)];
                    const int loopOff = roundResiduals[static_cast<size_t>(base + 1)];
                    const int gA = roundResiduals[static_cast<size_t>(base + 2)];
                    const int pack = roundResiduals[static_cast<size_t>(base + 3)];
                    int oldPack = 0;
                    if (pack >= 0 && pack < 64) {
                        const int kA = (pack >> 3) + 2;
                        const int kB = (pack & 7) + 2;
                        oldPack = kA * 64 + kB;
                    } else {
                        oldPack = pack - 64;
                    }
                    expanded.push_back(7);
                    expanded.push_back(loopIdx);
                    expanded.push_back(loopOff);
                    expanded.push_back(gA);
                    expanded.push_back(oldPack);
                    expanded.push_back(roundResiduals[static_cast<size_t>(base + 4)]);
                    expanded.push_back(roundResiduals[static_cast<size_t>(base + 5)]);
                    expanded.push_back(roundResiduals[static_cast<size_t>(base + 6)]);
                }
                compactOff += compactCount;
            }
            if (compactOff != static_cast<int>(roundResiduals.size())) return false;
            roundResiduals.swap(expanded);
        }
    }
    dataOffset = probe;
    return true;
}
}  // namespace

static std::shared_ptr<MCGAL::Mesh> readSeamDedupBaseMesh(char* buffer, size_t& dataOffset) {
    const char nbQuantBits = readChar(buffer, dataOffset);
    MCGAL::Point bboxMin = readPoint(buffer, dataOffset);
    const float f_quantStep = readFloat(buffer, dataOffset);
    const unsigned V = static_cast<unsigned>(readInt(buffer, dataOffset));
    const unsigned F = static_cast<unsigned>(readInt(buffer, dataOffset));

    auto* points = new std::deque<MCGAL::Point>();
    auto* faces = new std::deque<uint32_t*>();

    unsigned bitOffset = 0;
    auto readQPoint = [&]() {
        int p[3];
        for (int i = 0; i < 3; ++i) p[i] = static_cast<int>(readBitsOff(nbQuantBits, buffer, bitOffset, dataOffset));
        MCGAL::Point pos;
        for (int i = 0; i < 3; ++i) pos[i] = bboxMin[i] + (p[i] + 0.5f) * f_quantStep;
        return pos;
    };
    // seed vertices
    points->push_back(readQPoint());
    points->push_back(readQPoint());
    if (bitOffset != 0) { bitOffset = 0; ++dataOffset; }
    const int numDup = readInt(buffer, dataOffset);
    auto readVarint = [&]() -> int {
        std::uint32_t z = 0;
        int shift = 0;
        for (int guard = 0; guard < 5; ++guard) {
            const std::uint8_t b = static_cast<std::uint8_t>(readChar(buffer, dataOffset));
            z |= static_cast<std::uint32_t>(b & 0x7Fu) << shift;
            if ((b & 0x80u) == 0) break;
            shift += 7;
        }
        return static_cast<int>((z >> 1) ^ static_cast<std::uint32_t>(-static_cast<int>(z & 1u)));
    };
    std::vector<unsigned char> isDup(V > 2 ? V - 2 : 0, 0);
    int prevOrd = -1;
    for (int i = 0; i < numDup; ++i) {
        const int ord = prevOrd + readVarint();
        prevOrd = ord;
        if (ord >= 0 && ord < static_cast<int>(isDup.size())) isDup[static_cast<size_t>(ord)] = 1;
    }
    MCGAL::Point prevPoint = points->at(1);
    for (unsigned ord = 0; ord + 2 < V; ++ord) {
        MCGAL::Point p;
        if (isDup[ord]) {
            p = prevPoint;
        } else {
            p = readQPoint();
            prevPoint = p;
        }
        points->push_back(p);
    }
    bitOffset = 0;
    if (V > 2 && numDup < static_cast<int>(V - 2)) ++dataOffset;

    ++dataOffset;
    const char connBit = readChar(buffer, dataOffset);
    for (unsigned i = 0; i < F; ++i) {
        uint32_t* f = new uint32_t[3];
        for (int j = 0; j < 3; ++j) f[j] = readBitsOff(connBit, buffer, bitOffset, dataOffset);
        faces->push_back(f);
    }
    bitOffset = 0;
    ++dataOffset;

    if (std::getenv("PMSF_SEAM_DEBUG")) {
        unsigned maxIdx = 0;
        for (auto* f : *faces)
            for (int j = 0; j < 3; ++j) maxIdx = std::max(maxIdx, f[j]);
        std::fprintf(stderr,
                     "[seam-dedup-dec] V=%u F=%u numDup=%d points=%zu faces=%zu connBit=%d "
                     "maxFaceIdx=%u\n",
                     V, F, numDup, points->size(), faces->size(), static_cast<int>(connBit),
                     maxIdx);
    }
    auto mesh = MCGAL::buildFromBuffer(DEFAULT_MESH_ID, points, faces);
    if (mesh) {
        mesh->enableQuantization = true;
        mesh->i_nbQuantBits = static_cast<unsigned char>(nbQuantBits);
        mesh->bboxMin = bboxMin;
        mesh->f_quantStep = f_quantStep;
    }
    for (unsigned i = 0; i < faces->size(); ++i) delete[] faces->at(i);
    delete faces;
    delete points;
    return mesh;
}

// Peek the segmented blob prefix and apply its base-facet -> group RLE to a
// shared (not yet seam-cut) base mesh.  The main importBlob() later re-reads the
// same prefix for the cut mesh; this pre-pass exists only so the shared base can
// be split into per-group components before symbol readers are initialized.
static bool peekApplySegmentedBaseGroups(char* blob, size_t size,
                                         const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (std::getenv("PMSF_SEAM_DEBUG")) { int g=0; if (blob && size>=4) std::memcpy(&g, blob, 4); std::cerr << "[peek-entry] blob=" << (void*)blob << " size=" << size << " g=" << g << std::endl; }
    if (!blob || !mesh || size < 3 * sizeof(int)) return false;
    auto i32 = [&](size_t at, int& out) -> bool {
        if (at + sizeof(int) > size) return false;
        std::memcpy(&out, blob + at, sizeof(int));
        return true;
    };
    int groupCount = 0;
    if (!i32(0, groupCount) || groupCount < 1 || groupCount > 65536) return false;
    const size_t meta = 2 * sizeof(int) + static_cast<size_t>(groupCount) * 3 * sizeof(int);
    if (std::getenv("PMSF_SEAM_DEBUG")) { int a=0,b=0; i32(0,a); i32(4,b); std::cerr << "[peek] G=" << a << " first=" << b << " size=" << size << " meta=" << meta << std::endl; }
    int baseFacetCount = 0, runCount = 0;
    if (!i32(meta, baseFacetCount) || !i32(meta + sizeof(int), runCount) ||
        runCount < 1 || runCount > (1 << 22)) {
        return false;
    }
    size_t at = meta + 2 * sizeof(int);
    std::vector<int> groupOfFacet;
    groupOfFacet.reserve(static_cast<size_t>(baseFacetCount));
    for (int r = 0; r < runCount; ++r) {
        if (at + 5 > size) return false;
        std::int8_t gid = 0;
        std::memcpy(&gid, blob + at, sizeof(gid));
        at += sizeof(gid);
        int runLen = 0;
        std::memcpy(&runLen, blob + at, sizeof(int));
        at += sizeof(int);
        if (gid < 0 || static_cast<int>(gid) >= groupCount || runLen < 1) return false;
        for (int i = 0; i < runLen; ++i) groupOfFacet.push_back(static_cast<int>(gid));
    }
    if (std::getenv("PMSF_SEAM_DEBUG")) std::cerr << "[peek] baseF=" << baseFacetCount << " runs=" << runCount << " parsed=" << groupOfFacet.size() << " at=" << at << std::endl;
    if (static_cast<int>(groupOfFacet.size()) != baseFacetCount) return false;
    int fi = 0;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        if (fi >= baseFacetCount) break;
        f->setGroupId(groupOfFacet[static_cast<size_t>(fi++)]);
    }
    return fi == baseFacetCount;
}

std::shared_ptr<MCGAL::Mesh> OptimizedBasicDeserializeOperator::deserializeBaseMesh() {
    const bool dbg = []() {
        const char* v = std::getenv("PMSF_EDGEFLIP_DEBUG");
        return v && v[0] == '1';
    }();
    std::shared_ptr<MCGAL::Mesh> mesh;
    if (header_.isEnableQuantization()) {
        const size_t quantMetadataOffset = dataOffset_;
        if (std::getenv("PMSF_SEAM_BASE_DEDUP")) {
            mesh = readSeamDedupBaseMesh(buffer_, dataOffset_);
        } else {
            mesh = MCGAL::readBaseMeshWithQuantization(DEFAULT_MESH_ID, buffer_, dataOffset_);
        }
        if (mesh) {
            mesh->enableQuantization = true;
            // MCGAL's legacy helper reads q but does not retain it on Mesh.
            // The residual reader must use the stream's q, not Mesh's default.
            mesh->i_nbQuantBits =
                static_cast<unsigned char>(buffer_[quantMetadataOffset]);
        }
    } else {
        mesh = MCGAL::readBaseMesh(DEFAULT_MESH_ID, buffer_, dataOffset_);
        if (mesh) mesh->enableQuantization = false;
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
    if (std::getenv("PMSF_SC_BOUNDARY_ANCHOR_STAGE") ||
        std::getenv("PMSF_BC_ANCHOR_DIAG")) {
        g_scBaseVertexOrder.clear();
        for (MCGAL::Vertex* v : mesh->vertices())
            if (v && !v->isRemoved()) g_scBaseVertexOrder.push_back(v);
    }
    if (auto* optimizedReader =
            dynamic_cast<OptimizedSymbolReadOperator*>(symbolReadOperator_.get())) {
        optimizedReader->setMesh(mesh);
    }
    reconstructOperator_->init(mesh);
    reconstructOperator_->setEnablePrediction(header_.isEnablePrediction());

    std::vector<int> parsedRoundCounts;
    std::vector<std::pair<int, int>> parsedSymbols;
    std::vector<EdgeFlipPostProcessOperator::CoordSymbol> parsedCoordSymbols;
    std::vector<int> parsedRoundBitCounts;
    std::vector<unsigned char> parsedRoundBits;
    std::vector<int> parsedBoundaryBitCounts;
    std::vector<unsigned char> parsedBoundaryBits;
    std::vector<int> parsedBoundaryResiduals;
    int parsedBoundaryMode = 0;
    readBoundaryBitPayload(buffer_, totalSize, dataOffset_, parsedBoundaryBitCounts, parsedBoundaryBits,
                           parsedBoundaryResiduals, parsedBoundaryMode);
    if (mesh && std::getenv("PMSF_SEAM_SHARED_BASE")) {
        const size_t blobStart = dataOffset_;
        const bool peeked = peekApplySegmentedBaseGroups(buffer_ + blobStart,
                                         totalSize > blobStart ? totalSize - blobStart : 0,
                                         mesh);
        if (std::getenv("PMSF_SEAM_DEBUG")) std::cerr << "[shared-base] peek=" << (peeked?1:0) << " meshV=" << mesh->size_of_vertices() << " F=" << mesh->size_of_facets() << std::endl;
        if (peeked) {
            auto cut = pmsf::SeamCutOperator::run(mesh, {});
            if (cut.mesh) {
                cut.mesh->enableQuantization = mesh->enableQuantization;
                cut.mesh->i_nbQuantBits = mesh->i_nbQuantBits;
                cut.mesh->bboxMin = mesh->bboxMin;
                cut.mesh->f_quantStep = mesh->f_quantStep;
                mesh = cut.mesh;
                this->mesh_ = mesh;
                if (auto* optimizedReader = dynamic_cast<OptimizedSymbolReadOperator*>(symbolReadOperator_.get())) optimizedReader->setMesh(mesh);
                reconstructOperator_->init(mesh);
                reconstructOperator_->setEnablePrediction(header_.isEnablePrediction());
            }
        }
    }

    boundaryApplyMode_ = parsedBoundaryMode;
    if ((boundaryApplyMode_ == 70 || boundaryApplyMode_ == 71) && mesh_) {
        // Mirror the encoder's end-of-round proxy canonicalisation on the freshly
        // read base so the first (reverse) round's BFS traversal state matches.
        canonicalizeDecoderProxiesDirected(mesh_);
        g_scBaseVertexOrder.clear();
        for (MCGAL::Vertex* bv : mesh_->vertices())
            if (bv && !bv->isRemoved()) g_scBaseVertexOrder.push_back(bv);
    }

    if (boundaryApplyMode_ == 10 && std::getenv("PMSF_SEAM_FINAL_BOUNDARY")) {
        // Final-base boundary compression: base stream contains M' (collapsed).
        const size_t blobStart = dataOffset_;
        peekApplySegmentedBaseGroups(buffer_ + blobStart,
                                     totalSize > blobStart ? totalSize - blobStart : 0,
                                     mesh);
        std::vector<int> ones;
        for (size_t bi = 0; bi < parsedBoundaryBits.size(); ++bi)
            if (parsedBoundaryBits[bi] != 0) ones.push_back(static_cast<int>(bi));
        const int nOps = static_cast<int>(ones.size());
        std::vector<int> removedIdx(nOps, -1);
        std::unordered_map<int, MCGAL::Vertex*> newVertices;
        for (int rec = nOps - 1; rec >= 0; --rec) {
            const size_t ri = static_cast<size_t>(rec) * 6;
            if (ri + 5 >= parsedBoundaryResiduals.size()) break;
            const int edgeIndex = parsedBoundaryResiduals[ri];
            const int dx = parsedBoundaryResiduals[ri + 1];
            const int dy = parsedBoundaryResiduals[ri + 2];
            const int dz = parsedBoundaryResiduals[ri + 3];
            const int origIndex = parsedBoundaryResiduals[ri + 4];
            const int gid = parsedBoundaryResiduals[ri + 5];
            removedIdx[static_cast<size_t>(rec)] = origIndex;
            const auto edges = pmsf::edgeflip::collectMeshBoundaryEdgesSorted(mesh);
            if (edgeIndex < 0 || edgeIndex >= static_cast<int>(edges.size())) continue;
            MCGAL::Halfedge* target = edges[static_cast<size_t>(edgeIndex)];
            if (!target || target->isRemoved() || target->opposite() != nullptr || !target->face()) continue;
            const MCGAL::PointInt ha = mesh->floatPosToInt(target->vertex()->point());
            const MCGAL::PointInt hb = mesh->floatPosToInt(target->end_vertex()->point());
            const MCGAL::PointInt pi((ha.x() + hb.x()) / 2 + dx,
                                     (ha.y() + hb.y()) / 2 + dy,
                                     (ha.z() + hb.z()) / 2 + dz);
            MCGAL::Vertex* nv = nullptr;
            if (pmsf::splitBoundaryEdgeInsertVertex(mesh, target, mesh->intPosToFloat(pi), &nv) && nv) {
                newVertices[origIndex] = nv;
                for (MCGAL::Halfedge* h : pmsf::boundedRing(nv)) {
                    if (h && h->face()) h->face()->setGroupId(gid);
                }
            }
        }
        const int baseCount = static_cast<int>(mesh->vertices().size()) - nOps;
        if (baseCount >= 0) {
            std::set<int> removed(removedIdx.begin(), removedIdx.end());
            std::vector<MCGAL::Vertex*> base(mesh->vertices().begin(),
                                             mesh->vertices().begin() + baseCount);
            std::vector<MCGAL::Vertex*> ordered;
            ordered.reserve(base.size() + newVertices.size());
            size_t bi = 0;
            const int total = static_cast<int>(base.size() + newVertices.size());
            for (int i = 0; i < total; ++i) {
                if (removed.count(i)) {
                    auto it = newVertices.find(i);
                    if (it != newVertices.end() && it->second) ordered.push_back(it->second);
                } else if (bi < base.size()) {
                    ordered.push_back(base[bi++]);
                }
            }
            mesh->vertices().swap(ordered);
        }
        setenv("PMSF_SEAM_BASE_GROUPS_PREASSIGNED", "1", 1);
        if (std::getenv("PMSF_SEAM_DEBUG"))
            std::cerr << "[seam-final-preinverse] ops=" << nOps
                      << " V=" << mesh->size_of_vertices()
                      << " F=" << mesh->size_of_facets() << std::endl;
    }

    if (parsedBoundaryMode == 30) {
        pmsf::setCompactBoundaryPayloadForVerify(parsedBoundaryBits, parsedBoundaryResiduals);
    }
    if (parsedBoundaryMode == 41) {
        pmsf::pendingBoundaryTopologyOps().clear();
        pmsf::parseBoundaryTopologyPatchBits(parsedBoundaryBits,
                                             pmsf::pendingBoundaryTopologyOps());
    }
    boundaryRoundBitCounts_ = std::move(parsedBoundaryBitCounts);
    boundaryRoundBits_ = std::move(parsedBoundaryBits);
    boundaryRoundResiduals_ = std::move(parsedBoundaryResiduals);

    if (boundaryApplyMode_ == 21 && std::getenv("PMSF_SEAM_FINAL_COMPLEX_FAN")) {
        peekApplySegmentedBaseGroups(buffer_ + dataOffset_,
                                     totalSize > dataOffset_ ? totalSize - dataOffset_ : 0,
                                     mesh);
        if (boundaryRoundResiduals_.size() >= 12) {
            const int k = boundaryRoundResiduals_[0];
            MCGAL::PointInt qv(boundaryRoundResiduals_[1], boundaryRoundResiduals_[2], boundaryRoundResiduals_[3]);
            MCGAL::PointInt qa(boundaryRoundResiduals_[4], boundaryRoundResiduals_[5], boundaryRoundResiduals_[6]);
            MCGAL::PointInt qb(boundaryRoundResiduals_[7], boundaryRoundResiduals_[8], boundaryRoundResiduals_[9]);
            const int apexSide = boundaryRoundResiduals_[10];
            const int gid = boundaryRoundResiduals_[11];
            const int vIdx = boundaryRoundResiduals_.size() > 13 ? boundaryRoundResiduals_[12] : -1;
            const int vPid = boundaryRoundResiduals_.size() > 13 ? boundaryRoundResiduals_[13] : -1;
            std::vector<int> faceIdx, facePid;
            if (boundaryRoundResiduals_.size() > 14) {
                const int recordedK = boundaryRoundResiduals_[14];
                for (int i = 0; i < recordedK && 15 + 2 * i + 1 < static_cast<int>(boundaryRoundResiduals_.size()); ++i) {
                    faceIdx.push_back(boundaryRoundResiduals_[15 + 2 * i]);
                    facePid.push_back(boundaryRoundResiduals_[16 + 2 * i]);
                }
            }
            MCGAL::Halfedge* found = nullptr;
            for (MCGAL::Facet* f : mesh->faces()) {
                if (!f || f->isRemoved() || !f->proxyHalfedge() || f->groupId() != gid) continue;
                MCGAL::Halfedge* h0 = f->proxyHalfedge();
                MCGAL::Halfedge* h = h0; int guard = 0;
                do {
                    if (h && !h->isRemoved() && h->opposite() == nullptr && h->vertex() && h->end_vertex()) {
                        auto p0 = mesh->floatPosToInt(h->vertex()->point());
                        auto p1 = mesh->floatPosToInt(h->end_vertex()->point());
                        if ((p0 == qa && p1 == qb) || (p0 == qb && p1 == qa)) { found = h; break; }
                    }
                    h = h->next();
                } while (h && h != h0 && ++guard < 64);
                if (found) break;
            }
            if (std::getenv("PMSF_SEAM_DEBUG")) {
                std::cerr << "[final-complex] k=" << k << " gid=" << gid << " found=" << (found?1:0)
                          << " qa=" << qa.x() << "," << qa.y() << "," << qa.z()
                          << " qb=" << qb.x() << "," << qb.y() << "," << qb.z() << std::endl;
            }
            if (found) {
                MCGAL::Point apexPos = (apexSide > 0) ? found->vertex()->point() : found->end_vertex()->point();
                MCGAL::Point vPos = mesh->intPosToFloat(qv);
                std::vector<MCGAL::Facet*> newFaces;
                MCGAL::Vertex* newV = nullptr;
                if (!pmsf::restoreBoundaryVertexFanCheap(mesh, found, vPos, k, gid, &apexPos, &newFaces, &newV)) {
                    std::cerr << "[final-complex-fan] restore failed" << std::endl;
                } else {
                    if (newV && vPid >= 0) newV->setPoolId(vPid);
                    for (size_t i = 0; i < newFaces.size() && i < facePid.size(); ++i)
                        if (newFaces[i] && facePid[i] >= 0) newFaces[i]->setPoolId(facePid[i]);
                    // restore vertex order
                    auto& vv = mesh->vertices();
                    if (newV) {
                        for (size_t vi = 0; vi < vv.size(); ++vi) if (vv[vi] == newV) { vv.erase(vv.begin() + vi); break; }
                        int at = std::max(0, std::min(vIdx, static_cast<int>(vv.size())));
                        vv.insert(vv.begin() + at, newV);
                    }
                    // restore face order
                    auto& fv = mesh->faces();
                    std::vector<MCGAL::Facet*> live;
                    for (MCGAL::Facet* f : fv) {
                        if (!f || f->isRemoved()) continue;
                        bool isNew = false;
                        for (auto* nf : newFaces) if (nf == f) isNew = true;
                        if (!isNew) live.push_back(f);
                    }
                    std::vector<std::pair<int, MCGAL::Facet*>> toInsert;
                    for (size_t i = 0; i < newFaces.size() && i < faceIdx.size(); ++i)
                        toInsert.push_back({faceIdx[i], newFaces[i]});
                    std::sort(toInsert.begin(), toInsert.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
                    for (auto& pr : toInsert) {
                        int at = std::max(0, std::min(pr.first, static_cast<int>(live.size())));
                        live.insert(live.begin() + at, pr.second);
                    }
                    fv = live;
                    if (std::getenv("PMSF_SEAM_DEBUG"))
                        std::cerr << "[final-complex-fan] restored k=" << k << " gid=" << gid
                                  << " reordered" << std::endl;
                }
            }
        }
        setenv("PMSF_SEAM_BASE_GROUPS_PREASSIGNED", "1", 1);
    }

    if (std::getenv("PMSF_SEAM_DEBUG")) std::cerr << "[bnd-parse] rounds=" << boundaryRoundBitCounts_.size() << " mode=" << boundaryApplyMode_ << " residuals=" << boundaryRoundResiduals_.size() << std::endl;
    boundaryRoundBitPrefix_.assign(boundaryRoundBitCounts_.size() + 1, 0);
    for (size_t i = 0; i < boundaryRoundBitCounts_.size(); ++i) {
        int c = boundaryRoundBitCounts_[i];
        if (c < 0) c = 0;
        boundaryRoundBitPrefix_[i + 1] = boundaryRoundBitPrefix_[i] + c;
    }
    boundaryRoundResidualPrefix_.assign(boundaryRoundBitCounts_.size() + 1, 0);
    for (size_t i = 0; i < boundaryRoundBitCounts_.size(); ++i) {
        const int start = boundaryRoundBitPrefix_[i];
        const int end = boundaryRoundBitPrefix_[i + 1];
        int ones = 0;
        for (int bi = start; bi < end && bi < static_cast<int>(boundaryRoundBits_.size()); ++bi)
            if (boundaryRoundBits_[static_cast<size_t>(bi)] != 0) ++ones;
        const bool restoreIds = std::getenv("PMSF_SEAM_BOUNDARY_RESTORE_IDS") != nullptr;
        const bool restoreFaceIds = std::getenv("PMSF_SEAM_BOUNDARY_RESTORE_FACE_IDS") != nullptr;
        const int stride = (boundaryApplyMode_ == 20) ? 4 : (boundaryApplyMode_ == 61 || boundaryApplyMode_ == 62 || boundaryApplyMode_ == 63 || boundaryApplyMode_ == 64 || boundaryApplyMode_ == 65 || boundaryApplyMode_ == 66 || boundaryApplyMode_ == 70) ? 1 : (boundaryApplyMode_ == 13) ? 10 : (boundaryApplyMode_ == 8 || boundaryApplyMode_ == 12) ? 4 : (boundaryApplyMode_ == 11 ? (std::getenv("PMSF_SEAM_BOUNDARY_ORDERED_NOVERTEX") ? 4 : (std::getenv("PMSF_SEAM_BOUNDARY_ORDERED_NOFACE") ? 5 : (restoreFaceIds ? (std::getenv("PMSF_SEAM_BOUNDARY_RESTORE_HALF_IDS") ? 16 : 10) : (restoreIds ? 8 : 7)))) : 3);
        boundaryRoundResidualPrefix_[i + 1] = boundaryRoundResidualPrefix_[i] + ones * stride;
    }
    if (std::getenv("PMSF_BOUNDARY_DEBUG")) {
        std::cerr << "[boundary-parse] rounds=" << boundaryRoundBitCounts_.size()
                  << " bits=" << boundaryRoundBits_.size()
                  << " residuals=" << boundaryRoundResiduals_.size()
                  << " mode=" << boundaryApplyMode_ << std::endl;
    }
    const size_t payloadStartOffset = dataOffset_;
    const EdgeFlipPayloadKind payloadKind =
        readEdgeFlipPayload(buffer_, dataOffset_, totalSize, parsedRoundCounts, parsedSymbols,
                            parsedCoordSymbols, parsedRoundBitCounts, parsedRoundBits);
    if (std::getenv("PMSF_EDGEFLIP_DEBUG")) {
        std::cerr << "[edgeflip-parse][opt] kind=" << static_cast<int>(payloadKind)
                  << " roundCounts=" << parsedRoundCounts.size()
                  << " bitCounts=" << parsedRoundBitCounts.size()
                  << " bits=" << parsedRoundBits.size()
                  << " first=" << (parsedRoundBitCounts.empty() ? -1 : parsedRoundBitCounts.front())
                  << std::endl;
    }
    if (dbg) {
        std::fprintf(stderr,
                     "[edgeflip-debug][opt] payload kind=%d start=%zu end=%zu rounds=%zu idSyms=%zu coordSyms=%zu\n",
                     static_cast<int>(payloadKind),
                     payloadStartOffset,
                     dataOffset_,
                     parsedRoundCounts.size(),
                     parsedSymbols.size(),
                     parsedCoordSymbols.size());
    }
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
               payloadKind == EdgeFlipPayloadKind::InterleavedV7 ||
               payloadKind == EdgeFlipPayloadKind::InterleavedV8 ||
               payloadKind == EdgeFlipPayloadKind::InterleavedV9) {
        setenv("PMSF_EDGEFLIP_STABLE_BFS", "1", 1);
        edgeFlipSeedBfsOrder_ = (payloadKind == EdgeFlipPayloadKind::InterleavedV8);
        edgeFlipCanonicalBits_ = (payloadKind == EdgeFlipPayloadKind::InterleavedV9);
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

    // ========== RD 预测路径：使用 range coder 一次性解码全部符号和残差 ==========
    if (header_.isEnablePrediction()) {
        if (header_.isEnableQuantization()) {
            symbolReadOperator_->enableQuantization(mesh_->bboxMin, mesh_->f_quantStep);
        }
        // PHASE60 物品 3：分区（layout-32）产物。
        //
        // 编码端与后端无关：`SegmentedOptimizedSymbolCollectOperator::
        // exportToBufferWithRangeCoder` 写出的字节段在列式与二进制产物里**完全
        // 相同**（本函数下方 `BasicSerializeOperator::serialize` 把它整段追加在
        // 基网 + 边翻转 payload 之后，即文件尾部）。此前解码镜像只存在于列式
        // 一侧，二进制把这段前缀当成普通符号流整体 import ⇒ 前缀被当符号消费，
        // 实测 N=2 abort 于 `!rcDecodedResiduals_.empty()`、N=4 于
        // `!rcDecodedFacetSymbols_.empty()`。镜像已抽到
        // `pmsf::SegmentedRangeCodedDecode`（impl/common/），此处只是转发。
        if (isSegmentedRangeCodedPayload()) {
            if (boundaryApplyMode_ == 42 && mesh_ && !boundaryRoundResiduals_.empty()) {
                const auto& res = boundaryRoundResiduals_;
                const int n = res[0];
                std::vector<MCGAL::Vertex*> anchors(static_cast<std::size_t>(std::max(0, n)), nullptr);
                std::size_t ri = 1;
                for (int i = 0; i < n && ri + 3 <= res.size(); ++i) {
                    const int qx = res[ri++];
                    const int qy = res[ri++];
                    const int qz = res[ri++];
                    if (qx < 0) continue;
                    for (int tol = 0; tol <= 2 && !anchors[static_cast<std::size_t>(i)]; ++tol) {
                        for (MCGAL::Vertex* v : mesh_->vertices()) {
                            if (!v || v->isRemoved()) continue;
                            auto q = mesh_->floatPosToInt(v->point());
                            if (std::abs(q.x() - qx) <= tol && std::abs(q.y() - qy) <= tol &&
                                std::abs(q.z() - qz) <= tol) {
                                anchors[static_cast<std::size_t>(i)] = v;
                                break;
                            }
                        }
                    }
                }
                pmsf::bfsGroupAnchors() = anchors;
            }
            pmsf::SegmentedRangeCodedDecode::Context ctx;
            ctx.header = header_;
            ctx.mesh = mesh_;
            ctx.reconstructor = reconstructOperator_.get();
            ctx.codecVersion = 0;  // 二进制容器无版本字段（见 SymbolReadOperator::setCodecVersion）
            ctx.bboxMin = mesh_->bboxMin;
            ctx.quantStep = mesh_->f_quantStep;
            if (!segmentedDecode_.importBlob(buffer_ + dataOffset_, totalSize - dataOffset_, ctx)) {
                throw std::runtime_error("[OptimizedBasicDeserializeOperator] 分区 blob 前缀非法（groupNumber=" +
                                         std::to_string(header_.getGroupNumber()) + "，blobSize=" + std::to_string(totalSize - dataOffset_) + "）");
            }
            segmentedRangeCoded_ = true;
            if (boundaryApplyMode_ == 42 && std::getenv("PMSF_SEG_BFS_DECODE") &&
                mesh && !boundaryRoundResiduals_.empty()) {
                const auto& res = boundaryRoundResiduals_;
                const int n = res[0];
                std::vector<MCGAL::Vertex*> anchors(static_cast<std::size_t>(std::max(0, n)), nullptr);
                std::size_t ri = 1;
                for (int i = 0; i < n && ri + 3 <= res.size(); ++i) {
                    const int qx = res[ri++];
                    const int qy = res[ri++];
                    const int qz = res[ri++];
                    if (qx < 0) continue;
                    for (int tol = 0; tol <= 2 && !anchors[static_cast<std::size_t>(i)]; ++tol) {
                        for (MCGAL::Vertex* v : mesh->vertices()) {
                            if (!v || v->isRemoved()) continue;
                            auto q = mesh->floatPosToInt(v->point());
                            if (std::abs(q.x() - qx) <= tol && std::abs(q.y() - qy) <= tol &&
                                std::abs(q.z() - qz) <= tol) {
                                anchors[static_cast<std::size_t>(i)] = v;
                                break;
                            }
                        }
                    }
                }
                pmsf::bfsGroupAnchors() = anchors;
                pmsf::bfsRelabelFacetGroups(mesh, anchors);
                int live = 0; for (MCGAL::Vertex* v : anchors) if (v) ++live;
                std::fprintf(stderr, "[seg-bfs-decode] anchors=%d/%d\n", live, n);
            }
            return mesh;
        }
        symbolReadOperator_->init({0, 0, 0}, header_);
        symbolReadOperator_->importFromRangeCodedBuffer(
            buffer_, dataOffset_,
            header_.isEnableQuantization() ? mesh_->i_nbQuantBits : 0);
        return mesh;
    }

    int facetSymbolSize = readInt(buffer_, dataOffset_);
    int edgeSymbolSize = readInt(buffer_, dataOffset_);
    if (dbg) {
        std::fprintf(stderr,
                     "[edgeflip-debug][opt] symbolSizes facet=%d edge=%d off=%zu\n",
                     facetSymbolSize,
                     edgeSymbolSize,
                     dataOffset_);
    }
    int vertexSymbolSize = 0;
    if (header_.isEnableAdjustment()) {
        vertexSymbolSize = readInt(buffer_, dataOffset_);
    }

    pointOffset_ = facetSymbolSize + edgeSymbolSize + vertexSymbolSize + dataOffset_;
    size_t offset1 = dataOffset_ + facetSymbolSize;
    size_t offset2 = offset1 + edgeSymbolSize;
    size_t offset3 = offset2;
    if (header_.isEnableAdjustment()) {
        offset3 = offset3 + vertexSymbolSize;
    }
    symbolReadOperator_->init({offset1, offset2, offset3}, header_);

    if (header_.isEnableCompress()) {
        char* facetSymbolBuffer = new char[BUFFER_SIZE];
        char* edgeSymbolBuffer = new char[BUFFER_SIZE];
        char* vertexSymbolBuffer = new char[BUFFER_SIZE];
        char* symbolBuffer = new char[BUFFER_SIZE];
        char* pointBuffer = new char[BUFFER_SIZE];
        int realFacetSymbolSize = 0;
        int realEdgeSymbolSize = 0;
        int realVertexSymbolSize = 0;
        int realPointSize = 0;
        deserializeCharPointer(buffer_, dataOffset_, facetSymbolBuffer, realFacetSymbolSize);
        deserializeCharPointer(buffer_, offset1, edgeSymbolBuffer, realEdgeSymbolSize);
        if (header_.isEnableAdjustment()) {
            deserializeCharPointer(buffer_, offset2, vertexSymbolBuffer, realVertexSymbolSize);
        }
        deserializeCharPointer(buffer_, offset3, pointBuffer, realPointSize);
        delete[] buffer_;
        char* newBuffer = new char[realFacetSymbolSize + realEdgeSymbolSize + realVertexSymbolSize + realPointSize];
        std::memcpy(newBuffer, facetSymbolBuffer, realFacetSymbolSize);
        std::memcpy(newBuffer + realFacetSymbolSize, edgeSymbolBuffer, realEdgeSymbolSize);
        if (header_.isEnableAdjustment()) {
            std::memcpy(newBuffer + realFacetSymbolSize + realEdgeSymbolSize, vertexSymbolBuffer, realVertexSymbolSize);
        }
        std::memcpy(newBuffer + realFacetSymbolSize + realEdgeSymbolSize + realVertexSymbolSize, pointBuffer, realPointSize);
        offset1 = realFacetSymbolSize;
        offset2 = offset1 + realEdgeSymbolSize;

        offset3 = offset2 + realVertexSymbolSize;

        delete[] facetSymbolBuffer;
        delete[] edgeSymbolBuffer;
        delete[] vertexSymbolBuffer;
        delete[] pointBuffer;
        buffer_ = newBuffer;
        dataOffset_ = 0;
        symbolReadOperator_->init({offset1, offset2, offset3}, header_);
    }
    if (std::getenv("PMSF_SEAM_DEBUG"))
        std::fprintf(stderr, "[bfs-map-check] mode=%d res=%zu\n", boundaryApplyMode_,
                     boundaryRoundResiduals_.size());
    if (boundaryApplyMode_ == 42 && mesh && !boundaryRoundResiduals_.empty()) {
        const auto& res = boundaryRoundResiduals_;
        const int n = res[0];
        std::vector<MCGAL::Vertex*> anchors(static_cast<std::size_t>(std::max(0, n)), nullptr);
        std::size_t ri = 1;
        for (int i = 0; i < n && ri + 3 <= res.size(); ++i) {
            const int qx = res[ri++];
            const int qy = res[ri++];
            const int qz = res[ri++];
            if (qx < 0) continue;
            for (int tol = 0; tol <= 2 && !anchors[static_cast<std::size_t>(i)]; ++tol) {
                for (MCGAL::Vertex* v : mesh->vertices()) {
                    if (!v || v->isRemoved()) continue;
                    auto q = mesh->floatPosToInt(v->point());
                    if (std::abs(q.x() - qx) <= tol && std::abs(q.y() - qy) <= tol &&
                        std::abs(q.z() - qz) <= tol) {
                        anchors[static_cast<std::size_t>(i)] = v;
                        break;
                    }
                }
            }
        }
        pmsf::bfsGroupAnchors() = anchors;
        pmsf::bfsRelabelFacetGroups(mesh, anchors);
        int live = 0; for (MCGAL::Vertex* v : anchors) if (v) ++live;
        std::fprintf(stderr, "[seg-bfs-decode] anchors=%d/%d\n", live, n);
    }
    return mesh;
}

namespace {
}  // namespace

namespace {
using FaceTripleMap = pmsf::FaceTripleMap;
using pmsf::makeFaceTripleKey;
using pmsf::buildFaceTripleMap;

struct BndV3OpRec {
    int ak = -1, aa = 0, ab = 0;
    int bk = -1, ba = 0, bb = 0;
    int aHasPos = 0, aq[3] = {0, 0, 0};
    int bHasPos = 0, bq[3] = {0, 0, 0};
    int gA = -1, gB = -1, kA = 0, kB = 0, orient = 0;
    int rq[3] = {0, 0, 0};
    int tak = -1, taa = 0, tab = 0;
    int tbk = -1, tba = 0, tbb = 0;
    // Explicit patch topology: {gid, k0,a0,b0, k1,a1,b1, k2,a2,b2} per face.
    std::vector<std::array<int, 22>> expOrig;
    std::vector<std::array<int, 22>> expAdd;
    // Ordered-inverse container ranks (only for rounds in orderRounds).
    int vRank = -1;
    std::vector<int> faceRanks;
    // V4: local dictionary records.  v4Uniq = {tag, kind, a, b, qx, qy, qz};
    // v4Orig/v4Add = {gid, localIdx0, localIdx1, localIdx2}.
    std::vector<std::array<int, 7>> v4Uniq;
    std::vector<std::array<int, 4>> v4Orig;
    std::vector<std::array<int, 4>> v4Add;
};
struct BndV3Payload {
    bool decoupled = false;  // boundary-first decode schedule
    bool explicitMode = false;  // per-op explicit patch topology included
    bool njrMode = false;       // deterministic reconstruction (no join replay)
    bool v4Mode = false;        // V4 local-dictionary explicit records
    bool certsAll = false;      // per-op ring certificates for every op
    std::set<std::pair<int, int>> certOps;   // selective certificate ops
    int decodeStrat = 0;        // candidate/flip ordering strategy (payload bits 5-6)
    bool roundRetry = false;    // round-level validate+retry mode (payload bit 7)
    std::vector<int> roundGroupBoundaryEdges;  // per round: expected group-boundary edges (-1 absent)
    std::set<int> orderRounds;  // rounds with ordered-inverse container ranks
    bool explicitAll = false;   // explicit patch available for every op
    std::set<std::pair<int, int>> explicitOps;  // (round, op) with explicit patch
    bool parsed = false;
    bool valid = false;
    int R = 0;
    std::vector<int> orders;
    std::vector<std::vector<BndV3OpRec>> rounds;
    std::vector<std::pair<int, int>> anchors;
    std::unordered_map<int, std::array<int, 6>> seeds;  // round -> (id0 kind/a/b, id1 kind/a/b)
};
inline BndV3Payload& bndV3State() {
    static BndV3Payload st;
    return st;
}
inline bool bndV3ParseV2(const pmsf::bndid::BndBytes& bytes) {
    BndV3Payload& st = bndV3State();
    pmsf::bndid::ByteReader rd(bytes);
    auto getId = [&](int& kind, int& a, int& b) {
        kind = static_cast<int>(rd.getByte());
        b = static_cast<int>(rd.getU());
        a = (kind > 0) ? static_cast<int>(rd.getU()) : 0;
        if (kind == 3 || kind < 0) rd.ok = false;
    };
    const int v3Flags = static_cast<int>(rd.getByte());
    st.decoupled = (v3Flags & 1) != 0;
    st.explicitMode = (v3Flags & 4) != 0;
    st.njrMode = (v3Flags & 8) != 0;
    st.v4Mode = (v3Flags & 16) != 0;
    st.decodeStrat = (v3Flags >> 5) & 3;
    st.roundRetry = (v3Flags & 128) != 0;
    if (st.njrMode) pmsf::forceNoJoinReplay() = true;
    int nA = static_cast<int>(rd.getU());
    if (nA < 0 || nA > 1000000) return false;
    {
        int prevB = 0;
        for (int i = 0; i < nA; ++i) {
            const int g = static_cast<int>(rd.getU());
            prevB += static_cast<int>(rd.getU());
            if (!rd.ok) return false;
            st.anchors.emplace_back(g, prevB);
        }
    }
    st.R = static_cast<int>(rd.getU());
    if (st.R < 0 || st.R > 100000) return false;
    const int nS = static_cast<int>(rd.getU());
    if (nS < 0 || nS > 1000000) return false;
    std::vector<std::pair<int, std::array<int, 6>>> changes;
    for (int i = 0; i < nS; ++i) {
        const int r = static_cast<int>(rd.getU());
        int k0 = 0, a0 = 0, b0 = 0, k1 = 0, a1 = 0, b1 = 0;
        getId(k0, a0, b0);
        getId(k1, a1, b1);
        if (!rd.ok) return false;
        changes.emplace_back(r, std::array<int, 6>{k0, a0, b0, k1, a1, b1});
    }
    {
        std::size_t ci = 0;
        std::array<int, 6> cur{-1, -1, -1, -1, -1, -1};
        for (int r = 0; r < st.R; ++r) {
            if (ci < changes.size() && changes[ci].first == r) cur = changes[ci++].second;
            if (cur[0] >= 0) st.seeds[r] = cur;
        }
    }
    if (st.explicitMode) {
        const std::uint32_t nExp = rd.getU();
        if (nExp == 0xFFFFFFFFu) {
            st.explicitAll = true;
        } else {
            for (std::uint32_t i = 0; i < nExp; ++i) {
                const int er = static_cast<int>(rd.getU());
                const int eo = static_cast<int>(rd.getU());
                st.explicitOps.emplace(er, eo);
            }
        }
    }
    if ((v3Flags & 2) != 0) {
        const std::uint32_t nCert = rd.getU();
        if (nCert == 0xFFFFFFFFu) {
            st.certsAll = true;
        } else {
            for (std::uint32_t i = 0; i < nCert; ++i) {
                const int cr = static_cast<int>(rd.getU());
                const int co = static_cast<int>(rd.getU());
                st.certOps.emplace(cr, co);
            }
        }
    }
    if (st.roundRetry) {
        const std::uint32_t nOrder = rd.getU();
        for (std::uint32_t i = 0; i < nOrder; ++i) {
            st.orderRounds.insert(static_cast<int>(rd.getU()));
        }
    }
    st.orders.assign(static_cast<std::size_t>(st.R), 0);
    st.rounds.resize(static_cast<std::size_t>(st.R));
    for (int r = 0; r < st.R; ++r) {
        const int ord = static_cast<int>(rd.getU());
        const int n = static_cast<int>(rd.getU());
        if (n < 0 || n > 1000000) return false;
        st.orders[static_cast<std::size_t>(r)] = ord;
        auto& ops = st.rounds[static_cast<std::size_t>(r)];
        ops.reserve(static_cast<std::size_t>(n));
        for (int oi = 0; oi < n; ++oi) {
            BndV3OpRec op;
            auto readVRef = [&](int& kind, int& aa, int& bb, int& hasPos, int* q) {
                kind = static_cast<int>(rd.getByte());
                if (kind == 3) {
                    hasPos = 1;
                    q[0] = static_cast<int>(rd.getU());
                    q[1] = static_cast<int>(rd.getU());
                    q[2] = static_cast<int>(rd.getU());
                    return;
                }
                hasPos = 0;
                bb = static_cast<int>(rd.getU());
                if (kind > 0) aa = static_cast<int>(rd.getU());
            };
            readVRef(op.ak, op.aa, op.ab, op.aHasPos, op.aq);
            readVRef(op.bk, op.ba, op.bb, op.bHasPos, op.bq);
            op.gA = static_cast<int>(rd.getS());
            op.gB = static_cast<int>(rd.getS());
            const std::uint32_t kcode = rd.getU();
            op.kA = static_cast<int>(kcode & 63u);
            op.kB = static_cast<int>((kcode >> 6) & 63u);
            op.orient = static_cast<int>((kcode >> 12) & 15u);
            for (int k = 0; k < 3; ++k) op.rq[k] = static_cast<int>(rd.getU());
            if (st.roundRetry && st.orderRounds.count(r) != 0) {
                op.vRank = static_cast<int>(rd.getS());
                const int nR = op.kA + op.kB;
                op.faceRanks.reserve(static_cast<std::size_t>(std::max(0, nR)));
                for (int k = 0; k < nR; ++k) op.faceRanks.push_back(static_cast<int>(rd.getS()));
            }
            if ((v3Flags & 2) != 0 &&
                (st.certsAll || st.certOps.count({r, oi}) != 0)) {
                // Per-op ring certificates (third-vertex stable IDs).  Byte 3 means the
                // encoder could not resolve it -> strict certificate pass is skipped.
                auto readCert = [&](int& k, int& a, int& b) {
                    k = static_cast<int>(rd.getByte());
                    if (k == 3) {
                        k = -1;
                        return;
                    }
                    b = static_cast<int>(rd.getU());
                    a = (k > 0) ? static_cast<int>(rd.getU()) : 0;
                };
                readCert(op.tak, op.taa, op.tab);
                readCert(op.tbk, op.tba, op.tbb);
            }
            if ((v3Flags & 4) != 0 &&
                (st.explicitAll || st.explicitOps.count({r, oi}) != 0)) {
                if (st.v4Mode) {
                    auto readFaces4 = [&](std::vector<std::array<int, 4>>& dst) {
                        const int n = static_cast<int>(rd.getU());
                        if (n < 0 || n > 1000) return false;
                        dst.reserve(static_cast<std::size_t>(n));
                        for (int i = 0; i < n; ++i) {
                            std::array<int, 4> rec{};
                            rec[0] = static_cast<int>(rd.getS());
                            rec[1] = static_cast<int>(rd.getU());
                            rec[2] = static_cast<int>(rd.getU());
                            rec[3] = static_cast<int>(rd.getU());
                            dst.push_back(rec);
                        }
                        return true;
                    };
                    const int nUniq = static_cast<int>(rd.getU());
                    if (nUniq < 0 || nUniq > 1000) return false;
                    op.v4Uniq.reserve(static_cast<std::size_t>(nUniq));
                    for (int i = 0; i < nUniq; ++i) {
                        std::array<int, 7> rec{};
                        rec[0] = static_cast<int>(rd.getU());
                        if (rec[0] == 0 || rec[0] == 2) {
                            rec[1] = static_cast<int>(rd.getU());
                            rec[3] = static_cast<int>(rd.getU());
                            if (rec[1] > 0) rec[2] = static_cast<int>(rd.getU());
                        }
                        if (rec[0] == 1) {
                            rec[4] = static_cast<int>(rd.getU());
                            rec[5] = static_cast<int>(rd.getU());
                            rec[6] = static_cast<int>(rd.getU());
                        }
                        if (rec[0] == 2) {
                            rec[4] = static_cast<int>(rd.getU());
                            rec[5] = static_cast<int>(rd.getU());
                            rec[6] = static_cast<int>(rd.getU());
                        }
                        op.v4Uniq.push_back(rec);
                    }
                    if (!readFaces4(op.v4Orig) || !readFaces4(op.v4Add)) return false;
                } else {
                auto readFaces = [&](std::vector<std::array<int, 22>>& dst) {
                    const int n = static_cast<int>(rd.getU());
                    if (n < 0 || n > 1000) return false;
                    dst.reserve(static_cast<std::size_t>(n));
                    for (int i = 0; i < n; ++i) {
                        std::array<int, 22> rec{};
                        rec[0] = static_cast<int>(rd.getS());
                        for (int k = 0; k < 3; ++k) {
                            const int tag = static_cast<int>(rd.getU());
                            int* vr = &rec[1 + 7 * k];
                            vr[0] = tag;
                            if (tag == 0 || tag == 2) {
                                vr[1] = static_cast<int>(rd.getU());
                                vr[3] = static_cast<int>(rd.getU());
                                if (vr[1] > 0) vr[2] = static_cast<int>(rd.getU());
                            }
                            if (tag == 1 || tag == 2) {
                                vr[4] = static_cast<int>(rd.getU());
                                vr[5] = static_cast<int>(rd.getU());
                                vr[6] = static_cast<int>(rd.getU());
                            }
                        }
                        dst.push_back(rec);
                    }
                    return true;
                };
                if (!readFaces(op.expOrig) || !readFaces(op.expAdd)) return false;
                }
            }
            if (!rd.ok) return false;
            ops.push_back(op);
        }
        if (st.roundRetry) {
            if (st.roundGroupBoundaryEdges.size() < static_cast<std::size_t>(st.R))
                st.roundGroupBoundaryEdges.assign(static_cast<std::size_t>(st.R), -1);
            const std::uint32_t gbe = rd.getU();
            st.roundGroupBoundaryEdges[static_cast<std::size_t>(r)] =
                (gbe == 0) ? -1 : static_cast<int>(gbe - 1);
        }
    }
    if (!rd.ok || rd.pos != bytes.size()) return false;
    st.valid = true;
    return true;
}

inline bool bndV3Parse(const std::vector<int>& res) {
    BndV3Payload& st = bndV3State();
    if (st.parsed) return st.valid;
    st.parsed = true;
    if (res.empty()) return false;
    if (res[0] < 0) {
        const std::size_t n = static_cast<std::size_t>(-res[0]);
        pmsf::bndid::BndBytes bytes;
        bytes.reserve(n);
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t wi = 1 + i / 4;
            if (wi >= res.size()) return false;
            bytes.push_back(static_cast<unsigned char>((res[wi] >> (8 * (i % 4))) & 0xFF));
        }
        return bndV3ParseV2(bytes);
    }
    std::size_t cur = 0;
    auto rd = [&](int& v) -> bool {
        if (cur >= res.size()) return false;
        v = res[cur++];
        return true;
    };
    int magic = 0, ver = 0;
    if (!rd(magic) || magic != 0x4233) return false;
    if (!rd(ver) || ver != 1) return false;
    int nA = 0;
    if (!rd(nA) || nA < 0) return false;
    for (int i = 0; i < nA; ++i) {
        int g = 0, b = 0;
        if (!rd(g) || !rd(b)) return false;
        st.anchors.emplace_back(g, b);
    }
    int nSeeds = 0;
    if (!rd(nSeeds) || nSeeds < 0) return false;
    for (int i = 0; i < nSeeds; ++i) {
        int r = 0;
        if (!rd(r)) return false;
        std::array<int, 6> sq{};
        for (int k = 0; k < 6; ++k)
            if (!rd(sq[static_cast<std::size_t>(k)])) return false;
        st.seeds[r] = sq;
    }
    if (!rd(st.R) || st.R < 0) return false;
    st.orders.assign(static_cast<std::size_t>(st.R), 0);
    st.rounds.resize(static_cast<std::size_t>(st.R));
    for (int r = 0; r < st.R; ++r) {
        int ord = 0, n = 0;
        if (!rd(ord) || !rd(n) || n < 0) return false;
        st.orders[static_cast<std::size_t>(r)] = ord;
        auto& ops = st.rounds[static_cast<std::size_t>(r)];
        ops.reserve(static_cast<std::size_t>(n));
        for (int oi = 0; oi < n; ++oi) {
            BndV3OpRec op;
            if (!rd(op.ak) || !rd(op.aa) || !rd(op.ab)) return false;
            if (!rd(op.bk) || !rd(op.ba) || !rd(op.bb)) return false;
            if (!rd(op.gA) || !rd(op.gB) || !rd(op.kA) || !rd(op.kB) || !rd(op.orient)) return false;
            if (!rd(op.rq[0]) || !rd(op.rq[1]) || !rd(op.rq[2])) return false;
            if (!rd(op.tak) || !rd(op.taa) || !rd(op.tab)) return false;
            if (!rd(op.tbk) || !rd(op.tba) || !rd(op.tbb)) return false;
            ops.push_back(op);
        }
    }
    st.valid = true;
    return true;
}
using BndV3OutMap = std::unordered_map<MCGAL::Vertex*, std::vector<MCGAL::Halfedge*>>;
inline void bndV3BuildOutMap(MCGAL::Mesh* mesh, BndV3OutMap& outs) {
    outs.clear();
    if (!mesh) return;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        for (MCGAL::Halfedge* h : *f) {
            if (!h || h->isRemoved() || !h->vertex() || !h->end_vertex()) continue;
            outs[h->vertex()].push_back(h);
        }
    }
}
inline const std::unordered_set<MCGAL::Vertex*>* bndV3AnchorLoopSet(MCGAL::Mesh* mesh, int group) {
    static thread_local std::unordered_map<int, std::unordered_set<MCGAL::Vertex*>> cache;
    static thread_local int cacheMeshId = -1;
    if (!mesh) return nullptr;
    if (cacheMeshId != mesh->meshId()) {
        cache.clear();
        cacheMeshId = mesh->meshId();
    }
    auto it = cache.find(group);
    if (it != cache.end()) return it->second.empty() ? nullptr : &it->second;
    BndV3Payload& st = bndV3State();
    if (!st.valid) return nullptr;
    int baseIdx = -1;
    for (const auto& ar : st.anchors) {
        if (ar.first == group) { baseIdx = ar.second; break; }
    }
    if (baseIdx < 0) {
        cache.emplace(group, std::unordered_set<MCGAL::Vertex*>{});
        return nullptr;
    }
    pmsf::bndid::Id id;
    id.kind = 0;
    id.a = 0;
    id.b = baseIdx;
    MCGAL::Vertex* av = pmsf::bndid::decGet(id);
    std::unordered_set<MCGAL::Vertex*> set;
    if (!av || av->isRemoved()) {
        cache.emplace(group, std::move(set));
        return nullptr;
    }
    std::vector<MCGAL::Halfedge*> loop;
    if (!pmsf::walkLoopFromFixedAnchor(mesh, av, group, loop) || loop.empty()) {
        cache.emplace(group, std::move(set));
        return nullptr;
    }
    for (MCGAL::Halfedge* h : loop) {
        if (!h) continue;
        if (h->vertex()) set.insert(h->vertex());
        if (h->end_vertex()) set.insert(h->end_vertex());
    }
    auto res = cache.emplace(group, std::move(set));
    return res.first->second.empty() ? nullptr : &res.first->second;
}

inline MCGAL::Halfedge* bndSafeFindHalfedge(MCGAL::Vertex* a, MCGAL::Vertex* b);

inline std::uint64_t bndPosKey(int x, int y, int z) {
    auto c = [](int v) -> std::uint64_t {
        return static_cast<std::uint64_t>(std::max(0, std::min(v, (1 << 21) - 1)));
    };
    return (c(x) << 42) | (c(y) << 21) | c(z);
}

// Live-vertex quantized-position map used by explicit-patch records whose vertex
// had no resolvable stable ID at encode time (position fallback, explicit only).
inline std::unordered_map<std::uint64_t, MCGAL::Vertex*> bndBuildLivePosMap(MCGAL::Mesh* mesh) {
    std::unordered_map<std::uint64_t, MCGAL::Vertex*> out;
    if (!mesh) return out;
    out.reserve(mesh->vertices().size() * 2 + 1);
    std::unordered_set<std::uint64_t> ambiguous;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        const auto q = mesh->floatPosToInt(v->point());
        const std::uint64_t key = bndPosKey(q.x(), q.y(), q.z());
        if (out.count(key)) {
            ambiguous.insert(key);
            continue;
        }
        out[key] = v;
    }
    for (std::uint64_t k : ambiguous) out.erase(k);
    return out;
}

// Explicit-patch inverse: rebuild the forward patch purely from stored stable IDs.
// Used when the incremental compact inverse context has drifted (K-free large meshes).
inline bool bndV3ApplyExplicitOp(
    const std::shared_ptr<MCGAL::Mesh>& mesh, const BndV3OpRec& op, int round, int opIndex,
    FaceTripleMap& faces, const std::unordered_map<std::uint64_t, MCGAL::Vertex*>* posMap) {
    if (!mesh) return false;
    auto explFail = [&](const char* why) {
        return false;
    };
    static const bool fastCreate = []() {
        setenv("PMSF_SC_FAST_CREATE_FACE", "1", 0);
        return true;
    }();
    (void)fastCreate;
    auto resolve = [&](int kind, int a, int b) -> MCGAL::Vertex* {
        if (kind < 0 || kind > 2) return nullptr;
        pmsf::bndid::Id id;
        id.kind = kind;
        id.a = a;
        id.b = b;
        return pmsf::bndid::decGet(id);
    };
    auto resolveByPos = [&](int qx, int qy, int qz) -> MCGAL::Vertex* {
        if (!posMap) return nullptr;
        auto it = posMap->find(bndPosKey(qx, qy, qz));
        return it == posMap->end() ? nullptr : it->second;
    };
    auto resolveRec = [&](const int* vrec) -> MCGAL::Vertex* {
        const int tag = vrec[0];
        if (tag == 0 || tag == 2) {
            MCGAL::Vertex* v = resolve(vrec[1], vrec[2], vrec[3]);
            if (v) return v;
            if (tag == 2) return resolveByPos(vrec[4], vrec[5], vrec[6]);
            return nullptr;
        }
        return resolveByPos(vrec[4], vrec[5], vrec[6]);
    };
    std::vector<std::array<MCGAL::Vertex*, 3>> addTris;
    std::vector<std::array<MCGAL::Vertex*, 3>> origTris;
    std::vector<int> origGid;
    if (!op.v4Uniq.empty()) {
        std::vector<MCGAL::Vertex*> locals;
        locals.reserve(op.v4Uniq.size());
        for (const auto& rec : op.v4Uniq) {
            if (rec[0] == 3) {  // this op's removed vertex placeholder
                locals.push_back(nullptr);
                continue;
            }
            MCGAL::Vertex* v = nullptr;
            if (rec[0] == 0 || rec[0] == 2) v = resolve(rec[1], rec[2], rec[3]);
            if (!v && (rec[0] == 1 || rec[0] == 2)) v = resolveByPos(rec[4], rec[5], rec[6]);
            if (!v) return explFail("v4-resolve");
            locals.push_back(v);
        }
        auto localVert = [&](int idx) -> MCGAL::Vertex* {
            if (idx < 0 || idx >= static_cast<int>(locals.size())) return nullptr;
            return locals[static_cast<std::size_t>(idx)];
        };
        auto buildV4 = [&](const std::vector<std::array<int, 4>>& recs,
                           std::vector<std::array<MCGAL::Vertex*, 3>>& tris,
                           std::vector<int>* gids) -> bool {
            tris.clear();
            if (gids) gids->clear();
            tris.reserve(recs.size());
            if (gids) gids->reserve(recs.size());
            for (const auto& rec : recs) {
                std::array<MCGAL::Vertex*, 3> tri{localVert(rec[1]), localVert(rec[2]),
                                                  localVert(rec[3])};
                if (!tri[0] || !tri[1] || !tri[2]) return false;
                tris.push_back(tri);
                if (gids) gids->push_back(rec[0]);
            }
            return true;
        };
        if (!buildV4(op.v4Add, addTris, nullptr)) return explFail("v4-add");
        if (!buildV4(op.v4Orig, origTris, &origGid)) return explFail("v4-orig");
    } else {
    addTris.reserve(op.expAdd.size());
    for (const auto& rec : op.expAdd) {
        std::array<MCGAL::Vertex*, 3> tri{};
        for (int k = 0; k < 3; ++k)
            tri[static_cast<std::size_t>(k)] = resolveRec(&rec[1 + 4 * k]);
        if (!tri[0] || !tri[1] || !tri[2]) {
            return explFail("resolve-add");
        }
        addTris.push_back(tri);
    }
    origTris.reserve(op.expOrig.size());
    origGid.reserve(op.expOrig.size());
    for (const auto& rec : op.expOrig) {
        std::array<MCGAL::Vertex*, 3> tri{};
        for (int k = 0; k < 3; ++k) {
            const int tag = rec[1 + 4 * k];
            const int kind = rec[2 + 4 * k];
            const int a = rec[3 + 4 * k];
            const int b = rec[4 + 4 * k];
            if (tag == 0 && kind == 2 && a == round && b == opIndex) {
                tri[static_cast<std::size_t>(k)] = nullptr;  // this op's removed vertex
            } else {
                tri[static_cast<std::size_t>(k)] = resolveRec(&rec[1 + 4 * k]);
                if (!tri[static_cast<std::size_t>(k)]) return explFail("resolve-orig");
            }
        }
        origTris.push_back(tri);
        origGid.push_back(rec[0]);
    }
    }
    // Pre-check that every forward-patch face exists before mutating the mesh.
    std::vector<MCGAL::Facet*> toRemove;
    toRemove.reserve(addTris.size());
    for (const auto& tri : addTris) {
        auto it = faces.find(makeFaceTripleKey(tri[0], tri[1], tri[2]));
        if (it == faces.end() || !it->second || it->second->isRemoved()) return explFail("find-add");
        toRemove.push_back(it->second);
    }
    if (toRemove.empty()) return explFail("no-add-faces");
    for (std::size_t i = 0; i < toRemove.size(); ++i) {
        pmsf::removeFaceForPatch(mesh, toRemove[i]);
        faces.erase(makeFaceTripleKey(addTris[i][0], addTris[i][1], addTris[i][2]));
    }
    MCGAL::Vertex* nv = MCGAL::contextPool.allocateVertexFromPool(
        mesh->meshId(),
        mesh->intPosToFloat(MCGAL::PointInt(op.rq[0], op.rq[1], op.rq[2])));
    if (!nv) return explFail("alloc-vertex");
    nv->halfedges().clear();
    nv->setGroupId(op.gA);
    mesh->vertices().push_back(nv);
    pmsf::bndid::decRegisterStage(nv, round, opIndex);
    for (std::size_t i = 0; i < origTris.size(); ++i) {
        auto& tri = origTris[i];
        for (auto& vv : tri)
            if (!vv) vv = nv;
        if (tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2]) continue;
        MCGAL::Facet* nf = pmsf::createFaceFromTripleManual(mesh, tri[0], tri[1], tri[2],
                                                            origGid[i]);
        if (!nf) return explFail("create-orig");
        faces[makeFaceTripleKey(tri[0], tri[1], tri[2])] = nf;
    }
    pmsf::repairHalfedgePairing(mesh);
    pmsf::rebuildVertexHalfedgeRings(mesh);
    return true;
}

inline bool bndV3ApplyOp(const std::shared_ptr<MCGAL::Mesh>& mesh, const BndV3OpRec& op, int round, int opIndex, const BndV3OutMap& outs, FaceTripleMap* explicitFaces = nullptr, const std::unordered_map<std::uint64_t, MCGAL::Vertex*>* explicitPos = nullptr, const std::unordered_map<std::uint64_t, MCGAL::Vertex*>* endpointPos = nullptr) {
    pmsf::bndid::Id aid;
    aid.kind = op.ak;
    aid.a = op.aa;
    aid.b = op.ab;
    pmsf::bndid::Id bid;
    bid.kind = op.bk;
    bid.a = op.ba;
    bid.b = op.bb;
    auto resolveEndpoint = [&](MCGAL::Vertex* idVertex, int hasPos, const int* q) -> MCGAL::Vertex* {
        if (!hasPos) return idVertex;
        if (!endpointPos) return nullptr;
        auto it = endpointPos->find(bndPosKey(q[0], q[1], q[2]));
        return it == endpointPos->end() ? nullptr : it->second;
    };
    MCGAL::Vertex* va = resolveEndpoint(pmsf::bndid::decGet(aid), op.aHasPos, op.aq);
    MCGAL::Vertex* vb = resolveEndpoint(pmsf::bndid::decGet(bid), op.bHasPos, op.bq);
    if (!va || !vb || va->isRemoved() || vb->isRemoved()) {
        std::fprintf(stderr, "[bnd-v3] round=%d op=%d registry miss ak=%d bk=%d\n", round, opIndex,
                     op.ak, op.bk);
        return false;
    }
    // Locate the boundary edge by IDs only.  The capture side face is gA; the edge may
    // appear in either orientation after earlier inverse rounds, so collect both and try
    // both kA/kB orders x 4 flip codes (same robustness envelope as the mode70 decoder).
    std::vector<MCGAL::Halfedge*> cands;
    bool anchorStrict = true;
    auto matchId = [&](MCGAL::Vertex* v, int k, int a, int b) -> bool {
        if (!v) return false;
        pmsf::bndid::Id got;
        if (!pmsf::bndid::decLookup(v, got)) return false;
        return got.kind == k && got.a == a && got.b == b;
    };
    auto thirdVertexOf = [&](MCGAL::Facet* f, MCGAL::Vertex* x, MCGAL::Vertex* y) -> MCGAL::Vertex* {
        if (!f || !x || !y) return nullptr;
        for (MCGAL::Halfedge* hh : *f) {
            MCGAL::Vertex* v = hh ? hh->vertex() : nullptr;
            if (!v || v == x || v == y) continue;
            return v;
        }
        return nullptr;
    };
    if (explicitFaces && (!op.expAdd.empty() || !op.expOrig.empty())) {
        if (bndV3ApplyExplicitOp(mesh, op, round, opIndex, *explicitFaces, explicitPos)) return true;
        std::fprintf(stderr, "[bnd-v3] round=%d op=%d explicit inverse failed -> compact fallback\n",
                     round, opIndex);
    }
    bool certFallbackUsed = false;
    auto consider = [&](MCGAL::Halfedge* h) {
        if (!h || h->isRemoved() || !h->vertex() || !h->end_vertex()) return;
        if (!h->face() || h->face()->isRemoved()) return;
        MCGAL::Halfedge* opp = h->opposite();
        if (!opp || opp->isRemoved() || !opp->face() || opp->face()->isRemoved()) return;
        const int fg = h->face()->groupId();
        const int og = opp->face()->groupId();
        const bool pairOK = (fg == op.gA && og == op.gB) || (fg == op.gB && og == op.gA);
        if (!pairOK) return;
        if (anchorStrict) {
            // Anchor-driven boundary-loop check (topology + segmentation only, no
            // coordinates): the merged edge endpoints must lie on the group-gA
            // boundary loop reachable from the per-group fixed anchor.
            const auto* aset = bndV3AnchorLoopSet(mesh.get(), op.gA);
            if (aset) {
                if (aset->count(h->vertex()) == 0 || aset->count(h->end_vertex()) == 0) return;
            }
        }
        MCGAL::Vertex* tvF = thirdVertexOf(h->face(), h->vertex(), h->end_vertex());
        MCGAL::Vertex* tvO = thirdVertexOf(opp->face(), h->vertex(), h->end_vertex());
        const bool certOK =
            (fg == op.gA)
                ? (matchId(tvF, op.tak, op.taa, op.tab) && matchId(tvO, op.tbk, op.tba, op.tbb))
                : (matchId(tvF, op.tbk, op.tba, op.tbb) && matchId(tvO, op.tak, op.taa, op.tab));
        if (!certOK && !certFallbackUsed) return;  // strict pass first
        for (MCGAL::Halfedge* c : cands)
            if (c == h) return;
        cands.push_back(h);
    };
    auto scanOut = [&](MCGAL::Vertex* v) {
        auto it = outs.find(v);
        if (it == outs.end()) return;
        for (MCGAL::Halfedge* h : it->second) {
            if (!h || h->isRemoved()) continue;
            if ((h->vertex() == vb && h->end_vertex() == va) ||
                (h->vertex() == va && h->end_vertex() == vb))
                consider(h);
        }
    };
    scanOut(va);
    scanOut(vb);
    if (cands.empty()) {
        certFallbackUsed = true;
        anchorStrict = false;
        std::fprintf(stderr,
                     "[bnd-v3] round=%d op=%d cert/anchor-miss -> group-pair fallback\n", round,
                     opIndex);
        scanOut(va);
        scanOut(vb);
    }
    // Direct-edge fallback: outs map is built from ring traversal and can miss the
    // capture edge when a vertex ring was damaged by an earlier inverse.  Look up the
    // edge between the two decoded endpoint vertices directly (step-capped), regardless
    // of current face group labels; the patch inversion still validates the topology.
    if (MCGAL::Halfedge* dh = bndSafeFindHalfedge(va, vb)) {
        if (MCGAL::Halfedge* d2 = dh->opposite()) {
            if (!d2->isRemoved()) cands.push_back(d2);
        }
        cands.push_back(dh);
        std::fprintf(stderr, "[bnd-v3] round=%d op=%d direct-edge fallback cands=%zu\n", round,
                     opIndex, cands.size());
    }
    if (cands.empty()) {
        auto qv = [&](MCGAL::Vertex* vv) {
            if (!vv) return std::array<int, 3>{-1, -1, -1};
            const auto q = mesh->floatPosToInt(vv->point());
            return std::array<int, 3>{q.x(), q.y(), q.z()};
        };
        const auto qa = qv(va);
        const auto qb = qv(vb);
        std::fprintf(stderr,
                     "[bnd-v3] round=%d op=%d no id edge gA=%d a=(%d,%d,%d)qa=(%d,%d,%d) b=(%d,%d,%d)qb=(%d,%d,%d)\n",
                     round, opIndex, op.gA, op.ak, op.aa, op.ab, qa[0], qa[1], qa[2], op.bk, op.ba,
                     op.bb, qb[0], qb[1], qb[2]);
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
            for (MCGAL::Halfedge* h : *f) {
                if (!h || h->isRemoved()) continue;
                if (!((h->vertex() == vb && h->end_vertex() == va) ||
                      (h->vertex() == va && h->end_vertex() == vb)))
                    continue;
                MCGAL::Halfedge* opp = h->opposite();
                std::fprintf(stderr, "[bnd-v3-edge] dir=%s faceG=%d oppG=%d\n",
                             h->vertex() == vb ? "b->a" : "a->b",
                             (h->face() && !h->face()->isRemoved()) ? h->face()->groupId() : -999,
                             (opp && opp->face() && !opp->face()->isRemoved())
                                 ? opp->face()->groupId()
                                 : -999);
            }
        }
        return false;
    }
    // Ensure both directions of each candidate edge are present (group side may be flipped).
    {
        std::vector<MCGAL::Halfedge*> aug;
        aug.reserve(cands.size() * 2);
        for (MCGAL::Halfedge* h : cands) {
            aug.push_back(h);
            if (h && h->opposite() && !h->opposite()->isRemoved()) aug.push_back(h->opposite());
        }
        cands.swap(aug);
    }
    const int recFlipA = op.orient & 1;
    const int recFlipB = (op.orient >> 1) & 1;
    const int decodeStrat = (g_bndV3RetryStrat >= 0) ? g_bndV3RetryStrat : bndV3State().decodeStrat;
    const int flipCodes[4][2] = {{recFlipA, recFlipB},
                                 {recFlipA ^ 1, recFlipB},
                                 {recFlipA, recFlipB ^ 1},
                                 {recFlipA ^ 1, recFlipB ^ 1}};
    for (std::size_t cii = 0; cii < cands.size(); ++cii) {
        const std::size_t ci = (decodeStrat & 1) ? (cands.size() - 1 - cii) : cii;
        MCGAL::Halfedge* hh = cands[ci];
        if (!hh || hh->isRemoved() || !hh->vertex() || !hh->end_vertex()) continue;
        const auto qa = mesh->floatPosToInt(hh->end_vertex()->point());
        const auto qb = mesh->floatPosToInt(hh->vertex()->point());
        const bool hhFaceIsGA = (hh->face() && hh->face()->groupId() == op.gA);
        const int gAparam = hhFaceIsGA ? op.gA : op.gB;
        const int baseKa = hhFaceIsGA ? op.kA : op.kB;
        const int baseKb = hhFaceIsGA ? op.kB : op.kA;
        for (int koi = 0; koi < 2; ++koi) {
            const int ko = (decodeStrat & 2) ? (1 - koi) : koi;
            const int ka = ko ? baseKb : baseKa;
            const int kb = ko ? baseKa : baseKb;
            for (int fci = 0; fci < 4; ++fci) {
                const auto& fc = flipCodes[(decodeStrat & 4) ? (3 - fci) : fci];
                std::vector<int> compact;
                compact.reserve(19);
                compact.push_back(18);
                compact.push_back(qa.x());
                compact.push_back(qa.y());
                compact.push_back(qa.z());
                compact.push_back(qb.x());
                compact.push_back(qb.y());
                compact.push_back(qb.z());
                compact.push_back(gAparam);
                compact.push_back(ka);
                compact.push_back(kb);
                compact.push_back(op.rq[0]);
                compact.push_back(op.rq[1]);
                compact.push_back(op.rq[2]);
                for (int k = 0; k < 6; ++k) compact.push_back(-1);
                const bool ordered = op.vRank >= 0 &&
                                     static_cast<int>(op.faceRanks.size()) == ka + kb;
                if (ordered) {
                    compact[0] = 19 + ka + kb;
                    compact.push_back(op.vRank);
                    for (int rk : op.faceRanks) compact.push_back(rk);
                }
                pmsf::g_scForcedCandidate = hh;
                pmsf::g_scForceFlipA = fc[0];
                pmsf::g_scForceFlipB = fc[1];
                std::size_t cur = 0;
                MCGAL::Vertex* removed = nullptr;
                const bool ok =
                    ordered ? pmsf::applySingleBoundaryCompactPatchOrderedInverse(mesh, compact,
                                                                                  cur, &removed)
                            : pmsf::applySingleBoundaryCompactPatchInverse(mesh, compact, cur,
                                                                           nullptr, &removed);
                pmsf::g_scForcedCandidate = nullptr;
                pmsf::g_scForceFlipA = -1;
                pmsf::g_scForceFlipB = -1;
                if (ok && cur == compact.size() && removed) {
                    pmsf::repairHalfedgePairing(mesh);
                    pmsf::rebuildVertexHalfedgeRings(mesh);
                    pmsf::bndid::decRegisterStage(removed, round, opIndex);
                    return true;
                }
            }
        }
        // Ring-size variant fallback: early-round state drift can make the actual
        // ring sizes differ by one from the recorded kA/kB.  Try the neighbouring
        // (kA, kB) combinations; writer verification rejects any artifact where the
        // reconstructed topology does not match the encoder exactly.
        for (int da = -1; da <= 1; ++da) {
            for (int db = -1; db <= 1; ++db) {
                if (da == 0 && db == 0) continue;
                for (int koi = 0; koi < 2; ++koi) {
                    const int ko = (decodeStrat & 2) ? (1 - koi) : koi;
                    const int ka = (ko ? baseKb : baseKa) + da;
                    const int kb = (ko ? baseKa : baseKb) + db;
                    if (ka < 1 || kb < 1) continue;
                    for (int fci = 0; fci < 4; ++fci) {
                        const auto& fc = flipCodes[(decodeStrat & 4) ? (3 - fci) : fci];
                        std::vector<int> compact;
                        compact.reserve(19);
                        compact.push_back(18);
                        compact.push_back(qa.x());
                        compact.push_back(qa.y());
                        compact.push_back(qa.z());
                        compact.push_back(qb.x());
                        compact.push_back(qb.y());
                        compact.push_back(qb.z());
                        compact.push_back(gAparam);
                        compact.push_back(ka);
                        compact.push_back(kb);
                        compact.push_back(op.rq[0]);
                        compact.push_back(op.rq[1]);
                        compact.push_back(op.rq[2]);
                        for (int k = 0; k < 6; ++k) compact.push_back(-1);
                        const bool ordered = op.vRank >= 0 &&
                                             static_cast<int>(op.faceRanks.size()) == ka + kb;
                        if (ordered) {
                            compact[0] = 19 + ka + kb;
                            compact.push_back(op.vRank);
                            for (int rk : op.faceRanks) compact.push_back(rk);
                        }
                        pmsf::g_scForcedCandidate = hh;
                        pmsf::g_scForceFlipA = fc[0];
                        pmsf::g_scForceFlipB = fc[1];
                        std::size_t cur = 0;
                        MCGAL::Vertex* removed = nullptr;
                        const bool ok =
                            ordered ? pmsf::applySingleBoundaryCompactPatchOrderedInverse(
                                          mesh, compact, cur, &removed)
                                    : pmsf::applySingleBoundaryCompactPatchInverse(
                                          mesh, compact, cur, nullptr, &removed);
                        pmsf::g_scForcedCandidate = nullptr;
                        pmsf::g_scForceFlipA = -1;
                        pmsf::g_scForceFlipB = -1;
                        if (ok && cur == compact.size() && removed) {
                            pmsf::repairHalfedgePairing(mesh);
                            pmsf::rebuildVertexHalfedgeRings(mesh);
                            pmsf::bndid::decRegisterStage(removed, round, opIndex);
                            return true;
                        }
                    }
                }
            }
        }
    }
    std::fprintf(stderr, "[bnd-v3] round=%d op=%d inverse failed kA=%d kB=%d\n", round, opIndex,
                 op.kA, op.kB);
    return false;
}
inline void bndV3ApplyRound(const std::shared_ptr<MCGAL::Mesh>& mesh, int round) {
    BndV3Payload& st = bndV3State();
    if (!st.valid) return;
    if (round < 0 || round >= st.R) return;
    const auto& ops = st.rounds[static_cast<std::size_t>(round)];
    BndV3OutMap outs;
    bndV3BuildOutMap(mesh.get(), outs);
    FaceTripleMap explicitFaces;
    std::unordered_map<std::uint64_t, MCGAL::Vertex*> explicitPos;
    if (bndV3State().explicitMode) {
        explicitFaces = buildFaceTripleMap(mesh.get());
        explicitPos = bndBuildLivePosMap(mesh.get());
    }
    std::unordered_map<std::uint64_t, MCGAL::Vertex*> endpointPos;
    bool needEndpointPos = false;
    for (const auto& oneOp : ops) {
        if (oneOp.aHasPos || oneOp.bHasPos) {
            needEndpointPos = true;
            break;
        }
    }
    if (needEndpointPos) endpointPos = bndBuildLivePosMap(mesh.get());
    const bool forwardOps = std::getenv("PMSF_BND_V3_OP_FORWARD") != nullptr;
    if (forwardOps) {
        for (int oi = 0; oi < static_cast<int>(ops.size()); ++oi) {
            if (!bndV3ApplyOp(mesh, ops[static_cast<std::size_t>(oi)], round, oi, outs, bndV3State().explicitMode ? &explicitFaces : nullptr, bndV3State().explicitMode ? &explicitPos : nullptr, needEndpointPos ? &endpointPos : nullptr)) {
                if (std::getenv("PMSF_BND_V3_EXPLICIT_SCAN")) {
                    std::fprintf(stderr, "[bnd-v3-exp-required] %d:%d\n", round, oi);
                    continue;
                }
                throw std::runtime_error("bnd-v3: stage inverse failed");
            }
        }
    } else {
        for (int oi = static_cast<int>(ops.size()) - 1; oi >= 0; --oi) {
            if (!bndV3ApplyOp(mesh, ops[static_cast<std::size_t>(oi)], round, oi, outs,
                              bndV3State().explicitMode ? &explicitFaces : nullptr)) {
                if (std::getenv("PMSF_BND_V3_EXPLICIT_SCAN")) {
                    std::fprintf(stderr, "[bnd-v3-exp-required] %d:%d\n", round, oi);
                    continue;
                }
                throw std::runtime_error("bnd-v3: stage inverse failed");
            }
        }
    }
}
// Hang-safe replacement for Vertex::find_halfedge: a corrupted vertex ring (possible
// after an inverse that produced inconsistent halfedge pairing) can make the MCGAL
// implementation loop forever.  boundedRing() is step-capped, so this returns null
// instead of hanging; the caller then fails fast and writer verification rejects the
// artifact (K-adaptation lowers the responsible round).
inline MCGAL::Halfedge* bndSafeFindHalfedge(MCGAL::Vertex* a, MCGAL::Vertex* b) {
    if (!a || !b) return nullptr;
    for (MCGAL::Halfedge* h : pmsf::boundedRing(a)) {
        if (!h || h->isRemoved() || !h->vertex() || !h->end_vertex()) continue;
        if ((h->vertex() == a && h->end_vertex() == b) ||
            (h->vertex() == b && h->end_vertex() == a))
            return h;
    }
    return nullptr;
}

inline MCGAL::Halfedge* bndV3SeedHalfedge(MCGAL::Mesh* mesh, int round) {
    if (!mesh) return nullptr;
    BndV3Payload& st = bndV3State();
    if (!st.valid) return nullptr;
    auto it = st.seeds.find(round);
    if (it == st.seeds.end()) return nullptr;
    pmsf::bndid::Id id0;
    id0.kind = it->second[0];
    id0.a = it->second[1];
    id0.b = it->second[2];
    pmsf::bndid::Id id1;
    id1.kind = it->second[3];
    id1.a = it->second[4];
    id1.b = it->second[5];
    MCGAL::Vertex* v0 = pmsf::bndid::decGet(id0);
    MCGAL::Vertex* v1 = pmsf::bndid::decGet(id1);
    if (!v0 || !v1 || v0->isRemoved() || v1->isRemoved()) return nullptr;
    MCGAL::Halfedge* h = bndSafeFindHalfedge(v0, v1);
    if (!h) return nullptr;
    if (h->vertex() != v0 && h->opposite()) h = h->opposite();
    return (h && h->vertex() == v0) ? h : nullptr;
}
}  // namespace

// ---- round-level snapshot / retry support (K-free correct-first decoding) ----

static std::shared_ptr<MCGAL::Mesh> decCloneExactMesh(
    const std::shared_ptr<MCGAL::Mesh>& mesh,
    std::unordered_map<int, MCGAL::Vertex*>* outMap) {
    if (!mesh) return nullptr;
    auto out = std::make_shared<MCGAL::Mesh>();
    out->setMeshId(mesh->meshId());
    out->bboxMin = mesh->bboxMin;
    out->bboxMax = mesh->bboxMax;
    out->f_quantStep = mesh->f_quantStep;
    out->i_nbQuantBits = mesh->i_nbQuantBits;
    out->enableQuantization = mesh->enableQuantization;
    std::unordered_map<int, MCGAL::Vertex*> vmap;
    std::unordered_map<MCGAL::Vertex*, MCGAL::Vertex*> pmap;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        MCGAL::Vertex* nv = MCGAL::contextPool.allocateVertexFromPool(out->meshId(), v->point());
        if (!nv) return nullptr;
        nv->halfedges().clear();
        nv->setGroupId(v->groupId());
        out->add_vertex(nv);
        vmap[v->poolId()] = nv;
        pmap[v] = nv;
    }
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        std::vector<MCGAL::Vertex*> vs;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (!h || !h->vertex()) break;
            auto it = pmap.find(h->vertex());
            if (it == pmap.end()) break;
            vs.push_back(it->second);
            h = h->next();
        } while (h && h != h0 && ++guard < 64);
        if (vs.size() < 3) continue;
        MCGAL::Facet* nf = MCGAL::contextPool.allocateFaceFromPool(out->meshId(), vs);
        if (!nf) return nullptr;
        nf->setGroupId(f->groupId());
        out->add_face(nf);
    }
    for (MCGAL::Facet* f : out->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h) {
                if (h->opposite() == nullptr) h->setBoundary();
                else { h->setNotBoundary(); h->opposite()->setNotBoundary(); }
            }
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
    }
    if (outMap) *outMap = std::move(vmap);
    return out;
}

// Replace mesh contents by a clone (vector swap) and remap the bndid decoder registry
// through the original->snapshot and snapshot->fresh maps.
static void decRestoreMeshFromSnapshot(
    const std::shared_ptr<MCGAL::Mesh>& mesh,
    const std::shared_ptr<MCGAL::Mesh>& snap,
    const std::unordered_map<int, MCGAL::Vertex*>& origToSnap,
    const pmsf::bndid::DecoderState& decSnap) {
    if (!mesh || !snap) return;
    std::unordered_map<int, MCGAL::Vertex*> snapToFresh;
    auto fresh = decCloneExactMesh(snap, &snapToFresh);
    if (!fresh) return;
    std::unordered_map<int, MCGAL::Vertex*> origToFresh;
    origToFresh.reserve(origToSnap.size());
    for (const auto& kv : origToSnap) {
        auto it = snapToFresh.find(kv.second->poolId());
        if (it != snapToFresh.end()) origToFresh[kv.first] = it->second;
    }
    mesh->vertices().swap(fresh->vertices());
    mesh->faces().swap(fresh->faces());
    mesh->bboxMin = snap->bboxMin;
    mesh->bboxMax = snap->bboxMax;
    mesh->f_quantStep = snap->f_quantStep;
    mesh->i_nbQuantBits = snap->i_nbQuantBits;
    mesh->enableQuantization = snap->enableQuantization;
    // remap decoder registry
    auto& dec = pmsf::bndid::dec();
    dec.byId = decSnap.byId;
    dec.byVertex.clear();
    dec.baseOrder.clear();
    dec.round = decSnap.round;
    dec.counter = decSnap.counter;
    for (auto& kv : dec.byId) {
        auto it = origToFresh.find(kv.second->poolId());
        if (it != origToFresh.end()) kv.second = it->second;
    }
    for (const auto& kv : dec.byId) dec.byVertex[kv.second] = kv.first;
    dec.baseOrder.reserve(decSnap.baseOrder.size());
    for (MCGAL::Vertex* v : decSnap.baseOrder) {
        auto it = origToFresh.find(v->poolId());
        dec.baseOrder.push_back(it != origToFresh.end() ? it->second : nullptr);
    }
}

void OptimizedBasicDeserializeOperator::applyBoundaryPatchesForRound(int boundaryRound) {
    if (std::getenv("PMSF_BOUNDARY_SKIP_APPLY")) return;
    if (boundaryApplyMode_ == 71) {
        if (!bndV3Parse(boundaryRoundResiduals_)) {
            std::fprintf(stderr, "[bnd-v3] bad payload (residual ints=%zu)\n",
                         boundaryRoundResiduals_.size());
            throw std::runtime_error("bnd-v3: bad payload");
        }
        if (boundaryRound < 0) return;
        if (bndV3State().decoupled) return;  // applied in finalizeBoundaryInverses
        if (boundaryAppliedRounds_.count(boundaryRound)) return;
        boundaryAppliedRounds_.insert(boundaryRound);
        const bool retryMode = bndV3State().roundRetry;
        const int expectedEdges =
            (boundaryRound < static_cast<int>(bndV3State().roundGroupBoundaryEdges.size()))
                ? bndV3State().roundGroupBoundaryEdges[static_cast<std::size_t>(boundaryRound)]
                : -1;
        std::shared_ptr<MCGAL::Mesh> snapMesh;
        std::unordered_map<int, MCGAL::Vertex*> origToSnap;
        pmsf::bndid::DecoderState decSnap;
        int maxAttempts = 1;
        if (retryMode) {
            snapMesh = decCloneExactMesh(mesh_, &origToSnap);
            if (snapMesh) {
                decSnap = pmsf::bndid::dec();
                maxAttempts = 8;
            }
        }
        bool roundDone = false;
        std::string lastError;
        for (int attempt = 0; attempt < maxAttempts && !roundDone; ++attempt) {
            if (attempt > 0 && snapMesh) {
                decRestoreMeshFromSnapshot(mesh_, snapMesh, origToSnap, decSnap);
                std::fprintf(stderr, "[bnd-v3-retry] round=%d attempt=%d\n", boundaryRound,
                             attempt);
            } else if (attempt > 0) {
                break;
            }
            g_bndV3RetryStrat = (attempt == 0) ? -1 : (attempt - 1);
            try {
                bndV3ApplyRound(mesh_, boundaryRound);
            } catch (const std::exception& e) {
                lastError = e.what();
                continue;
            }
            if (!retryMode || expectedEdges < 0) {
                roundDone = true;
                break;
            }
            pmsf::repairHalfedgePairing(mesh_);
            pmsf::rebuildVertexHalfedgeRings(mesh_);
            const int actualEdges = pmsf::countGroupBoundaryEdges(mesh_.get());
            if (actualEdges == expectedEdges) {
                roundDone = true;
            } else {
                std::fprintf(stderr,
                             "[bnd-v3-retry] round=%d attempt=%d group-boundary edges %d != %d\n",
                             boundaryRound, attempt, actualEdges, expectedEdges);
            }
        }
        g_bndV3RetryStrat = -1;
        if (!roundDone) {
            std::fprintf(stderr, "[bnd-v3-retry] round=%d failed: %s\n", boundaryRound,
                         lastError.c_str());
            throw std::runtime_error("bnd-v3: round inverse failed after retries");
        }
        // Mirror the encoder's canonicalizeTraversalProxies(mesh) right after the
        // stage round (runV2StageNow): next round's interior replay is proxy-dependent.
        if (mesh_) canonicalizeTraversalProxies(mesh_);
        return;
    }
    if (boundaryRound < 0 || boundaryRoundResidualPrefix_.empty()) return;
    if (boundaryAppliedRounds_.count(boundaryRound)) return;
    boundaryAppliedRounds_.insert(boundaryRound);
    if (boundaryApplyMode_ == 70) {
        static thread_local int v2RoundsCached = -1;
        static thread_local std::vector<std::pair<int, MCGAL::Vertex*>> v2Anchors;
        static thread_local std::vector<int> v2AnchorGroups;
        static thread_local std::vector<int> v2MetaOffset;
        static bool v2Ready = false;
        const int totalRounds = static_cast<int>(boundaryRoundBitCounts_.size());
        if (!v2Ready || v2RoundsCached != totalRounds) {
            v2Ready = true;
            v2RoundsCached = totalRounds;
            v2Anchors.clear();
            v2AnchorGroups.clear();
            v2MetaOffset.assign(static_cast<std::size_t>(std::max(0, totalRounds)), -1);
            if (boundaryRoundResiduals_.empty()) return;
            int cursor = 0;
            const int nA = boundaryRoundResiduals_[static_cast<std::size_t>(cursor++)];
            for (int k = 0; k < nA; ++k) {
                if (cursor + 1 >= static_cast<int>(boundaryRoundResiduals_.size())) break;
                const int g = boundaryRoundResiduals_[static_cast<std::size_t>(cursor++)];
                const int baseIdx = boundaryRoundResiduals_[static_cast<std::size_t>(cursor++)];
                MCGAL::Vertex* v = nullptr;
                if (baseIdx >= 0 && baseIdx < static_cast<int>(g_scBaseVertexOrder.size()))
                    v = g_scBaseVertexOrder[static_cast<std::size_t>(baseIdx)];
                v2Anchors.emplace_back(g, v);
                v2AnchorGroups.push_back(g);
            }
            for (int r = 0; r < totalRounds; ++r) {
                if (boundaryRoundBitCounts_[static_cast<std::size_t>(r)] <= 0) continue;
                v2MetaOffset[static_cast<std::size_t>(r)] = cursor;
                int ones = 0;
                const int st = boundaryRoundBitPrefix_[static_cast<std::size_t>(r)];
                const int en = boundaryRoundBitPrefix_[static_cast<std::size_t>(r + 1)];
                for (int bi = st; bi < en && bi < static_cast<int>(boundaryRoundBits_.size()); ++bi)
                    if (boundaryRoundBits_[static_cast<std::size_t>(bi)] != 0) ++ones;
                cursor += 1 + 6 * ones;
            }
            std::fprintf(stderr, "[bc-v2-dec] anchors=%d metaReady=%d\n",
                         static_cast<int>(v2Anchors.size()), cursor);
            if (std::getenv("PMSF_BC_V2_ANCHORS")) {
                std::fprintf(stderr, "[bc-v2-anchors-dec] n=%zu\n", v2Anchors.size());
                for (std::size_t ai = 0; ai < v2Anchors.size(); ++ai) {
                    const auto aq = (v2Anchors[ai].second && !v2Anchors[ai].second->isRemoved())
                                        ? mesh_->floatPosToInt(v2Anchors[ai].second->point())
                                        : MCGAL::PointInt(-1, -1, -1);
                    std::fprintf(stderr,
                                 "[bc-v2-anchors-dec] i=%zu g=%d q=(%d,%d,%d)\n", ai,
                                 v2Anchors[ai].first, aq.x(), aq.y(), aq.z());
                }
            }
            if (std::getenv("PMSF_BC_V2_DEBUG") && boundaryRound == totalRounds - 1) {
                pmsf::repairHalfedgePairing(mesh_);
                pmsf::rebuildVertexHalfedgeRings(mesh_);
                std::fprintf(stderr, "[bc-v2-base]");
                for (const auto& ag : v2Anchors) {
                    std::vector<MCGAL::Halfedge*> dl;
                    pmsf::walkLoopFromFixedAnchor(mesh_.get(), ag.second, ag.first, dl);
                    std::fprintf(stderr, " g%d=%zu", ag.first, dl.size());
                }
                std::fprintf(stderr, "\n");
            }
        }
        if (boundaryRound < 0 || boundaryRound >= totalRounds) return;
        const int roundCount = boundaryRoundBitCounts_[static_cast<std::size_t>(boundaryRound)];
        if (roundCount <= 0) return;
        if (v2MetaOffset[static_cast<std::size_t>(boundaryRound)] < 0)
            throw std::runtime_error("bc-v2: missing round metadata");
        pmsf::repairHalfedgePairing(mesh_);
        pmsf::rebuildVertexHalfedgeRings(mesh_);
        std::vector<MCGAL::Halfedge*> edges;
        std::vector<int> offsets;
        if (!pmsf::scV2EnumerateAnchors(mesh_.get(), v2Anchors, edges, &offsets)) {
            std::fprintf(stderr, "[bc-v2-dec] round=%d anchor enumeration failed\n", boundaryRound);
            throw std::runtime_error("bc-v2: anchor enumeration failed");
        }
        {
            std::size_t lv = 0, lf = 0;
            for (MCGAL::Vertex* v : mesh_->vertices())
                if (v && !v->isRemoved()) ++lv;
            for (MCGAL::Facet* f : mesh_->faces())
                if (f && !f->isRemoved()) ++lf;
            std::fprintf(stderr,
                         "[bc-v2-dec-debug] round=%d decodedEdges=%zu encoded=%d liveV=%zu liveF=%zu\n",
                         boundaryRound, edges.size(), roundCount, lv, lf);
        }
        if (std::getenv("PMSF_BC_V2_DEBUG")) {
            for (std::size_t ai = 0; ai < v2Anchors.size(); ++ai) {
                std::vector<MCGAL::Halfedge*> dbgLoop;
                const bool wok = pmsf::walkLoopFromFixedAnchor(
                    mesh_.get(), v2Anchors[ai].second, v2Anchors[ai].first, dbgLoop);
                const auto q = (v2Anchors[ai].second && !v2Anchors[ai].second->isRemoved())
                                   ? mesh_->floatPosToInt(v2Anchors[ai].second->point())
                                   : MCGAL::PointInt(0, 0, 0);
                std::fprintf(stderr,
                             "[bc-v2-anchor] dec round=%d g=%d q=(%d,%d,%d) loop=%zu walk=%d\n",
                             boundaryRound, v2Anchors[ai].first, q.x(), q.y(), q.z(),
                             dbgLoop.size(), wok ? 1 : 0);
            }
            char dbgPath[512];
            std::snprintf(dbgPath, sizeof(dbgPath), "/tmp/bc_v2_dec_round_%03d.off", boundaryRound);
            mesh_->dumpto_oldtype(dbgPath);
            std::snprintf(dbgPath, sizeof(dbgPath), "/tmp/bc_v2_dec_round_%03d.gids", boundaryRound);
            if (FILE* gf = std::fopen(dbgPath, "w")) {
                const int mq = (mesh_->i_nbQuantBits > 0) ? ((1 << mesh_->i_nbQuantBits) - 1)
                                                          : (1 << 30);
                for (MCGAL::Facet* ff : mesh_->faces()) {
                    if (!ff || ff->isRemoved() || !ff->proxyHalfedge()) continue;
                    const auto fv = pmsf::sbdetail::faceVertices(ff);
                    if (!fv[0]) continue;
                    std::vector<std::array<int, 3>> qs;
                    for (int k = 0; k < 3; ++k) {
                        const auto q = mesh_->floatPosToInt(fv[k]->point());
                        qs.push_back({std::clamp(q.x(), 0, mq), std::clamp(q.y(), 0, mq),
                                      std::clamp(q.z(), 0, mq)});
                    }
                    std::sort(qs.begin(), qs.end());
                    std::fprintf(gf, "%d %d %d %d %d %d %d %d %d %d\n", ff->groupId(),
                                 qs[0][0], qs[0][1], qs[0][2], qs[1][0], qs[1][1], qs[1][2],
                                 qs[2][0], qs[2][1], qs[2][2]);
                }
                std::fclose(gf);
            }
        }
        if (static_cast<int>(edges.size()) != roundCount) {
            std::fprintf(stderr, "[bc-v2-dec] round=%d enumeration size %zu != encoded %d\n",
                         boundaryRound, edges.size(), roundCount);
            throw std::runtime_error("bc-v2: boundary enumeration size mismatch");
        }
        const int startBit = boundaryRoundBitPrefix_[static_cast<std::size_t>(boundaryRound)];
        int rank = 0;
        for (int idx = 0; idx < roundCount; ++idx) {
            const int bitPos = startBit + idx;
            if (bitPos < 0 || bitPos >= static_cast<int>(boundaryRoundBits_.size())) break;
            if (boundaryRoundBits_[static_cast<std::size_t>(bitPos)] == 0) continue;
            int ai = 0;
            while (ai + 1 < static_cast<int>(offsets.size()) && offsets[static_cast<std::size_t>(ai + 1)] <= idx)
                ++ai;
            const int gA = v2AnchorGroups[static_cast<std::size_t>(ai)];
            MCGAL::Halfedge* edge = edges[static_cast<std::size_t>(idx)];
            const int metaPos =
                v2MetaOffset[static_cast<std::size_t>(boundaryRound)] + 1 + rank * 6;
            ++rank;
            if (metaPos + 5 >= static_cast<int>(boundaryRoundResiduals_.size()))
                throw std::runtime_error("bc-v2: truncated op metadata");
            const int kA = boundaryRoundResiduals_[static_cast<std::size_t>(metaPos)];
            const int kB = boundaryRoundResiduals_[static_cast<std::size_t>(metaPos + 1)];
            const int rx = boundaryRoundResiduals_[static_cast<std::size_t>(metaPos + 2)];
            const int ry = boundaryRoundResiduals_[static_cast<std::size_t>(metaPos + 3)];
            const int rz = boundaryRoundResiduals_[static_cast<std::size_t>(metaPos + 4)];
            const int orient = boundaryRoundResiduals_[static_cast<std::size_t>(metaPos + 5)];
            bool applied = false;
            const int recFlipA = orient & 1;
            const int recFlipB = (orient >> 1) & 1;
            auto tryInverse = [&](MCGAL::Halfedge* h, int ka, int kb, int fa, int fb) -> bool {
                if (!h || h->isRemoved() || !h->vertex() || !h->end_vertex()) return false;
                const auto qa = mesh_->floatPosToInt(h->end_vertex()->point());
                const auto qb = mesh_->floatPosToInt(h->vertex()->point());
                std::vector<int> compact;
                compact.reserve(19);
                compact.push_back(18);
                compact.push_back(qa.x()); compact.push_back(qa.y()); compact.push_back(qa.z());
                compact.push_back(qb.x()); compact.push_back(qb.y()); compact.push_back(qb.z());
                compact.push_back(gA);
                compact.push_back(ka);
                compact.push_back(kb);
                compact.push_back(rx); compact.push_back(ry); compact.push_back(rz);
                for (int k = 0; k < 6; ++k) compact.push_back(-1);
                pmsf::g_scForcedCandidate = h;
                pmsf::g_scForceFlipA = fa;
                pmsf::g_scForceFlipB = fb;
                std::size_t cur = 0;
                std::vector<MCGAL::Facet*> restoredDbg;
                MCGAL::Vertex* removedDbg = nullptr;
                const bool ok = pmsf::applySingleBoundaryCompactPatchInverse(
                    mesh_, compact, cur, &restoredDbg, &removedDbg);
                pmsf::g_scForcedCandidate = nullptr;
                pmsf::g_scForceFlipA = -1;
                pmsf::g_scForceFlipB = -1;
                if (!ok || cur != compact.size()) return false;
                if (std::getenv("PMSF_BC_V2_DEBUG") && mesh_ && removedDbg) {
                    const auto qd = mesh_->floatPosToInt(removedDbg->point());
                    std::fprintf(stderr, "[bc-v2-restored] round=%d gA=%d rm=(%d,%d,%d) faces=",
                                 boundaryRound, gA, qd.x(), qd.y(), qd.z());
                    for (MCGAL::Facet* rf : restoredDbg) {
                        const auto fv = pmsf::sbdetail::faceVertices(rf);
                        std::fprintf(stderr, " [gid=%d", rf->groupId());
                        for (MCGAL::Vertex* vv : fv) {
                            if (!vv) continue;
                            const auto q2 = mesh_->floatPosToInt(vv->point());
                            std::fprintf(stderr, " (%d,%d,%d)", q2.x(), q2.y(), q2.z());
                        }
                        std::fprintf(stderr, "]");
                    }
                    std::fprintf(stderr, "\n");
                }
                pmsf::repairHalfedgePairing(mesh_);
                pmsf::rebuildVertexHalfedgeRings(mesh_);
                return true;
            };
            MCGAL::Halfedge* raw = edge;
            MCGAL::Halfedge* other = raw ? raw->opposite() : nullptr;
            const int flipCodes[4][2] = {{recFlipA, recFlipB},
                                         {recFlipA ^ 1, recFlipB},
                                         {recFlipA, recFlipB ^ 1},
                                         {recFlipA ^ 1, recFlipB ^ 1}};
            int pickCode = -1;
            int usedFa = -1, usedFb = -1;
            for (int ci = 0; ci < 4 && !applied; ++ci) {
                MCGAL::Halfedge* hh = (ci < 2) ? raw : other;
                const int ka = (ci % 2 == 0) ? kA : kB;
                const int kb = (ci % 2 == 0) ? kB : kA;
                for (const auto& fc : flipCodes) {
                    if (tryInverse(hh, ka, kb, fc[0], fc[1])) {
                        applied = true;
                        pickCode = ci;
                        usedFa = fc[0];
                        usedFb = fc[1];
                        break;
                    }
                }
            }
            if (applied && std::getenv("PMSF_BC_V2_DEBUG") &&
                (pickCode != 0 || usedFa != recFlipA || usedFb != recFlipB)) {
                std::fprintf(stderr,
                             "[bc-v2-flipused] round=%d bit=%d rec=(%d,%d) used=(%d,%d) pick=%d\n",
                             boundaryRound, idx, recFlipA, recFlipB, usedFa, usedFb, pickCode);
            }
            if (applied && pickCode != 0 && std::getenv("PMSF_BC_V2_DEBUG")) {
                std::fprintf(stderr,
                             "[bc-v2-pick] round=%d bit=%d gA=%d kA=%d kB=%d orient=%d pick=%d\n",
                             boundaryRound, idx, gA, kA, kB, orient, pickCode);
            }
            if (!applied) {
                std::fprintf(stderr,
                             "[bc-v2-dec] round=%d op=%d inverse failed (edgeBit=%d kA=%d kB=%d)\n",
                             boundaryRound, rank - 1, idx, kA, kB);
                throw std::runtime_error("bc-v2: boundary inverse failed");
            }
        }
        if (std::getenv("PMSF_BC_V2_DEBUG")) {
            char postPath[512];
            std::snprintf(postPath, sizeof(postPath), "/tmp/bc_v2_dec_postinv_%03d.off",
                          boundaryRound);
            mesh_->dumpto_oldtype(postPath);
            char gidPath[512];
            std::snprintf(gidPath, sizeof(gidPath), "/tmp/bc_v2_dec_postinv_%03d.gids",
                          boundaryRound);
            if (FILE* gf = std::fopen(gidPath, "w")) {
                const int mq = (mesh_->i_nbQuantBits > 0) ? ((1 << mesh_->i_nbQuantBits) - 1)
                                                          : (1 << 30);
                for (MCGAL::Facet* ff : mesh_->faces()) {
                    if (!ff || ff->isRemoved() || !ff->proxyHalfedge()) continue;
                    const auto fv = pmsf::sbdetail::faceVertices(ff);
                    if (!fv[0]) continue;
                    std::vector<std::array<int, 3>> qs;
                    for (int k = 0; k < 3; ++k) {
                        const auto q = mesh_->floatPosToInt(fv[k]->point());
                        qs.push_back({std::clamp(q.x(), 0, mq), std::clamp(q.y(), 0, mq),
                                      std::clamp(q.z(), 0, mq)});
                    }
                    std::sort(qs.begin(), qs.end());
                    std::fprintf(gf, "%d %d %d %d %d %d %d %d %d %d\n", ff->groupId(),
                                 qs[0][0], qs[0][1], qs[0][2], qs[1][0], qs[1][1], qs[1][2],
                                 qs[2][0], qs[2][1], qs[2][2]);
                }
                std::fclose(gf);
            }
        }
        return;
    }
    if (boundaryApplyMode_ == 64 || boundaryApplyMode_ == 65 || boundaryApplyMode_ == 66) {
        const int start = boundaryRoundResidualPrefix_[static_cast<size_t>(boundaryRound)];
        const int end = boundaryRoundResidualPrefix_[static_cast<size_t>(boundaryRound + 1)];
        std::vector<std::pair<int, int>> ops;
        int pos = start;
        if (boundaryApplyMode_ == 66 && start < end) pos = start + 1;  // ordering marker
        while (pos < end) {
            const int len = boundaryRoundResiduals_[static_cast<size_t>(pos)];
            if ((len != 7 && len != 9 && len != 10 && len != 11 && len != 12 && len != 18 && !(len >= 19 && len <= 275)) ||
                pos + 1 + len > end)
                break;
            ops.emplace_back(pos, 1 + len);
            pos += 1 + len;
        }
        if (std::getenv("PMSF_SC_DEBUG") && !ops.empty()) {
            std::map<int, int> fcount, bcount;
            for (MCGAL::Facet* f : mesh_->faces()) {
                if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                fcount[f->groupId()]++;
                MCGAL::Halfedge* h0 = f->proxyHalfedge();
                MCGAL::Halfedge* h = h0;
                int guard = 0;
                do {
                    MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
                    if (h && !h->isRemoved() &&
                        (!opp || opp->isRemoved() || !opp->face() || opp->face()->isRemoved() ||
                         opp->face()->groupId() != f->groupId()))
                        bcount[f->groupId()]++;
                    h = h ? h->next() : nullptr;
                } while (h && h != h0 && ++guard < 64);
            }
            std::size_t liveV = 0;
            for (MCGAL::Vertex* v : mesh_->vertices())
                if (v && !v->isRemoved()) ++liveV;
            std::fprintf(stderr, "[sc-dec-groups] round=%d ops=%zu liveV=%zu", boundaryRound,
                         ops.size(), liveV);
            for (auto& kv : fcount)
                std::fprintf(stderr, " g%d:F%d/B%d", kv.first, kv.second,
                             bcount.count(kv.first) ? bcount[kv.first] : 0);
            std::fprintf(stderr, "\n");
        }
        // Base mesh pairing comes straight from buildFromBuffer; normalize once so
        // the first inverse op sees the same canonical pairing as later (repaired)
        // operations.
        pmsf::repairHalfedgePairing(mesh_);
        if (std::getenv("PMSF_BC_ANCHOR_DIAG")) {
            pmsf::rebuildVertexHalfedgeRings(mesh_);
            static thread_local std::vector<std::pair<int, MCGAL::Vertex*>> diagAnchors;
            static thread_local bool diagAnchorsLoaded = false;
            if (!diagAnchorsLoaded) {
                diagAnchorsLoaded = true;
                const char* af = std::getenv("PMSF_BC_ANCHOR_FILE");
                const char* apath = (af && af[0]) ? af : "/tmp/bc_anchors_q.txt";
                char bpath[512];
                std::snprintf(bpath, sizeof(bpath), "%s.base", apath);
                FILE* f = nullptr;
                if (FILE* bf = std::fopen(bpath, "r")) {
                    int g = 0, baseIdx = -1;
                    while (std::fscanf(bf, "%d %d", &g, &baseIdx) == 2) {
                        if (baseIdx >= 0 && baseIdx < static_cast<int>(g_scBaseVertexOrder.size()))
                            diagAnchors.emplace_back(g, g_scBaseVertexOrder[static_cast<std::size_t>(baseIdx)]);
                    }
                    std::fclose(bf);
                    std::fprintf(stderr, "[bc-anchor-base][dec] loaded=%zu baseOrder=%zu\n",
                                 diagAnchors.size(), g_scBaseVertexOrder.size());
                }
                if (diagAnchors.empty() && (f = std::fopen(apath, "r")) != nullptr) {
                    int g = 0, qx = 0, qy = 0, qz = 0;
                    const int mq = (mesh_->i_nbQuantBits > 0)
                                       ? ((1 << mesh_->i_nbQuantBits) - 1)
                                       : (1 << 30);
                    while (std::fscanf(f, "%d %d %d %d", &g, &qx, &qy, &qz) == 4) {
                        MCGAL::Vertex* found = nullptr;
                        for (MCGAL::Vertex* v : mesh_->vertices()) {
                            if (!v || v->isRemoved()) continue;
                            const auto q = mesh_->floatPosToInt(v->point());
                            if (std::clamp(q.x(), 0, mq) == std::clamp(qx, 0, mq) &&
                                std::clamp(q.y(), 0, mq) == std::clamp(qy, 0, mq) &&
                                std::clamp(q.z(), 0, mq) == std::clamp(qz, 0, mq)) {
                                found = v;
                                break;
                            }
                        }
                        if (found) diagAnchors.emplace_back(g, found);
                    }
                    std::fclose(f);
                }
            }
            pmsf::dumpFixedAnchorLoopSequences(mesh_.get(), diagAnchors, "dec", boundaryRound);
            {
                char bcp[512];
                std::snprintf(bcp, sizeof(bcp), "/tmp/bc_dec_round_%03d.off", boundaryRound);
                mesh_->dumpto_oldtype(bcp);
                std::snprintf(bcp, sizeof(bcp), "/tmp/bc_dec_round_%03d.gids", boundaryRound);
                if (FILE* gf = std::fopen(bcp, "w")) {
                    const int mqg = (mesh_->i_nbQuantBits > 0)
                                        ? ((1 << mesh_->i_nbQuantBits) - 1)
                                        : (1 << 30);
                    for (MCGAL::Facet* ffac : mesh_->faces()) {
                        if (!ffac || ffac->isRemoved() || !ffac->proxyHalfedge()) continue;
                        const auto fvp = pmsf::sbdetail::faceVertices(ffac);
                        if (!fvp[0]) continue;
                        std::vector<std::array<int, 3>> qs;
                        for (int k = 0; k < 3; ++k) {
                            const auto q = mesh_->floatPosToInt(fvp[k]->point());
                            qs.push_back({std::clamp(q.x(), 0, mqg), std::clamp(q.y(), 0, mqg),
                                          std::clamp(q.z(), 0, mqg)});
                        }
                        std::sort(qs.begin(), qs.end());
                        std::fprintf(gf, "%d %d %d %d %d %d %d %d %d %d\n", ffac->groupId(),
                                     qs[0][0], qs[0][1], qs[0][2], qs[1][0], qs[1][1],
                                     qs[1][2], qs[2][0], qs[2][1], qs[2][2]);
                    }
                    std::fclose(gf);
                }
            }
            const auto hs = pmsf::groupBoundaryEnumHashesAllLoops(mesh_.get());
            for (const auto& gh : hs)
                std::fprintf(stderr, "[bc-diag-dec] round=%d g=%d hash=%llx\n", boundaryRound,
                             gh.first, gh.second);
        }
        int applied = 0;
        int revIdx = 0;
        for (auto it = ops.rbegin(); it != ops.rend(); ++it, ++revIdx) {
            std::vector<int> opData(boundaryRoundResiduals_.begin() + it->first,
                                    boundaryRoundResiduals_.begin() + it->first + it->second);
            std::size_t cursor = 0;
            const bool gok = pmsf::applyGroupPatchInverseDirect(mesh_, opData, cursor) &&
                             cursor == opData.size();
            if (gok) {
                ++applied;
                pmsf::repairHalfedgePairing(mesh_);
                pmsf::rebuildVertexHalfedgeRings(mesh_);
            }
            if (std::getenv("PMSF_SC_DEBUG") && opData.size() >= 5) {
                const bool qlocOp = opData[0] == 11;
                const int packedK = qlocOp ? opData[8] : opData[4];
                const int gidDbg = qlocOp ? opData[7] : opData[3];
                std::fprintf(stderr, "[sc-group-op] round=%d op=%d ok=%d gA=%d kA=%d kB=%d len=%d\n",
                             boundaryRound, static_cast<int>(ops.size()) - 1 - revIdx, gok ? 1 : 0,
                             gidDbg, packedK / 64, packedK % 64, opData[0]);
            }
        }
        if (std::getenv("PMSF_SC_DEBUG"))
            std::cerr << "[sc-group-apply] round=" << boundaryRound << " ops=" << ops.size()
                      << " applied=" << applied << std::endl;
        // Fail fast instead of continuing on a silently corrupted topology: a
        // boundary inverse patch that cannot be re-applied means the locator no
        // longer matches the decoder state (quantization/locator ambiguity or a
        // broken artifact).  The payload is compact and every op is independently
        // reversible, so a partial application is always a decode error.
        if (applied != static_cast<int>(ops.size()) &&
            std::getenv("PMSF_SC_INVERSE_TOLERANT") == nullptr) {
            std::fprintf(stderr,
                         "[sc-inverse-fail] round=%d applied=%d ops=%zu: boundary patch "
                         "application failed (topologically inconsistent state or locator "
                         "ambiguity); aborting decode\n",
                         boundaryRound, applied, ops.size());
            throw std::runtime_error("boundary inverse patch failed");
        }
        return;
    }
    if (boundaryApplyMode_ == 63) {
        const int start = boundaryRoundResidualPrefix_[static_cast<size_t>(boundaryRound)];
        const int end = boundaryRoundResidualPrefix_[static_cast<size_t>(boundaryRound + 1)];
        int pos = start;
        if (boundaryRound + 1 == static_cast<int>(boundaryRoundBitCounts_.size())) {
            g_scDecodedAnchors.clear();
            if (pos < end) {
                const int n = boundaryRoundResiduals_[static_cast<size_t>(pos++)];
                std::vector<std::pair<int, int>> recs;
                recs.reserve(static_cast<std::size_t>(std::max(0, n)));
                for (int i = 0; i < n && pos + 2 <= end; ++i) {
                    const int baseIdx = boundaryRoundResiduals_[static_cast<size_t>(pos++)];
                    const int group = boundaryRoundResiduals_[static_cast<size_t>(pos++)];
                    recs.emplace_back(baseIdx, group);
                }
                g_scDecodedAnchors = pmsf::resolveAnchorsFromBase(
                    mesh_, recs, g_scBaseVertexOrder.empty() ? nullptr : &g_scBaseVertexOrder);
                if (std::getenv("PMSF_SC_DEBUG")) {
                    for (std::size_t k = 0; k < g_scDecodedAnchors.size(); ++k) {
                        const auto& d = g_scDecodedAnchors[k];
                        const MCGAL::PointInt q = mesh_->floatPosToInt(d.v->point());
                        std::fprintf(stderr, "  decAnchor[%zu] group=%d q=(%d,%d,%d)\n", k, d.group,
                                     q.x(), q.y(), q.z());
                    }
                }
            }
            if (std::getenv("PMSF_SC_DEBUG"))
                std::fprintf(stderr, "[sc-anchor-dec] round=%d anchors=%zu\n", boundaryRound,
                             g_scDecodedAnchors.size());
            g_scSeedByRound.clear();
            if (pos < end) {
                const int seedN = boundaryRoundResiduals_[static_cast<size_t>(pos++)];
                for (int si = 0; si < seedN && pos + 7 <= end; ++si) {
                    const int r = boundaryRoundResiduals_[static_cast<size_t>(pos++)];
                    std::array<int, 6> sq{};
                    for (int k = 0; k < 6; ++k)
                        sq[static_cast<std::size_t>(k)] =
                            boundaryRoundResiduals_[static_cast<size_t>(pos++)];
                    g_scSeedByRound[r] = sq;
                }
            }
        }
        std::vector<std::pair<int, int>> ops;
        while (pos < end) {
            const int len = boundaryRoundResiduals_[static_cast<size_t>(pos)];
            if (len != 11 || pos + 1 + len > end) break;
            ops.emplace_back(pos, 1 + len);
            pos += 1 + len;
        }
        MCGAL::Vertex* seedV0 = seed ? seed->vertex() : nullptr;
        MCGAL::Vertex* seedV1 = seed ? seed->end_vertex() : nullptr;
        int applied = 0;
        int revIdx = 0;
        for (auto it = ops.rbegin(); it != ops.rend(); ++it, ++revIdx) {
            std::vector<int> opData(boundaryRoundResiduals_.begin() + it->first,
                                    boundaryRoundResiduals_.begin() + it->first + it->second);
            std::size_t cursor = 0;
            int vBefore = 0, fBefore = 0, vAfter = 0, fAfter = 0;
            const bool ok = pmsf::applyAnchorPatchInverseDirect(mesh_, g_scDecodedAnchors, opData, cursor) &&
                            cursor == opData.size();
            if (ok) ++applied;
        }
        if (std::getenv("PMSF_SC_NO_CANON") == nullptr)
            pmsf::canonicalizeBoundaryMeshOrder(mesh_);
        if (std::getenv("PMSF_SC_NO_SEED_REBIND") == nullptr) {
        // Restore the encoder-side BFS seed for this round (its endpoints were
        // recorded at collect time).  This is what makes decodeOneRound follow the
        // same traversal as the encoder's collectRound.
        auto seedIt = g_scSeedByRound.find(boundaryRound);
        if (seedIt != g_scSeedByRound.end()) {
            const int maxQ = (1 << mesh_->i_nbQuantBits) - 1;
            auto clampV = [&](int x) { return std::clamp(x, 0, maxQ); };
            MCGAL::Vertex* v0 = nullptr;
            MCGAL::Vertex* v1 = nullptr;
            for (MCGAL::Vertex* v : mesh_->vertices()) {
                if (!v || v->isRemoved()) continue;
                const auto q = mesh_->floatPosToInt(v->point());
                const int qx = clampV(q.x()), qy = clampV(q.y()), qz = clampV(q.z());
                if (!v0 && qx == seedIt->second[0] && qy == seedIt->second[1] &&
                    qz == seedIt->second[2])
                    v0 = v;
                if (!v1 && qx == seedIt->second[3] && qy == seedIt->second[4] &&
                    qz == seedIt->second[5])
                    v1 = v;
                if (v0 && v1) break;
            }
            if (v0 && v1) {
                if (MCGAL::Halfedge* sh = bndSafeFindHalfedge(v0, v1)) {
                    if (sh->vertex() != v0 && sh->opposite()) sh = sh->opposite();
                    if (sh->vertex() == v0) seed = sh;
                }
            }
        }
        // Fallback: the boundary inverse may have removed/recreated the face that
        // owned the BFS seed halfedge.
        if (seed && (seed->isRemoved() || !seed->vertex() || !seed->end_vertex() ||
                     seed->vertex()->isRemoved() || seed->end_vertex()->isRemoved())) {
            MCGAL::Halfedge* rebind = nullptr;
            if (seedV0 && seedV1) {
                if (!seedV0->isRemoved() && !seedV1->isRemoved())
                    rebind = bndSafeFindHalfedge(seedV0, seedV1);
            }
            if (rebind) seed = rebind;
        }
        }
        if (std::getenv("PMSF_SC_DEBUG"))
            std::cerr << "[sc-anchor-apply] round=" << boundaryRound << " ops=" << ops.size()
                      << " applied=" << applied << std::endl;
        return;
    }
    if (boundaryApplyMode_ == 62) {
        const int start = boundaryRoundResidualPrefix_[static_cast<size_t>(boundaryRound)];
        const int end = boundaryRoundResidualPrefix_[static_cast<size_t>(boundaryRound + 1)];
        std::vector<std::pair<int, int>> ops;  // (payload begin incl. len, total ints)
        int pos = start;
        while (pos < end) {
            const int len = boundaryRoundResiduals_[static_cast<size_t>(pos)];
            if (len != 18 || pos + 1 + len > end) break;
            ops.emplace_back(pos, 1 + len);
            pos += 1 + len;
        }
        int applied = 0;
        for (auto it = ops.rbegin(); it != ops.rend(); ++it) {
            std::vector<int> opData(boundaryRoundResiduals_.begin() + it->first,
                                    boundaryRoundResiduals_.begin() + it->first + it->second);
            std::size_t cursor = 0;
            if (pmsf::applySingleBoundaryCompactPatchInverse(mesh_, opData, cursor) &&
                cursor == opData.size())
                ++applied;
        }
        if (std::getenv("PMSF_SC_DEBUG"))
            std::cerr << "[sc-boundary-compact-apply] round=" << boundaryRound
                      << " ops=" << ops.size() << " applied=" << applied << std::endl;
        return;
    }
    if (boundaryApplyMode_ == 61) {
        const int start = boundaryRoundResidualPrefix_[static_cast<size_t>(boundaryRound)];
        const int end = boundaryRoundResidualPrefix_[static_cast<size_t>(boundaryRound + 1)];
        std::vector<std::pair<int, int>> ops;  // (payload begin, length)
        int pos = start;
        while (pos < end) {
            const int len = boundaryRoundResiduals_[static_cast<size_t>(pos)];
            if (len <= 0 || pos + 1 + len > end) break;
            ops.emplace_back(pos + 1, len);
            pos += 1 + len;
        }
        if (boundaryRound + 1 == static_cast<int>(boundaryRoundBitCounts_.size()))
            g_scCanonicalPos.clear();
        int applied = 0;
        for (auto it = ops.rbegin(); it != ops.rend(); ++it) {
            std::vector<int> opData(boundaryRoundResiduals_.begin() + it->first,
                                    boundaryRoundResiduals_.begin() + it->first + it->second);
            std::size_t cursor = 0;
            if (pmsf::applySingleBoundaryPatchInversePositional(mesh_, opData, cursor,
                                                                &g_scCanonicalPos) &&
                cursor == opData.size())
                ++applied;
        }
        if (std::getenv("PMSF_SC_DEBUG"))
            std::cerr << "[sc-boundary-apply] round=" << boundaryRound
                      << " ops=" << ops.size() << " applied=" << applied << std::endl;
        return;
    }
    if (boundaryApplyMode_ == 21) return;
    // Modes 30/31 are boundary storage formats (anchor/1-bit derivation),
    // not mesh patch operations.  They are verified separately and must not
    // modify the decoded mesh here.
    if (boundaryApplyMode_ == 50) {
        const auto& res = boundaryRoundResiduals_;
        if (res.empty()) return;
        struct SplitRec {
            int round = -1;
            int d = 0;
            MCGAL::PointInt vPos{0, 0, 0};
            std::vector<MCGAL::PointInt> ringQ;
            std::vector<int> ringG;
        };
        std::size_t ri = 0;
        std::vector<SplitRec> recs;
        while (ri + 5 <= res.size()) {
            if (ri + 5 > res.size()) break;
            SplitRec rec;
            rec.round = res[ri++];
            rec.d = res[ri++];
            if (rec.d < 4 || rec.d > 12) break;
            rec.vPos = MCGAL::PointInt(res[ri], res[ri + 1], res[ri + 2]);
            ri += 3;
            if (ri + static_cast<std::size_t>(4 * rec.d) > res.size()) break;
            rec.ringQ.resize(static_cast<std::size_t>(rec.d));
            rec.ringG.resize(static_cast<std::size_t>(rec.d));
            for (int i = 0; i < rec.d; ++i) {
                rec.ringQ[static_cast<std::size_t>(i)] =
                    MCGAL::PointInt(rec.vPos.x() + res[ri], rec.vPos.y() + res[ri + 1],
                                    rec.vPos.z() + res[ri + 2]);
                ri += 3;
                rec.ringG[static_cast<std::size_t>(i)] = res[ri++];
            }
            recs.push_back(std::move(rec));
        }
        int applied = 0;
        for (int i = static_cast<int>(recs.size()) - 1; i >= 0; --i) {
            const auto& rec = recs[static_cast<std::size_t>(i)];
            if (rec.round != boundaryRound) continue;
            if (pmsf::restoreSharedGroupBoundaryVertexManual(
                    mesh_, rec.d, rec.vPos, rec.ringQ, rec.ringG))
                ++applied;
        }
        if (std::getenv("PMSF_BOUNDARY_DEBUG"))
            std::cerr << "[group-boundary-split-apply] round=" << boundaryRound
                      << " records=" << recs.size() << " applied=" << applied << std::endl;
        return;
    }
    if (boundaryApplyMode_ == 41) {
        const int applied = pmsf::applyBoundaryTopologyPatchRound(
            mesh_, boundaryRoundBits_, boundaryRound);
        std::cerr << "[boundary-topology-apply] round=" << boundaryRound
                  << " applied=" << applied << " bits=" << boundaryRoundBits_.size()
                  << std::endl;
        return;
    }
    if (boundaryApplyMode_ == 30 || boundaryApplyMode_ == 31 ||
        boundaryApplyMode_ == 40 || boundaryApplyMode_ == 42) return;
    if (boundaryApplyMode_ == 23) {
        const auto& res = boundaryRoundResiduals_;
        if (res.size() < 6) return;
        struct SharedBoundaryRecord {
            int round = -1;
            int d = 0;
            MCGAL::PointInt vPos{0, 0, 0};
            std::vector<MCGAL::PointInt> ringQ;
            std::vector<int> ringG;
            std::vector<std::array<MCGAL::PointInt, 3>> overrideFaces;
            std::vector<int> overrideGroups;
        };
        std::vector<SharedBoundaryRecord> records;
        size_t ri = 0;
        const int payloadVersion = res[ri++];
        int opCount = payloadVersion;
        int anchorInts = 0;
        if (payloadVersion == 2) {
            if (ri >= res.size()) return;
            anchorInts = res[ri++];
            if (anchorInts < 0 || ri + static_cast<size_t>(anchorInts) >= res.size()) return;
            ri += static_cast<size_t>(anchorInts);
            opCount = res[ri++];
        }
        if (opCount <= 0 || opCount > 256) return;
        if (std::getenv("PMSF_BOUNDARY_DEBUG"))
            std::cerr << "[shared-boundary-payload] version=" << payloadVersion
                      << " anchorInts=" << anchorInts << " ops=" << opCount << std::endl;
        for (int oi = 0; oi < opCount; ++oi) {
            if (ri + 7 > res.size()) break;
            SharedBoundaryRecord rec;
            rec.round = res[ri++];
            const int opInterface = res[ri++];
            const int opBoundaryIndex = res[ri++];
            (void)opInterface;
            (void)opBoundaryIndex;
            rec.d = res[ri++];
            if (rec.d < 4 || rec.d > 12) break;
            rec.vPos = MCGAL::PointInt(res[ri], res[ri + 1], res[ri + 2]);
            ri += 3;
            if (ri + static_cast<size_t>(4 * rec.d) > res.size()) break;
            rec.ringQ.resize(static_cast<size_t>(rec.d));
            rec.ringG.resize(static_cast<size_t>(rec.d));
            for (int i = 0; i < rec.d; ++i) {
                rec.ringQ[static_cast<size_t>(i)] =
                    MCGAL::PointInt(rec.vPos.x() + res[ri], rec.vPos.y() + res[ri + 1], rec.vPos.z() + res[ri + 2]);
                ri += 3;
                rec.ringG[static_cast<size_t>(i)] = res[ri++];
            }
            if (ri < res.size()) {
                const int nOverride = res[ri++];
                for (int oi = 0; oi < nOverride && ri + 9 < res.size(); ++oi) {
                    std::array<MCGAL::PointInt, 3> tri{};
                    for (int k = 0; k < 3; ++k) {
                        tri[static_cast<size_t>(k)] = MCGAL::PointInt(res[ri], res[ri + 1], res[ri + 2]);
                        ri += 3;
                    }
                    rec.overrideFaces.push_back(tri);
                    rec.overrideGroups.push_back(res[ri++]);
                }
            }
            records.push_back(std::move(rec));
        }
        auto applyRecord = [&](const SharedBoundaryRecord& rec) -> bool {
            const int d = rec.d;
            std::vector<MCGAL::Vertex*> ringV(static_cast<size_t>(d), nullptr);
            for (int i = 0; i < d; ++i) {
                for (MCGAL::Vertex* v : mesh_->vertices()) {
                    if (!v || v->isRemoved()) continue;
                    auto q = mesh_->floatPosToInt(v->point());
                    if (q.x() == rec.ringQ[static_cast<size_t>(i)].x() &&
                        q.y() == rec.ringQ[static_cast<size_t>(i)].y() &&
                        q.z() == rec.ringQ[static_cast<size_t>(i)].z()) { ringV[static_cast<size_t>(i)] = v; break; }
                }
            }
            for (auto* v : ringV) if (!v) return false;
            std::set<MCGAL::Vertex*> ringSet(ringV.begin(), ringV.end());
            std::vector<MCGAL::Facet*> fillFaces;
            for (MCGAL::Facet* f : mesh_->faces()) {
                if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                MCGAL::Vertex* fv[3]; int n = 0;
                MCGAL::Halfedge* h0 = f->proxyHalfedge(); MCGAL::Halfedge* hh = h0; int guard = 0;
                do { if (hh && hh->vertex() && n < 3) fv[n++] = hh->vertex(); hh = hh->next(); } while (hh && hh != h0 && ++guard < 64);
                if (n != 3) continue;
                if (ringSet.count(fv[0]) && ringSet.count(fv[1]) && ringSet.count(fv[2])) fillFaces.push_back(f);
            }
            for (MCGAL::Facet* f : fillFaces) {
                if (f && !f->isRemoved()) pmsf::removeFaceForPatch(mesh_, f);
            }
            MCGAL::Vertex* nv = MCGAL::contextPool.allocateVertexFromPool(mesh_->meshId(), mesh_->intPosToFloat(rec.vPos));
            if (!nv) return false;
            nv->halfedges().clear();
            nv->setGroupId(rec.ringG[0]);
            mesh_->vertices().push_back(nv);
            for (int i = 0; i < d; ++i) {
                std::vector<MCGAL::Vertex*> vs{nv, ringV[static_cast<size_t>(i)], ringV[static_cast<size_t>((i + 1) % d)]};
                MCGAL::Facet* nf = MCGAL::contextPool.allocateFaceFromPool(mesh_->meshId(), vs);
                if (!nf) return false;
                nf->setGroupId(rec.ringG[static_cast<size_t>(i)]);
                mesh_->add_face(nf);
            }
            for (size_t oi = 0; oi < rec.overrideFaces.size() && oi < rec.overrideGroups.size(); ++oi) {
                auto keyOf = [](const std::array<int, 3>& q) { return std::array<int, 3>{q[0], q[1], q[2]}; };
                std::array<std::array<int, 3>, 3> want{};
                for (int k = 0; k < 3; ++k) want[static_cast<size_t>(k)] = {rec.overrideFaces[oi][static_cast<size_t>(k)].x(), rec.overrideFaces[oi][static_cast<size_t>(k)].y(), rec.overrideFaces[oi][static_cast<size_t>(k)].z()};
                std::sort(want.begin(), want.end());
                for (MCGAL::Facet* f : mesh_->faces()) {
                    if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                    std::array<std::array<int, 3>, 3> have{};
                    int n = 0;
                    MCGAL::Halfedge* h0 = f->proxyHalfedge(); MCGAL::Halfedge* hh = h0; int guard = 0;
                    do {
                        if (hh && hh->vertex() && n < 3) {
                            auto q = mesh_->floatPosToInt(hh->vertex()->point());
                            have[static_cast<size_t>(n++)] = {q.x(), q.y(), q.z()};
                        }
                        hh = hh->next();
                    } while (hh && hh != h0 && ++guard < 64);
                    if (n != 3) continue;
                    std::sort(have.begin(), have.end());
                    if (have == want) { f->setGroupId(rec.overrideGroups[oi]); break; }
                }
            }
            return true;
        };
        int applied = 0;
        if (boundaryRound == 0) {
            for (int i = static_cast<int>(records.size()) - 1; i >= 0; --i) {
                if (records[static_cast<size_t>(i)].round == 0 && applyRecord(records[static_cast<size_t>(i)])) ++applied;
            }
            for (int i = static_cast<int>(records.size()) - 1; i >= 0; --i) {
                if (records[static_cast<size_t>(i)].round == -1 && applyRecord(records[static_cast<size_t>(i)])) ++applied;
            }
        } else {
            for (int i = static_cast<int>(records.size()) - 1; i >= 0; --i) {
                if (records[static_cast<size_t>(i)].round == boundaryRound && applyRecord(records[static_cast<size_t>(i)])) ++applied;
            }
        }
        if (std::getenv("PMSF_BOUNDARY_DEBUG"))
            std::cerr << "[shared-boundary-patch-apply] round=" << boundaryRound
                      << " ops=" << records.size() << " applied=" << applied << std::endl;
        return;
    }

    if (boundaryApplyMode_ == 22) {
        if (boundaryRound != 0) return;
        if (boundaryRoundResiduals_.empty()) return;
        const int count = boundaryRoundResiduals_[0];
        int ri = 1;
        int applied = 0;
        for (int op = 0; op < count; ++op) {
            if (ri + 11 >= static_cast<int>(boundaryRoundResiduals_.size())) break;
            const int k = boundaryRoundResiduals_[ri++];
            MCGAL::PointInt qv(boundaryRoundResiduals_[ri++], boundaryRoundResiduals_[ri++], boundaryRoundResiduals_[ri++]);
            MCGAL::PointInt qa(boundaryRoundResiduals_[ri++], boundaryRoundResiduals_[ri++], boundaryRoundResiduals_[ri++]);
            MCGAL::PointInt qb(boundaryRoundResiduals_[ri++], boundaryRoundResiduals_[ri++], boundaryRoundResiduals_[ri++]);
            const int apexSide = boundaryRoundResiduals_[ri++];
            const int gid = boundaryRoundResiduals_[ri++];
            MCGAL::Halfedge* found = nullptr;
            for (MCGAL::Facet* f : mesh_->faces()) {
                if (!f || f->isRemoved() || !f->proxyHalfedge() || f->groupId() != gid) continue;
                MCGAL::Halfedge* h0 = f->proxyHalfedge();
                MCGAL::Halfedge* h = h0; int guard = 0;
                do {
                    if (h && !h->isRemoved() && h->opposite() == nullptr && h->vertex() && h->end_vertex()) {
                        auto p0 = mesh_->floatPosToInt(h->vertex()->point());
                        auto p1 = mesh_->floatPosToInt(h->end_vertex()->point());
                        if ((p0 == qa && p1 == qb) || (p0 == qb && p1 == qa)) { found = h; break; }
                    }
                    h = h->next();
                } while (h && h != h0 && ++guard < 64);
                if (found) break;
            }
            if (!found) continue;
            MCGAL::Point apexPos = (apexSide > 0) ? found->vertex()->point() : found->end_vertex()->point();
            if (pmsf::restoreBoundaryVertexFanCheap(mesh_, found, mesh_->intPosToFloat(qv), k, gid, &apexPos)) ++applied;
        }
        if (std::getenv("PMSF_SEAM_DEBUG"))
            std::cerr << "[pre-complex-fan-apply] requested=" << count << " applied=" << applied << std::endl;
        return;
    }

    const bool mixedBoundary = boundaryApplyMode_ == 12;
    const bool round0Boundary = (boundaryRound == 0);
    const int effectiveMode = mixedBoundary ? (round0Boundary ? 8 : 11) : boundaryApplyMode_;
    if (effectiveMode == 20) {
        const int startBit = boundaryRoundBitPrefix_[static_cast<size_t>(boundaryRound)];
        const int endBit = boundaryRoundBitPrefix_[static_cast<size_t>(boundaryRound + 1)];
        const int resStart = boundaryRoundResidualPrefix_[static_cast<size_t>(boundaryRound)];
        const auto order = pmsf::collectBoundaryEdgesInOrder(mesh_);
        if (std::getenv("PMSF_SEAM_DEBUG")) std::cerr << "[orderbits-dec] round=" << boundaryRound << " order=" << order.size() << " hash=" << pmsf::hashBoundaryOrder(mesh_, order) << std::endl;
        const int roundBits = endBit - startBit;
        if (roundBits != static_cast<int>(order.size())) {
            std::cerr << "[seam-boundary-orderbits] round=" << boundaryRound
                      << " bitCount=" << roundBits << " orderSize=" << order.size()
                      << " (mismatch, skip)" << std::endl;
            return;
        }
        std::vector<int> positions;
        for (int bi = 0; bi < roundBits; ++bi) {
            if (boundaryRoundBits_[static_cast<size_t>(startBit + bi)] != 0) positions.push_back(bi);
        }
        int applied = 0;
        int resIdx = resStart;
        for (auto it = positions.rbegin(); it != positions.rend(); ++it) {
            const int pos = *it;
            if (pos < 0 || pos >= static_cast<int>(order.size())) continue;
            if (resIdx + 2 >= static_cast<int>(boundaryRoundResiduals_.size())) break;
            MCGAL::Halfedge* target = order[static_cast<size_t>(pos)];
            const int kSigned = boundaryRoundResiduals_[resIdx++];
            const int k = std::abs(kSigned);
            const int dx = boundaryRoundResiduals_[resIdx++];
            const int dy = boundaryRoundResiduals_[resIdx++];
            const int dz = boundaryRoundResiduals_[resIdx++];
            if (!target || target->isRemoved() || target->opposite() != nullptr || !target->face()) continue;
            const MCGAL::PointInt ha = mesh_->floatPosToInt(target->vertex()->point());
            const MCGAL::PointInt hb = mesh_->floatPosToInt(target->end_vertex()->point());
            const MCGAL::PointInt pi((ha.x() + hb.x()) / 2 + dx,
                                     (ha.y() + hb.y()) / 2 + dy,
                                     (ha.z() + hb.z()) / 2 + dz);
            const MCGAL::Point posPoint = mesh_->intPosToFloat(pi);
            if (k <= 2) {
                MCGAL::Vertex* nv = nullptr;
                if (pmsf::splitBoundaryEdgeInsertVertex(mesh_, target, posPoint, &nv) && nv) ++applied;
            } else {
                const int gid = target->face()->groupId();
                MCGAL::Point apexPos = (kSigned > 0) ? target->vertex()->point() : target->end_vertex()->point();
                if (pmsf::restoreBoundaryVertexFanCheap(mesh_, target, posPoint, k, gid, &apexPos)) ++applied;
            }
        }
        if (std::getenv("PMSF_SEAM_DEBUG"))
            std::cerr << "[seam-boundary-orderbits-apply] round=" << boundaryRound
                      << " applied=" << applied << " order=" << order.size() << std::endl;
        return;
    }
    if (effectiveMode == 11) {
        if (std::getenv("PMSF_SEAM_BOUNDARY_CANON")) pmsf::canonicalizeMeshOrder(mesh_);
        const int startBit = boundaryRoundBitPrefix_[static_cast<size_t>(boundaryRound)];
        const int endBit = boundaryRoundBitPrefix_[static_cast<size_t>(boundaryRound + 1)];
        const int resStart = boundaryRoundResidualPrefix_[static_cast<size_t>(boundaryRound)];
        std::vector<int> records;
        for (int bi = startBit; bi < endBit; ++bi) {
            if (bi < 0 || bi >= static_cast<int>(boundaryRoundBits_.size())) break;
            if (boundaryRoundBits_[static_cast<size_t>(bi)] != 0)
                records.push_back(static_cast<int>(records.size()));
        }
        int applied = 0;
        for (auto it = records.rbegin(); it != records.rend(); ++it) {
            const int rec = *it;
            const bool noVertex = mixedBoundary || std::getenv("PMSF_SEAM_BOUNDARY_ORDERED_NOVERTEX") != nullptr;
            const bool restoreIds = std::getenv("PMSF_SEAM_BOUNDARY_RESTORE_IDS") != nullptr;
            const bool restoreFaceIds = std::getenv("PMSF_SEAM_BOUNDARY_RESTORE_FACE_IDS") != nullptr;
            const int recStride = noVertex ? 4 : (std::getenv("PMSF_SEAM_BOUNDARY_ORDERED_NOFACE") ? 5 : (restoreFaceIds ? (std::getenv("PMSF_SEAM_BOUNDARY_RESTORE_HALF_IDS") ? 16 : 10) : (restoreIds ? 8 : 7)));
            const int ri = resStart + rec * recStride;
            if (ri + recStride - 1 >= static_cast<int>(boundaryRoundResiduals_.size())) break;
            const int edgeIndex = boundaryRoundResiduals_[ri];
            const int dx = boundaryRoundResiduals_[ri + 1];
            const int dy = boundaryRoundResiduals_[ri + 2];
            const int dz = boundaryRoundResiduals_[ri + 3];
            const int origVidx = noVertex ? -1 : boundaryRoundResiduals_[ri + 4];
            const int vPid = restoreIds || restoreFaceIds ? boundaryRoundResiduals_[ri + 5] : -1;
            const int f0idx = recStride >= 7 ? boundaryRoundResiduals_[ri + ((restoreIds || restoreFaceIds) ? 6 : 5)] : -1;
            const int f1idx = recStride >= 7 ? boundaryRoundResiduals_[ri + ((restoreIds || restoreFaceIds) ? 7 : 6)] : -1;
            const int f0Pid = restoreFaceIds ? boundaryRoundResiduals_[ri + 8] : -1;
            const int f1Pid = restoreFaceIds ? boundaryRoundResiduals_[ri + 9] : -1;
            const bool restoreHalfIds = std::getenv("PMSF_SEAM_BOUNDARY_RESTORE_HALF_IDS") != nullptr;
            const auto edges = pmsf::edgeflip::collectMeshBoundaryEdgesSorted(mesh_);
            if (std::getenv("PMSF_SEAM_DEBUG") && boundaryRound == 0)
                std::cerr << "[ord0] rec=" << rec << " edgeIndex=" << edgeIndex << " edges=" << edges.size() << " V=" << mesh_->size_of_vertices() << " F=" << mesh_->size_of_facets() << std::endl;
            if (edgeIndex < 0 || edgeIndex >= static_cast<int>(edges.size())) continue;
            MCGAL::Halfedge* target = edges[static_cast<size_t>(edgeIndex)];
            if (!target || target->isRemoved() || target->opposite() != nullptr || !target->face()) continue;
            const MCGAL::PointInt ha = mesh_->floatPosToInt(target->vertex()->point());
            const MCGAL::PointInt hb = mesh_->floatPosToInt(target->end_vertex()->point());
            const MCGAL::PointInt pi((ha.x() + hb.x()) / 2 + dx,
                                     (ha.y() + hb.y()) / 2 + dy,
                                     (ha.z() + hb.z()) / 2 + dz);
            MCGAL::Facet* splitF0 = target->face();
            MCGAL::Vertex* nv = nullptr;
            if (!pmsf::splitBoundaryEdgeInsertVertex(mesh_, target, mesh_->intPosToFloat(pi), &nv) || !nv) continue;
            if (restoreFaceIds) {
                if (splitF0 && f0Pid >= 0) splitF0->setPoolId(f0Pid);
                MCGAL::Facet* splitF1 = mesh_->faces().empty() ? nullptr : mesh_->faces().back();
                if (splitF1 && splitF1 != splitF0 && f1Pid >= 0) splitF1->setPoolId(f1Pid);
            }
            if (restoreIds || restoreFaceIds) { if (vPid >= 0) nv->setPoolId(vPid); }
            std::vector<MCGAL::Facet*> newFaces;
            for (MCGAL::Halfedge* h : pmsf::boundedRing(nv)) {
                if (h && h->face() && !h->face()->isRemoved()) {
                    bool seen = false;
                    for (auto* f : newFaces) if (f == h->face()) seen = true;
                    if (!seen) newFaces.push_back(h->face());
                }
            }
            if (f0idx >= 0 && newFaces.size() == 2) {
                auto& fv = mesh_->faces();
                std::vector<MCGAL::Facet*> live;
                live.reserve(fv.size());
                for (MCGAL::Facet* f : fv) {
                    if (!f || f->isRemoved()) continue;
                    if (f == newFaces[0] || f == newFaces[1]) continue;
                    live.push_back(f);
                }
                int a = std::min(f0idx, f1idx);
                int b = std::max(f0idx, f1idx);
                if (a < 0) a = 0;
                if (a > static_cast<int>(live.size())) a = static_cast<int>(live.size());
                live.insert(live.begin() + a, newFaces[0]);
                if (b > static_cast<int>(live.size())) b = static_cast<int>(live.size());
                live.insert(live.begin() + b, newFaces[1]);
                fv = live;
            }
            if (!noVertex) {
                auto& vv = mesh_->vertices();
                for (size_t vi = 0; vi < vv.size(); ++vi) {
                    if (vv[vi] == nv) { vv.erase(vv.begin() + vi); break; }
                }
                int at = std::max(0, std::min(origVidx, static_cast<int>(vv.size())));
                vv.insert(vv.begin() + at, nv);
                mesh_->garbage_collection();
            }
            ++applied;
        }
        if (std::getenv("PMSF_SEAM_DEBUG"))
            std::cerr << "[seam-boundary-ordered-apply] round=" << boundaryRound
                      << " applied=" << applied << std::endl;
        return;
    }
    if (effectiveMode == 13) {
        const int startBit = boundaryRoundBitPrefix_[static_cast<size_t>(boundaryRound)];
        const int endBit = boundaryRoundBitPrefix_[static_cast<size_t>(boundaryRound + 1)];
        const int resStart = boundaryRoundResidualPrefix_[static_cast<size_t>(boundaryRound)];
        std::vector<int> records;
        for (int bi = startBit; bi < endBit; ++bi) {
            if (bi < 0 || bi >= static_cast<int>(boundaryRoundBits_.size())) break;
            if (boundaryRoundBits_[static_cast<size_t>(bi)] != 0)
                records.push_back(static_cast<int>(records.size()));
        }
        int applied = 0;
        if (std::getenv("PMSF_SEAM_DEBUG"))
            std::cerr << "[seam-boundary13-start] round=" << boundaryRound
                      << " records=" << records.size() << std::endl;
        for (auto it = records.rbegin(); it != records.rend(); ++it) {
            const int rec = *it;
            const int ri = resStart + rec * 10;
            if (ri + 9 >= static_cast<int>(boundaryRoundResiduals_.size())) break;
            const MCGAL::PointInt aq(boundaryRoundResiduals_[ri],
                                     boundaryRoundResiduals_[ri + 1],
                                     boundaryRoundResiduals_[ri + 2]);
            const MCGAL::PointInt bq(boundaryRoundResiduals_[ri + 3],
                                     boundaryRoundResiduals_[ri + 4],
                                     boundaryRoundResiduals_[ri + 5]);
            const int edgeGid = boundaryRoundResiduals_[ri + 6];
            const int dx = boundaryRoundResiduals_[ri + 7];
            const int dy = boundaryRoundResiduals_[ri + 8];
            const int dz = boundaryRoundResiduals_[ri + 9];
            const auto edges = pmsf::edgeflip::collectMeshBoundaryEdgesSorted(mesh_);
            MCGAL::Halfedge* target = nullptr;
            for (MCGAL::Halfedge* e : edges) {
                if (!e || e->isRemoved() || e->opposite() != nullptr || !e->face()) continue;
                if (!e->vertex() || !e->end_vertex()) continue;
                if (edgeGid >= 0 && e->face()->groupId() != edgeGid) continue;
                auto qa = mesh_->floatPosToInt(e->vertex()->point());
                auto qb = mesh_->floatPosToInt(e->end_vertex()->point());
                if ((qa == aq && qb == bq) || (qa == bq && qb == aq)) { target = e; break; }
            }
            if (!target) {
                if (std::getenv("PMSF_SEAM_DEBUG"))
                    std::cerr << "[seam-boundary13-miss-edge] round=" << boundaryRound
                              << " a=" << aq.x() << "," << aq.y() << "," << aq.z() << std::endl;
                continue;
            }
            const MCGAL::PointInt ha = mesh_->floatPosToInt(target->vertex()->point());
            const MCGAL::PointInt hb = mesh_->floatPosToInt(target->end_vertex()->point());
            const MCGAL::PointInt pi((ha.x() + hb.x()) / 2 + dx,
                                     (ha.y() + hb.y()) / 2 + dy,
                                     (ha.z() + hb.z()) / 2 + dz);
            if (pmsf::restoreBoundaryCollapseViaFaces(mesh_, target, mesh_->intPosToFloat(pi),
                                                      edgeGid))
                ++applied;
        }
        if (std::getenv("PMSF_SEAM_DEBUG"))
            std::cerr << "[seam-boundary13-apply] round=" << boundaryRound
                      << " applied=" << applied << std::endl;
        return;
    }
    if (effectiveMode == 8) {
        const int startBit = boundaryRoundBitPrefix_[static_cast<size_t>(boundaryRound)];
        const int endBit = boundaryRoundBitPrefix_[static_cast<size_t>(boundaryRound + 1)];
        const int resStart = boundaryRoundResidualPrefix_[static_cast<size_t>(boundaryRound)];
        std::vector<int> records;
        for (int bi = startBit; bi < endBit; ++bi) {
            if (bi < 0 || bi >= static_cast<int>(boundaryRoundBits_.size())) break;
            if (boundaryRoundBits_[static_cast<size_t>(bi)] != 0) {
                records.push_back(static_cast<int>(records.size()));
            }
        }
        int applied = 0;
        if (std::getenv("PMSF_SEAM_DEBUG"))
            std::cerr << "[seam-boundary-start] round=" << boundaryRound
                      << " records=" << records.size() << std::endl;
        for (auto it = records.rbegin(); it != records.rend(); ++it) {
            const int rec = *it;
            const int resIdx = resStart + rec * 4;
            if (resIdx + 3 >= static_cast<int>(boundaryRoundResiduals_.size())) break;
            const int edgeIndex = boundaryRoundResiduals_[resIdx];
            const int dx = boundaryRoundResiduals_[resIdx + 1];
            const int dy = boundaryRoundResiduals_[resIdx + 2];
            const int dz = boundaryRoundResiduals_[resIdx + 3];
            const auto edges = pmsf::edgeflip::collectMeshBoundaryEdgesSorted(mesh_);
            if (edgeIndex < 0 || edgeIndex >= static_cast<int>(edges.size())) {
                if (std::getenv("PMSF_SEAM_DEBUG"))
                    std::cerr << "[seam-boundary-miss-index] " << edgeIndex << std::endl;
                continue;
            }
            MCGAL::Halfedge* target = edges[static_cast<size_t>(edgeIndex)];
            if (target && !target->isRemoved() && target->opposite() == nullptr && target->face()) {
                const MCGAL::PointInt ha = mesh_->floatPosToInt(target->vertex()->point());
                const MCGAL::PointInt hb = mesh_->floatPosToInt(target->end_vertex()->point());
                const MCGAL::PointInt pi((ha.x() + hb.x()) / 2 + dx,
                                         (ha.y() + hb.y()) / 2 + dy,
                                         (ha.z() + hb.z()) / 2 + dz);
                const MCGAL::Point p = mesh_->intPosToFloat(pi);
                if (std::getenv("PMSF_SEAM_DEBUG"))
                    std::cerr << "[seam-boundary-op] round=" << boundaryRound << " rec=" << rec
                              << " edgeIndex=" << edgeIndex << " edges=" << edges.size()
                              << " V=" << mesh_->size_of_vertices() << std::endl;
                if (pmsf::splitBoundaryEdgeInsertVertex(mesh_, target, p, nullptr)) {
                    ++applied;
                } else if (std::getenv("PMSF_SEAM_DEBUG")) {
                    std::cerr << "[seam-boundary-miss-split]" << std::endl;
                }
            } else if (std::getenv("PMSF_SEAM_DEBUG")) {
                std::cerr << "[seam-boundary-miss-edge] rec=" << rec << std::endl;
            }
        }
        if (std::getenv("PMSF_BOUNDARY_DEBUG") || std::getenv("PMSF_SEAM_DEBUG"))
            std::cerr << "[seam-boundary-apply] round=" << boundaryRound
                      << " applied=" << applied << std::endl;
        return;
    }
    if (boundaryApplyMode_ == 7) {
        const int startBit = boundaryRoundBitPrefix_[static_cast<size_t>(boundaryRound)];
        const int endBit = boundaryRoundBitPrefix_[static_cast<size_t>(boundaryRound + 1)];
        int resIdx = boundaryRoundResidualPrefix_[static_cast<size_t>(boundaryRound)];
        int applied = 0;
        for (int bi = startBit; bi < endBit; ++bi) {
            if (bi < 0 || bi >= static_cast<int>(boundaryRoundBits_.size())) break;
            if (boundaryRoundBits_[static_cast<size_t>(bi)] == 0) continue;
            if (resIdx + 3 >= static_cast<int>(boundaryRoundResiduals_.size())) break;
            const int nOrig = boundaryRoundResiduals_[resIdx++];
            const int nPost = boundaryRoundResiduals_[resIdx++];
            const int nGroups = boundaryRoundResiduals_[resIdx++];
            const int nBoundary = boundaryRoundResiduals_[resIdx++];
            auto readPoint = [&]() {
                const MCGAL::Point p = pmsf::patchPoint(mesh_, &boundaryRoundResiduals_[resIdx]);
                resIdx += 3;
                return p;
            };
            pmsf::BoundaryFanPatch fan;
            fan.removed = readPoint();
            for (int i = 0; i < nOrig; ++i) {
                std::array<MCGAL::Point,3> tri;
                for (int k = 0; k < 3; ++k) tri[k] = readPoint();
                fan.origFaces.push_back(tri);
            }
            for (int i = 0; i < nPost; ++i) {
                std::array<MCGAL::Point,3> tri;
                for (int k = 0; k < 3; ++k) tri[k] = readPoint();
                fan.postFaces.push_back(tri);
            }
            fan.origGroups.assign(boundaryRoundResiduals_.begin() + resIdx,
                                  boundaryRoundResiduals_.begin() + resIdx + nGroups);
            resIdx += nGroups;
            for (int i = 0; i < nBoundary; ++i) {
                const MCGAL::Point a = readPoint();
                const MCGAL::Point b = readPoint();
                fan.boundaryEdges.push_back({a, b});
            }
            fan.valid = true;
            if (pmsf::applyBoundaryFanPatch(mesh_, fan)) ++applied;
        }
        if (std::getenv("PMSF_BOUNDARY_DEBUG"))
            std::cerr << "[boundary-fan-apply] round=" << boundaryRound
                      << " applied=" << applied << std::endl;
        return;
    }
    if (boundaryApplyMode_ == 5 || boundaryApplyMode_ == 6) {
        const int startBit = boundaryRoundBitPrefix_[static_cast<size_t>(boundaryRound)];
        const int endBit = boundaryRoundBitPrefix_[static_cast<size_t>(boundaryRound + 1)];
        const int resStart = boundaryRoundResidualPrefix_[static_cast<size_t>(boundaryRound)];
        int resIdx = resStart;
        int applied = 0;
        for (int bi = startBit; bi < endBit; ++bi) {
            if (bi < 0 || bi >= static_cast<int>(boundaryRoundBits_.size())) break;
            if (boundaryRoundBits_[static_cast<size_t>(bi)] == 0) continue;
            if (resIdx + 25 >= static_cast<int>(boundaryRoundResiduals_.size())) break;
            auto readPoint = [&](int off) {
                return pmsf::patchPoint(mesh_, &boundaryRoundResiduals_[resIdx + off]);
            };
            pmsf::BoundaryCollapsePatch patch;
            patch.removed = readPoint(0);
            patch.a = readPoint(3); patch.b = readPoint(6); patch.c = readPoint(9);
            patch.f0ProxyA = readPoint(12); patch.f0ProxyB = readPoint(15);
            patch.f1ProxyA = readPoint(18); patch.f1ProxyB = readPoint(21);
            patch.g0 = boundaryRoundResiduals_[resIdx + 24];
            patch.g1 = boundaryRoundResiduals_[resIdx + 25];
            patch.valid = true;
            if (pmsf::applyBoundaryCollapsePatch(mesh_, patch)) ++applied;
            resIdx += 26;
        }
        if (std::getenv("PMSF_BOUNDARY_DEBUG"))
            std::cerr << "[boundary-patch-round-apply] round=" << boundaryRound
                      << " applied=" << applied << std::endl;
        return;
    }
    if (boundaryApplyMode_ == 2) {
        const int startBit = boundaryRoundBitPrefix_[static_cast<size_t>(boundaryRound)];
        const int endBit = boundaryRoundBitPrefix_[static_cast<size_t>(boundaryRound + 1)];
        const int resStart = boundaryRoundResidualPrefix_[static_cast<size_t>(boundaryRound)];
        const auto edges = pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(mesh_);
        int resIdx = resStart;
        int applied = 0;
        for (int bi = startBit; bi < endBit; ++bi) {
            if (bi < 0 || bi >= static_cast<int>(boundaryRoundBits_.size())) break;
            if (boundaryRoundBits_[static_cast<size_t>(bi)] == 0) continue;
            if (resIdx + 8 >= static_cast<int>(boundaryRoundResiduals_.size())) break;
            const MCGAL::PointInt aInt(boundaryRoundResiduals_[resIdx], boundaryRoundResiduals_[resIdx+1], boundaryRoundResiduals_[resIdx+2]);
            const MCGAL::PointInt bInt(boundaryRoundResiduals_[resIdx+3], boundaryRoundResiduals_[resIdx+4], boundaryRoundResiduals_[resIdx+5]);
            const MCGAL::PointInt pInt(boundaryRoundResiduals_[resIdx+6], boundaryRoundResiduals_[resIdx+7], boundaryRoundResiduals_[resIdx+8]);
            resIdx += 9;
            MCGAL::Halfedge* target = nullptr;
            for (MCGAL::Halfedge* h : edges) {
                if (!h || h->isRemoved() || h->opposite() != nullptr || !h->face()) continue;
                const MCGAL::PointInt ha = mesh_->floatPosToInt(h->vertex()->point());
                const MCGAL::PointInt hb = mesh_->floatPosToInt(h->end_vertex()->point());
                if ((ha.x()==aInt.x() && ha.y()==aInt.y() && ha.z()==aInt.z() &&
                     hb.x()==bInt.x() && hb.y()==bInt.y() && hb.z()==bInt.z()) ||
                    (ha.x()==bInt.x() && ha.y()==bInt.y() && ha.z()==bInt.z() &&
                     hb.x()==aInt.x() && hb.y()==aInt.y() && hb.z()==aInt.z())) {
                    target = h; break;
                }
            }
            if (target) {
                const MCGAL::Point p = mesh_->intPosToFloat(pInt);
                if (pmsf::splitBoundaryEdgeInsertVertex(mesh_, target, p, nullptr)) ++applied;
            }
        }
        if (std::getenv("PMSF_BOUNDARY_DEBUG"))
            std::cerr << "[boundary-patch-apply] round=" << boundaryRound << " applied=" << applied << std::endl;
        return;
    }
    if (boundaryApplyMode_ == 1) {
        const int startBit = boundaryRoundBitPrefix_[static_cast<size_t>(boundaryRound)];
        const int endBit = boundaryRoundBitPrefix_[static_cast<size_t>(boundaryRound + 1)];
        const auto verts = pmsf::edgeflip::collectMeshBoundaryVerticesCanonical(mesh_);
        int resIdx = boundaryRoundResidualPrefix_[static_cast<size_t>(boundaryRound)];
        int applied = 0;
        for (int bi = startBit; bi < endBit; ++bi) {
            if (bi < 0 || bi >= static_cast<int>(boundaryRoundBits_.size())) break;
            if (boundaryRoundBits_[static_cast<size_t>(bi)] == 0) continue;
            const int vi = bi - startBit;
            if (resIdx + 2 >= static_cast<int>(boundaryRoundResiduals_.size())) break;
            if (vi >= 0 && vi < static_cast<int>(verts.size())) {
                MCGAL::Vertex* v = verts[static_cast<size_t>(vi)];
                if (v && !v->isRemoved()) {
                    const MCGAL::PointInt cur = mesh_->floatPosToInt(v->point());
                    const MCGAL::PointInt orig(cur.x() + boundaryRoundResiduals_[resIdx],
                                               cur.y() + boundaryRoundResiduals_[resIdx + 1],
                                               cur.z() + boundaryRoundResiduals_[resIdx + 2]);
                    v->setVertexPoint(mesh_->intPosToFloat(orig));
                    ++applied;
                }
            }
            resIdx += 3;
        }
        if (std::getenv("PMSF_BOUNDARY_DEBUG"))
            std::cerr << "[boundary-shift-apply] round=" << boundaryRound << " applied=" << applied << std::endl;
        return;
    }
    if (boundaryRound + 1 >= static_cast<int>(boundaryRoundResidualPrefix_.size())) return;
    const int startBit = boundaryRoundBitPrefix_[static_cast<size_t>(boundaryRound)];
    const int endBit = boundaryRoundBitPrefix_[static_cast<size_t>(boundaryRound + 1)];
    const int resStart = boundaryRoundResidualPrefix_[static_cast<size_t>(boundaryRound)];
    const int resEnd = boundaryRoundResidualPrefix_[static_cast<size_t>(boundaryRound + 1)];
    if (resEnd <= resStart) return;
    const auto edges = pmsf::edgeflip::collectMeshBoundaryEdgesCanonical(mesh_);
    int resIdx = resStart;
    int applied = 0;
    for (int bi = startBit; bi < endBit; ++bi) {
        if (bi < 0 || bi >= static_cast<int>(boundaryRoundBits_.size())) break;
        if (boundaryRoundBits_[static_cast<size_t>(bi)] == 0) continue;
        if (resIdx + 2 >= static_cast<int>(boundaryRoundResiduals_.size())) break;
        const int ei = bi - startBit;
        if (ei < 0 || ei >= static_cast<int>(edges.size())) { resIdx += 3; continue; }
        MCGAL::Halfedge* h = edges[static_cast<size_t>(ei)];
        if (h && !h->isRemoved() && h->opposite() == nullptr && h->face()) {
            const MCGAL::PointInt pi(boundaryRoundResiduals_[resIdx],
                                     boundaryRoundResiduals_[resIdx + 1],
                                     boundaryRoundResiduals_[resIdx + 2]);
            const MCGAL::Point p = mesh_->intPosToFloat(pi);
            pmsf::splitBoundaryEdgeInsertVertex(mesh_, h, p, nullptr);
            ++applied;
        }
        resIdx += 3;
    }
    if (std::getenv("PMSF_BOUNDARY_DEBUG")) {
        std::cerr << "[boundary-apply] round=" << boundaryRound
                  << " bits=" << (endBit - startBit) << " applied=" << applied << std::endl;
    }
    // Mirror the encoder's canonicalizeTraversalProxies() call after a staged round.
    // The inverse restores faces at different pool slots; later rounds' proxy-
    // dependent residual prediction / BFS order drifts unless the live decoder
    // mesh is normalized to the same traversal state.
    if (mesh_ && std::getenv("PMSF_SC_CANON_AFTER_ANY_APPLY"))
        canonicalizeTraversalProxies(mesh_);
    if (mesh_ && std::getenv("PMSF_SC_CANON_DIRECTED_AFTER_ANY_APPLY"))
        canonicalizeDecoderProxiesDirected(mesh_);
}

namespace {
// Diagnostic order bridge: reorder the live decoder mesh vertices/faces to the
// exact orders stored in a reference OFF (encoder pre-stage state).  Used to test
// whether face/vertex order is what breaks later-round symbol replay.
bool scOrderBridgeFromOff(const std::shared_ptr<MCGAL::Mesh>& mesh, const char* path) {
    if (!mesh || !path) return false;
    FILE* f = std::fopen(path, "r");
    if (!f) return false;
    char line[512];
    if (!std::fgets(line, sizeof(line), f)) { std::fclose(f); return false; }
    // skip to counts line
    int nv = 0, nf = 0;
    while (std::fgets(line, sizeof(line), f)) {
        if (line[0] == '#' || line[0] == '\n') continue;
        if (std::sscanf(line, "%d %d", &nv, &nf) == 2) break;
    }
    std::vector<MCGAL::Point> ov(static_cast<std::size_t>(std::max(0, nv)));
    for (int i = 0; i < nv; ++i) {
        if (!std::fgets(line, sizeof(line), f)) { std::fclose(f); return false; }
        double x = 0, y = 0, z = 0;
        if (std::sscanf(line, "%lf %lf %lf", &x, &y, &z) < 3) { std::fclose(f); return false; }
        ov[static_cast<std::size_t>(i)] = MCGAL::Point(static_cast<float>(x), static_cast<float>(y),
                                                       static_cast<float>(z));
    }
    std::vector<std::array<int, 3>> of(static_cast<std::size_t>(std::max(0, nf)));
    for (int i = 0; i < nf; ++i) {
        if (!std::fgets(line, sizeof(line), f)) { std::fclose(f); return false; }
        int k = 0, a = 0, b = 0, c = 0;
        if (std::sscanf(line, "%d %d %d %d", &k, &a, &b, &c) < 4) { std::fclose(f); return false; }
        of[static_cast<std::size_t>(i)] = {a, b, c};
    }
    std::fclose(f);
    // map OFF vertex -> decoder vertex (nearest, greedy by grid cell)
    auto addLive = [&](MCGAL::Vertex* v) { (void)v; };
    std::vector<MCGAL::Vertex*> liveV;
    for (MCGAL::Vertex* v : mesh->vertices())
        if (v && !v->isRemoved()) liveV.push_back(v);
    auto qKey = [&](MCGAL::Point pt) {
        return std::array<long long, 3>{static_cast<long long>(std::llround(pt[0] * 1e4)),
                                        static_cast<long long>(std::llround(pt[1] * 1e4)),
                                        static_cast<long long>(std::llround(pt[2] * 1e4))};
    };
    std::unordered_map<long long, std::vector<MCGAL::Vertex*>> grid;
    auto hash3 = [](const std::array<long long, 3>& q) {
        return static_cast<long long>(q[0] * 73856093LL) ^ (q[1] * 19349663LL) ^
               (q[2] * 83492791LL);
    };
    for (MCGAL::Vertex* v : liveV) grid[hash3(qKey(v->point()))].push_back(v);
    std::unordered_map<MCGAL::Vertex*, bool> used;
    std::vector<MCGAL::Vertex*> mapped(ov.size(), nullptr);
    for (std::size_t i = 0; i < ov.size(); ++i) {
        const auto q = qKey(ov[i]);
        for (int dx = -1; dx <= 1 && !mapped[i]; ++dx)
            for (int dy = -1; dy <= 1 && !mapped[i]; ++dy)
                for (int dz = -1; dz <= 1 && !mapped[i]; ++dz) {
                    const auto h = hash3({q[0] + dx, q[1] + dy, q[2] + dz});
                    auto it = grid.find(h);
                    if (it == grid.end()) continue;
                    for (MCGAL::Vertex* v : it->second) {
                        if (used[v]) continue;
                        MCGAL::Point pp = v->point();
                        if (std::abs(pp[0] - ov[i][0]) > 1e-2f || std::abs(pp[1] - ov[i][1]) > 1e-2f ||
                            std::abs(pp[2] - ov[i][2]) > 1e-2f)
                            continue;
                        mapped[i] = v;
                        used[v] = true;
                        break;
                    }
                }
        if (!mapped[i]) { addLive(nullptr); return false; }
    }
    // new vertex vector: OFF order, then any unmapped live, then removed tombstones
    std::vector<MCGAL::Vertex*> newVerts;
    newVerts.reserve(mesh->vertices().size());
    for (MCGAL::Vertex* v : mapped) newVerts.push_back(v);
    for (MCGAL::Vertex* v : liveV)
        if (!used[v]) newVerts.push_back(v);
    for (MCGAL::Vertex* v : mesh->vertices())
        if (v && v->isRemoved()) newVerts.push_back(v);
    // map OFF face -> decoder face by vertex-pointer triple
    std::unordered_map<std::size_t, std::vector<MCGAL::Facet*>> byKey;
    auto faceKey = [&](MCGAL::Facet* fc) {
        const auto vs = pmsf::sbdetail::faceVertices(fc);
        std::array<MCGAL::Vertex*, 3> t{vs[0], vs[1], vs[2]};
        std::sort(t.begin(), t.end());
        return std::hash<MCGAL::Vertex*>()(t[0]) * 1315423911u ^
               std::hash<MCGAL::Vertex*>()(t[1]) * 2654435761u ^
               std::hash<MCGAL::Vertex*>()(t[2]);
    };
    std::vector<MCGAL::Facet*> liveF;
    for (MCGAL::Facet* fc : mesh->faces())
        if (fc && !fc->isRemoved() && fc->proxyHalfedge()) liveF.push_back(fc);
    std::unordered_map<MCGAL::Facet*, bool> fused;
    std::vector<MCGAL::Facet*> newFaces;
    newFaces.reserve(liveF.size());
    int matched = 0;
    for (const auto& tri : of) {
        MCGAL::Vertex* want[3] = {mapped[static_cast<std::size_t>(tri[0])],
                                  mapped[static_cast<std::size_t>(tri[1])],
                                  mapped[static_cast<std::size_t>(tri[2])]};
        MCGAL::Facet* hit = nullptr;
        for (MCGAL::Facet* fc : liveF) {
            if (fused[fc]) continue;
            const auto vs = pmsf::sbdetail::faceVertices(fc);
            if (!vs[0]) continue;
            auto has = [&](MCGAL::Vertex* x) {
                return vs[0] == x || vs[1] == x || vs[2] == x;
            };
            if (has(want[0]) && has(want[1]) && has(want[2])) { hit = fc; break; }
        }
        if (!hit) continue;
        fused[hit] = true;
        newFaces.push_back(hit);
        ++matched;
    }
    for (MCGAL::Facet* fc : liveF)
        if (!fused[fc]) newFaces.push_back(fc);
    for (MCGAL::Facet* fc : mesh->faces())
        if (fc && fc->isRemoved()) newFaces.push_back(fc);
    mesh->vertices().swap(newVerts);
    mesh->faces().swap(newFaces);
    if (std::getenv("PMSF_SC_DEBUG"))
        std::fprintf(stderr, "[sc-order-bridge] matched %d/%d faces\n", matched, nf);
    return true;
}
}  // namespace

void OptimizedBasicDeserializeOperator::finalizeBoundaryInverses(int rounds) {
    if (boundaryApplyMode_ != 71) return;
    if (!bndV3Parse(boundaryRoundResiduals_)) return;
    BndV3Payload& st = bndV3State();
    if (!st.valid || !st.decoupled) return;
    for (int r = rounds - 1; r >= 0; --r) {
        if (boundaryAppliedRounds_.count(r)) continue;
        // Encoder canonicalizes proxies right after each stage round; mirror that
        // before reversing a round so the compact inverse sees the same proxy state.
        if (mesh_) canonicalizeTraversalProxies(mesh_);
            boundaryAppliedRounds_.insert(r);
            bndV3ApplyRound(mesh_, r);
    }
    if (mesh_) canonicalizeTraversalProxies(mesh_);
}

void OptimizedBasicDeserializeOperator::deserializeOneRound() {
    static int s_round = 0;
    if (std::getenv("PMSF_BC_V2_DEBUG") && mesh_) {
        std::size_t lv = 0, lf = 0;
        for (MCGAL::Vertex* v : mesh_->vertices())
            if (v && !v->isRemoved()) ++lv;
        for (MCGAL::Facet* f : mesh_->faces())
            if (f && !f->isRemoved()) ++lf;
        std::fprintf(stderr,
                     "[bc-v2-call] s_round=%d liveV=%zu liveF=%zu rounds=%zu mode=%d\n", s_round,
                     lv, lf, boundaryRoundBitCounts_.size(), boundaryApplyMode_);
        if (std::getenv("PMSF_BC_V2_GIDS_ALL")) {
            char sp[512];
            std::snprintf(sp, sizeof(sp), "/tmp/bc_v2_dec_state_s%02d.gids", s_round);
            if (FILE* gf = std::fopen(sp, "w")) {
                const int mq = (mesh_->i_nbQuantBits > 0) ? ((1 << mesh_->i_nbQuantBits) - 1)
                                                          : (1 << 30);
                for (MCGAL::Facet* ff : mesh_->faces()) {
                    if (!ff || ff->isRemoved() || !ff->proxyHalfedge()) continue;
                    const auto fv = pmsf::sbdetail::faceVertices(ff);
                    if (!fv[0]) continue;
                    std::vector<std::array<int, 3>> qs;
                    for (int k = 0; k < 3; ++k) {
                        const auto q = mesh_->floatPosToInt(fv[k]->point());
                        qs.push_back({std::clamp(q.x(), 0, mq), std::clamp(q.y(), 0, mq),
                                      std::clamp(q.z(), 0, mq)});
                    }
                    std::sort(qs.begin(), qs.end());
                    std::fprintf(gf, "%d %d %d %d %d %d %d %d %d %d\n", ff->groupId(),
                                 qs[0][0], qs[0][1], qs[0][2], qs[1][0], qs[1][1], qs[1][2],
                                 qs[2][0], qs[2][1], qs[2][2]);
                }
                std::fclose(gf);
            }
        }
    }
    if (s_round == 0) {
        if (std::getenv("PMSF_BC_ANCHOR_DIAG") && mesh_) {
            const auto hs = pmsf::groupBoundaryEnumHashesAllLoops(mesh_.get());
            for (const auto& gh : hs)
                std::fprintf(stderr, "[bc-diag-final-dec] g=%d hash=%llx\n", gh.first, gh.second);
            mesh_->dumpto_oldtype("/tmp/bc_dec_final.off");
        }
        if (const char* bg = std::getenv("PMSF_SC_DEC_BASE_GID")) {
            if (FILE* ff = std::fopen(bg, "w"); ff && mesh_) {
                const int mq = (mesh_->i_nbQuantBits > 0) ? ((1 << mesh_->i_nbQuantBits) - 1) : (1 << 30);
                for (MCGAL::Facet* ffac : mesh_->faces()) {
                    if (!ffac || ffac->isRemoved() || !ffac->proxyHalfedge()) continue;
                    const auto vs = pmsf::sbdetail::faceVertices(ffac);
                    if (!vs[0]) continue;
                    std::vector<std::array<int, 3>> qs;
                    for (int k = 0; k < 3; ++k) {
                        const auto q = mesh_->floatPosToInt(vs[k]->point());
                        qs.push_back({std::clamp(q.x(), 0, mq), std::clamp(q.y(), 0, mq),
                                      std::clamp(q.z(), 0, mq)});
                    }
                    std::sort(qs.begin(), qs.end());
                    std::fprintf(ff, "%d %d %d %d %d %d %d %d %d %d\n", ffac->groupId(),
                                 qs[0][0], qs[0][1], qs[0][2], qs[1][0], qs[1][1], qs[1][2],
                                 qs[2][0], qs[2][1], qs[2][2]);
                }
                std::fclose(ff);
            }
        }
    }
    if (std::getenv("PMSF_GBS_APPLY_BEFORE") && !boundaryRoundBitCounts_.empty()) {
        const int inverseRound = static_cast<int>(boundaryRoundBitCounts_.size()) - 1 - s_round;
        if (inverseRound >= 0) applyBoundaryPatchesForRound(inverseRound);
    }
    if (std::getenv("PMSF_SEAM_BOUNDARY_ROUND_END") &&
        (boundaryApplyMode_ == 8 || boundaryApplyMode_ == 11 ||
         boundaryApplyMode_ == 13 || boundaryApplyMode_ == 20) &&
        !boundaryRoundBitCounts_.empty()) {
        const int inverseRound = static_cast<int>(boundaryRoundBitCounts_.size()) - 1 - s_round;
        if (inverseRound >= 0) applyBoundaryPatchesForRound(inverseRound);
    }
    // The artifact itself records the boundary-stage codec (apply mode 61..64);
    // decode must not depend on the caller re-setting the encoder-side env switches.
    bool scApplyBoundaryBeforeInterior = false;
    // 0 = IFB (interior->flip->boundary, boundary inverse before flip undo),
    // 1 = BIF (boundary->interior->flip, inverse after interior undo),
    // 2 = IBF (interior->boundary->flip, inverse after flip undo, before interior).
    int scRoundStageOrder = 0;
    const int inverseRoundCandidate = static_cast<int>(boundaryRoundBitCounts_.size()) - 1 - s_round;
    if (!boundaryRoundBitCounts_.empty() && std::getenv("PMSF_SC_BOUNDARY_PRE") == nullptr) {
        if (boundaryApplyMode_ == 71) {
            bndV3Parse(boundaryRoundResiduals_);
            BndV3Payload& st = bndV3State();
            if (st.valid && inverseRoundCandidate >= 0 &&
                inverseRoundCandidate < static_cast<int>(st.orders.size())) {
                scRoundStageOrder = st.orders[static_cast<std::size_t>(inverseRoundCandidate)];
                scApplyBoundaryBeforeInterior =
                    (scRoundStageOrder == 0 || scRoundStageOrder == 4);
                if (mesh_) {
                    MCGAL::Halfedge* sh =
                        bndV3SeedHalfedge(mesh_.get(), inverseRoundCandidate);
                    if (sh) seed = sh;
                }
            }
            if (std::getenv("PMSF_BC_V2_DEBUG"))
                std::fprintf(stderr, "[bnd-v3-marker] inverseRound=%d order=%d before=%d\n",
                             inverseRoundCandidate, scRoundStageOrder,
                             scApplyBoundaryBeforeInterior ? 1 : 0);
        } else if (boundaryApplyMode_ == 61 || boundaryApplyMode_ == 62 ||
                   boundaryApplyMode_ == 63 || boundaryApplyMode_ == 64) {
            scApplyBoundaryBeforeInterior = true;
        } else if (boundaryApplyMode_ == 70) {
            int marker = -1;
            if (!boundaryRoundResiduals_.empty()) {
                int cursor = 0;
                const int nA = boundaryRoundResiduals_.empty()
                                   ? 0
                                   : boundaryRoundResiduals_[cursor++];
                cursor += 2 * nA;
                for (int r = 0; r < static_cast<int>(boundaryRoundBitCounts_.size()); ++r) {
                    if (boundaryRoundBitCounts_[static_cast<std::size_t>(r)] <= 0) continue;
                    int ones = 0;
                    const int st = boundaryRoundBitPrefix_[static_cast<std::size_t>(r)];
                    const int en = boundaryRoundBitPrefix_[static_cast<std::size_t>(r + 1)];
                    for (int bi = st; bi < en && bi < static_cast<int>(boundaryRoundBits_.size()); ++bi)
                        if (boundaryRoundBits_[static_cast<std::size_t>(bi)] != 0) ++ones;
                    if (cursor < static_cast<int>(boundaryRoundResiduals_.size()) &&
                        r == inverseRoundCandidate)
                        marker = boundaryRoundResiduals_[static_cast<std::size_t>(cursor)];
                    cursor += 1 + 6 * ones;
                }
            }
            scRoundStageOrder = (marker >= 0 && marker <= 5) ? marker : 0;
            scApplyBoundaryBeforeInterior = (scRoundStageOrder == 0 || scRoundStageOrder == 4);
            if (std::getenv("PMSF_BC_V2_DEBUG"))
                std::fprintf(stderr, "[bc-v2-marker] inverseRound=%d marker=%d before=%d\n",
                             inverseRoundCandidate, marker, scApplyBoundaryBeforeInterior ? 1 : 0);
        } else if (boundaryApplyMode_ == 66) {
            // Mixed ordering: the first int of each round's payload is the marker
            // (0 = stage after the interior elimination -> decoder applies inverse
            // before undoing the round; 1 = stage before -> decoder applies after).
            const int r = static_cast<int>(boundaryRoundBitCounts_.size()) - 1 - s_round;
            if (r >= 0 && r + 1 < static_cast<int>(boundaryRoundResidualPrefix_.size())) {
                const int st = boundaryRoundResidualPrefix_[static_cast<std::size_t>(r)];
                const int en = boundaryRoundResidualPrefix_[static_cast<std::size_t>(r + 1)];
                if (st >= 0 && st < en && st < static_cast<int>(boundaryRoundResiduals_.size()))
                    scApplyBoundaryBeforeInterior =
                        boundaryRoundResiduals_[static_cast<std::size_t>(st)] == 0;
            }
        }
    }
    if (scApplyBoundaryBeforeInterior) {
        const int inverseRound = static_cast<int>(boundaryRoundBitCounts_.size()) - 1 - s_round;
        if (inverseRound >= 0) {
            applyBoundaryPatchesForRound(inverseRound);
            if (const char* pif = std::getenv("PMSF_SC_DEC_POSTINV_FLAGS"); pif && mesh_) {
                if (FILE* ff = std::fopen(pif, "a")) {
                    std::fprintf(ff, "ROUND %d\n", inverseRound);
                    const int mqf = (mesh_->i_nbQuantBits > 0)
                                        ? ((1 << mesh_->i_nbQuantBits) - 1)
                                        : (1 << 30);
                    for (MCGAL::Facet* ffac : mesh_->faces()) {
                        if (!ffac || ffac->isRemoved() || !ffac->proxyHalfedge()) continue;
                        const auto fvs = pmsf::sbdetail::faceVertices(ffac);
                        if (!fvs[0]) continue;
                        std::vector<std::array<int, 3>> qs;
                        for (int k = 0; k < 3; ++k) {
                            const auto q = mesh_->floatPosToInt(fvs[k]->point());
                            qs.push_back({std::clamp(q.x(), 0, mqf), std::clamp(q.y(), 0, mqf),
                                          std::clamp(q.z(), 0, mqf)});
                        }
                        std::sort(qs.begin(), qs.end());
                        int amask2 = 0;
                        for (int pos = 0; pos < 3; ++pos) {
                            for (MCGAL::Halfedge* hh : {ffac->proxyHalfedge(),
                                                        ffac->proxyHalfedge()->next(),
                                                        ffac->proxyHalfedge()->next()->next()}) {
                                if (!hh || !hh->vertex()) continue;
                                const auto hv = mesh_->floatPosToInt(hh->vertex()->point());
                                if (qs[static_cast<std::size_t>(pos)] ==
                                    std::array<int, 3>{std::clamp(hv.x(), 0, mqf),
                                                       std::clamp(hv.y(), 0, mqf),
                                                       std::clamp(hv.z(), 0, mqf)}) {
                                    if (hh->isAdded()) amask2 |= (1 << pos);
                                    break;
                                }
                            }
                        }
                        std::fprintf(ff, "%d %d %d %d %d %d %d %d %d %d %d %d %d\n",
                                     ffac->groupId(), ffac->isSplittable() ? 1 : 0,
                                     ffac->isUnsplittable() ? 1 : 0, qs[0][0], qs[0][1],
                                     qs[0][2], qs[1][0], qs[1][1], qs[1][2], qs[2][0],
                                     qs[2][1], qs[2][2], amask2);
                    }
                    std::fclose(ff);
                }
            }
            if (const char* ob = std::getenv("PMSF_SC_ORDERBRIDGE_OFF"); ob && mesh_ &&
                inverseRound >= 0 && inverseRound < static_cast<int>(boundaryRoundBitCounts_.size()) &&
                boundaryRoundBitCounts_[static_cast<std::size_t>(inverseRound)] > 0)
                scOrderBridgeFromOff(mesh_, ob);
            if (std::getenv("PMSF_SC_CANON_AFTER_APPLY") && mesh_)
                canonicalizeTraversalProxies(mesh_);
        }
    }
    const bool dbg = []() {
        const char* v = std::getenv("PMSF_EDGEFLIP_DEBUG");
        return v && v[0] == '1';
    }();
    if (dbg) {
        std::fprintf(stderr, "[edgeflip-debug][opt] round=%d begin\n", s_round);
    }
    std::vector<std::pair<MCGAL::Halfedge*, EdgeFlipPostProcessOperator::CoordSymbol>>
        pendingEdgeFlipUndoTargets;
    int pendingEdgeFlipCompressRound = -1;
    const int interleavedRounds = !edgeFlipRoundBitCounts_.empty()
                                      ? static_cast<int>(edgeFlipRoundBitCounts_.size())
                                      : static_cast<int>(edgeFlipRoundCounts_.size());
    // Flip replay inside the segmented branch: artifacts carrying the coordinate
    // flip payload (written automatically by the multi-round flip path, see
    // CompressPipeline D514) can be undone deterministically, so replay is enabled
    // by default for them; PMSF_EDGEFLIP_SEG_UNDO still forces it for older payloads
    // and PMSF_EDGEFLIP_NO_UNDO disables it.  Non-coordinate payloads are untouched.
    const bool efSegUndoActive = std::getenv("PMSF_EDGEFLIP_SEG_UNDO") != nullptr ||
                                 !edgeFlipCoordSymbols_.empty() || edgeFlipCanonicalBits_;
    // 分区（layout-32）产物的每轮重建：整段（每分区 seed 刷新 → 只在本组面内
    // BFS → 瞬态标记保存/恢复 → reconstruct 顺序）在共享镜像里，与列式同源。
    // 轮数由本算子外部的主循环决定，与列式/非分区路径一致。

    if (segmentedRangeCoded_) {
        // segmented boundary consume (before interior reconstruct)
        // mode13 (round-end endpoint records) does not use the seed-BFS boundary
        // consume; running it after mid-stream boundary splits can hit corrupted
        // halfedge cycles and spin forever.
        if (!boundaryRoundBitCounts_.empty() && boundaryApplyMode_ != 13) {
            const int boundaryRounds = static_cast<int>(boundaryRoundBitCounts_.size());
            const int boundaryRound = boundaryRounds - 1 - s_round;
            if (boundaryRound >= 0 && boundaryRound + 1 < static_cast<int>(boundaryRoundBitPrefix_.size())) {
                const int startBit = boundaryRoundBitPrefix_[static_cast<size_t>(boundaryRound)];
                const int endBit = boundaryRoundBitPrefix_[static_cast<size_t>(boundaryRound + 1)];
                const auto boundaryEdges = pmsf::edgeflip::collectBoundaryEdgesSeedBfs(mesh_, seed);
                const int n = std::min(endBit - startBit, static_cast<int>(boundaryEdges.size()));
                int ones = 0;
                for (int bi = 0; bi < n; ++bi) {
                    const int bitIdx = startBit + bi;
                    if (bitIdx < 0 || bitIdx >= static_cast<int>(boundaryRoundBits_.size())) break;
                    if (boundaryRoundBits_[static_cast<size_t>(bitIdx)] != 0) ++ones;
                }
                if (std::getenv("PMSF_BOUNDARY_DEBUG")) {
                    std::cerr << "[boundary-consume-seg] round=" << boundaryRound
                              << " expected=" << (endBit - startBit)
                              << " cand=" << boundaryEdges.size()
                              << " ones=" << ones << std::endl;
                }
            }
        }
        const int boundaryRoundAfter = static_cast<int>(boundaryRoundBitCounts_.size()) - 1 - s_round;
        pmsf::pendingBoundaryTopologyRound() = boundaryRoundAfter;
        if (boundaryApplyMode_ == 42 && !pmsf::bfsGroupAnchors().empty() &&
            std::getenv("PMSF_SEG_BFS_DECODE") != nullptr) {
            pmsf::bfsRelabelFacetGroups(mesh_, pmsf::bfsGroupAnchors());
            if (std::getenv("PMSF_SEG_LABEL_HASH") && mesh_) {
                unsigned long long h = 1469598103934665603ull;
                auto mix = [&](long long v) { h ^= (unsigned long long)v; h *= 1099511628211ull; };
                for (MCGAL::Facet* f : mesh_->faces()) {
                    if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                    std::vector<std::array<int, 3>> vs;
                    MCGAL::Halfedge* h0 = f->proxyHalfedge();
                    MCGAL::Halfedge* hh = h0; int guard = 0;
                    do {
                        if (hh && hh->vertex()) {
                            auto q = mesh_->floatPosToInt(hh->vertex()->point());
                            vs.push_back({q.x(), q.y(), q.z()});
                        }
                        hh = hh ? hh->next() : nullptr;
                    } while (hh && hh != h0 && ++guard < 64);
                    std::sort(vs.begin(), vs.end());
                    for (auto& t : vs) { mix(t[0]); mix(t[1]); mix(t[2]); }
                    mix(f->groupId());
                }
                std::fprintf(stderr, "[seg-label-hash] dec round=%d hash=%llu V=%zu F=%zu\n",
                             boundaryRoundAfter, (unsigned long long)h,
                             mesh_->size_of_vertices(), mesh_->size_of_facets());
            }
        }
        if (std::getenv("PMSF_SC_CANON_FACES") && mesh_) {
            pmsf::scCanonicalFaceSort(mesh_.get());
            canonicalizeTraversalProxies(mesh_);
        }
        // Reverse ordering: boundary inverse (pre hook) -> edge-flip undo -> interior undo.
        // Flip payload in the multi-round path uses coordinate symbols: the recorded edge
        // flip replaced edge (a,b) by (c,d)= (sym.oa, sym.ob); undo by locating (c,d).
        auto doFlipUndoSeg = [&]() {
        if (efSegUndoActive && std::getenv("PMSF_EDGEFLIP_NO_UNDO") == nullptr &&
            !edgeFlipCoordSymbols_.empty() && edgeFlipDecodedRounds_ < interleavedRounds) {
            const int numFlipRounds = !edgeFlipRoundBitCounts_.empty()
                                          ? static_cast<int>(edgeFlipRoundBitCounts_.size())
                                          : static_cast<int>(edgeFlipRoundCounts_.size());
            const int compressRound = numFlipRounds - 1 - edgeFlipDecodedRounds_;
            if (compressRound >= 0 &&
                compressRound + 1 < static_cast<int>(edgeFlipPrefix_.size())) {
                const int s0 = edgeFlipPrefix_[static_cast<std::size_t>(compressRound)];
                const int s1 = std::min(
                    edgeFlipPrefix_[static_cast<std::size_t>(compressRound + 1)],
                    static_cast<int>(edgeFlipCoordSymbols_.size()));
                // q -> vertex map (vertices are not modified by flips, so this stays valid
                // for the whole round; building it once keeps the undo O(V + flips*deg)).
                const int qbits = mesh_->i_nbQuantBits > 0 ? mesh_->i_nbQuantBits : 20;
                const int maxq = (mesh_->i_nbQuantBits > 0)
                                     ? ((1 << mesh_->i_nbQuantBits) - 1)
                                     : ((1 << 20) - 1);
                auto clampArr = [&](const std::array<int, 3>& q) {
                    return std::array<int, 3>{std::clamp(q[0], 0, maxq), std::clamp(q[1], 0, maxq),
                                              std::clamp(q[2], 0, maxq)};
                };
                auto packQ = [&](const std::array<int, 3>& q) -> std::uint64_t {
                    const std::uint64_t mask = (1ull << qbits) - 1ull;
                    return ((static_cast<std::uint64_t>(q[0]) & mask) << (2 * qbits)) |
                           ((static_cast<std::uint64_t>(q[1]) & mask) << qbits) |
                           (static_cast<std::uint64_t>(q[2]) & mask);
                };
                std::unordered_map<std::uint64_t, std::vector<MCGAL::Vertex*>> vmap;
                vmap.reserve(mesh_->vertices().size() * 2);
                for (MCGAL::Vertex* v : mesh_->vertices()) {
                    if (!v || v->isRemoved()) continue;
                    const auto qi = mesh_->floatPosToInt(v->point());
                    vmap[packQ(clampArr({qi[0], qi[1], qi[2]}))].push_back(v);
                }
                int undone = 0;
                int missed = 0;
                int fallback = 0;
                auto qOfV = [&](MCGAL::Vertex* v) {
                    const MCGAL::PointInt qi = mesh_->floatPosToInt(v->point());
                    return clampArr({qi[0], qi[1], qi[2]});
                };
                auto cmatch = [&](MCGAL::Vertex* v, const std::array<int, 3>& q) {
                    if (!v || v->isRemoved()) return false;
                    const auto a = qOfV(v);
                    return a[0] == q[0] && a[1] == q[1] && a[2] == q[2];
                };
                auto roleMatchC = [&](MCGAL::Halfedge* h, const std::array<int, 3>& sa,
                                      const std::array<int, 3>& sb, const std::array<int, 3>& soa,
                                      const std::array<int, 3>& sob) {
                    if (!h || h->isRemoved() || !h->next() || !h->opposite() || !h->opposite()->next())
                        return false;
                    const auto ra = qOfV(h->vertex());
                    const auto rb = qOfV(h->end_vertex());
                    if (!((ra == sa && rb == sb) || (ra == sb && rb == sa))) return false;
                    MCGAL::Vertex* va = h->next()->end_vertex();
                    MCGAL::Vertex* vb = h->opposite()->next()->end_vertex();
                    if (!va || !vb) return false;
                    return (cmatch(va, soa) && cmatch(vb, sob)) ||
                           (cmatch(va, sob) && cmatch(vb, soa));
                };
                auto findScan = [&](const std::array<int, 3>& sa, const std::array<int, 3>& sb,
                                    const std::array<int, 3>& soa, const std::array<int, 3>& sob) {
                    for (MCGAL::Facet* f : mesh_->faces()) {
                        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                        for (MCGAL::Halfedge* hh : *f) {
                            if (!hh || hh->isRemoved()) continue;
                            if (roleMatchC(hh, sa, sb, soa, sob)) return hh;
                        }
                    }
                    return static_cast<MCGAL::Halfedge*>(nullptr);
                };
                // Per-round undirected-edge index: unordered endpoint-q pair -> halfedge.
                // Built once per round (O(E)) and updated on every flip (edge a-b becomes
                // c-d), so symbol lookup stays O(1) instead of scanning the whole mesh.
                // (The scan fallback used to cost minutes on armadillo.)
                using EdgeKey = std::pair<std::uint64_t, std::uint64_t>;
                struct EdgeKeyHash {
                    std::size_t operator()(const EdgeKey& k) const {
                        return std::hash<std::uint64_t>{}(k.first) *
                                   static_cast<std::size_t>(0x9E3779B97F4A7C15ull) ^
                               std::hash<std::uint64_t>{}(k.second);
                    }
                };
                auto edgePairKey = [&](const std::array<int, 3>& u,
                                       const std::array<int, 3>& v) -> EdgeKey {
                    std::uint64_t a = packQ(u);
                    std::uint64_t b = packQ(v);
                    if (a > b) std::swap(a, b);
                    return EdgeKey{a, b};
                };
                auto buildEdgeIndex = [&]() {
                    std::unordered_map<EdgeKey, MCGAL::Halfedge*, EdgeKeyHash> idx;
                    idx.reserve(mesh_->size_of_facets() * 4 + 16);
                    for (MCGAL::Facet* f : mesh_->faces()) {
                        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                        for (MCGAL::Halfedge* hh : *f) {
                            if (!hh || hh->isRemoved() || !hh->vertex() || !hh->end_vertex())
                                continue;
                            idx.emplace(edgePairKey(qOfV(hh->vertex()), qOfV(hh->end_vertex())), hh);
                        }
                    }
                    return idx;
                };
                std::unordered_map<EdgeKey, MCGAL::Halfedge*, EdgeKeyHash> edgeIndex =
                    buildEdgeIndex();
                const std::size_t fBefore = mesh_->size_of_facets();
                const std::size_t vBefore = mesh_->size_of_vertices();
                auto t0 = std::chrono::steady_clock::now();
                for (int i = s1 - 1; i >= s0; --i) {
                    const auto& sym = edgeFlipCoordSymbols_[static_cast<std::size_t>(i)];
                    const auto sa = clampArr(sym.a);
                    const auto sb = clampArr(sym.b);
                    const auto soa = clampArr(sym.oa);
                    const auto sob = clampArr(sym.ob);
                    MCGAL::Halfedge* h = nullptr;
                    {
                        const auto eit = edgeIndex.find(edgePairKey(sa, sb));
                        if (eit != edgeIndex.end()) {
                            MCGAL::Halfedge* cand = eit->second;
                            if (cand && !cand->isRemoved() && isTrianglePairFlippable(cand) &&
                                roleMatchC(cand, sa, sb, soa, sob)) {
                                h = cand;
                            }
                        }
                    }
                    const auto ita = vmap.find(packQ(sa));
                    const auto itb = vmap.find(packQ(sb));
                    if (ita != vmap.end() && itb != vmap.end()) {
                        // boundedRing walks live next pointers; the cached vertex halfedge
                        // list is stale after manual relinking and can even loop forever.
                        for (MCGAL::Vertex* va : ita->second) {
                            for (MCGAL::Vertex* vb : itb->second) {
                                if (!va || !vb || va == vb) continue;
                                MCGAL::Halfedge* cand = nullptr;
                                for (MCGAL::Halfedge* hh : pmsf::boundedRing(va)) {
                                    if (hh && !hh->isRemoved() && hh->end_vertex() == vb) {
                                        cand = hh;
                                        break;
                                    }
                                    if (hh && !hh->isRemoved() && hh->vertex() == vb &&
                                        hh->end_vertex() == va) {
                                        cand = hh;
                                        break;
                                    }
                                }
                                if (cand && isTrianglePairFlippable(cand) &&
                                    roleMatchC(cand, sa, sb, soa, sob)) {
                                    h = cand;
                                    break;
                                }
                            }
                            if (h) break;
                        }
                    }
                    if (!h) {
                        if (std::getenv("PMSF_EDGEFLIP_DEBUG") && fallback < 4) {
                            MCGAL::PointInt qa = mesh_->floatPosToInt(
                                ita != vmap.end() && !ita->second.empty() ? ita->second.front()->point()
                                                                          : MCGAL::Point(0, 0, 0));
                            std::fprintf(stderr,
                                         "[ef-seg-missq] round=%d symA=(%d,%d,%d) symB=(%d,%d,%d) vmapHit=%d/%d nA=%zu nB=%zu curQ=(%d,%d,%d)\n",
                                         compressRound, sa[0], sa[1], sa[2], sb[0], sb[1], sb[2],
                                         ita != vmap.end() ? 1 : 0, itb != vmap.end() ? 1 : 0,
                                         ita != vmap.end() ? ita->second.size() : 0,
                                         itb != vmap.end() ? itb->second.size() : 0, qa[0], qa[1], qa[2]);
                        }
                        h = findScan(sa, sb, soa, sob);
                        ++fallback;
                    }
                    if (h) {
                        // Normalize the flip direction: the encoder's symbol is taken
                        // after flipping h with h->vertex() == sym.a, so undoing through
                        // the opposite halfedge would reverse the restored triangles'
                        // winding (same connectivity, inconsistent orientation).
                        const auto hv = qOfV(h->vertex());
                        if (!(hv == sa) && h->opposite()) h = h->opposite();
                    }
                    if (h) {
                        // The encoder records the symbol after its edge_flip: for the
                        // post-flip halfedge h we must have h->next()->end == sym.oa and
                        // h->opposite()->next()->end == sym.ob (the old-side vertices).
                        // If those are swapped the current local winding is mirrored, and
                        // the exact inverse is obtained by flipping the opposite halfedge.
                        auto qNext = [&](MCGAL::Halfedge* hh) {
                            return qOfV(hh->next()->end_vertex());
                        };
                        if (h->next() && h->next()->end_vertex() && h->opposite() &&
                            h->opposite()->next() && h->opposite()->next()->end_vertex()) {
                            const bool direct = qNext(h) == soa &&
                                                qOfV(h->opposite()->next()->end_vertex()) == sob;
                            const bool swapped = qNext(h) == sob &&
                                                 qOfV(h->opposite()->next()->end_vertex()) == soa;
                            if (!direct && swapped && h->opposite()) h = h->opposite();
                        }
                    }
                    if (h && !h->isRemoved() && isTrianglePairFlippable(h)) {
                        const bool haveSides = h->next() && h->next()->end_vertex() &&
                                               h->opposite() && h->opposite()->next() &&
                                               h->opposite()->next()->end_vertex();
                        const auto qc = haveSides ? qOfV(h->next()->end_vertex())
                                                  : std::array<int, 3>{};
                        const auto qd = haveSides ? qOfV(h->opposite()->next()->end_vertex())
                                                  : std::array<int, 3>{};
                        const EdgeKey oldKey = edgePairKey(qOfV(h->vertex()), qOfV(h->end_vertex()));
                        mesh_->edge_flip(h);
                        edgeIndex.erase(oldKey);
                        if (haveSides) edgeIndex[edgePairKey(qc, qd)] = h;
                        ++undone;
                    } else {
                        ++missed;
                    }
                }                if (std::getenv("PMSF_EDGEFLIP_DEBUG")) {
                    std::fprintf(stderr,
                                 "[ef-seg-counts] round=%d s0=%d s1=%d V %zu->%zu F %zu->%zu\n",
                                 compressRound, s0, s1, vBefore, mesh_->size_of_vertices(), fBefore,
                                 mesh_->size_of_facets());
                    auto t1 = std::chrono::steady_clock::now();
                    std::fprintf(stderr, "[ef-seg-time] round=%d ms=%lld fallback=%d\n", compressRound,
                                 (long long)std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count(),
                                 fallback);
                }
                if (missed > 0 && std::getenv("PMSF_EDGEFLIP_DEBUG"))
                    std::fprintf(stderr, "[ef-seg-miss] round=%d missed=%d\n", compressRound, missed);
                if (std::getenv("PMSF_EDGEFLIP_DEBUG"))
                    std::fprintf(stderr,
                                 "[ef-seg-undo] round=%d decoded=%d symbols=%d undone=%d\n",
                                 compressRound, edgeFlipDecodedRounds_, s1 - s0, undone);
                ++edgeFlipDecodedRounds_;
            }
        }
        if (const char* ef = std::getenv("PMSF_EDGEFLIP_DECFLAGS_DIR");
            ef && mesh_ && edgeFlipDecodedRounds_ > 0) {
            std::filesystem::create_directories(ef);
            char fpath[512];
            const int numFlipRounds = !edgeFlipRoundBitCounts_.empty()
                                          ? static_cast<int>(edgeFlipRoundBitCounts_.size())
                                          : static_cast<int>(edgeFlipRoundCounts_.size());
            const int dumpRound = numFlipRounds - edgeFlipDecodedRounds_;
            std::snprintf(fpath, sizeof(fpath), "%s/dec_flags_%03d.txt", ef, dumpRound);
            if (FILE* ff = std::fopen(fpath, "w")) {
                const int mq = (mesh_->i_nbQuantBits > 0) ? ((1 << mesh_->i_nbQuantBits) - 1)
                                                          : (1 << 30);
                for (MCGAL::Facet* ffac : mesh_->faces()) {
                    if (!ffac || ffac->isRemoved() || !ffac->proxyHalfedge()) continue;
                    std::array<std::pair<std::array<int, 3>, MCGAL::Vertex*>, 3> items{};
                    int nItems = 0;
                    for (int k = 0; k < 3; ++k) {
                        MCGAL::Halfedge* hh = (k == 0) ? ffac->proxyHalfedge()
                                                       : (k == 1 ? ffac->proxyHalfedge()->next()
                                                                 : ffac->proxyHalfedge()->next()->next());
                        if (!hh || !hh->vertex()) continue;
                        const auto q = mesh_->floatPosToInt(hh->vertex()->point());
                        items[static_cast<std::size_t>(nItems++)] = {
                            {std::clamp(q.x(), 0, mq), std::clamp(q.y(), 0, mq),
                             std::clamp(q.z(), 0, mq)},
                            hh->vertex()};
                    }
                    if (nItems != 3) continue;
                    std::sort(items.begin(), items.end(),
                              [](const auto& a, const auto& b) { return a.first < b.first; });
                    int amask = 0;
                    for (int pos = 0; pos < 3; ++pos) {
                        for (MCGAL::Halfedge* hh : {ffac->proxyHalfedge(),
                                                    ffac->proxyHalfedge()->next(),
                                                    ffac->proxyHalfedge()->next()->next()}) {
                            if (hh && hh->vertex() == items[pos].second && hh->isAdded())
                                amask |= (1 << pos);
                        }
                    }
                    std::fprintf(ff, "%d %d %d %d %d %d %d %d %d %d %d %d %d\n",
                                 ffac->groupId(), ffac->isSplittable() ? 1 : 0,
                                 ffac->isUnsplittable() ? 1 : 0, items[0].first[0],
                                 items[0].first[1], items[0].first[2], items[1].first[0],
                                 items[1].first[1], items[1].first[2], items[2].first[0],
                                 items[2].first[1], items[2].first[2], amask);
                }
                std::fclose(ff);
            }
        }
        // V9: one bit per all-edge in canonical (endpoint-q sorted) order.  The
        // encoder locks the two triangles of every flipped edge (and their ring), so
        // the marked edges are pairwise independent and the decoder can simply
        // re-flip the marked edges of the same canonical list; no per-flip
        // coordinates are needed.
        if (std::getenv("PMSF_EDGEFLIP_NO_UNDO") == nullptr && edgeFlipCanonicalBits_ &&
            edgeFlipCoordSymbols_.empty() && !edgeFlipRoundBits_.empty() &&
            edgeFlipDecodedRounds_ < interleavedRounds) {
            const int numRounds = static_cast<int>(edgeFlipRoundBitCounts_.size());
            const int compressRound = numRounds - 1 - edgeFlipDecodedRounds_;
            if (compressRound >= 0 &&
                compressRound + 1 < static_cast<int>(edgeFlipRoundBitPrefix_.size())) {
                const int startBit = edgeFlipRoundBitPrefix_[static_cast<std::size_t>(compressRound)];
                const int endBit =
                    edgeFlipRoundBitPrefix_[static_cast<std::size_t>(compressRound + 1)];
                const std::vector<MCGAL::Halfedge*> replayEdges =
                    edgeFlipSeedBfsOrder_ && seed
                        ? pmsf::edgeflip::collectAllEdgesSeedBfs(mesh_, seed)
                        : pmsf::edgeflip::collectAllEdgesCanonical(mesh_);
                const int n = std::min(endBit - startBit, static_cast<int>(replayEdges.size()));
                int undone = 0;
                int missed = 0;
                for (int i = 0; i < n; ++i) {
                    const int bitIdx = startBit + i;
                    if (bitIdx < 0 || bitIdx >= static_cast<int>(edgeFlipRoundBits_.size())) break;
                    if (edgeFlipRoundBits_[static_cast<std::size_t>(bitIdx)] == 0) continue;
                    MCGAL::Halfedge* h = replayEdges[static_cast<std::size_t>(i)];
                    if (!h || h->isRemoved() || !isTrianglePairFlippable(h)) {
                        ++missed;
                        continue;
                    }
                    mesh_->edge_flip(h);
                    ++undone;
                }
                if (boundaryApplyMode_ == 70 && mesh_) {
                    canonicalizeDecoderProxiesDirected(mesh_);
                }
                if (std::getenv("PMSF_EDGEFLIP_DEBUG"))
                    std::fprintf(stderr,
                                 "[ef-v9-undo] round=%d bits=%d cand=%zu undone=%d missed=%d V=%zu F=%zu\\n",
                                 compressRound, endBit - startBit, replayEdges.size(), undone,
                                 missed, mesh_->size_of_vertices(), mesh_->size_of_facets());
                if (std::getenv("PMSF_BC_V2_DEBUG") && mesh_) {
                    char afp[512];
                    std::snprintf(afp, sizeof(afp), "/tmp/bc_v2_dec_afterflip_%03d.off",
                                  compressRound);
                    mesh_->dumpto_oldtype(afp);
                }
                if ((std::getenv("PMSF_BC_V2_DEBUG") || std::getenv("PMSF_BC_V2_GIDS_ALL")) &&
                    mesh_ && std::getenv("PMSF_BC_V2_GIDS_ALL")) {
                    char agp[512];
                    std::snprintf(agp, sizeof(agp), "/tmp/bc_v2_dec_afterflip_%03d.gids",
                                  compressRound);
                    if (FILE* gf = std::fopen(agp, "w")) {
                        const int mq = (mesh_->i_nbQuantBits > 0)
                                           ? ((1 << mesh_->i_nbQuantBits) - 1)
                                           : (1 << 30);
                        for (MCGAL::Facet* ff : mesh_->faces()) {
                            if (!ff || ff->isRemoved() || !ff->proxyHalfedge()) continue;
                            const auto fv = pmsf::sbdetail::faceVertices(ff);
                            if (!fv[0]) continue;
                            std::vector<std::array<int, 3>> qs;
                            for (int k = 0; k < 3; ++k) {
                                const auto q = mesh_->floatPosToInt(fv[k]->point());
                                qs.push_back({std::clamp(q.x(), 0, mq), std::clamp(q.y(), 0, mq),
                                              std::clamp(q.z(), 0, mq)});
                            }
                            std::sort(qs.begin(), qs.end());
                            std::fprintf(gf, "%d %d %d %d %d %d %d %d %d %d\n", ff->groupId(),
                                         qs[0][0], qs[0][1], qs[0][2], qs[1][0], qs[1][1],
                                         qs[1][2], qs[2][0], qs[2][1], qs[2][2]);
                        }
                        std::fclose(gf);
                    }
                }
                ++edgeFlipDecodedRounds_;
            }
        }
        };
        if (scRoundStageOrder <= 2) doFlipUndoSeg();
        // IBF = interior -> boundary -> flip: reverse order is flip undo
        // (already done above), boundary inverse here, then interior undo.
        if (scRoundStageOrder == 2) {
            const int inverseRound = static_cast<int>(boundaryRoundBitCounts_.size()) - 1 - s_round;
            if (inverseRound >= 0) applyBoundaryPatchesForRound(inverseRound);
        }
        if (std::getenv("PMSF_EDGEFLIP_DEBUG"))
            std::fprintf(stderr, "[ef-seg-preint] s_round=%d\n", s_round);
        // [experiment, env-gated] The encoder canonicalizes traversal proxies right
        // before collecting the round's interior symbols; mirror it here so the
        // proxy-dependent residual prediction (insert_point ring) matches.
        if (mesh_ && std::getenv("PMSF_SC_DEC_CANON_BEFORE_INTERIOR"))
            canonicalizeTraversalProxies(mesh_);
        segmentedDecode_.decodeOneRound();
        if (mesh_ && std::getenv("PMSF_SC_DEC_CANON_AFTER_INTERIOR"))
            canonicalizeTraversalProxies(mesh_);
        if (efSegUndoActive && mesh_) {
            pmsf::rebuildVertexHalfedgeRings(mesh_);
            pmsf::repairHalfedgePairing(mesh_);
            // The encoder calls canonicalizeTraversalProxies(mesh) right after its
            // per-round edge-flip pass; mirror it here before replaying the round's
            // interior symbols so proxy-dependent traversal state matches.
            canonicalizeTraversalProxies(mesh_);
        }
        // Orders 3/4/5 place the flip after the interior elimination.  BFI (3) and
        // FIB (4) undo the flip right after the interior; FBI (5) restores the
        // boundary first (flip is the last forward stage).
        if (scRoundStageOrder == 5) {
            const int inverseRound5 =
                static_cast<int>(boundaryRoundBitCounts_.size()) - 1 - s_round;
            if (inverseRound5 >= 0) applyBoundaryPatchesForRound(inverseRound5);
        }
        if (scRoundStageOrder == 3 || scRoundStageOrder == 4 || scRoundStageOrder == 5)
            doFlipUndoSeg();

        if (std::getenv("PMSF_EDGEFLIP_DEBUG"))
            std::fprintf(stderr, "[ef-seg-postint] s_round=%d\n", s_round);
        if (const char* asd = std::getenv("PMSF_SC_DEC_ALL_STATE"); asd && mesh_ &&
            !boundaryRoundBitCounts_.empty()) {
            const int totalR = static_cast<int>(boundaryRoundBitCounts_.size());
            const int label = totalR - 1 - s_round;
            if (FILE* ff = std::fopen(asd, s_round == 0 ? "w" : "a")) {
                std::fprintf(ff, "ROUND %d\n", label);
                for (MCGAL::Facet* ffac : mesh_->faces()) {
                    if (!ffac || ffac->isRemoved() || !ffac->proxyHalfedge()) continue;
                    const auto vs = pmsf::sbdetail::faceVertices(ffac);
                    if (!vs[0]) continue;
                    std::vector<std::array<int, 3>> qs;
                    for (int k = 0; k < 3; ++k) {
                        const auto q = mesh_->floatPosToInt(vs[k]->point());
                        qs.push_back({q.x(), q.y(), q.z()});
                    }
                    std::sort(qs.begin(), qs.end());
                    std::fprintf(ff, "%d %d %d %d %d %d %d %d %d %d %d %d\n", ffac->groupId(),
                                 ffac->isSplittable() ? 1 : 0, ffac->isUnsplittable() ? 1 : 0,
                                 qs[0][0], qs[0][1], qs[0][2], qs[1][0], qs[1][1], qs[1][2],
                                 qs[2][0], qs[2][1], qs[2][2]);
                }
                std::fclose(ff);
            }
        }
        if (const char* gd = std::getenv("PMSF_SC_DEC_ROUND_GID_DIR"); gd && mesh_) {
            const bool gidOnlyLast =
                std::getenv("PMSF_SC_DEC_ROUND_GID_ONLY_LAST") != nullptr;
            const int gidTotalRounds = boundaryRoundBitCounts_.empty()
                                           ? -1
                                           : static_cast<int>(boundaryRoundBitCounts_.size());
            if (gidOnlyLast && gidTotalRounds > 0 && s_round != gidTotalRounds - 1) {
                // skip intermediate round dumps (large meshes/30 rounds are expensive)
            } else {
            std::filesystem::create_directories(gd);
            char gpath[512];
            std::snprintf(gpath, sizeof(gpath), "%s/dec_gid_%03d.txt", gd, s_round);
            if (FILE* gf = std::fopen(gpath, "w")) {
                for (MCGAL::Facet* ff : mesh_->faces()) {
                    if (!ff || ff->isRemoved() || !ff->proxyHalfedge()) continue;
                    auto vv = pmsf::sbdetail::faceVertices(ff);
                    if (!vv[0]) continue;
                    std::vector<std::array<int, 3>> qs;
                    for (int k = 0; k < 3; ++k) {
                        const auto q = mesh_->floatPosToInt(vv[k]->point());
                        qs.push_back({q.x(), q.y(), q.z()});
                    }
                    std::sort(qs.begin(), qs.end());
                    std::fprintf(gf, "%d %d %d %d %d %d %d %d %d %d\n", ff->groupId(),
                                 qs[0][0], qs[0][1], qs[0][2], qs[1][0], qs[1][1], qs[1][2],
                                 qs[2][0], qs[2][1], qs[2][2]);
                }
                for (MCGAL::Vertex* vv : mesh_->vertices()) {
                    if (!vv || vv->isRemoved()) continue;
                    const auto q = mesh_->floatPosToInt(vv->point());
                    std::fprintf(gf, "V %d %d %d %d\n", vv->groupId(), q.x(), q.y(), q.z());
                }
                std::fclose(gf);
            }
            }
        }
        if (std::getenv("PMSF_SC_DEBUG")) {
            std::size_t lv = 0, lf = 0;
            for (MCGAL::Vertex* v : mesh_->vertices())
                if (v && !v->isRemoved()) ++lv;
            for (MCGAL::Facet* f : mesh_->faces())
                if (f && !f->isRemoved()) ++lf;
            std::fprintf(stderr, "[sc-dec-state] s_round=%d liveV=%zu liveF=%zu (vecV=%zu vecF=%zu)\n",
                         s_round, lv, lf, mesh_->size_of_vertices(), mesh_->size_of_facets());
        }
        if (std::getenv("PMSF_BOUNDARY_DEBUG"))
            std::cerr << "[btp-pending] round=" << boundaryRoundAfter
                      << " left=" << pmsf::pendingBoundaryTopologyOps().size() << std::endl;
        applyBoundaryPatchesForRound(boundaryRoundAfter);
        if (std::getenv("PMSF_ROUND_DEDUP_FACES") && mesh_) {
            const int dded = pmsf::dedupIdenticalFaces(mesh_);
            if (dded > 0 && std::getenv("PMSF_BOUNDARY_DEBUG"))
                std::fprintf(stderr, "[round-dedup->dec] s_round=%d removed=%d\n", s_round, dded);
        }
        ++s_round;
        return;
    }

    if (!std::getenv("PMSF_EDGEFLIP_NO_UNDO") && hasInterleavedEdgeFlipPayload_ &&
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
                if (dbg) {
                    std::cerr << "[ef-prefix] decoded=" << edgeFlipDecodedRounds_
                              << " numRounds=" << numRounds
                              << " prefixSize=" << edgeFlipRoundBitPrefix_.size()
                              << " compressRound=" << compressRound
                              << " startBit=" << startBit << " endBit=" << endBit
                              << " expected=" << expected << std::endl;
                }
                const bool allEdgeStream = true;
                const std::vector<MCGAL::Halfedge*> replayEdges =
                    allEdgeStream
                        ? (edgeFlipSeedBfsOrder_ && seed
                               ? pmsf::edgeflip::collectAllEdgesSeedBfs(mesh_, seed)
                               : pmsf::edgeflip::collectAllEdgesCanonical(mesh_))
                        : pmsf::edgeflip::collectEdgeFlipCandidatesBfs(mesh_);
                const int n = std::min(expected, static_cast<int>(replayEdges.size()));
                std::vector<std::pair<MCGAL::Halfedge*, EdgeFlipPostProcessOperator::CoordSymbol>>
                    undoTargets;
                int matchedFlips = 0;
                int expectedFlips = 0;
                if (!edgeFlipCoordSymbols_.empty() &&
                    compressRound + 1 < static_cast<int>(edgeFlipPrefix_.size())) {
                    const int startIdx = edgeFlipPrefix_[static_cast<size_t>(compressRound)];
                    const int endIdx = std::min(edgeFlipPrefix_[static_cast<size_t>(compressRound + 1)],
                                                static_cast<int>(edgeFlipCoordSymbols_.size()));
                    expectedFlips = std::max(0, endIdx - startIdx);
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
                            MCGAL::Halfedge* h =
                                orientCandidate(replayEdges[static_cast<size_t>(i)], sym);
                            undoTargets.emplace_back(h, sym);
                            if (h) ++matchedFlips;
                        }
                    } else {
                        undoTargets.reserve(static_cast<size_t>(n));
                        for (int k = 0; k < n; ++k) {
                            const int i = allEdgeStream ? k : (n - 1 - k);
                            const int bitIdx = startBit + i;
                            if (bitIdx < 0 || bitIdx >= static_cast<int>(edgeFlipRoundBits_.size())) break;
                            if (edgeFlipRoundBits_[static_cast<size_t>(bitIdx)] == 0) continue;
                            MCGAL::Halfedge* ce = replayEdges[static_cast<size_t>(i)];
                            EdgeFlipPostProcessOperator::CoordSymbol minsym;
                            auto keyOf = [&](MCGAL::Vertex* v) {
                                auto k = pmsf::edgeflip::bfsVertexKey(mesh_, v);
                                return std::array<int,3>{static_cast<int>(k[0]), static_cast<int>(k[1]), static_cast<int>(k[2])};
                            };
                            if (ce && ce->vertex() && ce->end_vertex() && ce->next() &&
                                ce->opposite() && ce->opposite()->next()) {
                                minsym.a = keyOf(ce->vertex());
                                minsym.b = keyOf(ce->end_vertex());
                                minsym.oa = keyOf(ce->next()->end_vertex());
                                minsym.ob = keyOf(ce->opposite()->next()->end_vertex());
                            }
                            undoTargets.emplace_back(ce, minsym);
                            ++matchedFlips;
                        }
                        expectedFlips = matchedFlips;
                    }
                    pendingEdgeFlipUndoTargets = std::move(undoTargets);
                    if (dbg) {
                        std::fprintf(stderr,
                                     "[edgeflip-debug][opt] round=%d bits=%d cand=%zu pendingFlip=%d/%d\n",
                                     s_round, expected, replayEdges.size(), matchedFlips, expectedFlips);
                    }
                }
                ++edgeFlipDecodedRounds_;
                if (dbg) {
                    std::fprintf(stderr,
                                 "[edgeflip-debug][opt] round=%d undo_done decoded=%d\n",
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
                int matchedFlips = 0;
    
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
                        MCGAL::Halfedge* h = findHalfedgeByCoordSymbol(mesh_, s);
                        if (h) ++matchedFlips;
                        undoOneFlip(mesh_, h);
                    }
                } else {
                    assignBfsLocalVertexIds(mesh_, seed);
                    const std::vector<MCGAL::Vertex*> idToVertex = buildLocalIdMap(mesh_);
                    const int symbolEnd = std::min(endIdx, static_cast<int>(edgeFlipSymbols_.size()));
                    for (int i = symbolEnd - 1; i >= startIdx; --i) {
                        const auto& s = edgeFlipSymbols_[static_cast<size_t>(i)];
                        if (dbg && i == symbolEnd - 1) {
                            std::fprintf(stderr,
                                         "[edgeflip-debug][opt] round=%d first_sym=(%d,%d) id_cap=%zu\n",
                                         s_round, s.first, s.second, idToVertex.size());
                        }
                        MCGAL::Halfedge* h = findHalfedgeByPoolIds(mesh_, s.first, s.second);
                        if (!h) {
                            h = findHalfedgeByLocalIds(mesh_, idToVertex, s.first, s.second);
                        }
                        if (!h) h = findHalfedgeByVertexIds(mesh_, s.first, s.second);
                        if (h) ++matchedFlips;
                        undoOneFlip(mesh_, h);
                    }
                }
                if (edgeFlipDumpEnabled()) {
                    mesh_->dumpto_oldtype(afterPath);
                }
                if (dbg) {
                    std::fprintf(stderr,
                                 "[edgeflip-debug][opt] round=%d symbols=%d matched=%d\n",
                                 s_round, endIdx - startIdx, matchedFlips);
                }
            }
            ++edgeFlipDecodedRounds_;
            if (dbg) {
                std::fprintf(stderr,
                             "[edgeflip-debug][opt] round=%d undo_done decoded=%d\n",
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
        if (std::getenv("PMSF_EDGEFLIP_DEBUG")) {
            std::cerr << "[ef-round] dec before_undo F=" << mesh_->size_of_facets()
                      << " V=" << mesh_->size_of_vertices()
                      << " pending=" << pendingEdgeFlipUndoTargets.size() << std::endl;
        }
        for (const auto& item : pendingEdgeFlipUndoTargets) {
            if (!item.first) { std::cerr << "[ef-null] pending undo target is null" << std::endl; }
            if (true) {
                if (item.first && !item.first->isRemoved()) {
                    mesh_->edge_flip(item.first);
                }
            } else {
                undoOneFlip(mesh_, item.first, item.second);
            }
        }
        canonicalizeDecoderProxies(mesh_);
        if (std::getenv("PMSF_EDGEFLIP_REBUILD_AFTER_UNDO") && pendingEdgeFlipCompressRound >= 0) {
            char tmpPath[] = "/tmp/pmsf_edgeflip_dec_rebuild_XXXXXX.off";
            const int fd = mkstemps(tmpPath, 4);
            if (fd >= 0) {
                close(fd);
                MCGAL::Vertex* sv0 = seed ? seed->vertex() : nullptr;
                MCGAL::Vertex* sv1 = seed ? seed->end_vertex() : nullptr;
                mesh_->dumpto_oldtype(tmpPath);
                auto rebuilt = std::make_shared<MCGAL::Mesh>(tmpPath);
                std::remove(tmpPath);
                if (rebuilt) {
                    rebuilt->bboxMin = mesh_->bboxMin; rebuilt->bboxMax = mesh_->bboxMax;
                    rebuilt->f_quantStep = mesh_->f_quantStep; rebuilt->i_nbQuantBits = mesh_->i_nbQuantBits;
                    rebuilt->enableQuantization = mesh_->enableQuantization;
                    if (sv0 && sv1) {
                        auto samePoint = [](const MCGAL::Point& a, const MCGAL::Point& b) {
                            const double dx = static_cast<double>(a.x()) - b.x();
                            const double dy = static_cast<double>(a.y()) - b.y();
                            const double dz = static_cast<double>(a.z()) - b.z();
                            return dx*dx + dy*dy + dz*dz < 1e-10;
                        };
                        MCGAL::Vertex* nv0 = nullptr; MCGAL::Vertex* nv1 = nullptr;
                        for (MCGAL::Vertex* v : rebuilt->vertices()) {
                            if (!v || v->isRemoved()) continue;
                            if (!nv0 && samePoint(v->point(), sv0->point())) nv0 = v;
                            if (!nv1 && samePoint(v->point(), sv1->point())) nv1 = v;
                        }
                        if (nv0 && nv1) { if (MCGAL::Halfedge* h = bndSafeFindHalfedge(nv0, nv1)) seed = h; }
                    }
                    mesh_ = rebuilt;
                }
            }
        }
        if (std::getenv("PMSF_EDGEFLIP_DEBUG")) {
            std::cerr << "[ef-round] dec after_undo F=" << mesh_->size_of_facets()
                      << " V=" << mesh_->size_of_vertices() << std::endl;
        }
        if (edgeFlipDumpEnabled()) {
            mesh_->dumpto_oldtype(afterPath);
        }
    }

    if (!boundaryRoundBitCounts_.empty()) {
        const int boundaryRounds = static_cast<int>(boundaryRoundBitCounts_.size());
        const int boundaryRound = boundaryRounds - 1 - s_round;
        if (boundaryRound >= 0 && boundaryRound + 1 < static_cast<int>(boundaryRoundBitPrefix_.size())) {
            const int startBit = boundaryRoundBitPrefix_[static_cast<size_t>(boundaryRound)];
            const int endBit = boundaryRoundBitPrefix_[static_cast<size_t>(boundaryRound + 1)];
            const auto boundaryEdges =
                pmsf::edgeflip::collectBoundaryEdgesSeedBfs(mesh_, seed);
            const int n = std::min(endBit - startBit, static_cast<int>(boundaryEdges.size()));
            int consumedFlips = 0;
            for (int i = 0; i < n; ++i) {
                const int bitIdx = startBit + i;
                if (bitIdx < 0 || bitIdx >= static_cast<int>(boundaryRoundBits_.size())) break;
                if (boundaryRoundBits_[static_cast<size_t>(bitIdx)] != 0) {
                    // Boundary simplification inverse is wired in the next phase.
                    ++consumedFlips;
                }
            }
            if (std::getenv("PMSF_BOUNDARY_DEBUG")) {
                std::cerr << "[boundary-consume] round=" << boundaryRound
                          << " expected=" << (endBit - startBit)
                          << " cand=" << boundaryEdges.size()
                          << " ones=" << consumedFlips << std::endl;
            }
        }
    }

    if (!boundaryRoundBitCounts_.empty() &&
        ((boundaryApplyMode_ == 1 && s_round == 0) || boundaryApplyMode_ == 5 ||
         ((boundaryApplyMode_ == 6 || boundaryApplyMode_ == 7) && s_round == 0))) {
        applyBoundaryPatchesForRound(static_cast<int>(boundaryRoundBitCounts_.size()) - 1 - s_round);
    }

    vertices.clear();
    halfedges.clear();
    facets.clear();
    if (header_.isEnableQuantization()) {
        symbolReadOperator_->enableQuantization(mesh_->bboxMin, mesh_->f_quantStep);
    }
    if (header_.isEnablePrediction()) {
        // range-coder 模式：符号与残差已全部解码到 symbolReadOperator 内部队列，
        // 这里直接按 BFS 从 seed 出发消费队列重建本轮。
        if (std::getenv("PMSF_SC_CANON_FACES") && mesh_)
            pmsf::scCanonicalFaceSort(mesh_.get());
        symbolReadOperator_->collectFromDecoded(seed, vertices, halfedges, facets);    } else {
        symbolReadOperator_->collect(seed, buffer_, dataOffset_, vertices, halfedges, facets);    }

    if (std::getenv("PMSF_EDGEFLIP_DEBUG")) { std::cerr << "[ef-round] dec after_reconstruct F=" << mesh_->size_of_facets() << " V=" << mesh_->size_of_vertices() << std::endl; }
    reconstructOperator_->reconstruct(vertices, halfedges, facets);
    if (!boundaryRoundBitCounts_.empty()) {
        applyBoundaryPatchesForRound(static_cast<int>(boundaryRoundBitCounts_.size()) - 1 - s_round);
    }
    if (pendingEdgeFlipCompressRound >= 0 && edgeFlipDumpEnabled()) {
        std::filesystem::create_directories("build/aaaaresult/decompress/rounds");
        char roundPath[512];
        std::snprintf(roundPath, sizeof(roundPath),
                      "build/aaaaresult/decompress/rounds/round_%03d_after_reconstruct.off",
                      pendingEdgeFlipCompressRound);
        mesh_->dumpto_oldtype(roundPath);
    }
    if (dbg) {
        std::fprintf(stderr, "[edgeflip-debug][opt] round=%d reconstruct_done\n", s_round);
    }
    ++s_round;
}

static void canonicalizeDecoderProxies(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!mesh) return;
    auto key = [&](MCGAL::Halfedge* h) {
        if (!h || !h->vertex() || !h->end_vertex()) return std::array<long long, 6>{0,0,0,0,0,0};
        auto vk = [&](MCGAL::Vertex* v) -> std::array<long long, 3> {
            if (!v) return {LLONG_MIN, LLONG_MIN, LLONG_MIN};
            if (mesh && mesh->enableQuantization) {
                MCGAL::PointInt q = mesh->floatPosToInt(v->point());
                return {q[0], q[1], q[2]};
            }
            constexpr double kScale = 100000000.0;
            return {static_cast<long long>(std::llround(static_cast<double>(v->x()) * kScale)),
                    static_cast<long long>(std::llround(static_cast<double>(v->y()) * kScale)),
                    static_cast<long long>(std::llround(static_cast<double>(v->z()) * kScale))};
        };
        auto a = vk(h->vertex());
        auto b = vk(h->end_vertex());
        if (b < a) std::swap(a, b);
        return std::array<long long, 6>{a[0],a[1],a[2],b[0],b[1],b[2]};
    };
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        MCGAL::Halfedge* best = nullptr;
        for (MCGAL::Halfedge* h : *f) {
            if (!h || h->isRemoved()) continue;
            if (!best || key(best) < key(h)) best = h;
        }
        if (best) f->setProxyHalfedge(best);
    }
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        MCGAL::Halfedge* best = nullptr;
        for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
            if (!h || h->isRemoved() || h->vertex() != v) continue;
            if (!best || key(best) < key(h)) best = h;
        }
        if (best) v->setProxyHalfedge(best);
    }
}

static void canonicalizeDecoderProxiesDirected(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!mesh) return;
    auto key = [&](MCGAL::Halfedge* h) {
        auto vk = [&](MCGAL::Vertex* v) -> std::array<long long, 3> {
            if (!v) return {LLONG_MIN, LLONG_MIN, LLONG_MIN};
            if (mesh->enableQuantization) {
                MCGAL::PointInt q = mesh->floatPosToInt(v->point());
                return {q[0], q[1], q[2]};
            }
            constexpr double kScale = 100000000.0;
            return {static_cast<long long>(std::llround(static_cast<double>(v->x()) * kScale)),
                    static_cast<long long>(std::llround(static_cast<double>(v->y()) * kScale)),
                    static_cast<long long>(std::llround(static_cast<double>(v->z()) * kScale))};
        };
        const auto a = vk(h ? h->vertex() : nullptr);
        const auto b = vk(h ? h->end_vertex() : nullptr);
        return std::array<long long, 6>{a[0], a[1], a[2], b[0], b[1], b[2]};
    };
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        MCGAL::Halfedge* best = nullptr;
        for (MCGAL::Halfedge* h : *f) {
            if (!h || h->isRemoved() || !h->vertex() || h->vertex()->isRemoved()) continue;
            if (!best || key(best) < key(h)) best = h;
        }
        if (best) f->setProxyHalfedge(best);
    }
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        MCGAL::Halfedge* best = nullptr;
        for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
            if (!h || h->isRemoved() || h->vertex() != v || !h->end_vertex() ||
                h->end_vertex()->isRemoved())
                continue;
            if (!best || key(best) < key(h)) best = h;
        }
        if (best) v->setProxyHalfedge(best);
    }
}

std::shared_ptr<MCGAL::Mesh> OptimizedBasicDeserializeOperator::exportMesh() {
    return mesh_;
}
