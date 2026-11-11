#include "boundary/BoundaryCompactStorage.h"

#include "common/GroupBoundaryEnumeration.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace pmsf {
namespace {

int bitsForValue(int n) {
    int b = 1;
    while ((1 << b) < n && b < 31) ++b;
    return b;
}

int boundaryHashShift() {
    const char* v = std::getenv("PMSF_BOUNDARY_HASH_SHIFT");
    if (!v || !v[0]) return 2;
    const int s = std::atoi(v);
    return (s < 0) ? 0 : (s > 8 ? 8 : s);
}

int varintBits(std::uint32_t v) {
    int bytes = 1;
    while (v >= 0x80u) { v >>= 7; ++bytes; }
    return bytes * 8;
}

std::vector<std::array<int, 6>> deriveSegmentEdgeKeys(MCGAL::Mesh* mesh,
                                                      MCGAL::Vertex* anchor,
                                                      int gidA,
                                                      int gidB);

}  // namespace

BoundaryStorageStats measureBoundaryStorage(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    BoundaryStorageStats st;
    if (!mesh) return st;
    st.quantBits = mesh->i_nbQuantBits;
    int maxGroup = -1;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (f && !f->isRemoved()) maxGroup = std::max(maxGroup, f->groupId());
    }
    st.groupCount = maxGroup + 1;

    const auto segments = collectGroupBoundarySegments(mesh.get());
    st.segmentCount = static_cast<int>(segments.size());

    std::set<std::pair<int, int>> uniqueEdges;
    std::set<int> uniqueVerts;
    int totalEdgeCount = 0;
    for (const auto& seg : segments) {
        totalEdgeCount += static_cast<int>(seg.edges.size());
        for (MCGAL::Halfedge* h : seg.edges) {
            if (!h || !h->vertex() || !h->end_vertex()) continue;
            int a = h->vertex()->poolId();
            int b = h->end_vertex()->poolId();
            if (a > b) std::swap(a, b);
            uniqueEdges.insert({a, b});
            uniqueVerts.insert(a);
            uniqueVerts.insert(b);
        }
    }
    st.boundaryEdgeCount = static_cast<int>(uniqueEdges.size());
    st.boundaryVertexCount = static_cast<int>(uniqueVerts.size());

    const int vertexIndexBits = bitsForValue(std::max(1, st.boundaryVertexCount));
    const int gidBits = bitsForValue(std::max(1, st.groupCount));

    // Naive: every boundary vertex position + group id, and every boundary
    // edge as two vertex indices, plus a small header.
    st.naiveBits = 32 + 32;
    st.naiveBits += static_cast<std::size_t>(st.boundaryVertexCount) *
                    (3u * st.quantBits + static_cast<unsigned>(gidBits));
    st.naiveBits += static_cast<std::size_t>(st.boundaryEdgeCount) * 2u *
                    static_cast<unsigned>(vertexIndexBits);

    // Compact: one anchor position + one direction bit + one varint length
    // per segment.  Boundary connectivity/geometry is derived by walking the
    // group-boundary graph from the anchor.
    st.compactBits = 32;
    for (const auto& seg : segments) {
        st.anchorBits += 3u * st.quantBits;
        st.bitFlags += 1;
        st.compactBits += 3u * st.quantBits + 1u +
                          static_cast<std::size_t>(varintBits(static_cast<std::uint32_t>(seg.edges.size())));
    }
    return st;
}

BoundaryBitPayload buildCompactBoundaryPayload(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                               int rounds) {
    BoundaryBitPayload p;
    if (!mesh) return p;
    const auto segments = collectGroupBoundarySegments(mesh.get());
    p.roundBitCounts.assign(static_cast<std::size_t>(std::max(1, rounds)), 0);
    if (!segments.empty()) p.roundBitCounts[0] = static_cast<int>(segments.size());
    p.residuals.push_back(static_cast<int>(segments.size()));
    int prevX = 0, prevY = 0, prevZ = 0;
    int segIndex = 0;
    for (const auto& seg : segments) {
        MCGAL::PointInt q(0, 0, 0);
        if (seg.anchor) q = mesh->floatPosToInt(seg.anchor->point());
        const int gidA = seg.gidA;
        const int gidB = seg.gidB;
        int dir = 0;
        if (seg.anchor) {
            for (MCGAL::Halfedge* h : seg.edges) {
                if (h && h->vertex() == seg.anchor) { dir = 0; break; }
                if (h && h->end_vertex() == seg.anchor) { dir = 1; break; }
            }
        }
        const int qx = q.x(), qy = q.y(), qz = q.z();
        std::vector<std::array<int, 6>> edgeKeys;
        for (MCGAL::Halfedge* h : seg.edges) {
            if (!h || !h->vertex() || !h->end_vertex()) continue;
            auto a = mesh->floatPosToInt(h->vertex()->point());
            auto b = mesh->floatPosToInt(h->end_vertex()->point());
            std::array<int, 3> pa{a.x(), a.y(), a.z()};
            std::array<int, 3> pb{b.x(), b.y(), b.z()};
            if (pb < pa) std::swap(pa, pb);
            edgeKeys.push_back({pa[0], pa[1], pa[2], pb[0], pb[1], pb[2]});
        }
        std::sort(edgeKeys.begin(), edgeKeys.end());
        edgeKeys.erase(std::unique(edgeKeys.begin(), edgeKeys.end()), edgeKeys.end());
        auto derivedKeys = deriveSegmentEdgeKeys(mesh.get(), seg.anchor, gidA, gidB);
        std::sort(derivedKeys.begin(), derivedKeys.end());
        derivedKeys.erase(std::unique(derivedKeys.begin(), derivedKeys.end()), derivedKeys.end());
        const bool fallback = derivedKeys != edgeKeys;
        unsigned long long hash = 1469598103934665603ull;
        auto mix = [&](long long v) {
            hash ^= static_cast<unsigned long long>(v);
            hash *= 1099511628211ull;
        };
        // Coarse checksum: Q12 base-mesh coordinates can differ by +-1 between
        // encoder and decoder, while the boundary topology is identical.  Use
        // q>>2 for the hash so the payload validates across that jitter.
        const int hs = boundaryHashShift();
        auto cq = [hs](int v) -> int {
            return hs > 0 ? (v + (1 << (hs - 1))) >> hs : v;
        };
        mix(cq(qx)); mix(cq(qy)); mix(cq(qz));
        std::vector<std::array<int, 6>> coarseKeys;
        coarseKeys.reserve(edgeKeys.size());
        for (const auto& k : edgeKeys)
            coarseKeys.push_back({cq(k[0]), cq(k[1]), cq(k[2]),
                                  cq(k[3]), cq(k[4]), cq(k[5])});
        std::sort(coarseKeys.begin(), coarseKeys.end());
        coarseKeys.erase(std::unique(coarseKeys.begin(), coarseKeys.end()),
                         coarseKeys.end());
        for (const auto& k : coarseKeys)
            for (int v : k) mix(v);
        ++segIndex;
        p.bits.push_back(static_cast<unsigned char>(dir & 1));
        p.residuals.push_back(gidA);
        p.residuals.push_back(gidB);
        p.residuals.push_back(qx - prevX);
        p.residuals.push_back(qy - prevY);
        p.residuals.push_back(qz - prevZ);
        const int anchorDeg = seg.anchor
                                  ? (seg.anchor->is_degree_dirty()
                                         ? seg.anchor->vertex_degree()
                                         : seg.anchor->vertex_degree_cached())
                                  : -1;
        p.residuals.push_back(anchorDeg);
        p.residuals.push_back(static_cast<int>(edgeKeys.size()));
        p.residuals.push_back(static_cast<int>(hash & 0x7fffffff));
        p.residuals.push_back(fallback ? 1 : 0);
        if (fallback) {
            p.residuals.push_back(static_cast<int>(edgeKeys.size()));
            for (const auto& k : edgeKeys)
                for (int v : k) p.residuals.push_back(v);
        }
        prevX = qx; prevY = qy; prevZ = qz;
    }
    return p;
}

BoundaryBitPayload buildNaiveBoundaryPayload(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                             int rounds) {
    BoundaryBitPayload p;
    if (!mesh) return p;
    const auto segments = collectGroupBoundarySegments(mesh.get());
    p.roundBitCounts.assign(static_cast<std::size_t>(std::max(1, rounds)), 0);
    std::vector<MCGAL::Vertex*> verts;
    std::unordered_map<int, int> vertIndex;
    std::vector<std::pair<int, int>> edges;
    std::set<std::pair<int, int>> seenEdge;
    for (const auto& seg : segments) {
        for (MCGAL::Halfedge* h : seg.edges) {
            if (!h || !h->vertex() || !h->end_vertex()) continue;
            MCGAL::Vertex* a = h->vertex();
            MCGAL::Vertex* b = h->end_vertex();
            auto addVert = [&](MCGAL::Vertex* v) {
                if (!v) return;
                if (vertIndex.find(v->poolId()) == vertIndex.end()) {
                    vertIndex[v->poolId()] = static_cast<int>(verts.size());
                    verts.push_back(v);
                }
            };
            addVert(a); addVert(b);
            int ia = vertIndex[a->poolId()];
            int ib = vertIndex[b->poolId()];
            if (ia > ib) std::swap(ia, ib);
            if (seenEdge.insert({ia, ib}).second) edges.push_back({ia, ib});
        }
    }
    p.residuals.push_back(static_cast<int>(segments.size()));
    p.residuals.push_back(static_cast<int>(verts.size()));
    for (MCGAL::Vertex* v : verts) {
        auto q = mesh->floatPosToInt(v->point());
        int gid = 0;
        for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
            if (h && h->face() && !h->face()->isRemoved()) { gid = h->face()->groupId(); break; }
        }
        p.residuals.push_back(q.x());
        p.residuals.push_back(q.y());
        p.residuals.push_back(q.z());
        p.residuals.push_back(gid);
    }
    p.residuals.push_back(static_cast<int>(edges.size()));
    for (auto& e : edges) {
        p.residuals.push_back(e.first);
        p.residuals.push_back(e.second);
    }
    return p;
}

namespace {
std::vector<MCGAL::Halfedge*> orientedBoundaryAt(MCGAL::Mesh* mesh, MCGAL::Vertex* v,
                                                 int gidLeft, int gidRight) {
    std::vector<MCGAL::Halfedge*> out;
    if (!mesh || !v) return out;
    for (MCGAL::Halfedge* h : v->halfedges()) {
        if (!h || h->isRemoved() || !h->vertex() || h->vertex() != v) continue;
        MCGAL::Halfedge* o = h->opposite();
        if (!o || !h->face() || !o->face() || h->face()->isRemoved() || o->face()->isRemoved()) continue;
        if (h->face()->groupId() == gidLeft && o->face()->groupId() == gidRight)
            out.push_back(h);
    }
    return out;
}

void collectWalkKeys(const std::shared_ptr<MCGAL::Mesh>& mesh,
                     MCGAL::Vertex* anchor,
                     MCGAL::Halfedge* start,
                     std::vector<std::array<int, 6>>& keys) {
    if (!mesh || !anchor || !start) return;
    MCGAL::Halfedge* h = start;
    for (int guard = 0; guard < 100000 && h; ++guard) {
        if (!h->vertex() || !h->end_vertex()) break;
        auto a = mesh->floatPosToInt(h->vertex()->point());
        auto b = mesh->floatPosToInt(h->end_vertex()->point());
        std::array<int, 3> pa{a.x(), a.y(), a.z()};
        std::array<int, 3> pb{b.x(), b.y(), b.z()};
        if (pb < pa) std::swap(pa, pb);
        keys.push_back({pa[0], pa[1], pa[2], pb[0], pb[1], pb[2]});
        MCGAL::Vertex* nextv = h->end_vertex();
        if (nextv == anchor) break;
        const int left = h->face() ? h->face()->groupId() : -1;
        const int right = (h->opposite() && h->opposite()->face()) ? h->opposite()->face()->groupId() : -1;
        MCGAL::Halfedge* next = nullptr;
        int incident = 0;
        for (MCGAL::Halfedge* cand : orientedBoundaryAt(mesh.get(), nextv, left, right)) {
            ++incident;
            if (cand == h || (h->opposite() && cand == h->opposite())) continue;
            next = cand;
        }
        // For one fixed side orientation there is exactly one outgoing boundary
        // halfedge at a degree-2 boundary vertex.  Zero means an endpoint,
        // more than one means a junction.
        if (incident != 1 || !next) break;
        h = next;
    }
}

std::vector<std::array<int, 6>> deriveSegmentEdgeKeys(MCGAL::Mesh* mesh,
                                                      MCGAL::Vertex* anchor,
                                                      int gidA,
                                                      int gidB) {
    std::vector<std::array<int, 6>> keys;
    if (!mesh || !anchor) return keys;
    std::shared_ptr<MCGAL::Mesh> nonOwning(mesh, [](MCGAL::Mesh*) {});
    for (int swap = 0; swap < 2; ++swap) {
        const int left = swap ? gidB : gidA;
        const int right = swap ? gidA : gidB;
        for (MCGAL::Halfedge* start : orientedBoundaryAt(mesh, anchor, left, right)) {
            if (!start) continue;
            collectWalkKeys(nonOwning, anchor, start, keys);
        }
    }
    std::sort(keys.begin(), keys.end());
    keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
    return keys;
}

}  // namespace

static std::vector<unsigned char> g_verifyBits;
static std::vector<int> g_verifyResiduals;

void setCompactBoundaryPayloadForVerify(std::vector<unsigned char> bits,
                                        std::vector<int> residuals) {
    g_verifyBits = std::move(bits);
    g_verifyResiduals = std::move(residuals);
}
bool hasCompactBoundaryPayloadForVerify() { return !g_verifyResiduals.empty(); }

BoundaryVerifyStats verifyCompactBoundaryPayload(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    BoundaryVerifyStats st;
    if (!mesh || g_verifyResiduals.empty()) return st;
    const auto& res = g_verifyResiduals;
    std::size_t ri = 0;
    const int segmentCount = res[ri++];
    st.segments = segmentCount;
    int prevX = 0, prevY = 0, prevZ = 0;
    for (int si = 0; si < segmentCount; ++si) {
        if (ri + 7 > res.size()) { ++st.failed; continue; }
        const int gidA = res[ri++];
        const int gidB = res[ri++];
        const int dx = res[ri++];
        const int dy = res[ri++];
        const int dz = res[ri++];
        const int anchorDeg = res[ri++];
        const int expectedEdges = res[ri++];
        const int expectedHash = res[ri++];
        const int fallback = (ri < res.size()) ? res[ri++] : 0;
        std::vector<std::array<int, 6>> explicitKeys;
        if (fallback && ri < res.size()) {
            const int n = res[ri++];
            for (int i = 0; i < n && ri + 6 <= res.size(); ++i) {
                explicitKeys.push_back({res[ri], res[ri + 1], res[ri + 2],
                                        res[ri + 3], res[ri + 4], res[ri + 5]});
                ri += 6;
            }
        }
        MCGAL::PointInt q(prevX + dx, prevY + dy, prevZ + dz);
        prevX = q.x(); prevY = q.y(); prevZ = q.z();
        MCGAL::Vertex* anchor = nullptr;
        // Exact match first, then allow the +-1/+-2 quantization jitter that the
        // Q12 base-mesh round trip can introduce.  Anchor degree disambiguates.
        for (int pass = 0; pass < 3 && !anchor; ++pass) {
            const int tol = pass;
            for (MCGAL::Vertex* v : mesh->vertices()) {
                if (!v || v->isRemoved()) continue;
                auto vq = mesh->floatPosToInt(v->point());
                if (std::abs(vq.x() - q.x()) > tol ||
                    std::abs(vq.y() - q.y()) > tol ||
                    std::abs(vq.z() - q.z()) > tol)
                    continue;
                if (anchorDeg >= 0) {
                    const int d = v->is_degree_dirty() ? v->vertex_degree()
                                                       : v->vertex_degree_cached();
                    if (d != anchorDeg) continue;
                }
                anchor = v;
                break;
            }
        }
        if (!anchor) { ++st.anchorMissing; ++st.failed; continue; }
        std::vector<std::array<int, 6>> allKeys;
        if (fallback) {
            allKeys = explicitKeys;
            ++st.fallbackSegments;
        } else {
            for (int swap = 0; swap < 2; ++swap) {
                const int left = swap ? gidB : gidA;
                const int right = swap ? gidA : gidB;
                for (MCGAL::Halfedge* start : orientedBoundaryAt(mesh.get(), anchor, left, right)) {
                    if (!start) continue;
                    collectWalkKeys(mesh, anchor, start, allKeys);
                }
            }
        }
        std::sort(allKeys.begin(), allKeys.end());
        allKeys.erase(std::unique(allKeys.begin(), allKeys.end()), allKeys.end());
        unsigned long long hash = 1469598103934665603ull;
        auto mix = [&](long long v) {
            hash ^= static_cast<unsigned long long>(v);
            hash *= 1099511628211ull;
        };
        const int hs = boundaryHashShift();
        auto cq = [hs](int v) -> int {
            return hs > 0 ? (v + (1 << (hs - 1))) >> hs : v;
        };
        mix(cq(q.x())); mix(cq(q.y())); mix(cq(q.z()));
        std::vector<std::array<int, 6>> coarseKeys;
        coarseKeys.reserve(allKeys.size());
        for (const auto& k : allKeys)
            coarseKeys.push_back({cq(k[0]), cq(k[1]), cq(k[2]),
                                  cq(k[3]), cq(k[4]), cq(k[5])});
        std::sort(coarseKeys.begin(), coarseKeys.end());
        coarseKeys.erase(std::unique(coarseKeys.begin(), coarseKeys.end()),
                         coarseKeys.end());
        for (const auto& k : coarseKeys)
            for (int v : k) mix(v);
        const bool matched = static_cast<int>(allKeys.size()) == expectedEdges &&
                             static_cast<int>(hash & 0x7fffffff) == expectedHash;
        if (matched) ++st.verified;
        else {
            ++st.walkMismatch;
            ++st.failed;
        }
    }
    return st;
}

}  // namespace pmsf
