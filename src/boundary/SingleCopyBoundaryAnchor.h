#pragma once

/// Per-group boundary traversal + anchor-addressed boundary patches (D203).
/// Unlike group-pair loops, per-group boundary traversal is well-defined at
/// triple junctions (a vertex where 3+ groups meet), because the next boundary
/// edge is found by rotating inside the group own face fan.

#include "SingleCopyBoundaryCompact.h"
#include "boundary/BoundaryIdRegistry.h"
#include "common/BoundedRingTraversal.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace pmsf {

// Group-boundary edge count (undirected): edges whose two sides belong to different
// groups, plus open (one-sided) edges.  Used as a per-round global topology invariant.
inline int countGroupBoundaryEdges(MCGAL::Mesh* mesh) {
    if (!mesh) return 0;
    long long groupSides = 0;
    long long openEdges = 0;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && !h->isRemoved()) {
                MCGAL::Halfedge* opp = h->opposite();
                if (!opp || opp->isRemoved() || !opp->face() || opp->face()->isRemoved()) {
                    ++openEdges;
                } else if (opp->face()->groupId() != f->groupId()) {
                    ++groupSides;
                }
            }
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
    }
    return static_cast<int>(openEdges + groupSides / 2);
}


// Forward declarations for order-key helpers (definitions below, used by both
// anchor and group stages).
inline std::vector<int> scTopoVertexKey(MCGAL::Mesh* mesh, MCGAL::Vertex* v);
inline std::vector<long long> scVertexOrderKey(MCGAL::Mesh* mesh, MCGAL::Vertex* v);
// Forward declarations for the per-op verification helpers (defined below).
inline std::uint64_t scTrialClampedHash(MCGAL::Mesh* mesh);
inline std::shared_ptr<MCGAL::Mesh> scBuildDecoderBaseFromMesh(MCGAL::Mesh* mesh);
inline bool applyGroupPatchInverseDirect(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                         const std::vector<int>& data,
                                         std::size_t& cursor);

/// Per-op pre-state hashes recorded by the encoder stage (diagnostic: lets the
/// post-stage verification locate the first inverse that does not reproduce the
/// encoder state).  The stage and the verification run in the same process.
inline std::vector<std::uint64_t>& scStageOpPreHashes() {
    static thread_local std::vector<std::uint64_t> hashes;
    return hashes;
}

/// Vertices touched by mid-stream boundary stages.  The pipeline unions them into
/// the shared frozen set so later rounds cannot eliminate them (an elimination of a
/// vertex whose fan mixes stage-created faces would make the decoder inherit wrong
/// group labels).
inline std::set<MCGAL::Vertex*>& scStageFrozenVertices() {
    static thread_local std::set<MCGAL::Vertex*> verts;
    return verts;
}


struct ScAnchorDef {
    MCGAL::Vertex* v = nullptr;
    int group = -1;
};

inline std::array<int, 3> scAnchorQ(MCGAL::Mesh* mesh, MCGAL::Vertex* v) {
    const MCGAL::PointInt q = mesh->floatPosToInt(v->point());
    // Clamp to the quantization range so encoder-side unclamped q (e.g. 2^bits
    // at the bbox max due to float truncation) sorts identically to decoder-side
    // clamped q; loop rotation/ordering must match exactly.
    if (mesh->i_nbQuantBits > 0) {
        const int maxQ = (1 << mesh->i_nbQuantBits) - 1;
        return {std::clamp(q.x(), 0, maxQ), std::clamp(q.y(), 0, maxQ),
                std::clamp(q.z(), 0, maxQ)};
    }
    return {q.x(), q.y(), q.z()};
}

inline bool scIsGroupBoundaryHalfedge(MCGAL::Halfedge* h, int g) {
    if (!h || h->isRemoved() || !h->face() || h->face()->isRemoved() ||
        !h->end_vertex() || h->end_vertex()->isRemoved())
        return false;
    if (h->face()->groupId() != g) return false;
    MCGAL::Halfedge* opp = h->opposite();
    if (!opp || opp->isRemoved() || !opp->face() || opp->face()->isRemoved()) return false;
    return opp->face()->groupId() != g;
}

/// Rotate from boundary halfedge h (face group g, g on the left) around its end
/// vertex through the group-g face fan until the next boundary edge is found.
inline MCGAL::Halfedge* scNextGroupBoundaryHalfedge(MCGAL::Halfedge* h, int g) {
    if (!h) return nullptr;
    MCGAL::Halfedge* e = h->next();
    int guard = 0;
    while (e && ++guard < 256) {
        MCGAL::Halfedge* opp = e->opposite();
        if (!opp || opp->isRemoved() || !opp->face() || opp->face()->isRemoved()) return nullptr;
        if (opp->face()->groupId() != g) return e;
        e = opp->next();
    }
    return nullptr;
}

/// One deterministic anchor per group-boundary loop (min quantized position
/// vertex of the loop).  Anchors must be frozen/protected by the caller.
inline std::vector<ScAnchorDef> collectGroupAnchors(MCGAL::Mesh* mesh) {
    std::vector<ScAnchorDef> out;
    if (!mesh) return out;
    int maxGid = -1;
    for (MCGAL::Facet* f : mesh->faces())
        if (f && !f->isRemoved()) maxGid = std::max(maxGid, f->groupId());
    for (int g = 0; g <= maxGid; ++g) {
        std::vector<MCGAL::Halfedge*> edges;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved() || !f->proxyHalfedge() || f->groupId() != g) continue;
            MCGAL::Halfedge* h0 = f->proxyHalfedge();
            MCGAL::Halfedge* h = h0;
            int guard = 0;
            do {
                if (scIsGroupBoundaryHalfedge(h, g)) edges.push_back(h);
                h = h ? h->next() : nullptr;
            } while (h && h != h0 && ++guard < 64);
        }
        std::set<MCGAL::Halfedge*> visited;
        for (MCGAL::Halfedge* seed : edges) {
            if (!seed || visited.count(seed)) continue;
            std::vector<MCGAL::Halfedge*> loop;
            MCGAL::Halfedge* h = seed;
            int guard = 0;
            while (h && !visited.count(h) && ++guard < 100000) {
                visited.insert(h);
                loop.push_back(h);
                MCGAL::Halfedge* next = scNextGroupBoundaryHalfedge(h, g);
                if (!next || next == seed) break;
                h = next;
            }
            if (loop.empty()) continue;
            MCGAL::Vertex* best = nullptr;
            std::array<int, 3> bestQ{0, 0, 0};
            for (MCGAL::Halfedge* he : loop) {
                MCGAL::Vertex* cands[2] = {he ? he->vertex() : nullptr,
                                          he ? he->end_vertex() : nullptr};
                for (MCGAL::Vertex* v : cands) {
                    if (!v || v->isRemoved()) continue;
                    const auto q = scAnchorQ(mesh, v);
                    if (!best || q < bestQ) { best = v; bestQ = q; }
                }
            }
            if (!best) continue;
            ScAnchorDef d;
            d.v = best;
            d.group = g;
            out.push_back(d);
        }
    }
    return out;
}

/// Walk the group-g boundary loop containing anchor, starting from the
/// outgoing boundary edge at anchor with deterministic tie-breaking.
inline bool walkGroupLoop(MCGAL::Mesh* mesh, MCGAL::Vertex* anchor, int g,
                          std::vector<MCGAL::Halfedge*>& loop) {
    loop.clear();
    if (!mesh || !anchor || anchor->isRemoved()) return false;
    // Topology-stable tie-break (positions can drift by one quantization step
    // between encoder and decoder): opposite group id, then the sorted incident
    // group-id signature of the next vertex.  Quantized position is only a last
    // resort for symmetry ties.
    auto vertexGroupSig = [&](MCGAL::Vertex* v) {
        std::vector<int> gs;
        for (MCGAL::Halfedge* hh : pmsf::boundedRing(v)) {
            if (!hh || hh->isRemoved()) continue;
            if (hh->face() && !hh->face()->isRemoved()) gs.push_back(hh->face()->groupId());
            MCGAL::Halfedge* op = hh->opposite();
            if (op && !op->isRemoved() && op->face() && !op->face()->isRemoved())
                gs.push_back(op->face()->groupId());
        }
        std::sort(gs.begin(), gs.end());
        return gs;
    };
    MCGAL::Halfedge* start = nullptr;
    int bestOpp = 0;
    std::vector<int> bestSig;
    std::array<int, 3> bestQ{0, 0, 0};
    for (MCGAL::Halfedge* h : pmsf::boundedRing(anchor)) {
        if (!h || h->vertex() != anchor || h->isRemoved()) continue;
        if (!scIsGroupBoundaryHalfedge(h, g)) continue;
        const int opp = h->opposite()->face()->groupId();
        auto sig = vertexGroupSig(h->end_vertex());
        const auto q = scAnchorQ(mesh, h->end_vertex());
        bool better = false;
        if (!start) better = true;
        else if (opp != bestOpp) better = opp < bestOpp;
        else if (sig != bestSig) better = sig < bestSig;
        else better = q < bestQ;
        if (better) { start = h; bestOpp = opp; bestSig = std::move(sig); bestQ = q; }
    }
    if (!start) return false;
    MCGAL::Halfedge* h = start;
    int guard = 0;
    MCGAL::Halfedge* lastGood = start;
    MCGAL::Halfedge* breakEdge = nullptr;
    while (h && ++guard < 100000) {
        loop.push_back(h);
        lastGood = h;
        MCGAL::Halfedge* next = scNextGroupBoundaryHalfedge(h, g);
        if (!next) { breakEdge = h; break; }
        if (next == start) return true;
        h = next;
    }
    return false;
}

/// Deterministic group-boundary enumeration hash for the anchor-enumerated
/// boundary stream design: for every group-boundary loop, walk it from its
/// topology-stable anchor (minimum quantized position vertex, frozen on the
/// encoder side) and hash the sequence of per-edge (opposite-group, end-vertex
/// group signature, vertex degree) tuples.  No halfedge ids / container order
/// are used, so an encoder and a decoder state that are topologically identical
/// must produce the same hash.  Returns a sorted vector of (group, hash) pairs.
inline std::vector<std::pair<int, unsigned long long>> groupBoundaryEnumHashes(MCGAL::Mesh* mesh) {
    std::vector<std::pair<int, unsigned long long>> out;
    if (!mesh) return out;
    const auto anchors = collectGroupAnchors(mesh);
    for (const auto& d : anchors) {
        std::vector<MCGAL::Halfedge*> loop;
        if (!walkGroupLoop(mesh, d.v, d.group, loop) || loop.empty()) continue;
        // Per-edge topological key: (degree, sorted incident group-id signature).
        auto edgeKey = [&](MCGAL::Halfedge* e) {
            std::vector<int> key;
            if (!e || e->isRemoved() || !e->end_vertex()) return key;
            MCGAL::Vertex* v = e->end_vertex();
            std::vector<int> sig;
            int deg = 0;
            for (MCGAL::Halfedge* hh : pmsf::boundedRing(v)) {
                if (!hh || hh->isRemoved()) continue;
                ++deg;
                if (hh->face() && !hh->face()->isRemoved()) sig.push_back(hh->face()->groupId());
                MCGAL::Halfedge* op = hh->opposite();
                if (op && !op->isRemoved() && op->face() && !op->face()->isRemoved())
                    sig.push_back(op->face()->groupId());
            }
            std::sort(sig.begin(), sig.end());
            key.push_back(deg);
            // Pure topology signature for the enumeration-determinism test:
            // do NOT include group ids (they may differ transiently between
            // encoder and decoder states even when the topology is identical).
            return key;
        };
        std::vector<std::vector<int>> keys;
        keys.reserve(loop.size());
        for (MCGAL::Halfedge* e : loop) keys.push_back(edgeKey(e));
        auto flatSeq = [&](std::size_t rot) {
            std::vector<int> flat;
            for (std::size_t i = 0; i < keys.size(); ++i) {
                const auto& k = keys[(rot + i) % keys.size()];
                flat.push_back(static_cast<int>(k.size()));
                flat.insert(flat.end(), k.begin(), k.end());
            }
            return flat;
        };
        std::size_t best = 0;
        std::vector<int> bestSeq = flatSeq(0);
        for (std::size_t r = 1; r < keys.size(); ++r) {
            auto seq = flatSeq(r);
            if (seq < bestSeq) { bestSeq = std::move(seq); best = r; }
        }
        unsigned long long h = 1469598103934665603ull;
        auto mix = [&](unsigned long long v) { h ^= v; h *= 1099511628211ull; };
        mix(static_cast<unsigned long long>(bestSeq.size()));
        for (int v : bestSeq) mix(static_cast<unsigned long long>(v + 1));
        mix(static_cast<unsigned long long>(loop.size()));
        out.emplace_back(d.group, h);
    }
    std::sort(out.begin(), out.end());
    return out;
}

// ---------------------------------------------------------------------------
// Topology-only group-boundary enumeration (boundary v2, D522/D524).
//
// Anchor: boundary vertex with the canonical minimum topological key (no
// quantized positions involved).  Loop walk: outgoing boundary halfedge chosen
// by (opposite group, sorted neighbour group signature, canonical vertex key)
// and then rotated so the sequence of vertex keys is lexicographically minimal.
// Both sides of a correct encode/decode pair therefore enumerate the same loop
// even if face/halfedge container order or proxy assignment differs.
// ---------------------------------------------------------------------------
inline std::vector<std::pair<int, std::vector<int>>> scTopoEnumVertexKeys(
    MCGAL::Mesh* mesh, const std::vector<MCGAL::Halfedge*>& loop) {
    std::vector<std::pair<int, std::vector<int>>> out;
    out.reserve(loop.size());
    for (MCGAL::Halfedge* e : loop) {
        out.emplace_back(e ? 1 : 0, e ? scTopoVertexKey(mesh, e->vertex()) : std::vector<int>{});
    }
    return out;
}

inline std::vector<std::pair<int, std::vector<MCGAL::Halfedge*>>> collectTopoGroupLoops(
    MCGAL::Mesh* mesh) {
    std::vector<std::pair<int, std::vector<MCGAL::Halfedge*>>> out;
    if (!mesh) return out;
    int maxGid = -1;
    for (MCGAL::Facet* f : mesh->faces())
        if (f && !f->isRemoved()) maxGid = std::max(maxGid, f->groupId());
    for (int g = 0; g <= maxGid; ++g) {
        std::vector<MCGAL::Halfedge*> edges;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved() || !f->proxyHalfedge() || f->groupId() != g) continue;
            MCGAL::Halfedge* h0 = f->proxyHalfedge();
            MCGAL::Halfedge* h = h0;
            int guard = 0;
            do {
                if (scIsGroupBoundaryHalfedge(h, g)) edges.push_back(h);
                h = h ? h->next() : nullptr;
            } while (h && h != h0 && ++guard < 64);
        }
        std::set<MCGAL::Halfedge*> visited;
        for (MCGAL::Halfedge* seed : edges) {
            if (!seed || visited.count(seed)) continue;
            std::vector<MCGAL::Halfedge*> loop;
            MCGAL::Halfedge* h = seed;
            int guard = 0;
            while (h && !visited.count(h) && ++guard < 100000) {
                visited.insert(h);
                loop.push_back(h);
                MCGAL::Halfedge* next = scNextGroupBoundaryHalfedge(h, g);
                if (!next || next == seed) break;
                h = next;
            }
            if (loop.size() < 2) continue;
            // canonical rotation by topology-only vertex keys
            std::vector<std::vector<int>> keys;
            keys.reserve(loop.size());
            for (MCGAL::Halfedge* e : loop) keys.push_back(scTopoVertexKey(mesh, e->vertex()));
            auto flatSeq = [&](std::size_t rot) {
                std::vector<int> flat;
                for (std::size_t i = 0; i < loop.size(); ++i) {
                    const auto& k = keys[(rot + i) % loop.size()];
                    flat.push_back(static_cast<int>(k.size()));
                    flat.insert(flat.end(), k.begin(), k.end());
                }
                return flat;
            };
            std::size_t best = 0;
            std::vector<int> bestSeq = flatSeq(0);
            for (std::size_t r = 1; r < loop.size(); ++r) {
                auto seq = flatSeq(r);
                if (seq < bestSeq) { bestSeq = std::move(seq); best = r; }
            }
            std::vector<MCGAL::Halfedge*> rot;
            rot.reserve(loop.size());
            for (std::size_t i = 0; i < loop.size(); ++i)
                rot.push_back(loop[(best + i) % loop.size()]);
            out.emplace_back(g, std::move(rot));
        }
    }
    // loop order canonical by topological key of first edge's vertex
    std::sort(out.begin(), out.end(),
              [&](const std::pair<int, std::vector<MCGAL::Halfedge*>>& a,
                  const std::pair<int, std::vector<MCGAL::Halfedge*>>& b) {
                  if (a.first != b.first) return a.first < b.first;
                  const auto ka = scTopoVertexKey(mesh, a.second.front()->vertex());
                  const auto kb = scTopoVertexKey(mesh, b.second.front()->vertex());
                  if (ka != kb) return ka < kb;
                  return a.second.size() < b.second.size();
              });
    return out;
}

/// One anchor vertex per topological group-boundary loop, chosen purely by the
/// canonical topology key (minimum).  No quantized positions involved.
inline std::vector<std::pair<int, MCGAL::Vertex*>> collectTopoGroupAnchors(MCGAL::Mesh* mesh) {
    std::vector<std::pair<int, MCGAL::Vertex*>> out;
    const auto loops = collectTopoGroupLoops(mesh);
    for (const auto& gl : loops) {
        MCGAL::Vertex* best = nullptr;
        std::vector<int> bestKey;
        for (MCGAL::Halfedge* e : gl.second) {
            for (MCGAL::Vertex* v : {e ? e->vertex() : nullptr, e ? e->end_vertex() : nullptr}) {
                if (!v || v->isRemoved()) continue;
                auto key = scTopoVertexKey(mesh, v);
                if (!best || key < bestKey) { best = v; bestKey = std::move(key); }
            }
        }
        if (best) out.emplace_back(gl.first, best);
    }
    std::sort(out.begin(), out.end(), [&](const auto& a, const auto& b) {
        if (a.first != b.first) return a.first < b.first;
        const auto ka = scTopoVertexKey(mesh, a.second);
        const auto kb = scTopoVertexKey(mesh, b.second);
        return ka < kb;
    });
    return out;
}

/// Topology-only walk from a fixed anchor vertex (boundary v2 enumeration).
inline bool walkGroupLoopTopo(MCGAL::Mesh* mesh, MCGAL::Vertex* anchor, int g,
                              std::vector<MCGAL::Halfedge*>& loop) {
    loop.clear();
    if (!mesh || !anchor || anchor->isRemoved()) return false;
    auto neighborGroupSig = [&](MCGAL::Vertex* v) {
        std::vector<int> gs;
        for (MCGAL::Halfedge* hh : pmsf::boundedRing(v)) {
            if (!hh || hh->isRemoved()) continue;
            if (hh->face() && !hh->face()->isRemoved()) gs.push_back(hh->face()->groupId());
            MCGAL::Halfedge* op = hh->opposite();
            if (op && !op->isRemoved() && op->face() && !op->face()->isRemoved())
                gs.push_back(op->face()->groupId());
        }
        std::sort(gs.begin(), gs.end());
        return gs;
    };
    MCGAL::Halfedge* start = nullptr;
    int bestOpp = 0;
    std::vector<int> bestSig;
    std::vector<int> bestKey;
    for (MCGAL::Halfedge* h : pmsf::boundedRing(anchor)) {
        if (!h || h->vertex() != anchor || h->isRemoved()) continue;
        if (!scIsGroupBoundaryHalfedge(h, g)) continue;
        const int opp = h->opposite()->face()->groupId();
        auto sig = neighborGroupSig(h->end_vertex());
        auto key = scTopoVertexKey(mesh, h->end_vertex());
        bool better = false;
        if (!start) better = true;
        else if (opp != bestOpp) better = opp < bestOpp;
        else if (sig != bestSig) better = sig < bestSig;
        else better = key < bestKey;
        if (better) { start = h; bestOpp = opp; bestSig = std::move(sig); bestKey = std::move(key); }
    }
    if (!start) return false;
    MCGAL::Halfedge* h = start;
    int guard = 0;
    MCGAL::Halfedge* lastGood = start;
    MCGAL::Halfedge* breakEdge = nullptr;
    while (h && ++guard < 100000) {
        loop.push_back(h);
        lastGood = h;
        MCGAL::Halfedge* next = scNextGroupBoundaryHalfedge(h, g);
        if (!next) { breakEdge = h; break; }
        if (next == start) return true;
        h = next;
    }
    return false;
}

/// Topology-only enumeration hash (boundary v2 diagnostics): per loop, walk from
/// the canonical topological anchor and hash the sequence of (degree,
/// neighbour-group-signature) keys; rotation-invariant via canonical rotation.
inline std::vector<std::pair<int, unsigned long long>> groupBoundaryEnumHashesTopo(MCGAL::Mesh* mesh) {
    std::vector<std::pair<int, unsigned long long>> out;
    if (!mesh) return out;
    const auto anchors = collectTopoGroupAnchors(mesh);
    for (const auto& d : anchors) {
        std::vector<MCGAL::Halfedge*> loop;
        if (!walkGroupLoopTopo(mesh, d.second, d.first, loop) || loop.empty()) continue;
        auto edgeKey = [&](MCGAL::Halfedge* e) {
            std::vector<int> key;
            if (!e || e->isRemoved() || !e->end_vertex()) return key;
            MCGAL::Vertex* v = e->end_vertex();
            std::vector<int> sig;
            int deg = 0;
            for (MCGAL::Halfedge* hh : pmsf::boundedRing(v)) {
                if (!hh || hh->isRemoved()) continue;
                ++deg;
                if (hh->face() && !hh->face()->isRemoved()) sig.push_back(hh->face()->groupId());
                MCGAL::Halfedge* op = hh->opposite();
                if (op && !op->isRemoved() && op->face() && !op->face()->isRemoved())
                    sig.push_back(op->face()->groupId());
            }
            std::sort(sig.begin(), sig.end());
            key.push_back(deg);
            key.push_back(static_cast<int>(sig.size()));
            for (int gv : sig) key.push_back(gv + 1);
            return key;
        };
        std::vector<std::vector<int>> keys;
        keys.reserve(loop.size());
        for (MCGAL::Halfedge* e : loop) keys.push_back(edgeKey(e));
        auto flatSeq = [&](std::size_t rot) {
            std::vector<int> flat;
            for (std::size_t i = 0; i < keys.size(); ++i) {
                const auto& k = keys[(rot + i) % keys.size()];
                flat.push_back(static_cast<int>(k.size()));
                flat.insert(flat.end(), k.begin(), k.end());
            }
            return flat;
        };
        std::size_t best = 0;
        std::vector<int> bestSeq = flatSeq(0);
        for (std::size_t r = 1; r < keys.size(); ++r) {
            auto seq = flatSeq(r);
            if (seq < bestSeq) { bestSeq = std::move(seq); best = r; }
        }
        unsigned long long hsh = 1469598103934665603ull;
        auto mix = [&](unsigned long long v) { hsh ^= v; hsh *= 1099511628211ull; };
        mix(static_cast<unsigned long long>(bestSeq.size()));
        for (int v : bestSeq) mix(static_cast<unsigned long long>(v + 1));
        mix(static_cast<unsigned long long>(loop.size()));
        out.emplace_back(d.first, hsh);
    }
    std::sort(out.begin(), out.end());
    return out;
}

/// All-loop group-boundary enumeration hash (boundary v2 diagnostic): for every
/// canonical loop from collectTopoGroupLoops, hash the rotation-canonical sequence
/// of (degree, neighbour-group-signature) keys, then combine all loops of a group
/// into one order-independent group hash.  No anchor/junction tie-break involved.
inline std::vector<std::pair<int, unsigned long long>> groupBoundaryEnumHashesAllLoops(
    MCGAL::Mesh* mesh) {
    std::vector<std::pair<int, unsigned long long>> out;
    if (!mesh) return out;
    const auto loops = collectTopoGroupLoops(mesh);
    std::map<int, std::vector<unsigned long long>> perGroup;
    for (const auto& gl : loops) {
        const int g = gl.first;
        const auto& loop = gl.second;
        auto edgeKey = [&](MCGAL::Halfedge* e) {
            std::vector<int> key;
            if (!e || e->isRemoved() || !e->end_vertex()) return key;
            MCGAL::Vertex* v = e->end_vertex();
            std::vector<int> sig;
            int deg = 0;
            for (MCGAL::Halfedge* hh : pmsf::boundedRing(v)) {
                if (!hh || hh->isRemoved()) continue;
                ++deg;
                if (hh->face() && !hh->face()->isRemoved()) sig.push_back(hh->face()->groupId());
                MCGAL::Halfedge* op = hh->opposite();
                if (op && !op->isRemoved() && op->face() && !op->face()->isRemoved())
                    sig.push_back(op->face()->groupId());
            }
            std::sort(sig.begin(), sig.end());
            key.push_back(deg);
            key.push_back(static_cast<int>(sig.size()));
            for (int gv : sig) key.push_back(gv + 1);
            return key;
        };
        std::vector<std::vector<int>> keys;
        keys.reserve(loop.size());
        for (MCGAL::Halfedge* e : loop) keys.push_back(edgeKey(e));
        auto flatSeq = [&](std::size_t rot) {
            std::vector<int> flat;
            for (std::size_t i = 0; i < keys.size(); ++i) {
                const auto& k = keys[(rot + i) % keys.size()];
                flat.push_back(static_cast<int>(k.size()));
                flat.insert(flat.end(), k.begin(), k.end());
            }
            return flat;
        };
        std::size_t best = 0;
        std::vector<int> bestSeq = flatSeq(0);
        for (std::size_t r = 1; r < keys.size(); ++r) {
            auto seq = flatSeq(r);
            if (seq < bestSeq) { bestSeq = std::move(seq); best = r; }
        }
        (void)best;
        unsigned long long lh = 1469598103934665603ull;
        auto mix = [&](unsigned long long v) { lh ^= v; lh *= 1099511628211ull; };
        mix(static_cast<unsigned long long>(bestSeq.size()));
        for (int v : bestSeq) mix(static_cast<unsigned long long>(v + 1));
        mix(static_cast<unsigned long long>(loop.size()));
        perGroup[g].push_back(lh);
    }
    for (auto& kv : perGroup) {
        std::sort(kv.second.begin(), kv.second.end());
        unsigned long long gh = 1469598103934665603ull;
        auto mix = [&](unsigned long long v) { gh ^= v; gh *= 1099511628211ull; };
        mix(static_cast<unsigned long long>(kv.second.size()));
        for (unsigned long long lh : kv.second) mix(lh);
        out.emplace_back(kv.first, gh);
    }
    return out;
}

/// Diagnostic: dump the canonical topology-only loop enumeration as quantized
/// vertex positions (sequences only; positions are NOT used for enumeration).
inline void dumpTopoGroupLoopSequences(MCGAL::Mesh* mesh, const char* tag, int round) {
    if (!mesh) return;
    const auto loops = collectTopoGroupLoops(mesh);
    std::fprintf(stderr, "[bc-enumdbg] %s round=%d loops=%zu\n", tag, round, loops.size());
    for (const auto& gl : loops) {
        std::fprintf(stderr, "[bc-enum] %s round=%d g=%d n=%zu", tag, round, gl.first,
                     gl.second.size());
        for (MCGAL::Halfedge* e : gl.second) {
            if (!e || !e->vertex()) continue;
            const auto q = mesh->floatPosToInt(e->vertex()->point());
            std::fprintf(stderr, " %d,%d,%d", q.x(), q.y(), q.z());
        }
        std::fprintf(stderr, "\n");
    }
}

/// Fixed-anchor enumeration (boundary v2 diagnostics and later payload).
///
/// Picks one **non-junction** anchor vertex per canonical group-boundary loop of
/// the given mesh.  A non-junction boundary vertex has exactly one outgoing
/// group-boundary halfedge for its group, so the loop enumeration starting at
/// that vertex is unique (no rotation tie-break, no positions).
inline std::vector<std::pair<int, MCGAL::Vertex*>> collectFixedLoopAnchors(MCGAL::Mesh* mesh) {
    std::vector<std::pair<int, MCGAL::Vertex*>> out;
    if (!mesh) return out;
    const auto loops = collectTopoGroupLoops(mesh);
    for (const auto& gl : loops) {
        const int g = gl.first;
        MCGAL::Vertex* best = nullptr;
        for (MCGAL::Halfedge* e : gl.second) {
            if (!e || !e->vertex() || e->vertex()->isRemoved()) continue;
            MCGAL::Vertex* v = e->vertex();
            int outgoing = 0;
            for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
                if (!h || h->isRemoved() || h->vertex() != v) continue;
                if (scIsGroupBoundaryHalfedge(h, g)) ++outgoing;
            }
            if (outgoing == 1) { best = v; break; }
        }
        if (best) out.emplace_back(g, best);
    }
    return out;
}

/// Walk the group-g boundary loop starting at the unique outgoing group-boundary
/// halfedge of a non-junction anchor.  Returns false if the anchor is not a
/// non-junction boundary vertex of this group or the loop does not close.
inline bool walkLoopFromFixedAnchor(MCGAL::Mesh* mesh, MCGAL::Vertex* anchor, int g,
                                    std::vector<MCGAL::Halfedge*>& loop) {
    loop.clear();
    if (!mesh || !anchor || anchor->isRemoved()) return false;
    MCGAL::Halfedge* start = nullptr;
    for (MCGAL::Halfedge* h : pmsf::boundedRing(anchor)) {
        if (!h || h->isRemoved() || h->vertex() != anchor) continue;
        if (!scIsGroupBoundaryHalfedge(h, g)) continue;
        if (start) return false;  // junction vertex -> ambiguous start
        start = h;
    }
    if (!start) return false;
    MCGAL::Halfedge* h = start;
    int guard = 0;
    while (h && ++guard < 100000) {
        loop.push_back(h);
        MCGAL::Halfedge* next = scNextGroupBoundaryHalfedge(h, g);
        if (!next) break;
        if (next == start) return true;
        h = next;
    }
    return false;
}

/// Diagnostic dump of anchor-driven loop enumeration as quantized positions
/// (positions are only printed, never used for the enumeration itself).
inline void dumpFixedAnchorLoopSequences(
    MCGAL::Mesh* mesh, const std::vector<std::pair<int, MCGAL::Vertex*>>& anchors,
    const char* tag, int round) {
    if (!mesh) return;
    for (const auto& ag : anchors) {
        std::vector<MCGAL::Halfedge*> loop;
        if (!walkLoopFromFixedAnchor(mesh, ag.second, ag.first, loop)) {
            std::fprintf(stderr, "[bc-anc] %s round=%d g=%d anchorFail\n", tag, round, ag.first);
            continue;
        }
        std::fprintf(stderr, "[bc-anc] %s round=%d g=%d n=%zu", tag, round, ag.first, loop.size());
        for (MCGAL::Halfedge* e : loop) {
            if (!e || !e->vertex()) continue;
            const auto q = mesh->floatPosToInt(e->vertex()->point());
            std::fprintf(stderr, " %d,%d,%d", q.x(), q.y(), q.z());
        }
        std::fprintf(stderr, "\n");
    }
}

/// Capture anchor-addressed patch: [len=8][anchorId][edgeOffset][gA][kA][kB][rq3]
// ---------------------------------------------------------------------------
// Boundary v2 (mode 70): fixed-anchor enumerated boundary edge stream.
//
// One fixed non-junction anchor per group-boundary loop enumerates the loop in
// topological order (see walkLoopFromFixedAnchor).  Per stage round the encoder
// records one bit per enumerated boundary edge (1 = participated in the round's
// boundary topology compression) plus, for every marked edge in enumeration
// order, the inverse-collapse attributes [kA][kB][removed q(3)].  The decoder
// rebuilds the same enumeration from base-index anchors and applies inverse
// splits for the marked edges; positions are only attributes.
// ---------------------------------------------------------------------------
struct BoundaryV2Op {
    int group = -1;
    int gB = -1;  // V3: opposite-side group of the merged boundary edge (ID-side disambiguation)
    MCGAL::Vertex* thirdA = nullptr;  // V3 ring certificate: third vertex of capture-side face (gA)
    MCGAL::Vertex* thirdB = nullptr;  // V3 ring certificate: third vertex of opposite face (gB)
    int kA = 0;
    int kB = 0;
    std::array<int, 3> rq{0, 0, 0};
    std::array<int, 3> removedQ{0, 0, 0};  // V3: absolute clamped q of the removed vertex
    MCGAL::Vertex* removedVertex = nullptr;  // V3: S-ID binding (assigned after final sort)
    MCGAL::Vertex* a = nullptr;  // merged boundary edge endpoints (post-stage)
    MCGAL::Vertex* b = nullptr;
    int orient = 0;  // 0: enumeration edge itself is the raw capture orientation
    // Ordered-inverse metadata (only when PMSF_BND_V3_ORDER_ROUNDS is enabled):
    // pre-collapse container ranks for the removed vertex and the kA+kB restored faces.
    int vRank = -1;
    std::vector<int> faceRanks;
    // Explicit-patch data (V3 optional): the complete topology of the forward patch,
    // so the decoder can rebuild it by ID even when incremental inverse context has
    // drifted.  Only filled when PMSF_BND_V3_EXPLICIT is enabled.
    std::vector<std::array<MCGAL::Vertex*, 3>> expOrig;
    std::vector<int> expOrigGid;
    std::vector<std::array<MCGAL::Vertex*, 3>> expAdd;
    std::vector<int> expAddGid;
};

struct BoundaryV2Round {
    std::vector<BoundaryV2Op> ops;    // sorted by bit index (enumeration order)
    std::vector<unsigned char> bits;  // one bit per anchor-enumerated boundary edge
    int edgeCount = 0;
    // Global topology invariant after the stage: group-boundary (undirected) + open edges.
    // -1 = not recorded (old artifacts).  Decoder validates the inverse round against it.
    int groupBoundaryEdges = -1;
};

inline bool scV2TouchesLocked(MCGAL::Vertex* v, const std::set<MCGAL::Vertex*>& locked) {
    if (!v || v->isRemoved()) return true;
    if (locked.count(v)) return true;
    for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
        if (h && !h->isRemoved() && h->end_vertex() && locked.count(h->end_vertex())) return true;
    }
    return false;
}

/// Concatenate the anchor-driven boundary enumerations.  Returns false if any
/// anchor is not a valid non-junction boundary vertex (encoder/decoder mismatch).
inline void scV2SyncAnchorProxies(
    MCGAL::Mesh* mesh, const std::vector<std::pair<int, MCGAL::Vertex*>>& anchors) {
    if (!mesh) return;
    for (const auto& ag : anchors) {
        MCGAL::Vertex* v = ag.second;
        if (!v || v->isRemoved()) continue;
        MCGAL::Halfedge* best = nullptr;
        for (MCGAL::Halfedge* h : v->halfedges()) {
            if (!h || h->isRemoved() || h->vertex() != v) continue;
            best = h;
            break;
        }
        if (best) v->setProxyHalfedge(best);
    }
}

inline bool scV2EnumerateAnchors(
    MCGAL::Mesh* mesh, const std::vector<std::pair<int, MCGAL::Vertex*>>& anchors,
    std::vector<MCGAL::Halfedge*>& edges, std::vector<int>* anchorEdgeOffsets = nullptr) {
    edges.clear();
    scV2SyncAnchorProxies(mesh, anchors);
    if (anchorEdgeOffsets) anchorEdgeOffsets->clear();
    for (const auto& ag : anchors) {
        if (anchorEdgeOffsets) anchorEdgeOffsets->push_back(static_cast<int>(edges.size()));
        std::vector<MCGAL::Halfedge*> loop;
        if (!walkLoopFromFixedAnchor(mesh, ag.second, ag.first, loop)) {
            if (std::getenv("PMSF_BC_V2_DEBUG")) {
                int out = 0, deg = 0;
                if (ag.second && !ag.second->isRemoved()) {
                    const auto q = mesh->floatPosToInt(ag.second->point());
                    for (MCGAL::Halfedge* h : pmsf::boundedRing(ag.second)) {
                        if (!h || h->isRemoved() || h->vertex() != ag.second) continue;
                        ++deg;
                        if (scIsGroupBoundaryHalfedge(h, ag.first)) ++out;
                    }
                    int deg2 = 0;
                    for (MCGAL::Halfedge* h : pmsf::boundedRing(ag.second)) {
                        if (h && !h->isRemoved() && h->vertex() == ag.second) ++deg2;
                    }
                    int ptrMember = 0;
                    for (MCGAL::Vertex* vv : mesh->vertices())
                        if (vv == ag.second) { ptrMember = 1; break; }
                    int incPtr = 0, incQ = 0, orphanFaceVerts = 0;
                    std::set<MCGAL::Vertex*> vset(mesh->vertices().begin(), mesh->vertices().end());
                    for (MCGAL::Facet* ff : mesh->faces()) {
                        if (!ff || ff->isRemoved()) continue;
                        const auto fv = pmsf::sbdetail::faceVertices(ff);
                        bool hasPtr = false, hasQ = false;
                        for (MCGAL::Vertex* vv : fv) {
                            if (!vv) continue;
                            if (vv == ag.second) hasPtr = true;
                            const auto qq = mesh->floatPosToInt(vv->point());
                            if (qq.x() == q.x() && qq.y() == q.y() && qq.z() == q.z())
                                hasQ = true;
                            if (!vset.count(vv)) ++orphanFaceVerts;
                        }
                        if (hasPtr) ++incPtr;
                        if (hasQ) ++incQ;
                    }
                    int foundSameQ = 0;
                    MCGAL::Vertex* sameQ = nullptr;
                    for (MCGAL::Vertex* vv : mesh->vertices()) {
                        if (!vv || vv->isRemoved()) continue;
                        const auto qq = mesh->floatPosToInt(vv->point());
                        if (qq.x() == q.x() && qq.y() == q.y() && qq.z() == q.z()) {
                            ++foundSameQ;
                            sameQ = vv;
                        }
                    }
                    std::fprintf(stderr,
                                 "[bc-v2-enum-fail] g=%d q=(%d,%d,%d) removed=%d ringOut=%d "
                                 "deg=%d deg2=%d ptrMember=%d vlist=%zu sameQCount=%d incPtr=%d "
                                 "incQ=%d orphanFV=%d half=%zu\n",
                                 ag.first, q.x(), q.y(), q.z(),
                                 ag.second->isRemoved() ? 1 : 0, out, deg, deg2,
                                 ptrMember, mesh->vertices().size(), foundSameQ, incPtr, incQ,
                                 orphanFaceVerts, ag.second->halfedges().size());
                } else {
                    std::fprintf(stderr, "[bc-v2-enum-fail] g=%d anchor null/removed\n", ag.first);
                }
            }
            return false;
        }
        edges.insert(edges.end(), loop.begin(), loop.end());
    }
    if (anchorEdgeOffsets) anchorEdgeOffsets->push_back(static_cast<int>(edges.size()));
    return true;
}

/// Encoder stage for boundary v2: collapses boundary vertices with the existing
/// boundary-vertex collapse primitive, locks the ring of every merged edge so
/// the round's ops are independent, and captures [kA, kB, removed q, merged
/// edge endpoints] for the payload.  Returns the number of applied ops, or -1
/// if the post-stage anchor enumeration cannot locate a merged edge.
inline int runSingleCopyBoundaryStageV2(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                        int roundIndex, int maxOps,
                                        const std::set<MCGAL::Vertex*>& protectedVerts,
                                        const std::vector<std::pair<int, MCGAL::Vertex*>>& anchors,
                                        BoundaryV2Round& out) {
    out = BoundaryV2Round();
    if (!mesh || maxOps <= 0) return 0;
    // Uniformity target: per-group mean boundary edge length on the current mesh.
    // Used by the env-gated target-length candidate ordering below.
    std::unordered_map<int, double> groupMeanBoundaryLen;
    {
        std::unordered_map<int, double> sum;
        std::unordered_map<int, int> cnt;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
            for (MCGAL::Halfedge* h : *f) {
                if (!h || h->isRemoved() || !h->vertex() || !h->end_vertex()) continue;
                MCGAL::Halfedge* opp = h->opposite();
                if (!opp || opp->isRemoved() || !opp->face() || opp->face()->isRemoved()) continue;
                if (h->face()->groupId() == opp->face()->groupId()) continue;
                const int gid = h->face()->groupId();
                const double dx = static_cast<double>(h->end_vertex()->x()) - h->vertex()->x();
                const double dy = static_cast<double>(h->end_vertex()->y()) - h->vertex()->y();
                const double dz = static_cast<double>(h->end_vertex()->z()) - h->vertex()->z();
                sum[gid] += std::sqrt(dx * dx + dy * dy + dz * dz);
                ++cnt[gid];
            }
        }
        for (const auto& kv : cnt) {
            if (kv.second > 0) groupMeanBoundaryLen[kv.first] = sum[kv.first] / kv.second;
        }
    }
    const bool v2Debug = std::getenv("PMSF_BC_V2_DEBUG") != nullptr;
    if (v2Debug) {
        {
            char prePath[512];
            std::snprintf(prePath, sizeof(prePath), "/tmp/bc_v2_enc_pres_%03d.off", roundIndex);
            mesh->dumpto_oldtype(prePath);
        }
        std::vector<MCGAL::Halfedge*> preEdges;
        if (scV2EnumerateAnchors(mesh.get(), anchors, preEdges, nullptr))
            std::fprintf(stderr, "[bc-v2-stage-debug] round=%d preEdges=%zu\n", roundIndex,
                         preEdges.size());
    }
    auto qOf = [&](MCGAL::Vertex* v) {
        const auto qi = mesh->floatPosToInt(v->point());
        return std::array<int, 3>{qi.x(), qi.y(), qi.z()};
    };
    std::set<MCGAL::Vertex*> anchorSet;
    for (const auto& ag : anchors)
        if (ag.second) anchorSet.insert(ag.second);
    std::unordered_map<long long, int> posCount;
    posCount.reserve(mesh->vertices().size() * 2 + 1);
    auto posKey = [&](MCGAL::Vertex* v) {
        const auto q = qOf(v);
        return static_cast<long long>(q[0]) * 1000003LL * 1000003LL +
               static_cast<long long>(q[1]) * 1000003LL + q[2];
    };
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        ++posCount[posKey(v)];
    }
    auto uniquePos = [&](MCGAL::Vertex* v) {
        if (!v || v->isRemoved()) return false;
        auto it = posCount.find(posKey(v));
        return it != posCount.end() && it->second == 1;
    };
    std::set<MCGAL::Vertex*> locked;
    std::set<int> rejected;
    int applied = 0;
    auto scanCandidates = [&]() {
        std::set<MCGAL::Vertex*> out;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
            MCGAL::Halfedge* h0 = f->proxyHalfedge();
            MCGAL::Halfedge* h = h0;
            int guard = 0;
            do {
                MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
                if (h && h->vertex() && h->next() && h->next()->vertex() && opp &&
                    !opp->isRemoved() && opp->face() && !opp->face()->isRemoved() &&
                    opp->face()->groupId() != f->groupId()) {
                    if (!h->vertex()->isRemoved()) out.insert(h->vertex());
                    if (!h->next()->vertex()->isRemoved()) out.insert(h->next()->vertex());
                }
                h = h ? h->next() : nullptr;
            } while (h && h != h0 && ++guard < 64);
        }
        return out;
    };
    std::set<MCGAL::Vertex*> cachedCands;
    const int rescanEvery = []() {
        const char* e = std::getenv("PMSF_SC_BOUNDARY_KFREE_RESCAN");
        const int v = e ? std::atoi(e) : 64;
        return v > 0 ? v : 64;
    }();
    const int orderEvery = []() {
        const char* e = std::getenv("PMSF_SC_BOUNDARY_KFREE_ORDER");
        const int v = e ? std::atoi(e) : 1;
        return v > 0 ? v : 1;
    }();
    const bool pqMode =
        std::getenv("PMSF_SC_BOUNDARY_KFREE") != nullptr &&
        std::getenv("PMSF_SC_BOUNDARY_KFREE_PQ_OFF") == nullptr;
    struct KfEntry {
        double score;
        int id;
        MCGAL::Vertex* v;
    };
    struct KfCmp {
        bool operator()(const KfEntry& a, const KfEntry& b) const {
            if (a.score != b.score) return a.score > b.score;
            return a.id > b.id;
        }
    };
    std::priority_queue<KfEntry, std::vector<KfEntry>, KfCmp> pq;
    std::unordered_map<MCGAL::Vertex*, double> pqScore;
    double kfreeMaxRatio = 0.0;
    if (const char* um2 = std::getenv("PMSF_BC_V2_UNIFORM_MAX")) kfreeMaxRatio = std::atof(um2);
    FaceTripleMap stageFaceTriples;
    if (pqMode) stageFaceTriples = buildFaceTripleMap(mesh.get());
    auto scoreOf = [&](MCGAL::Vertex* v) -> double {
        if (!v || v->isRemoved()) return 1e18;
std::set<MCGAL::Vertex*> nb;
        for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
            if (!h || h->isRemoved() || !h->vertex() || !h->end_vertex()) continue;
            if (!h->face() || h->face()->isRemoved()) continue;
            MCGAL::Halfedge* opp = h->opposite();
            if (!opp || opp->isRemoved() || !opp->face() || opp->face()->isRemoved())
                continue;
            if (h->face()->groupId() == opp->face()->groupId()) continue;
            MCGAL::Vertex* w = (h->vertex() == v) ? h->end_vertex() : h->vertex();
            if (!w || w == v || w->isRemoved()) continue;
            nb.insert(w);
        }
if (nb.size() < 2) return 1e17;
        std::vector<double> lens;
        lens.reserve(2);
        for (MCGAL::Vertex* w : nb) {
            const double dx = static_cast<double>(w->x()) - v->x();
            const double dy = static_cast<double>(w->y()) - v->y();
            const double dz = static_cast<double>(w->z()) - v->z();
            lens.push_back(std::sqrt(dx * dx + dy * dy + dz * dz));
        }
        std::sort(lens.begin(), lens.end());
        const double lo = lens.front();
        const double hi = lens.back();
        double ratioScore = (lo > 1e-12) ? (hi / lo) : 1e16;
        bool targetEnabled =
            std::getenv("PMSF_BC_V2_UNIFORM_TARGET") != nullptr && nb.size() >= 2;
        if (targetEnabled) {
            if (const char* te = std::getenv("PMSF_BC_V2_UNIFORM_TARGET_EARLY")) {
                const int early = std::atoi(te);
                if (early > 0 && roundIndex >= early) targetEnabled = false;
            }
        }
        if (targetEnabled) {
            // Target-length score: the merged boundary edge should be close to
            // this group's mean boundary edge length (keeps spacing uniform).
            double dev = 0.0;
            int nd = 0;
            for (const auto& gm : groupMeanBoundaryLen) {
                const int gid = gm.first;
                const double mean = gm.second;
                if (mean <= 1e-12) continue;
                std::vector<MCGAL::Vertex*> side;
                for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
                    if (!h || h->isRemoved() || !h->vertex() || !h->end_vertex()) continue;
                    MCGAL::Halfedge* opp = h->opposite();
                    if (!opp || opp->isRemoved() || !opp->face() ||
                        opp->face()->isRemoved())
                        continue;
                    if (!h->face() || h->face()->isRemoved()) continue;
                    const bool onGroup =
                        (h->face()->groupId() == gid &&
                         opp->face()->groupId() != gid) ||
                        (opp->face()->groupId() == gid &&
                         h->face()->groupId() != gid);
                    if (!onGroup) continue;
                    MCGAL::Vertex* w =
                        (h->vertex() == v) ? h->end_vertex() : h->vertex();
                    if (w && w != v && !w->isRemoved()) side.push_back(w);
                }
                std::sort(side.begin(), side.end());
                side.erase(std::unique(side.begin(), side.end()), side.end());
                if (side.size() != 2) continue;
                const double dx = static_cast<double>(side[1]->x()) - side[0]->x();
                const double dy = static_cast<double>(side[1]->y()) - side[0]->y();
                const double dz = static_cast<double>(side[1]->z()) - side[0]->z();
                const double merged = std::sqrt(dx * dx + dy * dy + dz * dz);
                dev += std::fabs(merged - mean) / mean;
                ++nd;
            }
            if (nd > 0) ratioScore = std::min(ratioScore, dev / nd);
            // Local target variant (PMSF_BC_V2_UNIFORM_TARGET=2): target the
            // average of the two adjacent boundary edges, not the group mean.
            // Purely local, less aggressive, keeps decoder op sequences closer
            // to the previously-verified schedules on bunny/h19.
            const char* tm = std::getenv("PMSF_BC_V2_UNIFORM_TARGET");
            if (tm && tm[0] == '2') {
                for (const auto& gm : groupMeanBoundaryLen) {
                    const int gid = gm.first;
                    std::vector<std::pair<MCGAL::Vertex*, double>> side;
                    for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
                        if (!h || h->isRemoved() || !h->vertex() ||
                            !h->end_vertex())
                            continue;
                        MCGAL::Halfedge* opp = h->opposite();
                        if (!opp || opp->isRemoved() || !opp->face() ||
                            opp->face()->isRemoved())
                            continue;
                        if (!h->face() || h->face()->isRemoved()) continue;
                        const bool onGroup =
                            (h->face()->groupId() == gid &&
                             opp->face()->groupId() != gid) ||
                            (opp->face()->groupId() == gid &&
                             h->face()->groupId() != gid);
                        if (!onGroup) continue;
                        MCGAL::Vertex* w =
                            (h->vertex() == v) ? h->end_vertex() : h->vertex();
                        if (!w || w == v || w->isRemoved()) continue;
                        const double dx =
                            static_cast<double>(w->x()) - v->x();
                        const double dy =
                            static_cast<double>(w->y()) - v->y();
                        const double dz =
                            static_cast<double>(w->z()) - v->z();
                        side.emplace_back(w, std::sqrt(dx * dx + dy * dy + dz * dz));
                    }
                    if (side.size() != 2) continue;
                    const double target = (side[0].second + side[1].second) / 2.0;
                    if (target <= 1e-12) continue;
                    const double dx =
                        static_cast<double>(side[1].first->x()) - side[0].first->x();
                    const double dy =
                        static_cast<double>(side[1].first->y()) - side[0].first->y();
                    const double dz =
                        static_cast<double>(side[1].first->z()) - side[0].first->z();
                    const double merged = std::sqrt(dx * dx + dy * dy + dz * dz);
                    const double dev = std::fabs(merged - target) / target;
                    ratioScore = std::min(ratioScore, dev);
                }
            }
        }

        return ratioScore;
    };

    // Hoisted so the nextCand lambda (below) can capture it; assigned per iter.
    bool fullOrder = true;
    for (int iter = 0; iter < maxOps; ++iter) {
        fullOrder =
            (orderEvery <= 1) || (iter == 0) || (orderEvery > 0 && iter % orderEvery == 0);
        std::set<MCGAL::Vertex*> cands;
        if (iter == 0 || cachedCands.empty() || (rescanEvery > 0 && iter % rescanEvery == 0))
            cachedCands = scanCandidates();
        if (!pqMode) cands = cachedCands;
        std::vector<MCGAL::Vertex*> ordered;
        std::unordered_map<MCGAL::Vertex*, double> uniformScore;
        double uniformMaxRatio = 0.0;
        if (const char* um = std::getenv("PMSF_BC_V2_UNIFORM_MAX")) uniformMaxRatio = std::atof(um);
        if (!pqMode) {
        if (cands.empty()) break;
        ordered.assign(cands.begin(), cands.end());
        if (fullOrder && (std::getenv("PMSF_BC_V2_UNIFORM") || uniformMaxRatio > 0.0) &&
            ordered.size() > 1) {
            // Uniformity-driven ordering: prefer vertices whose two adjacent group
            // boundary edges have similar length (removing such a vertex keeps the
            // boundary spacing even).  Only affects the encoder candidate order; the
            // decoder consumes the recorded op list, so artifacts stay self-describing.
            uniformScore.reserve(ordered.size() * 2);
            for (MCGAL::Vertex* v : ordered) uniformScore[v] = scoreOf(v);
            std::sort(ordered.begin(), ordered.end(),
                      [&](MCGAL::Vertex* a, MCGAL::Vertex* b) {
                          const double sa = uniformScore[a], sb = uniformScore[b];
                          if (sa != sb) return sa < sb;
                          return (a ? a->poolId() : -1) < (b ? b->poolId() : -1);
                      });
        } else {
            std::sort(ordered.begin(), ordered.end(), [](MCGAL::Vertex* a, MCGAL::Vertex* b) {
                return (a ? a->poolId() : -1) < (b ? b->poolId() : -1);
            });
        }
        } else {
            if (iter == 0 || cachedCands.empty() ||
                (rescanEvery > 0 && iter % rescanEvery == 0)) {
                pq = decltype(pq)();
                pqScore.clear();
                for (MCGAL::Vertex* v : cachedCands) {
                    if (!v || v->isRemoved()) continue;
                    const double sc = scoreOf(v);
                    pqScore[v] = sc;
                    pq.push(KfEntry{sc, v->poolId(), v});
                }
            }
        }
        std::size_t orderedIdx = 0;
        auto nextCand = [&]() -> MCGAL::Vertex* {
            if (pqMode) {
                while (!pq.empty()) {
                    KfEntry e = pq.top();
                    pq.pop();
                    MCGAL::Vertex* v = e.v;
                    if (!v || v->isRemoved()) continue;
                    auto it = pqScore.find(v);
                    if (it == pqScore.end() || it->second != e.score) continue;
                    if (kfreeMaxRatio > 0.0 && e.score > kfreeMaxRatio) {
                        while (!pq.empty()) {
                            KfEntry x = pq.top();
                            pq.pop();
                            if (x.v) pqScore.erase(x.v);
                        }
                        return nullptr;
                    }
                    return v;
                }
                return nullptr;
            }
            if (orderedIdx < ordered.size()) return ordered[orderedIdx++];
            return nullptr;
        };
        bool ok = false;
        int dbgChecked = 0, dbgCollapseFail = 0, dbgClosedFail = 0;
        for (MCGAL::Vertex* v = nextCand(); v != nullptr; v = nextCand()) {
            if (!v || v->isRemoved() || rejected.count(v->poolId()) || protectedVerts.count(v) ||
                anchorSet.count(v))
                continue;
            ++dbgChecked;
            if (!pqMode && fullOrder && uniformMaxRatio > 0.0) {
                auto uit = uniformScore.find(v);
                if (uit == uniformScore.end() || uit->second > uniformMaxRatio) {
                    rejected.insert(v->poolId());
                    continue;
                }
            }
            if (scV2TouchesLocked(v, locked)) continue;
            if (!uniquePos(v)) continue;
            const auto rq = qOf(v);
            SingleBoundaryPatch patch;
            if (!collapseSingleGroupBoundaryVertex(
                    mesh, v, patch, pqMode ? &stageFaceTriples : nullptr)) {
                ++dbgCollapseFail;
                rejected.insert(v->poolId());
                continue;
            }
            if (!localSingleBoundaryPatchClosed(mesh, patch)) {
                ++dbgClosedFail;
                restoreSingleBoundaryPatch(mesh, patch);
                rejected.insert(v->poolId());
                continue;
            }
            int gA = -1, kA = 0, kB = 0;
            std::vector<MCGAL::Vertex*> ringA, ringB;
            if (patch.ringA.size() >= 2 && patch.ringB.size() >= 2 && patch.sideGidA >= 0 &&
                patch.sideGidB >= 0) {
                ringA = patch.ringA;
                ringB = patch.ringB;
                kA = static_cast<int>(ringA.size()) - 1;
                kB = static_cast<int>(ringB.size()) - 1;
                gA = patch.sideGidA;
            } else if (patch.addedFaces.size() >= 2) {
                gA = patch.addedFaces.front().gid;
                int nA = 0;
                for (const auto& f : patch.addedFaces)
                    if (f.gid == gA) ++nA;
                const int nB = static_cast<int>(patch.addedFaces.size()) - nA;
                if (nA >= 1 && nB >= 1) {
                    kA = nA + 1;
                    kB = nB + 1;
                    auto buildRing = [&](int startFace, int count) {
                        std::vector<MCGAL::Vertex*> ring;
                        const auto& f0 = patch.addedFaces[static_cast<std::size_t>(startFace)];
                        ring.push_back(f0.v[0]);
                        ring.push_back(f0.v[1]);
                        ring.push_back(f0.v[2]);
                        for (int i = 1; i < count; ++i)
                            ring.push_back(
                                patch.addedFaces[static_cast<std::size_t>(startFace + i)].v[2]);
                        return ring;
                    };
                    ringA = buildRing(0, nA);
                    ringB = buildRing(nA, nB);
                }
            }
            MCGAL::Vertex* a = ringA.empty() ? nullptr : ringA.front();
            MCGAL::Vertex* b = ringA.empty() ? nullptr : ringA.back();
            if (!a || !b || a == b || gA < 0 || kA < 1 || kB < 1) {
                restoreSingleBoundaryPatch(mesh, patch);
                rejected.insert(v->poolId());
                continue;
            }
            // Topology-driven local-fan validity: extremely large boundary fans make the
            // compact inverse ambiguous.  This is a local topology constraint (fan size),
            // not a per-round count K.  Default 0 = unlimited.
            static const int maxBoundaryFan = [] {
                const char* e = std::getenv("PMSF_SC_BOUNDARY_MAX_FAN");
                return e ? std::atoi(e) : 0;
            }();
            if (maxBoundaryFan > 0 && (kA > maxBoundaryFan || kB > maxBoundaryFan)) {
                restoreSingleBoundaryPatch(mesh, patch);
                rejected.insert(v->poolId());
                continue;
            }
            // The capture convention is hA: b -> a with face group gA (opposite
            // side gB).  Normalise a/b and both rings so this holds exactly; the
            // compact inverse and the flip-bit computation both rely on it.
            const int gBNorm = (patch.sideGidB >= 0) ? patch.sideGidB : patch.addedFaces.back().gid;
            MCGAL::Halfedge* hA = nullptr;
            for (MCGAL::Vertex* endpoint : {a, b}) {
                for (MCGAL::Halfedge* hh : pmsf::boundedRing(endpoint)) {
                    if (!hh || hh->isRemoved() || !hh->vertex() || !hh->end_vertex()) continue;
                    const bool endpointPair = (hh->vertex() == a && hh->end_vertex() == b) ||
                                              (hh->vertex() == b && hh->end_vertex() == a);
                    if (!endpointPair) continue;
                    MCGAL::Halfedge* opp2 = hh->opposite();
                    if (!opp2 || opp2->isRemoved() || !hh->face() || !opp2->face()) continue;
                    if (hh->face()->groupId() == gA && opp2->face()->groupId() == gBNorm) {
                        hA = hh;
                        break;
                    }
                }
                if (hA) break;
            }
            if (!hA) {
                restoreSingleBoundaryPatch(mesh, patch);
                rejected.insert(v->poolId());
                continue;
            }
            MCGAL::Vertex* na = hA->end_vertex();
            MCGAL::Vertex* nb = hA->vertex();
            if (na != a || nb != b) {
                if (ringA.front() == nb && ringA.back() == na) {
                    std::reverse(ringA.begin(), ringA.end());
                    std::reverse(ringB.begin(), ringB.end());
                    a = na;
                    b = nb;
                } else {
                    restoreSingleBoundaryPatch(mesh, patch);
                    rejected.insert(v->poolId());
                    continue;
                }
            }
            MCGAL::Halfedge* rawH = hA;
            auto thirdOf = [&](MCGAL::Facet* f, MCGAL::Vertex* x, MCGAL::Vertex* y) {
                MCGAL::Vertex* outv = nullptr;
                const auto fv = pmsf::sbdetail::faceVertices(f);
                for (MCGAL::Vertex* vv : fv) {
                    if (!vv || vv == x || vv == y) continue;
                    if (outv) return static_cast<MCGAL::Vertex*>(nullptr);
                    outv = vv;
                }
                return outv;
            };
            auto walkDecRing = [&](MCGAL::Halfedge* h, MCGAL::Vertex* apex, MCGAL::Vertex* other,
                                   int fanFaces, std::vector<MCGAL::Vertex*>& ring) -> bool {
                ring.clear();
                if (!h || !h->face() || fanFaces < 1) return false;
                MCGAL::Facet* f = h->face();
                MCGAL::Vertex* pth = thirdOf(f, apex, other);
                if (!pth) return false;
                std::vector<MCGAL::Vertex*> collected{pth};
                for (int step = 1; step < fanFaces; ++step) {
                    MCGAL::Halfedge* hedge = nullptr;
                    MCGAL::Halfedge* h0 = f->proxyHalfedge();
                    MCGAL::Halfedge* hh = h0;
                    int guard = 0;
                    do {
                        if (hh && hh->vertex() == apex && hh->end_vertex() == collected.back()) {
                            hedge = hh;
                            break;
                        }
                        hh = hh ? hh->next() : nullptr;
                    } while (hh && hh != h0 && ++guard < 64);
                    if (!hedge) return false;
                    MCGAL::Halfedge* opp2 = hedge->opposite();
                    if (!opp2 || opp2->isRemoved() || !opp2->face() || opp2->face()->isRemoved())
                        return false;
                    f = opp2->face();
                    MCGAL::Vertex* nxt = thirdOf(f, apex, collected.back());
                    if (!nxt || nxt == apex || nxt == other) return false;
                    collected.push_back(nxt);
                }
                std::reverse(collected.begin(), collected.end());
                ring.push_back(apex);
                ring.insert(ring.end(), collected.begin(), collected.end());
                return true;
            };
            auto cyclicMatch = [&](const pmsf::SingleBoundaryFace& ofc, MCGAL::Vertex* r,
                                   MCGAL::Vertex* x, MCGAL::Vertex* y) -> int {
                for (int st = 0; st < 3; ++st) {
                    if (ofc.v[st] == r && ofc.v[(st + 1) % 3] == x && ofc.v[(st + 2) % 3] == y)
                        return 1;
                    if (ofc.v[st] == r && ofc.v[(st + 1) % 3] == y && ofc.v[(st + 2) % 3] == x)
                        return -1;
                }
                return 0;
            };
            auto decideFlip = [&](int gid, const std::vector<MCGAL::Vertex*>& full) -> int {
                int unflipped = 0, flipped = 0;
                for (std::size_t i = 0; i + 1 < full.size(); ++i) {
                    for (const auto& ofc : patch.origFaces) {
                        if (ofc.gid != gid) continue;
                        const int mm = cyclicMatch(ofc, patch.removed, full[i], full[i + 1]);
                        if (mm == 1) ++unflipped;
                        else if (mm == -1) ++flipped;
                    }
                }
                if (v2Debug)
                    std::fprintf(stderr,
                                 "[bc-v2-flip-vote] gid=%d full=%zu unflipped=%d flipped=%d\\n",
                                 gid, full.size(), unflipped, flipped);
                return (flipped > unflipped) ? 1 : 0;
            };
            int flipA = 0, flipB = 0;
            if (rawH) {
                MCGAL::Halfedge* rawOpp = rawH->opposite();
                std::vector<MCGAL::Vertex*> ringDecA, ringDecB;
                if (walkDecRing(rawH, a, b, kA - 1, ringDecA) && rawOpp &&
                    walkDecRing(rawOpp, b, a, kB - 1, ringDecB)) {
                    std::vector<MCGAL::Vertex*> fullA = ringDecA;
                    fullA.push_back(b);
                    std::vector<MCGAL::Vertex*> fullB = ringDecB;
                    fullB.push_back(a);
                    const int gB = (patch.sideGidB >= 0) ? patch.sideGidB
                                                         : patch.addedFaces.back().gid;
                    flipA = decideFlip(gA, fullA);
                    flipB = decideFlip(gB, fullB);
                }
            }
            BoundaryV2Op op;
            op.group = gA;
            op.kA = kA;
            op.kB = kB;
            op.rq = rq;
            if (rawH && rawH->opposite() && rawH->opposite()->face() &&
                !rawH->opposite()->face()->isRemoved())
                op.gB = rawH->opposite()->face()->groupId();
            if (rawH) {
                if (rawH->face() && !rawH->face()->isRemoved())
                    op.thirdA = thirdOf(rawH->face(), a, b);
                if (rawH->opposite() && rawH->opposite()->face() &&
                    !rawH->opposite()->face()->isRemoved())
                    op.thirdB = thirdOf(rawH->opposite()->face(), b, a);
            }
            op.a = a;
            op.b = b;
            op.orient = flipA | (flipB << 1);
            if (patch.removedVertexRank >= 0 && patch.ringA.size() >= 2 &&
                patch.ringB.size() >= 2) {
                std::vector<int> ranks;
                bool ranksOk = true;
                auto addFaceRank = [&](MCGAL::Vertex* x, MCGAL::Vertex* y) {
                    for (const auto& rec : patch.origFaces) {
                        const auto has = [&](MCGAL::Vertex* z) {
                            return z && (rec.v[0] == z || rec.v[1] == z || rec.v[2] == z);
                        };
                        if (has(patch.removed) && has(x) && has(y)) {
                            if (rec.rank < 0 || !ranksOk) { ranksOk = false; return; }
                            ranks.push_back(rec.rank);
                            return;
                        }
                    }
                    ranksOk = false;
                };
                for (std::size_t i = 0; i + 1 < patch.ringA.size() && ranksOk; ++i)
                    addFaceRank(patch.ringA[i], patch.ringA[i + 1]);
                for (std::size_t i = 0; i + 1 < patch.ringB.size() && ranksOk; ++i)
                    addFaceRank(patch.ringB[i], patch.ringB[i + 1]);
                if (ranksOk && static_cast<int>(ranks.size()) == kA + kB) {
                    op.vRank = patch.removedVertexRank;
                    op.faceRanks = std::move(ranks);
                }
            }
            if (v2Debug) {
                auto qs = [&](MCGAL::Vertex* v) {
                    const auto q = mesh->floatPosToInt(v->point());
                    return std::array<int, 3>{q.x(), q.y(), q.z()};
                };
                std::fprintf(stderr, "[bc-v2-opinfo] round=%d gA=%d kA=%d kB=%d orient=%d\n",
                             roundIndex, gA, kA, kB, op.orient);
                std::fprintf(stderr, "[bc-v2-opinfo] ringA:");
                for (auto* v : ringA) { const auto q = qs(v); std::fprintf(stderr, " (%d,%d,%d)", q[0], q[1], q[2]); }
                std::fprintf(stderr, "\n[bc-v2-opinfo] ringB:");
                for (auto* v : ringB) { const auto q = qs(v); std::fprintf(stderr, " (%d,%d,%d)", q[0], q[1], q[2]); }
                std::fprintf(stderr, "\n[bc-v2-opinfo] added:");
                for (const auto& f : patch.addedFaces) std::fprintf(stderr, " [gid=%d]", f.gid);
                std::fprintf(stderr, "\n[bc-v2-opinfo] orig gidcounts:");
                { std::map<int,int> gc; for (const auto& f : patch.origFaces) ++gc[f.gid];
                  for (auto& kv : gc) std::fprintf(stderr, " gid%d=%d", kv.first, kv.second); }
                std::fprintf(stderr, " (origFaces=%zu)\n", patch.origFaces.size());
                for (const auto& ofc : patch.origFaces) {
                    std::fprintf(stderr, "[bc-v2-orig] gid=%d", ofc.gid);
                    for (int k = 0; k < 3; ++k) {
                        if (!ofc.v[k]) continue;
                        const auto q2 = mesh->floatPosToInt(ofc.v[k]->point());
                        std::fprintf(stderr, " (%d,%d,%d)", q2.x(), q2.y(), q2.z());
                    }
                    std::fprintf(stderr, "\n");
                }
            }
            if (patch.removed) {
                op.removedQ = qOf(patch.removed);
                op.removedVertex = patch.removed;
            }
            if (std::getenv("PMSF_BND_V3_EXPLICIT")) {
                op.expOrig.clear();
                op.expOrigGid.clear();
                for (const auto& ofc : patch.origFaces) {
                    op.expOrig.push_back({ofc.v[0], ofc.v[1], ofc.v[2]});
                    op.expOrigGid.push_back(ofc.gid);
                }
                op.expAdd.clear();
                op.expAddGid.clear();
                for (const auto& nf : patch.addedFaces) {
                    op.expAdd.push_back({nf.v[0], nf.v[1], nf.v[2]});
                    op.expAddGid.push_back(nf.gid);
                }
            }
            out.ops.push_back(op);
            // S-ID binding is assigned AFTER the final bit-index sort below, because
            // out.ops is reordered by boundary-edge bit index after capture.
            // Independence locking: no later op in this round may touch the
            // merged edge or its neighbourhood, so the inverse splits are
            // independent and the post-stage enumeration stays valid.
            for (MCGAL::Vertex* x : {a, b}) {
                locked.insert(x);
                for (MCGAL::Halfedge* h : pmsf::boundedRing(x))
                    if (h && h->end_vertex() && !h->end_vertex()->isRemoved())
                        locked.insert(h->end_vertex());
            }
            for (MCGAL::Vertex* x : ringB)
                if (x) locked.insert(x);
            ++applied;
            ok = true;
            // Incremental candidate maintenance: drop the removed vertex and add the
            // vertices of the newly created patch faces (new boundary vertices appear
            // there).  A periodic full rescan above corrects any drift.
            if (patch.removed) cachedCands.erase(patch.removed);
            for (const auto& nf : patch.addedFaces)
                for (int kk = 0; kk < 3; ++kk)
                    if (nf.v[kk] && !nf.v[kk]->isRemoved()) cachedCands.insert(nf.v[kk]);
            if (pqMode) {
                // Re-score only the locally affected vertices instead of the whole
                // candidate set; stale heap entries are discarded by score mismatch.
                std::set<MCGAL::Vertex*> affected;
                if (patch.removed) affected.insert(patch.removed);
                for (const auto& nf : patch.addedFaces)
                    for (int kk = 0; kk < 3; ++kk)
                        if (nf.v[kk]) affected.insert(nf.v[kk]);
                for (MCGAL::Vertex* vv : affected) {
                    if (!vv || vv->isRemoved()) {
                        pqScore.erase(vv);
                        continue;
                    }
                    const double sc = scoreOf(vv);
                    pqScore[vv] = sc;
                    pq.push(KfEntry{sc, vv->poolId(), vv});
                }
            }
            break;
        }
        if (!ok) break;
    }
    if (out.ops.empty()) return applied;
    std::vector<MCGAL::Halfedge*> edges;
    std::vector<int> anchorOffsets;
    if (!scV2EnumerateAnchors(mesh.get(), anchors, edges, &anchorOffsets)) {
        std::fprintf(stderr, "[bc-v2-stage] round=%d anchor enumeration failed\n", roundIndex);
        return -1;
    }
    out.edgeCount = static_cast<int>(edges.size());
    if (v2Debug) {
        std::fprintf(stderr, "[bc-v2-stage-debug] round=%d postEdges=%d ops=%zu\n", roundIndex,
                     out.edgeCount, out.ops.size());
        for (const auto& ag : anchors) {
            std::vector<MCGAL::Halfedge*> dbgLoop;
            const bool wok = walkLoopFromFixedAnchor(mesh.get(), ag.second, ag.first, dbgLoop);
            const auto q = (ag.second && !ag.second->isRemoved())
                               ? mesh->floatPosToInt(ag.second->point())
                               : MCGAL::PointInt(0, 0, 0);
            std::fprintf(stderr,
                         "[bc-v2-anchor] enc round=%d g=%d q=(%d,%d,%d) loop=%zu walk=%d\n",
                         roundIndex, ag.first, q.x(), q.y(), q.z(), dbgLoop.size(), wok ? 1 : 0);
        }
        char dbgPath[512];
        std::snprintf(dbgPath, sizeof(dbgPath), "/tmp/bc_v2_enc_round_%03d.off", roundIndex);
        mesh->dumpto_oldtype(dbgPath);
    }
    out.bits.assign(edges.size(), 0);
    std::vector<std::pair<int, BoundaryV2Op>> withIdx;
    withIdx.reserve(out.ops.size());
    for (const auto& op : out.ops) {
        int bitIndex = -1;
        for (int i = 0; i < static_cast<int>(edges.size()); ++i) {
            MCGAL::Halfedge* e = edges[static_cast<std::size_t>(i)];
            if (!e || !e->vertex() || !e->end_vertex() || !e->face()) continue;
            // Address the capture side of the seam explicitly: the enumeration
            // contains group-gA boundary halfedges (face group == gA), and the raw
            // capture orientation is b -> a.
            if (e->face()->groupId() != op.group) continue;
            if (e->vertex() == op.b && e->end_vertex() == op.a) {
                bitIndex = i;
                break;
            }
        }
        if (bitIndex < 0) {
            std::fprintf(stderr,
                         "[bc-v2-stage] round=%d merged edge not found on capture side gA=%d\n",
                         roundIndex, op.group);
            return -1;
        }
        BoundaryV2Op stored = op;
        if (v2Debug) {
            std::fprintf(stderr, "[bc-v2-op] round=%d bit=%d gA=%d kA=%d kB=%d\\n", roundIndex,
                         bitIndex, op.group, op.kA, op.kB);
        }
        withIdx.emplace_back(bitIndex, stored);
    }
    std::sort(withIdx.begin(), withIdx.end(),
              [](const auto& x, const auto& y) { return x.first < y.first; });
    out.ops.clear();
    for (int i = 0; i < static_cast<int>(edges.size()); ++i) {
        for (const auto& w : withIdx) {
            if (w.first != i) continue;
            out.bits[static_cast<std::size_t>(i)] = 1;
            out.ops.push_back(w.second);
        }
    }
    if (pmsf::bndid::enc().on) {
        for (int i = 0; i < static_cast<int>(out.ops.size()); ++i)
            pmsf::bndid::encNoteStageRemoved(out.ops[static_cast<std::size_t>(i)].removedVertex,
                                             roundIndex, i);
    }
    return applied;
}

/// Roll back a partially-applied v2 stage round.  When the post-stage anchor
/// enumeration fails (deep/multi-round staging on coarse meshes), the mesh has
/// already been mutated by the ops recorded in `round`; undo them in reverse
/// order through the same compact-patch inverse the decoder uses, so the round
/// can simply be left unstaged instead of aborting compression.
inline bool scV2RollbackRound(const std::shared_ptr<MCGAL::Mesh>& mesh,
                              const BoundaryV2Round& round) {
    if (!mesh) return false;
    for (auto it = round.ops.rbegin(); it != round.ops.rend(); ++it) {
        const BoundaryV2Op& op = *it;
        if (!op.a || !op.b || op.a->isRemoved() || op.b->isRemoved()) return false;
        MCGAL::Halfedge* raw = nullptr;
        for (MCGAL::Vertex* endpoint : {op.a, op.b}) {
            if (!endpoint || endpoint->isRemoved()) continue;
            for (MCGAL::Halfedge* h : pmsf::boundedRing(endpoint)) {
                if (!h || h->isRemoved() || !h->vertex() || !h->end_vertex()) continue;
                if (h->vertex() != op.b || h->end_vertex() != op.a) continue;
                if (!h->face() || h->face()->isRemoved()) continue;
                if (h->face()->groupId() != op.group) continue;
                raw = h;
                break;
            }
            if (raw) break;
        }
        if (!raw) {
                        return false;
        }
        const int recFlipA = op.orient & 1;
        const int recFlipB = (op.orient >> 1) & 1;
        const int flipCodes[4][2] = {{recFlipA, recFlipB},
                                     {recFlipA ^ 1, recFlipB},
                                     {recFlipA, recFlipB ^ 1},
                                     {recFlipA ^ 1, recFlipB ^ 1}};
        MCGAL::Halfedge* other = raw->opposite();
        bool applied = false;
        for (int ci = 0; ci < 2 && !applied; ++ci) {
            MCGAL::Halfedge* hh = (ci == 0) ? raw : other;
            if (!hh || hh->isRemoved() || !hh->vertex() || !hh->end_vertex()) continue;
            const int ka = (ci == 0) ? op.kA : op.kB;
            const int kb = (ci == 0) ? op.kB : op.kA;
            const auto qa = mesh->floatPosToInt(hh->end_vertex()->point());
            const auto qb = mesh->floatPosToInt(hh->vertex()->point());
            for (const auto& fc : flipCodes) {
                std::vector<int> compact;
                compact.reserve(19);
                compact.push_back(18);
                compact.push_back(qa.x());
                compact.push_back(qa.y());
                compact.push_back(qa.z());
                compact.push_back(qb.x());
                compact.push_back(qb.y());
                compact.push_back(qb.z());
                compact.push_back(op.group);
                compact.push_back(ka);
                compact.push_back(kb);
                compact.push_back(op.rq[0]);
                compact.push_back(op.rq[1]);
                compact.push_back(op.rq[2]);
                for (int k = 0; k < 6; ++k) compact.push_back(-1);
                g_scForcedCandidate = hh;
                g_scForceFlipA = fc[0];
                g_scForceFlipB = fc[1];
                std::size_t cur = 0;
                const bool ok = applySingleBoundaryCompactPatchInverse(mesh, compact, cur);
                g_scForcedCandidate = nullptr;
                g_scForceFlipA = -1;
                g_scForceFlipB = -1;
                if (ok && cur == compact.size()) {
                    applied = true;
                    break;
                }
            }
        }
        if (!applied) {
                        return false;
        }
        pmsf::repairHalfedgePairing(mesh);
        pmsf::rebuildVertexHalfedgeRings(mesh);
    }
    return true;
}


inline std::vector<int> captureAnchorPatch(MCGAL::Mesh* mesh,
                                           const std::vector<ScAnchorDef>& anchors,
                                           const SingleBoundaryPatch& patch) {
    if (!mesh || !patch.valid || !patch.removed || patch.addedFaces.size() < 2) return {};
    const int gA = patch.addedFaces.front().gid;
    int nA = 0;
    for (const auto& f : patch.addedFaces) if (f.gid == gA) ++nA;
    const int nB = static_cast<int>(patch.addedFaces.size()) - nA;
    if (nA < 1 || nB < 1) return {};
    const int kA = nA + 1;
    const int kB = nB + 1;
    auto buildRing = [&](int startFace, int count) {
        std::vector<MCGAL::Vertex*> ring;
        const auto& f0 = patch.addedFaces[static_cast<std::size_t>(startFace)];
        ring.push_back(f0.v[0]);
        ring.push_back(f0.v[1]);
        ring.push_back(f0.v[2]);
        for (int i = 1; i < count; ++i)
            ring.push_back(patch.addedFaces[static_cast<std::size_t>(startFace + i)].v[2]);
        return ring;
    };
    const auto ringA = buildRing(0, nA);
    const auto ringB = buildRing(nA, nB);
    if (static_cast<int>(ringA.size()) != kA + 1 || static_cast<int>(ringB.size()) != kB + 1)
        return {};
    if (ringB.front() != ringA.back() || ringB.back() != ringA.front()) return {};
    MCGAL::Vertex* a = ringA.front();
    MCGAL::Vertex* b = ringA.back();
    const int gB = patch.addedFaces.back().gid;
    // A-side boundary halfedge (b -> a) with face group gA
    MCGAL::Halfedge* hA = nullptr;
    for (MCGAL::Vertex* endpoint : {a, b}) {
        for (MCGAL::Halfedge* h : pmsf::boundedRing(endpoint)) {
            if (!h || h->isRemoved() || !h->vertex() || !h->end_vertex()) continue;
            if (h->vertex() != b || h->end_vertex() != a) continue;
            MCGAL::Halfedge* opp = h->opposite();
            if (!opp || opp->isRemoved() || !opp->face()) continue;
            if (h->face()->groupId() == gA && opp->face()->groupId() == gB) {
                hA = h;
                break;
            }
        }
        if (hA) break;
    }
    if (!hA) return {};
    for (std::size_t ai = 0; ai < anchors.size(); ++ai) {
        const ScAnchorDef& def = anchors[ai];
        if (def.group != gA) continue;
        std::vector<MCGAL::Halfedge*> loop;
        if (!walkGroupLoop(mesh, def.v, def.group, loop)) continue;
        for (std::size_t ei = 0; ei < loop.size(); ++ei) {
            if (loop[ei] != hA) continue;
            const auto rq = scloop::qOf(mesh, patch.removed);
            MCGAL::Point orig = patch.removed->point();
            MCGAL::Point basePos = mesh->intPosToFloat(
                MCGAL::PointInt(rq[0], rq[1], rq[2]));
            const float fineStep = mesh->f_quantStep / 1024.0f;
            int fine[3] = {0, 0, 0};
            for (int k = 0; k < 3; ++k) {
                const float d = (orig[k] - basePos[k]) / fineStep;
                int r = static_cast<int>(std::lround(d));
                r = std::clamp(r, -512, 511);
                fine[k] = r;
            }
            std::vector<int> out;
            out.push_back(11);
            out.push_back(static_cast<int>(ai));
            out.push_back(static_cast<int>(ei));
            out.push_back(gA);
            out.push_back(kA);
            out.push_back(kB);
            out.push_back(rq[0]);
            out.push_back(rq[1]);
            out.push_back(rq[2]);
            out.push_back(fine[0]);
            out.push_back(fine[1]);
            out.push_back(fine[2]);
            return out;
        }
    }
    return {};
}

/// In-memory inverse: anchor group-boundary traversal then compact restore.
inline bool applyAnchorPatchInverseInMemory(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                            const std::vector<ScAnchorDef>& anchors,
                                            const std::vector<int>& data,
                                            std::size_t& cursor) {
    if (!mesh || cursor >= data.size()) return false;
    const int len = data[cursor++];
    if (len != 11 || cursor + 11 > data.size()) return false;
    const int anchorId = data[cursor++];
    const int offset = data[cursor++];
    const int gA = data[cursor++];
    const int kA = data[cursor++];
    const int kB = data[cursor++];
    const MCGAL::PointInt rq{data[cursor], data[cursor + 1], data[cursor + 2]};
    cursor += 3;
    const int fineX = data[cursor++];
    const int fineY = data[cursor++];
    const int fineZ = data[cursor++];
    if (anchorId < 0 || anchorId >= static_cast<int>(anchors.size())) return false;
    const ScAnchorDef& def = anchors[static_cast<std::size_t>(anchorId)];
    std::vector<MCGAL::Halfedge*> loop;
    if (!walkGroupLoop(mesh.get(), def.v, def.group, loop)) {
        if (std::getenv("PMSF_SC_DEBUG"))
            std::fprintf(stderr, "[sc-anchor-inv] walk fail anchorId=%d group=%d\n", anchorId,
                         def.group);
        return false;
    }
    if (offset < 0 || offset >= static_cast<int>(loop.size())) {
        if (std::getenv("PMSF_SC_DEBUG"))
            std::fprintf(stderr, "[sc-anchor-inv] offset %d >= loop %zu\n", offset, loop.size());
        return false;
    }
    MCGAL::Halfedge* hA = loop[static_cast<std::size_t>(offset)];
    if (!hA || !hA->face() || hA->face()->groupId() != gA || !hA->opposite() ||
        !hA->opposite()->face()) {
        if (std::getenv("PMSF_SC_DEBUG"))
            std::fprintf(stderr, "[sc-anchor-inv] hA mismatch gA=%d\n", gA);
        return false;
    }
    MCGAL::Vertex* a = hA->end_vertex();
    MCGAL::Vertex* b = hA->vertex();
    if (!a || !b) return false;
    auto third = [](MCGAL::Halfedge* h0, MCGAL::Vertex* x, MCGAL::Vertex* y) -> MCGAL::Vertex* {
        if (!h0) return nullptr;
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && h->vertex() && h->vertex() != x && h->vertex() != y) return h->vertex();
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
        return nullptr;
    };
    MCGAL::Vertex* pt = third(hA->face()->proxyHalfedge(), a, b);
    MCGAL::Vertex* rt = third(hA->opposite()->face()->proxyHalfedge(), b, a);
    if (!pt || !rt) return false;
    const auto aq = scloop::qOf(mesh.get(), a);
    const auto bq = scloop::qOf(mesh.get(), b);
    const auto ptq = scloop::qOf(mesh.get(), pt);
    const auto rtq = scloop::qOf(mesh.get(), rt);
    std::vector<int> compact;
    compact.push_back(18);
    compact.push_back(aq[0]); compact.push_back(aq[1]); compact.push_back(aq[2]);
    compact.push_back(bq[0]); compact.push_back(bq[1]); compact.push_back(bq[2]);
    compact.push_back(gA); compact.push_back(kA); compact.push_back(kB);
    compact.push_back(rq.x()); compact.push_back(rq.y()); compact.push_back(rq.z());
    compact.push_back(ptq[0]); compact.push_back(ptq[1]); compact.push_back(ptq[2]);
    compact.push_back(rtq[0]); compact.push_back(rtq[1]); compact.push_back(rtq[2]);
    std::size_t c = 0;
    g_scOverridePosActive = true;
    g_scOverridePosQ = rq;
    g_scOverridePos = mesh->intPosToFloat(rq);
    const float fineStep = mesh->f_quantStep / 1024.0f;
    g_scOverridePos[0] += static_cast<float>(fineX) * fineStep;
    g_scOverridePos[1] += static_cast<float>(fineY) * fineStep;
    g_scOverridePos[2] += static_cast<float>(fineZ) * fineStep;
    const bool ok = applySingleBoundaryCompactPatchInverse(mesh, compact, c);
    g_scForcedCandidate = nullptr;
    g_scOverridePosActive = false;
    if (!ok) return false;
    return c == compact.size();
}

/// Direct anchor inverse: locate the boundary edge through group traversal and
/// forward to the compact restore path with a synthetic patch built from the
/// CURRENT state (so no encoder-side position identity is needed).
inline bool applyAnchorPatchInverseDirect(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                          const std::vector<ScAnchorDef>& anchors,
                                          const std::vector<int>& data,
                                          std::size_t& cursor) {
    if (!mesh || cursor >= data.size()) return false;
    const int len = data[cursor++];
    if (len != 11 || cursor + 11 > data.size()) return false;
    const int anchorId = data[cursor++];
    const int offset = data[cursor++];
    const int gA = data[cursor++];
    const int kA = data[cursor++];
    const int kB = data[cursor++];
    const MCGAL::PointInt rq{data[cursor], data[cursor + 1], data[cursor + 2]};
    cursor += 3;
    const int fineX = data[cursor++];
    const int fineY = data[cursor++];
    const int fineZ = data[cursor++];
    if (anchorId < 0 || anchorId >= static_cast<int>(anchors.size())) return false;
    const ScAnchorDef& def = anchors[static_cast<std::size_t>(anchorId)];
    std::vector<MCGAL::Halfedge*> loop;
    if (!walkGroupLoop(mesh.get(), def.v, def.group, loop)) {
        if (std::getenv("PMSF_SC_DEBUG"))
            std::fprintf(stderr, "[sc-anchor-inv] walk fail anchorId=%d group=%d\n", anchorId,
                         def.group);
        return false;
    }
    if (offset < 0 || offset >= static_cast<int>(loop.size())) {
        if (std::getenv("PMSF_SC_DEBUG"))
            std::fprintf(stderr, "[sc-anchor-inv] offset %d >= loop %zu\n", offset, loop.size());
        return false;
    }
    MCGAL::Halfedge* hA = loop[static_cast<std::size_t>(offset)];
    if (!hA || !hA->face() || hA->face()->groupId() != gA || !hA->opposite() ||
        !hA->opposite()->face()) {
        if (std::getenv("PMSF_SC_DEBUG"))
            std::fprintf(stderr, "[sc-anchor-inv] hA mismatch gA=%d\n", gA);
        return false;
    }
    MCGAL::Vertex* a = hA->end_vertex();
    MCGAL::Vertex* b = hA->vertex();
    if (!a || !b) return false;
    auto third = [](MCGAL::Halfedge* h0, MCGAL::Vertex* x, MCGAL::Vertex* y) -> MCGAL::Vertex* {
        if (!h0) return nullptr;
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && h->vertex() && h->vertex() != x && h->vertex() != y) return h->vertex();
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
        return nullptr;
    };
    MCGAL::Vertex* pt = third(hA->face()->proxyHalfedge(), a, b);
    MCGAL::Vertex* rt = third(hA->opposite()->face()->proxyHalfedge(), b, a);
    if (!pt || !rt) return false;
    const auto aq = scloop::qOf(mesh.get(), a);
    const auto bq = scloop::qOf(mesh.get(), b);
    const auto ptq = scloop::qOf(mesh.get(), pt);
    const auto rtq = scloop::qOf(mesh.get(), rt);
    std::vector<int> compact;
    compact.push_back(18);
    compact.push_back(aq[0]); compact.push_back(aq[1]); compact.push_back(aq[2]);
    compact.push_back(bq[0]); compact.push_back(bq[1]); compact.push_back(bq[2]);
    compact.push_back(gA); compact.push_back(kA); compact.push_back(kB);
    compact.push_back(rq.x()); compact.push_back(rq.y()); compact.push_back(rq.z());
    compact.push_back(ptq[0]); compact.push_back(ptq[1]); compact.push_back(ptq[2]);
    compact.push_back(rtq[0]); compact.push_back(rtq[1]); compact.push_back(rtq[2]);
    std::size_t c = 0;
    g_scOverridePosActive = true;
    g_scOverridePosQ = rq;
    g_scOverridePos = mesh->intPosToFloat(rq);
    const float fineStep = mesh->f_quantStep / 1024.0f;
    g_scOverridePos[0] += static_cast<float>(fineX) * fineStep;
    g_scOverridePos[1] += static_cast<float>(fineY) * fineStep;
    g_scOverridePos[2] += static_cast<float>(fineZ) * fineStep;
    const bool ok = applySingleBoundaryCompactPatchInverse(mesh, compact, c);
    g_scForcedCandidate = nullptr;
    g_scOverridePosActive = false;
    if (!ok) return false;
    return c == compact.size();
}

/// Resolve anchors to base-mesh vertex indices (base vertices are serialized in
/// live mesh->vertices() order) and back.

/// Base writer order: seed->vertex (vid 0), seed->end_vertex (vid 1), then the
/// remaining live vertices in mesh->vertices() order.
inline std::vector<int> anchorBaseIndicesWithSeed(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                                 const std::vector<ScAnchorDef>& anchors,
                                                 MCGAL::Vertex* seedV, MCGAL::Vertex* seedE) {
    std::vector<int> out(anchors.size(), -1);
    if (!mesh) return out;
    std::vector<MCGAL::Vertex*> order;
    order.reserve(mesh->vertices().size());
    if (seedV) order.push_back(seedV);
    if (seedE && seedE != seedV) order.push_back(seedE);
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        if (v == seedV || v == seedE) continue;
        order.push_back(v);
    }
    std::unordered_map<MCGAL::Vertex*, int> idx;
    for (std::size_t i = 0; i < order.size(); ++i) idx[order[i]] = static_cast<int>(i);
    for (std::size_t k = 0; k < anchors.size(); ++k) {
        auto it = idx.find(anchors[k].v);
        if (it != idx.end()) out[k] = it->second;
    }
    return out;
}

inline std::vector<ScAnchorDef> resolveAnchorsFromBase(
    const std::shared_ptr<MCGAL::Mesh>& mesh,
    const std::vector<std::pair<int, int>>& recs,
    const std::vector<MCGAL::Vertex*>* baseOrder = nullptr) {
    std::vector<ScAnchorDef> out;
    if (!mesh) return out;
    std::vector<MCGAL::Vertex*> live;
    if (baseOrder && !baseOrder->empty()) {
        live = *baseOrder;
    } else {
        for (MCGAL::Vertex* v : mesh->vertices()) if (v && !v->isRemoved()) live.push_back(v);
    }
    for (const auto& rec : recs) {
        ScAnchorDef d;
        if (rec.first >= 0 && rec.first < static_cast<int>(live.size())) {
            d.v = live[static_cast<std::size_t>(rec.first)];
            d.group = rec.second;
            out.push_back(d);
        }
    }
    return out;
}

/// Encoder stage for the anchor codec: apply up to maxOps boundary collapses,
/// appending anchor patches to roundData.
inline int runSingleCopyBoundaryEncoderStageAnchor(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                                  int roundIndex, int maxOps,
                                                  const std::set<MCGAL::Vertex*>& protectedVerts,
                                                  const std::vector<ScAnchorDef>& anchors,
                                                  std::vector<int>& roundData) {
    if (!mesh || maxOps <= 0) return 0;
    // Merge-by-position safety: a recreated boundary vertex whose quantized
    // position collides with another live vertex can be merged during decode
    // (seam-merge ambiguity).  Skip such candidates.
    std::unordered_map<long long, int> posCount;
    posCount.reserve(mesh->vertices().size() * 2 + 1);
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        const MCGAL::PointInt q = mesh->floatPosToInt(v->point());
        const long long key = static_cast<long long>(q.x()) * 1000003LL * 1000003LL +
                              static_cast<long long>(q.y()) * 1000003LL + q.z();
        ++posCount[key];
    }
    auto uniquePos = [&](MCGAL::Vertex* v) {
        if (!v) return false;
        const MCGAL::PointInt q = mesh->floatPosToInt(v->point());
        const long long key = static_cast<long long>(q.x()) * 1000003LL * 1000003LL +
                              static_cast<long long>(q.y()) * 1000003LL + q.z();
        auto it = posCount.find(key);
        return it != posCount.end() && it->second == 1;
    };
    int applied = 0;
    int rejAlready = 0, rejProtected = 0, rejOpenVertex = 0, rejCollapse = 0,
        rejClosed = 0, rejStrict = 0, rejOpenEdge = 0, rejCapture = 0, rejCornerVerify = 0;
    const bool cornerVerify = std::getenv("PMSF_SC_CORNER_VERIFY") != nullptr;
    if (cornerVerify) scStageOpPreHashes().clear();
    std::set<int> rejected;
    for (int iter = 0; iter < maxOps; ++iter) {
        std::set<MCGAL::Vertex*> cands;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
            MCGAL::Halfedge* h0 = f->proxyHalfedge();
            MCGAL::Halfedge* h = h0;
            int guard = 0;
            do {
                MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
                if (h && h->vertex() && h->next() && h->next()->vertex() && opp &&
                    !opp->isRemoved() && opp->face() && !opp->face()->isRemoved() &&
                    opp->face()->groupId() != f->groupId()) {
                    if (!h->vertex()->isRemoved()) cands.insert(h->vertex());
                    if (!h->next()->vertex()->isRemoved()) cands.insert(h->next()->vertex());
                }
                h = h ? h->next() : nullptr;
            } while (h && h != h0 && ++guard < 64);
        }
        if (cands.empty()) break;
        std::vector<MCGAL::Vertex*> ordered(cands.begin(), cands.end());
        std::vector<std::vector<long long>> orderKeys;
        orderKeys.reserve(ordered.size());
        for (MCGAL::Vertex* v : ordered) orderKeys.push_back(scVertexOrderKey(mesh.get(), v));
        std::vector<std::size_t> orderIdx(ordered.size());
        for (std::size_t i = 0; i < orderIdx.size(); ++i) orderIdx[i] = i;
        std::sort(orderIdx.begin(), orderIdx.end(), [&](std::size_t ia, std::size_t ib) {
            const auto& ka = orderKeys[ia];
            const auto& kb = orderKeys[ib];
            if (ka != kb) return ka < kb;
            return ia < ib;
        });
        std::vector<MCGAL::Vertex*> orderedSorted;
        orderedSorted.reserve(ordered.size());
        for (std::size_t i : orderIdx) orderedSorted.push_back(ordered[i]);
        ordered.swap(orderedSorted);
        bool ok = false;
        for (MCGAL::Vertex* v : ordered) {
            if (!v || rejected.count(v->poolId()) || protectedVerts.count(v) || !uniquePos(v))
                continue;
            SingleBoundaryPatch patch;
            if (!collapseSingleGroupBoundaryVertex(mesh, v, patch)) {
                rejected.insert(v->poolId());
                continue;
            }
            if (!localSingleBoundaryPatchClosed(mesh, patch)) {
                restoreSingleBoundaryPatch(mesh, patch);
                rejected.insert(v->poolId());
                continue;
            }
            std::vector<int> data = captureAnchorPatch(mesh.get(), anchors, patch);
            if (data.empty()) {
                restoreSingleBoundaryPatch(mesh, patch);
                rejected.insert(v->poolId());
                continue;
            }
            roundData.insert(roundData.end(), data.begin(), data.end());
            ++applied;
            ok = true;
            break;
        }
        if (!ok) break;
    }
    return applied;
}

/// In-memory closed-loop test: collect per-group anchors, apply boundary
/// collapses (anchors skipped), undo in reverse via anchor traversal.
inline int runAnchorRoundTripTest(std::shared_ptr<MCGAL::Mesh> mesh, int maxOps) {
    if (!mesh) return 1;
    const auto anchors = collectGroupAnchors(mesh.get());
    std::set<MCGAL::Vertex*> anchorSet;
    for (const auto& d : anchors) if (d.v) anchorSet.insert(d.v);
    const std::uint64_t h0 = scPositionalHash(mesh.get());
    const std::uint64_t o0 = scOrientedHash(mesh.get());
    auto liveV = [&]() {
        int n = 0;
        for (MCGAL::Vertex* v : mesh->vertices()) if (v && !v->isRemoved()) ++n;
        return n;
    };
    const int V0 = liveV();
    const int F0 = mesh->size_of_facets();
    std::vector<std::vector<int>> patches;
    std::set<int> rejected;
    int applied = 0;
    for (int iter = 0; iter < maxOps; ++iter) {
        std::set<MCGAL::Vertex*> cands;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
            MCGAL::Halfedge* h0h = f->proxyHalfedge();
            MCGAL::Halfedge* h = h0h;
            int guard = 0;
            do {
                MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
                if (h && h->vertex() && h->next() && h->next()->vertex() && opp &&
                    !opp->isRemoved() && opp->face() && !opp->face()->isRemoved() &&
                    opp->face()->groupId() != f->groupId()) {
                    if (!h->vertex()->isRemoved()) cands.insert(h->vertex());
                    if (!h->next()->vertex()->isRemoved()) cands.insert(h->next()->vertex());
                }
                h = h ? h->next() : nullptr;
            } while (h && h != h0h && ++guard < 64);
        }
        if (cands.empty()) break;
        std::vector<MCGAL::Vertex*> ordered(cands.begin(), cands.end());
        std::vector<std::vector<long long>> orderKeys;
        orderKeys.reserve(ordered.size());
        for (MCGAL::Vertex* v : ordered) orderKeys.push_back(scVertexOrderKey(mesh.get(), v));
        std::vector<std::size_t> orderIdx(ordered.size());
        for (std::size_t i = 0; i < orderIdx.size(); ++i) orderIdx[i] = i;
        std::sort(orderIdx.begin(), orderIdx.end(), [&](std::size_t ia, std::size_t ib) {
            const auto& ka = orderKeys[ia];
            const auto& kb = orderKeys[ib];
            if (ka != kb) return ka < kb;
            return ia < ib;
        });
        std::vector<MCGAL::Vertex*> orderedSorted;
        orderedSorted.reserve(ordered.size());
        for (std::size_t i : orderIdx) orderedSorted.push_back(ordered[i]);
        ordered.swap(orderedSorted);
        bool ok = false;
        for (MCGAL::Vertex* v : ordered) {
            if (!v || rejected.count(v->poolId()) || anchorSet.count(v)) continue;
            SingleBoundaryPatch patch;
            if (!collapseSingleGroupBoundaryVertex(mesh, v, patch)) {
                rejected.insert(v->poolId());
                continue;
            }
            if (!localSingleBoundaryPatchClosed(mesh, patch)) {
                restoreSingleBoundaryPatch(mesh, patch);
                rejected.insert(v->poolId());
                continue;
            }
            std::vector<int> data = captureAnchorPatch(mesh.get(), anchors, patch);
            if (data.empty()) {
                restoreSingleBoundaryPatch(mesh, patch);
                rejected.insert(v->poolId());
                continue;
            }
            patches.push_back(std::move(data));
            ++applied;
            ok = true;
            break;
        }
        if (!ok) break;
    }
    bool undoOk = true;
    for (auto it = patches.rbegin(); it != patches.rend(); ++it) {
        std::size_t cursor = 0;
        if (!applyAnchorPatchInverseInMemory(mesh, anchors, *it, cursor) ||
            cursor != it->size()) {
            undoOk = false;
            break;
        }
    }
    const bool exact = undoOk && scPositionalHash(mesh.get()) == h0 &&
                       scOrientedHash(mesh.get()) == o0;
    std::fprintf(stderr,
                 "[sc-anchor-test] anchors=%zu applied=%d V %d->%d->%d F %d->%d->%d "
                 "undoOk=%d exact=%d\n",
                 anchors.size(), applied, V0, liveV(), liveV(), F0, mesh->size_of_facets(),
                 mesh->size_of_facets(), undoOk ? 1 : 0, exact ? 1 : 0);
    return exact ? 0 : 1;
}

/// Canonical face order + proxy rotation, used after encoder boundary stage and
/// after decoder inverse so the next BFS (collectRound / decodeOneRound) sees
/// the same mesh ordering on both sides.
inline void canonicalizeBoundaryMeshOrder(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!mesh) return;
    struct Entry {
        std::array<long long, 10> key;
        MCGAL::Facet* f = nullptr;
        MCGAL::Halfedge* proxy = nullptr;
    };
    std::vector<Entry> entries;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* hs[3] = {nullptr, nullptr, nullptr};
        std::array<int, 3> q[3];
        MCGAL::Halfedge* h = h0;
        int n = 0;
        int guard = 0;
        const int maxQ = mesh->i_nbQuantBits > 0 ? ((1 << mesh->i_nbQuantBits) - 1) : (1 << 30);
        auto clampQ = [&](int x) { return std::clamp(x, 0, maxQ); };
        do {
            if (h && h->vertex() && n < 3) {
                hs[n] = h;
                const MCGAL::PointInt pi = mesh->floatPosToInt(h->vertex()->point());
                q[n] = {clampQ(pi.x()), clampQ(pi.y()), clampQ(pi.z())};
                ++n;
            }
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
        if (n != 3) continue;
        int best = 0;
        for (int i = 1; i < 3; ++i) {
            if (q[i] < q[best]) best = i;
        }
        Entry e;
        e.f = f;
        e.proxy = hs[best];
        int k = 0;
        for (int i = 0; i < 3; ++i) {
            const auto& qq = q[(best + i) % 3];
            e.key[k++] = qq[0];
            e.key[k++] = qq[1];
            e.key[k++] = qq[2];
        }
        e.key[9] = f->groupId();
        entries.push_back(e);
    }
    std::sort(entries.begin(), entries.end(), [](const Entry& a, const Entry& b) {
        return a.key < b.key;
    });
    std::vector<MCGAL::Facet*> ordered;
    ordered.reserve(entries.size());
    for (auto& e : entries) {
        e.f->reset(e.proxy);
        ordered.push_back(e.f);
    }
    mesh->faces() = std::move(ordered);
}

/// Walk group-g boundary loop starting from an explicit vertex (virtual anchor).
inline bool walkGroupLoopFromVertex(MCGAL::Mesh* mesh, MCGAL::Vertex* startV, int g,
                                    std::vector<MCGAL::Halfedge*>& loop) {
    loop.clear();
    if (!mesh || !startV || startV->isRemoved()) return false;
    MCGAL::Halfedge* start = nullptr;
    int bestOpp = 0;
    std::vector<int> bestSig;
    std::array<int, 3> bestQ{0, 0, 0};
    auto sigOf = [&](MCGAL::Vertex* v) {
        std::vector<int> gs;
        for (MCGAL::Halfedge* hh : pmsf::boundedRing(v)) {
            if (!hh || hh->isRemoved()) continue;
            if (hh->face() && !hh->face()->isRemoved()) gs.push_back(hh->face()->groupId());
        }
        std::sort(gs.begin(), gs.end());
        return gs;
    };
    for (MCGAL::Halfedge* h : pmsf::boundedRing(startV)) {
        if (!h || h->vertex() != startV || h->isRemoved()) continue;
        if (!scIsGroupBoundaryHalfedge(h, g)) continue;
        const int opp = h->opposite()->face()->groupId();
        auto sig = sigOf(h->end_vertex());
        const auto q = scAnchorQ(mesh, h->end_vertex());
        bool better = false;
        if (!start) better = true;
        else if (opp != bestOpp) better = opp < bestOpp;
        else if (sig != bestSig) better = sig < bestSig;
        else better = q < bestQ;
        if (better) { start = h; bestOpp = opp; bestSig = std::move(sig); bestQ = q; }
    }
    if (!start) return false;
    MCGAL::Halfedge* h = start;
    int guard = 0;
    while (h && ++guard < 100000) {
        loop.push_back(h);
        MCGAL::Halfedge* next = scNextGroupBoundaryHalfedge(h, g);
        if (!next) break;
        if (next == start) return true;
        h = next;
    }
    return false;
}

/// Enumerate all group-g boundary loops, each rotated so its first edge starts at
/// the loop vertex with the minimum quantized position.
/// Position-independent topology key of a vertex: degree, incident face group
/// multiset, neighbor degree multiset.  Used for canonical loop rotation so that
/// the encoder and decoder agree even when vertex positions drift by one
/// quantization step across rounds.
inline std::vector<int> scTopoVertexKey(MCGAL::Mesh* mesh, MCGAL::Vertex* v) {
    std::vector<int> key;
    if (!mesh || !v || v->isRemoved()) return key;
    std::vector<int> faceGroups;
    std::vector<int> neighborDeg;
    int deg = 0;
    for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
        if (!h || h->isRemoved()) continue;
        if (h->face() && !h->face()->isRemoved()) faceGroups.push_back(h->face()->groupId());
        MCGAL::Vertex* w = h->end_vertex();
        if (w && !w->isRemoved()) {
            // Live degree only: Vertex::halfedges() may contain stale entries from
            // removed faces; using its raw size makes the key history-dependent
            // and breaks loop rotation agreement between encoder/decoder.
            int liveDeg = 0;
            for (MCGAL::Halfedge* hh : pmsf::boundedRing(w)) {
                if (hh && !hh->isRemoved() && hh->face() && !hh->face()->isRemoved()) ++liveDeg;
            }
            neighborDeg.push_back(liveDeg);
            ++deg;
        }
    }
    std::sort(faceGroups.begin(), faceGroups.end());
    std::sort(neighborDeg.begin(), neighborDeg.end());
    key.push_back(deg);
    key.push_back(static_cast<int>(faceGroups.size()));
    key.insert(key.end(), faceGroups.begin(), faceGroups.end());
    key.insert(key.end(), neighborDeg.begin(), neighborDeg.end());
    return key;
}

inline std::vector<std::vector<MCGAL::Halfedge*>> collectGroupLoops(MCGAL::Mesh* mesh, int g) {
    std::vector<std::vector<MCGAL::Halfedge*>> out;
    if (!mesh) return out;
    std::vector<MCGAL::Halfedge*> edges;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge() || f->groupId() != g) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (scIsGroupBoundaryHalfedge(h, g)) edges.push_back(h);
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
    }
    std::set<MCGAL::Halfedge*> visited;
    for (MCGAL::Halfedge* seed : edges) {
        if (!seed || visited.count(seed)) continue;
        std::vector<MCGAL::Halfedge*> loop;
        MCGAL::Halfedge* h = seed;
        int guard = 0;
        while (h && !visited.count(h) && ++guard < 100000) {
            visited.insert(h);
            loop.push_back(h);
            MCGAL::Halfedge* next = scNextGroupBoundaryHalfedge(h, g);
            if (!next || next == seed) break;
            h = next;
        }
        if (loop.size() < 2) continue;
        std::vector<std::vector<int>> keys;
        keys.reserve(loop.size());
        for (MCGAL::Halfedge* e : loop) keys.push_back(scTopoVertexKey(mesh, e->vertex()));
        auto flatSeq = [&](std::size_t rot) {
            std::vector<int> flat;
            for (std::size_t i = 0; i < loop.size(); ++i) {
                const auto& k = keys[(rot + i) % loop.size()];
                flat.push_back(static_cast<int>(k.size()));
                flat.insert(flat.end(), k.begin(), k.end());
            }
            return flat;
        };
        std::size_t best = 0;
        std::vector<int> bestSeq = flatSeq(0);
        for (std::size_t r = 1; r < loop.size(); ++r) {
            auto seq = flatSeq(r);
            if (seq < bestSeq) { bestSeq = std::move(seq); best = r; }
        }
        std::vector<MCGAL::Halfedge*> rot;
        rot.reserve(loop.size());
        for (std::size_t i = 0; i < loop.size(); ++i)
            rot.push_back(loop[(best + i) % loop.size()]);
        out.push_back(std::move(rot));
    }
    std::sort(out.begin(), out.end(),
              [&](const std::vector<MCGAL::Halfedge*>& a, const std::vector<MCGAL::Halfedge*>& b) {
                  const auto ka = scTopoVertexKey(mesh, a[0]->vertex());
                  const auto kb = scTopoVertexKey(mesh, b[0]->vertex());
                  if (ka != kb) return ka < kb;
                  return a.size() < b.size();
              });
    return out;
}

/// Verify that the compact-codec style fan walk reproduces the expected ring.
/// Used at capture time so that collapses whose local fan is not simple (e.g.
/// a junction on the opposite endpoint) are rejected by the encoder instead of
/// producing an inverse the decoder cannot reproduce.
inline bool scWalkFanCheck(MCGAL::Halfedge* lastHalf, MCGAL::Vertex* apex,
                           MCGAL::Vertex* other, int fanFaces,
                           std::vector<MCGAL::Vertex*>& ring) {
    ring.clear();
    if (!lastHalf || !lastHalf->face() || fanFaces < 1) return false;
    auto thirdVertex = [](MCGAL::Halfedge* h0, MCGAL::Vertex* x,
                          MCGAL::Vertex* y) -> MCGAL::Vertex* {
        if (!h0) return nullptr;
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && h->vertex() && h->vertex() != x && h->vertex() != y) return h->vertex();
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
        return nullptr;
    };
    MCGAL::Facet* f = lastHalf->face();
    MCGAL::Vertex* p = thirdVertex(lastHalf, apex, other);
    if (!p) return false;
    std::vector<MCGAL::Vertex*> collected{p};
    for (int step = 1; step < fanFaces; ++step) {
        MCGAL::Halfedge* hedge = nullptr;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && h->vertex() == apex && h->end_vertex() == collected.back()) {
                hedge = h;
                break;
            }
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
        if (!hedge) return false;
        MCGAL::Halfedge* opp = hedge->opposite();
        if (!opp || opp->isRemoved() || !opp->face() || opp->face()->isRemoved()) return false;
        f = opp->face();
        MCGAL::Vertex* nxt = thirdVertex(f->proxyHalfedge(), apex, collected.back());
        if (!nxt || nxt == apex || nxt == other) return false;
        collected.push_back(nxt);
    }
    std::reverse(collected.begin(), collected.end());
    ring.push_back(apex);
    for (MCGAL::Vertex* x : collected) ring.push_back(x);
    return true;
}

inline std::vector<int> captureGroupPatch(MCGAL::Mesh* mesh, const SingleBoundaryPatch& patch) {
    if (!mesh || !patch.valid || !patch.removed) return {};
    // PMSF_SC_QLOC: unset or non-zero => compact q-locator payload; "0" disables it
    // so the topology-anchored loop locator (len 7) is used for PRE/mixed as well.
    const char* qlocEnv = std::getenv("PMSF_SC_QLOC");
    const bool useQLoc = qlocEnv && qlocEnv[0] != '0';
    int kA = 0, kB = 0, gA = -1, gB = -1;
    std::vector<MCGAL::Vertex*> ringA, ringB;
    if (patch.ringA.size() >= 2 && patch.ringB.size() >= 2 && patch.sideGidA >= 0 &&
        patch.sideGidB >= 0) {
        ringA = patch.ringA;
        ringB = patch.ringB;
        kA = static_cast<int>(ringA.size()) - 1;
        kB = static_cast<int>(ringB.size()) - 1;
        gA = patch.sideGidA;
        gB = patch.sideGidB;
    } else {
        if (patch.addedFaces.size() < 2) return {};
        gA = patch.addedFaces.front().gid;
        int nA = 0;
        for (const auto& f : patch.addedFaces) if (f.gid == gA) ++nA;
        const int nB = static_cast<int>(patch.addedFaces.size()) - nA;
        if (nA < 1 || nB < 1) return {};
        kA = nA + 1;
        kB = nB + 1;
        auto buildRing = [&](int startFace, int count) {
            std::vector<MCGAL::Vertex*> ring;
            const auto& f0 = patch.addedFaces[static_cast<std::size_t>(startFace)];
            ring.push_back(f0.v[0]);
            ring.push_back(f0.v[1]);
            ring.push_back(f0.v[2]);
            for (int i = 1; i < count; ++i)
                ring.push_back(patch.addedFaces[static_cast<std::size_t>(startFace + i)].v[2]);
            return ring;
        };
        ringA = buildRing(0, nA);
        ringB = buildRing(nA, nB);
        gB = patch.addedFaces.back().gid;
    }
    if (kA < 1 || kB < 1) return {};
    if (static_cast<int>(ringA.size()) != kA + 1 || static_cast<int>(ringB.size()) != kB + 1)
        return {};
    if (ringB.front() != ringA.back() || ringB.back() != ringA.front()) return {};
    MCGAL::Vertex* a = ringA.front();
    MCGAL::Vertex* b = ringA.back();
    // Corner ops: the loop-index locator can be stale after invert/re-collapse
    // (halfedge-pairing history), so address the boundary edge directly by its
    // endpoint q's.  Payload layout (len=11):
    //   aq(3) bq(3) gA packedK drq(3)
    if (kA == 1 || kB == 1) {
        if (kA < 1 || kB < 1 || kA >= 64 || kB >= 64) return {};
        const auto qa = scAnchorQ(mesh, a);
        const auto qb = scAnchorQ(mesh, b);
        const auto rq = scloop::qOf(mesh, patch.removed);
        const int drq0 = rq[0] - (qa[0] + qb[0]) / 2;
        const int drq1 = rq[1] - (qa[1] + qb[1]) / 2;
        const int drq2 = rq[2] - (qa[2] + qb[2]) / 2;
        std::vector<int> out;
        out.push_back(11);
        out.push_back(qa[0]); out.push_back(qa[1]); out.push_back(qa[2]);
        out.push_back(qb[0]); out.push_back(qb[1]); out.push_back(qb[2]);
        out.push_back(gA);
        out.push_back(kA * 64 + kB);
        out.push_back(drq0); out.push_back(drq1); out.push_back(drq2);
        return out;
    }
    MCGAL::Halfedge* hA = nullptr;
    for (MCGAL::Vertex* endpoint : {a, b}) {
        for (MCGAL::Halfedge* h : pmsf::boundedRing(endpoint)) {
            if (!h || h->isRemoved() || !h->vertex() || !h->end_vertex()) continue;
            if (h->vertex() != b || h->end_vertex() != a) continue;
            MCGAL::Halfedge* opp = h->opposite();
            if (!opp || opp->isRemoved() || !opp->face()) continue;
            if (h->face()->groupId() == gA && opp->face()->groupId() == gB) { hA = h; break; }
        }
        if (hA) break;
    }
    if (!hA) return {};
    // Reject collapses whose decoder-side fan walk cannot reproduce the exact
    // rings (junctions on the opposite endpoint etc.).
    {
        std::vector<MCGAL::Vertex*> gotA, gotB;
        const bool walkA = kA > 1 ? scWalkFanCheck(hA, a, b, kA - 1, gotA) : true;
        if (walkA && kA > 1) gotA.push_back(b);
        const bool walkB =
            (kB > 1 ? scWalkFanCheck(hA->opposite(), b, a, kB - 1, gotB) : true);
        if (walkB && kB > 1) gotB.push_back(a);
        const bool okA = walkA && (kA == 1 || gotA == ringA);
        const bool okB = walkB && (kB == 1 || gotB == ringB);
        if (!okA || !okB) {
            if (std::getenv("PMSF_SC_DEBUG"))
                std::fprintf(stderr, "[sc-fan-reject] walkA=%d okA=%d gotA=%zu ringA=%zu "
                                     "walkB=%d okB=%d gotB=%zu ringB=%zu kA=%d kB=%d\n",
                             walkA ? 1 : 0, okA ? 1 : 0, gotA.size(), ringA.size(),
                             walkB ? 1 : 0, okB ? 1 : 0, gotB.size(), ringB.size(), kA, kB);
            return {};
        }
    }
    if (useQLoc) {
        std::vector<int> compact = captureSingleBoundaryPatchCompact(mesh, patch);
        if (!compact.empty()) return compact;
        if (std::getenv("PMSF_SC_DEBUG"))
            std::fprintf(stderr, "[sc-qloc-capture-fail] kA=%d kB=%d gA=%d\n", kA, kB, gA);
    }
    // Loop locator: (loop index, offset) after canonical min-q rotation.  The
    // fan-walk validation above guarantees the decoder reproduces the same
    // local rings, so the addressing only needs the canonical loop order.
    const auto loops = collectGroupLoops(mesh, gA);
    int loopIdx = -1, loopOff = -1;
    for (std::size_t li = 0; li < loops.size() && loopIdx < 0; ++li) {
        for (std::size_t ei = 0; ei < loops[li].size(); ++ei) {
            if (loops[li][ei] == hA) {
                loopIdx = static_cast<int>(li);
                loopOff = static_cast<int>(ei);
                break;
            }
        }
    }
    if (loopIdx < 0) return {};
    const auto rq = scloop::qOf(mesh, patch.removed);
    MCGAL::Point orig = patch.removed->point();
    MCGAL::Point basePos = mesh->intPosToFloat(MCGAL::PointInt(rq[0], rq[1], rq[2]));
    const float fineStep = mesh->f_quantStep / 1024.0f;
    int fine[3] = {0, 0, 0};
    for (int k = 0; k < 3; ++k)
        fine[k] = std::clamp(
            static_cast<int>(std::lround((orig[k] - basePos[k]) / fineStep)), -512, 511);
    // Fine residual is only needed for freeze/collision workarounds; group mode
    // (no freeze) works with plain dequantized positions, so default to off.
    const bool fineAllowed = std::getenv("PMSF_SC_USE_FINE") != nullptr &&
                             std::getenv("PMSF_SC_NO_FINE") == nullptr;
    const bool hasFine =
        fineAllowed && (fine[0] != 0 || fine[1] != 0 || fine[2] != 0);
    const bool storeHash = std::getenv("PMSF_SC_OP_HASH") != nullptr;
    std::vector<int> out;
    if (kA < 1 || kB < 1 || kA >= 64 || kB >= 64) return {};
    const int packedK = kA * 64 + kB;
    const auto qaMid = scAnchorQ(mesh, a);
    const auto qbMid = scAnchorQ(mesh, b);
    const int drq[3] = {rq[0] - (qaMid[0] + qbMid[0]) / 2,
                        rq[1] - (qaMid[1] + qbMid[1]) / 2,
                        rq[2] - (qaMid[2] + qbMid[2]) / 2};
    out.push_back(7 + (hasFine ? 3 : 0) + (storeHash ? 2 : 0));
    out.push_back(loopIdx);
    out.push_back(loopOff);
    out.push_back(gA);
    out.push_back(packedK);
    out.push_back(drq[0]); out.push_back(drq[1]); out.push_back(drq[2]);
    if (hasFine) {
        out.push_back(fine[0]); out.push_back(fine[1]); out.push_back(fine[2]);
    }
    if (storeHash) {
        const unsigned long long ph =
            static_cast<unsigned long long>(scPositionalHash(mesh)) ^
            (static_cast<unsigned long long>(scOrientedHash(mesh)) * 1099511628211ull);
        out.push_back(static_cast<int>(ph & 0xffffffffull));
        out.push_back(static_cast<int>((ph >> 32) & 0xffffffffull));
    }
    return out;
}

/// Inverse for the virtual-anchor patch.
inline bool applyGroupPatchInverseDirect(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                         const std::vector<int>& data,
                                         std::size_t& cursor) {
    if (!mesh || cursor >= data.size()) return false;
    const int len = data[cursor++];
    if (len == 18) {
        --cursor;
        return applySingleBoundaryCompactPatchInverse(mesh, data, cursor);
    }
    if (len >= 19) {
        --cursor;
        return applySingleBoundaryCompactPatchOrderedInverse(mesh, data, cursor);
    }
    const int extra = len - 7;
    if ((extra != 0 && extra != 2 && extra != 3 && extra != 4 && extra != 5) ||
        cursor + static_cast<std::size_t>(len) > data.size()) {
        if (std::getenv("PMSF_SC_DEBUG"))
            std::fprintf(stderr, "[sc-ginv] bad len=%d size=%zu\n", len, data.size());
        return false;
    }
    if (len == 11) {
        // Direct endpoint-q locator (corner ops).  No loop traversal involved.
        const auto qa = std::array<int, 3>{data[cursor], data[cursor + 1], data[cursor + 2]};
        cursor += 3;
        const auto qb = std::array<int, 3>{data[cursor], data[cursor + 1], data[cursor + 2]};
        cursor += 3;
        const int gAq = data[cursor++];
        const int packedKq = data[cursor++];
        const int kAq = packedKq / 64;
        const int kBq = packedKq % 64;
        if (kAq < 1 || kBq < 1 || kAq >= 64 || kBq >= 64) return false;
        const MCGAL::PointInt drq{data[cursor], data[cursor + 1], data[cursor + 2]};
        cursor += 3;
        const MCGAL::PointInt rq{(qa[0] + qb[0]) / 2 + drq.x(), (qa[1] + qb[1]) / 2 + drq.y(),
                                 (qa[2] + qb[2]) / 2 + drq.z()};
        std::vector<int> compact;
        compact.push_back(18);
        compact.push_back(qa[0]); compact.push_back(qa[1]); compact.push_back(qa[2]);
        compact.push_back(qb[0]); compact.push_back(qb[1]); compact.push_back(qb[2]);
        compact.push_back(gAq);
        compact.push_back(kAq);
        compact.push_back(kBq);
        compact.push_back(rq.x()); compact.push_back(rq.y()); compact.push_back(rq.z());
        compact.push_back(-1); compact.push_back(-1); compact.push_back(-1);
        compact.push_back(-1); compact.push_back(-1); compact.push_back(-1);
        // Let the compact inverse enumerate q-matching candidates itself (distance ranked),
        // instead of trusting a single forced edge - duplicate coordinates otherwise pick
        // the wrong edge on meshes like fandisk.
        g_scForcedCandidate = nullptr;
        std::size_t c = 0;
        const bool okq = applySingleBoundaryCompactPatchInverse(mesh, compact, c);
        return okq && c == compact.size();
    }

    const int loopIdx = data[cursor++];
    const int loopOff = data[cursor++];
    const int gA = data[cursor++];
    const int packedK = data[cursor++];
    const int kA = packedK / 64;
    const int kB = packedK % 64;
    if (kA < 1 || kB < 1 || kA >= 64 || kB >= 64) return false;
    const MCGAL::PointInt drq{data[cursor], data[cursor + 1], data[cursor + 2]};
    cursor += 3;
    int fineX = 0, fineY = 0, fineZ = 0;
    bool hasFine = false;
    if (extra == 3 || extra == 5) {
        fineX = data[cursor++];
        fineY = data[cursor++];
        fineZ = data[cursor++];
        hasFine = true;
    }
    if (extra == 2 || extra == 5) cursor += 2;  // per-op expected hash (diagnostic)
    const auto loopList = collectGroupLoops(mesh.get(), gA);
    if (loopIdx < 0 || loopIdx >= static_cast<int>(loopList.size())) {
        if (std::getenv("PMSF_SC_DEBUG"))
            std::fprintf(stderr, "[sc-ginv] loopIdx=%d loops=%zu gA=%d\n", loopIdx,
                         loopList.size(), gA);
        return false;
    }
    const auto& hostLoop = loopList[static_cast<std::size_t>(loopIdx)];
    if (loopOff < 0 || loopOff >= static_cast<int>(hostLoop.size())) {
        if (std::getenv("PMSF_SC_DEBUG"))
            std::fprintf(stderr, "[sc-ginv] off=%d loopSize=%zu gA=%d\n", loopOff,
                         hostLoop.size(), gA);
        return false;
    }
    MCGAL::Halfedge* hA = hostLoop[static_cast<std::size_t>(loopOff)];
    if (!hA || !hA->face() || !hA->end_vertex() || !hA->vertex() ||
        hA->face()->groupId() != gA) {
        if (std::getenv("PMSF_SC_DEBUG"))
            std::fprintf(stderr, "[sc-ginv] bad hA gA=%d faceG=%d\n", gA,
                         (hA && hA->face()) ? hA->face()->groupId() : -9);
        return false;
    }
    const auto aq = scAnchorQ(mesh.get(), hA->end_vertex());
    const auto bq = scAnchorQ(mesh.get(), hA->vertex());
    const MCGAL::PointInt rq{(aq[0] + bq[0]) / 2 + drq.x(), (aq[1] + bq[1]) / 2 + drq.y(),
                             (aq[2] + bq[2]) / 2 + drq.z()};
    std::vector<int> compact;
    compact.push_back(18);
    compact.push_back(aq[0]); compact.push_back(aq[1]); compact.push_back(aq[2]);
    compact.push_back(bq[0]); compact.push_back(bq[1]); compact.push_back(bq[2]);
    compact.push_back(gA);
    compact.push_back(kA);
    compact.push_back(kB);
    compact.push_back(rq.x()); compact.push_back(rq.y()); compact.push_back(rq.z());
    compact.push_back(-1); compact.push_back(-1); compact.push_back(-1);
    compact.push_back(-1); compact.push_back(-1); compact.push_back(-1);
    std::size_t c = 0;
    g_scForcedCandidate = hA;
    if (hasFine) {
        g_scOverridePosActive = true;
        g_scOverridePosQ = rq;
        g_scOverridePos = mesh->intPosToFloat(rq);
        const float fineStep = mesh->f_quantStep / 1024.0f;
        g_scOverridePos[0] += static_cast<float>(fineX) * fineStep;
        g_scOverridePos[1] += static_cast<float>(fineY) * fineStep;
        g_scOverridePos[2] += static_cast<float>(fineZ) * fineStep;
    }
    const bool ok = applySingleBoundaryCompactPatchInverse(mesh, compact, c);
    g_scForcedCandidate = nullptr;
    g_scOverridePosActive = false;
    if (!ok) return false;
    return c == compact.size();
}

/// Encoder stage using virtual-anchor patches (no freeze, no serialized anchors).
/// A group-boundary vertex must have exactly one incoming and one outgoing
/// group-g boundary halfedge for every group g incident to it.  This is the
/// invariant scNextGroupBoundaryHalfedge relies on; a collapse that breaks it
/// makes the decoder-side loop walk degenerate (e.g. loops of size 2).
inline bool scBoundaryIsGroupRegular(MCGAL::Mesh* mesh) {
    if (!mesh) return false;
    std::map<std::pair<int, int>, int> outCnt, inCnt;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
            if (h && !h->isRemoved() && h->face() && h->vertex() && h->end_vertex() &&
                (!opp || opp->isRemoved() || !opp->face() || opp->face()->isRemoved() ||
                 opp->face()->groupId() != f->groupId())) {
                const int g = f->groupId();
                ++outCnt[{h->vertex()->poolId(), g}];
                ++inCnt[{h->end_vertex()->poolId(), g}];
            }
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
    }
    for (const auto& kv : outCnt)
        if (kv.second != 1) return false;
    for (const auto& kv : inCnt)
        if (kv.second != 1) return false;
    return true;
}

/// True when the vertex touches a real open boundary (a halfedge with no live
/// opposite face).  Such vertices must not be removed by the group-boundary
/// collapse: the group traversal / fan retriangulation assumes a closed (or
/// group-only) boundary.
inline bool scVertexOnOpenBoundary(MCGAL::Mesh* mesh, MCGAL::Vertex* v) {
    if (!mesh || !v || v->isRemoved()) return true;
    for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
        if (!h || h->isRemoved()) continue;
        MCGAL::Halfedge* opp = h->opposite();
        if (!opp || opp->isRemoved() || !opp->face() || opp->face()->isRemoved()) return true;
    }
    return false;
}

/// Number of halfedges that have no live opposite face (open boundary edges).
inline int scOpenBoundaryEdgeCount(MCGAL::Mesh* mesh) {
    if (!mesh) return -1;
    int n = 0;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && !h->isRemoved()) {
                MCGAL::Halfedge* opp = h->opposite();
                if (!opp || opp->isRemoved() || !opp->face() || opp->face()->isRemoved()) ++n;
            }
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
    }
    return n;
}

/// History-independent vertex order key: q, topology key, then sorted neighbour
/// q's.  Used to sort boundary-collapse candidates so the encoder dry-run clone
/// and the real pipeline pick exactly the same vertices (no pointer/poolId ties).
inline std::vector<long long> scVertexOrderKey(MCGAL::Mesh* mesh, MCGAL::Vertex* v) {
    std::vector<long long> key;
    if (!mesh || !v) return key;
    const MCGAL::PointInt q = mesh->floatPosToInt(v->point());
    key.push_back(q.x());
    key.push_back(q.y());
    key.push_back(q.z());
    for (int k : scTopoVertexKey(mesh, v)) key.push_back(k);
    std::vector<std::array<int, 3>> nq;
    for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
        if (!h || h->isRemoved() || !h->end_vertex() || h->end_vertex()->isRemoved()) continue;
        const MCGAL::PointInt qq = mesh->floatPosToInt(h->end_vertex()->point());
        nq.push_back({qq.x(), qq.y(), qq.z()});
    }
    std::sort(nq.begin(), nq.end());
    for (const auto& a : nq)
        for (int k : a) key.push_back(k);
    return key;
}

inline int runSingleCopyBoundaryEncoderStageGroup(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                                 int roundIndex, int maxOps,
                                                 const std::set<MCGAL::Vertex*>& protectedVerts,
                                                 std::vector<int>& roundData) {
    if (!mesh || maxOps <= 0) return 0;
    const int openBoundaryBefore = scOpenBoundaryEdgeCount(mesh.get());
    if (std::getenv("PMSF_SC_DEBUG")) {
        int boundaryHE = 0;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
            MCGAL::Halfedge* h0 = f->proxyHalfedge();
            MCGAL::Halfedge* h = h0;
            int guard = 0;
            do {
                MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
                if (h && !h->isRemoved() &&
                    (!opp || opp->isRemoved() || !opp->face() || opp->face()->isRemoved() ||
                     opp->face()->groupId() != f->groupId()))
                    ++boundaryHE;
                h = h ? h->next() : nullptr;
            } while (h && h != h0 && ++guard < 64);
        }
        std::map<int, int> fcount, bcount;
        for (MCGAL::Facet* f : mesh->faces()) {
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
        for (MCGAL::Vertex* v : mesh->vertices())
            if (v && !v->isRemoved()) ++liveV;
        std::fprintf(stderr, "[sc-stage-stats] round=%d liveV=%zu boundaryHE=%d want=%d", roundIndex,
                     liveV, boundaryHE, maxOps);
        for (auto& kv : fcount)
            std::fprintf(stderr, " g%d:F%d/B%d", kv.first, kv.second,
                         bcount.count(kv.first) ? bcount[kv.first] : 0);
        std::fprintf(stderr, "\n");
    }
    int applied = 0;
    int rejAlready = 0, rejProtected = 0, rejOpenVertex = 0, rejCollapse = 0,
        rejClosed = 0, rejStrict = 0, rejOpenEdge = 0, rejCapture = 0, rejCornerVerify = 0;
    const bool cornerVerify = std::getenv("PMSF_SC_CORNER_VERIFY") != nullptr;
    std::set<int> rejected;
    for (int iter = 0; iter < maxOps; ++iter) {
        std::set<MCGAL::Vertex*> cands;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
            MCGAL::Halfedge* h0 = f->proxyHalfedge();
            MCGAL::Halfedge* h = h0;
            int guard = 0;
            do {
                MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
                if (h && h->vertex() && h->next() && h->next()->vertex() && opp &&
                    !opp->isRemoved() && opp->face() && !opp->face()->isRemoved() &&
                    opp->face()->groupId() != f->groupId()) {
                    if (!h->vertex()->isRemoved()) cands.insert(h->vertex());
                    if (!h->next()->vertex()->isRemoved()) cands.insert(h->next()->vertex());
                }
                h = h ? h->next() : nullptr;
            } while (h && h != h0 && ++guard < 64);
        }
        if (cands.empty()) break;
        std::vector<MCGAL::Vertex*> ordered(cands.begin(), cands.end());
        {
            std::vector<std::vector<long long>> orderKeys;
            orderKeys.reserve(ordered.size());
            for (MCGAL::Vertex* v : ordered) orderKeys.push_back(scVertexOrderKey(mesh.get(), v));
            std::vector<std::size_t> orderIdx(ordered.size());
            for (std::size_t idx = 0; idx < orderIdx.size(); ++idx) orderIdx[idx] = idx;
            std::sort(orderIdx.begin(), orderIdx.end(), [&](std::size_t ia, std::size_t ib) {
                const auto& ka = orderKeys[ia];
                const auto& kb = orderKeys[ib];
                if (ka != kb) return ka < kb;
                return ia < ib;
            });
            std::vector<MCGAL::Vertex*> sortedVerts;
            sortedVerts.reserve(ordered.size());
            for (std::size_t idx : orderIdx) sortedVerts.push_back(ordered[idx]);
            ordered.swap(sortedVerts);
        }
        bool ok = false;
        for (MCGAL::Vertex* v : ordered) {
            if (!v || rejected.count(v->poolId()) || protectedVerts.count(v)) {
                if (!v) continue;
                if (rejected.count(v->poolId())) ++rejAlready; else ++rejProtected;
                continue;
            }
            if (scVertexOnOpenBoundary(mesh.get(), v)) {
                ++rejOpenVertex;
                rejected.insert(v->poolId());
                continue;
            }
            std::uint64_t cornerPreHash = 0;
            bool cornerPreHashReady = false;
            if (cornerVerify) {
                cornerPreHash = scTrialClampedHash(mesh.get());
                cornerPreHashReady = true;
            }
            SingleBoundaryPatch patch;
            if (!collapseSingleGroupBoundaryVertex(mesh, v, patch)) {
                ++rejCollapse;
                                rejected.insert(v->poolId());
                continue;
            }
            if (!localSingleBoundaryPatchClosed(mesh, patch)) {
                restoreSingleBoundaryPatch(mesh, patch);
                ++rejClosed;
                rejected.insert(v->poolId());
                continue;
            }
            pmsf::repairHalfedgePairing(mesh);
            // Only needed when later rounds must keep collecting symbols from the
            // pruned mesh; last-round stages are decoded immediately and tolerate
            // more general local fans.
            if (std::getenv("PMSF_SC_STAGE_STRICT_REGULAR") &&
                !scBoundaryIsGroupRegular(mesh.get())) {
                restoreSingleBoundaryPatch(mesh, patch);
                ++rejStrict;
                rejected.insert(v->poolId());
                continue;
            }
            // The base mesh is rebuilt from face indices at decode time, so its
            // halfedge pairing is canonical.  Re-pair here too before capturing,
            // otherwise encoder-side loop walking sees stale opposites and the
            // (loopIdx, offset) addresing differs from the decoder.
            pmsf::repairHalfedgePairing(mesh);
            pmsf::rebuildVertexHalfedgeRings(mesh);
            if (scOpenBoundaryEdgeCount(mesh.get()) != openBoundaryBefore) {
                restoreSingleBoundaryPatch(mesh, patch);
                ++rejOpenEdge;
                rejected.insert(v->poolId());
                continue;
            }
            std::vector<int> data = captureGroupPatch(mesh.get(), patch);
            if (data.empty()) {
                restoreSingleBoundaryPatch(mesh, patch);
                ++rejCapture;
                rejected.insert(v->poolId());
                continue;
            }
            // Corner ops (kA==1 or kB==1): verify on the real encoder mesh by inverting
            // the captured op (then re-applying the deterministic collapse).  This uses
            // no trial clone, so it cannot exhaust the global context pool.
            if (cornerVerify && cornerPreHashReady && data.size() >= 8 && data[0] >= 7) {
                const int packedK = data[4];
                const int kA = packedK / 64;
                const int kB = packedK % 64;
                if (kA == 1 || kB == 1) {
                    std::vector<int> op = data;
                    std::size_t cur = 0;
                    bool inverted =
                        applyGroupPatchInverseDirect(mesh, op, cur) && cur == op.size();
                    if (inverted) {
                        pmsf::repairHalfedgePairing(mesh);
                        pmsf::rebuildVertexHalfedgeRings(mesh);
                        inverted = scTrialClampedHash(mesh.get()) == cornerPreHash;
                    }
                    if (!inverted) {
                        restoreSingleBoundaryPatch(mesh, patch);
                        ++rejCornerVerify;
                        rejected.insert(v->poolId());
                        continue;
                    }
                    MCGAL::Vertex* v2 = nullptr;
                    const auto vq = mesh->floatPosToInt(patch.pos);
                    for (MCGAL::Vertex* u : mesh->vertices()) {
                        if (!u || u->isRemoved()) continue;
                        if (mesh->floatPosToInt(u->point()) == vq) {
                            v2 = u;
                            break;
                        }
                    }
                    SingleBoundaryPatch patch2;
                    if (!v2 || !collapseSingleGroupBoundaryVertex(mesh, v2, patch2)) {
                        ++rejCornerVerify;
                        rejected.insert(v->poolId());
                        continue;
                    }
                    std::vector<int> data2 = captureGroupPatch(mesh.get(), patch2);
                    if (data2.empty()) {
                        restoreSingleBoundaryPatch(mesh, patch2);
                        ++rejCornerVerify;
                        rejected.insert(v->poolId());
                        continue;
                    }
                    data = data2;
                }
            }
            roundData.insert(roundData.end(), data.begin(), data.end());
            if (const char* of = std::getenv("PMSF_SC_ENC_OPFLAGS"); of) {
                if (FILE* ff = std::fopen(of, "a")) {
                    std::fprintf(ff, "ROUND %d OP %d\n", roundIndex, applied);
                    for (MCGAL::Facet* ffac : mesh->faces()) {
                        if (!ffac || ffac->isRemoved() || !ffac->proxyHalfedge()) continue;
                        const auto vs = pmsf::sbdetail::faceVertices(ffac);
                        if (!vs[0]) continue;
                        std::vector<std::array<int, 3>> qs;
                        for (int k = 0; k < 3; ++k) {
                            const auto q = mesh->floatPosToInt(vs[k]->point());
                            qs.push_back({q.x(), q.y(), q.z()});
                        }
                        std::sort(qs.begin(), qs.end());
                        std::fprintf(ff, "%d %d %d %d %d %d %d %d %d %d %d %d\n",
                                     ffac->groupId(), ffac->isSplittable() ? 1 : 0,
                                     ffac->isUnsplittable() ? 1 : 0, qs[0][0], qs[0][1],
                                     qs[0][2], qs[1][0], qs[1][1], qs[1][2], qs[2][0],
                                     qs[2][1], qs[2][2]);
                    }
                    std::fclose(ff);
                }
            }
            for (MCGAL::Vertex* rv : patch.ringA)
                if (rv && !rv->isRemoved()) scStageFrozenVertices().insert(rv);
            for (MCGAL::Vertex* rv : patch.ringB)
                if (rv && !rv->isRemoved()) scStageFrozenVertices().insert(rv);
            if (cornerVerify) scStageOpPreHashes().push_back(cornerPreHash);
            ++applied;
            if (std::getenv("PMSF_SC_DEBUG") && data.size() >= 10)
                std::fprintf(stderr, "[sc-group-cap] round=%d op=%d gA=%d kA=%d kB=%d\n",
                             roundIndex, applied, data[7], data[8], data[9]);
            ok = true;
            break;
        }
        if (!ok) break;
    }
    return applied;
}


/// --- Encoder-side dry-run safety check for the group boundary stage ---
///
/// Applies the stage to a rebuilt copy of the round state, then runs the exact
/// decoder inverses in reverse and verifies the copy returns to the original
/// topology.  This removes hand-tuned maxOps thresholds: if K ops cannot be
/// inverted the encoder binary-searches a smaller K.
struct ScTrialSnapVert {
    MCGAL::Point p;
    int gid = 0;
};
struct ScTrialSnapFace {
    int v[3] = {0, 0, 0};
    int gid = 0;
};
struct ScTrialSnap {
    std::vector<ScTrialSnapVert> verts;
    std::vector<ScTrialSnapFace> faces;
};

inline ScTrialSnap scCaptureTrialSnap(MCGAL::Mesh* mesh) {
    ScTrialSnap snap;
    if (!mesh) return snap;
    std::unordered_map<MCGAL::Vertex*, int> index;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        index[v] = static_cast<int>(snap.verts.size());
        ScTrialSnapVert sv;
        sv.p = v->point();
        sv.gid = v->groupId();
        snap.verts.push_back(sv);
    }
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        std::array<MCGAL::Vertex*, 3> vs{nullptr, nullptr, nullptr};
        int k = 0;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && h->vertex() && k < 3) vs[k++] = h->vertex();
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
        if (k != 3) continue;
        ScTrialSnapFace sf;
        bool ok = true;
        for (int j = 0; j < 3; ++j) {
            auto it = index.find(vs[j]);
            if (it == index.end()) { ok = false; break; }
            sf.v[j] = it->second;
        }
        if (!ok) continue;
        sf.gid = f->groupId();
        snap.faces.push_back(sf);
    }
    return snap;
}

inline std::shared_ptr<MCGAL::Mesh> scBuildTrialMesh(const ScTrialSnap& snap, int meshId,
                                                  const MCGAL::Mesh* source = nullptr) {
    auto m = std::make_shared<MCGAL::Mesh>();
    m->setMeshId(meshId);
    // Quantization parameters are required for floatPosToInt / intPosToFloat; without
    // them a trial clone captures different q values than the real mesh (historical
    // dry-run fidelity mismatch).
    if (source) {
        m->i_nbQuantBits = source->i_nbQuantBits;
        m->f_quantStep = source->f_quantStep;
        m->bboxMin = source->bboxMin;
        m->enableQuantization = source->enableQuantization;
    }
    std::vector<MCGAL::Vertex*> vs;
    vs.reserve(snap.verts.size());
    for (const auto& sv : snap.verts) {
        MCGAL::Vertex* v = MCGAL::contextPool.allocateVertexFromPool(meshId, sv.p);
        if (!v) return nullptr;
        v->setGroupId(sv.gid);
        m->add_vertex(v);
        vs.push_back(v);
    }
    for (const auto& sf : snap.faces) {
        MCGAL::Facet* f = createFaceFromTripleManual(
            m, vs[static_cast<std::size_t>(sf.v[0])], vs[static_cast<std::size_t>(sf.v[1])],
            vs[static_cast<std::size_t>(sf.v[2])], sf.gid);
        if (!f) return nullptr;
    }
    pmsf::rebuildVertexHalfedgeRings(m);
    return m;
}

/// Env-gated preflight for multi-round every-round staging: run the v2 stage on
/// a throw-away clone first.  If the clone's post-stage anchor enumeration fails,
/// the real round is left unstaged instead of aborting compression.  The clone
/// is built from a position/group snapshot and reuses the source mesh's
/// quantization parameters, so forward stage selection is identical to the real
/// mesh (candidate order is the live-vertex iteration order in both).
inline bool scV2PreflightStage(const std::shared_ptr<MCGAL::Mesh>& mesh,
                               int roundIndex, int maxOps,
                               const std::set<MCGAL::Vertex*>& protectedVerts,
                               const std::vector<std::pair<int, MCGAL::Vertex*>>& anchors) {
    if (!mesh) return false;
    const ScTrialSnap snap = scCaptureTrialSnap(mesh.get());
    if (snap.faces.empty()) return false;
    auto trial = scBuildTrialMesh(snap, mesh->meshId(), mesh.get());
    if (!trial) return false;
    std::unordered_map<MCGAL::Vertex*, int> liveIndex;
    liveIndex.reserve(snap.verts.size() * 2);
    int li = 0;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        liveIndex[v] = li++;
    }
    std::vector<MCGAL::Vertex*> trialVerts;
    trialVerts.reserve(snap.verts.size());
    for (MCGAL::Vertex* v : trial->vertices()) {
        if (!v || v->isRemoved()) continue;
        trialVerts.push_back(v);
    }
    if (trialVerts.size() != snap.verts.size()) return false;
    auto mapVert = [&](MCGAL::Vertex* v) -> MCGAL::Vertex* {
        if (!v) return nullptr;
        auto it = liveIndex.find(v);
        if (it == liveIndex.end()) return nullptr;
        if (it->second < 0 || it->second >= static_cast<int>(trialVerts.size())) return nullptr;
        return trialVerts[static_cast<std::size_t>(it->second)];
    };
    std::set<MCGAL::Vertex*> trialProtected;
    for (MCGAL::Vertex* v : protectedVerts) {
        if (MCGAL::Vertex* tv = mapVert(v)) trialProtected.insert(tv);
    }
    std::vector<std::pair<int, MCGAL::Vertex*>> trialAnchors;
    trialAnchors.reserve(anchors.size());
    for (const auto& ag : anchors) {
        MCGAL::Vertex* tv = mapVert(ag.second);
        if (!tv) return false;
        trialAnchors.emplace_back(ag.first, tv);
    }
    BoundaryV2Round trialOut;
    const int trialOps =
        runSingleCopyBoundaryStageV2(trial, roundIndex, maxOps, trialProtected, trialAnchors, trialOut);
    return trialOps >= 0;
}

inline int scCountRoundPatches(const std::vector<int>& data) {
    int ops = 0;
    std::size_t pos = 0;
    while (pos < data.size()) {
        const int len = data[pos];
        if ((len != 7 && len != 9 && len != 10 && len != 11 && len != 12 && len != 18 && !(len >= 19 && len <= 275)) ||
            pos + 1 + static_cast<std::size_t>(len) > data.size())
            break;
        pos += 1 + static_cast<std::size_t>(len);
        ++ops;
    }
    return ops;
}

/// Clamped-q topology/position hash used by the encoder dry-run.  Unlike
/// scPositionalHash it clamps to the quantization range, because the decoder
/// rebuilds the base mesh from clamped q (a vertex at bbox max may be written
/// as 2^bits and read back as 2^bits-1).
inline std::uint64_t scTrialClampedHash(MCGAL::Mesh* mesh) {
    if (!mesh) return 0;
    const int maxQ = mesh->i_nbQuantBits > 0 ? (1 << mesh->i_nbQuantBits) - 1 : (1 << 30);
    auto cq = [&](MCGAL::Vertex* v) {
        const MCGAL::PointInt q = mesh->floatPosToInt(v->point());
        return std::array<int, 3>{std::clamp(q.x(), 0, maxQ), std::clamp(q.y(), 0, maxQ),
                                  std::clamp(q.z(), 0, maxQ)};
    };
    std::vector<std::array<int, 3>> verts;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        verts.push_back(cq(v));
    }
    std::sort(verts.begin(), verts.end());
    std::vector<std::array<int, 7>> faces;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        const auto vs = sbdetail::faceVertices(f);
        if (!vs[0]) continue;
        std::array<std::array<int, 3>, 3> q{};
        for (int k = 0; k < 3; ++k) q[k] = cq(vs[k]);
        std::sort(q.begin(), q.end());
        faces.push_back({q[0][0], q[0][1], q[0][2], q[1][0], q[1][1], q[1][2], f->groupId()});
    }
    std::sort(faces.begin(), faces.end());
    std::uint64_t h = 1469598103934665603ull;
    h = scMixHash(h, verts.size());
    for (const auto& v : verts)
        for (int k = 0; k < 3; ++k) h = scMixHash(h, static_cast<std::uint64_t>(v[k]));
    h = scMixHash(h, faces.size());
    for (const auto& f : faces)
        for (int k = 0; k < 7; ++k) h = scMixHash(h, static_cast<std::uint64_t>(f[k]));
    return h;
}

struct ScTrialStats {
    std::size_t V = 0, F = 0, H = 0;
    std::map<std::pair<int, int>, int> boundaryPairs;
};

/// Topology-only stats used for dry-run verification (positions are allowed to
/// differ by sub-quantization: the decoder rebuilds them from clamped q).
inline ScTrialStats scTrialMeshStats(MCGAL::Mesh* mesh) {
    ScTrialStats st;
    if (!mesh) return st;
    for (MCGAL::Vertex* v : mesh->vertices())
        if (v && !v->isRemoved()) ++st.V;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        ++st.F;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && !h->isRemoved()) {
                ++st.H;
                MCGAL::Halfedge* opp = h->opposite();
                const int ga = f->groupId();
                const int gb = (opp && !opp->isRemoved() && opp->face() &&
                                !opp->face()->isRemoved())
                                   ? opp->face()->groupId()
                                   : -1;
                st.boundaryPairs[{std::min(ga, gb), std::max(ga, gb)}] += 1;
            }
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
    }
    return st;
}

/// Build a mesh equivalent to what the decoder gets after reading the written
/// base mesh: vertex positions are quantized+clamped and rebuilt with
/// intPosToFloat, faces use the writer order (proxy halfedge traversal), and
/// halfedges are allocated through the same pool routine as buildFromBuffer.
inline std::shared_ptr<MCGAL::Mesh> scBuildDecoderBaseFromMesh(MCGAL::Mesh* mesh) {
    if (!mesh) return nullptr;
    auto m = std::make_shared<MCGAL::Mesh>();
    m->setMeshId(0);
    m->i_nbQuantBits = mesh->i_nbQuantBits;
    m->f_quantStep = mesh->f_quantStep;
    m->bboxMin = mesh->bboxMin;
    m->enableQuantization = mesh->enableQuantization;
    const int maxQ = mesh->i_nbQuantBits > 0 ? (1 << mesh->i_nbQuantBits) - 1 : (1 << 30);
    std::unordered_map<MCGAL::Vertex*, MCGAL::Vertex*> map;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        MCGAL::PointInt q = mesh->floatPosToInt(v->point());
        for (int k = 0; k < 3; ++k) q[k] = std::clamp(q[k], 0, maxQ);
        const MCGAL::Point p = mesh->intPosToFloat(q);
        MCGAL::Vertex* nv = MCGAL::contextPool.allocateVertexFromPool(0, p);
        if (!nv) return nullptr;
        m->add_vertex(nv);
        map[v] = nv;
    }
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        std::vector<MCGAL::Vertex*> vts;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && h->vertex()) {
                auto it = map.find(h->vertex());
                if (it == map.end()) return nullptr;
                vts.push_back(it->second);
            }
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
        if (vts.size() != 3) return nullptr;
        MCGAL::Facet* nf = MCGAL::contextPool.allocateFaceFromPool(0, vts);
        if (!nf) return nullptr;
        nf->setGroupId(f->groupId());
        m->add_face(nf);
    }
    return m;
}

inline bool scTrialStageInvertible(const ScTrialSnap& snap, const ScTrialStats& target,
                                   std::uint64_t targetHash, int roundIndex, int wantedOps,
                                   MCGAL::Mesh* source = nullptr) {
    if (wantedOps <= 0) return true;
    auto trial = scBuildTrialMesh(snap, 0, source);
    if (!trial) return false;
    pmsf::repairHalfedgePairing(trial);
    std::vector<int> data;
    std::set<MCGAL::Vertex*> noProtect;
    runSingleCopyBoundaryEncoderStageGroup(trial, roundIndex, wantedOps, noProtect, data);
    if (data.empty()) return false;
    std::vector<int> offsets;
    std::size_t pos = 0;
    while (pos < data.size()) {
        const int len = data[pos];
        if ((len != 7 && len != 9 && len != 10 && len != 11 && len != 12 && len != 18 && !(len >= 19 && len <= 275)) ||
            pos + 1 + static_cast<std::size_t>(len) > data.size())
            return false;
        offsets.push_back(static_cast<int>(pos));
        pos += 1 + static_cast<std::size_t>(len);
    }
    auto base = scBuildDecoderBaseFromMesh(trial.get());
    if (!base) return false;
    base->i_nbQuantBits = trial->i_nbQuantBits;
    base->f_quantStep = trial->f_quantStep;
    for (auto it = offsets.rbegin(); it != offsets.rend(); ++it) {
        const int startPos = *it;
        const int len = data[static_cast<std::size_t>(startPos)];
        std::vector<int> op(data.begin() + startPos, data.begin() + startPos + 1 + len);
        std::size_t cur = 0;
        if (!applyGroupPatchInverseDirect(base, op, cur) || cur != op.size()) return false;
        pmsf::repairHalfedgePairing(base);
    }
    const ScTrialStats got = scTrialMeshStats(base.get());
    const std::uint64_t gotHash = scTrialClampedHash(base.get());
    const bool ok = got.V == target.V && got.F == target.F && got.H == target.H &&
                    got.boundaryPairs == target.boundaryPairs && gotHash == targetHash;
    return ok;
}

/// Returns the largest K <= wantOps whose encoder round data is invertible by
/// the decoder-side inverse (binary search, bounded trials).
/// Verify a staged round's patch data by rebuilding the exact decoder base from
/// the (post-stage) real mesh, running the decoder inverses in reverse, and
/// comparing topology stats + clamped hash against the pre-stage reference.
inline bool scVerifyGroupStageOnDecoderBase(MCGAL::Mesh* mesh, const std::vector<int>& data,
                                            const ScTrialSnap& snap,
                                            const ScTrialStats& target,
                                            std::uint64_t targetHash) {
    (void)snap;
    if (!mesh || data.empty()) return true;
    std::vector<int> offsets;
    std::size_t pos = 0;
    while (pos < data.size()) {
        const int len = data[pos];
        if ((len != 7 && len != 9 && len != 10 && len != 11 && len != 12 && len != 18 && !(len >= 19 && len <= 275)) ||
            pos + 1 + static_cast<std::size_t>(len) > data.size())
            return false;
        offsets.push_back(static_cast<int>(pos));
        pos += 1 + static_cast<std::size_t>(len);
    }
    auto base = scBuildDecoderBaseFromMesh(mesh);
    if (!base) return false;
    base->i_nbQuantBits = mesh->i_nbQuantBits;
    base->f_quantStep = mesh->f_quantStep;
    pmsf::repairHalfedgePairing(base);
    int invIdx = 0;
    const int invTotal = static_cast<int>(offsets.size());
    for (auto it = offsets.rbegin(); it != offsets.rend(); ++it, ++invIdx) {
        const int startPos = *it;
        const int len = data[static_cast<std::size_t>(startPos)];
        std::vector<int> op(data.begin() + startPos, data.begin() + startPos + 1 + len);
        std::size_t cur = 0;
        if (!applyGroupPatchInverseDirect(base, op, cur) || cur != op.size()) {
            if (std::getenv("PMSF_SC_DEBUG"))
                std::fprintf(stderr,
                             "[sc-verify] inverse fail at reverseIndex=%d/%d (len=%d kA=%d kB=%d "
                             "gA=%d)\n",
                             invIdx, invTotal, len, len >= 7 ? op[4] / 64 : -1,
                             len >= 7 ? op[4] % 64 : -1, len >= 5 ? op[3] : -1);
            return false;
        }
        pmsf::repairHalfedgePairing(base);
        pmsf::rebuildVertexHalfedgeRings(base);
        const auto& preHashes = scStageOpPreHashes();
        if (!preHashes.empty() && invIdx < static_cast<int>(preHashes.size())) {
            const int encodeIndex = invTotal - 1 - invIdx;
            if (encodeIndex >= 0 && encodeIndex < static_cast<int>(preHashes.size())) {
                const std::uint64_t gotH = scTrialClampedHash(base.get());
                if (gotH != preHashes[static_cast<std::size_t>(encodeIndex)]) {
                    if (std::getenv("PMSF_SC_DEBUG"))
                        std::fprintf(stderr,
                                     "[sc-verify] state mismatch after inverse of op %d "
                                     "(len=%d kA=%d kB=%d gA=%d): got=%llx want=%llx\n",
                                     encodeIndex, len, len >= 7 ? op[4] / 64 : -1,
                                     len >= 7 ? op[4] % 64 : -1, len >= 5 ? op[3] : -1,
                                     static_cast<unsigned long long>(gotH),
                                     static_cast<unsigned long long>(
                                         preHashes[static_cast<std::size_t>(encodeIndex)]));
                    return false;
                }
            }
        }
    }
    const ScTrialStats got = scTrialMeshStats(base.get());
    const std::uint64_t gotHash = scTrialClampedHash(base.get());
    if (got.V != target.V || got.F != target.F || got.H != target.H ||
        got.boundaryPairs != target.boundaryPairs) {
        if (std::getenv("PMSF_SC_DEBUG"))
            std::fprintf(stderr,
                         "[sc-verify] stats mismatch got V=%zu F=%zu H=%zu pairs=%zu vs target "
                         "V=%zu F=%zu H=%zu pairs=%zu\n",
                         got.V, got.F, got.H, got.boundaryPairs.size(), target.V, target.F,
                         target.H, target.boundaryPairs.size());
        return false;
    }
    if (gotHash != targetHash) {
        if (std::getenv("PMSF_SC_DEBUG"))
            std::fprintf(stderr, "[sc-verify] hash mismatch got=%llx target=%llx\n",
                         static_cast<unsigned long long>(gotHash),
                         static_cast<unsigned long long>(targetHash));
        return false;
    }
    return true;
}

/// Undo staged patch data directly on the real encoder mesh (used when the
/// verification above rejects the stage).
inline bool scRollbackGroupStage(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                 const std::vector<int>& data) {
    if (!mesh || data.empty()) return true;
    std::vector<int> offsets;
    std::size_t pos = 0;
    while (pos < data.size()) {
        const int len = data[pos];
        if ((len != 7 && len != 9 && len != 10 && len != 11 && len != 12 && len != 18 && !(len >= 19 && len <= 275)) ||
            pos + 1 + static_cast<std::size_t>(len) > data.size())
            return false;
        offsets.push_back(static_cast<int>(pos));
        pos += 1 + static_cast<std::size_t>(len);
    }
    int applied = 0;
    for (auto it = offsets.rbegin(); it != offsets.rend(); ++it) {
        const int startPos = *it;
        const int len = data[static_cast<std::size_t>(startPos)];
        std::vector<int> op(data.begin() + startPos, data.begin() + startPos + 1 + len);
        std::size_t cur = 0;
        const bool ok = applyGroupPatchInverseDirect(mesh, op, cur);
        if (!ok || cur != op.size()) {
            return false;
        }
        ++applied;
        pmsf::repairHalfedgePairing(mesh);
        pmsf::rebuildVertexHalfedgeRings(mesh);
    }
    return true;
}

/// Find a live halfedge whose endpoint q's match (either direction).
inline MCGAL::Halfedge* scFindHalfedgeByQ(MCGAL::Mesh* mesh, const std::array<int, 3>& aq,
                                         const std::array<int, 3>& bq) {
    if (!mesh) return nullptr;
    const int maxQ = mesh->i_nbQuantBits > 0 ? (1 << mesh->i_nbQuantBits) - 1 : (1 << 30);
    auto cq = [&](MCGAL::Vertex* v) {
        const MCGAL::PointInt q = mesh->floatPosToInt(v->point());
        return std::array<int, 3>{std::clamp(q.x(), 0, maxQ), std::clamp(q.y(), 0, maxQ),
                                  std::clamp(q.z(), 0, maxQ)};
    };
    auto near = [](const std::array<int, 3>& p, const std::array<int, 3>& q) {
        return std::abs(p[0] - q[0]) <= 1 && std::abs(p[1] - q[1]) <= 1 &&
               std::abs(p[2] - q[2]) <= 1;
    };
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && h->vertex() && h->end_vertex() && !h->isRemoved()) {
                const auto qa = cq(h->vertex());
                const auto qb = cq(h->end_vertex());
                if ((near(qa, aq) && near(qb, bq)) || (near(qa, bq) && near(qb, aq)))
                    return h;
            }
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
    }
    return nullptr;
}

/// Rebuild the live mesh from a snapshot (used when a staged round fails
/// verification).  All existing vertex/facet pointers become invalid, so callers
/// must remap seeds/anchors afterwards.
inline bool scRestoreMeshFromSnap(std::shared_ptr<MCGAL::Mesh>& mesh, const ScTrialSnap& snap) {
    if (!mesh || snap.verts.empty() || snap.faces.empty()) return false;
    auto fresh = scBuildTrialMesh(snap, mesh->meshId());
    if (!fresh) return false;
    fresh->i_nbQuantBits = mesh->i_nbQuantBits;
    fresh->f_quantStep = mesh->f_quantStep;
    fresh->bboxMin = mesh->bboxMin;
    mesh = fresh;
    pmsf::repairHalfedgePairing(mesh);
    pmsf::rebuildVertexHalfedgeRings(mesh);
    return true;
}

inline int scFindSafeGroupMaxOps(MCGAL::Mesh* mesh, int roundIndex, int wantOps,
                                 const std::set<MCGAL::Vertex*>& protectedVerts,
                                 bool validatedConfig = false) {
    (void)protectedVerts;
    if (!mesh || wantOps <= 0) return 0;
    const bool autoMode = std::getenv("PMSF_SC_BOUNDARY_AUTO") != nullptr;
    const bool forced = std::getenv("PMSF_SC_DRYRUN") != nullptr;
    if (!autoMode && !forced) return wantOps;
    const ScTrialSnap snap = scCaptureTrialSnap(mesh);
    if (snap.verts.empty() || snap.faces.empty()) return 0;
    const ScTrialStats target = scTrialMeshStats(mesh);
    auto ref = scBuildTrialMesh(snap, 0, mesh);
    if (!ref) return 0;
    pmsf::repairHalfedgePairing(ref);
    const std::uint64_t targetHash = scTrialClampedHash(ref.get());
    if (autoMode && !forced) {
        // Bounded preflight: try wantOps; if the trial says the stage is not invertible,
        // descend geometrically to recover partial savings instead of going neutral.
        // Each trial allocates two clones from the global context pool (no per-object
        // free), so every attempt is budget-checked; when the budget is too low AUTO
        // falls back to "run the real stage + verification gate".
        auto poolRemaining = [](const std::vector<int>& sizes, const std::vector<int>& used) {
            if (sizes.empty() || used.empty()) return -1LL;
            return static_cast<long long>(sizes[0]) - static_cast<long long>(used[0]);
        };
        const long long fCount = static_cast<long long>(snap.faces.size());
        const long long vCount = static_cast<long long>(snap.verts.size());
        const auto trialBudgetOk = [&]() {
            const long long needH = 6LL * fCount + 8LL * vCount + 8192;
            const long long needF = 2LL * fCount + 8192;
            const long long needV = 2LL * vCount + 8192;
            const long long remH =
                poolRemaining(MCGAL::contextPool.hpoolSizes, MCGAL::contextPool.hindexs);
            const long long remF =
                poolRemaining(MCGAL::contextPool.fpoolSizes, MCGAL::contextPool.findexs);
            const long long remV =
                poolRemaining(MCGAL::contextPool.vpoolSizes, MCGAL::contextPool.vindexs);
            return !((remH >= 0 && remH < needH) || (remF >= 0 && remF < needF) ||
                     (remV >= 0 && remV < needV));
        };
        if (!trialBudgetOk()) {
            if (std::getenv("PMSF_SC_DEBUG"))
                std::fprintf(stderr,
                             "[sc-auto-preflight] round=%d want=%d skipped (context pool "
                             "budget too low)\n",
                             roundIndex, wantOps);
            return wantOps;
        }
        // Candidate ladder: explicit per-mesh optima measured on the validated matrix
        // (fandisk 1000-2000, horse 500, arc 350, lamp 250, the rest 200 or lower).
        // The trial is conservative, so descending this ladder recovers a larger safe K
        // than a plain /2 descent.  When the context pool only affords a couple of
        // trials, try the validated 200 baseline right after 500 so AUTO cannot
        // degrade to neutral merely because the high candidates were rejected.
        int affordable = 0;
        {
            const long long needH = 6LL * fCount + 8LL * vCount + 8192;
            const long long needF = 2LL * fCount + 8192;
            const long long needV = 2LL * vCount + 8192;
            const long long remH2 =
                poolRemaining(MCGAL::contextPool.hpoolSizes, MCGAL::contextPool.hindexs);
            const long long remF2 =
                poolRemaining(MCGAL::contextPool.fpoolSizes, MCGAL::contextPool.findexs);
            const long long remV2 =
                poolRemaining(MCGAL::contextPool.vpoolSizes, MCGAL::contextPool.vindexs);
            long long trials = 1LL << 30;
            if (remH2 >= 0) trials = std::min(trials, remH2 / needH);
            if (remF2 >= 0) trials = std::min(trials, remF2 / needF);
            if (remV2 >= 0) trials = std::min(trials, remV2 / needV);
            if (trials > 8) trials = 8;
            affordable = static_cast<int>(std::max(0LL, trials));
        }
        std::vector<int> order;
        order.push_back(500);
        if (affordable >= 5) {
            for (int k : {350, 300, 250, 200, 150, 100, 64, 32, 16, 8}) order.push_back(k);
        } else if (affordable >= 3) {
            for (int k : {350, 250, 200, 150, 100, 64, 32, 16, 8}) order.push_back(k);
        } else {
            // Tight budget: still try the 350 apex once, then fall back to the
            // validated 200 baseline.
            for (int k : {350, 200, 150, 100, 64, 32, 16, 8}) order.push_back(k);
        }
        if (!validatedConfig) {
            // Unvalidated round/segment combinations have no known-safe baseline.
            // - affordable <= 1 trial: skip the speculative 500 and try the 200
            //   heuristic directly (trial still gates it);
            // - otherwise try 200 right after 500 so a tight pool yields a partial
            //   gain instead of neutral.
            std::vector<int> reordered;
            if (affordable <= 1) {
                for (int k : {200, 150, 100, 64, 32, 16, 8}) reordered.push_back(k);
            } else {
                reordered.push_back(500);
                reordered.push_back(200);
                for (int k : order) {
                    if (k == 500 || k == 200) continue;
                    reordered.push_back(k);
                }
            }
            order.swap(reordered);
        }
        int attempts = 0;
        for (int k : order) {
            if (k > wantOps || k < 8) continue;
            if (attempts >= 8) break;
            if (!trialBudgetOk()) break;
            ++attempts;
            const bool ok =
                scTrialStageInvertible(snap, target, targetHash, roundIndex, k, mesh);
            if (std::getenv("PMSF_SC_DEBUG"))
                std::fprintf(stderr, "[sc-auto-preflight] round=%d try=%d accepted=%d\n",
                             roundIndex, k, ok ? 1 : 0);
            if (ok) return k;
        }
        // Preflight false negatives are known at deep rounds (coarse base): the
        // trial clone rejects even K values that the real stage+verification
        // accepts, e.g. bunny rounds=30/50 on the final round.  Do not fall back
        // blindly to 200 (observed non-invertible: verification fails and the
        // in-place rollback cannot recover => exit(3)).  Use a conservative K
        // scaled by the live vertex count and bounded by the empirically safe
        // 150; the real post-stage verification still gates the artifact.
        // Frozen validated matrix: keep the historical 200-op baseline byte-for-
        // byte (rounds 10/30, segments 1/4/32).  This branch is part of the
        // published AUTO baseline and must not change.
        if (validatedConfig) return std::max(8, std::min(200, wantOps));

        // Unvalidated configs (notably ANY rounds > 30): preflight trials are
        // known false-negative at deep rounds (coarse base), and the historical
        // blind 200 fallback is not invertible there (verification fails and the
        // in-place rollback cannot recover => exit(3)).  Use a conservative K
        // scaled by the live vertex count (cap 150, which verified on the deep
        // final rounds of 40/50/60/100-round runs); the real post-stage
        // verification still gates the artifact.
        const long long liveV = static_cast<long long>(vCount);
        long long fallback = std::max<long long>(8, liveV / 16);
        if (fallback > 150) fallback = 150;
        if (fallback > wantOps) fallback = wantOps;
        if (std::getenv("PMSF_SC_DEBUG"))
            std::fprintf(stderr,
                         "[sc-auto-preflight] round=%d trials-rejected -> fallback K=%lld "
                         "(liveV=%lld, rounds>30 path)\n",
                         roundIndex, fallback, liveV);
        return static_cast<int>(fallback);
    }
    int lo = 0, hi = wantOps;
    for (int iter = 0; iter < 8 && lo < hi; ++iter) {
        const int mid = lo + (hi - lo + 1) / 2;
        if (scTrialStageInvertible(snap, target, targetHash, roundIndex, mid, mesh))
            lo = mid;
        else
            hi = mid - 1;
    }
    if (std::getenv("PMSF_SC_DEBUG"))
        std::fprintf(stderr, "[sc-dryrun] round=%d want=%d safe=%d\n", roundIndex, wantOps, lo);
    return lo;
}

/// Deterministically sort live faces by (groupId, clamped quantized triangle) and
/// drop removed entries from the face vector.  Encoder and decoder call this at
/// matching points so that BFS/collect order is independent of erase/append
/// history (needed when boundary stages run in non-final rounds).
inline void scCanonicalFaceSort(MCGAL::Mesh* mesh) {
    if (!mesh) return;
    const int maxQ = mesh->i_nbQuantBits > 0 ? (1 << mesh->i_nbQuantBits) - 1 : (1 << 30);
    auto cq = [&](MCGAL::Vertex* v) {
        const MCGAL::PointInt q = mesh->floatPosToInt(v->point());
        return std::array<int, 3>{std::clamp(q.x(), 0, maxQ), std::clamp(q.y(), 0, maxQ),
                                  std::clamp(q.z(), 0, maxQ)};
    };
    struct Keyed {
        MCGAL::Facet* f;
        std::array<int, 16> key;  // gid + 3 sorted vertices + 12 oriented chars
    };
    std::vector<Keyed> list;
    list.reserve(mesh->faces().size());
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        std::vector<std::array<int, 3>> vs;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && h->vertex()) vs.push_back(cq(h->vertex()));
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
        if (vs.size() != 3) continue;
        std::array<std::array<int, 3>, 3> sorted = {vs[0], vs[1], vs[2]};
        std::sort(sorted.begin(), sorted.end());
        Keyed k;
        k.f = f;
        k.key.fill(0);
        k.key[0] = f->groupId();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) k.key[1 + i * 3 + j] = sorted[i][j];
        std::size_t best = 0;
        for (std::size_t i = 1; i < vs.size(); ++i)
            if (vs[i] < vs[best]) best = i;
        for (int i = 0; i < 3; ++i) {
            const auto& q = vs[(best + i) % 3];
            for (int j = 0; j < 3; ++j) k.key[10 + i * 3 + j] = q[j];
        }
        list.push_back(k);
    }
    std::sort(list.begin(), list.end(), [](const Keyed& a, const Keyed& b) {
        return a.key < b.key;
    });
    mesh->faces().clear();
    for (auto& k : list) mesh->faces().push_back(k.f);
}

}  // namespace pmsf
