#pragma once

/// Compact anchor/traversal-style patch codec for the single-copy group-boundary
/// collapse (D184/D189).  The patch stores only:
///
///   [len][loopId][edgeIndex][gA][kA][kB][resX][resY][resZ]
///
/// Decoder re-derives (a,b) and the two fan rings purely from the current mesh
/// topology by walking the group-boundary loop and the two fan triangulations.
/// No vertex positions/ids are needed for ring lookup, which avoids the
/// position-identity drift that broke the positional prototype.

#include "SingleBoundaryCollapse.h"
#include "common/BoundaryPatchCodec.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace pmsf {

// Optional exact float position for the next recreated boundary vertex (used by
// the anchor codec to carry sub-quantization precision and avoid merge-by-
// position ambiguity at coarse quant bits).
inline thread_local bool g_scOverridePosActive = false;
inline thread_local MCGAL::PointInt g_scOverridePosQ{0, 0, 0};
inline thread_local MCGAL::Point g_scOverridePos{};
// Optional exact candidate halfedge for the next compact inverse (set by the
// group codec after topology-based loop addressing, avoiding q-collision
// ambiguity in the endpoint scan).
inline thread_local MCGAL::Halfedge* g_scForcedCandidate = nullptr;
// v2 boundary stage inverse: force the restored fan orientation recorded by the
// encoder (0/1), or -1 to let the inverse pick the first locally valid flip.
inline thread_local int g_scForceFlipA = -1;
inline thread_local int g_scForceFlipB = -1;
inline bool scPointIntEquals(const MCGAL::PointInt& a, const MCGAL::PointInt& b) {
    return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
}


struct ScLoop {
    std::vector<MCGAL::Halfedge*> edges;  // canonical directed halfedges
    int gLo = -1;
    int gHi = -1;
};

namespace scloop {

inline std::array<int, 3> qOf(MCGAL::Mesh* mesh, MCGAL::Vertex* v) {
    const MCGAL::PointInt q = mesh->floatPosToInt(v->point());
    return {q.x(), q.y(), q.z()};
}

inline std::vector<MCGAL::Halfedge*> canonicalBoundaryHalfedges(MCGAL::Mesh* mesh) {
    std::vector<MCGAL::Halfedge*> out;
    if (!mesh) return out;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
            if (h && opp && !opp->isRemoved() && opp->face() && !opp->face()->isRemoved() &&
                f->groupId() < opp->face()->groupId()) {
                out.push_back(h);  // face with the smaller group id owns the canonical direction
            }
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
    }
    return out;
}

/// Deterministically enumerate all group-boundary loops.  Loop starts/ordering
/// are canonicalised by the minimum (start q, end q) pair of the loop.
inline std::vector<ScLoop> enumerateLoops(MCGAL::Mesh* mesh) {
    std::vector<ScLoop> loops;
    if (!mesh) return loops;
    const auto canon = canonicalBoundaryHalfedges(mesh);
    std::map<std::pair<int, int>, std::vector<MCGAL::Halfedge*>> pending;
    for (MCGAL::Halfedge* h : canon) {
        if (!h || !h->vertex() || !h->end_vertex()) continue;
        if (!h->opposite() || !h->opposite()->face()) continue;
        const int gLo = h->face()->groupId();
        const int gHi = h->opposite()->face()->groupId();
        pending[{gLo, gHi}].push_back(h);
    }
    auto keyOf = [&](MCGAL::Halfedge* h) {
        const auto qa = qOf(mesh, h->vertex());
        const auto qb = qOf(mesh, h->end_vertex());
        return std::array<int, 6>{qa[0], qa[1], qa[2], qb[0], qb[1], qb[2]};
    };
    for (auto& kv : pending) {
        auto& edges = kv.second;
        std::set<MCGAL::Halfedge*> used;
        for (MCGAL::Halfedge* seed : edges) {
            if (!seed || used.count(seed)) continue;
            // walk both directions from seed via 'next edge sharing the end vertex'
            std::vector<MCGAL::Halfedge*> loop;
            std::set<MCGAL::Halfedge*> local;
            MCGAL::Halfedge* cur = seed;
            int guard = 0;
            while (cur && !local.count(cur) && ++guard < 100000) {
                local.insert(cur);
                loop.push_back(cur);
                MCGAL::Vertex* v = cur->end_vertex();
                MCGAL::Halfedge* next = nullptr;
                for (MCGAL::Halfedge* cand : edges) {
                    if (!cand || cand == cur || cand->isRemoved()) continue;
                    if (cand->vertex() != v) continue;
                    if (next) {
                        // junction: deterministic min next endpoint
                        const auto qn = qOf(mesh, next->end_vertex());
                        const auto qc = qOf(mesh, cand->end_vertex());
                        if (std::array<int, 3>{qc[0], qc[1], qc[2]} <
                            std::array<int, 3>{qn[0], qn[1], qn[2]})
                            next = cand;
                    } else {
                        next = cand;
                    }
                }
                if (!next || next == seed || local.count(next)) break;
                cur = next;
            }
            if (loop.size() < 3) continue;
            // rotate so the loop starts at its canonical minimum edge
            std::size_t best = 0;
            std::array<int, 6> bestKey = keyOf(loop[0]);
            for (std::size_t i = 1; i < loop.size(); ++i) {
                const auto k = keyOf(loop[i]);
                if (k < bestKey) { bestKey = k; best = i; }
            }
            std::vector<MCGAL::Halfedge*> rotated;
            rotated.reserve(loop.size());
            for (std::size_t i = 0; i < loop.size(); ++i)
                rotated.push_back(loop[(best + i) % loop.size()]);
            for (MCGAL::Halfedge* h : rotated) used.insert(h);
            ScLoop l;
            l.edges = std::move(rotated);
            l.gLo = kv.first.first;
            l.gHi = kv.first.second;
            loops.push_back(std::move(l));
        }
    }
    // deterministic loop order by canonical start edge
    std::sort(loops.begin(), loops.end(), [&](const ScLoop& a, const ScLoop& b) {
        return keyOf(a.edges[0]) < keyOf(b.edges[0]);
    });
    return loops;
}


}  // namespace scloop

/// Create a float position whose floatPosToInt() is exactly the requested q.
/// intPosToFloat(q) can round-trip to q-1 for some q due to float32 truncation;
/// recreated vertices must key back to their stored q so position-based loop
/// ordering stays identical between encoder and decoder states.
inline MCGAL::Point scStableFloatForQ(MCGAL::Mesh* mesh, const MCGAL::PointInt& qIn) {
    // The quantized range is [0, 2^bits-1]; the encoder-side floatPosToInt can
    // produce 2^bits at the bbox max (float truncation), which the base writer
    // clamps.  Restored vertices must be clamped the same way.
    MCGAL::PointInt q = qIn;
    if (mesh->i_nbQuantBits > 0) {
        const int maxQ = (1 << mesh->i_nbQuantBits) - 1;
        q[0] = std::clamp(q[0], 0, maxQ);
        q[1] = std::clamp(q[1], 0, maxQ);
        q[2] = std::clamp(q[2], 0, maxQ);
    }
    MCGAL::Point out = mesh->intPosToFloat(q);
    const float step = mesh->f_quantStep;
    auto resolve = [&](float base, int target, int axis) -> float {
        for (int i = 1; i <= 4; ++i) {
            for (float sign : {1.0f, -1.0f}) {
                MCGAL::Point p = out;
                p[axis] = base + sign * (0.25f * static_cast<float>(i)) * step;
                const MCGAL::PointInt pq = mesh->floatPosToInt(p);
                const int got = (axis == 0) ? pq.x() : (axis == 1) ? pq.y() : pq.z();
                if (got == target) return p[axis];
            }
        }
        return out[axis];
    };
    out[0] = resolve(out[0], q.x(), 0);
    out[1] = resolve(out[1], q.y(), 1);
    out[2] = resolve(out[2], q.z(), 2);
    return out;
}

/// Capture a compact patch from the already-applied pointer patch.
inline std::vector<int> captureSingleBoundaryPatchCompact(MCGAL::Mesh* mesh,
                                                         const SingleBoundaryPatch& patch) {
    std::vector<int> out;
    auto fail = [&](const char* why) -> std::vector<int> {
        if (std::getenv("PMSF_SC_DEBUG"))
            std::fprintf(stderr, "[sc-compact-capture-fail] %s (added=%zu)\n", why,
                         patch.addedFaces.size());
        return {};
    };
    if (!mesh || !patch.valid || !patch.removed || patch.addedFaces.size() < 2)
        return fail("invalid patch");
    const int gA = patch.addedFaces.front().gid;
    int nA = 0;  // number of NEW A-side fan triangles
    for (const auto& f : patch.addedFaces) if (f.gid == gA) ++nA;
    const int nB = static_cast<int>(patch.addedFaces.size()) - nA;
    if (nA < 1 || nB < 1) return fail("nA/nB");
    const int kA = nA + 1;  // old A-side fan face count
    const int kB = nB + 1;
    auto buildRing = [&](int startFace, int count) -> std::vector<MCGAL::Vertex*> {
        std::vector<MCGAL::Vertex*> ring;
        const auto& f0 = patch.addedFaces[static_cast<std::size_t>(startFace)];
        ring.push_back(f0.v[0]);
        ring.push_back(f0.v[1]);
        ring.push_back(f0.v[2]);
        for (int i = 1; i < count; ++i) {
            ring.push_back(patch.addedFaces[static_cast<std::size_t>(startFace + i)].v[2]);
        }
        return ring;
    };
    std::vector<MCGAL::Vertex*> ringA = buildRing(0, nA);
    std::vector<MCGAL::Vertex*> ringB = buildRing(nA, nB);
    if (ringA.size() != static_cast<std::size_t>(kA + 1) ||
        ringB.size() != static_cast<std::size_t>(kB + 1))
        return fail("ring size");
    if (ringB.front() != ringA.back() || ringB.back() != ringA.front())
        return fail("ring endpoints");

    // predicted removed position: average of ring vertex coordinates
    std::vector<MCGAL::Vertex*> ringVerts;
    std::unordered_set<MCGAL::Vertex*> seen;
    for (MCGAL::Vertex* v : ringA) if (v && seen.insert(v).second) ringVerts.push_back(v);
    for (MCGAL::Vertex* v : ringB) if (v && seen.insert(v).second) ringVerts.push_back(v);
    long long sx = 0, sy = 0, sz = 0;
    for (MCGAL::Vertex* v : ringVerts) {
        const auto q = scloop::qOf(mesh, v);
        sx += q[0]; sy += q[1]; sz += q[2];
    }
    const int n = static_cast<int>(ringVerts.size());
    if (n <= 0) return fail("ring empty");
    const int px = static_cast<int>(sx / n), py = static_cast<int>(sy / n),
              pz = static_cast<int>(sz / n);
    const auto rq = scloop::qOf(mesh, patch.removed);

    if (ringA.front() != ringB.back() || ringA.back() != ringB.front())
        return fail("ring endpoints 2");
    const auto aq = scloop::qOf(mesh, ringA.front());
    const auto bq = scloop::qOf(mesh, ringA.back());
    // Local fingerprint: the third vertices of the last A/B fan triangles.
    // They disambiguate boundary edges whose endpoints are within locator
    // tolerance of each other (silent wrong-candidate matches).
    MCGAL::Vertex* pThird = patch.addedFaces[static_cast<std::size_t>(nA - 1)].v[1];
    MCGAL::Vertex* rThird = patch.addedFaces[static_cast<std::size_t>(nA + nB - 1)].v[1];
    if (!pThird || !rThird) return fail("fingerprint vertices");
    const auto ptq = scloop::qOf(mesh, pThird);
    const auto rtq = scloop::qOf(mesh, rThird);
    out.push_back(18);  // payload length (ints after this one)
    out.push_back(aq[0]); out.push_back(aq[1]); out.push_back(aq[2]);
    out.push_back(bq[0]); out.push_back(bq[1]); out.push_back(bq[2]);
    out.push_back(gA);
    out.push_back(kA);
    out.push_back(kB);
    // Store the removed vertex position absolutely: same 3 ints as a residual
    // but exact on decode (no ring-q drift accumulating into vertex positions).
    out.push_back(rq[0]);
    out.push_back(rq[1]);
    out.push_back(rq[2]);
    out.push_back(ptq[0]); out.push_back(ptq[1]); out.push_back(ptq[2]);
    out.push_back(rtq[0]); out.push_back(rtq[1]); out.push_back(rtq[2]);
    // Optional order-restoration metadata: pre-collapse ranks of the removed vertex and
    // of every old fan face, emitted in the exact order the inverse recreates them
    // (A fan pair-by-pair, then B fan).  Restoring these positions makes the next
    // round's symbol replay see the encoder's container order again.
    if (std::getenv("PMSF_SC_QLOC_ORDER") && patch.removedVertexRank >= 0 &&
        patch.ringA.size() >= 2 && patch.ringB.size() >= 2) {
        std::vector<int> ranks;
        bool ok = true;
        auto addFaceRank = [&](MCGAL::Vertex* x, MCGAL::Vertex* y) {
            for (const auto& rec : patch.origFaces) {
                const auto has = [&](MCGAL::Vertex* z) {
                    return z && (rec.v[0] == z || rec.v[1] == z || rec.v[2] == z);
                };
                if (has(patch.removed) && has(x) && has(y)) {
                    if (rec.rank < 0) { ok = false; return; }
                    ranks.push_back(rec.rank);
                    return;
                }
            }
            ok = false;
        };
        for (std::size_t i = 0; i + 1 < patch.ringA.size() && ok; ++i)
            addFaceRank(patch.ringA[i], patch.ringA[i + 1]);
        for (std::size_t i = 0; i + 1 < patch.ringB.size() && ok; ++i)
            addFaceRank(patch.ringB[i], patch.ringB[i + 1]);
        if (ok && static_cast<int>(ranks.size()) == kA + kB) {
            out[0] = 19 + static_cast<int>(ranks.size());
            out.push_back(patch.removedVertexRank);
            for (int rk : ranks) out.push_back(rk);
        }
    }
    return out;
}

/// Undo one compact patch.  Rings are re-derived via deterministic boundary-loop
/// traversal + fan walks; no position/id lookup is involved.
inline bool applySingleBoundaryCompactPatchInverse(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                                  const std::vector<int>& data,
                                                  std::size_t& cursor,
                                                  std::vector<MCGAL::Facet*>* outRestored = nullptr,
                                                  MCGAL::Vertex** outRemoved = nullptr) {
    if (!mesh || cursor >= data.size()) return false;
    const int len = data[cursor++];
    if (len != 18 || cursor + 18 > data.size()) return false;
    const MCGAL::PointInt aq{data[cursor], data[cursor + 1], data[cursor + 2]};
    cursor += 3;
    const MCGAL::PointInt bq{data[cursor], data[cursor + 1], data[cursor + 2]};
    cursor += 3;
    const int gA = data[cursor++];
    const int kA = data[cursor++];
    const int kB = data[cursor++];
    const int rx = data[cursor++];
    const int ry = data[cursor++];
    const int rz = data[cursor++];
    const MCGAL::PointInt ptq{data[cursor], data[cursor + 1], data[cursor + 2]};
    cursor += 3;
    const MCGAL::PointInt rtq{data[cursor], data[cursor + 1], data[cursor + 2]};
    cursor += 3;
    if (kA < 1 || kB < 1) return false;
    const MCGAL::PointInt rq{rx, ry, rz};
    const int locTol = []() {
        const char* v = std::getenv("PMSF_SC_LOCATOR_TOL");
        return v ? std::atoi(v) : 1;
    }();
    const int maxTries = []() {
        const char* v = std::getenv("PMSF_SC_INVERSE_TRIES");
        return v ? std::max(1, std::atoi(v)) : 8;
    }();
    auto fail = [&](const char* why) -> bool {
        if (std::getenv("PMSF_SC_DEBUG"))
            std::fprintf(stderr,
                         "[sc-compact-fail] %s aq=(%d,%d,%d) bq=(%d,%d,%d) gA=%d kA=%d kB=%d\n",
                         why, aq.x(), aq.y(), aq.z(), bq.x(), bq.y(), bq.z(), gA, kA, kB);
        return false;
    };
    auto qDiff = [](const MCGAL::PointInt& p, const MCGAL::PointInt& q) {
        return std::max({std::abs(p.x() - q.x()), std::abs(p.y() - q.y()),
                         std::abs(p.z() - q.z())});
    };
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
    auto triOf = [](MCGAL::Facet* f) {
        std::array<MCGAL::Vertex*, 3> t{nullptr, nullptr, nullptr};
        if (!f || !f->proxyHalfedge()) return t;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int k = 0, guard = 0;
        do {
            if (h && h->vertex() && k < 3) t[k++] = h->vertex();
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
        return t;
    };
    auto walkFan = [&](MCGAL::Halfedge* lastHalf, MCGAL::Vertex* apex, MCGAL::Vertex* other,
                       int fanFaces, std::vector<MCGAL::Vertex*>& ring,
                       std::vector<MCGAL::Facet*>& faces) -> bool {
        ring.clear();
        faces.clear();
        if (fanFaces == 0) {
            // Corner case: this side contributed no new fan faces; the ring is just
            // the apex endpoint (the caller appends the other endpoint).
            ring.push_back(apex);
            return true;
        }
        if (!lastHalf || !lastHalf->face() || fanFaces < 1) {
            if (std::getenv("PMSF_SC_DEBUG"))
                std::fprintf(stderr, "[sc-walk-fail] early lastHalf=%p face=%p fanFaces=%d\n",
                             (void*)lastHalf, lastHalf && lastHalf->face() ? (void*)lastHalf->face() : nullptr,
                             fanFaces);
            return false;
        }
        MCGAL::Facet* f = lastHalf->face();
        faces.push_back(f);
        MCGAL::Vertex* p = thirdVertex(lastHalf, apex, other);
        if (!p) {
            if (std::getenv("PMSF_SC_DEBUG"))
                std::fprintf(stderr, "[sc-walk-fail] no third apex=%p other=%p faceG=%d\n",
                             (void*)apex, (void*)other, f->groupId());
            return false;
        }
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
            if (!hedge) {
                if (std::getenv("PMSF_SC_DEBUG"))
                    std::fprintf(stderr, "[sc-walk-fail] step=%d no hedge faceG=%d\n", step,
                                 f->groupId());
                return false;
            }
            MCGAL::Halfedge* opp = hedge->opposite();
            if (!opp || opp->isRemoved() || !opp->face() || opp->face()->isRemoved()) {
                if (std::getenv("PMSF_SC_DEBUG"))
                    std::fprintf(stderr, "[sc-walk-fail] step=%d bad opp faceG=%d\n", step,
                                 f->groupId());
                return false;
            }
            f = opp->face();
            faces.push_back(f);
            MCGAL::Vertex* nxt = thirdVertex(f->proxyHalfedge(), apex, collected.back());
            if (!nxt || nxt == apex || nxt == other) {
                if (std::getenv("PMSF_SC_DEBUG"))
                    std::fprintf(stderr, "[sc-walk-fail] step=%d bad nxt faceG=%d\n", step,
                                 f->groupId());
                return false;
            }
            collected.push_back(nxt);
        }
        std::reverse(collected.begin(), collected.end());
        ring.push_back(apex);
        for (MCGAL::Vertex* x : collected) ring.push_back(x);
        return true;
    };

    struct Candidate {
        MCGAL::Vertex* a = nullptr;
        MCGAL::Vertex* b = nullptr;
        int gB = -1;
        std::vector<MCGAL::Vertex*> ringA;
        std::vector<MCGAL::Vertex*> ringB;
        std::vector<std::array<MCGAL::Vertex*, 3>> fanA;
        std::vector<std::array<MCGAL::Vertex*, 3>> fanB;
        long long dist = 0;
    };
    std::vector<Candidate> cands;
    int diagNearQ = 0, diagWalkFail = 0, diagThirdFail = 0, diagFpFail = 0;
    const bool candDbg = std::getenv("PMSF_SC_DEBUG") != nullptr;
    auto buildCandidate = [&](MCGAL::Halfedge* h) {
        MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
        if (!h || !h->vertex() || !h->end_vertex() || h->vertex() == h->end_vertex() || !opp ||
            opp->isRemoved() || !opp->face() || opp->face()->isRemoved() ||
            opp->face()->groupId() == gA || !h->face() || h->face()->isRemoved() ||
            h->face()->groupId() != gA) {
            if (candDbg && g_scForcedCandidate)
                std::fprintf(stderr,
                             "[sc-force-bad-basic] h=%p kA=%d kB=%d gA=%d hG=%d oG=%d hRem=%d "
                             "oRem=%d v=%p ev=%p\n",
                             (void*)h, kA, kB, gA,
                             (h && h->face() && !h->face()->isRemoved()) ? h->face()->groupId() : -999,
                             (opp && opp->face() && !opp->face()->isRemoved())
                                 ? opp->face()->groupId()
                                 : -999,
                             (h && h->face()) ? (h->face()->isRemoved() ? 1 : 0) : -1,
                             (opp && opp->face()) ? (opp->face()->isRemoved() ? 1 : 0) : -1,
                             h ? (void*)h->vertex() : nullptr, h ? (void*)h->end_vertex() : nullptr);
            return;
        }
        Candidate cand;
        cand.a = h->end_vertex();
        cand.b = h->vertex();
        cand.gB = opp->face()->groupId();
        std::vector<MCGAL::Facet*> facesA, facesB;
        const bool wa = walkFan(h, cand.a, cand.b, kA - 1, cand.ringA, facesA);
        const bool wb = wa && walkFan(opp, cand.b, cand.a, kB - 1, cand.ringB, facesB);
        if (!wa || !wb) {
            ++diagWalkFail;
            if (candDbg && g_scForcedCandidate)
                std::fprintf(stderr, "[sc-force-walk] wa=%d wb=%d kA=%d kB=%d gA=%d\n", (int)wa,
                             (int)wb, kA, kB, gA);
            return;
        }
        MCGAL::Vertex* pThirdC = thirdVertex(h, cand.a, cand.b);
        MCGAL::Vertex* rThirdC = thirdVertex(opp, cand.b, cand.a);
        if (!pThirdC || !rThirdC) {
            ++diagThirdFail;
            if (candDbg && g_scForcedCandidate)
                std::fprintf(stderr, "[sc-force-third] p=%p r=%p kA=%d kB=%d\n", (void*)pThirdC,
                             (void*)rThirdC, kA, kB);
            return;
        }
        const MCGAL::PointInt pq = mesh->floatPosToInt(pThirdC->point());
        const MCGAL::PointInt rq2 = mesh->floatPosToInt(rThirdC->point());
        const bool needFingerprint = ptq.x() >= 0 || rtq.x() >= 0;
        if (needFingerprint && (qDiff(pq, ptq) > locTol || qDiff(rq2, rtq) > locTol)) {
            ++diagFpFail;
            if (candDbg && g_scForcedCandidate)
                std::fprintf(stderr,
                             "[sc-force-fp] kA=%d kB=%d gA=%d pq=(%d,%d,%d) ptq=(%d,%d,%d) "
                             "diff=%d/%d\n",
                             kA, kB, gA, pq.x(), pq.y(), pq.z(), ptq.x(), ptq.y(), ptq.z(),
                             qDiff(pq, ptq), qDiff(rq2, rtq));
            return;
        }
        std::vector<MCGAL::Vertex*> unionRing;
        std::unordered_set<MCGAL::Vertex*> seen;
        for (MCGAL::Vertex* x : cand.ringA)
            if (x && seen.insert(x).second) unionRing.push_back(x);
        for (MCGAL::Vertex* x : cand.ringB)
            if (x && seen.insert(x).second) unionRing.push_back(x);
        long long sx = 0, sy = 0, sz = 0;
        for (MCGAL::Vertex* x : unionRing) {
            const MCGAL::PointInt q = mesh->floatPosToInt(x->point());
            sx += q.x(); sy += q.y(); sz += q.z();
        }
        const int n = static_cast<int>(unionRing.size());
        if (n <= 0) return;
        const long long cdist = std::max({std::abs(sx / n - rq.x()), std::abs(sy / n - rq.y()),
                                          std::abs(sz / n - rq.z())});
        const MCGAL::PointInt qa = mesh->floatPosToInt(cand.a->point());
        const MCGAL::PointInt qb = mesh->floatPosToInt(cand.b->point());
        const long long edist = std::max(
            {std::abs(qa.x() - rq.x()), std::abs(qa.y() - rq.y()), std::abs(qa.z() - rq.z()),
             std::abs(qb.x() - rq.x()), std::abs(qb.y() - rq.y()), std::abs(qb.z() - rq.z())});
        cand.dist = std::max(cdist, edist);
        for (MCGAL::Facet* ff : facesA) cand.fanA.push_back(triOf(ff));
        for (MCGAL::Facet* ff : facesB) cand.fanB.push_back(triOf(ff));
        cands.push_back(std::move(cand));
    };
    if (g_scForcedCandidate) {
        buildCandidate(g_scForcedCandidate);
    } else {
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved() || !f->proxyHalfedge() || f->groupId() != gA) continue;
            MCGAL::Halfedge* h0 = f->proxyHalfedge();
            MCGAL::Halfedge* h = h0;
            int guard = 0;
            do {
                MCGAL::Halfedge* opp = h ? h->opposite() : nullptr;
                if (h && h->vertex() && h->end_vertex() && h->vertex() != h->end_vertex() && opp &&
                    !opp->isRemoved() && opp->face() && !opp->face()->isRemoved() &&
                    opp->face()->groupId() != gA) {
                    const MCGAL::PointInt qv = mesh->floatPosToInt(h->vertex()->point());
                    const MCGAL::PointInt qe = mesh->floatPosToInt(h->end_vertex()->point());
                    if (qDiff(qv, bq) <= locTol && qDiff(qe, aq) <= locTol) {
                        ++diagNearQ;
                        buildCandidate(h);
                    }
                }
                h = h ? h->next() : nullptr;
            } while (h && h != h0 && ++guard < 64);
        }
    }
    if (cands.empty()) {
        if (std::getenv("PMSF_SC_DEBUG"))
            std::fprintf(stderr,
                         "[sc-cand-diag] aq=(%d,%d,%d) bq=(%d,%d,%d) gA=%d kA=%d kB=%d tol=%d "
                         "nearQ=%d walkFail=%d thirdFail=%d fpFail=%d\n",
                         aq.x(), aq.y(), aq.z(), bq.x(), bq.y(), bq.z(), gA, kA, kB, locTol,
                         diagNearQ, diagWalkFail, diagThirdFail, diagFpFail);
        return fail("no boundary-edge candidate");
    }
    std::sort(cands.begin(), cands.end(),
              [](const Candidate& x, const Candidate& y) { return x.dist < y.dist; });
    const bool useStableFloat = std::getenv("PMSF_SC_NO_STABLE_FLOAT") == nullptr;

    auto restoreFanFromTriples = [&](const std::vector<std::array<MCGAL::Vertex*, 3>>& fan, int gid) {
        for (const auto& t : fan) {
            if (!t[0] || !t[1] || !t[2]) continue;
            createFaceFromTripleManual(mesh, t[0], t[1], t[2], gid);
        }
    };
    auto validateLocal = [&](MCGAL::Vertex* removed, const std::vector<MCGAL::Vertex*>& fullA,
                             const std::vector<MCGAL::Vertex*>& fullB, int gBv,
                             const std::vector<MCGAL::Facet*>& restoredA,
                             const std::vector<MCGAL::Facet*>& restoredB, int flipA,
                             int flipB) -> bool {
        int d = 0, nAfaces = 0, nBfaces = 0, nBoundary = 0;
        for (MCGAL::Halfedge* h : pmsf::boundedRing(removed)) {
            if (!h || h->isRemoved()) continue;
            ++d;
            if (!h->face() || h->face()->isRemoved()) return false;
            MCGAL::Halfedge* opp = h->opposite();
            if (!opp || opp->isRemoved() || !opp->face() || opp->face()->isRemoved()) return false;
            const int fg = h->face()->groupId();
            const int og = opp->face()->groupId();
            if (fg == gA) ++nAfaces;
            else if (fg == gBv) ++nBfaces;
            else return false;
            if (fg != og) {
                ++nBoundary;
                if (!((fg == gA && og == gBv) || (fg == gBv && og == gA))) return false;
            }
        }
        if (d != kA + kB || nAfaces != kA || nBfaces != kB || nBoundary != 2) {
            if (std::getenv("PMSF_SC_DEBUG"))
                std::fprintf(stderr,"[sc-val] counts d=%d/%d nA=%d/%d nB=%d/%d nb=%d\n", d, kA+kB,
                             nAfaces, kA, nBfaces, kB, nBoundary);
            return false;
        }
        auto checkOriented = [&](const std::vector<MCGAL::Vertex*>& full, int gid,
                                 const std::vector<MCGAL::Facet*>& restored, int flip) -> bool {
            if (restored.size() + 1 != full.size()) return false;
            for (std::size_t i = 0; i + 1 < full.size(); ++i) {
                MCGAL::Halfedge* want = nullptr;
                MCGAL::Vertex* uw = flip ? full[i + 1] : full[i];
                MCGAL::Vertex* vw = flip ? full[i] : full[i + 1];
                MCGAL::Facet* f = restored[i];
                if (!f || f->isRemoved() || !f->proxyHalfedge()) return false;
                MCGAL::Halfedge* h0 = f->proxyHalfedge();
                MCGAL::Halfedge* h = h0;
                int guard = 0;
                do {
                    if (h && h->vertex() == uw && h->end_vertex() == vw) {
                        want = h;
                        break;
                    }
                    h = h ? h->next() : nullptr;
                } while (h && h != h0 && ++guard < 64);
                if (!want) {
                    if (std::getenv("PMSF_SC_DEBUG"))
                        std::fprintf(stderr,"[sc-val] orient gid=%d flip=%d i=%zu no halfedge\n", gid,
                                     flip, i);
                    return false;
                }
                MCGAL::Halfedge* opp = want->opposite();
                if (!opp || opp->isRemoved() || !opp->face() || opp->face()->isRemoved()) {
                    if (std::getenv("PMSF_SC_DEBUG"))
                        std::fprintf(stderr,"[sc-val] orient gid=%d flip=%d i=%zu no opp\n", gid,
                                     flip, i);
                    return false;
                }
                (void)gid;  // ring edge may itself be a group-boundary edge
            }
            return true;
        };
        return checkOriented(fullA, gA, restoredA, flipA) &&
               checkOriented(fullB, gBv, restoredB, flipB);
    };

    int tries = 0;
    for (const Candidate& cand : cands) {
        if (++tries > maxTries) break;
        std::vector<MCGAL::Vertex*> fullA = cand.ringA;
        std::vector<MCGAL::Vertex*> fullB = cand.ringB;
        fullA.push_back(cand.b);
        fullB.push_back(cand.a);
        for (int flipA = (g_scForceFlipA >= 0 ? g_scForceFlipA : 0);
             flipA < (g_scForceFlipA >= 0 ? g_scForceFlipA + 1 : 2); ++flipA) {
            for (int flipB = (g_scForceFlipB >= 0 ? g_scForceFlipB : 0);
                 flipB < (g_scForceFlipB >= 0 ? g_scForceFlipB + 1 : 2); ++flipB) {
                for (const auto& fan : cand.fanA)
                    for (MCGAL::Vertex* x : fan)
                        if (!x || x->isRemoved()) return fail("bad candidate fan");
                // remove current fan faces
                for (const auto& t : cand.fanA)
                    if (t[0] && t[1] && t[2]) {
                        for (MCGAL::Facet* f : mesh->faces()) {
                            if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                            const auto fs = sbdetail::faceVertices(f);
                            if (!fs[0]) continue;
                            auto has = [&](MCGAL::Vertex* x) {
                                return fs[0] == x || fs[1] == x || fs[2] == x;
                            };
                            if (has(t[0]) && has(t[1]) && has(t[2]) &&
                                f->groupId() == gA) {
                                removeFaceForPatch(mesh, f);
                                break;
                            }
                        }
                    }
                for (const auto& t : cand.fanB)
                    if (t[0] && t[1] && t[2]) {
                        for (MCGAL::Facet* f : mesh->faces()) {
                            if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                            const auto fs = sbdetail::faceVertices(f);
                            if (!fs[0]) continue;
                            auto has = [&](MCGAL::Vertex* x) {
                                return fs[0] == x || fs[1] == x || fs[2] == x;
                            };
                            if (has(t[0]) && has(t[1]) && has(t[2]) &&
                                f->groupId() == cand.gB) {
                                removeFaceForPatch(mesh, f);
                                break;
                            }
                        }
                    }
                MCGAL::Point removedPos = useStableFloat ? scStableFloatForQ(mesh.get(), rq)
                                                         : mesh->intPosToFloat(rq);
                if (g_scOverridePosActive && scPointIntEquals(g_scOverridePosQ, rq))
                    removedPos = g_scOverridePos;
                MCGAL::Vertex* removed =
                    MCGAL::contextPool.allocateVertexFromPool(mesh->meshId(), removedPos);
                if (!removed) return fail("alloc vertex");
                removed->halfedges().clear();
                removed->setGroupId(gA);
                mesh->vertices().push_back(removed);
                std::vector<MCGAL::Facet*> restoredA, restoredB;
                bool createOk = true;
                for (int i = 0; i + 1 < static_cast<int>(fullA.size()) && createOk; ++i) {
                    MCGAL::Vertex *v0 = fullA[i], *v1 = fullA[i + 1];
                    if (flipA) std::swap(v0, v1);
                    MCGAL::Facet* nf = createFaceFromTripleManual(mesh, removed, v0, v1, gA);
                    if (!nf) createOk = false;
                    else restoredA.push_back(nf);
                }
                for (int i = 0; i + 1 < static_cast<int>(fullB.size()) && createOk; ++i) {
                    MCGAL::Vertex *v0 = fullB[i], *v1 = fullB[i + 1];
                    if (flipB) std::swap(v0, v1);
                    MCGAL::Facet* nf = createFaceFromTripleManual(mesh, removed, v0, v1, cand.gB);
                    if (!nf) createOk = false;
                    else restoredB.push_back(nf);
                }
                pmsf::rebuildVertexHalfedgeRings(mesh);
                if (createOk) pmsf::repairHalfedgePairing(mesh);
                const bool validLocal =
                    createOk && validateLocal(removed, fullA, fullB, cand.gB, restoredA, restoredB,
                                              flipA, flipB);
                if (!validLocal && std::getenv("PMSF_SC_DEBUG")) {
                    std::fprintf(stderr,
                                 "[sc-compact-invalid] createOk=%d rA=%zu/%zu rB=%zu/%zu fullA=%zu fullB=%zu gA=%d gB=%d flipA=%d flipB=%d\n",
                                 createOk ? 1 : 0, restoredA.size(), fullA.size() - 1,
                                 restoredB.size(), fullB.size() - 1, fullA.size(), fullB.size(), gA,
                                 cand.gB, flipA, flipB);
                    auto dump = [&](const char* tag, const std::vector<MCGAL::Vertex*>& ring) {
                        std::fprintf(stderr, "  %s:", tag);
                        for (MCGAL::Vertex* x : ring) {
                            const MCGAL::PointInt q = mesh->floatPosToInt(x->point());
                            std::fprintf(stderr, " (%d,%d,%d)", q.x(), q.y(), q.z());
                        }
                        std::fprintf(stderr, "\n");
                    };
                    dump("fullA", fullA);
                    dump("fullB", fullB);
                    for (std::size_t fi = 0; fi < restoredA.size(); ++fi) {
                        const auto t = triOf(restoredA[fi]);
                        std::fprintf(stderr, "  rA[%zu] gid=%d:", fi, restoredA[fi]->groupId());
                        for (MCGAL::Vertex* x : t) {
                            const MCGAL::PointInt q = mesh->floatPosToInt(x->point());
                            std::fprintf(stderr, " (%d,%d,%d)", q.x(), q.y(), q.z());
                        }
                        std::fprintf(stderr, "\n");
                    }
                }
                if (validLocal) {
                    if (std::getenv("PMSF_SC_DEBUG"))
                        std::fprintf(stderr,
                                     "[sc-compact-pick] ok try=%d dist=%lld gB=%d flipA=%d flipB=%d\n",
                                     tries, cand.dist, cand.gB, flipA, flipB);
                    if (outRestored) {
                        outRestored->clear();
                        outRestored->insert(outRestored->end(), restoredA.begin(), restoredA.end());
                        outRestored->insert(outRestored->end(), restoredB.begin(), restoredB.end());
                    }
                    if (outRemoved) *outRemoved = removed;
                    return true;
                }
                // rollback
                for (MCGAL::Facet* f : restoredA)
                    if (f && !f->isRemoved()) removeFaceForPatch(mesh, f);
                for (MCGAL::Facet* f : restoredB)
                    if (f && !f->isRemoved()) removeFaceForPatch(mesh, f);
                for (std::size_t vi = 0; vi < mesh->vertices().size(); ++vi) {
                    if (mesh->vertices()[vi] == removed) {
                        mesh->vertices().erase(mesh->vertices().begin() +
                                               static_cast<std::ptrdiff_t>(vi));
                        break;
                    }
                }
                restoreFanFromTriples(cand.fanA, gA);
                restoreFanFromTriples(cand.fanB, cand.gB);
                pmsf::rebuildVertexHalfedgeRings(mesh);
            }
        }
        if (std::getenv("PMSF_SC_DEBUG"))
            std::fprintf(stderr, "[sc-compact-retry] try=%d dist=%lld all flips failed\n", tries,
                         cand.dist);
    }
    return fail("all candidates failed local validation");
}

/// Ordered variant of the compact inverse: payload = len(=19+nRanks) [18 compact ints]
/// [removedVertexRank][restored face ranks in creation order].  After a successful
/// inverse the removed vertex and restored fan faces are re-inserted at their
/// pre-collapse container positions so following rounds replay the encoder order.
inline bool applySingleBoundaryCompactPatchOrderedInverse(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                                          const std::vector<int>& data,
                                                          std::size_t& cursor,
                                                          MCGAL::Vertex** outRemoved = nullptr) {
    if (!mesh || cursor >= data.size()) return false;
    const std::size_t start = cursor;
    const int len = data[cursor++];
    if (len < 19 || cursor + static_cast<std::size_t>(len) > data.size()) return false;
    const int nRanks = len - 19;
    std::vector<int> compact;
    compact.reserve(18);
    for (int k = 0; k < 18; ++k) compact.push_back(data[cursor + static_cast<std::size_t>(k)]);
    const int vRank = data[cursor + 18];
    std::vector<int> ranks;
    ranks.reserve(static_cast<std::size_t>(nRanks));
    for (int k = 0; k < nRanks; ++k) ranks.push_back(data[cursor + 19 + static_cast<std::size_t>(k)]);
    std::size_t c = 0;
    std::vector<MCGAL::Facet*> restored;
    MCGAL::Vertex* removed = nullptr;
    if (!applySingleBoundaryCompactPatchInverse(mesh, compact, c, &restored, &removed)) {
        cursor = start;
        return false;
    }
    if (outRemoved) *outRemoved = removed;
    cursor = start + 1 + static_cast<std::size_t>(len);
    if (!removed || restored.size() != ranks.size()) return true;
    {
        std::vector<MCGAL::Vertex*> live, tombstones;
        for (MCGAL::Vertex* v : mesh->vertices()) {
            if (!v) continue;
            if (v->isRemoved()) tombstones.push_back(v);
            else live.push_back(v);
        }
        live.erase(std::remove(live.begin(), live.end(), removed), live.end());
        int pos = std::clamp(vRank, 0, static_cast<int>(live.size()));
        live.insert(live.begin() + pos, removed);
        live.insert(live.end(), tombstones.begin(), tombstones.end());
        mesh->vertices().swap(live);
    }
    {
        std::vector<MCGAL::Facet*> live, tombstones;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f) continue;
            if (f->isRemoved()) tombstones.push_back(f);
            else live.push_back(f);
        }
        std::unordered_set<MCGAL::Facet*> rset(restored.begin(), restored.end());
        live.erase(std::remove_if(live.begin(), live.end(),
                                  [&](MCGAL::Facet* f) { return rset.count(f) != 0; }),
                   live.end());
        std::vector<std::pair<int, MCGAL::Facet*>> ins;
        ins.reserve(restored.size());
        for (std::size_t i = 0; i < restored.size(); ++i) ins.emplace_back(ranks[i], restored[i]);
        std::sort(ins.begin(), ins.end(), [](const std::pair<int, MCGAL::Facet*>& a,
                                             const std::pair<int, MCGAL::Facet*>& b) {
            if (a.first != b.first) return a.first < b.first;
            return a.second < b.second;
        });
        int used = 0;
        for (const auto& pr : ins) {
            int idx = pr.first - used;
            if (idx < 0) idx = 0;
            if (idx > static_cast<int>(live.size())) idx = static_cast<int>(live.size());
            live.insert(live.begin() + idx, pr.second);
            ++used;
        }
        live.insert(live.end(), tombstones.begin(), tombstones.end());
        mesh->faces().swap(live);
    }
    return true;
}
inline std::string scCompactSignature(MCGAL::Mesh* mesh) {
    if (!mesh) return "";
    std::vector<std::array<int, 3>> verts;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        const MCGAL::PointInt q = mesh->floatPosToInt(v->point());
        verts.push_back({q.x(), q.y(), q.z()});
    }
    std::sort(verts.begin(), verts.end());
    std::vector<std::array<int, 10>> faces;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        const auto vs = sbdetail::faceVertices(f);
        if (!vs[0]) continue;
        std::array<std::array<int, 3>, 3> p{};
        for (int k = 0; k < 3; ++k) {
            const MCGAL::PointInt q = mesh->floatPosToInt(vs[k]->point());
            p[k] = {q.x(), q.y(), q.z()};
        }
        std::sort(p.begin(), p.end());
        faces.push_back({p[0][0], p[0][1], p[0][2], p[1][0], p[1][1], p[1][2],
                         p[2][0], p[2][1], p[2][2], f->groupId()});
    }
    std::sort(faces.begin(), faces.end());
    std::string sig = "CV" + std::to_string(verts.size()) + ":";
    for (const auto& v : verts)
        sig += std::to_string(v[0]) + "," + std::to_string(v[1]) + "," + std::to_string(v[2]) + ";";
    sig += "|CF" + std::to_string(faces.size()) + ":";
    for (const auto& f : faces) {
        for (int k = 0; k < 10; ++k) sig += std::to_string(f[k]) + ",";
        sig += ";";
    }
    return sig;
}

/// Orientation-sensitive signature: canonical-rotation face cycles (positions,
/// orientation preserved) + directed-edge opposite pairing.  Unlike the plain
/// positional signature this catches reversed faces / wrong halfedge pairing.
inline std::string scOrientedSignature(MCGAL::Mesh* mesh) {
    if (!mesh) return "";
    auto q = [&](MCGAL::Vertex* v) {
        const MCGAL::PointInt p = mesh->floatPosToInt(v->point());
        return std::array<int, 3>{p.x(), p.y(), p.z()};
    };
    std::vector<std::string> faces;
    std::vector<std::string> pairs;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        std::vector<std::array<int, 3>> vs;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && h->vertex()) vs.push_back(q(h->vertex()));
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
        if (vs.empty()) continue;
        // canonical rotation while preserving orientation
        std::size_t best = 0;
        for (std::size_t i = 1; i < vs.size(); ++i) if (vs[i] < vs[best]) best = i;
        std::string key;
        for (std::size_t i = 0; i < vs.size(); ++i) {
            const auto& p = vs[(best + i) % vs.size()];
            key += std::to_string(p[0]) + "," + std::to_string(p[1]) + "," +
                   std::to_string(p[2]) + ";";
        }
        key += "g" + std::to_string(f->groupId());
        faces.push_back(key);
        h = h0;
        guard = 0;
        do {
            if (h && h->vertex() && h->next() && h->next()->vertex()) {
                const auto pa = q(h->vertex());
                const auto pb = q(h->next()->vertex());
                std::string edge = std::to_string(pa[0]) + "," + std::to_string(pa[1]) + "," +
                                   std::to_string(pa[2]) + ">" + std::to_string(pb[0]) + "," +
                                   std::to_string(pb[1]) + "," + std::to_string(pb[2]);
                MCGAL::Halfedge* opp = h->opposite();
                if (opp && !opp->isRemoved() && opp->vertex() && opp->end_vertex()) {
                    const auto qa = q(opp->vertex());
                    const auto qb = q(opp->end_vertex());
                    edge += "|" + std::to_string(qa[0]) + "," + std::to_string(qa[1]) + "," +
                            std::to_string(qa[2]) + ">" + std::to_string(qb[0]) + "," +
                            std::to_string(qb[1]) + "," + std::to_string(qb[2]);
                } else {
                    edge += "|none";
                }
                pairs.push_back(std::move(edge));
            }
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
    }
    std::sort(faces.begin(), faces.end());
    std::sort(pairs.begin(), pairs.end());
    std::string sig = "OF" + std::to_string(faces.size()) + ":";
    for (const auto& f : faces) sig += f + "|";
    sig += "OE" + std::to_string(pairs.size()) + ":";
    for (const auto& p : pairs) sig += p + "|";
    return sig;
}

inline std::uint64_t scMixHash(std::uint64_t h, std::uint64_t v) {
    h ^= v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
    return h;
}

/// Fast positional hash (unordered face vertex-position triples + vertex set).
inline std::uint64_t scPositionalHash(MCGAL::Mesh* mesh) {
    if (!mesh) return 0;
    std::vector<std::array<int, 7>> faces;
    std::vector<std::array<int, 3>> verts;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        const auto q = scloop::qOf(mesh, v);
        verts.push_back(q);
    }
    std::sort(verts.begin(), verts.end());
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        const auto vs = sbdetail::faceVertices(f);
        if (!vs[0]) continue;
        std::array<std::array<int, 3>, 3> p{};
        for (int k = 0; k < 3; ++k) {
            const MCGAL::PointInt q = mesh->floatPosToInt(vs[k]->point());
            p[k] = {q.x(), q.y(), q.z()};
        }
        std::sort(p.begin(), p.end());
        faces.push_back({p[0][0], p[0][1], p[0][2], p[1][0], p[1][1], p[1][2], f->groupId()});
    }
    std::sort(faces.begin(), faces.end());
    std::uint64_t h = 1469598103934665603ull;
    h = scMixHash(h, verts.size());
    for (const auto& v : verts) {
        h = scMixHash(h, static_cast<std::uint64_t>(v[0]));
        h = scMixHash(h, static_cast<std::uint64_t>(v[1]));
        h = scMixHash(h, static_cast<std::uint64_t>(v[2]));
    }
    h = scMixHash(h, faces.size());
    for (const auto& f : faces)
        for (int k = 0; k < 7; ++k) h = scMixHash(h, static_cast<std::uint64_t>(f[k]));
    return h;
}

/// Fast orientation/pairing hash: canonical-rotation oriented face cycles plus
/// directed-edge opposite pairing.
inline std::uint64_t scOrientedHash(MCGAL::Mesh* mesh) {
    if (!mesh) return 0;
    auto q = [&](MCGAL::Vertex* v) {
        const MCGAL::PointInt p = mesh->floatPosToInt(v->point());
        return std::array<int, 3>{p.x(), p.y(), p.z()};
    };
    std::vector<std::uint64_t> faceHashes;
    std::vector<std::uint64_t> pairHashes;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        std::vector<std::array<int, 3>> vs;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && h->vertex()) vs.push_back(q(h->vertex()));
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
        if (vs.empty()) continue;
        std::size_t best = 0;
        for (std::size_t i = 1; i < vs.size(); ++i) if (vs[i] < vs[best]) best = i;
        std::uint64_t fh = 1469598103934665603ull;
        for (std::size_t i = 0; i < vs.size(); ++i) {
            const auto& p = vs[(best + i) % vs.size()];
            fh = scMixHash(fh, static_cast<std::uint64_t>(p[0]));
            fh = scMixHash(fh, static_cast<std::uint64_t>(p[1]));
            fh = scMixHash(fh, static_cast<std::uint64_t>(p[2]));
        }
        fh = scMixHash(fh, static_cast<std::uint64_t>(f->groupId()));
        faceHashes.push_back(fh);
        h = h0;
        guard = 0;
        do {
            if (h && h->vertex() && h->next() && h->next()->vertex()) {
                const auto pa = q(h->vertex());
                const auto pb = q(h->next()->vertex());
                std::uint64_t eh = 1469598103934665603ull;
                for (int k = 0; k < 3; ++k) {
                    eh = scMixHash(eh, static_cast<std::uint64_t>(pa[k]));
                    eh = scMixHash(eh, static_cast<std::uint64_t>(pb[k]));
                }
                MCGAL::Halfedge* opp = h->opposite();
                if (opp && !opp->isRemoved() && opp->vertex() && opp->end_vertex()) {
                    const auto qa = q(opp->vertex());
                    const auto qb = q(opp->end_vertex());
                    for (int k = 0; k < 3; ++k) {
                        eh = scMixHash(eh, static_cast<std::uint64_t>(qa[k]));
                        eh = scMixHash(eh, static_cast<std::uint64_t>(qb[k]));
                    }
                } else {
                    eh = scMixHash(eh, 0xdeadbeefull);
                }
                pairHashes.push_back(eh);
            }
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
    }
    std::sort(faceHashes.begin(), faceHashes.end());
    std::sort(pairHashes.begin(), pairHashes.end());
    std::uint64_t h = 1469598103934665603ull;
    h = scMixHash(h, faceHashes.size());
    for (std::uint64_t x : faceHashes) h = scMixHash(h, x);
    h = scMixHash(h, pairHashes.size());
    for (std::uint64_t x : pairHashes) h = scMixHash(h, x);
    return h;
}

/// Encoder stage (mode62): apply up to maxOps single-copy group-boundary
/// collapses at the end of a round, appending compact patches (each carries its
/// own leading length) to roundData.
inline int runSingleCopyBoundaryEncoderStageCompact(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                                   int roundIndex, int maxOps,
                                                   const std::set<MCGAL::Vertex*>& protectedVerts,
                                                   std::vector<int>& roundData) {
    if (!mesh || maxOps <= 0) return 0;
    int applied = 0;
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
        std::sort(ordered.begin(), ordered.end(), [&](MCGAL::Vertex* a, MCGAL::Vertex* b) {
            const auto qa = mesh->floatPosToInt(a->point());
            const auto qb = mesh->floatPosToInt(b->point());
            return std::array<int, 3>{qa.x(), qa.y(), qa.z()} <
                   std::array<int, 3>{qb.x(), qb.y(), qb.z()};
        });
        bool ok = false;
        for (MCGAL::Vertex* v : ordered) {
            if (!v || rejected.count(v->poolId()) || protectedVerts.count(v)) continue;
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
            std::vector<int> data = captureSingleBoundaryPatchCompact(mesh.get(), patch);
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

/// Apply N collapses, capture compact patches, undo in reverse, compare exact
/// position-based signature.  Validates the compact codec end-to-end in memory.
inline int runSingleBoundaryCompactRoundTripTest(std::shared_ptr<MCGAL::Mesh> mesh, int maxOps) {
    if (!mesh) return 1;
    const std::uint64_t sig0 = scPositionalHash(mesh.get());
    const std::uint64_t ori0 = scOrientedHash(mesh.get());
    auto liveV = [&]() {
        int n = 0;
        for (MCGAL::Vertex* v : mesh->vertices()) if (v && !v->isRemoved()) ++n;
        return n;
    };
    const int V0 = liveV();
    const int F0 = mesh->size_of_facets();
    int b0 = 0, nm0 = 0, dup0 = 0, nt0 = 0;
    countSingleBoundaryManifold(mesh, b0, nm0, dup0, nt0);
    std::vector<std::vector<int>> patches;
    std::vector<std::uint64_t> postSigs;
    std::vector<std::uint64_t> postOriented;
    std::set<int> rejected;
    int applied = 0;
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
        std::sort(ordered.begin(), ordered.end(), [&](MCGAL::Vertex* a, MCGAL::Vertex* b) {
            const auto qa = mesh->floatPosToInt(a->point());
            const auto qb = mesh->floatPosToInt(b->point());
            return std::array<int, 3>{qa.x(), qa.y(), qa.z()} <
                   std::array<int, 3>{qb.x(), qb.y(), qb.z()};
        });
        bool ok = false;
        for (MCGAL::Vertex* v : ordered) {
            if (!v || rejected.count(v->poolId())) continue;
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
            std::vector<int> data = captureSingleBoundaryPatchCompact(mesh.get(), patch);
            if (data.empty()) {
                restoreSingleBoundaryPatch(mesh, patch);
                rejected.insert(v->poolId());
                continue;
            }
            patches.push_back(std::move(data));
            postSigs.push_back(scPositionalHash(mesh.get()));
            postOriented.push_back(scOrientedHash(mesh.get()));
            ++applied;
            ok = true;
            break;
        }
        if (!ok) break;
    }
    int b1 = 0, nm1 = 0, dup1 = 0, nt1 = 0;
    countSingleBoundaryManifold(mesh, b1, nm1, dup1, nt1);
    const int V1 = liveV();
    const int F1 = mesh->size_of_facets();
    bool undoOk = true;
    for (std::size_t k = patches.size(); k-- > 0;) {
        std::size_t cursor = 0;
        if (!applySingleBoundaryCompactPatchInverse(mesh, patches[k], cursor) ||
            cursor != patches[k].size()) {
            undoOk = false;
            std::fprintf(stderr, "[sc-compact-undo] inverse failed at patch=%zu/%zu\n", k,
                         patches.size());
            break;
        }
        const std::uint64_t want = (k == 0) ? sig0 : postSigs[k - 1];
        const std::uint64_t wantOri = (k == 0) ? ori0 : postOriented[k - 1];
        const bool posOk = scPositionalHash(mesh.get()) == want;
        const bool oriOk = scOrientedHash(mesh.get()) == wantOri;
        if (!posOk || !oriOk) {
            if (std::getenv("PMSF_SC_DEBUG"))
                std::fprintf(stderr,
                             "[sc-compact-undo] state mismatch after undoing patch=%zu (posOk=%d oriOk=%d)\n",
                             k, posOk ? 1 : 0, oriOk ? 1 : 0);
            undoOk = false;
            break;
        }
    }
    int b2 = 0, nm2 = 0, dup2 = 0, nt2 = 0;
    countSingleBoundaryManifold(mesh, b2, nm2, dup2, nt2);
    const bool exact = undoOk && scPositionalHash(mesh.get()) == sig0 &&
                       scOrientedHash(mesh.get()) == ori0;
    std::fprintf(stderr,
                 "[sc-compact-test] ops=%d V %d->%d->%d F %d->%d->%d "
                 "closed=%d/%d nonManifold=%d/%d dup=%d/%d nonTri=%d/%d afterUndo=%d/%d/%d/%d exact=%d\n",
                 applied, V0, V1, liveV(), F0, F1, mesh->size_of_facets(), b0, b1, nm0, nm1,
                 dup0, dup1, nt0, nt1, b2, nm2, dup2, nt2, exact ? 1 : 0);
    return exact ? 0 : 1;
}

}  // namespace pmsf
