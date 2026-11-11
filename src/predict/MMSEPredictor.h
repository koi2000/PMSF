// SPDX-License-Identifier: Apache-2.0
//
// MMSEPredictor.h — MMSE paper predictor stack (Lee et al., ICIP 2011, §2.2).
//
// This header implements the *paper's* geometry prediction modes for the MMSE
// progressive codec, all in the **quantized integer domain** so that encoder
// and decoder can compute bit-identically:
//
//   mode 0: barycentric prediction (§2.2.1 eq.(1)) — prediction residual is the
//           framework's existing r1 = v - barycenter, i.e. correction = 0.
//   mode 1: dual-ring prediction (§2.2.1 eq.(3)) with alpha = 0.5.
//   mode 2: dual-ring prediction (§2.2.1 eq.(3)) with alpha = 1.0.
//   mode 3: MMSE linear LS prediction (§2.2.2 eq.(4)-(8)) with rho = 0.997.
//
// Dual-ring (§2.2.1):
//   u_j  = mean_{l in N_j} x_l
//   Su   = sum_{l in N_j} 1/v_l^2 * ( v_l*x_l - sum_{k in N_l, k!=j} x_k )
//   w    = 1 + alpha * sum_{l in N_j} 1/v_l^2
//   xhat = (u_j + alpha*Su) / w
// With our integer domain we return the *correction over the reconstructed
// barycenter*:
//   corr = round(xhat) - uInt,  uInt = trunc( (sum of quantized ring x)/|N_j| )
// because the framework decoder reconstructs v = uInt + residual (see
// BasicReconstructOperator::reconstructBarycenterPointNoAlloc).
//
// In the *coarser* mesh where the prediction happens, vertex j is absent, so
// for l in N_j the paper's v_l (= |N_l| in the finer mesh, which contains j)
// equals (current coarse neighbours of l) + 1, and
// sum_{k in N_l, k != j} x_k equals the sum over l's coarse neighbours.
// (Using the coarse valence without the +1 makes the formula collapse by a
// constant factor on planar regions — measured: residual stream +20 KB.)
//
// MMSE LS (§2.2.2): the neighbourhood S is the union of the 1-ring and 2-ring
// of vertex j (topological distance 1 or 2).  R_kl = rho^{d_kl},
// C_k = rho^{d_kj}; weights w = R^{-1} C; xhat = sum_k w_k x_k.
//
// SYMMETRY CONTRACT (why this works):
//   - Every ring is walked with an explicit *anchor halfedge taken from the
//     gate polygon* (halfedge circulation: h -> h->opposite()->next()), never
//     from Vertex::proxyHalfedge().  The gate polygon ring is bit- identical on
//     both sides (PCLTTM deterministic fan + normalized proxyHalfedge), while a
//     coarse vertex's proxyHalfedge can point at a halfedge whose face was
//     merged by elimination — which made the encoder walk a different ring
//     than the decoder (measured: connectivity/topology decode FAIL).
//   - All arithmetic that depends on mesh geometry is done on quantized integer
//     coordinates recovered with `quantInt(vertex)`; float reconstructed
//     positions are only ever converted to int (never the other way round), so
//     encoder (original float positions) and decoder (intPosToFloat positions)
//     agree exactly.
//   - The per-call order of the predictor equals the per-polyFace BFS order on
//     both sides, so the per-cluster mode sequence can be replayed from a small
//     mode stream (see MMSEDualRingSymbolCollectOperator).
//
// Default-off contract: every entry point is inert unless the MMSE collector
// enables the predictor (env PMSF_MMSE_PREDICTOR) or the MMSE reader installs a
// replay round (bitstream self-describing flag).  Nothing here is reachable from
// any other algorithm's gate path, so the frozen baselines stay byte-identical.

#ifndef PMSF_PREDICT_MMSE_PREDICTOR_H
#define PMSF_PREDICT_MMSE_PREDICTOR_H

#include "Facet.h"
#include "Halfedge.h"
#include "Point.h"
#include "Vertex.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <utility>
#include <vector>

namespace pmsf {
namespace mmse_pred {

// ---------------------------------------------------------------------------
// Mode ids (must match the 2-bit stream encoding used by the MMSE blob).
// ---------------------------------------------------------------------------
enum : int {
    kModeBarycentric = 0,
    kModeDualRing05  = 1,
    kModeDualRing1   = 2,
    kModeMMSELS      = 3,
    kModeCount       = 4,
};

// Paper constant (§3): correlation coefficient of eq.(7)/(8).
inline constexpr double kRho = 0.997;

// Ring-walk safety bound (a well-formed umbrella closes in <= valence steps).
inline constexpr int kMaxRingSteps = 256;

// ---------------------------------------------------------------------------
// Quantization context (set by the MMSE collector / reader before use).
// ---------------------------------------------------------------------------
struct QuantContext {
    MCGAL::Point bboxMin{0.0f, 0.0f, 0.0f};
    float quantStep = 0.0f;
    bool valid = false;
};

inline QuantContext& quantContext() {
    static QuantContext ctx;
    return ctx;
}

inline void setQuantContext(const MCGAL::Point& bboxMin, float quantStep) {
    QuantContext& ctx = quantContext();
    ctx.bboxMin = bboxMin;
    ctx.quantStep = quantStep;
    ctx.valid = (quantStep > 0.0f);
}

inline void clearQuantContext() {
    QuantContext& ctx = quantContext();
    ctx.quantStep = 0.0f;
    ctx.valid = false;
}

/// Quantize a vertex position exactly like Mesh::floatPosToInt().  Used on both
/// encoder (original float position) and decoder (intPosToFloat reconstructed
/// position) — trunc after subtracting bboxMin, so both sides recover the same
/// integer grid coordinate.
inline MCGAL::PointInt quantInt(MCGAL::Vertex* v) {
    if (v == nullptr) return MCGAL::PointInt(0, 0, 0);
    const QuantContext& ctx = quantContext();
    const MCGAL::Point p = v->point();
    const double px = static_cast<double>(p.x());
    const double py = static_cast<double>(p.y());
    const double pz = static_cast<double>(p.z());
    if (!ctx.valid) {
        // Defensive: should never happen on the MMSE paths (both sides call
        // setQuantContext).  Rounding keeps the value bounded.
        return MCGAL::PointInt(static_cast<int>(std::lround(px)),
                               static_cast<int>(std::lround(py)),
                               static_cast<int>(std::lround(pz)));
    }
    return MCGAL::PointInt(
        static_cast<int>((px - static_cast<double>(ctx.bboxMin.x())) /
                         static_cast<double>(ctx.quantStep)),
        static_cast<int>((py - static_cast<double>(ctx.bboxMin.y())) /
                         static_cast<double>(ctx.quantStep)),
        static_cast<int>((pz - static_cast<double>(ctx.bboxMin.z())) /
                         static_cast<double>(ctx.quantStep)));
}

// ---------------------------------------------------------------------------
// Decoder replay state: installed per round by the MMSE read operator right
// before the PCLTTM-compatible BFS starts.  The predictor hook in
// RDPredictor.h::rdCurvaturePrediction() consults this.
// ---------------------------------------------------------------------------
struct ReplayState {
    bool enabled = false;
    int clusterSize = 0;                                // vertices per cluster
    const std::vector<std::uint8_t>* modes = nullptr;   // per-cluster mode ids
    std::size_t callIndex = 0;                          // polyFace counter (round)
};

inline ReplayState& replayState() {
    static ReplayState st;
    return st;
}

inline void clearReplayState() {
    ReplayState& st = replayState();
    st.enabled = false;
    st.clusterSize = 0;
    st.modes = nullptr;
    st.callIndex = 0;
}

// ---------------------------------------------------------------------------
// Deterministic halfedge-circulation helpers.
//
// MCGAL convention: a halfedge h has vertex() = start, end_vertex() = end,
// next() = next in the same face loop.  The halfedges *outgoing* from vertex v
// are circulated by:  h -> h->opposite()->next().
// ---------------------------------------------------------------------------

/// Collect the gate polygon ring: (vertex, outgoing halfedge) pairs around the
/// removed vertex j.  The outgoing halfedge (h->vertex() == ring vertex) is the
/// anchor used to circulate that vertex's umbrella deterministically.
inline void collectGateRing(MCGAL::Facet* f,
                            std::vector<MCGAL::Vertex*>& verts,
                            std::vector<MCGAL::Halfedge*>& anchors) {
    verts.clear();
    anchors.clear();
    if (f == nullptr) return;
    MCGAL::Halfedge* st = f->proxyHalfedge();
    if (st == nullptr) return;
    MCGAL::Halfedge* it = st;
    for (int steps = 0; steps < kMaxRingSteps; ++steps) {
        MCGAL::Vertex* v = it->vertex();
        if (v != nullptr && !v->isRemoved()) {
            verts.push_back(v);
            anchors.push_back(it);
        }
        MCGAL::Halfedge* nxt = it->next();
        if (nxt == nullptr || nxt == st) break;
        it = nxt;
    }
}

/// Circulate the umbrella of `start->vertex()` starting at outgoing halfedge
/// `start`.  Fills the (deduplicated) neighbour vertices plus, for each, an
/// outgoing halfedge *of the neighbour* (h->opposite()) for further anchored
/// walks.
inline void collectUmbrella(MCGAL::Halfedge* start,
                            std::vector<MCGAL::Vertex*>& neighbours,
                            std::vector<MCGAL::Halfedge*>* neighbourAnchors) {
    neighbours.clear();
    if (neighbourAnchors != nullptr) neighbourAnchors->clear();
    if (start == nullptr || start->vertex() == nullptr) return;
    MCGAL::Vertex* center = start->vertex();
    MCGAL::Halfedge* h = start;
    for (int steps = 0; steps < kMaxRingSteps; ++steps) {
        if (h == nullptr || h->vertex() != center) break;
        MCGAL::Vertex* nb = h->end_vertex();
        if (nb != nullptr && nb != center && !nb->isRemoved()) {
            bool dup = false;
            for (MCGAL::Vertex* seen : neighbours) {
                if (seen == nb) { dup = true; break; }
            }
            if (!dup) {
                neighbours.push_back(nb);
                if (neighbourAnchors != nullptr) {
                    // h is (center -> nb), so h->opposite() is (nb -> center):
                    // an outgoing halfedge of nb.
                    neighbourAnchors->push_back(h->opposite());
                }
            }
        }
        MCGAL::Halfedge* opp = h->opposite();
        if (opp == nullptr || opp->isRemoved()) break;  // boundary / dead link
        MCGAL::Halfedge* nxt = opp->next();
        if (nxt == nullptr) break;
        h = nxt;
        if (h == start) break;
    }
}

// ---------------------------------------------------------------------------
// Face geometry scratch: gate ring + quantized positions + integer barycenter.
// ---------------------------------------------------------------------------
struct FaceGeom {
    std::vector<MCGAL::Vertex*> ringVerts;
    std::vector<MCGAL::Halfedge*> ringAnchors;  // outgoing halfedge at each ring vertex
    std::vector<MCGAL::PointInt> ringQ;
    MCGAL::PointInt uInt{0, 0, 0};  // trunc(int mean) — same as the decoder's
                                    // reconstructBarycenterPointNoAlloc()
    double ux = 0.0, uy = 0.0, uz = 0.0;  // exact double mean of ringQ
    bool ok = false;
};

inline bool prepareFaceGeom(MCGAL::Facet* f, FaceGeom& g) {
    collectGateRing(f, g.ringVerts, g.ringAnchors);
    g.ringQ.clear();
    g.ok = false;
    if (g.ringVerts.empty()) return false;
    g.ringQ.reserve(g.ringVerts.size());
    int64_t sx = 0, sy = 0, sz = 0;
    for (MCGAL::Vertex* v : g.ringVerts) {
        const MCGAL::PointInt q = quantInt(v);
        g.ringQ.push_back(q);
        sx += q.x();
        sy += q.y();
        sz += q.z();
    }
    const int n = static_cast<int>(g.ringQ.size());
    g.uInt = MCGAL::PointInt(static_cast<int>(sx / n),
                             static_cast<int>(sy / n),
                             static_cast<int>(sz / n));
    g.ux = static_cast<double>(sx) / static_cast<double>(n);
    g.uy = static_cast<double>(sy) / static_cast<double>(n);
    g.uz = static_cast<double>(sz) / static_cast<double>(n);
    g.ok = true;
    return true;
}

inline double roundHalfAway(double v) {
    return (v >= 0.0) ? std::floor(v + 0.5) : std::ceil(v - 0.5);
}

// ---------------------------------------------------------------------------
// Dual-ring prediction (§2.2.1 eq.(3)).
// ---------------------------------------------------------------------------
struct DualRingTerms {
    double sux = 0.0, suy = 0.0, suz = 0.0;
    double wsum = 0.0;
    bool ok = false;
};

inline bool computeDualRingTerms(const FaceGeom& g, DualRingTerms& t) {
    t = DualRingTerms{};
    if (!g.ok) return false;
    std::vector<MCGAL::Vertex*> neigh;
    for (std::size_t i = 0; i < g.ringVerts.size(); ++i) {
        collectUmbrella(g.ringAnchors[i], neigh, nullptr);
        // NOTE: the removed vertex j is NOT in the coarse mesh, but the paper's
        // v_l is |N_l| in the finer mesh, and j is in N_l.  Hence
        //   v_l = (coarse neighbours of l) + 1,
        // and sum_{k in N_l, k != j} x_k = sum of l's coarse neighbours.
        const int vl = static_cast<int>(neigh.size()) + 1;
        const double w =
            1.0 / (static_cast<double>(vl) * static_cast<double>(vl));
        const MCGAL::PointInt ql = g.ringQ[i];
        int64_t skx = 0, sky = 0, skz = 0;
        for (MCGAL::Vertex* k : neigh) {
            const MCGAL::PointInt qk = quantInt(k);
            skx += qk.x();
            sky += qk.y();
            skz += qk.z();
        }
        t.sux += w * (static_cast<double>(vl) * static_cast<double>(ql.x()) -
                      static_cast<double>(skx));
        t.suy += w * (static_cast<double>(vl) * static_cast<double>(ql.y()) -
                      static_cast<double>(sky));
        t.suz += w * (static_cast<double>(vl) * static_cast<double>(ql.z()) -
                      static_cast<double>(skz));
        t.wsum += w;
    }
    t.ok = true;
    return true;
}

/// Correction over the integer barycenter for a dual-ring alpha.
inline MCGAL::PointInt dualRingCorrection(const FaceGeom& g,
                                          const DualRingTerms& t,
                                          double alpha) {
    if (!g.ok) return MCGAL::PointInt(0, 0, 0);
    const double w = 1.0 + alpha * t.wsum;
    if (w <= 0.0) return MCGAL::PointInt(0, 0, 0);
    const double x = (g.ux + alpha * t.sux) / w;
    const double y = (g.uy + alpha * t.suy) / w;
    const double z = (g.uz + alpha * t.suz) / w;
    return MCGAL::PointInt(static_cast<int>(roundHalfAway(x)) - g.uInt.x(),
                           static_cast<int>(roundHalfAway(y)) - g.uInt.y(),
                           static_cast<int>(roundHalfAway(z)) - g.uInt.z());
}

// ---------------------------------------------------------------------------
// MMSE linear LS prediction (§2.2.2 eq.(4)-(8)).
//
// Neighbourhood S = N1 ∪ N2 (topological distance 1 or 2 from j), each vertex
// carried with an anchor outgoing halfedge so every ring walk is deterministic.
// Pairwise topological distances among S: BFS with anchor walks, depths 1..4
// (rho^d decays to 0.988 by d=4; farther pairs are clamped to d=5).
// R_kl = rho^{d_kl} (diag 1), C_k = rho^{d_kj}; solve R w = C (Gauss with
// partial pivoting) and predict xhat = sum_k w_k x_k.
// ---------------------------------------------------------------------------
inline constexpr int kLsMaxSpan = 5;  // distance clamp

struct LsNeighbourhood {
    std::vector<MCGAL::Vertex*> verts;      // S (union of 1-ring and 2-ring)
    std::vector<MCGAL::Halfedge*> anchors;  // outgoing halfedge per S vertex
    std::vector<MCGAL::PointInt> pos;       // quantized positions of S
    std::vector<int> distFromJ;             // 1 for N1, 2 for N2
    bool ok = false;
};

inline bool buildLsNeighbourhood(const FaceGeom& g, LsNeighbourhood& nb) {
    nb = LsNeighbourhood{};
    if (!g.ok) return false;
    auto contains = [](const std::vector<MCGAL::Vertex*>& v, MCGAL::Vertex* x) {
        for (MCGAL::Vertex* e : v) {
            if (e == x) return true;
        }
        return false;
    };
    // 1-ring.
    for (std::size_t i = 0; i < g.ringVerts.size(); ++i) {
        nb.verts.push_back(g.ringVerts[i]);
        nb.anchors.push_back(g.ringAnchors[i]);
        nb.distFromJ.push_back(1);
    }
    const std::size_t n1 = nb.verts.size();
    // 2-ring: neighbours of the 1-ring vertices that are not in the 1-ring.
    std::vector<MCGAL::Vertex*> neigh;
    std::vector<MCGAL::Halfedge*> neighAnchors;
    for (std::size_t i = 0; i < n1; ++i) {
        collectUmbrella(nb.anchors[i], neigh, &neighAnchors);
        for (std::size_t k = 0; k < neigh.size(); ++k) {
            if (contains(nb.verts, neigh[k])) continue;
            nb.verts.push_back(neigh[k]);
            nb.anchors.push_back(neighAnchors[k]);
            nb.distFromJ.push_back(2);
        }
    }
    const int n = static_cast<int>(nb.verts.size());
    if (n < 3) return false;
    nb.pos.reserve(nb.verts.size());
    for (MCGAL::Vertex* v : nb.verts) nb.pos.push_back(quantInt(v));
    nb.ok = true;
    return true;
}

/// BFS distance between two anchored vertices, clamped to [1, kLsMaxSpan].
inline int meshDistanceAnchored(MCGAL::Vertex* a, MCGAL::Halfedge* anchorA,
                                MCGAL::Vertex* b, int maxDepth) {
    if (a == b) return 0;
    if (a == nullptr || b == nullptr || anchorA == nullptr) return kLsMaxSpan;
    std::vector<MCGAL::Vertex*> frontierV{a};
    std::vector<MCGAL::Halfedge*> frontierH{anchorA};
    std::vector<MCGAL::Vertex*> visited{a};
    std::vector<MCGAL::Vertex*> neigh;
    std::vector<MCGAL::Halfedge*> neighAnchors;
    for (int depth = 1; depth <= maxDepth; ++depth) {
        std::vector<MCGAL::Vertex*> nextV;
        std::vector<MCGAL::Halfedge*> nextH;
        for (std::size_t i = 0; i < frontierV.size(); ++i) {
            collectUmbrella(frontierH[i], neigh, &neighAnchors);
            for (std::size_t k = 0; k < neigh.size(); ++k) {
                if (neigh[k] == b) return depth;
                bool seen = false;
                for (MCGAL::Vertex* w : visited) {
                    if (w == neigh[k]) { seen = true; break; }
                }
                if (!seen) {
                    visited.push_back(neigh[k]);
                    nextV.push_back(neigh[k]);
                    nextH.push_back(neighAnchors[k]);
                }
            }
        }
        frontierV.swap(nextV);
        frontierH.swap(nextH);
        if (frontierV.empty()) break;
    }
    return kLsMaxSpan;
}

/// Solve the small dense system R w = C in place (Gauss elimination, partial
/// pivoting).  R is n x n row-major, C is length n.
inline bool solveDense(double* R, double* C, int n) {
    for (int col = 0; col < n; ++col) {
        int pivot = col;
        double best = std::fabs(R[static_cast<std::size_t>(col) * n + col]);
        for (int row = col + 1; row < n; ++row) {
            const double v = std::fabs(R[static_cast<std::size_t>(row) * n + col]);
            if (v > best) { best = v; pivot = row; }
        }
        if (best < 1e-12) return false;
        if (pivot != col) {
            for (int k = 0; k < n; ++k) {
                std::swap(R[static_cast<std::size_t>(col) * n + k],
                          R[static_cast<std::size_t>(pivot) * n + k]);
            }
            std::swap(C[col], C[pivot]);
        }
        const double inv = 1.0 / R[static_cast<std::size_t>(col) * n + col];
        for (int row = col + 1; row < n; ++row) {
            const double f = R[static_cast<std::size_t>(row) * n + col] * inv;
            if (f == 0.0) continue;
            for (int k = col; k < n; ++k) {
                R[static_cast<std::size_t>(row) * n + k] -=
                    f * R[static_cast<std::size_t>(col) * n + k];
            }
            C[row] -= f * C[col];
        }
    }
    for (int row = n - 1; row >= 0; --row) {
        double acc = C[row];
        for (int k = row + 1; k < n; ++k) {
            acc -= R[static_cast<std::size_t>(row) * n + k] * C[k];
        }
        const double diag = R[static_cast<std::size_t>(row) * n + row];
        if (std::fabs(diag) < 1e-12) return false;
        C[row] = acc / diag;
    }
    return true;
}

inline MCGAL::PointInt mmseLsCorrection(const FaceGeom& g,
                                        const LsNeighbourhood& nb,
                                        double rho = kRho) {
    if (!g.ok || !nb.ok) return MCGAL::PointInt(0, 0, 0);
    const int n = static_cast<int>(nb.verts.size());
    std::vector<double> R(static_cast<std::size_t>(n) * n, 0.0);
    std::vector<double> C(static_cast<std::size_t>(n), 0.0);
    for (int i = 0; i < n; ++i) {
        C[i] = std::pow(rho, static_cast<double>(nb.distFromJ[i]));
        for (int k = 0; k < n; ++k) {
            const int d =
                (i == k) ? 0
                         : meshDistanceAnchored(nb.verts[i], nb.anchors[i],
                                                nb.verts[k], kLsMaxSpan);
            R[static_cast<std::size_t>(i) * n + k] =
                std::pow(rho, static_cast<double>(d));
        }
    }
    // R is badly conditioned (rho -> 1); a tiny ridge keeps the solve stable.
    // The ridge is part of the *decided* predictor on both sides, so symmetry
    // is preserved.
    constexpr double kRidge = 1e-9;
    for (int i = 0; i < n; ++i) R[static_cast<std::size_t>(i) * n + i] += kRidge;
    if (!solveDense(R.data(), C.data(), n)) return MCGAL::PointInt(0, 0, 0);
    // Sanity gate: the LS weights must form a sane convex-ish combination.
    // When R^{-1} C blows up (near-singular R), fall back to barycentric rather
    // than emitting a wild prediction (both sides apply the same gate, so no
    // symmetry risk).
    double wsum = 0.0, wmax = 0.0, wabs = 0.0;
    for (int i = 0; i < n; ++i) {
        wsum += C[i];
        wabs += std::fabs(C[i]);
        wmax = std::max(wmax, std::fabs(C[i]));
    }
    if (!(wsum > 0.5 && wsum < 1.5) || wabs > 4.0 || wmax > 2.0) {
        return MCGAL::PointInt(0, 0, 0);
    }
    double x = 0.0, y = 0.0, z = 0.0;
    for (int i = 0; i < n; ++i) {
        x += C[i] * static_cast<double>(nb.pos[i].x());
        y += C[i] * static_cast<double>(nb.pos[i].y());
        z += C[i] * static_cast<double>(nb.pos[i].z());
    }
    return MCGAL::PointInt(static_cast<int>(roundHalfAway(x)) - g.uInt.x(),
                           static_cast<int>(roundHalfAway(y)) - g.uInt.y(),
                           static_cast<int>(roundHalfAway(z)) - g.uInt.z());
}

// ---------------------------------------------------------------------------
// Full per-face correction for one mode (used by the encoder decision and by
// the decoder replay hook).  mode 0 returns the zero correction, i.e. the
// prediction residual stays the framework's existing barycentric r1.
// ---------------------------------------------------------------------------
inline MCGAL::PointInt correctionForMode(MCGAL::Facet* f, int mode) {
    if (f == nullptr || mode <= kModeBarycentric || mode >= kModeCount) {
        return MCGAL::PointInt(0, 0, 0);
    }
    FaceGeom g;
    if (!prepareFaceGeom(f, g)) return MCGAL::PointInt(0, 0, 0);
    if (mode == kModeDualRing05 || mode == kModeDualRing1) {
        DualRingTerms t;
        if (!computeDualRingTerms(g, t)) return MCGAL::PointInt(0, 0, 0);
        return dualRingCorrection(g, t, mode == kModeDualRing05 ? 0.5 : 1.0);
    }
    if (mode == kModeMMSELS) {
        LsNeighbourhood nb;
        if (!buildLsNeighbourhood(g, nb)) return MCGAL::PointInt(0, 0, 0);
        return mmseLsCorrection(g, nb, kRho);
    }
    return MCGAL::PointInt(0, 0, 0);
}

/// Compute all four candidate corrections for one face with a single gate-ring
/// traversal (encoder-side mode decision).  `includeLs=false` leaves mode 3 at
/// the barycentric correction so the decision degrades to 3 modes.
inline void correctionsForFace(MCGAL::Facet* f, bool includeLs,
                               MCGAL::PointInt out[kModeCount]) {
    for (int i = 0; i < kModeCount; ++i) out[i] = MCGAL::PointInt(0, 0, 0);
    FaceGeom g;
    if (!prepareFaceGeom(f, g)) return;
    DualRingTerms t;
    if (computeDualRingTerms(g, t)) {
        out[kModeDualRing05] = dualRingCorrection(g, t, 0.5);
        out[kModeDualRing1] = dualRingCorrection(g, t, 1.0);
    }
    if (includeLs) {
        LsNeighbourhood nb;
        if (buildLsNeighbourhood(g, nb)) {
            out[kModeMMSELS] = mmseLsCorrection(g, nb, kRho);
        }
    }
}

/// Decoder replay: pick the mode for the current polyFace from the per-cluster
/// mode list and return the correction.  Increments the per-round call index.
inline MCGAL::PointInt replayCorrection(MCGAL::Facet* f) {
    ReplayState& st = replayState();
    const std::size_t idx = st.callIndex++;
    int mode = kModeBarycentric;
    if (st.modes != nullptr && !st.modes->empty()) {
        std::size_t cluster = (st.clusterSize > 0)
                                  ? (idx / static_cast<std::size_t>(st.clusterSize))
                                  : std::size_t{0};
        if (cluster >= st.modes->size()) cluster = st.modes->size() - 1;
        mode = static_cast<int>((*st.modes)[cluster]);
    }
    return correctionForMode(f, mode);
}

// ---------------------------------------------------------------------------
// Environment parsing (encoder side).
// ---------------------------------------------------------------------------
/// PMSF_MMSE_PREDICTOR: "", "0", "off" => disabled; "bary"|"dr0" => fixed mode 0;
/// "dr05" => fixed mode 1; "dr1" => fixed mode 2; "ls" => fixed mode 3;
/// "mode" => per-cluster mode decision over all four modes.
/// Returns -2 when disabled, -1 for per-cluster decision, else the fixed mode.
inline int predictorModeFromEnv() {
    const char* env = std::getenv("PMSF_MMSE_PREDICTOR");
    if (env == nullptr || env[0] == '\0' || env[0] == '0') return -2;
    if (std::strcmp(env, "off") == 0) return -2;
    if (std::strcmp(env, "bary") == 0 || std::strcmp(env, "dr0") == 0) {
        return kModeBarycentric;
    }
    if (std::strcmp(env, "dr05") == 0) return kModeDualRing05;
    if (std::strcmp(env, "dr1") == 0) return kModeDualRing1;
    if (std::strcmp(env, "ls") == 0) return kModeMMSELS;
    if (std::strcmp(env, "mode") == 0) return -1;
    return -2;
}

inline int clusterSizeFromEnv(int fallback = 32) {
    const char* env = std::getenv("PMSF_MMSE_CLUSTER");
    if (env == nullptr || env[0] == '\0') return fallback;
    const int v = std::atoi(env);
    if (v < 1) return fallback;
    if (v > 4096) return 4096;
    return v;
}

inline bool lsEnabledFromEnv() {
    const char* env = std::getenv("PMSF_MMSE_LS");
    if (env == nullptr || env[0] == '\0') return true;
    return env[0] != '0';
}

}  // namespace mmse_pred
}  // namespace pmsf

#endif  // PMSF_PREDICT_MMSE_PREDICTOR_H
