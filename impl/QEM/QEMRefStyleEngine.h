/*
 * @file QEMRefStyleEngine.h
 * @brief Framework-owned QEM simplification engine with the reference
 *        union-find / face-list topology policy (task-16).
 *
 * Why this file exists
 * --------------------
 * The E250 acceptance row for QEM compares a 250-face approximation of bunny
 * against the reference full-QEM implementation.  The framework's MCGAL path
 * (`QEMFaceBudget` + `Mesh::new_collapse`) cannot reproduce the reference
 * operating point because the MCGAL link condition changes the collapse order
 * long before 250 faces (it stalls at ~3.1k faces; see
 * impl/QEM/QEM_E250_AUDIT.md).  The reference itself is a standalone engine
 * that stores vertices as union-find representatives plus per-vertex incident
 * face lists, has no link condition, and relies on its own flip penalty to
 * reject bad collapses.  This header is our own implementation of that
 * topology *policy*: it does not link, read or shell out to
 * `reference/OpenMesh_QEM/` and it is compiled from framework sources only.
 *
 * What is implemented (all four reference-policy pieces)
 * ------------------------------------------------------
 *  1. Independent-set deferral: `cannot_delete` is rebuilt every round; a
 *     candidate whose target vertex is protected is *deferred* to the next
 *     round instead of dropped (the legacy framework path blocks the one-ring
 *     persistently).
 *  2. Exact quadric recomputation: after every collapse the survivor and its
 *     live one-ring recompute Q from their surviving incident face planes
 *     (paper Eq. 2/3) instead of accumulating Q1+Q2 forever.
 *  3. Local candidate re-push: the survivor's live one-ring gets fresh
 *     candidates with per-pair timers; stale heap entries are dropped by
 *     version counter.
 *  4. Face-flip penalty: every surviving face incident to the collapsed pair
 *     compares its old and new normalized normal; dot <= 0.25 => cost 1e5
 *     (paper §6 "Preventing Mesh Inversion").
 *
 * Numerical conventions deliberately mirror the reference engine (unit plane
 * quadric per face, 4x4 Gaussian elimination with partial pivoting for the
 * optimal placement, midpoint fallback when the matrix is singular) so the two
 * engines can be cross-checked.  Tie-breaking is deterministic by default
 * (seeded ranking of candidate insertion order) so results are reproducible;
 * `TieBreakMode::Random` mirrors the reference's `rand()` tie-break and is used
 * only for engine-equivalence validation, never as measurement evidence.
 *
 * Env-gated usage: tools/QEMFaceBudget.cpp dispatches here only when
 * `PMSF_QEM_REF_ENGINE=1`; the default legacy MCGAL path is untouched.
 */
#ifndef PMSF_IMPL_QEM_QEM_REF_STYLE_ENGINE_H
#define PMSF_IMPL_QEM_QEM_REF_STYLE_ENGINE_H

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace pmsf {
namespace qem_refstyle {

struct Vec3d {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct TriangleMesh {
    std::vector<Vec3d> vertices;
    std::vector<std::array<int, 3>> faces;
};

enum class TieBreakMode {
    Seeded = 0,  // deterministic pseudo-random ranking of insertion order (default)
    Fifo = 1,    // earliest inserted candidate wins near-ties
    Lifo = 2,    // latest inserted candidate wins near-ties
    Random = 3,  // reference `rand() & 1` behaviour; for validation only
};

/// One incident face of the victim at collapse time, rotated so that the
/// victim is the first corner: the face was `(victim, a, b)` cyclically.  The
/// paired progressive decoder needs exactly this orientation to rebuild the
/// face: if `a`/`b` is the survivor the face was killed by the collapse,
/// otherwise the existing `(survivor, a, b)` face is re-attributed to the
/// victim.  This makes the reverse operation self-contained (no face ids /
/// adjacency lookups are stored) and also covers non-manifold victims.
struct CollapseFanFace {
    int a = 0;
    int b = 0;
};

/// Everything the paired progressive decoder needs to invert one collapse.
/// Vertex ids are *original* vertex ids (union-find representatives are always
/// original ids), so they stay valid across the whole reverse decode.
struct CollapseRecord {
    int survivor = -1;
    int victim = -1;
    /// Positions of both endpoints *before* the collapse (the collapse moves the
    /// survivor to the optimal placement; the reverse step restores both).
    double survivorPosBefore[3] = {0.0, 0.0, 0.0};
    double victimPosBefore[3] = {0.0, 0.0, 0.0};
    /// Simplification round this collapse belongs to (1-based, engine round id).
    long long roundIndex = 0;
    /// All live faces incident to the victim at collapse time, as oriented
    /// (victim, a, b) pairs.
    std::vector<CollapseFanFace> fanFaces;
};

/// Optional recording sink used by the progressive encoder
/// (impl/QEM/QEMProgressiveArtifact.h).  A null recorder leaves the engine's
/// behaviour and output byte-identical to the plain snapshot path.
class CollapseRecorder {
  public:
    virtual ~CollapseRecorder() = default;
    virtual void onCollapse(const CollapseRecord& record) = 0;
};

struct SimplifyOptions {
    /// Desired output face count.  Collapse budget is
    /// (inputFaces - targetFaces) / 2, which is exactly the reference
    /// `target = (int)((1 - simp_rate) * face_num) >> 1` with
    /// simp_rate = 1 - targetFaces / inputFaces.
    int targetFaces = 250;
    /// Paper §3.2 pair threshold t.  t=0 => edge-only contraction (the
    /// reference calibration setting); t>0 adds non-edge pairs (brute force,
    /// small meshes only -- the reference uses a k-d split that is itself an
    /// approximation, so t>0 is bit-for-bit comparable only at t=0).
    double pairThreshold = 0.0;
    TieBreakMode tieBreak = TieBreakMode::Seeded;
    std::uint64_t tieBreakSeed = 1;
    bool verbose = false;
    /// Optional progressive-collapse recorder (nullptr = snapshot path only).
    CollapseRecorder* recorder = nullptr;
};

struct SimplifyStats {
    int inputVertices = 0;
    int inputFaces = 0;
    int outputVertices = 0;
    int outputFaces = 0;
    long long collapseCount = 0;
    long long roundCount = 0;
    int targetCollapses = 0;
    double totalCost = 0.0;
    bool reachedTarget = false;
    bool stalled = false;
    /// Original vertex id of every exported vertex, in exported order.  The
    /// progressive encoder needs this to key the base mesh by stable ids.
    std::vector<int> outputVertexOriginalIds;

    std::string summary() const {
        std::ostringstream os;
        os << "input=(v=" << inputVertices << ",f=" << inputFaces << ")"
           << " output=(v=" << outputVertices << ",f=" << outputFaces << ")"
           << " collapses=" << collapseCount << "/" << targetCollapses
           << " rounds=" << roundCount << " total_cost=" << totalCost
           << " reached_target=" << (reachedTarget ? "true" : "false")
           << " stalled=" << (stalled ? "true" : "false");
        return os.str();
    }
};

namespace detail {

/// Reference face record: the three live union-find representatives plus the
/// unit plane (a,b,c,d) recomputed whenever the face survives a collapse.
struct Face {
    int v1 = 0;
    int v2 = 0;
    int v3 = 0;
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    double d = 0.0;
};

/// Reference vertex record: position, accumulated 4x4 quadric, and the set of
/// live incident faces (rebuilt as faces die / are remapped).
struct Vertex {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double Q[4][4] = {};
    std::set<int> faceList;
};

/// Heap entry: candidate contraction (u, v) -> (x, y, z) with cost `err`.
/// `seq` is a monotonically increasing insertion id used by the deterministic
/// tie-break modes; `timeCnt` is the per-pair version stamp that invalidates
/// stale heap entries.
struct Candidate {
    int u = 0;
    int v = 0;
    double err = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    int timeCnt = 0;
    long long seq = 0;
};

inline std::uint64_t splitmix64(std::uint64_t x) {
    x += 0x9E3779B97F4A7C15ull;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
    return x ^ (x >> 31);
}

struct CandidateCmp {
    TieBreakMode mode = TieBreakMode::Seeded;
    std::uint64_t seed = 1;

    std::uint64_t rank(long long seq) const {
        return splitmix64(seed ^ static_cast<std::uint64_t>(seq));
    }

    /// Strict-weak-order-free comparator mirroring the reference
    /// `(a.err > b.err + 1e-9) || (|a.err-b.err| < 1e-9 && rand()&1)`;
    /// the deterministic modes replace `rand()&1` with a fixed order.
    bool operator()(const Candidate& a, const Candidate& b) const {
        if (a.err > b.err + 1e-9) return true;
        if (std::fabs(a.err - b.err) < 1e-9) {
            switch (mode) {
                case TieBreakMode::Random:
                    return (std::rand() & 1) != 0;
                case TieBreakMode::Lifo:
                    return a.seq < b.seq;
                case TieBreakMode::Seeded:
                    if (rank(a.seq) != rank(b.seq)) return rank(a.seq) > rank(b.seq);
                    return a.seq > b.seq;
                case TieBreakMode::Fifo:
                default:
                    return a.seq > b.seq;
            }
        }
        return false;
    }
};

class Engine {
  public:
    Engine(const TriangleMesh& input, const SimplifyOptions& options,
           SimplifyStats* stats)
        : opts_(options),
          stats_(stats),
          heap_(CandidateCmp{options.tieBreak, options.tieBreakSeed}) {
        vertexNum_ = static_cast<int>(input.vertices.size());
        faceNum_ = static_cast<int>(input.faces.size());
        vertices_.resize(static_cast<size_t>(vertexNum_));
        faces_.resize(static_cast<size_t>(faceNum_));
        for (int i = 0; i < vertexNum_; ++i) {
            vertices_[static_cast<size_t>(i)].x = input.vertices[static_cast<size_t>(i)].x;
            vertices_[static_cast<size_t>(i)].y = input.vertices[static_cast<size_t>(i)].y;
            vertices_[static_cast<size_t>(i)].z = input.vertices[static_cast<size_t>(i)].z;
        }
        for (int i = 0; i < faceNum_; ++i) {
            Face& f = faces_[static_cast<size_t>(i)];
            f.v1 = input.faces[static_cast<size_t>(i)][0];
            f.v2 = input.faces[static_cast<size_t>(i)][1];
            f.v3 = input.faces[static_cast<size_t>(i)][2];
        }
        pre_.resize(static_cast<size_t>(vertexNum_));
        bound_.resize(static_cast<size_t>(vertexNum_), 0);
        edges_.resize(static_cast<size_t>(vertexNum_));
        origEdges_.resize(static_cast<size_t>(vertexNum_));
    }

    bool run(TriangleMesh* output, std::string* error);

  private:
    // ---- reference cost -------------------------------------------------
    void faceComputePara(int fi);
    void vertexComputeQ(int vi);
    double computeMinCost(int u, int v, double* outX, double* outY, double* outZ) const;
    bool flipPenalty(int w, int other, double qx, double qy, double qz) const;

    // ---- reference engine bookkeeping -----------------------------------
    bool valid(const Candidate& edge) const;
    bool liveVertex(int v) const;
    bool liveFace(int f) const;
    int liveVertexCount() const;
    int liveFaceCount() const;
    void protectDeletedNeighborhood(int deleted, std::vector<char>& cannotDelete);
    bool collapseEdge(const Candidate& edge, std::vector<char>* cannotDelete);
    void pushCandidate(int u, int v, double err, double x, double y, double z,
                       int timeCnt);
    std::pair<int, int> edgeKey(int a, int b) const {
        return a < b ? std::make_pair(a, b) : std::make_pair(b, a);
    }
    double dist(int u, int v) const;

    SimplifyOptions opts_;
    SimplifyStats* stats_ = nullptr;

    int vertexNum_ = 0;
    int faceNum_ = 0;
    std::vector<Vertex> vertices_;
    std::vector<Face> faces_;
    std::vector<std::set<int>> edges_;
    std::vector<std::set<int>> origEdges_;
    std::vector<int> pre_;      // union-find parent (representative == root)
    std::vector<char> bound_;   // reference boundary lock (initial open edges)
    std::map<std::pair<int, int>, int> timer_;
    std::priority_queue<Candidate, std::vector<Candidate>, CandidateCmp> heap_;
    long long nextSeq_ = 0;
    long long currentRound_ = 0;  // engine round id, stamped into CollapseRecord
};

inline double Engine::dist(int u, int v) const {
    const Vertex& a = vertices_[static_cast<size_t>(u)];
    const Vertex& b = vertices_[static_cast<size_t>(v)];
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

inline void Engine::faceComputePara(int fi) {
    Face& f = faces_[static_cast<size_t>(fi)];
    assert(f.v1 == pre_[static_cast<size_t>(f.v1)]);
    assert(f.v2 == pre_[static_cast<size_t>(f.v2)]);
    assert(f.v3 == pre_[static_cast<size_t>(f.v3)]);
    const Vertex& p1 = vertices_[static_cast<size_t>(f.v1)];
    const Vertex& p2 = vertices_[static_cast<size_t>(f.v2)];
    const Vertex& p3 = vertices_[static_cast<size_t>(f.v3)];
    const double x1 = p2.x - p1.x, y1 = p2.y - p1.y, z1 = p2.z - p1.z;
    const double x2 = p3.x - p1.x, y2 = p3.y - p1.y, z2 = p3.z - p1.z;
    double a = y1 * z2 - y2 * z1;
    double b = z1 * x2 - z2 * x1;
    double c = x1 * y2 - x2 * y1;
    const double len = std::sqrt(a * a + b * b + c * c);
    a /= len;
    b /= len;
    c /= len;
    f.a = a;
    f.b = b;
    f.c = c;
    f.d = -(a * p1.x + b * p1.y + c * p1.z);
}

inline void Engine::vertexComputeQ(int vi) {
    Vertex& v = vertices_[static_cast<size_t>(vi)];
    std::memset(v.Q, 0, sizeof(v.Q));
    for (int fi : v.faceList) {
        const Face& f = faces_[static_cast<size_t>(fi)];
        const double p[4] = {f.a, f.b, f.c, f.d};
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                v.Q[i][j] += p[i] * p[j];
            }
        }
    }
}

/// Paper Eq. (1): solve the 4x4 system with the homogeneous last row for the
/// optimal placement; cost is the quadric error at that placement.  The
/// elimination / fallback order matches the reference engine so the two can be
/// compared numerically.
inline double Engine::computeMinCost(int u, int v, double* outX, double* outY,
                                     double* outZ) const {
    assert(pre_[static_cast<size_t>(u)] == u);
    assert(pre_[static_cast<size_t>(v)] == v);
    const Vertex& vu = vertices_[static_cast<size_t>(u)];
    const Vertex& vv = vertices_[static_cast<size_t>(v)];

    double A[4][4];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            A[i][j] = vu.Q[i][j] + vv.Q[i][j];
        }
    }
    A[3][0] = A[3][1] = A[3][2] = 0.0;
    A[3][3] = 1.0;
    double b[4] = {0.0, 0.0, 0.0, 1.0};
    double x[4] = {0.0, 0.0, 0.0, 0.0};

    bool solved = true;
    for (int i = 0; i < 4; ++i) {
        int maxLine = i;
        for (int j = i + 1; j < 4; ++j) {
            if (std::fabs(A[j][i]) > std::fabs(A[maxLine][i])) maxLine = j;
        }
        for (int j = i; j < 4; ++j) std::swap(A[i][j], A[maxLine][j]);
        std::swap(b[i], b[maxLine]);
        const double t = A[i][i];
        if (std::fabs(t) < 1e-10) {
            x[0] = (vu.x + vv.x) / 2.0;
            x[1] = (vu.y + vv.y) / 2.0;
            x[2] = (vu.z + vv.z) / 2.0;
            x[3] = 1.0;
            solved = false;
            break;
        }
        for (int j = i; j < 4; ++j) A[i][j] /= t;
        b[i] /= t;
        for (int j = i + 1; j < 4; ++j) {
            if (std::fabs(A[j][i]) > 1e-8) {
                const double s = A[j][i];
                for (int k = i; k < 4; ++k) A[j][k] -= A[i][k] * s;
                b[j] -= b[i] * s;
            }
        }
    }
    if (solved) {
        for (int i = 3; i >= 0; --i) {
            x[i] = b[i];
            for (int k = i + 1; k < 4; ++k) x[i] -= A[i][k] * x[k];
        }
    }
    assert(std::fabs(x[3] - 1.0) < 1e-8);

    *outX = x[0];
    *outY = x[1];
    *outZ = x[2];

    double cost = 0.0;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            cost += x[i] * x[j] * (vu.Q[i][j] + vv.Q[i][j]);
        }
    }

    // Paper §6 flip penalty over every surviving face incident to u or v.
    if (flipPenalty(u, v, x[0], x[1], x[2]) ||
        flipPenalty(v, u, x[0], x[1], x[2])) {
        cost = 1e5;
    }
    return cost;
}

/// True if replacing vertex `w` by the placement q would invert the normal of
/// a surviving face incident to w (the reference's "inverse" check).
inline bool Engine::flipPenalty(int w, int other, double qx, double qy,
                                double qz) const {
    const Vertex& vw = vertices_[static_cast<size_t>(w)];
    for (int fi : vw.faceList) {
        const Face& f = faces_[static_cast<size_t>(fi)];
        if (f.v1 == other || f.v2 == other || f.v3 == other) continue;
        const Vertex& p1 = vertices_[static_cast<size_t>(f.v1)];
        const Vertex& p2 = vertices_[static_cast<size_t>(f.v2)];
        const Vertex& p3 = vertices_[static_cast<size_t>(f.v3)];
        double x1, y1, z1, x2, y2, z2, na, nb, nc, nna, nnb, nnc;
        if (f.v1 == w) {
            x1 = p2.x - p1.x; y1 = p2.y - p1.y; z1 = p2.z - p1.z;
            x2 = p3.x - p1.x; y2 = p3.y - p1.y; z2 = p3.z - p1.z;
            na = y1 * z2 - y2 * z1; nb = z1 * x2 - z2 * x1; nc = x1 * y2 - x2 * y1;
            x1 = p2.x - qx; y1 = p2.y - qy; z1 = p2.z - qz;
            x2 = p3.x - qx; y2 = p3.y - qy; z2 = p3.z - qz;
            nna = y1 * z2 - y2 * z1; nnb = z1 * x2 - z2 * x1; nnc = x1 * y2 - x2 * y1;
        } else if (f.v2 == w) {
            x1 = p1.x - p2.x; y1 = p1.y - p2.y; z1 = p1.z - p2.z;
            x2 = p3.x - p2.x; y2 = p3.y - p2.y; z2 = p3.z - p2.z;
            na = y1 * z2 - y2 * z1; nb = z1 * x2 - z2 * x1; nc = x1 * y2 - x2 * y1;
            x1 = p1.x - qx; y1 = p1.y - qy; z1 = p1.z - qz;
            x2 = p3.x - qx; y2 = p3.y - qy; z2 = p3.z - qz;
            nna = y1 * z2 - y2 * z1; nnb = z1 * x2 - z2 * x1; nnc = x1 * y2 - x2 * y1;
        } else {
            x1 = p1.x - p3.x; y1 = p1.y - p3.y; z1 = p1.z - p3.z;
            x2 = p2.x - p3.x; y2 = p2.y - p3.y; z2 = p2.z - p3.z;
            na = y1 * z2 - y2 * z1; nb = z1 * x2 - z2 * x1; nc = x1 * y2 - x2 * y1;
            x1 = p1.x - qx; y1 = p1.y - qy; z1 = p1.z - qz;
            x2 = p2.x - qx; y2 = p2.y - qy; z2 = p2.z - qz;
            nna = y1 * z2 - y2 * z1; nnb = z1 * x2 - z2 * x1; nnc = x1 * y2 - x2 * y1;
        }
        const double len1 = std::sqrt(na * na + nb * nb + nc * nc);
        const double len2 = std::sqrt(nna * nna + nnb * nnb + nnc * nnc);
        na /= len1; nb /= len1; nc /= len1;
        nna /= len2; nnb /= len2; nnc /= len2;
        if (na * nna + nb * nnb + nc * nnc <= 0.25) return true;
    }
    return false;
}

inline bool Engine::liveVertex(int u) const {
    return u >= 0 && u < vertexNum_ && pre_[static_cast<size_t>(u)] == u;
}

inline bool Engine::liveFace(int f) const {
    if (f < 0 || f >= faceNum_) return false;
    const Face& face = faces_[static_cast<size_t>(f)];
    return liveVertex(face.v1) && liveVertex(face.v2) && liveVertex(face.v3) &&
           face.v1 != face.v2 && face.v1 != face.v3 && face.v2 != face.v3;
}

inline int Engine::liveVertexCount() const {
    int count = 0;
    for (int i = 0; i < vertexNum_; ++i) {
        if (liveVertex(i)) ++count;
    }
    return count;
}

inline int Engine::liveFaceCount() const {
    int count = 0;
    for (int i = 0; i < faceNum_; ++i) {
        if (liveFace(i)) ++count;
    }
    return count;
}

inline bool Engine::valid(const Candidate& edge) const {
    if (edge.u != pre_[static_cast<size_t>(edge.u)]) return false;
    if (edge.v != pre_[static_cast<size_t>(edge.v)]) return false;
    if (bound_[static_cast<size_t>(edge.u)] || bound_[static_cast<size_t>(edge.v)]) {
        return false;
    }
    if (edge.err > 1e4) return false;
    auto it = timer_.find(edgeKey(edge.u, edge.v));
    if (it == timer_.end()) return false;
    return edge.timeCnt == it->second;
}

inline void Engine::protectDeletedNeighborhood(int deleted,
                                               std::vector<char>& cannotDelete) {
    if (deleted < 0 || deleted >= vertexNum_) return;
    cannotDelete[static_cast<size_t>(deleted)] = 1;
    const std::vector<int> neighbors(edges_[static_cast<size_t>(deleted)].begin(),
                                     edges_[static_cast<size_t>(deleted)].end());
    for (int v : neighbors) {
        if (liveVertex(v)) cannotDelete[static_cast<size_t>(v)] = 1;
    }
}

inline void Engine::pushCandidate(int u, int v, double err, double x, double y,
                                  double z, int timeCnt) {
    Candidate c;
    c.u = u;
    c.v = v;
    c.err = err;
    c.x = x;
    c.y = y;
    c.z = z;
    c.timeCnt = timeCnt;
    c.seq = ++nextSeq_;
    heap_.push(c);
}

inline bool Engine::collapseEdge(const Candidate& edge,
                                 std::vector<char>* cannotDelete) {
    if (!valid(edge)) return false;
    const int u = edge.u;
    const int v = edge.v;
    if (cannotDelete &&
        (*cannotDelete)[static_cast<size_t>(v)]) {
        return false;
    }
    assert(u < v);
    if (cannotDelete) protectDeletedNeighborhood(v, *cannotDelete);
    if (opts_.recorder) {
        // Snapshot the pre-collapse state for the paired progressive decoder.
        CollapseRecord record;
        record.survivor = u;
        record.victim = v;
        record.roundIndex = currentRound_;
        record.survivorPosBefore[0] = vertices_[static_cast<size_t>(u)].x;
        record.survivorPosBefore[1] = vertices_[static_cast<size_t>(u)].y;
        record.survivorPosBefore[2] = vertices_[static_cast<size_t>(u)].z;
        record.victimPosBefore[0] = vertices_[static_cast<size_t>(v)].x;
        record.victimPosBefore[1] = vertices_[static_cast<size_t>(v)].y;
        record.victimPosBefore[2] = vertices_[static_cast<size_t>(v)].z;
        record.fanFaces.reserve(vertices_[static_cast<size_t>(v)].faceList.size());
        for (int fi : vertices_[static_cast<size_t>(v)].faceList) {
            const Face& f = faces_[static_cast<size_t>(fi)];
            CollapseFanFace fan;
            if (f.v1 == v) {
                fan.a = f.v2;
                fan.b = f.v3;
            } else if (f.v2 == v) {
                fan.a = f.v3;
                fan.b = f.v1;
            } else {
                assert(f.v3 == v);
                fan.a = f.v1;
                fan.b = f.v2;
            }
            record.fanFaces.push_back(fan);
        }
        opts_.recorder->onCollapse(record);
    }
    pre_[static_cast<size_t>(v)] = u;
    vertices_[static_cast<size_t>(u)].x = edge.x;
    vertices_[static_cast<size_t>(u)].y = edge.y;
    vertices_[static_cast<size_t>(u)].z = edge.z;
    edges_[static_cast<size_t>(u)].erase(v);
    origEdges_[static_cast<size_t>(u)].erase(v);
    // Kill u's faces that contain v (the contracted edge's triangles) and
    // recompute the plane of u's surviving faces.
    for (auto it = vertices_[static_cast<size_t>(u)].faceList.begin();
         it != vertices_[static_cast<size_t>(u)].faceList.end();) {
        const int fi = *it;
        const Face& f = faces_[static_cast<size_t>(fi)];
        if (f.v1 == v || f.v2 == v || f.v3 == v) {
            ++it;  // erase(f) below may erase the element itself; advance first
            vertices_[static_cast<size_t>(f.v1)].faceList.erase(fi);
            vertices_[static_cast<size_t>(f.v2)].faceList.erase(fi);
            vertices_[static_cast<size_t>(f.v3)].faceList.erase(fi);
        } else {
            faceComputePara(fi);
            ++it;
        }
    }
    // Remap v's surviving faces to u, splicing the adjacency of the other two
    // corners from v to u.
    for (auto it = vertices_[static_cast<size_t>(v)].faceList.begin();
         it != vertices_[static_cast<size_t>(v)].faceList.end(); ++it) {
        const int fi = *it;
        Face& f = faces_[static_cast<size_t>(fi)];
        if (f.v1 == u || f.v2 == u || f.v3 == u) continue;
        if (f.v1 == v) {
            f.v1 = u;
            const int w2 = f.v2;
            const int w3 = f.v3;
            edges_[static_cast<size_t>(u)].insert(w2);
            origEdges_[static_cast<size_t>(u)].insert(w2);
            edges_[static_cast<size_t>(w2)].erase(v);
            origEdges_[static_cast<size_t>(w2)].erase(v);
            edges_[static_cast<size_t>(w2)].insert(u);
            origEdges_[static_cast<size_t>(w2)].insert(u);
            edges_[static_cast<size_t>(u)].insert(w3);
            origEdges_[static_cast<size_t>(u)].insert(w3);
            edges_[static_cast<size_t>(w3)].erase(v);
            origEdges_[static_cast<size_t>(w3)].erase(v);
            edges_[static_cast<size_t>(w3)].insert(u);
            origEdges_[static_cast<size_t>(w3)].insert(u);
        } else if (f.v2 == v) {
            f.v2 = u;
            const int w1 = f.v1;
            const int w3 = f.v3;
            edges_[static_cast<size_t>(u)].insert(w1);
            origEdges_[static_cast<size_t>(u)].insert(w1);
            edges_[static_cast<size_t>(w1)].erase(v);
            origEdges_[static_cast<size_t>(w1)].erase(v);
            edges_[static_cast<size_t>(w1)].insert(u);
            origEdges_[static_cast<size_t>(w1)].insert(u);
            edges_[static_cast<size_t>(u)].insert(w3);
            origEdges_[static_cast<size_t>(u)].insert(w3);
            edges_[static_cast<size_t>(w3)].erase(v);
            origEdges_[static_cast<size_t>(w3)].erase(v);
            edges_[static_cast<size_t>(w3)].insert(u);
            origEdges_[static_cast<size_t>(w3)].insert(u);
        } else if (f.v3 == v) {
            f.v3 = u;
            const int w1 = f.v1;
            const int w2 = f.v2;
            edges_[static_cast<size_t>(u)].insert(w1);
            origEdges_[static_cast<size_t>(u)].insert(w1);
            edges_[static_cast<size_t>(w1)].erase(v);
            origEdges_[static_cast<size_t>(w1)].erase(v);
            edges_[static_cast<size_t>(w1)].insert(u);
            origEdges_[static_cast<size_t>(w1)].insert(u);
            edges_[static_cast<size_t>(u)].insert(w2);
            origEdges_[static_cast<size_t>(u)].insert(w2);
            edges_[static_cast<size_t>(w2)].erase(v);
            origEdges_[static_cast<size_t>(w2)].erase(v);
            edges_[static_cast<size_t>(w2)].insert(u);
            origEdges_[static_cast<size_t>(w2)].insert(u);
        }
        faceComputePara(fi);
        vertices_[static_cast<size_t>(u)].faceList.insert(fi);
    }
    vertices_[static_cast<size_t>(v)].faceList.clear();
    edges_[static_cast<size_t>(v)].clear();
    origEdges_[static_cast<size_t>(v)].clear();

    // Exact quadric refresh for the survivor and its live one-ring, then local
    // candidate re-push with per-pair timers (stale heap entries are versioned
    // out by `valid`).
    vertexComputeQ(u);
    const std::vector<int> neighbors(edges_[static_cast<size_t>(u)].begin(),
                                     edges_[static_cast<size_t>(u)].end());
    for (int w : neighbors) {
        if (pre_[static_cast<size_t>(w)] != w) {
            edges_[static_cast<size_t>(u)].erase(w);
            origEdges_[static_cast<size_t>(u)].erase(w);
            continue;
        }
        vertexComputeQ(w);
        assert(u != w);
        const int newTime = ++timer_[edgeKey(u, w)];
        const bool isOrigEdge =
            origEdges_[static_cast<size_t>(u)].find(w) !=
            origEdges_[static_cast<size_t>(u)].end();
        if (isOrigEdge || dist(u, w) < opts_.pairThreshold) {
            double x = 0.0, y = 0.0, z = 0.0;
            const double err = computeMinCost(u, w, &x, &y, &z);
            if (u < w) {
                pushCandidate(u, w, err, x, y, z, newTime);
            } else {
                pushCandidate(w, u, err, x, y, z, newTime);
            }
        }
    }
    return true;
}

inline bool Engine::run(TriangleMesh* output, std::string* error) {
    // Optional non-edge pairs (paper §3.2 aggregation).  t=0 is the reference
    // calibration operating point and adds nothing; t>0 is brute forced and
    // therefore only practical on small meshes.
    if (opts_.pairThreshold > 0.0) {
        if (vertexNum_ > 8000) {
            if (error) {
                *error = "pair threshold t>0 brute force limited to <=8000 vertices";
            }
            return false;
        }
        for (int i = 0; i < vertexNum_; ++i) {
            for (int j = i + 1; j < vertexNum_; ++j) {
                if (dist(i, j) < opts_.pairThreshold) {
                    edges_[static_cast<size_t>(i)].insert(j);
                    edges_[static_cast<size_t>(j)].insert(i);
                }
            }
        }
    }

    // ---- initialization (reference order) -------------------------------
    for (int i = 0; i < vertexNum_; ++i) pre_[static_cast<size_t>(i)] = i;
    for (int fi = 0; fi < faceNum_; ++fi) {
        Face& f = faces_[static_cast<size_t>(fi)];
        if (f.v1 < 0 || f.v1 >= vertexNum_ || f.v2 < 0 || f.v2 >= vertexNum_ ||
            f.v3 < 0 || f.v3 >= vertexNum_ || f.v1 == f.v2 || f.v2 == f.v3 ||
            f.v1 == f.v3) {
            if (error) *error = "input OFF contains invalid or degenerate triangle";
            return false;
        }
        vertices_[static_cast<size_t>(f.v1)].faceList.insert(fi);
        vertices_[static_cast<size_t>(f.v2)].faceList.insert(fi);
        vertices_[static_cast<size_t>(f.v3)].faceList.insert(fi);
    }
    for (int fi = 0; fi < faceNum_; ++fi) faceComputePara(fi);
    for (int vi = 0; vi < vertexNum_; ++vi) vertexComputeQ(vi);
    for (int fi = 0; fi < faceNum_; ++fi) {
        const Face& f = faces_[static_cast<size_t>(fi)];
        edges_[static_cast<size_t>(f.v1)].insert(f.v2);
        edges_[static_cast<size_t>(f.v1)].insert(f.v3);
        edges_[static_cast<size_t>(f.v2)].insert(f.v1);
        edges_[static_cast<size_t>(f.v2)].insert(f.v3);
        edges_[static_cast<size_t>(f.v3)].insert(f.v1);
        edges_[static_cast<size_t>(f.v3)].insert(f.v2);
        origEdges_[static_cast<size_t>(f.v1)].insert(f.v2);
        origEdges_[static_cast<size_t>(f.v1)].insert(f.v3);
        origEdges_[static_cast<size_t>(f.v2)].insert(f.v1);
        origEdges_[static_cast<size_t>(f.v2)].insert(f.v3);
        origEdges_[static_cast<size_t>(f.v3)].insert(f.v1);
        origEdges_[static_cast<size_t>(f.v3)].insert(f.v2);
    }

    // Reference boundary lock: an edge with fewer than two incident faces marks
    // both endpoints as boundary (non-removable) for the whole run.
    for (int u = 0; u < vertexNum_; ++u) {
        if (bound_[static_cast<size_t>(u)]) continue;
        const std::vector<int> neighbors(edges_[static_cast<size_t>(u)].begin(),
                                         edges_[static_cast<size_t>(u)].end());
        for (int v : neighbors) {
            int faceCnt = 0;
            for (int fi : vertices_[static_cast<size_t>(u)].faceList) {
                const Face& f = faces_[static_cast<size_t>(fi)];
                if (f.v1 == v || f.v2 == v || f.v3 == v) ++faceCnt;
            }
            if (faceCnt < 2) {
                bound_[static_cast<size_t>(u)] = 1;
                bound_[static_cast<size_t>(v)] = 1;
            }
        }
    }

    // ---- initial candidates ---------------------------------------------
    for (int u = 0; u < vertexNum_; ++u) {
        for (int v : edges_[static_cast<size_t>(u)]) {
            if (u > v) continue;
            timer_[std::make_pair(u, v)] = 1;
            double x = 0.0, y = 0.0, z = 0.0;
            const double err = computeMinCost(u, v, &x, &y, &z);
            pushCandidate(u, v, err, x, y, z, 1);
        }
    }

    const int targetCollapses = (faceNum_ - opts_.targetFaces) / 2;
    stats_->inputVertices = vertexNum_;
    stats_->inputFaces = faceNum_;
    stats_->targetCollapses = targetCollapses;
    if (targetCollapses < 0) {
        if (error) *error = "targetFaces exceeds input face count";
        return false;
    }

    // ---- reference round schedule ---------------------------------------
    long long totalCollapsed = 0;
    long long roundId = 0;
    double totalCost = 0.0;
    while (totalCollapsed < targetCollapses) {
        ++roundId;
        currentRound_ = roundId;
        std::vector<char> cannotDelete(static_cast<size_t>(vertexNum_), 0);
        std::vector<Candidate> deferred;
        long long collapsedThisRound = 0;
        while (totalCollapsed < targetCollapses) {
            Candidate edge;
            bool found = false;
            while (!heap_.empty()) {
                while (!heap_.empty() && !valid(heap_.top())) heap_.pop();
                if (heap_.empty()) break;
                edge = heap_.top();
                heap_.pop();
                if (cannotDelete[static_cast<size_t>(edge.v)]) {
                    // Defer, do not drop: the protection set resets next round.
                    deferred.push_back(edge);
                    continue;
                }
                found = true;
                break;
            }
            if (!found) break;
            if (collapseEdge(edge, &cannotDelete)) {
                totalCost += edge.err;
                ++totalCollapsed;
                ++collapsedThisRound;
            }
        }
        for (const Candidate& c : deferred) heap_.push(c);
        if (collapsedThisRound == 0 && totalCollapsed < targetCollapses) {
            stats_->stalled = true;
            break;
        }
        if (opts_.verbose) {
            std::fprintf(stderr,
                         "[QEMRefEngine] round=%lld collapsed=%lld total=%lld "
                         "live_vertices=%d live_faces=%d\n",
                         roundId, collapsedThisRound, totalCollapsed,
                         liveVertexCount(), liveFaceCount());
        }
    }

    stats_->collapseCount = totalCollapsed;
    stats_->roundCount = roundId;
    stats_->totalCost = totalCost;
    stats_->reachedTarget = totalCollapsed >= targetCollapses;

    // ---- export snapshot ------------------------------------------------
    std::vector<int> outId(static_cast<size_t>(vertexNum_), -1);
    int outVertexCount = 0;
    for (int i = 0; i < vertexNum_; ++i) {
        if (liveVertex(i)) outId[static_cast<size_t>(i)] = outVertexCount++;
    }
    output->vertices.clear();
    output->vertices.reserve(static_cast<size_t>(outVertexCount));
    for (int i = 0; i < vertexNum_; ++i) {
        if (outId[static_cast<size_t>(i)] < 0) continue;
        output->vertices.push_back(
            Vec3d{vertices_[static_cast<size_t>(i)].x,
                  vertices_[static_cast<size_t>(i)].y,
                  vertices_[static_cast<size_t>(i)].z});
    }
    output->faces.clear();
    for (int fi = 0; fi < faceNum_; ++fi) {
        if (!liveFace(fi)) continue;
        const Face& f = faces_[static_cast<size_t>(fi)];
        output->faces.push_back(std::array<int, 3>{
            outId[static_cast<size_t>(f.v1)], outId[static_cast<size_t>(f.v2)],
            outId[static_cast<size_t>(f.v3)]});
    }
    stats_->outputVertices = static_cast<int>(output->vertices.size());
    stats_->outputFaces = static_cast<int>(output->faces.size());
    stats_->outputVertexOriginalIds.clear();
    stats_->outputVertexOriginalIds.reserve(static_cast<size_t>(outVertexCount));
    for (int i = 0; i < vertexNum_; ++i) {
        if (outId[static_cast<size_t>(i)] >= 0) {
            stats_->outputVertexOriginalIds.push_back(i);
        }
    }
    return true;
}

}  // namespace detail

// ===========================================================================
// OFF I/O helpers (triangle OFF only; matches what the E250 harness feeds in)
// ===========================================================================

inline bool loadOffMesh(const std::string& path, TriangleMesh* mesh,
                        std::string* error) {
    std::ifstream in(path);
    if (!in) {
        if (error) *error = "cannot open input mesh: " + path;
        return false;
    }
    std::string header;
    if (!std::getline(in, header)) {
        if (error) *error = "empty input mesh: " + path;
        return false;
    }
    std::istringstream hs(header);
    std::string tag;
    hs >> tag;
    if (tag != "OFF") {
        if (error) *error = "unsupported OFF header '" + tag + "' (expected OFF)";
        return false;
    }
    long long nv = -1, nf = -1, ne = 0;
    bool countsOnHeader = static_cast<bool>(hs >> nv >> nf);
    if (countsOnHeader) {
        hs >> ne;
    } else {
        if (!(in >> nv >> nf)) {
            if (error) *error = "missing OFF vertex/face counts";
            return false;
        }
        if (!(in >> ne)) ne = 0;
    }
    if (nv < 0 || nf < 0 || nv > 100000000 || nf > 100000000) {
        if (error) *error = "invalid OFF counts";
        return false;
    }
    mesh->vertices.clear();
    mesh->faces.clear();
    mesh->vertices.reserve(static_cast<size_t>(nv));
    mesh->faces.reserve(static_cast<size_t>(nf));
    for (long long i = 0; i < nv; ++i) {
        std::string line;
        do {
            if (!std::getline(in, line)) {
                if (error) *error = "truncated OFF vertex list";
                return false;
            }
        } while (line.find_first_not_of(" \t\r\n") == std::string::npos);
        std::istringstream ls(line);
        Vec3d p;
        if (!(ls >> p.x >> p.y >> p.z)) {
            if (error) *error = "malformed OFF vertex line";
            return false;
        }
        mesh->vertices.push_back(p);
    }
    for (long long i = 0; i < nf; ++i) {
        std::string line;
        do {
            if (!std::getline(in, line)) {
                if (error) *error = "truncated OFF face list";
                return false;
            }
        } while (line.find_first_not_of(" \t\r\n") == std::string::npos);
        std::istringstream ls(line);
        int corners = 0;
        if (!(ls >> corners) || corners != 3) {
            if (error) *error = "non-triangle OFF face unsupported";
            return false;
        }
        int a = -1, b = -1, c = -1;
        if (!(ls >> a >> b >> c)) {
            if (error) *error = "malformed OFF face line";
            return false;
        }
        mesh->faces.push_back(std::array<int, 3>{a, b, c});
    }
    return true;
}

/// decimals == 0 writes full precision (max_digits10); decimals == 4 reproduces
/// the reference harness's `%.4lf` OBJ round-trip so outputs can be diffed.
inline bool writeOffMesh(const std::string& path, const TriangleMesh& mesh,
                         std::string* error, int decimals = 0) {
    std::ofstream out(path);
    if (!out) {
        if (error) *error = "cannot open output mesh: " + path;
        return false;
    }
    out << "OFF\n"
        << mesh.vertices.size() << " " << mesh.faces.size() << " 0\n";
    if (decimals > 0) {
        out << std::fixed << std::setprecision(decimals);
    } else {
        out << std::setprecision(std::numeric_limits<double>::max_digits10);
    }
    for (const Vec3d& p : mesh.vertices) {
        out << p.x << " " << p.y << " " << p.z << "\n";
    }
    for (const std::array<int, 3>& f : mesh.faces) {
        out << "3 " << f[0] << " " << f[1] << " " << f[2] << "\n";
    }
    if (!out) {
        if (error) *error = "failed writing output mesh: " + path;
        return false;
    }
    return true;
}

inline SimplifyStats simplifyMesh(const TriangleMesh& input,
                                  TriangleMesh* output,
                                  const SimplifyOptions& options,
                                  std::string* error) {
    SimplifyStats stats;
    detail::Engine engine(input, options, &stats);
    if (!engine.run(output, error)) {
        stats = SimplifyStats{};
    }
    return stats;
}

}  // namespace qem_refstyle
}  // namespace pmsf

#endif  // PMSF_IMPL_QEM_QEM_REF_STYLE_ENGINE_H
