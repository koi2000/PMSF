#include "boundary/BoundaryTopologyPatchCodec.h"

#include "common/BoundedRingTraversal.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <climits>
#include <iostream>
#include <map>
#include "common/CachedEnv.h"

namespace pmsf {
namespace {

void putBit(std::vector<unsigned char>& bits, int b) {
    bits.push_back(static_cast<unsigned char>(b & 1));
}

void putBits(std::vector<unsigned char>& bits, std::uint32_t v, int n) {
    for (int i = n - 1; i >= 0; --i) putBit(bits, (v >> i) & 1u);
}

void putExpGolomb(std::vector<unsigned char>& bits, std::uint32_t v) {
    const std::uint32_t x = v + 1u;
    int n = 0;
    while ((x >> n) > 1u) ++n;
    for (int i = 0; i < n; ++i) putBit(bits, 0);
    for (int i = n; i >= 0; --i) putBit(bits, (x >> i) & 1u);
}

struct BitReader {
    const std::vector<unsigned char>& bits;
    std::size_t at = 0;
    bool ok = true;
    explicit BitReader(const std::vector<unsigned char>& b) : bits(b) {}
    int getBit() {
        if (at >= bits.size()) { ok = false; return 0; }
        return bits[at++] & 1;
    }
    std::uint32_t getBits(int n) {
        std::uint32_t v = 0;
        for (int i = 0; i < n; ++i) v = (v << 1) | static_cast<std::uint32_t>(getBit());
        return v;
    }
    std::uint32_t getExpGolomb() {
        int n = 0;
        while (ok && getBit() == 0) ++n;
        std::uint32_t x = 1u;
        for (int i = 0; i < n; ++i) x = (x << 1) | static_cast<std::uint32_t>(getBit());
        return x - 1u;
    }
};

std::uint32_t zigzag(int v) {
    return (static_cast<std::uint32_t>(v) << 1) ^ static_cast<std::uint32_t>(v >> 31);
}
int unzigzag(std::uint32_t z) {
    return static_cast<int>((z >> 1) ^ static_cast<std::uint32_t>(-static_cast<int>(z & 1u)));
}

struct FaceKey {
    std::array<std::array<int, 3>, 3> v{};
    MCGAL::Facet* f = nullptr;
    bool operator<(const FaceKey& o) const { return v < o.v; }
};

int coarseQMatchTol() {
    const char* v = PMSF_CACHED_ENV("PMSF_BTP_COARSE_TOL");
    if (!v || !v[0]) return 1;
    const int t = std::atoi(v);
    return t < 0 ? 0 : (t > 32 ? 32 : t);
}

}  // namespace

std::vector<unsigned char> encodeBoundaryTopologyPatchBits(
    const std::vector<BoundaryTopologyPatchOp>& inputOps) {
    std::vector<unsigned char> bits;
    std::vector<BoundaryTopologyPatchOp> ops = inputOps;
    std::sort(ops.begin(), ops.end(), [](const auto& a, const auto& b) {
        if (a.round != b.round) return a.round < b.round;
        if (a.qx != b.qx) return a.qx < b.qx;
        if (a.qy != b.qy) return a.qy < b.qy;
        return a.qz < b.qz;
    });
    // v2 marker.
    putBit(bits, 1);
    std::vector<std::pair<int, std::pair<std::size_t, std::size_t>>> groups;  // round,[begin,end)
    for (std::size_t i = 0; i < ops.size();) {
        std::size_t j = i + 1;
        while (j < ops.size() && ops[j].round == ops[i].round) ++j;
        groups.push_back({ops[i].round, {i, j}});
        i = j;
    }
    putBits(bits, static_cast<std::uint32_t>(groups.size()) & 63u, 6);
    for (const auto& g : groups) {
        putBits(bits, static_cast<std::uint32_t>(g.first) & 63u, 6);
        const std::size_t n = g.second.second - g.second.first;
        putExpGolomb(bits, static_cast<std::uint32_t>(n - 1));
        int prevX = 0, prevY = 0, prevZ = 0;
        bool first = true;
        for (std::size_t i = g.second.first; i < g.second.second; ++i) {
            const auto& op = ops[i];
            // Coarse identity: q >> 3, 9 bits per axis.
            putBits(bits, static_cast<std::uint32_t>((op.qx >> 3) & 511), 9);
            putBits(bits, static_cast<std::uint32_t>((op.qy >> 3) & 511), 9);
            putBits(bits, static_cast<std::uint32_t>((op.qz >> 3) & 511), 9);
            (void)first;
            (void)prevX; (void)prevY; (void)prevZ;
            const int d = static_cast<int>(op.gids.size());
            putBits(bits, static_cast<std::uint32_t>(d) & 31u, 5);
            // dominant gid + change bitmap + changed gids
            int dominant = 0, bestCount = -1;
            for (int cand = 0; cand < 32; ++cand) {
                int c = 0;
                for (int gid : op.gids) if (gid == cand) ++c;
                if (c > bestCount) { bestCount = c; dominant = cand; }
            }
            putBits(bits, static_cast<std::uint32_t>(dominant) & 31u, 5);
            for (int i2 = 0; i2 < d; ++i2)
                putBit(bits, op.gids[static_cast<std::size_t>(i2)] != dominant ? 1 : 0);
            for (int i2 = 0; i2 < d; ++i2)
                if (op.gids[static_cast<std::size_t>(i2)] != dominant)
                    putBits(bits, static_cast<std::uint32_t>(op.gids[static_cast<std::size_t>(i2)]) & 31u, 5);
        }
    }
    return bits;
}

bool parseBoundaryTopologyPatchBits(const std::vector<unsigned char>& bits,
                                    std::vector<BoundaryTopologyPatchOp>& ops) {
    ops.clear();
    BitReader r(bits);
    const int version = r.getBit();
    if (!r.ok) return false;
    if (version == 0) {
        // Legacy v1.
        int prevX = 0, prevY = 0, prevZ = 0;
        bool first = true;
        while (r.ok && r.at < bits.size()) {
            BoundaryTopologyPatchOp op;
            op.round = static_cast<int>(r.getBits(6));
            if (first) {
                op.qx = static_cast<int>(r.getBits(12));
                op.qy = static_cast<int>(r.getBits(12));
                op.qz = static_cast<int>(r.getBits(12));
                first = false;
            } else {
                op.qx = prevX + unzigzag(r.getExpGolomb());
                op.qy = prevY + unzigzag(r.getExpGolomb());
                op.qz = prevZ + unzigzag(r.getExpGolomb());
            }
            if (!r.ok) return false;
            prevX = op.qx; prevY = op.qy; prevZ = op.qz;
            const int d = static_cast<int>(r.getBits(5));
            if (d <= 0) return false;
            op.gids.resize(static_cast<std::size_t>(d));
            for (int i = 0; i < d; ++i)
                op.gids[static_cast<std::size_t>(i)] = static_cast<int>(r.getBits(5));
            ops.push_back(std::move(op));
        }
        return r.ok;
    }
    const int groupCount = static_cast<int>(r.getBits(6));
    for (int gi = 0; gi < groupCount; ++gi) {
        const int round = static_cast<int>(r.getBits(6));
        const std::uint32_t nMinus1 = r.getExpGolomb();
        const std::size_t n = static_cast<std::size_t>(nMinus1) + 1;
        int prevX = 0, prevY = 0, prevZ = 0;
        for (std::size_t i = 0; i < n; ++i) {
            BoundaryTopologyPatchOp op;
            op.round = round;
            op.qx = static_cast<int>(r.getBits(9)) << 3;
            op.qy = static_cast<int>(r.getBits(9)) << 3;
            op.qz = static_cast<int>(r.getBits(9)) << 3;
            if (!r.ok) return false;
            const int d = static_cast<int>(r.getBits(5));
            if (d <= 0 || d > 31) return false;
            const int dominant = static_cast<int>(r.getBits(5));
            std::vector<int> changed;
            for (int i2 = 0; i2 < d; ++i2)
                if (r.getBit()) changed.push_back(1);
                else changed.push_back(0);
            op.gids.assign(static_cast<std::size_t>(d), dominant);
            for (int i2 = 0; i2 < d; ++i2) {
                if (!changed[static_cast<std::size_t>(i2)]) continue;
                op.gids[static_cast<std::size_t>(i2)] = static_cast<int>(r.getBits(5));
            }
            if (!r.ok) return false;
            ops.push_back(std::move(op));
        }
    }
    return r.ok;
}

int applyBoundaryTopologyPatchRound(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                    const std::vector<unsigned char>& bits,
                                    int round) {
    if (!mesh) return 0;
    std::vector<BoundaryTopologyPatchOp> ops;
    const bool parsed = parseBoundaryTopologyPatchBits(bits, ops);
    if (PMSF_CACHED_ENV("PMSF_BOUNDARY_DEBUG")) {
        int rmin = 9999, rmax = -1;
        for (const auto& op : ops) { rmin = std::min(rmin, op.round); rmax = std::max(rmax, op.round); }
        std::cerr << "[btp-codec] parsed=" << parsed << " ops=" << ops.size()
                  << " bits=" << bits.size() << " roundRange=" << rmin << ".." << rmax;
        if (!ops.empty()) std::cerr << " first=" << ops.front().round << "@" << ops.front().qx << "," << ops.front().qy << "," << ops.front().qz;
        std::cerr << std::endl;
    }
    if (!parsed) return 0;
    std::map<MCGAL::Vertex*, std::vector<MCGAL::Facet*>> incidence;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* hh = h0;
        int guard = 0;
        do {
            if (hh && hh->vertex()) incidence[hh->vertex()].push_back(f);
            hh = hh ? hh->next() : nullptr;
        } while (hh && hh != h0 && ++guard < 64);
    }
    int applied = 0;
    for (const auto& op : ops) {
        if (op.round != round) continue;
        MCGAL::Vertex* center = nullptr;
        int bestDist = INT_MAX;
        for (const auto& kv : incidence) {
            MCGAL::Vertex* v = kv.first;
            if (!v || v->isRemoved()) continue;
            if (static_cast<int>(kv.second.size()) != static_cast<int>(op.gids.size())) continue;
            auto q = mesh->floatPosToInt(v->point());
            const int d = std::abs((q.x() >> 3) - (op.qx >> 3)) +
                          std::abs((q.y() >> 3) - (op.qy >> 3)) +
                          std::abs((q.z() >> 3) - (op.qz >> 3));
            if (d < bestDist && d <= coarseQMatchTol() * 3 + 3) { bestDist = d; center = v; }
        }
        if (!center) continue;
        std::vector<FaceKey> faces;
        for (MCGAL::Facet* cand : incidence[center]) {
            if (!cand || cand->isRemoved() || !cand->proxyHalfedge()) continue;
            FaceKey fk;
            fk.f = cand;
            int n = 0;
            MCGAL::Halfedge* h0 = cand->proxyHalfedge();
            MCGAL::Halfedge* hh = h0;
            int guard = 0;
            do {
                if (hh && hh->vertex() && n < 3) {
                    auto q = mesh->floatPosToInt(hh->vertex()->point());
                    fk.v[static_cast<std::size_t>(n++)] = {q.x(), q.y(), q.z()};
                }
                hh = hh ? hh->next() : nullptr;
            } while (hh && hh != h0 && ++guard < 64);
            if (n != 3) continue;
            std::sort(fk.v.begin(), fk.v.end());
            faces.push_back(fk);
        }
        std::sort(faces.begin(), faces.end());
        if (faces.size() != op.gids.size()) continue;
        for (std::size_t i = 0; i < faces.size(); ++i) {
            if (faces[i].f) faces[i].f->setGroupId(op.gids[i]);
        }
        center->setGroupId(op.gids.empty() ? center->groupId() : op.gids.front());
        ++applied;
    }
    if (PMSF_CACHED_ENV("PMSF_BOUNDARY_DEBUG"))
        std::cerr << "[btp-apply-robust] round=" << round << " applied=" << applied
                  << " V=" << mesh->size_of_vertices() << " F=" << mesh->size_of_facets() << std::endl;
    return applied;
}

std::vector<BoundaryTopologyPatchOp>& pendingBoundaryTopologyOps() {
    static std::vector<BoundaryTopologyPatchOp> ops;
    return ops;
}
int& pendingBoundaryTopologyRound() {
    static int r = -1;
    return r;
}

bool applyBoundaryTopologyAtVertex(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                   MCGAL::Vertex* center,
                                   int round) {
    if (!mesh || !center) return false;
    if (const char* orderDec = PMSF_CACHED_ENV("PMSF_BTP_ORDER_DEC")) {
        FILE* of = std::fopen(orderDec, "a");
        if (of) {
            auto q = mesh->floatPosToInt(center->point());
            std::fprintf(of, "%d %d %d %d\n", round, q.x(), q.y(), q.z());
            std::fclose(of);
        }
    }
    static int callCount = 0;
    if (PMSF_CACHED_ENV("PMSF_BOUNDARY_DEBUG") && callCount < 8) {
        ++callCount;
        auto cq0 = mesh->floatPosToInt(center->point());
        int shown = 0;
        std::cerr << "[btp-insert] call=" << callCount << " round=" << round
                  << " pending=" << pendingBoundaryTopologyOps().size()
                  << " centerQ=" << cq0.x() << "," << cq0.y() << "," << cq0.z();
        for (const auto& op : pendingBoundaryTopologyOps()) {
            if (op.round != round || shown >= 3) continue;
            std::cerr << " op=" << op.qx << "," << op.qy << "," << op.qz;
            ++shown;
        }
        std::cerr << std::endl;
    }
    auto& ops = pendingBoundaryTopologyOps();
    auto cq = mesh->floatPosToInt(center->point());
    int found = -1;
    int tol = 2;
    if (const char* t = PMSF_CACHED_ENV("PMSF_BTP_TOL")) tol = std::max(0, std::atoi(t));
    const bool ignoreRound = PMSF_CACHED_ENV("PMSF_BTP_IGNORE_ROUND") != nullptr;
    for (std::size_t i = 0; i < ops.size(); ++i) {
        const auto& op = ops[i];
        if (!ignoreRound && op.round != round) continue;
        const int ctol = coarseQMatchTol();
        if (std::abs((cq.x() >> 3) - (op.qx >> 3)) <= ctol &&
            std::abs((cq.y() >> 3) - (op.qy >> 3)) <= ctol &&
            std::abs((cq.z() >> 3) - (op.qz >> 3)) <= ctol) {
            found = static_cast<int>(i); break;
        }
    }
    if (found < 0) return false;
    const auto op = ops[static_cast<std::size_t>(found)];
    std::vector<FaceKey> faces;
    // The reconstructed center vertex may not have a populated halfedge ring at
    // this point; collect its incident triangles by scanning the mesh instead.
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        FaceKey fk;
        fk.f = f;
        int n = 0;
        bool touches = false;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* hh = h0;
        int guard = 0;
        do {
            if (hh && hh->vertex() && n < 3) {
                MCGAL::Vertex* v = hh->vertex();
                if (v == center) touches = true;
                auto q = mesh->floatPosToInt(v->point());
                fk.v[static_cast<std::size_t>(n++)] = {q.x(), q.y(), q.z()};
            }
            hh = hh ? hh->next() : nullptr;
        } while (hh && hh != h0 && ++guard < 64);
        if (n != 3 || !touches) continue;
        std::sort(fk.v.begin(), fk.v.end());
        faces.push_back(fk);
    }
    std::sort(faces.begin(), faces.end());
    if (PMSF_CACHED_ENV("PMSF_BOUNDARY_DEBUG"))
        std::cerr << "[btp-insert-match] round=" << round << " q=" << op.qx << ","
                  << op.qy << "," << op.qz << " faces=" << faces.size()
                  << " gids=" << op.gids.size() << std::endl;
    if (faces.size() != op.gids.size()) return false;
    for (std::size_t i = 0; i < faces.size(); ++i) {
        if (faces[i].f) faces[i].f->setGroupId(op.gids[i]);
    }
    center->setGroupId(op.gids.empty() ? center->groupId() : op.gids.front());
    if (const char* ad = PMSF_CACHED_ENV("PMSF_BTP_PATCH_APPLY_ORDER")) {
        if (FILE* af = std::fopen(ad, "a")) {
            auto q = mesh->floatPosToInt(center->point());
            std::fprintf(af, "%d %d %d %d %d %d\n", round, q.x(), q.y(), q.z(),
                         found, static_cast<int>(op.gids.size()));
            std::fclose(af);
        }
    }
    ops.erase(ops.begin() + found);
    return true;
}

}  // namespace pmsf
