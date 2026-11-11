#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
};

double dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

double norm2(const Vec3& v) {
    return dot(v, v);
}

Vec3 normalized(const Vec3& v) {
    const double n2 = norm2(v);
    if (n2 <= 0.0) return {};
    return v / std::sqrt(n2);
}

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

constexpr double kPpmcPlanarityEpsilon = 0.05;

double cross2(const Vec2& a, const Vec2& b, const Vec2& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool onSegment2(const Vec2& a, const Vec2& b, const Vec2& p) {
    const double eps = 1e-12;
    return std::fabs(cross2(a, b, p)) <= eps &&
           p.x >= std::min(a.x, b.x) - eps && p.x <= std::max(a.x, b.x) + eps &&
           p.y >= std::min(a.y, b.y) - eps && p.y <= std::max(a.y, b.y) + eps;
}

bool segmentsIntersect2(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d) {
    const double c1 = cross2(a, b, c);
    const double c2 = cross2(a, b, d);
    const double c3 = cross2(c, d, a);
    const double c4 = cross2(c, d, b);
    const double eps = 1e-12;
    if (((c1 > eps && c2 < -eps) || (c1 < -eps && c2 > eps)) &&
        ((c3 > eps && c4 < -eps) || (c3 < -eps && c4 > eps))) {
        return true;
    }
    return onSegment2(a, b, c) || onSegment2(a, b, d) ||
           onSegment2(c, d, a) || onSegment2(c, d, b);
}

Vec2 projectPoint(const Vec3& p, const Vec3& n) {
    const double ax = std::fabs(n.x);
    const double ay = std::fabs(n.y);
    const double az = std::fabs(n.z);
    if (ax >= ay && ax >= az) return {p.y, p.z};
    if (ay >= ax && ay >= az) return {p.x, p.z};
    return {p.x, p.y};
}

struct Vertex {
    Vec3 p;
    bool alive = true;
    std::array<double, 10> q{};
};

struct Face {
    std::array<int, 3> v{};
    bool alive = true;
};

struct TriKey {
    int a = 0;
    int b = 0;
    int c = 0;

    bool operator==(const TriKey& o) const {
        return a == o.a && b == o.b && c == o.c;
    }
};

struct TriKeyHash {
    std::size_t operator()(const TriKey& key) const {
        std::uint64_t h = 1469598103934665603ull;
        auto mix = [&](std::uint32_t v) {
            h ^= v;
            h *= 1099511628211ull;
        };
        mix(static_cast<std::uint32_t>(key.a));
        mix(static_cast<std::uint32_t>(key.b));
        mix(static_cast<std::uint32_t>(key.c));
        return static_cast<std::size_t>(h);
    }
};

TriKey canonicalTri(int a, int b, int c) {
    std::array<int, 3> ids{a, b, c};
    std::sort(ids.begin(), ids.end());
    return {ids[0], ids[1], ids[2]};
}

std::uint64_t edgeKey(int a, int b) {
    if (a > b) std::swap(a, b);
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(a)) << 32) |
           static_cast<std::uint32_t>(b);
}

int edgeA(std::uint64_t key) {
    return static_cast<int>(key >> 32);
}

int edgeB(std::uint64_t key) {
    return static_cast<int>(key & 0xffffffffu);
}

bool faceContains(const Face& f, int v) {
    return f.v[0] == v || f.v[1] == v || f.v[2] == v;
}

bool faceContainsBoth(const Face& f, int a, int b) {
    return faceContains(f, a) && faceContains(f, b);
}

int thirdVertex(const Face& f, int a, int b) {
    for (int v : f.v) {
        if (v != a && v != b) return v;
    }
    return -1;
}

struct Mesh {
    std::vector<Vertex> vertices;
    std::vector<Face> faces;
    double diag2 = 1.0;
    double minArea2 = 1e-30;
    double minEdge2 = 1e-24;

    Vec3 pointFor(int id, int movedId, const Vec3& moved) const {
        return id == movedId ? moved : vertices[id].p;
    }

    Vec3 faceNormal(const Face& f) const {
        const Vec3 a = vertices[f.v[0]].p;
        const Vec3 b = vertices[f.v[1]].p;
        const Vec3 c = vertices[f.v[2]].p;
        return normalized(cross(b - a, c - a));
    }

    double faceArea2(const Face& f) const {
        const Vec3 a = vertices[f.v[0]].p;
        const Vec3 b = vertices[f.v[1]].p;
        const Vec3 c = vertices[f.v[2]].p;
        return norm2(cross(b - a, c - a));
    }

    void updateScale() {
        Vec3 mn{std::numeric_limits<double>::max(),
                std::numeric_limits<double>::max(),
                std::numeric_limits<double>::max()};
        Vec3 mx{std::numeric_limits<double>::lowest(),
                std::numeric_limits<double>::lowest(),
                std::numeric_limits<double>::lowest()};
        bool any = false;
        for (const auto& v : vertices) {
            if (!v.alive) continue;
            any = true;
            mn.x = std::min(mn.x, v.p.x);
            mn.y = std::min(mn.y, v.p.y);
            mn.z = std::min(mn.z, v.p.z);
            mx.x = std::max(mx.x, v.p.x);
            mx.y = std::max(mx.y, v.p.y);
            mx.z = std::max(mx.z, v.p.z);
        }
        if (!any) {
            diag2 = 1.0;
        } else {
            diag2 = std::max(norm2(mx - mn), 1e-24);
        }
        minArea2 = std::max(1e-30, diag2 * diag2 * 1e-18);
        minEdge2 = std::max(1e-24, diag2 * 1e-14);
    }

    int aliveFaceCount() const {
        int n = 0;
        for (const auto& f : faces) {
            if (f.alive) ++n;
        }
        return n;
    }

    int aliveVertexCount() const {
        int n = 0;
        for (const auto& v : vertices) {
            if (v.alive) ++n;
        }
        return n;
    }

    void cleanForDisplay() {
        updateScale();

        for (auto& f : faces) {
            if (!f.alive) continue;
            if (f.v[0] == f.v[1] || f.v[1] == f.v[2] || f.v[2] == f.v[0]) {
                f.alive = false;
                continue;
            }
            if (!vertices[f.v[0]].alive || !vertices[f.v[1]].alive || !vertices[f.v[2]].alive) {
                f.alive = false;
                continue;
            }
            const double e0 = norm2(vertices[f.v[0]].p - vertices[f.v[1]].p);
            const double e1 = norm2(vertices[f.v[1]].p - vertices[f.v[2]].p);
            const double e2 = norm2(vertices[f.v[2]].p - vertices[f.v[0]].p);
            if (e0 <= minEdge2 || e1 <= minEdge2 || e2 <= minEdge2 || faceArea2(f) <= minArea2) {
                f.alive = false;
            }
        }

        std::unordered_set<TriKey, TriKeyHash> seenFaces;
        std::unordered_map<std::uint64_t, int> edgeUse;
        for (auto& f : faces) {
            if (!f.alive) continue;
            const TriKey tk = canonicalTri(f.v[0], f.v[1], f.v[2]);
            if (!seenFaces.insert(tk).second) {
                f.alive = false;
                continue;
            }

            const std::uint64_t e0 = edgeKey(f.v[0], f.v[1]);
            const std::uint64_t e1 = edgeKey(f.v[1], f.v[2]);
            const std::uint64_t e2 = edgeKey(f.v[2], f.v[0]);
            if (edgeUse[e0] >= 2 || edgeUse[e1] >= 2 || edgeUse[e2] >= 2) {
                f.alive = false;
                seenFaces.erase(tk);
                continue;
            }
            ++edgeUse[e0];
            ++edgeUse[e1];
            ++edgeUse[e2];
        }
    }
};

struct Topology {
    std::vector<std::vector<int>> incidentFaces;
    std::vector<std::unordered_set<int>> neighbors;
    std::unordered_map<std::uint64_t, std::vector<int>> edgeFaces;
    std::unordered_map<TriKey, std::vector<int>, TriKeyHash> faceOwners;
    std::vector<unsigned char> boundaryVertex;
};

std::string nextToken(std::istream& in) {
    std::string tok;
    while (in >> tok) {
        if (!tok.empty() && tok[0] == '#') {
            std::string rest;
            std::getline(in, rest);
            continue;
        }
        return tok;
    }
    return {};
}

bool nextDataLine(std::istream& in, std::string& line) {
    while (std::getline(in, line)) {
        const std::size_t comment = line.find('#');
        if (comment != std::string::npos) line.erase(comment);
        const std::size_t first = line.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) continue;
        const std::size_t last = line.find_last_not_of(" \t\r\n");
        line = line.substr(first, last - first + 1);
        return true;
    }
    return false;
}

Mesh readOff(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open input: " + path);

    std::string line;
    if (!nextDataLine(in, line) || line != "OFF") throw std::runtime_error("input is not OFF: " + path);
    if (!nextDataLine(in, line)) throw std::runtime_error("missing OFF counts: " + path);
    std::istringstream counts(line);
    int nv = 0;
    int nf = 0;
    int ne = 0;
    counts >> nv >> nf >> ne;
    if (!counts || nv < 0 || nf < 0) throw std::runtime_error("invalid OFF counts: " + path);

    Mesh mesh;
    mesh.vertices.resize(nv);
    for (int i = 0; i < nv; ++i) {
        if (!nextDataLine(in, line)) throw std::runtime_error("truncated OFF vertices: " + path);
        std::istringstream row(line);
        row >> mesh.vertices[i].p.x >> mesh.vertices[i].p.y >> mesh.vertices[i].p.z;
        if (!row) throw std::runtime_error("invalid OFF vertex row: " + path);
    }

    mesh.faces.reserve(static_cast<std::size_t>(nf) * 2);
    for (int i = 0; i < nf; ++i) {
        if (!nextDataLine(in, line)) throw std::runtime_error("truncated OFF faces: " + path);
        std::istringstream row(line);
        int deg = 0;
        row >> deg;
        std::vector<int> ids(static_cast<std::size_t>(deg));
        for (int j = 0; j < deg; ++j) row >> ids[j];
        if (!row) throw std::runtime_error("invalid OFF face row: " + path);
        if (deg < 3) continue;
        for (int j = 1; j + 1 < deg; ++j) {
            if (ids[0] == ids[j] || ids[j] == ids[j + 1] || ids[j + 1] == ids[0]) continue;
            mesh.faces.push_back({{ids[0], ids[j], ids[j + 1]}, true});
        }
    }
    mesh.cleanForDisplay();
    return mesh;
}

Topology buildTopology(const Mesh& mesh) {
    Topology topo;
    topo.incidentFaces.assign(mesh.vertices.size(), {});
    topo.neighbors.assign(mesh.vertices.size(), {});
    topo.boundaryVertex.assign(mesh.vertices.size(), 0);

    for (int fi = 0; fi < static_cast<int>(mesh.faces.size()); ++fi) {
        const Face& f = mesh.faces[fi];
        if (!f.alive) continue;
        for (int k = 0; k < 3; ++k) {
            topo.incidentFaces[f.v[k]].push_back(fi);
            const int a = f.v[k];
            const int b = f.v[(k + 1) % 3];
            topo.neighbors[a].insert(b);
            topo.neighbors[b].insert(a);
            topo.edgeFaces[edgeKey(a, b)].push_back(fi);
        }
        topo.faceOwners[canonicalTri(f.v[0], f.v[1], f.v[2])].push_back(fi);
    }

    for (const auto& kv : topo.edgeFaces) {
        if (kv.second.size() != 2) {
            const int a = edgeA(kv.first);
            const int b = edgeB(kv.first);
            if (a >= 0 && a < static_cast<int>(topo.boundaryVertex.size())) topo.boundaryVertex[a] = 1;
            if (b >= 0 && b < static_cast<int>(topo.boundaryVertex.size())) topo.boundaryVertex[b] = 1;
        }
    }
    return topo;
}

Vec3 newellNormal(const Mesh& mesh, const std::vector<int>& ring) {
    Vec3 n{};
    if (ring.size() < 3) return n;
    for (std::size_t i = 0; i < ring.size(); ++i) {
        const Vec3& a = mesh.vertices[ring[i]].p;
        const Vec3& b = mesh.vertices[ring[(i + 1) % ring.size()]].p;
        n.x += (a.y - b.y) * (a.z + b.z);
        n.y += (a.z - b.z) * (a.x + b.x);
        n.z += (a.x - b.x) * (a.y + b.y);
    }
    return normalized(n);
}

Vec3 averageFaceNormal(const Mesh& mesh, const std::vector<int>& faces) {
    Vec3 n{};
    for (int fid : faces) {
        if (fid < 0 || fid >= static_cast<int>(mesh.faces.size())) continue;
        const Face& f = mesh.faces[fid];
        if (!f.alive) continue;
        const Vec3 raw = cross(mesh.vertices[f.v[1]].p - mesh.vertices[f.v[0]].p,
                               mesh.vertices[f.v[2]].p - mesh.vertices[f.v[0]].p);
        n = n + raw;
    }
    return normalized(n);
}

bool projectedSimpleConvex(const Mesh& mesh,
                           const std::vector<int>& ring,
                           const Vec3& normal,
                           bool requireConvex) {
    const int n = static_cast<int>(ring.size());
    if (n < 3) return false;
    std::vector<Vec2> p(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) p[i] = projectPoint(mesh.vertices[ring[i]].p, normal);

    for (int i = 0; i < n; ++i) {
        const int i2 = (i + 1) % n;
        for (int j = i + 1; j < n; ++j) {
            const int j2 = (j + 1) % n;
            if (i == j || i2 == j || j2 == i) continue;
            if (segmentsIntersect2(p[i], p[i2], p[j], p[j2])) return false;
        }
    }

    if (!requireConvex || n == 3) return true;
    double sign = 0.0;
    for (int i = 0; i < n; ++i) {
        const double c = cross2(p[i], p[(i + 1) % n], p[(i + 2) % n]);
        if (std::fabs(c) <= 1e-12) return false;
        if (sign == 0.0) sign = c;
        else if (sign * c < 0.0) return false;
    }
    return true;
}

bool collectTriangleRing(const Mesh& mesh,
                         const Topology& topo,
                         int center,
                         std::vector<int>& ring,
                         std::vector<int>& affected) {
    ring.clear();
    affected.clear();
    if (center < 0 || center >= static_cast<int>(mesh.vertices.size())) return false;
    if (!mesh.vertices[center].alive || topo.boundaryVertex[center]) return false;

    std::unordered_map<int, int> nextOf;
    std::unordered_set<int> verticesSeen;
    for (int fid : topo.incidentFaces[center]) {
        const Face& f = mesh.faces[fid];
        if (!f.alive) continue;
        int ci = -1;
        for (int k = 0; k < 3; ++k) {
            if (f.v[k] == center) {
                ci = k;
                break;
            }
        }
        if (ci < 0) return false;
        const int next = f.v[(ci + 1) % 3];
        const int prev = f.v[(ci + 2) % 3];
        if (next == prev || next == center || prev == center) return false;
        if (!nextOf.emplace(next, prev).second) return false;
        verticesSeen.insert(next);
        verticesSeen.insert(prev);
        affected.push_back(fid);
    }
    if (nextOf.size() < 3 || nextOf.size() != verticesSeen.size()) return false;

    int start = nextOf.begin()->first;
    int cur = start;
    for (std::size_t guard = 0; guard <= nextOf.size(); ++guard) {
        ring.push_back(cur);
        auto it = nextOf.find(cur);
        if (it == nextOf.end()) return false;
        cur = it->second;
        if (cur == start) break;
    }
    if (cur != start || ring.size() != nextOf.size()) return false;
    std::unordered_set<int> unique(ring.begin(), ring.end());
    return unique.size() == ring.size();
}

std::vector<std::vector<std::array<int, 3>>> triangulateInterval(const std::vector<int>& ring,
                                                                 int left,
                                                                 int right) {
    if (right <= left + 1) return {{}};
    if (right == left + 2) return {{{ring[left], ring[left + 1], ring[right]}}};

    std::vector<std::vector<std::array<int, 3>>> out;
    for (int mid = left + 1; mid < right; ++mid) {
        auto lhs = triangulateInterval(ring, left, mid);
        auto rhs = triangulateInterval(ring, mid, right);
        for (const auto& lt : lhs) {
            for (const auto& rt : rhs) {
                std::vector<std::array<int, 3>> combined;
                combined.reserve(1 + lt.size() + rt.size());
                combined.push_back({ring[left], ring[mid], ring[right]});
                combined.insert(combined.end(), lt.begin(), lt.end());
                combined.insert(combined.end(), rt.begin(), rt.end());
                out.push_back(std::move(combined));
            }
        }
    }
    return out;
}

std::vector<std::vector<std::array<int, 3>>> enumerateTriangulations(const std::vector<int>& ring) {
    if (ring.size() < 3) return {};
    return triangulateInterval(ring, 0, static_cast<int>(ring.size()) - 1);
}

void orientTrianglesToNormal(const Mesh& mesh,
                             std::vector<std::array<int, 3>>& triangles,
                             const Vec3& patchNormal) {
    for (auto& t : triangles) {
        const Vec3 raw = cross(mesh.vertices[t[1]].p - mesh.vertices[t[0]].p,
                               mesh.vertices[t[2]].p - mesh.vertices[t[0]].p);
        if (dot(raw, patchNormal) < 0.0) std::swap(t[1], t[2]);
    }
}

bool validatePatchTriangles(const Mesh& mesh,
                            const Topology& topo,
                            const std::vector<int>& affected,
                            const std::vector<std::array<int, 3>>& triangles,
                            const Vec3& patchNormal,
                            double minNormalDot) {
    if (triangles.empty() || norm2(patchNormal) <= 0.0) return false;
    std::unordered_set<int> affectedSet(affected.begin(), affected.end());
    std::unordered_set<TriKey, TriKeyHash> newFaces;
    std::unordered_map<std::uint64_t, int> newEdgeUse;

    auto existingEdgeUseOutside = [&](std::uint64_t e) {
        int count = 0;
        auto it = topo.edgeFaces.find(e);
        if (it == topo.edgeFaces.end()) return 0;
        for (int fid : it->second) {
            if (affectedSet.find(fid) == affectedSet.end() && mesh.faces[fid].alive) ++count;
        }
        return count;
    };

    for (const auto& t : triangles) {
        if (t[0] == t[1] || t[1] == t[2] || t[2] == t[0]) return false;
        if (!mesh.vertices[t[0]].alive || !mesh.vertices[t[1]].alive || !mesh.vertices[t[2]].alive) {
            return false;
        }
        const Vec3 p0 = mesh.vertices[t[0]].p;
        const Vec3 p1 = mesh.vertices[t[1]].p;
        const Vec3 p2 = mesh.vertices[t[2]].p;
        const Vec3 raw = cross(p1 - p0, p2 - p0);
        if (norm2(raw) <= mesh.minArea2) return false;
        const Vec3 n = normalized(raw);
        if (dot(n, patchNormal) <= minNormalDot) return false;

        const TriKey tk = canonicalTri(t[0], t[1], t[2]);
        if (!newFaces.insert(tk).second) return false;
        auto ownerIt = topo.faceOwners.find(tk);
        if (ownerIt != topo.faceOwners.end()) {
            for (int owner : ownerIt->second) {
                if (affectedSet.find(owner) == affectedSet.end() && mesh.faces[owner].alive) return false;
            }
        }
        ++newEdgeUse[edgeKey(t[0], t[1])];
        ++newEdgeUse[edgeKey(t[1], t[2])];
        ++newEdgeUse[edgeKey(t[2], t[0])];
    }

    for (const auto& kv : newEdgeUse) {
        if (existingEdgeUseOutside(kv.first) + kv.second > 2) return false;
    }
    return true;
}

double triangleQualityScore(const Mesh& mesh, const std::array<int, 3>& t) {
    const Vec3 a = mesh.vertices[t[0]].p;
    const Vec3 b = mesh.vertices[t[1]].p;
    const Vec3 c = mesh.vertices[t[2]].p;
    const double e0 = norm2(a - b);
    const double e1 = norm2(b - c);
    const double e2 = norm2(c - a);
    const double area2 = std::max(norm2(cross(b - a, c - a)), mesh.minArea2);
    return (e0 + e1 + e2) / std::sqrt(area2);
}

double triangulationQualityScore(const Mesh& mesh, const std::vector<std::array<int, 3>>& triangles) {
    double score = 0.0;
    for (const auto& t : triangles) score += triangleQualityScore(mesh, t);
    return score;
}

std::vector<std::array<int, 3>> fanTriangulation(const std::vector<int>& ring, int root) {
    std::vector<std::array<int, 3>> tris;
    const int n = static_cast<int>(ring.size());
    if (n < 3) return tris;
    tris.reserve(static_cast<std::size_t>(n - 2));
    for (int off = 1; off + 1 < n; ++off) {
        tris.push_back({ring[static_cast<std::size_t>(root)],
                        ring[static_cast<std::size_t>((root + off) % n)],
                        ring[static_cast<std::size_t>((root + off + 1) % n)]});
    }
    return tris;
}

std::vector<std::vector<std::array<int, 3>>> fanTriangulations(const std::vector<int>& ring) {
    std::vector<std::vector<std::array<int, 3>>> out;
    out.reserve(ring.size());
    for (int root = 0; root < static_cast<int>(ring.size()); ++root) {
        out.push_back(fanTriangulation(ring, root));
    }
    return out;
}

std::vector<std::vector<std::array<int, 3>>> pcatmZTriangulations(const std::vector<int>& ring) {
    std::vector<std::vector<std::array<int, 3>>> out;
    if (ring.size() != 6) return out;
    out.reserve(6);
    for (int s = 0; s < 6; ++s) {
        auto r = [&](int off) {
            return ring[static_cast<std::size_t>((s + off) % 6)];
        };
        out.push_back({{std::array<int, 3>{r(0), r(1), r(2)},
                        std::array<int, 3>{r(0), r(2), r(5)},
                        std::array<int, 3>{r(2), r(3), r(4)},
                        std::array<int, 3>{r(2), r(4), r(5)}}});
    }
    return out;
}

std::vector<std::vector<std::array<int, 3>>> pcatmPentagonTriangulations(const std::vector<int>& ring) {
    std::vector<std::vector<std::array<int, 3>>> out;
    if (ring.size() != 5) return out;
    out.reserve(5);
    for (int s = 0; s < 5; ++s) {
        auto r = [&](int off) {
            return ring[static_cast<std::size_t>((s + off) % 5)];
        };
        out.push_back({{std::array<int, 3>{r(0), r(1), r(2)},
                        std::array<int, 3>{r(0), r(2), r(4)},
                        std::array<int, 3>{r(2), r(3), r(4)}}});
    }
    return out;
}

std::vector<std::vector<std::array<int, 3>>> pcatmAlternatingTriangulations(const std::vector<int>& ring) {
    std::vector<std::vector<std::array<int, 3>>> out;
    const int n = static_cast<int>(ring.size());
    if (n < 7) return out;
    out.reserve(ring.size());
    for (int s = 0; s < n; ++s) {
        std::vector<int> r;
        r.reserve(ring.size());
        for (int i = 0; i < n; ++i) r.push_back(ring[static_cast<std::size_t>((s + i) % n)]);

        int left = 0;
        int right = n - 1;
        bool takeLeftEar = true;
        std::vector<std::array<int, 3>> tris;
        tris.reserve(static_cast<std::size_t>(n - 2));
        while (right - left >= 2) {
            if (takeLeftEar) {
                tris.push_back({r[left], r[left + 1], r[right]});
                ++left;
            } else {
                tris.push_back({r[left], r[right - 1], r[right]});
                --right;
            }
            takeLeftEar = !takeLeftEar;
        }
        out.push_back(std::move(tris));
    }
    return out;
}

std::vector<std::vector<std::array<int, 3>>> patchTriangulationsForAlgo(const std::string& algo,
                                                                         const std::vector<int>& ring) {
    if (algo == "default") return {fanTriangulation(ring, 0)};
    if (algo == "pcatm" && ring.size() == 5) return pcatmPentagonTriangulations(ring);
    if (algo == "pcatm" && ring.size() == 6) return pcatmZTriangulations(ring);
    if (algo == "pcatm" && ring.size() >= 7) return pcatmAlternatingTriangulations(ring);
    if (ring.size() <= 8) return enumerateTriangulations(ring);
    return fanTriangulations(ring);
}

Vec3 barycenter(const Mesh& mesh, const std::vector<int>& ids) {
    Vec3 p{};
    for (int id : ids) p = p + mesh.vertices[id].p;
    return ids.empty() ? p : p / static_cast<double>(ids.size());
}

Vec3 laplacianAt(const Mesh& mesh, const Topology& topo, int v) {
    Vec3 avg{};
    int count = 0;
    for (int n : topo.neighbors[v]) {
        if (!mesh.vertices[n].alive) continue;
        avg = avg + mesh.vertices[n].p;
        ++count;
    }
    if (count == 0) return {};
    return avg / static_cast<double>(count) - mesh.vertices[v].p;
}

Vec3 averageBoundaryLaplacian(const Mesh& mesh, const Topology& topo, const std::vector<int>& ring) {
    Vec3 l{};
    for (int v : ring) l = l + laplacianAt(mesh, topo, v);
    return ring.empty() ? l : l / static_cast<double>(ring.size());
}

int aliveNeighborCount(const Mesh& mesh, const Topology& topo, int v) {
    if (v < 0 || v >= static_cast<int>(topo.neighbors.size())) return 0;
    int count = 0;
    for (int n : topo.neighbors[v]) {
        if (n >= 0 && n < static_cast<int>(mesh.vertices.size()) && mesh.vertices[n].alive) ++count;
    }
    return count;
}

double vertexNormalSpread(const Mesh& mesh, const Topology& topo, int v) {
    if (v < 0 || v >= static_cast<int>(topo.incidentFaces.size())) return 0.0;
    Vec3 avg{};
    int count = 0;
    for (int fid : topo.incidentFaces[v]) {
        if (fid < 0 || fid >= static_cast<int>(mesh.faces.size()) || !mesh.faces[fid].alive) continue;
        const Vec3 n = mesh.faceNormal(mesh.faces[fid]);
        if (norm2(n) <= 0.0) continue;
        avg = avg + n;
        ++count;
    }
    if (count <= 1) return 0.0;
    avg = normalized(avg);
    double spread = 0.0;
    for (int fid : topo.incidentFaces[v]) {
        if (fid < 0 || fid >= static_cast<int>(mesh.faces.size()) || !mesh.faces[fid].alive) continue;
        const Vec3 n = mesh.faceNormal(mesh.faces[fid]);
        if (norm2(n) <= 0.0) continue;
        spread = std::max(spread, 1.0 - dot(avg, n));
    }
    return spread;
}

std::vector<double> computeAnchorScores(const Mesh& mesh, const Topology& topo) {
    std::vector<double> raw(mesh.vertices.size(), 0.0);
    double sum = 0.0;
    double sum2 = 0.0;
    int count = 0;
    const double diag = std::sqrt(std::max(mesh.diag2, 1e-24));
    for (int v = 0; v < static_cast<int>(mesh.vertices.size()); ++v) {
        if (!mesh.vertices[v].alive || topo.boundaryVertex[v]) continue;
        const double lap = std::sqrt(std::max(0.0, norm2(laplacianAt(mesh, topo, v)))) / diag;
        raw[v] = lap + vertexNormalSpread(mesh, topo, v);
        sum += raw[v];
        sum2 += raw[v] * raw[v];
        ++count;
    }
    if (count <= 1) return raw;
    const double mean = sum / static_cast<double>(count);
    const double variance = std::max(1e-18, sum2 / static_cast<double>(count) - mean * mean);
    const double invStd = 1.0 / std::sqrt(variance);
    for (int v = 0; v < static_cast<int>(mesh.vertices.size()); ++v) {
        if (!mesh.vertices[v].alive || topo.boundaryVertex[v]) {
            raw[v] = 0.0;
        } else {
            raw[v] = (raw[v] - mean) * invStd;
        }
    }
    return raw;
}

bool isDecimatingPass(int round) {
    return ((std::max(1, round) - 1) % 2) == 0;
}

bool paperValenceAllowed(const std::string& algo, int valence, int round) {
    if (valence < 3) return false;
    const bool decimating = isDecimatingPass(round);
    if (algo == "rd") {
        return decimating ? (valence >= 4 && valence <= 6) : (valence == 3);
    }
    if (algo == "aptm") {
        return decimating ? (valence <= 6) : (valence == 3);
    }
    if (algo == "pclttm") {
        return decimating ? (valence > 3 && valence <= 6) : (valence == 3);
    }
    if (algo == "mmse") return valence <= 8;
    if (algo == "pcatm") return valence <= 10;
    return valence <= 10;
}

bool ringAdjacent(const std::unordered_map<int, int>& ringPos, int a, int b, int ringSize) {
    auto ia = ringPos.find(a);
    auto ib = ringPos.find(b);
    if (ia == ringPos.end() || ib == ringPos.end()) return false;
    const int d = std::abs(ia->second - ib->second);
    return d == 1 || d == ringSize - 1;
}

std::unordered_map<int, int> estimateBoundaryValences(const Mesh& mesh,
                                                       const Topology& topo,
                                                       const std::vector<int>& ring,
                                                       const std::vector<std::array<int, 3>>& triangles) {
    std::unordered_map<int, int> ringPos;
    for (int i = 0; i < static_cast<int>(ring.size()); ++i) ringPos[ring[i]] = i;

    std::unordered_map<int, int> estimated;
    for (int v : ring) estimated[v] = std::max(0, aliveNeighborCount(mesh, topo, v) - 1);

    std::unordered_set<std::uint64_t> addedDiagonals;
    for (const auto& t : triangles) {
        for (int i = 0; i < 3; ++i) {
            const int a = t[i];
            const int b = t[(i + 1) % 3];
            if (ringPos.find(a) == ringPos.end() || ringPos.find(b) == ringPos.end()) continue;
            if (ringAdjacent(ringPos, a, b, static_cast<int>(ring.size()))) continue;
            if (!addedDiagonals.insert(edgeKey(a, b)).second) continue;
            ++estimated[a];
            ++estimated[b];
        }
    }
    return estimated;
}

double boundaryValenceRegularityScore(const Mesh& mesh,
                                      const Topology& topo,
                                      const std::vector<int>& ring,
                                      const std::vector<std::array<int, 3>>& triangles) {
    const auto estimated = estimateBoundaryValences(mesh, topo, ring, triangles);
    double score = 0.0;
    for (int v : ring) {
        auto it = estimated.find(v);
        if (it == estimated.end()) continue;
        score += std::abs(it->second - 6);
    }
    return score;
}

int estimatedValenceThreeCount(const Mesh& mesh,
                               const Topology& topo,
                               const std::vector<int>& ring,
                               const std::vector<std::array<int, 3>>& triangles) {
    const auto estimated = estimateBoundaryValences(mesh, topo, ring, triangles);
    int count = 0;
    for (const auto& kv : estimated) {
        if (kv.second == 3) ++count;
    }
    return count;
}

double patchTriangulationScore(const Mesh& mesh,
                               const Topology& topo,
                               const std::string& algo,
                               const std::vector<int>& ring,
                               const std::vector<std::array<int, 3>>& triangles) {
    if (algo == "default") return static_cast<double>(ring.size());
    double score = triangulationQualityScore(mesh, triangles);
    if (algo == "rd") {
        score += 0.75 * boundaryValenceRegularityScore(mesh, topo, ring, triangles);
        score -= 0.50 * static_cast<double>(estimatedValenceThreeCount(mesh, topo, ring, triangles));
    } else if (algo == "aptm") {
        score += 0.50 * boundaryValenceRegularityScore(mesh, topo, ring, triangles);
    } else if (algo == "pcatm") {
        score += 0.25 * boundaryValenceRegularityScore(mesh, topo, ring, triangles);
    } else if (algo == "pclttm") {
        score += 1.25 * boundaryValenceRegularityScore(mesh, topo, ring, triangles);
    }
    return score;
}

Vec3 dualRingPrediction(const Mesh& mesh, const Topology& topo, int center, double alpha) {
    std::vector<int> ring;
    if (center >= 0 && center < static_cast<int>(topo.neighbors.size())) {
        for (int v : topo.neighbors[center]) {
            if (v >= 0 && v < static_cast<int>(mesh.vertices.size()) && mesh.vertices[v].alive) ring.push_back(v);
        }
    }
    const Vec3 primary = barycenter(mesh, ring);
    if (ring.empty() || alpha <= 0.0) return primary;

    Vec3 numerator = primary;
    double denominator = 1.0;
    for (int l : ring) {
        const int vl = std::max(1, aliveNeighborCount(mesh, topo, l));
        Vec3 second{};
        for (int k : topo.neighbors[l]) {
            if (k == center || k < 0 || k >= static_cast<int>(mesh.vertices.size()) || !mesh.vertices[k].alive) continue;
            second = second + mesh.vertices[k].p;
        }
        const Vec3 term = mesh.vertices[l].p * static_cast<double>(vl) - second;
        const double w = 1.0 / static_cast<double>(vl * vl);
        numerator = numerator + term * (alpha * w);
        denominator += alpha * w;
    }
    return numerator / denominator;
}

Vec3 mmsePrediction(const Mesh& mesh, const Topology& topo, int center) {
    if (center < 0 || center >= static_cast<int>(topo.neighbors.size())) return {};
    std::unordered_map<int, double> weights;
    constexpr double rho = 0.85;
    for (int v : topo.neighbors[center]) {
        if (v < 0 || v >= static_cast<int>(mesh.vertices.size()) || !mesh.vertices[v].alive) continue;
        weights[v] = std::max(weights[v], rho);
        for (int n : topo.neighbors[v]) {
            if (n == center || n < 0 || n >= static_cast<int>(mesh.vertices.size()) || !mesh.vertices[n].alive) continue;
            weights[n] = std::max(weights[n], rho * rho);
        }
    }
    Vec3 pred{};
    double total = 0.0;
    for (const auto& kv : weights) {
        pred = pred + mesh.vertices[kv.first].p * kv.second;
        total += kv.second;
    }
    if (total <= 0.0) return dualRingPrediction(mesh, topo, center, 0.0);
    return pred / total;
}

void addPlaneQuadric(std::array<double, 10>& q, const Vec3& n, double d) {
    q[0] += n.x * n.x;
    q[1] += n.x * n.y;
    q[2] += n.x * n.z;
    q[3] += n.x * d;
    q[4] += n.y * n.y;
    q[5] += n.y * n.z;
    q[6] += n.y * d;
    q[7] += n.z * n.z;
    q[8] += n.z * d;
    q[9] += d * d;
}

std::array<double, 10> addQuadric(const std::array<double, 10>& a,
                                  const std::array<double, 10>& b) {
    std::array<double, 10> q{};
    for (int i = 0; i < 10; ++i) q[i] = a[i] + b[i];
    return q;
}

double quadricError(const std::array<double, 10>& q, const Vec3& p) {
    return q[0] * p.x * p.x + 2.0 * q[1] * p.x * p.y +
           2.0 * q[2] * p.x * p.z + 2.0 * q[3] * p.x +
           q[4] * p.y * p.y + 2.0 * q[5] * p.y * p.z +
           2.0 * q[6] * p.y + q[7] * p.z * p.z +
           2.0 * q[8] * p.z + q[9];
}

double det3(double a00, double a01, double a02,
            double a10, double a11, double a12,
            double a20, double a21, double a22) {
    return a00 * (a11 * a22 - a12 * a21) -
           a01 * (a10 * a22 - a12 * a20) +
           a02 * (a10 * a21 - a11 * a20);
}

Vec3 qemOptimalPoint(const Mesh& mesh, int a, int b) {
    const auto q = addQuadric(mesh.vertices[a].q, mesh.vertices[b].q);
    const double det = det3(q[0], q[1], q[2],
                            q[1], q[4], q[5],
                            q[2], q[5], q[7]);
    if (std::fabs(det) > 1e-10) {
        return {
            -det3(q[3], q[1], q[2], q[6], q[4], q[5], q[8], q[5], q[7]) / det,
            -det3(q[0], q[3], q[2], q[1], q[6], q[5], q[2], q[8], q[7]) / det,
            -det3(q[0], q[1], q[3], q[1], q[4], q[6], q[2], q[5], q[8]) / det,
        };
    }

    const Vec3 p0 = mesh.vertices[a].p;
    const Vec3 p1 = mesh.vertices[b].p;
    const Vec3 pm = (p0 + p1) * 0.5;
    Vec3 best = pm;
    double bestErr = quadricError(q, pm);
    const double e0 = quadricError(q, p0);
    if (e0 < bestErr) {
        bestErr = e0;
        best = p0;
    }
    const double e1 = quadricError(q, p1);
    if (e1 < bestErr) {
        best = p1;
    }
    return best;
}

void computeQuadrics(Mesh& mesh) {
    for (auto& v : mesh.vertices) v.q.fill(0.0);
    for (const Face& f : mesh.faces) {
        if (!f.alive) continue;
        const Vec3 a = mesh.vertices[f.v[0]].p;
        const Vec3 b = mesh.vertices[f.v[1]].p;
        const Vec3 c = mesh.vertices[f.v[2]].p;
        const Vec3 n = normalized(cross(b - a, c - a));
        if (norm2(n) <= 0.0) continue;
        const double d = -dot(n, a);
        for (int v : f.v) addPlaneQuadric(mesh.vertices[v].q, n, d);
    }
}

bool hasQuadrics(const Mesh& mesh) {
    for (const auto& v : mesh.vertices) {
        for (double x : v.q) {
            if (std::fabs(x) > 0.0) return true;
        }
    }
    return false;
}

struct Args {
    std::string algo = "qem";
    std::string input;
    std::string outputDir;
    int rounds = 30;
    int minFaces = 600;
    double ratio = 0.88;
    double normalDot = -1.0;
    bool ppmcStrictTriangulate = false;
};

bool startsWith(const std::string& s, const char* p) {
    const std::string prefix(p);
    return s.size() >= prefix.size() && s.compare(0, prefix.size(), prefix) == 0;
}

Args parseArgs(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        std::string a(argv[i]);
        if (startsWith(a, "--algo=")) args.algo = a.substr(7);
        else if (startsWith(a, "--input=")) args.input = a.substr(8);
        else if (startsWith(a, "--output-dir=")) args.outputDir = a.substr(13);
        else if (startsWith(a, "--rounds=")) args.rounds = std::stoi(a.substr(9));
        else if (startsWith(a, "--ratio=")) args.ratio = std::stod(a.substr(8));
        else if (startsWith(a, "--min-faces=")) args.minFaces = std::stoi(a.substr(12));
        else if (startsWith(a, "--normal-dot=")) args.normalDot = std::stod(a.substr(13));
        else if (a == "--ppmc-strict-triangulate") args.ppmcStrictTriangulate = true;
        else throw std::runtime_error("unknown arg: " + a);
    }
    if (args.input.empty()) throw std::runtime_error("--input is required");
    if (args.outputDir.empty()) throw std::runtime_error("--output-dir is required");
    if (args.algo != "default" && args.algo != "qem" && args.algo != "rd" &&
        args.algo != "ppmc" && args.algo != "pomar" && args.algo != "mmse" &&
        args.algo != "aptm" && args.algo != "pcatm" && args.algo != "pclttm" &&
        args.algo != "vqmmc") {
        throw std::runtime_error("--algo must be one of: default, qem, rd, ppmc, pomar, mmse, aptm, pcatm, pclttm, vqmmc");
    }
    if (args.rounds < 0) args.rounds = 0;
    args.ratio = std::clamp(args.ratio, 0.50, 0.98);
    return args;
}

double defaultNormalDot(const std::string& algo) {
    if (algo == "qem") return 0.25;     // impl/OpenMesh/QEM inversion guard.
    if (algo == "rd") return 0.60;
    if (algo == "ppmc") return 0.70;
    if (algo == "pomar") return 0.70;   // POMAR's original normal guard is > 0.7.
    if (algo == "mmse") return 0.65;
    if (algo == "aptm") return 0.70;
    if (algo == "pcatm") return 0.65;
    if (algo == "pclttm") return 0.70;
    if (algo == "vqmmc") return 0.55;
    return 0.70;
}

bool collapseAllowed(const Mesh& mesh,
                     const Topology& topo,
                     int from,
                     int to,
                     const Vec3& target,
                     double minNormalDot) {
    if (from == to || from < 0 || to < 0 ||
        from >= static_cast<int>(mesh.vertices.size()) ||
        to >= static_cast<int>(mesh.vertices.size())) {
        return false;
    }
    if (!mesh.vertices[from].alive || !mesh.vertices[to].alive) return false;
    if (topo.boundaryVertex[from] || topo.boundaryVertex[to]) return false;

    const auto edgeIt = topo.edgeFaces.find(edgeKey(from, to));
    if (edgeIt == topo.edgeFaces.end() || edgeIt->second.size() != 2) return false;

    const Face& fa = mesh.faces[edgeIt->second[0]];
    const Face& fb = mesh.faces[edgeIt->second[1]];
    const int tipA = thirdVertex(fa, from, to);
    const int tipB = thirdVertex(fb, from, to);
    if (tipA < 0 || tipB < 0 || tipA == tipB) return false;

    int common = 0;
    for (int n : topo.neighbors[from]) {
        if (n == to) continue;
        if (topo.neighbors[to].find(n) == topo.neighbors[to].end()) continue;
        if (n != tipA && n != tipB) return false;
        ++common;
    }
    if (common != 2) return false;

    const Vec3 midpoint = (mesh.vertices[from].p + mesh.vertices[to].p) * 0.5;
    const double edgeLen2 = std::max(norm2(mesh.vertices[from].p - mesh.vertices[to].p), mesh.minEdge2);
    const double maxTargetDrift2 = std::max(edgeLen2 * 4.0, mesh.diag2 * 1e-4);
    if (norm2(target - midpoint) > maxTargetDrift2) return false;

    std::unordered_set<int> affected;
    for (int fid : topo.incidentFaces[from]) affected.insert(fid);
    for (int fid : topo.incidentFaces[to]) affected.insert(fid);
    if (affected.empty()) return false;

    std::unordered_set<TriKey, TriKeyHash> newFaces;
    std::unordered_map<std::uint64_t, int> newEdgeUse;

    auto existingEdgeUseOutside = [&](std::uint64_t e) {
        int count = 0;
        auto it = topo.edgeFaces.find(e);
        if (it == topo.edgeFaces.end()) return 0;
        for (int fid : it->second) {
            if (affected.find(fid) == affected.end() && mesh.faces[fid].alive) ++count;
        }
        return count;
    };

    for (int fid : affected) {
        const Face& oldFace = mesh.faces[fid];
        if (!oldFace.alive) continue;
        if (faceContainsBoth(oldFace, from, to)) continue;

        Face nf = oldFace;
        for (int& v : nf.v) {
            if (v == from) v = to;
        }
        if (nf.v[0] == nf.v[1] || nf.v[1] == nf.v[2] || nf.v[2] == nf.v[0]) return false;

        const Vec3 oldN = mesh.faceNormal(oldFace);
        const Vec3 p0 = mesh.pointFor(nf.v[0], to, target);
        const Vec3 p1 = mesh.pointFor(nf.v[1], to, target);
        const Vec3 p2 = mesh.pointFor(nf.v[2], to, target);
        const double e0 = norm2(p0 - p1);
        const double e1 = norm2(p1 - p2);
        const double e2 = norm2(p2 - p0);
        const Vec3 newRaw = cross(p1 - p0, p2 - p0);
        if (e0 <= mesh.minEdge2 || e1 <= mesh.minEdge2 || e2 <= mesh.minEdge2 ||
            norm2(newRaw) <= mesh.minArea2) {
            return false;
        }
        const Vec3 newN = normalized(newRaw);
        if (norm2(oldN) <= 0.0 || dot(oldN, newN) <= minNormalDot) return false;

        const TriKey tk = canonicalTri(nf.v[0], nf.v[1], nf.v[2]);
        if (!newFaces.insert(tk).second) return false;
        auto ownerIt = topo.faceOwners.find(tk);
        if (ownerIt != topo.faceOwners.end()) {
            for (int owner : ownerIt->second) {
                if (affected.find(owner) == affected.end() && mesh.faces[owner].alive) return false;
            }
        }

        ++newEdgeUse[edgeKey(nf.v[0], nf.v[1])];
        ++newEdgeUse[edgeKey(nf.v[1], nf.v[2])];
        ++newEdgeUse[edgeKey(nf.v[2], nf.v[0])];
    }

    if (newFaces.empty()) return false;
    for (const auto& kv : newEdgeUse) {
        if (existingEdgeUseOutside(kv.first) + kv.second > 2) return false;
    }
    return true;
}

double normalVariance(const Mesh& mesh, const Topology& topo, int a, int b) {
    Vec3 avg{};
    int count = 0;
    auto addIncident = [&](int v) {
        for (int fid : topo.incidentFaces[v]) {
            const Face& f = mesh.faces[fid];
            if (!f.alive || faceContainsBoth(f, a, b)) continue;
            const Vec3 n = mesh.faceNormal(f);
            if (norm2(n) <= 0.0) continue;
            avg = avg + n;
            ++count;
        }
    };
    addIncident(a);
    addIncident(b);
    if (count <= 1) return 0.0;
    avg = normalized(avg);
    double spread = 0.0;
    for (int fid : topo.incidentFaces[a]) {
        const Face& f = mesh.faces[fid];
        if (!f.alive || faceContainsBoth(f, a, b)) continue;
        spread = std::max(spread, 1.0 - dot(avg, mesh.faceNormal(f)));
    }
    for (int fid : topo.incidentFaces[b]) {
        const Face& f = mesh.faces[fid];
        if (!f.alive || faceContainsBoth(f, a, b)) continue;
        spread = std::max(spread, 1.0 - dot(avg, mesh.faceNormal(f)));
    }
    return std::max(0.0, spread);
}

struct Candidate {
    int from = -1;
    int to = -1;
    Vec3 target;
    double cost = 0.0;
};

double candidateCost(const Mesh& mesh,
                     const Topology& topo,
                     const std::string& algo,
                     int from,
                     int to,
                     const Vec3& target) {
    if (algo == "pomar") {
        double farthestUnconnected = -1.0;
        const Vec3& toPoint = mesh.vertices[to].p;
        for (int nb : topo.neighbors[from]) {
            if (nb == to || !mesh.vertices[nb].alive) continue;
            if (topo.neighbors[to].find(nb) != topo.neighbors[to].end()) continue;
            farthestUnconnected = std::max(farthestUnconnected, norm2(toPoint - mesh.vertices[nb].p));
        }
        if (farthestUnconnected < 0.0) {
            return std::numeric_limits<double>::max();
        }
        return farthestUnconnected / mesh.diag2 +
               0.25 * normalVariance(mesh, topo, from, to) +
               1e-6 * norm2(mesh.vertices[from].p - mesh.vertices[to].p) / mesh.diag2;
    }
    if (algo == "vqmmc") {
        const Vec3 residual = mesh.vertices[from].p - target;
        const double residual2 = norm2(residual) / mesh.diag2;
        const double step = std::sqrt(std::max(mesh.diag2, 1e-24)) / 1024.0;
        const Vec3 quantized{std::round(residual.x / step) * step,
                             std::round(residual.y / step) * step,
                             std::round(residual.z / step) * step};
        const double quantError2 = norm2(residual - quantized) / mesh.diag2;
        return residual2 + 4.0 * quantError2 + 0.15 * normalVariance(mesh, topo, from, to);
    }
    const auto q = addQuadric(mesh.vertices[from].q, mesh.vertices[to].q);
    const double qerr = std::max(0.0, quadricError(q, target));
    const double len = norm2(mesh.vertices[from].p - mesh.vertices[to].p) / mesh.diag2;
    return qerr + 1e-6 * len;
}

std::vector<Candidate> buildCandidates(const Mesh& mesh,
                                       const Topology& topo,
                                       const std::string& algo,
                                       double minNormalDot) {
    std::vector<Candidate> out;
    out.reserve(topo.edgeFaces.size());
    for (const auto& kv : topo.edgeFaces) {
        if (kv.second.size() != 2) continue;
        const int a = edgeA(kv.first);
        const int b = edgeB(kv.first);
        if (!mesh.vertices[a].alive || !mesh.vertices[b].alive) continue;
        if (topo.boundaryVertex[a] || topo.boundaryVertex[b]) continue;

        if (algo == "pomar" || algo == "vqmmc") {
            for (const auto [remove, keep] : std::array<std::pair<int, int>, 2>{{{a, b}, {b, a}}}) {
                const Vec3 target = mesh.vertices[keep].p;
                if (!collapseAllowed(mesh, topo, remove, keep, target, minNormalDot)) continue;
                const double cost = candidateCost(mesh, topo, algo, remove, keep, target);
                if (std::isfinite(cost)) out.push_back({remove, keep, target, cost});
            }
            continue;
        }

        int keep = a;
        int remove = b;
        if (topo.incidentFaces[b].size() > topo.incidentFaces[a].size()) {
            keep = b;
            remove = a;
        }

        const Vec3 midpoint = (mesh.vertices[a].p + mesh.vertices[b].p) * 0.5;
        const Vec3 qem = qemOptimalPoint(mesh, a, b);
        std::vector<Vec3> targets{qem, midpoint, mesh.vertices[keep].p, mesh.vertices[remove].p};

        bool found = false;
        Candidate best;
        for (const Vec3& target : targets) {
            if (!collapseAllowed(mesh, topo, remove, keep, target, minNormalDot)) continue;
            const double cost = candidateCost(mesh, topo, algo, remove, keep, target);
            if (!found || cost < best.cost) {
                best = {remove, keep, target, cost};
                found = true;
            }
        }
        if (found) out.push_back(best);
    }
    std::sort(out.begin(), out.end(), [](const Candidate& a, const Candidate& b) {
        return a.cost < b.cost;
    });
    return out;
}

bool applyCollapse(Mesh& mesh, const Candidate& c) {
    if (c.from < 0 || c.to < 0 || !mesh.vertices[c.from].alive || !mesh.vertices[c.to].alive) {
        return false;
    }
    mesh.vertices[c.to].q = addQuadric(mesh.vertices[c.to].q, mesh.vertices[c.from].q);
    mesh.vertices[c.to].p = c.target;
    for (Face& f : mesh.faces) {
        if (!f.alive) continue;
        const bool hasFrom = faceContains(f, c.from);
        const bool hasTo = faceContains(f, c.to);
        if (hasFrom && hasTo) {
            f.alive = false;
            continue;
        }
        if (hasFrom) {
            for (int& v : f.v) {
                if (v == c.from) v = c.to;
            }
            if (f.v[0] == f.v[1] || f.v[1] == f.v[2] || f.v[2] == f.v[0]) {
                f.alive = false;
            }
        }
    }
    mesh.vertices[c.from].alive = false;
    return true;
}

struct PatchCandidate {
    int center = -1;
    std::vector<int> ring;
    std::vector<int> affectedFaces;
    std::vector<std::array<int, 3>> triangles;
    double cost = 0.0;
    int valence = 0;
};

bool buildBestPatchCandidate(const Mesh& mesh,
                             const Topology& topo,
                             const std::string& algo,
                             int center,
                             int round,
                             const std::vector<double>& anchorScores,
                             double minNormalDot,
                             PatchCandidate& out) {
    std::vector<int> ring;
    std::vector<int> affected;
    if (!collectTriangleRing(mesh, topo, center, ring, affected)) return false;

    const int valence = static_cast<int>(ring.size());
    if (!paperValenceAllowed(algo, valence, round)) return false;
    if (algo == "aptm" && center < static_cast<int>(anchorScores.size()) && anchorScores[center] >= 1.7) {
        return false;
    }

    Vec3 patchNormal = averageFaceNormal(mesh, affected);
    if (norm2(patchNormal) <= 0.0) patchNormal = newellNormal(mesh, ring);
    if (norm2(patchNormal) <= 0.0) return false;
    if (!projectedSimpleConvex(mesh, ring, patchNormal, true)) return false;

    bool found = false;
    std::vector<std::array<int, 3>> bestTris;
    double bestScore = 0.0;

    auto triangulations = patchTriangulationsForAlgo(algo, ring);
    for (auto tris : triangulations) {
        orientTrianglesToNormal(mesh, tris, patchNormal);
        if (!validatePatchTriangles(mesh, topo, affected, tris, patchNormal, minNormalDot)) continue;
        const double score = patchTriangulationScore(mesh, topo, algo, ring, tris);
        if (!found || score < bestScore) {
            found = true;
            bestScore = score;
            bestTris = std::move(tris);
        }
    }
    if (!found) return false;

    double cost = static_cast<double>(center);
    if (algo == "rd") {
        const Vec3 pred = barycenter(mesh, ring);
        const Vec3 residual = mesh.vertices[center].p - pred;
        const Vec3 curvature = averageBoundaryLaplacian(mesh, topo, ring);
        const Vec3 curvedResidual = residual - curvature * 0.5;
        const int valencePriority = isDecimatingPass(round) ? (valence - 4) : 0;
        cost = static_cast<double>(valencePriority) * 1e6 +
               norm2(curvedResidual) / mesh.diag2 +
               1e-4 * bestScore;
    } else if (algo == "mmse") {
        const Vec3 centerPoint = mesh.vertices[center].p;
        const Vec3 pred0 = dualRingPrediction(mesh, topo, center, 0.0);
        const Vec3 pred1 = dualRingPrediction(mesh, topo, center, 0.5);
        const Vec3 pred2 = dualRingPrediction(mesh, topo, center, 1.0);
        const Vec3 pred3 = mmsePrediction(mesh, topo, center);
        const double e0 = norm2(centerPoint - pred0);
        const double e1 = norm2(centerPoint - pred1);
        const double e2 = norm2(centerPoint - pred2);
        const double e3 = norm2(centerPoint - pred3);
        cost = std::min({e0, e1, e2, e3}) / mesh.diag2 + 1e-4 * bestScore;
    } else if (algo == "aptm") {
        const double anchorScore = center < static_cast<int>(anchorScores.size())
                                       ? std::max(0.0, anchorScores[center])
                                       : 0.0;
        const int valencePriority = isDecimatingPass(round) ? std::abs(valence - 6) : 0;
        cost = anchorScore * 0.25 + static_cast<double>(valencePriority) * 0.05 + 1e-4 * bestScore;
    } else if (algo == "pcatm") {
        int valencePriority = 6;
        if (valence == 6) valencePriority = 0;
        else if (valence == 5) valencePriority = 1;
        else if (valence == 4) valencePriority = 2;
        else if (valence == 3) valencePriority = 3;
        else valencePriority = valence - 3;
        const Vec3 pred = barycenter(mesh, ring);
        cost = static_cast<double>(valencePriority) * 1e5 +
               norm2(mesh.vertices[center].p - pred) / mesh.diag2 +
               1e-4 * bestScore;
    } else if (algo == "pclttm") {
        int valencePriority = 3;
        if (isDecimatingPass(round)) {
            if (valence == 6) valencePriority = 0;
            else if (valence == 5) valencePriority = 1;
            else if (valence == 4) valencePriority = 2;
        } else {
            valencePriority = 0;
        }
        cost = static_cast<double>(valencePriority) * 1e6 +
               0.25 * normalVariance(mesh, topo, center, ring.front()) +
               1e-4 * bestScore;
    }

    out.center = center;
    out.ring = std::move(ring);
    out.affectedFaces = std::move(affected);
    out.triangles = std::move(bestTris);
    out.cost = cost;
    out.valence = valence;
    return true;
}

bool applyPatchRemoval(Mesh& mesh, const PatchCandidate& c) {
    if (c.center < 0 || c.center >= static_cast<int>(mesh.vertices.size())) return false;
    if (!mesh.vertices[c.center].alive) return false;
    for (int fid : c.affectedFaces) {
        if (fid >= 0 && fid < static_cast<int>(mesh.faces.size())) {
            mesh.faces[fid].alive = false;
        }
    }
    for (const auto& t : c.triangles) {
        mesh.faces.push_back({{t[0], t[1], t[2]}, true});
    }
    mesh.vertices[c.center].alive = false;
    return true;
}

int simplifyPatchRound(Mesh& mesh,
                       const std::string& algo,
                       int round,
                       double ratio,
                       int minFaces,
                       double minNormalDot) {
    mesh.cleanForDisplay();
    const Topology topo = buildTopology(mesh);
    const std::vector<double> anchorScores = algo == "aptm" ? computeAnchorScores(mesh, topo) : std::vector<double>{};
    const int currentFaces = mesh.aliveFaceCount();
    const int targetFaces = std::max(minFaces, static_cast<int>(std::floor(currentFaces * ratio)));
    const int faceBudget = std::max(0, currentFaces - targetFaces);
    const int maxRemovals = std::max(1, faceBudget / 2);
    if (currentFaces <= minFaces || faceBudget <= 0) return 0;

    std::vector<PatchCandidate> candidates;
    candidates.reserve(mesh.vertices.size());
    for (int v = 0; v < static_cast<int>(mesh.vertices.size()); ++v) {
        PatchCandidate c;
        if (buildBestPatchCandidate(mesh, topo, algo, v, round, anchorScores, minNormalDot, c)) {
            candidates.push_back(std::move(c));
        }
    }
    std::sort(candidates.begin(), candidates.end(), [](const PatchCandidate& a, const PatchCandidate& b) {
        if (a.cost != b.cost) return a.cost < b.cost;
        return a.center < b.center;
    });

    std::vector<unsigned char> blocked(mesh.vertices.size(), 0);
    std::vector<PatchCandidate> selected;
    selected.reserve(static_cast<std::size_t>(maxRemovals));
    for (const PatchCandidate& c : candidates) {
        if (static_cast<int>(selected.size()) >= maxRemovals) break;
        if (blocked[c.center]) continue;
        bool conflict = false;
        for (int v : c.ring) {
            if (blocked[v]) {
                conflict = true;
                break;
            }
        }
        if (conflict) continue;
        selected.push_back(c);
        blocked[c.center] = 1;
        for (int v : c.ring) {
            blocked[v] = 1;
            for (int n : topo.neighbors[v]) blocked[n] = 1;
        }
    }

    int applied = 0;
    for (const PatchCandidate& c : selected) {
        if (applyPatchRemoval(mesh, c)) ++applied;
    }
    mesh.cleanForDisplay();
    return applied;
}

int simplifyRound(Mesh& mesh,
                  const std::string& algo,
                  int round,
                  double ratio,
                  int minFaces,
                  double minNormalDot) {
    if (algo == "default" || algo == "rd" || algo == "mmse" || algo == "aptm" ||
        algo == "pcatm" || algo == "pclttm") {
        return simplifyPatchRound(mesh, algo, round, ratio, minFaces, minNormalDot);
    }

    mesh.cleanForDisplay();
    if (algo == "qem" && !hasQuadrics(mesh)) computeQuadrics(mesh);
    const Topology topo = buildTopology(mesh);
    const int currentFaces = mesh.aliveFaceCount();
    const int targetFaces = std::max(minFaces, static_cast<int>(std::floor(currentFaces * ratio)));
    const int faceBudget = std::max(0, currentFaces - targetFaces);
    const int maxCollapses = std::max(1, (faceBudget + 1) / 2);
    if (currentFaces <= minFaces || faceBudget <= 0) return 0;

    const std::vector<Candidate> candidates = buildCandidates(mesh, topo, algo, minNormalDot);
    std::vector<Candidate> selected;
    selected.reserve(static_cast<std::size_t>(maxCollapses));
    std::vector<unsigned char> blocked(mesh.vertices.size(), 0);

    for (const Candidate& c : candidates) {
        if (static_cast<int>(selected.size()) >= maxCollapses) break;
        if (blocked[c.from] || blocked[c.to]) continue;
        selected.push_back(c);
        blocked[c.from] = 1;
        blocked[c.to] = 1;
        for (int n : topo.neighbors[c.from]) blocked[n] = 1;
        for (int n : topo.neighbors[c.to]) blocked[n] = 1;
    }

    int applied = 0;
    for (const Candidate& c : selected) {
        if (applyCollapse(mesh, c)) ++applied;
    }
    mesh.cleanForDisplay();
    return applied;
}

void writeOff(const Mesh& mesh, const std::string& path) {
    std::unordered_map<int, int> remap;
    std::vector<Vec3> verts;
    std::vector<std::array<int, 3>> faces;
    std::unordered_set<TriKey, TriKeyHash> seenFaces;
    std::unordered_map<std::uint64_t, int> edgeUse;

    auto mapVertex = [&](int oldId) {
        auto it = remap.find(oldId);
        if (it != remap.end()) return it->second;
        const int id = static_cast<int>(verts.size());
        remap.emplace(oldId, id);
        verts.push_back(mesh.vertices[oldId].p);
        return id;
    };

    for (const Face& f : mesh.faces) {
        if (!f.alive) continue;
        if (!mesh.vertices[f.v[0]].alive || !mesh.vertices[f.v[1]].alive || !mesh.vertices[f.v[2]].alive) {
            continue;
        }
        const TriKey oldKey = canonicalTri(f.v[0], f.v[1], f.v[2]);
        if (!seenFaces.insert(oldKey).second) continue;
        const std::uint64_t e0Old = edgeKey(f.v[0], f.v[1]);
        const std::uint64_t e1Old = edgeKey(f.v[1], f.v[2]);
        const std::uint64_t e2Old = edgeKey(f.v[2], f.v[0]);
        if (edgeUse[e0Old] >= 2 || edgeUse[e1Old] >= 2 || edgeUse[e2Old] >= 2) continue;
        ++edgeUse[e0Old];
        ++edgeUse[e1Old];
        ++edgeUse[e2Old];
        faces.push_back({mapVertex(f.v[0]), mapVertex(f.v[1]), mapVertex(f.v[2])});
    }

    std::filesystem::create_directories(std::filesystem::path(path).parent_path());
    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot write: " + path);
    out << "OFF\n";
    out << verts.size() << ' ' << faces.size() << " 0\n";
    out << std::fixed << std::setprecision(9);
    for (const Vec3& v : verts) out << v.x << ' ' << v.y << ' ' << v.z << '\n';
    for (const auto& f : faces) out << "3 " << f[0] << ' ' << f[1] << ' ' << f[2] << '\n';
}

struct PolyFace {
    std::vector<int> v;
    bool alive = true;
    bool centerRemoved = false;
};

struct PolyTopology {
    std::vector<std::vector<int>> incidentFaces;
    std::vector<std::unordered_set<int>> neighbors;
    std::unordered_map<std::uint64_t, std::vector<int>> edgeFaces;
    std::vector<unsigned char> boundaryVertex;
};

struct PolyMesh {
    std::vector<Vertex> vertices;
    std::vector<PolyFace> faces;
    double diag2 = 1.0;
    double minArea2 = 1e-30;
    double minEdge2 = 1e-24;

    void updateScale() {
        Vec3 mn{std::numeric_limits<double>::max(),
                std::numeric_limits<double>::max(),
                std::numeric_limits<double>::max()};
        Vec3 mx{std::numeric_limits<double>::lowest(),
                std::numeric_limits<double>::lowest(),
                std::numeric_limits<double>::lowest()};
        bool any = false;
        for (const auto& v : vertices) {
            if (!v.alive) continue;
            any = true;
            mn.x = std::min(mn.x, v.p.x);
            mn.y = std::min(mn.y, v.p.y);
            mn.z = std::min(mn.z, v.p.z);
            mx.x = std::max(mx.x, v.p.x);
            mx.y = std::max(mx.y, v.p.y);
            mx.z = std::max(mx.z, v.p.z);
        }
        diag2 = any ? std::max(norm2(mx - mn), 1e-24) : 1.0;
        minArea2 = std::max(1e-30, diag2 * diag2 * 1e-18);
        minEdge2 = std::max(1e-24, diag2 * 1e-14);
    }

    Vec3 faceNormal(const PolyFace& f) const {
        Vec3 n{};
        for (std::size_t i = 0; i < f.v.size(); ++i) {
            const Vec3& a = vertices[f.v[i]].p;
            const Vec3& b = vertices[f.v[(i + 1) % f.v.size()]].p;
            n.x += (a.y - b.y) * (a.z + b.z);
            n.y += (a.z - b.z) * (a.x + b.x);
            n.z += (a.x - b.x) * (a.y + b.y);
        }
        return normalized(n);
    }

    double faceArea2Fan(const PolyFace& f) const {
        if (f.v.size() < 3) return 0.0;
        double a2 = 0.0;
        const Vec3& p0 = vertices[f.v[0]].p;
        for (std::size_t i = 1; i + 1 < f.v.size(); ++i) {
            a2 += norm2(cross(vertices[f.v[i]].p - p0, vertices[f.v[i + 1]].p - p0));
        }
        return a2;
    }

    int aliveFaceCount() const {
        int n = 0;
        for (const auto& f : faces) {
            if (f.alive) ++n;
        }
        return n;
    }

    int aliveVertexCount() const {
        int n = 0;
        for (const auto& v : vertices) {
            if (v.alive) ++n;
        }
        return n;
    }

    void cleanForDisplay() {
        updateScale();
        for (auto& f : faces) {
            if (!f.alive) continue;
            std::vector<int> cleaned;
            cleaned.reserve(f.v.size());
            for (int id : f.v) {
                if (id < 0 || id >= static_cast<int>(vertices.size()) || !vertices[id].alive) {
                    cleaned.clear();
                    break;
                }
                if (cleaned.empty() || cleaned.back() != id) cleaned.push_back(id);
            }
            if (cleaned.size() > 1 && cleaned.front() == cleaned.back()) cleaned.pop_back();
            std::unordered_set<int> unique(cleaned.begin(), cleaned.end());
            if (cleaned.size() < 3 || unique.size() != cleaned.size()) {
                f.alive = false;
                continue;
            }
            f.v = std::move(cleaned);
            bool tiny = false;
            for (std::size_t i = 0; i < f.v.size(); ++i) {
                const Vec3& a = vertices[f.v[i]].p;
                const Vec3& b = vertices[f.v[(i + 1) % f.v.size()]].p;
                if (norm2(a - b) <= minEdge2) {
                    tiny = true;
                    break;
                }
            }
            if (tiny || faceArea2Fan(f) <= minArea2) f.alive = false;
        }

        std::unordered_map<std::uint64_t, int> edgeUse;
        for (auto& f : faces) {
            if (!f.alive) continue;
            bool invalid = false;
            std::vector<std::uint64_t> faceEdges;
            faceEdges.reserve(f.v.size());
            for (std::size_t i = 0; i < f.v.size(); ++i) {
                const std::uint64_t e = edgeKey(f.v[i], f.v[(i + 1) % f.v.size()]);
                if (edgeUse[e] >= 2) {
                    invalid = true;
                    break;
                }
                faceEdges.push_back(e);
            }
            if (invalid) {
                f.alive = false;
            } else {
                for (std::uint64_t e : faceEdges) ++edgeUse[e];
            }
        }
    }
};

PolyMesh readPolyOff(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open input: " + path);
    std::string line;
    if (!nextDataLine(in, line) || line != "OFF") throw std::runtime_error("input is not OFF: " + path);
    if (!nextDataLine(in, line)) throw std::runtime_error("missing OFF counts: " + path);
    std::istringstream counts(line);
    int nv = 0;
    int nf = 0;
    int ne = 0;
    counts >> nv >> nf >> ne;
    if (!counts || nv < 0 || nf < 0) throw std::runtime_error("invalid OFF counts: " + path);

    PolyMesh mesh;
    mesh.vertices.resize(nv);
    for (int i = 0; i < nv; ++i) {
        if (!nextDataLine(in, line)) throw std::runtime_error("truncated OFF vertices: " + path);
        std::istringstream row(line);
        row >> mesh.vertices[i].p.x >> mesh.vertices[i].p.y >> mesh.vertices[i].p.z;
        if (!row) throw std::runtime_error("invalid OFF vertex row: " + path);
    }
    mesh.faces.reserve(nf);
    for (int i = 0; i < nf; ++i) {
        if (!nextDataLine(in, line)) throw std::runtime_error("truncated OFF faces: " + path);
        std::istringstream row(line);
        int deg = 0;
        row >> deg;
        PolyFace f;
        f.v.resize(static_cast<std::size_t>(deg));
        for (int j = 0; j < deg; ++j) row >> f.v[j];
        if (!row) throw std::runtime_error("invalid OFF face row: " + path);
        if (deg >= 3) mesh.faces.push_back(std::move(f));
    }
    mesh.cleanForDisplay();
    return mesh;
}

PolyTopology buildPolyTopology(const PolyMesh& mesh) {
    PolyTopology topo;
    topo.incidentFaces.assign(mesh.vertices.size(), {});
    topo.neighbors.assign(mesh.vertices.size(), {});
    topo.boundaryVertex.assign(mesh.vertices.size(), 0);
    for (int fi = 0; fi < static_cast<int>(mesh.faces.size()); ++fi) {
        const PolyFace& f = mesh.faces[fi];
        if (!f.alive) continue;
        for (std::size_t i = 0; i < f.v.size(); ++i) {
            const int a = f.v[i];
            const int b = f.v[(i + 1) % f.v.size()];
            topo.incidentFaces[a].push_back(fi);
            topo.neighbors[a].insert(b);
            topo.neighbors[b].insert(a);
            topo.edgeFaces[edgeKey(a, b)].push_back(fi);
        }
    }
    for (const auto& kv : topo.edgeFaces) {
        if (kv.second.size() != 2) {
            const int a = edgeA(kv.first);
            const int b = edgeB(kv.first);
            if (a >= 0 && a < static_cast<int>(topo.boundaryVertex.size())) topo.boundaryVertex[a] = 1;
            if (b >= 0 && b < static_cast<int>(topo.boundaryVertex.size())) topo.boundaryVertex[b] = 1;
        }
    }
    return topo;
}

Vec3 polyNewellNormal(const PolyMesh& mesh, const std::vector<int>& ring) {
    Vec3 n{};
    for (std::size_t i = 0; i < ring.size(); ++i) {
        const Vec3& a = mesh.vertices[ring[i]].p;
        const Vec3& b = mesh.vertices[ring[(i + 1) % ring.size()]].p;
        n.x += (a.y - b.y) * (a.z + b.z);
        n.y += (a.z - b.z) * (a.x + b.x);
        n.z += (a.x - b.x) * (a.y + b.y);
    }
    return normalized(n);
}

bool polyIsPlanar(const PolyMesh& mesh, const std::vector<int>& ring, double epsilon) {
    if (ring.size() < 3) return false;
    Vec3 barycenter{};
    for (int id : ring) barycenter = barycenter + mesh.vertices[id].p;
    barycenter = barycenter / static_cast<double>(ring.size());

    double characteristicSize = 0.0;
    for (int id : ring) characteristicSize += norm2(mesh.vertices[id].p - barycenter);
    characteristicSize /= static_cast<double>(ring.size());
    if (characteristicSize <= mesh.minEdge2) return false;

    const Vec3 n = polyNewellNormal(mesh, ring);
    if (norm2(n) <= 0.0) return false;
    for (int id : ring) {
        const double dist = dot(mesh.vertices[id].p - barycenter, n);
        if (dist * dist > epsilon * characteristicSize) return false;
    }
    return true;
}

bool polyProjectedSimpleConvex(const PolyMesh& mesh,
                               const std::vector<int>& ring,
                               const Vec3& normal,
                               bool requireConvex) {
    const int n = static_cast<int>(ring.size());
    if (n < 3) return false;
    std::vector<Vec2> p(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) p[i] = projectPoint(mesh.vertices[ring[i]].p, normal);
    for (int i = 0; i < n; ++i) {
        const int i2 = (i + 1) % n;
        for (int j = i + 1; j < n; ++j) {
            const int j2 = (j + 1) % n;
            if (i == j || i2 == j || j2 == i) continue;
            if (segmentsIntersect2(p[i], p[i2], p[j], p[j2])) return false;
        }
    }
    if (!requireConvex || n == 3) return true;
    double sign = 0.0;
    for (int i = 0; i < n; ++i) {
        const double c = cross2(p[i], p[(i + 1) % n], p[(i + 2) % n]);
        if (std::fabs(c) <= 1e-12) return false;
        if (sign == 0.0) sign = c;
        else if (sign * c < 0.0) return false;
    }
    return true;
}

struct PpmcIncident {
    int faceId = -1;
    std::vector<int> segment;
};

struct PpmcCandidate {
    int center = -1;
    std::vector<int> boundary;
    std::vector<int> affectedFaces;
    std::vector<PpmcIncident> incidents;
    double cost = 0.0;
};

bool collectPpmcBoundary(const PolyMesh& mesh,
                         const PolyTopology& topo,
                         int center,
                         std::vector<int>& boundary,
                         std::vector<PpmcIncident>& incidents) {
    boundary.clear();
    incidents.clear();
    if (center < 0 || center >= static_cast<int>(mesh.vertices.size())) return false;
    if (!mesh.vertices[center].alive || topo.boundaryVertex[center]) return false;

    std::unordered_map<int, int> segmentByFirstVertex;
    for (int fid : topo.incidentFaces[center]) {
        const PolyFace& f = mesh.faces[fid];
        if (!f.alive) continue;
        int pos = -1;
        for (int i = 0; i < static_cast<int>(f.v.size()); ++i) {
            if (f.v[i] == center) {
                pos = i;
                break;
            }
        }
        if (pos < 0) return false;
        std::vector<int> seg;
        for (int off = 1; off < static_cast<int>(f.v.size()); ++off) {
            const int id = f.v[(pos + off) % static_cast<int>(f.v.size())];
            if (id == center) return false;
            seg.push_back(id);
        }
        if (seg.size() < 2) return false;
        const int idx = static_cast<int>(incidents.size());
        if (!segmentByFirstVertex.emplace(seg.front(), idx).second) return false;
        incidents.push_back({fid, std::move(seg)});
    }
    if (incidents.size() < 3) return false;

    int start = incidents.front().segment.front();
    int cur = start;
    boundary.push_back(start);
    for (std::size_t guard = 0; guard <= incidents.size(); ++guard) {
        auto it = segmentByFirstVertex.find(cur);
        if (it == segmentByFirstVertex.end()) return false;
        const std::vector<int>& seg = incidents[static_cast<std::size_t>(it->second)].segment;
        boundary.push_back(seg.back());
        cur = seg.back();
        if (cur == start) break;
    }
    if (cur != start) return false;
    if (!boundary.empty() && boundary.back() == boundary.front()) boundary.pop_back();
    std::unordered_set<int> unique(boundary.begin(), boundary.end());
    return boundary.size() >= 3 && unique.size() == boundary.size();
}

bool validPolyRingForDisplay(const PolyMesh& mesh, const std::vector<int>& ring, bool requireConvex) {
    if (ring.size() < 3) return false;
    std::unordered_set<int> unique;
    unique.reserve(ring.size());
    for (std::size_t i = 0; i < ring.size(); ++i) {
        const int a = ring[i];
        const int b = ring[(i + 1) % ring.size()];
        if (a < 0 || a >= static_cast<int>(mesh.vertices.size()) || !mesh.vertices[a].alive) return false;
        if (!unique.insert(a).second) return false;
        if (b < 0 || b >= static_cast<int>(mesh.vertices.size()) || !mesh.vertices[b].alive) return false;
        if (norm2(mesh.vertices[a].p - mesh.vertices[b].p) <= mesh.minEdge2) return false;
    }

    const Vec3 n = polyNewellNormal(mesh, ring);
    if (norm2(n) <= 0.0) return false;
    if (!polyIsPlanar(mesh, ring, kPpmcPlanarityEpsilon)) return false;
    return polyProjectedSimpleConvex(mesh, ring, n, requireConvex);
}

double ppmcRemovalError(const PolyMesh& mesh, int center, const std::vector<int>& boundary) {
    if (boundary.size() < 3) return std::numeric_limits<double>::max();
    Vec3 b{};
    double perimeter = 0.0;
    for (std::size_t i = 0; i < boundary.size(); ++i) {
        const Vec3& p = mesh.vertices[boundary[i]].p;
        const Vec3& q = mesh.vertices[boundary[(i + 1) % boundary.size()]].p;
        b = b + p;
        perimeter += std::sqrt(std::max(0.0, norm2(q - p)));
    }
    b = b / static_cast<double>(boundary.size());
    if (perimeter <= 0.0) return std::numeric_limits<double>::max();

    double area = 0.0;
    for (std::size_t i = 0; i < boundary.size(); ++i) {
        const Vec3& p = mesh.vertices[boundary[i]].p;
        const Vec3& q = mesh.vertices[boundary[(i + 1) % boundary.size()]].p;
        area += 0.5 * std::sqrt(std::max(0.0, norm2(cross(p - b, q - b))));
    }

    const Vec3 n = polyNewellNormal(mesh, boundary);
    const double height = std::fabs(dot(mesh.vertices[center].p - b, n));
    const double volume = area * height / 3.0;
    if (volume <= 0.0) return 0.0;
    return std::cbrt(volume) * static_cast<double>(boundary.size()) / perimeter;
}

bool validatePpmcReplacement(const PolyMesh& mesh,
                             const PolyTopology& topo,
                             const std::vector<int>& boundary,
                             const std::vector<PpmcIncident>& incidents) {
    if (!validPolyRingForDisplay(mesh, boundary, true)) return false;

    std::unordered_set<int> affected;
    affected.reserve(incidents.size());
    for (const PpmcIncident& incident : incidents) affected.insert(incident.faceId);

    std::unordered_map<std::uint64_t, int> baseEdgeUse;
    std::unordered_map<std::uint64_t, int> newEdgeUse;
    auto outsideEdgeUse = [&](std::uint64_t e) {
        auto cached = baseEdgeUse.find(e);
        if (cached != baseEdgeUse.end()) return cached->second;
        int count = 0;
        const auto it = topo.edgeFaces.find(e);
        if (it != topo.edgeFaces.end()) {
            for (int fid : it->second) {
                if (fid >= 0 && fid < static_cast<int>(mesh.faces.size()) &&
                    mesh.faces[fid].alive && affected.find(fid) == affected.end()) {
                    ++count;
                }
            }
        }
        baseEdgeUse.emplace(e, count);
        return count;
    };

    auto addFace = [&](const std::vector<int>& ring, bool requireConvex) {
        if (!validPolyRingForDisplay(mesh, ring, requireConvex)) return false;
        for (std::size_t i = 0; i < ring.size(); ++i) {
            const std::uint64_t e = edgeKey(ring[i], ring[(i + 1) % ring.size()]);
            if (outsideEdgeUse(e) + ++newEdgeUse[e] > 2) return false;
        }
        return true;
    };

    for (const PpmcIncident& incident : incidents) {
        if (incident.segment.size() >= 3 && !addFace(incident.segment, true)) return false;
    }
    return addFace(boundary, true);
}

bool buildPpmcCandidate(const PolyMesh& mesh,
                        const PolyTopology& topo,
                        int center,
                        PpmcCandidate& out) {
    std::vector<int> boundary;
    std::vector<PpmcIncident> incidents;
    if (!collectPpmcBoundary(mesh, topo, center, boundary, incidents)) return false;
    if (boundary.size() > 16) return false;
    const double removalError = ppmcRemovalError(mesh, center, boundary);
    if (removalError >= 0.5) return false;
    if (!validatePpmcReplacement(mesh, topo, boundary, incidents)) return false;

    std::vector<int> affected;
    affected.reserve(incidents.size());
    for (const PpmcIncident& incident : incidents) affected.push_back(incident.faceId);
    std::unordered_set<int> affectedSet(affected.begin(), affected.end());
    std::unordered_set<int> boundarySet(boundary.begin(), boundary.end());
    std::unordered_map<int, int> boundaryPos;
    for (int i = 0; i < static_cast<int>(boundary.size()); ++i) {
        boundaryPos[boundary[i]] = i;
    }
    auto adjacentOnBoundary = [&](int a, int b) {
        auto ia = boundaryPos.find(a);
        auto ib = boundaryPos.find(b);
        if (ia == boundaryPos.end() || ib == boundaryPos.end()) return false;
        const int n = static_cast<int>(boundary.size());
        const int d = std::abs(ia->second - ib->second);
        return d == 1 || d == n - 1;
    };

    for (int v : boundary) {
        int borderNeighbors = 0;
        for (int nb : topo.neighbors[v]) {
            if (boundarySet.find(nb) == boundarySet.end()) continue;
            ++borderNeighbors;
            if (!adjacentOnBoundary(v, nb)) {
                const auto it = topo.edgeFaces.find(edgeKey(v, nb));
                if (it != topo.edgeFaces.end()) {
                    for (int fid : it->second) {
                        if (affectedSet.find(fid) == affectedSet.end() && mesh.faces[fid].alive) return false;
                    }
                }
            }
        }
        if (borderNeighbors > 2) return false;
    }

    out.center = center;
    out.boundary = std::move(boundary);
    out.affectedFaces = std::move(affected);
    out.incidents = std::move(incidents);
    out.cost = removalError;
    return true;
}

bool applyPpmcRemoval(PolyMesh& mesh, const PpmcCandidate& c) {
    if (c.center < 0 || c.center >= static_cast<int>(mesh.vertices.size())) return false;
    if (!mesh.vertices[c.center].alive || c.boundary.size() < 3) return false;
    for (const PpmcIncident& incident : c.incidents) {
        if (incident.faceId < 0 || incident.faceId >= static_cast<int>(mesh.faces.size())) return false;
        mesh.faces[incident.faceId].alive = false;
        if (incident.segment.size() >= 3) mesh.faces.push_back({incident.segment, true, false});
    }
    mesh.vertices[c.center].alive = false;
    mesh.faces.push_back({c.boundary, true, true});
    return true;
}

int simplifyPpmcRound(PolyMesh& mesh, double ratio, int minFaces) {
    mesh.cleanForDisplay();
    const PolyTopology topo = buildPolyTopology(mesh);
    const int currentFaces = mesh.aliveFaceCount();
    const int targetFaces = std::max(minFaces, static_cast<int>(std::floor(currentFaces * ratio)));
    if (currentFaces <= minFaces || currentFaces <= targetFaces) return 0;
    const int maxRemovals = std::max(1, (currentFaces - targetFaces + 1) / 2);

    std::vector<PpmcCandidate> candidates;
    for (int v = 0; v < static_cast<int>(mesh.vertices.size()); ++v) {
        PpmcCandidate c;
        if (buildPpmcCandidate(mesh, topo, v, c)) candidates.push_back(std::move(c));
    }
    std::sort(candidates.begin(), candidates.end(), [](const PpmcCandidate& a, const PpmcCandidate& b) {
        if (a.cost != b.cost) return a.cost < b.cost;
        return a.center < b.center;
    });

    std::vector<unsigned char> blocked(mesh.vertices.size(), 0);
    std::vector<unsigned char> faceBlocked(mesh.faces.size(), 0);
    std::vector<PpmcCandidate> selected;
    selected.reserve(static_cast<std::size_t>(maxRemovals));
    for (const PpmcCandidate& c : candidates) {
        if (static_cast<int>(selected.size()) >= maxRemovals) break;
        if (blocked[c.center]) continue;
        bool conflict = false;
        for (int f : c.affectedFaces) {
            if (f >= 0 && f < static_cast<int>(faceBlocked.size()) && faceBlocked[f]) {
                conflict = true;
                break;
            }
        }
        for (int v : c.boundary) {
            if (blocked[v]) {
                conflict = true;
                break;
            }
        }
        if (conflict) continue;
        selected.push_back(c);
        blocked[c.center] = 1;
        for (int v : c.boundary) blocked[v] = 1;
        for (int f : c.affectedFaces) {
            if (f >= 0 && f < static_cast<int>(faceBlocked.size())) faceBlocked[f] = 1;
        }
    }

    int applied = 0;
    for (const PpmcCandidate& c : selected) {
        if (applyPpmcRemoval(mesh, c)) ++applied;
    }
    mesh.cleanForDisplay();
    return applied;
}

void writePolyOffNonStrict(const PolyMesh& mesh, const std::string& path) {
    std::unordered_map<int, int> remap;
    std::vector<Vec3> verts;
    std::vector<std::vector<int>> faces;
    std::unordered_map<std::uint64_t, int> edgeUse;

    auto mapVertex = [&](int oldId) {
        auto it = remap.find(oldId);
        if (it != remap.end()) return it->second;
        const int id = static_cast<int>(verts.size());
        remap.emplace(oldId, id);
        verts.push_back(mesh.vertices[oldId].p);
        return id;
    };

    for (const PolyFace& f : mesh.faces) {
        if (!f.alive || f.v.size() < 3) continue;
        if (!validPolyRingForDisplay(mesh, f.v, true)) continue;

        bool ok = true;
        std::vector<std::uint64_t> faceEdges;
        faceEdges.reserve(f.v.size());
        for (std::size_t i = 0; i < f.v.size(); ++i) {
            const std::uint64_t e = edgeKey(f.v[i], f.v[(i + 1) % f.v.size()]);
            if (edgeUse[e] >= 2) {
                ok = false;
                break;
            }
            faceEdges.push_back(e);
        }
        if (!ok) continue;
        for (std::uint64_t e : faceEdges) ++edgeUse[e];

        std::vector<int> face;
        face.reserve(f.v.size());
        for (int v : f.v) face.push_back(mapVertex(v));
        faces.push_back(std::move(face));
    }

    std::filesystem::create_directories(std::filesystem::path(path).parent_path());
    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot write: " + path);
    out << "OFF\n";
    out << verts.size() << ' ' << faces.size() << " 0\n";
    out << std::fixed << std::setprecision(9);
    for (const Vec3& v : verts) out << v.x << ' ' << v.y << ' ' << v.z << '\n';
    for (const auto& f : faces) {
        out << f.size();
        for (int v : f) out << ' ' << v;
        out << '\n';
    }
}

void writePolyOffTriangulated(const PolyMesh& mesh, const std::string& path) {
    std::unordered_map<int, int> remap;
    std::vector<Vec3> verts;
    std::vector<std::array<int, 3>> faces;
    std::unordered_set<TriKey, TriKeyHash> seenFaces;
    std::unordered_map<std::uint64_t, int> edgeUse;

    auto mapVertex = [&](int oldId) {
        auto it = remap.find(oldId);
        if (it != remap.end()) return it->second;
        const int id = static_cast<int>(verts.size());
        remap.emplace(oldId, id);
        verts.push_back(mesh.vertices[oldId].p);
        return id;
    };

    auto addTriangle = [&](int a, int b, int c, const Vec3& faceNormal) {
        if (a == b || b == c || c == a) return;
        if (a < 0 || a >= static_cast<int>(mesh.vertices.size()) || !mesh.vertices[a].alive) return;
        if (b < 0 || b >= static_cast<int>(mesh.vertices.size()) || !mesh.vertices[b].alive) return;
        if (c < 0 || c >= static_cast<int>(mesh.vertices.size()) || !mesh.vertices[c].alive) return;

        const Vec3 raw = cross(mesh.vertices[b].p - mesh.vertices[a].p,
                               mesh.vertices[c].p - mesh.vertices[a].p);
        if (norm2(raw) <= mesh.minArea2) return;
        if (dot(raw, faceNormal) < 0.0) std::swap(b, c);

        const TriKey oldKey = canonicalTri(a, b, c);
        if (!seenFaces.insert(oldKey).second) return;

        const std::uint64_t e0Old = edgeKey(a, b);
        const std::uint64_t e1Old = edgeKey(b, c);
        const std::uint64_t e2Old = edgeKey(c, a);
        if (edgeUse[e0Old] >= 2 || edgeUse[e1Old] >= 2 || edgeUse[e2Old] >= 2) return;
        ++edgeUse[e0Old];
        ++edgeUse[e1Old];
        ++edgeUse[e2Old];

        faces.push_back({mapVertex(a), mapVertex(b), mapVertex(c)});
    };

    for (const PolyFace& f : mesh.faces) {
        if (!f.alive || f.v.size() < 3) continue;
        if (!validPolyRingForDisplay(mesh, f.v, true)) continue;
        const Vec3 n = polyNewellNormal(mesh, f.v);
        for (std::size_t i = 1; i + 1 < f.v.size(); ++i) {
            addTriangle(f.v[0], f.v[i], f.v[i + 1], n);
        }
    }

    std::filesystem::create_directories(std::filesystem::path(path).parent_path());
    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot write: " + path);
    out << "OFF\n";
    out << verts.size() << ' ' << faces.size() << " 0\n";
    out << std::fixed << std::setprecision(9);
    for (const Vec3& v : verts) out << v.x << ' ' << v.y << ' ' << v.z << '\n';
    for (const auto& f : faces) out << "3 " << f[0] << ' ' << f[1] << ' ' << f[2] << '\n';
}

int runPpmc(const Args& args) {
    PolyMesh mesh = readPolyOff(args.input);
    std::filesystem::create_directories(args.outputDir);
    auto writePpmcOff = [&](const std::string& path) {
        if (args.ppmcStrictTriangulate) writePolyOffTriangulated(mesh, path);
        else writePolyOffNonStrict(mesh, path);
    };
    writePpmcOff(args.outputDir + "/round_00.off");
    int exported = 1;
    int emptyPasses = 0;
    for (int r = 1; r <= args.rounds; ++r) {
        const int removed = simplifyPpmcRound(mesh, args.ratio, args.minFaces);
        if (removed <= 0) {
            if (++emptyPasses >= 4) break;
            continue;
        }
        emptyPasses = 0;
        std::ostringstream name;
        name << args.outputDir << "/round_" << std::setw(2) << std::setfill('0') << r << ".off";
        writePpmcOff(name.str());
        ++exported;
        if (mesh.aliveFaceCount() <= args.minFaces) break;
    }
    std::cout << "algo=ppmc"
              << "\texported_files=" << exported
              << "\tstrict_triangulate=" << (args.ppmcStrictTriangulate ? "true" : "false")
              << "\tvertices=" << mesh.aliveVertexCount()
              << "\tfaces=" << mesh.aliveFaceCount()
              << "\toutput_dir=" << args.outputDir << '\n';
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        Args args = parseArgs(argc, argv);
        if (args.normalDot < -0.5) args.normalDot = defaultNormalDot(args.algo);
        if (args.algo == "ppmc") return runPpmc(args);

        Mesh mesh = readOff(args.input);
        std::filesystem::create_directories(args.outputDir);
        writeOff(mesh, args.outputDir + "/round_00.off");

        int exported = 1;
        int emptyPasses = 0;
        for (int r = 1; r <= args.rounds; ++r) {
            const int collapsed = simplifyRound(mesh, args.algo, r, args.ratio, args.minFaces, args.normalDot);
            if (collapsed <= 0) {
                if (++emptyPasses >= 4) break;
                continue;
            }
            emptyPasses = 0;
            std::ostringstream name;
            name << args.outputDir << "/round_" << std::setw(2) << std::setfill('0') << r << ".off";
            writeOff(mesh, name.str());
            ++exported;
            if (mesh.aliveFaceCount() <= args.minFaces) break;
        }

        std::cout << "algo=" << args.algo
                  << "\texported_files=" << exported
                  << "\tvertices=" << mesh.aliveVertexCount()
                  << "\tfaces=" << mesh.aliveFaceCount()
                  << "\toutput_dir=" << args.outputDir << '\n';
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "VisualQualityCompress: " << e.what() << '\n';
        return 1;
    }
}
