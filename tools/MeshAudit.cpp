#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <numeric>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Face {
    int a = -1;
    int b = -1;
    int c = -1;
};

struct Mesh {
    std::vector<Vec3> vertices;
    std::vector<Face> faces;
    std::size_t sourceFaces = 0;
    std::size_t nonTriangleSourceFaces = 0;
    std::size_t nonPlanarSourceFaces = 0;
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
    std::size_t operator()(const TriKey& k) const {
        std::uint64_t h = 1469598103934665603ull;
        auto mix = [&](std::uint32_t v) {
            h ^= v;
            h *= 1099511628211ull;
        };
        mix(static_cast<std::uint32_t>(k.a));
        mix(static_cast<std::uint32_t>(k.b));
        mix(static_cast<std::uint32_t>(k.c));
        return static_cast<std::size_t>(h);
    }
};

struct Dsu {
    std::vector<int> parent;
    std::vector<int> rank;

    explicit Dsu(std::size_t n) : parent(n), rank(n, 0) {
        std::iota(parent.begin(), parent.end(), 0);
    }

    int find(int x) {
        while (parent[static_cast<std::size_t>(x)] != x) {
            parent[static_cast<std::size_t>(x)] =
                parent[static_cast<std::size_t>(parent[static_cast<std::size_t>(x)])];
            x = parent[static_cast<std::size_t>(x)];
        }
        return x;
    }

    void unite(int a, int b) {
        a = find(a);
        b = find(b);
        if (a == b) return;
        if (rank[static_cast<std::size_t>(a)] < rank[static_cast<std::size_t>(b)]) {
            std::swap(a, b);
        }
        parent[static_cast<std::size_t>(b)] = a;
        if (rank[static_cast<std::size_t>(a)] == rank[static_cast<std::size_t>(b)]) {
            ++rank[static_cast<std::size_t>(a)];
        }
    }
};

struct Audit {
    std::size_t vertices = 0;
    std::size_t faces = 0;
    std::size_t sourceFaces = 0;
    std::size_t nonTriangleSourceFaces = 0;
    std::size_t nonPlanarSourceFaces = 0;
    std::size_t invalidFaces = 0;
    std::size_t repeatedVertexFaces = 0;
    std::size_t zeroAreaFaces = 0;
    std::size_t duplicateFaces = 0;
    std::size_t uniqueEdges = 0;
    std::size_t boundaryEdges = 0;
    std::size_t nonManifoldEdges = 0;
    std::size_t connectedComponents = 0;
    std::size_t isolatedVertices = 0;
    std::size_t badLinkVertices = 0;
    std::size_t closedLinkVertices = 0;
    std::size_t boundaryLinkVertices = 0;
    std::size_t maxIncidentFaces = 0;
};

static bool nextToken(std::istream& in, std::string& tok) {
    while (in >> tok) {
        if (!tok.empty() && tok[0] == '#') {
            std::string rest;
            std::getline(in, rest);
            continue;
        }
        return true;
    }
    return false;
}

static bool parseInt(std::istream& in, int& out) {
    std::string tok;
    if (!nextToken(in, tok)) return false;
    out = std::stoi(tok);
    return true;
}

static bool parseDouble(std::istream& in, double& out) {
    std::string tok;
    if (!nextToken(in, tok)) return false;
    out = std::stod(tok);
    return true;
}

static Vec3 sub(const Vec3& a, const Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

static Vec3 add(const Vec3& a, const Vec3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

static Vec3 div(const Vec3& a, double s) {
    return {a.x / s, a.y / s, a.z / s};
}

static double dot3(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static Vec3 cross3(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

static double norm2(const Vec3& v) {
    return dot3(v, v);
}

static bool sourceIndicesValid(const std::vector<int>& ids, std::size_t vertexCount) {
    for (int id : ids) {
        if (id < 0 || static_cast<std::size_t>(id) >= vertexCount) return false;
    }
    return true;
}

static Vec3 sourcePolygonNormal(const std::vector<Vec3>& vertices, const std::vector<int>& ids) {
    Vec3 n{};
    for (std::size_t i = 0; i < ids.size(); ++i) {
        const Vec3& p = vertices[static_cast<std::size_t>(ids[i])];
        const Vec3& q = vertices[static_cast<std::size_t>(ids[(i + 1) % ids.size()])];
        n = add(n, cross3(p, q));
    }
    const double n2 = norm2(n);
    return n2 <= 0.0 ? Vec3{} : div(n, std::sqrt(n2));
}

static bool sourcePolygonPlanar(const std::vector<Vec3>& vertices,
                                const std::vector<int>& ids,
                                double epsilon) {
    if (ids.size() <= 3) return true;
    if (!sourceIndicesValid(ids, vertices.size())) return true;

    Vec3 barycenter{};
    for (int id : ids) barycenter = add(barycenter, vertices[static_cast<std::size_t>(id)]);
    barycenter = div(barycenter, static_cast<double>(ids.size()));

    double characteristicSize = 0.0;
    for (int id : ids) {
        characteristicSize += norm2(sub(vertices[static_cast<std::size_t>(id)], barycenter));
    }
    characteristicSize /= static_cast<double>(ids.size());
    if (characteristicSize <= 0.0) return false;

    const Vec3 normal = sourcePolygonNormal(vertices, ids);
    if (norm2(normal) <= 0.0) return false;
    for (int id : ids) {
        const double dist = dot3(normal, sub(vertices[static_cast<std::size_t>(id)], barycenter));
        if (dist * dist > epsilon * characteristicSize) return false;
    }
    return true;
}

static bool loadOFF(const std::string& path, Mesh& mesh) {
    std::ifstream in(path);
    if (!in) {
        std::cerr << "error=cannot_open path=" << path << "\n";
        return false;
    }

    std::string header;
    if (!nextToken(in, header)) return false;
    const bool hasColor = header == "COFF";
    const bool hasNormal = header == "NOFF";
    if (header != "OFF" && !hasColor && !hasNormal) {
        std::cerr << "error=unsupported_header header=" << header << "\n";
        return false;
    }

    int nv = 0;
    int nf = 0;
    int ne = 0;
    if (!parseInt(in, nv) || !parseInt(in, nf) || !parseInt(in, ne) || nv < 0 || nf < 0) {
        std::cerr << "error=invalid_counts\n";
        return false;
    }

    mesh.vertices.resize(static_cast<std::size_t>(nv));
    for (int i = 0; i < nv; ++i) {
        if (!parseDouble(in, mesh.vertices[static_cast<std::size_t>(i)].x) ||
            !parseDouble(in, mesh.vertices[static_cast<std::size_t>(i)].y) ||
            !parseDouble(in, mesh.vertices[static_cast<std::size_t>(i)].z)) {
            std::cerr << "error=truncated_vertices index=" << i << "\n";
            return false;
        }
        if (hasNormal) {
            double tmp = 0.0;
            for (int k = 0; k < 3; ++k) {
                if (!parseDouble(in, tmp)) return false;
            }
        }
        if (hasColor) {
            std::string rest;
            std::getline(in, rest);
        }
    }

    mesh.sourceFaces = static_cast<std::size_t>(nf);
    mesh.faces.reserve(static_cast<std::size_t>(nf));
    for (int i = 0; i < nf; ++i) {
        int degree = 0;
        if (!parseInt(in, degree) || degree < 0) {
            std::cerr << "error=invalid_face_degree index=" << i << "\n";
            return false;
        }
        std::vector<int> ids(static_cast<std::size_t>(degree));
        for (int j = 0; j < degree; ++j) {
            if (!parseInt(in, ids[static_cast<std::size_t>(j)])) {
                std::cerr << "error=truncated_face index=" << i << "\n";
                return false;
            }
        }
        if (degree != 3) ++mesh.nonTriangleSourceFaces;
        if (degree > 3 && !sourcePolygonPlanar(mesh.vertices, ids, 0.05)) {
            ++mesh.nonPlanarSourceFaces;
        }
        if (degree < 3) continue;
        for (int j = 1; j + 1 < degree; ++j) {
            mesh.faces.push_back({ids[0], ids[static_cast<std::size_t>(j)],
                                  ids[static_cast<std::size_t>(j + 1)]});
        }
    }
    return true;
}

static double area2(const Vec3& a, const Vec3& b, const Vec3& c) {
    const double ux = b.x - a.x;
    const double uy = b.y - a.y;
    const double uz = b.z - a.z;
    const double vx = c.x - a.x;
    const double vy = c.y - a.y;
    const double vz = c.z - a.z;
    const double cx = uy * vz - uz * vy;
    const double cy = uz * vx - ux * vz;
    const double cz = ux * vy - uy * vx;
    return cx * cx + cy * cy + cz * cz;
}

static TriKey canonicalFace(const Face& f) {
    std::array<int, 3> v{f.a, f.b, f.c};
    std::sort(v.begin(), v.end());
    return {v[0], v[1], v[2]};
}

static std::uint64_t edgeKey(int a, int b) {
    if (a > b) std::swap(a, b);
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(a)) << 32) |
           static_cast<std::uint32_t>(b);
}

static bool validIndex(const Face& f, std::size_t n) {
    return f.a >= 0 && f.b >= 0 && f.c >= 0 &&
           static_cast<std::size_t>(f.a) < n &&
           static_cast<std::size_t>(f.b) < n &&
           static_cast<std::size_t>(f.c) < n;
}

static bool repeatedVertex(const Face& f) {
    return f.a == f.b || f.b == f.c || f.c == f.a;
}

static bool checkLocalLink(const std::vector<std::pair<int, int>>& pairs) {
    if (pairs.empty()) return false;

    std::vector<int> nodes;
    std::vector<int> degrees;
    std::vector<int> parent;

    auto findLocal = [&](int x) {
        while (parent[static_cast<std::size_t>(x)] != x) {
            parent[static_cast<std::size_t>(x)] =
                parent[static_cast<std::size_t>(parent[static_cast<std::size_t>(x)])];
            x = parent[static_cast<std::size_t>(x)];
        }
        return x;
    };

    auto nodeIndex = [&](int v) {
        for (std::size_t i = 0; i < nodes.size(); ++i) {
            if (nodes[i] == v) return static_cast<int>(i);
        }
        nodes.push_back(v);
        degrees.push_back(0);
        parent.push_back(static_cast<int>(parent.size()));
        return static_cast<int>(nodes.size() - 1);
    };

    for (auto [a, b] : pairs) {
        if (a == b) return false;
        const int ia = nodeIndex(a);
        const int ib = nodeIndex(b);
        ++degrees[static_cast<std::size_t>(ia)];
        ++degrees[static_cast<std::size_t>(ib)];
        int ra = findLocal(ia);
        int rb = findLocal(ib);
        if (ra != rb) parent[static_cast<std::size_t>(rb)] = ra;
    }

    int root = -1;
    int degreeOne = 0;
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        const int d = degrees[i];
        if (d < 1 || d > 2) return false;
        if (d == 1) ++degreeOne;
        const int r = findLocal(static_cast<int>(i));
        if (root < 0) root = r;
        else if (r != root) return false;
    }

    return degreeOne == 0 || degreeOne == 2;
}

static Audit analyze(const Mesh& mesh) {
    Audit a;
    a.vertices = mesh.vertices.size();
    a.faces = mesh.faces.size();
    a.sourceFaces = mesh.sourceFaces;
    a.nonTriangleSourceFaces = mesh.nonTriangleSourceFaces;
    a.nonPlanarSourceFaces = mesh.nonPlanarSourceFaces;

    std::vector<char> topologicallyUsable(mesh.faces.size(), 0);
    std::unordered_set<TriKey, TriKeyHash> seenFaces;
    seenFaces.reserve(mesh.faces.size() * 2 + 1);

    std::unordered_map<std::uint64_t, int> edgeCounts;
    edgeCounts.reserve(mesh.faces.size() * 2 + 1);

    Dsu dsu(mesh.vertices.size());
    std::vector<std::uint32_t> incidentCount(mesh.vertices.size(), 0);
    std::vector<char> vertexUsed(mesh.vertices.size(), 0);

    for (std::size_t i = 0; i < mesh.faces.size(); ++i) {
        const Face& f = mesh.faces[i];
        if (!validIndex(f, mesh.vertices.size())) {
            ++a.invalidFaces;
            continue;
        }
        if (repeatedVertex(f)) {
            ++a.repeatedVertexFaces;
            continue;
        }
        if (area2(mesh.vertices[static_cast<std::size_t>(f.a)],
                  mesh.vertices[static_cast<std::size_t>(f.b)],
                  mesh.vertices[static_cast<std::size_t>(f.c)]) <= 1e-30) {
            ++a.zeroAreaFaces;
        }
        const TriKey key = canonicalFace(f);
        if (!seenFaces.insert(key).second) {
            ++a.duplicateFaces;
        }

        topologicallyUsable[i] = 1;
        edgeCounts[edgeKey(f.a, f.b)]++;
        edgeCounts[edgeKey(f.b, f.c)]++;
        edgeCounts[edgeKey(f.c, f.a)]++;
        dsu.unite(f.a, f.b);
        dsu.unite(f.b, f.c);
        vertexUsed[static_cast<std::size_t>(f.a)] = 1;
        vertexUsed[static_cast<std::size_t>(f.b)] = 1;
        vertexUsed[static_cast<std::size_t>(f.c)] = 1;
        ++incidentCount[static_cast<std::size_t>(f.a)];
        ++incidentCount[static_cast<std::size_t>(f.b)];
        ++incidentCount[static_cast<std::size_t>(f.c)];
    }

    a.uniqueEdges = edgeCounts.size();
    for (const auto& kv : edgeCounts) {
        if (kv.second == 1) ++a.boundaryEdges;
        if (kv.second > 2) ++a.nonManifoldEdges;
    }

    std::unordered_set<int> components;
    components.reserve(mesh.vertices.size());
    for (std::size_t i = 0; i < vertexUsed.size(); ++i) {
        if (vertexUsed[i]) components.insert(dsu.find(static_cast<int>(i)));
        else ++a.isolatedVertices;
    }
    a.connectedComponents = components.size();

    std::vector<std::size_t> offsets(mesh.vertices.size() + 1, 0);
    for (std::size_t i = 0; i < incidentCount.size(); ++i) {
        offsets[i + 1] = offsets[i] + incidentCount[i];
        a.maxIncidentFaces = std::max<std::size_t>(a.maxIncidentFaces, incidentCount[i]);
    }
    std::vector<std::pair<int, int>> linkPairs(offsets.back());
    std::vector<std::size_t> cursor = offsets;

    for (std::size_t i = 0; i < mesh.faces.size(); ++i) {
        if (!topologicallyUsable[i]) continue;
        const Face& f = mesh.faces[i];
        linkPairs[cursor[static_cast<std::size_t>(f.a)]++] = {f.b, f.c};
        linkPairs[cursor[static_cast<std::size_t>(f.b)]++] = {f.c, f.a};
        linkPairs[cursor[static_cast<std::size_t>(f.c)]++] = {f.a, f.b};
    }

    std::vector<std::pair<int, int>> local;
    for (std::size_t v = 0; v < mesh.vertices.size(); ++v) {
        const std::size_t begin = offsets[v];
        const std::size_t end = offsets[v + 1];
        if (begin == end) continue;
        local.assign(linkPairs.begin() + static_cast<std::ptrdiff_t>(begin),
                     linkPairs.begin() + static_cast<std::ptrdiff_t>(end));
        if (!checkLocalLink(local)) {
            ++a.badLinkVertices;
            continue;
        }
        int degreeOne = 0;
        std::vector<int> nodes;
        std::vector<int> degrees;
        for (auto [x, y] : local) {
            for (int n : {x, y}) {
                auto it = std::find(nodes.begin(), nodes.end(), n);
                if (it == nodes.end()) {
                    nodes.push_back(n);
                    degrees.push_back(1);
                } else {
                    ++degrees[static_cast<std::size_t>(it - nodes.begin())];
                }
            }
        }
        for (int d : degrees) {
            if (d == 1) ++degreeOne;
        }
        if (degreeOne == 0) ++a.closedLinkVertices;
        else ++a.boundaryLinkVertices;
    }

    return a;
}

static bool writeOFF(const std::string& path, const Mesh& mesh,
                     const std::vector<char>& keep) {
    std::vector<int> remap(mesh.vertices.size(), -1);
    std::vector<Face> faces;
    faces.reserve(mesh.faces.size());
    int nextId = 0;
    for (std::size_t i = 0; i < mesh.faces.size(); ++i) {
        if (!keep[i]) continue;
        const Face& f = mesh.faces[i];
        for (int v : {f.a, f.b, f.c}) {
            if (remap[static_cast<std::size_t>(v)] < 0) {
                remap[static_cast<std::size_t>(v)] = nextId++;
            }
        }
        faces.push_back({remap[static_cast<std::size_t>(f.a)],
                         remap[static_cast<std::size_t>(f.b)],
                         remap[static_cast<std::size_t>(f.c)]});
    }

    std::vector<Vec3> vertices;
    vertices.resize(static_cast<std::size_t>(nextId));
    for (std::size_t i = 0; i < remap.size(); ++i) {
        if (remap[i] >= 0) vertices[static_cast<std::size_t>(remap[i])] = mesh.vertices[i];
    }

    std::ofstream out(path);
    if (!out) return false;
    out.setf(std::ios::fixed);
    out.precision(9);
    out << "OFF\n" << vertices.size() << " " << faces.size() << " 0\n";
    for (const Vec3& p : vertices) {
        out << p.x << " " << p.y << " " << p.z << "\n";
    }
    for (const Face& f : faces) {
        out << "3 " << f.a << " " << f.b << " " << f.c << "\n";
    }
    return true;
}

static std::vector<char> basicRepairKeepMask(const Mesh& mesh) {
    std::vector<char> keep(mesh.faces.size(), 0);
    std::unordered_set<TriKey, TriKeyHash> seenFaces;
    seenFaces.reserve(mesh.faces.size() * 2 + 1);
    std::unordered_map<std::uint64_t, int> edgeCounts;
    edgeCounts.reserve(mesh.faces.size() * 2 + 1);

    for (std::size_t i = 0; i < mesh.faces.size(); ++i) {
        const Face& f = mesh.faces[i];
        if (!validIndex(f, mesh.vertices.size()) || repeatedVertex(f)) continue;
        if (area2(mesh.vertices[static_cast<std::size_t>(f.a)],
                  mesh.vertices[static_cast<std::size_t>(f.b)],
                  mesh.vertices[static_cast<std::size_t>(f.c)]) <= 1e-30) {
            continue;
        }
        const TriKey key = canonicalFace(f);
        if (!seenFaces.insert(key).second) continue;
        const std::uint64_t e0 = edgeKey(f.a, f.b);
        const std::uint64_t e1 = edgeKey(f.b, f.c);
        const std::uint64_t e2 = edgeKey(f.c, f.a);
        if (edgeCounts[e0] >= 2 || edgeCounts[e1] >= 2 || edgeCounts[e2] >= 2) {
            continue;
        }
        ++edgeCounts[e0];
        ++edgeCounts[e1];
        ++edgeCounts[e2];
        keep[i] = 1;
    }
    return keep;
}

static void printAudit(const Audit& a) {
    const bool valid = a.invalidFaces == 0 && a.repeatedVertexFaces == 0 &&
                       a.zeroAreaFaces == 0 && a.duplicateFaces == 0 &&
                       a.nonTriangleSourceFaces == 0;
    const bool twoManifoldWithBoundary =
        valid && a.nonManifoldEdges == 0 && a.badLinkVertices == 0;
    const bool closedTwoManifold =
        twoManifoldWithBoundary && a.boundaryEdges == 0;

    std::cout << "vertices=" << a.vertices << "\n"
              << "faces=" << a.faces << "\n"
              << "source_faces=" << a.sourceFaces << "\n"
              << "non_triangle_source_faces=" << a.nonTriangleSourceFaces << "\n"
              << "non_planar_source_faces=" << a.nonPlanarSourceFaces << "\n"
              << "invalid_faces=" << a.invalidFaces << "\n"
              << "repeated_vertex_faces=" << a.repeatedVertexFaces << "\n"
              << "zero_area_faces=" << a.zeroAreaFaces << "\n"
              << "duplicate_faces=" << a.duplicateFaces << "\n"
              << "unique_edges=" << a.uniqueEdges << "\n"
              << "boundary_edges=" << a.boundaryEdges << "\n"
              << "non_manifold_edges=" << a.nonManifoldEdges << "\n"
              << "connected_components=" << a.connectedComponents << "\n"
              << "isolated_vertices=" << a.isolatedVertices << "\n"
              << "bad_vertex_links=" << a.badLinkVertices << "\n"
              << "closed_link_vertices=" << a.closedLinkVertices << "\n"
              << "boundary_link_vertices=" << a.boundaryLinkVertices << "\n"
              << "max_incident_faces_per_vertex=" << a.maxIncidentFaces << "\n"
              << "valid_triangle_mesh=" << (valid ? "true" : "false") << "\n"
              << "two_manifold_with_boundary_ok="
              << (twoManifoldWithBoundary ? "true" : "false") << "\n"
              << "closed_two_manifold_ok=" << (closedTwoManifold ? "true" : "false")
              << "\n";
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: " << argv[0] << " input.off [--repair output.off]\n";
        return 1;
    }

    std::string input = argv[1];
    std::string repairOutput;
    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            if (argc != 2) {
                std::cerr << "error=help_must_be_used_alone\n";
                return 2;
            }
            std::cout << "usage: " << argv[0]
                      << " input.off [--repair output.off]\n";
            return 0;
        }
        if (arg != "--repair") {
            std::cerr << "error=unknown_argument argument=" << arg << "\n"
                      << "usage: " << argv[0]
                      << " input.off [--repair output.off]\n";
            return 2;
        }
        if (!repairOutput.empty()) {
            std::cerr << "error=duplicate_argument argument=--repair\n";
            return 2;
        }
        if (i + 1 >= argc) {
            std::cerr << "error=missing_argument_value argument=--repair\n";
            return 2;
        }
        const std::string candidate = argv[++i];
        if (candidate.empty() || candidate.rfind("--", 0) == 0) {
            std::cerr << "error=invalid_argument_value argument=--repair\n";
            return 2;
        }
        repairOutput = candidate;
    }

    Mesh mesh;
    if (!loadOFF(input, mesh)) return 1;
    Audit audit = analyze(mesh);
    printAudit(audit);

    if (!repairOutput.empty()) {
        std::vector<char> keep = basicRepairKeepMask(mesh);
        const std::size_t kept =
            static_cast<std::size_t>(std::count(keep.begin(), keep.end(), 1));
        if (!writeOFF(repairOutput, mesh, keep)) {
            std::cerr << "repair_written=false\n";
            return 1;
        }
        std::cout << "repair_written=true\n"
                  << "repair_output=" << repairOutput << "\n"
                  << "repair_kept_faces=" << kept << "\n"
                  << "repair_dropped_faces=" << (mesh.faces.size() - kept) << "\n";
    }

    const bool ok = audit.invalidFaces == 0 && audit.repeatedVertexFaces == 0 &&
                    audit.zeroAreaFaces == 0 && audit.duplicateFaces == 0 &&
                    audit.nonTriangleSourceFaces == 0 &&
                    audit.nonManifoldEdges == 0 && audit.badLinkVertices == 0;
    return ok ? 0 : 2;
}
