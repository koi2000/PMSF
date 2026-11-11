#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

struct Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    double norm2() const { return x * x + y * y + z * z; }
    double axis(int a) const {
        if (a == 0) return x;
        if (a == 1) return y;
        return z;
    }
};

struct Mesh {
    std::vector<Vec3> vertices;
    std::vector<std::vector<int>> faces;
};

struct Edge {
    int a = 0, b = 0;
    bool operator<(const Edge& o) const {
        return a == o.a ? b < o.b : a < o.a;
    }
    bool operator==(const Edge& o) const { return a == o.a && b == o.b; }
};

struct Key3 {
    long long x = 0, y = 0, z = 0;
    bool operator==(const Key3& o) const { return x == o.x && y == o.y && z == o.z; }
};

struct Key3Hash {
    size_t operator()(const Key3& k) const {
        uint64_t x = static_cast<uint64_t>(k.x) * 0x9E3779B185EBCA87ull;
        uint64_t y = static_cast<uint64_t>(k.y) * 0xC2B2AE3D27D4EB4Full;
        uint64_t z = static_cast<uint64_t>(k.z) * 0x165667B19E3779F9ull;
        return static_cast<size_t>(x ^ (y << 1) ^ (z << 2));
    }
};

static Edge makeEdge(int a, int b) {
    if (a > b) std::swap(a, b);
    return {a, b};
}

static bool loadOFF(const std::string& path, Mesh& mesh) {
    std::ifstream fin(path);
    if (!fin) {
        std::cerr << "ERROR: cannot open " << path << "\n";
        return false;
    }
    std::string header;
    fin >> header;
    if (header != "OFF" && header != "COFF" && header != "NOFF") {
        std::cerr << "ERROR: unsupported OFF header " << header << "\n";
        return false;
    }
    int nv = 0, nf = 0, ne = 0;
    fin >> nv >> nf >> ne;
    if (nv < 0 || nf < 0) {
        std::cerr << "ERROR: invalid OFF counts\n";
        return false;
    }
    mesh.vertices.resize(static_cast<size_t>(nv));
    for (int i = 0; i < nv; ++i) {
        fin >> mesh.vertices[i].x >> mesh.vertices[i].y >> mesh.vertices[i].z;
        if (header == "NOFF") {
            double nx, ny, nz;
            fin >> nx >> ny >> nz;
        }
    if (header == "COFF") {
            int r, g, b, a = 255;
            fin >> r >> g >> b;
            if (fin.peek() != '\n' && fin.peek() != '\r') fin >> a;
        }
    }
    mesh.faces.resize(static_cast<size_t>(nf));
    std::string line;
    std::getline(fin, line);
    for (int i = 0; i < nf; ++i) {
        do {
            if (!std::getline(fin, line)) return false;
        } while (line.find_first_not_of(" \t\r\n") == std::string::npos || line[0] == '#');
        std::stringstream ss(line);
        int n = 0;
        ss >> n;
        if (!ss || n < 0) return false;
        mesh.faces[i].resize(static_cast<size_t>(n));
        for (int j = 0; j < n; ++j) {
            ss >> mesh.faces[i][j];
            if (!ss) return false;
        }
    }
    return true;
}

static double triArea2(const Vec3& a, const Vec3& b, const Vec3& c) {
    Vec3 ab = b - a;
    Vec3 ac = c - a;
    const double cx = ab.y * ac.z - ab.z * ac.y;
    const double cy = ab.z * ac.x - ab.x * ac.z;
    const double cz = ab.x * ac.y - ab.y * ac.x;
    return cx * cx + cy * cy + cz * cz;
}

struct Stats {
    int vertices = 0;
    int faces = 0;
    int triangles = 0;
    int invalidIndices = 0;
    int degenerateFaces = 0;
    int repeatedVertexFaces = 0;
    int zeroLengthFaceEdges = 0;
    int duplicateFaces = 0;
    int uniqueEdges = 0;
    int boundaryEdges = 0;
    int nonManifoldEdges = 0;
    int connectedComponents = 0;
    int isolatedVertices = 0;
    std::map<int, int> faceDegreeHist;
};

struct NearestStats {
    double rmse = 0.0;
    double maxDist = 0.0;
    int compared = 0;
    int withinTol = 0;
    int overTol = 0;
};

struct KdNode {
    int point = -1;
    int axis = 0;
    std::unique_ptr<KdNode> left;
    std::unique_ptr<KdNode> right;
};

class KdTree {
public:
    explicit KdTree(const std::vector<Vec3>& pts) : pts_(pts) {
        std::vector<int> ids(pts_.size());
        std::iota(ids.begin(), ids.end(), 0);
        root_ = build(ids, 0, static_cast<int>(ids.size()), 0);
    }

    double nearestDist2(const Vec3& q) const {
        double best = std::numeric_limits<double>::infinity();
        nearest(root_.get(), q, best);
        return best;
    }

private:
    const std::vector<Vec3>& pts_;
    std::unique_ptr<KdNode> root_;

    std::unique_ptr<KdNode> build(std::vector<int>& ids, int lo, int hi, int depth) {
        if (lo >= hi) return nullptr;
        const int axis = depth % 3;
        const int mid = lo + (hi - lo) / 2;
        std::nth_element(ids.begin() + lo, ids.begin() + mid, ids.begin() + hi,
                         [&](int a, int b) {
                             return pts_[static_cast<size_t>(a)].axis(axis) <
                                    pts_[static_cast<size_t>(b)].axis(axis);
                         });
        auto node = std::make_unique<KdNode>();
        node->point = ids[static_cast<size_t>(mid)];
        node->axis = axis;
        node->left = build(ids, lo, mid, depth + 1);
        node->right = build(ids, mid + 1, hi, depth + 1);
        return node;
    }

    void nearest(const KdNode* node, const Vec3& q, double& best) const {
        if (!node) return;
        const Vec3& p = pts_[static_cast<size_t>(node->point)];
        best = std::min(best, (p - q).norm2());
        const double diff = q.axis(node->axis) - p.axis(node->axis);
        const KdNode* first = diff < 0.0 ? node->left.get() : node->right.get();
        const KdNode* second = diff < 0.0 ? node->right.get() : node->left.get();
        nearest(first, q, best);
        if (diff * diff <= best) nearest(second, q, best);
    }
};

static NearestStats directedNearestStats(const Mesh& src, const Mesh& dst, double tol) {
    NearestStats s;
    s.compared = static_cast<int>(src.vertices.size());
    if (src.vertices.empty() || dst.vertices.empty()) {
        s.overTol = s.compared;
        s.maxDist = std::numeric_limits<double>::infinity();
        s.rmse = std::numeric_limits<double>::infinity();
        return s;
    }
    KdTree tree(dst.vertices);
    long double sum = 0.0;
    const double tol2 = tol * tol;
    for (const Vec3& p : src.vertices) {
        const double d2 = tree.nearestDist2(p);
        sum += d2;
        const double d = std::sqrt(d2);
        s.maxDist = std::max(s.maxDist, d);
        if (d2 <= tol2) ++s.withinTol;
        else ++s.overTol;
    }
    s.rmse = std::sqrt(static_cast<double>(sum / std::max(1, s.compared)));
    return s;
}

static std::vector<int> canonicalFace(std::vector<int> f) {
    std::sort(f.begin(), f.end());
    return f;
}

static std::map<Edge, int> edgeUseCounts(const Mesh& m) {
    std::map<Edge, int> counts;
    for (const auto& f : m.faces) {
        if (f.size() < 2) continue;
        for (size_t i = 0; i < f.size(); ++i) {
            int a = f[i];
            int b = f[(i + 1) % f.size()];
            if (a >= 0 && b >= 0 && a < (int)m.vertices.size() && b < (int)m.vertices.size() && a != b) {
                counts[makeEdge(a, b)]++;
            }
        }
    }
    return counts;
}

static Stats computeStats(const Mesh& m) {
    Stats s;
    s.vertices = static_cast<int>(m.vertices.size());
    s.faces = static_cast<int>(m.faces.size());
    std::map<std::vector<int>, int> faceCounts;
    std::vector<std::vector<int>> adj(m.vertices.size());

    for (const auto& f : m.faces) {
        s.faceDegreeHist[static_cast<int>(f.size())]++;
        if (f.size() >= 3) s.triangles += static_cast<int>(f.size()) - 2;
        bool invalid = false;
        for (int v : f) {
            if (v < 0 || v >= (int)m.vertices.size()) invalid = true;
        }
        if (invalid) {
            s.invalidIndices++;
            continue;
        }
        if (f.size() < 3) {
            s.degenerateFaces++;
        } else {
            bool deg = false;
            std::set<int> faceVerts;
            for (int v : f) {
                if (!faceVerts.insert(v).second) deg = true;
            }
            if (deg) s.repeatedVertexFaces++;
            for (size_t i = 1; i + 1 < f.size(); ++i) {
                if (triArea2(m.vertices[f[0]], m.vertices[f[i]], m.vertices[f[i + 1]]) < 1e-30) {
                    deg = true;
                    break;
                }
            }
            if (deg) s.degenerateFaces++;
        }
        faceCounts[canonicalFace(f)]++;
        for (size_t i = 0; i < f.size(); ++i) {
            int a = f[i];
            int b = f[(i + 1) % f.size()];
            if (a != b) {
                adj[a].push_back(b);
                adj[b].push_back(a);
            } else {
                s.zeroLengthFaceEdges++;
            }
        }
    }
    for (const auto& [face, count] : faceCounts) {
        if (count > 1) s.duplicateFaces += count - 1;
    }

    auto edges = edgeUseCounts(m);
    s.uniqueEdges = static_cast<int>(edges.size());
    for (const auto& [e, count] : edges) {
        if (count == 1) s.boundaryEdges++;
        if (count > 2) s.nonManifoldEdges++;
    }

    std::vector<char> seen(m.vertices.size(), 0);
    for (size_t i = 0; i < m.vertices.size(); ++i) {
        if (adj[i].empty()) {
            s.isolatedVertices++;
            continue;
        }
        if (seen[i]) continue;
        s.connectedComponents++;
        std::queue<int> q;
        q.push(static_cast<int>(i));
        seen[i] = 1;
        while (!q.empty()) {
            int v = q.front();
            q.pop();
            for (int nb : adj[v]) {
                if (!seen[nb]) {
                    seen[nb] = 1;
                    q.push(nb);
                }
            }
        }
    }
    return s;
}

struct Compare {
    Stats a, b;
    int sameIndexFaces = 0;
    int sameUnorderedFaces = 0;
    int commonEdges = 0;
    int edgesOnlyA = 0;
    int edgesOnlyB = 0;
    double indexVertexRmse = 0.0;
    double indexVertexMax = 0.0;
    int indexVerticesCompared = 0;
    int coordMatchedVertices = 0;
    double coordMatchMaxDist = 0.0;
    int remappedSameUnorderedFaces = 0;
    int remappedCommonEdges = 0;
    int remappedEdgesOnlyRef = 0;
    int remappedEdgesOnlyTest = 0;
    NearestStats refToTestNearest;
    NearestStats testToRefNearest;
};

static Key3 quantKey(const Vec3& p, double tol) {
    return {llround(p.x / tol), llround(p.y / tol), llround(p.z / tol)};
}

static std::map<Edge, int> remappedEdgeUseCounts(const Mesh& m, const std::vector<int>& remap) {
    std::map<Edge, int> counts;
    for (const auto& f : m.faces) {
        if (f.size() < 2) continue;
        for (size_t i = 0; i < f.size(); ++i) {
            int ta = f[i];
            int tb = f[(i + 1) % f.size()];
            if (ta < 0 || tb < 0 || ta >= (int)remap.size() || tb >= (int)remap.size()) continue;
            int a = remap[ta];
            int b = remap[tb];
            if (a >= 0 && b >= 0 && a != b) counts[makeEdge(a, b)]++;
        }
    }
    return counts;
}

static int edgeDiffScore(const std::map<Edge, int>& refEdges,
                         const Mesh& test,
                         const std::vector<int>& testToRef) {
    auto testEdges = remappedEdgeUseCounts(test, testToRef);
    int score = 0;
    for (const auto& [e, count] : refEdges) {
        if (!testEdges.count(e)) ++score;
    }
    for (const auto& [e, count] : testEdges) {
        if (!refEdges.count(e)) ++score;
    }
    return score;
}

static std::set<int> edgeDiffRefs(const std::map<Edge, int>& refEdges,
                                  const Mesh& test,
                                  const std::vector<int>& testToRef) {
    auto testEdges = remappedEdgeUseCounts(test, testToRef);
    std::set<int> refs;
    for (const auto& [e, count] : refEdges) {
        if (!testEdges.count(e)) {
            refs.insert(e.a);
            refs.insert(e.b);
        }
    }
    for (const auto& [e, count] : testEdges) {
        if (!refEdges.count(e)) {
            refs.insert(e.a);
            refs.insert(e.b);
        }
    }
    return refs;
}

static void improveRemapByLocalTopology(const Mesh& ref,
                                        const Mesh& test,
                                        const std::map<Edge, int>& refEdges,
                                        double coordTol,
                                        std::vector<int>& testToRef) {
    auto dist = [&](int ti, int ri) -> double {
        return std::sqrt((test.vertices[static_cast<size_t>(ti)] -
                          ref.vertices[static_cast<size_t>(ri)]).norm2());
    };
    const double maxSwapDist = coordTol * 1.75;

    int currentScore = edgeDiffScore(refEdges, test, testToRef);
    if (currentScore > 2000) return;
    for (int iter = 0; iter < 8 && currentScore > 0; ++iter) {
        std::vector<int> refToTest(ref.vertices.size(), -1);
        for (int ti = 0; ti < (int)testToRef.size(); ++ti) {
            int ri = testToRef[static_cast<size_t>(ti)];
            if (ri >= 0 && ri < (int)refToTest.size()) {
                refToTest[static_cast<size_t>(ri)] = ti;
            }
        }

        std::vector<int> candidateTests;
        for (int ri : edgeDiffRefs(refEdges, test, testToRef)) {
            if (ri >= 0 && ri < (int)refToTest.size()) {
                int ti = refToTest[static_cast<size_t>(ri)];
                if (ti >= 0) candidateTests.push_back(ti);
            }
        }
        std::sort(candidateTests.begin(), candidateTests.end());
        candidateTests.erase(std::unique(candidateTests.begin(), candidateTests.end()),
                             candidateTests.end());
        if (candidateTests.size() > 512) break;

        int bestA = -1;
        int bestB = -1;
        int bestScore = currentScore;
        for (size_t ia = 0; ia < candidateTests.size(); ++ia) {
            int ta = candidateTests[ia];
            int ra = testToRef[static_cast<size_t>(ta)];
            if (ra < 0) continue;
            for (size_t ib = ia + 1; ib < candidateTests.size(); ++ib) {
                int tb = candidateTests[ib];
                int rb = testToRef[static_cast<size_t>(tb)];
                if (rb < 0) continue;
                if (dist(ta, rb) > maxSwapDist || dist(tb, ra) > maxSwapDist) continue;

                std::swap(testToRef[static_cast<size_t>(ta)],
                          testToRef[static_cast<size_t>(tb)]);
                int score = edgeDiffScore(refEdges, test, testToRef);
                std::swap(testToRef[static_cast<size_t>(ta)],
                          testToRef[static_cast<size_t>(tb)]);

                if (score < bestScore) {
                    bestScore = score;
                    bestA = ta;
                    bestB = tb;
                    if (bestScore == 0) break;
                }
            }
            if (bestScore == 0) break;
        }

        if (bestA < 0 || bestB < 0 || bestScore >= currentScore) break;
        std::swap(testToRef[static_cast<size_t>(bestA)],
                  testToRef[static_cast<size_t>(bestB)]);
        currentScore = bestScore;
    }
}

static Compare compareMeshes(const Mesh& a, const Mesh& b, double coordTol) {
    Compare c;
    c.a = computeStats(a);
    c.b = computeStats(b);

    const int nf = std::min<int>(a.faces.size(), b.faces.size());
    for (int i = 0; i < nf; ++i) {
        if (a.faces[i] == b.faces[i]) c.sameIndexFaces++;
    }
    std::map<std::vector<int>, int> fa, fb;
    for (const auto& f : a.faces) fa[canonicalFace(f)]++;
    for (const auto& f : b.faces) fb[canonicalFace(f)]++;
    for (const auto& [f, ca] : fa) {
        auto it = fb.find(f);
        if (it != fb.end()) c.sameUnorderedFaces += std::min(ca, it->second);
    }

    auto ea = edgeUseCounts(a);
    auto eb = edgeUseCounts(b);
    for (const auto& [e, count] : ea) {
        if (eb.count(e)) c.commonEdges++;
        else c.edgesOnlyA++;
    }
    for (const auto& [e, count] : eb) {
        if (!ea.count(e)) c.edgesOnlyB++;
    }

    const int nv = std::min<int>(a.vertices.size(), b.vertices.size());
    c.indexVerticesCompared = nv;
    long double sum = 0.0;
    for (int i = 0; i < nv; ++i) {
        double d2 = (a.vertices[i] - b.vertices[i]).norm2();
        sum += d2;
        c.indexVertexMax = std::max(c.indexVertexMax, std::sqrt(d2));
    }
    c.indexVertexRmse = nv > 0 ? std::sqrt(static_cast<double>(sum / nv)) : 0.0;
    c.refToTestNearest = directedNearestStats(a, b, coordTol);
    c.testToRefNearest = directedNearestStats(b, a, coordTol);

    std::unordered_map<Key3, std::vector<int>, Key3Hash> refByCoord;
    refByCoord.reserve(a.vertices.size() * 2 + 1);
    for (int i = 0; i < (int)a.vertices.size(); ++i) {
        refByCoord[quantKey(a.vertices[static_cast<size_t>(i)], coordTol)].push_back(i);
    }
    std::vector<char> refUsed(a.vertices.size(), 0);
    std::vector<int> testToRef(b.vertices.size(), -1);
    for (int i = 0; i < (int)b.vertices.size(); ++i) {
        Key3 key = quantKey(b.vertices[static_cast<size_t>(i)], coordTol);
        int best = -1;
        double bestD2 = std::numeric_limits<double>::max();
        for (long long dx = -1; dx <= 1; ++dx) {
            for (long long dy = -1; dy <= 1; ++dy) {
                for (long long dz = -1; dz <= 1; ++dz) {
                    Key3 probe{key.x + dx, key.y + dy, key.z + dz};
                    auto it = refByCoord.find(probe);
                    if (it == refByCoord.end()) continue;
                    for (int refIdx : it->second) {
                        if (refUsed[static_cast<size_t>(refIdx)]) continue;
                        double d2 = (a.vertices[static_cast<size_t>(refIdx)] -
                                     b.vertices[static_cast<size_t>(i)]).norm2();
                        if (d2 < bestD2) {
                            bestD2 = d2;
                            best = refIdx;
                        }
                    }
                }
            }
        }
        if (best >= 0 && std::sqrt(bestD2) <= coordTol * 1.75) {
            testToRef[static_cast<size_t>(i)] = best;
            refUsed[static_cast<size_t>(best)] = 1;
        }
    }

    improveRemapByLocalTopology(a, b, ea, coordTol, testToRef);
    c.coordMatchedVertices = 0;
    c.coordMatchMaxDist = 0.0;
    for (int i = 0; i < (int)testToRef.size(); ++i) {
        int ri = testToRef[static_cast<size_t>(i)];
        if (ri < 0) continue;
        ++c.coordMatchedVertices;
        c.coordMatchMaxDist = std::max(
            c.coordMatchMaxDist,
            std::sqrt((a.vertices[static_cast<size_t>(ri)] -
                       b.vertices[static_cast<size_t>(i)]).norm2()));
    }

    std::map<std::vector<int>, int> remapFaces;
    for (const auto& f : b.faces) {
        std::vector<int> rf;
        rf.reserve(f.size());
        bool ok = true;
        for (int v : f) {
            if (v < 0 || v >= (int)testToRef.size() || testToRef[static_cast<size_t>(v)] < 0) {
                ok = false;
                break;
            }
            rf.push_back(testToRef[static_cast<size_t>(v)]);
        }
        if (ok) remapFaces[canonicalFace(rf)]++;
    }
    for (const auto& [f, ca] : fa) {
        auto it = remapFaces.find(f);
        if (it != remapFaces.end()) c.remappedSameUnorderedFaces += std::min(ca, it->second);
    }

    auto reb = remappedEdgeUseCounts(b, testToRef);
    for (const auto& [e, count] : ea) {
        if (reb.count(e)) c.remappedCommonEdges++;
        else c.remappedEdgesOnlyRef++;
    }
    for (const auto& [e, count] : reb) {
        if (!ea.count(e)) c.remappedEdgesOnlyTest++;
    }
    return c;
}

static void printStats(const char* name, const Stats& s) {
    std::cout << name << "_vertices=" << s.vertices << "\n";
    std::cout << name << "_faces=" << s.faces << "\n";
    std::cout << name << "_triangles=" << s.triangles << "\n";
    std::cout << name << "_unique_edges=" << s.uniqueEdges << "\n";
    std::cout << name << "_boundary_edges=" << s.boundaryEdges << "\n";
    std::cout << name << "_nonmanifold_edges=" << s.nonManifoldEdges << "\n";
    std::cout << name << "_components=" << s.connectedComponents << "\n";
    std::cout << name << "_isolated_vertices=" << s.isolatedVertices << "\n";
    std::cout << name << "_degenerate_faces=" << s.degenerateFaces << "\n";
    std::cout << name << "_repeated_vertex_faces=" << s.repeatedVertexFaces << "\n";
    std::cout << name << "_zero_length_face_edges=" << s.zeroLengthFaceEdges << "\n";
    std::cout << name << "_duplicate_faces=" << s.duplicateFaces << "\n";
    std::cout << name << "_invalid_indices=" << s.invalidIndices << "\n";
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <reference.off> <test.off> [--tol value]\n";
        return 2;
    }
    double coordTol = 1e-7;
    for (int i = 3; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "--tol") coordTol = std::stod(argv[++i]);
    }
    Mesh a, b;
    if (!loadOFF(argv[1], a) || !loadOFF(argv[2], b)) return 3;
    Compare c = compareMeshes(a, b, coordTol);
    std::cout << std::setprecision(12);
    printStats("ref", c.a);
    printStats("test", c.b);
    std::cout << "same_index_faces=" << c.sameIndexFaces << "\n";
    std::cout << "same_unordered_faces=" << c.sameUnorderedFaces << "\n";
    std::cout << "common_edges=" << c.commonEdges << "\n";
    std::cout << "edges_only_ref=" << c.edgesOnlyA << "\n";
    std::cout << "edges_only_test=" << c.edgesOnlyB << "\n";
    std::cout << "index_vertex_rmse=" << c.indexVertexRmse << "\n";
    std::cout << "index_vertex_max=" << c.indexVertexMax << "\n";
    std::cout << "coord_tol=" << coordTol << "\n";
    std::cout << "coord_matched_vertices=" << c.coordMatchedVertices << "\n";
    std::cout << "coord_match_max_dist=" << c.coordMatchMaxDist << "\n";
    std::cout << "remapped_same_unordered_faces=" << c.remappedSameUnorderedFaces << "\n";
    std::cout << "remapped_common_edges=" << c.remappedCommonEdges << "\n";
    std::cout << "remapped_edges_only_ref=" << c.remappedEdgesOnlyRef << "\n";
    std::cout << "remapped_edges_only_test=" << c.remappedEdgesOnlyTest << "\n";
    std::cout << "nn_ref_to_test_rmse=" << c.refToTestNearest.rmse << "\n";
    std::cout << "nn_ref_to_test_max=" << c.refToTestNearest.maxDist << "\n";
    std::cout << "nn_ref_to_test_over_tol=" << c.refToTestNearest.overTol << "\n";
    std::cout << "nn_test_to_ref_rmse=" << c.testToRefNearest.rmse << "\n";
    std::cout << "nn_test_to_ref_max=" << c.testToRefNearest.maxDist << "\n";
    std::cout << "nn_test_to_ref_over_tol=" << c.testToRefNearest.overTol << "\n";

    const bool countsOk = c.a.vertices == c.b.vertices && c.a.faces == c.b.faces;
    const bool refValid = c.a.invalidIndices == 0 && c.a.degenerateFaces == 0 &&
                          c.a.duplicateFaces == 0 && c.a.zeroLengthFaceEdges == 0;
    const bool testValid = c.b.invalidIndices == 0 && c.b.degenerateFaces == 0 &&
                           c.b.duplicateFaces == 0 && c.b.zeroLengthFaceEdges == 0;
    const bool manifoldProfileOk = c.a.nonManifoldEdges == c.b.nonManifoldEdges &&
                                   c.a.boundaryEdges == c.b.boundaryEdges &&
                                   c.a.connectedComponents == c.b.connectedComponents &&
                                   c.a.isolatedVertices == c.b.isolatedVertices;
    const bool coordinateRemapComplete = c.coordMatchedVertices == c.a.vertices &&
                                         c.coordMatchedVertices == c.b.vertices;
    const bool connectivityOk = coordinateRemapComplete &&
                                c.remappedEdgesOnlyRef == 0 &&
                                c.remappedEdgesOnlyTest == 0 &&
                                c.remappedSameUnorderedFaces == c.a.faces;
    const bool indexConnectivityOk = c.sameIndexFaces == c.a.faces &&
                                     c.edgesOnlyA == 0 &&
                                     c.edgesOnlyB == 0;
    const bool topologyOk = testValid && manifoldProfileOk &&
                            (connectivityOk || indexConnectivityOk);
    const bool indexGeomOk = c.indexVerticesCompared == c.a.vertices &&
                             c.indexVertexMax <= coordTol;
    const bool nearestGeomOk = c.refToTestNearest.overTol == 0 &&
                               c.testToRefNearest.overTol == 0;
    std::cout << "counts_ok=" << (countsOk ? "true" : "false") << "\n";
    std::cout << "ref_valid=" << (refValid ? "true" : "false") << "\n";
    std::cout << "test_valid=" << (testValid ? "true" : "false") << "\n";
    std::cout << "manifold_profile_ok=" << (manifoldProfileOk ? "true" : "false") << "\n";
    std::cout << "coordinate_remap_complete=" << (coordinateRemapComplete ? "true" : "false") << "\n";
    std::cout << "index_connectivity_ok=" << (indexConnectivityOk ? "true" : "false") << "\n";
    std::cout << "connectivity_ok=" << (connectivityOk ? "true" : "false") << "\n";
    std::cout << "topology_ok=" << (topologyOk ? "true" : "false") << "\n";
    std::cout << "index_geometry_ok=" << (indexGeomOk ? "true" : "false") << "\n";
    std::cout << "nearest_geometry_ok=" << (nearestGeomOk ? "true" : "false") << "\n";
    return (countsOk && topologyOk && nearestGeomOk) ? 0 : 1;
}
