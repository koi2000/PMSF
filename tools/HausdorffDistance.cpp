#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }
    double length() const { return std::sqrt(x * x + y * y + z * z); }
};

struct Triangle {
    Vec3 a, b, c;
};

struct SimpleMesh {
    std::vector<Vec3> vertices;
    std::vector<std::vector<int>> faces;
};

// Ericson "Real-Time Collision Detection" - closest point on triangle
static Vec3 closestPointOnTriangle(const Vec3& p, const Triangle& tri) {
    Vec3 ab = tri.b - tri.a;
    Vec3 ac = tri.c - tri.a;
    Vec3 ap = p - tri.a;

    double d1 = ab.dot(ap);
    double d2 = ac.dot(ap);
    if (d1 <= 0.0 && d2 <= 0.0) return tri.a;

    Vec3 bp = p - tri.b;
    double d3 = ab.dot(bp);
    double d4 = ac.dot(bp);
    if (d3 >= 0.0 && d4 <= d3) return tri.b;

    double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        double v = d1 / (d1 - d3);
        return tri.a + ab * v;
    }

    Vec3 cp = p - tri.c;
    double d5 = ab.dot(cp);
    double d6 = ac.dot(cp);
    if (d6 >= 0.0 && d5 <= d6) return tri.c;

    double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        double w = d2 / (d2 - d6);
        return tri.a + ac * w;
    }

    double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
        double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return tri.b + (tri.c - tri.b) * w;
    }

    double denom = 1.0 / (va + vb + vc);
    double v = vb * denom;
    double w = vc * denom;
    return tri.a + ab * v + ac * w;
}

static bool loadOFF(const std::string& path, SimpleMesh& mesh) {
    std::ifstream fin(path);
    if (!fin.is_open()) {
        std::cerr << "错误: 无法打开文件 " << path << std::endl;
        return false;
    }

    std::string header;
    fin >> header;
    if (header != "OFF" && header != "COFF" && header != "NOFF") {
        std::cerr << "错误: 不是有效的 OFF 文件 (头部: " << header << ")" << std::endl;
        return false;
    }

    int nv, nf, ne;
    fin >> nv >> nf >> ne;

    mesh.vertices.resize(nv);
    std::string line;
    std::getline(fin, line);
    for (int i = 0; i < nv; i++) {
        do {
            if (!std::getline(fin, line)) {
                std::cerr << "警告: 读取文件时发生错误，部分数据可能不完整" << std::endl;
                return false;
            }
        } while (line.find_first_not_of(" \t\r\n") == std::string::npos || line[0] == '#');
        std::stringstream ss(line);
        ss >> mesh.vertices[i].x >> mesh.vertices[i].y >> mesh.vertices[i].z;
        if (!ss) {
            std::cerr << "警告: 读取文件时发生错误，部分数据可能不完整" << std::endl;
            return false;
        }
    }

    mesh.faces.resize(nf);
    for (int i = 0; i < nf; i++) {
        do {
            if (!std::getline(fin, line)) {
                std::cerr << "警告: 读取文件时发生错误，部分数据可能不完整" << std::endl;
                return false;
            }
        } while (line.find_first_not_of(" \t\r\n") == std::string::npos || line[0] == '#');
        std::stringstream ss(line);
        int n;
        ss >> n;
        if (!ss || n < 0) {
            std::cerr << "警告: 读取文件时发生错误，部分数据可能不完整" << std::endl;
            return false;
        }
        mesh.faces[i].resize(n);
        for (int j = 0; j < n; j++) {
            ss >> mesh.faces[i][j];
            if (!ss) {
                std::cerr << "警告: 读取文件时发生错误，部分数据可能不完整" << std::endl;
                return false;
            }
        }
    }

    return true;
}

static std::vector<Triangle> extractTriangles(const SimpleMesh& mesh) {
    std::vector<Triangle> tris;
    for (const auto& face : mesh.faces) {
        if (face.size() < 3) continue;
        for (size_t i = 1; i + 1 < face.size(); i++) {
            Triangle t;
            t.a = mesh.vertices[face[0]];
            t.b = mesh.vertices[face[i]];
            t.c = mesh.vertices[face[i + 1]];
            tris.push_back(t);
        }
    }
    return tris;
}

static double pointToMeshDistance(const Vec3& p, const std::vector<Triangle>& tris) {
    double minDist = std::numeric_limits<double>::max();
    for (const auto& tri : tris) {
        Vec3 closest = closestPointOnTriangle(p, tri);
        double dist = (p - closest).length();
        if (dist < minDist) minDist = dist;
    }
    return minDist;
}


struct Aabb {
    Vec3 min{1e300, 1e300, 1e300};
    Vec3 max{-1e300, -1e300, -1e300};
    void expand(const Vec3& p) {
        min.x = std::min(min.x, p.x);
        min.y = std::min(min.y, p.y);
        min.z = std::min(min.z, p.z);
        max.x = std::max(max.x, p.x);
        max.y = std::max(max.y, p.y);
        max.z = std::max(max.z, p.z);
    }
};

static double pointAabbDist2(const Vec3& p, const Aabb& box) {
    const double dx = std::max({box.min.x - p.x, 0.0, p.x - box.max.x});
    const double dy = std::max({box.min.y - p.y, 0.0, p.y - box.max.y});
    const double dz = std::max({box.min.z - p.z, 0.0, p.z - box.max.z});
    return dx * dx + dy * dy + dz * dz;
}

/// A small static AABB tree.  The original tool was O(V*F); on armadillo
/// (172k V, 345k F) that is >60 billion closest-point tests and does not
/// finish.  The BVH makes each query O(log F).
class TriangleBvh {
  public:
    explicit TriangleBvh(const std::vector<Triangle>& tris) : tris_(tris) {
        order_.resize(tris_.size());
        std::iota(order_.begin(), order_.end(), 0);
        if (!tris_.empty()) {
            nodes_.reserve(tris_.size() * 2);
            root_ = build(0, static_cast<int>(tris_.size()));
        }
    }

    double distance(const Vec3& p) const {
        if (tris_.empty()) return std::numeric_limits<double>::infinity();
        double best2 = std::numeric_limits<double>::infinity();
        std::vector<int> stack;
        stack.push_back(root_);
        while (!stack.empty()) {
            const int nodeIndex = stack.back();
            stack.pop_back();
            const BvhNode& node = nodes_[nodeIndex];
            if (pointAabbDist2(p, node.box) >= best2) continue;
            if (node.left < 0) {
                for (int i = node.start; i < node.start + node.count; ++i) {
                    const Triangle& tri = tris_[order_[i]];
                    const Vec3 closest = closestPointOnTriangle(p, tri);
                    const Vec3 d = p - closest;
                    const double dist2 = d.dot(d);
                    if (dist2 < best2) best2 = dist2;
                }
            } else {
                const double dl = pointAabbDist2(p, nodes_[node.left].box);
                const double dr = pointAabbDist2(p, nodes_[node.right].box);
                // Push the farther child first so the nearer one is processed first.
                if (dl < dr) {
                    stack.push_back(node.right);
                    stack.push_back(node.left);
                } else {
                    stack.push_back(node.left);
                    stack.push_back(node.right);
                }
            }
        }
        return std::sqrt(best2);
    }

  private:
    struct BvhNode {
        Aabb box;
        int start = 0;
        int count = 0;
        int left = -1;
        int right = -1;
    };

    Aabb boxForRange(int start, int count) const {
        Aabb box;
        for (int i = start; i < start + count; ++i) {
            const Triangle& tri = tris_[order_[i]];
            box.expand(tri.a);
            box.expand(tri.b);
            box.expand(tri.c);
        }
        return box;
    }

    int build(int start, int count) {
        const int index = static_cast<int>(nodes_.size());
        nodes_.push_back(BvhNode{boxForRange(start, count), start, count, -1, -1});
        if (count <= 8) return index;

        Aabb centroidBox;
        // NOTE: `centroid` takes a *triangle id* (the value stored in order_).
        // std::nth_element's comparator receives those element values directly,
        // so it must index tris_ by the id, not by the position in order_
        // (the old `tris_[order_[i]]` double-indirection made the comparator
        // inconsistent and could overrun the range in __unguarded_partition).
        const auto centroid = [&](int triangleId) {
            const Triangle& tri = tris_[triangleId];
            return Vec3{(tri.a.x + tri.b.x + tri.c.x) / 3.0,
                        (tri.a.y + tri.b.y + tri.c.y) / 3.0,
                        (tri.a.z + tri.b.z + tri.c.z) / 3.0};
        };
        for (int i = start; i < start + count; ++i)
            centroidBox.expand(centroid(order_[i]));
        const double ex = centroidBox.max.x - centroidBox.min.x;
        const double ey = centroidBox.max.y - centroidBox.min.y;
        const double ez = centroidBox.max.z - centroidBox.min.z;
        int axis = (ex > ey) ? (ex > ez ? 0 : 2) : (ey > ez ? 1 : 2);
        const int mid = count / 2;
        std::nth_element(order_.begin() + start, order_.begin() + start + mid,
                         order_.begin() + start + count, [&](int lhs, int rhs) {
                             const Vec3 a = centroid(lhs);
                             const Vec3 b = centroid(rhs);
                             return axis == 0 ? a.x < b.x : (axis == 1 ? a.y < b.y : a.z < b.z);
                         });
        const int left = build(start, mid);
        const int right = build(start + mid, count - mid);
        nodes_[index].left = left;
        nodes_[index].right = right;
        return index;
    }

    const std::vector<Triangle>& tris_;
    std::vector<int> order_;
    std::vector<BvhNode> nodes_;
    int root_ = -1;
};

struct HausdorffResult {
    double oneSidedAB;
    double oneSidedBA;
    double symmetric;
    double meanAB;
    double meanBA;
    double rmsAB;
    double rmsBA;
};

static HausdorffResult computeHausdorff(const SimpleMesh& meshA,
                                          const SimpleMesh& meshB,
                                          bool bruteForce = false,
                                          int sampleVertices = 0) {
    HausdorffResult res{};

    auto trisA = extractTriangles(meshA);
    auto trisB = extractTriangles(meshB);

    std::cerr << "  Mesh A: " << meshA.vertices.size() << " vertices, "
              << trisA.size() << " triangles" << std::endl;
    std::cerr << "  Mesh B: " << meshB.vertices.size() << " vertices, "
              << trisB.size() << " triangles" << std::endl;
    std::cerr << "  acceleration: " << (bruteForce ? "brute-force" : "BVH") << std::endl;

    std::unique_ptr<TriangleBvh> bvhA;
    std::unique_ptr<TriangleBvh> bvhB;
    if (!bruteForce) {
        bvhA = std::make_unique<TriangleBvh>(trisA);
        bvhB = std::make_unique<TriangleBvh>(trisB);
    }
    const auto distanceToA = [&](const Vec3& p) {
        return bruteForce ? pointToMeshDistance(p, trisA) : bvhA->distance(p);
    };
    const auto distanceToB = [&](const Vec3& p) {
        return bruteForce ? pointToMeshDistance(p, trisB) : bvhB->distance(p);
    };

    const int nA = static_cast<int>(meshA.vertices.size());
    const int nB = static_cast<int>(meshB.vertices.size());
    const int strideA = (sampleVertices > 0 && nA > sampleVertices)
                            ? std::max(1, nA / sampleVertices) : 1;
    const int strideB = (sampleVertices > 0 && nB > sampleVertices)
                            ? std::max(1, nB / sampleVertices) : 1;
    std::cerr << "  vertex sample: " << (sampleVertices > 0 ? std::to_string(sampleVertices) : std::string("all"))
              << " (stride A=" << strideA << ", B=" << strideB << ")" << std::endl;

    // A -> B
    double maxAB = 0, sumAB = 0, sumSqAB = 0;
    int countA = 0;
    for (int i = 0; i < nA; i += strideA) {
        double d = distanceToB(meshA.vertices[i]);
        maxAB = std::max(maxAB, d);
        sumAB += d;
        sumSqAB += d * d;
        ++countA;
    }

    // B -> A
    double maxBA = 0, sumBA = 0, sumSqBA = 0;
    int countB = 0;
    for (int i = 0; i < nB; i += strideB) {
        double d = distanceToA(meshB.vertices[i]);
        maxBA = std::max(maxBA, d);
        sumBA += d;
        sumSqBA += d * d;
        ++countB;
    }

    res.oneSidedAB = maxAB;
    res.oneSidedBA = maxBA;
    res.symmetric = std::max(maxAB, maxBA);
    res.meanAB = countA > 0 ? sumAB / countA : 0;
    res.meanBA = countB > 0 ? sumBA / countB : 0;
    res.rmsAB = countA > 0 ? std::sqrt(sumSqAB / countA) : 0;
    res.rmsBA = countB > 0 ? std::sqrt(sumSqBA / countB) : 0;

    return res;
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Hausdorff distance calculator for OFF meshes" << std::endl;
        std::cerr << "Usage: " << argv[0] << " <meshA.off> <meshB.off> [--json] [--brute] [--sample N]" << std::endl;
        std::cerr << std::endl;
        std::cerr << "Outputs:" << std::endl;
        std::cerr << "  Hausdorff(A->B)  : max distance from A vertices to B surface" << std::endl;
        std::cerr << "  Hausdorff(B->A)  : max distance from B vertices to A surface" << std::endl;
        std::cerr << "  Symmetric        : max of the above two" << std::endl;
        std::cerr << "  Mean / RMS       : average and root-mean-square distances" << std::endl;
        return 1;
    }

    std::string pathA = argv[1];
    std::string pathB = argv[2];
    bool jsonOutput = false;
    bool bruteForce = false;
    int sampleVertices = 0;
    for (int i = 3; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--json") {
            jsonOutput = true;
        } else if (arg == "--brute") {
            bruteForce = true;
        } else if (arg == "--sample" && i + 1 < argc) {
            sampleVertices = std::max(0, std::atoi(argv[++i]));
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            return 1;
        }
    }

    SimpleMesh meshA, meshB;

    std::cerr << "Loading mesh A: " << pathA << std::endl;
    if (!loadOFF(pathA, meshA)) return 1;

    std::cerr << "Loading mesh B: " << pathB << std::endl;
    if (!loadOFF(pathB, meshB)) return 1;

    std::cerr << "Computing Hausdorff distance..." << std::endl;
    HausdorffResult res = computeHausdorff(meshA, meshB, bruteForce, sampleVertices);

    std::cout << std::setprecision(10);

    if (jsonOutput) {
        std::cout << "{" << std::endl;
        std::cout << "  \"hausdorff_ab\": " << res.oneSidedAB << "," << std::endl;
        std::cout << "  \"hausdorff_ba\": " << res.oneSidedBA << "," << std::endl;
        std::cout << "  \"hausdorff_symmetric\": " << res.symmetric << "," << std::endl;
        std::cout << "  \"mean_ab\": " << res.meanAB << "," << std::endl;
        std::cout << "  \"mean_ba\": " << res.meanBA << "," << std::endl;
        std::cout << "  \"rms_ab\": " << res.rmsAB << "," << std::endl;
        std::cout << "  \"rms_ba\": " << res.rmsBA << std::endl;
        std::cout << "}" << std::endl;
    } else {
        std::cout << std::endl;
        std::cout << "========== Hausdorff Distance Results ==========" << std::endl;
        std::cout << "  Hausdorff(A->B) : " << res.oneSidedAB << std::endl;
        std::cout << "  Hausdorff(B->A) : " << res.oneSidedBA << std::endl;
        std::cout << "  Symmetric       : " << res.symmetric << std::endl;
        std::cout << "  Mean(A->B)      : " << res.meanAB << std::endl;
        std::cout << "  Mean(B->A)      : " << res.meanBA << std::endl;
        std::cout << "  RMS(A->B)       : " << res.rmsAB << std::endl;
        std::cout << "  RMS(B->A)       : " << res.rmsBA << std::endl;
        std::cout << "================================================" << std::endl;
    }

    return 0;
}
