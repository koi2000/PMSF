/*
 * @file SurfaceMetrics.cpp
 * @brief Metro/MESH-style surface-sampling distance metrics for paper alignment.
 *
 * The existing HausdorffDistance samples *vertices* of mesh A against surface B.
 * Metro/MESH sample points on the surface (area-weighted) instead.  This tool
 * implements the paper-style symmetric surface sampling:
 *   - sample N points uniformly by triangle area on each mesh;
 *   - point-to-surface nearest distance via an AABB BVH;
 *   - report one-sided RMS / max / mean, symmetric MSE, normalized RMS.
 *
 * Usage:
 *   SurfaceMetrics <ref.off> <test.off> [--samples N] [--json]
 */
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }
    double length() const { return std::sqrt(dot(*this)); }
};

struct Triangle {
    Vec3 a, b, c;
};

struct Mesh {
    std::vector<Vec3> vertices;
    std::vector<std::vector<int>> faces;
};

bool loadOFF(const std::string& path, Mesh& mesh) {
    std::ifstream in(path);
    if (!in) {
        std::cerr << "cannot open " << path << "\n";
        return false;
    }
    std::string line;
    do {
        if (!std::getline(in, line)) return false;
    } while (line.find_first_not_of(" \t\r\n") == std::string::npos || line[0] == '#');
    if (line.rfind("OFF", 0) == 0) {
        do {
            if (!std::getline(in, line)) return false;
        } while (line.find_first_not_of(" \t\r\n") == std::string::npos || line[0] == '#');
    }
    int nv = 0, nf = 0, ne = 0;
    {
        std::stringstream ss(line);
        if (!(ss >> nv >> nf >> ne)) return false;
    }
    mesh.vertices.resize(nv);
    for (int i = 0; i < nv; ++i) {
        do {
            if (!std::getline(in, line)) return false;
        } while (line.find_first_not_of(" \t\r\n") == std::string::npos || line[0] == '#');
        std::stringstream ss(line);
        ss >> mesh.vertices[i].x >> mesh.vertices[i].y >> mesh.vertices[i].z;
        if (!ss) return false;
    }
    mesh.faces.resize(nf);
    for (int i = 0; i < nf; ++i) {
        do {
            if (!std::getline(in, line)) return false;
        } while (line.find_first_not_of(" \t\r\n") == std::string::npos || line[0] == '#');
        std::stringstream ss(line);
        int n = 0;
        if (!(ss >> n) || n < 3) return false;
        mesh.faces[i].resize(n);
        for (int j = 0; j < n; ++j) {
            if (!(ss >> mesh.faces[i][j])) return false;
        }
    }
    return true;
}

std::vector<Triangle> trianglesOf(const Mesh& mesh) {
    std::vector<Triangle> tris;
    for (const auto& face : mesh.faces) {
        for (std::size_t i = 1; i + 1 < face.size(); ++i) {
            tris.push_back({mesh.vertices[face[0]], mesh.vertices[face[i]],
                            mesh.vertices[face[i + 1]]});
        }
    }
    return tris;
}

Vec3 closestPointOnTriangle(const Vec3& p, const Triangle& tri) {
    const Vec3 ab = tri.b - tri.a;
    const Vec3 ac = tri.c - tri.a;
    const Vec3 ap = p - tri.a;
    const double d1 = ab.dot(ap);
    const double d2 = ac.dot(ap);
    if (d1 <= 0.0 && d2 <= 0.0) return tri.a;
    const Vec3 bp = p - tri.b;
    const double d3 = ab.dot(bp);
    const double d4 = ac.dot(bp);
    if (d3 >= 0.0 && d4 <= d3) return tri.b;
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        const double v = d1 / (d1 - d3);
        return tri.a + ab * v;
    }
    const Vec3 cp = p - tri.c;
    const double d5 = ab.dot(cp);
    const double d6 = ac.dot(cp);
    if (d6 >= 0.0 && d5 <= d6) return tri.c;
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        const double w = d2 / (d2 - d6);
        return tri.a + ac * w;
    }
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
        const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return tri.b + (tri.c - tri.b) * w;
    }
    const double denom = 1.0 / (va + vb + vc);
    const double v = vb * denom;
    const double w = vc * denom;
    return tri.a + ab * v + ac * w;
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

double pointAabbDist2(const Vec3& p, const Aabb& box) {
    const double dx = std::max({box.min.x - p.x, 0.0, p.x - box.max.x});
    const double dy = std::max({box.min.y - p.y, 0.0, p.y - box.max.y});
    const double dz = std::max({box.min.z - p.z, 0.0, p.z - box.max.z});
    return dx * dx + dy * dy + dz * dz;
}

class Bvh {
  public:
    explicit Bvh(const std::vector<Triangle>& tris) : tris_(tris) {
        order_.resize(tris_.size());
        std::iota(order_.begin(), order_.end(), 0);
        // Small prefix meshes can contain degenerate triangles; the AABB build
        // is only an accelerator, so fall back to brute force instead of risking
        // nth_element UB on pathological centroids.
        useBvh_ = tris_.size() >= 256;
        if (useBvh_ && !tris_.empty()) {
            nodes_.reserve(tris_.size() * 2);
            root_ = build(0, static_cast<int>(tris_.size()));
        }
    }

    double distance(const Vec3& p) const {
        if (tris_.empty()) return std::numeric_limits<double>::infinity();
        if (!useBvh_) {
            double best2 = std::numeric_limits<double>::infinity();
            for (const auto& tri : tris_) {
                const Vec3 closest = closestPointOnTriangle(p, tri);
                const Vec3 d = p - closest;
                best2 = std::min(best2, d.dot(d));
            }
            return std::sqrt(best2);
        }
        double best2 = std::numeric_limits<double>::infinity();
        std::vector<int> stack;
        stack.push_back(root_);
        while (!stack.empty()) {
            const int ni = stack.back();
            stack.pop_back();
            const Node& node = nodes_[ni];
            if (pointAabbDist2(p, node.box) >= best2) continue;
            if (node.left < 0) {
                for (int i = node.start; i < node.start + node.count; ++i) {
                    const Vec3 closest = closestPointOnTriangle(p, tris_[order_[i]]);
                    const Vec3 d = p - closest;
                    best2 = std::min(best2, d.dot(d));
                }
            } else {
                const double dl = pointAabbDist2(p, nodes_[node.left].box);
                const double dr = pointAabbDist2(p, nodes_[node.right].box);
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
    struct Node {
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
        nodes_.push_back(Node{boxForRange(start, count), start, count, -1, -1});
        if (count <= 8) return index;
        Aabb centroidBox;
        const auto centroid = [&](int i) {
            const Triangle& tri = tris_[order_[i]];
            return Vec3{(tri.a.x + tri.b.x + tri.c.x) / 3.0,
                        (tri.a.y + tri.b.y + tri.c.y) / 3.0,
                        (tri.a.z + tri.b.z + tri.c.z) / 3.0};
        };
        for (int i = start; i < start + count; ++i) centroidBox.expand(centroid(i));
        const double ex = centroidBox.max.x - centroidBox.min.x;
        const double ey = centroidBox.max.y - centroidBox.min.y;
        const double ez = centroidBox.max.z - centroidBox.min.z;
        const int axis = (ex > ey) ? (ex > ez ? 0 : 2) : (ey > ez ? 1 : 2);
        const int mid = count / 2;
        // NOTE: `lhs`/`rhs` are **triangle indices** (values stored in order_), NOT
        // positions. The comparator must therefore read tris_[lhs] directly. Using
        // the position-based centroid() helper here would index order_ while
        // std::nth_element is itself permuting order_ → comparator is not a strict
        // weak ordering (UB; reproducible SIGSEGV on some vertex orderings).
        const auto triCentroid = [&](int triIndex) {
            const Triangle& tri = tris_[triIndex];
            return Vec3{(tri.a.x + tri.b.x + tri.c.x) / 3.0,
                        (tri.a.y + tri.b.y + tri.c.y) / 3.0,
                        (tri.a.z + tri.b.z + tri.c.z) / 3.0};
        };
        std::nth_element(order_.begin() + start, order_.begin() + start + mid,
                         order_.begin() + start + count, [&](int lhs, int rhs) {
                             const Vec3 a = triCentroid(lhs);
                             const Vec3 b = triCentroid(rhs);
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
    std::vector<Node> nodes_;
    bool useBvh_ = false;
    int root_ = -1;
};

std::vector<Vec3> sampleSurface(const std::vector<Triangle>& tris, int count,
                                std::uint32_t seed) {
    std::vector<Vec3> out;
    out.reserve(static_cast<std::size_t>(count));
    if (tris.empty() || count <= 0) return out;

    std::vector<double> cdf;
    cdf.reserve(tris.size());
    double total = 0.0;
    for (const auto& tri : tris) {
        const Vec3 cross = (tri.b - tri.a).cross(tri.c - tri.a);
        total += 0.5 * cross.length();
        cdf.push_back(total);
    }
    if (total <= 0.0) return out;

    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    for (int i = 0; i < count; ++i) {
        const double target = u01(rng) * total;
        const auto it = std::lower_bound(cdf.begin(), cdf.end(), target);
        const std::size_t ti = std::min<std::size_t>(
            static_cast<std::size_t>(it - cdf.begin()), tris.size() - 1);
        const Triangle& tri = tris[ti];
        const double r1 = std::sqrt(u01(rng));
        const double r2 = u01(rng);
        const double b0 = 1.0 - r1;
        const double b1 = r1 * (1.0 - r2);
        const double b2 = r1 * r2;
        out.push_back(tri.a * b0 + tri.b * b1 + tri.c * b2);
    }
    return out;
}

struct OneSided {
    double max = 0.0;
    double mean = 0.0;
    double rms = 0.0;
};

OneSided metrics(const std::vector<Vec3>& points, const Bvh& bvh) {
    OneSided m;
    if (points.empty()) return m;
    double sum = 0.0;
    double sumSq = 0.0;
    for (const auto& p : points) {
        const double d = bvh.distance(p);
        m.max = std::max(m.max, d);
        sum += d;
        sumSq += d * d;
    }
    const double n = static_cast<double>(points.size());
    m.mean = sum / n;
    m.rms = std::sqrt(sumSq / n);
    return m;
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0]
                  << " <ref.off> <test.off> [--samples N] [--json]\n";
        return 1;
    }
    const std::string refPath = argv[1];
    const std::string testPath = argv[2];
    int samples = 100000;
    bool json = false;
    for (int i = 3; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--json") {
            json = true;
        } else if (arg == "--samples" && i + 1 < argc) {
            samples = std::max(1, std::atoi(argv[++i]));
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            return 1;
        }
    }

    Mesh ref, test;
    if (!loadOFF(refPath, ref) || !loadOFF(testPath, test)) return 1;
    const auto refTris = trianglesOf(ref);
    const auto testTris = trianglesOf(test);
    if (refTris.empty() || testTris.empty()) {
        std::cerr << "empty mesh\n";
        return 1;
    }
    std::cerr << "  ref: " << ref.vertices.size() << " vertices, " << refTris.size()
              << " triangles\n";
    std::cerr << "  test: " << test.vertices.size() << " vertices, " << testTris.size()
              << " triangles\n";
    std::cerr << "  surface samples: " << samples << "\n";

    const auto refPoints = sampleSurface(refTris, samples, 12345u);
    const auto testPoints = sampleSurface(testTris, samples, 54321u);
    Bvh refBvh(refTris);
    Bvh testBvh(testTris);

    const OneSided ab = metrics(refPoints, testBvh);
    const OneSided ba = metrics(testPoints, refBvh);
    const double rmsMax = std::max(ab.rms, ba.rms);
    const double mseSym = 0.5 * (ab.rms * ab.rms + ba.rms * ba.rms);

    Vec3 bmin{1e300, 1e300, 1e300};
    Vec3 bmax{-1e300, -1e300, -1e300};
    for (const auto& v : ref.vertices) {
        bmin.x = std::min(bmin.x, v.x);
        bmin.y = std::min(bmin.y, v.y);
        bmin.z = std::min(bmin.z, v.z);
        bmax.x = std::max(bmax.x, v.x);
        bmax.y = std::max(bmax.y, v.y);
        bmax.z = std::max(bmax.z, v.z);
    }
    const double diag = (bmax - bmin).length();
    const double normalized = diag > 0.0 ? rmsMax / diag : 0.0;

    std::cout << std::setprecision(12);
    if (json) {
        std::cout << "{\n"
                  << "  \"samples\": " << samples << ",\n"
                  << "  \"rms_ab\": " << ab.rms << ",\n"
                  << "  \"rms_ba\": " << ba.rms << ",\n"
                  << "  \"rms_max\": " << rmsMax << ",\n"
                  << "  \"mse_sym\": " << mseSym << ",\n"
                  << "  \"max_ab\": " << ab.max << ",\n"
                  << "  \"max_ba\": " << ba.max << ",\n"
                  << "  \"hausdorff_sym\": " << std::max(ab.max, ba.max) << ",\n"
                  << "  \"bbox_diag\": " << diag << ",\n"
                  << "  \"normalized_rms_max\": " << normalized << "\n"
                  << "}" << std::endl;
    } else {
        std::cout << "SurfaceMetrics\n"
                  << "  samples          : " << samples << "\n"
                  << "  RMS ref->test    : " << ab.rms << "\n"
                  << "  RMS test->ref    : " << ba.rms << "\n"
                  << "  RMS max          : " << rmsMax << "\n"
                  << "  MSE symmetric    : " << mseSym << "\n"
                  << "  H ref->test      : " << ab.max << "\n"
                  << "  H test->ref      : " << ba.max << "\n"
                  << "  bbox diagonal    : " << diag << "\n"
                  << "  normalized RMS   : " << normalized << "\n";
    }
    return 0;
}
