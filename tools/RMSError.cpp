#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
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
        std::cerr << "ERROR: Cannot open file " << path << std::endl;
        return false;
    }

    std::string header;
    fin >> header;
    if (header != "OFF" && header != "COFF" && header != "NOFF") {
        std::cerr << "ERROR: Not a valid OFF file (header: " << header << ")" << std::endl;
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
                std::cerr << "WARNING: Read error, data may be incomplete" << std::endl;
                return false;
            }
        } while (line.find_first_not_of(" \t\r\n") == std::string::npos || line[0] == '#');
        std::stringstream ss(line);
        ss >> mesh.vertices[i].x >> mesh.vertices[i].y >> mesh.vertices[i].z;
        if (!ss) {
            std::cerr << "WARNING: Read error, data may be incomplete" << std::endl;
            return false;
        }
    }

    mesh.faces.resize(nf);
    for (int i = 0; i < nf; i++) {
        do {
            if (!std::getline(fin, line)) {
                std::cerr << "WARNING: Read error, data may be incomplete" << std::endl;
                return false;
            }
        } while (line.find_first_not_of(" \t\r\n") == std::string::npos || line[0] == '#');
        std::stringstream ss(line);
        int n;
        ss >> n;
        if (!ss || n < 0) {
            std::cerr << "WARNING: Read error, data may be incomplete" << std::endl;
            return false;
        }
        mesh.faces[i].resize(n);
        for (int j = 0; j < n; j++) {
            ss >> mesh.faces[i][j];
            if (!ss) {
                std::cerr << "WARNING: Read error, data may be incomplete" << std::endl;
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

static int countDegenerateFaces(const SimpleMesh& mesh) {
    int count = 0;
    for (const auto& face : mesh.faces) {
        if (face.size() < 3) {
            count++;
            continue;
        }
        for (size_t i = 1; i + 1 < face.size(); i++) {
            const Vec3& a = mesh.vertices[face[0]];
            const Vec3& b = mesh.vertices[face[i]];
            const Vec3& c = mesh.vertices[face[i + 1]];
            double area = (b - a).cross(c - a).length() * 0.5;
            if (area < 1e-20) {
                count++;
                break;
            }
        }
    }
    return count;
}

static int countZeroAreaTriangles(const std::vector<Triangle>& tris) {
    int count = 0;
    for (const auto& tri : tris) {
        double area = (tri.b - tri.a).cross(tri.c - tri.a).length() * 0.5;
        if (area < 1e-20) count++;
    }
    return count;
}

struct RMSErrorResult {
    // Vertex-position RMSE (point-to-surface)
    double rmseAB;
    double rmseBA;
    double rmseSymmetric;

    // Vertex-position max error
    double maxErrorAB;
    double maxErrorBA;

    // Counts
    int vertexCountA;
    int vertexCountB;
    int faceCountA;
    int faceCountB;
    int triCountA;
    int triCountB;

    // Topology health
    int degenerateFacesA;
    int degenerateFacesB;

    bool passed;
};

static RMSErrorResult computeRMSError(const SimpleMesh& meshA, const SimpleMesh& meshB,
                                       int sampleN = 0) {
    RMSErrorResult res{};

    auto trisA = extractTriangles(meshA);
    auto trisB = extractTriangles(meshB);

    res.vertexCountA = (int)meshA.vertices.size();
    res.vertexCountB = (int)meshB.vertices.size();
    res.faceCountA = (int)meshA.faces.size();
    res.faceCountB = (int)meshB.faces.size();
    res.triCountA = (int)trisA.size();
    res.triCountB = (int)trisB.size();
    res.degenerateFacesA = countDegenerateFaces(meshA);
    res.degenerateFacesB = countDegenerateFaces(meshB);

    int nA = res.vertexCountA;
    int nB = res.vertexCountB;

    // Build vertex index lists (full or sampled)
    std::vector<int> idxA, idxB;
    if (sampleN > 0 && sampleN < nA) {
        idxA.resize(sampleN);
        for (int i = 0; i < sampleN; i++)
            idxA[i] = (int)((double)i * nA / sampleN);
        nA = sampleN;
    } else {
        idxA.resize(res.vertexCountA);
        for (int i = 0; i < res.vertexCountA; i++) idxA[i] = i;
    }
    if (sampleN > 0 && sampleN < nB) {
        idxB.resize(sampleN);
        for (int i = 0; i < sampleN; i++)
            idxB[i] = (int)((double)i * nB / sampleN);
        nB = sampleN;
    } else {
        idxB.resize(res.vertexCountB);
        for (int i = 0; i < res.vertexCountB; i++) idxB[i] = i;
    }

    // A -> B: distance from each (sampled) A vertex to mesh B surface
    double maxAB = 0, sumSqAB = 0;
    for (int i : idxA) {
        double d = pointToMeshDistance(meshA.vertices[i], trisB);
        maxAB = std::max(maxAB, d);
        sumSqAB += d * d;
    }

    // B -> A: distance from each (sampled) B vertex to mesh A surface
    double maxBA = 0, sumSqBA = 0;
    for (int j : idxB) {
        double d = pointToMeshDistance(meshB.vertices[j], trisA);
        maxBA = std::max(maxBA, d);
        sumSqBA += d * d;
    }

    res.rmseAB = nA > 0 ? std::sqrt(sumSqAB / nA) : 0;
    res.rmseBA = nB > 0 ? std::sqrt(sumSqBA / nB) : 0;
    res.rmseSymmetric = std::sqrt((sumSqAB + sumSqBA) / (nA + nB));
    res.maxErrorAB = maxAB;
    res.maxErrorBA = maxBA;

    return res;
}

int main(int argc, char* argv[]) {
    double threshold = 0.0;
    int sampleN = 0;
    bool jsonOutput = false;
    std::string pathA, pathB;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--json") {
            jsonOutput = true;
        } else if (arg == "--threshold" && i + 1 < argc) {
            threshold = std::stod(argv[++i]);
        } else if (arg == "--sample" && i + 1 < argc) {
            sampleN = std::stoi(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            std::cerr << "RMSError: RMSE-based mesh comparison tool" << std::endl;
            std::cerr << "Usage: " << argv[0] << " <meshA.off> <meshB.off> [--threshold N] [--sample N] [--json]" << std::endl;
            std::cerr << std::endl;
            std::cerr << "  meshA.off    Original/reference mesh" << std::endl;
            std::cerr << "  meshB.off    Decompressed/test mesh" << std::endl;
            std::cerr << "  --threshold  Max acceptable symmetric RMSE (default: 0)" << std::endl;
            std::cerr << "  --sample N   Sample N evenly-spaced vertices (fast approx)" << std::endl;
            std::cerr << "  --json       Machine-parseable JSON output" << std::endl;
            std::cerr << std::endl;
            std::cerr << "Exit codes:" << std::endl;
            std::cerr << "  0  RMSE <= threshold AND no degenerate faces in B" << std::endl;
            std::cerr << "  1  RMSE > threshold" << std::endl;
            std::cerr << "  2  Degenerate faces detected in mesh B" << std::endl;
            std::cerr << "  3  File read error" << std::endl;
            return 0;
        } else if (pathA.empty()) {
            pathA = arg;
        } else if (pathB.empty()) {
            pathB = arg;
        }
    }

    if (pathA.empty() || pathB.empty()) {
        std::cerr << "ERROR: Two OFF files required. Use --help for usage." << std::endl;
        return 3;
    }

    SimpleMesh meshA, meshB;

    std::cerr << "Loading mesh A (reference): " << pathA << std::endl;
    if (!loadOFF(pathA, meshA)) return 3;

    std::cerr << "Loading mesh B (test):      " << pathB << std::endl;
    if (!loadOFF(pathB, meshB)) return 3;

    std::cerr << "Computing RMSE..."
              << (sampleN > 0 ? " (sampled " + std::to_string(sampleN) + " vertices)" : "")
              << std::endl;
    RMSErrorResult res = computeRMSError(meshA, meshB, sampleN);

    bool topologyOk = (res.degenerateFacesB == 0);
    bool rmseOk = (res.rmseSymmetric <= threshold);

    std::cout << std::setprecision(10);

    if (jsonOutput) {
        std::cout << "{" << std::endl;
        std::cout << "  \"rmse_ab\": " << res.rmseAB << "," << std::endl;
        std::cout << "  \"rmse_ba\": " << res.rmseBA << "," << std::endl;
        std::cout << "  \"rmse_symmetric\": " << res.rmseSymmetric << "," << std::endl;
        std::cout << "  \"max_error_ab\": " << res.maxErrorAB << "," << std::endl;
        std::cout << "  \"max_error_ba\": " << res.maxErrorBA << "," << std::endl;
        std::cout << "  \"vertex_count_a\": " << res.vertexCountA << "," << std::endl;
        std::cout << "  \"vertex_count_b\": " << res.vertexCountB << "," << std::endl;
        std::cout << "  \"face_count_a\": " << res.faceCountA << "," << std::endl;
        std::cout << "  \"face_count_b\": " << res.faceCountB << "," << std::endl;
        std::cout << "  \"tri_count_a\": " << res.triCountA << "," << std::endl;
        std::cout << "  \"tri_count_b\": " << res.triCountB << "," << std::endl;
        std::cout << "  \"degenerate_faces_a\": " << res.degenerateFacesA << "," << std::endl;
        std::cout << "  \"degenerate_faces_b\": " << res.degenerateFacesB << "," << std::endl;
        std::cout << "  \"rmse_ok\": " << (rmseOk ? "true" : "false") << "," << std::endl;
        std::cout << "  \"topology_ok\": " << (topologyOk ? "true" : "false") << std::endl;
        std::cout << "}" << std::endl;
    } else {
        std::cout << std::endl;
        std::cout << "==================== RMSError Results ====================" << std::endl;
        std::cout << "  Reference (A): " << pathA << std::endl;
        std::cout << "  Test      (B): " << pathB << std::endl;
        std::cout << "----------------------------------------------------------" << std::endl;
        std::cout << "  Vertices:  A=" << res.vertexCountA << "  B=" << res.vertexCountB << std::endl;
        std::cout << "  Faces:     A=" << res.faceCountA << "  B=" << res.faceCountB << std::endl;
        std::cout << "  Triangles: A=" << res.triCountA << "  B=" << res.triCountB << std::endl;
        std::cout << "----------------------------------------------------------" << std::endl;
        std::cout << "  RMSE(A->B)     : " << res.rmseAB << std::endl;
        std::cout << "  RMSE(B->A)     : " << res.rmseBA << std::endl;
        std::cout << "  RMSE(symmetric): " << res.rmseSymmetric << std::endl;
        std::cout << "  MaxError(A->B) : " << res.maxErrorAB << std::endl;
        std::cout << "  MaxError(B->A) : " << res.maxErrorBA << std::endl;
        std::cout << "----------------------------------------------------------" << std::endl;
        std::cout << "  Degenerate faces: A=" << res.degenerateFacesA
                  << "  B=" << res.degenerateFacesB << std::endl;
        std::cout << "----------------------------------------------------------" << std::endl;
        std::cout << "  RMSE threshold: " << threshold << std::endl;
        std::cout << "  RMSE check:     " << (rmseOk ? "PASS" : "FAIL") << std::endl;
        std::cout << "  Topology check: " << (topologyOk ? "PASS" : "FAIL") << std::endl;
        std::cout << "==========================================================" << std::endl;
    }

    if (!topologyOk) return 2;
    if (!rmseOk) return 1;
    return 0;
}
