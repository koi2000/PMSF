// MeshVertexDeviation —— 定位"哪些顶点被放错了位置"
//
// 背景：HausdorffDistance 只给 max/mean/RMS，能说明"有离群点"，但指不出**是哪些**。
// 而 `docs/refactor/PHASE1_KAPPA_CALIBRATION_2026-09-19.md` §9 已证明 ppmc 的
// hausdorff 由极少数顶点决定（RMS 比 max 小 61×）——要修就必须先定位它们。
//
// §9.1 已警告：**不能按顶点序号逐点比对**（重建会改顶点顺序，序号错配会得到
// 6.33 这种假位移，比真实误差大 88×）。所以本工具按**空间**求最近距离，
// 与 HausdorffDistance 同源，并额外报出每个离群顶点在 B 中的局部拓扑
// （度、是否在边界、相邻面最小面积），以便判断是不是特征边/角点处的退化。
//
// 用法：
//   MeshVertexDeviation <reference.off> <decoded.off> [topK=20] [--json]
//
// 与 HausdorffDistance 一样**不依赖 MCGAL / src/**（LINK_LIBRARIES 清空）。

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}
    Vec3 operator-(const Vec3& o) const {
        return {x - o.x, y - o.y, z - o.z};
    }
    Vec3 operator+(const Vec3& o) const {
        return {x + o.x, y + o.y, z + o.z};
    }
    Vec3 operator*(double s) const {
        return {x * s, y * s, z * s};
    }
    double dot(const Vec3& o) const {
        return x * o.x + y * o.y + z * o.z;
    }
    Vec3 cross(const Vec3& o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }
    double length() const {
        return std::sqrt(x * x + y * y + z * z);
    }
};

struct Triangle {
    Vec3 a, b, c;
};

struct SimpleMesh {
    std::vector<Vec3> vertices;
    std::vector<std::vector<int>> faces;
};

// Ericson "Real-Time Collision Detection" —— 点到三角形的最近点
Vec3 closestPointOnTriangle(const Vec3& p, const Triangle& tri) {
    Vec3 ab = tri.b - tri.a;
    Vec3 ac = tri.c - tri.a;
    Vec3 ap = p - tri.a;

    double d1 = ab.dot(ap);
    double d2 = ac.dot(ap);
    if (d1 <= 0.0 && d2 <= 0.0)
        return tri.a;

    Vec3 bp = p - tri.b;
    double d3 = ab.dot(bp);
    double d4 = ac.dot(bp);
    if (d3 >= 0.0 && d4 <= d3)
        return tri.b;

    double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        double v = d1 / (d1 - d3);
        return tri.a + ab * v;
    }

    Vec3 cp = p - tri.c;
    double d5 = ab.dot(cp);
    double d6 = ac.dot(cp);
    if (d6 >= 0.0 && d5 <= d6)
        return tri.c;

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

bool loadOFF(const std::string& path, SimpleMesh& mesh) {
    std::ifstream in(path);
    if (!in) {
        std::cerr << "错误: 无法打开文件 " << path << std::endl;
        return false;
    }
    std::string header;
    in >> header;
    if (header != "OFF" && header.rfind("OFF", 0) != 0) {
        std::cerr << "错误: 不是有效的 OFF 文件 (头部: " << header << ")" << std::endl;
        return false;
    }
    long long nv = 0, nf = 0, ne = 0;
    in >> nv >> nf >> ne;
    mesh.vertices.reserve(static_cast<size_t>(nv));
    for (long long i = 0; i < nv; ++i) {
        double x = 0, y = 0, z = 0;
        in >> x >> y >> z;
        if (!in) {
            std::cerr << "警告: 顶点读取在 " << i << " 处提前结束" << std::endl;
            break;
        }
        mesh.vertices.emplace_back(x, y, z);
    }
    mesh.faces.reserve(static_cast<size_t>(nf));
    for (long long i = 0; i < nf; ++i) {
        int k = 0;
        in >> k;
        if (!in || k < 3) {
            if (!in)
                break;
            continue;
        }
        std::vector<int> f(static_cast<size_t>(k));
        for (int j = 0; j < k; ++j)
            in >> f[static_cast<size_t>(j)];
        if (!in)
            break;
        mesh.faces.push_back(std::move(f));
    }
    return true;
}

std::vector<Triangle> extractTriangles(const SimpleMesh& mesh) {
    std::vector<Triangle> tris;
    for (const auto& face : mesh.faces) {
        if (face.size() < 3)
            continue;
        for (size_t i = 1; i + 1 < face.size(); i++) {
            Triangle t;
            t.a = mesh.vertices[static_cast<size_t>(face[0])];
            t.b = mesh.vertices[static_cast<size_t>(face[i])];
            t.c = mesh.vertices[static_cast<size_t>(face[i + 1])];
            tris.push_back(t);
        }
    }
    return tris;
}

// 顶点在**解码网格 B** 里的局部拓扑：度、是否边界、相邻三角形最小面积。
struct LocalInfo {
    int valence = 0;          // 关联的面数
    bool onBoundary = false;  // 是否含只出现一次的边
    double minTriArea = 0.0;  // 相邻三角形最小面积（0 表示有退化面）
    double maxTriArea = 0.0;
    int degenerateTris = 0;  // 面积 < 1e-12 的相邻面数
};

void buildLocalInfo(const SimpleMesh& mesh, std::vector<LocalInfo>& info) {
    info.assign(mesh.vertices.size(), LocalInfo{});

    std::map<std::pair<int, int>, int> edgeCount;
    std::vector<double> triArea(mesh.faces.size(), 0.0);

    for (size_t fi = 0; fi < mesh.faces.size(); ++fi) {
        const auto& f = mesh.faces[fi];
        if (f.size() < 3)
            continue;
        // 面积（扇形三角化求和）
        double area = 0.0;
        const Vec3& p0 = mesh.vertices[static_cast<size_t>(f[0])];
        for (size_t i = 1; i + 1 < f.size(); ++i) {
            Vec3 e1 = mesh.vertices[static_cast<size_t>(f[i])] - p0;
            Vec3 e2 = mesh.vertices[static_cast<size_t>(f[i + 1])] - p0;
            area += 0.5 * e1.cross(e2).length();
        }
        triArea[fi] = area;

        for (size_t i = 0; i < f.size(); ++i) {
            int a = f[i];
            int b = f[(i + 1) % f.size()];
            if (a == b)
                continue;
            auto key = std::minmax(a, b);
            edgeCount[{key.first, key.second}]++;
        }
        for (int v : f) {
            if (v < 0 || static_cast<size_t>(v) >= info.size())
                continue;
            LocalInfo& li = info[static_cast<size_t>(v)];
            ++li.valence;
            if (li.minTriArea == 0.0 || area < li.minTriArea)
                li.minTriArea = area;
            if (area > li.maxTriArea)
                li.maxTriArea = area;
            if (area < 1e-12)
                ++li.degenerateTris;
        }
    }

    for (const auto& [e, c] : edgeCount) {
        if (c != 1)
            continue;
        if (e.first >= 0 && static_cast<size_t>(e.first) < info.size())
            info[static_cast<size_t>(e.first)].onBoundary = true;
        if (e.second >= 0 && static_cast<size_t>(e.second) < info.size())
            info[static_cast<size_t>(e.second)].onBoundary = true;
    }
}

struct Deviation {
    int vertex = -1;
    Vec3 pos;
    double dist = 0.0;
};

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "MeshVertexDeviation —— 定位被放错位置的顶点（按空间求距离，非按序号）\n"
                  << "Usage: " << argv[0] << " <reference.off> <decoded.off> [topK=20] [--json]\n";
        return 1;
    }
    const std::string pathRef = argv[1];
    const std::string pathDec = argv[2];
    int topK = 20;
    bool json = false;
    std::string pathBase;  // 可选的第三方参照：用于"是谁把这点搞坏的"归因
    for (int i = 3; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--json") {
            json = true;
        } else if (a == "--compare" && i + 1 < argc) {
            pathBase = argv[++i];
        } else {
            topK = std::max(1, std::atoi(a.c_str()));
        }
    }

    SimpleMesh ref, dec;
    if (!loadOFF(pathRef, ref) || !loadOFF(pathDec, dec))
        return 2;

    // 可选基线（例：原版实现的解码结果）。对每个最差顶点同时报出它在基线上的偏差——
    // 若基线在该点很准而本实现在该点很差，那这个点就是**本实现独有的缺陷**，
    // 而不是"这个网格本来就难"。这比只看单侧 max 更有归因力。
    SimpleMesh base;
    std::vector<Triangle> baseTris;
    if (!pathBase.empty()) {
        if (!loadOFF(pathBase, base))
            return 2;
        baseTris = extractTriangles(base);
    }

    std::vector<Triangle> refTris = extractTriangles(ref);
    std::vector<LocalInfo> local;
    buildLocalInfo(dec, local);

    std::vector<Deviation> devs;
    devs.reserve(dec.vertices.size());
    double sum = 0.0, sumSq = 0.0, worst = 0.0;
    for (size_t i = 0; i < dec.vertices.size(); ++i) {
        const Vec3& p = dec.vertices[i];
        double best = std::numeric_limits<double>::max();
        for (const auto& t : refTris) {
            Vec3 c = closestPointOnTriangle(p, t);
            double d = (p - c).length();
            if (d < best)
                best = d;
        }
        if (best == std::numeric_limits<double>::max())
            best = 0.0;
        Deviation dv;
        dv.vertex = static_cast<int>(i);
        dv.pos = p;
        dv.dist = best;
        devs.push_back(dv);
        sum += best;
        sumSq += best * best;
        if (best > worst)
            worst = best;
    }

    // 归因列：基线**自己的顶点**在对应位置的误差（基线顶点 → 参考表面）。
    //
    // 注意这里**不能**用「decoded 的顶点 P 到基线表面的距离」当作基线的误差 ——
    // 那是两回事：若 P 本身被放错，它离基线表面自然远，会被误判成"基线也差"。
    // 正确口径是：找到 P 最近的基线顶点，报**那个顶点自己的**偏差。
    // （本工具第一版就是这样写错的，实测立刻被 ppmc/bunny 的 v1 揭穿：
    //   两侧数字都是 1.07e-4，看似"两边都差"，实为基线在该处根本没有坏顶点。）
    std::vector<double> baseVertexDev;  // 基线各顶点 → 参考表面的偏差
    if (!baseTris.empty()) {
        baseVertexDev.assign(base.vertices.size(), 0.0);
        for (size_t i = 0; i < base.vertices.size(); ++i) {
            double best = std::numeric_limits<double>::max();
            for (const auto& t : refTris) {
                Vec3 c = closestPointOnTriangle(base.vertices[i], t);
                double d = (base.vertices[i] - c).length();
                if (d < best)
                    best = d;
            }
            baseVertexDev[i] = (best == std::numeric_limits<double>::max()) ? 0.0 : best;
        }
    }

    // 对每个 decoded 顶点，找最近的基线顶点（用于把两侧的"同一点"对齐）
    std::vector<int> nearestBase;
    if (!baseTris.empty()) {
        nearestBase.assign(dec.vertices.size(), -1);
        for (size_t i = 0; i < dec.vertices.size(); ++i) {
            double best = std::numeric_limits<double>::max();
            int bj = -1;
            for (size_t j = 0; j < base.vertices.size(); ++j) {
                double d = (dec.vertices[i] - base.vertices[j]).length();
                if (d < best) {
                    best = d;
                    bj = static_cast<int>(j);
                }
            }
            nearestBase[i] = bj;
        }
    }
    auto baseDevAt = [&](int v) -> double {
        if (baseVertexDev.empty() || v < 0)
            return 0.0;
        int bj = nearestBase[static_cast<size_t>(v)];
        return (bj < 0) ? 0.0 : baseVertexDev[static_cast<size_t>(bj)];
    };

    std::sort(devs.begin(), devs.end(), [](const Deviation& a, const Deviation& b) { return a.dist > b.dist; });

    const size_t n = dec.vertices.size();
    const double mean = n ? sum / static_cast<double>(n) : 0.0;
    const double rms = n ? std::sqrt(sumSq / static_cast<double>(n)) : 0.0;

    if (json) {
        std::cout << std::setprecision(12) << "{\n";
        std::cout << "  \"reference\": \"" << pathRef << "\",\n";
        std::cout << "  \"decoded\": \"" << pathDec << "\",\n";
        std::cout << "  \"vertices\": " << n << ",\n";
        std::cout << "  \"max\": " << worst << ",\n";
        std::cout << "  \"mean\": " << mean << ",\n";
        std::cout << "  \"rms\": " << rms << ",\n";
        std::cout << "  \"top\": [\n";
        for (int i = 0; i < topK && i < static_cast<int>(devs.size()); ++i) {
            const auto& d = devs[static_cast<size_t>(i)];
            const auto& li = local[static_cast<size_t>(d.vertex)];
            std::cout << "    {\"v\": " << d.vertex << ", \"dist\": " << d.dist << ", \"valence\": " << li.valence
                      << ", \"boundary\": " << (li.onBoundary ? "true" : "false") << ", \"minTriArea\": " << li.minTriArea
                      << ", \"degenerateTris\": " << li.degenerateTris << "}";
            std::cout << (i + 1 < topK && i + 1 < static_cast<int>(devs.size()) ? ",\n" : "\n");
        }
        std::cout << "  ]\n}\n";
        return 0;
    }

    std::cerr << "reference : " << pathRef << "  (" << ref.vertices.size() << " v, " << ref.faces.size() << " f)\n";
    std::cerr << "decoded   : " << pathDec << "  (" << dec.vertices.size() << " v, " << dec.faces.size() << " f)\n";

    std::cout << "=== 单侧偏差（decoded 的每个顶点 → reference 表面）===\n";
    std::cout << std::setprecision(10);
    std::cout << "  max  = " << worst << "\n";
    std::cout << "  mean = " << mean << "\n";
    std::cout << "  rms  = " << rms << "\n";
    if (mean > 0.0) {
        std::cout << "  max/mean = " << (worst / mean) << "   max/rms = " << (worst / rms) << "   （比值大 ⇒ 误差由少数离群点决定）\n";
    }

    // 反推"要多少个点出错才能撑起这个 RMS"——与 §9 同一手法，用于核对本次采样
    if (n && rms > 0.0) {
        std::cout << "\n=== 离群点贡献分解（若其余点误差≈mean）===\n";
        for (int k : {1, 2, 5, 10, 20, 50}) {
            double need = std::sqrt((rms * rms * static_cast<double>(n) - mean * mean * static_cast<double>(n - k)) / static_cast<double>(k));
            std::cout << "  若只有 " << std::setw(3) << k << " 个点出错，单点误差需 ≈ " << need << "\n";
        }
    }

    std::cout << "\n=== 最差的 " << topK << " 个顶点（含局部拓扑）===\n";
    std::cout << std::setw(8) << "vertex" << std::setw(16) << "dist" << std::setw(9) << "valence" << std::setw(9) << "boundary" << std::setw(16)
              << "minTriArea" << std::setw(8) << "degen";
    if (!baseVertexDev.empty())
        std::cout << std::setw(18) << "err@baseline";
    std::cout << "   position\n";
    for (int i = 0; i < topK && i < static_cast<int>(devs.size()); ++i) {
        const auto& d = devs[static_cast<size_t>(i)];
        const auto& li = local[static_cast<size_t>(d.vertex)];
        std::cout << std::setw(8) << d.vertex << std::setw(16) << d.dist << std::setw(9) << li.valence << std::setw(9)
                  << (li.onBoundary ? "yes" : "no") << std::setw(16) << li.minTriArea << std::setw(8) << li.degenerateTris;
        if (!baseVertexDev.empty())
            std::cout << std::setw(18) << baseDevAt(d.vertex);
        std::cout << "   (" << d.pos.x << ", " << d.pos.y << ", " << d.pos.z << ")\n";
    }

    // 汇总：离群点里有多少在边界上 / 有多少相邻退化面
    int boundaryCount = 0, degenerateCount = 0;
    for (int i = 0; i < topK && i < static_cast<int>(devs.size()); ++i) {
        const auto& li = local[static_cast<size_t>(devs[static_cast<size_t>(i)].vertex)];
        if (li.onBoundary)
            ++boundaryCount;
        if (li.degenerateTris > 0)
            ++degenerateCount;
    }
    std::cout << "\n前 " << std::min<int>(topK, static_cast<int>(devs.size())) << " 个最差顶点中：在边界上 " << boundaryCount << "，相邻退化面 >0 的 "
              << degenerateCount << "\n";

    if (!baseVertexDev.empty()) {
        int onlyUs = 0, bothBad = 0;
        int n = std::min<int>(topK, static_cast<int>(devs.size()));
        for (int i = 0; i < n; ++i) {
            const auto& d = devs[static_cast<size_t>(i)];
            double bd = baseDevAt(d.vertex);
            if (d.dist > 3.0 * std::max(bd, 1e-30)) {
                ++onlyUs;  // 本实现在此点独有地差（基线在同一位置好得多）
            } else {
                ++bothBad;  // 基线在同一位置也差 ⇒ 该点本身难
            }
        }
        std::cout << "归因（基线 = " << pathBase << "）：\n";
        std::cout << "  err@baseline = 基线**离该位置最近的顶点**其自身的偏差（同口径）\n";
        std::cout << "  **本实现独有**地差（本实现 > 3× 基线）：" << onlyUs << "\n";
        std::cout << "  两边都差（该点本身难）          ：" << bothBad << "\n";
    }
    return 0;
}
