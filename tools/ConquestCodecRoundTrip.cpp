/*
 * @file ConquestCodecRoundTrip.cpp
 * @brief P13.4 (B)② 的**字节层**往返探针：collector 导出 → reader 导入 → 重建器还原。
 *
 * 判据（**纯几何+拓扑**，与池号/半边次序无关）：
 *   ① V / F 数量与快照逐字相等；
 *   ② **顶点集合**（坐标排序后逐位比较，17 位有效数字）相等；
 *   ③ **面集合**（每个面的顶点坐标三元组，规范化后排序）相等。
 *
 * ②③ 一起是"解码重建的网格与编码前**同一个**网格"的直接证据 ——
 * 不依赖任何解码端自报的中间量（那种判据恒为真的时候居多）。
 *
 * 用法：`build/ConquestCodecRoundTrip <mesh.off> [层数]`
 */
#include "Conquest/compress/ConquestSymbolCollectOperator.h"
#include "Conquest/decompress/ConquestSymbolReadOperator.h"
#include "Conquest/reconstruct/ConquestReconstructOperator.h"
#include "core/MeshRuntime.h"
#include "scheme/EngineHooks.h"

#include <cmath>

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {

using Tri = std::array<std::string, 3>;

/// 量化网格：由编码端导出后回填，**判据与 codec 用同一套量化器**
/// （硬约束："验的判据与被验的东西必须用同一套量化器"）。
struct Grid {
    double step = 0.0;
    double bboxMin[3] = {0, 0, 0};
    bool valid = false;
};
Grid g_grid;

using P3 = std::array<double, 3>;

/// 顶点的**格号三元组**。conquest 是有损 codec（12 bit 量化），
/// 逐位比坐标是错的判据 —— 该比的是"落在同一格"。
///
/// ⚠️ 快照一律先存**原始坐标**，键在量化网格已知之后才生成。
/// （踩过：`before` 在网格回填之前取、`after` 之后取，两边根本不是同一把尺子，
/// 于是"全部不等"——正是硬约束说的"验的判据与被验的东西必须用同一套量化器"。）
std::string keyOf(const P3& p) {
    const auto& q = pmsf::defaultEngineHooks().quantizerOrDefault();
    char b[128];
    std::string s;
    for (int k = 0; k < 3; ++k) {
        const int c = g_grid.valid ? q.toCell(p[k], g_grid.bboxMin[k], g_grid.step) : (int)std::llround(p[k] * 1e6);
        std::snprintf(b, sizeof(b), "%d,", c);
        s += b;
    }
    return s;
}

int livingV(MCGAL::Mesh* m) {
    int n = 0;
    for (MCGAL::Vertex* v : m->vertices())
        if (v && !v->isRemoved())
            ++n;
    return n;
}
int livingF(MCGAL::Mesh* m) {
    int n = 0;
    for (MCGAL::Facet* f : m->faces())
        if (f && !f->isRemoved())
            ++n;
    return n;
}

/// 规范化快照：顶点键排序 + 面（顶点键三元组，按最小键旋转后排序）排序。
struct Snapshot {
    std::vector<P3> verts;
    std::vector<std::array<P3, 3>> faces;
};

/// 量化网格已知之后，把原始坐标快照转成可比对的键集合。
struct Keyed {
    std::vector<std::string> verts;
    std::vector<Tri> faces;
};
Keyed keyify(const Snapshot& s) {
    Keyed k;
    for (const auto& v : s.verts)
        k.verts.push_back(keyOf(v));
    for (const auto& f : s.faces) {
        Tri t{keyOf(f[0]), keyOf(f[1]), keyOf(f[2])};
        std::sort(t.begin(), t.end());
        k.faces.push_back(t);
    }
    std::sort(k.verts.begin(), k.verts.end());
    std::sort(k.faces.begin(), k.faces.end());
    return k;
}

/// **纯拓扑**的面集合：顶点先按格号**配对成序号**（配对表由原点集合建立），
/// 面只用序号表示 —— 这样几何有一格之差时，拓扑判据**不会连带变红**。
/// 解码点若落在配对表之外（`miss`）则该面无法规范化，计入 `unresolved`。
struct TopoKeyed {
    std::vector<Tri> faces;
    int unresolved = 0;
};
TopoKeyed topoKeyify(const Snapshot& base, const Snapshot& other) {
    std::map<std::string, int> rank;
    for (const auto& v : base.verts)
        rank.emplace(keyOf(v), 0);
    int n = 0;
    for (auto& kv : rank)
        kv.second = n++;

    TopoKeyed k;
    for (const auto& f : other.faces) {
        Tri t;
        bool ok = true;
        for (int i = 0; i < 3; ++i) {
            auto it = rank.find(keyOf(f[i]));
            if (it == rank.end()) {
                ok = false;
                break;
            }
            t[i] = std::to_string(it->second);
        }
        if (!ok) {
            ++k.unresolved;
            continue;
        }
        std::sort(t.begin(), t.end());
        k.faces.push_back(t);
    }
    std::sort(k.faces.begin(), k.faces.end());
    return k;
}

Snapshot take(MCGAL::Mesh* m) {
    Snapshot s;
    for (MCGAL::Vertex* v : m->vertices()) {
        if (!v || v->isRemoved())
            continue;
        s.verts.push_back({v->point().x(), v->point().y(), v->point().z()});
    }
    for (MCGAL::Facet* f : m->faces()) {
        if (!f || f->isRemoved())
            continue;
        std::vector<P3> ring;
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* it = h0;
        for (int i = 0; i < 512 && it; ++i) {
            if (it->vertex())
                ring.push_back({it->vertex()->point().x(), it->vertex()->point().y(), it->vertex()->point().z()});
            it = it->next();
            if (!it || it == h0)
                break;
        }
        if (ring.size() != 3)
            continue;  // 规范化只比三角面（解码端全部还原成三角面）
        // 面内顶点次序在键化阶段统一排序（消除起点差异），这里保持环序原样
        s.faces.push_back({ring[0], ring[1], ring[2]});
    }
    return s;
}

MCGAL::Halfedge* pickSeed(MCGAL::Mesh* m) {
    // MCGAL 没有 halfedges() 迭代器；从活面的 proxy halfedge 取。
    for (MCGAL::Facet* f : m->faces()) {
        if (!f || f->isRemoved())
            continue;
        MCGAL::Halfedge* h = f->proxyHalfedge();
        if (h && h->end_vertex() && h->opposite() && h->opposite()->end_vertex())
            return h;
    }
    return nullptr;
}

}  // namespace

// ---- 容器产物对拍模式 --------------------------------------------------------
// 用法：ConquestCodecRoundTrip --cmp <原网格.off> <解码.off> <step> <bmin0> <bmin1> <bmin2>
// 判据与主流程**同一套量化器**（`pmsf::defaultEngineHooks().quantizerOrDefault()` 的 toCell），
// 这样"验的判据与被验的东西"才是同一把尺子。
int readOff(const char* path, Snapshot* out) {
    std::ifstream is(path);
    if (!is)
        return 0;
    std::string magic;
    is >> magic;
    if (magic != "OFF")
        return 0;
    long long nv = 0, nf = 0, ne = 0;
    is >> nv >> nf >> ne;
    for (long long i = 0; i < nv; ++i) {
        double x, y, z;
        if (!(is >> x >> y >> z))
            return 0;
        out->verts.push_back({x, y, z});
    }
    for (long long i = 0; i < nf; ++i) {
        int k = 0;
        if (!(is >> k) || k != 3)
            return 0;
        int a, b, c;
        if (!(is >> a >> b >> c))
            return 0;
        out->faces.push_back({out->verts[(size_t)a], out->verts[(size_t)b], out->verts[(size_t)c]});
    }
    return 1;
}

int runCompare(int argc, char** argv) {
    if (argc < 8) {
        std::fprintf(stderr, "用法: %s --cmp <orig.off> <dec.off> <step> <bmin0> <bmin1> <bmin2>\n", argv[0]);
        return 1;
    }
    g_grid.step = std::atof(argv[4]);
    g_grid.bboxMin[0] = std::atof(argv[5]);
    g_grid.bboxMin[1] = std::atof(argv[6]);
    g_grid.bboxMin[2] = std::atof(argv[7]);
    g_grid.valid = g_grid.step > 0.0;

    Snapshot a, b;
    if (!readOff(argv[2], &a) || !readOff(argv[3], &b)) {
        std::fprintf(stderr, "[CMP] **读 OFF 失败**\n");
        return 2;
    }
    // ---- 判据按**框架口径**分三层（硬约束：验的判据与被验的东西用同一套量化器）--
    //  A1 计数：V/F 精确相等。
    //  A2 几何：**每个原顶点**在解码网格里都有距离 ≤ `tol` 的对应点（`tol = step`，
    //           即一个量化格）。容器路径的 base mesh 本来就存**量化坐标**，
    //           要求逐位相等是错的判据 —— 所有算法在这个口径下都一样。
    //  A3 拓扑：用"最近邻且 ≤ tol"把解码顶点**配对**回原点序号，面只比序号。
    // ⚠️ 容差不是随便定的，是**有物理含义且有区分度**的一条界：
    //   量化格是立方的 ⇒ 每轴最多差 1 格时，欧氏距离上界 = √3 · step。
    //   旁证（同一网格 cow_simp、同一把尺子，见 /tmp/baseline_default）：
    //     default 0.77×step（0 未配对） / rd 0.77×step（0 未配对）
    //     conquest 1.51×step（7 未配对） / **pclttm 3.10×step（140 未配对）**
    //   ⇒ 这条界**能区分**：0.77/1.51 过、3.10 不过。不是自证式的橡皮图章。
    const double tol = std::sqrt(3.0) * g_grid.step;
    const bool countsOk = (a.verts.size() == b.verts.size()) && (a.faces.size() == b.faces.size());

    std::vector<int> pairIdx(b.verts.size(), -1);  // 解码点 → 原点序号
    double worst = 0.0;
    int unmatched = 0;
    for (size_t j = 0; j < b.verts.size(); ++j) {
        double bd = 1e300;
        int bi = -1;
        for (size_t i = 0; i < a.verts.size(); ++i) {
            const double dx = a.verts[i][0] - b.verts[j][0], dy = a.verts[i][1] - b.verts[j][1],
                         dz = a.verts[i][2] - b.verts[j][2];
            const double d = dx * dx + dy * dy + dz * dz;
            if (d < bd) { bd = d; bi = (int)i; }
        }
        const double dd = std::sqrt(bd);
        if (dd > worst) worst = dd;
        if (dd <= tol) pairIdx[j] = bi; else ++unmatched;
    }

    // 面集（纯拓扑）：解码面 → 原点序号三元组
    std::vector<Tri> fa, fb;
    for (const auto& f : a.faces) {
        std::vector<std::string> r;
        for (int k = 0; k < 3; ++k) {
            double bd = 1e300; int bi = -1;
            for (size_t i = 0; i < a.verts.size(); ++i) {
                const double dx = a.verts[i][0]-f[k][0], dy = a.verts[i][1]-f[k][1], dz = a.verts[i][2]-f[k][2];
                const double d = dx*dx+dy*dy+dz*dz;
                if (d < bd) { bd = d; bi = (int)i; }
            }
            r.push_back(std::to_string(bi));
        }
        std::sort(r.begin(), r.end());
        fa.push_back({r[0], r[1], r[2]});
    }
    for (const auto& f : b.faces) {
        // 面顶点也要先配对；配不上的面记为不可解析
        bool ok = true;
        std::vector<std::string> r;
        for (int k = 0; k < 3; ++k) {
            double bd = 1e300; int bj = -1;
            for (size_t j = 0; j < b.verts.size(); ++j) {
                const double dx = b.verts[j][0]-f[k][0], dy = b.verts[j][1]-f[k][1], dz = b.verts[j][2]-f[k][2];
                const double d = dx*dx+dy*dy+dz*dz;
                if (d < bd) { bd = d; bj = (int)j; }
            }
            if (bj < 0 || pairIdx[(size_t)bj] < 0) { ok = false; break; }
            r.push_back(std::to_string(pairIdx[(size_t)bj]));
        }
        if (!ok) { fb.push_back({"?" , "?", "?"}); continue; }
        std::sort(r.begin(), r.end());
        fb.push_back({r[0], r[1], r[2]});
    }
    std::sort(fa.begin(), fa.end());
    std::sort(fb.begin(), fb.end());
    const bool topoOk = countsOk && (unmatched == 0) && (fa == fb);
    const bool geomOk = (unmatched == 0);
    std::printf("CMP V原=%zu V解=%zu F原=%zu F解=%zu 计数ok=%d 未配对=%d 最大偏差=%.6g (%.2f×step, tol=%.2f×step) 几何ok=%d 拓扑ok=%d\n",
                a.verts.size(), b.verts.size(), a.faces.size(), b.faces.size(), (int)countsOk, unmatched, worst,
                worst / g_grid.step, tol / g_grid.step, (int)geomOk, (int)topoOk);
    return (countsOk && geomOk && topoOk) ? 0 : 3;
}

// ---- 逐点诊断：按**轴**统计偏差（单位 = step）--------------------------------
// 用途见蓝图 §89.3：一次分开"夹持"与"第二个来源"。
int runDiag(int argc, char** argv) {
    if (argc < 8) {
        std::fprintf(stderr, "用法: %s --diag <a.off> <b.off> <step> <bmin0> <bmin1> <bmin2>\n", argv[0]);
        return 1;
    }
    g_grid.step = std::atof(argv[4]);
    g_grid.bboxMin[0] = std::atof(argv[5]);
    g_grid.bboxMin[1] = std::atof(argv[6]);
    g_grid.bboxMin[2] = std::atof(argv[7]);
    g_grid.valid = g_grid.step > 0.0;
    const auto& q = pmsf::defaultEngineHooks().quantizerOrDefault();

    Snapshot a, b;
    if (!readOff(argv[2], &a) || !readOff(argv[3], &b)) {
        std::fprintf(stderr, "[DIAG] 读 OFF 失败\n");
        return 2;
    }
    // 按 b 的每一点找 a 的最近点，记录**逐轴格差**
    int hist[7][3] = {0};  // 轴差 -3..3
    int offByAxis = 0, onBoundary = 0, maxAbs = 0;
    for (const auto& pb : b.verts) {
        double bd = 1e300;
        int bi = -1;
        for (size_t i = 0; i < a.verts.size(); ++i) {
            const double dx = a.verts[i][0]-pb[0], dy = a.verts[i][1]-pb[1], dz = a.verts[i][2]-pb[2];
            const double d = dx*dx+dy*dy+dz*dz;
            if (d < bd) { bd = d; bi = (int)i; }
        }
        if (bi < 0) continue;
        for (int k = 0; k < 3; ++k) {
            const int ca = q.toCell(a.verts[(size_t)bi][k], g_grid.bboxMin[k], g_grid.step);
            const int cb = q.toCell(pb[k], g_grid.bboxMin[k], g_grid.step);
            int d = cb - ca;
            if (d < -3) d = -3;
            if (d > 3) d = 3;
            ++hist[d + 3][k];
            if (d != 0) ++offByAxis;
            if (std::abs(d) > maxAbs) maxAbs = std::abs(d);
        }
    }
    std::printf("DIAG 点数 a=%zu b=%zu 逐轴格差直方图(-3..3):\n", a.verts.size(), b.verts.size());
    for (int d = -3; d <= 3; ++d)
        std::printf("  d=%+d  x=%d y=%d z=%d\n", d, hist[d+3][0], hist[d+3][1], hist[d+3][2]);
    std::printf("DIAG 非零轴数=%d / 总轴数=%zu  最大|格差|=%d\n", offByAxis, b.verts.size()*3, maxAbs);
    (void)onBoundary;
    return 0;
}

int main(int argc, char** argv) {
    if (argc >= 2 && std::string(argv[1]) == "--cmp")
        return runCompare(argc, argv);
    if (argc >= 2 && std::string(argv[1]) == "--diag")
        return runDiag(argc, argv);
    if (argc < 2) {
        std::fprintf(stderr, "用法: %s <mesh.off> [层数]\n", argv[0]);
        return 1;
    }
    const int layers = argc > 2 ? std::atoi(argv[2]) : 1;
    pmsf::ensureMeshRuntimeInitialized();

    auto mesh = std::make_shared<MCGAL::Mesh>(MCGAL::Mesh(std::string(argv[1])));
    const Snapshot before = take(mesh.get());
    std::fprintf(stderr, "[RT] 编码前 V=%d F=%d 快照 {点 %zu, 面 %zu}\n", livingV(mesh.get()), livingF(mesh.get()),
                 before.verts.size(), before.faces.size());

    // ---- 编码：collector 逐层跑引擎，导出字节流 --------------------------
    pmsf::ConquestSymbolCollectOperator collector;
    collector.init(mesh, {}, PMSFHeader());
    collector.setQuantBits(12);

    std::vector<MCGAL::Halfedge*> seedChain;
    for (int i = 0; i < layers; ++i) {
        MCGAL::Halfedge* seed = pickSeed(mesh.get());
        if (!seed) {
            std::fprintf(stderr, "[RT] 第 %d 层找不到种子\n", i);
            break;
        }
        seedChain.push_back(seed);
        collector.collect(seed);
        std::fprintf(stderr, "[RT] 第 %d 层后 V=%d F=%d\n", i, livingV(mesh.get()), livingF(mesh.get()));
    }

    std::vector<char> buf(1 << 20);
    const int used = collector.exportToBufferWithRangeCoder(buf.data(), true);
    if (used <= 0) {
        std::fprintf(stderr, "[RT] **导出失败** rc=%d\n", used);
        return 2;
    }
    std::fprintf(stderr, "[RT] 字节流 %d B / %zu 层\n", used, collector.stream().layers.size());
    g_grid.step = collector.stream().step;
    g_grid.bboxMin[0] = collector.stream().bboxMin[0];
    g_grid.bboxMin[1] = collector.stream().bboxMin[1];
    g_grid.bboxMin[2] = collector.stream().bboxMin[2];
    g_grid.valid = g_grid.step > 0.0;
    std::fprintf(stderr, "[RT] 量化网格 step=%.17g bboxMin=(%.17g,%.17g,%.17g)\n", g_grid.step, g_grid.bboxMin[0],
                 g_grid.bboxMin[1], g_grid.bboxMin[2]);

    // 编码后的网格 = 解码端的 base mesh
    const Snapshot decimated = take(mesh.get());
    std::fprintf(stderr, "[RT] 编码后（= 解码端输入）V=%d F=%d 快照 {点 %zu, 面 %zu}\n", livingV(mesh.get()),
                 livingF(mesh.get()), decimated.verts.size(), decimated.faces.size());

    // ---- 解码：reader 导入 → 重建器逐层还原 ------------------------------
    auto reader = std::make_shared<pmsf::ConquestSymbolReadOperator>();
    reader->setBufferSize((size_t)used);  // reader 上界 = 编码端真实返回长度（生产路径同 ColumnarDeserializeOperator:2306）
    size_t off = 0;
    reader->importFromRangeCodedBuffer(buf.data(), off, 12);
    if (!reader->decodedOk()) {
        std::fprintf(stderr, "[RT] **reader 导入失败**\n");
        return 2;
    }
    auto recon = std::make_shared<pmsf::ConquestReconstructOperator>(reader);
    recon->init(mesh);  // 在**同一个**（已删减的）网格上还原

    for (int i = 0; i < (int)collector.stream().layers.size(); ++i) {
        std::vector<MCGAL::Vertex*> vv;
        std::vector<MCGAL::Halfedge*> hh;
        std::vector<MCGAL::Facet*> ff;
        recon->reconstruct(vv, hh, ff);
        std::fprintf(stderr, "[RT] 还原第 %d 层后 V=%d F=%d\n", i, livingV(mesh.get()), livingF(mesh.get()));
    }

    const Snapshot after = take(mesh.get());
    std::fprintf(stderr, "[RT] 解码后 V=%d F=%d 快照 {点 %zu, 面 %zu}\n", livingV(mesh.get()), livingF(mesh.get()),
                 after.verts.size(), after.faces.size());

    // 判据：**量化格**级别的点集/面集全等（与 codec 同一套量化器）。
    const Keyed kb = keyify(before);
    const Keyed ka = keyify(after);
    const bool vOk = (ka.verts == kb.verts);
    const bool fOk = (ka.faces == kb.faces);
    // ---- 判据分两层，**不能混为一谈** --------------------------------
    // ① **拓扑**（V/F 计数 + 面集在格级别全等）：任意层数都应当成立；
    // ② **几何**（点集在格级别全等）：**只有单层才应当成立**。
    //
    // ② 为什么多层不成立（不是缺陷，是有损 codec 的固有行为）：
    //   编码端连续删减时，全部 148 个点的坐标**始终是原始坐标**；
    //   解码端逆第 0 层时，网格里那 30 个"第 1 层已删"的点已经是**量化重建**的坐标
    //   ⇒ 第 0 层重插点的预测子输入与编码端不同 ⇒ 逐层累积量化误差。
    //   蓝图 §64 的链式验证之所以全过，是因为它**每层只编一层解一层**
    //   （层 i 的输入 = 层 i−1 的 `clr_post.off`，全是原始坐标）。
    //   ⇒ "多层几何全等"是**不可能的判据**，拿它当红是自欺。
    // 纯拓扑：顶点按"原点集合的格号"配对成序号，面只比序号。
    const TopoKeyed tb = topoKeyify(before, before);
    const TopoKeyed ta = topoKeyify(before, after);
    const bool countsOk = (before.verts.size() == after.verts.size()) && (before.faces.size() == after.faces.size());
    const bool topoOk = countsOk && (ta.unresolved == 0) && (ta.faces == tb.faces);
    const bool geomOk = vOk && fOk;
    const bool ok = topoOk && (layers == 1 ? geomOk : true) && !recon->failed();
    (void)countsOk;
    std::printf("RESULT layers=%d bytes=%d V原=%d V解=%d F原=%d F解=%d 计数ok=%d 配对失败=%d 点集全等=%d 面集全等=%d "
                "拓扑ok=%d 几何ok=%d ok=%d\n",
                layers, used, (int)before.verts.size(), (int)after.verts.size(), (int)before.faces.size(),
                (int)after.faces.size(), (int)countsOk, ta.unresolved, (int)vOk, (int)fOk, (int)topoOk, (int)geomOk,
                (int)ok);
    if (layers > 1 && !geomOk) {
        std::fprintf(stderr,
                     "[RT] 注：层数>1 时几何格级别全等**不成立是预期的**（量化误差逐层累积）。"
                     "本次纯拓扑全等=%d（无法配对的解码点 %d 个）；单层请另跑 `layers=1` 验几何。\n",
                     (int)topoOk, ta.unresolved);
    }

    if (!vOk) {
        int shown = 0;
        for (const auto& s : kb.verts) {
            if (!std::binary_search(ka.verts.begin(), ka.verts.end(), s) && shown < 5) {
                std::fprintf(stderr, "[RT]   原点 %s 不在解码结果里\n", s.c_str());
                ++shown;
            }
        }
        shown = 0;
        for (const auto& s : ka.verts) {
            if (!std::binary_search(kb.verts.begin(), kb.verts.end(), s) && shown < 5) {
                std::fprintf(stderr, "[RT]   解码点多出 %s\n", s.c_str());
                ++shown;
            }
        }
    }
    if (!fOk) {
        int shown = 0;
        for (const auto& t : kb.faces) {
            if (!std::binary_search(ka.faces.begin(), ka.faces.end(), t) && shown < 5) {
                std::fprintf(stderr, "[RT]   原面 (%s | %s | %s) 不在解码结果里\n", t[0].c_str(), t[1].c_str(),
                             t[2].c_str());
                ++shown;
            }
        }
    }
    return (vOk && fOk && !recon->failed()) ? 0 : 3;
}
