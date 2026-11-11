// SPDX-License-Identifier: Apache-2.0
//
// tools/ConquestEngineVerify.cpp —— `src/conquest/FrontierConquestEngine` 的
// **对拍驱动器**：用迁移进框架的引擎跑一遍，把连通性符号序列导出来，
// 与外部参考实现的 oracle 比。
//
// 为什么需要它：`tools/AdConquestVerify.cpp` 里有一份**独立**的引擎实现
// （带大量调试探针）。迁移进 `src/` 之后，必须证明**迁进来的那份**与
// 原来对拍通过的那份**逐符号一致** —— 否则就是"两份实现各自能跑"，
// 而不是"迁移成功"。
//
// 用法：ConquestEngineVerify <mesh.off> <out_symbols.txt>
//   PMSF_CE_SEED_A / PMSF_CE_SEED_B : 种子两端坐标（与 oracle dump 对接）
//   PMSF_CE_MAXV                    : 顶点数降到此值以下即停（复刻 oracle 停止规则）
//   PMSF_CE_QBITS                   : 量化位数（默认 10）

#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Point.h"
#include "Vertex.h"

#include "conquest/FrontierConquestEngine.h"
#include "core/MeshRuntime.h"
#include "entropy/RangeCoderEntropyCoder.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

namespace {

// 收集几何残差（量化整数域的三元组）
struct CollectSink : pmsf::GeometrySink {
    std::vector<int> v;
    void onRemovedVertex(const int d[3]) override {
        v.push_back(d[0]);
        v.push_back(d[1]);
        v.push_back(d[2]);
    }
};

int livingVertices(MCGAL::Mesh* mesh) {
    int n = 0;
    for (MCGAL::Vertex* v : mesh->vertices())
        if (v && !v->isRemoved())
            ++n;
    return n;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "用法: %s <mesh.off> <out_symbols.txt>\n", argv[0]);
        return 1;
    }
    pmsf::ensureMeshRuntimeInitialized();
    auto mesh = std::make_shared<MCGAL::Mesh>(MCGAL::Mesh(std::string(argv[1])));

    // ---- 种子：按**坐标**匹配（库的表示顺序不同，按编号对不齐）----
    auto parseXYZ = [](const char* s, double o[3]) -> bool { return s && std::sscanf(s, "%lf %lf %lf", &o[0], &o[1], &o[2]) == 3; };
    double A[3], B[3];
    if (!parseXYZ(std::getenv("PMSF_CE_SEED_A"), A) || !parseXYZ(std::getenv("PMSF_CE_SEED_B"), B)) {
        std::fprintf(stderr, "需要 PMSF_CE_SEED_A / PMSF_CE_SEED_B（种子两端坐标）\n");
        return 1;
    }
    MCGAL::Vertex* va = nullptr;
    MCGAL::Vertex* vb = nullptr;
    double bestA = 1e30, bestB = 1e30;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved())
            continue;
        double d = 0;
        for (int k = 0; k < 3; ++k) {
            const double c[3] = {v->x(), v->y(), v->z()};
            d += (c[k] - A[k]) * (c[k] - A[k]);
        }
        if (d < bestA) {
            bestA = d;
            va = v;
        }
        d = 0;
        for (int k = 0; k < 3; ++k) {
            const double c[3] = {v->x(), v->y(), v->z()};
            d += (c[k] - B[k]) * (c[k] - B[k]);
        }
        if (d < bestB) {
            bestB = d;
            vb = v;
        }
    }
    // 找 B -> A 的半边
    MCGAL::Halfedge* seed = nullptr;
    if (va && vb && va != vb) {
        MCGAL::Halfedge* start = vb->proxyHalfedge();
        if (start) {
            MCGAL::Halfedge* it = start;
            for (int i = 0; i < 256; ++i) {
                if (it->end_vertex() == va) {
                    seed = it;
                    break;
                }
                MCGAL::Halfedge* o = it->opposite();
                if (!o)
                    break;
                it = o->next();
                if (!it || it == start)
                    break;
            }
        }
    }
    if (!seed) {
        std::fprintf(stderr, "[CE] seed 未找到\n");
        return 1;
    }
    std::fprintf(stderr, "[CE] seed v%d->v%d (d2 %.3g/%.3g)\n", seed->vertex()->poolId(), seed->end_vertex()->poolId(), bestA, bestB);

    const int maxV = std::getenv("PMSF_CE_MAXV") ? std::atoi(std::getenv("PMSF_CE_MAXV")) : 0;

    pmsf::FrontierConquestEngine engine;

    // 量化网格：包围盒在**原始输入网格**上算（与参考实现一致），由 V0 推 step。
    CollectSink sink;
    {
        float mn[3] = {1e30f, 1e30f, 1e30f}, mx[3] = {-1e30f, -1e30f, -1e30f};
        for (MCGAL::Vertex* v : mesh->vertices()) {
            if (!v || v->isRemoved())
                continue;
            const float c[3] = {v->x(), v->y(), v->z()};
            for (int k = 0; k < 3; ++k) {
                mn[k] = std::min(mn[k], c[k]);
                mx[k] = std::max(mx[k], c[k]);
            }
        }
        const double span = std::max({(double)(mx[0] - mn[0]), (double)(mx[1] - mn[1]), (double)(mx[2] - mn[2])});
        const int qbits = std::getenv("PMSF_CE_QBITS") ? std::atoi(std::getenv("PMSF_CE_QBITS")) : 10;
        pmsf::FrontierConquestEngine::QuantGrid qg;
        for (int k = 0; k < 3; ++k)
            qg.bboxMin[k] = mn[k];
        qg.step = span / (double)(1 << qbits);
        engine.setQuantGrid(qg);
        std::fprintf(stderr, "[CE] quant grid: span=%.6g step=%.6g (Qbits=%d)\n", span, qg.step, qbits);
    }
    pmsf::ConquestState st;
    std::vector<int> syms;

    int prevV = livingVertices(mesh.get());
    std::fprintf(stderr, "[CE] V0=%d\n", prevV);
    // 两族符号**分开收集**：删减层与清理层的字母表不同（5 元 vs 2 元），
    // 熵编码器也应当分开建（oracle 就是这么做的：Un_Regulation 用 Connectivity(2)）。
    std::vector<int> symsDec, symsReg;
    std::vector<size_t> layerMarks;  // 每层在 sink.v 里的起点（几何分层用）
    for (int round = 0; round < 200; ++round) {
        // ⚠️ symsDec/symsReg 在循环外声明、**跨轮累积**，所以这里只能追加
        // **本轮新增**的那一段。用 `before = syms.size()` 去索引 symsDec 是错的
        // （两者长度不在同一空间）—— 会让 syms 反复重复累加。我第一版就这么错过，
        // 量出 216816 个符号（应为 30000）。好在符号数明显不对，当场暴露。
        const size_t bd0 = symsDec.size(), br0 = symsReg.size();
        const size_t gl0 = sink.v.size();  // 几何：本层起点
        layerMarks.push_back(gl0);
        (void)gl0;
        const auto d = engine.decimateLayer(mesh.get(), st, seed, &symsDec, &sink);
        const auto r = engine.regulateLayer(mesh.get(), st, seed, &symsReg, &sink);
        for (size_t i = bd0; i < symsDec.size(); ++i)
            syms.push_back(symsDec[i]);
        for (size_t i = br0; i < symsReg.size(); ++i)
            syms.push_back(symsReg[i]);
        const int nowV = livingVertices(mesh.get());
        std::fprintf(stderr, "[CE] round %d: dec=%d reg=%d V=%d syms=%zu\n", round, d.removedVertices, r.removedVertices, nowV, syms.size());
        if (maxV > 0 && nowV < maxV)
            break;
        if (nowV == prevV)
            break;
        prevV = nowV;
    }

    FILE* fp = std::fopen(argv[2], "w");
    if (!fp) {
        std::fprintf(stderr, "cannot write %s\n", argv[2]);
        return 1;
    }
    std::fprintf(fp, "symbols");
    for (int s : syms)
        std::fprintf(fp, " %d", s);
    std::fprintf(fp, "\n");
    std::fclose(fp);
    std::fprintf(stderr, "[CE] wrote %zu symbols\n", syms.size());

    // ---- 几何残差统计（量化整数域）----
    if (!sink.v.empty()) {
        const size_t nverts = sink.v.size() / 3;
        long long mx[3] = {0, 0, 0};
        double s2 = 0;
        for (size_t i = 0; i + 2 < sink.v.size(); i += 3)
            for (int k = 0; k < 3; ++k) {
                const long long a = std::llabs((long long)sink.v[i + k]);
                if (a > mx[k])
                    mx[k] = a;
                s2 += (double)sink.v[i + k] * (double)sink.v[i + k];
            }
        std::fprintf(stderr, "[CE] geom: %zu deleted verts, |d|max=(%lld,%lld,%lld) rms=%.4f (quantized units)\n", nverts, mx[0], mx[1], mx[2],
                     nverts ? std::sqrt(s2 / (double)(nverts * 3)) : 0.0);
        // 残差三分量的**相关结构**：若强相关，则"旋转到局部标架"（Frenet）确实能降熵；
        // 若不相关，则 Frenet 不是缺口所在，得另找。这是"先验证前提"的便宜做法 ——
        // 比先实现 100 行 Frenet 再发现方向错了要省得多。
        {
            double m[3] = {0, 0, 0};
            for (size_t i2 = 0; i2 + 2 < sink.v.size(); i2 += 3)
                for (int k = 0; k < 3; ++k)
                    m[k] += sink.v[i2 + k];
            for (int k = 0; k < 3; ++k)
                m[k] /= (double)nverts;
            double cov[3][3] = {{0}};
            for (size_t i2 = 0; i2 + 2 < sink.v.size(); i2 += 3) {
                double d[3];
                for (int k = 0; k < 3; ++k)
                    d[k] = sink.v[i2 + k] - m[k];
                for (int a = 0; a < 3; ++a)
                    for (int b = 0; b < 3; ++b)
                        cov[a][b] += d[a] * d[b];
            }
            for (int a = 0; a < 3; ++a)
                for (int b = 0; b < 3; ++b)
                    cov[a][b] /= (double)nverts;
            const double sd0 = std::sqrt(cov[0][0]), sd1 = std::sqrt(cov[1][1]), sd2 = std::sqrt(cov[2][2]);
            std::fprintf(stderr, "[CE] geom 标准差=(%.3f,%.3f,%.3f) 相关系数=(%.3f,%.3f,%.3f)\n", sd0, sd1, sd2,
                         (sd0 * sd1 > 0) ? cov[0][1] / (sd0 * sd1) : 0.0, (sd0 * sd2 > 0) ? cov[0][2] / (sd0 * sd2) : 0.0,
                         (sd1 * sd2 > 0) ? cov[1][2] / (sd1 * sd2) : 0.0);
        }
    }

    // ---- 几何残差的熵编码（range/offset 两遍式的**简化版**）----
    // 逐分量取 (min,max)，编码 (v - min)。这就是"range/offset"的骨架：
    // 先确定该分量的动态范围（range），再编码相对偏移（offset）。
    // ⚠️ 这是**骨架**，不是 oracle 的两遍式（它还有跨分量的联合建模）。
    if (std::getenv("PMSF_CE_ENTROPY") && !sink.v.empty()) {
        const size_t nv = sink.v.size() / 3;
        size_t totalBytes = 0;
        for (int k = 0; k < 3; ++k) {
            int lo = 0, hi = 0;
            for (size_t i = 0; i + 2 < sink.v.size(); i += 3) {
                lo = std::min(lo, sink.v[i + k]);
                hi = std::max(hi, sink.v[i + k]);
            }
            const int span = hi - lo + 1;
            int bits = 1;
            while ((1 << bits) < span)
                ++bits;
            std::vector<std::uint32_t> u;
            u.reserve(nv);
            for (size_t i = 0; i + 2 < sink.v.size(); i += 3)
                u.push_back((std::uint32_t)(sink.v[i + k] - lo));
            pmsf::RangeCoderEntropyCoder c(bits);
            std::vector<std::uint8_t> b;
            c.encode(u, b);
            std::vector<std::uint32_t> bak;
            c.decode(b.data(), b.size(), u.size(), bak);
            bool ok = (bak.size() == u.size());
            for (size_t i = 0; ok && i < bak.size(); ++i)
                ok = (bak[i] == u[i]);
            totalBytes += b.size();
            std::fprintf(stderr, "  [geom comp %d] range=[%d,%d] span=%d alphabet=%d bit  %zu syms -> %zu B  往返=%s\n", k, lo, hi, span, bits,
                         u.size(), b.size(), ok ? "一致" : "**不一致**");
        }
        std::fprintf(stderr, "[CE] entropy(几何/全局range): %zu B  bpv=%.4f (V0=6475)\n", totalBytes, 8.0 * (double)totalBytes / 6475.0);

        // ---- 逐层 range/offset（oracle 的结构）vs 全局 range ----
        // oracle 每层写 4 个定长头（Alpha_range/offset, Gamma_range/offset，各 Qbit+1 bit），
        // 然后用**该层实际 range** 做字母表。全局 range 会被大残差的层拖大，
        // 让早期小残差的层白白多花比特。
        layerMarks.push_back(sink.v.size());  // 收尾边界
        std::vector<uint32_t> a[3];
        size_t hdrBytes = 0, pow2Bytes = 0, exactBytes = 0;
        const int qbits = std::getenv("PMSF_CE_QBITS") ? std::atoi(std::getenv("PMSF_CE_QBITS")) : 10;
        for (size_t li = 0; li + 1 < layerMarks.size(); ++li) {
            const size_t lo = layerMarks[li], hi = layerMarks[li + 1];
            if (hi <= lo)
                continue;
            for (int k = 0; k < 3; ++k) {
                int mn = 0, mx = 0;
                for (size_t i = lo; i + 2 < hi; i += 3) {
                    mn = std::min(mn, sink.v[i + k]);
                    mx = std::max(mx, sink.v[i + k]);
                }
                const int span = std::max(2, mx - mn + 1);
                int bits = 1;
                while ((1 << bits) < span)
                    ++bits;
                // 4 个定长头 / 层（本简化版每分量 2 个：range+offset），各 (qbits+1) bit
                hdrBytes += (size_t)((2 * (qbits + 1) + 7) / 8);
                for (size_t i = lo; i + 2 < hi; i += 3)
                    a[k].push_back((uint32_t)(sink.v[i + k] - mn));
                // 两种字母表各编一遍，比较：
                //   (a) 2 的幂（`1<<bits`）—— 框架 S2 原来的唯一选项
                //   (b) **精确** span —— oracle 的做法（qsmodel 本就支持任意 n）
                std::vector<std::uint8_t> bb;
                {
                    pmsf::RangeCoderEntropyCoder c(bits);
                    std::vector<std::uint32_t> seg(a[k].end() - (hi - lo) / 3, a[k].end());
                    c.encode(seg, bb);
                }
                pow2Bytes += bb.size();
                bb.clear();
                {
                    pmsf::RangeCoderEntropyCoder c(span, pmsf::RangeCoderEntropyCoder::ExactAlphabet{});
                    std::vector<std::uint32_t> seg(a[k].end() - (hi - lo) / 3, a[k].end());
                    c.encode(seg, bb);
                }
                exactBytes += bb.size();
                a[k].erase(a[k].end() - (hi - lo) / 3, a[k].end());
            }
        }
        const size_t hdrOnly = hdrBytes;
        std::fprintf(stderr, "[CE] entropy(几何/逐层+2的幂字母表): %zu B (含头)  bpv=%.4f\n", hdrOnly + pow2Bytes,
                     8.0 * (double)(hdrOnly + pow2Bytes) / 6475.0);
        std::fprintf(stderr, "[CE] entropy(几何/逐层+**精确**字母表): %zu B (含头, 省 %zu B)  bpv=%.4f\n", hdrOnly + exactBytes,
                     (long long)(pow2Bytes - exactBytes), 8.0 * (double)(hdrOnly + exactBytes) / 6475.0);
    }

    // ---- 熵编码：接框架的 range coder，量连通性 bpv ----
    // 字母表 {0,1,2,3,4} ⇒ 3 bit。oracle 的连通性 bpv = 4.10903 b/v
    // （P13_1 文档 §2），可用于对拍"迁移是否捕获了该族能力"。
    if (std::getenv("PMSF_CE_ENTROPY")) {
        std::vector<std::uint32_t> symu(syms.begin(), syms.end());
        pmsf::RangeCoderEntropyCoder coder(3);
        std::vector<std::uint8_t> bytes;
        coder.encode(symu, bytes);
        // 往返自检：解回来必须逐个相同
        std::vector<std::uint32_t> back;
        coder.decode(bytes.data(), bytes.size(), symu.size(), back);
        bool rt = (back.size() == symu.size());
        for (size_t i = 0; rt && i < back.size(); ++i)
            if (back[i] != symu[i])
                rt = false;
        const int v0 = 6475;  // 输入顶点数（用于 bpv 口径）
        std::fprintf(stderr,
                     "[CE] entropy(混合): %zu symbols -> %zu bytes  %.4f bit/sym  "
                     "bpv=%.4f (V0=%d)  往返=%s\n",
                     symu.size(), bytes.size(), symu.empty() ? 0.0 : 8.0 * (double)bytes.size() / (double)symu.size(),
                     8.0 * (double)bytes.size() / (double)v0, v0, rt ? "一致" : "**不一致**");

        // 分族编码：删减层用 5 元字母表、清理层用 2 元字母表（与 oracle 一致）
        auto enc = [](const std::vector<int>& src, int bits) -> std::pair<size_t, bool> {
            if (src.empty())
                return {0, true};
            std::vector<std::uint32_t> u(src.begin(), src.end());
            pmsf::RangeCoderEntropyCoder c(bits);
            std::vector<std::uint8_t> b;
            c.encode(u, b);
            std::vector<std::uint32_t> bak;
            c.decode(b.data(), b.size(), u.size(), bak);
            bool ok = (bak.size() == u.size());
            for (size_t i = 0; ok && i < bak.size(); ++i)
                ok = (bak[i] == u[i]);
            return {b.size(), ok};
        };
        auto [bd, okd] = enc(symsDec, 3);
        auto [br, okr] = enc(symsReg, 1);
        const size_t tot = bd + br;
        std::fprintf(stderr,
                     "[CE] entropy(分族): dec %zu syms -> %zu B (3bit), reg %zu syms -> %zu B (1bit) "
                     "= %zu B  bpv=%.4f  往返=%s\n",
                     symsDec.size(), bd, symsReg.size(), br, tot, 8.0 * (double)tot / (double)v0, (okd && okr) ? "一致" : "**不一致**");
    }
    return 0;
}
