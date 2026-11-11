// SPDX-License-Identifier: Apache-2.0
//
// RDCanonicalBlobFormat.h — task-43 route A 的 sv=71 三流字节布局（编码端/解码端
// **同一个头文件**，保证 framing 与 range-coder 模型逐字互逆）。
//
// 容器：`pmsf::ConnectivityBlobBuilder(pmsf::rdTagConnectivityBlobSpec(
//         numRounds, /*schemeVersion=*/71, /*streamCount=*/3))`
//   stream 0 = valence/null 符号（每层一条 RC 段）
//   stream 1 = residual（Frenet lifting 后的整数三元组，每层一条 RC 段）
//   stream 2 = 本层 seed（tgt(seed) / tgt(opp(seed)) 的量化格号，各 3×int32）
//
// 每个 stream 的载荷 framing：
//   stream0: [int32 nSyms][RC bytes]        （nSyms==0 时整个 stream 为空）
//   stream1: [int32 nResiduals][u8 bx][u8 by][u8 bz][u8 dropBits][RC bytes]
//   stream2: [int32 hasSeed][int32 seedQ0x..z][int32 seedQ1x..z]
//
// 符号字母表（stream0）：0..3 = 被删 front vertex 的 valence-3（3..6），
// 4 = null patch。RC：5 元 adaptive qsmodel × 4 个上下文（按上一个符号分组）。
// 残差 RC：自适应位宽 + bit-plane-major（与 PCLTTag sv=3/`RDPatch sv=7` 同一模型族：
// 每 (component,bit) 一个 firstOne/afterOne 二元 qsmodel）。默认参数
// lg_totf=10 / rescale=1<<9，两端相同。
#ifndef PMSF_IMPL_RD_RD_CANONICAL_BLOB_FORMAT_H
#define PMSF_IMPL_RD_RD_CANONICAL_BLOB_FORMAT_H

#include "Point.h"

#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"

#include <cstdint>
#include <cstring>
#include <span>
#include <stdexcept>
#include <vector>

namespace rd_canon {

constexpr int kCanonBlobSchemeVersion = 71;
constexpr int kCanonStreamSymbols = 0;
constexpr int kCanonStreamResiduals = 1;
constexpr int kCanonStreamSeed = 2;
constexpr int kCanonStreamCount = 3;

// ---- stream 0: valence/null 符号 -------------------------------------------
inline int symbolCtx(int prev) {
    if (prev < 0) return 0;
    if (prev == 0 || prev == 1) return 1;  // VAL_3 / VAL_4
    if (prev == 2 || prev == 3) return 2;  // VAL_5 / VAL_6
    return 3;                              // null
}

inline void encodeSymbolStream(const std::vector<int>& syms,
                               std::vector<std::uint8_t>& out) {
    out.clear();
    if (syms.empty()) return;
    const std::size_t cap = syms.size() * 4 + 4096;
    std::vector<char> buf(cap, 0);
    size_t off = 0;
    rangecoder rc;
    rc.p_data = buf.data();
    rc.p_dataOffset = &off;
    start_encoding(&rc, 0, 0);
    qsmodel ms[4];
    for (int i = 0; i < 4; ++i) initqsmodel(&ms[i], 5, 10, 1 << 9, nullptr, 1);
    int prev = -1;
    for (int sym : syms) {
        int s = sym;
        if (s < 0 || s > 4) s = 4;
        qsmodel* mp = &ms[symbolCtx(prev)];
        int sf = 0, lf = 0;
        qsgetfreq(mp, s, &sf, &lf);
        encode_shift(&rc, sf, lf, 10);
        qsupdate(mp, s);
        prev = s;
    }
    const std::size_t rcSize = done_encoding(&rc);
    for (auto& m : ms) deleteqsmodel(&m);
    out.resize(4 + rcSize);
    const int32_t n = static_cast<int32_t>(syms.size());
    std::memcpy(out.data(), &n, 4);
    if (rcSize > 0) std::memcpy(out.data() + 4, buf.data(), rcSize);
}

inline void decodeSymbolStream(std::span<const std::uint8_t> sl,
                               std::vector<int>& syms) {
    syms.clear();
    if (sl.empty()) return;
    if (sl.size() < 4) throw std::runtime_error("[RDCanon sv=71] symbol stream too small");
    int32_t n = 0;
    std::memcpy(&n, sl.data(), 4);
    if (n <= 0) return;
    std::vector<char> staging(sl.begin() + 4, sl.end());
    size_t off = 0;
    rangecoder rc;
    rc.p_data = staging.data();
    rc.p_dataOffset = &off;
    if (start_decoding(&rc) != 0)
        throw std::runtime_error("[RDCanon sv=71] symbol start_decoding failed");
    qsmodel ms[4];
    for (int i = 0; i < 4; ++i) initqsmodel(&ms[i], 5, 10, 1 << 9, nullptr, 0);
    syms.reserve(static_cast<std::size_t>(n));
    int prev = -1;
    for (int32_t k = 0; k < n; ++k) {
        qsmodel* mp = &ms[symbolCtx(prev)];
        int ltval = decode_culshift(&rc, 10);
        int sym = qsgetsym(mp, ltval);
        int sf = 0, lf = 0;
        qsgetfreq(mp, sym, &sf, &lf);
        decode_update(&rc, sf, lf, 1 << 10);
        qsupdate(mp, sym);
        if (sym < 0 || sym > 4) sym = 4;
        syms.push_back(sym);
        prev = sym;
    }
    done_decoding(&rc);
    for (auto& m : ms) deleteqsmodel(&m);
}

// ---- stream 1: residual ----------------------------------------------------
inline int zigzagWidth(const std::vector<int32_t>& vals) {
    uint32_t maxZ = 0;
    for (int32_t r : vals) {
        const uint32_t z = (static_cast<uint32_t>(r) << 1) ^ static_cast<uint32_t>(r >> 31);
        if (z > maxZ) maxZ = z;
    }
    int bits = 0;
    while (maxZ != 0) {
        ++bits;
        maxZ >>= 1;
    }
    return bits > 31 ? 31 : bits;
}

inline void encodeResidualStream(const std::vector<MCGAL::PointInt>& residuals,
                                 std::vector<std::uint8_t>& out) {
    out.clear();
    const std::size_t N = residuals.size();
    if (N == 0) return;
    std::vector<int32_t> comp[3];
    for (int a = 0; a < 3; ++a) comp[a].reserve(N);
    for (const auto& r : residuals) {
        comp[0].push_back(static_cast<int32_t>(r.x()));
        comp[1].push_back(static_cast<int32_t>(r.y()));
        comp[2].push_back(static_cast<int32_t>(r.z()));
    }
    int bits[3] = {zigzagWidth(comp[0]), zigzagWidth(comp[1]), zigzagWidth(comp[2])};
    const int offsets[3] = {0, bits[0], bits[0] + bits[1]};
    const int numCtx = bits[0] + bits[1] + bits[2];
    std::vector<std::uint8_t> outHead(8, 0);
    const int32_t n32 = static_cast<int32_t>(N);
    std::memcpy(outHead.data(), &n32, 4);
    outHead[4] = static_cast<std::uint8_t>(bits[0]);
    outHead[5] = static_cast<std::uint8_t>(bits[1]);
    outHead[6] = static_cast<std::uint8_t>(bits[2]);
    outHead[7] = 0;  // dropBits（route A 不丢位）
    if (numCtx == 0) {
        out = std::move(outHead);
        return;
    }
    const std::size_t cap = N * 12 + 65536;
    std::vector<char> buf(cap, 0);
    size_t off = 0;
    rangecoder rc;
    rc.p_data = buf.data();
    rc.p_dataOffset = &off;
    start_encoding(&rc, 0, 0);
    std::vector<qsmodel> bpFirst1(numCtx), bpAfter1(numCtx);
    for (int i = 0; i < numCtx; ++i) {
        initqsmodel(&bpFirst1[i], 2, 10, 1 << 9, nullptr, 1);
        initqsmodel(&bpAfter1[i], 2, 10, 1 << 9, nullptr, 1);
    }
    for (int a = 0; a < 3; ++a) {
        const int width = bits[a];
        if (width == 0) continue;
        std::vector<unsigned char> seen(N, 0);
        for (int b = width - 1; b >= 0; --b) {
            const int mi = offsets[a] + b;
            for (std::size_t i = 0; i < N; ++i) {
                const uint32_t z = (static_cast<uint32_t>(comp[a][i]) << 1) ^
                                   static_cast<uint32_t>(comp[a][i] >> 31);
                const int bit = static_cast<int>((z >> b) & 1u);
                qsmodel* mp = seen[i] ? &bpAfter1[mi] : &bpFirst1[mi];
                int sf = 0, lf = 0;
                qsgetfreq(mp, bit, &sf, &lf);
                encode_shift(&rc, sf, lf, 10);
                qsupdate(mp, bit);
                if (bit) seen[i] = 1;
            }
        }
    }
    const std::size_t rcSize = done_encoding(&rc);
    for (auto& m : bpFirst1) deleteqsmodel(&m);
    for (auto& m : bpAfter1) deleteqsmodel(&m);
    out = std::move(outHead);
    out.resize(8 + rcSize);
    if (rcSize > 0) std::memcpy(out.data() + 8, buf.data(), rcSize);
}

inline void decodeResidualStream(std::span<const std::uint8_t> sl,
                                 std::vector<MCGAL::PointInt>& residuals) {
    residuals.clear();
    if (sl.empty()) return;
    if (sl.size() < 8) throw std::runtime_error("[RDCanon sv=71] residual stream too small");
    int32_t N = 0;
    std::memcpy(&N, sl.data(), 4);
    if (N <= 0) return;
    const int bits[3] = {sl[4], sl[5], sl[6]};
    const int offsets[3] = {0, bits[0], bits[0] + bits[1]};
    const int numCtx = bits[0] + bits[1] + bits[2];
    residuals.reserve(static_cast<std::size_t>(N));
    if (numCtx == 0) {
        for (int32_t i = 0; i < N; ++i) residuals.push_back(MCGAL::PointInt(0, 0, 0));
        return;
    }
    std::vector<char> staging(sl.begin() + 8, sl.end());
    size_t off = 0;
    rangecoder rc;
    rc.p_data = staging.data();
    rc.p_dataOffset = &off;
    if (start_decoding(&rc) != 0)
        throw std::runtime_error("[RDCanon sv=71] residual start_decoding failed");
    std::vector<qsmodel> bpFirst1(numCtx), bpAfter1(numCtx);
    for (int i = 0; i < numCtx; ++i) {
        initqsmodel(&bpFirst1[i], 2, 10, 1 << 9, nullptr, 0);
        initqsmodel(&bpAfter1[i], 2, 10, 1 << 9, nullptr, 0);
    }
    std::vector<std::vector<int32_t>> comp(3, std::vector<int32_t>(static_cast<std::size_t>(N), 0));
    for (int a = 0; a < 3; ++a) {
        const int width = bits[a];
        if (width == 0) continue;
        std::vector<unsigned char> seen(static_cast<std::size_t>(N), 0);
        for (int b = width - 1; b >= 0; --b) {
            const int mi = offsets[a] + b;
            for (std::size_t i = 0; i < static_cast<std::size_t>(N); ++i) {
                qsmodel* mp = seen[i] ? &bpAfter1[mi] : &bpFirst1[mi];
                int ltval = decode_culshift(&rc, 10);
                int sym = qsgetsym(mp, ltval);
                int sf = 0, lf = 0;
                qsgetfreq(mp, sym, &sf, &lf);
                decode_update(&rc, sf, lf, 1 << 10);
                qsupdate(mp, sym);
                if (sym) {
                    comp[a][i] = static_cast<int32_t>(static_cast<uint32_t>(comp[a][i]) |
                                                      (1u << b));
                    seen[i] = 1;
                }
            }
        }
    }
    done_decoding(&rc);
    for (auto& m : bpFirst1) deleteqsmodel(&m);
    for (auto& m : bpAfter1) deleteqsmodel(&m);
    for (int32_t i = 0; i < N; ++i) {
        auto unzigzag = [](uint32_t z) -> int32_t {
            return static_cast<int32_t>(z >> 1) ^ -static_cast<int32_t>(z & 1u);
        };
        residuals.push_back(MCGAL::PointInt(unzigzag(static_cast<uint32_t>(comp[0][i])),
                                            unzigzag(static_cast<uint32_t>(comp[1][i])),
                                            unzigzag(static_cast<uint32_t>(comp[2][i]))));
    }
}

// ---- stream 2: per-layer seed ---------------------------------------------
inline void encodeSeedStream(bool hasSeed, const int q0[3], const int q1[3],
                             std::vector<std::uint8_t>& out) {
    out.clear();
    const int32_t has = hasSeed ? 1 : 0;
    out.resize(4 + (hasSeed ? 24 : 0));
    std::memcpy(out.data(), &has, 4);
    if (hasSeed) {
        std::memcpy(out.data() + 4, q0, 12);
        std::memcpy(out.data() + 16, q1, 12);
    }
}

inline bool decodeSeedStream(std::span<const std::uint8_t> sl, int q0[3], int q1[3]) {
    if (sl.size() < 4) return false;
    int32_t has = 0;
    std::memcpy(&has, sl.data(), 4);
    if (has == 0) return false;
    if (sl.size() < 28) return false;
    std::memcpy(q0, sl.data() + 4, 12);
    std::memcpy(q1, sl.data() + 16, 12);
    return true;
}

}  // namespace rd_canon

#endif  // PMSF_IMPL_RD_RD_CANONICAL_BLOB_FORMAT_H
