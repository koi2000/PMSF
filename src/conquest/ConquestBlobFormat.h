/*
 * @file ConquestBlobFormat.h
 * @brief valence-conquest 连通性流的**字节布局**（编码端与解码端的唯一来源）。
 *
 * 布局（**小端**，显式逐字节写，不依赖宿主字节序）—— **version 2**：
 *
 *   [magic 'C''Q''S''T'][u16 version=2][u16 flags]
 *   [f64 step][f64 bboxMin[3]]                 ← 量化网格，**随流走**
 *   [u32 numLayers]
 *   每层：
 *     [f64 seedPos[2][3]]                        ← 本层种子（tgt(seed) 与 tgt(opp(seed))）
 *     [u32 numSymbols][u32 symBytes][symBytes B]   ← 连通性，3-bit 字母表 range-coded
 *     [u32 numDeltas]                              ← 几何残差，逐分量 range-coded
 *       分量 k=0..2 各：[i32 lo_k][u32 span_k][u32 bytes_k][bytes_k B]
 *
 * **v1 → v2 的变更**（v38-d 实测：v1 原始 int32 全靠外层 zstd，同运行点
 * payload 20.7 KB vs 流级 range-coded 13.2 KB）：symbols 与 deltas 在 blob
 * 内部做熵编码——symbols 走 `RangeCoderEntropyCoder(3)`（字母表 {0..4}，
 * P13.3 往返已验证）；deltas 逐层逐分量取 (lo, hi)，编码 v−lo，字母表用
 * 精确 span（非 2 的幂不补齐，P13.3 实测只省 67 B 但语义更干净）。
 * 编码在 writeStream 内完成（先编码进内存缓冲，超 cap 返回 -1）。
 *
 * **为什么量化网格要写进流里**（而不是从 metadata 取）：蓝图 §54 的教训 ——
 * 量化器的定义（`bboxMin` + `step`）只要经过一次有损中间表示，
 * 解码端算出的格号就会在边界上翻转，而症状看起来像"被测对象错了"。
 * 双精度写读是免费的，这个坑不值得再踩一次。
 *
 * **为什么几何残差按层独立**：每一层的删点集合不同，残差与层号一一对应；
 * 解码端逐层消费，层边界就是"符号耗尽"。
 */
#ifndef PMSF_CONQUEST_CONQUEST_BLOB_FORMAT_H
#define PMSF_CONQUEST_CONQUEST_BLOB_FORMAT_H

#include "entropy/RangeCoderEntropyCoder.h"
#include "entropy/SignedBitplaneCoder.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace pmsf {
namespace conquest_blob {

inline constexpr char kMagic[4] = {'C', 'Q', 'S', 'T'};
inline constexpr std::uint16_t kVersion = 2;
inline constexpr std::uint16_t kBitplaneVersion = 3;

/// @brief 一层的符号与几何残差（= 编码端一次 `decimateLayer` 的全部输出）。
///
/// **种子按坐标记，不按 poolId**：编码端与解码端的 `MCGAL` 池号**不可比**
/// （解码端从快照重建，池号是新的）。探针阶段已实测：靠坐标 `nearestVertex`
/// 找回顶点、再 `halfedgeBetween(b, a)` 取回半边，是唯一走得通的那条路
/// （传成 `(a, b)` 会拿到反向半边 ⇒ PLUS/MINUS 互换 ⇒ 全线分岔）。
struct LayerRecord {
    /// 层种子：`seedPos[0] = tgt(seed)` 的坐标，`seedPos[1] = tgt(opp(seed))`。
    double seedPos[2][3] = {{0, 0, 0}, {0, 0, 0}};
    std::vector<int> symbols;
    std::vector<std::array<int, 3>> deltas;
};

/// @brief 一条完整的流。
struct Stream {
    double step = 0.0;
    double bboxMin[3] = {0.0, 0.0, 0.0};
    std::vector<LayerRecord> layers;
    bool signedBitplanes = false;
    signed_bitplane::Context bitplaneContext = signed_bitplane::Context::Magnitude;

    bool gridValid() const {
        return step > 0.0;
    }
};

namespace detail {

inline void putU16(char*& p, std::uint16_t v) {
    *p++ = (char)(v & 0xff);
    *p++ = (char)((v >> 8) & 0xff);
}
inline void putU32(char*& p, std::uint32_t v) {
    for (int i = 0; i < 4; ++i)
        *p++ = (char)((v >> (8 * i)) & 0xff);
}
inline void putI32(char*& p, std::int32_t v) {
    putU32(p, (std::uint32_t)v);
}
inline void putF64(char*& p, double v) {
    std::uint64_t b = 0;
    std::memcpy(&b, &v, sizeof(b));
    for (int i = 0; i < 8; ++i)
        *p++ = (char)((b >> (8 * i)) & 0xff);
}

struct Reader {
    const char* p = nullptr;
    const char* end = nullptr;
    bool bad = false;

    bool need(std::size_t n) {
        if (bad || (std::size_t)(end - p) < n) {
            bad = true;
            return false;
        }
        return true;
    }
    std::uint16_t u16() {
        if (!need(2))
            return 0;
        const std::uint16_t v = (std::uint8_t)p[0] | ((std::uint16_t)(std::uint8_t)p[1] << 8);
        p += 2;
        return v;
    }
    std::uint32_t u32() {
        if (!need(4))
            return 0;
        std::uint32_t v = 0;
        for (int i = 0; i < 4; ++i)
            v |= (std::uint32_t)(std::uint8_t)p[i] << (8 * i);
        p += 4;
        return v;
    }
    std::int32_t i32() {
        return (std::int32_t)u32();
    }
    double f64() {
        if (!need(8))
            return 0.0;
        std::uint64_t b = 0;
        for (int i = 0; i < 8; ++i)
            b |= (std::uint64_t)(std::uint8_t)p[i] << (8 * i);
        p += 8;
        double v = 0.0;
        std::memcpy(&v, &b, sizeof(v));
        return v;
    }
};

/// @brief 流的总字节数上界（**未编码的原始大小**，仅用于粗略预检；
///         v2 的真实大小取决于熵编码结果，以 writeStream 返回值为准）。
inline std::size_t serializedSize(const Stream& s) {
    std::size_t n = 4 + 2 + 2 + 8 * 4 + 4;
    for (const auto& L : s.layers)
        n += 8 * 6 + 4 + 4 + 4 * L.symbols.size() + 4 + 3 * (4 + 4 + 4 * L.deltas.size());
    return n;
}

// symbols：3-bit 字母表 {0..4}（引擎输出），值在编码器内 clamp。
inline std::vector<std::uint8_t> codeSymbols(const std::vector<int>& symbols) {
    std::vector<std::uint32_t> u(symbols.begin(), symbols.end());
    pmsf::RangeCoderEntropyCoder coder(3);
    std::vector<std::uint8_t> out;
    coder.encode(u, out);
    return out;
}

inline std::vector<int> decodeSymbols(const std::uint8_t* bytes, std::size_t n, std::size_t count) {
    pmsf::RangeCoderEntropyCoder coder(3);
    std::vector<std::uint32_t> u;
    coder.decode(bytes, n, count, u);
    return std::vector<int>(u.begin(), u.end());
}

// 一个分量的 deltas：编码 v−lo。span 精确字母表；span 过大（>65536）时
// 退回 2 的幂 bits 模式。返回编码字节。
inline std::vector<std::uint8_t> codeComponent(const std::vector<std::array<int, 3>>& deltas,
                                               int k, std::int32_t* loOut, std::uint32_t* spanOut) {
    std::int32_t lo = 0, hi = 0;
    if (!deltas.empty()) {
        lo = hi = deltas[0][k];
        for (const auto& d : deltas) {
            lo = std::min(lo, d[k]);
            hi = std::max(hi, d[k]);
        }
    }
    *loOut = lo;
    *spanOut = (std::uint32_t)((std::int64_t)hi - lo + 1);
    const std::int64_t span = (std::int64_t)hi - lo + 1;
    std::vector<std::uint32_t> u;
    u.reserve(deltas.size());
    for (const auto& d : deltas)
        u.push_back((std::uint32_t)((std::int64_t)d[k] - lo));
    std::vector<std::uint8_t> out;
    // qsmodel 约束：lg_totf = max(10, bits) = 10 时初始频数 initval =
    // (1<<lg_totf)/n，**n > 1<<lg_totf ⇒ initval = 0 ⇒ 尾部符号零频率 ⇒
    // encode_shift 段错**（span=1557 实测复现）。精确字母表仅在
    // span ≤ 1<<10 = 1024 时安全；更大走 bits 模式（1<<bits ≥ span 恒安全）。
    if (span <= (1 << 10) && span >= 2) {
        pmsf::RangeCoderEntropyCoder coder((int)span, pmsf::RangeCoderEntropyCoder::ExactAlphabet{});
        coder.encode(u, out);
    } else {
        int bits = 1;
        while (bits < 32 && ((std::int64_t)1 << bits) < span)
            ++bits;
        pmsf::RangeCoderEntropyCoder coder(bits);
        coder.encode(u, out);
    }
    return out;
}

inline std::vector<int> decodeComponent(const std::uint8_t* bytes, std::size_t n, std::size_t count,
                                        std::int32_t lo, std::uint32_t span) {
    std::vector<std::uint32_t> u;
    if ((std::int64_t)span <= (1 << 10) && span >= 2) {
        pmsf::RangeCoderEntropyCoder coder((int)span, pmsf::RangeCoderEntropyCoder::ExactAlphabet{});
        coder.decode(bytes, n, count, u);
    } else {
        int bits = 1;
        while (bits < 32 && ((std::int64_t)1 << bits) < span)
            ++bits;
        pmsf::RangeCoderEntropyCoder coder(bits);
        coder.decode(bytes, n, count, u);
    }
    std::vector<int> out(count);
    for (std::size_t i = 0; i < count; ++i)
        out[i] = (int)((std::int64_t)lo + (std::int64_t)u[i]);
    return out;
}

}  // namespace detail

/// @brief 序列化到 `out`（容量 `cap`）。返回写入字节数；容量不足返回 -1。
/// v2：symbols / deltas 先熵编码进内存缓冲，超 cap 返回 -1（编码成本已付，
/// 调用方拿返回值判断，不得再用 serializedSize 当精确值）。
inline int writeStream(const Stream& s, char* out, std::size_t cap) {
    if (!out)
        return -1;

    std::vector<char> buf;
    buf.reserve(4096);
    auto putBytes = [&](const void* d, std::size_t n) {
        const char* c = (const char*)d;
        buf.insert(buf.end(), c, c + n);
    };
    auto putU32v = [&](std::uint32_t v) {
        char t[4];
        for (int i = 0; i < 4; ++i)
            t[i] = (char)((v >> (8 * i)) & 0xff);
        putBytes(t, 4);
    };
    auto putI32v = [&](std::int32_t v) { putU32v((std::uint32_t)v); };
    auto putF64v = [&](double v) {
        std::uint64_t b = 0;
        std::memcpy(&b, &v, sizeof(b));
        char t[8];
        for (int i = 0; i < 8; ++i)
            t[i] = (char)((b >> (8 * i)) & 0xff);
        putBytes(t, 8);
    };

    putBytes(kMagic, 4);
    const std::uint16_t version = s.signedBitplanes ? kBitplaneVersion : kVersion;
    char u16[2] = {(char)(version & 0xff), (char)((version >> 8) & 0xff)};
    putBytes(u16, 2);
    char z16[2] = {0, 0};
    putBytes(z16, 2);  // flags
    putF64v(s.step);
    for (int k = 0; k < 3; ++k)
        putF64v(s.bboxMin[k]);
    putU32v((std::uint32_t)s.layers.size());
    for (const auto& L : s.layers) {
        for (int a = 0; a < 2; ++a)
            for (int k = 0; k < 3; ++k)
                putF64v(L.seedPos[a][k]);
        // 连通性
        const std::vector<std::uint8_t> sym = detail::codeSymbols(L.symbols);
        putU32v((std::uint32_t)L.symbols.size());
        putU32v((std::uint32_t)sym.size());
        putBytes(sym.data(), sym.size());
        // 几何残差：逐分量 range/offset
        putU32v((std::uint32_t)L.deltas.size());
        if (s.signedBitplanes) {
            for (int axis = 0; axis < 3; ++axis) {
                std::vector<std::int32_t> values;
                values.reserve(L.deltas.size());
                for (const auto& delta : L.deltas) values.push_back(delta[axis]);
                const auto block = signed_bitplane::encode(values, s.bitplaneContext);
                putU32v(static_cast<std::uint32_t>(block.size()));
                putBytes(block.data(), block.size());
            }
        } else {
        for (int k = 0; k < 3; ++k) {
            std::int32_t lo = 0;
            std::uint32_t span = 0;
            const std::vector<std::uint8_t> comp = detail::codeComponent(L.deltas, k, &lo, &span);
            putI32v(lo);
            putU32v(span);
            putU32v((std::uint32_t)comp.size());
            putBytes(comp.data(), comp.size());
        }
        }
    }
    if (buf.size() > cap)
        return -1;
    std::memcpy(out, buf.data(), buf.size());
    return (int)buf.size();
}

/// @brief 从 `in[0..size)` 反序列化（v2，熵编码）。成功返回 true；magic/版本/长度不符返回 false。
inline bool readStream(const char* in, std::size_t size, Stream* out) {
    if (!in || !out || size < 4 + 2 + 2 + 8 * 4 + 4)
        return false;
    for (int i = 0; i < 4; ++i)
        if (in[i] != kMagic[i])
            return false;

    detail::Reader r;
    r.p = in + 4;
    r.end = in + size;
    const std::uint16_t ver = r.u16();
    if (ver != kVersion && ver != kBitplaneVersion)
        return false;
    (void)r.u16();  // flags，本版恒 0

    Stream s;
    s.signedBitplanes = ver == kBitplaneVersion;
    s.step = r.f64();
    for (int k = 0; k < 3; ++k)
        s.bboxMin[k] = r.f64();
    const std::uint32_t numLayers = r.u32();
    if (r.bad)
        return false;
    // 防御：每层至少 seedPos 48 字节，层数不可能超过剩余字节数
    if ((std::size_t)numLayers * 48 > size)
        return false;

    s.layers.resize(numLayers);
    for (auto& L : s.layers) {
        for (int a = 0; a < 2; ++a)
            for (int k = 0; k < 3; ++k)
                L.seedPos[a][k] = r.f64();
        if (r.bad)
            return false;
        const std::uint32_t ns = r.u32();
        const std::uint32_t nsBytes = r.u32();
        if (r.bad || (std::size_t)nsBytes > (std::size_t)(r.end - r.p))
            return false;
        L.symbols = detail::decodeSymbols((const std::uint8_t*)r.p, nsBytes, ns);
        r.p += nsBytes;
        const std::uint32_t nd = r.u32();
        if (r.bad)
            return false;
        std::vector<int> comp[3];
        if (s.signedBitplanes) {
            for (int axis = 0; axis < 3; ++axis) {
                const std::uint32_t nBytes = r.u32();
                if (r.bad || nBytes > static_cast<std::size_t>(r.end - r.p)) return false;
                try {
                    const auto values = signed_bitplane::decode(reinterpret_cast<const std::uint8_t*>(r.p), nBytes, nd);
                    comp[axis].assign(values.begin(), values.end());
                    if (nBytes > 9) s.bitplaneContext = static_cast<signed_bitplane::Context>(static_cast<std::uint8_t>(r.p[9]));
                } catch (const std::exception&) { return false; }
                r.p += nBytes;
            }
        } else {
        for (int k = 0; k < 3; ++k) {
            const std::int32_t lo = r.i32();
            const std::uint32_t span = r.u32();
            const std::uint32_t nBytes = r.u32();
            if (r.bad || (std::size_t)nBytes > (std::size_t)(r.end - r.p))
                return false;
            comp[k] = detail::decodeComponent((const std::uint8_t*)r.p, nBytes, nd, lo, span);
            r.p += nBytes;
        }
        }
        if (r.bad)
            return false;
        L.deltas.resize(nd);
        for (std::uint32_t i = 0; i < nd; ++i) {
            L.deltas[i][0] = comp[0][i];
            L.deltas[i][1] = comp[1][i];
            L.deltas[i][2] = comp[2][i];
        }
    }
    if (r.bad)
        return false;
    *out = std::move(s);
    return true;
}

}  // namespace conquest_blob
}  // namespace pmsf

#endif  // PMSF_CONQUEST_CONQUEST_BLOB_FORMAT_H
