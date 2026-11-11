#ifndef PMSF_STORAGE_PAPER_COMPACT_BIN_ENVELOPE_H
#define PMSF_STORAGE_PAPER_COMPACT_BIN_ENVELOPE_H

#include <zlib.h>

#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

#ifdef PMSF_HAS_BROTLI
#    include <brotli/decode.h>
#    include <brotli/encode.h>
#endif

namespace pmsf {

struct PMSFCompactBinEnvelope {
    std::uint32_t rawLen = 0;
    std::uint32_t compLen = 0;
    std::vector<std::uint8_t> wrapped;
    std::vector<std::uint8_t> payload;
};

inline std::uint32_t readPaperCompactU32LE(
    const std::vector<std::uint8_t>& blob,
    std::size_t off) {
    return static_cast<std::uint32_t>(blob[off]) |
           (static_cast<std::uint32_t>(blob[off + 1]) << 8) |
           (static_cast<std::uint32_t>(blob[off + 2]) << 16) |
           (static_cast<std::uint32_t>(blob[off + 3]) << 24);
}

inline void appendPaperCompactU32LE(
    std::vector<std::uint8_t>& blob,
    std::uint32_t value) {
    blob.push_back(static_cast<std::uint8_t>(value & 0xffu));
    blob.push_back(static_cast<std::uint8_t>((value >> 8) & 0xffu));
    blob.push_back(static_cast<std::uint8_t>((value >> 16) & 0xffu));
    blob.push_back(static_cast<std::uint8_t>((value >> 24) & 0xffu));
}

// ============================================================================
// 容器级字节压缩后端（**默认关闭，默认路径逐字节不变**）
//
// 背景：Tier-2（`PMSF_V2_COMPACT=1`）下 `_compact.bin` 的 payload 是**基网段**
// 独占的，分区跑点上它是六列表里 Base 列的唯一落盘面。实测 N=128 的基网
// payload 里「面索引」占 70 KB / 100 KB，而 zlib-9 只压掉 ~11%，是全跑点最
// 大的单块可压缩残量；换成 brotli-11 同一 payload 从 89,381 B 降到 79,391 B。
//
// 口径：只换**容器字节压缩器**，不改 payload 的任何语义 —— 量化位深、drop、
// 符号流一律不动，解码结果逐比特相同。默认（未设 `PMSF_COMPACT_CODEC`）仍走
// 原 zlib-9 分支，写出字节与引入本机制前完全一致。
//
// 线格式：默认 `[u32 rawLen][u32 compLen][zlib stream]`。
// 当 `rawLen` 最高位置 1 时表示「扩展后端」，此时 stream 的第一字节是
// codec id，其后才是压缩流；`compLen` 仍等于「第 8 字节之后的总字节数」，
// 因此既有 `8 + compLen == fileSize` 的边界校验全部保持成立。
// 旧解码器读到置位文件会因长度校验失败而拒绝（而不是静默解错）。
// ============================================================================
inline constexpr std::uint32_t kCompactBinExtendedCodecFlag = 0x80000000u;
inline constexpr std::uint8_t kCompactBinCodecZlib = 1;
inline constexpr std::uint8_t kCompactBinCodecBrotli = 2;
/// PHASE62 物品1：**内容编码**（不是容器替换）—— 位级二进制上下文自适应 RC。
///
/// 与 QEM `edgeSym` 上下文化同源：符号序列一个字节都不改，只换承载它的熵编码。
/// 关键设计约束（附 41/42/43 三次失败的教训）：**上下文必须由已解码的前缀唯一
/// 决定**。字节级 256 元模型在这里学不动（每上下文约 390 样本），故改用位级
/// 二值模型；上下文取 `(varint 内位置, 字节内位序, 前一位)`，其中 varint 位置
/// 由「刚编完的那个字节的最高位是否为 1」递推 —— **纯因果，编解码天然同源**。
inline constexpr std::uint8_t kCompactBinCodecBitCtx = 3;

/// 位级上下文状态机。**编解码两侧必须逐位走同一条路径** —— 它同时决定用哪个
/// 模型，一旦分叉解码必错（附 43 的段错误根因就是上下文掺了只有编码端才知道的
/// 结构信息）。本实现只吃**已解码前缀**，故天然对称。
///
/// 上下文 = 四段的笛卡尔积，每段都可关：
///   bitIdx                      总是有（8）
///   prefixBits  当前字节已编前缀的**末** prefixBits 位
///   prevShr     上一完整字节 >> prevShr（prevShr>=8 ⇒ 不用上一字节）
///   vpCap       varint 内位置（截断到 vpCap）
/// 位级而非字节级：字节级 256 元模型在 ~390 样本/上下文下学不动（附 41 实测）。
struct CompactBinBitCtx {
    // ⚠️ 默认值 = N=128 实测最优（见附 48 的扫描表）。**改动必须重跑扫描**：
    // 这三段是可关的笛卡尔积，多一段 ≠ 更好 —— 实测「用上一字节」反而更差
    // （prevShr<8 全部劣于 prevShr=8），因为每格样本数被摊薄。
    int prefixBits = 3;   ///< 0 = 不用当前字节前缀
    int prevShr = 8;      ///< 8 = 不用上一字节
    int vpCap = 3;        ///< 0 = 不用 varint 位置

    int varintPos = 0;
    int bitIdx = 0;
    int highBit = 0;
    int prevByte = 0;
    std::size_t byteIndex = 0;

    int numContexts() const {
        int n = 8;
        if (prefixBits > 0) n *= (1 << prefixBits);
        if (prevShr < 8) n *= (1 << (8 - prevShr));
        if (vpCap > 0) n *= (vpCap + 1);
        return n;
    }

    /// @param prefix 当前字节**已编码**的前缀值（bitIdx 位）
    int ctx(int prefix) const {
        int c = bitIdx;
        if (prefixBits > 0) {
            const int shift = bitIdx > prefixBits ? bitIdx - prefixBits : 0;
            c = c * (1 << prefixBits) + (prefix >> shift);
        }
        if (prevShr < 8) c = c * (1 << (8 - prevShr)) + (prevByte >> prevShr);
        if (vpCap > 0) {
            const int vp = varintPos < vpCap ? varintPos : vpCap;
            c = c * (vpCap + 1) + vp;
        }
        return c;
    }

    /// 编完一位后推进状态（**只吃这一位**，故两侧同源）。
    void push(int bit) {
        if (bitIdx == 0) highBit = bit;
        if (++bitIdx == 8) {
            bitIdx = 0;
            varintPos = highBit ? (varintPos + 1) : 0;
            ++byteIndex;
        }
    }
    void endByte(int byte) { prevByte = byte; }
};

/// 位级上下文自适应编码。返回 RC 字节数（写入 `out`）。
/// @param lgTotf    qsmodel 的 lg_totf（每模型总频 = 1<<lgTotf）
/// @param rescaleLog qsmodel 的 rescale（= 1<<rescaleLog）
inline std::size_t encodeCompactBinBitCtx(const std::vector<std::uint8_t>& payload,
                                          const CompactBinBitCtx& cfg,
                                          int lgTotf,
                                          int rescaleLog,
                                          char* out,
                                          std::size_t maxSize) {
    rangecoder rc;
    std::size_t off = 0;
    rc.p_data = out;
    rc.p_dataOffset = &off;
    start_encoding(&rc, 0, 0);

    CompactBinBitCtx st = cfg;
    const int nctx = st.numContexts();
    std::vector<qsmodel> models(static_cast<std::size_t>(nctx));
    for (int i = 0; i < nctx; ++i) {
        initqsmodel(&models[static_cast<std::size_t>(i)], 2, lgTotf, 1 << rescaleLog, nullptr, 1);
    }

    for (std::uint8_t byte : payload) {
        int prefix = 0;
        for (int b = 7; b >= 0; --b) {
            const int bit = (byte >> b) & 1;
            qsmodel* mp = &models[static_cast<std::size_t>(st.ctx(prefix))];
            int syfreq = 0;
            int ltfreq = 0;
            qsgetfreq(mp, bit, &syfreq, &ltfreq);
            encode_shift(&rc, syfreq, ltfreq, lgTotf);
            qsupdate(mp, bit);
            st.push(bit);
            prefix = (prefix << 1) | bit;
        }
        st.endByte(byte);
    }
    const std::size_t rcSize = done_encoding(&rc);
    for (int i = 0; i < nctx; ++i) {
        deleteqsmodel(&models[static_cast<std::size_t>(i)]);
    }
    (void)maxSize;
    return rcSize;
}

/// `encodeCompactBinBitCtx` 的严格逆。**逐位走同一个 `CompactBinBitCtx`**，
/// 位数由 `rawLen`（已在外层头里）唯一确定 ⇒ 无需额外长度字段。
inline bool decodeCompactBinBitCtx(const char* stream,
                                   std::size_t streamLen,
                                   const CompactBinBitCtx& cfg,
                                   int lgTotf,
                                   int rescaleLog,
                                   std::size_t rawLen,
                                   std::vector<std::uint8_t>& out) {
    // start_decoding() consumes two bytes before any model operation.
    if (stream == nullptr || streamLen < 2) {
        return false;
    }
    std::vector<char> staging(stream, stream + streamLen);
    std::size_t off = 0;
    rangecoder rc{};
    rc.p_data = staging.data();
    rc.p_dataOffset = &off;
    if (start_decoding(&rc) != 0) {
        return false;
    }
    rc.p_dataEnd = staging.data() + staging.size();

    CompactBinBitCtx st = cfg;
    const int nctx = st.numContexts();
    std::vector<qsmodel> models(static_cast<std::size_t>(nctx));
    for (int i = 0; i < nctx; ++i) {
        // 二元模型只需 cf[1] 分隔两个区间，无需 qsgetsym 的通用搜索表。
        // compress=1 仅省掉 search 分配和每次 rescale 时的表重建；cf/newf
        // 初始化及更新与旧解码器完全相同，不改变概率模型或线格式。
        initqsmodel(&models[static_cast<std::size_t>(i)], 2, lgTotf, 1 << rescaleLog, nullptr, 1);
    }

    out.resize(rawLen);
    for (std::size_t i = 0; i < rawLen; ++i) {
        int byte = 0;
        int prefix = 0;
        for (int b = 7; b >= 0; --b) {
            qsmodel* mp = &models[static_cast<std::size_t>(st.ctx(prefix))];
            const int ltval = decode_culshift(&rc, lgTotf);
            const int sym = static_cast<freq>(ltval) >= mp->cf[1] ? 1 : 0;
            int syfreq = 0;
            int ltfreq = 0;
            qsgetfreq(mp, sym, &syfreq, &ltfreq);
            decode_update(&rc, syfreq, ltfreq, 1 << lgTotf);
            qsupdate(mp, sym);
            byte = (byte << 1) | sym;
            st.push(sym);
            prefix = (prefix << 1) | sym;
        }
        st.endByte(byte);
        out[i] = static_cast<std::uint8_t>(byte);
    }
    done_decoding(&rc);
    const bool inputOk = rc.inputError == 0 && off <= streamLen;
    for (int i = 0; i < nctx; ++i) {
        deleteqsmodel(&models[static_cast<std::size_t>(i)]);
    }
    return inputOk;
}

/// 请求的容器压缩后端（`PMSF_COMPACT_CODEC`）：空/zlib ⇒ zlib；
/// brotli/2 ⇒ brotli；既无环境变量又无库时静默回落 zlib。
inline std::uint8_t requestedCompactBinCodec() {
    const char* env = std::getenv("PMSF_COMPACT_CODEC");
    // ⚠️ 空/未设置**不是** zlib —— PHASE62 物品1 起默认是 bitctx（见函数末尾）。
    // 曾经这里提前 `return kCompactBinCodecZlib`，把下面的默认分支变成不可达，
    // 于是「改成默认」静默失效（附 48 记录）。
    if (env != nullptr && env[0] != '\0') {
    if (std::strcmp(env, "brotli") == 0 || std::strcmp(env, "br") == 0 || std::strcmp(env, "2") == 0) {
#ifdef PMSF_HAS_BROTLI
        return kCompactBinCodecBrotli;
#else
        std::cerr << "[CompactBin] PMSF_COMPACT_CODEC=brotli 但本构建未链接 brotli，"
                     "回落 zlib"
                  << std::endl;
        return kCompactBinCodecZlib;
#endif
    }
    if (std::strcmp(env, "bitctx") == 0 || std::strcmp(env, "3") == 0) {
        return kCompactBinCodecBitCtx;
    }
    if (std::strcmp(env, "zlib") == 0 || std::strcmp(env, "deflate") == 0 || std::strcmp(env, "1") == 0) {
        return kCompactBinCodecZlib;
    }
    // ⚠️ PHASE62 物品1：**bitctx 现在是默认**。理由：它是**内容编码**改进
    //（payload 语义一个字节不改，只换承载它的熵编码），不是容器替换；
    // 且实测门禁 34 格 **28 改善 / 6 不变 / 0 变差**（硬约束④「重冻须全改善」）。
    // `PMSF_COMPACT_CODEC=zlib` 回退旧行为，供对照与回退。
    }  // env 非空
    return kCompactBinCodecBitCtx;
}

/// bitctx 的三个模型参数（读数在**编码端**定，但会写进流里 ⇒ 解码端不依赖 env）。
inline int compactBinBitCtxIntParam(const char* name, int dflt) {
    if (const char* v = std::getenv(name)) {
        const int parsed = std::atoi(v);
        if (parsed > 0) return parsed;
    }
    return dflt;
}

/// 把 payload 压成完整的 `<base>_compact.bin` 字节（含 8 字节外层头）。
/// 返回 false 表示压缩器失败（调用方按原样报错退出，不静默降级）。
inline bool compressCompactBinPayloadWithCodec(
    const std::vector<std::uint8_t>& payload,
    std::uint8_t codec,
    std::vector<std::uint8_t>& wrapped) {
    if (codec != kCompactBinCodecZlib && codec != kCompactBinCodecBitCtx &&
        codec != kCompactBinCodecBrotli) {
        return false;
    }
#ifndef PMSF_HAS_BROTLI
    if (codec == kCompactBinCodecBrotli) {
        return false;
    }
#endif
    std::vector<std::uint8_t> compressed;
    if (codec == kCompactBinCodecBitCtx) {
        const int lgTotf = compactBinBitCtxIntParam("PMSF_ORDER1_LG_TOTF", 10);
        const int rescaleLog = compactBinBitCtxIntParam("PMSF_ORDER1_RESCALE_LOG", 5);
        CompactBinBitCtx cfg;
        cfg.prefixBits = compactBinBitCtxIntParam("PMSF_ORDER1_PREFIX_BITS", 3);
        cfg.prevShr = compactBinBitCtxIntParam("PMSF_ORDER1_PREV_SHR", 8);
        cfg.vpCap = compactBinBitCtxIntParam("PMSF_ORDER1_VP_CAP", 3);
        // 最坏情况：每个 bit 花到 lg_totf 位 ⇒ 每字节 lg_totf 字节。
        const std::size_t cap = payload.size() * static_cast<std::size_t>(lgTotf + 1) + 65536;
        std::vector<char> buf(cap);
        const std::size_t rcSize =
            encodeCompactBinBitCtx(payload, cfg, lgTotf, rescaleLog, buf.data(), cap);
        // 自描述：3 个参数进流头，解码端不依赖 env（判据②）。
        compressed.reserve(5 + rcSize);
        compressed.push_back(static_cast<std::uint8_t>(lgTotf));
        compressed.push_back(static_cast<std::uint8_t>(rescaleLog));
        compressed.push_back(static_cast<std::uint8_t>(cfg.prefixBits));
        compressed.push_back(static_cast<std::uint8_t>(cfg.prevShr));
        compressed.push_back(static_cast<std::uint8_t>(cfg.vpCap));
        compressed.insert(compressed.end(),
                          reinterpret_cast<const std::uint8_t*>(buf.data()),
                          reinterpret_cast<const std::uint8_t*>(buf.data()) + rcSize);
    } else if (codec == kCompactBinCodecBrotli) {
#ifdef PMSF_HAS_BROTLI
        std::size_t bound = BrotliEncoderMaxCompressedSize(payload.size());
        if (bound == 0) {
            bound = payload.size() + payload.size() / 4 + 1024;
        }
        compressed.resize(bound);
        std::size_t outLen = bound;
        const BROTLI_BOOL ok = BrotliEncoderCompress(
            /*quality=*/11, /*lgwin=*/BROTLI_MAX_WINDOW_BITS, BROTLI_MODE_GENERIC, payload.size(), payload.data(), &outLen, compressed.data());
        if (!ok) {
            return false;
        }
        compressed.resize(outLen);
#endif
    } else {
        uLongf cap = compressBound(static_cast<uLong>(payload.size()));
        compressed.resize(cap);
        uLongf outLen = cap;
        const int zret = compress2(compressed.data(), &outLen, payload.data(), static_cast<uLong>(payload.size()),
                                   /*level=*/9);
        if (zret != Z_OK) {
            return false;
        }
        compressed.resize(outLen);
    }

    const std::uint32_t rawLen = static_cast<std::uint32_t>(payload.size());
    wrapped.clear();
    if (codec == kCompactBinCodecZlib) {
        wrapped.reserve(8 + compressed.size());
        appendPaperCompactU32LE(wrapped, rawLen);
        appendPaperCompactU32LE(wrapped, static_cast<std::uint32_t>(compressed.size()));
    } else {
        wrapped.reserve(9 + compressed.size());
        appendPaperCompactU32LE(wrapped, rawLen | kCompactBinExtendedCodecFlag);
        appendPaperCompactU32LE(wrapped, static_cast<std::uint32_t>(compressed.size() + 1));
        wrapped.push_back(codec);
    }
    wrapped.insert(wrapped.end(), compressed.begin(), compressed.end());
    return true;
}

inline bool compressCompactBinPayload(
    const std::vector<std::uint8_t>& payload,
    std::vector<std::uint8_t>& wrapped) {
    return compressCompactBinPayloadWithCodec(
        payload, requestedCompactBinCodec(), wrapped);
}

inline std::uint16_t readPaperCompactU16LE(const std::uint8_t*& p) {
    std::uint16_t value = static_cast<std::uint16_t>(p[0]) |
                          (static_cast<std::uint16_t>(p[1]) << 8);
    p += 2;
    return value;
}

inline std::uint32_t readPaperCompactU32LE(const std::uint8_t*& p) {
    std::uint32_t value = static_cast<std::uint32_t>(p[0]) |
                          (static_cast<std::uint32_t>(p[1]) << 8) |
                          (static_cast<std::uint32_t>(p[2]) << 16) |
                          (static_cast<std::uint32_t>(p[3]) << 24);
    p += 4;
    return value;
}

inline std::int32_t readPaperCompactI32LE(const std::uint8_t*& p) {
    return static_cast<std::int32_t>(readPaperCompactU32LE(p));
}

inline std::uint8_t readPaperCompactU8(const std::uint8_t*& p) {
    return *p++;
}

inline float readPaperCompactF32LE(const std::uint8_t*& p) {
    const std::uint32_t bits = readPaperCompactU32LE(p);
    float value = 0.0f;
    std::memcpy(&value, &bits, sizeof(float));
    return value;
}

struct PaperCompactPayloadHeader {
    std::uint16_t version = 0;
    std::uint16_t flags = 0;
    bool hasQuant = false;
    bool hasSeg = false;
    bool compactU16 = false;
    bool triFaces = false;
    bool varintBase = false;
    bool edgeBreakerBase = false;
    bool splitResidualBase = false;
    bool nativeEdgeBreakerBase = false;
    bool hasCanonicalSourceLayout = false;
    /// 存储方案自描述（Phase S0 / R13）。**该位未置时产物与引入本机制前逐字节相同**，
    /// 因此现存全部产物零迁移——解码端见不到该字段即按原始方案解。
    bool hasStorageScheme = false;
    std::int32_t round = 0;
    std::int32_t groupNumber = 0;
    std::uint8_t flagBits = 0;
    std::uint8_t flagBits2 = 0;
    std::int32_t codecVersion = 0;
    std::int32_t blobLayout = 0;
    std::int32_t canonicalSourceLayout = 0;
    std::int32_t origTotalVertices = 0;
    std::int32_t origTotalFaces = 0;
    std::int32_t quantBits = 0;
    float bboxMinX = 0.0f;
    float bboxMinY = 0.0f;
    float bboxMinZ = 0.0f;
    float quantStep = 0.0f;
    /// 方案名与版本。仅当 hasStorageScheme 为真时有意义。
    std::string storageSchemeName;
    std::uint32_t storageSchemeVersion = 0;
};

/// @brief 写入存储方案自描述块：`u32 nameLen | name[] | u32 version`。
///
/// 调用方必须**先**在 `flags` 里置 `0x100`，再在 quant 块之后调用本函数 ——
/// 这样未指定方案的产物一个字节都不会多。
inline void appendPaperCompactStorageScheme(std::vector<std::uint8_t>& out,
                                            const std::string& name,
                                            std::uint32_t version) {
    appendPaperCompactU32LE(out, static_cast<std::uint32_t>(name.size()));
    out.insert(out.end(), name.begin(), name.end());
    appendPaperCompactU32LE(out, version);
}

inline PaperCompactPayloadHeader readPaperCompactPayloadHeader(
    const std::uint8_t*& p,
    bool hasSecondFlagByte) {
    PaperCompactPayloadHeader header;
    header.version = readPaperCompactU16LE(p);
    header.flags = readPaperCompactU16LE(p);
    header.hasQuant = (header.flags & 0x1) != 0;
    header.hasSeg = (header.flags & 0x2) != 0;
    header.compactU16 = (header.flags & 0x4) != 0;
    header.triFaces = (header.flags & 0x8) != 0;
    header.varintBase = (header.flags & 0x10) != 0;
    header.edgeBreakerBase = (header.flags & 0x20) != 0;
    header.splitResidualBase = (header.flags & 0x40) != 0;
    header.nativeEdgeBreakerBase = (header.flags & 0x200) != 0;
    header.hasCanonicalSourceLayout = (header.flags & 0x80) != 0;
    header.hasStorageScheme = (header.flags & 0x100) != 0;

    header.round = readPaperCompactI32LE(p);
    header.groupNumber = readPaperCompactI32LE(p);
    header.flagBits = readPaperCompactU8(p);
    if (hasSecondFlagByte) {
        header.flagBits2 = readPaperCompactU8(p);
    }
    header.codecVersion = readPaperCompactI32LE(p);
    header.blobLayout = readPaperCompactI32LE(p);
    if (header.hasCanonicalSourceLayout) {
        header.canonicalSourceLayout = readPaperCompactI32LE(p);
    }
    header.origTotalVertices = readPaperCompactI32LE(p);
    header.origTotalFaces = readPaperCompactI32LE(p);

    if (header.hasQuant) {
        header.quantBits = readPaperCompactI32LE(p);
        header.bboxMinX = readPaperCompactF32LE(p);
        header.bboxMinY = readPaperCompactF32LE(p);
        header.bboxMinZ = readPaperCompactF32LE(p);
        header.quantStep = readPaperCompactF32LE(p);
    }

    if (header.hasStorageScheme) {
        const std::uint32_t nameLen = readPaperCompactU32LE(p);
        header.storageSchemeName.assign(reinterpret_cast<const char*>(p), nameLen);
        p += nameLen;
        header.storageSchemeVersion = readPaperCompactU32LE(p);
    }
    return header;
}

// Bounded entry point for untrusted input. The cursor is advanced only after
// the complete optional header has been validated.
inline bool readPaperCompactPayloadHeader(
    const std::uint8_t*& p, const std::uint8_t* end,
    bool hasSecondFlagByte, PaperCompactPayloadHeader& header) {
    if (p > end || static_cast<std::size_t>(end - p) < 4) return false;
    const std::uint16_t flags = static_cast<std::uint16_t>(p[2]) |
                               (static_cast<std::uint16_t>(p[3]) << 8);
    const std::size_t fixedSize = 4 + 8 + 1 + (hasSecondFlagByte ? 1 : 0) +
                                 8 + ((flags & 0x80u) ? 4 : 0) + 8 +
                                 ((flags & 0x1u) ? 20 : 0);
    if (static_cast<std::size_t>(end - p) < fixedSize) return false;
    if ((flags & 0x100u) != 0) {
        const auto* scheme = p + fixedSize;
        if (static_cast<std::size_t>(end - scheme) < 8) return false;
        const std::uint32_t nameLen = readPaperCompactU32LE(scheme);
        if (nameLen > static_cast<std::size_t>(end - scheme) - 4) return false;
    }
    header = readPaperCompactPayloadHeader(p, hasSecondFlagByte);
    return true;
}

inline bool readPMSFCompactBinEnvelopeBytes(
    const std::vector<std::uint8_t>& compactBin,
    const std::string& sourceLabel,
    const char* logPrefix,
    PMSFCompactBinEnvelope& envelope) {
    if (compactBin.size() < 8) {
        std::cerr << logPrefix << " compact bin 头损坏: "
                  << sourceLabel << std::endl;
        return false;
    }

    envelope.rawLen = readPaperCompactU32LE(compactBin, 0);
    envelope.compLen = readPaperCompactU32LE(compactBin, 4);
    const bool extendedCodec = (envelope.rawLen & kCompactBinExtendedCodecFlag) != 0;
    envelope.rawLen &= ~kCompactBinExtendedCodecFlag;
    if (envelope.rawLen == 0 || envelope.compLen == 0) {
        std::cerr << logPrefix << " compact bin 头损坏: "
                  << sourceLabel << std::endl;
        return false;
    }
    if (envelope.compLen > compactBin.size() - 8u ||
        compactBin.size() != 8u + static_cast<std::size_t>(envelope.compLen)) {
        std::cerr << logPrefix << " compact bin 内容不足: "
                  << sourceLabel << std::endl;
        return false;
    }

    envelope.wrapped.assign(
        compactBin.begin(),
        compactBin.begin() + 8u + static_cast<std::size_t>(envelope.compLen));
    envelope.payload.resize(envelope.rawLen);
    const std::uint8_t* compressed = envelope.wrapped.data() + 8;
    std::size_t compressedLen = envelope.compLen;
    std::uint8_t codec = kCompactBinCodecZlib;
    if (extendedCodec) {
        if (compressedLen < 1) {
            std::cerr << logPrefix << " compact bin 扩展后端缺少 codec 字节: " << sourceLabel << std::endl;
            return false;
        }
        codec = compressed[0];
        ++compressed;
        --compressedLen;
    }

    if (codec == kCompactBinCodecBitCtx) {
        // Five self-describing parameter bytes plus the two-byte decoder
        // bootstrap and the minimum range-coder trailer bytes.  The range
        // coder is bounded below as well, but reject obviously short streams
        // before entering it so malformed v6 envelopes fail deterministically.
        if (compressedLen < 10) {
            std::cerr << logPrefix << " compact bin bitctx 头不足: " << sourceLabel << std::endl;
            return false;
        }
        const int lgTotf = compressed[0];
        const int rescaleLog = compressed[1];
        CompactBinBitCtx cfg;
        cfg.prefixBits = compressed[2];
        cfg.prevShr = compressed[3];
        cfg.vpCap = compressed[4];
        if (lgTotf < 2 || lgTotf > 24 || rescaleLog < 1 || rescaleLog > lgTotf ||
            cfg.prefixBits < 0 || cfg.prefixBits > 8 || cfg.prevShr < 0 || cfg.prevShr > 8 ||
            cfg.vpCap < 0 || cfg.vpCap > 31) {
            std::cerr << logPrefix << " compact bin bitctx 参数越界: " << sourceLabel << std::endl;
            return false;
        }
        // ⚠️ 指针偏移与长度必须**同步**等于自描述参数字节数（5）。曾经只改了长度
        // 没改偏移 ⇒ 码流整体错位 1 字节 ⇒ start_decoding 失败、解码端静默回落到
        // parquet 路径（见附 48 的排障）。
        return decodeCompactBinBitCtx(reinterpret_cast<const char*>(compressed) + 5,
                                      compressedLen - 5,
                                      cfg,
                                      lgTotf,
                                      rescaleLog,
                                      envelope.rawLen,
                                      envelope.payload);
    }
    if (codec == kCompactBinCodecBrotli) {
#ifdef PMSF_HAS_BROTLI
        std::size_t outLen = envelope.payload.size();
        const BrotliDecoderResult bret = BrotliDecoderDecompress(compressedLen, compressed, &outLen, envelope.payload.data());
        if (bret != BROTLI_DECODER_RESULT_SUCCESS || outLen != envelope.rawLen) {
            std::cerr << logPrefix << " compact bin brotli 解压失败 rc=" << static_cast<int>(bret) << " outLen=" << outLen << "/" << envelope.rawLen
                      << std::endl;
            return false;
        }
        return true;
#else
        std::cerr << logPrefix << " compact bin 需要 brotli 但本构建未链接: " << sourceLabel << std::endl;
        return false;
#endif
    }
    if (codec != kCompactBinCodecZlib) {
        std::cerr << logPrefix << " compact bin 未知 codec=" << static_cast<int>(codec) << ": " << sourceLabel << std::endl;
        return false;
    }

    uLongf outLen = envelope.rawLen;
    const int zret = uncompress(envelope.payload.data(), &outLen, compressed, static_cast<uLong>(compressedLen));
    if (zret != Z_OK || outLen != envelope.rawLen) {
        std::cerr << logPrefix << " compact bin zlib 解压失败 rc=" << zret
                  << " outLen=" << outLen << "/" << envelope.rawLen
                  << std::endl;
        return false;
    }

    return true;
}

inline bool readPMSFCompactBinEnvelopeFile(
    const std::string& path,
    const char* logPrefix,
    PMSFCompactBinEnvelope& envelope) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) {
        return false;
    }
    const std::vector<std::uint8_t> compactBin(
        (std::istreambuf_iterator<char>(ifs)),
        std::istreambuf_iterator<char>());
    return readPMSFCompactBinEnvelopeBytes(
        compactBin,
        path,
        logPrefix,
        envelope);
}

}  // namespace pmsf

#endif  // PMSF_STORAGE_PAPER_COMPACT_BIN_ENVELOPE_H
