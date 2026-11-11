// SPDX-License-Identifier: Apache-2.0
//
// RDCanonicalPaperRa.h — task-54: RD page-level RA（PRA1）profile 的 CONN round
// 重排（纯字节工具，header-only）。
//
// 背景（task-53 ledger / task-54 定位）：
//   route A 编码端把 CONN rounds 按 **encoder 顺序** 写进 blob
//   （container round r = encoder layer r，CONN scheme_version=71），解码端按
//   **总 R 反序**映射（decoder round i ← container R-1-i）。
//   page-RA 的 PRA1 profile 按 container round 顺序切 block(10) 块，readPrefix(k)
//   只读前 ceil(k/10) 块。对 RD 而言，前 k 块 = **最细**的 encoder 层，而 decoder
//   的前 k 轮需要的是**最粗**的末尾 k 层 ⇒ k=10/25 prefix decode mismatch
//   （k=0/50 因两端恰好对齐而匹配）。
//
// 修法（不改官方 fused artifact、不改 encoder canonical bytes）：
//   RA profile 生成时把 CONN payload 的 **round 顺序整体反转**
//   （新 container round r = encoder layer R-1-r），并把 header 的
//   scheme_version 标成 **72**（语义："rounds 已按 decoder 顺序存储"）。
//   解码端看到 sv=72 时直接按 container 顺序取层（decoder round i ← container i）：
//     - prefix k：前 ceil(k/10) 块 = decoder rounds 0..c*10-1 = encoder layers
//       R-1..R-c*10 ⇒ max_lod=k 解码 == canonical maxLod refs；
//     - full k=50：container 顺序恰为原解码顺序 ⇒ decoded file 与 canonical 逐字节相同；
//     - sv=71 官方 artifact 的 reader 反序映射完全不变。
//
// 本文件只做字节级重排：输入 = 合法 CONN header（sv=71）+ payload（rounds 个
// "streams 连续段"），输出 = sv=72 header（offsets 重算）+ 反转后的 payload。
#ifndef PMSF_IMPL_RD_RD_CANONICAL_PAPER_RA_H
#define PMSF_IMPL_RD_RD_CANONICAL_PAPER_RA_H

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace rd_canon {

/// CONN container scheme_version 语义（与 RDCanonicalBlobFormat.h 对齐）：
///   71 = rounds 按 encoder 顺序（官方 fused / canonical columnar artifact）；
///   72 = rounds 已按 decoder 顺序（仅 page-RA profile 使用）。
constexpr int kCanonConnSchemeEncoderOrder = 71;
constexpr int kCanonConnSchemeDecoderOrder = 72;

struct RdConnReorderResult {
    std::vector<std::uint8_t> header;       ///< sv=72 CONN header（offsets 已重算）
    std::vector<std::uint8_t> payload;      ///< decoder-order rounds 串接
    /// payload-relative **每轮起点**，size R+1（= 工具 ConnRaw::roundOffsets 口径；
    /// 不是 CONN header 里那张 R*S+1 的扁平表）。
    std::vector<std::size_t> roundOffsets;
    std::uint32_t rounds = 0;
    std::uint32_t streams = 0;
    bool ok = false;
};

inline std::uint16_t rdRaReadU16(const std::vector<std::uint8_t>& b, std::size_t off) {
    return static_cast<std::uint16_t>(b[off]) |
           static_cast<std::uint16_t>(static_cast<std::uint16_t>(b[off + 1]) << 8);
}

inline std::uint32_t rdRaReadU32(const std::vector<std::uint8_t>& b, std::size_t off) {
    return static_cast<std::uint32_t>(b[off]) |
           (static_cast<std::uint32_t>(b[off + 1]) << 8) |
           (static_cast<std::uint32_t>(b[off + 2]) << 16) |
           (static_cast<std::uint32_t>(b[off + 3]) << 24);
}

inline void rdRaPutU16(std::vector<std::uint8_t>& out, std::uint16_t v) {
    out.push_back(static_cast<std::uint8_t>(v & 0xffu));
    out.push_back(static_cast<std::uint8_t>((v >> 8) & 0xffu));
}

inline bool rdRaPutU32(std::vector<std::uint8_t>& out, std::uint64_t v) {
    if (v > 0xffffffffull) return false;
    const std::uint32_t x = static_cast<std::uint32_t>(v);
    out.push_back(static_cast<std::uint8_t>(x & 0xffu));
    out.push_back(static_cast<std::uint8_t>((x >> 8) & 0xffu));
    out.push_back(static_cast<std::uint8_t>((x >> 16) & 0xffu));
    out.push_back(static_cast<std::uint8_t>((x >> 24) & 0xffu));
    return true;
}

/// 把 sv=71（encoder-order rounds）的 CONN blob 重排为 sv=72（decoder-order）。
/// @return ok=false 表示输入不是合法的 sv=71 CONN header/payload（调用方 fail-closed）。
inline RdConnReorderResult reorderConnRoundsDecoderFirst(
    const std::vector<std::uint8_t>& header,
    const std::vector<std::uint8_t>& payload) {
    RdConnReorderResult res;
    if (header.size() < 16) return res;
    if (std::memcmp(header.data(), "CONN", 4) != 0) return res;
    const std::uint16_t sv = rdRaReadU16(header, 6);
    if (sv != static_cast<std::uint16_t>(kCanonConnSchemeEncoderOrder)) return res;
    const std::uint32_t rounds = rdRaReadU32(header, 8);
    const std::uint32_t streams = rdRaReadU32(header, 12);
    if (rounds == 0 || streams == 0) return res;
    const std::size_t entries =
        static_cast<std::size_t>(rounds) * static_cast<std::size_t>(streams) + 1u;
    const std::size_t needHdr = 16u + 4u * entries;
    if (header.size() < needHdr) return res;
    std::vector<std::size_t> off(entries);
    for (std::size_t i = 0; i < entries; ++i) {
        off[i] = rdRaReadU32(header, 16u + 4u * i);
    }
    if (off[0] != 0 || off.back() != payload.size()) return res;
    for (std::size_t i = 1; i < entries; ++i) {
        if (off[i] < off[i - 1]) return res;  // 非单调 ⇒ 非法
    }

    // payload：round 顺序整体反转（每轮内 streams 顺序不变）。
    std::vector<std::size_t> newOff(entries, 0);
    res.payload.clear();
    res.payload.reserve(payload.size());
    for (std::uint32_t r = 0; r < rounds; ++r) {
        const std::uint32_t src = rounds - 1u - r;
        const std::size_t srcBase = off[static_cast<std::size_t>(src) * streams];
        for (std::uint32_t s = 0; s < streams; ++s) {
            const std::size_t idx = static_cast<std::size_t>(r) * streams + s;
            newOff[idx] = res.payload.size() +
                          (off[static_cast<std::size_t>(src) * streams + s] - srcBase);
        }
        const std::size_t begin = srcBase;
        const std::size_t end = off[(static_cast<std::size_t>(src) + 1u) * streams];
        res.payload.insert(res.payload.end(),
                           payload.begin() + static_cast<std::ptrdiff_t>(begin),
                           payload.begin() + static_cast<std::ptrdiff_t>(end));
    }
    newOff[entries - 1] = res.payload.size();

    // header：magic + 原 layout + sv=72 + 同 rounds/streams + 重算 offsets。
    res.header.clear();
    res.header.reserve(needHdr);
    res.header.push_back('C');
    res.header.push_back('O');
    res.header.push_back('N');
    res.header.push_back('N');
    res.header.push_back(header[4]);  // blob_layout（与官方 artifact 同值）
    res.header.push_back(header[5]);
    rdRaPutU16(res.header, static_cast<std::uint16_t>(kCanonConnSchemeDecoderOrder));
    if (!rdRaPutU32(res.header, rounds)) return res;
    if (!rdRaPutU32(res.header, streams)) return res;
    for (std::size_t i = 0; i < entries; ++i) {
        if (!rdRaPutU32(res.header, newOff[i])) return res;
    }
    res.roundOffsets.resize(static_cast<std::size_t>(rounds) + 1u);
    for (std::uint32_t r = 0; r < rounds; ++r) {
        res.roundOffsets[r] = newOff[static_cast<std::size_t>(r) * streams];
    }
    res.roundOffsets[rounds] = res.payload.size();
    res.rounds = rounds;
    res.streams = streams;
    res.ok = true;
    return res;
}

}  // namespace rd_canon

#endif  // PMSF_IMPL_RD_RD_CANONICAL_PAPER_RA_H
