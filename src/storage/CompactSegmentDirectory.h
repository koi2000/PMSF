#ifndef PMSF_STORAGE_COMPACT_SEGMENT_DIRECTORY_H
#define PMSF_STORAGE_COMPACT_SEGMENT_DIRECTORY_H

// CompactSegmentDirectory.h —— `_compact.bin` 的**通用多段目录**（主计划 §7 R8）。
//
// 背景：目标 2 要求所有算法产出**唯一形态**的 `_compact.bin`
// （自描述段目录：header + 分段 offset 表 + codec_id）。单流算法（RD / PCLTTM …）
// 的载荷天然是一段 range-coded 流，用既有的 v1..v5 布局即可表达；但有些算法的
// 载荷**本质是多段**（例如按簇切分的连通性/元数据/残差，或分立的位流 + 码本）。
//
// R8 的裁决是**方案 A**：唯一产物形态不变，为多段载荷补一个**通用段目录**，
// 并且该目录必须进 `src/` 核心供未来算法复用 ——
// **不得**在核心写任何算法特判，也**不承认**"两类形态"。
//
// 因此本文件只认识"若干个 {名字, codec_id, 字节} 段"，不认识任何算法。
// 段名由调用方（`impl/` 侧）给出，序号即目录顺序。
//
// 字节布局（全部 little-endian，追加在既有 payload header 之后）：
//
//   "V2CP"                    magic (4B)
//   u16 version = 6           段目录容器版本
//   u16 flags                 与 v1..v5 同义（bit0=hasQuant … bit7=hasCanonical）
//   --- 通用 metadata（**与 v1..v5 同序**，便于统一解析 codec_id 与规模）---
//   i32 round
//   i32 groupNumber
//   u8  flagBits
//   u8  flagBits2
//   i32 codecVersion          容器级 codec_id 前半
//   i32 blobLayout            容器级 codec_id 后半
//   [i32 canonicalSourceLayout]   仅当 flags bit7
//   i32 totalVertices
//   i32 totalFaces
//   [i32 quantBits, f32 bboxMinXYZ ×3, f32 quantStep]  仅当 flags bit0
//   --- 通用段目录 ---
//   u32 numSegments
//   ×numSegments:
//     u32 nameLen
//     u8  name[nameLen]       UTF-8，无终止符
//     u32 offset              相对 **payload 起点** 的字节偏移
//     u32 length
//     i32 codecId             该段自身的 codec 标识
//   --- 段数据（offset 指向此处）---
//
// 外层与 v1..v5 一致：`[u32 rawLen][u32 compLen][stream]`。
// 具体 codec 由 PMSFCompactBinEnvelope 的统一、自描述封装器选择；显式设置
// `PMSF_COMPACT_CODEC` 时服从该选择，未设置时 v6 默认用 zlib。v6 段通常承载
// 已经内部压缩的 Parquet 字节，避免为尝试 bitctx 把编码工作量翻倍。

#include "scheme/StorageSchemeRegistry.h"
#include "storage/PMSFCompactBinEnvelope.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <limits>
#include <set>
#include <string>
#include <vector>

namespace pmsf {

/// 段目录容器版本。v1..v5 是单流布局；v6 起带通用段目录。
inline constexpr std::uint16_t kCompactSegmentContainerVersion = 6;

struct CompactSegment {
    std::string name;
    std::int32_t codecId = 0;
    std::vector<std::uint8_t> bytes;
};

inline void appendCompactU16LE(std::vector<std::uint8_t>& out, std::uint16_t v) {
    out.push_back(static_cast<std::uint8_t>(v & 0xffu));
    out.push_back(static_cast<std::uint8_t>((v >> 8) & 0xffu));
}

inline void appendCompactU8(std::vector<std::uint8_t>& out, std::uint8_t v) {
    out.push_back(v);
}

inline void appendCompactI32LE(std::vector<std::uint8_t>& out, std::int32_t v) {
    appendPaperCompactU32LE(out, static_cast<std::uint32_t>(v));
}

inline void appendCompactF32LE(std::vector<std::uint8_t>& out, float v) {
    static_assert(sizeof(float) == 4, "float must be 32-bit");
    std::uint32_t bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    appendPaperCompactU32LE(out, bits);
}

/// 由既有 payload header + 若干段构造 v6 容器 payload。
/// `header.version` 会被写成 kCompactSegmentContainerVersion（其余字段原样保留）。
inline std::vector<std::uint8_t> buildCompactSegmentPayload(
    PaperCompactPayloadHeader header,
    const std::vector<CompactSegment>& segments) {
    std::vector<std::uint8_t> out;
    out.push_back('V');
    out.push_back('2');
    out.push_back('C');
    out.push_back('P');
    appendCompactU16LE(out, kCompactSegmentContainerVersion);
    // Phase S0（R13）：非默认存储方案才置位/写块，未指定时产物逐字节不变。
    const ::pmsf::StorageSchemePtr scheme = ::pmsf::activeStorageScheme();
    const bool hasScheme = (scheme != nullptr) && !scheme->info().name.empty();
    std::uint16_t flags = header.flags;
    if (hasScheme) flags |= 0x100;
    appendCompactU16LE(out, flags);

    appendCompactI32LE(out, header.round);
    appendCompactI32LE(out, header.groupNumber);
    appendCompactU8(out, header.flagBits);
    appendCompactU8(out, header.flagBits2);
    appendCompactI32LE(out, header.codecVersion);
    appendCompactI32LE(out, header.blobLayout);
    if (header.hasCanonicalSourceLayout) {
        appendCompactI32LE(out, header.canonicalSourceLayout);
    }
    appendCompactI32LE(out, header.origTotalVertices);
    appendCompactI32LE(out, header.origTotalFaces);
    if (header.hasQuant) {
        appendCompactI32LE(out, header.quantBits);
        appendCompactF32LE(out, header.bboxMinX);
        appendCompactF32LE(out, header.bboxMinY);
        appendCompactF32LE(out, header.bboxMinZ);
        appendCompactF32LE(out, header.quantStep);
    }
    if (hasScheme) {
        appendPaperCompactStorageScheme(out, scheme->info().name,
                                        scheme->info().version);
    }

    // 目录：先占位算长度，再回填 offset（offset 相对 payload 起点）。
    const std::size_t dirStart = out.size();
    std::size_t dirSize = 4;  // numSegments
    for (const auto& s : segments) {
        dirSize += 4 + s.name.size() + 4 + 4 + 4;
    }
    const std::uint32_t dataStart = static_cast<std::uint32_t>(dirStart + dirSize);

    appendPaperCompactU32LE(out, static_cast<std::uint32_t>(segments.size()));
    std::uint32_t cursor = dataStart;
    for (const auto& s : segments) {
        appendPaperCompactU32LE(out, static_cast<std::uint32_t>(s.name.size()));
        out.insert(out.end(), s.name.begin(), s.name.end());
        appendPaperCompactU32LE(out, cursor);
        appendPaperCompactU32LE(out, static_cast<std::uint32_t>(s.bytes.size()));
        appendCompactI32LE(out, s.codecId);
        cursor += static_cast<std::uint32_t>(s.bytes.size());
    }
    for (const auto& s : segments) {
        out.insert(out.end(), s.bytes.begin(), s.bytes.end());
    }
    return out;
}

/// 使用统一 compact-bin envelope 压缩并加 `[u32 rawLen][u32 compLen]` 外层，
/// 得到 `<base>_compact.bin` 的完整字节。显式 codec 选择保持严格可复现；
/// 未显式选择时使用 zlib，以保留旧 v6 产物的体积和编码成本。
///
/// v6 段目录与普通 compact-bin 共享同一个自描述 codec 入口。
/// 段内容、codecId 和 payload 语义均不改变。
inline bool wrapCompactBinPayload(const std::vector<std::uint8_t>& payload,
                                  std::vector<std::uint8_t>& wrapped) {
    const char* requested = std::getenv("PMSF_COMPACT_CODEC");
    if (requested != nullptr && requested[0] != '\0') {
        return compressCompactBinPayload(payload, wrapped);
    }
    return compressCompactBinPayloadWithCodec(
        payload, kCompactBinCodecZlib, wrapped);
}

/// Validate a v6 directory before exposing slices. False means unsupported or
/// malformed input; a known v6 artifact must not silently fall back on failure.
inline bool readCompactSegmentDirectory(const std::vector<std::uint8_t>& payload,
                                        std::vector<std::string>& names,
                                        std::vector<std::int32_t>& codecIds,
                                        std::vector<std::uint32_t>& offsets,
                                        std::vector<std::uint32_t>& lengths) {
    names.clear();
    codecIds.clear();
    offsets.clear();
    lengths.clear();
    if (payload.size() < 8 || payload[0] != 'V' || payload[1] != '2' ||
        payload[2] != 'C' || payload[3] != 'P') {
        return false;
    }

    if (payload.size() > std::numeric_limits<std::uint32_t>::max()) return false;

    const std::uint8_t* p = payload.data() + 4;
    const std::uint8_t* const end = payload.data() + payload.size();
    PaperCompactPayloadHeader header;
    if (!readPaperCompactPayloadHeader(p, end, true, header) ||
        header.version != kCompactSegmentContainerVersion) {
        return false;
    }
    const auto remaining = [&]() { return static_cast<std::size_t>(end - p); };
    if (remaining() < 4) return false;
    const std::uint32_t numSegments = readPaperCompactU32LE(p);
    // Every entry needs at least nameLen + offset + length + codecId.
    if (numSegments > remaining() / 16) return false;
    std::vector<std::string> parsedNames;
    std::vector<std::int32_t> parsedCodecs;
    std::vector<std::uint32_t> parsedOffsets, parsedLengths;
    std::set<std::string> uniqueNames;
    for (std::uint32_t i = 0; i < numSegments; ++i) {
        if (remaining() < 4) return false;
        const std::uint32_t nameLen = readPaperCompactU32LE(p);
        if (nameLen > remaining() || remaining() - nameLen < 12) {
            return false;
        }
        parsedNames.emplace_back(reinterpret_cast<const char*>(p), nameLen);
        if (!uniqueNames.insert(parsedNames.back()).second) return false;
        p += nameLen;
        parsedOffsets.push_back(readPaperCompactU32LE(p));
        parsedLengths.push_back(readPaperCompactU32LE(p));
        parsedCodecs.push_back(readPaperCompactI32LE(p));
    }
    const std::size_t dataStart = static_cast<std::size_t>(p - payload.data());

    struct SegmentRange {
        std::uint32_t begin;
        std::uint32_t end;
    };
    std::vector<SegmentRange> ranges;
    ranges.reserve(parsedOffsets.size());
    for (std::size_t i = 0; i < parsedOffsets.size(); ++i) {
        const std::uint64_t begin = parsedOffsets[i];
        const std::uint64_t finish = begin + parsedLengths[i];
        if (begin < dataStart || finish > payload.size()) {
            return false;
        }
        if (parsedLengths[i] != 0) {
            ranges.push_back({parsedOffsets[i], static_cast<std::uint32_t>(finish)});
        }
    }
    std::sort(ranges.begin(), ranges.end(),
              [](const SegmentRange& a, const SegmentRange& b) {
                  if (a.begin != b.begin) return a.begin < b.begin;
                  return a.end < b.end;
              });
    for (std::size_t i = 1; i < ranges.size(); ++i) {
        if (ranges[i - 1].end > ranges[i].begin) {
            return false;
        }
    }
    names.swap(parsedNames);
    codecIds.swap(parsedCodecs);
    offsets.swap(parsedOffsets);
    lengths.swap(parsedLengths);
    return true;
}

}  // namespace pmsf

#endif  // PMSF_STORAGE_COMPACT_SEGMENT_DIRECTORY_H
