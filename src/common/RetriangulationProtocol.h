// SPDX-License-Identifier: Apache-2.0
//
// RetriangulationProtocol.h — Phase 4 PR-A 公共抽象（"协议槽"接口）。
//
// 设计意图（详见 docs/paper_alignment/PHASE4_PLAN.md §3.2）：
//   PMSF 框架支持多种 connectivity 存储方式。三篇论文（PCLTTM / PCATM / RD）
//   各自有独立的"决定性三角化协议"——encoder 发什么 + decoder 推什么——
//   差异在 retriangulation 推断逻辑，而 stream 容器（ConnectivityBlobContainer）
//   与 ColumnarSerializeOperator 完全可复用。
//
//   本 header 只做"协议 ID 注册 + 元数据查询"，**不引入新的运行时依赖**：
//     - RetriProtocol enum 与 BlobLayout 的 CODEC_016/CODEC_017/RD_TAG 1:1 对应；
//     - RetriProtocolDesc 静态描述（name + stream 名字数组）供 BPV / diag 输出；
//     - getProtocolDesc(p) 静态返回。
//
//   旧 9 算法不调用本 header，零影响。
//
// 用法示例（PR-B/PR-C 落地 collector / reader 时）：
//   const auto& desc = pmsf::getProtocolDesc(pmsf::RetriProtocol::RD_TAG);
//   std::cerr << "[Bpv] proto=" << desc.name
//             << " streams=" << desc.num_streams << std::endl;
//
// 限制：
//   - 本 header **不** 持有 collector / reader factory（保留给 algorithm-side
//     工具入口装配）；这是为了避免在公共抽象层引入 src/compress/* 依赖，让
//     header 本身保持 zero-dependency（仅 <cstdint>）。
//
// 详见 docs/paper_alignment/PHASE4_PLAN.md §2 / §3.2。

#ifndef SRC_COMMON_RETRIANGULATION_PROTOCOL_H_
#define SRC_COMMON_RETRIANGULATION_PROTOCOL_H_

#include <cstdint>

namespace pmsf {

/// @brief 决定性三角化协议 ID（数值与 BlobLayout 严格 1:1 对齐）。
///
/// 枚举值与 ColumnarSerializeOperator::BlobLayout 共享数字：
///   - 16 = CODEC_016    : ⊕/⊖ tag + retriangulation table → N-2 对角线 implicit
///   - 17 = CODEC_017  : Z-shape coloring + 2/4-color → patch 边界 implicit
///   - 18 = RD_TAG        : prioritized gate + ⊕/⊖ tag + retriangulation table
///                          + 二阶 curvature predictor
///
/// 与 BlobLayout 对应数字必须一致——decoder 端 `case static_cast<int>(p):`
/// dispatch 直接 reuse。
enum class RetriProtocol : std::uint16_t {
    UNKNOWN       = 0,
    CODEC_016    = 16,
    CODEC_017  = 17,
    RD_TAG        = 18,
};

/// @brief 协议元数据（每协议一个静态实例）。
///
/// 用于 BPV diag 输出 / 单元测试。**不强制** collector/reader 持有此对象；
/// 算法可纯粹按 BlobLayout 数字编码，完全不引入本 header。
///
/// stream_names 数组以 nullptr 结尾，长度 == num_streams。指针指向 .rodata
/// （字面量），调用方不要 free。
struct RetriProtocolDesc {
    RetriProtocol      id;
    const char*        name;          ///< 协议短名（"CODEC_016" / ...）
    int                num_streams;   ///< ConnectivityBlob 段数（与 sv 对应）
    int                blob_layout;   ///< 1:1 数字（= static_cast<int>(id)）
    int                scheme_version;///< 当前默认 sv（CODEC_016=3 已落地，其它待定）
    const char* const* stream_names;  ///< 字面量指针数组（长度 = num_streams）
};

namespace detail {

inline const char* const kCodec016Streams[] = {
    "valence",        // 5-元 RC + 4-context + V_OTHER inline 256-元 escape
    "fusedFaceSym",   // 4-元 RC: {NotPoly=0, PolyU=1, PolyP=2, PolyM=3}
    "residual",       // bit-plane RC，含二阶 curvature predictor
};

inline const char* const kCodec017Streams[] = {
    "valence",        // 5-元 RC（per polyFace 1 sym）
    "colorBits",      // 1 或 2 bit / BFS triangle（mode 决定）
    "colorMode",      // 1 byte / round（2-color=0 / 4-color=1）
    "residual",       // bit-plane RC，PCATM 一阶 barycenter residual
};

inline const char* const kRDTagCodecStreams[] = {
    "valence",        // 同 CODEC_016
    "isPolyBit",      // 2-元 RC（per BFS face 1 bit；不含 ⊕/⊖ tag）
    "residual",       // bit-plane RC，含 RD curvature predictor
};

inline const char* const kUnknownStreams[] = {
    nullptr,
};

}  // namespace detail

/// @brief 静态协议元数据查询。未知协议返回 UNKNOWN 描述符（num_streams=0）。
inline const RetriProtocolDesc& getProtocolDesc(RetriProtocol p) noexcept {
    static constexpr RetriProtocolDesc kUnknown = {
        RetriProtocol::UNKNOWN, "UNKNOWN", 0, 0, 0, detail::kUnknownStreams};
    static constexpr RetriProtocolDesc kCodec016Desc = {
        RetriProtocol::CODEC_016, "CODEC_016",
        /*num_streams=*/3, /*blob_layout=*/16, /*sv=*/3,
        detail::kCodec016Streams};
    static constexpr RetriProtocolDesc kCodec017Desc = {
        RetriProtocol::CODEC_017, "CODEC_017",
        /*num_streams=*/4, /*blob_layout=*/17, /*sv=*/1,
        detail::kCodec017Streams};
    static constexpr RetriProtocolDesc kRDTagCodecDesc = {
        RetriProtocol::RD_TAG, "RD_TAG",
        /*num_streams=*/3, /*blob_layout=*/18, /*sv=*/1,
        detail::kRDTagCodecStreams};
    switch (p) {
        case RetriProtocol::CODEC_016:   return kCodec016Desc;
        case RetriProtocol::CODEC_017: return kCodec017Desc;
        case RetriProtocol::RD_TAG:       return kRDTagCodecDesc;
        case RetriProtocol::UNKNOWN:
        default:                          return kUnknown;
    }
}

/// @brief BlobLayout 数字 → RetriProtocol 反查（仅 16/17/18 命中本 enum）。
inline RetriProtocol blobLayoutToProtocol(int blobLayout) noexcept {
    switch (blobLayout) {
        case 16: return RetriProtocol::CODEC_016;
        case 17: return RetriProtocol::CODEC_017;
        case 18: return RetriProtocol::RD_TAG;
        default: return RetriProtocol::UNKNOWN;
    }
}

}  // namespace pmsf

#endif  // SRC_COMMON_RETRIANGULATION_PROTOCOL_H_
