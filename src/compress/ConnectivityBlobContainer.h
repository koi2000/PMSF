/*
 * @file        ConnectivityBlobContainer.h
 * @brief       通用 rc_blob 容器协议 v3 —— 连通性比特流所有 5 套方案的统一封装。
 * @author      koi
 * @date        2026-05-10
 *
 * Phase 0 · Step 0.3 of docs/CONNECTIVITY_BITSTREAM_IMPLEMENTATION_PLAN.md.
 *
 * 背景：
 *   Parquet V2 的 `_symbols.parquet` 固定单列 `rc_blob BINARY`，内部布局靠
 *   `BlobLayout` enum + 本容器的 `scheme_version` 协商。
 *   所有 5 套 connectivity 方案 (V/B/C/P/H) 都产出多轮 × 多子流的字节块，
 *   用本容器统一打包可让 `ColumnarDeserialize` + `setMaxRounds()` 随机访问
 *   不再需要算法特有的 blob 拆包代码。
 *
 * 字节布局（little-endian，header 对齐到 byte，payload 紧随其后）：
 *
 *   [uint32  magic          = 0x4E4E4F43 ('CONN' LE)]
 *   [uint16  blob_layout    ]                           // 与 BlobLayout enum 对齐
 *   [uint16  scheme_version ]                           // 同一 layout 内的灰度子版本
 *   [uint32  num_rounds = R ]
 *   [uint32  num_streams = S]
 *   [uint32  stream_offsets[(R+1) * S]]                 // 扁平二维表，见下
 *   [bytes   payload (串接 R × S 个子流字节)]
 *
 * stream_offsets[r * S + s] = 第 r 轮、第 s 个子流的**起始字节**（相对 payload 起点）。
 * 同一行内按 s=0..S-1 单调不减；行之间（r -> r+1）也单调不减。
 * 额外约定：`stream_offsets[R * S + 0]`（即尾哨兵）== payload 总长度；
 *         因此 `stream_offsets` 真实长度为 `R * S + 1`。
 *
 * 取 (round, stream) 的切片字节范围：
 *   begin = stream_offsets[r * S + s]
 *   end   = (s + 1 < S) ? stream_offsets[r * S + (s + 1)]
 *                       : stream_offsets[(r + 1) * S + 0]
 *
 * 这让：
 *   - 按 round 截断（setMaxRounds=R'）：直接读 payload 的前
 *     `stream_offsets[R' * S + 0]` 个字节即可（以及对应 offsets 前缀）。
 *   - 按 stream_id 拿全量（跨所有轮）：依序遍历每轮切片，内部无冗余结构。
 *
 * Rollback 注意：
 *   - scheme_version=0 预留给 unit-test；
 *   - scheme_version=1 为 Phase 0 首发；所有 Phase 1+ 新方案从 sv=2 起。
 *   - magic/layout/version 任何一项 mismatch 都 fail-fast，不做"尽力解析"。
 */

#ifndef PMSF_COMPRESS_CONNECTIVITY_BLOB_CONTAINER_H
#define PMSF_COMPRESS_CONNECTIVITY_BLOB_CONTAINER_H

#include "storage/PaperBlobLayoutRegistry.h"
#include "utils/BitStream.h"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace pmsf {

/// @brief 'CONN' 的 little-endian 整数表示：byte[0]='C'(0x43), ..., byte[3]='N'(0x4E).
constexpr uint32_t kConnectivityMagic = 0x4E4E4F43u;

/// @brief 纯数据视图 —— header 字段。
///        writeTo / readFrom 负责字节序列化到 BitWriter/BitReader。
struct ConnectivityBlobHeader {
    uint32_t magic           = kConnectivityMagic;
    uint16_t blob_layout     = 0;  // 对应 ColumnarSerializeOperator::BlobLayout
    uint16_t scheme_version  = 1;  // Phase 0 默认 sv=1
    uint32_t num_rounds      = 0;
    uint32_t num_streams     = 0;

    /// @brief 扁平二维 offsets 表。
    ///        长度 = num_rounds * num_streams + 1（末尾是 payload 总长 sentinel）。
    std::vector<uint32_t> stream_offsets;

    /// @brief 序列化到已对齐的 BitWriter（字节流）。
    void writeTo(BitWriter& w) const;

    /// @brief 从 BitReader 读一份 header。失败抛 std::runtime_error。
    static ConnectivityBlobHeader readFrom(BitReader& r);

    /// @brief header 在字节流中占用的总字节数（含 stream_offsets）。
    size_t headerByteSize() const;
};

// --------------------------- Builder ----------------------------------------

/// @brief 构建连通性 blob：按 (round_id, stream_id) 顺序 append 子流字节，finalize 输出。
///
/// 典型用法（方案 V, S stream / R round）:
///     ConnectivityBlobBuilder b(pmsf::makePaperConnectivityBlobSpec(
///         blobLayout, schemeVersion, R, streamCount));
///     for (int r = 0; r < R; ++r) {
///         b.appendStreamBytes(r, kStreamPatchCodes, patchCodesRc);
///         b.appendStreamBytes(r, kStreamEscapes, escapeBytes);
///         b.appendStreamBytes(r, kStreamResiduals, residualsRc);
///     }
///     auto blob = b.finalize();
///
/// 不要求按顺序调用（内部用 (R,S) 矩阵存储），但同一个 (r,s) 只允许写一次。
class ConnectivityBlobBuilder {
  public:
    ConnectivityBlobBuilder(int blob_layout,
                            int scheme_version,
                            int num_rounds,
                            int num_streams);
    explicit ConnectivityBlobBuilder(const PaperConnectivityBlobSpec& spec);

    /// @brief 追加一段子流字节。
    /// @param round_id   [0, num_rounds).
    /// @param stream_id  [0, num_streams).
    /// @param bytes      子流字节内容；可空（表示该 (r,s) 无数据，对应 size=0 切片）。
    void appendStreamBytes(int round_id, int stream_id, const std::vector<uint8_t>& bytes);

    /// @brief 同上，span 形式。
    void appendStreamBytes(int round_id, int stream_id, std::span<const uint8_t> bytes);

    /// @brief 固化，返回完整 blob（header + payload）。调用后对象回到空状态。
    std::vector<uint8_t> finalize();

    int numRounds() const { return num_rounds_; }
    int numStreams() const { return num_streams_; }

  private:
    int                                num_rounds_;
    int                                num_streams_;
    int                                blob_layout_;
    int                                scheme_version_;
    // cells_[r][s] = bytes for (r, s)，初始空
    std::vector<std::vector<std::vector<uint8_t>>> cells_;
    std::vector<uint8_t>               filled_;  // R*S 掩码，防重复写
};

// --------------------------- Reader -----------------------------------------

/// @brief blob 解析器 —— 零拷贝视图，按 (round_id, stream_id) 取切片。
class ConnectivityBlobReader {
  public:
    /// @throws std::runtime_error 若 magic / header 损坏
    explicit ConnectivityBlobReader(const std::vector<uint8_t>& blob);
    /// @brief span overload；blob 必须在 Reader 生命期内保持有效。
    explicit ConnectivityBlobReader(std::span<const uint8_t> blob);

    const ConnectivityBlobHeader& header() const { return header_; }

    /// @return (round, stream) 的字节切片（零拷贝 view）。
    ///         若 (r,s) 越界，抛 std::out_of_range；空切片返回 size==0。
    std::span<const uint8_t> getStream(int round_id, int stream_id) const;

    /// @return round r 的所有子流合计字节数（= offsets[(r+1)*S] - offsets[r*S]）。
    size_t roundTotalBytes(int round_id) const;

    /// @return 轮数 / 子流数。
    int numRounds() const { return static_cast<int>(header_.num_rounds); }
    int numStreams() const { return static_cast<int>(header_.num_streams); }

  private:
    void                     parse();
    ConnectivityBlobHeader   header_;
    const uint8_t*           payload_ = nullptr;  // 指向 blob 内 payload 起点
    size_t                   payload_size_ = 0;
    // 保活（span overload 时指向外部；vector overload 时拷入）
    std::vector<uint8_t>     owned_;
    std::span<const uint8_t> view_;
};

}  // namespace pmsf

#endif  // PMSF_COMPRESS_CONNECTIVITY_BLOB_CONTAINER_H
