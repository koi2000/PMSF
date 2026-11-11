/*
 * @file        BitStream.h
 * @brief       Little-endian bit-packing writer / reader.
 * @author      koi
 * @date        2026-05-10
 *
 * Phase 0 · Step 0.2 of docs/CONNECTIVITY_BITSTREAM_IMPLEMENTATION_PLAN.md.
 *
 * 这对工具类是连通性比特流所有 5 套方案 (V / B / C / P / H) 的共用基础设施：
 *   - BitWriter 支持按 1..64 bit 为单位 append，并可插入字节对齐区域 /
 *     ULEB128 变长整数。finalize() 末尾补 0 使结果 byte 对齐。
 *   - BitReader 是逆操作；bitPos_ 永远从 MSB-first（同一字节内先写的 bit
 *     占据低 bit index）推进。
 *   - 字节序：多字节量 (uint16/32/64 经 writeBits) 由调用方拆成 bits；
 *     writeBytes 做透明 memcpy，由调用方保证 host-endian 与写入端口一致。
 *
 * bit-in-byte 方向约定（示例）：
 *   writeBit(1); writeBit(0); writeBit(1);  // 3 bits
 *   // 字节 buf_[0] 第 0 位 = 1，第 1 位 = 0，第 2 位 = 1, 其余 bit 待填
 *   // finalize() 后末位补 0。
 *
 * 这个约定与 src/rangecoder 里 range-coder 的 per-byte 输出**互不冲突**
 * （range-coder 产出是纯 byte 流），因此 BitStream 可以和 RC bytes 在
 * ConnectivityBlobContainer 内部并列共存而不需要对齐调整。
 */

#ifndef PMSF_UTILS_BITSTREAM_H
#define PMSF_UTILS_BITSTREAM_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace pmsf {

/// @brief Little-endian bit-packing writer. 调用 finalize() 拿到 byte vector。
class BitWriter {
  public:
    BitWriter();

    /// @brief 追加一位。
    void writeBit(bool b);

    /// @brief 追加 nbits 位的 value 到流尾。
    /// @param value 整数值，低 nbits 位被写入；高位忽略。
    /// @param nbits 写入位数，必须 ∈ [1, 64]。
    void writeBits(uint64_t value, int nbits);

    /// @brief 字节对齐追加裸字节。若当前位不对齐，则先补 0 到 byte 边界。
    void writeBytes(const void* data, size_t n);

    /// @brief 追加 ULEB128 变长整数。
    void writeVarint(uint64_t value);

    /// @brief 已写入的 bit 数（不含最后 padding）。
    size_t sizeInBits() const;

    /// @brief 已写入的 byte 数（若当前 bit 不对齐，向上取整，相当于 finalize 后的长度）。
    size_t sizeInBytes() const;

    /// @brief 冻结流，返回 byte vector（末尾补 0 到 byte 对齐）。
    ///        调用后对象回到空状态（可被复用或丢弃）。
    std::vector<uint8_t> finalize();

  private:
    std::vector<uint8_t> buf_;
    int bitInCurrentByte_;  // 下一个待写 bit 在当前 byte 内的位索引，0..7；==0 表示 byte 已满/新 byte
};

/// @brief BitWriter 的对偶读取器。
class BitReader {
  public:
    BitReader(const uint8_t* data, size_t size_in_bytes);

    bool     readBit();
    uint64_t readBits(int nbits);

    /// @brief 字节对齐读取；若当前位不对齐，先丢弃至 byte 边界。
    void readBytes(void* dst, size_t n);

    uint64_t readVarint();

    /// @brief 剩余可读 bit 数（考虑当前未对齐的字节头）。
    size_t bitsRemaining() const;

  private:
    const uint8_t* data_;
    size_t         total_bytes_;
    size_t         bytePos_;
    int            bitPos_;  // 下一个要读 bit 在 data_[bytePos_] 内的位索引，0..7
};

}  // namespace pmsf

#endif  // PMSF_UTILS_BITSTREAM_H
