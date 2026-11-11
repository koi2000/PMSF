/*
 * @file        BitStream.cpp
 * @brief       BitWriter / BitReader 实现。
 * @author      koi
 * @date        2026-05-10
 *
 * 见 BitStream.h 头注释。实现要点：
 *   - buf_ 以 byte 为单位 append；bitInCurrentByte_ ∈ [0,7] 表示当前 byte 内
 *     已占用的低 bit 数。当 bitInCurrentByte_ == 0 且写入新 bit 时，push 一个 0x00
 *     到 buf_ 尾部。
 *   - writeBits 把输入 value 从 bit0 (LSB) 到 bit{nbits-1} 依次写入。
 *   - writeBits(value, 8) 的结果 == writeBytes(&byteOfValue, 1)（当流已对齐时）。
 */

#include "BitStream.h"

#include <cassert>
#include <cstring>
#include <stdexcept>

namespace pmsf {

// ----------------------------- BitWriter ------------------------------------

BitWriter::BitWriter()
    : buf_(), bitInCurrentByte_(0) {}

void BitWriter::writeBit(bool b) {
    if (bitInCurrentByte_ == 0) {
        buf_.push_back(0);
    }
    if (b) {
        buf_.back() |= static_cast<uint8_t>(1u << bitInCurrentByte_);
    }
    bitInCurrentByte_ = (bitInCurrentByte_ + 1) & 0x7;
}

void BitWriter::writeBits(uint64_t value, int nbits) {
    if (nbits <= 0 || nbits > 64) {
        throw std::invalid_argument("BitWriter::writeBits: nbits must be in [1, 64]");
    }
    // 逐 bit 写入（LSB first）。实现简单且避免跨 byte 边界的位移坑。
    // Phase 0 的 BitStream 不是热路径（真正热路径是 range-coder byte 流），
    // 这里优先正确性与可读性。
    for (int i = 0; i < nbits; ++i) {
        writeBit((value >> i) & 0x1ULL);
    }
}

void BitWriter::writeBytes(const void* data, size_t n) {
    // 对齐到 byte 边界
    if (bitInCurrentByte_ != 0) {
        bitInCurrentByte_ = 0;  // 当前 byte 剩下的高 bit 自然是 0，直接跳过
    }
    const uint8_t* p = static_cast<const uint8_t*>(data);
    buf_.insert(buf_.end(), p, p + n);
    // bitInCurrentByte_ 仍为 0（整字节追加不产生部分字节）
}

void BitWriter::writeVarint(uint64_t value) {
    // ULEB128
    while (value >= 0x80) {
        writeBits(static_cast<uint8_t>((value & 0x7FU) | 0x80U), 8);
        value >>= 7;
    }
    writeBits(static_cast<uint8_t>(value & 0x7FU), 8);
}

size_t BitWriter::sizeInBits() const {
    if (buf_.empty()) {
        return 0;
    }
    // 已完整填满的 byte 数 = buf_.size() - (bitInCurrentByte_ ? 1 : 0)
    // 再加上当前 byte 已占用的 bit 数。
    if (bitInCurrentByte_ == 0) {
        return buf_.size() * 8;
    }
    return (buf_.size() - 1) * 8 + static_cast<size_t>(bitInCurrentByte_);
}

size_t BitWriter::sizeInBytes() const {
    return buf_.size();
}

std::vector<uint8_t> BitWriter::finalize() {
    std::vector<uint8_t> out;
    out.swap(buf_);
    bitInCurrentByte_ = 0;
    return out;
}

// ----------------------------- BitReader ------------------------------------

BitReader::BitReader(const uint8_t* data, size_t size_in_bytes)
    : data_(data), total_bytes_(size_in_bytes), bytePos_(0), bitPos_(0) {}

bool BitReader::readBit() {
    if (bytePos_ >= total_bytes_) {
        throw std::out_of_range("BitReader::readBit: out of data");
    }
    bool b = (data_[bytePos_] >> bitPos_) & 0x1U;
    bitPos_ = (bitPos_ + 1) & 0x7;
    if (bitPos_ == 0) {
        ++bytePos_;
    }
    return b;
}

uint64_t BitReader::readBits(int nbits) {
    if (nbits <= 0 || nbits > 64) {
        throw std::invalid_argument("BitReader::readBits: nbits must be in [1, 64]");
    }
    uint64_t v = 0;
    for (int i = 0; i < nbits; ++i) {
        if (readBit()) {
            v |= (1ULL << i);
        }
    }
    return v;
}

void BitReader::readBytes(void* dst, size_t n) {
    // 对齐到 byte 边界
    if (bitPos_ != 0) {
        bitPos_ = 0;
        ++bytePos_;  // 丢弃当前半字节
    }
    if (bytePos_ + n > total_bytes_) {
        throw std::out_of_range("BitReader::readBytes: out of data");
    }
    std::memcpy(dst, data_ + bytePos_, n);
    bytePos_ += n;
}

uint64_t BitReader::readVarint() {
    uint64_t result = 0;
    int      shift  = 0;
    while (true) {
        uint64_t byte = readBits(8);
        result |= (byte & 0x7FULL) << shift;
        if ((byte & 0x80ULL) == 0) {
            break;
        }
        shift += 7;
        if (shift >= 64) {
            throw std::runtime_error("BitReader::readVarint: value exceeds uint64_t");
        }
    }
    return result;
}

size_t BitReader::bitsRemaining() const {
    if (bytePos_ >= total_bytes_) {
        return 0;
    }
    return (total_bytes_ - bytePos_) * 8 - static_cast<size_t>(bitPos_);
}

}  // namespace pmsf
