#ifndef BUFFER_UTILS_H
#define BUFFER_UTILS_H
#include "EntropyCodec.h"
#include "core.h"
#include <assert.h>
#include <cstddef>
#include <cstring>  // For memcpy
#include <immintrin.h>
#include <stdint.h>
#include <unistd.h>
#include <vector>

static void writeCharPointer(char* buffer, size_t& dataOffset, char* data, int size) {
    std::memcpy(buffer + dataOffset, data, size * sizeof(char));
    dataOffset += size;
}

static void writeCharPointer(char* buffer, size_t& dataOffset, uint8_t* data, int size) {
    std::memcpy(buffer + dataOffset, data, size * sizeof(char));
    dataOffset += size;
}

static void readCharPointer(char* buffer, size_t& dataOffset, char* data, int size) {
    std::memcpy(data, buffer + dataOffset, size * sizeof(char));
    dataOffset += size;
}

static char* readCharPointer(char* buffer, size_t& dataOffset, int size) {
    char* pointer = buffer + dataOffset;
    dataOffset += size;
    return pointer;
}

// Write a given number of bits in a buffer.
static void writeBitsOff(uint32_t data, unsigned i_nbBits, char* p_dest, unsigned& i_bitOffset, size_t& offset) {
    assert(i_nbBits <= 25);

    uint32_t dataToAdd = data << (32 - i_nbBits - i_bitOffset);
    // Swap the integer bytes because the x86 architecture is little endian.
    dataToAdd = __builtin_bswap32(dataToAdd);  // Call a GCC builtin function.
    char* dst = p_dest + offset;
    // Write the data.
    *(uint32_t*)dst |= dataToAdd;

    // Update the size and offset.
    offset += (i_bitOffset + i_nbBits) / 8;
    i_bitOffset = (i_bitOffset + i_nbBits) % 8;
}

/**
 * Write a given number of bits in a buffer.
 */
static void writeBits(uint32_t data, unsigned i_nbBits, char* p_dest, unsigned& i_bitOffset, size_t& offset) {
    assert(i_nbBits <= 25);

    uint32_t dataToAdd = data << (32 - i_nbBits - i_bitOffset);
    // Swap the integer bytes because the x86 architecture is little endian.
    dataToAdd = __builtin_bswap32(dataToAdd);  // Call a GCC builtin function.

    // Write the data.
    *(uint32_t*)p_dest |= dataToAdd;

    // Update the size and offset.
    offset += (i_bitOffset + i_nbBits) / 8;
    i_bitOffset = (i_bitOffset + i_nbBits) % 8;
}

// /**
//  * Read a given number of bits in a buffer.
//  */
static uint32_t readBitsOff(unsigned i_nbBits, char* p_src, unsigned& i_bitOffset, size_t& offset) {
    assert(i_nbBits <= 25);

    // Build the mask.
    uint32_t mask = 0;
    for (unsigned i = 0; i < 32 - i_bitOffset; ++i)
        mask |= 1 << i;
    // Swap the mask bytes because the x86 architecture is little endian.
    mask = __builtin_bswap32(mask);  // Call a GCC builtin function.

    char* src = p_src + offset;
    uint32_t data = *(uint32_t*)src & mask;

    // Swap the integer bytes because the x86 architecture is little endian.
    data = __builtin_bswap32(data);  // Call a GCC builtin function.

    data >>= 32 - i_nbBits - i_bitOffset;

    // Update the size and offset.
    offset += (i_bitOffset + i_nbBits) / 8;
    i_bitOffset = (i_bitOffset + i_nbBits) % 8;

    return data;
}

/**
 * Read a given number of bits in a buffer.
 */
static uint32_t readBits(unsigned i_nbBits, char* p_src, unsigned& i_bitOffset, size_t& offset) {
    assert(i_nbBits <= 25);

    // Build the mask.
    uint32_t mask = 0;
    for (unsigned i = 0; i < 32 - i_bitOffset; ++i)
        mask |= 1 << i;
    // Swap the mask bytes because the x86 architecture is little endian.
    mask = __builtin_bswap32(mask);  // Call a GCC builtin function.

    uint32_t data = *(uint32_t*)p_src & mask;

    // Swap the integer bytes because the x86 architecture is little endian.
    data = __builtin_bswap32(data);  // Call a GCC builtin function.

    data >>= 32 - i_nbBits - i_bitOffset;

    // Update the size and offset.
    offset += (i_bitOffset + i_nbBits) / 8;
    i_bitOffset = (i_bitOffset + i_nbBits) % 8;

    return data;
}

static void writeBool(char* buffer, unsigned& i_bitOffset, size_t& dataOffset, bool b) {
    writeBitsOff((uint32_t)b, 1, buffer, i_bitOffset, dataOffset);
}

static bool readBool(char* buffer, unsigned& i_bitOffset, size_t& dataOffset) {
    bool b = readBitsOff(1, buffer, i_bitOffset, dataOffset);
    return b;
}

// Write a floating point number in the data buffer.
static void writeFloat(char* buffer, size_t& dataOffset, float f) {
    *(float*)(buffer + dataOffset) = f;
    dataOffset += sizeof(float);
}

// static void writeBits(uint32_t data, unsigned i_nbBits, char* p_dest, unsigned& i_bitOffset, size_t& offset) {
//     assert(i_nbBits <= 25);

//     for (unsigned i = 0; i < i_nbBits; ++i) {
//         unsigned bit = (data >> (i_nbBits - 1 - i)) & 1;
//         unsigned bytePos = offset + (i_bitOffset / 8);
//         unsigned bitPos = 7 - (i_bitOffset % 8); // 高位在前
//         p_dest[bytePos] &= ~(1 << bitPos); // 先清零
//         p_dest[bytePos] |= (bit << bitPos);
//         ++i_bitOffset;
//         if (i_bitOffset == 8) {
//             i_bitOffset = 0;
//             ++offset;
//         }
//     }
// }

// // 读取指定数量的比特从缓冲区
// static uint32_t readBits(unsigned i_nbBits, char* p_src, unsigned& i_bitOffset, size_t& offset) {
//     assert(i_nbBits <= 25);

//     uint32_t result = 0;
//     for (unsigned i = 0; i < i_nbBits; ++i) {
//         unsigned bytePos = offset + (i_bitOffset / 8);
//         unsigned bitPos = 7 - (i_bitOffset % 8); // 高位在前
//         unsigned bit = (p_src[bytePos] >> bitPos) & 1;
//         result = (result << 1) | bit;
//         ++i_bitOffset;
//         if (i_bitOffset == 8) {
//             i_bitOffset = 0;
//             ++offset;
//         }
//     }
//     return result;
// }

/**
 * Read a floating point number in the data buffer.
 */
static float readFloat(char* buffer, size_t& dataOffset) {
    float f = *(float*)(buffer + dataOffset);
    dataOffset += sizeof(float);
    return f;
}

// Write an integer in the data buffer
static void writeInt(char* buffer, size_t& dataOffset, int i) {
    *(int*)(buffer + dataOffset) = i;
    dataOffset += sizeof(int);
}

// Write a floating point number in the data buffer.
// static void writePoint(char* buffer, size_t& dataOffset, MCGAL::Point& p) {
//     for (unsigned i = 0; i < 3; ++i) {
//         writeFloat(buffer, dataOffset, p[i]);
//     }
// }

static void writePoint(char* buffer, size_t& dataOffset, MCGAL::Point p) {
    for (unsigned i = 0; i < 3; ++i) {
        writeFloat(buffer, dataOffset, p[i]);
    }
}

/**
 * Read an integer in the data buffer.
 */
static int readInt(char* buffer, size_t& dataOffset) {
    int i = *(int*)(buffer + dataOffset);
    dataOffset += sizeof(int);
    return i;
}

// Write a floating point number in the data buffer.
static MCGAL::Point readPoint(char* buffer, size_t& dataOffset) {
    float coord[3];
    // int id = readInt(buffer, dataOffset);
    for (unsigned i = 0; i < 3; ++i) {
        coord[i] = readFloat(buffer, dataOffset);
    }
    MCGAL::Point pt(coord[0], coord[1], coord[2]);
    return pt;
}

/**
 * Read a 16 bits integer in the data buffer.
 */
static int16_t readInt16(char* buffer, size_t& dataOffset) {
    int16_t i = *(int16_t*)(buffer + dataOffset);
    dataOffset += sizeof(int16_t);
    return i;
}

// Write a 16 bits integer in the data buffer
static void writeInt16(char* buffer, size_t& dataOffset, int16_t i) {
    *(int16_t*)(buffer + dataOffset) = i;
    dataOffset += sizeof(int16_t);
}

/**
 * Read a 16 bits integer in the data buffer.
 */
static uint16_t readuInt16(char* buffer, size_t& dataOffset) {
    uint16_t i = *(uint16_t*)(buffer + dataOffset);
    dataOffset += sizeof(uint16_t);
    return i;
}

// Write a 16 bits integer in the data buffer
static void writeuInt16(char* buffer, size_t& dataOffset, uint16_t i) {
    *(uint16_t*)(buffer + dataOffset) = i;
    dataOffset += sizeof(uint16_t);
}

/**
 * Read a byte in the data buffer.
 */
static char readChar(char* buffer, size_t& dataOffset) {
    // _mm_prefetch((char*)(buffer + dataOffset + 32), _MM_HINT_T0);
    char i = *(char*)(buffer + dataOffset);
    dataOffset += sizeof(char);
    return i;
}

static unsigned char readUChar(char* buffer, size_t& dataOffset) {
    // _mm_prefetch((char*)(buffer + dataOffset + 32), _MM_HINT_T0);
    unsigned char i = *(unsigned char*)(buffer + dataOffset);
    dataOffset += sizeof(unsigned char);
    return i;
}

static unsigned char readCharLocalOffset(char* buffer, size_t& dataOffset) {
    // _mm_prefetch((char*)(buffer + dataOffset + 32), _MM_HINT_T0);
    unsigned char i = *(unsigned char*)(buffer + dataOffset);
    dataOffset += sizeof(unsigned char);
    return i;
}

// Write a byte in the data buffer
static void writeChar(char* buffer, size_t& dataOffset, unsigned char i) {
    *(unsigned char*)(buffer + dataOffset) = i;
    dataOffset += sizeof(unsigned char);
}

static void writeChar(unsigned char* buffer, size_t& dataOffset, unsigned char i) {
    *(unsigned char*)(buffer + dataOffset) = i;
    dataOffset += sizeof(unsigned char);
}

static float readFloatByOffset(char* buffer, size_t offset) {
    float f = *(float*)(buffer + offset);
    return f;
}

static int16_t readInt16ByOffset(char* buffer, size_t offset) {
    int16_t i = *(int16_t*)(buffer + offset);
    return i;
}

static uint16_t readuInt16ByOffset(char* buffer, size_t offset) {
    uint16_t i = *(uint16_t*)(buffer + offset);
    return i;
}

static int readIntByOffset(char* buffer, size_t offset) {
    int i = *(int*)(buffer + offset);
    return i;
}

static unsigned char readCharByOffset(char* buffer, size_t offset) {
    unsigned char i = *(unsigned char*)(buffer + offset);
    return i;
}

static MCGAL::Point readPointByOffset(char* buffer, size_t offset) {
    float coord[3];
    int id = readIntByOffset(buffer, offset);
    offset += sizeof(int);
    for (unsigned i = 0; i < 3; ++i) {
        coord[i] = readFloatByOffset(buffer, offset);
        offset += sizeof(float);
    }
    MCGAL::Point pt(coord[0], coord[1], coord[2]);
    return pt;
}

static float readFloatByOffsetRef(char* buffer, size_t& offset) {
    float f = *(float*)(buffer + offset);
    offset += sizeof(float);
    return f;
}

static int16_t readInt16ByOffsetRef(char* buffer, size_t& offset) {
    int16_t i = *(int16_t*)(buffer + offset);
    offset += sizeof(int16_t);
    return i;
}

static uint16_t readuInt16ByOffsetRef(char* buffer, size_t& offset) {
    uint16_t i = *(uint16_t*)(buffer + offset);
    offset += sizeof(uint16_t);
    return i;
}

static int readIntByOffsetRef(char* buffer, size_t& offset) {
    int i = *(int*)(buffer + offset);
    offset += sizeof(int);
    return i;
}

static unsigned char readCharByOffsetRef(char* buffer, size_t& offset) {
    unsigned char i = *(unsigned char*)(buffer + offset);
    offset += sizeof(unsigned char);
    return i;
}

static MCGAL::Point readPointByOffsetRef(char* buffer, size_t& offset) {
    float coord[3];
    int id = readIntByOffset(buffer, offset);
    offset += sizeof(int);
    for (unsigned i = 0; i < 3; ++i) {
        coord[i] = readFloatByOffset(buffer, offset);
        offset += sizeof(float);
    }
    MCGAL::Point pt(coord[0], coord[1], coord[2]);
    return pt;
}

// Write a compressed block as: [origSize:int][compSize:int][payload]
// If compression fails or doesn't save space, writes uncompressed data (compSize=0)
static void writeCompressedBlock(char* buffer, size_t& dataOffset, const char* data, int dataSize) {
    if (dataSize <= 0 || data == nullptr) {
        writeInt(buffer, dataOffset, 0);
        writeInt(buffer, dataOffset, 0);
        return;
    }

    // Try to compress the data
    std::vector<uint8_t> compressed;
    bool compressionOk = zlibCompress(reinterpret_cast<const uint8_t*>(data), static_cast<size_t>(dataSize), compressed, Z_BEST_COMPRESSION);

    // If compression failed or didn't save space, store uncompressed
    if (!compressionOk || compressed.size() >= static_cast<size_t>(dataSize)) {
        writeInt(buffer, dataOffset, dataSize);
        writeInt(buffer, dataOffset, 0);  // compSize = 0 means uncompressed
        writeCharPointer(buffer, dataOffset, const_cast<char*>(data), dataSize);
    } else {
        // Store compressed data
        writeInt(buffer, dataOffset, dataSize);
        writeInt(buffer, dataOffset, static_cast<int>(compressed.size()));
        writeCharPointer(buffer, dataOffset, reinterpret_cast<char*>(compressed.data()), static_cast<int>(compressed.size()));
    }
}

// Read a possibly compressed block written as: [origSize:int][compSize:int][payload]
// If compSize == 0, payload length == origSize and is uncompressed.
// Returns newly allocated char* of length origSize; caller owns memory.
static char* readCompressedBlock(char* buffer, size_t& dataOffset) {
    int origSize = readInt(buffer, dataOffset);
    int compSize = readInt(buffer, dataOffset);
    if (origSize <= 0) {
        return nullptr;
    }
    if (compSize == 0) {
        char* out = new char[origSize];
        readCharPointer(buffer, dataOffset, out, origSize);
        return out;
    }
    const unsigned char* compPtr = reinterpret_cast<unsigned char*>(buffer + dataOffset);
    dataOffset += compSize;
    std::vector<uint8_t> decompressed;
    bool ok = zlibDecompress(compPtr, static_cast<size_t>(compSize), decompressed, static_cast<size_t>(origSize));
    if (!ok || decompressed.size() != static_cast<size_t>(origSize)) {
        return nullptr;
    }
    char* out = new char[origSize];
    std::memcpy(out, decompressed.data(), static_cast<size_t>(origSize));
    return out;
}

static void serializeCharPointer(char* val, int size, char*& outBlock, int& outBlockSize) {
    outBlock = nullptr;
    outBlockSize = 0;
    if (size <= 0 || val == nullptr) {
        outBlockSize = sizeof(int) * 2;
        outBlock = new char[outBlockSize];
        size_t localOffset = 0;
        writeInt(outBlock, localOffset, 0);
        writeInt(outBlock, localOffset, 0);
        return;
    }
    std::vector<uint8_t> compressed;
    bool ok = zlibCompress(reinterpret_cast<const uint8_t*>(val), static_cast<size_t>(size), compressed);
    int compSize = (!ok || compressed.size() >= static_cast<size_t>(size)) ? 0 : static_cast<int>(compressed.size());

    outBlockSize = static_cast<int>(sizeof(int) * 2 + (compSize == 0 ? size : compSize));
    outBlock = new char[outBlockSize];
    size_t localOffset = 0;
    writeInt(outBlock, localOffset, size);
    writeInt(outBlock, localOffset, compSize);
    if (compSize == 0) {
        writeCharPointer(outBlock, localOffset, reinterpret_cast<uint8_t*>(val), size);
    } else {
        writeCharPointer(outBlock, localOffset, compressed.data(), compSize);
    }
}

static void deserializeCharPointer(char* val, int size, char*& outBlock, int& outBlockSize) {
    outBlock = nullptr;
    outBlockSize = 0;

    size_t localOffset = 0;
    int origSize = readInt(val + size, localOffset);
    int compSize = readInt(val + size, localOffset);
    if (origSize <= 0) {
        outBlock = nullptr;
        return;
    }
    if (compSize == 0) {
        outBlock = new char[origSize];
        outBlockSize = origSize;
        std::memcpy(outBlock, val + size + localOffset, static_cast<size_t>(origSize));
        return;
    }
    char* outPtr = nullptr;
    int outSize = 0;
    bool ok = zlibDecompressAlloc(val + size + localOffset, compSize, origSize, outPtr, outSize);
    if (!ok || outSize != origSize) {
        outBlock = nullptr;
        return;
    }
    outBlock = outPtr;
    outBlockSize = outSize;
}
#endif