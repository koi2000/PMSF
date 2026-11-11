/*
 * @Author: koi
 * @Date: 2025-09-10 22:22:37
 * @Description: 
 */
#ifndef ENTROPY_CODEC_H
#define ENTROPY_CODEC_H

#include <vector>
#include <cstdint>
#include <zlib.h>
#include <cstring>

inline bool zlibCompress(const uint8_t* inputData,
                         size_t inputSize,
                         std::vector<uint8_t>& outputData,
                         int level = Z_BEST_COMPRESSION) {
    if (inputSize == 0) {
        outputData.clear();
        return true;
    }
    uLongf bound = compressBound(static_cast<uLong>(inputSize));
    outputData.resize(bound);
    uLongf destLen = bound;
    int ret = compress2(reinterpret_cast<Bytef*>(outputData.data()),
                        &destLen,
                        reinterpret_cast<const Bytef*>(inputData),
                        static_cast<uLong>(inputSize),
                        level);
    if (ret != Z_OK) {
        outputData.clear();
        return false;
    }
    outputData.resize(destLen);
    return true;
}

inline bool zlibDecompress(const uint8_t* inputData,
                           size_t inputSize,
                           std::vector<uint8_t>& outputData,
                           size_t expectedOutputSize) {
    outputData.resize(expectedOutputSize);
    uLongf destLen = static_cast<uLongf>(expectedOutputSize);
    int ret = uncompress(reinterpret_cast<Bytef*>(outputData.data()),
                         &destLen,
                         reinterpret_cast<const Bytef*>(inputData),
                         static_cast<uLong>(inputSize));
    if (ret != Z_OK || destLen != expectedOutputSize) {
        outputData.clear();
        return false;
    }
    return true;
}

// Convenience APIs matching the requested form: (char*, size) -> (new char*, new size)
inline bool zlibCompressAlloc(const char* inputPtr,
                              int inputSize,
                              char*& outputPtr,
                              int& outputSize,
                              int level = Z_BEST_COMPRESSION) {
    outputPtr = nullptr;
    outputSize = 0;
    if (inputSize <= 0 || inputPtr == nullptr) {
        return true;
    }
    std::vector<uint8_t> compressed;
    bool ok = zlibCompress(reinterpret_cast<const uint8_t*>(inputPtr), static_cast<size_t>(inputSize), compressed, level);
    if (!ok) {
        return false;
    }
    outputSize = static_cast<int>(compressed.size());
    outputPtr = new char[outputSize];
    std::memcpy(outputPtr, compressed.data(), static_cast<size_t>(outputSize));
    return true;
}

inline bool zlibDecompressAlloc(const char* inputPtr,
                                int inputSize,
                                int expectedOutputSize,
                                char*& outputPtr,
                                int& outputSize) {
    outputPtr = nullptr;
    outputSize = 0;
    if (inputSize < 0 || expectedOutputSize < 0 || inputPtr == nullptr) {
        return false;
    }
    std::vector<uint8_t> decompressed;
    bool ok = zlibDecompress(reinterpret_cast<const uint8_t*>(inputPtr), static_cast<size_t>(inputSize), decompressed, static_cast<size_t>(expectedOutputSize));
    if (!ok || static_cast<int>(decompressed.size()) != expectedOutputSize) {
        return false;
    }
    outputSize = expectedOutputSize;
    outputPtr = new char[outputSize];
    std::memcpy(outputPtr, decompressed.data(), static_cast<size_t>(outputSize));
    return true;
}

#endif

