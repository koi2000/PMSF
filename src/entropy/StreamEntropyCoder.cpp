/*
 * @file StreamEntropyCoder.cpp
 * @brief 流熵编码器采集缝的实现（主计划 v8 §4 Phase S / R13）。
 */
#include "entropy/StreamEntropyCoder.h"

#include "scheme/StorageSchemeRegistry.h"

#include <cstdio>
#include <cstdlib>

namespace pmsf {

std::shared_ptr<EntropyCoder> injectedStreamCoder(const std::string& streamKind) {
    StorageSchemePtr scheme = activeStorageScheme();
    if (!scheme) return nullptr;  // 原始方案：交给调用方走原有路径
    EntropyCoderFactory factory = scheme->coderFactory();
    if (factory.empty()) return nullptr;  // 该方案没注入熵编码器
    return factory.create(streamKind);
}

}  // namespace pmsf
