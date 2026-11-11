/*
 * @file ConquestSymbolReadOperator.cpp
 * @brief `ConquestSymbolReadOperator.h` 的实现。字节布局的唯一来源是
 * `src/conquest/ConquestBlobFormat.h` —— 本文件不另写一份解析逻辑。
 */
#include "Conquest/decompress/ConquestSymbolReadOperator.h"

#include <iostream>

namespace pmsf {

void ConquestSymbolReadOperator::importFromRangeCodedBuffer(char* buffer, size_t& dataOffset, unsigned nbQuantBits) {
    (void)nbQuantBits;  // 量化网格随流走，见头文件说明
    if (!buffer) {
        decodedOk_ = false;
        return;
    }
    // ⚠️ 上界必须是**调用方给的真实长度**，不能猜（见头文件 `setBufferSize` 的说明）。
    if (bufferSize_ <= dataOffset) {
        std::cerr << "[Conquest reader] **未设置/非法的缓冲区长度** bufferSize=" << bufferSize_
                  << " dataOffset=" << dataOffset << "\n";
        decodedOk_ = false;
        return;
    }
    conquest_blob::Stream s;
    if (!conquest_blob::readStream(buffer + dataOffset, bufferSize_ - dataOffset, &s)) {
        std::cerr << "[Conquest reader] **字节流解析失败**（magic/版本/长度不符）\n";
        decodedOk_ = false;
        return;
    }
    installDecodedStream(std::move(s));
}

void ConquestSymbolReadOperator::installDecodedStream(conquest_blob::Stream stream) {
    stream_ = std::move(stream);
    layerIndex_ = 0;
    decodedOk_ = true;
}

void ConquestSymbolReadOperator::collect(std::vector<MCGAL::Halfedge*> /*seeds*/,
                                         char* /*buffer*/,
                                         size_t& /*dataOffset*/,
                                         std::vector<MCGAL::Vertex*>& /*vertices*/,
                                         std::vector<MCGAL::Halfedge*>& /*halfedge*/,
                                         std::vector<MCGAL::Facet*>& /*facets*/) {
    // 本 layout 走"队列已装填"支（`paperBlobLayoutUsesDecodedQueues(15) == true`），
    // 框架不会调到这里；留着只是为满足纯虚契约。
}

void ConquestSymbolReadOperator::collect(MCGAL::Halfedge* /*seed*/,
                                         char* /*buffer*/,
                                         size_t& /*dataOffset*/,
                                         std::vector<MCGAL::Vertex*>& /*vertices*/,
                                         std::vector<MCGAL::Halfedge*>& /*halfedge*/,
                                         std::vector<MCGAL::Facet*>& /*facets*/) {
    // 同上。
}

void ConquestSymbolReadOperator::collectFromDecoded(MCGAL::Halfedge* /*seed*/,
                                                    std::vector<MCGAL::Vertex*>& /*vertices*/,
                                                    std::vector<MCGAL::Halfedge*>& /*halfedge*/,
                                                    std::vector<MCGAL::Facet*>& /*facets*/) {
    // 每轮一步：**不在这里解码**。重建器紧接着会被框架调用，它握着网格，
    // 由它调 `pmsf::decodeLayer`。这里让位是为了保持"解码实体只有一处"。
}

const conquest_blob::LayerRecord* ConquestSymbolReadOperator::takeNextLayer() {
    if (!decodedOk_ || layerIndex_ >= stream_.layers.size())
        return nullptr;
    return &stream_.layers[layerIndex_++];
}

}  // namespace pmsf
