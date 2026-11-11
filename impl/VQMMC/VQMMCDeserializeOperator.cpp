#include "VQMMCDeserializeOperator.h"
#include "common/BoundedRingTraversal.h"
#include "VQMMCStrictSymbolReadOperator.h"
#include "common/AlphabetStreamCodec.h"
#include "BufferUtils.h"
#include "Configuration.h"
#include "EntropyCodec.h"
#include "Halfedge.h"
#include "MeshUtils.h"
#include "Vertex.h"
#include <cstddef>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

std::shared_ptr<MCGAL::Mesh> VQMMCDeserializeOperator::deserializeBaseMesh() {
    std::shared_ptr<MCGAL::Mesh> mesh;
    if (header_.isEnableQuantization()) {
        const size_t quantMetadataOffset = dataOffset_;
        mesh = MCGAL::readBaseMeshWithQuantization(DEFAULT_MESH_ID, buffer_, dataOffset_);
        if (mesh) {
            mesh->i_nbQuantBits =
                static_cast<unsigned char>(buffer_[quantMetadataOffset]);
        }
    } else {
        mesh = MCGAL::readBaseMesh(DEFAULT_MESH_ID, buffer_, dataOffset_);
    }
    if (!mesh || mesh->vertices().size() < 2) {
        throw std::runtime_error("[VQMMC] failed to decode base mesh");
    }

    MCGAL::Vertex* v0 = mesh->vertices()[0];
    MCGAL::Vertex* v1 = mesh->vertices()[1];
    for (MCGAL::Halfedge* hit : pmsf::boundedRing(v0)) {
        if (hit->end_vertex() == v1) {
            seed_ = hit;
            break;
        }
    }
    if (!seed_) {
        throw std::runtime_error("[VQMMC] failed to locate base mesh seed");
    }

    this->mesh_ = mesh;
    reconstructOperator_->init(mesh);

    if (strictMode_) {
        // Strict mode: remaining buffer is a single ConnectivityBlobContainer.
        // Pass it directly to the strict reader — no zlib block splitting.
        if (dataOffset_ > totalSize) {
            throw std::runtime_error("[VQMMC] base mesh exceeds input buffer");
        }
        size_t blobSize = totalSize - dataOffset_;
        char* blobBuf = new char[blobSize];
        std::memcpy(blobBuf, buffer_ + dataOffset_, blobSize);
        delete[] buffer_;
        buffer_ = blobBuf;
        totalSize = blobSize;
        dataOffset_ = 0;

        symbolReadOperator_->init({0, 0, 0}, header_);
        auto* strictReader = dynamic_cast<VQMMCStrictSymbolReadOperator*>(
            vqSymbolReader_.get());
        if (strictReader)
            strictReader->setBlobSize(blobSize);
        return mesh;
    }

    // 读取三段 {origSize, compSize, data}，解压后拼接成一块连续 buffer
    auto readBlock = [this](std::vector<char>& out) {
        if (dataOffset_ > totalSize || totalSize - dataOffset_ < 2 * sizeof(int)) {
            throw std::runtime_error("[VQMMC] truncated block header");
        }
        int origSize = readInt(buffer_, dataOffset_);
        int compSize = readInt(buffer_, dataOffset_);
        if (origSize < 0 || compSize == std::numeric_limits<int>::min()) {
            throw std::runtime_error("[VQMMC] invalid block size");
        }
        const size_t payloadSize =
            compSize < 0 ? static_cast<size_t>(-compSize)
            : compSize > 0 ? static_cast<size_t>(compSize)
                           : static_cast<size_t>(origSize);
        if (payloadSize > totalSize - dataOffset_) {
            throw std::runtime_error("[VQMMC] block exceeds input buffer");
        }
        out.resize(static_cast<size_t>(origSize));
        if (origSize == 0) {
            dataOffset_ += payloadSize;
            return;
        }
        if (compSize < 0) {
            // P16：负值 = 自适应二元 RC；解出逐位相同的位数组（MSB-first）
            std::vector<uint8_t> bits = pmsf::decodeAlphabetStream(
                reinterpret_cast<const uint8_t*>(buffer_ + dataOffset_),
                payloadSize, 2);
            if (bits.size() != static_cast<size_t>(origSize) * 8) {
                throw std::runtime_error("[VQMMC] invalid bitstream block");
            }
            for (int i = 0; i < origSize; ++i) {
                uint8_t byte = 0;
                for (int b = 7; b >= 0; --b)
                    byte |= (uint8_t)(bits[(size_t)i * 8 + (7 - b)] << b);
                out[(size_t)i] = (char)byte;
            }
            dataOffset_ += (size_t)(-compSize);
        } else if (compSize == 0) {
            std::memcpy(out.data(), buffer_ + dataOffset_, (size_t)origSize);
            dataOffset_ += (size_t)origSize;
        } else {
            std::vector<uint8_t> decompressed;
            bool ok = zlibDecompress(reinterpret_cast<const uint8_t*>(buffer_ + dataOffset_),
                                     payloadSize,
                                     decompressed,
                                     static_cast<size_t>(origSize));
            if (!ok) {
                throw std::runtime_error("[VQMMC] failed to decompress block");
            }
            std::memcpy(out.data(), decompressed.data(), (size_t)origSize);
            dataOffset_ += (size_t)compSize;
        }
    };

    std::vector<char> facetBlock, edgeBlock, vqBlock;
    readBlock(facetBlock);
    readBlock(edgeBlock);
    readBlock(vqBlock);

    // 合并成单块 buffer，释放原始 buffer
    size_t newSize = facetBlock.size() + edgeBlock.size() + vqBlock.size();
    char* newBuffer = new char[newSize];
    if (!facetBlock.empty()) std::memcpy(newBuffer, facetBlock.data(), facetBlock.size());
    if (!edgeBlock.empty())  std::memcpy(newBuffer + facetBlock.size(), edgeBlock.data(), edgeBlock.size());
    if (!vqBlock.empty())    std::memcpy(newBuffer + facetBlock.size() + edgeBlock.size(), vqBlock.data(), vqBlock.size());

    delete[] buffer_;
    buffer_ = newBuffer;
    totalSize = newSize;
    dataOffset_ = 0;

    size_t facetStart = 0;
    size_t edgeStart = facetBlock.size();
    size_t vqStart = facetBlock.size() + edgeBlock.size();

    symbolReadOperator_->init({edgeStart, vqStart, vqStart}, header_);
    vqSymbolReader_->setVQDataOffset(vqStart);
    vqSymbolReader_->setStreamBounds(edgeStart, vqStart, newSize);

    // dataOffset_ 用于读取 facet 符号，从 0 开始（facet block 位于 buffer_ 开头）
    dataOffset_ = 0;

    return mesh;
}

void VQMMCDeserializeOperator::deserializeOneRound() {
    vertices_.clear();
    halfedges_.clear();
    facets_.clear();

    if (header_.isEnableQuantization()) {
        symbolReadOperator_->enableQuantization(mesh_->bboxMin, mesh_->f_quantStep);
    }

    symbolReadOperator_->collect(seed_, buffer_, dataOffset_, vertices_, halfedges_, facets_);

    postProcessor_->inverseProcess(mesh_, facets_);

    reconstructOperator_->reconstruct(vertices_, halfedges_, facets_);
}
