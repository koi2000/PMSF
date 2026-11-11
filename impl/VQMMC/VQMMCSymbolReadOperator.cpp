#include "VQMMCSymbolReadOperator.h"
#include "BFSVersionManager.h"
#include "BufferUtils.h"
#include "Configuration.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Point.h"
#include "VQMMC/common/VQMMCQuantization.h"
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <stdexcept>
#include <vector>

void VQMMCSymbolReadOperator::collect(MCGAL::Halfedge* seed,
                                       char* buffer,
                                       size_t& dataOffset,
                                       std::vector<MCGAL::Vertex*>& vertices,
                                       std::vector<MCGAL::Halfedge*>& halfedge,
                                       std::vector<MCGAL::Facet*>& facets) {
    unsigned f_bitOffset = 0;
    unsigned e_bitOffset = 0;

    const auto requireBytes = [](size_t offset, size_t count, size_t end,
                                 const char* what) {
        if (offset > end || count > end - offset) {
            throw std::runtime_error(std::string("[VQMMC] truncated ") + what);
        }
    };
    const auto readBoundedInt = [&](size_t& offset, size_t end) {
        requireBytes(offset, sizeof(int), end, "VQ header");
        return readInt(buffer, offset);
    };
    const auto readBoundedChar = [&](size_t& offset, size_t end,
                                    const char* what) {
        requireBytes(offset, sizeof(char), end, what);
        return static_cast<unsigned char>(readChar(buffer, offset));
    };
    const auto readBoundedFloat = [&](size_t& offset, size_t end) {
        requireBytes(offset, sizeof(float), end, "VQ header");
        return readFloat(buffer, offset);
    };
    const auto readBoundedBits = [&](unsigned bits, unsigned& bitOffset,
                                     size_t& offset, size_t end) {
        const size_t occupied = (static_cast<size_t>(bitOffset) + bits + 7) / 8;
        requireBytes(offset, occupied, end, "VQ bitstream");
        uint32_t value = 0;
        for (unsigned i = 0; i < bits; ++i) {
            const unsigned position = bitOffset + i;
            const auto byte = static_cast<unsigned char>(
                buffer[offset + position / 8]);
            value = (value << 1) | ((byte >> (7 - position % 8)) & 1u);
        }
        offset += (bitOffset + bits) / 8;
        bitOffset = (bitOffset + bits) % 8;
        return value;
    };

    // ---- 读取本轮 VQ 头 ----
    int codebookSize = readBoundedInt(vqDataOffset_, vqEnd_);
    bool absoluteGeometry = false;
    if (codebookSize < 0) {
        absoluteGeometry = true;
    }
    const int codebookCount = pmsf::vqmmcCodebookCount(codebookSize);
    int numIndices = readBoundedInt(vqDataOffset_, vqEnd_);
    if (numIndices < 0) {
        throw std::runtime_error("[VQMMC] negative index count");
    }
    if (codebookCount == 0 && numIndices != 0) {
        throw std::runtime_error(
            "[VQMMC] non-empty index stream without a codebook");
    }
    std::vector<MCGAL::Point> codebook;
    int idxBits = 1;
    if (codebookCount > 0) {
        unsigned char cbBits =
            readBoundedChar(vqDataOffset_, vqEnd_, "codebook header");
        if (cbBits < 2 || cbBits > 16) {
            throw std::runtime_error("[VQMMC] invalid codebook coordinate bits");
        }
        float cMin[3];
        float cStep[3];
        for (int k = 0; k < 3; k++)
            cMin[k] = readBoundedFloat(vqDataOffset_, vqEnd_);
        for (int k = 0; k < 3; k++)
            cStep[k] = readBoundedFloat(vqDataOffset_, vqEnd_);

        codebook.resize(codebookCount);
        unsigned cb_bitOffset = 0;
        for (int j = 0; j < codebookCount; j++) {
            float coord[3];
            for (int k = 0; k < 3; k++) {
                uint32_t q = readBoundedBits(
                    cbBits, cb_bitOffset, vqDataOffset_, vqEnd_);
                coord[k] = cMin[k] + ((float)q) * cStep[k];
            }
            codebook[j] = MCGAL::Point(coord[0], coord[1], coord[2]);
        }
        if (cb_bitOffset != 0) {
            requireBytes(vqDataOffset_, 1, vqEnd_, "codebook padding");
            vqDataOffset_++;
        }

        idxBits = readBoundedChar(vqDataOffset_, vqEnd_, "index header");
        if (idxBits < 1 || idxBits > 16) {
            throw std::runtime_error("[VQMMC] invalid codebook index bits");
        }
        int minimumIdxBits = 1;
        while ((1 << minimumIdxBits) < codebookCount) {
            ++minimumIdxBits;
        }
        if (idxBits < minimumIdxBits) {
            throw std::runtime_error("[VQMMC] insufficient codebook index bits");
        }
    }
    const size_t indexBytes =
        (static_cast<size_t>(numIndices) * static_cast<size_t>(idxBits) + 7) / 8;
    requireBytes(vqDataOffset_, indexBytes, vqEnd_, "index stream");

    gateQueue_.push(seed->poolId());
    int current_version = MCGAL::BFSVersionManager::current_version++;

    unsigned idx_bitOffset = 0;
    int indexCount = 0;
    double residualAbsSum = 0.0;
    int residualSamples = 0;

    while (!gateQueue_.empty()) {
        int hid = gateQueue_.front();
        gateQueue_.pop();
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndexInSubPool(DEFAULT_MESH_ID, hid);

        if (h->isVisited(current_version) || h->isRemoved()) continue;
        h->setVisited(current_version);

        requireBytes(offsets[0], 1, edgeEnd_, "edge symbol stream");
        char esymbol = enableQuantization_
            ? static_cast<char>(
                  readBoundedBits(1, e_bitOffset, offsets[0], edgeEnd_))
            : readChar(buffer, offsets[0]);

        if (esymbol) {
            h->setAdded();
            halfedge.push_back(h);
        }

        MCGAL::Facet* f = h->face();
        if (!f->isVisited(current_version)) {
            f->setVisited(current_version);

            requireBytes(dataOffset, 1, facetEnd_, "facet symbol stream");
            char symbol = enableQuantization_
                ? static_cast<char>(
                      readBoundedBits(1, f_bitOffset, dataOffset, facetEnd_))
                : readChar(buffer, dataOffset);

            if (symbol) {
                f->setSplittable();

                if (indexCount >= numIndices) {
                    throw std::runtime_error(
                        "[VQMMC] facet index count exceeds round header");
                }
                uint32_t idx = readBoundedBits(
                    idxBits, idx_bitOffset, vqDataOffset_, vqEnd_);
                indexCount++;

                if (idx >= static_cast<uint32_t>(codebookCount)) {
                    throw std::runtime_error("[VQMMC] codebook index out of range");
                }
                MCGAL::Point residual = codebook[idx];
                residualAbsSum += std::abs(residual.x()) +
                                  std::abs(residual.y()) +
                                  std::abs(residual.z());
                residualSamples++;
                if (absoluteGeometry) {
                    f->setProtocolHint(21);
                }
                f->setRemovedVertexPos(residual);
                facets.push_back(f);
            }
        }

        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp != nullptr && !hOpp->isVisited(current_version)) {
                gateQueue_.push(hOpp->poolId());
            }
            if (!hIt->isVisited(current_version)) {
                gateQueue_.push(hIt->poolId());
            }
            hIt = hIt->next();
        } while (hIt != h);
    }

    if (idx_bitOffset != 0) {
        requireBytes(vqDataOffset_, 1, vqEnd_, "index padding");
        vqDataOffset_++;
    }
    if (indexCount != numIndices) {
        throw std::runtime_error("[VQMMC] round index count mismatch");
    }


    if (enableQuantization_) {
        requireBytes(dataOffset, 1, facetEnd_, "facet padding");
        requireBytes(offsets[0], 1, edgeEnd_, "edge padding");
        dataOffset++;
        offsets[0]++;
    }
}

void VQMMCSymbolReadOperator::collect(std::vector<MCGAL::Halfedge*> seeds,
                                       char* buffer,
                                       size_t& dataOffset,
                                       std::vector<MCGAL::Vertex*>& vertices,
                                       std::vector<MCGAL::Halfedge*>& halfedge,
                                       std::vector<MCGAL::Facet*>& facets) {
    if (!seeds.empty()) {
        collect(seeds[0], buffer, dataOffset, vertices, halfedge, facets);
    }
}
