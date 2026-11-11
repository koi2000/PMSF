#ifndef PMSF_DECOMPRESS_PAPER_READER_LAYOUT_REGISTRY_H
#define PMSF_DECOMPRESS_PAPER_READER_LAYOUT_REGISTRY_H

#include "BufferUtils.h"
#include "common/PaperLayoutAliases.h"
#include "decompress/SymbolReadOperator.h"
#include "Mesh.h"
#include "header/PMSFHeader.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂
#include "common/strict/PCATMStrictPMSFSymbolReadOperator.h"
#include "common/strict/PCLTTMStrictSymbolReadOperator.h"
#include "RD/decompress/RDStrictSymbolReadOperator.h"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <iostream>
#include <memory>
#include <vector>

namespace pmsf {

struct PMSFReaderImportResult {
    bool handled = false;
    bool initialized = true;
};

enum class PMSFReaderLayoutClass {
    Unknown,
    RegistryRangeCoded,
    RegistrySimpleUnpack,
    RegistryBufferTransform,
    SpecialUnpack,
    ReservedUnimplemented,
};

inline bool paperBlobLayoutUsesRangeCodedReaderRegistry(int layout) {
    switch (layout) {
        case kPaperBlobLayoutRangeCodedPred:
        case kPaperBlobLayoutValenceRC:
        case kPaperBlobLayoutDefaultPPMCLite:
        case kPaperBlobLayoutRDGate:
        case kPaperBlobLayoutRDGateOnly:
        case kPaperBlobLayoutValencePatchOnly:
        case kPaperBlobLayoutPCATMColor:
        case kPaperBlobLayoutPCLTTMTag:
        case kPaperBlobLayoutPCATMZColor:
        case kPaperBlobLayoutRDTag:
        case kPaperBlobLayoutPCATMPaper:
        case kPaperBlobLayoutVQMMCCodebook:
        case kPaperBlobLayoutMMSEDualRing:
        case kPaperBlobLayoutPCATMPure:
        case kPaperBlobLayoutPCATMStrictPaper:
        case kPaperBlobLayoutAPTMConquest:
        case kPaperBlobLayoutMMSEStrict:
        case kPaperBlobLayoutRDStrict:
            return true;
        default:
            return false;
    }
}

inline bool paperBlobLayoutUsesSimpleReaderRegistry(int layout) {
    return layout == kPaperBlobLayoutPPMCRC;
}

inline bool paperBlobLayoutUsesBufferTransformReaderRegistry(int layout) {
    return layout == kPaperBlobLayoutBasic ||
           layout == kPaperBlobLayoutOptimizedPlain;
}

inline PMSFReaderLayoutClass paperReaderLayoutClass(int layout) {
    if (paperBlobLayoutUsesRangeCodedReaderRegistry(layout)) {
        return PMSFReaderLayoutClass::RegistryRangeCoded;
    }
    if (paperBlobLayoutUsesSimpleReaderRegistry(layout)) {
        return PMSFReaderLayoutClass::RegistrySimpleUnpack;
    }
    if (paperBlobLayoutUsesBufferTransformReaderRegistry(layout)) {
        return PMSFReaderLayoutClass::RegistryBufferTransform;
    }

    switch (layout) {
        case kPaperBlobLayoutPPMCSegmentedRC:
        case kPaperBlobLayoutSegmentedRangeCodedPred:
            return PMSFReaderLayoutClass::SpecialUnpack;
        case kPaperBlobLayoutCPMEdgeSplit:
        case kPaperBlobLayoutPomar4Stream:
        case kPaperBlobLayoutRDBitplane:
            return PMSFReaderLayoutClass::ReservedUnimplemented;
        default:
            return PMSFReaderLayoutClass::Unknown;
    }
}

inline std::vector<std::size_t> paperReaderInitOffsetsForLayout(int layout) {
    switch (layout) {
        case kPaperBlobLayoutPCATMStrictPaper:
        case kPaperBlobLayoutAPTMConquest:
            return {0, 0};
        default:
            return {0, 0, 0};
    }
}

inline void attachPMSFReaderMeshIfRequired(
    int layout,
    ::SymbolReadOperator* reader,
    const std::shared_ptr<MCGAL::Mesh>& mesh) {
    switch (layout) {
        case kPaperBlobLayoutPCATMStrictPaper:
            if (auto* typed =
                    dynamic_cast<::PCATMStrictPMSFSymbolReadOperator*>(reader)) {
                typed->setMesh(mesh);
            }
            break;
        case kPaperBlobLayoutAPTMConquest:
            if (auto* typed = dynamic_cast<PCLTTMStrictSymbolReadOperator*>(reader)) {
                typed->setMesh(mesh);
            }
            break;
        default:
            break;
    }
}

inline PMSFReaderImportResult paperImportRangeCodedBlobForLayout(
    int layout,
    ::SymbolReadOperator* reader,
    char* blob,
    std::size_t& dataOffset,
    unsigned nbQuantBits,
    const ::PMSFHeader& header,
    const std::shared_ptr<MCGAL::Mesh>& mesh) {
    PMSFReaderImportResult result;
    if (reader == nullptr ||
        !paperBlobLayoutUsesRangeCodedReaderRegistry(layout)) {
        return result;
    }

    reader->init(paperReaderInitOffsetsForLayout(layout), header);
    reader->importFromRangeCodedBuffer(blob, dataOffset, nbQuantBits);
    attachPMSFReaderMeshIfRequired(layout, reader, mesh);
    result.handled = true;
    return result;
}

inline PMSFReaderImportResult paperImportBufferTransformForLayout(
    int layout,
    ::SymbolReadOperator* reader,
    std::vector<char>& blob,
    std::size_t& dataOffset,
    const ::PMSFHeader& header) {
    PMSFReaderImportResult result;
    if (reader == nullptr ||
        !paperBlobLayoutUsesBufferTransformReaderRegistry(layout)) {
        return result;
    }

    result.handled = true;

    if (layout == kPaperBlobLayoutBasic) {
        if (blob.size() < sizeof(int)) {
            std::cerr << "[Columnar] v2 BASIC blob 太短" << std::endl;
            result.initialized = false;
            return result;
        }

        int symbolSize = 0;
        std::memcpy(&symbolSize, blob.data(), sizeof(int));
        dataOffset = sizeof(int);
        std::size_t pointOffset =
            dataOffset + static_cast<std::size_t>(symbolSize);

        if (header.isEnableCompress()) {
            char* symBlock = nullptr;
            char* ptBlock = nullptr;
            int realSymSize = 0;
            int realPtSize = 0;
            deserializeCharPointer(blob.data(), dataOffset, symBlock, realSymSize);
            deserializeCharPointer(blob.data(), pointOffset, ptBlock, realPtSize);

            if (!symBlock || realSymSize <= 0) {
                std::cerr << "[Columnar] zlib 解压 symbol 段失败" << std::endl;
                if (symBlock) delete[] symBlock;
                if (ptBlock) delete[] ptBlock;
                result.initialized = false;
                return result;
            }

            std::vector<char> flat(static_cast<std::size_t>(realSymSize) +
                                   static_cast<std::size_t>(std::max(0, realPtSize)));
            std::memcpy(flat.data(), symBlock, realSymSize);
            if (realPtSize > 0 && ptBlock) {
                std::memcpy(flat.data() + realSymSize, ptBlock, realPtSize);
            }
            delete[] symBlock;
            if (ptBlock) delete[] ptBlock;

            blob = std::move(flat);
            dataOffset = 0;
            reader->init({static_cast<std::size_t>(realSymSize)}, header);
        } else {
            reader->init({pointOffset}, header);
        }
        return result;
    }

    if (layout == kPaperBlobLayoutOptimizedPlain) {
        std::size_t off = 0;
        int facetSymbolSize = 0;
        int edgeSymbolSize = 0;
        int vertexSymbolSize = 0;
        std::memcpy(&facetSymbolSize, blob.data() + off, sizeof(int));
        off += sizeof(int);
        std::memcpy(&edgeSymbolSize, blob.data() + off, sizeof(int));
        off += sizeof(int);
        if (header.isEnableAdjustment()) {
            std::memcpy(&vertexSymbolSize, blob.data() + off, sizeof(int));
            off += sizeof(int);
        }
        std::size_t off1 = off + facetSymbolSize;
        std::size_t off2 = off1 + edgeSymbolSize;
        std::size_t off3 = off2 + vertexSymbolSize;
        dataOffset = off;

        if (header.isEnableCompress()) {
            char* fb = nullptr;
            char* eb = nullptr;
            char* vb = nullptr;
            char* pb = nullptr;
            int rfs = 0;
            int res = 0;
            int rvs = 0;
            int rps = 0;
            deserializeCharPointer(blob.data(), off, fb, rfs);
            deserializeCharPointer(blob.data(), off1, eb, res);
            if (header.isEnableAdjustment()) {
                deserializeCharPointer(blob.data(), off2, vb, rvs);
            }
            deserializeCharPointer(blob.data(), off3, pb, rps);

            std::size_t total = static_cast<std::size_t>(std::max(0, rfs)) +
                                static_cast<std::size_t>(std::max(0, res)) +
                                static_cast<std::size_t>(std::max(0, rvs)) +
                                static_cast<std::size_t>(std::max(0, rps));
            std::vector<char> flat(total, 0);
            std::size_t p = 0;
            if (fb && rfs > 0) {
                std::memcpy(flat.data() + p, fb, rfs);
                p += rfs;
            }
            if (eb && res > 0) {
                std::memcpy(flat.data() + p, eb, res);
                p += res;
            }
            if (header.isEnableAdjustment() && vb && rvs > 0) {
                std::memcpy(flat.data() + p, vb, rvs);
                p += rvs;
            }
            if (pb && rps > 0) {
                std::memcpy(flat.data() + p, pb, rps);
            }
            delete[] fb;
            delete[] eb;
            if (vb) delete[] vb;
            delete[] pb;

            blob = std::move(flat);
            dataOffset = 0;
            std::size_t o1 = static_cast<std::size_t>(rfs);
            std::size_t o2 = o1 + static_cast<std::size_t>(res);
            std::size_t o3 = o2 + static_cast<std::size_t>(rvs);
            reader->init({o1, o2, o3}, header);
        } else {
            reader->init({off1, off2, off3}, header);
        }
    }

    return result;
}

inline PMSFReaderImportResult paperImportSimpleUnpackForLayout(
    int layout,
    ::SymbolReadOperator* reader,
    std::size_t& dataOffset,
    const ::PMSFHeader& header) {
    PMSFReaderImportResult result;
    if (reader == nullptr ||
        !paperBlobLayoutUsesSimpleReaderRegistry(layout)) {
        return result;
    }

    dataOffset = 0;
    reader->groupId = -1;
    reader->init({0}, header);
    result.handled = true;
    return result;
}

}  // namespace pmsf

#endif  // PMSF_DECOMPRESS_PAPER_READER_LAYOUT_REGISTRY_H
