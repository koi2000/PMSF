/*
 * @Author: pomar-refactor
 * @Description: Pomar 存储模块重构版 —— 序列化操作符实现
 *
 * 主要步骤（serialize(path)）：
 *   1. 从 PomarColumnarSymbolCollectOperator 拿到五类符号队列与 cluster 元信息；
 *   2. determineCoderValues：统计 min/max/range；
 *   3. 先 compress 全局 cluster (id=0)，比特流直接写入 buffer_；
 *   4. 若 random-access 开启，再把剩余 cluster (id>=1) 写入独立缓冲；
 *   5. writeClusterSizes 把 cluster meta 写入主 buffer_；
 *   6. 把 cluster 比特流拷回主 buffer_；
 *   7. 按框架惯例：writeInt(len) + buffer_ 写出到文件。
 *
 * base mesh 的写入移至独立函数 serializeBaseMeshWithSeed，
 * 与其他实现（PPMCSerializeOperator）保持形参风格一致。
 */

#include "POMAR/compress/PomarColumnarSerializeOperator.h"
#include "common/BoundedRingTraversal.h"
#include "common/DebugTool.h"
#include "POMAR/compress/PomarColumnarSymbolCollectOperator.h"

#include "BufferUtils.h"
#include "compress/RangeCoderStreamEncoder.h"
#include "Configuration.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Point.h"
#include "Vertex.h"
#include "storage/PMSFUnifiedArtifactWriter.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>

// P36 物品3：`BUFFER_SIZE` 是 **600 MB**（deps/MCGAL/include/Configuration.h:53）。
// 原先三处都是 `new char[BUFFER_SIZE]` 紧跟 `std::memset(…, 0, BUFFER_SIZE)`
// —— **急切零化 600 MB**，perf 显示它连同首页缺页占 POMAR enc 的 ~7%（最大单项）。
//
// 改法的**语义与原来逐位相同**（仍是「一块全零的 600 MB 缓冲」），只是让内核
// 直接交出**已零化的惰性页**，从而省掉急切 memset 与其缺页：
//   · `calloc` 对大块走 mmap，匿名页由内核保证为零 ⇒ **不写就不缺页**；
//   · 原实现 `new[] + memset` 会**立刻触碰全部 600 MB**，无论用不用得到。
// 注意不能只删 memset 了事：range coder 的收尾可能读到未写区，
// 那些字节必须**确定为零**（这也是当初写 memset 的原因），`calloc` 保留了这个保证。
static char* pomarZeroBuffer() {
    return static_cast<char*>(std::calloc(BUFFER_SIZE, 1));
}

static bool pomarSizeBreakdownEnabled() {
    const char* value = std::getenv("PMSF_POMAR_SIZE_BREAKDOWN");
    return value && *value && std::string(value) != "0";
}

std::shared_ptr<PomarColumnarSerializeOperator::PomarCollector> PomarColumnarSerializeOperator::pomarCollector() const {
    auto c = std::dynamic_pointer_cast<PomarCollector>(collector_);
    assert(c != nullptr && "PomarColumnarSerializeOperator 要求传入 PomarColumnarSymbolCollectOperator");
    return c;
}

void PomarColumnarSerializeOperator::initRangeCoders() {
    rangeCoder_.p_data = buffer_;
    rangeCoder_.p_dataOffset = &dataOffset_;

    // Mes 侧只用于可选的 geometry bit breakdown；默认关闭以免重复编码。
    if (pomarSizeBreakdownEnabled()) {
        p_dataMes_ = pomarZeroBuffer();
        p_dataOffsetMes_ = new size_t(0);
        rangeCoderMes_.p_data = p_dataMes_;
        rangeCoderMes_.p_dataOffset = p_dataOffsetMes_;
    }
}

void PomarColumnarSerializeOperator::freeAuxBuffers() {
    std::free(p_dataMes_);
    delete p_dataOffsetMes_;
    p_dataMes_ = nullptr;
    p_dataOffsetMes_ = nullptr;

    std::free(p_dataClusters_);
    delete p_dataOffsetClusters_;
    p_dataClusters_ = nullptr;
    p_dataOffsetClusters_ = nullptr;
}

void PomarColumnarSerializeOperator::init() {
    // Called before elimination: garbage_collection later changes mesh counts.
    const auto mesh = collector_ ? collector_->getMesh() : nullptr;
    if (!mesh) {
        throw std::runtime_error("[POMAR] serializer requires the original mesh");
    }
    originalVertices_ = static_cast<unsigned>(mesh->size_of_vertices());
    originalFaces_ = static_cast<unsigned>(mesh->size_of_facets());
    buffer_ = pomarZeroBuffer();
    dataOffset_ = 0;
}

void PomarColumnarSerializeOperator::serializeInt(int val) {
    writeInt(buffer_, dataOffset_, val);
}

void PomarColumnarSerializeOperator::serializeBaseMesh(std::shared_ptr<MCGAL::Mesh> /*mesh*/) {
    // Pomar 在 base mesh 中需要 seed，所以使用 serializeBaseMeshWithSeed 版本
}

void PomarColumnarSerializeOperator::serializeBaseMeshWithSeed(std::shared_ptr<MCGAL::Mesh> mesh,
                                                         std::vector<MCGAL::Halfedge*> /*seeds*/) {
    // 先做 garbage collection + 量化，保证后续写出的基网格坐标位于
    // [0, 1 << i_nbQuantBits) 范围内。
    mesh->garbage_collection();
    mesh->quantizeVertexPositions();

    // ---- bounding box / quantization ----
    writeChar(buffer_, dataOffset_, 1);  // codec version
    i_connectivitySize_ += 8;

    writePoint(buffer_, dataOffset_, mesh->bboxMin);
    writeFloat(buffer_, dataOffset_, mesh->f_quantStep);
    i_geometrySize_ += 4 * sizeof(float) * 8;

    i_nbVerticesBaseMesh_ = mesh->size_of_vertices();
    unsigned i_nbFacesBaseMesh = mesh->size_of_facets();
    unsigned i_nbBitsPerVertex = (unsigned)std::ceil(std::log((double)i_nbVerticesBaseMesh_) / std::log(2.0));

    unsigned i_bitOffset = 0;
    dataOffset_++;

    // geometry quantization bits
    assert(mesh->i_nbQuantBits - 1 < 1 << 4);
    writeBits(mesh->i_nbQuantBits - 1, 4, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
    i_geometrySize_ += 4;

    // vertex / face 数量（各 16 bits）
    writeBits(i_nbVerticesBaseMesh_, 16, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
    writeBits(i_nbFacesBaseMesh, 16, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
    i_connectivitySize_ += 32;

    // vertices 坐标
    unsigned i_id = 0;
    std::map<int, unsigned> vertexMap;
    for (MCGAL::Vertex* v_it : mesh->vertices()) {
        if (v_it->isRemoved()) {
            continue;
        }
        MCGAL::PointInt p = mesh->floatPosToInt(v_it->point());
        for (unsigned i = 0; i < 3; ++i) {
            assert(p[i] < 1 << mesh->i_nbQuantBits);
            writeBits(p[i], mesh->i_nbQuantBits, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
        }
        vertexMap[v_it->poolId()] = i_id++;
    }
    i_geometrySize_ += i_nbVerticesBaseMesh_ * 3 * mesh->i_nbQuantBits;

    // faces 拓扑（顶点 idx 三元组）
    bool b_firstFace = true;
    MCGAL::Vertex* vs[3] = {nullptr, nullptr, nullptr};
    for (MCGAL::Facet* f_it : mesh->faces()) {
        if (f_it->isRemoved()) {
            continue;
        }
        int cnt = 0;
        for (MCGAL::Halfedge* fh_it : *f_it) {
            uint32_t id = vertexMap[fh_it->vertex()->poolId()];
            writeBits(id, i_nbBitsPerVertex, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
            if (b_firstFace) {
                vs[cnt++] = fh_it->vertex();
            }
        }
        b_firstFace = false;
        i_connectivitySize_ += i_nbBitsPerVertex * 3;
    }

    // Pomar global/clustered LOD
    auto collector = pomarCollector();
    i_nbGlobalLOD_ = collector->getNbGlobalLOD();
    i_nbClusteredLOD_ = collector->getNbClusteredLOD();

    assert((int)i_nbGlobalLOD_ < 1 << 6);
    writeBits(i_nbGlobalLOD_, 6, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
    assert((int)i_nbClusteredLOD_ < 1 << 6);
    writeBits(i_nbClusteredLOD_, 6, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
    i_connectivitySize_ += 12;

    if (i_bitOffset == 0) {
        dataOffset_--;
    }
}

void PomarColumnarSerializeOperator::serialize(std::string path) {
    auto collector = pomarCollector();

    const auto& nbSplitSymbols = collector->getNbSplitSymbols();
    const auto& nbSkipsHehSymbols = collector->getNbSkipsHehSymbols();
    const auto& vertexSplitSymbols = collector->getVertexSplitSymbols();
    const auto& geometrySymbols = collector->getGeometrySymbols();
    const auto& geometryNormalSymbols = collector->getGeometryNormalSymbols();

    // P8/D12 对拍抓手（**默认关闭，零行为变化**）：把编码端 vertexSplitSymbols 的
    // **值序列**落到 `$PMSF_POMAR_DUMP_DIR/encode_vertexSplitSymbols.txt`，与解码端
    // 同名文件 diff，用来区分"序列不同源（传输/口径错误）"与"序列同源但被误消费"。
    //
    // 前提核实（2026-09-22）：原 `PomarSerializeOperator.cpp:282` 的 DEBU_DUMP_SYMBOL
    // 整段在 `#ifdef DEBUG` 内 ⇒ **Release（门禁构建）下是空实现、产不出文件**；且
    // 那条在**非 columnar 的遗留路径**上，与本行（columnar）不是同一条。故此处补通道。
    // dump 的正是下一行本就要用的同一批引用，不额外计算。
    DUMP_SYMBOL_IF_ENV(collector->getVertexSplitSymbols(), "PMSF_POMAR_DUMP_DIR",
                       "encode_vertexSplitSymbols.txt");

    i_nbClusters_ = collector->getNbClusters();
    clusterSizes_.assign(i_nbClusters_, 0);

    determineCoderValues(nbSplitSymbols, nbSkipsHehSymbols, vertexSplitSymbols, geometrySymbols, geometryNormalSymbols);

    initRangeCoders();

    // 记录 base mesh 段结束（initRangeCoders 之后 dataOffset_ 已指向 cluster 0 起点）
    baseMeshEndOffset_ = dataOffset_;

    // 先编码 0 号 cluster，比特流直接落在主 buffer_ 里
    compressCluster(0, nbSplitSymbols, nbSkipsHehSymbols, vertexSplitSymbols, geometrySymbols, geometryNormalSymbols);
    cluster0EndOffset_ = dataOffset_;

    if (b_randomAccess_ && i_nbClusters_ > 1) {
        // 剩余 cluster 先写到临时 buffer 里
        p_dataClusters_ = pomarZeroBuffer();
        p_dataOffsetClusters_ = new size_t(0);

        rangeCoder_.p_data = p_dataClusters_;
        rangeCoder_.p_dataOffset = p_dataOffsetClusters_;

        for (unsigned i_clusterId = 1; i_clusterId < i_nbClusters_; ++i_clusterId) {
            compressCluster(i_clusterId, nbSplitSymbols, nbSkipsHehSymbols, vertexSplitSymbols, geometrySymbols, geometryNormalSymbols);
        }

        // 回到主缓冲：先写 cluster sizes 以便解压时随机访问
        rangeCoder_.p_data = buffer_;
        rangeCoder_.p_dataOffset = &dataOffset_;
        writeClusterSizes();
        clusterMetaEndOffset_ = dataOffset_;

        // 追加 cluster 数据
        std::memcpy(buffer_ + dataOffset_, p_dataClusters_, *p_dataOffsetClusters_);
        dataOffset_ += *p_dataOffsetClusters_;
    } else {
        clusterMetaEndOffset_ = cluster0EndOffset_;
    }

    // ---- 写入文件：长度前缀 + buffer_ 内容（与 BasicSerializeOperator 一致） ----
    if (!skipFileWrite_) {
        writeLengthPrefixedLegacySingleFile(path, buffer_, dataOffset_, 0);
        freeAuxBuffers();
    }
    // 注意：skipFileWrite_ 时 buffer_ 由 wrapper 负责释放（freeAuxBuffers 也被推迟），
    // 因为 wrapper 需要在 serialize() 返回后继续读取 buffer_ + clusterSizes_。
}

// ---- cluster meta ----

void PomarColumnarSerializeOperator::writeClusterSizes() {
    unsigned i_maxSize = *std::max_element(++(clusterSizes_.begin()), clusterSizes_.end());
    unsigned i_nbBitsPerClusterSize = (unsigned)std::ceil(std::log((double)(i_maxSize + 1)) / std::log(2.0));

    unsigned i_bitOffset = 0;
    dataOffset_++;

    assert(i_nbBitsPerClusterSize < 1 << 4);
    writeBits(i_nbBitsPerClusterSize, 4, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
    i_connectivitySize_ += 4;

    for (unsigned i_clusterId = 1; i_clusterId < i_nbClusters_; ++i_clusterId) {
        assert((int)clusterSizes_[i_clusterId] < 1 << i_nbBitsPerClusterSize);
        writeBits(clusterSizes_[i_clusterId], i_nbBitsPerClusterSize, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
    }
    i_connectivitySize_ += (i_nbClusters_ - 1) * i_nbBitsPerClusterSize;

    assert(i_nbBitsNbSplits_ < 1 << 3);
    writeBits(i_nbBitsNbSplits_, 3, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
    assert(i_nbBitsNbSkips_ < 1 << 3);
    writeBits(i_nbBitsNbSkips_, 3, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
    assert(i_nbBitsVertexSplits_ < 1 << 3);
    writeBits(i_nbBitsVertexSplits_, 3, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
    assert(i_nbBitsMinResidual_ < 1 << 5);
    writeBits(i_nbBitsMinResidual_, 5, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
    assert(i_nbBitsResidualRange_ < 1 << 5);
    writeBits(i_nbBitsResidualRange_, 5, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
    assert(i_nbBitsMinResidualNormal_ < 1 << 5);
    writeBits(i_nbBitsMinResidualNormal_, 5, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
    assert(i_nbBitsResidualNormalRange_ < 1 << 5);
    writeBits(i_nbBitsResidualNormalRange_, 5, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);

    if (i_bitOffset == 0) {
        dataOffset_--;
    }

    assert(i_minMinResidual_ > -(1 << 15));
    writeInt16(buffer_, dataOffset_, i_minMinResidual_);
    assert(i_minMinResidualNormal_ > -(1 << 15));
    writeInt16(buffer_, dataOffset_, i_minMinResidualNormal_);
}

// ---- 熵编码主体 ----

void PomarColumnarSerializeOperator::compressCluster(unsigned i_clusterId,
                                                const std::vector<std::deque<unsigned>>& nbSplitSymbols,
                                                const std::vector<std::deque<unsigned>>& nbSkipsHehSymbols,
                                                const std::vector<std::deque<unsigned>>& vertexSplitSymbols,
                                                const std::vector<std::deque<int>>& geometrySymbols,
                                                const std::vector<std::deque<int>>& geometryNormalSymbols) {
    pmsf::MultiModelRangeEncoder encoder(rangeCoder_.p_data, rangeCoder_.p_dataOffset);
    std::unique_ptr<pmsf::MultiModelRangeEncoder> mesEncoder;
    if (pomarSizeBreakdownEnabled()) {
        mesEncoder = std::make_unique<pmsf::MultiModelRangeEncoder>(
            rangeCoderMes_.p_data, rangeCoderMes_.p_dataOffset);
    }

    ModelIds mainIds;
    ModelIds mesIds;
    initAndEncodeModels(i_clusterId, encoder, mesEncoder.get(), mainIds, mesIds);

    unsigned i_len = nbSplitSymbols[i_clusterId].size();
    unsigned i_splitIndex = 0;
    for (unsigned i = 0; i < i_len; ++i) {
        unsigned i_nbSplits = nbSplitSymbols[i_clusterId][i];
        encoder.encode(mainIds.nbSplit, static_cast<int>(i_nbSplits));

        for (unsigned j = 0; j < i_nbSplits; ++j) {
            encoder.encode(mainIds.nbSkips,
                           static_cast<int>(nbSkipsHehSymbols[i_clusterId][i_splitIndex]));

            encoder.encode(mainIds.vertexSplit,
                           static_cast<int>(vertexSplitSymbols[i_clusterId][2 * i_splitIndex]));
            encoder.encode(mainIds.vertexSplit,
                           static_cast<int>(vertexSplitSymbols[i_clusterId][2 * i_splitIndex + 1]));

            unsigned tang0 = geometrySymbols[i_clusterId][2 * i_splitIndex] - i_minResidual_;
            unsigned tang1 = geometrySymbols[i_clusterId][2 * i_splitIndex + 1] - i_minResidual_;
            assert(tang0 < residualRanges_[i_clusterId]);
            assert(tang1 < residualRanges_[i_clusterId]);
            encoder.encode(mainIds.geometry, static_cast<int>(tang0));
            if (mesEncoder) {
                mesEncoder->encode(mesIds.geometry, static_cast<int>(tang0));
            }
            encoder.encode(mainIds.geometry, static_cast<int>(tang1));
            if (mesEncoder) {
                mesEncoder->encode(mesIds.geometry, static_cast<int>(tang1));
            }

            unsigned normal = geometryNormalSymbols[i_clusterId][i_splitIndex] - i_minResidualNormal_;
            assert(normal < residualNormalRanges_[i_clusterId]);
            encoder.encode(mainIds.geometryNormal, static_cast<int>(normal));
            if (mesEncoder) {
                mesEncoder->encode(mesIds.geometryNormal, static_cast<int>(normal));
            }

            i_splitIndex++;
        }
    }

    unsigned i_size = static_cast<unsigned>(encoder.finish());
    unsigned i_sizeGeom =
        mesEncoder ? static_cast<unsigned>(mesEncoder->finish()) : 0;

    clusterSizes_[i_clusterId] = i_size;
    if (mesEncoder) {
        i_connectivitySize_ += (i_size - i_sizeGeom) * 8;
        i_geometrySize_ += i_sizeGeom * 8;
    }
}

void PomarColumnarSerializeOperator::initAndEncodeModels(unsigned i_clusterId,
                                                           pmsf::MultiModelRangeEncoder& encoder,
                                                           pmsf::MultiModelRangeEncoder* mesEncoder,
                                                           ModelIds& mainIds,
                                                           ModelIds& mesIds) {
    unsigned i_nbBits;

    i_maxNbSplits_ = maxNbSplits_[i_clusterId];
    mainIds.nbSplit = encoder.addModel({static_cast<int>(i_maxNbSplits_ + 1), 10, 1 << 9});

    i_nbBits = i_clusterId > 0 ? i_nbBitsNbSplits_ : 3;
    assert(i_maxNbSplits_ < (unsigned)(1 << i_nbBits));
    encoder.encodeRawSymbol(static_cast<int>(i_maxNbSplits_), static_cast<int>(i_nbBits));

    i_maxNbSkipsHehSymbol_ = 0;
    i_maxVertexSplits_ = 0;
    i_minResidual_ = 0;
    i_minResidualNormal_ = 0;

    if (i_maxNbSplits_ > 0) {
        i_maxNbSkipsHehSymbol_ = maxNbSkips_[i_clusterId];
        mainIds.nbSkips = encoder.addModel(
            {static_cast<int>(i_maxNbSkipsHehSymbol_ + 1), 10, 1 << 9});

        i_nbBits = i_clusterId > 0 ? i_nbBitsNbSkips_ : 4;
        assert(i_maxNbSkipsHehSymbol_ < (unsigned)(1 << i_nbBits));
        encoder.encodeRawSymbol(static_cast<int>(i_maxNbSkipsHehSymbol_),
                                static_cast<int>(i_nbBits));

        i_maxVertexSplits_ = maxVertexSplits_[i_clusterId];
        mainIds.vertexSplit = encoder.addModel(
            {static_cast<int>(i_maxVertexSplits_ + 1), 10, 1 << 9});

        i_nbBits = i_clusterId > 0 ? i_nbBitsVertexSplits_ : 5;
        assert(i_maxVertexSplits_ < (unsigned)(1 << i_nbBits));
        encoder.encodeRawSymbol(static_cast<int>(i_maxVertexSplits_),
                                static_cast<int>(i_nbBits));

        i_minResidual_ = minResiduals_[i_clusterId];
        mainIds.geometry = encoder.addModel(
            {static_cast<int>(residualRanges_[i_clusterId]), 18, 1 << 17});

        if (i_clusterId > 0) {
            assert(i_minResidual_ - i_minMinResidual_ < (1 << i_nbBitsMinResidual_));
            encoder.encodeRawSymbol(i_minResidual_ - i_minMinResidual_,
                                    static_cast<int>(i_nbBitsMinResidual_));
            if (mesEncoder) {
                mesEncoder->encodeRawSymbol(
                    i_minResidual_ - i_minMinResidual_,
                    static_cast<int>(i_nbBitsMinResidual_));
            }
            assert(residualRanges_[i_clusterId] < (unsigned)(1 << i_nbBitsResidualRange_));
            encoder.encodeRawSymbol(static_cast<int>(residualRanges_[i_clusterId]),
                                    static_cast<int>(i_nbBitsResidualRange_));
            if (mesEncoder) {
                mesEncoder->encodeRawSymbol(
                    static_cast<int>(residualRanges_[i_clusterId]),
                    static_cast<int>(i_nbBitsResidualRange_));
            }
        } else {
            encoder.encodeRawShort(static_cast<uint16_t>(i_minResidual_));
            if (mesEncoder) {
                mesEncoder->encodeRawShort(
                    static_cast<uint16_t>(i_minResidual_));
            }
            encoder.encodeRawShort(static_cast<uint16_t>(residualRanges_[i_clusterId]));
            if (mesEncoder) {
                mesEncoder->encodeRawShort(
                    static_cast<uint16_t>(residualRanges_[i_clusterId]));
            }
        }

        i_minResidualNormal_ = minResidualNormals_[i_clusterId];
        mainIds.geometryNormal = encoder.addModel(
            {static_cast<int>(residualNormalRanges_[i_clusterId]), 18, 1 << 17});

        if (i_clusterId > 0) {
            assert(i_minResidualNormal_ - i_minMinResidualNormal_ < (1 << i_nbBitsMinResidualNormal_));
            encoder.encodeRawSymbol(i_minResidualNormal_ - i_minMinResidualNormal_,
                                    static_cast<int>(i_nbBitsMinResidualNormal_));
            if (mesEncoder) {
                mesEncoder->encodeRawSymbol(
                    i_minResidualNormal_ - i_minMinResidualNormal_,
                    static_cast<int>(i_nbBitsMinResidualNormal_));
            }
            assert(residualNormalRanges_[i_clusterId] < (unsigned)(1 << i_nbBitsResidualNormalRange_));
            encoder.encodeRawSymbol(static_cast<int>(residualNormalRanges_[i_clusterId]),
                                    static_cast<int>(i_nbBitsResidualNormalRange_));
            if (mesEncoder) {
                mesEncoder->encodeRawSymbol(
                    static_cast<int>(residualNormalRanges_[i_clusterId]),
                    static_cast<int>(i_nbBitsResidualNormalRange_));
            }
        } else {
            encoder.encodeRawShort(static_cast<uint16_t>(i_minResidualNormal_));
            if (mesEncoder) {
                mesEncoder->encodeRawShort(
                    static_cast<uint16_t>(i_minResidualNormal_));
            }
            encoder.encodeRawShort(static_cast<uint16_t>(residualNormalRanges_[i_clusterId]));
            if (mesEncoder) {
                mesEncoder->encodeRawShort(
                    static_cast<uint16_t>(residualNormalRanges_[i_clusterId]));
            }
        }

        if (mesEncoder) {
            mesIds.nbSplit = mesEncoder->addModel(
                {static_cast<int>(i_maxNbSplits_ + 1), 10, 1 << 9});
            mesIds.nbSkips = mesEncoder->addModel(
                {static_cast<int>(i_maxNbSkipsHehSymbol_ + 1), 10, 1 << 9});
            mesIds.vertexSplit = mesEncoder->addModel(
                {static_cast<int>(i_maxVertexSplits_ + 1), 10, 1 << 9});
            mesIds.geometry = mesEncoder->addModel(
                {static_cast<int>(residualRanges_[i_clusterId]), 18, 1 << 17});
            mesIds.geometryNormal = mesEncoder->addModel(
                {static_cast<int>(residualNormalRanges_[i_clusterId]), 18, 1 << 17});
        }
    }
}

void PomarColumnarSerializeOperator::determineCoderValues(const std::vector<std::deque<unsigned>>& nbSplitSymbols,
                                                    const std::vector<std::deque<unsigned>>& nbSkipsHehSymbols,
                                                    const std::vector<std::deque<unsigned>>& vertexSplitSymbols,
                                                    const std::vector<std::deque<int>>& geometrySymbols,
                                                    const std::vector<std::deque<int>>& geometryNormalSymbols) {
    maxNbSplits_.resize(i_nbClusters_);
    maxNbSkips_.resize(i_nbClusters_);
    maxVertexSplits_.resize(i_nbClusters_);
    minResiduals_.resize(i_nbClusters_);
    residualRanges_.resize(i_nbClusters_);
    minResidualNormals_.resize(i_nbClusters_);
    residualNormalRanges_.resize(i_nbClusters_);

    for (unsigned i_clusterId = 0; i_clusterId < i_nbClusters_; ++i_clusterId) {
        if (nbSplitSymbols[i_clusterId].empty()) {
            maxNbSplits_[i_clusterId] = 0;
            continue;
        }
        maxNbSplits_[i_clusterId] = *std::max_element(nbSplitSymbols[i_clusterId].begin(), nbSplitSymbols[i_clusterId].end());
        if (maxNbSplits_[i_clusterId] > 0) {
            maxNbSkips_[i_clusterId] = *std::max_element(nbSkipsHehSymbols[i_clusterId].begin(), nbSkipsHehSymbols[i_clusterId].end());
            maxVertexSplits_[i_clusterId] = *std::max_element(vertexSplitSymbols[i_clusterId].begin(), vertexSplitSymbols[i_clusterId].end());

            minResiduals_[i_clusterId] = *std::min_element(geometrySymbols[i_clusterId].begin(), geometrySymbols[i_clusterId].end());
            i_maxResidual_ = *std::max_element(geometrySymbols[i_clusterId].begin(), geometrySymbols[i_clusterId].end());
            residualRanges_[i_clusterId] = i_maxResidual_ - minResiduals_[i_clusterId] + 1;

            minResidualNormals_[i_clusterId] = *std::min_element(geometryNormalSymbols[i_clusterId].begin(), geometryNormalSymbols[i_clusterId].end());
            i_maxResidualNormal_ = *std::max_element(geometryNormalSymbols[i_clusterId].begin(), geometryNormalSymbols[i_clusterId].end());
            residualNormalRanges_[i_clusterId] = i_maxResidualNormal_ - minResidualNormals_[i_clusterId] + 1;

            // Defensive (2026-05-01): mirror of PomarSerializeOperator
            // sanitize — catches any INT_MIN / overflow residuals that
            // slipped past the Frenet clamp in PomarColumnarSymbolCollectOperator.
            if (residualRanges_[i_clusterId] >= (1u << 30)) {
                std::cerr << "[POMARv2] cluster " << i_clusterId
                          << " geom range overflow " << residualRanges_[i_clusterId]
                          << " — clamping to 1" << std::endl;
                minResiduals_[i_clusterId] = 0;
                i_maxResidual_ = 0;
                residualRanges_[i_clusterId] = 1;
            }
            if (residualNormalRanges_[i_clusterId] >= (1u << 30)) {
                std::cerr << "[POMARv2] cluster " << i_clusterId
                          << " normal range overflow " << residualNormalRanges_[i_clusterId]
                          << " — clamping to 1" << std::endl;
                minResidualNormals_[i_clusterId] = 0;
                i_maxResidualNormal_ = 0;
                residualNormalRanges_[i_clusterId] = 1;
            }
        }
    }

    if (i_nbClusters_ > 1) {
        i_nbBitsNbSplits_ = (unsigned)std::ceil(std::log((double)(*std::max_element(maxNbSplits_.begin() + 1, maxNbSplits_.end()) + 1)) / std::log(2.0));
        i_nbBitsNbSkips_ = (unsigned)std::ceil(std::log((double)(*std::max_element(maxNbSkips_.begin() + 1, maxNbSkips_.end()) + 1)) / std::log(2.0));
        i_nbBitsVertexSplits_ =
            (unsigned)std::ceil(std::log((double)(*std::max_element(maxVertexSplits_.begin() + 1, maxVertexSplits_.end()) + 1)) / std::log(2.0));

        i_minMinResidual_ = *std::min_element(minResiduals_.begin() + 1, minResiduals_.end());
        i_nbBitsMinResidual_ =
            (unsigned)std::ceil(std::log((double)(*std::max_element(minResiduals_.begin() + 1, minResiduals_.end()) - i_minMinResidual_ + 1)) / std::log(2.0));
        i_nbBitsResidualRange_ =
            (unsigned)std::ceil(std::log((double)(*std::max_element(residualRanges_.begin() + 1, residualRanges_.end()) + 1)) / std::log(2.0));

        i_minMinResidualNormal_ = *std::min_element(minResidualNormals_.begin() + 1, minResidualNormals_.end());
        i_nbBitsMinResidualNormal_ =
            (unsigned)std::ceil(std::log((double)(*std::max_element(minResidualNormals_.begin() + 1, minResidualNormals_.end()) - i_minMinResidualNormal_ + 1)) /
                                std::log(2.0));
        i_nbBitsResidualNormalRange_ = (unsigned)std::ceil(
            std::log((double)(*std::max_element(residualNormalRanges_.begin() + 1, residualNormalRanges_.end()) + 1)) / std::log(2.0));
    }
}
