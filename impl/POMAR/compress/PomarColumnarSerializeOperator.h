/*
 * @Author: pomar-refactor
 * @Description: Pomar 存储模块重构版 —— 序列化操作符
 *
 * 本类严格遵循框架 SerializeOperator 抽象：
 *   - 使用基类 buffer_ / dataOffset_ 作为统一的输出缓冲区；
 *   - 通过 serializeHeader / serializeBaseMeshWithSeed / serialize 三段式完成；
 *   - serialize() 阶段读取 PomarColumnarSymbolCollectOperator 中的符号队列，做
 *     range-coder 熵编码，然后将比特流拼接进 buffer_，最后按框架惯例
 *     "先写 int 长度，再写 buffer_ 内容"到文件。
 *
 * 与旧版 PomarSerializeOperator 的主要差异：
 *   1) 不再自行管理 p_data / p_dataOffset，统一接入框架 buffer_；
 *   2) header / baseMesh 的写入拆分到独立虚函数中；
 *   3) 符号收集与熵编码彻底解耦；
 *   4) 文件布局带长度前缀，与 BasicSerializeOperator 保持一致，方便
 *      统一 DecompressOperator 的 init(path) 直接读取。
 */

#pragma once

#include "rangecoder/rangecod.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "compress/SerializeOperator.h"

#include <cstddef>
#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace pmsf {
class MultiModelRangeEncoder;
}

class PomarColumnarSymbolCollectOperator;

class PomarColumnarSerializeOperator : public SerializeOperator {
  public:
    PomarColumnarSerializeOperator(std::shared_ptr<SymbolCollectOperator> collector,
                             bool enablePrediction = false,
                             bool enableQuantization = false,
                             bool enableCompress = false)
        : SerializeOperator(collector, enablePrediction, enableQuantization, enableCompress) {}

    // skipFileWrite_ 路径下父类 serialize() 不调用 freeAuxBuffers()（wrapper 需要
    // 继续读 buffer_）；析构时兜底释放 p_dataMes_ / p_dataClusters_，避免重复测量
    // 累积泄漏。std::free(nullptr) / delete nullptr 均为安全空操作。
    ~PomarColumnarSerializeOperator() override { freeAuxBuffers(); }

    // ---- SerializeOperator 标准接口 ----
    void init() override;
    void serializeInt(int val) override;
    void serializeBaseMesh(std::shared_ptr<MCGAL::Mesh> mesh) override;
    void serializeBaseMeshWithSeed(std::shared_ptr<MCGAL::Mesh> mesh, std::vector<MCGAL::Halfedge*> seed) override;
    void serialize(std::string path) override;

    void setRandomAccess(bool b) {
        b_randomAccess_ = b;
    }

    // ============ column-native Parquet V2 支持 ============
    // 供 POMARColumnarSerializeOperator 在 serialize() 结束后读取底层 buffer
    // 的段边界（base mesh / cluster 0 / cluster sizes meta / clusters rest），
    // 以便按关系列拆分写入独立的 parquet 文件。不改变原 serialize(string) 的
    // 行为与字节布局。
    const char* rawBuffer()           const { return buffer_; }
    std::size_t rawBufferSize()       const { return dataOffset_; }
    std::size_t rawBaseMeshEnd()      const { return baseMeshEndOffset_; }
    std::size_t rawCluster0End()      const { return cluster0EndOffset_; }
    std::size_t rawClusterMetaEnd()   const { return clusterMetaEndOffset_; }
    const std::deque<unsigned>& rawClusterSizes() const { return clusterSizes_; }
    unsigned rawNbClusters()          const { return i_nbClusters_; }
    unsigned rawNbGlobalLOD()         const { return i_nbGlobalLOD_; }
    unsigned rawNbClusteredLOD()      const { return i_nbClusteredLOD_; }
    unsigned rawOriginalVertices()    const { return originalVertices_; }
    unsigned rawOriginalFaces()       const { return originalFaces_; }

    // 允许 wrapper 跳过写文件，直接取 buffer_ 内容。
    void setSkipFileWrite(bool v) { skipFileWrite_ = v; }
    bool skipFileWrite() const    { return skipFileWrite_; }

  private:
    using PomarCollector = PomarColumnarSymbolCollectOperator;

    /// 访问基础收集器（dynamic_cast 保护）
    std::shared_ptr<PomarCollector> pomarCollector() const;

    /// 统计各 cluster 的 min/max/range 以及公共 bit width
    void determineCoderValues(const std::vector<std::deque<unsigned>>& nbSplitSymbols,
                              const std::vector<std::deque<unsigned>>& nbSkipsHehSymbols,
                              const std::vector<std::deque<unsigned>>& vertexSplitSymbols,
                              const std::vector<std::deque<int>>& geometrySymbols,
                              const std::vector<std::deque<int>>& geometryNormalSymbols);

    /// 为第 i_clusterId 个 cluster 进行熵编码
    void compressCluster(unsigned i_clusterId,
                         const std::vector<std::deque<unsigned>>& nbSplitSymbols,
                         const std::vector<std::deque<unsigned>>& nbSkipsHehSymbols,
                         const std::vector<std::deque<unsigned>>& vertexSplitSymbols,
                         const std::vector<std::deque<int>>& geometrySymbols,
                         const std::vector<std::deque<int>>& geometryNormalSymbols);

    /// 写 cluster 元信息（sizes、bit widths、min residual 等）到主缓冲
    void writeClusterSizes();

    struct ModelIds {
        int nbSplit = -1;
        int nbSkips = -1;
        int vertexSplit = -1;
        int geometry = -1;
        int geometryNormal = -1;
    };
    void initAndEncodeModels(unsigned i_clusterId,
                             pmsf::MultiModelRangeEncoder& encoder,
                             pmsf::MultiModelRangeEncoder* mesEncoder,
                             ModelIds& mainIds,
                             ModelIds& mesIds);

    /// 以 buffer_ 为后备缓冲的一组 range-coder 操作辅助
    void initRangeCoders();
    void freeAuxBuffers();

  private:
    // ---- cluster 侧写入缓冲（第二阶段） ----
    char* p_dataClusters_ = nullptr;
    size_t* p_dataOffsetClusters_ = nullptr;

    // ---- range coder 状态 ----
    rangecoder rangeCoder_{};
    rangecoder rangeCoderMes_{};

    // 用于 Mes (geometry only) 的辅助缓冲
    char* p_dataMes_ = nullptr;
    size_t* p_dataOffsetMes_ = nullptr;

    // ---- cluster 层级信息 ----
    bool b_randomAccess_ = true;
    unsigned i_nbClusters_ = 0;
    unsigned i_nbGlobalLOD_ = 0;
    unsigned i_nbClusteredLOD_ = 0;
    unsigned i_nbVerticesBaseMesh_ = 0;
    unsigned originalVertices_ = 0;
    unsigned originalFaces_ = 0;
    std::deque<unsigned> clusterSizes_;

    // ---- 熵编码统计量 ----
    std::vector<unsigned> maxNbSplits_;
    std::vector<unsigned> maxNbSkips_;
    std::vector<unsigned> maxVertexSplits_;
    std::vector<int> minResiduals_;
    std::vector<unsigned> residualRanges_;
    std::vector<int> minResidualNormals_;
    std::vector<unsigned> residualNormalRanges_;

    unsigned i_nbBitsNbSplits_ = 0;
    unsigned i_nbBitsNbSkips_ = 0;
    unsigned i_nbBitsVertexSplits_ = 0;
    unsigned i_nbBitsMinResidual_ = 0;
    unsigned i_nbBitsResidualRange_ = 0;
    unsigned i_nbBitsMinResidualNormal_ = 0;
    unsigned i_nbBitsResidualNormalRange_ = 0;

    int i_minMinResidual_ = 0;
    int i_minMinResidualNormal_ = 0;

    // 当前 cluster 快照
    unsigned i_maxNbSplits_ = 0;
    unsigned i_maxNbSkipsHehSymbol_ = 0;
    unsigned i_maxVertexSplits_ = 0;
    int i_minResidual_ = 0;
    int i_maxResidual_ = 0;
    int i_minResidualNormal_ = 0;
    int i_maxResidualNormal_ = 0;

    // 统计输出（压缩率）
    size_t i_connectivitySize_ = 0;
    size_t i_geometrySize_ = 0;

    // 段边界记录（column-native wrapper 用）
    std::size_t baseMeshEndOffset_    = 0;
    std::size_t cluster0EndOffset_    = 0;
    std::size_t clusterMetaEndOffset_ = 0;
    bool skipFileWrite_ = false;
};
