/*
 * @Author: pomar-refactor
 * @Description: Pomar 存储模块重构版 —— 符号读取器
 *
 * 对应 SymbolReadOperator 在框架中的角色：
 *   - 通过 buffer_/dataOffset_ 消费 PomarColumnarSerializeOperator 写入的比特流；
 *   - 提供按 cluster 维度的熵解码（bundle range-coder 状态），并把结果存进
 *     内部的五类符号队列，供 PomarColumnarDeserializeOperator 在每一轮 refine 时按
 *     BFS 顺序消费。
 *
 * 与框架 SymbolReadOperator 的 collect/collectFromDecoded 接口保持一致：
 *   - collect(seed, buffer, dataOffset, ...) 用于一次性的 cluster 解码；
 *   - collectFromDecoded(seed, ...) 在实际逐轮重建时调用（Pomar 并非
 *     对象语义的 add-vertex 流式还原，因此这里保留默认空实现，实际
 *     重建由 DeserializeOperator 侧完成）。
 */
#pragma once

#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Point.h"
#include "decompress/SymbolReadOperator.h"

#include <cstddef>
#include <deque>
#include <memory>
#include <vector>

class PomarColumnarSymbolReadOperator : public SymbolReadOperator {
  public:
    PomarColumnarSymbolReadOperator() = default;
    ~PomarColumnarSymbolReadOperator() = default;

    // 框架抽象接口：Pomar 的符号解码由 deserializer 按 cluster 触发，
    // 因此下面两个原生 collect 接口保持空实现即可。
    void collect(std::vector<MCGAL::Halfedge*> /*seeds*/,
                 char* /*buffer*/,
                 size_t& /*dataOffset*/,
                 std::vector<MCGAL::Vertex*>& /*vertices*/,
                 std::vector<MCGAL::Halfedge*>& /*halfedge*/,
                 std::vector<MCGAL::Facet*>& /*facets*/) override {}

    void collect(MCGAL::Halfedge* /*seed*/,
                 char* /*buffer*/,
                 size_t& /*dataOffset*/,
                 std::vector<MCGAL::Vertex*>& /*vertices*/,
                 std::vector<MCGAL::Halfedge*>& /*halfedge*/,
                 std::vector<MCGAL::Facet*>& /*facets*/) override {}

    // ---- Pomar 专用接口：按 cluster 解码符号 ----

    /// 绑定主缓冲（由 DeserializeOperator::init(path) 读入）
    void bindBuffer(char* buffer, size_t* dataOffset) {
        buffer_ = buffer;
        dataOffsetPtr_ = dataOffset;
        rangeCoder_.p_data = buffer;
        rangeCoder_.p_dataOffset = dataOffset;
    }

    /// 初始化内部队列容量（根据 cluster 总数）
    void resizeClusters(unsigned nbClusters);

    /// 从比特流解码一个 cluster 的符号到内部队列
    /// @param i_clusterId cluster id
    /// @param i_maxLOD    该 cluster 要解码的 LOD 层数
    void decodeCluster(unsigned i_clusterId, unsigned i_maxLOD, unsigned i_nbBaseVertices);

    // ---- cluster meta 元信息（从主 buffer 读取后由 deserializer 注入） ----
    void setClusterMeta(unsigned nbBitsNbSplits,
                        unsigned nbBitsNbSkips,
                        unsigned nbBitsVertexSplits,
                        unsigned nbBitsMinResidual,
                        unsigned nbBitsResidualRange,
                        unsigned nbBitsMinResidualNormal,
                        unsigned nbBitsResidualNormalRange,
                        int minMinResidual,
                        int minMinResidualNormal) {
        i_nbBitsNbSplits_ = nbBitsNbSplits;
        i_nbBitsNbSkips_ = nbBitsNbSkips;
        i_nbBitsVertexSplits_ = nbBitsVertexSplits;
        i_nbBitsMinResidual_ = nbBitsMinResidual;
        i_nbBitsResidualRange_ = nbBitsResidualRange;
        i_nbBitsMinResidualNormal_ = nbBitsMinResidualNormal;
        i_nbBitsResidualNormalRange_ = nbBitsResidualNormalRange;
        i_minMinResidual_ = minMinResidual;
        i_minMinResidualNormal_ = minMinResidualNormal;
    }

    // ---- 对 Deserializer 开放的只读访问 ----
    std::vector<std::deque<unsigned>>& getNbSplitSymbols() {
        return nbSplitSymbols_;
    }
    std::vector<std::deque<unsigned>>& getNbSkipsHehSymbols() {
        return nbSkipsHehSymbols_;
    }
    std::vector<std::deque<unsigned>>& getVertexSplitSymbols() {
        return vertexSplitSymbols_;
    }
    std::vector<std::deque<int>>& getGeometrySymbols() {
        return geometrySymbols_;
    }
    std::vector<std::deque<int>>& getGeometryNormalSymbols() {
        return geometryNormalSymbols_;
    }

  private:
    /// 解码/初始化 range-coder 模型（对应压缩侧 initAndEncodeModels）
    void decodeAndInitModels(unsigned i_clusterId);
    /// 销毁 range-coder 模型
    void deleteModels();

    unsigned decodeNbBits(rangecoder* rc, unsigned i_nbBits);

  private:
    char* buffer_ = nullptr;
    size_t* dataOffsetPtr_ = nullptr;

    rangecoder rangeCoder_{};
    qsmodel nbSkipsHehSymbolsModel_{};
    qsmodel vertexSplitSymbolsModel_{};
    qsmodel nbSplitSymbolsModel_{};
    qsmodel geometrySymbolsModel_{};
    qsmodel geometryNormalSymbolsModel_{};

    std::vector<std::deque<unsigned>> nbSplitSymbols_;
    std::vector<std::deque<unsigned>> nbSkipsHehSymbols_;
    std::vector<std::deque<unsigned>> vertexSplitSymbols_;
    std::vector<std::deque<int>> geometrySymbols_;
    std::vector<std::deque<int>> geometryNormalSymbols_;

    unsigned i_maxNbSplits_ = 0;
    unsigned i_maxNbSkipsHehSymbol_ = 0;
    unsigned i_maxVertexSplits_ = 0;
    int i_minResidual_ = 0;
    int i_maxResidual_ = 0;
    int i_minResidualNormal_ = 0;
    int i_maxResidualNormal_ = 0;

    unsigned i_nbBitsNbSplits_ = 0;
    unsigned i_nbBitsNbSkips_ = 0;
    unsigned i_nbBitsVertexSplits_ = 0;
    unsigned i_nbBitsMinResidual_ = 0;
    unsigned i_nbBitsResidualRange_ = 0;
    unsigned i_nbBitsMinResidualNormal_ = 0;
    unsigned i_nbBitsResidualNormalRange_ = 0;

    int i_minMinResidual_ = 0;
    int i_minMinResidualNormal_ = 0;
};
