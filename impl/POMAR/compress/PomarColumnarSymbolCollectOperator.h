/*
 * @Author: pomar-refactor
 * @Description: Pomar 存储模块重构版 —— 符号收集器
 *
 * 职责（符合框架 SymbolCollectOperator 规范）：
 *   1. 基于 BFS 对 base mesh 所有顶点进行 cluster 编号，记录每个 cluster 的种子半边；
 *   2. 按 LOD 从粗到细遍历 mesh，从 MCGAL::contextPool.operations 中读取每一轮
 *      执行过的 vertexSplit 操作，并把它们编码为五类符号队列（按 cluster 分桶存储）；
 *   3. 在 collect() 返回后，由 SerializeOperator 通过 getter 读取这些队列，
 *      继续进行 rangecoder 熵编码与文件写入。
 *
 * 注意：该类不直接写文件，也不负责熵编码，只做符号的抽取与临时存储，
 * 这一点与 BasicSymbolCollectOperator 的分工是一致的。
 */
#pragma once

#include "Halfedge.h"
#include "PerformedOperation.h"
#include "compress/SymbolCollectOperator.h"
#include <cstddef>
#include <deque>
#include <memory>
#include <vector>

class PomarColumnarSymbolCollectOperator : public SymbolCollectOperator {
  public:
    PomarColumnarSymbolCollectOperator() = default;
    ~PomarColumnarSymbolCollectOperator() = default;

    /// 框架统一入口：单 seed 情形
    void collect(MCGAL::Halfedge* seed) override;
    /// 框架统一入口：多 seed 情形（Pomar 仅使用第一个）
    void collect(std::vector<MCGAL::Halfedge*> seeds) override;

    /// Pomar 在 serializer 内部完成最终的熵编码/写文件，
    /// 因此此处提供一个空实现以满足抽象约束。
    int exportToBuffer(char* buffer, bool enableQuantization = false, bool enableCompress = false) override;
    const pmsf::PMSFSymbolTable* pmsfSymbolTable() const override { return &paperTable_; }

    // ---- 对 Serializer 暴露的只读访问接口 ----

    const std::vector<std::deque<unsigned>>& getNbSplitSymbols() const {
        return nbSplitSymbols_;
    }
    const std::vector<std::deque<unsigned>>& getNbSkipsHehSymbols() const {
        return nbSkipsHehSymbols_;
    }
    const std::vector<std::deque<unsigned>>& getVertexSplitSymbols() const {
        return vertexSplitSymbols_;
    }
    const std::vector<std::deque<int>>& getGeometrySymbols() const {
        return geometrySymbols_;
    }
    const std::vector<std::deque<int>>& getGeometryNormalSymbols() const {
        return geometryNormalSymbols_;
    }

    const std::vector<MCGAL::Halfedge*>& getClusterSeedHeh() const {
        return clusterSeedHeh_;
    }

    unsigned getNbClusters() const {
        return i_nbClusters_;
    }
    unsigned getNbGlobalLOD() const {
        return i_nbGlobalLOD_;
    }
    unsigned getNbClusteredLOD() const {
        return i_nbClusteredLOD_;
    }
    unsigned getNbLODs() const {
        return i_nbLODs_;
    }

    /// Serializer 在开始前可以通过该接口调整 cluster 划分比例
    void setRandomAccessRatio(float p) {
        random_access_ratio_ = p;
        target_cluster_count_ = 0;
    }

    /// 可视化/实验分支使用：按目标 cluster 数选择 global/clustered LOD 分界。
    /// 不设置时保持 POMAR 原始 random_access_ratio_ 行为。
    void setTargetClusterCount(unsigned count) {
        target_cluster_count_ = count;
    }

    /// 仅计算 LOD 划分（不触发符号收集与 mesh 重放）。
    /// 供 Serializer 在 serializeBaseMeshWithSeed 之前查询 i_nbGlobalLOD/i_nbClusteredLOD。
    void computeLODPartition();

  private:
    // -------- 内部阶段 --------

    /// 读取 contextPool.operations 确定 LOD 数量；按比例 p 决定 global/clustered LOD
    void determineNbNonClusteredLOD(float p);
    void determineNbNonClusteredLODByTargetClusters(unsigned targetClusters);

    /// 以 seed 为起点 BFS 给所有顶点分配 groupId，并记录各 cluster 的 seedHeh
    void numberClusters(MCGAL::Halfedge* firstHeh);

    /// 把某条半边所属边上的两个半边打上 b_dontCount 标记（只在该 cluster 生效）
    void markEdgeHalfedgesToNotCount(unsigned i_clusterId, MCGAL::Halfedge* heh);
    void markEdgesToNotCount(unsigned i_clusterId, const MCGAL::PerformedOperation& op);

    /// 编码一次 vertex split，产生 vertexSplit/geometry/geometryNormal 符号
    void codeVertexSplit(unsigned i_clusterId, MCGAL::Halfedge* heh, MCGAL::PerformedOperation& op);

    /// 核心扫描：按 LOD 从粗到细重放 split 操作并产生符号
    void collectAllLODSymbols();

  private:
    // ---- cluster 划分信息 ----
    std::vector<MCGAL::Halfedge*> clusterSeedHeh_;
    unsigned i_nbClusters_ = 1;
    unsigned i_minClusterId_ = 0;

    // ---- LOD 信息 ----
    unsigned i_nbLODs_ = 0;
    unsigned i_nbGlobalLOD_ = 0;
    unsigned i_nbClusteredLOD_ = 0;
    float random_access_ratio_ = 0.03f;
    unsigned target_cluster_count_ = 0;

    // ---- 五类符号队列（按 cluster 分桶） ----
    std::vector<std::deque<unsigned>> nbSplitSymbols_;
    std::vector<std::deque<unsigned>> nbSkipsHehSymbols_;
    std::vector<std::deque<unsigned>> vertexSplitSymbols_;
    std::vector<std::deque<int>> geometrySymbols_;
    std::vector<std::deque<int>> geometryNormalSymbols_;
    pmsf::PMSFSymbolTable paperTable_;
};
