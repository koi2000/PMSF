/*
 * @Author: pomar-refactor
 * @Description: Pomar 存储模块重构版 —— 反序列化操作符
 *
 * 完全遵循 DeserializeOperator 框架流程：
 *   1. init(path)                 —— 由基类负责读取文件到 buffer_；
 *   2. deserializeHeader(header) —— 读取 PMSFHeader；
 *   3. deserializeBaseMesh()     —— 读取 base mesh、cluster meta、符号流；
 *   4. deserializeOneRound()     —— 每调用一次推进一个 LOD。
 *
 * 关键职责拆分：
 *   - 符号解码由 PomarColumnarSymbolReadOperator 承担；
 *   - vertex split 的实际 mesh 操作由 PomarReconstructOperatorV2 承担。
 */
#pragma once

#include "Halfedge.h"
#include "Mesh.h"
#include "Vertex.h"
#include "decompress/DeserializeOperator.h"

#include <cstddef>
#include <deque>
#include <memory>
#include <queue>
#include <set>
#include <vector>

namespace MCGAL {
struct PerformedOperation;
}

class PomarColumnarSymbolReadOperator;
class PomarReconstructOperatorV2;

class PomarColumnarDeserializeOperator : public DeserializeOperator {
  public:
    PomarColumnarDeserializeOperator(std::shared_ptr<SymbolReadOperator> symbolReadOperator,
                               std::shared_ptr<ReconstructOperator> reconstructOperator)
        : DeserializeOperator(symbolReadOperator, reconstructOperator) {}

    ~PomarColumnarDeserializeOperator() = default;

    std::shared_ptr<MCGAL::Mesh> deserializeBaseMesh() override;
    void deserializeOneRound() override;

    void setSeeds(std::vector<MCGAL::Halfedge*> /*seeds*/) override {}

  private:
    // ---- 内部流程 ----
    void readHeaderAndBaseMesh();
    void readClusterSizes();
    void numberClusters();

    /// 为每个 cluster 计算需要解码到第几层 LOD
    void setRequiredNbLOD(MCGAL::Vertex* vhInit, unsigned* p_requiredNbLOD);

    /// 从符号队列按 BFS 顺序消费符号，产生一轮的 PerformedOperation 队列
    void refineCluster(unsigned i_clusterId, std::queue<MCGAL::PerformedOperation>& opToPerform);

    /// 解码一次 vertexSplit 符号，生成对应 PerformedOperation
    MCGAL::PerformedOperation decodeVertexSplit(MCGAL::Halfedge* heh, unsigned i_clusterId, unsigned i_splitId);
    bool isDecodeCandidateValid(MCGAL::Halfedge* heh, unsigned i_clusterId, unsigned i_splitId);

    /// 标记"该 cluster 内不需要再计数"的半边
    void markEdgeHalfedgesToNotCount(unsigned i_clusterId, MCGAL::Halfedge* heh);
    void markEdgesToNotCount(unsigned i_clusterId, const MCGAL::PerformedOperation& op);

    std::shared_ptr<PomarColumnarSymbolReadOperator> pomarReader() const;
    std::shared_ptr<PomarReconstructOperatorV2> pomarReconstructor() const;

  private:
    // ---- cluster 分组信息 ----
    unsigned i_nbClusters_ = 1;
    unsigned i_nbGlobalLOD_ = 0;
    unsigned i_nbClusteredLOD_ = 0;
    std::deque<unsigned> clusterSizes_;
    std::vector<MCGAL::Halfedge*> clusterSeedHeh_;
    std::vector<unsigned> clusterVertexCounts_;
    size_t dataOffsetClusters_ = 0;

    // 每 cluster 的游标（消费符号用）
    std::deque<unsigned> symbolIds_;
    std::deque<unsigned> splitIds_;

    // 当前推进到的 LOD 层
    unsigned curGlobalLOD_ = 0;
    unsigned curClusteredLOD_ = 0;
    bool globalLODConsumed_ = false;  // 全局 LOD 是否已全部处理完

    // 每 cluster 需要解到的 LOD 层数（仅 clustered 阶段使用）
    std::vector<unsigned> p_requiredNbLOD_;
};
