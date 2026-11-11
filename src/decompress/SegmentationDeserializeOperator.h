/*
 * @Author: koi
 * @Date: 2025-09-02 10:39:10
 * @Description: 分区解压缩操作符：支持按分区独立解压到任意目标轮次
 */
#ifndef SEGMENTATION_DESERIALIZE_OPERATOR_H
#define SEGMENTATION_DESERIALIZE_OPERATOR_H

#include "DeserializeOperator.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "decompress/SymbolReadOperator.h"
#include <cstddef>
#include <memory>
#include <vector>

class SegmentationDeserializeOperator : public DeserializeOperator {
  public:
    SegmentationDeserializeOperator(std::shared_ptr<SymbolReadOperator> symbolReadOperator,
                                    std::shared_ptr<ReconstructOperator> reconstructOperator)
        : DeserializeOperator(symbolReadOperator, reconstructOperator) {}

    ~SegmentationDeserializeOperator() = default;

    std::shared_ptr<MCGAL::Mesh> deserializeBaseMesh() override;

    std::shared_ptr<MCGAL::Mesh> exportMesh() override;

    /// @brief 将所有分区同步推进一轮（向后兼容的行为）
    void deserializeOneRound() override;

    /// @brief 仅对指定分区推进一轮（新增）
    /// @param groupId 目标分区编号 [0, groupNumber)
    /// @return 是否成功推进（若已到末尾返回 false）
    bool deserializeOneRoundForGroup(int groupId);

    /// @brief 将指定分区解压到第 targetRound 轮（新增）
    /// @param groupId 目标分区编号 [0, groupNumber)
    /// @param targetRound 目标累计轮次（1 表示解压 1 轮；取值范围 [0, roundNumber]）
    ///                    若 targetRound 小于当前已解压轮次则为 no-op（不支持回退）
    void decompressGroupToRound(int groupId, int targetRound);

    /// @brief 将所有分区解压到同一目标轮次
    /// @param targetRound 同上（0 表示仅基网；roundNumber 表示全量解压）
    void decompressAllToRound(int targetRound);

    int getGroupNumber() const { return groupNumber_; }
    int getRoundNumber() const { return roundNumber_; }
    int getCurrentRound(int groupId) const {
        if (groupId < 0 || groupId >= (int)groupCurrentRound_.size()) return 0;
        return groupCurrentRound_[groupId];
    }

    std::vector<MCGAL::Halfedge*> seeds;

  protected:
    // 新布局：分区起点 + 每分区每轮累计偏移（相对分区起点）
    int groupNumber_ = 0;
    int roundNumber_ = 0;
    size_t payloadStart_ = 0;                          // 数据 payload 的起始字节（meta 之后）
    std::vector<size_t> groupStart_;                   // 每个分区数据段的起点（绝对 offset）
    std::vector<std::vector<size_t>> groupRoundEnd_;   // groupRoundEnd_[g][r] = 分区 g 第 r+1 轮解压结束时的绝对 offset
    std::vector<int> groupCurrentRound_;               // 每个分区已解压轮次（[0, R]）
};

#endif
