#pragma once

#include "OptimizedSymbolCollectOperator.h"
#include "SymbolCollectOperator.h"

#include <cstdint>
#include <memory>
#include <vector>

class SegmentedOptimizedSymbolCollectOperator : public SymbolCollectOperator {
  public:
    SegmentedOptimizedSymbolCollectOperator() = default;

    void init(std::shared_ptr<MCGAL::Mesh> mesh,
              std::vector<MCGAL::Halfedge*> seeds,
              PMSFHeader header) override;

    void collect(MCGAL::Halfedge* seed) override;
    void collect(std::vector<MCGAL::Halfedge*> seeds) override;

    /// P18.2 阶段 B：分区推进（内层 group 循环）。
    /// 基类默认退回 `collect(seed)`；本类覆写为按 `activeMask` 推进各分区。
    void collectRound(MCGAL::Halfedge* seed,
                      const std::vector<MCGAL::Halfedge*>& seeds,
                      const std::vector<std::uint8_t>& activeMask,
                      int round) override;

    void collectActive(
        const std::vector<MCGAL::Halfedge*>& seeds,
        const std::vector<std::uint8_t>& activeMask,
        const std::vector<std::vector<MCGAL::Facet*>>* liveFacesByGroup = nullptr);

    /// P18.2 PHASE59 物品 1/3：把预测开关**转发给内层 collector**。
    ///
    /// 不覆写时，序列化器在 `exportToBuffer*` 之前那句
    /// `setEnablePrediction(true)`（`BasicSerializeOperator.cpp:259`）只落在
    /// **本包装**上；内层实例早在 `init()` 的 `ensureCollectors()` 里按基类
    /// 默认 `false` 构造好了 ⇒ **分段路径与不分段路径的坐标编码器不是同一个**。
    /// 实测（附 17）：N=1 → `useResidualBitPlane=1`；N=2 的内层 →
    /// `enablePrediction=0, useAdaptiveModel=1`。
    void setEnablePrediction(bool v) override;

    int exportToBuffer(char* buffer,
                       bool enableQuantization = false,
                       bool enableCompress = false) override;
    int exportToBufferWithRangeCoder(char* buffer,
                                     bool enableQuantization = false) override;
    bool supportsRangeCoderExport() const override { return true; }

    const pmsf::PMSFSymbolTable* pmsfSymbolTable() const override;

  private:
    void ensureCollectors(std::size_t count);

    std::vector<std::unique_ptr<OptimizedSymbolCollectOperator>> collectors_;

  public:
    /// Debug: per-group facet symbol count collected for one round (0 if absent).
    std::size_t groupRoundFacetSymbols(std::size_t group, int round) const;

  private:
    mutable pmsf::PMSFSymbolTable paperTableCache_{};
    int roundsCollected_ = 0;
};
