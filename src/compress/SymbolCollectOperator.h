/*
 * @Author: koi
 * @Date: 2025-08-29 20:50:10
 * @Description:
 */
#ifndef SYMBOLCOLLECTOPERATOR_H
#define SYMBOLCOLLECTOPERATOR_H

#include "Halfedge.h"
#include "Mesh.h"
#include "core/Symbol.h"  // Phase A.1: RoundSymbols
#include "header/PMSFHeader.h"
#include <cstdint>
#include <deque>
#include <memory>
#include <vector>

/**
 * @brief 符号收集操作符
 * @details 需要对数据结构和函数进行抽象，
 */
class SymbolCollectOperator {
  public:
    SymbolCollectOperator() = default;
    virtual void init(std::shared_ptr<MCGAL::Mesh> mesh, std::vector<MCGAL::Halfedge*> seeds, PMSFHeader header) {
      mesh_ = mesh;
      seeds_ = seeds;
      header_ = header;
    }
    // 收集一轮的信息
    virtual void collect(MCGAL::Halfedge* h) = 0;
    virtual void collect(std::vector<MCGAL::Halfedge*> hs) {};

    /// @brief B.1 gate-level conquest: 每轮 pipeline 把本轮 eliminate 产
    ///        生的 delta gates（每个 `entry.edge` 经 eliminate 后被 reset
    ///        成 polyFace->proxyHalfedge()）按顺序传进来。gate-level collector
    ///        用它作为本轮遍历的起点集合；BFS-based collector（Optimized /
    ///        Valence）默认忽略此参数，退回 `collect(seed)` 路径。
    ///
    /// @param seed    单 seed（非分区模式）或首个 group seed（分区模式），与
    ///                现有 collect(seed) 语义一致
    /// @param gates   per-round delta halfedge 列表（encoder/decoder 均不
    ///                发到码流，仅作 encoder 本地 hint 使用；decoder 侧由
    ///                同样的 "gate ordering rule" 独立重建，保证同步）
    virtual void collect(MCGAL::Halfedge* seed,
                         const std::vector<MCGAL::Halfedge*>& gates) {
        (void)gates;
        collect(seed);  // 默认忽略 gates，走原 BFS 路径
    }

    /// @brief P18.2 阶段 B：**分区推进**的每轮入口（嵌套 round×group 的外层
    ///        由管线持有，本函数负责**一轮内推进所有仍有前沿的分区**）。
    ///
    /// **默认实现忽略分区参数、退回单 seed 的 `collect(seed, gates)`** ——
    /// 于是非分区 collector（也就是现有全部算法）行为**逐字节不变**，
    /// 这正是物品 2 的硬约束「非分区配置产物逐字节不变」。
    ///
    /// 分段 collector（`SegmentedOptimizedSymbolCollectOperator`）覆写它，
    /// 用 `activeMask` 决定本轮推进哪些分区。`seeds` 是分区种子
    /// （长度 = 分区数，由阶段 A 的 `SegmentOperator::exportSeeds()` 产出）。
    ///
    /// @param seed        主循环的单 seed（回退路径用）
    /// @param seeds       分区种子，长度 = 分区数
    /// @param activeMask  第 i 位表示分区 i 本轮是否仍有前沿
    /// @param round       当前轮号（0-based），供分区内部状态使用
    virtual void collectRound(MCGAL::Halfedge* seed,
                              const std::vector<MCGAL::Halfedge*>& seeds,
                              const std::vector<std::uint8_t>& activeMask,
                              int round) {
        (void)seeds;
        (void)activeMask;
        (void)round;
        collect(seed);
    }

    // 将收集到的信息导出到buffer中
    virtual int exportToBuffer(char* buffer, bool enableQuantization = false, bool enableCompress = false) = 0;
    virtual int exportToBufferWithRangeCoder(char* buffer, bool enableQuantization = false) {return 0;};

    /// @brief 本 collector 是否真的实现了 range-coder 导出。
    ///
    /// 基类的 `exportToBufferWithRangeCoder` 默认**返回 0**（"没实现"）。如果
    /// 管线开了 prediction、序列化器就走这条路，而 collector 没实现 —— 结果是
    /// **静默写出 0 字节的轮数据**，产物里只剩 base mesh：解码端于是从缓冲区
    /// 尾部之外读数据，解出垃圾并在几轮后撞断言。
    /// 实测样本：某算法二进制产物 30 轮只有 3252 B（同配置列式 285 KB），
    /// 解码在第 4 轮 `Facet::setSplittable` 断言。
    ///
    /// 所以把「支持与否」提成契约，让序列化器能**大声失败**而不是静默产空。
    virtual bool supportsRangeCoderExport() const { return false; }
    /// @brief 设置是否启用残差编码（RD 预测路径）。默认关闭；启用后 collector 导出残差而非原坐标
    virtual void setEnablePrediction(bool v) { enablePrediction_ = v; }
    virtual std::vector<MCGAL::Halfedge*> exportSeeds() {
      return seeds_;
    }

    std::shared_ptr<MCGAL::Mesh> getMesh() const { return mesh_; }

    /// @brief 是否保留 lifted base-mesh 量化（PPMC / PPMCSegmentation / PCLTTMTag 等）。
    ///        框架侧通过此接口决定 base-mesh 量化位数是否 +1，无需识别具体 collector。
    virtual bool preservesLiftedBaseMesh() const { return false; }

    /// @brief Gate-level collector 是否暴露了 gate residual / valence / isPoly 队列（RDTag）。
    virtual bool hasGateLevelSymbolCheck() const { return false; }
    virtual const std::vector<std::deque<MCGAL::PointInt>>& gateResidualQueues() const {
        static const std::vector<std::deque<MCGAL::PointInt>> empty;
        return empty;
    }
    virtual const std::vector<std::deque<std::uint8_t>>& gateValenceQueues() const {
        static const std::vector<std::deque<std::uint8_t>> empty;
        return empty;
    }
    virtual const std::vector<std::deque<std::uint8_t>>& gateIsPolyQueues() const {
        static const std::vector<std::deque<std::uint8_t>> empty;
        return empty;
    }
    virtual const std::vector<std::deque<std::uint8_t>>& gateOtherValQueues() const {
        static const std::vector<std::deque<std::uint8_t>> empty;
        return empty;
    }

    /// @brief Unified paper-symbol accessor. Subclasses that mirror their
    ///        per-round state into `pmsf::RoundSymbols` expose it here so
    ///        downstream storage code can consume the paper-level contract
    ///        Delta_r = (F, E^t, E^f, V, C^f, C^v) through
    ///        pmsf::toPMSFSymbolStream() instead of recognizing each
    ///        algorithm-specific collector type.
    ///
    ///        Default implementation maps `rounds()` through the role-aware
    ///        pmsf::toPMSFSymbolStream() -> pmsf::toPMSFSymbolTable() path;
    ///        collectors that still return
    ///        nullptr from `rounds()` remain legacy-only and produce no
    ///        paper table unless they override pmsfSymbolTable() directly.
    virtual const std::vector<pmsf::RoundSymbols>* rounds() const { return nullptr; }

    /// @brief Partition-aware paper symbol table. Segmented/random-access
    ///        collectors should expose {Delta_r^p} here so storage backends do
    ///        not need collector-specific group/round offset tables.
    virtual const pmsf::PMSFSymbolTable* pmsfSymbolTable() const {
        const auto* rs = rounds();
        if (!rs) return nullptr;
        paperTableCache_ = pmsf::toPMSFSymbolTable(
            pmsf::toPMSFSymbolStream(*rs));
        return &paperTableCache_;
    }

    /// @brief Already-compressed payloads that still belong to the paper
    ///        relation key (object_id, partition_id, lod_id, payload_name).
    ///        Storage backends can write these through the generic blob table
    ///        instead of allocating one physical table per codec.
    virtual const pmsf::PMSFSymbolPayloadTable* paperSymbolPayloadTable() const {
        return nullptr;
    }

    std::shared_ptr<MCGAL::Mesh> mesh_;
    std::vector<MCGAL::Halfedge*> seeds_;
    PMSFHeader header_;
    bool enablePrediction_ = false;
    mutable pmsf::PMSFSymbolTable paperTableCache_;
};
#endif
