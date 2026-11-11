/*
 * @Author: koi
 * @Date: 2025-09-17 17:19:40
 * @Description:
 */
/*
 * @Author: koi
 * @Date: 2025-08-29 21:11:03
 * @Description:
 */

#ifndef SEGMENTATION_SYMBOLCOLLECTOPERATOR_H
#define SEGMENTATION_SYMBOLCOLLECTOPERATOR_H

#include "SymbolCollectOperator.h"
#include <memory>
#include <vector>

class SegmentationSymbolCollectOperator : public SymbolCollectOperator {
  public:
    SegmentationSymbolCollectOperator() = default;

    ~SegmentationSymbolCollectOperator(){};

    /// P18.2 修复：**必须覆写 `init()`**。
    ///
    /// 原实现把「惰性 resize 四个 segment 队列」与 `groupIdx = 0` 都放在
    /// `collect(std::vector<seeds>)` 里；但管线走的是
    /// `collect(seed, gates)` → 基类 → `collect(seed)`（单 seed 重载），
    /// **从不调用 vector 重载** ⇒ 队列始终为空、`groupIdx` 为陈旧值，
    /// 于是 `segmentFacetSymbolQueues[groupIdx]` 越界崩溃
    /// （gdb 实测 SIGSEGV @ SegmentationSymbolCollectOperator.cpp:139）。
    ///
    /// 本类此前全树零构造点（见 docs 的 P18.2 勘定），第一次被 EFOSS 真正
    /// 实例化就崩 —— 这就是原因。把初始化挪到管线**一定会调用**的 `init()`
    /// 上，两条重载就都安全了。
    void init(std::shared_ptr<MCGAL::Mesh> mesh,
              std::vector<MCGAL::Halfedge*> seeds,
              PMSFHeader header) override {
        SymbolCollectOperator::init(mesh, seeds, header);
        if (!inited) {
            inited = true;
            const std::size_t n = seeds.empty() ? 1u : seeds.size();
            segmentFacetSymbolQueues.resize(n);
            segmentEdgeSymbolQueues.resize(n);
            segmentIpointQueues.resize(n);
            segmentPointQueues.resize(n);
        }
        groupIdx = 0;
    }

    void collect(MCGAL::Halfedge* seed) override;
    void collect(std::vector<MCGAL::Halfedge*> seed) override;
    int exportToBuffer(char* buffer, bool enableQuantization = false, bool enableCompress = false) override;
    const pmsf::PMSFSymbolTable* pmsfSymbolTable() const override { return &paperTable_; }

  private:
    void collectHalfedgeSymbol(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Halfedge* seed);
    void collectFacetSymbol(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Halfedge* seed);

  public:
    // Parquet V2 多表方案需要直接访问内部队列
    const std::vector<std::vector<std::deque<MCGAL::Point>>>& getSegmentPointQueues() const { return segmentPointQueues; }
    const std::vector<std::vector<std::deque<MCGAL::PointInt>>>& getSegmentIpointQueues() const { return segmentIpointQueues; }
    const std::vector<std::vector<std::deque<char>>>& getSegmentFacetSymbolQueues() const { return segmentFacetSymbolQueues; }
    const std::vector<std::vector<std::deque<char>>>& getSegmentEdgeSymbolQueues() const { return segmentEdgeSymbolQueues; }

  private:
    std::vector<std::vector<std::deque<MCGAL::Point>>> segmentPointQueues;
    std::vector<std::vector<std::deque<MCGAL::PointInt>>> segmentIpointQueues;
    std::vector<std::vector<std::deque<char>>> segmentFacetSymbolQueues;
    std::vector<std::vector<std::deque<char>>> segmentEdgeSymbolQueues;
    pmsf::PMSFSymbolTable paperTable_;
    int groupIdx = 0;
    int inited = false;
    int groupId = -1;

    int fcount = 0;
    int ecount = 0;
};

#endif
