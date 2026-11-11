/*
 * @Author: koi
 * @Date: 2025-09-13 22:25:20
 * @Description:
 */
#ifndef BASCI_SEGMENT_OPERATOR_H
#define BASCI_SEGMENT_OPERATOR_H

#include "Halfedge.h"
#include "Mesh.h"
#include "SegmentOperator.h"
#include <memory>
#include <vector>

/**
 * 传入一个网格，然后可以自行操作如何进行segment，可以使用很多现有的分区方法
 * 返回临界图
 * 关于segment的主要问题有以下几个，
 * 1. 如何分区
 * 2. 边界如何处理，边界上重复的点如何处理
 * segment 分两种方式，第一种是指定数量 预分区 + 边界处理
 * 第二种就是由压缩算法来指定分区数量
 */
class BasicSegmentOperator : public SegmentOperator {
  public:
    BasicSegmentOperator(int number) : number(number) {}
    ~BasicSegmentOperator() = default;

    void segment(std::shared_ptr<MCGAL::Mesh>& mesh);

    void markBoundary(std::shared_ptr<MCGAL::Mesh>& mesh);

    std::vector<MCGAL::Halfedge*> exportSeeds();

    std::shared_ptr<Graph> exportGraph();

    void buildGraph();

    /// P18.2 阶段 B 物品 1：exact-N 模式。
    ///
    /// 默认 `false` ⇒ 保持既有 merge 语义（组数是「目标」不是「保证」：
    /// 实测 N=8→7、16→17、64→63），既有配置与门禁逐字节不变。
    /// `true` ⇒ 关闭 merge 分支、跑满 `number` 组、最后一组吸收余量，
    /// 使实际组数**恰好**等于 `number`（论文表需要）。
    void setExactNumSegments(bool on) { exactN_ = on; }
    bool isExactNumSegments() const { return exactN_; }

    std::vector<MCGAL::Halfedge*> seeds;
    int number;

  private:
    bool exactN_ = false;
};

#endif