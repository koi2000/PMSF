/*
 * @Author: koi
 * @Date: 2025-09-13 22:25:20
 * @Description:
 */
#pragma once

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
class PreDefinedVertexSegmentOperator : public SegmentOperator {
  public:
  PreDefinedVertexSegmentOperator(int number) : number(number) {}
    ~PreDefinedVertexSegmentOperator() = default;

    void segment(std::shared_ptr<MCGAL::Mesh>& mesh);

    void markBoundary(std::shared_ptr<MCGAL::Mesh>& mesh);

    std::vector<MCGAL::Halfedge*> exportSeeds();

    std::shared_ptr<Graph> exportGraph();

    void buildGraph();

    std::vector<MCGAL::Halfedge*> seeds;
    int number;
};