/*
 * @Author: koi
 * @Date: 2025-08-26 22:22:37
 * @Description: 
 */
#ifndef SEGMENT_OPERATOR_H
#define SEGMENT_OPERATOR_H

#include "Halfedge.h"
#include "Mesh.h"
#include "core.h"
#include <memory>
#include <vector>
#include "../common/Graph.h"

/**
 * 传入一个网格，然后可以自行操作如何进行segment，可以使用很多现有的分区方法
 * 返回临界图
 * 更改分区的单位为vertex
*/
class SegmentOperator {
public:
    SegmentOperator() = default;
    virtual ~SegmentOperator() = default;

    virtual void segment(std::shared_ptr<MCGAL::Mesh>& mesh) = 0;

    virtual std::vector<MCGAL::Halfedge*> exportSeeds() = 0;

    virtual std::shared_ptr<Graph> exportGraph() = 0;

    std::shared_ptr<MCGAL::Mesh> mesh;

    std::shared_ptr<Graph> g;
};

#endif