/*
 * @Author: koi
 * @Date: 2025-08-26 22:01:15
 * @Description:
 */
/*
 * @Author: koi
 * @Description: 请填写简介
 */
#ifndef SELECT_OPERATOR_H
#define SELECT_OPERATOR_H

#include "Halfedge.h"
#include "Mesh.h"
#include "common/HalfedgeErrorEntry.h"
#include "operator/IsRemovableOperator.h"
#include "options/SelectOptions.h"
#include <functional>
#include <memory>

/**
 * 使用的顺序是先select出来一条边，传入的会有IsRemovable，用于检查选出来的时候可以用
 * 然后进入eliminate，eliminate后会进行一系列的mark，然后会有一个postprocessor
 *
 */

class SelectOperator {
  public:
    SelectOperator(SelectOptions options) : options_(options){};

    virtual void init(std::shared_ptr<MCGAL::Mesh> mesh, bool singleGroupOnly = false, int groupId = 0) = 0;

    virtual ~SelectOperator() = default;
    /**
     * @brief 选择一条边
     *
     * @param halfedge 指针形式传入，函数内部设置
     * @return true 这一轮仍有可以选择的会返回true，否则会返回false
     * @return false
     */
    virtual bool select(HalfedgeErrorEntry& entry) = 0;

    virtual bool update(HalfedgeErrorEntry& entry) = 0;

    virtual void addIsRemovableOperator(std::shared_ptr<IsRemovableOperator> op) = 0;

    virtual void selectSeed(MCGAL::Halfedge*& seed, std::function<MCGAL::Halfedge*(std::shared_ptr<MCGAL::Mesh>)> seedSelector = nullptr) {
        if (seedSelector) {
            seed = seedSelector(mesh_);
        } else {
            size_t i_heInitId = (float)rand() / RAND_MAX * mesh_->size_of_facets();
            seed = mesh_->get_facet(i_heInitId)->proxyHalfedge();
        }
    };

    virtual void reset(bool singleGroupOnly = false, int groupId = 0) = 0;

    /// @brief 重新选择一个"活"的种子半边（部分算法每轮重新随机播种，见
    ///        CompressPipelineConfig::reseedEachRound）。默认无操作。
    virtual void reseedLive() {}

    /// @brief 每轮 reset 之后、选择开始之前的一次性「轮状态修正」钩子。
    /// 默认空实现（其它算法字节不变）。
    virtual void refreshRoundState() {}

    /// @brief 请求 selector 在每轮第一次 select 前刷新顶点 degree 缓存。
    /// 默认空实现（其它算法字节不变）。
    virtual void setRefreshDegreesEachRound(bool) {}

    std::shared_ptr<MCGAL::Mesh> mesh_;
    SelectOptions options_;
};

#endif