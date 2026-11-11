#ifndef GREEDY_SELECTOR_H
#define GREEDY_SELECTOR_H

#include "Halfedge.h"
#include "Mesh.h"
#include "SelectOperator.h"
#include "common/FixedQueue.h"
#include "operator/IsRemovableOperator.h"
#include "selection/HalfedgeSelectionQueue.h"
#include "core/SelectionPolicy.h"

#include <memory>

class GreedySelector : public SelectOperator {
  public:
  GreedySelector(SelectOptions options);
    void init(std::shared_ptr<MCGAL::Mesh> mesh, bool singleGroupOnly = false, int groupId = 0) override;
    GreedySelector();
    ~GreedySelector();
    bool select(HalfedgeErrorEntry& entry) override;
    bool update(HalfedgeErrorEntry& entry) override;
    void addIsRemovableOperator(std::shared_ptr<IsRemovableOperator> op) override;
    void reset(bool singleGroupOnly = false, int groupId = 0) override;
    void selectSeed(MCGAL::Halfedge*& seed, std::function<MCGAL::Halfedge*(std::shared_ptr<MCGAL::Mesh>)> seedSelector = nullptr) override;

    /// @brief 每轮重新播种：随机起点线性扫描，取第一个"活"面作为种子。
    ///        参考实现每个 decimation conquest 都重新 rand() 选种子。
    void reseedLive() override;

    /// @brief 刷新所有 live 顶点的 degree 缓存（见 CompressPipelineConfig 注释）。
    void refreshRoundState() override;
    void setRefreshDegreesEachRound(bool on) override {
        refreshDegreesEachRound_ = on;
    }

    bool isRemovable(MCGAL::Halfedge* h);
    bool inActiveGroup(MCGAL::Halfedge* h) const;

    bool singleGroupOnly_ = false;
    int groupId_ = -1;
    bool freshProbeDone_ = false;
    bool refreshDegreesEachRound_ = false;
    FixedQueue<MCGAL::Halfedge*> gateQueue = FixedQueue<MCGAL::Halfedge*>(100000);
    int visited_version = -1;
    std::vector<std::shared_ptr<IsRemovableOperator>> isRemovableOperators_;
    /// 选择策略（见 core/SelectionPolicy.h）。构造时从 SelectOptions 取出 ——
    /// 算法特有的候选打分由它带进来，selector 本身不认识任何算法。
    pmsf::SelectionPolicy policy_;
    int i_nbRemovedVertices = 0;
    MCGAL::Halfedge* seed_ = nullptr;
};

#endif
