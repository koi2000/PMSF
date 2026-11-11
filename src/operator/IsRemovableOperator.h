/*
 * @Author: koi
 * @Date: 2025-08-25 22:29:32
 * @Description:
 */
#ifndef ISREOVABLEOPERATOR_H
#define ISREOVABLEOPERATOR_H

#include "../common/HalfedgeErrorEntry.h"
#include "../core/SelectionPolicy.h"
#include "Halfedge.h"
#include "Vertex.h"

class IsRemovableOperator {
  public:
    IsRemovableOperator() = default;

    virtual void init(std::vector<MCGAL::Vertex*> v, std::vector<MCGAL::Halfedge*> h, std::vector<MCGAL::Facet*> f){};

    virtual bool isRemovable(HalfedgeErrorEntry& entry) = 0;

    /// @brief 下发选择策略。由 `SelectOptions::Builder::build()` 统一调用，
    ///        算子自己不需要关心「策略从哪来」。
    void setSelectionPolicy(const pmsf::SelectionPolicy& policy) { policy_ = policy; }
    const pmsf::SelectionPolicy& selectionPolicy() const { return policy_; }

  protected:
    pmsf::SelectionPolicy policy_;
};

#endif