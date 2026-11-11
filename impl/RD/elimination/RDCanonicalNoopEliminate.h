// SPDX-License-Identifier: Apache-2.0
//
// RDCanonicalNoopEliminate.h — task-43 route A 的**空消除算子**。
//
// route A 的遍历（RDCanonicalTraversal）**自带消除**：gate queue 的 pop 决定
// 谁被删、怎么重三角化。若框架的 select/eliminate 链再删一遍，mesh 会被删两次、
// encoder 的 conquest 状态与 decoder replay 立刻分岔。故 PMSF_RD_CANON=1 时
// 把 eliminate 换成恒 false 的空算子，每轮形状变成：
//   select（空转一遍）→ collector->collect(seed)：**遍历在这里删点**
// 只在本 env 分支被构造；默认 rd_tag 路径完全不认识它。
#ifndef PMSF_IMPL_RD_RD_CANONICAL_NOOP_ELIMINATE_H
#define PMSF_IMPL_RD_RD_CANONICAL_NOOP_ELIMINATE_H

#include "elimination/EliminateOperator.h"

#include <memory>
#include <utility>

namespace rd_canon {

class RDCanonicalNoopEliminateOperator final : public EliminateOperator {
  public:
    void init(std::shared_ptr<MCGAL::Mesh> mesh, PMSFHeader header,
              bool compress_boundary = false) override {
        mesh_ = std::move(mesh);
        (void)header;
        (void)compress_boundary;
    }

    bool eliminate(HalfedgeErrorEntry& /*entry*/) override {
        return false;
    }
    bool remove_point(MCGAL::Halfedge* /*h*/) override {
        return false;
    }
    bool triangulate(MCGAL::Halfedge* /*h*/) override {
        return false;
    }
    bool postprocess(MCGAL::Halfedge* /*h*/) override {
        return false;
    }
    MCGAL::Halfedge* encode_boundary(MCGAL::Halfedge* /*h*/) override {
        return nullptr;
    }

  private:
    std::shared_ptr<MCGAL::Mesh> mesh_;
};

}  // namespace rd_canon

#endif  // PMSF_IMPL_RD_RD_CANONICAL_NOOP_ELIMINATE_H
