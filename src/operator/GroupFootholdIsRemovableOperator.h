/*
 * @file GroupFootholdIsRemovableOperator.h
 * @brief PHASE61 物品 1 ①：保证**每个分区在基网上至少留一个面**。
 *
 * 背景（附 30 / 30b / 30c 实测）：
 * `BoundaryIsRemovableOperator` 间接保护了整条分区边界，代价是基网塌不下去
 * （N=128 时 8,238 顶点、BPV 55.53 出带）。摘掉它基网塌到 593 顶点、BPV 35.26
 * **进带**，但 3 个分区在基网上一个面都不剩 ⇒ 解码端该组的符号流无落脚点 ⇒
 * `Assertion !rcDecodedFacetSymbols_.empty()' failed`。
 *
 * 附 30b 又证伪了「保护每组 seed 就够」（消失分区从 3 变成 6）——
 * 需要的保证是 **facet 级**的，不是顶点级。
 *
 * 本算子给出**充分条件**：给每个分区指定一个**代表面**，禁止任何会消掉它的
 * 消除操作。代表面活着 ⇒ 该组在基网上至少有一个面 ⇒ 落脚点保住。
 * 比附 30c 规格里的「全局每组存活面计数」便宜得多：**纯局部判断**
 * （只看候选顶点 1-ring，≈6 个面）。
 *
 * 只对分区路径有意义：不分区时 `footholds_` 为空 ⇒ 恒返回 true ⇒ 零影响。
 */
#ifndef GROUP_FOOTHOLD_ISREMOVABLEOPERATOR_H
#define GROUP_FOOTHOLD_ISREMOVABLEOPERATOR_H

#include "Halfedge.h"
#include "Vertex.h"
#include "common/BoundedRingTraversal.h"
#include "operator/IsRemovableOperator.h"
#include <unordered_set>
#include <vector>

class GroupFootholdIsRemovableOperator : public IsRemovableOperator {
  public:
    GroupFootholdIsRemovableOperator() = default;

    /// 每个分区一个代表面（下标 = 分区号；nullptr 表示该组无代表面）。
    void setFootholds(std::vector<MCGAL::Facet*> footholds) {
        footholds_ = std::move(footholds);
        set_.clear();
        for (MCGAL::Facet* f : footholds_) {
            if (f != nullptr) set_.insert(f);
        }
    }

    bool isRemovable(HalfedgeErrorEntry& entry) {
        if (set_.empty()) return true;  // 未分区：不干预
        MCGAL::Halfedge* h = entry.edge;
        if (h == nullptr) return true;
        // 消掉一个中心顶点会抹掉它 1-ring 上的**全部**面；
        // 只要其中任何一个是某分区的代表面，就拒绝 —— 否则该分区会消失。
        // ⚠️ **两个端点都要看**：`isRemovable` 拿到的是 entry.edge，而消除会
        // 同时抹掉**两端** 1-ring 上的面。第一版只看 `h->vertex()`，
        // 实测 N=128 仍有 1 个分区消失（其余全部保住）—— 正是漏了另一端。
        for (MCGAL::Halfedge* ring : pmsf::boundedRing(h->vertex())) {
            if (ring != nullptr && set_.count(ring->face()) != 0) return false;
        }
        if (h->end_vertex() != h->vertex()) {
            for (MCGAL::Halfedge* ring : pmsf::boundedRing(h->end_vertex())) {
                if (ring != nullptr && set_.count(ring->face()) != 0) return false;
            }
        }
        return true;
    }

  private:
    std::vector<MCGAL::Facet*> footholds_;
    std::unordered_set<MCGAL::Facet*> set_;
};

#endif
