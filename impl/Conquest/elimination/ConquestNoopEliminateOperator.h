/*
 * @file ConquestNoopEliminateOperator.h
 * @brief **空操作**的消除算子：让框架的 select/eliminate 链对网格**什么都不做**。
 *
 * **为什么需要它**（蓝图 §62.2 路线 A 的另一半）：
 * 其它算法的分工是"框架负责选谁删、怎么删，collector 只做符号映射"；
 * conquest 正相反 —— `FrontierConquestEngine` **自己跑前沿 BFS 并自己删点**
 * （那是本次唯一被逐层验证过的遍历，不能换）。若框架再叠一层 elimination，
 * 网格会被**删两遍**，两侧立刻分岔。
 *
 * 装上本算子之后，每轮的形状变成：
 *   select（跑一遍但什么都不删，`eliminate` 恒返回 false）
 *   → collector->collect(seed)：**引擎在这里删点**，并把本层符号收好
 * 而 `stopOnEmptyRound` 默认为 false ⇒ 框架会老老实实跑满 R 轮，
 * 每轮调一次 `collect`，与"一层一次"的参考语义一一对应。
 *
 * ⚠️ 本类**不认识任何算法语义**：它只是"不施加遍历"这个设计定案的具体载体。
 */
#ifndef PMSF_CONQUEST_ELIMINATION_CONQUEST_NOOP_ELIMINATE_OPERATOR_H
#define PMSF_CONQUEST_ELIMINATION_CONQUEST_NOOP_ELIMINATE_OPERATOR_H

#include "elimination/EliminateOperator.h"

#include <memory>
#include <utility>

namespace pmsf {

class ConquestNoopEliminateOperator final : public EliminateOperator {
  public:
    void init(std::shared_ptr<MCGAL::Mesh> mesh, PMSFHeader header, bool compress_boundary = false) override {
        mesh_ = std::move(mesh);
        (void)header;
        (void)compress_boundary;
    }

    /// 恒 false ⇒ 框架侧一个点都不删；真正的删减在 collector 驱动的引擎里。
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

}  // namespace pmsf

#endif  // PMSF_CONQUEST_ELIMINATION_CONQUEST_NOOP_ELIMINATE_OPERATOR_H
