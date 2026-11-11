/*
 * @file AnchorVetoPredicate.h
 * @brief H2 DecimationPredicate 的锚点保留实现（APTM 语义的 hook 形态）。
 *
 * 范式实证（P13.3 S5 hook 的第一次真实接入）：锚点集合由**调用方**给定
 * （env `PMSF_CONQUEST_ANCHOR_IDS`，逗号分隔的 poolId —— poolId 在
 * 删减全程稳定，顶点只删不建）。判定 = 默认判据 ∧ 非锚点。
 *
 * 为什么解码端零改动：被 veto 的锚点只是**留在 base mesh 里**随基网格
 * 原样传输，残差与符号流对它一概不发 —— 往返完整性由构造保证。
 *
 * 此文件在 impl/，可出现方案语义；src/ 只认识 EngineHooks 接口。
 */
#ifndef PMSF_CONQUEST_HOOKS_ANCHOR_VETO_PREDICATE_H
#define PMSF_CONQUEST_HOOKS_ANCHOR_VETO_PREDICATE_H

#include "scheme/EngineHooks.h"

#include <cstdint>
#include <unordered_set>

namespace pmsf {

class AnchorVetoPredicate final : public DecimationPredicate {
  public:
    explicit AnchorVetoPredicate(std::unordered_set<std::uint32_t> anchors)
        : anchors_(std::move(anchors)) {}

    bool canDecimate(const FrontVertexView& v) const override {
        if (anchors_.count(v.id) != 0)
            return false;  // 锚点保留（APTM 的 anchor-preservation 语义）
        return DecimationPredicate::canDecimate(v);
    }

    std::size_t anchorCount() const { return anchors_.size(); }

  private:
    std::unordered_set<std::uint32_t> anchors_;
};

}  // namespace pmsf

#endif  // PMSF_CONQUEST_HOOKS_ANCHOR_VETO_PREDICATE_H
