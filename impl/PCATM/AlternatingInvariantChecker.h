// SPDX-License-Identifier: Apache-2.0
//
// AlternatingInvariantChecker.h — PCATM Z-shape 的 alternating invariant
// 校验器（论文 §4 核心不变量）：
//
//   "整个 mesh 上不存在 color='1' (BLUE) 的两个 triangle 共享 edge"
//
// 用途：
//   1. encoder PCATMZShapeEliminateOperator 每完成一轮 retriangulation 后
//      在 debug build 调 check(...) (assert)；release build 失败时降级为
//      4-color mode (PR-3 接入)；
//   2. 单元测试 tests/test_alternating_invariant.cpp 用其构造 known-good /
//      known-bad mesh 验证；
//   3. PR-3 端到端 round-trip 时每轮跑 check(mesh)。
//
// 详见 docs/paper_alignment/PCATM_OFFICIAL_REPRO_PROMPT.md PR-2 §R5。

#ifndef SRC_COMMON_ALTERNATING_INVARIANT_CHECKER_H_
#define SRC_COMMON_ALTERNATING_INVARIANT_CHECKER_H_

#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"

#include <cstddef>
#include <utility>
#include <vector>

namespace pmsf {
namespace pcatm {

struct AlternatingViolation {
    MCGAL::Facet* faceA;
    MCGAL::Facet* faceB;  // 与 faceA 共享某条 edge，且两者均为 BLUE (color=1)
};

class AlternatingInvariantChecker {
  public:
    /// @brief 扫全 mesh，检查 alternating invariant。
    /// @return true 全 mesh 不变量成立；false 出现违规（详见 violations()）。
    bool check(MCGAL::Mesh* mesh) {
        violations_.clear();
        if (!mesh) return true;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved()) continue;
            if (f->getTriangleColor() != MCGAL::Facet::BLUE) continue;
            // 遍历 f 的 halfedge ring
            for (auto it = f->halfedges_begin(); it != MCGAL::Facet::halfedge_iterator(nullptr); ++it) {
                MCGAL::Halfedge* h = *it;
                if (!h) continue;
                MCGAL::Halfedge* opp = h->opposite();
                if (!opp) continue;
                MCGAL::Facet* g = opp->face();
                if (!g || g->isRemoved()) continue;
                if (g == f) continue;
                if (g->getTriangleColor() == MCGAL::Facet::BLUE) {
                    // 去重：仅当 (f, g) 中 f 指针更小时记录，避免对称重复。
                    if (f < g) {
                        violations_.push_back({f, g});
                    }
                }
            }
        }
        return violations_.empty();
    }

    /// 单 N-gon polygon ring 的 patch-内 alternating invariant 检查。
    /// 入参：N-2 个 triangle 的 color 序列。
    static bool checkLocalSequence(const std::vector<int>& colorSeq) {
        for (std::size_t i = 1; i < colorSeq.size(); ++i) {
            if (colorSeq[i] == colorSeq[i - 1]) return false;
        }
        return true;
    }

    const std::vector<AlternatingViolation>& violations() const { return violations_; }

  private:
    std::vector<AlternatingViolation> violations_;
};

}  // namespace pcatm
}  // namespace pmsf

#endif  // SRC_COMMON_ALTERNATING_INVARIANT_CHECKER_H_
