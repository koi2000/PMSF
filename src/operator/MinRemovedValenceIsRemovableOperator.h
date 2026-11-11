// SPDX-License-Identifier: Apache-2.0
//
// Optional strict-PCATM experiment: reject removed vertices below a configured
// live valence.  This is intentionally opt-in because it can increase the base
// mesh when used to force paper 2-color eligible rounds.

#ifndef PMSF_OPERATOR_MIN_REMOVED_VALENCE_IS_REMOVABLE_OPERATOR_H
#define PMSF_OPERATOR_MIN_REMOVED_VALENCE_IS_REMOVABLE_OPERATOR_H

#include "operator/IsRemovableOperator.h"

#include "Halfedge.h"
#include "Vertex.h"

namespace pmsf {

class MinRemovedValenceIsRemovableOperator final : public IsRemovableOperator {
  public:
    explicit MinRemovedValenceIsRemovableOperator(int minValence)
        : minValence_(minValence) {}

    bool isRemovable(HalfedgeErrorEntry& entry) override {
        if (!entry.edge || !entry.edge->end_vertex()) return false;
        return entry.edge->end_vertex()->vertex_degree() >= minValence_;
    }

  private:
    int minValence_ = 0;
};

}  // namespace pmsf

#endif  // PMSF_OPERATOR_MIN_REMOVED_VALENCE_IS_REMOVABLE_OPERATOR_H
