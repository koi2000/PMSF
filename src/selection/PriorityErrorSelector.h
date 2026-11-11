/*
 * @Author: koi
 * @Date: 2025-08-27 11:35:42
 * @Description: 根据error来进行选择的selector
 */
#ifndef PRIORITY_ERROR_SELECTOR_H
#define PRIORITY_ERROR_SELECTOR_H

#include "Mesh.h"
#include "SelectOperator.h"
#include "selection/HalfedgeSelectionQueue.h"
#include "operator/IsRemovableOperator.h"
#include <cstdint>
#include <memory>
#include <vector>

class PriorityErrorSelector : public SelectOperator {
  public:
    PriorityErrorSelector(SelectOptions options);
    void init(std::shared_ptr<MCGAL::Mesh> mesh, bool singleGroupOnly = false, int groupId = 0) override;
    void initFromFaces(std::shared_ptr<MCGAL::Mesh> mesh,
                       const std::vector<MCGAL::Facet*>& faces);
    PriorityErrorSelector();
    ~PriorityErrorSelector();
    bool select(HalfedgeErrorEntry& entry) override;
    bool update(HalfedgeErrorEntry& entry) override;
    void addIsRemovableOperator(std::shared_ptr<IsRemovableOperator> op) override;
    void reset(bool singleGroupOnly = false, int groupId = 0) override;

  private:
    HalfedgeSelectionQueue queue;

    // PHASE67: `update()` is called once per elimination and used to build two
    // std::unordered_set<int> per call -- every insert allocating a node (~95 per
    // call, ~16M for armadillo; measured 6.2% of runtime in malloc/_int_malloc).
    // These epoch stamps replace them: O(1) membership, zero allocation per call.
    // The sets only gate WHETHER to append/push; both iteration orders come from
    // the `affected` vector, so the sequence of pushes is identical =>
    // byte-identical output (verified by the gate).
    std::vector<std::uint64_t> seenStamp_;
    std::vector<std::uint64_t> pushedStamp_;
    std::uint64_t seenEpoch_ = 0;
    std::uint64_t pushedEpoch_ = 0;
};

#endif
