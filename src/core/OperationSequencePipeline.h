// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <stdexcept>

namespace pmsf {

/// Selection -> removability -> elimination -> symbol collection -> update.
/// Representation-independent companion to the halfedge pipeline. The state
/// and operators remain algorithm-owned; the framework controls their order.
/// This permits non-manifold pair contractions without a halfedge link test.
struct OperationSequenceStats {
    std::size_t selected = 0;
    std::size_t rejected = 0;
    std::size_t eliminated = 0;
};

template<class Entry, class Select, class Removable, class Eliminate,
         class Collect, class Update>
OperationSequenceStats runOperationSequence(Select&& select, Removable&& removable,
                                            Eliminate&& eliminate, Collect&& collect,
                                            Update&& update) {
    OperationSequenceStats stats;
    Entry entry;
    while (select(entry)) {
        ++stats.selected;
        if (!removable(entry)) {
            ++stats.rejected;
            continue;
        }
        auto symbols = eliminate(entry);
        collect(symbols);
        update(entry);
        ++stats.eliminated;
    }
    return stats;
}

}  // namespace pmsf
