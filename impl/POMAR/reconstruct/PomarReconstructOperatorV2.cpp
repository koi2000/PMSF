/*
 * @Author: pomar-refactor
 * @Description: Pomar 存储模块重构版 —— 重建操作符实现
 */
#include "PomarReconstructOperatorV2.h"

#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Vertex.h"

#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <string>
#include <vector>

namespace {
struct PomarApplyStats {
    unsigned long long total = 0;
    unsigned long long invalidBeforeFallback = 0;
    unsigned long long recoveredByFallback = 0;
    unsigned long long skipped = 0;
};

PomarApplyStats& pomarApplyStats() {
    static PomarApplyStats stats;
    return stats;
}

bool pomarApplyStatsEnabled() {
    static const bool enabled = [] {
        const char* v = std::getenv("PMSF_POMAR_APPLY_STATS");
        return v != nullptr && v[0] != '\0' && v[0] != '0';
    }();
    return enabled;
}
}  // namespace

void PomarReconstructOperatorV2::applyOne(MCGAL::PerformedOperation& op) {
    auto validSplit = [](const MCGAL::PerformedOperation& candidate) {
        return candidate.v1 && candidate.vr && candidate.vhPivotL && candidate.vhPivotR &&
               candidate.v1 != candidate.vhPivotL &&
               candidate.v1 != candidate.vhPivotR &&
               candidate.vhPivotL != candidate.vhPivotR &&
               candidate.v1->find_halfedge(candidate.vhPivotL) != nullptr &&
               candidate.v1->find_halfedge(candidate.vhPivotR) != nullptr;
    };
    PomarApplyStats* stats = pomarApplyStatsEnabled() ? &pomarApplyStats() : nullptr;
    if (stats) stats->total++;
    if (!validSplit(op)) {
        if (stats) stats->invalidBeforeFallback++;
        if (op.v1) {
            for (MCGAL::Halfedge* h : *op.v1) {
                MCGAL::Halfedge* prev = h ? h->find_prev() : nullptr;
                MCGAL::Vertex* left = h ? h->end_vertex() : nullptr;
                MCGAL::Vertex* right = prev ? prev->vertex() : nullptr;
                if (!left || !right || left == right || left == op.v1 || right == op.v1) {
                    continue;
                }
                op.vhPivotL = left;
                op.vhPivotR = right;
                if (validSplit(op)) {
                    if (stats) stats->recoveredByFallback++;
                    break;
                }
            }
        }
    }
    if (!validSplit(op)) {
        if (stats) stats->skipped++;
        return;
    }
    mesh_->new_vertex_split(op.vr, op.v1, op.vhPivotL, op.vhPivotR);
    if (op.v1->b_marked) {
        op.vr->b_marked = true;
    }
    op.vr->setGroupId(op.v1->groupId());

    for (MCGAL::Halfedge* cvh_it : *op.vr) {
        cvh_it->face()->updateNormal();
    }
}

void PomarReconstructOperatorV2::applyOperations(std::queue<MCGAL::PerformedOperation>& opToPerform) {
    const char* sortEnv = std::getenv("PMSF_POMAR_APPLY_SORT");
    if (sortEnv && sortEnv[0] != '\0' && sortEnv[0] != '0') {
        std::vector<MCGAL::PerformedOperation> ops;
        while (!opToPerform.empty()) {
            ops.push_back(opToPerform.front());
            opToPerform.pop();
        }
        const std::string mode(sortEnv);
        auto vid = [](MCGAL::Vertex* v) { return v ? v->poolId() : -1; };
        if (mode == "v1_asc" || mode == "v1_desc") {
            std::sort(ops.begin(), ops.end(), [&](const auto& a, const auto& b) {
                return mode == "v1_asc" ? vid(a.v1) < vid(b.v1)
                                        : vid(a.v1) > vid(b.v1);
            });
        } else if (mode == "v2_asc" || mode == "v2_desc") {
            std::sort(ops.begin(), ops.end(), [&](const auto& a, const auto& b) {
                return mode == "v2_asc" ? vid(a.v2) < vid(b.v2)
                                        : vid(a.v2) > vid(b.v2);
            });
        } else if (mode == "vr_asc" || mode == "vr_desc") {
            std::sort(ops.begin(), ops.end(), [&](const auto& a, const auto& b) {
                return mode == "vr_asc" ? vid(a.vr) < vid(b.vr)
                                        : vid(a.vr) > vid(b.vr);
            });
        }
        for (auto& op : ops) {
            applyOne(op);
        }
        if (pomarApplyStatsEnabled()) {
            const auto& stats = pomarApplyStats();
            std::cerr << "[POMAR apply stats] total=" << stats.total
                      << " invalid_before_fallback=" << stats.invalidBeforeFallback
                      << " recovered_by_fallback=" << stats.recoveredByFallback
                      << " skipped=" << stats.skipped << std::endl;
        }
        return;
    }
    const char* reverseEnv = std::getenv("PMSF_POMAR_APPLY_REVERSE");
    if (reverseEnv && reverseEnv[0] != '\0' && reverseEnv[0] != '0') {
        std::vector<MCGAL::PerformedOperation> ops;
        while (!opToPerform.empty()) {
            ops.push_back(opToPerform.front());
            opToPerform.pop();
        }
        for (auto it = ops.rbegin(); it != ops.rend(); ++it) {
            applyOne(*it);
        }
        if (pomarApplyStatsEnabled()) {
            const auto& stats = pomarApplyStats();
            std::cerr << "[POMAR apply stats] total=" << stats.total
                      << " invalid_before_fallback=" << stats.invalidBeforeFallback
                      << " recovered_by_fallback=" << stats.recoveredByFallback
                      << " skipped=" << stats.skipped << std::endl;
        }
        return;
    }
    while (!opToPerform.empty()) {
        MCGAL::PerformedOperation op = opToPerform.front();
        opToPerform.pop();
        applyOne(op);
    }
    if (pomarApplyStatsEnabled()) {
        const auto& stats = pomarApplyStats();
        std::cerr << "[POMAR apply stats] total=" << stats.total
                  << " invalid_before_fallback=" << stats.invalidBeforeFallback
                  << " recovered_by_fallback=" << stats.recoveredByFallback
                  << " skipped=" << stats.skipped << std::endl;
    }
}
