/*
 * @file        PatchBFSSort.cpp
 * @brief       Encoder/decoder shared patch-face BFS ordering.
 */

#include "PatchBFSSort.h"

#include "BFSVersionManager.h"
#include "Facet.h"
#include "Halfedge.h"

#include <queue>
#include <unordered_set>

namespace pmsf {

std::vector<MCGAL::Facet*> sortPatchesByBFS(
    const std::vector<MCGAL::Facet*>& patches,
    const std::vector<MCGAL::Halfedge*>& seeds,
    MCGAL::Mesh* mesh) {
    (void)mesh;

    std::vector<MCGAL::Facet*> ordered;
    ordered.reserve(patches.size());
    if (patches.empty()) {
        return ordered;
    }

    std::unordered_set<MCGAL::Facet*> wanted;
    wanted.reserve(patches.size() * 2 + 1);
    for (MCGAL::Facet* f : patches) {
        if (f != nullptr) {
            wanted.insert(f);
        }
    }

    std::unordered_set<MCGAL::Facet*> emitted;
    emitted.reserve(wanted.size() * 2 + 1);
    auto emitIfPatch = [&](MCGAL::Facet* f) {
        if (f != nullptr && wanted.find(f) != wanted.end() && emitted.insert(f).second) {
            ordered.push_back(f);
        }
    };

    std::queue<MCGAL::Halfedge*> gateQueue;
    for (MCGAL::Halfedge* seed : seeds) {
        if (seed != nullptr) {
            gateQueue.push(seed);
        }
    }

    const int currentVersion = MCGAL::BFSVersionManager::current_version++;
    while (!gateQueue.empty()) {
        MCGAL::Halfedge* h = gateQueue.front();
        gateQueue.pop();
        if (h == nullptr || h->isVisited(currentVersion) || h->isRemoved()) {
            continue;
        }
        h->setVisited(currentVersion);

        MCGAL::Facet* f = h->face();
        if (f != nullptr && !f->isVisited(currentVersion)) {
            f->setVisited(currentVersion);
            emitIfPatch(f);
            if (ordered.size() == wanted.size()) {
                return ordered;
            }
        }

        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp != nullptr && !hOpp->isVisited(currentVersion)) {
                gateQueue.push(hOpp);
            }
            if (!hIt->isVisited(currentVersion)) {
                gateQueue.push(hIt);
            }
            hIt = hIt->next();
        } while (hIt != nullptr && hIt != h);
    }

    // Disconnected or seed-unreachable patch faces keep caller order instead
    // of being dropped. This preserves a total deterministic order while the
    // primary connected component still matches the paper/RDGateOnly BFS.
    for (MCGAL::Facet* f : patches) {
        if (f != nullptr && emitted.insert(f).second) {
            ordered.push_back(f);
        }
    }
    return ordered;
}

}  // namespace pmsf
