#pragma once
// BFS facet-group relabeling from one permanent anchor per group.
#include "Mesh.h"
#include <deque>
#include <memory>
#include <unordered_map>
#include <vector>

namespace pmsf {

inline std::vector<MCGAL::Vertex*>& bfsGroupAnchors() {
    static std::vector<MCGAL::Vertex*> anchors;
    return anchors;
}

inline void bfsRelabelFacetGroups(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                  const std::vector<MCGAL::Vertex*>& anchors) {
    if (!mesh || anchors.empty()) return;
    std::unordered_map<MCGAL::Facet*, int> label;
    std::deque<MCGAL::Facet*> dq;
    for (std::size_t g = 0; g < anchors.size(); ++g) {
        MCGAL::Vertex* a = anchors[g];
        if (!a || a->isRemoved()) continue;
        MCGAL::Facet* f = nullptr;
        for (MCGAL::Halfedge* h : a->halfedges())
            if (h && h->face() && !h->face()->isRemoved()) { f = h->face(); break; }
        if (!f || label.count(f)) continue;
        label[f] = static_cast<int>(g);
        dq.push_back(f);
    }
    while (!dq.empty()) {
        MCGAL::Facet* f = dq.front(); dq.pop_front();
        const int g = label[f];
        MCGAL::Halfedge* h0 = f->proxyHalfedge();
        MCGAL::Halfedge* h = h0;
        int guard = 0;
        do {
            if (h && h->opposite() && h->opposite()->face()) {
                MCGAL::Facet* nb = h->opposite()->face();
                if (!nb->isRemoved() && !label.count(nb)) {
                    label[nb] = g;
                    dq.push_back(nb);
                }
            }
            h = h ? h->next() : nullptr;
        } while (h && h != h0 && ++guard < 64);
    }
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        auto it = label.find(f);
        if (it != label.end()) f->setGroupId(it->second);
    }
}

}  // namespace pmsf
