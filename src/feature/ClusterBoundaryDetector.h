/*
 * @file ClusterBoundaryDetector.h
 * @brief POMAR-style 聚类 + 边界 anchor 检测。
 *
 * 论文 POMAR (Peng et al. 2019) 的核心：
 *   1. 用 region-growing 把面片聚成若干 cluster，criterion 是相邻面法向夹角 < θ。
 *   2. 每个 cluster 内的面共享一个 reference plane，属性（法向、曲率）用 delta
 *      relative to reference 编码，bpv 显著下降。
 *   3. Cluster 边界上的顶点是"anchor" —— 后续 eliminate 保守对待，保证跨 cluster
 *      的 topology 保留。
 *
 * 本 detector 返回 cluster 边界顶点集合（= 1-ring 内面分属 ≥2 cluster 的顶点）。
 * 只实现边界检测部分；cluster id 的编码 side-channel 由调用方挂到 sc_cluster_id 上。
 */
#ifndef PMSF_FEATURE_CLUSTER_BOUNDARY_DETECTOR_H
#define PMSF_FEATURE_CLUSTER_BOUNDARY_DETECTOR_H

#include "FeatureDetector.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Vertex.h"

#include <cmath>
#include <cstdint>
#include <memory>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace pmsf {

class ClusterBoundaryDetector final : public FeatureDetector {
public:
    /// @param normal_angle_cos 相邻面法向点积阈值（余弦）。0.85 ≈ 32°。
    explicit ClusterBoundaryDetector(double normal_angle_cos = 0.85)
        : threshold_(normal_angle_cos) {}

    /// Expose the cluster ids computed in detect() for algorithms that want
    /// to emit them as a side channel. Valid until the next detect() call.
    const std::unordered_map<MCGAL::Facet*, std::uint32_t>& facetClusterIds() const {
        return facetCluster_;
    }

    FeatureSet detect(std::shared_ptr<MCGAL::Mesh> mesh) override {
        facetCluster_.clear();
        FeatureSet boundary;
        if (!mesh) return boundary;

        // ---- Region-growing on facets ----
        std::uint32_t nextId = 0;
        std::queue<MCGAL::Facet*> q;
        for (MCGAL::Facet* seed : mesh->faces()) {
            if (seed == nullptr || seed->isRemoved()) continue;
            if (facetCluster_.count(seed)) continue;
            facetCluster_[seed] = nextId;
            q.push(seed);
            const MCGAL::Vector3 seedN = normalize(seed->computeNormal());
            while (!q.empty()) {
                MCGAL::Facet* f = q.front();
                q.pop();
                // Visit face-neighbours via halfedges.
                MCGAL::Halfedge* st = f->proxyHalfedge();
                if (st == nullptr) continue;
                MCGAL::Halfedge* it = st;
                do {
                    MCGAL::Halfedge* opp = it->opposite();
                    if (opp != nullptr && opp->face() != nullptr && !opp->face()->isRemoved()) {
                        MCGAL::Facet* nb = opp->face();
                        if (facetCluster_.count(nb) == 0) {
                            const MCGAL::Vector3 nbN = normalize(nb->computeNormal());
                            const double dot = static_cast<double>(seedN | nbN);
                            if (dot >= threshold_) {
                                facetCluster_[nb] = nextId;
                                q.push(nb);
                            }
                        }
                    }
                    it = it->next();
                } while (it != st);
            }
            ++nextId;
        }

        // ---- Mark vertices that sit on cluster boundaries ----
        for (MCGAL::Vertex* v : mesh->vertices()) {
            if (v == nullptr || v->isRemoved()) continue;
            std::unordered_set<std::uint32_t> ids;
            for (MCGAL::Halfedge* h : v->halfedges()) {
                if (h == nullptr || h->face() == nullptr || h->face()->isRemoved()) continue;
                auto it = facetCluster_.find(h->face());
                if (it != facetCluster_.end()) ids.insert(it->second);
                if (ids.size() >= 2) break;
            }
            if (ids.size() >= 2) {
                boundary.insert(static_cast<std::uint32_t>(v->poolId()));
            }
        }
        return boundary;
    }

private:
    static MCGAL::Vector3 normalize(MCGAL::Vector3 n) {
        const float len = n.length();
        if (len > 1e-8f) n.normalize();
        return n;
    }

    double threshold_;
    std::unordered_map<MCGAL::Facet*, std::uint32_t> facetCluster_;
};

}  // namespace pmsf

#endif  // PMSF_FEATURE_CLUSTER_BOUNDARY_DETECTOR_H
