/*
 * @file CurvatureHysteresisDetector.h
 * @brief APTM 锚点检测 —— 从 impl/APTM/anchor_detection.h 迁移到框架层。
 *
 * 论文：Cheng et al. 2006, "Adaptive Progressive Mesh by Anchor Triangle Mapping"
 *
 * 算法（见原 anchor_detection.h 的注释）：
 *   1. 每顶点 k(v) = |k_max| + |k_min|（1-环邻居的有限差分主曲率和）。
 *   2. cf(v) = (k(v) - μ) / σ 归一化。
 *   3. 滞后阈值 BFS：|cf| > upperThreshold=1.7 作为种子，传播到
 *      |cf| > lowerThreshold=1.2 的邻居，得到连通特征区域。
 *   4. 每个特征区域内保留 |cf| 最大的顶点作为锚点。
 *
 * 框架化接口：返回 `FeatureSet`（unordered_set<poolId>），下游 IsRemovable /
 * Predictor 可 O(1) 查询某顶点是否是锚点。
 */
#ifndef PMSF_FEATURE_CURVATURE_HYSTERESIS_DETECTOR_H
#define PMSF_FEATURE_CURVATURE_HYSTERESIS_DETECTOR_H

#include "FeatureDetector.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Point.h"
#include "Vertex.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

namespace pmsf {

class CurvatureHysteresisDetector final : public FeatureDetector {
public:
    CurvatureHysteresisDetector(double upper_threshold = 1.7,
                                double lower_threshold = 1.2)
        : upper_(upper_threshold), lower_(lower_threshold) {}

    FeatureSet detect(std::shared_ptr<MCGAL::Mesh> mesh) override {
        FeatureSet out;
        if (!mesh) return out;

        std::vector<MCGAL::Vertex*> active;
        std::unordered_map<MCGAL::Vertex*, int> id;
        active.reserve(mesh->vertices().size());
        id.reserve(mesh->vertices().size() * 2);
        for (MCGAL::Vertex* v : mesh->vertices()) {
            if (v == nullptr || v->isRemoved()) continue;
            id[v] = static_cast<int>(active.size());
            active.push_back(v);
        }
        const std::size_t N = active.size();
        if (N == 0) return out;

        // 1. per-vertex k = |k_max| + |k_min|
        std::vector<double> k(N, 0.0);
        double sum = 0.0;
        for (std::size_t i = 0; i < N; ++i) {
            double kmax = 0.0, kmin = 0.0;
            principalCurvatures(active[i], kmax, kmin);
            k[i] = std::abs(kmax) + std::abs(kmin);
            sum += k[i];
        }
        const double mean = sum / static_cast<double>(N);
        double var = 0.0;
        for (double kv : k) { const double d = kv - mean; var += d * d; }
        double stddev = std::sqrt(var / static_cast<double>(N));
        if (stddev < 1e-12) stddev = 1.0;

        // 2. normalised cf
        std::vector<double> cf(N, 0.0);
        for (std::size_t i = 0; i < N; ++i) cf[i] = (k[i] - mean) / stddev;

        // 3. hysteresis BFS to grow feature regions
        std::vector<std::uint8_t> isFeature(N, 0);
        {
            std::queue<int> q;
            for (std::size_t i = 0; i < N; ++i) {
                if (std::abs(cf[i]) <= upper_ || isFeature[i]) continue;
                isFeature[i] = 1;
                q.push(static_cast<int>(i));
                while (!q.empty()) {
                    const int idx = q.front();
                    q.pop();
                    MCGAL::Vertex* v = active[idx];
                    for (auto vv = v->vertices_begin(); vv != v->vertices_end(); ++vv) {
                        MCGAL::Vertex* nb = *vv;
                        if (nb == nullptr || nb->isRemoved()) continue;
                        auto it = id.find(nb);
                        if (it == id.end()) continue;
                        const int nidx = it->second;
                        if (!isFeature[nidx] && std::abs(cf[nidx]) > lower_) {
                            isFeature[nidx] = 1;
                            q.push(nidx);
                        }
                    }
                }
            }
        }

        // 4. one anchor per connected feature region (largest |cf|)
        std::vector<std::uint8_t> visited(N, 0);
        std::vector<int> region;
        region.reserve(64);
        std::queue<int> bfs;
        for (std::size_t s = 0; s < N; ++s) {
            if (!isFeature[s] || visited[s]) continue;
            region.clear();
            bfs.push(static_cast<int>(s));
            visited[s] = 1;
            while (!bfs.empty()) {
                const int idx = bfs.front();
                bfs.pop();
                region.push_back(idx);
                MCGAL::Vertex* v = active[idx];
                for (auto vv = v->vertices_begin(); vv != v->vertices_end(); ++vv) {
                    MCGAL::Vertex* nb = *vv;
                    if (nb == nullptr || nb->isRemoved()) continue;
                    auto it = id.find(nb);
                    if (it == id.end()) continue;
                    const int nidx = it->second;
                    if (isFeature[nidx] && !visited[nidx]) {
                        visited[nidx] = 1;
                        bfs.push(nidx);
                    }
                }
            }
            int bestIdx = region[0];
            double bestK = std::abs(cf[region[0]]);
            for (int r : region) {
                if (std::abs(cf[r]) > bestK) { bestK = std::abs(cf[r]); bestIdx = r; }
            }
            out.insert(static_cast<std::uint32_t>(active[bestIdx]->poolId()));
        }
        return out;
    }

private:
    static MCGAL::Vector3 vertexNormal(MCGAL::Vertex* v) {
        MCGAL::Vector3 n(0, 0, 0);
        int count = 0;
        for (MCGAL::Halfedge* h : v->halfedges()) {
            if (h == nullptr || h->face() == nullptr || h->face()->isRemoved()) continue;
            n = n + h->face()->computeNormal();
            ++count;
        }
        if (count > 0) {
            n = n / count;
            const float len = n.length();
            if (len > 1e-8f) n.normalize();
        }
        return n;
    }

    static void principalCurvatures(MCGAL::Vertex* v, double& kmax, double& kmin) {
        const MCGAL::Point p = v->point();
        const MCGAL::Vector3 n = vertexNormal(v);
        double lo = 0.0, hi = 0.0;
        bool any = false;
        for (auto vv = v->vertices_begin(); vv != v->vertices_end(); ++vv) {
            MCGAL::Vertex* nb = *vv;
            if (nb == nullptr || nb->isRemoved()) continue;
            MCGAL::Point diff = nb->point() - p;
            const double dist = diff.length();
            if (dist <= 1e-8) continue;
            const double nd = diff.dot(n);
            const double kappa = 2.0 * nd / (dist * dist);
            if (!any) { lo = hi = kappa; any = true; }
            else { lo = std::min(lo, kappa); hi = std::max(hi, kappa); }
        }
        kmin = any ? lo : 0.0;
        kmax = any ? hi : 0.0;
    }

    double upper_;
    double lower_;
};

}  // namespace pmsf

#endif  // PMSF_FEATURE_CURVATURE_HYSTERESIS_DETECTOR_H
