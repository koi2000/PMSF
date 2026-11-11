// SPDX-License-Identifier: Apache-2.0
//
// APTMSimLossyDeserializer.h — P2-pre spike（WS-B3 / task-14）
//
// 目的：用**受控实验**证明 polar decode 失效的根因是「tag sv=3 的拓扑推断依赖
// 量化坐标（lossless-only 捷径）」，而不是 polar 数学/3-bit 精度。
//
// 做法（只影响 decode 进程，不动编码约定）：
//   1) 用**无损** tag-cartesian 产物（sv=3，decode 六项全 true）正常解码；
//   2) 在每轮 `exportMesh()` 之后，把重建网格的顶点位置整体加一个**确定性的
//      小整数偏移**（±delta 个量化单位，用 poolId 哈希选符号/轴向），模拟
//      「几何 coder 有损 ⇒ decoder 位置 ≠ encoder 位置」的最小事实；
//   3) 继续下一轮解码。父类 reader 在下一轮 `collectFromDecoded` 里对每个面调
//      `normalize_proxy_to_min_position_vertex(bboxMin_, f_quantStep_)` ——
//      若偏移改变了某面的字典序最小顶点，proxy/fan 推断就会与 encoder 分叉。
//
// 判据（预先登记）：
//   - 若 delta=1（1 个量化单位 = bbox/4096，远小于 polar 的典型误差）就使
//     counts/topology/connectivity 失效 ⇒ 根因锁定为坐标约定的 lossless-only 假设；
//   - 若 delta 很大仍不影响结构 ⇒ 根因不在 normalize，需要另找（本 spike 否证）。
//
// 同时输出「proxy 翻转率」统计：对当前轮的每个面，比较
//   argmin_lex(量化坐标) vs argmin_lex(量化坐标 + 偏移)
// 的环上顶点是否相同，给出 delta∈{1,2,4,8} 各自的翻转率 —— 这是「v0 只有
// 坐标相关来源」的直接量化，也是 P2-pre 设计里「v0 side-stream 代价」的下界输入。
//
// 环境变量：
//   PMSF_APTM_SPIKE_NOISE_DELTA=N   实际扰动幅度（量化单位，0/未设 = 不扰动）
//   PMSF_APTM_SPIKE_NOISE_STATS=1   只统计翻转率、不扰动（可与 DELTA 同时用）
//   PMSF_APTM_SPIKE_NOISE_ALL=1     扰动所有活顶点（默认）；0 = 只扰动本轮新增顶点
//
// 该 deserializer 只在 `PMSF_APTM_USE_APTM_TAG=1` 且上述 env 打开时接入
// （APTMAlgorithm::makeDecompressOptions），默认 decode 路径零影响。

#ifndef PMSF_APTM_DECOMPRESS_APTMSIM_LOSSY_DESERIALIZER_H
#define PMSF_APTM_DECOMPRESS_APTMSIM_LOSSY_DESERIALIZER_H

#include "decompress/ColumnarDeserializeOperator.h"

#include "Halfedge.h"
#include "Mesh.h"
#include "Point.h"
#include "Vertex.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>

class APTMSimLossyDeserializer final : public ColumnarDeserializeOperator {
  public:
    APTMSimLossyDeserializer(std::shared_ptr<SymbolReadOperator> reader,
                             std::shared_ptr<ReconstructOperator> reconstructor)
        : ColumnarDeserializeOperator(std::move(reader), std::move(reconstructor)) {}

    std::shared_ptr<MCGAL::Mesh> exportMesh() override {
        std::shared_ptr<MCGAL::Mesh> mesh =
            ColumnarDeserializeOperator::exportMesh();
        if (!mesh) return mesh;

        const int delta = envInt("PMSF_APTM_SPIKE_NOISE_DELTA");
        const bool statsOnly = envInt("PMSF_APTM_SPIKE_NOISE_STATS") != 0;
        const bool perturbAll = envInt("PMSF_APTM_SPIKE_NOISE_ALL", 0) != 0;
        if (delta <= 0 && !statsOnly) return mesh;

        // 1) 翻转率统计（在扰动前算；对每个面比较 clean vs offset 的 argmin）
        proxyFlipStats(mesh, delta > 0 ? delta : 1);

        // 2) 实际扰动（模拟几何有损）。只扰 "新增" 时按 baseVertexCount_ 之后的
        //    pool 索引近似：decoder 的顶点分配是单调 append 的（基网先分配）。
        if (delta > 0) {
            perturbMesh(mesh, delta, perturbAll);
        }
        return mesh;
    }

    /// `afterBaseMeshHook` 在解码轮循环之前调用：记录基网顶点数，
    /// 使「只扰动有损插入顶点、基网保持精确」成为默认语义（与真实码流一致）。
    void setBaseVertexCount(size_t n) {
        baseVertexCount_ = n;
        baseVertexCountCaptured_ = true;
    }
    bool baseVertexCountCaptured() const { return baseVertexCountCaptured_; }

  private:
    static int envInt(const char* name, int fallback = 0) {
        const char* v = std::getenv(name);
        if (!v || !*v) return fallback;
        return std::atoi(v);
    }

    /// poolId → 确定性三轴偏移（−delta..+delta 的整数）。
    static MCGAL::PointInt offsetFor(int poolId, int round, int delta) {
        std::uint32_t h = static_cast<std::uint32_t>(poolId) * 2654435761u;
        h ^= static_cast<std::uint32_t>(round) * 2246822519u;
        auto axis = [&](int shift) -> int {
            const std::uint32_t bits = (h >> shift) & 0x3u;
            if (bits == 0u) return 0;
            return (bits & 1u) ? delta : -delta;
        };
        return MCGAL::PointInt(axis(0), axis(2), axis(4));
    }

    static MCGAL::PointInt quantOf(MCGAL::Mesh* mesh, MCGAL::Vertex* v) {
        return mesh->floatPosToInt(v->point());
    }

    /// 计算 face 的 ring 上按 (x,y,z) 字典序最小的顶点（对应 MCGAL
    /// `normalize_proxy_to_min_position_vertex` 的排序键）。
    static MCGAL::Vertex* lexMinVertex(MCGAL::Mesh* mesh, MCGAL::Facet* f,
                                       int round, int delta) {
        MCGAL::Vertex* best = nullptr;
        MCGAL::PointInt bestQ;
        for (MCGAL::Vertex* v : f->getVertices()) {
            if (v == nullptr) continue;
            MCGAL::PointInt q = quantOf(mesh, v);
            const MCGAL::PointInt off = offsetFor(v->poolId(), round, delta);
            q = MCGAL::PointInt(q.x() + off.x(), q.y() + off.y(), q.z() + off.z());
            if (best == nullptr || q.x() < bestQ.x() ||
                (q.x() == bestQ.x() &&
                 (q.y() < bestQ.y() ||
                  (q.y() == bestQ.y() && q.z() < bestQ.z())))) {
                best = v;
                bestQ = q;
            }
        }
        return best;
    }

    static MCGAL::Vertex* lexMinClean(MCGAL::Mesh* mesh, MCGAL::Facet* f) {
        MCGAL::Vertex* best = nullptr;
        MCGAL::PointInt bestQ;
        for (MCGAL::Vertex* v : f->getVertices()) {
            if (v == nullptr) continue;
            const MCGAL::PointInt q = quantOf(mesh, v);
            if (best == nullptr || q.x() < bestQ.x() ||
                (q.x() == bestQ.x() &&
                 (q.y() < bestQ.y() ||
                  (q.y() == bestQ.y() && q.z() < bestQ.z())))) {
                best = v;
                bestQ = q;
            }
        }
        return best;
    }

    void proxyFlipStats(std::shared_ptr<MCGAL::Mesh> mesh, int delta) {
        long long faces = 0, sumRing = 0;
        long long flips[5] = {0, 0, 0, 0, 0};  // delta ∈ {1,2,4,8} 的翻转数
        const int deltas[5] = {1, 2, 4, 8, 0};
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved()) continue;
            MCGAL::Vertex* clean = lexMinClean(mesh.get(), f);
            if (clean == nullptr) continue;
            ++faces;
            sumRing += static_cast<long long>(f->getVertices().size());
            for (int i = 0; i < 4; ++i) {
                MCGAL::Vertex* noisy =
                    lexMinVertex(mesh.get(), f, roundIdx_, deltas[i]);
                if (noisy != clean) ++flips[i];
            }
        }
        totalFaces_ += faces;
        totalRing_ += sumRing;
        for (int i = 0; i < 4; ++i) totalFlips_[i] += flips[i];
        ++roundIdx_;
        std::fprintf(stderr,
                     "[APTM-SPIKE] round=%d faces=%lld ringSum=%lld "
                     "flips(d=1,2,4,8)=%lld,%lld,%lld,%lld\n",
                     roundIdx_ - 1, faces, sumRing, flips[0], flips[1],
                     flips[2], flips[3]);
        if (totalFaces_ > 0) {
            std::fprintf(stderr,
                         "[APTM-SPIKE][cum] faces=%lld ringSum=%lld "
                         "flipRate(d=1,2,4,8)=%.3f%%,%.3f%%,%.3f%%,%.3f%% "
                         "avgRing=%.2f\n",
                         totalFaces_, totalRing_,
                         100.0 * totalFlips_[0] / totalFaces_,
                         100.0 * totalFlips_[1] / totalFaces_,
                         100.0 * totalFlips_[2] / totalFaces_,
                         100.0 * totalFlips_[3] / totalFaces_,
                         static_cast<double>(totalRing_) /
                             static_cast<double>(totalFaces_));
        }
        std::fflush(stderr);
    }

    void perturbMesh(std::shared_ptr<MCGAL::Mesh> mesh, int delta,
                     bool perturbAll) {
        int perturbed = 0;
        const size_t vcount = mesh->size_of_vertices();
        for (size_t i = 0; i < vcount; ++i) {
            MCGAL::Vertex* v = mesh->get_vertex(static_cast<int>(i));
            if (v == nullptr || v->isRemoved()) continue;
            if (!perturbAll &&
                static_cast<long long>(i) <
                    static_cast<long long>(baseVertexCount_)) {
                continue;
            }
            const MCGAL::PointInt q = quantOf(mesh.get(), v);
            const MCGAL::PointInt off = offsetFor(v->poolId(), roundIdx_ - 1, delta);
            const MCGAL::Point p =
                mesh->intPosToFloat(MCGAL::PointInt(q.x() + off.x(),
                                                    q.y() + off.y(),
                                                    q.z() + off.z()));
            v->setPoint(p);
            ++perturbed;
        }
        std::fprintf(stderr,
                     "[APTM-SPIKE] round=%d perturbed_vertices=%d delta=%d "
                     "all=%d\n",
                     roundIdx_ - 1, perturbed, delta, perturbAll ? 1 : 0);
        std::fflush(stderr);
    }

    int roundIdx_ = 0;
    bool baseVertexCountCaptured_ = false;
    size_t baseVertexCount_ = 0;
    long long totalFaces_ = 0;
    long long totalRing_ = 0;
    long long totalFlips_[4] = {0, 0, 0, 0};
};

#endif  // PMSF_APTM_DECOMPRESS_APTMSIM_LOSSY_DESERIALIZER_H
