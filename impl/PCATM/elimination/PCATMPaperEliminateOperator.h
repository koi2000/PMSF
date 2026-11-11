// SPDX-License-Identifier: Apache-2.0
//
// PCATMPaperEliminateOperator.h — PCATM 论文严格对齐 (BlobLayout=20, sv=1)
// encoder-side eliminate operator。
//
// 设计契约（M1-b Step 2a，2026-05-11）：
//   1. 几何 = 论文 §4 + Figure 2 (1)(2)(3) 真 Z-shape (zigzag) 三角化，
//      不再是 PCATMEliminate / PCATMZColor 的 v0-fan。
//      具体 zigzag 选取由 impl/PCATM/PCATMZShapeGeometry.h::computeZShape 决定
//      （N=3..12 ground-truth 已 PR-1 单元测试通过：tests/test_pcatm_zshape_geometry.cpp）。
//
//   2. 着色 = patch 内 alternating RED/BLUE，与 computeZShape 输出的 Triangle::color
//      字段严格对齐，论文 §4 invariant：
//        - patch 内：相邻 triangle 颜色不同 (alternating)
//        - 跨 patch：BLUE 三角形不与任何邻 patch 的 BLUE 共享 edge
//      跨 patch invariant 由 src/common/AlternatingInvariantChecker 校验
//      （unit test: tests/test_alternating_invariant.cpp）。
//
//   3. remove_point 与 PCATMZColorEliminate 完全一致：
//        - rdBarycenterQuantized（量化整数累加平均，避免浮点累积偏差）
//        - normalize_proxy_to_min_position_vertex（量化整数字典序）
//        - setProtocolHint(20) (CODEC_020)
//      decoder 端可对每个 polyFace 用同一 normalize 拿到完全一致的 polyVerts 顺序，
//      bit-exact 复用 computeZShape 反演对角线集合（M1-b Step 2c，下轮）。
//
// 与 PCATMZColorEliminateOperator 的差异（最小 diff，仅几何不同）：
//   - 重写 triangulate(h)：用 computeZShape 拿对角线索引序列 → 调
//     Mesh::split_facet(h_a, h_b) × (N-3) 真实切 polygon。
//   - 不再走父类 fan triangulation。
//   - 着色严格按 computeZShape 输出的 Triangle::color 贴每个 face。
//
// 9 算法 baseline（含 PCATMColor sv=3 / PCATMZColor sv=1）零回归：
//   旧路径不实例化本类。本类仅在
//     - tools/DefaultPCATMPaperColumnarCompress.cpp
//   中显式装配。
//
// 详见 docs/paper_alignment/pcatm_zshape_groundtruth.md。

#pragma once

#include "PCATMZColorEliminateOperator.h"

#include <vector>

namespace MCGAL {
class Halfedge;
class Facet;
class Vertex;
}  // namespace MCGAL

class PCATMPaperEliminateOperator : public PCATMZColorEliminateOperator {
  public:
    PCATMPaperEliminateOperator() = default;
    ~PCATMPaperEliminateOperator() = default;

    bool eliminate(HalfedgeErrorEntry& entry) override;

    /// @brief 重写 triangulate —— 用 computeZShape 输出真 Z-shape 几何，
    ///        对每条新对角线调 Mesh::split_facet 实现拓扑切分。
    /// @return 与基类一致：成功 true。
    bool triangulate(MCGAL::Halfedge* h) override;

  private:
    /// @brief 在已 normalize 的 polyFace 上枚举 ring，返回 polyVerts 顺序。
    ///        Index 0 = polyFace->proxyHalfedge()->vertex()。
    ///        长度 = polyFace->getPolyValence()（remove_point 已写入）。
    static std::vector<MCGAL::Vertex*> collectPolygonRing(MCGAL::Facet* polyFace);

    /// @brief 在 polyVerts ring 上找出从 polyVerts[va] 出发、终于 polyVerts[(va+1)%N]
    ///        的那条 halfedge（即 polygon edge）。这是 split_facet 调用所需的"h_a"
    ///        语义：split_facet(h_a, h_b) 创建对角线 h_a.end_vertex → h_b.end_vertex。
    /// @return nullptr 表示找不到（拓扑异常，CHECK_TRUE 会失败）。
    static MCGAL::Halfedge* findOutgoingHalfedge(MCGAL::Facet* polyFace,
                                                 MCGAL::Vertex* fromVertex);

    /// @brief 把 4-color enum 设置到 face：computeZShape 的 color ∈ {0, 1}
    ///        对应 MCGAL::Facet::TriangleColorFlag::RED (=0) / BLUE (=1)。
    static void applyColor(MCGAL::Facet* face, int colorBit);
};
