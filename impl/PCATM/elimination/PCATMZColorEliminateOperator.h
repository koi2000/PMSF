// SPDX-License-Identifier: Apache-2.0
//
// PCATMZColorEliminateOperator.h — PCATM Z-Color (Phase 4 PR-C, BlobLayout=17,
// sv=1) encoder-side eliminate operator.
//
// 设计契约（与 PCATMZColorReconstruct = BasicReconstructOperator 对称）：
//   1. 几何继承 PCATMEliminateOperator —— fan-triangulate + alternating RED/BLUE
//      贴色（见 triangulate_{degree_5,degree_6,general}）。这正是 PCATM 论文 §4
//      要求的"每 triangle 1 bit 2-coloring"的编码端源头。
//   2. **在 PCATMEliminateOperator::triangulate 之前** 调
//      `polyFace->normalize_proxy_to_min_position_vertex(bboxMin, f_quantStep)`，
//      让 proxyHalfedge.vertex 对齐 vertex **量化整数坐标**字典序最小 —— 这是
//      decoder 端 v0-fan 拓扑反演（PCLTTM Tag sv=3 同源）能正确工作的必要前提。
//      PCATMEliminateOperator 的 triangulate(h) 内部用 polyFace->proxyHalfedge()
//      生成 polyVerts / polyHalfedges，经本步 normalize 后其顺序与 decoder 端
//      lazy normalize 后的顺序一致 —— bit-exact BFS 对称。
//   3. [无 ⊕/⊖ tag evolution] —— PCATM 协议不依赖 PCLTTM Vertex::TagFlag。
//
// 与 PCATMEliminateOperator 的差异（最小 diff）：
//   - 仅在 remove_point 之后、triangulate 之前多调一次 normalize_proxy_to_min_position_vertex。
//   - 9 算法 baseline（含 PCATMColor sv=3 走 PCATMEliminateOperator）零影响，因为旧路径
//     不实例化本类。
//
// 详见 docs/paper_alignment/PHASE4_PLAN.md §3.4 (PR-C)。

#pragma once

#include "PCATMEliminateOperator.h"

class PCATMZColorEliminateOperator : public PCATMEliminateOperator {
  public:
    PCATMZColorEliminateOperator() = default;
    ~PCATMZColorEliminateOperator() = default;

    bool eliminate(HalfedgeErrorEntry& entry) override;

    /// @brief 重写 remove_point —— 用 `rdBarycenterQuantized`（量化整数累加平均）
    ///        替代父类 PCATMEliminateOperator 的"浮点平均后量化"。
    ///        这是 decoder 端 `BasicReconstructOperator::insert_point` 反预测使用
    ///        的同一函数，保证编/解码端 bit-exact 对齐 —— 否则会有 ±1 量化单位
    ///        累积偏差，多轮后导致几何崩坏（observed: round 12 bad_alloc）。
    bool remove_point(MCGAL::Halfedge* h) override;
};
