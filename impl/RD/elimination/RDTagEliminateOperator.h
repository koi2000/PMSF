// SPDX-License-Identifier: Apache-2.0
//
// RDTagEliminateOperator.h — RD Tag (Phase 4 PR-B, BlobLayout=18, sv=1)
// encoder-side eliminate operator.
//
// 设计契约（与 RDTagReconstruct（= BasicReconstructOperator）对称）：
//   1. 几何完全继承 VertexRemovalEliminateOperator —— v0-fan，零变化。
//   2. 与 PCLTTMTagEliminateOperator 的差异：**不做 ⊕/⊖ tag evolution**。
//      RD 论文 (Ahn et al. 2011) 的 retriangulation rule 与 Alliez Fig.1 等价，
//      但 RD 的 connectivity 节省靠的是 prioritized gate ordering + valence
//      priors，不需要 PCLTTM 的 vertex tag 机制。
//   3. 仅做：remove_point → normalize_proxy_to_min_position_vertex(量化整数字典序)
//      → triangulate fan → postprocess。同 PCLTTMTagEliminateOperator 的步骤
//      1+2+5（去掉 step 4 tag evolution）。
//
// 对称性源头：
//   - encoder 端 polyVerts[0] = h->face()->proxyHalfedge()->vertex()；
//   - decoder 端 polyVerts[0] 同样由 face->proxyHalfedge() 决定，因双方
//     共享 ConnectivityBlobContainer 容器顺序 + base_mesh proxyHalfedge 量化
//     整数字典序 normalize → bit-exact 一致。
//
// 详见 docs/paper_alignment/PHASE4_PLAN.md §3.3 (PR-B)。

#pragma once

#include "elimination/VertexRemovalEliminateOperator.h"

class RDTagEliminateOperator : public VertexRemovalEliminateOperator {
  public:
    RDTagEliminateOperator() = default;
    ~RDTagEliminateOperator() = default;

    bool eliminate(HalfedgeErrorEntry& entry) override;
};
