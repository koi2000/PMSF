// SPDX-License-Identifier: Apache-2.0
//
// RDPatchDecoderConquest.h —— PR-M4.b (2026-05-12)：RD Patch Conquest decoder
// 端的 conquest 主循环驱动。
//
// 设计契约：mirror encoder src/core/RDPatchPipeline.cpp 的 runConquest 逻辑：
//   - 同样 6-bucket priority queue
//   - 同样 DEC/CLN phase 切换（queue 空即切；cycle 无 progress 则退出）
//   - 同样 face-level dedup（per-phase）
//   - 同样 1-ring push 顺序（snapshotRing + pushNewGates）
//   - 同样 NULL_PATCH 邻居 push 策略
//
// 关键差异（与 encoder runConquest）：
//   - 不调 IsRemovable 检查；不调 RDTagEliminate
//   - 每次 pop 一个 face → consume next sym
//     - sym = NULL_PATCH (DEC): 不动 mesh，标 face dedup，push 邻居（mirror encoder）
//     - sym = V_v (DEC): mesh->create_center_vertex_without_init(face->proxyHalfedge(),
//       face_barycenter) 加 1 个 vertex（fan triangulate；新 vertex 几何用 face
//       barycenter 占位，因 PR-M2 stream 1 residual 未实装），push 1-ring 邻居
//     - sym = V_3 (CLN): 同 DEC V_3 路径
//
// **PR-M4.b 已知不对称问题**（详见 docs/paper_alignment/PR_M_RD_PATCH_DESIGN.md
// §M4 后续工作）：
//   - encoder forward 在 finest mesh 上选 first_gate (longest_edge)；
//   - decoder 在 base mesh 上选 first_gate (longest_edge)；
//   两条 first_gate 不一定是同一条 halfedge，priority queue 起点不同 →
//   sym 序列消费偏移。表现为 round-trip 后 V 数量 != V_in。
//
//   修复方案（M5 之前）：要么 encoder 写 first_gate metadata（量化坐标对），
//   要么 encoder 改成 backward-emit（base→finest 反向重新 emit symbols），
//   论文做法是后者（详见设计文档）。本 session PR-M4.b 接受这个问题，
//   先把 decoder 端主循环 + 加点机制实装跑通。
//
// 几何精度 caveat（PR-M2 connectivity-only）：
//   stream 1 residual 未实装 → 加点几何用 face barycenter 占位，与原始
//   vertex 几何位置可能差很多。M5 之前需要补 stream 1 residual。
//
// 详见 docs/paper_alignment/PR_M_RD_PATCH_DESIGN.md。

#ifndef PMSF_DECOMPRESS_RD_PATCH_DECODER_CONQUEST_H
#define PMSF_DECOMPRESS_RD_PATCH_DECODER_CONQUEST_H

#include <array>
#include <memory>
#include <vector>

#include "common/patch/RDPatchPipeline.h"  // RDPatchSymbol

namespace MCGAL {
class Mesh;
}

namespace pmsf {

/// 反演结果（diag 用）。
struct RDPatchDecoderResult {
    int  numVerticesIn        = 0;   // base mesh V 数（反演前）
    int  numVerticesOut       = 0;   // 反演后 V 数（理想 = encoder.V_in）
    int  numAddedVertices     = 0;   // 真加了多少 vertex
    int  numNullPopsConsumed  = 0;   // 消费多少 NULL_PATCH sym
    int  numVValPopsConsumed  = 0;   // 消费多少 V_v sym
    int  numSymbolsTotal      = 0;   // 消费总 sym 数
    int  numSymsAvailable     = 0;   // decodedSymbols.size()
    int  numDecCycles         = 0;
    int  numSteps             = 0;
    bool reachedMaxSteps      = false;
    bool exhaustedSymbols     = false;  // 主循环未 break 时已读完所有 sym
    long long bucketHits[6]   = {0, 0, 0, 0, 0, 0};
};

/// 跑 decoder conquest 反演。返回结果（不抛异常，diag 在 result 里）。
///
/// @param mesh 已重建的 base mesh（V_out=encoder base mesh）
/// @param decodedSymbols  encoder emit 的 symbol 序列
/// @param resX/Y/Z   每个 V_v sym 对应的 residual int32 三元组（按 V_v emit 顺序）。
///                   若为空（sv=1），则 decoder 加点用 face barycenter 占位（M4.b 行为）。
///                   若非空（sv=2），decoder 加点用：
///                     vertex_quant = face_bary_quant + residual
///                     vertex_float = mesh->intPosToFloat(vertex_quant)
RDPatchDecoderResult runRDPatchDecoderConquest(
    std::shared_ptr<MCGAL::Mesh> mesh,
    const std::vector<RDPatchSymbol>& decodedSymbols,
    const std::vector<int>& resX = {},
    const std::vector<int>& resY = {},
    const std::vector<int>& resZ = {},
    const std::vector<int>& faceRanks = {},
    const std::vector<unsigned char>& ringSizes = {},
    const std::vector<int>& ringQX = {},
    const std::vector<int>& ringQY = {},
    const std::vector<int>& ringQZ = {},
    const std::vector<int>& absQX = {},
    const std::vector<int>& absQY = {},
    const std::vector<int>& absQZ = {},
    const std::vector<int>& firstGateQ = {},
    const std::vector<unsigned char>& inputOffsets = {},
    const std::vector<std::array<int, 6>>& symbolInputQs = {},
    bool forcePMSFBoundaryGates = false,
    bool forceStrictRDConquest = false,
    const std::vector<std::array<int, 9>>& faceQ = {},
    const std::vector<unsigned char>& faceRingSizes = {},
    const std::vector<int>& faceRingQX = {},
    const std::vector<int>& faceRingQY = {},
    const std::vector<int>& faceRingQZ = {},
    const std::vector<unsigned char>& faceSide = {});

RDPatchDecoderResult runDirectRDPatchReplay(
    std::shared_ptr<MCGAL::Mesh> mesh,
    const std::vector<RDPatchSymbol>& decodedSymbols,
    const std::vector<int>& resX,
    const std::vector<int>& resY,
    const std::vector<int>& resZ,
    const std::vector<int>& faceRanks,
    const std::vector<unsigned char>& ringSizes,
    const std::vector<int>& ringQX,
    const std::vector<int>& ringQY,
    const std::vector<int>& ringQZ,
    const std::vector<int>& absQX,
    const std::vector<int>& absQY,
    const std::vector<int>& absQZ);

RDPatchDecoderResult runFaceIdRDPatchReplay(
    std::shared_ptr<MCGAL::Mesh> mesh,
    const std::vector<RDPatchSymbol>& decodedSymbols,
    const std::vector<int>& resX,
    const std::vector<int>& resY,
    const std::vector<int>& resZ,
    const std::vector<int>& faceIds);

RDPatchDecoderResult runFaceRankRDPatchReplay(
    std::shared_ptr<MCGAL::Mesh> mesh,
    const std::vector<RDPatchSymbol>& decodedSymbols,
    const std::vector<int>& resX,
    const std::vector<int>& resY,
    const std::vector<int>& resZ,
    const std::vector<int>& faceRanks);

RDPatchDecoderResult runPointRDPatchReplay(
    std::shared_ptr<MCGAL::Mesh> mesh,
    const std::vector<RDPatchSymbol>& decodedSymbols,
    const std::vector<int>& resX,
    const std::vector<int>& resY,
    const std::vector<int>& resZ,
    const std::vector<int>& absQX,
    const std::vector<int>& absQY,
    const std::vector<int>& absQZ);

/// Encoder-side helper for RD_STRICT: starting from a copy of the final base
/// mesh, replay the same strict decoder conquest and re-emit the internally
/// recorded removals in refinement order. Ring/input records are consumed only
/// as encoder-side anchors and are not part of the serialized RD_STRICT layout.
RDPatchPipelineResult rebuildRDPatchResultInStrictDecodeOrder(
    std::shared_ptr<MCGAL::Mesh> baseMesh,
    const RDPatchPipelineResult& forwardResult);

/// Repeatedly run strict decode-order rebuild on the still-unmatched forward
/// removals. Each successful pass starts from the mesh state reached by the
/// previous pass, so the returned stream can be decoded as deterministic
/// subrounds from the stored base mesh back to the original mesh.
RDPatchPipelineResult rebuildRDPatchResultInSplitStrictDecodeOrder(
    std::shared_ptr<MCGAL::Mesh> baseMesh,
    const RDPatchPipelineResult& forwardResult);

/// Encoder-side helper for topology-safe APTM/PCLTTM: starting from the final
/// base mesh, replay V insertions in exact reverse removal order using the
/// encoder-captured boundary rings, but emit only a face-rank patch locator.
RDPatchPipelineResult rebuildRDPatchResultInReverseRingOrder(
    std::shared_ptr<MCGAL::Mesh> baseMesh,
    const RDPatchPipelineResult& forwardResult);

}  // namespace pmsf

#endif  // PMSF_DECOMPRESS_RD_PATCH_DECODER_CONQUEST_H
