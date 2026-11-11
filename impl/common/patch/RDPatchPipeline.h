// SPDX-License-Identifier: Apache-2.0
//
// RDPatchPipeline.h —— PR-M（2026-05-12）：RD 论文 §2 patch-level conquest
// 主循环驱动。**不复用** CompressPipeline 多轮主循环——论文 conquest 是单一
// gate-queue 序列，没有 round 概念。
//
// 与 CompressPipeline::run 的根本不同：
//   - CompressPipeline:  for round in 0..R: { selector.select_loop → eliminate → collector.collect }
//                        每轮一个 BFS 遍历整个 mesh，emit per-face symbol。
//   - RDPatchPipeline:    while !gate_queue.empty(): { pop highest priority gate
//                                                    → DEC/CLN process →
//                                                    push new boundary edges }
//                        单一 conquest 序列，emit per-pop 1 symbol。
//
// PR-M1（本 session）实装内容：
//   1. encoder 端 conquest 主循环跑通 base mesh（V_in 34834 → V_base ≤ ~50）
//   2. emit 内存里的 symbol 序列（std::vector<RDPatchSymbol>），不写 parquet
//   3. 打印诊断：num_decimated_vertices / num_null_patches / num_dec_cycles /
//      bucket usage histogram / V_in / V_out
//   4. 不死锁、不 infinite loop（max_steps 4×V 兜底）
//
// PR-M1 简化（详见 docs/paper_alignment/PR_M_RD_PATCH_DESIGN.md §5）：
//   - vertex tag 全保留 Untagged（不做 ⊕/⊖ evolution）→ 全部进 bucket 5
//   - patch null-flag 全保留 NonNull（不做 null 标记演化）
//   - first_gate 用最长边（与 RD baseline customSeedSelector 一致）
//   - is_removable 用 RDTagEliminate 间接复用 BasicIsRemovable 链
//   - 新边界 push 简化：被删 vertex 1-ring 的"对方端 vertex 引出的"halfedges 的 opposite
//
// 6-bucket 完整版 + tag/null evolution 留给 M2 落地（M2 配合 SymbolCollect 重写）。
//
// 详见 docs/paper_alignment/PR_M_RD_PATCH_DESIGN.md §2-§5。

#ifndef PMSF_CORE_RD_PATCH_PIPELINE_H
#define PMSF_CORE_RD_PATCH_PIPELINE_H

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace MCGAL {
class Mesh;
class Halfedge;
class Vertex;
}  // namespace MCGAL

namespace pmsf {

class RDPatchGateQueue;

/// PR-M conquest emit 出来的 symbol 类型（in-memory，M2 会接 SymbolCollect 串联）。
/// 字母表（与 BlobLayout=24 RD_PATCH 协议规格一致）：
///   NULL_PATCH=0, V_3=1, V_4=2, V_5=3, V_6=4, V_OTHER=5
/// V_OTHER 后跟 1 byte uint8 真实 valence（PR-M1 阶段不发 escape；M2 接入）。
enum class RDPatchSymbolKind : unsigned char {
    NULL_PATCH = 0,
    V_3        = 1,
    V_4        = 2,
    V_5        = 3,
    V_6        = 4,
    V_OTHER    = 5,
};

struct RDPatchSymbol {
    RDPatchSymbolKind kind;
    /// 当 kind==V_OTHER 时存真实 valence；其他情况此字段无意义。
    unsigned char     escape_valence;
    /// 该 symbol 由哪一个 phase（DEC=0 / CLN=1）emit。仅 PR-M1 诊断用。
    unsigned char     phase;
    /// 该 symbol pop 时来自哪个 bucket（1..6）；nullptr 时为 -1。
    int               bucket;
    /// A_i/B_i layer index, starting at 0 for the first DEC/CLN pair.
    unsigned short    layer;
};

/// RDPatchPipeline 配置（PR-M1 阶段最小集；M2 起会扩 collector/serializer 槽）。
struct RDPatchPipelineConfig {
    /// 每次 erase_center_vertex 之前是否调用 RDTagEliminate 的 normalize+triangulate。
    /// 默认 true：复用 RDTag 的 polygon retriangulation 路径，让 mesh 保持三角化态。
    bool useRDTagEliminate = true;

    /// 主循环兜底 max steps；超过即 abort。默认 50×V（足够多 cycle 跑完）。
    int  maxStepsMultiplier = 50;

    /// 每多少 step 打印一次进度（PMSF_RDPATCH_VERBOSE=1 时生效）。0=不打印。
    int  progressLogEvery = 5000;

    /// PR-O (2026-05-12)：单 conquest 内最多跑几个 DEC+CLN cycle。默认 1。
    /// 实测 (bunny multi-round V_in=34834, target V_base=50)：
    ///   maxCycles=1: rounds=52, V_base=456, ProtoBPV=26.60, BaseBPV=1.17, TotalBPV=27.77
    ///   maxCycles=2: rounds=78, V_base=235, ProtoBPV=27.49, BaseBPV=0.56, TotalBPV=28.06
    ///   maxCycles=3 (PR-N): rounds=63, V_base=194, ProtoBPV=27.46, BaseBPV=0.47, TotalBPV=27.93
    /// maxCycles=1 略优（TotalBPV -0.6% vs PR-N），主要因为多余 cycle 几乎全是
    /// NULL_PATCH（cycle 2/3 重走 mesh 确认无新可删，浪费 sym）。
    /// 多轮压缩通过 RDPatchMultiRoundBench tool 的"外层" round 循环实现。
    /// decoder 必须用相同 maxCycles。
    int  maxCycles = 1;

    /// Optional cap on removals in one conquest run (0 = unlimited). Used by
    /// RDStrict to split large layers into smaller, independently replayable
    /// sublayers.
    int  maxRemovalsPerRun = 0;

    /// PCLTTM strict storage needs the Alliez/Desbrun conquest semantics, not
    /// the RD patch exploration shortcuts used by the earlier BPV experiments.
    /// When enabled:
    ///   - DEC/CLN both emit NULL_PATCH for rejected active gates.
    bool strictPCLTTMConquest = false;

    /// RD paper-aligned conquest. This is separate from PCLTTM because RD uses
    /// the paper's prioritized gate selection and patch alphabet, while PCLTTM
    /// keeps its own storage path. When enabled:
    ///   - single first-gate traversal, no all-face seeding
    ///   - boundary-gate propagation from Figure 10
    ///   - real 6-bucket PGS, independent of environment variables
    ///   - DEC removes only valence 4..6, CLN removes valence 3
    ///   - DEC/CLN both emit NULL_PATCH
    bool strictRDConquest = false;

    /// Experimental Figure 10(a,b) boundary-gate push. It is intentionally
    /// separate from strictPCLTTMConquest because the current MCGAL halfedge
    /// gate orientation still needs decoder-walk calibration. The default
    /// strict path keeps the robust existing gate expansion while using the
    /// paper storage alphabet and layer semantics.
    bool usePMSFBoundaryGates = false;

    /// Emit residuals in the approximate Frenet frame of the active patch.
    /// This is only an encoder-side storage choice; callers must use the same
    /// mode if/when strict decoding is enabled.
    bool useFrenetResiduals = false;

    /// Strict RD internal retry hook: when strictUseAllowedRemovalFilter is
    /// true, only vertices whose quantized coordinates are present here may be
    /// removed. An empty list then means "remove nothing"; with the flag false
    /// the list is ignored. This is not serialized.
    bool strictUseAllowedRemovalFilter = false;
    std::vector<std::array<int, 3>> strictAllowedRemovalQs;
};

/// PR-M1 主入口：从 .off 文件加载 mesh，跑 conquest，返回简化后 mesh + symbol 序列。
struct RDPatchPipelineResult {
    std::shared_ptr<MCGAL::Mesh> mesh;          // conquest 后的 base mesh
    std::vector<RDPatchSymbol>   symbols;       // 单序列 emit 出来的 symbol
    int   numVerticesIn       = 0;
    int   numVerticesOut      = 0;              // base mesh 顶点数
    int   numDecimatedTotal   = 0;              // 共删了多少 vertex
    int   numNullPatches      = 0;              // 共 emit NULL_PATCH 数
    int   numDecCycles        = 0;              // DEC+CLN 完成多少轮
    int   numSteps            = 0;              // pop 总次数
    bool  reachedMaxSteps     = false;          // 是否撞 max_steps 兜底
    /// bucket usage histogram（pop 次数）。
    long long bucketHits[6]   = {0, 0, 0, 0, 0, 0};
    /// first_gate 量化坐标（M2 起写 metadata sideinfo）。
    int firstGateV0Q[3]       = {0, 0, 0};
    int firstGateV1Q[3]       = {0, 0, 0};
    /// Strict PCLTTM storage side-info: input-boundary seed edge per A_i/B_i
    /// layer, stored as q0.xyz,q1.xyz in layer order.
    std::vector<std::array<int, 6>> layerSeedQs;
    /// Strict decoder synchronization side-info: input-boundary edge per
    /// emitted symbol, stored in symbol order.
    std::vector<std::array<int, 6>> symbolInputQs;

    /// PR-M5 (sv=2)：每次 emit V_v sym 时同时记录的"被删 vertex 量化坐标 -
    /// face 量化 barycenter"残差 (ZigZag 之前的 int32 三元组，按 V_v emit 顺序)。
    /// 顺序与 symbols 中的 V_v sym 一一对应（NULL_PATCH 不占 entry）。
    /// decoder 读 stream 1 拿到同长 vector，加点时 residual + face_bary_quant →
    /// 真 vertex 量化坐标 → intPosToFloat → 真 vertex 浮点坐标。
    /// **caveat**：如 first_gate 不对称导致两端 V_v 序列对应的 face 不同，
    /// residual 仍按顺序消费但几何不严格 round-trip。M5 阶段先做 BPV 测算。
    std::vector<int> vvResidualsX;
    std::vector<int> vvResidualsY;
    std::vector<int> vvResidualsZ;
    /// Absolute quantized coordinates of each removed vertex, aligned with the
    /// V-code residual streams above. Diagnostic strict storage can use these
    /// to decouple geometry from an experimental decoder walk.
    std::vector<int> vvAbsQX;
    std::vector<int> vvAbsQY;
    std::vector<int> vvAbsQZ;
    /// Optional strict-replay face locator: up to 3 gate-face vertices' absQ.
    std::vector<std::array<int, 9>> vvFaceQ;
    /// Full gate-face ring locator (paper/fidelity path).
    std::vector<unsigned char> vvFaceRingSizes;
    std::vector<unsigned char> vvFaceSide;  // 0/1 side of symbolInputQ edge
    std::vector<int> vvFaceRingQX;
    std::vector<int> vvFaceRingQY;
    std::vector<int> vvFaceRingQZ;
    /// Experimental reverse-decode side-info: for each V-symbol, the rank of
    /// the face left by encoder after removal/retriangulation among living
    /// faces at that moment. This is serialized separately and consumed in
    /// reverse order by the decoder.
    std::vector<int> vvDecodeFaceRanks;
    /// Boundary ring quantized coordinates for each emitted V-symbol. These are
    /// encoder-side reconstruction anchors used by the optional backward
    /// emitter to locate the exact triangulated patch in the round base mesh.
    std::vector<unsigned char> vvRingSizes;
    /// Low 3 bits: active input edge offset in the normalized ring. Bit 3:
    /// the edge is reversed relative to the active gate input direction.
    std::vector<unsigned char> vvInputOffsets;
    std::vector<int> vvFaceRank;  // living-face rank used by direct replay
    std::vector<int> vvRingQX;
    std::vector<int> vvRingQY;
    std::vector<int> vvRingQZ;
    /// Original mesh vertex ids aligned with V-symbols and ring entries. They
    /// are not a final compact representation; the strict storage layer can
    /// convert them into decoder reconstruction ids for deterministic replay.
    std::vector<int> vvRemovedVertexIds;
    std::vector<int> vvRingVertexIds;
    /// Hybrid APTM locator: number of V records that can be reconstructed from
    /// face-rank locators before falling back to explicit ring coordinates.
    /// Negative means "not a hybrid stream".
    int locatorRankPrefix = -1;
    /// Sparse APTM locator: number of V records decoded by the ordinary
    /// conquest walk before falling back to explicit ring coordinates for the
    /// remaining tail records. Negative means "not a sparse-ring stream".
    int locatorRingPrefix = -1;
    unsigned char leakyFactors = 0x15;
};

/// 加载 + 跑 conquest 的便捷入口。把 .off 路径喂进来，返回 result。
/// PR-M1 阶段不写 parquet，纯做主循环验收。内部调 PMSF/contextPool 初始化 +
/// load mesh + runRDPatchEncoderOnMesh。
RDPatchPipelineResult runRDPatchEncoderPR_M1(const std::string& inputPath,
                                             const RDPatchPipelineConfig& cfg = {});

/// PR-M2 起的 mesh-in entry：跑 conquest 主循环（DEC/CLN cycle、6-bucket queue
/// pop loop、symbol 序列产出），假设 mesh 已加载、PMSF/contextPool 已 init。
/// 供 APTM strict conquest 等活路径（PCLTTMStrictSymbolCollectOperator）直接驱动。
RDPatchPipelineResult runRDPatchEncoderOnMesh(std::shared_ptr<MCGAL::Mesh> mesh,
                                              const RDPatchPipelineConfig& cfg = {});

}  // namespace pmsf

#endif  // PMSF_CORE_RD_PATCH_PIPELINE_H
