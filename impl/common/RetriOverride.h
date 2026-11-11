// SPDX-License-Identifier: Apache-2.0
//
// RetriOverride.h —— Phase 5 PR-E：baseline 算法的"决定性三角化协议"运行时切换钩子。
//
// 总览：
//   9 个 baseline `impl/XXX/XXX{Compress,Decompress}.cpp` 通过检测环境变量
//   （PMSF_RD_USE_RETRI_TAG=1 / PMSF_PCLTTM_USE_RETRI_TAG=1 /
//    PMSF_PCATM_USE_RETRI_COLOR=1 / PMSF_PCATM_USE_RETRI_ZCOLOR=1 /
//    PMSF_APTM_USE_RETRI_TAG=1 / PMSF_MMSE_USE_RETRI_TAG=1 /
//    PMSF_QEM_USE_RETRI_TAG=1），决定是走自家 baseline 路径（md5 字节级零回归）
//   还是走 Phase 4 已落地的三种 Tag/ZColor 协议路径之一。
//
//   当开关 OFF（默认）时：本 header 的两个顶层函数返回 false，baseline 主程序
//   按原逻辑继续走 `CompressPipeline::run(...) / DecompressPipeline::Builder().run()`，
//   产出的 `out_symbols.parquet` 与 Phase 4 baseline md5 完全一致。
//
//   当开关 ON 时：顶层函数接管整个 run，直接调用与
//   `tools/Default<Proto>Columnar{Compress,Decompress}.cpp` 一致的装配路径，
//   返回 true → baseline main 看到 true 后立刻 `return 0`，**跳过**原 baseline 调度。
//
// 设计约束（PHASE5_PLAN.md §3.2）：
//   1. baseline md5 零回归：env 未设时，本 header 的函数"零副作用"。
//   2. encoder/decoder 必须配对：若 compress 用了 env，decompress 也必须用相同
//      env，否则 parquet 的 BlobLayout 与 reader 不匹配 → 会在 deserialize 时
//      fail-fast。
//   3. 不改 CMake：所有新增 Tag/ZColor operator 的 .cpp 已被 ALL_SOURCES 全量
//      链入每个 baseline target，无需额外 add_sources。
//
// 与 Phase 4 tools 入口的差异：
//   - tools/Default<Proto>ColumnarCompress.cpp 的 SelectOptions / eliminate 是
//     与那个协议严格对齐的（例如 PCATMZColor 用 DegreeSixIsRemovable +
//     PCATMZColorEliminate）。
//   - 本文件的覆盖路径**保留 baseline 自己的 Select/Eliminate**（通过传入的
//     `CompressOptions& options` 获取），**只替换 collector + protocol
//     stamping**。这样语义：RD baseline 的 tag-priority cost + VertexRemoval
//     + DegreeSix/Three + RD_TAG collector。这是"协议插拔"，不是"算法替换"。
//   - 例外：对于 PCATM 协议覆盖（CODEC_014 / CODEC_017），因为
//     PCATMZColorEliminate 的 barycenter bit-exact 逻辑是 round-trip 正确性
//     的 **必要条件**（Phase 4 PR-C 第 12 轮 bad_alloc 教训），这里会把
//     CompressOptions 的 eliminate 替换为 PCATMZColorEliminateOperator。
//   - CODEC_016 覆盖同理需要 PCLTTMTagEliminateOperator（vertex tag evolution
//     是 fusedFaceSym 解码的必要条件）。
//   - RD_TAG 覆盖用 RDTagEliminateOperator（只做 proxy normalize + protocolHint
//     stamping，对 baseline 算法语义是 no-op extension，安全）。

#ifndef PMSF_CORE_RETRI_OVERRIDE_H
#define PMSF_CORE_RETRI_OVERRIDE_H

#include <cstdlib>
#include "common/PaperLayoutAliases.h"

#include "algorithm/RetriProtocolRegistry.h"
#include "algorithm/RetriProtocolTuning.h"
#include "APTM/boundary/APTMPMSFBoundary.h"
#include "MMSE/boundary/MMSEPMSFBoundary.h"
#include "PCATM/boundary/PCATMPMSFBoundary.h"
#include "PCLTTM/boundary/PCLTTMPMSFBoundary.h"
#include "PPMC/boundary/DefaultPPMCLitePMSFBoundary.h"
#include "PPMC/boundary/PPMCPMSFBoundary.h"
#include "PPMC/boundary/SegmentationPMSFBoundary.h"
#include "QEM/boundary/QEMPMSFBoundary.h"
#include "RD/boundary/RDPMSFBoundary.h"
#include "RD/boundary/RDPatchPMSFBoundary.h"
#include "VQMMC/boundary/VQMMCPMSFBoundary.h"
#include <cstdint>
#include <cmath>
#include <unordered_set>
#include <cstring>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "ContextPool.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Options.h"
#include "core/MeshRuntime.h"  // 需要 pmsf::ensureMeshRuntimeInitialized(); 触发 contextPool.initPoolSize + registerPool
#include "Point.h"
#include "common/AlgoPreset.h"
#include "core/CompressPipeline.h"
#include "predict/RDPredictor.h"  // kVPropRDR1
#include "storage/PMSFSymbolStorageAdapter.h"

#ifdef COLUMNAR_SUPPORT
#    include "MMSE/compress/MMSEDualRingSymbolCollectOperator.h"
#    include "APTM/compress/APTMConquestSymbolCollectOperator.h"
#    include "VQMMC/compress/VQMMCCodebookSymbolCollectOperator.h"
#    include "PCATM/compress/PCATMColorSymbolCollectOperator.h"
#    include "PCATM/compress/PCATMPMSFSymbolCollectOperator.h"
#    include "PCATM/compress/PCATMPureSymbolCollectOperator.h"
#    include "common/strict/PCATMStrictPMSFSymbolCollectOperator.h"
#    include "PCATM/compress/PCATMZColorSymbolCollectOperator.h"
#    include "PCLTTM/compress/PCLTTMTagSymbolCollectOperator.h"
#    include "common/strict/PCLTTMStrictSymbolCollectOperator.h"
#    include "compress/ColumnarSerializeOperator.h"
#    include "RD/compress/RDTagSymbolCollectOperator.h"
#    include "RD/compress/RDCanonicalCollector.h"
#    include "RD/elimination/RDCanonicalNoopEliminate.h"
#    include "MMSE/decompress/MMSEDualRingSymbolReadOperator.h"
#    include "common/strict/PCLTTMStrictSymbolReadOperator.h"
#    include "VQMMC/decompress/VQMMCCodebookSymbolReadOperator.h"
#    include "PCATM/decompress/PCATMColorSymbolReadOperator.h"
#    include "common/strict/PCATMStrictPMSFSymbolReadOperator.h"
#    include "PCATM/decompress/PCATMZColorSymbolReadOperator.h"
#    include "PCLTTM/decompress/PCLTTMTagSymbolReadOperator.h"
#    include "PCATM/decompress/PCATMPMSFSymbolReadOperator.h"
#    include "PCATM/decompress/PCATMPureSymbolReadOperator.h"
#    include "PCATM/decompress/PCATMZColorSymbolReadOperator.h"
#    include "decompress/ColumnarDeserializeOperator.h"
#    include "RD/decompress/RDTagSymbolReadOperator.h"
#    include "RD/decompress/RDCanonicalSymbolReadOperator.h"
#    include "RD/reconstruct/RDCanonicalReconstructOperator.h"
#    include "elimination/PCATMPaperEliminateOperator.h"
#    include "elimination/PCATMZColorEliminateOperator.h"
#    include "elimination/PCLTTMTagEliminateOperator.h"
#    include "elimination/RDTagEliminateOperator.h"
#    include "Mesh.h"
#    include "operator/MinRemovedValenceIsRemovableOperator.h"
#    include "reconstruct/BasicReconstructOperator.h"
#    include "reconstruct/PCATMReconstructOperator.h"
#    include "reconstruct/PCLTTMTagReconstructOperator.h"
#    include "feature/CurvatureHysteresisDetector.h"
#endif

namespace pmsf {

// ---------------------------------------------------------------------------
// Phase 5 PR-E：baseline 协议覆盖枚举
// ---------------------------------------------------------------------------
enum class RetriOverrideKind {
    None           = 0,   // 未开启，走 baseline
    RD_Tag         = 1,   // 走 BlobLayout::RD_TAG (18)
    PCLTTM_Tag     = 2,   // 走 BlobLayout::CODEC_016 (16)
    PCATM_Color    = 3,   // 走 BlobLayout::CODEC_014 (14)
    PCATM_ZColor   = 4,   // 走 BlobLayout::CODEC_017 (17)
    PCATM_Paper    = 5,   // 走 BlobLayout::CODEC_020 (20, true Z-shape)
    PCATM_Pure     = 6,   // 走 BlobLayout::CODEC_021 (21, PR-K3 信息论最小集)
    MMSE_DualRing   = 7,   // 走 BlobLayout::CODEC_022 (22, PR-L4b)
    VQMMC_Codebook  = 8,   // 走 BlobLayout::CODEC_023 (23, PR-L5)
    PCATM_Strict    = 11,  // 走 BlobLayout::CODEC_025 (25), encoder-side strict storage
    APTM_Conquest   = 12,  // 走 BlobLayout::CODEC_026 (26), APTM anchors + strict conquest
};

namespace retri_override_detail {

// 读 env 并判"真"（存在 + 首字符不是 0 也不是空串）。与 CompressPipeline
// 里 `PMSF_PROFILE` 的判真逻辑同源（一致可读性）。
inline bool isEnvOn(const char* name) {
    const char* v = std::getenv(name);
    return v && *v && std::strcmp(v, "0") != 0;
}

inline bool paperExactEnvOn(const char* name) {
    return pmsf::pmsfStorageEnvEnabled(name);
}

/// @brief 量化重合顶点守卫（协议覆盖路径共用）。
///
/// 低量化位数会把不同顶点映射到同一量化坐标；这类碰撞会让基于量化坐标的
/// BFS tie-break / 环预测在编码端与解码端分叉，解出的网格虽然顶点数相同，
/// 但连通性/几何都可能错误（实测：MMSE columnar LetterA q10 invalid；PCATM
/// q11 / PCLTTM q10 columnar LetterA 拓扑不同构）。标准管线有等价的 minEdge
/// 自适应守卫；协议覆盖路径此前各自硬编码 qb，没有这道保护。
///
/// 返回保证「所有顶点量化坐标互不相同」的最小位数（从 qb 起逐位提升，上限
/// 16 = 位面码流上限）；无碰撞时原样返回 qb（既有产物逐字节不变）。
inline int raiseQuantBitsForVertexCollisions(
    const std::shared_ptr<MCGAL::Mesh>& mesh, float maxRange, int qb, const char* tag) {
    if (!mesh || maxRange <= 0.0f || qb < 1 || qb > 16) return qb;
    auto countCollisions = [&mesh, maxRange](int bits) -> std::size_t {
        const float step = maxRange / static_cast<float>(1u << bits);
        const float bx = mesh->bboxMin[0];
        const float by = mesh->bboxMin[1];
        const float bz = mesh->bboxMin[2];
        std::unordered_set<std::uint64_t> seen;
        seen.reserve(mesh->vertices().size() * 2);
        std::size_t collisions = 0;
        for (MCGAL::Vertex* v : mesh->vertices()) {
            if (v == nullptr || v->isRemoved()) continue;
            const MCGAL::Point& p = v->point();
            const long long qx =
                static_cast<long long>(std::floor((p.x() - bx) / step + 0.5f));
            const long long qy =
                static_cast<long long>(std::floor((p.y() - by) / step + 0.5f));
            const long long qz =
                static_cast<long long>(std::floor((p.z() - bz) / step + 0.5f));
            const std::uint64_t kx =
                static_cast<std::uint64_t>(qx + (1LL << 20)) & 0x1FFFFFULL;
            const std::uint64_t ky =
                static_cast<std::uint64_t>(qy + (1LL << 20)) & 0x1FFFFFULL;
            const std::uint64_t kz =
                static_cast<std::uint64_t>(qz + (1LL << 20)) & 0x1FFFFFULL;
            const std::uint64_t key = kx | (ky << 21) | (kz << 42);
            if (!seen.insert(key).second) ++collisions;
        }
        return collisions;
    };
    if (countCollisions(qb) == 0) return qb;
    int guard = qb;
    while (guard < 16 && countCollisions(guard) > 0) ++guard;
    std::fprintf(stderr,
                 "[%s] quantBits raised %d -> %d to keep quantized vertices "
                 "distinct (maxRange=%g)\n",
                 tag, qb, guard, static_cast<double>(maxRange));
    return guard;
}

inline std::string formatPoint(const MCGAL::Point& p) {
    return "(" + std::to_string(p.x()) + "," + std::to_string(p.y()) + "," +
           std::to_string(p.z()) + ")";
}

inline std::string formatPointInt(const MCGAL::PointInt& p) {
    return "(" + std::to_string(p.x()) + "," + std::to_string(p.y()) + "," +
           std::to_string(p.z()) + ")";
}

inline void validatePCATMStrictQuantizedInput(
    const std::shared_ptr<MCGAL::Mesh>& mesh,
    const CompressOptions& options) {
    if (!mesh) return;
    if (!options.isEnablePrediction() || !options.isEnableQuantization()) {
        throw std::runtime_error(
            "[PCATM_STRICT][q-grid] strict paper storage requires "
            "prediction and quantization");
    }

    const unsigned quantBits = mesh->i_nbQuantBits;
    if (quantBits == 0 || quantBits >= 31) {
        throw std::runtime_error(
            "[PCATM_STRICT][q-grid] invalid quantBits=" +
            std::to_string(quantBits));
    }
    if (!std::isfinite(mesh->f_quantStep) || mesh->f_quantStep <= 0.0f) {
        throw std::runtime_error(
            "[PCATM_STRICT][q-grid] invalid quantStep=" +
            std::to_string(mesh->f_quantStep));
    }
    for (int axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(mesh->bboxMin[axis]) ||
            !std::isfinite(mesh->bboxMax[axis]) ||
            mesh->bboxMax[axis] < mesh->bboxMin[axis]) {
            throw std::runtime_error(
                "[PCATM_STRICT][q-grid] invalid bounding box axis=" +
                std::to_string(axis) + " min=" +
                std::to_string(mesh->bboxMin[axis]) + " max=" +
                std::to_string(mesh->bboxMax[axis]));
        }
    }

    // When set, restore the historical hard-refusal behavior (throw on any
    // Q12-degenerate face). The lossless-safe default instead warns and
    // continues: such faces reflect genuinely coincident/collinear input
    // geometry at the chosen quant grid and are reproduced reversibly by the
    // strict codec (verified: full compress->decompress round-trips with
    // counts/connectivity/nearest-geometry gates true). This does NOT enable
    // PMSF_REPAIR_DEGENERATE and does NOT change the decoded vertex count.
    const bool strictRefuseDegenerate =
        isEnvOn("PMSF_PCATM_STRICT_REFUSE_QGRID_DEGENERATE");
    std::size_t degenerateRepeatedFaces = 0;
    std::size_t degenerateCollinearFaces = 0;

    std::size_t faceCount = 0;
    for (MCGAL::Facet* face : mesh->faces()) {
        if (!face || face->isRemoved()) continue;
        ++faceCount;
        const std::vector<MCGAL::Vertex*> vertices = face->getVertices();
        if (vertices.size() != 3) {
            throw std::runtime_error(
                "[PCATM_STRICT][q-grid] input face is not triangular: face=" +
                std::to_string(face->poolId()) + " vertex_count=" +
                std::to_string(vertices.size()));
        }

        MCGAL::PointInt q[3];
        for (int corner = 0; corner < 3; ++corner) {
            MCGAL::Vertex* vertex = vertices[corner];
            if (!vertex) {
                throw std::runtime_error(
                    "[PCATM_STRICT][q-grid] null vertex: face=" +
                    std::to_string(face->poolId()) + " corner=" +
                    std::to_string(corner));
            }
            const MCGAL::Point point = vertex->point();
            if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
                !std::isfinite(point.z())) {
                throw std::runtime_error(
                    "[PCATM_STRICT][q-grid] non-finite vertex: face=" +
                    std::to_string(face->poolId()) + " vertex=" +
                    std::to_string(vertex->globalVertexId()) + " point=" +
                    formatPoint(point));
            }
            q[corner] = mesh->floatPosToInt(point);
        }

        const bool repeatedQuantizedVertex =
            (q[0].x() == q[1].x() && q[0].y() == q[1].y() &&
             q[0].z() == q[1].z()) ||
            (q[1].x() == q[2].x() && q[1].y() == q[2].y() &&
             q[1].z() == q[2].z()) ||
            (q[0].x() == q[2].x() && q[0].y() == q[2].y() &&
             q[0].z() == q[2].z());
        const MCGAL::PointInt cross = (q[1] - q[0]).cross(q[2] - q[0]);
        const bool quantizedCollinear =
            cross.x() == 0 && cross.y() == 0 && cross.z() == 0;
        if (repeatedQuantizedVertex || quantizedCollinear) {
            const std::string detail =
                std::string("[PCATM_STRICT][q-grid] q") +
                std::to_string(quantBits) + "-degenerate face: face=" +
                std::to_string(face->poolId()) + " vertices=(" +
                std::to_string(vertices[0]->globalVertexId()) + "," +
                std::to_string(vertices[1]->globalVertexId()) + "," +
                std::to_string(vertices[2]->globalVertexId()) + ") q0=" +
                formatPointInt(q[0]) + " q1=" + formatPointInt(q[1]) +
                " q2=" + formatPointInt(q[2]) +
                (repeatedQuantizedVertex
                     ? " reason=repeated_quantized_vertex"
                     : " reason=quantized_collinear");
            if (strictRefuseDegenerate) {
                throw std::runtime_error(
                    "[PCATM_STRICT][q-grid] input is not representable at q" +
                    std::to_string(quantBits) + ": " + detail);
            }
            // Lossless-safe default: distinct float vertices that fall into the
            // same (or a collinear) Q-cell are preserved as separate vertices by
            // the strict codec; this only yields a decoded zero-area face at the
            // quantization resolution, which does not change the vertex count or
            // break connectivity / nearest-geometry reconstruction.
            if (repeatedQuantizedVertex) ++degenerateRepeatedFaces;
            else ++degenerateCollinearFaces;
            std::fprintf(stderr, "%s (tolerated; lossless round-trip)\n",
                         detail.c_str());
        }
    }

    std::fprintf(stderr,
                 "[PCATM_STRICT][q-grid] valid input: quantBits=%u "
                 "quantStep=%g faces=%zu vertices=%zu "
                 "degenerate_repeated_faces=%zu "
                 "degenerate_collinear_faces=%zu\n",
                 quantBits, mesh->f_quantStep, faceCount,
                 mesh->vertices().size(), degenerateRepeatedFaces,
                 degenerateCollinearFaces);
}

}  // namespace retri_override_detail

// 给定 baseline 算法 preset，返回**默认最优**协议覆盖（或 env 显式指定的）。
//
// 2026-05-12 起切换"最优即默认"策略（用户 query "将所有 impl 的默认路径换成算法的最优 bpv 路径"）：
//   RD     默认 → RD_Tag    (ConBPV 7.21，仓库最优，replaces RD_GATE=7 baseline)
//   PCLTTM 默认 → PCLTTM_Tag(16)：正确性优先的 tag/fused-face-symbol 存储。
//   PCATM  默认 → baseline。CODEC_021/ZCOLOR 在 LetterA 上仍是实验路径，
//                  需要显式 env opt-in。
//   MMSE   默认 → RD_Tag。CODEC_022 在 LetterA 上仍是实验路径，
//                  需要显式 env opt-in。
//
// 反向 opt-out：若设置 PMSF_USE_ORIG_BASELINE=1 则回到旧 baseline（用于
// md5 字节级回归诊断；旧 baseline md5 已由 docs/paper_alignment/regression_baseline.md
// 保留为历史记录）。
//
// 显式 env 覆盖：
//   PMSF_RD_USE_RETRI_TAG           → RD_Tag (显式确认)
//   PMSF_RD_USE_PCLTTM_TAG          → PCLTTM_Tag storage for RD selector
//   PMSF_PCLTTM_USE_RETRI_TAG       → PCLTTM_Tag (显式确认；也是默认)
//   PMSF_PCATM_USE_PURE_K3          → PCATM_Pure (显式确认)
//   PMSF_PCATM_USE_PURE_PAPER       → PCATM_Paper
//   PMSF_PCATM_USE_RETRI_ZCOLOR     → PCATM_ZColor
//   PMSF_PCATM_USE_RETRI_COLOR      → PCATM_Color
//   PMSF_APTM_USE_RETRI_TAG         → RD_Tag (APTM 借；也是当前低 BPV 可逆默认)
//   PMSF_APTM_USE_CONQUEST          → APTM_Conquest (实验性 strict conquest locator)
//   PMSF_MMSE_USE_RETRI_TAG         → RD_Tag (MMSE 借)
//   PMSF_MMSE_USE_PCLTTM_TAG        → PCLTTM_Tag storage for MMSE selector
//   PMSF_QEM_USE_RETRI_TAG          → RD_Tag (QEM 借)
//
// 其它 preset（PPMC / POMAR / VQMMC / PCLTTMValence）：无覆盖（plan §3.3）。
inline RetriOverrideKind detectRetriOverride(AlgoPreset preset) {
    using namespace retri_override_detail;

    // opt-out：显式请求旧 baseline 路径（md5 回归诊断用）。
    const bool useOrigBaseline = isEnvOn("PMSF_USE_ORIG_BASELINE");

    switch (preset) {
        case AlgoPreset::Default:
            (void)useOrigBaseline;
            // LetterA: the RD_TAG override can produce a stream that aborts
            // during BasicReconstructOperator::merge_face. Keep Default on
            // its verified baseline storage unless an explicit tool opts into
            // a specialized protocol.
            return RetriOverrideKind::None;
        case AlgoPreset::RD:
            if (isEnvOn("PMSF_RD_USE_PCLTTM_TAG")) return RetriOverrideKind::PCLTTM_Tag;
            if (isEnvOn("PMSF_RD_USE_RETRI_TAG")) return RetriOverrideKind::RD_Tag;
            if (useOrigBaseline) return RetriOverrideKind::None;
            // **默认走 RD_Tag**（ConBPV 7.21，仓库最优）
            return RetriOverrideKind::RD_Tag;
        case AlgoPreset::PCLTTM:
            if (isEnvOn("PMSF_PCLTTM_USE_RETRI_TAG")) return RetriOverrideKind::PCLTTM_Tag;
            if (useOrigBaseline) return RetriOverrideKind::None;
            // 默认走已验证 round-trip 的 CODEC_016。
            return RetriOverrideKind::PCLTTM_Tag;
        case AlgoPreset::PCATM:
            // 显式 env 分级（从更激进到更保守）
            if (isEnvOn("PMSF_PCATM_USE_STRICT_PAPER")) return RetriOverrideKind::PCATM_Strict;
            if (isEnvOn("PMSF_PCATM_USE_PURE_K3"))      return RetriOverrideKind::PCATM_Pure;
            if (isEnvOn("PMSF_PCATM_USE_PURE_PAPER"))   return RetriOverrideKind::PCATM_Paper;
            if (isEnvOn("PMSF_PCATM_USE_RETRI_ZCOLOR")) return RetriOverrideKind::PCATM_ZColor;
            if (isEnvOn("PMSF_PCATM_USE_RETRI_COLOR"))  return RetriOverrideKind::PCATM_Color;
            (void)useOrigBaseline;
            // LetterA: CODEC_021 hangs in decoder and ZCOLOR aborts; keep the
            // verified baseline path as default, while preserving opt-in envs.
            return RetriOverrideKind::None;
        case AlgoPreset::APTM:
            if (isEnvOn("PMSF_APTM_USE_RETRI_TAG")) return RetriOverrideKind::RD_Tag;
            if (isEnvOn("PMSF_APTM_USE_CONQUEST")) return RetriOverrideKind::APTM_Conquest;
            if (useOrigBaseline) return RetriOverrideKind::None;
            // RD_TAG bridge for APTM currently produces a single-round connectivity
            // blob that does not round-trip with the multi-round decoder
            // (residual round-context count mismatch). Keep the verified baseline
            // path as default; RD_TAG and strict conquest remain explicit opt-in.
            return RetriOverrideKind::None;
        case AlgoPreset::MMSE:
            if (isEnvOn("PMSF_MMSE_USE_PCLTTM_TAG")) return RetriOverrideKind::PCLTTM_Tag;
            if (isEnvOn("PMSF_MMSE_USE_DUAL_RING")) return RetriOverrideKind::MMSE_DualRing;
            if (isEnvOn("PMSF_MMSE_USE_RETRI_TAG")) return RetriOverrideKind::RD_Tag;
            if (useOrigBaseline) return RetriOverrideKind::None;
            // LetterA: CODEC_022 hangs in decoder and the historical
            // Columnar baseline aborts with the current predictive header.
            // RD_TAG is the verified reversible path for this executable.
            return RetriOverrideKind::RD_Tag;
        case AlgoPreset::QEM:
            if (isEnvOn("PMSF_QEM_USE_RETRI_TAG")) return RetriOverrideKind::RD_Tag;
            if (useOrigBaseline) return RetriOverrideKind::None;
            return RetriOverrideKind::RD_Tag;
        case AlgoPreset::VQMMC:
            if (isEnvOn("PMSF_VQMMC_USE_CODEBOOK")) return RetriOverrideKind::VQMMC_Codebook;
            return RetriOverrideKind::None;
        default:
            return RetriOverrideKind::None;
    }
}

// 人类可读的协议名（用于日志 / bpv diag）。
inline const char* retriOverrideName(RetriOverrideKind k) {
    switch (k) {
        case RetriOverrideKind::None:         return "None";
        case RetriOverrideKind::RD_Tag:       return "RD_TAG(18)";
        case RetriOverrideKind::PCLTTM_Tag:   return "CODEC_016(16)";
        case RetriOverrideKind::PCATM_Color:  return "CODEC_014(14)";
        case RetriOverrideKind::PCATM_ZColor: return "CODEC_017(17)";
        case RetriOverrideKind::PCATM_Paper:  return "CODEC_020(20)";
        case RetriOverrideKind::PCATM_Pure:    return "CODEC_021(21)";
        case RetriOverrideKind::MMSE_DualRing:  return "CODEC_022(22)";
        case RetriOverrideKind::VQMMC_Codebook: return "CODEC_023(23)";
        case RetriOverrideKind::PCATM_Strict:   return "CODEC_025(25)";
        case RetriOverrideKind::APTM_Conquest:  return "CODEC_026(26)";
    }
    return "?";
}

// 人类可读的 preset 名（日志用）。AlgoPreset.h 当前未提供，这里就近补一个
// （纯内联，不影响 ABI）。只覆盖本 header 会触碰的 7 个 baseline。
inline const char* retriPresetName(AlgoPreset p) {
    switch (p) {
        case AlgoPreset::Default:       return "Default";
        case AlgoPreset::QEM:           return "QEM";
        case AlgoPreset::PPMC:          return "PPMC";
        case AlgoPreset::MMSE:          return "MMSE";
        case AlgoPreset::APTM:          return "APTM";
        case AlgoPreset::PCATM:         return "PCATM";
        case AlgoPreset::PCLTTM:        return "PCLTTM";
        case AlgoPreset::PCLTTMValence: return "PCLTTMValence";
        case AlgoPreset::RD:            return "RD";
        case AlgoPreset::POMAR:         return "POMAR";
        case AlgoPreset::POMARV2:       return "POMARV2";
        case AlgoPreset::VQMMC:         return "VQMMC";
    }
    return "?";
}

// ---------------------------------------------------------------------------
// Compress 覆盖：检测 env，on 时接管整个 run，成功返回 true。
//
// 调用约定：baseline main 在构造完 CompressOptions 后、调 CompressPipeline::run
// 之前调本函数；若返回 true，main 立刻 `return 0`，不走 baseline 调度。
// ---------------------------------------------------------------------------
#ifdef COLUMNAR_SUPPORT
/// @brief 把历史的 per-preset 行为固化成参数（与 Phase 1 之前的 `preset ==` 分支
///        逐条等价；改这里等于改所有走协议覆盖的算法的字节输出，需跑回归）。
inline RetriProtocolTuning tuningForPreset(AlgoPreset preset) {
    RetriProtocolTuning t;
    switch (preset) {
        case AlgoPreset::RD:
            t.adaptiveResidual = true;
            t.defaultQuantBits = 11;
            t.quantBitsEnv = "PMSF_RDTAG_QUANT_BITS";
            break;
        case AlgoPreset::APTM:
            t.defaultQuantBits = 12;
            t.quantBitsEnv = "PMSF_RDTAG_QUANT_BITS";
            t.altQuantBitsEnv = "PMSF_APTM_QUANT_BITS";
            break;
        case AlgoPreset::QEM:
            t.keepAdjustment = true;
            // 与历史一致：QEM 无人为设置量化环境变量时保持默认位宽。
            t.requireQuantBitsEnv = true;
            t.quantBitsEnv = "PMSF_RDTAG_QUANT_BITS";
            t.altQuantBitsEnv = "PMSF_APTM_QUANT_BITS";
            break;
        default:
            t.requireQuantBitsEnv = true;
            t.quantBitsEnv = "PMSF_RDTAG_QUANT_BITS";
            t.altQuantBitsEnv = "PMSF_APTM_QUANT_BITS";
            break;
    }
    return t;
}

/// @brief 协议 id（`ISimplificationAlgorithm::retriangulationProtocol()` 的返回值）
///        → 覆盖种类。空串 / "none" ⇒ 不覆盖。
/// @brief 产物自带的 blob_layout → 读取算子族。
///
/// 数字取自 `src/storage/PaperBlobLayoutRegistry.h`（权威定义），与
/// `retriOverrideName()` 括号里的编号一一对应。
///
/// 为什么需要它：有些算法的协议字符串**取决于环境变量**（发布配置里才有），
/// 而随机访问查询的进程未必带着那套环境变量。产物的 blob_layout 是它自己写下的，
/// 据此选 reader 才能做到「同一个产物在哪个进程里解压都一样」。
/// 这也正是设计文档 §2.3「metadata 记名 + 注册表再实例化」的方向。
inline RetriOverrideKind retriOverrideKindFromLayout(int layout) {
    switch (layout) {
        case pmsf::kPaperBlobLayoutPCATMColor:      return RetriOverrideKind::PCATM_Color;
        case pmsf::kPaperBlobLayoutPCLTTMTag:       return RetriOverrideKind::PCLTTM_Tag;
        case pmsf::kPaperBlobLayoutPCATMZColor:     return RetriOverrideKind::PCATM_ZColor;
        case pmsf::kPaperBlobLayoutRDTag:           return RetriOverrideKind::RD_Tag;
        case pmsf::kPaperBlobLayoutPCATMPaper:      return RetriOverrideKind::PCATM_Paper;
        case pmsf::kPaperBlobLayoutPCATMPure:       return RetriOverrideKind::PCATM_Pure;
        case pmsf::kPaperBlobLayoutMMSEDualRing:    return RetriOverrideKind::MMSE_DualRing;
        case pmsf::kPaperBlobLayoutVQMMCCodebook:   return RetriOverrideKind::VQMMC_Codebook;
        case pmsf::kPaperBlobLayoutPCATMStrictPaper:return RetriOverrideKind::PCATM_Strict;
        case pmsf::kPaperBlobLayoutAPTMConquest:    return RetriOverrideKind::APTM_Conquest;
        default:                                    return RetriOverrideKind::None;
    }
}

inline RetriOverrideKind retriOverrideKindFromId(const std::string& id) {
    if (id.empty() || id == "none") return RetriOverrideKind::None;
    if (id == "rd_tag")        return RetriOverrideKind::RD_Tag;
    if (id == "pclttm_tag")    return RetriOverrideKind::PCLTTM_Tag;
    if (id == "pcatm_color")   return RetriOverrideKind::PCATM_Color;
    if (id == "pcatm_zcolor")  return RetriOverrideKind::PCATM_ZColor;
    if (id == "pcatm_pure")    return RetriOverrideKind::PCATM_Pure;
    if (id == "pcatm_paper")   return RetriOverrideKind::PCATM_Paper;
    if (id == "pcatm_strict")  return RetriOverrideKind::PCATM_Strict;
    if (id == "aptm_conquest") return RetriOverrideKind::APTM_Conquest;
    if (id == "mmse_dual_ring") return RetriOverrideKind::MMSE_DualRing;
    if (id == "vqmmc_codebook") return RetriOverrideKind::VQMMC_Codebook;
    return RetriOverrideKind::None;
}

/// @brief 协议覆盖的实际执行体。`presetForLog` 仅供日志与尚未契约化的分支使用；
///        ById 入口只会以 RD_Tag 调用它。
inline bool runRetriCompressOverride(CompressOptions& options,
                                     RetriOverrideKind kind,
                                     const RetriProtocolTuning& tuning,
                                     AlgoPreset presetForLog,
                                     pmsf::Backend backend) {
    const AlgoPreset preset = presetForLog;
    // PMSF 构造触发 contextPool.initPoolSize(1) + registerPool —— 这是所有
    // baseline impl main 都依赖 `CompressPipeline::run` 内部 `pmsf::ensureMeshRuntimeInitialized();` 隐式
    // 完成的 pool 初始化步骤。我们 override 旁路了 run()，所以必须显式做。
    // 按输入 OFF 头预读规模注册（标定乘数收敛在 MeshRuntime），避免默认小池
    // 在大模型（如 armadillo）上耗尽。
    pmsf::ensureMeshRuntimeInitializedForInputFile(options.getPath());

    // 为 residual 的二阶 curvature predictor attach kVPropRDR1 顶点属性。
    // 三种 Tag 协议的 encoder BFS 二遍 + decoder collectFromDecoded 都共享读写这个属性。
    MCGAL::contextPool.attachVertexProperty<MCGAL::PointInt>(
        DEFAULT_MESH_ID, MCGAL::kVPropRDR1);

    // 构造对应协议的 pipeline cfg + 可能 override eliminate。
    // M4-P0：覆盖路径必须跟随请求的存储后端 —— paper 单文件容器与 columnar
    // 段表产物布局不同，硬编码 defaultColumnar 会把 paper 后端写成多文件。
    auto cfg = (backend == pmsf::Backend::Paper)
                   ? pmsf::CompressPipelineConfig::defaultPaper(
                         tuning.paperArtifactAlgorithm.empty() ? pmsf::retriPresetName(presetForLog)
                                                               : tuning.paperArtifactAlgorithm)
                   : pmsf::CompressPipelineConfig::defaultColumnar();

    switch (kind) {
        case RetriOverrideKind::RD_Tag: {
            if (std::getenv("PMSF_V2_COMPACT") == nullptr) {
                setenv("PMSF_V2_COMPACT", "1", /*overwrite=*/0);
            }
            if (tuning.adaptiveResidual &&
                std::getenv("PMSF_RDTAG_ADAPTIVE_RESIDUAL") == nullptr) {
                setenv("PMSF_RDTAG_ADAPTIVE_RESIDUAL", "1", /*overwrite=*/0);
            }
            // RD_TAG eliminate 继承 VertexRemoval，仅多做 proxy normalize +
            // protocolHint stamping；对 RD baseline 的 tag-priority select
            // 完全兼容（作用在不同 hook 点）。
            // 对 APTM/MMSE/QEM 也安全：它们的 eliminate 也是 VertexRemoval 派生。
            options.setEliminate(std::make_shared<RDTagEliminateOperator>());
            // QEM 的 quadric optimal placement 是算法语义的一部分，不能在 RD_TAG
            // 覆盖路径中关闭。配套 RDTag sv=6 会把 survivor old-position 作为
            // 相对当前 QEM placement 的预测残差写入 adjustment stream，decoder
            // 在拆点前恢复。其它 preset 仍沿用 RD_TAG 的旧几何假设，避免额外流。
            options.setEnableAdjustment(tuning.keepAdjustment);
            cfg.collectorFactory = []() -> std::shared_ptr<SymbolCollectOperator> {
                return std::make_shared<RDTagSymbolCollectOperator>();
            };
            // 复制 PMSF::rdColumnarCompress 的关键 cfg（否则 baseline 的
            // trackContextLevel/updateAfterEliminate 会漏掉）。
            cfg.trackContextLevel = true;
            cfg.enableIndependentSet = true;
            cfg.updateAfterEliminate = false;
            // ---- task-43 route A（env PMSF_RD_CANON=1，默认关闭）-------------
            const char* rdCanonEnv = std::getenv("PMSF_RD_CANON");
            const bool rdCanon = (rdCanonEnv != nullptr && *rdCanonEnv != '\0' &&
                                  std::strcmp(rdCanonEnv, "0") != 0);
            if (rdCanon) {
                options.setEliminate(std::make_shared<
                                     rd_canon::RDCanonicalNoopEliminateOperator>());
                cfg.collectorFactory =
                    []() -> std::shared_ptr<SymbolCollectOperator> {
                    return std::make_shared<rd_canon::RDCanonicalCollector>();
                };
                cfg.enableIndependentSet = false;
                cfg.updateAfterEliminate = false;
                cfg.trackContextLevel = true;
                cfg.seedStrategy =
                    pmsf::CompressPipelineConfig::SeedStrategy::SelectSeed;
                cfg.customSeedSelector =
                    [](std::shared_ptr<MCGAL::Mesh> mesh) -> MCGAL::Halfedge* {
                    if (!mesh) return nullptr;
                    MCGAL::Halfedge* best = nullptr;
                    std::uint64_t bestKey = ~static_cast<std::uint64_t>(0);
                    for (MCGAL::Facet* f : mesh->faces()) {
                        if (f == nullptr || f->isRemoved()) continue;
                        MCGAL::Halfedge* proxy = f->proxyHalfedge();
                        if (proxy == nullptr || proxy->vertex() == nullptr) continue;
                        const MCGAL::PointInt q =
                            mesh->floatPosToInt(proxy->vertex()->point());
                        const std::uint64_t key =
                            (static_cast<std::uint64_t>(
                                 static_cast<std::uint32_t>(q.x()) & 0x1fffffu)
                             << 42) |
                            (static_cast<std::uint64_t>(
                                 static_cast<std::uint32_t>(q.y()) & 0x1fffffu)
                             << 21) |
                            static_cast<std::uint64_t>(
                                static_cast<std::uint32_t>(q.z()) & 0x1fffffu);
                        if (key < bestKey) {
                            bestKey = key;
                            best = proxy;
                        }
                    }
                    return best;
                };
            }
            // ---- task-19 P1/P2a（env-gated，默认路径逐字节不变）-------------
            // PMSF_RD_PGS=1：
            //   (a) P1 动态打分：打开 updateAfterEliminate，让 tag/conquered
            //       演化真正进入 priority 重算（默认 false 时队列只建一次）。
            //   (b) P2a 确定性 conquest：RDAlgorithm::selector() 已切 Greedy；
            //       这里补 **可复现 seed**（GreedySelector::selectSeed 的 rand()
            //       回退不可接受）+ 显式 SelectSeed。customSeedSelector 用
            //       「face 的 proxy 顶点量化坐标字典序最小」作 canonical 规则，
            //       decoder 侧可用同一规则独立复算（P3 replay 的接缝）。
            //   (c) GreedySelector 自管理 gateQueue/conquered，无需 PQ 度量。
            const char* rdPgsEnv = std::getenv("PMSF_RD_PGS");
            const bool rdPgs = (rdPgsEnv != nullptr && *rdPgsEnv != '\0' &&
                                std::strcmp(rdPgsEnv, "0") != 0);
            if (rdPgs) {
                cfg.updateAfterEliminate = true;
                // P2a/P3：论文的 conquest 是「pop 一个 gate ⇒ 消一个 front vertex」
                // 的 1:1 关系；框架 independent-set 会在 select 之后二次拒绝
                // （gate 被消费但无 elimination）破坏该对应，也会给 decoder replay
                // 增加一层状态。PGS 实验关闭 IS（默认路径仍 true，逐字节不变）。
                cfg.enableIndependentSet = false;
                cfg.seedStrategy =
                    pmsf::CompressPipelineConfig::SeedStrategy::SelectSeed;
                cfg.customSeedSelector =
                    [](std::shared_ptr<MCGAL::Mesh> mesh) -> MCGAL::Halfedge* {
                    if (!mesh) return nullptr;
                    MCGAL::Halfedge* best = nullptr;
                    std::uint64_t bestKey = ~static_cast<std::uint64_t>(0);
                    for (MCGAL::Facet* f : mesh->faces()) {
                        if (f == nullptr || f->isRemoved()) continue;
                        MCGAL::Halfedge* proxy = f->proxyHalfedge();
                        if (proxy == nullptr || proxy->vertex() == nullptr) continue;
                        const MCGAL::PointInt q =
                            mesh->floatPosToInt(proxy->vertex()->point());
                        const std::uint64_t key =
                            (static_cast<std::uint64_t>(
                                 static_cast<std::uint32_t>(q.x()) & 0x1fffffu)
                             << 42) |
                            (static_cast<std::uint64_t>(
                                 static_cast<std::uint32_t>(q.y()) & 0x1fffffu)
                             << 21) |
                            static_cast<std::uint64_t>(
                                static_cast<std::uint32_t>(q.z()) & 0x1fffffu);
                        if (key < bestKey) {
                            bestKey = key;
                            best = proxy;
                        }
                    }
                    return best;
                };
            }
            cfg.preInitPreparation =
                [tuning](std::shared_ptr<MCGAL::Mesh> mesh,
                         PMSFHeader&,
                         const CompressOptions&) {
                    if (!mesh) return;
                    if (!tuning.shouldApplyQuantBits()) return;
                    const int qb = tuning.resolveQuantBits();
                    if (qb < 8 || qb > 16) {
                        std::fprintf(stderr,
                                     "[RD_TAG] ignore invalid quantBits=%d\n", qb);
                        return;
                    }
                    mesh->i_nbQuantBits = static_cast<unsigned>(qb);
                    float maxRange = 0.0f;
                    for (int axis = 0; axis < 3; ++axis) {
                        maxRange = std::max(maxRange,
                                            mesh->bboxMax[axis] - mesh->bboxMin[axis]);
                    }
                    mesh->f_quantStep =
                        maxRange / static_cast<float>(1u << mesh->i_nbQuantBits);
                    std::fprintf(stderr,
                                 "[RD_TAG] quantBits=%u quantStep=%g\n",
                                 mesh->i_nbQuantBits, mesh->f_quantStep);
                };
            break;
        }
        case RetriOverrideKind::PCLTTM_Tag: {
            // CODEC_016 eliminate 必须是 PCLTTMTagEliminateOperator —— 因为
            // encoder 端的 vertex tag evolution 是 decoder 端解 fusedFaceSym
            // 的必要配对（Phase 3 Step 1）。
            if (std::getenv("PMSF_V2_COMPACT") == nullptr) {
                setenv("PMSF_V2_COMPACT", "1", /*overwrite=*/0);
            }
            auto patches = tuning.localFrameResidual ? std::make_shared<pclttm_frenet::PendingPatches>() : nullptr;
            options.setEliminate(std::make_shared<PCLTTMTagEliminateOperator>(patches));
            cfg.collectorFactory = [patches]() -> std::shared_ptr<SymbolCollectOperator> {
                return std::make_shared<PCLTTMTagSymbolCollectOperator>(patches);
            };
            cfg.seedStrategy = pmsf::CompressPipelineConfig::SeedStrategy::SelectThenExtract;
            cfg.recomputeVertexDegreesOnce = true;
            cfg.trackContextLevel = true;
            cfg.updateAfterEliminate = false;
            cfg.preInitPreparation =
                [](std::shared_ptr<MCGAL::Mesh> mesh,
                   PMSFHeader&,
                   const CompressOptions&) {
                    if (!mesh) return;
                    int qb = 12;
                    if (const char* qbEnv = std::getenv("PMSF_PCLTTM_QUANT_BITS")) {
                        qb = std::atoi(qbEnv);
                    } else if (const char* qbEnv = std::getenv("PMSF_RDTAG_QUANT_BITS")) {
                        qb = std::atoi(qbEnv);
                    }
                    if (qb < 8 || qb > 16) {
                        std::fprintf(stderr,
                                     "[PCLTTM] ignore invalid quantBits=%d\n",
                                     qb);
                        return;
                    }
                    float maxRange = 0.0f;
                    for (int axis = 0; axis < 3; ++axis) {
                        maxRange = std::max(maxRange,
                                            mesh->bboxMax[axis] - mesh->bboxMin[axis]);
                    }
                    qb = retri_override_detail::raiseQuantBitsForVertexCollisions(
                        mesh, maxRange, qb, "PCLTTM");
                    mesh->i_nbQuantBits = static_cast<unsigned>(qb);
                    mesh->f_quantStep =
                        maxRange / static_cast<float>(1u << mesh->i_nbQuantBits);
                    std::fprintf(stderr,
                                 "[PCLTTM] quantBits=%u quantStep=%g\n",
                                 mesh->i_nbQuantBits, mesh->f_quantStep);
                };
            break;
        }
        case RetriOverrideKind::APTM_Conquest: {
            if (std::getenv("PMSF_PCLTTM_SINGLE_SEED") == nullptr) {
                setenv("PMSF_PCLTTM_SINGLE_SEED", "1", /*overwrite=*/0);
            }
            if (std::getenv("PMSF_PCLTTM_REVERSE_WITHIN_LAYERS") == nullptr) {
                setenv("PMSF_PCLTTM_REVERSE_WITHIN_LAYERS", "0", /*overwrite=*/0);
            }
            if (std::getenv("PMSF_V2_COMPACT") == nullptr) {
                setenv("PMSF_V2_COMPACT", "1", /*overwrite=*/0);
            }
            if (std::getenv("PMSF_APTM_QUANT_BITS") == nullptr) {
                setenv("PMSF_APTM_QUANT_BITS", "12", /*overwrite=*/0);
            }
            if (std::getenv("PMSF_PCLTTM_WRITE_RECON_RING_IDS") == nullptr) {
                setenv("PMSF_PCLTTM_WRITE_RECON_RING_IDS", "0", /*overwrite=*/0);
            }
            if (std::getenv("PMSF_PCLTTM_WRITE_RECON_ORIGINAL_IDS") == nullptr) {
                setenv("PMSF_PCLTTM_WRITE_RECON_ORIGINAL_IDS", "0", /*overwrite=*/0);
            }
            if (std::getenv("PMSF_PCLTTM_REVERSE_V_ONLY") == nullptr) {
                setenv("PMSF_PCLTTM_REVERSE_V_ONLY", "1", /*overwrite=*/0);
            }
            if (std::getenv("PMSF_PCLTTM_REVERSE_RANKS") == nullptr) {
                setenv("PMSF_PCLTTM_REVERSE_RANKS", "1", /*overwrite=*/0);
            }
            if (std::getenv("PMSF_PCLTTM_PACK_AS_IS") == nullptr) {
                setenv("PMSF_PCLTTM_PACK_AS_IS", "1", /*overwrite=*/0);
            }
            if (std::getenv("PMSF_PCLTTM_WRITE_RANKS") == nullptr) {
                setenv("PMSF_PCLTTM_WRITE_RANKS", "1", /*overwrite=*/0);
            }
            if (std::getenv("PMSF_PCLTTM_RANK_CODEC") == nullptr) {
                setenv("PMSF_PCLTTM_RANK_CODEC", "aptm_locator_v1", /*overwrite=*/0);
            }
            if (std::getenv("PMSF_RDPATCH_WRITE_FACE_RANKS") == nullptr) {
                setenv("PMSF_RDPATCH_WRITE_FACE_RANKS", "1", /*overwrite=*/0);
            }
            if (std::getenv("PMSF_PCLTTM_WRITE_RINGS") == nullptr) {
                setenv("PMSF_PCLTTM_WRITE_RINGS", "0", /*overwrite=*/0);
            }

            auto mesh = std::make_shared<MCGAL::Mesh>(MCGAL::Mesh(options.getPath()));
            {
                pmsf::CurvatureHysteresisDetector detector;
                const pmsf::FeatureSet anchors = detector.detect(mesh);
                const int meshId = mesh->meshId();
                for (std::uint32_t poolId : anchors) {
                    MCGAL::Vertex* v =
                        MCGAL::contextPool.getVertexByIndexInSubPool(
                            meshId, static_cast<int>(poolId));
                    if (v && !v->isRemoved()) {
                        v->setAnchor();
                    }
                }
                std::fprintf(stderr, "[APTM] detected anchors: %zu\n", anchors.size());
            }

            if (const char* qbEnv = std::getenv("PMSF_APTM_QUANT_BITS")) {
                const int qb = std::atoi(qbEnv);
                if (qb >= 8 && qb <= 16) {
                    mesh->i_nbQuantBits = static_cast<unsigned>(qb);
                    float maxRange = 0.0f;
                    for (int axis = 0; axis < 3; ++axis) {
                        maxRange = std::max(maxRange,
                                            mesh->bboxMax[axis] - mesh->bboxMin[axis]);
                    }
                    mesh->f_quantStep =
                        maxRange / static_cast<float>(1u << mesh->i_nbQuantBits);
                }
            }

            PMSFHeader header = PMSFHeader::Builder()
                                    .setRound(1)
                                    .setEnableCompress(true)
                                    .setEnableQuantization(true)
                                    .setEnablePrediction(true)
                                    .build();
            auto collector = std::make_shared<APTMConquestSymbolCollectOperator>();
            collector->init(mesh, /*seeds=*/{}, header);
            collector->collect(nullptr);

            MCGAL::Halfedge* baseSeed = nullptr;
            for (MCGAL::Facet* f : mesh->faces()) {
                if (!f || f->isRemoved()) continue;
                for (MCGAL::Halfedge* h : *f) {
                    if (h && !h->isRemoved()) {
                        baseSeed = h;
                        break;
                    }
                }
                if (baseSeed) break;
            }
            if (!baseSeed) {
                throw std::runtime_error("[CODEC_026] no living base-mesh halfedge");
            }

            auto serializer = std::make_shared<ColumnarSerializeOperator>(
                collector,
                /*enablePrediction=*/true,
                /*enableQuantization=*/true,
                /*enableCompress=*/true);
            serializer->init();
            serializer->setHeader(std::make_shared<PMSFHeader>(header));
            serializer->serializeBaseMeshWithSeed(mesh, {baseSeed});
            serializer->serialize(options.getOutputPath());

            std::fprintf(stderr,
                         "[RetriOverride] preset=%s proto=%s → manual APTM conquest storage "
                         "(conn=%d B, geom=%d B, overhead=%d B)\n",
                         retriPresetName(preset), retriOverrideName(kind),
                         collector->lastConnectivityBytes(),
                         collector->lastGeometryBytes(),
                         collector->lastContainerOverheadBytes());
            return true;
        }
        case RetriOverrideKind::PCATM_Strict: {
            // PCATM strict-paper storage: keep the PCATM-Paper elimination path
            // (barycenter residual + alternating face colors) but export only
            // the paper streams: color LZ + residual Huffman.
            options.setEliminate(std::make_shared<PCATMPaperEliminateOperator>());
            options.setEnableAdjustment(false);
            if (const char* minValenceEnv =
                    std::getenv("PMSF_PCATM_STRICT_MIN_VALENCE")) {
                const int minValence = std::atoi(minValenceEnv);
                if (minValence > 0) {
                    options.getSelect()->addIsRemovableOperator(
                        std::make_shared<pmsf::MinRemovedValenceIsRemovableOperator>(
                            minValence));
                }
            }
            cfg.collectorFactory = []() -> std::shared_ptr<SymbolCollectOperator> {
                return std::make_shared<PCATMStrictPMSFSymbolCollectOperator>();
            };
            cfg.seedStrategy = pmsf::CompressPipelineConfig::SeedStrategy::SelectThenExtract;
            if (std::getenv("PMSF_PCATM_STRICT_FACE_ORDER") == nullptr &&
                std::getenv("PMSF_PCATM_STRICT_MORTON_FACE_ORDER") == nullptr &&
                std::getenv("PMSF_PCATM_STRICT_STABLE_FACE_ORDER") == nullptr) {
                setenv("PMSF_PCATM_STRICT_STABLE_FACE_ORDER", "1", /*overwrite=*/0);
            }
            cfg.recomputeVertexDegreesOnce = true;
            cfg.trackContextLevel = true;
            cfg.enableIndependentSet = true;
            cfg.updateAfterEliminate = false;
            cfg.stopOnEmptyRound = true;
            cfg.preInitPreparation =
                [](std::shared_ptr<MCGAL::Mesh> mesh,
                   PMSFHeader&,
                   const CompressOptions& options) {
                    if (!mesh) return;
                    const char* qbEnv = std::getenv("PMSF_PCATM_STRICT_QUANT_BITS");
                    if (qbEnv && *qbEnv && std::atoi(qbEnv) != 12) {
                        std::fprintf(stderr,
                                     "[PCATM_STRICT] ignore invalid "
                                     "PMSF_PCATM_STRICT_QUANT_BITS=%d "
                                     "(fixed at 12 for this PCATM path)\n",
                                     std::atoi(qbEnv));
                    }
                    mesh->i_nbQuantBits = 12;
                    float maxRange = 0.0f;
                    for (int axis = 0; axis < 3; ++axis) {
                        maxRange = std::max(maxRange,
                                            mesh->bboxMax[axis] - mesh->bboxMin[axis]);
                    }
                    mesh->f_quantStep =
                        maxRange / static_cast<float>(1u << mesh->i_nbQuantBits);
                    std::fprintf(stderr,
                                 "[PCATM_STRICT] quantBits=%u quantStep=%g\n",
                                 mesh->i_nbQuantBits, mesh->f_quantStep);
                    const char* validateEnv =
                        std::getenv("PMSF_PCATM_STRICT_VALIDATE_QGRID");
                    if (validateEnv && std::strcmp(validateEnv, "0") == 0) {
                        std::fprintf(stderr,
                                     "[PCATM_STRICT][q-grid] validation "
                                     "disabled by "
                                     "PMSF_PCATM_STRICT_VALIDATE_QGRID=0\n");
                        return;
                    }
                    retri_override_detail::validatePCATMStrictQuantizedInput(
                        mesh, options);
                };
            cfg.preProcessPosition  = pmsf::CompressPipelineConfig::PreProcessPosition::Once;
            cfg.postProcessPosition = pmsf::CompressPipelineConfig::PostProcessPosition::EveryRound;
            break;
        }
        case RetriOverrideKind::PCATM_Color: {
            // CODEC_014 sv=3 协议，使用 PCATM baseline 的 PCATMEliminate
            // （不改 eliminate—— PCATMColorCollect 在 BFS 里按现有 RED/BLUE color
            // 贴面写入 colorBits 流，与 PCATM baseline 的 eliminate bit-exact）。
            cfg.collectorFactory = []() -> std::shared_ptr<SymbolCollectOperator> {
                return std::make_shared<PCATMColorSymbolCollectOperator>();
            };
            cfg.seedStrategy = pmsf::CompressPipelineConfig::SeedStrategy::SelectThenExtract;
            cfg.recomputeVertexDegreesOnce = true;
            cfg.trackContextLevel = true;
            cfg.enableIndependentSet = true;
            cfg.updateAfterEliminate = false;
            cfg.preProcessPosition  = pmsf::CompressPipelineConfig::PreProcessPosition::Once;
            cfg.postProcessPosition = pmsf::CompressPipelineConfig::PostProcessPosition::EveryRound;
            break;
        }
        case RetriOverrideKind::PCATM_ZColor: {
            // CODEC_017 sv=1 协议，eliminate 必须切到 PCATMZColorEliminate
            // —— 它是 PCATMEliminate 子类，额外修了 barycenter 为量化整数累加
            // 平均（与 decoder rdBarycenterQuantized bit-exact）。这是 Phase 4
            // PR-C 第 12 轮 bad_alloc 的修复；不这么做会在多轮后触发
            // self-intersection → bad_alloc。
            options.setEliminate(std::make_shared<PCATMZColorEliminateOperator>());
            cfg.collectorFactory = []() -> std::shared_ptr<SymbolCollectOperator> {
                return std::make_shared<PCATMZColorSymbolCollectOperator>();
            };
            cfg.seedStrategy = pmsf::CompressPipelineConfig::SeedStrategy::SelectThenExtract;
            cfg.recomputeVertexDegreesOnce = true;
            cfg.trackContextLevel = true;
            cfg.enableIndependentSet = true;
            cfg.updateAfterEliminate = false;
            cfg.preProcessPosition  = pmsf::CompressPipelineConfig::PreProcessPosition::Once;
            cfg.postProcessPosition = pmsf::CompressPipelineConfig::PostProcessPosition::EveryRound;
            break;
        }
        case RetriOverrideKind::PCATM_Paper: {
            // CODEC_020 sv=1：论文严格 Z-shape 路径（BlobLayout=20）。
            // encoder 用 PCATMPaperEliminateOperator 的 true Z-shape；decoder reader
            // 用当前 mesh 的 patch-tree candidates 替换父类 fan candidates，并给
            // polyFace 标记 protocolHint=20，由 reconstruct 的 strict-paper 分支
            // 合并 patch diagonals 后插点。
            options.setEliminate(std::make_shared<PCATMPaperEliminateOperator>());
            cfg.collectorFactory = []() -> std::shared_ptr<SymbolCollectOperator> {
                return std::make_shared<PCATMPMSFSymbolCollectOperator>();
            };
            cfg.seedStrategy = pmsf::CompressPipelineConfig::SeedStrategy::SelectThenExtract;
            cfg.recomputeVertexDegreesOnce = true;
            cfg.trackContextLevel = true;
            cfg.enableIndependentSet = true;
            cfg.updateAfterEliminate = false;
            cfg.preProcessPosition  = pmsf::CompressPipelineConfig::PreProcessPosition::Once;
            cfg.postProcessPosition = pmsf::CompressPipelineConfig::PostProcessPosition::EveryRound;
            break;
        }
        case RetriOverrideKind::PCATM_Pure: {
            // CODEC_021 sv=1：PR-K3 信息论最小集协议（BlobLayout=21）。
            // 3 段流（valence + isPolyBit + residual，砍 color）。encoder eliminate
            // 仍用 PCATMPaperEliminateOperator（true Z-shape + barycenter vc），
            // 但 collector 切换为 PCATMPureSymbolCollectOperator（继承 PCATMPaper，
            // 砍 color 段输出）。预期 connBPV ~ 7.13，与 RDTag 等价。
            options.setEliminate(std::make_shared<PCATMPaperEliminateOperator>());
            cfg.collectorFactory = []() -> std::shared_ptr<SymbolCollectOperator> {
                return std::make_shared<PCATMPureSymbolCollectOperator>();
            };
            cfg.seedStrategy = pmsf::CompressPipelineConfig::SeedStrategy::SelectThenExtract;
            cfg.recomputeVertexDegreesOnce = true;
            cfg.trackContextLevel = true;
            cfg.enableIndependentSet = true;
            cfg.updateAfterEliminate = false;
            cfg.preProcessPosition  = pmsf::CompressPipelineConfig::PreProcessPosition::Once;
            cfg.postProcessPosition = pmsf::CompressPipelineConfig::PostProcessPosition::EveryRound;
            cfg.preInitPreparation =
                [](std::shared_ptr<MCGAL::Mesh> mesh,
                   PMSFHeader&,
                   const CompressOptions&) {
                    if (!mesh) return;
                    int qb = 12;
                    if (const char* qbEnv = std::getenv("PMSF_PCATM_QUANT_BITS")) {
                        qb = std::atoi(qbEnv);
                    } else if (const char* qbEnv = std::getenv("PMSF_RDTAG_QUANT_BITS")) {
                        qb = std::atoi(qbEnv);
                    }
                    if (qb < 8 || qb > 16) {
                        std::fprintf(stderr,
                                     "[PCATM] ignore invalid quantBits=%d\n",
                                     qb);
                        return;
                    }
                    float maxRange = 0.0f;
                    for (int axis = 0; axis < 3; ++axis) {
                        maxRange = std::max(maxRange,
                                            mesh->bboxMax[axis] - mesh->bboxMin[axis]);
                    }
                    qb = retri_override_detail::raiseQuantBitsForVertexCollisions(
                        mesh, maxRange, qb, "PCATM");
                    mesh->i_nbQuantBits = static_cast<unsigned>(qb);
                    mesh->f_quantStep =
                        maxRange / static_cast<float>(1u << mesh->i_nbQuantBits);
                    std::fprintf(stderr,
                                 "[PCATM] quantBits=%u quantStep=%g\n",
                                 mesh->i_nbQuantBits, mesh->f_quantStep);
                };
            break;
        }
        case RetriOverrideKind::VQMMC_Codebook: {
            // CODEC_023 sv=1: PR-L5 协议（BlobLayout=23）。
            // 3 段流（valence + isPolyBit + residual），与 RDTag(18) 同构。
            options.setEliminate(std::make_shared<RDTagEliminateOperator>());
            options.setEnableAdjustment(false);
            cfg.collectorFactory = []() -> std::shared_ptr<SymbolCollectOperator> {
                return std::make_shared<VQMMCCodebookSymbolCollectOperator>();
            };
            cfg.trackContextLevel = true;
            cfg.updateAfterEliminate = false;
            break;
        }
        case RetriOverrideKind::MMSE_DualRing: {
            // CODEC_022：BlobLayout=22。默认 sv=3 使用
            // valence + isPolyBit + adaptive residual 三段流；实验开关
            // PMSF_MMSE_USE_MODE_DECISION=1 时才写 sv=2 的 mode 流。
            // PMSF_MMSE_USE_FIXED_DUAL_RING=1 可切换 sv=4，但高轮数
            // decoder 需要重算双环预测，当前仅作为诊断路径保留。
            if (std::getenv("PMSF_V2_COMPACT") == nullptr) {
                setenv("PMSF_V2_COMPACT", "1", /*overwrite=*/0);
            }
            if (std::getenv("PMSF_RDTAG_ADAPTIVE_RESIDUAL") == nullptr) {
                setenv("PMSF_RDTAG_ADAPTIVE_RESIDUAL", "1", /*overwrite=*/0);
            }
            options.setEliminate(std::make_shared<RDTagEliminateOperator>());
            options.setEnableAdjustment(false);
            cfg.collectorFactory = []() -> std::shared_ptr<SymbolCollectOperator> {
                return std::make_shared<MMSEDualRingSymbolCollectOperator>();
            };
            cfg.trackContextLevel = true;
            cfg.updateAfterEliminate = false;
            cfg.preInitPreparation =
                [](std::shared_ptr<MCGAL::Mesh> mesh,
                   PMSFHeader&,
                   const CompressOptions&) {
                    if (!mesh) return;
                    const bool explicitBits =
                        std::getenv("PMSF_MMSE_QUANT_BITS") != nullptr ||
                        std::getenv("PMSF_RDTAG_QUANT_BITS") != nullptr;
                    int qb = 10;
                    if (const char* qbEnv = std::getenv("PMSF_MMSE_QUANT_BITS")) {
                        qb = std::atoi(qbEnv);
                    } else if (const char* qbEnv = std::getenv("PMSF_RDTAG_QUANT_BITS")) {
                        qb = std::atoi(qbEnv);
                    }
                    if (qb < 8 || qb > 16) {
                        std::fprintf(stderr,
                                     "[MMSE] ignore invalid quantBits=%d\n",
                                     qb);
                        return;
                    }
                    float maxRange = 0.0f;
                    for (int axis = 0; axis < 3; ++axis) {
                        maxRange = std::max(maxRange,
                                            mesh->bboxMax[axis] - mesh->bboxMin[axis]);
                    }
                    // Correctness guard (same role as the framework's adaptive
                    // quant-bits guard for the standard pipeline): if distinct
                    // vertices collapse onto one quantized position, the
                    // dual-ring BFS tie-breaks and ring predictions diverge
                    // between encoder and decoder, and the decoded mesh becomes
                    // invalid (observed: mmse/columnar/LetterA at q10 =>
                    // test_valid=false, 111 degenerate faces).  Only when the
                    // bit count was NOT explicitly requested, raise it until
                    // every vertex has a unique quantized position (cap 16 =
                    // bit-plane codec limit).  Meshes already collision-free at
                    // q10 (bunny, fandisk) keep the exact baseline bytes.
                    if (!explicitBits && maxRange > 0.0f) {
                        auto countCollisions =
                            [&mesh, maxRange](int bits) -> std::size_t {
                            const float step =
                                maxRange / static_cast<float>(1u << bits);
                            const float bx = mesh->bboxMin[0];
                            const float by = mesh->bboxMin[1];
                            const float bz = mesh->bboxMin[2];
                            std::unordered_set<std::uint64_t> seen;
                            seen.reserve(mesh->vertices().size() * 2);
                            std::size_t collisions = 0;
                            for (MCGAL::Vertex* v : mesh->vertices()) {
                                if (v == nullptr || v->isRemoved()) continue;
                                const MCGAL::Point& p = v->point();
                                const long long qx = static_cast<long long>(
                                    std::floor((p.x() - bx) / step + 0.5f));
                                const long long qy = static_cast<long long>(
                                    std::floor((p.y() - by) / step + 0.5f));
                                const long long qz = static_cast<long long>(
                                    std::floor((p.z() - bz) / step + 0.5f));
                                const std::uint64_t kx =
                                    static_cast<std::uint64_t>(qx + (1LL << 20)) & 0x1FFFFFULL;
                                const std::uint64_t ky =
                                    static_cast<std::uint64_t>(qy + (1LL << 20)) & 0x1FFFFFULL;
                                const std::uint64_t kz =
                                    static_cast<std::uint64_t>(qz + (1LL << 20)) & 0x1FFFFFULL;
                                const std::uint64_t key = kx | (ky << 21) | (kz << 42);
                                if (!seen.insert(key).second) ++collisions;
                            }
                            return collisions;
                        };
                        int guardBits = qb;
                        while (guardBits < 16 && countCollisions(guardBits) > 0) {
                            ++guardBits;
                        }
                        if (guardBits != qb) {
                            std::fprintf(stderr,
                                         "[MMSE] quantBits raised %d -> %d to keep "
                                         "quantized vertices distinct (maxRange=%g)\n",
                                         qb, guardBits,
                                         static_cast<double>(maxRange));
                            qb = guardBits;
                        }
                    }
                    mesh->i_nbQuantBits = static_cast<unsigned>(qb);
                    mesh->f_quantStep =
                        maxRange / static_cast<float>(1u << mesh->i_nbQuantBits);
                    std::fprintf(stderr,
                                 "[MMSE] quantBits=%u quantStep=%g\n",
                                 mesh->i_nbQuantBits, mesh->f_quantStep);
                };
            break;
        }
        case RetriOverrideKind::None: /* already returned */ return false;
    }

    std::fprintf(stderr,
                 "[RetriOverride] preset=%s proto=%s → runCompress with override\n",
                 retriPresetName(preset), retriOverrideName(kind));

    pmsf::runCompress(options, cfg);
    return true;
}


inline bool tryRunCompressWithRetriOverride(CompressOptions& options,
                                            AlgoPreset preset) {
    const RetriOverrideKind kind = detectRetriOverride(preset);
    const RetriProtocolTuning tuning = tuningForPreset(preset);
    if (kind == RetriOverrideKind::None) return false;
    if (preset == AlgoPreset::QEM &&
        pmsf::boundaryFlagOn(qem_boundary::kPaperExactSpec)) {
        throw std::runtime_error(
            "[RetriOverride] QEM paper-exact replay requested, but the "
            "override protocol is an RD/PCLTTM bridge, not "
            "Garland-Heckbert edge-contraction / close-pair replay.");
    }
    if (preset == AlgoPreset::APTM) {
        if (pmsf::boundaryFlagOn(aptm_boundary::kPaperExactSpec)) {
            throw std::runtime_error(
                "[RetriOverride] APTM paper-exact replay requested, but the "
                "override protocols are still RD/PCLTTM bridges without "
                "APTM polar geometry or anchor-aware reconstruction.");
        }
    }
    if (preset == AlgoPreset::MMSE) {
        if (pmsf::boundaryFlagOn(pmsf::mmse_boundary::kPaperExactSpec)) {
            throw std::runtime_error(
                "[RetriOverride] MMSE paper-exact replay requested, but the "
                "override protocol is an RD/PCLTTM bridge unless the "
                "explicit MMSE dual-ring collector is selected and validated.");
        }
    }

    if (kind == RetriOverrideKind::PCATM_Strict) {
        if (std::getenv("PMSF_POOL_F") == nullptr) {
            setenv("PMSF_POOL_F", "500000", /*overwrite=*/0);
        }
        if (std::getenv("PMSF_POOL_H") == nullptr) {
            setenv("PMSF_POOL_H", "6000000", /*overwrite=*/0);
        }
    }
    if (kind == RetriOverrideKind::APTM_Conquest) {
        if (std::getenv("PMSF_POOL_V") == nullptr) {
            setenv("PMSF_POOL_V", "500000", /*overwrite=*/0);
        }
        if (std::getenv("PMSF_POOL_H") == nullptr) {
            setenv("PMSF_POOL_H", "10000000", /*overwrite=*/0);
        }
        if (std::getenv("PMSF_POOL_F") == nullptr) {
            setenv("PMSF_POOL_F", "500000", /*overwrite=*/0);
        }
    }

    // 旧 preset 入口只服务 columnar baseline 覆盖（paper 走 ById + 显式 backend）。
    return runRetriCompressOverride(options, kind, tuning, preset,
                                    pmsf::Backend::Columnar);
}

/// @brief Phase 1：按协议 id 触发压缩覆盖（取代按 AlgoPreset 枚举的分发）。
///
/// 协议分支本身已经是「按 kind 分发」的协议注册表；这里只把入口从 AlgoPreset
/// 换成字符串 id。`presetForLog` 传 Default 仅影响日志文字；脚本化的
/// paper-exact 守卫仍留在旧的 preset 入口上（那是诊断开关，不属于协议语义）。
inline bool tryRunCompressWithRetriOverrideById(CompressOptions& options,
                                                const std::string& protocolId,
                                                const RetriProtocolTuning& tuning,
                                                pmsf::Backend backend) {
    // M4-P0：paper 后端协议覆盖是 opt-in；默认关闭时 paper 产物逐字节不变。
    if (backend == pmsf::Backend::Paper && !pmsf::paperRetriEnabled() && tuning.paperArtifactAlgorithm.empty()) {
        return false;
    }
    const RetriOverrideKind kind = retriOverrideKindFromId(protocolId);
    if (kind == RetriOverrideKind::None) return false;
    return runRetriCompressOverride(options, kind, tuning, AlgoPreset::Default, backend);
}
/// @brief 解压覆盖的实际执行体。
/// @brief 某个协议的解码端算子对：reader + reconstructor。
///
/// 这是「整段接管」与「解压器工厂」共用的**唯一**一份选择逻辑。两边各写一份是
/// 实测踩过的坑：漏掉 RDR1 属性附件时 MMSE 直接断言失败，漏掉某个 case 时查询
/// 路径静默退回默认三元组、解出的是 base mesh。
/// @return kind == None 时 false。
inline bool makeRetriReaderPair(RetriOverrideKind kind,
                                std::shared_ptr<SymbolReadOperator>& reader,
                                std::shared_ptr<ReconstructOperator>& reconstructor) {
    if (kind == RetriOverrideKind::None) {
        return false;
    }
    // 池尺寸：查询路径在 `ensureMeshRuntimeInitialized()` **之前**调用本函数，
    // 所以在这里设是有效的；整段接管路径里 `tryRunDecompressWithRetriOverride`
    // 已经先设过一遍，这里靠 overwrite=0 保持幂等。
    if (kind == RetriOverrideKind::PCATM_Strict) {
        if (std::getenv("PMSF_POOL_F") == nullptr) {
            setenv("PMSF_POOL_F", "500000", /*overwrite=*/0);
        }
        if (std::getenv("PMSF_POOL_H") == nullptr) {
            setenv("PMSF_POOL_H", "6000000", /*overwrite=*/0);
        }
    }
    if (kind == RetriOverrideKind::APTM_Conquest) {
        if (std::getenv("PMSF_POOL_V") == nullptr) {
            setenv("PMSF_POOL_V", "500000", /*overwrite=*/0);
        }
        if (std::getenv("PMSF_POOL_H") == nullptr) {
            setenv("PMSF_POOL_H", "10000000", /*overwrite=*/0);
        }
        if (std::getenv("PMSF_POOL_F") == nullptr) {
            setenv("PMSF_POOL_F", "500000", /*overwrite=*/0);
        }
    }
    switch (kind) {
        case RetriOverrideKind::RD_Tag: {
            const char* rdCanonEnv = std::getenv("PMSF_RD_CANON");
            const bool rdCanon = (rdCanonEnv != nullptr && *rdCanonEnv != '\0' &&
                                  std::strcmp(rdCanonEnv, "0") != 0);
            if (rdCanon) {
                auto canonReader = std::make_shared<
                    rd_canon::RDCanonicalSymbolReadOperator>();
                reader        = canonReader;
                reconstructor = std::make_shared<
                    rd_canon::RDCanonicalReconstructOperator>(canonReader);
                break;
            }
            if (std::getenv("PMSF_RECON_SKIP_BOTH_SPLITTABLE") == nullptr) {
                setenv("PMSF_RECON_SKIP_BOTH_SPLITTABLE", "1", /*overwrite=*/0);
            }
            reader        = std::make_shared<RDTagSymbolReadOperator>();
            reconstructor = std::make_shared<BasicReconstructOperator>();
            break;
        }
        case RetriOverrideKind::PCLTTM_Tag: {
            auto typedReader = std::make_shared<PCLTTMTagSymbolReadOperator>();
            reader = typedReader;
            reconstructor = std::make_shared<PCLTTMTagReconstructOperator>(typedReader);
            break;
        }
        case RetriOverrideKind::PCATM_Color:
            reader        = std::make_shared<PCATMColorSymbolReadOperator>();
            reconstructor = std::make_shared<BasicReconstructOperator>();
            break;
        case RetriOverrideKind::PCATM_ZColor:
            reader        = std::make_shared<PCATMZColorSymbolReadOperator>();
            reconstructor = std::make_shared<BasicReconstructOperator>();
            break;
        case RetriOverrideKind::PCATM_Paper:
            // CODEC_020(20)：4 段流与 ZColor 同构，但 collectFromDecoded 会
            // 替换 actual patch-tree candidates 并标记 protocolHint=20。
            reader        = std::make_shared<PCATMPMSFSymbolReadOperator>();
            reconstructor = std::make_shared<BasicReconstructOperator>();
            break;
        case RetriOverrideKind::PCATM_Pure:
            // CODEC_021(21)：3 段流与 RDTag 同构，但 collectFromDecoded 会
            // 替换 actual patch-tree candidates 并标记 protocolHint=20。
            reader        = std::make_shared<PCATMPureSymbolReadOperator>();
            reconstructor = std::make_shared<BasicReconstructOperator>();
            break;
        case RetriOverrideKind::VQMMC_Codebook:
            // PR-L5 落地（2026-05-12）：CODEC_023(23) decoder 实装。
            reader        = std::make_shared<VQMMCCodebookSymbolReadOperator>();
            reconstructor = std::make_shared<BasicReconstructOperator>();
            break;
        case RetriOverrideKind::MMSE_DualRing:
            if (std::getenv("PMSF_RECON_SKIP_BOTH_SPLITTABLE") == nullptr) {
                setenv("PMSF_RECON_SKIP_BOTH_SPLITTABLE", "1", /*overwrite=*/0);
            }
            // PR-L4b 落地（2026-05-12）：CODEC_022(22) decoder 实装。
            // MMSEDualRingSymbolReadOperator 继承 RDTagSymbolReadOperator，
            // 3 段流字节布局相同，仅 blob_layout 校验 18→22。
            reader        = std::make_shared<MMSEDualRingSymbolReadOperator>();
            reconstructor = std::make_shared<BasicReconstructOperator>();
            break;
        case RetriOverrideKind::APTM_Conquest:
            if (std::getenv("PMSF_PCLTTM_SINGLE_SEED") == nullptr) {
                setenv("PMSF_PCLTTM_SINGLE_SEED", "1", /*overwrite=*/0);
            }
            if (std::getenv("PMSF_PCLTTM_STRICT_EXACT_PATCH") == nullptr &&
                std::getenv("PMSF_PCLTTM_STRICT_ENABLE_LOOSE_PATCH") == nullptr) {
                setenv("PMSF_PCLTTM_STRICT_EXACT_PATCH", "1", /*overwrite=*/0);
                setenv("PMSF_PCLTTM_STRICT_ENABLE_LOOSE_PATCH", "0", /*overwrite=*/0);
            }
            if (std::getenv("PMSF_PCLTTM_AVOID_DUP_Q") == nullptr) {
                setenv("PMSF_PCLTTM_AVOID_DUP_Q", "1", /*overwrite=*/0);
            }
            reader        = std::make_shared<pmsf::PCLTTMStrictSymbolReadOperator>();
            reconstructor = std::make_shared<BasicReconstructOperator>();
            break;
        case RetriOverrideKind::PCATM_Strict:
            if (std::getenv("PMSF_PCATM_STRICT_FACE_ORDER") == nullptr &&
                std::getenv("PMSF_PCATM_STRICT_MORTON_FACE_ORDER") == nullptr &&
                std::getenv("PMSF_PCATM_STRICT_STABLE_FACE_ORDER") == nullptr) {
                setenv("PMSF_PCATM_STRICT_STABLE_FACE_ORDER", "1", /*overwrite=*/0);
            }
            reader        = std::make_shared<PCATMStrictPMSFSymbolReadOperator>();
            reconstructor = std::make_shared<PCATMReconstructOperator>();
            break;
        case RetriOverrideKind::None:
            break;
    }
    return true;
}

inline bool runRetriDecompressOverride(RetriOverrideKind kind,
                                       AlgoPreset presetForLog,
                                       const std::string& inputPath,
                                       const std::string& outputPath) {
    const AlgoPreset preset = presetForLog;
    std::shared_ptr<SymbolReadOperator>   reader;
    std::shared_ptr<ReconstructOperator>  reconstructor;

    if (!makeRetriReaderPair(kind, reader, reconstructor)) {
        return false;
    }

    // 与 Compress 端对称：池注册是 baseline `DecompressPipeline::Builder().run()`
    // 内部隐式完成的步骤。override 旁路了它，必须显式做 —— 且要**按产物里的
    // 原始网格规模**注册：先 init 读 compact header 的 origTotals（纯文件读，
    // 不碰池），再按标定乘数注册（runDecompress 里那次注册因幂等跳过），
    // 最后才 attach 顶点属性（attach 要求池已就位，否则被 ContextPool 静默
    // 丢弃，RD 系 predictor 取属性时撞断言）。
    auto deserializer = std::make_shared<ColumnarDeserializeOperator>(reader, reconstructor);
    deserializer->init(inputPath);
    int origV = 0, origF = 0;
    deserializer->originalMeshCounts(origV, origF);
    if (origV > 0 && origF > 0) {
        pmsf::ensureMeshRuntimeInitializedForMeshSize(origV, origF);
    } else {
        pmsf::ensureMeshRuntimeInitialized();
    }

    // 解码端对称 attach kVPropRDR1（与 Compress 端 PMSF 构造 + attach 顺序一致）。
    MCGAL::contextPool.attachVertexProperty<MCGAL::PointInt>(
        DEFAULT_MESH_ID, MCGAL::kVPropRDR1);

    DecompressOptions options = DecompressOptions::Builder()
                                    .setDeserializeOperator(deserializer)
                                    .setPath(inputPath)
                                    .setOutputPath(outputPath)
                                    .build();

    std::fprintf(stderr,
                 "[RetriOverride] preset=%s proto=%s → runDecompress with override\n",
                 retriPresetName(preset), retriOverrideName(kind));

    pmsf::runDecompress(options);
    return true;
}

// ---------------------------------------------------------------------------
// 解压器工厂：与上面的「整段接管」共用同一段 reader/reconstructor 与**同一套
// 解码上下文副作用**，但只交出解压器对象 —— 解码循环仍由框架跑。随机访问查询走
// 这条路，才能既拿到与算法自己解压时逐字节相同的 reader，又能让框架按 max_lod
// 截断轮数。
// ---------------------------------------------------------------------------

/// @brief 某个协议在解码前必须施加的上下文副作用。
///
/// 与 `runRetriDecompressOverride` 里那份**逐字对应**：进程级环境变量在
/// `makeRetriReaderPair` 里按 kind 设（那是两条路径共用的唯一一份），网格属性附件
/// 放在这里 —— 它要求 ContextPool 已经初始化，只能挂在对象的 `init()` 之后。
///
/// 两边不一致的后果是实测过的：漏掉 RDR1 附件时 MMSE 的查询路径直接断言失败。
/// `attachVertexProperty` 自身幂等（同名同类型属性是 no-op），所以重复调用安全。
inline void applyRetriDecodeSideEffects(RetriOverrideKind /*kind*/) {
    MCGAL::contextPool.attachVertexProperty<MCGAL::PointInt>(DEFAULT_MESH_ID,
                                                             MCGAL::kVPropRDR1);
}

/// @brief 按产物自带的 blob_layout 决定 reader/reconstructor 的列式解压器。
///
/// 两条来源，优先级从高到低：
///   1. 算法声明的协议（`protocol` 解析出的 kind）—— 显式声明就该照办；
///   2. 产物 metadata 里的 `blob_layout` —— 自描述兜底。协议字符串可能取决于
///      环境变量，产物自己写下的 layout 不会。
/// 两者都定不出来（layout 是 PPMC/Default 这类非协议布局）就沿用算法注册表的
/// 默认算子对，行为与改造前一致。
///
/// 换 reader 必须等 `init()`：blob_layout 是 init 从 metadata 里读出来的。而
/// `deserializeBaseMesh()` 在 init 之后才调用，所以此处的替换是及时的。
class RetriContextDeserializer : public ColumnarDeserializeOperator {
  public:
    RetriContextDeserializer(std::shared_ptr<SymbolReadOperator> reader,
                             std::shared_ptr<ReconstructOperator> reconstructor,
                             RetriOverrideKind kind,
                             std::shared_ptr<SymbolReadOperator> fallbackReader = nullptr,
                             std::shared_ptr<ReconstructOperator> fallbackReconstructor = nullptr)
        : ColumnarDeserializeOperator(std::move(reader), std::move(reconstructor)),
          kind_(kind),
          fallbackReader_(std::move(fallbackReader)),
          fallbackReconstructor_(std::move(fallbackReconstructor)) {}

    void init(std::string path) override {
        ColumnarDeserializeOperator::init(std::move(path));

        RetriOverrideKind effective = kind_;
        if (effective == RetriOverrideKind::None) {
            // 协议没声明 —— 问产物。canonical 包装布局下真正的 layout 记在
            // canonical_source_layout。
            const int layout = (blobLayout_ == pmsf::kPaperBlobLayoutCanonical &&
                                canonicalSourceLayout_ != 0)
                                   ? canonicalSourceLayout_
                                   : blobLayout_;
            effective = retriOverrideKindFromLayout(layout);
        }

        if (effective != RetriOverrideKind::None) {
            std::shared_ptr<SymbolReadOperator> reader;
            std::shared_ptr<ReconstructOperator> reconstructor;
            if (makeRetriReaderPair(effective, reader, reconstructor)) {
                symbolReadOperator_   = std::move(reader);
                reconstructOperator_  = std::move(reconstructor);
                applyRetriDecodeSideEffects(effective);
                // runDecompress 把池注册推迟到 init() 之后（按产物里的原始
                // 网格规模按需申请）⇒ 此时池可能尚未注册，上面的 attach 会被
                // ContextPool 静默丢弃（meshId 越界直接 return）。记下来，
                // 在 deserializeBaseMesh()（池必然已就位）再补一次幂等 attach。
                needsDeferredSideEffects_ = true;
                return;
            }
        }
        // 定不出来：保持构造时由调用方给出的算子对。
        (void)fallbackReader_;
        (void)fallbackReconstructor_;
    }

    std::shared_ptr<MCGAL::Mesh> deserializeBaseMesh() override {
        // 幂等补挂（同名同类型 attach 是 no-op）。漏挂的实测后果：RD 系
        // curvature predictor 在 deserializeOneRound 里取 kVPropRDR1 直接
        // 撞 OpenMesh PropertyContainer 断言。
        if (needsDeferredSideEffects_) {
            applyRetriDecodeSideEffects(kind_);
        }
        return ColumnarDeserializeOperator::deserializeBaseMesh();
    }

  private:
    RetriOverrideKind kind_;
    bool needsDeferredSideEffects_ = false;
    std::shared_ptr<SymbolReadOperator>  fallbackReader_;
    std::shared_ptr<ReconstructOperator> fallbackReconstructor_;
};

/// @brief M4-P0：按协议 id 解析原生 reader/reconstructor 对（Paper 工厂用）。
///        false = 协议为空/未知；true = reader/reconstructor 已就绪。
inline bool makeRetriReaderPairForProtocol(
    const std::string& protocol, std::shared_ptr<SymbolReadOperator>& reader,
    std::shared_ptr<ReconstructOperator>& reconstructor) {
    const RetriOverrideKind kind = retriOverrideKindFromId(protocol);
    if (kind == RetriOverrideKind::None) return false;
    return makeRetriReaderPair(kind, reader, reconstructor);
}

inline std::shared_ptr<DeserializeOperator> makeRetriDeserializerForProtocol(
    const std::string& protocol, std::shared_ptr<SymbolReadOperator> defaultReader,
    std::shared_ptr<ReconstructOperator> defaultReconstructor) {
    const RetriOverrideKind kind = retriOverrideKindFromId(protocol);

    std::shared_ptr<SymbolReadOperator>  reader;
    std::shared_ptr<ReconstructOperator> reconstructor;
    if (kind != RetriOverrideKind::None) {
        makeRetriReaderPair(kind, reader, reconstructor);
    } else {
        // 协议为空 / 未知：先用调用方给的默认算子对构造，等 init() 读到产物的
        // blob_layout 再决定要不要换。
        reader        = std::move(defaultReader);
        reconstructor = std::move(defaultReconstructor);
    }
    return std::make_shared<RetriContextDeserializer>(reader, reconstructor, kind);
}



// ---------------------------------------------------------------------------
// Decompress 覆盖：与 Compress 配对。构造匹配的 reader + reconstructor +
// ColumnarDeserializeOperator，然后跑 `runDecompress(options)`，返回 true。
//
// 调用约定：baseline decompress main 在 DecompressOptions 构造好 input/output
// 路径后，先调本函数；返回 true 则 main 立刻 `return 0`。
//
// 输入约定（与 baseline main 对齐）：
//   - `inputPath` / `outputPath` 由 main 从命令行解析得来
//   - preset 同 compress 端
// ---------------------------------------------------------------------------
inline bool tryRunDecompressWithRetriOverride(AlgoPreset preset,
                                              const std::string& inputPath,
                                              const std::string& outputPath) {
    const RetriOverrideKind kind = detectRetriOverride(preset);
    if (kind == RetriOverrideKind::None) return false;

    if (kind == RetriOverrideKind::PCATM_Strict) {
        if (std::getenv("PMSF_POOL_F") == nullptr) {
            setenv("PMSF_POOL_F", "500000", /*overwrite=*/0);
        }
        if (std::getenv("PMSF_POOL_H") == nullptr) {
            setenv("PMSF_POOL_H", "6000000", /*overwrite=*/0);
        }
    }
    if (kind == RetriOverrideKind::APTM_Conquest) {
        if (std::getenv("PMSF_POOL_V") == nullptr) {
            setenv("PMSF_POOL_V", "500000", /*overwrite=*/0);
        }
        if (std::getenv("PMSF_POOL_H") == nullptr) {
            setenv("PMSF_POOL_H", "10000000", /*overwrite=*/0);
        }
        if (std::getenv("PMSF_POOL_F") == nullptr) {
            setenv("PMSF_POOL_F", "500000", /*overwrite=*/0);
        }
    }

    return runRetriDecompressOverride(kind, preset, inputPath, outputPath);
}

/// @brief Phase 1：按协议 id 触发解压覆盖（与 ById 的压缩端配对）。
inline bool tryRunDecompressWithRetriOverrideById(const std::string& protocolId,
                                                  const std::string& inputPath,
                                                  const std::string& outputPath) {
    const RetriOverrideKind kind = retriOverrideKindFromId(protocolId);
    if (kind == RetriOverrideKind::None) return false;
    return runRetriDecompressOverride(kind, AlgoPreset::Default, inputPath, outputPath);
}

#else
inline bool tryRunCompressWithRetriOverride(CompressOptions& /*options*/,
                                            AlgoPreset /*preset*/) {
    return false;
}

inline bool tryRunDecompressWithRetriOverride(AlgoPreset /*preset*/,
                                              const std::string& /*inputPath*/,
                                              const std::string& /*outputPath*/) {
    return false;
}
#endif

// ---------------------------------------------------------------------------
// Phase 1：把协议覆盖实现注册进 src/ 的接缝（依赖倒置）。
// src/ 的 `ISimplificationAlgorithm::compress()/decompress()` 按协议 id 调它，
// 谁也没有硬编码算法名。
// ---------------------------------------------------------------------------
#ifdef COLUMNAR_SUPPORT
namespace {
const bool g_retriOverridesRegistered = [] {
    pmsf::registerRetriCompressOverride(
        [](CompressOptions& options, const std::string& protocol,
           const pmsf::RetriProtocolTuning& tuning, pmsf::Backend backend) {
            return pmsf::tryRunCompressWithRetriOverrideById(options, protocol, tuning,
                                                             backend);
        });
    pmsf::registerRetriDecompressOverride(
        [](const std::string& protocol, const std::string& inputPath,
           const std::string& outputPath) {
            return pmsf::tryRunDecompressWithRetriOverrideById(protocol, inputPath,
                                                              outputPath);
        });
    pmsf::registerRetriDeserializerFactory(
        [](const std::string& protocol,
           std::shared_ptr<SymbolReadOperator> defaultReader,
           std::shared_ptr<ReconstructOperator> defaultReconstructor)
            -> std::shared_ptr<DeserializeOperator> {
            return makeRetriDeserializerForProtocol(protocol, std::move(defaultReader),
                                                    std::move(defaultReconstructor));
        });
    pmsf::registerRetriReaderPairFactory(
        [](const std::string& protocol, std::shared_ptr<SymbolReadOperator>& reader,
           std::shared_ptr<ReconstructOperator>& reconstructor) {
            return pmsf::makeRetriReaderPairForProtocol(protocol, reader, reconstructor);
        });
    pmsf::registerRetriDecodeSideEffects(
        [](const std::string& protocol) {
            const RetriOverrideKind kind = retriOverrideKindFromId(protocol);
            if (kind == RetriOverrideKind::None) return false;
            applyRetriDecodeSideEffects(kind);
            return true;
        });
    return true;
}();
}  // namespace
#endif  // COLUMNAR_SUPPORT

}  // namespace pmsf

#endif  // PMSF_CORE_RETRI_OVERRIDE_H
