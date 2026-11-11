// SPDX-License-Identifier: Apache-2.0
//
// APTMTagSymbolCollectOperator.h — APTM (Cheng, Jin, Liu, Pacific Graphics 2006)
// P1: anchors-aware A&D/tag connectivity + APTM geometry prototype.
//
// 论文路径拆解（见 impl/APTM/AUDIT_2026-10-22.md §4）：
//   P1 = 换用 A&D-style patch-conquest 底部（PCLTTM Tag 的 valence + fusedFaceSym
//        连通性符号）+ 论文 §4.2 几何 coder 的**原型**（Frenet 局部系 / polar θφ）；
//   P2 = polar 完整版（θ/φ 3-bit + 质量口径登记 + 编解码对称化）；
//   P3 = 真 A&D valence-driven connectivity（去掉 fusedFaceSym）。
//
// 本类只做「继承 + 覆写」，不复制父类代码：
//   - `collect(seed, gates)`：先调父类（valence / fusedFaceSym / residual 队列），
//     再在队尾把 residual 从 Cartesian 残差变换进几何 coder 的编码域：
//       * `frenet` 模式：父类在 `PMSF_RDTAG_FRENET_RESIDUAL=1` 下已经把它换成
//         可逆整数 lifting 的 Frenet 分量 (c_n, c_t, c_b)；
//       * `polar`  模式：再把 (c_n,c_t,c_b) 量化成 (R, θq, φq)（θ/φ 3-bit，
//         论文 §4.2），并**重放本轮预测链**让 encoder 的 r1 状态与 decoder 一致。
//   - `exportToBufferWithRangeCoder(...)`：几何模式 != cartesian 时临时摘掉
//     RD 的 Frenet 环境变量（父类 layout-16 export 会拒绝写 Frenet 流）、调父类
//     export 写 3 段流，然后把容器 scheme_version 从 3 改成 APTM 自己的
//     sv=64(frenet) / 65(polar)，使 decoder 侧能识别并走 APTM 的 reader。
//
// 兼容性契约：未设 `PMSF_APTM_USE_APTM_TAG=1` 时本类根本不实例化
// （APTMAlgorithm::collectorFactory() 返回空），默认 APTM 路径
// （84,496 B / 34.052 bpv）**逐字节不变**。
//
// 几何模式（`PMSF_APTM_TAG_GEOM`）：
//   - 未设 / "cartesian"：纯 tag 连通性 + tag cartesian residual（P1 阶段一基线）；
//   - "frenet"：Frenet 局部系（可逆整数 lifting，无损）—— 论文 §4.1 的
//                "local approximate Frenet coordinate frame"；
//   - "polar" ：θ/φ 各 3-bit 的 polar 量化（论文 §4.2，**有损**——论文自己的
//                3-bit tangential 也是有损，质量口径见 P1 报告）。
//
// 与 PCLTTM 的差异（= APTM 论文贡献）：
//   - anchor 由 `AnchorPreprocessor` 每轮重标为不可删（APTM §3），因此 base mesh
//     更大、tag conquest 的 null_patch 更多（论文 §4.1 明确）；
//   - 几何 coder 用 Frenet/polar（论文 §4.2），不是 PCLTTM 的 Cartesian bit-plane。

#ifndef PMSF_APTM_COMPRESS_APTMTAG_SYMBOL_COLLECT_OPERATOR_H
#define PMSF_APTM_COMPRESS_APTMTAG_SYMBOL_COLLECT_OPERATOR_H

#include "APTMTagSchemeVersion.h"
#include "PCLTTM/compress/PCLTTMTagSymbolCollectOperator.h"

#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

namespace pmsf {
namespace aptm_tag {

/// @brief `PMSF_APTM_USE_APTM_TAG=1` 时启用本 collector（APTMAlgorithm 判定）。
inline bool aptmTagPathEnabled() {
    const char* v = std::getenv("PMSF_APTM_USE_APTM_TAG");
    return v && *v && std::string(v) != "0";
}

/// @brief 几何 coder 模式（`PMSF_APTM_TAG_GEOM`）。
enum class GeometryMode {
    Cartesian = 0,  ///< tag 原生 Cartesian bit-plane residual（P1 阶段一基线）
    Frenet = 1,     ///< 可逆整数 lifting 的 Frenet 分量（无损；论文 §4.1 frame）
    Polar = 2,      ///< Frenet + θ/φ 3-bit polar 量化（论文 §4.2；有损）
};

inline GeometryMode resolveGeometryMode() {
    const char* v = std::getenv("PMSF_APTM_TAG_GEOM");
    if (!v || !*v) return GeometryMode::Cartesian;
    const std::string mode(v);
    if (mode == "frenet") return GeometryMode::Frenet;
    if (mode == "polar") return GeometryMode::Polar;
    if (mode == "cartesian") return GeometryMode::Cartesian;
    return GeometryMode::Cartesian;
}

}  // namespace aptm_tag

/// APTM Tag collector
///   - cartesian 模式：完全继承父类（tag 连通性 + tag Cartesian residual）；
///   - frenet / polar 模式：覆盖 collect/export（见文件头注释）。
class APTMTagSymbolCollectOperator final : public PCLTTMTagSymbolCollectOperator {
  public:
    APTMTagSymbolCollectOperator() = default;

    void collect(MCGAL::Halfedge* seed,
                 const std::vector<MCGAL::Halfedge*>& gates) override;

    int exportToBufferWithRangeCoder(char* buffer,
                                     bool enableQuantization = false) override;

    /// 诊断探针（P1 证据脚本读）：最近一次 export 的 3 段流字节。
    int lastValenceStreamBytes() const { return lastValenceBytes_; }
    int lastFusedFaceSymStreamBytes() const { return lastFusedBytes_; }
    int lastResidualStreamBytes() const { return lastResidualBytes_; }
    int lastContainerOverheadBytes() const { return lastContainerBytes_; }
    int lastSchemeVersion() const { return lastSchemeVersion_; }

  private:
    /// 把本轮 residual 队列从 Frenet 分量 (c_n,c_t,c_b) 量化成 (R,θq,φq)，
    /// 并重放本轮预测链写回 decoder 一致的 r1 状态（见 .cpp 注释）。
    void quantizePolarQueues();

    /// P2-pre 诊断：polyFace valence 分布 + 「v0 索引」side-stream 代价估计。
    /// 拓扑推断去坐标化若走「序列化 v0 环上索引」的兜底方案，代价 ≈
    /// Σ_face log2(valence) bits；同时给出 valence=3（无 fan 对角线、
    /// 纯拓扑无法定 v0）的占比 —— 这是 P2-pre 设计的两个关键输入。
    void emitValenceCostDiag();

    int lastValenceBytes_ = 0;
    int lastFusedBytes_ = 0;
    int lastResidualBytes_ = 0;
    int lastContainerBytes_ = 0;
    int lastSchemeVersion_ = 3;
};

}  // namespace pmsf

#endif  // PMSF_APTM_COMPRESS_APTMTAG_SYMBOL_COLLECT_OPERATOR_H
