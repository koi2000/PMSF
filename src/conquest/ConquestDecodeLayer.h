/*
 * @file ConquestDecodeLayer.h
 * @brief 前沿式删减的**解码端镜像：一层**（框架原生，**算法无关**）。
 *
 * 与 `FrontierConquestEngine::decimateLayer` 严格互逆：编码端每删一个前端点就
 * 发 `valence - 3` 个符号（或 `4` = null patch），解码端按**同一条 BFS**重放，
 * 遇到删除符号就把区域并回多边形、插回中心点、按量化残差还原位置。
 *
 * **验证状态（2026-09-22）**：与编码端逐层对拍通过 —— 4 个网格
 * （cow_simp 148/292、fandisk 6475/12946、cow 2904/5804、bimba 8857/17710）
 * × 8 层链式（V 148→114→81→58→45→35→27→19），全部 `topoGeomOk=1`
 * （规范面逐格全等 + 顶点数/面数精确 + 存活的原始顶点逐位精确）。
 * 证据与推理过程见 `docs/refactor/P13_4_DECODER_BLUEPRINT_2026-09-22.md`。
 *
 * **本文件不认识任何具体算法名**（lint 纪律）：可调之处一律走
 * `scheme/EngineHooks.h` 的引擎级 hook，**必须与编码端同一套**。
 */
#ifndef PMSF_CONQUEST_CONQUEST_DECODE_LAYER_H
#define PMSF_CONQUEST_CONQUEST_DECODE_LAYER_H

#include "conquest/FrontierConquestEngine.h"

#include <array>
#include <cstddef>
#include <cstdio>
#include <vector>

namespace pmsf {

struct ConquestDecodeControl;

/// @brief 框架默认 hook（五项全留空 ⇒ 回落原始论文行为）。
///
/// 与 `FrontierConquestEngine::hooksOrDefault()` 是**同一个语义**；
/// 单列出来是为了让解码端在"没注入"时能拿到同一个常量引用。
inline const EngineHooks& defaultEngineHooks() {
    static const EngineHooks kDefault;
    return kDefault;
}

/// @brief 解码一层的结果。
struct ConquestDecodeResult {
    std::size_t expandedSymbols = 0;
    std::size_t inferredNulls = 0;
    int inserted = 0;          ///< 插回的顶点数（= 本层删掉的顶点数）
    int nullPatch = 0;         ///< null-patch 次数
    std::size_t consumed = 0;  ///< 实际消费的连通性符号数
    /// @brief 插回点的环大小与 valence 不符的次数（正常恒为 0）。
    int ringMismatch = 0;
    /// @brief **诊断**：`ringOf`（纯 ccw）与编码端 `ringNeighbors`（先 cw 后 ccw）
    /// 给出的环**不同**的次数 —— 蓝图 §79.5 记下的已知隐患，这里量它。
    int ringRuleDiff = 0;
    /// @brief **诊断**：两者给出的邻居算术平均之差的最大值（单位：世界坐标）。
    double ringMeanDiffMax = 0.0;
    /// @brief 是否正常走到符号耗尽。`false` = 中途碰到不可恢复的拓扑错误。
    bool ok = true;
};

/// Deferred geometry record produced by the connectivity reconstruction layer.
/// The caller can wait for the entire layer topology before choosing predictors
/// or connectivity-derived partitions. Positions are never used by this record.
struct ConquestDeferredGeometry {
    MCGAL::Vertex* center = nullptr;
    std::vector<MCGAL::Vertex*> ring;
    std::array<int, 3> residual{};
};

inline void foldConquestGeometryCell(std::uint64_t& hash, std::int32_t cell) {
    const auto value = static_cast<std::uint32_t>(cell);
    for (int byte = 0; byte < 4; ++byte) { hash ^= (value >> (8 * byte)) & 255u; hash *= 1099511628211ull; }
}
inline std::uint64_t conquestGeometryCellSignature(const std::vector<std::array<int, 3>>& values) {
    std::uint64_t hash = 1469598103934665603ull;
    for (const auto& value : values) for (int cell : value) foldConquestGeometryCell(hash, cell);
    return hash;
}
struct ConquestGeometryRestoreResult {
    std::uint64_t cellSignature = 1469598103934665603ull;
    std::size_t negativePredictions = 0;
};

/// Geometry stage for records returned by topology-only decodeLayer. The
/// algorithm installs its layer-wide predictor/partition context beforehand;
/// quantization, inverse residual transform and vertex placement stay here.
/// Topology inverse of the degree-three cleaning sweep. Geometry remains in
/// deferred records so the same predictor/quantizer restoration layer is used.
ConquestDecodeResult decodeCleaningLayer(MCGAL::Mesh* mesh, ConquestState& state,
    MCGAL::Halfedge* seed, const std::vector<int>& symbols,
    const std::vector<std::array<int,3>>& residuals,
    std::vector<ConquestDeferredGeometry>& geometry);

ConquestGeometryRestoreResult restoreDeferredConquestGeometry(const std::vector<ConquestDeferredGeometry>& records,
                                     const FrontierConquestEngine::QuantGrid& grid,
                                     const EngineHooks& hooks = defaultEngineHooks());

/// @brief 解码**一层**。与 `FrontierConquestEngine::decimateLayer` 互逆。
///
/// @param mesh   目标网格（编码端的 base mesh，就地还原）
/// @param st     引擎状态（**本函数会 `reset()`**，与编码端每次进层一致）
/// @param seed   层种子半边（随码流，与编码端同一个）
/// @param symbols 本层的连通性符号流：`0..3` = 删除（valence = s + 3），`4` = null
/// @param deltas 每次插回点的**量化整数残差**（按插入顺序）
/// @param grid   量化网格（包围盒下界 + 步长），必须与编码端同一套
/// @param hooks  引擎级 hook，**必须与编码端同一份实例**（默认用框架默认值）
ConquestDecodeResult decodeLayer(MCGAL::Mesh* mesh,
                                 ConquestState& st,
                                 MCGAL::Halfedge* seed,
                                 const std::vector<int>& symbols,
                                 const std::vector<std::array<int, 3>>& deltas,
                                 const FrontierConquestEngine::QuantGrid& grid,
                                 const EngineHooks& hooks = defaultEngineHooks(),
                                 /// 诊断用（默认 nullptr，零默认影响）：逐插点写 `toCell(pred)`。
                                 /// **由调用方开文件并传入**（`src/` 不读环境变量、不自己开文件）。
                                 std::FILE* predTrace = nullptr,
                                 std::vector<ConquestDeferredGeometry>* deferredGeometry = nullptr,
                                 ConquestDecodeControl* control = nullptr);

}  // namespace pmsf

#endif  // PMSF_CONQUEST_CONQUEST_DECODE_LAYER_H
