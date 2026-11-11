/*
 * @file EngineHooks.h
 * @brief 前沿式顶点删减引擎的**引擎级 hook 接口**（StorageScheme 的第五组组件，S5）。
 *
 * 设计依据：`docs/MASTER_PLAN_2026-09-17.md` §4 P13 行「抽象（接入成本最小化）」。
 *
 * 背景：框架里已有 S1–S4 四个**流级**组件（见 `scheme/StorageScheme.h`）。
 * 它们描述"发什么符号、怎么熵编码、怎么装段"。但有一类优化**不在这四个维度上**：
 * 它们改的是**遍历与预测本身** —— 谁先被处理、哪些点可删、位置怎么预测、
 * 残差怎么量化、符号发出去之后还做什么变换。这些是**引擎级**的，故单列 S5。
 *
 * 与 S1–S4 完全同一模式：**五个 hook 全部留空 ⇒ 回落框架默认**，
 * 而默认值就是原始论文行为 ⇒ **默认路径逐字节不变**。
 *
 * | 组件 | 注入内容 | 默认（= 原始方案） |
 * |---|---|---|
 * | H1 `GatePriorityPolicy`  | 前沿 gate 的处理次序   | **FIFO**（先入先出） |
 * | H2 `DecimationPredicate` | 哪些前端点可删         | **3 ≤ valence ≤ 6 且非边界** |
 * | H3 `GeometryPredictor`   | 被删点的位置预测子     | **邻居位置算术平均** |
 * | H4 `ResidualQuantizer`   | 残差的量化/变换        | **网格量化，恒等变换** |
 * | H5 `SymbolPostProc`      | 符号发出后的变换       | **恒等**（不做 code packing） |
 *
 * 硬约束（与 `StorageScheme.h` 同）：
 *   - 注入是**叠加**不是替换：原始行为永远保留且为默认；
 *   - 注入方案与原始方案**同门禁**（A1–A3 / 确定性 / QUANT_MISMATCH，无例外通道）；
 *   - **`src/` 只认识接口**：出现具体方案名即 lint 违规，实现在 `impl/` 或用户侧。
 *
 * 本文件只定义**接口与默认实现**，不认识任何具体算法。
 */
#ifndef PMSF_SCHEME_ENGINE_HOOKS_H
#define PMSF_SCHEME_ENGINE_HOOKS_H

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

namespace pmsf {

// ---- 引擎交给 hook 的**只读上下文** ---------------------------------------
//
// 刻意用**朴素值类型**而不是网格句柄：hook 实现（在 `impl/`）不应该、
// 也不需要知道引擎内部用的是哪套网格表示。引擎负责把当前状态摊平传进来，
// 把 hook 的返回值翻译回网格操作。这样换网格库不动 hook，换 hook 不动引擎。

/// @brief 一个"前沿顶点"在被判定时能看到的全部信息。
struct FrontVertexView {
    int valence = 0;         ///< 该顶点的度
    bool isBorder = false;   ///< 是否为边界点
    bool conquered = false;  ///< 是否已被本层征服过（引擎的状态位）
    std::uint32_t id = 0;    ///< 引擎侧稳定编号（只用于 hook 内部记账，不回传）
};

/// @brief 预测子看到的：被删点的**一环邻居位置**（已摊平，xyz 连续存放）。
struct PatchView {
    const double* neighborPos = nullptr;  ///< 长度 = 3 * neighborCount
    int neighborCount = 0;                ///< = 被删点的度
    double quantStep = 0.0;               ///< 当前量化步长（网格边长）
    const double* bboxMin = nullptr;      ///< 包围盒下界（xyz）
    const std::uint32_t* neighborIds = nullptr;
    /// @brief 被删/被插中心点在本端网格内的稳定编号（`Vertex::poolId`）。
    ///
    /// **两端各自的编号**：编码端 = 被删中心点，解码端 = 刚分配的中心点；两侧
    /// 池号不可比，hook 只能用它在本端网格里**定位**该顶点（例如从某个邻居的
    /// 一环里把它排除），**不得**当作跨端相等的数值使用。默认实现不读它，
    /// 故对既有方案零影响。
    std::uint32_t centerId = 0;
};

// ---- H1 · GatePriorityPolicy ----------------------------------------------

/// @brief H1 的只读上下文：引擎在 gate 出队时合成（顺序无关、双端可复算）。
///
/// 背景：论文的 prioritized gate selection（如 RD §3.2）需要看 gate 两端的
/// sign/tag、当前 valence、以及邻边里有多少条已进入 conquered region。这些量
/// 引擎自己就知道；把它摊平成中性 POD 传给 hook，`src/` 不需要认识任何算法名。
struct GatePriorityView {
    std::uint8_t frontSign = 0;    ///< 0 none / 1 PLUS / 2 MINUS（ConquestState sign）
    std::uint8_t backSign = 0;
    std::uint16_t frontValence = 0;
    std::uint16_t backValence = 0;
    std::uint16_t frontConquered = 0;  ///< 前端点邻边里另一端已 CONQUERED 的条数
    std::uint16_t backConquered = 0;
    std::uint8_t frontConqOk = 0;      ///< frontConquered ∈ [2,3]（论文 low-conquered）
    std::uint8_t backConqOk = 0;
};

/// @brief 前沿 gate 的处理次序。
///
/// 引擎维护一个"前沿"（待处理的 gate 集合）。默认按 **FIFO** 取出。
/// 若方案需要别的次序（例如按某种重要性排序），实现 `comparator()` 即可。
class GatePriorityPolicy {
  public:
    virtual ~GatePriorityPolicy() = default;

    /// @brief 返回 true 表示 `a` 应当**先于** `b` 被取出。
    /// 默认返回 false ⇒ 不构成严格弱序 ⇒ 引擎退化为 FIFO。
    /// @param aKey/aKey 引擎给出的中性排序键（越小越"早"由引擎定义）。
    virtual bool higherPriority(std::uint64_t /*aKey*/, std::uint64_t /*bKey*/) const {
        return false;
    }

    /// @brief 带只读上下文的重载（H1 PGS 用；默认 false ⇒ 与 FIFO 等价）。
    virtual bool higherPriority(const GatePriorityView& /*a*/,
                                const GatePriorityView& /*b*/) const {
        return false;
    }

    /// @brief 是否启用优先级（false ⇒ 引擎走纯 FIFO 快路径）。
    virtual bool isActive() const {
        return false;
    }
};

// ---- H2 · DecimationPredicate ---------------------------------------------

/// @brief 哪些前端点可删。留空 = 默认（`3 ≤ valence ≤ 6` 且非边界点）。
class DecimationPredicate {
  public:
    virtual ~DecimationPredicate() = default;

    /// @brief 可否删这个前端点。默认即原始方案的判据。
    virtual bool canDecimate(const FrontVertexView& v) const {
        return v.valence >= 3 && v.valence <= 6 && !v.isBorder && !v.conquered;
    }

    /// Optional stop/feature constraints for the degree-three cleanup phase.
    /// Its original driver already checks degree, boundary and conquered state.
    virtual bool canRegulate(const FrontVertexView&) const { return true; }
};

// ---- H3 · GeometryPredictor -----------------------------------------------

/// @brief 被删点的位置预测子。留空 = 默认（一圈邻居位置的**算术平均**）。
///
/// 注意：预测子只影响**残差的熵**，不影响重建精度 —— 残差在量化整数域计算，
/// 解码侧 `位置 = 预测 + 残差` 精确还原。所以这个 hook 是**纯压缩率杠杆**。
class GeometryPredictor {
  public:
    virtual ~GeometryPredictor() = default;

    /// Optional integer-domain prediction. Extrapolated cells may be negative;
    /// converting cell -1 to world and back through a clamping quantizer loses
    /// that value. Returning true avoids the lossy round-trip. Existing hooks
    /// return false and retain their historical world-domain behavior.
    virtual bool predictQuantized(const PatchView&, int[3]) const { return false; }

    /// @brief 写出预测位置到 `out[0..2]`。默认 = 邻居算术平均。
    virtual void predict(const PatchView& p, double out[3]) const {
        double x = 0, y = 0, z = 0;
        for (int i = 0; i < p.neighborCount; ++i) {
            x += p.neighborPos[3 * i + 0];
            y += p.neighborPos[3 * i + 1];
            z += p.neighborPos[3 * i + 2];
        }
        if (p.neighborCount > 0) {
            x /= p.neighborCount;
            y /= p.neighborCount;
            z /= p.neighborCount;
        }
        out[0] = x;
        out[1] = y;
        out[2] = z;
    }
};

// ---- H4 · ResidualQuantizer -----------------------------------------------

/// @brief 残差的量化与变换。默认 = **均匀网格量化 + 恒等变换**。
///
/// 量化规格（各向同性单一步长 = 包围盒最长边 / 2^Qbit）见
/// `docs/refactor/P13_1_ORACLE_BASELINE_2026-09-22.md` §17。
class ResidualQuantizer {
  public:
    virtual ~ResidualQuantizer() = default;

    /// @brief 世界坐标 → 整数格。默认：`q = ceil((c - min)/step) - 1`，`-1` 夹到 `0`。
    virtual int toCell(double c, double min, double step) const {
        int q = static_cast<int>(std::ceil((c - min) / step)) - 1.0;
        if (q == -1)
            q = 0;
        return q;
    }

    /// @brief 整数格 → 世界坐标（格中心）。默认：`min + (q + 0.5) * step`。
    virtual double toWorld(int q, double min, double step) const {
        return min + (static_cast<double>(q) + 0.5) * step;
    }

    /// @brief 整数残差的**可选坐标变换**（默认恒等）。
    ///
    /// 原始方案在某些模式下会把残差旋到局部标架里再编码。默认不做任何变换，
    /// 即 `out == in`。变换必须是**可逆**的：解码侧用 `inverseTransform` 还原。
    virtual void transform(const int in[3], int out[3], const PatchView& /*p*/) const {
        out[0] = in[0];
        out[1] = in[1];
        out[2] = in[2];
    }

    /// @brief `transform` 的逆。默认恒等。
    virtual void inverseTransform(const int in[3], int out[3], const PatchView& /*p*/) const {
        out[0] = in[0];
        out[1] = in[1];
        out[2] = in[2];
    }
};

// ---- H5 · SymbolPostProc --------------------------------------------------

/// @brief 符号发出之后的变换（默认恒等）。供 code packing 一类方案使用。
///
/// 引擎每次发射一个符号就调一次。默认实现什么都不做 ⇒ 行为与未注入完全相同。
class SymbolPostProc {
  public:
    virtual ~SymbolPostProc() = default;

    /// @brief `kind` 是中性的流标识（0 = 连通性，1 = 几何，2 = 其它）。
    virtual void onSymbol(int /*kind*/, std::uint32_t /*value*/) const {}
};

// ---- 打包 ------------------------------------------------------------------

/// @brief 五个引擎级 hook 的**可选**集合。任何一项为 nullptr 即回落默认。
struct EngineHooks {
    std::shared_ptr<GatePriorityPolicy> gatePriority;      ///< H1
    std::shared_ptr<DecimationPredicate> decimation;       ///< H2
    std::shared_ptr<GeometryPredictor> geometryPredictor;  ///< H3
    std::shared_ptr<ResidualQuantizer> residualQuantizer;  ///< H4
    std::shared_ptr<SymbolPostProc> symbolPostProc;        ///< H5

    /// @brief 是否**全部**留空（= 原始方案）。引擎据此走无 hook 的快路径。
    bool isEmpty() const {
        return !gatePriority && !decimation && !geometryPredictor && !residualQuantizer && !symbolPostProc;
    }

    // ---- 取值辅助：nullptr 一律回落默认实现 ----
    //
    // 用**静态默认实例**而不是临时构造：hook 在热循环里被调用，
    // 每次构造一个 shared_ptr 会显著拖慢压缩。默认实例无状态，可安全共享。
    const GatePriorityPolicy& gatePriorityOrDefault() const {
        static const GatePriorityPolicy kDefault;
        return gatePriority ? *gatePriority : kDefault;
    }
    const DecimationPredicate& decimationOrDefault() const {
        static const DecimationPredicate kDefault;
        return decimation ? *decimation : kDefault;
    }
    const GeometryPredictor& geometryOrDefault() const {
        static const GeometryPredictor kDefault;
        return geometryPredictor ? *geometryPredictor : kDefault;
    }
    const ResidualQuantizer& quantizerOrDefault() const {
        static const ResidualQuantizer kDefault;
        return residualQuantizer ? *residualQuantizer : kDefault;
    }
    const SymbolPostProc& symbolPostProcOrDefault() const {
        static const SymbolPostProc kDefault;
        return symbolPostProc ? *symbolPostProc : kDefault;
    }
};

using EngineHooksPtr = std::shared_ptr<const EngineHooks>;

}  // namespace pmsf

#endif  // PMSF_SCHEME_ENGINE_HOOKS_H
