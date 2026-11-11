/*
 * @file Capability.h
 * @brief Capability-set abstraction that replaces the 12 ad-hoc behavior
 *        booleans in CompressPipelineConfig.
 *
 * Motivation: the current CompressPipelineConfig accumulates a boolean per
 * paper-specific pipeline tweak (updateAfterEliminate, enableIndependentSet,
 * preProcessPosition, postProcessPosition, headerCarryPrediction,
 * recomputeVertexDegreesOnce, trackContextLevel, serializerNeedsExplicitHeader,
 * serializerWantsSharedHeader, …). Every new algorithm threatens to add one.
 *
 * The capability model flips the ownership: each operator (ErrorMetric,
 * Predictor, SymbolCollector, …) declares the capabilities it needs, and
 * the pipeline composes them. Adding a new paper is additive — extend the
 * enum, make the relevant operator opt in, and the pipeline handles it.
 *
 * Phase 12 delivers the declarative vocabulary. Wiring the capability set
 * into the actual CompressPipelineConfig happens piecemeal during Phase 13
 * algorithm migrations — this keeps the legacy boolean path live and
 * byte-identical until an algorithm actively migrates.
 */
#ifndef PMSF_CORE_CAPABILITY_H
#define PMSF_CORE_CAPABILITY_H

#include <cstdint>
#include <initializer_list>

namespace pmsf {

/// Known pipeline capabilities. Stable ordinal values — never renumber.
///
/// Naming convention: a capability describes what is NEEDED by the pipeline
/// (or by one of its operators), not what is enabled by default. Absence of
/// a capability means "use the lightweight default".
enum class Capability : std::uint32_t {
    None                         = 0,

    // --- Compress-side capabilities ---
    UpdateAfterEliminate         = 1u << 0,  ///< 每次消除后重算代价状态（最优放置类度量需要）
    IndependentSet               = 1u << 1,  ///< 请求互不重叠的独立集选择（每轮一次）
    RecomputeDegreesOnce         = 1u << 2,  ///< 顶点度数只在开头重算一次，之后按增量维护
    ContextLevelTracking         = 1u << 3,  ///< 按 LOD 维护上下文状态
    HeaderCarriesPrediction      = 1u << 4,  ///< 预测信息写进 header，而不是每轮数据段
    PrePositionProcessEveryRound = 1u << 5,  ///< 每轮消除之前处理顶点位置
    PostPositionProcessEveryRound = 1u << 6, ///< 每轮消除之后处理顶点位置

    // --- Serialization capabilities ---
    SerializerSharedHeader       = 1u << 7,  ///< serializer wants a header shared with the collector
    SerializerExplicitHeader     = 1u << 8,  ///< serializer builds its own header
    SideChannelResidual          = 1u << 9,  ///< the pipeline will emit per-symbol residuals (Unified)
    SideChannelCodebookIndex     = 1u << 10, ///< codebook indices per symbol
    SideChannelContextLabel      = 1u << 11, ///< context labels per symbol
    SideChannelAnchorMask        = 1u << 12, ///< anchor / feature mask per symbol

    // --- Decompress-side capabilities (mirror) ---
    PredictorReconstructsPoint   = 1u << 13, ///< decompress-time reconstructor consults a Predictor
    CodebookLookup               = 1u << 14, ///< decompress-time reconstructor consults a Codebook

    // --- Phase 1: algorithm-hook derived capabilities ---
    //
    // 这些位不再手填，而是由 `ISimplificationAlgorithm::capabilities()` 从「哪些
    // hook 被重写」自动推导：重写 vertexShift() ⇒ 需要 V_r 流与 C_r^v 流，等等。
    // 它们取代 CompressOptions::Builder 上那批 per-algorithm 的布尔开关
    // （设计文档 §2.1.1）。
    HookVertexShift              = 1u << 15, ///< 重写 H4 ⇒ 需要 V_r / C_r^v 流
    HookEdgeFlip                 = 1u << 16, ///< 重写 H6 ⇒ 需要 E_r^f 流
    HookSegmentation             = 1u << 17, ///< 重写 H7 ⇒ 需要 partition 表
    HookBoundaryHandling         = 1u << 18, ///< 重写 H8 ⇒ 需要 boundary 表 / 边界装配
    HookReverseVertexShift       = 1u << 19, ///< 重写 H5 ⇒ 隐式逆移位（解码侧唯一 hook）
    HookDeclaredProperties       = 1u << 20, ///< 重写 declareProperties() ⇒ 算法私有网格属性
    HookDeclaredSideChannels     = 1u << 21, ///< 重写 declareSideChannels() ⇒ 自描述扩展列

    _MaxBit                      = 1u << 22  ///< add capabilities below this bit; grows to 32
};

/// Small bit-set over Capability. Trivial and zero-cost.
class CapabilitySet {
public:
    constexpr CapabilitySet() = default;
    constexpr CapabilitySet(Capability c) : bits_(static_cast<std::uint32_t>(c)) {}
    constexpr CapabilitySet(std::initializer_list<Capability> cs) {
        for (auto c : cs) bits_ |= static_cast<std::uint32_t>(c);
    }

    constexpr void add(Capability c) { bits_ |= static_cast<std::uint32_t>(c); }
    constexpr void remove(Capability c) { bits_ &= ~static_cast<std::uint32_t>(c); }

    constexpr bool has(Capability c) const {
        return (bits_ & static_cast<std::uint32_t>(c)) != 0;
    }

    constexpr CapabilitySet operator|(CapabilitySet other) const {
        CapabilitySet out = *this;
        out.bits_ |= other.bits_;
        return out;
    }
    constexpr CapabilitySet& operator|=(CapabilitySet other) {
        bits_ |= other.bits_;
        return *this;
    }

    constexpr std::uint32_t raw() const { return bits_; }

private:
    std::uint32_t bits_ = 0;
};

}  // namespace pmsf

#endif  // PMSF_CORE_CAPABILITY_H
