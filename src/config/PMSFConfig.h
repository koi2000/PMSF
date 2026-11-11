/*
 * @file PMSFConfig.h
 * @brief Phase A/B: JSON 配置驱动的统一压缩/解压入口 —— 配置结构体。
 */
#pragma once

#include "core/Backend.h"
#include "json.hpp"
#include <string>
#include <unordered_map>

namespace pmsf {

/// @brief Hook 插件描述（QEM 等算法可加载外部 .so 自定义误差度量）。
struct PMSFHookConfig {
    /// 内置 hook 名称；与 plugin 二选一。
    std::string builtin;
    /// 外部共享库路径（绝对或相对路径）。
    std::string plugin;
    /// 可选：工厂函数名，默认 "pmsf_create_error_metric_v1"。
    std::string factory = "pmsf_create_error_metric_v1";
};

/// @brief 单条 JSON 配置解析后的强类型结构。
struct PMSFConfig {
    Backend backend = Backend::Columnar;

    /// @brief config.json 的 `algorithm` 字段原文 —— 算法注册表的字符串键，
    ///        也是写进 metadata、解压侧再实例化所依赖的算法名（设计文档 §2.1.2）。
    ///        这是算法分发的**唯一**键；`algorithm` 枚举已在 Phase 4 删除。
    std::string algorithmName;

    /// @brief config.json 的 `algorithm_options` 段，交给 ISimplificationAlgorithm::configure()。
    nlohmann::json algorithmOptions = nlohmann::json::object();


    std::string input;
    std::string output;

    int rounds = -1;  // <0 表示使用算法默认值

    /// @brief 解压侧 LOD 截断：只解前 `max_lod` 轮（<0 = 全量）。
    /// 用于随机访问/prefix 验证与 R-D 曲线的端点采样。
    int maxLod = -1;

    // 行为开关（与 CompressOptions 对齐）
    //
    // P18.1（S1 F32 复活）：这些 bool 默认 false，**无法区分「键缺席」与「显式
    // 写 false」** —— 而算法 profile 的声明值此前一律压过 config，于是
    // `"quantization": false` 根本开不出 F32 档。`quantizationSet` 记录该键
    // **是否在 config 里出现过**：
    //   缺席     → 沿用算法 profile（既有门禁配置不写该键 ⇒ 行为逐字节不变）
    //   显式出现 → 以 config 为准（`false` 即 F32 档）
    // 只给 quantization 加这个标志：只有它需要「显式 false 生效」的语义，
    // 其余开关维持原覆盖规则不变，以免扩大改动面。
    bool quantization = false;
    bool quantizationSet = false;

    /// P18.2（E2）：分区数。`segmentsSet` 记录该键是否在 config 里出现过
    /// （仿 `quantizationSet`）：
    ///   缺席      → 沿用算法 profile 的 segmentation 位（现状：不分区）
    ///   显式出现  → 启用分区并把 N 交给 SegmentationOption::setNumSegments
    /// `segments = 0` 表示「启用分区但由系统内定分区数」
    /// （见 src/segmentation/SegmentationOption.h 的语义注释）。
    int segments = 0;
    bool segmentsSet = false;

    /// Optional independent boundary-compression module. When enabled the
    /// standard artifact is untouched and a companion boundary section is
    /// written next to it (see BoundaryCompressionModule).
    bool boundaryCompression = false;
    bool prediction = false;
    bool segmentation = false;
    bool adjustment = false;
    bool compression = false;

    // 算法特有参数
    double alpha = -1.0;        // MMSE α，<0 使用默认值
    int codebookSize = -1;      // VQMMC codebook size，<0 使用默认值

    // Hook 配置
    PMSFHookConfig hooks;

    // 透传字段：供未来扩展或算法特殊开关使用
    std::unordered_map<std::string, std::string> extra;

    // Default 算法变体（用于将旧 Default*Columnar* 工具迁移到配置驱动）。
    // 当算法为 default 且 variant 非空时，由 DefaultVariantRegistry 选择
    // 对应的 collector/eliminator/reader/reconstructor 工厂。
    std::string variant;

    /// @brief config.json 的 `storage_scheme` 字段（Phase S0 / R13）。
    ///        空 = 原始方案（行为逐字节不变）；非空 = 从存储方案注册表按名实例化。
    ///        名字即写进产物 metadata、解码端再实例化所用的键。
    std::string storageScheme;

    /// @brief 返回该算法在 config 未指定 rounds 时的默认值。
    int defaultRoundsForAlgorithm() const;
    /// @brief 返回实际应使用的 rounds（config 显式值优先）。
    int effectiveRounds() const {
        return rounds >= 0 ? rounds : defaultRoundsForAlgorithm();
    }
    /// @brief 返回实际应使用的 alpha（config 显式值优先）。
    double effectiveAlpha() const {
        return alpha >= 0.0 ? alpha : 0.5;
    }
    /// @brief 返回实际应使用的 codebookSize（config 显式值优先）。
    int effectiveCodebookSize() const {
        return codebookSize > 0 ? codebookSize : 256;
    }
};

}  // namespace pmsf
