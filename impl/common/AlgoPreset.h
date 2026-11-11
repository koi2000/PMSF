/*
 * @file AlgoPreset.h
 * @brief 算法预设枚举 + 到 PMSF 成员函数名的映射表。
 *
 * **这个头住在 `impl/`**：枚举体就是一张算法名单，属于插件层词汇。
 * `src/` 的框架代码只认 `core/Backend.h` 里的 `Backend` 与算法注册表的字符串键，
 * 不再需要把算法名单编进框架（见 scripts/tests/lint_src_no_algo_names.sh）。
 *
 * 设计理念：
 *   - 不合并具体 operator 子类（PPMC 的 range coder + prediction 逻辑与 Basic 完全不同，
 *     强合并会破坏 byte-identical，代价远超收益）。
 *   - 也不重命名基类。只在 "用户接入" 层引入**一个组装 facade**，把"算法 x 后端"映射到
 *     具体 operator 组合。
 *   - PMSF::xxxCompress / xxxDecompress 保留原状，继续作为 pipeline 的入口；
 *     DecompressPipeline::run 依据 preset 派发到对应命名方法。
 */
#pragma once

#include "core/Backend.h"

namespace pmsf {

/// @brief 论文预设。复现新论文时，如果能完全复用已存在的 operator 组合，直接传 Default；
///        否则在 AlgoPreset.cpp 的两张 factory table 里加一行注册即可（零 PMSF.cpp 改动）。
enum class AlgoPreset {
    /// 通用路径：OptimizedSymbolCollect + Basic/Columnar Serialize（= `PMSF::compress`）
    Default,
    /// QEM = Basic SymbolCollect + Basic/Columnar Serialize + QEM 特定 SelectOptions
    QEM,
    /// PPMC = 自带 range coder + prediction 的 PPMCSymbolCollect/PPMCSerialize
    PPMC,
    /// MMSE = Optimized collect + Basic/Columnar Serialize + MMSEPostProcessor
    MMSE,
    /// APTM = Optimized + level%2 Degree6 + AnchorPreprocessor
    APTM,
    /// PCATM = PCATMEliminate + independent set selector
    PCATM,
    /// PCLTTM = 默认 operator + degree3/6 Level2
    PCLTTM,
    /// PCLTTMValence = Valence-code SymbolCollect / Read（独立 rc_blob layout = VALENCE_RC=5）
    PCLTTMValence,
    /// RD = 最长边 seed + RDEliminate + level%2 Degree6 + Relaxed DegreeThree policy
    RD,
    /// POMAR = PomarEliminate + Pomar SymbolCollect/Serialize
    POMAR,
    /// POMARV2 = Pomar V2（refactor 分支 —— Pomar ColumnarSymbolCollectOperator 等全套）
    POMARV2,
    /// VQMMC = VQMMC SymbolCollect + VectorQuantizer codebook sidecar
    VQMMC,
};

/// @brief 对齐旧 PMSF::xxx{Compress,Decompress} 的 member function 命名（小写前缀）。
///        例如 AlgoPreset::QEM + Backend::Binary → "qemcompress" / "qemdecompress"；
///        AlgoPreset::QEM + Backend::Columnar → "qemColumnarCompress" / "qemColumnarDecompress"。
const char* pmsfCompressMethodName(AlgoPreset preset, Backend backend);
const char* pmsfDecompressMethodName(AlgoPreset preset, Backend backend);

}  // namespace pmsf
