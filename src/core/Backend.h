/*
 * @file Backend.h
 * @brief 物理存储后端枚举。
 *
 * 从 `AlgoPreset.h` 拆出来：原先 `Backend`（框架词汇）与 `AlgoPreset`
 * （算法白名单枚举）挤在同一个头里，导致任何只想说"用哪个后端"的框架代码
 * 都得把算法名单一起包含进来 —— 那正是反硬编码 lint 打击的东西。
 * 拆开后 `src/` 只依赖本头，算法名单回归 `impl/`。
 */
#pragma once

namespace pmsf {

/// @brief 后端枚举：Binary（legacy byte stream）/ Columnar（4 表关系式列存）/
///        Unified（在 V2 的 4 表基础上新增动态 side-channel 表 + 可选 blob 表）/
///        Paper（PaperStream v1：单文件 `_paper.parquet`，M2b pilot）。
///
/// V3 与 V2 物理上并列——V3 的 4 个基础表与 V2 字节级兼容（可直接用 V2 reader
/// 读取），仅通过不同的文件名后缀（_v3_metadata / _v3_base_mesh / _v3_symbols /
/// _v3_boundary）+ 可选的第 5 表 `_v3_sidechannel` + 可选的第 6 表 `_v3_blob`
/// 区分。目的是让 9 篇论文 + default 以最小代价迁移到 V3 schema，同时让未来
/// 新增 side channel（predictor residual / codebook index / feature label …）
/// 不需要再改核心 schema。
enum class Backend { Binary, Columnar, Unified, Paper };

}  // namespace pmsf
