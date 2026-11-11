/*
 * @file SelectionPolicy.h
 * @brief 选择 / 可消除性策略 —— 算法把「怎么挑候选」交给框架的唯一载体。
 *
 * ## 为什么要有这个接缝
 *
 * 有些算法的选择行为不是默认行为：候选打分不同、误差项不同、允许参与消除的
 * 半边集合不同。这些差异**过去是硬编码在 `src/` 里的算法 if 分支**，靠
 * `getenv("PMSF_<算法>_...")` 切换 —— 例如 selection queue 与 removability 算子
 * 里那几处读特定算法环境变量的分支（具体是哪几处见对应提交说明）。
 *
 * 那违反框架的根本约束（`src/` 不认识任何算法）。本结构把这条差异收敛成
 * **三个算法无关的开关**：框架只问「策略是什么」，不问「你是谁」。
 *
 * ## 与 SelectOptions 的关系
 *
 * 策略经 `SelectOptions` 下发：`Builder::withSelectionPolicy(...)` →
 * `SelectOptions::getSelectionPolicy()`。`Builder::build()` 会把它同时写进
 * `IsRemovableOperator` 链的每个算子（算子实例在链上，拿不到 SelectOptions）。
 *
 * ## 默认值即「论文标准行为」
 *
 * 全部默认字段 = 框架默认行为，因此**不设策略的算法逐字节不变**。
 */
#pragma once

#include "Halfedge.h"

#include <functional>

namespace pmsf {

/// @brief 选择 / 可消除性策略。空 function 字段表示「用框架默认」。
struct SelectionPolicy {
    /// 是否允许「本轮新增的」（isAdded）半边参与候选。
    ///
    /// 论文标准 = 否：一个刚被三角化产生出来的半边不应立刻又被消掉，否则
    /// 环三角化与消除互相打架。个别严格复现路径需要放开，故留此开关。
    bool allowAddedHalfedges = false;

    /// 候选打分覆盖。返回**越大越优先**（调用方用 `>` 比较取最大）。
    /// 为空 ⇒ 调用方走默认的「第一个可消除候选」。
    std::function<float(MCGAL::Halfedge*)> candidateScore;

    /// 误差函数覆盖。为空 ⇒ 框架默认（返回 0，即退化成「只用入队顺序」）。
    ///
    /// 注意签名带 `p`（新位置）是沿用 `SelectOptions` 的误差访问器约定；
    /// 当前所有实现都不写 `p`。
    std::function<float(MCGAL::Halfedge*, MCGAL::Point&)> errorOverride;
};

}  // namespace pmsf
