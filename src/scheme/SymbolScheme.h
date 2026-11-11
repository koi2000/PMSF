/*
 * @file SymbolScheme.h
 * @brief 符号方案（SymbolScheme）—— 把 collector「发什么符号」抽成可重写组件。
 *        主计划 v10 §8 ① Phase S2 第一批。
 *
 * **这是 bpv 的根因杠杆**（设计文档 §1）：框架统一遍历每顶点约发 17 个符号，
 * 论文约 6 个。熵编码侧已在下界（实测），所以唯一量级足够的改动是**换掉
 * "发什么符号"这件事本身**。本接口就是那件事的挂载点。
 *
 * ## 默认 = 现状（硬约束）
 *
 * `StorageScheme::symbolScheme()` 默认返回 **nullptr**，此时：
 *   - 解码端/编码端**一行都不会走本接口**；
 *   - 产物与引入本机制前**逐字节相同**。
 * 这与 `ISimplificationAlgorithm` 的全默认 hook 是同一条纪律：
 * **注入是叠加，不是替换**。
 *
 * ## 与 EntropyCoder 注入的分工
 *
 * | 组件 | 管什么 | 阶段 |
 * |---|---|---|
 * | `SymbolScheme`（本文件） | **发哪些符号**（信息量本身） | S2 |
 * | `EntropyCoderFactory` | 已发的符号**怎么压**（边界的几个百分点） | S1 |
 *
 * 两者可叠加：先由 SymbolScheme 决定发什么，再交给 coder 压。任一组件的
 * 默认值都表示"用框架现有实现"。
 *
 * ## src/ 纪律
 *
 * 本文件只定义**接口**。具体方案（含任何论文名/方案名）必须放在
 * `impl/` 或用户侧 —— src/ 出现即 lint 违规（设计文档 §6）。
 */
#ifndef PMSF_SCHEME_SYMBOL_SCHEME_H
#define PMSF_SCHEME_SYMBOL_SCHEME_H

#include <cstdint>
#include <string>
#include <vector>

namespace pmsf {

struct SchemeInfo;  // scheme/StorageScheme.h

/// @brief 编码侧一次「本轮要发什么符号」的决策入口。
///
/// 实现者只需重写自己关心的部分，其余保持基类默认（= 现状）。
/// **所有钩子的默认实现都是"不干预"**，因此一个空实现等价于原始方案。
class SymbolScheme {
public:
    virtual ~SymbolScheme() = default;

    /// @brief 方案标识，与 `StorageScheme::info()` 同名同版本。
    ///        写进产物自描述，解码端据此再实例化（S0 已落地）。
    virtual std::string name() const = 0;
    virtual std::uint32_t version() const = 0;

    /// @brief 本方案是否接管 `streamKind` 这条流。
    ///
    /// `streamKind` 用**中性**的流标识（同 `SymbolStreamSpec::name`，
    /// 如 "valence"/"isPoly"），**不是**算法名 —— 保证 src/ 不出现算法/方案名。
    ///
    /// 默认返回 false ⇒ 该流按框架现有 collector 行为发符号。
    /// 只有返回 true 的流，框架才会调用下面的钩子。
    virtual bool overridesStream(const std::string& /*streamKind*/) const { return false; }

    /// @brief 本方案声明的流标识列表（诊断/自描述用）。
    ///        默认空 = 不接管任何流 = 现状。
    virtual std::vector<std::string> declaredStreams() const { return {}; }

    /// @brief 生产端（编码）：把**默认**符号序列变换成**方案自己**的序列。
    ///
    /// 入参/出参都是 `symbols`：进来到的是该流按现有 collector 行为产出的序列，
    /// 方案就地把它换成自己的表示（**通常更短** —— 这正是 bpv 杠杆的用法）。
    /// 只在 `overridesStream(streamKind)` 为真时被调用，返回 true 表示已接管。
    ///
    /// 返回 false 表示"本次不接管，请回落现有行为"—— **每轮可变**，
    /// 允许方案只接管部分轮次。默认 false。
    virtual bool emitSymbols(const std::string& /*streamKind*/,
                             std::vector<std::uint32_t>& /*symbols*/) const {
        return false;
    }

    /// @brief 消费端（解码）：把**方案自己**的序列还原成**默认**序列。
    ///
    /// **必须与 `emitSymbols` 严格互逆**（`readSymbols(emitSymbols(x)) == x`）。
    /// 为什么必须带出参：方案的价值在"编码端发得更少"，解码端就得把它**展开**
    /// 回默认序列 —— 只接一个 in 参数无法表达这件事（初版接口就是这么写错的，
    /// 等于把"能改符号"的能力架空了）。
    ///
    /// 编解码成对改是 S2 的硬要求：只改一侧会被 A1（顶点数）或确定性门禁抓住。
    /// 返回 true 表示已接管并把结果写入 `out`；false = 走现有 reader。
    virtual bool readSymbols(const std::string& /*streamKind*/,
                             const std::vector<std::uint32_t>& /*in*/,
                             std::vector<std::uint32_t>& /*out*/) const {
        return false;
    }

    /// @brief 方案是否**无损**（`readSymbols(emitSymbols(x)) == x`）。
    /// 框架不据此放行任何门禁（A1–A3/确定性对所有方案一视同仁），
    /// 仅供诊断与自描述使用。
    virtual bool lossless() const { return true; }
};

}  // namespace pmsf

#endif  // PMSF_SCHEME_SYMBOL_SCHEME_H
