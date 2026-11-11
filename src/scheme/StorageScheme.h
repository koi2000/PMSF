/*
 * @file StorageScheme.h
 * @brief 存储优化注入机制的核心抽象（主计划 v8 §4 Phase S / R13）。
 *
 * 设计文档：`docs/refactor/STORAGE_SCHEME_INJECTION_DESIGN.md`（唯一依据）。
 *
 * 一个 `StorageScheme` 是**一个命名、带版本的方案包**，内含四个**可选**组件。
 * 任何组件留空即**回落框架默认** —— 这正是「与原始方案优雅结合」的形式化，
 * 与 `ISimplificationAlgorithm` 的全默认 hook 是同一模式。
 *
 * | 组件 | 注入内容 | 默认（= 原始方案） |
 * |---|---|---|
 * | S1 SymbolScheme     | 每轮发什么符号、几条流、什么遍历顺序 | 现有各 collector 行为 |
 * | S2 EntropyCoderFactory | 按流种类选熵编码器               | 现有内嵌 range coder |
 * | S3 SegmentLayout    | 流如何装入通用段目录                 | 现有布局 |
 * | S4 SchemeMetadata   | 方案自描述字段                       | 空 |
 *
 * **本文件（S0）只定义接口与注册表**：四个组件在本阶段全部为「留空 = 默认」，
 * 因此默认路径**逐字节不变**。S1 填实 S2、S2 阶段填实 S1（见设计文档 §5）。
 *
 * 硬约束（违反即改动作废）：
 *   - 注入是**叠加**不是替换：原始方案永远保留且为默认；
 *   - 注入方案与原始方案**同门禁**（A1–A3 / 确定性 / QUANT_MISMATCH，无例外通道）；
 *   - **src/ 只认识本接口与注册表**，出现具体方案名即 lint 违规；
 *     方案实现在 `impl/` 或用户侧代码里。
 */
#ifndef PMSF_SCHEME_STORAGE_SCHEME_H
#define PMSF_SCHEME_STORAGE_SCHEME_H

#include "scheme/EngineHooks.h"
#include "scheme/SymbolScheme.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace pmsf {

/// @brief 方案的注册表键与自描述。`name` 写进产物 metadata，解码端据此再实例化。
struct SchemeInfo {
    std::string name;
    std::uint32_t version = 0;
};

// ---- 四个可选组件的类型 ----------------------------------------------------
//
// S0 只把**类型名与传递方式**定下来；具体能力按设计文档 §5 分期填实。
// 组件留空（nullptr / 空 function）一律表示"用框架默认"，此时行为与
// 未指定方案**逐字节相同**。

// SymbolScheme 本体见 scheme/SymbolScheme.h（S2 第一批落地）。
using SymbolSchemePtr = std::shared_ptr<SymbolScheme>;

class EntropyCoder;                                  // src/entropy/EntropyCoder.h

/// @brief S2 组件：按流种类返回该流的熵编码器。返回 nullptr 表示用默认内嵌 coder。
///
/// `streamKind` 是**中性**的流标识（见 StreamKind），不是算法名 —— 保证
/// 方案实现无需知道、src/ 也不出现任何具体方案。
struct EntropyCoderFactory {
    std::function<std::shared_ptr<EntropyCoder>(const std::string& streamKind)>
        create;  ///< 空 = 全部用默认
    bool empty() const { return !create; }
};

/// @brief S3 组件：自定义流如何装入通用段目录。空 = 现有布局。
struct SegmentLayoutFn {
    std::function<void()> apply;  ///< 占位：S3 阶段填实为真实签名
    bool empty() const { return !apply; }
};

/// @brief S4 组件：方案自描述字段的写入器（附加键值对进 metadata）。
class SchemeMetadataBuilder {
public:
    virtual ~SchemeMetadataBuilder() = default;
    /// @brief 追加一个自描述字段。键名由方案定义，值以 UTF-8 文本承载。
    virtual void put(const std::string& key, const std::string& value) = 0;
};

/// @brief 存储方案。默认实现（不注册任何方案）等价于"全部用框架默认"。
class StorageScheme {
public:
    virtual ~StorageScheme() = default;

    /// 注册表键 + 版本。写进产物 metadata，解码端按 name 再实例化并按 version 校验。
    virtual SchemeInfo info() const = 0;

    /// S2 组件：符号方案（决定**发哪些符号**，bpv 的根因杠杆）。
    /// **默认 nullptr = 框架现有 collector 行为 = 逐字节现状**。
    virtual SymbolSchemePtr symbolScheme() const { return nullptr; }

    /// 便捷：取当前方案声明的符号方案；无方案/未声明 ⇒ nullptr（= 现状）。
    /// 约定：返回 nullptr 时调用方**一行都不许走本接口**，必须原样走现有路径。
    SymbolSchemePtr activeSymbolScheme() const { return symbolScheme(); }

    /// S2：熵编码器工厂。空 = 现有内嵌 range coder。
    virtual EntropyCoderFactory coderFactory() const { return {}; }

    /// S3：段布局。空 = 现有布局。
    virtual SegmentLayoutFn segmentLayout() const { return {}; }

    /// S4：向产物 metadata 追加方案自描述字段（除 name/version 之外的参数）。
    virtual void declareSchemeMetadata(SchemeMetadataBuilder& /*out*/) const {}

    /// S5：**引擎级** hook（前沿次序 / 可否删 / 预测子 / 残差量化 / 符号后处理）。
    ///
    /// S1–S4 管的是"发什么符号、怎么熵编码、怎么装段"；S5 管的是
    /// **遍历与预测本身**。返回 nullptr 表示五项全部用默认
    /// ⇒ 行为与未指定方案**逐字节相同**（与 S1–S4 同一模式：留空即回落）。
    virtual EngineHooksPtr engineHooks() const { return nullptr; }
};

using StorageSchemePtr = std::shared_ptr<StorageScheme>;

}  // namespace pmsf

#endif  // PMSF_SCHEME_STORAGE_SCHEME_H
