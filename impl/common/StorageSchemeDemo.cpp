/*
 * @file StorageSchemeDemo.cpp
 * @brief Phase S0/S1 的**示例存储方案**（主计划 v8 §4 Phase S / R13）。
 *
 * 这个文件演示「用户如何把存储优化方案注入框架」——它**不在 src/**，
 * 所以可以自由使用具体方案名（src/ 出现方案名才是 lint 违规）。
 *
 * 三个方案，对应设计文档 §2 的两条注入路径：
 *
 *   | 方案名 | 作用 | 用途 |
 *   |---|---|---|
 *   | `passthrough_v1` | 四个组件全留空 | S0 验收：证明「接口+注册表+自描述+按名再实例化」端到端通，且语义上等价于原始方案 |
 *   | `static_huffman_v1` | 注入 StaticHuffman 熵编码器 | S1 验收：证明能注入（注：活路径无宿主，见 D7） |
 *   | `symbol_noop_v1` | 声明接管符号发射但不改任何东西 | S2 接口本体验收：证明可重写且**默认零影响** |
 *
 * 用法：config.json 里写 `"storage_scheme": "static_huffman_v1"`。
 * 不写 = 原始方案 = 行为逐字节不变。
 */
#include <fstream>
#include "scheme/StorageSchemeRegistry.h"

#include "entropy/StaticHuffmanCoder.h"

#include <memory>
#include <string>

namespace {

/// 四个组件全留空的方案。
///
/// 语义上**等价于原始方案**，但它仍然会在产物 metadata 里留下
/// `storage_scheme = {name, version}` —— 这正是 S0 要验收的自描述能力：
/// 解码端仅凭这一个字段就能按名再实例化。
class PassthroughScheme : public pmsf::StorageScheme {
public:
    pmsf::SchemeInfo info() const override { return {"passthrough_v1", 1}; }
};

// ---- S1：熵编码器注入 -------------------------------------------------------

/// 把 `pmsf::StaticHuffmanCoder` 包一层。
///
/// 框架的 `pmsf::EntropyCoder` 接口是 `reset/encode/decode` 三件套；
/// StaticHuffmanCoder 已实现该接口，这里只是把它交给 scheme 工厂，
/// 由流编码器按 `streamKind` 取用。
class StaticHuffmanScheme : public pmsf::StorageScheme {
public:
    pmsf::SchemeInfo info() const override { return {"static_huffman_v1", 1}; }

    pmsf::EntropyCoderFactory coderFactory() const override {
        pmsf::EntropyCoderFactory factory;
        // 所有流都换成 StaticHuffman。返回 nullptr 会让该流回落默认内嵌 coder，
        // 所以这里每个 streamKind 都给一个实例 —— 默认路径不受影响。
        factory.create = [](const std::string& /*streamKind*/) -> std::shared_ptr<pmsf::EntropyCoder> {
            return std::make_shared<pmsf::StaticHuffmanCoder>();
        };
        return factory;
    }
};

/// S2 第一批：符号方案接口的示例实现。
///
/// 它的 `emitSymbols`/`readSymbols` 都返回 false ⇒「本次不接管，请回落现有行为」，
/// 因此 round-trip 结果与原始方案**逐字节相同**。这正是一个实现该接口的
/// 方案应当具备的**默认安全**：不显式接管就什么都不改。
///
/// 真正会改变 bpv 的方案（如论文的符号方案）在 `emitSymbols` 里返回 true
/// 并提供自己的符号序列，同时在解码侧 `readSymbols` 里严格互逆 —— 编解码
/// 必须成对改，只改一侧会被 A1（顶点数）或确定性门禁立刻抓住。
class NoOpSymbolScheme : public pmsf::SymbolScheme {
public:
    std::string name() const override { return "symbol_noop_v1"; }
    std::uint32_t version() const override { return 1; }

    /// 声明"本方案关心这两条流"，但 emit/read 都返回 false ⇒ 实际不接管。
    std::vector<std::string> declaredStreams() const override { return {"valence", "isPoly"}; }
    bool overridesStream(const std::string& streamKind) const override {
        return streamKind == "valence" || streamKind == "isPoly";
    }
};

class SymbolNoOpScheme : public pmsf::StorageScheme {
public:
    pmsf::SchemeInfo info() const override { return {"symbol_noop_v1", 1}; }
    pmsf::SymbolSchemePtr symbolScheme() const override {
        return std::make_shared<NoOpSymbolScheme>();
    }
};

/// S2 符号方案钩子的**可逆演示**：把 isPoly 位取反。
///
/// 它**不减小**体积（取反是双射），存在的意义是证明两个钩子真的被驱动、
/// 且 `readSymbols(emitSymbols(x)) == x` 成立 —— 也就是"方案能改写本流发了
/// 什么符号"这条链路是通的。真正能降 bpv 的方案（论文的 conquest 符号方案）
/// 照同一形状写：`emitSymbols` 里把序列换成更短的表示，`readSymbols` 里展开回来。
///
/// 注意纪律：方案**不获任何门禁豁免** —— 它产出的产物照样要过 A1–A3/确定性，
/// 跑不过就是方案写错了。
class InvertIsPolySymbolScheme : public pmsf::SymbolScheme {
public:
    std::string name() const override { return "ispoly_invert_v1"; }
    std::uint32_t version() const override { return 1; }

    std::vector<std::string> declaredStreams() const override { return {"isPoly"}; }
    bool overridesStream(const std::string& kind) const override { return kind == "isPoly"; }

    bool emitSymbols(const std::string& kind,
                     std::vector<std::uint32_t>& symbols) const override {
        if (kind != "isPoly") return false;
        for (auto& v : symbols) v ^= 1u;   // 0<->1
        return true;
    }

    bool readSymbols(const std::string& kind,
                     const std::vector<std::uint32_t>& in,
                     std::vector<std::uint32_t>& out) const override {
        if (kind != "isPoly") return false;
        out = in;
        for (auto& v : out) v ^= 1u;       // 自逆：同一个操作还原
        return true;
    }
};

class IsPolyInvertScheme : public pmsf::StorageScheme {
public:
    pmsf::SchemeInfo info() const override { return {"ispoly_invert_v1", 1}; }
    pmsf::SymbolSchemePtr symbolScheme() const override {
        return std::make_shared<InvertIsPolySymbolScheme>();
    }
};

}  // namespace

PMSF_REGISTER_STORAGE_SCHEME("ispoly_invert_v1", [] {
    return std::make_shared<IsPolyInvertScheme>();
});

PMSF_REGISTER_STORAGE_SCHEME("symbol_noop_v1", [] {
    return std::make_shared<SymbolNoOpScheme>();
});

PMSF_REGISTER_STORAGE_SCHEME("passthrough_v1", [] {
    return std::make_shared<PassthroughScheme>();
});

PMSF_REGISTER_STORAGE_SCHEME("static_huffman_v1", [] {
    return std::make_shared<StaticHuffmanScheme>();
});
