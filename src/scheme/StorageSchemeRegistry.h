/*
 * @file StorageSchemeRegistry.h
 * @brief 存储方案的按名注册表（主计划 v8 §4 Phase S / R13）。
 *
 * 沿用算法注册表（`SimplificationAlgorithmRegistry`）的既有模式：**字符串键**、
 * 静态注册、按名再实例化。解码端拿产物 metadata 里的 `storage_scheme.name`
 * 来这里查表，与「算法按 `info().name` 再实例化」完全同构。
 *
 * **src/ 只认识本注册表**：具体方案名不出现在 src/，方案实现在 `impl/`
 * 或用户侧（设计文档 §6）。
 */
#ifndef PMSF_SCHEME_STORAGE_SCHEME_REGISTRY_H
#define PMSF_SCHEME_STORAGE_SCHEME_REGISTRY_H

#include "scheme/StorageScheme.h"

#include <functional>
#include <string>
#include <vector>

namespace pmsf {

using StorageSchemeFactory = std::function<StorageSchemePtr()>;

/// @brief 注册一个方案。同名重复注册返回 false（不覆盖，避免静默替换）。
bool registerStorageScheme(const std::string& name, StorageSchemeFactory factory);

/// @brief 按名实例化。未注册返回 nullptr（调用方负责给出明确诊断，不静默回落）。
StorageSchemePtr createStorageScheme(const std::string& name);

/// @brief 该名字是否已注册。
bool isStorageSchemeRegistered(const std::string& name);

/// @brief 已注册的全部方案名（排序后），用于诊断与 `--list-schemes`。
std::vector<std::string> registeredStorageSchemeNames();

/// @brief 静态注册器：在静态初始化期把方案登记进注册表。
class StorageSchemeRegistrar {
public:
    StorageSchemeRegistrar(const char* name, StorageSchemeFactory factory) {
        registerStorageScheme(name, std::move(factory));
    }
};

// ---- 本轮运行的「当前方案」 -------------------------------------------------
//
// 编码端：由管线入口从 config 的 `storage_scheme` 解析后设置；
// 解码端：由反序列化器从产物 metadata 读出后设置。
// 未设置 ⇒ 返回 nullptr ⇒ **原始方案**，行为与未引入本机制时逐字节相同。
//
// 为什么是进程级而不是穿参：方案要被**反序列化器**（读 header 时）与
// **序列化器**（写 header 时）两个相距很远的点读到，而它们之间没有共同的
// 配置对象。这与 `BlobLayout` 早期用全局注册表是同一取舍；若后续 S3/S4
// 需要多方案并存，再改为显式穿参。

/// @brief 设置本轮运行的方案。传 nullptr 表示回到原始方案。
void setActiveStorageScheme(StorageSchemePtr scheme);

/// @brief 取本轮运行的方案。nullptr = 原始方案。
StorageSchemePtr activeStorageScheme();

/// @brief 取本轮方案的**符号方案**（S2 组件）。无方案 / 方案未声明 ⇒ nullptr。
///
/// 约定与 `injectedStreamCoder` 相同：**nullptr 时调用方一行都不许走本接口**，
/// 必须原样走现有 collector 路径 —— 这是"默认逐字节不变"的来源。
/// 与 `injectedStreamCoder` 分开成两个函数，是因为两者的默认回落语义不同：
/// 编码器缺失 ⇒ 该流回落现有 coder；符号方案缺失 ⇒ **整条链路**回落现有行为。
SymbolSchemePtr activeSymbolScheme();

/// @brief 解码端：按产物自描述的 `storage_scheme{name,version}` 再实例化并设为当前方案。
///
/// 三种情形，**都不静默回落**（设计文档 §3.3「版本化拒绝」）：
///   - `name` 为空 ⇒ 旧产物/原始方案，清空当前方案并返回 true；
///   - 名字未注册 ⇒ 返回 false（调用方须给出面向用户的明确错误，不猜、不错解）；
///   - 名字已注册但 `version` 不符 ⇒ 返回 false（同一方案的不同版本可能位流不兼容）。
///
/// `why` 非空时写入失败原因，供调用方拼进错误信息。
bool activateStorageSchemeFromArtifact(const std::string& name,
                                       std::uint32_t version,
                                       std::string* why = nullptr);

// ---- 「方案失败是致命的」标记 -----------------------------------------------
//
// 为什么需要它：反序列化器读 compact.bin 失败时会 **fallback 到 parquet**（Tier 2
// 的既有设计）。但「结构读不动」和「产物声明的方案本进程不认」是两回事 ——
// 后者若也走 fallback，就会拿着**错误的方案**继续解，正是设计文档 §3.3 要禁止的
// 「静默错解」（实测：v99 产物触发了版本拒绝，却被 fallback 吞掉、进程退出码 0）。
//
// 因此任何一次方案激活失败都会置上这个粘性标记，反序列化器在决定 fallback 前
// 必须先查它：非空 ⇒ 不可回落，直接失败。

/// @brief 上次方案激活失败的原因；空 = 无失败。
const std::string& storageSchemeFatalError();

/// @brief 清空失败标记（每个解码会话开始时调用）。
void clearStorageSchemeFatalError();

}  // namespace pmsf

/// @brief 把一个存储方案注册进全局注册表。
///
/// 用法（方案实现在 impl/ 或用户侧，**不得**出现在 src/）：
/// ```cpp
/// PMSF_REGISTER_STORAGE_SCHEME("my_scheme_v1", [] {
///     return std::make_shared<MyScheme>();
/// });
/// ```
///
/// `name` 即 config.json 的 `storage_scheme` 字段值，也是写进产物 metadata、
/// 解码端再实例化所用的键。
#define PMSF_REGISTER_STORAGE_SCHEME(name, factory)          \
    namespace {                                              \
    const ::pmsf::StorageSchemeRegistrar                     \
        PMSF_SCHEME_REGISTRAR_CAT(pmsf_storage_scheme_reg_,  \
                                  __LINE__)(name, factory);  \
    }

// `##` 会阻止实参展开，故用两级间接把 `__LINE__` 展开后再拼接。
#define PMSF_SCHEME_REGISTRAR_CAT(a, b) PMSF_SCHEME_REGISTRAR_CAT_(a, b)
#define PMSF_SCHEME_REGISTRAR_CAT_(a, b) a##b

#endif  // PMSF_SCHEME_STORAGE_SCHEME_REGISTRY_H
