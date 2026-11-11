/*
 * @file StorageSchemeRegistry.cpp
 * @brief 存储方案注册表与「当前方案」的实现（主计划 v8 §4 Phase S / R13）。
 */
#include "scheme/StorageSchemeRegistry.h"

#include <algorithm>
#include <iostream>
#include <map>
#include <mutex>

namespace pmsf {
namespace {

/// 函数内静态：避免跨 TU 的静态初始化顺序问题（方案注册器可能在任意 TU
/// 的静态初始化期运行，而注册表本身必须已经可用）。
std::map<std::string, StorageSchemeFactory>& registry() {
    static std::map<std::string, StorageSchemeFactory> instance;
    return instance;
}

std::mutex& registryMutex() {
    static std::mutex instance;
    return instance;
}

StorageSchemePtr& activeSlot() {
    static StorageSchemePtr instance;
    return instance;
}

std::string& fatalSlot() {
    static std::string instance;
    return instance;
}

}  // namespace

bool registerStorageScheme(const std::string& name, StorageSchemeFactory factory) {
    if (name.empty() || !factory) return false;
    std::lock_guard<std::mutex> lock(registryMutex());
    // 不覆盖：同名重复注册多半是两处方案实现撞名，静默覆盖会让人查很久。
    auto inserted = registry().emplace(name, std::move(factory));
    if (!inserted.second) {
        std::cerr << "[scheme] 存储方案 \"" << name
                  << "\" 重复注册，已忽略后一个（不覆盖既有实现）" << std::endl;
        return false;
    }
    return true;
}

StorageSchemePtr createStorageScheme(const std::string& name) {
    if (name.empty()) return nullptr;
    StorageSchemeFactory factory;
    {
        std::lock_guard<std::mutex> lock(registryMutex());
        auto it = registry().find(name);
        if (it == registry().end()) return nullptr;
        factory = it->second;
    }
    return factory ? factory() : nullptr;
}

bool isStorageSchemeRegistered(const std::string& name) {
    std::lock_guard<std::mutex> lock(registryMutex());
    return registry().find(name) != registry().end();
}

std::vector<std::string> registeredStorageSchemeNames() {
    std::lock_guard<std::mutex> lock(registryMutex());
    std::vector<std::string> names;
    names.reserve(registry().size());
    for (const auto& kv : registry()) names.push_back(kv.first);
    std::sort(names.begin(), names.end());
    return names;
}

void setActiveStorageScheme(StorageSchemePtr scheme) { activeSlot() = std::move(scheme); }

StorageSchemePtr activeStorageScheme() { return activeSlot(); }

SymbolSchemePtr activeSymbolScheme() {
    StorageSchemePtr scheme = activeSlot();
    if (!scheme) return nullptr;  // 原始方案
    return scheme->symbolScheme();
}

bool activateStorageSchemeFromArtifact(const std::string& name,
                                       std::uint32_t version,
                                       std::string* why) {
    auto fail = [&](const std::string& msg) {
        if (why != nullptr) *why = msg;
        fatalSlot() = msg;  // 粘性：反序列化器据此拒绝 fallback
        return false;
    };
    if (name.empty()) {  // 旧产物：无该字段 ⇒ 原始方案
        setActiveStorageScheme(nullptr);
        fatalSlot().clear();
        return true;
    }
    if (!isStorageSchemeRegistered(name)) {
        std::string known;
        for (const auto& n : registeredStorageSchemeNames()) {
            if (!known.empty()) known += ", ";
            known += n;
        }
        return fail("产物声明的存储方案 \"" + name + "\" 未在本进程注册" +
                    (known.empty() ? "（当前没有任何方案注册）" : "（已注册：" + known + "）"));
    }
    StorageSchemePtr scheme = createStorageScheme(name);
    if (!scheme) return fail("存储方案 \"" + name + "\" 实例化失败");
    const std::uint32_t actual = scheme->info().version;
    if (actual != version) {
        return fail("存储方案 \"" + name + "\" 版本不符：产物为 v" +
                    std::to_string(version) + "，本进程注册的是 v" +
                    std::to_string(actual));
    }
    setActiveStorageScheme(std::move(scheme));
    fatalSlot().clear();
    return true;
}

const std::string& storageSchemeFatalError() { return fatalSlot(); }

void clearStorageSchemeFatalError() { fatalSlot().clear(); }

}  // namespace pmsf
