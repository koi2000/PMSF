/*
 * @file DecompressOperatorRegistry.h
 * @brief Phase A：解压侧 operator 三件套工厂注册表。
 *
 * 目标：让 src/core/DecompressPipeline.cpp 不再直接 #include 任何算法专属的
 * reader/reconstructor/deserializer 头，只通过本注册表按 (算法名, 后端) 获取工厂。
 *
 * 每个算法在 impl/<algo>/ 的翻译单元里用静态 NamedDecompressOperatorRegistrar
 * 对象注册自己的 binary / columnar / unified 三件套工厂。
 */
#pragma once

#include "core/DecompressPipeline.h"

#include <string>

#include <functional>
#include <memory>
#include <unordered_map>

namespace pmsf {

/// @brief 解压 operator 三件套工厂。
using DecompressOperatorFactory = std::function<DecompressOperatorTriple()>;

/// @brief 全局解压 operator 工厂注册表。
class DecompressOperatorRegistry {
  public:
    static DecompressOperatorRegistry& instance();

    // 键 = `ISimplificationAlgorithm::info().name`，也就是压缩时写进 metadata 的
    // 算法名。解压时框架按名查表再实例化组件，因此不需要任何 `XxxDecompress.cpp`。
    void registerFactoryByName(const std::string& name, Backend backend,
                               DecompressOperatorFactory factory);
    bool hasName(const std::string& name, Backend backend) const;
    DecompressOperatorTriple makeTripleByName(const std::string& name, Backend backend) const;

  private:
    DecompressOperatorRegistry() = default;
    std::unordered_map<std::string, DecompressOperatorFactory> namedEntries_[4];
};

/// @brief 按算法名注册的 RAII 注册器。链接时自动完成注册。
struct NamedDecompressOperatorRegistrar {
    NamedDecompressOperatorRegistrar(std::string name, Backend backend,
                                     DecompressOperatorFactory factory);
};

}  // namespace pmsf
