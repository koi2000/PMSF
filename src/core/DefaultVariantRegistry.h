/*
 * @file DefaultVariantRegistry.h
 * @brief Phase A/B: Default 算法变体注册表。
 *
 * 旧的 tools/Default*Columnar* 工具本质上是 "Default + 某种特定 collector/eliminator
 * /reader/reconstructor 组合"。本注册表把这些组合抽象成名字 -> lambda 工厂，使
 * PMSFCompress/PMSFDecompress 仅通过 config.variant 即可选择不同 Default 变体，
 * 无需为每个变体维护单独可执行文件。
 */
#pragma once

#include "config/PMSFConfig.h"
#include "core/CompressPipeline.h"
#include "core/DecompressPipeline.h"
#include "options/CompressOptions.h"

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

namespace pmsf {

/// @brief 单个 Default 变体所需的完整压缩配置。
struct DefaultCompressSetup {
    CompressOptions options;
    CompressPipelineConfig pipeline;
};

/// @brief 单个 Default 变体需要的压缩/解压工厂。
struct DefaultVariantFactories {
    /// 根据 PMSFConfig 构造完整压缩配置（options + pipeline）。
    using CompressSetupFactory = std::function<DefaultCompressSetup(const PMSFConfig&)>;

    /// 根据 PMSFConfig 构造解压三件套（deserializer / reader / reconstructor）。
    using DecompressTripleFactory = std::function<DecompressOperatorTriple(const PMSFConfig&)>;

    CompressSetupFactory compress;
    DecompressTripleFactory decompress;
};

/// @brief 全局 Default 变体注册表。
class DefaultVariantRegistry {
  public:
    static DefaultVariantRegistry& instance();

    /// @brief 注册一个变体。通常由 impl/common/DefaultVariantEntries.cpp 的静态对象调用。
    void registerVariant(const std::string& name, DefaultVariantFactories factories);

    /// @brief 查询某变体是否已注册。
    bool has(const std::string& name) const;

    /// @brief 构造压缩配置。未注册时抛出。
    DefaultCompressSetup makeCompressSetup(const PMSFConfig& cfg) const;

    /// @brief 构造解压三件套。未注册时抛出。
    DecompressOperatorTriple makeDecompressTriple(const PMSFConfig& cfg) const;

  private:
    DefaultVariantRegistry() = default;
    std::unordered_map<std::string, DefaultVariantFactories> entries_;
};

/// @brief RAII 注册器。
struct DefaultVariantRegistrar {
    DefaultVariantRegistrar(const std::string& name, DefaultVariantFactories factories);
};

}  // namespace pmsf
