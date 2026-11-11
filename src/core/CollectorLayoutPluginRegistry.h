/*
 * @file CollectorLayoutPluginRegistry.h
 * @brief Phase B：把 collector -> BlobLayout / export 策略的判断逻辑从 src/compress
 * 解耦到 impl/ 的插件注册表。
 *
 * 框架代码（src/compress/PMSFCollectorLayoutRegistry.h 及其使用者）只依赖本文件中的
 * 通用结构体和注册表接口；具体算法 collector 的 dynamic_cast 识别逻辑放在
 * impl/common/CollectorLayoutPlugin.cpp（或各算法自己的插件文件）中注册。
 */
#pragma once

#include "compress/SymbolCollectOperator.h"
#include "storage/PaperBlobLayoutRegistry.h"

#include <functional>
#include <memory>
#include <vector>

namespace pmsf {

struct PMSFCollectorLayoutChoice {
    bool valid = false;
    std::int32_t blobLayout = kPaperBlobLayoutUnknown;
    std::int32_t codecVersionOverride = 0;
};

struct PMSFCollectorExportResult {
    bool handled = false;
    int usedBytes = 0;
};

/// @brief 单个插件：根据 collector 与 enablePrediction 决定 layout 选择。
using CollectorLayoutChooser =
    std::function<PMSFCollectorLayoutChoice(SymbolCollectOperator* collector, bool enablePrediction)>;

/// @brief 单个插件：执行 collector 的 payload 导出。
using CollectorExportExecutor = std::function<PMSFCollectorExportResult(
    SymbolCollectOperator* collector,
    char* buffer,
    bool enableQuantization,
    bool enablePrediction,
    bool enableCompress)>;

/// @brief 布局/导出插件注册表。
class CollectorLayoutPluginRegistry {
  public:
    static CollectorLayoutPluginRegistry& instance();

    /// @brief 注册一个布局选择插件。多个插件按注册顺序尝试，第一个返回 valid 的生效。
    void registerLayoutChooser(CollectorLayoutChooser chooser);

    /// @brief 注册一个导出执行插件。多个插件按注册顺序尝试，第一个 handled 的生效。
    void registerExportExecutor(CollectorExportExecutor executor);

    PMSFCollectorLayoutChoice chooseLayout(SymbolCollectOperator* collector,
                                           bool enablePrediction) const;
    PMSFCollectorExportResult executeExport(SymbolCollectOperator* collector,
                                            char* buffer,
                                            bool enableQuantization,
                                            bool enablePrediction,
                                            bool enableCompress) const;

  private:
    CollectorLayoutPluginRegistry() = default;
    std::vector<CollectorLayoutChooser> layoutChoosers_;
    std::vector<CollectorExportExecutor> exportExecutors_;
};

/// @brief RAII 注册器。
struct CollectorLayoutPluginRegistrar {
    CollectorLayoutPluginRegistrar(CollectorLayoutChooser chooser,
                                   CollectorExportExecutor executor = {});
};

}  // namespace pmsf
