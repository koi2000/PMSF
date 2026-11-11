/*
 * @file PMSFCollectorLayoutRegistry.h
 * @brief Phase B：框架侧 collector layout / export 分发的薄包装。
 *
 * 旧的 dynamic_cast 识别逻辑已迁移到 impl/common/CollectorLayoutPlugin.cpp 并通过
 * CollectorLayoutPluginRegistry 注册。本文件只保留通用接口，供 ColumnarSerializeOperator
 * 等框架代码调用。
 */
#ifndef PMSF_COMPRESS_PAPER_COLLECTOR_LAYOUT_REGISTRY_H
#define PMSF_COMPRESS_PAPER_COLLECTOR_LAYOUT_REGISTRY_H

#include "core/CollectorLayoutPluginRegistry.h"

namespace pmsf {

/// @brief 根据 collector 类型选择 paper blob layout 与 codec version。
/// @details 实际判断逻辑由 impl/ 层注册的插件完成；未注册时返回 invalid choice。
inline PMSFCollectorLayoutChoice paperCollectorLayoutChoice(
    SymbolCollectOperator* collector,
    bool enablePrediction) {
    return CollectorLayoutPluginRegistry::instance().chooseLayout(collector, enablePrediction);
}

/// @brief 执行 collector 的 payload 导出。
/// @details 实际导出逻辑由 impl/ 层注册的插件完成。
inline PMSFCollectorExportResult paperExportCollectorPayloadBuffer(
    SymbolCollectOperator* collector,
    char* buffer,
    bool enableQuantization,
    bool enablePrediction,
    bool enableCompress) {
    return CollectorLayoutPluginRegistry::instance().executeExport(
        collector, buffer, enableQuantization, enablePrediction, enableCompress);
}

}  // namespace pmsf

#endif  // PMSF_COMPRESS_PAPER_COLLECTOR_LAYOUT_REGISTRY_H
