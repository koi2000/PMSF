/*
 * @file CollectorLayoutPluginRegistry.cpp
 * @brief CollectorLayoutPluginRegistry 实现。
 */
#include "core/CollectorLayoutPluginRegistry.h"

#include <stdexcept>

namespace pmsf {

CollectorLayoutPluginRegistry& CollectorLayoutPluginRegistry::instance() {
    static CollectorLayoutPluginRegistry registry;
    return registry;
}

void CollectorLayoutPluginRegistry::registerLayoutChooser(CollectorLayoutChooser chooser) {
    if (chooser) layoutChoosers_.push_back(std::move(chooser));
}

void CollectorLayoutPluginRegistry::registerExportExecutor(CollectorExportExecutor executor) {
    if (executor) exportExecutors_.push_back(std::move(executor));
}

PMSFCollectorLayoutChoice CollectorLayoutPluginRegistry::chooseLayout(
    SymbolCollectOperator* collector, bool enablePrediction) const {
    for (const auto& chooser : layoutChoosers_) {
        auto choice = chooser(collector, enablePrediction);
        if (choice.valid) return choice;
    }
    return PMSFCollectorLayoutChoice{};
}

PMSFCollectorExportResult CollectorLayoutPluginRegistry::executeExport(
    SymbolCollectOperator* collector,
    char* buffer,
    bool enableQuantization,
    bool enablePrediction,
    bool enableCompress) const {
    for (const auto& executor : exportExecutors_) {
        auto result = executor(collector, buffer, enableQuantization, enablePrediction, enableCompress);
        if (result.handled) return result;
    }
    return PMSFCollectorExportResult{};
}

CollectorLayoutPluginRegistrar::CollectorLayoutPluginRegistrar(CollectorLayoutChooser chooser,
                                                               CollectorExportExecutor executor) {
    CollectorLayoutPluginRegistry::instance().registerLayoutChooser(std::move(chooser));
    CollectorLayoutPluginRegistry::instance().registerExportExecutor(std::move(executor));
}

}  // namespace pmsf
