#ifndef IMPL_COMMON_COLLECTOR_SYMBOLS_TABLE_BUILDER_H
#define IMPL_COMMON_COLLECTOR_SYMBOLS_TABLE_BUILDER_H

#include <memory>
#include <arrow/api.h>
#include <arrow/table.h>
#include "compress/SymbolCollectOperator.h"

namespace pmsf {

struct CollectorSymbolsTableContext {
    SymbolCollectOperator* collector = nullptr;
    bool enableQuantization = false;
    bool enablePrediction = false;
    bool sharedBaseMeshCoordinates = false;
    bool sharedBaseMeshFaceCoordIds = false;
};

std::shared_ptr<arrow::Schema> buildCollectorSymbolsSchema();
std::shared_ptr<arrow::Schema> buildCollectorSymbolsSchema(bool enableQuantization);
std::shared_ptr<arrow::Table> buildCollectorSymbolsTable(const CollectorSymbolsTableContext& ctx);

/// @brief Debug-only canonical-stream byte-level self-check：根据 blob layout 把 canonical
///        RoundSymbols 重新组装成算法原生 blob，供 ColumnarSerializeOperator 与 legacy
///        exporter 的输出做字节对比。算法相关逻辑集中在此，避免框架侧引入算法头文件。

} // namespace pmsf

#endif // IMPL_COMMON_COLLECTOR_SYMBOLS_TABLE_BUILDER_H
