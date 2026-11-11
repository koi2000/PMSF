/*
 * @file ConquestLayoutPlugin.cpp
 * @brief 把 conquest 的 collector 接进框架的 **layout/导出插件注册表**。
 *
 * `ColumnarSerializeOperator` 不认识任何算法 collector：它调
 * `pmsf::paperCollectorLayoutChoice` / `paperExportCollectorPayloadBuffer`，
 * 由 `impl/` 侧注册的插件回答。**这正是给新算法留的接缝** ——
 * 本文件让 conquest 在 `impl/` 内自注册，**不需要改 `src/compress/`**。
 *
 * 产出 `kPaperBlobLayoutConquestRC = 15`；配套解码链路：
 *   `ConquestSymbolReadOperator::importFromRangeCodedBuffer`
 *   + `ConquestReconstructOperator`（调 `pmsf::decodeLayer`）。
 */
#include "Conquest/compress/ConquestSymbolCollectOperator.h"
#include "core/CollectorLayoutPluginRegistry.h"
#include "storage/PaperBlobLayoutRegistry.h"

namespace pmsf {
namespace {

PMSFCollectorLayoutChoice chooseConquestLayout(SymbolCollectOperator* collector, bool /*enablePrediction*/) {
    PMSFCollectorLayoutChoice choice;
    if (collector == nullptr)
        return choice;
    if (dynamic_cast<ConquestSymbolCollectOperator*>(collector)) {
        choice.valid = true;
        choice.blobLayout = kPaperBlobLayoutConquestRC;
        // ★ **不走 codec=4 的 canonical 段表**（蓝图 §81.4）。
        // v4 段表只认 class-I（RANGE_CODED_PRED/OPTIMIZED_PLAIN/CODEC_006）与
        // 少数几个自带 reassembler 的 layout；conquest 的整流落进去会被当成
        // **class-I 位面残差**还原 ⇒ 长度/偏移全错 ⇒ `rcBlob_.assign` 段错
        // （gdb 已定位到那一行）。codec=2 走的是"整条 blob 原样进 rc_blob"，
        // 正是 `ColumnarDeserializeOperator` 的 conquest 分支读的那一路。
        choice.codecVersionOverride = 2;
    }
    return choice;
}

PMSFCollectorExportResult executeConquestExport(SymbolCollectOperator* collector,
                                                char* buffer,
                                                bool enableQuantization,
                                                bool /*enablePrediction*/,
                                                bool /*enableCompress*/) {
    PMSFCollectorExportResult result;
    auto* conquest = dynamic_cast<ConquestSymbolCollectOperator*>(collector);
    if (conquest == nullptr)
        return result;
    result.handled = true;
    result.usedBytes = conquest->exportToBufferWithRangeCoder(buffer, enableQuantization);
    return result;
}

CollectorLayoutPluginRegistrar g_conquestLayoutPlugin(chooseConquestLayout, executeConquestExport);

}  // namespace
}  // namespace pmsf
