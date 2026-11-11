// SPDX-License-Identifier: Apache-2.0
//
// MMSECQuestLayoutPlugin.cpp — register the `mmse_cq` collector in the framework's
// layout/export plugin registry (same seam the conquest codec uses).
//
// `ColumnarSerializeOperator` knows no algorithm collectors: it asks the
// `CollectorLayoutPluginRegistry`, whose choosers/executors are registered from
// impl/ TUs.  This file gives `MMSECQuestSymbolCollectOperator`:
//   * blob_layout = kPaperBlobLayoutConquestRC (15) — reuses the verified
//     conquest decode branch in ColumnarDeserializeOperator (whole blob into
//     rc_blob, per-round collectFromDecoded + reconstruct);
//   * codecVersionOverride = 2 — "raw whole blob", NOT the codec=4 canonical
//     segment table (which would treat the CQST bytes as class-I bit-plane
//     residuals and corrupt the offsets);
//   * export executor → MMSECQuestSymbolCollectOperator::exportToBufferWithRangeCoder
//     (writes [u32 trailerLen][MMT1 trailer][CQST v2]).
//
// No src/ change is involved: this is the documented per-algorithm seam.

#include "MMSE/compress/MMSECQuestSymbolCollectOperator.h"
#include "core/CollectorLayoutPluginRegistry.h"
#include "storage/PaperBlobLayoutRegistry.h"

namespace pmsf {
namespace {

PMSFCollectorLayoutChoice chooseMMSECQuestLayout(SymbolCollectOperator* collector,
                                                 bool /*enablePrediction*/) {
    PMSFCollectorLayoutChoice choice;
    if (collector == nullptr) return choice;
    if (dynamic_cast<MMSECQuestSymbolCollectOperator*>(collector)) {
        choice.valid = true;
        choice.blobLayout = kPaperBlobLayoutConquestRC;
        choice.codecVersionOverride = 2;
    }
    return choice;
}

PMSFCollectorExportResult executeMMSECQuestExport(SymbolCollectOperator* collector,
                                                  char* buffer,
                                                  bool enableQuantization,
                                                  bool /*enablePrediction*/,
                                                  bool /*enableCompress*/) {
    PMSFCollectorExportResult result;
    auto* codec = dynamic_cast<MMSECQuestSymbolCollectOperator*>(collector);
    if (codec == nullptr) return result;
    result.handled = true;
    result.usedBytes = codec->exportToBufferWithRangeCoder(buffer, enableQuantization);
    return result;
}

CollectorLayoutPluginRegistrar g_mmseCQuestLayoutPlugin(chooseMMSECQuestLayout,
                                                        executeMMSECQuestExport);

}  // namespace
}  // namespace pmsf
