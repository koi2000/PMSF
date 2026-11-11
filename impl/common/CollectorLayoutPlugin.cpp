/*
 * @file impl/common/CollectorLayoutPlugin.cpp
 * @brief Phase B： collector -> BlobLayout / export 策略插件实现。
 *
 * 本文件包含原本 src/compress/PMSFCollectorLayoutRegistry.h 中的算法专属 dynamic_cast
 * 判断逻辑，现在以插件形式注册到 src/core/CollectorLayoutPluginRegistry，使框架侧不再
 * 需要知道任何具体 collector 类型。
 */
#include "core/CollectorLayoutPluginRegistry.h"
#include "common/PaperLayoutAliases.h"

#include "APTM/compress/APTMConquestSymbolCollectOperator.h"
#include "compress/BasicSymbolCollectOperator.h"
#include "DefaultPPMCLite/compress/DefaultPPMCLiteSymbolCollectOperator.h"
#include "MMSE/compress/MMSEDualRingSymbolCollectOperator.h"
#include "compress/OptimizedSymbolCollectOperator.h"
#include "PCATM/compress/PCATMColorSymbolCollectOperator.h"
#include "PCATM/compress/PCATMPMSFSymbolCollectOperator.h"
#include "PCATM/compress/PCATMPureSymbolCollectOperator.h"
#include "common/strict/PCATMStrictPMSFSymbolCollectOperator.h"
#include "PCATM/compress/PCATMZColorSymbolCollectOperator.h"
#include "common/strict/PCLTTMStrictSymbolCollectOperator.h"
#include "PCLTTM/compress/PCLTTMTagSymbolCollectOperator.h"
#include "PPMC/compress/PPMCSegmentationSymbolCollectOperator.h"
#include "PPMC/compress/PPMCSymbolCollectOperator.h"
#include "common/patch/RDGateOnlySymbolCollectOperator.h"
#include "RD/compress/RDSymbolCollectOperator.h"
#include "RD/compress/RDTagSymbolCollectOperator.h"
#include "RD/compress/RDStrictSymbolCollectOperator.h"
#include "compress/SegmentedOptimizedSymbolCollectOperator.h"
#include "compress/SymbolCollectOperator.h"
#include "ValencePatchOnly/compress/ValencePatchOnlySymbolCollectOperator.h"
#include "compress/ValenceSymbolCollectOperator.h"
#include "VQMMC/compress/VQMMCCodebookSymbolCollectOperator.h"

#include <cstdlib>

namespace pmsf {
namespace {

PMSFCollectorLayoutChoice chooseLayoutImpl(SymbolCollectOperator* collector,
                                           bool /*enablePrediction*/) {
    PMSFCollectorLayoutChoice choice;
    if (collector == nullptr) {
        return choice;
    }

    choice.valid = true;
    if (dynamic_cast<RDStrictSymbolCollectOperator*>(collector)) {
        choice.blobLayout = kPaperBlobLayoutRDStrict;
    } else if (dynamic_cast<APTMConquestSymbolCollectOperator*>(collector)) {
        choice.blobLayout = kPaperBlobLayoutAPTMConquest;
    } else if (dynamic_cast<PCATMStrictPMSFSymbolCollectOperator*>(collector)) {
        choice.blobLayout = kPaperBlobLayoutPCATMStrictPaper;
    } else if (dynamic_cast<PCATMPureSymbolCollectOperator*>(collector)) {
        choice.blobLayout = kPaperBlobLayoutPCATMPure;
    } else if (dynamic_cast<PCATMPMSFSymbolCollectOperator*>(collector)) {
        choice.blobLayout = kPaperBlobLayoutPCATMPaper;
    } else if (dynamic_cast<PCATMZColorSymbolCollectOperator*>(collector)) {
        choice.blobLayout = kPaperBlobLayoutPCATMZColor;
    } else if (dynamic_cast<VQMMCCodebookSymbolCollectOperator*>(collector)) {
        choice.blobLayout = kPaperBlobLayoutVQMMCCodebook;
    } else if (dynamic_cast<MMSEDualRingSymbolCollectOperator*>(collector)) {
        const char* strictEnv = std::getenv("PMSF_MMSE_STRICT");
        choice.blobLayout = (strictEnv && strictEnv[0] == '1')
                                ? kPaperBlobLayoutMMSEStrict
                                : kPaperBlobLayoutMMSEDualRing;
    } else if (dynamic_cast<RDTagSymbolCollectOperator*>(collector)) {
        choice.blobLayout = kPaperBlobLayoutRDTag;
        choice.codecVersionOverride = 4;
    } else if (dynamic_cast<PCLTTMTagSymbolCollectOperator*>(collector)) {
        choice.blobLayout = kPaperBlobLayoutPCLTTMTag;
        choice.codecVersionOverride = 4;
    } else if (dynamic_cast<PCATMColorSymbolCollectOperator*>(collector)) {
        choice.blobLayout = kPaperBlobLayoutPCATMColor;
        choice.codecVersionOverride = 4;
    } else if (dynamic_cast<DefaultPPMCLiteSymbolCollectOperator*>(collector)) {
        choice.blobLayout = kPaperBlobLayoutDefaultPPMCLite;
        choice.codecVersionOverride = 4;
    } else if (dynamic_cast<ValencePatchOnlySymbolCollectOperator*>(collector)) {
        choice.blobLayout = kPaperBlobLayoutValencePatchOnly;
        choice.codecVersionOverride = 4;
    } else if (dynamic_cast<RDGateOnlySymbolCollectOperator*>(collector)) {
        choice.blobLayout = kPaperBlobLayoutRDGateOnly;
    } else if (dynamic_cast<RDSymbolCollectOperator*>(collector)) {
        choice.blobLayout = kPaperBlobLayoutRDGate;
        choice.codecVersionOverride = 4;
    } else if (dynamic_cast<ValenceSymbolCollectOperator*>(collector)) {
        choice.blobLayout = kPaperBlobLayoutValenceRC;
        choice.codecVersionOverride = 4;
    } else if (dynamic_cast<SegmentedOptimizedSymbolCollectOperator*>(collector)) {
        // 本标签对应的**字节布局已变两次**（枚举值未动）。
        // 旧：  [groupCount][groupStart[G+1]][payload]
        // 新：  [groupCount][seedA[G]][seedB[G]][groupStart[G+1]][payload]
        // 新₂： [groupCount][seedA[G]][seedB[G]][groupStart[G+1]]
        //       [baseFacetCount][rleRunCount][(gid:int8)(runLen:int32|varint) × runs]
        //       A negative rleRunCount selects varint run lengths.
        //       [payload]                       ← 后一段仅 groupCount > 1 时存在
        //       seedX[g]  = 分区 g 种子半边的两个端点 poolId（-1 = 无种子）
        //       基网 facet → 分区通道：RLE + int8 的基网 facet 分区号序列。
        // 变更原因（1）：解码端必须知道每个分区 BFS 的起点，而单 seed 时代的
        // 「从 vertices()[0]/[1] 重算」对多 seed 不成立。
        // 变更原因（2，PHASE59 物品 3）：附 16 证明「分组 → 消除 → 基网 → 分组」
        // 成环，解码端**拿不到**与编码端相同的基网分区号 ⇒ 必须下发。
        // 写出方是 SegmentedOptimizedSymbolCollectOperator::exportToBufferWithRangeCoder；
        // 消费者是 ColumnarDeserializeOperator 的 layout-32 前缀解析（两者必须同步）。
        choice.blobLayout = kPaperBlobLayoutSegmentedRangeCodedPred;
    } else if (dynamic_cast<OptimizedSymbolCollectOperator*>(collector)) {
        // Optimized 在启用预测时是 RangeCodedPred，否则是 OptimizedPlain；
        // 但本函数签名里没有 enablePrediction，这里保留历史默认：Unknown，让 export
        // 插件根据 enablePrediction 再决定。若未来需要，可改用注册表传入的
        // enablePrediction 参数。
        choice.valid = false;
        choice.blobLayout = kPaperBlobLayoutUnknown;
    } else if (dynamic_cast<PPMCSegmentationSymbolCollectOperator*>(collector)) {
        choice.blobLayout = kPaperBlobLayoutPPMCSegmentedRC;
    } else if (dynamic_cast<PPMCSymbolCollectOperator*>(collector)) {
        choice.blobLayout = kPaperBlobLayoutPPMCRC;
        choice.codecVersionOverride = 4;
    } else if (dynamic_cast<BasicSymbolCollectOperator*>(collector)) {
        choice.blobLayout = kPaperBlobLayoutBasic;
    } else {
        choice.valid = false;
        choice.blobLayout = kPaperBlobLayoutUnknown;
    }
    return choice;
}

PMSFCollectorExportResult executeExportImpl(SymbolCollectOperator* collector,
                                            char* buffer,
                                            bool enableQuantization,
                                            bool enablePrediction,
                                            bool enableCompress) {
    PMSFCollectorExportResult result;
    if (collector == nullptr) {
        return result;
    }

    auto exportRangeWithoutPredictionStamp = [&]() {
        result.handled = true;
        result.usedBytes =
            collector->exportToBufferWithRangeCoder(buffer, enableQuantization);
        return result;
    };
    auto exportRangeWithPredictionStamp = [&](bool prediction) {
        collector->setEnablePrediction(prediction);
        result.handled = true;
        result.usedBytes =
            collector->exportToBufferWithRangeCoder(buffer, enableQuantization);
        return result;
    };
    auto exportPlain = [&]() {
        result.handled = true;
        result.usedBytes =
            collector->exportToBuffer(buffer, enableQuantization, enableCompress);
        return result;
    };

    if (dynamic_cast<RDStrictSymbolCollectOperator*>(collector) ||
        dynamic_cast<APTMConquestSymbolCollectOperator*>(collector) ||
        dynamic_cast<PCLTTMStrictSymbolCollectOperator*>(collector)) {
        return exportRangeWithoutPredictionStamp();
    }
    if (dynamic_cast<DefaultPPMCLiteSymbolCollectOperator*>(collector)) {
        return exportRangeWithPredictionStamp(true);
    }
    if (dynamic_cast<PCATMStrictPMSFSymbolCollectOperator*>(collector) ||
        dynamic_cast<PCATMPureSymbolCollectOperator*>(collector) ||
        dynamic_cast<PCATMPMSFSymbolCollectOperator*>(collector) ||
        dynamic_cast<PCATMZColorSymbolCollectOperator*>(collector) ||
        dynamic_cast<VQMMCCodebookSymbolCollectOperator*>(collector) ||
        dynamic_cast<MMSEDualRingSymbolCollectOperator*>(collector) ||
        dynamic_cast<RDTagSymbolCollectOperator*>(collector) ||
        dynamic_cast<PCLTTMTagSymbolCollectOperator*>(collector) ||
        dynamic_cast<PCATMColorSymbolCollectOperator*>(collector) ||
        dynamic_cast<ValencePatchOnlySymbolCollectOperator*>(collector) ||
        dynamic_cast<RDGateOnlySymbolCollectOperator*>(collector) ||
        dynamic_cast<RDSymbolCollectOperator*>(collector) ||
        dynamic_cast<ValenceSymbolCollectOperator*>(collector) ||
        dynamic_cast<SegmentedOptimizedSymbolCollectOperator*>(collector)) {
        return exportRangeWithPredictionStamp(enablePrediction);
    }
    if (dynamic_cast<OptimizedSymbolCollectOperator*>(collector) && enablePrediction) {
        return exportRangeWithPredictionStamp(true);
    }
    if (dynamic_cast<PPMCSegmentationSymbolCollectOperator*>(collector) ||
        dynamic_cast<PPMCSymbolCollectOperator*>(collector) ||
        dynamic_cast<BasicSymbolCollectOperator*>(collector) ||
        dynamic_cast<OptimizedSymbolCollectOperator*>(collector)) {
        return exportPlain();
    }

    return result;
}

// 处理 OptimizedSymbolCollectOperator 的 layout 分支：enablePrediction 为 true 时
// 是 RangeCodedPred，否则是 OptimizedPlain。
PMSFCollectorLayoutChoice chooseOptimizedLayout(SymbolCollectOperator* collector,
                                                bool enablePrediction) {
    PMSFCollectorLayoutChoice choice;
    if (collector == nullptr) return choice;
    if (dynamic_cast<OptimizedSymbolCollectOperator*>(collector)) {
        choice.valid = true;
        choice.blobLayout = enablePrediction ? kPaperBlobLayoutRangeCodedPred
                                             : kPaperBlobLayoutOptimizedPlain;
    }
    return choice;
}

CollectorLayoutPluginRegistrar g_collectorLayoutPlugin(
    [](SymbolCollectOperator* collector, bool enablePrediction) -> PMSFCollectorLayoutChoice {
        // Optimized 分支依赖 enablePrediction，先尝试。
        auto optChoice = chooseOptimizedLayout(collector, enablePrediction);
        if (optChoice.valid) return optChoice;
        return chooseLayoutImpl(collector, enablePrediction);
    },
    executeExportImpl);

}  // namespace
}  // namespace pmsf
