/*
 * @file impl/POMAR/DecompressFactoryRegistration.cpp
 * @brief Phase A：为 POMAR / POMARV2 注册解压 operator 工厂。
 */
#include "core/DecompressOperatorRegistry.h"

#include "decompress/ColumnarDeserializeOperator.h"
#include "POMAR/decompress/PomarColumnarDeserializeOperator.h"
#include "POMAR/decompress/PomarColumnarSymbolReadOperator.h"
#include "POMAR/decompress/POMARColumnarDeserializeOperator.h"
#include "decompress/UnifiedMultiTableDeserializeOperator.h"
#include "reconstruct/BasicReconstructOperator.h"
#include "POMAR/reconstruct/PomarReconstructOperatorV2.h"

namespace pmsf {
namespace {

DecompressOperatorTriple makePOMARColumnarTriple(bool unified) {
    auto reader = std::make_shared<PomarColumnarSymbolReadOperator>();
    auto reconstructor = std::make_shared<PomarReconstructOperatorV2>();
    // POMAR 的列式/统一后端都使用自己的 4 表 deserializer。
    (void)unified;
    auto deserializer = std::make_shared<POMARColumnarDeserializeOperator>(reader, reconstructor);
    return {deserializer, reader, reconstructor};
}

struct POMARRegistrations {
    POMARRegistrations() {
        // 解压按 `info().name` 分发，三个后端都要按名注册，否则会回落到框架
        // Default 解压器（POMAR 的位流它读不懂）。
        //
        // 2026-09-23 合流（P9 §61）：三个后端**注册同一个三元组**。
        // 原先 Binary 走 PomarDeserializeOperator + MockSymbolReadOperator，
        // 那是与列式无关的第二套解码器；压缩端已只保留列式（见
        // POMARAlgorithm.cpp 顶部注释），这里同步收敛。
        //
        // 刻意写成三个独立 lambda 而非复用同一个变量：注册器的存储语义
        // （按值 / 按引用）不该由这里假定。
        NamedDecompressOperatorRegistrar("pomar", Backend::Binary,
                                         []() { return makePOMARColumnarTriple(false); });
        NamedDecompressOperatorRegistrar("pomar", Backend::Columnar,
                                         []() { return makePOMARColumnarTriple(false); });
        NamedDecompressOperatorRegistrar("pomar", Backend::Unified,
                                         []() { return makePOMARColumnarTriple(false); });
    }
} g_pomarRegistrations;

}  // namespace
}  // namespace pmsf
