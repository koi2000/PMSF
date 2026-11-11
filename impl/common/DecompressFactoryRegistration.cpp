/*
 * @file impl/common/DecompressFactoryRegistration.cpp
 * @brief Phase A：为通用/默认 preset 注册解压 operator 工厂。
 *
 * 注册的 preset 后端组合：
 *   - Default / APTM / PCATM / PCLTTM / RD（binary / columnar / unified）
 *   - QEM（binary / columnar / unified）
 *   - PCLTTMValence（binary / columnar / unified）
 *
 * 这些 preset 的三件套都是通用 operator，没有算法专属子类。
 */
#include "core/DecompressOperatorRegistry.h"

#include "decompress/BasicDeserializeOperator.h"
#include "decompress/BasicSymbolReadOperator.h"
#include "decompress/ColumnarDeserializeOperator.h"
#include "decompress/OptimizedBasicDeserializeOperator.h"
#include "decompress/OptimizedSymbolReadOperator.h"
#include "decompress/UnifiedMultiTableDeserializeOperator.h"
#include "decompress/ValenceSymbolReadOperator.h"
#include "reconstruct/BasicReconstructOperator.h"

namespace pmsf {
namespace {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
DecompressOperatorTriple makeOptimizedBasicTriple() {
    auto reader = std::make_shared<OptimizedSymbolReadOperator>();
    auto reconstructor = std::make_shared<BasicReconstructOperator>();
    auto deserializer = std::make_shared<OptimizedBasicDeserializeOperator>(reader, reconstructor);
    return {deserializer, reader, reconstructor};
}

DecompressOperatorTriple makeOptimizedColumnarTriple(bool unified) {
    auto reader = std::make_shared<OptimizedSymbolReadOperator>();
    auto reconstructor = std::make_shared<BasicReconstructOperator>();
    std::shared_ptr<DeserializeOperator> deserializer =
        unified ? std::make_shared<UnifiedMultiTableDeserializeOperator>(reader, reconstructor)
                : std::make_shared<ColumnarDeserializeOperator>(reader, reconstructor);
    return {deserializer, reader, reconstructor};
}

DecompressOperatorTriple makeBasicTriple() {
    auto reader = std::make_shared<BasicSymbolReadOperator>();
    auto reconstructor = std::make_shared<BasicReconstructOperator>();
    auto deserializer = std::make_shared<BasicDeserializeOperator>(reader, reconstructor);
    return {deserializer, reader, reconstructor};
}

DecompressOperatorTriple makeBasicColumnarTriple(bool unified) {
    auto reader = std::make_shared<BasicSymbolReadOperator>();
    auto reconstructor = std::make_shared<BasicReconstructOperator>();
    std::shared_ptr<DeserializeOperator> deserializer =
        unified ? std::make_shared<UnifiedMultiTableDeserializeOperator>(reader, reconstructor)
                : std::make_shared<ColumnarDeserializeOperator>(reader, reconstructor);
    return {deserializer, reader, reconstructor};
}

DecompressOperatorTriple makeValenceTriple() {
    auto reader = std::make_shared<ValenceSymbolReadOperator>();
    auto reconstructor = std::make_shared<BasicReconstructOperator>();
    auto deserializer = std::make_shared<OptimizedBasicDeserializeOperator>(reader, reconstructor);
    return {deserializer, reader, reconstructor};
}

DecompressOperatorTriple makeValenceColumnarTriple(bool unified) {
    auto reader = std::make_shared<ValenceSymbolReadOperator>();
    auto reconstructor = std::make_shared<BasicReconstructOperator>();
    std::shared_ptr<DeserializeOperator> deserializer =
        unified ? std::make_shared<UnifiedMultiTableDeserializeOperator>(reader, reconstructor)
                : std::make_shared<ColumnarDeserializeOperator>(reader, reconstructor);
    return {deserializer, reader, reconstructor};
}

// ---------------------------------------------------------------------------
// 通用算法（Default / APTM / PCATM / PCLTTM / RD）
// ---------------------------------------------------------------------------
/// @brief 把同一套「通用优化」三件套工厂按算法名注册三遍（三后端）。
///
/// 名字 = `ISimplificationAlgorithm::info().name`，即压缩时写进 metadata 的算法名。
/// 解压时框架据此再实例化组件，因此算法不需要任何 XxxDecompress.cpp。
void registerGenericPresetByName(const std::string& name) {
    NamedDecompressOperatorRegistrar(name, Backend::Binary, makeOptimizedBasicTriple);
    NamedDecompressOperatorRegistrar(name, Backend::Columnar,
                                     []() { return makeOptimizedColumnarTriple(false); });
    NamedDecompressOperatorRegistrar(name, Backend::Unified,
                                     []() { return makeOptimizedColumnarTriple(true); });
}

struct GenericRegistrations {
    GenericRegistrations() {
        registerGenericPresetByName("default");
        registerGenericPresetByName("aptm");
        registerGenericPresetByName("pcatm");
        registerGenericPresetByName("pclttm");
        registerGenericPresetByName("rd");

        // QEM：列式/统一后端用 Basic 三元组（与压缩端的 Basic 采集器配对）。
        NamedDecompressOperatorRegistrar("qem", Backend::Columnar,
                                         []() { return makeBasicColumnarTriple(false); });
        NamedDecompressOperatorRegistrar("qem", Backend::Unified,
                                         []() { return makeBasicColumnarTriple(true); });
        // 与压缩端配对：QEM 的二进制路径改用 Optimized 采集器（见
        // impl/QEM/QEMAlgorithm.cpp 里为什么放弃 Basic 采集器），reader 必须跟着换。
        NamedDecompressOperatorRegistrar("qem", Backend::Binary, makeOptimizedBasicTriple);

        // PCLTTMValence：valence 采集/读取三件套。
        NamedDecompressOperatorRegistrar("pclttm_valence", Backend::Binary, makeValenceTriple);
        NamedDecompressOperatorRegistrar("pclttm_valence", Backend::Columnar,
                                         []() { return makeValenceColumnarTriple(false); });
        NamedDecompressOperatorRegistrar("pclttm_valence", Backend::Unified,
                                         []() { return makeValenceColumnarTriple(true); });
    }
} g_genericRegistrations;

}  // namespace
}  // namespace pmsf
