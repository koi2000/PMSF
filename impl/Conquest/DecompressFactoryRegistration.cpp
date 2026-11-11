/*
 * @file impl/Conquest/DecompressFactoryRegistration.cpp
 * @brief 为算法名 `"conquest"` 注册解压 operator 三件套。
 *
 * 框架按 `ISimplificationAlgorithm::info().name`（压缩时写进 metadata）查表
 * 再实例化组件，因此本算法**不需要任何 `XxxDecompress.cpp`**。
 *
 * 三件套的分工（与 `P13_4_DECODER_BLUEPRINT_2026-09-22.md` §80 一致）：
 *   - deserializer：框架通用的 `ColumnarDeserializeOperator`。它按
 *     `blob_layout = 15` 走 conquest 分支（装填 reader 队列 → 每轮
 *     `collectFromDecoded` → `reconstruct`），**不需要算法专属 deserializer**。
 *   - reader：`ConquestSymbolReadOperator`（只把字节流读回内存）。
 *   - reconstructor：`ConquestReconstructOperator`（编排到 `pmsf::decodeLayer`）。
 */
#include "core/DecompressOperatorRegistry.h"

#include "Conquest/decompress/ConquestSymbolReadOperator.h"
#include "Conquest/reconstruct/ConquestReconstructOperator.h"
#include "decompress/ColumnarDeserializeOperator.h"

namespace pmsf {
namespace {

DecompressOperatorTriple makeConquestTriple() {
    auto reader = std::make_shared<ConquestSymbolReadOperator>();
    // ⚠️ 重建器必须持有 reader 的**同一个实例**：层的取用是 reader 上的游标，
    // 换一个实例就取不到层（实测会表现为"每轮什么都不做、V/F 一动不动"）。
    auto reconstructor = std::make_shared<ConquestReconstructOperator>(reader);
    auto deserializer = std::make_shared<ColumnarDeserializeOperator>(reader, reconstructor);
    return {deserializer, reader, reconstructor};
}

struct ConquestRegistrations {
    ConquestRegistrations() {
        // 解压按 `info().name` 分发；三个后端都注册，缺哪个就会回落到框架
        // Default 解压器（conquest 的位流它读不懂）。
        NamedDecompressOperatorRegistrar("conquest", Backend::Binary, makeConquestTriple);
        NamedDecompressOperatorRegistrar("conquest", Backend::Columnar, makeConquestTriple);
        NamedDecompressOperatorRegistrar("conquest", Backend::Unified, makeConquestTriple);
    }
} g_conquestRegistrations;

}  // namespace
}  // namespace pmsf
