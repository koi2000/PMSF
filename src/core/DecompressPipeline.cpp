/*
 * @file DecompressPipeline.cpp
 * @brief 解压 pipeline Builder 实现 + 按算法名映射到 operator 三件套的工厂。
 *
 * 两点注意事项：
 *   (1) MMSEDeserializeOperator 与 VQMMCDeserializeOperator 的实现位于
 *       impl/MMSE/ 与 impl/VQMMC/，是 per-target 额外源文件（参见 cmake/PMSFAlgo.cmake
 *       的 pmsf_add_algo_pair(VQMMC COMPRESS_EXTRA ...)）。它们不链接进通用 core 静态库，
 *       所以本文件的 factory **不能** #include 它们的头文件——否则 core 对象文件会因
 *       未解析符号而 link error。
 *       这些算法在 impl/ main 里通过 Builder::withDeserializer(...) 手动注入
 *       自家的 DeserializeOperator，factory 对它们返回空三件套。
 *
 *   (2) 其他所有算法的 operator 都在 src/decompress / src/reconstruct 里，属于
 *       框架核心，factory 可以直接 make_shared。
 */
#include "core/DecompressPipeline.h"

#include "core/CompressPipeline.h"  // runDecompress
#include "core/DecompressOperatorRegistry.h"


#include "decompress/BasicDeserializeOperator.h"
#include "decompress/BasicSymbolReadOperator.h"
#include "decompress/DeserializeOperator.h"
#include "decompress/OptimizedBasicDeserializeOperator.h"
#include "decompress/OptimizedSymbolReadOperator.h"
#include "decompress/ValenceSymbolReadOperator.h"
#include "reconstruct/BasicReconstructOperator.h"

#ifdef COLUMNAR_SUPPORT
#    include "decompress/ColumnarDeserializeOperator.h"
#    include "decompress/UnifiedMultiTableDeserializeOperator.h"
#endif

#include <stdexcept>

namespace pmsf {

DecompressOperatorTriple makeDecompressOperatorsForAlgorithm(
    const std::string& algorithmName, Backend backend) {
    auto& registry = DecompressOperatorRegistry::instance();
    if (registry.hasName(algorithmName, backend)) {
        return registry.makeTripleByName(algorithmName, backend);
    }
    if (registry.hasName("default", backend)) {
        return registry.makeTripleByName("default", backend);
    }
    return DecompressOperatorTriple{};
}

std::shared_ptr<DeserializeOperator> makeDeserializeOperatorForAlgorithm(
    const std::string& algorithmName, Backend backend) {
    auto& registry = DecompressOperatorRegistry::instance();
    if (registry.hasName(algorithmName, backend)) {
        return registry.makeTripleByName(algorithmName, backend).deserializer;
    }
    // 未按名注册 ⇒ 回落框架 Default 解压器（算法未声明自有布局，用 canonical 布局）。
    if (registry.hasName("default", backend)) {
        return registry.makeTripleByName("default", backend).deserializer;
    }
    throw std::runtime_error(
        "[DecompressPipeline] 无法为算法 '" + algorithmName +
        "' 构造解压器：既没有按名注册的工厂，也没有 Default 回落路径。");
}

// ---------------------------------------------------------------------------
// Builder
// ---------------------------------------------------------------------------

DecompressPipeline::Builder::Builder() = default;

DecompressPipeline::Builder&
DecompressPipeline::Builder::withBackend(Backend backend) {
    backend_ = backend;
    return *this;
}

DecompressPipeline::Builder& DecompressPipeline::Builder::withInput(std::string path) {
    input_ = std::move(path);
    return *this;
}
DecompressPipeline::Builder& DecompressPipeline::Builder::withOutput(std::string path) {
    output_ = std::move(path);
    return *this;
}

DecompressPipeline::Builder&
DecompressPipeline::Builder::withExtraDouble(const std::string& key, double value) {
    if (key == "alpha") extraAlpha_ = value;
    else if (key == "codebookSize") extraCodebookSize_ = static_cast<int>(value);
    return *this;
}

DecompressPipeline::Builder&
DecompressPipeline::Builder::withDeserializer(std::shared_ptr<DeserializeOperator> op) {
    overrideDeserializer_ = std::move(op);
    return *this;
}
DecompressPipeline::Builder&
DecompressPipeline::Builder::withSymbolReader(std::shared_ptr<SymbolReadOperator> op) {
    overrideSymbolReader_ = std::move(op);
    return *this;
}
DecompressPipeline::Builder&
DecompressPipeline::Builder::withReconstructor(std::shared_ptr<ReconstructOperator> op) {
    overrideReconstructor_ = std::move(op);
    return *this;
}

DecompressOptions DecompressPipeline::Builder::build() const {
    std::shared_ptr<DeserializeOperator> deserializer = overrideDeserializer_;
    if (!deserializer) {
        throw std::runtime_error(
            "DecompressPipeline::Builder: 需要显式 .withDeserializer(...)；"
            "按算法名自动装配请走 pmsf::makeDecompressOperatorTripleByName / runDecompress。");
    }
    return DecompressOptions::Builder()
        .setDeserializeOperator(deserializer)
        .setPath(input_)
        .setOutputPath(output_)
        .build();
}

void DecompressPipeline::Builder::run() {
    DecompressOptions options = this->build();
    DecompressPipeline::run(options);
}

// ---------------------------------------------------------------------------
// 主循环
// ---------------------------------------------------------------------------

/// 解压的唯一执行入口。算子三件套已由调用方（`Builder::build` 或算法注册表的
/// `makeDecompressOperatorTripleByName`）装进 `options`，这里不再做任何分派。
void DecompressPipeline::run(DecompressOptions& options) {
    (void)pmsf::runDecompress(options);
}

}  // namespace pmsf
