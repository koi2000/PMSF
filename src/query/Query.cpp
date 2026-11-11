/*
 * @file Query.cpp
 * @brief 见 Query.h。
 */
#include "query/Query.h"

#include "core/CompressPipeline.h"
#include "algorithm/SimplificationAlgorithmRegistry.h"
#include "core/DecompressPipeline.h"
#include "decompress/DeserializeOperator.h"
#include "options/DecompressOptions.h"

#include <stdexcept>

namespace pmsf {

QueryResult runQuery(const std::string& artifactPath,
                     const Query& q,
                     const std::string& algorithmName,
                     Backend backend,
                     const std::string& outputPath) {
    if (q.roi.groupIds.size() > 1) {
        throw std::runtime_error(
            "[Query] 一次查询只支持单个 region id；多区域查询请拆成多次调用"
            "（谓词下推是行级过滤，多区域会被编译成析取，当前后端不展开）。");
    }

    // 走算法注册表拿到**该算法自己的**解压选项（含它注册的 reader/reconstructor
    // 与 afterBaseMesh 钩子）。这是 Query 能对任意算法成立的前提：不同的简化算法
    // 写出的符号流需要配套的读取算子，旁路装配出来的解压器只能解出 base mesh。
    auto& registry = SimplificationAlgorithmRegistry::instance();
    auto algorithm = registry.create(algorithmName);  // 注册表内部已归一化大小写

    DecompressOptions options =
        algorithm ? algorithm->makeDecompressOptions(backend, artifactPath, outputPath)
                  : DecompressOptions::Builder()
                        .setDeserializeOperator(
                            makeDeserializeOperatorForAlgorithm(algorithmName, backend))
                        .setPath(artifactPath)
                        .setOutputPath(outputPath)
                        .build();
    auto deserializer = options.getDeserializeOperator();
    if (!deserializer) {
        throw std::runtime_error("[Query] 无法为算法 '" + algorithmName + "' 构造解压器");
    }

    // 把 Query 编译成后端谓词（论文式 (1)(2)(3)）。
    const int regionId = q.roi.groupIds.empty() ? -1 : q.roi.groupIds.front();
    deserializer->setQueryRegion(regionId);
    deserializer->setQueryMaxLod(q.maxLod);

    QueryResult result;
    result.predicatePushedDown = deserializer->supportsRandomAccess();
    if (result.predicatePushedDown && regionId >= 0) {
        result.decodedRegions.push_back(regionId);
    }

    result.mesh = runDecompress(options, algorithm ? algorithm->afterBaseMeshHook()
                                                   : AfterBaseMeshFn{});
    if (!outputPath.empty() && result.mesh) {
        result.mesh->dumpto_oldtype(outputPath);
    }
    return result;
}

}  // namespace pmsf
