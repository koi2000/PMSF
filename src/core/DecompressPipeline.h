/*
 * @file DecompressPipeline.h
 * @brief Phase 7 重构：解压端 fluent Builder。让 impl/ 层 main 不再手写
 *        `make_shared<PPMCSymbolReadOperator>()` / `make_shared<ColumnarDeserializeOperator>(...)` 等
 *        具体 operator 子类的选择，统一通过算法注册表 + Backend 两个键选定。
 *
 * 对偶于 src/core/CompressPipeline.h 的 CompressPipeline::Builder。
 *
 * 用法（impl/PCLTTM/PCLTTMDecompress.cpp 迁移后形态）：
 *
 * ```cpp
 * int main(int argc, char* argv[]) {
 *     auto args = pmsf_impl::parseArgs(argc, argv, "...");
 *     TimeRecorder recorder(...);
 *     pmsf::DecompressPipeline::Builder()
 *         .withBackend(backendOf(args))
 *         .withInput(args.input)
 *         .withOutput(args.output)
 *         .run();
 *     return 0;
 * }
 * ```
 *
 * 新算法接入时只需在 DecompressPipeline.cpp 的 factory 里注册一行，不需要改 impl/。
 */
#pragma once

#include <memory>
#include <string>

#include "core/Backend.h"
#include "options/DecompressOptions.h"

class DeserializeOperator;
class SymbolReadOperator;
class ReconstructOperator;

namespace pmsf {

class DecompressPipeline {
  public:
    /// @brief 默认 Builder 构造：Backend=Binary。
    class Builder {
      public:
        Builder();

        Builder& withBackend(Backend backend);
        Builder& withInput(std::string path);
        Builder& withOutput(std::string path);

        /// @brief 其他 PMSF-specific 参数（比如 MMSE 的 alpha、VQMMC 的 codebook size）
        ///        直接暴露为 double 字典，由算法自己的解压路径消费。
        Builder& withExtraDouble(const std::string& key, double value);

        /// @brief 允许手动指定已经造好的 operator 三件套，覆盖按算法名查表得到的默认值。
        Builder& withDeserializer(std::shared_ptr<DeserializeOperator> op);
        Builder& withSymbolReader(std::shared_ptr<SymbolReadOperator> op);
        Builder& withReconstructor(std::shared_ptr<ReconstructOperator> op);

        /// @brief 构建最终 DecompressOptions —— 组装 operator 三件套并注入。
        DecompressOptions build() const;

        /// @brief Builder + run 一体化：build() 后立刻调用 DecompressPipeline::run()。
        void run();

        Backend backend() const { return backend_; }

      private:
        Backend backend_ = Backend::Binary;
        std::string input_;
        std::string output_;
        std::shared_ptr<DeserializeOperator> overrideDeserializer_;
        std::shared_ptr<SymbolReadOperator> overrideSymbolReader_;
        std::shared_ptr<ReconstructOperator> overrideReconstructor_;
        double extraAlpha_ = 0.5;  // MMSE
        int extraCodebookSize_ = 256;  // VQMMC
    };

    /// @brief 执行解压主循环。
    static void run(DecompressOptions& options);
};

struct DecompressOperatorTriple {
    std::shared_ptr<DeserializeOperator> deserializer;
    std::shared_ptr<SymbolReadOperator> reader;
    std::shared_ptr<ReconstructOperator> reconstructor;
};
/// @brief 按算法名取「解压器 + reader + reconstructor」三件套。
///
/// 与 `makeDeserializeOperatorForAlgorithm` 的差别：它还交出 reader/reconstructor
/// —— 随机访问查询要在 init 之后按产物自带的 blob_layout 换掉这一对，只拿解压器
/// 换不了。
DecompressOperatorTriple makeDecompressOperatorsForAlgorithm(
    const std::string& algorithmName, Backend backend);

/// @brief 按算法名取解压器（Phase 1，设计文档 §2.1.2）。
///
/// 名字来自压缩时写进 metadata 的 `ISimplificationAlgorithm::info().name`；
/// 未注册时回落到框架 Default 解压器。这是「解压侧零算法代码」的落点：
/// 算法在自己的 cpp 里注册一次组件，解压端由框架按名再实例化。
std::shared_ptr<DeserializeOperator> makeDeserializeOperatorForAlgorithm(
    const std::string& algorithmName, Backend backend);

}  // namespace pmsf
