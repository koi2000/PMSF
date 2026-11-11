/*
 * @file RetriProtocolRegistry.h
 * @brief 协议覆盖的依赖倒置接缝（Phase 1）。
 *
 * 「协议覆盖」的**实现**在 impl/common/RetriOverride.h —— 那是插件层，可以自由
 * include 各算法的算子。但 src/ 的算法契约需要能在 `compress()` 里触发它，
 * 又不能反向依赖 impl/。
 *
 * 因此这里放一组函数指针槽：impl/common 在静态初始化时注册实现，src/ 只按
 * 协议 id 调用。谁也没硬编码算法名。
 */
#ifndef PMSF_ALGORITHM_RETRIPROTOCOLREGISTRY_H
#define PMSF_ALGORITHM_RETRIPROTOCOLREGISTRY_H

#include "algorithm/RetriProtocolTuning.h"
#include "core/Backend.h"

#include <functional>
#include <memory>
#include <string>

class CompressOptions;
class DeserializeOperator;
class SymbolReadOperator;
class ReconstructOperator;

namespace pmsf {

/// 压缩侧协议覆盖：返回 true 表示「已接管，调用方不要再跑标准管线」。
/// M4-P0：把 backend 传进来 —— 覆盖路径必须构造与请求后端匹配的 pipeline
/// （columnar 段表 vs paper 单文件容器），不能硬编码 defaultColumnar。
using RetriCompressOverrideFn =
    std::function<bool(CompressOptions&, const std::string& protocol,
                       const RetriProtocolTuning&, Backend backend)>;
/// 解压侧协议覆盖：与压缩侧配对。
using RetriDecompressOverrideFn =
    std::function<bool(const std::string& protocol, const std::string& inputPath,
                       const std::string& outputPath)>;

/// 解压器工厂：返回「reader + reconstructor + 列式解压器」，解码循环仍由框架跑。
///
/// 与上面两个「整段接管」的覆盖不同，这个槽只交出**解压器对象**。随机访问查询
/// (runQuery) 需要这一层：它要的是「和算法自己解压时逐字节相同的 reader/
/// reconstructor」，但解码只跑前 max_lod 轮、且要挂上 (roi, lod) 谓词 ——
/// 整段接管做不到这件事。
///
/// `defaultReader` / `defaultReconstructor` 是算法注册表给出的默认算子对：当
/// `protocol` 为空（有些算法的协议取决于环境变量，查询进程里未必有）时，工厂
/// 返回的解压器会在 init 读到产物自带的 blob_layout 之后再决定换不换 —— 换不出来
/// 就沿用这一对。**决策依据是产物，不是进程环境**。
using RetriDeserializerFactoryFn = std::function<std::shared_ptr<DeserializeOperator>(
    const std::string& protocol, std::shared_ptr<SymbolReadOperator> defaultReader,
    std::shared_ptr<ReconstructOperator> defaultReconstructor)>;

/// Paper 后端的 retri 解压器工厂：实现方（COLUMNAR TU）用原生 reader pair
/// 构造 `PaperDeserializeOperator`，读单文件 paper 容器。协议为空/非 retri 时
/// 返回 nullptr（调用方保留既有 paper 三元组）；协议非空但 reader 未实现 ⇒
/// 抛异常 fail-fast，**不得**静默用 generic paper 三元组。
using RetriPaperDeserializerFactoryFn = RetriDeserializerFactoryFn;

/// @brief 根据协议 id 解析原生 reader/reconstructor 对（供 Paper 工厂使用）。
/// @return false 表示协议为空/未知，调用方不得据此替换 reader。
using RetriReaderPairFactoryFn = std::function<bool(
    const std::string& protocol, std::shared_ptr<SymbolReadOperator>& reader,
    std::shared_ptr<ReconstructOperator>& reconstructor)>;

/// @brief retri 解码上下文副作用（env 开关 / 网格属性 attach），按协议 id。
///        与 `makeRetriReaderPair` 成对：paper 解压器在 init/baseMesh 后补挂
///        kVPropRDR1 等属性，否则 RD 系 predictor 取属性时撞断言。
using RetriDecodeSideEffectsFn = std::function<bool(const std::string& protocol)>;
void registerRetriDecodeSideEffects(RetriDecodeSideEffectsFn fn);
bool applyRetriDecodeSideEffects(const std::string& protocol);

/// @brief Paper 后端 retri 的 opt-in 开关：`PMSF_PAPER_RETRI=1`（默认 0）。
///        默认关闭时 paper 压缩/解压走原 generic 路径，产物字节零变化。
bool paperRetriEnabled();

void registerRetriCompressOverride(RetriCompressOverrideFn fn);
void registerRetriDecompressOverride(RetriDecompressOverrideFn fn);
void registerRetriDeserializerFactory(RetriDeserializerFactoryFn fn);
void registerRetriPaperDeserializerFactory(RetriPaperDeserializerFactoryFn fn);
void registerRetriReaderPairFactory(RetriReaderPairFactoryFn fn);

/// @brief 按协议 id 触发压缩覆盖。协议为空的协议或未注册的实现 ⇒ 返回 false。
/// @param backend 目标存储后端（Columnar/Paper；Paper 仅在 PMSF_PAPER_RETRI=1 时
///        由调用方放行）。
bool tryRunRetriCompressOverride(CompressOptions& options, const std::string& protocol,
                                 const RetriProtocolTuning& tuning, Backend backend);
/// @brief 按协议 id 触发解压覆盖。
bool tryRunRetriDecompressOverride(const std::string& protocol,
                                   const std::string& inputPath,
                                   const std::string& outputPath);

/// @brief 构造解压器。
///
/// @param protocol 算法声明的协议 id（可能为空）
/// @param defaultReader/defaultReconstructor 算法注册表给出的默认算子对
/// @return 未注册实现时返回 nullptr（调用方回落到默认三元组）。
std::shared_ptr<DeserializeOperator> makeRetriDeserializer(
    const std::string& protocol, std::shared_ptr<SymbolReadOperator> defaultReader,
    std::shared_ptr<ReconstructOperator> defaultReconstructor);

/// @brief Paper 后端的 retri 解压器（nullptr = 非 retri 协议，保留既有三元组）。
std::shared_ptr<DeserializeOperator> makeRetriPaperDeserializer(
    const std::string& protocol, std::shared_ptr<SymbolReadOperator> defaultReader,
    std::shared_ptr<ReconstructOperator> defaultReconstructor);

/// @brief 解析某协议的原生 reader/reconstructor 对（Paper 工厂用）。
bool makeRetriReaderPair(const std::string& protocol,
                         std::shared_ptr<SymbolReadOperator>& reader,
                         std::shared_ptr<ReconstructOperator>& reconstructor);

}  // namespace pmsf

#endif  // PMSF_ALGORITHM_RETRIPROTOCOLREGISTRY_H
