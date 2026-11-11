/*
 * @file StreamEntropyCoder.h
 * @brief 流熵编码器的采集缝：优先用注入方案给的 coder，否则回落现有内嵌实现。
 *        （主计划 v8 §4 Phase S / R13，设计文档 §5 的 S1 阶段）
 *
 * **为什么需要这一层**：`pmsf::EntropyCoder` 接口（`src/entropy/EntropyCoder.h`）
 * 自 Phase 10 就存在，三个实现也齐备，但**全仓库零调用** —— 流编码器
 * （`CanonicalSymbolEncoder`）直接就地构造 `RangeCoderEntropyCoder`。
 * 于是「按流选熵编码器」这个能力一直没被接上。本文件把它接上：
 *
 *   - 方案给了该流的 coder ⇒ 用它；
 *   - 方案没给（或根本没有方案）⇒ 返回 nullptr，调用方走**原有代码路径**。
 *
 * 「没给就走原有路径」是**逐字节兼容**的关键：默认路径不是"等价实现"，
 * 而是**同一段代码**。S0/S1 的硬判据是默认路径产物与冻结基线逐字节一致，
 * 任何"看起来等价"的重写都不满足它。
 */
#ifndef PMSF_ENTROPY_STREAM_ENTROPY_CODER_H
#define PMSF_ENTROPY_STREAM_ENTROPY_CODER_H

#include "entropy/EntropyCoder.h"

#include <memory>
#include <string>

namespace pmsf {

/// @brief 取 `streamKind` 这一条流的注入编码器。
///
/// @param streamKind 中性流标识（用 `SymbolStreamSpec::name`，**不是**算法名）。
/// @return 方案为该流提供的编码器；无方案或该流未覆盖时返回 nullptr。
///
/// 调用方**必须**把 nullptr 当作"走原有路径"，而不是"生成一个默认编码器" ——
/// 后者会让默认路径的字节依赖本函数，破坏逐字节兼容。
std::shared_ptr<EntropyCoder> injectedStreamCoder(const std::string& streamKind);

}  // namespace pmsf

#endif  // PMSF_ENTROPY_STREAM_ENTROPY_CODER_H
