/*
 * @file RetriProtocolTuning.h
 * @brief 决定性三角化协议覆盖的**算法侧调参**（设计文档 §2.1.1 的 H3/H6 延伸）。
 *
 * 背景：仓库里有一批「协议覆盖」路径（`impl/common/RetriOverride.h`）——同一份
 * 简化算法可以换用另一种确定性三角化/连通性协议存储。历史上这些路径的算法差异
 * 写成 `preset == AlgoPreset::X` 分支，把算法名硬编码在协议实现里。
 *
 * Phase 1 把差异抽成这个纯数据结构：**协议实现只认参数，不认算法名**；算法通过
 * `ISimplificationAlgorithm::retriangulationTuning()` 提供自己的取值。
 *
 * 本头文件只有数据、没有算法名，因此可以放在 src/。
 */
#ifndef PMSF_ALGORITHM_RETRIPROTOCOLTUNING_H
#define PMSF_ALGORITHM_RETRIPROTOCOLTUNING_H

#include <cstdlib>
#include <string>

namespace pmsf {

struct RetriProtocolTuning {
    /// 打开自适应残差流（PMSF_RDTAG_ADAPTIVE_RESIDUAL=1）。
    bool adaptiveResidual = false;
    /// 保留算法的 H4 顶点移位流（带 optimal placement 的几何驱动算法需要）。
    bool keepAdjustment = false;
    /// Encode layer residuals in a reconstructible local patch frame.
    bool localFrameResidual = false;
    /// Nonempty for an explicit profile that publishes its name in PaperStream
    /// and opts into the registered protocol without the legacy env switch.
    std::string paperArtifactAlgorithm;
    /// 无环境变量覆盖时使用的量化位宽。
    int defaultQuantBits = 11;
    /// 算法自己的量化位宽环境变量名。
    std::string quantBitsEnv;
    /// 备选环境变量名。
    std::string altQuantBitsEnv;
    /// true ⇒ 两个环境变量都未设置时**不施加**量化设置（保持默认位宽）。
    bool requireQuantBitsEnv = false;

    bool shouldApplyQuantBits() const {
        if (!requireQuantBitsEnv) return true;
        auto hasEnv = [](const std::string& name) {
            if (name.empty()) return false;
            const char* v = std::getenv(name.c_str());
            return v && *v;
        };
        return hasEnv(quantBitsEnv) || hasEnv(altQuantBitsEnv);
    }

    /// 解析实际使用的量化位宽。
    int resolveQuantBits() const {
        auto readEnv = [](const std::string& name) -> int {
            if (name.empty()) return -1;
            const char* v = std::getenv(name.c_str());
            if (!v || !*v) return -1;
            return std::atoi(v);
        };
        const int fromOwn = readEnv(quantBitsEnv);
        if (fromOwn > 0) return fromOwn;
        const int fromAlt = readEnv(altQuantBitsEnv);
        if (fromAlt > 0) return fromAlt;
        return defaultQuantBits;
    }
};

}  // namespace pmsf

#endif  // PMSF_ALGORITHM_RETRIPROTOCOLTUNING_H
