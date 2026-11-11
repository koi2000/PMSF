/*
 * @file DefaultAlgorithm.cpp
 * @brief `default` 的插件实现 —— 框架的标准算法（无算法特有 hook）。
 *
 * ## 为什么要把它做成插件
 *
 * Phase 1 起压缩入口按名字查插件注册表，查不到才回落到「legacy 过渡桥」
 * （`AlgoPreset` + `impl/<ALGO>/PMSFEntry.cpp`）。9 个论文算法都已迁入插件，
 * **`default` 是最后一个还在走过渡桥的名字**（实测：
 * `PMSF_LOG_ALGO_DISPATCH=1` 下其余 9 个报 `plugin`，只有 `default` 报
 * `legacy-bridge`）。把它接过来，过渡桥就没有用户了，Phase 4 才能整体删掉
 * `AlgoPreset` 的名字表与字符串派发链。
 *
 * ## 为什么这个插件几乎是空的
 *
 * 框架的 `standardPipelineConfig(backend)` + `ISimplificationAlgorithm` 的默认
 * 实现，已经与旧 `PMSF::compress` / `PMSF::parquetV2Compress` 逐条对应：
 *
 *   - 采集器 / 序列化器 / 解压三元组：`defaultBinary()` / `defaultColumnar()`
 *     给的就是同一套（Optimized 采集 ↔ Optimized 读取）；
 *   - 量化前处理：框架标准 `preInitPreparation` 读 `PMSF_DEFAULT_QUANT_BITS`，
 *     与旧 `parquetV2Compress` 里那段**逐字相同**（只是日志前缀不同）；
 *   - 协议覆盖：`default` 不声明协议，与旧路径一致。
 *
 * 唯一的差别是二进制后端要 `headerCarryPrediction = true` —— 旧
 * `PMSF::compress` 显式设了这一位。所以本插件只重写 `pipelineTuner`。
 *
 * 预期产物**逐字节不变**；回归门禁的 6 个 default 格子是判据。
 */
#include "algorithm/ISimplificationAlgorithm.h"
#include "algorithm/SimplificationAlgorithmRegistry.h"
#include "core/CompressPipeline.h"  // CompressPipelineConfig 的完整定义

namespace pmsf {
namespace {

class DefaultAlgorithm final : public ISimplificationAlgorithm {
  public:
    AlgorithmInfo info() const override {
        return AlgorithmInfo{
            .name = "default",
            .version = 1,
            .primitive = PrimitiveKind::Vertex,
            .selection = SelectionKind::GeometryDriven,
        };
    }

    /// 唯一的算法特有开关：二进制后端让 header 承载 prediction 位
    /// （旧 `PMSF::compress` 的行为）。列式后端不动 —— 旧
    /// `parquetV2Compress` 也没设它。
    PipelineTuner pipelineTuner() const override {
        return [](CompressPipelineConfig& cfg, Backend backend) {
            if (backend == Backend::Binary) {
                cfg.headerCarryPrediction = true;
            }
        };
    }
};

}  // namespace
}  // namespace pmsf

PMSF_REGISTER_SIMPLIFICATION_ALGORITHM(pmsf::DefaultAlgorithm);
