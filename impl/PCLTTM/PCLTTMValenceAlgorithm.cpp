/*
 * @file PCLTTMValenceAlgorithm.cpp
 * @brief `pclttm_valence` 的插件实现。
 *
 * ## 为什么要有这个文件
 *
 * Phase 1 起入口按名字查插件注册表，查不到才回落到「legacy 过渡桥」
 * （`AlgoPreset` + `impl/<ALGO>/PMSFEntry.cpp`）。`default` 迁完之后，
 * **`pclttm_valence` 是过渡桥最后一个用户**（清点 configs/scripts/impl/tools
 * 里出现过的全部算法名，只有它还没注册成插件）。把它接过来，过渡桥就没有用户，
 * Phase 4 才能整体删掉 `AlgoPreset` 的名字表与字符串派发链。
 *
 * ## 与旧路径的逐条对应
 *
 * 旧实现 `impl/PCLTTM/PMSFEntry_Valence.cpp::runPCLTTMValenceCompress`：
 *
 *   SelectOptions：ErrorSource::Halfedge +
 *                  DegreeThree / DegreeSix / Basic / UnifiedManifold 四个可消除算子
 *   Select：PriorityErrorSelector       （= 契约的默认 selector）
 *   Eliminate：VertexRemovalEliminateOperator（= 契约的默认 eliminate）
 *   CompressOptions：compress/prediction/quantization 全开
 *   管线：CompressPipeline::run(..., AlgoPreset::PCLTTMValence, backend)
 *
 * `AlgoPreset::PCLTTMValence` 在 Builder 里只设了四个通用开关
 * （updateAfterEliminate=false / recomputeVertexDegreesOnce / trackContextLevel /
 * headerCarryPrediction），所以这里只重写 `removability()` + `pipelineTuner()`。
 *
 * 解压侧无需改动：`NamedDecompressOperatorRegistrar("pclttm_valence", ...)` 早已
 * 在 `impl/common/DecompressFactoryRegistration.cpp` 注册了三后端的 valence 三元组。
 *
 * ⚠️ 这里刻意沿用 `impl/PCLTTM/Degree{Three,Six}IsRemovableOperator.h` 这两个
 * **历史 shim**：它们保留了 PCLTTMValence 的度数上下界（与 canonical 的
 * `Level2DegreeThreeIsRemovableOperator` 不同）。换成 canonical 会改变产物字节，
 * 而本次迁移的要求是**逐字节不变**。
 */
#include "algorithm/ISimplificationAlgorithm.h"
#include "algorithm/SimplificationAlgorithmRegistry.h"
#include "core/CompressPipeline.h"
#include "elimination/VertexRemovalEliminateOperator.h"
#include "operator/BasicIsRemovableOperator.h"
#include "operator/UnifiedManifoldIsRemovableOperator.h"

#include "PCLTTM/DegreeSixIsRemovableOperator.h"
#include "PCLTTM/DegreeThreeIsRemovableOperator.h"

namespace pmsf {
namespace {

/// Phase 1 校验用：把注册名临时改成 "<name>_probe"，就能在同一份二进制里让
/// `algorithm` 字段分别命中 legacy 过渡桥与插件路径，做真正的 A/B。
/// Phase 4 删除 legacy 桥后本开关一并移除。
static std::string registeredName(const char* real, const char* probeEnv,
                                  const char* probeName) {
    const char* v = std::getenv(probeEnv);
    return (v && *v) ? std::string(probeName) : std::string(real);
}

class PCLTTMValenceAlgorithm final : public ISimplificationAlgorithm {
  public:
    AlgorithmInfo info() const override {
        return AlgorithmInfo{
            // 名字必须与配置解析后的键一致：normalizeKey 只做小写 + 去空白，
            // **不去下划线**，所以 "PCLTTMValence" → "pclttmvalence"。
            .name = registeredName("pclttmvalence", "PMSF_PROBE_PCLTTM_VALENCE",
                                   "pclttmvalence_probe"),
            .version = 1,
            .primitive = PrimitiveKind::Vertex,
            .selection = SelectionKind::GeometryDriven,
        };
    }

    /// H1：度数上下界 + 流形 + 基础保护。顺序与旧 SelectOptions 一致。
    IsRemovableChain removability() const override {
        IsRemovableChain chain;
        chain.push_back(std::make_shared<DegreeThreeIsRemovableOperator>());
        chain.push_back(std::make_shared<DegreeSixIsRemovableOperator>());
        chain.push_back(std::make_shared<BasicIsRemovableOperator>());
        chain.push_back(std::make_shared<UnifiedManifoldIsRemovableOperator>());
        return chain;
    }

    /// 旧 Builder 对 `AlgoPreset::PCLTTMValence` 设的四个通用开关。
    /// 该算法未显式配置 rounds 时的默认轮数（原写在 src/config 的表里）。
    int defaultRounds() const override { return 48; }

    PipelineTuner pipelineTuner() const override {
        return [](CompressPipelineConfig& cfg, Backend /*backend*/) {
            cfg.updateAfterEliminate = false;
            cfg.recomputeVertexDegreesOnce = true;
            cfg.trackContextLevel = true;
            cfg.headerCarryPrediction = true;
        };
    }
};

}  // namespace
}  // namespace pmsf

PMSF_REGISTER_SIMPLIFICATION_ALGORITHM(pmsf::PCLTTMValenceAlgorithm);
