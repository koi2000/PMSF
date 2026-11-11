/*
 * @file PCATMAlgorithm.cpp
 * @brief PCATM (Cohen-Or et al., IEEE Vis 1999) 接入 ISimplificationAlgorithm 契约。
 *
 * 论文 Tbl. tab:hook_loc 里 PCATM 重写的是 H1（70 行）+ H3（372 行）。本文件中：
 *   - H1 removability  = DegreeSix → Basic → UnifiedManifold（顺序与论文一致）
 *   - 消除算子          = PCATMEliminateOperator（Z-shape 约束的顶点删除）
 *   - H8 边界           = configurePcatmDefaultPMSFBoundary
 *   - 管线微调          = 论文所述的 select-then-extract 种子 + 每轮度重算 +
 *                         上下文层级跟踪 + MIS + 前处理一次/后处理每轮
 *   - 其余（H2 误差度量不需要 —— 连通驱动；H3 走框架确定性环三角化 +
 *     PCATMEliminate 内部的 Z-shape 重建）全部使用框架默认。
 *
 * 字节兼容说明：DegreeSix 用的是 `impl/PCATM/DegreeSixIsRemovableOperator.h`，
 * 即仓库里为**字节兼容**保留的历史 shim（规范版本在
 * `src/operator/Level2DegreeNIsRemovableOperator.h`，但两者度界不同）。
 * Phase 2 的 SymbolCodec 落地后应换成规范版本并重跑回归确认差异。
 */
#include "algorithm/SimplificationAlgorithmRegistry.h"

#include "common/strict/PCATMStrictScoring.h"

#include "DegreeSixIsRemovableOperator.h"
#include "PCATM/elimination/PCATMEliminateOperator.h"
#include "DefaultPMSFBoundaryImpl.h"
#include "core/CompressPipeline.h"
#include "operator/BasicIsRemovableOperator.h"
#include "operator/UnifiedManifoldIsRemovableOperator.h"

#include <cstdlib>
#include <memory>

namespace pmsf {
namespace {

    /// Phase 1 校验用：把注册名临时改成 "<name>_probe"，就能在同一份二进制里
    /// 让 `algorithm` 字段分别命中 legacy 过渡桥与插件路径，做真正的 A/B
    /// （跨二进制的基线比对受 TU 集合敏感性影响，不可靠）。
    /// Phase 4 删除 legacy 桥后本开关一并移除。
    static std::string registeredName(const char* real, const char* probeEnv,
                                      const char* probeName) {
        const char* v = std::getenv(probeEnv);
        return (v && *v) ? std::string(probeName) : std::string(real);
    }



class PCATMAlgorithm final : public ISimplificationAlgorithm {
  public:
    AlgorithmInfo info() const override {
        return AlgorithmInfo{
            .name = registeredName("pcatm", "PMSF_PROBE_PCATM", "pcatm_probe"),
            .version = 1,
            .primitive = PrimitiveKind::Vertex,
            .selection = SelectionKind::ConnectivityDriven,
        };
    }

    /// H1：论文的 Z-shape 可消除性链 —— 度 6 优先、再基础可消除、再流形保护。
    IsRemovableChain removability() const override {
        IsRemovableChain chain;
        chain.push_back(std::make_shared<DegreeSixIsRemovableOperator>());
        chain.push_back(std::make_shared<BasicIsRemovableOperator>());
        chain.push_back(std::make_shared<UnifiedManifoldIsRemovableOperator>());
        return chain;
    }

    /// 论文 §3 的 Z-shape 约束消除（含 barycenter 的 bit-exact 重建逻辑）。
    EliminateOperatorPtr eliminate() const override {
        return std::make_shared<PCATMEliminateOperator>();
    }

    /// 协议覆盖：配置驱动入口也必须与历史 AlgoPreset 入口保持同一优先级。
    /// 这些开关既影响候选/消除策略，也决定 collector/reconstructor 的 wire
    /// protocol；只在 selectionPolicy() 中读取会造成“脚本声称 strict，
    /// 产物却按默认 PCATM 编码”的口径错配。
    std::string retriangulationProtocol() const override {
        if (pcatm_detail::envOn("PMSF_PCATM_USE_STRICT_PAPER")) return "pcatm_strict";
        if (pcatm_detail::envOn("PMSF_PCATM_USE_PURE_K3")) return "pcatm_pure";
        if (pcatm_detail::envOn("PMSF_PCATM_USE_PURE_PAPER")) return "pcatm_paper";
        if (pcatm_detail::envOn("PMSF_PCATM_USE_RETRI_ZCOLOR")) return "pcatm_zcolor";
        if (pcatm_detail::envOn("PMSF_PCATM_USE_RETRI_COLOR")) return "pcatm_color";
        return {};
    }

    /// H8：PCATM 的边界侧信道。
    /// 选择策略：严格复现模式（`PMSF_PCATM_USE_STRICT_PAPER`，可再叠加
    /// `PMSF_PCATM_AGGRESSIVE_REMOVABLE`）下换成论文的候选打分与误差项。
    /// 两个环境变量都只从进程外设置，因此这里一次性解析与历史上「首次调用时
    /// 解析」等价。未开严格模式 ⇒ 返回全默认策略 ⇒ 与历史行为逐字节一致。
    SelectionPolicy selectionPolicy() const override {
        return pcatm_detail::makeSelectionPolicy();
    }

    BoundaryConfigurator boundaryConfigurator() const override {
        return [](CompressPipelineConfig& cfg) { configurePcatmDefaultPMSFBoundary(cfg); };
    }

    /// 论文 §4 的管线行为：select-then-extract 种子、每轮度重算、上下文层级、
    /// 独立集选择，前处理一次 + 后处理每轮。
    PipelineTuner pipelineTuner() const override {
        return [](CompressPipelineConfig& cfg, Backend /*backend*/) {
            cfg.headerCarryPrediction = true;
            cfg.seedStrategy = CompressPipelineConfig::SeedStrategy::SelectThenExtract;
            cfg.recomputeVertexDegreesOnce = true;
            cfg.trackContextLevel = true;
            cfg.enableIndependentSet = true;
            cfg.updateAfterEliminate = false;
            cfg.preProcessPosition = CompressPipelineConfig::PreProcessPosition::Once;
            cfg.postProcessPosition = CompressPipelineConfig::PostProcessPosition::EveryRound;
        };
    }
};

}  // namespace
}  // namespace pmsf

PMSF_REGISTER_SIMPLIFICATION_ALGORITHM(pmsf::PCATMAlgorithm);
