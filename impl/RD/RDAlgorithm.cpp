/*
 * @file RDAlgorithm.cpp
 * @brief RD (Ahn et al. 2011, "R-D optimized progressive compression of 3D meshes
 *        using prioritized gate selection and curvature prediction") 接入契约。
 *
 * 论文 Tbl. tab:hook_loc 里 RD 重写的是 H1（12 行）+ H2（97 行）。
 *
 *   - H2 errorMetric    = RDErrorMetricImpl（6 级 tag-priority 梯度，论文 §3.2）
 *   - declareProperties = vpTag / epError 两个网格属性
 *   - H1 removability   = Basic → UnifiedManifold → DegreeThree → DegreeSix
 *   - 协议              = rd_tag（论文 §3.3 的 prioritized gate selection +
 *                         curvature prediction 由 RD_TAG 协议承载），
 *                         自适应残差流；PMSF_USE_ORIG_BASELINE=1 可关掉
 *   - 其余全部框架默认（H3 环三角化、H4 无移位、选择器优先队列、压缩主循环、
 *                        序列化、解压、CLI、配置解析）
 *
 * 字节兼容：与原 `impl/RD/PMSFEntry.cpp` 的 runRDCompress 逐字节一致
 * （由 bench 的 compressed_size_bytes 验证）。
 */
#include "algorithm/SimplificationAlgorithmRegistry.h"

#include "RDErrorMetric.h"
#include "RDPGS.h"
#include "core/CompressPipeline.h"
#include "core/SelectionPolicy.h"
#include "elimination/VertexRemovalEliminateOperator.h"

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



bool envOn(const char* name) {
    const char* v = std::getenv(name);
    return v && *v && std::string(v) != "0";
}

class RDAlgorithm final : public ISimplificationAlgorithm {
  public:
    /// RDErrorMetricImpl 按引用持有 PropertyHandles；句柄必须活过度量的使用期。
    mutable rd_impl::PropertyHandles handles_;

    AlgorithmInfo info() const override {
        return AlgorithmInfo{
            .name = registeredName("rd", "PMSF_PROBE_RD", "rd_probe"),
            .version = 1,
            .primitive = PrimitiveKind::Vertex,
            .selection = SelectionKind::GeometryDriven,
        };
    }

    /// H1：与历史实现一致的顺序（基础 → 流形 → 度 3 → 度 6）。
    IsRemovableChain removability() const override {
        IsRemovableChain chain;
        chain.push_back(std::make_shared<BasicIsRemovableOperator>());
        chain.push_back(std::make_shared<UnifiedManifoldIsRemovableOperator>());
        chain.push_back(std::make_shared<DegreeThreeIsRemovableOperator>());
        chain.push_back(std::make_shared<DegreeSixIsRemovableOperator>());
        return chain;
    }

    /// task-19 P2a：`PMSF_RD_PGS=1` 时选择器切到 GreedySelector —— 框架里现成的
    /// A&D 式 **gate-queue conquest**（FIFO gateQueue + conquered flags +
    /// SelectionPolicy::candidateScore）。默认 PriorityError 不变（逐字节）。
    /// 注意：真正接管 rd_tag 压缩的是 impl/common/RetriOverride.h 的覆盖路径，
    /// 但 select 对象由注册表在覆盖之前按本 hook 构建 ⇒ 这里切换即可生效。
    SelectorKind selector() const override {
        return rd_impl::pgsEnabled() ? SelectorKind::Greedy
                                     : SelectorKind::PriorityError;
    }

    /// task-19 P2a：GreedySelector 在一个 gate face 的环上挑候选时用
    /// `candidateScore`（**越大越优先**）。这里装论文 §3.2 的 6 级 priority。
    SelectionPolicy selectionPolicy() const override {
        SelectionPolicy policy;
        if (!rd_impl::pgsEnabled()) return policy;
        policy.candidateScore = [](MCGAL::Halfedge* h) -> float {
            return static_cast<float>(rd_impl::pgsPriority(h));
        };
        return policy;
    }

    /// vpTag（PLUS/MINUS 顶点标签）与 epError 属性。
    /// 注册在 H2 的工厂里做一次即可 —— 框架在网格就绪后会先调 declareProperties()
    /// 再调 errorMetric() 工厂，两边都注册会造成重复 add_property。
    PropertyInitFn declareProperties() const override { return {}; }

    /// H2 —— RD 的 6 级 tag-priority 误差。
    /// task-19 P1：`PMSF_RD_PGS=1` 时改用真 PGS 度量（tag 演化 + conquered
    /// region，实现在 RDPGS.h）；默认 env 关闭 ⇒ 原 RDErrorMetricImpl 不变。
    ErrorMetricFactory errorMetric() const override {
        return [this](std::shared_ptr<MCGAL::Mesh> mesh) -> std::shared_ptr<ErrorMetric> {
            if (rd_impl::pgsEnabled()) {
                rd_impl::pgsRegisterDiag();
                return std::make_shared<rd_impl::RDPGSMetric>();
            }
            rd_impl::registerProperties(handles_, mesh);
            return std::make_shared<rd_impl::RDErrorMetricImpl>(handles_);
        };
    }

    /// task-19 P1：PGS 的 eliminate 侧改动落在实际被 rd_tag 覆盖路径使用的
    /// `RDTagEliminateOperator`（env PMSF_RD_PGS=1 时做 tag 演化/消除序号），
    /// 因此这里不覆盖 eliminate（保持框架默认，避免双路径歧义）。
    /// 度量侧见 errorMetric() 的 RDPGSMetric 分支。

    /// 协议覆盖：默认 rd_tag；PMSF_USE_ORIG_BASELINE=1 时回到无覆盖的 baseline。
    /// P18.1a：本算法的残差流按整数位平面设计，**没有未量化（F32）码流布局**
    /// —— `*Tag` 采集器的 `exportRC` 会 `throw`。声明出来，让框架在 config
    /// 派发期就给出可读错误，而不是白跑一遍再深抛（实测约 1 s）。
    bool requiresQuantization() const override { return true; }

    std::string retriangulationProtocol() const override {
        if (envOn("PMSF_USE_ORIG_BASELINE")) return {};
        return "rd_tag";
    }

    RetriProtocolTuning retriangulationTuning() const override {
        RetriProtocolTuning t;
        t.adaptiveResidual = true;
        t.defaultQuantBits = 11;
        t.quantBitsEnv = "PMSF_RDTAG_QUANT_BITS";
        return t;
    }
};

}  // namespace
}  // namespace pmsf

PMSF_REGISTER_SIMPLIFICATION_ALGORITHM(pmsf::RDAlgorithm);
