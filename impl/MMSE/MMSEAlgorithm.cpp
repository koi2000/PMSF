/*
 * @file MMSEAlgorithm.cpp
 * @brief MMSE (Lee et al., ICIP 2011, "3D mesh compression based on dual-ring
 *        prediction and MMSE prediction") 接入契约。
 *
 * 论文 Tbl. tab:hook_loc 里 MMSE 重写的是 H1（12 行）、H4（101 行）、H5（19 行）
 * 与预测器（129 行）。
 *
 *   - H1 removability = Basic → UnifiedManifold → DegreeThree → DegreeSix
 *   - H4 前处理       = MMSEPreprocessor（双环预测的上下文准备）
 *   - 后处理          = MMSEPostProcessor(alpha)（论文的 MMSE 预测残差调整）
 *   - 协议            = mmse_dual_ring（论文的双环预测由该协议承载：valence +
 *                       isPolyBit + 自适应残差三段流）
 *   - 其余全部框架默认
 *
 * 字节核实：见 scripts/tests/verify_algo_migration.sh（同二进制 A/B）。
 */
#include "algorithm/SimplificationAlgorithmRegistry.h"

#include "DegreeSixIsRemovableOperator.h"
#include "DegreeThreeIsRemovableOperator.h"
#include "MMSEPipeline.h"
#include "core/CompressPipeline.h"
#include "operator/BasicIsRemovableOperator.h"
#include "operator/UnifiedManifoldIsRemovableOperator.h"

#include <cmath>
#include <cstdlib>
#include <memory>

namespace pmsf {

namespace {

    /// Phase 1 校验用：把注册名临时改成 "<name>_probe"，就能在同一份二进制里
    /// 让 `algorithm` 字段分别命中 legacy 过渡桥与插件路径，做真正的 A/B。
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

class MMSEAlgorithm final : public ISimplificationAlgorithm {
  public:
    AlgorithmInfo info() const override {
        return AlgorithmInfo{
            .name = registeredName("mmse", "PMSF_PROBE_MMSE", "mmse_probe"),
            .version = 1,
            .primitive = PrimitiveKind::Vertex,
            .selection = SelectionKind::GeometryDriven,
        };
    }

    IsRemovableChain removability() const override {
        IsRemovableChain chain;
        chain.push_back(std::make_shared<BasicIsRemovableOperator>());
        chain.push_back(std::make_shared<UnifiedManifoldIsRemovableOperator>());
        chain.push_back(std::make_shared<DegreeThreeIsRemovableOperator>());
        chain.push_back(std::make_shared<DegreeSixIsRemovableOperator>());
        return chain;
    }

    /// H4：双环预测的编码前准备。
    PreProcessOperatorPtr preProcessOperator() const override {
        return std::make_shared<MMSEPreprocessor>();
    }

    /// 论文的 MMSE 预测残差调整；α 来自 config（默认 0.5）。
    PostProcessOperatorPtr postProcessOperator() const override {
        return std::make_shared<MMSEPostProcessor>(alpha_);
    }

    /// 论文 §3 的双环预测协议。显式 env 可切到其它已实现的分支。
    std::string retriangulationProtocol() const override {
        if (envOn("PMSF_USE_ORIG_BASELINE")) return {};
        if (envOn("PMSF_MMSE_USE_PCLTTM_TAG")) return "pclttm_tag";
        if (envOn("PMSF_MMSE_USE_RETRI_TAG")) return "rd_tag";
        if (envOn("PMSF_MMSE_USE_DUAL_RING")) return "mmse_dual_ring";
        return "mmse_dual_ring";
    }

    /// P18.1a：本算法的残差流按整数位平面设计，**没有未量化（F32）码流布局**
    /// —— `*Tag` 采集器的 `exportRC` 会 `throw`。声明出来，让框架在 config
    /// 派发期就给出可读错误，而不是白跑一遍再深抛（实测约 1 s）。
    bool requiresQuantization() const override { return true; }

    /// H1 / `SelectionPolicy`：论文 §4.2 的 **prioritized gate selection (PGS)**。
    ///
    /// **2026-09-25 起为默认开启**（`PMSF_MMSE_PGS_OFF=1` 可退回框架默认策略）。
    /// 翻转默认依据（四网格实测，quality 两项指标逐位相同）：
    ///   horse   137,152 → 133,152 B（22.630476 → 21.970464 bpv）
    ///   bunny    83,038 →  80,183 B（19.070 → 18.414 bpv）
    ///   LetterA  15,250 →  14,845 B
    ///   fandisk  20,856 →  20,292 B
    /// 质量的 RMSE / Hausdorff **逐位相同**（非「未变差」，是每一位都相等）。
    /// 详见 docs/BPV_REDUCTION_PLAN_2026-09-25.md §15/§16。
    ///
    /// 论文 §4.2：优先选「更可能产生低 valence、尤其 valence-3 顶点」的 gate，
    /// 依据 `V' = V + (v − 6)`。**当前只取第一刀近似：端点 valence 之和**，
    /// 论文的另两项因素（⊕/⊖ 标签、是否来自 null patch）**尚未实现**。
    /// 队列是 `MinErrorComparator` 的 min-heap ⇒ **误差小者先出**，
    /// 故 error 取端点 valence 之和即等价于「低 valence 优先」。

    SelectionPolicy selectionPolicy() const override {
        if (envOn("PMSF_MMSE_PGS_OFF")) {
            return {};
        }
        SelectionPolicy policy;
        // Residual-aware MMSE variant: prefer gates whose removed vertex lies
        // close to its one-ring barycenter (small predicted residual), while
        // retaining the paper's low-valence preference.
        const char* residualSel = std::getenv("PMSF_MMSE_RESIDUAL_SELECT");
        float residualWeight = 0.0f;
        if (residualSel && *residualSel && residualSel[0] != '0') {
            residualWeight = std::strtof(residualSel, nullptr);
            if (!(residualWeight > 0.0f)) residualWeight = 0.05f;
        }
        const char* flatSel = std::getenv("PMSF_MMSE_FLAT_SELECT");
        float flatWeight = 0.0f;
        if (flatSel && *flatSel && flatSel[0] != '0') {
            flatWeight = std::strtof(flatSel, nullptr);
            if (!(flatWeight > 0.0f)) flatWeight = 0.05f;
        }
        policy.errorOverride = [residualWeight, flatWeight](MCGAL::Halfedge* e, MCGAL::Point& /*p*/) -> float {
            if (e == nullptr) return 0.0f;
            MCGAL::Vertex* a = e->vertex();
            MCGAL::Vertex* b = e->end_vertex();
            const float va = (a != nullptr) ? static_cast<float>(a->vertex_degree()) : 0.0f;
            const float vb = (b != nullptr) ? static_cast<float>(b->vertex_degree()) : 0.0f;
            float score = va + vb;
            if (residualWeight > 0.0f && b != nullptr && !b->isRemoved()) {
                double sx = 0, sy = 0, sz = 0;
                int n = 0;
                for (MCGAL::Halfedge* hb : b->halfedges()) {
                    if (hb == nullptr || hb->opposite() == nullptr) continue;
                    MCGAL::Vertex* nb = hb->opposite()->vertex();
                    if (nb == nullptr || nb == b) continue;
                    sx += nb->point()[0]; sy += nb->point()[1]; sz += nb->point()[2];
                    ++n;
                }
                if (n > 0) {
                    const double dx = b->point()[0] - sx / n;
                    const double dy = b->point()[1] - sy / n;
                    const double dz = b->point()[2] - sz / n;
                    score += residualWeight * static_cast<float>(std::sqrt(dx * dx + dy * dy + dz * dz));
                }
            }
            if (flatWeight > 0.0f && b != nullptr && !b->isRemoved()) {
                double nx = 0, ny = 0, nz = 0;
                int fn = 0;
                for (MCGAL::Halfedge* hb : b->halfedges()) {
                    if (hb == nullptr || hb->face() == nullptr) continue;
                    auto n = hb->face()->computeNormal();
                    double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
                    if (len < 1e-12) continue;
                    nx += n[0] / len; ny += n[1] / len; nz += n[2] / len;
                    ++fn;
                }
                if (fn > 0) {
                    double alen = std::sqrt(nx * nx + ny * ny + nz * nz);
                    if (alen > 1e-12) {
                        nx /= alen; ny /= alen; nz /= alen;
                        double rough = 0.0;
                        for (MCGAL::Halfedge* hb : b->halfedges()) {
                            if (hb == nullptr || hb->face() == nullptr) continue;
                            auto n = hb->face()->computeNormal();
                            double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
                            if (len < 1e-12) continue;
                            double dot = (n[0] * nx + n[1] * ny + n[2] * nz) / len;
                            rough += 1.0 - dot;
                        }
                        score += flatWeight * static_cast<float>(rough);
                    }
                }
            }
            return score;
        };
        return policy;
    }

    /// 算法私有配置：α 来自 config.json 的 algorithm_options（缺省 0.5）。
    void configure(const nlohmann::json& options) override {
        if (options.contains("alpha") && options.at("alpha").is_number()) {
            alpha_ = options.at("alpha").get<double>();
        }
    }

  private:
    double alpha_ = 0.5;
};

}  // namespace
}  // namespace pmsf

PMSF_REGISTER_SIMPLIFICATION_ALGORITHM(pmsf::MMSEAlgorithm);
