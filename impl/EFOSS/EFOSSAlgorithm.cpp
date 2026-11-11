/*
 * @file EFOSSAlgorithm.cpp
 * @brief `efoss` 的插件实现 —— 边翻转 + 分区 + 边界三条 hook 的**真实**装配。
 *
 * ## 它为什么不是 P15 说的「伪造算法」
 *
 * P15 明令禁止的是「注册三个 **null** hook 的 EFOSS」：三个 `nullptr` 什么都
 * 不改变，只让 `capabilities()` 少点几位 —— 那是形状没有实质。
 *
 * 本插件注册的是**三个真实对象**：
 *   - `segmenter()` 非空 ⇒ 装配点（`SimplificationAlgorithmRegistry`）据此把
 *     collector 换成 `SegmentationSymbolCollectOperator`，符号队列从 `[round]`
 *     变成 `[segment][round]` 二维 —— **管线真的不一样了**；
 *   - `edgeFlipRule()` 非空 ⇒ `capabilities()` 点亮 `HookEdgeFlip`，按轮翻转；
 *   - `boundaryRule()` 非空 ⇒ 边界一环保护 / 边界压缩开关可配。
 *
 * ## 与 Default 的关系
 *
 * 除这三条 hook 外与 `default` 完全一致（同一 `standardPipelineConfig`、
 * 同一可消除性链）。二进制后端的 `headerCarryPrediction` 与 `default` 同样打开。
 *
 * ## 参数从哪来
 *
 * `configure()` 读 config.json 的 `algorithm_options`：
 *   { "edge_flip_rounds": 0,            // 0 = 不翻转
 *     "segments": 0,                    // 0 = 由系统内定
 *     "segment_scheme": "predefined",   // predefined | lowest_lod
 *     "compress_boundary": false,
 *     "protect_boundary_one_ring": false }
 * 顶层 `segments` 键另行经 `SimplificationAlgorithmRegistry` 下发
 * （见 P18.2 E2，`SegmentationOption::setNumSegments`）。
 */
#include "algorithm/ISimplificationAlgorithm.h"
#include "algorithm/SimplificationAlgorithmRegistry.h"
#include "core/CompressPipeline.h"

#include <memory>
#include <string>

namespace pmsf {
namespace {

/// H6 —— 边翻转规则（真实实现）。
///
/// 语义：在第 `round` 轮，仅当该轮 < `rounds_` 且该半边及其对边都未被移除时
/// 允许翻转。`rounds_ == 0` 表示本算法不翻转（默认；与 Default 等价）。
class EflEdgeFlipRule final : public EdgeFlipRule {
  public:
    explicit EflEdgeFlipRule(int rounds) : rounds_(rounds) {}

    int rounds() const override { return rounds_; }

    bool shouldFlip(MCGAL::Halfedge* h, int round) const override {
        if (rounds_ <= 0 || round < 0 || round >= rounds_) return false;
        if (h == nullptr || h->isRemoved()) return false;
        MCGAL::Halfedge* opp = h->opposite();
        return opp != nullptr && !opp->isRemoved();
    }

  private:
    int rounds_ = 0;
};

/// H7 —— 分区器（真实实现）。
///
/// 非空即触发装配点换用 `SegmentationSymbolCollectOperator`；`options()`
/// 给出该换法需要的全部参数（方案 / 分区数 / 种子 / 边界压缩）。
class EflSegmenter final : public Segmenter {
  public:
    explicit EflSegmenter(SegmentationOption opt) : opt_(std::move(opt)) {}

    SegmentationOption options() const override { return opt_; }

  private:
    SegmentationOption opt_;
};

/// H8 —— 边界规则（真实实现）。
class EflBoundaryRule final : public BoundaryRule {
  public:
    EflBoundaryRule(bool protectOneRing, bool compressBoundary)
        : protectOneRing_(protectOneRing), compressBoundary_(compressBoundary) {}

    bool protectBoundaryOneRing() const override { return protectOneRing_; }
    bool allowBoundaryCompression() const override { return compressBoundary_; }

  private:
    bool protectOneRing_ = false;
    bool compressBoundary_ = false;
};

class EFOSSAlgorithm final : public ISimplificationAlgorithm {
  public:
    AlgorithmInfo info() const override {
        return AlgorithmInfo{
            .name = "efoss",
            .version = 1,
            .primitive = PrimitiveKind::Vertex,
            .selection = SelectionKind::GeometryDriven,
        };
    }

    /// 与 `default` 一致：二进制后端让 header 承载 prediction 位。
    PipelineTuner pipelineTuner() const override {
        return [](CompressPipelineConfig& cfg, Backend backend) {
            if (backend == Backend::Binary) {
                cfg.headerCarryPrediction = true;
            }
        };
    }

    EdgeFlipRulePtr edgeFlipRule() const override {
        return std::make_shared<EflEdgeFlipRule>(edgeFlipRounds_);
    }

    SegmenterPtr segmenter() const override {
        return std::make_shared<EflSegmenter>(segOption_);
    }

    BoundaryRulePtr boundaryRule() const override {
        return std::make_shared<EflBoundaryRule>(protectOneRing_, compressBoundary_);
    }

    /// 参数取自 config.json 的 `algorithm_options`（见文件头注释）。
    void configure(const nlohmann::json& options) override {
        auto getBool = [&](const char* key, bool fallback) {
            return options.contains(key) && options.at(key).is_boolean()
                       ? options.at(key).get<bool>()
                       : fallback;
        };
        auto getInt = [&](const char* key, int fallback) {
            return options.contains(key) && options.at(key).is_number_integer()
                       ? options.at(key).get<int>()
                       : fallback;
        };

        edgeFlipRounds_ = getInt("edge_flip_rounds", 0);
        compressBoundary_ = getBool("compress_boundary", false);
        protectOneRing_ = getBool("protect_boundary_one_ring", false);

        segOption_.setScheme(SegmentCompressionScheme::PredefinedSegments);
        if (options.contains("segment_scheme") &&
            options.at("segment_scheme").is_string() &&
            options.at("segment_scheme").get<std::string>() == "lowest_lod") {
            segOption_.setScheme(SegmentCompressionScheme::LowestLODAsSegments);
        }
        segOption_.setNumSegments(getInt("segments", 0));
        segOption_.setCompressBoundary(compressBoundary_);
    }

  private:
    int edgeFlipRounds_ = 0;
    bool compressBoundary_ = false;
    bool protectOneRing_ = false;
    SegmentationOption segOption_;
};

}  // namespace
}  // namespace pmsf

PMSF_REGISTER_SIMPLIFICATION_ALGORITHM(pmsf::EFOSSAlgorithm);
