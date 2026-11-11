/*
 * @file QEMAlgorithm.cpp
 * @brief QEM (Garland & Heckbert 1997) 接入 ISimplificationAlgorithm 契约。
 *
 * 这是设计文档 §2.1.4 点名的**试点**：论文 Tbl. tab:hook_loc 里 QEM 只重写 H2
 * （90 行），其余全是框架标准行为。本文件的算法特有代码就是 H2 那一段。
 *
 *   - H2 errorMetric    = OptimalPlacementQuadricError（论文的二次误差 + 最优放置）
 *   - declareProperties = quadric 网格属性注册
 *   - H1 removability   = IsFlip(H6) → Basic → UnifiedManifold → Boundary
 *   - H8 边界           = configureQemPMSFBoundary
 *   - 符号布局          = BasicSymbolCollectOperator（对称的 Basic reader）
 *   - 量化开关          = 保留 adjustment（QEM 的 optimal placement 是算法语义的
 *                        一部分，不能在协议覆盖里被关掉）
 *   - 协议              = rd_tag（环境变量 PMSF_USE_ORIG_BASELINE 可关掉）
 *
 * 其余（H3 环三角化、H4 无移位、H7 不分区、选择器、压缩主循环、序列化、解压、
 * CLI、配置解析）全部由框架承担。
 *
 * 字节兼容：与原 `impl/QEM/PMSFEntry.cpp` 的 runQEMCompress 逐字节一致
 * （由 bench 的 compressed_size_bytes 验证）。
 */
#include "algorithm/SimplificationAlgorithmRegistry.h"

#include "DefaultPMSFBoundaryImpl.h"
#include "operator/IsFlipOperator.h"
#include "QEMQuadric.h"
#include "QEMPaperOperators.h"
#include "compress/BasicSymbolCollectOperator.h"
#include "core/CompressPipeline.h"
#include "metric/OptimalPlacementQuadricError.h"
#include "operator/BasicIsRemovableOperator.h"
#include "operator/BoundaryIsRemovableOperator.h"
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



bool envOn(const char* name) {
    const char* v = std::getenv(name);
    return v && *v && std::string(v) != "0";
}

class QEMAlgorithm final : public ISimplificationAlgorithm {
  public:
    /// 属性句柄必须活得比度量久：makeQuadricAccessor/Refresher 内部按引用捕获
    /// `h`，而 ErrorMetric 在整个 pipeline 运行期间持有这些 lambda。
    /// 算法对象由注册表按次创建、生命周期覆盖整次 compress()，因此放成员即可。
    mutable qem_impl::PropertyHandles handles_;

    // Explicit configuration selects the original 1997 equations. Legacy
    // rd_tag and reference-calibration evidence retain their own identities.
    bool paper1997_ = false;
    qem_paper1997::Configuration paperOptions_;

    void configure(const nlohmann::json& options) override {
        const auto execution = options.value("execution", std::string("legacy"));
        if (execution != "legacy" && execution != "paper1997") {
            throw std::runtime_error("QEM execution must be legacy or paper1997");
        }
        paper1997_ = execution == "paper1997";
        if (paper1997_) paperOptions_ = qem_paper1997::configuration(options);
    }

    void compress(const PMSFConfig& cfg) override {
        if (paper1997_) { qem_paper1997::compress(cfg, paperOptions_); return; }
        ISimplificationAlgorithm::compress(cfg);
    }

    void decompress(const PMSFConfig& cfg) override {
        std::string path = cfg.input;
        if (cfg.backend == Backend::Paper && !std::filesystem::is_regular_file(path)) {
            path = std::filesystem::path(path).replace_extension().string() + "_paper.parquet";
        }
        if (paper1997_ || qem_paper1997::isOwnArtifact(path) ||
            (cfg.backend == Backend::Paper && qem_paper1997::isPaperArtifact(path))) {
            qem_paper1997::decompress(cfg); return;
        }
        ISimplificationAlgorithm::decompress(cfg);
    }


    AlgorithmInfo info() const override {
        return AlgorithmInfo{
            .name = registeredName("qem", "PMSF_PROBE_QEM", "qem_probe"),
            .version = 1,
            .primitive = PrimitiveKind::Vertex,
            .selection = SelectionKind::GeometryDriven,
        };
    }

    /// H1 + H6：翻转检测（H6 以可消除性算子的形式参与链）+ 基础/流形/边界保护。
    IsRemovableChain removability() const override {
        IsRemovableChain chain;
        // perf（2026-11-09 QEM 实测：IsFlipOperator::check 占 31.6% cycles，
        // ~410 ms/次压缩）：不可消除性算子是纯 AND（HalfedgeSelectionQueue 逐项
        // 短路，各算子只读），与顺序无关。把最贵的翻转检测放到链尾，让
        // Basic/Unified/Boundary 先短路拒绝大部分候选，翻转检测只对可能通过的
        // 候选执行 ⇒ 判定结果逐字节不变，只是少调用。
        chain.push_back(std::make_shared<BasicIsRemovableOperator>());
        chain.push_back(std::make_shared<UnifiedManifoldIsRemovableOperator>());
        chain.push_back(std::make_shared<BoundaryIsRemovableOperator>());
        chain.push_back(std::make_shared<IsFlipOperator>());
        return chain;
    }

    /// quadric 属性（论文 §3：每顶点 Q 矩阵 += 邻接面平面方程外积）。
    PropertyInitFn declareProperties() const override {
        return [this](std::shared_ptr<MCGAL::Mesh> mesh) {
            qem_impl::registerQuadricProperties(handles_, mesh);
        };
    }

    /// H2 —— QEM 唯一的算法核心。
    ErrorMetricFactory errorMetric() const override {
        return [this](std::shared_ptr<MCGAL::Mesh> mesh) -> std::shared_ptr<ErrorMetric> {
            qem_impl::registerQuadricProperties(handles_, mesh);
            return std::make_shared<OptimalPlacementQuadricError>(
                qem_impl::makeQuadricAccessor(handles_, mesh->meshId()),
                qem_impl::makeQuadricRefresher(handles_, mesh->meshId()));
        };
    }

    /// QEM 的度量把最优放置写进 target（论文 §4 的 optimal vertex placement）。
    ErrorAccessStyle errorAccessStyle() const override {
        return ErrorAccessStyle::OptimalPlacement;
    }

    /// QEM 的 Q 矩阵在每次消除后必须刷新。
    bool needsAfterEliminateRefresh() const override { return true; }

    // 不重写 collectorFactory()：用框架默认的 OptimizedSymbolCollectOperator。
    //
    // 历史实现这里返回 BasicSymbolCollectOperator（与 BasicSymbolReadOperator 成对），
    // 但那条配对在 `prediction: true` 下是坏的：BasicSymbolCollectOperator 没有实现
    // `exportToBufferWithRangeCoder`（基类默认返回 0），序列化器走预测分支就写出
    // **0 字节的轮数据**，二进制产物里只剩 base mesh。实测 30 轮 bunny 只有 3252 B
    // （同配置列式 285 KB），解码端从缓冲区之外读数据，第 4 轮就撞
    // `Facet::setSplittable` 断言。
    //
    // 与其余 8 个算法的二进制路径对齐：Optimized 采集 ↔ Optimized 读取
    // （解压侧注册见 DecompressFactoryRegistration.cpp 的 qem/Binary）。

    BoundaryConfigurator boundaryConfigurator() const override {
        return [](CompressPipelineConfig& cfg) { configureQemPMSFBoundary(cfg); };
    }

    /// perf（2026-11-09 QEM 实测）：QEM 码流不依赖 vertex proxy canonical 化
    /// （bunny/LetterA/fandisk 上 artifact 逐字节不变），跳过每轮 O(V) 的顶点
    /// canonicalize；面 proxy 仍 canonicalize（必需）。用户可显式设 0 关闭。
    CompressOptionsTuner compressOptionsTuner() const override {
        return [](CompressOptions&) {
            setenv("PMSF_SKIP_VERTEX_CANON", "1", 0);
        };
    }

    /// QEM 的 optimal placement 是算法语义的一部分 ⇒ 保留 adjustment 流。
    CompressionProfile compressionProfile() const override {
        CompressionProfile profile;
        profile.adjustment = true;
        return profile;
    }

    /// 协议覆盖：默认 rd_tag；PMSF_USE_ORIG_BASELINE=1 时回到无覆盖的 baseline。
    /// P18.1a：本算法的残差流按整数位平面设计，**没有未量化（F32）码流布局**
    /// —— `*Tag` 采集器的 `exportRC` 会 `throw`。声明出来，让框架在 config
    /// 派发期就给出可读错误，而不是白跑一遍再深抛（实测约 1 s）。
    bool requiresQuantization() const override { return !paper1997_; }

    std::string retriangulationProtocol() const override {
        if (envOn("PMSF_USE_ORIG_BASELINE")) return {};
        return "rd_tag";
    }

    RetriProtocolTuning retriangulationTuning() const override {
        RetriProtocolTuning t;
        t.keepAdjustment = true;
        // 与历史一致：无人为设置量化环境变量时保持默认位宽。
        t.requireQuantBitsEnv = true;
        t.quantBitsEnv = "PMSF_RDTAG_QUANT_BITS";
        t.altQuantBitsEnv = "PMSF_APTM_QUANT_BITS";
        return t;
    }
};

}  // namespace
}  // namespace pmsf

PMSF_REGISTER_SIMPLIFICATION_ALGORITHM(pmsf::QEMAlgorithm);
