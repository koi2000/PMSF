/*
 * @file APTMAlgorithm.cpp
 * @brief APTM (Cheng et al., "Anchors-based lossless compression of progressive
 *        triangle meshes") 接入 ISimplificationAlgorithm 契约。
 *
 * 论文 Tbl. tab:hook_loc 里 APTM 重写的是 H1（237 行，占绝对多数）。
 *
 *   - H1 removability        = Basic → UnifiedManifold → DegreeThree → DegreeSix
 *   - 前处理算子             = AnchorPreprocessor（论文的 anchor 判定与保锚）
 *   - H8 边界                = configureAptmPMSFBoundary
 *   - 管线微调               = select-then-extract 种子 + 上下文层级 +
 *                              每轮 anchor 前处理
 *   - 其余全部框架默认（H3 环三角化、H4 无移位、优先队列选择器、压缩主循环、
 *                       序列化、解压、CLI、配置解析）
 *
 * 默认**不走协议覆盖**（与历史一致：APTM 的 RD_TAG bridge 在 decoder 端有
 * residual round-context 计数不匹配的问题，只作为显式 opt-in）。
 * 设 PMSF_APTM_USE_RETRI_TAG=1 时启用 rd_tag 协议。
 *
 * 字节兼容：与原 `impl/APTM/PMSFEntry.cpp` 的 runAPTMCompress 逐字节一致。
 *
 * ---------------------------------------------------------------------------
 * P1（2026-10-22，WS-B3）：`PMSF_APTM_USE_APTM_TAG=1` —— 论文 coder 补齐路径的第一段。
 *
 * 动机（见 impl/APTM/AUDIT_2026-10-22.md §4 / docs/memory/PHASE1_PHASE2_PLAN.md §1.1）：
 * 忠实 default = 34.052 bpv（论文 17.80 → 1.913×），分量 connectivity 13.17 bpv
 * （= 论文 C=4.9 的 2.69×），因为 default 走的是通用 Optimized 三段流
 * （facet/edge/point），**不是** APTM 论文所基于的 A&D valence-conquest。
 * P1 = 换用框架内已验证 round-trip 的 A&D/tag patch-conquest 底部
 * （`PCLTTMTagSymbolCollectOperator` 的 valence + fusedFaceSym 连通性），
 * 几何先沿用 tag 的 bit-plane residual 流；P2 再换论文 §4.2 的 Frenet+polar θ/φ。
 *
 * 与「借协议覆盖」的区别（为什么不复用 PMSF_APTM_USE_RETRI_TAG）：
 *   1. RETRI_TAG 是 RD_Tag 别名（协议覆盖路径把 collector 换成 RDTag、round 语义
 *      按 RD 调参），产物是 RD 的字节，不是 APTM；已被 Lead 裁决禁用。
 *   2. PCLTTM Tag 协议覆盖路径会**丢 anchor 语义**：它自建 `defaultColumnar()`
 *      管线配置，不经过本类 `pipelineTuner()` 的 `preProcessPosition=EveryRound`，
 *      于是 AnchorPreprocessor 只在第 0 轮生效、后续轮 anchor 标志被 resetState
 *      清掉（anchor 可被删）。APTM 论文 §3/§4.1 的核心贡献正是「anchor 必须
 *      每轮保持不可删、进 base mesh、多送 null_patch」。本路径走标准管线，
 *      anchor 前处理每轮生效。
 *   3. 走标准管线还让 collectorFactory hook 可以换成 APTM 自己的子类
 *      （P2 的 polar coder），而不改 impl/common / impl/PCLTTM。
 *
 * 默认路径零改动纪律：`PMSF_APTM_USE_APTM_TAG` 未设时，下面每个 hook 的返回值
 * 与加入本路径之前**逐项相同**（collectorFactory = {}、eliminate = nullptr、
 * requiresQuantization = false、beforeCompressLoop = {}、pipelineTuner 原三行），
 * 因此默认 84,496 B / 34.052 bpv 逐字节不变（脚本会 md5 复核）。
 * ---------------------------------------------------------------------------
 */
#include "algorithm/SimplificationAlgorithmRegistry.h"

#include "APTMPipeline.h"
#include "AnchorPreprocessor.h"
#include "DefaultPMSFBoundaryImpl.h"
#include "Conquest/decompress/ConquestSymbolReadOperator.h"
#include "Conquest/elimination/ConquestNoopEliminateOperator.h"
#include "Conquest/reconstruct/ConquestReconstructOperator.h"
#include "compress/APTMConquestCollector.h"
#include "hooks/APTMConquestGeometryHook.h"
#include "compress/APTMTagSymbolCollectOperator.h"
#include "core/CompressPipeline.h"
#include "core/DecompressPipeline.h"
#include "decompress/APTMSimLossyDeserializer.h"
#include "decompress/APTMTagSymbolReadOperator.h"
#include "decompress/ColumnarDeserializeOperator.h"
#include "elimination/PCLTTMTagEliminateOperator.h"
#include "predict/RDPredictor.h"  // MCGAL::kVPropRDR1（tag residual 的二阶预测器状态）
#include "PCLTTM/reconstruct/PCLTTMTagReconstructOperator.h"

#include "ContextPool.h"
#include "Mesh.h"
#include "header/PMSFHeader.h"

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>

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

/// P1 路径开关（`PMSF_APTM_USE_APTM_TAG=1`）。未设 ⇒ 默认 APTM 字节路径。
bool aptmTagPathOn() { return aptm_tag::aptmTagPathEnabled(); }

/// P3 路径开关（`PMSF_APTM_USE_CONQUEST_P3=1`）：A&D valence/null_patch
/// connectivity（复用框架已验证的 `FrontierConquestEngine` + `decodeLayer`）
/// + APTM 锚点（AnchorVetoPredicate → null_patch、锚点进 base mesh）。
/// 优先级：conquest P3 > tag P1 > 默认。未设 ⇒ 默认路径逐字节不变。
bool aptmConquestP3On() {
    const char* v = std::getenv("PMSF_APTM_USE_CONQUEST_P3");
    return v && *v && std::string(v) != "0";
}

/// P1 路径的量化位宽解析（与框架/其它 tag 路径同口径）：
///   PMSF_APTM_QUANT_BITS > PMSF_RDTAG_QUANT_BITS > 12。
int resolveTagQuantBits() {
    int qb = 12;
    if (const char* e = std::getenv("PMSF_APTM_QUANT_BITS")) {
        qb = std::atoi(e);
    } else if (const char* e = std::getenv("PMSF_RDTAG_QUANT_BITS")) {
        qb = std::atoi(e);
    }
    return qb;
}

/// 与框架标准 preInitPreparation 相同的量化落地，但位宽走 APTM 自己的 env。
/// （标准实现只认 PMSF_DEFAULT_QUANT_BITS；此处保留其语义并允许 APTM 覆盖。）
void prepareTagQuantization(std::shared_ptr<MCGAL::Mesh> mesh,
                            const CompressOptions& options) {
    if (!mesh) return;
    mesh->enableQuantization = options.isEnableQuantization();
    if (!options.isEnableQuantization()) return;
    const int qb = resolveTagQuantBits();
    if (qb < 8 || qb > 16) {
        std::fprintf(stderr, "[APTM-Tag] ignore invalid quantBits=%d\n", qb);
        return;
    }
    mesh->i_nbQuantBits = static_cast<unsigned>(qb);
    float maxRange = 0.0f;
    for (int axis = 0; axis < 3; ++axis) {
        maxRange = std::max(maxRange, mesh->bboxMax[axis] - mesh->bboxMin[axis]);
    }
    mesh->f_quantStep = maxRange / static_cast<float>(1u << mesh->i_nbQuantBits);
    std::fprintf(stderr, "[APTM-Tag] quantBits=%u quantStep=%g\n",
                 mesh->i_nbQuantBits, mesh->f_quantStep);
}

class APTMAlgorithm final : public ISimplificationAlgorithm {
  public:
    AlgorithmInfo info() const override {
        return AlgorithmInfo{
            .name = registeredName("aptm", "PMSF_PROBE_APTM", "aptm_probe"),
            .version = 1,
            .primitive = PrimitiveKind::Vertex,
            .selection = aptmConquestP3On() ? SelectionKind::ConnectivityDriven
                                            : SelectionKind::GeometryDriven,
        };
    }

    /// H1 —— APTM 的算法主体：锚点约束把「度 3/度 6 + 流形」串起来。
    IsRemovableChain removability() const override {
        if (aptmConquestP3On()) return ISimplificationAlgorithm::removability();
        if (std::getenv("APTM_DEFAULT_CHAIN")) return ISimplificationAlgorithm::removability();
        IsRemovableChain chain;
        chain.push_back(std::make_shared<BasicIsRemovableOperator>());
        chain.push_back(std::make_shared<UnifiedManifoldIsRemovableOperator>());
        chain.push_back(std::make_shared<DegreeThreeIsRemovableOperator>());
        chain.push_back(std::make_shared<DegreeSixIsRemovableOperator>());
        return chain;
    }

    /// 论文 §3 的 anchor 预处理（每轮施加，见 pipelineTuner 的 preProcessPosition）。
    PreProcessOperatorPtr preProcessOperator() const override {
        // P3 conquest：锚点在 collector->init() 里检测并注入 AnchorVetoPredicate，
        // 不需要框架每轮重标（引擎自己从不删锚点）。
        if (aptmConquestP3On()) return nullptr;
        if (std::getenv("APTM_NO_ANCHOR")) return nullptr;
        return std::make_shared<AnchorPreprocessor>();
    }

    BoundaryConfigurator boundaryConfigurator() const override {
        // P3 conquest 的边界语义由引擎处理（isBorderVertex + null_patch），
        // 不叠加 APTM 的 side-channel（避免给 conquest blob 之外的产物加字节）。
        if (aptmConquestP3On()) return {};
        if (std::getenv("APTM_NO_BOUNDARY")) return {};
        return [](CompressPipelineConfig& cfg) { configureAptmPMSFBoundary(cfg); };
    }

    /// P1：tag 路径的消除算子必须是 `PCLTTMTagEliminateOperator`
    /// （vertex tag evolution 是 decoder 端解 fusedFaceSym 的必要配对，
    ///   与 PCLTTM Tag 协议覆盖路径同源）。默认路径返回 nullptr ⇒ 框架
    ///   `VertexRemovalEliminateOperator`（与改动前逐项相同）。
    EliminateOperatorPtr eliminate() const override {
        if (aptmConquestP3On()) {
            // 引擎自己跑前沿 BFS 并删点；框架侧必须空转（同一份已验证设计，
            // 见 impl/Conquest/elimination/ConquestNoopEliminateOperator.h）。
            return std::make_shared<ConquestNoopEliminateOperator>();
        }
        if (aptmTagPathOn()) {
            return std::make_shared<PCLTTMTagEliminateOperator>();
        }
        return nullptr;
    }

    /// P1：tag 路径的 collector 换成 APTM 自己的子类（继承 PCLTTM Tag）。
    CollectorFactory collectorFactory() const override {
        if (aptmConquestP3On()) {
            return []() -> std::shared_ptr<SymbolCollectOperator> {
                return std::make_shared<APTMConquestCollector>();
            };
        }
        if (!aptmTagPathOn()) return {};
        return []() -> std::shared_ptr<SymbolCollectOperator> {
            return std::make_shared<APTMTagSymbolCollectOperator>();
        };
    }

    /// tag residual 是整数位平面流 / conquest 需要量化网格：都不支持 F32 档。
    bool requiresQuantization() const override {
        return aptmTagPathOn() || aptmConquestP3On();
    }

    /// tag residual 的二阶 curvature predictor 用顶点属性 `kVPropRDR1` 存跨
    /// polyFace 的 r1 状态（与 PCLTTM/RD Tag 同一份机制）。attach 必须在任何
    /// collect() 之前 —— `beforeCompressLoop` 在 collector->init 之后、轮循环
    /// 之前执行，正好是框架为此类算法预留的时机。
    BeforeCompressLoopFn beforeCompressLoop() const override {
        if (aptmConquestP3On()) return {};  // conquest 不需要 RDR1 属性
        if (!aptmTagPathOn()) return {};
        return [](std::shared_ptr<MCGAL::Mesh>, PMSFHeader&,
                  std::shared_ptr<SymbolCollectOperator>, const CompressOptions&) {
            MCGAL::contextPool.attachVertexProperty<MCGAL::PointInt>(
                DEFAULT_MESH_ID, MCGAL::kVPropRDR1);
        };
    }


    /// P1 tag 路径的解压侧：产物是 CODEC_016 容器 + APTM 自己的 scheme_version
    /// （64/65），框架的按 layout 自动切换只会认出 PCLTTM Tag（sv=3），因此本
    /// 算法显式装配 APTM reader。父类 Frenet 反投影分支需要 `kVPropRDR1` 状态，
    /// 在 `afterBaseMeshHook` 里补挂（`runDecompress` 在 round 循环前的时机）。
    AfterBaseMeshFn afterBaseMeshHook() const override {
        if (aptmConquestP3On()) return {};  // conquest 解码不需要 RDR1
        if (!aptmTagPathOn()) return {};
        return [](const PMSFHeader&,
                  std::shared_ptr<DeserializeOperator> deserializer,
                  std::shared_ptr<MCGAL::Mesh> mesh) {
            MCGAL::contextPool.attachVertexProperty<MCGAL::PointInt>(
                DEFAULT_MESH_ID, MCGAL::kVPropRDR1);
            // P2-pre spike：把基网顶点数告诉噪声注入解压器（只扰动插入顶点）。
            if (auto* spike =
                    dynamic_cast<APTMSimLossyDeserializer*>(deserializer.get())) {
                if (mesh) {
                    spike->setBaseVertexCount(mesh->size_of_vertices());
                }
            }
        };
    }

    /// P1 tag 路径的解压器：显式 reader + PCLTTM Tag reconstructor（tag evolution
    /// 的 decoder mirror），不走框架的 layout→reader 自动切换（那会把 sv=64/65
    /// 当成 sv=3 的 Cartesian 流来解）。
    DecompressOptions makeDecompressOptions(Backend backend,
                                            const std::string& input,
                                            const std::string& output) const override {
        if (backend == Backend::Columnar && aptmConquestP3On()) {
            // 与 impl/Conquest/DecompressFactoryRegistration.cpp 的 tripe 完全同构：
            // reader 与 reconstructor 必须共享**同一个** reader 实例（层游标在
            // reader 上），deserializer 走 ColumnarDeserializeOperator 的 layout-15
            // 分支（装填 reader → 逐轮 reconstruct → decodeLayer）。
            auto reader = std::make_shared<ConquestSymbolReadOperator>();
            auto reconstructor =
                std::make_shared<ConquestReconstructOperator>(reader);
            // P3-P2/S1/S3：几何 hook 必须与编码端**同一份语义/参数**（Frenet+polar
            // 的 transform/inverseTransform 对称）。native 模式不注入（P3-P1 原路径）。
            const auto geomMode = aptm_conquest::resolveGeometryMode();
            if (geomMode != aptm_conquest::GeometryMode::Native) {
                reconstructor->setHooks(
                    aptm_conquest::makeAPTMConquestHooks(nullptr, geomMode));
            }
            auto deserializer = std::make_shared<ColumnarDeserializeOperator>(
                reader, reconstructor);
            return DecompressOptions::Builder()
                .setDeserializeOperator(deserializer)
                .setPath(input)
                .setOutputPath(output)
                .build();
        }
        if (backend == Backend::Paper && aptmConquestP3On()) {
            // M5-2（task-32 APTM 侧）：paper-v2 native artifact 里装的就是同一条
            // CQST v2 原生 blob（layout=15）+ canonical compact bin（base mesh）。
            // 但 src/ 的 paper 工厂给 "aptm" 注册的是通用 Optimized 三件套
            // （src/decompress/PaperDecompressFactoryRegistrationParquet.cpp），
            // 它读不了 layout-15 ⇒ M5-0 探针 decode SIGSEGV。
            //
            // 这里复用**已注册的 conquest paper 三件套**（PaperDeserializeOperator
            // + ConquestSymbolReadOperator + ConquestReconstructOperator(reader)），
            // 再装上与 canonical columnar APTM 解码**同一份** polar hooks
            // （levels=48，由 env 决定；解码端不需要 anchors/Arch-2 状态）。
            // 不改 src/、不改注册表、默认 paper APTM（env 未设）路径零影响。
            const auto triple =
                pmsf::makeDecompressOperatorsForAlgorithm("conquest", backend);
            if (!triple.deserializer) {
                throw std::runtime_error(
                    "[APTM][paper] conquest paper deserializer is not registered; "
                    "cannot decode layout-15 APTM paper artifact");
            }
            std::shared_ptr<ConquestReconstructOperator> recon =
                std::dynamic_pointer_cast<ConquestReconstructOperator>(
                    triple.reconstructor);
            if (!recon) {
                throw std::runtime_error(
                    "[APTM][paper] conquest paper triple does not expose a "
                    "ConquestReconstructOperator; refusing to decode with a "
                    "mismatched paper triple");
            }
            const auto geomMode = aptm_conquest::resolveGeometryMode();
            if (geomMode != aptm_conquest::GeometryMode::Native) {
                if (geomMode == aptm_conquest::GeometryMode::Polar &&
                    std::getenv("PMSF_APTM_CONQUEST_POLAR_LEVELS") == nullptr) {
                    // M5 已知限制（matrix notes）：paper dictionary 尚未自描述
                    // 精度参数（M6 加 aptm_polar_levels 字段）。缺 env 时按默认
                    // levels=8 解，会静默解错 l48 产物 —— 至少发一条 warning。
                    std::fprintf(
                        stderr,
                        "[APTM][paper] WARN polar geometry but "
                        "PMSF_APTM_CONQUEST_POLAR_LEVELS is unset; decoding with "
                        "default levels=%d (an artifact encoded with 48 levels "
                        "will be geometrically wrong)\n",
                        aptm_conquest::resolvePolarLevels());
                }
                recon->setHooks(
                    aptm_conquest::makeAPTMConquestHooks(nullptr, geomMode));
            }
            return DecompressOptions::Builder()
                .setDeserializeOperator(triple.deserializer)
                .setPath(input)
                .setOutputPath(output)
                .build();
        }
        if (backend == Backend::Columnar && aptmTagPathOn()) {
            auto reader = std::make_shared<APTMTagSymbolReadOperator>();
            auto reconstructor = std::make_shared<PCLTTMTagReconstructOperator>();
            std::shared_ptr<ColumnarDeserializeOperator> deserializer;
            const bool spikeOn =
                std::getenv("PMSF_APTM_SPIKE_NOISE_DELTA") != nullptr ||
                std::getenv("PMSF_APTM_SPIKE_NOISE_STATS") != nullptr;
            if (spikeOn) {
                deserializer = std::make_shared<APTMSimLossyDeserializer>(
                    reader, reconstructor);
            } else {
                deserializer = std::make_shared<ColumnarDeserializeOperator>(
                    reader, reconstructor);
            }
            return DecompressOptions::Builder()
                .setDeserializeOperator(deserializer)
                .setPath(input)
                .setOutputPath(output)
                .build();
        }
        return ISimplificationAlgorithm::makeDecompressOptions(backend, input,
                                                               output);
    }

    PipelineTuner pipelineTuner() const override {
        // P3 conquest：与 algorithm=conquest 完全一致 —— 不做任何管线覆盖，
        // 让框架标准 preInitPreparation（PMSF_DEFAULT_QUANT_BITS）落地量化网格。
        if (aptmConquestP3On()) return {};
        if (std::getenv("APTM_NO_TUNER")) return {};
        return [](CompressPipelineConfig& cfg, Backend /*backend*/) {
            if (aptmTagPathOn()) {
                // 复刻 PCLTTM Tag 协议覆盖路径的管线配置（impl/common/RetriOverride.h
                // case PCLTTM_Tag），但保留 APTM 自己的 anchor 前处理节奏：
                //   - seedStrategy       = select 后从 entry.edge 取 seed；
                //   - recomputeVertexDegreesOnce = true（tag BFS 依赖度数）；
                //   - trackContextLevel  = true；
                //   - updateAfterEliminate = false；
                //   - preProcessPosition = EveryRound ⇒ anchor 每轮重标不可删。
                cfg.seedStrategy = CompressPipelineConfig::SeedStrategy::SelectThenExtract;
                cfg.recomputeVertexDegreesOnce = true;
                cfg.trackContextLevel = true;
                cfg.updateAfterEliminate = false;
                cfg.preProcessPosition = CompressPipelineConfig::PreProcessPosition::EveryRound;
                cfg.preInitPreparation =
                    [](std::shared_ptr<MCGAL::Mesh> mesh, PMSFHeader&,
                       const CompressOptions& options) {
                        prepareTagQuantization(mesh, options);
                    };
                return;
            }
            // ---- 默认 APTM 路径（逐项保持原样）----
            cfg.seedStrategy = CompressPipelineConfig::SeedStrategy::SelectThenExtract;
            cfg.trackContextLevel = true;
            cfg.updateAfterEliminate = false;
            cfg.preProcessPosition = CompressPipelineConfig::PreProcessPosition::EveryRound;
        };
    }

    /// 默认不覆盖；显式 opt-in 时走 rd_tag。
    /// P1 tag 路径（PMSF_APTM_USE_APTM_TAG=1）必须走标准管线，不能被协议覆盖截走。
    std::string retriangulationProtocol() const override {
        if (aptmConquestP3On()) return {};
        if (aptmTagPathOn()) return {};
        if (envOn("PMSF_APTM_USE_RETRI_TAG")) return "rd_tag";
        return {};
    }

    RetriProtocolTuning retriangulationTuning() const override {
        RetriProtocolTuning t;
        t.defaultQuantBits = 12;
        t.quantBitsEnv = "PMSF_RDTAG_QUANT_BITS";
        t.altQuantBitsEnv = "PMSF_APTM_QUANT_BITS";
        return t;
    }

    /// P3 conquest 的默认轮数 = 16（P13.4 已验证运行点：fandisk/bunny 均
    /// rounds=16、base 17/89，见 docs/refactor/P13_4_D_MEASUREMENT_2026-09-23.md）。
    int defaultRounds() const override {
        if (aptmConquestP3On()) return 16;
        return 30;
    }
};

}  // namespace
}  // namespace pmsf

PMSF_REGISTER_SIMPLIFICATION_ALGORITHM(pmsf::APTMAlgorithm);
