/*
 * @file ISimplificationAlgorithm.h
 * @brief 框架对算法的唯一契约（设计文档 §2.1.1）。
 *
 * 目标形态：**接入一个算法 = 新建一个 cpp，继承本父类，重写算法特有的少数虚函数，
 * PMSF_REGISTER_ALGORITHM 宏注册，结束。** 压缩主循环、符号收集、序列化落盘、
 * 解压、CLI、配置解析全部由框架完成——不需要写任何 `XxxDecompress.cpp`。
 *
 * 论文 Tbl. tab:hook_definitions 的 H1–H8 落成本类上的虚函数，**除 `info()` 外
 * 全部有框架默认实现**。一个「零优化」的新算法只实现 `info()` 即可跑通完整
 * 压缩→存储→解压 round-trip（见 impl/Demo/DemoAlgorithm.cpp）。
 *
 * 解码侧：框架把 `info().name` / codec_id / predictor / quantizer 名写进 metadata，
 * 解压时按名从注册表再实例化组件、装配通用解码器。算法在 encode 侧实现的组件
 * 天然两侧可用（接口本身就是双向的），因此解压侧零算法代码。唯一需要算法显式
 * 提供的解码行为是 H5 逆移位 —— 与论文 §3.4「the only user-defined hook in the
 * decoding pipeline」一致。
 *
 * 反硬编码纪律：本文件与整个 src/ 不得出现任何具体算法名；分支只允许出现在
 * 注册表查表，不允许 `if (algo == ...)`。
 */
#ifndef PMSF_ALGORITHM_ISIMPLIFICATIONALGORITHM_H
#define PMSF_ALGORITHM_ISIMPLIFICATIONALGORITHM_H

#include "algorithm/RetriProtocolTuning.h"
#include "config/PMSFConfig.h"
#include "core/Capability.h"
#include "operator/IsRemovableOperator.h"
#include "segmentation/SegmentationOption.h"  // P18.2 E1: H7 Segmenter::options()
#include "options/DecompressOptions.h"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace MCGAL {
class Mesh;
class Halfedge;
class Vertex;
}  // namespace MCGAL

/// 以下框架类型都在**全局命名空间**（不是 pmsf::），前向声明必须放在这里，
/// 否则会在 pmsf 里造出一个同名但不同的类型。
class SymbolCollectOperator;    // src/compress/SymbolCollectOperator.h
class EliminateOperator;        // src/elimination/EliminateOperator.h
class CompressOptions;          // src/options/CompressOptions.h
class PreProcessOperator;       // src/preprocess/PreProcessOperator.h
class PostProcessOperator;      // src/postprocess/PostProcessOperator.h
class DecompressOptions;        // src/options/DecompressOptions.h
class DeserializeOperator;      // src/decompress/DeserializeOperator.h
class PMSFHeader;               // src/header/PMSFHeader.h

namespace pmsf {

class ErrorMetric;
class Predictor;
class EntropyCoder;
class Codebook;
class Quantizer;

/// 算法专属符号布局 codec（设计文档 §2.2，Phase 2 落地具体形态）。
class SymbolCodec;
/// 自描述侧信道的 schema 构建器（设计文档 §2.3 的 side_channel 表）。
class SideChannelSchemaBuilder;

/// @brief 被消除的原语类型（论文 §3.1.3）。
enum class PrimitiveKind {
    Vertex,     ///< 顶点删除
    HalfEdge,   ///< 半边折叠
    Edge,       ///< 边折叠
    Triangle,   ///< 三角形折叠
};

/// @brief 原语选择策略（论文 §3.1.4）。
enum class SelectionKind {
    ConnectivityDriven,  ///< 确定性 BFS，按遍历顺序消除
    GeometryDriven,      ///< 优先队列，按 H2 误差度量排序
};

/// @brief 算法元信息。`name` 是注册表键，也是写进 metadata 的解压侧再实例化依据。
struct AlgorithmInfo {
    std::string   name;        ///< 小写、无空格、稳定不变（注册表键）
    int           version = 1; ///< 存储格式语义版本，写入 metadata
    PrimitiveKind primitive = PrimitiveKind::Vertex;
    SelectionKind selection = SelectionKind::GeometryDriven;
};

// ---------------------------------------------------------------------------
// Hook 类型别名。全部复用框架既有算子/子系统基类，父类只是它们的装配面。
// ---------------------------------------------------------------------------

/// H2 的代价访问语义（`SelectOptions::withHalfedgeErrorAccessor` 的两种约定）。
///
/// 框架在 `HalfedgeSelectionQueue::computeError` 里拿到 target 后，会把它当作
/// 「顶点删除后落到哪里」用于几何残差；因此由谁填 target 是算法语义的一部分。
enum class ErrorAccessStyle {
    /// 框架用两端点中点填 target，再问 `cost(h)`。
    /// 对应 `HookBridge.h::makeHalfedgeErrorAccessor`（绝大多数连通驱动/tag 类算法）。
    Midpoint,
    /// 度量自己把最优放置写进 target，并把代价一起返回。
    /// 对应 `OptimalPlacementQuadricError`（论文里带 optimal placement 的几何驱动算法）。
    OptimalPlacement,
};

/// H1：可消除性算子链（按顺序链式判定）。
using IsRemovableChain = std::vector<std::shared_ptr<IsRemovableOperator>>;

/// 消除算子（论文 §3.1.5「Primitive Elimination」的装配面）。
/// 注意：`EliminateOperator` 与 `SymbolCollectOperator` 一样在全局命名空间。
using EliminateOperatorPtr = std::shared_ptr<EliminateOperator>;

/// 前处理算子（在每轮或一次性施加的算法预处理，例如锚点判定）。
using PreProcessOperatorPtr = std::shared_ptr<PreProcessOperator>;

/// 后处理算子（每轮结束后施加，例如预测残差调整）。
using PostProcessOperatorPtr = std::shared_ptr<PostProcessOperator>;

/// H1/H2 之后的算子链：选择器与种子策略。
enum class SelectorKind {
    PriorityError,   ///< 优先队列（几何驱动 / 框架默认）
    Greedy,          ///< 贪心（连通驱动的一种实现）
};

/// H2：误差度量工厂。框架在网格加载且 `declareProperties()` 的属性注册完成后
/// 调用一次，返回本次运行的度量实例；返回空表示使用框架默认（无度量）。
using ErrorMetricFactory =
    std::function<std::shared_ptr<ErrorMetric>(std::shared_ptr<MCGAL::Mesh>)>;

/// H3：环三角化。返回空表示使用框架确定性环三角化。
using TriangulatorPtr = std::shared_ptr<void>;

/// H4：顶点移位。返回空表示不移位。
using VertexShiftPtr = std::shared_ptr<void>;

/// H5：逆移位 —— 解码侧唯一 hook。返回空表示走显式残差。
using ReverseShiftPtr = std::shared_ptr<void>;

/// H6：边翻转规则。返回空表示不翻转（现状）。
///
/// P18.2（E1）：由 `shared_ptr<void>` 落成真实基类。**形状取自现有消费者**
/// —— `CompressPipeline` 的 edge-flip 后处理按轮给定位数/坐标符号
/// （`edgeFlipRoundBits` / `edgeFlipCoordSymbols`），故本接口按「规则 + 轮数」
/// 表达，而不是空壳。
class EdgeFlipRule {
  public:
    virtual ~EdgeFlipRule() = default;
    /// 参与翻转的轮数；0 = 不翻转。
    virtual int rounds() const { return 0; }
    /// 某轮是否允许翻转该半边。
    virtual bool shouldFlip(MCGAL::Halfedge* /*h*/, int /*round*/) const { return false; }
};
using EdgeFlipRulePtr = std::shared_ptr<EdgeFlipRule>;

/// H7：分区器。返回空表示不分区（现状）。
///
/// P18.2（E1）：由 `shared_ptr<void>` 落成真实基类。**形状取自现有消费者**
/// —— `SegmentationSymbolCollectOperator` 产出的是
/// `[segment][round]` 二维索引的符号队列（`segment*Queues`），
/// 即**分区是被编码的一维**、单分区可独立解码；其驱动参数正是
/// `SegmentationOption`（方案/分区数/种子/边界压缩）。
/// ⇒ 本接口的职责 = 「决定分区配置」，装配点据此换上传媒三件套。
class Segmenter {
  public:
    virtual ~Segmenter() = default;
    /// 分区配置，直接下发给 `Segmentation*` 算子组。
    virtual SegmentationOption options() const { return {}; }
};
using SegmenterPtr = std::shared_ptr<Segmenter>;

/// H8：边界规则。返回空表示走标准可消除性链（现状）。
///
/// P18.2（E1）：由 `shared_ptr<void>` 落成真实基类。**形状取自现有消费者**
/// —— `impl/Segmentation/` 的边界相关开关（`protectBoundaryOneRing` /
/// `allowBoundaryCompression`，见 segmented 复现脚本与
/// `SegmentationOption::setCompressBoundary`）。
class BoundaryRule {
  public:
    virtual ~BoundaryRule() = default;
    /// 是否保护边界一环不被消除。
    virtual bool protectBoundaryOneRing() const { return false; }
    /// 是否允许压缩边界。
    virtual bool allowBoundaryCompression() const { return false; }
};
using BoundaryRulePtr = std::shared_ptr<BoundaryRule>;

class CompressPipelineConfig;

/// 符号收集器的工厂（算法自有逻辑布局的产出方式）。
/// 过渡形态：设计文档 §2.1.3 的 L1「自定义符号布局」最终由 `symbolCodec()` 表达，
/// Phase 2 落地后本 hook 由 symbolCodec 导出；Phase 1 先把它显式化，
/// 消灭 `PMSF::xxxColumnarCompress` 里的 per-algorithm collector 分支。
using CollectorFactory = std::function<std::shared_ptr<SymbolCollectOperator>()>;

/// 边界处理的装配回调（H8 的框架装配面）。
using BoundaryConfigurator = std::function<void(CompressPipelineConfig&)>;

/// 管道微调回调：把「契约尚未建模、但框架管线已支持」的算法私有开关集中在这里，
/// 使它们不再散落在核心层的 per-algorithm 分支里。每加一项都应在设计文档中
/// 说明为什么它不是一个新的 hook。
/// @param cfg     待微调的管线配置
/// @param backend 本次压缩用的物理后端
///
/// 为什么要给 backend：有些算法只有某一后端才需要换序列化器（自有落盘格式只
/// 在二进制后端生效，列式后端仍走框架的列式序列化器）。不给后端，算法就只能
/// 无条件覆盖 serializerFactory，把另一个后端一起改坏。
using PipelineTuner = std::function<void(CompressPipelineConfig&, Backend)>;

/// CompressOptions 侧的对应物：框架的 `CompressOptions` 已支持、但契约尚未建模
/// 的开关（三角化开关、连通性预测、loading scheme …）。与 PipelineTuner 同理由：
/// 把这些开关收进算法自己的 cpp，而不是让核心层出现 per-algorithm 分支。
using CompressOptionsTuner = std::function<void(CompressOptions&)>;

/// 压缩开跑前的回调（论文里「每轮之外、B 循环之前」的算法行为）。
using BeforeCompressLoopFn = std::function<void(std::shared_ptr<MCGAL::Mesh>,
                                                PMSFHeader&,
                                                std::shared_ptr<SymbolCollectOperator>,
                                                const CompressOptions&)>;

/// 解压侧：读完基础网格、进入逐轮循环之前的下传回调（把 header 里的预测开关
/// 交给算法的 SymbolReadOperator）。对应 `runDecompress` 的 afterBaseMeshHook。
using AfterBaseMeshFn = std::function<void(const PMSFHeader&,
                                           std::shared_ptr<DeserializeOperator>,
                                           std::shared_ptr<MCGAL::Mesh>)>;

/// 解压侧：全部轮次重建完成、写盘之前的收尾（例如退化三角形修复）。
/// 之所以放在写盘之前，是因为修复必须体现在输出网格里。
using PostDecompressFn = std::function<void(std::shared_ptr<MCGAL::Mesh>)>;

using QuantizerPtr    = std::shared_ptr<Quantizer>;
using PredictorPtr    = std::shared_ptr<Predictor>;
using EntropyCoderPtr = std::shared_ptr<EntropyCoder>;
using SymbolCodecPtr  = std::shared_ptr<SymbolCodec>;

/// MCGAL 网格属性声明：算法私有属性由算法提供初始化回调，框架负责在正确的
/// 时机（网格加载后、算子构造前）调用，取代 entry 里的 `registerProperties` lambda。
using PropertyInitFn = std::function<void(std::shared_ptr<MCGAL::Mesh>)>;

/// 压缩开关的框架默认值。算法的 12 个 bool 不再手填：默认 = 框架标准行为，
/// 算法只在论文确实不同的时候重写。
///
/// `adjustment` 默认为 false —— 它对应 H4 顶点移位，而 H4 的框架默认是「不移位」
/// （vertexShift() 返回空）。要打开它的算法必须同时重写 vertexShift()（编码）
/// 与 reverseVertexShift()（解码），否则解码端拿不到 V_r / C_r^v 流。
struct CompressionProfile {
    bool prediction   = true;
    bool quantization = true;
    bool adjustment   = false;
    bool compression  = true;
    bool segmentation = false;
};

/// @brief 算法父类。除 `info()` 外全部虚函数都有默认实现。
class ISimplificationAlgorithm {
  public:
    virtual ~ISimplificationAlgorithm() = default;

    // ---- 元信息（唯一纯虚）------------------------------------------------
    virtual AlgorithmInfo info() const = 0;

    // ---- 编码侧 hooks（论文 H1–H4, H6–H8）--------------------------------
    /// H1 默认可消除性算子链 = Basic + UnifiedManifold（框架标准行为）。
    virtual IsRemovableChain removability() const;
    /// H2 默认无误差度量（连通驱动不需要）。
    virtual ErrorMetricFactory errorMetric() const { return {}; }
    /// H2 的 target 语义。默认 Midpoint（框架填中点 + `cost(h)`）。
    virtual ErrorAccessStyle errorAccessStyle() const { return ErrorAccessStyle::Midpoint; }
    /// 是否在每次消除后回调 H2 的度量做状态刷新（有状态的度量需要）。
    /// 默认 false：不安装回调，与历史实现一致。
    virtual bool needsAfterEliminateRefresh() const { return false; }
    /// 消除算子。默认 nullptr ⇒ 框架 `VertexRemovalEliminateOperator`
    /// （论文 §3.1.5：所有经典算子都可分解为顶点删除 + 可选调整）。
    virtual EliminateOperatorPtr eliminate() const { return nullptr; }
    /// 选择器。默认 PriorityError（几何驱动算法配合 H2 使用）。
    virtual SelectorKind selector() const { return SelectorKind::PriorityError; }
    /// 后处理算子。默认 nullptr ⇒ 无后处理。
    virtual PostProcessOperatorPtr postProcessOperator() const { return nullptr; }

    /// 前处理算子。默认 nullptr ⇒ 无前处理。
    virtual PreProcessOperatorPtr preProcessOperator() const { return nullptr; }
    /// H3 默认 nullptr ⇒ 框架确定性环三角化。
    virtual TriangulatorPtr triangulation() const { return nullptr; }
    /// H4 默认 nullptr ⇒ 不移位。
    virtual VertexShiftPtr vertexShift() const { return nullptr; }
    /// H6 默认 nullptr ⇒ 不翻转。
    virtual EdgeFlipRulePtr edgeFlipRule() const { return nullptr; }
    /// H7 默认 nullptr ⇒ 不分区。
    virtual SegmenterPtr segmenter() const { return nullptr; }
    /// H8 默认 nullptr ⇒ 边界走标准可消除性链。
    virtual BoundaryRulePtr boundaryRule() const { return nullptr; }

    // ---- 解码侧 hooks ------------------------------------------------------
    /// H5 逆移位 —— 论文 §3.4 说的「解码管线中唯一由用户定义的 hook」。
    virtual ReverseShiftPtr reverseVertexShift() const { return nullptr; }
    /// 解压侧头信息下传。默认空。
    virtual AfterBaseMeshFn afterBaseMeshHook() const { return {}; }
    /// 解压后收尾（写盘前）。默认空。
    virtual PostDecompressFn postDecompressHook() const { return {}; }

    // ---- 可插拔子系统 -----------------------------------------------------
    virtual QuantizerPtr    quantizer() const { return nullptr; }
    virtual PredictorPtr    predictor() const { return nullptr; }
    virtual EntropyCoderPtr entropyCoder() const { return nullptr; }
    /// 算法专属符号布局。默认 nullptr ⇒ canonical codec（设计文档 §2.2）。
    virtual SymbolCodecPtr  symbolCodec() const { return nullptr; }

    // ---- 声明式扩展 -------------------------------------------------------
    /// 自描述侧信道列（残差/码本索引/anchor_mask …）。
    virtual void declareSideChannels(SideChannelSchemaBuilder&) const {}
    /// MCGAL 网格属性初始化。默认空。
    virtual PropertyInitFn declareProperties() const { return {}; }
    /// 算法私有配置：来自 config.json 的 `algorithm_options` 段。
    virtual void configure(const nlohmann::json& /*options*/) {}

    /// 压缩开关的框架默认（预测/量化/调整/压缩开，分区关）。
    virtual CompressionProfile compressionProfile() const { return {}; }

    /// 符号收集器工厂。默认空 ⇒ 框架 OptimizedSymbolCollectOperator。
    virtual CollectorFactory collectorFactory() const { return {}; }
    /// 未显式配置 `rounds` 时使用的默认轮数。默认 30。
    ///
    /// 由算法自己声明，而**不是**在 `src/config/` 里维护一张「算法名 -> 轮数」的表 ——
    /// 那张表的键必须与注册名逐字一致，一旦错位（例如大小写/下划线写法不同）
    /// 就会**静默**回落到兜底值。本仓库实测踩过：`pclttm_valence` 条目因为
    /// 注册名实际是 `pclttmvalence` 而从未命中。让算法自己说，这个错位就没有了。
    virtual int defaultRounds() const { return 30; }

    /// 选择 / 可消除性策略（见 core/SelectionPolicy.h）。默认空 ⇒ 框架默认行为。
    /// 这是「算法特有的候选打分 / 误差项 / 新增半边放行」的唯一入口 ——
    /// 框架不再为此读任何算法专属环境变量。
    virtual SelectionPolicy selectionPolicy() const { return {}; }

    /// 本算法是否**要求量化**（即不支持 F32 档）。
    ///
    /// P18.1a 防呆：config 写 `"quantization": false` 时若算法声明 true，
    /// 框架在**派发期**（config 校验点）即报出可读错误。否则要等跑进 `*Tag`
    /// 采集器的 `exportRC` 才深抛 —— 实测白跑一遍才失败（MMSE 约 1.0 s、
    /// QEM 约 1.1 s）。
    ///
    /// 默认 false = 支持未量化。**由算法自己声明**，框架不读任何算法名
    /// （`src/` 禁算法名）。声明 true 的是残差按整数位平面设计、没有对应
    /// 未量化码流布局的那几个协议。
    virtual bool requiresQuantization() const { return false; }

    /// H8 装配面。默认空 ⇒ 框架标准边界处理。
    virtual BoundaryConfigurator boundaryConfigurator() const { return {}; }
    /// 管线微调。默认空 ⇒ 完全使用框架标准管线。
    virtual PipelineTuner pipelineTuner() const { return {}; }
    /// CompressOptions 微调。默认空 ⇒ 完全使用 hook 推导出的选项。
    virtual CompressOptionsTuner compressOptionsTuner() const { return {}; }
    /// 压缩开跑前回调。默认空。
    virtual BeforeCompressLoopFn beforeCompressLoop() const { return {}; }

    /// 决定性三角化协议 id（见 impl/common/RetriOverride.h）。
    /// 默认空 ⇒ 不启用协议覆盖。取代 `detectRetriOverride(AlgoPreset)` 的枚举分支。
    virtual std::string retriangulationProtocol() const { return {}; }
    /// 协议覆盖的算法侧调参（协议实现只认参数，不认算法名）。
    virtual RetriProtocolTuning retriangulationTuning() const { return {}; }

    // ---- 能力自动推导 -----------------------------------------------------
    /// @brief 由「哪些 hook 被重写」推导出的能力位集（复用 src/core/Capability.h
    ///        的词汇表）。取代 CompressOptions::Builder 上那批手填布尔开关。
    CapabilitySet capabilities() const;

    // ---- 执行入口（框架提供默认实现）--------------------------------------
    /// 默认实现：查询各 hook → 装配框架既有 Operator → 跑统一 CompressPipeline。
    /// 算法只有在使用了契约尚未覆盖的子系统时才重写本方法，并在注释里写明缺哪个 hook。
    virtual void compress(const PMSFConfig& cfg);
    /// 默认实现：按 metadata 记名再实例化组件 → 通用解码器。
    virtual void decompress(const PMSFConfig& cfg);

    /// @brief 组装本算法的解压选项：解压器（含算法注册的 reader/reconstructor）
    ///        加算法自己的 afterBaseMesh 钩子。
    ///
    /// 随机访问查询 (runQuery) 复用**同一条**装配路径，再往上挂 (roi, lod) 谓词。
    /// 否则 Query 就只对「不需要任何算法级钩子」的算法有效 —— 那等于 Query 不是
    /// 框架能力，只是一条旁路（实测：旁路装配的解压器只能解出 base mesh）。
    virtual DecompressOptions makeDecompressOptions(Backend backend,
                                                    const std::string& input,
                                                    const std::string& output) const;

  protected:
    /// 注册表键（= info().name）。框架内部使用。
    const std::string& registryKey() const;
};

}  // namespace pmsf

#endif  // PMSF_ALGORITHM_ISIMPLIFICATIONALGORITHM_H
