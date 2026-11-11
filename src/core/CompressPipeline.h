/*
 * @file CompressPipeline.h
 * @brief PMSF 通用压缩管线配置 —— Phase 1 重构核心。
 *
 * 历史上 src/PMSF.cpp 里 ~20 个 *Compress / *ColumnarCompress 方法是同一
 * 段压缩主循环的 copy-paste（build header → load mesh → init select/eliminate
 * → seed → R 轮 BFS collect → serialize），唯一差异是：
 *   - SymbolCollectOperator 的具体子类
 *   - SerializeOperator 的具体子类（Binary vs Columnar vs PomarV2 vs ...）
 *   - 每轮前/每轮后 / 主循环之前 / 序列化之前的算法特有钩子
 *   - seed 落位方式（selectSeed vs "select→entry.edge"）
 *   - 是否需要 select->update(entry)（QEM 的 Q 矩阵刷新）
 *   - 是否开启 MIS 独立集约束（PCATM 专属）
 *
 * 本文件把这些差异项抽成 CompressPipelineConfig；pmsf::runCompress
 * 是真正的通用主循环。PMSF 里的每个 xxxCompress 方法退化为"构造 cfg +
 * 调用 runCompress"的 10-15 行转发。
 *
 * 注意：
 *   1. 保持所有公开语义与重构前完全一致（包括 debug 分支、seed 注册 etc）。
 *   2. POMAR V2 / VQMMC / segment / hierarchy 因为结构特殊（collector
 *      computeLODPartition / codebook / 嵌套 per-segment 循环），不走
 *      runCompress，保留各自专用入口。
 */
#ifndef PMSF_CORE_COMPRESS_PIPELINE_H
#define PMSF_CORE_COMPRESS_PIPELINE_H

#include <functional>
#include <memory>

#include "compress/SerializeOperator.h"
#include "compress/SymbolCollectOperator.h"
#include "core/Backend.h"
#include "core/Capability.h"
#include "header/PMSFHeader.h"
#include "options/CompressOptions.h"
#include "options/DecompressOptions.h"

// 注意：`SegmentOperator` 是**全局命名空间**的类（src/segmentation/SegmentOperator.h），
// 不是 pmsf 的。前向声明必须放在全局作用域 —— 放进 namespace pmsf 会静默
// 声明出一个**不同的类型**，于是下面 segmentOperatorFactory 的签名跟实现
// （BasicSegmentOperator 等）对不上。
class SegmentOperator;

namespace MCGAL {
class Mesh;
class Halfedge;
}  // namespace MCGAL

namespace pmsf {

// Forward declarations for hook types — Phase 10/13+ wiring.
// Keeping them as fwds lets existing TUs that include this header stay
// slim; an algorithm that actually passes one of these to the Builder
// just includes the concrete hook header in its own main file.
class ErrorMetric;
class Predictor;
class EntropyCoder;
class Codebook;
class FeatureDetector;

/// @brief 压缩管线的算法特化点。每项都是可选的（默认 no-op）。
struct CompressPipelineConfig {
    // ---- 必填：决定"怎么收集符号 + 怎么落盘" ----
    using CollectorFactory = std::function<std::shared_ptr<SymbolCollectOperator>()>;
    using SerializerFactory = std::function<std::shared_ptr<SerializeOperator>(
        std::shared_ptr<SymbolCollectOperator>, const CompressOptions&)>;
    CollectorFactory collectorFactory;
    SerializerFactory serializerFactory;

    /// @brief P18.2 阶段 A：**分区阶段**（框架此前完全没有的那一环）。
    ///
    /// 非空 ⇒ 在 `collector->init` 之前跑一次 `segment(mesh)`：

    /// 该调用会给每个 facet 写真实 `groupId`（默认是 -1，从来没有被赋过值），
    /// 并用 `exportSeeds()` 返回真 seeds（长度 = 分区数）替代写死的
    /// 单元素 `{seed}`。
    ///
    /// 空 ⇒ **默认路径**：`collector->init(mesh, {seed}, header)`，
    /// 与加入本字段之前逐字节一致。
    ///
    /// 注意（P18.2 §5b）：这只是**阶段 A**。多分区推进还需要主循环改成
    /// 「外层 round、内层 group」并调用 `collectActive(...)`，以及解码端对称
    /// 实现；见 docs/refactor/P18.2_PARTITION_STAGE_ATTRIBUTION.md §6。
    std::function<std::shared_ptr<SegmentOperator>()> segmentOperatorFactory;

    /// @brief P18.2 路线 (a) 两遍改造：**第一遍（勘察遍）**。
    ///
    /// 非空 ⇒ step 10 **不跑** `segmentOperatorFactory()` —— 因为那一遍要分区的
    /// 对象**不是本遍手上的全网格**（附 8 实测：step 10 见 34,834 顶点），
    /// 而是轮循环跑完后的**基网**（实测 7,081 顶点）。改为在轮循环结束后
    /// （彼处 `mesh` 就是基网）对基网跑一次 `segment()`，沿坍缩记录回填，
    /// 把 **顶点序号 → 分区** 写进本表（长度 = 全网格顶点数；-1 = 无归属）。
    ///
    /// 索引是**顶点序号**而非 poolId：跨网格 poolId 不对应（附 15 实测）。
    /// 空 ⇒ 默认路径，行为逐字节不变。
    std::shared_ptr<std::vector<int>> segmentGroupTableOut;

    /// @brief P18.2 路线 (a) 两遍改造：**第二遍（分区遍）**。
    ///
    /// 非空 ⇒ step 10 **不跑** `segmentOperatorFactory()`，改为按本表把全网格
    /// 每个 facet 的 `groupId` 写出来，并由分组导出 seeds（分组 → seed 半边）。
    /// 与 `segmentGroupTableOut` 互斥（一个写、一个读）。
    std::shared_ptr<const std::vector<int>> segmentGroupTableIn;

    /// @brief B.1 Sprint 2 探针：runCompress 可选挂一个**并行 collector**，在每轮
    ///        主 collector->collect(seed, gates) 之后紧接着也调用
    ///        probe->collect(seed, gates)。不参与码流写入（serializer 不看它）。
    ///        用途：验证 gate-level valence 采集的 raw symbol 数/V，
    ///        不干扰主路径。默认 nullptr（未启用）。
    CollectorFactory probeCollectorFactory;

    // ---- seed 落位策略 ----
    // SelectSeed         : select->selectSeed(seed) —— 与 selectSeedFn 配合
    //                      （RD 传 seedSelector lambda；其余算法传 nullptr 即可）
    // SelectThenExtract  : select->select(entry); seed = entry.edge
    //                      （PCATM / PCLTTM / APTM / MMSE 的历史行为）
    enum class SeedStrategy { SelectSeed, SelectThenExtract };
    SeedStrategy seedStrategy = SeedStrategy::SelectSeed;
    /// @brief 可选，传给 select->selectSeed 的自定义 selector（RD 用最长边）
    std::function<MCGAL::Halfedge*(std::shared_ptr<MCGAL::Mesh>)> customSeedSelector;

    // ---- 钩子（均允许 nullptr） ----
    /// @brief 主循环开始之前、select/eliminate/collector init 完成之后 调用。
    ///        典型用途：PPMC 的 mesh->quantizeVertexPositions()（在 init 前）、
    ///        PCATM 的 mesh->recomputeAllVertexDegrees()、header 标志下传到
    ///        SymbolCollector 的特殊配置。
    ///
    /// 注意：若需要在 select->init 之前做事（例如 PPMC 的 quantize 必须最先
    /// 发生，否则 select 里读到的顶点坐标就不对），请用 preInitPreparation。
    std::function<void(std::shared_ptr<MCGAL::Mesh>, PMSFHeader&,
                       std::shared_ptr<SymbolCollectOperator>,
                       const CompressOptions&)>
        beforeCompressLoop;

    /// @brief 在 mesh 构造之后、任何 Operator::init 之前调用。
    ///        用于 PPMC 的 quantizeVertexPositions —— 必须在顶点坐标
    ///        被 SelectOperator 读到之前替换为量化后的整型位置。
    std::function<void(std::shared_ptr<MCGAL::Mesh>, PMSFHeader&,
                       const CompressOptions&)>
        preInitPreparation;

    /// @brief 每轮开始时调用，i 为当前 round index（0..R-1）。
    ///        典型用途：contextPool.setMetadata("level", i) 的 PCATM/PCLTTM/RD/APTM/MMSE
    std::function<void(int /*roundIdx*/, std::shared_ptr<MCGAL::Mesh>)> beforeRound;

    /// @brief 每轮 eliminate 阶段结束、collector->collect 之前调用。
    ///        典型用途：MMSE 的顶点拉普拉斯 PostProcess（替代 options.getPostProcessOperator
    ///        的历史位置）。runCompress 的默认行为是若 cfg.afterRound==nullptr 则
    ///        调用 options.getPostProcessOperator()->process(mesh)（如果存在），
    ///        保证 PCATM/MMSE/APTM 的旧语义一字不差。
    std::function<void(int /*roundIdx*/, std::shared_ptr<MCGAL::Mesh>)> afterRound;

    /// @brief 在 serializer->init / serializeBaseMesh / serialize 之前调用。
    ///        典型用途：POMAR V2 的 collector->computeLODPartition()，
    ///        VQMMC 的 codebook sidecar 构造。
    std::function<void(std::shared_ptr<SymbolCollectOperator>,
                       std::shared_ptr<SerializeOperator>,
                       std::shared_ptr<MCGAL::Mesh>, MCGAL::Halfedge* /*seed*/)>
        beforeSerialize;

    // ---- 行为开关（默认值对齐 default/qem 等"最朴素"路径） ----

    /// @brief 每轮是否重新随机播种（参考实现每个 decimation conquest 都重新
    ///        rand() 选种子；PPMC 打开，其余算法沿用上一轮种子）。只影响编码端
    ///        消除顺序（码流自带操作序列），解码端无感。
    bool reseedEachRound = false;
    /// @brief 每轮选择前强制刷新所有 live 顶点的 degree 缓存。
    ///
    /// MCGAL 的 split_facet / erase_center_vertex 通过 inc/dec 维护邻居 degree
    /// 缓存，但**不会先把 dirty 的缓存算出来**：若某顶点上一轮结束后 degree
    /// 是 dirty（未计算），增量更新会作用在过期值上并把 dirty 清掉 → degree
    /// 永久错误。PPMC 的 isRemovable 依赖 degree，错误值会系统性漏删顶点
    /// （实测 horse19k 15 轮 base 419V → 133V，artifact −1.5 kB）。
    /// 默认关闭以保持其它算法字节不变。
    bool refreshVertexDegreesEachRound = false;

    /// @brief 每次 eliminate 成功后是否调用 select->update(entry)。
    ///        true 表示 PriorityError 类 selector 需要刷新邻边 cost / QEM Q-matrix hook。
    ///        QEM + PPMCColumnar 需要 true；其余算法保持 false，与原 PMSF::xxxCompress 一致。
    bool updateAfterEliminate = false;

    /// @brief 是否使用 IndependentSetSelector 约束每轮只删独立顶点集。
    ///        PCATM / PCATMColumnar 专属。
    bool enableIndependentSet = false;

    /// @brief 是否在 0 elimination 轮（select 不出任何可消除元素）时提前中断主循环
    ///        并把 header.round 缩到实际跑过的轮数。默认 false（历史行为：跑满 R 轮，
    ///        哪怕后面几轮全空）。PPMC 风格的 collector 在空轮上 `i_lenGeom > 0`
    ///        assertion 会触发，因此这种 collector 必须把此 flag 设 true（与
    ///        `PMSF::ppmcColumnarCompress` 的 `if (count == 0) break` 同义）。
    bool stopOnEmptyRound = false;

    /// @brief options.getPreProcessOperator() 的调用位置。
    ///
    ///   - None        : 不消费 preProcessOperator。default / qem / pclttm / rd 等使用。
    ///   - Once        : 在 eliminate->init 之前调用一次（PCATM 的历史行为）。
    ///   - EveryRound  : 每轮主循环开始时调用（APTM 的历史行为；MMSE 历史注释掉了 pre，
    ///                   若要等同旧行为保持 None）。
    enum class PreProcessPosition { None, Once, EveryRound };
    PreProcessPosition preProcessPosition = PreProcessPosition::None;

    /// @brief options.getPostProcessOperator() 的调用位置。
    ///
    ///   - None        : 不消费。
    ///   - EveryRound  : 每轮 select 阶段结束、collector->collect 之前调用（PCATM / MMSE）。
    enum class PostProcessPosition { None, EveryRound };
    PostProcessPosition postProcessPosition = PostProcessPosition::None;

    /// @brief 解释 header 时是否透传 enablePrediction（Compress 默认已经透传量化
    ///        /压缩/分区/调整四个 flag；部分算法 header 还要把 prediction 也写入）。
    ///        PPMC / compress(default) / rd / pclttmValence / segment* 需要 true。
    bool headerCarryPrediction = false;

    /// @brief 是否在 serialize 前调用 serializer->serializeHeader(header)。
    ///        Binary 路径都需要；Parquet V2 路径不需要（header 走 setHeader + 元数据表）。
    ///        默认 true。Columnar pipeline 工厂构造时把它置为 false。
    bool serializerNeedsExplicitHeader = true;

    /// @brief 是否以 setHeader(shared_ptr<PMSFHeader>) 的方式把 header 交给 serializer。
    ///        Columnar pipeline 需要（走 setHeader），Binary pipeline 不需要（已在
    ///        serializeHeader 里落盘）。默认 false。
    bool serializerWantsSharedHeader = false;

    /// @brief 主循环之前是否先 mesh->recomputeAllVertexDegrees()。
    ///        PCATM / PCLTTM / RD / APTM / MMSE 热路径的 DegreeN operator 依赖此。
    ///        默认 false（QEM / default 等不需要）。
    bool recomputeVertexDegreesOnce = false;

    /// @brief 主循环之前设置 contextPool.setMetadata("level", 0)。beforeRound 会每轮
    ///        更新为当前 i；这里的开关仅是"是否启用此 meta"。
    ///        开启：PCATM / PCLTTM / RD / APTM / MMSE。默认 false。
    bool trackContextLevel = false;

    // ---- Phase 10/13+ 算法 hook 槽（可选；runCompress 不默认消费） ----
    // 算法主函数调用 `.withErrorMetric(...)` 等 Builder 方法写入这些字段，
    // 然后通过 accessor（如 cfg.getErrorMetric()）在 registerProperties /
    // withHalfedgeErrorAccessor 等现有回调里取出来用。
    std::shared_ptr<ErrorMetric>     errorMetric;
    std::shared_ptr<Predictor>       predictor;
    std::shared_ptr<EntropyCoder>    entropyCoder;
    std::shared_ptr<Codebook>        codebook;
    std::shared_ptr<FeatureDetector> featureDetector;

    // ---- Phase A.5: CapabilitySet 统一视图 ----
    // 新算法可以写 `cfg.capabilities.add(Capability::UpdateAfterEliminate)`
    // 替代设置 12 个分立 bool。为不破坏现有代码路径，runCompress 仍读 bool
    // 字段；`effectiveCapabilities()` 把 bool 字段映射到 CapabilitySet，
    // 便于 impl/ 查询"这个 pipeline 当前需要哪些 capability"而不用一条一条
    // 读 bool。下一阶段（Phase B 所有算法迁完之后）可以反转主从，runCompress
    // 直接读 capabilities。
    CapabilitySet capabilities;

    /// 合成 CapabilitySet：优先用 `capabilities` 里显式 set 的位，再把 12 个
    /// 历史 bool 字段 OR 进去。算法只要读这个就能看到"完整需求表"。
    CapabilitySet effectiveCapabilities() const {
        CapabilitySet out = capabilities;
        if (updateAfterEliminate)        out.add(Capability::UpdateAfterEliminate);
        if (enableIndependentSet)        out.add(Capability::IndependentSet);
        if (recomputeVertexDegreesOnce)  out.add(Capability::RecomputeDegreesOnce);
        if (trackContextLevel)           out.add(Capability::ContextLevelTracking);
        if (headerCarryPrediction)       out.add(Capability::HeaderCarriesPrediction);
        if (serializerWantsSharedHeader) out.add(Capability::SerializerSharedHeader);
        if (serializerNeedsExplicitHeader) out.add(Capability::SerializerExplicitHeader);
        return out;
    }

    // accessor 以便 impl/ 调用点语法一致
    std::shared_ptr<ErrorMetric>     getErrorMetric()     const { return errorMetric; }
    std::shared_ptr<Predictor>       getPredictor()       const { return predictor; }
    std::shared_ptr<EntropyCoder>    getEntropyCoder()    const { return entropyCoder; }
    std::shared_ptr<Codebook>        getCodebook()        const { return codebook; }
    std::shared_ptr<FeatureDetector> getFeatureDetector() const { return featureDetector; }

    // ---- 预置工厂（方便 PMSF.cpp 一行复用） ----
    /// @brief 构造一个"OptimizedSymbolCollect + BasicSerialize"的默认 Binary 路径 cfg。
    ///        不填 before/afterRound 等钩子 —— 调用者按需追加。
    static CompressPipelineConfig defaultBinary();
    /// @brief 构造一个"OptimizedSymbolCollect + ColumnarSerialize"的默认 Columnar 路径 cfg。
    static CompressPipelineConfig defaultColumnar();
    /// @brief PaperStream v1 pilot: single-file `_paper.parquet`.
    static CompressPipelineConfig defaultPaper(const std::string& algorithmName = "default");
    /// @brief 构造一个"OptimizedSymbolCollect + UnifiedMultiTableSerialize"的默认
    ///        Unified 路径 cfg。V3 writer 继承 V2，外加 side-channel/blob 两张表。
    static CompressPipelineConfig defaultUnified();
};

// Phase 5：Builder 便捷 API —— 把 PreProcessOperator / RoundPreProcessor
// 作为一等公民允许 impl/ 层 main 用流畅接口配置 pipeline。
// 注意：下面两个 include 需要 PMSF 前向声明，已在上面的 fwd decl 里覆盖。
}  // namespace pmsf

#include "preprocess/PreProcessOperator.h"
#include "preprocess/ContextLevelTrackingPreprocessor.h"  // for RoundPreProcessor base

#include <vector>

namespace pmsf {

/// @brief Phase 5 Pipeline Builder —— 以流畅接口封装 CompressPipelineConfig
///        （见 CompressPipeline::Builder）。
class CompressPipeline {
  public:
    /// @brief 以流畅接口装配一次压缩所需的 cfg + 算子。
    ///
    /// 用法（impl/ 层 main）：
    ///   pmsf::CompressPipeline::Builder()
    ///       .withBackend(backend)
    ///       .withCollectorFactory(...)
    ///       .build();
    class Builder {
      public:
        Builder();

        // ---- 工厂 / 后端 ----
        Builder& withBackend(Backend b);
        Builder& withCollectorFactory(CompressPipelineConfig::CollectorFactory f);
        Builder& withSerializerFactory(CompressPipelineConfig::SerializerFactory f);

        // ---- 预处理 Operator（Phase 5 新增抽象） ----
        /// Mesh 构造后、Operator 初始化之前调用一次（PPMC 量化用）
        Builder& withPreInitPreprocessor(std::shared_ptr<PreProcessOperator> op);
        /// 主循环开始前调用一次（对齐旧 `options.setPreProcessOperator` 的一个位置）
        Builder& withPreprocessor(std::shared_ptr<PreProcessOperator> op);
        /// 每轮 i 开始时调用（ContextLevelTracking 用）
        Builder& withRoundPreprocessor(std::shared_ptr<RoundPreProcessor> op);

        // ---- 行为开关 ----
        Builder& withUpdateAfterEliminate(bool v);
        Builder& withEnableIndependentSet(bool v);
        Builder& withRecomputeVertexDegreesOnce(bool v);
        Builder& withHeaderCarryPrediction(bool v);
        Builder& withSeedStrategy(CompressPipelineConfig::SeedStrategy s);
        Builder& withCustomSeedSelector(
            std::function<MCGAL::Halfedge*(std::shared_ptr<MCGAL::Mesh>)> f);

        // ---- 通用钩子直通（给必要时 escape hatch） ----
        Builder& withBeforeCompressLoop(decltype(CompressPipelineConfig::beforeCompressLoop) f);
        Builder& withAfterRound(decltype(CompressPipelineConfig::afterRound) f);
        Builder& withBeforeSerialize(decltype(CompressPipelineConfig::beforeSerialize) f);

        // ---- Phase 10/13+ 算法 hook 注入 —— 纯存储槽；runCompress 不默认消费，
        //      由算法主函数通过 `pmsf::CompressPipelineConfig::getErrorMetric()`
        //      等 accessor 拉出来用。这让 impl/*.cpp 的 main 可以真正做
        //       `.withErrorMetric(std::make_shared<MyError>())` 而不必再把
        //       lambda 写在主文件里。
        Builder& withErrorMetric(std::shared_ptr<ErrorMetric> m);
        Builder& withPredictor(std::shared_ptr<Predictor> p);
        Builder& withEntropyCoder(std::shared_ptr<EntropyCoder> e);
        Builder& withCodebook(std::shared_ptr<Codebook> c);
        Builder& withFeatureDetector(std::shared_ptr<FeatureDetector> f);

        /// @brief Phase A.5: declarative capability-set API。新算法可写
        ///        `.withCapability(Capability::UpdateAfterEliminate)
        ///          .withCapability(Capability::ContextLevelTracking)` 替代
        ///        `.withUpdateAfterEliminate(true).withTrackContextLevel(true)`。
        ///        两种方式可共存：capabilities 字段是累加的。
        Builder& withCapability(Capability c);
        Builder& withCapabilities(CapabilitySet s);

        /// @brief 构建最终 CompressPipelineConfig —— 把 preprocessor list fold 进 lambda。
        CompressPipelineConfig build() const;

      private:
        Backend backend_ = Backend::Columnar;
        CompressPipelineConfig base_;
        std::vector<std::shared_ptr<PreProcessOperator>> preInitPreprocessors_;
        std::vector<std::shared_ptr<PreProcessOperator>> preprocessors_;
        std::vector<std::shared_ptr<RoundPreProcessor>> roundPreprocessors_;
    };

};

/// @brief 通用压缩主循环。把 PMSF.cpp 里 20 多个 xxxCompress 的共同逻辑集中在此。
///
/// 流程：
///   1) build PMSFHeader（含可选 enablePrediction 透传）
///   2) load mesh
///   3) cfg.preInitPreparation(mesh, header, options)  ← e.g. PPMC quantize
///   4) collector/serializer 由 cfg.factory 构造
///   5) select->options_.getRegisterFunc()(mesh) 如有
///   6) select->init(mesh) / eliminate->init / collector->init
///   7) seed 按 cfg.seedStrategy 落位（SelectSeed 可用 cfg.customSeedSelector）
///   8) seedIsRemovableOperator 加入链
///   9) options.getPreProcessOperator()->process(mesh) 如有
///  10) cfg.beforeCompressLoop(mesh, header, collector, options)
///  11) for i in 0..R:
///         cfg.beforeRound(i, mesh)
///         inner loop { select→[MIS]→eliminate→[update] }
///         cfg.afterRound(i, mesh)  或 options.getPostProcessOperator()->process(mesh)
///         collector->collect(seed); select->reset()
///  12) cfg.beforeSerialize(collector, serializer, mesh, seed)
///  13) serializer->init()
///  14) [if Columnar] serializer->setHeader(shared_ptr)
///      [else Binary]  serializer->serializeHeader(header)
///  15) serializer->serializeBaseMeshWithSeed(mesh, {seed}); serializer->serialize(path)
void runCompress(CompressOptions& options, const CompressPipelineConfig& cfg);

/// @brief 通用解压主循环。把 PMSF.cpp 里 20 多个 xxxDecompress 的共同逻辑集中在此。
///
/// 行为：
///   - init(path) 加载 buffer / parquet
///   - 若 deserializer 支持 header()（Columnar init 时已读入 metadata
///     parquet），直接取；否则调用 deserializeHeader。
///   - deserializeBaseMesh → 循环 deserializeOneRound(R 次)
///   - 如果 options.getOutputPath() 非空，dumpto_oldtype 写 .off
///
/// 返回的 mesh 存入 PMSF::mesh_ （runDecompress 是 PMSF 成员函数的包装调用者，
/// 因此 mesh_ 的设置放在 PMSF 成员函数层，而非此处）。
///
/// @param afterBaseMeshHook 可选钩子，在 deserializeBaseMesh 返回后、进入每轮
///        deserializeOneRound 循环之前调用。典型用途：PPMC 把 header.isEnablePrediction()
///        dynamic_cast 到 PPMCSymbolReadOperator 后下传。
std::shared_ptr<MCGAL::Mesh> runDecompress(
    DecompressOptions& options,
    std::function<void(const PMSFHeader&,
                       std::shared_ptr<DeserializeOperator>,
                       std::shared_ptr<MCGAL::Mesh>)> afterBaseMeshHook = nullptr);

}  // namespace pmsf

#endif
