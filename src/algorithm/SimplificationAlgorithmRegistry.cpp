/*
 * @file SimplificationAlgorithmRegistry.cpp
 * @brief 算法注册表 + `ISimplificationAlgorithm` 的框架默认实现。
 *
 * 默认实现链对应论文的「框架标准行为」：
 *   H1 默认链 = Basic + UnifiedManifold + Boundary 算子（src/operator/ 直接复用）；
 *   H3 默认 = 框架确定性环三角化；量化/预测/熵编码默认 = 框架内置件。
 * 一个「零优化」的新算法只实现 info() 即可跑通完整压缩→存储→解压。
 */
#include <cmath>

#include "algorithm/SimplificationAlgorithmRegistry.h"
#include "common/BoundedRingTraversal.h"

#include "common/TimeRecorder.h"
#include "algorithm/RetriProtocolRegistry.h"
#include "metric/ErrorMetric.h"
// P18.2 阶段 A：H7 跳现在真的接分段器了。分段 collector 仍不在这里 include ——
// 阶段 A 只产出真 groupId + 真 seeds；分段 collector 要等阶段 B（嵌套
// per-segment 循环）与阶段 C（解码端）才有意义。
#include "compress/SegmentedOptimizedSymbolCollectOperator.h"  // P18.2 阶段 B：分区 collector
#include "segmentation/BasicSegmentOperator.h"
#include "segmentation/SegmentationOption.h"
#include "core/CompressPipeline.h"
#include "core/DecompressPipeline.h"
#include "elimination/VertexRemovalEliminateOperator.h"
#include "operator/BasicIsRemovableOperator.h"
#include "operator/BoundaryIsRemovableOperator.h"
#include "operator/UnifiedManifoldIsRemovableOperator.h"
#include "options/CompressOptions.h"
#include "decompress/DeserializeOperator.h"
#include "options/DecompressOptions.h"
#include "core/SelectionPolicy.h"
#include "options/SelectOptions.h"
#include "selection/ErrorSource.h"
#include "selection/GreedySelector.h"
#include "selection/PriorityErrorSelector.h"
#include "selection/SelectOperator.h"
// PHASE59 物品 3 前置：勘察遍产物基名重定向 + 废件整族清理。
#include "storage/PMSFSymbolStorageAdapter.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <stdexcept>

#include "scheme/StorageSchemeRegistry.h"

namespace pmsf {

namespace {
/// 注册表键规范化：小写、去首尾空白。配置里的大小写与插件声明必须命中同一条。
std::string normalizeKey(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
        if (c == ' ' || c == '\t' || c == '\n' || c == '\r') continue;
        out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
    return out;
}
}  // namespace

// ===========================================================================
// SimplificationAlgorithmRegistry
// ===========================================================================

SimplificationAlgorithmRegistry& SimplificationAlgorithmRegistry::instance() {
    static SimplificationAlgorithmRegistry registry;
    return registry;
}

void SimplificationAlgorithmRegistry::add(const std::string& name, Factory factory) {
    if (name.empty()) {
        throw std::runtime_error("[SimplificationAlgorithmRegistry] 算法名不能为空（info().name）");
    }
    auto [it, inserted] = factories_.emplace(normalizeKey(name), std::move(factory));
    if (!inserted) {
        throw std::runtime_error("[SimplificationAlgorithmRegistry] 算法名重复注册: " + name);
    }
}

std::shared_ptr<ISimplificationAlgorithm> SimplificationAlgorithmRegistry::create(
    const std::string& name) const {
    auto it = factories_.find(normalizeKey(name));
    if (it == factories_.end()) {
        return nullptr;
    }
    return it->second();
}

bool SimplificationAlgorithmRegistry::has(const std::string& name) const {
    return factories_.find(normalizeKey(name)) != factories_.end();
}

std::vector<std::string> SimplificationAlgorithmRegistry::names() const {
    std::vector<std::string> out;
    out.reserve(factories_.size());
    for (const auto& kv : factories_) {
        out.push_back(kv.first);
    }
    std::sort(out.begin(), out.end());
    return out;
}

// ===========================================================================
// ISimplificationAlgorithm —— 默认 hook
// ===========================================================================

IsRemovableChain ISimplificationAlgorithm::removability() const {
    // H1 默认链：可消除性 = Basic ∧ UnifiedManifold ∧ Boundary（论文的框架标准行为）。
    IsRemovableChain chain;
    chain.push_back(std::make_shared<BasicIsRemovableOperator>());
    chain.push_back(std::make_shared<UnifiedManifoldIsRemovableOperator>());
    // PHASE61 物品 1 ①：**测量用**开关（env 门控，默认关闭）。
    //
    // 附 26c 推断「分区细 ⇒ 边界被 BoundaryIsRemovableOperator 保护 ⇒
    // 基网不能缩小 ⇒ 出带」。那是**推断**，本开关把它变成**可测**：
    // 摘掉该算子，看基网塌到多少 ⇒ 得到杠杆的量级。
    // ⚠️ 摘掉会改变消除结果 ⇒ 属**以质量换 bpv**，仅供测量，不得作为交付口径。
    if (std::getenv("PMSF_NO_BOUNDARY_PROTECT") == nullptr &&
        std::getenv("PMSF_OWNED_BOUNDARY") == nullptr) {
        chain.push_back(std::make_shared<BoundaryIsRemovableOperator>());
    }
    return chain;
}

CapabilitySet ISimplificationAlgorithm::capabilities() const {
    // 由「哪些 hook 被重写」自动推导，取代 CompressOptions::Builder 的手填开关。
    CapabilitySet caps;
    if (vertexShift()) {
        caps.add(Capability::HookVertexShift);
    }
    if (edgeFlipRule()) {
        caps.add(Capability::HookEdgeFlip);
    }
    if (segmenter()) {
        caps.add(Capability::HookSegmentation);
    }
    if (boundaryRule() || boundaryConfigurator()) {
        caps.add(Capability::HookBoundaryHandling);
    }
    if (reverseVertexShift()) {
        caps.add(Capability::HookReverseVertexShift);
    }
    if (declareProperties()) {
        caps.add(Capability::HookDeclaredProperties);
    }
    return caps;
}

namespace {

/// 按后端取框架标准管线配置。
CompressPipelineConfig standardPipelineConfig(Backend backend,
                                               const std::string& algorithmName) {
    switch (backend) {
        case Backend::Binary:   return CompressPipelineConfig::defaultBinary();
        case Backend::Columnar: return CompressPipelineConfig::defaultColumnar();
        case Backend::Unified:  return CompressPipelineConfig::defaultUnified();
        case Backend::Paper:    return CompressPipelineConfig::defaultPaper(algorithmName);
    }
    return CompressPipelineConfig::defaultColumnar();
}

const char* backendTag(Backend backend) {
    switch (backend) {
        case Backend::Binary:   return "binary";
        case Backend::Columnar: return "columnar";
        case Backend::Unified:  return "unified";
        case Backend::Paper:    return "paper";
    }
    return "unknown";
}

}  // namespace

void ISimplificationAlgorithm::compress(const PMSFConfig& cfg) {
    const AlgorithmInfo meta = info();
    const CompressionProfile profile = compressionProfile();

    // ---- H1 + H2 → SelectOptions ------------------------------------------
    auto selectBuilder = SelectOptions::Builder();
    // 框架标准选择源 = Halfedge：连通驱动算法没有误差度量时，队列按顶点度数打分
    // （HalfedgeSelectionQueue::computeError 的默认分支），与 Default 路径一致。
    selectBuilder.withErrorSource(ErrorSource::Halfedge);
    // 算法特有的选择策略（候选打分 / 误差覆盖 / 新增半边放行）。
    // 不声明策略的算法得到全默认值 ⇒ 与历史行为逐字节一致。
    selectBuilder.withSelectionPolicy(selectionPolicy());

    const auto propInit = declareProperties();

    // H2：误差度量工厂在网格就绪后调用一次，装配 accessor 与 eliminate 后刷新。
    // 属性注册与度量构造的顺序敏感：registerProperties 必须先跑，度量才能取到属性句柄。
    if (const auto metricFactory = errorMetric()) {
        auto metric = std::make_shared<std::shared_ptr<ErrorMetric>>();
        selectBuilder.withRegisterProperties(
            [propInit, metricFactory, metric](std::shared_ptr<MCGAL::Mesh> m) {
                if (propInit) {
                    propInit(m);
                }
                if (m) {
                    *metric = metricFactory(m);
                }
            });
        const ErrorAccessStyle style = errorAccessStyle();
        selectBuilder.withHalfedgeErrorAccessor(
            [metric, style](MCGAL::Halfedge* e, MCGAL::Point& p) -> float {
                if (e == nullptr) return std::numeric_limits<float>::infinity();
                if (!*metric) return std::numeric_limits<float>::infinity();
                if (style == ErrorAccessStyle::OptimalPlacement) {
                    // 度量自己写 target（最优放置）。
                    return static_cast<float>((*metric)->costWithTarget(e, p));
                }
                // 框架填中点，再问与 target 无关的 cost。
                // ── PHASE64：与 `costWithTarget` 同源的空指针保护 ──────────
                // 上一条 `e == nullptr` 只防了半边本身；`opposite()` 在**边界
                // 半边**上为空（附 84），此处不防会在开边界大网格上段错误。
                // 约定与上面一致：不可坍缩 ⇒ 无穷。
                // 字节中性理由见 OptimalPlacementQuadricError.h（该路径在
                // bunny/horse 上从未触发）。
                if (e->vertex() == nullptr || e->opposite() == nullptr ||
                    e->opposite()->vertex() == nullptr) {
                    return std::numeric_limits<float>::infinity();
                }
                const MCGAL::Point a = e->vertex()->point();
                const MCGAL::Point b = e->opposite()->vertex()->point();
                p = MCGAL::Point(0.5f * (a.x() + b.x()), 0.5f * (a.y() + b.y()),
                                 0.5f * (a.z() + b.z()));
                return static_cast<float>((*metric)->cost(e));
            });
        // 只有确实需要刷新的度量才安装该回调 —— 「装不装」本身是可观测行为
        // （它改变 SelectOptions 的字段，进而影响选择器的拷贝/判空路径），
        // 因此由算法显式声明，而不是框架一律装上。
        if (needsAfterEliminateRefresh()) {
            selectBuilder.withOnAfterEliminate([metric](MCGAL::Vertex* s) {
                if (!*metric || s == nullptr || s->isRemoved()) return;
                for (MCGAL::Halfedge* he : pmsf::boundedRing(s)) {
                    if (he && !he->isRemoved()) {
                        (*metric)->onAfterEliminate(he);
                        return;
                    }
                }
            });
        }
    } else if (propInit) {
        selectBuilder.withRegisterProperties(propInit);
    }

    // H1：可消除性算子链。
    for (const auto& op : removability()) {
        selectBuilder.withIsRemovableOperator(op);
    }
    auto sopt = selectBuilder.build();

    // 选择器：几何驱动用优先队列（H2 的度量在此排序）；连通驱动用贪心。
    std::shared_ptr<SelectOperator> selectorOp =
        (selector() == SelectorKind::Greedy)
            ? std::static_pointer_cast<SelectOperator>(std::make_shared<GreedySelector>(*sopt))
            : std::static_pointer_cast<SelectOperator>(
                  std::make_shared<PriorityErrorSelector>(*sopt));

    // 消除算子：默认顶点删除（论文 §3.1.5 的统一消除抽象）。
    auto eliminateOp = eliminate();
    if (!eliminateOp) {
        eliminateOp = std::make_shared<VertexRemovalEliminateOperator>();
    }

    // ---- CompressOptions ---------------------------------------------------
    auto coptBuilder = CompressOptions::Builder()
                    .setRound(cfg.effectiveRounds())
                    .setPath(cfg.input)
                    .setOutputPath(cfg.output)
                    .setEnablePrediction(profile.prediction)
                    // P18.1：config 显式写了 quantization 就以它为准
                    // （"quantization": false 即 F32 档）；缺席才回落 profile。
                    // 门禁配置均显式写 quantization ⇒ 行为逐字节不变。
                    .setEnableQuantization(cfg.quantizationSet ? cfg.quantization
                                                              : profile.quantization)
                    // P18.2（E2）：config 显式写 `segments` ⇒ 启用分区并把 N
                    // 下发给 SegmentationOption（此前 setNumSegments 全树零调用方）。
                    // 缺席 ⇒ 回落 profile，行为逐字节不变。
                    // P18.2 修正：判据从 `>= 0` 改成 `> 1`。
                    //
                    // 原来 `segments: 1` 也置位 —— 但 N==1 走的是**默认
                    // collector**（单分区在语义上就是不分区，见 H7 那跳），
                    // 产物里根本没有分段 blob。该位却写进 metadata，于是
                    // 解码端的产物侧守卫（ColumnarDeserializeOperator）
                    // 会**误拒一个本来能解的 N=1 产物**（实测：加守卫后
                    // 正常解码从 rc=0 变成 rc=1）。
                    //
                    // 该位必须如实反映编码端**实际用了哪个 collector**：
                    // 只有 segments > 1 才真的换成分段 collector。
                    .setEnableSegmentation(cfg.segmentsSet ? (cfg.segments > 1)
                                                           : profile.segmentation)
                    .setEnableAdjustment(profile.adjustment)
                    .setEnableCompress(profile.compression)
                    .setSelect(selectorOp)
                    .setEliminate(eliminateOp);
    if (const auto pre = preProcessOperator()) {
        coptBuilder.setPreProcessOperator(pre);
    }
    if (const auto post = postProcessOperator()) {
        coptBuilder.setPostProcessOperator(post);
    }
    if (cfg.segmentsSet) {
        SegmentationOption seg;
        seg.setNumSegments(cfg.segments);
        coptBuilder.setSegmentationOption(seg);
    }
    auto copt = coptBuilder.build();

    if (const auto optTuner = compressOptionsTuner()) {
        optTuner(copt);
    }

    // 决定性三角化协议的覆盖路径（设计文档 §2.1.3）。算法只声明协议 id 与调参；
    // 实现在 impl/ 侧按 id 查表，src/ 不认识任何协议名之外的东西。
    // 协议覆盖只对列式后端成立：这些协议路径内部构造的是
    // `CompressPipelineConfig::defaultColumnar()`（协议载荷本身依赖列式段表）。
    // 这与历史实现里的 `cfg.backend == Backend::Columnar && tryRun...` 守卫一致 ——
    // 不加守卫时 binary 后端会被错误地写成一堆列式产物。
    TimeRecorder recorder0(meta.name + "_" + backendTag(cfg.backend) + "_compress");
    // M4-P0：Paper 后端的协议覆盖是 opt-in（PMSF_PAPER_RETRI=1）。默认关闭时
    // paper 压缩仍走标准 bridge 管线，产物逐字节不变；打开时覆盖路径必须拿到
    // 真实 backend，才能构造 `defaultPaper()` 而不是硬编码的 columnar 管线。
    const auto retriTuning = retriangulationTuning();
    const bool retriOverrideAllowed =
        cfg.backend == Backend::Columnar ||
        (cfg.backend == Backend::Paper &&
            (pmsf::paperRetriEnabled() || !retriTuning.paperArtifactAlgorithm.empty()));
    if (retriOverrideAllowed &&
        pmsf::tryRunRetriCompressOverride(copt, retriangulationProtocol(),
                                          retriTuning, cfg.backend)) {
        return;
    }

    TimeRecorder recorder(meta.name + "_" + backendTag(cfg.backend) + "_compress");
    const auto perfPhaseOn = [] {
        const char* e = std::getenv("PMSF_PERF_PHASE");
        return e && *e && *e != '0';
    };
    const auto perfT0 = std::chrono::steady_clock::now();

    // ---- 管线配置：框架标准 + 算法的 H8 / 微调 ------------------------------
    auto pipe = standardPipelineConfig(cfg.backend, meta.name);

    // 框架标准预处理：把 CompressOptions 的量化开关落到网格状态，并按
    // PMSF_DEFAULT_QUANT_BITS 设定量化步长。这段逻辑与算法无关（原本散落在
    // PMSF::parquetV2Compress 里，属于设计文档 P3 的「核心层算法赘肉」），
    // 这里上提为框架对全部契约算法的标准行为。
    pipe.preInitPreparation = [](std::shared_ptr<MCGAL::Mesh> mesh, PMSFHeader&,
                                 const CompressOptions& options) {
        if (!mesh) return;
        mesh->enableQuantization = options.isEnableQuantization();
        if (!options.isEnableQuantization()) return;
        float maxRange = 0.0f;
        for (unsigned axis = 0; axis < 3; ++axis) {
            maxRange = std::max(maxRange, mesh->bboxMax[axis] - mesh->bboxMin[axis]);
        }
        int quantBits = 12;
        bool explicitBits = false;
        if (const char* quantBitsEnv = std::getenv("PMSF_DEFAULT_QUANT_BITS");
            quantBitsEnv && *quantBitsEnv) {
            const int q = std::atoi(quantBitsEnv);
            if (q >= 4 && q <= 22) {
                quantBits = q;
                explicitBits = true;
            } else {
                std::fprintf(stderr, "[framework] ignore invalid PMSF_DEFAULT_QUANT_BITS=%d\n", q);
            }
        }
        if (!explicitBits) {
            // Correctness guard: Q12 on a large-extent mesh can map distinct
            // vertices to the same quantized position (degenerate faces /
            // ambiguous merge). Pick enough bits to keep the shortest edge
            // distinguishable, clamped to a safe storage range.
            double minEdge = 0.0;
            bool haveEdge = false;
            for (MCGAL::Facet* f : mesh->faces()) {
                if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                MCGAL::Halfedge* h0 = f->proxyHalfedge();
                MCGAL::Halfedge* h = h0;
                int guard = 0;
                do {
                    if (h && h->vertex() && h->end_vertex()) {
                        const MCGAL::Point& a = h->vertex()->point();
                        const MCGAL::Point& b = h->end_vertex()->point();
                        const double dx = static_cast<double>(a.x()) - b.x();
                        const double dy = static_cast<double>(a.y()) - b.y();
                        const double dz = static_cast<double>(a.z()) - b.z();
                        const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
                        if (len > 0.0 && (!haveEdge || len < minEdge)) {
                            minEdge = len;
                            haveEdge = true;
                        }
                    }
                    h = h ? h->next() : nullptr;
                } while (h && h != h0 && ++guard < 64);
            }
            if (haveEdge && minEdge > 0.0 && maxRange > 0.0f) {
                const double ratio = static_cast<double>(maxRange) / minEdge;
                const int needed = static_cast<int>(std::ceil(std::log2(ratio))) + 1;
                // Cap at 16: the bit-plane codec/decoder has only been validated
                // up to 16 quant bits; >16-bit artifacts were observed to hang on
                // decode. Remaining collisions are repaired by
                // PMSF_REPAIR_DEGENERATE=1 (enabled by the standard bench configs).
                quantBits = std::clamp(needed, 12, 16);
            }
        }
        mesh->i_nbQuantBits = static_cast<unsigned>(quantBits);
        mesh->f_quantStep = maxRange / static_cast<float>(1u << mesh->i_nbQuantBits);
    };
    if (const auto collector = collectorFactory()) {
        pipe.collectorFactory = collector;
    }
    // P18.2 E2 第三跳 + E3：H7（`Segmenter`）非空 ⇒ 需要**分段 collector**。
    //
    // ---------------------------------------------------------------------
    // 本跳**没有可接的装配**，故显式拒绝，而不是换一个会产出坏产物的 collector。
    // 两条候选路都止步于同一个缺失的管线阶段（P18.2 物品 3 书面归因）：
    //
    // ① `SegmentationSymbolCollectOperator`：自持 `[segment][round]` 二维队列，
    //    但**无出口**（无 exportToBufferWithRangeCoder）且 layout 未登记；首次
    //    实例化即段错误（rc=139，gdb: segmentFacetSymbolQueues[groupIdx] 越界
    //    —— 其惰性 resize + groupIdx=0 写在 `collect(vector<seeds>)` 重载里，
    //    而 pipeline 从不调用该重载）。
    //
    // ② `SegmentedOptimizedSymbolCollectOperator`：出口与 layout 都齐
    //    （CollectorLayoutPlugin.cpp:88），换入后越过 dispatch 进入 coder ——
    //    但**必然**卡在
    //        BUG: rescaling left 21 total frequency   (qsmodel.c dorescale)
    //    只写出 out_base_mesh.parquet。
    //
    // ## ② 的根因（已证，非猜测）
    //
    // 该类 `ensureCollectors(count)` 按 seeds 数建内层 collector 并逐个
    // `setGroupFilter(i)`；内层 `OptimizedSymbolCollectOperator` 据此过滤
    //    (OptimizedSymbolCollectOperator.cpp:36) h->face()->groupId() == groupFilter_
    // 而 **`groupId` 从未被赋值**：
    //    - 默认 -1（deps/MCGAL/include/Halfedge.h:248 `int groupId_ = -1;`）
    //    - 赋值只发生在 `src/segmentation/*SegmentOperator.cpp` 的 `segment()` 里
    //      （BasicSegmentOperator.cpp:47 / VertexSegmentOperator.cpp:73 /
    //       PreDefinedFacetSegmentOperator.cpp:130 …）
    //    - **而压缩管线从不构造、也从不调用任何 SegmentOperator**
    //      （src/core/、src/algorithm/、tools/ 全树零调用点）
    //
    // 于是 `CompressPipeline.cpp:534` 的
    //        collector->init(mesh, {seed}, header);
    // 永远只给**一个** seed ⇒ `ensureCollectors(1)` ⇒ `groupFilter_ = 0`，
    // 而每个面的 groupId 恒为 -1 ⇒ 内层 collector 的 group-0 视图**恒为空**；
    // 其字母表却取自 `header_`（整个网格）⇒ 空视图 + 全字母表产出不一致符号流
    // ⇒ range coder 模型频率漂移 ⇒ dorescale 不变量被破坏。
    //
    // 同一缺口的另一面：`SegmentationOption::getNumSegments()` / `getSeeds()`
    // 全树**零消费者** —— E2 那跳写进去的 `segments: N` 存下后没人读过。
    // N 现在不可能影响任何事，故物品 4（8 路 N 扫描）与物品 5
    // （tab:partition_bpv）与物品 3 是**同一个阻塞**，不是三个。
    //
    // ⇒ 缺的不是接线，是**一个管线阶段**：在 collector->init 之前跑
    //   `SegmentOperator::segment(mesh)` 产出真 seeds + 真 groupId，在
    //   `CompressPipeline.cpp:534` 传真 seeds，解码端再对称反向。这是
    //   P13 conquest 量级的工程。
    //
    // 按 P16 硬约束「同一根因卡 3 轮 → 书面归因，回退优先于强推」：此处回退，
    // 并让失败**响亮且准确** —— 报出缺什么，而不是产出坏产物或假装分区成功。
    // ---------------------------------------------------------------------
    //
    // ---------------------------------------------------------------------
    // P18.2 阶段 A：**接上分区阶段本身**。
    //
    // 上面那条「框架没有分区管线阶段」的归因（见 docs/refactor/
    // P18.2_PARTITION_STAGE_ATTRIBUTION.md）在本轮被当成待办实现掉：
    // `CompressPipeline.cpp` 的第 10 步现在会先跑 `segmentOperatorFactory()`
    // 产出的 SegmentOperator，再把 `exportSeeds()` 的真 seeds 喂给
    // collector->init —— 于是 facet 的 groupId 第一次真的被赋值。
    //
    // 本块只负责**把分段器的构造挂上去**；分区数从 H7 的
    // `SegmentationOption::getNumSegments()` 取（这正是此前全树零消费者的
    // 那个接口 —— config `segments` 经 Registry 的 E2 那跳写进去）。
    //
    // 默认路径零字节变化：不返回 `segmenter()` 的算法 ⇒ 本块不执行 ⇒
    // `pipe.segmentOperatorFactory` 保持为空 ⇒ 管线走 `{seed}`。
    // P18.2 路线 (a) 两遍改造的接线位（消费点在本函数末尾）。
    //
    // 为什么两遍：`segment()` 的分区对象与解码端能重算的对象必须是**同一个网格**。
    // step 10 手上是全网格（实测 34,834 顶点），而解码端 round 1 手上是基网
    // （实测 7,081 顶点）—— 两者不是同一个东西（附 8/附 15 实测）。
    // 故第一遍跑到基网、分区、再沿坍缩记录把分组回填成一张
    // 「顶点序号 → 分区」表；第二遍带着这张表重跑一遍消除。
    //
    // ⚠️ 附 16 实测**否证**了当初的期望「两遍的基网相同（分组不影响消除）」：
    //    分组**会**改变消除 —— `BoundaryIsRemovableOperator`（默认 is-removable
    //    链的一员）用 `groupId` 判定分区边界并拒绝跨区坍缩。故两遍的基网
    //    实测不同（121218 vs 121174 字节）。三点对照证明这与 collector/seeds
    //    无关：把第二遍的 collector 换回不分区、seeds 换回 1 个，基网
    //    **逐字节不变**（121174 == 121174）—— 变的只有 groupId。
    //    这张表因此**不能**由解码端从基网重算 —— 它必须**随产物下发**。
    //    保留本骨架是因为它产出的那张表**就是**分组可传输的表示。
    bool segTwoPass = false;
    CompressPipelineConfig::CollectorFactory segPlainCollectorFactory;
    if (const auto segmentHook = segmenter()) {
        const SegmentationOption segOption = segmentHook->options();
        // 分区数取哪个：**顶层 `segments` 键优先**（`segmentsSet` 存在位，仿
        // `quantizationSet`）。这正是 P18.2 物品 2 要求的链路
        //    config `segments` → SegmentationOption::setNumSegments → 装配
        // 此前只有算法自己的 `algorithm_options.segments` 能影响分区数，
        // 顶层键写了也没人读；`segmentsSet` 为假时保持算法侧的值不变
        // （既有配置不写该键 ⇒ 行为不变）。
        const int numSegments =
            cfg.segmentsSet ? cfg.segments : segOption.getNumSegments();
        if (numSegments <= 0) {
            throw std::runtime_error(
                "[assembly] 算法 '" + info().name +
                "' 声明了 segmenter()，但分区数 <= 0：分区数需经 config 的 "
                "`segments` 键显式给出（-> SegmentationOption::setNumSegments）。"
                "0 表示「由系统内定」，而系统内定这一档尚未实现，故先显式拒绝，"
                "以免静默退化成不分区。");
        }
        // N==1 直通：单分区在语义上就是不分区，分段机制（BasicSegmentOperator +
        // 分段 collector + seed/通道前缀）对它是纯开销 —— 实测 N=1 走分段路径比
        // 论文 tex 的 N=1 口径（未分区便宜路径）慢 ~1.3×/~2.0×（enc/dec）。
        // 跳过整条分段装配后 N=1 回落到默认管线，产物与「不写 segments」的
        // 默认算法同构，解码端由 metadata 的 segmentation 位自动走非分段路径。
        // N>=2 分支逻辑未动 ⇒ 其产物逐字节不变。
        if (numSegments > 1) {
        pipe.segmentOperatorFactory =
            [numSegments]() -> std::shared_ptr<SegmentOperator> {
            auto segmentOperator =
                std::make_shared<BasicSegmentOperator>(numSegments);
            // P18.2 PHASE60 物品 1：exact-N 模式**改为默认开启**。
            //
            // 此前它需显式 `PMSF_SEG_EXACT_N=1` 才开，理由是「既有行为逐字节
            // 不变」。**该理由已被实测推翻**：不开时分组数**不是**请求的 N ——
            // 实测 N=128 → `groups=129`，而第 129 组在基网上**没有存活面**
            // （解码端 `seeded=128 unseeded=1`）⇒ 该组的符号无从起 BFS
            // ⇒ 解码丢 189 顶点（34,645 / 69,286 vs 原 34,834 / 69,664）。
            //
            // 分组数失准正是 merge 分支（`cnt < target/2 || idx > sampleNumber`）
            // 与 `idx` 可越过 `sampleNumber` 造成的；exact-N 关掉它们，
            // 并让末组吸收余量 ⇒ 组数恰为 N。该模式同时消除空组自旋
            // （见 BasicSegmentOperator.cpp 的空组进展守卫）。
            //
            // **默认路径零影响**：默认路径（不写 `segments`）压根不构造
            // `BasicSegmentOperator`，本行不可达。
            // `PMSF_SEG_EXACT_N=0` 仍可显式退回旧行为（供对照实验）。
            const char* exact = std::getenv("PMSF_SEG_EXACT_N");
            segmentOperator->setExactNumSegments(exact == nullptr || std::string(exact) != "0");
            return segmentOperator;
        };

        // P18.2 阶段 B：分区 collector。
        //
        // 换入 `SegmentedOptimizedSymbolCollectOperator` 现在**有了前提**：
        // 上面刚接上的分区阶段会给每个 facet 写真实 groupId、并给出长度 = N
        // 的真 seeds —— 而该类正是按 `groupId == groupFilter_(i)` 过滤的
        // （`OptimizedSymbolCollectOperator.cpp:36`）。阶段 A 之前它看到的是
        // 恒为 -1 的 groupId ⇒ 空视图 + 全字母表 ⇒ qsmodel 的
        // "BUG: rescaling"；那个失败是**分区不存在**导致的，不是本类的问题。
        //
        // 出口：`exportToBufferWithRangeCoder` 产出
        //   [groupCount][seedA[G]][seedB[G]][groupStart[G+1]]
            //   [baseFacetCount][rleRunCount][(gid:int8)(runLen:int32|varint) × runs][payload]
            //   A negative rleRunCount selects varint run lengths.
        // （基网 facet → 分区通道仅 G > 1 时存在；PHASE59 物品 3 加上，
        //   消费者见 ColumnarDeserializeOperator 的 layout-32 前缀解析）
        // 且 layout 已在 impl/common/CollectorLayoutPlugin.cpp:88 登记。
        //
        // 默认路径零字节变化：本块只在 `segmenter()` 非空时执行。
        //
        // **N >= 1 都换分段 collector**（P: 让 N=1 并入 N>=2 的分区曲线）。
        //
        // 旧注记曾说「N==1 时不换」，理由是「单分区 collector 产不出自洽流
        // （N=1 rc=1 `BUG: rescaling left 24`）」。该 rc=1 早被此前的
        // off-by-one 修复（OptimizedSymbolCollectOperator.cpp coord 模型钳位
        // `>` → `>=`）与 exact-N 默认化消解 —— 实测 N=1 压缩 rc=0、流自洽
        // （184 基网 + 34650 符号 = 34834 顶点）。残留的失败在**解码端**：单分区
        // seed 不自洽（分区器那条 seed 不入基网 ⇒ 通道写 -1 ⇒ 两侧 BFS 起点
        // 不同）。该根因已在 CompressPipeline（单分区 seed 取主循环 seed）与
        // SegmentedRangeCodedDecode（seed 通道消费放宽到 groupNumber>=1）两处
        // 修掉，N=1 现逐项回环（34834/69664，三门全绿）。
        //
        // 作用域：本块只在 `segmenter()` 非空时执行（当前仅 efoss）。不返回
        // segmenter 的算法（default/QEM/PPMC/... 多网格/逐算法路径）**不进本块**，
        // 字节零变化。N>=2 分支逻辑未动 ⇒ 其产物逐字节不变。
        if (numSegments >= 1) {
            segPlainCollectorFactory = pipe.collectorFactory;
            segTwoPass = true;
            pipe.collectorFactory = []() -> std::shared_ptr<SymbolCollectOperator> {
                return std::make_shared<SegmentedOptimizedSymbolCollectOperator>();
            };
        }
        }  // if (numSegments > 1) —— N==1 直通，见上方注记
    }
    if (const auto boundary = boundaryConfigurator()) {
        boundary(pipe);
    }
    if (const auto tuner = pipelineTuner()) {
        tuner(pipe, cfg.backend);
    }
    if (const auto beforeLoop = beforeCompressLoop()) {
        pipe.beforeCompressLoop = beforeLoop;
    }

    // P18.2 PHASE59 附 21：**判别实验开关** —— 绕开两遍包装，跑回旧的一遍路径。
    //
    // 两遍包装唯一的立项理由是「seeds/groupIds 两侧各自重算」，而附 16 已实测
    // 把那条路证伪（分组必须随产物下发）。既然反正要传，两遍就只剩负担：
    //   1. 编码翻倍 + 必须放大 PMSF_POOL_*（第二个网格会耗尽 arena）；
    //   2. 人为造出**两个不同的基网**；
    //   3. **pass 2 的 seeds 取自全网格**（applySegmentGroupTable 返回全网格
    //      proxyHalfedge），而解码端只能从**基网**重算 seeds ⇒ 两边的每组
    //      BFS 起点**永远不可能相同**。
    //
    // 第 3 条正是这几轮一直在找的「第 0 轮之后分岔」的现成候选解释。
    // 故用本开关做一次判别：N=2 走**旧的一遍路径**（step 10 在全网格上
    // segment()，exportSeeds() 直接喂 collector），看它能不能回环。
    // 默认关闭 ⇒ 行为零变化。
    // P18.2 PHASE59 附 22：**两遍包装已回退为 opt-in**（`PMSF_SEG_TWO_PASS=1`）。
    //
    // 判别实验（附 21）已经判定：一遍路径 N∈{1..128} **8 档全部精确回环**，
    // 而两遍在 N=2 就不对。两遍的唯一立项理由（「两侧各自重算」）早被附 16
    // 证伪，它自己反而是失败的直接来源 —— 故默认走**一遍**。
    //
    // 代码保留不删：`segmentGroupTableOut` 那张表在附 16 里被证明
    // 「就是分组的可传输表示」，将来若要「多进程/离线分区」仍可能要它。
    // 默认关闭 ⇒ 默认路径与旧行为零差异（旧行为本来就只有 segments>1 才走）。
    const bool twoPassOptIn = std::getenv("PMSF_SEG_TWO_PASS") != nullptr;
    if (segTwoPass && twoPassOptIn) {
        // 第一遍（勘察遍）：**不分区**，跑完后在基网上分区 + 回填，
        // 产出「顶点序号 → 分区」表。
        auto groupTable = std::make_shared<std::vector<int>>();
        CompressPipelineConfig pass1 = pipe;
        pass1.collectorFactory = segPlainCollectorFactory;
        pass1.segmentGroupTableOut = groupTable;
        pass1.segmentGroupTableIn.reset();
        CompressOptions pass1Options = copt;
        // 勘察遍的产物基名必须与分区遍**分开**。
        //
        // 为什么（PHASE59 物品 3 实测）：分区遍走 residual-only 布局，
        // `ColumnarSerializeOperator` 按设计只写 residual 表、**不写** lod 表
        // （"Skip it and let the decoder reconstruct from the single residual row"），
        // 而勘察遍写的是完整家族（`_bit` 2 行 + `_lod` 1 行）。两者共用基名时
        // 勘察遍的 `_bit` / `_lod` 原地留下，解码端 `reassembleBlobFromSegmentsV4`
        // 便拿勘察遍的 lod 行（stream_count=2、residual_len=58074）去拼分区遍的
        // residual（110,261 B）⇒ 得到 24 + 50,778 + 58,074 = 161,063 B 的混合
        // blob，前四个 int 是 17088/33690/58074/295882（垃圾），分区前缀永远读不到。
        //
        // 故：勘察遍写 `<真实基名>_p1tmp`，跑完立即整族删除。
        // 本块只在 `segments > 1` 时执行 ⇒ 默认路径零变化。
        const std::string pass1Base = pmsf::paperStorageStripExtension(copt.getOutputPath()) + "_p1tmp";
        // 诊断开关 PMSF_SEG_PASS1_ONLY 保留旧语义：勘察遍直接写真实基名、
        // 产物留在原处供人工检视（此时整族不重定向、不清理）。
        const bool pass1Only = std::getenv("PMSF_SEG_PASS1_ONLY") != nullptr;
        pass1Options.setOutputPath(pass1Only ? copt.getOutputPath() : pass1Base);
        pmsf::runCompress(pass1Options, pass1);

        if (pass1Only) {
            return;
        }
        pmsf::removePaperArtifactsForBase(pass1Base);

        // 第二遍（分区遍）：把表套用到全网格，带分组重跑。
        //
        // ⚠️ 两遍各自载入一份网格，而 `ContextPool` 的 arena **不释放**
        // （`MeshRuntime.cpp` 的 `registerPool` 只在首次调用时执行）——
        // 故本路径要求放大的 `PMSF_POOL_V/H/F`，与附 15 实测一致。
        pipe.segmentGroupTableIn = groupTable;
        pipe.segmentGroupTableOut.reset();
        // 诊断开关（P18.2 附 16）：第二遍仍用**不分区的 collector**，
        // 于是两遍之间唯一的差别只剩「facet 上有没有 groupId」。
        // 若基网仍不同 ⇒ 差别来自**消除路径读 groupId**，与 collector 无关。
        if (std::getenv("PMSF_SEG_PASS2_PLAIN_COLLECTOR")) {
            pipe.collectorFactory = segPlainCollectorFactory;
        }
        CompressOptions pass2Options = copt;
        pmsf::runCompress(pass2Options, pipe);
        return;
    }

    if (perfPhaseOn()) {
        const double setupMs = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - perfT0).count();
        std::fprintf(stderr, "[PERF_PHASE] setup_before_run=%.1f ms\n", setupMs);
    }
    const auto perfRun0 = std::chrono::steady_clock::now();
    pmsf::runCompress(copt, pipe);
    if (perfPhaseOn()) {
        const double runMs = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - perfRun0).count();
        std::fprintf(stderr, "[PERF_PHASE] runCompress_wall=%.1f ms\n", runMs);
    }
}

DecompressOptions ISimplificationAlgorithm::makeDecompressOptions(
    Backend backend, const std::string& input, const std::string& output) const {
    const AlgorithmInfo meta = info();
    std::shared_ptr<DeserializeOperator> deserializer;
    if (backend == Backend::Columnar) {
        // 协议覆盖的产物只有配套 reader 读得懂（reader/reconstructor 与压缩端一一对应）。
        // 算法没声明协议时（有些协议取决于环境变量），解压器会按产物自带的
        // blob_layout 再决定一次 —— 决策依据是产物，不是进程环境。
        const auto triple = pmsf::makeDecompressOperatorsForAlgorithm(meta.name, backend);
        deserializer = pmsf::makeRetriDeserializer(retriangulationProtocol(), triple.reader,
                                                   triple.reconstructor);
    }
    if (backend == Backend::Paper && pmsf::paperRetriEnabled()) {
        // M4-P0：paper 后端的 retri 解码走原生 reader pair + PaperDeserializeOperator
        // （容器是单文件 `_paper.parquet`，不能用 columnar 解压器）。协议为空/非
        // retri 的算法返回 nullptr，保留既有 paper 三元组；协议非空但 reader 未
        // 实现时工厂抛异常 fail-fast，不得静默回落到 generic。
        const auto triple = pmsf::makeDecompressOperatorsForAlgorithm(meta.name, backend);
        deserializer = pmsf::makeRetriPaperDeserializer(
            retriangulationProtocol(), triple.reader, triple.reconstructor);
    }
    if (!deserializer) {
        deserializer = pmsf::makeDeserializeOperatorForAlgorithm(meta.name, backend);
    }
    return DecompressOptions::Builder()
        .setDeserializeOperator(deserializer)
        .setPath(input)
        .setOutputPath(output)
        .build();
}

void ISimplificationAlgorithm::decompress(const PMSFConfig& cfg) {
    const AlgorithmInfo meta = info();
    TimeRecorder recorder(meta.name + "_" + backendTag(cfg.backend) + "_decompress");

    // 输出路径延后设置：postDecompressHook（如退化三角形修复）必须在写盘之前
    // 施加到网格上，因此先只解码、收尾、最后再落盘。
    TimeRecorder recorder0(meta.name + "_" + backendTag(cfg.backend) + "_decompress");
    // Prefix/LOD queries need the deserializer state machine (and its
    // setQueryMaxLod); the retri override writes a full mesh directly and
    // bypasses max_lod, so skip it for prefix decodes.
    if (cfg.backend == Backend::Columnar && cfg.maxLod < 0 &&
        pmsf::tryRunRetriDecompressOverride(retriangulationProtocol(), cfg.input,
                                            cfg.output)) {
        return;
    }

    // P18.2 PHASE59 物品 3：config 侧的分区守卫**已删**。
    //
    // 它原先在 `segments > 1` 时响亮拒绝（分区 blob 布局当时无解码端镜像，
    // 按默认布局读会越界）。镜像是**按产物属性**分派的
    // （`blob_layout == 32` → `ColumnarDeserializeOperator::deserializeBaseMesh`
    // 里的分段分派），因此解压侧传不传 `segments` 都走同一条路；
    // 产物侧的孪生守卫（同批）也已删除。
    DecompressOptions options = makeDecompressOptions(cfg.backend, cfg.input, "");
    if (cfg.maxLod >= 0) {
        if (auto deserializer = options.getDeserializeOperator()) {
            deserializer->setQueryMaxLod(cfg.maxLod);
        }
    }
    auto mesh = pmsf::runDecompress(options, afterBaseMeshHook());
    if (const auto post = postDecompressHook()) {
        post(mesh);
    }
    if (!cfg.output.empty() && mesh) {
        mesh->dumpto_oldtype(cfg.output);
    }
}

const std::string& ISimplificationAlgorithm::registryKey() const {
    static thread_local std::string key;
    key = info().name;
    return key;
}

// ===========================================================================
// 按名分发
// ===========================================================================

namespace {

std::shared_ptr<ISimplificationAlgorithm> requireAlgorithm(const PMSFConfig& cfg) {
    auto& reg = SimplificationAlgorithmRegistry::instance();
    auto algo = reg.create(cfg.algorithmName);
    if (!algo) {
        std::string available;
        for (const auto& n : reg.names()) {
            available += (available.empty() ? "" : ", ") + n;
        }
        throw std::runtime_error("[SimplificationAlgorithmRegistry] 算法 '" + cfg.algorithmName +
                                 "' 未注册。已注册: " + available);
    }
    algo->configure(cfg.algorithmOptions);
    return algo;
}

}  // namespace

namespace {

/// @brief Phase S0（R13）：把 config 的 `storage_scheme` 激活为当前存储方案。
///
/// 压缩端：名字来自 config —— 未指定即原始方案（行为逐字节不变）。
/// 解压端：这里设的是**期望值**；真正权威的是产物自描述，反序列化器读出后会覆盖
///         （见 `activateStorageSchemeFromArtifact`）。两者不一致时以产物为准。
///
/// 名字未注册一律**抛错**，不静默回落 —— 用户明确要求了方案却拿到原始方案，
/// 是很难查的静默错解。
void applyStorageSchemeFromConfig(const pmsf::PMSFConfig& cfg) {
    if (cfg.storageScheme.empty()) {
        pmsf::setActiveStorageScheme(nullptr);
        return;
    }
    pmsf::StorageSchemePtr scheme = pmsf::createStorageScheme(cfg.storageScheme);
    if (!scheme) {
        std::string known;
        for (const auto& n : pmsf::registeredStorageSchemeNames()) {
            if (!known.empty()) known += ", ";
            known += n;
        }
        throw std::runtime_error("存储方案 \"" + cfg.storageScheme + "\" 未注册" +
                                 (known.empty() ? "（当前没有任何方案注册）"
                                                : "（已注册：" + known + "）"));
    }
    pmsf::setActiveStorageScheme(std::move(scheme));
}

}  // namespace

void dispatchCompressByName(const PMSFConfig& cfg) {
    applyStorageSchemeFromConfig(cfg);
    if (SimplificationAlgorithmRegistry::instance().has(cfg.algorithmName)) {
        // P18.1a 防呆：config 显式关量化，而算法声明「要求量化」⇒ 在这里就报。
        // 不这么做的代价是实测过的：要跑进 `*Tag` 采集器的 exportRC 才抛，
        // MMSE 白跑约 1.0 s、QEM 约 1.1 s 才失败，且错误信息只有协议名，
        // 看不出是 config 的 quantization 写错了。
        if (cfg.quantizationSet && !cfg.quantization) {
            auto alg = requireAlgorithm(cfg);
            if (alg->requiresQuantization()) {
                throw std::runtime_error(
                    "[config] 算法 '" + cfg.algorithmName +
                    "' 要求 quantization=true（其残差流按整数位平面设计，"
                    "没有未量化/F32 的码流布局）；但 config 显式写了 "
                    "\"quantization\": false。请改回 true，或改用支持 F32 档的"
                    "算法（如 Default）。");
            }
        }
        requireAlgorithm(cfg)->compress(cfg);
        return;
    }
    // Phase 4：过渡桥已删除 —— 全部算法名（含 default 与 valence 变体）都注册为插件，
    // 查不到即未注册。原先的 `isLegacyPreset()` 回落到 preset 路径，随过渡桥一并消失。
    (void)requireAlgorithm(cfg);  // 抛「未注册」并列出已注册算法
}

void dispatchDecompressByName(const PMSFConfig& cfg) {
    applyStorageSchemeFromConfig(cfg);
    if (SimplificationAlgorithmRegistry::instance().has(cfg.algorithmName)) {
        requireAlgorithm(cfg)->decompress(cfg);
        return;
    }
    // Phase 4：过渡桥已删除（同 compress 侧）。
    (void)requireAlgorithm(cfg);
}

}  // namespace pmsf
