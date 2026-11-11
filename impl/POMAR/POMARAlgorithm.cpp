/*
 * @file POMARAlgorithm.cpp
 * @brief POMAR (Maglo et al. 2013, "POMAR: Compression of Progressive Oriented
 *        Meshes Accessible Randomly") 接入 ISimplificationAlgorithm 契约。
 *
 * 论文 Tbl. tab:hook_loc 里 POMAR 重写的是 H2（56 行）+ H3（84 行）+ 后置分区布局。
 *
 * ## 为什么这个插件覆写了 compress()/decompress()
 *
 * POMAR 是设计文档 §2.1.3 的 **L2 级**案例：它有自己的多 LOD 全局布局
 * （`POMARColumnarSerializeOperator` + `PomarColumnarSymbolCollectOperator`），
 * 而且 collector 需要在消除循环结束后做一次 `computeLODPartition()`，
 * 再由**单一种子**驱动整棵树的重建 —— 这条流水线与框架的
 * `CompressPipelineConfig` 形状不同。设计文档把「POMAR V2 / VQMMC / segment /
 * hierarchy 四条专用路径迁入统一管线」排在 **Phase 4**。
 *
 * 因此本轮的接入形态是：把原本散在 `impl/common/PMSFAlgorithmMethods.cpp` 里的
 * POMAR 主循环**搬进插件自己的 cpp**，由契约的 `compress()/decompress()` 承载。
 * 收益是实打实的：算法代码回到 `impl/POMAR/`，解压侧不再需要任何入口文件，
 * `src/` 与 `impl/common/` 都少一块算法赘肉；Phase 4 再把这条循环收敛到
 * `runCompress` 的形状。
 *
 *   - H1 removability = BasicCollapseIsRemovableOperator（论文的边坍缩可消除性）
 *   - H2 entry customizer = pomar_impl::pomarEntryCustomizer（论文的优先级规则）
 *   - 消除算子       = PomarEliminateOperator
 *   - H7/H3 分区     = collector 的 computeLODPartition()（后置分区，论文 §4）
 *   - L2 布局        = POMARColumnarSerializeOperator
 *
 * 字节核实（2026-09-23 更新）：原先写的 `verify_algo_migration.sh pomar`
 * **已退役**——该脚本比的是「插件路径 vs legacy 过渡桥」，而 legacy 桥在
 * Phase 4 已被删除（脚本自己的头注释就写着「Phase 4 删除 legacy 桥后，本脚本
 * 随之退役」）。实测：它对**未经改动**的 qem 同样报 `legacy=0 B`，即脚本已死，
 * 不是 POMAR 的回归。
 *
 * 当前有效的核实方式（P9 合流后实跑，见 LOC_REFACTOR_PLAN §61）：
 *   scripts/tests/bench_columnar.sh   → pomar 107319 / 16045 / 23775
 *   scripts/tests/bench_binary.sh     → 同一组数字（合流的定义）
 *   scripts/tests/validate_regression.sh → 60/60 byte-identical
 *   scripts/tests/test_determinism.sh → pass=36 fail=0
 */
#include "algorithm/SimplificationAlgorithmRegistry.h"

#include "POMAR/POMARCustomizer.h"
#include "POMAR/compress/POMARColumnarSerializeOperator.h"

#include "POMAR/compress/PomarColumnarSymbolCollectOperator.h"
#include "POMAR/elimination/PomarEliminateOperator.h"
#include "operator/BasicCollapseIsRemovableOperator.h"
#include "POMAR/decompress/POMARColumnarDeserializeOperator.h"
#include "POMAR/decompress/PomarColumnarSymbolReadOperator.h"
#include "POMAR/reconstruct/PomarReconstructOperatorV2.h"
#include "common/HalfedgeErrorEntry.h"
#include "core/CompressPipeline.h"
#include "core/MeshRuntime.h"
#include "core/DecompressPipeline.h"
#include "core/DecompressOperatorRegistry.h"
#include "header/PMSFHeader.h"
#include "options/CompressOptions.h"
#include "options/DecompressOptions.h"
#include "options/SelectOptions.h"
#include "selection/ErrorSource.h"
#include "selection/PriorityErrorSelector.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

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

// Phase 1：这两段原先在 impl/common/PMSFAlgorithmMethods.cpp 的匿名 namespace 里；
// 搬进 POMAR 自己的插件后，框架侧不再持有 POMAR 专有逻辑。
void applyPomarClusterEnv(const std::shared_ptr<PomarColumnarSymbolCollectOperator>& collector) {
    if (const char* env = std::getenv("PMSF_POMAR_TARGET_CLUSTERS")) {
        const unsigned value = static_cast<unsigned>(std::strtoul(env, nullptr, 10));
        if (value > 0) {
            collector->setTargetClusterCount(value);
            return;
        }
    }
    if (const char* env = std::getenv("PMSF_POMAR_RANDOM_ACCESS_RATIO")) {
        const float value = std::strtof(env, nullptr);
        if (value > 0.0f && value <= 1.0f) {
            collector->setRandomAccessRatio(value);
        }
    }
}

void applyPomarQuantBitsEnv(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!mesh) return;
    const char* env = std::getenv("PMSF_POMAR_QUANT_BITS");
    if (!env || !*env) return;
    const int value = std::atoi(env);
    if (value < 2 || value > 16) {
        std::fprintf(stderr,
                     "[POMAR] ignore invalid PMSF_POMAR_QUANT_BITS=%d "
                     "(supported range is 2..16)\n",
                     value);
        return;
    }
    mesh->i_nbQuantBits = static_cast<unsigned>(value);
    float maxRange = 0.0f;
    for (unsigned axis = 0; axis < 3; ++axis) {
        maxRange = std::max(maxRange, mesh->bboxMax[axis] - mesh->bboxMin[axis]);
    }
    mesh->f_quantStep = maxRange / static_cast<float>(1u << mesh->i_nbQuantBits);
    std::fprintf(stderr, "[POMAR] quantBits=%u quantStep=%g\n", mesh->i_nbQuantBits,
                 mesh->f_quantStep);
}

struct PomarRepairStats {
    int initialDegenerateFaces = 0;
    int repairedDegenerateFaces = 0;
    int remainingDegenerateFaces = 0;
    double maxVertexDisplacement = 0.0;
};

double pomarTriangleArea2(const MCGAL::Point& a,
                          const MCGAL::Point& b,
                          const MCGAL::Point& c) {
    const double ux = static_cast<double>(b.x()) - a.x();
    const double uy = static_cast<double>(b.y()) - a.y();
    const double uz = static_cast<double>(b.z()) - a.z();
    const double vx = static_cast<double>(c.x()) - a.x();
    const double vy = static_cast<double>(c.y()) - a.y();
    const double vz = static_cast<double>(c.z()) - a.z();
    const double cx = uy * vz - uz * vy;
    const double cy = uz * vx - ux * vz;
    const double cz = ux * vy - uy * vx;
    return cx * cx + cy * cy + cz * cz;
}

bool pomarDegenerateFace(MCGAL::Facet* face) {
    if (!face || face->isRemoved()) return false;
    const std::vector<MCGAL::Vertex*> vertices = face->getVertices();
    return vertices.size() == 3 && vertices[0] && vertices[1] && vertices[2] &&
           pomarTriangleArea2(vertices[0]->point(), vertices[1]->point(),
                              vertices[2]->point()) <= 1e-30;
}

std::array<double, 3> pomarPerpendicularDirection(const MCGAL::Point& a,
                                                  const MCGAL::Point& b) {
    const std::array<double, 3> edge{
        static_cast<double>(b.x()) - a.x(),
        static_cast<double>(b.y()) - a.y(),
        static_cast<double>(b.z()) - a.z()};
    unsigned axis = 0;
    if (std::fabs(edge[1]) < std::fabs(edge[axis])) axis = 1;
    if (std::fabs(edge[2]) < std::fabs(edge[axis])) axis = 2;
    std::array<double, 3> basis{0.0, 0.0, 0.0};
    basis[axis] = 1.0;
    std::array<double, 3> direction{
        edge[1] * basis[2] - edge[2] * basis[1],
        edge[2] * basis[0] - edge[0] * basis[2],
        edge[0] * basis[1] - edge[1] * basis[0]};
    const double length = std::sqrt(direction[0] * direction[0] +
                                     direction[1] * direction[1] +
                                     direction[2] * direction[2]);
    if (length <= 1e-30) return {1.0, 0.0, 0.0};
    for (double& component : direction) component /= length;
    return direction;
}

bool pomarCandidatePreservesIncidentFaces(
    MCGAL::Vertex* moved,
    const MCGAL::Point& candidate,
    MCGAL::Facet* target,
    const std::unordered_map<MCGAL::Vertex*, std::vector<MCGAL::Facet*>>& incident,
    const std::unordered_set<MCGAL::Facet*>& initialDegenerate) {
    const auto it = incident.find(moved);
    if (it == incident.end()) return false;
    for (MCGAL::Facet* face : it->second) {
        if (!face || face->isRemoved()) continue;
        const std::vector<MCGAL::Vertex*> vertices = face->getVertices();
        if (vertices.size() != 3 || !vertices[0] || !vertices[1] || !vertices[2]) {
            continue;
        }
        MCGAL::Point points[3] = {
            vertices[0]->point(), vertices[1]->point(), vertices[2]->point()};
        for (unsigned i = 0; i < 3; ++i) {
            if (vertices[i] == moved) points[i] = candidate;
        }
        const bool degenerate = pomarTriangleArea2(points[0], points[1], points[2]) <= 1e-30;
        if (face == target ||
            (degenerate && initialDegenerate.find(face) == initialDegenerate.end())) {
            if (degenerate) return false;
        }
    }
    return true;
}

bool pomarTryRepairFace(
    MCGAL::Facet* face,
    const std::unordered_map<MCGAL::Vertex*, std::vector<MCGAL::Facet*>>& incident,
    const std::unordered_set<MCGAL::Facet*>& initialDegenerate,
    float quantStep,
    std::unordered_map<MCGAL::Vertex*, MCGAL::Point>& originalPoints,
    double& maxDisplacement) {
    const std::vector<MCGAL::Vertex*> vertices = face->getVertices();
    if (vertices.size() != 3 || !vertices[0] || !vertices[1] || !vertices[2]) return false;

    const std::array<std::array<double, 3>, 6> axisDirections{{
        {1.0, 0.0, 0.0}, {-1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0}, {0.0, -1.0, 0.0},
        {0.0, 0.0, 1.0}, {0.0, 0.0, -1.0}}};
    // D23：**先用尽可能小的位移**去破退化，再逐步放大。
    // 退化判据是 area^2 <= 1e-30（近乎「严格共线」），故**亚量化步千分之一**
    // 的位移就足以让面积跨过阈值；用大步长只会白白把顶点推离碰撞判据。
    // 实测（armadillo，quantStep≈0.037）：从 0.125×step(=4.6e-3) 起跳会让
    // 解码端坐标重映射的错误配对出 4 条边/6 个面；改从 1e-3×step(≈3.7e-5)
    // 起跳既破退化又保住重映射（详见 DEFECT_TRACKING D23）。
    const std::array<double, 6> scales{{
        std::max(static_cast<double>(quantStep) * 0.001, 1e-7),
        std::max(static_cast<double>(quantStep) * 0.01, 1e-7),
        std::max(static_cast<double>(quantStep) * 0.125, 1e-7),
        std::max(static_cast<double>(quantStep) * 0.25, 2e-7),
        std::max(static_cast<double>(quantStep) * 0.5, 4e-7),
        std::max(static_cast<double>(quantStep), 8e-7)}};

    for (unsigned movedIndex = 0; movedIndex < 3; ++movedIndex) {
        const unsigned otherA = (movedIndex + 1) % 3;
        const unsigned otherB = (movedIndex + 2) % 3;
        const std::array<double, 3> normal =
            pomarPerpendicularDirection(vertices[otherA]->point(), vertices[otherB]->point());
        std::vector<std::array<double, 3>> directions;
        directions.reserve(8);
        directions.push_back(normal);
        directions.push_back({-normal[0], -normal[1], -normal[2]});
        for (const auto& direction : axisDirections) directions.push_back(direction);

        MCGAL::Vertex* moved = vertices[movedIndex];
        const MCGAL::Point current = moved->point();
        for (double scale : scales) {
            for (const auto& direction : directions) {
                const MCGAL::Point candidate(
                    current.x() + static_cast<float>(scale * direction[0]),
                    current.y() + static_cast<float>(scale * direction[1]),
                    current.z() + static_cast<float>(scale * direction[2]));
                if (!pomarCandidatePreservesIncidentFaces(
                        moved, candidate, face, incident, initialDegenerate)) {
                    continue;
                }
                const auto original = originalPoints.emplace(moved, current);
                const MCGAL::Point& first = original.first->second;
                const double dx = static_cast<double>(candidate.x()) - first.x();
                const double dy = static_cast<double>(candidate.y()) - first.y();
                const double dz = static_cast<double>(candidate.z()) - first.z();
                maxDisplacement = std::max(
                    maxDisplacement, std::sqrt(dx * dx + dy * dy + dz * dz));
                moved->setPoint(candidate);
                return true;
            }
        }
    }
    return false;
}

int repairDegenerateTriangleGeometry(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                     PomarRepairStats* stats = nullptr) {
    if (!mesh) return 0;

    std::vector<MCGAL::Facet*> faces;
    std::unordered_map<MCGAL::Vertex*, std::vector<MCGAL::Facet*>> incident;
    std::unordered_set<MCGAL::Facet*> initialDegenerate;
    faces.reserve(mesh->size_of_facets());
    incident.reserve(mesh->size_of_vertices() * 2 + 1);
    for (MCGAL::Facet* face : mesh->faces()) {
        if (!face || face->isRemoved()) continue;
        faces.push_back(face);
        for (MCGAL::Vertex* vertex : face->getVertices()) {
            if (vertex) incident[vertex].push_back(face);
        }
        if (pomarDegenerateFace(face)) initialDegenerate.insert(face);
    }

    std::unordered_map<MCGAL::Vertex*, MCGAL::Point> originalPoints;
    std::unordered_set<MCGAL::Facet*> remaining = initialDegenerate;
    double maxDisplacement = 0.0;
    for (unsigned pass = 0; pass < 64 && !remaining.empty(); ++pass) {
        bool progress = false;
        for (MCGAL::Facet* face : faces) {
            if (remaining.find(face) == remaining.end()) continue;
            if (!pomarDegenerateFace(face)) {
                remaining.erase(face);
                continue;
            }
            if (!pomarTryRepairFace(face, incident, initialDegenerate,
                                     mesh->f_quantStep, originalPoints,
                                     maxDisplacement)) {
                continue;
            }
            progress = true;
            for (MCGAL::Vertex* vertex : face->getVertices()) {
                if (!vertex) continue;
                for (MCGAL::Facet* incidentFace : incident[vertex]) {
                    if (pomarDegenerateFace(incidentFace)) remaining.insert(incidentFace);
                    else remaining.erase(incidentFace);
                }
            }
        }
        if (!progress) break;
    }

    int remainingCount = 0;
    for (MCGAL::Facet* face : initialDegenerate) {
        if (pomarDegenerateFace(face)) ++remainingCount;
    }
    const int repaired = static_cast<int>(initialDegenerate.size()) - remainingCount;
    if (stats) {
        stats->initialDegenerateFaces = static_cast<int>(initialDegenerate.size());
        stats->repairedDegenerateFaces = repaired;
        stats->remainingDegenerateFaces = remainingCount;
        stats->maxVertexDisplacement = maxDisplacement;
    }
    return repaired;
}

class POMARAlgorithm final : public ISimplificationAlgorithm {
  public:
    AlgorithmInfo info() const override {
        return AlgorithmInfo{
            .name = registeredName("pomar", "PMSF_PROBE_POMAR", "pomar_probe"),
            .version = 1,
            .primitive = PrimitiveKind::Edge,
            .selection = SelectionKind::GeometryDriven,
        };
    }

    /// 论文的边坍缩可消除性；优先级由 H2 的 entry customizer 给出。
    IsRemovableChain removability() const override {
        IsRemovableChain chain;
        chain.push_back(std::make_shared<BasicCollapseIsRemovableOperator>());
        return chain;
    }

    EliminateOperatorPtr eliminate() const override {
        return std::make_shared<PomarEliminateOperator>();
    }

    /// 论文的优先级规则以 entry customizer 的形式参与选择器构造。
    /// 契约目前把它建模成「选择器前置过滤」，Phase 4 会并入 H2 的度量接口。
    static std::shared_ptr<SelectOptions> buildSelectOptions() {
        return SelectOptions::Builder()
            .withErrorSource(ErrorSource::Halfedge)
            .withEntryCustomizer(pomar_impl::pomarEntryCustomizer)
            .withIsRemovableOperator(std::make_shared<BasicCollapseIsRemovableOperator>())
            .build();
    }

    /// L2：POMAR 自带的多 LOD 全局布局 + 单种子驱动的整树重建。
    void compress(const PMSFConfig& cfg) override {
        // 本插件自己加载网格，因此必须自己确保 MCGAL ContextPool 已初始化
        // （框架的 runCompress 会做，这里绕过了它）。按输入 OFF 头预读规模
        // 注册（标定乘数收敛在 MeshRuntime），避免巨型固定池的初始化常数。
        ensureMeshRuntimeInitializedForInputFile(cfg.input);

        PMSFHeader header = PMSFHeader::Builder()
                                .setRound(cfg.effectiveRounds())
                                .setEnableCompress(cfg.compression)
                                .setEnableQuantization(cfg.quantization)
                                .setEnableSegmentation(cfg.segmentation)
                                .setEnableAdjustMent(cfg.adjustment)
                                .build();

        auto mesh = std::make_shared<MCGAL::Mesh>(MCGAL::Mesh(cfg.input));
        applyPomarQuantBitsEnv(mesh);

        auto select = std::make_shared<PriorityErrorSelector>(*buildSelectOptions());
        auto eliminate = std::make_shared<PomarEliminateOperator>();

        // ---- 后端合流（2026-09-23，P9 §61；D2 许可：产物字节可变、基线重冻）----
        //
        // POMAR 曾有两套**完全独立**的落盘实现：
        //   binary    : PomarSymbolCollectOperator + PomarSerializeOperator
        //               （自包含整体编码器，遍历 contextPool.operations 自己写
        //                header + base mesh + 各 LOD 符号）
        //   columnar  : PomarColumnarSymbolCollectOperator + POMARColumnarSerializeOperator
        // 两者是同一算法的两次实现，信息量相同却产出不同字节（bunny 101417 vs
        // 107319），维护面翻倍。现**只保留 columnar 一条**：无论 cfg.backend 取值
        // 为何，POMAR 都走列式。规范产物形态 `<stem>_compact.bin`（10/10 算法，
        // verify_artifact_layout.sh 断言）由此成为 POMAR 的唯一形态。
        auto collector = std::make_shared<PomarColumnarSymbolCollectOperator>();
        applyPomarClusterEnv(collector);
        auto serializer = std::make_shared<POMARColumnarSerializeOperator>(
            collector, /*enablePrediction=*/cfg.prediction,
            /*enableQuantization=*/cfg.quantization, /*enableCompress=*/cfg.compression);
        serializer->setPaperStream(cfg.backend == Backend::Paper);

        MCGAL::Halfedge* seed = nullptr;
        MCGAL::Point p;
        if (select->options_.getRegisterFunc()) {
            select->options_.getRegisterFunc()(mesh);
        }
        select->init(mesh);
        eliminate->init(mesh, header);
        collector->init(mesh, {}, header);
        serializer->init();

        for (int i = 0; i < cfg.effectiveRounds(); i++) {
            MCGAL::Halfedge* candidate = nullptr;
            MCGAL::contextPool.increaseLod();
            HalfedgeErrorEntry entry(candidate, p, 0, 0);
            while (select->select(entry)) {
                candidate = entry.edge;
                (void)eliminate->eliminate(entry);
            }
            select->reset();
        }

        for (MCGAL::Facet* f : mesh->faces()) {
            if (f->isRemoved()) continue;
            for (MCGAL::Halfedge* h : *f) { seed = h; break; }
            if (seed) break;
        }

        serializer->setHeader(std::make_shared<PMSFHeader>(header));
        serializer->serializeHeader(header);
        collector->computeLODPartition();
        serializer->serializeBaseMeshWithSeed(mesh, {seed});
        collector->collect(seed);
        serializer->serialize(cfg.output);
    }

    /// 解压选项：交给框架跑通用解码循环时，解压器必须是 POMAR 自己的那个。
    ///
    /// 默认实现（算法注册表里的通用三元组）解不出 POMAR 的 blob 布局；随机访问
    /// 查询 (runQuery) 走的正是「框架循环 + 本函数给出的解压器」这条路。
    DecompressOptions makeDecompressOptions(Backend backend, const std::string& input,
                                            const std::string& output) const override {
        auto reader = std::make_shared<PomarColumnarSymbolReadOperator>();
        auto reconstructor = std::make_shared<PomarReconstructOperatorV2>();
        auto deserializer =
            std::make_shared<POMARColumnarDeserializeOperator>(reader, reconstructor);
        deserializer->setPaperStream(backend == Backend::Paper);
        return DecompressOptions::Builder()
            .setDeserializeOperator(deserializer)
            .setPath(input)
            .setOutputPath(output)
            .build();
    }

    /// 解压侧：POMAR 自己的 4 表 deserializer + 单轮循环。
    void decompress(const PMSFConfig& cfg) override {
        // 池按产物记录的原始规模注册：init 只读文件（不碰池），读完再注册，
        // 不再靠脚本统一塞巨型池。POMARColumnarDeserializeOperator::init 会
        // 从 compact header 的 origTotal 字段取回压缩前原始 V/F（旧产物为 0，
        // 回落内置默认）。
        auto deserializer = makeDeserializeOperatorForAlgorithm("pomar", Backend::Columnar);
        if (auto* paper = dynamic_cast<POMARColumnarDeserializeOperator*>(deserializer.get())) {
            paper->setPaperStream(cfg.backend == Backend::Paper);
        }
        deserializer->init(cfg.input);
        int origV = 0, origF = 0;
        deserializer->originalMeshCounts(origV, origF);
        if (origV > 0 && origF > 0) {
            ensureMeshRuntimeInitializedForMeshSize(origV, origF);
        } else {
            ensureMeshRuntimeInitialized();
        }

        PMSFHeader header;
        deserializer->deserializeHeader(header);
        auto mesh = deserializer->deserializeBaseMesh();

        int round = header.getRound();
        int _ri = 0;
        while (round--) {
            if (!envOn("PMSF_POMAR_SKIP_OUTER_RESET")) {
                // 用框架版的**完整**复位，而不是 MCGAL::Mesh::resetState()。
                //
                // 后者只覆盖「活着的面」触及的元素；已移除的面所包裹的元素不会
                // 复位，其 addedFlag_/splittableFlag_ 会跨轮残留。主解码循环
                // （CompressPipeline.cpp）早已改成 resetRoundElementState，
                // 但这里是一条**手写的**轮循环，漏了这一处 —— 实测后果是
                // pomar/LetterA 列式解码丢 1 顶点 2 面、并产出 701 个退化面
                // （见 DEFECT_TRACKING D3）。binary 路径不走这个循环，所以没事。
                pmsf::resetRoundElementState(mesh);
            }
            deserializer->deserializeOneRound();
        }

        // D23：**这一段恢复为无条件调用**（d0514cc 之前即如此）。
        // d0514cc 同时做了两件事，叠加后使修复彻底失效：
        //   ① 把这段从无条件调用改成 `envOn("PMSF_REPAIR_DEGENERATE")` 门控；
        //   ② 把所有 configs/algorithms/*.json 的该键从 "1" 翻成 "0"，
        //      且各 runner 也 `export PMSF_REPAIR_DEGENERATE=0` ——
        //      而 `tools/PMSFDecompress.cpp:30` 会把 config 的 extra `setenv(...,1)`
        //      覆盖进来，故门控必然看到 "0"。
        // 结果：解码网格残留 32 个退化面（armadillo）⇒ `test_valid=false`。
        // POMAR 的退化面来自量化后**坐标重合**的顶点，是这条解码路径**固有**的，
        // 故保持「无条件」语义 —— 与冻结基线时的行为逐位一致。
        // （用 `=0` 想关掉它的调用方请注意：POMAR 这里不读该开关；通用路径
        //   `CompressPipeline.cpp` 的 `runDecompress` 仍然读。）
        {
            PomarRepairStats repairStats;
            repairDegenerateTriangleGeometry(mesh, &repairStats);
            if (repairStats.initialDegenerateFaces > 0) {
                std::fprintf(stderr,
                             "[POMAR Columnar Decompress] repaired_degenerate_faces=%d "
                             "remaining_degenerate_faces=%d max_displacement=%g\n",
                             repairStats.repairedDegenerateFaces,
                             repairStats.remainingDegenerateFaces,
                             repairStats.maxVertexDisplacement);
            }
        }
        if (!cfg.output.empty()) {
            deserializer->exportMesh()->dumpto_oldtype(cfg.output);
        }
    }

    /// 解压组件在同一 cpp 内注册 —— 算法不写任何解压入口。
    static void registerDecompressComponents() {
        NamedDecompressOperatorRegistrar("pomar", Backend::Columnar,
                                         []() -> DecompressOperatorTriple {
            auto reader = std::make_shared<PomarColumnarSymbolReadOperator>();
            auto reconstructor = std::make_shared<PomarReconstructOperatorV2>();
            auto deserializer =
                std::make_shared<POMARColumnarDeserializeOperator>(reader, reconstructor);
            return {deserializer, reader, reconstructor};
        });
    }
};

const bool g_pomarDecompressRegistered = [] {
    POMARAlgorithm::registerDecompressComponents();
    return true;
}();

}  // namespace
}  // namespace pmsf

PMSF_REGISTER_SIMPLIFICATION_ALGORITHM(pmsf::POMARAlgorithm);
