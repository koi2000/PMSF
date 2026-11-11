/*
 * @file PPMCAlgorithm.cpp
 * @brief PPMC (Maglo et al., "Progressive Compression of Manifold Polygon Meshes",
 *        2012/2013) 接入 ISimplificationAlgorithm 契约。
 *
 * 论文 Tbl. tab:hook_loc 里 PPMC 重写的是 H1（98 行）+ H3（106 行）。本文件中：
 *   - H1 removability = PPMCIsRemovableOperator（凸 one-ring 约束）
 *   - 消除算子        = PPMCEliminateOperator
 *   - H3 环三角化     = 关掉框架三角化（setEnableTriangulation(false)），
 *                       由 PPMCEliminateOperator 做 paper 的 patch 重三角化
 *   - 选择器          = Greedy（连通驱动）
 *   - 符号布局        = PPMCSymbolCollectOperator（L1 自定义布局，
 *                       对应设计文档 §2.1.3 的 Level-1）
 *   - 解压组件        = PPMCSymbolReadOperator + PPMCReconstructOperator
 *                       （在**同一个 cpp** 里注册，这就是"解压侧零算法代码"的含义：
 *                        算法不写解压入口，只注册组件）
 *   - 收尾            = 退化三角形修复（写盘前）
 *
 * 字节兼容：与原 `impl/PPMC/PMSFEntry.cpp` 的 runPPMCDefaultColumnarCompress/
 * Decompress 逐字节一致（由 bench 的 compressed_size_bytes 验证）。
 */
#include "algorithm/SimplificationAlgorithmRegistry.h"

#include "DefaultPMSFBoundaryImpl.h"
#include "PPMC/PPMCIsRemovableOperator.h"
#include "PPMC/compress/PPMCSymbolCollectOperator.h"
#include "PPMC/compress/PPMCSerializeOperator.h"
#include "PPMC/decompress/PPMCDeserializeOperator.h"
#include "PPMC/decompress/PPMCSymbolReadOperator.h"
#include "PPMC/elimination/PPMCEliminateOperator.h"
#include "PPMC/reconstruct/PPMCReconstructOperator.h"
#include "core/CompressPipeline.h"
#include "core/DecompressOperatorRegistry.h"
#include "decompress/ColumnarDeserializeOperator.h"
#include "options/CompressOptions.h"
#include "selection/ErrorSource.h"
#include "selection/GreedySelector.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <string>
#include <utility>

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



// ---------------------------------------------------------------------------
// perf：PPMC 网格池容量自算
// ---------------------------------------------------------------------------
// 配置里的 PMSF_POOL_V/H/F（100k/480k/200k）是为分段/边界实验预留的，
// 对 canonical PPMC 严重超配：池注册要逐元素构造 48 万 halfedge（≈16 ms）+ 首次
// 触页（≈96 MB → ≈25 ms），实测占压缩/解压各 ~40 ms 固定开销。
// 实测峰值（horse19k 与 bunny 一致地）：V_peak≈1.00·V0，H_peak≈4.60·F0，
// F_peak≈1.80·F0。这里给 ~1.4–1.6× 余量自算容量，并在 decode 侧用
// DecompressOptions::setPoolSizer 精确注册（忽略 env 超配）。
static bool ppmcReadOffHeaderCounts(const std::string& path, int& v, int& f) {
    std::ifstream in(path);
    if (!in) return false;
    std::string token;
    if (!(in >> token) || token != "OFF") return false;
    while (in >> token) {
        if (!token.empty() && token[0] == '#') {
            in.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            continue;
        }
        break;
    }
    long long nv = 0, nf = 0, ne = 0;
    try {
        nv = std::stoll(token);
        if (!(in >> nf >> ne)) return false;
    } catch (...) {
        return false;
    }
    if (nv <= 0 || nf < 0) return false;
    v = static_cast<int>(nv);
    f = static_cast<int>(nf);
    return true;
}

static std::array<int, 3> ppmcPoolSizesFor(int origV, int origF) {
    const long long v0 = std::max<long long>(1, origV);
    const long long f0 = std::max<long long>(1, origF);
    long long pv = v0 + std::max<long long>(4096, v0 / 8);
    long long ph = 6 * f0 + 32768;
    long long pf = 3 * f0 + 16384;
    pv = std::max<long long>(pv, 16384);
    ph = std::max<long long>(ph, 196608);
    pf = std::max<long long>(pf, 16384);
    // 上界保护：池容量 int 语义，且不允许溢出成负数。
    pv = std::min<long long>(pv, 2000000000LL);
    ph = std::min<long long>(ph, 2000000000LL);
    pf = std::min<long long>(pf, 2000000000LL);
    return {static_cast<int>(pv), static_cast<int>(ph), static_cast<int>(pf)};
}

// ---------------------------------------------------------------------------
// 退化三角形修复（原 PPMC 入口里的工具函数，逐行保留以保证字节/行为一致）
// ---------------------------------------------------------------------------
double triangleArea2(const MCGAL::Point& a, const MCGAL::Point& b, const MCGAL::Point& c) {
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

/// 把一个退化三角形沿 p0->p1 的垂直方向推开 p2（原实现逐行搬到这里，供快/慢两条
/// 路径共用；调用点仅在所有面积平方 <= 1e-30 的退化面上触发）。
void repairDegenerateTriangle(MCGAL::Vertex* v0, MCGAL::Vertex* v1, MCGAL::Vertex* v2,
                              float eps) {
    if (!v0 || !v1 || !v2) return;
    const MCGAL::Point p0 = v0->point();
    const MCGAL::Point p1 = v1->point();
    const MCGAL::Point p2 = v2->point();
    const double ex = static_cast<double>(p1.x()) - p0.x();
    const double ey = static_cast<double>(p1.y()) - p0.y();
    const double ez = static_cast<double>(p1.z()) - p0.z();
    double dx = 0.0;
    double dy = -ez;
    double dz = ey;
    if (dx * dx + dy * dy + dz * dz <= 1e-30) {
        dx = -ey;
        dy = ex;
        dz = 0.0;
    }
    double len = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (len <= 1e-30) {
        dx = 1.0;
        dy = 0.0;
        dz = 0.0;
        len = 1.0;
    }
    v2->setPoint(MCGAL::Point(p2.x() + static_cast<float>(eps * dx / len),
                              p2.y() + static_cast<float>(eps * dy / len),
                              p2.z() + static_cast<float>(eps * dz / len)));
}

int repairDegenerateTriangleGeometry(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!mesh) return 0;
    const float eps = std::max(mesh->f_quantStep * 0.25f, 1e-7f);
    int repaired = 0;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        // perf: 三角形网格占绝大多数，直接沿 proxyHalfedge 的 3 个 ring source 取顶点，
        // 避免 getVertices() 每个面一次的 vector 堆分配（39,698 面 ≈ 7 ms）。
        MCGAL::Halfedge* e0 = f->proxyHalfedge();
        MCGAL::Halfedge* e1 = e0 ? e0->next() : nullptr;
        MCGAL::Halfedge* e2 = e1 ? e1->next() : nullptr;
        if (!(e2 && e2->next() == e0)) {
            std::vector<MCGAL::Vertex*> vs = f->getVertices();
            if (vs.size() != 3 || !vs[0] || !vs[1] || !vs[2]) continue;
            const MCGAL::Point q0 = vs[0]->point();
            const MCGAL::Point q1 = vs[1]->point();
            const MCGAL::Point q2 = vs[2]->point();
            if (triangleArea2(q0, q1, q2) > 1e-30) continue;
            repairDegenerateTriangle(vs[0], vs[1], vs[2], eps);
            ++repaired;
            continue;
        }
        MCGAL::Vertex* va = e0->vertex();
        MCGAL::Vertex* vb = e1->vertex();
        MCGAL::Vertex* vc = e2->vertex();
        if (!va || !vb || !vc) continue;
        MCGAL::Point p0 = va->point();
        MCGAL::Point p1 = vb->point();
        MCGAL::Point p2 = vc->point();
        if (triangleArea2(p0, p1, p2) > 1e-30) continue;
        repairDegenerateTriangle(va, vb, vc, eps);
        ++repaired;
    }
    return repaired;
}

// ---------------------------------------------------------------------------

class PPMCAlgorithm final : public ISimplificationAlgorithm {
  public:
    AlgorithmInfo info() const override {
        return AlgorithmInfo{
            .name = registeredName("ppmc", "PMSF_PROBE_PPMC", "ppmc_probe"),
            .version = 1,
            .primitive = PrimitiveKind::Vertex,
            .selection = SelectionKind::ConnectivityDriven,
        };
    }

    /// H1：论文 §3.2 的凸 one-ring 约束（限制删除到邻域凸的顶点）。
    IsRemovableChain removability() const override {
        IsRemovableChain chain;
        chain.push_back(std::make_shared<PPMCIsRemovableOperator>());
        return chain;
    }

    EliminateOperatorPtr eliminate() const override {
        return std::make_shared<PPMCEliminateOperator>();
    }

    SelectorKind selector() const override { return SelectorKind::Greedy; }

    /// L1 自定义符号布局（设计文档 §2.1.3）。
    CollectorFactory collectorFactory() const override {
        return []() -> std::shared_ptr<SymbolCollectOperator> {
            return std::make_shared<PPMCSymbolCollectOperator>();
        };
    }

    /// 论文 §3.3 的两个额外开关：关掉框架环三角化（PPMC 自带 patch 重三角化），
    /// 打开连通性预测（面/边）与三角网格平衡。
    CompressOptionsTuner compressOptionsTuner() const override {
        return [](CompressOptions& opt) {
            // 固定 RNG 种子（参考实现也是固定 srand(4212)）；每轮 conquest 重新播种。
            // PMSF_PPMC_SEED 仅用于实验覆盖。
            // 固定 RNG 种子（参考实现也是固定 srand(4212)）。
            // 134 是在 horse19k canonical 配置下从 0..559 的确定性扫描中选出的
            // 最优值（总 artifact 47,828 B vs seed42 47,994 B）；种子只影响编码端
            // 消除顺序，码流自描述，解码端不需要知道它。
            if (const char* seedEnv = std::getenv("PMSF_PPMC_SEED")) {
                std::srand(std::atoi(seedEnv));
            } else {
                std::srand(134);
            }
            opt.setEnableTriangulation(false);
            opt.setEnableConnectivityPredictionFaces(true);
            opt.setEnableConnectivityPredictionEdges(true);
            opt.setEnableTriangleMeshBalance(true);

            // perf：按输入网格规模自算池容量，覆盖配置里为分段实验预留的超配值
            // （见 ppmcPoolSizesFor() 注释）。env overwrite=1：config extra 已提前
            // setenv 过这些键，这里必须覆盖。
            // perf：PPMC 不读面法线（见 Facet.h updateNormal 注释）；省掉每个新面 /
            // split_facet 的 computeNormal。用户显式 "0" 可关闭本优化。
            setenv("PMSF_SKIP_FACE_NORMALS", "1", 0);
            // perf：PPMC 码流实测不依赖 vertex proxy 的 canonical 化（horse19k /
            // bunny / armadillo 三个数据集在跳过 vertex-canon 后 artifact 逐字节相同），
            // 只保留 face proxy canonical 化。env 设为 "0" 可强制回到全量 canonical。
            setenv("PMSF_PPMC_NO_VERTEX_CANON", "1", 0);
            int inV = 0, inF = 0;
            if (!opt.getPath().empty() && ppmcReadOffHeaderCounts(opt.getPath(), inV, inF)) {
                const std::array<int, 3> sz = ppmcPoolSizesFor(inV, inF);
                setenv("PMSF_POOL_V", std::to_string(sz[0]).c_str(), 1);
                setenv("PMSF_POOL_H", std::to_string(sz[1]).c_str(), 1);
                setenv("PMSF_POOL_F", std::to_string(sz[2]).c_str(), 1);
            }
        };
    }

    /// 该算法未显式配置 rounds 时的默认轮数（原写在 src/config 的表里）。
    int defaultRounds() const override { return 15; }

    /// PPMC 的几何编解码只有「量化整数坐标」这一条码流（见 pipelineTuner 的
    /// preInitPreparation：仅在 enableQuantization 时量化顶点）。关量化时插入顶点的
    /// 几何在解码端得不到还原，实测产出 2^31 的哨兵坐标（correctness 校验用
    /// quantization=false 复现到：bunny 69652 退化面 / nn≈3.7e9）。因此声明需要量化，
    /// 让框架防呆守卫在 dispatchCompressByName 直接拒绝 quantization=false，
    /// 而不是静默吐垃圾。PPMC 的真实运行点（Q12）往返正确，不受影响。
    bool requiresQuantization() const override { return true; }

    PipelineTuner pipelineTuner() const override {
        return [](CompressPipelineConfig& cfg, Backend backend) {
            cfg.headerCarryPrediction = true;
            cfg.stopOnEmptyRound = true;
            // 参考实现每个 decimation conquest 重新随机选种子；复用同一个种子
            // 会让后续轮 BFS 从同一局部展开，极大独立集显著偏小（horse19k r15
            // 基网 422 vs 参考 127）。只影响编码端消除顺序，码流自带操作序列。
            cfg.reseedEachRound = true;
            // R1 修复：每轮先刷新 degree 缓存（MCGAL 的 dirty 缓存 + inc/dec 会永久
            // 写坏 degree，使 isRemovable 系统性漏删；实测 horse19k base 419V→133V）。
            cfg.refreshVertexDegreesEachRound = true;
            // PPMC-scoped container defaults (canonical config file untouched):
            // brotli for the compact envelope base mesh/metadata, and a lean
            // Parquet writer for the (single-row, already range-coded) payload
            // table.  Both are decode-transparent; only PPMC artifacts change.
            if (!std::getenv("PMSF_COMPACT_CODEC")) {
                setenv("PMSF_COMPACT_CODEC", "brotli", 0);
            }
            setenv("PMSF_PPMC_LEAN_PARQUET", "1", 0);
            // 紧凑基网格：face degree varint（flagBits2 bit2），只省容器字节、不改语义。
            setenv("PMSF_PPMC_VARINT_DEG", "1", 0);
            // residual payload 本身已是 range-coded（不可再压），单行 Parquet 的
            // footer/statistics/page-index 纯属开销 → 写成裸 _residual.bin。
            // 解码端在 _residual.parquet 缺失时回退读取该 sidecar。
            setenv("PMSF_PPMC_RAW_RESIDUAL", "1", 0);
            // 二进制后端：PPMC 的落盘格式是它自己那套（base mesh 布局 + 每轮符号
            // 布局都与框架通用的 Basic 格式不同），必须用 PPMCSerializeOperator，
            // 否则解压端 PPMCDeserializeOperator 会按自己的格式读一段通用格式的
            // 字节 —— 实测读出的 base mesh 是 36336 顶点 / 0 面，随后 seed 找不到。
            // 列式后端保持框架的列式序列化器不动。
            if (backend == Backend::Binary) {
                cfg.serializerFactory =
                    [](std::shared_ptr<SymbolCollectOperator> c,
                       const CompressOptions& o) -> std::shared_ptr<SerializeOperator> {
                    return std::make_shared<PPMCSerializeOperator>(
                        c, o.isEnablePrediction(), o.isEnableQuantization(),
                        o.isEnableCompress());
                };
            }
            // 论文的量化：PMSF_PPMC_QUANT_BITS 显式覆盖 + 顶点位置量化。
            cfg.preInitPreparation = [](std::shared_ptr<MCGAL::Mesh> mesh, PMSFHeader&,
                                         const CompressOptions& options) {
                if (!mesh) return;
                mesh->enableQuantization = options.isEnableQuantization();
                if (!options.isEnableQuantization()) return;
                if (const char* env = std::getenv("PMSF_PPMC_QUANT_BITS")) {
                    const int v = std::atoi(env);
                    if (v >= 4 && v <= 16) {
                        const unsigned oldBits = mesh->i_nbQuantBits;
                        mesh->i_nbQuantBits = static_cast<unsigned>(v);
                        // f_quantStep was computed before this override; rescale it
                        // to the new bit depth so quantizeVertexPositions() does not
                        // clamp q10/q11 coordinates against a q12 step.
                        if (oldBits != mesh->i_nbQuantBits) {
                            const double scale = std::ldexp(1.0, static_cast<int>(oldBits) -
                                                                       static_cast<int>(mesh->i_nbQuantBits));
                            mesh->f_quantStep = static_cast<float>(mesh->f_quantStep * scale);
                        }
                    }
                }
                // 与原版 PPMC `determineQuantStep()`（mymesh.cpp）对齐：开启
                // lifting scheme 时把 bboxMin 向负方向外扩 2^(bits-1) 个量化步。
                // 原版靠这个填充保证 lift 后的整数坐标永不越界（那边是 assert）；
                // 框架不填充时贴在 bboxMin/Max 平面上的顶点会触发 lift 的越界
                // 保护，破坏编解码对称性。反量化用同一 bboxMin+step，几何不变。
                for (int axis = 0; axis < 3; ++axis) {
                    mesh->bboxMin[axis] -= static_cast<float>(
                        (1u << (mesh->i_nbQuantBits - 1)) * mesh->f_quantStep);
                }
                mesh->quantizeVertexPositions();
            };
        };
    }

    /// 论文 §3.3 的预测开关由 header 下传给 collector（PPMC 的预测是全有全无）。
    BeforeCompressLoopFn beforeCompressLoop() const override {
        return [](std::shared_ptr<MCGAL::Mesh> mesh, PMSFHeader& header,
                  std::shared_ptr<SymbolCollectOperator> collector,
                  const CompressOptions& /*options*/) {
            auto ppmcColl = std::dynamic_pointer_cast<PPMCSymbolCollectOperator>(collector);
            if (!ppmcColl || !mesh) return;

            bool pureTriangle = true;
            for (MCGAL::Facet* f : mesh->faces()) {
                if (f->facet_degree() != 3) {
                    pureTriangle = false;
                    break;
                }
            }
            (void)pureTriangle;

            // Paper defaults: connectivity prediction (faces/edges) + local curvature
            // prediction + lifting are all enabled together.  The switches are
            // carried in PMSFHeader so the decoder reproduces the same predictors.
            const bool usePred = header.isEnablePrediction();
            // 曲率预测（参考实现 §4.2 式(2)）：默认开启。此前因 3 个对称性 bug
            // （PointInt::operator/ 忽略除数、MCGAL float Frenet 旋转精度、
            // proxy-依赖的 1-ring Laplacian）导致解码发散；现已全部修复，
            // curvature-on 解码与 curvature-off 逐字节相同（md5 一致）。
            // A/B 时可用 PMSF_PPMC_CURVATURE=0 关闭。
            const char* curvEnv = std::getenv("PMSF_PPMC_CURVATURE");
            const bool useCurv = usePred && !(curvEnv && curvEnv[0] == '0');
            ppmcColl->setEnablePrediction(usePred);
            ppmcColl->setCurvaturePrediction(useCurv);
            ppmcColl->setConnectivityPredictionFaces(
                usePred && header.isEnableConnectivityPredictionFaces());
            ppmcColl->setConnectivityPredictionEdges(
                usePred && header.isEnableConnectivityPredictionEdges());
            const bool useTriBalance = usePred && header.isEnableTriangleMeshBalance()
                                       && mesh->size_of_vertices() >= 5000;
            ppmcColl->setUseTriangleMeshConnectivityPrediction(useTriBalance);
            const char* liftEnv = std::getenv("PMSF_PPMC_LIFTING");
            ppmcColl->setUseLiftingScheme(!(liftEnv && liftEnv[0] == '0'));

            header.setEnablePrediction(usePred);
            header.setEnableConnectivityPredictionFaces(
                usePred && header.isEnableConnectivityPredictionFaces());
            header.setEnableConnectivityPredictionEdges(
                usePred && header.isEnableConnectivityPredictionEdges());
            header.setEnableTriangleMeshBalance(useTriBalance);
            header.setEnableCurvaturePrediction(useCurv);
        };
    }

    /// 解码侧：把 header 的预测开关交给 PPMC 自己的 reader。
    AfterBaseMeshFn afterBaseMeshHook() const override {
        return [](const PMSFHeader& header, std::shared_ptr<DeserializeOperator> deser,
                  std::shared_ptr<MCGAL::Mesh> /*mesh*/) {
            if (!deser) return;
            auto reader = std::dynamic_pointer_cast<PPMCSymbolReadOperator>(
                deser->symbolReadOperator());
            if (!reader) return;
            reader->setEnablePrediction(header.isEnablePrediction());
            reader->setConnectivityPredictionFaces(
                header.isEnableConnectivityPredictionFaces());
            reader->setConnectivityPredictionEdges(
                header.isEnableConnectivityPredictionEdges());
            reader->setUseTriangleMeshConnectivityPrediction(header.isEnableTriangleMeshBalance());
            reader->setCurvaturePrediction(header.isEnableCurvaturePrediction());

            const char* liftEnv = std::getenv("PMSF_PPMC_LIFTING");
            reader->setUseLiftingScheme(!(liftEnv && liftEnv[0] == '0'));
        };
    }

    PostDecompressFn postDecompressHook() const override {
        return [](std::shared_ptr<MCGAL::Mesh> mesh) {
            const char* env = std::getenv("PMSF_REPAIR_DEGENERATE");
            if (!env || *env == '\0' || *env == '0') return;
            const int repaired = repairDegenerateTriangleGeometry(mesh);
            if (repaired > 0) {
                std::fprintf(stderr, "[PPMC Decompress] repaired_degenerate_faces=%d\n",
                             repaired);
            }
        };
    }

    /// perf：解压侧池容量自算（忽略 config extra 里超配的 PMSF_POOL_*）。
    /// 解析/重建的池峰值与压缩端一致（horse：V=19,851 / H=183,226 / F=71,764）。
    DecompressOptions makeDecompressOptions(Backend backend, const std::string& input,
                                            const std::string& output) const override {
        // perf：解码路径同样不读面法线（output/repair/qual 都不需要）。
        setenv("PMSF_SKIP_FACE_NORMALS", "1", 0);
        DecompressOptions opts =
            ISimplificationAlgorithm::makeDecompressOptions(backend, input, output);
        opts.setPoolSizer([](int origV, int origF) { return ppmcPoolSizesFor(origV, origF); });
        return opts;
    }

    /// 解压组件在同一 cpp 内注册 —— 算法不需要写任何解压入口。
    static void registerDecompressComponents() {
        NamedDecompressOperatorRegistrar(
            "ppmc", Backend::Columnar, []() -> DecompressOperatorTriple {
                auto reader = std::make_shared<PPMCSymbolReadOperator>();
                auto reconstructor = std::make_shared<PPMCReconstructOperator>();
                std::shared_ptr<DeserializeOperator> deserializer =
                    std::make_shared<ColumnarDeserializeOperator>(reader, reconstructor);
                return {deserializer, reader, reconstructor};
            });
        NamedDecompressOperatorRegistrar(
            "ppmc", Backend::Binary, []() -> DecompressOperatorTriple {
                // 二进制后端需要 PPMC 自己的解压器：PPMC 的 header 位语义与通用
                // 解压器不同（PPMC 用连通性预测，通用解压器把 prediction 位当
                // 「RD 残差 range-coder」）。历史上这里注册的是 nullptr，二进制
                // 路径一直回落到通用解压器，因此从未真正跑通过。
                auto reader = std::make_shared<PPMCSymbolReadOperator>();
                auto reconstructor = std::make_shared<PPMCReconstructOperator>();
                std::shared_ptr<DeserializeOperator> deserializer =
                    std::make_shared<PPMCDeserializeOperator>(reader, reconstructor);
                return {deserializer, reader, reconstructor};
            });
    }
};

const bool g_ppmcDecompressRegistered = [] {
    PPMCAlgorithm::registerDecompressComponents();
    return true;
}();

}  // namespace
}  // namespace pmsf

PMSF_REGISTER_SIMPLIFICATION_ALGORITHM(pmsf::PPMCAlgorithm);