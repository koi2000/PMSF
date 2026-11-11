/*
 * @file RoundExportDecompress.cpp
 * @brief P17.2b：从压缩产物导出**逐轮** OFF 快照（渐进 LOD 导出）。
 *
 * 用法:
 *   RoundExportDecompress --algo=<name> --input=<prefix> --output-dir=<dir>
 *                         [--backend=columnar|binary] [--max-rounds=K]
 *
 * 背景：原 `tools/RoundExportDecompress.cpp`（593 行）在 e5af44c（2026-06-29）
 * 随同族工具一并删除，且其引用的 API 面（`core/AlgoPreset.h`、
 * `decompress/MMSEDualRingSymbolReadOperator.h` 等）此后已随 P15/P16 重构迁移，
 * **不能直接恢复**。本文件按当前 API 重写，只走算法注册表这一条正路。
 *
 * 机制（与 `src/query/Query.cpp` 的随机访问同源，不新增旁路装配）：
 *   1. `registry.create(algo)` 取该算法自己注册的解压选项（含配套 reader/
 *      reconstructor 与 afterBaseMesh 钩子）；
 *   2. 对 k 逐档 `deserializer->setQueryMaxLod(k)`（= `setMaxRounds(k)`，
 *      见 `decompress/ColumnarDeserializeOperator.h:66`）；
 *   3. `runDecompress` → `mesh->dumpto_oldtype(<dir>/round_<k>.off`)。
 *
 * 判据（计划 P17.2b）：导出轮数与压缩 rounds 一致；各轮顶点数单调；
 * 最终轮 = 全分辨率。
 */
#include "Mesh.h"
#include "algorithm/SimplificationAlgorithmRegistry.h"
#include "config/PMSFConfigParser.h"
#include "core/CompressPipeline.h"
#include "core/DecompressPipeline.h"
#include "decompress/ColumnarDeserializeOperator.h"
#include "ContextPool.h"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

namespace {

std::string argValue(int argc, char* argv[], const std::string& key,
                     const std::string& fallback = std::string()) {
    const std::string flag = "--" + key + "=";
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a.rfind(flag, 0) == 0) return a.substr(flag.size());
    }
    return fallback;
}

}  // namespace

int main(int argc, char* argv[]) {
    const std::string algo = argValue(argc, argv, "algo");
    const std::string input = argValue(argc, argv, "input");
    const std::string outDir = argValue(argc, argv, "output-dir");
    const std::string backendStr = argValue(argc, argv, "backend", "columnar");
    const int maxRoundsArg = std::atoi(argValue(argc, argv, "max-rounds", "0").c_str());
    // P18.2 §0：`--only-round=K` —— **一个进程只导一轮**。
    //
    // 为什么必须这样：`ContextPool::registerPool` 是 `new Vertex[n]/Halfedge[n]/
    // Facet[n]` 后 `push_back`，**从不释放**（`releasePool()` 是空实现、
    // `~ContextPool` 的 delete 被注释掉），且 `deps/MCGAL` 禁改。框架自己的
    // 用法是「一个进程一次解压」（PMSFDecompress / Query 皆然）。本工具原先
    // 在一个进程里循环 K 轮、每轮重新 registerPool ⇒ 囤 K 副全尺寸 arena。
    // 实测 xyzrgb_dragon（3,609,600 v）在第 26 轮耗尽，随后 V×128 的公式值
    // 直接 OOM（主计划 v68 三证）。改成逐轮一进程后，峰值内存 = 单轮。
    const int onlyRound = std::atoi(argValue(argc, argv, "only-round", "0").c_str());
    // PHASE60 物品 2：`--only-group=G`（G>=0）—— **只解一个分区**。
    //
    // 与 `--only-round` 同源：都走 `DeserializeOperator` 已有的随机访问接口
    // （`setQueryMaxLod` / `setTargetGroupId`，见
    // `decompress/ColumnarDeserializeOperator.h:60-67`），不新增旁路装配。
    // `setTargetGroupId` 只声明在 `ColumnarDeserializeOperator` 上，故这里
    // dynamic_cast —— 二进制后端（无分区）没有该接口，会响亮拒绝。
    const int onlyGroup = std::atoi(argValue(argc, argv, "only-group", "-1").c_str());

    if (algo.empty() || input.empty() || outDir.empty()) {
        std::cerr << "用法: " << argv[0]
                  << " --algo=<name> --input=<prefix> --output-dir=<dir>"
                     " [--backend=columnar|binary] [--max-rounds=K]"
                     " [--only-round=K] [--only-group=G]\n";
        return 1;
    }

    const pmsf::Backend backend = (backendStr == "binary")
                                      ? pmsf::Backend::Binary
                                      : pmsf::Backend::Columnar;

    try {
        auto& registry = pmsf::SimplificationAlgorithmRegistry::instance();
        auto algorithm = registry.create(algo);
        if (!algorithm) {
            std::cerr << "[RoundExport] 算法 '" << algo << "' 未注册\n";
            return 1;
        }

        std::filesystem::create_directories(outDir);

        // 轮数必须显式给。框架没有「读产物问轮数」的只读访问器（解压器要
        // init 之后才有 header），而调用方（benchmark 脚本）本就知道压缩时
        // 传的 rounds —— 与其在这里猜，不如要求它传。
        const int maxRounds = maxRoundsArg;
        if (maxRounds <= 0) {
            std::cerr << "[RoundExport] 必须显式传 --max-rounds=K\n";
            return 1;
        }

        const int kFirst = (onlyRound > 0) ? onlyRound : 1;
        const int kLast = (onlyRound > 0) ? onlyRound : maxRounds;
        if (onlyRound > maxRounds) {
            std::cerr << "[RoundExport] --only-round=" << onlyRound
                      << " 超出 --max-rounds=" << maxRounds << "\n";
            return 1;
        }

        int exported = 0;
        std::size_t prevVerts = 0;
        for (int k = kFirst; k <= kLast; ++k) {
            std::string stem = "round_" + std::to_string(k);
            if (onlyGroup >= 0)
                stem += "_group_" + std::to_string(onlyGroup);
            const std::string outPath = (std::filesystem::path(outDir) / (stem + ".off")).string();

            // 池注册：单轮模式下进程内只解压一次 ⇒ 只注册一副 arena，
            // 峰值内存 = 该轮所需（见 main 顶部 --only-round 的说明）。
            // 仍保留 initPoolSize/registerPool 以设定尺寸上限。
            MCGAL::contextPool.initPoolSize(1);
            MCGAL::contextPool.registerPool(
                MCGAL::ContextPool::poolSizeFromEnv("ROUNDEXPORT_POOL_V", 2000000),
                MCGAL::ContextPool::poolSizeFromEnv("ROUNDEXPORT_POOL_H", 8000000),
                MCGAL::ContextPool::poolSizeFromEnv("ROUNDEXPORT_POOL_F", 4000000));

            // 每次都重新取一遍选项：解压器是有状态的，跨轮复用会把上一轮
            // 的 visited/队列状态带进来。
            auto options = algorithm->makeDecompressOptions(backend, input, outPath);
            auto deserializer = options.getDeserializeOperator();
            if (!deserializer) {
                std::cerr << "[RoundExport] 无法为算法 '" << algo
                          << "' 构造解压器\n";
                return 1;
            }
            deserializer->setQueryMaxLod(k);
            if (onlyGroup >= 0) {
                auto* columnar = dynamic_cast<ColumnarDeserializeOperator*>(deserializer.get());
                if (columnar == nullptr) {
                    std::cerr << "[RoundExport] --only-group 需要列式（columnar）"
                                 "后端的分区解压器，当前算法 '"
                              << algo << "' 的产物不是分区布局\n";
                    return 1;
                }
                columnar->setOnlyGroupId(onlyGroup);
            }

            auto mesh = pmsf::runDecompress(options, algorithm->afterBaseMeshHook());
            if (!mesh) {
                std::cerr << "[RoundExport] round " << k << " 解压失败\n";
                return 1;
            }
            const std::size_t verts = mesh->size_of_vertices();
            mesh->dumpto_oldtype(outPath);
            std::cout << "[RoundExport] round=" << k << " vertices=" << verts
                      << " monotone=" << ((k == 1 || verts >= prevVerts) ? 1 : 0)
                      << " out=" << outPath << std::endl;
            prevVerts = verts;
            ++exported;
        }

        std::cout << "[RoundExport] 完成: 轮数=" << maxRounds
                  << " 导出=" << exported
                  << " 顶点数=" << prevVerts << std::endl;

        // P18.2 物品 5：**workaround，不是修复**。
        //
        // 实测某次 `--only-round=19` 在**全部工作完成后**挂死：日志已打出上面
        // 这行、OFF 已写、除 0/1/2 外所有 fd 已关闭；进程 `State: S`、
        // 4 线程全在 `futex_wait_queue`、CPU time 37 s / wall 41 min
        // ⇒ 卡在 **main 返回后的进程拆除 / 静态析构**，而非计算或 OOM。
        //
        // 根因**未定**：已用最小复现证伪 `MCGAL::ThreadPool` 的静态析构
        // （构造 128 工作线程后从 main 返回，rc=0 干净退出）；
        // 剩余嫌疑是 Arrow 的全局线程池，但 `ptrace_scope=1` 挡住栈回溯，
        // 且该挂死是**间歇性**的（该 artifact round 1–18 均干净退出）。
        //
        // 故这里用 `std::_Exit` 跳过全部静态析构：本工具在这一点上所有产物
        // 都已落盘并 close，没有需要静态析构去 flush 的状态。
        // **一旦根因查清（拿到栈回溯），应移除本 workaround 改为真修复。**
        std::cout.flush();
        std::cerr.flush();
        std::_Exit(0);
    } catch (const std::exception& e) {
        std::cerr << "[RoundExport] error: " << e.what() << "\n";
        return 1;
    }
}
