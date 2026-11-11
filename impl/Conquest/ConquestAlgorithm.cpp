/*
 * @file ConquestAlgorithm.cpp
 * @brief valence-conquest 原生 codec 的算法入口。
 *
 * 接入面（P13.4 (B)②）：
 *   - 算法名 `"conquest"` —— 写进 metadata，也是解压侧按名分派注册表的键；
 *   - `collectorFactory()` 给出编码端薄壳；遍历仍在引擎里（蓝图 §62.2 路线 A）；
 *   - 解压侧三件套在 `impl/Conquest/DecompressFactoryRegistration.cpp` 按同名注册
 *     ⇒ 本算法**不需要**任何 `XxxDecompress.cpp`。
 *
 * ⚠️ 与既有算法的**架构差别**：别的算法把"选谁删 / 怎么删"交给框架的
 * selection + elimination 链，collector 只做符号映射；conquest 的引擎
 * （`FrontierConquestEngine`）**自己跑前沿 BFS 并自己删点**。
 * 因此本算法**必须**让框架侧不施加遍历 —— 载体就是下面的
 * `ConquestNoopEliminateOperator`（`eliminate()` 恒 false）。
 */
#include "Conquest/compress/ConquestSymbolCollectOperator.h"
#include "Conquest/elimination/ConquestNoopEliminateOperator.h"
#include "algorithm/ISimplificationAlgorithm.h"
#include "algorithm/SimplificationAlgorithmRegistry.h"

namespace pmsf {
namespace {

class ConquestAlgorithm final : public ISimplificationAlgorithm {
  public:
    AlgorithmInfo info() const override {
        return AlgorithmInfo{
            .name = "conquest",
            .version = 1,
            .primitive = PrimitiveKind::Vertex,
            .selection = SelectionKind::ConnectivityDriven,
        };
    }

    /// ★ **框架侧不得施加遍历**（蓝图 §62.2 路线 A 的另一半）。
    /// conquest 的引擎自己跑前沿 BFS 并自己删点；若框架的 select/eliminate 链
    /// 再叠一层，网格会被**删两遍**，两侧立刻分岔。装一个恒 false 的空算子之后，
    /// 每轮形状变成 `select（空转）→ collector->collect(seed)（引擎在这里删点）`。
    EliminateOperatorPtr eliminate() const override {
        return std::make_shared<ConquestNoopEliminateOperator>();
    }

    /// 遍历在引擎里（蓝图 §62.2）：collector 自己驱动前沿 BFS 并删点。
    CollectorFactory collectorFactory() const override {
        return []() -> std::shared_ptr<SymbolCollectOperator> {
            return std::make_shared<ConquestSymbolCollectOperator>();
        };
    }
};

}  // namespace
}  // namespace pmsf

PMSF_REGISTER_SIMPLIFICATION_ALGORITHM(pmsf::ConquestAlgorithm);
