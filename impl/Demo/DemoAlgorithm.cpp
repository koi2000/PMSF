/*
 * @file DemoAlgorithm.cpp
 * @brief 接入成本的**下界演示**：只实现 info() 的「零优化」算法。
 *
 * 这是设计文档 §2.1.1 的验收标尺 —— 一个新算法不需要任何 hook、不需要任何
 * 解压入口、不需要改 src/ 或 CMake，只写下面这 ~20 行就能跑通
 * 压缩 → 统一存储 → 解压 round-trip，并自动获得 ROI×LOD 随机访问能力。
 *
 * 框架为标准行为补齐的部分：
 *   H1 可消除性链 = Basic + UnifiedManifold + Boundary
 *   H2 无误差度量（连通驱动）          H3 确定性环三角化
 *   H4/H5/H6 不移位/不翻转             H7/H8 不分区/标准边界
 *   量化 = Q12、预测 = parallelogram、存储 = canonical 六表 schema
 *
 * 对照：QEM 的完整接入（impl/QEM/QEMAlgorithm.cpp）也只多了一个 H2 hook。
 *
 * 用法（与其它算法完全一致，无需专属可执行文件）：
 *   build/PMSFCompress   configs/algorithms/Demo.json
 *   build/PMSFDecompress configs/algorithms/Demo.json
 */
#include "algorithm/SimplificationAlgorithmRegistry.h"

namespace pmsf {
namespace {

class DemoAlgorithm final : public ISimplificationAlgorithm {
  public:
    AlgorithmInfo info() const override {
        return AlgorithmInfo{
            .name = "demo",
            .version = 1,
            .primitive = PrimitiveKind::Vertex,
            .selection = SelectionKind::ConnectivityDriven,
        };
    }
    // 其余全部走框架默认 —— 这就是全部接入代码。
};

}  // namespace
}  // namespace pmsf

PMSF_REGISTER_SIMPLIFICATION_ALGORITHM(pmsf::DemoAlgorithm);
