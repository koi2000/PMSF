/*
 * @file ConquestReconstructOperator.h
 * @brief valence-conquest 的**重建器**：每轮把一层符号还原成拓扑与几何。
 *
 * 它**不自己实现解码** —— 只做三件编排：
 *   1. 从 `ConquestSymbolReadOperator` 取走本层的记录；
 *   2. 用记录里的**种子坐标**在当前网格上找回种子半边
 *      （池号在编解码两端不可比，必须按坐标找）；
 *   3. 调 `pmsf::decodeLayer`（`src/conquest/ConquestDecodeLayer.h`）。
 *
 * ⚠️ **解码实体只有一处**：`pmsf::decodeLayer` 是逐层验证过的那一份
 * （四网格 × 8 层链式 `topoGeomOk=1`）。本类**不得**复制它的逻辑。
 */
#ifndef PMSF_CONQUEST_RECONSTRUCT_CONQUEST_RECONSTRUCT_OPERATOR_H
#define PMSF_CONQUEST_RECONSTRUCT_CONQUEST_RECONSTRUCT_OPERATOR_H

#include "conquest/ConquestDecodeLayer.h"
#include "conquest/FrontierConquestEngine.h"
#include "reconstruct/ReconstructOpertator.h"

#include <memory>
#include <vector>

namespace pmsf {

class ConquestSymbolReadOperator;  // impl/Conquest/decompress/

class ConquestReconstructOperator : public ReconstructOperator {
  public:
    explicit ConquestReconstructOperator(std::shared_ptr<ConquestSymbolReadOperator> reader);

    /// @brief 与编码端**同一份** hook 实例（留空则回落框架默认）。
    void setHooks(EngineHooksPtr hooks) {
        hooks_ = std::move(hooks);
    }

    void reconstruct(MCGAL::Halfedge* halfedge) override;
    void reconstruct(std::vector<MCGAL::Vertex*>& vertices,
                     std::vector<MCGAL::Halfedge*>& halfedge,
                     std::vector<MCGAL::Facet*>& facets) override;

    /// @brief 最近一层的解码统计（诊断）。
    const ConquestDecodeResult& lastResult() const {
        return last_;
    }
    /// @brief 累计插回的顶点数（>0 表示真的动了网格）。
    int totalInserted() const {
        return totalInserted_;
    }
    /// @brief 是否出现过不可恢复的失败。
    bool failed() const {
        return failed_;
    }

  private:
    std::shared_ptr<ConquestSymbolReadOperator> reader_;
    EngineHooksPtr hooks_;
    ConquestState st_;
    ConquestDecodeResult last_;
    int totalInserted_ = 0;
    bool failed_ = false;
    bool loggedOnce_ = false;
};

}  // namespace pmsf

#endif  // PMSF_CONQUEST_RECONSTRUCT_CONQUEST_RECONSTRUCT_OPERATOR_H
