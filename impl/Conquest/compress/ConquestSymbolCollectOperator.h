/*
 * @file ConquestSymbolCollectOperator.h
 * @brief 顶点删减式（valence conquest）编解码的**编码端薄壳**。
 *
 * 形状照 `impl/POMAR/compress/PomarSymbolCollectOperator.h`（全仓最小，27 行）：
 * **只有 4 个覆写**，其余继承 `SymbolCollectOperator` 的默认实现。
 *
 * ⚠️ **遍历留在引擎里**（设计定案见
 * `docs/refactor/P13_4_DECODER_BLUEPRINT_2026-09-22.md` §62.2）：
 * `FrontierConquestEngine` 的前沿式 FIFO 是本次唯一被逐层验证过的遍历
 * （四网格 × 8 层链式，层往返 `topoGeomOk=1`），
 * **不得**把它改成 `RDGateOnlySymbolCollectOperator` 那种"遍历交给基类"的分工。
 *
 * 真负载 = 连通性符号串 + 几何残差（整数域 `cellDelta`），
 * 由 `exportToBufferWithRangeCoder` 按 `conquest_blob` 的字节布局写出
 * （该布局在编码端与解码端是**同一个头文件**，见
 * `src/conquest/ConquestBlobFormat.h`）。
 */
#ifndef PMSF_CONQUEST_COMPRESS_CONQUEST_SYMBOL_COLLECT_OPERATOR_H
#define PMSF_CONQUEST_COMPRESS_CONQUEST_SYMBOL_COLLECT_OPERATOR_H

#include "compress/SymbolCollectOperator.h"
#include "conquest/ConquestBlobFormat.h"
#include "conquest/FrontierConquestEngine.h"

#include <array>
#include <cstdint>
#include <deque>
#include <memory>
#include <vector>

namespace pmsf {

class ConquestSymbolCollectOperator : public SymbolCollectOperator {
  public:
    ConquestSymbolCollectOperator() = default;

    /// 基类的 `init` 是**唯一**提供网格的入口（`MCGAL::Halfedge` 本身不带 mesh 指针）。
    void init(std::shared_ptr<MCGAL::Mesh> mesh, std::vector<MCGAL::Halfedge*> seeds, PMSFHeader header) override;

    /// @brief 跑**一层**：`decimateLayer` 一次，把它的两样输出收成一条 `LayerRecord`。
    /// 框架每轮调一次，与"一层一次"的参考语义一致。
    void collect(MCGAL::Halfedge* seed) override;

    void collect(std::vector<MCGAL::Halfedge*> seeds) override;
    int exportToBuffer(char* buffer, bool enableQuantization = false, bool enableCompress = false) override;

    /// @brief 主导出：把已收集的各层按 `conquest_blob` 布局写出。返回字节数（失败 -1）。
    int exportToBufferWithRangeCoder(char* buffer, bool enableQuantization = false) override;

    const pmsf::PMSFSymbolTable* pmsfSymbolTable() const override;

    // ---- 量化网格 ----------------------------------------------------------
    /// @brief 量化位数（各向同性单步长 = 包围盒最长边 / 2^bits）。默认 12。
    void setQuantBits(unsigned bits) {
        quantBits_ = bits;
    }
    /// @brief 显式指定量化网格（与外部约定一致时用；不设则由 `init` 按网格包围盒现算）。
    void setQuantGrid(const pmsf::FrontierConquestEngine::QuantGrid& g) {
        grid_ = g;
        gridForced_ = true;
    }
    const pmsf::FrontierConquestEngine::QuantGrid& quantGrid() const {
        return grid_;
    }

    // ---- 供布局插件与测试读取 ----------------------------------------------
    const conquest_blob::Stream& stream() const {
        return stream_;
    }
    unsigned quantBits() const {
        return quantBits_;
    }
    /// @brief 引擎的五个 hook（与解码端必须同一份实例）。
    void setHooks(pmsf::EngineHooksPtr hooks) {
        hooks_ = std::move(hooks);
    }

  private:
    /// @brief 按网格包围盒现算量化网格（`init` 里做一次；已显式指定则跳过）。
    void computeGridFromMesh();
    /// @brief 把工作网格**吸附到格中心**（闭环预测的前提，见 .cpp 里的说明）。
    void snapMeshToCells();

    /// 连通性/几何的**逐层**记录；`exportToBufferWithRangeCoder` 直接写它。
    conquest_blob::Stream stream_;
    std::shared_ptr<MCGAL::Mesh> mesh_;
    pmsf::FrontierConquestEngine::QuantGrid grid_;
    bool gridForced_ = false;
    unsigned quantBits_ = 12;
    pmsf::EngineHooksPtr hooks_;
    mutable pmsf::PMSFSymbolTable paperTable_;
};

}  // namespace pmsf

#endif  // PMSF_CONQUEST_COMPRESS_CONQUEST_SYMBOL_COLLECT_OPERATOR_H
