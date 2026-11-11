/*
 * @Author: koi
 * @Date: 2025-08-26 22:07:17
 * @Description:
 */
#ifndef ELIMINATE_OPERATOR_H
#define ELIMINATE_OPERATOR_H

#include "../common/HalfedgeErrorEntry.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "header/PMSFHeader.h"
#include <memory>
#include <vector>

namespace MCGAL { class Facet; }

/**
 * eliminate中需要传入边，然后执行删除操作，这里需要考虑一些其他问题，如何存以及周围的更新
 * 这里可以分为三步，1. 删除点，2.三角化 3. 调整点的位置
 */

class EliminateOperator {
  public:
    EliminateOperator() = default;

    virtual void init(std::shared_ptr<MCGAL::Mesh> mesh, PMSFHeader header, bool compress_boundary = false) {
        mesh_ = mesh;
        compress_boundary_ = compress_boundary;
        header_ = header;
    };

    virtual ~EliminateOperator() = default;

    virtual bool eliminate(HalfedgeErrorEntry& entry) = 0;

    virtual bool remove_point(MCGAL::Halfedge* h) = 0;

    virtual bool triangulate(MCGAL::Halfedge* h) = 0;

    virtual bool postprocess(MCGAL::Halfedge* h) = 0;

    virtual MCGAL::Halfedge* encode_boundary(MCGAL::Halfedge* h) = 0;

    // ================================================================
    // Phase 0 · Step 0.4 (docs/CONNECTIVITY_BITSTREAM_IMPLEMENTATION_PLAN.md)
    // ----------------------------------------------------------------
    // 连通性比特流的所有方案（V / B / C / P / H）都需要 "本轮被 decimate
    // 的 patch face 列表"作为 collector 输入。为避免每个 collector 自己重
    // 新跑 BFS 去识别 patch face（冗余且容易和 eliminate 内部状态脱钩），
    // 把累积 hook 下沉到这里，默认实现为 no-op —— 完全不影响已绿化的
    // Parquet V2 / Binary baseline。
    //
    // 约定：
    //   - encoder 每次 remove_point/eliminate 成功后，子类把新产出的
    //     polyFace (post-fan-triangulation) append 到内部 vector；
    //   - CompressPipeline 在 round loop 顶部调用 resetRoundAccumulators()
    //     清空上一轮遗留；
    //   - collector 在 round loop 内调用 takeRoundDecimatedPatches() /
    //     takeRoundCleaningPatches() 取出本轮列表（**take 语义：取走并清空**）。
    //
    // PCLTTM 的 cleaning conquest 子轮：caller 先
    //   setInCleaningPhase(true)，eliminate() 产出的 patch 进入
    //   roundCleaning_；再切回 false 恢复 roundDecimated_。
    //
    // 任何不需要这些 hook 的子类都可以继续使用默认 no-op 实现。
    // ================================================================

    /// @brief 取走本轮 "decimating conquest" 阶段被 decimate 的 patch face 列表。
    ///        返回后内部清空；同一轮重复调用返回空。
    virtual std::vector<MCGAL::Facet*> takeRoundDecimatedPatches() { return {}; }

    /// @brief 取走本轮 "cleaning conquest" 阶段被删除的 V3 顶点 patch 列表。
    ///        只有区分 cleaning/decim 子轮的算法（PCLTTM 论文）产出非空。
    virtual std::vector<MCGAL::Facet*> takeRoundCleaningPatches() { return {}; }

    /// @brief 重置本轮累加器。Pipeline 在 round loop 顶部统一调用。
    virtual void resetRoundAccumulators() {}

    /// @brief 通知 eliminator 当前处于 cleaning 子轮。
    ///        默认 no-op；支持 cleaning 的子类 override 存起来。
    virtual void setInCleaningPhase(bool /*inCleaning*/) {}

    /// @brief 该 eliminate 算子是否以 edge-collapse 为主要 decimation 模式。
    ///        Pipeline 据此决定 VisualTopology guard 的模式，无需识别具体算法子类。
    virtual bool usesEdgeCollapseMode() const { return false; }

    std::shared_ptr<MCGAL::Mesh> mesh_;
    bool compress_boundary_ = false;
    PMSFHeader header_;
};

#endif