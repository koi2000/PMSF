/*
 * @Author: koi
 * @Date: 2025-08-26 22:18:50
 * @Description:
 */

#pragma once

#include "EliminateOperator.h"
#include "Halfedge.h"
#include <climits>
#include <cstdlib>
#include <vector>

namespace pmsf {
extern int currentCompressRoundForGuard;
}

class VertexRemovalEliminateOperator : public EliminateOperator {
  public:
    VertexRemovalEliminateOperator() = default;
    ~VertexRemovalEliminateOperator() = default;

    void init(std::shared_ptr<MCGAL::Mesh> mesh, PMSFHeader header, bool compress_boundary = false) override;

    bool eliminate(HalfedgeErrorEntry& entry) override;
    bool remove_point(MCGAL::Halfedge* h) override;
    bool triangulate(MCGAL::Halfedge* h) override;
    bool postprocess(MCGAL::Halfedge* h) override;
    MCGAL::Halfedge* encode_boundary(MCGAL::Halfedge* h) override;

    // Phase 0 · Step 0.4 —— 暴露本轮 patch face 列表。默认实现见基类。
    std::vector<MCGAL::Facet*> takeRoundDecimatedPatches() override;
    std::vector<MCGAL::Facet*> takeRoundCleaningPatches() override;
    void                        resetRoundAccumulators() override;
    void                        setInCleaningPhase(bool inCleaning) override;

    int groupId = -1;

  protected:
    /// Removed vertex quantized position saved in remove_point() so the residual
    /// can be re-predicted after fan-triangulation using the final patch face
    /// ring that the decoder will actually insert into.
    MCGAL::PointInt pendingRemovedPosInt_{0, 0, 0};
    bool hasPendingRemovedPos_ = false;
    MCGAL::Vertex* pendingRemovedVertex_ = nullptr;

  protected:
    /// @brief 决定是否在 erase_center_vertex 后维护 1-ring 顶点的缓存度数。
    ///        默认维护；若算法自己管理度数可 override 返回 false。
    virtual bool shouldMaintainRingDegreeCache() const { return true; }

    /// @brief 计算并返回被移除顶点的 valence（即 polygon 边数）。
    ///        默认在 erase 前读取 removedVertex->vertex_degree()；PCATM 等算法可 override。
    virtual int computeRemovedValence(MCGAL::Halfedge* h,
                                      MCGAL::Vertex* removedVertex);

    /// @brief 计算 polyFace 上记录的“被移除顶点整数残差”。
    /// @param h        入口 halfedge。
    /// @param hface    erase_center_vertex 返回的 face 入口 halfedge。
    /// @param vrPoint  被移除顶点的浮点坐标（erase 前缓存）。
    /// @param vrPos    被移除顶点的量化整数坐标（erase 前缓存）。
    virtual MCGAL::Vector3i computeRemovedVertexResidualInt(
        MCGAL::Halfedge* h,
        MCGAL::Halfedge* hface,
        const MCGAL::Point& vrPoint,
        const MCGAL::PointInt& vrPos);

    /// @brief 计算并存入 polyFace->removedVertexPos() 的浮点坐标/残差。
    ///        默认关闭 quantization 时存 barycentric 残差，否则存原始坐标。
    virtual MCGAL::Point computeStoredVertexPoint(
        MCGAL::Halfedge* h,
        MCGAL::Halfedge* hface,
        const MCGAL::Point& vrPoint);

    /// @brief 把新产出的 patch face 注册到本轮列表（decimating / cleaning）。
    void recordRoundPatch(MCGAL::Facet* patchFace);

    // 本轮 decimation 中新产出的 polyFace（post-fan triangulation 入口的 face）。
    // encoder 的 BFS 遍历会以它们为 patch 根；新 collector 取走。
    std::vector<MCGAL::Facet*> roundDecimated_;
    // PCLTTM cleaning conquest 子轮的 patch list；除非 caller 主动 setInCleaningPhase(true)，
    // 否则始终为空。
    std::vector<MCGAL::Facet*> roundCleaning_;
    bool inCleaningPhase_ = false;
};
