/*
 * @Author: koi
 * @Date: 2025-09-01 09:36:36
 * @Description:
 */

#pragma once

#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"
#include "Facet.h"
#include "decompress/SymbolReadOperator.h"
#include "Vertex.h"
#include "common/FixedQueue.h"
#include <cstddef>
#include <cstdint>   // P2.a-extend：int64_t / __int128 整数累加器
#include <memory>
#include <unordered_map>
#include <stdexcept>
#include <utility>
#include <vector>

class PPMCSymbolReadOperator : public SymbolReadOperator {
  public:
    PPMCSymbolReadOperator() = default;

    // ─── P1 对齐：解码端的预测开关与编码端严格对称 ──────────────────────────
    // `setEnablePrediction` 打开曲率 + 两个连接性预测；若编码端 header 告知 mesh
    // 是纯三角网格，还要调 `setUseTriangleMeshConnectivityPrediction(true)`，
    // 否则 triangle-mesh 分支不进入，流解析将错位。
    // ⚠️ P3.c（2026-05-01）：启用 connectivity prediction（与编码端对称）。
    //   curvature prediction 仍关闭（P3.d 启用）。
    void setEnablePrediction(bool v) {
        b_useCurvaturePrediction = false;             // P3.d TODO
        b_useConnectivityPredictionFaces = v;         // ✅ P3.c 启用
        b_useConnectivityPredictionEdges = v;         // ✅ P3.c 启用
    }
    void setConnectivityPredictionFaces(bool v) { b_useConnectivityPredictionFaces = v; }
    void setConnectivityPredictionEdges(bool v) { b_useConnectivityPredictionEdges = v; }
    void setCurvaturePrediction(bool v) { b_useCurvaturePrediction = v; }
    void setUseTriangleMeshConnectivityPrediction(bool v) {
        b_useTriangleMeshConnectivityPredictionFaces = v;
    }
    void setUseLiftingScheme(bool v) { b_useLiftingScheme = v; }
    /// M4-4 typed v2: consume PMSFSymbolTable rows directly (no env gate).
    void setPaperTypedTableMode(bool v) { paperTypedTableMode_ = v; }
    void setMesh(std::shared_ptr<MCGAL::Mesh> mesh) { mesh_ = std::move(mesh); }
    void setDebugOperationId(int v) { debugOperationId_ = v; }

    void collect(MCGAL::Halfedge* seed,
                 char* buffer,
                 size_t& dataOffset,
                 std::vector<MCGAL::Vertex*>& vertices,
                 std::vector<MCGAL::Halfedge*>& halfedge,
                 std::vector<MCGAL::Facet*>& facets);

    void collect(std::vector<MCGAL::Halfedge*> seeds,
                 char* buffer,
                 size_t& dataOffset,
                 std::vector<MCGAL::Vertex*>& vertices,
                 std::vector<MCGAL::Halfedge*>& halfedge,
                 std::vector<MCGAL::Facet*>& facets);

    /// @brief 从 Parquet 解析的结构化符号数据导入内部队列（不使用 range coder）
    void importFromParquet(const std::vector<bool>& facetSymbols,
                           const std::vector<bool>& edgeSymbols,
                           const std::vector<bool>& vertexSymbols,
                           const std::vector<MCGAL::Point>& points) override;

    /// @brief 从已导入的内部队列中按 BFS 顺序消费符号并重建（不使用 range coder）
    void collectFromDecoded(MCGAL::Halfedge* seed,
                            std::vector<MCGAL::Vertex*>& vertices,
                            std::vector<MCGAL::Halfedge*>& halfedge,
                            std::vector<MCGAL::Facet*>& facets) override;

    void undecimationStep(MCGAL::Halfedge* seed, std::vector<MCGAL::Facet*>& facets);
    void InsertedEdgeDecodingStep(MCGAL::Halfedge* seed, std::vector<MCGAL::Halfedge*>& halfedges);
    // ⚠️ P3.b 对齐：回退到浮点增量均值（与编码端完全对称）。
    void updateAvgSurfaces(bool b_split, float f_faceSurface);
    void decodeGeometrySym(MCGAL::Halfedge* heh_gate, MCGAL::Facet* fh);
    MCGAL::PointInt avgLaplacianVect(MCGAL::Halfedge* heh_gate);
    void updateAvgEdgeLen(bool b_original, float f_edgeLen);
    /// P3.e 对齐：解码端 unlift（原版 PPMC lift(true)）
    void lift(bool b_unlift, std::shared_ptr<MCGAL::Mesh> mesh);

    // 曲率确定性 1-ring 聚合缓存（见 .cpp 注释；与编码端同构、按 poolId 平铺）
    std::vector<int> curvSumQX_, curvSumQY_, curvSumQZ_;
    std::vector<int> curvCnt_;
    std::vector<int> curvPosQX_, curvPosQY_, curvPosQZ_;
    bool curvatureNeighborCacheValid_ = false;
    void resetCurvatureNeighborCache() { curvatureNeighborCacheValid_ = false; }

  private:
    bool inGroup(MCGAL::Halfedge* h) const;
    MCGAL::Halfedge* findNextUnconqueredFaceSeed() const;
    MCGAL::Halfedge* findNextUnprocessedHalfedgeSeed() const;
    bool collectSplitZlibOperation(MCGAL::Halfedge* seed,
                                   char* buffer,
                                   size_t& dataOffset,
                                   std::vector<MCGAL::Vertex*>& vertices,
                                   std::vector<MCGAL::Halfedge*>& halfedge,
                                   std::vector<MCGAL::Facet*>& facets);
    void undecimationStepSplit(MCGAL::Halfedge* seed,
                               const std::vector<char>& faceConn,
                               const std::vector<MCGAL::PointInt>& residuals,
                               bool predictionUsed,
                               std::size_t faceSymbolLimit,
                               std::vector<MCGAL::Facet*>& facets);
    void insertedEdgeDecodingStepSplit(MCGAL::Halfedge* seed,
                                       const std::vector<char>& edgeConn,
                                       bool predictionUsed,
                                       std::size_t edgeSymbolLimit,
                                       std::vector<MCGAL::Halfedge*>& halfedges);
    void applyDecodedFrenetResidual(MCGAL::Halfedge* heh_gate,
                                    MCGAL::Facet* fh,
                                    const MCGAL::PointInt& frenetResidual);
    // Compression and decompression variables.
    rangecoder rangeCoder;

    // Range coder data model.
    qsmodel alphaBetaModel, gammaModel, quantModel, connectModel;

    int alphaBetaMin, gammaMin;

    // ⚠️ P3.b 对齐（2026-05-01）：与 collector 端同步回退到浮点增量均值。
    //   `avg = (avg*(n-1) + f)/n`；判据 `fabs(f - avg_A) <= fabs(f - avg_B)`。
    float f_avgSurfaceFaceWithoutCenterRemoved;
    float f_avgSurfaceFaceWithCenterRemoved;

    float f_avgInsertedEdgesLength;
    float f_avgOriginalEdgesLength;

    unsigned i_nbFacesWithoutCenterRemoved;
    unsigned i_nbFacesWithCenterRemoved;

    unsigned i_nbInsertedEdges;
    unsigned i_nbOriginalEdges;

    unsigned i_nbGoodPredictions;
    bool b_predictionUsed;

    // Codec features status.
    bool b_useAdaptiveQuantization = false;
    bool b_useLiftingScheme = false;
    bool paperTypedTableMode_ = false;
    bool b_useCurvaturePrediction = false;
    bool b_useConnectivityPredictionFaces = false;
    bool b_useConnectivityPredictionEdges = false;
    bool b_useTriangleMeshConnectivityPredictionFaces = false;
    // ⚠️ P2.a 对齐：与 collector 端保持一致，扩容避免 bunny 级数据 BFS 峰值溢出。
    FixedQueue<int> gateQueue = FixedQueue<int>(300000);
    // void collectHalfedgeSymbol(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Halfedge* seed);
    // void collectFacetSymbol(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Halfedge* seed);

    // Parquet 模式：从结构化数据导入的符号队列
    std::deque<bool> parquetFacetSymbols_;
    std::deque<bool> parquetEdgeSymbols_;
    std::deque<MCGAL::PointInt> parquetGeomSymbols_;
    bool useParquetMode_ = false;
    std::size_t nativeRoundIndex_ = 0;
    std::vector<MCGAL::Facet*> boundaryScanFacets_;
    std::shared_ptr<MCGAL::Mesh> mesh_;
    int debugOperationId_ = -1;
};