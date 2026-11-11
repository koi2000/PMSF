/*
 * @Author: koi
 * @Date: 2025-08-29 21:11:03
 * @Description:
 */

#ifndef PPMC_SYMBOLCOLLECTOPERATOR_H
#define PPMC_SYMBOLCOLLECTOPERATOR_H

#include "core/PMSFBoundarySideChannel.h"
#include "storage/PMSFSymbolStorageAdapter.h"
#include "PPMC/boundary/PPMCPMSFBoundary.h"
#include "rangecoder/rangecod.h"
#include "Halfedge.h"
#include "Point.h"
#include "compress/SymbolCollectOperator.h"
#include "common/FixedQueue.h"
#include <cstddef>
#include <cstdint>   // P2.a-extend：int64_t / __int128 整数累加器
#include <cstdlib>
#include <memory>
#include <unordered_map>
#include <stdexcept>
#include <vector>

class PPMCSymbolCollectOperator : public SymbolCollectOperator {
  public:
    bool preservesLiftedBaseMesh() const override { return true; }

    struct EncodedByteBreakdown {
        std::uint64_t removedConnectivityBytes = 0;
        std::uint64_t insertedConnectivityBytes = 0;
        std::uint64_t residualBytes = 0;
        std::uint64_t removedStreamBytes = 0;
        std::uint64_t insertedStreamBytes = 0;
        std::uint32_t operations = 0;

        std::uint64_t facetSymbolBytes() const {
            return removedConnectivityBytes + insertedConnectivityBytes;
        }
        std::uint64_t totalStreamBytes() const {
            return removedStreamBytes + insertedStreamBytes;
        }
    };

    PPMCSymbolCollectOperator() = default;

    void collect(MCGAL::Halfedge* seed) override;
    void collect(std::vector<MCGAL::Halfedge*> seed) override;
    int exportToBuffer(char* buffer, bool enableQuantization = false, bool enableCompress = false) override;

  public:
    // Parquet V2 多表方案需要直接访问内部队列（不经过 range coder）
    const std::deque<std::deque<std::pair<unsigned, unsigned>>>& getConnectFaceSym() const { return connectFaceSym; }
    const std::deque<std::deque<std::pair<unsigned, unsigned>>>& getConnectEdgeSym() const { return connectEdgeSym; }
    const std::deque<std::deque<MCGAL::PointInt>>& getGeometrySym() const { return geometrySym; }
    int getDecimationCount() const { return i_curDecimationId; }
    bool hasOperationData(int operationId) const;
    int exportOperationToBuffer(int operationId, char* buffer);
    const EncodedByteBreakdown& getLastEncodedByteBreakdown() const { return lastEncodedByteBreakdown_; }
    const pmsf::PMSFSymbolTable* pmsfSymbolTable() const override {
        paperTableCache_.clear();
        paperTableCache_.reserve(connectFaceSym.size());
        for (std::size_t op = 0; op < connectFaceSym.size(); ++op) {
            pmsf::PMSFSymbolRecord& row = pmsf::appendPMSFSymbolRecord(
                paperTableCache_,
                0,
                groupFilter_ >= 0 ? groupFilter_ : 0,
                static_cast<std::int32_t>(op));
            pmsf::appendPPMCPaperContext(
                row.delta,
                -1,
                static_cast<std::int32_t>(b_useConnectivityPredictionFaces ? 1 : 0),
                static_cast<std::int32_t>(b_useConnectivityPredictionEdges ? 1 : 0),
                static_cast<std::int32_t>(b_useTriangleMeshConnectivityPredictionFaces ? 1 : 0),
                static_cast<std::int32_t>(b_useCurvaturePrediction ? 1 : 0),
                0,
                static_cast<std::int32_t>(b_useLiftingScheme ? 1 : 0),
                static_cast<std::int32_t>(b_useLiftingScheme ? 1 : 0),
                static_cast<std::int32_t>(b_useAdaptiveQuantization ? 1 : 0),
                static_cast<std::int32_t>(groupFilter_));
            pmsf::appendPPMCPaperFaceSymbols(row.delta, connectFaceSym[op]);
            if (op < connectEdgeSym.size()) {
                pmsf::appendPPMCPaperEdgeSymbols(row.delta, connectEdgeSym[op]);
            }
            if (op < geometrySym.size()) {
                pmsf::appendPPMCPaperGeometrySymbols(row.delta, geometrySym[op]);
            }
        }
        return &paperTableCache_;
    }

    const pmsf::PMSFSymbolPayloadTable* paperSymbolPayloadTable() const override;

    // ⚠️ P1 对齐：把 PMSF 入口的 `options.isEnablePrediction()` 一次性映射到 PPMC
    //   的三个预测开关（曲率残差 + 面片连接性预测 + 边连接性预测）。
    //   原版 PPMC main.cpp 的默认值也是三个全开，PPMC 论文中"几何预测 + 连接性预测"
    //   正是几何 + 连接性两端 entropy 下降的主要来源。
    //
    // ⚠️ P3.b 已完成浮点语义回退（faceSurface/edgeLen/barycenter 全部 float，
    //   判据 fabs，累加器 f_avg 增量均值）。
    //
    // ⚠️ P3.c（2026-05-01 继续推进）：启用 connectivity prediction（face + edge）。
    //   编/解码两端都走浮点 fabs 判据 + 同一 BFS 顺序 → bit-identical。
    //   curvature prediction 仍关闭（P3.d 再处理 laplacian 镜像）。
    //   triangle-mesh balance 预测由入口（`setUseTriangleMeshConnectivityPrediction`）控制，
    //   PMSF.cpp 会在 bunny（纯三角）场景下调用开启。
    void setEnablePrediction(bool v) override {
        enablePrediction_ = v;
        b_useCurvaturePrediction = false;             // P3.d TODO（laplacian 镜像 + lifting）
        b_useConnectivityPredictionFaces = v;         // ✅ P3.c 启用
        b_useConnectivityPredictionEdges = v;         // ✅ P3.c 启用
    }

    /// 细粒度开关：仅在确保编/解码 faceSurface 浮点对称后才手动启用。
    void setConnectivityPredictionFaces(bool v) { b_useConnectivityPredictionFaces = v; }
    void setConnectivityPredictionEdges(bool v) { b_useConnectivityPredictionEdges = v; }
    void setCurvaturePrediction(bool v) { b_useCurvaturePrediction = v; }

    /// P1 对齐：只有当输入是"纯三角网格"时才启用 triangle-mesh 连接性预测分支
    ///   （neighborBalance 预测）。原版 PPMC 在 `mymesh.cpp` 构造时检测 is_pure_triangle()
    ///   来决定是否开启。PMSF 入口在调用 collect 前可检查 mesh 的所有 facet degree 并传入。
    void setUseTriangleMeshConnectivityPrediction(bool v) {
        b_useTriangleMeshConnectivityPredictionFaces = v;
    }

    /// P3.e 对齐：启用 lifting scheme（原版 PPMC mymeshLifting.cpp）
    void setUseLiftingScheme(bool v) { b_useLiftingScheme = v; }
    void setGroupFilter(int groupId) { groupFilter_ = groupId; }

  private:
    bool inGroup(MCGAL::Halfedge* h) const;
    MCGAL::Halfedge* findNextUnprocessedFaceSeed() const;
    MCGAL::Halfedge* findNextUnprocessedHalfedgeSeed() const;
    void RemovedVertexCodingStep();
    void InsertedEdgeCodingStep();
    // ⚠️ P0 对齐：与原版 PPMC `mymeshComp.cpp::determineResiduals()` 对齐。
    //   在 RemovedVertexCodingStep 之前以同样的 BFS 顺序遍历每个 splittable face，
    //   计算 residual = Q(removedVertexPos) - Q(barycenter(face))，写入
    //   facet 的 RemovedVertexPosInt。后续 determineGeometrySym 直接读取该残差。
    //   不做这步的话 alphaBeta 范围会爆炸（绝对量化坐标 vs 小残差），range coder 失效。
    void determineResiduals();
    void encodeRemovedVertices(unsigned i_operationId);
    void encodeInsertedEdges(unsigned i_operationId);
    int exportOperationToSplitZlibBuffer(int operationId, char* buffer);
    void collectHalfedgeSymbol(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Halfedge* seed);
    void collectFacetSymbol(std::shared_ptr<MCGAL::Mesh> mesh, MCGAL::Halfedge* seed);
    // ⚠️ P2.a-extend 对齐：`updateAvgSurfaces` / `updateAvgEdgeLen` 改为纯整数累加
    //   接口——直接把单 face/edge 的 int64 几何度量加到 __int128 累加和上，不再
    //   计算浮点均值。"谁更接近均值"的判据通过整数交叉相乘完成（见 RemovedVertexCodingStep）。
    // ⚠️ P3.b 对齐（2026-05-01）：重新回退到原版 PPMC 的浮点增量均值。整数化方案
    //   经 P2.b 实测在启用 prediction 后仍有 1-sym 偏移（根因是 MCGAL/CGAL 底层
    //   非对称性，见 PPMC_P3_HANDOFF.md §6.2）。由于原版 PPMC 编/解码两端走完全
    //   相同浮点代码，浮点语义的 bit-identical 保障在实现上反而更稳。
    void updateAvgSurfaces(bool b_split, float f_faceSurface);
    void determineGeometrySym(MCGAL::Halfedge* heh_gate, MCGAL::Facet* fh);
    void updateAvgEdgeLen(bool b_original, float f_edgeLen);
    MCGAL::PointInt avgLaplacianVect(MCGAL::Halfedge* heh_gate);
    void lift(bool b_unlift);

    // ⚠️ P2.a 对齐：PPMC 开启 connectivity prediction 后，每轮 BFS 内"重复入队"机制
    //   会让 gateQueue 峰值比纯 ratio-coder 路径更高（预测分支读取 `isProcessed` 导致
    //   某些 halfedge 在 pop 前多次 push）。bunny (34k 顶点) 量级观测到超过 30000 的峰值，
    //   因此这里预留 300000 容量，与 GreedySelector 持平（覆盖 ≤ 300k 顶点的 mesh）。
    FixedQueue<int> gateQueue = FixedQueue<int>(300000);

    // ⚠️ P2.a-extend 对齐：彻底转为 __int128 纯整数累加器，消除编/解码两端浮点累加
    //   顺序差异对预测判据的影响。
    // ⚠️ P3.b 对齐（2026-05-01）：回退到原版 PPMC 浮点增量均值。
    //   `f_avgSurfaceFace*` / `f_avgOriginalEdgesLength` / `f_avgInsertedEdgesLength`
    //   按 PPMC 论文原版语义：`avg = (avg*(n-1) + f)/n`。
    //   编/解码两端走同一 BFS 顺序 + 同一浮点代码 → 两端累加顺序同源，结果 bit-identical。
    float f_avgSurfaceFaceWithoutCenterRemoved;
    float f_avgSurfaceFaceWithCenterRemoved;

    float f_avgInsertedEdgesLength;
    float f_avgOriginalEdgesLength;

    unsigned i_nbFacesWithoutCenterRemoved;
    unsigned i_nbFacesWithCenterRemoved;

    unsigned i_nbInsertedEdges;
    unsigned i_nbOriginalEdges;
    unsigned i_nbGoodPredictions = 0;

    bool b_predictionUsed = false;

    // Codec features status.
    bool b_useAdaptiveQuantization = false;
    bool b_useLiftingScheme = false;
    bool b_useCurvaturePrediction = false;
    bool b_useConnectivityPredictionFaces = false;
    bool b_useConnectivityPredictionEdges = false;
    bool b_useTriangleMeshConnectivityPredictionFaces = false;

    int i_curDecimationId = 0;
    rangecoder rangeCoder;
    int alphaBetaMin;
    // ⚠️ P0 对齐：USE_BIJECTION 开启后 γ 分量用独立的 qsmodel(gammaModel)，
    //   range coder 编码时需要单独写 `gammaMin / gammaRange`，因此必须有此成员。
    //   原版 PPMC `mymesh.h:516` 即为 `int alphaBetaMin, gammaMin`；复现头文件
    //   之前缺失 gammaMin → 编译时报 "use of undeclared identifier 'gammaMin'"。
    int gammaMin = 0;
    int groupFilter_ = -1;
    EncodedByteBreakdown lastEncodedByteBreakdown_;

    // Geometry symbol list.
    std::deque<std::deque<MCGAL::PointInt>> geometrySym;

    // Connectivity symbol list.
    std::deque<std::deque<std::pair<unsigned, unsigned>>> connectFaceSym;
    std::deque<std::deque<std::pair<unsigned, unsigned>>> connectEdgeSym;
    std::deque<unsigned> facesConnectPredictionUsed;
    std::deque<unsigned> edgesConnectPredictionUsed;
    mutable pmsf::PMSFSymbolTable paperTableCache_;
    mutable pmsf::PMSFSymbolPayloadTable paperPayloadTableCache_;
    mutable int paperPayloadTableCacheDecimationId_ = -1;
    mutable int paperPayloadTableCacheGroupFilter_ = -2147483647;

    MCGAL::Halfedge* seed = nullptr;
    std::vector<MCGAL::Facet*> boundaryScanFacets_;

    /// @brief 曲率预测的确定性「每轮 1-ring 聚合缓存」。
    ///
    /// 语义等价于原 unordered_map<Vertex*, (neighbours,count)> 版本，但按 poolId
    /// 平铺成裸数组：Laplacian 只需要每个顶点的**邻居位置之和**与**非 new 半边计数**
    /// （整数加法顺序无关），因此每轮一遍 O(Σdeg) 即可，避免数千次 hash/vector
    /// 分配（实测 horse19k 曲率 Laplacian 140ms → 几 ms，码流逐字节不变）。
    std::vector<int> curvSumQX_, curvSumQY_, curvSumQZ_;  // Σ Q(neighbour)
    std::vector<int> curvCnt_;                            // non-new ring halfedge count
    std::vector<int> curvPosQX_, curvPosQY_, curvPosQZ_;  // Q(center) cache for queries
    bool curvatureNeighborCacheValid_ = false;
    void resetCurvatureNeighborCache() { curvatureNeighborCacheValid_ = false; }
};

#endif