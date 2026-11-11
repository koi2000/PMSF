#include "common/DebugTool.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "MeshUtils.h"
#include "PPMCReconstructOperator.h"
#include "PerformedOperation.h"
#include <cassert>
#include <cstddef>
#include <cstdlib>
#include <utility>
#include <vector>

namespace {
}  // namespace

// ═════════════════════════════════════════════════════════════════════════════
// ⚠️ P3.c 状态（2026-05-02）：
// ─────────────────────────────────────────────────────────────────────────────
//
// P3.b 浮点语义回退已完成（详见下方 §P3.b 备注）。P3.c 本轮启用 connectivity
// prediction（face + edge）并完成以下调整：
//
//   1. `PPMCSymbolCollectOperator::setEnablePrediction(v)`：
//      - `b_useConnectivityPredictionFaces = v`, `b_useConnectivityPredictionEdges = v`
//      - `b_useCurvaturePrediction = false`（P3.d 再启用）
//      对称地更新 `PPMCSymbolReadOperator::setEnablePrediction(v)`。
//
//   2. `PMSF::ppmcCompress` / `PMSF::ppmcColumnarCompress`：
//      **只有纯三角网格** 才实际启用 prediction（`usePred = options.isEnablePrediction() && pureTriangle`）。
//      非三角（LetterA 等 n-gon）mesh 启用 prediction 会在 round 6+ 触发 stream
//      漂移 → alphaBetaRange 读出 0 → `initqsmodel` SIGFPE。原版 PPMC (`mymesh.cpp`
//      构造) 同样只在 `is_pure_triangle()` 时开 prediction。`header.setEnablePrediction`
//      同步到实际决策（而非用户意图）。
//
//   3. Seed refresh（每轮 v0→v1 重查找，等价原版 `pushHehInit`）：
//      - `PPMCDeserializeOperator::deserializeOneRound` 从 `seedV0/seedV1` 倒序查
//        `end_vertex==seedV1 && !isRemoved()` 的最新 halfedge。
//      - `ColumnarDeserializeOperator::deserializeOneRound`（v2 路径）同步加 refresh。
//      - 实测 bunny 在 round 4 后 v0→v1 halfedge 被 reconstruct 的 `join_face`
//        标记 removed；此时保留 stale seed 仍可跑（seed->face() 存活，BFS 起点
//        合法），refresh 失败时**静默 fallback 到 stale seed**。
//
//   4. Seed halfedge guard（强制 seed 永远 b_original=true）：**已尝试并禁用**。
//      实测启用后会破坏 `b_predictionUsed` 判据在编/解码两端的同步，让解码端
//      第 9 轮附近读出 alphaBetaRange=0 → SIGFPE。当前配置下（pureTriangle +
//      prediction + refresh）guard 并非正确性前提，故留 `// if (h == seed...)`
//      注释占位，等 P3.d 引入 lifting 后再评估。
//
// 回归结果（bench_columnar / bench_binary，n=1）：
//   | 数据集   | 路径        | 状态 | BPV   | P3.b 基准 | 降幅   |
//   |----------|-------------|------|-------|-----------|--------|
//   | bunny    | RC          | ✅   | 18.11 | 19.21     | -1.10  |
//   | bunny    | Parquet V2  | ✅   | 21.29 | 22.39     | -1.10  |
//   | LetterA  | Parquet V2  | ✅   | 69.12 | 69.13     | ±0 (pureTriangle=false，auto-off) |
//   | LetterA  | RC          | ❌   | —     | —         | pre-existing NB_BITS_FACE_DEGREE_BASE_MESH |
//
// 非 PPMC 方法（aptm/default/mmse/pcatm/pclttm/pomar/qem/rd/vqmmc）全部回归通过。
//
// 距论文 bunny RC ≤16.5 还差 ~1.6 bpv，需 P3.d（curvature prediction + lifting scheme）补齐。
// ═════════════════════════════════════════════════════════════════════════════

// ═════════════════════════════════════════════════════════════════════════════
// ⚠️ P3.b 状态（2026-05-01）：
// ─────────────────────────────────────────────────────────────────────────────
//
// P3.b 浮点语义回退完成：
//   - `deps/MCGAL/include/MeshUtils.h` 新增 `edgeLen(heh)`（与原版 MyMesh::edgeLen 同源）
//     并保留原有浮点 `faceSurface` / `barycenter`；整数化 helper（`faceSurfaceSqQuant`、
//     `edgeLenSqQuant`、`barycenterIntQuant`、`quantizedSquaredDistance`）保留作参考，
//     不再由 PPMC 编/解码路径调用。
//   - `PPMCSymbolCollectOperator` / `PPMCSymbolReadOperator` 的累加器回到原版
//     `f_avgSurfaceFace*` / `f_avgInsertedEdgesLength` / `f_avgOriginalEdgesLength`
//     浮点增量均值；判据回到原版 `fabs(f - avg_A) <= fabs(f - avg_B)` 浮点比较
//     （见 namespace `predictSplitByAreaFloat` / `predictOriginalByLenFloat` /
//     `faceClassRatiosClose`）。
//   - `determineResiduals` / `decodeGeometrySym` / `collectFromDecoded` 里的
//     `Q(barycenter)` 全部改回 `floatPosToInt(MCGAL::barycenter(h))` —— 与原版
//     `CGAL::ORIGIN + Σv/d` 浮点累加语义同源。Parquet V2 路径（collectFromDecoded）
//     也同步改回，避免与 RC 路径走不同 barycenter 公式。
//   - 编/解码两侧 BFS 循环加上 cycle-guard，首次断裂立即 abort + 打印 poolId，
//     便于定位 merge_face 错位的起点（本次 P3.b 在 bunny/LetterA Parquet V2 上
//     未触发 guard，说明浮点语义下拓扑稳定）。
//
// 回归结果（bench_columnar / bench_binary，n=1）：
//   | 数据集   | 路径        | 状态 | BPV   | P2.b 基准 | 偏差    |
//   |----------|-------------|------|-------|-----------|---------|
//   | bunny    | RC          | ✅   | 19.21 | 19.19     | +0.02   |
//   | bunny    | Parquet V2  | ✅   | 22.39 | 22.23     | +0.16   |
//   | LetterA  | Parquet V2  | ✅   | 69.13 | 61.57     | +7.56*  |
//   | LetterA  | RC          | ❌   | —     | 34.27     | pre-existing |
//
//   * LetterA PQ V2 BPV 上升的根因：原整数 second moment 对 polygon face 起点
//     敏感度更低（误判更多 → 预测关闭也能混合 smoothing），浮点 fan Heron 真实
//     反映 polygon 面积使得预测方差更大；prediction 启用后 BPV 有望降回去。
//   * LetterA RC 失败是预先存在的 `NB_BITS_FACE_DEGREE_BASE_MESH=3` 对 base-mesh
//     face degree>10 不兼容的 assertion（见 PPMCSerializeOperator.cpp:124），与
//     P3.b 无关。
//
// prediction 默认仍关闭（`setEnablePrediction` 硬编码 false），P3.c 再启用。
// ═════════════════════════════════════════════════════════════════════════════

void PPMCReconstructOperator::reconstruct(MCGAL::Halfedge* halfedge) {}

void PPMCReconstructOperator::reconstruct(std::vector<MCGAL::Vertex*>& vertices,
                                          std::vector<MCGAL::Halfedge*>& halfedge,
                                          std::vector<MCGAL::Facet*>& facets) {
    (void)halfedge;
    for (MCGAL::Vertex* v : vertices) {
        v->setPoint(v->vertexPoint());
    }
    for (MCGAL::Facet* f : facets) {
        insert_point(f);
    }

    std::vector<MCGAL::Halfedge*> addedHalfedges;
    for (MCGAL::Facet* f : mesh_->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        MCGAL::Halfedge* h = f->proxyHalfedge();
        MCGAL::Halfedge* start = h;
        int guard = 0;
        do {
            if (h && !h->isRemoved() && h->isAdded()) {
                addedHalfedges.push_back(h);
            }
            h = h ? h->next() : nullptr;
        } while (h && h != start && ++guard <= 2048);
    }

    long joined = 0;
    for (MCGAL::Halfedge* hit : addedHalfedges) {
        if (!hit || hit->isRemoved()) continue;
        merge_face(hit);
        ++joined;
    }
}

void PPMCReconstructOperator::merge_face(MCGAL::Halfedge* halfedge) {
    if (halfedge->isRemoved()) {
        return;
    }
    MCGAL::join_face(halfedge);
}

void PPMCReconstructOperator::insert_point(MCGAL::Facet* facet) {
    MCGAL::PointInt baryQ = mesh_->floatPosToInt(MCGAL::barycenter(facet->proxyHalfedge()));
    MCGAL::Point p = mesh_->intPosToFloat(baryQ + facet->getRemovedVertexPosInt());
        // mesh_->dumpto_oldtype("./decode/test1.off");
    MCGAL::Halfedge* hnew = mesh_->create_center_vertex_without_init(facet->proxyHalfedge(), p);
    // mesh_->dumpto_oldtype("./decode/test2.off");
    if (!hnew || !hnew->vertex()) return;
    if (hnew->vertex()->proxyHalfedge() == nullptr) {
        hnew->vertex()->setProxyHalfedge(hnew);
    }
    const bool fillCenterHalfedgeList = hnew->vertex()->halfedges().empty();
    MCGAL::Halfedge* hit = hnew;
    int guard = 0;
    do {
        hit->setNew();
        if (hit->opposite()) {
            hit->opposite()->setNew();
        }
        if (fillCenterHalfedgeList) {
            hnew->vertex()->halfedges().push_back(hit);
        }
        hit = hit->opposite() ? hit->opposite()->next() : nullptr;
    } while (hit && hit != hnew && ++guard <= 2048);
}
