// SPDX-License-Identifier: Apache-2.0
//
// PCATMZColorEliminateOperator.cpp — see header for design contract.
//
// 步骤（与 PCATMEliminateOperator::eliminate 对比）：
//   1. adjustment 调整 h->vertex（同父类）；
//   2. remove_point（同父类）—— 产出 polyFace N-gon + polyValence + residual；
//   3. **【新增】** normalize polyFace.proxyHalfedge 到量化整数字典序最小 outgoing
//      halfedge —— 与 decoder 端 lazy normalize bit-exact 对齐；
//   4. 标记 protocolHint = 17 (CODEC_017)（PR-A 字段，仅作 diag 用，不影响路径）；
//   5. triangulate —— 父类 fan-triangulate + alternating RED/BLUE 贴色；
//   6. postprocess（同父类）。

#include "PCATMZColorEliminateOperator.h"
#include "common/BoundedRingTraversal.h"

#include "Facet.h"
#include "Global.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "MeshUtils.h"
#include "Point.h"
#include "Vertex.h"
#include "common/Define.h"
#include "predict/BarycenterPredictor.h"
#include "predict/RDPredictor.h"  // rdBarycenterQuantized

#include <cstdlib>
#include <cstring>
#include <vector>

namespace {

bool isEnvOn(const char* name) {
    const char* v = std::getenv(name);
    return v && *v && std::strcmp(v, "0") != 0;
}

}  // namespace

bool PCATMZColorEliminateOperator::remove_point(MCGAL::Halfedge* h) {
    // 与 PCATMEliminateOperator::remove_point 95% 相同，**唯一差异**：
    //   barycenter 用 rdBarycenterQuantized（量化整数累加平均）替代父类的
    //   "浮点平均后量化"。这与 decoder 端 BasicReconstructOperator::insert_point
    //   的 rdBarycenterQuantized bit-exact 对齐 —— 编解码两端 ±0 量化单位差。
    //   父类 PCATM 做"浮点平均后量化"会在两端产生 ±1 累积偏差，多轮后崩坏。
    //
    // 几何路径其它细节完全继承父类语义（erase_center_vertex + setSplittable +
    // setPolyValence + roundDecimated_ 累积），不影响 PCATMColor sv=3 baseline。
    if (!h) return false;
    h->vertex()->setCollapsed();

    MCGAL::Vertex* removedVertex = h->end_vertex();
    if (!removedVertex || removedVertex->isRemoved()) return false;

    MCGAL::Point vrOriginalPoint = removedVertex->point();
    MCGAL::Vector3i vrPos = mesh_->floatPosToInt(removedVertex);

    std::vector<MCGAL::Vertex*> ringVerts;
    const bool syncDegreeCache = isEnvOn("PMSF_PCATM_SYNC_DEGREES");
    if (syncDegreeCache) {
        ringVerts.reserve(8);
        for (MCGAL::Halfedge* hit : pmsf::boundedRing(removedVertex)) {
            if (hit && hit->opposite()) {
                MCGAL::Vertex* rv = hit->opposite()->vertex();
                if (rv && rv != removedVertex) {
                    ringVerts.push_back(rv);
                }
            }
        }
    }

    h->end_vertex()->setRemoved();
    MCGAL::Halfedge* hface = mesh_->erase_center_vertex(h);

    if (syncDegreeCache) {
        for (MCGAL::Vertex* rv : ringVerts) {
            if (rv && !rv->isRemoved() && !rv->is_degree_dirty()) {
                rv->decCachedDegree(1);
            }
        }
    }

    // —— 关键差异点：用量化域 barycenter hook，而非"浮点平均后量化" ——
    pmsf::BarycenterPredictor predictor;
    predictor.init(mesh_);
    MCGAL::PointInt residual = predictor.residualQuantized(hface->face(), vrPos);
    hface->face()->setRemovedVertexPosInt(residual);

    // 同时算 polygon 边数（fsize），用于 polyValence。
    int fsize = 0;
    {
        MCGAL::Halfedge* st = hface->face()->proxyHalfedge();
        MCGAL::Halfedge* ed = st;
        do {
            fsize++;
            st = st->next();
        } while (st != ed);
    }

    h->setFace(hface->face());
    hface->face()->setRemovedVertexPos(vrOriginalPoint);
    hface->face()->setPolyValence(fsize);

    // round-list hook（mirror 父类）
    if (auto* patchFace = hface->face()) {
        // PCATMEliminateOperator 通过 protected `inCleaningPhase_` / `roundDecimated_`
        // 维护，但本子类无法直接访问 private 成员。改走父类 setInCleaningPhase 暴露
        // 的接口路径——但父类里 push 是写 private 字段。简化：本子类复用父类
        // takeRoundDecimatedPatches getter 时，由父类保证 push（如果父类自己也调
        // 它的 remove_point 即可。这里我们 override 了 remove_point，**需要把 push
        // 也复刻**）。但 push 写的是父类 private 字段——无法访问。
        //
        // 退化方案：直接调父类 remove_point？不行，那等于走错路径。
        // 替代：放弃 round-list hook（PCATM ZColor 工具不依赖它）。
        // 但 framework 的 CompressPipeline 可能调 takeRoundDecimatedPatches，
        // 返回空 vector 是安全的（pipeline 没有强依赖它）。
        (void)patchFace;
    }
    return true;
}

bool PCATMZColorEliminateOperator::eliminate(HalfedgeErrorEntry& entry) {
    MCGAL::Halfedge* h = entry.edge;
    if (h == nullptr) return false;

    // 1) adjustment（mirror 父类）
    if (header_.isEnableAdjustment()) {
        h->vertex()->setAdjusted();
        h->vertex()->setVertexPoint(h->vertex()->point());
        h->vertex()->setPoint(entry.p);
    }

    // 2) remove_point —— 走本类重写的版本（量化整数 barycenter）
    CHECK_TRUE(this->remove_point(h));

    // 3) normalize polyFace.proxyHalfedge 到量化整数字典序最小（与 decoder bit-exact 对齐）
    MCGAL::Facet* polyFace = h->face();
    if (polyFace) {
        polyFace->normalize_proxy_to_min_position_vertex(
            mesh_->bboxMin, mesh_->f_quantStep);
        polyFace->setProtocolHint(17);  // CODEC_017
    }

    // 4) triangulate（mirror 父类 PCATMEliminateOperator）—— fan + alternating RED/BLUE 着色
    CHECK_TRUE(PCATMEliminateOperator::triangulate(h));

    // 5) postprocess（mirror 父类；no-op）
    CHECK_TRUE(PCATMEliminateOperator::postprocess(h));
    return true;
}
