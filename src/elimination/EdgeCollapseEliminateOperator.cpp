/*
 * @Author: koi
 * @Date: 2025-08-27 22:37:20
 * @Description: 真正的 edge-collapse 操作单元：把 h 的 end_vertex 合并到 vertex，
 *               拓扑由 Mesh::halfedge_collapse 维护。供 QEM 等需要严格 edge-contraction
 *               语义的算法使用（VertexRemovalEliminateOperator 是另一种"删一邻域"路径）。
 */
#include "EdgeCollapseEliminateOperator.h"
#include "Facet.h"
#include "Mesh.h"
#include "Vertex.h"

using namespace MCGAL;

bool EdgeCollapseEliminateOperator::eliminate(HalfedgeErrorEntry& entry) {
    if (entry.edge == nullptr)
        return false;
    // ⚠️ QEM/edge-contraction 路径：先把 v0 移到 entry.p（QEM 算的最优合并点），
    //   再触发 halfedge_collapse 把 v1 合并到 v0。
    if (header_.isEnableAdjustment()) {
        entry.edge->vertex()->setAdjusted();
        // 注意：这里与 VertexRemovalEliminateOperator 一致，先存"未调整时位置"
        //       供解码端 BFS 对齐，再覆盖为 entry.p。
        entry.edge->vertex()->setVertexPoint(entry.edge->vertex()->point());
        entry.edge->vertex()->setPoint(entry.p);
    }
    if (!remove_point(entry.edge)) return false;
    triangulate(entry.edge);
    postprocess(entry.edge);
    encode_boundary(entry.edge);
    return true;
}

bool EdgeCollapseEliminateOperator::remove_point(MCGAL::Halfedge* h) {
    if (h == nullptr) return false;
    Vertex* v1 = h->vertex();
    Vertex* v2 = h->end_vertex();
    if (v1 == nullptr || v2 == nullptr || v1->isRemoved() || v2->isRemoved())
        return false;

    if (mesh_ == nullptr) return false;
    // 链路安全检查：避免合并产生非流形拓扑（"bow-tie"），
    // new_is_collapse_ok 实现了 OpenMesh 风格的 link condition 检查。
    if (!mesh_->new_is_collapse_ok(h)) return false;

    // Mesh::new_collapse(h)：把 v2 合并到 v1，移除 h/h->opposite() 及两侧塌缩面，
    // 同步重指 v2 的所有半边到 v1。v1 的位置已在 eliminate() 中设为 entry.p。
    mesh_->new_collapse(h);
    return true;
}

bool EdgeCollapseEliminateOperator::triangulate(MCGAL::Halfedge* h) {
    // halfedge_collapse 直接维护三角拓扑，无需再三角化
    return true;
}

bool EdgeCollapseEliminateOperator::postprocess(MCGAL::Halfedge* h) {
    return true;
}

MCGAL::Halfedge* EdgeCollapseEliminateOperator::encode_boundary(MCGAL::Halfedge* h) {
    return nullptr;
}
