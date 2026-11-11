/*****************************************************************************
 * @Author: koi
 * @Date: 2025-08-27 22:37:03
 * @Description: PCATM 消除操作实现。继承 VertexRemovalEliminateOperator 的通用骨架，
 *               只保留 PCATM 论文特有的残差计算与三角化策略。
 *****************************************************************************/
#include "PCATM/elimination/PCATMEliminateOperator.h"
#include "Facet.h"
#include "Global.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "MeshUtils.h"
#include "Point.h"
#include "Vertex.h"
#include "common/Define.h"
#include <vector>

namespace {

inline int polygonFaceSize(MCGAL::Facet* f) {
    if (!f) return 0;
    int n = 0;
    MCGAL::Halfedge* st = f->proxyHalfedge();
    MCGAL::Halfedge* it = st;
    if (!it) return 0;
    do {
        ++n;
        it = it->next();
    } while (it != st && it != nullptr);
    return n;
}

}  // namespace

int PCATMEliminateOperator::computeRemovedValence(MCGAL::Halfedge* /*h*/,
                                                  MCGAL::Vertex* /*removedVertex*/) {
    // PCATM 在 erase_center_vertex 后通过 polygon loop 数边数，避免依赖
    // 可能 dirty 的 vertex_degree()。
    return 0;  // 实际值在 computeRemovedVertexResidualInt 中一并写入 face。
}

MCGAL::Vector3i PCATMEliminateOperator::computeRemovedVertexResidualInt(
    MCGAL::Halfedge* /*h*/,
    MCGAL::Halfedge* hface,
    const MCGAL::Point& /*vrPoint*/,
    const MCGAL::PointInt& vrPos) {
    MCGAL::Facet* polyFace = hface->face();
    const int fsize = polygonFaceSize(polyFace);
    polyFace->setPolyValence(fsize);

    // PCATM 使用简单 barycentric 整数预测（论文 §3）。
    MCGAL::PointInt predPos = mesh_->floatPosToInt(
        MCGAL::barycenter(polyFace->proxyHalfedge()));
    return MCGAL::Vector3i(vrPos.x() - predPos.x(),
                           vrPos.y() - predPos.y(),
                           vrPos.z() - predPos.z());
}

bool PCATMEliminateOperator::triangulate(MCGAL::Halfedge* h) {
    if (!h)
        return false;
    int groupId = h->face()->groupId();
    MCGAL::Facet* polyFace = h->face();

    std::vector<MCGAL::Vertex*> polyVerts;
    std::vector<MCGAL::Halfedge*> polyHalfedges;
    {
        MCGAL::Halfedge* st = polyFace->proxyHalfedge();
        MCGAL::Halfedge* ed = st;
        MCGAL::Halfedge* it = st;
        polyVerts.reserve(8);
        polyHalfedges.reserve(8);
        do {
            polyHalfedges.push_back(it);
            polyVerts.push_back(it->vertex());
            it = it->next();
        } while (it != ed && it != nullptr);
    }
    const int n = static_cast<int>(polyVerts.size());

    if (n < 4) {
        h->face()->setSplittable();
        return true;
    }
    h->face()->setSplittable();

    if (n == 5) {
        return triangulate_degree_5(polyFace, polyVerts, polyHalfedges, groupId);
    } else if (n == 6) {
        return triangulate_degree_6(polyFace, polyVerts, polyHalfedges, groupId);
    }
    return triangulate_general(polyFace, polyVerts, polyHalfedges, groupId);
}

bool PCATMEliminateOperator::triangulate_degree_5(
    MCGAL::Facet* polyFace,
    const std::vector<MCGAL::Vertex*>& polyVerts,
    const std::vector<MCGAL::Halfedge*>& polyHalfedges,
    int groupId) {
    MCGAL::Halfedge* p1 = nullptr;
    MCGAL::Halfedge* p2 = nullptr;

    {
        MCGAL::Halfedge* h0 = polyHalfedges[0];
        MCGAL::Halfedge* h1 = polyHalfedges[1];
        MCGAL::Halfedge* hnew = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(
            mesh_->meshId(), polyVerts[2], polyVerts[0]);
        p1 = hnew;
        hnew->setAdded();
        h0->setNext(h1);
        h1->setNext(hnew);
        hnew->setNext(h0);
        polyFace->reset(h0);
        polyFace->setTriangleColor(MCGAL::Facet::TriangleColorFlag::RED);
    }

    {
        MCGAL::Halfedge* h0 = polyHalfedges[2];
        MCGAL::Halfedge* h1 = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(
            mesh_->meshId(), polyVerts[3], polyVerts[0]);
        MCGAL::Halfedge* h2 = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(
            mesh_->meshId(), polyVerts[0], polyVerts[2]);
        h2->setOpposite(p1);
        p1->setOpposite(h2);
        p2 = h1;

        h1->setAdded();
        h2->setAdded();
        h0->setNext(h1);
        h1->setNext(h2);
        h2->setNext(h0);
        MCGAL::Facet* newFace = MCGAL::contextPool.allocateFaceFromPool(mesh_->meshId(), h0);
        newFace->setGroupId(groupId);
        newFace->setTriangleColor(MCGAL::Facet::TriangleColorFlag::BLUE);
        mesh_->add_face(newFace);
    }

    {
        MCGAL::Halfedge* h0 = polyHalfedges[3];
        MCGAL::Halfedge* h1 = polyHalfedges[4];
        MCGAL::Halfedge* hnew = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(
            mesh_->meshId(), polyVerts[0], polyVerts[3]);
        hnew->setOpposite(p2);
        p2->setOpposite(hnew);
        hnew->setAdded();
        h0->setNext(h1);
        h1->setNext(hnew);
        hnew->setNext(h0);
        MCGAL::Facet* newFace = MCGAL::contextPool.allocateFaceFromPool(mesh_->meshId(), h0);
        newFace->setGroupId(groupId);
        newFace->setTriangleColor(MCGAL::Facet::TriangleColorFlag::RED);
        mesh_->add_face(newFace);
    }

    return true;
}

bool PCATMEliminateOperator::triangulate_degree_6(
    MCGAL::Facet* polyFace,
    const std::vector<MCGAL::Vertex*>& polyVerts,
    const std::vector<MCGAL::Halfedge*>& polyHalfedges,
    int groupId) {
    MCGAL::Halfedge* p1 = nullptr;
    MCGAL::Halfedge* p2 = nullptr;

    {
        MCGAL::Halfedge* h0 = polyHalfedges[0];
        MCGAL::Halfedge* h1 = polyHalfedges[1];
        MCGAL::Halfedge* hnew = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(
            mesh_->meshId(), polyVerts[2], polyVerts[0]);
        p1 = hnew;
        hnew->setAdded();
        h0->setNext(h1);
        h1->setNext(hnew);
        hnew->setNext(h0);
        polyFace->reset(h0);
        polyFace->setTriangleColor(MCGAL::Facet::TriangleColorFlag::RED);
    }

    {
        MCGAL::Halfedge* h0 = polyHalfedges[2];
        MCGAL::Halfedge* h1 = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(
            mesh_->meshId(), polyVerts[3], polyVerts[0]);
        MCGAL::Halfedge* h2 = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(
            mesh_->meshId(), polyVerts[0], polyVerts[2]);
        h2->setOpposite(p1);
        p1->setOpposite(h2);
        p2 = h1;

        h1->setAdded();
        h2->setAdded();
        h0->setNext(h1);
        h1->setNext(h2);
        h2->setNext(h0);
        MCGAL::Facet* newFace = MCGAL::contextPool.allocateFaceFromPool(mesh_->meshId(), h0);
        newFace->setGroupId(groupId);
        newFace->setTriangleColor(MCGAL::Facet::TriangleColorFlag::BLUE);
        mesh_->add_face(newFace);
    }

    {
        MCGAL::Halfedge* h0 = polyHalfedges[3];
        MCGAL::Halfedge* h1 = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(
            mesh_->meshId(), polyVerts[4], polyVerts[0]);
        MCGAL::Halfedge* h2 = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(
            mesh_->meshId(), polyVerts[0], polyVerts[3]);
        h2->setOpposite(p2);
        p2->setOpposite(h2);
        p1 = h1;

        h1->setAdded();
        h2->setAdded();
        h0->setNext(h1);
        h1->setNext(h2);
        h2->setNext(h0);
        MCGAL::Facet* newFace = MCGAL::contextPool.allocateFaceFromPool(mesh_->meshId(), h0);
        newFace->setGroupId(groupId);
        newFace->setTriangleColor(MCGAL::Facet::TriangleColorFlag::RED);
        mesh_->add_face(newFace);
    }

    {
        MCGAL::Halfedge* h0 = polyHalfedges[4];
        MCGAL::Halfedge* h1 = polyHalfedges[5];
        MCGAL::Halfedge* hnew = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(
            mesh_->meshId(), polyVerts[0], polyVerts[4]);
        hnew->setOpposite(p1);
        p1->setOpposite(hnew);
        hnew->setAdded();
        h0->setNext(h1);
        h1->setNext(hnew);
        hnew->setNext(h0);
        MCGAL::Facet* newFace = MCGAL::contextPool.allocateFaceFromPool(mesh_->meshId(), h0);
        newFace->setGroupId(groupId);
        newFace->setTriangleColor(MCGAL::Facet::TriangleColorFlag::BLUE);
        mesh_->add_face(newFace);
    }

    return true;
}

bool PCATMEliminateOperator::triangulate_general(
    MCGAL::Facet* polyFace,
    const std::vector<MCGAL::Vertex*>& polyVerts,
    const std::vector<MCGAL::Halfedge*>& polyHalfedges,
    int groupId) {
    const int n = static_cast<int>(polyVerts.size());
    MCGAL::Halfedge* prevDiag = nullptr;

    for (int i = 0; i < n - 2; ++i) {
        MCGAL::Facet::TriangleColorFlag color =
            (i % 2 == 0) ? MCGAL::Facet::TriangleColorFlag::RED
                         : MCGAL::Facet::TriangleColorFlag::BLUE;

        if (i == 0) {
            MCGAL::Halfedge* h0 = polyHalfedges[0];
            MCGAL::Halfedge* h1 = polyHalfedges[1];
            MCGAL::Halfedge* hnew = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(
                mesh_->meshId(), polyVerts[2], polyVerts[0]);
            hnew->setAdded();
            h0->setNext(h1);
            h1->setNext(hnew);
            hnew->setNext(h0);
            polyFace->reset(h0);
            polyFace->setTriangleColor(color);
            prevDiag = hnew;
        } else if (i == n - 3) {
            MCGAL::Halfedge* h0 = polyHalfedges[n - 2];
            MCGAL::Halfedge* h1 = polyHalfedges[n - 1];
            MCGAL::Halfedge* hnew = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(
                mesh_->meshId(), polyVerts[0], polyVerts[n - 2]);
            hnew->setOpposite(prevDiag);
            prevDiag->setOpposite(hnew);
            hnew->setAdded();
            h0->setNext(h1);
            h1->setNext(hnew);
            hnew->setNext(h0);
            MCGAL::Facet* newFace = MCGAL::contextPool.allocateFaceFromPool(mesh_->meshId(), h0);
            newFace->setGroupId(groupId);
            newFace->setTriangleColor(color);
            mesh_->add_face(newFace);
        } else {
            MCGAL::Halfedge* h0 = polyHalfedges[i + 1];
            MCGAL::Halfedge* h1 = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(
                mesh_->meshId(), polyVerts[i + 2], polyVerts[0]);
            MCGAL::Halfedge* h2 = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(
                mesh_->meshId(), polyVerts[0], polyVerts[i + 1]);
            h2->setOpposite(prevDiag);
            prevDiag->setOpposite(h2);

            h1->setAdded();
            h2->setAdded();
            h0->setNext(h1);
            h1->setNext(h2);
            h2->setNext(h0);
            MCGAL::Facet* newFace = MCGAL::contextPool.allocateFaceFromPool(mesh_->meshId(), h0);
            newFace->setGroupId(groupId);
            newFace->setTriangleColor(color);
            mesh_->add_face(newFace);
            prevDiag = h1;
        }
    }

    return true;
}
