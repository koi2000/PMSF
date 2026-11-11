/*
 * @Author: koi
 * @Date: 2025-08-27 22:37:03
 * @Description:
 */
#include "BoundaryVertexRemovalEliminateOperator.h"
#include "common/BoundedRingTraversal.h"
#include "Facet.h"
#include "Global.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "MeshUtils.h"
#include "Point.h"
#include "Vertex.h"
#include "common/Define.h"
#include "common/HalfedgeErrorEntry.h"
#include <cstddef>
#include <vector>

void BoundaryVertexRemovalEliminateOperator::init(std::shared_ptr<MCGAL::Mesh> mesh,PMSFHeader header, bool compress_boundary) {
    mesh_ = mesh;
    compress_boundary_ = compress_boundary;
}

bool BoundaryVertexRemovalEliminateOperator::eliminate(HalfedgeErrorEntry& entry) {
    MCGAL::Halfedge* h = entry.edge;
    if (h == nullptr)
        return false;
    CHECK_TRUE(remove_point(h));
    CHECK_TRUE(triangulate(h));
    CHECK_TRUE(postprocess(h));
    return true;
}

bool BoundaryVertexRemovalEliminateOperator::remove_point(MCGAL::Halfedge* h) {
    if (!h)
        return false;
    h->vertex()->setCollapsed();
    MCGAL::Vertex* removedVertex = h->end_vertex();
    if (!removedVertex || removedVertex->isRemoved())
        return false;
    h->end_vertex()->setRemoved();
    MCGAL::Halfedge* hface = mesh_->erase_center_vertex(h);

    // MCGAL::Point barycenter(0, 0, 0);
    // MCGAL::Halfedge* st = hface->face()->proxyHalfedge();
    // MCGAL::Halfedge* ed = st;
    // int fsize = 0;
    // do {
    //     barycenter = barycenter + st->vertex()->point();
    //     fsize++;
    //     st = st->next();
    // } while (st != ed);
    // barycenter = barycenter / hface->face()->getVertices().size();

    // // barycenter = barycenter / vrNeighborsBefore.size();

    // MCGAL::Vector3 normal = (hface->face()->computeNormal() + hface->opposite()->face()->computeNormal()).normalize();

    // MCGAL::Vector3i predPos = mesh_->floatPosToInt(barycenter);
    // MCGAL::Vector3i vrPos = mesh_->floatPosToInt(h->end_vertex());
    // MCGAL::Vector3i residual = vrPos - predPos;

    // {
    //     MCGAL::Vector3 t1, t2;
    //     MCGAL::determineFrenetFrame(h->vertex()->point(), h->end_vertex()->point(), normal, t1, t2);
    //     MCGAL::Vector3i toEncode = MCGAL::frenetRotation(residual, t1, t2, normal);
    //     // hface->face()->setRemovedVertexPosInt(toEncode);
    //     hface->face()->setRemovedVertexPosInt(residual);
    //     // Vec3i toEncode = vrPos;
    // }

    hface->face()->setRemovedVertexPos(h->end_vertex()->point() - h->vertex()->point());
    h->setFace(hface->face());
    // hface->face()->setRemovedVertexPos(h->end_vertex()->point());
    return true;
}

bool BoundaryVertexRemovalEliminateOperator::triangulate(MCGAL::Halfedge* h) {
    if (!h)
        return false;
    int groupId = h->face()->groupId();
    MCGAL::Facet* polyFace = h->face();
    // if (!polyFace || polyFace->isRemoved())
    //     return false;

    std::vector<MCGAL::Vertex*> polyVerts = polyFace->getVertices();
    std::vector<MCGAL::Halfedge*> polyHalfedges = polyFace->getHalfedges();
    int n = static_cast<int>(polyVerts.size());
    if (n < 4) {
        h->face()->setSplittable();
        return true;
    }
    h->face()->setSplittable();
    h = polyFace->proxyHalfedge();

    for (int i = 0; i < n - 2; ++i) {
        if (i == 0) {
            MCGAL::Halfedge* h0 = polyHalfedges[0];
            MCGAL::Halfedge* h1 = polyHalfedges[1];
            MCGAL::Vertex* v1 = polyVerts[2];
            MCGAL::Vertex* v2 = polyVerts[0];
            MCGAL::Halfedge* hnew = MCGAL::contextPool.allocateHalfedgeFromPool(mesh_->meshId(), v1, v2);
            hnew->setAdded();
            h0->setNext(h1);
            h1->setNext(hnew);
            hnew->setNext(h0);
            polyFace->reset(h0);
        } else if (i == n - 3) {
            MCGAL::Halfedge* h0 = polyHalfedges[n - 2];
            MCGAL::Halfedge* h1 = polyHalfedges[n - 1];
            MCGAL::Vertex* v1 = polyVerts[0];
            MCGAL::Vertex* v2 = polyVerts[n - 2];
            MCGAL::Halfedge* hnew = MCGAL::contextPool.allocateHalfedgeFromPool(mesh_->meshId(), v1, v2);
            hnew->setAdded();
            h0->setNext(h1);
            h1->setNext(hnew);
            hnew->setNext(h0);
            MCGAL::Facet* newFace = MCGAL::contextPool.allocateFaceFromPool(mesh_->meshId(), h0);
            newFace->setGroupId(groupId);
            mesh_->add_face(newFace);
        } else {
            MCGAL::Vertex* v0 = polyVerts[0];
            MCGAL::Vertex* v1 = polyVerts[i + 1];
            MCGAL::Vertex* v2 = polyVerts[i + 2];
            MCGAL::Halfedge* h0 = polyHalfedges[i + 1];
            MCGAL::Halfedge* h1 = MCGAL::contextPool.allocateHalfedgeFromPool(mesh_->meshId(), v2, v0);
            MCGAL::Halfedge* h2 = MCGAL::contextPool.allocateHalfedgeFromPool(mesh_->meshId(), v0, v1);
            h1->setAdded();
            h2->setAdded();
            h0->setNext(h1);
            h1->setNext(h2);
            h2->setNext(h0);
            MCGAL::Facet* newFace = MCGAL::contextPool.allocateFaceFromPool(mesh_->meshId(), h0);
            newFace->setGroupId(groupId);
            mesh_->add_face(newFace);
        }
    }

    return true;
}

bool BoundaryVertexRemovalEliminateOperator::postprocess(MCGAL::Halfedge* h) {
    return true;
}

static void setNewGroupId(MCGAL::Facet* f) {
    for (auto it = f->halfedges_begin(); it != f->halfedges_end(); it++) {
        if (!(*it)->isBoundary()) {
            f->setGroupId((*it)->face()->groupId());
            break;
        }
    }
}

/**
 * 后面需要单独验证是否正确
 */
// 问题：如果下下个边界在多边形上，就会导致边界出错
MCGAL::Halfedge* BoundaryVertexRemovalEliminateOperator::encode_boundary(MCGAL::Halfedge* h) {
    // if (!compress_boundary_ || !h->isBoundary()) {
    //     return true;
    // }
    int gid1 = h->face()->groupId();
    int gid2 = h->opposite()->face()->groupId();
    MCGAL::Halfedge* boundary = h;
    MCGAL::Halfedge* nxtBoundary = h->next_boundary();
    MCGAL::Vertex* stPoint = h->vertex();
    MCGAL::Vertex* edPoint = nxtBoundary->end_vertex();
    MCGAL::Halfedge* outHalfedge = nullptr;
    remove_point(h);
    MCGAL::Halfedge* st = h->face()->proxyHalfedge();
    MCGAL::Halfedge* ed = st;
    std::set<MCGAL::Halfedge*> facetSet;
    do {
        facetSet.insert(st);
        st = st->next();
    } while (st != ed);
    MCGAL::Vertex* outPoint = nullptr;
    // 锚定出点，不是面内部的边界会被标记为出点
    // 然后重新标定边界，边界为出点和入点之间的点
    MCGAL::Halfedge* stHalfedge = nullptr;
    do {
        MCGAL::Vertex* v = st->vertex();
        if (st->vertex() == stPoint) {
            stHalfedge = st;
        }
        st->setNotBoundary();
        st->opposite()->setNotBoundary();
        st = st->next();
    } while (st != ed);

    do {
        MCGAL::Vertex* v = st->vertex();
        for (MCGAL::Halfedge* hit : pmsf::boundedRing(v)) {
            if (hit->isBoundary() && hit->vertex() != stPoint && !facetSet.count(hit) && hit->face()->groupId() == gid1) {
                outPoint = hit->vertex();
                outHalfedge = hit;
            }
        }
        st = st->next();
    } while (st != ed);
    // while (true) {
    //     if (stHalfedge->vertex() == outPoint) {
    //         break;
    //     }
    //     stHalfedge->setBoundary();
    //     stHalfedge = stHalfedge->next();
    //     // st =st->next();
    // }
    triangulate(h);
    for (MCGAL::Halfedge* hit : facetSet) {
        if (hit->face()->groupId() != hit->opposite()->face()->groupId()) {
            hit->setBoundary();
            hit->opposite()->setBoundary();
        } else {
            hit->setNotBoundary();
            hit->opposite()->setNotBoundary();
        }
    }
    return outHalfedge;
}
