/*
 * @Author: koi
 * @Date: 2025-08-27 22:37:03
 * @Description:
 */
#include "SegVertexRemovalEliminateOperator.h"
#include "common/BoundedRingTraversal.h"
#include "common/DebugTool.h"
#include "Facet.h"
#include "Global.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "MeshUtils.h"
#include "Point.h"
#include "Vertex.h"
#include "common/Define.h"
#include <cassert>
#include <cstddef>
#include <vector>

namespace {

bool touchesTopologyBoundary(MCGAL::Vertex* v) {
    if (!v) return false;
    for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
        if (!h) continue;
        if (!h->opposite()) return true;
        MCGAL::Halfedge* prev = h->find_prev();
        if (prev && !prev->opposite()) return true;
    }
    return false;
}

void markSplittable(MCGAL::Facet* f) {
    if (!f) return;
    if (f->isSplittable()) return;
    if (f->isUnsplittable()) {
        f->resetSplittableFlag();
    }
    f->setSplittable();
}

}  // namespace

void SegVertexRemovalEliminateOperator::init(std::shared_ptr<MCGAL::Mesh> mesh, PMSFHeader header, bool compress_boundary) {
    mesh_ = mesh;
    compress_boundary_ = compress_boundary;
    header_ = header;
}

bool SegVertexRemovalEliminateOperator::eliminate(HalfedgeErrorEntry& entry) {
    MCGAL::Halfedge* h = entry.edge;
    if (h == nullptr)
        return false;
    // if (compress_boundary_ && h->isBoundary()) {
    //     CHECK_TRUE(encode_boundary(h));
    //     return true;
    // }
    bool isBoundary = h->end_vertex()->isBoundary() || touchesTopologyBoundary(h->end_vertex());
    if (isBoundary) {
        h->end_vertex()->setBoundary();
    }
    if (h->end_vertex()->poolId() == 10398) {
        int i = 0;
    }
    if (!isBoundary) {
        CHECK_TRUE(remove_point(h));
        CHECK_TRUE(triangulate(h));
        CHECK_TRUE(postprocess(h));
    } else {
        CHECK_TRUE(remove_point(h));
        CHECK_TRUE(triangulate_plus(h));
        CHECK_TRUE(postprocess(h));
    }

    // CHECK_TRUE(encode_boundary(h));
    if (h && h->face() && h->face()->proxyHalfedge()) {
        entry.edge = h->face()->proxyHalfedge();
    }
    return true;
}

bool SegVertexRemovalEliminateOperator::remove_point(MCGAL::Halfedge* h) {
    if (!h)
        return false;
    h->vertex()->setCollapsed();
    // DEBUG_DUMP_VERTEX_THREE_RING(h->end_vertex(), "./segment/a_1.off");
    // h->vertex()->setPoint((h->vertex()->point() + h->end_vertex()->point()) / 2);
    MCGAL::Vertex* removedVertex = h->end_vertex();
    if (!removedVertex || removedVertex->isRemoved())
        return false;
    // h->end_vertex()->setRemoved();
    MCGAL::Halfedge* hface = mesh_->erase_center_vertex_plus(h);
    if (!hface || !hface->face()) {
        return false;
    }
    // hface->face()->setRemoved();
    // DEBUG_DUMP_VERTEX_THREE_RING(h->vertex(), "./segment/a_2.off");

    // MCGAL::Point barycenter(0, 0, 0);
    // MCGAL::Halfedge* st = hface;
    // MCGAL::Halfedge* ed = nullptr;
    // int fsize = 0;
    // do {
    //     barycenter = barycenter + st->vertex()->point();
    //     fsize++;
    //     st = st->next();
    // } while (st != ed);
    // barycenter = barycenter / hface->face()->getVertices().size();

    // barycenter = barycenter / vrNeighborsBefore.size();

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

    // hface->face()->setRemovedVertexPos(h->end_vertex()->point() - h->vertex()->point());
    h->setFace(hface->face());

    hface->face()->setRemovedVertexPos(h->end_vertex()->point());
    // h = hface;
    return true;
}

bool SegVertexRemovalEliminateOperator::triangulate_plus(MCGAL::Halfedge* h) {
    if (!h)
        return false;
    int groupId = h->face()->groupId();
    MCGAL::Facet* polyFace = h->face();
    // if (!polyFace || polyFace->isRemoved())
    //     return false;

    std::vector<MCGAL::Vertex*> polyVerts = polyFace->getVertices();
    std::vector<MCGAL::Halfedge*> polyHalfedges = polyFace->getHalfedges();
    for (MCGAL::Halfedge* hit : polyHalfedges) {
        hit->b_collapseOk = false;
    }
    int n = static_cast<int>(polyVerts.size());
    // if (n < 4) {
    //     h->face()->setSplittable();
    //     return true;
    // }
    markSplittable(h->face());
    h = polyFace->proxyHalfedge();
    MCGAL::Halfedge* p1 = nullptr;
    MCGAL::Halfedge* p2 = nullptr;

    for (int i = 0; i < n - 2; ++i) {
        if (i == 0) {
            MCGAL::Halfedge* h0 = polyHalfedges[0];
            MCGAL::Halfedge* h1 = polyHalfedges[1];
            MCGAL::Vertex* v1 = polyVerts[2];
            MCGAL::Vertex* v2 = polyVerts[0];
            // assert(!v1->find_halfedge(v2));
            // assert(!v2->find_halfedge(v1));
            MCGAL::Halfedge* hnew = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(mesh_->meshId(), v1, v2);
            if (v1->proxyHalfedge() == nullptr) {
                v1->setProxyHalfedge(hnew);
            }
            p1 = hnew;
            hnew->setAdded();
            h0->setNext(h1);
            h1->setNext(hnew);
            hnew->setNext(h0);
            polyFace->reset(h0);
        }
        // else if (i == n - 3) {
        //     MCGAL::Halfedge* h0 = polyHalfedges[n - 2];
        //     MCGAL::Halfedge* h1 = polyHalfedges[n - 1];
        //     MCGAL::Vertex* v1 = polyVerts[0];
        //     MCGAL::Vertex* v2 = polyVerts[n - 2];
        //     MCGAL::Halfedge* hnew = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(mesh_->meshId(), v1, v2);
        //     // p2 = hnew;
        //     hnew->setOpposite(p1);
        //     p1->setOpposite(hnew);
        //     hnew->setAdded();
        //     h0->setNext(h1);
        //     h1->setNext(hnew);
        //     hnew->setNext(h0);
        //     MCGAL::Facet* newFace = MCGAL::contextPool.allocateFaceFromPool(mesh_->meshId(), h0);
        //     newFace->setGroupId(groupId);
        //     mesh_->add_face(newFace);
        // }
        else {
            MCGAL::Vertex* v0 = polyVerts[0];
            MCGAL::Vertex* v1 = polyVerts[i + 1];
            MCGAL::Vertex* v2 = polyVerts[i + 2];
            MCGAL::Halfedge* h0 = polyHalfedges[i + 1];
            // assert(!v2->find_halfedge(v0));
            // assert(!v0->find_halfedge(v1));
            // assert(!v0->find_halfedge(v2));
            // assert(!v1->find_halfedge(v0));

            MCGAL::Halfedge* h1 = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(mesh_->meshId(), v2, v0);
            if (v2->proxyHalfedge() == nullptr) {
                v2->setProxyHalfedge(h1);
            }
            MCGAL::Halfedge* h2 = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(mesh_->meshId(), v0, v1);
            if (v0->proxyHalfedge() == nullptr) {
                v0->setProxyHalfedge(h2);
            }

            // h1->setOpposite(h2);
            h2->setOpposite(p1);
            p1->setOpposite(h2);
            p1 = h1;

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
    // p1->setOpposite(p2);
    // p2->setOpposite(p1);

    return true;
}

bool SegVertexRemovalEliminateOperator::triangulate(MCGAL::Halfedge* h) {
    if (!h)
        return false;
    int groupId = h->face()->groupId();
    MCGAL::Facet* polyFace = h->face();
    // if (!polyFace || polyFace->isRemoved())
    //     return false;

    std::vector<MCGAL::Vertex*> polyVerts = polyFace->getVertices();
    std::vector<MCGAL::Halfedge*> polyHalfedges = polyFace->getHalfedges();
    for (MCGAL::Halfedge* hit : polyHalfedges) {
        hit->b_collapseOk = false;
    }
    int n = static_cast<int>(polyVerts.size());
    if (n < 4) {
        markSplittable(h->face());
        return true;
    }
    markSplittable(h->face());
    h = polyFace->proxyHalfedge();
    MCGAL::Halfedge* p1 = nullptr;
    MCGAL::Halfedge* p2 = nullptr;

    for (int i = 0; i < n - 2; ++i) {
        if (i == 0) {
            MCGAL::Halfedge* h0 = polyHalfedges[0];
            MCGAL::Halfedge* h1 = polyHalfedges[1];
            MCGAL::Vertex* v1 = polyVerts[2];
            MCGAL::Vertex* v2 = polyVerts[0];
            MCGAL::Halfedge* hnew = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(mesh_->meshId(), v1, v2);
            if (v1->proxyHalfedge() == nullptr) {
                v1->setProxyHalfedge(hnew);
            }
            p1 = hnew;
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
            MCGAL::Halfedge* hnew = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(mesh_->meshId(), v1, v2);
            if (v1->proxyHalfedge() == nullptr) {
                v1->setProxyHalfedge(hnew);
            }
            // p2 = hnew;
            hnew->setOpposite(p1);
            p1->setOpposite(hnew);
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
            MCGAL::Halfedge* h1 = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(mesh_->meshId(), v2, v0);
            MCGAL::Halfedge* h2 = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(mesh_->meshId(), v0, v1);
            if (v2->proxyHalfedge() == nullptr) {
                v2->setProxyHalfedge(h1);
            }
            if (v0->proxyHalfedge() == nullptr) {
                v0->setProxyHalfedge(h2);
            }
            // h1->setOpposite(h2);
            h2->setOpposite(p1);
            p1->setOpposite(h2);
            p1 = h1;

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
    // p1->setOpposite(p2);
    // p2->setOpposite(p1);

    return true;
}

bool SegVertexRemovalEliminateOperator::postprocess(MCGAL::Halfedge* h) {
    for (MCGAL::Halfedge* hit : *h->vertex()) {
        hit->b_collapseOk = false;
        if (hit->opposite() != nullptr) {
            hit->opposite()->b_collapseOk = false;
        }
    }
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
MCGAL::Halfedge* SegVertexRemovalEliminateOperator::encode_boundary(MCGAL::Halfedge* h) {
    return nullptr;
}
