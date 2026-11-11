#include "Halfedge.h"
#include "Facet.h"
#include "Vertex.h"

namespace MCGAL {

int g_pmsfBaseFaceIndex = -1;
int g_pmsfBaseFaceVids[3] = {-1, -1, -1};

// #define ENABLE_HALFEDGE_DUPLICATE_CHECK 1

// create a new half edge, setup the opposite of this half edge if needed
Halfedge::Halfedge(Vertex* v1, Vertex* v2) {
    vertex_ = v1;
    end_vertex_ = v2;
    if (v1->proxyHalfedge() == nullptr) {
        v1->setProxyHalfedge(this);
    }
    vertex_->halfedges().push_back(this);
    if (v2->proxyHalfedge() == nullptr) {
        return;
    }
    // TODO: 完善 ENABLE_HALFEDGE_DUPLICATE_CHECK
    // #ifdef ENABLE_HALFEDGE_DUPLICATE_CHECK
    for (Halfedge* h : v2->halfedges()) {
        if (h->end_vertex_ == v1) {
            if (h->opposite()) {
                printf("create half edge:\n");
                v1->print();
                v2->print();
                h->opposite()->face()->print_off();
            }
            assert(h->opposite() == nullptr);
            h->setOpposite(this);
            this->setOpposite(h);
            break;
        }
    }
    // #endif
}

void Halfedge::reset_without_init(Vertex* v1, Vertex* v2) {
    vertex_ = v1;
    end_vertex_ = v2;
}

void Halfedge::reset(Vertex* v1, Vertex* v2) {
    vertex_ = v1;
    end_vertex_ = v2;
    if (v1->proxyHalfedge() == nullptr) {
        v1->setProxyHalfedge(this);
    }
    vertex_->halfedges().push_back(this);
    // end_vertex_->opposite_half_edges.insert(this);
    // in case this is the second half edge
    // TODO: 完善 ENABLE_HALFEDGE_DUPLICATE_CHECK
    // #ifdef ENABLE_HALFEDGE_DUPLICATE_CHECK
    if (v2->proxyHalfedge() == nullptr) {
        return;
    }
    for (Halfedge* h : v2->halfedges()) {
        if (h->end_vertex_ == v1) {
            if (h->opposite()) {
                static int s_dupEdgeLogs = 0;
                if (s_dupEdgeLogs < 8) {
                    ++s_dupEdgeLogs;
                    if (std::getenv("PMSF_NE_DUP_EDGE_DEBUG") ||
                        std::getenv("PMSF_BOUNDARY_DEBUG")) {
                        auto qa = v1->point();
                        auto qb = v2->point();
                        std::fprintf(stderr,
                                     "[dup-edge] face=%d vids=%d,%d,%d v1=(%.6f,%.6f,%.6f) "
                                     "v2=(%.6f,%.6f,%.6f)\n",
                                     g_pmsfBaseFaceIndex, g_pmsfBaseFaceVids[0],
                                     g_pmsfBaseFaceVids[1], g_pmsfBaseFaceVids[2], qa.x(),
                                     qa.y(), qa.z(), qb.x(), qb.y(), qb.z());
                    }
                }
            }
            // Keep the historical behavior: pair over an existing opposite.
            h->setOpposite(this);
            this->setOpposite(h);
            break;
        }
    }
    // #endif
}

Halfedge* Halfedge::next_boundary() {
    MCGAL::Halfedge* boundary = this;
    MCGAL::Halfedge* nxt = boundary->next();
    if (nxt->isBoundary()) {
        return nxt;
    }
    nxt = boundary->next()->opposite()->next();
    if (nxt->isBoundary()) {
        return nxt;
    }
    for (MCGAL::Halfedge* hit : *boundary->end_vertex()) {
        if (hit->opposite() != boundary && hit->isBoundary()) {
            return hit;
        }
    }
    return nullptr;
}

void Halfedge::setVertex(Vertex* v1, Vertex* v2) {
    vertex_ = v1;
    end_vertex_ = v2;
}

float Halfedge::length() {
    return (vertex_->point() - end_vertex_->point()).length();
}

Halfedge::~Halfedge() {}
}  // namespace MCGAL
