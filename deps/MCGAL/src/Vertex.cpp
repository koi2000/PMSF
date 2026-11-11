#include "Vertex.h"
#include "Halfedge.h"
namespace MCGAL {

// 2026-05-04: halfedge_iterator::operator++ 已 inline 到 Vertex.h 末尾
// （perf: 占 RD bunny 33% CPU）。此处只保留 Vertex 其他成员的定义。

int Vertex::vertex_degree() {
    if (degree_dirty_) {
        recompute_degree();
    }
    return vertex_degree_;
}

void Vertex::recompute_degree() {
    int degree = 0;
    for (Halfedge* hit : *this) {
        if (hit != nullptr) {
            degree++;
        }
    }
    vertex_degree_ = degree;
    degree_dirty_ = false;
}

bool Vertex::isConnected(Vertex* v) const {
    for (Halfedge* hit : *this) {
        if (hit->end_vertex() == v) {
            return true;
        }
    }
    return false;
}

Halfedge* Vertex::find_halfedge(Vertex* v) {
    for (Halfedge* hit : *this) {
        if (hit->end_vertex() == v) {
            return hit;
        }
    }
    return nullptr;
}

}  // namespace MCGAL
