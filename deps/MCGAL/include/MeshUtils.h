#ifndef MESH_UTILS_H
#define MESH_UTILS_H

#include "BufferUtils.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Matrix.h"
#include "Mesh.h"
#include "Point.h"
#include "Vertex.h"
#include "core.h"
#include "symetric_matrix.h"
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <memory>
#include <vector>
// #include "common/DebugTool.h"

namespace MCGAL {

extern int g_pmsfBaseFaceIndex;
extern int g_pmsfBaseFaceVids[3];

static double VertexError(const SymetricMatrix& q, double x, double y, double z) {
    return q[0] * x * x + 2 * q[1] * x * y + 2 * q[2] * x * z + 2 * q[3] * x + q[4] * y * y + 2 * q[5] * y * z + 2 * q[6] * y + q[7] * z * z +
           2 * q[8] * z + q[9];
}

static double CalculateError(SymetricMatrix v0m, SymetricMatrix v1m, MCGAL::Halfedge* edge, MCGAL::Point& ptResult) {
    MCGAL::Halfedge* h0 = edge;
    MCGAL::Vertex* v0 = h0->vertex();
    MCGAL::Vertex* v1 = h0->end_vertex();
    SymetricMatrix q = v0m + v1m;

    double dError = 0;
    // The number for Det, is the index in q
    double det = q.Det(0, 1, 2, 1, 4, 5, 2, 5, 7);
    if (fabs(det) > 1e-6 && !edge->isBoundary()) {
        ptResult[0] = -1 / det * (q.Det(1, 2, 3, 4, 5, 6, 5, 7, 8));  // vx = A41/det(q_delta)
        ptResult[1] = 1 / det * (q.Det(0, 2, 3, 1, 5, 6, 2, 7, 8));   // vy = A42/det(q_delta)
        ptResult[2] = -1 / det * (q.Det(0, 1, 3, 1, 4, 6, 2, 5, 8));  // vz = A43/det(q_delta)

        dError = VertexError(q, ptResult[0], ptResult[1], ptResult[2]);
    } else {
        // find if v0, or v1, or midpoint
        MCGAL::Point pt0 = v0->point();
        MCGAL::Point pt1 = v1->point();
        MCGAL::Point pt2 = (pt0 + pt1) / 2;
        double error0 = VertexError(q, pt0[0], pt0[1], pt0[2]);
        double error1 = VertexError(q, pt1[0], pt1[1], pt1[2]);
        double error2 = VertexError(q, pt2[0], pt2[1], pt2[2]);
        dError = std::min(error0, std::min(error1, error2));
        if (error0 == dError) {
            ptResult = pt0;
        }
        if (error1 == dError) {
            ptResult = pt1;
        }
        if (error2 == dError) {
            ptResult = pt2;
        }
    }

    return dError;
}

#ifdef USE_DEPRECATED
static bool IsFlipped(MCGAL::Halfedge* edge, const MCGAL::Point& ptTarget) {
    MCGAL::Vertex* v0 = edge->vertex();
    MCGAL::Vertex* v1 = edge->end_vertex();
    // 检查坍缩后是否会导致重叠面片或非流形网格
    // 坍缩后确保其他点的邻居数不能少于三个

    for (MCGAL::Halfedge* h : v0->halfedges()) {
        if (h->face()->isRemoved()) {
            continue;
        }
        std::vector<MCGAL::Vertex*> vs;
        MCGAL::Halfedge* st = h->face()->proxyHalfedge();
        MCGAL::Halfedge* ed = st;
        do {
            vs.push_back(st->vertex());
            st = st->next();
        } while (st != ed);
        if (vs[0] == v1 || vs[1] == v1 || vs[2] == v1) {
            continue;
        }
        int idxV0 = 0;
        for (int i = 0; i < vs.size(); i++) {
            MCGAL::Vertex* v = vs[i];
            if (vs[i] == v0) {
                idxV0 = i;
            }
        }
        MCGAL::Point pt0 = v0->point();
        MCGAL::Point pt1 = vs[(idxV0 + 1) % 3]->point();
        MCGAL::Point pt2 = vs[(idxV0 + 2) % 3]->point();

        MCGAL::Point dir1 = pt1 - ptTarget;
        dir1.normalize();
        MCGAL::Point dir2 = pt2 - ptTarget;
        dir2.normalize();

        // The angle below can be adjusted, but now is enough
        // if the angle between dir1 and dir2 small than 2.6 angle, return true
        if (fabs(dir1.dot(dir2)) > 0.999)
            return true;
        MCGAL::Point normold;
        try {
            normold = h->face()->computeNormal();
        } catch (std::string exp) { return true; }
        MCGAL::Point norm = dir1.cross(dir2);
        norm.normalize();

        // if the angle between normold and norm large than 78.5 angle, return true
        if ((normold.dot(norm)) < 0.2)
            return true;
    }
    return false;
}
#endif

// static MCGAL::Halfedge* next_boundary(int ogroupId, MCGAL::Halfedge* boundary) {
//     MCGAL::Halfedge* nxt = boundary->next();
//     if (nxt->isBoundary()) {
//         return nxt;
//     }
//     nxt = boundary->next()->opposite()->next();
//     if (nxt->isBoundary()) {
//         return nxt;
//     }
//     for (MCGAL::Halfedge* hit : boundary->end_vertex()->halfedges()) {
//         if (hit->opposite() != boundary && hit->isBoundary() && hit->opposite()->face()->groupId() == ogroupId) {
//             return hit;
//         }
//         // if (hit->face()->groupId() != ogroupId && hit->opposite()->face()->groupId() == ogroupId) {
//         //     return hit;
//         // }
//     }
//     return nullptr;
// }

static Halfedge* find_prev(Halfedge* h) {
    Halfedge* g = h;
    while (g->next() != h) {
        g = g->next();
    }
    return g;
}

static MCGAL::Vector3 crossProduct(const MCGAL::Point& A, const MCGAL::Point& B, const MCGAL::Point& P) {
    MCGAL::Vector3 AB = B - A;
    MCGAL::Vector3 AP = P - A;
    return AB.cross(AP);
}

static bool compareVertex(MCGAL::Vertex* a, MCGAL::Vertex* b) {
    if (a->x() != b->x()) {
        return a->x() < b->x();
    }
    if (a->y() != b->y()) {
        return a->y() < b->y();
    }
    return a->z() < b->z();
}

static std::shared_ptr<MCGAL::Mesh> buildFromBuffer(int meshId, std::deque<MCGAL::Point>* p_pointDeque, std::deque<uint32_t*>* p_faceDeque) {
    std::shared_ptr<MCGAL::Mesh> mesh = std::make_shared<MCGAL::Mesh>();
    mesh->setMeshId(meshId);
    std::vector<MCGAL::Vertex*> vertices;
    for (std::size_t i = 0; i < p_pointDeque->size(); ++i) {
        float x, y, z;
        MCGAL::Point p = p_pointDeque->at(i);
        MCGAL::Vertex* vt = MCGAL::contextPool.allocateVertexFromPool(meshId, p);
        mesh->add_vertex(vt);
        vertices.push_back(vt);
    }
    for (int i = 0; i < p_faceDeque->size(); ++i) {
        uint32_t* ptr = p_faceDeque->at(i);
        int num_face_vertices = 3;
        std::vector<MCGAL::Vertex*> vts;
        std::vector<int> vtIds;
        for (int j = 0; j < num_face_vertices; ++j) {
            int vertex_index = ptr[j];
            vtIds.push_back(vertex_index);
            vts.push_back(vertices[vertex_index]);
        }
        g_pmsfBaseFaceIndex = i;
        g_pmsfBaseFaceVids[0] = static_cast<int>(vtIds[0]);
        g_pmsfBaseFaceVids[1] = static_cast<int>(vtIds[1]);
        g_pmsfBaseFaceVids[2] = static_cast<int>(vtIds[2]);
        MCGAL::Facet* face = MCGAL::contextPool.allocateFaceFromPool(meshId, vts);
        mesh->add_face(face);
    }
    vertices.clear();
    return mesh;
}

static std::shared_ptr<MCGAL::Mesh> buildFromBufferNonTri(int meshId, std::deque<MCGAL::Point>* p_pointDeque, std::deque<uint32_t*>* p_faceDeque) {
    std::shared_ptr<MCGAL::Mesh> mesh = std::make_shared<MCGAL::Mesh>();
    mesh->setMeshId(meshId);
    std::vector<MCGAL::Vertex*> vertices;
    for (std::size_t i = 0; i < p_pointDeque->size(); ++i) {
        float x, y, z;
        MCGAL::Point p = p_pointDeque->at(i);
        MCGAL::Vertex* vt = MCGAL::contextPool.allocateVertexFromPool(meshId, p);
        mesh->add_vertex(vt);
        vertices.push_back(vt);
    }
    for (int i = 0; i < p_faceDeque->size(); ++i) {
        uint32_t* ptr = p_faceDeque->at(i);
        int num_face_vertices = ptr[0];
        std::vector<MCGAL::Vertex*> vts;
        std::vector<int> vtIds;
        for (int j = 1; j <= num_face_vertices; ++j) {
            int vertex_index = ptr[j];
            vtIds.push_back(vertex_index);
            vts.push_back(vertices[vertex_index]);
        }
        MCGAL::Facet* face = MCGAL::contextPool.allocateFaceFromPool(meshId, vts);
        mesh->add_face(face);
    }
    vertices.clear();
    return mesh;
}

static std::shared_ptr<MCGAL::Mesh> readBaseMesh(int meshId, char* buffer, size_t& dataOffset) {
    unsigned i_nbVerticesBaseMesh = readInt(buffer, dataOffset);
    unsigned i_nbFacesBaseMesh = readInt(buffer, dataOffset);
    // MCGAL::contextPool.registerPool(i_nbVerticesBaseMesh * 20, i_nbFacesBaseMesh * 100, i_nbFacesBaseMesh * 50);
    std::deque<MCGAL::Point>* p_pointDeque = new std::deque<MCGAL::Point>();
    std::deque<uint32_t*>* p_faceDeque = new std::deque<uint32_t*>();
    for (unsigned i = 0; i < i_nbVerticesBaseMesh; ++i) {
        MCGAL::Point pos = readPoint(buffer, dataOffset);
        p_pointDeque->push_back(pos);
    }
    for (unsigned i = 0; i < i_nbFacesBaseMesh; ++i) {
        // int nv = readInt(buffer, dataOffset);
        int nv = 3;
        uint32_t* f = new uint32_t[nv];
        // f[0] = nv;
        for (unsigned j = 0; j < nv; ++j) {
            f[j] = readInt(buffer, dataOffset);
        }
        p_faceDeque->push_back(f);
    }
    auto mesh = buildFromBuffer(meshId, p_pointDeque, p_faceDeque);

    for (unsigned i = 0; i < p_faceDeque->size(); ++i) {
        delete[] p_faceDeque->at(i);
    }
    delete p_faceDeque;
    delete p_pointDeque;
    return mesh;
}

static std::vector<int> charPtrToVectorInt(char* data, size_t& dataOffset, int size) {
    std::vector<int> result(size);
    std::memcpy(result.data(), data + dataOffset, size * sizeof(int));
    dataOffset += size * sizeof(int);
    return result;
}

static std::vector<size_t> charPtrToVectorSizeT(char* data, size_t& dataOffset, int size) {
    std::vector<int> result(size);
    std::memcpy(result.data(), data + dataOffset, size * sizeof(int));
    dataOffset += size * sizeof(int);
    std::vector<size_t> res;
    for (int i = 0; i < size; i++) {
        res.push_back(result[i]);
    }
    return res;
}

static PointInt floatPosToInt(Point p, MCGAL::Point bboxMin, float f_quantStep) {
    static const bool nearInt = (std::getenv("PMSF_QUANT_NEAR_INT") != nullptr);
    auto conv = [&](float x, int axis) -> int {
        float d = (x - bboxMin[axis]) / f_quantStep;
        if (nearInt) {
            const int t = static_cast<int>(d);
            if (d - static_cast<float>(t) > 0.999f) d = static_cast<float>(t + 1);
        }
        return static_cast<int>(d);
    };
    return PointInt(conv(p[0], 0), conv(p[1], 1), conv(p[2], 2));
}

static inline MCGAL::Point intPosToFloat(PointInt p, MCGAL::Point bboxMin, float f_quantStep) {
    return Point(bboxMin[0] + (p[0] + 0.5) * f_quantStep, bboxMin[1] + (p[1] + 0.5) * f_quantStep, bboxMin[2] + (p[2] + 0.5) * f_quantStep);
}

static std::shared_ptr<MCGAL::Mesh> readBaseMeshWithQuantization(int meshId, char* buffer, size_t& dataOffset, bool enableQuantization = true) {
    char nbQuantBits;
    MCGAL::Point bboxMin;
    float f_quantStep;
    if (enableQuantization) {
        nbQuantBits = readChar(buffer, dataOffset);
        bboxMin = readPoint(buffer, dataOffset);
        f_quantStep = readFloat(buffer, dataOffset);
    }

    unsigned i_nbVerticesBaseMesh = readInt(buffer, dataOffset);
    unsigned i_nbFacesBaseMesh = readInt(buffer, dataOffset);
    if (std::getenv("PMSF_SERIALIZE_DEBUG"))
        std::fprintf(stderr, "[dec-base-hdr] V=%u F=%u dataOffset=%zu qbits=%d\n",
                     i_nbVerticesBaseMesh, i_nbFacesBaseMesh, dataOffset,
                     static_cast<int>(nbQuantBits));
    // MCGAL::contextPool.registerPool(i_nbVerticesBaseMesh * 5, i_nbFacesBaseMesh * 15, i_nbFacesBaseMesh * 5);
    // MCGAL::contextPool.registerPool(i_nbVerticesBaseMesh * 30, i_nbFacesBaseMesh * 120, i_nbFacesBaseMesh * 60);
    std::deque<MCGAL::Point>* p_pointDeque = new std::deque<MCGAL::Point>();
    std::deque<uint32_t*>* p_faceDeque = new std::deque<uint32_t*>();

    unsigned i_bitOffset = 0;
    for (unsigned i = 0; i < i_nbVerticesBaseMesh; ++i) {
        MCGAL::Point pos;
        if (enableQuantization) {
            int p[3];
            for (int i = 0; i < 3; i++) {
                p[i] = readBitsOff(nbQuantBits, buffer, i_bitOffset, dataOffset);
            }
            MCGAL::PointInt pi = PointInt(p[0], p[1], p[2]);
            pos[0] = bboxMin[0] + (p[0] + 0.5) * f_quantStep;
            pos[1] = bboxMin[1] + (p[1] + 0.5) * f_quantStep;
            pos[2] = bboxMin[2] + (p[2] + 0.5) * f_quantStep;
            // Round-trip guard: float precision at the quantization maximum can
            // make trunc((pos-bboxMin)/step) return p-1.  The encoder's
            // floatPosToInt truncates, so nudge the decoded coordinate until the
            // round-trip is exact.  This only changes the rare boundary vertex by
            // a fraction of one quantization step.
            for (int k = 0; k < 3; ++k) {
                for (int guard = 0; guard < 2; ++guard) {
                    const int q = static_cast<int>((pos[k] - bboxMin[k]) / f_quantStep);
                    if (q == p[k]) break;
                    pos[k] += static_cast<float>(p[k] - q) * 0.5f * f_quantStep;
                }
            }
            // pos = intPosToFloat(pi, bboxMin, f_quantStep);
        } else {
            pos = readPoint(buffer, dataOffset);
        }

        p_pointDeque->push_back(pos);
    }
    i_bitOffset = 0;
    dataOffset++;

    char connBit = readChar(buffer, dataOffset);
    std::vector<uint32_t> fs;
    // uint32_t ddd = readBits(connBit, buffer, i_bitOffset, --dataOffset);
    for (unsigned i = 0; i < i_nbFacesBaseMesh; ++i) {
        // int nv = readInt(buffer, dataOffset);
        uint32_t* f = new uint32_t[3];
        // f[0] = 3;
        for (unsigned j = 0; j < 3; ++j) {
            f[j] = readBitsOff(connBit, buffer, i_bitOffset, dataOffset);
            assert(f[j] >= 0 && f[j] < i_nbVerticesBaseMesh);
            fs.push_back(f[j]);
        }
        p_faceDeque->push_back(f);
    }
    i_bitOffset = 0;
    dataOffset++;
    if (std::getenv("PMSF_SERIALIZE_DEBUG")) {
        unsigned mx = 0;
        for (auto* f : *p_faceDeque)
            for (int j = 0; j < 3; ++j) mx = std::max(mx, f[j]);
        std::fprintf(stderr,
                     "[dec-base-face] connBit=%d dataOffset=%zu faces=%zu maxIdx=%u\n",
                     static_cast<int>(connBit), dataOffset, p_faceDeque->size(), mx);
    }
    // DEBUG_DUMP_CONTAINER<uint32_t>(fs, );
    // std::ofstream file("./fsd.txt");
    // for (const auto& elem : fs) {
    //     int int_elem = (int)elem;
    //     file << int_elem << '\n';
    // }
    // file.close();
    auto mesh = buildFromBuffer(meshId, p_pointDeque, p_faceDeque);
    if (enableQuantization) {
        mesh->bboxMin = bboxMin;
        mesh->f_quantStep = f_quantStep;
    }
    for (unsigned i = 0; i < p_faceDeque->size(); ++i) {
        delete[] p_faceDeque->at(i);
    }
    delete p_faceDeque;
    delete p_pointDeque;
    return mesh;
}

static inline void remove_tip(Halfedge* h) {
    h->setNext(h->next()->opposite()->next());
}

/**
 * 还是顺序的问题 明天定位一下顺序的问题处在哪里
 */
static Halfedge* join_face(Halfedge* h) {
    // if (h->opposite() == nullptr) {
    //     return nullptr;
    // }
    Halfedge* hprev = find_prev(h);
    Halfedge* gprev = find_prev(h->opposite());
    remove_tip(hprev);
    remove_tip(gprev);
    h->opposite()->setRemoved();
    h->setRemoved();
    if (h->vertex()->proxyHalfedge() == h) {
        h->vertex()->setProxyHalfedge(hprev);
    }
    if (h->opposite()->vertex()->proxyHalfedge() == h->opposite()) {
        h->opposite()->vertex()->setProxyHalfedge(gprev);
    }
    if (gprev->face()->isSplittable()) {
        hprev->face()->setRemoved();
        assert(!gprev->isRemoved());
        gprev->face()->reset_without_init(gprev);
    } else {
        gprev->face()->setRemoved();
        assert(!hprev->isRemoved());
        hprev->face()->reset_without_init(hprev);
    }
    return hprev;
}

static MCGAL::Vector3 calculateFaceNormal(const MCGAL::Facet* face) {
    MCGAL::Vector3 normal = face->computeNormal();
    return normal;
}

static MCGAL::Vector3 calculateFaceNormal(std::vector<MCGAL::Vertex*> vs) {
    if (vs.size() < 3) {
        throw std::string("Cannot calculate normal: less than 3 vertices provided");
    }

    const MCGAL::Point& p1 = vs[0]->point();
    const MCGAL::Point& p2 = vs[1]->point();
    const MCGAL::Point& p3 = vs[2]->point();

    MCGAL::Vector3 v1 = p2 - p1;  // Vector from p1 to p2
    MCGAL::Vector3 v2 = p3 - p1;  // Vector from p1 to p3

    MCGAL::Vector3 normal = v1.cross(v2);

    float len = normal.length();
    if (len == 0) {
        throw std::string("The normal vector is zero; points are colinear or the face is degenerate");
    }
    return normal;
}

static bool areNormalOk(MCGAL::Vector3 n1, MCGAL::Vector3 n2) {
    if (n1 == MCGAL::Vector3(0, 0, 0) || n2 == MCGAL::Vector3(0, 0, 0) || (n1 | n2) > 0.7)
        return true;
    else
        return false;
}

static MCGAL::Vector3 calculateFaceNormal(MCGAL::Vertex* vx0, MCGAL::Vertex* vx1, MCGAL::Vertex* vx2) {
    const MCGAL::Point& p1 = vx0->point();
    const MCGAL::Point& p2 = vx1->point();
    const MCGAL::Point& p3 = vx2->point();

    MCGAL::Vector3 v1 = p2 - p1;  // Vector from p1 to p2
    MCGAL::Vector3 v2 = p3 - p1;  // Vector from p1 to p3

    MCGAL::Vector3 normal = v1.cross(v2);

    float len = normal.length();
    // if (len == 0) {
    //     throw std::string("The normal vector is zero; points are colinear or the face is degenerate");
    // }

    return normal;
}

static bool new_is_collapse_ok(Halfedge* v0v1) {
    Halfedge*& v1v0 = v0v1->opposite();
    Vertex*& v0 = v1v0->end_vertex();
    Vertex*& v1 = v0v1->end_vertex();
    Vertex*& vl = v0v1->next()->end_vertex();
    Vertex*& vr = v1v0->next()->end_vertex();

    if (vl == vr)
        return false;

    for (Halfedge* hit : *v0) {
        if (hit->end_vertex() != vl && hit->end_vertex() != vr && hit->end_vertex()->isConnected(v1))
            return false;
    }
    return true;
}

#ifdef USE_DEPRECATED
static bool is_collapse_ok(MCGAL::Halfedge* v0v1) {
    // 检查边是否已删除
    if (v0v1->isRemoved()) {
        return false;
    }

    MCGAL::Halfedge* v1v0 = v0v1->opposite();
    MCGAL::Vertex* v0 = v1v0->vertex();
    MCGAL::Vertex* v1 = v0v1->vertex();
    if (v1->halfedges().size() + v0->halfedges().size() >= 16) {
        return false;
    }

    bool v0v1_triangle = false;
    bool v1v0_triangle = false;

    // 检查是否为三角形面
    if (!v0v1->isBoundary()) {
        v0v1_triangle = v0v1->face()->facet_degree() == 3;
    }
    if (!v1v0->isBoundary()) {
        v1v0_triangle = v1v0->face()->facet_degree() == 3;
    }

    // 获取下一个顶点
    MCGAL::Vertex* v_01_n = v0v1->next()->end_vertex();
    MCGAL::Vertex* v_10_n = v1v0->next()->end_vertex();
    if (v_01_n->halfedges().size() <= 3 || v_10_n->halfedges().size() <= 3) {
        return false;
    }

    // 检查顶点是否已删除
    if (v0->isRemoved() || v1->isRemoved()) {
        return false;
    }

    // 检查三角形面的边界情况
    MCGAL::Vertex* vl = nullptr;
    if (!v0v1->isBoundary()) {
        if (v0v1_triangle) {
            MCGAL::Halfedge* h1 = v0v1->next();
            MCGAL::Halfedge* h2 = h1->next();

            vl = h1->vertex();
            if (h1->opposite()->isBoundary() && h2->opposite()->isBoundary()) {
                return false;
            }
        }
    }

    MCGAL::Vertex* vr = nullptr;
    if (!v1v0->isBoundary()) {
        if (v1v0_triangle) {
            MCGAL::Halfedge* h1 = v1v0->next();
            MCGAL::Halfedge* h2 = h1->next();

            vr = h1->vertex();
            if (h1->opposite()->isBoundary() && h2->opposite()->isBoundary()) {
                return false;
            }
        }
    }

    // 检查 vl 和 vr 是否相同
    if (vl && vr && vl == vr) {
        return false;
    }

    // 检查边界顶点
    // if (v0->isBoundary() && v1->isBoundary() && !v0v1->isBoundary() && !v1v0->isBoundary()) {
    //     return false;
    // }

    // 检查一环邻域交集
    std::set<MCGAL::Vertex*> v0_neighbors;
    std::set<MCGAL::Vertex*> v1_neighbors;

    // 收集 v0 的邻居
    for (MCGAL::Halfedge* hit : v0->halfedges()) {
        if (!hit->end_vertex()->isRemoved() && hit->face() != v0v1->face() && hit->face() != v1v0->face()) {
            v0_neighbors.insert(hit->end_vertex());
        }
    }

    // 收集 v1 的邻居
    for (MCGAL::Halfedge* hit : v1->halfedges()) {
        if (!hit->end_vertex()->isRemoved() && hit->face() != v0v1->face() && hit->face() != v1v0->face()) {
            v1_neighbors.insert(hit->end_vertex());
        }
    }

    // 检查交集
    std::set<MCGAL::Vertex*> cneighbor;
    for (MCGAL::Vertex* neighbor : v0_neighbors) {
        if (v1_neighbors.count(neighbor) > 0) {
            // 排除特殊三角形情况
            if (!(neighbor == v_01_n && v0v1_triangle) && !(neighbor == v_10_n && v1v0_triangle)) {
                return false;
            }
            cneighbor.insert(neighbor);
        }
    }
    MCGAL::Vertex* cv1 = v0v1->next()->end_vertex();
    MCGAL::Vertex* cv2 = v0v1->opposite()->next()->end_vertex();
    for (MCGAL::Halfedge* hit : cv1->halfedges()) {
        if (hit->end_vertex() == cv2) {
            return false;
        }
    }

    // 检查三角形面的退化情况
    if (v0v1_triangle) {
        MCGAL::Halfedge* one = v0v1->next();
        MCGAL::Halfedge* two = one->next();

        one = one->opposite();
        two = two->opposite();

        if (one->face() == two->face() && one->face()->facet_degree() != 3) {
            return false;
        }
    }

    if (v1v0_triangle) {
        MCGAL::Halfedge* one = v1v0->next();
        MCGAL::Halfedge* two = one->next();

        one = one->opposite();
        two = two->opposite();

        if (one->face() == two->face() && one->face()->facet_degree() != 3) {
            return false;
        }
    }

    // 额外的特殊情况检查
    if (v_01_n == v_10_n && v0v1_triangle && v1v0_triangle) {
        return false;
    }

    // 通过所有检查
    return true;
}
#endif

// Matrix product.
static Matrix matProd(Matrix const& m, Matrix const& n) {
    float mT[3][3];
    mT[0][0] = m.r0()[0];
    mT[0][1] = m.r0()[1];
    mT[0][2] = m.r0()[2];
    mT[1][0] = m.r1()[0];
    mT[1][1] = m.r1()[1];
    mT[1][2] = m.r1()[2];
    mT[2][0] = m.r2()[0];
    mT[2][1] = m.r2()[1];
    mT[2][2] = m.r2()[2];

    float nT[3][3];
    nT[0][0] = n.r0()[0];
    nT[0][1] = n.r0()[1];
    nT[0][2] = n.r0()[2];
    nT[1][0] = n.r1()[0];
    nT[1][1] = n.r1()[1];
    nT[1][2] = n.r1()[2];
    nT[2][0] = n.r2()[0];
    nT[2][1] = n.r2()[1];
    nT[2][2] = n.r2()[2];

    float C[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++)
                C[i][j] += mT[i][k] * nT[k][j];
        }
    }

    return Matrix(C[0][0], C[0][1], C[0][2], C[1][0], C[1][1], C[1][2], C[2][0], C[2][1], C[2][2]);
}

static inline int signe(const float x) {
    return (x < 0) ? -1 : 1;
}

// Description : To find a bijection through a rotation transformation in 3D with only integer coordinates.
static std::vector<Matrix> compRotationMat(Vector3& T1, Vector3& T2, Vector3& normal) {
    Matrix rotMat(T1[0], T2[0], normal[0], T1[1], T2[1], normal[1], T1[2], T2[2], normal[2]);
    Matrix M = rotMat;

    Matrix D1(1, 0, 0, 0, 1, 0, 0, 0, 1);
    Matrix D2(0, 1, 0, 1, 0, 0, 0, 0, 1);
    Matrix D3(-1, 0, 0, 0, -1, 0, 0, 0, 1);
    Matrix D4(1, 0, 0, 0, 0, 1, 0, 1, 0);
    Matrix D5(1, 0, 0, 0, -1, 0, 0, 0, -1);

    std::vector<Matrix> S(15);

    // Verify in order to find the smallest rotation angle.
    if (abs((int)M.r0()[2]) > abs((int)M.r1()[2]))
        S[0] = D2;
    else
        S[0] = D1;

    M = matProd(S[0], M);

    if (M.r1()[2] < 0)
        S[1] = D3;
    else
        S[1] = D1;

    M = matProd(S[1], M);

    // Determine first rotation angle phi.
    float len = (M.r0()[2] * M.r0()[2]) + (M.r1()[2] * M.r1()[2]);
    float phi = len == 0 ? 0 : signe(-1 * M.r0()[2]) * acos((float)(M.r1()[2] / sqrt(len)));

    S[2] = Matrix(1, -tan(phi / 2), 0, 0, 1, 0, 0, 0, 1);
    S[3] = Matrix(1, 0, 0, sin(phi), 1, 0, 0, 0, 1);
    S[4] = S[2];

    Matrix R1inv(cos(phi), sin(phi), 0, -sin(phi), cos(phi), 0, 0, 0, 1);

    M = matProd(R1inv, M);

    if (abs((int)M.r1()[2]) > abs((int)M.r2()[2]))
        S[5] = D4;
    else
        S[5] = D1;

    M = matProd(S[5], M);

    if (M.r2()[2] < 0)
        S[6] = D5;
    else
        S[6] = D1;

    M = matProd(S[6], M);

    // Determine second rotation angle psi.
    len = (M.r1()[2] * M.r1()[2]) + (M.r2()[2] * M.r2()[2]);
    float psi = len == 0 ? 0 : signe(-1 * M.r1()[2]) * acos((float)(M.r2()[2] / sqrt(len)));

    S[7] = Matrix(1, 0, 0, 0, 1, -tan(psi / 2), 0, 0, 1);
    S[8] = Matrix(1, 0, 0, 0, 1, 0, 0, sin(psi), 1);
    S[9] = S[7];

    Matrix R2inv(1, 0, 0, 0, cos(psi), sin(psi), 0, -sin(psi), cos(psi));

    M = matProd(R2inv, M);

    if (abs((int)M.r0()[1]) > abs((int)M.r1()[1]))
        S[10] = D2;
    else
        S[10] = D1;

    M = matProd(S[10], M);

    if (M.r1()[1] < 0)
        S[11] = D3;
    else
        S[11] = D1;

    M = matProd(S[11], M);

    // Determine last rotation angle theta.
    len = (M.r0()[1] * M.r0()[1]) + (M.r1()[1] * M.r1()[1]);
    float theta = len == 0 ? 0 : signe(-1 * M.r0()[1]) * acos((float)(M.r1()[1] / sqrt(len)));

    S[12] = Matrix(1, -tan(theta / 2), 0, 0, 1, 0, 0, 0, 1);
    S[13] = Matrix(1, 0, 0, sin(theta), 1, 0, 0, 0, 1);
    S[14] = S[12];

    return S;
}

static void determineFrenetFrame(MCGAL::Vector3 v1, const MCGAL::Vector3 v2, MCGAL::Vector3& normal, MCGAL::Vector3& t1, MCGAL::Vector3& t2) {
    MCGAL::Vector3 gateVector = v1 - v2;
    // Determine t1.
    double mid1 = gateVector | normal;
    t1 = gateVector - normal * mid1;
    double f_t1Length = sqrt(t1 | t1);
    if (f_t1Length != 0)
        t1 = t1 / f_t1Length;

    // Determine t2;
    t2 = normal % t1;
}

static Vector3i frenetRotation(Vector3i& Dist, Vector3& T1, Vector3& T2, Vector3& normal) {
    std::vector<Matrix> S = compRotationMat(T1, T2, normal);

    Vector3 u(Dist[0], Dist[1], Dist[2]);
    Matrix m_inter;

    // Procedure of the bijection.
    for (int i = 0; i < 15; i++) {
        if (i == 0 || i == 1 || i == 5 || i == 6 || i == 10 || i == 11)
            m_inter = S[i];
        else
            m_inter = Matrix(S[i].r0()[0], -S[i].r0()[1], -S[i].r0()[2], -S[i].r1()[0], S[i].r1()[1], -S[i].r1()[2], -S[i].r2()[0], -S[i].r2()[1],
                             S[i].r2()[2]);
        u = m_inter * u;

        int x = round(u[0]);
        int y = round(u[1]);
        int z = round(u[2]);

        u = Vector3(x, y, z);
    }

    return Vector3i(u[0], u[1], u[2]);
}

static Vector3i invFrenetRotation(Vector3i& Frenet, Vector3& T1, Vector3& T2, Vector3& normal) {
    std::vector<Matrix> S = compRotationMat(T1, T2, normal);

    Vector3 u(Frenet[0], Frenet[1], Frenet[2]);
    Matrix m_inter;

    for (int i = 14; i > -1; i--) {
        m_inter = S[i];
        u = m_inter * u;

        int x = round(u[0]);
        int y = round(u[1]);
        int z = round(u[2]);

        u = Vector3(x, y, z);
    }

    return Vector3i(u[0], u[1], u[2]);
}

static MCGAL::Halfedge* next_boundary(MCGAL::Halfedge* boundary) {
    MCGAL::Halfedge* nxt = boundary->next();
    if (nxt->isBoundary()) {
        return nxt;
    }
    nxt = boundary->next()->opposite()->next();
    if (nxt->isBoundary()) {
        return nxt;
    }
    for (MCGAL::Halfedge* hit : *boundary->end_vertex()) {
        if (hit->isBoundary()) {
            return hit;
        }
    }
    return nullptr;
}

static MCGAL::Halfedge* next_boundary(int ogroupId, MCGAL::Halfedge* boundary) {
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

static MCGAL::Halfedge* next_boundary(int inner, int outer, MCGAL::Halfedge* boundary) {
    MCGAL::Halfedge* nxt = boundary->next();
    if (nxt->isBoundary()) {
        if (nxt->face()->groupId() == inner && nxt->opposite()->face()->groupId() == outer) {
            return nxt;
        }
    }
    nxt = boundary->next()->opposite()->next();
    if (nxt->isBoundary()) {
        if (nxt->face()->groupId() == inner && nxt->opposite()->face()->groupId() == outer) {
            return nxt;
        }
    }
    for (MCGAL::Halfedge* hit : *boundary->end_vertex()) {
        if (hit->opposite()->isBoundary() && hit->isBoundary() && hit->face()->groupId() == inner && hit->opposite()->face()->groupId() == outer) {
            return hit;
        }
    }
    return nullptr;
}

static void vertexSplit(std::shared_ptr<Mesh> mesh, PerformedOperation& op) {
    op.vr->setNotRemoved();
    // op.vr->setPoint(mesh->intPosToFloat(mesh->floatPosToInt(op.vr)));
    mesh->add_vertex(op.vr);
    mesh->new_vertex_split(op.vr, op.v1, op.vhPivotL, op.vhPivotR);

    op.vr->setGroupId(op.v1->groupId());

    for (Halfedge* hit : *op.vr) {
        hit->face()->updateNormal();
    }

// Checks
#if 0
    assert(op.vrNeighborsBefore.size() == p_dmesh->valence(op.vr));
    for (set<DeciMesh::VertexHandle>::iterator vhIt = op.vrNeighborsBefore.begin();
         vhIt != op.vrNeighborsBefore.end(); ++vhIt)
        assert(p_dmesh->areVerticesConnected(op.vr, *vhIt));
#endif
}

static bool isPointInTriangle(const MCGAL::Point& A, const MCGAL::Point& B, const MCGAL::Point& C, const MCGAL::Point& P) {
    // 计算三个向量叉积
    MCGAL::Vector3 cross1 = crossProduct(A, B, P);
    MCGAL::Vector3 cross2 = crossProduct(B, C, P);
    MCGAL::Vector3 cross3 = crossProduct(C, A, P);

    // 检查是否所有叉积的z分量符号相同
    bool hasNeg = (cross1.z() < 0) || (cross2.z() < 0) || (cross3.z() < 0);
    bool hasPos = (cross1.z() > 0) || (cross2.z() > 0) || (cross3.z() > 0);

    return !(hasNeg && hasPos);  // 如果叉积符号不一致，返回false
}

constexpr double SELF_EDGE_SCORE = 6.3;
constexpr double EPS = 1e-5;

static double opposite_corner_angle(MCGAL::Halfedge* hit) {
    // MCGAL::Vertex* vi = hit->vertex();
    // MCGAL::Vertex* vj = hit->end_vertex();
    // MCGAL::Halfedge* next = hit->next();
    // MCGAL::Vertex* vk = next->end_vertex();

    // MCGAL::Vector3 pi(vi);
    // MCGAL::Vector3 pj(vj);
    // MCGAL::Vector3 pk(vk);

    // MCGAL::Vector3 v1 = pi - pk;
    // MCGAL::Vector3 v2 = pj - pk;

    // double cos_theta = v1.normalize().dot(v2.normalize());
    // cos_theta = std::max(-1.0, std::min(1.0, cos_theta));
    // double res = std::acos(cos_theta);

    double lij, ljk, lki;
    lij = hit->length();
    ljk = hit->next()->length();
    lki = hit->next()->next()->length();

    // uniformly scale the edge to increase numerics
    double inv_lsum = 1.0 / (lij + ljk + lki);
    lij *= inv_lsum;  // equiv to lij /= lsum
    ljk *= inv_lsum;  // equiv to ljk /= lsum
    lki *= inv_lsum;  // equiv to lki /= lsum

    // law of cosines
    double d = (ljk * ljk + lki * lki - lij * lij) / (2 * ljk * lki);
    // compute arccos
    if (abs(d) <= 1) {
        // satisfy triangle inequality
        return acos(d);
    } else if (abs(d) > 1 && abs(d) < (1 + EPS)) {
        // slightly violate triangle inequality, clip it
        d = std::max(-1.0, std::min(d, 1.0));  // clip to -1~1
        return acos(d);
    } else {
        std::cout << "WARNING: In opposite_corner_angle.cpp, the triangle violates triangle inequality" << std::endl;
        return std::numeric_limits<double>::quiet_NaN();
    }
}

static bool is_diamond_convex(MCGAL::Halfedge* fs) {
    MCGAL::Halfedge* fs_next = fs->next();
    MCGAL::Halfedge* fs_next_next = fs_next->next();
    MCGAL::Halfedge* fs_twin = fs->opposite();
    MCGAL::Halfedge* fs_twin_next = fs_twin->next();
    MCGAL::Halfedge* fs_twin_next_next = fs_twin_next->next();

    double angle_i_jk = opposite_corner_angle(fs_next);
    double angle_i_lj = opposite_corner_angle(fs_twin_next_next);

    double angle_j_ki = opposite_corner_angle(fs_next_next);
    double angle_j_il = opposite_corner_angle(fs_twin_next);

    double angle_i = angle_i_jk + angle_i_lj;
    double angle_j = angle_j_ki + angle_j_il;
    return (angle_i < (M_PI + EPS) && angle_j < (M_PI + EPS));
}

static int argmax_vec(const std::vector<double>& vec) {
    return std::max_element(vec.begin(), vec.end()) - vec.begin();
}

static void face_n_ring_faces(MCGAL::Facet* f, const int num_rings, std::vector<MCGAL::Facet*>& faces) {
    std::set<MCGAL::Facet*> fIdx_set;
    fIdx_set.insert(f);

    for (int iter = 0; iter < num_rings; iter++) {
        std::set<MCGAL::Facet*> to_append_fIdx_set;
        for (MCGAL::Facet* f : fIdx_set) {
            for (MCGAL::Halfedge* hit : *f) {
                to_append_fIdx_set.insert(hit->opposite()->face());
            }
        }
        fIdx_set.insert(to_append_fIdx_set.begin(), to_append_fIdx_set.end());
    }

    // turn set into vectorXi
    faces.resize(fIdx_set.size());
    int ii = 0;
    for (MCGAL::Facet* f : fIdx_set) {
        faces[ii] = f;
        ii += 1;
    }
}

static bool is_delaunay(MCGAL::Halfedge* hit) {
    double theta_A = opposite_corner_angle(hit);
    double theta_B = opposite_corner_angle(hit->opposite());
    return ((theta_A + theta_B) <= (M_PI + EPS));
}

static void get_edge_lengths(MCGAL::Vertex* v,
                             std::vector<std::vector<double>>& l_012,
                             std::vector<std::vector<double>>& scale_012,
                             std::vector<MCGAL::Halfedge*>& all_one_ring_fs) {
    for (MCGAL::Halfedge* hit : *v) {
        all_one_ring_fs.push_back(hit);
    }
    int nF_local = all_one_ring_fs.size();
    l_012.resize(nF_local, std::vector<double>(3));
    scale_012.resize(nF_local, std::vector<double>(3));
    for (int ii = 0; ii < nF_local; ii++) {
        MCGAL::Halfedge* fs = all_one_ring_fs[ii];
        l_012[ii][0] = fs->length();

        MCGAL::Vertex* vi = fs->vertex();
        MCGAL::Vertex* vj = fs->end_vertex();
        if (vi == v) {
            scale_012[ii][0] += 1;
        }
        if (vj == v) {
            scale_012[ii][0] += 1;
        }
        MCGAL::Halfedge* fs_next = fs->next();
        l_012[ii][1] = fs_next->length();
        vi = fs_next->vertex();
        vj = fs_next->end_vertex();
        if (vi == v) {
            scale_012[ii][1] += 1;
        }
        if (vj == v) {
            scale_012[ii][1] += 1;
        }
        MCGAL::Halfedge* fs_next_next = fs_next->next();
        l_012[ii][2] = fs_next_next->length();
        vi = fs_next_next->vertex();
        vj = fs_next_next->end_vertex();
        if (vi == v) {
            scale_012[ii][2] += 1;
        }
        if (vj == v) {
            scale_012[ii][2] += 1;
        }
    }
}

/*
使用纯 STL 实现的 CETM (Conformal Equiareal Triangle Mesh) 扁平化算法
不依赖任何第三方库（如 Eigen）

数据结构说明：
- l012: 一个 nF x 3 的矩阵，用 std::vector<std::vector<double>> 表示
  每一行存储一个三角形面的三条边长 [l0, l1, l2]
- s012: 一个 nF x 3 的矩阵，存储每条边与目标顶点的接触次数（缩放因子）
  用于决定哪些边需要缩放
- u: 共形缩放因子（输出）
*/

// ========= 辅助函数：从边长计算对顶角 =========
/*
功能：根据三角形的三条边长，计算与 lopp 相对的角（对顶角）
输入：
  - lopps: 对边长度数组
  - lbs: 第一条邻边长度数组
  - lcs: 第二条邻边长度数组
输出：
  - angles: 对顶角数组（弧度制）
算法：使用余弦定理
*/
static void opposite_angles_from_lengths(const std::vector<double>& lopps,
                                         const std::vector<double>& lbs,
                                         const std::vector<double>& lcs,
                                         std::vector<double>& angles) {
    int nF = lopps.size();
    angles.resize(nF);

    for (int ii = 0; ii < nF; ii++) {
        double lopp = lopps[ii];
        double lb = lbs[ii];
        double lc = lcs[ii];

        // 归一化边长（避免数值问题）
        double lsum = lopp + lb + lc;
        if (lsum == 0.0) {
            angles[ii] = 0.0;
            continue;
        }
        double scale = 1.0 / lsum;
        lopp = lopp * scale;
        lb = lb * scale;
        lc = lc * scale;

        // 使用余弦定理计算角度：cos(angle) = (b² + c² - a²) / (2bc)
        double d = (lb * lb + lc * lc - lopp * lopp) / (2.0 * lb * lc);

        // 限制 d 在 [-1, 1] 范围内（避免数值误差导致 acos 出错）
        d = std::max(-1.0, std::min(1.0, d));
        angles[ii] = std::acos(d);
    }
}

// ========= 从边长计算高斯曲率 =========
/*
功能：计算顶点处的高斯曲率
输入：
  - l012: nF x 3 矩阵，每个三角形面的三条边长
输出：
  - 高斯曲率值（对于内部顶点，应该是 2π - 角度和）
*/
static double gaussian_curvature_from_lengths(const std::vector<std::vector<double>>& l012) {
    int nF = l012.size();
    if (nF == 0)
        return 0.0;

    // 提取三列：l0, l1, l2
    std::vector<double> l0(nF), l1(nF), l2(nF);
    for (int i = 0; i < nF; i++) {
        l0[i] = l012[i][0];
        l1[i] = l012[i][1];
        l2[i] = l012[i][2];
    }

    // 计算对顶角（与 l0 相对的角）
    std::vector<double> angles;
    opposite_angles_from_lengths(l0, l1, l2, angles);

    // 计算角度和
    double angle_sum = std::accumulate(angles.begin(), angles.end(), 0.0);

    // 对于内部顶点，高斯曲率 = 2π - 角度和
    return 2.0 * M_PI - angle_sum;
}

// ========= 计算牛顿法的步长 delta_u =========
/*
功能：使用牛顿法计算缩放因子的更新步长
输入：
  - l012: 当前缩放后的边长矩阵
输出：
  - 步长 du = gradient / (2 * hessian)
算法：
  - gradient = 2π - 角度和（当前的高斯曲率）
  - hessian = (cot(α) + cot(β)) 的和 / 4
*/
static double delta_u(const std::vector<std::vector<double>>& l012) {
    int nF = l012.size();
    if (nF == 0)
        return 0.0;

    // 提取三列
    std::vector<double> l0(nF), l1(nF), l2(nF);
    for (int i = 0; i < nF; i++) {
        l0[i] = l012[i][0];
        l1[i] = l012[i][1];
        l2[i] = l012[i][2];
    }

    // ========= 计算梯度（gradient） =========
    // 梯度 = 2π - 角度和（当前的高斯曲率）
    std::vector<double> angles;
    opposite_angles_from_lengths(l1, l2, l0, angles);
    double gradient = 2.0 * M_PI - std::accumulate(angles.begin(), angles.end(), 0.0);

    // ========= 计算海塞矩阵（hessian） =========
    // 计算 α 角（与 l0 相对的角）的余切值
    std::vector<double> alphas, cot_alphas;
    opposite_angles_from_lengths(l0, l1, l2, alphas);
    cot_alphas.resize(nF);
    for (int ii = 0; ii < nF; ii++) {
        if (alphas[ii] <= 0.0 || alphas[ii] >= M_PI) {
            cot_alphas[ii] = 0.0;  // 退化情况
        } else {
            cot_alphas[ii] = 1.0 / std::tan(alphas[ii]);
        }
    }

    // 计算 β 角（与 l1 相对的角）的余切值
    std::vector<double> betas, cot_betas;
    opposite_angles_from_lengths(l2, l0, l1, betas);
    cot_betas.resize(nF);
    for (int ii = 0; ii < nF; ii++) {
        if (betas[ii] <= 0.0 || betas[ii] >= M_PI) {
            cot_betas[ii] = 0.0;  // 退化情况
        } else {
            cot_betas[ii] = 1.0 / std::tan(betas[ii]);
        }
    }

    // 计算海塞矩阵：hessian = (cot(α) + cot(β)) 的和 / 4
    double cot_alphas_sum = std::accumulate(cot_alphas.begin(), cot_alphas.end(), 0.0);
    double cot_betas_sum = std::accumulate(cot_betas.begin(), cot_betas.end(), 0.0);
    double hessian = (cot_betas_sum + cot_alphas_sum) / 4.0;

    // 避免除零
    if (std::abs(hessian) < 1e-10) {
        return 0.0;
    }

    // 牛顿法步长：du = gradient / (2 * hessian)
    return gradient * 0.5 / hessian;
}

// ========= 缩放边长 =========
/*
功能：根据缩放因子 u 和接触次数 s012，对边长进行缩放
输入：
  - l012: 原始边长矩阵
  - s012: 每条边与目标顶点的接触次数矩阵（决定缩放强度）
  - u: 共形缩放因子
输出：
  - l012_scaled: 缩放后的边长矩阵
算法：
  - 对于每条边，缩放因子 = exp(s * u / 2)
  - 其中 s 是该边与目标顶点的接触次数（0, 1, 或 2）
*/
static void scale_edge_lengths(const std::vector<std::vector<double>>& l012,
                               const std::vector<std::vector<double>>& s012,
                               const double& u,  // conformal scaling factor
                               std::vector<std::vector<double>>& l012_scaled) {
    int nF = l012.size();
    if (nF == 0) {
        l012_scaled.clear();
        return;
    }

    l012_scaled.resize(nF);
    for (int i = 0; i < nF; i++) {
        l012_scaled[i].resize(3);
        for (int j = 0; j < 3; j++) {
            // 计算缩放因子：exp(s * u / 2)
            double scale_factor = std::exp(s012[i][j] * u / 2.0);
            // 应用缩放
            l012_scaled[i][j] = l012[i][j] * scale_factor;
        }
    }
}

// ========= 主函数：CETM 扁平化内部顶点 =========
/*
功能：使用共形等面积三角网格（CETM）方法，通过缩放边长将内部顶点扁平化
     使得该顶点处的高斯曲率变为 0

输入：
  - l012: nF x 3 矩阵，每个三角形面的三条边长
  - s012: nF x 3 矩阵，每条边与目标顶点的接触次数（用于决定缩放强度）
输出：
  - u: 共形缩放因子（如果失败则返回 NaN）

算法：
  1. 使用牛顿法迭代求解缩放因子 u
  2. 每次迭代：
     a. 计算当前高斯曲率
     b. 如果已收敛（曲率接近 0），退出
     c. 计算牛顿法步长 du
     d. 尝试更新 u，如果导致 NaN 则回溯（减小步长）
  3. 如果达到最大迭代次数仍未收敛，返回 NaN
*/
static void cetm_flatten_interior_vertex(const std::vector<std::vector<double>>& l012, const std::vector<std::vector<double>>& s012, double& u) {
    u = 0.0;                              // 初始缩放因子
    double convergence_threshold = 1e-7;  // 收敛阈值

    // 计算初始高斯曲率
    double Kv = gaussian_curvature_from_lengths(l012);

    // 迭代参数
    int max_iter = 50;       // 最大迭代次数
    int decrease_iter = 10;  // 回溯最大次数
    double rho = 0.5;        // 回溯时的步长衰减因子

    // 初始化缩放后的边长矩阵
    std::vector<std::vector<double>> l012_scaled = l012;

    // ========= 主迭代循环 =========
    for (int iter = 0; iter < max_iter; iter++) {
        // 检查是否已收敛
        if (std::abs(Kv) < convergence_threshold) {
            break;
        }

        // ========= 使用牛顿法计算下降步长 =========
        double du = delta_u(l012_scaled);

        // ========= 尝试更新缩放因子 =========
        double step_size = 1.0;  // 初始步长为 1.0（完整牛顿步）
        scale_edge_lengths(l012, s012, u - step_size * du, l012_scaled);
        Kv = gaussian_curvature_from_lengths(l012_scaled);

        // ========= 如果出现 NaN，进行回溯（backtracking） =========
        for (int jj = 0; jj < decrease_iter; jj++) {
            if (std::isnan(Kv)) {
                // 如果出现 NaN，减小步长并重新计算
                step_size *= rho;
                scale_edge_lengths(l012, s012, u - step_size * du, l012_scaled);
            } else {
                // 如果没有 NaN，接受这个步长，更新 u
                u = u - step_size * du;
                break;
            }
            // 重新计算高斯曲率
            Kv = gaussian_curvature_from_lengths(l012_scaled);
        }

        // ========= 如果回溯后仍然是 NaN，说明失败 =========
        if (std::isnan(Kv)) {
            // 违反了三角形不等式，无法继续
            u = std::numeric_limits<double>::quiet_NaN();
            return;
        }
    }

    // ========= 检查最终是否收敛 =========
    if (std::abs(Kv) >= convergence_threshold) {
        // 未能在最大迭代次数内收敛
        u = std::numeric_limits<double>::quiet_NaN();
        return;
    }

    // 成功收敛，u 已经更新为正确的值
}

static MCGAL::Point barycenter(MCGAL::Halfedge* heh_gate) {
    if (!heh_gate) return MCGAL::Point(0, 0, 0);
    MCGAL::Halfedge* heh = heh_gate;
    MCGAL::Vector3 barycenter;
    unsigned i_degree = 0;
    unsigned guard = 0;

    while (heh && ++guard <= 8192) {
        barycenter = barycenter + MCGAL::Vector3(heh->vertex()->point());
        i_degree++;
        if (!heh->next()) {
            barycenter = barycenter + MCGAL::Vector3(heh->end_vertex()->point());
            i_degree++;
            break;
        }
        heh = heh->next();
        if (heh == heh_gate) break;
    }

    if (guard > 8192) return MCGAL::Point(0, 0, 0);
    return i_degree == 0 ? MCGAL::Point(0, 0, 0) : barycenter / i_degree;
}

/**
 * Compute the normal of the polygon starting from a given halfedge.
 * 与原版 PPMC `MyMesh::computeNormal(Halfedge_const_handle)` 语义一致：
 *   n = sum_i (OP_i × OP_{i+1})  (i 沿 heh_gate 起的多边形顶点)
 * 然后归一化；若退化（叉积为零）返回零向量。
 *
 * 注意不要替换为 `Facet::computeNormal()`：后者从 proxyHalfedge 起算，
 * 起点不同会导致 Frenet 切向 t1 不同，从而编/解码两侧不一致。
 */
static MCGAL::Vector3 computeNormalFromHeh(MCGAL::Halfedge* heh_gate) {
    if (!heh_gate) return MCGAL::Vector3(0, 0, 0);
    MCGAL::Halfedge* heh = heh_gate;
    MCGAL::Vector3 n(0, 0, 0);
    unsigned guard = 0;
    while (heh && ++guard <= 8192) {
        const MCGAL::Point& p1 = heh->vertex()->point();
        const MCGAL::Point& p2 = heh->next()
                                      ? heh->next()->vertex()->point()
                                      : heh->end_vertex()->point();
        // OP × OP'
        MCGAL::Vector3 op(p1);
        MCGAL::Vector3 op2(p2);
        n = n + op.cross(op2);
        if (!heh->next()) break;
        heh = heh->next();
        if (heh == heh_gate) break;
    }
    if (guard > 8192) return MCGAL::Vector3(0, 0, 0);

    float len = n.length();
    if (len == 0)
        return MCGAL::Vector3(0, 0, 0);
    return MCGAL::Vector3(n[0] / len, n[1] / len, n[2] / len);
}

/**
 * Compute the surface of a triangle using Heron's formula.
 */
static float triangleSurface(const MCGAL::Point p[]) {
    // ⚠️ BUG FIX (P0):
    //   原实现写的是 sqrt((p[0]-p[1]).length())，由于 MCGAL::Point::length()
    //   已经返回欧氏长度（含 sqrt），外层多套一层 sqrt 会让 a/b/c = sqrt(edgeLen)，
    //   Heron 公式算出的"面积"将与实际面积无关，进而让 PPMC 基于面积均值的连接性
    //   预测准确率退化为随机。CGAL 原版是 sqrt(squared_length())，因此此处直接用
    //   一次 length() 即为正确边长。
    float a = (p[0] - p[1]).length();
    float b = (p[1] - p[2]).length();
    float c = (p[2] - p[0]).length();
    float s = (a + b + c) / 2;

    float f_sqSurface = s * (s - a) * (s - b) * (s - c);

    return f_sqSurface <= 0 ? 0 : sqrt(f_sqSurface);
}

/**
 * P3.b 对齐：原版 PPMC `MyMesh::edgeLen(heh)` 的 MCGAL 等价实现。
 * 返回 halfedge 两端点的浮点欧氏距离 sqrt((p1 - p2)²)。
 * 与 quantizedSquaredDistance（整数量化版本）并存，供浮点路径使用。
 */
static inline float edgeLen(MCGAL::Halfedge* heh) {
    const MCGAL::Point& p1 = heh->vertex()->point();
    const MCGAL::Point& p2 = heh->opposite()->vertex()->point();
    MCGAL::Vector3 d = p1 - p2;
    return d.length();
}

/**
 * Returns a face surface.
 */
static float faceSurface(MCGAL::Halfedge* heh) {
    if (!heh) return 0;
    std::vector<Point> polygonPos;
    polygonPos.reserve(heh->face() ? heh->face()->facet_degree() : 8);
    Vector3 bary(0, 0, 0);
    MCGAL::Halfedge* hIt = heh;
    unsigned guard = 0;
    while (hIt && ++guard <= 8192) {
        polygonPos.push_back(hIt->vertex()->point());
        bary = bary + Vector3(hIt->vertex()->point());
        if (!hIt->next()) {
            polygonPos.push_back(hIt->end_vertex()->point());
            bary = bary + Vector3(hIt->end_vertex()->point());
            break;
        }
        hIt = hIt->next();
        if (hIt == heh) break;
    }
    if (guard > 8192) return 0;

    const unsigned i_degree = static_cast<unsigned>(polygonPos.size());
    if (i_degree < 3) return 0;
    Point b = bary / i_degree;

    float f_ret = 0;
    for (unsigned i = 0; i < i_degree; i++) {
        Point trianglePos[3];
        trianglePos[0] = b;
        trianglePos[1] = polygonPos[i];
        trianglePos[2] = polygonPos[(i + 1) % i_degree];
        f_ret += triangleSurface(trianglePos);
    }

    assert(f_ret >= 0);

    return f_ret;
}

// ─── P2.a 对齐：整数化（量化坐标）几何度量 ────────────────────────────────────
// 问题：MCGAL 的 halfedge 迭代顺序与原版 CGAL Polyhedron_3 不完全镜像，导致浮点
//       `faceSurface` / `edgeLen` 在编/解码端累加顺序不同 → 浮点舍入差 → PPMC
//       connectivity prediction 在少量 face 上结论相反 → b_split 反解 → 拓扑崩。
//
// 解决：所有"用于比较大小 / 取均值"的几何度量，改为基于**量化整数坐标**计算：
//   - 边长度：返回 |v2 - v1|² (int64_t)
//   - 三角面积：用整数 Heron 展开 `16·S² = 4a²b² - (a² + b² - c²)²`
//             （a²/b²/c² 均为 int64 squared length），返回 int64 值
//   - 多边形面积：整数三角剖分累加
//
// 注意：PPMC 的预测判据只需要"哪个更接近均值"的比较和累加，**不需要真实面积数值**。
//       因此保留平方/16x 的比例因子完全 OK；只要两端使用相同整数公式，就字节级一致。

/// 返回 |P1 - P2|² （基于量化整数坐标），int64 避免 12-bit × 12-bit × 3 溢出。
static inline int64_t quantizedSquaredDistance(const MCGAL::Point& p1, const MCGAL::Point& p2,
                                                const MCGAL::Point& bboxMin, float f_quantStep) {
    // ⚠️ P2.b 修复：用 trunc 与 floatPosToInt/getQuantizedPos 严格一致。
    int64_t q1x = (int64_t)((p1.x() - bboxMin.x()) / f_quantStep);
    int64_t q1y = (int64_t)((p1.y() - bboxMin.y()) / f_quantStep);
    int64_t q1z = (int64_t)((p1.z() - bboxMin.z()) / f_quantStep);
    int64_t q2x = (int64_t)((p2.x() - bboxMin.x()) / f_quantStep);
    int64_t q2y = (int64_t)((p2.y() - bboxMin.y()) / f_quantStep);
    int64_t q2z = (int64_t)((p2.z() - bboxMin.z()) / f_quantStep);
    int64_t dx = q1x - q2x, dy = q1y - q2y, dz = q1z - q2z;
    return dx * dx + dy * dy + dz * dz;
}

/// P2.a 对齐：边（halfedge）在量化坐标下的 squared length。
/// 用于 `InsertedEdgeCodingStep` 的 edge length prediction。两端字节级一致。
static inline int64_t edgeLenSqQuant(MCGAL::Halfedge* h, const MCGAL::Point& bboxMin, float f_quantStep) {
    return quantizedSquaredDistance(h->vertex()->point(), h->opposite()->vertex()->point(),
                                     bboxMin, f_quantStep);
}

/// P2.a 对齐：三角形 (p1, p2, p3) 在量化坐标下的 16·S² （int64 精确值）。
/// Heron 展开：16·S² = 4·a²·b² − (a² + b² − c²)²  其中 a,b,c 为边长，a²/b²/c² 为 int64。
/// 由于 PPMC 只用该值做"谁更接近均值"的比较，返回 16·S² 不影响预测正确性。
static inline int64_t triangleSurfaceSqQuant(const MCGAL::Point& p1, const MCGAL::Point& p2,
                                              const MCGAL::Point& p3,
                                              const MCGAL::Point& bboxMin, float f_quantStep) {
    int64_t a2 = quantizedSquaredDistance(p1, p2, bboxMin, f_quantStep);
    int64_t b2 = quantizedSquaredDistance(p2, p3, bboxMin, f_quantStep);
    int64_t c2 = quantizedSquaredDistance(p3, p1, bboxMin, f_quantStep);
    int64_t t = a2 + b2 - c2;  // 可能为负
    // 16·S² = 4·a²·b² − t²
    // 为避免 int64 溢出（12-bit 坐标 → a²≤3·4095² ≈ 5·10^7，a²·b²≤2.5·10^15 < 2^63/4），
    // 在常规量化位宽下安全。对于更高位宽，可考虑 __int128，但 PPMC 默认 12-bit。
    int64_t sq = 4 * a2 * b2 - t * t;
    return sq < 0 ? 0 : sq;
}

/// P2.a 对齐：face 在量化坐标下的"面积度量"。
///
/// ⚠️ P2.b 修复：原实现 `Σ(16·S²_triangle_in_fan)` **不是 cycle-invariant**
///   ——对 deg≥5 polygon，fan triangulation 起点不同会得到不同的三角化形状，
///   每个三角的 16·S² 是非线性量，加和**依赖 cycle 起点**。实测 bunny R=8 后
///   deg=5/6 face 在编/解码两端因 face cycle 起点偏移导致 sQuant 微差 → 预测
///   反解 → 拓扑崩。
///
///   修复方案：改用 **second moment** `Σ |q_i - q_bary|²`（量化坐标空间下的
///   "顶点到中心的距离平方和"）。该量：
///     - 整数加法/平方累加，与 cycle 起点完全无关（环上和不变）；
///     - 与 face 几何尺度强相关（PPMC 论文里 "face surface" 的物理意义即 face 的
///       几何特征量，second moment 与之等价）；
///     - 编/解码两端 bit-identical，无需 reconstruct 拓扑严格镜像。
///
///   value 范围：12-bit 量化下单顶点距离 ≤ 4096·√3 ≈ 7100，平方 ≤ 5e7，
///   degree=10 累加 ≤ 5e8 ≪ 2^63 → int64 安全。
static inline int64_t faceSurfaceSqQuant(MCGAL::Halfedge* heh,
                                          const MCGAL::Point& bboxMin, float f_quantStep) {
    unsigned i_degree = heh->face()->facet_degree();
    if (i_degree < 3)
        return 0;

    // 累加各顶点的整数量化坐标 → 整数 barycenter（向下取整）。
    int64_t bx = 0, by = 0, bz = 0;
    std::vector<int64_t> px(i_degree), py(i_degree), pz(i_degree);
    MCGAL::Halfedge* hIt = heh;
    unsigned i = 0;
    do {
        const MCGAL::Point& p = hIt->vertex()->point();
        // ⚠️ P2.b 修复：与 floatPosToInt/getQuantizedPos 严格一致使用 trunc，
        //   而非 std::floor —— 见 `barycenterIntQuant` 同段注释。
        px[i] = (int64_t)((p.x() - bboxMin.x()) / f_quantStep);
        py[i] = (int64_t)((p.y() - bboxMin.y()) / f_quantStep);
        pz[i] = (int64_t)((p.z() - bboxMin.z()) / f_quantStep);
        bx += px[i]; by += py[i]; bz += pz[i];
        ++i;
        hIt = hIt->next();
    } while (hIt != heh);
    bx /= (int64_t)i_degree;
    by /= (int64_t)i_degree;
    bz /= (int64_t)i_degree;

    // P2.b 修复：second moment 累加（cycle-invariant）。
    int64_t total = 0;
    for (unsigned j = 0; j < i_degree; ++j) {
        int64_t dx = px[j] - bx;
        int64_t dy = py[j] - by;
        int64_t dz = pz[j] - bz;
        total += dx * dx + dy * dy + dz * dz;
    }
    return total;
}

/// P2.a-extend 对齐：整数版本 barycenter，避免浮点累加顺序差异导致的 ULP 级误差。
///   在 PPMC residual 计算中，`Q(barycenter)` 必须在编/解码两端 bit-identical，
///   否则 `Q(removedPos) = residual + Q(barycenter)` 在两端差 1 → 新顶点坐标差 1·step
///   → 后续轮次 sQuant 不同 → connectivity prediction 误判 → 拓扑崩。
///   此函数直接累加每个顶点的整数量化坐标（int64，加法可交换/结合，与 cycle 顺序无关），
///   最后整除 degree（向下取整）得到整数 barycenter。
static inline MCGAL::PointInt barycenterIntQuant(MCGAL::Halfedge* heh_gate,
                                                  const MCGAL::Point& bboxMin, float f_quantStep) {
    int64_t bx = 0, by = 0, bz = 0;
    int64_t cnt = 0;
    MCGAL::Halfedge* heh = heh_gate;
    do {
        const MCGAL::Point& p = heh->vertex()->point();
        // ⚠️ P2.b 修复：与 `Mesh::floatPosToInt` / `getQuantizedPos` 严格一致 ——
        //   后两者使用 C-style truncation `(int)((p - bboxMin)/step)`（PointInt 构造
        //   函数从 double 隐式 trunc）。原 P2.a-extend 用 `std::floor` 在某些边界
        //   值上 trunc 与 floor 表现不同（例如负值 -0.5：trunc → 0，floor → -1）。
        //   bunny 第一卦限正数下表面看似一致，但 reconstruct 创建的新 vertex 浮点
        //   位置 `bboxMin + (q + 0.5)*step` 因 IEEE-754 累加可能略低/略高于 (q+0.5)*step，
        //   trunc 与 floor 在 q+0.5 ≈ ε 区间会差 1 → baryQ 差 1 → residual 反算的
        //   absQ 差 1 → 新 vertex 整数坐标差 1·step。
        //   改为 (int64_t) trunc，使 barycenterIntQuant 与 floatPosToInt/getQuantizedPos
        //   完全同源、bit-identical。
        bx += (int64_t)((p.x() - bboxMin.x()) / f_quantStep);
        by += (int64_t)((p.y() - bboxMin.y()) / f_quantStep);
        bz += (int64_t)((p.z() - bboxMin.z()) / f_quantStep);
        cnt++;
        heh = heh->next();
    } while (heh != heh_gate);
    if (cnt == 0)
        return MCGAL::PointInt(0, 0, 0);
    return MCGAL::PointInt((int)(bx / cnt), (int)(by / cnt), (int)(bz / cnt));
}

}  // namespace MCGAL

#endif
