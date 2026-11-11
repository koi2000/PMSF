#ifndef MESH_H
#define MESH_H
#include "Configuration.h"
#include "Facet.h"
#include "Halfedge.h"
#include "JemallocStl.h"
#include "Point.h"
#include "Vertex.h"
#include "VertexSplitNode.h"
#include <algorithm>
#include <climits>
#include <stdlib.h>
#include <string>
#include <vector>
namespace MCGAL {
class Vector3;
class Vertex;
class Halfedge;
class Facet;
class Point;

#define MESH_BUCKET_SIZE 4096
namespace detail {
// perf（2026-11-08，perf record 实测 floatPosToInt 内 getenv 占 21.5% cycles）：
// 量化步进相关的实验开关在进程内不会改变，缓存一次；语义与原实现完全一致
//（getenv 只要非空指针即为真，包括设置为空字符串的情况）。
inline bool quantNearIntEnabled() {
    static const bool enabled = (std::getenv("PMSF_QUANT_NEAR_INT") != nullptr);
    return enabled;
}
inline bool quantMaxClampEnabled() {
    static const bool enabled = (std::getenv("PMSF_QUANT_MAX_CLAMP") != nullptr);
    return enabled;
}
}  // namespace detail

class Mesh {
  public:
    Mesh(std::string path) {
        this->loadOFF(path);
    }
    Mesh() {
        faces_.reserve(MESH_BUCKET_SIZE);
        vertices_.reserve(MESH_BUCKET_SIZE);
    }
    ~Mesh();

    std::vector<Vertex*>& vertices() {
        return vertices_;
    }

    std::vector<Facet*>& faces() {
        return faces_;
    }

    // IOS
    bool loadOFF(std::string path);
    void dumpto(std::string path);
    void print();
    std::string to_string();
    inline Vertex* get_vertex(int vseq = 0) {
        return vertices_[vseq];
    };

    inline Facet* get_facet(int fseq = 0) {
        return faces_[fseq];
    };

    int meshId() {
        return meshId_;
    }

    void setMeshId(int meshId) {
        this->meshId_ = meshId;
    }

    static bool is_collapse_ok(MCGAL::Halfedge* v0v1);

    static bool IsFlipped(MCGAL::Halfedge* edge, const MCGAL::Point& ptTarget);

    bool new_is_collapse_ok(MCGAL::Halfedge* v0v1) const;

    void new_collapse(MCGAL::Halfedge* h);

    Halfedge* insert_loop(Halfedge* _hh);

    Halfedge* new_vertex_split(MCGAL::Vertex* v1, MCGAL::Vertex* v2, MCGAL::Vertex* v3, MCGAL::Vertex* v4);
    Halfedge* insert_edge(Vertex* _vh, Halfedge* _h0, Halfedge* _h1);

    void edge_flip(MCGAL::Halfedge* hit);

    // element operating
    Vertex* add_vertex(Vertex* vertex);
    Facet* add_face(std::vector<Vertex*>& vs);
    Facet* add_face(Facet* face);
    void eraseFacetByPointer(Facet* facet);
    void eraseVertexByPointer(Vertex* vertex);
    void markGroupId(MCGAL::Halfedge* seed, int groupId);
    void dumpto_oldtype(std::string path);
    void vertex_based_submesh_dumpto_oldtype(std::string path, int groupId);
    void facet_based_submesh_dumpto_oldtype(std::string path, int groupId);
    Halfedge* create_center_vertex_plus(Halfedge* h, Point point);

    void checkMesh();
    void checkMesh(int groupId);

    Facet* remove_vertex(Vertex* v);
    Halfedge* merge_edges(Vertex* v);

    size_t size_of_vertices() {
        return vertices_.size();
    }

    size_t size_of_facets() {
        return faces_.size();
    }

    void resetState();

    void invalidateAllVertexDegrees();
    void recomputeAllVertexDegrees();

    void garbage_collection();

    bool isEdgeCollapseValid(MCGAL::Halfedge* edge);

    // PHASE64 物品2：**内联**。函数体只有 3 次减法 + 3 次除法，但它是
    // **跨 TU 的成员函数调用**，实测在 N=2 分段解码里占 **24.80%**
    //（`lessHalfedgeKey` 每次比较要调它 ~4 次）。搬到头里之后调用点就地展开，
    // 只读成员 `bboxMin`/`f_quantStep`、无副作用 ⇒ **产物逐位不变**。
    PointInt floatPosToInt(Point p) {
        auto conv = [&](float x) -> int {
            float d = (x - bboxMin[0]) / f_quantStep;
            (void)d;
            return 0;
        };
        (void)conv;
        const bool nearInt = detail::quantNearIntEnabled();
        auto convAxis = [&](float x, int axis) -> int {
            float d = (x - bboxMin[axis]) / f_quantStep;
            if (nearInt) {
                const int t = static_cast<int>(d);
                if (d - static_cast<float>(t) > 0.999f) d = static_cast<float>(t + 1);
            }
            return static_cast<int>(d);
        };
        PointInt q(convAxis(p[0], 0), convAxis(p[1], 1), convAxis(p[2], 2));
        // Optional experiment: the base-mesh writer stores q in nbQuantBits
        // bits and clamps the maximum to 2^bits-1.  If the encoder's in-memory
        // q is 2^bits for a bbox-max vertex, encoder and decoder disagree by
        // one quantization step.  Clamp here as well when requested.
        if (enableQuantization && detail::quantMaxClampEnabled()) {
            const int maxQ = (1 << i_nbQuantBits) - 1;
            q = PointInt(std::min(std::max(q[0], 0), maxQ),
                         std::min(std::max(q[1], 0), maxQ),
                         std::min(std::max(q[2], 0), maxQ));
        }
        return q;
    }

    MCGAL::Point intPosToFloat(PointInt p);

    MCGAL::Vertex* halfedge_collapse(MCGAL::Halfedge* h, MCGAL::Point newp);

    MCGAL::Vertex* halfedge_collapse(MCGAL::Halfedge* h);

    MCGAL::Halfedge* vertex_split(MCGAL::Vertex* v, MCGAL::VertexSplitNode* node);

    Halfedge* split_facet(Halfedge* h, Halfedge* g);

    Halfedge* split_facet_non_meshId(Halfedge* h, Halfedge* g);

    Halfedge* create_center_vertex(Halfedge* h);

    Halfedge* create_center_vertex(Halfedge* h, Point point);

    // used for decode
    Halfedge* create_center_vertex_without_init(Halfedge* h, Point point);

    inline void close_tip(Halfedge* h, Vertex* v) const;

    inline void insert_tip(Halfedge* h, Halfedge* v) const;

    Halfedge* find_prev(Halfedge* h) const;

    Halfedge* erase_center_vertex(Halfedge* h);

    Halfedge* erase_center_vertex_plus(Halfedge* h);

    void set_face_in_face_loop(Halfedge* h, Facet* f) const;

    inline void remove_tip(Halfedge* h) const;

    Halfedge* join_face(Halfedge* h);

    MCGAL::Vector3 computeNormal();

    void driftAlongNormal(int step);

    void updateBboxMax(MCGAL::Point bMax);

    void updateBboxMin(MCGAL::Point bMin);

    void quantizeVertexPositions();

  public:
    JemallocVector<Vertex*> vertices_;
    JemallocVector<Facet*> faces_;
    int meshId_ = -1;
    int nb_vertices, nb_faces, nb_edges;
    MCGAL::Point bboxMin = MCGAL::Point(INT_MAX, INT_MAX, INT_MAX);
    MCGAL::Point bboxMax = MCGAL::Point(INT_MIN, INT_MIN, INT_MIN);
    bool enableQuantization = true;
    float f_quantStep;
    unsigned i_nbQuantBits = 12;
};
}  // namespace MCGAL
#endif