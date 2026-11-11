#ifndef VERTEX_H
#define VERTEX_H
#include "Facet.h"
#include "Halfedge.h"
#include "Point.h"
#include "VertexSplitNode.h"
#include <assert.h>
#include <list>
#include <set>
#include <stdexcept>
#include <stdint.h>
#include <stdlib.h>
#include <vector>

#define LIST_ITER

namespace MCGAL {

class Halfedge;
class Facet;
class Vertex;

#define VERTEX_BUCKET_SIZE 3

class Vertex : public Point {
    enum RemovedFlag { NotRemoved, Removed };
    enum RemovableFlag { NotRemovable, Removable };
    enum ProcessedFlag { NotProcessed, Processed };
    enum BoundaryFlag { NotBoundary, Boundary };
    enum CollapsedFlag { NotCollapsed, Collapsed };
    enum ConqueredFlag { Unconquered = 0, Conquered = 1 };
    enum AdjustmentFlag { Unadjusted = 0, Adjusted = 1 };
    enum AnchorFlag { NotAnchor = 0, Anchor = 1 };
    // 2026-05-10 路径 B (PCLTTM Tag, BlobLayout=16) 唯一允许的 MCGAL 改动：
    // 在 Vertex 加 TagFlag 三态枚举（Untagged/Plus/Minus），与 removableFlag_/
    // conqueredFlag_ 同级。仅 PCLTTMTagEliminate/Reconstruct 路径会读写它，
    // 现有 9 个算法不读 → 字节级零回归。详见
    // docs/paper_alignment/PCLTTM_OFFICIAL_REPRO_PROMPT.md §R3。
  public:
    enum TagFlag : std::uint8_t { Untagged = 0, Plus = 1, Minus = 2 };
  private:

  public:
    bool forbidden = false;
    bool b_marked = false;
    Vertex() : Point() {}
    Vertex(const Point& p) : Point(p) {}
    Vertex(float v1, float v2, float v3) : Point(v1, v2, v3) {}

    int poolId() {
        return poolId_;
    }

    int groupId() {
        return groupId_;
    }

    void setPoolId(int poolId) {
        this->poolId_ = poolId;
    }

    // void setMeshId(int meshId) {
    //     this->meshId_ = meshId;
    // }

    int vertex_degree();
    void recompute_degree();

    inline void invalidate_degree() {
        degree_dirty_ = true;
    }

    inline bool is_degree_dirty() const {
        return degree_dirty_;
    }

    // ---- 增量维护的 O(1) 快路径 ----
    // 约束：使用者必须保证缓存已被正确初始化（比如一次 recompute_degree()）。
    // 读取：直接返回缓存值，不会触发重算。
    inline int vertex_degree_cached() const {
        return vertex_degree_;
    }

    // 强制设置缓存值（便于外部在增量更新后直接写入）
    inline void setCachedDegree(int d) {
        vertex_degree_ = d;
        degree_dirty_ = false;
    }

    // 邻接变化时的 O(1) 调整：相邻顶点新增 / 丢掉一条半边。
    // 调用方需保证缓存值已被 recompute_degree() 初始化过。
    inline void incCachedDegree(int delta = 1) {
        if (!degree_dirty_) {
            vertex_degree_ += delta;
        }
    }

    inline void decCachedDegree(int delta = 1) {
        if (!degree_dirty_) {
            vertex_degree_ -= delta;
            if (vertex_degree_ < 0) vertex_degree_ = 0;
        }
    }

    void print() {
        printf("%f %f %f\n", v[0], v[1], v[2]);
    }

    void setVid(int id) {
        this->vid_ = id;
    }

    void setGroupId(int id) {
        this->groupId_ = id;
    }

    int vid() {
        return vid_;
    }

    float x() const {
        return v[0];
    }

    float y() const {
        return v[1];
    }

    float z() const {
        return v[2];
    }

    int vid() const {
        return vid_;
    }

    void setOriginVertexId(int id) {
        originVertexId_ = id;
    }

    int originVertexId() const {
        return originVertexId_;
    }

    int globalVertexId() const {
        return originVertexId_ >= 0 ? originVertexId_ : poolId_;
    }

    Point point() {
        return Point(this->v[0], this->v[1], this->v[2]);
    }

    Point* pointptr() {
        return (Point*)this;
    }

    void setPoint(const Point& p) {
        this->v[0] = p.x();
        this->v[1] = p.y();
        this->v[2] = p.z();
    }

    void setPoint(float x, float y, float z) {
        this->v[0] = x;
        this->v[1] = y;
        this->v[2] = z;
    }

    void setPoint(float x, float y, float z, int id) {
        this->v[0] = x;
        this->v[1] = y;
        this->v[2] = z;
    }

    virtual void resetState() {
        this->processedFlag_ = NotProcessed;
        this->conquredFlag_ = Unconquered;
        this->collapsedFlag_ = NotCollapsed;
        this->adjustmentFlag_ = Unadjusted;
        this->removableFlag_ = (anchorFlag_ == Anchor) ? NotRemovable : Removable;
        // 注意：不在 resetState 中 invalidate degree。
        // degree 仅随拓扑变化（erase_center_vertex / triangulate）改变，
        // 与"状态标记"正交；每轮 reset 都 invalidate 会让所有 isRemovable 检查
        // 退化成 O(n)。拓扑修改点通过 incCachedDegree/decCachedDegree 或
        // invalidate_degree 显式维护。
        // this->degree_dirty_ = true;
    }

    inline bool isAdjusted() const {
        return adjustmentFlag_ == Adjusted;
    }

    inline void setAdjusted() {
        adjustmentFlag_ = Adjusted;
    }

    inline void setUnadjusted() {
        adjustmentFlag_ = Unadjusted;
    }

    inline bool isConquered() const {
        return conquredFlag_ == Conquered;
    }

    inline void setConquered() {
        conquredFlag_ = Conquered;
    }

    inline void setUnConquered() {
        conquredFlag_ = Unconquered;
    }

    inline bool isBoundary() const {
        return boundaryFlag_ == Boundary;
    }

    inline void setBoundary() {
        boundaryFlag_ = Boundary;
    }

    inline void setNotBoundary() {
        boundaryFlag_ = NotBoundary;
    }

    inline void setNotRemoved() {
        removedFlag_ = NotRemoved;
    }

    inline void setRemoved() {
        removedFlag_ = Removed;
    }

    inline void setRemovable() {
        removableFlag_ = Removable;
    }

    inline void setNotRemovable() {
        removableFlag_ = NotRemovable;
    }

    inline bool isRemovable() {
        return removableFlag_ == Removable;
    }

    inline void setAnchor() {
        anchorFlag_ = Anchor;
        removableFlag_ = NotRemovable;
    }

    inline void clearAnchor() {
        anchorFlag_ = NotAnchor;
        removableFlag_ = Removable;
    }

    inline bool isAnchor() const {
        return anchorFlag_ == Anchor;
    }

    inline bool isRemoved() {
        return removedFlag_ == Removed;
    }

    inline void setCollapsed() {
        collapsedFlag_ = Collapsed;
    }

    inline bool isCollapsed() {
        return collapsedFlag_ == Collapsed;
    }

    inline void setProcessed() {
        processedFlag_ = Processed;
    }

    inline bool isProcessed() {
        return processedFlag_ == Processed;
    }

    // -------- 2026-05-10 PCLTTM Tag (path-B) 4 个 inline 方法 --------
    // 仅 PCLTTMTagEliminate/Reconstruct 路径调用，现有 9 个算法不读不写。
    // 详见 docs/paper_alignment/PCLTTM_OFFICIAL_REPRO_PROMPT.md §R3。
    inline TagFlag getTag() const {
        return tagFlag_;
    }

    inline void setTag(TagFlag t) {
        tagFlag_ = t;
    }

    inline void clearTag() {
        tagFlag_ = Untagged;
    }

    inline bool hasTag() const {
        return tagFlag_ != Untagged;
    }
    // -----------------------------------------------------------------

#ifdef LIST_ITER
    class halfedge_iterator {
      private:
        Halfedge* current_;
        Halfedge* start_;
        bool is_first_iteration_;
        bool switched_to_ccw_ = false;

      public:
        // Constructor
        halfedge_iterator(Halfedge* start = nullptr) : current_(start), start_(start), is_first_iteration_(true) {}

        // Dereference operator
        Halfedge* operator*() const {
            return current_;
        }

        // Pre-increment operator
        halfedge_iterator& operator++();

        // Post-increment operator
        halfedge_iterator operator++(int) {
            halfedge_iterator tmp = *this;
            ++(*this);
            return tmp;
        }

        // Equality comparison
        bool operator==(const halfedge_iterator& other) const {
            return current_ == other.current_;
        }

        // Inequality comparison
        bool operator!=(const halfedge_iterator& other) const {
            return !(*this == other);
        }
    };
#endif

#ifdef VECTOR_ITER
    class halfedge_iterator {
      private:
        Halfedge* current_;
        Halfedge* start_;
        bool is_first_iteration_;
        bool switched_to_ccw_ = false;

      public:
        // Constructor
        halfedge_iterator(Halfedge* start = nullptr) : current_(start), start_(start), is_first_iteration_(true) {}

        // Dereference operator
        Halfedge* operator*() const {
            return current_;
        }

        // Pre-increment operator
        halfedge_iterator& operator++();

        // Post-increment operator
        halfedge_iterator operator++(int) {
            halfedge_iterator tmp = *this;
            ++(*this);
            return tmp;
        }

        // Equality comparison
        bool operator==(const halfedge_iterator& other) const {
            return current_ == other.current_;
        }

        // Inequality comparison
        bool operator!=(const halfedge_iterator& other) const {
            return !(*this == other);
        }
    };
#endif

    // Iterator begin method - returns iterator pointing to the first halfedge
    halfedge_iterator begin() const {
        return halfedge_iterator(proxyHalfedge_);
    }

    // Iterator end method - returns iterator representing past-the-end
    halfedge_iterator end() const {
        return halfedge_iterator(nullptr);
    }

    Halfedge* proxyHalfedge() {
        return proxyHalfedge_;
    }

    void setProxyHalfedge(MCGAL::Halfedge* hit) {
        proxyHalfedge_ = hit;
    }

    void setVertexPoint(const MCGAL::Point& point) {
        vertexPoint_ = point;
    }

    MCGAL::Point vertexPoint() const {
        return vertexPoint_;
    }

    inline bool isVisited(int current_version) const {
        return visited_version == current_version;
    }

    inline void setVisited(int current_version) {
        visited_version = current_version;
    }

    bool isConnected(Vertex* v) const;

    MCGAL::Halfedge* find_halfedge(Vertex* v);

    std::vector<Halfedge*>& halfedges() {
        return halfedges_;
    };

    // Iterator for adjacent vertices through facets
    class vertex_iterator {
      private:
        const Vertex* self_;                          // The vertex we're iterating neighbors for
        halfedge_iterator halfedge_it_;               // Iterator over halfedges of self_
        Facet::halfedge_iterator facet_halfedge_it_;  // Iterator over halfedges of current facet
        std::set<Vertex*> visited_;                   // Track visited vertices to avoid duplicates
        Vertex* current_vertex_;                      // Current vertex being pointed to

        // Find the next valid adjacent vertex
        void advance() {
            while (true) {
                // If we're currently iterating through a facet's halfedges
                if (facet_halfedge_it_ != Facet::halfedge_iterator(nullptr)) {
                    while (facet_halfedge_it_ != Facet::halfedge_iterator(nullptr)) {
                        Halfedge* h = *facet_halfedge_it_;
                        if (h != nullptr) {
                            Vertex* v = h->end_vertex();
                            // Check if vertex is valid, not self, and not visited
                            if (v != nullptr && v != self_ && visited_.find(v) == visited_.end()) {
                                visited_.insert(v);
                                current_vertex_ = v;
                                ++facet_halfedge_it_;
                                return;
                            }
                        }
                        ++facet_halfedge_it_;
                    }
                    // Finished with current facet, reset facet iterator
                    facet_halfedge_it_ = Facet::halfedge_iterator(nullptr);
                }

                // Move to next halfedge of self_
                if (halfedge_it_ == halfedge_iterator(nullptr)) {
                    // No more halfedges, iteration complete
                    current_vertex_ = nullptr;
                    return;
                }

                Halfedge* h = *halfedge_it_;
                ++halfedge_it_;

                if (h != nullptr) {
                    // Get the facet from this halfedge
                    Facet* face = h->face();
                    if (face != nullptr) {
                        // Start iterating through this facet's halfedges
                        facet_halfedge_it_ = face->begin();
                        // Continue loop to process this facet's vertices
                        continue;
                    }
                    // If halfedge has no face (boundary case where opposite is nullptr),
                    // we skip it as per requirement to get vertices through facets
                }
            }
        }

      public:
        // Constructor
        vertex_iterator(const Vertex* self, Halfedge* start_halfedge)
            : self_(self), halfedge_it_(start_halfedge), facet_halfedge_it_(nullptr), current_vertex_(nullptr) {
            if (start_halfedge != nullptr) {
                advance();
            }
        }

        // End iterator constructor
        vertex_iterator() : self_(nullptr), halfedge_it_(nullptr), facet_halfedge_it_(nullptr), current_vertex_(nullptr) {}

        // Dereference operator
        Vertex* operator*() const {
            return current_vertex_;
        }

        // Pre-increment operator
        vertex_iterator& operator++() {
            advance();
            return *this;
        }

        // Post-increment operator
        vertex_iterator operator++(int) {
            vertex_iterator tmp = *this;
            ++(*this);
            return tmp;
        }

        // Equality comparison
        bool operator==(const vertex_iterator& other) const {
            return current_vertex_ == other.current_vertex_;
        }

        // Inequality comparison
        bool operator!=(const vertex_iterator& other) const {
            return !(*this == other);
        }
    };

    // Iterator begin method - returns iterator pointing to the first adjacent vertex
    vertex_iterator vertices_begin() const {
        return vertex_iterator(this, proxyHalfedge_);
    }

    // Iterator end method - returns iterator representing past-the-end
    vertex_iterator vertices_end() const {
        return vertex_iterator();
    }

    // bool eraseHalfedgeByPointer(Halfedge* halfedge) {
    //     for (auto it = halfedges_.begin(); it != halfedges_.end();) {
    //         if ((*it) == halfedge) {
    //             halfedges_.erase(it);
    //             return true;
    //         } else {
    //             it++;
    //         }
    //     }
    //     return false;
    // }

    // void setVertexSplitNode(VertexSplitNode* node) {
    //     this->vsplitNode_ = node;
    // }

    // VertexSplitNode* vsplitNode() {
    //     return this->vsplitNode_;
    // }

  private:
    int vid_ = -1;
    int poolId_ = -1;
    int originVertexId_ = -1;
    // int meshId_ = -1;
    int groupId_ = 0;
    std::vector<Halfedge*> halfedges_;
    RemovedFlag removedFlag_ = NotRemoved;
    ProcessedFlag processedFlag_ = NotProcessed;
    CollapsedFlag collapsedFlag_ = NotCollapsed;
    ConqueredFlag conquredFlag_ = Unconquered;
    BoundaryFlag boundaryFlag_ = NotBoundary;
    AdjustmentFlag adjustmentFlag_ = Unadjusted;
    RemovableFlag removableFlag_ = Removable;
    AnchorFlag anchorFlag_ = NotAnchor;

    // VertexSplitNode* vsplitNode_;
    MCGAL::Halfedge* proxyHalfedge_ = nullptr;
    MCGAL::Point vertexPoint_;
    int visited_version = 0;

    int vertex_degree_ = -1;
    bool degree_dirty_ = true;

    // 2026-05-10 PCLTTM Tag (path-B) 唯一新增字段。default = Untagged，
    // 现有 9 个算法不读 → 字节级零回归。
    TagFlag tagFlag_ = Untagged;
};

// 2026-05-04 性能优化：inline halfedge_iterator::operator++
//   perf 显示未 inline 前此函数占 RD bunny 33% CPU（且 find_prev 已 inline）。
//   所有调用者来自 `for (Halfedge* h : *vertex)`，每轮 halfedge_iterator::operator++
//   走 1~3 次 step_cw（=find_prev + opposite）。内联后编译器能把 find_prev 的
//   三角快捷路径、opposite/next 的单次 load 完全折叠到 caller，并消除函数调用开销。
//   Vertex.h 顶部已 include Halfedge.h，此处 Halfedge 完整可见。
inline Vertex::halfedge_iterator& Vertex::halfedge_iterator::operator++() {
    if (current_ == nullptr) {
        return *this;
    }

    auto step_cw = [](Halfedge* h) -> Halfedge* {
        if (h == nullptr)
            return nullptr;
        Halfedge* n2 = h->find_prev();
        return n2->opposite();  // prev(opposite(h)) in triangle meshes
    };
    auto step_ccw = [](Halfedge* h) -> Halfedge* {
        if (h == nullptr)
            return nullptr;
        Halfedge* opp = h->opposite();
        if (opp == nullptr)
            return nullptr;
        return opp->next();
    };

    if (is_first_iteration_) {
        is_first_iteration_ = false;
        Halfedge* next_cw = step_cw(current_);
        if (next_cw == nullptr) {
            switched_to_ccw_ = true;
            current_ = step_ccw(start_);
            return *this;
        }
        if (next_cw == start_) {
            current_ = nullptr;
            return *this;
        }
        current_ = next_cw;
        return *this;
    }

    if (!switched_to_ccw_) {
        Halfedge* next_cw = step_cw(current_);
        if (next_cw == nullptr) {
            switched_to_ccw_ = true;
            current_ = step_ccw(start_);
            return *this;
        }
        if (next_cw == start_) {
            current_ = nullptr;
            return *this;
        }
        current_ = next_cw;
        return *this;
    }

    Halfedge* next_ccw = step_ccw(current_);
    if (next_ccw == nullptr || next_ccw == start_) {
        current_ = nullptr;
        return *this;
    }
    current_ = next_ccw;
    return *this;
}

}  // namespace MCGAL

#endif
