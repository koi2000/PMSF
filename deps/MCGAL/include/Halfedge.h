#ifndef HALFEDGE_H
#define HALFEDGE_H
#include "Point.h"
#include <assert.h>
#include <atomic>
namespace MCGAL {
class Point;
class Vertex;
class Facet;

struct VertexRemovalEncodeSchema {
    int number;
    Point p;
};

class Halfedge {
  public:
    bool forbidden = false;
    bool b_collapseOk = true;
    bool b_dontCount = false;
    Halfedge(){};
    Halfedge(Vertex* v1, Vertex* v2);
    ~Halfedge();

    void setVertex(Vertex* v1, Vertex* v2);

    void reset(Vertex* v1, Vertex* v2);

    void reset_without_init(Vertex* v1, Vertex* v2);

    void reset_without_pushback(Vertex* v1, Vertex* v2);

    // 2026-05-04 性能优化：inline 定义放在 Facet.h 末尾，此处保留声明。
    //   perf 显示未 inline 前 find_prev 占 RD/MMSE/APTM 17% CPU（多数是 6 指令
    //   三角快捷路径的函数调用开销）。Facet.h 已包含 Halfedge.h，所以把定义放
    //   到 Facet.h 末尾是安全的 —— 所有 Halfedge::find_prev 调用点都通过
    //   MeshUtils.h / Mesh.h 间接包含 Facet.h。
    inline Halfedge* find_prev();

    Halfedge* next_boundary();

    inline void resetBfsFlag() {
        processedFlag_ = NotProcessed;
    }

    inline void resetState() {
        processedFlag_ = NotProcessed;
        addedFlag_ = Original;
        paperLevel_ = -1;
        b_collapseOk = true;
        flipTag_ = false;
    }

    Point getRemovedVertexPos() const;

    /* Processed flag */
    inline void resetProcessedFlag() {
        processedFlag_ = NotProcessed;
    }

    inline void setProcessed() {
        processedFlag_ = Processed;
    }

    inline void setUnProcessed() {
        processedFlag_ = NotProcessed;
    }

    inline bool isBoundary() const {
        return boundaryFlag_ == IsBoundary;
    }

    inline void setBoundary() {
        boundaryFlag_ = IsBoundary;
    }

    inline void setNotBoundary() {
        boundaryFlag_ = NotBoundary;
    }

    inline bool isVisited(int current_version) const {
        return visited_version == current_version;
    }

    inline void setVisited(int current_version) {
        visited_version = current_version;
    }

    inline bool isProcessed() const {
        return processedFlag_ == Processed;
    }

    inline void setRemoved() {
        removedFlag_ = Removed;
    }

    inline bool isRemoved() {
        return removedFlag_ == Removed;
    }

    // 自定义的比较函数，只比较名字
    bool operator==(const MCGAL::Halfedge& other) const {
        return poolId_ == other.poolId_;
    }

    inline Vertex*& vertex() {
        return vertex_;
    }

    inline void setVertex(Vertex* v) {
        this->vertex_ = v;
    }

    inline Vertex*& end_vertex() {
        return end_vertex_;
    }

    inline void setEndVertex(Vertex* v) {
        this->end_vertex_ = v;
    }

    inline Facet*& face() {
        return face_;
    }

    inline void setFace(Facet* f) {
        face_ = f;
    }

    inline Halfedge*& next() {
        return next_;
    }

    inline void setNext(Halfedge* h) {
        next_ = h;
    }

    inline Halfedge*& opposite() {
        return opposite_;
    }

    inline void setOpposite(Halfedge* h) {
        opposite_ = h;
    }

    inline int poolId() const {
        return poolId_;
    }

    inline void setPoolId(int& id) {
        poolId_ = id;
    }

    // inline int meshId() const {
    //     return meshId_;
    // }

    // inline void setMeshId(int id) {
    //     meshId_ = id;
    // }

    inline int groupId() const {
        return groupId_;
    }

    inline void setGroupId(int& id) {
        groupId_ = id;
    }

    float length();

    inline void resetAdded() {
        addedFlag_ = Original;
    }

    inline void setAdded() {
        assert(addedFlag_ == Original);
        addedFlag_ = Added;
    }

    inline bool isAdded() const {
        return addedFlag_ == Added;
    }

    inline void setNew() {
        addedFlag_ = New;
    }

    inline bool isNew() const {
        return addedFlag_ == New;
    }

    inline bool isOriginal() const {
        return addedFlag_ == Original;
    }
    inline void setOriginal() {
        addedFlag_ = Original;
        paperLevel_ = -1;
    }

    inline void setPaperLevel(int level) {
        paperLevel_ = level;
    }

    inline int paperLevel() const {
        return paperLevel_;
    }

    inline void setFlipTag() {
        flipTag_ = true;
    }

    inline void clearFlipTag() {
        flipTag_ = false;
    }

    inline bool flipTag() const {
        return flipTag_;
    }

    // inline bool hasRemovedVertex() {
    //     return removedVertex_ == nullptr;
    // }

    // void setRemovedVertex(Point* v) {
    //     this->removedVertex_ = v;
    // }

  private:
    enum ProcessedFlag { NotProcessed, Processed };
    enum RemovedFlag { NotRemoved, Removed };
    enum BoundaryFlag { NotBoundary, IsBoundary };
    enum AddedFlag { Original, Added, New };

    AddedFlag addedFlag_ = Original;
    ProcessedFlag processedFlag_ = NotProcessed;
    RemovedFlag removedFlag_ = NotRemoved;
    BoundaryFlag boundaryFlag_ = NotBoundary;

    Vertex* vertex_ = nullptr;
    Vertex* end_vertex_ = nullptr;
    Facet* face_ = nullptr;
    Halfedge* next_ = nullptr;
    Halfedge* opposite_ = nullptr;

    // Point* removedVertex_ = nullptr;
    int visited_version = 0;
    int paperLevel_ = -1;

    int poolId_ = -1;
    // int meshId_ = -1;
    int groupId_ = -1;
    bool flipTag_ = false;
    // VertexRemovalEncodeSchema vSchema_;
};
}  // namespace MCGAL

#endif
