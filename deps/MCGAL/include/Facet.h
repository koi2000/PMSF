#ifndef FACET_H
#define FACET_H
#include "Halfedge.h"
#include "Point.h"
#include <assert.h>
#include <stdint.h>
#include <cstdlib>
#include <set>
#include <vector>
namespace MCGAL {
class Point;
class Vertex;
class Halfedge;
class Facet {
  private:
    enum ProcessedFlag { NotProcessed, Processed };
    enum RemovedFlag { NotRemoved, Removed };
    enum SplittableFlag { Unknown = 0, Splittable = 1, Unsplittable = 2 };

  public:
    // PCATM paper triangle colors.
    //
    // Four-color mode uses NullColor + three patch colors. Two-color mode
    // reuses the legacy RED/BLUE names as aliases for 0/1 so existing
    // PCATM_ZCOLOR/Paper code keeps compiling unchanged.
    enum TriangleColorFlag : uint8_t {
        NullColor = 0,
        PatchColor1 = 1,
        PatchColor2 = 2,
        PatchColor3 = 3,
        RED = NullColor,
        BLUE = PatchColor1,
    };

    class halfedge_iterator {
      private:
        Halfedge* current_;
        Halfedge* start_;
        bool is_first_iteration_;

      public:
        // Constructor
        halfedge_iterator(Halfedge* start = nullptr) : current_(start), start_(start), is_first_iteration_(true) {}

        // Dereference operator
        Halfedge* operator*() const {
            return current_;
        }

        // Pre-increment operator
        halfedge_iterator& operator++() {
            if (current_ != nullptr) {
                if (is_first_iteration_) {
                    is_first_iteration_ = false;
                } else {
                    // Stop if we've completed the loop and returned to start
                    if (current_->next() == start_) {
                        current_ = nullptr;
                        return *this;
                    }
                }
                current_ = current_->next();
            }
            return *this;
        }

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

    halfedge_iterator begin() const {
        return halfedge_iterator(proxyHalfedge_);
    }

    // Iterator end method - returns iterator representing past-the-end
    halfedge_iterator end() const {
        return halfedge_iterator(nullptr);
    }

    // Iterator begin method - returns iterator pointing to the first halfedge
    halfedge_iterator halfedges_begin() const {
        return halfedge_iterator(proxyHalfedge_);
    }

    // Iterator end method - returns iterator representing past-the-end
    halfedge_iterator halfedges_end() const {
        return halfedge_iterator(nullptr);
    }

    ~Facet();

    Facet(){};
    Facet(const Facet& face);
    Facet(Halfedge* hit);
    Facet(const std::vector<Vertex*>& vs);

    // void setMeshId(int meshId) {
    //     this->meshId_ = meshId;
    // }

    void setGroupId(int gid) {
        this->groupId_ = gid;
    }

    Facet* clone();
    void reset(Halfedge* h);
    void reset(std::vector<Halfedge*>& hs);
    void reset(const std::vector<Vertex*>& vs);
    void reset(const std::vector<Vertex*>& vs, int meshId);
    void reset_without_init(Halfedge* h);

    // PCLTTM/PCATM Phase 2 专用：让 proxyHalfedge 指向 vs[0]→vs[1]
    // 而不是默认的 vs[N-1]→vs[0]。详见实现的注释。
    void reset_proxy_first(const std::vector<Vertex*>& vs);
    void reset_proxy_first(const std::vector<Vertex*>& vs, int meshId);

    // PCLTTM/PCATM Phase 2 专用：把 proxyHalfedge 重置为 face 中
    // **vertex 量化坐标字典序最小**的那个 vertex 出发的 outgoing halfedge。
    //
    // 这是 encoder/decoder 双端共用的"标准化 proxyHalfedge"规则。
    //
    // ⚠️ 浮点版（无参）已废弃：encoder/decoder 两端的 vertex.point() 浮点数
    //    并非 bit-exact 一致（encoder 直接读 OFF 文件 float；decoder 走
    //    intPosToFloat 反量化），float 字典序在边界值上会发散，导致两端
    //    选出的 v0 不一致。请使用接收 (bboxMin, f_quantStep) 的版本，
    //    用整数量化坐标做字典序，保证 bit-exact 一致。
    //
    // 9 个旧算法不调本方法，零影响。
    void normalize_proxy_to_min_position_vertex();
    void normalize_proxy_to_min_position_vertex(const Point& bboxMin, float f_quantStep);

    void remove(Halfedge* h);
    MCGAL::Vector3 computeNormal() const;

    // 返回缓存的法向量
    inline MCGAL::Vector3& normal() {
        return normal_;
    }

    // 重新计算并更新缓存的法向量。
    // perf（2026-11-08 bunny 实测 computeNormal 2.15% cycles）：PPMC 编码/解码
    // 路径完全不读面法线（它用 faceSurface / 自己的 Newell 法线）。PPMC 的
    // compressOptionsTuner / makeDecompressOptions 会设置 PMSF_SKIP_FACE_NORMALS，
    // 该进程内直接跳过 sqrt/div；其它算法不设该 env，行为不变。
    inline void updateNormal() {
        static const bool skipFaceNormals = (std::getenv("PMSF_SKIP_FACE_NORMALS") != nullptr);
        if (skipFaceNormals) return;
        normal_ = computeNormal();
    }

    inline void setProxyHalfedge(Halfedge* h) {
        proxyHalfedge_ = h;
    }

    // override
    bool equal(const Facet& rhs) const;
    bool operator==(const Facet& rhs) const;

    // to_string method
    void print();
    void print_off();

    void dumpTo(std::string path);

    inline void resetBfsFlag() {
        processedFlag_ = NotProcessed;
    }

    inline void resetState() {
        processedFlag_ = NotProcessed;
        splittableFlag_ = Unknown;
        paperLevel_ = -1;
    }

    inline void resetSplittableFlag() {
        splittableFlag_ = Unknown;
    }

    inline void resetProcessedFlag() {
        processedFlag_ = NotProcessed;
    }

    inline void setProcessedFlag(ProcessedFlag flag) {
        processedFlag_ = flag;
    }

    inline ProcessedFlag processedFlag() {
        return processedFlag_;
    }

    inline void setProcessedFlag() {
        processedFlag_ = Processed;
    }

    inline void setUnProcessed() {
        processedFlag_ = NotProcessed;
    }

    inline bool isProcessed() const {
        return processedFlag_ == Processed;
    }

    inline void setRemoved() {
        removedFlag_ = Removed;
    }

    inline void setNotRemoved() {
        removedFlag_ = NotRemoved;
    }

    inline bool isRemoved() {
        return removedFlag_ == Removed;
    }

    inline int poolId() const {
        return poolId_;
    }

    inline void setPoolId(int id) {
        poolId_ = id;
    }
    inline int groupId() const {
        return groupId_;
    }

    inline void groupId(int id) {
        groupId_ = id;
    }

    // inline int meshId() const {
    //     return meshId_;
    // }

    // inline void meshId(int id) {
    //     meshId_ = id;
    // }

    inline Halfedge* proxyHalfedge() const {
        return proxyHalfedge_;
    }

    inline void proxyHalfedge(Halfedge* h) {
        proxyHalfedge_ = h;
    }

    std::vector<Vertex*> getVertices();
    // perf（2026-11-10）：填充到调用方复用缓冲，避免每个面一次 vector 分配。
    void getVertices(std::vector<Vertex*>& out);

    std::vector<Halfedge*> getHalfedges();

    inline bool isConquered() const {
        return (splittableFlag_ == Splittable || splittableFlag_ == Unsplittable || removedFlag_ == Removed);
    }

    inline bool isSplittable() const {
        return (splittableFlag_ == Splittable);
    }

    inline bool isUnsplittable() const {
        return (splittableFlag_ == Unsplittable);
    }

    inline void setSplittable() {
        assert(splittableFlag_ == Unknown);
        splittableFlag_ = Splittable;
    }

    inline void setUnsplittable() {
        assert(splittableFlag_ == Unknown || splittableFlag_ == Unsplittable);
        splittableFlag_ = Unsplittable;
    }

    // Triangle 2-coloring for Z-shape triangulation
    inline void setTriangleColor(TriangleColorFlag color) {
        triangleColor_ = color;
    }

    inline TriangleColorFlag getTriangleColor() const {
        return triangleColor_;
    }

    // PCATM strict paper patch tag. This is intentionally separate from
    // triangleColor_: legacy RED/BLUE stores the Z-shape 2-coloring, while the
    // strict paper path needs a 0..3 patch color used by the symbol collector.
    inline void setPCATMPatchTag(uint8_t tag) {
        pcatmPatchTag_ = static_cast<uint8_t>(tag & 0x3u);
    }

    inline uint8_t pcatmPatchTag() const {
        return pcatmPatchTag_;
    }

    // ----------------------------------------------------------------------
    // Phase 4 PR-A (2026-05-11) —— retriangulation protocol hint。
    // 让 collector / reader 不依赖外部 metadata 即可在 BFS 第一遍判断
    // "这一轮 face 走哪个协议分支"（PCLTTM_TAG / PCATM_ZCOLOR / RD_TAG）。
    //
    // 数值与 pmsf::RetriProtocol（src/common/RetriangulationProtocol.h）+
    // ColumnarSerializeOperator::BlobLayout 三方 1:1 对齐：
    //   0 = UNKNOWN（默认；旧 9 算法走原始路径，不读不写）
    //   16 = PCLTTM_TAG
    //   17 = PCATM_ZCOLOR
    //   18 = RD_TAG
    //
    // 旧 9 算法不调本 setter，零字节级回归。Phase 4 PR-B/PR-C 的
    // *Eliminate / *Collect / *Read 算子按需读写。
    // ----------------------------------------------------------------------
    inline void setProtocolHint(uint8_t p) {
        protocolHint_ = p;
    }

    inline uint8_t getProtocolHint() const {
        return protocolHint_;
    }

    inline void setPaperLevel(int level) {
        paperLevel_ = level;
    }

    inline int paperLevel() const {
        return paperLevel_;
    }

    Point getRemovedVertexPos() const;
    PointInt getRemovedVertexPosInt() const;

    void setRemovedVertexPos(Point p);
    void setRemovedVertexPosInt(PointInt p);

    // Valence-code 码字：eliminate 时记录 polyFace 对应的"被移除顶点度数"。
    // 取值范围典型在 {3, 4, 5, 6}（PCLTTM 仅移除低 valence 顶点）。
    inline int getPolyValence() const {
        return polyValence_;
    }
    inline void setPolyValence(int v) {
        polyValence_ = v;
    }

    // 2026-05-04 性能优化：从 Facet.cpp 移到 header 里内联。
    //   `Halfedge::find_prev` 在三角面上走快捷路径会调 `face->facet_degree() == 3`，
    //   perf 显示 bunny 上 find_prev 占 RD/MMSE/APTM 27% CPU；必须 inline 才有效。
    inline int facet_degree() const { return facet_degree_; }

    double areas();

    inline bool isVisited(int current_version) const {
        return visited_version == current_version;
    }

    inline void setVisited(int current_version) {
        visited_version = current_version;
    }

  private:
    SplittableFlag splittableFlag_ = Unknown;
    ProcessedFlag processedFlag_ = NotProcessed;
    RemovedFlag removedFlag_ = NotRemoved;
    TriangleColorFlag triangleColor_ = RED;
    uint8_t pcatmPatchTag_ = 0;

    int poolId_ = -1;
    int groupId_ = -1;
    // int meshId_ = -1;
    MCGAL::Halfedge* proxyHalfedge_ = nullptr;
    MCGAL::Point removedVertexPos;
    MCGAL::PointInt removedVertexPosInt;
    // Valence-code pipeline 使用。由 VertexRemovalEliminateOperator::remove_point 在
    // `erase_center_vertex` 之前写入（= 被移除顶点的 vertex_degree()），
    // 序列化阶段由 ValenceSymbolCollectOperator 读取用于生成连通性码字。
    // 仅在 isSplittable() == true 的 polyFace 上有意义；每轮 resetState 后保持旧值不影响语义。
    int polyValence_ = 0;
    // std::vector<Vertex*> vertices_;
    // std::vector<Halfedge*> halfedges_;
    int facet_degree_ = 3;
    int visited_version = 0;
    MCGAL::Vector3 normal_;
    // Phase 4 PR-A (2026-05-11): retriangulation protocol hint。默认 0=UNKNOWN，
    // 旧 9 算法不读不写 → 字节级零回归。仅 Phase 4 新增的 *Eliminate/*Collect/
    // *Read 算子读写。详见 docs/paper_alignment/PHASE4_PLAN.md §3.2。
    uint8_t protocolHint_ = 0;
    int paperLevel_ = -1;
};

}  // namespace MCGAL

// 2026-05-04 性能优化：Halfedge::find_prev 的三角面快捷路径 inline 定义。
//   此前作为非 inline 函数导致 perf 显示 find_prev 占 RD/MMSE/APTM 17% CPU；
//   inline 后三角面走 2 次 load + ret，n-gon 走原循环。放在 Facet.h 末尾是
//   为了让 Facet::facet_degree() 的定义对 find_prev 可见 —— Halfedge.h 仅前向
//   声明 Facet，所以无法在 Halfedge.h 的类体里直接 inline。
namespace MCGAL {
inline Halfedge* Halfedge::find_prev() {
    if (face_ && face_->facet_degree() == 3) {
        return this->next()->next();
    }
    Halfedge* g = this;
    while (g->next() != this) {
        g = g->next();
    }
    return g;
}
}  // namespace MCGAL
#endif
