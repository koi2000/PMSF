#ifndef CONTEXTPOOL_H
#define CONTEXTPOOL_H
#include "Configuration.h"
#include "Facet.h"
#include "Halfedge.h"
#include "PerformedOperation.h"
#include "Vertex.h"
#include "openmesh/Property.hh"
#include "openmesh/PropertyContainer.hh"
#include <atomic>
#include <cassert>
#include <deque>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace MCGAL {

const std::string ENABLE_QUANTIZATION = "enable_quantization";
const std::string ENABLE_PREDICTION = "enable_prediction";
const std::string ENABLE_COMPRESS = "enable_compress";
const std::string ENABLE_SEGMENT = "enable_segment";

class ContextValue {
  public:
    enum class Type { None, Int, Float, Double, Long, String };

    ContextValue();
    ContextValue(int v);
    ContextValue(float v);
    ContextValue(double v);
    ContextValue(long v);
    ContextValue(const std::string& v);
    ContextValue(const char* v);

    ContextValue(const ContextValue& other);
    ContextValue(ContextValue&& other) noexcept;

    ContextValue& operator=(const ContextValue& other);
    ContextValue& operator=(ContextValue&& other) noexcept;

    ~ContextValue();

    Type type() const {
        return type_;
    }

    int asInt(int defaultValue = 0) const;
    float asFloat(float defaultValue = 0.0f) const;
    double asDouble(double defaultValue = 0.0) const;
    long asLong(long defaultValue = 0L) const;
    const std::string& asString() const;

    bool isNumeric() const;

  private:
    union Storage {
        int intValue;
        float floatValue;
        double doubleValue;
        long longValue;
        std::string stringValue;

        Storage() {}
        ~Storage() {}
    };

    void destroy();
    void copyFrom(const ContextValue& other);
    void moveFrom(ContextValue&& other);

    Storage data_;
    Type type_ = Type::None;
};

class ContextPool {
  private:
    int poolIndex = 0;
    std::unordered_map<std::string, ContextValue> metadata_;
    mutable std::mutex metadataMutex_;
    // "level" key 的无锁热缓存（见 levelFast 注释）。setMetadata("level", ...) 同步写。
    std::atomic<int> cachedLevel_{0};

  public:
    std::vector<std::multimap<int, PerformedOperation>> operations;
    std::vector<int> vindexs;
    std::vector<int> hindexs;
    std::vector<int> findexs;

    std::vector<int> vpoolSizes;
    std::vector<int> hpoolSizes;
    std::vector<int> fpoolSizes;

    std::vector<MCGAL::Vertex*> vpools;
    std::vector<MCGAL::Halfedge*> hpools;
    std::vector<MCGAL::Facet*> fpools;
    // perf（2026-11-08 perf record：池注册占 PPMC 压缩 ~21 ms，其中 H=13.1 ms、F=6.4 ms）：
    // H/F 池改为**裸内存 + 按需 placement-new**。未分配的槽位不再逐个默认构造；
    // cursor 以内的槽位全部由 allocate*/preAlloc* 原地构造，语义与
    // “注册时全部构造 + 分配时 setPoolId/reset”完全一致。
    std::vector<char> hraw_;
    std::vector<char> fraw_;

    std::vector<OpenMesh::PropertyContainer> vprops_;
    std::vector<OpenMesh::PropertyContainer> hprops_;
    std::vector<OpenMesh::PropertyContainer> eprops_;
    std::vector<OpenMesh::PropertyContainer> fprops_;
    std::vector<OpenMesh::PropertyContainer> mprops_;

    ContextPool();

  public:
    ~ContextPool();

    static ContextPool& getInstance() {
        static ContextPool contextPool;
        return contextPool;
    }

    void initPoolSize(int size);

    int registerPool(int estimateVSize, int estimateHSize, int estimateFSize);
    bool hasPool(int index) const;
    static int poolSizeFromEnv(const char* name, int fallback);
    void releasePool(int index);

    ContextPool(const ContextPool&) = delete;
    ContextPool& operator=(const ContextPool&) = delete;

    void reset() {}

    void halfedgeResetState() {
        for (int i = 0; i < hpools.size(); i++) {
            for (int j = 0; j < hpoolSizes[i]; j++) {
                if (hpools[i][j].isRemoved()) {
                    continue;
                }
                hpools[i][j].resetState();
            }
        }
    }

    void increaseLod() {
        operations.emplace_back();
    }

    void pushOperation(int poolId, PerformedOperation& op) {
        // operations.back()[poolId] = PerformedOperation(v1, v2, v3, v4);
        operations.back().emplace(poolId, op);
    }

    template <class T> void add_property(int index, OpenMesh::VPropHandleT<T>& _ph, const std::string& _name = "<vprop>") {
        _ph = OpenMesh::VPropHandleT<T>(vprops_[index].add(T(), _name));
        vprops_[index].resize(vpoolSizes[index]);
    }

    template <class T> void add_property(int index, OpenMesh::HPropHandleT<T>& _ph, const std::string& _name = "<eprop>") {
        _ph = OpenMesh::HPropHandleT<T>(hprops_[index].add(T(), _name));
        hprops_[index].resize(hpoolSizes[index]);
    }

    template <class T> void add_property(int index, OpenMesh::FPropHandleT<T>& _ph, const std::string& _name = "<fprop>") {
        _ph = OpenMesh::FPropHandleT<T>(fprops_[index].add(T(), _name));
        fprops_[index].resize(fpoolSizes[index]);
    }

    template <class T> typename OpenMesh::VPropHandleT<T>::reference property(int index, OpenMesh::VPropHandleT<T> _ph, MCGAL::Vertex* _vh) {
        return vprops_[index].property(_ph)[_vh->poolId()];
    }

    template <class T>
    typename OpenMesh::VPropHandleT<T>::const_reference property(int index, OpenMesh::VPropHandleT<T> _ph, MCGAL::Vertex* _vh) const {
        return vprops_[index].property(_ph)[_vh->poolId()];
    }

    template <class T> typename OpenMesh::HPropHandleT<T>::reference property(int index, OpenMesh::HPropHandleT<T> _ph, MCGAL::Halfedge* _hh) {
        return hprops_[index].property(_ph)[_hh->poolId()];
    }

    template <class T> typename OpenMesh::VPropHandleT<T>::reference property(int index, const std::string& _name, MCGAL::Vertex* _vh) {
        auto handle = vprops_[index].handle(T(), _name);
        return vprops_[index].property(handle)[_vh->poolId()];
    }

    template <class T>
    typename OpenMesh::VPropHandleT<T>::const_reference property(int index, const std::string& _name, MCGAL::Vertex* _vh) const {
        auto handle = vprops_[index].handle(T(), _name);
        return vprops_[index].property(handle)[_vh->poolId()];
    }

    template <class T> typename OpenMesh::HPropHandleT<T>::reference property(int index, const std::string& _name, MCGAL::Halfedge* _hh) {
        auto handle = hprops_[index].handle(T(), _name);
        return hprops_[index].property(handle)[_hh->poolId()];
    }

    template <class T>
    typename OpenMesh::HPropHandleT<T>::const_reference property(int index, const std::string& _name, MCGAL::Halfedge* _hh) const {
        auto handle = hprops_[index].handle(T(), _name);
        return hprops_[index].property(handle)[_hh->poolId()];
    }

    template <class T> typename OpenMesh::FPropHandleT<T>::reference property(int index, const std::string& _name, MCGAL::Facet* _fh) {
        auto handle = fprops_[index].handle(T(), _name);
        return fprops_[index].property(handle)[_fh->poolId()];
    }

    template <class T>
    typename OpenMesh::FPropHandleT<T>::const_reference property(int index, const std::string& _name, MCGAL::Facet* _fh) const {
        auto handle = fprops_[index].handle(T(), _name);
        return fprops_[index].property(handle)[_fh->poolId()];
    }

    template <class T>
    typename OpenMesh::HPropHandleT<T>::const_reference property(int index, OpenMesh::HPropHandleT<T> _ph, MCGAL::Halfedge* _hh) const {
        return hprops_[index].property(_ph)[_hh->poolId()];
    }

    template <class T> typename OpenMesh::FPropHandleT<T>::reference property(int index, OpenMesh::FPropHandleT<T> _ph, MCGAL::Facet* _fh) {
        return fprops_[index].property(_ph)[_fh->poolId()];
    }

    template <class T>
    typename OpenMesh::FPropHandleT<T>::const_reference property(int index, OpenMesh::FPropHandleT<T> _ph, MCGAL::Facet* _fh) const {
        return fprops_[index].property(_ph)[_fh->poolId()];
    }

    template <class T> typename OpenMesh::VPropHandleT<T>::reference property(OpenMesh::VPropHandleT<T> _ph, MCGAL::Vertex* _vh) {
        return vprops_[DEFAULT_MESH_ID].property(_ph)[_vh->poolId()];
    }

    template <class T> typename OpenMesh::VPropHandleT<T>::const_reference property(OpenMesh::VPropHandleT<T> _ph, MCGAL::Vertex* _vh) const {
        return vprops_[DEFAULT_MESH_ID].property(_ph)[_vh->poolId()];
    }

    template <class T> typename OpenMesh::HPropHandleT<T>::reference property(OpenMesh::HPropHandleT<T> _ph, MCGAL::Halfedge* _hh) {
        return hprops_[DEFAULT_MESH_ID].property(_ph)[_hh->poolId()];
    }

    template <class T> typename OpenMesh::HPropHandleT<T>::const_reference property(OpenMesh::HPropHandleT<T> _ph, MCGAL::Halfedge* _hh) const {
        return hprops_[DEFAULT_MESH_ID].property(_ph)[_hh->poolId()];
    }

    template <class T> typename OpenMesh::FPropHandleT<T>::reference property(OpenMesh::FPropHandleT<T> _ph, MCGAL::Facet* _fh) {
        return fprops_[DEFAULT_MESH_ID].property(_ph)[_fh->poolId()];
    }

    template <class T> typename OpenMesh::FPropHandleT<T>::const_reference property(OpenMesh::FPropHandleT<T> _ph, MCGAL::Facet* _fh) const {
        return fprops_[DEFAULT_MESH_ID].property(_ph)[_fh->poolId()];
    }

    // Property access by name (using DEFAULT_MESH_ID)
    template <class T> typename OpenMesh::VPropHandleT<T>::reference property(const std::string& _name, MCGAL::Vertex* _vh) {
        auto handle = vprops_[DEFAULT_MESH_ID].handle(T(), _name);
        return vprops_[DEFAULT_MESH_ID].property(handle)[_vh->poolId()];
    }

    template <class T> typename OpenMesh::VPropHandleT<T>::const_reference property(const std::string& _name, MCGAL::Vertex* _vh) const {
        auto handle = vprops_[DEFAULT_MESH_ID].handle(T(), _name);
        return vprops_[DEFAULT_MESH_ID].property(handle)[_vh->poolId()];
    }

    template <class T> typename OpenMesh::HPropHandleT<T>::reference property(const std::string& _name, MCGAL::Halfedge* _hh) {
        auto handle = hprops_[DEFAULT_MESH_ID].handle(T(), _name);
        return hprops_[DEFAULT_MESH_ID].property(handle)[_hh->poolId()];
    }

    template <class T> typename OpenMesh::HPropHandleT<T>::const_reference property(const std::string& _name, MCGAL::Halfedge* _hh) const {
        auto handle = hprops_[DEFAULT_MESH_ID].handle(T(), _name);
        return hprops_[DEFAULT_MESH_ID].property(handle)[_hh->poolId()];
    }

    template <class T> typename OpenMesh::FPropHandleT<T>::reference property(const std::string& _name, MCGAL::Facet* _fh) {
        auto handle = fprops_[DEFAULT_MESH_ID].handle(T(), _name);
        return fprops_[DEFAULT_MESH_ID].property(handle)[_fh->poolId()];
    }

    template <class T> typename OpenMesh::FPropHandleT<T>::const_reference property(const std::string& _name, MCGAL::Facet* _fh) const {
        auto handle = fprops_[DEFAULT_MESH_ID].handle(T(), _name);
        return fprops_[DEFAULT_MESH_ID].property(handle)[_fh->poolId()];
    }

    // ============================================================================
    // PCLTTM Phase 3 路径 A 专用：幂等 attach 顶点属性
    // ----------------------------------------------------------------------------
    //   Phase 3 的二阶 curvature predictor 要在 PCLTTM Tag 的 encoder/decoder
    //   两端共享 vertex 上的 r1 属性（kVPropRDR1）。`property<T>(meshId, name, vh)`
    //   底层依赖属性已经被 add_property 注册过，否则 handle().idx == -1 会触发
    //   PropertyContainer::property(_h) 的 assert（debug）/ UB（release）。
    //   旧 9 算法不调本函数，零影响。idempotent：name 已存在 → no-op。
    // ============================================================================
    template <class T>
    void attachVertexProperty(int meshId, const std::string& name) {
        if (meshId < 0 || meshId >= static_cast<int>(vprops_.size())) return;
        // 如果已存在同名同类型属性 → no-op
        OpenMesh::BasePropHandleT<T> existing = vprops_[meshId].handle(T(), name);
        if (existing.is_valid()) {
            // 容量同步：vpoolSizes 在 attach 之后可能变化，确保 vector 已 resize
            vprops_[meshId].resize(vpoolSizes[meshId]);
            return;
        }
        OpenMesh::VPropHandleT<T> ph;
        add_property(meshId, ph, name);
    }

    inline int getFindex(int index) {
        return findexs[index];
    }

    inline int getHindex(int index) {
        return hindexs[index];
    }

    inline int getVindex(int index) {
        return vindexs[index];
    }

    inline MCGAL::Vertex* getVertexPoolByIndex(int groupId) {
        return vpools[groupId];
    }

    inline MCGAL::Halfedge* getHalfedgePoolByIndex(int groupId) {
        return hpools[groupId];
    }

    inline MCGAL::Facet* getFacetPoolByIndex(int groupId) {
        return fpools[groupId];
    }

    inline MCGAL::Vertex* getVertexByIndexInSubPool(int groupId, int offset) {
        return &vpools[groupId][offset];
    }

    inline MCGAL::Halfedge* getHalfedgeByIndexInSubPool(int groupId, int offset) {
        return &hpools[groupId][offset];
    }

    inline MCGAL::Facet* getFacetByIndexInSubPool(int groupId, int offset) {
        return &fpools[groupId][offset];
    }

    inline MCGAL::Vertex* getVertexByIndex(int index, int offset) {
        return &vpools[index][offset];
    }

    inline MCGAL::Halfedge* getHalfedgeByIndex(int index, int offset) {
        return &hpools[index][offset];
    }

    inline MCGAL::Facet* getFacetByIndex(int index, int offset) {
        return &fpools[index][offset];
    }

    inline MCGAL::Vertex* allocateVertexFromPool(int index) {
        ensureCapacity(index, vindexs[index], vpoolSizes, "vertex");
        const int pid = vindexs[index]++;
        MCGAL::Vertex* v = &vpools[index][pid];
        v->setPoolId(pid);
        return v;
    }

    inline MCGAL::Vertex* allocateVertexFromPool(int index, float x, float y, float z) {
        ensureCapacity(index, vindexs[index], vpoolSizes, "vertex");
        const int pid = vindexs[index]++;
        MCGAL::Vertex* v = &vpools[index][pid];
        v->setPoolId(pid);
        v->setPoint(x, y, z);
        return v;
    }

    inline MCGAL::Vertex* allocateVertexFromPool(int index, MCGAL::Point p) {
        ensureCapacity(index, vindexs[index], vpoolSizes, "vertex");
        const int pid = vindexs[index]++;
        MCGAL::Vertex* v = &vpools[index][pid];
        v->setPoolId(pid);
        v->setPoint(p);
        return v;
    }

    /// @brief 取得 [index][pid] 的已构造 Halfedge 槽（裸池则先 placement-new）。
    inline MCGAL::Halfedge* halfedgeSlot(int index, int pid) {
        MCGAL::Halfedge* h = &hpools[index][pid];
        if (index < static_cast<int>(hraw_.size()) && hraw_[index]) {
            ::new (static_cast<void*>(h)) MCGAL::Halfedge();
        }
        return h;
    }
    inline MCGAL::Facet* facetSlot(int index, int pid) {
        MCGAL::Facet* f = &fpools[index][pid];
        if (index < static_cast<int>(fraw_.size()) && fraw_[index]) {
            ::new (static_cast<void*>(f)) MCGAL::Facet();
        }
        return f;
    }

    inline MCGAL::Halfedge* allocateHalfedgeFromPool(int index) {
        ensureCapacity(index, hindexs[index], hpoolSizes, "halfedge");
        int pid = hindexs[index]++;
        MCGAL::Halfedge* h = halfedgeSlot(index, pid);
        h->setPoolId(pid);
        return h;
    }

    inline MCGAL::Halfedge* allocateHalfedgeFromPoolWithOutInit(int index, MCGAL::Vertex* v1, MCGAL::Vertex* v2) {
        ensureCapacity(index, hindexs[index], hpoolSizes, "halfedge");
        int pid = hindexs[index]++;
        MCGAL::Halfedge* h = halfedgeSlot(index, pid);
        h->setPoolId(pid);
        h->reset_without_init(v1, v2);
        h->setGroupId(index);
        return h;
    }

    inline MCGAL::Halfedge* allocateHalfedgeFromPool(int index, MCGAL::Vertex* v1, MCGAL::Vertex* v2) {
        ensureCapacity(index, hindexs[index], hpoolSizes, "halfedge");
        int pid = hindexs[index]++;
        MCGAL::Halfedge* h = halfedgeSlot(index, pid);
        h->setPoolId(pid);
        h->reset(v1, v2);
        h->setGroupId(index);
        return h;
    }

    inline MCGAL::Facet* allocateFaceFromPool(int index) {
        ensureCapacity(index, findexs[index], fpoolSizes, "facet");
        const int pid = findexs[index]++;
        MCGAL::Facet* f = facetSlot(index, pid);
        f->setPoolId(pid);
        return f;
    }

    inline MCGAL::Facet* allocateFaceFromPool(int index, const std::vector<MCGAL::Vertex*>& vts) {
        ensureCapacity(index, findexs[index], fpoolSizes, "facet");
        const int pid = findexs[index]++;
        MCGAL::Facet* f = facetSlot(index, pid);
        f->setPoolId(pid);
        f->setGroupId(index);
        f->reset(vts);
        return f;
    }

    // PCLTTM/PCATM Phase 2 专用：reset_proxy_first 版本，让 face 的
    // proxyHalfedge 指向 vs[0]→vs[1]，与 encoder Columnar 序列化的
    // faceVids[0] = encoder.proxyHalfedge.vertex 语义对齐。
    // 旧 9 算法仍调 allocateFaceFromPool 使用 reset()，零影响。
    inline MCGAL::Facet* allocateFaceFromPoolProxyFirst(int index, const std::vector<MCGAL::Vertex*>& vts) {
        ensureCapacity(index, findexs[index], fpoolSizes, "facet");
        const int pid = findexs[index]++;
        MCGAL::Facet* f = facetSlot(index, pid);
        f->setPoolId(pid);
        f->setGroupId(index);
        f->reset_proxy_first(vts);
        return f;
    }

    inline MCGAL::Facet* allocateFaceFromPool(int index, MCGAL::Halfedge* halfedge) {
        ensureCapacity(index, findexs[index], fpoolSizes, "facet");
        const int pid = findexs[index]++;
        MCGAL::Facet* f = facetSlot(index, pid);
        f->setPoolId(pid);
        f->setGroupId(index);
        f->reset_without_init(halfedge);
        return f;
    }

    inline int preAllocVertex(int index, int size) {
        ensureCapacity(index, vindexs[index] + size - 1, vpoolSizes, "vertex");
        int ret = vindexs[index];
        vindexs[index] += size;
        return ret;
    }

    inline int preAllocHalfedge(int index, int size) {
        ensureCapacity(index, hindexs[index] + size - 1, hpoolSizes, "halfedge");
        int ret = hindexs[index];
        if (index < static_cast<int>(hraw_.size()) && hraw_[index]) {
            for (int i = 0; i < size; ++i)
                ::new (static_cast<void*>(&hpools[index][ret + i])) MCGAL::Halfedge();
        }
        hindexs[index] += size;
        return ret;
    }

    inline int preAllocFace(int index, int size) {
        ensureCapacity(index, findexs[index] + size - 1, fpoolSizes, "facet");
        int ret = findexs[index];
        if (index < static_cast<int>(fraw_.size()) && fraw_[index]) {
            for (int i = 0; i < size; ++i)
                ::new (static_cast<void*>(&fpools[index][ret + i])) MCGAL::Facet();
        }
        findexs[index] += size;
        return ret;
    }

    inline void ensureCapacity(int index, int next, const std::vector<int>& sizes, const char* kind) const {
        if (index < 0 || index >= static_cast<int>(sizes.size()) || next < 0 || next >= sizes[index]) {
            const int size = (index >= 0 && index < static_cast<int>(sizes.size())) ? sizes[index] : -1;
            throw std::runtime_error(std::string("ContextPool ") + kind +
                                     " pool exhausted: pool=" + std::to_string(index) +
                                     " next=" + std::to_string(next) +
                                     " size=" + std::to_string(size));
        }
    }

    void setMetadata(const std::string& key, ContextValue value);
    bool hasMetadata(const std::string& key) const;
    ContextValue getMetadataOrDefault(const std::string& key, const ContextValue& defaultValue = ContextValue()) const;
    bool tryGetMetadata(const std::string& key, ContextValue& outValue) const;
    void removeMetadata(const std::string& key);
    void clearMetadata();

    // ⚠️ 2026-05-04 性能优化：热路径 "level" 缓存。
    //   `Level2DegreeN*::isRemovable` 每条候选 halfedge 调用
    //   `getMetadataOrDefault("level", ...)` 两次 —— bunny × 30 rounds × ~70k halfedges
    //   ≈ 400 万次 mutex acquire + hashmap.find + string hash + ContextValue 拷贝。
    //   在 bench 里 MMSE/APTM/PCLTTM/RD/VQMMC 单次压缩 5–9 秒，其中 30%–50% 落在这里。
    //   提供一个 `int levelFast()` 直接读 atomic 的快速路径；`setMetadata("level", ...)`
    //   内部会同步更新此原子（见 ContextPool.cpp），语义等价，但开销 O(1) 无锁。
    inline int levelFast() const noexcept { return cachedLevel_.load(std::memory_order_relaxed); }
};

}  // namespace MCGAL
#endif
