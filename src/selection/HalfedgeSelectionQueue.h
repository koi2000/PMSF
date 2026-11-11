// A standalone selection module for choosing the next halfedge to collapse.
// It maintains an internal min-priority queue ordered by user-provided error metrics.
//
// Usage:
//   - Provide either a halfedge error accessor or a vertex error accessor (or both).
//   - When using vertex-based errors, the halfedge error is computed by combining
//     the two endpoint vertex errors via the provided combiner (default: min).
//   - Push or build from a mesh, then call popNext() to get the next best halfedge.
//
// Extensibility:
//   - Use the entryCustomizer callback to set additional fields in HalfedgeErrorEntry
//   - Example: Set custom data based on halfedge properties or topology
//
// ⚠️ 2026-05-04 性能优化：
//   `currentVersions_` 由 `std::unordered_map<Halfedge*, uint64_t>` 改为按 `poolId()`
//   稠密索引的 `std::vector<uint64_t>`（0 = 已失效 / 没见过）。
//   - 6 个算法上 select 循环累计耗时中，hashing + `unordered_map::find/erase/insert`
//     是最大的共同开销之一（`pushOrUpdate` 每条半边一次，`popNextEntry` 每次 pop
//     一次 find + 一次 erase）。
//   - 稠密 vector 访问是 O(1) 无 hash；clear 只要 memset 零。
//   - 语义 100% 等价：所有调用点都用 `version==0` 表示"未在队"。BPV 无变化。
//   `visitedVertices_` 同样从 `std::set<int>` 改为 `std::vector<uint8_t>` 按 vertex
//   poolId 标记 visited，消除 rb-tree 平衡维护。

#pragma once

#include <algorithm>
#include <any>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <unordered_map>  // 保留以防外部引用；实现已改为稠密 vector。
#include <vector>

#include "../common/HalfedgeErrorEntry.h"
#include "../operator/IsRemovableOperator.h"
#include "ErrorSource.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Vertex.h"

struct VertexErrorEntry {
    MCGAL::Vertex* vertex;
    float error;
    std::uint64_t version;
};

struct MinErrorComparator {
    bool operator()(const HalfedgeErrorEntry& a, const HalfedgeErrorEntry& b) const {
        // priority_queue puts the "largest" first; we invert to get min-heap behavior
        return a.error > b.error;
    }
};

// Forward declaration of entry customizer function signature
// Users can provide a callback to customize HalfedgeErrorEntry before it's added to the queue
using EntryCustomizer = std::function<bool(HalfedgeErrorEntry&, MCGAL::Halfedge*)>;

class HalfedgeSelectionQueue {
  public:
    HalfedgeSelectionQueue() = default;
    HalfedgeSelectionQueue(ErrorSource source,
                           std::function<float(MCGAL::Halfedge*, MCGAL::Point& p)> halfedgeErrorAccessor = nullptr,
                           std::function<float(MCGAL::Vertex*)> vertexErrorAccessor = nullptr,
                           std::vector<std::shared_ptr<IsRemovableOperator>> operators = {},
                           EntryCustomizer entryCustomizer = nullptr,
                           pmsf::SelectionPolicy policy = {})
        : source_(source), halfedgeErrorAccessor_(std::move(halfedgeErrorAccessor)), vertexErrorAccessor_(std::move(vertexErrorAccessor)),
          operators_(operators), entryCustomizer_(entryCustomizer), policy_(std::move(policy)) {
        if (!isValidEdgePredicate_) {
            isValidEdgePredicate_ = [](MCGAL::Halfedge* e) { return e != nullptr && !e->isRemoved(); };
        }
    }

    void clear() {
        // Only reset slots touched by this queue instance. The dense arrays can
        // grow to pool size, so full-vector clearing dominates multi-round runs.
        for (int pid : touchedVersionSlots_) {
            if (pid >= 0 && static_cast<size_t>(pid) < currentVersions_.size()) {
                currentVersions_[pid] = 0;
            }
        }
        touchedVersionSlots_.clear();
        for (int vid : touchedVisitedVertices_) {
            if (vid >= 0 && static_cast<size_t>(vid) < visitedVertices_.size()) {
                visitedVertices_[vid] = 0;
            }
        }
        touchedVisitedVertices_.clear();
        versionsSize_ = 0;  // 逻辑 "empty" 计数器
        // Discard the stale heap in one container teardown.  Popping every
        // element performs O(n log n) heap maintenance even though the queue
        // is being cleared wholesale; replacing it is O(n) destruction and
        // preserves the same empty-queue semantics.
        heap_ = decltype(heap_)(MinErrorComparator{});
        globalVersionCounter_ = 0;
    }

    bool empty() const {
        return versionsSize_ == 0;
    }

    std::size_t size() const {
        return versionsSize_;
    }

    void reserve(std::size_t n) {
        if (n > currentVersions_.size()) {
            currentVersions_.resize(n, 0);
        }
    }

    void addIsRemovableOperator(std::shared_ptr<IsRemovableOperator> op) {
        operators_.push_back(op);
    }

    // Set or update the entry customizer callback
    void setEntryCustomizer(EntryCustomizer customizer) {
        entryCustomizer_ = customizer;
    }

    bool isRemovable(HalfedgeErrorEntry& entry) {
        for (auto& op : operators_) {
            if (!op->isRemovable(entry)) {
                return false;
            }
        }
        return true;
    }

    bool isValid(HalfedgeErrorEntry& entry) {
        if(source_ == ErrorSource::Vertex) {
            int vid = entry.edge->end_vertex()->poolId();
            if (vid >= 0 && vid < (int)visitedVertices_.size() && visitedVertices_[vid]) {
                return false;
            }
        }
        return isValidEdgePredicate_(entry.edge) && isRemovable(entry);
    }

    // Build queue from all halfedges incident to vertices of the mesh.
    // This avoids coupling with any specific mesh traversal elsewhere in the project.
    void buildFromMesh(std::shared_ptr<MCGAL::Mesh>& mesh, bool singleGroupOnly = false, int groupId = 0) {
        clear();
        auto& faces = mesh->faces();
        if (source_ == ErrorSource::Halfedge) {
            for (MCGAL::Facet* f : faces) {
                if(singleGroupOnly && f->groupId() != groupId)
                    continue;
                if (f == nullptr || f->isRemoved())
                    continue;
                // auto& halfedges = v->halfedges();
                for (MCGAL::Halfedge* h : *f) {
                    if (h == nullptr || h->isRemoved())
                        continue;
                    // Only index each undirected edge once (by convention: original flag or lower poolId)
                    // if (!isValid(h))
                    //     continue;
                    // if (h->opposite() && h->poolId() > h->opposite()->poolId())
                    //     continue;
                    pushOrUpdate(h);
                }
            }
        } else if (source_ == ErrorSource::Vertex) {
            // 可以将所有的都加进去，但是内嵌一个set来记录哪些被处理过了
            auto& vertices = mesh->vertices();
            for (MCGAL::Facet* f : faces) {
                if(singleGroupOnly && f->groupId() != groupId)
                    continue;
                if (f == nullptr || f->isRemoved())
                    continue;
                for (MCGAL::Halfedge* h : *f) {
                    if (h == nullptr || h->isRemoved()) {
                        continue;
                    }
                    pushOrUpdate(h);
                }
            }
        } else {
            throw std::invalid_argument("Invalid error source");
        }
    }

    void buildFromFaces(const std::vector<MCGAL::Facet*>& faces) {
        clear();
        if (source_ != ErrorSource::Halfedge && source_ != ErrorSource::Vertex) {
            throw std::invalid_argument("Invalid error source");
        }
        for (MCGAL::Facet* f : faces) {
            if (f == nullptr || f->isRemoved())
                continue;
            for (MCGAL::Halfedge* h : *f) {
                if (h == nullptr || h->isRemoved())
                    continue;
                pushOrUpdate(h);
            }
        }
    }

    // Insert or update the error for a given halfedge.
    void pushOrUpdate(MCGAL::Halfedge* edge) {
        if (edge == nullptr)
            return;
        // if (!isValid(edge))
        //     return;
        MCGAL::Point p;
        float err = computeError(edge, p);
        std::uint64_t version = ++globalVersionCounter_;
        ensureVersionCapacity(edge->poolId());
        uint64_t& slot = currentVersions_[edge->poolId()];
        if (slot == 0) {
            ++versionsSize_;  // 新加入的半边：逻辑计数 +1
            touchedVersionSlots_.push_back(edge->poolId());
        }
        slot = version;

        // Create the entry and apply customizer if provided
        HalfedgeErrorEntry entry(edge, p, err, version);

        heap_.push(entry);

        // Periodic compaction: rebuild heap when stale entries dominate.
        if (heap_.size() * 100 > compactRatioPercent() * versionsSize_ &&
            versionsSize_ > 1000) {
            compact();
        }
    }

    // Compact: rebuild heap with only live entries.
    void compact() {
        std::vector<HalfedgeErrorEntry> live;
        live.reserve(versionsSize_);
        while (!heap_.empty()) {
            auto e = heap_.top();
            heap_.pop();
            int pid = e.edge ? e.edge->poolId() : -1;
            if (pid >= 0 && pid < (int)currentVersions_.size()
                && currentVersions_[pid] == e.version && isValid(e)) {
                live.push_back(e);
            }
        }
        heap_ = decltype(heap_)(MinErrorComparator{}, std::move(live));
    }

    // Remove an edge from consideration (lazy remove).
    void remove(MCGAL::Halfedge* edge) {
        if (edge == nullptr)
            return;
        int pid = edge->poolId();
        if (pid >= 0 && pid < (int)currentVersions_.size() && currentVersions_[pid] != 0) {
            currentVersions_[pid] = 0;
            if (versionsSize_ > 0) --versionsSize_;
        }
    }

    // Pop and return the complete entry including custom data
    // Use this when you need to access customData field
    HalfedgeErrorEntry popNextEntry() {
        while (!heap_.empty()) {
            HalfedgeErrorEntry top = heap_.top();
            heap_.pop();
            int pid = top.edge ? top.edge->poolId() : -1;
            if (pid < 0 || pid >= (int)currentVersions_.size()) {
                continue;
            }
            uint64_t ver = currentVersions_[pid];
            if (ver == 0) {
                continue;  // removed or superseded
            }
            if (ver != top.version) {
                continue;  // superseded by a newer update
            }
            if (!isValid(top)) {
                currentVersions_[pid] = 0;
                if (versionsSize_ > 0) --versionsSize_;
                continue;
            }
            currentVersions_[pid] = 0;
            if (versionsSize_ > 0) --versionsSize_;
            if(source_ == ErrorSource::Vertex) {
                int vid = top.edge->end_vertex()->poolId();
                if (vid >= 0) {
                    if (vid >= (int)visitedVertices_.size()) {
                        visitedVertices_.resize(static_cast<size_t>(vid) + 1024, 0);
                    }
                    if (!visitedVertices_[vid]) {
                        visitedVertices_[vid] = 1;
                        touchedVisitedVertices_.push_back(vid);
                    }
                }
            }
            if (entryCustomizer_) {
                if(!entryCustomizer_(top, top.edge)) {
                    continue;
                }
            }
            return top;  // Return complete entry with custom data
        }
        return HalfedgeErrorEntry(nullptr, MCGAL::Point(), 0.0f, 0);
    }

    // Recompute error for a specific edge and update its priority.
    void recompute(MCGAL::Halfedge* edge) {
        pushOrUpdate(edge);
    }

    float peekMinError() const {
        // Not strictly exact due to lazy removal; used for diagnostics.
        if (heap_.empty())
            return std::numeric_limits<float>::infinity();
        return heap_.top().error;
    }

  private:
    static std::size_t compactRatioPercent() {
        static const std::size_t ratio = [] {
            const char* value = std::getenv("PMSF_QUEUE_COMPACT_RATIO_PERCENT");
            if (value == nullptr || *value == '\0') return std::size_t{400};
            char* end = nullptr;
            const unsigned long parsed = std::strtoul(value, &end, 10);
            if (end == value || *end != '\0' || parsed < 100 || parsed > 10000) {
                return std::size_t{400};
            }
            return static_cast<std::size_t>(parsed);
        }();
        return ratio;
    }

    void ensureVersionCapacity(int pid) {
        if (pid < 0) return;
        if ((size_t)pid >= currentVersions_.size()) {
            currentVersions_.resize(static_cast<size_t>(pid) + 1024, 0);
        }
    }

    float computeError(MCGAL::Halfedge* edge, MCGAL::Point& p) const {
        if (source_ == ErrorSource::Halfedge) {
            if (halfedgeErrorAccessor_)
                return halfedgeErrorAccessor_(edge, p);
        } else {
            if (vertexErrorAccessor_) {
                if (edge->end_vertex())
                    return vertexErrorAccessor_(edge->end_vertex());
            }
        }
        // 默认使用原来的点作为新的位置
        p = edge->vertex()->point();
        // 算法特有的误差项由 SelectionPolicy 提供；为空即框架默认（返回 0）。
        if (policy_.errorOverride) {
            return policy_.errorOverride(edge, p);
        }
        // 如果不传入，就默认不使用error计算 返回0
        return 0;
    }

  private:
    ErrorSource source_;
    std::function<float(MCGAL::Halfedge*, MCGAL::Point&)> halfedgeErrorAccessor_;
    std::function<float(MCGAL::Vertex*)> vertexErrorAccessor_;
    std::vector<std::shared_ptr<IsRemovableOperator>> operators_;
    std::function<bool(MCGAL::Halfedge*)> isValidEdgePredicate_;
    EntryCustomizer entryCustomizer_;
    pmsf::SelectionPolicy policy_;

    std::priority_queue<HalfedgeErrorEntry, std::vector<HalfedgeErrorEntry>, MinErrorComparator> heap_;
    // 按 halfedge->poolId() 稠密索引的 version slot。0 表示"未入队/已失效"，非零为 globalVersionCounter_。
    std::vector<std::uint64_t> currentVersions_;
    std::vector<int> touchedVersionSlots_;
    std::size_t versionsSize_ = 0;  // 非零 slot 的数量（维护 empty()/size() 语义）
    std::uint64_t globalVersionCounter_ = 0;
    // 按 vertex->poolId() 稠密索引的 visited flag（Vertex error source 模式用）。
    std::vector<std::uint8_t> visitedVertices_;
    std::vector<int> touchedVisitedVertices_;
};
