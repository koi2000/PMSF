/*
 * @Author: koi
 * @Date: 2025-08-27 15:24:33
 * @Description:
 */
#include "PriorityErrorSelector.h"
#include "common/BoundedRingTraversal.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Vertex.h"
#include "selection/HalfedgeSelectionQueue.h"
#include "operator/BoundaryIsRemovableOperator.h"
#include <asm-generic/errno.h>
#include <functional>
#include <set>
#include <unordered_set>
#include <vector>

PriorityErrorSelector::PriorityErrorSelector(SelectOptions options) : SelectOperator(options) {
    ErrorSource source = options.getErrorSource();
    auto halfedgeErrorAccessor = options.getHalfedgeErrorAccessor();
    auto vertexErrorAccessor = options.getVertexErrorAccessor();
    auto entryCustomizer = options.getEntryCustomizer();
    queue = HalfedgeSelectionQueue(source, halfedgeErrorAccessor, vertexErrorAccessor,
                                   options.getIsRemovableOperators(), entryCustomizer,
                                   options.getSelectionPolicy());
}

void PriorityErrorSelector::init(std::shared_ptr<MCGAL::Mesh> mesh, bool singleGroupOnly, int groupId) {
    mesh_ = mesh;
    queue.buildFromMesh(mesh, singleGroupOnly, groupId);
}

void PriorityErrorSelector::initFromFaces(std::shared_ptr<MCGAL::Mesh> mesh,
                                          const std::vector<MCGAL::Facet*>& faces) {
    mesh_ = mesh;
    queue.buildFromFaces(faces);
}

void PriorityErrorSelector::addIsRemovableOperator(std::shared_ptr<IsRemovableOperator> op) {
    queue.addIsRemovableOperator(op);
}

PriorityErrorSelector::~PriorityErrorSelector() {}

void PriorityErrorSelector::reset(bool singleGroupOnly, int groupId) {
    mesh_->resetState();
    queue.buildFromMesh(mesh_, singleGroupOnly, groupId);
}

bool PriorityErrorSelector::select(HalfedgeErrorEntry& entry) {
    // perf（2026-11-09 QEM 实测）：该 env 检查原先每次 select 一次（3.4 万次/压缩）。
    static const bool boundarySelectStage = [] {
        const char* st = std::getenv("PMSF_BOUNDARY_SELECT_STAGE");
        return st && st[0] && st[0] != '0';
    }();
    if (boundarySelectStage) {
        static int lastRound = -1;
        static int usedInRound = 0;
        static std::set<MCGAL::Vertex*> chosenThisRound;
        const char* roundEnv = std::getenv("PMSF_BOUNDARY_SELECT_ROUND");
        const int round = roundEnv ? std::atoi(roundEnv) : 0;
        if (round != lastRound) {
            lastRound = round;
            usedInRound = 0;
            chosenThisRound.clear();
        }
        int maxOps = 1;
        if (const char* m = std::getenv("PMSF_BOUNDARY_SELECT_MAX"))
            maxOps = std::max(0, std::atoi(m));
        if (usedInRound < maxOps && mesh_) {
            auto& frozen = pmsf::sharedBoundaryFrozenVertices();
            for (MCGAL::Facet* f : mesh_->faces()) {
                if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                MCGAL::Halfedge* h0 = f->proxyHalfedge();
                MCGAL::Halfedge* h = h0;
                int guard = 0;
                do {
                    if (h && !h->isRemoved() && h->face() && h->opposite() &&
                        h->opposite()->face() && !h->opposite()->face()->isRemoved() &&
                        h->face()->groupId() != h->opposite()->face()->groupId()) {
                        MCGAL::Vertex* v = h->end_vertex();
                        if (v && !v->isRemoved() && frozen.count(v) == 0 &&
                            chosenThisRound.count(v) == 0) {
                            const int deg = v->is_degree_dirty() ? v->vertex_degree()
                                                                 : v->vertex_degree_cached();
                            if (deg >= 4 && deg <= 10) {
                                bool conflict = false;
                                for (MCGAL::Halfedge* rh : pmsf::boundedRing(v)) {
                                    if (rh && rh->end_vertex() &&
                                        chosenThisRound.count(rh->end_vertex())) {
                                        conflict = true;
                                        break;
                                    }
                                }
                                if (!conflict) {
                                    chosenThisRound.insert(v);
                                    for (MCGAL::Halfedge* rh : pmsf::boundedRing(v))
                                        if (rh && rh->end_vertex())
                                            chosenThisRound.insert(rh->end_vertex());
                                    entry = HalfedgeErrorEntry(h, MCGAL::Point(0, 0, 0), 0, 0);
                                    ++usedInRound;
                                    return true;
                                }
                            }
                        }
                    }
                    h = h ? h->next() : nullptr;
                } while (h && h != h0 && ++guard < 64);
            }
        }
    }
    entry = queue.popNextEntry();
    // entry.edge = pair.first;
    // entry.p = pair.second;
    if (entry.edge == nullptr) {
        return false;
    }
    return true;
}

bool PriorityErrorSelector::update(HalfedgeErrorEntry& entry) {
    // ============================================================
    // QEM 关键路径：在 eliminate（顶点合并 / 边折叠）之后，
    //  1) 调用 onAfterEliminate 钩子让算法侧刷新顶点属性（如 Q 矩阵）
    //  2) 重算合并幸存顶点 v0 周围所有非 removed 半边的 cost，并刷新优先队列
    // 之前实现里 `h->isRemoved()` 的取反 bug 让 update 实际为空操作，
    // QEM 复现一直停留在"使用初始 Q + 初始 cost"的状态，这是 BPV / 几何
    // 质量与原版差距巨大的根本原因之一。
    // ============================================================
    if (entry.edge == nullptr) {
        return false;
    }
    MCGAL::Halfedge* halfedge = entry.edge;
    if (halfedge == nullptr || halfedge->isRemoved()) {
        return false;
    }

    // VertexRemovalEliminateOperator rewrites entry.edge to the generated
    // patch face proxy. For QEM that means the survivor-only update is too
    // narrow: all patch vertices can have changed incident faces/normals, and
    // their queued target points must be recomputed before the next pop.
    std::vector<MCGAL::Vertex*> affected;
    ++seenEpoch_;
    const std::uint64_t seenE = seenEpoch_;
    auto addVertex = [&](MCGAL::Vertex* v) {
        if (v == nullptr || v->isRemoved()) return;
        const int id = v->poolId();
        if (id < 0) return;
        if (static_cast<std::size_t>(id) >= seenStamp_.size()) {
            seenStamp_.resize(static_cast<std::size_t>(id) + 1024, 0);
        }
        if (seenStamp_[static_cast<std::size_t>(id)] != seenE) {
            seenStamp_[static_cast<std::size_t>(id)] = seenE;
            affected.push_back(v);
        }
    };

    addVertex(halfedge->vertex());
    if (MCGAL::Facet* f = halfedge->face(); f != nullptr && !f->isRemoved()) {
        for (MCGAL::Vertex* v : f->getVertices()) {
            addVertex(v);
            if (v == nullptr || v->isRemoved()) continue;
            for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
                if (h != nullptr && !h->isRemoved()) addVertex(h->end_vertex());
            }
        }
    }

    if (affected.empty()) return false;

    // 1) 算法层面属性刷新（QEM: recompute Q on affected vertices).
    if (options_.getOnAfterEliminate()) {
        for (MCGAL::Vertex* v : affected) {
            options_.getOnAfterEliminate()(v);
        }
    }

    // 2) 把受影响顶点周围的半边重新算 cost 入队（lazy-remove 旧版本号）。
    //    cost(u,v) = cost(v,u) 对 QEM/大多数 metric 成立，只推一个方向。
    ++pushedEpoch_;
    const std::uint64_t pushedE = pushedEpoch_;
    for (MCGAL::Vertex* v : affected) {
        for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
            if (h == nullptr || h->isRemoved()) continue;
            MCGAL::Halfedge* opp = h->opposite();
            MCGAL::Halfedge* canonical = h;
            if (opp && !opp->isRemoved() && opp->poolId() < h->poolId()) {
                canonical = opp;
            }
            const int cid = canonical->poolId();
            if (cid >= 0) {
                if (static_cast<std::size_t>(cid) >= pushedStamp_.size()) {
                    pushedStamp_.resize(static_cast<std::size_t>(cid) + 1024, 0);
                }
                if (pushedStamp_[static_cast<std::size_t>(cid)] != pushedE) {
                    pushedStamp_[static_cast<std::size_t>(cid)] = pushedE;
                    queue.pushOrUpdate(canonical);
                }
            }
        }
    }
    return true;
}
