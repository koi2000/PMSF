/*
 * @Author: koi
 * @Date: 2025-08-27 15:24:33
 * @Description:
 */
#include "GreedySelector.h"
#include "BFSVersionManager.h"
#include "Facet.h"
#include "Halfedge.h"
#include "selection/HalfedgeSelectionQueue.h"
#include <asm-generic/errno.h>
#include <cstdlib>
#include <cstring>
#include <functional>
#include "common/BoundedRingTraversal.h"
#include "operator/BoundaryIsRemovableOperator.h"

namespace {

bool envOn(const char* name) {
    const char* v = std::getenv(name);
    return v && *v && std::strcmp(v, "0") != 0;
}

int removedCenterValence(MCGAL::Halfedge* h) {
    if (!h || !h->next() || !h->next()->opposite()) return 0;
    int degree = 1;
    MCGAL::Halfedge* g = h->next()->opposite();
    int guard = 0;
    while (g && g != h) {
        ++degree;
        if (!g->next() || !g->next()->opposite()) return 0;
        g = g->next()->opposite();
        if (++guard > 128) return 0;
    }
    return degree;
}

}  // namespace

GreedySelector::GreedySelector(SelectOptions options) : SelectOperator(options) {
    visited_version = MCGAL::BFSVersionManager::current_version++;
    isRemovableOperators_ = options.getIsRemovableOperators();
    policy_ = options.getSelectionPolicy();
}

void GreedySelector::init(std::shared_ptr<MCGAL::Mesh> mesh, bool singleGroupOnly, int groupId) {
    mesh_ = mesh;
    this->singleGroupOnly_ = singleGroupOnly;
    this->groupId_ = groupId;
}

void GreedySelector::addIsRemovableOperator(std::shared_ptr<IsRemovableOperator> op) {
    this->isRemovableOperators_.push_back(op);
}

 void GreedySelector::selectSeed(MCGAL::Halfedge*& seed, std::function<MCGAL::Halfedge*(std::shared_ptr<MCGAL::Mesh>)> seedSelector) {
    if (seedSelector) {
        seed = seedSelector(mesh_);
    } else {
        size_t i_heInitId = (float)rand() / RAND_MAX * mesh_->size_of_facets();
        seed = mesh_->get_facet(i_heInitId)->proxyHalfedge();
    }
    seed_ = seed;
    gateQueue.push(seed_);
};

GreedySelector::~GreedySelector() {}

void GreedySelector::refreshRoundState() {
    if (!mesh_) return;
    // MCGAL 的 incCachedDegree/decCachedDegree 只在缓存「非 dirty」时生效；而
    // vertex_degree() 只有在 dirty 时才重算。二者组合下，若某顶点的缓存一直是
    // dirty，拓扑增量永远不会被记录，直到被读到时才按当时的环重算——这本身没错，
    // 但 PPMC 的 isRemovable 读 degree 的时机与 split/erase 交错，导致部分顶点
    // 的 degree 在「增量已失效」的窗口里被读到。每轮开始前统一重算一次
    // （让后续增量更新处在非 dirty 状态）即可消除这种时序依赖。
    // 实测 horse19k：15 轮 base 419V→133V，artifact 49,807→48,268 B。
    for (MCGAL::Vertex* v : mesh_->vertices()) {
        if (!v || v->isRemoved()) continue;
        (void)v->vertex_degree();
    }
}

void GreedySelector::reseedLive() {
    if (!mesh_) return;
    const size_t n = mesh_->size_of_facets();
    if (n == 0) return;
    const size_t start = static_cast<size_t>((static_cast<float>(rand()) / RAND_MAX) * static_cast<float>(n));
    for (size_t k = 0; k < n; ++k) {
        MCGAL::Facet* f = mesh_->get_facet((start + k) % n);
        if (f && !f->isRemoved() && f->proxyHalfedge() && !f->proxyHalfedge()->isRemoved()) {
            seed_ = f->proxyHalfedge();
            return;
        }
    }
}

void GreedySelector::reset(bool singleGroupOnly, int groupId) {
    singleGroupOnly_ = singleGroupOnly;
    groupId_ = groupId;
    gateQueue.push(seed_);
    mesh_->resetState();
    freshProbeDone_ = false;
}

bool GreedySelector::inActiveGroup(MCGAL::Halfedge* h) const {
    if (!singleGroupOnly_) {
        return h != nullptr;
    }
    if (!h) return false;
    if (h->face()) {
        return h->face()->groupId() == groupId_;
    }
    return h->vertex() && h->vertex()->groupId() == groupId_;
}

bool GreedySelector::isRemovable(MCGAL::Halfedge* h) {
    for (const auto& op : isRemovableOperators_) {
        HalfedgeErrorEntry entry(h);
        if (!op->isRemovable(entry)) {
            return false;
        }
    }
    return true;
}

bool GreedySelector::select(HalfedgeErrorEntry& entry) {
    // [R1 修复] 每轮第一次 select 前刷新 degree 缓存（含 round 0；pipeline 的
    // reset 钩子对第 0 轮不生效，必须放在 selector 内）。
    if (refreshDegreesEachRound_ && !freshProbeDone_ && mesh_) {
        freshProbeDone_ = true;
        refreshRoundState();
    }
    // [boundary stage selector prototype, env gated] Prefer one group-boundary
    // vertex per round, bypassing the normal boundary protection operator.
    // perf（2026-11-08 bunny 实测）：该 env 检查原先每个 select 调用一次
    // （35,370 次/次压缩）。开关进程内不变，缓存一次；语义不变。
    static const bool boundarySelectStage = [] {
        const char* st = std::getenv("PMSF_BOUNDARY_SELECT_STAGE");
        return st && st[0] && st[0] != '0';
    }();
    if (boundarySelectStage) {
        static int lastRound = -1;
        static int usedInRound = 0;
        const char* roundEnv = std::getenv("PMSF_BOUNDARY_SELECT_ROUND");
        const int round = roundEnv ? std::atoi(roundEnv) : 0;
        if (round != lastRound) { lastRound = round; usedInRound = 0; }
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
                        if (v && !v->isRemoved() && frozen.count(v) == 0) {
                            const int deg = v->is_degree_dirty() ? v->vertex_degree()
                                                                 : v->vertex_degree_cached();
                            if (deg >= 4 && deg <= 10) {
                                entry.edge = h;
                                ++usedInRound;
                                return true;
                            }
                        }
                    }
                    h = h ? h->next() : nullptr;
                } while (h && h != h0 && ++guard < 64);
            }
        }
    }
    // 名字随契约走：该值来自 SelectionPolicy，与任何具体算法无关。
    const bool useCustomCandidateScore = static_cast<bool>(policy_.candidateScore);
    while (!gateQueue.empty()) {
        MCGAL::Halfedge* h = gateQueue.front();
        gateQueue.pop();
        if (!inActiveGroup(h)) {
            continue;
        }
        MCGAL::Facet* f = h->face();
        if (f->isConquered()) {
            continue;
        }
        int numSplittableVerts = 0;
        MCGAL::Halfedge* unconqueredVertexHE = nullptr;
        int bestScore = -1000000;
        for (MCGAL::Halfedge* hh = h->next(); hh && hh != h; hh = hh->next()) {
            if (isRemovable(hh)) {
                if (!useCustomCandidateScore && numSplittableVerts == 0) {
                    unconqueredVertexHE = hh;
                } else if (useCustomCandidateScore) {
                    const int score = static_cast<int>(policy_.candidateScore(hh));
                    if (!unconqueredVertexHE || score > bestScore) {
                        unconqueredVertexHE = hh;
                        bestScore = score;
                    }
                }
                ++numSplittableVerts;
            }
        }
        if (numSplittableVerts == 0) {
            f->setUnsplittable();
            MCGAL::Halfedge* hh = h;
            while (hh) {
                hh->end_vertex()->setConquered();
                MCGAL::Halfedge* hOpp = hh->opposite();

                if (hOpp && hOpp->face() && inActiveGroup(hOpp) && !hOpp->face()->isConquered()) {
                    gateQueue.push(hOpp);
                }
                hh = hh->next();
                if (hh == h) break;
            }
        } else {
            entry.edge = unconqueredVertexHE;
            return true;
        }
    }
    return false;
}

bool GreedySelector::update(HalfedgeErrorEntry& entry) {
    MCGAL::Halfedge* h = entry.edge;
    MCGAL::Halfedge* hNewFace = entry.edge;
    MCGAL::Facet* f = h->face();
    while (h) {
        MCGAL::Halfedge* hOpp = h->opposite();
        // assert(!hOpp->is_border());
        if (hOpp && hOpp->face() && inActiveGroup(hOpp) && !hOpp->face()->isConquered()) {
            gateQueue.push(hOpp);
        }
        h = h->next();
        if (h == hNewFace) break;
    }
    // Increment the number of removed vertices.
    i_nbRemovedVertices++;
    return true;
}
