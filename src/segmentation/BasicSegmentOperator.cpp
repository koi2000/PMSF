/*
 * @Author: koi
 * @Date: 2025-09-13 23:10:21
 * @Description:
 */
#include "BasicSegmentOperator.h"
#include "common/BoundaryGraphDerive.h"
#include "common/BoundedRingTraversal.h"
#include "Global.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "MeshUtils.h"
#include "Vertex.h"
#include "common/Graph.h"
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <random>
#include <vector>

#define RANDOM_SEED 345

void BasicSegmentOperator::segment(std::shared_ptr<MCGAL::Mesh>& mesh) {
    this->mesh = mesh;
    // P18.2：`SegmentOperator::g` 是默认空的 `shared_ptr<Graph>`，而
    // `buildGraph()` 第一件事就是 `g->addHEdge(...)` —— 不先分配就是空指针
    // 解引用。此前本类全树零构造点（第一次被接进管线），所以这个洞从没被
    // 触发过；不补上，第一次真正分区必崩。
    if (!g) {
        g = std::make_shared<Graph>();
    }
    // 管线里的 SEG_TRACE 只在 segment() **返回后**才打，无法区分是
    markBoundary(mesh);
    buildGraph();
}

void BasicSegmentOperator::markBoundary(std::shared_ptr<MCGAL::Mesh>& mesh) {
    // P18.2 路线 (a)：**这里曾经加过「先清残留状态」的重置，已回退。**
    //
    // 当时的推理：`markBoundary` 用 `setProcessedFlag()`/`isProcessed()` 当
    // BFS 访问标记却不清零，而基网上这些标志被消除阶段置脏 —— 于是补了一段
    //     for (fit : mesh->faces()) { fit->resetProcessedFlag(); fit->setGroupId(-1); }
    // （对齐 `PreDefinedFacetSegmentOperator.cpp:55-59` 的既有做法）。
    //
    // **实测证明该改动有害且无效**：
    //   有害 —— step 10 的 `unassigned` 从 **0 变成 2334**（N=8）/ 4768（N=16）。
    //           这些面 groupId 停在 -1，而分段 collector 的过滤器
    //           （`f->groupId() != groupFilter_`）会**直接跳过**它们 ⇒ 丢符号。
    //           即：该重置把「上游残留状态」当噪声清掉的同时，也把
    //           `markBoundary` 赖以达到 unassigned=0 的那部分状态清掉了。
    //   无效 —— 基网上的挂死**依旧**（分阶段打点显示卡在 markBoundary 本身）。
    // ⇒ 按「回退优先于强推」回退。
    //
    // 记下两条线索供下一轮：基网挂死发生在 **markBoundary 内部**（不是
    // buildGraph），且 step 10 的 unassigned=0 依赖某种**外部既有状态** ——
    // 这两件事很可能是同一个东西。

    // P18.2 路线 (a) 第三障碍修复：`fsize` 必须取**活面数**。
    //
    // `mesh->size_of_facets()` 是**高水位**，**把已移除的面也算进去** ——
    // 消元会分裂面，基网上它报 135,751，而原网格只有 69,664，
    // 实际存活的面更少（实测扫 90,000 条只有 3,275 条非 removed）。
    //
    // 下面 `targetNumber = fsize / sampleNumber` ⇒ 基网上 **target 被算大约
    // 10 倍**（≈16,968 而非 ≈1,776）。附 9d 里那个「BFS 一口气吸到 13,459」
    // **不是跑飞，是在追一个错的目标**。
    //
    // 故先数活面再算 targetNumber。计数与下面 allSet 的装载用同一条
    // `isRemoved()` 判据，保证两者一致。
    int fsize = 0;
    for (MCGAL::Facet* fit : mesh->faces()) {
        if (!fit->isRemoved()) {
            ++fsize;
        }
    }
    int sampleNumber = number;
    std::vector<int> targetNumber(sampleNumber, fsize / sampleNumber);
    targetNumber[sampleNumber - 1] = fsize - (sampleNumber - 1) * (fsize / sampleNumber);
    std::vector<int> tmpNumber(sampleNumber, 0);
    std::set<int> allSet;
    // P18.2 路线 (a) 修复：**只收存活面**。
    //
    // 消元之后 `mesh->faces()` 仍会吐出已移除/陈旧的 facet（实测 bunny
    // 只有 69,664 个面，却取到 poolId 50213 / 87273），它们的 halfedge 环
    // 是断的 ⇒ 下面两处 `do { … st = st->next(); } while (st != ed)` 成环
    // 不闭合，直接死循环（`[SEG_RING] walk1/walk2 exceeded 100000`）。
    //
    // 同文件的既定做法就是这样过滤：`OptimizedSymbolCollectOperator.cpp`
    // 的 :246/:350/:360/:376 一律 `if (!f || f->isRemoved() || …) continue;`。
    // `markBoundary` 此前无条件全收 —— 在 step 10（干净网格）看不出问题，
    // 在基网（消元后）就挂死。
    long long faceScan = 0;
    for (MCGAL::Facet* fit : mesh->faces()) {
        if (fit->isRemoved()) {
            continue;
        }
        allSet.insert(fit->poolId());
    }
    int idx = 0;

    std::map<int, int> group2cnt;
    std::mt19937 gen(RANDOM_SEED);
    // P18.2 阶段 B 物品 1：exact-N 模式。
    //
    // 既有模式（`exactN_ == false`）会把过小的组并进邻居、且允许 `idx` 越过
    // `sampleNumber`，于是实际组数是「目标」而非「保证」（实测 N=8→7、
    // 16→17、64→63）。`exactN_` 打开时：不做 merge、跑满 sampleNumber 组、
    // 最后一组吸收余量 ⇒ 恰好 N 组。默认关闭 ⇒ 既有行为逐字节不变。
    // 若这里持续打印而 idx 不前进 ⇒ 空组守卫在每次只推进一个面。
    long long groupLoopIter = 0;
    while (!allSet.empty() && (!exactN_ || idx < sampleNumber)) {
        std::set<int> group;
        std::uniform_int_distribution<> dis(0, allSet.size() - 1);
        auto it = allSet.begin();
        std::advance(it, dis(gen));
        MCGAL::Facet* fit = MCGAL::contextPool.getFacetByIndex(mesh->meshId(), *it);
        fit->setGroupId(idx);
        // int cnt = 1;

        // 本组的目标面数。exactN_ 模式下最后一组吸收全部余量，
        // 使总组数恰为 sampleNumber（其余各档仍是均分的 targetNumber）。
        int target = targetNumber[idx % targetNumber.size()];
        if (exactN_ && idx == sampleNumber - 1) {
            target = static_cast<int>(allSet.size());
        }

        std::queue<int> gateQueue;
        gateQueue.push(fit->poolId());
        std::set<int> neighbours;
        long long bfsPops = 0;
        while (!gateQueue.empty()) {
            int fid = gateQueue.front();
            gateQueue.pop();
            MCGAL::Facet* f = MCGAL::contextPool.getFacetByIndex(mesh->meshId(), fid);
            // P18.2：已移除的面直接跳过 —— 否则会走它的坏环，把两个环走的
            // 100000 次上限白白烧掉（表现为「有界但极慢」，实测仍超时）。
            if (f->isRemoved() || f->isProcessed()) {
                continue;
            }
            f->setProcessedFlag();
            group.insert(f->poolId());
            int cnt = group.size();
            // P18.2 根因：**`==` 应为 `>=`**。
            //
            // BFS 每次插入邻居后比较 `cnt == target`。但一次邻域遍历可能
            // **连插多个**，`cnt` 会直接**越过** `target`（1768 → 1770），
            // 此后 `==` 永不成立 ⇒ BFS 不再收缩 ⇒ **吞掉整个网格**。
            //
            // 实测（基网探针，N=8，target=1769）：
            //     [SEG_GROUP] bfs pops=20000 queue=120 group=13459
            // 第一组的 BFS 已经吸到 13,459 个面还在继续。
            //
            // step 10（干净网格）上每次恰好落在 target，所以 `==` 碰巧成立、
            // 从没暴露 —— 与物品 1 的 `>`/`>=` 是**同一类** bug。
            // 达到目标即**结束本组 BFS**（`continue` 不够：它只是不再扩展，
            // 但仍会把队列里剩下的面逐个 pop 并 `group.insert`，于是 group
            // 继续膨胀 —— 实测基网上涨到 13,459，而 target 只有 1,769）。
            if (cnt >= target) {
                break;
            }
            MCGAL::Halfedge* st = f->proxyHalfedge();
            MCGAL::Halfedge* ed = st;
            MCGAL::Vertex* minV = st->vertex();
            MCGAL::Halfedge* hIt = st;
            // P18.2 诊断：环走可能不闭合。基网（消元后）上 removed 面的
            // halfedge 环可能是断的 ⇒ `st->next()` 成环、永不回到 `ed`
            // ⇒ 死循环（实测基网探针卡在 markBoundary 内即此处嫌疑）。
            int ringGuard1 = 0;
            const int kRingGuard = 100000;
            do {
                if (++ringGuard1 > kRingGuard) {
                    std::fprintf(stderr,
                                 "[SEG_RING] walk1 (min-vertex) exceeded %d at facet %d\n",
                                 kRingGuard, f->poolId());
                    std::fflush(stderr);
                    break;
                }
                if (MCGAL::compareVertex(minV, st->vertex())) {
                    minV = st->vertex();
                    hIt = st;
                }
                st = st->next();
            } while (st != ed);

            MCGAL::Halfedge* h = hIt;
            int ringGuard2 = 0;
            do {
                if (++ringGuard2 > kRingGuard) {
                    std::fprintf(stderr,
                                 "[SEG_RING] walk2 (neighbour) exceeded %d at facet %d\n",
                                 kRingGuard, f->poolId());
                    std::fflush(stderr);
                    break;
                }
                MCGAL::Halfedge* hOpp = hIt ? hIt->opposite() : nullptr;
                // 开放边界上的 halfedge 没有 opposite；跳过即可。
                if (!hOpp || !hOpp->face()) {
                    hIt = hIt->next();
                    continue;
                }
                if (hOpp->face()->isRemoved()) {
                    // P18.2：不把已移除的面纳入分组/队列（同上面的理由）。
                    hIt = hIt->next();
                    continue;
                }
                if (!hOpp->face()->isProcessed()) {
                    hOpp->face()->setGroupId(f->groupId());
                    gateQueue.push(hOpp->face()->poolId());
                    // if (!group.count(hOpp->face->poolId)) {
                    //     cnt++;
                    // }
                    group.insert(hOpp->face()->poolId());
                    int cnt = group.size();
                    if (cnt >= target) {  // P18.2：同上的 `==` → `>=`
                        break;
                    }
                } else if (hOpp->face()->groupId() != f->groupId() && hOpp->face()->groupId() != -1) {
                    neighbours.insert(hOpp->face()->groupId());
                }
                hIt = hIt->next();
            } while (hIt != h);
        }
        // P18.2 缺陷修复（N=128 挂死）：上面随机取到的 facet 若已被标记
        // processed，第 64 行的 `continue` 让 BFS 立刻排空，`group` 为空；
        // 下面的 set_difference 于是**什么都没删**，allSet 不缩，
        // `while (!allSet.empty())` 死循环。
        // 实测：N=128 时 100% CPU 跑满 1 小时 44 分不结束（wall == CPU time
        // ⇒ 纯自旋，不是慢）。空组时强制把该 facet 移出 allSet，保证每轮有进展。
        if (group.empty()) {
            allSet.erase(*it);
            continue;
        }

        int cnt = group.size();
        group2cnt[idx] = cnt;
        // `group` is a subset of `allSet`.  Erasing those keys in place
        // preserves the exact ordered set seen by the next random selection,
        // while avoiding a full O(|allSet|) copy for every partition.
        for (int fid : group) {
            allSet.erase(fid);
        }
        // merge 分支：exactN_ 下整体关闭（merge 正是组数失准的来源）。
        if (!exactN_ && (cnt < target / 2 || idx > sampleNumber)) {
            int mincnt = INT_MAX;
            int resId = -1;
            for (int nid : neighbours) {
                int cnt = group2cnt[nid];
                if (cnt < mincnt) {
                    resId = nid;
                    mincnt = cnt;
                }
            }
            group2cnt[resId] += cnt;
            for (int fid : group) {
                fit = MCGAL::contextPool.getFacetByIndex(mesh->meshId(), fid);
                // fit->groupId = resId;
                fit->setGroupId(resId);
            }
            continue;
        }
        seeds.push_back(fit->proxyHalfedge());
        idx++;
    }
    // P18.2 路线 (a) 第四障碍：**这里也必须跳过已移除的面**。
    //
    // 分组循环跑完之后卡在这里（实测：[SEG_GROUP] iter=8 idx=7 之后
    // 再无 `markBoundary done`）。本循环遍历 `mesh->faces()` 的**全部**
    // 135,751 个条目（其中 96% 是 removed），而 removed 面的 halfedge 环是
    // 断的 ⇒ `halfedges_begin()` 到 `halfedges_end()` 走不到头。
    // 与前三处是同一个根因，只是位置不同。
    for (MCGAL::Facet* fit : mesh->faces()) {
        if (fit->isRemoved()) {
            continue;
        }
        for (auto it = fit->halfedges_begin(); it != fit->halfedges_end(); it++) {
            MCGAL::Halfedge* he = *it;
            MCGAL::Halfedge* ho = he ? he->opposite() : nullptr;
            if (!he || !he->face() || !ho || !ho->face()) continue;
            if (he->face()->groupId() != ho->face()->groupId()) {
                he->setBoundary();
                ho->setBoundary();
            }
        }
    }
}

void BasicSegmentOperator::buildGraph() {
    // 开放边界网格上 MCGAL::next_boundary 会解引用 null opposite；
    // 这里不需要 boundary graph 时直接跳过（组划分已完成）。
    for (MCGAL::Facet* fit : mesh->faces()) {
        if (!fit || fit->isRemoved()) continue;
        for (auto hitIt = fit->halfedges_begin(); hitIt != fit->halfedges_end(); ++hitIt) {
            MCGAL::Halfedge* hit = *hitIt;
            if (!hit) continue;
            if (!hit->opposite() || !hit->opposite()->face()) {
                return;
            }
        }
    }
    std::set<MCGAL::Vertex*> triPoints;
    for (MCGAL::Vertex* vit : mesh->vertices()) {
        std::set<int> cnt;
        for (MCGAL::Halfedge* hit : pmsf::boundedRing(vit)) {
            cnt.insert(hit->face()->groupId());
        }
        if (cnt.size() >= 3) {
            triPoints.insert(vit);
        }
    }
    for (MCGAL::Vertex* vit : triPoints) {
        for (MCGAL::Halfedge* hit : pmsf::boundedRing(vit)) {
            if (hit->isBoundary()) {
                // if (triPoints.count(hit->vertex) && triPoints.count(hit->end_vertex)) {
                //     g.addHEdge(hit->face->groupId, {hit->vertex->id, hit->end_vertex->id, hit->opposite->face->groupId, hit->end_vertex->id});
                //     continue;
                // }
                MCGAL::Halfedge* st = hit;
                int inner = st->face()->groupId();
                int outer = st->opposite()->face()->groupId();
                int dis = 0;
                // P18.2 缺陷修复（N=8 挂死点）：`next_boundary(inner, outer, st)`
                // 会在 `st->end_vertex()` 的环上搜出**已经走过**的那条半边，
                // 于是 st 成环、永不返回 nullptr，而循环条件正是
                // `while (st != nullptr)` ⇒ **死循环**（实测 N=8 exact-N：
                // 100% CPU，wall == CPU time，永不结束）。
                //
                // `next_boundary` 在 deps/MCGAL/include/MeshUtils.h:825 是
                // static 函数，MCGAL 禁改 ⇒ 在**调用方**加已访问集合：
                // 每条边界半边最多走一次，环闭合即停。语义不变（本来就是
                // 走一圈），只是把「永不终止」变成「走完即止」。
                std::set<MCGAL::Halfedge*> visitedBoundary;
                do {
                    assert(st->isBoundary());
                    if (!visitedBoundary.insert(st).second) {
                        break;  // 环已闭合：已经回到走过的半边
                    }
                    MCGAL::Halfedge* ed = st;
                    if (triPoints.count(st->end_vertex())) {
                        int i = 0;
                    }
                    st = next_boundary(inner, outer, st);

                    if (st == nullptr) {
                        g->addHEdge(inner, outer, {hit->vertex()->poolId(), hit->end_vertex()->poolId(), ed->end_vertex()->poolId()});
                    }
                    dis++;
                } while (st != nullptr);
                // g.addEdge({hit->vertex->id, hit->end_vertex->id, hit->opposite->face->groupId, st->end_vertex->id}, {st->end_vertex->id, st->vertex->id, hit->face->groupId, hit->vertex->id});
            }
        }
    }
    g->setTriPoints(triPoints);
    if (std::getenv("PMSF_BOUNDARY_GRAPH_SCHEME") &&
        std::string(std::getenv("PMSF_BOUNDARY_GRAPH_SCHEME")) == "start_only") {
        g->setBoundaryScheme(BoundaryGraphScheme::StartOnly);
    }
    if (std::getenv("PMSF_BOUNDARY_GRAPH_VALIDATE_START_ONLY")) {
        int checked = 0;
        int mismatched = 0;
        for (auto& [groupA, neighbors] : g->graph()) {
            for (auto& [groupB, nodes] : neighbors) {
                for (const Node& legacy : nodes) {
                    Node derived;
                    ++checked;
                    if (!pmsf::deriveBoundaryTriple(mesh, groupA, groupB, legacy.st, derived) ||
                        derived.ed != legacy.ed || derived.stop != legacy.stop) {
                        ++mismatched;
                        if (mismatched <= 8) {
                            std::cerr << "[boundary-start-only] mismatch A=" << groupA
                                      << " B=" << groupB << " st=" << legacy.st
                                      << " legacy=(" << legacy.ed << "," << legacy.stop
                                      << ") derived=(" << derived.ed << "," << derived.stop
                                      << ")" << std::endl;
                        }
                    }
                }
            }
        }
        std::cerr << "[boundary-start-only] checked=" << checked
                  << " mismatched=" << mismatched << std::endl;
    }
}

std::vector<MCGAL::Halfedge*> BasicSegmentOperator::exportSeeds() {
    return seeds;
}

std::shared_ptr<Graph> BasicSegmentOperator::exportGraph() {
    return g;
}
