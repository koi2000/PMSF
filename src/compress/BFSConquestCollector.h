/*****************************************************************************
 * @Author: koi
 * @Description: 通用 face-conquest BFS 遍历框架。
 * 阶段 G：把 "opposite/self/next" 面访问顺序、visited 标记、gate queue 管理等
 *        boilerplate 从各算法 SymbolCollectOperator 中抽象出来。算法实现只需
 *       提供 BFSConquestVisitor 回调，在 face 首访时写入自己的符号队列。
 *****************************************************************************/
#ifndef PMSF_COMPRESS_BFS_CONQUEST_COLLECTOR_H
#define PMSF_COMPRESS_BFS_CONQUEST_COLLECTOR_H

#include "BFSVersionManager.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"

#include <cstdint>
#include <queue>
#include <vector>

namespace pmsf {

/// @brief BFS face-conquest 访问器接口。算法实现者只关心每个 face 首次被访问时
///        应该产出什么符号；BFS 顺序、queue 管理、visited 标记由 runFaceConquestBFS 统一处理。
class BFSConquestVisitor {
  public:
    virtual ~BFSConquestVisitor() = default;

    /// @brief 一轮 BFS 开始前的初始化回调。
    /// @param roundIdx   当前轮次（encoder 顺序编号）。
    /// @param gateCount  本轮期望访问到的 polyFace 数（由上游 eliminate 提供）。
    virtual void onRoundBegin(int roundIdx, std::size_t gateCount) {
        (void)roundIdx;
        (void)gateCount;
    }

    /// @brief 某个 face 第一次被 BFS 访问到时调用。
    /// @param f          被访问的 face。
    /// @param gate       BFS 进入该 face 的入口 halfedge。
    /// @param bfsFaceIdx 本轮内 face 访问顺序编号（0-based）。
    virtual void onFaceFirstVisit(MCGAL::Facet* f,
                                  MCGAL::Halfedge* gate,
                                  int bfsFaceIdx) = 0;

    /// @brief 一轮 BFS 结束后的收尾回调。
    /// @param bfsFaceCount 本轮实际访问到的 face 数量。
    virtual void onRoundEnd(int bfsFaceCount) { (void)bfsFaceCount; }
};

/// @brief 执行一次与 decoder 严格对称的 face-conquest BFS。
///
/// 遍历顺序：
///   1. 以 seed halfedge 为起点入队；
///   2. 每取出一条 halfedge，先标记 visited；
///   3. 若其所在 face 未被访问，则调用 visitor.onFaceFirstVisit；
///   4. 邻居入队顺序：opposite（若存在且未访问） -> 自身 sibling -> 沿 face 的 next() 继续。
///
/// 该顺序必须与对应 reader 的 BFS 完全一致；修改时需同步验证 round-trip。
///
/// @return 本轮实际访问到的 face 数量。
inline int runFaceConquestBFS(
    MCGAL::Halfedge* seed,
    const std::vector<MCGAL::Halfedge*>& gates,
    MCGAL::Mesh* mesh,
    BFSConquestVisitor& visitor,
    const char* /*diagLabel*/ = nullptr) {
    if (mesh == nullptr) {
        return 0;
    }

    int current_version = MCGAL::BFSVersionManager::current_version++;
    std::queue<int> gateQueue;
    if (seed != nullptr) {
        gateQueue.push(seed->poolId());
    }

    int bfsFaceIdx = 0;
    while (!gateQueue.empty()) {
        int hid = gateQueue.front();
        gateQueue.pop();

        MCGAL::Halfedge* h =
            MCGAL::contextPool.getHalfedgeByIndexInSubPool(mesh->meshId(), hid);
        if (h == nullptr || h->isVisited(current_version) || h->isRemoved()) {
            continue;
        }
        h->setVisited(current_version);

        MCGAL::Facet* f = h->face();
        bool faceFirstVisit = (f != nullptr) && !f->isVisited(current_version);
        if (faceFirstVisit) {
            f->setVisited(current_version);
            visitor.onFaceFirstVisit(f, h, bfsFaceIdx);
            ++bfsFaceIdx;
        }

        // 邻居入队：与 decoder 严格对称
        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp != nullptr && !hOpp->isVisited(current_version)) {
                gateQueue.push(hOpp->poolId());
            }
            if (!hIt->isVisited(current_version)) {
                gateQueue.push(hIt->poolId());
            }
            hIt = hIt->next();
        } while (hIt != h && hIt != nullptr);
    }

    visitor.onRoundEnd(bfsFaceIdx);
    return bfsFaceIdx;
}

}  // namespace pmsf

#endif  // PMSF_COMPRESS_BFS_CONQUEST_COLLECTOR_H
