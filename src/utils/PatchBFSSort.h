/*
 * @file        PatchBFSSort.h
 * @brief       Phase 1 · Step 1.3 关键不变量 R2 的根基 —— encoder/decoder 共享的
 *              BFS face-visit 顺序约定。
 * @author      koi
 * @date        2026-05-10
 *
 * 见 docs/CONNECTIVITY_BITSTREAM_IMPLEMENTATION_PLAN.md §1.3 + §A R2。
 *
 * 任何"按 BFS 排序的 patch face 列表"在 encoder / decoder / reconstructor 三侧都
 * **必须**通过本头文件提供的 free function 计算，避免代码漂移。
 *
 * 当前 Phase 1 范围内的方案 V (Valence-Patch-Only) 直接 mirror 父类
 * `RDGateOnlySymbolCollectOperator::collect(seed, gates)` / 父类 reader 的 BFS：
 *   - gateQueue 从 seed->poolId() 起；
 *   - 每次 dequeue halfedge h，标记 h->setVisited，face 首访时计 face；
 *   - 然后绕当前 face 的 next() 链，把每个 hIt 的 opposite() 与自身 sibling 入队
 *     （opposite 先入队，自身后入队）。
 *
 * 共享函数：
 *   - sortPatchesByBFS(...)
 *
 * Phase 1 Step 1.1 阶段 collector 仍直接 mirror 父类 BFS；本 free function 给
 * reconstructor/decoder 侧提供同一规则，避免后续接入时再次复制 BFS 细节。
 */

#ifndef PMSF_UTILS_PATCH_BFS_SORT_H
#define PMSF_UTILS_PATCH_BFS_SORT_H

#include <vector>

namespace MCGAL {
class Halfedge;
class Facet;
class Mesh;
}  // namespace MCGAL

namespace pmsf {

/// @brief 按 BFS 顺序对 patch face 列表排序，结果 deterministic 且
///        encoder / decoder 等价。
///
/// 对 seed 无法到达的 patch face，函数按调用方输入顺序追加，确保不会丢失 patch。
std::vector<MCGAL::Facet*> sortPatchesByBFS(
    const std::vector<MCGAL::Facet*>& patches,
    const std::vector<MCGAL::Halfedge*>& seeds,
    MCGAL::Mesh* mesh);

}  // namespace pmsf

#endif  // PMSF_UTILS_PATCH_BFS_SORT_H
