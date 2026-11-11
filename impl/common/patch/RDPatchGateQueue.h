// SPDX-License-Identifier: Apache-2.0
//
// RDPatchGateQueue.h —— RD 论文 §2.2 patch-level conquest 的 6-bucket priority FIFO。
//
// PR-M1（2026-05-12）：
//   论文 RD §2.2 step 1-6 的 gate 优先级编号系统。每条 gate halfedge 入队时按其
//   两端顶点的 ⊕/⊖ tag、所在 patch 是否 null、以及顶点是否"已收满 2/3 incident
//   processed edges"分入 6 个 bucket：
//
//   bucket 1: ⊖⊕ from non-null patches; ⊖ has 2/3 incident processed edges
//   bucket 2: ⊖⊕ from null patches;     ⊖ has 2/3 incident processed edges
//   bucket 3: ⊖⊖ or ⊕⊕ from non-null;   both vertices have 2/3 incident
//   bucket 4: ⊖⊖ or ⊕⊕ from null;        both vertices have 2/3 incident
//   bucket 5: all other from non-null patches
//   bucket 6: all other from null patches
//
//   pop 时从 bucket 1 → 6 顺序取首元素（FIFO）。
//
// PR-M1 简化（详见 docs/paper_alignment/PR_M_RD_PATCH_DESIGN.md §5）：
//   - 暂不实装 tag evolution（所有 vertex tag 视为 Untagged）→ 永远 bucket 5/6
//   - 暂不实装 null-patch 区分 → 永远 bucket 5（non-null）
//   - 即 PR-M1 阶段 6-bucket 退化为单 FIFO；priority API 完整保留以便 M2/M3 真接入
//
// 设计要点：
//   - 队列里存 Halfedge*；priority 由 push() 时计算的 bucket 决定，不动态重排
//   - 每个 bucket 内部用 std::deque<Halfedge*> 维持 FIFO
//   - 同一 halfedge 不重复入队：encoder/decoder 都靠 mesh 持久状态（face::isProcessed 等）
//     在 push 之前先判 dedup；本队列不强制 set
//   - empty()/pop() 由调用方在主循环里使用，与 phase 切换协同
//
// 详见 docs/paper_alignment/PR_M_RD_PATCH_DESIGN.md §2.1, §4.1。

#ifndef PMSF_COMPRESS_RD_PATCH_GATE_QUEUE_H
#define PMSF_COMPRESS_RD_PATCH_GATE_QUEUE_H

#include <array>
#include <cstddef>
#include <deque>

namespace MCGAL {
class Halfedge;
}  // namespace MCGAL

namespace pmsf {

/// Vertex 在 RD conquest 上的 ⊕/⊖ tag。Untagged 是初始状态，conquest 推进过程中
/// retriangulation 后按论文 Fig. 1 规则赋 ⊕ 或 ⊖。PR-M1 阶段所有 vertex 都保留
/// Untagged（M2 起接入真 evolution）。
enum class RDPatchVertexTag : unsigned char {
    Untagged = 0,
    Plus     = 1,   // ⊕
    Minus    = 2,   // ⊖
};

/// PR-M1 简化：所有 patch 默认 non-null（即"上一次访问该 patch 的 boundary edge
/// 没有标 NULL_PATCH"）。M2 接入 SymbolCollect 时改为 per-Facet 标志。
enum class RDPatchNullFlag : unsigned char {
    NonNull = 0,
    Null    = 1,
};

/// 6-bucket priority FIFO，论文 RD §2.2。
class RDPatchGateQueue {
  public:
    static constexpr int kNumBuckets = 6;

    RDPatchGateQueue() = default;

    /// 计算 halfedge 的 bucket 编号（1..6）。pure function，不读 queue 内部状态。
    /// @param vTagSrc/vTagDst   gate 两端 vertex 的 ⊕/⊖ tag
    /// @param patchNull         halfedge->face() 所在 patch 的 null flag
    /// @param srcSaturated      vertex(src) 已 incident-in-processed = 2/3 of its degree
    /// @param dstSaturated      同上 for end_vertex(dst)
    static int computeBucket(RDPatchVertexTag vTagSrc,
                             RDPatchVertexTag vTagDst,
                             RDPatchNullFlag  patchNull,
                             bool srcSaturated,
                             bool dstSaturated);

    /// 把 halfedge 入对应 bucket 的 FIFO 尾部。bucket 由 caller 算好（解耦 mesh 查询）。
    void push(MCGAL::Halfedge* h, int bucket);

    /// 取出最高优先级 bucket 的队首；空队列返回 nullptr。
    MCGAL::Halfedge* popHighestPriority();

    /// 同 pop，但额外通过 outBucket 返回它来自哪个 bucket（diag 用）。
    MCGAL::Halfedge* popHighestPriority(int* outBucket);

    /// 总元素数。
    std::size_t size() const;

    /// 是否所有 bucket 都空。
    bool empty() const;

    /// 清空所有 bucket（不释放内存）。
    void clear();

    /// 各 bucket 当前 size（diag 用）。
    std::array<std::size_t, kNumBuckets> bucketSizes() const;

    /// P10 前沿驱动规则用：按指针移除指定元素。
    /// 与 popHighestPriority 不同，本方法**保持其余元素的相对次序**（前沿规则
    /// 每步只移走一条，若用"弹出再压回"会反转 bucket 内 FIFO 次序，污染后续）。
    /// @return 找到并移除返回 true；不在队列中返回 false。
    bool removeEntry(MCGAL::Halfedge* h);

    /// P10 阶段 0 诊断用：只读遍历队列内容。
    /// 回调返回 true 时提前停止（避免无谓全扫）。**不改变队列状态**。
    template <typename Fn>
    void forEachEntry(Fn&& fn) const {
        for (const auto& q : buckets_) {
            for (MCGAL::Halfedge* h : q) {
                if (fn(h)) return;
            }
        }
    }

  private:
    /// buckets_[i] 对应论文 bucket i+1（1-based 在论文，0-based 在数组）。
    std::array<std::deque<MCGAL::Halfedge*>, kNumBuckets> buckets_;
};

}  // namespace pmsf

#endif  // PMSF_COMPRESS_RD_PATCH_GATE_QUEUE_H
