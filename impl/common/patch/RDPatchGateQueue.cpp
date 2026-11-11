// SPDX-License-Identifier: Apache-2.0
//
// RDPatchGateQueue.cpp —— see header for design contract.
//
// 实装要点：
//   bucket 1..6 完全按论文 §2.2 step 1..6 的次序定义；computeBucket() 是
//   pure function，可被 encoder 和 decoder 同样调用，对同一 mesh 状态返回同
//   bucket 编号 → 这是双端 conquest 同步的核心 invariant。

#include "common/patch/RDPatchGateQueue.h"

#include "Halfedge.h"

namespace pmsf {

// 论文 §2.2 6-bucket priority。返回 [1, 6]。
int RDPatchGateQueue::computeBucket(RDPatchVertexTag vTagSrc,
                                    RDPatchVertexTag vTagDst,
                                    RDPatchNullFlag  patchNull,
                                    bool srcSaturated,
                                    bool dstSaturated) {
    const bool isPlusMinus =
        (vTagSrc == RDPatchVertexTag::Minus && vTagDst == RDPatchVertexTag::Plus) ||
        (vTagSrc == RDPatchVertexTag::Plus  && vTagDst == RDPatchVertexTag::Minus);
    const bool isPureSign =
        (vTagSrc == RDPatchVertexTag::Plus  && vTagDst == RDPatchVertexTag::Plus) ||
        (vTagSrc == RDPatchVertexTag::Minus && vTagDst == RDPatchVertexTag::Minus);

    // bucket 1/2: ⊖⊕ where ⊖ has 2/3 incident processed edges
    // 论文 step 1: ⊖ has 2/3 → 即 src 是 ⊖ 的话看 srcSaturated；dst 是 ⊖ 的话看 dstSaturated
    if (isPlusMinus) {
        const bool minusSaturated = (vTagSrc == RDPatchVertexTag::Minus) ? srcSaturated : dstSaturated;
        if (minusSaturated) {
            return (patchNull == RDPatchNullFlag::Null) ? 2 : 1;
        }
    }

    // bucket 3/4: ⊖⊖ or ⊕⊕，**两端都** saturated
    if (isPureSign && srcSaturated && dstSaturated) {
        return (patchNull == RDPatchNullFlag::Null) ? 4 : 3;
    }

    // bucket 5/6: 其余
    return (patchNull == RDPatchNullFlag::Null) ? 6 : 5;
}

void RDPatchGateQueue::push(MCGAL::Halfedge* h, int bucket) {
    if (h == nullptr) return;
    if (bucket < 1 || bucket > kNumBuckets) return;
    buckets_[bucket - 1].push_back(h);
}

MCGAL::Halfedge* RDPatchGateQueue::popHighestPriority() {
    return popHighestPriority(nullptr);
}

MCGAL::Halfedge* RDPatchGateQueue::popHighestPriority(int* outBucket) {
    for (int i = 0; i < kNumBuckets; ++i) {
        if (!buckets_[i].empty()) {
            MCGAL::Halfedge* h = buckets_[i].front();
            buckets_[i].pop_front();
            if (outBucket) *outBucket = i + 1;
            return h;
        }
    }
    if (outBucket) *outBucket = -1;
    return nullptr;
}

bool RDPatchGateQueue::removeEntry(MCGAL::Halfedge* h) {
    if (h == nullptr) return false;
    for (auto& b : buckets_) {
        for (auto it = b.begin(); it != b.end(); ++it) {
            if (*it == h) {
                b.erase(it);
                return true;
            }
        }
    }
    return false;
}

std::size_t RDPatchGateQueue::size() const {
    std::size_t total = 0;
    for (const auto& b : buckets_) total += b.size();
    return total;
}

bool RDPatchGateQueue::empty() const {
    for (const auto& b : buckets_) {
        if (!b.empty()) return false;
    }
    return true;
}

void RDPatchGateQueue::clear() {
    for (auto& b : buckets_) b.clear();
}

std::array<std::size_t, RDPatchGateQueue::kNumBuckets>
RDPatchGateQueue::bucketSizes() const {
    std::array<std::size_t, kNumBuckets> out{};
    for (int i = 0; i < kNumBuckets; ++i) out[i] = buckets_[i].size();
    return out;
}

}  // namespace pmsf
