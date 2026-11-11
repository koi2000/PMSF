// SPDX-License-Identifier: Apache-2.0
//
// RDPGS.h — RD PGS (Ahn et al., VisComput 27(6-8):769-779, 2011, §3.2)
// 的 env-gated 基础设施。task-19 P1：
//
//   1) **真 tag 演化**：当前忠实 rd_tag 路径的 vpTag 从未演化（6 级 priority 的
//      vpTag 属性恒 UNTAGGED、from_null 恒 false ⇒ RDErrorMetric 的 cost 恒 2.0，
//      排序实际 = 堆插入序）。本文件用 MCGAL::Vertex::TagFlag（⊕/⊖）做真 tag 演化
//      （复用 impl/common/PCLTTMTagTable.h 的 applyTagEvolution，与 PCLTTM/PCATM
//      ../../common/strict 同一份规则）。
//   2) **conquered-region 标记**：论文 priority 需要「2-3 条 incident edges 在
//      已征服区域」。当前路径没人 setProcessed；这里在每次消除后用
//      Vertex/Facet 的 isConquered/setConquered（MCGAL 自带 flag，默认路径无人读）
//      标记 patch 环顶点与其 incident faces。
//   3) **6 级 priority**：本路径实测 null_patch=0（可消除性链在入队前就过滤掉了
//      paper 的 null patch 候选，见 PMSF_RD_PATCH_SPIKE），故 from_null 恒 false，
//      论文 1/3/5 级（6/4/2）为可达级，2/4/6 级不可达（P2 gate-queue 后再补）。
//   4) **消除序号**：给每个 patch face 记录全局消除序号，供 P2「按 selection 顺序
//      发符号、去掉 isPoly locator」的字节投影诊断使用。
//
// 全部 env-gated（PMSF_RD_PGS=1）：不开时 RDAlgorithm 不安装 PGS metric / eliminate
// ⇒ 默认 RD/PCLTTM 产物逐字节不变。
#ifndef PMSF_IMPL_RD_RDPGS_H
#define PMSF_IMPL_RD_RDPGS_H

#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Point.h"
#include "Vertex.h"
#include "common/BoundedRingTraversal.h"
#include "common/PCLTTMTagTable.h"
#include "metric/ErrorMetric.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unordered_map>
#include <vector>

namespace rd_impl {

inline bool rdPgsEnvOn(const char* key) {
    const char* v = std::getenv(key);
    return v != nullptr && *v != '\0' && std::strcmp(v, "0") != 0;
}

/// P1 主开关。诊断子开关：
///   PMSF_RD_PGS_DIAG=1     退出时打印 priority 直方图
inline bool pgsEnabled() {
    static const bool on = rdPgsEnvOn("PMSF_RD_PGS");
    return on;
}

inline bool pgsDiagEnabled() {
    static const bool on = pgsEnabled() && rdPgsEnvOn("PMSF_RD_PGS_DIAG");
    return on;
}

struct RDPGSState {
    std::unordered_map<const MCGAL::Facet*, std::int32_t> elimSeq;
    /// 以「被消顶点量化整数坐标」为 key 的消除序号（P2 字节投影用）：
    /// patch face 的 Facet* 在同一轮后续消除中会被合并/重建，指针不可靠；
    /// 而 `Facet::getRemovedVertexPos()`（RD 路径 = 原始顶点位置）可稳量化回同一 key。
    std::unordered_map<std::uint64_t, std::int32_t> elimSeqByPos;
    std::int32_t nextSeq = 0;
    long long priorityHist[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    long long costCalls = 0;
};

inline RDPGSState& pgsState() {
    static RDPGSState s;
    return s;
}

inline pmsf::pclttm::Tag toCommonTag(MCGAL::Vertex::TagFlag t) {
    switch (t) {
        case MCGAL::Vertex::Plus:  return pmsf::pclttm::Tag::Plus;
        case MCGAL::Vertex::Minus: return pmsf::pclttm::Tag::Minus;
        default:                   return pmsf::pclttm::Tag::Untagged;
    }
}

inline MCGAL::Vertex::TagFlag toMcgalTag(pmsf::pclttm::Tag t) {
    switch (t) {
        case pmsf::pclttm::Tag::Plus:  return MCGAL::Vertex::Plus;
        case pmsf::pclttm::Tag::Minus: return MCGAL::Vertex::Minus;
        default:                       return MCGAL::Vertex::Untagged;
    }
}

/// 「该顶点的 incident edges 中有多少条位于已征服区域」。
/// 论文 Eq./§3.2 的 "2 or 3 incident edges in the conquered region"。
/// 以 incident face 的 isConquered 作为 conquered-region 代理（Facet flag，
/// 由本文件的 eliminate 钩子维护；默认路径无人读写该 flag）。
inline int conqueredIncidentEdges(MCGAL::Vertex* v) {
    if (v == nullptr || v->isRemoved()) return 0;
    int c = 0;
    for (MCGAL::Halfedge* hit : pmsf::boundedRing(v)) {
        if (hit == nullptr || hit->isRemoved()) continue;
        MCGAL::Facet* f = hit->face();
        if (f != nullptr && f->isConquered()) ++c;
    }
    return c;
}

inline bool lowConqueredValence(MCGAL::Vertex* v) {
    const int c = conqueredIncidentEdges(v);
    return c == 2 || c == 3;
}

/// 论文 §3.2 的 6 级 priority（1 = 最低，6 = 最高）。
/// 本路径 null_patch 实测为 0 ⇒ from_null 恒 false，可达级别 = 6 / 4 / 2。
inline int pgsPriority(MCGAL::Halfedge* he) {
    if (he == nullptr || he->isRemoved()) return 0;
    MCGAL::Vertex* back = he->vertex();
    MCGAL::Vertex* front = he->end_vertex();
    if (back == nullptr || front == nullptr || front->isRemoved()) return 0;

    const MCGAL::Vertex::TagFlag tb = back->getTag();
    const MCGAL::Vertex::TagFlag tf = front->getTag();
    const bool plus = MCGAL::Vertex::Plus;
    (void)plus;
    const bool backPlus = (tb == MCGAL::Vertex::Plus);
    const bool backMinus = (tb == MCGAL::Vertex::Minus);
    const bool frontPlus = (tf == MCGAL::Vertex::Plus);
    const bool frontMinus = (tf == MCGAL::Vertex::Minus);
    const bool oppositeTags = (backPlus && frontMinus) || (backMinus && frontPlus);
    const bool sameTags =
        ((backMinus && frontMinus) || (backPlus && frontPlus));

    // 论文 1 级：⊕⊖ 边（非 null patch），其 ⊖ 顶点在 conquered region 有 2-3 条边。
    if (oppositeTags) {
        MCGAL::Vertex* minusV = backMinus ? back : front;
        if (lowConqueredValence(minusV)) return 6;
        return 2;
    }
    // 论文 3 级：⊖⊖/⊕⊕ 边，两端都在 conquered region 有 2-3 条边。
    if (sameTags) {
        if (lowConqueredValence(back) && lowConqueredValence(front)) return 4;
        return 2;
    }
    // 论文 5 级：其余。
    return 2;
}

/// 交给 min-heap 的 key：越小越先 pop。priority 6 → -6 最先。
inline float pgsHeapError(MCGAL::Halfedge* he) {
    const int p = pgsPriority(he);
    if (pgsDiagEnabled()) {
        auto& s = pgsState();
        ++s.costCalls;
        if (p >= 0 && p < 8) ++s.priorityHist[p];
    }
    // p==0（invalid/removed）→ 排到最后。
    return (p <= 0) ? 1000.0f : -static_cast<float>(p);
}

/// RD PGS 专用 ErrorMetric（P1）。
class RDPGSMetric final : public pmsf::ErrorMetric {
  public:
    double cost(MCGAL::Halfedge* he) const override {
        return static_cast<double>(pgsHeapError(he));
    }
};

/// 量化整数坐标打包成 64-bit key（12-bit/轴，与 q12 一致；更低位按需扩展）。
inline std::uint64_t pgsPosKey(const MCGAL::PointInt& p) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(p.x()) & 0x1fffffu) << 42) |
           (static_cast<std::uint64_t>(static_cast<std::uint32_t>(p.y()) & 0x1fffffu) << 21) |
           (static_cast<std::uint64_t>(static_cast<std::uint32_t>(p.z()) & 0x1fffffu));
}

/// 消除后置处理：tag 演化 + conquered 标记 + 消除序号。
///   ring        = 消除前捕获的 patch 环（顺序与 boundedRing 一致）
///   patchFace   = 消除后的 patch face（entry.edge->face()）
///   removedKey  = 被消顶点量化位置 key（pgsPosKey）
inline void pgsPostRemove(const std::vector<MCGAL::Vertex*>& ring,
                          MCGAL::Facet* patchFace,
                          std::uint64_t removedKey) {
    if (!pgsEnabled()) return;
    auto& s = pgsState();
    const std::int32_t seq = s.nextSeq++;
    if (patchFace != nullptr) {
        s.elimSeq[patchFace] = seq;
    }
    s.elimSeqByPos[removedKey] = seq;
    // 注：Facet::isConquered() 是派生量（splittableFlag_ ∈ {Splittable,
    // Unsplittable, Removed}），patch face 在 eliminate 的 triangulate 后自动为
    // Splittable ⇒ 无需也无法在算法侧 set。conqueredIncidentEdges() 直接用它。
    // conquered 标记：patch 环顶点 + 其在 conquered 侧的面。
    for (MCGAL::Vertex* v : ring) {
        if (v == nullptr || v->isRemoved()) continue;
        v->setConquered();
    }
    // tag 演化（与 PCLTTM/PCATM strict 同一份 impl/common/PCLTTMTagTable.h 规则）。
    if (ring.size() >= 3) {
        const int N = static_cast<int>(ring.size());
        const auto tagLeft = toCommonTag(ring[0]->getTag());
        const auto tagRight = toCommonTag(ring[N - 1]->getTag());
        std::vector<pmsf::pclttm::TagEvolutionRecord> scratch;
        pmsf::pclttm::applyTagEvolution<MCGAL::Vertex>(
            ring, tagLeft, tagRight, pmsf::pclttm::AnchorKind::V0First,
            /*getTagFn=*/[](MCGAL::Vertex* v) { return toCommonTag(v->getTag()); },
            /*setTagFn=*/[](MCGAL::Vertex* v, pmsf::pclttm::Tag t) {
                v->setTag(toMcgalTag(t));
            },
            /*poolIdFn=*/[](MCGAL::Vertex* v) { return v ? v->poolId() : -1; },
            scratch);
    }
}

inline void pgsPrintDiag() {
    if (!pgsDiagEnabled()) return;
    auto& s = pgsState();
    std::fprintf(stderr,
                 "[RD_PGS] costCalls=%lld elimSeq=%d priority_hist: p6=%lld "
                 "p4=%lld p2=%lld p0=%lld\n",
                 s.costCalls, s.nextSeq, s.priorityHist[6], s.priorityHist[4],
                 s.priorityHist[2], s.priorityHist[0]);
}

inline void pgsRegisterDiag() {
    static const bool once = []() {
        std::atexit(pgsPrintDiag);
        return true;
    }();
    (void)once;
}

}  // namespace rd_impl

#endif  // PMSF_IMPL_RD_RDPGS_H
