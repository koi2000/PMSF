// SPDX-License-Identifier: Apache-2.0
//
// PCLTTMTagTable.h — Single source of truth for PCLTTM Tag (BlobLayout=16, sv=1)
// vertex tag evolution rules (path-B equivalent regularization, see
// `docs/paper_alignment/PCLTTM_OFFICIAL_REPRO_PROMPT.md` §R4 / R5).
//
// 路径 B 简化版：v0-fan 几何下的 vertex-tag 演化规则。它**不严格等于**
// Alliez & Desbrun 2001 §3.5 Fig.9 的 64 项 tag table —— 论文表是 tag-table-
// driven 真 retriangulation 几何下的 1-ring 顶点 tag 分配；本表是 v0-fan
// 几何下的等价正则化方案，目的是为 encoder/decoder 提供 single-source-of-
// truth 的对称演化规则，使 vertexTag 流的熵保持低于 isPoly+edgeSym 联合。
//
// encoder PCLTTMTagEliminateOperator (PR-2) 与 decoder
// PCLTTMTagReconstructOperator (PR-2) 共用本文件中的 evolve() 函数，
// 单元测试 `tests/test_pclttm_tag_table.cpp` (PR-1) 加载
// `docs/paper_alignment/pclttm_tag_table_groundtruth.csv` 对每行 assert
// evolve() 与 ground truth 一致。
//
// TODO(paper §3.5, docs/pdfs/PCLTTM.md §4.4): 默认路径仍是 v0-fan 对称实现，
// 不是论文 Figure 9 的真 tag-table retriangulation。当前文件已经提供共享的
// tag-guided retriangulation descriptor；`PMSF_PCLTTM_USE_TAG_TABLE_TRIANGULATION=1`
// 下 encoder/decoder 可走非 fan added-edge patch 路径。degree 3..6 且两端 tag
// 已知时 descriptor 复用 `paperRetriangulateFig9`；真正完成 Figure 9 对齐仍需
// 用论文 64 项表/CSV ground truth 校准该共享实现。

#ifndef SRC_COMMON_PCLTTM_TAG_TABLE_H_
#define SRC_COMMON_PCLTTM_TAG_TABLE_H_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "common/PMSFRetriangulation.h"

namespace pmsf {
namespace pclttm {

// 与 MCGAL::Vertex::TagFlag 一致；为了避免本头文件依赖 MCGAL，单独再声明。
enum class Tag : uint8_t {
    Untagged = 0,
    Plus     = 1,
    Minus    = 2,
};

// anchor_kind 是 path-B 的环切起点指示：v0_first=0 表示 1-ring 的 v0 在前驱
// gate 端；v0_last=1 表示 v0 在后继 gate 端。几何上等价（v0-fan 锚点固定 0），
// 但需要双方约定，否则 encoder/decoder 会在同一 vertex 上分配相反的 tag。
enum class AnchorKind : uint8_t {
    V0First = 0,
    V0Last  = 1,
};

// path-B simplified evolution rule with **unconditional anchoring**（Phase 2）：
//
//   原 (Phase 1) 规则：两端都 Untagged 时中间全 Untagged —— 这导致初始 mesh 所有
//   vertex 都 Untagged 时 tag 永远不播种，演化退化为无操作，无法承载任何
//   retriangulation 信号。
//
//   Phase 2 新规则：无论两端 tag 是什么，中间按 anchor-direction 交替 Plus/Minus。
//     - tag_seq[0]           = tagLeft  (若 Untagged → 默认 Plus，作为左端 anchor)
//     - tag_seq[degree - 1]  = tagRight (若 Untagged → 默认 Minus)
//     - for i in 1..degree-2:
//         anchorKind == V0First: tag_seq[i] = (i % 2 == 1) ? Plus : Minus
//         else (V0Last):         tag_seq[i] = (i % 2 == 1) ? Minus : Plus
//
// 这样 evolve 对 N 与 anchorKind 是纯确定性函数，不再依赖 tagLeft/tagRight 的
// Untagged 状态。Phase 2 的 tag-driven retriangulation 按此规则产生的 Plus/Minus
// 模式做对角线配对。
//
//   注意：原 CSV (`pclttm_tag_table_groundtruth.csv`) 的 "Untagged-Untagged →
//   全 Untagged" 行已不再是 ground truth——Phase 2 的验证走另一份
//   `pclttm_tag_table_phase2_groundtruth.csv`（同一 Table header，但新规则下的
//   tag_seq）。本 evolve 函数只保留一份新规则；旧 CSV 作为 historical reference
//   保留。
//
//   degree     ∈ {3, 4, 5, 6, ...}（调用方按 mod-pattern 推广 > 6，本函数无硬限）。
//   tagLeft    = 前驱 gate 端 vertex 的当前 tag（Phase 2: Untagged 默认替换为 Plus）。
//   tagRight   = 后继 gate 端 vertex 的当前 tag（Phase 2: Untagged 默认替换为 Minus）。
//   anchorKind = 环切起点（path-B 默认 V0First）。
inline std::vector<Tag> evolve(int degree, Tag tagLeft, Tag tagRight, AnchorKind anchorKind) {
    std::vector<Tag> seq(static_cast<size_t>(degree), Tag::Untagged);
    if (degree <= 0) return seq;

    // Phase 2: 若两端 Untagged，默认 Plus / Minus anchor（确保 tag 真的被播种）
    Tag effL = (tagLeft  == Tag::Untagged) ? Tag::Plus  : tagLeft;
    Tag effR = (tagRight == Tag::Untagged) ? Tag::Minus : tagRight;

    seq[0] = effL;
    if (degree == 1) return seq;
    seq[static_cast<size_t>(degree - 1)] = effR;
    if (degree <= 2) return seq;
    for (int i = 1; i < degree - 1; ++i) {
        if (anchorKind == AnchorKind::V0First) {
            seq[static_cast<size_t>(i)] = (i % 2 == 1) ? Tag::Plus : Tag::Minus;
        } else {
            seq[static_cast<size_t>(i)] = (i % 2 == 1) ? Tag::Minus : Tag::Plus;
        }
    }
    return seq;
}

struct TagTableDiagonal {
    int va = -1;
    int vb = -1;
};

struct TagTableTriangle {
    int a = -1;
    int b = -1;
    int c = -1;
};

struct TagTableRetriangulation {
    bool valid = false;
    std::vector<TagTableDiagonal> diagonals;
    std::vector<TagTableTriangle> triangles;
};

// Tag-guided polygon retriangulation descriptor for the paper path.
//
// The original paper constrains degree 3..6 patches through Figure 9's
// plus/minus tag table. This helper keeps that decision in the shared
// PCLTTM layer instead of hiding it inside one storage or algorithm backend:
// callers get an ordered list of diagonals over the normalized polygon ring.
//
// It is intentionally side-effect free. Encoder and decoder can both use the
// same descriptor before calling their mesh mutation primitives, which is the
// invariant the paper relies on: tags determine geometry; the bitstream does
// not carry diagonals.
inline TagTableRetriangulation computeTagGuidedRetriangulation(
    const std::vector<Tag>& tagSeq) {

    TagTableRetriangulation out;
    const int N = static_cast<int>(tagSeq.size());
    if (N < 3) {
        return out;
    }
    if (N == 3) {
        out.valid = true;
        out.triangles.push_back({0, 1, 2});
        return out;
    }

    auto toPaperTag = [](Tag tag) -> int {
        if (tag == Tag::Plus) return 1;
        if (tag == Tag::Minus) return 2;
        return 0;
    };
    const int leftTag = toPaperTag(tagSeq.front());
    const int rightTag = toPaperTag(tagSeq.back());
    if (N <= 6 && leftTag != 0 && rightTag != 0) {
        const pmsf::RetriangulationResult paper =
            pmsf::paperRetriangulateFig9(N, leftTag, rightTag);
        if (static_cast<int>(paper.triangles.size()) == N - 2) {
            for (const auto& tri : paper.triangles) {
                out.triangles.push_back({tri[0], tri[1], tri[2]});
            }
            for (const auto& diag : paper.diagonals) {
                out.diagonals.push_back({diag[0], diag[1]});
            }
            out.valid = true;
            return out;
        }
    }

    std::vector<int> ring;
    ring.reserve(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i) ring.push_back(i);

    bool preferPlus = true;
    while (ring.size() > 3) {
        const Tag desired = preferPlus ? Tag::Plus : Tag::Minus;
        int pos = -1;
        if (preferPlus) {
            for (std::size_t i = 0; i < ring.size(); ++i) {
                if (tagSeq[static_cast<std::size_t>(ring[i])] == desired) {
                    pos = static_cast<int>(i);
                    break;
                }
            }
        } else {
            for (int i = static_cast<int>(ring.size()) - 1; i >= 0; --i) {
                if (tagSeq[static_cast<std::size_t>(ring[static_cast<std::size_t>(i)])] == desired) {
                    pos = i;
                    break;
                }
            }
        }
        if (pos < 0) {
            pos = preferPlus ? 0 : static_cast<int>(ring.size()) - 1;
        }

        const int n = static_cast<int>(ring.size());
        const int prev = ring[static_cast<std::size_t>((pos + n - 1) % n)];
        const int curr = ring[static_cast<std::size_t>(pos)];
        const int next = ring[static_cast<std::size_t>((pos + 1) % n)];
        out.triangles.push_back({prev, curr, next});
        out.diagonals.push_back({prev, next});
        ring.erase(ring.begin() + pos);
        preferPlus = !preferPlus;
    }

    out.triangles.push_back({ring[0], ring[1], ring[2]});
    out.valid = static_cast<int>(out.triangles.size()) == N - 2 &&
                static_cast<int>(out.diagonals.size()) == N - 3;
    return out;
}

// 解析 CSV 中 tag_seq 字段（如 "P;P;M"）→ vector<Tag>。供单元测试使用。
inline std::vector<Tag> parseTagSeq(const std::string& s) {
    std::vector<Tag> out;
    Tag cur = Tag::Untagged;
    bool started = false;
    auto pushIf = [&](bool seen) {
        if (seen) {
            out.push_back(cur);
            started = false;
            cur = Tag::Untagged;
        }
    };
    for (char c : s) {
        if (c == ';' || c == ' ' || c == '\t' || c == '\r' || c == '\n') {
            pushIf(started);
        } else {
            switch (c) {
                case 'U': cur = Tag::Untagged; started = true; break;
                case 'P': cur = Tag::Plus;     started = true; break;
                case 'M': cur = Tag::Minus;    started = true; break;
                default: /* skip unknown chars */ break;
            }
        }
    }
    pushIf(started);
    return out;
}

inline Tag parseTag(const std::string& s) {
    if (s == "Untagged" || s == "U" || s == "0") return Tag::Untagged;
    if (s == "Plus"     || s == "P" || s == "1") return Tag::Plus;
    if (s == "Minus"    || s == "M" || s == "2") return Tag::Minus;
    return Tag::Untagged;
}

inline AnchorKind parseAnchorKind(const std::string& s) {
    if (s == "1" || s == "V0Last" || s == "v0_last") return AnchorKind::V0Last;
    return AnchorKind::V0First;
}

inline const char* tagToCStr(Tag t) {
    switch (t) {
        case Tag::Plus:  return "P";
        case Tag::Minus: return "M";
        default:         return "U";
    }
}

// =====================================================================
// 共享的 polyFace tag-演化 helper（encoder 与 decoder 双端共用）
// =====================================================================
//
// 每条记录代表一个被演化的 ring vertex：
//   poolId    = vertex pool id
//   tagBefore = 演化前 tag (uint8_t cast 自 enum)
//   tagAfter  = 演化后 tag (uint8_t cast 自 enum)
struct TagEvolutionRecord {
    int     poolId;
    uint8_t tagBefore;
    uint8_t tagAfter;
};

// 把 polyVerts 的 tag 演化抽象为可单元测试的 free helper。模板化让
// 调用方可以传任意 vertex 类型（实践中传 MCGAL::Vertex*，单元测试可
// 传 mock vertex 类型）。
//
// 入参：
//   polyVerts        : 长度 N 的 ring vertex 指针序列（左端 polyVerts[0]，
//                      右端 polyVerts[N-1]，与 encoder/decoder 双端取序一致）。
//   tagLeftBefore    : polyVerts[0] 当前 tag（来自外部读取）。
//   tagRightBefore   : polyVerts[N-1] 当前 tag。
//   anchor           : 环切起点（path-B 默认 V0First）。
//   getTagFn         : (V*) -> Tag         读取一个 vertex 的当前 tag。
//   setTagFn         : (V*, Tag) -> void   写入新 tag。
//   poolIdFn         : (V*) -> int         拿 vertex 的 pool id。
//
// 输出：
//   追加到 traceOut（不清空）的 N 条 TagEvolutionRecord。
template <typename V, typename GetTagFn, typename SetTagFn, typename PoolIdFn>
inline void applyTagEvolution(const std::vector<V*>& polyVerts,
                              Tag tagLeftBefore,
                              Tag tagRightBefore,
                              AnchorKind anchor,
                              GetTagFn getTagFn,
                              SetTagFn setTagFn,
                              PoolIdFn poolIdFn,
                              std::vector<TagEvolutionRecord>& traceOut) {
    const int N = static_cast<int>(polyVerts.size());
    if (N < 3) return;
    const auto tagSeq = evolve(N, tagLeftBefore, tagRightBefore, anchor);
    if (static_cast<int>(tagSeq.size()) != N) return;
    for (int i = 0; i < N; ++i) {
        V* v = polyVerts[static_cast<std::size_t>(i)];
        if (!v) continue;
        TagEvolutionRecord rec;
        rec.poolId    = poolIdFn(v);
        rec.tagBefore = static_cast<uint8_t>(getTagFn(v));
        setTagFn(v, tagSeq[static_cast<std::size_t>(i)]);
        rec.tagAfter  = static_cast<uint8_t>(getTagFn(v));
        traceOut.push_back(rec);
    }
}

}  // namespace pclttm
}  // namespace pmsf

#endif  // SRC_COMMON_PCLTTM_TAG_TABLE_H_
