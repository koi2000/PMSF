/*****************************************************************************
 * @file impl/common/SegmentedRangeCodedDecode.cpp
 * @brief 分区（segmented）产物解码镜像的实现 —— 见头文件的说明。
 *
 * 本文件的 `importBlob` / `decodeOneRound` 两块正文是从
 * `impl/common/ColumnarDeserializeOperator.cpp` 的
 * `paperBlobLayoutIsSegmentedRangeCodedPred` 分支**原样搬出**的（含全部
 * env 门控诊断，默认关闭），只做了机械改名：
 *     rcBlob_ / blobLayout_ 相关 → 入参 `blob` / `size`
 *     segmentedOptimized{GroupNumber,GroupStart,Readers}_ → {groupNumber_,groupStart_,readers_}
 *     segmentedSeedVertices_ → seedVertices_      parquetV2VerboseLogs() → segmentedVerboseLogs()
 *     bboxMin{X,Y,Z}_ → bboxMin_.{x,y,z}()        reconstructOperator_ → reconstructor_
 * 搬出的意义是让**二进制后端**共用同一份镜像（此前它按普通 range-coded blob
 * 整体导入 ⇒ 前缀被当符号消费 ⇒ 断言失败）。
 */
#include "SegmentedRangeCodedDecode.h"
#include "common/CanonicalTieBreakKey.h"

#include "common/BoundedRingTraversal.h"
#include "common/BfsGroupLabel.h"
#include "storage/SegmentedGroupChannel.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <unordered_map>
#include <iostream>
#include <sstream>
#include <string>

namespace pmsf {

namespace {

bool segmentedVerboseLogs() {
    static const bool enabled = [] {
        const char* env = std::getenv("PMSF_V2_VERBOSE");
        return env && env[0] != '\0' && std::strcmp(env, "0") != 0 &&
               std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
    }();
    return enabled;
}

bool segmentedReverseGroupReconstruct() {
    static const bool enabled = [] {
        const char* env = std::getenv("PMSF_SEGMENTED_RECONSTRUCT_REVERSE_GROUPS");
        return env && env[0] != '\0' && std::strcmp(env, "0") != 0 &&
               std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
    }();
    return enabled;
}

bool segmentedBatchGroupReconstruct() {
    static const bool enabled = [] {
        const char* env = std::getenv("PMSF_SEGMENTED_RECONSTRUCT_BATCH_GROUPS");
        return env && env[0] != '\0' && std::strcmp(env, "0") != 0 &&
               std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
    }();
    return enabled;
}

}  // namespace

// ---------------------------------------------------------------------------
// 半边定位三件套 + seed 刷新（从 ColumnarDeserializeOperator 的同名 lambda 搬出，
// 该处现在委托到这里，保证「分区重算」与「非分区路径」用的是同一份规则）。
// ---------------------------------------------------------------------------
MCGAL::Halfedge* findLiveHalfedgeInMesh(MCGAL::Mesh* mesh,
                                        MCGAL::Vertex* from,
                                        MCGAL::Vertex* to,
                                        int groupId) {
    if (!mesh || !from) return nullptr;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        if (groupId >= 0 && f->groupId() != groupId) continue;
        MCGAL::Halfedge* h = f->proxyHalfedge();
        MCGAL::Halfedge* start = h;
        int guard = 0;
        do {
            if (h && !h->isRemoved() && h->vertex() == from &&
                (to == nullptr || h->end_vertex() == to)) {
                return h;
            }
            h = h ? h->next() : nullptr;
        } while (h && h != start && ++guard <= 8192);
    }
    return nullptr;
}

MCGAL::Halfedge* findAnyLiveHalfedgeInMesh(MCGAL::Mesh* mesh, int groupId) {
    if (!mesh) return nullptr;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        if (groupId >= 0 && f->groupId() != groupId) continue;
        MCGAL::Halfedge* h = f->proxyHalfedge();
        MCGAL::Halfedge* start = h;
        int guard = 0;
        do {
            if (h && !h->isRemoved()) return h;
            h = h ? h->next() : nullptr;
        } while (h && h != start && ++guard <= 8192);
    }
    return nullptr;
}

MCGAL::Halfedge* findCanonicalGroupHalfedgeInMesh(MCGAL::Mesh* mesh, int groupId) {
    if (!mesh || groupId < 0) return nullptr;
    MCGAL::Halfedge* best = nullptr;
    std::array<double, 6> bestKey{};
    auto qkey = [&](MCGAL::Halfedge* h) -> std::array<double, 6> {
        if (mesh->enableQuantization) {
            MCGAL::PointInt a = mesh->floatPosToInt(h->vertex()->point());
            MCGAL::PointInt b = mesh->floatPosToInt(h->end_vertex()->point());
            return {static_cast<double>(a.x()), static_cast<double>(a.y()),
                    static_cast<double>(a.z()), static_cast<double>(b.x()),
                    static_cast<double>(b.y()), static_cast<double>(b.z())};
        }
        const MCGAL::Point& a = h->vertex()->point();
        const MCGAL::Point& b = h->end_vertex()->point();
        return {a.x(), a.y(), a.z(), b.x(), b.y(), b.z()};
    };
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge() ||
            f->groupId() != groupId) {
            continue;
        }
        MCGAL::Halfedge* h = f->proxyHalfedge();
        MCGAL::Halfedge* start = h;
        int guard = 0;
        do {
            if (h && !h->isRemoved() && h->vertex() && h->end_vertex()) {
                const std::array<double, 6> key = qkey(h);
                if (!best || key < bestKey) {
                    best = h;
                    bestKey = key;
                }
            }
            h = h ? h->next() : nullptr;
        } while (h && h != start && ++guard <= 8192);
    }
    return best;
}

void refreshLiveSeedHalfedge(MCGAL::Mesh* mesh,
                             MCGAL::Vertex* v0,
                             MCGAL::Vertex* v1,
                             MCGAL::Halfedge*& outSeed,
                             int groupId,
                             bool preferCanonicalSeed) {
    if (!v0 || !v1) return;
    if (preferCanonicalSeed && groupId >= 0) {
        if (MCGAL::Halfedge* canonical = findCanonicalGroupHalfedgeInMesh(mesh, groupId)) {
            outSeed = canonical;
            return;
        }
    }
    // create_center_vertex_without_init does not maintain Vertex::halfedges().
    // Search the live face rings instead, which mirrors OpenMesh pushHehInit().
    if (MCGAL::Halfedge* fresh = findLiveHalfedgeInMesh(mesh, v0, v1, groupId)) {
        outSeed = fresh;
        return;
    }
    if (MCGAL::Halfedge* fallback = findLiveHalfedgeInMesh(mesh, v0, nullptr, groupId)) {
        outSeed = fallback;
        return;
    }
    if (MCGAL::Halfedge* fallback = findLiveHalfedgeInMesh(mesh, v1, nullptr, groupId)) {
        outSeed = fallback;
        return;
    }
    if (outSeed && !outSeed->isRemoved() && outSeed->face() &&
        !outSeed->face()->isRemoved() &&
        (groupId < 0 || outSeed->face()->groupId() == groupId)) {
        return;
    }
    if (MCGAL::Halfedge* fallback = findAnyLiveHalfedgeInMesh(mesh, groupId)) {
        outSeed = fallback;
    }
}

// ---------------------------------------------------------------------------
// 类成员
// ---------------------------------------------------------------------------
MCGAL::Halfedge* SegmentedRangeCodedDecode::findLiveHalfedge(MCGAL::Vertex* from,
                                                             MCGAL::Vertex* to,
                                                             int groupId) const {
    return findLiveHalfedgeInMesh(mesh_.get(), from, to, groupId);
}

MCGAL::Halfedge* SegmentedRangeCodedDecode::findAnyLiveHalfedge(int groupId) const {
    return findAnyLiveHalfedgeInMesh(mesh_.get(), groupId);
}

MCGAL::Halfedge* SegmentedRangeCodedDecode::findCanonicalGroupHalfedge(int groupId) const {
    return findCanonicalGroupHalfedgeInMesh(mesh_.get(), groupId);
}

void SegmentedRangeCodedDecode::refreshSeed(MCGAL::Vertex* v0,
                                            MCGAL::Vertex* v1,
                                            MCGAL::Halfedge*& outSeed,
                                            int groupId) {
    const char* canonicalSeedEnv = std::getenv("PMSF_SEGMENTED_DEFAULT_CANONICAL_SEED");
    const bool canonicalSeed =
        canonicalSeedEnv && canonicalSeedEnv[0] != '\0' && canonicalSeedEnv[0] != '0';
    refreshLiveSeedHalfedge(mesh_.get(), v0, v1, outSeed, groupId, canonicalSeed);
}

bool segmentedRangeCodedBlobMatches(const char* blob,
                                    std::size_t size,
                                    int liveVertexCount,
                                    int liveFacetCount) {
    if (blob == nullptr || size < 4 + 2 * sizeof(int)) {
        return false;
    }
    auto i32 = [&](std::size_t at, int& out) -> bool {
        if (at + sizeof(int) > size) {
            return false;
        }
        std::memcpy(&out, blob + at, sizeof(int));
        return true;
    };

    int groupCount = 0;
    if (!i32(0, groupCount) || groupCount < 2 || groupCount > 65536) {
        return false;
    }
    const std::size_t metaBytes =
        sizeof(int) + static_cast<std::size_t>(groupCount) * 2 * sizeof(int) +
        static_cast<std::size_t>(groupCount + 1) * sizeof(int);
    if (metaBytes + 2 * sizeof(int) > size) {
        return false;
    }
    // 种子通道（编码端的**基网 vid**，-1 = 无种子）
    for (int g = 0; g < groupCount; ++g) {
        int a = 0;
        int b = 0;
        if (!i32(sizeof(int) + static_cast<std::size_t>(g) * 2 * sizeof(int), a) ||
            !i32(sizeof(int) + static_cast<std::size_t>(g) * 2 * sizeof(int) + sizeof(int), b)) {
            return false;
        }
        if (a < -1 || a >= liveVertexCount || b < -1 || b >= liveVertexCount) {
            return false;
        }
    }
    // 偏移表：首项 0、单调不减、末项 = 负载字节数
    int payloadBytes = 0;
    for (int g = 0; g <= groupCount; ++g) {
        int v = 0;
        if (!i32(sizeof(int) + static_cast<std::size_t>(groupCount) * 2 * sizeof(int) +
                     static_cast<std::size_t>(g) * sizeof(int),
                 v) ||
            v < 0 || (g == 0 && v != 0) || (g > 0 && v < payloadBytes)) {
            return false;
        }
        payloadBytes = v;
    }
    // 基网 facet → 分区通道：使用共享健壮解析器（支持 fixed/varint 游程）。
    pmsf::ParsedSegmentedGroupChannel parsedChannel;
    std::size_t channelOffset = metaBytes;
    const bool channelOk = pmsf::parseSegmentedGroupChannel(
        blob, size, channelOffset, groupCount, liveFacetCount, &parsedChannel);
    const int baseFacetCount = parsedChannel.baseFacetCount;
    const int runCount = static_cast<int>(parsedChannel.runCount);
    const long long runSum = baseFacetCount;
    const std::size_t at = channelOffset;
    const bool ok = channelOk &&
                    static_cast<std::size_t>(static_cast<long long>(at) + payloadBytes) == size;
    if (ok && segmentedVerboseLogs()) {
        std::fprintf(stderr,
                     "[SEG_BINARY_PROBE] G=%d runs=%d baseFacetCount=%d(live=%d) "
                     "runSum=%lld payload=%d prefix+channel=%zu blob=%zu liveVerts=%d ok=%d\n",
                     groupCount, runCount, baseFacetCount, liveFacetCount, runSum, payloadBytes,
                     at, size, liveVertexCount, static_cast<int>(ok));
    }
    return ok;
}

bool SegmentedRangeCodedDecode::importBlob(char* blob, std::size_t size, const Context& ctx) {
    header_ = ctx.header;
    mesh_ = ctx.mesh;
    reconstructor_ = ctx.reconstructor;
    codecVersion_ = ctx.codecVersion;
    bboxMin_ = ctx.bboxMin;
    quantStep_ = ctx.quantStep;
    // 分区选择谓词由调用方每轮下发（`setGroupSelection`），此处不重置 ——
    // 它们可能在基网解码之后才被设置。
    currentRound_ = 0;
    initialized_ = false;
    groupNumber_ = 0;
    groupStart_.clear();
    readers_.clear();
    seedVertices_.clear();
    seeds_.clear();

    if (!mesh_) {
        std::cerr << "[SegmentedRangeCodedDecode] mesh is null" << std::endl;
        return false;
    }

            // P18.2 PHASE59 物品 3：**分区产物的解码镜像**。
            //
            // 布局（唯一写出方 SegmentedOptimizedSymbolCollectOperator::
            // exportToBufferWithRangeCoder，字节序与它逐字段对称）：
            //   [groupCount][seedA[G]][seedB[G]][groupStart[G+1]]
            //   [baseFacetCount][rleRunCount][(gid:int8)(runLen:int32) × runs]
            //   [payload]
            // 后两段（基网 facet → 分区）只在 groupCount > 1 时存在。
            //
            // seedA/seedB 是**编码端全网格**的顶点 poolId，解码端 round 1 手上只有
            // 基网（poolId 跨网格不对应）⇒ 本路径**不消费**它们，只按布局读掉；
            // 每组 seed 仍由 base_mesh 表给出（`seedVertices_`，与分段
            // PPMC 路径同源）。
            size_t off = 0;
            auto readI32 = [&](int& out) -> bool {
                if (off + sizeof(int) > size)
                    return false;
                std::memcpy(&out, blob + off, sizeof(int));
                off += sizeof(int);
                return true;
            };
            // 诊断开关（layout-32 专用）：把 blob 头几个 int 打出来。
            // 这一段的病征全是「前缀读错位」形态（混合 blob、偏移表截断），
            // 而 `[groupCount]`、`groupStart[]`、`[baseFacetCount][runs]` 一眼可辨，
            // 比逐层加打印快得多 —— 实测 110261→首 int 序列 `2 … 0 104286 109336
            // 11872 177` 就是这么确认前缀对齐的。

            if (!readI32(groupNumber_) || groupNumber_ < 0) {
                std::cerr << "[Columnar] segmented optimized blob header invalid" << std::endl;
                return false;
            }

            // 种子通道：编码为**基网顶点序号**（见
            // `SegmentedOptimizedSymbolCollectOperator::exportToBufferWithRangeCoder`
            // 的布局注释），此处按布局读入，稍后解析成种子半边。
            std::vector<std::pair<int, int>> seedOrdinals;
            for (int group = 0; group < groupNumber_; ++group) {
                int a = 0;
                int b = 0;
                if (!readI32(a) || !readI32(b)) {
                    std::cerr << "[Columnar] segmented optimized seed table truncated" << std::endl;
                    return false;
                }
                seedOrdinals.emplace_back(a, b);
            }
            // [PHASE59 诊断，默认关闭] 把传下来的序号当 poolId 去查，
            // 用于证伪「序号 == poolId」。

            groupStart_.assign(static_cast<std::size_t>(groupNumber_ + 1), 0);
            for (int group = 0; group <= groupNumber_; ++group) {
                int v = 0;
                if (!readI32(v)) {
                    std::cerr << "[Columnar] segmented optimized offset table truncated" << std::endl;
                    return false;
                }
                groupStart_[static_cast<std::size_t>(group)] = static_cast<size_t>(std::max(0, v));
            }

            // 基网 facet → 分区通道：按**基网 facet 的建立序**贴回 `groupId`。
            // 消除产生的面从父面**继承** groupId（见 VertexRemovalEliminateOperator
            // ::triangulate），故只要这个起点一致，解码端重建出的分区归属就与
            // 编码端一致 —— 这是附 16「两侧各自重算成环」的破环点。
            if (groupNumber_ > 1) {
                int liveFacetCount = 0;
                for (MCGAL::Facet* f : mesh_->faces()) {
                    if (f && !f->isRemoved()) ++liveFacetCount;
                }
                std::size_t channelOffset = off;
                pmsf::ParsedSegmentedGroupChannel parsedChannel;
                if (!pmsf::parseSegmentedGroupChannel(blob, size, channelOffset,
                                                      groupNumber_, liveFacetCount,
                                                      &parsedChannel)) {
                    std::cerr << "[Columnar] segmented optimized base-facet group channel "
                                 "parse failed liveF=" << liveFacetCount << std::endl;
                    return false;
                }
                off = channelOffset;
                std::vector<int> facetGroups;
                facetGroups.reserve(static_cast<std::size_t>(parsedChannel.baseFacetCount));
                for (const auto& run : parsedChannel.runs) {
                    facetGroups.insert(facetGroups.end(),
                                       static_cast<std::size_t>(run.length),
                                       static_cast<int>(run.gid));
                }
                std::size_t applied = 0;
                std::size_t liveFacets = 0;
                const bool preassigned = std::getenv("PMSF_SEAM_BASE_GROUPS_PREASSIGNED") != nullptr;
                for (MCGAL::Facet* f : mesh_->faces()) {
                    if (!f || f->isRemoved())
                        continue;
                    ++liveFacets;
                    if (preassigned) {
                        ++applied;
                        continue;
                    }
                    if (applied >= facetGroups.size())
                        continue;
                    f->setGroupId(facetGroups[applied++]);
                }
                if (segmentedVerboseLogs() || applied != liveFacets) {
                    std::cerr << "[Columnar] base-facet group channel: runs=" << parsedChannel.runCount << " entries=" << facetGroups.size()
                              << " applied=" << applied << " live_facets=" << liveFacets << "\n";
                }
                // [PHASE59 诊断，默认关闭] dump 解码端**实际贴上**的分区序列。
            }

            // [BFS group-labelled stream, env gated] Recompute facet groups
            // from the seed channel via the same multi-source BFS used by the
            // encoder prototype (PMSF_SEG_BFS_LABELS).
            if (std::getenv("PMSF_SEG_BFS_DECODE") && mesh_ && groupNumber_ > 0 &&
                !seedOrdinals.empty()) {
                std::unordered_map<MCGAL::Facet*, int> label;
                std::deque<MCGAL::Facet*> dq;
                auto vertexByOrdinal = [&](int ord) -> MCGAL::Vertex* {
                    if (ord < 0) return nullptr;
                    if (ord < static_cast<int>(mesh_->vertices().size())) {
                        MCGAL::Vertex* v = mesh_->vertices()[static_cast<std::size_t>(ord)];
                        if (v && !v->isRemoved()) return v;
                    }
                    for (MCGAL::Vertex* v : mesh_->vertices())
                        if (v && !v->isRemoved() && v->vid() == ord) return v;
                    return nullptr;
                };
                for (int g = 0; g < groupNumber_; ++g) {
                    MCGAL::Vertex* va = vertexByOrdinal(seedOrdinals[static_cast<std::size_t>(g)].first);
                    MCGAL::Vertex* vb = vertexByOrdinal(seedOrdinals[static_cast<std::size_t>(g)].second);
                    MCGAL::Halfedge* seedH = nullptr;
                    if (va) {
                        for (MCGAL::Halfedge* hh : va->halfedges()) {
                            if (hh && hh->end_vertex() == vb) { seedH = hh; break; }
                        }
                        if (!seedH && va->proxyHalfedge()) seedH = va->proxyHalfedge();
                    }
                    MCGAL::Facet* f = (seedH && seedH->face() && !seedH->face()->isRemoved())
                                          ? seedH->face()
                                          : nullptr;
                    if (!f || label.count(f)) continue;
                    label[f] = g;
                    dq.push_back(f);
                }
                while (!dq.empty()) {
                    MCGAL::Facet* f = dq.front(); dq.pop_front();
                    const int g = label[f];
                    MCGAL::Halfedge* h0 = f->proxyHalfedge();
                    MCGAL::Halfedge* h = h0;
                    int guard = 0;
                    do {
                        if (h && h->opposite() && h->opposite()->face()) {
                            MCGAL::Facet* nb = h->opposite()->face();
                            if (!nb->isRemoved() && !label.count(nb)) {
                                label[nb] = g;
                                dq.push_back(nb);
                            }
                        }
                        h = h ? h->next() : nullptr;
                    } while (h && h != h0 && ++guard < 64);
                }
                int applied = 0;
                for (MCGAL::Facet* f : mesh_->faces()) {
                    if (!f || f->isRemoved()) continue;
                    auto it = label.find(f);
                    if (it != label.end()) { f->setGroupId(it->second); ++applied; }
                }
            }

            if (!pmsf::bfsGroupAnchors().empty() &&
                std::getenv("PMSF_SEG_BFS_DECODE") != nullptr) {
                pmsf::bfsRelabelFacetGroups(mesh_, pmsf::bfsGroupAnchors());
            }

            // 每组 seed：**解码端重算**。
            //
            // 编码端的规则写在 `CompressPipeline.cpp::applySegmentGroupTable` 里：
            // 「每组 seed = 该组**第一个被遍历到的 facet** 的 `proxyHalfedge()`。
            //   用遍历序而不是「任取一个」，是为了让它**可重算**。」
            // 那句注释就是本段的许可证：两侧拿到同一份基网 facet 分区号序列后，
            // 按同一遍历序取每组首个面，各自算出的就是同一条 seed 半边。
            //
            // 为什么不消费 blob 的 seedA/seedB：它们是编码端**全网格**（消除之前）
            // 的顶点 poolId，而解码端此刻手上只有基网，且 `Mesh::garbage_collection`
            // 不重编号池 ⇒ 两侧 poolId 不对应（附 18 风险 1；P18.2 附 15 亦实测
            // `applied=0 missing=69664`）。布局里保留它们只是为了偏移表对齐。
            //
            // 不填这一段的后果（PHASE59 实测）：`seeds_` 只有 base_mesh 表给出的
            // 一条（表恒单行），round 循环的 `endGroup =
            // min(groupNumber, seeds_.size())` 把 group 1 整组跳过 ⇒ 只重建出
            // 约一半网格（23,307 顶点 / 28,939 面 vs 原网格 34,834 / 69,664）。
            if (mesh_ && groupNumber_ > 0) {
                const std::size_t groupCount = static_cast<std::size_t>(groupNumber_);
                seedVertices_.assign(groupCount, {nullptr, nullptr});
                seeds_.assign(groupCount, nullptr);
                std::vector<std::uint8_t> seeded(groupCount, 0);
                const bool useSeedChannelVids =
                    std::getenv("PMSF_SEG_SEED_CHANNEL") != nullptr && !seedOrdinals.empty();
                const bool useChannelSeeds =
                    !pmsf::bfsGroupAnchors().empty() &&
                    std::getenv("PMSF_SEG_BFS_DECODE") != nullptr &&
                    !seedOrdinals.empty();
                if (useSeedChannelVids) {
                    // seedX[g] are written as base-mesh vids; the decoder's base vertex
                    // insertion order matches that vid space, so indices resolve directly.
                    for (int g = 0; g < groupNumber_; ++g) {
                        const int a = seedOrdinals[static_cast<std::size_t>(g)].first;
                        const int b = seedOrdinals[static_cast<std::size_t>(g)].second;
                        if (a < 0 || b < 0 || a >= static_cast<int>(mesh_->vertices().size()) ||
                            b >= static_cast<int>(mesh_->vertices().size()))
                            continue;
                        MCGAL::Vertex* va = mesh_->vertices()[static_cast<std::size_t>(a)];
                        MCGAL::Vertex* vb = mesh_->vertices()[static_cast<std::size_t>(b)];
                        if (!va || !vb || va->isRemoved() || vb->isRemoved()) continue;
                        MCGAL::Halfedge* h = nullptr;
                        for (MCGAL::Halfedge* cand : va->halfedges()) {
                            if (cand && cand->end_vertex() == vb) { h = cand; break; }
                        }
                        if (!h) {
                            for (MCGAL::Halfedge* cand : va->halfedges()) {
                                if (cand && cand->vertex() == va) { h = cand; break; }
                            }
                        }
                        if (!h) continue;
                        seeds_[static_cast<std::size_t>(g)] = h;
                        seedVertices_[static_cast<std::size_t>(g)] = {va, vb};
                        seeded[static_cast<std::size_t>(g)] = 1;
                    }
                }
                if (useChannelSeeds) {
                    auto vertexByOrdinal = [&](int ord) -> MCGAL::Vertex* {
                        if (ord < 0) return nullptr;
                        if (ord < static_cast<int>(mesh_->vertices().size())) {
                            MCGAL::Vertex* v = mesh_->vertices()[static_cast<std::size_t>(ord)];
                            if (v && !v->isRemoved()) return v;
                        }
                        for (MCGAL::Vertex* v : mesh_->vertices())
                            if (v && !v->isRemoved() && v->vid() == ord) return v;
                        return nullptr;
                    };
                    for (int g = 0; g < groupNumber_; ++g) {
                        MCGAL::Vertex* va = vertexByOrdinal(seedOrdinals[static_cast<std::size_t>(g)].first);
                        MCGAL::Vertex* vb = vertexByOrdinal(seedOrdinals[static_cast<std::size_t>(g)].second);
                        if (!va || !vb) continue;
                        MCGAL::Halfedge* h = nullptr;
                        for (MCGAL::Halfedge* cand : va->halfedges()) {
                            if (cand && cand->end_vertex() == vb) { h = cand; break; }
                        }
                        if (!h) {
                            for (MCGAL::Halfedge* cand : vb->halfedges()) {
                                if (cand && cand->end_vertex() == va && cand->opposite()) {
                                    h = cand->opposite();
                                    break;
                                }
                            }
                        }
                        if (!h) continue;
                        seeds_[static_cast<std::size_t>(g)] = h;
                        seedVertices_[static_cast<std::size_t>(g)] = {va, vb};
                        seeded[static_cast<std::size_t>(g)] = 1;
                    }
                }
                if (!useChannelSeeds)
                for (MCGAL::Facet* f : mesh_->faces()) {
                    if (!f || f->isRemoved())
                        continue;
                    const int g = f->groupId();
                    if (g < 0 || g >= groupNumber_ || seeded[static_cast<std::size_t>(g)]) {
                        continue;
                    }
                    MCGAL::Halfedge* h = f->proxyHalfedge();
                    if (!h || !h->vertex() || !h->end_vertex())
                        continue;
                    seeded[static_cast<std::size_t>(g)] = 1;
                    seedVertices_[static_cast<std::size_t>(g)] = {h->vertex(), h->end_vertex()};
                    seeds_[static_cast<std::size_t>(g)] = h;
                }
                int unseeded = 0;
                for (int g = 0; g < groupNumber_; ++g) {
                    if (!seeded[static_cast<std::size_t>(g)])
                        ++unseeded;
                }
                // PHASE60 物品 1 ②：**无 seed 组补种**（确定性、两侧同规则）。
                //
                // 背景：`groups=129` 时第 129 组在基网上**没有存活面** ⇒
                // `seeded=128 unseeded=1` ⇒ 该组无从起 BFS ⇒ 解码丢 189 顶点。
                // 根因（分组数失准）已由「exact-N 改为默认开」修掉
                // （实测 N∈{2..128} 恒 `groups==N`、`unseeded==0`）。
                //
                // 本兜底是**第二道防线**：让「某组在基网上确实没有面」这种边界
                // 仍有**确定的**起点，而不是 nullptr。
                // 规则：取 `mesh_->faces()` 序中**第一个存活的 facet** 的
                // `proxyHalfedge()` —— **与组号无关**，故编解码两侧可重算。
                // 语义上也自洽：该组的 BFS 过滤条件是 `face()->groupId() == g`，
                // 而没有任何面属于 g ⇒ 从哪条半边起 BFS 都收不到符号，
                // 起点只影响「不崩」，不影响内容。
                //
                // ⚠️ 实测 `unseeded` 在 exact-N 下恒为 0（N∈{2..128}），
                // 故本分支**当前不被触发**，是纯防御性代码 —— 不声称它被验证过。
                if (unseeded > 0) {
                    MCGAL::Halfedge* fallback = nullptr;
                    for (MCGAL::Facet* f : mesh_->faces()) {
                        if (!f || f->isRemoved()) continue;
                        MCGAL::Halfedge* h = f->proxyHalfedge();
                        if (h && h->vertex() && h->end_vertex()) {
                            fallback = h;
                            break;
                        }
                    }
                    if (fallback) {
                        for (int g = 0; g < groupNumber_; ++g) {
                            if (seeded[static_cast<std::size_t>(g)]) continue;
                            seeded[static_cast<std::size_t>(g)] = 1;
                            seedVertices_[static_cast<std::size_t>(g)] = {
                                fallback->vertex(), fallback->end_vertex()};
                            seeds_[static_cast<std::size_t>(g)] = fallback;
                        }
                        std::cerr << "[Columnar] segmented seed fallback: "
                                  << unseeded << " group(s) had no live base-mesh "
                                  << "facet; seeded deterministically\n";
                        unseeded = 0;
                    }
                }
                if (segmentedVerboseLogs() || unseeded) {
                    std::cerr << "[Columnar] segmented seed recompute: groups=" << groupNumber_
                              << " seeded=" << (groupNumber_ - unseeded) << " unseeded=" << unseeded << "\n";
                }

                // **真 seed 覆盖**（PHASE59 物品 3）。
                //
                // 上面那段「重算」两侧规则**同形但不同源**：编码端在
                // `applySegmentGroupTable` 里取的是**全网格**（消除之前、34,834 顶点）
                // `faces()` 序中每组首个 facet 的 `proxyHalfedge()`，而解码端此刻
                // 手上只有基网（5,939 顶点）。同一个「首个」，指的是**不同的面** ——
                // 实测两者的量化坐标落在不同区域（enc g0 (2005,1656,2953)/
                // (1984,1655,2936) vs dec g0 (1966,1654,2917)/(2023,1657,2972)），
                // 于是两侧 BFS 起点不同 ⇒ 标记集合只有 190/2384 项相同。
                //
                // 隔离实验（附 20）已证明这就是主因：把编码端**真** seed 喂进来，
                // 标记集合变成 2384/2384 + 119/119 全等，round 0 也精确
                // （g0 dF=2148=2×1074、g1 dF=106=2×53 ⇒ 14128 = E_9）。
                // 所以这里消费通道里传下来的序号，覆盖掉重算值。
                // N=1 **也消费**（groupNumber_ >= 1）：此前把 N=1 挡在外面，理由是
                // 「其重算值已被验证与编码端一致」—— 那是 N=1 走**默认 collector**
                // 的时代。现在 N=1 并入分段路径（见 SimplificationAlgorithmRegistry
                // 的 `numSegments >= 1`）：编码端单分区 seed 由 CompressPipeline 取
                // **主循环 seed**（恒受保护、落入基网、base vid=0/1）经本通道下发；
                // 解码端若改用重算值则两侧 BFS 起点不同 ⇒ 每面 splittable 位错配
                // ⇒ 解码塌成 11,653/16,325。消费后 N=1 逐项回环。N≥2 行为不变。
                //
                // [PHASE59 诊断，默认关闭] `PMSF_SEG_NO_SEED_CHANNEL` 关掉本段消费，
                // 用于**隔离**「真 seed 本身导致的失败」与「本段消费导致的失败」。
                if (groupNumber_ >= 1 && !std::getenv("PMSF_SEG_NO_SEED_CHANNEL")) {
                    std::vector<MCGAL::Vertex*> byOrdinal;
                    byOrdinal.reserve(mesh_->size_of_vertices());
                    for (MCGAL::Vertex* v : mesh_->vertices()) {
                        if (!v || v->isRemoved())
                            continue;
                        byOrdinal.push_back(v);
                    }
                    // [PHASE59 诊断，默认关闭] 解码端顶点序 ↔ 坐标对拍表。
                    std::size_t applied = 0;
                    for (std::size_t g = 0; g < seedOrdinals.size() && g < seeds_.size(); ++g) {
                        const int a = seedOrdinals[g].first;
                        const int b = seedOrdinals[g].second;
                        if (a < 0 || b < 0)
                            continue;
                        if (static_cast<std::size_t>(a) >= byOrdinal.size() || static_cast<std::size_t>(b) >= byOrdinal.size()) {
                            continue;
                        }
                        MCGAL::Vertex* va = byOrdinal[static_cast<std::size_t>(a)];
                        MCGAL::Vertex* vb = byOrdinal[static_cast<std::size_t>(b)];
                        // [PHASE59 诊断，默认关闭] 序号落点。
                        if (!va || !vb || va == vb)
                            continue;
                        MCGAL::Halfedge* found = nullptr;
                        for (MCGAL::Halfedge* hit : pmsf::boundedRing(va)) {
                            if (hit->end_vertex() == vb) {
                                found = hit;
                                break;
                            }
                        }
                        if (!found) {
                            for (MCGAL::Halfedge* hit : pmsf::boundedRing(vb)) {
                                if (hit->end_vertex() == va) {
                                    found = hit;
                                    break;
                                }
                            }
                        }
                        if (!found)
                            continue;
                        seedVertices_[g] = {va, vb};
                        seeds_[g] = found;
                        ++applied;
                    }
                    if (segmentedVerboseLogs() || applied != seedOrdinals.size()) {
                        std::cerr << "[Columnar] segmented seed ordinal apply: requested=" << seedOrdinals.size() << " applied=" << applied
                                  << " base_vertices=" << byOrdinal.size() << "\n";
                    }
                }

                // [PHASE59 诊断，默认关闭] **隔离实验**：直接把编码端的 seed
                // 量化坐标喂给解码端，用它在基网里定位 seed 顶点对。
                // 目的只是回答「seed 不同是不是回环失败的全部原因」，
                // 不是修复形态（真修复必须让编码端把 seed 传下来）。
                // [PHASE59 诊断，默认关闭] 解码端重算出的 seed 身份。
            }

            const size_t payloadStart = off;
            readers_.clear();
            readers_.resize(
                static_cast<std::size_t>(groupNumber_));
            for (int group = 0; group < groupNumber_; ++group) {
                size_t start = payloadStart +
                    groupStart_[static_cast<std::size_t>(group)];
                size_t end = payloadStart +
                    groupStart_[static_cast<std::size_t>(group + 1)];
                if (start > size) start = size;
                if (end > size) end = size;
                if (end < start) end = start;

                auto reader = std::make_unique<OptimizedSymbolReadOperator>();
                reader->setMesh(mesh_);
                reader->header_ = header_;
                reader->setCodecVersion(codecVersion_);
                reader->init({0, 0, 0}, header_);
                if (header_.isEnableQuantization()) {
                    reader->enableQuantization(
                        MCGAL::Point(bboxMin_.x(), bboxMin_.y(), bboxMin_.z()), quantStep_);
                }
                if (end > start) {
                    size_t localOff = 0;
                    reader->importFromRangeCodedBuffer(
                        blob + start,
                        localOff,
                        header_.isEnableQuantization() ? mesh_->i_nbQuantBits : 0);
                }
                readers_[static_cast<std::size_t>(group)] =
                    std::move(reader);
            }
    initialized_ = true;
    return true;
}

void SegmentedRangeCodedDecode::decodeOneRound() {
            if (groupNumber_ <= 0 ||
                readers_.empty()) {
                std::cerr << "[Columnar] segmented optimized readers are not initialized"
                          << std::endl;
                currentRound_++;
                return;
            }

            // Match the non-segmented/PPMC decode contract: per-round
            // splittable/added/processed state is transient. Leaving the
            // previous round's flags in place makes collectFromDecoded call
            // Facet::setSplittable() on an already-marked face.
            if (mesh_) {
                mesh_->resetState();
            }

            int beginGroup = 0;
            int endGroup = std::min<int>(groupNumber_, seeds_.size());
            // 单分区解码（PHASE60 物品 2）：`onlyGroupId_` 优先于 (roi) 谓词 —— 后者
            // 在基网表上是行级谓词，对单行基网表会把整张表滤空（见头文件
            // `setOnlyGroupId`）。本块是分区路径上**唯一**受单分区开关影响的地方：
            // 它把「本轮重建哪些分区」收窄到一组，其余（基网 / 段表 / 种子通道）
            // 照常整段读 —— 这正是「组 g 的符号只够重建组 g」该有的语义。
            const int groupSelect = (onlyGroupId_ >= 0) ? onlyGroupId_ : targetGroupId_;
            if (groupSelect >= 0) {
                beginGroup = std::min(groupSelect, groupNumber_);
                endGroup = std::min(beginGroup + 1, groupNumber_);
            } else if (maxGroups_ >= 0) {
                endGroup = std::min(endGroup, maxGroups_);
            }

            struct PendingSegmentedOptimized {
                int group = -1;
                std::vector<MCGAL::Vertex*> vertices;
                std::vector<MCGAL::Halfedge*> halfedges;
                std::vector<MCGAL::Facet*> facets;
            };
            std::vector<PendingSegmentedOptimized> pending;
            pending.reserve(static_cast<std::size_t>(std::max(0, endGroup - beginGroup)));

            auto buildLiveFacesByGroup = [&]() {
                std::vector<std::vector<MCGAL::Facet*>> facesByGroup(
                    static_cast<std::size_t>(std::max(0, groupNumber_)));
                if (!mesh_) return facesByGroup;
                for (MCGAL::Facet* f : mesh_->faces()) {
                    if (!f || f->isRemoved()) continue;
                    const int group = f->groupId();
                    if (group < 0 || group >= groupNumber_) continue;
                    facesByGroup[static_cast<std::size_t>(group)].push_back(f);
                }
                return facesByGroup;
            };
            auto liveFacesByGroup = buildLiveFacesByGroup();

            auto restorePendingState = [&]() {
                for (auto& item : pending) {
                    for (MCGAL::Facet* f : item.facets) {
                        if (!f || f->isRemoved()) continue;
                        if (f->isUnsplittable()) {
                            f->resetSplittableFlag();
                        }
                        if (!f->isSplittable()) {
                            f->setSplittable();
                        }
                    }
                    for (MCGAL::Halfedge* h : item.halfedges) {
                        if (!h || h->isRemoved()) continue;
                        if (!h->isAdded()) {
                            h->setAdded();
                        }
                    }
                }
            };
            auto resetPendingTransientState = [&](PendingSegmentedOptimized& item) {
                for (MCGAL::Vertex* v : item.vertices) {
                    if (v && !v->isRemoved()) {
                        v->resetState();
                    }
                }
                for (MCGAL::Facet* f : item.facets) {
                    if (f && !f->isRemoved()) {
                        f->resetState();
                    }
                }
                for (MCGAL::Halfedge* h : item.halfedges) {
                    if (!h || h->isRemoved()) continue;
                    h->resetState();
                    MCGAL::Halfedge* opp = h->opposite();
                    if (opp && !opp->isRemoved() && opp->face() &&
                        opp->face()->groupId() == item.group) {
                        opp->resetState();
                    }
                }
            };

            // 每轮重建都会**长出新面**，故「本组活面清单」必须每轮重算。
            // 只在 import 后算一次的话，第 1 轮起清单就只剩基网那批面，
            // `collectFromDecodedFromFaces` 的收尾扫描便漏掉历轮新增的面。
            liveFacesByGroup = buildLiveFacesByGroup();
            if (std::getenv("PMSF_SEG_CANON_TAIL")) {
                auto faceKey = [&](MCGAL::Facet* f) {
                    std::array<std::array<int, 3>, 3> k{};
                    MCGAL::Halfedge* h0 = f ? f->proxyHalfedge() : nullptr;
                    MCGAL::Halfedge* h = h0;
                    int i = 0;
                    int guard = 0;
                    do {
                        if (!h || !h->vertex() || i >= 3) break;
                        MCGAL::PointInt q = mesh_->floatPosToInt(h->vertex()->point());
                        const int mq = (mesh_->i_nbQuantBits > 0) ? ((1 << mesh_->i_nbQuantBits) - 1) : (1 << 30);
                        k[static_cast<std::size_t>(i)] = {std::clamp(q.x(), 0, mq),
                                                          std::clamp(q.y(), 0, mq),
                                                          std::clamp(q.z(), 0, mq)};
                        ++i;
                        h = h->next();
                    } while (h && h != h0 && ++guard < 64);
                    std::sort(k.begin(), k.end());
                    return k;
                };
                for (auto& bucket : liveFacesByGroup) {
                    std::sort(bucket.begin(), bucket.end(), [&](MCGAL::Facet* a, MCGAL::Facet* b) {
                        return faceKey(a) < faceKey(b);
                    });
                }
            }

            for (int group = beginGroup; group < endGroup; ++group) {
                // 种子数组 `seeds_` / `seedVertices_` 一律按**绝对** group id
                // 索引（见 `seedVertices_[g] = …` 的填充处，上面 seed 通道
                // 那段）。原先 `targetGroupId_ >= 0` 时算的是 `group - beginGroup`
                // —— 那是「相对被选子集」的下标，指到的是 **group 0 的种子**：
                // 单分区解码（PHASE60 物品 2）第一次实测就是拿 g0 的 seed 去读 g3
                // 的 payload，`collectFromDecodedFromFaces` 直接收不到符号。
                // 绝对索引对默认路径（`targetGroupId_ < 0`，此时恒有
                // `beginGroup == 0`）与旧式 `maxGroups_` 路径都是**同一个值**，
                // 故本行对非单分区路径零影响。
                const int seedIndex = group;
                if (group < 0 ||
                    seedIndex < 0 || seedIndex >= static_cast<int>(seeds_.size()) ||
                    seeds_[seedIndex] == nullptr ||
                    group >= static_cast<int>(readers_.size()) ||
                    !readers_[static_cast<std::size_t>(group)]) {
                    continue;
                }
                if (seedIndex < static_cast<int>(seedVertices_.size())) {
                    refreshSeed(seedVertices_[seedIndex].first,
                                seedVertices_[seedIndex].second,
                                seeds_[seedIndex],
                                group);
                }
                                if (seeds_[seedIndex] == nullptr) continue;

                PendingSegmentedOptimized item;
                item.group = group;
                auto& reader = readers_[static_cast<std::size_t>(group)];
                reader->groupId = group;
                reader->header_ = header_;
                if (header_.isEnableQuantization()) {
                    reader->enableQuantization(
                        MCGAL::Point(bboxMin_.x(), bboxMin_.y(), bboxMin_.z()), quantStep_);
                }
                // P18.2 PHASE59 实验（env 门控，默认关闭）：强制不做「只在本组面内
                // BFS」的限制，用来判定该限制是症状来源还是正确设计。
                const bool noFaceRestrict = std::getenv("PMSF_SEG_NO_FACE_RESTRICT") != nullptr;
                if (group < static_cast<int>(liveFacesByGroup.size()) && !noFaceRestrict) {
                    reader->collectFromDecodedFromFaces(
                        seeds_[seedIndex],
                        liveFacesByGroup[static_cast<std::size_t>(group)],
                        item.vertices,
                        item.halfedges,
                        item.facets);
                } else {
                    reader->collectFromDecoded(
                        seeds_[seedIndex], item.vertices, item.halfedges, item.facets);
                }
                if (segmentedVerboseLogs()) {
                    std::cerr << "[Columnar] segmented optimized group=" << group
                              << " round=" << currentRound_
                              << " vertices=" << item.vertices.size()
                              << " halfedges=" << item.halfedges.size()
                              << " facets=" << item.facets.size()
                              << "\n";
                }
                // [PHASE59 诊断，默认关闭] 与编码端 PMSF_SEG_HE_DUMP 同口径对拍。
                pending.push_back(std::move(item));
                resetPendingTransientState(pending.back());
            }
            // P18.2 PHASE59 实验（env 门控，默认关闭）：
            // `restorePendingState()` 一次性把**所有** pending 分组的 facet 都标成
            // Splittable，于是 g0 的 reconstruct 在 join 时会看到 g1 的面也是
            // Splittable（实测 g0 round0：splittableFlagged=1127 vs inFacets=1074，
            // 差值 53 恰为 g1 的 facet 数）。非分区路径没有这个错位
            // （N=1 实测每轮 splittableFlagged == inFacets）。
            // 本开关改成「只标正在重建的那一组」，用来判定该错位是不是回环失败的因子。
            const bool markPerGroup = std::getenv("PMSF_SEG_MARK_PER_GROUP") != nullptr;
            auto markOnlyPendingItem = [&](PendingSegmentedOptimized& target) {
                for (auto& item : pending) {
                    const bool on = (&item == &target);
                    for (MCGAL::Facet* f : item.facets) {
                        if (!f || f->isRemoved())
                            continue;
                        if (!on) {
                            if (f->isSplittable())
                                f->resetSplittableFlag();
                            continue;
                        }
                        if (f->isUnsplittable())
                            f->resetSplittableFlag();
                        if (!f->isSplittable())
                            f->setSplittable();
                    }
                    if (!on)
                        continue;
                    for (MCGAL::Halfedge* h : item.halfedges) {
                        if (!h || h->isRemoved())
                            continue;
                        if (!h->isAdded())
                            h->setAdded();
                    }
                }
            };
            if (!markPerGroup)
                restorePendingState();

            auto liveVNow = [&]() {
                std::size_t n = 0;
                for (MCGAL::Vertex* v : mesh_->vertices()) {
                    if (v && !v->isRemoved())
                        ++n;
                }
                return n;
            };
            auto liveFNow = [&]() {
                std::size_t n = 0;
                for (MCGAL::Facet* f : mesh_->faces()) {
                    if (f && !f->isRemoved())
                        ++n;
                }
                return n;
            };
            auto reconstructPending = [&](PendingSegmentedOptimized& item) {
                if (markPerGroup)
                    markOnlyPendingItem(item);
                const std::size_t v0 = liveVNow(), f0 = liveFNow();
                reconstructor_->reconstruct(item.vertices, item.halfedges, item.facets);
            };
            if (segmentedBatchGroupReconstruct()) {
                PendingSegmentedOptimized combined;
                combined.group = -1;
                for (auto& item : pending) {
                    combined.vertices.insert(combined.vertices.end(),
                                             item.vertices.begin(), item.vertices.end());
                    combined.halfedges.insert(combined.halfedges.end(),
                                              item.halfedges.begin(), item.halfedges.end());
                    combined.facets.insert(combined.facets.end(),
                                           item.facets.begin(), item.facets.end());
                }
                reconstructPending(combined);
            } else if (segmentedReverseGroupReconstruct()) {
                for (auto it = pending.rbegin(); it != pending.rend(); ++it) {
                    reconstructPending(*it);
                }
            } else {
                for (auto& item : pending) {
                    reconstructPending(item);
                }
            }

            // [PHASE59 因果实验，默认关闭] 把本轮的 facet groupId 强制钉到编码端
            // 目的：判定「12 个新面 groupId 不同」是不是**唯一**的轮 0 缺陷 —— 即
            // 下游的 261 面缺口是否只是它的后果。不是修复手段，是因果实验。
            if (const char* gidFix = std::getenv("PMSF_SEG_GID_FIX")) {
                // 文件按 `ROUND <n>` 分节（n = 本 dump 的轮号 = currentRound_ + 1），
                // 每节内每行 `<gid> x1 y1 z1 x2 y2 z2 x3 y3 z3`（坐标已排序）。
                // 每轮都重新钉一次：若 gid 是**唯一**缺陷，逐轮钉死后回环应当全对。
                static std::unordered_map<int, std::unordered_map<std::string, int>> gidFixByRound;
                static bool gidFixLoaded = false;
                if (!gidFixLoaded) {
                    gidFixLoaded = true;
                    std::ifstream in(gidFix);
                    std::string line;
                    int cur = -1;
                    while (std::getline(in, line)) {
                        if (line.rfind("ROUND", 0) == 0) {
                            cur = std::atoi(line.c_str() + 5);
                            continue;
                        }
                        if (cur < 0)
                            continue;
                        std::istringstream iss(line);
                        int g = 0;
                        if (!(iss >> g))
                            continue;
                        std::vector<std::array<int, 3>> vs;
                        int x = 0, y = 0, z = 0;
                        while (iss >> x >> y >> z)
                            vs.push_back({x, y, z});
                        if (vs.size() != 3)
                            continue;
                        int sp = 0;
                        if (!(iss >> sp))
                            sp = -1;  // 未给出 ⇒ 不钉瞬态标记
                        std::sort(vs.begin(), vs.end());
                        char key[128];
                        std::snprintf(key, sizeof(key), "%d_%d_%d_%d_%d_%d_%d_%d_%d",
                                      vs[0][0], vs[0][1], vs[0][2], vs[1][0], vs[1][1], vs[1][2], vs[2][0], vs[2][1], vs[2][2]);
                        // 编码：gid*4 + (sp+1)，sp ∈ {-1,0,1}（-1 = 不给 ⇒ 不钉瞬态标记）
                        gidFixByRound[cur][key] = g * 4 + (sp + 1);
                    }
                    std::fprintf(stderr, "[SEG_GID_FIX] loaded %zu rounds\n", gidFixByRound.size());
                }
                auto fixIt = gidFixByRound.find(currentRound_ + 1);
                if (fixIt != gidFixByRound.end()) {
                    const std::unordered_map<std::string, int>& want = fixIt->second;
                    int applied = 0, missing = 0;
                    for (MCGAL::Facet* f : mesh_->faces()) {
                        if (!f || f->isRemoved())
                            continue;
                        std::vector<std::array<int, 3>> vs;
                        MCGAL::Halfedge* h0 = f->proxyHalfedge();
                        MCGAL::Halfedge* h = h0;
                        int guard = 0;
                        do {
                            if (!h || ++guard > 32)
                                break;
                            MCGAL::PointInt p = mesh_->floatPosToInt(h->vertex()->point());
                            vs.push_back({p.x(), p.y(), p.z()});
                            h = h->next();
                        } while (h && h != h0);
                        if (vs.size() != 3)
                            continue;
                        std::sort(vs.begin(), vs.end());
                        char key[128];
                        std::snprintf(key, sizeof(key), "%d_%d_%d_%d_%d_%d_%d_%d_%d",
                                      vs[0][0], vs[0][1], vs[0][2], vs[1][0], vs[1][1], vs[1][2], vs[2][0], vs[2][1], vs[2][2]);
                        auto it = want.find(key);
                        if (it == want.end()) {
                            ++missing;
                            continue;
                        }
                        const int wantGid = it->second / 4;
                        const int wantSplit = it->second % 4 - 1;  // -1 = 不钉
                        bool changed = false;
                        if (f->groupId() != wantGid) {
                            f->setGroupId(wantGid);
                            changed = true;
                        }
                        if (wantSplit >= 0 && f->isSplittable() != (wantSplit == 1)) {
                            if (wantSplit == 1)
                                f->setSplittable();
                            else
                                f->resetSplittableFlag();
                            changed = true;
                        }
                        if (changed)
                            ++applied;
                    }
                    {
                        std::size_t lf = 0;
                        for (MCGAL::Facet* df : mesh_->faces())
                            if (df && !df->isRemoved()) ++lf;
                        std::fprintf(stderr,
                                     "[SEG_GID_FIX] round=%d reapplied=%d missing=%d liveF=%zu\n",
                                     currentRound_ + 1, applied, missing, lf);
                    }
                    std::fflush(stderr);
                }
            }

            // 写出本组重建**之后**的整网格活状态，标号 = 下一轮轮号（即解码端
            // 第 d 轮 collect 时刻的状态 = 编码端第 (R-1-d) 轮 collect 时刻的状态）。
            
            currentRound_++;
}

}  // namespace pmsf
