#include "SegmentedOptimizedSymbolCollectOperator.h"

#include "BufferUtils.h"
#include "Configuration.h"
#include "EdgeBreakerMeshAdapter.h"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iterator>
#include <unordered_map>

namespace {

bool segmentedGroupChannelVarint() {
    const char* env = std::getenv("PMSF_SEGMENTED_GROUP_CHANNEL_VARINT");
    return env && *env && std::strcmp(env, "0") != 0 &&
           std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
}

// Lever A（分区 BPV）：把基网 facet 的落点顺序按 groupId 稳定排序，使
// facet→partition 通道从「空间交错的长 RLE」退化为「每组一段」(~G 段)。两处必须
// 用**完全相同**的排序：这里（通道写出）与 ColumnarSerializeOperator 的基网 facet
// 三元组写出。解码端按基网 facet 建立序（= base_mesh 表行序）贴 groupId，故只要
// 两处编码序一致，回环即正确。默认关闭，经 N=1..128 逐字节回归后再设默认。
bool segmentedBaseFaceSortEnabled() {
    const char* env = std::getenv("PMSF_SEG_BASE_FACE_SORT");
    return env && *env && std::strcmp(env, "0") != 0 &&
           std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
}

bool segmentedBaseEdgeBreakerNativeEnabled() {
    const char* env = std::getenv("PMSF_BASE_EDGEBREAKER_NATIVE");
    return env && *env && std::strcmp(env, "0") != 0 &&
           std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
}

std::size_t varUInt32Bytes(std::uint32_t value) {
    std::size_t bytes = 1;
    while (value >= 0x80u) {
        value >>= 7;
        ++bytes;
    }
    return bytes;
}

void writeVarUInt32(char* buffer, std::size_t& offset, std::uint32_t value) {
    while (value >= 0x80u) {
        buffer[offset++] = static_cast<char>((value & 0x7fu) | 0x80u);
        value >>= 7;
    }
    buffer[offset++] = static_cast<char>(value);
}

}  // namespace

void SegmentedOptimizedSymbolCollectOperator::init(
    std::shared_ptr<MCGAL::Mesh> mesh,
    std::vector<MCGAL::Halfedge*> seeds,
    PMSFHeader header) {
    SymbolCollectOperator::init(mesh, seeds, header);
    ensureCollectors(seeds.size());
}

void SegmentedOptimizedSymbolCollectOperator::setEnablePrediction(bool v) {
    SymbolCollectOperator::setEnablePrediction(v);
    // 两条路都要覆盖：内层**已构造**的（此处）与**之后**才构造的
    // （`ensureCollectors` 从 `enablePrediction_` 取，上面那句已更新它）。
    for (auto& collector : collectors_) {
        if (collector) collector->setEnablePrediction(v);
    }
}

void SegmentedOptimizedSymbolCollectOperator::ensureCollectors(std::size_t count) {
    if (collectors_.size() == count) return;
    collectors_.clear();
    collectors_.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        auto collector = std::make_unique<OptimizedSymbolCollectOperator>();
        collector->setEnablePrediction(enablePrediction_);
        collector->setGroupFilter(static_cast<int>(i));
        collector->init(mesh_, seeds_, header_);
        collectors_.push_back(std::move(collector));
    }
}

void SegmentedOptimizedSymbolCollectOperator::collect(MCGAL::Halfedge* seed) {
    if (collectors_.empty()) ensureCollectors(1);
    if (!collectors_.empty() && seed) {
        collectors_[0]->collect(seed);
        ++roundsCollected_;
        paperTableCache_.clear();
    }
}

void SegmentedOptimizedSymbolCollectOperator::collect(std::vector<MCGAL::Halfedge*> seeds) {
    if (seeds.empty()) return;
    seeds_ = seeds;
    ensureCollectors(seeds.size());
    for (std::size_t i = 0; i < seeds.size() && i < collectors_.size(); ++i) {
        if (!seeds[i]) continue;
        collectors_[i]->collect(seeds[i]);
    }
    ++roundsCollected_;
    paperTableCache_.clear();
}

void SegmentedOptimizedSymbolCollectOperator::collectRound(
    MCGAL::Halfedge* seed,
    const std::vector<MCGAL::Halfedge*>& seeds,
    const std::vector<std::uint8_t>& activeMask,
    int /*round*/) {
    // **N=1 也必须用真 seed**，不能退回主循环的 `seed`。
    //
    // 实测（N=1）：退回 `collect(seed)` 会走 `collectors_[0]->collect(seed)`，
    // 而主循环的 seed 来自 `selectSeed`（解码端也是按 vertices()[0]/[1] 重算
    // 的那一个），**与分区的 `exportSeeds()[0]` 不是同一条半边** ⇒ 内层
    // collector 从错误的起点 BFS ⇒ 符号流不自洽 ⇒
    // `BUG: rescaling left 49 total frequency`。
    // N=4 之所以没事，正是因为 collectActive 用的是 seeds[i] 而不是 seed。
    (void)seed;
    collectActive(seeds, activeMask, nullptr);
}

void SegmentedOptimizedSymbolCollectOperator::collectActive(
    const std::vector<MCGAL::Halfedge*>& seeds,
    const std::vector<std::uint8_t>& activeMask,
    const std::vector<std::vector<MCGAL::Facet*>>* liveFacesByGroup) {
    if (seeds.empty()) return;
    seeds_ = seeds;
    ensureCollectors(seeds.size());
    // Perf (slope-flatten): each inner `collect(seed)` ends with an O(F) scan of
    // the WHOLE mesh to pick up its group's live faces not reached by BFS
    // (OptimizedSymbolCollectOperator.cpp tail scan over `mesh_->faces()`).
    // Called once per group ⇒ the per-round cost is O(groups·F). We hoist a
    // single O(F) bucket pass here that groups live faces by groupId **in
    // mesh_->faces() container order**, then feed each collector only its own
    // bucket via collectFromFaces() (the override path consumes the exact same
    // faces, in the exact same order, applying the identical isRemoved/
    // isVisited/groupFilter_ predicates). The emitted symbol order — hence the
    // output bytes — is therefore unchanged; this is a pure O(groups·F)→O(F)
    // refactor. `PMSF_SEG_NO_BUCKET=1` restores the old per-group full scan
    // (A/B byte-identity escape hatch).
    static const bool bucketDisabled = [] {
        const char* e = std::getenv("PMSF_SEG_NO_BUCKET");
        return e && *e && std::strcmp(e, "0") != 0;
    }();
    std::vector<std::vector<MCGAL::Facet*>> localBuckets;
    const std::vector<std::vector<MCGAL::Facet*>>* groups = liveFacesByGroup;
    if (!groups && !bucketDisabled && mesh_) {
        localBuckets.assign(seeds.size(), std::vector<MCGAL::Facet*>());
        for (MCGAL::Facet* f : mesh_->faces()) {
            if (!f || f->isRemoved()) continue;
            const int g = f->groupId();
            if (g >= 0 && static_cast<std::size_t>(g) < localBuckets.size()) {
                localBuckets[static_cast<std::size_t>(g)].push_back(f);
            }
        }
        groups = &localBuckets;
    }
    // Multi-round boundary staging: canonical (geometric) tail-scan order so the
    // decoder reproduces the collector's face order even when a stage inverse
    // restored faces at the end of the container instead of their historical slots.
    if (std::getenv("PMSF_SEG_CANON_TAIL") && groups && mesh_) {
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
                k[static_cast<std::size_t>(i)] = {std::clamp(q.x(), 0, mq), std::clamp(q.y(), 0, mq),
                                                  std::clamp(q.z(), 0, mq)};
                ++i;
                h = h->next();
            } while (h && h != h0 && ++guard < 64);
            std::sort(k.begin(), k.end());
            return k;
        };
        for (auto& bucket : const_cast<std::vector<std::vector<MCGAL::Facet*>>&>(*groups)) {
            std::sort(bucket.begin(), bucket.end(),
                      [&](MCGAL::Facet* a, MCGAL::Facet* b) { return faceKey(a) < faceKey(b); });
        }
    }
    for (std::size_t i = 0; i < seeds.size() && i < collectors_.size(); ++i) {
        if (i >= activeMask.size() || !activeMask[i] || !seeds[i]) continue;
        if (groups && i < groups->size()) {
            collectors_[i]->collectFromFaces(seeds[i], (*groups)[i]);
        } else {
            collectors_[i]->collect(seeds[i]);
        }
    }
    ++roundsCollected_;
    paperTableCache_.clear();
}

const pmsf::PMSFSymbolTable* SegmentedOptimizedSymbolCollectOperator::pmsfSymbolTable() const {
    paperTableCache_.clear();
    for (std::size_t group = 0; group < collectors_.size(); ++group) {
        const auto& collector = collectors_[group];
        if (!collector) continue;
        const auto* rounds = collector->rounds();
        if (!rounds || rounds->empty()) continue;
        auto rows = pmsf::toPMSFSymbolTable(*rounds, 0, static_cast<std::int32_t>(group));
        paperTableCache_.insert(paperTableCache_.end(),
                                std::make_move_iterator(rows.begin()),
                                std::make_move_iterator(rows.end()));
    }
    return paperTableCache_.empty() ? nullptr : &paperTableCache_;
}

int SegmentedOptimizedSymbolCollectOperator::exportToBuffer(
    char* buffer,
    bool enableQuantization,
    bool /*enableCompress*/) {
    return exportToBufferWithRangeCoder(buffer, enableQuantization);
}

int SegmentedOptimizedSymbolCollectOperator::exportToBufferWithRangeCoder(
    char* buffer,
    bool enableQuantization) {
    if (!buffer) return 0;
    const int groupCount = static_cast<int>(collectors_.size());
    // P18.2 阶段 C 前置：**种子通道**。
    //
    // 分区 BFS 的起点必须让解码端知道。单 seed 时代它是由解码端**重算**的
    // （`mesh->vertices()[0]` → `[1]`，OptimizedBasicDeserializeOperator.cpp:708、
    // ParquetDeserializeOperator.cpp:109、BasicDeserializeOperator.cpp:711 三处
    // 皆然），多 seed 无法这样重算，而原布局
    //     [groupCount][groupStart[G+1]][payload]
    // 里**没有任何地方**放它们 —— 这就是物品 3 的阻塞点。
    //
    // 新布局（本函数是唯一写出方，故改布局只需同步 CollectorLayoutPlugin.cpp:88）：
    //     [groupCount][seedA[G]][seedB[G]][groupStart[G+1]]
    //     [baseFacetCount][rleRunCount][(gid:int8)(runLen:int32) × rleRunCount]
    //     [payload]
    // seedX[g] = 分区 g 的种子半边的两个端点，编码为**基网 vid**
    //            （`Vertex::vid()`；-1 表示无种子）。
    //
    // 为什么不是 poolId（P18.2 附 20 实测）：原实现写的是**全网格**（消除之前）
    // 顶点的 poolId，而解码端此刻手上只有基网 ⇒ 实测
    // `[SEED_POOL_PROBE] g=0 aspool=(65548,40740) resolved=((missing))((missing))`
    // （基网只有 5939 个顶点），通道**内容**无法被消费 —— 物品 3 的阻塞点。
    //
    // 为什么是 vid 而不是 `mesh_->vertices()` 的活顶点序号（实测决定）：
    // 基网表的**行内顶点序**由 `serializeBaseMeshWithSeed` 定：
    //   种子两端点占 vid = gid*2 / gid*2+1，同组其余顶点再按 `vertices()` 序
    //   从 `seeds.size()*2` 起递增（ColumnarSerializeOperator.cpp:241-275）。
    // 解码端按表行序 `addBaseVertex` ⇒ 其顶点下标空间 == 编码端的 vid 空间，
    // **不是** `vertices()` 序号。实测两例（N=2，g0/g1 种子端点）：
    //   坐标 (2005,1656,2953)：enc ord=5443、**vid=5465**、dec ord=5465 ✓
    //   坐标 (1984,1655,2936)：enc ord=1163、**vid=3288**、dec ord=3288 ✓
    // 用 ord 时两侧落点不同 ⇒ `applied=0`（找不到半边），回环失败。
    // 调用序保证 vid 已就绪：`CompressPipeline.cpp:1524` 的
    // `serializeBaseMeshWithSeed` 在 `serialize()`（本函数被调）**之前**。
    //
    // 布局字节数不变（仍是每组 2 个 int）。若 vid 将来失效（如共享坐标模式），
    // 解码端找不到半边 ⇒ applied < requested ⇒ 退回重算，与改动前同风险。
    //
    // **基网 facet → 分区通道**（PHASE59 物品 3，附 18 落点三）。
    //
    // 为什么必须下发而不是解码端重算：附 16 证明「分组 → 消除 → 基网 → 分组」成环，
    // 两侧拿不到同一个基网。而消除过程中 groupId 是**继承**的
    // （`VertexRemovalEliminateOperator::triangulate` 里 `newFace->setGroupId(父面)`），
    // 所以只要**起点（基网 facet 的分区号）**一致，解码端重建出的继承链就与编码端一致。
    //
    // 为什么 RLE + int8：分区在空间上连续 ⇒ 基网 facet 的分区号是长游程。原样
    // int32 写是 11,872 × 4 B ≈ 47 KB，而整个 `_compact.bin` 才 ~78 KB（附 18b 实测）。
    // 分区号 ∈ [0, G)，G ≤ 128 ⇒ int8 足够。
    //
    // **顺序约定**（两侧必须一致）：按 `mesh_->faces()` 的遍历序、跳过已删面。
    // 解码端把它按**基网 facet 的建立序**（= `addBaseFace` 的调用序 = base_mesh 表
    // 的行序）逐个贴上。两者在本路径的产物形态下一致：base_mesh 表当前恒为
    // **单行**（编码端从不对 header 设 groupNumber，见 `serializeBaseMeshWithSeed`
    // 的 `groupNumber <= 1` 分支），故表序就是网格序。
    const std::size_t metaBytes =
        sizeof(int) + static_cast<std::size_t>(groupCount) * 2 * sizeof(int) + static_cast<std::size_t>(groupCount + 1) * sizeof(int);

    // 基网 facet → 分区 的游程序列（仅 groupCount > 1 时写出）。
    // Lever A：可选按 groupId 稳定排序落点顺序，让游程退化为 ~G 段。
    std::vector<std::pair<int, int>> facetGroupRuns;  // (gid, count)
    int baseFacetCount = 0;
    if (groupCount > 1) {
        std::vector<MCGAL::Facet*> orderedFacets;
        for (MCGAL::Facet* f : mesh_->faces()) {
            if (!f || f->isRemoved())
                continue;
            orderedFacets.push_back(f);
        }
        if (segmentedBaseEdgeBreakerNativeEnabled()) {
            // Native EdgeBreaker: the decoder reconstructs base faces in EB
            // traversal order, so emit the channel in that SAME order. The EB
            // visit order is derived from the identical plan the encoder uses
            // (buildEbPlan over mesh_->faces() container order), guaranteeing
            // both sides agree without any shared mutable state.
            pmsf_eb::EbMeshPlan plan =
                pmsf_eb::buildEbPlan(mesh_.get(), /*baseMeshQuantBits=*/12,
                                     /*wantCoords=*/false);
            if (plan.ok &&
                plan.facesInOrder.size() == orderedFacets.size()) {
                std::vector<int> vo = pmsf_eb::computeEbVisitOrder(
                    plan.faces, static_cast<int>(plan.verts.size()));
                if (vo.size() == plan.facesInOrder.size()) {
                    std::vector<MCGAL::Facet*> ebOrder;
                    ebOrder.reserve(vo.size());
                    for (int idx : vo)
                        ebOrder.push_back(plan.facesInOrder[static_cast<std::size_t>(idx)]);
                    orderedFacets.swap(ebOrder);
                }
            }
        } else if (segmentedBaseFaceSortEnabled()) {
            // stable_sort 保组内原序 ⇒ 与 ColumnarSerializeOperator 基网 facet
            // 写出序必须用**同一把**排序键（groupId，稳定）。
            std::stable_sort(orderedFacets.begin(), orderedFacets.end(),
                             [](MCGAL::Facet* a, MCGAL::Facet* b) {
                                 return a->groupId() < b->groupId();
                             });
        }
        for (MCGAL::Facet* f : orderedFacets) {
            const int gid = f->groupId();
            ++baseFacetCount;
            if (!facetGroupRuns.empty() && facetGroupRuns.back().first == gid) {
                ++facetGroupRuns.back().second;
            } else {
                facetGroupRuns.emplace_back(gid, 1);
            }
        }
    }
    const bool varintGroupChannel = groupCount > 1 && segmentedGroupChannelVarint();
    std::size_t channelBytes = 0;
    if (groupCount > 1) {
        channelBytes = sizeof(int) + sizeof(int);
        for (const auto& run : facetGroupRuns) {
            channelBytes += sizeof(std::int8_t);
            channelBytes += varintGroupChannel
                                ? varUInt32Bytes(static_cast<std::uint32_t>(run.second))
                                : sizeof(int);
        }
    }

    // [PHASE59 诊断，默认关闭] dump 基网 facet → 分区 的**逐个**序列，
    // 供与解码端贴上的结果逐项对拍。

    std::vector<std::vector<char>> groupBlobs(static_cast<std::size_t>(groupCount));
    std::vector<int> groupStarts(static_cast<std::size_t>(groupCount + 1), 0);

    int payloadBytes = 0;
    std::unique_ptr<char[]> scratch(new char[BUFFER_SIZE]);
    for (int group = 0; group < groupCount; ++group) {
        groupStarts[static_cast<std::size_t>(group)] = payloadBytes;
        int used = 0;
        if (collectors_[static_cast<std::size_t>(group)]) {
            used = collectors_[static_cast<std::size_t>(group)]->exportToBufferWithRangeCoder(scratch.get(), enableQuantization);
        }
        if (used < 0)
            used = 0;
        groupBlobs[static_cast<std::size_t>(group)].assign(scratch.get(), scratch.get() + used);
        payloadBytes += used;
    }
    groupStarts[static_cast<std::size_t>(groupCount)] = payloadBytes;

    // [PHASE59 诊断，默认关闭] 编码端顶点序 ↔ 坐标 两个索引空间的对拍表。

    // [PHASE59 诊断，默认关闭] 编码端种子序号与其落点坐标。

    size_t offset = 0;
    writeInt(buffer, offset, groupCount);
    // 种子通道（见上）：每个分区两个端点，编码为**基网 vid**。
    for (int g = 0; g < groupCount; ++g) {
        MCGAL::Halfedge* s = (static_cast<std::size_t>(g) < seeds_.size()) ? seeds_[static_cast<std::size_t>(g)] : nullptr;
        const int a = (s && s->vertex()) ? s->vertex()->vid() : -1;
        const int b = (s && s->end_vertex()) ? s->end_vertex()->vid() : -1;
        writeInt(buffer, offset, a);
        writeInt(buffer, offset, b);
    }
    for (int v : groupStarts)
        writeInt(buffer, offset, v);
    // 基网 facet → 分区通道（仅分区产物写出；G <= 1 时整段缺席 ⇒ 默认路径零变化）。
    if (groupCount > 1) {
        writeInt(buffer, offset, baseFacetCount);
        const int encodedRunCount =
            varintGroupChannel
                ? -static_cast<int>(facetGroupRuns.size())
                : static_cast<int>(facetGroupRuns.size());
        writeInt(buffer, offset, encodedRunCount);
        for (const auto& run : facetGroupRuns) {
            const std::int8_t gid = static_cast<std::int8_t>(run.first);
            std::memcpy(buffer + offset, &gid, sizeof(gid));
            offset += sizeof(gid);
            if (varintGroupChannel) {
                writeVarUInt32(buffer, offset, static_cast<std::uint32_t>(run.second));
            } else {
                writeInt(buffer, offset, run.second);
            }
        }
    }
    for (const auto& blob : groupBlobs) {
        if (!blob.empty()) {
            std::memcpy(buffer + offset, blob.data(), blob.size());
            offset += blob.size();
        }
    }

    // [PHASE59 诊断，默认关闭] 每组每轮的 facet/edge 符号计数（未反转的编码序）。
    std::cout << "[SEGMENTED_OPTIMIZED_RC_BREAKDOWN]"
              << " groups=" << groupCount << " rounds=" << roundsCollected_ << " meta_bytes=" << metaBytes
              << " base_group_channel_facets=" << baseFacetCount << " base_group_channel_runs=" << facetGroupRuns.size()
              << " base_group_channel_codec=" << (varintGroupChannel ? "varint" : "fixed32")
              << " base_group_channel_bytes=" << channelBytes << " payload_bytes=" << payloadBytes
              << " total_bytes=" << offset << std::endl;
    return static_cast<int>(offset);
}
