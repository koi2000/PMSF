/*****************************************************************************
 * @file impl/common/SegmentedRangeCodedDecode.h
 * @brief 分区（segmented）产物的**后端无关**解码镜像。
 *
 * 编码端 `SegmentedOptimizedSymbolCollectOperator::exportToBufferWithRangeCoder`
 * 与后端**无关**：列式与二进制写出的都是同一段 layout-32 blob。但直到 PHASE60
 * 这段 blob 的解码镜像只存在于 `impl/common/ColumnarDeserializeOperator.cpp`
 * （`blobLayout_ == kPaperBlobLayoutSegmentedRangeCodedPred` 分支）里，于是二进制
 * 后端拿到同一段字节却按「普通 range-coded blob」整体 `importFromRangeCodedBuffer`
 * ⇒ 前缀被当成符号流消费 ⇒ `Assertion !rcDecodedFacetSymbols_.empty() failed`
 * （实测 N=2/N=4 均 abort）。
 *
 * 本类把那段镜像的**机制**抽出来，供两个消费者共用：
 *   * `ColumnarDeserializeOperator`（列式 / Unified 后端）
 *   * `OptimizedBasicDeserializeOperator`（二进制后端）
 * 抽出的四块（与 blob 布局逐字段对称）：
 *   1. 前缀解析 `[groupCount][seedA[G]][seedB[G]][groupStart[G+1]]`
 *      + int8+RLE 的基网 facet → 分区通道
 *      `[baseFacetCount][rleRunCount][(gid:int8)(runLen:int32|varint) × runs]`
 *      A negative `rleRunCount` selects the self-describing varint run-length
 *      form; positive values retain the original fixed-width form.
 *   2. 把该通道**贴上**基网 facet（起点一致 ⇒ 消除期的继承链两侧同源）
 *   3. 每组 seed 的**解码端重算** + 通道下发的基网 `vid` 覆盖
 *   4. 每分区每轮的驱动器（collect + reconstruct 的瞬态标记/顺序约定）
 *
 * 调用序：基网就绪后 `importBlob(...)` 一次；随后每轮 `decodeOneRound()` 一次，
 * 与列式/二进制各自的主循环轮数一致。
 */
#ifndef PMSF_IMPL_COMMON_SEGMENTED_RANGE_CODED_DECODE_H
#define PMSF_IMPL_COMMON_SEGMENTED_RANGE_CODED_DECODE_H

#include "Halfedge.h"
#include "Mesh.h"
#include "Point.h"
#include "decompress/OptimizedSymbolReadOperator.h"
#include "header/PMSFHeader.h"
#include "reconstruct/ReconstructOpertator.h"

#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

namespace pmsf {

// ---------------------------------------------------------------------------
// 半边定位三件套 + seed 刷新。
//
// 从 `ColumnarDeserializeOperator::deserializeOneRound` 的同名 lambda **原样**
// 搬出：分区路径每轮都要重找 seed 半边（reconstruct 会让旧半边指针失效），
// 而非分区路径与分段 PPMC 路径用的是同一套规则，故单源在此、三处委托。
//
// `create_center_vertex_without_init` 不维护 `Vertex::halfedges()`，所以只能扫
// 活面环 —— 与 OpenMesh 的 `pushHehInit()` 同口径。
// ---------------------------------------------------------------------------
/// `from → to` 的一条活半边；`to == nullptr` = 从 `from` 出发的任一条；
/// `groupId >= 0` 时只看该分区的面。
MCGAL::Halfedge* findLiveHalfedgeInMesh(MCGAL::Mesh* mesh,
                                        MCGAL::Vertex* from,
                                        MCGAL::Vertex* to,
                                        int groupId = -1);
/// 任一条活半边（同上过滤）。
MCGAL::Halfedge* findAnyLiveHalfedgeInMesh(MCGAL::Mesh* mesh, int groupId = -1);
/// 分区内**量化坐标字典序最小**的活半边（env 门控实验路径用）。
MCGAL::Halfedge* findCanonicalGroupHalfedgeInMesh(MCGAL::Mesh* mesh, int groupId);
/// 按「先 `v0→v1`，再 `v0`、`v1` 单向，最后任取」的顺序刷新 `outSeed`；
/// 都找不到时保留原值。`preferCanonicalSeed` = 调用方的
/// `PMSF_SEGMENTED_DEFAULT_CANONICAL_SEED` 开关且布局适用。
void refreshLiveSeedHalfedge(MCGAL::Mesh* mesh,
                             MCGAL::Vertex* v0,
                             MCGAL::Vertex* v1,
                             MCGAL::Halfedge*& outSeed,
                             int groupId = -1,
                             bool preferCanonicalSeed = false);

/// 布局识别：这段字节**是不是** layout-32（分区）blob？
///
/// 二进制容器里**没有** layout 字段（列式把它存在 metadata 表），所以二进制
/// 侧的消费者只能按写出方
/// （`SegmentedOptimizedSymbolCollectOperator::exportToBufferWithRangeCoder`）
/// 的逐字段约束**反查**：`groupCount` 落在合理区间、种子是合法基网 vid、
/// 偏移表首项 0 且单调、RLE 通道自洽、以及最后那条字节等式
/// `前缀 + 通道 + 负载 == blob 长度`。
/// 最后一条把 5 个数钉死在一条算术关系上 —— 普通 range-coded blob 撞上的
/// 概率可忽略，所以本函数可以安全地当作「布局判别器」用。
///
/// 列式侧不需要它（layout 直接来自 metadata），故这里是二进制侧唯一消费者，
/// 但布局知识必须与 `importBlob` 的解析**同源**，所以放在同一处。
///
/// @param liveVertexCount / liveFacetCount 基网**解码后**的活顶点 / 活面数，
///        用于校验种子序号与 `baseFacetCount` 落在真实规模内。
bool segmentedRangeCodedBlobMatches(const char* blob,
                                    std::size_t size,
                                    int liveVertexCount,
                                    int liveFacetCount);

class SegmentedRangeCodedDecode {
  public:
    /// 后端提供的运行时上下文。所有指针都是**借用**；`mesh` 必须活得比本对象久。
    struct Context {
        PMSFHeader header;
        std::shared_ptr<MCGAL::Mesh> mesh;
        ReconstructOperator* reconstructor = nullptr;
        /// 语义版本透传给每组 reader。二进制容器没有版本字段（默认 0，等价于
        /// “最新”语义，见 `SymbolReadOperator::setCodecVersion` 的注释）；列式
        /// 从 metadata 表透传（本产物为 4）。两者在本路径上同为 `skipFaceBased`。
        int codecVersion = 0;
        // 量化参数（`header.isEnableQuantization()` 为真时必需）
        MCGAL::Point bboxMin;
        float quantStep = 0.0f;
    };

    /// 前缀解析 + 基网 facet 通道 + 每组 reader 构造。
    /// @return false 表示 blob 前缀非法或基网未就绪（消息已打到 stderr）。
    bool importBlob(char* blob, std::size_t size, const Context& ctx);

    /// 每轮重建哪些分区。语义与 `ColumnarDeserializeOperator` 的三个同名开关
    /// 相同（`setOnlyGroupId` 优先于 (roi) 谓词，只用绝对 group id 索引 seed），
    /// 且**每轮现读** —— 谓词可能在基网解码之后才被设置。默认 -1 ⇒ 全部。
    void setGroupSelection(int onlyGroupId, int targetGroupId, int maxGroups) {
        onlyGroupId_ = onlyGroupId;
        targetGroupId_ = targetGroupId;
        maxGroups_ = maxGroups;
    }

    /// 把每个被选中的分区**推进一轮**。轮数由调用方的主循环决定
    /// （= `PMSFHeader::getRound()`）；超出产物轮数时各组自然空转。
    void decodeOneRound();

    bool initialized() const {
        return initialized_;
    }
    int groupNumber() const {
        return groupNumber_;
    }
    int currentRound() const {
        return currentRound_;
    }
    /// 每分区 seed 半边（**绝对** group id 索引）。
    const std::vector<MCGAL::Halfedge*>& seeds() const {
        return seeds_;
    }
    /// 每分区 seed 的端点对（同上，绝对索引）。
    const std::vector<std::pair<MCGAL::Vertex*, MCGAL::Vertex*>>& seedVertices() const {
        return seedVertices_;
    }

  private:
    struct PendingGroup {
        int group = -1;
        std::vector<MCGAL::Vertex*> vertices;
        std::vector<MCGAL::Halfedge*> halfedges;
        std::vector<MCGAL::Facet*> facets;
    };

    MCGAL::Halfedge* findLiveHalfedge(MCGAL::Vertex* from, MCGAL::Vertex* to, int groupId = -1) const;
    MCGAL::Halfedge* findAnyLiveHalfedge(int groupId = -1) const;
    MCGAL::Halfedge* findCanonicalGroupHalfedge(int groupId) const;
    void refreshSeed(MCGAL::Vertex* v0, MCGAL::Vertex* v1, MCGAL::Halfedge*& outSeed, int groupId);

    PMSFHeader header_;
    std::shared_ptr<MCGAL::Mesh> mesh_;
    ReconstructOperator* reconstructor_ = nullptr;
    int codecVersion_ = 0;
    MCGAL::Point bboxMin_;
    float quantStep_ = 0.0f;
    int onlyGroupId_ = -1;
    int targetGroupId_ = -1;
    int maxGroups_ = -1;

    bool initialized_ = false;
    int groupNumber_ = 0;
    /// 每分区 payload 起点（G+1 项，已折算成 blob 内**绝对**偏移）
    std::vector<std::size_t> groupStart_;
    std::vector<std::unique_ptr<OptimizedSymbolReadOperator>> readers_;
    std::vector<std::pair<MCGAL::Vertex*, MCGAL::Vertex*>> seedVertices_;
    std::vector<MCGAL::Halfedge*> seeds_;
    int currentRound_ = 0;
};

}  // namespace pmsf

#endif  // PMSF_IMPL_COMMON_SEGMENTED_RANGE_CODED_DECODE_H
