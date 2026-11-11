/*
 * @Author: koi
 * @Description: Parquet V2 多表方案序列化操作符
 *
 * 采用 4 个独立 Parquet 表（独立 Schema），消除单表方案中的大量空值：
 *   - mesh_metadata: PMSFHeader 元数据（1行）
 *   - group_base_mesh: 按分区存储基础网格（每 group 一行）
 *   - group_round_symbols: 按 (group_id, round_id) 存储符号数据
 *   - boundary_graph: 分区边界图信息
 *
 * 所有表存储在同一个 Parquet 文件中，通过 Row Group 分隔。
 * 每个表有独立的 Schema，通过文件级 KV 元数据记录各表的 Row Group 范围。
 *
 * 编码优化：RLE（布尔符号）、DELTA_BINARY_PACKED（ID/索引）、BYTE_STREAM_SPLIT（浮点）
 * 压缩策略：ZSTD level 3
 */
#ifndef COLUMNAR_SERIALIZEOPERATOR_H
#define COLUMNAR_SERIALIZEOPERATOR_H

#include "SerializeOperator.h"
#include "SegmentationSymbolCollectOperator.h"
#include "OptimizedSymbolCollectOperator.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "../common/Graph.h"
#include <cstdint>
#include <string>
#include <memory>
#include <vector>

#include <arrow/api.h>
#include <arrow/io/api.h>
#include <arrow/builder.h>
#include <parquet/arrow/writer.h>
#include <parquet/properties.h>

/**
 * @brief Parquet V2 多表方案序列化操作符
 * @details 将 PMSF 压缩数据以 4 个独立表写入 Parquet 文件。
 *
 * 多文件方案：每个表写入独立的 Parquet 文件
 *   - <basename>_metadata.parquet
 *   - <basename>_base_mesh.parquet
 *   - <basename>_symbols.parquet
 *   - <basename>_boundary.parquet
 *
 * Row Group 策略：
 *   - group_base_mesh: 每个 group 一个 Row Group
 *   - group_round_symbols: 每个 group 一个 Row Group（包含该 group 所有 round）
 */
class ColumnarSerializeOperator : public SerializeOperator {
  public:
    ColumnarSerializeOperator(std::shared_ptr<SymbolCollectOperator> collector,
                               bool enablePrediction = false,
                               bool enableQuantization = false,
                               bool enableCompress = false)
        : SerializeOperator(collector, enablePrediction, enableQuantization, enableCompress) {}
    ~ColumnarSerializeOperator() = default;
    void init() override;

    void serializeInt(int val) override;

    /// @brief 记录压缩前原始网格规模；serialize 时写进 compact header 的
    ///        origTotalVertices/origTotalFaces（线格式里该字段本就存在，
    ///        此前被误填为基网数）。未设置（<=0）时保持旧行为（写基网数）。
    void setOriginalMeshCounts(int vertices, int faces) override {
        originalInputVertices_ = vertices;
        originalInputFaces_ = faces;
    }

    void serializeBaseMesh(std::shared_ptr<MCGAL::Mesh> mesh) override;

    void serializeBaseMeshWithSeed(std::shared_ptr<MCGAL::Mesh> mesh,
                                   std::vector<MCGAL::Halfedge*> seeds) override;

    void serialize(std::string path) override;

    /// @brief 设置分区边界图（可选，仅分区模式下使用）
    void setBoundaryGraph(std::shared_ptr<Graph> graph) {
        boundaryGraph_ = graph;
    }

    /// @brief 设置 codec 版本
    ///   - 1: 旧版 bool-list / int16-list 多列方案（向后兼容，读取端可识别）
    ///   - 2: BPV 优化版（默认）。symbols 表退化为单 BINARY 列 rc_blob，
    ///        blob 内容直接来自 collector 的 exportToBuffer*() —— 与非 Parquet
    ///        BasicSerializeOperator 产生的符号比特流**逐字节等价**，因此完全复用
    ///        非 Parquet 版本已经验证过的熵编码 / 位面 / 上下文模型等 BPV 优化。
    void setCodecVersion(int v) { codecVersion_ = v; }
    int  getCodecVersion() const { return codecVersion_; }

    /// @brief Phase D (v3-refactor)：暴露 blobLayout_ 给 V3 紧凑模式写入 compact bin。
    /// V2 的 compact 路径（Tier 2 移植，2026-05-09）也读这个值——共享语义不变。
    int  getBlobLayout() const { return static_cast<int>(blobLayout_); }

    /// @brief Tier 2 (2026-05-09)：V2 紧凑模式开关。启用后 serialize() 在写完 4 张 parquet 后
    ///        额外产出 `<base>_compact.bin`（zlib-deflated PMSFHeader + groupBaseMeshData_），
    ///        并删除 `<base>_metadata.parquet` + `<base>_base_mesh.parquet`。其字节布局与
    ///        V3 的 `_v3_compact.bin` 相同，magic 为 "V2CP"。
    ///        与 V3 的 setCompactMode() 完全独立，互不影响。
    ///        通过环境变量 `PMSF_V2_COMPACT=1` 也可启用（不需要直接设置）。
    void setV2CompactMode(bool v) { v2CompactMode_ = v; }
    bool isV2CompactMode() const { return v2CompactMode_; }

    /// @brief 分区 base mesh 坐标字典模式。
    /// 拓扑顶点仍按 group 保持独立，face_vertices 仍引用拓扑顶点 ID；
    /// 坐标只写入共享字典一次，每个拓扑顶点通过 coord_id 引用坐标。
    void setSharedBaseMeshCoordinates(bool v) { sharedBaseMeshCoordinates_ = v; }
    bool isSharedBaseMeshCoordinates() const { return sharedBaseMeshCoordinates_; }

    /// @brief 分区 base mesh 直接用共享坐标 ID 存 face_vertices。
    /// 这是更激进的 shared-coords 变体：不再为每个 group 写 local_id -> coord_id
    /// 映射，解码时按 group 按需把 coord_id 实例化为局部拓扑顶点。
    void setSharedBaseMeshFaceCoordIds(bool v) {
        sharedBaseMeshFaceCoordIds_ = v;
        if (v) sharedBaseMeshCoordinates_ = true;
    }
    bool isSharedBaseMeshFaceCoordIds() const { return sharedBaseMeshFaceCoordIds_; }

    /// @brief 分区 base mesh 紧凑存储模式。
    /// 坐标写入 `<base>_vertices.parquet` 的全局 block 表；每个分区在
    /// `<base>_partitions.parquet` 中只写一个 bitpacked base_blob。
    /// 该模式要求 segmentation + quantization + shared face coord ids。
    void setPartitionedCompactBaseMesh(bool v) {
        partitionedCompactBaseMesh_ = v;
        if (v) setSharedBaseMeshFaceCoordIds(true);
    }
    bool isPartitionedCompactBaseMesh() const { return partitionedCompactBaseMesh_; }

    /// @brief blob 内部 layout 枚举：解码端根据它选择对称的解码入口
    enum class BlobLayout : int {
        UNKNOWN          = pmsf::kPaperBlobLayoutUnknown,
        // OptimizedSymbolCollectOperator::exportToBufferWithRangeCoder 产出
        // → OptimizedSymbolReadOperator::importFromRangeCodedBuffer 消费
        RANGE_CODED_PRED = pmsf::kPaperBlobLayoutRangeCodedPred,
        // BasicSymbolCollectOperator::exportToBuffer 产出
        // → BasicSymbolReadOperator::collect 按轮消费 ([int symSize][sym][pts] + 可选 zlib)
        BASIC            = pmsf::kPaperBlobLayoutBasic,
        // PPMCSymbolCollectOperator::exportToBuffer 产出（多段 RC 串接）
        // → PPMCSymbolReadOperator::collect 按轮消费（直接从 buffer 起点）
        CODEC_003          = pmsf::kPaperBlobLayoutCodec003,
        // OptimizedSymbolCollectOperator::exportToBuffer 产出（4 段位流，可选 zlib）
        // → OptimizedSymbolReadOperator::collect 按轮消费
        OPTIMIZED_PLAIN  = pmsf::kPaperBlobLayoutOptimizedPlain,
        // ValenceSymbolCollectOperator::exportToBufferWithRangeCoder 产出
        // → ValenceSymbolReadOperator::importFromRangeCodedBuffer 消费
        // 7 元码字 + point 残差 / 位面，论文 PCLTTM 连通性编码路径。
        VALENCE_RC       = pmsf::kPaperBlobLayoutValenceRC,
        // DefaultPPMCLiteSymbolCollectOperator::exportToBufferWithRangeCoder 产出
        // → DefaultPPMCLiteSymbolReadOperator::importFromRangeCodedBuffer 消费
        // 与 RANGE_CODED_PRED 的 v3 per-edge 码流字节级布局相同（facet/edge/
        // vertex/point 四段），唯一区别是 point 段的 bit-plane residual 已
        // 经过 Frenet 旋转到局部 (α,β,γ) 坐标系。decoder 按 BFS 同步反旋转,
        // 因此 point 段在 decoder 侧是"延迟解码"的。
        CODEC_006 = pmsf::kPaperBlobLayoutCodec006,
        // P13.4 (B)②：valence-conquest 原生 codec。
        // ConquestSymbolCollectOperator::exportToBufferWithRangeCoder 产出
        // → ConquestSymbolReadOperator::importFromRangeCodedBuffer 装填队列
        //   → ConquestReconstructOperator::reconstruct 调 `pmsf::decodeLayer` 还拓扑。
        // 单元：每轮一段 [numSymbols][symbols][numDeltas][deltas]，前配自描述头部
        // （magic "CQST" + 版本 + 量化网格）。量化网格**写进流里**而不是从
        // metadata 取 —— 蓝图 §54 的教训：量化器的定义经有损中间表示会伪装成
        // "被测对象错了"。
        CONQUEST_RC = pmsf::kPaperBlobLayoutConquestRC,
        // RDSymbolCollectOperator::exportToBufferWithRangeCoder 产出
        // → RDSymbolReadOperator::importFromRangeCodedBuffer 消费
        // Phase A：字节 layout 等价于 VALENCE_RC（RDSymbolCollectOperator 继承
        // 自 ValenceSymbolCollectOperator，直接 fall-through 到父类导出）。
        // 独立 BlobLayout 的意义是让未来 Phase B 改动（真·gate-only BFS、
        // Frenet 旋转、cleaning-layer 码字拆分）全部落到 RDSymbolReadOperator，
        // Valence baseline 保持字节级不变。
        RD_GATE          = pmsf::kPaperBlobLayoutRDGate,
        // RDGateOnlySymbolCollectOperator::exportToBufferWithRangeCoder 产出
        // Phase B encoder-only marker (2026-05-09)。
        // 每轮只 emit `gates.size()` 个 5-元 valence symbol（V_3..V_6, V_OTHER），
        // 不发 NULL_PATCH / FAN_CHILD 码字，没有 edge-sym 段。
        // 布局：`[rcValSize][rcPointSize][numGates][numPointCoords][numRounds]
        //         [gatesPerRound*numRounds][rcValBuf][rcPointBuf]`。
        // ⚠️ 本 layout 的 decoder（RDGateOnlySymbolReadOperator）**尚未实装**。
        // ColumnarDeserializeOperator `case 8:` 直接 fail-fast；请勿对 RD_GATE_ONLY
        // 的 parquet 尝试解压。未来 session 会补 decoder 并用 v0-fan 反演推回
        // topology。
        RD_GATE_ONLY     = pmsf::kPaperBlobLayoutRDGateOnly,
        // ────────────────────────────────────────────────────────────────
        // Phase 1 · Step 1.1 (docs/CONNECTIVITY_BITSTREAM_IMPLEMENTATION_PLAN.md §1)
        // 方案 V — Valence-Patch-Only。
        // 与 RD_GATE_ONLY (=8) 字节流的差异：
        //   - 把 rcVal (5 元) + rcIsPoly (per-face 1 bit) 两段合并为一个
        //     6 元 alphabet stream（NULL_PATCH=0, V_3..V_6, V_OTHER），
        //     用 ContextAdaptiveRangeCoder 的 4-context 模型编码（first /
        //     prev=NULL / prev=V* / prev=OTHER）。
        //   - 保留 rcEdge (per-polyFace 3 bit) 段不动，确保 v0-fan 几何
        //     反演 byte-identical（v2 协议去 edge-sym 已实测 RMS×340 偏差，
        //     不可恢复）。
        //   - 保留 rcPoint (bit-plane 残差) 段不动。
        // 字节流用 ConnectivityBlobContainer v3 协议封装（magic='CONN'），
        // 子流：[stream 0=valence6, 1=edgeSym, 2=residual]，每轮共 3 个子流。
        // 配套 collector / reader：
        //   - ValencePatchOnlySymbolCollectOperator (src/compress/)
        //   - ValencePatchOnlySymbolReadOperator   (src/decompress/)
        // PCLTTM-Valence 首发；Step 1.6 推广到 RD/MMSE/APTM/PCATM/PCLTTM。
        VALENCE_PATCH_ONLY = pmsf::kPaperBlobLayoutValencePatchOnly,
        // ────────────────────────────────────────────────────────────────
        // Phase 0 · Step 0.5 预留的新方案 BlobLayout（占位，decoder 暂未实装）
        // 见 docs/CONNECTIVITY_BITSTREAM_IMPLEMENTATION_PLAN.md §5.x。
        // 任何 Phase 3-5 之外的生产者都不应该产出这三个 layout；
        // ColumnarDeserializeOperator 对它们 fail-fast。
        // ────────────────────────────────────────────────────────────────
        /// 方案 C（CPM-EdgeSplit）—— VQMMC / QEM 的每条 edge 1 bit split 流。Phase 3 落地。
        CPM_EDGE_SPLIT   = pmsf::kPaperBlobLayoutCPMEdgeSplit,
        /// 方案 P（POMAR-FourStream）—— POMAR 的 (s_v, s_e, s_l, s_r) 四流。Phase 4 落地。
        CODEC_012    = pmsf::kPaperBlobLayoutCodec012,
        /// 方案 H（RD-BitPlane）—— RD 论文 R-D-ordered bit-plane 流。Phase 5 落地。
        RD_BITPLANE      = pmsf::kPaperBlobLayoutRDBitplane,
        /// PCATM 论文 2-color 方案（Cohen-Or et al. 2002）—— 3 段流：
        ///   stream 0 (valence):  5 元 alphabet (V_3..V_OTHER) + 4-context RC
        ///                        + V_OTHER inline uint8 escape；每 patch 1 sym。
        ///   stream 1 (color):    2 元 RC，每 BFS-encountered face 1 bit =
        ///                        (face.getTriangleColor() == BLUE ? 1 : 0)；
        ///                        由 PCATMEliminateOperator Z-shape 三角化预先着色。
        ///   stream 2 (residual): bit-plane RC（verbatim 父类 ipointQueues_ 路径）。
        /// 封装：ConnectivityBlobContainer v3 (magic='CONN')，R 轮 × S=3 段流。
        /// 配套 collector / reader：
        ///   - PCATMColorSymbolCollectOperator (src/compress/)
        ///   - PCATMColorSymbolReadOperator    (src/decompress/，CP-3.3+)
        CODEC_014      = pmsf::kPaperBlobLayoutCodec014,
        // PCLTTM Tag (Alliez & Desbrun 2001) path-B BlobLayout (2026-05-10).
        // 3 段流：valence (5-元 RC) + vertexTag (3-元 RC) + residual (bit-plane RC)。
        // 显式排除：isPoly stream / edgeSym stream / color stream
        // (vertex tag 的信息已涵盖 polyFace boundary 信号，几何由 v0-fan 默认重建)。
        // 配套 collector / reader：
        //   - PCLTTMTagSymbolCollectOperator (src/compress/，继承 PCATMColor)
        //   - PCLTTMTagSymbolReadOperator    (src/decompress/，PR-3 stub，PR-4 完整)
        // 详见 docs/paper_alignment/PCLTTM_OFFICIAL_REPRO_PROMPT.md PR-3。
        CODEC_016       = pmsf::kPaperBlobLayoutCodec016,
        // ────────────────────────────────────────────────────────────────
        // Phase 4 PR-A 占位枚举（2026-05-11）—— 仅 enum 注册，decoder 实装在 PR-B/PR-C。
        // 详见 docs/paper_alignment/PHASE4_PLAN.md §3.2/§3.3/§3.4。
        // 本 BlobLayout 枚举值与 pmsf::RetriProtocol（src/common/RetriangulationProtocol.h）
        // 严格 1:1 对齐。
        // ────────────────────────────────────────────────────────────────
        /// PCATM Z-shape + 2/4-color 协议（Cohen-Or et al. 1999）—— PR-C 实装。
        /// 4 段流：valence + colorBits + colorMode + residual；decoder 用
        /// union-find 合并同色相邻 triangle 推 patch 边界，**完全砍 edgeSym**。
        /// 配套 collector / reader：PCATMZColorSymbolCollectOperator /
        /// PCATMZColorSymbolReadOperator（PR-C 落地）。
        CODEC_017     = pmsf::kPaperBlobLayoutCodec017,
        /// RD Tag 协议（Ahn et al. 2011）—— PR-B 实装。
        /// 3 段流：valence + isPolyBit + residual；与 CODEC_016 高度同构，
        /// **砍 vertex tag 流**（RD 没有 PCLTTM 的 ⊕/⊖ tag stream）。
        /// 配套 collector / reader：RDTagSymbolCollectOperator /
        /// RDTagSymbolReadOperator（PR-B 落地）。
        RD_TAG           = pmsf::kPaperBlobLayoutRDTag,
        // ────────────────────────────────────────────────────────────────
        // M1-b Step 2 落地（2026-05-11）—— PCATM 论文严格对齐协议。
        // 详见 docs/paper_alignment/pcatm_zshape_groundtruth.md +
        //     impl/PCATM/PCATMZShapeGeometry.h (PR-1 ground-truth helper)。
        // ────────────────────────────────────────────────────────────────
        /// PCATM 论文 Z-shape 严格几何（Cohen-Or, Levin, Remez 1999）—— M1-b 实装。
        /// 4 段流（与 CODEC_017 字节布局相同，仅协议号不同以让 decoder 走真 Z-shape
        /// 反演路径）：valence + isPolyBit + colorBits + residual。
        /// 与 CODEC_017 (=17) 的本质差异：
        ///   - encoder 几何：真 Z-shape (zigzag) 三角化（vs CODEC_017 的 fan + 着色）
        ///   - decoder 几何：用 PCATMZShapeGeometry::computeZShape 反演 N-3 条对角线
        ///                   （vs CODEC_017 的 v0-fan 反演；M1-b Step 2c 落地）
        /// M1-b Step 2a/2b/2c：encoder、4 段流 collector、reader 的 paper hint
        /// 标记，以及 reconstruct strict-paper 合并路径已落地。
        /// 配套 collector / eliminate / reader：
        ///   - PCATMPMSFSymbolCollectOperator (src/compress/)
        ///   - PCATMPaperEliminateOperator     (src/elimination/)
        ///   - PCATMPMSFSymbolReadOperator    (src/decompress/)
        CODEC_020      = pmsf::kPaperBlobLayoutCodec020,
        // ────────────────────────────────────────────────────────────────
        // PR-K3 落地（2026-05-11）—— PCATM 论文"信息论最小"协议。
        // ────────────────────────────────────────────────────────────────
        /// PCATM Pure Paper（Cohen-Or, Levin, Remez 1999 §4 信息论最小集）—— PR-K3 实装。
        /// **3 段流**（砍 color 流，与 RDTag=18 同构）：valence + isPolyBit + residual。
        ///
        /// 为什么砍 color 而非按论文严格只发 color：
        ///   - 论文原始方案（"only color stream + LZ"）decoder 需要 UF-based Z-shape 反演
        ///     来从 color 序列推 patch 边界；PR-J2.1 spike 在 decimating-only 层可行，
        ///     cleaning 层由于 n=3 triangulate 早返，无法可靠实装。
        ///   - 当前 PCATMPaper encoder 用 true Z-shape + alternating color，
        ///     color 流仍作为 protocol 字段保留；Pure 路径砍掉该段以控制 BPV。
        ///   - 砍 color 段后 conn 段 = valence 2.32 bpv + isPolyBit 4.81 bpv = 7.13 bpv，
        ///     与仓库最优 RDTag (7.13) 同量级；相比 PCATMPaper 16.47 大幅缩减。
        ///   - 若未来实装 UF 反演，可新增 BlobLayout=22 走"只发 color" 路径达 ~6 bpv。
        ///
        /// 与 RDTag(18) 的本质差异：
        ///   - 都是 3 段流（valence + isPolyBit + residual），字节布局相同
        ///   - encoder 端 eliminate / selector 不同：
        ///       * RDTag 用 RDTagEliminateOperator（RD priority cost）
        ///       * PCATMPure 用 PCATMPaperEliminateOperator（PCATM priority cost +
        ///         barycenter 量化整数累加平均，论文 §5 vc predictor）
        ///   - BlobLayout 号 21 vs 18 用于 decoder-side 区分两种 eliminate 产出
        ///     （虽然 decoder 算法完全相同，但协议号是 encoder/decoder 配对的唯一标识）
        ///
        /// 配套 collector / reader：
        ///   - PCATMPureSymbolCollectOperator (src/compress/)
        ///   - PCATMPureSymbolReadOperator   (src/decompress/)
        ///
        /// 详见 docs/paper_alignment/PCATM_PURE_PAPER_NEXT_SESSION_PROMPT.md §C PR-K3。
        CODEC_021       = pmsf::kPaperBlobLayoutCodec021,
        /// MMSE Dual-Ring 协议（PR-L4b）—— 3 段流：valence + isPolyBit + residual，
        /// 与 RDTag(18) 同构，residual 段经过 MMSEPostProcessor dual-ring 预变换。
        /// 配套 collector / reader：
        ///   - MMSEDualRingSymbolCollectOperator (src/compress/)
        ///   - MMSEDualRingSymbolReadOperator   (src/decompress/)
        CODEC_022   = pmsf::kPaperBlobLayoutCodec022,
        /// VQMMC Codebook 协议（PR-L5）—— 4 段流：valence + isPolyBit +
        /// codebook + indices。将 VQ codebook 嵌入 ConnectivityBlobContainer。
        /// 配套 collector / reader：
        ///   - VQMMCCodebookSymbolCollectOperator (src/compress/)
        ///   - VQMMCCodebookSymbolReadOperator   (src/decompress/)
        CODEC_023   = pmsf::kPaperBlobLayoutCodec023,
        CODEC_025  = pmsf::kPaperBlobLayoutCodec025,
        CODEC_026       = pmsf::kPaperBlobLayoutCodec026,
        CODEC_027         = pmsf::kPaperBlobLayoutCodec027,
        CODEC_028       = pmsf::kPaperBlobLayoutCodec028,
        /// VQMMC paper-alignment strict path:
        ///   ConnectivityBlobContainer stream 0 = BFS facet/isSplittable bits,
        ///   stream 1 = BFS edge/isAdded bits,
        ///   stream 2 = range-coded VQ codebook indices,
        ///   stream 3 = range-coded VQ codebook payload.
        /// This is a transitional strict-storage layout; the full paper
        /// reproduction plan replaces the BFS streams with collapse/refinement
        /// operation streams.
        CODEC_030        = pmsf::kPaperBlobLayoutCodec030,
        /// Segmented PPMC native payload:
        ///   [groupCount][roundCount][groupStart[G]][roundEnd[G][R]][payload]
        /// where each group payload is the normal PPMCSymbolCollectOperator
        /// RC stream, with per-group/per-round byte offsets for random access.
        CODEC_031   = pmsf::kPaperBlobLayoutCodec031,
        /// Segmented default/optimized payload:
        ///   [groupCount][groupStart[G+1]][payload]
        /// where each payload chunk is one OptimizedSymbolCollectOperator
        /// RANGE_CODED_PRED blob for a group.
        SEGMENTED_RANGE_CODED_PRED =
            pmsf::kPaperBlobLayoutSegmentedRangeCodedPred,
    };

  private:
    // ========== Schema 构建 ==========

    /// @brief 构建 mesh_metadata 表的 Schema
    std::shared_ptr<arrow::Schema> buildMetadataSchema();

    /// @brief 构建 group_base_mesh 表的 Schema
    std::shared_ptr<arrow::Schema> buildBaseMeshSchema();

    /// @brief 构建 boundary_graph 表的 Schema
    std::shared_ptr<arrow::Schema> buildBoundarySchema();

    /// @brief 构建 Parquet Writer Properties（编码+压缩配置）
    std::shared_ptr<parquet::WriterProperties> buildWriterProperties();

    /// @brief 构建 Arrow Writer Properties
    std::shared_ptr<parquet::ArrowWriterProperties> buildArrowWriterProperties();

    // ========== 数据构建 ==========

    /// @brief 构建 mesh_metadata 表
    std::shared_ptr<arrow::Table> buildMetadataTable();

    /// @brief 构建 group_base_mesh 表
    std::shared_ptr<arrow::Table> buildBaseMeshTable();

    /// @brief 构建 group_round_symbols 表
    std::shared_ptr<arrow::Table> buildSymbolsTable();

    /// @brief 构建 boundary_graph 表
    std::shared_ptr<arrow::Table> buildBoundaryTable();

    // ========== 写入方法 ==========

    /// @brief 写入单个 Parquet 文件
    void writeParquetFile(const std::string& path,
                          std::shared_ptr<arrow::Schema> schema,
                          std::shared_ptr<arrow::Table> table,
                          int rowGroupSize = -1);

  protected:
    // ========== 成员变量（protected: 供 Unified 子类读访问） ==========

    /// @brief 缓存的基础网格数据（在 serializeBaseMeshWithSeed 中填充）。
    /// v3-refactor 阶段 D：V3 子类读取它生成 `<base>_v3_compact.bin` 替代
    /// `_v3_metadata.parquet` + `_v3_base_mesh.parquet`，剔除 Parquet 固定 overhead。
    struct GroupBaseMeshData {
        int groupId;
        // seed 顶点坐标
        float seedV0X, seedV0Y, seedV0Z;
        float seedV1X, seedV1Y, seedV1Z;
        // 量化 seed 坐标
        int32_t seedV0QX, seedV0QY, seedV0QZ;
        int32_t seedV1QX, seedV1QY, seedV1QZ;
        int32_t seedV0CoordId = -1;
        int32_t seedV1CoordId = -1;
        // 非 seed 顶点
        std::vector<float> vertexX, vertexY, vertexZ;
        std::vector<int32_t> vertexQX, vertexQY, vertexQZ;
        std::vector<int32_t> vertexSharedPositions;
        std::vector<int32_t> vertexCoordIds;
        bool faceVerticesAreCoordIds = false;
        int numVertices;
        // 面片数据（支持多边形面片）
        std::vector<int32_t> faceVertices;  // 所有面片顶点 ID 的扁平列表
        std::vector<int32_t> faceDegrees;   // 每个面片的度数
        int numFaces;
    };

    std::vector<GroupBaseMeshData> groupBaseMeshData_;
    bool sharedBaseMeshCoordinates_ = false;
    bool sharedBaseMeshFaceCoordIds_ = false;
    bool partitionedCompactBaseMesh_ = false;
    std::vector<float> sharedVertexX_, sharedVertexY_, sharedVertexZ_;
    std::vector<int32_t> sharedVertexQX_, sharedVertexQY_, sharedVertexQZ_;
    std::vector<std::uint8_t> lastSymbolsBlob_;
    int totalVertices_ = 0;
    int totalFaces_ = 0;
    // 压缩前原始网格规模（setOriginalMeshCounts 注入；<=0 表示未设置）。
    int originalInputVertices_ = 0;
    int originalInputFaces_ = 0;

  private:

    /// @brief 分区边界图
    std::shared_ptr<Graph> boundaryGraph_;

  protected:

    /// @brief codec 版本。
    ///          1 = 旧 bool-list 多列 symbols 表格式；
    ///          2 = BPV 优化：symbols 表压成单列 `rc_blob`，per-halfedge edge 符号；
    ///          3 = per-edge edge 符号（每条无向 edge 只发 1 bit，首次 pop 消费；
    ///              编/解码端靠共享 `hOpp->isVisited(current_version)` 分类
    ///              "首次 / 二次 pop"，bit 数约为 v2 的一半）。
    ///          4 = 列式段表：bit(Connectivity) + residual(Geometry) + lod(偏移) 三表。
    /// 默认 4；外部需要手动回落时调用 setCodecVersion(2/3)。
    static constexpr int CODEC_COLUMNAR_SEG = 4;
    int codecVersion_ = 4;

    /// @brief 选定的 blob layout（serialize 时根据 collector 类型 + prediction 自动决定）
    BlobLayout blobLayout_ = BlobLayout::UNKNOWN;

    /// @brief When blobLayout_ is set to kPaperBlobLayoutCanonical, this records
    /// the original algorithm-native layout so the decoder can pick the right
    /// canonical spec and reassembly adapter.
    std::int32_t canonicalSourceLayout_ = 0;

    /// @brief Tier 2：V2 紧凑模式标志。serialize() 末尾根据它决定是否写
    /// `<base>_compact.bin` 并删除两张小 parquet。默认 false 保持向后兼容。
    bool v2CompactMode_ = false;

    /// @brief Tier 2：把 PMSFHeader + groupBaseMeshData_ + codec/blobLayout 等
    /// 元信息打包进 zlib-deflated 单文件。布局与 V3 compact 一致，magic 为 "V2CP"。
    bool buildV2CompactBinPayload(std::vector<std::uint8_t>& compactBin);

    /// @brief Tier 2：把 PMSFHeader + groupBaseMeshData_ + codec/blobLayout 等
    /// 元信息打包进 zlib-deflated 单文件。布局与 V3 compact 一致，magic 为 "V2CP"。
    /// @return 成功 true；任何失败 false（serialize() 不删 parquet 表）。
    bool writeV2CompactBin(const std::string& path);

    /// @brief 写入“全局坐标 block + per-partition base_blob”的分区紧凑格式。
    bool writePartitionedCompactBaseMesh(const std::string& baseName);

    // ========== BPV 优化路径：symbols 表 rc_blob 列 ==========

    /// @brief 构建 BPV 优化版本的 symbols schema：
    ///        group_id INT32 + rc_blob BINARY
    std::shared_ptr<arrow::Schema> buildSymbolsSchemaV2();

    /// @brief 使用非 Parquet 版本的 collector->exportToBuffer*() 生成熵编码后的
    ///        符号比特流，并以单行（每 group 一行）写入 BINARY 列。
    /// @details 这是 BPV 优化的核心：直接复用与 BasicSerializeOperator::serialize
    ///          一致的 range-coder / bit-plane / 上下文自适应模型等熵编码路径，
    ///          从而让 Parquet V2 的 BPV 与非 Parquet 版本保持在同一量级。
    std::shared_ptr<arrow::Table> buildSymbolsTableV2();

    // ========== codec=4 列式段表 ==========

    struct ColumnarSegment {
        int streamId = 0;
        int encodingMode = 0;  // 0=entropy bytes, 1=native column encoding
        std::vector<std::uint8_t> bytes;
    };
    struct ColumnarSegments {
        std::vector<ColumnarSegment> bitSegments;
        std::vector<std::uint8_t> residualBytes;
        int residualEncodingMode = 0;
        std::vector<std::uint8_t> headerBytes;  // original Class I/III header (sizes + counts)
    };

    std::shared_ptr<arrow::Schema> buildBitSchemaV4();
    std::shared_ptr<arrow::Schema> buildResidualSchemaV4();
    std::shared_ptr<arrow::Schema> buildLodSchemaV4();
    std::shared_ptr<arrow::Schema> buildSideSchemaV4();

    std::shared_ptr<arrow::Table> buildBitTableV4(const ColumnarSegments& segs);
    std::shared_ptr<arrow::Table> buildResidualTableV4(const ColumnarSegments& segs);
    std::shared_ptr<arrow::Table> buildLodTableV4(const ColumnarSegments& segs,
                                                  int bitTotalBytes,
                                                  int residualTotalBytes);

    /// @brief Split the entropy-encoded rc_blob into connectivity (bit) and
    ///        geometry (residual) segments according to BlobLayout.
    ///        Phase A2: encoding_mode always 0; bytes are verbatim copies.
    ColumnarSegments splitBlobToSegmentsV4(const char* blob,
                                           int blobSize,
                                           BlobLayout layout);

    // ========== Phase A3: per-stream min-size + native column candidate ==========

    /// @brief Raw symbols/points decoded from a Class I blob.
    struct ClassIDecodedStreams {
        std::vector<char> facetSymbols;
        std::vector<char> edgeSymbols;
        std::vector<char> vertexSymbols;
        std::vector<MCGAL::Point> points;
        std::vector<MCGAL::PointInt> residuals;
        std::vector<MCGAL::Point> vertexPoints;
        bool hasVertex = false;
        bool useResidual = false;
    };

    /// @brief Per-stream encoding decision for Class I.
    struct ClassIEncodingDecision {
        std::vector<int8_t> bitModes;  // 0 = entropy bytes, 1 = native Parquet column
        int8_t residualMode = 0;
        // diagnostic sizes
        std::vector<int> bitEntropySizes;
        std::vector<int> bitNativeSizes;
        int residualEntropySize = 0;
        int residualNativeSize = 0;
    };

    /// @brief Decode a Class I blob back to raw symbols/points using the
    ///        existing OptimizedSymbolReadOperator. Used by A3 to measure
    ///        Parquet native column sizes without reimplementing entropy decoding.
    ClassIDecodedStreams decodeClassIBlobToRawStreams(const char* blob,
                                                      int blobSize,
                                                      BlobLayout layout);

    /// @brief Measure compressed bytes of a single boolean stream stored as
    ///        Parquet list<bool> with RLE/bit-packing (paper Table 2 Boolean).
    int measureNativeBoolStreamSize(const std::vector<char>& symbols);

    /// @brief Measure compressed bytes of a single float geometry stream stored
    ///        as Parquet list<float> with BYTE_STREAM_SPLIT (paper Table 2 Float).
    int measureNativeFloatStreamSize(const std::vector<MCGAL::Point>& points);

    /// @brief Measure compressed bytes of a single quantized integer geometry
    ///        stream stored as Parquet list<int32> with DELTA_BINARY_PACKED.
    int measureNativeQuantStreamSize(const std::vector<MCGAL::PointInt>& residuals);

    /// @brief Compare entropy bytes vs native Parquet column sizes for each
    ///        Class I stream and decide encoding_mode per stream.
    ClassIEncodingDecision chooseClassIEncodingModes(
        const ColumnarSegments& segs,
        const ClassIDecodedStreams& decoded);

    /// @brief Re-encode decoded Class I raw streams back to a Class I blob.
    ///        Used by codec=4 A3 fallback when a stream is stored as a native
    ///        Parquet column (encoding_mode=1) and must be converted back to
    ///        entropy bytes for the existing SymbolReadOperator path.
    /// @return number of bytes written, or -1 on failure.
    int reencodeClassIFromRawStreams(const ClassIDecodedStreams& decoded,
                                     BlobLayout layout,
                                     char* outBuffer,
                                     int maxSize);

    /// @brief Serialize a boolean stream as a single-column Parquet table
    ///        (list<bool> with RLE). Returns the raw Parquet file bytes.
    std::vector<std::uint8_t> serializeNativeBoolColumn(
        const std::vector<char>& symbols);

    /// @brief Serialize a float geometry stream as a single-column Parquet table
    ///        (list<float> with BYTE_STREAM_SPLIT+ZSTD). Returns Parquet bytes.
    std::vector<std::uint8_t> serializeNativeFloatColumn(
        const std::vector<MCGAL::Point>& points);

    /// @brief Serialize a quantized integer geometry stream as a single-column
    ///        Parquet table (list<int32> with DELTA_BINARY_PACKED+ZSTD).
    std::vector<std::uint8_t> serializeNativeQuantColumn(
        const std::vector<MCGAL::PointInt>& residuals);
};

#endif
