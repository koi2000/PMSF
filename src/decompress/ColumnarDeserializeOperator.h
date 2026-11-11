/*
 * @Author: koi
 * @Description: Parquet V2 多表方案反序列化操作符
 *
 * 从 4 个独立 Parquet 文件中读取结构化数据：
 *   - <basename>_metadata.parquet
 *   - <basename>_base_mesh.parquet
 *   - <basename>_symbols.parquet
 *   - <basename>_boundary.parquet
 *
 * 支持：
 *   - 按 group_id 随机访问特定分区（Row Group 级别跳过）
 *   - 按 round_id 随机访问特定 LOD（行级过滤）
 */
#ifndef COLUMNAR_DESERIALIZE_OPERATOR_H
#define COLUMNAR_DESERIALIZE_OPERATOR_H

#include "DeserializeOperator.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "OptimizedSymbolReadOperator.h"
#include "decompress/SymbolReadOperator.h"
#include "../common/Graph.h"
// PHASE60 物品 3：分区（layout-32）产物的后端无关解码镜像。列式与二进制共用；
// 本类的 `paperBlobLayoutIsSegmentedRangeCodedPred` 分支只负责转发。
#include "SegmentedRangeCodedDecode.h"
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <arrow/api.h>
#include <arrow/compute/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/reader.h>

class ColumnarDeserializeOperator : public DeserializeOperator {
  public:
    ColumnarDeserializeOperator(std::shared_ptr<SymbolReadOperator> symbolReadOperator,
                                 std::shared_ptr<ReconstructOperator> reconstructOperator)
        : DeserializeOperator(symbolReadOperator, reconstructOperator) {}

    ~ColumnarDeserializeOperator() = default;

    /// @brief 从 Parquet V2 多文件读取数据
    void init(std::string path) override;

    /// M2c native paper path: bind the already-created mesh into the PPMC reader
    /// after paper-table symbols are attached (the generic init binding may run
    /// before the reader exists in this path).
    void bindPPMCReaderMesh();

    /// @brief M4-4 native PaperStream v2 decode seeding: parse a v2 compact base
    ///        payload (V2CP envelope: header/base mesh/codec/blobLayout/quant/bbox)
    ///        and hand the algorithm-native symbol blob (raw, already unwrapped)
    ///        straight to the codec-4 import path. Performs NO legacy Parquet /
    ///        side-table reads. Call deserializeBaseMesh() afterwards (it builds
    ///        the base mesh and imports the native blob), then deserializeOneRound().
    /// @return false with `error` on malformed input (fail closed).
    bool initPaperV2Native(const std::string& compactBinPath,
                           std::vector<char> nativeBlob,
                           std::string* error = nullptr);

    /// M4-4 typed PaperStream v2 decode seeding: parse the v2 compact base payload
    /// and hand a pre-decoded PMSFSymbolTable to the per-round typed symbol driver.
    bool initPaperV2Typed(const std::string& compactBinPath,
                          pmsf::PMSFSymbolTable symbols,
                          std::string* error = nullptr);

    void setSeeds(std::vector<MCGAL::Halfedge*> seeds) override {
        seeds_ = seeds;
        if (!seeds.empty()) seed_ = seeds[0];
    }

    std::shared_ptr<MCGAL::Mesh> deserializeBaseMesh() override;
    void deserializeOneRound() override;
    std::shared_ptr<MCGAL::Mesh> exportMesh() override;

    std::shared_ptr<Graph> getBoundaryGraph() const { return boundaryGraph_; }

    // ========== 随机访问接口 ==========
    void setTargetGroupId(int groupId) { targetGroupId_ = groupId; }
    void setMaxRounds(int maxRounds) { maxRounds_ = maxRounds; }
    void setMaxGroups(int maxGroups) { maxGroups_ = maxGroups; }

    /// PHASE60 物品 2：**只解一个分区**（G >= 0 生效）。
    ///
    /// 与 `setTargetGroupId` 的区别是**语义作用域**，不是重载：
    /// `setTargetGroupId` 是 (roi, lod) 谓词里的 region 维度，它在 `readBaseMeshTable`
    /// 里是**行级**谓词（`gid != targetGroupId_` 就跳行）。那套行级语义假设
    /// 「基网表按分区拆行」，而分区（codec-4 / `kPaperBlobLayoutSegmentedRangeCodedPred`）
    /// 产物的基网表**恒为单行**（整个基网是各分区共享的一个对象，行内 group_id 为 0）
    /// —— 于是 `setTargetGroupId(1)` 会把唯一的基网行也滤掉，实测直接
    /// 「无基础网格数据」并段错误（本轮 N=4 g=1/2/3 三档）。
    ///
    /// 故单分区解码用**独立**开关：它只收窄 `deserializeOneRound` 的**分区轮循环**
    /// （beginGroup/endGroup），不参与任何行级谓词 —— 基网、段表照常整段读。
    /// 默认 -1 ⇒ 与旧行为逐字节一致。
    void setOnlyGroupId(int groupId) {
        onlyGroupId_ = groupId;
    }
    int onlyGroupId() const {
        return onlyGroupId_;
    }

    // 框架统一的 (roi, lod) 谓词入口（设计文档 §2.5）。
    void setQueryRegion(int regionId) override { setTargetGroupId(regionId); }
    void setQueryMaxLod(int maxLod) override { setMaxRounds(maxLod); }
    int queryMaxLod() const override { return maxRounds_; }
    bool supportsRandomAccess() const override { return true; }

  private:

  protected:
    // ========== 读取各表（protected: 供 Unified 子类在紧凑模式下跳过重写） ==========
    // v3-refactor 阶段 D：V3 reader 在紧凑模式下会改读 `<base>_v3_compact.bin`，
    // 绕过 metadata/base_mesh 两张 parquet 小表；需要在 protected 区域保留入口供
    // V2 调用，也保留入口供 V3 在 compact bin 不存在时 fallback，以及 V3 直接
    // 调用 symbols/boundary 读取。
    void readMetadataTable(const std::string& path);

    // (roi, lod) 谓词的 region 维度是行级过滤（见 .cpp 中的 targetGroupId_ 检查）；
    // lod 维度由解码主循环按 queryMaxLod() 截断轮数落实，**不是**行级谓词 ——
    // 三张段表是整段 blob 的重组输入，裁行会拼出残缺 blob。详见 .cpp 中的长注释。
    bool readEmbeddedMetadataFromPartitionedBase(const std::string& baseName);
    void readBaseMeshTable(const std::string& path);
    bool readPartitionedBaseMeshTables(const std::string& baseName);
    void readSymbolsTable(const std::string& path);
    void readBoundaryTable(const std::string& path);

    /// @brief codec=4 列式段表读取与重组：读 bit/residual/lod 三表，还原出与
    ///        codec=3 等价的 rc_blob 字节流。
    bool readColumnarSymbolsTablesV4(const std::string& baseName);
    bool reassembleBlobFromSegmentsV4(const arrow::Table& bitTable,
                                      const arrow::Table& residualTable,
                                      const arrow::Table* lodTable);

    /// @brief codec=4 A3 fallback: a mode=1 native Parquet column was stored;
    ///        deserialize it back to raw symbols/points and re-entropy-encode.
    struct ReencodeInput {
        std::vector<char> facetSymbols;
        std::vector<char> edgeSymbols;
        std::vector<char> vertexSymbols;
        std::vector<MCGAL::Point> points;
        std::vector<MCGAL::PointInt> residuals;
        bool hasVertex = false;
        bool useResidual = false;
    };
    std::vector<char> deserializeNativeBoolColumn(const std::vector<std::uint8_t>& bytes);
    std::vector<MCGAL::Point> deserializeNativeFloatColumn(const std::vector<std::uint8_t>& bytes);
    std::vector<MCGAL::PointInt> deserializeNativeQuantColumn(const std::vector<std::uint8_t>& bytes);
    int reencodeClassIFromRawStreamsForDecode(const ReencodeInput& input,
                                              char* outBuffer,
                                              int maxSize);

    bool importPMSFSymbolTableAsSymbolData(const pmsf::PMSFSymbolTable& table,
                                            bool replaceExisting = false);

    /// @brief Tier 2 (2026-05-09)：V2 紧凑模式读 `<base>_compact.bin`。
    /// 成功 true（填充 header_/baseMeshData_/codec/blobLayout/quant 等）；
    /// 失败 false（init 会回退到 readMetadataTable + readBaseMeshTable）。
    bool readV2CompactBin(const std::string& path);
    bool readV2CompactBinPayload(const std::vector<std::uint8_t>& compactBin,
                                 const std::string& sourceLabel,
                                 bool quietMagicMismatch = false);
    bool readV2CompactPayloadBytes(const std::vector<std::uint8_t>& payload,
                                   std::uint32_t rawLen,
                                   std::uint32_t compLen,
                                   const std::string& sourceLabel,
                                   bool quietMagicMismatch = false);

  private:
    // ========== Arrow 辅助方法 ==========
    std::vector<float> extractFloatList(std::shared_ptr<arrow::Array> array, int64_t row);
    std::vector<int32_t> extractInt32List(std::shared_ptr<arrow::Array> array, int64_t row);
    std::vector<int16_t> extractInt16List(std::shared_ptr<arrow::Array> array, int64_t row);
    std::vector<bool> extractBoolList(std::shared_ptr<arrow::Array> array, int64_t row);

    std::shared_ptr<arrow::Table> readParquetFile(const std::string& path);

  protected:
    // ========== 缓存的解析数据（protected: 供 V3 紧凑模式直接写入） ==========
    struct GroupBaseMeshData {
        int groupId;
        float seedV0X, seedV0Y, seedV0Z;
        float seedV1X, seedV1Y, seedV1Z;
        int32_t seedV0QX, seedV0QY, seedV0QZ;
        int32_t seedV1QX, seedV1QY, seedV1QZ;
        int32_t seedV0CoordId = -1;
        int32_t seedV1CoordId = -1;
        int numVertices, numFaces;
        std::vector<float> vertexX, vertexY, vertexZ;
        std::vector<int32_t> vertexQX, vertexQY, vertexQZ;
        std::vector<int32_t> vertexSharedPositions;
        std::vector<int32_t> vertexCoordIds;
        bool faceVerticesAreCoordIds = false;
        std::vector<int32_t> faceVertices;  // 所有面片顶点 ID 的扁平列表
        std::vector<int32_t> faceDegrees;   // 每个面片的度数
    };

  private:
    struct GroupSymbolData {
        int groupId;
        int roundId;
        std::vector<bool> facetSymbols;
        std::vector<bool> edgeSymbols;
        std::vector<bool> vertexSymbols;
        std::vector<float> pointX, pointY, pointZ;
        std::vector<int32_t> pointQX, pointQY, pointQZ;
        // 顶点附加点流（QEM/adjustment）
        std::vector<float> vertexPointX, vertexPointY, vertexPointZ;
        std::vector<int32_t> vertexPointQX, vertexPointQY, vertexPointQZ;
    };

  protected:
    std::vector<GroupBaseMeshData> baseMeshData_;
    bool sharedBaseMeshCoordinates_ = false;
    std::vector<float> sharedVertexX_, sharedVertexY_, sharedVertexZ_;
    std::vector<int32_t> sharedVertexQX_, sharedVertexQY_, sharedVertexQZ_;

  private:
    std::vector<GroupSymbolData> symbolData_;
    std::shared_ptr<Graph> boundaryGraph_;

    MCGAL::Halfedge* seed_ = nullptr;
    // P3 对齐：同 PPMCDeserializeOperator，每轮需要从固定顶点对重新查找
    //   v0→v1 的 halfedge，以绕过 reconstruct 对旧 halfedge 指针的失效。
    MCGAL::Vertex* seedV0_ = nullptr;
    MCGAL::Vertex* seedV1_ = nullptr;
    std::vector<MCGAL::Vertex*> vertices_;
    std::vector<MCGAL::Halfedge*> halfedges_;
    std::vector<MCGAL::Facet*> facets_;

    int targetGroupId_ = -1;
    /// 单分区解码开关，见 `setOnlyGroupId` 的说明。只作用于分区轮循环。
    int onlyGroupId_ = -1;
    int maxRounds_ = -1;
    int maxGroups_ = -1;
    int currentRound_ = 0;

  protected:
    // 量化参数（protected: V3 紧凑模式直接写入）
    int quantBits_ = 0;
    float bboxMinX_ = 0, bboxMinY_ = 0, bboxMinZ_ = 0;
    float quantStep_ = 0;

    // ========== V2 BPV 优化路径（protected: V3 紧凑模式直接写入） ==========
    // codec_version == 2 时生效：symbols 表只有一个 rc_blob BINARY 列，
    // 解码时按"非 Parquet 版本"的方式逐轮消费。
    int  codecVersion_ = 1;           // 默认 1：向后兼容旧 bool-list 路径
    bool enablePrediction_ = false;   // 从 metadata 读出；影响使用哪个解码入口
    int  blobLayout_ = 0;             // 从 metadata 读出，对应 ColumnarSerializeOperator::BlobLayout
    int  canonicalSourceLayout_ = 0;  // 当 blobLayout_==kPaperBlobLayoutCanonical 时，记录原始算法 layout
public:
    int  origTotalVertices_ = 0;      // 压缩前原始 mesh 顶点数（从 metadata.total_vertices 读）；
                                      // 用于 PPMC "small-mesh triangle-balance prediction disable" 决策。
    int  origTotalFaces_ = 0;         // 压缩前原始 mesh 面数；用于 data-dependent context pool 估算。

    // 解码池容量按原始网格规模注册（runDecompress 在 deserializeBaseMesh 前调用）。
    void originalMeshCounts(int& v, int& f) const override {
        v = origTotalVertices_;
        f = origTotalFaces_;
    }
private:
    std::vector<char> rcBlob_;        // 从 symbols.parquet 读取的熵编码字节流
    size_t rcBlobOffset_ = 0;         // 逐轮 collect 消费的游标
    bool rcBlobInitialized_ = false;
    bool paperV2Typed_ = false;  // 是否已调用 importFromRangeCodedBuffer
    int segmentedGroupNumber_ = 0;
    int segmentedRoundNumber_ = 0;
    size_t segmentedPayloadStart_ = 0;
    std::vector<size_t> segmentedGroupStart_;
    std::vector<std::vector<size_t>> segmentedGroupRoundEnd_;
    std::vector<int> segmentedGroupCurrentRound_;
    // PHASE60 物品 3：分区产物的解码镜像（前缀解析 + 基网 facet 通道 + 每分区
    // seed + 每组 reader + 轮驱动）。与二进制后端
    // `OptimizedBasicDeserializeOperator` 共用 `impl/common/SegmentedRangeCodedDecode`。
    // 原先散落在本类的 `segmentedOptimized*` 状态随之下沉；`seeds_` /
    // `segmentedSeedVertices_` 仍在 import 后回写一份，供**分段 PPMC** 轮驱动
    // 与外部消费者按绝对 group id 索引。
    pmsf::SegmentedRangeCodedDecode segmentedDecode_;
    std::vector<std::pair<MCGAL::Vertex*, MCGAL::Vertex*>> segmentedSeedVertices_;
};

#endif
