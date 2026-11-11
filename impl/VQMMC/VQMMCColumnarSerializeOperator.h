/*
 * VQMMCColumnarSerializeOperator
 * --------------------------------
 * VQMMC × Parquet V2 的 column-native 序列化器。
 *
 * 取代旧的 `ColumnarBlobSerializeOperator(inner=BasicSerializeOperator,
 * variant="vqmmc")` 单 BLOB 列方案。新方案产出 4 张关系表：
 *
 *   <basename>_metadata.parquet     1 行
 *       variant="vqmmc", codec_version=3, num_rounds, enable_*,
 *       num_vertices_base, num_faces_base, quant_bits, quant_step,
 *       bbox_min_x/y/z, codebook_size_max, base_mesh_bytes
 *
 *   <basename>_base_mesh.parquet    1 行
 *       seed_v0_q{x,y,z} INT32, seed_v1_q{x,y,z} INT32,
 *       vertex_q{x,y,z}  LIST<INT32>,
 *       face_vertices    LIST<INT32>,
 *       face_degrees     LIST<INT32>,
 *       num_vertices     INT32,
 *       num_faces        INT32,
 *       conn_bits        INT32          # 顶点连接索引位宽 (BasicSerializer 的 connBit)
 *
 *   <basename>_symbols.parquet      2 列 BINARY，每列独立 ZSTD 列压缩
 *       facet_symbols_orig_size INT32   # 未压缩字节
 *       facet_symbols_comp_size INT32   # zlib 后字节（=0 表示未压缩）
 *       facet_symbols           BINARY  # zlib 压缩 / 透传后的位流
 *       edge_symbols_orig_size  INT32
 *       edge_symbols_comp_size  INT32
 *       edge_symbols            BINARY
 *
 *   <basename>_vq.parquet           每轮一行（最旧轮在最前，与 collect/decode
 *                                    端 reverse-iterate 顺序对齐）
 *       round_id          INT32
 *       codebook_size     INT32
 *       num_indices       INT32
 *       cb_quant_bits     INT32
 *       cb_min_x/y/z      FLOAT
 *       cb_step_x/y/z     FLOAT
 *       cb_quantized_x    LIST<INT32>   # 每码字 X 维量化值（cb_quant_bits 位）
 *       cb_quantized_y    LIST<INT32>
 *       cb_quantized_z    LIST<INT32>
 *       idx_bits          INT32
 *       indices           LIST<INT32>   # BFS 顺序的码本索引
 *
 * 设计决策：
 *   1. 几何符号（facet/edge）保留 zlib 段：与 BasicSerializeOperator 完全
 *      字节级等价，BPV 与原版一致。Parquet 列再加一层 ZSTD 不会有显著收益。
 *   2. VQ 部分**完全列式拆解**——码本量化、索引、bit width 全是结构化数据，
 *      论文 [Lavoue 2009] §4 的"per-round codebook + indices"语义一一对应。
 *      列存友好，便于 SQL 选择性投影（例如只读某一轮码本做可视化）。
 *   3. base_mesh 与 metadata 单独成表，与 PPMC Columnar 同构，便于跨算法
 *      复用 SQL 工具链与 schema 演进。
 *   4. 设置 PMSF_UNIFIED_STORAGE=1 时，facet/edge symbols、codebook/index 以及
 *      显式 unified-only 所需的 metadata/base_mesh 可写入统一 paper payload
 *      relation；可用 `PMSF_VQMMC_SKIP_COMPAT_TABLES=1` 一次性跳过全部
 *      VQMMC 兼容表，或用
 *      PMSF_VQMMC_SKIP_COMPAT_SYMBOLS_TABLE=1 /
 *      PMSF_VQMMC_SKIP_COMPAT_VQ_TABLE=1 /
 *      PMSF_VQMMC_SKIP_COMPAT_BASE_TABLES=1 分别跳过兼容 `_symbols.parquet`、
 *      `_vq.parquet` 和 `_metadata.parquet`/`_base_mesh.parquet`。默认不写
 *      metadata/base_mesh payload，避免未请求 unified-only 时额外增加 BPV。
 */
#pragma once

#include "compress/SerializeOperator.h"
#include "compress/SymbolCollectOperator.h"

#include <memory>
#include <string>

class VQMMCColumnarSerializeOperator : public SerializeOperator {
  public:
    VQMMCColumnarSerializeOperator(std::shared_ptr<SymbolCollectOperator> collector,
                                    bool enablePrediction = false,
                                    bool enableQuantization = true,
                                    bool enableCompress = false)
        : SerializeOperator(collector, enablePrediction, enableQuantization, enableCompress) {}

    ~VQMMCColumnarSerializeOperator() override = default;

    // ---- SerializeOperator 标准接口 ----
    void init() override;
    void serializeInt(int val) override;
    void serializeBaseMesh(std::shared_ptr<MCGAL::Mesh> mesh) override;
    void serializeBaseMeshWithSeed(std::shared_ptr<MCGAL::Mesh> mesh,
                                   std::vector<MCGAL::Halfedge*> seeds) override;
    /// 真正的 4 表写出在此完成。`path` 期望以 `.parquet` 结尾，会被去后缀做 basename。
    void serialize(std::string path) override;

  private:
    // base mesh 写入（在 serializeBaseMeshWithSeed 阶段填充）
    struct BaseMeshSnapshot {
        bool valid = false;
        int  numVertices = 0;
        int  numFaces = 0;
        int  connBits = 0;
        // 量化坐标（quant 模式下使用 — 与论文实现一致默认开启）
        int  seedV0Q[3]{};
        int  seedV1Q[3]{};
        std::vector<int> vertexQX, vertexQY, vertexQZ;
        std::vector<int> faceVertices;   // flatten
        std::vector<int> faceDegrees;
        // 量化 metadata
        int   nbQuantBits = 0;
        float bboxMin[3]{};
        float quantStep = 0.f;
    };
    BaseMeshSnapshot baseMesh_;
};
