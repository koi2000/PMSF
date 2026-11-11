/*
 * POMAR column-native Parquet V2 序列化算子
 *
 * 目标：取代 `ColumnarBlobSerializeOperator("pomar")` 的单 `payload` BLOB 方案，
 *      把 `PomarColumnarSerializeOperator` 生成的 inner buffer 按语义段拆成
 *      4 张独立 parquet 表，实现"column-native 关系存储"：
 *
 *   _metadata.parquet   1 行，纯标量列：num_rounds / enable_*（5 个） /
 *                       num_clusters / nb_global_lod / nb_clustered_lod /
 *                       cluster0_size / cluster_meta_size / clusters_rest_size /
 *                       base_mesh_size。
 *   _base_mesh.parquet  1 行 1 列 `base_mesh_blob`（BINARY）——
 *                       POMAR 的 base mesh 是位打包混合段，单独拆成多个
 *                       parquet 列反而显著膨胀，故保留为独立语义段。
 *   _cluster0.parquet   1 行 1 列 `cluster0_rc_blob`（BINARY）——
 *                       全局 LOD 的 range-coded 熵流。
 *   _clusters.parquet   N-1 行（若 random-access 开启），列：
 *                       `cluster_id`（INT32）/ `cluster_size`（INT32）/
 *                       `rc_blob`（BINARY）——每 cluster 一行，
 *                       支持按需投影读取单个 cluster。
 *                       另附一行 id=-1 `cluster_meta_blob` 存 writeClusterSizes 的
 *                       bit-packed 元信息（含 nb_bits_* / minMinResidual 等）。
 *
 * Round-trip 契约：反序列化端把四张表重新拼成与 inner buffer 逐字节等价的
 * 数据流，喂给已有的 `PomarColumnarDeserializeOperator`，reader / reconstruct 完全
 * 不动，保证几何结果与 binary 后端字节级一致。
 */

#pragma once

#include "POMAR/compress/PomarColumnarSerializeOperator.h"

#include <memory>
#include <string>

class POMARColumnarSerializeOperator : public PomarColumnarSerializeOperator {
  public:
    POMARColumnarSerializeOperator(std::shared_ptr<SymbolCollectOperator> collector,
                                    bool enablePrediction,
                                    bool enableQuantization,
                                    bool enableCompress)
        : PomarColumnarSerializeOperator(collector, enablePrediction, enableQuantization, enableCompress) {}

    ~POMARColumnarSerializeOperator() override = default;

    // 覆盖 serialize：先让父类按常规流程把数据写进 buffer_（但不写文件），
    // 然后把 buffer_ 按段拆成 4 张 parquet 文件。
    void serialize(std::string path) override;

    void setPaperStream(bool enabled) { paperStream_ = enabled; }

    /// @brief 子类（V3）可调用：在 V2 serialize 完成后**阻止** buffer_ 被释放，
    ///        以便 V3 再读 rawBuffer()/rawSegmentOffsets 写 `_v3_blob.parquet`。
    ///        调用方负责在最后 `releaseBufferNow()`。默认 false，保持 V2 自释放语义。
    void setKeepBufferAfterSerialize(bool v) { keepBuffer_ = v; }
    /// @brief 释放由 setKeepBufferAfterSerialize(true) 推迟的 buffer_。
    void releaseBufferNow();

  protected:
    bool keepBuffer_ = false;
    bool paperStream_ = false;
};
