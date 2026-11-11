/*
 * @Author: koi
 * @Description: Parquet格式序列化操作符
 * 将PMSF压缩数据导出为Apache Parquet列式存储格式
 */
#ifndef PARQUET_SERIALIZEOPERATOR_H
#define PARQUET_SERIALIZEOPERATOR_H

#include "SerializeOperator.h"
#include <string>

/**
 * @brief Parquet格式序列化操作符
 * @details 将PMSF压缩后的数据（Header、BaseMesh、Symbol）导出为Parquet列式存储格式。
 * Parquet文件的Schema设计：
 *   - header_data: BYTE_ARRAY  (序列化的PMSFHeader二进制数据)
 *   - base_mesh_data: BYTE_ARRAY  (序列化的BaseMesh二进制数据)
 *   - symbol_data: BYTE_ARRAY  (序列化的符号+点数据二进制数据)
 *   - num_vertices: INT32  (基础网格顶点数，用于元数据查询)
 *   - num_faces: INT32  (基础网格面片数，用于元数据查询)
 *   - num_rounds: INT32  (压缩轮次数，用于元数据查询)
 *   - enable_quantization: BOOLEAN  (是否启用量化)
 *   - enable_compress: BOOLEAN  (是否启用zlib压缩)
 *
 * 这种设计保持了与现有二进制格式的完全兼容性，
 * 同时利用Parquet的列式存储和内置压缩（Snappy/Zstd）提供额外的压缩收益。
 * 设置 `PMSF_UNIFIED_STORAGE=1 PMSF_LEGACY_SINGLEFILE_SKIP_COMPAT_FILE=1`
 * 时，可跳过主 Parquet 文件，仅把 total binary buffer 写入统一 paper payload
 * `pmsf_legacy_binary_buffer`；默认不写该重复 payload，避免 BPV 上升。
 */
class ParquetSerializeOperator : public SerializeOperator {
  public:
    ParquetSerializeOperator(std::shared_ptr<SymbolCollectOperator> collector,
                             bool enablePrediction = false,
                             bool enableQuantization = false,
                             bool enableCompress = false)
        : SerializeOperator(collector, enablePrediction, enableQuantization, enableCompress) {}

    void init() override;

    void serializeInt(int val) override;

    void serializeBaseMesh(std::shared_ptr<MCGAL::Mesh> mesh) override;

    void serializeBaseMeshWithSeed(std::shared_ptr<MCGAL::Mesh> mesh, std::vector<MCGAL::Halfedge*> seed) override;

    void serialize(std::string path) override;

  private:
    /// @brief 记录BaseMesh序列化后的偏移量，用于分割header/basemesh/symbol区域
    size_t baseMeshStartOffset_ = 0;
    size_t baseMeshEndOffset_ = 0;
};
#endif
