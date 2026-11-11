/*
 * @Author: koi
 * @Description: Parquet格式反序列化操作符
 * 从Apache Parquet文件读取PMSF压缩数据并解压缩
 */
#ifndef PARQUET_DESERIALIZE_OPERATOR_H
#define PARQUET_DESERIALIZE_OPERATOR_H

#include "DeserializeOperator.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "decompress/SymbolReadOperator.h"
#include <cstddef>
#include <memory>
#include <string>

/**
 * @brief Parquet格式反序列化操作符
 * @details 从Parquet文件中读取total_binary_data列，还原为原始buffer格式，
 * 然后复用现有的反序列化逻辑进行解压缩。
 */
class ParquetDeserializeOperator : public DeserializeOperator {
  public:
    ParquetDeserializeOperator(std::shared_ptr<SymbolReadOperator> symbolReadOperator,
                               std::shared_ptr<ReconstructOperator> reconstructOperator)
        : DeserializeOperator(symbolReadOperator, reconstructOperator) {}

    ~ParquetDeserializeOperator() = default;

    /// @brief 从Parquet文件读取数据，还原为内存buffer
    void init(std::string path) override;

    void setSeeds(std::vector<MCGAL::Halfedge*> seeds) override {
        seed = seeds[0];
    }

    std::shared_ptr<MCGAL::Mesh> deserializeBaseMesh() override;

    std::shared_ptr<MCGAL::Mesh> exportMesh() override;

    void deserializeOneRound() override;

    MCGAL::Halfedge* seed = nullptr;
    std::vector<MCGAL::Vertex*> vertices;
    std::vector<MCGAL::Halfedge*> halfedges;
    std::vector<MCGAL::Facet*> facets;
};

#endif
