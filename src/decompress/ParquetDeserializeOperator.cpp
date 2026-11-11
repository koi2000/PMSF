/*
 * @Author: koi
 * @Description: Parquet格式反序列化操作符实现
 */
#include "ParquetDeserializeOperator.h"
#include "common/BoundedRingTraversal.h"
#include "../common/DebugTool.h"
#include "BufferUtils.h"
#include "Configuration.h"
#include "Global.h"
#include "Halfedge.h"
#include "MeshUtils.h"
#include "Vertex.h"
#include "storage/PMSFSymbolStorageAdapter.h"
#include <cstddef>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/reader.h>

void ParquetDeserializeOperator::init(std::string path) {
    clearPMSFSymbolContext();

    // 从Parquet文件读取total_binary_data列，还原为原始buffer
    auto result = arrow::io::ReadableFile::Open(path);
    if (!result.ok()) {
        if (restoreLegacySingleFileBufferFromPMSFPayload(path)) {
            std::cout << "[ParquetDeserialize] restored from unified paper payload: "
                      << path << std::endl;
            return;
        }
        std::cerr << "无法打开Parquet文件: " << path << " 错误: " << result.status().ToString() << std::endl;
        return;
    }
    auto infile = result.ValueOrDie();

    std::unique_ptr<parquet::arrow::FileReader> reader;
    auto st = parquet::arrow::OpenFile(infile, arrow::default_memory_pool()).Value(&reader);
    if (!st.ok()) {
        std::cerr << "无法解析Parquet文件: " << st.ToString() << std::endl;
        return;
    }

    // 读取整个表
    std::shared_ptr<arrow::Table> table;
    st = reader->ReadTable(&table);
    if (!st.ok()) {
        std::cerr << "读取Parquet表失败: " << st.ToString() << std::endl;
        return;
    }

    std::cout << "[ParquetDeserialize] 读取Parquet文件: " << path << std::endl;
    std::cout << "  - 行数: " << table->num_rows() << std::endl;
    std::cout << "  - 列数: " << table->num_columns() << std::endl;

    // 打印元数据
    auto metadata = table->schema()->metadata();
    if (metadata) {
        std::cout << "  - 元数据:" << std::endl;
        for (int i = 0; i < metadata->size(); i++) {
            std::cout << "    " << metadata->key(i) << " = " << metadata->value(i) << std::endl;
        }
    }

    // 读取total_binary_data列（第3列，索引3）
    auto totalBinaryCol = table->column(3);  // total_binary_data
    auto chunk = std::static_pointer_cast<arrow::BinaryArray>(totalBinaryCol->chunk(0));
    int32_t len = 0;
    const uint8_t* data = chunk->GetValue(0, &len);

    // 复制到buffer_
    buffer_ = new char[len];
    std::memcpy(buffer_, data, len);
    totalSize = len;

    // 打印各区域大小信息
    auto numRoundsCol = table->column(4);  // num_rounds
    auto headerSizeCol = table->column(8); // header_size
    auto baseMeshSizeCol = table->column(9); // base_mesh_size
    auto symbolSizeCol = table->column(10); // symbol_data_size

    auto nrChunk = std::static_pointer_cast<arrow::Int32Array>(numRoundsCol->chunk(0));
    auto hsChunk = std::static_pointer_cast<arrow::Int32Array>(headerSizeCol->chunk(0));
    auto bsChunk = std::static_pointer_cast<arrow::Int32Array>(baseMeshSizeCol->chunk(0));
    auto ssChunk = std::static_pointer_cast<arrow::Int32Array>(symbolSizeCol->chunk(0));

    std::cout << "  - Rounds: " << nrChunk->Value(0) << std::endl;
    std::cout << "  - Header: " << hsChunk->Value(0) << " bytes" << std::endl;
    std::cout << "  - BaseMesh: " << bsChunk->Value(0) << " bytes" << std::endl;
    std::cout << "  - Symbol: " << ssChunk->Value(0) << " bytes" << std::endl;
    std::cout << "  - Total: " << len << " bytes" << std::endl;

    loadPMSFSymbolSidecarIfPresent(path);
}

std::shared_ptr<MCGAL::Mesh> ParquetDeserializeOperator::deserializeBaseMesh() {
    // 复用OptimizedBasicDeserializeOperator的逻辑
    std::shared_ptr<MCGAL::Mesh> mesh;
    if (header_.isEnableQuantization()) {
        const size_t quantMetadataOffset = dataOffset_;
        mesh = MCGAL::readBaseMeshWithQuantization(DEFAULT_MESH_ID, buffer_, dataOffset_);
        if (mesh) {
            mesh->i_nbQuantBits =
                static_cast<unsigned char>(buffer_[quantMetadataOffset]);
        }
    } else {
        mesh = MCGAL::readBaseMesh(DEFAULT_MESH_ID, buffer_, dataOffset_);
    }

    // 查找seed半边
    MCGAL::Vertex* v0 = mesh->vertices()[0];
    MCGAL::Vertex* v1 = mesh->vertices()[1];
    for (MCGAL::Halfedge* hit : pmsf::boundedRing(v0)) {
        if (hit->end_vertex() == v1) {
            seed = hit;
            break;
        }
    }
    this->mesh_ = mesh;
    reconstructOperator_->init(mesh);

    // 读取符号区域的大小信息
    int facetSymbolSize = readInt(buffer_, dataOffset_);
    int edgeSymbolSize = readInt(buffer_, dataOffset_);
    int vertexSymbolSize = 0;
    if (header_.isEnableAdjustment()) {
        vertexSymbolSize = readInt(buffer_, dataOffset_);
    }

    pointOffset_ = facetSymbolSize + edgeSymbolSize + vertexSymbolSize + dataOffset_;
    size_t offset1 = dataOffset_ + facetSymbolSize;
    size_t offset2 = offset1 + edgeSymbolSize;
    size_t offset3 = offset2;
    if (header_.isEnableAdjustment()) {
        offset3 = offset3 + vertexSymbolSize;
    }
    symbolReadOperator_->init({offset1, offset2, offset3}, header_);

    if (header_.isEnableCompress()) {
        char* facetSymbolBuffer = new char[BUFFER_SIZE];
        char* edgeSymbolBuffer = new char[BUFFER_SIZE];
        char* vertexSymbolBuffer = new char[BUFFER_SIZE];
        char* pointBuffer = new char[BUFFER_SIZE];
        int realFacetSymbolSize = 0;
        int realEdgeSymbolSize = 0;
        int realVertexSymbolSize = 0;
        int realPointSize = 0;
        deserializeCharPointer(buffer_, dataOffset_, facetSymbolBuffer, realFacetSymbolSize);
        deserializeCharPointer(buffer_, offset1, edgeSymbolBuffer, realEdgeSymbolSize);
        if (header_.isEnableAdjustment()) {
            deserializeCharPointer(buffer_, offset2, vertexSymbolBuffer, realVertexSymbolSize);
        }
        deserializeCharPointer(buffer_, offset3, pointBuffer, realPointSize);
        delete[] buffer_;
        char* newBuffer = new char[realFacetSymbolSize + realEdgeSymbolSize + realVertexSymbolSize + realPointSize];
        std::memcpy(newBuffer, facetSymbolBuffer, realFacetSymbolSize);
        std::memcpy(newBuffer + realFacetSymbolSize, edgeSymbolBuffer, realEdgeSymbolSize);
        if (header_.isEnableAdjustment()) {
            std::memcpy(newBuffer + realFacetSymbolSize + realEdgeSymbolSize, vertexSymbolBuffer, realVertexSymbolSize);
        }
        std::memcpy(newBuffer + realFacetSymbolSize + realEdgeSymbolSize + realVertexSymbolSize, pointBuffer, realPointSize);
        offset1 = realFacetSymbolSize;
        offset2 = offset1 + realEdgeSymbolSize;
        offset3 = offset2 + realVertexSymbolSize;
        delete[] facetSymbolBuffer;
        delete[] edgeSymbolBuffer;
        delete[] vertexSymbolBuffer;
        delete[] pointBuffer;
        buffer_ = newBuffer;
        dataOffset_ = 0;
        symbolReadOperator_->init({offset1, offset2, offset3}, header_);
    }
    return mesh;
}

void ParquetDeserializeOperator::deserializeOneRound() {
    vertices.clear();
    halfedges.clear();
    facets.clear();
    if (header_.isEnableQuantization()) {
        symbolReadOperator_->enableQuantization(mesh_->bboxMin, mesh_->f_quantStep);
    }
    symbolReadOperator_->collect(seed, buffer_, dataOffset_, vertices, halfedges, facets);
    reconstructOperator_->reconstruct(vertices, halfedges, facets);
}

std::shared_ptr<MCGAL::Mesh> ParquetDeserializeOperator::exportMesh() {
    return mesh_;
}
