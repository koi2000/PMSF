/*
 * @Author: mig-v2
 * @Description: Parquet V2 通用 BLOB 包装反序列化器
 *
 * 与 ColumnarBlobSerializeOperator 对偶：
 *   1. 读取 <basename>_metadata.parquet 提取 variant / header 元数据；
 *   2. 读取 <basename>_payload.parquet 提取内部 serializer 原生二进制；
 *   3. 把 payload 写入一个临时 .bin，再转交内部 DeserializeOperator 的
 *      init(path) / 对应 deserialize 方法，实现完整解压。
 */
#ifndef COLUMNAR_BLOB_DESERIALIZE_OPERATOR_H
#define COLUMNAR_BLOB_DESERIALIZE_OPERATOR_H

#include "DeserializeOperator.h"
#include <memory>
#include <string>

class ColumnarBlobDeserializeOperator : public DeserializeOperator {
  public:
    /// @param inner 内部真正的 DeserializeOperator
    /// @param expectedVariant 期望的 variant；若不一致会在 init 时警告
    ColumnarBlobDeserializeOperator(std::shared_ptr<DeserializeOperator> inner,
                                      std::string expectedVariant)
        : DeserializeOperator(
              inner ? inner->symbolReadOperator_ : nullptr,
              inner ? inner->reconstructOperator_ : nullptr),
          inner_(inner), expectedVariant_(std::move(expectedVariant)) {}

    ~ColumnarBlobDeserializeOperator() = default;

    /// 从 Parquet V2 Blob 形式读取
    void init(std::string path) override;

    void setSeeds(std::vector<MCGAL::Halfedge*> seeds) override {
        seeds_ = seeds;
        if (inner_) inner_->setSeeds(seeds);
    }

    void deserializeHeader(PMSFHeader& header) override {
        // 元数据已在 init 中读出；若 inner_ 依赖 buffer 流里的 header，则再调用
        if (inner_) inner_->deserializeHeader(header);
        header_ = header;
    }

    std::shared_ptr<MCGAL::Mesh> deserializeBaseMesh() override {
        if (!inner_) return nullptr;
        auto m = inner_->deserializeBaseMesh();
        mesh_ = m;
        return m;
    }

    void deserializeOneRound() override {
        if (inner_) inner_->deserializeOneRound();
    }

    std::shared_ptr<MCGAL::Mesh> exportMesh() override {
        if (inner_) return inner_->exportMesh();
        return mesh_;
    }

    PMSFHeader header() override {
        if (inner_) return inner_->header();
        return header_;
    }

    // 解码池容量估计：metadata 在 init 中已读，直接返回原始网格计数。
    void originalMeshCounts(int& v, int& f) const override {
        if (origTotalVertices_ > 0 || origTotalFaces_ > 0) {
            v = origTotalVertices_;
            f = origTotalFaces_;
            return;
        }
        if (inner_) inner_->originalMeshCounts(v, f);
    }

  private:
    std::shared_ptr<DeserializeOperator> inner_;
    std::string expectedVariant_;
    std::string tempBinaryPath_;  // init 过程中产生的临时 bin 文件
    int origTotalVertices_ = 0;     // metadata.total_vertices（init 读取）
    int origTotalFaces_ = 0;        // metadata.total_faces
};

#endif
