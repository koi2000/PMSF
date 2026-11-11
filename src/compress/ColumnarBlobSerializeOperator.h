/*
 * @Author: mig-v2
 * @Description: Parquet V2 通用 BLOB 包装序列化器
 *
 * 设计思路：
 *   对于 POMAR V2 / VQMMC / 未来任意自定义二进制格式的方法，
 *   其 serialize(path) 产物都是 [int32 len][raw buffer] 的扁平二进制结构。
 *   本类不自行定义 Parquet schema，而是把"内层 serializer 产生的整块
 *   buffer"直接作为 Parquet BLOB 列存储在单行 Parquet 文件里，从而让
 *   这些方法也能获得 Parquet V2 的统一产物布局：
 *     - <basename>_metadata.parquet  （最小化元数据 + variant 标识）
 *     - <basename>_payload.parquet   （单行 BINARY 列 payload）
 *
 *   Parquet 仍然享受 ZSTD 压缩，因此压缩产物大小不会比原二进制文件大。
 *
 * 使用方式：
 *   auto inner = std::make_shared<PomarColumnarSerializeOperator>(collector, ...);
 *   auto outer = std::make_shared<ColumnarBlobSerializeOperator>(
 *                    inner, "pomar");          // variant 名
 *   outer->init();
 *   outer->setHeader(header);
 *   outer->serializeBaseMeshWithSeed(mesh, {seed});
 *   outer->serialize(outputBaseName);
 */
#ifndef COLUMNAR_BLOB_SERIALIZE_OPERATOR_H
#define COLUMNAR_BLOB_SERIALIZE_OPERATOR_H

#include "SerializeOperator.h"
#include <memory>
#include <string>

class ColumnarBlobSerializeOperator : public SerializeOperator {
  public:
    /// @param inner 内部真正完成序列化的 Operator（产出 [int len][buf] 文件）
    /// @param variant 变体名（写入 metadata 的 variant 列，例如 "pomar" / "vqmmc"）
    ColumnarBlobSerializeOperator(std::shared_ptr<SerializeOperator> inner,
                                    std::string variant)
        : SerializeOperator(inner ? inner->collector_ : nullptr,
                            inner ? inner->enablePrediction_ : false,
                            inner ? inner->enableQuantization_ : false,
                            inner ? inner->enableCompress_ : false),
          inner_(inner), variant_(std::move(variant)) {}

    ~ColumnarBlobSerializeOperator() override = default;

    // ---- SerializeOperator 标准接口 ----
    // 所有调用透明代理给 inner_，inner_ 自行管理 buffer_/dataOffset_

    void init() override {
        if (inner_) inner_->init();
        // 同步到本类字段（部分上游代码会访问 header_ 等）
        if (inner_) header_ = inner_->header_;
    }

    void serializeInt(int val) override {
        if (inner_) inner_->serializeInt(val);
    }

    void serializeHeader(PMSFHeader header) override {
        if (inner_) inner_->serializeHeader(header);
        header_ = std::make_shared<PMSFHeader>(header);
    }

    void setHeader(std::shared_ptr<PMSFHeader> h) override {
        if (inner_) inner_->setHeader(h);
        header_ = h;
    }

    void serializeBaseMesh(std::shared_ptr<MCGAL::Mesh> mesh) override {
        if (inner_) inner_->serializeBaseMesh(mesh);
    }

    void serializeBaseMeshWithSeed(std::shared_ptr<MCGAL::Mesh> mesh,
                                   std::vector<MCGAL::Halfedge*> seeds) override {
        if (inner_) inner_->serializeBaseMeshWithSeed(mesh, seeds);
    }

    /// 将 inner_ 产出的完整二进制（包括 inner 自己的 [int32 len][buf]）
    /// 转换为 2 个 Parquet 文件：<basename>_metadata.parquet / _payload.parquet。
    /// 参数 path 中 .parquet 后缀会被去除作为 basename（与 ColumnarSerializeOperator 一致）。
    void serialize(std::string path) override;

  private:
    std::shared_ptr<SerializeOperator> inner_;
    std::string variant_;
};

#endif
