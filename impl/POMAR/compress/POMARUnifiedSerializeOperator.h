/*
 * @file POMARUnifiedSerializeOperator.h
 * @brief POMAR × Parquet V3 专属 serializer（v3-refactor 阶段 3a）。
 *
 * 设计抉择：POMAR 的 inner buffer 由 4 段组成（base_mesh / cluster0 / cluster
 * meta / clusters rest），V2 版本把它们各自拆成 4 张专属 schema 的 parquet 表
 * 以支持 per-cluster random-access。V3 的迁移承载方式是把这 4 段**原样写入
 * `_v3_blob.parquet` 的统一 paper payload relation**。这样：
 *
 *   (1) V3 reader 只需要 `_v3_metadata.parquet` + `_v3_blob.parquet` 就能
 *       重建完整 inner buffer（复用现有 PomarColumnarDeserializeOperator 做解码）；
 *   (2) V2 的专属 4 表（`_cluster0.parquet` 等）依然产出以保证 V2 reader 的
 *       random-access 能力不退化；
 *   (3) 未来如果要废弃 V2 专属 schema，只需删除 V2 兼容表写路径，V3 侧
 *       仍然通过 paper payload relation 保持独立。
 *
 * 继承策略：继承 `POMARColumnarSerializeOperator` 复用它已经写好的 "inner
 * buffer 拆段 → 4 张 V2 专属表" 流程；**在 V2 写完之后**额外产出 2 张 V3 表
 * （_v3_metadata / _v3_blob），以及把原 4 张 V2 表按 V3 命名再写一遍（前缀
 * `_v3_`）以保证 V3 路径产物完整闭环。
 *
 * 规模：<80 LOC header + <160 LOC cpp。
 */
#ifndef POMAR_UNIFIED_SERIALIZE_OPERATOR_H
#define POMAR_UNIFIED_SERIALIZE_OPERATOR_H

#include "POMAR/compress/POMARColumnarSerializeOperator.h"
#include <memory>
#include <string>

class POMARUnifiedSerializeOperator : public POMARColumnarSerializeOperator {
  public:
    POMARUnifiedSerializeOperator(std::shared_ptr<SymbolCollectOperator> collector,
                                    bool enablePrediction,
                                    bool enableQuantization,
                                    bool enableCompress)
        : POMARColumnarSerializeOperator(collector, enablePrediction,
                                          enableQuantization, enableCompress) {}

    ~POMARUnifiedSerializeOperator() override = default;

    /// @brief serialize(path)：
    ///   1. 把 path 变形为 `<base>_v3.parquet`；
    ///   2. 委托 V2 专属 writer 产出 4 张 `<base>_v3_{metadata,base_mesh,
    ///      cluster0,clusters}.parquet`（保持 V2 reader 可读）；
    ///   3. 从父类暴露的 rawBuffer() / segment offsets 生成 4 段 paper payload，统一
    ///      写入 `<base>_v3_blob.parquet`（V3 reader 通用入口）。
    void serialize(std::string path) override;
};

#endif
