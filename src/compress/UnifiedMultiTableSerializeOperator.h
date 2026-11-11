/*
 * @file UnifiedMultiTableSerializeOperator.h
 * @brief Parquet V3 多表 Serializer —— 9 篇论文 + default 的迁移目标。
 *
 * v3-refactor 分支新增。**设计哲学：V3 = V2 的严格超集**。
 *
 * 物理文件布局（一次 serialize() 调用产出 4~6 个 parquet 文件）：
 *
 *   <base>_v3_metadata.parquet     ← 与 V2 字节兼容（同 schema），额外写入 KV
 *                                    元数据 `pmsf_format_version=3` 与 `pmsf_side_channels`
 *                                    字符串（若算法注册了 side channel）。
 *   <base>_v3_base_mesh.parquet    ← 与 V2 字节兼容。
 *   <base>_v3_symbols.parquet      ← 与 V2 字节兼容（BPV 优化路径 codec=2）。
 *   <base>_v3_boundary.parquet     ← 与 V2 字节兼容，且仅在 boundaryGraph_ 非空时生成。
 *
 *   <base>_v3_sidechannel.parquet  ← **V3 新增**。当算法通过 registerSideChannel()
 *                                    注册过 SideChannelSpec 且对应 RoundSymbols::side
 *                                    有内容时产出；每行一个 side cell，列包含
 *                                    round/pos/channel_id + 具体类型列。若所有
 *                                    side 为空则文件不写。
 *
 *   <base>_v3_blob.parquet         ← **V3 新增**。承载统一 paper payload relation：
 *                                    PPMC/POMAR/VQMMC/通用 rc 兼容 payload 先映射到
 *                                    论文符号命名（如 pmsc_ppmc_R_payload、
 *                                    pomar_meta、Parquet symbols RC payload），再以单一
 *                                    blob schema 写入。读端可为旧 raw 名称注册
 *                                    兼容别名，但磁盘侧不再 raw/paper 双写。
 *                                    若无 addBlob()/setPMSFSymbolPayloadTable()
 *                                    调用则不写。
 *
 * 读端使用 UnifiedMultiTableDeserializeOperator，对前 4 张表原样沿用 V2 reader
 * 代码，对第 5/6 张表做 side-channel / blob 的显式解析。
 *
 * 因此：
 *   - 任意已实现 V2 的算法都可一行切换到 V3（只是多了两张可选的 side-table）；
 *   - 新论文要落盘残差 / 特征向量 / codebook idx，只需 registerSideChannel()
 *     一次，不再需要改 V2 的 4-table schema；
 *   - 与 V2 物理文件名后缀（`_v3_` vs `_`）完全隔离，同一目录下两套可共存。
 */
#ifndef UNIFIED_MULTI_TABLE_SERIALIZE_OPERATOR_H
#define UNIFIED_MULTI_TABLE_SERIALIZE_OPERATOR_H

#include "ColumnarSerializeOperator.h"
#include "core/Symbol.h"
#include "storage/PMSFPayloadDescriptorRegistry.h"
#include "storage/PMSFSymbolStorageAdapter.h"
#include "storage/PMSFUnifiedArtifactWriter.h"

#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace pmsf {

/// @brief Payload blob carrier. `name` may be a legacy raw key or an already
///        canonical paper payload key; writeBlobTable() normalizes it to the
///        unified paper payload relation before writing.
struct NamedBlob {
    std::string name;   // 业务语义 key (e.g. "ppmc_rc", "vq_codebook", "pomar_meta")
    std::string kind;   // 进一步的子类型标签（可为空）
    std::vector<std::uint8_t> bytes;
};

inline void appendUnifiedNamedBlob(std::vector<NamedBlob>& blobs,
                                     NamedBlob blob) {
    blobs.push_back(std::move(blob));
}

}  // namespace pmsf

/// @brief V3 多表 serializer。**继承** `ColumnarSerializeOperator` 以复用
///        `serializeBaseMeshWithSeed` 与 4 表构造代码，不触碰 V2 字节路径。
class UnifiedMultiTableSerializeOperator : public ColumnarSerializeOperator {
  public:
    UnifiedMultiTableSerializeOperator(std::shared_ptr<SymbolCollectOperator> collector,
                                         bool enablePrediction = false,
                                         bool enableQuantization = false,
                                         bool enableCompress = false)
        : ColumnarSerializeOperator(collector, enablePrediction, enableQuantization, enableCompress) {}

    /// @brief 注册一个 side-channel。算法在 compress 主循环外注册；collect 阶段
    ///        填充到 RoundSymbols::side[name]（通过 Bridge 或专属 collector）。
    void registerSideChannel(pmsf::SideChannelSpec spec) {
        pmsf::appendPMSFSideChannelSpec(sideChannels_, std::move(spec));
    }

    /// @brief 追加一段 payload blob（一次 serialize 可以 addBlob() 多次）。
    ///        写出时会规范化为 paper payload relation，避免算法专属 schema
    ///        或 raw/paper 双份 payload。
    void addBlob(pmsf::NamedBlob blob) {
        pmsf::appendUnifiedNamedBlob(blobs_, std::move(blob));
    }
    void addPMSFSymbolPayload(pmsf::PMSFSymbolPayloadRecord row) {
        pmsf::setPMSFPayloadRecordRoleFromNameCodec(row);
        auto paperBlob = pmsf::makePaperNamedBlob(std::move(row));
        pmsf::NamedBlob nb;
        nb.name = std::move(paperBlob.name);
        nb.kind = std::move(paperBlob.kind);
        nb.bytes = std::move(paperBlob.bytes);
        addBlob(std::move(nb));
    }

    /// @brief 注入一个 SymbolStream 作为 side-channel 数据源。若未注入，V3
    ///        尝试通过 collector_ dynamic_cast 到 Basic/Optimized 读队列并构建
    ///        一个 F/E/V/P/VP-only 的 stream（无 side 值）。
    void setSymbolStream(pmsf::SymbolStream stream) {
        stream_ = pmsf::storageStreamFromUnifiedRounds(stream);
        streamInjected_ = true;
        pmsfSymbolTableMode_ = false;
        pmsfSymbolMappingMode_ = false;
    }

    /// @brief Inject a paper-level table directly. This is the preferred path
    ///        for segmented / random-access algorithms because it preserves
    ///        the paper's (partition_id, lod_id, Delta_r^p) structure.
    void setPMSFSymbolTable(pmsf::PMSFSymbolTable table) {
        const bool hasMapping = pmsf::pmsfSymbolTableHasSymbolMapping(table);
        stream_ = pmsf::storageStreamFromPMSFSymbolTable(table);
        streamInjected_ = true;
        pmsfSymbolTableMode_ = true;
        pmsfSymbolMappingMode_ = hasMapping;
    }

    /// @brief Inject already-compressed physical payloads under the same
    ///        paper relation key. This is the migration target for existing
    ///        rc_blob/blob_layout paths: keep compact bytes, but stop creating
    ///        one Parquet schema per algorithm.
    void setPMSFSymbolPayloadTable(const pmsf::PMSFSymbolPayloadTable& table) {
        for (const auto& row : table) {
            addPMSFSymbolPayload(row);
        }
    }

    /// @brief 主要入口。覆盖基类：把 `path` 改成 `<base>_v3.parquet` 伪装传给 V2::serialize，
    ///        产出 4 张 `_v3_xxx.parquet`；随后写 side-channel / blob 两张可选表。
    void serialize(std::string path) override;

    const std::vector<pmsf::SideChannelSpec>& sideChannels() const { return sideChannels_; }
    const std::vector<pmsf::NamedBlob>&       blobs()        const { return blobs_; }

    // ========================================================================
    // 阶段 D：紧凑模式 —— 用 `<base>_v3_compact.bin` 替代 `_v3_metadata.parquet`
    // + `_v3_base_mesh.parquet` 两张 Parquet 表。
    //
    // 动机：bunny 实测 `_v3_metadata.parquet` = 5593 B（1.28 bpv），
    //       `_v3_base_mesh.parquet` = 35161 B（8.08 bpv）。这 40.7 KB / 9.36 bpv
    //       **不是数据本身太大**，而是 Parquet 文件格式 footer/schema/RG stats
    //       在极小 table（1 row / 几 row）上的固定 overhead。换成裸二进制 +
    //       zlib 后 bunny 实测降至 ~10 KB / ~2.3 bpv。
    //
    // 启用方式：
    //   1. 环境变量 `PMSF_V3_COMPACT=1`（推荐，不改 CLI）
    //   2. 或在 compress 入口调 setCompactMode(true)
    //
    // 文件布局：复用 V2 compact container（内部 magic "V2CP"）。reader 仍保留
    // legacy "V3CP" fallback 以读取历史产物；新写出统一使用同一 O/M_0^p
    // compact-base layout，避免 V2/V3 为 base mesh 维护两套物理格式。
    // ========================================================================
    void setCompactMode(bool v) { compactMode_ = v; }
    bool isCompactMode() const { return compactMode_; }

  private:
    std::vector<pmsf::SideChannelSpec> sideChannels_;
    std::vector<pmsf::NamedBlob>       blobs_;
    pmsf::SymbolStream                 stream_;
    bool                               streamInjected_ = false;
    bool                               pmsfSymbolTableMode_ = false;
    bool                               pmsfSymbolMappingMode_ = false;
    bool                               compactMode_ = false;  // 阶段 D

    /// @brief 把 V2 path 变换成 V3 basename —— 去扩展名后追加 `_v3`，再补回 `.parquet`。
    ///        这样 V2::serialize 内部去掉 `.parquet` 得到 `<base>_v3`，生成的 4 张
    ///        `<base>_v3_metadata.parquet` ... 正好满足 V3 命名规范。
    static std::string toUnifiedPath(const std::string& path);

    /// @brief 若用户没有通过 setSymbolStream 注入，尝试从 collector_ 推断出
    ///        一份仅含 F/E/V/P/VP 的 stream。PPMC 等自带 collector 且没有暴露
    ///        deque 的情形下返回空。
    pmsf::SymbolStream buildStreamFromCollector() const;

    /// @brief 写 `<base>_v3_sidechannel.parquet`，当且仅当某个 round 的 side 非空。
    void writeSideChannelTable(const std::string& path);

    /// @brief 写 `<base>_v3_blob.parquet`，当且仅当 blobs_ 非空；写入前会把
    ///        payload 名称统一到论文符号命名。
    void writeBlobTableForBase(const std::string& baseName);

    /// @brief 阶段 D：写 `<base>_v3_compact.bin`，把 metadata + base_mesh 打包
    ///        成紧凑 zlib-deflated 二进制。返回 true 表示成功，调用方随后应
    ///        删除 `_v3_metadata.parquet` + `_v3_base_mesh.parquet`。
    bool writeCompactBin(const std::string& path);
};

#endif
