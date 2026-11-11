/*
 * @file UnifiedMultiTableDeserializeOperator.h
 * @brief V3 多表 reader —— 继承 V2 reader 读前 4 张表，扩展读 side-channel / blob。
 *
 * 用户（impl/ 层 main）调用：
 *   pmsf::DecompressPipeline::Builder()
 *       .withAlgoPreset(pmsf::AlgoPreset::<ALGO>, pmsf::Backend::Unified)
 *       .withInput(args.input)
 *       .withOutput(args.output)
 *       .run();
 *
 * 传入的 `input` 约定形如 `<base>.parquet`，V3 reader 会自动把它解释成
 * `<base>_v3_metadata.parquet / _v3_base_mesh.parquet / ...`。这一路径
 * 变换逻辑由 `DecompressPipeline` / `DecompressOptions` 层处理，不需要 user
 * 手动加 `_v3_` 后缀。
 */
#ifndef UNIFIED_MULTI_TABLE_DESERIALIZE_OPERATOR_H
#define UNIFIED_MULTI_TABLE_DESERIALIZE_OPERATOR_H

#include "ColumnarDeserializeOperator.h"
#include "storage/PMSFPayloadDescriptorRegistry.h"
#include "core/Symbol.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include <string>
#include <unordered_map>
#include <vector>

namespace pmsf {
struct RecoveredSideChannels {
    std::vector<SideChannelSpec> specs;
    /// 按 round 组织，对齐 SymbolStream。
    SymbolStream stream;
    PMSFSymbolTable paperTable;
    bool isPMSFSymbolTable = false;
    bool hasSymbolMapping = false;
};
}  // namespace pmsf

class UnifiedMultiTableDeserializeOperator : public ColumnarDeserializeOperator {
  public:
    UnifiedMultiTableDeserializeOperator(std::shared_ptr<SymbolReadOperator> reader,
                                           std::shared_ptr<ReconstructOperator> reconstructor)
        : ColumnarDeserializeOperator(reader, reconstructor) {}

    /// @brief path 约定为 `<base>.parquet`，内部统一前置 `_v3` 再委托 V2。
    void init(std::string path) override;

    /// @brief 读出的 side-channel 数据（若存在）。
    const pmsf::SymbolStream& sideStream() const { return side_.stream; }
    const std::vector<pmsf::SideChannelSpec>& sideChannelSpecs() const { return side_.specs; }
    const pmsf::PMSFSymbolTable& pmsfSymbolTable() const { return side_.paperTable; }
    bool hasPMSFSymbolTable() const { return side_.isPMSFSymbolTable && !side_.paperTable.empty(); }

    /// @brief 读出的 blob（按名字查）。
    const std::vector<std::uint8_t>* findBlob(const std::string& name) const {
        return pmsf::findPaperNamedBlobBytes(blobs_, name);
    }
    pmsf::PMSFSymbolPayloadTable paperSymbolPayloadTable() const;
    const std::vector<std::uint8_t>* findPMSFPayload(std::int32_t objectId,
                                                      std::int32_t partitionId,
                                                      std::int32_t lodId,
                                                      const std::string& payloadName) const;
    const std::vector<std::uint8_t>* findPMSFPayload(
        const pmsf::PMSFSymbolPayloadSpec& spec) const;
    const std::vector<std::uint8_t>* findPMSFPayloadByRole(
        pmsf::PMSFPayloadRole role,
        std::int32_t objectId = 0,
        std::int32_t partitionId = 0,
        std::int32_t lodId = 0) const;
    bool hasPMSFSymbolPayloadTable() const { return hasPMSFPayloadTable_; }

  private:
    static std::string toV3BaseName(const std::string& path);
    void readSideChannelTable(const std::string& path);
    void readBlobTableForBase(const std::string& baseName);

    /// @brief 阶段 D：读 `<base>_v3_compact.bin`，把 metadata + base_mesh 填进
    ///        基类 header_ / baseMeshData_ / quantBits_ / ...。新文件复用 V2CP
    ///        compact container；reader 保留 legacy V3CP fallback。
    ///        返回 true 表示成功解析并跳过 `_v3_metadata.parquet` /
    ///        `_v3_base_mesh.parquet` 两张表。
    bool readCompactBin(const std::string& path);

    pmsf::RecoveredSideChannels                         side_;
    std::unordered_map<std::string, std::vector<std::uint8_t>> blobs_;
    std::unordered_map<std::string, std::string> blobKinds_;
    pmsf::PMSFSymbolPayloadTable paperPayloadTable_;
    bool hasPMSFPayloadTable_ = false;
};

#endif
