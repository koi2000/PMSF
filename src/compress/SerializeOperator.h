/*
 * @Author: koi
 * @Date: 2025-08-29 17:49:34
 * @Description:
 */
#ifndef SerializeOperator_H
#define SerializeOperator_H

#include "Halfedge.h"
#include "core/PaperEdgeFlipSideChannel.h"
#include "core/PMSFSideChannel.h"
#include "storage/PMSFStorageContextSideChannel.h"
#include "Mesh.h"
#include "compress/SymbolCollectOperator.h"
#include "header/PMSFHeader.h"
#include "postprocess/EdgeFlipPostProcessOperator.h"
#include "segmentation/SegmentationOption.h"
#include "PMSFPayloadAdapterRegistry.h"
#include "storage/PMSFUnifiedArtifactWriter.h"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

/**
 * @brief 序列化操作符
 * @details 这里的序列化类型有很多区别
 * 首先header就有区别，然后就是序列化每一轮的symbol
 */
class SerializeOperator {
  public:
    SerializeOperator(std::shared_ptr<SymbolCollectOperator> collector,
                      bool enablePrediction = false,
                      bool enableQuantization = false,
                      bool enableCompress = false)
        : collector_(collector), enablePrediction_(enablePrediction), enableQuantization_(enableQuantization), enableCompress_(enableCompress) {}
    // virtual ~SerializeOperator() = 0;
    virtual ~SerializeOperator() {};

    virtual void init() = 0;

    // virtual void initBuffer(int size) = 0;

    virtual void serialize(std::string path) = 0;

    /// @brief 告知序列化器「压缩前原始网格」的顶点/面片数。
    ///
    /// 默认 no-op：不需要该信息的序列化器（如 Basic）忽略即可。
    /// Columnar 系把它写进 compact header 的 origTotalVertices/origTotalFaces
    /// 字段，供解码端按需注册内存池（见 MeshRuntime.h）。
    virtual void setOriginalMeshCounts(int vertices, int faces) {
        (void)vertices;
        (void)faces;
    }

    virtual void serializeInt(int val) {};

    void setEnableEdgeFlipPostProcess(bool enable) {
        enableEdgeFlipPostProcess_ = enable;
    }
    bool isEnableEdgeFlipPostProcess() const {
        return enableEdgeFlipPostProcess_;
    }
    void setEdgeFlipPostProcessConfig(int rounds,
                                      double longEdgeRatio,
                                      double minImprovementRatio) {
        edgeFlipPostProcessRounds_ = rounds > 0 ? rounds : 1;
        if (longEdgeRatio > 0.0) edgeFlipPostProcessLongEdgeRatio_ = longEdgeRatio;
        if (minImprovementRatio > 0.0) edgeFlipPostProcessMinImprovementRatio_ = minImprovementRatio;
    }
    void setEdgeFlipPayload(const std::vector<int>& roundCounts,
                            const std::vector<std::pair<int, int>>& symbols) {
        edgeFlipRoundCounts_ = roundCounts;
        edgeFlipSymbols_ = symbols;
        edgeFlipCoordSymbols_.clear();
    }
    void setEdgeFlipPayload(const std::vector<int>& roundCounts,
                            const std::vector<EdgeFlipPostProcessOperator::CoordSymbol>& coordSymbols) {
        edgeFlipRoundCounts_ = roundCounts;
        edgeFlipCoordSymbols_ = coordSymbols;
        edgeFlipSymbols_.clear();
    }
    void setEdgeFlipBitPayload(const std::vector<int>& roundBitCounts,
                               const std::vector<unsigned char>& roundBits) {
        edgeFlipRoundBitCounts_ = roundBitCounts;
        edgeFlipRoundBits_ = roundBits;
    }

    void setBoundaryBitPayload(const std::vector<int>& roundBitCounts,
                               const std::vector<unsigned char>& roundBits,
                               const std::vector<int>& roundResiduals = {},
                               int applyMode = 0) {
        boundaryRoundBitCounts_ = roundBitCounts;
        boundaryRoundBits_ = roundBits;
        boundaryRoundResiduals_ = roundResiduals;
        boundaryApplyMode_ = applyMode;
    }

    virtual void setHeader(std::shared_ptr<PMSFHeader> header) {
        this->header_ = header;
    }

    void setSeamCutBase(bool v) { seamCutBase_ = v; }
    bool seamCutBase() const { return seamCutBase_; }

    /// @brief header 已定稿（算法可能刚在 beforeCompressLoop 里改写过它）。
    ///
    /// 序列化器里凡是有「必须与 header 记录一致」的开关，都在这里同步。理由：
    /// 解码端是**读 header** 决定走哪条导入路径的，序列化器若按别的来源决定
    /// 走了另一条导出路径，产物就自相矛盾 —— 不报错，只默默解错。
    /// 实测：某算法在 beforeCompressLoop 里把 header 的 prediction 位按它自己
    /// 的语义关掉（它用连通性预测，不是残差预测），而 BasicSerializeOperator
    /// 仍按 options 走 range-coder 导出，二进制产物直接不可解。
    virtual void onHeaderFinalized(const PMSFHeader& /*header*/) {}

    virtual void serializeHeader(PMSFHeader header) {
        header.serialize(buffer_, dataOffset_);
    }

    virtual void serializeBaseMesh(std::shared_ptr<MCGAL::Mesh> mesh) = 0;

    virtual void serializeBaseMeshWithSeed(std::shared_ptr<MCGAL::Mesh> mesh, std::vector<MCGAL::Halfedge*> seed) = 0;

    void setSegmentationOption(const SegmentationOption& s) {
        segmentation_ = s;
    }
    const SegmentationOption& getSegmentationOption() const {
        return segmentation_;
    }

    std::shared_ptr<MCGAL::Mesh>& mesh() {
        return collector_->mesh_;
    }

    bool buildPMSFSymbolTableWithSerializerSideChannels(
        pmsf::PMSFSymbolTable& out) const {
        out.clear();
        if (!collector_) {
            return false;
        }
        const pmsf::PMSFSymbolTable* base = collector_->pmsfSymbolTable();
        if (!base || base->empty()) {
            return false;
        }

        out = *base;
        return augmentPMSFSymbolTableWithSerializerSideChannels(out);
    }

    void addPaperSidecarValue(std::string key, std::int32_t value) {
        paperSidecarSideValues_.emplace_back(std::move(key), value);
    }

    bool augmentPMSFSymbolTableWithSerializerSideChannels(
        pmsf::PMSFSymbolTable& table) const {
        const bool hasSymbolMapping =
            pmsf::pmsfSymbolTableHasSymbolMapping(table);
        bool changed = false;
        auto ensureRound = [&](std::size_t r) -> pmsf::PMSFSymbolRecord& {
            return pmsf::ensurePMSFSymbolRecord(table, r, 0, 0);
        };

        if (hasSymbolMapping && enableEdgeFlipPostProcess_) {
            if (!edgeFlipRoundBitCounts_.empty()) {
                changed = true;
                std::size_t bitOffset = 0;
                for (std::size_t r = 0; r < edgeFlipRoundBitCounts_.size(); ++r) {
                    const int count = std::max(edgeFlipRoundBitCounts_[r], 0);
                    auto& row = ensureRound(r);
                    pmsf::appendPaperEdgeFlipCandidateBitCount(
                        row.delta, static_cast<std::int32_t>(count));
                    for (int i = 0; i < count; ++i) {
                        const bool bit =
                            bitOffset < edgeFlipRoundBits_.size() &&
                            edgeFlipRoundBits_[bitOffset] != 0;
                        pmsf::appendPaperScalarSymbolForRole(
                            row.delta, pmsf::PMSFSymbolRole::FlipEdge, bit ? 1 : 0);
                        ++bitOffset;
                    }
                }
            } else if (!edgeFlipRoundCounts_.empty()) {
                changed = true;
                std::size_t symbolOffset = 0;
                std::size_t coordOffset = 0;
                const bool hasCoordSymbols = !edgeFlipCoordSymbols_.empty();
                for (std::size_t r = 0; r < edgeFlipRoundCounts_.size(); ++r) {
                    const int count = std::max(edgeFlipRoundCounts_[r], 0);
                    auto& row = ensureRound(r);
                    pmsf::appendPaperEdgeFlipRoundCount(
                        row.delta, static_cast<std::int32_t>(count));
                    for (int i = 0; i < count; ++i) {
                        pmsf::appendPaperScalarSymbolForRole(row.delta, pmsf::PMSFSymbolRole::FlipEdge, 1);
                        if (hasCoordSymbols && coordOffset < edgeFlipCoordSymbols_.size()) {
                            const auto& sym = edgeFlipCoordSymbols_[coordOffset++];
                            pmsf::appendPaperEdgeFlipCoordSymbol(
                                row.delta, sym);
                        } else if (symbolOffset < edgeFlipSymbols_.size()) {
                            const auto& sym = edgeFlipSymbols_[symbolOffset++];
                            pmsf::appendPaperEdgeFlipEndpointPair(
                                row.delta, sym);
                        }
                    }
                }
            }
        }

        if (!paperSidecarSideValues_.empty()) {
            changed = true;
            pmsf::appendPaperI32SideChannelEntries(
                ensureRound(0).delta, paperSidecarSideValues_);
        }

        return changed;
    }

    void writeUnifiedPMSFArtifactsForSerializer(
        const std::string& outputPathOrBase,
        bool stripExtension = true) const {
        // perf（2026-11-08 perf record：serialize 29.6 ms 中 ~25 ms 在这里）：
        // unified 存储未开启时，本函数下游的 writeUnifiedPMSFArtifactsForTable* 全部
        // 立即 return（见 PMSFUnifiedArtifactWriter.h / PMSFUnifiedArtifactWriter 的
        // #ifdef 分支），但这里先无条件构造完整的 PMSFSymbolTable（把每轮所有符号
        // 展开成 string/variant 记录，逐轮 appendPaper*），纯属死工作。
        // 早退语义与原来逐字节等价（unified 关 ⇒ 不写任何 sidecar/parquet）。
        if (!pmsf::unifiedStorageEnabled()) {
            return;
        }
        pmsf::PMSFSymbolTable augmented;
        if (buildPMSFSymbolTableWithSerializerSideChannels(augmented)) {
#ifdef COLUMNAR_SUPPORT
            std::vector<pmsf::PaperNamedBlob> blobs;
            if (collector_) {
                if (const auto* payloads = collector_->paperSymbolPayloadTable()) {
                    pmsf::appendPMSFPayloadRows(blobs, *payloads);
                }
            }
            if (blobs.empty()) {
                pmsf::writeUnifiedPMSFArtifactsForTable(
                    outputPathOrBase, augmented, stripExtension);
            } else {
                pmsf::writeUnifiedPMSFArtifactsForTableAndNamedBlobs(
                    outputPathOrBase, augmented, std::move(blobs), stripExtension);
            }
#else
            pmsf::writeUnifiedPMSFArtifactsForTable(
                outputPathOrBase, augmented, stripExtension);
#endif
            return;
        }
        pmsf::writeUnifiedPMSFArtifactsForCollector(
            outputPathOrBase, collector_, stripExtension);
    }

    template <typename BlobVector>
    void writeUnifiedPMSFArtifactsForSerializerAndNamedBlobs(
        const std::string& outputPathOrBase,
        BlobVector blobs,
        bool stripExtension = true) const {
        if (!pmsf::unifiedStorageEnabled()) {
            return;  // 同 writeUnifiedPMSFArtifactsForSerializer 的死工作早退
        }
        pmsf::PMSFSymbolTable augmented;
        if (buildPMSFSymbolTableWithSerializerSideChannels(augmented)) {
#ifdef COLUMNAR_SUPPORT
            if (collector_) {
                if (const auto* payloads = collector_->paperSymbolPayloadTable()) {
                    pmsf::appendPMSFPayloadRows(blobs, *payloads);
                }
            }
#endif
            pmsf::writeUnifiedPMSFArtifactsForTableAndNamedBlobs(
                outputPathOrBase, augmented, std::move(blobs), stripExtension);
            return;
        }
        pmsf::writeUnifiedPMSFArtifactsForCollectorAndNamedBlobs(
            outputPathOrBase, collector_, std::move(blobs), stripExtension);
    }

    void writePMSFPayloadOnlyFallbackSidecarForSerializer(
        const std::string& baseName,
        const std::string& sideKeyPrefix,
        const std::vector<std::pair<std::string, std::int32_t>>& extraSideValues = {}) const {
        if (!pmsf::unifiedStorageEnabled()) {
            return;
        }

        pmsf::PMSFSymbolTable table;
        (void)buildPMSFSymbolTableWithSerializerSideChannels(table);
        if (table.empty() && collector_) {
            if (const auto* source = collector_->pmsfSymbolTable()) {
                table = *source;
            }
        }
        if (table.empty()) {
            table.emplace_back();
        }

        pmsf::appendPMSFStorageContextMappingAndEntries(
            table.front().delta, sideKeyPrefix, 1, 0, extraSideValues);
        pmsf::writeUnifiedPMSFArtifactsForTable(baseName, table, false);
    }

    bool writeLengthPrefixedLegacySingleFile(
        const std::string& path,
        const void* data,
        std::size_t size,
        std::int32_t backendId) {
        const bool skipCompatFile = pmsf::legacySingleFileSkipCompatFile();
        addPaperSidecarValue(pmsf::kLegacySingleFileCompatFileSkippedSideName,
                             skipCompatFile ? 1 : 0);
        addPaperSidecarValue(pmsf::kLegacySingleFileBackendSideName, backendId);

        if (skipCompatFile) {
            auto blobs = pmsf::legacySingleFilePayloadBlob(data, size);
            writeUnifiedPMSFArtifactsForSerializerAndNamedBlobs(
                path, std::move(blobs));
            if (!pmsf::collectorHasPMSFSymbolTable(collector_)) {
                const std::string baseName =
                    pmsf::stripOutputExtensionForPMSFArtifacts(path);
                writePMSFPayloadOnlyFallbackSidecarForSerializer(
                    baseName,
                    pmsf::kLegacySingleFileSidecarPrefix,
                    pmsf::legacySingleFileFallbackSideValues(backendId));
            }
            return true;
        }

        std::ofstream fout(path, std::ios::binary);
        int len = static_cast<int>(size);
        fout.write(reinterpret_cast<const char*>(&len), sizeof(int));
        fout.write(reinterpret_cast<const char*>(data), len);
        fout.close();
        writeUnifiedPMSFArtifactsForSerializer(path);
        return false;
    }

    template <typename BlobVector>
    void writeUnifiedPMSFArtifactsForTableAndNamedBlobsForSerializer(
        const std::string& outputPathOrBase,
        pmsf::PMSFSymbolTable table,
        BlobVector blobs,
        bool stripExtension = true) const {
        augmentPMSFSymbolTableWithSerializerSideChannels(table);
        pmsf::writeUnifiedPMSFArtifactsForTableAndNamedBlobs(
            outputPathOrBase, table, std::move(blobs), stripExtension);
    }

    // virtual void serializeCharPointer(char* val, int size) = 0;

    bool enablePrediction_;
    bool enableCompress_;
    bool enableQuantization_;
    bool enableEdgeFlipPostProcess_ = false;
    int edgeFlipPostProcessRounds_ = 1;
    double edgeFlipPostProcessLongEdgeRatio_ = 1.35;
    double edgeFlipPostProcessMinImprovementRatio_ = 0.90;
    std::vector<int> edgeFlipRoundCounts_;
    std::vector<std::pair<int, int>> edgeFlipSymbols_;
    std::vector<EdgeFlipPostProcessOperator::CoordSymbol> edgeFlipCoordSymbols_;
    std::vector<int> edgeFlipRoundBitCounts_;
    std::vector<unsigned char> edgeFlipRoundBits_;
    std::vector<int> boundaryRoundBitCounts_;
    std::vector<unsigned char> boundaryRoundBits_;
    std::vector<int> boundaryRoundResiduals_;
    int boundaryApplyMode_ = 0;
    bool seamCutBase_ = false;
    std::vector<std::pair<std::string, std::int32_t>> paperSidecarSideValues_;
    std::shared_ptr<PMSFHeader> header_;
    SegmentationOption segmentation_;
    std::shared_ptr<SymbolCollectOperator> collector_;
    char* buffer_;
    size_t dataOffset_ = 0L;
};
#endif
