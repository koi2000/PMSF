/*
 * POMARColumnarDeserializeOperator
 *
 * 对称消费 `POMARColumnarSerializeOperator` 生成的 4 张 parquet 表：
 *   _metadata / _base_mesh / _cluster0 / _clusters
 *
 * 反序列化策略：读取 4 表后**重新拼回与 inner buffer 逐字节等价的字节流**，
 * 然后 wrap 一个 inner `PomarColumnarDeserializeOperator`，让它按原路径解码。
 * 算法层（reader / reconstruct）完全不动，round-trip 保证 byte-level 一致。
 */

#pragma once

#include "decompress/DeserializeOperator.h"
#include "header/PMSFHeader.h"

#include <cstddef>
#include <memory>
#include <string>

class PomarColumnarDeserializeOperator;
class SymbolReadOperator;
class ReconstructOperator;

class POMARColumnarDeserializeOperator : public DeserializeOperator {
  public:
    POMARColumnarDeserializeOperator(std::shared_ptr<SymbolReadOperator> reader,
                                      std::shared_ptr<ReconstructOperator> reconstruct);
    ~POMARColumnarDeserializeOperator();

    void init(std::string path) override;
    void setPaperStream(bool enabled) { paperStream_ = enabled; }
    void deserializeHeader(PMSFHeader& header) override;
    std::shared_ptr<MCGAL::Mesh> deserializeBaseMesh() override;
    void deserializeOneRound() override;
    void setSeeds(std::vector<MCGAL::Halfedge*> seeds) override;
    std::shared_ptr<MCGAL::Mesh> exportMesh() override;

    /// 压缩前原始网格规模（init 从 compact header 的 origTotal 字段读；
    /// 旧产物该字段为 0 ⇒ 返回 0,0，调用方回落默认池）。
    void originalMeshCounts(int& v, int& f) const override {
        v = origTotalVertices_;
        f = origTotalFaces_;
    }

    // ---- (roi, lod) 随机访问谓词（设计文档 §2.5）----------------------------
    /// region 维度：行级过滤，只解码该 group。
    void setQueryRegion(int regionId) override;
    /// lod 维度：解码主循环按 queryMaxLod() 截断轮数。
    void setQueryMaxLod(int maxLod) override { maxRounds_ = maxLod; }
    int queryMaxLod() const override { return maxRounds_; }
    bool supportsRandomAccess() const override { return true; }

  private:
    int maxRounds_ = -1;
    int origTotalVertices_ = 0;  // compact header origTotalVertices（init 读取）
    int origTotalFaces_ = 0;     // compact header origTotalFaces
    std::shared_ptr<PomarColumnarDeserializeOperator> inner_;
    char* assembledBuffer_ = nullptr;
    std::size_t assembledSize_ = 0;
    PMSFHeader header_;
    bool paperStream_ = false;
};
