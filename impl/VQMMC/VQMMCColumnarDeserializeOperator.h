/*
 * VQMMCColumnarDeserializeOperator
 * ----------------------------------
 * 与 VQMMCColumnarSerializeOperator 对称的列存读取器。
 *
 * 实现策略：从 4 张 parquet 表读出后，**在内存中按 BasicSerializeOperator + zlib
 * 三段块的旧布局重新拼成连续 buffer**，然后把 buffer 喂给现有的
 * VQMMCSymbolReadOperator —— 这样：
 *   1. 算法层（reconstruct / read operator）逐字节兼容。
 *   2. 列存的"按列投影"语义在写入端真实呈现（DuckDB / pyarrow 可单独读列）。
 *   3. 不必复制 readBaseMesh / build mesh from buffer 的几百行逻辑。
 *
 * Decompress 入口（impl/VQMMC/VQMMCColumnarDecompress.cpp）拿到本类后调用：
 *   - init(path)               读 4 张 parquet → 重组 buffer_
 *   - deserializeHeader(h)     从重组 buffer_ 读 PMSFHeader（兼容路径）
 *     // 若 header_ 已设置，直接复用
 *   - deserializeBaseMesh()    委托给 inner VQMMCDeserializeOperator
 *   - deserializeOneRound()    同上
 *   - exportMesh()             同上
 */
#pragma once

#include "decompress/DeserializeOperator.h"

#include <memory>
#include <string>

class VQMMCDeserializeOperator;

class VQMMCColumnarDeserializeOperator : public DeserializeOperator {
  public:
    explicit VQMMCColumnarDeserializeOperator(std::shared_ptr<VQMMCDeserializeOperator> inner)
        : DeserializeOperator(nullptr, nullptr), inner_(inner) {}

    ~VQMMCColumnarDeserializeOperator();

    // ---- DeserializeOperator 标准接口 ----
    void init(std::string path) override;
    void deserializeHeader(PMSFHeader& header) override;
    std::shared_ptr<MCGAL::Mesh> deserializeBaseMesh() override;
    void deserializeOneRound() override;
    std::shared_ptr<MCGAL::Mesh> exportMesh() override;

    void setSeeds(std::vector<MCGAL::Halfedge*> seeds) override;

    // ---- (roi, lod) 随机访问谓词（设计文档 §2.5）----------------------------
    void setQueryRegion(int regionId) override;
    void setQueryMaxLod(int maxLod) override { maxRounds_ = maxLod; }
    int queryMaxLod() const override { return maxRounds_; }
    bool supportsRandomAccess() const override { return true; }

  private:
    int maxRounds_ = -1;
    std::shared_ptr<VQMMCDeserializeOperator> inner_;
    /// 重组后的 inner-buffer。所有权属于本类；inner_ 在 init 阶段被注入此 buffer。
    char* assembledBuffer_ = nullptr;
    std::size_t assembledSize_ = 0;
};
