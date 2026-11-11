// SPDX-License-Identifier: Apache-2.0
//
// RDCanonicalCollector.h — task-43 route A 的编码端 collector（env PMSF_RD_CANON=1）。
//
// 职责：每轮跑**一层** canonical frontier conquest（RDCanonicalTraversal.h），
// 把 (valence/null 符号, 残差, 本层 seed) 收进 sv=71 三流 blob。
//
// 与默认 rd_tag 的分工差异：rd_tag 由框架 selector/eliminate 决定消除、collector
// 只做 BFS 符号映射；route A 的遍历**自带消除**（论文 §3.3 的 gate queue 本身就是
// locator），故框架侧必须装空 eliminate（RDCanonicalNoopEliminate.h），让 collect()
// 成为唯一推进 mesh 的地方。
//
// 布局/熵编码见 RDCanonicalBlobFormat.h。默认路径零影响：本类只在
// impl/common/RetriOverride.h 的 RD_TAG 分支里、`PMSF_RD_CANON=1` 时构造。
#ifndef PMSF_IMPL_RD_COMPRESS_RD_CANONICAL_COLLECTOR_H
#define PMSF_IMPL_RD_COMPRESS_RD_CANONICAL_COLLECTOR_H

#include "RD/compress/RDTagSymbolCollectOperator.h"

#include "RD/RDCanonicalTraversal.h"
#include "RD/RDLift.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace rd_canon {

class RDCanonicalCollector final : public RDTagSymbolCollectOperator {
  public:
    RDCanonicalCollector() = default;

    void init(std::shared_ptr<MCGAL::Mesh> mesh,
              std::vector<MCGAL::Halfedge*> seeds, PMSFHeader header) override;

    void collect(MCGAL::Halfedge* seed) override;
    void collect(MCGAL::Halfedge* seed,
                 const std::vector<MCGAL::Halfedge*>& gates) override;
    void collect(std::vector<MCGAL::Halfedge*> seeds) override;

    int exportToBufferWithRangeCoder(char* buffer,
                                     bool enableQuantization = false) override;
    int exportToBuffer(char* buffer, bool enableQuantization = false,
                       bool enableCompress = false) override;
    const pmsf::PMSFSymbolTable* pmsfSymbolTable() const override {
        return nullptr;
    }

    // ---- 证据用 accessors ---------------------------------------------------
    struct LayerRecord {
        std::vector<int> syms;
        std::vector<MCGAL::PointInt> residuals;
        bool hasSeed = false;
        int seedQ[6] = {0, 0, 0, 0, 0, 0};
    };
    std::size_t layerCount() const {
        return layers_.size();
    }
    const QuantGrid& quantGrid() const {
        return grid_;
    }

  private:
    /// 现算量化网格并**把工作网格吸附到格中心**（两端世界逐位一致的前提）。
    void prepareGridAndSnap();
    /// 选本层 seed：优先框架给的活 seed；死了则按 canonical 规则现算。
    MCGAL::Halfedge* pickLayerSeed(MCGAL::Halfedge* hint);
    /// 记录本层 seed 的量化格号（tgt(seed) / tgt(opp(seed))）。
    void recordSeed(LayerRecord& layer, MCGAL::Halfedge* seed) const;
    void runOneLayer(MCGAL::Halfedge* seed);

    QuantGrid grid_;
    std::vector<LayerRecord> layers_;
    MCGAL::Halfedge* frameworkSeed_ = nullptr;
};

}  // namespace rd_canon

#endif  // PMSF_IMPL_RD_COMPRESS_RD_CANONICAL_COLLECTOR_H
