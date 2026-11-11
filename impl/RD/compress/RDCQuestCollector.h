// SPDX-License-Identifier: Apache-2.0
//
// RDCQuestCollector.h — task-19：RD on conquest engine 的编码端薄壳。
//
// 在已验证的 `ConquestSymbolCollectOperator` 之上只做三件事：
//   1) 注入 RD 引擎 hooks（H4 = 论文 Eq.(3) 可逆 lifting 局部 Frenet；H3 = 论文
//      Eq.(2) curvature，coarse-graph 对称；H1 默认关）；
//   2) 每层 collect 前后记录 gate 范围（H3 finalize 用）；
//   3) 每层结束后调 `finalizeLayer()`：在该层 **coarse 网格**上重算 H3 预测并
//      覆盖该层 delta（引擎运行期 H3 只返回默认 mean，理由见 RDCQuestHooks.h 的
//      对称契约；同层已删点/已插入点都不可见）。
//
// 不复制父类遍历/流布局：`stream()` 是父类成员，finalize 在 push 之后对
// `layers.back()` 就地改 delta（同一对象、无并发，故 const_cast 是安全的）。
#ifndef PMSF_RD_COMPRESS_RD_CQUEST_COLLECTOR_H
#define PMSF_RD_COMPRESS_RD_CQUEST_COLLECTOR_H

#include "Conquest/compress/ConquestSymbolCollectOperator.h"
#include "RD/hooks/RDCQuestHooks.h"

#include <memory>
#include <vector>

namespace pmsf {

class RDCQuestCollector final : public ConquestSymbolCollectOperator {
  public:
    RDCQuestCollector() = default;

    void init(std::shared_ptr<MCGAL::Mesh> mesh,
              std::vector<MCGAL::Halfedge*> seeds, PMSFHeader header) override {
        rdMesh_ = mesh;
        st_ = std::make_shared<rd_cq::RDCQuestGeomState>();
        st_->encoderMode = true;
        st_->h3Enabled = rd_cq::h3EnabledFromEnv();
        ConquestSymbolCollectOperator::init(std::move(mesh), std::move(seeds),
                                            std::move(header));
    }

    /// 每层重建 hook 集合（H3/H4 记录 per-layer 状态，天然干净）。
    void collect(MCGAL::Halfedge* seed) override {
        setHooks(rd_cq::makeRDCQuestHooks(st_, /*encoderMode=*/true));
        if (st_) st_->beginCollectLayer();
        ConquestSymbolCollectOperator::collect(seed);
        if (!st_) return;
        st_->endCollectLayer();
        // H3：在本层 coarse 网格上重算论文 Eq.(2) 并覆盖该层 delta。
        auto& stream = const_cast<conquest_blob::Stream&>(this->stream());
        if (!stream.layers.empty()) {
            st_->finalizeLayer(stream.layers.back(), stream,
                               stream.layers.size() - 1);
        }
    }

    void collect(std::vector<MCGAL::Halfedge*> seeds) override {
        if (!seeds.empty()) collect(seeds[0]);
    }

  private:
    std::shared_ptr<MCGAL::Mesh> rdMesh_;  // 本类持有（父类 mesh_ 私有）
    std::shared_ptr<rd_cq::RDCQuestGeomState> st_;
};

}  // namespace pmsf

#endif  // PMSF_RD_COMPRESS_RD_CQUEST_COLLECTOR_H
