// SPDX-License-Identifier: Apache-2.0
//
// RDCQuestAlgorithm.cpp — task-19 P2b：`rd_cq`（RD on conquest engine）。
//
// 论文映射（Ahn et al., VisComput 27(6-8):769-779, 2011）：
//   * traversal / connectivity = 论文 §2 的 A&D valence-driven decimating/cleaning
//     conquest（valence/null_patch 符号；decoder 按同一 BFS replay）——
//     直接复用框架内**逐层验证过**的原生 codec：
//       encoder `pmsf::FrontierConquestEngine::decimateLayer`
//       decoder `pmsf::decodeLayer`（P13.3/P13.4，4 网格 × 8 层链式 topoGeomOk=1）
//     ⇒ **没有 isPoly locator**（这是 rd_tag 路径 5.30 bpv 的结构成本）。
//   * H1 GatePriorityPolicy = 论文 §3.2 prioritized gate selection（本轮留默认
//     FIFO；PGS 需要引擎 additive diff，已单独报 Lead 审批）。
//   * H3 GeometryPredictor  = 论文 §3.1 curvature prediction（本轮留默认算术平均；
//     下一增量接入，需要 per-layer 状态与粗网格对称契约）。
//   * H4 ResidualQuantizer  = 论文 §3.1 Eq.(3) 局部 Frenet 系投影 —— 本文件注入
//     `RDFrenetQuantizer`（可逆整数 lifting，几何逐格无损）。
//   * H5 SymbolPostProc     = 论文 §3.3 code packing（可选，本轮默认恒等）。
//
// 与既有路径的关系：默认 `algorithm=rd`（rd_tag，25.184 bpv）与
// `algorithm=conquest`（22.150 bpv）**逐字节不变**；`rd_cq` 是新增注册名，
// env-gated（`PMSF_RD_CQ_GEOM` 控制 H4；不设也默认 frame，因为 rd_cq 本身即
// 实验名）。阶段一 faithful_path 在 rd_cq 真实 artifact <25.184 + decode 六项
// true + H1/H3/H4 生效证据齐全前仍是 rd_tag（Lead 裁决）。
#include "algorithm/ISimplificationAlgorithm.h"
#include "algorithm/SimplificationAlgorithmRegistry.h"

#include "Conquest/decompress/ConquestSymbolReadOperator.h"
#include "Conquest/elimination/ConquestNoopEliminateOperator.h"
#include "Conquest/reconstruct/ConquestReconstructOperator.h"
#include "RD/compress/RDCQuestCollector.h"
#include "RD/hooks/RDCQuestHooks.h"
#include "RD/reconstruct/RDCQuestReconstructOperator.h"
#include "decompress/ColumnarDeserializeOperator.h"

#include <memory>
#include <string>

namespace pmsf {
namespace {

class RDCQuestAlgorithm final : public ISimplificationAlgorithm {
  public:
    AlgorithmInfo info() const override {
        return AlgorithmInfo{
            .name = "rd_cq",
            .version = 1,
            .primitive = PrimitiveKind::Vertex,
            .selection = SelectionKind::ConnectivityDriven,
        };
    }

    /// 遍历/删点在 conquest 引擎里：框架侧必须空转（与 `algorithm=conquest`
    /// 完全同一设计，见 impl/Conquest/elimination/ConquestNoopEliminateOperator.h）。
    EliminateOperatorPtr eliminate() const override {
        return std::make_shared<ConquestNoopEliminateOperator>();
    }

    /// 编码端：已验证的 conquest 薄壳 + RD H4 hook（每轮注入）。
    CollectorFactory collectorFactory() const override {
        return []() -> std::shared_ptr<SymbolCollectOperator> {
            return std::make_shared<RDCQuestCollector>();
        };
    }

    /// conquest 几何残差按整数格量化（与 conquest 路径同口径）。
    bool requiresQuantization() const override { return true; }

    /// 解码端：与 impl/Conquest/DecompressFactoryRegistration.cpp 的 tripe **完全
    /// 同构**（reader 与 reconstructor 必须共享同一个 reader 实例），只是把 RD 的
    /// H4 hook 注入 reconstructor（transform/inverseTransform 必须同源）。
    DecompressOptions makeDecompressOptions(Backend backend,
                                            const std::string& input,
                                            const std::string& output) const override {
        if (backend == Backend::Columnar) {
            auto reader = std::make_shared<ConquestSymbolReadOperator>();
            // H3/H4 的 decoder 状态：同层已插入中心集合 + coarse-graph 预测。
            auto st = std::make_shared<rd_cq::RDCQuestGeomState>();
            st->encoderMode = false;
            st->h3Enabled = rd_cq::h3EnabledFromEnv();
            auto reconstructor =
                std::make_shared<RDCQuestReconstructOperator>(reader, st);
            if (auto hooks = rd_cq::makeRDCQuestHooks(st, /*encoderMode=*/false)) {
                reconstructor->setHooks(std::move(hooks));
            }
            auto deserializer = std::make_shared<ColumnarDeserializeOperator>(
                reader, reconstructor);
            return DecompressOptions::Builder()
                .setDeserializeOperator(deserializer)
                .setPath(input)
                .setOutputPath(output)
                .build();
        }
        return ISimplificationAlgorithm::makeDecompressOptions(backend, input,
                                                               output);
    }
};

}  // namespace
}  // namespace pmsf

PMSF_REGISTER_SIMPLIFICATION_ALGORITHM(pmsf::RDCQuestAlgorithm);
