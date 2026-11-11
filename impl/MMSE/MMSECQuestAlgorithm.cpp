// SPDX-License-Identifier: Apache-2.0
//
// MMSECQuestAlgorithm.cpp — MMSE on the valence-conquest backbone (P3).
//
// The MMSE paper (Lee et al., ICIP 2011, §2) builds its progressive codec on the
// **vertex decimation conquest of Alliez & Desbrun [2]** and encodes each
// layer with the valence-driven connectivity coder of [2].  This framework has
// that mechanism as an algorithm-agnostic engine
// (`src/conquest/FrontierConquestEngine` + `ConquestDecodeLayer`, verified
// symbol-for-symbol / layer-by-layer), with five S5 `EngineHooks` and a decoder
// that replays the *same* hook instance.
//
// P3 therefore does not reimplement the traversal.  This algorithm registers a
// separate name (`mmse_cq`) whose codec is:
//   * connectivity + decimation: the framework valence-conquest engine (paper [2]),
//   * geometry prediction:       MMSE paper §2.2 (dual-ring / MMSE-LS) injected
//                                through H3 `GeometryPredictor`,
//   * geometry residual coding:  the engine's per-layer / per-component
//                                range-coded residual transport.
//
// Why a separate algorithm name (Lead decision 2026-10-22):
//   the canonical `algorithm=MMSE` path is the pre-existing RD-tag-compatible
//   bridge and MUST stay byte-identical (62,633 B / 25.241247 bpv baseline).
//   Registering `mmse_cq` keeps that untouched and gives the new path an honest
//   metadata name.  Phase-1 row registration (`faithful_path`) is a docs/gate
//   concern owned by the Lead / targets-quality.
//
// P3-B1 (this state): the H3 hook is the paper's barycentric prediction (mode 0),
// which is arithmetically identical to the engine default ⇒ artifacts are
// byte-identical to `algorithm=conquest` at the same rounds.  That is the
// plumbing check before P3-B2 adds the dual-ring / LS / mode stream.
//
// Decoder side: zero algorithm code — the generic ColumnarDeserializeOperator plus
// the conquest reader/reconstructor triple registered below.  The reader and
// reconstructor are the verified conquest implementations; the MMSE prediction
// rides in through `setHooks()` (same hook set as the encoder).

#include "algorithm/ISimplificationAlgorithm.h"
#include "algorithm/SimplificationAlgorithmRegistry.h"
#include "core/DecompressOperatorRegistry.h"

#include "Conquest/elimination/ConquestNoopEliminateOperator.h"
#include "MMSE/compress/MMSECQuestSymbolCollectOperator.h"
#include "MMSE/decompress/MMSECQuestSymbolReadOperator.h"
#include "MMSE/reconstruct/MMSECQuestReconstructOperator.h"
#include "decompress/ColumnarDeserializeOperator.h"
#include "predict/MMSEConquestHooks.h"

namespace pmsf {
namespace {

/// @brief MMSE paper algorithm on the valence-conquest backbone.
class MMSECQuestAlgorithm final : public ISimplificationAlgorithm {
  public:
    void configure(const nlohmann::json& options) override {
        const std::string entropy = options.value("entropy", std::string("legacy"));
        if (entropy != "legacy" && entropy != "paper-bitplane") {
            throw std::runtime_error("MMSE entropy must be legacy or paper-bitplane");
        }
        paperEntropy_ = entropy == "paper-bitplane";
        const auto segmentation = options.value("segmentation", std::string("legacy-chunks"));
        if (segmentation != "legacy-chunks" && segmentation != "recursive-topology") {
            throw std::runtime_error("MMSE segmentation must be legacy-chunks or recursive-topology");
        }
        topology_ = segmentation == "recursive-topology";
        const auto predictor = options.value("predictor", std::string("legacy-coarse"));
        if (predictor != "legacy-coarse" && predictor != "fine-ring-mmse")
            throw std::runtime_error("MMSE predictor must be legacy-coarse or fine-ring-mmse");
        fineRing_ = predictor == "fine-ring-mmse";
        if (fineRing_ && !topology_) throw std::runtime_error("MMSE fine-ring-mmse requires recursive-topology");
        relations_ = options.value("paper_relations",false);
        if (relations_ && !topology_) throw std::runtime_error("MMSE paper_relations requires recursive-topology");
        threshold_ = options.value("cluster_threshold", 32);
        topologySeed_ = options.value("segmentation_seed", std::uint64_t(1));
        if (threshold_ < 1 || threshold_ > 4096 || (topology_ && !paperEntropy_)) {
            throw std::runtime_error("MMSE recursive topology requires paper-bitplane entropy and threshold in [1,4096]");
        }
        const std::string context = options.value("bitplane_context", std::string("magnitude"));
        if (context == "plane") context_ = signed_bitplane::Context::Plane;
        else if (context == "magnitude") context_ = signed_bitplane::Context::Magnitude;
        else if (context == "neighbors") context_ = signed_bitplane::Context::Neighbors;
        else throw std::runtime_error("MMSE bitplane_context must be plane, magnitude or neighbors");
    }
    bool paperEntropy_ = false;
    bool topology_ = false;
    bool fineRing_ = false;
    bool relations_ = false;
    int threshold_ = 32;
    std::uint64_t topologySeed_ = 1;
    signed_bitplane::Context context_ = signed_bitplane::Context::Magnitude;

    AlgorithmInfo info() const override {
        return AlgorithmInfo{
            .name = "mmse_cq",
            .version = 1,
            .primitive = PrimitiveKind::Vertex,
            .selection = SelectionKind::ConnectivityDriven,
        };
    }

    /// The conquest engine performs its own frontier BFS and vertex removal, so
    /// the framework's select/eliminate chain must do nothing (same contract as
    /// the plain conquest algorithm).
    EliminateOperatorPtr eliminate() const override {
        return std::make_shared<ConquestNoopEliminateOperator>();
    }

    /// Encoder collector: conquest traversal + MMSE paper predictor stack.
    CollectorFactory collectorFactory() const override {
        const bool enabled = paperEntropy_;
        const auto context = context_;
        const bool topology = topology_;
        const int threshold = threshold_;
        const auto seed = topologySeed_;
        const bool relations = relations_;
        const bool fineRing = fineRing_;
        return [enabled, context, topology, threshold, seed, relations, fineRing]() -> std::shared_ptr<SymbolCollectOperator> {
            auto collector = std::make_shared<MMSECQuestSymbolCollectOperator>();
            collector->setPaperEntropy(enabled, context);
            collector->setRecursiveTopology(topology, threshold, seed);
            collector->setPaperRelations(relations);
            collector->setFineRingPrediction(fineRing);
            return collector;
        };
    }
};

/// @brief Decoder triple: MMSE reader (mode trailer) + reconstructor (same hooks).
DecompressOperatorTriple makeMMSECQuestTriple() {
    auto reader = std::make_shared<MMSECQuestSymbolReadOperator>();
    auto state = std::make_shared<mmse_pred::MMSECQuestGeomState>();
    // ⚠️ 重建器必须持有 reader 的同一个实例（层游标在 reader 上）。
    auto reconstructor = std::make_shared<MMSECQuestReconstructOperator>(reader, state);
    auto deserializer = std::make_shared<ColumnarDeserializeOperator>(reader, reconstructor);
    return {deserializer, reader, reconstructor};
}

struct MMSECQuestDecompressRegistrations {
    MMSECQuestDecompressRegistrations() {
        // 解压按 metadata 里的 `info().name` 分发；三个后端都注册，缺哪个都会
        // 回落到框架 Default 解压器（它读不懂 conquest layout 的位流）。
        NamedDecompressOperatorRegistrar("mmse_cq", Backend::Binary, makeMMSECQuestTriple);
        NamedDecompressOperatorRegistrar("mmse_cq", Backend::Columnar, makeMMSECQuestTriple);
        NamedDecompressOperatorRegistrar("mmse_cq", Backend::Unified, makeMMSECQuestTriple);
    }
} g_mmseCqDecompressRegistrations;  // 静态实例：链接时完成注册（漏掉它会静默回落 Default 解压器）

}  // namespace
}  // namespace pmsf

PMSF_REGISTER_SIMPLIFICATION_ALGORITHM(pmsf::MMSECQuestAlgorithm);
