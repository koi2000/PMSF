/*
 * @file PCLTTMAlgorithm.cpp
 * @brief PCLTTM (Alliez & Desbrun, SIGGRAPH 2001, "Progressive compression for
 *        lossless transmission of triangle meshes") 接入契约。
 *
 * 论文 Tbl. tab:hook_loc 里 PCLTTM 重写的是 H1（42 行）+ H3（193 行）。
 *
 *   - H1 removability = DegreeThree → DegreeSix → Basic → UnifiedManifold
 *                       （与历史实现同序）
 *   - H8 边界         = configurePclttmDefaultPMSFBoundary
 *   - 管线微调        = select-then-extract 种子 + 每轮度重算 + 上下文层级
 *   - 协议            = pclttm_tag（论文的 valence 约束重三角化由该协议承载；
 *                       encoder 端的 vertex tag evolution 与 decoder 端的
 *                       fusedFaceSym 是配对关系，缺一不可）
 *   - 其余全部框架默认
 *
 * 字节核实：与 legacy 路径在同一份二进制内 A/B 相同（见
 * docs/refactor/ACCEPTANCE_2026-09-15.md §5.5）。
 */
#include "algorithm/SimplificationAlgorithmRegistry.h"
#include "PCLTTM/PCLTTMConquestOperators.h"
#include "Conquest/elimination/ConquestNoopEliminateOperator.h"
#include "decompress/ColumnarDeserializeOperator.h"
#include "storage/PMSFCompactBinEnvelope.h"
#if defined(COLUMNAR_SUPPORT)
#include "decompress/PaperDeserializeOperator.h"
#endif

#include "DefaultPMSFBoundaryImpl.h"
#include "DegreeSixIsRemovableOperator.h"
#include "DegreeThreeIsRemovableOperator.h"
#if defined(COLUMNAR_SUPPORT)
#include "algorithm/RetriProtocolRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"
#include "storage/PMSFSymbolStorageAdapter.h"
#include "storage/PaperStreamReaderV2.h"
#endif
#include "core/CompressPipeline.h"
#include "operator/BasicIsRemovableOperator.h"
#include "operator/UnifiedManifoldIsRemovableOperator.h"

#include <cstdlib>
#include <memory>

namespace pmsf {
namespace {

    /// Phase 1 校验用：把注册名临时改成 "<name>_probe"，就能在同一份二进制里
    /// 让 `algorithm` 字段分别命中 legacy 过渡桥与插件路径，做真正的 A/B
    /// （跨二进制的基线比对受 TU 集合敏感性影响，不可靠）。
    /// Phase 4 删除 legacy 桥后本开关一并移除。
    static std::string registeredName(const char* real, const char* probeEnv,
                                      const char* probeName) {
        const char* v = std::getenv(probeEnv);
        return (v && *v) ? std::string(probeName) : std::string(real);
    }

class PCLTTMAlgorithm final : public ISimplificationAlgorithm {
  public:
    void configure(const nlohmann::json& options) override {
        const auto geometry = options.value("geometry", std::string("cartesian"));
        if (geometry != "cartesian" && geometry != "frenet-two-pass") {
            throw std::runtime_error("PCLTTM geometry must be cartesian or frenet-two-pass");
        }
        frenet_ = geometry == "frenet-two-pass";
        const auto execution=options.value("execution",std::string("legacy-tag"));
        if(execution!="legacy-tag"&&execution!="paper-conquest")throw std::runtime_error("PCLTTM execution must be legacy-tag or paper-conquest");
        conquest_=execution=="paper-conquest";
        packing_=options.value("code_packing",false);
        if(packing_&&!conquest_)throw std::runtime_error("PCLTTM code_packing requires paper-conquest");
        if(conquest_&&!frenet_)throw std::runtime_error("PCLTTM paper-conquest requires frenet-two-pass geometry");
    }

    EliminateOperatorPtr eliminate() const override {
        return conquest_?std::make_shared<ConquestNoopEliminateOperator>():ISimplificationAlgorithm::eliminate();
    }
    CollectorFactory collectorFactory() const override {
        if(!conquest_)return {};
        const bool packing=packing_;
        return [packing](){return std::make_shared<pclttm_conquest::Collector>(packing);};
    }
    DecompressOptions conquestOptions(Backend backend,const std::string& input,const std::string& output) const {
        auto reader=std::make_shared<pclttm_conquest::Reader>();
        auto reconstruct=std::make_shared<pclttm_conquest::Reconstructor>(reader);
        std::shared_ptr<DeserializeOperator> deserialize;
#if defined(COLUMNAR_SUPPORT)
        if(backend==Backend::Paper)deserialize=std::make_shared<PaperDeserializeOperator>(reader,reconstruct);
        else
#endif
        deserialize=std::make_shared<ColumnarDeserializeOperator>(reader,reconstruct);
        return DecompressOptions::Builder().setDeserializeOperator(deserialize).setPath(input).setOutputPath(output).build();
    }

    void compress(const PMSFConfig& cfg) override {
        if (frenet_) {
            if ((cfg.backend != Backend::Columnar && cfg.backend != Backend::Paper) ||
                !cfg.quantization || !cfg.prediction || cfg.segmentation) {
                throw std::runtime_error("PCLTTM Frenet requires quantized, predicted, unsegmented Columnar/Paper encoding");
            }
            if (cfg.backend == Backend::Paper && !cfg.extra.count("PMSF_PAPERSTREAM_V2")) {
                throw std::runtime_error("PCLTTM Frenet Paper encoding requires explicit paper_storage");
            }
        }
        ISimplificationAlgorithm::compress(cfg);
    }

#if defined(COLUMNAR_SUPPORT)
    DecompressOptions makeDecompressOptions(Backend backend,const std::string& input,
                                            const std::string& output) const override {
        if(backend==Backend::Columnar) {
            const auto stem=pmsf::paperStorageStripArtifactStemRole(pmsf::paperStorageStripExtension(input));
            PMSFCompactBinEnvelope envelope;
            if(readPMSFCompactBinEnvelopeFile(pmsf::paperCompactBinPath(stem),"[PCLTTM probe]",envelope) && envelope.payload.size()>=8) {
                const auto* begin=envelope.payload.data()+4;PaperCompactPayloadHeader header;
                if(readPaperCompactPayloadHeader(begin,envelope.payload.data()+envelope.payload.size(),true,header)&&header.blobLayout==kPaperBlobLayoutConquestRC)
                    return conquestOptions(backend,input,output);
            }
        }
        if (backend == Backend::Paper && !pmsf::paperRetriEnabled()) {
            const auto path = input.ends_with("_paper.parquet") ? input :
                pmsf::paperStorageStripExtension(input)+"_paper.parquet";
            const pmsf::PaperStreamReaderV2 probe(path);
            const auto* object = probe.v2Object();
            if(object && object->algorithm==info().name && object->blobLayout==kPaperBlobLayoutConquestRC)
                return conquestOptions(backend,input,output);
            if (object && object->algorithm==info().name && object->blobLayout==pmsf::kPaperBlobLayoutCodec016) {
                // The artifact's original protocol is declared by its layout;
                // the reader selects sv3/sv4 or the named typed geometry stream.
                auto deserializer=pmsf::makeRetriPaperDeserializer(retriangulationProtocol(),{},{});
                if (!deserializer) throw std::runtime_error("PCLTTM Paper protocol is not registered");
                return DecompressOptions::Builder().setDeserializeOperator(deserializer)
                    .setPath(input).setOutputPath(output).build();
            }
        }
        return ISimplificationAlgorithm::makeDecompressOptions(backend,input,output);
    }
#endif

    void decompress(const PMSFConfig& cfg) override {
        auto self=cfg;
#if defined(COLUMNAR_SUPPORT)
        if(cfg.backend==Backend::Columnar) {
            const auto stem=pmsf::paperStorageStripArtifactStemRole(pmsf::paperStorageStripExtension(cfg.input));
            PMSFCompactBinEnvelope envelope;
            if(readPMSFCompactBinEnvelopeFile(pmsf::paperCompactBinPath(stem),"[PCLTTM probe]",envelope)&&envelope.payload.size()>=8) {
                const auto* begin=envelope.payload.data()+4;PaperCompactPayloadHeader header;
                if(readPaperCompactPayloadHeader(begin,envelope.payload.data()+envelope.payload.size(),true,header)&&header.blobLayout==kPaperBlobLayoutConquestRC) {
                    const auto before=conquest_;conquest_=true;
                    try {ISimplificationAlgorithm::decompress(self);} catch(...) {conquest_=before;throw;}
                    conquest_=before;return;
                }
            }
        }
#endif
        ISimplificationAlgorithm::decompress(cfg);
    }

    AlgorithmInfo info() const override {
        return AlgorithmInfo{
            .name = registeredName("pclttm", "PMSF_PROBE_PCLTTM", "pclttm_probe"),
            .version = 1,
            .primitive = PrimitiveKind::Vertex,
            .selection = conquest_?SelectionKind::ConnectivityDriven:SelectionKind::GeometryDriven,
        };
    }

    IsRemovableChain removability() const override {
        IsRemovableChain chain;
        chain.push_back(std::make_shared<DegreeThreeIsRemovableOperator>());
        chain.push_back(std::make_shared<DegreeSixIsRemovableOperator>());
        chain.push_back(std::make_shared<BasicIsRemovableOperator>());
        chain.push_back(std::make_shared<UnifiedManifoldIsRemovableOperator>());
        return chain;
    }

    BoundaryConfigurator boundaryConfigurator() const override {
        return [](CompressPipelineConfig& cfg) { configurePclttmDefaultPMSFBoundary(cfg); };
    }

    PipelineTuner pipelineTuner() const override {
        const bool conquest=conquest_;
        return [conquest](CompressPipelineConfig& cfg, Backend /*backend*/) {
            if(conquest) {cfg.headerCarryPrediction=true;return;}
            cfg.seedStrategy = CompressPipelineConfig::SeedStrategy::SelectThenExtract;
            cfg.recomputeVertexDegreesOnce = true;
            cfg.trackContextLevel = true;
            cfg.updateAfterEliminate = false;
        };
    }

    /// 论文的 valence 约束重三角化协议。
    /// P18.1a：本算法的残差流按整数位平面设计，**没有未量化（F32）码流布局**
    /// —— `*Tag` 采集器的 `exportRC` 会 `throw`。声明出来，让框架在 config
    /// 派发期就给出可读错误，而不是白跑一遍再深抛（实测约 1 s）。
    bool requiresQuantization() const override { return true; }

    std::string retriangulationProtocol() const override {
        return conquest_?std::string{}:std::string("pclttm_tag");
    }

    /// 量化位宽（Phase 0 口径对齐）。
    ///
    /// 此前本算法**没有**重写本方法，于是量化位宽取 `RetriProtocolTuning` 的
    /// 结构体默认值 11，且 `quantBitsEnv`/`altQuantBitsEnv` 都是空串 ——
    /// 意味着 **没有任何环境变量能改它**（实测：`PMSF_DEFAULT_QUANT_BITS=10`
    /// 与 `PMSF_PCLTTM_QUANT_BITS=10` 都无效，产物仍是 q=11）。
    /// 而 `PCLTTM.json` 里的 `PMSF_PCLTTM_QUANT_BITS=11` 因此是一把**死键**。
    ///
    /// 论文 Table 1 的 fandisk 是 10-bit，所以这里把该键接通，
    /// `defaultQuantBits` 保持 11 以免改变未显式配置时的既有行为。
    RetriProtocolTuning retriangulationTuning() const override {
        RetriProtocolTuning t;
        t.defaultQuantBits = 11;
        t.localFrameResidual = frenet_;
        if (frenet_) t.paperArtifactAlgorithm=info().name;
        t.quantBitsEnv = "PMSF_PCLTTM_QUANT_BITS";
        return t;
    }

  private:
    bool frenet_ = false;
    bool conquest_ = false;
    bool packing_ = false;
};

}  // namespace
}  // namespace pmsf

PMSF_REGISTER_SIMPLIFICATION_ALGORITHM(pmsf::PCLTTMAlgorithm);
