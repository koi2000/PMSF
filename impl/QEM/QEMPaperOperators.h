// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "QEMPaperEngine.h"
#include "QEMProgressiveArtifact.h"
#include "core/OperationSequencePipeline.h"
#include "config/PMSFConfig.h"
#ifdef COLUMNAR_SUPPORT
#include "QEMPaperRelationalMapping.h"
#endif

#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace pmsf::qem_paper1997 {

struct Configuration {
    Options engine;
    bool geometry64 = true;
};

inline Configuration configuration(const nlohmann::json& options) {
    Configuration out;
    out.engine.targetFaces=options.value("target_faces",250);
    out.engine.pairThreshold=options.value("pair_threshold",0.0);
    out.engine.boundaryWeight=options.value("boundary_weight",1000.0);
    out.engine.preventInversion=options.value("prevent_inversion",true);
    out.engine.lodBatchSize=options.value("lod_batch_size",256);
    const auto precision=options.value("geometry_precision",std::string("float64"));
    if (precision!="float64"&&precision!="float32") throw std::runtime_error("QEM paper geometry_precision must be float64 or float32");
    out.geometry64=precision=="float64";
    return out;
}

class SymbolCollector final : public qem_refstyle::CollapseRecorder {
  public:
    std::vector<Record> records;
    void onCollapse(const Record& record) override { records.push_back(record); }
    qem_prog::ArtifactStreams finish(const Engine& engine,int originalV,int originalF,bool f64,
                                    qem_prog::EncodeStats& stats) const {
        std::vector<int> originalIds;
        const auto base=engine.snapshot(&originalIds);
        qem_prog::EncodeOptions options;options.geomF64=f64;
        qem_prog::ArtifactStreams streams;std::string error;
        if (!qem_prog::buildLogicalStreams(base,originalIds,records,originalV,originalF,options,&streams,&stats,&error)) {
            throw std::runtime_error("QEM paper symbol collector: "+error);
        }
        return streams;
    }
};

inline void compress(const PMSFConfig& cfg,const Configuration& settings) {
    if (cfg.quantizationSet&&cfg.quantization) throw std::runtime_error("QEM paper1997 requires unquantized input; specify quantization=false");
    if (cfg.backend!=Backend::Paper&&cfg.backend!=Backend::Binary) throw std::runtime_error("QEM paper1997 supports paper or binary canonical storage");
    Mesh input;std::string error;
    if (!qem_refstyle::loadOffMesh(cfg.input,&input,&error)) throw std::runtime_error(error);
    Engine engine(input,settings.engine);
    SymbolCollector collector;
    const auto operationStats=runOperationSequence<Candidate>(
        [&](auto& entry){return engine.select(entry);},
        [&](const auto& entry){return engine.removable(entry);},
        [&](const auto& entry){return engine.eliminate(entry);},
        [&](const Record& symbols){collector.onCollapse(symbols);},
        [&](const auto& entry){engine.update(entry);});
    if (engine.stalled()||engine.liveFaceCount()>settings.engine.targetFaces) throw std::runtime_error("QEM paper1997 stalled above the requested face budget");
    qem_prog::EncodeStats encoded;
    const auto streams=collector.finish(engine,input.vertices.size(),input.faces.size(),settings.geometry64,encoded);
    if (cfg.backend==Backend::Paper) {
#ifdef COLUMNAR_SUPPORT
        const auto stem=std::filesystem::path(cfg.output).replace_extension().string();
        writePaperRelations(stem+"_paper.parquet",streams,encoded,settings.geometry64);
#else
        throw std::runtime_error("QEM paper1997 PaperStream requires columnar support");
#endif
    } else {
        std::vector<std::uint8_t> bytes;
        qem_prog::EncodeOptions options;options.geomF64=settings.geometry64;
        if (!qem_prog::serializeOwnFormat(streams,options,&bytes,&encoded,&error)) throw std::runtime_error(error);
        std::ofstream target(cfg.output,std::ios::binary);
        target.write(reinterpret_cast<const char*>(bytes.data()),bytes.size());
        if (!target) throw std::runtime_error("QEM canonical artifact write failed");
    }
    std::cout<<"QEM_PAPER1997_ENCODE algorithm=qem core_driver=OperationSequencePipeline"
             <<" original_vertices="<<input.vertices.size()<<" original_faces="<<input.faces.size()
             <<" base_faces="<<engine.liveFaceCount()<<" operations="<<operationStats.eliminated
             <<" rejected="<<operationStats.rejected<<" lods="<<encoded.rounds
             <<" policy=additive_quadrics/global_heap initial_threshold="<<settings.engine.pairThreshold
             <<" boundary_weight="<<settings.engine.boundaryWeight<<" geometry="<<(settings.geometry64?"float64":"float32")<<"\n";
}

inline bool isOwnArtifact(const std::string& path) {
    std::ifstream source(path,std::ios::binary);char magic[8]={};source.read(magic,8);
    return source.gcount()==8&&std::string(magic,8)=="QEMPROG1";
}
inline bool isPaperArtifact(const std::string& path) {
#ifdef COLUMNAR_SUPPORT
    std::ifstream source(path,std::ios::binary);char magic[4]={};source.read(magic,4);
    if (source.gcount()!=4||std::string(magic,4)!="PAR1") return false;
    PaperStreamReaderV2 probe(path);
    const auto base=probe.readPrefix(0,0);
    return base.object.algorithm=="qem"&&base.object.schemeVersion==1997&&
           base.object.blobLayout==kPaperBlobLayoutQEMProgressive;
#else
    return false;
#endif
}

inline void decompress(const PMSFConfig& cfg) {
    Mesh mesh;qem_prog::DecodeStats decoded;
    qem_prog::DecodeOptions options;options.maxAppliedRounds=cfg.maxLod;
    std::string error;std::uint64_t bytesRead=0,rowGroups=0;
    if (cfg.backend==Backend::Paper) {
#ifdef COLUMNAR_SUPPORT
        auto path=cfg.input;
        if (!std::filesystem::is_regular_file(path)) path=std::filesystem::path(path).replace_extension().string()+"_paper.parquet";
        qem_prog::SeamStreams streams;
        std::vector<std::uint8_t> declaredHeader, prefix;
        if (!readPaperRelations(path,cfg.maxLod,&streams,&error,&bytesRead,&rowGroups,&declaredHeader)||
            !qem_prog::buildOwnFormatFromStreams(streams,&prefix,&error)) throw std::runtime_error(error);
        // Preserve the original full-stream counters: the existing decoder
        // recognizes max_lod/byte-prefix stopping instead of interpreting this
        // partial set of rows as a complete mesh with inconsistent counts.
        std::copy(declaredHeader.begin(),declaredHeader.end(),prefix.begin());
        if (!qem_prog::decodeProgressiveArtifact(prefix.data(),prefix.size(),options,&mesh,&decoded,&error)) throw std::runtime_error(error);
#else
        throw std::runtime_error("QEM paper1997 PaperStream requires columnar support");
#endif
    } else {
        std::ifstream source(cfg.input,std::ios::binary);
        std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(source)),{});
        if (!qem_prog::decodeProgressiveArtifact(bytes.data(),bytes.size(),options,&mesh,&decoded,&error)) throw std::runtime_error(error);
        bytesRead=bytes.size();
    }
    if (!qem_refstyle::writeOffMesh(cfg.output,mesh,&error,0)) throw std::runtime_error(error);
    std::cout<<"QEM_PROGRESSIVE_DECODE "<<decoded.summary()<<" bytes_read="<<bytesRead<<" row_groups_read="<<rowGroups<<"\n";
}
}  // namespace pmsf::qem_paper1997
