// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "PCLTTM/PCLTTMConquestSymbols.h"
#include "Conquest/decompress/ConquestSymbolReadOperator.h"
#include "compress/SymbolCollectOperator.h"
#include "conquest/ConquestDecodeLayer.h"
#include "conquest/ConquestMeshPrimitives.h"
#include "reconstruct/ReconstructOpertator.h"

namespace pmsf::pclttm_conquest {

class LocalGeometry final : public ResidualQuantizer {
 public:
    mutable std::vector<pclttm_frenet::PatchCode> patches;
    mutable std::vector<std::array<int,3>> targetCells;
    unsigned denominator=2;
    pclttm_frenet::Frame frame(const PatchView& view) const {
        std::vector<pclttm_frenet::Cell> ring;
        for(int i=0;i<view.neighborCount;++i) {
            pclttm_frenet::Cell q{};
            for(int k=0;k<3;++k)q[k]=toCell(view.neighborPos[3*i+k],view.bboxMin[k],view.quantStep);
            ring.push_back(q);
        }
        return pclttm_frenet::makeFrame(ring);
    }
    void transform(const int in[3],int out[3],const PatchView& view) const override {
        const auto patch=pclttm_frenet::projectPatch(frame(view),{in[0],in[1],in[2]});patches.push_back(patch);
        targetCells.push_back({in[0],in[1],in[2]});
        std::copy(patch.halfCell.begin(),patch.halfCell.end(),out);
    }
    void inverseTransform(const int in[3],int out[3],const PatchView& view) const override {
        const auto target=pclttm_frenet::restore(frame(view),{in[0],in[1],in[2]},denominator);
        std::copy(target.begin(),target.end(),out);
    }
};
class AbsoluteCellPredictor final : public GeometryPredictor {
 public:
    bool predictQuantized(const PatchView&,int out[3]) const override {out[0]=out[1]=out[2]=0;return true;}
    void predict(const PatchView& view,double out[3]) const override {
        for(int k=0;k<3;++k)out[k]=view.bboxMin[k]+.5*view.quantStep;
    }
};
inline EngineHooksPtr geometryHooks(std::shared_ptr<LocalGeometry> geometry) {
    auto hooks=std::make_shared<EngineHooks>();hooks->residualQuantizer=std::move(geometry);
    hooks->geometryPredictor=std::make_shared<AbsoluteCellPredictor>();return hooks;
}
class Collector final : public SymbolCollectOperator {
 public:
    explicit Collector(bool packing=false):packing_(packing){}
    void init(std::shared_ptr<MCGAL::Mesh> mesh,std::vector<MCGAL::Halfedge*> seeds,PMSFHeader header) override;
    void collect(MCGAL::Halfedge* seed) override;
    void collect(std::vector<MCGAL::Halfedge*> seeds) override {if(!seeds.empty())collect(seeds.front());}
    bool supportsRangeCoderExport() const override {return true;}
    int exportToBuffer(char*,bool=false,bool=false) override {throw std::runtime_error("PCLTTM conquest requires its native entropy profile");}
    int exportToBufferWithRangeCoder(char* buffer,bool=false) override;
    const PMSFSymbolTable* pmsfSymbolTable() const override;
 private:
    Layers encoder_;
    Layers reversed() const;
    void preparePacking() const;
    bool packing_=false;
    mutable std::unique_ptr<Layers> packed_;
    mutable PMSFSymbolTable symbols_;
    std::shared_ptr<LocalGeometry> geometry_=std::make_shared<LocalGeometry>();
    EngineHooksPtr hooks_=geometryHooks(geometry_);
};
class Reader final : public ConquestSymbolReadOperator {
 public:
    void importFromRangeCodedBuffer(char*,std::size_t&,unsigned) override;
    void setPMSFSymbolContext(const PMSFSymbolTable*,const PMSFSymbolPayloadTable*) override;
    bool cleaning(std::size_t i) const {return layerKinds_.at(i);}
    unsigned denominator(std::size_t i) const {return denominators_.at(i);}
    bool packed(std::size_t i) const {return packed_.at(i);}
    std::size_t expandedSymbols(std::size_t i) const {return expanded_.at(i);}
 private:
    void install(Layers layers);
    std::vector<bool> layerKinds_,packed_;
    std::vector<std::size_t> expanded_;
    std::vector<unsigned> denominators_;
};
class Reconstructor final : public ReconstructOperator {
 public:
    explicit Reconstructor(std::shared_ptr<Reader> reader):reader_(std::move(reader)){}
    void reconstruct(MCGAL::Halfedge*) override {}
    void reconstruct(std::vector<MCGAL::Vertex*>&,std::vector<MCGAL::Halfedge*>&,std::vector<MCGAL::Facet*>&) override;
 private:
    std::shared_ptr<Reader> reader_;
    std::shared_ptr<LocalGeometry> geometry_=std::make_shared<LocalGeometry>();
    EngineHooksPtr hooks_=geometryHooks(geometry_);
};
}
