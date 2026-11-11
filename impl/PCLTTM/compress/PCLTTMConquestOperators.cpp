// SPDX-License-Identifier: Apache-2.0
#include "PCLTTM/PCLTTMConquestOperators.h"
#include "core/CollectorLayoutPluginRegistry.h"
#include "PCLTTM/PCLTTMPatchPatterns.h"
#include "conquest/ConquestMeshClone.h"
#include "conquest/ConquestReplayTrace.h"
#include <iostream>
#include "Configuration.h"

namespace pmsf::pclttm_conquest {

void Collector::init(std::shared_ptr<MCGAL::Mesh> mesh,std::vector<MCGAL::Halfedge*> seeds,PMSFHeader header) {
    SymbolCollectOperator::init(mesh,seeds,header);
    if(!mesh_||!mesh_->enableQuantization||mesh_->i_nbQuantBits<8||mesh_->i_nbQuantBits>16||!(mesh_->f_quantStep>0))
        throw std::runtime_error("PCLTTM conquest requires a valid q8–q16 grid");
    packed_.reset();encoder_={};encoder_.step=mesh_->f_quantStep;encoder_.quantBits=mesh_->i_nbQuantBits;
    for(int k=0;k<3;++k)encoder_.lower[k]=mesh_->bboxMin[k];
    for(auto* vertex:mesh_->vertices())if(vertex&&!vertex->isRemoved()) {
        auto cells=mesh_->floatPosToInt(vertex->point());double xyz[3];
        for(int k=0;k<3;++k){cells[k]=std::clamp(cells[k],0,(1<<encoder_.quantBits)-1);
            xyz[k]=float(encoder_.lower[k])+(float(cells[k])+.5f)*float(encoder_.step);}
        vertex->setPoint(MCGAL::Point(xyz[0],xyz[1],xyz[2]));
    }
}
void Collector::collect(MCGAL::Halfedge* seed) {
    if(!seed||!seed->opposite())throw std::runtime_error("PCLTTM conquest missing oriented seed");
    packed_.reset();
    Layer layer;layer.cleaning=encoder_.layers.size()%2;
    for(int k=0;k<3;++k){layer.topology.seedPos[0][k]=seed->end_vertex()->point()[k];layer.topology.seedPos[1][k]=seed->vertex()->point()[k];}
    FrontierConquestEngine::QuantGrid grid;
    grid.step=encoder_.step;std::copy(std::begin(encoder_.lower),std::end(encoder_.lower),grid.bboxMin);
    struct Sink final : GeometrySink {std::vector<std::array<int,3>>& values;explicit Sink(std::vector<std::array<int,3>>& v):values(v){}
        void onRemovedVertex(const int q[3]) override{values.push_back({q[0],q[1],q[2]});}} sink(layer.topology.deltas);
    geometry_->patches.clear();geometry_->targetCells.clear();
    // A triangle with its reverse is not an admissible simplicial base mesh.
    // Apply the explicit four-vertex stop to both conquests, including the
    // cleanup's mark-then-delete phase where live counts have not changed yet.
    struct StopAtFour final : DecimationPredicate {
        const LocalGeometry& geometry;std::size_t before;
        StopAtFour(const LocalGeometry& g,std::size_t n):geometry(g),before(n){}
        bool budget() const {return before>geometry.patches.size()+4;}
        bool canDecimate(const FrontVertexView& v) const override {return budget()&&DecimationPredicate::canDecimate(v);}
        bool canRegulate(const FrontVertexView&) const override {return budget();}
    };
    std::size_t live=0;for(auto* vertex:mesh_->vertices())live+=vertex&&!vertex->isRemoved();
    auto layerHooks=std::make_shared<EngineHooks>(*hooks_);
    layerHooks->decimation=std::make_shared<StopAtFour>(*geometry_,live);
    FrontierConquestEngine engine(layerHooks);engine.setQuantGrid(grid);
    ConquestState state;
    const auto result=layer.cleaning?engine.regulateLayer(mesh_.get(),state,seed,&layer.topology.symbols,&sink):
        engine.decimateLayer(mesh_.get(),state,seed,&layer.topology.symbols,&sink);
    if(result.removedVertices!=static_cast<int>(geometry_->patches.size())||layer.topology.deltas.size()!=geometry_->patches.size())
        throw std::runtime_error("PCLTTM conquest geometry symbol count differs");
    auto local=pclttm_frenet::chooseLayer(geometry_->patches);layer.topology.deltas.assign(local.coefficients.begin(),local.coefficients.end());
    layer.geometry=pclttm_frenet::encodeLayer(local);
    std::cout<<"PCLTTM_CONQUEST encoder round="<<encoder_.layers.size()<<" kind="<<(layer.cleaning?"clean":"decimate")
        <<" symbols="<<layer.topology.symbols.size()<<" removed="<<result.removedVertices<<" denominator="<<local.denominator
        <<" type0="<<engine.unhandledType0Count()
        <<" whole_cell_misses="<<std::count_if(geometry_->patches.begin(),geometry_->patches.end(),[](const auto& p){return !p.wholeExact;})
        <<" target_cells="<<conquestGeometryCellSignature(geometry_->targetCells)<<"\n";
    if(engine.unhandledType0Count())throw std::runtime_error("PCLTTM conquest undefined tag configuration");
    encoder_.layers.push_back(std::move(layer));
}
void Collector::preparePacking() const {
    if(packed_)return;
    auto source=encoder_;std::reverse(source.layers.begin(),source.layers.end());
    std::size_t vertices=0;
    for(auto* vertex:mesh_->vertices())vertices+=vertex&&!vertex->isRemoved();
    for(const auto& layer:source.layers)vertices+=layer.topology.deltas.size();
    ConquestMeshClone baseline(*mesh_,vertices),candidate(*mesh_,vertices);
    auto geometry=std::make_shared<LocalGeometry>();const auto hooks=geometryHooks(geometry);
    FrontierConquestEngine::QuantGrid grid;grid.step=source.step;
    std::copy(std::begin(source.lower),std::end(source.lower),grid.bboxMin);
    for(std::size_t i=0;i<source.layers.size();++i) {
        auto& layer=source.layers[i];
        auto seed=[&](MCGAL::Mesh* mesh){
            auto* a=nearestVertex(mesh,layer.topology.seedPos[0]);auto* b=nearestVertex(mesh,layer.topology.seedPos[1]);
            auto* edge=halfedgeBetween(b,a);
            if(!edge)throw std::runtime_error("conquest packing simulation cannot recover seed");
            return edge;
        };
        auto* leftSeed=seed(baseline.get());auto* rightSeed=seed(candidate.get());
        ConquestState a,b;std::vector<ConquestDeferredGeometry> left,right;
        ConquestReplayTrace traceBefore,traceAfter;std::vector<std::uint8_t> feasible;
        ConquestDecodeControl before{&patchPatterns(),false,layer.topology.symbols.size(),&traceBefore,&feasible};
        const auto uncompressed=layer.cleaning?decodeCleaningLayer(baseline.get(),a,leftSeed,layer.topology.symbols,layer.topology.deltas,left):
            decodeLayer(baseline.get(),a,leftSeed,layer.topology.symbols,layer.topology.deltas,grid,*hooks,nullptr,&left,&before);
        if(!uncompressed.ok)throw std::runtime_error("conquest packing unpacked simulation failed discovery");
        layer.expandedSymbols=layer.topology.symbols.size();
        if(!layer.cleaning) {
            layer.topology.symbols=packConquestNulls(layer.topology.symbols,feasible);layer.packed=true;
        }
        ConquestDecodeControl after{&patchPatterns(),layer.packed,layer.expandedSymbols,&traceAfter,nullptr};
        const auto compressed=layer.cleaning?decodeCleaningLayer(candidate.get(),b,rightSeed,layer.topology.symbols,layer.topology.deltas,right):
            decodeLayer(candidate.get(),b,rightSeed,layer.topology.symbols,layer.topology.deltas,grid,*hooks,nullptr,&right,&after);
        if(!compressed.ok||(!layer.cleaning&&traceBefore.events!=traceAfter.events))
            throw std::runtime_error("conquest packing changed an exact gate/flag/tag/queue replay event");
        geometry->denominator=pclttm_frenet::layer_detail::get(layer.geometry.data()+8);
        const auto targetA=restoreDeferredConquestGeometry(left,grid,*hooks),targetB=restoreDeferredConquestGeometry(right,grid,*hooks);
        if(targetA.cellSignature!=targetB.cellSignature)throw std::runtime_error("conquest packing changed target cells");
        const auto graphA=snapshotConquestLayerTopology(leftSeed),graphB=snapshotConquestLayerTopology(rightSeed);
        if(graphA.adjacency!=graphB.adjacency||graphA.meshBoundary!=graphB.meshBoundary)
            throw std::runtime_error("conquest packing changed full rooted topology");
        for(std::size_t v=0;v<graphA.vertices.size();++v)for(int k=0;k<3;++k)
            if(graphA.vertices[v]->point()[k]!=graphB.vertices[v]->point()[k])throw std::runtime_error("conquest packing changed exact vertex coordinates");
        for(std::size_t v=0;v<graphA.vertices.size();++v)
            if(a.flag(graphA.vertices[v])!=b.flag(graphB.vertices[v])||a.sign(graphA.vertices[v])!=b.sign(graphB.vertices[v]))
                throw std::runtime_error("conquest packing changed full vertex states/tags");
        auto faceStates=[](MCGAL::Mesh* mesh,const ConquestLayerTopology& graph,const ConquestState& state) {
            std::vector<std::array<std::size_t,4>> faces;
            for(auto* face:mesh->faces())if(face&&!face->isRemoved()) {
                auto* start=face->proxyHalfedge();auto* edge=start;std::array<std::size_t,3> ids{};
                for(int corner=0;corner<3;++corner) {
                    if(!edge)throw std::runtime_error("conquest simulation has a broken face");
                    ids[corner]=graph.poolToIndex.at(static_cast<std::uint32_t>(edge->vertex()->poolId()));edge=edge->next();
                }
                if(edge!=start)throw std::runtime_error("conquest simulation has a nontriangular face");
                const auto rotation=static_cast<std::size_t>(std::min_element(ids.begin(),ids.end())-ids.begin());
                faces.push_back({ids[rotation],ids[(rotation+1)%3],ids[(rotation+2)%3],state.fflag(face)});
            }
            std::sort(faces.begin(),faces.end());return faces;
        };
        if(faceStates(baseline.get(),graphA,a)!=faceStates(candidate.get(),graphB,b))
            throw std::runtime_error("conquest packing changed full oriented faces/states");
        std::cout<<"PCLTTM_PACKING layer="<<i<<" expanded="<<layer.expandedSymbols<<" stored="<<layer.topology.symbols.size()
            <<" inferred="<<compressed.inferredNulls<<" exact_events="<<traceBefore.events.size()
            <<" target_cells="<<targetA.cellSignature<<"\n";
    }
    packed_=std::make_unique<Layers>(std::move(source));
}
Layers Collector::reversed() const {
    if(packing_){preparePacking();return *packed_;}
    auto result=encoder_;std::reverse(result.layers.begin(),result.layers.end());return result;
}
int Collector::exportToBufferWithRangeCoder(char* buffer,bool) {
    if(!buffer)return -1;const auto data=encodeNative(reversed());
    if(data.size()>BUFFER_SIZE)return -1;std::memcpy(buffer,data.data(),data.size());return static_cast<int>(data.size());
}
const PMSFSymbolTable* Collector::pmsfSymbolTable() const {
    symbols_.clear();const auto source=reversed();
    for(std::size_t i=0;i<source.layers.size();++i)symbols_.insert(symbols_.begin(),toSymbols(source,i));
    return &symbols_;
}
void Reader::install(Layers layers) {
    conquest_blob::Stream stream;stream.step=layers.step;std::copy(std::begin(layers.lower),std::end(layers.lower),stream.bboxMin);
    layerKinds_.clear();denominators_.clear();packed_.clear();expanded_.clear();
    for(auto& layer:layers.layers) {
        packed_.push_back(layer.packed);expanded_.push_back(layer.expandedSymbols?layer.expandedSymbols:layer.topology.symbols.size());
        layerKinds_.push_back(layer.cleaning);denominators_.push_back(pclttm_frenet::layer_detail::get(layer.geometry.data()+8));
        stream.layers.push_back(std::move(layer.topology));
    }
    installDecodedStream(std::move(stream));
}
void Reader::setPMSFSymbolContext(const PMSFSymbolTable* symbols,const PMSFSymbolPayloadTable* payloads) {
    SymbolReadOperator::setPMSFSymbolContext(symbols,payloads);
    if(symbols&&!symbols->empty())install(fromSymbols(*symbols));
}
void Reader::importFromRangeCodedBuffer(char* buffer,std::size_t& offset,unsigned) {
    if(!buffer||offset>bufferSize())throw std::runtime_error("PCLTTM conquest native input bounds");
    install(decodeNative({reinterpret_cast<std::uint8_t*>(buffer+offset),bufferSize()-offset}));offset=bufferSize();
}
void Reconstructor::reconstruct(std::vector<MCGAL::Vertex*>&,std::vector<MCGAL::Halfedge*>&,std::vector<MCGAL::Facet*>&) {
    const auto* layer=reader_->takeNextLayer();if(!layer)throw std::runtime_error("PCLTTM conquest layer underflow");
    const auto i=reader_->layerCursor()-1;
    auto* a=nearestVertex(mesh_.get(),layer->seedPos[0]);auto* b=nearestVertex(mesh_.get(),layer->seedPos[1]);auto* seed=halfedgeBetween(b,a);
    if(!seed)throw std::runtime_error("PCLTTM conquest cannot find declared seed");
    geometry_->denominator=reader_->denominator(i);
    FrontierConquestEngine::QuantGrid grid;grid.step=reader_->stream().step;
    std::copy(std::begin(reader_->stream().bboxMin),std::end(reader_->stream().bboxMin),grid.bboxMin);
    ConquestState state;std::vector<ConquestDeferredGeometry> geometry;
    ConquestDecodeControl control{&patchPatterns(),reader_->packed(i),reader_->expandedSymbols(i),nullptr,nullptr};
    const auto result=reader_->cleaning(i)?decodeCleaningLayer(mesh_.get(),state,seed,layer->symbols,layer->deltas,geometry):
        decodeLayer(mesh_.get(),state,seed,layer->symbols,layer->deltas,grid,*hooks_,nullptr,&geometry,reader_->packed(i)?&control:nullptr);
    if(!result.ok||geometry.size()!=layer->deltas.size()||result.consumed!=layer->symbols.size())
        throw std::runtime_error("PCLTTM conquest topology replay failed");
    const auto restored=restoreDeferredConquestGeometry(geometry,grid,*hooks_);
    std::cout<<"PCLTTM_CONQUEST decoder lod="<<i<<" kind="<<(reader_->cleaning(i)?"clean":"decimate")
        <<" symbols="<<(reader_->packed(i)?result.expandedSymbols:result.consumed)
        <<" stored_symbols="<<result.consumed<<" inferred="<<result.inferredNulls
        <<" inserted="<<result.inserted<<" target_cells="<<restored.cellSignature<<"\n";
}

namespace {
PMSFCollectorLayoutChoice choose(SymbolCollectOperator* collector,bool) {
    if(!dynamic_cast<Collector*>(collector))return {};
    return {true,kPaperBlobLayoutConquestRC,2};
}
PMSFCollectorExportResult write(SymbolCollectOperator* collector,char* data,bool quant,bool,bool) {
    auto* typed=dynamic_cast<Collector*>(collector);if(!typed)return {};
    return {true,typed->exportToBufferWithRangeCoder(data,quant)};
}
CollectorLayoutPluginRegistrar registration(choose,write);
}
}
