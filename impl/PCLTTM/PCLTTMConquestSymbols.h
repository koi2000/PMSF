// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "PCLTTM/PCLTTMFrenetLayer.h"
#include "conquest/ConquestBlobFormat.h"
#include "core/Symbol.h"
#include <cstring>

namespace pmsf::pclttm_conquest {
inline constexpr char kMeta[]="pclttm.conquest.layer";
inline constexpr char kGeometry[]="paper_role/CfQ/pclttm_conquest_pfr1";
inline constexpr std::size_t kMetaBytes=116;
inline constexpr std::size_t kPackedMetaBytes=120;
struct Layer {
    conquest_blob::LayerRecord topology;
    std::vector<std::uint8_t> geometry;
    bool cleaning=false;
    bool packed=false;
    std::size_t expandedSymbols=0;
};
struct Layers {
    double step=0.;
    double lower[3]{};
    unsigned quantBits=12;
    std::vector<Layer> layers; // decoder order
};
inline void put(std::vector<std::uint8_t>& out,std::uint32_t value){pclttm_frenet::layer_detail::put(out,value);}
inline void putDouble(std::vector<std::uint8_t>& out,double value){std::uint64_t bits;std::memcpy(&bits,&value,8);for(int byte=0;byte<8;++byte)out.push_back(static_cast<std::uint8_t>(bits>>(8*byte)));}
inline const std::vector<std::uint8_t>& binary(const PaperRoundSymbols& delta,const char* name) {
    const auto it=delta.side.find(name);
    if(it==delta.side.end()||it->second.size()!=1||!std::holds_alternative<std::vector<std::uint8_t>>(it->second.front()))
        throw std::runtime_error("PCLTTM conquest missing binary layer stream");
    return std::get<std::vector<std::uint8_t>>(it->second.front());
}
inline std::uint32_t checksum(std::span<const std::uint8_t> metadata,const std::deque<SymbolCode>& codes,std::span<const std::uint8_t> geometry) {
    auto crc=crc32(0,metadata.data(),static_cast<uInt>(metadata.size()));
    for(auto code:codes){std::uint8_t bytes[4];for(int b=0;b<4;++b)bytes[b]=static_cast<std::uint8_t>(std::uint32_t(code)>>(8*b));crc=crc32(crc,bytes,4);}
    return static_cast<std::uint32_t>(crc32(crc,geometry.data(),static_cast<uInt>(geometry.size())));
}
inline PMSFSymbolRecord toSymbols(const Layers& source,std::size_t i) {
    const auto& layer=source.layers.at(i);
    auto row=makePMSFSymbolRecord(0,0,static_cast<int>(source.layers.size()-1-i));
    for(auto code:layer.topology.symbols)row.delta.facetSymbols.push_back(layer.cleaning?(code==0?3:4):code);
    const unsigned version=layer.packed?2u:1u;
    std::vector<std::uint8_t> meta={'P','C','L',static_cast<std::uint8_t>('0'+version)};put(meta,version);put(meta,static_cast<std::uint32_t>(source.layers.size()-1-i));
    put(meta,static_cast<std::uint32_t>(source.layers.size()));put(meta,(layer.cleaning?1u:0u)|(layer.packed?2u:0u));put(meta,source.quantBits);
    putDouble(meta,source.step);for(auto x:source.lower)putDouble(meta,x);
    for(const auto& point:layer.topology.seedPos)for(auto x:point)putDouble(meta,x);
    put(meta,static_cast<std::uint32_t>(row.delta.facetSymbols.size()));put(meta,static_cast<std::uint32_t>(layer.topology.deltas.size()));
    if(layer.packed)put(meta,static_cast<std::uint32_t>(layer.expandedSymbols));
    put(meta,checksum(meta,row.delta.facetSymbols,layer.geometry));
    if(meta.size()!=(layer.packed?kPackedMetaBytes:kMetaBytes))throw std::runtime_error("internal conquest metadata size");
    row.delta.side[kMeta].emplace_back(std::move(meta));row.delta.side[kGeometry].emplace_back(layer.geometry);
    return row;
}
inline Layers fromSymbols(const PMSFSymbolTable& symbols) {
    Layers out;
    unsigned originalRounds=0;
    for(std::size_t i=0;i<symbols.size();++i) {
        const auto& row=symbols[i];const auto& delta=row.delta;
        if(row.objectId!=0||row.partitionId!=0||row.lodId!=static_cast<int>(i)||delta.side.size()!=2||
            !delta.vertexSymbols.empty()||!delta.triangulationEdgeSymbols.empty()||!delta.flipEdgeSymbols.empty()||
            !delta.removedVertexCoords.empty()||!delta.removedVertexResiduals.empty()||!delta.shiftedVertexCoords.empty()||!delta.shiftedVertexResiduals.empty())
            throw std::runtime_error("invalid PCLTTM conquest role mapping/order");
        const auto& meta=binary(delta,kMeta);const auto& geo=binary(delta,kGeometry);
        const bool packedHeader=meta.size()==kPackedMetaBytes&&std::memcmp(meta.data(),"PCL2",4)==0;
        if(!packedHeader&&(meta.size()!=kMetaBytes||std::memcmp(meta.data(),"PCL1",4)))throw std::runtime_error("invalid PCLTTM conquest layer header");
        conquest_blob::detail::Reader reader{reinterpret_cast<const char*>(meta.data()+4),reinterpret_cast<const char*>(meta.data()+meta.size()),false};
        const auto version=reader.u32(),round=reader.u32(),rounds=reader.u32(),kind=reader.u32(),qbits=reader.u32();
        const auto step=reader.f64();double lower[3];for(auto& x:lower)x=reader.f64();
        Layer layer;layer.cleaning=(kind&1u)!=0;layer.packed=(kind&2u)!=0;
        for(auto& point:layer.topology.seedPos)for(auto& x:point){x=reader.f64();if(!std::isfinite(x))throw std::runtime_error("non-finite conquest seed");}
        const auto count=reader.u32(),points=reader.u32();
        layer.expandedSymbols=packedHeader?reader.u32():count;
        const auto crc=reader.u32();
        if(reader.bad||version!=(packedHeader?2u:1u)||layer.packed!=packedHeader||!rounds||rounds>65536||round>=rounds||kind>3||
            layer.cleaning!=(round%2!=0)||(layer.packed&&layer.cleaning)||i!=rounds-1-round||
            qbits<8||qbits>16||!std::isfinite(step)||step<=0||!std::all_of(std::begin(lower),std::end(lower),[](double x){return std::isfinite(x);})||
            count!=delta.facetSymbols.size()||count>pclttm_frenet::layer_detail::kMaxPoints*8||points>pclttm_frenet::layer_detail::kMaxPoints||
            layer.expandedSymbols<count||layer.expandedSymbols>pclttm_frenet::layer_detail::kMaxPoints*8)
            throw std::runtime_error("invalid PCLTTM conquest metadata dimensions");
        if(crc!=checksum(std::span(meta).first(meta.size()-4),delta.facetSymbols,geo))throw std::runtime_error("PCLTTM conquest layer checksum mismatch");
        if(i==0){out.step=step;std::copy(std::begin(lower),std::end(lower),out.lower);out.quantBits=qbits;originalRounds=rounds;}
        else if(out.step!=step||!std::equal(std::begin(lower),std::end(lower),out.lower)||out.quantBits!=qbits||rounds!=originalRounds)
            throw std::runtime_error("PCLTTM conquest inconsistent layer grid/count");
        for(auto code:delta.facetSymbols){
            if(code<0||code>4||(layer.cleaning&&code<3))throw std::runtime_error("invalid PCLTTM conquest valence code");
            layer.topology.symbols.push_back(layer.cleaning?(code==3?0:1):code);
        }
        const auto geometry=pclttm_frenet::decodeLayer(geo,points);
        layer.topology.deltas.assign(geometry.coefficients.begin(),geometry.coefficients.end());layer.geometry=geo;
        out.layers.push_back(std::move(layer));
    }
    return out;
}
inline std::vector<std::uint8_t> encodeTopology(const std::deque<SymbolCode>& codes) {
    std::vector<char> staging(codes.size()+64);std::size_t cursor=0;rangecoder coder{};coder.p_data=staging.data();coder.p_dataOffset=&cursor;start_encoding(&coder,0,0);
    qsmodel model{};initqsmodel(&model,5,10,1<<9,nullptr,1);
    for(auto code:codes){if(code<0||code>4){deleteqsmodel(&model);throw std::runtime_error("invalid conquest symbol for arithmetic coder");}
        int f=0,c=0;qsgetfreq(&model,code,&f,&c);encode_shift(&coder,f,c,10);qsupdate(&model,code);}
    done_encoding(&coder);deleteqsmodel(&model);return {staging.begin(),staging.begin()+cursor};
}
inline std::deque<SymbolCode> decodeTopology(std::span<const std::uint8_t> bytes,std::size_t count) {
    if(bytes.size()<3||count>pclttm_frenet::layer_detail::kMaxPoints*8)throw std::runtime_error("truncated conquest arithmetic stream");
    std::vector<char> staging(bytes.begin(),bytes.end());std::size_t cursor=0;rangecoder coder{};coder.p_data=staging.data();coder.p_dataOffset=&cursor;
    if(start_decoding(&coder)!=0)throw std::runtime_error("invalid conquest arithmetic start");coder.p_dataEnd=staging.data()+staging.size();
    qsmodel model{};initqsmodel(&model,5,10,1<<9,nullptr,0);std::deque<SymbolCode> codes;
    for(std::size_t i=0;i<count;++i){const auto fraction=decode_culshift(&coder,10);if(coder.inputError||fraction>=1024){deleteqsmodel(&model);throw std::runtime_error("truncated conquest symbols");}
        const int code=qsgetsym(&model,fraction);int f=0,c=0;qsgetfreq(&model,code,&f,&c);decode_update(&coder,f,c,1024);qsupdate(&model,code);codes.push_back(code);}
    deleteqsmodel(&model);if(coder.inputError)throw std::runtime_error("truncated conquest symbols");return codes;
}
inline std::vector<std::uint8_t> encodeNative(const Layers& source) {
    const bool packed=std::any_of(source.layers.begin(),source.layers.end(),[](const auto& layer){return layer.packed;});
    std::vector<std::uint8_t> out={'P','C','T',static_cast<std::uint8_t>(packed?'2':'1')};put(out,static_cast<std::uint32_t>(source.layers.size()));
    for(std::size_t i=0;i<source.layers.size();++i) {
        const auto row=toSymbols(source,i);const auto& meta=binary(row.delta,kMeta);const auto& geo=binary(row.delta,kGeometry);const auto topo=encodeTopology(row.delta.facetSymbols);
        out.insert(out.end(),meta.begin(),meta.end());put(out,static_cast<std::uint32_t>(topo.size()));put(out,static_cast<std::uint32_t>(geo.size()));
        out.insert(out.end(),topo.begin(),topo.end());out.insert(out.end(),geo.begin(),geo.end());
    }
    return out;
}
inline Layers decodeNative(std::span<const std::uint8_t> bytes) {
    using pclttm_frenet::layer_detail::get;
    if(bytes.size()<8||std::memcmp(bytes.data(),"PCT",3)||(bytes[3]!='1'&&bytes[3]!='2'))throw std::runtime_error("invalid conquest native header");
    const bool packed=bytes[3]=='2';
    const auto rounds=get(bytes.data()+4);if(rounds>65536)throw std::runtime_error("invalid conquest native count");
    std::size_t offset=8;PMSFSymbolTable rows;
    for(std::size_t i=0;i<rounds;++i) {
        if(bytes.size()-offset<kMetaBytes+8)throw std::runtime_error("truncated conquest layer");
        auto row=makePMSFSymbolRecord(0,0,static_cast<int>(i));
        const auto metaBytes=std::memcmp(bytes.data()+offset,"PCL2",4)==0?kPackedMetaBytes:kMetaBytes;
        if((metaBytes==kPackedMetaBytes&&!packed)||bytes.size()-offset<metaBytes+8)throw std::runtime_error("invalid packed conquest native layer header");
        const auto meta=bytes.subspan(offset,metaBytes);offset+=metaBytes;
        const auto topo=get(bytes.data()+offset),geo=get(bytes.data()+offset+4);offset+=8;
        if(topo>bytes.size()-offset||geo>bytes.size()-offset-topo)throw std::runtime_error("invalid conquest payload lengths");
        row.delta.facetSymbols=decodeTopology(bytes.subspan(offset,topo),get(meta.data()+104));offset+=topo;
        row.delta.side[kMeta].emplace_back(std::vector<std::uint8_t>(meta.begin(),meta.end()));
        row.delta.side[kGeometry].emplace_back(std::vector<std::uint8_t>(bytes.begin()+offset,bytes.begin()+offset+geo));offset+=geo;
        rows.push_back(std::move(row));
    }
    if(offset!=bytes.size())throw std::runtime_error("unexpected conquest native trailing bytes");
    return fromSymbols(rows);
}
}
