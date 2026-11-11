// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "MMSE/MMSEModeEntropy.h"
#include "conquest/ConquestBlobFormat.h"
#include "core/Symbol.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace pmsf::mmse_symbols {
inline constexpr const char* kLayerMetadata = "mmse.layer_meta";
inline constexpr const char* kModeArithmetic = "mmse.mode_arithmetic";
inline constexpr std::size_t kLayerMetadataBytes = 128;

struct LayerMetadata {
    std::uint32_t encoderRound=0,rounds=0,threshold=32;
    std::uint64_t seed=1;
    signed_bitplane::Context context=signed_bitplane::Context::Magnitude;
    bool fineRingPrediction=false;
};

inline void putU64(std::vector<std::uint8_t>& out,std::uint64_t value) {
    for(int byte=0;byte<8;++byte)out.push_back(static_cast<std::uint8_t>(value>>(8*byte)));
}
inline void putF64(std::vector<std::uint8_t>& out,double value) {
    std::uint64_t bits;std::memcpy(&bits,&value,sizeof(bits));putU64(out,bits);
}
inline const std::vector<std::uint8_t>& requiredBytes(const PaperRoundSymbols& delta,const char* name) {
    const auto found=delta.side.find(name);
    if(found==delta.side.end()||found->second.size()!=1||!std::holds_alternative<std::vector<std::uint8_t>>(found->second.front()))
        throw std::runtime_error(std::string("MMSE missing/invalid typed stream ")+name);
    return std::get<std::vector<std::uint8_t>>(found->second.front());
}
inline std::uint32_t layerChecksum(const std::vector<std::uint8_t>& metadata,
                                  const PaperRoundSymbols& delta,const std::vector<std::uint8_t>& modes) {
    auto checksum=crc32(0L,metadata.data(),static_cast<uInt>(metadata.size()));
    auto fold=[&](std::int32_t value) {
        std::uint8_t bytes[4];for(int byte=0;byte<4;++byte)bytes[byte]=static_cast<std::uint8_t>(std::uint32_t(value)>>(8*byte));
        checksum=crc32(checksum,bytes,4);
    };
    for(auto code:delta.facetSymbols)fold(code);
    for(const auto& point:delta.removedVertexResiduals)for(int axis=0;axis<3;++axis)fold(point[axis]);
    checksum=crc32(checksum,modes.data(),static_cast<uInt>(modes.size()));
    return static_cast<std::uint32_t>(checksum);
}

// A logical-symbol mapper, with no Parquet classes or file I/O. The framework
// owns all physical tables, role encodings, row groups and prefix selection.
inline PMSFSymbolRecord encodeLayer(const conquest_blob::Stream& stream,
                                    const std::vector<std::uint8_t>& modes,
                                    const LayerMetadata& info) {
    if(info.encoderRound>=stream.layers.size()||info.rounds!=stream.layers.size())
        throw std::runtime_error("MMSE typed encoder layer range mismatch");
    const auto& layer=stream.layers[info.encoderRound];
    auto row=makePMSFSymbolRecord(0,0,static_cast<std::int32_t>(info.encoderRound));
    row.delta.facetSymbols.assign(layer.symbols.begin(),layer.symbols.end());
    for(const auto& point:layer.deltas)row.delta.removedVertexResiduals.emplace_back(point[0],point[1],point[2]);
    auto arithmetic=encodeModes(modes);
    const auto version = info.fineRingPrediction ? 2u : 1u;
    std::vector<std::uint8_t> metadata={'M','R','L',static_cast<std::uint8_t>('0'+version)};putU32(metadata,version);
    putU32(metadata,info.encoderRound);putU32(metadata,info.rounds);
    putU32(metadata,info.threshold);putU64(metadata,info.seed);
    putU32(metadata,static_cast<std::uint32_t>(info.context));
    putF64(metadata,stream.step);for(auto value:stream.bboxMin)putF64(metadata,value);
    for(const auto& point:layer.seedPos)for(auto value:point)putF64(metadata,value);
    putU32(metadata,static_cast<std::uint32_t>(layer.symbols.size()));
    putU32(metadata,static_cast<std::uint32_t>(layer.deltas.size()));
    putU32(metadata,static_cast<std::uint32_t>(modes.size()));
    putU32(metadata,layerChecksum(metadata,row.delta,arithmetic));
    if(metadata.size()!=kLayerMetadataBytes)throw std::runtime_error("MMSE typed metadata internal size mismatch");
    row.delta.side[kLayerMetadata].emplace_back(std::move(metadata));
    row.delta.side[kModeArithmetic].emplace_back(std::move(arithmetic));
    return row;
}

struct DecodedLayers {
    conquest_blob::Stream stream;
    std::vector<std::vector<std::uint8_t>> modes;
    std::uint32_t threshold=32;
    std::uint64_t seed=1;
    bool fineRingPrediction=false;
};

inline DecodedLayers decodeLayers(const PMSFSymbolTable& rows) {
    DecodedLayers out;out.stream.signedBitplanes=true;
    std::uint32_t originalRounds=0;
    for(std::size_t index=0;index<rows.size();++index) {
        const auto& row=rows[index];const auto& delta=row.delta;
        if(row.objectId!=0||row.partitionId!=0||row.lodId!=static_cast<std::int32_t>(index))
            throw std::runtime_error("MMSE typed rows must be a contiguous decode prefix of partition zero");
        if(!delta.triangulationEdgeSymbols.empty()||!delta.flipEdgeSymbols.empty()||!delta.vertexSymbols.empty()||
           !delta.removedVertexCoords.empty()||!delta.shiftedVertexCoords.empty()||!delta.shiftedVertexResiduals.empty())
            throw std::runtime_error("MMSE unexpected values in unused typed roles");
        if(delta.side.size()!=2)throw std::runtime_error("MMSE unexpected/missing typed extension streams");
        const auto& metadata=requiredBytes(delta,kLayerMetadata);
        const auto& arithmetic=requiredBytes(delta,kModeArithmetic);
        if(metadata.size()!=kLayerMetadataBytes||std::memcmp(metadata.data(),"MRL",3))
            throw std::runtime_error("MMSE bad typed metadata magic/size");
        conquest_blob::detail::Reader reader{reinterpret_cast<const char*>(metadata.data()+4),
            reinterpret_cast<const char*>(metadata.data()+metadata.size()),false};
        const auto version=reader.u32();
        const auto encoderRound=reader.u32(),rounds=reader.u32(),threshold=reader.u32();
        const auto seedLow=reader.u32(),seedHigh=reader.u32();
        const auto seed=std::uint64_t(seedLow)|(std::uint64_t(seedHigh)<<32);
        const auto context=reader.u32();
        const auto step=reader.f64();double lower[3];for(auto& value:lower)value=reader.f64();
        conquest_blob::LayerRecord layer;
        for(auto& point:layer.seedPos)for(auto& value:point) {value=reader.f64();if(!std::isfinite(value))throw std::runtime_error("MMSE non-finite typed seed");}
        const auto symbols=reader.u32(),gates=reader.u32(),count=reader.u32(),checksum=reader.u32();
        if(reader.bad||reader.p!=reader.end||version<1||version>2||metadata[3]!=static_cast<std::uint8_t>('0'+version)||!rounds||rounds>65536||encoderRound>=rounds||index!=rounds-1-encoderRound||
           threshold<1||threshold>4096||context>static_cast<std::uint32_t>(signed_bitplane::Context::Neighbors)||
           !std::isfinite(step)||step<=0||!std::all_of(std::begin(lower),std::end(lower),[](double value){return std::isfinite(value);}))
            throw std::runtime_error("MMSE invalid typed layer metadata values/order");
        if(symbols!=delta.facetSymbols.size()||gates!=delta.removedVertexResiduals.size()||
           symbols>signed_bitplane::kMaxCoefficients||gates>signed_bitplane::kMaxCoefficients||count>gates||(gates>0&&count==0))
            throw std::runtime_error("MMSE typed layer symbol/residual/mode count mismatch");
        if(layerChecksum(std::vector<std::uint8_t>(metadata.begin(),metadata.end()-4),delta,arithmetic)!=checksum)
            throw std::runtime_error("MMSE typed layer checksum mismatch");
        if(index==0) {
            originalRounds=rounds;out.threshold=threshold;out.seed=seed;out.fineRingPrediction=version==2;
            out.stream.step=step;std::copy(std::begin(lower),std::end(lower),out.stream.bboxMin);
            out.stream.bitplaneContext=static_cast<signed_bitplane::Context>(context);
        } else if(rounds!=originalRounds||out.threshold!=threshold||out.seed!=seed||out.fineRingPrediction!=(version==2)||
                  out.stream.step!=step||!std::equal(std::begin(lower),std::end(lower),out.stream.bboxMin)||
                  out.stream.bitplaneContext!=static_cast<signed_bitplane::Context>(context))
            throw std::runtime_error("MMSE inconsistent typed global metadata across layers");
        for(auto symbol:delta.facetSymbols) {
            if(symbol<0||symbol>4)throw std::runtime_error("MMSE typed F symbol outside conquest alphabet");
            layer.symbols.push_back(symbol);
        }
        for(const auto& point:delta.removedVertexResiduals)layer.deltas.push_back({point[0],point[1],point[2]});
        out.modes.push_back(decodeModes(reinterpret_cast<const char*>(arithmetic.data()),arithmetic.size(),count));
        out.stream.layers.push_back(std::move(layer));
    }
    return out;
}
} // namespace pmsf::mmse_symbols
