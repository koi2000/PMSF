// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "PCLTTM/PCLTTMFrenetGeometry.h"
#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"
#include <bit>
#include <cstddef>
#include <cstring>
#include <vector>
#include <zlib.h>

namespace pmsf::pclttm_frenet {

// Two-pass local-coordinate stream: first find the tangent and normal
// alphabets (offset and span), then write two order-zero arithmetic streams.
// PFR1 specifies this implementation's framing, not an author wire format.
inline constexpr char kGeometryStream[]="paper_role/CfQ/pclttm_frenet_pfr1";
inline constexpr char kValenceEscape[]="pclttm_tag/valence_other";

struct Layer {
    unsigned denominator=1;
    std::vector<Cell> coefficients;
};
namespace layer_detail {
inline constexpr std::uint32_t kMaxSpan=1u<<20;
inline constexpr std::size_t kMaxPoints=1u<<24;
inline constexpr std::size_t kHeaderSize=36;

inline void put(std::vector<std::uint8_t>& out,std::uint32_t value) {
    for(int byte=0;byte<4;++byte)out.push_back(static_cast<std::uint8_t>(value>>(8*byte)));
}
inline std::uint32_t get(const std::uint8_t* data) {
    std::uint32_t out=0;for(int byte=0;byte<4;++byte)out|=std::uint32_t(data[byte])<<(8*byte);return out;
}
struct Range {
    std::int32_t offset=0;
    std::uint32_t span=1;
};
inline Range rangeOf(const std::vector<std::int32_t>& symbols) {
    if(symbols.empty())return {};
    const auto [low,high]=std::minmax_element(symbols.begin(),symbols.end());
    const auto span=std::int64_t(*high)-*low+1;
    if(span>kMaxSpan)throw std::runtime_error("PCLTTM Frenet alphabet exceeds bounded range");
    return {*low,static_cast<std::uint32_t>(span)};
}
struct Model {
    qsmodel value{};
    explicit Model(Range range,bool encode,int bits) {
        initqsmodel(&value,static_cast<int>(range.span),bits,1<<std::min(bits-1,12),nullptr,encode);
    }
    ~Model(){deleteqsmodel(&value);}
    Model(const Model&)=delete;
    Model& operator=(const Model&)=delete;
};
inline std::vector<std::uint8_t> encodeRange(const std::vector<std::int32_t>& symbols,Range range) {
    if(range.span==1)return {};
    const int bits=std::max(10,1+static_cast<int>(std::bit_width(range.span-1)));
    Model model(range,true,bits);
    std::vector<char> staging(symbols.size()*3+32);
    std::size_t bytes=0;
    rangecoder coder{};coder.p_data=staging.data();coder.p_dataOffset=&bytes;
    start_encoding(&coder,0,0);
    for(auto value:symbols) {
        const int code=static_cast<int>(std::int64_t(value)-range.offset);
        int frequency=0,cumulative=0;
        qsgetfreq(&model.value,code,&frequency,&cumulative);
        encode_shift(&coder,frequency,cumulative,bits);qsupdate(&model.value,code);
    }
    done_encoding(&coder);
    return {staging.begin(),staging.begin()+bytes};
}
inline std::vector<std::int32_t> decodeRange(std::span<const std::uint8_t> stream,std::size_t count,Range range) {
    if(!range.span||range.span>kMaxSpan||std::int64_t(range.offset)+range.span-1>std::numeric_limits<std::int32_t>::max())
        throw std::runtime_error("invalid Frenet offset/span");
    if(range.span==1) {
        if(!stream.empty())throw std::runtime_error("non-empty constant Frenet range stream");
        return std::vector<std::int32_t>(count,range.offset);
    }
    if(!count||stream.size()<3)throw std::runtime_error("truncated Frenet range stream");
    const int bits=std::max(10,1+static_cast<int>(std::bit_width(range.span-1)));
    Model model(range,false,bits);
    std::vector<char> staging(stream.begin(),stream.end());
    std::size_t cursor=0;rangecoder coder{};coder.p_data=staging.data();coder.p_dataOffset=&cursor;
    if(start_decoding(&coder)!=0)throw std::runtime_error("invalid Frenet arithmetic start");
    coder.p_dataEnd=staging.data()+staging.size();
    std::vector<std::int32_t> values;values.reserve(count);
    for(std::size_t i=0;i<count;++i) {
        const auto fraction=decode_culshift(&coder,bits);
        if(coder.inputError||fraction>=(1u<<bits))throw std::runtime_error("truncated Frenet arithmetic payload");
        const int code=qsgetsym(&model.value,static_cast<int>(fraction));
        if(code<0||static_cast<std::uint32_t>(code)>=range.span)throw std::runtime_error("invalid Frenet arithmetic symbol");
        int frequency=0,cumulative=0;
        qsgetfreq(&model.value,code,&frequency,&cumulative);
        decode_update(&coder,frequency,cumulative,1u<<bits);qsupdate(&model.value,code);
        values.push_back(static_cast<std::int32_t>(std::int64_t(range.offset)+code));
    }
    if(coder.inputError)throw std::runtime_error("truncated Frenet arithmetic payload");
    return values;
}
inline std::uint32_t checksum(std::span<const std::uint8_t> meta,std::span<const std::uint8_t> data) {
    const auto head=crc32(0,meta.data(),static_cast<uInt>(meta.size()));
    return static_cast<std::uint32_t>(crc32(head,data.data(),static_cast<uInt>(data.size())));
}
}

inline Layer chooseLayer(std::span<const PatchCode> patches) {
    Layer layer;
    layer.denominator=std::all_of(patches.begin(),patches.end(),[](const auto& p){return p.wholeExact;})?1:2;
    layer.coefficients.reserve(patches.size());
    for(const auto& p:patches)layer.coefficients.push_back(layer.denominator==1?p.wholeCell:p.halfCell);
    return layer;
}

inline std::vector<std::uint8_t> encodeLayer(const Layer& layer) {
    using namespace layer_detail;
    if((layer.denominator!=1&&layer.denominator!=2)||layer.coefficients.size()>kMaxPoints)
        throw std::runtime_error("invalid Frenet layer dimensions");
    std::vector<std::int32_t> tangent,normal;
    tangent.reserve(layer.coefficients.size()*2);normal.reserve(layer.coefficients.size());
    for(const auto& code:layer.coefficients) {tangent.push_back(code[0]);tangent.push_back(code[1]);normal.push_back(code[2]);}
    const auto tr=rangeOf(tangent),nr=rangeOf(normal);
    const auto ts=encodeRange(tangent,tr),ns=encodeRange(normal,nr);
    std::vector<std::uint8_t> out={'P','F','R','1'};
    put(out,static_cast<std::uint32_t>(layer.coefficients.size()));put(out,layer.denominator);
    put(out,static_cast<std::uint32_t>(tr.offset));put(out,tr.span);
    put(out,static_cast<std::uint32_t>(nr.offset));put(out,nr.span);
    put(out,static_cast<std::uint32_t>(ts.size()));
    put(out,0);
    out.insert(out.end(),ts.begin(),ts.end());out.insert(out.end(),ns.begin(),ns.end());
    const auto crc=checksum(std::span(out).first(kHeaderSize-4),std::span(out).subspan(kHeaderSize));
    for(int byte=0;byte<4;++byte)out[kHeaderSize-4+byte]=static_cast<std::uint8_t>(crc>>(8*byte));
    return out;
}

inline Layer decodeLayer(std::span<const std::uint8_t> stream,std::size_t expectedPoints) {
    using namespace layer_detail;
    if(stream.size()<kHeaderSize||stream.size()>std::numeric_limits<uInt>::max()||std::memcmp(stream.data(),"PFR1",4)!=0)
        throw std::runtime_error("invalid PCLTTM Frenet layer header");
    const auto* data=stream.data();
    const auto count=get(data+4),denominator=get(data+8),tangentBytes=get(data+28);
    if(count!=expectedPoints||count>kMaxPoints||(denominator!=1&&denominator!=2)||tangentBytes>stream.size()-kHeaderSize)
        throw std::runtime_error("invalid PCLTTM Frenet layer count/step/length");
    if(get(data+32)!=checksum(stream.first(kHeaderSize-4),stream.subspan(kHeaderSize)))
        throw std::runtime_error("PCLTTM Frenet layer checksum mismatch");
    const Range tr{static_cast<std::int32_t>(get(data+12)),get(data+16)},nr{static_cast<std::int32_t>(get(data+20)),get(data+24)};
    const auto tangent=decodeRange(stream.subspan(kHeaderSize,tangentBytes),std::size_t(count)*2,tr);
    const auto normal=decodeRange(stream.subspan(kHeaderSize+tangentBytes),count,nr);
    Layer layer{denominator,{}};layer.coefficients.reserve(count);
    for(std::size_t i=0;i<count;++i)layer.coefficients.push_back({tangent[2*i],tangent[2*i+1],normal[i]});
    return layer;
}
}
