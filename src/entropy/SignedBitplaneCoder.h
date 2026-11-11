// SPDX-License-Identifier: Apache-2.0
#pragma once

// Signed-magnitude MSB-first coding. A coefficient's first 1 is followed by
// its sign, while subsequent magnitude bits use separate arithmetic contexts.
// Context profiles are explicit in the stream; no decoder env is required.
#include "rangecoder/rangecod.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

namespace pmsf::signed_bitplane {
inline constexpr std::size_t kMaxCoefficients = 64 * 1024 * 1024;

enum class Context : std::uint8_t {
    Plane = 0,          // per-plane significance, one refinement + sign
    Magnitude = 1,      // per-plane significance, magnitude-age contexts
    Neighbors = 2      // neighbor significance + magnitude-prefix contexts
};
namespace detail {
inline void put32(std::vector<std::uint8_t>& out,std::uint32_t value) {
    for(int k=0;k<4;++k) out.push_back(static_cast<std::uint8_t>(value>>(8*k)));
}
inline std::uint32_t get32(const std::uint8_t* p) {
    return std::uint32_t(p[0])|(std::uint32_t(p[1])<<8)|(std::uint32_t(p[2])<<16)|(std::uint32_t(p[3])<<24);
}
struct Model {
    std::uint32_t zero=1,one=1;
    void update(bool bit) {
        ++(bit?one:zero);
        if(zero+one>=32768) { zero=(zero+1)/2;one=(one+1)/2; }
    }
};
inline int significanceContext(Context context,int plane,std::size_t index,const std::vector<int>& highest) {
    if(context!=Context::Neighbors) return plane;
    const int left=index>0&&highest[index-1]>=0;
    const int right=index+1<highest.size()&&highest[index+1]>=0;
    return plane*4+left+2*right;
}
inline int magnitudeContext(Context context,int plane,int first,std::uint32_t prefix) {
    if(context==Context::Plane) return 128;
    const int age=std::min(3,first-plane-1);
    return 128+age*4+(context==Context::Neighbors?int(prefix&3u):0);
}
inline std::uint32_t checksum(const std::vector<std::int32_t>& values) {
    // IEEE CRC-32 over little-endian int32 coefficients. Keep this primitive
    // header dependency-free so the portable conquest tools need no zlib link.
    static const auto table=[] {
        std::array<std::uint32_t,256> values{};
        for (std::uint32_t i=0;i<256;++i) {
            auto value=i;
            for(int bit=0;bit<8;++bit)value=(value>>1)^((value&1u)?0xedb88320u:0u);
            values[i]=value;
        }
        return values;
    }();
    std::uint32_t sum=0xffffffffu;
    for(auto value:values) {
        const auto u=static_cast<std::uint32_t>(value);
        for(int k=0;k<4;++k)sum=table[(sum^(u>>(8*k)))&0xffu]^(sum>>8);
    }
    return sum^0xffffffffu;
}
} // namespace detail

inline std::vector<std::uint8_t> encode(const std::vector<std::int32_t>& values,Context context=Context::Magnitude) {
    if(values.size()>kMaxCoefficients||static_cast<unsigned>(context)>2) throw std::runtime_error("bitplane: invalid size/context");
    std::vector<std::uint32_t> magnitudes; magnitudes.reserve(values.size());
    std::uint32_t maximum=0;
    for(auto value:values) {
        const auto magnitude=static_cast<std::uint32_t>(value<0?-std::int64_t(value):std::int64_t(value));
        maximum=std::max(maximum,magnitude);magnitudes.push_back(magnitude);
    }
    int bits=0;for(auto n=maximum;n;n>>=1)++bits;
    std::vector<std::uint8_t> payload;
    if(bits) {
        const std::size_t bins=values.size()*static_cast<std::size_t>(bits+1);
        if(bins>(std::numeric_limits<std::size_t>::max()-4096)/2) throw std::runtime_error("bitplane: output capacity overflow");
        std::vector<char> buffer(bins*2+4096);
        std::size_t offset=0;rangecoder coder{};coder.p_data=buffer.data();coder.p_dataOffset=&offset;
        start_encoding(&coder,0,0);
        std::array<detail::Model,145> models;
        std::vector<int> highest(values.size(),-1);
        auto bit=[&](int modelIndex,bool symbol) {
            auto& model=models[modelIndex];
            encode_freq(&coder,symbol?model.one:model.zero,symbol?model.zero:0,model.zero+model.one);
            model.update(symbol);
        };
        for(int plane=bits-1;plane>=0;--plane) {
            for(std::size_t i=0;i<values.size();++i) {
                const bool symbol=(magnitudes[i]>>plane)&1u;
                if(highest[i]<0) {
                    bit(detail::significanceContext(context,plane,i,highest),symbol);
                    if(symbol) { bit(144,values[i]<0);highest[i]=plane; }
                } else {
                    const std::uint32_t prefix=plane==31?0:magnitudes[i]>>(plane+1);
                    bit(detail::magnitudeContext(context,plane,highest[i],prefix),symbol);
                }
            }
        }
        done_encoding(&coder);
        payload.assign(buffer.begin(),buffer.begin()+offset);
    }
    std::vector<std::uint8_t> out={'B','P','S','1'};
    detail::put32(out,static_cast<std::uint32_t>(values.size()));
    out.push_back(static_cast<std::uint8_t>(bits));out.push_back(static_cast<std::uint8_t>(context));out.push_back(0);out.push_back(0);
    detail::put32(out,static_cast<std::uint32_t>(payload.size()));
    detail::put32(out,detail::checksum(values));
    out.insert(out.end(),payload.begin(),payload.end());
    return out;
}

inline std::vector<std::int32_t> decode(const std::uint8_t* input,std::size_t size,
                                      std::size_t expectedCount) {
    if(!input||size<20||std::memcmp(input,"BPS1",4)||detail::get32(input+4)!=expectedCount||
        input[8]>32||input[9]>2||input[10]||input[11]||std::size_t(detail::get32(input+12))!=size-20) throw std::runtime_error("bitplane: invalid header/length/count");
    if(expectedCount>kMaxCoefficients) throw std::runtime_error("bitplane: coefficient count exceeds supported bound");
    const int bits=input[8];const auto context=static_cast<Context>(input[9]);
    std::vector<std::int32_t> values(expectedCount,0);
    if(!bits) {
        if(size!=20) throw std::runtime_error("bitplane: zero-plane stream has payload");
    } else {
        if(size<23||expectedCount==0) throw std::runtime_error("bitplane: missing arithmetic payload");
        std::vector<char> buffer(input+20,input+size);
        std::size_t offset=0;rangecoder coder{};coder.p_data=buffer.data();coder.p_dataOffset=&offset;
        if(start_decoding(&coder)!=0) throw std::runtime_error("bitplane: arithmetic start failed");
        coder.p_dataEnd=buffer.data()+buffer.size();
        std::array<detail::Model,145> models;
        std::vector<int> highest(expectedCount,-1);
        std::vector<std::uint32_t> magnitudes(expectedCount,0);
        std::vector<bool> negative(expectedCount,false);
        auto bit=[&](int modelIndex) {
            auto& model=models[modelIndex];
            const auto total=model.zero+model.one;
            const auto cumulative=decode_culfreq(&coder,total);
            if(coder.inputError||cumulative>=total) throw std::runtime_error("bitplane: truncated/invalid arithmetic code");
            const bool symbol=cumulative>=model.zero;
            decode_update(&coder,symbol?model.one:model.zero,symbol?model.zero:0,total);
            model.update(symbol);
            return symbol;
        };
        for(int plane=bits-1;plane>=0;--plane) {
            for(std::size_t i=0;i<expectedCount;++i) {
                bool symbol;
                if(highest[i]<0) {
                    symbol=bit(detail::significanceContext(context,plane,i,highest));
                    if(symbol) { negative[i]=bit(144);highest[i]=plane; }
                } else {
                    const auto prefix=plane==31?0:magnitudes[i]>>(plane+1);
                    symbol=bit(detail::magnitudeContext(context,plane,highest[i],prefix));
                }
                if(symbol)magnitudes[i]|=std::uint32_t(1)<<plane;
            }
        }
        if(coder.inputError) throw std::runtime_error("bitplane: arithmetic buffer overrun");
        for(std::size_t i=0;i<expectedCount;++i) {
            const std::int64_t value=negative[i]?-std::int64_t(magnitudes[i]):std::int64_t(magnitudes[i]);
            if(value<std::numeric_limits<std::int32_t>::min()||value>std::numeric_limits<std::int32_t>::max()) throw std::runtime_error("bitplane: signed coefficient overflow");
            values[i]=static_cast<std::int32_t>(value);
        }
    }
    if(detail::checksum(values)!=detail::get32(input+16)) throw std::runtime_error("bitplane: coefficient checksum mismatch");
    return values;
}
} // namespace pmsf::signed_bitplane
