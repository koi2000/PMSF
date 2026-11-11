// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "entropy/RangeCoderEntropyCoder.h"
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include <zlib.h>

namespace pmsf::mmse_symbols {

inline void putU32(std::vector<std::uint8_t>& out, std::uint32_t value) {
    for (int byte=0; byte<4; ++byte) out.push_back(static_cast<std::uint8_t>(value>>(8*byte)));
}
inline std::uint32_t getU32(const char* input) {
    std::uint32_t value=0;
    for (int byte=0; byte<4; ++byte) value|=std::uint32_t(static_cast<unsigned char>(input[byte]))<<(8*byte);
    return value;
}

// Same MMT2/MMT3 arithmetic payload, including the checksum of decoded modes.
inline std::vector<std::uint8_t> encodeModes(const std::vector<std::uint8_t>& modes) {
    for (auto mode:modes) if (mode>=4) throw std::runtime_error("MMSE invalid prediction mode");
    std::vector<std::uint32_t> symbols(modes.begin(),modes.end());
    RangeCoderEntropyCoder coder(2);
    std::vector<std::uint8_t> arithmetic; coder.encode(symbols,arithmetic);
    std::vector<std::uint8_t> out;
    putU32(out,static_cast<std::uint32_t>(crc32(0L,modes.data(),static_cast<uInt>(modes.size()))));
    out.insert(out.end(),arithmetic.begin(),arithmetic.end());
    return out;
}

inline std::vector<std::uint8_t> decodeModes(const char* input,std::size_t bytes,std::size_t count) {
    if(bytes<7)throw std::runtime_error("MMSE mode arithmetic payload is truncated");
    const std::uint32_t checksum=getU32(input);
    std::vector<char> payload(input+4,input+bytes);
    std::size_t offset=0;rangecoder coder{};coder.p_data=payload.data();coder.p_dataOffset=&offset;
    if(start_decoding(&coder)!=0)throw std::runtime_error("MMSE arithmetic mode start failed");
    coder.p_dataEnd=payload.data()+payload.size();
    qsmodel model{};initqsmodel(&model,4,10,1<<9,nullptr,0);
    std::vector<std::uint8_t> modes; modes.reserve(count);
    for(std::size_t i=0;i<count;++i) {
        const auto cumulative=decode_culshift(&coder,10);
        if(coder.inputError||cumulative>=1024) { deleteqsmodel(&model);throw std::runtime_error("MMSE mode arithmetic input ended early"); }
        const int symbol=qsgetsym(&model,static_cast<int>(cumulative));
        if(symbol<0||symbol>=4) { deleteqsmodel(&model);throw std::runtime_error("MMSE invalid prediction mode"); }
        int frequency=0,lower=0;qsgetfreq(&model,symbol,&frequency,&lower);
        decode_update(&coder,frequency,lower,1024);qsupdate(&model,symbol);
        modes.push_back(static_cast<std::uint8_t>(symbol));
    }
    deleteqsmodel(&model);
    if(coder.inputError||static_cast<std::uint32_t>(crc32(0L,modes.data(),static_cast<uInt>(modes.size())))!=checksum)
        throw std::runtime_error("MMSE mode stream checksum mismatch");
    return modes;
}
} // namespace pmsf::mmse_symbols
