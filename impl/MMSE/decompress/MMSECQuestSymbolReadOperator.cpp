// SPDX-License-Identifier: Apache-2.0
// MMT1 retains the frozen 2-bit mode trailer. MMT2 uses arithmetic mode symbols
// and a CQST v3 signed-MSB-first residual stream; both are self-described.
#include "MMSE/decompress/MMSECQuestSymbolReadOperator.h"
#include "MMSE/MMSELayerSymbols.h"

#include <cstring>
#include <stdexcept>
#include <zlib.h>

namespace pmsf {
namespace {
std::uint32_t readU32(const char* p) {
    std::uint32_t value=0;
    for(int k=0;k<4;++k)value|=std::uint32_t(static_cast<unsigned char>(p[k]))<<(8*k);
    return value;
}

}

void MMSECQuestSymbolReadOperator::setPMSFSymbolContext(const PMSFSymbolTable* symbols,
                                                       const PMSFSymbolPayloadTable* payloads) {
    SymbolReadOperator::setPMSFSymbolContext(symbols,payloads);
    awaitingTypedMetadata_=symbols && !symbols->empty() && symbols->front().delta.side.count(mmse_symbols::kLayerMetadata)==0;
    if (!symbols || symbols->empty() || awaitingTypedMetadata_) return;
    auto decoded=mmse_symbols::decodeLayers(*symbols);
    layerModes_.clear();
    recursiveTopology_=true;fineRingPrediction_=decoded.fineRingPrediction;topologyThreshold_=static_cast<int>(decoded.threshold);topologySeed_=decoded.seed;
    for (auto& modes:decoded.modes)layerModes_.push_back({std::move(modes),topologyThreshold_});
    installDecodedStream(std::move(decoded.stream));
}

void MMSECQuestSymbolReadOperator::collectFromDecoded(MCGAL::Halfedge* seed,
    std::vector<MCGAL::Vertex*>& vertices,std::vector<MCGAL::Halfedge*>& halfedges,std::vector<MCGAL::Facet*>& facets) {
    if (awaitingTypedMetadata_)
        throw std::runtime_error("MMSE typed relations lack required layer metadata");
    ConquestSymbolReadOperator::collectFromDecoded(seed,vertices,halfedges,facets);
}

void MMSECQuestSymbolReadOperator::importFromRangeCodedBuffer(char* buffer,std::size_t& dataOffset,unsigned nbQuantBits) {
    layerModes_.clear();
    awaitingTypedMetadata_ = false;
    recursiveTopology_ = false;
    fineRingPrediction_ = false;
    const std::size_t total=bufferSize();
    if(!buffer||dataOffset>total||total-dataOffset<4)throw std::runtime_error("MMSE truncated mode trailer length");
    const std::uint32_t trailerLen=readU32(buffer+dataOffset);
    const std::size_t trailerOff=dataOffset+4;
    if(trailerLen<12||trailerLen>(1u<<20)||trailerLen>total-trailerOff)throw std::runtime_error("MMSE invalid mode trailer length");
    const char* trailer=buffer+trailerOff;
    const std::uint32_t version=readU32(trailer+4);
    if(version<1 || version>4 || std::memcmp(trailer,"MMT",3) || trailer[3]!=static_cast<char>('0'+version)) {
        throw std::runtime_error("MMSE unsupported mode trailer magic/version");
    }
    const std::uint32_t layers=readU32(trailer+8);
    std::size_t off=12;
    recursiveTopology_=version>=3;
    fineRingPrediction_=version==4;
    if(recursiveTopology_) {
        if(trailerLen<24)throw std::runtime_error("MMSE recursive topology header truncated");
        topologyThreshold_=static_cast<int>(readU32(trailer+12));
        topologySeed_=std::uint64_t(readU32(trailer+16))|(std::uint64_t(readU32(trailer+20))<<32);
        if(topologyThreshold_<1 || topologyThreshold_>4096)throw std::runtime_error("MMSE topology threshold out of range");
        off=24;
    }
    if(layers>(trailerLen-off)/12)throw std::runtime_error("MMSE impossible mode layer count");
    layerModes_.resize(layers);
    std::vector<std::uint32_t> gateCounts;
    for(std::size_t layer=0;layer<layers;++layer) {
        if(off+12>trailerLen)throw std::runtime_error("MMSE mode header truncated");
        const auto gateCount=readU32(trailer+off);
        const auto cluster=readU32(trailer+off+4);
        const auto byteCount=readU32(trailer+off+8);
        if(gateCount>signed_bitplane::kMaxCoefficients||byteCount>trailerLen-off-12)throw std::runtime_error("MMSE invalid mode dimension/length");
        std::size_t modeCount;
        if(recursiveTopology_) {
            modeCount=cluster;
            if(modeCount>gateCount || (gateCount>0 && modeCount==0))throw std::runtime_error("MMSE invalid recursive mode count");
            layerModes_[layer].clusterSize=topologyThreshold_;
        } else {
            if(!cluster||cluster>4096)throw std::runtime_error("MMSE invalid legacy cluster threshold");
            modeCount=(std::size_t(gateCount)+cluster-1)/cluster;
            layerModes_[layer].clusterSize=static_cast<int>(cluster);
        }
        gateCounts.push_back(gateCount);
        auto& modes=layerModes_[layer].modes;
        if(version>=2) modes=mmse_symbols::decodeModes(trailer+off+12,byteCount,modeCount);
        else {
            if(byteCount!=(modeCount+3)/4)throw std::runtime_error("MMSE packed mode byte count mismatch");
            modes.resize(modeCount);
            for(std::size_t i=0;i<modeCount;++i) {
                const auto packed=static_cast<std::uint8_t>(trailer[off+12+(i>>2)]);
                modes[i]=(packed>>(2*(i&3)))&3;
            }
        }
        off+=12+byteCount;
    }
    if(off!=trailerLen)throw std::runtime_error("MMSE mode trailer has trailing bytes");
    std::size_t cqstOff=trailerOff+trailerLen;
    ConquestSymbolReadOperator::importFromRangeCodedBuffer(buffer,cqstOff,nbQuantBits);
    dataOffset=trailerOff+trailerLen;
    if(!decodedOk()||stream().layers.size()!=layerModes_.size())throw std::runtime_error("MMSE CQST/layer table mismatch");
    if(recursiveTopology_ && !stream().signedBitplanes)throw std::runtime_error("MMSE recursive topology requires the declared signed-bitplane residual protocol");
    for(std::size_t layer=0;layer<gateCounts.size();++layer)
        if(stream().layers[layer].deltas.size()!=gateCounts[layer])throw std::runtime_error("MMSE gate/mode table residual count mismatch");
}
} // namespace pmsf
