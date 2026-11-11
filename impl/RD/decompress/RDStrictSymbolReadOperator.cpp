// SPDX-License-Identifier: Apache-2.0

#include "RD/decompress/RDStrictSymbolReadOperator.h"

#include "compress/ConnectivityBlobContainer.h"
#include "RD/RDStrictLayout.h"
#include "common/patch/RDPatchDecoderConquest.h"
#include "Mesh.h"
#include "predict/KLTTransform.h"

#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"

#include <algorithm>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>

namespace pmsf {
namespace {

std::shared_ptr<MCGAL::Mesh> canonicalReloadMesh(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!mesh) return nullptr;
    char path[] = "/tmp/pmsf_rdstrict_decode_XXXXXX.off";
    const int fd = mkstemps(path, 4);
    if (fd < 0) {
        throw std::runtime_error("[RDStrict decoder] cannot create canonical temp mesh");
    }
    close(fd);
    mesh->dumpto_oldtype(path);
    auto copy = std::make_shared<MCGAL::Mesh>();
    if (!copy->loadOFF(path)) {
        std::remove(path);
        throw std::runtime_error("[RDStrict decoder] cannot reload canonical temp mesh");
    }
    std::remove(path);
    copy->bboxMin = mesh->bboxMin;
    copy->bboxMax = mesh->bboxMax;
    copy->f_quantStep = mesh->f_quantStep;
    copy->i_nbQuantBits = mesh->i_nbQuantBits;
    copy->enableQuantization = mesh->enableQuantization;
    copy->setMeshId(mesh->meshId());
    return copy;
}

std::int32_t readI32(std::span<const std::uint8_t> s,
                     std::size_t& off,
                     const char* what) {
    if (off + 4 > s.size()) {
        throw std::runtime_error(std::string("[RDStrict decoder] truncated ") + what);
    }
    std::int32_t v = 0;
    std::memcpy(&v, s.data() + off, 4);
    off += 4;
    return v;
}

std::uint32_t readU32(std::span<const std::uint8_t> s,
                      std::size_t& off,
                      const char* what) {
    if (off + 4 > s.size()) {
        throw std::runtime_error(std::string("[RDStrict decoder] truncated ") + what);
    }
    std::uint32_t v = 0;
    std::memcpy(&v, s.data() + off, 4);
    off += 4;
    return v;
}

std::uint16_t readU16(std::span<const std::uint8_t> s,
                      std::size_t& off,
                      const char* what) {
    if (off + 2 > s.size()) {
        throw std::runtime_error(std::string("[RDStrict decoder] truncated ") + what);
    }
    const std::uint16_t v =
        static_cast<std::uint16_t>(s[off])
        | static_cast<std::uint16_t>(static_cast<std::uint16_t>(s[off + 1]) << 8);
    off += 2;
    return v;
}

double readF64(std::span<const std::uint8_t> s,
               std::size_t& off,
               const char* what) {
    if (off + 8 > s.size()) {
        throw std::runtime_error(std::string("[RDStrict decoder] truncated ") + what);
    }
    std::uint64_t u = 0;
    for (int i = 0; i < 8; ++i) {
        u |= static_cast<std::uint64_t>(s[off + static_cast<std::size_t>(i)]) << (8 * i);
    }
    off += 8;
    double v = 0.0;
    std::memcpy(&v, &u, sizeof(v));
    return v;
}

std::vector<RDPatchSymbol> decodeConnectivityStream(std::span<const std::uint8_t> sl) {
    if (sl.size() < 8) {
        throw std::runtime_error("[RDStrict decoder] connectivity stream too small");
    }
    std::size_t off = 0;
    const std::int32_t numSyms = readI32(sl, off, "connectivity symbol count");
    const std::int32_t numOther = readI32(sl, off, "connectivity V_OTHER count");
    if (numSyms < 0 || numOther < 0 || numOther > numSyms) {
        throw std::runtime_error("[RDStrict decoder] invalid connectivity counts");
    }

    std::vector<char> rcBytes(sl.begin() + static_cast<std::ptrdiff_t>(off), sl.end());
    std::size_t rcOff = 0;
    rangecoder rc;
    rc.p_data = rcBytes.data();
    rc.p_dataOffset = &rcOff;
    if (start_decoding(&rc) < 0) {
        throw std::runtime_error("[RDStrict decoder] connectivity RC start_decoding failed");
    }

    constexpr int kAlphabetSize = 6;
    constexpr int kLgTotf = 10;
    constexpr int kRescale = 1 << 9;
    constexpr int kContexts = 4;

    qsmodel models[kContexts];
    for (auto& model : models) {
        initqsmodel(&model, kAlphabetSize, kLgTotf, kRescale, nullptr, 0);
    }
    qsmodel escapeModel;
    initqsmodel(&escapeModel, 256, kLgTotf, kRescale, nullptr, 0);

    auto contextFor = [](int prev) {
        if (prev < 0) return 0;
        if (prev == static_cast<int>(RDPatchSymbolKind::NULL_PATCH)) return 1;
        if (prev == static_cast<int>(RDPatchSymbolKind::V_OTHER)) return 3;
        return 2;
    };

    std::vector<RDPatchSymbol> out;
    out.reserve(static_cast<std::size_t>(numSyms));
    int prev = -1;
    int otherSeen = 0;
    for (std::int32_t i = 0; i < numSyms; ++i) {
        qsmodel& model = models[contextFor(prev)];
        const int ltval = decode_culshift(&rc, kLgTotf);
        const int sym = qsgetsym(&model, ltval);
        if (sym < static_cast<int>(RDPatchSymbolKind::NULL_PATCH)
            || sym > static_cast<int>(RDPatchSymbolKind::V_OTHER)) {
            throw std::runtime_error("[RDStrict decoder] invalid connectivity symbol");
        }
        int syfreq = 0;
        int ltfreq = 0;
        qsgetfreq(&model, sym, &syfreq, &ltfreq);
        decode_update(&rc, syfreq, ltfreq, 1 << kLgTotf);
        qsupdate(&model, sym);

        unsigned char escape = 0;
        if (sym == static_cast<int>(RDPatchSymbolKind::V_OTHER)) {
            const int escLt = decode_culshift(&rc, kLgTotf);
            const int esc = qsgetsym(&escapeModel, escLt);
            qsgetfreq(&escapeModel, esc, &syfreq, &ltfreq);
            decode_update(&rc, syfreq, ltfreq, 1 << kLgTotf);
            qsupdate(&escapeModel, esc);
            escape = static_cast<unsigned char>(esc);
            ++otherSeen;
        }

        out.push_back(RDPatchSymbol{
            static_cast<RDPatchSymbolKind>(sym),
            escape,
            /*phase=*/0,
            /*bucket=*/-1,
            /*layer=*/0});
        prev = sym;
    }

    done_decoding(&rc);
    for (auto& model : models) deleteqsmodel(&model);
    deleteqsmodel(&escapeModel);

    if (otherSeen != numOther) {
        throw std::runtime_error("[RDStrict decoder] V_OTHER count mismatch");
    }
    return out;
}

void decodeGeometryStream(std::span<const std::uint8_t> sl,
                          std::int32_t& residualCount,
                          std::vector<int>& outX,
                          std::vector<int>& outY,
                          std::vector<int>& outZ,
                          const std::vector<RDPatchSymbol>& decodedSymbols,
                          std::vector<RDStrictBitplaneRef>& bitplaneRefs,
                          int& decodedBitplaneRefs,
                          int maxRefsOverride = -1) {
    std::size_t off = 0;
    const auto magic = readU32(sl, off, "geometry magic");
    if (magic != rdstrict::kGeometryMagic) {
        throw std::runtime_error("[RDStrict decoder] bad geometry stream magic");
    }
    const auto sv = readU32(sl, off, "geometry version");
    if (sv != pmsf::rdStrictConnectivityBlobSpec(1).schemeVersion) {
        throw std::runtime_error("[RDStrict decoder] unsupported geometry stream version");
    }
    residualCount = readI32(sl, off, "geometry residual count");
    if (residualCount < 0) {
        throw std::runtime_error("[RDStrict decoder] invalid geometry residual count");
    }
    if (off + 4 > sl.size()) {
        throw std::runtime_error("[RDStrict decoder] truncated geometry flags");
    }
    const std::uint8_t leakyFactors = sl[off++];
    (void)leakyFactors;
    const std::uint8_t flags = sl[off++];
    const std::uint8_t encoding = sl[off++];
    const std::uint8_t reserved = sl[off++];
    if (reserved != 0) {
        throw std::runtime_error("[RDStrict decoder] geometry reserved byte must be zero");
    }

    outX.clear();
    outY.clear();
    outZ.clear();
    bitplaneRefs.clear();
    decodedBitplaneRefs = 0;
    if ((flags & 1u) == 0) {
        return;
    }
    if (encoding != 1 && encoding != 2) {
        throw std::runtime_error("[RDStrict decoder] unsupported geometry residual encoding");
    }
    if (off + 4 > sl.size()) {
        throw std::runtime_error("[RDStrict decoder] truncated geometry bit-plane header");
    }
    const int bpBits = static_cast<int>(sl[off++]);
    const std::uint8_t r0 = sl[off++];
    const std::uint8_t r1 = sl[off++];
    const std::uint8_t r2 = sl[off++];
    if (r0 != 0 || r1 != 0 || r2 != 0 || bpBits <= 0 || bpBits > 31) {
        throw std::runtime_error("[RDStrict decoder] invalid geometry bit-plane header");
    }

    outX.assign(static_cast<std::size_t>(residualCount), 0);
    outY.assign(static_cast<std::size_t>(residualCount), 0);
    outZ.assign(static_cast<std::size_t>(residualCount), 0);
    if (residualCount == 0) return;

    if (encoding == 2) {
        struct Payload {
            std::uint16_t layer = 0;
            std::uint8_t component = 0;
            std::uint8_t bitplane = 0;
            std::int32_t bitCount = 0;
            std::span<const std::uint8_t> bytes;
        };

        const auto payloadCount = readI32(sl, off, "geometry bitplane payload count");
        if (payloadCount < 0 || payloadCount > 10'000'000) {
            throw std::runtime_error("[RDStrict decoder] invalid geometry payload count");
        }
        std::vector<Payload> payloads;
        payloads.reserve(static_cast<std::size_t>(payloadCount));
        for (std::int32_t i = 0; i < payloadCount; ++i) {
            Payload p;
            p.layer = readU16(sl, off, "geometry payload layer");
            if (off + 2 > sl.size()) {
                throw std::runtime_error("[RDStrict decoder] truncated geometry payload ids");
            }
            p.component = sl[off++];
            p.bitplane = sl[off++];
            p.bitCount = readI32(sl, off, "geometry payload bit count");
            const auto byteLen = readU32(sl, off, "geometry payload byte length");
            if (p.bitCount < 0 || off + byteLen > sl.size() ||
                p.component > 2 || p.bitplane >= bpBits) {
                throw std::runtime_error("[RDStrict decoder] invalid geometry payload metadata");
            }
            p.bytes = std::span<const std::uint8_t>(sl.data() + off, byteLen);
            off += byteLen;
            payloads.push_back(p);
        }

        std::vector<std::vector<std::size_t>> layerResidualIndices;
        int residualCursor = 0;
        for (const RDPatchSymbol& s : decodedSymbols) {
            if (s.kind == RDPatchSymbolKind::NULL_PATCH) continue;
            if (residualCursor >= residualCount) {
                throw std::runtime_error("[RDStrict decoder] residual layer overflow");
            }
            const auto layer = static_cast<std::size_t>(s.layer);
            if (layer >= layerResidualIndices.size()) {
                layerResidualIndices.resize(layer + 1);
            }
            layerResidualIndices[layer].push_back(static_cast<std::size_t>(residualCursor));
            ++residualCursor;
        }
        if (residualCursor != residualCount) {
            throw std::runtime_error("[RDStrict decoder] residual layer count mismatch");
        }

        std::vector<std::uint32_t> z[3];
        for (auto& comp : z) comp.assign(static_cast<std::size_t>(residualCount), 0);

        bitplaneRefs.reserve(payloads.size());
        for (const Payload& p : payloads) {
            bitplaneRefs.push_back(RDStrictBitplaneRef{
                /*round=*/0,
                p.layer,
                p.component,
                p.bitplane});
        }
        int maxRefs = static_cast<int>(payloads.size());
        if (maxRefsOverride >= 0) {
            maxRefs = maxRefsOverride;
        } else {
            const char* maxEnv = std::getenv("PMSF_RDSTRICT_MAX_BITPLANES");
            if (maxEnv && maxEnv[0] != '\0') {
                maxRefs = std::max(0, std::atoi(maxEnv));
            }
        }
        maxRefs = std::min(maxRefs, static_cast<int>(payloads.size()));

        auto decodePayloadBits = [&](const Payload& payload) {
            std::vector<std::uint8_t> bits(static_cast<std::size_t>(payload.bitCount), 0);
            if (payload.bitCount == 0) return bits;
            std::vector<char> rcBytes(payload.bytes.begin(), payload.bytes.end());
            std::size_t rcOff = 0;
            rangecoder rc;
            rc.p_data = rcBytes.data();
            rc.p_dataOffset = &rcOff;
            if (start_decoding(&rc) < 0) {
                throw std::runtime_error("[RDStrict decoder] bitplane RC start_decoding failed");
            }
            qsmodel model;
            initqsmodel(&model, 2, 10, 1 << 9, nullptr, 0);
            for (std::int32_t i = 0; i < payload.bitCount; ++i) {
                const int ltval = decode_culshift(&rc, 10);
                const int bit = qsgetsym(&model, ltval);
                int syfreq = 0;
                int ltfreq = 0;
                qsgetfreq(&model, bit, &syfreq, &ltfreq);
                decode_update(&rc, syfreq, ltfreq, 1 << 10);
                qsupdate(&model, bit);
                bits[static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(bit);
            }
            done_decoding(&rc);
            deleteqsmodel(&model);
            return bits;
        };

        auto applyPayload = [&](const Payload& payload) {
            const auto layer = static_cast<std::size_t>(payload.layer);
            if (layer >= layerResidualIndices.size()) {
                if (payload.bitCount == 0) return;
                throw std::runtime_error("[RDStrict decoder] payload layer has no residuals");
            }
            const auto& indices = layerResidualIndices[layer];
            if (static_cast<std::size_t>(payload.bitCount) != indices.size()) {
                throw std::runtime_error("[RDStrict decoder] bitplane payload count mismatch");
            }
            auto bits = decodePayloadBits(payload);
            const int bitIndex = bpBits - 1 - static_cast<int>(payload.bitplane);
            for (std::size_t i = 0; i < bits.size(); ++i) {
                if (bits[i]) {
                    z[payload.component][indices[i]] |= (1u << bitIndex);
                }
            }
        };

        for (int i = 0; i < maxRefs; ++i) {
            applyPayload(payloads[static_cast<std::size_t>(i)]);
            ++decodedBitplaneRefs;
        }

        auto unzigzag = [](std::uint32_t v) {
            return static_cast<int>((v >> 1) ^ (~(v & 1u) + 1u));
        };
        for (std::int32_t i = 0; i < residualCount; ++i) {
            const auto idx = static_cast<std::size_t>(i);
            outX[idx] = unzigzag(z[0][idx]);
            outY[idx] = unzigzag(z[1][idx]);
            outZ[idx] = unzigzag(z[2][idx]);
        }
        return;
    }

    std::vector<char> rcBytes(sl.begin() + static_cast<std::ptrdiff_t>(off), sl.end());
    std::size_t rcOff = 0;
    rangecoder rc;
    rc.p_data = rcBytes.data();
    rc.p_dataOffset = &rcOff;
    if (start_decoding(&rc) < 0) {
        throw std::runtime_error("[RDStrict decoder] geometry RC start_decoding failed");
    }

    std::vector<qsmodel> bpFirst1(static_cast<std::size_t>(3 * bpBits));
    std::vector<qsmodel> bpAfter1(static_cast<std::size_t>(3 * bpBits));
    auto modelIndex = [bpBits](int comp, int bit) {
        return static_cast<std::size_t>(comp * bpBits + bit);
    };
    for (int c = 0; c < 3; ++c) {
        for (int b = 0; b < bpBits; ++b) {
            initqsmodel(&bpFirst1[modelIndex(c, b)], 2, 10, 1 << 9, nullptr, 0);
            initqsmodel(&bpAfter1[modelIndex(c, b)], 2, 10, 1 << 9, nullptr, 0);
        }
    }

    auto decOne = [&](int comp) {
        std::uint32_t z = 0;
        bool seenOne = false;
        for (int b = bpBits - 1; b >= 0; --b) {
            qsmodel* model = seenOne ? &bpAfter1[modelIndex(comp, b)]
                                     : &bpFirst1[modelIndex(comp, b)];
            const int ltval = decode_culshift(&rc, 10);
            const int bit = qsgetsym(model, ltval);
            int syfreq = 0;
            int ltfreq = 0;
            qsgetfreq(model, bit, &syfreq, &ltfreq);
            decode_update(&rc, syfreq, ltfreq, 1 << 10);
            qsupdate(model, bit);
            if (bit) {
                seenOne = true;
                z |= (1u << b);
            }
        }
        return static_cast<int>((z >> 1) ^ (~(z & 1u) + 1u));
    };

    for (std::int32_t i = 0; i < residualCount; ++i) {
        outX[static_cast<std::size_t>(i)] = decOne(0);
        outY[static_cast<std::size_t>(i)] = decOne(1);
        outZ[static_cast<std::size_t>(i)] = decOne(2);
    }
    decodedBitplaneRefs = 3 * bpBits;

    done_decoding(&rc);
    for (int c = 0; c < 3; ++c) {
        for (int b = 0; b < bpBits; ++b) {
            deleteqsmodel(&bpFirst1[modelIndex(c, b)]);
            deleteqsmodel(&bpAfter1[modelIndex(c, b)]);
        }
    }
}

RDStrictSideInfo parseSideInfo(
    std::span<const std::uint8_t> sl,
    std::vector<RDStrictPartitionInfo>& partitions) {
    std::size_t off = 0;
    const auto magic = readU32(sl, off, "side-info magic");
    if (magic != rdstrict::kSideInfoMagic) {
        throw std::runtime_error("[RDStrict decoder] bad side-info stream magic");
    }
    const auto sv = readU32(sl, off, "side-info version");
    if (sv != pmsf::rdStrictConnectivityBlobSpec(1).schemeVersion) {
        throw std::runtime_error("[RDStrict decoder] unsupported side-info version");
    }

    RDStrictSideInfo info;
    info.numVerticesIn = readI32(sl, off, "numVerticesIn");
    info.numVerticesOut = readI32(sl, off, "numVerticesOut");
    info.numDecimatedTotal = readI32(sl, off, "numDecimatedTotal");
    info.numNullPatches = readI32(sl, off, "numNullPatches");
    info.numDecCycles = readI32(sl, off, "numDecCycles");
    for (int i = 0; i < 3; ++i) info.firstGateV0Q[i] = readI32(sl, off, "firstGateV0Q");
    for (int i = 0; i < 3; ++i) info.firstGateV1Q[i] = readI32(sl, off, "firstGateV1Q");

    const auto layerCount = readI32(sl, off, "layerCount");
    if (layerCount < 0 || layerCount > 1'000'000) {
        throw std::runtime_error("[RDStrict decoder] invalid layer count");
    }
    info.layerSymbolCounts.assign(static_cast<std::size_t>(layerCount), 0);
    info.layerVSymbolCounts.assign(static_cast<std::size_t>(layerCount), 0);
    info.layerBitplaneCounts.assign(static_cast<std::size_t>(layerCount), 0);
    info.layerLeakyFactors.assign(static_cast<std::size_t>(layerCount), 0);
    info.layerKltFlags.assign(static_cast<std::size_t>(layerCount), 0);
    partitions.clear();
    for (std::int32_t i = 0; i < layerCount; ++i) {
        const auto layerIndex = readI32(sl, off, "layer index");
        const auto symbolCount = readI32(sl, off, "layer symbol count");
        const auto vSymbolCount = readI32(sl, off, "layer V-symbol count");
        if (layerIndex < 0 || layerIndex >= layerCount || symbolCount < 0 || vSymbolCount < 0) {
            throw std::runtime_error("[RDStrict decoder] invalid layer side-info row");
        }
        info.layerSymbolCounts[static_cast<std::size_t>(layerIndex)] = symbolCount;
        info.layerVSymbolCounts[static_cast<std::size_t>(layerIndex)] = vSymbolCount;
    }
    for (std::int32_t i = 0; i < layerCount; ++i) {
        if (off + 4 > sl.size()) {
            throw std::runtime_error("[RDStrict decoder] truncated layer coding parameters");
        }
        info.layerBitplaneCounts[static_cast<std::size_t>(i)] = sl[off++];
        info.layerLeakyFactors[static_cast<std::size_t>(i)] = sl[off++];
        info.layerKltFlags[static_cast<std::size_t>(i)] = sl[off++];
        const auto reserved = sl[off++];
        if (reserved != 0) {
            throw std::runtime_error("[RDStrict decoder] non-zero reserved layer side-info");
        }
        const auto partitionCount = readI32(sl, off, "partition count");
        if (partitionCount < 0 || partitionCount > 1'000'000) {
            throw std::runtime_error("[RDStrict decoder] invalid partition count");
        }
        for (std::int32_t p = 0; p < partitionCount; ++p) {
            RDStrictPartitionInfo pi;
            pi.layer = static_cast<std::uint16_t>(i);
            pi.partition = readU16(sl, off, "partition id");
            pi.size = readI32(sl, off, "partition size");
            if (off + 2 > sl.size()) {
                throw std::runtime_error("[RDStrict decoder] truncated partition flags");
            }
            pi.useKltFlags = sl[off++];
            const auto basisLen = sl[off++];
            if (pi.size < 0 || off + basisLen > sl.size()) {
                throw std::runtime_error("[RDStrict decoder] invalid partition KLT metadata");
            }
            pi.kltBasis.assign(sl.begin() + static_cast<std::ptrdiff_t>(off),
                               sl.begin() + static_cast<std::ptrdiff_t>(off + basisLen));
            off += basisLen;
            partitions.push_back(std::move(pi));
        }
    }
    if (off + 4 <= sl.size()) {
        const auto v2magic = readU32(sl, off, "v2 side-info magic");
        if (v2magic == 0x36534452u) {  // "RDS6" face rank only
            const auto rankCount = readI32(sl, off, "r6 rank count");
            if (rankCount < 0 || rankCount > 10'000'000) throw std::runtime_error("bad r6 rank");
            info.faceRanks.resize(static_cast<std::size_t>(rankCount));
            for (std::int32_t i = 0; i < rankCount; ++i) {
                info.faceRanks[static_cast<std::size_t>(i)] = readI32(sl, off, "r6 rank");
            }
            return info;
        }
        if (v2magic == 0x35534452u) {  // "RDS5" ring only
            auto readVar5 = [&]() -> std::uint32_t {
                std::uint32_t v = 0; int shift = 0;
                while (off < sl.size()) {
                    const std::uint8_t b = sl[off++];
                    v |= static_cast<std::uint32_t>(b & 0x7F) << shift;
                    if ((b & 0x80) == 0) break;
                    shift += 7;
                }
                return v;
            };
            auto unzz5 = [](std::uint32_t v) -> int {
                return static_cast<int>((v >> 1) ^ (~(v & 1u) + 1u));
            };
            const auto ringCount = readI32(sl, off, "r5 ring count");
            if (ringCount < 0 || ringCount > 10'000'000) throw std::runtime_error("bad r5 ring");
            for (std::int32_t i = 0; i < ringCount; ++i) {
                if (off >= sl.size()) throw std::runtime_error("truncated r5 ring");
                const auto n = sl[off++];
                info.ringSizes.push_back(n);
                int prevX = 0, prevY = 0, prevZ = 0;
                for (int k = 0; k < static_cast<int>(n); ++k) {
                    prevX += unzz5(readVar5());
                    prevY += unzz5(readVar5());
                    prevZ += unzz5(readVar5());
                    info.ringQX.push_back(prevX);
                    info.ringQY.push_back(prevY);
                    info.ringQZ.push_back(prevZ);
                }
            }
            return info;
        }
        if (v2magic == 0x34534452u) {  // "RDS4" minimal
            const auto absCount = readI32(sl, off, "min absQ count");
            if (absCount < 0 || absCount > 10'000'000) throw std::runtime_error("bad min absQ");
            info.absQX.resize(static_cast<std::size_t>(absCount));
            info.absQY.resize(static_cast<std::size_t>(absCount));
            info.absQZ.resize(static_cast<std::size_t>(absCount));
            for (std::int32_t i = 0; i < absCount; ++i) {
                if (off + 5 > sl.size()) throw std::runtime_error("truncated packed absQ");
                std::uint64_t packed = 0;
                for (int b = 0; b < 5; ++b) packed |= static_cast<std::uint64_t>(sl[off++]) << (8 * b);
                info.absQX[static_cast<std::size_t>(i)] = static_cast<int>(packed & 0xFFFu);
                info.absQY[static_cast<std::size_t>(i)] = static_cast<int>((packed >> 12) & 0xFFFu);
                info.absQZ[static_cast<std::size_t>(i)] = static_cast<int>((packed >> 24) & 0xFFFu);
            }
            auto readVar = [&]() -> std::uint32_t {
                std::uint32_t v = 0;
                int shift = 0;
                while (off < sl.size()) {
                    const std::uint8_t b = sl[off++];
                    v |= static_cast<std::uint32_t>(b & 0x7F) << shift;
                    if ((b & 0x80) == 0) break;
                    shift += 7;
                }
                return v;
            };
            auto unzz = [](std::uint32_t v) -> int {
                return static_cast<int>((v >> 1) ^ (~(v & 1u) + 1u));
            };
            const auto ringCount = readI32(sl, off, "min ring count");
            if (ringCount < 0 || ringCount > 10'000'000) throw std::runtime_error("bad min ring");
            for (std::int32_t i = 0; i < ringCount; ++i) {
                if (off >= sl.size()) throw std::runtime_error("truncated min ring");
                const auto n = sl[off++];
                info.ringSizes.push_back(n);
                int prevX = i < static_cast<std::int32_t>(info.absQX.size()) ? info.absQX[static_cast<std::size_t>(i)] : 0;
                int prevY = i < static_cast<std::int32_t>(info.absQY.size()) ? info.absQY[static_cast<std::size_t>(i)] : 0;
                int prevZ = i < static_cast<std::int32_t>(info.absQZ.size()) ? info.absQZ[static_cast<std::size_t>(i)] : 0;
                for (int k = 0; k < static_cast<int>(n); ++k) {
                    prevX += unzz(readVar());
                    prevY += unzz(readVar());
                    prevZ += unzz(readVar());
                    info.ringQX.push_back(prevX);
                    info.ringQY.push_back(prevY);
                    info.ringQZ.push_back(prevZ);
                }
            }
            return info;
        }
        if (v2magic == 0x32534452u) {  // "RDS2"
            const auto absCount = readI32(sl, off, "absQ count");
            if (absCount < 0 || absCount > 10'000'000) {
                throw std::runtime_error("[RDStrict decoder] invalid absQ count");
            }
            info.absQX.resize(static_cast<std::size_t>(absCount));
            info.absQY.resize(static_cast<std::size_t>(absCount));
            info.absQZ.resize(static_cast<std::size_t>(absCount));
            for (std::int32_t i = 0; i < absCount; ++i) {
                info.absQX[static_cast<std::size_t>(i)] = readI32(sl, off, "absQX");
                info.absQY[static_cast<std::size_t>(i)] = readI32(sl, off, "absQY");
                info.absQZ[static_cast<std::size_t>(i)] = readI32(sl, off, "absQZ");
            }
            const auto faceCount = readI32(sl, off, "faceQ count");
            if (faceCount < 0 || faceCount > 10'000'000) {
                throw std::runtime_error("[RDStrict decoder] invalid faceQ count");
            }
            info.faceQ.resize(static_cast<std::size_t>(faceCount));
            for (std::int32_t i = 0; i < faceCount; ++i) {
                for (int k = 0; k < 9; ++k) {
                    info.faceQ[static_cast<std::size_t>(i)][static_cast<std::size_t>(k)] =
                        readI32(sl, off, "faceQ");
                }
            }
            const auto ringCount = readI32(sl, off, "faceRing count");
            if (ringCount < 0 || ringCount > 10'000'000) {
                throw std::runtime_error("[RDStrict decoder] invalid faceRing count");
            }
            for (std::int32_t i = 0; i < ringCount; ++i) {
                if (off >= sl.size()) {
                    throw std::runtime_error("[RDStrict decoder] truncated faceRing size");
                }
                const auto n = sl[off++];
                info.faceRingSizes.push_back(n);
                for (int k = 0; k < static_cast<int>(n); ++k) {
                    info.faceRingQX.push_back(readI32(sl, off, "faceRingQX"));
                    info.faceRingQY.push_back(readI32(sl, off, "faceRingQY"));
                    info.faceRingQZ.push_back(readI32(sl, off, "faceRingQZ"));
                }
            }
            const auto sideCount = readI32(sl, off, "faceSide count");
            if (sideCount < 0 || sideCount > 10'000'000) {
                throw std::runtime_error("[RDStrict decoder] invalid faceSide count");
            }
            info.faceSide.resize(static_cast<std::size_t>(sideCount));
            for (std::int32_t i = 0; i < sideCount; ++i) {
                if (off >= sl.size()) throw std::runtime_error("[RDStrict decoder] truncated faceSide");
                info.faceSide[static_cast<std::size_t>(i)] = sl[off++];
            }
            const auto ringSideCount = readI32(sl, off, "ringSide count");
            if (ringSideCount < 0 || ringSideCount > 10'000'000) {
                throw std::runtime_error("[RDStrict decoder] invalid ringSide count");
            }
            for (std::int32_t i = 0; i < ringSideCount; ++i) {
                if (off >= sl.size()) throw std::runtime_error("[RDStrict decoder] truncated ringSide size");
                const auto n = sl[off++];
                info.ringSizes.push_back(n);
                for (int k = 0; k < static_cast<int>(n); ++k) {
                    info.ringQX.push_back(readI32(sl, off, "ringQX"));
                    info.ringQY.push_back(readI32(sl, off, "ringQY"));
                    info.ringQZ.push_back(readI32(sl, off, "ringQZ"));
                }
            }
            const auto symCount = readI32(sl, off, "symbolInput count");
            if (symCount < 0 || symCount > 10'000'000) {
                throw std::runtime_error("[RDStrict decoder] invalid symbolInput count");
            }
            info.symbolInputQs.resize(static_cast<std::size_t>(symCount));
            for (std::int32_t i = 0; i < symCount; ++i) {
                for (int k = 0; k < 6; ++k) {
                    info.symbolInputQs[static_cast<std::size_t>(i)][static_cast<std::size_t>(k)] =
                        readI32(sl, off, "symbolInputQ");
                }
            }
            const auto seedCount = readI32(sl, off, "layerSeed count");
            if (seedCount < 0 || seedCount > 10'000'000) {
                throw std::runtime_error("[RDStrict decoder] invalid layerSeed count");
            }
            info.layerSeedQs.resize(static_cast<std::size_t>(seedCount));
            for (std::int32_t i = 0; i < seedCount; ++i) {
                for (int k = 0; k < 6; ++k) {
                    info.layerSeedQs[static_cast<std::size_t>(i)][static_cast<std::size_t>(k)] =
                        readI32(sl, off, "layerSeedQ");
                }
            }
            const auto offsetCount = readI32(sl, off, "inputOffset count");
            if (offsetCount < 0 || offsetCount > 10'000'000) {
                throw std::runtime_error("[RDStrict decoder] invalid inputOffset count");
            }
            info.inputOffsets.resize(static_cast<std::size_t>(offsetCount));
            for (std::int32_t i = 0; i < offsetCount; ++i) {
                if (off >= sl.size()) {
                    throw std::runtime_error("[RDStrict decoder] truncated inputOffset");
                }
                info.inputOffsets[static_cast<std::size_t>(i)] = sl[off++];
            }
        }
    }
    return info;
}

void applyLayerBoundaries(std::vector<RDPatchSymbol>& symbols,
                          const RDStrictSideInfo& info) {
    std::size_t cursor = 0;
    for (std::size_t layer = 0; layer < info.layerSymbolCounts.size(); ++layer) {
        const auto count = info.layerSymbolCounts[layer];
        if (count < 0 || cursor + static_cast<std::size_t>(count) > symbols.size()) {
            throw std::runtime_error("[RDStrict decoder] invalid layer symbol boundary");
        }
        for (std::int32_t i = 0; i < count; ++i) {
            symbols[cursor++].layer = static_cast<unsigned short>(layer);
        }
    }
    if (cursor != symbols.size()) {
        throw std::runtime_error("[RDStrict decoder] layer symbol count mismatch");
    }
}

void applyPartitionInverseKLT(const std::vector<RDPatchSymbol>& symbols,
                              const std::vector<RDStrictPartitionInfo>& partitions,
                              std::vector<int>& x,
                              std::vector<int>& y,
                              std::vector<int>& z) {
    if (partitions.empty()) return;
    std::vector<std::vector<std::size_t>> layerResidualIndices;
    std::size_t residualCursor = 0;
    for (const RDPatchSymbol& s : symbols) {
        if (s.kind == RDPatchSymbolKind::NULL_PATCH) continue;
        const auto layer = static_cast<std::size_t>(s.layer);
        if (layer >= layerResidualIndices.size()) {
            layerResidualIndices.resize(layer + 1);
        }
        layerResidualIndices[layer].push_back(residualCursor++);
    }
    if (residualCursor != x.size() || x.size() != y.size() || x.size() != z.size()) {
        throw std::runtime_error("[RDStrict decoder] KLT residual count mismatch");
    }

    std::vector<std::size_t> layerCursor(layerResidualIndices.size(), 0);
    for (const RDStrictPartitionInfo& p : partitions) {
        const auto layer = static_cast<std::size_t>(p.layer);
        if (layer >= layerResidualIndices.size()) {
            throw std::runtime_error("[RDStrict decoder] KLT partition layer out of range");
        }
        const auto& indices = layerResidualIndices[layer];
        auto& cursor = layerCursor[layer];
        if (p.size < 0 || cursor + static_cast<std::size_t>(p.size) > indices.size()) {
            throw std::runtime_error("[RDStrict decoder] KLT partition size mismatch");
        }
        if (p.useKltFlags == 0) {
            cursor += static_cast<std::size_t>(p.size);
            continue;
        }
        if (p.useKltFlags != 0x07 || p.kltBasis.empty()) {
            throw std::runtime_error("[RDStrict decoder] unsupported KLT partition flags");
        }
        KLTBasis basis{};
        if (unpackKLTBasis(p.kltBasis.data(),
                           static_cast<int>(p.kltBasis.size()),
                           basis) <= 0 || !basis.valid) {
            throw std::runtime_error("[RDStrict decoder] invalid KLT basis");
        }
        std::vector<int> lx;
        std::vector<int> ly;
        std::vector<int> lz;
        lx.reserve(static_cast<std::size_t>(p.size));
        ly.reserve(static_cast<std::size_t>(p.size));
        lz.reserve(static_cast<std::size_t>(p.size));
        for (std::int32_t i = 0; i < p.size; ++i) {
            const auto idx = indices[cursor + static_cast<std::size_t>(i)];
            lx.push_back(x[idx]);
            ly.push_back(y[idx]);
            lz.push_back(z[idx]);
        }
        applyKLTInverse(lx, ly, lz, basis);
        for (std::int32_t i = 0; i < p.size; ++i) {
            const auto idx = indices[cursor + static_cast<std::size_t>(i)];
            x[idx] = lx[static_cast<std::size_t>(i)];
            y[idx] = ly[static_cast<std::size_t>(i)];
            z[idx] = lz[static_cast<std::size_t>(i)];
        }
        cursor += static_cast<std::size_t>(p.size);
    }
}

}  // namespace

void RDStrictSymbolReadOperator::importFromRangeCodedBuffer(
    char* buffer,
    size_t& dataOffset,
    unsigned /*nbQuantBits*/) {
    constexpr size_t kArbitraryLargeView = static_cast<size_t>(1) << 32;
    std::span<const std::uint8_t> view(
        reinterpret_cast<const std::uint8_t*>(buffer + dataOffset),
        kArbitraryLargeView);

    ConnectivityBlobReader rd(view);
    const auto& hdr = rd.header();
    const auto spec = pmsf::rdStrictConnectivityBlobSpec(
        static_cast<std::int32_t>(hdr.num_rounds));
    pmsf::requirePaperConnectivityBlobHeaderMatchesSpec(
        hdr, spec, "RDStrict");
    if (hdr.num_rounds == 0) {
        throw std::runtime_error(
            "[RDStrict decoder] expected at least one round");
    }
    decodedResidualCount_ = 0;
    rounds_.clear();
    if (hdr.num_rounds > 1) {
        decodedSymbols_.clear();
        decodedResidualsX_.clear();
        decodedResidualsY_.clear();
        decodedResidualsZ_.clear();
        bitplaneRefs_.clear();
        decodedBitplaneRefs_ = 0;

        std::vector<std::vector<std::uint8_t>> geometryStreams;
        geometryStreams.reserve(hdr.num_rounds);

        for (std::uint32_t r = 0; r < hdr.num_rounds; ++r) {
            RoundState state;
            state.symbols = decodeConnectivityStream(
                rd.getStream(static_cast<int>(r), rdstrict::Connectivity));
            auto sideStream = rd.getStream(static_cast<int>(r), rdstrict::SideInfo);
            state.sideInfo = parseSideInfo(sideStream, state.partitions);
            applyLayerBoundaries(state.symbols, state.sideInfo);
            auto geom = rd.getStream(static_cast<int>(r), rdstrict::Geometry);
            geometryStreams.emplace_back(geom.begin(), geom.end());
            rounds_.push_back(std::move(state));
        }

        int remainingRefs = 10'000'000;
        const char* maxEnv = std::getenv("PMSF_RDSTRICT_MAX_BITPLANES");
        if (maxEnv && maxEnv[0] != '\0') {
            remainingRefs = std::max(0, std::atoi(maxEnv));
        }

        for (std::size_t r = 0; r < rounds_.size(); ++r) {
            RoundState& state = rounds_[r];
            decodeGeometryStream(std::span<const std::uint8_t>(
                                     geometryStreams[r].data(),
                                     geometryStreams[r].size()),
                                 state.residualCount,
                                 state.residualsX,
                                 state.residualsY,
                                 state.residualsZ,
                                 state.symbols,
                                 state.bitplaneRefs,
                                 state.decodedBitplaneRefs,
                                 remainingRefs);
            applyPartitionInverseKLT(state.symbols,
                                     state.partitions,
                                     state.residualsX,
                                     state.residualsY,
                                     state.residualsZ);
            remainingRefs = std::max(0, remainingRefs - state.decodedBitplaneRefs);

            decodedSymbols_.insert(decodedSymbols_.end(),
                                   state.symbols.begin(),
                                   state.symbols.end());
            decodedResidualsX_.insert(decodedResidualsX_.end(),
                                      state.residualsX.begin(),
                                      state.residualsX.end());
            decodedResidualsY_.insert(decodedResidualsY_.end(),
                                      state.residualsY.begin(),
                                      state.residualsY.end());
            decodedResidualsZ_.insert(decodedResidualsZ_.end(),
                                      state.residualsZ.begin(),
                                      state.residualsZ.end());
            decodedResidualCount_ += state.residualCount;
            decodedBitplaneRefs_ += state.decodedBitplaneRefs;
            for (auto ref : state.bitplaneRefs) {
                ref.round = static_cast<std::uint16_t>(r);
                bitplaneRefs_.push_back(ref);
            }
        }
        sideInfo_ = rounds_.empty() ? RDStrictSideInfo{} : rounds_.front().sideInfo;
        if (!rounds_.empty()) {
            sideInfo_.numVerticesOut = rounds_.back().sideInfo.numVerticesOut;
            sideInfo_.numDecimatedTotal = 0;
            sideInfo_.numNullPatches = 0;
            sideInfo_.layerSymbolCounts.clear();
            sideInfo_.layerVSymbolCounts.clear();
            sideInfo_.layerBitplaneCounts.clear();
            sideInfo_.layerLeakyFactors.clear();
            sideInfo_.layerKltFlags.clear();
            partitionInfos_.clear();
            std::uint16_t layerOffset = 0;
            for (const RoundState& r : rounds_) {
                sideInfo_.numDecimatedTotal += r.sideInfo.numDecimatedTotal;
                sideInfo_.numNullPatches += r.sideInfo.numNullPatches;
                sideInfo_.layerSymbolCounts.insert(sideInfo_.layerSymbolCounts.end(),
                                                   r.sideInfo.layerSymbolCounts.begin(),
                                                   r.sideInfo.layerSymbolCounts.end());
                sideInfo_.layerVSymbolCounts.insert(sideInfo_.layerVSymbolCounts.end(),
                                                   r.sideInfo.layerVSymbolCounts.begin(),
                                                   r.sideInfo.layerVSymbolCounts.end());
                sideInfo_.layerBitplaneCounts.insert(sideInfo_.layerBitplaneCounts.end(),
                                                    r.sideInfo.layerBitplaneCounts.begin(),
                                                    r.sideInfo.layerBitplaneCounts.end());
                sideInfo_.layerLeakyFactors.insert(sideInfo_.layerLeakyFactors.end(),
                                                  r.sideInfo.layerLeakyFactors.begin(),
                                                  r.sideInfo.layerLeakyFactors.end());
                sideInfo_.layerKltFlags.insert(sideInfo_.layerKltFlags.end(),
                                              r.sideInfo.layerKltFlags.begin(),
                                              r.sideInfo.layerKltFlags.end());
                for (auto p : r.partitions) {
                    p.layer = static_cast<std::uint16_t>(p.layer + layerOffset);
                    partitionInfos_.push_back(std::move(p));
                }
                layerOffset = static_cast<std::uint16_t>(
                    layerOffset + r.sideInfo.layerSymbolCounts.size());
            }
        }

        const size_t consumed = hdr.headerByteSize() +
            (hdr.stream_offsets.empty() ? 0 : hdr.stream_offsets.back());
        dataOffset += consumed;
        const auto logSpec = pmsf::rdStrictConnectivityBlobSpec(
            static_cast<std::int32_t>(rounds_.size()));
        std::cerr << "[RDStrict decoder] parsed layout="
                  << logSpec.blobLayout << " sv=" << logSpec.schemeVersion
                  << " rounds=" << rounds_.size()
                  << " symbols=" << decodedSymbols_.size()
                  << " residuals=" << decodedResidualCount_
                  << " bitplaneRefs=" << bitplaneRefs_.size()
                  << " decodedRefs=" << decodedBitplaneRefs_
                  << " V_in=" << sideInfo_.numVerticesIn
                  << " V_base=" << sideInfo_.numVerticesOut
                  << " removed=" << sideInfo_.numDecimatedTotal
                  << "\n";
        return;
    }

    decodedSymbols_ = decodeConnectivityStream(
        rd.getStream(0, rdstrict::Connectivity));
    partitionInfos_.clear();
    sideInfo_ = parseSideInfo(rd.getStream(0, rdstrict::SideInfo), partitionInfos_);
    applyLayerBoundaries(decodedSymbols_, sideInfo_);
    decodeGeometryStream(rd.getStream(0, rdstrict::Geometry),
                         decodedResidualCount_,
                         decodedResidualsX_,
                         decodedResidualsY_,
                         decodedResidualsZ_,
                         decodedSymbols_,
                         bitplaneRefs_,
                         decodedBitplaneRefs_);
    applyPartitionInverseKLT(decodedSymbols_,
                             partitionInfos_,
                             decodedResidualsX_,
                             decodedResidualsY_,
                             decodedResidualsZ_);

    const size_t consumed = hdr.headerByteSize() +
        (hdr.stream_offsets.empty() ? 0 : hdr.stream_offsets.back());
    dataOffset += consumed;

    const auto logSpec = pmsf::rdStrictConnectivityBlobSpec(1);
    std::cerr << "[RDStrict decoder] parsed layout="
              << logSpec.blobLayout << " sv=" << logSpec.schemeVersion
              << " symbols=" << decodedSymbols_.size()
              << " residuals=" << decodedResidualCount_
              << " bitplaneRefs=" << bitplaneRefs_.size()
              << " decodedRefs=" << decodedBitplaneRefs_
              << " V_in=" << sideInfo_.numVerticesIn
              << " V_base=" << sideInfo_.numVerticesOut
              << " removed=" << sideInfo_.numDecimatedTotal
              << "\n";
}

std::shared_ptr<MCGAL::Mesh> cloneMeshForRoundRetry(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!mesh) return nullptr;
    char path[] = "/tmp/pmsf_rdstrict_round_XXXXXX.off";
    const int fd = mkstemps(path, 4);
    if (fd < 0) return nullptr;
    close(fd);
    mesh->dumpto_oldtype(path);
    auto copy = std::make_shared<MCGAL::Mesh>(path);
    std::remove(path);
    if (!copy) return nullptr;
    copy->bboxMin = mesh->bboxMin;
    copy->bboxMax = mesh->bboxMax;
    copy->f_quantStep = mesh->f_quantStep;
    copy->i_nbQuantBits = mesh->i_nbQuantBits;
    copy->enableQuantization = mesh->enableQuantization;
    copy->setMeshId(mesh->meshId());
    return copy;
}

void RDStrictSymbolReadOperator::collectFromDecoded(
    MCGAL::Halfedge* /*seed*/,
    std::vector<MCGAL::Vertex*>& /*vertices*/,
    std::vector<MCGAL::Halfedge*>& /*halfedge*/,
    std::vector<MCGAL::Facet*>& /*facets*/) {
    if (!mesh_) {
        std::cerr << "[RDStrict decoder] no mesh injected; keeping base mesh\n";
        return;
    }
    std::vector<int> firstGateQ;
    firstGateQ.reserve(6);
    for (int i = 0; i < 3; ++i) firstGateQ.push_back(sideInfo_.firstGateV0Q[i]);
    for (int i = 0; i < 3; ++i) firstGateQ.push_back(sideInfo_.firstGateV1Q[i]);

    mesh_->resetState();
    mesh_->invalidateAllVertexDegrees();
    for (MCGAL::Vertex* v : mesh_->vertices()) {
        if (v && !v->isRemoved()) v->clearTag();
    }
    if (!rounds_.empty()) {
        RDPatchDecoderResult aggregate{};
        int decodeRoundOrdinal = 0;
        int decodeRoundLimit = 10'000'000;
        if (const char* e = std::getenv("PMSF_RDSTRICT_DECODE_ROUNDS")) {
            const int v = std::atoi(e);
            if (v > 0) decodeRoundLimit = v;
        }
        for (auto it = rounds_.rbegin(); it != rounds_.rend(); ++it) {
            if (decodeRoundOrdinal >= decodeRoundLimit) break;
            ++decodeRoundOrdinal;
            std::vector<int> roundFirstGateQ;
            roundFirstGateQ.reserve(6);
            for (int i = 0; i < 3; ++i) roundFirstGateQ.push_back(it->sideInfo.firstGateV0Q[i]);
            for (int i = 0; i < 3; ++i) roundFirstGateQ.push_back(it->sideInfo.firstGateV1Q[i]);

            mesh_->resetState();
            mesh_->invalidateAllVertexDegrees();
            for (MCGAL::Vertex* v : mesh_->vertices()) {
                if (v && !v->isRemoved()) v->clearTag();
            }
            if (!it->sideInfo.faceRanks.empty()) {
                lastDecoderResult_ = runFaceIdRDPatchReplay(
                    mesh_, it->symbols, it->residualsX, it->residualsY, it->residualsZ,
                    it->sideInfo.faceRanks);
            }
            if (std::getenv("PMSF_RDSTRICT_POINT_REPLAY") &&
                !it->sideInfo.absQX.empty()) {
                lastDecoderResult_ = runPointRDPatchReplay(
                    mesh_, it->symbols, it->residualsX, it->residualsY, it->residualsZ,
                    it->sideInfo.absQX, it->sideInfo.absQY, it->sideInfo.absQZ);
            }
            const bool rankReplay = !it->sideInfo.faceRanks.empty();
            const bool pointReplay = !rankReplay && std::getenv("PMSF_RDSTRICT_POINT_REPLAY") &&
                                     !it->sideInfo.absQX.empty();
            const bool directRingReplay = !rankReplay && !pointReplay &&
                                          std::getenv("PMSF_RDSTRICT_DIRECT_REPLAY") &&
                                          !it->sideInfo.ringSizes.empty();
            if (directRingReplay) {
                auto roundBackup = cloneMeshForRoundRetry(mesh_);
                lastDecoderResult_ = runDirectRDPatchReplay(
                    mesh_, it->symbols,
                    it->residualsX, it->residualsY, it->residualsZ,
                    /*faceRanks=*/{},
                    (std::getenv("PMSF_RDSTRICT_DIRECT_FACERING")
                         ? it->sideInfo.faceRingSizes
                         : it->sideInfo.ringSizes),
                    (std::getenv("PMSF_RDSTRICT_DIRECT_FACERING")
                         ? it->sideInfo.faceRingQX : it->sideInfo.ringQX),
                    (std::getenv("PMSF_RDSTRICT_DIRECT_FACERING")
                         ? it->sideInfo.faceRingQY : it->sideInfo.ringQY),
                    (std::getenv("PMSF_RDSTRICT_DIRECT_FACERING")
                         ? it->sideInfo.faceRingQZ : it->sideInfo.ringQZ),
                    it->sideInfo.absQX, it->sideInfo.absQY, it->sideInfo.absQZ);
                const bool complete =
                    lastDecoderResult_.numAddedVertices == it->sideInfo.numDecimatedTotal &&
                    lastDecoderResult_.numSymbolsTotal == static_cast<int>(it->symbols.size());
                if (!complete && roundBackup &&
                    !std::getenv("PMSF_RDSTRICT_NO_HYBRID_FALLBACK")) {
                    mesh_ = roundBackup;
                    lastDecoderResult_ = runRDPatchDecoderConquest(
                        mesh_, it->symbols,
                        it->residualsX, it->residualsY, it->residualsZ,
                        /*faceRanks=*/{}, /*ringSizes=*/{}, /*ringQX=*/{},
                        /*ringQY=*/{}, /*ringQZ=*/{},
                        it->sideInfo.absQX, it->sideInfo.absQY, it->sideInfo.absQZ,
                        roundFirstGateQ,
                        it->sideInfo.inputOffsets,
                        it->sideInfo.symbolInputQs,
                        /*forcePMSFBoundaryGates=*/true,
                        /*forceStrictRDConquest=*/true,
                        it->sideInfo.faceQ,
                        it->sideInfo.faceRingSizes,
                        it->sideInfo.faceRingQX,
                        it->sideInfo.faceRingQY,
                        it->sideInfo.faceRingQZ,
                        it->sideInfo.faceSide);
                }
            } else if (!pointReplay && !rankReplay) {
                lastDecoderResult_ = runRDPatchDecoderConquest(
                    mesh_, it->symbols,
                    it->residualsX, it->residualsY, it->residualsZ,
                    /*faceRanks=*/{}, /*ringSizes=*/{}, /*ringQX=*/{},
                    /*ringQY=*/{}, /*ringQZ=*/{},
                    it->sideInfo.absQX, it->sideInfo.absQY, it->sideInfo.absQZ,
                    roundFirstGateQ,
                    it->sideInfo.inputOffsets,
                    it->sideInfo.symbolInputQs,
                    /*forcePMSFBoundaryGates=*/true,
                    /*forceStrictRDConquest=*/true,
                    it->sideInfo.faceQ,
                    it->sideInfo.faceRingSizes,
                    it->sideInfo.faceRingQX,
                    it->sideInfo.faceRingQY,
                    it->sideInfo.faceRingQZ,
                    it->sideInfo.faceSide);
            }
            std::cerr << "[RDStrict decoder] round " << decodeRoundOrdinal
                      << "/" << rounds_.size()
                      << " added=" << lastDecoderResult_.numAddedVertices
                      << " consumed=" << lastDecoderResult_.numSymbolsTotal
                      << "/" << lastDecoderResult_.numSymsAvailable
                      << " V_out=" << lastDecoderResult_.numVerticesOut
                      << "\n";

            if (aggregate.numVerticesIn == 0) {
                aggregate.numVerticesIn = lastDecoderResult_.numVerticesIn;
            }
            aggregate.numVerticesOut = lastDecoderResult_.numVerticesOut;
            aggregate.numAddedVertices += lastDecoderResult_.numAddedVertices;
            aggregate.numNullPopsConsumed += lastDecoderResult_.numNullPopsConsumed;
            aggregate.numVValPopsConsumed += lastDecoderResult_.numVValPopsConsumed;
            aggregate.numSymbolsTotal += lastDecoderResult_.numSymbolsTotal;
            aggregate.numSymsAvailable += lastDecoderResult_.numSymsAvailable;
            aggregate.numDecCycles += lastDecoderResult_.numDecCycles;
            aggregate.numSteps += lastDecoderResult_.numSteps;
            aggregate.reachedMaxSteps = aggregate.reachedMaxSteps || lastDecoderResult_.reachedMaxSteps;
            aggregate.exhaustedSymbols = aggregate.exhaustedSymbols || lastDecoderResult_.exhaustedSymbols;
            for (int i = 0; i < 6; ++i) {
                aggregate.bucketHits[i] += lastDecoderResult_.bucketHits[i];
            }
            if (std::next(it) != rounds_.rend()) {
                mesh_ = canonicalReloadMesh(mesh_);
            }
        }
        lastDecoderResult_ = aggregate;
        std::cerr << "[RDStrict decoder] multi-round conquest reconstruct"
                  << " rounds=" << rounds_.size()
                  << " added=" << lastDecoderResult_.numAddedVertices
                  << " consumed=" << lastDecoderResult_.numSymbolsTotal
                  << "/" << lastDecoderResult_.numSymsAvailable
                  << " V_out=" << lastDecoderResult_.numVerticesOut
                  << "\n";
        return;
    }
    const bool singleDirect = std::getenv("PMSF_RDSTRICT_DIRECT_REPLAY") &&
                              !sideInfo_.ringSizes.empty();
    if (singleDirect) {
        auto backup = cloneMeshForRoundRetry(mesh_);
        lastDecoderResult_ = runDirectRDPatchReplay(
            mesh_, decodedSymbols_, decodedResidualsX_, decodedResidualsY_,
            decodedResidualsZ_, /*faceRanks=*/{},
            sideInfo_.ringSizes, sideInfo_.ringQX, sideInfo_.ringQY, sideInfo_.ringQZ,
            sideInfo_.absQX, sideInfo_.absQY, sideInfo_.absQZ);
        if (lastDecoderResult_.numAddedVertices != sideInfo_.numDecimatedTotal && backup) {
            mesh_ = backup;
            lastDecoderResult_ = runRDPatchDecoderConquest(
                mesh_, decodedSymbols_, decodedResidualsX_, decodedResidualsY_,
                decodedResidualsZ_, /*faceRanks=*/{}, /*ringSizes=*/{},
                /*ringQX=*/{}, /*ringQY=*/{}, /*ringQZ=*/{},
                sideInfo_.absQX, sideInfo_.absQY, sideInfo_.absQZ, firstGateQ,
                sideInfo_.inputOffsets, sideInfo_.symbolInputQs,
                /*forcePMSFBoundaryGates=*/true, /*forceStrictRDConquest=*/true,
                sideInfo_.faceQ, sideInfo_.faceRingSizes, sideInfo_.faceRingQX,
                sideInfo_.faceRingQY, sideInfo_.faceRingQZ, sideInfo_.faceSide);
        }
    } else {
    lastDecoderResult_ = runRDPatchDecoderConquest(
        mesh_,
        decodedSymbols_,
        decodedResidualsX_,
        decodedResidualsY_,
        decodedResidualsZ_,
        /*faceRanks=*/{},
        /*ringSizes=*/{},
        /*ringQX=*/{},
        /*ringQY=*/{},
        /*ringQZ=*/{},
        /*absQX=*/sideInfo_.absQX,
        /*absQY=*/sideInfo_.absQY,
        /*absQZ=*/sideInfo_.absQZ,
        firstGateQ,
        /*inputOffsets=*/sideInfo_.inputOffsets,
        /*symbolInputQs=*/sideInfo_.symbolInputQs,
        /*forcePMSFBoundaryGates=*/true,
        /*forceStrictRDConquest=*/true,
        sideInfo_.faceQ,
        sideInfo_.faceRingSizes,
        sideInfo_.faceRingQX,
        sideInfo_.faceRingQY,
        sideInfo_.faceRingQZ,
        sideInfo_.faceSide);
    }

    std::cerr << "[RDStrict decoder] conquest reconstruct"
              << " added=" << lastDecoderResult_.numAddedVertices
              << " consumed=" << lastDecoderResult_.numSymbolsTotal
              << "/" << lastDecoderResult_.numSymsAvailable
              << " V_out=" << lastDecoderResult_.numVerticesOut
              << "\n";
}

void RDStrictSymbolReadOperator::collect(
    MCGAL::Halfedge* /*seed*/,
    char* /*buffer*/,
    size_t& /*dataOffset*/,
    std::vector<MCGAL::Vertex*>& /*vertices*/,
    std::vector<MCGAL::Halfedge*>& /*halfedge*/,
    std::vector<MCGAL::Facet*>& /*facets*/) {
    throw std::runtime_error("[RDStrict decoder] use importFromRangeCodedBuffer");
}

void RDStrictSymbolReadOperator::collect(
    std::vector<MCGAL::Halfedge*> /*seeds*/,
    char* /*buffer*/,
    size_t& /*dataOffset*/,
    std::vector<MCGAL::Vertex*>& /*vertices*/,
    std::vector<MCGAL::Halfedge*>& /*halfedge*/,
    std::vector<MCGAL::Facet*>& /*facets*/) {
    throw std::runtime_error("[RDStrict decoder] use importFromRangeCodedBuffer");
}

}  // namespace pmsf
