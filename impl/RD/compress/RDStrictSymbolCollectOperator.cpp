// SPDX-License-Identifier: Apache-2.0

#include "RD/compress/RDStrictSymbolCollectOperator.h"
#include "RD/boundary/RDStrictPMSFBoundary.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include "Configuration.h"
#include "compress/ConnectivityBlobContainer.h"
#include "compress/RangeCoderStreamEncoder.h"
#include "common/patch/RDPMSFSymbolMapper.h"
#include "RD/RDStrictLayout.h"
#include "common/patch/RDPatchDecoderConquest.h"
#include "predict/KLTTransform.h"

#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"

#include <algorithm>
#include <array>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <stdexcept>
#include <utility>
#include <vector>
#include <unistd.h>

namespace pmsf {
namespace {

constexpr int kResidualBpBits = 20;

struct OrderEntry {
    std::uint16_t layer = 0;
    std::uint8_t component = 0;
    std::uint8_t bitplane = 0;
    std::uint32_t ones = 0;
    double rateBits = 0.0;
    double importance = 0.0;
};

struct PartitionCodingInfo {
    std::uint16_t layer = 0;
    std::uint16_t partition = 0;
    std::int32_t size = 0;
    std::uint8_t useKltFlags = 0;
    std::vector<std::uint8_t> kltBasis;
};

struct RDStrictCodingPlan {
    int layerCount = 0;
    std::vector<int> layerSymbols;
    std::vector<int> layerVSymbols;
    std::vector<unsigned short> residualLayers;
    std::vector<int> coeffX;
    std::vector<int> coeffY;
    std::vector<int> coeffZ;
    std::vector<PartitionCodingInfo> partitions;
};

int envInt(const char* name, int fallback) {
    const char* v = std::getenv(name);
    if (!v || v[0] == '\0') return fallback;
    return std::max(1, std::atoi(v));
}

bool envDisabled(const char* name) {
    const char* v = std::getenv(name);
    return v && v[0] != '\0' && std::atoi(v) == 0;
}

bool envEnabled(const char* name) {
    return pmsf::pmsfStorageEnvEnabled(name);
}

void requireRDStrictPaperExactIfRequested() {
    if (!pmsf::rdStrictPaperExactRequired()) {
        return;
    }
    throw std::runtime_error(
        "[RDStrictSymbolCollect] paper-exact RD path requested, but "
        "paper inverse transform / exact geometry replay are not implemented "
        "for RD_STRICT(29) yet");
}

int envIntLocal(const char* name, int fallback) {
    const char* v = std::getenv(name);
    if (!v || v[0] == '\0') return fallback;
    return std::max(1, std::atoi(v));
}

void appendU8(std::vector<std::uint8_t>& out, std::uint8_t v) {
    out.push_back(v);
}

void appendI32(std::vector<std::uint8_t>& out, std::int32_t v) {
    const auto u = static_cast<std::uint32_t>(v);
    out.push_back(static_cast<std::uint8_t>(u & 0xffu));
    out.push_back(static_cast<std::uint8_t>((u >> 8) & 0xffu));
    out.push_back(static_cast<std::uint8_t>((u >> 16) & 0xffu));
    out.push_back(static_cast<std::uint8_t>((u >> 24) & 0xffu));
}

void appendU32(std::vector<std::uint8_t>& out, std::uint32_t v) {
    out.push_back(static_cast<std::uint8_t>(v & 0xffu));
    out.push_back(static_cast<std::uint8_t>((v >> 8) & 0xffu));
    out.push_back(static_cast<std::uint8_t>((v >> 16) & 0xffu));
    out.push_back(static_cast<std::uint8_t>((v >> 24) & 0xffu));
}

void appendU16(std::vector<std::uint8_t>& out, std::uint16_t v) {
    out.push_back(static_cast<std::uint8_t>(v & 0xffu));
    out.push_back(static_cast<std::uint8_t>((v >> 8) & 0xffu));
}

void appendF64(std::vector<std::uint8_t>& out, double v) {
    std::uint64_t u = 0;
    static_assert(sizeof(u) == sizeof(v));
    std::memcpy(&u, &v, sizeof(v));
    for (int i = 0; i < 8; ++i) {
        out.push_back(static_cast<std::uint8_t>((u >> (8 * i)) & 0xffu));
    }
}

int symbolCode(const RDPatchSymbol& s) {
    return static_cast<int>(s.kind);
}

std::uint32_t zigZag32(int value) {
    return (static_cast<std::uint32_t>(value) << 1)
           ^ static_cast<std::uint32_t>(value >> 31);
}

std::vector<unsigned short> residualLayersFor(
    const RDPatchPipelineResult& result) {
    std::vector<unsigned short> residualLayers;
    residualLayers.reserve(result.vvResidualsX.size());
    for (const RDPatchSymbol& s : result.symbols) {
        if (s.kind != RDPatchSymbolKind::NULL_PATCH) {
            residualLayers.push_back(s.layer);
        }
    }
    if (residualLayers.size() != result.vvResidualsX.size()) {
        throw std::runtime_error("[RDStrictSymbolCollect] residual/layer count mismatch");
    }
    return residualLayers;
}

std::vector<int> layerVSymbolCountsFor(const RDPatchPipelineResult& result,
                                       int layerCount) {
    std::vector<int> layerVSymbols(static_cast<std::size_t>(std::max(layerCount, 0)), 0);
    for (const RDPatchSymbol& s : result.symbols) {
        const int layer = static_cast<int>(s.layer);
        if (layer < 0 || layer >= layerCount) continue;
        if (s.kind != RDPatchSymbolKind::NULL_PATCH) {
            ++layerVSymbols[static_cast<std::size_t>(layer)];
        }
    }
    return layerVSymbols;
}

std::vector<int> layerSymbolCountsFor(const RDPatchPipelineResult& result,
                                      int layerCount) {
    std::vector<int> layerSymbols(static_cast<std::size_t>(std::max(layerCount, 0)), 0);
    for (const RDPatchSymbol& s : result.symbols) {
        const int layer = static_cast<int>(s.layer);
        if (layer < 0 || layer >= layerCount) continue;
        ++layerSymbols[static_cast<std::size_t>(layer)];
    }
    return layerSymbols;
}

RDStrictCodingPlan buildCodingPlan(const RDPatchPipelineResult& result) {
    RDStrictCodingPlan plan;
    int maxLayer = -1;
    for (const RDPatchSymbol& s : result.symbols) {
        maxLayer = std::max(maxLayer, static_cast<int>(s.layer));
    }
    plan.layerCount = maxLayer + 1;
    plan.layerSymbols = layerSymbolCountsFor(result, plan.layerCount);
    plan.layerVSymbols = layerVSymbolCountsFor(result, plan.layerCount);
    plan.residualLayers = residualLayersFor(result);
    plan.coeffX = result.vvResidualsX;
    plan.coeffY = result.vvResidualsY;
    plan.coeffZ = result.vvResidualsZ;

    if (plan.coeffX.size() != plan.coeffY.size() ||
        plan.coeffX.size() != plan.coeffZ.size()) {
        throw std::runtime_error("[RDStrictSymbolCollect] residual stream size mismatch");
    }
    if (envDisabled("PMSF_RDSTRICT_USE_KLT")) {
        return plan;
    }

    const int threshold = envInt("PMSF_RDSTRICT_PARTITION_SIZE", 1024);
    for (int layer = 0; layer < plan.layerCount; ++layer) {
        std::vector<std::size_t> indices;
        indices.reserve(static_cast<std::size_t>(std::max(plan.layerVSymbols[static_cast<std::size_t>(layer)], 0)));
        for (std::size_t i = 0; i < plan.residualLayers.size(); ++i) {
            if (static_cast<int>(plan.residualLayers[i]) == layer) {
                indices.push_back(i);
            }
        }
        std::size_t pos = 0;
        std::uint16_t partitionId = 0;
        while (pos < indices.size()) {
            const std::size_t end = std::min(pos + static_cast<std::size_t>(threshold),
                                             indices.size());
            std::vector<int> x;
            std::vector<int> y;
            std::vector<int> z;
            x.reserve(end - pos);
            y.reserve(end - pos);
            z.reserve(end - pos);
            for (std::size_t k = pos; k < end; ++k) {
                const auto idx = indices[k];
                x.push_back(plan.coeffX[idx]);
                y.push_back(plan.coeffY[idx]);
                z.push_back(plan.coeffZ[idx]);
            }

            PartitionCodingInfo p;
            p.layer = static_cast<std::uint16_t>(layer);
            p.partition = partitionId++;
            p.size = static_cast<std::int32_t>(x.size());

            KLTBasis basis = computeKLTBasis(x, y, z);
            unsigned char packed[10] = {};
            const int packedLen = packKLTBasis(basis, packed, 10);
            if (packedLen > 0) {
                KLTBasis quantized{};
                if (unpackKLTBasis(packed, packedLen, quantized) <= 0 || !quantized.valid) {
                    throw std::runtime_error("[RDStrictSymbolCollect] invalid quantized KLT basis");
                }
                applyKLTForward(x, y, z, quantized);
                p.useKltFlags = 0x07;
                p.kltBasis.assign(packed, packed + packedLen);
                for (std::size_t k = pos; k < end; ++k) {
                    const auto idx = indices[k];
                    const auto local = k - pos;
                    plan.coeffX[idx] = x[local];
                    plan.coeffY[idx] = y[local];
                    plan.coeffZ[idx] = z[local];
                }
            }
            plan.partitions.push_back(std::move(p));
            pos = end;
        }
    }
    return plan;
}

std::vector<OrderEntry> buildBitplaneOrder(const RDPatchPipelineResult& result,
                                           int layerCount,
                                           const std::vector<int>& layerVSymbols,
                                           const std::vector<unsigned short>& residualLayers,
                                           const std::vector<int>& coeffX,
                                           const std::vector<int>& coeffY,
                                           const std::vector<int>& coeffZ) {
    std::vector<double> propagationW(static_cast<std::size_t>(std::max(layerCount, 0)), 0.0);
    std::uint64_t suffix = 0;
    for (int l = layerCount; l-- > 0;) {
        suffix += static_cast<std::uint64_t>(layerVSymbols[static_cast<std::size_t>(l)]);
        const int n = layerVSymbols[static_cast<std::size_t>(l)];
        propagationW[static_cast<std::size_t>(l)] =
            (n > 0) ? static_cast<double>(result.numVerticesOut + suffix)
                        / static_cast<double>(n)
                    : 0.0;
    }

    std::vector<OrderEntry> order;
    order.reserve(static_cast<std::size_t>(std::max(layerCount, 0) * 3 * kResidualBpBits));
    const std::vector<int>* comps[3] = {
        &coeffX,
        &coeffY,
        &coeffZ,
    };
    for (int l = 0; l < layerCount; ++l) {
        const int layerN = layerVSymbols[static_cast<std::size_t>(l)];
        if (layerN <= 0) continue;
        for (int c = 0; c < 3; ++c) {
            for (int p = 0; p < kResidualBpBits; ++p) {
                std::uint32_t ones = 0;
                for (std::size_t i = 0; i < residualLayers.size(); ++i) {
                    if (static_cast<int>(residualLayers[i]) != l) continue;
                    const int bitIndex = kResidualBpBits - 1 - p;
                    if ((zigZag32((*comps[c])[i]) >> bitIndex) & 1u) ++ones;
                }
                if (ones == 0) {
                    continue;
                }
                const double prob = static_cast<double>(ones) / static_cast<double>(layerN);
                double entropy = 0.0;
                if (prob > 0.0 && prob < 1.0) {
                    entropy = -prob * std::log2(prob)
                              - (1.0 - prob) * std::log2(1.0 - prob);
                }
                const double rateBits = std::max(1.0, entropy * layerN);
                const double step = static_cast<double>(1u << (kResidualBpBits - 1 - p));
                const double deltaD = propagationW[static_cast<std::size_t>(l)]
                                      * static_cast<double>(ones) * step * step;
                order.push_back(OrderEntry{
                    static_cast<std::uint16_t>(l),
                    static_cast<std::uint8_t>(c),
                    static_cast<std::uint8_t>(p),
                    ones,
                    rateBits,
                    deltaD / rateBits});
            }
        }
    }
    std::stable_sort(order.begin(), order.end(),
                     [](const OrderEntry& a, const OrderEntry& b) {
                         if (a.importance != b.importance) return a.importance > b.importance;
                         if (a.layer != b.layer) return a.layer < b.layer;
                         if (a.component != b.component) return a.component < b.component;
                         return a.bitplane < b.bitplane;
                     });
    return order;
}

std::vector<std::uint8_t> encodeConnectivityStream(
    const std::vector<RDPatchSymbol>& symbols) {
    std::deque<std::uint8_t> codes;
    std::deque<std::uint8_t> otherVals;
    for (const RDPatchSymbol& s : symbols) {
        codes.push_back(static_cast<std::uint8_t>(symbolCode(s)));
        if (s.kind == RDPatchSymbolKind::V_OTHER) {
            otherVals.push_back(s.escape_valence);
        }
    }

    const std::size_t bufSize = std::max<std::size_t>(BUFFER_SIZE / 12,
                                                      symbols.size() * 2 + 1024);
    std::vector<char> buf(bufSize);

    auto contextFor = [](int prev) {
        if (prev < 0) return 0;
        if (prev == static_cast<int>(RDPatchSymbolKind::NULL_PATCH)) return 1;
        if (prev == static_cast<int>(RDPatchSymbolKind::V_OTHER)) return 3;
        return 2;
    };

    const std::size_t rcSize = pmsf::encodeContextAdaptiveUint8Queue(
        codes, 6, 4, contextFor,
        &otherVals, 256, static_cast<int>(RDPatchSymbolKind::V_OTHER),
        buf.data(), bufSize);

    std::vector<std::uint8_t> out;
    appendI32(out, static_cast<std::int32_t>(symbols.size()));
    appendI32(out, static_cast<std::int32_t>(otherVals.size()));
    out.insert(out.end(),
               reinterpret_cast<std::uint8_t*>(buf.data()),
               reinterpret_cast<std::uint8_t*>(buf.data()) + rcSize);
    return out;
}

std::vector<std::uint8_t> buildGeometryStream(
    const RDPatchPipelineResult& result,
    const RDStrictCodingPlan& plan) {
    const auto& rx = plan.coeffX;
    const auto& ry = plan.coeffY;
    const auto& rz = plan.coeffZ;
    if (rx.size() != ry.size() || rx.size() != rz.size()) {
        throw std::runtime_error("[RDStrictSymbolCollect] residual stream size mismatch");
    }

    std::vector<std::uint8_t> out;
    appendU32(out, rdstrict::kGeometryMagic);
    appendU32(out, pmsf::rdStrictConnectivityBlobSpec(1).schemeVersion);
    appendI32(out, static_cast<std::int32_t>(rx.size()));
    appendU8(out, result.leakyFactors);
    appendU8(out, 1);  // flags: bit 0 = residual payload present.
    appendU8(out, 2);  // encoding: per-(layer, component, bitplane) RC payload table.
    appendU8(out, 0);

    appendU8(out, kResidualBpBits);
    appendU8(out, 0);
    appendU8(out, 0);
    appendU8(out, 0);

    const int layerCount = plan.layerCount;
    const auto& residualLayers = plan.residualLayers;
    const auto& layerVSymbols = plan.layerVSymbols;
    const std::vector<int>* comps[3] = {&rx, &ry, &rz};

    struct Payload {
        std::uint16_t layer = 0;
        std::uint8_t component = 0;
        std::uint8_t bitplane = 0;
        std::int32_t bitCount = 0;
        std::vector<std::uint8_t> bytes;
    };
    std::vector<Payload> payloads;
    payloads.reserve(static_cast<std::size_t>(std::max(layerCount, 0) * 3 * kResidualBpBits));

    auto encodePlane = [&](int layer, int component, int plane) {
        std::vector<std::uint8_t> bits;
        bits.reserve(static_cast<std::size_t>(std::max(layerVSymbols[static_cast<std::size_t>(layer)], 0)));
        const int bitIndex = kResidualBpBits - 1 - plane;
        for (std::size_t i = 0; i < residualLayers.size(); ++i) {
            if (static_cast<int>(residualLayers[i]) != layer) continue;
            bits.push_back(static_cast<std::uint8_t>(
                (zigZag32((*comps[component])[i]) >> bitIndex) & 1u));
        }

        Payload p;
        p.layer = static_cast<std::uint16_t>(layer);
        p.component = static_cast<std::uint8_t>(component);
        p.bitplane = static_cast<std::uint8_t>(plane);
        p.bitCount = static_cast<std::int32_t>(bits.size());
        if (!bits.empty()) {
            const std::size_t bufSize = std::max<std::size_t>(64, bits.size() * 2 + 64);
            std::vector<char> buf(bufSize);
            std::deque<uint8_t> bitsDeque(bits.begin(), bits.end());
            const std::size_t rcSize = pmsf::encodeUint8Queue(
                bitsDeque, 2, buf.data(), bufSize);
            p.bytes.insert(p.bytes.end(),
                           reinterpret_cast<std::uint8_t*>(buf.data()),
                           reinterpret_cast<std::uint8_t*>(buf.data()) + rcSize);
        }
        payloads.push_back(std::move(p));
    };

    for (int layer = 0; layer < layerCount; ++layer) {
        if (layerVSymbols[static_cast<std::size_t>(layer)] <= 0) continue;
        for (int component = 0; component < 3; ++component) {
            for (int plane = 0; plane < kResidualBpBits; ++plane) {
                encodePlane(layer, component, plane);
            }
        }
    }

    auto order = buildBitplaneOrder(result, layerCount, layerVSymbols, residualLayers, rx, ry, rz);
    std::vector<Payload> orderedPayloads;
    orderedPayloads.reserve(payloads.size());
    for (const OrderEntry& e : order) {
        auto it = std::find_if(payloads.begin(), payloads.end(),
                               [&](const Payload& p) {
                                   return p.layer == e.layer &&
                                          p.component == e.component &&
                                          p.bitplane == e.bitplane;
                               });
        if (it == payloads.end()) {
            throw std::runtime_error("[RDStrictSymbolCollect] RD payload order mismatch");
        }
        orderedPayloads.push_back(std::move(*it));
    }

    appendI32(out, static_cast<std::int32_t>(orderedPayloads.size()));
    for (const Payload& p : orderedPayloads) {
        appendU16(out, p.layer);
        appendU8(out, p.component);
        appendU8(out, p.bitplane);
        appendI32(out, p.bitCount);
        appendU32(out, static_cast<std::uint32_t>(p.bytes.size()));
        out.insert(out.end(), p.bytes.begin(), p.bytes.end());
    }
    return out;
}

void appendVarU32(std::vector<std::uint8_t>& out, std::uint32_t v) {
    while (v >= 0x80) { out.push_back(static_cast<std::uint8_t>(v | 0x80)); v >>= 7; }
    out.push_back(static_cast<std::uint8_t>(v));
}

std::vector<std::uint8_t> buildSideInfoStream(
    const RDPatchPipelineResult& result,
    const RDStrictCodingPlan& plan) {
    std::vector<std::uint8_t> out;
    appendU32(out, rdstrict::kSideInfoMagic);
    appendU32(out, pmsf::rdStrictConnectivityBlobSpec(1).schemeVersion);
    appendI32(out, result.numVerticesIn);
    appendI32(out, result.numVerticesOut);
    appendI32(out, result.numDecimatedTotal);
    appendI32(out, result.numNullPatches);
    appendI32(out, result.numDecCycles);
    for (int i = 0; i < 3; ++i) appendI32(out, result.firstGateV0Q[i]);
    for (int i = 0; i < 3; ++i) appendI32(out, result.firstGateV1Q[i]);

    const int layerCount = plan.layerCount;
    appendI32(out, layerCount);
    for (int i = 0; i < layerCount; ++i) {
        appendI32(out, i);
        appendI32(out, plan.layerSymbols[static_cast<std::size_t>(i)]);
        appendI32(out, plan.layerVSymbols[static_cast<std::size_t>(i)]);
    }

    for (int i = 0; i < layerCount; ++i) {
        appendU8(out, kResidualBpBits);
        appendU8(out, result.leakyFactors);
        std::uint8_t layerFlags = 0;
        int partitionCount = 0;
        for (const auto& p : plan.partitions) {
            if (p.layer == i) {
                layerFlags |= p.useKltFlags;
                ++partitionCount;
            }
        }
        appendU8(out, layerFlags);
        appendU8(out, 0);
        appendI32(out, partitionCount);
        for (const auto& p : plan.partitions) {
            if (p.layer != i) continue;
            appendU16(out, p.partition);
            appendI32(out, p.size);
            appendU8(out, p.useKltFlags);
            appendU8(out, static_cast<std::uint8_t>(p.kltBasis.size()));
            out.insert(out.end(), p.kltBasis.begin(), p.kltBasis.end());
        }
    }
    if (std::getenv("PMSF_RDSTRICT_FACE_RANK")) {
        appendU32(out, 0x36534452u);  // "RDS6" face rank only
        appendI32(out, static_cast<std::int32_t>(result.vvFaceRank.size()));
        for (int rank : result.vvFaceRank) appendI32(out, rank);
        return out;
    }
    if (std::getenv("PMSF_RDSTRICT_RING_ONLY")) {
        appendU32(out, 0x35534452u);  // "RDS5" ring only
        appendI32(out, static_cast<std::int32_t>(result.vvRingSizes.size()));
        std::size_t ringOnlyOff = 0;
        for (auto n : result.vvRingSizes) {
            appendU8(out, n);
            int prevX = 0, prevY = 0, prevZ = 0;
            for (int k = 0; k < static_cast<int>(n); ++k) {
                auto zz = [](int d) { return (static_cast<std::uint32_t>(d) << 1) ^ static_cast<std::uint32_t>(d >> 31); };
                appendVarU32(out, zz(result.vvRingQX[ringOnlyOff] - prevX));
                appendVarU32(out, zz(result.vvRingQY[ringOnlyOff] - prevY));
                appendVarU32(out, zz(result.vvRingQZ[ringOnlyOff] - prevZ));
                prevX = result.vvRingQX[ringOnlyOff];
                prevY = result.vvRingQY[ringOnlyOff];
                prevZ = result.vvRingQZ[ringOnlyOff];
                ++ringOnlyOff;
            }
        }
        return out;
    }
    if (std::getenv("PMSF_RDSTRICT_MINIMAL_SIDE")) {
        appendU32(out, 0x34534452u);  // "RDS4" minimal: absQ + ring only
        appendI32(out, static_cast<std::int32_t>(result.vvAbsQX.size()));
        for (std::size_t i = 0; i < result.vvAbsQX.size(); ++i) {
            const std::uint64_t packed =
                (static_cast<std::uint64_t>(result.vvAbsQX[i]) & 0xFFFu) |
                ((static_cast<std::uint64_t>(result.vvAbsQY[i]) & 0xFFFu) << 12) |
                ((static_cast<std::uint64_t>(result.vvAbsQZ[i]) & 0xFFFu) << 24);
            for (int b = 0; b < 5; ++b) out.push_back(static_cast<std::uint8_t>(packed >> (8 * b)));
        }
        appendI32(out, static_cast<std::int32_t>(result.vvRingSizes.size()));
        std::size_t minimalRingOff = 0;
        std::size_t minimalRingV = 0;
        for (auto n : result.vvRingSizes) {
            appendU8(out, n);
            int prevX = minimalRingV < result.vvAbsQX.size() ? result.vvAbsQX[minimalRingV] : 0;
            int prevY = minimalRingV < result.vvAbsQY.size() ? result.vvAbsQY[minimalRingV] : 0;
            int prevZ = minimalRingV < result.vvAbsQZ.size() ? result.vvAbsQZ[minimalRingV] : 0;
            ++minimalRingV;
            for (int k = 0; k < static_cast<int>(n); ++k) {
                auto zz = [](int d) { return (static_cast<std::uint32_t>(d) << 1) ^ static_cast<std::uint32_t>(d >> 31); };
                appendVarU32(out, zz(result.vvRingQX[minimalRingOff] - prevX));
                appendVarU32(out, zz(result.vvRingQY[minimalRingOff] - prevY));
                appendVarU32(out, zz(result.vvRingQZ[minimalRingOff] - prevZ));
                prevX = result.vvRingQX[minimalRingOff];
                prevY = result.vvRingQY[minimalRingOff];
                prevZ = result.vvRingQZ[minimalRingOff];
                ++minimalRingOff;
            }
        }
        return out;
    }
    // v2: residual-facet alignment side channel. Old v1 readers stop at the
    // layer table; new readers detect the trailing magic.
    appendU32(out, 0x32534452u);  // "RDS2"
    appendI32(out, static_cast<std::int32_t>(result.vvAbsQX.size()));
    for (std::size_t i = 0; i < result.vvAbsQX.size(); ++i) {
        appendI32(out, result.vvAbsQX[i]);
        appendI32(out, result.vvAbsQY[i]);
        appendI32(out, result.vvAbsQZ[i]);
    }
    appendI32(out, static_cast<std::int32_t>(result.vvFaceQ.size()));
    for (const auto& fq : result.vvFaceQ) {
        for (int k = 0; k < 9; ++k) appendI32(out, fq[static_cast<std::size_t>(k)]);
    }
    appendI32(out, static_cast<std::int32_t>(result.vvFaceRingSizes.size()));
    std::size_t ringCoord = 0;
    for (auto n : result.vvFaceRingSizes) {
        appendU8(out, n);
        for (int k = 0; k < static_cast<int>(n); ++k) {
            appendI32(out, result.vvFaceRingQX[ringCoord]);
            appendI32(out, result.vvFaceRingQY[ringCoord]);
            appendI32(out, result.vvFaceRingQZ[ringCoord]);
            ++ringCoord;
        }
    }
    appendI32(out, static_cast<std::int32_t>(result.vvFaceSide.size()));
    for (auto v : result.vvFaceSide) appendU8(out, v);
    appendI32(out, static_cast<std::int32_t>(result.vvRingSizes.size()));
    std::size_t ringQOff = 0;
    for (auto n : result.vvRingSizes) {
        appendU8(out, n);
        for (int k = 0; k < static_cast<int>(n); ++k) {
            appendI32(out, result.vvRingQX[ringQOff]);
            appendI32(out, result.vvRingQY[ringQOff]);
            appendI32(out, result.vvRingQZ[ringQOff]);
            ++ringQOff;
        }
    }
    appendI32(out, static_cast<std::int32_t>(result.symbolInputQs.size()));
    for (const auto& q : result.symbolInputQs) {
        for (int k = 0; k < 6; ++k) appendI32(out, q[static_cast<std::size_t>(k)]);
    }
    appendI32(out, static_cast<std::int32_t>(result.layerSeedQs.size()));
    for (const auto& q : result.layerSeedQs) {
        for (int k = 0; k < 6; ++k) appendI32(out, q[static_cast<std::size_t>(k)]);
    }
    appendI32(out, static_cast<std::int32_t>(result.vvInputOffsets.size()));
    for (auto v : result.vvInputOffsets) appendU8(out, v);
    return out;
}

std::shared_ptr<MCGAL::Mesh> cloneMeshForBackwardEmit(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!mesh) return nullptr;
    char path[] = "/tmp/pmsf_rdstrict_base_XXXXXX.off";
    const int fd = mkstemps(path, 4);
    if (fd < 0) {
        throw std::runtime_error("[RDStrictSymbolCollect] cannot create temporary base mesh");
    }
    close(fd);
    mesh->dumpto_oldtype(path);
    auto copy = std::make_shared<MCGAL::Mesh>();
    if (!copy->loadOFF(path)) {
        std::remove(path);
        throw std::runtime_error("[RDStrictSymbolCollect] cannot reload temporary base mesh");
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

RDPatchPipelineResult filterRemainingForwardRecords(
    const RDPatchPipelineResult& src,
    const std::vector<int>& usedForwardIndices) {
    std::vector<char> used(src.vvResidualsX.size(), 0);
    for (int idx : usedForwardIndices) {
        if (idx >= 0 && idx < static_cast<int>(used.size())) {
            used[static_cast<std::size_t>(idx)] = 1;
        }
    }

    RDPatchPipelineResult out;
    out.mesh = src.mesh;
    out.numVerticesIn = src.numVerticesIn;
    out.numVerticesOut = src.numVerticesOut;
    out.firstGateV0Q[0] = src.firstGateV0Q[0];
    out.firstGateV0Q[1] = src.firstGateV0Q[1];
    out.firstGateV0Q[2] = src.firstGateV0Q[2];
    out.firstGateV1Q[0] = src.firstGateV1Q[0];
    out.firstGateV1Q[1] = src.firstGateV1Q[1];
    out.firstGateV1Q[2] = src.firstGateV1Q[2];
    out.layerSeedQs = src.layerSeedQs;
    out.leakyFactors = src.leakyFactors;

    std::size_t ringCursor = 0;
    int vCursor = 0;
    for (std::size_t si = 0; si < src.symbols.size(); ++si) {
        const RDPatchSymbol& sym = src.symbols[si];
        if (sym.kind == RDPatchSymbolKind::NULL_PATCH) continue;
        const int cur = vCursor++;
        const int ringN = (cur >= 0 && cur < static_cast<int>(src.vvRingSizes.size()))
                              ? static_cast<int>(src.vvRingSizes[static_cast<std::size_t>(cur)])
                              : 0;
        if (cur >= 0 && cur < static_cast<int>(used.size()) && !used[static_cast<std::size_t>(cur)]) {
            out.symbols.push_back(sym);
            if (si < src.symbolInputQs.size()) out.symbolInputQs.push_back(src.symbolInputQs[si]);
            out.vvResidualsX.push_back(src.vvResidualsX[static_cast<std::size_t>(cur)]);
            out.vvResidualsY.push_back(src.vvResidualsY[static_cast<std::size_t>(cur)]);
            out.vvResidualsZ.push_back(src.vvResidualsZ[static_cast<std::size_t>(cur)]);
            if (cur < static_cast<int>(src.vvAbsQX.size())) {
                out.vvAbsQX.push_back(src.vvAbsQX[static_cast<std::size_t>(cur)]);
                out.vvAbsQY.push_back(src.vvAbsQY[static_cast<std::size_t>(cur)]);
                out.vvAbsQZ.push_back(src.vvAbsQZ[static_cast<std::size_t>(cur)]);
            }
            if (cur < static_cast<int>(src.vvInputOffsets.size())) {
                out.vvInputOffsets.push_back(src.vvInputOffsets[static_cast<std::size_t>(cur)]);
            }
            out.vvRingSizes.push_back(static_cast<unsigned char>(ringN));
            for (int k = 0; k < ringN; ++k) {
                const std::size_t qi = ringCursor + static_cast<std::size_t>(k);
                if (qi < src.vvRingQX.size() && qi < src.vvRingQY.size() &&
                    qi < src.vvRingQZ.size()) {
                    out.vvRingQX.push_back(src.vvRingQX[qi]);
                    out.vvRingQY.push_back(src.vvRingQY[qi]);
                    out.vvRingQZ.push_back(src.vvRingQZ[qi]);
                }
            }
            ++out.numDecimatedTotal;
        }
        ringCursor += static_cast<std::size_t>(std::max(ringN, 0));
    }
    if (!out.symbolInputQs.empty()) {
        const auto& q = out.symbolInputQs.front();
        out.firstGateV0Q[0] = q[0];
        out.firstGateV0Q[1] = q[1];
        out.firstGateV0Q[2] = q[2];
        out.firstGateV1Q[0] = q[3];
        out.firstGateV1Q[1] = q[4];
        out.firstGateV1Q[2] = q[5];
        out.layerSeedQs.clear();
        out.layerSeedQs.push_back(q);
    }
    return out;
}

std::vector<RDPatchPipelineResult> buildSplitDecodeRounds(
    const std::shared_ptr<MCGAL::Mesh>& baseMesh,
    const RDPatchPipelineResult& forward) {
    std::vector<RDPatchPipelineResult> rounds;
    if (!baseMesh || forward.numDecimatedTotal <= 0) return rounds;
    auto simMesh = cloneMeshForBackwardEmit(baseMesh);
    RDPatchPipelineResult remaining = forward;
    const int maxSubRounds = envIntLocal("PMSF_RDSTRICT_MAX_SUBROUNDS", 32);
    int total = 0;
    for (int i = 0; i < maxSubRounds && remaining.numDecimatedTotal > 0; ++i) {
        RDPatchPipelineResult round =
            rebuildRDPatchResultInStrictDecodeOrder(simMesh, remaining);
        if (round.numDecimatedTotal <= 0 || round.vvDecodeFaceRanks.empty()) break;
        total += round.numDecimatedTotal;
        std::vector<int> used = round.vvDecodeFaceRanks;
        round.vvDecodeFaceRanks.clear();
        rounds.push_back(std::move(round));
        remaining = filterRemainingForwardRecords(remaining, used);
        std::fprintf(stderr,
                     "[RDStrictSymbolCollect] split backward subround %d removed=%d remaining=%d\n",
                     i + 1,
                     rounds.back().numDecimatedTotal,
                     remaining.numDecimatedTotal);
    }
    if (total != forward.numDecimatedTotal) {
        rounds.clear();
    }
    return rounds;
}

std::vector<std::array<int, 3>> absQsFromResult(const RDPatchPipelineResult& r) {
    std::vector<std::array<int, 3>> out;
    const std::size_t n = std::min(r.vvAbsQX.size(),
                         std::min(r.vvAbsQY.size(), r.vvAbsQZ.size()));
    out.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        out.push_back({r.vvAbsQX[i], r.vvAbsQY[i], r.vvAbsQZ[i]});
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

bool sameAbsQSet(const std::vector<std::array<int, 3>>& a,
                 const std::vector<std::array<int, 3>>& b) {
    return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin());
}

}  // namespace

void RDStrictSymbolCollectOperator::collect(MCGAL::Halfedge* /*h*/) {
    if (!mesh_) {
        throw std::runtime_error("[RDStrictSymbolCollect] collect before init");
    }
    splitDecodeRounds_.clear();
    auto originalMesh = cloneMeshForBackwardEmit(mesh_);
    lastResult_ = runRDPatchEncoderOnMesh(mesh_, cfg_);
    const int targetBaseEnv = []() {
        const char* e = std::getenv("PMSF_RDSTRICT_TARGET_BASE");
        const int v = e ? std::atoi(e) : 50;
        return v > 0 ? v : 50;
    }();
    const int outerLimit = []() {
        const char* e = std::getenv("PMSF_RDSTRICT_OUTER_ROUNDS");
        const int v = e ? std::atoi(e) : 64;
        return v > 0 ? v : 64;
    }();
    std::vector<RDPatchPipelineResult> layers;
    int originalIn = lastResult_.numVerticesIn;
    int guard = 0;
    std::fprintf(stderr, "[RDStrict outer] start Vout=%d target=%d limit=%d dec=%d\n",
                 lastResult_.numVerticesOut, targetBaseEnv, outerLimit,
                 lastResult_.numDecimatedTotal);
    while (guard++ < outerLimit && lastResult_.mesh) {
        layers.push_back(lastResult_);
        if (lastResult_.numVerticesOut <= targetBaseEnv ||
            lastResult_.numDecimatedTotal <= 0) {
            break;
        }
        auto nextMesh = cloneMeshForBackwardEmit(lastResult_.mesh);
        auto next = runRDPatchEncoderOnMesh(nextMesh, cfg_);
        std::fprintf(stderr, "[RDStrict outer] layer=%zu nextVout=%d dec=%d\n",
                     layers.size(), next.numVerticesOut, next.numDecimatedTotal);
        if (next.numDecimatedTotal <= 0) break;
        lastResult_ = std::move(next);
    }
    if (!layers.empty() && lastResult_.numDecimatedTotal > 0 &&
        layers.back().mesh != lastResult_.mesh) {
        // The outer loop can stop on the round limit after running one more
        // layer; include that final result before reorder/export.
        layers.push_back(lastResult_);
    }
    if (!layers.empty()) {
        const bool reorderLayers = !std::getenv("PMSF_RDSTRICT_NO_OUTER_REORDER");
        if (reorderLayers) {
            int reorderedCount = 0;
            std::vector<RDPatchPipelineResult> ordered;
            ordered.reserve(layers.size());
            for (auto& layer : layers) {
                auto baseCopy = cloneMeshForBackwardEmit(layer.mesh);
                RDPatchPipelineResult re =
                    rebuildRDPatchResultInStrictDecodeOrder(baseCopy, layer);
                if (re.numDecimatedTotal == layer.numDecimatedTotal) {
                    re.mesh = layer.mesh;
                    ordered.push_back(std::move(re));
                    ++reorderedCount;
                    continue;
                }
                auto subs = buildSplitDecodeRounds(layer.mesh, layer);
                int subTotal = 0;
                for (const auto& sub : subs) subTotal += sub.numDecimatedTotal;
                if (!subs.empty() && subTotal == layer.numDecimatedTotal) {
                    // buildSplitDecodeRounds returns decode-order subrounds; the
                    // global layers vector is reverse()d again before export, so
                    // store them in reverse to keep decode order after that.
                    for (auto it = subs.rbegin(); it != subs.rend(); ++it) {
                        ordered.push_back(std::move(*it));
                    }
                    ++reorderedCount;
                } else {
                    ordered.push_back(std::move(layer));
                }
            }
            layers = std::move(ordered);
            std::fprintf(stderr, "[RDStrict outer] reordered %d/%zu layers to decode order\n",
                         reorderedCount, layers.size());
        }
                std::reverse(layers.begin(), layers.end());
        splitDecodeRounds_ = layers;
        lastResult_.numVerticesIn = originalIn;
        lastResult_.mesh->i_nbQuantBits = mesh_->i_nbQuantBits;
        mesh_ = lastResult_.mesh;
        return;
    }
    if (std::getenv("PMSF_RDSTRICT_FORWARD_ONLY")) {
        mesh_ = lastResult_.mesh;
        return;
    }
    if (cfg_.strictRDConquest && lastResult_.numDecimatedTotal > 0) {
        auto baseForSerialize = lastResult_.mesh;
        auto baseCopy = cloneMeshForBackwardEmit(baseForSerialize);
        RDPatchPipelineResult reordered =
            rebuildRDPatchResultInStrictDecodeOrder(baseCopy, lastResult_);
        const bool acceptIncomplete = []() {
            const char* v = std::getenv("PMSF_RDSTRICT_ACCEPT_INCOMPLETE_BACKWARD");
            return v && v[0] != '\0' && std::atoi(v) != 0;
        }();
        if (reordered.numDecimatedTotal == lastResult_.numDecimatedTotal || acceptIncomplete) {
            lastResult_ = std::move(reordered);
            lastResult_.mesh = baseForSerialize;
            mesh_ = baseForSerialize;
        } else {
            splitDecodeRounds_ = buildSplitDecodeRounds(lastResult_.mesh, lastResult_);
            if (!splitDecodeRounds_.empty()) {
                std::fprintf(stderr,
                             "[RDStrictSymbolCollect] split backward emit complete subrounds=%zu\n",
                             splitDecodeRounds_.size());
                return;
            }
            std::vector<std::array<int, 3>> allowed = absQsFromResult(reordered);
            const int maxPrunePasses = envIntLocal("PMSF_RDSTRICT_PRUNE_PASSES", 8);
            for (int pass = 0; pass < maxPrunePasses && !allowed.empty(); ++pass) {
                RDPatchPipelineConfig prunedCfg = cfg_;
                prunedCfg.strictUseAllowedRemovalFilter = true;
                prunedCfg.strictAllowedRemovalQs = allowed;
                auto trialMesh = cloneMeshForBackwardEmit(originalMesh);
                RDPatchPipelineResult trial = runRDPatchEncoderOnMesh(trialMesh, prunedCfg);
                if (trial.numDecimatedTotal <= 0) break;
                auto trialBase = trial.mesh;
                RDPatchPipelineResult trialReordered =
                    rebuildRDPatchResultInStrictDecodeOrder(cloneMeshForBackwardEmit(trialBase),
                                                           trial);
                if (trialReordered.numDecimatedTotal == trial.numDecimatedTotal) {
                    trialReordered.mesh = trialBase;
                    lastResult_ = std::move(trialReordered);
                    mesh_ = trialBase;
                    splitDecodeRounds_.clear();
                    std::fprintf(stderr,
                                 "[RDStrictSymbolCollect] pruned strict paper stream "
                                 "complete pass=%d removed=%d\n",
                                 pass + 1,
                                 lastResult_.numDecimatedTotal);
                    return;
                }
                std::vector<std::array<int, 3>> nextAllowed = absQsFromResult(trialReordered);
                std::fprintf(stderr,
                             "[RDStrictSymbolCollect] prune pass %d compatible=%zu/%d\n",
                             pass + 1,
                             nextAllowed.size(),
                             trial.numDecimatedTotal);
                if (nextAllowed.empty() || sameAbsQSet(nextAllowed, allowed)) break;
                allowed = std::move(nextAllowed);
            }
            std::fprintf(stderr,
                         "[RDStrictSymbolCollect] WARNING backward emit incomplete "
                         "(%d/%d); using original forward stream\n",
                         reordered.numDecimatedTotal,
                         lastResult_.numDecimatedTotal);
        }
    }
}

int RDStrictSymbolCollectOperator::exportToBuffer(
    char* /*buffer*/, bool /*enableQuantization*/, bool /*enableCompress*/) {
    throw std::runtime_error("[RDStrictSymbolCollect] use exportToBufferWithRangeCoder");
}

const PMSFSymbolTable* RDStrictSymbolCollectOperator::pmsfSymbolTable() const {
    if (lastResult_.symbols.empty() && lastResult_.numVerticesIn == 0 &&
        lastResult_.numVerticesOut == 0) {
        return nullptr;
    }
    if (paperTableCache_.empty()) {
        paperTableCache_ = pmsfSymbolTableFromRDPatchResult(lastResult_, "rd_strict");
        if (!paperTableCache_.empty()) {
            auto& delta = paperTableCache_.front().delta;
            pmsf::appendRDStrictStorageSideChannel(
                delta,
                static_cast<std::int32_t>(cfg_.strictRDConquest ? 1 : 0),
                static_cast<std::int32_t>(cfg_.usePMSFBoundaryGates ? 1 : 0),
                static_cast<std::int32_t>(cfg_.useFrenetResiduals ? 1 : 0),
                static_cast<std::int32_t>(lastConnectivityBytes_),
                static_cast<std::int32_t>(lastGeometryBytes_),
                static_cast<std::int32_t>(lastSideInfoBytes_),
                static_cast<std::int32_t>(lastContainerOverheadBytes_),
                static_cast<std::int32_t>(splitDecodeRounds_.size()));
        }
    }
    return paperTableCache_.empty() ? nullptr : &paperTableCache_;
}

int RDStrictSymbolCollectOperator::exportToBufferWithRangeCoder(
    char* buffer, bool /*enableQuantization*/) {
    if (!buffer) return 0;
    requireRDStrictPaperExactIfRequested();
    if (!prebuiltBlob_.empty()) {
        std::memcpy(buffer, prebuiltBlob_.data(), prebuiltBlob_.size());
        return static_cast<int>(prebuiltBlob_.size());
    }
    if (lastResult_.symbols.empty()) {
        throw std::runtime_error("[RDStrictSymbolCollect] no symbols collected");
    }

    if (!splitDecodeRounds_.empty()) {
        struct RoundStreams {
            std::vector<std::uint8_t> connectivity;
            std::vector<std::uint8_t> geometry;
            std::vector<std::uint8_t> sideInfo;
        };
        std::vector<RoundStreams> streams;
        streams.reserve(splitDecodeRounds_.size());
        int connBytes = 0;
        int geomBytes = 0;
        int sideBytes = 0;
        for (auto it = splitDecodeRounds_.rbegin(); it != splitDecodeRounds_.rend(); ++it) {
            auto codingPlan = buildCodingPlan(*it);
            RoundStreams rs;
            rs.connectivity = encodeConnectivityStream(it->symbols);
            rs.geometry = buildGeometryStream(*it, codingPlan);
            rs.sideInfo = buildSideInfoStream(*it, codingPlan);
            connBytes += static_cast<int>(rs.connectivity.size());
            geomBytes += static_cast<int>(rs.geometry.size());
            sideBytes += static_cast<int>(rs.sideInfo.size());
            streams.push_back(std::move(rs));
        }

        ConnectivityBlobBuilder builder(
            rdStrictConnectivityBlobSpec(
                static_cast<std::int32_t>(streams.size())));
        for (std::size_t r = 0; r < streams.size(); ++r) {
            builder.appendStreamBytes(static_cast<int>(r), rdstrict::Connectivity,
                                      streams[r].connectivity);
            builder.appendStreamBytes(static_cast<int>(r), rdstrict::Geometry,
                                      streams[r].geometry);
            builder.appendStreamBytes(static_cast<int>(r), rdstrict::SideInfo,
                                      streams[r].sideInfo);
        }
        auto blob = builder.finalize();
        std::memcpy(buffer, blob.data(), blob.size());
        lastConnectivityBytes_ = connBytes;
        lastGeometryBytes_ = geomBytes;
        lastSideInfoBytes_ = sideBytes;
        lastContainerOverheadBytes_ = static_cast<int>(blob.size()) - connBytes - geomBytes - sideBytes;
        return static_cast<int>(blob.size());
    }

    auto codingPlan = buildCodingPlan(lastResult_);
    auto connectivity = encodeConnectivityStream(lastResult_.symbols);
    auto geometry = buildGeometryStream(lastResult_, codingPlan);
    auto sideInfo = buildSideInfoStream(lastResult_, codingPlan);

    ConnectivityBlobBuilder builder(
        rdStrictConnectivityBlobSpec(/*roundCount=*/1));
    builder.appendStreamBytes(0, rdstrict::Connectivity, connectivity);
    builder.appendStreamBytes(0, rdstrict::Geometry, geometry);
    builder.appendStreamBytes(0, rdstrict::SideInfo, sideInfo);

    auto blob = builder.finalize();
    std::memcpy(buffer, blob.data(), blob.size());

    lastConnectivityBytes_ = static_cast<int>(connectivity.size());
    lastGeometryBytes_ = static_cast<int>(geometry.size());
    lastSideInfoBytes_ = static_cast<int>(sideInfo.size());
    lastContainerOverheadBytes_ = static_cast<int>(blob.size())
        - lastConnectivityBytes_ - lastGeometryBytes_ - lastSideInfoBytes_;
    return static_cast<int>(blob.size());
}

}  // namespace pmsf
