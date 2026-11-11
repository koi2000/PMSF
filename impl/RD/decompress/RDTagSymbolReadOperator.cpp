// SPDX-License-Identifier: Apache-2.0
//
// RDTagSymbolReadOperator.cpp — see header for design contract.
//
// 与 PCLTTMTagSymbolReadOperator::importFromRangeCodedBuffer 的差异（最小化）：
//   - 期望 hdr.blob_layout = 18，hdr.num_streams = 3。
//   - hdr.scheme_version = 1：stream 1 解 2-元 isPolyBit RC（Phase 4 PR-B）。
//   - hdr.scheme_version = 2：stream 1 解 RLE+RC 编码的 run lengths（Phase 5 PR-J1）。
//   - hdr.scheme_version = 4：sv=3 adaptive residual + signed LSB truncation。
//   - hdr.scheme_version = 17：global isPoly uses per-round static probabilities。
//   - hdr.scheme_version = 18：global residual uses round-bucket contexts。
//   - **不填** rcDecodedVertexTags_。

#include "scheme/StorageSchemeRegistry.h"
#include "entropy/StreamEntropyCoder.h"
#include "common/RDTagCanonicalCodec.h"  // PHASE61: edgeSym 上下文解码器（与编码端成对）
#include "RD/decompress/RDTagSymbolReadOperator.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂

#include "BFSVersionManager.h"
#include "ContextPool.h"
#include "Halfedge.h"
#include "compress/ConnectivityBlobContainer.h"
#include "compress/ResidualAxisContext.h"
#include "RD/compress/RDSymbolCollectOperator.h"  // RDValenceCode
#include "Point.h"
#include "Vertex.h"

#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <array>
#include <cstring>
#include <iostream>
#include <queue>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

/// PR-J1 (sv=2) 解码：与 encodeIsPolyMultiContextRC 镜像。
///
/// 字节布局（与 sv=1 完全相同）：[int32 numBits N][bytes RC]
/// 区别仅在 RC 内部用 6-bit 历史 64-context 替代 1-context。
///
/// 输出：把 N 个 bit 顺序 push 到 outQueue。
void decodeIsPolyMultiContextRC(const std::span<const uint8_t>& sl,
                                std::deque<uint8_t>& outQueue) {
    constexpr int kHistoryBits = 3;
    constexpr int kNumCtx = 1 << kHistoryBits;  // 8
    constexpr int kHistoryMask = kNumCtx - 1;
    constexpr int lg_totf = 10;
    constexpr int rescale = 1 << 9;

    if (sl.size() < 4) {
        throw std::runtime_error("[RDTag decoder sv=2] isPoly multi-ctx header too small");
    }
    int32_t numBits = 0;
    std::memcpy(&numBits, sl.data(), 4);
    if (numBits == 0) return;

    std::vector<char> rcStaging(sl.begin() + 4, sl.end());
    size_t rcOffset = 0;
    rangecoder rc;
    rc.p_data = rcStaging.data();
    rc.p_dataOffset = &rcOffset;
    if (start_decoding(&rc) != 0) {
        throw std::runtime_error("[RDTag decoder sv=2] isPoly multi-ctx start_decoding failed");
    }

    std::vector<qsmodel> models(kNumCtx);
    for (int i = 0; i < kNumCtx; ++i) {
        initqsmodel(&models[i], 2, lg_totf, rescale, nullptr, 0);
    }

    int history = 0;  // mirror encoder 端初始 = 0
    for (int32_t k = 0; k < numBits; ++k) {
        qsmodel* mp = &models[history & kHistoryMask];
        int syfreq, ltfreq;
        int ltval = decode_culshift(&rc, lg_totf);
        int sym = qsgetsym(mp, ltval);
        qsgetfreq(mp, sym, &syfreq, &ltfreq);
        decode_update(&rc, syfreq, ltfreq, 1 << lg_totf);
        qsupdate(mp, sym);
        outQueue.push_back(sym ? 1 : 0);
        history = ((history << 1) | sym) & kHistoryMask;
    }
    done_decoding(&rc);
    for (auto& m : models) deleteqsmodel(&m);
}

void decodeBitstream2RC(const std::span<const uint8_t>& sl,
                        std::deque<uint8_t>& outQueue,
                        const char* label) {
    constexpr int lg_totf = 10;
    constexpr int rescale = 1 << 9;

    if (sl.size() < 4) {
        throw std::runtime_error(std::string("[RDTag decoder] ") + label + " header too small");
    }
    int32_t numBits = 0;
    std::memcpy(&numBits, sl.data(), 4);
    if (numBits == 0) return;

    // Phase S2（R13/R14）：与编码侧同一采集缝。注入时码流由注入 coder 产生；
    // numBits 仍是符号个数（编码侧照写），故这里照读。默认分支一字不动。
    if (auto injected = ::pmsf::injectedStreamCoder("isPoly")) {
        std::vector<std::uint32_t> syms;
        injected->reset();
        injected->decode(sl.data() + 4, sl.size() - 4,
                         static_cast<std::size_t>(numBits), syms);
        for (std::uint32_t v : syms) outQueue.push_back(static_cast<uint8_t>(v ? 1 : 0));
        return;
    }

    std::vector<char> rcStaging(sl.begin() + 4, sl.end());
    size_t rcOffset = 0;
    rangecoder rc;
    rc.p_data = rcStaging.data();
    rc.p_dataOffset = &rcOffset;
    if (start_decoding(&rc) != 0) {
        throw std::runtime_error(std::string("[RDTag decoder] ") + label + " start_decoding failed");
    }
    qsmodel m;
    initqsmodel(&m, 2, lg_totf, rescale, nullptr, 0);
    for (int32_t k = 0; k < numBits; ++k) {
        int syfreq, ltfreq;
        int ltval = decode_culshift(&rc, lg_totf);
        int sym = qsgetsym(&m, ltval);
        qsgetfreq(&m, sym, &syfreq, &ltfreq);
        decode_update(&rc, syfreq, ltfreq, 1 << lg_totf);
        qsupdate(&m, sym);
        outQueue.push_back(sym ? 1 : 0);
    }
    done_decoding(&rc);
    deleteqsmodel(&m);
}

bool invert3x3(const double m[9], double inv[9]) {
    const double a = m[0], b = m[1], c = m[2];
    const double d = m[3], e = m[4], f = m[5];
    const double g = m[6], h = m[7], i = m[8];
    const double A =  (e * i - f * h);
    const double B = -(d * i - f * g);
    const double C =  (d * h - e * g);
    const double det = a * A + b * B + c * C;
    if (std::fabs(det) < 1e-9) return false;
    const double id = 1.0 / det;
    inv[0] = A * id;
    inv[1] = -(b * i - c * h) * id;
    inv[2] =  (b * f - c * e) * id;
    inv[3] = B * id;
    inv[4] =  (a * i - c * g) * id;
    inv[5] = -(a * f - c * d) * id;
    inv[6] = C * id;
    inv[7] = -(a * h - b * g) * id;
    inv[8] =  (a * e - b * d) * id;
    return true;
}

void decodeResidualBitplaneMajorAdaptive(
    const std::span<const uint8_t>& sl,
    int sv,
    std::deque<MCGAL::PointInt>& outQueue,
    const char* label) {
    // Phase S2（R13/R14）：与编码侧同一采集缝。展平约定见设计文档 §13.2
    // （zigzag + 3×N 符号 + 个数写进流）。默认分支一字不动。
    if (auto injected = ::pmsf::injectedStreamCoder("residual")) {
        if (sl.size() < 4) {
            throw std::runtime_error(std::string("[RDTag decoder ") + label +
                                     "] injected residual header too small");
        }
        int32_t n = 0;
        std::memcpy(&n, sl.data(), 4);
        if (n < 0) {
            throw std::runtime_error(std::string("[RDTag decoder ") + label +
                                     "] injected residual negative count");
        }
        std::vector<std::uint32_t> syms;
        injected->reset();
        injected->decode(sl.data() + 4, sl.size() - 4,
                         static_cast<std::size_t>(n) * 3u, syms);
        auto unzigzag = [](std::uint32_t zz) -> std::int32_t {
            return static_cast<std::int32_t>((zz >> 1) ^ (~(zz & 1u) + 1u));
        };
        for (std::size_t i = 0; i + 2 < syms.size(); i += 3) {
            MCGAL::PointInt p(unzigzag(syms[i]), unzigzag(syms[i + 1]),
                              unzigzag(syms[i + 2]));
            outQueue.push_back(p);
        }
        return;
    }

    if (sl.size() < 8) {
        throw std::runtime_error(std::string("[RDTag decoder ") + label + "] residual header too small");
    }
    int32_t numCoords = 0;
    std::uint32_t rawNumCoords = 0;
    std::memcpy(&rawNumCoords, sl.data(), 4);
    const bool axisCtx = (rawNumCoords & 0x80000000u) != 0;
    const bool kltActive = (rawNumCoords & 0x40000000u) != 0;
    const bool planeOrderActive = (rawNumCoords & 0x20000000u) != 0;
    numCoords = static_cast<std::int32_t>(rawNumCoords & 0x1fffffffu);
    if (numCoords < 0 || (numCoords % 3) != 0) {
        throw std::runtime_error(std::string("[RDTag decoder ") + label + "] invalid residual count");
    }
    const int numResiduals = numCoords / 3;
    const int bits[3] = {
        static_cast<int>(sl[4]),
        static_cast<int>(sl[5]),
        static_cast<int>(sl[6]),
    };
    const int residualDropBits = static_cast<int>(sl[7]);
    if (pmsf::paperBlobSchemeVersionRDTagRequiresZeroAdaptiveResidualDropByte(
            sv) &&
        residualDropBits != 0) {
        throw std::runtime_error("[RDTag decoder sv=3] reserved byte must be 0");
    }
    if (residualDropBits < 0 || residualDropBits > 30) {
        throw std::runtime_error(std::string("[RDTag decoder ") + label + "] invalid residual drop");
    }
    for (int axis = 0; axis < 3; ++axis) {
        if (bits[axis] < 0 || bits[axis] > 31) {
            throw std::runtime_error(std::string("[RDTag decoder ") + label + "] invalid bit width");
        }
    }

    std::uint32_t kltChunk = 0;
    size_t kltUsedBasisCount = 0;
    std::vector<std::array<double, 9>> kltBases;
    std::vector<std::array<double, 9>> kltInverse;
    size_t kltUseFlagsOff = 16;
    bool kltPartitionMode = false;
    std::vector<std::size_t> kltPartitionOfResidual;
    if (kltActive) {
        if (sl.size() < 16) {
            throw std::runtime_error(std::string("[RDTag decoder ") + label +
                                     "] KLT header too small");
        }
        std::uint32_t numChunks = 0;
        std::memcpy(&kltChunk, sl.data() + 8, 4);
        std::memcpy(&numChunks, sl.data() + 12, 4);
        kltPartitionMode = (kltChunk == 0xFFFFFFFFu);
        if ((!kltPartitionMode && kltChunk < 8) || numChunks == 0 ||
            sl.size() < 16 + (size_t)numChunks) {
            throw std::runtime_error(std::string("[RDTag decoder ") + label +
                                     "] invalid KLT chunk table");
        }
        size_t useFlagsOff = 16;
        if (kltPartitionMode) {
            if (sl.size() < 16 + 4 * (size_t)numChunks + numChunks) {
                throw std::runtime_error(std::string("[RDTag decoder ") + label +
                                         "] KLT partition table truncated");
            }
            std::uint64_t sum = 0;
            std::vector<std::uint32_t> lengths(numChunks);
            for (std::uint32_t pi = 0; pi < numChunks; ++pi) {
                std::memcpy(&lengths[pi], sl.data() + 16 + 4 * (size_t)pi, 4);
                sum += lengths[pi];
            }
            if (sum != static_cast<std::uint64_t>(numResiduals)) {
                throw std::runtime_error(std::string("[RDTag decoder ") + label +
                                         "] KLT partition sizes do not match residual count");
            }
            kltPartitionOfResidual.resize(static_cast<size_t>(numResiduals));
            std::size_t cursor = 0;
            for (std::uint32_t pi = 0; pi < numChunks; ++pi) {
                for (std::uint32_t k = 0; k < lengths[pi]; ++k)
                    kltPartitionOfResidual[cursor++] = static_cast<std::size_t>(pi);
            }
            useFlagsOff = 16 + 4 * (size_t)numChunks;
        }
        kltUseFlagsOff = useFlagsOff;
        std::vector<std::uint8_t> useFlags(numChunks);
        std::memcpy(useFlags.data(), sl.data() + useFlagsOff, numChunks);
        size_t basisOff = useFlagsOff + (size_t)numChunks;
        kltBases.resize(numChunks);
        kltInverse.resize(numChunks);
        for (std::uint32_t bi = 0; bi < numChunks; ++bi) {
            const double ident[9] = {1,0,0, 0,1,0, 0,0,1};
            for (int k = 0; k < 9; ++k) kltBases[bi][k] = ident[k];
            if (!useFlags[bi]) continue;
            ++kltUsedBasisCount;
            float basisF[9];
            std::memcpy(basisF, sl.data() + basisOff, sizeof(basisF));
            basisOff += 36;
            for (int k = 0; k < 9; ++k) kltBases[bi][k] = (double)basisF[k];
        }
        for (std::uint32_t bi = 0; bi < numChunks; ++bi) {
            double m[9];
            for (int k = 0; k < 9; ++k) m[k] = kltBases[bi][k];
            double inv[9];
            if (!invert3x3(m, inv)) {
                const double ident[9] = {1,0,0, 0,1,0, 0,0,1};
                for (int k = 0; k < 9; ++k) inv[k] = ident[k];
            }
            for (int k = 0; k < 9; ++k) kltInverse[bi][k] = inv[k];
        }
    }
    std::vector<std::vector<int>> planeOrderAxis(3);
    if (planeOrderActive) {
        const int numPlaneCtx = bits[0] + bits[1] + bits[2];
        const size_t orderOff =
            kltActive ? (kltUseFlagsOff + kltBases.size() + kltUsedBasisCount * 36) : 8;
        if (sl.size() < orderOff + static_cast<size_t>(numPlaneCtx)) {
            throw std::runtime_error(std::string("[RDTag decoder ") + label +
                                     "] bitplane order table truncated");
        }
        size_t orderCursor = orderOff;
        for (int axis = 0; axis < 3; ++axis) {
            planeOrderAxis[axis].resize(static_cast<size_t>(bits[axis]));
            std::vector<int> seenPlane(static_cast<size_t>(bits[axis]), 0);
            for (int k = 0; k < bits[axis]; ++k) {
                const int b = static_cast<int>(sl[orderCursor++]);
                if (b < 0 || b >= bits[axis] || seenPlane[static_cast<size_t>(b)]) {
                    throw std::runtime_error(std::string("[RDTag decoder ") + label +
                                             "] invalid bitplane order entry");
                }
                seenPlane[static_cast<size_t>(b)] = 1;
                planeOrderAxis[axis][static_cast<size_t>(k)] = b;
            }
        }
    }
    const size_t kltHeaderBytes =
        kltActive ? (kltUseFlagsOff + kltBases.size() + kltUsedBasisCount * 36) : 8;
    const size_t residualHeaderBytes =
        kltHeaderBytes +
        (planeOrderActive ? static_cast<size_t>(bits[0] + bits[1] + bits[2]) : 0);
    const int offsets[3] = {0, bits[0], bits[0] + bits[1]};
    const int numCtx = bits[0] + bits[1] + bits[2];
    std::vector<uint32_t> zc[3];
    std::vector<unsigned char> seen[3];
    for (int axis = 0; axis < 3; ++axis) {
        zc[axis].assign(static_cast<size_t>(numResiduals), 0);
        seen[axis].assign(static_cast<size_t>(numResiduals), 0);
    }
    if (numResiduals > 0 && numCtx > 0) {
        std::vector<char> rcStaging(sl.begin() + residualHeaderBytes, sl.end());
        size_t rcOffset = 0;
        rangecoder rc;
        rc.p_data = rcStaging.data();
        rc.p_dataOffset = &rcOffset;
        if (start_decoding(&rc) != 0) {
            throw std::runtime_error(std::string("[RDTag decoder ") + label + "] residual start_decoding failed");
        }
        std::vector<qsmodel> bpFirst1(numCtx), bpAfter1(numCtx);
        for (int i = 0; i < numCtx; ++i) {
            initqsmodel(&bpFirst1[i], 2, 10, 1 << 9, nullptr, 0);
            initqsmodel(&bpAfter1[i], 2, 10, 1 << 9, nullptr, 0);
        }
        std::vector<qsmodel> bpYFirst1, bpYAfter1, bpZFirst1, bpZAfter1;
        std::vector<uint8_t> axisBucketX, axisBucketXY;
        if (axisCtx) {
            const int yModels = bits[1] * pmsf::kResidualAxisBucketCount;
            const int zModels = bits[2] * pmsf::kResidualAxisBucketCount;
            bpYFirst1.resize(yModels); bpYAfter1.resize(yModels);
            bpZFirst1.resize(zModels); bpZAfter1.resize(zModels);
            for (int i = 0; i < yModels; ++i) {
                initqsmodel(&bpYFirst1[i], 2, 10, 1 << 9, nullptr, 0);
                initqsmodel(&bpYAfter1[i], 2, 10, 1 << 9, nullptr, 0);
            }
            for (int i = 0; i < zModels; ++i) {
                initqsmodel(&bpZFirst1[i], 2, 10, 1 << 9, nullptr, 0);
                initqsmodel(&bpZAfter1[i], 2, 10, 1 << 9, nullptr, 0);
            }
            axisBucketX.assign(static_cast<size_t>(numResiduals), 0);
            axisBucketXY.assign(static_cast<size_t>(numResiduals), 0);
        }
        for (int axis = 0; axis < 3; ++axis) {
            if (axisCtx && axis == 1) {
                for (int i = 0; i < numResiduals; ++i) {
                    const std::uint32_t zz = zc[0][static_cast<size_t>(i)];
                    const std::int32_t rx = static_cast<std::int32_t>(
                        (zz >> 1) ^ -(std::int32_t)(zz & 1u));
                    axisBucketX[static_cast<size_t>(i)] =
                        static_cast<uint8_t>(pmsf::residualAxisBucket(rx));
                }
            } else if (axisCtx && axis == 2) {
                for (int i = 0; i < numResiduals; ++i) {
                    const std::uint32_t zx = zc[0][static_cast<size_t>(i)];
                    const std::uint32_t zy = zc[1][static_cast<size_t>(i)];
                    const std::int32_t rx = static_cast<std::int32_t>(
                        (zx >> 1) ^ -(std::int32_t)(zx & 1u));
                    const std::int32_t ry = static_cast<std::int32_t>(
                        (zy >> 1) ^ -(std::int32_t)(zy & 1u));
                    const int bx = pmsf::residualAxisBucket(rx);
                    const int by = pmsf::residualAxisBucket(ry);
                    axisBucketXY[static_cast<size_t>(i)] = static_cast<uint8_t>(
                        bx * pmsf::kResidualAxisBucketCount + by);
                }
            }
            const int width = bits[axis];
            if (width == 0) continue;
            for (int qi = 0; qi < width; ++qi) {
                const int b = planeOrderActive
                                  ? planeOrderAxis[axis][static_cast<size_t>(qi)]
                                  : width - 1 - qi;
                const int mi = offsets[axis] + b;
                for (int i = 0; i < numResiduals; ++i) {
                    qsmodel* mp = nullptr;
                    if (axisCtx && axis == 1) {
                        const int idx = b * pmsf::kResidualAxisBucketCount +
                                        axisBucketX[static_cast<size_t>(i)];
                        mp = seen[axis][static_cast<size_t>(i)]
                                 ? &bpYAfter1[idx] : &bpYFirst1[idx];
                    } else if (axisCtx && axis == 2) {
                        const int idx = b * pmsf::kResidualAxisBucketCount +
                                        axisBucketX[static_cast<size_t>(i)];
                        mp = seen[axis][static_cast<size_t>(i)]
                                 ? &bpZAfter1[idx] : &bpZFirst1[idx];
                    } else {
                        mp = seen[axis][static_cast<size_t>(i)]
                                 ? &bpAfter1[mi] : &bpFirst1[mi];
                    }
                    int ltval = decode_culshift(&rc, 10);
                    int sym = qsgetsym(mp, ltval);
                    int sf, lf;
                    qsgetfreq(mp, sym, &sf, &lf);
                    decode_update(&rc, sf, lf, 1 << 10);
                    qsupdate(mp, sym);
                    if (sym) {
                        seen[axis][static_cast<size_t>(i)] = 1;
                        zc[axis][static_cast<size_t>(i)] |= (1u << b);
                    }
                }
            }
        }
        done_decoding(&rc);
        for (int i = 0; i < numCtx; ++i) {
            deleteqsmodel(&bpFirst1[i]);
            deleteqsmodel(&bpAfter1[i]);
        }
        for (auto& m : bpYFirst1) deleteqsmodel(&m);
        for (auto& m : bpYAfter1) deleteqsmodel(&m);
        for (auto& m : bpZFirst1) deleteqsmodel(&m);
        for (auto& m : bpZAfter1) deleteqsmodel(&m);
    }

    const int32_t scale = residualDropBits > 0 ? (1 << residualDropBits) : 1;
    for (int i = 0; i < numResiduals; ++i) {
        const int32_t rx = static_cast<int32_t>(
            (zc[0][static_cast<size_t>(i)] >> 1) ^
            -(int32_t)(zc[0][static_cast<size_t>(i)] & 1));
        const int32_t ry = static_cast<int32_t>(
            (zc[1][static_cast<size_t>(i)] >> 1) ^
            -(int32_t)(zc[1][static_cast<size_t>(i)] & 1));
        const int32_t rz = static_cast<int32_t>(
            (zc[2][static_cast<size_t>(i)] >> 1) ^
            -(int32_t)(zc[2][static_cast<size_t>(i)] & 1));
        MCGAL::PointInt decoded(
            static_cast<int32_t>(rx * scale),
            static_cast<int32_t>(ry * scale),
            static_cast<int32_t>(rz * scale));
        if (kltActive) {
            const size_t bi = kltPartitionMode
                                  ? kltPartitionOfResidual[static_cast<size_t>(i)]
                                  : std::min<size_t>((size_t)i / (size_t)kltChunk,
                                                     kltBases.size() - 1);
            const auto& inv = kltInverse[bi];
            const double t[3] = {(double)decoded.x(), (double)decoded.y(), (double)decoded.z()};
            decoded = MCGAL::PointInt(
                (int32_t)std::llround(inv[0]*t[0] + inv[1]*t[1] + inv[2]*t[2]),
                (int32_t)std::llround(inv[3]*t[0] + inv[4]*t[1] + inv[5]*t[2]),
                (int32_t)std::llround(inv[6]*t[0] + inv[7]*t[1] + inv[8]*t[2]));
        }
        outQueue.emplace_back(decoded);
    }
}

uint32_t readUVarint32(const std::span<const uint8_t>& sl, size_t& off) {
    uint32_t value = 0;
    int shift = 0;
    while (off < sl.size() && shift <= 28) {
        const uint8_t byte = sl[off++];
        value |= static_cast<uint32_t>(byte & 0x7fu) << shift;
        if ((byte & 0x80u) == 0) return value;
        shift += 7;
    }
    throw std::runtime_error("[RDTag decoder sv=13] malformed round-count varint");
}


}  // namespace

void RDTagSymbolReadOperator::importFromRangeCodedBuffer(
    char* buffer,
    size_t& dataOffset,
    unsigned nbQuantBits) {

    constexpr size_t kArbitraryLargeView = static_cast<size_t>(1) << 32;
    std::span<const uint8_t> view(reinterpret_cast<const uint8_t*>(buffer + dataOffset),
                                   kArbitraryLargeView);

    pmsf::ConnectivityBlobReader rd(view);
    const auto& hdr = rd.header();
    if (!pmsf::paperBlobSchemeVersionIsRDTag(hdr.scheme_version)) {
        throw std::runtime_error(
            "[RDTag decoder] expected scheme_version=1..18 or 19(V6Ctx), got " +
            std::to_string(hdr.scheme_version));
    }
    const bool hasAdjustmentStream =
        pmsf::paperBlobSchemeVersionRDTagHasAdjustmentStream(
            hdr.scheme_version);
    useEdgeSymTopology_ =
        pmsf::paperBlobSchemeVersionRDTagUsesEdgeSymTopology(
            hdr.scheme_version);
    const uint16_t expectedStreams = static_cast<uint16_t>(
        pmsf::paperBlobStreamCountRDTagForScheme(hdr.scheme_version));
    const auto spec = pmsf::rdTagConnectivityBlobSpec(
        static_cast<std::int32_t>(hdr.num_rounds),
        static_cast<std::int32_t>(hdr.scheme_version),
        expectedStreams);
    pmsf::requirePaperConnectivityBlobHeaderMatchesSpec(hdr, spec, "RDTag");

    int numRounds = static_cast<int>(hdr.num_rounds);
    const int sv = static_cast<int>(hdr.scheme_version);
    // PR-FRENET（论文 §3.1 Eq.(3)）：sv=7 反投影局部 Frenet 系残差；
    // sv=8 另加 exact correction 流（反投影后加回 ⇒ 逐量化单位无损）。
    rcUseFrenetResidual_ =
        pmsf::paperBlobSchemeVersionRDTagUsesFrenetResidual(sv);
    rcUseFrenetExactResidual_ =
        pmsf::paperBlobSchemeVersionRDTagUsesFrenetExactResidual(sv);
    rcDecodedRoundCount_ = numRounds;

    rcUsingResidualLocal_ = header_.isEnablePrediction() && header_.isEnableQuantization();
    if (!rcUsingResidualLocal_) {
        throw std::runtime_error(
            "[RDTag decoder] expects enablePrediction && enableQuantization");
    }
    const int BP_BITS = static_cast<int>(nbQuantBits) + 1;

    faceBitsPerRound_.assign(numRounds, 0);

    constexpr int lg_totf = 10;
    constexpr int rescale = 1 << 9;

    // P9 / D1（sv 日落）：原 `if (UsesGlobalStreamMetadata(sv)) { ... }` 整块已删除
    // （该谓词在 V3/V6 下均为 false —— §46 表，已由折叠后的双消费者验证证实）。
    // ⚠️ 该块内的 `UsesLocalSingleContextIsPoly` 在 V3/V6 下**为 true**，
    //    不属本块、未被触及；`UsesEdgeSymTopology`（== V6）同样保留。

    for (int r = 0; r < numRounds; ++r) {
        // ---- stream 0: valence (mirror PCLTTMTag) ----
        {
            auto sl = rd.getStream(r, pmsf::kPaperBlobStreamRDTagValence);
            if (sl.size() < 8) throw std::runtime_error("[RDTag decoder] stream 0 too small");
            int32_t numPoly = 0, numOther = 0;
            std::memcpy(&numPoly,  sl.data(),     4);
            std::memcpy(&numOther, sl.data() + 4, 4);

            std::vector<char> rcStaging(sl.begin() + 8, sl.end());
            size_t rcOffset = 0;
            rangecoder rc;
            rc.p_data = rcStaging.data();
            rc.p_dataOffset = &rcOffset;
            if (start_decoding(&rc) != 0) {
                throw std::runtime_error("[RDTag decoder] valence RC start_decoding failed");
            }

            constexpr int alphabet_size = 8;
            qsmodel ms[4];
            for (int i = 0; i < 4; ++i) {
                initqsmodel(&ms[i], alphabet_size, lg_totf, rescale, nullptr, 0);
            }
            qsmodel esc;
            initqsmodel(&esc, 256, 10, 1 << 9, nullptr, 0);

            auto deriveCtx = [](int prev) {
                if (prev < 0) return 0;
                if (prev == (int)RDValenceCode::VAL_3 || prev == (int)RDValenceCode::VAL_4) return 1;
                if (prev == (int)RDValenceCode::VAL_5 || prev == (int)RDValenceCode::VAL_6) return 2;
                return 3;
            };

            int prev = -1, otherSeen = 0;
            for (int k = 0; k < numPoly; ++k) {
                int ctx = deriveCtx(prev);
                int syfreq, ltfreq;
                int ltval = decode_culshift(&rc, lg_totf);
                int sym = qsgetsym(&ms[ctx], ltval);
                qsgetfreq(&ms[ctx], sym, &syfreq, &ltfreq);
                decode_update(&rc, syfreq, ltfreq, 1 << lg_totf);
                qsupdate(&ms[ctx], sym);
                rcDecodedValCodes_.push_back((uint8_t)sym);

                if (sym == (int)RDValenceCode::VAL_OTHER) {
                    int eltval = decode_culshift(&rc, 10);
                    int esym = qsgetsym(&esc, eltval);
                    int esyfreq, eltfreq;
                    qsgetfreq(&esc, esym, &esyfreq, &eltfreq);
                    decode_update(&rc, esyfreq, eltfreq, 1 << 10);
                    qsupdate(&esc, esym);
                    rcDecodedOtherVals_.push_back((uint8_t)esym);
                    otherSeen++;
                }
                prev = sym;
            }
            done_decoding(&rc);
            for (auto& m : ms) deleteqsmodel(&m);
            deleteqsmodel(&esc);
            (void)otherSeen; (void)numOther;
        }

        // ---- stream 1: isPolyBit (sv=1/3/4/5: 2-元 RC; sv=2: multi-context RC) ----
        // ⚠️ 与 PCLTTMTag sv=3 (4-元 fusedFaceSym) 的差异点：直接 push 到
        //   rcDecodedIsPoly_，**不填** rcDecodedVertexTags_（保持空）。
        if (pmsf::paperBlobSchemeVersionRDTagUsesLocalSingleContextIsPoly(sv)) {
            auto sl = rd.getStream(r, pmsf::kPaperBlobStreamRDTagIsPoly);
            if (sl.size() < 4) throw std::runtime_error("[RDTag decoder sv=1/3/4/5/6/7/8] isPoly too small");
            int32_t numFaceBits = 0;
            std::memcpy(&numFaceBits, sl.data(), 4);
            faceBitsPerRound_[r] = numFaceBits;
            // Phase S2（R13/R14）：与编码侧同一钩子、同一位置（每轮 gate isPoly）。
            // 把方案自己的序列**展开**回默认序列；必须与 emitSymbols 严格互逆。
            {
                auto ss = ::pmsf::activeSymbolScheme();
                if (ss && ss->overridesStream("isPoly")) {
                    const std::vector<std::uint32_t> in(rcDecodedIsPoly_.begin(),
                                                        rcDecodedIsPoly_.end());
                    std::vector<std::uint32_t> out;
                    if (ss->readSymbols("isPoly", in, out)) {
                        rcDecodedIsPoly_.assign(out.begin(), out.end());
                    }
                }
            }
            decodeBitstream2RC(sl, rcDecodedIsPoly_, "isPoly");
            // 不 push rcDecodedVertexTags_：RD 协议无 ⊕/⊖ tag 流。
        } else /* sv == 2 (PR-J1, multi-context RC) */ {
            auto sl = rd.getStream(r, pmsf::kPaperBlobStreamRDTagIsPoly);
            if (sl.size() < 4) throw std::runtime_error("[RDTag decoder sv=2] isPoly too small");
            int32_t numFaceBits = 0;
            std::memcpy(&numFaceBits, sl.data(), 4);
            faceBitsPerRound_[r] = numFaceBits;
            decodeIsPolyMultiContextRC(sl, rcDecodedIsPoly_);
            // 不 push rcDecodedVertexTags_：RD 协议无 ⊕/⊖ tag 流。
        }

        // ---- stream 2: edgeSym for sv=6, otherwise removed-vertex residual ----
        if (pmsf::paperBlobSchemeVersionRDTagUsesEdgeSymTopology(sv)) {
            auto sl = rd.getStream(r, pmsf::kPaperBlobStreamRDTagEdgeSym);
            if (!pmsf::paperBlobSchemeVersionRDTagUsesContextEdgeSym(sv)) {
                // sv=6（老线格式）：单模型编码器。**注入缝的回落语义一并
                // 归到这里** —— 老 V6 无论注入与否都走这条路径。
                decodeBitstream2RC(sl, rcDecodedEdgeSyms_, "edgeSym");
            } else {
                // PHASE61 物品2：与编码端 `encodeEdgeSymWithContext` 严格互逆。
                // 本轮 isPoly = rcDecodedIsPoly_ 的**尾部** faceBitsPerRound_[r] 个
                // （上面刚解完；Phase S2 方案的展开在**追加之前**做，故尾部即本轮
                // 原始位 ⇒ 与编码端作为上下文的序列同源）。
                const std::size_t nFaceThisRound =
                    static_cast<std::size_t>(faceBitsPerRound_[r]);
                const std::size_t tailStart =
                    (rcDecodedIsPoly_.size() >= nFaceThisRound)
                        ? (rcDecodedIsPoly_.size() - nFaceThisRound)
                        : 0;
                pmsf::rd_tag::decodeEdgeSymWithContext(
                    sl, rcDecodedIsPoly_, tailStart, rcDecodedEdgeSyms_);
            }
        }

        // ---- residual stream (sv=6: stream 3; older RD_TAG: stream 2) ----
        {
            const int residualStream =
                pmsf::paperBlobStreamRDTagLocalResidualForScheme(sv);
            auto sl = rd.getStream(r, residualStream);
            if (sl.size() < 4) throw std::runtime_error("[RDTag decoder] residual too small");
            int32_t numCoords = 0;
            std::memcpy(&numCoords, sl.data(), 4);
            if (pmsf::paperBlobSchemeVersionRDTagUsesAdaptiveLocalResidual(sv)) {
                decodeResidualBitplaneMajorAdaptive(sl, sv, rcDecodedResiduals_, "stream2");
            } else {
                std::vector<char> rcStaging(sl.begin() + 4, sl.end());
                size_t rcOffset = 0;
                rangecoder rc;
                rc.p_data = rcStaging.data();
                rc.p_dataOffset = &rcOffset;
                if (start_decoding(&rc) != 0) {
                    throw std::runtime_error("[RDTag decoder] residual start_decoding failed");
                }
                std::vector<qsmodel> bpFirst1(BP_BITS), bpAfter1(BP_BITS);
                for (int b = 0; b < BP_BITS; ++b) {
                    initqsmodel(&bpFirst1[b], 2, 10, 1 << 9, nullptr, 0);
                    initqsmodel(&bpAfter1[b], 2, 10, 1 << 9, nullptr, 0);
                }
                auto decResidualBP = [&]() -> int32_t {
                    uint32_t z = 0;
                    bool seenOne = false;
                    for (int b = BP_BITS - 1; b >= 0; --b) {
                        qsmodel* mp = seenOne ? &bpAfter1[b] : &bpFirst1[b];
                        int syfreq, ltfreq;
                        int ltval = decode_culshift(&rc, 10);
                        int sym = qsgetsym(mp, ltval);
                        qsgetfreq(mp, sym, &syfreq, &ltfreq);
                        decode_update(&rc, syfreq, ltfreq, 1 << 10);
                        qsupdate(mp, sym);
                        z = (z << 1) | (uint32_t)sym;
                        if (sym) seenOne = true;
                    }
                    return (int32_t)((z >> 1) ^ -(int32_t)(z & 1));
                };
                int coordsConsumed = 0;
                while (coordsConsumed < numCoords) {
                    int32_t rx = decResidualBP();
                    int32_t ry = decResidualBP();
                    int32_t rz = decResidualBP();
                    rcDecodedResiduals_.push_back(MCGAL::PointInt(rx, ry, rz));
                    coordsConsumed += 3;
                }
                done_decoding(&rc);
                for (int b = 0; b < BP_BITS; ++b) {
                    deleteqsmodel(&bpFirst1[b]);
                    deleteqsmodel(&bpAfter1[b]);
                }
            }
        }

        // ---- exact Frenet correction (sv=8 stream 3) ----
        // PR-FRENET：反投影后加回 correction ⇒ 残差与 sv=3 逐量化单位一致。
        if (pmsf::paperBlobSchemeVersionRDTagUsesFrenetCorrectionStream(sv)) {
            auto sl = rd.getStream(r, pmsf::kPaperBlobStreamRDTagFrenetCorrection);
            decodeResidualBitplaneMajorAdaptive(sl, sv, rcDecodedFrenetCorrections_, "frenet-correction");
        }

        // ---- QEM survivor adjustment residual (sv=5 stream 3; sv=6 stream 4) ----
        if (pmsf::paperBlobSchemeVersionRDTagHasAdjustmentStream(sv)) {
            const int adjustmentStream =
                pmsf::paperBlobStreamRDTagAdjustmentForScheme(sv);
            auto sl = rd.getStream(r, adjustmentStream);
            if (sl.size() < 4) {
                throw std::runtime_error("[RDTag decoder sv=5] adjustment stream too small");
            }
            uint32_t symBytes = 0;
            std::memcpy(&symBytes, sl.data(), 4);
            if (sl.size() < 4u + symBytes) {
                throw std::runtime_error("[RDTag decoder sv=5] invalid adjustment symbol payload");
            }
            auto symSl = std::span<const uint8_t>(sl.data() + 4, symBytes);
            int32_t numVertexBits = 0;
            if (symSl.size() >= 4) std::memcpy(&numVertexBits, symSl.data(), 4);
            vertexAdjustBitsPerRound_.push_back(numVertexBits);
            decodeBitstream2RC(symSl, rcDecodedVertexAdjustSymbols_, "adjustment symbols");

            auto resSl = std::span<const uint8_t>(
                sl.data() + 4 + symBytes,
                sl.size() - 4 - symBytes);
            decodeResidualBitplaneMajorAdaptive(
                resSl,
                sv,
                rcDecodedVertexAdjustResiduals_,
                "adjustment residual");
        }
    }

    const size_t blobSize = hdr.headerByteSize() +
                            (hdr.stream_offsets.empty() ? 0 : hdr.stream_offsets.back());
    dataOffset += blobSize;

    const char* env = std::getenv("PMSF_BPV_DIAG");
    if (env && env[0] == '1') {
        std::cerr << "[RDTag decoder sv=" << sv << "] rounds=" << numRounds
                  << " totalIsPoly=" << rcDecodedIsPoly_.size()
                  << " totalValCodes=" << rcDecodedValCodes_.size()
                  << " totalResiduals=" << rcDecodedResiduals_.size()
                  << " totalAdjustSymbols=" << rcDecodedVertexAdjustSymbols_.size()
                  << " totalAdjustResiduals=" << rcDecodedVertexAdjustResiduals_.size()
                  << " blobSize=" << blobSize << std::endl;
    }
}

void RDTagSymbolReadOperator::collectFromDecoded(
    MCGAL::Halfedge* seed,
    std::vector<MCGAL::Vertex*>& vertices,
    std::vector<MCGAL::Halfedge*>& halfedge,
    std::vector<MCGAL::Facet*>& facets) {

    const int roundIdx = currentRoundIdx_;
    const bool hasAdjustmentData =
        !vertexAdjustBitsPerRound_.empty() ||
        !rcDecodedVertexAdjustSymbols_.empty() ||
        !rcDecodedVertexAdjustResiduals_.empty();

    if (useEdgeSymTopology_) {
        RDGateOnlySymbolReadOperator::collectFromDecoded(seed, vertices, halfedge, facets);
    } else {
        PCLTTMTagSymbolReadOperator::collectFromDecoded(seed, vertices, halfedge, facets);
    }

    if (!header_.isEnableAdjustment() || !hasAdjustmentData || seed == nullptr) {
        return;
    }

    const int expectedVertexBits =
        (roundIdx >= 0 && roundIdx < (int)vertexAdjustBitsPerRound_.size())
            ? vertexAdjustBitsPerRound_[roundIdx]
            : -1;
    int consumedVertexBits = 0;

    // perf（2026-11-10）：该 BFS 每轮遍历全网格；std::queue 的 deque 逐元素分配
    // 是热点。改为预分配 vector + head 游标的 FIFO（顺序完全一致）。
    std::vector<int> q;
    q.reserve(4096);
    q.push_back(seed->poolId());
    std::size_t qHead = 0;
    const int currentVersion = MCGAL::BFSVersionManager::current_version++;

    while (qHead < q.size()) {
        const int hid = q[qHead++];
        MCGAL::Halfedge* h =
            MCGAL::contextPool.getHalfedgeByIndexInSubPool(DEFAULT_MESH_ID, hid);
        if (h == nullptr || h->isVisited(currentVersion) || h->isRemoved()) continue;
        h->setVisited(currentVersion);

        MCGAL::Vertex* v = h->vertex();
        if (v != nullptr && !v->isVisited(currentVersion)) {
            v->setVisited(currentVersion);
            if (rcDecodedVertexAdjustSymbols_.empty()) {
                throw std::runtime_error("[RDTag decoder sv=5] adjustment symbol underflow");
            }
            const uint8_t symbol = rcDecodedVertexAdjustSymbols_.front();
            rcDecodedVertexAdjustSymbols_.pop_front();
            consumedVertexBits++;

            if (symbol) {
                if (rcDecodedVertexAdjustResiduals_.empty()) {
                    throw std::runtime_error("[RDTag decoder sv=5] adjustment residual underflow");
                }
                const MCGAL::PointInt residual = rcDecodedVertexAdjustResiduals_.front();
                rcDecodedVertexAdjustResiduals_.pop_front();

                const MCGAL::PointInt pred = getQuantizedPos(v->point());
                const MCGAL::PointInt target(
                    pred.x() + residual.x(),
                    pred.y() + residual.y(),
                    pred.z() + residual.z());
                const float step = f_quantStep_ * static_cast<float>(1 << i_curQuantizationId);
                const MCGAL::Point oldPoint(
                    bboxMin_.x() + (target.x() + 0.5f) * step,
                    bboxMin_.y() + (target.y() + 0.5f) * step,
                    bboxMin_.z() + (target.z() + 0.5f) * step);

                v->setAdjusted();
                v->setVertexPoint(oldPoint);
                vertices.push_back(v);
            }
        }

        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp != nullptr && !hOpp->isVisited(currentVersion)) {
                q.push_back(hOpp->poolId());
            }
            if (hIt != nullptr && !hIt->isVisited(currentVersion)) {
                q.push_back(hIt->poolId());
            }
            hIt = hIt->next();
        } while (hIt != h && hIt != nullptr);
    }

    if (expectedVertexBits >= 0 && consumedVertexBits != expectedVertexBits) {
        std::cerr << "[RDTag decoder sv=5] WARN round=" << roundIdx
                  << " adjustedVertexBfs=" << consumedVertexBits
                  << " expected=" << expectedVertexBits << std::endl;
    }
}
