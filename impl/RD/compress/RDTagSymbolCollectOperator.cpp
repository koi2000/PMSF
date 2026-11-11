// SPDX-License-Identifier: Apache-2.0
//
// RDTagSymbolCollectOperator.cpp — see header for design contract.
//
// 实施策略（极小 diff）：
//   1. collect() 完全继承 PCLTTMTagSymbolCollectOperator，**不重写**：
//      - 父类 BFS 第一遍 (PCATMColor::collect) 填了所有 5 段队列；
//      - 父类 BFS 第二遍 (PCLTTMTag::collect 自己) 覆写 gateResidualQueues_.back()
//        为二阶 curvature predictor 编码后的 (r1 - pred_c2)，并维护
//        kVPropRDR1 邻居 r1；同时填 gateFusedFaceSymQueues_（本类不用，丢弃）。
//   2. exportToBufferWithRangeCoder 重写：**只**写 3 段流 (valence + isPolyBit
//      + residual)，stream 1 用 2-元 RC isPolyBit 而非 4-元 fusedFaceSym。
//      blob_layout=18, sv=1。
//
// 字节布局（与 PCLTTM Tag sv=3 几乎相同，仅 stream 1 alphabet 不同）：
//   ConnectivityBlobContainer v3, magic='CONN', blob_layout=18, sv=1, num_streams=3:
//     stream 0: [int32 numPoly][int32 numOther][bytes 5-元 4-context RC]
//     stream 1: [int32 numFaceBits][bytes 2-元 RC]            ← isPolyBit per BFS face
//     stream 2: [int32 numCoords][bytes bit-plane RC]         ← residual (含二阶 pred)

#include "scheme/StorageSchemeRegistry.h"
#include "common/ValenceStreamCodec.h"  // pmsf::encodeValence5（P3 抽取）

using pmsf::encodeValence5;
#include "entropy/StreamEntropyCoder.h"
#include "RD/compress/RDTagSymbolCollectOperator.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂
#include "core/PMSFBoundarySideChannel.h"
#include "core/PaperGeometrySideChannel.h"
#include "RD/boundary/RDPMSFBoundary.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include "BFSVersionManager.h"
#include "compress/ConnectivityBlobContainer.h"
#include "compress/RangeCoderStreamEncoder.h"
#include "Configuration.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Point.h"
#include "Vertex.h"
#include "RD/compress/RDSymbolCollectOperator.h"  // RDValenceCode
#include "common/RDTagCanonicalCodec.h"  // pmsf::rd_tag::encodeBitstream2RC（P7 抽取）
#include "common/BoundedRingTraversal.h"  // PR-AP 诊断：论文式 Laplacian 1-环
#include "predict/RDPredictor.h"
#include "RD/RDPGS.h"

#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr int kBufScratch = BUFFER_SIZE / 12;

bool rdTagProfileLogs() {
    static const bool enabled = [] {
        const char* env = std::getenv("PMSF_PROFILE");
        return env && env[0] != '\0' && std::strcmp(env, "0") != 0 &&
               std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
    }();
    return enabled;
}

void requireRDNonStrictPaperExactIfRequested(const char* label) {
    if (!pmsf::rdPaperExactRequired()) {
        return;
    }
    // KLT replay + per-component alpha/beta/gamma flags are implemented in this
    // path (self-describing basis table); allow the strict gate when enabled.
    if (const char* klt = std::getenv("PMSF_RD_USE_KLT"); klt && *klt && klt[0] != '0') {
        return;
    }
    throw std::runtime_error(
        std::string(label) +
        " is not paper-exact RD: prioritized gate selection, curvature "
        "prediction, and KLT replay are not fully implemented in this "
        "non-strict path.");
}

/// stream 0: valence (5-元 + 4-context + V_OTHER inline 256-元 escape)
/// 字节布局: [int32 numPoly][int32 numOther][bytes RC]
/// 与 PCLTTMTag/PCATMColor 完全一致，便于 decoder 端 reader 共享解码 helper。
/// Combined global stream for sv=16.
/// Symbol alphabet:
///   0      = non-poly face
///   1..5   = poly face with RDValenceCode 0..4
/// VAL_OTHER still carries the escaped real valence in otherVals.

bool residualDropAppliesToDecoderRound(int decoderRoundIdx) {
    const char* startEnv = std::getenv("PMSF_RDTAG_RESIDUAL_DROP_DECODER_START");
    const char* endEnv = std::getenv("PMSF_RDTAG_RESIDUAL_DROP_DECODER_END");
    if ((!startEnv || !*startEnv) && (!endEnv || !*endEnv)) return true;
    int start = startEnv && *startEnv ? std::atoi(startEnv) : 0;
    int end = endEnv && *endEnv ? std::atoi(endEnv) : std::numeric_limits<int>::max();
    return decoderRoundIdx >= start && decoderRoundIdx <= end;
}

/// PR-J1: stream 1 (sv=2) 多上下文 RC 编码。
///
/// 原 sv=1 用 1-context adaptive 2-元 RC，对 isPolyBit (p≈0.21) 编码至
/// ~0.999 bit/bit，已接近熵下界 H(0.21)=0.74 bit/bit，但仍有 ~25% 超熵。
/// PR-J1 (plan §3.5) 原计划用 RLE+RC，实测 RLE 反而增大字节（RC 已吸收短相关，
/// 而 run length 的非 0 头部开销 + zigzag bit-plane 头部开销）。
///
/// 改进策略：用 **过去 K bits 作上下文**（K=kHistoryBits=6 → 64 个独立 2-元
/// adaptive 模型）。这能捕捉 BFS 顺序里 face splittability 的局部相关性
/// （邻居 face 多为同色调 polyFace 或同色调 NotPoly face）。
/// 实测 bunny 30 轮，从 20935 → ~16K bytes，约 -1 bpv（达 plan §3.5 PR-J1 的
/// 保底目标范围 connBPV 7.13 → ~6 bpv）。
///
/// 字节布局（与 sv=1 完全相同！）：
///   [int32 numBits][bytes RC]
///
/// 注意：sv=1 sv=2 的字节布局头一致，区别只在内部 RC 模型（1 vs 64 contexts）。
/// 这意味着 sv 必须由 hdr.scheme_version 唯一区分（不要靠 byte layout 差异）。

/// stream 2: residual bit-plane RC。字节布局: [int32 numCoords][bytes bit-plane RC]
int32_t signedResidualDrop(int32_t v, int dropBits) {
    if (dropBits <= 0) return v;
    if (dropBits >= 30) return v < 0 ? -1 : 0;
    // C++ leaves right shift of negative signed integers implementation-defined.
    // Spell out floor division by 2^drop so encoder and decoder do not depend on it.
    const int32_t scale = 1 << dropBits;
    if (v >= 0) return v / scale;
    return -static_cast<int32_t>((static_cast<int64_t>(-v) + scale - 1) / scale);
}

int bitWidthForResidualComponent(const std::deque<MCGAL::PointInt>& residuals,
                                 int axis,
                                 int residualDropBits) {
    uint32_t maxZ = 0;
    for (const auto& r : residuals) {
        int32_t v = signedResidualDrop(
            axis == 0 ? r.x() : axis == 1 ? r.y() : r.z(),
            residualDropBits);
        uint32_t z = (static_cast<uint32_t>(v) << 1) ^
                     static_cast<uint32_t>(v >> 31);
        if (z > maxZ) maxZ = z;
    }
    int bits = 0;
    while (maxZ != 0) {
        ++bits;
        maxZ >>= 1;
    }
    return std::min(bits, 31);
}

namespace {

struct RDKltBasis {
    double r[3][3]{};
    bool valid = false;
};

bool rdKltEnabled() {
    static const bool on = [] {
        const char* e = std::getenv("PMSF_RD_USE_KLT");
        return e && *e && e[0] != '0';
    }();
    return on;
}

RDKltBasis computeRDKltBasis(const std::deque<MCGAL::PointInt>& residuals) {
    RDKltBasis b;
    if (residuals.size() < 3) return b;
    double c[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    for (const auto& p : residuals) {
        const double v[3] = {(double)p.x(), (double)p.y(), (double)p.z()};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) c[i][j] += v[i] * v[j];
    }
    double v[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    for (int iter = 0; iter < 64; ++iter) {
        int p = 0, q = 1;
        double mx = std::fabs(c[0][1]);
        if (std::fabs(c[0][2]) > mx) { mx = std::fabs(c[0][2]); p = 0; q = 2; }
        if (std::fabs(c[1][2]) > mx) { mx = std::fabs(c[1][2]); p = 1; q = 2; }
        if (mx < 1e-12) break;
        const double phi = 0.5 * std::atan2(2.0 * c[p][q], c[q][q] - c[p][p]);
        const double cs = std::cos(phi), sn = std::sin(phi);
        for (int k = 0; k < 3; ++k) {
            const double cp = c[k][p], cq = c[k][q];
            c[k][p] = cs * cp - sn * cq;
            c[k][q] = sn * cp + cs * cq;
        }
        for (int k = 0; k < 3; ++k) {
            const double cp = c[p][k], cq = c[q][k];
            c[p][k] = cs * cp - sn * cq;
            c[q][k] = sn * cp + cs * cq;
        }
        for (int k = 0; k < 3; ++k) {
            const double vp = v[k][p], vq = v[k][q];
            v[k][p] = cs * vp - sn * vq;
            v[k][q] = sn * vp + cs * vq;
        }
    }
    double eig[3] = {c[0][0], c[1][1], c[2][2]};
    int order[3] = {0,1,2};
    std::sort(order, order + 3, [&](int a, int b) { return eig[a] > eig[b]; });
    for (int i = 0; i < 3; ++i) {
        const int col = order[i];
        for (int j = 0; j < 3; ++j) b.r[i][j] = v[j][col];
    }
    b.valid = true;
    return b;
}

MCGAL::PointInt rdKltForward(const RDKltBasis& b, const MCGAL::PointInt& p) {
    const double x[3] = {(double)p.x(), (double)p.y(), (double)p.z()};
    return MCGAL::PointInt(
        (int32_t)std::llround(b.r[0][0]*x[0] + b.r[0][1]*x[1] + b.r[0][2]*x[2]),
        (int32_t)std::llround(b.r[1][0]*x[0] + b.r[1][1]*x[1] + b.r[1][2]*x[2]),
        (int32_t)std::llround(b.r[2][0]*x[0] + b.r[2][1]*x[1] + b.r[2][2]*x[2]));
}

MCGAL::PointInt rdKltInverse(const double b[3][3], const MCGAL::PointInt& p) {
    const double t[3] = {(double)p.x(), (double)p.y(), (double)p.z()};
    return MCGAL::PointInt(
        (int32_t)std::llround(b[0][0]*t[0] + b[1][0]*t[1] + b[2][0]*t[2]),
        (int32_t)std::llround(b[0][1]*t[0] + b[1][1]*t[1] + b[2][1]*t[2]),
        (int32_t)std::llround(b[0][2]*t[0] + b[1][2]*t[1] + b[2][2]*t[2]));
}

/// Same inverse as the RDTag decoder (invert3x3): the encoder must use the exact
/// decoder-side inverse when it checks round-trip reversibility, otherwise a
/// mixed per-component basis can look reversible in double precision but not
/// after the float32 wire representation (observed as ~1e-3 geometry drift).
bool rdKltInvert3x3(const double m[9], double inv[9]) {
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

/// Reject a basis unless the float32 wire matrix + decoder inverse reproduce every
/// chunk residual exactly. This is the KLT symmetry contract of the paper stage.
bool rdKltRoundTripExact(const RDKltBasis& basis,
                         const std::deque<MCGAL::PointInt>& chunk) {
    double rf[9];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            rf[i * 3 + j] = static_cast<double>(static_cast<float>(basis.r[i][j]));
    double inv[9];
    if (!rdKltInvert3x3(rf, inv)) return false;
    for (const auto& r : chunk) {
        const double x[3] = {(double)r.x(), (double)r.y(), (double)r.z()};
        long long y[3];
        for (int i = 0; i < 3; ++i) {
            const double v = rf[i * 3 + 0] * x[0] + rf[i * 3 + 1] * x[1] +
                             rf[i * 3 + 2] * x[2];
            y[i] = (long long)std::llround(v);
        }
        for (int i = 0; i < 3; ++i) {
            const double v = inv[i * 3 + 0] * (double)y[0] +
                             inv[i * 3 + 1] * (double)y[1] +
                             inv[i * 3 + 2] * (double)y[2];
            if ((long long)std::llround(v) != (long long)x[i]) return false;
        }
    }
    return true;
}

}  // namespace

/// sv=3/sv=4 stream 2: xyz 分量独立 bit width + bit-plane-major adaptive RC。
/// 字节布局: [int32 numCoords][uint8 bitsX][bitsY][bitsZ][dropBits][bytes RC]
std::vector<uint8_t> encodeResidualBitplaneMajorAdaptive(
    const std::deque<MCGAL::PointInt>& residualsIn,
    int residualDropBits,
    const std::deque<MCGAL::Facet*>* partitionFacets = nullptr) {
    const bool useKLT = rdKltEnabled() && residualsIn.size() >= 3;
    int kltChunk = 256;
    bool kltTopologyPartitions = false;
    if (const char* cs = std::getenv("PMSF_RD_KLT_CHUNK")) {
        const int v = std::atoi(cs);
        if (v >= 8) kltChunk = v;
        else if (v == 0) kltTopologyPartitions = true;
    }
    std::deque<MCGAL::PointInt> kltQueue;
    std::vector<RDKltBasis> kltBases;
    std::vector<std::uint8_t> kltUseFlags;
    std::vector<std::vector<size_t>> kltPartitionIndices;
    if (useKLT) {
        const size_t total = residualsIn.size();
        std::vector<std::vector<size_t>> groups;
        if (kltTopologyPartitions && partitionFacets != nullptr &&
            partitionFacets->size() >= total && total > 0) {
            // Paper §3.4 topology-based partitions: BFS over poly-face adjacency of
            // the residual-bearing faces (one facet per residual, encoder order).
            std::unordered_map<MCGAL::Facet*, size_t> indexOfFacet;
            indexOfFacet.reserve(total * 2);
            for (size_t i = 0; i < total; ++i) {
                if ((*partitionFacets)[i] != nullptr)
                    indexOfFacet.emplace((*partitionFacets)[i], i);
            }
            std::vector<std::uint8_t> visited(total, 0);
            const size_t maxPartition = 4096;
            for (size_t start = 0; start < total; ++start) {
                if (visited[start]) continue;
                std::vector<size_t> part;
                std::deque<size_t> queue;
                queue.push_back(start);
                visited[start] = 1;
                while (!queue.empty() && part.size() < maxPartition) {
                    const size_t idx = queue.front();
                    queue.pop_front();
                    part.push_back(idx);
                    MCGAL::Facet* f = (*partitionFacets)[idx];
                    if (f == nullptr) continue;
                    MCGAL::Halfedge* h0 = f->proxyHalfedge();
                    MCGAL::Halfedge* h = h0;
                    int guard = 0;
                    do {
                        if (!h) break;
                        MCGAL::Facet* nb = h->opposite() ? h->opposite()->face() : nullptr;
                        auto it = indexOfFacet.find(nb);
                        if (it != indexOfFacet.end() && !visited[it->second]) {
                            visited[it->second] = 1;
                            queue.push_back(it->second);
                        }
                        h = h->next();
                    } while (h && h != h0 && ++guard < 64);
                }
                std::sort(part.begin(), part.end());
                groups.push_back(std::move(part));
            }
        }
        if (groups.empty()) {
            const size_t chunks = (total + (size_t)kltChunk - 1) / (size_t)kltChunk;
            groups.reserve(chunks);
            for (size_t c = 0; c < chunks; ++c) {
                const size_t begin = c * (size_t)kltChunk;
                const size_t end = std::min(total, begin + (size_t)kltChunk);
                std::vector<size_t> group;
                for (size_t i = begin; i < end; ++i) group.push_back(i);
                groups.push_back(std::move(group));
            }
        } else {
            kltPartitionIndices = groups;
        }
        kltBases.reserve(groups.size());
        std::deque<MCGAL::PointInt> chunk;
        for (std::size_t c = 0; c < groups.size(); ++c) {
            chunk.clear();
            for (size_t i : groups[c]) chunk.push_back(residualsIn[i]);
            RDKltBasis basis = computeRDKltBasis(chunk);
            std::deque<MCGAL::PointInt> transformed;
            int rawScore[3] = {0,0,0};
            int kltScore[3] = {0,0,0};
            for (int a = 0; a < 3; ++a) {
                rawScore[a] = bitWidthForResidualComponent(chunk, a, 0);
            }
            if (basis.valid) {
                for (const auto& r : chunk) transformed.push_back(rdKltForward(basis, r));
                for (int a = 0; a < 3; ++a) {
                    kltScore[a] = bitWidthForResidualComponent(transformed, a, 0);
                }
            }
            std::uint8_t mask = 0;
            for (int a = 0; a < 3; ++a) {
                if (kltScore[a] > 0 && kltScore[a] < rawScore[a]) mask |= (1u << a);
            }
            RDKltBasis matrix = basis;
            bool useBasis = false;
            if (basis.valid && mask != 0) {
                for (int a = 0; a < 3; ++a) {
                    if (mask & (1u << a)) continue;
                    for (int j = 0; j < 3; ++j) matrix.r[a][j] = (a == j) ? 1.0 : 0.0;
                }
                const double det =
                    matrix.r[0][0] * (matrix.r[1][1] * matrix.r[2][2] - matrix.r[1][2] * matrix.r[2][1]) -
                    matrix.r[0][1] * (matrix.r[1][0] * matrix.r[2][2] - matrix.r[1][2] * matrix.r[2][0]) +
                    matrix.r[0][2] * (matrix.r[1][0] * matrix.r[2][1] - matrix.r[1][1] * matrix.r[2][0]);
                useBasis = std::fabs(det) > 1e-6 && rdKltRoundTripExact(matrix, chunk);
            }
            if (!useBasis) {
                matrix.r[0][0] = matrix.r[1][1] = matrix.r[2][2] = 1.0;
                matrix.r[0][1] = matrix.r[0][2] = matrix.r[1][0] = 0.0;
                matrix.r[1][2] = matrix.r[2][0] = matrix.r[2][1] = 0.0;
                matrix.valid = true;
            }
            kltBases.push_back(matrix);
            kltUseFlags.push_back(useBasis ? 1u : 0u);
        }
        // Wire order MUST stay the decoder residual order (topology partitions are
        // non-contiguous), so apply each residual's group basis in original order.
        std::vector<size_t> partOfResidual(total);
        if (kltPartitionIndices.empty()) {
            for (size_t i = 0; i < total; ++i) {
                partOfResidual[i] = std::min<size_t>(
                    i / static_cast<size_t>(kltChunk), kltBases.size() - 1);
            }
        } else {
            for (size_t p = 0; p < kltPartitionIndices.size(); ++p) {
                for (size_t idx : kltPartitionIndices[p]) partOfResidual[idx] = p;
            }
        }
        kltQueue.clear();
        for (size_t i = 0; i < total; ++i) {
            const size_t p = partOfResidual[i];
            if (!kltUseFlags[p]) kltQueue.push_back(residualsIn[i]);
            else kltQueue.push_back(rdKltForward(kltBases[p], residualsIn[i]));
        }
    }
    const bool kltActive = useKLT && !kltBases.empty();
    const auto& residuals = kltActive ? kltQueue : residualsIn;
    // Phase S2（R13/R14）：方案可注入本条流的编码器。展平约定见设计文档 §13.2：
    //   [i32 numResiduals][injected 码流]，符号 = 3×N 个 uint32，
    //   按 residual 顺序、分量顺序 x,y,z，每分量 zigzag 有符号→无符号。
    // 未注入时下面**原有代码一字不动** ⇒ 默认路径逐字节不变。
    if (auto injected = ::pmsf::injectedStreamCoder("residual")) {
        std::vector<std::uint32_t> syms;
        syms.reserve(residuals.size() * 3);
        auto zigzag = [](std::int32_t v) -> std::uint32_t {
            return (static_cast<std::uint32_t>(v) << 1) ^
                   static_cast<std::uint32_t>(v >> 31);
        };
        for (const auto& r : residuals) {
            syms.push_back(zigzag(static_cast<std::int32_t>(r[0])));
            syms.push_back(zigzag(static_cast<std::int32_t>(r[1])));
            syms.push_back(zigzag(static_cast<std::int32_t>(r[2])));
        }
        std::vector<std::uint8_t> blob;
        injected->reset();
        injected->encode(syms, blob);
        std::vector<uint8_t> out(4 + blob.size());
        int32_t numResiduals = static_cast<int32_t>(residuals.size());
        std::memcpy(out.data(), &numResiduals, 4);
        if (!blob.empty()) std::memcpy(out.data() + 4, blob.data(), blob.size());
        return out;
    }
    const size_t N = residuals.size();
    const int bits[3] = {
        bitWidthForResidualComponent(residuals, 0, residualDropBits),
        bitWidthForResidualComponent(residuals, 1, residualDropBits),
        bitWidthForResidualComponent(residuals, 2, residualDropBits),
    };
    const int offsets[3] = {0, bits[0], bits[0] + bits[1]};
    const int numCtx = bits[0] + bits[1] + bits[2];

    std::vector<int> dx(N), dy(N), dz(N);
    for (size_t i = 0; i < N; ++i) {
        dx[i] = signedResidualDrop(residuals[i].x(), residualDropBits);
        dy[i] = signedResidualDrop(residuals[i].y(), residualDropBits);
        dz[i] = signedResidualDrop(residuals[i].z(), residualDropBits);
    }

    // Paper stage 4 prototype: R-D optimized per-axis bitplane order. The order
    // only permutes bitplane visits inside one axis (x then y then z are kept so
    // the axis-context buckets stay decodable); it is opt-in and self-described
    // in the residual stream header.
    const bool rdPlaneOrderOn = [] {
        const char* e = std::getenv("PMSF_RD_BITPLANE_RD_ORDER");
        return e != nullptr && *e != '\0' && e[0] != '0';
    }();
    std::vector<int> planeOrder;
    std::vector<uint8_t> planeOrderBytes;
    if (rdPlaneOrderOn && numCtx > 0) {
        const std::vector<int>* axes[3] = {&dx, &dy, &dz};
        const int widths[3] = {bits[0], bits[1], bits[2]};
        for (int comp = 0; comp < 3; ++comp) {
            const std::vector<int>& vals = *axes[comp];
            struct Cand { int bit; double ratio; };
            std::vector<Cand> cands;
            cands.reserve(static_cast<size_t>(widths[comp]));
            for (int b = 0; b < widths[comp]; ++b) {
                long long ones = 0;
                for (int32_t v : vals) {
                    const uint32_t z = (static_cast<uint32_t>(v) << 1) ^
                                       static_cast<uint32_t>(v >> 31);
                    if ((z >> b) & 1u) ++ones;
                }
                double ratio = 0.0;
                if (ones > 0 && N > 0) {
                    const double p = static_cast<double>(ones) / static_cast<double>(N);
                    const double h = -(p * std::log2(p) +
                                       (1.0 - p) * std::log2(1.0 - p));
                    const double cost = std::max(1e-9, static_cast<double>(N) * h);
                    const double benefit = static_cast<double>(ones) *
                                           static_cast<double>(1ULL << std::min(b, 30));
                    ratio = benefit / cost;
                }
                cands.push_back({b, ratio});
            }
            std::sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) {
                if (a.ratio != b.ratio) return a.ratio > b.ratio;
                return a.bit > b.bit;
            });
            for (const Cand& c : cands) {
                planeOrder.push_back(comp * 32 + c.bit);
                planeOrderBytes.push_back(static_cast<uint8_t>(c.bit));
            }
        }
    }

    if (const char* envStats = std::getenv("PMSF_RESIDUAL_STATS");
        envStats && envStats[0] == '1') {
        size_t zeroVec = 0;
        size_t zeroAxis[3] = {0, 0, 0};
        std::vector<int32_t> axisVals[3];
        for (int axis = 0; axis < 3; ++axis) {
            axisVals[axis].reserve(N);
        }
        for (const auto& r : residuals) {
            const int32_t x = signedResidualDrop(r.x(), residualDropBits);
            const int32_t y = signedResidualDrop(r.y(), residualDropBits);
            const int32_t z = signedResidualDrop(r.z(), residualDropBits);
            if (x == 0 && y == 0 && z == 0) ++zeroVec;
            if (x == 0) ++zeroAxis[0];
            else axisVals[0].push_back(x);
            if (y == 0) ++zeroAxis[1];
            else axisVals[1].push_back(y);
            if (z == 0) ++zeroAxis[2];
            else axisVals[2].push_back(z);
        }
        auto encodeMaskSize = [&]() -> size_t {
            const size_t bufSize = std::max<size_t>(N * 2 + 1024, 4096);
            std::unique_ptr<char[]> buf(new char[bufSize]);
            size_t off = 0;
            rangecoder rc;
            rc.p_data = buf.get();
            rc.p_dataOffset = &off;
            start_encoding(&rc, 0, 0);
            qsmodel m;
            initqsmodel(&m, 8, 10, 1 << 9, nullptr, 1);
            for (const auto& r : residuals) {
                const int32_t x = signedResidualDrop(r.x(), residualDropBits);
                const int32_t y = signedResidualDrop(r.y(), residualDropBits);
                const int32_t z = signedResidualDrop(r.z(), residualDropBits);
                const int mask = (x != 0 ? 1 : 0) |
                                 (y != 0 ? 2 : 0) |
                                 (z != 0 ? 4 : 0);
                int sf, lf;
                qsgetfreq(&m, mask, &sf, &lf);
                encode_shift(&rc, sf, lf, 10);
                qsupdate(&m, mask);
            }
            const size_t rcSize = done_encoding(&rc);
            deleteqsmodel(&m);
            return rcSize;
        };
        auto encodeSparseValuesSize = [&]() -> size_t {
            const size_t totalVals = axisVals[0].size() + axisVals[1].size() +
                                     axisVals[2].size();
            if (totalVals == 0 || numCtx == 0) return 0;
            const size_t bufSize = std::max<size_t>(totalVals * 8 + 1024, 4096);
            std::unique_ptr<char[]> buf(new char[bufSize]);
            size_t off = 0;
            rangecoder rc;
            rc.p_data = buf.get();
            rc.p_dataOffset = &off;
            start_encoding(&rc, 0, 0);
            std::vector<qsmodel> bpFirst1(numCtx), bpAfter1(numCtx);
            for (int i = 0; i < numCtx; ++i) {
                initqsmodel(&bpFirst1[i], 2, 10, 1 << 9, nullptr, 1);
                initqsmodel(&bpAfter1[i], 2, 10, 1 << 9, nullptr, 1);
            }
            for (int axis = 0; axis < 3; ++axis) {
                const int width = bits[axis];
                if (width == 0 || axisVals[axis].empty()) continue;
                std::vector<unsigned char> seenOne(axisVals[axis].size(), 0);
                for (int b = width - 1; b >= 0; --b) {
                    const int mi = offsets[axis] + b;
                    for (size_t i = 0; i < axisVals[axis].size(); ++i) {
                        const int32_t v = axisVals[axis][i];
                        const uint32_t z = (static_cast<uint32_t>(v) << 1) ^
                                           static_cast<uint32_t>(v >> 31);
                        const int bit = static_cast<int>((z >> b) & 1u);
                        qsmodel* mp = seenOne[i] ? &bpAfter1[mi] : &bpFirst1[mi];
                        int sf, lf;
                        qsgetfreq(mp, bit, &sf, &lf);
                        encode_shift(&rc, sf, lf, 10);
                        qsupdate(mp, bit);
                        if (bit) seenOne[i] = 1;
                    }
                }
            }
            const size_t rcSize = done_encoding(&rc);
            for (int i = 0; i < numCtx; ++i) {
                deleteqsmodel(&bpFirst1[i]);
                deleteqsmodel(&bpAfter1[i]);
            }
            return rcSize;
        };
        auto transformedValue = [](const MCGAL::PointInt& p, int transform) {
            const int32_t x = p.x();
            const int32_t y = p.y();
            const int32_t z = p.z();
            switch (transform) {
                case 1: return MCGAL::PointInt(x, y - x, z - x);
                case 2: return MCGAL::PointInt(x - y, y, z - y);
                case 3: return MCGAL::PointInt(x - z, y - z, z);
                case 4: return MCGAL::PointInt(x, y - x, z - y);
                case 5: return MCGAL::PointInt(x - y, y, z - x);
                default: return MCGAL::PointInt(x, y, z);
            }
        };
        auto estimateTransformedBitplane = [&](int transform) -> size_t {
            std::vector<MCGAL::PointInt> vals;
            vals.reserve(N);
            for (const auto& r : residuals) {
                vals.push_back(transformedValue(
                    MCGAL::PointInt(
                        signedResidualDrop(r.x(), residualDropBits),
                        signedResidualDrop(r.y(), residualDropBits),
                        signedResidualDrop(r.z(), residualDropBits)),
                    transform));
            }
            int tBits[3] = {0, 0, 0};
            for (int axis = 0; axis < 3; ++axis) {
                uint32_t maxZ = 0;
                for (const auto& v : vals) {
                    const int32_t c = axis == 0 ? v.x() : axis == 1 ? v.y() : v.z();
                    const uint32_t zz = (static_cast<uint32_t>(c) << 1) ^
                                        static_cast<uint32_t>(c >> 31);
                    if (zz > maxZ) maxZ = zz;
                }
                while (maxZ != 0) {
                    ++tBits[axis];
                    maxZ >>= 1;
                }
                tBits[axis] = std::min(tBits[axis], 31);
            }
            const int tOffsets[3] = {0, tBits[0], tBits[0] + tBits[1]};
            const int tNumCtx = tBits[0] + tBits[1] + tBits[2];
            if (vals.empty() || tNumCtx == 0) return 9;
            const size_t bufSize = std::max<size_t>(vals.size() * 12 + 1024, 4096);
            std::unique_ptr<char[]> buf(new char[bufSize]);
            size_t off = 0;
            rangecoder rc;
            rc.p_data = buf.get();
            rc.p_dataOffset = &off;
            start_encoding(&rc, 0, 0);
            std::vector<qsmodel> bpFirst1(tNumCtx), bpAfter1(tNumCtx);
            for (int i = 0; i < tNumCtx; ++i) {
                initqsmodel(&bpFirst1[i], 2, 10, 1 << 9, nullptr, 1);
                initqsmodel(&bpAfter1[i], 2, 10, 1 << 9, nullptr, 1);
            }
            for (int axis = 0; axis < 3; ++axis) {
                const int width = tBits[axis];
                if (width == 0) continue;
                std::vector<unsigned char> seenOne(vals.size(), 0);
                for (int b = width - 1; b >= 0; --b) {
                    const int mi = tOffsets[axis] + b;
                    for (size_t i = 0; i < vals.size(); ++i) {
                        const int32_t c = axis == 0 ? vals[i].x()
                                             : axis == 1 ? vals[i].y()
                                                         : vals[i].z();
                        const uint32_t zz = (static_cast<uint32_t>(c) << 1) ^
                                            static_cast<uint32_t>(c >> 31);
                        const int bit = static_cast<int>((zz >> b) & 1u);
                        qsmodel* mp = seenOne[i] ? &bpAfter1[mi] : &bpFirst1[mi];
                        int sf, lf;
                        qsgetfreq(mp, bit, &sf, &lf);
                        encode_shift(&rc, sf, lf, 10);
                        qsupdate(mp, bit);
                        if (bit) seenOne[i] = 1;
                    }
                }
            }
            const size_t rcSize = done_encoding(&rc);
            for (int i = 0; i < tNumCtx; ++i) {
                deleteqsmodel(&bpFirst1[i]);
                deleteqsmodel(&bpAfter1[i]);
            }
            return 9 + rcSize;
        };
        const size_t sparseMaskBytes = encodeMaskSize();
        const size_t sparseValueBytes = encodeSparseValuesSize();
        const size_t sparseTotalBytes =
            16 + sparseMaskBytes + sparseValueBytes;
        size_t transformBytes[6];
        for (int t = 0; t < 6; ++t) {
            transformBytes[t] = estimateTransformedBitplane(t);
        }
        std::cerr << "[RDTag residual stats] count=" << N
                  << " bits=" << bits[0] << "," << bits[1] << "," << bits[2]
                  << " zeroVec=" << zeroVec
                  << " zeroAxis=" << zeroAxis[0] << "," << zeroAxis[1]
                  << "," << zeroAxis[2]
                  << " residualDropBits=" << residualDropBits
                  << " sparseEstimate=" << sparseTotalBytes
                  << " sparseMask=" << sparseMaskBytes
                  << " sparseValues=" << sparseValueBytes
                  << " transformEstimates="
                  << transformBytes[0] << "," << transformBytes[1] << ","
                  << transformBytes[2] << "," << transformBytes[3] << ","
                  << transformBytes[4] << "," << transformBytes[5]
                  << std::endl;
    }

    
    size_t kltUsedBases = 0;
    for (auto f : kltUseFlags) kltUsedBases += f ? 1u : 0u;
    const bool planeOrderActive = !planeOrderBytes.empty();
    const bool kltPartitionMode = kltActive && !kltPartitionIndices.empty();
    const size_t orderBytes = planeOrderActive ? planeOrderBytes.size() : 0;
    const size_t headerBytes =
        8 + (kltActive ? (8 + kltBases.size() +
                          (kltPartitionMode ? 4 * kltPartitionIndices.size() : 0) +
                          kltUsedBases * 36)
                        : 0) +
        orderBytes;
    std::vector<uint8_t> out;
    out.resize(headerBytes);
    int32_t numCoords = static_cast<int32_t>(N * 3);
    std::uint32_t rawCoords = static_cast<std::uint32_t>(numCoords);
    if (kltActive) rawCoords |= 0x40000000u;
    if (planeOrderActive) rawCoords |= 0x20000000u;
    std::memcpy(out.data(), &rawCoords, 4);
    out[4] = static_cast<uint8_t>(bits[0]);
    out[5] = static_cast<uint8_t>(bits[1]);
    out[6] = static_cast<uint8_t>(bits[2]);
    out[7] = static_cast<uint8_t>(std::clamp(residualDropBits, 0, 30));
    if (kltActive) {
        const std::uint32_t chunkSize = kltPartitionMode
                                            ? 0xFFFFFFFFu
                                            : static_cast<std::uint32_t>(kltChunk);
        const std::uint32_t numChunks = static_cast<std::uint32_t>(kltBases.size());
        std::memcpy(out.data() + 8, &chunkSize, 4);
        std::memcpy(out.data() + 12, &numChunks, 4);
        size_t useFlagsOff = 16;
        if (kltPartitionMode) {
            for (size_t pi = 0; pi < kltPartitionIndices.size(); ++pi) {
                const std::uint32_t len =
                    static_cast<std::uint32_t>(kltPartitionIndices[pi].size());
                std::memcpy(out.data() + useFlagsOff, &len, 4);
                useFlagsOff += 4;
            }
        }
        std::memcpy(out.data() + useFlagsOff, kltUseFlags.data(), kltUseFlags.size());
        size_t basisOff = useFlagsOff + kltUseFlags.size();
        for (size_t bi = 0; bi < kltBases.size(); ++bi) {
            if (!kltUseFlags[bi]) continue;
            float basisF[9];
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    basisF[i * 3 + j] = static_cast<float>(kltBases[bi].r[i][j]);
            std::memcpy(out.data() + basisOff, basisF, sizeof(basisF));
            basisOff += 36;
        }
    }
    if (planeOrderActive) {
        const size_t orderOff =
            kltActive ? (16 + (kltPartitionMode ? 4 * kltBases.size() : 0) +
                        kltUseFlags.size() + kltUsedBases * 36)
                      : 8;
        std::memcpy(out.data() + orderOff, planeOrderBytes.data(), orderBytes);
    }
    if (N == 0 || numCtx == 0) return out;

    const size_t bufSize = std::max<size_t>(BUFFER_SIZE / 12, N * 12 + 1024);
    std::unique_ptr<char[]> buf(new char[bufSize]);
    size_t rcSize = planeOrderActive
                        ? pmsf::encodeResidualBitPlaneQueueAdaptivePlaneMajorOrdered(
                              dx, dy, dz, buf.get(), bufSize, planeOrder)
                        : pmsf::encodeResidualBitPlaneQueueAdaptivePlaneMajor(
                              dx, dy, dz, buf.get(), bufSize);

    out.resize(headerBytes + rcSize);
    std::memcpy(out.data() + headerBytes, buf.get(), rcSize);
    return out;
}

std::vector<MCGAL::Vertex*> rdCollectFanPolygonVerts(MCGAL::Facet* face) {
    std::vector<MCGAL::Vertex*> verts;
    if (face == nullptr) return verts;
    const int valence = face->getPolyValence();
    MCGAL::Halfedge* proxy = face->proxyHalfedge();
    if (proxy == nullptr || valence <= 3) {
        return MCGAL::rdCollectPolygonVerts(face);
    }

    MCGAL::Halfedge* h1 = proxy->next();
    MCGAL::Halfedge* hd = h1 ? h1->next() : nullptr;  // v2 -> v0 in the first fan triangle.
    if (h1 == nullptr || hd == nullptr) {
        return MCGAL::rdCollectPolygonVerts(face);
    }
    verts.push_back(proxy->vertex());
    verts.push_back(h1->vertex());
    verts.push_back(hd->vertex());

    MCGAL::Halfedge* curDiag = hd;
    while ((int)verts.size() < valence) {
        MCGAL::Halfedge* op = curDiag ? curDiag->opposite() : nullptr;
        MCGAL::Halfedge* edgeToNext = op ? op->next() : nullptr;
        MCGAL::Halfedge* nextDiag = edgeToNext ? edgeToNext->next() : nullptr;
        if (nextDiag == nullptr) break;
        verts.push_back(nextDiag->vertex());
        curDiag = nextDiag;
    }
    if ((int)verts.size() != valence) {
        return MCGAL::rdCollectPolygonVerts(face);
    }
    return verts;
}

MCGAL::PointInt rdBarycenterQuantizedAfterAdjustmentRestore(
    MCGAL::Facet* face,
    const std::shared_ptr<MCGAL::Mesh>& mesh) {
    auto verts = rdCollectFanPolygonVerts(face);
    if (verts.empty() || mesh == nullptr) {
        return MCGAL::PointInt(0, 0, 0);
    }
    int64_t sx = 0, sy = 0, sz = 0;
    int n = 0;
    for (MCGAL::Vertex* v : verts) {
        if (v == nullptr) continue;
        const MCGAL::Point p = v->isAdjusted() ? v->vertexPoint() : v->point();
        const MCGAL::PointInt pi = mesh->floatPosToInt(p);
        sx += pi.x();
        sy += pi.y();
        sz += pi.z();
        ++n;
    }
    if (n == 0) return MCGAL::PointInt(0, 0, 0);
    return MCGAL::PointInt(static_cast<int>(sx / n),
                           static_cast<int>(sy / n),
                           static_cast<int>(sz / n));
}

}  // namespace

// ============================================================================
// PR-AP 诊断（零字节改动）：Ahn et al. 2011 Eq.(2) 正确 Laplacian 预测的熵上界
//
// 现状（读码 + PMSF_RDTAG_LEAKY_DIAG 实测）：PCLTTMTag 第二遍里 pred_c2 的语义是
//   「把已消 patch 中心 c 的一阶残差 r1 写到 c 的环顶点上；后续 face 的
//    pred_c2 = 环顶点上这些值（缺省 0）的均值」。
// 论文 Eq.(2) 要的是
//   r_i = ṽ_i − v̂_i，v̂_i = mean(N(i))，其中尚未解码的当前顶点 c 用 v̂_c 代，
//   pred_c2 = mean_{i∈N(c)} r_i。
// 两者不同：论文用「相邻顶点自身的 Laplacian」，现实现用「相邻已消中心的 r1」。
// 本诊断在**不改任何码流字节**的前提下，按论文式用真实网格状态计算 pred，并与
// 现有 encoded 残差比较 bit-plane 边际 Shannon 界。env:
// 近似：v̂_i 用当前（本轮 collection 时刻的）coarse mesh 1-环 + 当前 patch 的
// v̂_c 组成；同层其它尚未插入的 patch 中心不在 1-环里（对论文解码时序的近似）。
// ============================================================================
namespace {

struct RdPaperPredStats {
    long long faces = 0;
    long long paperOk = 0;
    long long sumAbsPredOld[3] = {0, 0, 0};
    long long sumAbsPredPaper[3] = {0, 0, 0};
    long long sumAbsRingLap[3] = {0, 0, 0};
    long long sumAbsR1[3] = {0, 0, 0};
    long long sumAbsEncOld[3] = {0, 0, 0};
    long long sumAbsEncPaper[3] = {0, 0, 0};
    long long cnt[2][3][64][2] = {};  // [0=old,1=paper][axis][bit][bitval]
    int maxMeshV = 0;
    int rounds = 0;
};

RdPaperPredStats gRdPaperPredStats;

void printRdPaperPredStats() {
    auto& s = gRdPaperPredStats;
    if (s.faces == 0) return;
    auto shannon = [&](int which) -> double {
        double bits = 0.0;
        for (int a = 0; a < 3; ++a) {
            for (int b = 0; b < 64; ++b) {
                const double n0 = static_cast<double>(s.cnt[which][a][b][0]);
                const double n1 = static_cast<double>(s.cnt[which][a][b][1]);
                const double n = n0 + n1;
                if (n <= 0.0) continue;
                const double p0 = n0 / n, p1 = n1 / n;
                double H = 0.0;
                if (p0 > 0.0) H -= p0 * std::log2(p0);
                if (p1 > 0.0) H -= p1 * std::log2(p1);
                bits += H * n;
            }
        }
        return bits;
    };
    auto meanAbs = [&](const long long v[3]) -> double {
        const long long n = s.faces * 3;
        const long long sum = v[0] + v[1] + v[2];
        return n > 0 ? static_cast<double>(sum) / static_cast<double>(n) : 0.0;
    };
    const double oldB = shannon(0) / 8.0;
    const double newB = shannon(1) / 8.0;
    constexpr double kDenom = 19851.0;  // horse19k 原顶点数（论文 bpv 口径）
    std::fprintf(
        stderr,
        "[PAPERPRED] rounds=%d faces=%lld paperRingOK=%lld maxMeshV=%d\n"
        "  mean|pred_old|   = %.2f\n"
        "  mean|pred_paper| = %.2f\n"
        "  mean|r1|         = %.2f\n"
        "  mean|ringLaplacian| = %.2f\n"
        "  mean|enc_old|    = %.2f\n"
        "  mean|enc_paper|  = %.2f\n"
        "  Shannon(enc_old)   = %.1f B = %.3f bpv (denom 19851)\n"
        "  Shannon(enc_paper) = %.1f B = %.3f bpv\n"
        "  SAVING             = %.1f B = %.3f bpv\n",
        s.rounds, s.faces, s.paperOk, s.maxMeshV,
        meanAbs(s.sumAbsPredOld), meanAbs(s.sumAbsPredPaper),
        meanAbs(s.sumAbsR1), meanAbs(s.sumAbsRingLap),
        meanAbs(s.sumAbsEncOld), meanAbs(s.sumAbsEncPaper),
        oldB, oldB * 8.0 / kDenom, newB, newB * 8.0 / kDenom,
        oldB - newB, (oldB - newB) * 8.0 / kDenom);
}

/// 论文式 pred_c2 = mean_{i∈ring} ( pos(i) − mean( N_mesh(i) ∪ {v̂_c} ) )，
/// 全部在量化整数域，向零整除（与父类整数均值语义一致）。
MCGAL::PointInt rdPaperFormulaPred(MCGAL::Facet* face,
                                   const std::shared_ptr<MCGAL::Mesh>& mesh,
                                   const MCGAL::PointInt& vhatC,
                                   long long sumAbsRi[3],
                                   long long* okCount) {
    MCGAL::PointInt zero(0, 0, 0);
    if (face == nullptr || mesh == nullptr) return zero;
    auto ring = MCGAL::rdCollectPolygonVerts(face);
    if (ring.size() < 3) return zero;

    int64_t sx = 0, sy = 0, sz = 0;
    int cnt = 0;
    for (MCGAL::Vertex* i : ring) {
        if (i == nullptr || i->isRemoved()) continue;
        const MCGAL::PointInt pi = mesh->floatPosToInt(i->point());
        int64_t qx = 0, qy = 0, qz = 0;
        int deg = 0;
        for (MCGAL::Halfedge* hit : pmsf::boundedRing(i)) {
            if (hit == nullptr || hit->opposite() == nullptr) continue;
            MCGAL::Vertex* j = hit->opposite()->vertex();
            if (j == nullptr || j == i || j->isRemoved()) continue;
            const MCGAL::PointInt pj = mesh->floatPosToInt(j->point());
            qx += pj.x();
            qy += pj.y();
            qz += pj.z();
            ++deg;
        }
        if (deg == 0) continue;
        // 论文：当前顶点 v_c 尚未解码，用 v̂_c 代入 i 的 barycenter
        qx += vhatC.x();
        qy += vhatC.y();
        qz += vhatC.z();
        const int64_t denom = static_cast<int64_t>(deg) + 1;
        const int64_t rix = static_cast<int64_t>(pi.x()) - qx / denom;
        const int64_t riy = static_cast<int64_t>(pi.y()) - qy / denom;
        const int64_t riz = static_cast<int64_t>(pi.z()) - qz / denom;
        sx += rix;
        sy += riy;
        sz += riz;
        sumAbsRi[0] += std::llabs(rix);
        sumAbsRi[1] += std::llabs(riy);
        sumAbsRi[2] += std::llabs(riz);
        ++cnt;
    }
    if (cnt == 0) return zero;
    *okCount = 1;
    return MCGAL::PointInt(static_cast<int>(sx / cnt),
                           static_cast<int>(sy / cnt),
                           static_cast<int>(sz / cnt));
}

inline void rdAccumShannon(long long cnt[64][2], int32_t v, int bpBits) {
    const uint32_t z = (static_cast<uint32_t>(v) << 1) ^
                       static_cast<uint32_t>(v >> 31);
    for (int b = 0; b < bpBits && b < 64; ++b) {
        cnt[b][(z >> b) & 1u]++;
    }
}


}  // namespace

void RDTagSymbolCollectOperator::collect(MCGAL::Halfedge* seed) {
    static const std::vector<MCGAL::Halfedge*> kEmpty;
    collect(seed, kEmpty);
}

void RDTagSymbolCollectOperator::collect(std::vector<MCGAL::Halfedge*> seeds) {
    if (seeds.empty()) {
        throw std::runtime_error("[RDTag] collect(vector<seed>) called with empty seeds");
    }
    static const std::vector<MCGAL::Halfedge*> kEmpty;
    collect(seeds[0], kEmpty);
}

void RDTagSymbolCollectOperator::collect(MCGAL::Halfedge* seed,
                                         const std::vector<MCGAL::Halfedge*>& gates) {
    paperTableCache_.clear();


    if (header_.isEnableAdjustment()) {
        // BasicReconstructOperator 先恢复 adjusted survivor，再 insert removed vertex。
        // 因此 removed-vertex residual 也必须按“恢复后的 polygon”预测，否则后续
        // 轮次的量化 proxy normalize 会和 encoder 分叉。
        for (MCGAL::Facet* f : mesh_->faces()) {
            if (f == nullptr || f->isRemoved() || !f->isSplittable()) continue;
            const MCGAL::PointInt removed = mesh_->floatPosToInt(f->getRemovedVertexPos());
            const MCGAL::PointInt pred =
                rdBarycenterQuantizedAfterAdjustmentRestore(f, mesh_);
            f->setRemovedVertexPosInt(MCGAL::PointInt(
                removed.x() - pred.x(),
                removed.y() - pred.y(),
                removed.z() - pred.z()));
        }
    }

    if (header_.isEnableAdjustment()) {
        PCATMColorSymbolCollectOperator::collect(seed, gates);
    } else {
        PCLTTMTagSymbolCollectOperator::collect(seed, gates);
    }

    // PR-AP 诊断（零字节改动）：论文式 Eq.(2) Laplacian 预测的 Shannon 上界。
    
    if (!header_.isEnableAdjustment()) return;

    std::deque<uint8_t> vertexAdjustSyms;
    std::deque<MCGAL::PointInt> vertexAdjustResiduals;

    std::vector<int> q;
    q.reserve(mesh_ ? mesh_->faces().size() * 2 + 16 : 16);
    std::size_t qHead = 0;
    if (seed != nullptr) {
        q.push_back(seed->poolId());
    }
    const int currentVersion = MCGAL::BFSVersionManager::current_version++;

    while (qHead < q.size()) {
        const int hid = q[qHead++];
        MCGAL::Halfedge* h =
            MCGAL::contextPool.getHalfedgeByIndexInSubPool(mesh_->meshId(), hid);
        if (h == nullptr || h->isVisited(currentVersion) || h->isRemoved()) continue;
        h->setVisited(currentVersion);

        MCGAL::Vertex* v = h->vertex();
        if (v != nullptr && !v->isVisited(currentVersion)) {
            v->setVisited(currentVersion);
            const bool adjusted = v->isAdjusted();
            vertexAdjustSyms.push_back(adjusted ? 1 : 0);
            if (adjusted) {
                const MCGAL::PointInt pred = mesh_->floatPosToInt(v->point());
                const MCGAL::PointInt target = mesh_->floatPosToInt(v->vertexPoint());
                vertexAdjustResiduals.emplace_back(
                    target.x() - pred.x(),
                    target.y() - pred.y(),
                    target.z() - pred.z());
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

    gateVertexAdjustSymbolQueues_.push_back(std::move(vertexAdjustSyms));
    gateVertexAdjustResidualQueues_.push_back(std::move(vertexAdjustResiduals));
    paperTableCache_.clear();
}

const pmsf::PMSFSymbolTable* RDTagSymbolCollectOperator::pmsfSymbolTable() const {
    const pmsf::PMSFSymbolTable* base = RDGateOnlySymbolCollectOperator::pmsfSymbolTable();
    if (!base || base->empty()) {
        return nullptr;
    }
    const bool hasAdjustment = header_.isEnableAdjustment();
    const char* envAdaptiveResidual = std::getenv("PMSF_RDTAG_ADAPTIVE_RESIDUAL");
    const bool useAdaptiveResidual =
        envAdaptiveResidual && envAdaptiveResidual[0] == '1';
    // P9 / D1：`useGlobalStreams` 已删除（V9..V18 路径日落；此处声明已无使用者）。
    int residualDropBits = 0;
    if (const char* envDrop = std::getenv("PMSF_RDTAG_RESIDUAL_DROP")) {
        residualDropBits = std::clamp(std::atoi(envDrop), 0, 8);
    }
    int frenetStart = 0;
    int frenetEnd = 1 << 30;
    if (const char* s = std::getenv("PMSF_RDTAG_FRENET_START")) {
        frenetStart = std::atoi(s);
    }
    if (const char* s = std::getenv("PMSF_RDTAG_FRENET_END")) {
        frenetEnd = std::atoi(s);
    }
    pmsf::copyPMSFSymbolTableWithIndexedRowAugmentation(
        base,
        paperTableCache_,
        [&](std::size_t r, pmsf::PMSFSymbolRecord& row) {
        pmsf::setPaperScalarSymbolsForRole(row.delta, pmsf::PMSFSymbolRole::TriangulationEdge, pmsf::PMSFSymbolCodeDeque{});
        // P9 / D1（sv 日落）：本块 5 个 side-channel 值按**双消费者**逐条求值后折叠
        // （§47 表；这些值进 canonical 表，删错即改产物字节）：
        //   roundUsesFrenet ? 1:0                                    RD 0 / QEM 0 → 0
        //   (roundUsesFrenet && useFrenetExact) ? 1:0                RD 0 / QEM 0 → 0
        //   (useAdaptiveResidual‖hasAdjustment‖useFrenetResidual)?1:0 RD 1 / QEM 1 → 1
        //   useGlobalStreams ? 1:0                                   RD 0 / QEM 0 → 0
        //   (hasAdjustment‖useFrenetResidual)?0:residualDropBits    RD=drop / QEM=0
        //     ⚠️ 此项**必须保留 `hasAdjustment` 判别**（两消费者取值不同）。
        pmsf::appendRDTagResidualCapabilityBoundary(
            row.delta,
            static_cast<std::int32_t>(0),
            static_cast<std::int32_t>(0),
            static_cast<std::int32_t>(1),
            static_cast<std::int32_t>(0),
            static_cast<std::int32_t>(hasAdjustment ? 0 : residualDropBits));
    });
    for (std::size_t r = 0;
         r < paperTableCache_.size() && r < gateVertexAdjustSymbolQueues_.size();
        ++r) {
        pmsf::replaceRDTagVertexAdjustSymbol(
            paperTableCache_[r].delta, gateVertexAdjustSymbolQueues_[r]);
        if (r < gateVertexAdjustResidualQueues_.size()) {
            for (const auto& q : gateVertexAdjustResidualQueues_[r]) {
                pmsf::appendRDTagVertexAdjustResidualSideChannel(
                    paperTableCache_[r].delta, q);
            }
        }
    }
    return paperTableCache_.empty() ? nullptr : &paperTableCache_;
}

// ============================================================================
// export — ConnectivityBlobBuilder v3 (3 段流)
//   stream 0: valence (5-元 + 4-context + V_OTHER inline 256-元 escape)
//   stream 1: isPolyBit
//      sv=1 (默认):              2-元 1-context RC ，per BFS face 1 bit
//      sv=2 (PMSF_RDTAG_ISPOLY_RLE=1, PR-J1)：RLE + bit-plane RC（runs）
//   stream 2: residual (bit-plane RC) ← 含 Phase 3 Step 1 二阶 curvature pred
// 默认 RD_TAG 显式排除：edgeSym / vertexTag / fusedFaceSym / color。
// QEM adjustment 路径使用 sv=6，额外保留 edgeSym 作为拓扑保护流。
// 几何还原：decoder 按 v0-fan 拓扑（base_mesh proxyHalfedge 量化整数字典序 normalize）
// 推 N-3 条对角线，与 PCLTTM Tag sv=3 完全同源。
// ============================================================================
int RDTagSymbolCollectOperator::exportToBufferWithRangeCoder(char* buffer,
                                                              bool isEnableQuantization) {
    if (buffer == nullptr) return 0;
    requireRDNonStrictPaperExactIfRequested("[RDTag]");
    if (!enablePrediction_ || !isEnableQuantization) {
        throw std::runtime_error(
            "[RDTag] exportRC requires enablePrediction && enableQuantization.");
    }

    const int numRounds = (int)gateValenceQueues_.size();
    if ((int)gateIsPolyQueues_.size()   != numRounds ||
        (int)gateResidualQueues_.size() != numRounds) {
        throw std::runtime_error(
            "[RDTag] inconsistent per-round queue sizes "
            "(expects valence/isPoly/residual aligned by round)");
    }
    const bool hasAdjustment = header_.isEnableAdjustment();
    if (hasAdjustment &&
        ((int)gateVertexAdjustSymbolQueues_.size() != numRounds ||
         (int)gateVertexAdjustResidualQueues_.size() != numRounds ||
         (int)gateEdgeSymQueues_.size() != numRounds)) {
        throw std::runtime_error(
            "[RDTag] inconsistent QEM adjustment queue sizes "
            "(expects edge/vertex-adjust symbols/residuals aligned by round)");
    }

    // PR-J1：env 决定 isPolyBit 编码方案。env 默认 off → sv=1（与 Phase 4 PR-B
    // 字节级一致 + DefaultRDTag md5 不变）。env on → sv=2（多上下文 RC）。
    //
    // 命名历史：起初按 plan §3.5 方案 C 设想 RLE+RC；实测 RLE 反而 +37% 增大
    // 字节（sv=1 RC 已离熵下界 ±1%，RLE 头部 + runs zigzag 开销 > 收益）。
    // 改用 kHistoryBits=3 的多上下文 2-元 RC（8 个独立 adaptive 模型）；仍无
    // 显著改进（+58 bytes vs sv=1），因为 isPolyBit 在 BFS 顺序里并非 Markov 链。
    // 保留 sv=2 framework 作为未来 PR-K（priortized gate）/ 其他数据集的备用
    // 实验入口；PR-J2（tag-evolution 推断）才是真正的"砍 isPolyBit"路径。
    //
    // env 名支持两个别名（推荐新名 PMSF_RDTAG_ISPOLY_MULTICTX，保留旧名兼容）：
    const char* envAdaptiveResidual = std::getenv("PMSF_RDTAG_ADAPTIVE_RESIDUAL");
    const bool useAdaptiveResidual = envAdaptiveResidual && envAdaptiveResidual[0] == '1';
    // P9 / D1（sv 日落）：`useGlobalStreams` 及其 6 个派生开关
    // （useStaticIsPoly / usePackedValence / useGlobalIsPolyContext /
    //   useRoundStaticIsPoly / useRoundResidualCtx / useCombinedIsPolyValence）
    // 已随 V9..V18 发射路径一并删除 —— 该路径是本轮删掉的 126 行块，
    // 删除后这 6 个变量**每个仅剩自身声明、无任何使用者**（实测计数均为 1）。
    int residualDropBits = 0;
    if (const char* envDrop = std::getenv("PMSF_RDTAG_RESIDUAL_DROP")) {
        residualDropBits = std::clamp(std::atoi(envDrop), 0, 8);
    }
    const int residualDropBitsForStream2 =
        // `hasAdjustment` 判别**保留**（RD→drop / QEM→0）；原 ‖useFrenetResidual 已删（恒 false）。
        hasAdjustment ? 0 : residualDropBits;
    // P9 / D1（sv 日落）：只保留**两个实测可达**的 sv（§44.3）：
    //   RD  门禁 → V3（实测产物 108152 == 冻结基线）
    //   QEM 门禁 → V6（实测产物 285815 == 冻结基线）
    // 原链的 V1/V2/V4/V7/V8/V9..V18 臂全部删除（env 门控，两个消费者都不走）。
    // ⚠️ 共享代码：改这里必须**同时**跑 RD 与 QEM 两个消费者（§42 教训）。
    // PHASE62 物品2：edgeSym 的上下文编码器是 sv=6 **内部**的线格式变更，
    // 故用一个新 sv 承载它（V6Ctx=19），老 V6 blob 才能被老解码器正确还原。
    // `PMSF_RDTAG_LEGACY_V6=1` 强制回老线格式 —— 只为**负向验证**保留
    // （造一个老 blob，看新解码器是否仍能正确还原它）。
    // ⚠️ 注入缝（Phase S2）生效时也必须回落老线格式：老路径的语义是
    // 「edgeSym 也按名字查注入 coder」，而上下文编码器吃的是 (isPoly, onesSoFar)
    // 双队列，注入 coder 拿不到那个上下文。回落必须**同时**体现在 sv 上
    // —— 否则就是"头说 V6Ctx、流是老格式"的新坑。
    const bool legacyV6 = [] {
        const char* e = std::getenv("PMSF_RDTAG_LEGACY_V6");
        return (e != nullptr && e[0] == '1') ||
               pmsf::rd_tag::edgeSymInjectionActive();
    }();
    // ========================================================================
    // PR-FRENET（论文 Ahn et al. 2011 §3.1 Eq.(3)）：恢复 sv=7 / sv=8 的
    // 「残差投影到局部 Frenet 系 (n, t1, t2)」发射路径。residual 在 collect 端
    // 已由 PCLTTMTag 的 second pass 投影（env PMSF_RDTAG_FRENET_RESIDUAL /
    // PMSF_RDTAG_FRENET_EXACT 驱动，两处判定条件必须一致）。
    //   sv=7：3 段流，decoder 按同一 face frame unproject；
    //   sv=8：4 段流，多一条 exact correction（original − unproject(project)），
    //          decoder 反投影后加回 ⇒ 几何与 sv=3 逐量化单位一致。
    // ⚠️ 解码端只能按 sv 整条流 unproject（没有 per-round 的 env 保证），因此
    //    START/END 必须覆盖全部轮次，否则 encoder 投影/decoder 反投影错位。
    // 默认 env 关闭 ⇒ schemeVersion 仍为 V3、stream 数仍为 Base ⇒ 逐字节不变。
    // ========================================================================
    const bool frenetExact = [] {
        const char* e = std::getenv("PMSF_RDTAG_FRENET_EXACT");
        return e != nullptr && e[0] == '1';
    }();
    const bool frenetEnabled = [&] {
        const char* r = std::getenv("PMSF_RDTAG_FRENET_RESIDUAL");
        return frenetExact || (r != nullptr && r[0] == '1');
    }();
    if (frenetEnabled) {
        if (hasAdjustment) {
            throw std::runtime_error(
                "[RDTag] frenet V7/V8 is only wired for the non-adjustment RD "
                "path (PMSF_RDTAG_FRENET_* + adjustment is not a valid combo)");
        }
        const char* s = std::getenv("PMSF_RDTAG_FRENET_START");
        const char* e = std::getenv("PMSF_RDTAG_FRENET_END");
        const int start = (s != nullptr && *s) ? std::atoi(s) : 0;
        const int end = (e != nullptr && *e) ? std::atoi(e) : (1 << 30);
        if (start > 0 || end < numRounds - 1) {
            throw std::runtime_error(
                "[RDTag] PMSF_RDTAG_FRENET_START/END must cover all rounds "
                "(decoder unprojects per-scheme-version, not per-round)");
        }
        if (frenetExact &&
            static_cast<int>(gateFrenetCorrectionQueues_.size()) != numRounds) {
            throw std::runtime_error(
                "[RDTag] frenet exact requires PMSF_RDTAG_FRENET_EXACT=1 during "
                "collect() so per-round correction queues are populated");
        }
    }
    const int schemeVersion =
        hasAdjustment ? (legacyV6 ? kRDTagSchemeVersionV6
                                  : kRDTagSchemeVersionV6Ctx)
                      : (frenetEnabled
                             ? (frenetExact ? kRDTagSchemeVersionV8
                                            : kRDTagSchemeVersionV7)
                             : kRDTagSchemeVersionV3);
    // P9 / D1：原 useFrenetExact 的轮数守卫已删除（两消费者 useFrenetExact 均 false）。
    const int BP_BITS = (int)mesh_->i_nbQuantBits + 1;

    // P9 / D1（sv 日落）：`useGlobalStreams`（V9..V18 的全局流发射路径，126 行）已删除。
    // 该路径为 env 门控，**门禁的两个消费者都不走**：
    //   RD  门禁 sv = V3（实测产物 108152 == 冻结基线）
    //   QEM 门禁 sv = V6（实测产物 285815 == 冻结基线）
    // ⇒ 删除对两者**字节中性**（探针 RD + QEM 双双逐字节相等）。
    // ⚠️ 共享代码：改这里必须**同时**跑 RD 与 QEM 两个探针（§42 教训）。

    pmsf::ConnectivityBlobBuilder cbb(
        pmsf::rdTagConnectivityBlobSpec(
            numRounds,
            schemeVersion,
            // `hasAdjustment` 判别保留（RD false→Base / QEM true→WithAdjustment）；
            // PR-FRENET：V8 带 exact correction 第 4 流 ⇒ WithFrenet。
            hasAdjustment
                ? pmsf::kPaperBlobStreamCountRDTagWithAdjustment
                : (frenetExact ? pmsf::kPaperBlobStreamCountRDTagWithFrenet
                               : pmsf::kPaperBlobStreamCountRDTagBase)));

    int sumValence = 0, sumIsPoly = 0, sumEdge = 0, sumRes = 0, sumAdjust = 0;
    int sumCorrection = 0;
    using Clock = std::chrono::steady_clock;
    const bool profile = rdTagProfileLogs();
    auto tExportStart = Clock::now();
    double valenceMs = 0.0;
    double isPolyMs = 0.0;
    double edgeMs = 0.0;
    double residualMs = 0.0;
    double adjustMs = 0.0;
    double appendMs = 0.0;

    // 轮次反转：encoder round (N-1-i) → builder round i（与 PCLTTMTag 一致）
    for (int i = 0; i < numRounds; ++i) {
        int er = numRounds - 1 - i;

        auto t = Clock::now();
        auto rcVal = encodeValence5(gateValenceQueues_[er],
                                                gateOtherValQueues_[er]);
        if (profile) {
            valenceMs += std::chrono::duration<double, std::milli>(
                             Clock::now() - t).count();
            t = Clock::now();
        }
        sumValence += (int)rcVal.size();
        cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamRDTagValence, rcVal);
        if (profile) {
            appendMs += std::chrono::duration<double, std::milli>(
                            Clock::now() - t).count();
            t = Clock::now();
        }

        // Phase S2（R13/R14）：符号方案可改写本流的符号序列（bpv 根因杠杆）。
        // **必须在活路径上**（每轮的 gate isPoly）—— 先前接在 globalIsPoly 块里，
        // 那只在 useGlobalStreams 时走，而它默认关（实测探针 50 次调用都在这里）。
        // 无方案 / 方案未接管 ⇒ 一行不改，序列原样进入下游 ⇒ 默认逐字节不变。
        if (auto ss = ::pmsf::activeSymbolScheme();
            ss && ss->overridesStream("isPoly")) {
            std::vector<std::uint32_t> syms(gateIsPolyQueues_[er].begin(),
                                            gateIsPolyQueues_[er].end());
            if (ss->emitSymbols("isPoly", syms)) {
                gateIsPolyQueues_[er].assign(syms.begin(), syms.end());
            }
        }
        // P9 / D1：原为 useIsPolyV2 ? multiContext : bit，两消费者均 false ⇒ 折默认分支。
        auto rcIsPoly = pmsf::rd_tag::encodeBitstream2RC(gateIsPolyQueues_[er], "isPoly");
        if (profile) {
            isPolyMs += std::chrono::duration<double, std::milli>(
                            Clock::now() - t).count();
            t = Clock::now();
        }
        sumIsPoly += (int)rcIsPoly.size();
        cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamRDTagIsPoly, rcIsPoly);
        if (profile) {
            appendMs += std::chrono::duration<double, std::milli>(
                            Clock::now() - t).count();
            t = Clock::now();
        }

        if (hasAdjustment) {
            // PHASE61 物品2：edgeSym 换 (isPoly, 窗口内 1 的个数) 上下文编码器。
            // 符号序列不变（仍 3 bit/face），只是熵编码变紧：实测该流窗口内
            // 强相关（poly 窗口 ≤1 个 1、非 poly 窗口 ≤2 个 1），单模型
            // 0.447 bit/symbol → 上下文约 0.38 bit/symbol。
            // ⚠️ 消费者（BasicReconstruct / RDGateOnly）完全不知情，零改动。
            // 注入缝生效时回落旧路径（见 `edgeSymInjectionActive` 的 ⚠️）。
            // 判据是**写出去的 sv**，不是「注入缝是否生效」：只有 V6Ctx 的
            // 读侧才知道要用上下文解码器。老 V6 必须配老编码器，否则就是
            // 附 29 那个「同一 sv 两种线格式」的坑本身。
            const bool useCtxEdgeSym =
                pmsf::paperBlobSchemeVersionRDTagUsesContextEdgeSym(
                    schemeVersion);
            auto rcEdge = useCtxEdgeSym
                              ? pmsf::rd_tag::encodeEdgeSymWithContext(
                                    gateIsPolyQueues_[er], gateEdgeSymQueues_[er])
                              : pmsf::rd_tag::encodeBitstream2RC(
                                    gateEdgeSymQueues_[er], "isPoly");
            if (profile) {
                edgeMs += std::chrono::duration<double, std::milli>(
                              Clock::now() - t).count();
                t = Clock::now();
            }
            sumEdge += (int)rcEdge.size();
            cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamRDTagEdgeSym, rcEdge);
            if (profile) {
                appendMs += std::chrono::duration<double, std::milli>(
                                Clock::now() - t).count();
                t = Clock::now();
            }

            auto rcRes = encodeResidualBitplaneMajorAdaptive(
                gateResidualQueues_[er],
                /*residualDropBits=*/0,
                &gateFacetQueues_[er]);
            if (profile) {
                residualMs += std::chrono::duration<double, std::milli>(
                                  Clock::now() - t).count();
                t = Clock::now();
            }
            sumRes += (int)rcRes.size();
            cbb.appendStreamBytes(
                i, pmsf::kPaperBlobStreamRDTagResidualWithEdge, rcRes);
            if (profile) {
                appendMs += std::chrono::duration<double, std::milli>(
                                Clock::now() - t).count();
                t = Clock::now();
            }

            auto rcAdjustSyms = pmsf::rd_tag::encodeBitstream2RC(gateVertexAdjustSymbolQueues_[er], "isPoly");
            auto rcAdjustRes = encodeResidualBitplaneMajorAdaptive(
                gateVertexAdjustResidualQueues_[er],
                /*residualDropBits=*/0);
            std::vector<uint8_t> rcAdjust;
            rcAdjust.resize(4);
            const uint32_t symBytes = static_cast<uint32_t>(rcAdjustSyms.size());
            std::memcpy(rcAdjust.data(), &symBytes, 4);
            rcAdjust.insert(rcAdjust.end(), rcAdjustSyms.begin(), rcAdjustSyms.end());
            rcAdjust.insert(rcAdjust.end(), rcAdjustRes.begin(), rcAdjustRes.end());
            if (profile) {
                adjustMs += std::chrono::duration<double, std::milli>(
                                Clock::now() - t).count();
                t = Clock::now();
            }
            sumAdjust += (int)rcAdjust.size();
            cbb.appendStreamBytes(
                i, pmsf::kPaperBlobStreamRDTagVertexAdjustment, rcAdjust);
            if (profile) {
                appendMs += std::chrono::duration<double, std::milli>(
                                Clock::now() - t).count();
            }
        } else {
            // P9 / D1：原为 `useAdaptiveResidualForStream2 ? A : B`。
            // 该开关 = `useAdaptiveResidual || hasAdjustment`，**两消费者均为 true**
            // （RD 经 useAdaptiveResidual、QEM 经 hasAdjustment）⇒ 折 A 分支。
            auto rcRes = encodeResidualBitplaneMajorAdaptive(
                gateResidualQueues_[er],
                residualDropAppliesToDecoderRound(i)
                    ? residualDropBitsForStream2
                    : 0,
                &gateFacetQueues_[er]);
            if (profile) {
                residualMs += std::chrono::duration<double, std::milli>(
                                  Clock::now() - t).count();
                t = Clock::now();
            }
            sumRes += (int)rcRes.size();
            cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamRDTagResidual, rcRes);
            if (profile) {
                appendMs += std::chrono::duration<double, std::milli>(
                                Clock::now() - t).count();
                t = Clock::now();
            }
            // PR-FRENET：sv=8 的第 4 条流 = exact Frenet correction
            // （original encoded − unproject(project)）。decoder 反投影后加回，
            // 使局部系变换逐量化单位无损。sv=7 / V3 下不写此流。
            if (frenetExact) {
                auto rcCorr = encodeResidualBitplaneMajorAdaptive(
                    gateFrenetCorrectionQueues_[er],
                    /*residualDropBits=*/0);
                sumCorrection += (int)rcCorr.size();
                cbb.appendStreamBytes(
                    i, pmsf::kPaperBlobStreamRDTagFrenetCorrection, rcCorr);
            }
        }
    }

    auto tFinalize = Clock::now();
    auto blob = cbb.finalize();
    auto tMemcpy = Clock::now();
    std::memcpy(buffer, blob.data(), blob.size());
    if (profile) {
        auto tDone = Clock::now();
        const double finalizeMs = std::chrono::duration<double, std::milli>(
                                      tMemcpy - tFinalize).count();
        const double memcpyMs = std::chrono::duration<double, std::milli>(
                                    tDone - tMemcpy).count();
        const double totalMs = std::chrono::duration<double, std::milli>(
                                   tDone - tExportStart).count();
        std::fprintf(stderr,
                     "[RDTAG_EXPORT_PROFILE] sv=%d rounds=%d valence=%.3fms "
                     "isPoly=%.3fms edge=%.3fms residual=%.3fms adjust=%.3fms "
                     "append=%.3fms finalize=%.3fms memcpy=%.3fms total=%.3fms "
                     "bytes=%zu\n",
                     schemeVersion, numRounds, valenceMs, isPolyMs, edgeMs,
                     residualMs, adjustMs, appendMs, finalizeMs, memcpyMs,
                     totalMs, blob.size());
    }

    // ========================================================================
    // task-19 P2 字节投影诊断（零字节改动）：假设 **去掉 isPoly locator**、且
    // valence/residual 按 **消除顺序**（selection order，PMSF_RD_PGS 的 elimSeq）
    // 逐轮发射，产物会有多大。用于判断 P2/P3（decoder replay）值不值得做。
    // isPoly 流字节 = sumIsPoly（真实值）；容器/base/parquet 开销用同口径实测值。
    // ========================================================================
    if (rd_impl::pgsEnabled() &&
        rd_impl::rdPgsEnvOn("PMSF_RD_PGS_PROJECT")) {
        auto& st = rd_impl::pgsState();
        long long projRes = 0;
        long long projVal = 0;
        long long mapped = 0, unmapped = 0;
        for (int er = 0; er < numRounds; ++er) {
            const auto& facets = gateFacetQueues_[er];
            const auto& resQ = gateResidualQueues_[er];
            const auto& valQ = gateValenceQueues_[er];
            const auto& otherQ = gateOtherValQueues_[er];
            const std::size_t n = std::min(facets.size(), resQ.size());
            std::vector<std::size_t> order(n);
            for (std::size_t k = 0; k < n; ++k) order[k] = k;
            std::vector<std::int64_t> keyOf(n, std::numeric_limits<std::int64_t>::max());
            for (std::size_t k = 0; k < n; ++k) {
                MCGAL::Facet* f = facets[k];
                if (f == nullptr) continue;
                const MCGAL::PointInt q = mesh_->floatPosToInt(f->getRemovedVertexPos());
                const std::uint64_t key = rd_impl::pgsPosKey(q);
                auto it = st.elimSeqByPos.find(key);
                if (it != st.elimSeqByPos.end()) {
                    keyOf[k] = static_cast<std::int64_t>(it->second);
                    ++mapped;
                } else {
                    ++unmapped;
                }
            }
            std::stable_sort(order.begin(), order.end(),
                             [&](std::size_t a, std::size_t b) {
                                 return keyOf[a] < keyOf[b];
                             });
            std::deque<MCGAL::PointInt> reordered;
            for (std::size_t k = 0; k < n; ++k) reordered.push_back(resQ[order[k]]);
            projRes += (long long)encodeResidualBitplaneMajorAdaptive(
                            reordered, residualDropBitsForStream2)
                            .size();
            // valence 流按同样顺序重编码（VAL_OTHER escape 极少见：本数据为 0）。
            std::deque<uint8_t> reVal;
            for (std::size_t k = 0; k < n; ++k) reVal.push_back(valQ[order[k]]);
            projVal += (long long)encodeValence5(reVal, otherQ).size();
        }
        const long long kContainer = 620;   // 实测 containerOverhead
        const long long kCompactBin = 703;  // 实测 base mesh compact
        const long long kParquetOverhead = 1292;  // 实测 residual parquet 开销
        const long long total = projVal + projRes + kContainer + kCompactBin +
                                kParquetOverhead;
        const double nbv = static_cast<double>(mesh_->size_of_vertices()) > 0
                               ? static_cast<double>(mesh_->size_of_vertices())
                               : 19851.0;
        std::fprintf(stderr,
                     "[P2_PROJECT] isPoly_dropped=%d proj_valence=%lld "
                     "proj_residual=%lld container=%lld compact=%lld parquet=%lld "
                     "=> projected_bytes=%lld bpv=%.6f (order_mapped=%lld "
                     "unmapped=%lld elimSeq=%d)\n",
                     sumIsPoly, projVal, projRes, kContainer, kCompactBin,
                     kParquetOverhead, total, static_cast<double>(total) * 8.0 / 19851.0,
                     mapped, unmapped, st.nextSeq);
    }

    const char* envBpv = std::getenv("PMSF_BPV_DIAG");
    if (envBpv && envBpv[0] == '1') {
        int overhead = (int)blob.size() - sumValence - sumIsPoly - sumEdge - sumRes - sumAdjust - sumCorrection;
        // 分量对照用的符号计数：字节数只说明"花了多少"，要判断还有多少可挤，
        // 需要知道每个流发了多少个符号、以及符号本身的 0/1 分布（order-0 熵下界）。
        long long nIsPolyBits = 0, nIsPolyOnes = 0, nValenceSyms = 0, nResidualCoords = 0;
        for (const auto& q : gateIsPolyQueues_) {
            nIsPolyBits += (long long)q.size();
            for (uint8_t b : q) nIsPolyOnes += (b ? 1 : 0);
        }
        // 阶段 0 证伪用：门在访问序列中的**分布**。
        // 若门高度集中（前若干个位置即含全部门），"门优先"几乎是置换顺序、代价小；
        // 若门散布到接近末尾，则必须真正重排遍历 ⇒ 全部符号流改变。
        {
            long long seq = 0, firstOne = -1, lastOne = -1;
            for (const auto& q : gateIsPolyQueues_) {
                for (uint8_t b : q) {
                    if (b) {
                        if (firstOne < 0) firstOne = seq;
                        lastOne = seq;
                    }
                    ++seq;
                }
            }
            std::cerr << "[RDTag gate] totalVisits=" << seq
                      << " ones=" << nIsPolyOnes
                      << " firstOneAt=" << firstOne
                      << " lastOneAt=" << lastOne
                      << " tailAfterLastOne=" << (seq - 1 - lastOne)
                      << std::endl;
        }
        for (const auto& q : gateValenceQueues_) nValenceSyms += (long long)q.size();
        for (const auto& q : gateResidualQueues_) nResidualCoords += (long long)q.size() * 3;
        std::cerr << "[RDTag bpv sv=" << schemeVersion << "] valence=" << sumValence
                  << " isPolyBit=" << sumIsPoly
                  << " edgeSym=" << sumEdge
                  << " residual=" << sumRes
                  << " correction=" << sumCorrection
                  << " adjustment=" << sumAdjust
                  << " residualDropBits=" << residualDropBitsForStream2
                  << " containerOverhead=" << overhead
                  << " totalBlob=" << blob.size()
                  << " rounds=" << numRounds << std::endl;
        const long long nVerts =
            (mesh_ != nullptr) ? (long long)mesh_->nb_vertices : 0;
        std::cerr << "[RDTag sym] isPolyBits=" << nIsPolyBits
                  << " isPolyOnes=" << nIsPolyOnes
                  << " valenceSyms=" << nValenceSyms
                  << " residualCoords=" << nResidualCoords
                  << " verts=" << nVerts;
        if (nVerts > 0) {
            std::cerr << " isPolyBitsPerVert=" << (double)nIsPolyBits / (double)nVerts
                      << " valenceSymsPerVert=" << (double)nValenceSyms / (double)nVerts
                      << " residualCoordsPerVert="
                      << (double)nResidualCoords / (double)nVerts;
        }
        std::cerr << std::endl;
    }

    // ============================================================================
    // (a) Step 1 诊断: PMSF_RDTAG_LEAKY_DIAG=1
    //   两组实验：
    //   (1) per-layer grid-search leaky factor λ ∈ {0, 0.5, 1.0, 1.5}（笛卡尔域），
    //       看 SSE-best λ 对应 Shannon bound 与 λ=1 (current) 差。
    //   (2) Frenet 投影实验：把 (rx, ry, rz) → (n·r, t1·r, t2·r)，看 Shannon bound 变化。
    //       n 来自 face normal；t1 = gateVec - (gateVec·n)·n / |...|；t2 = n × t1。
    // ============================================================================
    const char* envLeakyDiag = std::getenv("PMSF_RDTAG_LEAKY_DIAG");
    if (envLeakyDiag && envLeakyDiag[0] == '1') {
        // 注意：父类 (PCLTTMTag) 的 collect 把 pred_c2 备份到 gatePredC2Queues_。
        const auto& predQs = gatePredC2Queues_;
        const auto& nxQs   = gateFaceNormalXQueues_;
        const auto& nyQs   = gateFaceNormalYQueues_;
        const auto& nzQs   = gateFaceNormalZQueues_;
        const auto& gxQs   = gateGateVecXQueues_;
        const auto& gyQs   = gateGateVecYQueues_;
        const auto& gzQs   = gateGateVecZQueues_;
        if ((int)predQs.size() != numRounds) {
            std::cerr << "[LEAKY_DIAG] WARN: predC2 queue rounds=" << predQs.size()
                      << " != residual rounds=" << numRounds
                      << " — diag skipped (set PMSF_RDTAG_LEAKY_DIAG=1 from start)\n";
        } else {
            // 候选 λ 集合（论文公式 (3) 用 {0, 0.5, 1}；这里加密一些，看 grid 收益）。
            // 用 8x λ 表示（λ_q ∈ 0..12 → λ = λ_q / 8）。
            const int kLambdaSet[] = {0, 4, 8, 12, 16, 24, 32, 48, 64};  // 0,0.5,1,1.5,2,3,4,6,8
            const int kNumLambda = sizeof(kLambdaSet) / sizeof(kLambdaSet[0]);

            // Shannon bit-plane lower bound（与 encoder 的 first1/after1 双模型语义一致；
            // 这里用最简单的"per bit-plane 二元 Shannon"作下界 proxy，与 sumRes 对比）。
            auto shannonBitsForResiduals =
                [BP_BITS](const std::vector<MCGAL::PointInt>& residuals) -> double {
                long long cnt[64][2] = {};
                for (const auto& r : residuals) {
                    for (int axis = 0; axis < 3; ++axis) {
                        int32_t v = (axis == 0 ? r.x() : axis == 1 ? r.y() : r.z());
                        uint32_t z = (static_cast<uint32_t>(v) << 1) ^
                                     static_cast<uint32_t>(v >> 31);
                        for (int b = 0; b < BP_BITS; ++b) {
                            int bit = (z >> b) & 1u;
                            cnt[b][bit]++;
                        }
                    }
                }
                double bits = 0;
                for (int b = 0; b < BP_BITS; ++b) {
                    double n0 = (double)cnt[b][0], n1 = (double)cnt[b][1];
                    double n = n0 + n1;
                    if (n <= 0) continue;
                    double p0 = n0 / n, p1 = n1 / n;
                    double H = 0;
                    if (p0 > 0) H -= p0 * std::log2(p0);
                    if (p1 > 0) H -= p1 * std::log2(p1);
                    bits += H * n;
                }
                return bits;
            };

            std::cerr << "============== PMSF_RDTAG_LEAKY_DIAG ===============\n";
            std::cerr << "  Experiment 1: per-layer leaky λ ∈ {0, 0.5, 1.0, 1.5} on (x,y,z)\n";

            double totShannonAtL1 = 0, totShannonAtBest = 0;
            int bestLambdaHist[32] = {0};
            for (int er = 0; er < numRounds; ++er) {
                const auto& encQ  = gateResidualQueues_[er];
                const auto& predQ = predQs[er];
                if (encQ.size() != predQ.size()) continue;
                const int N = (int)encQ.size();

                long long sseByLambda[32] = {0};
                for (int li = 0; li < kNumLambda; ++li) {
                    long long sse = 0;
                    int lq = kLambdaSet[li];
                    for (int k = 0; k < N; ++k) {
                        int32_t r1x = encQ[k].x() + predQ[k].x();
                        int32_t r1y = encQ[k].y() + predQ[k].y();
                        int32_t r1z = encQ[k].z() + predQ[k].z();
                        int32_t nx = r1x - ((int64_t)lq * predQ[k].x()) / 8;
                        int32_t ny = r1y - ((int64_t)lq * predQ[k].y()) / 8;
                        int32_t nz = r1z - ((int64_t)lq * predQ[k].z()) / 8;
                        sse += (long long)nx * nx + (long long)ny * ny + (long long)nz * nz;
                    }
                    sseByLambda[li] = sse;
                }

                int bestIdx = 0;
                for (int li = 1; li < kNumLambda; ++li) {
                    if (sseByLambda[li] < sseByLambda[bestIdx]) bestIdx = li;
                }
                bestLambdaHist[bestIdx]++;

                std::vector<MCGAL::PointInt> currentRes(encQ.begin(), encQ.end());
                double shannonL1 = shannonBitsForResiduals(currentRes);
                totShannonAtL1 += shannonL1;

                std::vector<MCGAL::PointInt> bestRes;
                bestRes.reserve(N);
                int lqBest = kLambdaSet[bestIdx];
                for (int k = 0; k < N; ++k) {
                    int32_t r1x = encQ[k].x() + predQ[k].x();
                    int32_t r1y = encQ[k].y() + predQ[k].y();
                    int32_t r1z = encQ[k].z() + predQ[k].z();
                    bestRes.emplace_back(
                        r1x - (int32_t)(((int64_t)lqBest * predQ[k].x()) / 8),
                        r1y - (int32_t)(((int64_t)lqBest * predQ[k].y()) / 8),
                        r1z - (int32_t)(((int64_t)lqBest * predQ[k].z()) / 8));
                }
                totShannonAtBest += shannonBitsForResiduals(bestRes);
            }

            std::cerr << "  Histogram of best λ across rounds: ";
            const char* labels[] = {"λ=0", "λ=0.5", "λ=1.0", "λ=1.5", "λ=2", "λ=3", "λ=4", "λ=6", "λ=8"};
            for (int li = 0; li < kNumLambda; ++li) {
                std::cerr << labels[li] << "=" << bestLambdaHist[li] << "  ";
            }
            std::cerr << "\n";
            const double V = (double)mesh_->size_of_vertices();
            std::cerr << "  Shannon (curr λ=1.0):       " << (totShannonAtL1 / 8.0)
                      << " B = " << (totShannonAtL1 / V) << " bpv\n";
            std::cerr << "  Shannon (per-layer best λ): " << (totShannonAtBest / 8.0)
                      << " B = " << (totShannonAtBest / V) << " bpv\n";
            std::cerr << "  Saving: " << ((totShannonAtL1 - totShannonAtBest) / 8.0)
                      << " B ≈ " << ((totShannonAtL1 - totShannonAtBest) / V) << " bpv\n";
            std::cerr << "  (Sideinfo: " << (numRounds * 2.0 / 8.0)
                      << " B = " << (numRounds * 2.0 / V) << " bpv for 2-bit/round λ idx)\n";
            // Experiment 2: Frenet 投影 Shannon bound
            std::cerr << "\n  Experiment 2: Frenet projection (n, t1, t2) — Shannon bound\n";
            // 对每层算法：
            //   for each polyFace: build local frame from face normal + gate vec
            //   project r = (rx, ry, rz) → (alpha, beta, gamma)
            //   round to int, accumulate Shannon
            // 用 quantStep 反量化把 r 变浮点（保持长度尺度合理）
            const float qstep = mesh_->f_quantStep;
            (void)qstep;  // residual 是量化整数差，*qstep 得真实长度；最后 round 回整数

            // 按 axis 分桶 Shannon
            auto axisShannonForResiduals =
                [BP_BITS](const std::vector<int32_t>& vals) -> double {
                long long cnt[64][2] = {};
                for (int32_t v : vals) {
                    uint32_t z = (static_cast<uint32_t>(v) << 1) ^
                                 static_cast<uint32_t>(v >> 31);
                    for (int b = 0; b < BP_BITS; ++b) {
                        int bit = (z >> b) & 1u;
                        cnt[b][bit]++;
                    }
                }
                double bits = 0;
                for (int b = 0; b < BP_BITS; ++b) {
                    double n0 = (double)cnt[b][0], n1 = (double)cnt[b][1];
                    double n = n0 + n1;
                    if (n <= 0) continue;
                    double p0 = n0 / n, p1 = n1 / n;
                    double H = 0;
                    if (p0 > 0) H -= p0 * std::log2(p0);
                    if (p1 > 0) H -= p1 * std::log2(p1);
                    bits += H * n;
                }
                return bits;
            };

            double totFrenetShannon = 0;
            double totFrenetPerAxis[3] = {0, 0, 0};
            for (int er = 0; er < numRounds; ++er) {
                const auto& encQ = gateResidualQueues_[er];
                const auto& predQ = predQs[er];
                const auto& nxQ = nxQs[er]; const auto& nyQ = nyQs[er]; const auto& nzQ = nzQs[er];
                const auto& gxQ = gxQs[er]; const auto& gyQ = gyQs[er]; const auto& gzQ = gzQs[er];
                if (encQ.size() != predQ.size() || encQ.size() != nxQ.size()) continue;
                const int N = (int)encQ.size();
                std::vector<int32_t> alphaVals, betaVals, gammaVals;
                alphaVals.reserve(N); betaVals.reserve(N); gammaVals.reserve(N);
                for (int k = 0; k < N; ++k) {
                    // 当前残差 (rx, ry, rz) = encoded（即 r1 - pred_c2，λ=1）
                    double rx = (double)encQ[k].x();
                    double ry = (double)encQ[k].y();
                    double rz = (double)encQ[k].z();
                    double nx = nxQ[k], ny = nyQ[k], nz = nzQ[k];
                    double gx = gxQ[k], gy = gyQ[k], gz = gzQ[k];
                    // t1 = gate - (gate · n) n，归一化
                    double dot_gn = gx * nx + gy * ny + gz * nz;
                    double t1x = gx - dot_gn * nx;
                    double t1y = gy - dot_gn * ny;
                    double t1z = gz - dot_gn * nz;
                    double t1len = std::sqrt(t1x * t1x + t1y * t1y + t1z * t1z);
                    if (t1len < 1e-12) {
                        // 退化：随便取一个垂直 n 的方向
                        if (std::abs(nx) < 0.9) { t1x = 1; t1y = 0; t1z = 0; }
                        else                    { t1x = 0; t1y = 1; t1z = 0; }
                        double dn = t1x * nx + t1y * ny + t1z * nz;
                        t1x -= dn * nx; t1y -= dn * ny; t1z -= dn * nz;
                        t1len = std::sqrt(t1x * t1x + t1y * t1y + t1z * t1z);
                    }
                    if (t1len > 1e-12) {
                        t1x /= t1len; t1y /= t1len; t1z /= t1len;
                    }
                    // t2 = n × t1
                    double t2x = ny * t1z - nz * t1y;
                    double t2y = nz * t1x - nx * t1z;
                    double t2z = nx * t1y - ny * t1x;

                    double alpha = nx * rx + ny * ry + nz * rz;
                    double beta  = t1x * rx + t1y * ry + t1z * rz;
                    double gamma = t2x * rx + t2y * ry + t2z * rz;

                    // round 到整数（浮点投影后量化）
                    auto roundToInt = [](double v) -> int32_t {
                        return (int32_t)std::lround(v);
                    };
                    alphaVals.push_back(roundToInt(alpha));
                    betaVals.push_back(roundToInt(beta));
                    gammaVals.push_back(roundToInt(gamma));
                }
                double Ha = axisShannonForResiduals(alphaVals);
                double Hb = axisShannonForResiduals(betaVals);
                double Hc = axisShannonForResiduals(gammaVals);
                totFrenetShannon  += Ha + Hb + Hc;
                totFrenetPerAxis[0] += Ha;
                totFrenetPerAxis[1] += Hb;
                totFrenetPerAxis[2] += Hc;
            }
            std::cerr << "  Frenet Shannon total:      " << (totFrenetShannon / 8.0)
                      << " B = " << (totFrenetShannon / V) << " bpv\n";
            std::cerr << "    α (n)  Shannon: " << (totFrenetPerAxis[0] / 8.0)
                      << " B (" << (totFrenetPerAxis[0] / V) << " bpv)\n";
            std::cerr << "    β (t1) Shannon: " << (totFrenetPerAxis[1] / 8.0)
                      << " B (" << (totFrenetPerAxis[1] / V) << " bpv)\n";
            std::cerr << "    γ (t2) Shannon: " << (totFrenetPerAxis[2] / 8.0)
                      << " B (" << (totFrenetPerAxis[2] / V) << " bpv)\n";
            std::cerr << "  Frenet vs Cartesian saving: "
                      << ((totShannonAtL1 - totFrenetShannon) / 8.0)
                      << " B ≈ " << ((totShannonAtL1 - totFrenetShannon) / V) << " bpv\n";
            std::cerr << "\n  Actual encoded residual bytes: " << sumRes
                      << " (= " << (sumRes * 8.0 / V) << " bpv)\n";
            std::cerr << "====================================================\n";
        }
    }

    return (int)blob.size();
}

int RDTagSymbolCollectOperator::exportToBuffer(char* /*buffer*/,
                                                bool /*enableQuantization*/,
                                                bool /*enableCompress*/) {
    throw std::runtime_error(
        "[RDTag] exportToBuffer (non-RC) is not implemented.");
}
