// SPDX-License-Identifier: Apache-2.0
//
// MMSEDualRingSymbolCollectOperator.cpp — MMSE dual-ring protocol (BlobLayout=22).
//
// The default encoder reuses the proven RDTag/PCLTTM deterministic fan topology
// and writes a compact, MMSE-labelled layout with fixed dual-ring residuals.
// A paper-aligned per-face mode decision path (dual-ring alpha={0,0.5,1} and
// MMSE) is available behind PMSF_MMSE_USE_MODE_DECISION=1 for experiments.

#include "common/ValenceStreamCodec.h"
#include "common/AlphabetStreamCodec.h"  // pmsf::encodeAlphabetStream（P11 抽取）  // pmsf::encodeValence5（P3 抽取）

using pmsf::encodeValence5;
#include "MMSEDualRingSymbolCollectOperator.h"
#include "common/PaperLayoutAliases.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂
#include "core/PMSFBoundarySideChannel.h"
#include "MMSE/boundary/MMSEPMSFBoundary.h"
#include "PMSFPayloadAdapterRegistry.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include "BFSVersionManager.h"
#include "compress/ConnectivityBlobContainer.h"
#include "compress/RangeCoderStreamEncoder.h"
#include "Configuration.h"
#include "ContextPool.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Point.h"
#include "Vertex.h"
#include "predict/MMSEPredictor.h"
#include "RD/compress/RDSymbolCollectOperator.h"

#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"

#include <algorithm>
#include <array>
#include <fstream>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

constexpr int kBufScratch = BUFFER_SIZE / 12;

bool mmseEnvEnabled(const char* name) {
    return pmsf::pmsfStorageEnvEnabled(name);
}

void requireMMSEPaperExactIfRequested() {
    if (!pmsf::boundaryFlagOn(pmsf::mmse_boundary::kPaperExactSpec)) {
        return;
    }
    throw std::runtime_error(
        "[MMSE-DR] requested paper-exact MMSE replay, but the current encoder "
        "uses an RD/PCLTTM-compatible topology bridge with MMSE residual "
        "metadata; exact paper mode decision and strict dual-ring replay are "
        "not implemented.");
}

// ---- stream 0: valence (5-元 + 4-context + V_OTHER escape) ----
// Identical to RDTag's encoder.



int32_t signedResidualDrop(int32_t v, int dropBits) {
    if (dropBits <= 0) return v;
    if (dropBits >= 30) return v < 0 ? -1 : 0;
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

// sv=3 stream 2: xyz-specific bit widths + bit-plane-major adaptive RC.
//
// P1 (2026-10-22): when the MMSE paper predictor stack is enabled
// (`predModes != nullptr`), a self-describing per-cluster 2-bit mode stream is
// appended right after the existing 8-byte residual header:
//   [i32 numCoords | flag 0x20000000][u8 b0][u8 b1][u8 b2][u8 drop]
//   [i32 numClusters][i32 clusterSize][packed 2-bit modes][bit-plane RC bytes]
// The decoder (MMSEDualRingSymbolReadOperator::importFromRangeCodedBuffer)
// detects the flag and replays the same mode sequence through the predictor
// hook in src/predict/RDPredictor.h::rdCurvaturePrediction().
std::vector<uint8_t> encodeResidualBPAdaptive(
    const std::deque<MCGAL::PointInt>& residuals,
    int residualDropBits,
    const std::vector<uint8_t>* predModes = nullptr,
    int predClusterSize = 0) {
    const size_t N = residuals.size();
    const int bits[3] = {
        bitWidthForResidualComponent(residuals, 0, residualDropBits),
        bitWidthForResidualComponent(residuals, 1, residualDropBits),
        bitWidthForResidualComponent(residuals, 2, residualDropBits),
    };
    const int offsets[3] = {0, bits[0], bits[0] + bits[1]};
    const int numCtx = bits[0] + bits[1] + bits[2];

    if (const char* envStats = std::getenv("PMSF_RESIDUAL_STATS");
        envStats && envStats[0] == '1') {
        size_t zeroVec = 0;
        size_t zeroAxis[3] = {0, 0, 0};
        for (const auto& r : residuals) {
            const int32_t x = signedResidualDrop(r.x(), residualDropBits);
            const int32_t y = signedResidualDrop(r.y(), residualDropBits);
            const int32_t z = signedResidualDrop(r.z(), residualDropBits);
            if (x == 0 && y == 0 && z == 0) ++zeroVec;
            if (x == 0) ++zeroAxis[0];
            if (y == 0) ++zeroAxis[1];
            if (z == 0) ++zeroAxis[2];
        }
        std::cerr << "[MMSE-DR residual stats] count=" << N
                  << " bits=" << bits[0] << "," << bits[1] << "," << bits[2]
                  << " zeroVec=" << zeroVec
                  << " zeroAxis=" << zeroAxis[0] << "," << zeroAxis[1]
                  << "," << zeroAxis[2]
                  << " residualDropBits=" << residualDropBits << std::endl;
    }

    std::vector<uint8_t> out;
    int32_t numCoords = static_cast<int32_t>(N * 3);
    // Self-describing flag: high bit of numCoords selects the axis-conditional
    // bitplane models; decoder masks it before using the count.
    const char* axisCtxEnv = std::getenv("PMSF_MMSE_RESID_AXIS_CTX");
    const bool axisCtxEnabled = axisCtxEnv && *axisCtxEnv && axisCtxEnv[0] != '0';
    const char* hashCtxEnv = std::getenv("PMSF_MMSE_RESID_HASH_CTX");
    const bool hashCtxEnabled = hashCtxEnv && *hashCtxEnv && hashCtxEnv[0] != '0';
    const bool predStream = predModes != nullptr && !predModes->empty() &&
                            predClusterSize > 0;
    if (axisCtxEnabled) {
        numCoords = static_cast<int32_t>(
            static_cast<uint32_t>(numCoords) | 0x80000000u);
    }
    if (hashCtxEnabled) {
        numCoords = static_cast<int32_t>(
            static_cast<uint32_t>(numCoords) | 0x40000000u);
    }
    if (predStream) {
        numCoords = static_cast<int32_t>(
            static_cast<uint32_t>(numCoords) | 0x20000000u);
    }
    out.resize(8);
    std::memcpy(out.data(), &numCoords, 4);
    out[4] = static_cast<uint8_t>(bits[0]);
    out[5] = static_cast<uint8_t>(bits[1]);
    out[6] = static_cast<uint8_t>(bits[2]);
    out[7] = static_cast<uint8_t>(std::clamp(residualDropBits, 0, 30));
    if (predStream) {
        // Compact mode section:
        //   [u8 modeSpec]                     modeSpec = 0..3  => fixed mode
        //                                     modeSpec = 0xFF  => per-cluster table
        //   if 0xFF: [u16 clusterSize][packed 2-bit modes]
        // A per-cluster round with a single cluster is byte-equivalent to the
        // fixed form (all polyFaces use modes[0]) and is written as fixed.
        if (predModes->size() == 1) {
            out.push_back(static_cast<uint8_t>((*predModes)[0] & 0x3u));
        } else {
            out.push_back(0xFFu);
            const std::uint16_t cs = static_cast<std::uint16_t>(
                std::clamp(predClusterSize, 1, 65535));
            const size_t csOffset = out.size();
            out.resize(csOffset + 2);
            std::memcpy(out.data() + csOffset, &cs, 2);
            const size_t modeBytes = (predModes->size() + 3) / 4;
            const size_t modesOffset = out.size();
            out.resize(modesOffset + modeBytes, 0);
            for (size_t c = 0; c < predModes->size(); ++c) {
                const std::uint8_t mode =
                    static_cast<std::uint8_t>((*predModes)[c] & 0x3u);
                out[modesOffset + (c >> 2)] |= static_cast<std::uint8_t>(
                    mode << (2 * static_cast<unsigned>(c & 3)));
            }
        }
        if (const char* envPred = std::getenv("PMSF_BPV_DIAG");
            envPred && envPred[0] == '1') {
            std::cerr << "[MMSE-DR pred] modes=" << predModes->size()
                      << " clusterSize=" << predClusterSize << std::endl;
        }
    }
    if (N == 0 || numCtx == 0) return out;

    std::vector<int> dx(N), dy(N), dz(N);
    for (size_t i = 0; i < N; ++i) {
        dx[i] = signedResidualDrop(residuals[i].x(), residualDropBits);
        dy[i] = signedResidualDrop(residuals[i].y(), residualDropBits);
        dz[i] = signedResidualDrop(residuals[i].z(), residualDropBits);
    }

    // Diagnostic: dump signed residuals per round for offline predictor/transform
    // entropy analysis (env-gated; no effect on the produced stream).

    const size_t bufSize = std::max<size_t>(BUFFER_SIZE / 12, N * 12 + 1024);
    std::unique_ptr<char[]> buf(new char[bufSize]);
    size_t rcSize = pmsf::encodeResidualBitPlaneQueueAdaptivePlaneMajor(
        dx, dy, dz, buf.get(), bufSize);

    const size_t headerBytes = out.size();
    out.resize(headerBytes + rcSize);
    std::memcpy(out.data() + headerBytes, buf.get(), rcSize);
    return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// P1 (2026-10-22, WS-B4): MMSE paper predictor stack post-pass.
//
// Called immediately after the RDTag/PCLTTM-compatible collection of one round.
// At this point:
//   * gateResidualQueues_.back()[k] holds the framework's raw first-order
//     residual r1 = v_quant - barycenter_int (the PCLTTM second pass rewrote it
//     to r1 - pred_c2, so recompute from the face instead of the queue);
//   * gateFacetQueues_.back()[k] is the polyFace (gate) whose removed vertex is
//     encoded by residual slot k — same BFS order as the decoder's replay.
// We overwrite each residual slot with `r1 - correction[mode]` and record the
// mode id stream (one 2-bit id per cluster of `PMSF_MMSE_CLUSTER` polyFaces).
// ---------------------------------------------------------------------------
void MMSEDualRingSymbolCollectOperator::applyMMSEPredictorStack() {
    // Keep gatePredModeQueues_ aligned one-entry-per-encoder-round even for
    // empty rounds (the export path indexes it by encoder round).
    if (gateResidualQueues_.empty() || gateFacetQueues_.empty()) {
        gatePredModeQueues_.emplace_back();
        gatePredClusterSizes_.push_back(0);
        return;
    }
    auto& resQueue = gateResidualQueues_.back();
    auto& facetQueue = gateFacetQueues_.back();
    const std::size_t n = std::min(resQueue.size(), facetQueue.size());
    if (n == 0) {
        gatePredModeQueues_.emplace_back();
        gatePredClusterSizes_.push_back(0);
        return;
    }

    const bool lsOn = pmsf::mmse_pred::lsEnabledFromEnv();
    std::vector<std::uint8_t> modes;
    int clusterSize = 0;

    auto correctionFor = [&](MCGAL::Facet* f, int mode) -> MCGAL::PointInt {
        return (mode == pmsf::mmse_pred::kModeBarycentric)
                   ? MCGAL::PointInt(0, 0, 0)
                   : pmsf::mmse_pred::correctionForMode(f, mode);
    };

    if (predFixedMode_ >= 0) {
        // Fixed mode ablation: one mode for the whole round.
        clusterSize = static_cast<int>(n);
        modes.assign(1, static_cast<std::uint8_t>(predFixedMode_));
        for (std::size_t k = 0; k < n; ++k) {
            MCGAL::Facet* f = facetQueue[k];
            if (f == nullptr) continue;
            const MCGAL::PointInt r1 = f->getRemovedVertexPosInt();
            const MCGAL::PointInt corr = correctionFor(f, predFixedMode_);
            resQueue[k] = MCGAL::PointInt(r1.x() - corr.x(),
                                          r1.y() - corr.y(),
                                          r1.z() - corr.z());
        }
    } else {
        // Per-cluster mode decision (paper §2.3 eq.(9)): pick the mode that
        // minimizes the sum of squared prediction residuals over the cluster.
        // Cluster = consecutive polyFaces in BFS order, threshold
        // PMSF_MMSE_CLUSTER (deterministic and decodable without side info).
        clusterSize = pmsf::mmse_pred::clusterSizeFromEnv(32);
        const std::size_t chunk = static_cast<std::size_t>(clusterSize);
        for (std::size_t begin = 0; begin < n; begin += chunk) {
            const std::size_t end = std::min(n, begin + chunk);
            const std::size_t m = end - begin;
            double err[pmsf::mmse_pred::kModeCount] = {0.0, 0.0, 0.0, 0.0};
            std::vector<MCGAL::PointInt> r1s(m);
            std::vector<MCGAL::PointInt> corrs(m * pmsf::mmse_pred::kModeCount);
            for (std::size_t i = 0; i < m; ++i) {
                MCGAL::Facet* f = facetQueue[begin + i];
                MCGAL::PointInt r1(0, 0, 0);
                MCGAL::PointInt cand[pmsf::mmse_pred::kModeCount];
                if (f != nullptr) {
                    r1 = f->getRemovedVertexPosInt();
                    pmsf::mmse_pred::correctionsForFace(f, lsOn, cand);
                } else {
                    for (int k = 0; k < pmsf::mmse_pred::kModeCount; ++k) {
                        cand[k] = MCGAL::PointInt(0, 0, 0);
                    }
                }
                r1s[i] = r1;
                for (int k = 0; k < pmsf::mmse_pred::kModeCount; ++k) {
                    corrs[i * pmsf::mmse_pred::kModeCount + k] = cand[k];
                    const double dx = static_cast<double>(r1.x() - cand[k].x());
                    const double dy = static_cast<double>(r1.y() - cand[k].y());
                    const double dz = static_cast<double>(r1.z() - cand[k].z());
                    err[k] += dx * dx + dy * dy + dz * dz;
                }
            }
            int best = 0;
            for (int k = 1; k < pmsf::mmse_pred::kModeCount; ++k) {
                if (err[k] < err[best]) best = k;
            }
            modes.push_back(static_cast<std::uint8_t>(best));
            for (std::size_t i = 0; i < m; ++i) {
                const MCGAL::PointInt& c =
                    corrs[i * pmsf::mmse_pred::kModeCount + best];
                const MCGAL::PointInt& r1 = r1s[i];
                resQueue[begin + i] = MCGAL::PointInt(r1.x() - c.x(),
                                                      r1.y() - c.y(),
                                                      r1.z() - c.z());
            }
        }
    }

    if (const char* env = std::getenv("PMSF_BPV_DIAG"); env && env[0] == '1') {
        int hist[pmsf::mmse_pred::kModeCount] = {0, 0, 0, 0};
        for (std::uint8_t md : modes) {
            if (md < pmsf::mmse_pred::kModeCount) ++hist[md];
        }
        std::cerr << "[MMSE-DR predictor] residuals=" << n
                  << " clusters=" << modes.size()
                  << " clusterSize=" << clusterSize
                  << " fixedMode=" << predFixedMode_
                  << " modeHist(bary,dr05,dr1,ls)=" << hist[0] << "," << hist[1]
                  << "," << hist[2] << "," << hist[3] << std::endl;
    }

    gatePredModeQueues_.push_back(std::move(modes));
    gatePredClusterSizes_.push_back(static_cast<std::int32_t>(clusterSize));
}

void MMSEDualRingSymbolCollectOperator::collect(MCGAL::Halfedge* seed) {
    static const std::vector<MCGAL::Halfedge*> kEmpty;
    collect(seed, kEmpty);
}

void MMSEDualRingSymbolCollectOperator::collect(std::vector<MCGAL::Halfedge*> seeds) {
    if (seeds.empty()) {
        throw std::runtime_error("[MMSE-DR] collect(vector<seed>) called with empty seeds");
    }
    static const std::vector<MCGAL::Halfedge*> kEmpty;
    collect(seeds[0], kEmpty);
}

void MMSEDualRingSymbolCollectOperator::collect(
    MCGAL::Halfedge* seed,
    const std::vector<MCGAL::Halfedge*>& gates) {
    paperTableCache_.clear();
    // P8 / D1（sv 日落）：门禁路径只走 V1 —— 本算子**委托父类** RDTag 完成收集。
    // 原双环 collect body 属 V2/V4 等 env 门控分支（`PMSF_MMSE_USE_MODE_DECISION`
    // / `PMSF_MMSE_USE_FIXED_DUAL_RING`）；门禁配置两者皆未设置，故原先就从这个
    // early-return 分支出去 ⇒ 本条**就是**门禁路径。
    // 实测锚点：产物 compact+residual = 83038，与冻结基线 mmse,bunny 精确相符。
    RDTagSymbolCollectOperator::collect(seed, gates);

    // ------------------------------------------------------------------
    // P1 (2026-10-22, WS-B4): MMSE paper predictor stack, env-gated.
    //
    // After the RDTag/PCLTTM-compatible collection (which stores the plain
    // barycentric r1 = v - barycenter in the residual queue), rewrite every
    // polyFace residual with the paper's predicted geometry:
    //     encoded = r1 - (xhat_mode - u_int)
    // and record one 2-bit mode id per cluster.  `PMSF_MMSE_PREDICTOR` selects
    // the mode: bary/dr0, dr05, dr1, ls, or "mode" (per-cluster decision over
    // all four paper modes).  Default off => byte-identical canonical artifact.
    // ------------------------------------------------------------------
    predFixedMode_ = pmsf::mmse_pred::predictorModeFromEnv();
    if (predFixedMode_ != -2) {
        pmsf::mmse_pred::setQuantContext(mesh_->bboxMin, mesh_->f_quantStep);
        applyMMSEPredictorStack();
        pmsf::mmse_pred::clearQuantContext();
    }
}

const pmsf::PMSFSymbolTable* MMSEDualRingSymbolCollectOperator::pmsfSymbolTable() const {
    const pmsf::PMSFSymbolTable* base = RDTagSymbolCollectOperator::pmsfSymbolTable();
    if (!base || base->empty()) {
        return nullptr;
    }
    const std::int32_t strictStorageRequested =
        static_cast<std::int32_t>(mmseEnvEnabled("PMSF_MMSE_STRICT") ? 1 : 0);
    const std::int32_t modeDecisionPresent =
        static_cast<std::int32_t>(gateModeQueues_.empty() ? 0 : 1);
    pmsf::copyPMSFSymbolTableWithIndexedRowAugmentation(
        base,
        paperTableCache_,
        [&](std::size_t r, pmsf::PMSFSymbolRecord& row) {
        pmsf::appendMMSEStorageContextSideChannels(
            row.delta,
            static_cast<float>(alpha_),
            static_cast<std::int32_t>(fixedDualRingPrediction_ ? 1 : 0),
            static_cast<std::int32_t>(clusterModeDecision_ ? 1 : 0),
            strictStorageRequested);
        pmsf::appendMMSEBoundaryCapabilitySideChannels(
            row.delta,
            static_cast<std::int32_t>(fixedDualRingPrediction_ ? 1 : 0),
            modeDecisionPresent);
        if (r < gateModeQueues_.size()) {
            pmsf::replaceMMSEPredictionModeSymbols(
                row.delta, gateModeQueues_[r]);
        }
    });
    return paperTableCache_.empty() ? nullptr : &paperTableCache_;
}

const pmsf::PMSFSymbolPayloadTable*
MMSEDualRingSymbolCollectOperator::paperSymbolPayloadTable() const {
    const bool strict = mmseEnvEnabled("PMSF_MMSE_STRICT");
    if (paperPayloadTableCacheRoundCount_ == gateValenceQueues_.size() &&
        paperPayloadTableCacheStrict_ == strict) {
        return paperPayloadTableCache_.empty() ? nullptr : &paperPayloadTableCache_;
    }
    paperPayloadTableCache_.clear();
    paperPayloadTableCacheRoundCount_ = gateValenceQueues_.size();
    paperPayloadTableCacheStrict_ = strict;
    if (gateValenceQueues_.empty()) {
        return nullptr;
    }

    std::vector<char> scratch(BUFFER_SIZE);
    auto* self = const_cast<MMSEDualRingSymbolCollectOperator*>(this);
    const int bytes = self->exportToBufferWithRangeCoder(scratch.data(), true);
    if (bytes <= 0) {
        return nullptr;
    }

    paperPayloadTableCache_ = pmsf::buildMMSEPayloadTable(
        scratch.data(), static_cast<std::size_t>(bytes), strict);
    return &paperPayloadTableCache_;
}

int MMSEDualRingSymbolCollectOperator::exportToBufferWithRangeCoder(
    char* buffer, bool isEnableQuantization) {
    if (buffer == nullptr) return 0;
    if (!enablePrediction_ || !isEnableQuantization) {
        throw std::runtime_error(
            "[MMSE-DR] exportRC requires enablePrediction && enableQuantization.");
    }
    requireMMSEPaperExactIfRequested();

    const int numRounds = (int)gateValenceQueues_.size();
    if ((int)gateIsPolyQueues_.size()   != numRounds ||
        (int)gateResidualQueues_.size() != numRounds) {
        throw std::runtime_error(
            "[MMSE-DR] inconsistent per-round queue sizes");
    }

    // Strict paper-aligned path: single {N,3,4,5,6} stream + order-0 AC
    const char* envStrict = std::getenv("PMSF_MMSE_STRICT");
    if (envStrict && envStrict[0] == '1') {
        constexpr int kBlobLayoutStrict = pmsf::kPaperBlobLayoutMMSEStrict;
        constexpr int kSchemeVerStrict = pmsf::kPaperBlobSchemeVersionMMSEStrict;

        // Merge isPolyBit + valence into {N,3,4,5,6}
        std::vector<std::deque<uint8_t>> strictCodes(numRounds);
        int totalCodes = 0, totalPoly = 0;
        for (int i = 0; i < numRounds; ++i) {
            auto& ip = gateIsPolyQueues_[i];
            auto& vc = gateValenceQueues_[i];
            size_t pi = 0;
            for (uint8_t b : ip) {
                if (b) {
                    uint8_t v8 = (pi < vc.size()) ? vc[pi] : 0;
                    uint8_t sc;
                    switch ((RDValenceCode)v8) {
                        case RDValenceCode::VAL_3: sc=3; break;
                        case RDValenceCode::VAL_4: sc=4; break;
                        case RDValenceCode::VAL_5: sc=5; break;
                        case RDValenceCode::VAL_6: sc=6; break;
                        default: sc=0; break;
                    }
                    strictCodes[i].push_back(sc);
                    pi++; totalPoly++;
                } else {
                    strictCodes[i].push_back(0);
                }
            }
            totalCodes += (int)strictCodes[i].size();
        }

        // Encode codes: 5-ary order-0 AC
        std::unique_ptr<char[]> cb(new char[BUFFER_SIZE/8]); size_t cs = 0;
        // strictCodes 中符号范围 {0,3,4,5,6}，映射到 {0,1,2,3,4} 给 5 元 model。
        {
            std::vector<std::deque<uint8_t>> mapped(numRounds);
            for (int i = 0; i < numRounds; ++i) {
                for (uint8_t c : strictCodes[i]) {
                    mapped[i].push_back(c == 0 ? 0 : static_cast<uint8_t>(c - 2));
                }
            }
            cs = pmsf::encodeUint8Queues(
                mapped, 5, true, cb.get(), BUFFER_SIZE / 8);
        }

        // Encode residuals: ZigZag bit-plane RC (same as existing)
        std::unique_ptr<char[]> rb(new char[BUFFER_SIZE/3]); size_t rs = 0;
        {
            const int bp = static_cast<int>(mesh_->i_nbQuantBits) + 1;
            rs = pmsf::encodeResidualBitPlaneQueues(
                gateResidualQueues_, bp, true, rb.get(), BUFFER_SIZE / 3);
        }

        // Serialize flat layout
        size_t o=0;
        std::memcpy(buffer+o,"MMSE",4); o+=4;
        writeInt(buffer,o,kBlobLayoutStrict); writeInt(buffer,o,kSchemeVerStrict);
        writeInt(buffer,o,numRounds); writeInt(buffer,o,totalCodes);
        writeInt(buffer,o,(int)cs); writeInt(buffer,o,totalPoly); writeInt(buffer,o,(int)rs);
        for (int i=numRounds-1;i>=0;--i) writeInt(buffer,o,(int)strictCodes[i].size());
        writeCharPointer(buffer,o,cb.get(),(int)cs); writeCharPointer(buffer,o,rb.get(),(int)rs);

        const char* ev=std::getenv("PMSF_BPV_DIAG");
        if(ev&&ev[0]=='1') std::cerr<<"[MMSE-Strict bpv] codes="<<cs<<" raw="<<totalCodes<<" res="<<rs<<" coords="<<totalPoly*3<<" rds="<<numRounds<<std::endl;
        return (int)o;
    }

    constexpr int kBlobLayoutMMSEDR = pmsf::kPaperBlobLayoutMMSEDualRing;
    // P8 / D1（sv 日落）：门禁路径 = **V3**（valence + isPolyBit + adaptive residual）。
    // 依据：① `impl/common/RetriOverride.h:775-786` 的 MMSE_DualRing 分支在本进程内
    // 无条件 `setenv("PMSF_RDTAG_ADAPTIVE_RESIDUAL","1")`；② 实测 PMSF_BPV_DIAG 打印
    // `sv=3`，且产物 83038 与冻结基线 mmse,bunny 精确相符。
    // V1/V2/V4/V5/V6/V7/V8 的判定与发射块（useModeDecision / fixedDualRingPrediction_ /
    // useIsPolyContext / useGlobalStreams / useMixedStreams）在门禁下恒 false ⇒ 全部删除。
    constexpr int kSchemeVersion = pmsf::kPaperBlobSchemeVersionMMSEDualRingV3;
    // ⚠️ `PMSF_RDTAG_RESIDUAL_DROP` 是**诊断/监控旋钮**，不是 sv 分支 ⇒ 保留
    // （硬约束 2：禁止删监控行）。它仍进入 BPV_DIAG 打印，并作为 V3 残差的丢弃位数。
    int residualDropBits = 0;
    if (const char* envDrop = std::getenv("PMSF_RDTAG_RESIDUAL_DROP")) {
        residualDropBits = std::clamp(std::atoi(envDrop), 0, 8);
    }
    pmsf::ConnectivityBlobBuilder cbb(
        pmsf::mmseDualRingConnectivityBlobSpec(
            numRounds,
            kSchemeVersion,
            // V3 无 mode 流（原为 useModeDecision ? WithMode : plain，恒取 plain）。
            pmsf::kPaperBlobStreamCountMMSEDualRing));

    int sumValence = 0, sumIsPoly = 0, sumMode = 0, sumRes = 0;
    // P16 诊断（2026-09-25）：把每流的「符号数 / RC 载荷 / 逐流固定头」拆开，
    // 回答「字节花在熵上还是花在每轮固定开销上」。只读计数，不改码流。
    long diagValSyms = 0, diagIsPolySyms = 0;
    long diagValRC = 0, diagIsPolyRC = 0;
    const int BP_BITS = (int)mesh_->i_nbQuantBits + 1;

    for (int i = 0; i < numRounds; ++i) {
        int er = numRounds - 1 - i;

        auto rcVal = encodeValence5(gateValenceQueues_[er],
                                     gateOtherValQueues_[er]);
        sumValence += (int)rcVal.size();
        diagValSyms += (long)gateValenceQueues_[er].size();
        diagValRC += (long)rcVal.size() - 8;  // 8 = numPoly/numOther 两个 i32 头
        cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamMMSEDualRingValence, rcVal);

        // V3 用 2 元 RC（原为 useIsPolyContext ? multiContext : bit，恒取 bit）。
        // P16：把本轮的 isPoly / valence 符号序列 dump 出来，供 python 侧
        // 做多阶上下文熵分析（先量后改）。仅 env 门控，不触碰码流。
        auto rcIsPoly = pmsf::encodeAlphabetStream(gateIsPolyQueues_[er], 2);
        sumIsPoly += (int)rcIsPoly.size();
        diagIsPolySyms += (long)gateIsPolyQueues_[er].size();
        diagIsPolyRC += (long)rcIsPoly.size() - 4;  // 4 = 单个 i32 头
        cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamMMSEDualRingIsPoly, rcIsPoly);

        // P8 / D1（sv 日落）：本条循环只剩 V3 发射。
        // 原 `if (useModeDecision) {...}`（V2 的 mode 流）删除 —— 该判定在门禁下恒 false。
        const int residualStream = pmsf::kPaperBlobStreamMMSEDualRingResidual;
        // 原为 useAdaptiveResidual ? A : B，恒取 A（框架在本进程内无条件 setenv
        // PMSF_RDTAG_ADAPTIVE_RESIDUAL=1，见 RetriOverride.h:775-786）⇒ 折叠为 A。
        // P1：MMSE paper predictor stack — per-cluster 2-bit mode stream rides
        // inside the residual stream (self-describing flag), so the container
        // layout and the frozen canonical artifact form stay untouched when
        // PMSF_MMSE_PREDICTOR is unset.
        const std::vector<uint8_t>* predModes = nullptr;
        int predClusterSize = 0;
        if (static_cast<std::size_t>(er) < gatePredModeQueues_.size()) {
            predModes = &gatePredModeQueues_[static_cast<std::size_t>(er)];
            predClusterSize = (static_cast<std::size_t>(er) < gatePredClusterSizes_.size())
                                  ? static_cast<int>(gatePredClusterSizes_[static_cast<std::size_t>(er)])
                                  : 0;
            if (predModes->empty()) predModes = nullptr;
        }
        auto rcRes = encodeResidualBPAdaptive(gateResidualQueues_[er], residualDropBits,
                                              predModes, predClusterSize);
        sumRes += (int)rcRes.size();
        cbb.appendStreamBytes(i, residualStream, rcRes);
    }

    auto blob = cbb.finalize();
    std::memcpy(buffer, blob.data(), blob.size());

    const char* envBpv = std::getenv("PMSF_BPV_DIAG");
    if (envBpv && envBpv[0] == '1') {
        int overhead = (int)blob.size() - sumValence - sumIsPoly - sumMode - sumRes;
        std::cerr << "[MMSE-DR bpv sv=" << kSchemeVersion << "] valence=" << sumValence
                  << " isPolyBit=" << sumIsPoly
                  << " mode=" << sumMode
                  << " residual=" << sumRes
                  << " residualDropBits=" << residualDropBits
                  << " containerOverhead=" << overhead
                  << " totalBlob=" << blob.size()
                  << " rounds=" << numRounds << std::endl;
        // P16：逐流的「符号数 / 载荷 / 固定头」三分
        const long nR = (numRounds > 0 ? numRounds : 1);
        std::cerr << "[MMSE-DR split] valence: syms=" << diagValSyms
                  << " rcPayload=" << diagValRC
                  << " hdr=" << (sumValence - diagValRC)
                  << " bitsPerSym="
                  << (diagValSyms > 0 ? (double)diagValRC * 8.0 / (double)diagValSyms
                                      : 0.0)
                  << "\n[MMSE-DR split] isPoly : syms=" << diagIsPolySyms
                  << " rcPayload=" << diagIsPolyRC
                  << " hdr=" << (sumIsPoly - diagIsPolyRC)
                  << " bitsPerSym="
                  << (diagIsPolySyms > 0
                          ? (double)diagIsPolyRC * 8.0 / (double)diagIsPolySyms
                          : 0.0)
                  << "\n[MMSE-DR split] 每轮流头: valence="
                  << (double)(sumValence - diagValRC) / (double)nR
                  << " isPoly=" << (double)(sumIsPoly - diagIsPolyRC) / (double)nR
                  << " B/round\n";
    }
    return (int)blob.size();
}

int MMSEDualRingSymbolCollectOperator::exportToBuffer(char*, bool, bool) {
    throw std::runtime_error("[MMSE-DR] exportToBuffer (non-RC) not implemented.");
}
