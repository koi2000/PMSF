// SPDX-License-Identifier: Apache-2.0
//
// PCLTTMStrictSymbolCollectOperator.cpp

#include "common/strict/PCLTTMStrictSymbolCollectOperator.h"
#include "common/PaperLayoutAliases.h"
#include "PCLTTM/boundary/PCLTTMPMSFBoundary.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include "Configuration.h"
#include "compress/ConnectivityBlobContainer.h"
#include "compress/RangeCoderStreamEncoder.h"
#include "BufferUtils.h"
#include "Mesh.h"
#include "common/patch/RDPMSFSymbolMapper.h"
#include "Vertex.h"
#include "common/patch/RDPatchDecoderConquest.h"

#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <functional>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include <unistd.h>

namespace pmsf {

namespace {

enum : uint8_t {
    kSymNull = 0,
    kSymV3 = 1,
    kSymV4 = 2,
    kSymV5 = 3,
    kSymV6 = 4,
};

int ceilLog2Positive(int v) {
    int lg = 0;
    int p = 1;
    while (p < v) {
        p <<= 1;
        ++lg;
    }
    return lg;
}

bool envOn(const char* name) {
    return pmsf::pmsfStorageEnvEnabled(name);
}

std::int32_t envFlagI32(const char* name) {
    return envOn(name) ? 1 : 0;
}

void requirePCLTTMStrictPaperExactIfRequested() {
    if (!pmsf::boundaryFlagOn(pmsf::pclttm_boundary::kPaperExactSpec)) {
        return;
    }
    throw std::runtime_error(
        "[PCLTTMStrict] requested paper-exact replay, but the current strict "
        "path is still an experimental patch-conquest bridge; exact topology "
        "and geometry replay are not implemented.");
}

void resetRoundLocalMeshState(std::shared_ptr<MCGAL::Mesh> mesh) {
    mesh->resetState();
    mesh->invalidateAllVertexDegrees();
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (v && !v->isRemoved()) v->clearTag();
    }
}

std::shared_ptr<MCGAL::Mesh> cloneMeshForStrictReplay(
    const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!mesh) return nullptr;
    char path[] = "/tmp/pmsf_pclttm_strict_base_XXXXXX.off";
    const int fd = mkstemps(path, 4);
    if (fd < 0) {
        throw std::runtime_error("[PCLTTMStrict] cannot create temporary base mesh");
    }
    close(fd);
    mesh->dumpto_oldtype(path);
    auto copy = std::make_shared<MCGAL::Mesh>();
    if (!copy->loadOFF(path)) {
        std::remove(path);
        throw std::runtime_error("[PCLTTMStrict] cannot reload temporary base mesh");
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

uint8_t strictSymbolFromPatch(const RDPatchSymbol& s, bool cleaningLayer) {
    switch (s.kind) {
        case RDPatchSymbolKind::NULL_PATCH:
            return kSymNull;
        case RDPatchSymbolKind::V_3:
            // Paper §3.7: cleaning-layer valence-3 codes are substituted by 6
            // before arithmetic coding to keep the occurrence peak at valence 6.
            return cleaningLayer ? kSymV6 : kSymV3;
        case RDPatchSymbolKind::V_4:
            return kSymV4;
        case RDPatchSymbolKind::V_5:
            return kSymV5;
        case RDPatchSymbolKind::V_6:
            return kSymV6;
        case RDPatchSymbolKind::V_OTHER:
            throw std::runtime_error(
                "[PCLTTMStrict] V_OTHER cannot appear in strict PCLTTM storage; "
                "valence > 6 must be encoded as NULL_PATCH.");
    }
    throw std::runtime_error("[PCLTTMStrict] unknown patch symbol");
}

int patchSymbolValence(const RDPatchSymbol& s) {
    switch (s.kind) {
        case RDPatchSymbolKind::V_3: return 3;
        case RDPatchSymbolKind::V_4: return 4;
        case RDPatchSymbolKind::V_5: return 5;
        case RDPatchSymbolKind::V_6: return 6;
        case RDPatchSymbolKind::V_OTHER:
            return s.escape_valence > 0 ? static_cast<int>(s.escape_valence) : 7;
        case RDPatchSymbolKind::NULL_PATCH:
            return 0;
    }
    return 0;
}

std::vector<uint8_t> encodeStrictConnectivity(const std::vector<uint8_t>& syms,
                                              int numNullBefore,
                                              int numNullAfter,
                                              int numRemoved,
                                              int numA,
                                              int numB,
                                              const std::vector<std::pair<int32_t, int32_t>>& layerPackedCounts,
                                              const std::vector<std::array<int, 6>>& layerSeedQs) {
    if (layerSeedQs.size() != layerPackedCounts.size()) {
        throw std::runtime_error("[PCLTTMStrict] layer seed/count mismatch");
    }
    std::deque<uint8_t> symDeque(syms.begin(), syms.end());
    const size_t bufSize = std::max<size_t>(4096, syms.size() * 2 + 1024);
    std::vector<char> buf(bufSize, 0);
    size_t rcSize = pmsf::encodeUint8Queue(symDeque, 5, buf.data(), bufSize);

    std::vector<uint8_t> out;
    out.resize((7 + layerPackedCounts.size() * 8) * sizeof(int32_t));
    int32_t header[7] = {
        (int32_t)syms.size(),
        (int32_t)numNullBefore,
        (int32_t)numNullAfter,
        (int32_t)numRemoved,
        (int32_t)numA,
        (int32_t)numB,
        (int32_t)layerPackedCounts.size(),
    };
    std::memcpy(out.data(), header, sizeof(header));
    size_t metaOff = sizeof(header);
    for (size_t i = 0; i < layerPackedCounts.size(); ++i) {
        const auto& seed = layerSeedQs[i];
        for (int q : seed) {
            const int32_t v = static_cast<int32_t>(q);
            std::memcpy(out.data() + metaOff, &v, sizeof(int32_t));
            metaOff += sizeof(int32_t);
        }
        const auto& c = layerPackedCounts[i];
        std::memcpy(out.data() + metaOff, &c.first, sizeof(int32_t));
        metaOff += sizeof(int32_t);
        std::memcpy(out.data() + metaOff, &c.second, sizeof(int32_t));
        metaOff += sizeof(int32_t);
    }
    out.insert(out.end(),
               reinterpret_cast<uint8_t*>(buf.data()),
               reinterpret_cast<uint8_t*>(buf.data()) + rcSize);
    return out;
}

std::vector<uint8_t> encodeResidualBitplane(const std::vector<int>& rx,
                                            const std::vector<int>& ry,
                                            const std::vector<int>& rz,
                                            int bpBits) {
    const size_t n = rx.size();
    if (ry.size() != n || rz.size() != n) {
        throw std::runtime_error("[PCLTTMStrict] residual axis size mismatch");
    }
    std::deque<MCGAL::PointInt> residuals;
    for (size_t i = 0; i < n; ++i) {
        residuals.emplace_back(rx[i], ry[i], rz[i]);
    }

    const size_t bufSize = std::max<size_t>(4096, n * 16 + 1024);
    std::vector<char> buf(bufSize, 0);
    size_t rcSize = pmsf::encodeResidualBitPlaneQueue(
        residuals, bpBits, buf.data(), bufSize);

    std::vector<uint8_t> out;
    out.resize(8);
    int32_t numCoords = (int32_t)(n * 3);
    int32_t bits = (int32_t)bpBits;
    std::memcpy(out.data(), &numCoords, 4);
    std::memcpy(out.data() + 4, &bits, 4);
    out.insert(out.end(),
               reinterpret_cast<uint8_t*>(buf.data()),
               reinterpret_cast<uint8_t*>(buf.data()) + rcSize);
    return out;
}

}  // namespace

void PCLTTMStrictSymbolCollectOperator::collect(MCGAL::Halfedge* /*h*/) {
    if (!mesh_) {
        throw std::runtime_error("[PCLTTMStrict] collect called before init");
    }

    const char* geomModeEnv = std::getenv("PMSF_PCLTTM_GEOM_MODE");
    const bool useCurvatureResiduals =
        geomModeEnv && std::string(geomModeEnv) == "curvature";
    const bool useFrenetResiduals =
        geomModeEnv && std::string(geomModeEnv) == "frenet";
    const bool useCartesianResiduals =
        !geomModeEnv || std::string(geomModeEnv) == "cartesian";
    const bool useAbsoluteResiduals =
        geomModeEnv && std::string(geomModeEnv) == "absolute";
    // The paper PCLTTM path stores one geometry triple per V-code. The current
    // strict decoder is topology-exact with cartesian residuals; Frenet remains
    // opt-in until its decoder-side frame is calibrated against Figure 12.
    // Curvature mode is an opt-in diagnostic against the repository's RD
    // predictor and must not add side markers to the geometry vector.
    if (useCurvatureResiduals) {
        unsetenv("PMSF_RDPATCH_NO_CURVATURE");
    } else if (!std::getenv("PMSF_RDPATCH_NO_CURVATURE")) {
        setenv("PMSF_RDPATCH_NO_CURVATURE", "1", 0);
    }
    if (envOn("PMSF_PCLTTM_WRITE_RANKS") &&
        !std::getenv("PMSF_RDPATCH_WRITE_FACE_RANKS")) {
        setenv("PMSF_RDPATCH_WRITE_FACE_RANKS", "1", 0);
    }

    RDPatchPipelineConfig cfg{};
    cfg.useRDTagEliminate = true;
    cfg.maxStepsMultiplier = 50;
    cfg.maxCycles = 1;
    cfg.progressLogEvery = 100;
    cfg.strictPCLTTMConquest = true;
    if (const char* e = std::getenv("PMSF_PCLTTM_USE_PAPER_BOUNDARY_GATES")) {
        cfg.usePMSFBoundaryGates = e[0] == '1';
    } else {
        cfg.usePMSFBoundaryGates = true;
    }
    cfg.useFrenetResiduals = useFrenetResiduals && !useCurvatureResiduals &&
                             !useCartesianResiduals && !useAbsoluteResiduals;

    const int targetBase = []() {
        if (const char* e = std::getenv("PMSF_PCLTTM_TARGET_BASE")) {
            int v = std::atoi(e);
            if (v > 0) return v;
        }
        return 50;
    }();
    const int maxOuterRounds = []() {
        if (const char* e = std::getenv("PMSF_PCLTTM_MAX_OUTER_ROUNDS")) {
            int v = std::atoi(e);
            if (v > 0) return v;
        }
        return 128;
    }();

    RDPatchPipelineResult aggregate{};
    aggregate.mesh = mesh_;
    bool first = true;
    bool baseForcingMode = false;
    unsigned short layerBase = 0;
    for (int round = 0; round < maxOuterRounds; ++round) {
        RDPatchPipelineResult r = runRDPatchEncoderOnMesh(mesh_, cfg);
        if (r.numDecimatedTotal <= 0) {
            if (r.numVerticesOut > targetBase && cfg.strictPCLTTMConquest) {
                                cfg.strictPCLTTMConquest = false;
                cfg.usePMSFBoundaryGates = false;
                baseForcingMode = true;
                resetRoundLocalMeshState(mesh_);
                continue;
            }
            break;
        }
        if (first) {
            aggregate.numVerticesIn = r.numVerticesIn;
            aggregate.firstGateV0Q[0] = r.firstGateV0Q[0];
            aggregate.firstGateV0Q[1] = r.firstGateV0Q[1];
            aggregate.firstGateV0Q[2] = r.firstGateV0Q[2];
            aggregate.firstGateV1Q[0] = r.firstGateV1Q[0];
            aggregate.firstGateV1Q[1] = r.firstGateV1Q[1];
            aggregate.firstGateV1Q[2] = r.firstGateV1Q[2];
            aggregate.leakyFactors = r.leakyFactors;
            first = false;
        }
        for (const auto& seedQ : r.layerSeedQs) {
            aggregate.layerSeedQs.push_back(seedQ);
        }
        aggregate.symbolInputQs.insert(aggregate.symbolInputQs.end(),
                                       r.symbolInputQs.begin(), r.symbolInputQs.end());
        for (auto s : r.symbols) {
            s.layer = static_cast<unsigned short>(s.layer + layerBase);
            aggregate.symbols.push_back(s);
        }
        aggregate.vvResidualsX.insert(aggregate.vvResidualsX.end(),
                                      r.vvResidualsX.begin(), r.vvResidualsX.end());
        aggregate.vvResidualsY.insert(aggregate.vvResidualsY.end(),
                                      r.vvResidualsY.begin(), r.vvResidualsY.end());
        aggregate.vvResidualsZ.insert(aggregate.vvResidualsZ.end(),
                                      r.vvResidualsZ.begin(), r.vvResidualsZ.end());
        aggregate.vvAbsQX.insert(aggregate.vvAbsQX.end(), r.vvAbsQX.begin(), r.vvAbsQX.end());
        aggregate.vvAbsQY.insert(aggregate.vvAbsQY.end(), r.vvAbsQY.begin(), r.vvAbsQY.end());
        aggregate.vvAbsQZ.insert(aggregate.vvAbsQZ.end(), r.vvAbsQZ.begin(), r.vvAbsQZ.end());
        aggregate.vvInputOffsets.insert(aggregate.vvInputOffsets.end(),
                                        r.vvInputOffsets.begin(), r.vvInputOffsets.end());
        aggregate.vvRingSizes.insert(aggregate.vvRingSizes.end(),
                                     r.vvRingSizes.begin(), r.vvRingSizes.end());
        aggregate.vvRingQX.insert(aggregate.vvRingQX.end(), r.vvRingQX.begin(), r.vvRingQX.end());
        aggregate.vvRingQY.insert(aggregate.vvRingQY.end(), r.vvRingQY.begin(), r.vvRingQY.end());
        aggregate.vvRingQZ.insert(aggregate.vvRingQZ.end(), r.vvRingQZ.begin(), r.vvRingQZ.end());
        aggregate.vvRemovedVertexIds.insert(aggregate.vvRemovedVertexIds.end(),
                                            r.vvRemovedVertexIds.begin(),
                                            r.vvRemovedVertexIds.end());
        aggregate.vvRingVertexIds.insert(aggregate.vvRingVertexIds.end(),
                                         r.vvRingVertexIds.begin(),
                                         r.vvRingVertexIds.end());
        aggregate.vvDecodeFaceRanks.insert(aggregate.vvDecodeFaceRanks.end(),
                                           r.vvDecodeFaceRanks.begin(),
                                           r.vvDecodeFaceRanks.end());
        aggregate.numDecimatedTotal += r.numDecimatedTotal;
        aggregate.numNullPatches += r.numNullPatches;
        aggregate.numSteps += r.numSteps;
        aggregate.numDecCycles += std::max(1, r.numDecCycles);
        for (int i = 0; i < 6; ++i) aggregate.bucketHits[i] += r.bucketHits[i];
        aggregate.numVerticesOut = r.numVerticesOut;
                layerBase = static_cast<unsigned short>(layerBase + std::max(1, r.numDecCycles));
        if (r.numDecimatedTotal <= 0 || r.numVerticesOut <= targetBase) break;
        resetRoundLocalMeshState(mesh_);
    }
    if (first) {
        aggregate.numVerticesIn = 0;
        aggregate.numVerticesOut = 0;
    }
    if (!aggregate.symbols.empty() && envOn("PMSF_PCLTTM_REVERSE_V_ONLY")) {
        if (aggregate.mesh && envOn("PMSF_PCLTTM_REVERSE_RANKS") &&
            !envOn("PMSF_PCLTTM_SKIP_BASE_GC_ALIGN")) {
            aggregate.mesh->garbage_collection();
            resetRoundLocalMeshState(aggregate.mesh);
        }
        bool reverseRankRebuildOk = false;
        RDPatchPipelineResult rankedPrefix;
        if (envOn("PMSF_PCLTTM_REVERSE_RANKS")) {
            rankedPrefix =
                rebuildRDPatchResultInReverseRingOrder(
                    cloneMeshForStrictReplay(aggregate.mesh), aggregate);
            if (rankedPrefix.numDecimatedTotal == aggregate.numDecimatedTotal) {
                rankedPrefix.mesh = aggregate.mesh;
                aggregate = std::move(rankedPrefix);
                mesh_ = aggregate.mesh;
                reverseRankRebuildOk = true;
                            } else {
                std::fprintf(stderr,
                             "[PCLTTMStrict] WARNING reverse rank stream incomplete "
                             "(%d/%d); falling back to reverse ring stream\n",
                             rankedPrefix.numDecimatedTotal,
                             aggregate.numDecimatedTotal);
            }
        }
        if (!reverseRankRebuildOk) {
        struct ReverseVRecord {
            RDPatchSymbol sym{};
            std::array<int, 6> inputQ{};
            std::vector<std::array<int, 3>> ringQ;
            std::vector<int> ringVertexIds;
            int removedVertexId = -1;
            int rx = 0, ry = 0, rz = 0;
            int ax = 0, ay = 0, az = 0;
            std::vector<int> faceRanks;
            unsigned char inputOffset = 0;
        };
        std::vector<ReverseVRecord> records;
        records.reserve(aggregate.vvResidualsX.size());
        size_t ringCursor = 0;
        size_t vCursor = 0;
        size_t rankCursor = 0;
        const bool flattenedFaceRanks =
            aggregate.vvDecodeFaceRanks.size() > aggregate.vvResidualsX.size();
        for (size_t si = 0; si < aggregate.symbols.size(); ++si) {
            const RDPatchSymbol& s = aggregate.symbols[si];
            if (s.kind == RDPatchSymbolKind::NULL_PATCH) continue;
            if (vCursor >= aggregate.vvResidualsX.size() ||
                vCursor >= aggregate.vvResidualsY.size() ||
                vCursor >= aggregate.vvResidualsZ.size() ||
                vCursor >= aggregate.vvRingSizes.size()) {
                break;
            }
            ReverseVRecord rec;
            rec.sym = s;
            rec.sym.phase = 0;
            rec.sym.layer = 0;
            rec.sym.bucket = -1;
            if (si < aggregate.symbolInputQs.size()) {
                rec.inputQ = aggregate.symbolInputQs[si];
            }
            if (vCursor < aggregate.vvRemovedVertexIds.size()) {
                rec.removedVertexId = aggregate.vvRemovedVertexIds[vCursor];
            }
            rec.rx = aggregate.vvResidualsX[vCursor];
            rec.ry = aggregate.vvResidualsY[vCursor];
            rec.rz = aggregate.vvResidualsZ[vCursor];
            if (vCursor < aggregate.vvAbsQX.size() &&
                vCursor < aggregate.vvAbsQY.size() &&
                vCursor < aggregate.vvAbsQZ.size()) {
                rec.ax = aggregate.vvAbsQX[vCursor];
                rec.ay = aggregate.vvAbsQY[vCursor];
                rec.az = aggregate.vvAbsQZ[vCursor];
            }
            if (vCursor < aggregate.vvInputOffsets.size()) {
                rec.inputOffset = aggregate.vvInputOffsets[vCursor];
            }
            if (flattenedFaceRanks) {
                const int rankN = std::max(1, patchSymbolValence(s) - 2);
                for (int k = 0; k < rankN && rankCursor < aggregate.vvDecodeFaceRanks.size(); ++k) {
                    rec.faceRanks.push_back(aggregate.vvDecodeFaceRanks[rankCursor++]);
                }
            } else if (vCursor < aggregate.vvDecodeFaceRanks.size()) {
                rec.faceRanks.push_back(aggregate.vvDecodeFaceRanks[vCursor]);
            }
            const int n = static_cast<int>(aggregate.vvRingSizes[vCursor]);
            rec.ringQ.reserve(static_cast<size_t>(std::max(n, 0)));
            for (int k = 0; k < n; ++k) {
                const size_t qi = ringCursor + static_cast<size_t>(k);
                if (qi < aggregate.vvRingQX.size() &&
                    qi < aggregate.vvRingQY.size() &&
                    qi < aggregate.vvRingQZ.size()) {
                    rec.ringQ.push_back({aggregate.vvRingQX[qi],
                                         aggregate.vvRingQY[qi],
                                         aggregate.vvRingQZ[qi]});
                    if (qi < aggregate.vvRingVertexIds.size()) {
                        rec.ringVertexIds.push_back(aggregate.vvRingVertexIds[qi]);
                    } else {
                        rec.ringVertexIds.push_back(-1);
                    }
                }
            }
            ringCursor += static_cast<size_t>(std::max(n, 0));
            records.push_back(std::move(rec));
            ++vCursor;
        }

        auto truncateRankPrefix = [](RDPatchPipelineResult& r, int keep) {
            keep = std::max(0, std::min(keep, r.numDecimatedTotal));
            auto resizeTo = [](auto& v, size_t n) {
                if (v.size() > n) v.resize(n);
            };

            size_t symbolKeep = 0;
            int keptV = 0;
            for (; symbolKeep < r.symbols.size(); ++symbolKeep) {
                if (r.symbols[symbolKeep].kind != RDPatchSymbolKind::NULL_PATCH) {
                    if (keptV >= keep) break;
                    ++keptV;
                }
            }
            resizeTo(r.symbols, symbolKeep);
            resizeTo(r.symbolInputQs, symbolKeep);

            const size_t keepV = static_cast<size_t>(keep);
            resizeTo(r.vvResidualsX, keepV);
            resizeTo(r.vvResidualsY, keepV);
            resizeTo(r.vvResidualsZ, keepV);
            resizeTo(r.vvAbsQX, keepV);
            resizeTo(r.vvAbsQY, keepV);
            resizeTo(r.vvAbsQZ, keepV);
            resizeTo(r.vvRemovedVertexIds, keepV);
            resizeTo(r.vvInputOffsets, keepV);

            size_t ringCoords = 0;
            const size_t ringGroups = std::min(keepV, r.vvRingSizes.size());
            for (size_t i = 0; i < ringGroups; ++i) {
                ringCoords += static_cast<size_t>(r.vvRingSizes[i]);
            }
            resizeTo(r.vvRingSizes, keepV);
            resizeTo(r.vvRingQX, ringCoords);
            resizeTo(r.vvRingQY, ringCoords);
            resizeTo(r.vvRingQZ, ringCoords);
            resizeTo(r.vvRingVertexIds, ringCoords);
            if (r.vvRingVertexIds.size() < ringCoords) {
                r.vvRingVertexIds.resize(ringCoords, -1);
            }

            size_t rankCount = 0;
            const bool flattenedRanks = r.vvDecodeFaceRanks.size() > r.vvResidualsX.size();
            if (flattenedRanks) {
                for (const RDPatchSymbol& s : r.symbols) {
                    if (s.kind == RDPatchSymbolKind::NULL_PATCH) continue;
                    rankCount += static_cast<size_t>(
                        std::max(1, patchSymbolValence(s) - 2));
                }
            } else {
                rankCount = keepV;
            }
            resizeTo(r.vvDecodeFaceRanks, rankCount);
            r.numDecimatedTotal = keep;
        };

        auto verifiedRankReplayPrefix = [&](const RDPatchPipelineResult& r) {
            if (r.numDecimatedTotal <= 0 || r.vvDecodeFaceRanks.empty()) return 0;
            const std::vector<int> empty;
            const std::vector<int>& absQX = useAbsoluteResiduals ? r.vvAbsQX : empty;
            const std::vector<int>& absQY = useAbsoluteResiduals ? r.vvAbsQY : empty;
            const std::vector<int>& absQZ = useAbsoluteResiduals ? r.vvAbsQZ : empty;
            RDPatchDecoderResult rr = runRDPatchDecoderConquest(
                cloneMeshForStrictReplay(aggregate.mesh),
                r.symbols,
                r.vvResidualsX,
                r.vvResidualsY,
                r.vvResidualsZ,
                r.vvDecodeFaceRanks,
                /*ringSizes=*/{},
                /*ringQX=*/{},
                /*ringQY=*/{},
                /*ringQZ=*/{},
                absQX,
                absQY,
                absQZ,
                /*firstGateQ=*/{},
                /*inputOffsets=*/{},
                /*symbolInputQs=*/{},
                /*forcePMSFBoundaryGates=*/true,
                /*forceStrictRDConquest=*/true);
            return rr.numAddedVertices;
        };

        RDPatchPipelineResult reversed{};
        int rankPrefix = rankedPrefix.numDecimatedTotal;
        if (rankPrefix > 0 && rankPrefix < static_cast<int>(records.size())) {
            if (envOn("PMSF_PCLTTM_VERIFY_HYBRID_PREFIX")) {
                const int replayPrefix = verifiedRankReplayPrefix(rankedPrefix);
                if (const char* e = std::getenv("PMSF_BPV_DIAG"); e && e[0] == '1') {
                    std::fprintf(stderr,
                                 "[PCLTTMStrict] verified hybrid rank prefix %d/%d\n",
                                 replayPrefix,
                                 rankPrefix);
                }
                if (replayPrefix >= 0 && replayPrefix < rankPrefix) {
                    rankPrefix = replayPrefix;
                }
            }
            int seamBackoff = 0;
            if (const char* e = std::getenv("PMSF_PCLTTM_HYBRID_SEAM_BACKOFF")) {
                seamBackoff = std::max(0, std::atoi(e));
            }
            rankPrefix = std::max(0, rankPrefix - seamBackoff);
            truncateRankPrefix(rankedPrefix, rankPrefix);
        }
        const bool useHybridLocator =
            envOn("PMSF_PCLTTM_REVERSE_RANKS") &&
            (!std::getenv("PMSF_PCLTTM_DISABLE_HYBRID_LOCATOR")) &&
            rankPrefix > 0 && rankPrefix < static_cast<int>(records.size());
        if (useHybridLocator) {
            reversed = std::move(rankedPrefix);
            std::vector<int> originalPrefixRemovedIds;
            originalPrefixRemovedIds.reserve(static_cast<size_t>(rankPrefix));
            int prefixRecordIndex = 0;
            for (auto it = records.rbegin();
                 it != records.rend() && prefixRecordIndex < rankPrefix;
                 ++it, ++prefixRecordIndex) {
                originalPrefixRemovedIds.push_back(it->removedVertexId);
            }
            if (static_cast<int>(originalPrefixRemovedIds.size()) == rankPrefix) {
                reversed.vvRemovedVertexIds = std::move(originalPrefixRemovedIds);
            }
            reversed.mesh = aggregate.mesh;
            reversed.numVerticesIn = aggregate.numVerticesIn;
            reversed.numVerticesOut = aggregate.numVerticesOut;
            reversed.numDecimatedTotal = rankPrefix;
            reversed.numDecCycles = 1;
            reversed.leakyFactors = aggregate.leakyFactors;
            reversed.locatorRankPrefix = rankPrefix;
            if (reversed.layerSeedQs.empty()) {
                reversed.layerSeedQs.push_back(aggregate.layerSeedQs.empty()
                                                   ? std::array<int, 6>{}
                                                   : aggregate.layerSeedQs.back());
            }
        } else {
            reversed.mesh = aggregate.mesh;
            reversed.numVerticesIn = aggregate.numVerticesIn;
            reversed.numVerticesOut = aggregate.numVerticesOut;
            reversed.numDecimatedTotal = 0;
            reversed.numDecCycles = 1;
            reversed.leakyFactors = aggregate.leakyFactors;
            reversed.layerSeedQs.push_back(aggregate.layerSeedQs.empty()
                                               ? std::array<int, 6>{}
                                               : aggregate.layerSeedQs.back());
        }
        int reverseIndex = 0;
        for (auto it = records.rbegin(); it != records.rend(); ++it, ++reverseIndex) {
            if (useHybridLocator && reverseIndex < rankPrefix) continue;
            reversed.symbols.push_back(it->sym);
            reversed.symbolInputQs.push_back(it->inputQ);
            reversed.vvResidualsX.push_back(it->rx);
            reversed.vvResidualsY.push_back(it->ry);
            reversed.vvResidualsZ.push_back(it->rz);
            reversed.vvAbsQX.push_back(it->ax);
            reversed.vvAbsQY.push_back(it->ay);
            reversed.vvAbsQZ.push_back(it->az);
            reversed.vvRemovedVertexIds.push_back(it->removedVertexId);
            reversed.vvInputOffsets.push_back(it->inputOffset);
            if (!useHybridLocator) {
                reversed.vvDecodeFaceRanks.insert(reversed.vvDecodeFaceRanks.end(),
                                                  it->faceRanks.begin(),
                                                  it->faceRanks.end());
            }
            reversed.vvRingSizes.push_back(
                static_cast<unsigned char>(std::min<size_t>(255, it->ringQ.size())));
            for (const auto& q : it->ringQ) {
                reversed.vvRingQX.push_back(q[0]);
                reversed.vvRingQY.push_back(q[1]);
                reversed.vvRingQZ.push_back(q[2]);
            }
            reversed.vvRingVertexIds.insert(reversed.vvRingVertexIds.end(),
                                            it->ringVertexIds.begin(),
                                            it->ringVertexIds.end());
            ++reversed.numDecimatedTotal;
        }
        reversed.numDecimatedTotal = static_cast<int>(records.size());
        aggregate = std::move(reversed);
        mesh_ = aggregate.mesh;
                }
    }
    if (!aggregate.symbols.empty() && envOn("PMSF_PCLTTM_REBUILD_DECODE_ORDER")) {
        auto baseForSerialize = aggregate.mesh;
        RDPatchPipelineResult reordered =
            rebuildRDPatchResultInStrictDecodeOrder(
                cloneMeshForStrictReplay(baseForSerialize), aggregate);
        const bool acceptIncomplete = envOn("PMSF_PCLTTM_ACCEPT_INCOMPLETE_REBUILD");
        if (reordered.numDecimatedTotal == aggregate.numDecimatedTotal ||
            acceptIncomplete) {
            reordered.mesh = baseForSerialize;
            aggregate = std::move(reordered);
            aggregate.mesh = baseForSerialize;
            mesh_ = baseForSerialize;
                    } else {
            RDPatchPipelineResult split =
                rebuildRDPatchResultInSplitStrictDecodeOrder(
                    cloneMeshForStrictReplay(baseForSerialize), aggregate);
            if (split.numDecimatedTotal == aggregate.numDecimatedTotal) {
                split.mesh = baseForSerialize;
                aggregate = std::move(split);
                aggregate.mesh = baseForSerialize;
                mesh_ = baseForSerialize;
                std::fprintf(stderr,
                             "[PCLTTMStrict] split rebuilt decode order removed=%d "
                             "symbols=%zu layers=%zu\n",
                             aggregate.numDecimatedTotal,
                             aggregate.symbols.size(),
                             aggregate.layerSeedQs.size());
            } else {
                std::fprintf(stderr,
                             "[PCLTTMStrict] WARNING decode-order rebuild incomplete "
                             "(single=%d split=%d/%d); keeping layered stream\n",
                             reordered.numDecimatedTotal,
                             split.numDecimatedTotal,
                             aggregate.numDecimatedTotal);
            }
        }
    }
    lastResult_ = std::move(aggregate);
    paperTableCache_.clear();
}

const PMSFSymbolTable* PCLTTMStrictSymbolCollectOperator::pmsfSymbolTable() const {
    if (lastResult_.symbols.empty() && lastResult_.numVerticesIn == 0 &&
        lastResult_.numVerticesOut == 0) {
        return nullptr;
    }
    if (paperTableCache_.empty()) {
        paperTableCache_ =
            pmsfSymbolTableFromRDPatchResult(lastResult_, pmsfChannelPrefix());
        if (!paperTableCache_.empty()) {
            auto& delta = paperTableCache_.front().delta;
            const std::string prefix = pmsfChannelPrefix();
            const char* geomModeEnv = std::getenv("PMSF_PCLTTM_GEOM_MODE");
            int geomMode = 0;  // 0=cartesian, 1=frenet, 2=curvature, 3=absolute
            if (geomModeEnv) {
                const std::string mode(geomModeEnv);
                if (mode == "frenet") geomMode = 1;
                else if (mode == "curvature") geomMode = 2;
                else if (mode == "absolute") geomMode = 3;
            }
            pmsf::appendPCLTTMStrictReplaySideChannel(
                delta,
                prefix,
                static_cast<std::int32_t>(geomMode),
                envFlagI32("PMSF_PCLTTM_WRITE_RANKS"),
                envFlagI32("PMSF_PCLTTM_WRITE_RINGS"),
                envFlagI32("PMSF_PCLTTM_USE_RANK_REPLAY"),
                envFlagI32("PMSF_PCLTTM_USE_RING_REPLAY"),
                envFlagI32("PMSF_PCLTTM_USE_RD_REPLAY"),
                envFlagI32("PMSF_PCLTTM_REVERSE_V_ONLY"),
                envFlagI32("PMSF_PCLTTM_REVERSE_RANKS"),
                envFlagI32("PMSF_PCLTTM_PACK_AS_IS"));
        }
    }
    return paperTableCache_.empty() ? nullptr : &paperTableCache_;
}

int PCLTTMStrictSymbolCollectOperator::exportStrictToBufferWithLayout(
    char* buffer, int blobLayout, int schemeVersion, const char* diagLabel) {
    if (buffer == nullptr) return 0;
    requirePCLTTMStrictPaperExactIfRequested();
    if (lastResult_.symbols.empty()) {
        throw std::runtime_error(
            "[PCLTTMStrict] export called before collect or with empty symbol stream");
    }

    struct LayerStreams {
        std::vector<uint8_t> aSyms;
        std::vector<uint8_t> bSyms;
        std::vector<int> ax, ay, az;
        std::vector<int> bx, by, bz;
        std::vector<uint8_t> aOffsets;
        std::vector<uint8_t> bOffsets;
        std::vector<std::vector<int32_t>> aRings;
        std::vector<std::vector<int32_t>> bRings;
        std::vector<std::vector<int32_t>> aReconRingIds;
        std::vector<std::vector<int32_t>> bReconRingIds;
        std::vector<std::array<int, 6>> aInputs;
        std::vector<std::array<int, 6>> bInputs;
        std::vector<int> aFaceRanks;
        std::vector<int> bFaceRanks;
    };
    unsigned short maxLayer = 0;
    size_t symbolCursor = 0;
    for (const auto& s : lastResult_.symbols) {
        maxLayer = std::max(maxLayer, s.layer);
    }
    std::vector<LayerStreams> layers(static_cast<size_t>(maxLayer) + 1);

    size_t residualCursor = 0;
    int nullBefore = 0;
    int nullPackedAway = 0;
    int removed = 0;
    int numA = 0;
    int numB = 0;
    // MEPP2/PCLTTM paper storage uses prediction residuals: removed vertex
    // quantized position minus the polygon barycenter reconstructed by the
    // decoder. Absolute geometry remains an explicit diagnostic escape hatch
    // because it hides topology/barycenter mismatches instead of fixing them.
    const bool useAbsoluteGeometry = []() {
        const char* e = std::getenv("PMSF_PCLTTM_GEOM_MODE");
        return e && std::string(e) == "absolute";
    }();
    std::vector<size_t> ringStarts(lastResult_.vvRingSizes.size() + 1, 0);
    for (size_t i = 0; i < lastResult_.vvRingSizes.size(); ++i) {
        ringStarts[i + 1] = ringStarts[i] + lastResult_.vvRingSizes[i];
    }
    std::unordered_map<int, int> poolIdToReconId;
    int nextReconId = 0;
    if (lastResult_.mesh) {
        bool canUseSerializedVids = true;
        int maxVid = -1;
        std::unordered_map<int, bool> seenVids;
        for (MCGAL::Vertex* v : lastResult_.mesh->vertices()) {
            if (!v || v->isRemoved()) continue;
            const int vid = v->vid();
            if (vid < 0 || seenVids.count(vid)) {
                canUseSerializedVids = false;
                break;
            }
            seenVids[vid] = true;
            maxVid = std::max(maxVid, vid);
        }
        if (canUseSerializedVids && maxVid >= 0) {
            for (MCGAL::Vertex* v : lastResult_.mesh->vertices()) {
                if (!v || v->isRemoved()) continue;
                poolIdToReconId.emplace(v->poolId(), v->vid());
            }
            nextReconId = maxVid + 1;
        } else {
        for (MCGAL::Vertex* v : lastResult_.mesh->vertices()) {
            if (!v || v->isRemoved()) continue;
            poolIdToReconId.emplace(v->poolId(), nextReconId++);
        }
        }
    }
    int insertedReconOrdinal = 0;
    for (size_t si = 0, vi = 0; si < lastResult_.symbols.size(); ++si) {
        const RDPatchSymbol& s = lastResult_.symbols[si];
        if (s.kind == RDPatchSymbolKind::NULL_PATCH) continue;
        if (vi < lastResult_.vvRemovedVertexIds.size()) {
            poolIdToReconId[lastResult_.vvRemovedVertexIds[vi]] =
                nextReconId + insertedReconOrdinal;
        }
        ++insertedReconOrdinal;
        ++vi;
    }
    std::vector<int32_t> greconOriginalIds;
    if (!poolIdToReconId.empty()) {
        int maxReconId = -1;
        for (const auto& kv : poolIdToReconId) {
            maxReconId = std::max(maxReconId, kv.second);
        }
        if (maxReconId >= 0) {
            greconOriginalIds.assign(static_cast<size_t>(maxReconId) + 1, -1);
            for (const auto& kv : poolIdToReconId) {
                if (kv.second >= 0 &&
                    static_cast<size_t>(kv.second) < greconOriginalIds.size()) {
                    greconOriginalIds[static_cast<size_t>(kv.second)] = kv.first;
                }
            }
        }
    }
    auto reconRingIdsForResidual = [&](size_t residualIdx) {
        std::vector<int32_t> ids;
        if (residualIdx >= lastResult_.vvRingSizes.size()) return ids;
        const size_t rs = ringStarts[residualIdx];
        const size_t re = ringStarts[residualIdx + 1];
        ids.reserve(re - rs);
        for (size_t qi = rs; qi < re; ++qi) {
            int32_t encoded = -1;
            if (qi < lastResult_.vvRingVertexIds.size()) {
                auto it = poolIdToReconId.find(lastResult_.vvRingVertexIds[qi]);
                if (it != poolIdToReconId.end()) encoded = static_cast<int32_t>(it->second);
            }
            ids.push_back(encoded);
        }
        return ids;
    };
    const bool flattenedFaceRanks =
        lastResult_.vvDecodeFaceRanks.size() > lastResult_.vvResidualsX.size();
    size_t faceRankCursor = 0;
    const int locatorRankPrefix = lastResult_.locatorRankPrefix;
    const int locatorRingPrefix = lastResult_.locatorRingPrefix;
    auto appendFaceRanksForSymbol = [&](std::vector<int>& dst,
                                        const RDPatchSymbol& s,
                                        size_t residualIdx) {
        if (locatorRankPrefix >= 0 &&
            residualIdx >= static_cast<size_t>(locatorRankPrefix)) {
            return;
        }
        if (flattenedFaceRanks) {
            const int n = std::max(1, patchSymbolValence(s) - 2);
            for (int k = 0; k < n && faceRankCursor < lastResult_.vvDecodeFaceRanks.size(); ++k) {
                dst.push_back(lastResult_.vvDecodeFaceRanks[faceRankCursor++]);
            }
        } else if (residualIdx < lastResult_.vvDecodeFaceRanks.size()) {
            dst.push_back(lastResult_.vvDecodeFaceRanks[residualIdx]);
        }
    };

    for (const auto& s : lastResult_.symbols) {
        const bool cleaning = (s.phase == 1);
        uint8_t strict = strictSymbolFromPatch(s, cleaning);
        if (strict == kSymNull) ++nullBefore;
        LayerStreams& L = layers[static_cast<size_t>(s.layer)];
        if (cleaning) {
            L.bSyms.push_back(strict);
            ++numB;
        } else {
            L.aSyms.push_back(strict);
            ++numA;
        }

        if (strict != kSymNull) {
            if (residualCursor >= lastResult_.vvResidualsX.size() ||
                residualCursor >= lastResult_.vvResidualsY.size() ||
                residualCursor >= lastResult_.vvResidualsZ.size()) {
                throw std::runtime_error(
                    "[PCLTTMStrict] residual count is smaller than V-code count");
            }
            if (useAbsoluteGeometry &&
                (residualCursor >= lastResult_.vvAbsQX.size() ||
                 residualCursor >= lastResult_.vvAbsQY.size() ||
                 residualCursor >= lastResult_.vvAbsQZ.size())) {
                throw std::runtime_error(
                    "[PCLTTMStrict] absolute geometry count is smaller than V-code count");
            }
            if (cleaning) {
                L.bx.push_back(useAbsoluteGeometry ? lastResult_.vvAbsQX[residualCursor]
                                                   : lastResult_.vvResidualsX[residualCursor]);
                L.by.push_back(useAbsoluteGeometry ? lastResult_.vvAbsQY[residualCursor]
                                                   : lastResult_.vvResidualsY[residualCursor]);
                L.bz.push_back(useAbsoluteGeometry ? lastResult_.vvAbsQZ[residualCursor]
                                                   : lastResult_.vvResidualsZ[residualCursor]);
                appendFaceRanksForSymbol(L.bFaceRanks, s, residualCursor);
                if (residualCursor < lastResult_.vvInputOffsets.size()) {
                    L.bOffsets.push_back(lastResult_.vvInputOffsets[residualCursor]);
                }
                if (symbolCursor < lastResult_.symbolInputQs.size()) {
                    L.bInputs.push_back(lastResult_.symbolInputQs[symbolCursor]);
                }
                L.bReconRingIds.push_back(reconRingIdsForResidual(residualCursor));
                if (residualCursor < lastResult_.vvRingSizes.size()) {
                    const size_t rs = ringStarts[residualCursor];
                    const size_t re = ringStarts[residualCursor + 1];
                    std::vector<int32_t> rec;
                    rec.push_back(static_cast<int32_t>(re - rs));
                    for (size_t qi = rs; qi < re; ++qi) {
                        rec.push_back(static_cast<int32_t>(lastResult_.vvRingQX[qi]));
                        rec.push_back(static_cast<int32_t>(lastResult_.vvRingQY[qi]));
                        rec.push_back(static_cast<int32_t>(lastResult_.vvRingQZ[qi]));
                    }
                    L.bRings.push_back(std::move(rec));
                }
            } else {
                L.ax.push_back(useAbsoluteGeometry ? lastResult_.vvAbsQX[residualCursor]
                                                   : lastResult_.vvResidualsX[residualCursor]);
                L.ay.push_back(useAbsoluteGeometry ? lastResult_.vvAbsQY[residualCursor]
                                                   : lastResult_.vvResidualsY[residualCursor]);
                L.az.push_back(useAbsoluteGeometry ? lastResult_.vvAbsQZ[residualCursor]
                                                   : lastResult_.vvResidualsZ[residualCursor]);
                appendFaceRanksForSymbol(L.aFaceRanks, s, residualCursor);
                if (residualCursor < lastResult_.vvInputOffsets.size()) {
                    L.aOffsets.push_back(lastResult_.vvInputOffsets[residualCursor]);
                }
                if (symbolCursor < lastResult_.symbolInputQs.size()) {
                    L.aInputs.push_back(lastResult_.symbolInputQs[symbolCursor]);
                }
                L.aReconRingIds.push_back(reconRingIdsForResidual(residualCursor));
                if (residualCursor < lastResult_.vvRingSizes.size()) {
                    const size_t rs = ringStarts[residualCursor];
                    const size_t re = ringStarts[residualCursor + 1];
                    std::vector<int32_t> rec;
                    rec.push_back(static_cast<int32_t>(re - rs));
                    for (size_t qi = rs; qi < re; ++qi) {
                        rec.push_back(static_cast<int32_t>(lastResult_.vvRingQX[qi]));
                        rec.push_back(static_cast<int32_t>(lastResult_.vvRingQY[qi]));
                        rec.push_back(static_cast<int32_t>(lastResult_.vvRingQZ[qi]));
                    }
                    L.aRings.push_back(std::move(rec));
                }
            }
            ++residualCursor;
            ++removed;
        }
        ++symbolCursor;
    }

    const bool packAsIs = envOn("PMSF_PCLTTM_PACK_AS_IS");
    std::vector<std::array<int, 6>> connectivityLayerSeeds = lastResult_.layerSeedQs;
    if (packAsIs) {
        const bool preservePackLayers =
            envOn("PMSF_PCLTTM_PACK_AS_IS_PRESERVE_LAYERS") ||
            (envOn("PMSF_PCLTTM_REBUILD_DECODE_ORDER") &&
             lastResult_.layerSeedQs.size() > 1);
        const size_t packLayerCount = preservePackLayers
                                          ? static_cast<size_t>(maxLayer) + 1
                                          : 1;
        layers.assign(packLayerCount, LayerStreams{});
        connectivityLayerSeeds.clear();
        if (preservePackLayers) {
            connectivityLayerSeeds.resize(packLayerCount);
            for (size_t i = 0; i < packLayerCount; ++i) {
                connectivityLayerSeeds[i] =
                    i < lastResult_.layerSeedQs.size()
                        ? lastResult_.layerSeedQs[i]
                        : std::array<int, 6>{};
            }
        } else {
            connectivityLayerSeeds.push_back(lastResult_.layerSeedQs.empty()
                                                 ? std::array<int, 6>{}
                                                 : lastResult_.layerSeedQs.front());
        }
        residualCursor = 0;
        symbolCursor = 0;
        faceRankCursor = 0;
        nullBefore = 0;
        removed = 0;
        numA = 0;
        numB = 0;
        for (const auto& s : lastResult_.symbols) {
            const uint8_t strict = strictSymbolFromPatch(s, false);
            if (strict == kSymNull) ++nullBefore;
            const size_t layerIndex =
                preservePackLayers
                    ? std::min(static_cast<size_t>(s.layer), layers.size() - 1)
                    : 0;
            LayerStreams& L = layers[layerIndex];
            L.aSyms.push_back(strict);
            ++numA;

            if (strict != kSymNull) {
                if (residualCursor >= lastResult_.vvResidualsX.size() ||
                    residualCursor >= lastResult_.vvResidualsY.size() ||
                    residualCursor >= lastResult_.vvResidualsZ.size()) {
                    throw std::runtime_error(
                        "[PCLTTMStrict] residual count is smaller than V-code count");
                }
                L.ax.push_back(useAbsoluteGeometry ? lastResult_.vvAbsQX[residualCursor]
                                                   : lastResult_.vvResidualsX[residualCursor]);
                L.ay.push_back(useAbsoluteGeometry ? lastResult_.vvAbsQY[residualCursor]
                                                   : lastResult_.vvResidualsY[residualCursor]);
                L.az.push_back(useAbsoluteGeometry ? lastResult_.vvAbsQZ[residualCursor]
                                                   : lastResult_.vvResidualsZ[residualCursor]);
                appendFaceRanksForSymbol(L.aFaceRanks, s, residualCursor);
                if (residualCursor < lastResult_.vvInputOffsets.size()) {
                    L.aOffsets.push_back(lastResult_.vvInputOffsets[residualCursor]);
                }
                if (symbolCursor < lastResult_.symbolInputQs.size()) {
                    L.aInputs.push_back(lastResult_.symbolInputQs[symbolCursor]);
                }
                L.aReconRingIds.push_back(reconRingIdsForResidual(residualCursor));
                if (residualCursor < lastResult_.vvRingSizes.size()) {
                    const size_t rs = ringStarts[residualCursor];
                    const size_t re = ringStarts[residualCursor + 1];
                    std::vector<int32_t> rec;
                    rec.push_back(static_cast<int32_t>(re - rs));
                    for (size_t qi = rs; qi < re; ++qi) {
                        rec.push_back(static_cast<int32_t>(lastResult_.vvRingQX[qi]));
                        rec.push_back(static_cast<int32_t>(lastResult_.vvRingQY[qi]));
                        rec.push_back(static_cast<int32_t>(lastResult_.vvRingQZ[qi]));
                    }
                    L.aRings.push_back(std::move(rec));
                }
                ++residualCursor;
                ++removed;
            }
            ++symbolCursor;
        }
    }

    // Paper §3.7 layer reshuffle: Bn An ... B1 A1. Layer-internal FIFO order is
    // preserved. Paper §4.2 code packing may omit only null patches that the
    // decoder can infer from the live conquest state. Until that inference is
    // reproduced exactly, strict default storage keeps all N symbols and leaves
    // the experimental run-packing hook opt-in.
    std::vector<uint8_t> packed;
    packed.reserve(static_cast<size_t>(numA + numB));
    std::vector<std::pair<int32_t, int32_t>> layerPackedCounts(layers.size(), {0, 0});
    const bool reverseWithinLayers = []() {
        const char* e = std::getenv("PMSF_PCLTTM_REVERSE_WITHIN_LAYERS");
        return e && e[0] == '1';
    }();
    const int nullRunCap = []() {
        if (const char* e = std::getenv("PMSF_PCLTTM_NULL_RUN_CAP")) {
            return std::atoi(e);
        }
        return -1;
    }();
    auto appendPackedCodes = [&](const std::vector<uint8_t>& src) {
        const size_t before = packed.size();
        int nullRun = 0;
        for (uint8_t s : src) {
            if (s == kSymNull) {
                if (nullRunCap >= 0 && nullRun >= nullRunCap) {
                    ++nullPackedAway;
                    ++nullRun;
                    continue;
                }
                ++nullRun;
            } else {
                nullRun = 0;
            }
            packed.push_back(s);
        }
        return static_cast<int32_t>(packed.size() - before);
    };
    auto appendPackedCodesMaybeReverse = [&](const std::vector<uint8_t>& src) {
        if (!reverseWithinLayers) return appendPackedCodes(src);
        std::vector<uint8_t> rev(src.rbegin(), src.rend());
        return appendPackedCodes(rev);
    };
    for (int li = static_cast<int>(layers.size()) - 1; li >= 0; --li) {
        LayerStreams& L = layers[static_cast<size_t>(li)];
        const int32_t bCount = appendPackedCodesMaybeReverse(L.bSyms);
        const int32_t aCount = appendPackedCodesMaybeReverse(L.aSyms);
        layerPackedCounts[static_cast<size_t>(li)] = {aCount, bCount};
    }
    const int nullAfter = nullBefore - nullPackedAway;

    auto conn = encodeStrictConnectivity(
        packed, nullBefore, nullAfter, removed,
        numA, numB, layerPackedCounts, connectivityLayerSeeds);

    // Geometry stream: one residual triple per V-code, ordered exactly like the
    // packed connectivity walk (Bn,An,...,B1,A1). Each layer stores per-axis
    // min/range metadata, then adaptive arithmetic-codes x/y/z residual symbols.
    // This mirrors MEPP2 Compression_Valence: connectivity symbols are coded
    // separately while geometry is stored as barycenter-prediction residuals.
    std::vector<int> gx, gy, gz;
    std::vector<int> granks;
    std::vector<uint8_t> goffsets;
    std::vector<std::vector<int32_t>> grings;
    std::vector<std::vector<int32_t>> greconRingIds;
    std::vector<std::array<int, 6>> ginputs;
    gx.reserve(static_cast<size_t>(removed));
    gy.reserve(static_cast<size_t>(removed));
    gz.reserve(static_cast<size_t>(removed));
    granks.reserve(static_cast<size_t>(removed));
    goffsets.reserve(static_cast<size_t>(removed));
    grings.reserve(static_cast<size_t>(removed));
    auto appendIntsMaybeReverse = [&](std::vector<int>& dst, const std::vector<int>& src) {
        if (reverseWithinLayers) dst.insert(dst.end(), src.rbegin(), src.rend());
        else dst.insert(dst.end(), src.begin(), src.end());
    };
    auto appendBytesMaybeReverse = [&](std::vector<uint8_t>& dst, const std::vector<uint8_t>& src) {
        if (reverseWithinLayers) dst.insert(dst.end(), src.rbegin(), src.rend());
        else dst.insert(dst.end(), src.begin(), src.end());
    };
    auto appendRingsMaybeReverse = [&](std::vector<std::vector<int32_t>>& dst,
                                       const std::vector<std::vector<int32_t>>& src) {
        if (reverseWithinLayers) dst.insert(dst.end(), src.rbegin(), src.rend());
        else dst.insert(dst.end(), src.begin(), src.end());
    };
    auto appendInputsMaybeReverse = [&](std::vector<std::array<int, 6>>& dst,
                                        const std::vector<std::array<int, 6>>& src) {
        if (reverseWithinLayers) dst.insert(dst.end(), src.rbegin(), src.rend());
        else dst.insert(dst.end(), src.begin(), src.end());
    };
    for (int li = static_cast<int>(layers.size()) - 1; li >= 0; --li) {
        LayerStreams& L = layers[static_cast<size_t>(li)];
        appendIntsMaybeReverse(gx, L.bx);
        appendIntsMaybeReverse(gx, L.ax);
        appendIntsMaybeReverse(gy, L.by);
        appendIntsMaybeReverse(gy, L.ay);
        appendIntsMaybeReverse(gz, L.bz);
        appendIntsMaybeReverse(gz, L.az);
        appendIntsMaybeReverse(granks, L.bFaceRanks);
        appendIntsMaybeReverse(granks, L.aFaceRanks);
        appendBytesMaybeReverse(goffsets, L.bOffsets);
        appendBytesMaybeReverse(goffsets, L.aOffsets);
        appendRingsMaybeReverse(grings, L.bRings);
        appendRingsMaybeReverse(grings, L.aRings);
        appendRingsMaybeReverse(greconRingIds, L.bReconRingIds);
        appendRingsMaybeReverse(greconRingIds, L.aReconRingIds);
        appendInputsMaybeReverse(ginputs, L.bInputs);
        appendInputsMaybeReverse(ginputs, L.aInputs);
    }

    auto encodeLayeredResiduals = [&]() {
        std::vector<char> buf(std::max<size_t>(4096, gx.size() * 12 + 1024), 0);
        size_t off = 0;
        pmsf::MultiModelRangeEncoder encoder(buf.data(), &off);

        std::vector<int32_t> meta;
        meta.push_back(static_cast<int32_t>(layers.size()));

        auto encodeAxis = [&](const std::vector<int>& vals, int32_t& mn, int32_t& range) {
            if (vals.empty()) {
                mn = 0;
                range = 0;
                return;
            }
            auto mm = std::minmax_element(vals.begin(), vals.end());
            mn = static_cast<int32_t>(*mm.first);
            const int32_t mx = static_cast<int32_t>(*mm.second);
            range = mx - mn + 1;
            if (range <= 1) return;
            if (range > 65536) {
                throw std::runtime_error(
                    "[PCLTTMStrict] layered residual range exceeds qsmodel guard");
            }
            const int lgTotf = std::max(10, ceilLog2Positive(range));
            const int rescale = 1 << std::min(15, std::max(1, lgTotf - 1));
            const int model = encoder.addModel(
                {static_cast<int>(range), lgTotf, rescale});
            for (int v : vals) {
                encoder.encode(model, static_cast<int>(v - mn));
            }
        };

        for (int li = static_cast<int>(layers.size()) - 1; li >= 0; --li) {
            LayerStreams& L = layers[static_cast<size_t>(li)];
            std::vector<int> lx, ly, lz;
            lx.reserve(L.bx.size() + L.ax.size());
            ly.reserve(L.by.size() + L.ay.size());
            lz.reserve(L.bz.size() + L.az.size());
            appendIntsMaybeReverse(lx, L.bx);
            appendIntsMaybeReverse(lx, L.ax);
            appendIntsMaybeReverse(ly, L.by);
            appendIntsMaybeReverse(ly, L.ay);
            appendIntsMaybeReverse(lz, L.bz);
            appendIntsMaybeReverse(lz, L.az);
            int32_t minX = 0, rangeX = 0, minY = 0, rangeY = 0, minZ = 0, rangeZ = 0;
            encodeAxis(lx, minX, rangeX);
            encodeAxis(ly, minY, rangeY);
            encodeAxis(lz, minZ, rangeZ);
            meta.push_back(static_cast<int32_t>(lx.size()));
            meta.push_back(minX);
            meta.push_back(rangeX);
            meta.push_back(minY);
            meta.push_back(rangeY);
            meta.push_back(minZ);
            meta.push_back(rangeZ);
        }
        encoder.finish();

        std::vector<uint8_t> out;
        out.resize(meta.size() * sizeof(int32_t));
        std::memcpy(out.data(), meta.data(), out.size());
        out.insert(out.end(),
                   reinterpret_cast<uint8_t*>(buf.data()),
                   reinterpret_cast<uint8_t*>(buf.data()) + off);
        return out;
    };
    std::vector<uint8_t> geom;
    const char* residualCodecEnv = std::getenv("PMSF_PCLTTM_RESIDUAL_CODEC");
    if (residualCodecEnv && std::string(residualCodecEnv) == "bitplane") {
        const int bpBits = std::max(1, static_cast<int>(lastResult_.mesh->i_nbQuantBits) + 1);
        geom = encodeResidualBitplane(gx, gy, gz, bpBits);
    } else {
        geom = encodeLayeredResiduals();
    }

    auto encodeLegacyOffsets = [&]() {
        std::deque<uint8_t> syms;
        for (uint8_t v : goffsets) {
            syms.push_back(static_cast<uint8_t>(v & 0x0f));
        }
        std::vector<char> buf(std::max<size_t>(4096, goffsets.size() * 2 + 64));
        size_t rcSize = pmsf::encodeUint8Queue(syms, 16, buf.data(), buf.size());
        std::vector<uint8_t> out(sizeof(int32_t));
        int32_t n = static_cast<int32_t>(goffsets.size());
        std::memcpy(out.data(), &n, sizeof(int32_t));
        out.insert(out.end(),
                   reinterpret_cast<uint8_t*>(buf.data()),
                   reinterpret_cast<uint8_t*>(buf.data()) + rcSize);
        return out;
    };
    auto encodeSplitOffsets = [&]() {
        auto encodeSmallAlphabet = [&](int alphabet,
                                       const std::function<int(uint8_t)>& symbolOf) {
            std::deque<uint8_t> syms;
            for (uint8_t v : goffsets) {
                syms.push_back(static_cast<uint8_t>(symbolOf(v)));
            }
            std::vector<char> buf(std::max<size_t>(4096, goffsets.size() * 2 + 64));
            size_t rcSize = pmsf::encodeUint8Queue(syms, alphabet, buf.data(), buf.size());
            return std::vector<uint8_t>(reinterpret_cast<uint8_t*>(buf.data()),
                                        reinterpret_cast<uint8_t*>(buf.data()) + rcSize);
        };

        // MEPP2 valence storage keeps connectivity and reverse/other-info as
        // separate binary streams. PCLTTM's analogous information is the
        // active input-edge offset, one reverse bit, and a tiny side relation
        // that disambiguates which side of the input edge owns the patch.
        const std::vector<uint8_t> offsetRc =
            encodeSmallAlphabet(8, [](uint8_t v) { return static_cast<int>(v & 0x07); });
        const std::vector<uint8_t> reverseRc =
            encodeSmallAlphabet(2, [](uint8_t v) { return (v & 0x08) ? 1 : 0; });
        const std::vector<uint8_t> sideRc =
            encodeSmallAlphabet(4, [](uint8_t v) { return static_cast<int>((v >> 4) & 0x03); });

        std::vector<uint8_t> out(sizeof(int32_t) * 5);
        size_t off = 0;
        const int32_t marker = -4;
        const int32_t n = static_cast<int32_t>(goffsets.size());
        const int32_t offsetBytes = static_cast<int32_t>(offsetRc.size());
        const int32_t reverseBytes = static_cast<int32_t>(reverseRc.size());
        const int32_t sideBytes = static_cast<int32_t>(sideRc.size());
        std::memcpy(out.data() + off, &marker, sizeof(int32_t));
        off += sizeof(int32_t);
        std::memcpy(out.data() + off, &n, sizeof(int32_t));
        off += sizeof(int32_t);
        std::memcpy(out.data() + off, &offsetBytes, sizeof(int32_t));
        off += sizeof(int32_t);
        std::memcpy(out.data() + off, &reverseBytes, sizeof(int32_t));
        off += sizeof(int32_t);
        std::memcpy(out.data() + off, &sideBytes, sizeof(int32_t));
        off += sizeof(int32_t);
        out.resize(off);
        out.insert(out.end(), offsetRc.begin(), offsetRc.end());
        out.insert(out.end(), reverseRc.begin(), reverseRc.end());
        out.insert(out.end(), sideRc.begin(), sideRc.end());
        return out;
    };
    auto encodeOffsets = [&]() {
        const char* codec = std::getenv("PMSF_PCLTTM_OFFSET_CODEC");
        if (codec && std::string(codec) == "legacy") return encodeLegacyOffsets();
        return encodeSplitOffsets();
    };
    const char* writeOffsetsEnv = std::getenv("PMSF_PCLTTM_WRITE_OFFSETS");
    const bool writeOffsets =
        writeOffsetsEnv && writeOffsetsEnv[0] == '1' &&
        !goffsets.empty() && goffsets.size() == gx.size();
    std::vector<uint8_t> offsets = writeOffsets ? encodeOffsets() : std::vector<uint8_t>();
    auto encodeRawRings = [&]() {
        size_t bytes = sizeof(int32_t) * 2;
        for (const auto& r : grings) {
            const size_t ringVerts = r.empty() ? 0 : (r.size() - 1) / 3;
            bytes += 1 + ringVerts * 3 * sizeof(uint16_t);
        }
        std::vector<uint8_t> out(bytes);
        size_t off = 0;
        int32_t marker = -1;
        int32_t n = static_cast<int32_t>(grings.size());
        std::memcpy(out.data() + off, &marker, sizeof(int32_t));
        off += sizeof(int32_t);
        std::memcpy(out.data() + off, &n, sizeof(int32_t));
        off += sizeof(int32_t);
        for (const auto& r : grings) {
            const uint8_t ringVerts =
                static_cast<uint8_t>(r.empty() ? 0 : std::min<size_t>(16, (r.size() - 1) / 3));
            out[off++] = ringVerts;
            for (size_t i = 0; i < static_cast<size_t>(ringVerts) * 3; ++i) {
                int32_t v = r[1 + i];
                if (v < 0) v = 0;
                if (v > 65535) v = 65535;
                uint16_t u = static_cast<uint16_t>(v);
                std::memcpy(out.data() + off, &u, sizeof(uint16_t));
                off += sizeof(uint16_t);
            }
        }
        out.resize(off);
        return out;
    };
    auto encodeDeltaRings = [&]() {
        int32_t numRings = static_cast<int32_t>(grings.size());
        size_t totalRingVerts = 0;
        std::vector<uint8_t> sizes;
        sizes.reserve(grings.size());
        for (const auto& r : grings) {
            const uint8_t ringVerts =
                static_cast<uint8_t>(r.empty() ? 0 : std::min<size_t>(16, (r.size() - 1) / 3));
            sizes.push_back(ringVerts);
            totalRingVerts += ringVerts;
        }

        std::vector<char> rcBuf(std::max<size_t>(4096, totalRingVerts * 3 * 3 + 1024), 0);
        size_t rcOff = 0;
        pmsf::MultiModelRangeEncoder encoder(rcBuf.data(), &rcOff);
        const int mx = encoder.addModel({65536, 16, 1 << 15});
        const int my = encoder.addModel({65536, 16, 1 << 15});
        const int mz = encoder.addModel({65536, 16, 1 << 15});
        std::array<uint16_t, 3> prev{0, 0, 0};
        auto encodeSym = [&](int model, uint16_t v, int axis) {
            const uint16_t sym =
                static_cast<uint16_t>(v - prev[static_cast<size_t>(axis)]);
            encoder.encode(model, static_cast<int>(sym));
            prev[static_cast<size_t>(axis)] = v;
        };
        for (size_t ri = 0; ri < grings.size(); ++ri) {
            const auto& r = grings[ri];
            const uint8_t ringVerts = sizes[ri];
            for (size_t vi = 0; vi < ringVerts; ++vi) {
                for (int axis = 0; axis < 3; ++axis) {
                    int32_t raw = r[1 + vi * 3 + static_cast<size_t>(axis)];
                    if (raw < 0) raw = 0;
                    if (raw > 65535) raw = 65535;
                    if (axis == 0) encodeSym(mx, static_cast<uint16_t>(raw), axis);
                    else if (axis == 1) encodeSym(my, static_cast<uint16_t>(raw), axis);
                    else encodeSym(mz, static_cast<uint16_t>(raw), axis);
                }
            }
        }
        encoder.finish();

        std::vector<uint8_t> out(sizeof(int32_t) * 2 + sizes.size());
        size_t off = 0;
        int32_t marker = -2;
        std::memcpy(out.data() + off, &marker, sizeof(int32_t));
        off += sizeof(int32_t);
        std::memcpy(out.data() + off, &numRings, sizeof(int32_t));
        off += sizeof(int32_t);
        out.resize(off);
        out.insert(out.end(), sizes.begin(), sizes.end());
        out.insert(out.end(),
                   reinterpret_cast<uint8_t*>(rcBuf.data()),
                   reinterpret_cast<uint8_t*>(rcBuf.data()) + rcOff);
        return out;
    };
    auto encodeDictionaryRings = [&]() {
        std::map<std::array<uint16_t, 3>, uint32_t> ids;
        std::vector<std::array<uint16_t, 3>> dict;
        std::vector<std::vector<uint32_t>> ringIds;
        ringIds.reserve(grings.size());
        auto clampU16 = [](int32_t v) {
            if (v < 0) return static_cast<uint16_t>(0);
            if (v > 65535) return static_cast<uint16_t>(65535);
            return static_cast<uint16_t>(v);
        };
        for (const auto& r : grings) {
            const uint8_t ringVerts =
                static_cast<uint8_t>(r.empty() ? 0 : std::min<size_t>(16, (r.size() - 1) / 3));
            std::vector<uint32_t> cur;
            cur.reserve(ringVerts);
            for (size_t vi = 0; vi < ringVerts; ++vi) {
                std::array<uint16_t, 3> q{
                    clampU16(r[1 + vi * 3]),
                    clampU16(r[1 + vi * 3 + 1]),
                    clampU16(r[1 + vi * 3 + 2]),
                };
                auto [it, inserted] =
                    ids.emplace(q, static_cast<uint32_t>(dict.size()));
                if (inserted) dict.push_back(q);
                cur.push_back(it->second);
            }
            ringIds.push_back(std::move(cur));
        }
        const bool wideIds = dict.size() > 65535;
        size_t bytes = sizeof(int32_t) * 3 + dict.size() * 3 * sizeof(uint16_t);
        bytes += ringIds.size();
        for (const auto& r : ringIds) {
            bytes += r.size() * (wideIds ? sizeof(uint32_t) : sizeof(uint16_t));
        }
        std::vector<uint8_t> out(bytes);
        size_t off = 0;
        const int32_t marker = wideIds ? -6 : -5;
        const int32_t n = static_cast<int32_t>(ringIds.size());
        const int32_t dictN = static_cast<int32_t>(dict.size());
        std::memcpy(out.data() + off, &marker, sizeof(int32_t));
        off += sizeof(int32_t);
        std::memcpy(out.data() + off, &n, sizeof(int32_t));
        off += sizeof(int32_t);
        std::memcpy(out.data() + off, &dictN, sizeof(int32_t));
        off += sizeof(int32_t);
        for (const auto& q : dict) {
            for (uint16_t v : q) {
                std::memcpy(out.data() + off, &v, sizeof(uint16_t));
                off += sizeof(uint16_t);
            }
        }
        for (const auto& r : ringIds) {
            out[off++] = static_cast<uint8_t>(std::min<size_t>(255, r.size()));
        }
        for (const auto& r : ringIds) {
            for (uint32_t id : r) {
                if (wideIds) {
                    std::memcpy(out.data() + off, &id, sizeof(uint32_t));
                    off += sizeof(uint32_t);
                } else {
                    const uint16_t u = static_cast<uint16_t>(id);
                    std::memcpy(out.data() + off, &u, sizeof(uint16_t));
                    off += sizeof(uint16_t);
                }
            }
        }
        out.resize(off);
        return out;
    };
    auto encodeDictionaryVarintRings = [&]() {
        auto putVarU32 = [](std::vector<uint8_t>& out, uint32_t v) {
            while (v >= 0x80u) {
                out.push_back(static_cast<uint8_t>((v & 0x7fu) | 0x80u));
                v >>= 7;
            }
            out.push_back(static_cast<uint8_t>(v));
        };
        auto putZigZag = [&](std::vector<uint8_t>& out, int32_t v) {
            const uint32_t z = (static_cast<uint32_t>(v) << 1) ^
                               static_cast<uint32_t>(v >> 31);
            putVarU32(out, z);
        };
        std::map<std::array<uint16_t, 3>, uint32_t> ids;
        std::vector<std::array<uint16_t, 3>> dict;
        std::vector<std::vector<uint32_t>> ringIds;
        ringIds.reserve(grings.size());
        auto clampU16 = [](int32_t v) {
            if (v < 0) return static_cast<uint16_t>(0);
            if (v > 65535) return static_cast<uint16_t>(65535);
            return static_cast<uint16_t>(v);
        };
        for (const auto& r : grings) {
            const uint8_t ringVerts =
                static_cast<uint8_t>(r.empty() ? 0 : std::min<size_t>(16, (r.size() - 1) / 3));
            std::vector<uint32_t> cur;
            cur.reserve(ringVerts);
            for (size_t vi = 0; vi < ringVerts; ++vi) {
                std::array<uint16_t, 3> q{
                    clampU16(r[1 + vi * 3]),
                    clampU16(r[1 + vi * 3 + 1]),
                    clampU16(r[1 + vi * 3 + 2]),
                };
                auto [it, inserted] =
                    ids.emplace(q, static_cast<uint32_t>(dict.size()));
                if (inserted) dict.push_back(q);
                cur.push_back(it->second);
            }
            ringIds.push_back(std::move(cur));
        }
        std::vector<uint8_t> out;
        out.reserve(dict.size() * 4 + ringIds.size() * 8);
        const int32_t marker = -7;
        const int32_t n = static_cast<int32_t>(ringIds.size());
        const int32_t dictN = static_cast<int32_t>(dict.size());
        for (int32_t v : {marker, n, dictN}) {
            const auto u = static_cast<uint32_t>(v);
            out.push_back(static_cast<uint8_t>(u & 0xffu));
            out.push_back(static_cast<uint8_t>((u >> 8) & 0xffu));
            out.push_back(static_cast<uint8_t>((u >> 16) & 0xffu));
            out.push_back(static_cast<uint8_t>((u >> 24) & 0xffu));
        }
        std::array<int32_t, 3> prevQ{0, 0, 0};
        for (const auto& q : dict) {
            for (int axis = 0; axis < 3; ++axis) {
                const int32_t cur = static_cast<int32_t>(q[static_cast<size_t>(axis)]);
                putZigZag(out, cur - prevQ[static_cast<size_t>(axis)]);
                prevQ[static_cast<size_t>(axis)] = cur;
            }
        }
        for (const auto& r : ringIds) {
            out.push_back(static_cast<uint8_t>(std::min<size_t>(255, r.size())));
        }
        for (const auto& r : ringIds) {
            int32_t prev = 0;
            for (uint32_t id : r) {
                const int32_t cur = static_cast<int32_t>(id);
                putZigZag(out, cur - prev);
                prev = cur;
            }
        }
        return out;
    };
    auto encodeRings = [&]() {
        const char* codec = std::getenv("PMSF_PCLTTM_RING_CODEC");
        if (codec && std::string(codec) == "ids_varint") return encodeDictionaryVarintRings();
        if (codec && std::string(codec) == "ids") return encodeDictionaryRings();
        if (codec && std::string(codec) == "raw") return encodeRawRings();
        return encodeDeltaRings();
    };
    auto encodeReconRingIds = [&]() {
        auto putI32 = [](std::vector<uint8_t>& out, int32_t v) {
            const size_t off = out.size();
            out.resize(off + sizeof(int32_t));
            std::memcpy(out.data() + off, &v, sizeof(int32_t));
        };
        auto putVarU32 = [](std::vector<uint8_t>& out, uint32_t v) {
            while (v >= 0x80u) {
                out.push_back(static_cast<uint8_t>((v & 0x7fu) | 0x80u));
                v >>= 7;
            }
            out.push_back(static_cast<uint8_t>(v));
        };
        auto putZigZag = [&](std::vector<uint8_t>& out, int32_t v) {
            const uint32_t z = (static_cast<uint32_t>(v) << 1) ^
                               static_cast<uint32_t>(v >> 31);
            putVarU32(out, z);
        };
        std::vector<uint8_t> out;
        const char* originalIdsEnv =
            std::getenv("PMSF_PCLTTM_WRITE_RECON_ORIGINAL_IDS");
        const bool writeOriginalIds =
            (!originalIdsEnv || originalIdsEnv[0] != '0') &&
            !greconOriginalIds.empty();
        putI32(out, writeOriginalIds ? -12 : -11);
        putI32(out, static_cast<int32_t>(greconRingIds.size()));
        if (writeOriginalIds) {
            putI32(out, static_cast<int32_t>(greconOriginalIds.size()));
        }
        for (const auto& r : greconRingIds) {
            out.push_back(static_cast<uint8_t>(std::min<size_t>(255, r.size())));
        }
        for (const auto& r : greconRingIds) {
            int32_t prev = 0;
            for (int32_t id : r) {
                putZigZag(out, id - prev);
                prev = id;
            }
        }
        if (writeOriginalIds) {
            int32_t prev = 0;
            for (int32_t id : greconOriginalIds) {
                putZigZag(out, id - prev);
                prev = id;
            }
        }
        return out;
    };
    auto encodeFaceRanks = [&]() {
        auto putI32 = [](std::vector<uint8_t>& out, int32_t v) {
            const size_t off = out.size();
            out.resize(off + sizeof(int32_t));
            std::memcpy(out.data() + off, &v, sizeof(int32_t));
        };
        auto putVarU32 = [](std::vector<uint8_t>& out, uint32_t v) {
            while (v >= 0x80u) {
                out.push_back(static_cast<uint8_t>((v & 0x7fu) | 0x80u));
                v >>= 7;
            }
            out.push_back(static_cast<uint8_t>(v));
        };
        auto putZigZag = [&](std::vector<uint8_t>& out, int32_t v) {
            const uint32_t z = (static_cast<uint32_t>(v) << 1) ^
                               static_cast<uint32_t>(v >> 31);
            putVarU32(out, z);
        };
        const char* codec = std::getenv("PMSF_PCLTTM_RANK_CODEC");
        auto strictValence = [](uint8_t s) {
            switch (s) {
                case kSymV3: return 3;
                case kSymV4: return 4;
                case kSymV5: return 5;
                case kSymV6: return 6;
                default: return 0;
            }
        };
        auto buildGroupedRankDeltas = [&]() {
            std::vector<int32_t> deltas;
            deltas.reserve(granks.size());
            size_t rankCursor = 0;
            int32_t prevFirst = 0;
            for (uint8_t sym : packed) {
                const int valence = strictValence(sym);
                if (valence <= 0) continue;
                const int n = std::max(1, valence - 2);
                if (rankCursor >= granks.size()) break;
                int32_t prev = static_cast<int32_t>(granks[rankCursor++]);
                deltas.push_back(prev - prevFirst);
                prevFirst = prev;
                for (int k = 1; k < n && rankCursor < granks.size(); ++k) {
                    const int32_t cur = static_cast<int32_t>(granks[rankCursor++]);
                    deltas.push_back(cur - prev);
                    prev = cur;
                }
            }
            return deltas;
        };
        auto encodeGroupedRankBitplanePayload = [&](const std::vector<int32_t>& deltas,
                                                    int& bpBitsOut) {
            uint32_t maxZ = 0;
            for (int32_t d : deltas) {
                const uint32_t z = (static_cast<uint32_t>(d) << 1) ^
                                   static_cast<uint32_t>(d >> 31);
                maxZ = std::max(maxZ, z);
            }
            const int bpBits = std::max(1, ceilLog2Positive(static_cast<int>(maxZ) + 1));
            std::vector<char> buf(std::max<size_t>(4096, deltas.size() * 4 + 1024), 0);
            size_t rcOff = 0;
            pmsf::MultiModelRangeEncoder encoder(buf.data(), &rcOff);
            const int firstModel = encoder.addModel({2, 10, 1 << 9});
            const int afterModel = encoder.addModel({2, 10, 1 << 9});
            for (int32_t d : deltas) {
                uint32_t z = (static_cast<uint32_t>(d) << 1) ^
                             static_cast<uint32_t>(d >> 31);
                bool seenOne = false;
                for (int b = bpBits - 1; b >= 0; --b) {
                    const int bit = static_cast<int>((z >> b) & 1u);
                    encoder.encode(seenOne ? afterModel : firstModel, bit);
                    if (bit) seenOne = true;
                }
            }
            encoder.finish();
            bpBitsOut = bpBits;
            return std::vector<uint8_t>(reinterpret_cast<uint8_t*>(buf.data()),
                                        reinterpret_cast<uint8_t*>(buf.data()) + rcOff);
        };
        auto countRankGroups = [&]() {
            if (locatorRankPrefix >= 0) {
                return static_cast<int32_t>(std::max(0, locatorRankPrefix));
            }
            int32_t groups = 0;
            for (uint8_t sym : packed) {
                if (strictValence(sym) > 0) ++groups;
            }
            return groups;
        };
        auto encodeAptmLocatorV1 = [&]() {
            std::vector<int32_t> deltas = buildGroupedRankDeltas();
            std::vector<uint8_t> varintPayload;
            varintPayload.reserve(deltas.size() * 2);
            for (int32_t d : deltas) putZigZag(varintPayload, d);

            std::vector<uint8_t> zPayload;
            const bool zOk = zlibCompress(varintPayload.data(),
                                          varintPayload.size(),
                                          zPayload,
                                          Z_BEST_COMPRESSION) &&
                             zPayload.size() < varintPayload.size();

            int bpBits = 0;
            std::vector<uint8_t> bitplanePayload =
                encodeGroupedRankBitplanePayload(deltas, bpBits);

            enum : int32_t {
                kModeBitplane = 0,
                kModeVarint = 1,
                kModeZlibVarint = 2,
            };
            int32_t mode = kModeBitplane;
            int32_t aux = bpBits;
            const std::vector<uint8_t>* payload = &bitplanePayload;
            if (varintPayload.size() < payload->size()) {
                mode = kModeVarint;
                aux = static_cast<int32_t>(varintPayload.size());
                payload = &varintPayload;
            }
            if (zOk && zPayload.size() < payload->size()) {
                mode = kModeZlibVarint;
                aux = static_cast<int32_t>(varintPayload.size());
                payload = &zPayload;
            }
            if (const char* forced = std::getenv("PMSF_PCLTTM_LOCATOR_MODE")) {
                const std::string fm(forced);
                if (fm == "varint") {
                    mode = kModeVarint;
                    aux = static_cast<int32_t>(varintPayload.size());
                    payload = &varintPayload;
                } else if (fm == "zlib_varint" && zOk) {
                    mode = kModeZlibVarint;
                    aux = static_cast<int32_t>(varintPayload.size());
                    payload = &zPayload;
                } else if (fm == "bitplane") {
                    mode = kModeBitplane;
                    aux = bpBits;
                    payload = &bitplanePayload;
                }
            }

            std::vector<uint8_t> out;
            out.reserve(sizeof(int32_t) * 5 + payload->size());
            putI32(out, -13);
            putI32(out, countRankGroups());
            putI32(out, static_cast<int32_t>(deltas.size()));
            putI32(out, mode);
            putI32(out, aux);
            out.insert(out.end(), payload->begin(), payload->end());
            return out;
        };
        auto encodeRingSubsetVarint = [&](size_t startRing) {
            auto putVarU32Local = [](std::vector<uint8_t>& out, uint32_t v) {
                while (v >= 0x80u) {
                    out.push_back(static_cast<uint8_t>((v & 0x7fu) | 0x80u));
                    v >>= 7;
                }
                out.push_back(static_cast<uint8_t>(v));
            };
            auto putZigZagLocal = [&](std::vector<uint8_t>& out, int32_t v) {
                const uint32_t z = (static_cast<uint32_t>(v) << 1) ^
                                   static_cast<uint32_t>(v >> 31);
                putVarU32Local(out, z);
            };
            std::map<std::array<uint16_t, 3>, uint32_t> ids;
            std::vector<std::array<uint16_t, 3>> dict;
            std::vector<std::vector<uint32_t>> ringIds;
            if (startRing > grings.size()) startRing = grings.size();
            ringIds.reserve(grings.size() - startRing);
            auto clampU16 = [](int32_t v) {
                if (v < 0) return static_cast<uint16_t>(0);
                if (v > 65535) return static_cast<uint16_t>(65535);
                return static_cast<uint16_t>(v);
            };
            for (size_t ri = startRing; ri < grings.size(); ++ri) {
                const auto& r = grings[ri];
                const uint8_t ringVerts =
                    static_cast<uint8_t>(r.empty() ? 0 : std::min<size_t>(16, (r.size() - 1) / 3));
                std::vector<uint32_t> cur;
                cur.reserve(ringVerts);
                for (size_t vi = 0; vi < ringVerts; ++vi) {
                    std::array<uint16_t, 3> q{
                        clampU16(r[1 + vi * 3]),
                        clampU16(r[1 + vi * 3 + 1]),
                        clampU16(r[1 + vi * 3 + 2]),
                    };
                    auto [it, inserted] =
                        ids.emplace(q, static_cast<uint32_t>(dict.size()));
                    if (inserted) dict.push_back(q);
                    cur.push_back(it->second);
                }
                ringIds.push_back(std::move(cur));
            }
            std::vector<uint8_t> out;
            out.reserve(dict.size() * 4 + ringIds.size() * 8);
            for (int32_t v : {static_cast<int32_t>(-7),
                              static_cast<int32_t>(ringIds.size()),
                              static_cast<int32_t>(dict.size())}) {
                putI32(out, v);
            }
            std::array<int32_t, 3> prevQ{0, 0, 0};
            for (const auto& q : dict) {
                for (int axis = 0; axis < 3; ++axis) {
                    const int32_t cur = static_cast<int32_t>(q[static_cast<size_t>(axis)]);
                    putZigZagLocal(out, cur - prevQ[static_cast<size_t>(axis)]);
                    prevQ[static_cast<size_t>(axis)] = cur;
                }
            }
            for (const auto& r : ringIds) {
                out.push_back(static_cast<uint8_t>(std::min<size_t>(255, r.size())));
            }
            for (const auto& r : ringIds) {
                int32_t prev = 0;
                for (uint32_t id : r) {
                    const int32_t cur = static_cast<int32_t>(id);
                    putZigZagLocal(out, cur - prev);
                    prev = cur;
                }
            }
            return out;
        };
        auto encodeReconRingSubsetVarint = [&](size_t startRing) {
            std::vector<uint8_t> out;
            auto putVarU32Local = [](std::vector<uint8_t>& out, uint32_t v) {
                while (v >= 0x80u) {
                    out.push_back(static_cast<uint8_t>((v & 0x7fu) | 0x80u));
                    v >>= 7;
                }
                out.push_back(static_cast<uint8_t>(v));
            };
            auto putZigZagLocal = [&](std::vector<uint8_t>& out, int32_t v) {
                const uint32_t z = (static_cast<uint32_t>(v) << 1) ^
                                   static_cast<uint32_t>(v >> 31);
                putVarU32Local(out, z);
            };
            if (startRing > greconRingIds.size()) startRing = greconRingIds.size();
            const int32_t n = static_cast<int32_t>(greconRingIds.size() - startRing);
            out.reserve(static_cast<size_t>(n) * 8 + sizeof(int32_t) * 2);
            putI32(out, -11);
            putI32(out, n);
            for (size_t ri = startRing; ri < greconRingIds.size(); ++ri) {
                out.push_back(static_cast<uint8_t>(
                    std::min<size_t>(255, greconRingIds[ri].size())));
            }
            for (size_t ri = startRing; ri < greconRingIds.size(); ++ri) {
                int32_t prev = 0;
                for (int32_t id : greconRingIds[ri]) {
                    putZigZagLocal(out, id - prev);
                    prev = id;
                }
            }
            return out;
        };
        auto encodeAptmHybridLocator = [&]() {
            std::vector<uint8_t> rankBlob = encodeAptmLocatorV1();
            const bool useReconIds =
                greconRingIds.size() == gx.size() &&
                std::getenv("PMSF_PCLTTM_HYBRID_Q_RINGS") == nullptr;
            std::vector<uint8_t> ringBlob =
                useReconIds
                    ? encodeReconRingSubsetVarint(
                          static_cast<size_t>(std::max(0, locatorRankPrefix)))
                    : encodeRingSubsetVarint(
                          static_cast<size_t>(std::max(0, locatorRankPrefix)));
            std::vector<uint8_t> out;
            out.reserve(sizeof(int32_t) * 4 + rankBlob.size() + ringBlob.size());
            putI32(out, useReconIds ? -15 : -14);
            putI32(out, static_cast<int32_t>(std::max(0, locatorRankPrefix)));
            putI32(out, static_cast<int32_t>(rankBlob.size()));
            putI32(out, static_cast<int32_t>(ringBlob.size()));
            out.insert(out.end(), rankBlob.begin(), rankBlob.end());
            out.insert(out.end(), ringBlob.begin(), ringBlob.end());
            return out;
        };
        auto encodeSparseRingTailLocator = [&]() {
            std::vector<uint8_t> ringBlob =
                encodeRingSubsetVarint(
                    static_cast<size_t>(std::max(0, locatorRingPrefix)));
            std::vector<uint8_t> out;
            out.reserve(sizeof(int32_t) * 3 + ringBlob.size());
            putI32(out, -16);
            putI32(out, static_cast<int32_t>(std::max(0, locatorRingPrefix)));
            putI32(out, static_cast<int32_t>(ringBlob.size()));
            out.insert(out.end(), ringBlob.begin(), ringBlob.end());
            return out;
        };
        if (codec && std::string(codec) == "aptm_locator_v1") {
            if (locatorRankPrefix > 0 &&
                locatorRankPrefix < static_cast<int>(gx.size()) &&
                grings.size() == gx.size()) {
                return encodeAptmHybridLocator();
            }
            return encodeAptmLocatorV1();
        }
        if (codec && std::string(codec) == "group_bitplane" &&
            granks.size() > gx.size()) {
            std::vector<int32_t> deltas = buildGroupedRankDeltas();
            int bpBits = 0;
            std::vector<uint8_t> payload =
                encodeGroupedRankBitplanePayload(deltas, bpBits);
            std::vector<uint8_t> out;
            out.reserve(12 + payload.size());
            putI32(out, -10);
            putI32(out, static_cast<int32_t>(deltas.size()));
            putI32(out, static_cast<int32_t>(bpBits));
            out.insert(out.end(), payload.begin(), payload.end());
            return out;
        }
        if (codec && std::string(codec) == "group_varint" &&
            granks.size() > gx.size()) {
            std::vector<uint8_t> out;
            out.reserve(granks.size() + 8);
            putI32(out, -9);
            putI32(out, static_cast<int32_t>(granks.size()));
            for (int32_t d : buildGroupedRankDeltas()) {
                putZigZag(out, d);
            }
            return out;
        }
        if (codec && std::string(codec) == "varint") {
            std::vector<uint8_t> out;
            out.reserve(granks.size() * 2 + 8);
            putI32(out, -8);
            putI32(out, static_cast<int32_t>(granks.size()));
            int32_t prev = 0;
            for (int v : granks) {
                const int32_t cur = static_cast<int32_t>(v);
                putZigZag(out, cur - prev);
                prev = cur;
            }
            return out;
        }
        std::vector<uint8_t> out(sizeof(int32_t) * (2 + granks.size()));
        size_t off = 0;
        int32_t marker = -3;
        int32_t n = static_cast<int32_t>(granks.size());
        std::memcpy(out.data() + off, &marker, sizeof(int32_t));
        off += sizeof(int32_t);
        std::memcpy(out.data() + off, &n, sizeof(int32_t));
        off += sizeof(int32_t);
        for (int v : granks) {
            int32_t x = static_cast<int32_t>(v);
            std::memcpy(out.data() + off, &x, sizeof(int32_t));
            off += sizeof(int32_t);
        }
        out.resize(off);
        return out;
    };
    auto encodeSparseRingTailLocator = [&]() {
        auto putI32Local = [](std::vector<uint8_t>& out, int32_t v) {
            const size_t off = out.size();
            out.resize(off + sizeof(int32_t));
            std::memcpy(out.data() + off, &v, sizeof(int32_t));
        };
        auto putVarU32Local = [](std::vector<uint8_t>& out, uint32_t v) {
            while (v >= 0x80u) {
                out.push_back(static_cast<uint8_t>((v & 0x7fu) | 0x80u));
                v >>= 7;
            }
            out.push_back(static_cast<uint8_t>(v));
        };
        auto putZigZagLocal = [&](std::vector<uint8_t>& out, int32_t v) {
            const uint32_t z = (static_cast<uint32_t>(v) << 1) ^
                               static_cast<uint32_t>(v >> 31);
            putVarU32Local(out, z);
        };
        const size_t startRing = static_cast<size_t>(std::max(0, locatorRingPrefix));
        std::map<std::array<uint16_t, 3>, uint32_t> ids;
        std::vector<std::array<uint16_t, 3>> dict;
        std::vector<std::vector<uint32_t>> ringIds;
        auto clampU16 = [](int32_t v) {
            if (v < 0) return static_cast<uint16_t>(0);
            if (v > 65535) return static_cast<uint16_t>(65535);
            return static_cast<uint16_t>(v);
        };
        for (size_t ri = std::min(startRing, grings.size()); ri < grings.size(); ++ri) {
            const auto& r = grings[ri];
            const uint8_t ringVerts =
                static_cast<uint8_t>(r.empty() ? 0 : std::min<size_t>(16, (r.size() - 1) / 3));
            std::vector<uint32_t> cur;
            cur.reserve(ringVerts);
            for (size_t vi = 0; vi < ringVerts; ++vi) {
                std::array<uint16_t, 3> q{
                    clampU16(r[1 + vi * 3]),
                    clampU16(r[1 + vi * 3 + 1]),
                    clampU16(r[1 + vi * 3 + 2]),
                };
                auto [it, inserted] =
                    ids.emplace(q, static_cast<uint32_t>(dict.size()));
                if (inserted) dict.push_back(q);
                cur.push_back(it->second);
            }
            ringIds.push_back(std::move(cur));
        }
        std::vector<uint8_t> ringBlob;
        ringBlob.reserve(dict.size() * 4 + ringIds.size() * 8 + 12);
        putI32Local(ringBlob, -7);
        putI32Local(ringBlob, static_cast<int32_t>(ringIds.size()));
        putI32Local(ringBlob, static_cast<int32_t>(dict.size()));
        std::array<int32_t, 3> prevQ{0, 0, 0};
        for (const auto& q : dict) {
            for (int axis = 0; axis < 3; ++axis) {
                const int32_t cur = static_cast<int32_t>(q[static_cast<size_t>(axis)]);
                putZigZagLocal(ringBlob, cur - prevQ[static_cast<size_t>(axis)]);
                prevQ[static_cast<size_t>(axis)] = cur;
            }
        }
        for (const auto& r : ringIds) {
            ringBlob.push_back(static_cast<uint8_t>(std::min<size_t>(255, r.size())));
        }
        for (const auto& r : ringIds) {
            int32_t prev = 0;
            for (uint32_t id : r) {
                const int32_t cur = static_cast<int32_t>(id);
                putZigZagLocal(ringBlob, cur - prev);
                prev = cur;
            }
        }

        std::vector<uint8_t> out;
        out.reserve(sizeof(int32_t) * 3 + ringBlob.size());
        putI32Local(out, -16);
        putI32Local(out, static_cast<int32_t>(std::max(0, locatorRingPrefix)));
        putI32Local(out, static_cast<int32_t>(ringBlob.size()));
        out.insert(out.end(), ringBlob.begin(), ringBlob.end());
        return out;
    };
    const char* writeRingsEnv = std::getenv("PMSF_PCLTTM_WRITE_RINGS");
    const char* writeRanksEnv = std::getenv("PMSF_PCLTTM_WRITE_RANKS");
    const char* writeReconRingIdsEnv = std::getenv("PMSF_PCLTTM_WRITE_RECON_RING_IDS");
    const bool writeHybridLocator =
        locatorRankPrefix > 0 &&
        locatorRankPrefix < static_cast<int>(gx.size()) &&
        !granks.empty() && grings.size() == gx.size();
    const bool writeSparseRingTail =
        locatorRingPrefix > 0 &&
        locatorRingPrefix < static_cast<int>(gx.size()) &&
        grings.size() == gx.size();
    const bool writeReconRingIds =
        writeReconRingIdsEnv && writeReconRingIdsEnv[0] == '1' &&
        !writeSparseRingTail &&
        !greconRingIds.empty() && greconRingIds.size() == gx.size();
    const bool writeRanks =
        writeRanksEnv && writeRanksEnv[0] == '1' &&
        !writeReconRingIds && !writeSparseRingTail && !granks.empty() &&
        (granks.size() >= gx.size() || writeHybridLocator);
    const bool writeRings =
        writeRingsEnv && writeRingsEnv[0] == '1' &&
        !writeReconRingIds && !writeSparseRingTail &&
        !grings.empty() && grings.size() == gx.size();
    std::vector<uint8_t> rings = writeRings ? encodeRings() : std::vector<uint8_t>();
    std::vector<uint8_t> sparseRingTail =
        writeSparseRingTail ? encodeSparseRingTailLocator() : std::vector<uint8_t>();
    std::vector<uint8_t> reconRingIds =
        writeReconRingIds ? encodeReconRingIds() : std::vector<uint8_t>();
    std::vector<uint8_t> ranks = writeRanks ? encodeFaceRanks() : std::vector<uint8_t>();
    auto encodeSymbolInputs = [&]() {
        std::vector<uint8_t> out((lastResult_.symbolInputQs.size() * 6 + 1) * sizeof(int32_t));
        size_t off = 0;
        int32_t n = static_cast<int32_t>(lastResult_.symbolInputQs.size());
        std::memcpy(out.data() + off, &n, sizeof(int32_t));
        off += sizeof(int32_t);
        for (const auto& q : lastResult_.symbolInputQs) {
            for (int v : q) {
                int32_t x = static_cast<int32_t>(v);
                std::memcpy(out.data() + off, &x, sizeof(int32_t));
                off += sizeof(int32_t);
            }
        }
        return out;
    };
    const char* writeSymbolInputsEnv = std::getenv("PMSF_PCLTTM_WRITE_SYMBOL_INPUTS");
    const bool writeSymbolInputs =
        writeSymbolInputsEnv && writeSymbolInputsEnv[0] == '1' &&
        lastResult_.symbolInputQs.size() == lastResult_.symbols.size();
    std::vector<uint8_t> symbolInputs =
        writeSymbolInputs ? encodeSymbolInputs() : std::vector<uint8_t>();
    auto encodeVInputs = [&]() {
        std::vector<uint8_t> out(sizeof(int32_t) * 2 + ginputs.size() * 6 * sizeof(uint16_t));
        size_t off = 0;
        int32_t marker = -1;
        int32_t n = static_cast<int32_t>(ginputs.size());
        std::memcpy(out.data() + off, &marker, sizeof(int32_t));
        off += sizeof(int32_t);
        std::memcpy(out.data() + off, &n, sizeof(int32_t));
        off += sizeof(int32_t);
        for (const auto& q : ginputs) {
            for (int v : q) {
                if (v < 0) v = 0;
                if (v > 65535) v = 65535;
                uint16_t u = static_cast<uint16_t>(v);
                std::memcpy(out.data() + off, &u, sizeof(uint16_t));
                off += sizeof(uint16_t);
            }
        }
        out.resize(off);
        return out;
    };
    const char* writeVInputsEnv = std::getenv("PMSF_PCLTTM_WRITE_V_INPUTS");
    const bool writeVInputs =
        writeVInputsEnv && writeVInputsEnv[0] == '1' &&
        !writeRings && !writeSymbolInputs && ginputs.size() == gx.size();
    std::vector<uint8_t> vInputs =
        writeVInputs ? encodeVInputs() : std::vector<uint8_t>();
    const bool writeRanksAsExtraStream = writeRings && writeRanks;
    const bool writeExtraStream =
        writeSymbolInputs || writeVInputs || writeRanksAsExtraStream;
    ConnectivityBlobBuilder cbb(
        pclttmStrictConnectivityBlobSpec(
            blobLayout,
            schemeVersion,
            writeExtraStream
                ? kPaperBlobStreamCountPCLTTMStrictWithExtra
                : ((writeRings || writeRanks || writeReconRingIds ||
                    writeSparseRingTail)
                       ? kPaperBlobStreamCountPCLTTMStrictWithReplay
                       : (writeOffsets
                              ? kPaperBlobStreamCountPCLTTMStrictWithOffsets
                              : kPaperBlobStreamCountPCLTTMStrictBase))));
    cbb.appendStreamBytes(0, kPaperBlobStreamPCLTTMStrictConnectivity, conn);
    cbb.appendStreamBytes(0, kPaperBlobStreamPCLTTMStrictGeometry, geom);
    if (writeOffsets) {
        cbb.appendStreamBytes(0, kPaperBlobStreamPCLTTMStrictOffsets, offsets);
    }
    if (writeRings || writeRanks || writeReconRingIds || writeSparseRingTail) {
        if (!writeOffsets) cbb.appendStreamBytes(0, kPaperBlobStreamPCLTTMStrictOffsets, std::vector<uint8_t>(sizeof(int32_t), 0));
        cbb.appendStreamBytes(0, kPaperBlobStreamPCLTTMStrictReplay,
                              writeSparseRingTail ? sparseRingTail
                              : (writeReconRingIds ? reconRingIds
                              : (writeRings ? rings : ranks)));
    }
    if (writeExtraStream) {
        if (!writeOffsets &&
            !(writeRings || writeRanks || writeReconRingIds || writeSparseRingTail)) {
            cbb.appendStreamBytes(0, kPaperBlobStreamPCLTTMStrictOffsets, std::vector<uint8_t>(sizeof(int32_t), 0));
        }
        if (!writeRings && !writeRanks && !writeReconRingIds && !writeSparseRingTail) cbb.appendStreamBytes(0, kPaperBlobStreamPCLTTMStrictReplay, std::vector<uint8_t>(sizeof(int32_t), 0));
        cbb.appendStreamBytes(0, kPaperBlobStreamPCLTTMStrictExtra,
                              writeSymbolInputs ? symbolInputs
                              : (writeVInputs ? vInputs : ranks));
    }
    auto blob = cbb.finalize();
    std::memcpy(buffer, blob.data(), blob.size());

    lastConnectivityBytes_ = (int)conn.size();
    lastGeometryBytes_ = (int)geom.size();
    lastContainerOverheadBytes_ =
        (int)blob.size() - lastConnectivityBytes_ - lastGeometryBytes_;

    if (const char* e = std::getenv("PMSF_BPV_DIAG"); e && e[0] == '1') {
        const double invV = lastResult_.numVerticesIn > 0
                                ? (1.0 / static_cast<double>(lastResult_.numVerticesIn))
                                : 0.0;
        const double connectivityBpv =
            8.0 * static_cast<double>(lastConnectivityBytes_) * invV;
        const double residualGeometryBpv =
            8.0 * static_cast<double>(lastGeometryBytes_) * invV;
        const double connectivityPlusResidualBpv =
            8.0 * static_cast<double>(lastConnectivityBytes_ + lastGeometryBytes_) * invV;
        const double strictBlobBpv =
            8.0 * static_cast<double>(blob.size()) * invV;
        int baseDeg3 = 0, baseDeg4 = 0, baseDeg5 = 0, baseDeg6 = 0, baseDegGt6 = 0;
        int baseFaces = 0;
        if (lastResult_.mesh) {
            for (MCGAL::Vertex* v : lastResult_.mesh->vertices()) {
                if (!v || v->isRemoved()) continue;
                const int d = v->vertex_degree();
                if (d == 3) ++baseDeg3;
                else if (d == 4) ++baseDeg4;
                else if (d == 5) ++baseDeg5;
                else if (d == 6) ++baseDeg6;
                else if (d > 6) ++baseDegGt6;
            }
            for (MCGAL::Facet* f : lastResult_.mesh->faces()) {
                if (f && !f->isRemoved()) ++baseFaces;
            }
        }
        const int baseVertices = lastResult_.numVerticesOut;
        const int qBits = lastResult_.mesh
                              ? std::max(1, static_cast<int>(lastResult_.mesh->i_nbQuantBits))
                              : 12;
        const int baseIndexBits = std::max(1, ceilLog2Positive(std::max(2, baseVertices)));
        const double baseGeomBytes =
            static_cast<double>(baseVertices) * 3.0 * static_cast<double>(qBits) / 8.0;
        const double baseTopoBytes =
            static_cast<double>(baseFaces) * 3.0 * static_cast<double>(baseIndexBits) / 8.0;
        const double baseRawBytes = baseGeomBytes + baseTopoBytes;
        const double baseBpv = 8.0 * baseRawBytes * invV;
        const double totalBpvIncludingBase = strictBlobBpv + baseBpv;
        const size_t sideStreamBytes = offsets.size() + rings.size() +
                                       sparseRingTail.size() +
                                       reconRingIds.size() + ranks.size() +
                                       symbolInputs.size() + vInputs.size();
        std::cerr << "[" << (diagLabel ? diagLabel : "PCLTTMStrict")
                  << " bpv sv=" << schemeVersion << "] conn="
                  << lastConnectivityBytes_
                  << " geometryResidual=" << lastGeometryBytes_
                  << " containerOverhead=" << lastContainerOverheadBytes_
                  << " totalBlob=" << blob.size()
                  << " offsets_B=" << offsets.size()
                  << " rings_B=" << rings.size()
                  << " sparseTail_B=" << sparseRingTail.size()
                  << " reconRingIds_B=" << reconRingIds.size()
                  << " ranks_B=" << ranks.size()
                  << " symbolInputs_B=" << symbolInputs.size()
                  << " vInputs_B=" << vInputs.size()
                  << " sideStreams_B=" << sideStreamBytes
                  << " connectivity_bpv=" << connectivityBpv
                  << " residual_geometry_bpv=" << residualGeometryBpv
                  << " connectivity_plus_residual_bpv=" << connectivityPlusResidualBpv
                  << " strict_blob_bpv=" << strictBlobBpv
                  << " base_raw_B=" << baseRawBytes
                  << " base_bpv=" << baseBpv
                  << " total_bpv_including_base=" << totalBpvIncludingBase
                  << " V_in=" << lastResult_.numVerticesIn
                  << " V_base=" << lastResult_.numVerticesOut
                  << " F_base=" << baseFaces
                  << " removed=" << removed
                  << " nullBefore=" << nullBefore
                  << " nullAfter=" << nullAfter
                  << " nullPackedAway=" << nullPackedAway
                  << " A_syms=" << numA
                  << " B_syms=" << numB
                  << " layers=" << layers.size()
                  << " base_deg3=" << baseDeg3
                  << " base_deg4=" << baseDeg4
                  << " base_deg5=" << baseDeg5
                  << " base_deg6=" << baseDeg6
                  << " base_deg_gt6=" << baseDegGt6
                  << "\n";
    }

    return (int)blob.size();
}

int PCLTTMStrictSymbolCollectOperator::exportToBufferWithRangeCoder(
    char* buffer, bool /*enableQuantization*/) {
    const auto spec = pmsf::pclttmStrictConnectivityBlobSpec(
        pmsf::kPaperBlobLayoutPCLTTMStrict,
        pmsf::kPaperBlobSchemeVersionPCLTTMStrict,
        pmsf::kPaperBlobStreamCountPCLTTMStrictBase);
    return exportStrictToBufferWithLayout(buffer,
                                          spec.blobLayout,
                                          spec.schemeVersion,
                                          "PCLTTMStrict");
}

int PCLTTMStrictSymbolCollectOperator::exportToBuffer(
    char* /*buffer*/, bool /*enableQuantization*/, bool /*enableCompress*/) {
    throw std::runtime_error("[PCLTTMStrict] non-RC export is not implemented");
}

}  // namespace pmsf
