#include "PPMC/decompress/PPMCSymbolReadOperator.h"
#include "common/BoundedRingTraversal.h"
#include "common/DebugTool.h"
#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"
#include "BFSVersionManager.h"
#include "BufferUtils.h"
#include "Configuration.h"
#include "Facet.h"
#include "Global.h"
#include "Halfedge.h"
#include "MeshUtils.h"
#include "Point.h"
#include "core.h"
#include "predict/FrenetResidualCodec.h"
#include "PPMC/PPMCFrenetRotation.h"
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <sstream>

namespace {
}  // namespace
#include <string>
#include <sys/types.h>
#include <vector>
#include <zlib.h>

namespace {

static int ppmcQsmodelRescale() {
    const char* e = std::getenv("PMSF_PPMC_QS_RESCALE");
    if (e && e[0]) {
        int v = std::atoi(e);
        if (v >= 6 && v <= 20) return 1 << v;
    }
    return 1 << 17;
}


// Double-precision inverse Frenet rotation (matches the encoder's
// ppmc::frenetRotationD exactly; see PPMC/PPMCFrenetRotation.h for why the
// MCGAL float path is not used for PPMC residuals any more).
MCGAL::PointInt ppmcInvFrenetResidual(const MCGAL::PointInt& frenetResidual,
                                      MCGAL::Halfedge* gate, MCGAL::Mesh* mesh) {
    // D28 fix: integer-lattice frame (bit-identical to encoder).
    pmsf::FrenetFrame fr = ppmc::ppmcHalfedgeFrameQuantized(gate, mesh);
    const double T1[3] = {static_cast<double>(fr.t1[0]), static_cast<double>(fr.t1[1]),
                          static_cast<double>(fr.t1[2])};
    const double T2[3] = {static_cast<double>(fr.t2[0]), static_cast<double>(fr.t2[1]),
                          static_cast<double>(fr.t2[2])};
    const double N[3] = {static_cast<double>(fr.normal[0]), static_cast<double>(fr.normal[1]),
                         static_cast<double>(fr.normal[2])};
    MCGAL::PointInt frenetCopy = frenetResidual;
    MCGAL::Vector3i fc(frenetCopy[0], frenetCopy[1], frenetCopy[2]);
    MCGAL::Vector3i inv = ppmc::invFrenetRotationD(fc, T1, T2, N);
    return MCGAL::PointInt(inv[0], inv[1], inv[2]);
}

// ─── P3.b 浮点判据 helpers（与 PPMCSymbolCollectOperator.cpp 同源） ───────────
// 在解码端做相同的浮点判据，走同一 BFS 顺序 → bit-identical。

bool ppmcMultiComponentSymbols() {
    const char* env = std::getenv("PMSF_PPMC_MULTI_COMPONENT_SYMBOLS");
    return env && *env && std::string(env) != "0";
}

bool ppmcSymbolDebugLogs() {
    static const bool enabled = [] {
        const char* env = std::getenv("PMSF_SEGMENTED_PPMC_DEBUG");
        return env && *env && std::string(env) != "0";
    }();
    return enabled;
}

struct PPMCEdgeTraversalDebugStats {
    std::size_t edgeVisits = 0;
    std::size_t coded = 0;
    std::size_t noCode = 0;
    std::size_t noCodeBothUnsplit = 0;
    std::size_t noCodeBoundaryUnsplit = 0;
    std::size_t sameGroupOpp = 0;
    std::size_t noSameGroupOpp = 0;
    std::size_t original = 0;
    std::size_t inserted = 0;
    std::size_t boundaryEnqueues = 0;
    std::size_t componentSeeds = 0;
    std::size_t brokenCycles = 0;
    std::size_t symbolOverflow = 0;
    std::vector<std::pair<int, int>> firstNoCodeEdges;
};

struct PPMCFaceTraversalDebugStats {
    std::size_t faceVisits = 0;
    std::size_t split = 0;
    std::size_t unsplit = 0;
    std::size_t residuals = 0;
    std::size_t componentSeeds = 0;
    std::size_t brokenCycles = 0;
    std::uint64_t sequenceHash = 1469598103934665603ull;
};

int ppmcTraceLimit() {
    static const int limit = [] {
        const char* env = std::getenv("PMSF_SEGMENTED_PPMC_TRACE_LIMIT");
        if (!env || !*env) return 0;
        return std::max(0, std::atoi(env));
    }();
    return limit;
}

std::uint64_t mixDebugHash(std::uint64_t h, std::uint64_t v) {
    h ^= v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
    h *= 1099511628211ull;
    return h;
}

std::uint64_t signedCoordBits(int v) {
    return static_cast<std::uint64_t>(static_cast<std::int64_t>(v)) ^
           0x8000000000000000ull;
}

unsigned faceDegreeFromGate(MCGAL::Halfedge* gate);

std::uint64_t faceRingDebugHash(MCGAL::Mesh* mesh,
                                MCGAL::Halfedge* gate,
                                bool split) {
    std::uint64_t h = split ? 0x73504c4954544142ull : 0x554e53504c495454ull;
    if (!gate) return h;
    h = mixDebugHash(h, static_cast<std::uint64_t>(faceDegreeFromGate(gate)));
    h = mixDebugHash(h, gate->face() ? static_cast<std::uint64_t>(gate->face()->groupId()) : ~0ull);
    MCGAL::Halfedge* it = gate;
    int guard = 0;
    while (it && ++guard <= 4096) {
        if (it->vertex() && mesh) {
            MCGAL::PointInt q = mesh->floatPosToInt(it->vertex()->point());
            h = mixDebugHash(h, signedCoordBits(q.x()));
            h = mixDebugHash(h, signedCoordBits(q.y()));
            h = mixDebugHash(h, signedCoordBits(q.z()));
        } else {
            h = mixDebugHash(h, 0xffffffffffffffffull);
        }
        if (!it->next()) {
            h = mixDebugHash(h, 0x4f50454e5f504154ull);
            break;
        }
        it = it->next();
        if (it == gate) break;
    }
    if (guard > 4096) h = mixDebugHash(h, 0x42524f4b454eull);
    return h;
}

void traceFaceDebug(const char* side,
                    int group,
                    int op,
                    std::size_t index,
                    MCGAL::Mesh* mesh,
                    MCGAL::Halfedge* gate,
                    bool split,
                    float surface) {
    const int limit = ppmcTraceLimit();
    if (limit <= 0 || index >= static_cast<std::size_t>(limit)) return;
    std::ostringstream verts;
    MCGAL::Halfedge* it = gate;
    int guard = 0;
    while (it && ++guard <= 32) {
        if (it->vertex() && mesh) {
            MCGAL::PointInt q = mesh->floatPosToInt(it->vertex()->point());
            verts << q.x() << "," << q.y() << "," << q.z();
        } else {
            verts << "null";
        }
        if (!it->next()) {
            verts << "|open";
            break;
        }
        it = it->next();
        if (it == gate) break;
        verts << ";";
    }
    std::cerr << "[PMSF_PPMC_FACE_TRACE] " << side
              << " group=" << group
              << " op=" << op
              << " idx=" << index
              << " poolId=" << (gate ? gate->poolId() : -1)
              << " face_gid=" << (gate && gate->face() ? gate->face()->groupId() : -1)
              << " degree=" << faceDegreeFromGate(gate)
              << " split=" << (split ? 1 : 0)
              << " surface=" << surface
              << " ring_hash=" << faceRingDebugHash(mesh, gate, split)
              << " verts=" << verts.str()
              << "\n";
}

inline bool predictSplitByAreaFloat(float f_faceSurface,
                                     float avgSplit, float avgUnsplit) {
    if ((avgSplit != 0.0f && std::fabs(f_faceSurface - avgSplit) <= std::fabs(f_faceSurface - avgUnsplit))
        || avgUnsplit == 0.0f)
        return true;
    return false;
}

inline bool predictOriginalByLenFloat(float f_edgeLen,
                                       float avgOrig, float avgIns) {
    if ((avgOrig != 0.0f && std::fabs(f_edgeLen - avgOrig) <= std::fabs(f_edgeLen - avgIns))
        || avgIns == 0.0f)
        return true;
    return false;
}

inline bool faceClassRatiosClose(unsigned cntSplit, unsigned cntUnsplit) {
    float total = (float)(cntSplit + cntUnsplit);
    if (total == 0.0f) return false;
    return std::fabs((float)cntSplit / total - (float)cntUnsplit / total) < 0.1f;
}

inline float safeEdgeLen(MCGAL::Halfedge* h) {
    if (!h) return 0.0f;
    if (h->opposite()) return MCGAL::edgeLen(h);
    if (!h->vertex() || !h->end_vertex()) return 0.0f;
    MCGAL::Vector3 d = h->vertex()->point() - h->end_vertex()->point();
    return d.length();
}

unsigned faceDegreeFromGate(MCGAL::Halfedge* gate) {
    if (!gate) return 0;
    unsigned degree = 0;
    MCGAL::Halfedge* h = gate;
    while (h) {
        ++degree;
        if (!h->next()) {
            ++degree;
            break;
        }
        h = h->next();
        if (h == gate) break;
    }
    return degree;
}

std::uint32_t readU32LELocal(const char* p) {
    return static_cast<std::uint32_t>(static_cast<unsigned char>(p[0])) |
           (static_cast<std::uint32_t>(static_cast<unsigned char>(p[1])) << 8) |
           (static_cast<std::uint32_t>(static_cast<unsigned char>(p[2])) << 16) |
           (static_cast<std::uint32_t>(static_cast<unsigned char>(p[3])) << 24);
}

std::uint32_t readVarU32Local(const std::vector<char>& data, std::size_t& off) {
    std::uint32_t x = 0;
    int shift = 0;
    while (off < data.size() && shift <= 28) {
        const auto b = static_cast<unsigned char>(data[off++]);
        x |= static_cast<std::uint32_t>(b & 0x7fu) << shift;
        if ((b & 0x80u) == 0) return x;
        shift += 7;
    }
    throw std::runtime_error("[PPMC DEC] malformed split_zlib varint");
}

std::int32_t unzigzag32Local(std::uint32_t v) {
    return static_cast<std::int32_t>((v >> 1) ^ (~(v & 1u) + 1u));
}

std::int32_t readVarI32Local(const std::vector<char>& data, std::size_t& off) {
    return unzigzag32Local(readVarU32Local(data, off));
}

bool decodeResidualPayload(const std::vector<char>& payload,
                           std::vector<MCGAL::PointInt>& residuals) {
    residuals.clear();
    if (payload.size() < 9) return false;
    const unsigned char codec = static_cast<unsigned char>(payload[0]);
    const std::uint32_t rawLen = readU32LELocal(payload.data() + 1);
    const std::uint32_t storedLen = readU32LELocal(payload.data() + 5);
    if (9ull + storedLen != payload.size()) return false;

    std::vector<char> raw(static_cast<std::size_t>(rawLen));
    if (codec == 0) {
        if (storedLen != rawLen) return false;
        std::memcpy(raw.data(), payload.data() + 9, raw.size());
    } else if (codec == 1) {
        uLongf outLen = static_cast<uLongf>(rawLen);
        const int zret = uncompress(reinterpret_cast<Bytef*>(raw.data()), &outLen,
                                    reinterpret_cast<const Bytef*>(payload.data() + 9),
                                    static_cast<uLong>(storedLen));
        if (zret != Z_OK || outLen != rawLen) return false;
    } else {
        return false;
    }

    std::size_t off = 0;
    const std::uint32_t count = readVarU32Local(raw, off);
    std::vector<std::int32_t> xs(count), ys(count), zs(count);
    for (std::uint32_t i = 0; i < count; ++i) xs[i] = readVarI32Local(raw, off);
    for (std::uint32_t i = 0; i < count; ++i) ys[i] = readVarI32Local(raw, off);
    for (std::uint32_t i = 0; i < count; ++i) zs[i] = readVarI32Local(raw, off);
    if (off != raw.size()) return false;
    residuals.reserve(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        residuals.emplace_back(xs[i], ys[i], zs[i]);
    }
    return true;
}

}  // namespace

bool PPMCSymbolReadOperator::inGroup(MCGAL::Halfedge* h) const {
    if (groupId < 0) {
        return h != nullptr;
    }
    if (!h) return false;
    if (h->face()) {
        return h->face()->groupId() == groupId;
    }
    return h->vertex() && h->vertex()->groupId() == groupId;
}

MCGAL::Halfedge* PPMCSymbolReadOperator::findNextUnconqueredFaceSeed() const {
    if (!mesh_) return nullptr;
    for (MCGAL::Facet* f : mesh_->faces()) {
        if (!f || f->isRemoved() || f->isConquered() || !f->proxyHalfedge()) continue;
        if (groupId >= 0 && f->groupId() != groupId) continue;
        return f->proxyHalfedge();
    }
    return nullptr;
}

MCGAL::Halfedge* PPMCSymbolReadOperator::findNextUnprocessedHalfedgeSeed() const {
    if (!mesh_) return nullptr;
    for (MCGAL::Facet* f : mesh_->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        if (groupId >= 0 && f->groupId() != groupId) continue;
        MCGAL::Halfedge* start = f->proxyHalfedge();
        MCGAL::Halfedge* h = start;
        int guard = 0;
        do {
            if (h && !h->isProcessed() && inGroup(h)) return h;
            h = h ? h->next() : nullptr;
        } while (h && h != start && ++guard <= 2048);
    }
    return nullptr;
}

void PPMCSymbolReadOperator::collect(MCGAL::Halfedge* seed,
                                     char* buffer,
                                     size_t& dataOffset,
                                     std::vector<MCGAL::Vertex*>& vertices,
                                     std::vector<MCGAL::Halfedge*>& halfedge,
                                     std::vector<MCGAL::Facet*>& facets) {
    while (!gateQueue.empty()) gateQueue.pop();
    if (collectSplitZlibOperation(seed, buffer, dataOffset, vertices, halfedge, facets)) {
        return;
    }
    rangeCoder.p_data = buffer;
    rangeCoder.p_dataOffset = &dataOffset;
    start_decoding(&rangeCoder);
    boundaryScanFacets_.clear();
    undecimationStep(seed, facets);
    InsertedEdgeDecodingStep(seed, halfedge);
}

bool PPMCSymbolReadOperator::collectSplitZlibOperation(
    MCGAL::Halfedge* seed,
    char* buffer,
    size_t& dataOffset,
    std::vector<MCGAL::Vertex*>& vertices,
    std::vector<MCGAL::Halfedge*>& halfedge,
    std::vector<MCGAL::Facet*>& facets) {
    (void)vertices;
    if (!seed || !buffer) return false;
    const char* p = buffer + dataOffset;
    if (p[0] != 'P' || p[1] != 'O' || p[2] != 'C' ||
        (p[3] != '1' && p[3] != '2')) {
        return false;
    }
    const bool countedPayload = p[3] == '2';
    const bool facePredictionUsed = static_cast<unsigned char>(p[4]) != 0;
    const bool edgePredictionUsed = static_cast<unsigned char>(p[5]) != 0;
    const std::uint32_t faceLen = readU32LELocal(p + 6);
    const std::uint32_t residualLen = readU32LELocal(p + 10);
    const std::uint32_t edgeLen = readU32LELocal(p + 14);
    std::size_t faceSymbolCount = std::numeric_limits<std::size_t>::max();
    std::size_t edgeSymbolCount = std::numeric_limits<std::size_t>::max();
    std::size_t off = dataOffset + 18;
    if (countedPayload) {
        faceSymbolCount = static_cast<std::size_t>(readU32LELocal(p + 18));
        edgeSymbolCount = static_cast<std::size_t>(readU32LELocal(p + 22));
        off = dataOffset + 26;
    }
    std::vector<char> faceConn(buffer + off, buffer + off + faceLen);
    off += faceLen;
    std::vector<char> residualPayload(buffer + off, buffer + off + residualLen);
    off += residualLen;
    std::vector<char> edgeConn(buffer + off, buffer + off + edgeLen);
    off += edgeLen;

    std::vector<MCGAL::PointInt> residuals;
    if (!decodeResidualPayload(residualPayload, residuals)) {
        throw std::runtime_error("[PPMC DEC] split_zlib residual payload decode failed");
    }
    if (ppmcSymbolDebugLogs()) {
        std::cerr << "[PMSF_SEGMENTED_PPMC_DEBUG] decode_header"
                  << " group=" << groupId
                  << " op=" << debugOperationId_
                  << " face_symbols=" << faceSymbolCount
                  << " residuals=" << residuals.size()
                  << " edge_symbols=" << edgeSymbolCount
                  << " face_bytes=" << faceLen
                  << " residual_bytes=" << residualLen
                  << " edge_bytes=" << edgeLen
                  << "\n";
    }

    dataOffset = off;
    boundaryScanFacets_.clear();
    undecimationStepSplit(seed, faceConn, residuals, facePredictionUsed,
                          faceSymbolCount, facets);
    insertedEdgeDecodingStepSplit(seed, edgeConn, edgePredictionUsed,
                                  edgeSymbolCount, halfedge);
    return true;
}

void PPMCSymbolReadOperator::applyDecodedFrenetResidual(
    MCGAL::Halfedge* heh_gate,
    MCGAL::Facet* fh,
    const MCGAL::PointInt& frenetResidual) {
    if (!fh) return;
    MCGAL::PointInt correction = frenetResidual;
#ifdef USE_BIJECTION
    correction = ppmcInvFrenetResidual(frenetResidual, heh_gate, mesh_.get());
#endif
    if (b_useCurvaturePrediction) {
        // 逆变换：r = invFrenet(g) - lap(face)/INV_ALPHA（INV_ALPHA=2）。
        correction = correction - avgLaplacianVect(heh_gate) / 2;
    }
    fh->setSplittable();
    fh->setProxyHalfedge(heh_gate);
    fh->setRemovedVertexPosInt(correction);
}

void PPMCSymbolReadOperator::undecimationStepSplit(
    MCGAL::Halfedge* seed,
    const std::vector<char>& faceConn,
    const std::vector<MCGAL::PointInt>& residuals,
    bool predictionUsed,
    std::size_t faceSymbolLimit,
    std::vector<MCGAL::Facet*>& facets) {
    if (mesh_) {
        for (MCGAL::Facet* f : mesh_->faces()) {
            if (f) f->resetProcessedFlag();
        }
    }
    while (!gateQueue.empty()) gateQueue.pop();
    gateQueue.push(seed->poolId());

    f_avgSurfaceFaceWithCenterRemoved = 0.0f;
    f_avgSurfaceFaceWithoutCenterRemoved = 0.0f;
    i_nbFacesWithCenterRemoved = 0;
    i_nbFacesWithoutCenterRemoved = 0;
    i_nbGoodPredictions = 0;
    b_predictionUsed = predictionUsed;

    rangecoder connRc;
    std::size_t connOff = 0;
    connRc.p_data = const_cast<char*>(faceConn.data());
    connRc.p_dataOffset = &connOff;
    start_decoding(&connRc);
    initqsmodel(&connectModel, 2, 10, 1 << 9, NULL, 0);

    std::size_t residualIndex = 0;
    std::size_t faceSymbolsDecoded = 0;
    PPMCFaceTraversalDebugStats faceDebugStats;
    const bool faceDebugEnabled = ppmcSymbolDebugLogs() || ppmcTraceLimit() > 0;
    const bool countedFaces =
        faceSymbolLimit != std::numeric_limits<std::size_t>::max();
    while (true) {
        if (faceSymbolsDecoded >= faceSymbolLimit) break;
        if (gateQueue.empty()) {
            if (!countedFaces) break;
            MCGAL::Halfedge* nextSeed = findNextUnconqueredFaceSeed();
            if (!nextSeed) break;
            gateQueue.push(nextSeed->poolId());
            ++faceDebugStats.componentSeeds;
        }
        int hIndex = gateQueue.front();
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndex(DEFAULT_MESH_ID, hIndex);
        gateQueue.pop();

        if (!h || !h->face()) continue;
        if (groupId >= 0 && !inGroup(h)) continue;
        MCGAL::Facet* f = h->face();
        if (f->isConquered()) continue;
        boundaryScanFacets_.push_back(f);

        int syfreq = 0, ltfreq = 0;
        ltfreq = decode_culshift(&connRc, 10);
        unsigned sym = qsgetsym(&connectModel, ltfreq);
        qsgetfreq(&connectModel, sym, &syfreq, &ltfreq);
        decode_update(&connRc, syfreq, ltfreq, 1 << 10);
        qsupdate(&connectModel, sym);
        ++faceSymbolsDecoded;

        MCGAL::Halfedge* hIt = h;
        int cg = 0;
        while (hIt) {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp && hOpp->face() && !hOpp->face()->isConquered() &&
                (groupId < 0 || inGroup(hOpp))) {
                gateQueue.push(hOpp->poolId());
            }
            if (!hIt->next()) break;
            hIt = hIt->next();
            if (++cg > 1024) {
                fprintf(stderr, "[PPMC DEC] BROKEN CYCLE in split_zlib neighbor push, deg=%u, poolId=%d — skip face\n",
                        faceDegreeFromGate(h), h->poolId());
                if (faceDebugEnabled) ++faceDebugStats.brokenCycles;
                break;
            }
            if (hIt == h) break;
        }
        if (cg > 1024) continue;

        float f_faceSurface = MCGAL::faceSurface(h);
        bool b_split = false;
        if (b_predictionUsed) {
            bool predictSplit = false;
            bool predictDone = false;
            const unsigned faceDegree = faceDegreeFromGate(h);
            // 与编码端一致（参考实现）：balance 预测不限制面度数。
            if (b_useTriangleMeshConnectivityPredictionFaces) {
                if (faceClassRatiosClose(i_nbFacesWithCenterRemoved, i_nbFacesWithoutCenterRemoved)) {
                    int i_neighborBalance = 0;
                    MCGAL::Halfedge* hnb = h;
                    int cg2 = 0;
                    while (hnb) {
                        MCGAL::Halfedge* hnbOpp = hnb->opposite();
                        MCGAL::Facet* fn = hnbOpp ? hnbOpp->face() : nullptr;
                        if (fn && fn->isProcessed()) {
                            if (fn->isSplittable())
                                i_neighborBalance++;
                            else if (fn->isUnsplittable())
                                i_neighborBalance--;
                        }
                        if (!hnb->next()) break;
                        hnb = hnb->next();
                        if (++cg2 > 1024) break;
                        if (hnb == h) break;
                    }
                    if (cg2 <= 1024) {
                        predictSplit = (i_neighborBalance < 0);
                        predictDone = true;
                    }
                }
            }
            if (!predictDone) {
                predictSplit = predictSplitByAreaFloat(f_faceSurface,
                                                       f_avgSurfaceFaceWithCenterRemoved,
                                                       f_avgSurfaceFaceWithoutCenterRemoved);
            }
            b_split = (sym == 1) ? predictSplit : !predictSplit;
        } else {
            b_split = sym == 1;
        }
                updateAvgSurfaces(b_split, f_faceSurface);
        if (faceDebugEnabled) {
            ++faceDebugStats.faceVisits;
            if (b_split) {
                ++faceDebugStats.split;
                ++faceDebugStats.residuals;
            } else {
                ++faceDebugStats.unsplit;
            }
            const std::uint64_t ringHash = faceRingDebugHash(mesh_.get(), h, b_split);
            faceDebugStats.sequenceHash =
                mixDebugHash(faceDebugStats.sequenceHash, ringHash);
            traceFaceDebug("decode",
                           groupId,
                           debugOperationId_,
                           faceDebugStats.faceVisits - 1,
                           mesh_.get(),
                           h,
                           b_split,
                           f_faceSurface);
        }
        if (b_split) {
            if (residualIndex >= residuals.size()) {
                throw std::runtime_error(
                    "[PPMC DEC] split_zlib residual underflow residual_index=" +
                    std::to_string(residualIndex) + " residual_count=" +
                    std::to_string(residuals.size()) + " face_symbols_decoded=" +
                    std::to_string(faceSymbolsDecoded) + " face_symbol_limit=" +
                    std::to_string(faceSymbolLimit));
            }
            applyDecodedFrenetResidual(h, f, residuals[residualIndex++]);
            facets.push_back(f);
        } else {
            f->setUnsplittable();
        }
        f->setProcessedFlag();
    }
    done_decoding(&connRc);
    deleteqsmodel(&connectModel);
    if (faceSymbolLimit != std::numeric_limits<std::size_t>::max() &&
        faceSymbolsDecoded != faceSymbolLimit) {
        std::cerr << "[PPMC DEC] split_zlib face symbol count mismatch decoded="
                  << faceSymbolsDecoded << " expected=" << faceSymbolLimit
                  << " residuals_used=" << residualIndex
                  << " residuals_total=" << residuals.size() << std::endl;
    }
    if (faceDebugEnabled ||
        (faceSymbolLimit != std::numeric_limits<std::size_t>::max() &&
         faceSymbolsDecoded != faceSymbolLimit)) {
        std::cerr << "[PMSF_PPMC_FACE_DEBUG] decode"
                  << " group=" << groupId
                  << " op=" << debugOperationId_
                  << " face_visits=" << faceDebugStats.faceVisits
                  << " expected=" << faceSymbolLimit
                  << " split=" << faceDebugStats.split
                  << " unsplit=" << faceDebugStats.unsplit
                  << " residuals_used=" << residualIndex
                  << " residuals_total=" << residuals.size()
                  << " component_seeds=" << faceDebugStats.componentSeeds
                  << " broken_cycles=" << faceDebugStats.brokenCycles
                  << " sequence_hash=" << faceDebugStats.sequenceHash
                  << "\n";
    }
    // Some open-boundary guards can skip broken-cycle faces after the encoder has
    // emitted their residuals. The legacy interleaved range stream tolerates this
    // by leaving trailing geometry symbols unread, so the experimental split
    // stream must do the same to remain a transparent storage variant.
}

void PPMCSymbolReadOperator::insertedEdgeDecodingStepSplit(
    MCGAL::Halfedge* seed,
    const std::vector<char>& edgeConn,
    bool predictionUsed,
    std::size_t edgeSymbolLimit,
    std::vector<MCGAL::Halfedge*>& halfedges) {
    while (!gateQueue.empty()) gateQueue.pop();
    gateQueue.push(seed->poolId());
    f_avgInsertedEdgesLength = 0.0f;
    f_avgOriginalEdgesLength = 0.0f;
    i_nbInsertedEdges = 0;
    i_nbOriginalEdges = 0;
    b_predictionUsed = predictionUsed;

    rangecoder connRc;
    std::size_t connOff = 0;
    connRc.p_data = const_cast<char*>(edgeConn.data());
    connRc.p_dataOffset = &connOff;
    start_decoding(&connRc);
    initqsmodel(&connectModel, 2, 10, 1 << 9, NULL, 0);

    PPMCEdgeTraversalDebugStats debugStats;
    auto enqueuePendingBoundaryEdges = [&]() -> int {
        int added = 0;
        for (MCGAL::Facet* f : boundaryScanFacets_) {
            if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
            MCGAL::Halfedge* start = f->proxyHalfedge();
            MCGAL::Halfedge* e = start;
            int guard = 0;
            do {
                if (e && e->face() == f && !e->isProcessed() && !e->opposite() && inGroup(e)) {
                    gateQueue.push(e->poolId());
                    ++added;
                    ++debugStats.boundaryEnqueues;
                }
                e = e ? e->next() : nullptr;
            } while (e && e != start && ++guard <= 2048);
        }
        return added;
    };
    auto findNextBoundaryScanHalfedgeSeed = [&]() -> MCGAL::Halfedge* {
        for (MCGAL::Facet* f : boundaryScanFacets_) {
            if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
            MCGAL::Halfedge* start = f->proxyHalfedge();
            MCGAL::Halfedge* e = start;
            int guard = 0;
            while (e && ++guard <= 4096) {
                if (e->face() == f && !e->isProcessed() &&
                    (groupId < 0 || inGroup(e))) {
                    return e;
                }
                if (!e->next()) break;
                e = e->next();
                if (e == start) break;
            }
        }
        return nullptr;
    };

    std::size_t edgeSymbolsDecoded = 0;
    const bool countedEdges =
        edgeSymbolLimit != std::numeric_limits<std::size_t>::max();
    while (true) {
        if (gateQueue.empty() && enqueuePendingBoundaryEdges() == 0) {
            if (!countedEdges) break;
            MCGAL::Halfedge* nextSeed = findNextBoundaryScanHalfedgeSeed();
            if (!nextSeed && edgeSymbolsDecoded < edgeSymbolLimit) {
                nextSeed = findNextUnprocessedHalfedgeSeed();
            }
            if (!nextSeed) break;
            gateQueue.push(nextSeed->poolId());
            ++debugStats.componentSeeds;
        }
        MCGAL::Halfedge* h =
            MCGAL::contextPool.getHalfedgeByIndex(DEFAULT_MESH_ID, gateQueue.front());
        gateQueue.pop();
        if (!h || !h->face()) continue;
        if (groupId >= 0 && !inGroup(h)) continue;
        if (h->isProcessed()) continue;

        ++debugStats.edgeVisits;
        h->setProcessed();
        if (h->opposite() && inGroup(h->opposite())) h->opposite()->setProcessed();

        MCGAL::Halfedge* hIt = h->next();
        int cg = 0;
        while (hIt && hIt->opposite() && hIt->opposite() != h &&
               (groupId < 0 || inGroup(hIt->opposite()))) {
            if (!hIt->isProcessed() && (groupId < 0 || inGroup(hIt))) {
                gateQueue.push(hIt->poolId());
            }
            hIt = hIt->opposite()->next();
            if (++cg > 2048) break;
        }
        if (cg > 2048) {
            ++debugStats.brokenCycles;
            continue;
        }
        bool b_original = true;
        float f_edgeLen = safeEdgeLen(h);
        const bool sameGroupOpp =
            h->opposite() && h->opposite()->face() &&
            (groupId < 0 || inGroup(h->opposite()));
        if (sameGroupOpp) {
            ++debugStats.sameGroupOpp;
        } else {
            ++debugStats.noSameGroupOpp;
        }
        const bool b_toCode = sameGroupOpp
                                  ? (h->face()->isSplittable() ||
                                     h->opposite()->face()->isSplittable())
                                  : h->face()->isSplittable();
        if (b_toCode) {
            if (countedEdges && edgeSymbolsDecoded >= edgeSymbolLimit) {
                ++debugStats.symbolOverflow;
                std::cerr << "[PPMC DEC] split_zlib edge symbol overflow"
                          << " decoded=" << edgeSymbolsDecoded
                          << " expected=" << edgeSymbolLimit
                          << " group=" << groupId
                          << " op=" << debugOperationId_
                          << " poolId=" << h->poolId()
                          << std::endl;
                break;
            }
            int syfreq = 0, ltfreq = 0;
            ltfreq = decode_culshift(&connRc, 10);
            unsigned sym = qsgetsym(&connectModel, ltfreq);
            qsgetfreq(&connectModel, sym, &syfreq, &ltfreq);
            decode_update(&connRc, syfreq, ltfreq, 1 << 10);
            qsupdate(&connectModel, sym);
            ++edgeSymbolsDecoded;
            ++debugStats.coded;

            if (b_predictionUsed) {
                bool predictOrig = predictOriginalByLenFloat(f_edgeLen,
                                                              f_avgOriginalEdgesLength,
                                                              f_avgInsertedEdgesLength);
                b_original = (sym == 1) ? predictOrig : !predictOrig;
            } else {
                b_original = sym == 0;
            }
            if (!b_original) {
                h->setAdded();
                halfedges.push_back(h);
            }
        } else {
            ++debugStats.noCode;
            const int reason = sameGroupOpp ? 1 : 2;
            if (sameGroupOpp) {
                ++debugStats.noCodeBothUnsplit;
            } else {
                ++debugStats.noCodeBoundaryUnsplit;
            }
            if (debugStats.firstNoCodeEdges.size() < 8) {
                debugStats.firstNoCodeEdges.emplace_back(h->poolId(), reason);
            }
        }
        if (b_original) {
            ++debugStats.original;
        } else {
            ++debugStats.inserted;
        }
        updateAvgEdgeLen(b_original, f_edgeLen);
    }
    done_decoding(&connRc);
    deleteqsmodel(&connectModel);
    if (edgeSymbolLimit != std::numeric_limits<std::size_t>::max() &&
        edgeSymbolsDecoded != edgeSymbolLimit) {
        std::cerr << "[PPMC DEC] split_zlib edge symbol count mismatch decoded="
                  << edgeSymbolsDecoded << " expected=" << edgeSymbolLimit
                  << std::endl;
    }
    if (ppmcSymbolDebugLogs() ||
        (edgeSymbolLimit != std::numeric_limits<std::size_t>::max() &&
         edgeSymbolsDecoded != edgeSymbolLimit)) {
        std::cerr << "[PMSF_PPMC_EDGE_DEBUG] decode"
                  << " group=" << groupId
                  << " op=" << debugOperationId_
                  << " edge_visits=" << debugStats.edgeVisits
                  << " edge_symbols_decoded=" << edgeSymbolsDecoded
                  << " expected=" << edgeSymbolLimit
                  << " conn_bytes=" << edgeConn.size()
                  << " conn_off=" << connOff
                  << " no_code=" << debugStats.noCode
                  << " no_code_both_unsplit=" << debugStats.noCodeBothUnsplit
                  << " no_code_boundary_unsplit=" << debugStats.noCodeBoundaryUnsplit
                  << " same_group_opp=" << debugStats.sameGroupOpp
                  << " no_same_group_opp=" << debugStats.noSameGroupOpp
                  << " original=" << debugStats.original
                  << " inserted=" << debugStats.inserted
                  << " boundary_enqueues=" << debugStats.boundaryEnqueues
                  << " component_seeds=" << debugStats.componentSeeds
                  << " broken_cycles=" << debugStats.brokenCycles
                  << " symbol_overflow=" << debugStats.symbolOverflow
                  << " first_no_code=";
        for (const auto& item : debugStats.firstNoCodeEdges) {
            std::cerr << item.first << ":" << item.second << ",";
        }
        std::cerr << "\n";
    }
}

/**
 * Update the average surfaces of the faces with an removed vertex and the others.
 */
// ⚠️ P3.b 对齐：与 collector 端同步回退到浮点增量均值。
void PPMCSymbolReadOperator::updateAvgSurfaces(bool b_split, float f_faceSurface) {
    if (b_split) {
        i_nbFacesWithCenterRemoved++;
        f_avgSurfaceFaceWithCenterRemoved = (f_avgSurfaceFaceWithCenterRemoved
                                              * (i_nbFacesWithCenterRemoved - 1) + f_faceSurface)
                                             / i_nbFacesWithCenterRemoved;
    } else {
        i_nbFacesWithoutCenterRemoved++;
        f_avgSurfaceFaceWithoutCenterRemoved = (f_avgSurfaceFaceWithoutCenterRemoved
                                                 * (i_nbFacesWithoutCenterRemoved - 1) + f_faceSurface)
                                                / i_nbFacesWithoutCenterRemoved;
    }
}

/**
 * Return the degree of a vertex without counting the new edges.
 */
inline unsigned vertexDegreeNotNew(MCGAL::Vertex* vh) {
    unsigned i_degree = 0;

    for (MCGAL::Halfedge* hit : pmsf::boundedRing(vh)) {
        if (!hit->isNew())
            i_degree++;
    }

    return i_degree;
}

/**
 * Return the average laplacian vector of the vertices of a face.
 */
MCGAL::PointInt PPMCSymbolReadOperator::avgLaplacianVect(MCGAL::Halfedge* heh_gate) {
    // 与编码端完全同构的确定性 1-ring 聚合缓存（详见 PPMCSymbolCollectOperator
    // 版本注释）。perf（2026-11-08）：按 poolId 平铺成 int 数组，避免每轮数千次
    // unordered_map/vector 分配；结果与原实现逐位相同。
    if (!curvatureNeighborCacheValid_) {
        const int n = static_cast<int>(mesh_->size_of_vertices());
        const int nClamped = (n < 0) ? 0 : n;
        curvSumQX_.assign(static_cast<std::size_t>(nClamped), 0);
        curvSumQY_.assign(static_cast<std::size_t>(nClamped), 0);
        curvSumQZ_.assign(static_cast<std::size_t>(nClamped), 0);
        curvCnt_.assign(static_cast<std::size_t>(nClamped), 0);
        std::vector<int> posQX(static_cast<std::size_t>(nClamped), 0);
        std::vector<int> posQY(static_cast<std::size_t>(nClamped), 0);
        std::vector<int> posQZ(static_cast<std::size_t>(nClamped), 0);
        for (MCGAL::Vertex* v : mesh_->vertices()) {
            if (!v || v->isRemoved()) continue;
            const int id = v->poolId();
            if (id < 0 || id >= nClamped) continue;
            // D28 fix: use the identical quantizer as the encoder
            // (mesh->floatPosToInt) so the curvature Laplacian cache matches bit-wise.
            const MCGAL::PointInt q = mesh_->floatPosToInt(v->point());
            posQX[static_cast<std::size_t>(id)] = q[0];
            posQY[static_cast<std::size_t>(id)] = q[1];
            posQZ[static_cast<std::size_t>(id)] = q[2];
        }
        for (MCGAL::Facet* f : mesh_->faces()) {
            if (!f || f->isRemoved()) continue;
            MCGAL::Halfedge* h0 = f->proxyHalfedge();
            if (!h0) continue;
            MCGAL::Halfedge* h = h0;
            int guard = 0;
            do {
                if (h && h->vertex() && h->end_vertex() && !h->isNew()) {
                    const int src = h->vertex()->poolId();
                    const int dst = h->end_vertex()->poolId();
                    if (src >= 0 && src < nClamped && dst >= 0 && dst < nClamped) {
                        const std::size_t si = static_cast<std::size_t>(src);
                        const std::size_t di = static_cast<std::size_t>(dst);
                        curvSumQX_[si] += posQX[di];
                        curvSumQY_[si] += posQY[di];
                        curvSumQZ_[si] += posQZ[di];
                        curvCnt_[si] += 1;
                    }
                }
                h = h ? h->next() : nullptr;
            } while (h && h != h0 && ++guard < 4096);
        }
        curvPosQX_ = std::move(posQX);
        curvPosQY_ = std::move(posQY);
        curvPosQZ_ = std::move(posQZ);
        curvatureNeighborCacheValid_ = true;
    }

    const int n = static_cast<int>(curvCnt_.size());
    MCGAL::Halfedge* heh = heh_gate;
    MCGAL::PointInt sumLaplacian;
    do {
        MCGAL::Vertex* center = heh->end_vertex();
        MCGAL::PointInt laplacian;
        int deg = 0;
        const int id = center ? center->poolId() : -1;
        if (id >= 0 && id < n) {
            const std::size_t si = static_cast<std::size_t>(id);
            deg = curvCnt_[si];
            if (deg > 0) {
                const int cx = curvPosQX_[si], cy = curvPosQY_[si], cz = curvPosQZ_[si];
                laplacian = MCGAL::PointInt(curvSumQX_[si] - deg * cx,
                                            curvSumQY_[si] - deg * cy,
                                            curvSumQZ_[si] - deg * cz);
            }
        }
        const int degSafe = (deg == 0) ? 1 : deg;
        sumLaplacian = sumLaplacian + MCGAL::PointInt(laplacian[0] / degSafe,
                                                     laplacian[1] / degSafe,
                                                     laplacian[2] / degSafe);
        heh = heh->next();
    } while (heh != heh_gate);

    const int faceDeg = static_cast<int>(heh_gate->face()->facet_degree());
    return MCGAL::PointInt(sumLaplacian[0] / faceDeg, sumLaplacian[1] / faceDeg,
                           sumLaplacian[2] / faceDeg);
}

/**
 * Decode the geometry symbols.
 *
 * P0 对齐：
 *   1. USE_BIJECTION 已在 Configuration.h 全局开启，必须用 MCGAL 风格的
 *      invFrenetRotation 把 (α,β,γ) 反旋回原残差 (rx,ry,rz)。
 *   2. 编码端写的是 residual = Q(removedPos) - Q(barycenter(h))，故这里需要加上
 *      Q(barycenter(h)) 才能恢复绝对量化坐标，让后续
 *      `PPMCReconstructOperator::insert_point` 直接用
 *      `mesh_->intPosToFloat(...)` 得到正确浮点位置。
 *   3. 原 CGAL 残留符号（`Vector`, `VectorInt`, `CGAL::NULL_VECTOR`,
 *      `computeNormal(heh)`, `determineFrenetFrame(heh,...)`)在 PMSF 框架下并不存在，
 *      已替换为 MCGAL 等价物。
 */
void PPMCSymbolReadOperator::decodeGeometrySym(MCGAL::Halfedge* heh_gate, MCGAL::Facet* fh) {
#ifdef USE_BIJECTION
    // D28 fix: integer-lattice frame (bit-identical to encoder).
    pmsf::FrenetFrame frame = ppmc::ppmcHalfedgeFrameQuantized(heh_gate, mesh_.get());
#endif

    int coord[3];
    for (unsigned i = 0; i < 3; ++i) {
        int syfreq, ltfreq;
#ifdef USE_BIJECTION
        if (i < 2) {
#endif
            // Decode the alpha and beta symbols.
            ltfreq = decode_culshift(&rangeCoder, 18);
            unsigned sym = qsgetsym(&alphaBetaModel, ltfreq);
            qsgetfreq(&alphaBetaModel, sym, &syfreq, &ltfreq);
            decode_update(&rangeCoder, syfreq, ltfreq, 1 << 18);
            // Update the alpha and beta model.
            qsupdate(&alphaBetaModel, sym);
            // Store the value.
            coord[i] = alphaBetaMin + sym;
#ifdef USE_BIJECTION
        } else {
            // Encode the gamma symbol.
            ltfreq = decode_culshift(&rangeCoder, 18);
            unsigned sym = qsgetsym(&gammaModel, ltfreq);
            qsgetfreq(&gammaModel, sym, &syfreq, &ltfreq);
            decode_update(&rangeCoder, syfreq, ltfreq, 1 << 18);
            // Update the gamma model.
            qsupdate(&gammaModel, sym);
            // Store the value.
            coord[i] = gammaMin + sym;
        }
#endif
    }

#ifdef USE_BIJECTION
    MCGAL::Vector3i frenetCoord(coord[0], coord[1], coord[2]);
    // Compute the barycenter with the same way as decoding to avoid computation errors.
    MCGAL::PointInt correction = ppmcInvFrenetResidual(
        MCGAL::PointInt(coord[0], coord[1], coord[2]), heh_gate, mesh_.get());
#else
    MCGAL::PointInt correction(coord[0], coord[1], coord[2]);
#endif

    if (b_useCurvaturePrediction) {
        // 逆变换：r = invFrenet(g) - lap(face)/2（与编码端 g = r + lap/2 对称）。
        correction = correction - avgLaplacianVect(heh_gate) / 2;
    }

    
    fh->setSplittable();
    // OpenMesh PPMC inserts the center vertex using the BFS gate halfedge.
    // Preserve that gate so PPMCReconstructOperator does not fan from a
    // possibly different face proxy.
    fh->setProxyHalfedge(heh_gate);
    // Keep OpenMesh semantics: the face stores residual, not absolute position.
    // Unlift consumes this residual; insertion later adds Q(barycenter(gate)).
    fh->setRemovedVertexPosInt(correction);
}

void PPMCSymbolReadOperator::undecimationStep(MCGAL::Halfedge* seed, std::vector<MCGAL::Facet*>& facets) {
    resetCurvatureNeighborCache();
    while (!gateQueue.empty()) gateQueue.pop();
        gateQueue.push(seed->poolId());

    // ⚠️ P3.b 对齐：浮点增量均值。
    f_avgSurfaceFaceWithCenterRemoved = 0.0f;
    f_avgSurfaceFaceWithoutCenterRemoved = 0.0f;
    i_nbFacesWithCenterRemoved = 0;
    i_nbFacesWithoutCenterRemoved = 0;
    i_nbGoodPredictions = 0;
#ifdef DEBUG
    std::vector<int> correctionVec;
#endif

    // Read if the prediction was used or not.
    unsigned test = decode_culshift(&rangeCoder, 1);
    b_predictionUsed = test;
    decode_update(&rangeCoder, 1, b_predictionUsed, 1 << 1);

    // Read the min values and the ranges.
    uint16_t i16_min;
    i16_min = decode_short(&rangeCoder);
    alphaBetaMin = *(int16_t*)&i16_min;
    unsigned alphaBetaRange = decode_short(&rangeCoder);

#ifdef USE_BIJECTION
    i16_min = decode_short(&rangeCoder);
    gammaMin = *(int16_t*)&i16_min;
    unsigned gammaRange = decode_short(&rangeCoder);
#endif

    // Init the range coder models.
    // Guard: alphaBetaRange==0 causes division by zero in resetqsmodel (cf[n]/n).
    const int qsRescaleQ = ppmcQsmodelRescale();
    initqsmodel(&alphaBetaModel, alphaBetaRange > 0 ? alphaBetaRange : 1, 18, qsRescaleQ, NULL, 0);
#ifdef USE_BIJECTION
    initqsmodel(&gammaModel, gammaRange > 0 ? gammaRange : 1, 18, qsRescaleQ, NULL, 0);
#endif
    initqsmodel(&connectModel, 2, 10, 1 << 9, NULL, 0);
    std::vector<char> facetSymbol;
    while (true) {
        if (gateQueue.empty()) {
            if (!ppmcMultiComponentSymbols()) break;
            MCGAL::Halfedge* nextSeed = findNextUnconqueredFaceSeed();
            if (!nextSeed) break;
            gateQueue.push(nextSeed->poolId());
        }
        int hIndex = gateQueue.front();
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndex(DEFAULT_MESH_ID, hIndex);
        gateQueue.pop();

        if (!h || !h->face()) continue;
        if (groupId >= 0 && !inGroup(h)) continue;
        MCGAL::Facet* f = h->face();

        // If the face is already processed, pick the next halfedge:
        if (f->isConquered())
            continue;
        boundaryScanFacets_.push_back(f);

        // Decode the face symbol.
        int syfreq, ltfreq;
        ltfreq = decode_culshift(&rangeCoder, 10);
        unsigned sym = qsgetsym(&connectModel, ltfreq);
        qsgetfreq(&connectModel, sym, &syfreq, &ltfreq);
        decode_update(&rangeCoder, syfreq, ltfreq, 1 << 10);
        // Update the model.
        qsupdate(&connectModel, sym);

        // Add the other halfedges to the queue.
        MCGAL::Halfedge* hIt = h;
        int cg = 0;
        while (hIt) {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp && hOpp->face() && !hOpp->face()->isConquered() &&
                (groupId < 0 || inGroup(hOpp)))
                gateQueue.push(hOpp->poolId());
            if (!hIt->next()) break;
            hIt = hIt->next();
            if (++cg > 1024) {
                fprintf(stderr, "[PPMC DEC] BROKEN CYCLE in undecimationStep neighbor push, deg=%u, poolId=%d — skip face\n",
                        faceDegreeFromGate(h), h->poolId());
                break;
            }
            if (hIt == h) break;
        }

        if (cg > 1024) continue;  // BROKEN CYCLE: skip this face

        bool b_split;
        // ⚠️ P3.b 对齐：与编码端同样使用浮点 face surface。
        float f_faceSurface = MCGAL::faceSurface(h);

        // ─── 连接性预测解码（与编码端 RemovedVertexCodingStep 对称） ────────────
        if (b_predictionUsed) {
            bool predictSplit = false;
            bool predictDone = false;

            const unsigned faceDegree = faceDegreeFromGate(h);
            // 与编码端一致（参考实现）：balance 预测不限制面度数。
            if (b_useTriangleMeshConnectivityPredictionFaces) {
                if (faceClassRatiosClose(i_nbFacesWithCenterRemoved, i_nbFacesWithoutCenterRemoved)) {
                    int i_neighborBalance = 0;
                    MCGAL::Halfedge* hnb = h;
                    int cg2 = 0;
                    while (hnb) {
                        MCGAL::Halfedge* hnbOpp = hnb->opposite();
                        MCGAL::Facet* fn = hnbOpp ? hnbOpp->face() : nullptr;
                        if (fn && fn->isProcessed()) {
                            if (fn->isSplittable())
                                i_neighborBalance++;
                            else if (fn->isUnsplittable())
                                i_neighborBalance--;
                        }
                        if (!hnb->next()) break;
                        hnb = hnb->next();
                        if (++cg2 > 1024) {
                            fprintf(stderr, "[PPMC DEC] BROKEN CYCLE in triangle balance, deg=%u, poolId=%d\n",
                                    faceDegree, h->poolId());
                            break;
                        }
                        if (hnb == h) break;
                    }
                    if (cg2 <= 1024) {
                        // 与编码端对称：balance>=0 → 预测 not-split；balance<0 → 预测 split。
                        predictSplit = (i_neighborBalance < 0);
                        predictDone = true;
                    }
                }
            }

            if (!predictDone) {
                // 面积均值预测（浮点 fabs 比较，与原版 mymeshDecomp.cpp L201 完全一致）
                predictSplit = predictSplitByAreaFloat(f_faceSurface,
                                                       f_avgSurfaceFaceWithCenterRemoved,
                                                       f_avgSurfaceFaceWithoutCenterRemoved);
            }

            // sym=1 表示预测正确，sym=0 表示预测错误 → 取反预测值
            b_split = (sym == 1) ? predictSplit : !predictSplit;
        } else {
            // No connectivity prediction.
            b_split = sym == 1 ? true : false;
        }
        facetSymbol.push_back(b_split);
                // Update the average surfaces.
        updateAvgSurfaces(b_split, f_faceSurface);

        // Decode the geometry symbol.
        if (b_split) {
            decodeGeometrySym(h, f);
            MCGAL::PointInt posInt = f->getRemovedVertexPosInt();
#ifdef DEBUG
            correctionVec.push_back(posInt[0]);
            correctionVec.push_back(posInt[1]);
            correctionVec.push_back(posInt[2]);
#endif
            facets.push_back(f);
        } else {
            f->setUnsplittable();
        }
        f->setProcessedFlag();
    }
#ifdef DEBUG
    DEBUG_DUMP_CONTAINER(correctionVec, "./decode/correctionVec.txt");
    DEBUG_DUMP_FACETS(facets, "./decode/facets.txt");
#endif

    // Stop the decoder.
    done_decoding(&rangeCoder);

    // Delete the models.
    deleteqsmodel(&connectModel);
    deleteqsmodel(&alphaBetaModel);
#ifdef USE_BIJECTION
    deleteqsmodel(&gammaModel);
#endif
}

/**
 * Update the average lengths of the original and inserted edges.
 */
// ⚠️ P3.b 对齐：浮点增量均值。
void PPMCSymbolReadOperator::updateAvgEdgeLen(bool b_original, float f_edgeLen) {
    if (b_original) {
        i_nbOriginalEdges++;
        f_avgOriginalEdgesLength = (f_avgOriginalEdgesLength
                                     * (i_nbOriginalEdges - 1) + f_edgeLen)
                                    / i_nbOriginalEdges;
    } else {
        i_nbInsertedEdges++;
        f_avgInsertedEdgesLength = (f_avgInsertedEdgesLength
                                     * (i_nbInsertedEdges - 1) + f_edgeLen)
                                    / i_nbInsertedEdges;
    }
}

void PPMCSymbolReadOperator::InsertedEdgeDecodingStep(MCGAL::Halfedge* seed, std::vector<MCGAL::Halfedge*>& halfedges) {
    while (!gateQueue.empty()) gateQueue.pop();
    // Add the first halfedge to the queue.
    gateQueue.push(seed->poolId());

    // ⚠️ P3.b 对齐：浮点累加器。
    f_avgInsertedEdgesLength = 0.0f;
    f_avgOriginalEdgesLength = 0.0f;
    i_nbInsertedEdges = 0;
    i_nbOriginalEdges = 0;

    // Init the range coder models.
    initqsmodel(&connectModel, 2, 10, 1 << 9, NULL, 0);

    // Start the decoder.
    start_decoding(&rangeCoder);

    // Read if the prediction was used or not.
    b_predictionUsed = decode_culshift(&rangeCoder, 1);
    decode_update(&rangeCoder, 1, b_predictionUsed, 1 << 1);
    std::vector<char> edgeSymbol;
    auto enqueuePendingBoundaryEdges = [&]() -> int {
        int added = 0;
        for (MCGAL::Facet* f : boundaryScanFacets_) {
            if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
            MCGAL::Halfedge* start = f->proxyHalfedge();
            MCGAL::Halfedge* e = start;
            int guard = 0;
            do {
                if (e && e->face() == f && !e->isProcessed() && !e->opposite() && inGroup(e)) {
                    gateQueue.push(e->poolId());
                    ++added;
                }
                e = e ? e->next() : nullptr;
            } while (e && e != start && ++guard <= 2048);
        }
        return added;
    };

    while (true) {
        if (gateQueue.empty() && enqueuePendingBoundaryEdges() == 0) {
            if (!ppmcMultiComponentSymbols()) break;
            MCGAL::Halfedge* nextSeed = findNextUnprocessedHalfedgeSeed();
            if (!nextSeed) break;
            gateQueue.push(nextSeed->poolId());
        }
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndex(DEFAULT_MESH_ID, gateQueue.front());
        gateQueue.pop();
        if (!h || !h->face()) {
            continue;
        }
        if (groupId >= 0 && !inGroup(h)) {
            continue;
        }

        // Test if the edge has already been conquered.
        if (h->isProcessed())
            continue;

        // Mark the halfedge as processed.
        h->setProcessed();
        if (h->opposite() && inGroup(h->opposite())) {
            h->opposite()->setProcessed();
        }

        // Add the other halfedges to the queue.
        MCGAL::Halfedge* hIt = h->next();
        int cg = 0;
        while (hIt && hIt->opposite() && hIt->opposite() != h &&
               (groupId < 0 || inGroup(hIt->opposite()))) {
            if (!hIt->isProcessed() && (groupId < 0 || inGroup(hIt))) {
                gateQueue.push(hIt->poolId());
            }
            hIt = hIt->opposite()->next();
            if (++cg > 2048) {
                fprintf(stderr, "[PPMC DEC] BROKEN CYCLE in InsertedEdgeDecodingStep neighbor walk, poolId=%d — skip\n",
                        h->poolId());
                break;
            }
        }
        if (cg > 2048) continue;  // BROKEN CYCLE: skip this edge

        bool b_original = true;
        // ⚠️ P3.b 对齐：浮点 edge length，与编码端一致。
        float f_edgeLen = safeEdgeLen(h);

        // Test if there is a symbol for this edge.
        // There is no symbol if the two faces of an egde are unsplitable.
        const bool sameGroupOpp =
            h->opposite() && h->opposite()->face() &&
            (groupId < 0 || inGroup(h->opposite()));
        const bool b_toCode = sameGroupOpp
                                  ? (h->face()->isSplittable() ||
                                     h->opposite()->face()->isSplittable())
                                  : h->face()->isSplittable();
        if (b_toCode) {
            // Decode the edge symbol.
            int syfreq, ltfreq;
            ltfreq = decode_culshift(&rangeCoder, 10);
            unsigned sym = qsgetsym(&connectModel, ltfreq);
            qsgetfreq(&connectModel, sym, &syfreq, &ltfreq);
            decode_update(&rangeCoder, syfreq, ltfreq, 1 << 10);
            // Update the model.
            qsupdate(&connectModel, sym);

            // Determine if the edge is original or not.
            if (b_predictionUsed) {
                bool predictOrig = predictOriginalByLenFloat(f_edgeLen,
                                                              f_avgOriginalEdgesLength,
                                                              f_avgInsertedEdgesLength);
                // 与编码端 sym=symPred 对称：sym=1 → 预测正确 → b_original=predictOrig；
                //                            sym=0 → 预测错误 → b_original=!predictOrig。
                b_original = (sym == 1) ? predictOrig : !predictOrig;
            } else
                b_original = sym == 0 ? true : false;

            edgeSymbol.push_back(sym);
            // Mark the edge to be removed.
            if (!b_original) {
                h->setAdded();
                halfedges.push_back(h);
            }
        }

        // Update the average edge lengths.
        updateAvgEdgeLen(b_original, f_edgeLen);
    }

// printf("Inserted edge decoding completed.\n");
#ifdef DEBUG
    DEBUG_DUMP_HALFEDGES(halfedges, "./decode/halfedges.txt");
#endif

    // Stop the decoder.
    done_decoding(&rangeCoder);

    // Delete the models.
    deleteqsmodel(&connectModel);

    // PPMC lifting is owned by the reader. Callers must not unlift again
    // before reconstruction, otherwise vertex positions are shifted twice.
    if (b_useLiftingScheme)
        lift(true, mesh_);  // Unlift the vertex positions.
    // i_curDecimationId++;  // Increment the current decimation operation id.
    // i_curOperationId++;
}

void PPMCSymbolReadOperator::collect(std::vector<MCGAL::Halfedge*> seeds,
                                     char* buffer,
                                     size_t& dataOffset,
                                     std::vector<MCGAL::Vertex*>& vertices,
                                     std::vector<MCGAL::Halfedge*>& halfedge,
                                     std::vector<MCGAL::Facet*>& facets) {}

// ============================================================================
// Parquet V2 结构化模式：不使用 range coder
// ============================================================================

void PPMCSymbolReadOperator::importFromParquet(const std::vector<bool>& facetSymbols,
                                                const std::vector<bool>& edgeSymbols,
                                                const std::vector<bool>& vertexSymbols,
                                                const std::vector<MCGAL::Point>& points) {
    (void)vertexSymbols;
            useParquetMode_ = true;

    // 将 facet symbols 导入内部队列
    parquetFacetSymbols_.clear();
    for (bool s : facetSymbols) {
        parquetFacetSymbols_.push_back(s);
    }

    // 将 edge symbols 导入内部队列
    parquetEdgeSymbols_.clear();
    for (bool s : edgeSymbols) {
        parquetEdgeSymbols_.push_back(s);
    }

    // ⚠️ P0 对齐：PPMC 编码端写入 geomSym[r] 的是 **Frenet 旋转后的残差**
    //   （residual = Q(removedPos) - Q(barycenter(h))，再经 Frenet 投影到 (α,β,γ)），
    //   分量可正可负，**不是绝对量化坐标**。
    //   - 量化模式：编码端直接 int16 存储残差三个分量；解码端 round 后取整即可，
    //     绝对**不能**走 (p.x - bboxMin)/step - 0.5 那种"绝对量化坐标反量化"公式。
    //   - 非量化模式：编码端把残差当 float 存；解码端同样直接取整即可。
    parquetGeomSymbols_.clear();
    for (auto& p : points) {
        parquetGeomSymbols_.push_back(MCGAL::PointInt(
            (int)std::lround(p.x()), (int)std::lround(p.y()), (int)std::lround(p.z())));
    }
}

void PPMCSymbolReadOperator::collectFromDecoded(MCGAL::Halfedge* seed,
                                                 std::vector<MCGAL::Vertex*>& vertices,
                                                 std::vector<MCGAL::Halfedge*>& halfedges,
                                                 std::vector<MCGAL::Facet*>& facets) {
    if (!seed) return;
        // M2c native paper path: import the flat paper symbol table here instead of
    // relying on the legacy symbols.parquet import.
    const bool nativePaper = paperTypedTableMode_ || []() {
        const char* env = std::getenv("PMSF_PAPER_NATIVE");
        return env && *env && std::strcmp(env, "0") != 0;
    }();
    // M2c native: the legacy path calls importFromParquet() once per round with
    // one row's symbols.  Do the same from the paper table; flattening all rows
    // desynchronizes the decoder's per-round BFS consumption.
    if (nativePaper && pmsfSymbolTable() != nullptr &&
        nativeRoundIndex_ < pmsfSymbolTable()->size()) {
        // M4-4 typed tables store rows in decoder (coarse -> fine) order;
        // legacy paper-table rows are encoder op order (fine -> coarse) and are
        // consumed in reverse. Pick the order from the typed flag.
        const std::size_t rowIndex = paperTypedTableMode_
                                         ? nativeRoundIndex_++
                                         : pmsfSymbolTable()->size() - 1 - nativeRoundIndex_++;
        const auto& row = (*pmsfSymbolTable())[rowIndex];
        std::vector<bool> facetSymbols;
        std::vector<bool> edgeSymbols;
        std::vector<MCGAL::Point> points;
        for (auto s : row.delta.facetSymbols) facetSymbols.push_back(s != 0);
        for (auto s : row.delta.triangulationEdgeSymbols) edgeSymbols.push_back(s != 0);
        for (const auto& p : row.delta.removedVertexResiduals) {
            points.emplace_back(static_cast<double>(p.x()),
                                static_cast<double>(p.y()),
                                static_cast<double>(p.z()));
        }
                importFromParquet(facetSymbols, edgeSymbols, {}, points);
    }
    if (!useParquetMode_) {
        throw std::runtime_error(
            "[PPMC DEC] collectFromDecoded called without imported Parquet symbols");
    }

    int facetConsumed = 0, facetMissing = 0;
    auto consumeFacetSym = [this, &facetConsumed, &facetMissing]() -> bool {
        if (parquetFacetSymbols_.empty()) {
            const bool tolerant = std::getenv("PMSF_PPMC_NATIVE_TOLERANT") != nullptr;
            if (tolerant) { ++facetMissing; return false; }
            throw std::runtime_error(
                "[PPMC DEC] facet symbol queue underflow in collectFromDecoded");
        }
        ++facetConsumed;
        bool sym = parquetFacetSymbols_.front();
        parquetFacetSymbols_.pop_front();
        return sym;
    };

    auto consumeGeomSym = [this]() -> MCGAL::PointInt {
        if (parquetGeomSymbols_.empty()) {
            throw std::runtime_error(
                "[PPMC DEC] geometry symbol queue underflow in collectFromDecoded");
        }
        MCGAL::PointInt sym = parquetGeomSymbols_.front();
        parquetGeomSymbols_.pop_front();
        return sym;
    };

    auto consumeEdgeSym = [this]() -> unsigned {
        if (parquetEdgeSymbols_.empty()) {
            throw std::runtime_error(
                "[PPMC DEC] edge symbol queue underflow in collectFromDecoded");
        }
        bool sym = parquetEdgeSymbols_.front();
        parquetEdgeSymbols_.pop_front();
        return sym ? 1u : 0u;
    };

    // ========== 第一步：undecimation（面片分裂） ==========
    resetCurvatureNeighborCache();
    if (mesh_) {
        // 参考实现 beginUndecimationConquest 在每轮开始 resetState 所有半边
        // （清掉上一轮 reconstruct 打的 isNew 标记）。曲率 inverse 的
        // vertexDegreeNotNew 依赖该标记与编码端对称 —— 编码端从不设 New，
        // 因此解码端也必须在几何解码前清空。
        for (MCGAL::Facet* f : mesh_->faces()) {
            if (!f) continue;
            f->resetProcessedFlag();
            MCGAL::Halfedge* h0 = f->proxyHalfedge();
            if (!h0) continue;
            MCGAL::Halfedge* hh = h0;
            int guard = 0;
            do {
                if (hh) hh->resetState();
                hh = hh ? hh->next() : nullptr;
            } while (hh && hh != h0 && ++guard < 64);
        }
        // NOTE: do NOT pre-mark faces Unsplittable here — the BFS below skips
        // isConquered() faces, and Unsplittable counts as conquered. The encoder
        // does its "non-conquered → Unsplittable" pass AFTER elimination (which
        // has already marked the splittable faces); the decoder equivalent is a
        // post-BFS pass after the undecimation loop (see below).
    }
    gateQueue.push(seed->poolId());

    while (true) {
        if (gateQueue.empty()) {
            break;
        }
        int hIndex = gateQueue.front();
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndex(DEFAULT_MESH_ID, hIndex);
        gateQueue.pop();
        if (!h || !h->face()) continue;
        if (groupId >= 0 && !inGroup(h)) continue;

        MCGAL::Facet* f = h->face();

        if (f->isConquered()) {
            continue;
        }

        // 从队列中消费 facet symbol
        bool b_split = consumeFacetSym();

        // 添加邻居半边到队列
        MCGAL::Halfedge* hIt = h;
        while (hIt) {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp && hOpp->face() && !hOpp->face()->isConquered() &&
                (groupId < 0 || inGroup(hOpp))) {
                gateQueue.push(hOpp->poolId());
            }
            if (!hIt->next()) break;
            hIt = hIt->next();
            if (hIt == h) break;
        }

        if (b_split) {
            // 从队列中消费 geometry symbol（此时是 Frenet 残差）
            MCGAL::PointInt frenetResidual = consumeGeomSym();

            // ⚠️ P0 对齐：与 decodeGeometrySym 等价的反 Frenet + 加 barycenter 还原
            //   绝对量化坐标，PPMCReconstructOperator 端直接 intPosToFloat 即可。
            MCGAL::PointInt correction = frenetResidual;
#ifdef USE_BIJECTION
            // 反 Frenet 旋转（与编码端 ppmc::frenetRotationD 严格对称，double 精度）
            correction = ppmcInvFrenetResidual(frenetResidual, h, mesh_.get());
#endif

            if (b_useCurvaturePrediction)
                correction = correction - avgLaplacianVect(h) / 2;

            
            f->setSplittable();
            f->setProxyHalfedge(h);
            f->setRemovedVertexPosInt(correction);
            facets.push_back(f);
        } else {
            f->setUnsplittable();
        }
        f->setProcessedFlag();
    }

    // Decoder-side equivalent of the encoder's "non-conquered → Unsplittable"
    // pass: after the face BFS every live face must be classified (Splittable /
    // Unsplittable) before edge decoding and unlift consume the flags.
    if (mesh_) {
        for (MCGAL::Facet* f : mesh_->faces()) {
            if (f && !f->isRemoved() && !f->isConquered()) f->setUnsplittable();
        }
    }
    // ========== 第二步：inserted edge decoding（边合并） ==========
    gateQueue.push(seed->poolId());

    while (true) {
        if (gateQueue.empty()) {
            break;
        }
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndex(DEFAULT_MESH_ID, gateQueue.front());
        gateQueue.pop();

        if (!h || !h->face()) continue;
        if (groupId >= 0 && !inGroup(h)) continue;
        if (h->isProcessed())
            continue;

        h->setProcessed();
        if (h->opposite() && inGroup(h->opposite())) {
            h->opposite()->setProcessed();
        }

        // 添加邻居半边到队列
        MCGAL::Halfedge* hIt = h->next();
        while (hIt && hIt->opposite() && hIt->opposite() != h &&
               (groupId < 0 || inGroup(hIt->opposite()))) {
            if (!hIt->isProcessed() && !hIt->isNew() &&
                (groupId < 0 || inGroup(hIt))) {
                gateQueue.push(hIt->poolId());
            }
            hIt = hIt->opposite()->next();
        }
        // 只有当两个面中至少有一个是 splittable 时才有 edge symbol
        const bool sameGroupOpp =
            h->opposite() && h->opposite()->face() &&
            (groupId < 0 || inGroup(h->opposite()));
        const bool b_toCode = sameGroupOpp
                                  ? (h->face()->isSplittable() ||
                                     h->opposite()->face()->isSplittable())
                                  : h->face()->isSplittable();
        if (b_toCode) {
            bool b_original = true;
            unsigned sym = consumeEdgeSym();
            b_original = (sym == 0);

            if (!b_original) {
                h->setAdded();
                halfedges.push_back(h);
            }
        }
    }

    // PPMC lifting is owned by the reader. Callers must not unlift again
    // before reconstruction, otherwise vertex positions are shifted twice.
    if (b_useLiftingScheme && std::getenv("PMSF_PPMC_NATIVE_NOLIFT") == nullptr)
        lift(true, mesh_);

    useParquetMode_ = false;
}

// ═════════════════════════════════════════════════════════════════════════════
// ⚠️ P3.e 对齐（2026-05-14）：与原版 PPMC `mymeshLifting.cpp::lift(true)` 对齐。
//   解码端 unlift：在 InsertedEdgeDecoding 之后、insertRemovedVertices 之前，
//   对每个顶点的邻近 splittable face 残差求和，从顶点位置减去 (b_unlift=true)。
//   编码端对应 lift(false) 在 PPMCSymbolCollectOperator 中。
// ═════════════════════════════════════════════════════════════════════════════
void PPMCSymbolReadOperator::lift(bool b_unlift, std::shared_ptr<MCGAL::Mesh> mesh) {
    if (!mesh) return;
    MCGAL::Halfedge* startHalfedge = nullptr;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (f && !f->isRemoved() && f->proxyHalfedge()) {
            startHalfedge = f->proxyHalfedge();
            break;
        }
    }
    if (!startHalfedge) return;

    for (auto* v : mesh->vertices())
        v->resetState();

    gateQueue.push(startHalfedge->poolId());

    while (!gateQueue.empty()) {
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndex(DEFAULT_MESH_ID, gateQueue.front());
        gateQueue.pop();
        if (!inGroup(h)) {
            continue;
        }

        // 原版 CGAL `h->vertex()` 是半边**终点**；MCGAL 的 vertex() 是起点，
        // 终点是 end_vertex()。必须与编码端 lift 同用 end_vertex()，
        // 否则洪泛退化且编解码两侧的顶点覆盖不一致 → unlift 不抵消。
        MCGAL::Vertex* vh = h->end_vertex();
        if (!vh || vh->isRemoved()) {
            continue;
        }
        if (vh->isConquered())
            continue;
        vh->setConquered();

        // 邻居入队
        MCGAL::Halfedge* hIt = h->next();
        while (hIt && hIt->opposite() && hIt->opposite() != h) {
            if (hIt->end_vertex() && !hIt->end_vertex()->isConquered())
                gateQueue.push(hIt->poolId());
            hIt = hIt->opposite()->next();
        }

        // 累加邻近 splittable face 的残差
        MCGAL::PointInt liftAcc(0, 0, 0);
        int nNeighbours = 0;
        hIt = h;
        do {
            MCGAL::Facet* f = hIt->face();
            if (f && f->isSplittable()) {
                liftAcc = liftAcc + f->getRemovedVertexPosInt();
            }
            nNeighbours++;
            MCGAL::Halfedge* next = hIt->next();
            hIt = next ? next->opposite() : nullptr;
        } while (hIt && hIt != h);
        if (nNeighbours == 0) {
            continue;
        }

        MCGAL::PointInt quantPos = mesh->floatPosToInt(vh->point());

        if (b_unlift)
            liftAcc = MCGAL::PointInt(-liftAcc[0], -liftAcc[1], -liftAcc[2]);

        int div = nNeighbours * 2;  // INV_GAMMA
        MCGAL::PointInt delta(liftAcc[0] / div, liftAcc[1] / div, liftAcc[2] / div);
        MCGAL::PointInt newPosInt(quantPos[0] + delta[0],
                                  quantPos[1] + delta[1],
                                  quantPos[2] + delta[2]);

        // 与编码端严格对称的越界跳过：编码端只在 newPosInt 落在
        // [0, 2^(bits+1)) 内时才 setPoint；若编码端跳过，顶点位置未变，
        // 解码端此处 quantPos 与编码端一致、delta 相反，同一判定同样跳过。
        const int maxVal = 1 << (mesh->i_nbQuantBits + 1);
        bool outOfRange = false;
        for (int i = 0; i < 3; ++i) {
            if (newPosInt[i] < 0 || newPosInt[i] >= maxVal) {
                outOfRange = true;
                break;
            }
        }
        if (outOfRange)
            continue;

        vh->setPoint(mesh->intPosToFloat(newPosInt));
    }
}
