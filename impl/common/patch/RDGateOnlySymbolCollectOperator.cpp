/*
 * @file RDGateOnlySymbolCollectOperator.cpp
 * @brief RD Phase B encoder — v2 协议（BFS-symmetric + is-polyFace bit + V_OTHER escape）。
 *
 * 见头文件注释。本协议与 `RDGateOnlySymbolReadOperator` round-trip。
 */
#include "common/patch/RDGateOnlySymbolCollectOperator.h"
#include "compress/BFSConquestCollector.h"
#include "compress/RangeCoderStreamEncoder.h"
#include "core/PMSFBoundarySideChannel.h"
#include "core/PaperGeometrySideChannel.h"
#include "RD/boundary/RDPMSFBoundary.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include "BufferUtils.h"
#include "Configuration.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Point.h"
#include "RD/compress/RDSymbolCollectOperator.h"  // RDValenceCode alphabet reuse
#include "core.h"

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

void requireRDNonStrictPaperExactIfRequested(const char* label) {
    if (!pmsf::rdPaperExactRequired()) {
        return;
    }
    throw std::runtime_error(
        std::string(label) +
        " is not paper-exact RD: prioritized gate selection, curvature "
        "prediction, and KLT replay are not fully implemented in this "
        "non-strict path.");
}

}  // namespace

// ---------- collect entry points ----------

void RDGateOnlySymbolCollectOperator::collect(MCGAL::Halfedge* /*seed*/) {
    // Phase B 只接受带 gates 的 overload；无 gates 的 single-seed 调用意味着上游
    // CompressPipeline 没有把 perRoundGates 传过来（或者旧的 Segmentation 路径），
    // 结果会是空 valence stream → BPV = 0 的假测量。直接 throw 方便发现。
    throw std::runtime_error(
        "[RDGateOnly] collect(seed) called without gates; pipeline must call "
        "collect(seed, perRoundGates). See CompressPipeline.cpp:360.");
}

void RDGateOnlySymbolCollectOperator::collect(std::vector<MCGAL::Halfedge*> /*seeds*/) {
    throw std::runtime_error(
        "[RDGateOnly] collect(vector<seed>) called without gates; PMSF "
        "Segmentation path is not supported in Phase B.");
}

// ---------- v2 协议：BFS from seed，每 face 首访发 is-polyFace bit + 条件 valence/residual ----------
//
// 对称复刻 ValenceSymbolReadOperator::collectFromDecoded 的 BFS 顺序：
//   opposite 先入队，自身 sibling 再入队，循环 face 三条边。
// 每 face 首访时：
//   1. 读 f->isSplittable() 决定 isPoly bit。
//   2. isPoly=1：取 valence code（V_3..V_6/V_OTHER），若 V_OTHER 记录真实 valence，
//      取 f->getRemovedVertexPosInt() 作残差。
//   3. isPoly=0：跳过 valence/residual。
//
void RDGateOnlySymbolCollectOperator::collect(MCGAL::Halfedge* seed,
                                              const std::vector<MCGAL::Halfedge*>& gates) {
    const int roundIdx = static_cast<int>(gateValenceQueues_.size());

    // 本轮输出队列
    std::deque<uint8_t> isPolyBits;
    std::deque<uint8_t> valenceCodes;
    std::deque<uint8_t> otherVals;
    std::deque<uint8_t> edgeSyms;
    std::deque<MCGAL::PointInt> residuals;

    int v3 = 0, v4 = 0, v5 = 0, v6 = 0, vOther = 0;
    int bfsFaceIdx = 0;
    long long gateNbrSum = 0, isolatedGates = 0, frontFaces = 0, frontNonGate = 0;

    struct Visitor : pmsf::BFSConquestVisitor {
        std::deque<uint8_t>* isPolyBits;
        std::deque<uint8_t>* valenceCodes;
        std::deque<uint8_t>* otherVals;
        std::deque<uint8_t>* edgeSyms;
        std::deque<MCGAL::PointInt>* residuals;
        int* v3, *v4, *v5, *v6, *vOther;
        int* bfsFaceIdx;
        // 诊断（PMSF_RD_DIAG）：门的"门邻域"统计，判定"门引导扩张"是否可行。
        long long* gateNbrSum;
        long long* isolatedGates;
        // 诊断：前沿规模 —— 「收紧为前沿 conquest」这条路径的收益上界。
        long long* frontFaces;    // 有 ≥1 个门邻居的受访面（= 受影响区域）
        long long* frontNonGate;  // 其中自身不是门的那些（仍需 flag 的部分）

        void onFaceFirstVisit(MCGAL::Facet* f,
                              MCGAL::Halfedge* gate,
                              int idx) override {
            bool isPoly = f->isSplittable();
            isPolyBits->push_back(isPoly ? 1 : 0);

            // —— 诊断：门的面邻接里有几个也是门 ——
            // 若门彼此相邻成片，"门引导扩张"（只从门继续扩张）就能以 O(门数) 访问量
            // 覆盖全部门 ⇒ isPoly 流可整条消失；若门多为孤立，该方向不可行。
            // 只读 f/邻面的 isSplittable()，不写任何队列，不影响产物字节。
            {
                int gateNbr = 0;
                MCGAL::Halfedge* he = gate;
                if (he != nullptr) {
                    do {
                        MCGAL::Halfedge* opp = he->opposite();
                        if (opp != nullptr && opp->face() != nullptr &&
                            opp->face()->isSplittable()) {
                            ++gateNbr;
                        }
                        he = he->next();
                    } while (he != gate && he != nullptr);
                }
                if (isPoly) {
                    *gateNbrSum += gateNbr;
                    if (gateNbr == 0) ++(*isolatedGates);
                }
                // 前沿规模：本面只要挨着任一扇门，就必须被"前沿 conquest"访问到。
                if (gateNbr > 0) {
                    ++(*frontFaces);
                    if (!isPoly) ++(*frontNonGate);
                }
            }

            int emittedValence = -1;
            MCGAL::PointInt residual(0, 0, 0);

            if (isPoly) {
                int valence = f->getPolyValence();
                RDValenceCode code;
                switch (valence) {
                    case 3: code = RDValenceCode::VAL_3; (*v3)++; break;
                    case 4: code = RDValenceCode::VAL_4; (*v4)++; break;
                    case 5: code = RDValenceCode::VAL_5; (*v5)++; break;
                    case 6: code = RDValenceCode::VAL_6; (*v6)++; break;
                    default:
                        code = RDValenceCode::VAL_OTHER;
                        (*vOther)++;
                        if (valence < 0 || valence > 255) {
                            std::cerr << "[RDGateOnly] V_OTHER valence=" << valence
                                      << " out of uint8 range; clamping" << std::endl;
                            valence = (valence < 0) ? 0 : 255;
                        }
                        otherVals->push_back(static_cast<uint8_t>(valence));
                        break;
                }
                valenceCodes->push_back(static_cast<uint8_t>(code));
                residual = f->getRemovedVertexPosInt();
                residuals->push_back(residual);
                emittedValence = valence;
            }

            MCGAL::Halfedge* hit = gate;
            for (int k = 0; k < 3 && hit != nullptr; ++k) {
                edgeSyms->push_back(hit->isAdded() ? 1 : 0);
                hit = hit->next();
            }

            *bfsFaceIdx = idx + 1;
        }
    };

    Visitor visitor;
    visitor.isPolyBits = &isPolyBits;
    visitor.valenceCodes = &valenceCodes;
    visitor.otherVals = &otherVals;
    visitor.edgeSyms = &edgeSyms;
    visitor.residuals = &residuals;
    visitor.v3 = &v3; visitor.v4 = &v4; visitor.v5 = &v5; visitor.v6 = &v6; visitor.vOther = &vOther;
    visitor.bfsFaceIdx = &bfsFaceIdx;
    visitor.gateNbrSum = &gateNbrSum;
    visitor.isolatedGates = &isolatedGates;
    visitor.frontFaces = &frontFaces;
    visitor.frontNonGate = &frontNonGate;

    pmsf::runFaceConquestBFS(seed, gates, mesh_.get(), visitor);


    int isPolySum = 0;
    for (uint8_t b : isPolyBits) isPolySum += b;
    if (isPolySum != (int)gates.size()) {
        std::cerr << "[RDGateOnly] WARN round=" << roundIdx
                  << " isPoly-count=" << isPolySum
                  << " != gates.size()=" << gates.size()
                  << " (may indicate BFS misses some polyFaces)" << std::endl;
    }

    gateIsPolyQueues_.push_back(std::move(isPolyBits));
    gateValenceQueues_.push_back(std::move(valenceCodes));
    gateOtherValQueues_.push_back(std::move(otherVals));
    gateEdgeSymQueues_.push_back(std::move(edgeSyms));
    gateResidualQueues_.push_back(std::move(residuals));
    paperTableCache_.clear();

    const char* env = std::getenv("PMSF_RD_DIAG");
    if (env && env[0] == '1') {
        std::cerr << "[RD-GateOnly round=" << roundIdx
                  << "] faces_bfs=" << bfsFaceIdx
                  << " polyFaces=" << isPolySum
                  << " gates.size()=" << gates.size()
                  << " V_3=" << v3 << " V_4=" << v4 << " V_5=" << v5
                  << " V_6=" << v6 << " V_OTHER=" << vOther
                  << " gateNbrSum=" << gateNbrSum
                  << " isolatedGates=" << isolatedGates
                  << " gateNbrAvg=" << (isPolySum ? (double)gateNbrSum / (double)isPolySum : 0.0)
                  << " frontFaces=" << frontFaces
                  << " frontNonGate=" << frontNonGate
                  << std::endl;
    }
}

const pmsf::PMSFSymbolTable* RDGateOnlySymbolCollectOperator::pmsfSymbolTable() const {
    paperTableCache_.clear();
    for (std::size_t r = 0; r < gateIsPolyQueues_.size(); ++r) {
        pmsf::PMSFSymbolRecord& row =
            pmsf::appendPMSFSymbolRecord(paperTableCache_, 0, 0, static_cast<std::int32_t>(r));

        const std::string prefix = pmsfChannelPrefix();
        pmsf::appendRDPMSFBoundaryExactPairs(row.delta, prefix);
        for (uint8_t bit : gateIsPolyQueues_[r]) {
            pmsf::appendRDGateIsPolySymbol(row.delta, bit != 0);
        }
        if (r < gateValenceQueues_.size()) {
            for (uint8_t code : gateValenceQueues_[r]) {
                pmsf::appendRDGateValenceCodeSymbol(
                    row.delta, prefix, static_cast<std::int32_t>(code));
            }
        }
        if (r < gateOtherValQueues_.size()) {
            for (uint8_t value : gateOtherValQueues_[r]) {
                pmsf::appendRDGateOtherValenceSymbol(
                    row.delta, prefix, static_cast<std::int32_t>(value));
            }
        }
        if (r < gateEdgeSymQueues_.size()) {
            for (uint8_t bit : gateEdgeSymQueues_[r]) {
                pmsf::appendRDGateTriangulationEdgeSymbol(row.delta, bit != 0);
            }
        }
        if (r < gateResidualQueues_.size()) {
            for (const auto& q : gateResidualQueues_[r]) {
                pmsf::appendRDGateResidualMirror(row.delta, prefix, q);
            }
        }
    }
    return paperTableCache_.empty() ? nullptr : &paperTableCache_;
}

// ---------- range-coded export ----------
//
// 布局（v3，2026-05-09 几何修复）：
//   [rcValSize:int32][rcIsPolySize:int32][rcEdgeSize:int32][rcPointSize:int32]
//   [numFaceBits:int32][numGates:int32][numEdgeSyms:int32][numPointCoords:int32]
//   [numRounds:int32][faceBitsPerRound:int32 × numRounds]
//   [rcValBuf:rcValSize bytes]
//   [rcIsPolyBuf:rcIsPolySize bytes]
//   [rcEdgeBuf:rcEdgeSize bytes]   ★ NEW v3
//   [rcPointBuf:rcPointSize bytes]
//
// RC 段：
//   - rcVal：5 元主 qsmodel；遇到 sym==V_OTHER 立即 inline 256 元 qsmodel 编码真实
//     valence（两模型共用同一 rangecoder）。
//   - rcIsPoly：2 元 adaptive qsmodel。
//   - rcEdge：2 元 adaptive qsmodel（per-polyFace 3 bit；NULL_PATCH face 不发）。 ★ NEW v3
//   - rcPoint：verbatim Valence 的 bit-plane 残差（ZigZag + per-bit bpFirst1/bpAfter1）。
//
int RDGateOnlySymbolCollectOperator::exportToBufferWithRangeCoder(char* buffer,
                                                                  bool isEnableQuantization) {
    if (buffer == nullptr) {
        // 仅用于 CompressPipeline 的 `probeCollector->exportTo*(nullptr, false)`
        // 触发 summary 打印；实际工具路径永远传真 buffer。
        return 0;
    }
    requireRDNonStrictPaperExactIfRequested("[RDGateOnly]");

    size_t offset = 0;

    // 与 Valence 分配比例大致对齐：valence 5-元熵低(~1.83 bit)，isPoly 也低(~0.7 bit)，
    // edge 段 numGates*3 bits ≈ 100K bits 即 ~12 KB raw → ~7-10 KB RC，
    // point 段最大（~17.74 bit per polyFace × 3 coords）。
    constexpr size_t VAL_BUF = BUFFER_SIZE / 12;
    constexpr size_t ISPOLY_BUF = BUFFER_SIZE / 12;
    constexpr size_t EDGE_BUF = BUFFER_SIZE / 8;
    constexpr size_t POINT_BUF = BUFFER_SIZE / 3;
    std::vector<char> rcValBuf(VAL_BUF, 0);
    std::vector<char> rcIsPolyBuf(ISPOLY_BUF, 0);
    std::vector<char> rcEdgeBuf(EDGE_BUF, 0);
    std::vector<char> rcPointBuf(POINT_BUF, 0);

    int totalFaceBits = 0, totalGates = 0, totalEdgeSyms = 0, totalPointCoords = 0;
    for (auto& q : gateIsPolyQueues_) totalFaceBits += (int)q.size();
    for (auto& q : gateValenceQueues_) totalGates += (int)q.size();
    for (auto& q : gateEdgeSymQueues_) totalEdgeSyms += (int)q.size();
    for (auto& q : gateResidualQueues_) totalPointCoords += (int)q.size() * 3;

    size_t rcValSize = 0, rcIsPolySize = 0, rcEdgeSize = 0, rcPointSize = 0;

    // ---- valence RC 段（含 V_OTHER inline escape）----
    rcValSize = pmsf::encodeValence5WithEscape(
        gateValenceQueues_, gateOtherValQueues_, true, rcValBuf.data(), rcValBuf.size());

    // ---- is-polyFace RC 段（2 元 adaptive）----
    rcIsPolySize = pmsf::encodeUint8Queues(
        gateIsPolyQueues_, 2, true, rcIsPolyBuf.data(), rcIsPolyBuf.size());

    // ---- ★ v3 NEW — edge-sym RC 段（2 元 adaptive，per polyFace 3 bit）----
    rcEdgeSize = pmsf::encodeUint8Queues(
        gateEdgeSymQueues_, 2, true, rcEdgeBuf.data(), rcEdgeBuf.size());

    // ---- point RC 段（bit-plane 残差；verbatim Valence 路径）----
    {
        unsigned nbQuantBits = mesh_->i_nbQuantBits;
        const bool useResidualBitPlane = enablePrediction_ && isEnableQuantization;
        if (!useResidualBitPlane) {
            throw std::runtime_error(
                "[RDGateOnly] exportRC requires enablePrediction && enableQuantization. "
                "CompressOptions missing one of them.");
        }
        const int BP_BITS = static_cast<int>(nbQuantBits) + 1;
        rcPointSize = pmsf::encodeResidualBitPlaneQueues(
            gateResidualQueues_, BP_BITS, true, rcPointBuf.data(), rcPointBuf.size());
    }

    // ---- header ----
    writeInt(buffer, offset, (int)rcValSize);
    writeInt(buffer, offset, (int)rcIsPolySize);
    writeInt(buffer, offset, (int)rcEdgeSize);
    writeInt(buffer, offset, (int)rcPointSize);
    writeInt(buffer, offset, totalFaceBits);
    writeInt(buffer, offset, totalGates);
    writeInt(buffer, offset, totalEdgeSyms);
    writeInt(buffer, offset, totalPointCoords);

    int numRounds = (int)gateIsPolyQueues_.size();
    writeInt(buffer, offset, numRounds);
    // faceBitsPerRound[] 也按逆序写：header 中的 idx 0 对应 encoder 最后一轮（即
    // decoder 第一轮），方便 decoder 用 currentRoundIdx_ 直接索引。
    for (int i = numRounds - 1; i >= 0; i--) {
        writeInt(buffer, offset, (int)gateIsPolyQueues_[i].size());
    }

    writeCharPointer(buffer, offset, rcValBuf.data(), (int)rcValSize);
    writeCharPointer(buffer, offset, rcIsPolyBuf.data(), (int)rcIsPolySize);
    writeCharPointer(buffer, offset, rcEdgeBuf.data(), (int)rcEdgeSize);
    writeCharPointer(buffer, offset, rcPointBuf.data(), (int)rcPointSize);

    lastRcValenceSize_ = (int)rcValSize;
    lastRcIsPolySize_ = (int)rcIsPolySize;
    lastRcEdgeSize_ = (int)rcEdgeSize;
    lastRcPointSize_ = (int)rcPointSize;

    const char* envBpv = std::getenv("PMSF_BPV_DIAG");
    if (envBpv && envBpv[0] == '1') {
        std::cerr << "[RD-GateOnly bpv] valence=" << rcValSize
                  << " (raw_codes=" << totalGates << ")"
                  << " isPoly=" << rcIsPolySize
                  << " (raw_bits=" << totalFaceBits << ")"
                  << " edge=" << rcEdgeSize
                  << " (raw_bits=" << totalEdgeSyms << ")"
                  << " point=" << rcPointSize
                  << " (coords=" << totalPointCoords << ")"
                  << " rounds=" << numRounds << std::endl;
    }
    return (int)offset;
}

// ---- plain export：Phase B 不支持非 RC 路径（Binary / V2 legacy），直接报错 ----
int RDGateOnlySymbolCollectOperator::exportToBuffer(char* /*buffer*/,
                                                    bool /*enableQuantization*/,
                                                    bool /*enableCompress*/) {
    throw std::runtime_error(
        "[RDGateOnly] exportToBuffer (non-RC) is not implemented. "
        "Phase B only supports Parquet V2 + range-coded blob.");
}
