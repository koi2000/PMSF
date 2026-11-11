/*
 * @file        ValencePatchOnlySymbolCollectOperator.cpp
 * @brief       方案 V — Valence-Patch-Only encoder 实现。
 *
 * 设计要点：
 *   - collect(seed, gates) 直接 delegate 到父类 RDGateOnly：父类 BFS 已经把
 *     5 个 per-round 队列填好，我们只在 export 时投影到 6-元 alphabet。
 *     这保证 BFS 顺序、isPoly 解释、V_OTHER escape、edge-sym walk 全部
 *     与父类对称（关键不变量 §F R2）。
 *   - exportToBufferWithRangeCoder 把 (rcVal+rcIsPoly, rcEdge, rcPoint)
 *     重新打包为 ConnectivityBlobContainer v3 协议的 3 个子流。
 */

#include "ValencePatchOnlySymbolCollectOperator.h"

#include "compress/ConnectivityBlobContainer.h"
#include "compress/RangeCoderStreamEncoder.h"
#include "core/PMSFSideChannel.h"
#include "core/ValencePMSFSideChannel.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂
#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"
#include "Configuration.h"   // BUFFER_SIZE
#include "Mesh.h"
#include "Point.h"

#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

// 父类 enum 是 RDValenceCode (5 元: VAL_3=0..VAL_OTHER=4)，方案 V 6 元 alphabet
// 按 +1 映射：sym5 → sym6 = sym5 + 1（原 VAL_3=0 → VAL_3=1，etc）。
inline uint8_t mapFiveToSixIfPoly(uint8_t sym5) {
    return static_cast<uint8_t>(sym5 + 1);  // 0..4 → 1..5
}

}  // namespace

// =============================================================================
// collect entry points
// =============================================================================

void ValencePatchOnlySymbolCollectOperator::collect(MCGAL::Halfedge* seed) {
    // PMSF 旧路径（如 PMSF::pclttmValenceColumnarCompress 的 round loop）调
    // collector->collect(seed) 而不传 gates。我们直接 forward 到父类的带 gates
    // overload，传一个空 vector。父类只把 gates 用于 isPolySum 断言（mismatch 仅 WARN，
    // 不 throw），所以传空 vector 不会破坏数据流，只是 WARN 日志会打印
    // "isPoly-count != gates.size()=0" —— 这是预期的 cosmetic 输出。
    RDGateOnlySymbolCollectOperator::collect(seed, std::vector<MCGAL::Halfedge*>{});
}

void ValencePatchOnlySymbolCollectOperator::collect(std::vector<MCGAL::Halfedge*> seeds) {
    // PMSF segmentation 路径：每个 seed 一段 BFS。当前 Phase 1 不支持多 seed
    // partitioning，throw 提示。
    throw std::runtime_error(
        "[ValencePatchOnly] collect(vector<seed>) (PMSF segmentation) "
        "not supported in Phase 1; use single-seed PCLTTMValenceCompress.");
}

void ValencePatchOnlySymbolCollectOperator::collect(
    MCGAL::Halfedge* seed,
    const std::vector<MCGAL::Halfedge*>& gates) {
    // 直接 delegate 到父类：父类把每轮的 (isPoly bits, valence5 codes,
    // V_OTHER escapes, edge-sym 3-bit triplets, residuals) 填到 5 个 protected
    // 队列里。我们什么都不动 —— BFS 顺序、几何 invariants、断言都保留。
    RDGateOnlySymbolCollectOperator::collect(seed, gates);
}

// =============================================================================
// project / encode helpers
// =============================================================================

std::vector<uint8_t> ValencePatchOnlySymbolCollectOperator::project6Alphabet(
    const std::deque<uint8_t>& isPoly,
    const std::deque<uint8_t>& valenceCodes5) {
    std::vector<uint8_t> out;
    out.reserve(isPoly.size());
    size_t valCursor = 0;
    for (uint8_t bit : isPoly) {
        if (bit == 0) {
            out.push_back(static_cast<uint8_t>(ValencePatchCode::NULL_PATCH));
        } else {
            if (valCursor >= valenceCodes5.size()) {
                throw std::runtime_error(
                    "[ValencePatchOnly] project6Alphabet: valence5 stream "
                    "underflow (isPoly=1 but no matching valence symbol).");
            }
            out.push_back(mapFiveToSixIfPoly(valenceCodes5[valCursor++]));
        }
    }
    if (valCursor != valenceCodes5.size()) {
        std::cerr << "[ValencePatchOnly] WARN: project6Alphabet leftover valence5 "
                  << "symbols (consumed=" << valCursor
                  << " total=" << valenceCodes5.size() << ")" << std::endl;
    }
    return out;
}

std::vector<uint8_t> ValencePatchOnlySymbolCollectOperator::encodeValence6WithContext(
    const std::vector<uint8_t>& codes6,
    const std::deque<uint8_t>& otherVals) {
    // 4-context RC for 6-元（实际 alphabet_size = 8，sym=6/7 永不出现）。
    // context 推导：
    //   prev == -1（first）        → ctx=0
    //   prev == 0  (NULL_PATCH)    → ctx=1
    //   prev ∈ {1..4} (V_3..V_6)   → ctx=2
    //   prev == 5  (V_OTHER)       → ctx=3
    std::vector<char> buf(std::max<size_t>(codes6.size() * 2 + 1024, 4096));
    auto deriveCtx = [](int prev) {
        if (prev < 0) return 0;
        if (prev == 0) return 1;
        if (prev == 5) return 3;
        return 2;  // prev ∈ {1..4}
    };

    size_t rcSize = pmsf::encodeContextAdaptiveUint8Queue(
        std::deque<uint8_t>(codes6.begin(), codes6.end()),
        8,   // 6 元 padding 到 8
        4,   // 4 contexts
        deriveCtx,
        &otherVals,
        256,
        static_cast<int>(ValencePatchCode::VAL_OTHER),
        buf.data(),
        buf.size());

    int otherEmitted = 0;
    for (uint8_t sym : codes6) {
        if (sym == static_cast<uint8_t>(ValencePatchCode::VAL_OTHER)) ++otherEmitted;
    }

    std::vector<uint8_t> out;
    out.reserve(8 + rcSize);
    int32_t numFaces = static_cast<int32_t>(codes6.size());
    int32_t numOther = otherEmitted;
    out.resize(8);
    std::memcpy(out.data(), &numFaces, 4);
    std::memcpy(out.data() + 4, &numOther, 4);
    out.insert(out.end(),
               reinterpret_cast<uint8_t*>(buf.data()),
               reinterpret_cast<uint8_t*>(buf.data()) + rcSize);
    return out;
}

std::vector<uint8_t> ValencePatchOnlySymbolCollectOperator::encodeEdgeSyms(
    const std::deque<uint8_t>& edgeSyms) {
    std::vector<char> buf(std::max<size_t>(edgeSyms.size() / 4 + 1024, 4096));
    size_t rcSize = pmsf::encodeUint8Queue(
        edgeSyms, 2, buf.data(), buf.size());

    std::vector<uint8_t> out;
    out.reserve(4 + rcSize);
    int32_t numEdgeSyms = static_cast<int32_t>(edgeSyms.size());
    out.resize(4);
    std::memcpy(out.data(), &numEdgeSyms, 4);
    out.insert(out.end(),
               reinterpret_cast<uint8_t*>(buf.data()),
               reinterpret_cast<uint8_t*>(buf.data()) + rcSize);
    return out;
}

std::vector<uint8_t> ValencePatchOnlySymbolCollectOperator::encodeResidualBitplane(
    const std::deque<MCGAL::PointInt>& residuals,
    int BP_BITS) {
    std::vector<char> buf(BUFFER_SIZE / 3, 0);
    size_t rcSize = pmsf::encodeResidualBitPlaneQueue(
        residuals, BP_BITS, buf.data(), buf.size());

    std::vector<uint8_t> out;
    out.reserve(4 + rcSize);
    int32_t numCoords = static_cast<int32_t>(residuals.size() * 3);
    out.resize(4);
    std::memcpy(out.data(), &numCoords, 4);
    out.insert(out.end(),
               reinterpret_cast<uint8_t*>(buf.data()),
               reinterpret_cast<uint8_t*>(buf.data()) + rcSize);
    return out;
}

// =============================================================================
// exportToBufferWithRangeCoder
// =============================================================================

int ValencePatchOnlySymbolCollectOperator::exportToBufferWithRangeCoder(
    char* buffer, bool isEnableQuantization) {
    if (buffer == nullptr) {
        // CompressPipeline 的 probe 路径：仅触发 summary。
        return 0;
    }
    if (!enablePrediction_ || !isEnableQuantization) {
        throw std::runtime_error(
            "[ValencePatchOnly] exportRC requires enablePrediction && enableQuantization.");
    }

    const int numRounds = static_cast<int>(gateIsPolyQueues_.size());
    if (static_cast<int>(gateValenceQueues_.size()) != numRounds ||
        static_cast<int>(gateEdgeSymQueues_.size()) != numRounds ||
        static_cast<int>(gateResidualQueues_.size()) != numRounds ||
        static_cast<int>(gateOtherValQueues_.size()) != numRounds) {
        throw std::runtime_error(
            "[ValencePatchOnly] inconsistent per-round queue sizes after parent BFS");
    }

    pmsf::ConnectivityBlobBuilder cbb(
        pmsf::valencePatchOnlyConnectivityBlobSpec(numRounds));

    int sumValence6Bytes = 0, sumEdgeBytes = 0, sumResidualBytes = 0;
    int totalSymbolsEmitted = 0;

    // 与父类一致：encoder round 0 是第一轮 decimation；decoder mesh 从 base mesh
    // 出发，需要 encoder 最后一轮的数据先到。因此**逆序**填 ConnectivityBlob 的
    // round id：encoder round (numRounds-1-i) 写到 builder 的 round i。
    const int BP_BITS = static_cast<int>(mesh_->i_nbQuantBits) + 1;

    for (int i = 0; i < numRounds; ++i) {
        int er = numRounds - 1 - i;  // encoder round 编号（输入侧）

        // stream 0 — 6 元 alphabet + 4-context RC + V_OTHER escape
        auto codes6 = project6Alphabet(gateIsPolyQueues_[er], gateValenceQueues_[er]);
        totalSymbolsEmitted += static_cast<int>(codes6.size());
        auto rcVal = encodeValence6WithContext(codes6, gateOtherValQueues_[er]);
        sumValence6Bytes += static_cast<int>(rcVal.size());
        cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamValencePatchOnlyValence6, rcVal);

        // stream 1 — edge-sym (per polyFace 3 bit, RC 2 元)
        auto rcEdge = encodeEdgeSyms(gateEdgeSymQueues_[er]);
        sumEdgeBytes += static_cast<int>(rcEdge.size());
        cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamValencePatchOnlyEdgeSym, rcEdge);

        // stream 2 — bit-plane 残差
        auto rcRes = encodeResidualBitplane(gateResidualQueues_[er], BP_BITS);
        sumResidualBytes += static_cast<int>(rcRes.size());
        cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamValencePatchOnlyResidual, rcRes);
    }

    auto blob = cbb.finalize();
    std::memcpy(buffer, blob.data(), blob.size());

    lastValence6Size_  = sumValence6Bytes;
    lastEdgeSize_      = sumEdgeBytes;
    lastResidualSize_  = sumResidualBytes;
    lastBlobTotalSize_ = static_cast<int>(blob.size());

    const char* envBpv = std::getenv("PMSF_BPV_DIAG");
    if (envBpv && envBpv[0] == '1') {
        std::cerr << "[ValencePatchOnly bpv] valence6=" << sumValence6Bytes
                  << " edge=" << sumEdgeBytes
                  << " residual=" << sumResidualBytes
                  << " containerOverhead=" << (blob.size() - sumValence6Bytes
                                               - sumEdgeBytes - sumResidualBytes)
                  << " totalBlob=" << blob.size()
                  << " rounds=" << numRounds
                  << " symbols=" << totalSymbolsEmitted << std::endl;
    }
    return static_cast<int>(blob.size());
}

const pmsf::PMSFSymbolTable* ValencePatchOnlySymbolCollectOperator::pmsfSymbolTable() const {
    const pmsf::PMSFSymbolTable* base = RDGateOnlySymbolCollectOperator::pmsfSymbolTable();
    if (base == nullptr || base->empty()) {
        return nullptr;
    }
    const std::string prefix = pmsfChannelPrefix();
    const std::deque<uint8_t> emptyValenceCodes;
    pmsf::copyPMSFSymbolTableWithIndexedRowAugmentation(
        base,
        paperTableCache_,
        [this, &prefix, &emptyValenceCodes](
            std::size_t r, pmsf::PMSFSymbolRecord& row) {
        if (r >= gateIsPolyQueues_.size()) {
            return;
        }
        const auto& valenceCodes =
            (r < gateValenceQueues_.size()) ? gateValenceQueues_[r] : emptyValenceCodes;
        auto codes6 = project6Alphabet(gateIsPolyQueues_[r], valenceCodes);
        pmsf::replaceValenceVertexSymbolsFromCodes(
            row.delta, codes6);
        pmsf::replaceValencePatch6CodeSideChannel(
            row.delta, prefix, codes6);
    });
    return paperTableCache_.empty() ? nullptr : &paperTableCache_;
}

int ValencePatchOnlySymbolCollectOperator::exportToBuffer(char* /*buffer*/,
                                                          bool /*enableQuantization*/,
                                                          bool /*enableCompress*/) {
    throw std::runtime_error(
        "[ValencePatchOnly] exportToBuffer (non-RC) not supported "
        "(Phase 1 only Columnar + range-coded blob).");
}

int ValencePatchOnlySymbolCollectOperator::totalValenceSymbolsEmitted() const {
    int total = 0;
    for (const auto& q : gateIsPolyQueues_) {
        total += static_cast<int>(q.size());
    }
    return total;
}
