// SPDX-License-Identifier: Apache-2.0
//
// PCATMPureSymbolCollectOperator.cpp — see header for design contract.
//
// 实现策略：复用父类 PCATMPMSFSymbolCollectOperator 的 helpers（valence/isPoly/residual
// 三种 RC 编码），字节布局与 PCATMPaper 的 3 段 subset 完全相同，仅：
//   - ConnectivityBlobBuilder blob_layout 改为 21
//   - num_streams 4 → 3
//   - 不写 color 段（即使 gateColorQueues_ 被父类 collect() 填充也不消费）

#include "PCATM/compress/PCATMPureSymbolCollectOperator.h"
#include "common/PaperLayoutAliases.h"
#include "core/PMSFBoundarySideChannel.h"
#include "PCATM/boundary/PCATMPMSFBoundary.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂
#include "storage/PMSFSymbolStorageAdapter.h"

#include "compress/ConnectivityBlobContainer.h"
#include "compress/RangeCoderStreamEncoder.h"
#include "Configuration.h"
#include "Mesh.h"
#include "Point.h"
#include "RD/compress/RDSymbolCollectOperator.h"  // RDValenceCode

#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

constexpr int kBufScratchPure = BUFFER_SIZE / 12;

bool pcatmEnvOn(const char* name) {
    return pmsf::pmsfStorageEnvEnabled(name);
}

void requirePCATMExactIfRequested(const char* label) {
    if (!pmsf::boundaryFlagOn(pmsf::pcatm_boundary::kPaperExactSpec)) {
        return;
    }
    throw std::runtime_error(
        std::string(label) +
        " is not paper-exact: PCATM pure/Z-shape reconstruction is still "
        "unverified.");
}

/// stream 0: valence (5-元 + 4-context + V_OTHER inline 256-元 escape)
std::vector<uint8_t> encodeValence5WithContextPure(const std::deque<uint8_t>& valCodes,
                                                    const std::deque<uint8_t>& otherVals) {
    std::vector<char> buf(std::max<size_t>(kBufScratchPure, valCodes.size() * 2 + 1024));
    auto deriveCtx = [](int prev) {
        if (prev < 0) return 0;
        if (prev == (int)RDValenceCode::VAL_3 ||
            prev == (int)RDValenceCode::VAL_4) return 1;
        if (prev == (int)RDValenceCode::VAL_5 ||
            prev == (int)RDValenceCode::VAL_6) return 2;
        return 3;
    };

    size_t rcSize = pmsf::encodeContextAdaptiveUint8Queue(
        valCodes,
        8,   // 5 元 padding 到 8
        4,   // 4 contexts
        deriveCtx,
        &otherVals,
        256,
        (int)RDValenceCode::VAL_OTHER,
        buf.data(),
        buf.size());

    std::vector<uint8_t> out;
    out.resize(8);
    int32_t numPoly  = (int32_t)valCodes.size();
    int32_t numOther = (int32_t)otherVals.size();
    std::memcpy(out.data(),     &numPoly,  4);
    std::memcpy(out.data() + 4, &numOther, 4);
    out.insert(out.end(),
               reinterpret_cast<uint8_t*>(buf.data()),
               reinterpret_cast<uint8_t*>(buf.data()) + rcSize);
    return out;
}

std::vector<uint8_t> encodeBitstream2RCPure(const std::deque<uint8_t>& bits) {
    std::vector<char> buf(std::max<size_t>(bits.size() / 4 + 1024, 4096));
    size_t rcSize = pmsf::encodeUint8Queue(
        bits, 2, buf.data(), buf.size());

    std::vector<uint8_t> out;
    out.resize(4);
    int32_t numBits = (int32_t)bits.size();
    std::memcpy(out.data(), &numBits, 4);
    out.insert(out.end(),
               reinterpret_cast<uint8_t*>(buf.data()),
               reinterpret_cast<uint8_t*>(buf.data()) + rcSize);
    return out;
}

/// PR-K4 (sv=2): 8-context RC for isPoly stream (mirror RDTag PR-J1)。
std::vector<uint8_t> encodeIsPolyMultiContextRCPure(const std::deque<uint8_t>& bits) {
    constexpr int kHistoryBits = 3;
    constexpr int kNumCtx = 1 << kHistoryBits;
    constexpr int kHistoryMask = kNumCtx - 1;

    std::vector<char> buf(std::max<size_t>(bits.size() / 4 + 1024, 4096));
    int history = 0;
    auto deriveCtx = [&history](int prev) {
        int ctx = history & kHistoryMask;
        if (prev >= 0) {
            history = ((history << 1) | prev) & kHistoryMask;
        }
        return ctx;
    };

    size_t rcSize = pmsf::encodeContextAdaptiveUint8Queue(
        bits,
        2,        // bit alphabet
        kNumCtx,
        deriveCtx,
        nullptr,  // no escape
        0,
        -1,
        buf.data(),
        buf.size());

    std::vector<uint8_t> out;
    out.resize(4);
    int32_t numBits = (int32_t)bits.size();
    std::memcpy(out.data(), &numBits, 4);
    out.insert(out.end(),
               reinterpret_cast<uint8_t*>(buf.data()),
               reinterpret_cast<uint8_t*>(buf.data()) + rcSize);
    return out;
}

std::vector<uint8_t> encodeResidualBitplanePure(
    const std::deque<MCGAL::PointInt>& residuals, int BP_BITS) {
    // ── P15（2026-09-24）：200 MB 缓冲由「每次调用零填充」改为**进程内复用** ──
    // 本函数**每轮调用一次**（逐轮残差流）。原写法
    // `std::vector<char> buf(BUFFER_SIZE / 3, 0)` 每轮把 200 MB **显式清零**
    // —— 50 轮即 10 GB 的 memset。与 PPMC `measBuf` 同一根因
    //（见 PPMCSymbolCollectOperator.cpp 的同名注释与 P15 实测：PPMC −93.6%）。
    //
    // 复用后同一块内存只在首次触碰时由内核清零一次。
    // 安全性：`encodeResidualBitPlaneQueue` 从 0 起顺序写入，缓冲是纯输出草稿。
    // 判据：门禁 60/60 逐字节相同（硬约束 1）。
    struct PureResBuf {
        char* buf = nullptr;
        std::size_t cap = 0;
        PureResBuf() : buf(new char[BUFFER_SIZE / 3]), cap(BUFFER_SIZE / 3) {}
    };
    static thread_local PureResBuf pureResScratch;
    char* const buf = pureResScratch.buf;
    const std::size_t bufSize = pureResScratch.cap;
    size_t rcSize = pmsf::encodeResidualBitPlaneQueue(
        residuals, BP_BITS, buf, bufSize);

    std::vector<uint8_t> out;
    out.resize(4);
    int32_t numCoords = (int32_t)(residuals.size() * 3);
    std::memcpy(out.data(), &numCoords, 4);
    out.insert(out.end(),
               reinterpret_cast<uint8_t*>(buf),
               reinterpret_cast<uint8_t*>(buf) + rcSize);
    return out;
}

}  // namespace

// ============================================================================
// export — ConnectivityBlobBuilder v3 (3 段流，sv=1, blob_layout=21)
// ============================================================================
int PCATMPureSymbolCollectOperator::exportToBufferWithRangeCoder(char* buffer,
                                                                  bool isEnableQuantization) {
    if (buffer == nullptr) return 0;
    requirePCATMExactIfRequested("[PCATMPure]");
    if (!enablePrediction_ || !isEnableQuantization) {
        throw std::runtime_error(
            "[PCATMPure] exportRC requires enablePrediction && enableQuantization.");
    }

    const int numRounds = (int)gateValenceQueues_.size();
    if ((int)gateIsPolyQueues_.size()   != numRounds ||
        (int)gateResidualQueues_.size() != numRounds) {
        throw std::runtime_error(
            "[PCATMPure sv=1/2] inconsistent per-round queue sizes "
            "(expects valence/isPoly/residual aligned by round)");
    }

    int schemeVersion = kPCATMPureSchemeVersion;
    if (const char* envSv = std::getenv("PMSF_PCATM_PURE_SV")) {
        if (envSv[0] == '2') {
            schemeVersion = pmsf::kPaperBlobSchemeVersionPCATMPureV2;
        }
    }

    pmsf::ConnectivityBlobBuilder cbb(
        pmsf::pcatmPureConnectivityBlobSpec(numRounds, schemeVersion));

    int sumValence = 0, sumIsPoly = 0, sumRes = 0;
    const int BP_BITS = (int)mesh_->i_nbQuantBits + 1;

    for (int i = 0; i < numRounds; ++i) {
        int er = numRounds - 1 - i;

        auto rcVal = encodeValence5WithContextPure(gateValenceQueues_[er],
                                                    gateOtherValQueues_[er]);
        sumValence += (int)rcVal.size();
        cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamPCATMPureValence, rcVal);

        auto rcIsPoly = pmsf::paperBlobSchemeVersionPCATMPureUsesMultiContextIsPoly(
                              schemeVersion)
                          ? encodeIsPolyMultiContextRCPure(gateIsPolyQueues_[er])
                          : encodeBitstream2RCPure(gateIsPolyQueues_[er]);
        sumIsPoly += (int)rcIsPoly.size();
        cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamPCATMPureIsPoly, rcIsPoly);

        auto rcRes = encodeResidualBitplanePure(gateResidualQueues_[er], BP_BITS);
        sumRes += (int)rcRes.size();
        cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamPCATMPureResidual, rcRes);
    }

    auto blob = cbb.finalize();
    std::memcpy(buffer, blob.data(), blob.size());

    const char* envBpv = std::getenv("PMSF_BPV_DIAG");
    if (envBpv && envBpv[0] == '1') {
        int overhead = (int)blob.size() - sumValence - sumIsPoly - sumRes;
        std::cerr << "[PCATMPure bpv sv=" << schemeVersion << "] valence=" << sumValence
                  << " isPolyBit=" << sumIsPoly
                  << " residual=" << sumRes
                  << " containerOverhead=" << overhead
                  << " totalBlob=" << blob.size()
                  << " rounds=" << numRounds << std::endl;
    }
    return (int)blob.size();
}

const pmsf::PMSFSymbolTable* PCATMPureSymbolCollectOperator::pmsfSymbolTable() const {
    const pmsf::PMSFSymbolTable* base = RDGateOnlySymbolCollectOperator::pmsfSymbolTable();
    if (!base || base->empty()) {
        return nullptr;
    }
    pmsf::copyPMSFSymbolTableWithRowAugmentation(
        base, paperTableCache_, [](pmsf::PMSFSymbolRecord& row) {
        pmsf::appendPCATMPureBoundary(row.delta);
    });
    return paperTableCache_.empty() ? nullptr : &paperTableCache_;
}

