/*
 * @file RDSymbolCollectOperator.cpp
 * @brief Phase A 实现：完全复用 ValenceSymbolCollectOperator 的 BFS + RC 导出。
 *        见 .h 顶部注释对 "为什么继承 Valence 而非独立 BFS" 的说明。
 */
#include "RD/compress/RDSymbolCollectOperator.h"
#include "core/PMSFBoundarySideChannel.h"
#include "core/PMSFSideChannel.h"
#include "RD/boundary/RDPMSFBoundary.h"
#include "Facet.h"
#include "Halfedge.h"

#include <array>
#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void requireRDPhaseAPaperExactIfRequested(const char* scope) {
    if (!pmsf::rdPaperExactRequired()) return;
    throw std::runtime_error(
        std::string(scope) +
        " PMSF_RD_REQUIRE_* exact mode requested, but RDSymbolCollectOperator "
        "Phase A is a Valence-compatible transport stream, not the paper RD "
        "gate-only / curvature-predictive / KLT stream.");
}

}  // namespace

// ----------------------------------------------------------------------------
// Phase A：兼容 probe 旧接口的 static helpers（供 diag 工具使用，不参与码流）。
// ----------------------------------------------------------------------------

RDValenceCode RDSymbolCollectOperator::encodeValence(int valence) {
    switch (valence) {
        case 3: return RDValenceCode::VAL_3;
        case 4: return RDValenceCode::VAL_4;
        case 5: return RDValenceCode::VAL_5;
        case 6: return RDValenceCode::VAL_6;
        default: return RDValenceCode::VAL_OTHER;
    }
}

int RDSymbolCollectOperator::decodeValence(RDValenceCode c) {
    switch (c) {
        case RDValenceCode::VAL_3: return 3;
        case RDValenceCode::VAL_4: return 4;
        case RDValenceCode::VAL_5: return 5;
        case RDValenceCode::VAL_6: return 6;
        default: return 7;
    }
}

int RDSymbolCollectOperator::exportToBufferWithRangeCoder(
    char* buffer, bool enableQuantization) {
    if (!buffer) {
        return ValenceSymbolCollectOperator::exportToBufferWithRangeCoder(
            buffer, enableQuantization);
    }
    requireRDPhaseAPaperExactIfRequested("[RDSymbolCollect]");
    return ValenceSymbolCollectOperator::exportToBufferWithRangeCoder(
        buffer, enableQuantization);
}

const pmsf::PMSFSymbolTable* RDSymbolCollectOperator::pmsfSymbolTable() const {
    const pmsf::PMSFSymbolTable* base =
        ValenceSymbolCollectOperator::pmsfSymbolTable();
    if (!base || base->empty()) {
        return nullptr;
    }
    pmsf::copyPMSFSymbolTableWithRowAugmentation(
        base, paperTableCache_, [](pmsf::PMSFSymbolRecord& row) {
        pmsf::appendRDPhaseAStorageSideChannel(row.delta);
    });
    return paperTableCache_.empty() ? nullptr : &paperTableCache_;
}
