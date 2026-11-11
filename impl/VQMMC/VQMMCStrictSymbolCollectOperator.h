/*
 * VQMMCStrictSymbolCollectOperator
 * ---------------------------------
 * Phase 1: Range-Coder-based VQMMC collector — replaces raw-bit+zlib with
 * adaptive range coder across 4 streams per round.
 *
 * Inherits VQMMCSymbolCollectOperator's BFS traversal + data queues.
 * Overrides only exportToBuffer() to produce BlobLayout=30.
 *
 * Stream layout (per round, reverse order):
 *   Stream 0: facetSym (isPolyBit)  — 2-ary adaptive RC
 *   Stream 1: edgeSym  (isAdded)    — 2-ary adaptive RC
 *   Stream 2: vqIndex               — per-bit-position 2-ary RC
 *   Stream 3: codebook               — delta-encoded coordinates + RC
 *
 * Container: ConnectivityBlobBuilder v3 (magic='CONN')
 * BlobLayout: 30 (VQMMC_STRICT_PAPER)
 */
#pragma once

#include "VQMMCSymbolCollectOperator.h"
#include <memory>

class VQMMCPostProcessor;

constexpr int kVQMMCStrictBlobLayout = 30;

class VQMMCStrictSymbolCollectOperator : public VQMMCSymbolCollectOperator {
  public:
    explicit VQMMCStrictSymbolCollectOperator(
        std::shared_ptr<VQMMCPostProcessor> postProcessor)
        : VQMMCSymbolCollectOperator(postProcessor) {}

    const pmsf::PMSFSymbolTable* pmsfSymbolTable() const override;

    int exportToBuffer(char* buffer, bool enableQuantization = false,
                       bool enableCompress = false) override;
};
