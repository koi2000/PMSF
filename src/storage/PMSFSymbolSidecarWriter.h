/*
 * @file PMSFSymbolSidecarWriter.h
 * @brief Small helpers for opt-in unified paper-symbol sidecar output.
 *
 * These helpers intentionally do nothing unless PMSF_UNIFIED_STORAGE=1.
 * Default Parquet/Binary outputs therefore keep their existing byte size and
 * timing profile while migration builds can emit the paper-aligned relation
 * beside legacy payloads.
 */
#ifndef PMSF_STORAGE_PAPER_SYMBOL_SIDECAR_WRITER_H
#define PMSF_STORAGE_PAPER_SYMBOL_SIDECAR_WRITER_H

#include "compress/UnifiedSerializeOperator.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include <string>

namespace pmsf {

inline bool writePMSFSymbolSidecar(const PMSFSymbolTable& table,
                                    const std::string& path) {
    if (!unifiedStorageEnabled() || table.empty()) return false;
    UnifiedSerializeOperator writer;
    // emitSidecar=false：侧车不再为自己写侧车。默认 true 会在本函数与
    // UnifiedSerializeOperator::serialize 之间形成无界互相递归（D5）。
    writer.serialize(table, path, /*emitSidecar=*/false);
    return true;
}

inline std::string paperSymbolSidecarPath(const std::string& baseStem) {
    return paperSideChannelTablePath(baseStem);
}

}  // namespace pmsf

#endif  // PMSF_STORAGE_PAPER_SYMBOL_SIDECAR_WRITER_H
