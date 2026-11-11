/*
 * @file UnifiedSerializeOperator.h
 * @brief Minimal Parquet V3 round-trip serializer.
 *
 * Phase 11 additive design goal: prove end-to-end round trip of (a) the
 * unified RoundSymbols container from src/core/Symbol.h, and (b) dynamic
 * side-channel columns declared at build time.
 *
 * Scope is deliberately small and legacy/prototype-only:
 *   - Writes a single Parquet file (V3 multi-table layout is documented
 *     in UnifiedMultiTableSerializeOperator.h and is the active
 *     algorithm migration target).
 *   - Persists core channels (F/E/V/P/VP) plus every registered side
 *     channel.
 *   - Round-trip verified via tests/test_unified.cpp.
 *
 * The old 5-file schema notes in UnifiedSchema.h are retained for tests
 * and schema-descriptor context only. Running algorithm backends should use
 * UnifiedMultiTableSerializeOperator plus unified paper payload blobs,
 * not introduce new per-algorithm codebook or blob schemas.
 */
#ifndef PMSF_COMPRESS_UNIFIED_SERIALIZE_OPERATOR_H
#define PMSF_COMPRESS_UNIFIED_SERIALIZE_OPERATOR_H

#include "UnifiedDeserializeOperator.h"
#include "UnifiedSchema.h"
#include "core/Symbol.h"

#include <string>
#include <utility>

namespace pmsf {

class UnifiedSerializeOperator {
public:
    UnifiedSerializeOperator() = default;

    /// Register a side channel before serialize(). Must match the channels
    /// actually populated on every RoundSymbols.
    void registerSideChannel(SideChannelSpec spec) {
        appendPMSFSideChannelSpec(sideChannels_, std::move(spec));
    }

    /// Write the given stream (core channels + side channels) to `path`.
    /// F/E/V core symbols are persisted as int32 alphabet values; the
    /// legacy bool column is retained only for old readers.
    /// The file is a single Parquet file containing one row group per
    /// round; within a round each row is one symbol index in the stream.
    void serialize(const SymbolStream& stream, const std::string& path);

    /// 写符号表 Parquet。`emitSidecar=true` 时同时写 `<path>_sidechannel.parquet`。
    ///
    /// 写侧车的那条路**必须**传 false：侧车自身再写侧车会无限互相递归
    /// （path 每层追加一次后缀，直到 ENAMETOOLONG）—— 这正是
    /// PMSF_UNIFIED_STORAGE=1 此前对**所有算法**都失败的原因（D5）。
    void serialize(const PMSFSymbolTable& table, const std::string& path,
                   bool emitSidecar = true);

    const std::vector<SideChannelSpec>& sideChannels() const { return sideChannels_; }

private:
    std::vector<SideChannelSpec> sideChannels_;
    bool pmsfSymbolTableMode_ = false;
    bool pmsfSymbolMappingMode_ = false;
};

}  // namespace pmsf

#endif  // PMSF_COMPRESS_UNIFIED_SERIALIZE_OPERATOR_H
