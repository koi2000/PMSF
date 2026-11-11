#ifndef PMSF_STORAGE_PAPER_STORAGE_CONTEXT_SIDE_CHANNEL_H
#define PMSF_STORAGE_PAPER_STORAGE_CONTEXT_SIDE_CHANNEL_H

#include "core/PMSFSideChannel.h"
#include "core/Symbol.h"

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace pmsf {

inline constexpr const char* kPMSFStorageContextPayloadOnlySuffix =
    "payload_only";
inline constexpr const char* kPMSFStorageContextSymbolMappingSuffix =
    "symbol_mapping";
inline constexpr const char* kPMSFStorageContextCompatSymbolsSkippedSuffix =
    "compat_symbols_skipped";
inline constexpr const char* kPMSFStorageContextCompatBaseSkippedSuffix =
    "compat_base_skipped";
inline constexpr const char* kPMSFStorageContextBasePayloadPresentSuffix =
    "base_payload_present";
inline constexpr const char* kPMSFStorageContextSymbolsPayloadPresentSuffix =
    "symbols_payload_present";
inline constexpr const char* kPMSFStorageContextCollectorPayloadPresentSuffix =
    "collector_payload_present";
inline constexpr const char* kPMSFStorageContextInjectedPayloadPresentSuffix =
    "injected_payload_present";
inline constexpr const char* kPMSFStorageContextSymbolsPayloadOnlySuffix =
    "symbols_payload_only";
inline constexpr const char* kPMSFStorageContextSymbolsSymbolMappingSuffix =
    "symbols_symbol_mapping";
inline constexpr const char* kPMSFStorageContextCountSuffix = "count";
inline constexpr const char* kPMSFStorageScopeUnifiedBlob =
    "unified_blob";

inline std::string paperStorageContextSideName(const std::string& prefix,
                                               const std::string& suffix)
{
    return prefix.empty() ? suffix : prefix + "_" + suffix;
}

template <typename RoundSymbols>
inline void appendPMSFStorageContextI32(RoundSymbols& delta,
                                         const std::string& prefix,
                                         const std::string& suffix,
                                         std::int32_t value)
{
    appendPaperI32SideChannel(delta,
                              paperStorageContextSideName(prefix, suffix),
                              value);
}

template <typename SideMap>
inline void appendPMSFStorageContextSideMapI32(SideMap& side,
                                                const std::string& prefix,
                                                const std::string& suffix,
                                                std::int32_t value)
{
    appendPaperI32SideMap(side,
                          paperStorageContextSideName(prefix, suffix),
                          value);
}

template <typename RoundSymbols>
inline void appendPMSFStorageContextMapping(RoundSymbols& delta,
                                             const std::string& prefix,
                                             std::int32_t payloadOnly,
                                             std::int32_t symbolMapping)
{
    appendPMSFStorageContextI32(
        delta, prefix, kPMSFStorageContextPayloadOnlySuffix, payloadOnly);
    appendPMSFStorageContextI32(delta,
                                 prefix,
                                 kPMSFStorageContextSymbolMappingSuffix,
                                 symbolMapping);
}

template <typename RoundSymbols>
inline void appendPMSFStorageContextSymbolMapping(RoundSymbols& delta,
                                                   const std::string& prefix,
                                                   std::int32_t symbolMapping)
{
    appendPMSFStorageContextI32(delta,
                                 prefix,
                                 kPMSFStorageContextSymbolMappingSuffix,
                                 symbolMapping);
}

template <typename SideMap>
inline void appendPMSFStorageContextSideMapMapping(
    SideMap& side,
    const std::string& prefix,
    std::int32_t payloadOnly,
    std::int32_t symbolMapping)
{
    appendPMSFStorageContextSideMapI32(side,
                                        prefix,
                                        kPMSFStorageContextPayloadOnlySuffix,
                                        payloadOnly);
    appendPMSFStorageContextSideMapI32(side,
                                        prefix,
                                        kPMSFStorageContextSymbolMappingSuffix,
                                        symbolMapping);
}

template <typename RoundSymbols>
inline void appendPMSFStorageContextMappingAndEntries(
    RoundSymbols& delta,
    const std::string& prefix,
    std::int32_t payloadOnly,
    std::int32_t symbolMapping,
    const std::vector<std::pair<std::string, std::int32_t>>& entries)
{
    appendPMSFStorageContextMapping(delta, prefix, payloadOnly, symbolMapping);
    appendPaperI32SideChannelEntries(delta, entries);
}


template <typename RoundSymbols>
inline void appendParquetStorageAudit(
    RoundSymbols& delta,
    const std::string& prefix,
    std::int32_t compatSymbolsSkipped,
    std::int32_t compatBaseSkipped,
    std::int32_t basePayloadPresent,
    std::int32_t symbolsPayloadPresent,
    std::int32_t collectorPayloadPresent,
    std::int32_t symbolsPayloadOnly,
    std::int32_t symbolsSymbolMapping,
    std::int32_t injectedPayloadPresent = -1)
{
    appendPMSFStorageContextI32(delta,
                                 prefix,
                                 kPMSFStorageContextCompatSymbolsSkippedSuffix,
                                 compatSymbolsSkipped);
    appendPMSFStorageContextI32(delta,
                                 prefix,
                                 kPMSFStorageContextCompatBaseSkippedSuffix,
                                 compatBaseSkipped);
    appendPMSFStorageContextI32(delta,
                                 prefix,
                                 kPMSFStorageContextBasePayloadPresentSuffix,
                                 basePayloadPresent);
    appendPMSFStorageContextI32(delta,
                                 prefix,
                                 kPMSFStorageContextSymbolsPayloadPresentSuffix,
                                 symbolsPayloadPresent);
    appendPMSFStorageContextI32(delta,
                                 prefix,
                                 kPMSFStorageContextCollectorPayloadPresentSuffix,
                                 collectorPayloadPresent);
    if (injectedPayloadPresent >= 0) {
        appendPMSFStorageContextI32(delta,
                                     prefix,
                                     kPMSFStorageContextInjectedPayloadPresentSuffix,
                                     injectedPayloadPresent);
    }
    appendPMSFStorageContextI32(delta,
                                 prefix,
                                 kPMSFStorageContextSymbolsPayloadOnlySuffix,
                                 symbolsPayloadOnly);
    appendPMSFStorageContextI32(delta,
                                 prefix,
                                 kPMSFStorageContextSymbolsSymbolMappingSuffix,
                                 symbolsSymbolMapping);
}


template <typename RoundSymbols>
inline void appendUnifiedBlobStorage(RoundSymbols& delta,
                                       std::int32_t blobCount)
{
    appendPMSFStorageContextMapping(
        delta, kPMSFStorageScopeUnifiedBlob, 1, 0);
    appendPMSFStorageContextI32(delta,
                                 kPMSFStorageScopeUnifiedBlob,
                                 kPMSFStorageContextCountSuffix,
                                 blobCount);
}

} // namespace pmsf

#endif // PMSF_STORAGE_PAPER_STORAGE_CONTEXT_SIDE_CHANNEL_H
