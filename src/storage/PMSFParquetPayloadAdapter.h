#ifndef PMSF_STORAGE_PARQUET_PAPER_PAYLOAD_ADAPTER_H
#define PMSF_STORAGE_PARQUET_PAPER_PAYLOAD_ADAPTER_H

#include "storage/PMSFPayloadDescriptorRegistry.h"

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace pmsf {

inline constexpr const char* kParquetSymbolsRCPayloadName =
    "pmsf_symbols_rc_blob";

inline std::string parquetSymbolsRCPayloadCodecName(int blobLayout) {
    return "columnar_blob_layout_" + std::to_string(blobLayout);
}

inline PMSFPayloadDescriptor parquetSymbolsRCPayloadDescriptor(
    int blobLayout) {
    return makePMSFPayloadDescriptor(
        PMSFPayloadRole::SymbolStreamBlob,
        kParquetSymbolsRCPayloadName,
        parquetSymbolsRCPayloadCodecName(blobLayout));
}

inline PMSFSymbolPayloadSpec parquetSymbolsRCPayloadSpec(int blobLayout) {
    return paperPayloadSpecFromDescriptor(
        parquetSymbolsRCPayloadDescriptor(blobLayout));
}

inline PMSFPayloadDescriptor parquetCompactBasePayloadDescriptor() {
    return makePMSFPayloadDescriptor(
        PMSFPayloadRole::CompactBase,
        kColumnarCompactBasePayloadName,
        kColumnarCompactBasePayloadCodecName);
}

inline PMSFSymbolPayloadSpec parquetCompactBasePayloadSpec() {
    return paperPayloadSpecFromDescriptor(
        parquetCompactBasePayloadDescriptor());
}

inline bool parquetSkipCompatSymbolsTableRequested() {
    return unifiedStorageEnabled() &&
           (pmsfStorageEnvEnabled("PMSF_PARQUET_SKIP_COMPAT_SYMBOLS_TABLE") ||
            pmsfStorageEnvEnabled("PMSF_COLUMNAR_SKIP_COMPAT_SYMBOLS_TABLE"));
}

inline bool parquetSkipCompatBaseTablesRequested() {
    return unifiedStorageEnabled() &&
           (pmsfStorageEnvEnabled("PMSF_PARQUET_SKIP_COMPAT_BASE_TABLES") ||
            pmsfStorageEnvEnabled("PMSF_COLUMNAR_SKIP_COMPAT_BASE_TABLES"));
}



inline void appendParquetSymbolsRCPayload(PMSFSymbolPayloadTable& rows,
                                          std::vector<std::uint8_t> payload,
                                          int blobLayout) {
    appendPMSFPayloadRecordFromDescriptor(
        rows, parquetSymbolsRCPayloadDescriptor(blobLayout), std::move(payload));
}

inline void appendParquetCompactBasePayload(PMSFSymbolPayloadTable& rows,
                                            std::vector<std::uint8_t> payload) {
    appendPMSFPayloadRecordFromDescriptor(
        rows, parquetCompactBasePayloadDescriptor(), std::move(payload));
}

}  // namespace pmsf

#endif  // PMSF_STORAGE_PARQUET_PAPER_PAYLOAD_ADAPTER_H
