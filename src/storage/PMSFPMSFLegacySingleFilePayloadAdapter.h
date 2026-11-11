#ifndef PMSF_STORAGE_LEGACY_SINGLE_FILE_PAYLOAD_ADAPTER_H
#define PMSF_STORAGE_LEGACY_SINGLE_FILE_PAYLOAD_ADAPTER_H

#include "storage/PMSFPayloadDescriptorRegistry.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace pmsf {

inline constexpr const char* kLegacySingleFileCompatFileSkippedSideName =
    "legacy_singlefile_compat_file_skipped";
inline constexpr const char* kLegacySingleFileBackendSideName =
    "legacy_singlefile_backend";
inline constexpr const char* kLegacySingleFileSidecarPrefix =
    "legacy_singlefile";
inline constexpr const char* kLegacySingleFileHeaderDataColumn =
    "header_data";
inline constexpr const char* kLegacySingleFileBaseMeshDataColumn =
    "base_mesh_data";
inline constexpr const char* kLegacySingleFileSymbolDataColumn =
    "symbol_data";
inline constexpr const char* kLegacySingleFileTotalBinaryDataColumn =
    "total_binary_data";
inline constexpr const char* kLegacySingleFileHeaderSizeColumn =
    "header_size";
inline constexpr const char* kLegacySingleFileBaseMeshSizeColumn =
    "base_mesh_size";
inline constexpr const char* kLegacySingleFileSymbolDataSizeColumn =
    "symbol_data_size";

inline bool legacySingleFileSkipCompatFile() {
    return unifiedStorageEnabled() &&
           pmsfStorageEnvEnabled("PMSF_LEGACY_SINGLEFILE_SKIP_COMPAT_FILE");
}

inline PMSFPayloadDescriptor legacySingleFileBinaryPayloadDescriptor() {
    return makePMSFPayloadDescriptor(
        PMSFPayloadRole::LegacyBinary,
        kLegacySingleFileBinaryPayloadName,
        kLegacySingleFileBinaryPayloadCodecName);
}

inline PMSFSymbolPayloadSpec legacySingleFileBinaryPayloadSpec() {
    return paperPayloadSpecFromDescriptor(
        legacySingleFileBinaryPayloadDescriptor());
}

inline std::vector<PaperNamedBlob> legacySingleFilePayloadBlob(
    const void* data,
    std::size_t size) {
    return {paperNamedBlobFromDescriptor(
        legacySingleFileBinaryPayloadDescriptor(), data, size)};
}

inline std::vector<std::pair<std::string, std::int32_t>>
legacySingleFileFallbackSideValues(std::int32_t backendId) {
    return {{kLegacySingleFileCompatFileSkippedSideName, 1},
            {kLegacySingleFileBackendSideName, backendId}};
}

}  // namespace pmsf

#endif  // PMSF_STORAGE_LEGACY_SINGLE_FILE_PAYLOAD_ADAPTER_H
