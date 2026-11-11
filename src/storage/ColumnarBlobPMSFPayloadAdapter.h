#ifndef PMSF_STORAGE_COLUMNAR_BLOB_PAPER_PAYLOAD_ADAPTER_H
#define PMSF_STORAGE_COLUMNAR_BLOB_PAPER_PAYLOAD_ADAPTER_H

#include "storage/PMSFPayloadDescriptorRegistry.h"

#include <cstddef>
#include <cstdint>
#include <string>

namespace pmsf {

inline constexpr const char* kColumnarBlobPayloadCodecSuffix = ".v2blob";
inline constexpr const char* kColumnarBlobVariantColumn = "variant";

inline std::string parquetV2BlobPayloadName(const std::string& variant) {
    return variant + "_payload";
}

inline std::string parquetV2BlobPayloadCodecName(const std::string& variant) {
    return variant + kColumnarBlobPayloadCodecSuffix;
}

inline std::string parquetV2BlobSidecarPrefix(const std::string& variant) {
    return variant + "_v2blob";
}

inline PMSFPayloadDescriptor parquetV2BlobPayloadDescriptor(
    const std::string& variant,
    std::int32_t objectId = 0,
    std::int32_t partitionId = 0,
    std::int32_t lodId = 0) {
    return makePMSFPayloadDescriptor(PMSFPayloadRole::AlgorithmPayload,
                                      objectId,
                                      partitionId,
                                      lodId,
                                      parquetV2BlobPayloadName(variant),
                                      parquetV2BlobPayloadCodecName(variant));
}

inline PMSFSymbolPayloadSpec parquetV2BlobPayloadSpec(
    const std::string& variant,
    std::int32_t objectId = 0,
    std::int32_t partitionId = 0,
    std::int32_t lodId = 0) {
    return paperPayloadSpecFromDescriptor(
        parquetV2BlobPayloadDescriptor(variant, objectId, partitionId, lodId));
}

inline PMSFSymbolPayloadRecord buildColumnarBlobPayloadRecord(
    const std::string& variant,
    const void* data,
    std::size_t size,
    std::int32_t objectId = 0,
    std::int32_t partitionId = 0,
    std::int32_t lodId = 0) {
    return paperPayloadRecordFromDescriptor(
        parquetV2BlobPayloadDescriptor(variant, objectId, partitionId, lodId),
        data,
        size);
}

inline void appendColumnarBlobPayload(PMSFSymbolPayloadTable& rows,
                                       const std::string& variant,
                                       const void* data,
                                       std::size_t size,
                                       std::int32_t objectId = 0,
                                       std::int32_t partitionId = 0,
                                       std::int32_t lodId = 0) {
    appendPMSFPayloadRecordFromDescriptor(
        rows,
        parquetV2BlobPayloadDescriptor(variant, objectId, partitionId, lodId),
        data,
        size);
}

inline std::string parquetV2BlobVariantFromPayloadCodec(
    const std::string& codecName) {
    const std::string suffix(kColumnarBlobPayloadCodecSuffix);
    if (codecName.size() > suffix.size() &&
        codecName.compare(codecName.size() - suffix.size(),
                          suffix.size(),
                          suffix) == 0) {
        return codecName.substr(0, codecName.size() - suffix.size());
    }
    return codecName;
}

}  // namespace pmsf

#endif  // PMSF_STORAGE_COLUMNAR_BLOB_PAPER_PAYLOAD_ADAPTER_H
