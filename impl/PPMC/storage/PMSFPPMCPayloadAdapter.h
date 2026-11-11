#ifndef PMSF_STORAGE_PPMC_PAPER_PAYLOAD_ADAPTER_H
#define PMSF_STORAGE_PPMC_PAPER_PAYLOAD_ADAPTER_H

#include "storage/PMSFPayloadDescriptorRegistry.h"

#include <cstddef>
#include <cstdint>

namespace pmsf {

inline constexpr const char* kPPMCOperationPayloadName = "ppmc_operation_rc";
inline constexpr const char* kPPMCPOC2PayloadCodecName = "ppmc_poc2_split_zlib";
inline constexpr const char* kPPMCRangeCoderPayloadCodecName = "ppmc_range_coder";

inline const char* ppmcOperationPayloadCodecName(const void* data,
                                                 std::size_t size) {
    if (data != nullptr && size >= 4) {
        const auto* bytes = reinterpret_cast<const char*>(data);
        if (bytes[0] == 'P' && bytes[1] == 'O' &&
            bytes[2] == 'C' && bytes[3] == '2') {
            return kPPMCPOC2PayloadCodecName;
        }
    }
    return kPPMCRangeCoderPayloadCodecName;
}

inline PMSFPayloadDescriptor ppmcOperationPayloadDescriptor(
    const void* data,
    std::size_t size,
    std::int32_t partitionId,
    std::int32_t lodId) {
    return makePMSFPayloadDescriptor(
        PMSFPayloadRole::OperationStream,
        0,
        partitionId,
        lodId,
        kPPMCOperationPayloadName,
        ppmcOperationPayloadCodecName(data, size));
}

inline PMSFSymbolPayloadSpec ppmcOperationPayloadSpec(
    const void* data,
    std::size_t size,
    std::int32_t partitionId,
    std::int32_t lodId) {
    return paperPayloadSpecFromDescriptor(
        ppmcOperationPayloadDescriptor(data, size, partitionId, lodId));
}

inline PMSFSymbolPayloadRecord buildPPMCOperationPayloadRecord(
    const void* data,
    std::size_t size,
    std::int32_t partitionId,
    std::int32_t lodId) {
    return paperPayloadRecordFromDescriptor(
        ppmcOperationPayloadDescriptor(data, size, partitionId, lodId),
        data,
        size);
}

inline void appendPPMCOperationPayload(PMSFSymbolPayloadTable& rows,
                                       const void* data,
                                       std::size_t size,
                                       std::int32_t partitionId,
                                       std::int32_t lodId) {
    appendPMSFPayloadRecordFromDescriptor(
        rows,
        ppmcOperationPayloadDescriptor(data, size, partitionId, lodId),
        data,
        size);
}

}  // namespace pmsf

#endif  // PMSF_STORAGE_PPMC_PAPER_PAYLOAD_ADAPTER_H
