#ifndef PMSF_STORAGE_MMSE_PAPER_PAYLOAD_ADAPTER_H
#define PMSF_STORAGE_MMSE_PAPER_PAYLOAD_ADAPTER_H

#include "storage/PMSFPayloadDescriptorRegistry.h"

#include <cstddef>
#include <cstdint>
#include <utility>

namespace pmsf {

inline constexpr const char* kMMSEDualRingPayloadName = "mmse_dual_ring_blob";
inline constexpr const char* kMMSEDualRingPayloadCodecName =
    "mmse_dual_ring_blob_layout_22";
inline constexpr const char* kMMSEStrictPayloadCodecName =
    "mmse_strict_blob_layout_27";

inline const char* mmsePayloadCodecName(bool strictStorage) {
    return strictStorage ? kMMSEStrictPayloadCodecName
                         : kMMSEDualRingPayloadCodecName;
}

inline PMSFPayloadDescriptor mmsePayloadDescriptor(bool strictStorage) {
    return makePMSFPayloadDescriptor(PMSFPayloadRole::ConnectivityBlob,
                                      kMMSEDualRingPayloadName,
                                      mmsePayloadCodecName(strictStorage));
}

inline PMSFSymbolPayloadSpec mmsePayloadSpec(bool strictStorage) {
    return paperPayloadSpecFromDescriptor(mmsePayloadDescriptor(strictStorage));
}

inline PMSFSymbolPayloadTable buildMMSEPayloadTable(
    const void* data,
    std::size_t size,
    bool strictStorage) {
    PMSFSymbolPayloadTable table;
    appendPMSFPayloadRecordFromDescriptor(
        table, mmsePayloadDescriptor(strictStorage), data, size);
    return table;
}

}  // namespace pmsf

#endif  // PMSF_STORAGE_MMSE_PAPER_PAYLOAD_ADAPTER_H
