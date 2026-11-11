#ifndef PMSF_VQMMC_QUANTIZATION_H
#define PMSF_VQMMC_QUANTIZATION_H

#include <algorithm>
#include <charconv>
#include <cstdlib>
#include <stdexcept>
#include <string_view>

namespace pmsf {

inline constexpr int kVQMMCPayloadCodebookQuantBits = 12;

// Codebook coordinate precision, NOT the mesh's global quantization q.
// Preserve the historical integer clamp, but reject malformed values instead
// of silently turning a typo into a two-bit (lower quality) codebook.
inline int vqmmcConfiguredCodebookQuantBits() {
    const char* env = std::getenv("VQMMC_CODEBOOK_QUANT_BITS");
    if (!env || !*env) return kVQMMCPayloadCodebookQuantBits;
    const std::string_view text(env);
    int bits = 0;
    const auto result = std::from_chars(text.data(), text.data() + text.size(), bits);
    if (result.ec != std::errc{} || result.ptr != text.data() + text.size()) {
        throw std::invalid_argument("VQMMC_CODEBOOK_QUANT_BITS must be an integer");
    }
    return std::clamp(bits, 2, 16);
}

// The signed codebook size is part of the legacy VQMMC wire protocol:
// positive means predictive residuals, negative means absolute coordinates.
// Zero remains the valid empty-round marker. Always validate before taking
// the magnitude so malformed streams cannot reach reserve/loop operations.
inline int vqmmcCodebookCount(int signedSize) {
    constexpr int maxCount = 1 << 16;  // uint16 indices address 0..65535
    if (signedSize < -maxCount || signedSize > maxCount) {
        throw std::runtime_error(
            "VQMMC codebook size exceeds uint16 index capacity");
    }
    return signedSize < 0 ? -signedSize : signedSize;
}

// Only the encoder reads these environment settings. Decoders derive the
// geometry mode from the signed codebook-size marker in the stream.
inline bool vqmmcConfiguredAbsoluteGeometry() {
    const char* absolute = std::getenv("VQMMC_ABS_GEOM");
    if (absolute && *absolute && std::string_view(absolute) != "0") {
        return true;
    }
    const char* predictive = std::getenv("VQMMC_USE_PREDICTIVE_RESIDUAL");
    return predictive && std::string_view(predictive) == "0";
}

inline int vqmmcSignedCodebookSize(int count, bool absoluteGeometry) {
    if (count < 0) {
        throw std::runtime_error("VQMMC negative codebook count");
    }
    (void)vqmmcCodebookCount(count);
    return absoluteGeometry ? -count : count;
}

}  // namespace pmsf
#endif
