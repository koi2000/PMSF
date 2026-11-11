#ifndef PMSF_RESIDUAL_AXIS_CONTEXT_H
#define PMSF_RESIDUAL_AXIS_CONTEXT_H

#include <cstdint>
#include <cstdlib>

namespace pmsf {

// Coarse bucket for residual-axis conditional bitplane contexts. Must stay
// bit-identical between encoder and decoder; the value is derived from a
// fully decoded earlier axis, so no side information is needed.
constexpr int kResidualAxisBucketCount = 13;

inline int residualAxisBucket(std::int32_t v) {
    const std::int32_t a = v < 0 ? -v : v;
    if (v == 0) return 0;
    if (a == 1) return v > 0 ? 1 : 2;
    if (a <= 3) return v > 0 ? 3 : 4;
    if (a <= 7) return v > 0 ? 5 : 6;
    if (a <= 15) return v > 0 ? 7 : 8;
    if (a <= 31) return v > 0 ? 9 : 10;
    return v > 0 ? 11 : 12;
}

inline bool residualAxisContextEnabled() {
    const char* env = std::getenv("PMSF_MMSE_RESID_AXIS_CTX");
    return env && *env && env[0] != '0';
}

}  // namespace pmsf

#endif  // PMSF_RESIDUAL_AXIS_CONTEXT_H
