#ifndef PMSF_CORE_VQMMC_PAPER_BOUNDARY_H
#define PMSF_CORE_VQMMC_PAPER_BOUNDARY_H

// R3/L1（PHASE63）：本文件原先有 5 个样板函数
//（`...EnvOn` 转发 + 4 个 `...Required()/...I32()`），合计 41 有效行，
// 内容全是 `envOn(A) || envOn(B)` 与 `? 1 : 0`。
// **机制**已收进 `core/PMSFBoundaryComponent.h`，本文件按 「算法侧只留参数声明」
// 只保留 **env 名参数**；调用点直接用 `pmsf::boundaryFlagOn(<spec>)`。
//
// 求值顺序（数组序短路）与 `? 1 : 0` 语义逐表达式保持 ⇒ 产物逐字节不变。

#include "core/PMSFBoundaryComponent.h"

#include <cstddef>

namespace pmsf {
namespace vqmmc_boundary {

const char* const kPaperExact[] = {
    "PMSF_VQMMC_REQUIRE_PAPER_EXACT",
    "VQMMC_REQUIRE_PAPER_EXACT",
};
const char* const kPaperConnectivity[] = {
    "PMSF_VQMMC_REQUIRE_PAPER_CONNECTIVITY",
    "VQMMC_REQUIRE_PAPER_CONN",
};

constexpr BoundaryFlagSpec kPaperExactSpec{kPaperExact, 2};
constexpr BoundaryFlagSpec kPaperConnectivitySpec{kPaperConnectivity, 2};

}  // namespace vqmmc_boundary
}  // namespace pmsf

#endif  // PMSF_CORE_VQMMC_PAPER_BOUNDARY_H
