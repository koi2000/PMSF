#ifndef PMSF_CORE_QEM_PAPER_BOUNDARY_H
#define PMSF_CORE_QEM_PAPER_BOUNDARY_H

// R3/L1（PHASE63）：原先两个样板函数（`...Required()` + `...I32()`）已收敛为
// 框架机制 `core/PMSFBoundaryComponent.h`；本文件只留 **env 名参数**。
// 调用点直接用 `pmsf::boundaryFlagOn(<spec>)`。语义逐表达式不变。

#include "core/PMSFBoundaryComponent.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include <cstddef>

namespace pmsf {
namespace qem_boundary {

const char* const kPaperExact[] = {"PMSF_QEM_REQUIRE_PMSF_EXACT"};
constexpr BoundaryFlagSpec kPaperExactSpec{kPaperExact, 1};

}  // namespace qem_boundary
}  // namespace pmsf

#endif  // PMSF_CORE_QEM_PAPER_BOUNDARY_H
