#ifndef PMSF_CORE_PMSF_BOUNDARY_COMPONENT_H
#define PMSF_CORE_PMSF_BOUNDARY_COMPONENT_H

// PMSFBoundaryComponent.h —— R3/L1：**参数化的边界组件**。
//
// 背景（PHASE63 附 54）：六个算法各有一份 `impl/<ALGO>/boundary/*PMSFBoundary.h`，
// 合计 986 有效行（PCATM 338 / PCLTTM 274 / PPMC 163 / APTM 91 / MMSE 91 /
// VQMMC 29）。逐份读下来，内容几乎全是**三种机械模式**：
//
//   C) 环境开关：`bool xRequired() { return envOn(A) || envOn(B); }`
//      外加一个 `...I32()` 变体（`? 1 : 0`）；
//   A) 侧信道替换：`replaceXSideMap` / `replaceXSymbols` 两两成对，
//      **只差**调 `replacePaperI32SideMap` 还是 `replacePaperI32SideChannel`；
//   B) 侧信道追加：一串 `appendPaperI32SideChannel(delta, prefix, name, value)`。
//
// 本组件把这三件事的**机制**收到框架里；算法侧只留**参数**（env 名字符串、
// (prefix, name, value) 三元组）。
//
// ⚠️ 为什么参数必须留在 `impl/<ALGO>/`：`src/` 有「禁算法名」的 lint 纪律。
//    算法专属的 env 名/字段名进不了这里 —— 这是本组件只收机制、不收数据的原因。
//
// ⚠️ 语义等价是**逐表达式**保持的，不是「看起来一样」：
//    - `boundaryFlagOn` 按**数组顺序**短路求值，与原 `A() || B()` 的求值顺序一致
//      （`pmsfStorageEnvEnabled` 无副作用，但顺序仍照抄，避免日后加副作用时踩雷）；
//    - `boundaryFlagI32` 就是 `? 1 : 0`。

#include "core/PMSFBoundarySideChannel.h"
#include "core/PMSFSideChannel.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include <cstddef>
#include <cstdint>

namespace pmsf {

/// 一族「任一环境变量为真即成立」的边界开关。
/// @note `envNames` 指向的字符串数组必须**静态存储期**（字面量即可）。
struct BoundaryFlagSpec {
    const char* const* envNames;
    std::size_t count;
};

/// 与 `envOn(a) || envOn(b) || ...` 等价；按 `envNames` 顺序短路。
inline bool boundaryFlagOn(const BoundaryFlagSpec& spec)
{
    for (std::size_t i = 0; i < spec.count; ++i) {
        if (pmsfStorageEnvEnabled(spec.envNames[i])) {
            return true;
        }
    }
    return false;
}

/// 与 `xRequired() ? 1 : 0` 等价。
inline std::int32_t boundaryFlagI32(const BoundaryFlagSpec& spec)
{
    return boundaryFlagOn(spec) ? 1 : 0;
}

/// A 模式：一对 `replaceXSideMap` / `replaceXSymbols` 的**共同机制**。
/// `SideChannel=true` 走 `replacePaperI32SideChannel`，否则走 `replacePaperI32SideMap`；
/// 两者的参数次序、prefix/name 传递方式与原实现逐一对齐。
template <bool SideChannel, typename Target, typename Values>
inline void replaceBoundaryField(Target& target,
                                 const char* prefix,
                                 const char* name,
                                 const Values& values)
{
    if (SideChannel) {
        replacePaperI32SideChannel(target, prefix, name, values);
    } else {
        replacePaperI32SideMap(target, prefix, name, values);
    }
}

/// B 模式：一串 `appendPaperI32SideChannel(delta, prefix, name, value)`。
/// 用法：`appendBoundaryFields(delta, "pcatm_pure", {{"topology_mode", 2}, ...});`
struct BoundaryI32Field {
    const char* name;
    std::int32_t value;
};

template <typename PaperRoundSymbols>
inline void appendBoundaryFields(PaperRoundSymbols& delta,
                                 const char* prefix,
                                 const BoundaryI32Field* fields,
                                 std::size_t count)
{
    for (std::size_t i = 0; i < count; ++i) {
        appendPaperI32SideChannel(delta, prefix, fields[i].name, fields[i].value);
    }
}

/// B 模式里 `appendPMSFBoundaryAbsentExactPair` 那一半。

}  // namespace pmsf

#endif  // PMSF_CORE_PMSF_BOUNDARY_COMPONENT_H
