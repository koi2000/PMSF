/*
 * @file PMSFStorageScopeNames.h
 * @brief 各算法**专有**的 storage scope 名（作为 side-channel key 前缀持久化）。
 *
 * ## 为什么住在这里
 *
 * `kPMSFStorageScopePomarLegacy` 原先长在 `src/storage/PMSFStorageContextSideChannel.h`
 * 里，与 `kPMSFStorageScopeUnifiedBlob` 这类**中性**常量混在一起。
 * 但它是某个算法的专有 scope 名，`src/` 不该认识具体算法
 * （见 scripts/tests/lint_src_no_algo_names.sh）。
 *
 * 勘察结果：该常量**只被 impl/ 使用**（5 处，全在 impl/），
 * 其中一处还是行为分支（`PMSFLegacyOperationSymbolAdapter.h` 里比较 prefix）。
 * 分支与消费者都在 impl/，所以这一步是**纯位置移动**，不涉及改名掩盖。
 *
 * ## 值的稳定性
 *
 * 字符串值 `"pomar_legacy"` 会作为 side-channel key 前缀**持久化进产物**，
 * 因此**值不可改**；外迁的只是 C++ 侧的声明位置。
 */
#pragma once

namespace pmsf {

/// 该算法历史落盘格式的 storage scope 名（side-channel key 前缀，已持久化）。
inline constexpr const char* kPMSFStorageScopePomarLegacy = "pomar_legacy";

}  // namespace pmsf
