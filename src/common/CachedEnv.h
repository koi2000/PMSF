#pragma once
// 热路径 env 缓存（perf 2026-11-10 QEM 解码实测）：诊断开关 getenv 在解码热循环
// 被调用 61 万次（__strncmp_avx2 17.5% + getenv 3.9% cycles）。env 在启动前设置、
// 运行期不变 ⇒ 每个展开点用独立 static 缓存首次查到的指针，语义与直接 getenv 相同
// （返回同一 char*）。不要用于「运行期 setenv 后需重新读取」的逻辑。
#include <cstdlib>

#define PMSF_CACHED_ENV(name) \
    ([]() -> const char* { static const char* pmsf_cached_env_v = std::getenv(name); return pmsf_cached_env_v; }())