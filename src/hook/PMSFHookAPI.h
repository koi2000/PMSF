/*
 * @file PMSFHookAPI.h
 * @brief Phase A/B: 运行时 hook 插件 C-ABI 定义。
 *
 * 外部共享库通过暴露 C 工厂函数向框架注册自定义误差度量；框架通过 dlopen/dlsym
 * 加载并包装成 pmsf::ErrorMetric 使用。
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/// @brief V1 误差度量 hook 接口。
///
/// 插件在堆上分配并返回一个本结构体实例；框架通过函数指针调用，并在释放时
/// 调 destroy。userdata 由插件自行解释（通常指向插件内部状态）。
struct PMSFErrorMetricV1 {
    void* userdata;

    /// @brief 计算半边折叠误差；targetPoint[3] 输出折叠目标位置（可选）。
    float (*cost)(void* userdata, void* halfedge, float* targetPoint /*3 floats*/);

    /// @brief 每次成功 eliminate 后回调；halfedge 为幸存顶点上的代表半边。
    void (*on_after_eliminate)(void* userdata, void* halfedge);

    /// @brief 释放 userdata 与本结构体。
    void (*destroy)(void* userdata);
};

/// @brief 插件工厂函数签名。返回值所有权转移给调用方。
///
/// 典型实现：
///   extern "C" PMSFErrorMetricV1* pmsf_create_error_metric_v1() { ... }
typedef struct PMSFErrorMetricV1* (*pmsf_create_error_metric_v1_t)();

#ifdef __cplusplus
}  // extern "C"
#endif
