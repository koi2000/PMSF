/*
 * @file HookRegistry.h
 * @brief Phase A/B: 运行时 hook 注册表。
 *
 * 支持两种来源：
 *   1) 内置 hook：通过 registerBuiltin 注册名称→工厂的映射。
 *   2) 外部 .so 插件：通过 dlopen/dlsym 加载 C-ABI 工厂函数。
 *
 * 返回的 MetricHandle 是 RAII 包装，析构时自动释放底层资源。
 */
#pragma once

#include "PMSFHookAPI.h"
#include "metric/ErrorMetric.h"

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

namespace pmsf {

/// @brief 对 PMSFErrorMetricV1 的 RAII 包装；同时实现 pmsf::ErrorMetric 接口。
class PluginErrorMetric : public ErrorMetric {
  public:
    explicit PluginErrorMetric(PMSFErrorMetricV1* metric);
    ~PluginErrorMetric() override;

    PluginErrorMetric(const PluginErrorMetric&) = delete;
    PluginErrorMetric& operator=(const PluginErrorMetric&) = delete;

    /// @brief ErrorMetric 接口：不带目标点的代价（内部用中点填充 target）。
    double cost(MCGAL::Halfedge* h) const override;

    /// @brief QEM 风格：返回代价并写出折叠目标位置。
    double costWithTarget(MCGAL::Halfedge* h, MCGAL::Point& target) const;

    /// @brief 成功 eliminate 后刷新回调。
    void onAfterEliminate(MCGAL::Halfedge* h) override;

  private:
    PMSFErrorMetricV1* metric_ = nullptr;
};

using ErrorMetricFactory = std::function<std::shared_ptr<ErrorMetric>()>;

class HookRegistry {
  public:
    HookRegistry();
    ~HookRegistry();

    HookRegistry(const HookRegistry&) = delete;
    HookRegistry& operator=(const HookRegistry&) = delete;

    /// @brief 注册一个内置 hook 名称（空实现即可，未来 curated hook 使用）。
    void registerBuiltin(const std::string& name, ErrorMetricFactory factory);

    /// @brief 根据配置创建误差度量：优先 builtin，其次 plugin .so。
    /// @throws std::runtime_error 当指定插件无法加载或工厂不存在时。
    std::shared_ptr<ErrorMetric> createErrorMetric(const std::string& builtinName,
                                                   const std::string& pluginPath,
                                                   const std::string& factorySymbol);

    /// @brief 查询是否已注册某内置 hook。
    bool hasBuiltin(const std::string& name) const;

  private:
    std::unordered_map<std::string, ErrorMetricFactory> builtins_;
    // 保存已 dlopen 的 handle，在 HookRegistry 析构时统一 dlclose。
    std::unordered_map<std::string, void*> openedLibs_;
};

}  // namespace pmsf
