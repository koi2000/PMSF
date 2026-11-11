/*
 * @file HookRegistry.cpp
 * @brief Phase A/B: hook 注册表实现。
 */
#include "HookRegistry.h"

#include "Halfedge.h"
#include "Point.h"
#include "Vertex.h"

#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <limits>
#include <stdexcept>

namespace pmsf {

// ---------------------------------------------------------------------------
// PluginErrorMetric
// ---------------------------------------------------------------------------

PluginErrorMetric::PluginErrorMetric(PMSFErrorMetricV1* metric) : metric_(metric) {
    if (!metric_ || !metric_->cost) {
        throw std::runtime_error("PluginErrorMetric: null metric or cost function");
    }
}

PluginErrorMetric::~PluginErrorMetric() {
    if (metric_ && metric_->destroy) {
        metric_->destroy(metric_->userdata);
    }
    // metric_ 本身由插件分配；destroy 应负责释放。
    metric_ = nullptr;
}

double PluginErrorMetric::cost(MCGAL::Halfedge* h) const {
    float target[3] = {0.0f, 0.0f, 0.0f};
    return static_cast<double>(metric_->cost(metric_->userdata, h, target));
}

double PluginErrorMetric::costWithTarget(MCGAL::Halfedge* h, MCGAL::Point& target) const {
    float t[3] = {0.0f, 0.0f, 0.0f};
    float c = metric_->cost(metric_->userdata, h, t);
    target = MCGAL::Point(t[0], t[1], t[2]);
    return static_cast<double>(c);
}

void PluginErrorMetric::onAfterEliminate(MCGAL::Halfedge* h) {
    if (metric_ && metric_->on_after_eliminate) {
        metric_->on_after_eliminate(metric_->userdata, h);
    }
}

// ---------------------------------------------------------------------------
// HookRegistry
// ---------------------------------------------------------------------------

HookRegistry::HookRegistry() = default;

HookRegistry::~HookRegistry() {
    builtins_.clear();
    for (auto& kv : openedLibs_) {
        if (kv.second) dlclose(kv.second);
    }
    openedLibs_.clear();
}

void HookRegistry::registerBuiltin(const std::string& name, ErrorMetricFactory factory) {
    builtins_[name] = std::move(factory);
}

bool HookRegistry::hasBuiltin(const std::string& name) const {
    return builtins_.find(name) != builtins_.end();
}

std::shared_ptr<ErrorMetric> HookRegistry::createErrorMetric(const std::string& builtinName,
                                                             const std::string& pluginPath,
                                                             const std::string& factorySymbol) {
    if (!builtinName.empty()) {
        auto it = builtins_.find(builtinName);
        if (it != builtins_.end()) {
            return it->second();
        }
        throw std::runtime_error("builtin hook not registered: " + builtinName);
    }

    if (pluginPath.empty()) {
        throw std::runtime_error("createErrorMetric: either builtin or plugin must be specified");
    }

    // 已加载则复用同一 handle，但每次都创建新的 metric 实例。
    void* handle = nullptr;
    auto it = openedLibs_.find(pluginPath);
    if (it != openedLibs_.end()) {
        handle = it->second;
    } else {
        handle = dlopen(pluginPath.c_str(), RTLD_NOW | RTLD_LOCAL);
        if (!handle) {
            const char* err = dlerror();
            throw std::runtime_error(std::string("dlopen failed for ") + pluginPath +
                                     ": " + (err ? err : "unknown error"));
        }
        openedLibs_[pluginPath] = handle;
    }

    void* sym = dlsym(handle, factorySymbol.c_str());
    const char* dlsymErr = dlerror();
    if (!sym) {
        throw std::runtime_error(std::string("dlsym failed for ") + factorySymbol +
                                 " in " + pluginPath + ": " + (dlsymErr ? dlsymErr : "unknown error"));
    }

    auto factory = reinterpret_cast<pmsf_create_error_metric_v1_t>(sym);
    PMSFErrorMetricV1* metric = factory();
    if (!metric) {
        throw std::runtime_error("plugin factory returned null metric: " + pluginPath);
    }
    return std::make_shared<PluginErrorMetric>(metric);
}

}  // namespace pmsf
