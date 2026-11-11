#pragma once

#include <chrono>
#include <functional>
#include <string>

// 轻量级的耗时统计器，支持自定义日志输出方式
class TimeRecorder {
  public:
    using LogFunc = std::function<void(const std::string&)>;

    // label 描述当前计时代码块，可选自定义日志函数
    explicit TimeRecorder(std::string label, LogFunc logger = {});
    ~TimeRecorder();

    // 返回距离上次 reset/构造 的耗时，单位 ms
    double elapsedMs() const;
    // 打印 checkpoint 并重置起始时间，返回本次耗时
    double checkpoint(const std::string& checkpointLabel = "");
    // 重置计时，可选更新 label
    void reset(const std::string& newLabel = "");

    static void setDefaultLogger(LogFunc logger);
    static LogFunc getDefaultLogger();

  private:
    std::string label_;
    std::chrono::steady_clock::time_point start_;
    LogFunc logger_;

    void log(const std::string& message) const;
};

// 便捷宏：自动为当前作用域创建计时器
#define SCOPE_TIMER(label) TimeRecorder scoped_timer_##__COUNTER__(label)
#define SCOPE_TIMER_WITH_LOG(label, logger) TimeRecorder scoped_timer_##__COUNTER__(label, logger)

