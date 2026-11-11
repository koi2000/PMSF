#include "TimeRecorder.h"

#include "Logutil.h"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>

namespace {
std::mutex g_logger_mutex;
TimeRecorder::LogFunc g_default_logger = [](const std::string& msg) {
    std::lock_guard<std::mutex> lock(g_logger_mutex);
    std::cerr << msg << std::endl;
};

std::string makePrefix() {
    std::ostringstream oss;
    oss << "[" << time_string() << "]"
        << "[tid " << std::this_thread::get_id() << "]";
    return oss.str();
}
}  // namespace

TimeRecorder::TimeRecorder(std::string label, LogFunc logger)
    : label_(std::move(label)), start_(std::chrono::steady_clock::now()) {
    if (logger) {
        logger_ = std::move(logger);
    } else {
        logger_ = getDefaultLogger();
    }
    log(makePrefix() + " start " + label_);
}

TimeRecorder::~TimeRecorder() {
    double ms = elapsedMs();
    std::ostringstream oss;
    oss << makePrefix() << " " << label_ << " cost " << std::fixed << std::setprecision(3) << ms << " ms";
    log(oss.str());
}

double TimeRecorder::elapsedMs() const {
    const auto now = std::chrono::steady_clock::now();
    const auto diff = std::chrono::duration_cast<std::chrono::microseconds>(now - start_);
    return diff.count() / 1000.0;
}

double TimeRecorder::checkpoint(const std::string& checkpointLabel) {
    double ms = elapsedMs();
    std::ostringstream oss;
    oss << makePrefix() << " " << label_;
    if (!checkpointLabel.empty()) {
        oss << " [" << checkpointLabel << "]";
    }
    oss << " checkpoint " << std::fixed << std::setprecision(3) << ms << " ms";
    log(oss.str());
    start_ = std::chrono::steady_clock::now();
    return ms;
}

void TimeRecorder::reset(const std::string& newLabel) {
    if (!newLabel.empty()) {
        label_ = newLabel;
    }
    start_ = std::chrono::steady_clock::now();
}

void TimeRecorder::setDefaultLogger(LogFunc logger) {
    if (!logger) {
        return;
    }
    std::lock_guard<std::mutex> lock(g_logger_mutex);
    g_default_logger = std::move(logger);
}

TimeRecorder::LogFunc TimeRecorder::getDefaultLogger() {
    std::lock_guard<std::mutex> lock(g_logger_mutex);
    return g_default_logger;
}

void TimeRecorder::log(const std::string& message) const {
    logger_(message);
}

