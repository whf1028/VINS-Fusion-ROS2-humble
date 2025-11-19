// Copyright (c) 2025
// VINS logger aligned with px4ctrl FlightLogger design

#pragma once

#include <string>
#include <fstream>
#include <map>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <cstdarg>
#include <chrono>
#include <utility>

namespace vins {

class VinsLogger {
public:
    enum class LogLevel {
        DEBUG = 0,
        INFO = 1,
        WARN = 2,
        ERROR = 3,
        FATAL = 4
    };

    enum class LogCategory {
        SYSTEM = 0,
        ESTIMATOR = 1,
        FEATURE_TRACKER = 2,
        GPU = 3,
        PERFORMANCE = 4,
        IMAGE_PROC = 5,
        IMU = 6,
        CALIBRATION = 7,
        VISUALIZATION = 8,
        ROS2 = 9,
        INITIALIZATION = 10,
        OPTIMIZATION = 11
    };

    static VinsLogger& getInstance();

    // Initialize with base dir and session id; returns true if ready (or disabled gracefully)
    bool initialize(const std::string& log_base_dir, const std::string& session_id);

    // Generic log (printf-style and std::string)
    void log(LogLevel level, LogCategory category, const char* format, ...);
    void log(LogLevel level, LogCategory category, const std::string& message);

    // Level helpers (printf-style)
    void debug(LogCategory category, const char* format, ...);
    void info(LogCategory category, const char* format, ...);
    void warn(LogCategory category, const char* format, ...);
    void error(LogCategory category, const char* format, ...);
    void fatal(LogCategory category, const char* format, ...);

    // Level helpers (std::string)
    void debug(LogCategory category, const std::string& message);
    void info(LogCategory category, const std::string& message);
    void warn(LogCategory category, const std::string& message);
    void error(LogCategory category, const std::string& message);
    void fatal(LogCategory category, const std::string& message);

    void setLogLevel(LogLevel level) { current_level_ = level; }
    bool isEnabled() const { return enabled_; }

    // 获取日志文件路径
    std::string getLogFilePath(LogCategory category) const;

    // Throttle and sampling helpers
    bool shouldLogThrottle(LogLevel level, LogCategory category, double period_sec);
    bool shouldLogEveryN(LogLevel level, LogCategory category, int n);

private:
    VinsLogger() = default;
    ~VinsLogger() = default;
    VinsLogger(const VinsLogger&) = delete;
    VinsLogger& operator=(const VinsLogger&) = delete;

    std::string timestamp();
    std::string levelToString(LogLevel level) const;
    std::string categoryToString(LogCategory category) const;
    std::string fileNameFor(LogCategory category) const;
    std::ofstream& streamFor(LogCategory category);
    void vlog(LogLevel level, LogCategory category, const char* format, va_list args);

    // 为 pair<LogCategory, LogLevel> 定义自定义哈希
    struct LogKeyHash {
        std::size_t operator()(const std::pair<LogCategory, LogLevel>& key) const noexcept {
            auto a = static_cast<std::size_t>(key.first);
            auto b = static_cast<std::size_t>(key.second);
            return (a << 4) ^ (b + 0x9e3779b97f4a7c15ULL + (a << 6) + (a >> 2));
        }
    };

private:
    bool enabled_ = true;
    LogLevel current_level_ = LogLevel::INFO;
    std::string base_dir_;
    std::string session_id_;
    std::map<LogCategory, std::unique_ptr<std::ofstream>> files_;
    std::mutex mutex_;

    // 节流与每N次计数
    std::unordered_map<std::pair<LogCategory, LogLevel>, std::chrono::steady_clock::time_point,
        LogKeyHash> last_log_times_;
    std::unordered_map<std::pair<LogCategory, LogLevel>, int, LogKeyHash> every_n_counters_;
};

// Convenience macros (category is enum token, e.g., GPU, SYSTEM)
#define VINS_LOG_DEBUG(category, ...) \
    ::vins::VinsLogger::getInstance().log(::vins::VinsLogger::LogLevel::DEBUG, ::vins::VinsLogger::LogCategory::category, __VA_ARGS__)

#define VINS_LOG_INFO(category, ...) \
    ::vins::VinsLogger::getInstance().log(::vins::VinsLogger::LogLevel::INFO, ::vins::VinsLogger::LogCategory::category, __VA_ARGS__)

#define VINS_LOG_WARN(category, ...) \
    ::vins::VinsLogger::getInstance().log(::vins::VinsLogger::LogLevel::WARN, ::vins::VinsLogger::LogCategory::category, __VA_ARGS__)

#define VINS_LOG_ERROR(category, ...) \
    ::vins::VinsLogger::getInstance().log(::vins::VinsLogger::LogLevel::ERROR, ::vins::VinsLogger::LogCategory::category, __VA_ARGS__)

#define VINS_LOG_FATAL(category, ...) \
    ::vins::VinsLogger::getInstance().log(::vins::VinsLogger::LogLevel::FATAL, ::vins::VinsLogger::LogCategory::category, __VA_ARGS__)

// Throttle macro: 在period_sec内仅打印一次
#define VINS_LOG_THROTTLE(level, category, period_sec, ...) \
    do { \
        auto &_logger = ::vins::VinsLogger::getInstance(); \
        if (_logger.shouldLogThrottle(::vins::VinsLogger::LogLevel::level, ::vins::VinsLogger::LogCategory::category, period_sec)) { \
            _logger.log(::vins::VinsLogger::LogLevel::level, ::vins::VinsLogger::LogCategory::category, __VA_ARGS__); \
        } \
    } while(0)

// Every-N macro: 仅每N次打印
#define VINS_LOG_EVERY_N(level, category, n, ...) \
    do { \
        auto &_logger = ::vins::VinsLogger::getInstance(); \
        if (_logger.shouldLogEveryN(::vins::VinsLogger::LogLevel::level, ::vins::VinsLogger::LogCategory::category, (n))) { \
            _logger.log(::vins::VinsLogger::LogLevel::level, ::vins::VinsLogger::LogCategory::category, __VA_ARGS__); \
        } \
    } while(0)

// Specialized helpers for GPU/performance (format messages at callsite)
#define VINS_LOG_GPU_OP(operation, duration_ms, success) \
    VINS_LOG_INFO(GPU, "GPU操作: %s, 耗时: %.2fms, 成功: %s", (operation), (duration_ms), (success) ? "true" : "false")

#define VINS_LOG_FEATURE_TRACKING(count, time_ms) \
    VINS_LOG_INFO(FEATURE_TRACKER, "特征跟踪: 数量=%d, 耗时=%.2fms", (int)(count), (double)(time_ms))

} // namespace vins
