// VINS logger aligned with px4ctrl FlightLogger

#include "logger.h"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>

namespace vins {

VinsLogger& VinsLogger::getInstance() {
    static VinsLogger inst;
    return inst;
}

bool VinsLogger::initialize(const std::string& log_base_dir, const std::string& session_id) {
    std::lock_guard<std::mutex> lk(mutex_);

    // 获取当前时间戳用于创建日期时间戳目录
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);

    // 创建日期时间戳目录名称，格式: YYYYMMDD_HHMMSS
    char timestamp_dir[32];
    std::snprintf(timestamp_dir, sizeof(timestamp_dir), "%04d%02d%02d_%02d%02d%02d",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                  tm.tm_hour, tm.tm_min, tm.tm_sec);

    // 构建完整路径: 基础目录/日期时间戳目录
    std::string base_path = log_base_dir.empty() ? std::string("/home/fsuav/fast-drone-250-humble-6.1/analyze_vins_log") : log_base_dir;
    base_dir_ = base_path + "/" + timestamp_dir;

    session_id_ = session_id;
    if (session_id_.empty()) {
        // Fallback to timestamp-based session id
        char buf[80];
        std::snprintf(buf, sizeof(buf), "session_%04d%02d%02d_%02d%02d%02d",
                      tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                      tm.tm_hour, tm.tm_min, tm.tm_sec);
        session_id_ = buf;
    }

    // Optional env to toggle
    if (const char* env_enabled = std::getenv("VINS_LOG_ENABLED")) {
        enabled_ = std::string(env_enabled) != "false";
    } else {
        enabled_ = true; // default on for VINS
    }

    if (const char* env_level = std::getenv("VINS_LOG_LEVEL")) {
        std::string lvl(env_level);
        if (lvl == "DEBUG") current_level_ = LogLevel::DEBUG;
        else if (lvl == "INFO") current_level_ = LogLevel::INFO;
        else if (lvl == "WARN") current_level_ = LogLevel::WARN;
        else if (lvl == "ERROR") current_level_ = LogLevel::ERROR;
        else if (lvl == "FATAL") current_level_ = LogLevel::FATAL;
    }

    // Create base directory only (simplified structure - no subdirectories)
    try {
        std::filesystem::create_directories(base_dir_);

        std::cout << "=== VINS Logger Initialized ===" << std::endl;
        std::cout << "Log directory: " << base_dir_ << std::endl;
        std::cout << "Session ID: " << session_id_ << std::endl;
        std::cout << "===============================" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Failed to create VINS log directory: " << e.what() << std::endl;
        return false;
    }

    return true;
}

std::string VinsLogger::timestamp() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << '.' << std::setw(3) << std::setfill('0') << ms.count();
    return oss.str();
}

std::string VinsLogger::levelToString(LogLevel level) const {
    switch (level) {
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO: return "INFO";
        case LogLevel::WARN: return "WARN";
        case LogLevel::ERROR: return "ERROR";
        case LogLevel::FATAL: return "FATAL";
    }
    return "INFO";
}

std::string VinsLogger::categoryToString(LogCategory c) const {
    switch (c) {
        case LogCategory::SYSTEM: return "system";
        case LogCategory::ESTIMATOR: return "estimator";
        case LogCategory::FEATURE_TRACKER: return "feature_tracker";
        case LogCategory::GPU: return "gpu";
        case LogCategory::PERFORMANCE: return "performance";
        case LogCategory::IMAGE_PROC: return "image_processing";
        case LogCategory::IMU: return "imu";
        case LogCategory::CALIBRATION: return "calibration";
        case LogCategory::VISUALIZATION: return "visualization";
        case LogCategory::ROS2: return "ros2";
        case LogCategory::INITIALIZATION: return "initialization";
        case LogCategory::OPTIMIZATION: return "optimization";
    }
    return "system";
}

std::string VinsLogger::fileNameFor(LogCategory category) const {
    // 简化结构: base_dir/session_category.log (无子目录)
    std::string cat_str = categoryToString(category);
    std::string filename = session_id_ + "_" + cat_str + ".log";
    return base_dir_ + "/" + filename;
}

std::string VinsLogger::getLogFilePath(LogCategory category) const {
    return fileNameFor(category);
}

std::ofstream& VinsLogger::streamFor(LogCategory category) {
    auto it = files_.find(category);
    if (it == files_.end() || !it->second || !(*it->second)) {
        std::string path = fileNameFor(category);
        auto ofs = std::unique_ptr<std::ofstream>(new std::ofstream(path, std::ios::app));
        files_[category] = std::move(ofs);
    }
    return *files_[category];
}

void VinsLogger::vlog(LogLevel level, LogCategory category, const char* format, va_list args) {
    if (!enabled_) return;
    if (static_cast<int>(level) < static_cast<int>(current_level_)) return;

    char buffer[2048];
    vsnprintf(buffer, sizeof(buffer), format, args);

    std::ostringstream line;
    line << '[' << timestamp() << "] [" << levelToString(level) << "] [" << categoryToString(category) << "] " << buffer << '\n';

    auto& ofs = streamFor(category);
    ofs << line.str();
    ofs.flush();
    if (level == LogLevel::ERROR || level == LogLevel::FATAL) {
        // Also echo errors to stderr
        fprintf(stderr, "%s", line.str().c_str());
    }
}

void VinsLogger::log(LogLevel level, LogCategory category, const char* format, ...) {
    std::lock_guard<std::mutex> lk(mutex_);
    va_list args;
    va_start(args, format);
    vlog(level, category, format, args);
    va_end(args);
}

void VinsLogger::log(LogLevel level, LogCategory category, const std::string& message) {
    if (!enabled_) return;
    if (static_cast<int>(level) < static_cast<int>(current_level_)) return;

    std::lock_guard<std::mutex> lk(mutex_);

    std::ostringstream line;
    line << '[' << timestamp() << "] [" << levelToString(level) << "] [" << categoryToString(category) << "] " << message << '\n';

    auto& ofs = streamFor(category);
    ofs << line.str();
    ofs.flush();
    if (level == LogLevel::ERROR || level == LogLevel::FATAL) {
        // Also echo errors to stderr
        fprintf(stderr, "%s", line.str().c_str());
    }
}

void VinsLogger::debug(LogCategory category, const char* format, ...) {
    std::lock_guard<std::mutex> lk(mutex_);
    va_list args; va_start(args, format); vlog(LogLevel::DEBUG, category, format, args); va_end(args);
}
void VinsLogger::info(LogCategory category, const char* format, ...) {
    std::lock_guard<std::mutex> lk(mutex_);
    va_list args; va_start(args, format); vlog(LogLevel::INFO, category, format, args); va_end(args);
}
void VinsLogger::warn(LogCategory category, const char* format, ...) {
    std::lock_guard<std::mutex> lk(mutex_);
    va_list args; va_start(args, format); vlog(LogLevel::WARN, category, format, args); va_end(args);
}
void VinsLogger::error(LogCategory category, const char* format, ...) {
    std::lock_guard<std::mutex> lk(mutex_);
    va_list args; va_start(args, format); vlog(LogLevel::ERROR, category, format, args); va_end(args);
}
void VinsLogger::fatal(LogCategory category, const char* format, ...) {
    std::lock_guard<std::mutex> lk(mutex_);
    va_list args; va_start(args, format); vlog(LogLevel::FATAL, category, format, args); va_end(args);
}

// std::string 版本的便捷方法
void VinsLogger::debug(LogCategory category, const std::string& message) {
    log(LogLevel::DEBUG, category, message);
}
void VinsLogger::info(LogCategory category, const std::string& message) {
    log(LogLevel::INFO, category, message);
}
void VinsLogger::warn(LogCategory category, const std::string& message) {
    log(LogLevel::WARN, category, message);
}
void VinsLogger::error(LogCategory category, const std::string& message) {
    log(LogLevel::ERROR, category, message);
}
void VinsLogger::fatal(LogCategory category, const std::string& message) {
    log(LogLevel::FATAL, category, message);
}

bool VinsLogger::shouldLogEveryN(LogLevel level, LogCategory category, int n) {
    if (!enabled_) return false;
    if (static_cast<int>(level) < static_cast<int>(current_level_)) return false;
    if (n <= 1) return true;

    auto key = std::make_pair(category, level);
    int& counter = every_n_counters_[key];
    counter++;
    if (counter >= n) {
        counter = 0;
        return true;
    }
    return false;
}

bool VinsLogger::shouldLogThrottle(LogLevel level, LogCategory category, double period_sec) {
    if (!enabled_) return false;
    if (static_cast<int>(level) < static_cast<int>(current_level_)) return false;

    auto key = std::make_pair(category, level);
    auto now = std::chrono::steady_clock::now();
    auto it = last_log_times_.find(key);

    if (it == last_log_times_.end()) {
        last_log_times_[key] = now;
        return true;
    }

    auto elapsed = std::chrono::duration<double>(now - it->second).count();
    if (elapsed >= period_sec) {
        it->second = now;
        return true;
    }

    return false;
}

} // namespace vins
