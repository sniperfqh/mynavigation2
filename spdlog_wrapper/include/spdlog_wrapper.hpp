#pragma once
#include <rcutils/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include <spdlog/async.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include "level_padded_flag.hpp"

#ifndef PACKAGE_NAME
#define PACKAGE_NAME "spdlog_wrapper"
#endif

struct SpdlogConfig
{
    std::string log_dir = "/home/byd/logs";
    size_t max_file_size_mb = 10 * 1024 * 1024;
    size_t max_files = 5;
    std::string console_level = "trace";
    std::string file_level = "trace";
    int flush_interval_seconds = 0;
    bool append_ros_timestamp = false;
    bool raw_ros_timestamp = false;
};

class SpdlogWrapper
{
public:

    static void init(const std::string &module,
                     const std::string &node_name);

    static void shutdown();

    static std::string short_file_name(const char *path);

private:
    static SpdlogConfig load_config_from_env();

    static SpdlogConfig load_from_yaml(const std::string &node_name);

    static std::string format_ros_time(rcutils_time_point_value_t timestamp);

    static spdlog::level::level_enum level_to_spdlog(int severity);

    static void output_handler(const rcutils_log_location_t *location,
                               int severity,
                               const char * /*name*/,
                               rcutils_time_point_value_t timestamp,
                               const char *format,
                               va_list *args);

    static inline std::shared_ptr<spdlog::async_logger> g_logger;
    static inline SpdlogConfig g_config = {};
    static inline spdlog::level::level_enum g_min_level;
};

#define LOG_TRACE(fmt, ...)                                \
    SPDLOG_TRACE("[{} {}:{}] " fmt,                        \
                 SpdlogWrapper::short_file_name(__FILE__), \
                 __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define LOG_DEBUG(fmt, ...)                                \
    SPDLOG_DEBUG("[{} {}:{}] " fmt,                        \
                 SpdlogWrapper::short_file_name(__FILE__), \
                 __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define LOG_INFO(fmt, ...)                                \
    SPDLOG_INFO("[{} {}:{}] " fmt,                        \
                SpdlogWrapper::short_file_name(__FILE__), \
                __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define LOG_WARN(fmt, ...)                                \
    SPDLOG_WARN("[{} {}:{}] " fmt,                        \
                SpdlogWrapper::short_file_name(__FILE__), \
                __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define LOG_ERROR(fmt, ...)                                \
    SPDLOG_ERROR("[{} {}:{}] " fmt,                        \
                 SpdlogWrapper::short_file_name(__FILE__), \
                 __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define LOG_CRITICAL(fmt, ...)                                \
    SPDLOG_CRITICAL("[{} {}:{}] " fmt,                        \
                    SpdlogWrapper::short_file_name(__FILE__), \
                    __FUNCTION__, __LINE__, ##__VA_ARGS__)
