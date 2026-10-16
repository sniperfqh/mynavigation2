# spdlog_wrapper

## 当前项目用法

封装 spdlog、YAML 配置与 ROS 日志转接，供规控与底盘适配等模块复用。

包类型：日志库。包名和构建依赖见 [package.xml](package.xml)，安装目标见 [CMakeLists.txt](CMakeLists.txt)。

主要目录：[src/](src/)、[include/](include/)、[config/](config/)。

直接依赖（清单节选，完整约束见 manifest）：`rclcpp`、`rcutils`、`spdlog`、`yaml-cpp`。

使用方式：由 C++ 依赖方链接本库，包含 [spdlog_wrapper.hpp](include/spdlog_wrapper.hpp)，通过 `SpdlogWrapper::init` 初始化，使用 `LOG_INFO` 等宏记录并在退出时调用 `SpdlogWrapper::shutdown`。不提供独立节点或 Python 客户端。

### 本项目接入说明

配置见 [config/log.yaml](config/log.yaml)；可用 `SPDLOG_WRAPPER_LOG_DIR`、`SPDLOG_WRAPPER_MAX_FILE_SIZE_MB`、`SPDLOG_WRAPPER_MAX_FILES`、`SPDLOG_WRAPPER_CONSOLE_LEVEL`、`SPDLOG_WRAPPER_FILE_LEVEL` 和 `SPDLOG_WRAPPER_FLUSH_INTERVAL_SECONDS` 覆盖。统一速度诊断是规控包中的独立节点，不等于所有 spdlog 输出逐消息归档；诊断格式见[速度日志](../nav2_regulated_modules/doc/velocity_logging.md)。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../README.md)；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。
