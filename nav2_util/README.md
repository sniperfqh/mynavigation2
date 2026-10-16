# Nav2 Util

## 当前项目用法

提供生命周期节点、Action Server、TF、里程计、几何、速度及 collision 状态等公共工具。

包类型：公共运行库。包名和构建依赖见 [package.xml](package.xml)，安装目标见 [CMakeLists.txt](CMakeLists.txt)。

主要目录：[src/](src/)、[include/](include/)、[test/](test/)。

直接依赖（清单节选，完整约束见 manifest）：`action_msgs`、`bond`、`bondcpp`、`byd_custom_msgs`、`geometry_msgs`、`launch`、`launch_testing_ament_cmake`、`libboost-program-options`、`libboost-program-options-dev`、`lifecycle_msgs`、`nav2_common`、`nav2_msgs` 等。

使用方式：由依赖方通过构建依赖、头文件或 Python 导入使用，不提供通用的独立导航启动命令。

### 本项目接入说明

`include/nav2_util/collision_state.hpp` 保存原始／有效状态、接收标记及单调时钟超时信息。构造、configure、reset 后首次未收到状态默认为 4；首次收到后非法值或超时按 0。该工具不创建导航任务，也不独立产生速度。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../README.md)；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。

## 原有说明与上游资料

The `nav2_util` package contains utilities abstracted from individual packages which may find use in other uses. Some examples of things you'll find here:

- Geometry utilities for computing distances and values in paths
- A Nav2 specific lifecycle node wrapper for boilerplate code and useful common utilities like `declare_parameter_if_not_declared()`
- Simplified service clients
- Simplified action servers
- Transformation and robot pose helpers

The long-term aim is for these utilities to find more permanent homes in other packages (within and outside of Nav2) or migrate to the raw tools made available in ROS 2.

## 中文翻译

### Nav2 Util

nav2_util 包提供从各个 Nav2 包中抽取的通用工具，包括路径距离和几何计算、Lifecycle 节点封装、declare_parameter_if_not_declared、简化 Service Client、简化 Action Server，以及 TF 和机器人 Pose 辅助函数。

长期目标是把这些工具迁移到更合适的包（包括 Nav2 之外的包），或迁移到 ROS 2 原生提供的底层工具中。
