# System Tests

## 当前项目用法

包含导航系统级测试脚本、场景和模型；当前 COLCON_IGNORE 使其不参加默认 colcon 构建。

包类型：系统测试资源。包名和构建依赖见 [package.xml](package.xml)，安装目标见 [CMakeLists.txt](CMakeLists.txt)。

主要目录：[src/](src/)、[models/](models/)、[maps/](maps/)、[scripts/](scripts/)。

直接依赖（清单节选，完整约束见 manifest）：`gazebo_ros_pkgs`、`geometry_msgs`、`launch_ros`、`launch_testing`、`lcov`、`nav2_amcl`、`nav2_behavior_tree`、`nav2_bringup`、`nav2_common`、`nav2_lifecycle_manager`、`nav2_map_server`、`nav2_msgs` 等。

### 本项目接入说明

当前根目录存在 COLCON_IGNORE，默认 colcon list/build 不包含本包。README 的上游测试示例不代表本地已执行，也不在本轮文档任务中移除此标记。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../README.md)；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。

## 原有说明与上游资料

The package provides tests for Component-Testing, Subsystem-Testing, and Full-System integration. Its main goal is to provide a location for smoke and integration tests of the navigation system to ensure that things are working properly on a high level. Unit and specific subsystem testing happens in the packages specific to those algorithms.

Most tests in this package will spin up Gazebo instances of a robot in an environment to have the robot complete some task in the space while tracking a specific modules results.  Some examples include

- System tests of a robot in a sandbox environment trying navigate to pose, navigate through poses, and waypoint following navigation types
- Random planning of thousands of paths in a generated environment to ensure default planners are working properly
- Testing the system can be brought up and down on the lifecycle transitions successfully multiple times
- Testing that the keepout and speed restricted zones work in a practical environment without going into keepout zones and slowing in speed restricted areas
- Testing behaviors in a sandbox environment to ensure they trigger and complete collision checking properly
- Testing system failures are properly recorded and can be recovered from

This is primarily for use in Nav2 CI to establish a high degree of maintainer confidence when merging in large architectural changes to the Nav2 project. However, this is also useful to test installs of Nav2 locally or for additional information.

## 中文翻译

### 系统测试

本包提供 Nav2 的高层系统测试，用于验证导航栈在仿真环境中的集成行为，包括规划、控制、定位、恢复、航点和 Lifecycle 启停。它主要服务于 Nav2 CI，在合并大型架构变更时建立维护者信心，也可用于本地验证 Nav2 安装。
