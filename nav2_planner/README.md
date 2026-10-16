# Nav2 Planner

## 当前项目用法

实现 ComputePathToPose／ComputePathThroughPoses Action，并管理全局规划插件与全局代价地图。

包类型：规划服务器。包名和构建依赖见 [package.xml](package.xml)，安装目标见 [CMakeLists.txt](CMakeLists.txt)。

主要目录：[src/](src/)、[include/](include/)、[test/](test/)。

直接依赖（清单节选，完整约束见 manifest）：`builtin_interfaces`、`geometry_msgs`、`nav2_common`、`nav2_core`、`nav2_costmap_2d`、`nav2_msgs`、`nav2_util`、`nav_msgs`、`pluginlib`、`rclcpp`、`rclcpp_action`、`rclcpp_lifecycle` 等。

### 本项目接入说明

规划完成返回 Path，不直接发布底盘速度；规控自主模式通过 Action 调用本服务器，固定路径 NavigationService 分支直接生成 Path。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../README.md)；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。

## 原有说明与上游资料

The Nav2 planner is a Task Server in Nav2 that implements the `nav2_behavior_tree::ComputePathToPose` interface.

A planning module implementing the `nav2_behavior_tree::ComputePathToPose` interface is responsible for generating a feasible path given start and end robot poses. It loads a map of potential planner plugins to do the path generation in different user-defined situations.

See the [Navigation Plugin list](https://navigation.ros.org/plugins/index.html) for a list of the currently known and available planner plugins.

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-planner-server.html) for additional parameter descriptions and a [tutorial about writing planner plugins](https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html).

## 中文翻译

### Nav2 Planner

Nav2 Planner 是全局规划任务服务器，实现路径规划 Action 接口。它负责接收目标 Pose 或 Pose 序列、获取机器人起点、处理 TF 和 Costmap、选择 Global Planner 插件、校验返回路径并发布规划结果。

规划器实现 nav2_core::GlobalPlanner 的 createPlan() 接口。一个 Planner Server 可以同时加载多个规划器，通过插件 ID 选择实际算法。参数说明和新建 Planner 插件教程见原文链接。
