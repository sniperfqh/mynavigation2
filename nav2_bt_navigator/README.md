# BT Navigator

## 当前项目用法

执行行为树，将导航 Action 编排为规划、控制与恢复任务；与本项目无 BT 协调器是不同入口。

包类型：导航协调节点。包名和构建依赖见 [package.xml](package.xml)，安装目标见 [CMakeLists.txt](CMakeLists.txt)。

主要目录：[src/](src/)、[include/](include/)。

直接依赖（清单节选，完整约束见 manifest）：`behaviortree_cpp_v3`、`geometry_msgs`、`nav2_behavior_tree`、`nav2_common`、`nav2_core`、`nav2_msgs`、`nav2_util`、`nav_msgs`、`rclcpp`、`rclcpp_action`、`rclcpp_lifecycle`、`spdlog_wrapper` 等。

### 本项目接入说明

标准 BT 兼容入口使用本包；推荐 nav2_regulated_modules 入口由 regulated_navigator 编排任务，不运行本包的行为树。是否执行 SmoothPath 取决于选中的行为树，不能仅凭 smoother_server 已启动判断。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../README.md)；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。

## 原有说明与上游资料

The BT Navigator (Behavior Tree Navigator) module implements the NavigateToPose and NavigateThroughPoses task interfaces. It is a [Behavior Tree](https://github.com/BehaviorTree/BehaviorTree.CPP/blob/master/docs/BT_basics.md)-based implementation of navigation that is intended to allow for flexibility in the navigation task and provide a way to easily specify complex robot behaviors.

See its [Configuration Guide Page](https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html) for additional parameter descriptions, as well as the [Nav2 Behavior Tree Explanation](https://docs.nav2.org/behavior_trees/index.html) pages explaining more context on the default behavior trees and examples provided in this package.

## Overview

The BT Navigator receives a goal pose and navigates the robot to the specified destination(s). To do so, the module reads an XML description of the Behavior Tree from a file, as specified by a Node parameter, and passes that to a generic [BehaviorTreeEngine class](../nav2_behavior_tree/include/nav2_behavior_tree/behavior_tree_engine.hpp) which uses the [Behavior-Tree.CPP library](https://github.com/BehaviorTree/BehaviorTree.CPP) to dynamically create and execute the BT. The BT XML can also be specified on a per-task basis so that your robot may have many different types of navigation or autonomy behaviors on a per-task basis.

## 中文翻译

### BT Navigator

BT Navigator（行为树导航器）实现 NavigateToPose 和 NavigateThroughPoses 任务接口。它以 Behavior-Tree.CPP 为基础，通过行为树提供灵活的导航任务编排，使复杂机器人行为能够用 XML 描述。

BT Navigator 接收目标 Pose 并把机器人导航到指定位置。节点参数指定行为树 XML 文件，模块读取后交给通用 BehaviorTreeEngine；引擎使用 Behavior-Tree.CPP 动态创建并执行行为树。也可以针对每个任务单独指定 XML，使同一机器人拥有多种导航或自主行为。
