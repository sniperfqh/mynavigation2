# BT Navigator

The BT Navigator (Behavior Tree Navigator) module implements the NavigateToPose and NavigateThroughPoses task interfaces. It is a [Behavior Tree](https://github.com/BehaviorTree/BehaviorTree.CPP/blob/master/docs/BT_basics.md)-based implementation of navigation that is intended to allow for flexibility in the navigation task and provide a way to easily specify complex robot behaviors.

See its [Configuration Guide Page](https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html) for additional parameter descriptions, as well as the [Nav2 Behavior Tree Explanation](https://docs.nav2.org/behavior_trees/index.html) pages explaining more context on the default behavior trees and examples provided in this package.

## Overview

The BT Navigator receives a goal pose and navigates the robot to the specified destination(s). To do so, the module reads an XML description of the Behavior Tree from a file, as specified by a Node parameter, and passes that to a generic [BehaviorTreeEngine class](../nav2_behavior_tree/include/nav2_behavior_tree/behavior_tree_engine.hpp) which uses the [Behavior-Tree.CPP library](https://github.com/BehaviorTree/BehaviorTree.CPP) to dynamically create and execute the BT. The BT XML can also be specified on a per-task basis so that your robot may have many different types of navigation or autonomy behaviors on a per-task basis.

## 中文翻译

# BT Navigator

BT Navigator（行为树导航器）实现 NavigateToPose 和 NavigateThroughPoses 任务接口。它以 Behavior-Tree.CPP 为基础，通过行为树提供灵活的导航任务编排，使复杂机器人行为能够用 XML 描述。

BT Navigator 接收目标 Pose 并把机器人导航到指定位置。节点参数指定行为树 XML 文件，模块读取后交给通用 BehaviorTreeEngine；引擎使用 Behavior-Tree.CPP 动态创建并执行行为树。也可以针对每个任务单独指定 XML，使同一机器人拥有多种导航或自主行为。
