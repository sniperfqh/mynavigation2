# nav2_msgs

## 当前项目用法

定义标准 Nav2 导航、规划、路径跟踪、平滑、行为及生命周期相关消息、服务和 Action。

包类型：接口包。包名和构建依赖见 [package.xml](package.xml)，安装目标见 [CMakeLists.txt](CMakeLists.txt)。

主要目录：[msg/](msg/)、[srv/](srv/)、[action/](action/)。

直接依赖（清单节选，完整约束见 manifest）：`action_msgs`、`builtin_interfaces`、`geometry_msgs`、`nav2_common`、`nav_msgs`、`rclcpp`、`rosidl_default_generators`、`std_msgs`。

### 接口文件

- [BehaviorTreeLog.msg](msg/BehaviorTreeLog.msg)：`nav2_msgs/msg/BehaviorTreeLog`。
- [BehaviorTreeStatusChange.msg](msg/BehaviorTreeStatusChange.msg)：`nav2_msgs/msg/BehaviorTreeStatusChange`。
- [CollisionMonitorState.msg](msg/CollisionMonitorState.msg)：`nav2_msgs/msg/CollisionMonitorState`。
- [Costmap.msg](msg/Costmap.msg)：`nav2_msgs/msg/Costmap`。
- [CostmapFilterInfo.msg](msg/CostmapFilterInfo.msg)：`nav2_msgs/msg/CostmapFilterInfo`。
- [CostmapMetaData.msg](msg/CostmapMetaData.msg)：`nav2_msgs/msg/CostmapMetaData`。
- [EdgeCost.msg](msg/EdgeCost.msg)：`nav2_msgs/msg/EdgeCost`。
- [Particle.msg](msg/Particle.msg)：`nav2_msgs/msg/Particle`。
- [ParticleCloud.msg](msg/ParticleCloud.msg)：`nav2_msgs/msg/ParticleCloud`。
- [Route.msg](msg/Route.msg)：`nav2_msgs/msg/Route`。
- [RouteEdge.msg](msg/RouteEdge.msg)：`nav2_msgs/msg/RouteEdge`。
- [RouteNode.msg](msg/RouteNode.msg)：`nav2_msgs/msg/RouteNode`。
- [SpeedLimit.msg](msg/SpeedLimit.msg)：`nav2_msgs/msg/SpeedLimit`。
- [VoxelGrid.msg](msg/VoxelGrid.msg)：`nav2_msgs/msg/VoxelGrid`。
- [ClearCostmapAroundRobot.srv](srv/ClearCostmapAroundRobot.srv)：`nav2_msgs/srv/ClearCostmapAroundRobot`。
- [ClearCostmapExceptRegion.srv](srv/ClearCostmapExceptRegion.srv)：`nav2_msgs/srv/ClearCostmapExceptRegion`。
- [ClearEntireCostmap.srv](srv/ClearEntireCostmap.srv)：`nav2_msgs/srv/ClearEntireCostmap`。
- [DynamicEdges.srv](srv/DynamicEdges.srv)：`nav2_msgs/srv/DynamicEdges`。
- [GetCostmap.srv](srv/GetCostmap.srv)：`nav2_msgs/srv/GetCostmap`。
- [IsPathValid.srv](srv/IsPathValid.srv)：`nav2_msgs/srv/IsPathValid`。
- [LoadMap.srv](srv/LoadMap.srv)：`nav2_msgs/srv/LoadMap`。
- [ManageLifecycleNodes.srv](srv/ManageLifecycleNodes.srv)：`nav2_msgs/srv/ManageLifecycleNodes`。
- [SaveMap.srv](srv/SaveMap.srv)：`nav2_msgs/srv/SaveMap`。
- [SetInitialPose.srv](srv/SetInitialPose.srv)：`nav2_msgs/srv/SetInitialPose`。
- [SetRouteGraph.srv](srv/SetRouteGraph.srv)：`nav2_msgs/srv/SetRouteGraph`。
- [AssistedTeleop.action](action/AssistedTeleop.action)：`nav2_msgs/action/AssistedTeleop`。
- [BackUp.action](action/BackUp.action)：`nav2_msgs/action/BackUp`。
- [ComputeAndTrackRoute.action](action/ComputeAndTrackRoute.action)：`nav2_msgs/action/ComputeAndTrackRoute`。
- [ComputePathThroughPoses.action](action/ComputePathThroughPoses.action)：`nav2_msgs/action/ComputePathThroughPoses`。
- [ComputePathToPose.action](action/ComputePathToPose.action)：`nav2_msgs/action/ComputePathToPose`。
- [ComputeRoute.action](action/ComputeRoute.action)：`nav2_msgs/action/ComputeRoute`。
- [DockRobot.action](action/DockRobot.action)：`nav2_msgs/action/DockRobot`。
- [DriveOnHeading.action](action/DriveOnHeading.action)：`nav2_msgs/action/DriveOnHeading`。
- [DummyBehavior.action](action/DummyBehavior.action)：`nav2_msgs/action/DummyBehavior`。
- [FollowPath.action](action/FollowPath.action)：`nav2_msgs/action/FollowPath`。
- [FollowWaypoints.action](action/FollowWaypoints.action)：`nav2_msgs/action/FollowWaypoints`。
- [NavigateThroughPoses.action](action/NavigateThroughPoses.action)：`nav2_msgs/action/NavigateThroughPoses`。
- [NavigateToPose.action](action/NavigateToPose.action)：`nav2_msgs/action/NavigateToPose`。
- [SmoothPath.action](action/SmoothPath.action)：`nav2_msgs/action/SmoothPath`。
- [Spin.action](action/Spin.action)：`nav2_msgs/action/Spin`。
- [UndockRobot.action](action/UndockRobot.action)：`nav2_msgs/action/UndockRobot`。
- [Wait.action](action/Wait.action)：`nav2_msgs/action/Wait`。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../README.md)；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。

## 原有说明与上游资料

The `nav2_msgs` package is a set of messages, services, and actions for the `Nav2` system. `Nav2` also makes use of the standard `nav_msgs`, where appropriate.

## 中文翻译

### nav2_msgs

nav2_msgs 包为 Nav2 系统定义消息、Service 和 Action 接口。Nav2 也会在适用场景下复用标准 nav_msgs 消息。该包只定义通信契约，具体 Action Server、Service Server 和算法实现位于其他 Nav2 包。
