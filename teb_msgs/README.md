# teb_msgs

## 当前项目用法

定义 TEB 优化轨迹、反馈和障碍物消息，供 teb_local_planner 使用。

包类型：接口包。包名和构建依赖见 [package.xml](package.xml)，安装目标见 [CMakeLists.txt](CMakeLists.txt)。

主要目录：[msg/](msg/)。

直接依赖（清单节选，完整约束见 manifest）：`builtin_interfaces`、`geometry_msgs`、`rosidl_default_runtime`、`std_msgs`。

### 接口文件

- [FeedbackMsg.msg](msg/FeedbackMsg.msg)：`teb_msgs/msg/FeedbackMsg`。
- [TrajectoryMsg.msg](msg/TrajectoryMsg.msg)：`teb_msgs/msg/TrajectoryMsg`。
- [TrajectoryPointMsg.msg](msg/TrajectoryPointMsg.msg)：`teb_msgs/msg/TrajectoryPointMsg`。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../README.md)；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。
