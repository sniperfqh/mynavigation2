# nav_2d_msgs

## 当前项目用法

提供二维位姿、路径、速度等公共消息类型，供 DWB 系列使用。

包类型：接口包。包名和构建依赖见 [package.xml](package.xml)，安装目标见 [CMakeLists.txt](CMakeLists.txt)。

主要目录：[msg/](msg/)。

直接依赖（清单节选，完整约束见 manifest）：`geometry_msgs`、`rosidl_default_generators`、`std_msgs`。

### 接口文件

- [Path2D.msg](msg/Path2D.msg)：`nav_2d_msgs/msg/Path2D`。
- [Pose2D32.msg](msg/Pose2D32.msg)：`nav_2d_msgs/msg/Pose2D32`。
- [Pose2DStamped.msg](msg/Pose2DStamped.msg)：`nav_2d_msgs/msg/Pose2DStamped`。
- [Twist2D.msg](msg/Twist2D.msg)：`nav_2d_msgs/msg/Twist2D`。
- [Twist2D32.msg](msg/Twist2D32.msg)：`nav_2d_msgs/msg/Twist2D32`。
- [Twist2DStamped.msg](msg/Twist2DStamped.msg)：`nav_2d_msgs/msg/Twist2DStamped`。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../../README.md)；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。
