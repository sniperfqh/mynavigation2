# Nav2 Controller

## 当前项目用法

实现 FollowPath Action，加载控制器、进度检查器和目标检查器，计算并发布速度。

包类型：控制服务器。包名和构建依赖见 [package.xml](package.xml)，安装目标见 [CMakeLists.txt](CMakeLists.txt)。

主要目录：[src/](src/)、[include/](include/)、[plugins/](plugins/)、[test/](test/)。

直接依赖（清单节选，完整约束见 manifest）：`angles`、`nav2_common`、`nav2_core`、`nav2_msgs`、`nav2_util`、`nav_2d_msgs`、`nav_2d_utils`、`pluginlib`、`rclcpp`、`rclcpp_action`、`spdlog_wrapper`、`std_msgs`。

插件声明：[plugins.xml](plugins.xml)。插件由对应宿主加载，插件类名以声明 XML 为准。

### 本项目接入说明

本项目固定路径使用 `FixedPathController` 与 `fixed_path_goal_checker`；其实现位于 `nav2_regulated_modules`。固定路径 collision 停止期间暂停无进展计时，恢复后沿用原路径。`cmd_vel` 的最终名称取决于 Launch remapping，规控入口重映射到 `/cmd_vel_nav`。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../README.md)；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。

## 原有说明与上游资料

The Nav2 Controller is a Task Server in Nav2 that implements the `nav2_msgs::action::FollowPath` action server.

An execution module implementing the `nav2_msgs::action::FollowPath` action server is responsible for generating command velocities for the robot, given the computed path from the planner module in `nav2_planner`. The nav2_controller package is designed to be loaded with multiple plugins for path execution. The plugins need to implement functions in the virtual base class defined in the `controller` header file in `nav2_core` package. It also contains progress checkers and goal checker plugins to abstract out that logic from specific controller implementations.

See the [Navigation Plugin list](https://navigation.ros.org/plugins/index.html) for a list of the currently known and available controller plugins.

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-controller-server.html) for additional parameter descriptions and a [tutorial about writing controller plugins](https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2controller_plugin.html).

## 中文翻译

### Nav2 Controller

Nav2 Controller 是 Nav2 中实现 nav2_msgs::action::FollowPath Action Server 的任务服务器。它接收 Planner 生成的路径，调用配置的控制器插件计算机器人速度，并负责控制周期、里程计、进度检查、目标检查和最终速度发布。

插件必须实现 nav2_core 中 controller 头文件定义的虚基类。一个 Controller Server 可以加载多个路径执行插件，并通过 Controller ID 选择。Progress Checker 和 Goal Checker 也以插件形式提供，使卡住判定和到达判定与具体控制器解耦。
