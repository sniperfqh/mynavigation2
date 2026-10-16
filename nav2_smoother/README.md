# Nav2 Smoother

## 当前项目用法

实现 SmoothPath Action，加载简单平滑与 Savitzky–Golay 等路径平滑插件。

包类型：平滑服务器与插件。包名和构建依赖见 [package.xml](package.xml)，安装目标见 [CMakeLists.txt](CMakeLists.txt)。

主要目录：[src/](src/)、[include/](include/)、[test/](test/)。

直接依赖（清单节选，完整约束见 manifest）：`angles`、`nav2_common`、`nav2_core`、`nav2_msgs`、`nav2_util`、`nav_2d_msgs`、`nav_2d_utils`、`pluginlib`、`rclcpp`、`rclcpp_action`、`rclcpp_components`、`spdlog_wrapper` 等。

插件声明：[plugins.xml](plugins.xml)。插件由对应宿主加载，插件类名以声明 XML 为准。

使用方式：在宿主服务器的插件列表和参数块中配置导出的类；仅构建本包不会自动切换当前运行控制器或规划器。

### 本项目接入说明

路径几何平滑与 nav2_velocity_smoother 的速度时间平滑是两项功能；启动服务器不代表每个入口一定调用 SmoothPath。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../README.md)；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。

## 原有说明与上游资料

The Nav2 smoother is a Task Server in Nav2 that implements the `nav2_behavior_tree::SmoothPath` interface.

A smoothing module implementing the `nav2_behavior_tree::SmoothPath` interface is responsible for improving path smoothness and/or quality, typically given an unsmoothed path from the planner module in `nav2_planner`. It loads a map of potential smoother plugins to do the path smoothing in different user-defined situations.

See the [Navigation Plugin list](https://navigation.ros.org/plugins/index.html) for a list of the currently known and available smoother plugins.

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-smoother-server.html) for additional parameter descriptions.

This package contains the Simple Smoother and Savitzky-Golay Smoother plugins.

## 中文翻译

### Nav2 Smoother

nav2_smoother 是 Nav2 的路径平滑任务服务器，提供 SmoothPath Action，并通过 Pluginlib 加载 Simple Smoother 和 Savitzky-Golay Smoother 插件。服务器把输入 Path 和时间预算传给插件，发布平滑路径，并可在返回前执行碰撞检查。
