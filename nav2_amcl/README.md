# AMCL

## 当前项目用法

使用粒子滤波、运动模型与激光地图匹配估计二维位姿，向导航提供定位变换。

包类型：定位节点。包名和构建依赖见 [package.xml](package.xml)，安装目标见 [CMakeLists.txt](CMakeLists.txt)。

主要目录：[src/](src/)、[include/](include/)、[test/](test/)。

直接依赖（清单节选，完整约束见 manifest）：`geometry_msgs`、`launch_ros`、`launch_testing`、`message_filters`、`nav2_common`、`nav2_msgs`、`nav2_util`、`nav_msgs`、`pluginlib`、`rclcpp`、`sensor_msgs`、`std_srvs` 等。

插件声明：[plugins.xml](plugins.xml)。插件由对应宿主加载，插件类名以声明 XML 为准。

### 本项目接入说明

当前 myworld_bringup/entry.launch.py 采用外部定位，不能因本包存在就认为 AMCL 已启动。其他兼容场景可单独包含定位 Launch；同一 TF 子坐标系应明确唯一的定位来源。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../README.md)；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。

## 原有说明与上游资料

Adaptive Monte Carlo Localization (AMCL) is a probabilistic localization module which estimates the position and orientation (i.e. Pose) of a robot in a given known map using a 2D laser scanner. This is largely a refactored port from ROS 1 without any algorithmic changes.

See the [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-amcl.html) for more details about configurable settings and their meanings.

## 中文翻译

### AMCL

自适应蒙特卡洛定位（AMCL）是概率定位模块。它使用二维激光扫描器，在已知地图中估计机器人位置和朝向，也就是机器人 Pose。该实现主要从 ROS 1 版本重构移植而来，算法本身没有改变。可配置参数及其含义请参阅原文 Configuration Guide。
