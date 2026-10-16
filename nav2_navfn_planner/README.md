# Navfn Planner

## 当前项目用法

提供栅格 Dijkstra／A* 路径搜索，由 Planner Server 加载。

包类型：规划插件。包名和构建依赖见 [package.xml](package.xml)，安装目标见 [CMakeLists.txt](CMakeLists.txt)。

主要目录：[src/](src/)、[include/](include/)、[test/](test/)。

直接依赖（清单节选，完整约束见 manifest）：`builtin_interfaces`、`geometry_msgs`、`nav2_common`、`nav2_core`、`nav2_costmap_2d`、`nav2_msgs`、`nav2_util`、`nav_msgs`、`pluginlib`、`rclcpp`、`rclcpp_action`、`rclcpp_lifecycle` 等。

插件声明：[global_planner_plugin.xml](global_planner_plugin.xml)。插件由对应宿主加载，插件类名以声明 XML 为准。

使用方式：在宿主服务器的插件列表和参数块中配置导出的类；仅构建本包不会自动切换当前运行控制器或规划器。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../README.md)；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。

## 原有说明与上游资料

The NavfnPlanner is a global planner plugin for the Nav2 Planner server. It implements the Navigation Function planner with either A\* or Dij. expansions. It is largely equivalent to its counterpart in ROS 1 Navigation. The Navfn planner assumes a circular robot (or a robot that can be approximated as circular for the purposes of global path planning) and operates on a weighted costmap.

The `global_planner` package from ROS (1) is a refactor on NavFn to make it more easily understandable, but it lacks in run-time performance and introduces suboptimal behaviors. As NavFn has been extremely stable for about 10 years at the time of porting, the maintainers felt no compelling reason to port over another, largely equivalent (but poorer functioning) planner.

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-navfn.html) for additional parameter descriptions.

## 中文翻译

### Navfn Planner

Navfn Planner 是 Nav2 的传统二维栅格全局规划器，使用代价地图上的势场传播计算从起点到目标点的路径。它支持 Dijkstra 和 A* 风格的搜索选项，并通过 nav2_core::GlobalPlanner 插件接口加载到 Planner Server。参数说明请参阅原文 Configuration Guide。
