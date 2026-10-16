# nav2_rviz_plugins

## 当前项目用法

提供导航目标工具、Nav2 面板与粒子云显示，供 RViz 加载。

包类型：RViz 插件。包名和构建依赖见 [package.xml](package.xml)，安装目标见 [CMakeLists.txt](CMakeLists.txt)。

主要目录：[src/](src/)、[include/](include/)。

直接依赖（清单节选，完整约束见 manifest）：`geometry_msgs`、`libqt5-core`、`libqt5-gui`、`libqt5-opengl`、`libqt5-widgets`、`nav2_lifecycle_manager`、`nav2_msgs`、`nav2_util`、`nav_msgs`、`pluginlib`、`qtbase5-dev`、`rclcpp` 等。

插件声明：[plugins_description.xml](plugins_description.xml)。插件由对应宿主加载，插件类名以声明 XML 为准。

### 本项目接入说明

在 RViz 中装载 `nav2_rviz_plugins/GoalTool`、`nav2_rviz_plugins/Navigation 2` 或 `nav2_rviz_plugins/ParticleCloud`。目标工具发送导航请求，不能替代外部定位或生命周期激活。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../README.md)；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。
