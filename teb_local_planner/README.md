# teb_local_planner

## 当前项目用法

通过 Timed Elastic Band 优化轨迹时间、障碍距离与运动学约束，作为 Nav2 控制插件使用。

包类型：控制插件。包名和构建依赖见 [package.xml](package.xml)，安装目标见 [CMakeLists.txt](CMakeLists.txt)。

主要目录：[src/](src/)、[include/](include/)、[launch/](launch/)、[params/](params/)、[config/](config/)、[test/](test/)、[scripts/](scripts/)。

直接依赖（清单节选，完整约束见 manifest）：`builtin_interfaces`、`dwb_critics`、`geometry_msgs`、`libg2o`、`nav2_core`、`nav2_costmap_2d`、`nav2_msgs`、`nav2_util`、`pluginlib`、`rclcpp`、`rclcpp_action`、`rclcpp_lifecycle` 等。

插件声明：[teb_local_planner_plugin.xml](teb_local_planner_plugin.xml)。插件由对应宿主加载，插件类名以声明 XML 为准。

启动资源：[scout_mini_teb_relocalization.launch.py](launch/scout_mini_teb_relocalization.launch.py)、[sentry_teb_navigation.launch.py](launch/sentry_teb_navigation.launch.py)、[teb_scout_mini_relocalization.launch.py](launch/teb_scout_mini_relocalization.launch.py)、[teb_sentry_navigation.launch.py](launch/teb_sentry_navigation.launch.py)。执行前先加载工作空间环境，具体依赖由所选入口决定。

使用方式：在宿主服务器的插件列表和参数块中配置导出的类；仅构建本包不会自动切换当前运行控制器或规划器。

### 本项目接入说明

控制插件声明见 teb_local_planner_plugin.xml，构建依赖含 g2o。是否启用 TEB 以实际 Controller Server 配置为准，不因包存在就视为默认控制器。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../README.md)；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。
