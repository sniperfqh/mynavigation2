# navigation2

## 当前项目用法

聚合 Nav2 栈依赖；源码仓库名称 navigation2 与本目录中的同名 ROS 元包应区分。

包类型：元包。包名和构建依赖见 [package.xml](package.xml)，安装目标见 [CMakeLists.txt](CMakeLists.txt)。

直接依赖（清单节选，完整约束见 manifest）：`nav2_amcl`、`nav2_behavior_tree`、`nav2_behaviors`、`nav2_bt_navigator`、`nav2_collision_monitor`、`nav2_constrained_smoother`、`nav2_controller`、`nav2_core`、`nav2_costmap_2d`、`nav2_dwb_controller`、`nav2_lifecycle_manager`、`nav2_map_server` 等。

使用方式：构建或安装本元包以收集依赖；实际节点由启动包或应用入口运行。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../README.md)；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。
