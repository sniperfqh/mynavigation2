# nav2_dwb_controller

## 当前项目用法

聚合 DWB 控制器相关包的依赖，不提供独立运行节点。

包类型：元包。包名和构建依赖见 [package.xml](package.xml)，安装目标见 [CMakeLists.txt](CMakeLists.txt)。

直接依赖（清单节选，完整约束见 manifest）：`costmap_queue`、`dwb_core`、`dwb_critics`、`dwb_msgs`、`dwb_plugins`、`nav_2d_msgs`、`nav_2d_utils`。

使用方式：构建或安装本元包以收集依赖；实际节点由启动包或应用入口运行。

构建命令统一从 `nav2_ws` 执行，详见[仓库构建与包索引](../../README.md)；不在包源码目录运行 colcon。本文依据当前源码描述职责，包存在、可构建或包含测试文件均不代表完成运行验收。
